#!/usr/bin/env python3
"""GUI front-end for Unlooper.py — runs the existing CLI tool as a subprocess
and shows live progress, results, and the rendered toolpath image.

Unlooper.py is run exactly as documented in the README:
`python Unlooper.py "<file>" <0|1> ...` (the processing stages live in
unlooper_core/). This GUI launches it as a child process and parses its console
output; the only thing it imports from unlooper_core is the shared colour palette.
"""

import json
import math
import os
import re
import sys
from pathlib import Path

import cv2
import numpy as np
from PIL import Image
from PySide6.QtCore import Qt, QProcess, QProcessEnvironment, Slot, Signal, QSize, QRectF, QLineF, QSettings
from PySide6.QtGui import QImage, QPixmap, QPainter, QPainterPath, QPen, QColor, QTextCursor
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QPushButton, QCheckBox, QFileDialog, QProgressBar, QPlainTextEdit,
    QGroupBox, QSplitter, QDoubleSpinBox, QListWidget, QGraphicsView,
    QGraphicsScene, QComboBox, QGraphicsPathItem,
)
from PySide6.QtSvgWidgets import QGraphicsSvgItem

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))
from unlooper_core.palette import SPEED_COLOURS, ACCEL_COLOURS as ACCEL_RGB  # noqa: E402
UNLOOPER_SCRIPT = SCRIPT_DIR / "Unlooper.py"

TQDM_RE = re.compile(r"^\s*(\d+)%\|")
# "Scaffold outputs progress: 42%", "Pixel coords progress: 42%" from Unlooper.py
STAGE_PROGRESS_RE = re.compile(r"^(.+) progress:\s*(\d+)%$")
DISTANCE_RE = re.compile(r"Distance travelled:\s*([\d.]+)\s*m")
TIME_RE = re.compile(r"Total Time:\s*(.+)")
MATERIAL_RE = re.compile(r"Material Used:\s*([\d.]+)\s*mg")
SIZE_RE = re.compile(r"Total size used x:\s*([\-\d.]+)\s*y:\s*([\-\d.]+)")
TIME_ACCEL_RE = re.compile(r"Total Time \(accel/junction\):\s*(.+)")
AVG_SPEED_RE = re.compile(r"Average speed \(accel/junction\):\s*([\d.]+)\s*mm/min")
BELOW_CTS_RE = re.compile(r"Path below CTS:\s*(.+)")

# Speed and jet-lag views: 32 bands of PrusaSlicer's colours (dark blue = slowest ... dark red
# = fastest), the same palette Unlooper.py draws its PNGs with
# Acceleration view: sign of the acceleration along the path
ACCEL_COLOURS = {-1: (ACCEL_RGB[0], "Decelerating"), 0: (ACCEL_RGB[1], "Constant speed"), 1: (ACCEL_RGB[2], "Accelerating")}
DIMMED = (225, 225, 225)  # everything outside the highlighted band
NOZZLE_UNDERLAY = (200, 200, 200)  # nozzle path under the jet path in the lag view

# Defaults for the GUI settings (remembered between sessions with QSettings)
DEFAULT_ACCELERATION = 1000.0  # mm/s^2
DEFAULT_JUNCTION_DEVIATION = 0.013  # mm, Marlin 2's default
DEFAULT_JERK = 5.0  # mm/s

# Number of toolpath segments per scene item. Big enough to keep the item count low on
# large files, small enough that rebuilding the chunk under the cursor is instant.
PREVIEW_CHUNK = 2000


def build_toolpath_paths(segments):
    # Build one QPainterPath per move type (1 = G0/G1, 2 = G2, 3 = G3) from rows of
    # (kind, x1, y1, x2, y2, cx, cy, sweep_deg, line) as written by Unlooper.py
    paths = {}
    last_end = {}
    for row in segments.tolist():
        kind, x1, y1, x2, y2, cx, cy, sweep = row[:8]
        kind = int(kind)
        path = paths.get(kind)
        if path is None:
            path = paths[kind] = QPainterPath()
        if last_end.get(kind) != (x1, y1):
            path.moveTo(x1, y1)
        if kind == 1:
            path.lineTo(x2, y2)
        else:
            r = math.hypot(x1 - cx, y1 - cy)
            start_angle = math.degrees(math.atan2(cy - y1, x1 - cx))
            path.arcTo(cx - r, cy - r, 2 * r, 2 * r, start_angle, sweep)
        last_end[kind] = (x2, y2)
    return paths


def is_code_line(line):
    # Mirrors how Unlooper.py filters lines (remove_comments / is_macro_line), so that an
    # editor line can be matched to the line index the toolpath segments were recorded
    # against. Comment-only lines (the ; header block, % parameters, #, M117) and blank
    # lines are not counted; square-bracket macro lines are, as Unlooper.py keeps them.
    stripped = line.strip()
    if not stripped:
        return False
    if stripped.startswith("[") or ("[" in stripped and "]" in stripped):
        return True
    if "%" in stripped or ";" in stripped:
        return stripped.split("%", 1)[0].split(";", 1)[0].strip() != ""
    if "#" in stripped or stripped.upper().startswith("M117"):
        return False
    return True


def build_sample_paths(samples, bin_of):
    # Polyline through the reduced pixel coords (x, y, speed, accel, line) from Unlooper.py,
    # one QPainterPath per colour bin. The piece from point i to point i+1 takes point i's
    # colour - Unlooper.py keeps a point wherever the colour changes, so this is exact.
    paths = {}
    rows = samples.tolist()
    last_bin = None
    for i in range(len(rows) - 1):
        x, y, speed, accel = rows[i][:4]
        b = bin_of(speed, accel)
        path = paths.get(b)
        if path is None:
            path = paths[b] = QPainterPath()
        if b != last_bin or path.currentPosition().x() != x or path.currentPosition().y() != y:
            path.moveTo(x, y)
        path.lineTo(rows[i + 1][0], rows[i + 1][1])
        last_bin = b
    return paths


class SpeedLegend(QWidget):
    # Colour bar for the speed (mm/min) and jet-lag (mm) views: one swatch per band. Hover a
    # swatch to see the range it stands for; click it to highlight that band in the preview
    # (click again, or right-click, to clear). Also shows the discrete acceleration legend.
    highlightChanged = Signal(object)  # band index or None

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedHeight(40)
        self.setMouseTracking(True)
        self.edges = None  # band edges in display units
        self.unit = ""
        self.decimals = 1
        self.marker = None  # value to tick (the CTS), display units
        self.entries = None
        self.message = ""
        self.hover = None
        self.highlight = None

    def set_bands(self, edges, unit, decimals=1, marker=None):
        self.edges = list(edges)
        self.unit, self.decimals, self.marker = unit, decimals, marker
        self.entries = None
        self.message = ""
        if self.highlight is not None and self.highlight >= len(self.edges) - 1:
            self.highlight = None
        self.update()

    def set_entries(self, entries):
        # Discrete legend: list of (rgb, label)
        self.edges = None
        self.entries = entries
        self.message = ""
        self.update()

    def set_message(self, message):
        self.edges = None
        self.entries = None
        self.message = message
        self.update()

    def clear_highlight(self):
        if self.highlight is not None:
            self.highlight = None
            self.update()
            self.highlightChanged.emit(None)

    def _bar(self):
        return QRectF(8, 2, self.width() - 16, 14)

    def _band_at(self, x):
        if not self.edges:
            return None
        bar = self._bar()
        bands = len(self.edges) - 1
        if not bar.left() <= x <= bar.right():
            return None
        return min(int((x - bar.left()) / bar.width() * bands), bands - 1)

    def _range_text(self, band):
        low, high = self.edges[band], self.edges[band + 1]
        return f"band {band + 1}/{len(self.edges) - 1}: {low:.{self.decimals}f} - {high:.{self.decimals}f} {self.unit}"

    def mouseMoveEvent(self, event):
        band = self._band_at(event.position().x())
        if band != self.hover:
            self.hover = band
            self.setToolTip(self._range_text(band) + "\nClick to highlight" if band is not None else "")
            self.update()

    def leaveEvent(self, event):
        self.hover = None
        self.update()

    def mousePressEvent(self, event):
        if not self.edges:
            return
        band = None if event.button() == Qt.MouseButton.RightButton else self._band_at(event.position().x())
        self.highlight = None if band == self.highlight else band
        self.update()
        self.highlightChanged.emit(self.highlight)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setPen(self.palette().windowText().color())
        if self.entries:
            width = (self.width() - 16) / len(self.entries)
            for i, (colour, label) in enumerate(self.entries):
                left = 8 + i * width
                painter.fillRect(QRectF(left, 2, width - 6, 14), QColor(*colour))
                painter.drawText(QRectF(left, 18, width - 6, 18), Qt.AlignmentFlag.AlignHCenter, label)
            return
        if not self.edges:
            painter.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter, self.message)
            return
        bar = self._bar()
        bands = len(self.edges) - 1
        step = bar.width() / bands
        for i in range(bands):
            colour = SPEED_COLOURS[i] if self.highlight in (None, i) else DIMMED
            painter.fillRect(QRectF(bar.left() + step * i, bar.top(), step + 1, bar.height()), QColor(*colour))
        for band, pen in ((self.hover, QPen(QColor("black"), 1)), (self.highlight, QPen(QColor("black"), 2))):
            if band is not None:
                painter.setPen(pen)
                painter.drawRect(QRectF(bar.left() + step * band, bar.top(), step, bar.height()))
        painter.setPen(self.palette().windowText().color())
        text_rect = QRectF(bar.left(), bar.bottom() + 2, bar.width(), 18)
        shown = self.hover if self.hover is not None else self.highlight
        if shown is not None:
            painter.drawText(text_rect, Qt.AlignmentFlag.AlignHCenter, self._range_text(shown))
        else:
            # Labels at every 8th band edge
            for i in range(0, bands + 1, 8):
                x = bar.left() + step * i
                align = Qt.AlignmentFlag.AlignLeft if i == 0 else Qt.AlignmentFlag.AlignRight if i == bands else Qt.AlignmentFlag.AlignHCenter
                rect = QRectF(x, text_rect.top(), 0, 18).adjusted(-60, 0, 60, 0)
                if i == 0:
                    rect = QRectF(x, text_rect.top(), 120, 18)
                elif i == bands:
                    rect = QRectF(x - 120, text_rect.top(), 120, 18)
                label = f"{self.edges[i]:.{self.decimals}f}" + (f" {self.unit}" if i in (0, bands) else "")
                painter.drawText(rect, align, label)
        if self.marker is not None and self.edges[-1] > self.edges[0]:
            x = bar.left() + bar.width() * min(max((self.marker - self.edges[0]) / (self.edges[-1] - self.edges[0]), 0.0), 1.0)
            painter.setPen(QPen(QColor("black"), 2))
            painter.drawLine(QLineF(x, bar.top() - 2, x, bar.bottom() + 2))


class ToolpathView(QGraphicsView):

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setTransformationAnchor(QGraphicsView.ViewportAnchor.AnchorUnderMouse)
        # Build-plate grid, set when a vector preview is loaded (None = plain background)
        self.grid_rect = None
        self.grid_spacing = 1000.0
        self.grid_pen = QPen(QColor(220, 220, 220), 0)
        self.plate_brush = QColor("white")
        self.outside_brush = QColor(90, 90, 90)

    def drawBackground(self, painter, rect):
        if self.grid_rect is None:
            super().drawBackground(painter, rect)
            return
        # Same look as the PNG: white build plate with light grey lines every 1 mm,
        # and a darker surround outside the plate
        painter.fillRect(rect, self.outside_brush)
        plate = self.grid_rect
        painter.fillRect(plate, self.plate_brush)
        area = rect.intersected(plate)
        if area.isEmpty():
            return
        # Thin the grid out when lines would be packed closer than 4 px
        spacing = self.grid_spacing
        while spacing * self.transform().m11() < 4:
            spacing *= 10
        # No antialiasing so each line lands on a single pixel row/column like the PNG's
        # grid, rather than being smeared faintly across two
        painter.save()
        painter.setRenderHint(QPainter.RenderHint.Antialiasing, False)
        painter.setPen(self.grid_pen)
        x = math.ceil(area.left() / spacing) * spacing
        while x <= area.right():
            painter.drawLine(QLineF(x, area.top(), x, area.bottom()))
            x += spacing
        y = math.ceil(area.top() / spacing) * spacing
        while y <= area.bottom():
            painter.drawLine(QLineF(area.left(), y, area.right(), y))
            y += spacing
        painter.restore()

    def wheelEvent(self, event):
        factor = 1.2 if event.angleDelta().y() > 0 else 1 / 1.2
        self.scale(factor, factor)
        event.accept()


class UnlooperWindow(QMainWindow):

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Unlooper")
        self.resize(1200, 800)

        self.input_files = []
        self._active_file = None
        self._queue_index = -1
        self._batch_cancelled = False
        self.output_base_dir = SCRIPT_DIR
        self.process = None
        self._out_buffer = ""
        self._run_completed = False
        self._run_render_mode = "preview"
        self._pending_place = None  # Editor/view position to return to after a replot
        self._clear_preview_state()

        self._build_ui()
        self._load_settings()
        self._update_run_enabled()

    # ── UI construction ──────────────────────────────────────────────────

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(6)

        # Top bar: file picker
        top = QHBoxLayout()
        self.open_btn = QPushButton("Open G-code Files…")
        self.open_btn.clicked.connect(self._open_files)
        top.addWidget(self.open_btn)
        self.clear_files_btn = QPushButton("Clear")
        self.clear_files_btn.setEnabled(False)
        self.clear_files_btn.clicked.connect(self._clear_files)
        top.addWidget(self.clear_files_btn)
        self.file_label = QLabel("No files selected")
        top.addWidget(self.file_label, stretch=1)
        root.addLayout(top)

        splitter = QSplitter(Qt.Orientation.Horizontal)

        # Left: options + results
        left = QWidget()
        left_layout = QVBoxLayout(left)
        left_layout.setContentsMargins(4, 4, 4, 4)

        queue_group = QGroupBox("Files Queue")
        queue_layout = QVBoxLayout(queue_group)
        self.queue_list = QListWidget()
        self.queue_list.setMaximumHeight(120)
        queue_layout.addWidget(self.queue_list)
        left_layout.addWidget(queue_group)

        opts_group = QGroupBox("Options")
        opts_layout = QVBoxLayout(opts_group)
        self.unloop_only_check = QCheckBox("Unloop only (skip image render + timing/material calc)")
        self.unloop_only_check.setToolTip(
            "Matches the second command-line argument to Unlooper.py.\n"
            "Off = full run (unloop + analyze + render image)."
        )
        opts_layout.addWidget(self.unloop_only_check)
        self.motion_only_check = QCheckBox("Motion calcs only (no plotting)")
        self.motion_only_check.setToolTip(
            "Calculates distance, time, material and size but skips all plotting.\n"
            "Sends render mode 'none' to Unlooper.py."
        )
        opts_layout.addWidget(self.motion_only_check)
        self.skip_pixel_check = QCheckBox("Skip pixel coords (faster, no speed/acceleration colouring)")
        self.skip_pixel_check.setToolTip(
            "Pixel coords place a point every 1 ms along the planned motion, giving the actual\n"
            "speed and acceleration colouring (and the lag-model pixel coords file).\n"
            "Skipping them saves a lot of time on long prints; the acceleration-aware time is still given."
        )
        opts_layout.addWidget(self.skip_pixel_check)
        self.lag_check = QCheckBox("Lag prediction (jet path)")
        self.lag_check.setToolTip(
            "Runs the jet lag model (Ievgenii's python_lag, ported) on the pixel coords of the\n"
            "corner-rounded path: where the jet lands as the nozzle speeds up, slows down and\n"
            "rounds corners. Adds the 'jet lag' colouring. Needs Acceleration > 0.\n"
            "Uses the file's CriticalTranslationSpeed unless the CTS override is set."
        )
        opts_layout.addWidget(self.lag_check)
        self.write_files_check = QCheckBox("Write lag-format files (large CSVs)")
        self.write_files_check.setToolTip(
            "Writes every pixel coord in Gcode_processing.py's layout for the lag model:\n"
            "  _pixel_cords.csv / _pixel_coords_motion.csv        sharp corners, as programmed\n"
            "  _corner_pixel_cords.csv / _corner_pixel_coords_motion.csv   corners rounded as run\n"
            "  _lag.csv                                            jet contact points (with lag prediction)\n"
            "Hundreds of MB each for long prints."
        )
        opts_layout.addWidget(self.write_files_check)
        render_row = QHBoxLayout()
        render_row.addWidget(QLabel("Render type:"))
        self.render_mode_combo = QComboBox()
        self.render_mode_combo.addItem("Vector preview (SVG)", "preview")
        self.render_mode_combo.addItem("Raster image (PNG)", "precise")
        self.render_mode_combo.addItem("Both SVG and PNG", "both")
        self.render_mode_combo.setToolTip("Choose which render output is generated on the next run.")
        render_row.addWidget(self.render_mode_combo, stretch=1)
        opts_layout.addLayout(render_row)
        # Motion-only overrides the render type, and unloop-only skips the motion calcs entirely
        self.motion_only_check.toggled.connect(self._update_option_states)
        self.unloop_only_check.toggled.connect(self._update_option_states)
        left_layout.addWidget(opts_group)

        overrides_group = QGroupBox("Overrides (leave at 0 to use the file's own values)")
        overrides_grid = QGridLayout(overrides_group)
        overrides_grid.addWidget(QLabel("Feed rate (mm/min):"), 0, 0)
        self.feedrate_spin = QDoubleSpinBox()
        self.feedrate_spin.setRange(0, 1_000_000)
        self.feedrate_spin.setDecimals(2)
        self.feedrate_spin.setSingleStep(10)
        self.feedrate_spin.setToolTip(
            "If > 0, dictates the estimated time directly:\n"
            "time = total distance / feed rate.\n"
            "Ignores the feed rates written in the G-code.\n"
            "Leave at 0 to use the G-code's own feed rates."
        )
        overrides_grid.addWidget(self.feedrate_spin, 0, 1)

        overrides_grid.addWidget(QLabel("Fiber diameter (µm):"), 1, 0)
        self.fibre_diameter_spin = QDoubleSpinBox()
        self.fibre_diameter_spin.setRange(0, 1_000_000)
        self.fibre_diameter_spin.setDecimals(3)
        self.fibre_diameter_spin.setSingleStep(0.1)
        self.fibre_diameter_spin.setToolTip(
            "If > 0, dictates the fibre diameter directly:\n"
            "Used for material calculations.\n"
            "Leave at 0 to not calculate the material used."
        )
        overrides_grid.addWidget(self.fibre_diameter_spin, 1, 1)

        overrides_grid.addWidget(QLabel("Density (g/mm³):"), 2, 0)
        self.density_spin = QDoubleSpinBox()
        self.density_spin.setRange(0, 1_000_000)
        self.density_spin.setDecimals(3)
        self.density_spin.setSingleStep(0.1)
        self.density_spin.setToolTip(
            "If > 0, dictates material used directly:\n"
            "material density (g/mm³).\n"
            "Leave at 0 to use the file's fibre diameter / material density."
        )
        overrides_grid.addWidget(self.density_spin, 2, 1)

        overrides_grid.addWidget(QLabel("Acceleration (mm/s²):"), 3, 0)
        self.accel_spin = QDoubleSpinBox()
        self.accel_spin.setRange(0, 1_000_000)
        self.accel_spin.setDecimals(1)
        self.accel_spin.setSingleStep(10)
        self.accel_spin.setToolTip(
            "If > 0, also estimates the time and the real speed along the path with\n"
            "constant acceleration and junction-deviation corners (Marlin 2 model).\n"
            "Enables the 'Actual speed' colouring of the preview.\n"
            "Leave at 0 for the plain distance / feed rate estimate."
        )
        overrides_grid.addWidget(self.accel_spin, 3, 1)

        overrides_grid.addWidget(QLabel("Junction deviation (mm):"), 4, 0)
        self.junction_spin = QDoubleSpinBox()
        self.junction_spin.setRange(0, 10)
        self.junction_spin.setDecimals(3)
        self.junction_spin.setSingleStep(0.001)
        self.junction_spin.setToolTip(
            "How much corner rounding the printer may use to keep its speed through a corner\n"
            "(constant-velocity mode, Marlin 2's junction deviation; default 0.013 mm).\n"
            "A corner only slows the head if it can't be taken at speed within this rounding\n"
            "at the acceleration limit. 0 = off (jerk only; both 0 = stop at every corner).\n"
            "Only used when acceleration is > 0."
        )
        overrides_grid.addWidget(self.junction_spin, 4, 1)

        overrides_grid.addWidget(QLabel("Jerk (mm/s):"), 5, 0)
        self.jerk_spin = QDoubleSpinBox()
        self.jerk_spin.setRange(0, 1_000)
        self.jerk_spin.setDecimals(2)
        self.jerk_spin.setSingleStep(0.5)
        self.jerk_spin.setToolTip(
            "Classic jerk: the largest instant speed change each axis may make at a corner.\n"
            "Applied together with the junction deviation - the lower corner speed wins.\n"
            "The two are linked: Marlin suggests junction deviation = 0.4 x jerk^2 / acceleration\n"
            "(5 mm/s at 1000 mm/s^2 -> 0.01 mm), so they give similar corner speeds.\n"
            "0 = off. Only used when acceleration is > 0."
        )
        overrides_grid.addWidget(self.jerk_spin, 5, 1)

        overrides_grid.addWidget(QLabel("CTS (mm/min):"), 6, 0)
        self.cts_spin = QDoubleSpinBox()
        self.cts_spin.setRange(0, 1_000_000)
        self.cts_spin.setDecimals(1)
        self.cts_spin.setSingleStep(10)
        self.cts_spin.setToolTip(
            "Critical translation speed for the lag model and 'Path below CTS'.\n"
            "Leave at 0 to use the file's CriticalTranslationSpeed parameter."
        )
        overrides_grid.addWidget(self.cts_spin, 6, 1)
        left_layout.addWidget(overrides_group)

        out_group = QGroupBox("Output Folder")
        out_layout = QVBoxLayout(out_group)
        self.output_dir_label = QLabel(str(SCRIPT_DIR))
        self.output_dir_label.setWordWrap(True)
        self.output_dir_label.setToolTip(
            "Unlooper.py creates its 'Output/<name>/' folder inside whichever\n"
            "directory it is run from. Choose that base directory here."
        )
        out_layout.addWidget(self.output_dir_label)
        browse_output_btn = QPushButton("Browse…")
        browse_output_btn.clicked.connect(self._browse_output_dir)
        out_layout.addWidget(browse_output_btn)
        left_layout.addWidget(out_group)

        run_row = QHBoxLayout()
        self.run_btn = QPushButton("Run")
        self.run_btn.setFixedHeight(36)
        f = self.run_btn.font()
        f.setBold(True)
        self.run_btn.setFont(f)
        self.run_btn.clicked.connect(self._start_run)
        run_row.addWidget(self.run_btn)
        self.cancel_btn = QPushButton("Cancel")
        self.cancel_btn.setEnabled(False)
        self.cancel_btn.clicked.connect(self._cancel_run)
        run_row.addWidget(self.cancel_btn)
        left_layout.addLayout(run_row)

        results_group = QGroupBox("Results")
        results_grid = QGridLayout(results_group)
        self._result_labels = {}
        for row, (key, caption) in enumerate([
            ("distance", "Distance travelled"),
            ("time", "Estimated time"),
            ("material", "Material used"),
            ("size", "Build size used (x, y)"),
            ("time_accel", "Est. time (accel/junction)"),
            ("avg_speed", "Avg. actual speed"),
            ("below_cts", "Path below CTS"),
        ]):
            cap = QLabel(caption + ":")
            val = QLabel("—")
            val.setStyleSheet("font-weight:bold;")
            results_grid.addWidget(cap, row, 0)
            results_grid.addWidget(val, row, 1)
            self._result_labels[key] = val
        left_layout.addWidget(results_group)

        self.output_folder_btn = QPushButton("Open Output Folder")
        self.output_folder_btn.setEnabled(False)
        self.output_folder_btn.clicked.connect(self._open_output_folder)
        left_layout.addWidget(self.output_folder_btn)

        left_layout.addStretch()

        # Right: editable code + vector preview + console log
        right_splitter = QSplitter(Qt.Orientation.Vertical)

        preview_group = QGroupBox("NCViewer Preview")
        preview_layout = QVBoxLayout(preview_group)
        self.preview_view = ToolpathView()
        self.preview_view.setScene(QGraphicsScene(self.preview_view))
        self.preview_view.setRenderHint(QPainter.RenderHint.Antialiasing)
        self.preview_view.setDragMode(QGraphicsView.DragMode.ScrollHandDrag)
        self.preview_view.setBackgroundBrush(Qt.GlobalColor.black)
        self.preview_view.setMinimumHeight(260)
        preview_layout.addWidget(self.preview_view)
        preview_row = QHBoxLayout()
        self.hide_after_cursor_check = QCheckBox("Hide toolpath after cursor")
        self.hide_after_cursor_check.setToolTip("Show only toolpath up to the current line in the G-code editor.")
        self.hide_after_cursor_check.toggled.connect(self._update_preview_limit)
        preview_row.addWidget(self.hide_after_cursor_check)
        self.cursor_line_label = QLabel("Cursor line: 1")
        preview_row.addWidget(self.cursor_line_label, stretch=1)
        self.colour_mode_combo = QComboBox()
        self.colour_mode_combo.addItem("Colour: move type", "kind")
        self.colour_mode_combo.addItem("Colour: actual speed", "speed")
        self.colour_mode_combo.addItem("Colour: acceleration", "accel")
        self.colour_mode_combo.addItem("Colour: jet lag", "lag")
        self.colour_mode_combo.setToolTip(
            "Speed and acceleration come from the pixel coords along the planned motion,\n"
            "which need Acceleration > 0 on the run (see Overrides).\n"
            "Jet lag shows where the jet lands (coloured by lag) over the nozzle path in grey;\n"
            "it needs 'Lag prediction' on the run.\n"
            "Click a colour in the legend to highlight that band."
        )
        self.colour_mode_combo.currentIndexChanged.connect(self._rebuild_preview_items)
        preview_row.addWidget(self.colour_mode_combo)
        fit_btn = QPushButton("Fit View")
        fit_btn.clicked.connect(self._fit_preview)
        preview_row.addWidget(fit_btn)
        preview_layout.addLayout(preview_row)
        self.speed_legend = SpeedLegend()
        self.speed_legend.setVisible(False)
        self.speed_legend.highlightChanged.connect(self._on_highlight_changed)
        preview_layout.addWidget(self.speed_legend)
        right_splitter.addWidget(preview_group)

        editor_group = QGroupBox("Unlooped G-code")
        editor_layout = QVBoxLayout(editor_group)
        self.code_editor = QPlainTextEdit()
        self.code_editor.setLineWrapMode(QPlainTextEdit.LineWrapMode.NoWrap)
        self.code_editor.setStyleSheet("font-family:Consolas,monospace; font-size:11px;")
        self.code_editor.setCenterOnScroll(True)
        self.code_editor.cursorPositionChanged.connect(self._update_preview_limit)
        self.code_editor.textChanged.connect(self._invalidate_code_line_index)
        self._code_line_index = None
        editor_layout.addWidget(self.code_editor)
        edit_row = QHBoxLayout()
        self.reload_code_btn = QPushButton("Reload Output Code")
        self.reload_code_btn.clicked.connect(self._load_code_editor)
        edit_row.addWidget(self.reload_code_btn)
        self.replot_btn = QPushButton("Replot Edited Code")
        self.replot_btn.setToolTip("Reprocess the edited code and return to the same line, zoom and position.")
        self.replot_btn.setEnabled(False)
        self.replot_btn.clicked.connect(lambda: self._replot_edited(keep_place=True))
        edit_row.addWidget(self.replot_btn)
        self.replot_all_btn = QPushButton("Replot All")
        self.replot_all_btn.setToolTip("Reprocess the edited code and show the whole toolpath from the top.")
        self.replot_all_btn.setEnabled(False)
        self.replot_all_btn.clicked.connect(lambda: self._replot_edited(keep_place=False))
        edit_row.addWidget(self.replot_all_btn)
        editor_layout.addLayout(edit_row)
        right_splitter.addWidget(editor_group)

        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(5000)
        self.log.setStyleSheet("font-family:Consolas,monospace; font-size:11px;")
        right_splitter.addWidget(self.log)
        right_splitter.setChildrenCollapsible(False)
        right_splitter.setSizes([440, 360, 220])
        right_splitter.setStretchFactor(0, 4)
        right_splitter.setStretchFactor(1, 3)
        right_splitter.setStretchFactor(2, 2)

        splitter.addWidget(left)
        splitter.addWidget(right_splitter)
        splitter.setChildrenCollapsible(False)
        splitter.setSizes([360, 840])
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        root.addWidget(splitter, stretch=1)

        self.progress_bar = QProgressBar()
        self.progress_bar.setRange(0, 100)
        self.progress_bar.setValue(0)
        root.addWidget(self.progress_bar)

        self.status_label = QLabel("Ready")
        root.addWidget(self.status_label)

    # ── File selection ───────────────────────────────────────────────────

    def _open_files(self):
        paths, _ = QFileDialog.getOpenFileNames(
            self, "Open G-code File(s)", "",
            "G-code / Text Files (*.gcode *.txt *.nc *.tap);;All Files (*)",
        )
        if not paths:
            return
        self.input_files = paths
        self.queue_list.clear()
        self.queue_list.addItems(Path(p).name for p in self.input_files)
        self.file_label.setText(
            paths[0] if len(paths) == 1 else f"{len(paths)} files selected"
        )
        self._active_file = None
        self._queue_index = -1
        self._reset_results()
        self._update_run_enabled()

    def _clear_files(self):
        self.input_files = []
        self._active_file = None
        self._queue_index = -1
        self.queue_list.clear()
        self.file_label.setText("No files selected")
        self._reset_results()
        self._update_run_enabled()

    def _set_queue_status(self, index: int, status: str):
        item = self.queue_list.item(index)
        if item is not None:
            item.setText(f"{Path(self.input_files[index]).name}  —  {status}")

    def _load_settings(self):
        # Last-used values, with acceleration 1000 mm/s^2, junction deviation 0.013 mm and jerk 5 mm/s the first time
        settings = QSettings("Unlooper", "Unlooper")
        self.feedrate_spin.setValue(float(settings.value("feedrate", 0.0)))
        self.fibre_diameter_spin.setValue(float(settings.value("fibre_diameter", 0.0)))
        self.density_spin.setValue(float(settings.value("density", 0.0)))
        self.accel_spin.setValue(float(settings.value("acceleration", DEFAULT_ACCELERATION)))
        self.junction_spin.setValue(float(settings.value("junction_deviation", DEFAULT_JUNCTION_DEVIATION)))
        self.jerk_spin.setValue(float(settings.value("jerk", DEFAULT_JERK)))
        self.cts_spin.setValue(float(settings.value("cts", 0.0)))
        self.lag_check.setChecked(settings.value("lag_prediction", "false") in (True, "true"))
        self.write_files_check.setChecked(settings.value("write_lag_files", "false") in (True, "true"))
        self.render_mode_combo.setCurrentIndex(max(0, self.render_mode_combo.findData(settings.value("render_mode", "preview"))))
        self.motion_only_check.setChecked(settings.value("motion_only", "false") in (True, "true"))
        self.skip_pixel_check.setChecked(settings.value("skip_pixel_coords", "false") in (True, "true"))
        self.colour_mode_combo.setCurrentIndex(max(0, self.colour_mode_combo.findData(settings.value("colour_mode", "kind"))))

    def _save_settings(self):
        settings = QSettings("Unlooper", "Unlooper")
        settings.setValue("feedrate", self.feedrate_spin.value())
        settings.setValue("fibre_diameter", self.fibre_diameter_spin.value())
        settings.setValue("density", self.density_spin.value())
        settings.setValue("acceleration", self.accel_spin.value())
        settings.setValue("junction_deviation", self.junction_spin.value())
        settings.setValue("jerk", self.jerk_spin.value())
        settings.setValue("cts", self.cts_spin.value())
        settings.setValue("lag_prediction", self.lag_check.isChecked())
        settings.setValue("write_lag_files", self.write_files_check.isChecked())
        settings.setValue("render_mode", self.render_mode_combo.currentData())
        settings.setValue("motion_only", self.motion_only_check.isChecked())
        settings.setValue("skip_pixel_coords", self.skip_pixel_check.isChecked())
        settings.setValue("colour_mode", self.colour_mode_combo.currentData())

    def _update_option_states(self):
        unloop_only = self.unloop_only_check.isChecked()
        self.motion_only_check.setEnabled(not unloop_only)
        self.render_mode_combo.setEnabled(not unloop_only and not self.motion_only_check.isChecked())
        self.skip_pixel_check.setEnabled(not unloop_only and not self.motion_only_check.isChecked())
        self.lag_check.setEnabled(not unloop_only)
        self.write_files_check.setEnabled(not unloop_only)

    def _update_run_enabled(self):
        running = self.process is not None and self.process.state() != QProcess.ProcessState.NotRunning
        self.run_btn.setEnabled(bool(self.input_files) and not running)
        self.cancel_btn.setEnabled(running)
        self.open_btn.setEnabled(not running)
        self.clear_files_btn.setEnabled(bool(self.input_files) and not running)

    def _reset_results(self):
        self._run_completed = False
        for lbl in self._result_labels.values():
            lbl.setText("—")
        self.preview_view.scene().clear()
        self._clear_preview_state()
        self.code_editor.clear()
        self.cursor_line_label.setText("Cursor line: 1")
        self.replot_btn.setEnabled(False)
        self.replot_all_btn.setEnabled(False)
        self.output_folder_btn.setEnabled(False)

    # ── Run / cancel ─────────────────────────────────────────────────────

    def _stem(self) -> str:
        return Path(self._active_file).stem

    def _output_dir(self) -> Path:
        return self.output_base_dir / "Output" / self._stem()

    @Slot()
    def _browse_output_dir(self):
        chosen = QFileDialog.getExistingDirectory(
            self, "Choose Output Base Folder", str(self.output_base_dir),
        )
        if not chosen:
            return
        self.output_base_dir = Path(chosen)
        self.output_dir_label.setText(str(self.output_base_dir))
        self._reset_results()

    @Slot()
    def _start_run(self):
        if not self.input_files or not UNLOOPER_SCRIPT.exists():
            self.status_label.setText(f"Error: cannot find {UNLOOPER_SCRIPT}")
            return

        self._save_settings()
        self.log.clear()
        self._batch_cancelled = False
        self._queue_index = -1
        self._run_next_file()

    def _run_next_file(self):
        self._queue_index += 1
        if self._queue_index >= len(self.input_files):
            self.status_label.setText(f"All {len(self.input_files)} file(s) complete")
            self._update_run_enabled()
            return

        self._active_file = self.input_files[self._queue_index]
        n = len(self.input_files)
        position = self._queue_index + 1
        name = Path(self._active_file).name

        self._reset_results()
        self._out_buffer = ""
        self.progress_bar.setValue(0)
        self.status_label.setText(f"[{position}/{n}] Starting {name}…")
        self.log.appendPlainText(f"\n===== [{position}/{n}] {name} =====")
        self._set_queue_status(self._queue_index, "Running")

        unloop_only = "1" if self.unloop_only_check.isChecked() else "0"
        feedrate = str(self.feedrate_spin.value())
        density = str(self.density_spin.value())
        fibre_diameter = str(self.fibre_diameter_spin.value())
        # flowrate = str(self.flowrate_spin.value())

        env = QProcessEnvironment.systemEnvironment()
        env.insert("PYTHONUNBUFFERED", "1")
        env.insert("PYTHONIOENCODING", "utf-8")

        self.process = QProcess(self)
        self.process.setProcessEnvironment(env)
        self.process.setWorkingDirectory(str(self.output_base_dir))
        self.process.setProcessChannelMode(QProcess.ProcessChannelMode.MergedChannels)
        self.process.setProgram(sys.executable)
        # Arguments are: Unlooper.py <file> <unloop_only> <feedrate> <density> <fibre_diameter> <render_mode>
        # Restored to provide more user control over the material estimate, as per the recent edits in Unlooper.py
        render_mode = "none" if self.motion_only_check.isChecked() else self.render_mode_combo.currentData()
        self._run_render_mode = render_mode
        # ... <acceleration mm/s^2> <junction deviation mm> <skip pixel coords> <jerk mm/s>
        #     <lag prediction> <CTS mm/min> <write lag-format files>
        accel = str(self.accel_spin.value())
        junction = str(self.junction_spin.value())
        skip_pixel = "1" if self.skip_pixel_check.isChecked() else "0"
        jerk = str(self.jerk_spin.value())
        lag = "1" if self.lag_check.isChecked() else "0"
        cts = str(self.cts_spin.value())
        write_files = "1" if self.write_files_check.isChecked() else "0"
        self.process.setArguments([str(UNLOOPER_SCRIPT), self._active_file, unloop_only, feedrate, density, fibre_diameter,
                                   render_mode, accel, junction, skip_pixel, jerk, lag, cts, write_files])
        self.process.readyReadStandardOutput.connect(self._on_output)
        self.process.finished.connect(self._on_finished)
        self.process.errorOccurred.connect(self._on_process_error)
        self.process.start()
        self._update_run_enabled()

    @Slot()
    def _cancel_run(self):
        if self.process and self.process.state() != QProcess.ProcessState.NotRunning:
            self._batch_cancelled = True
            self.status_label.setText("Cancelling…")
            self.process.kill()

    # ── Output parsing ───────────────────────────────────────────────────

    @Slot()
    def _on_output(self):
        data = bytes(self.process.readAllStandardOutput()).decode("utf-8", errors="replace")
        self._out_buffer += data
        parts = re.split(r"[\r\n]", self._out_buffer)
        self._out_buffer = parts[-1]
        for line in parts[:-1]:
            line = line.strip()
            if not line:
                continue
            m = TQDM_RE.match(line)
            if m:
                self.progress_bar.setValue(int(m.group(1)))
                self.status_label.setText(f"Rendering image… {m.group(1)}%")
                continue
            m = STAGE_PROGRESS_RE.match(line)
            if m:
                # Progress lines drive the bar and status only, so they don't flood the log
                self.progress_bar.setValue(int(m.group(2)))
                self.status_label.setText(f"{m.group(1)}… {m.group(2)}%")
                continue
            self.log.appendPlainText(line)
            self.status_label.setText(line)
            self._parse_metrics(line)

    def _parse_metrics(self, line: str):
        m = DISTANCE_RE.search(line)
        if m:
            self._result_labels["distance"].setText(f"{m.group(1)} m")
        m = TIME_RE.search(line)
        if m:
            self._result_labels["time"].setText(m.group(1).strip())
        m = MATERIAL_RE.search(line)
        if m:
            self._result_labels["material"].setText(f"{m.group(1)} mg")
        m = SIZE_RE.search(line)
        if m:
            self._result_labels["size"].setText(f"{m.group(1)} mm, {m.group(2)} mm")
        m = TIME_ACCEL_RE.search(line)
        if m:
            self._result_labels["time_accel"].setText(m.group(1).strip())
        m = AVG_SPEED_RE.search(line)
        if m:
            self._result_labels["avg_speed"].setText(f"{m.group(1)} mm/min")
        m = BELOW_CTS_RE.search(line)
        if m:
            self._result_labels["below_cts"].setText(m.group(1).strip())

    @Slot(int, QProcess.ExitStatus)
    def _on_finished(self, exit_code, exit_status):
        self._run_completed = True
        if exit_status == QProcess.ExitStatus.CrashExit:
            status = "Cancelled" if self._batch_cancelled else "Crashed"
            self.status_label.setText(status)
            self.progress_bar.setValue(0)
            self._set_queue_status(self._queue_index, status)
        elif exit_code == 0:
            self.status_label.setText("Done")
            self.progress_bar.setValue(100)
            self._load_preview()
            self._load_code_editor()
            if self._pending_place is not None:
                self._restore_place(self._pending_place)
            self._set_queue_status(self._queue_index, "Done")
        else:
            self.status_label.setText(f"Failed (exit code {exit_code}) — see log")
            self._set_queue_status(self._queue_index, "Failed")
        self.output_folder_btn.setEnabled(self._output_dir().exists())
        self._pending_place = None

        if self._batch_cancelled:
            self._update_run_enabled()
            return
        self._run_next_file()

    @Slot(QProcess.ProcessError)
    def _on_process_error(self, error):
        self.log.appendPlainText(f"[process error] {error}")

    def _clear_preview_state(self):
        if hasattr(self, "preview_view"):
            self.preview_view.grid_rect = None
        self._preview_segments = None
        self._preview_lines = None
        self._preview_chunk_items = []
        self._preview_partial_items = []
        self._preview_partial_key = None
        self._preview_colours = {}
        self._preview_samples = None
        self._preview_sample_lines = None
        self._preview_speed_range = (0.0, 1.0)
        self._preview_cts = 0.0
        self._raster_loaded = False
        self._raster = None  # (index image, palette) of an indexed PNG, for highlighting
        self._legend_info = None
        self._lag_info = None
        self._lag_samples = None
        self._lag_sample_lines = None
        self._lag_range = (0.0, 0.0)
        self._underlay_items = []
        self._highlight = None
        if hasattr(self, "speed_legend"):
            self.speed_legend.highlight = None

    def _read_json(self, suffix):
        path = self._output_dir() / f"{self._stem()}{suffix}"
        if path.exists():
            try:
                return json.loads(path.read_text(encoding="utf-8"))
            except (OSError, ValueError):
                return None
        return None

    def _load_preview(self):
        scene = self.preview_view.scene()
        scene.clear()
        self._clear_preview_state()
        self._legend_info = self._read_json("_pixel_coords_legend.json")
        self._lag_info = self._read_json("_lag_legend.json")
        mode = self._run_render_mode
        if mode in ("preview", "both") and self._load_vector_preview():
            return
        if mode in ("precise", "both") and self._load_raster_preview():
            self._update_legend()
            return
        if mode == "none":
            self.status_label.setText("Done - plotting skipped (motion calcs only)")
        else:
            self.status_label.setText("No preview was generated for this run")

    def _load_vector_preview(self):
        scene = self.preview_view.scene()
        seg_path = self._output_dir() / f"{self._stem()}_preview_segments.npz"
        if seg_path.exists():
            data = np.load(seg_path)
            segments = data["segments"]
            if len(segments) == 0:
                return False
            self._preview_segments = segments
            self._preview_lines = segments[:, 8]
            self._preview_colours = {k: QColor(*map(int, c)) for k, c in zip((1, 2, 3), data["colours"])}
            view = self.preview_view
            view.plate_brush = QColor(*map(int, data["background"]))
            if "grid_colour" in data:
                view.grid_pen = QPen(QColor(*map(int, data["grid_colour"])), 0)
                view.grid_spacing = float(data["grid_spacing"])
            samples = data["samples"] if "samples" in data else None
            if samples is not None and len(samples) > 1:
                self._preview_samples = samples
                self._preview_sample_lines = samples[:, 4]
                v_min, v_max = (float(v) for v in data["speed_range"]) if "speed_range" in data else (0.0, 0.0)
                self._preview_speed_range = (v_min, v_max) if v_max > v_min else (float(samples[:, 2].min()), float(samples[:, 2].max()))
                self._preview_cts = float(data["cts"]) if "cts" in data else 0.0
            lag_samples = data["lag_samples"] if "lag_samples" in data else None
            if lag_samples is not None and len(lag_samples) > 1:
                self._lag_samples = lag_samples
                self._lag_sample_lines = lag_samples[:, 3]
                self._lag_range = tuple(float(v) for v in data["lag_range"])
            self._build_all_chunks()
            self._update_legend()
            # Build plate = toolpath extents rounded out to whole grid squares plus a 1 mm
            # border, matching the PNG's plate
            bounds = scene.itemsBoundingRect()
            step = view.grid_spacing
            left = math.floor(bounds.left() / step) * step - step
            top = math.floor(bounds.top() / step) * step - step
            right = math.ceil(bounds.right() / step) * step + step
            bottom = math.ceil(bounds.bottom() / step) * step + step
            view.grid_rect = QRectF(left, top, right - left, bottom - top)
            scene.setSceneRect(view.grid_rect)
            self._fit_preview()
            self._update_preview_limit()
            return True

        # Older runs only have the SVG - show it as a single item (no per-line hiding)
        svg_path = self._output_dir() / f"{self._stem()}_preview.svg"
        if svg_path.exists():
            item = QGraphicsSvgItem(str(svg_path))
            if item.renderer().isValid():
                scene.addItem(item)
                scene.setSceneRect(item.boundingRect())
                self._fit_preview()
                return True
        return False

    def _sample_mode(self):
        # Speed / acceleration colouring draws the pixel coords instead of the G-code moves,
        # jet lag colouring draws the jet contact points
        mode = self.colour_mode_combo.currentData()
        if mode == "lag":
            return self._lag_samples is not None
        return mode in ("speed", "accel") and self._preview_samples is not None

    def _active_lines(self):
        if not self._sample_mode():
            return self._preview_lines
        return self._lag_sample_lines if self.colour_mode_combo.currentData() == "lag" else self._preview_sample_lines

    def _band_colours(self):
        # Colour per band, with everything but the highlighted band dimmed
        return {b: QColor(*(c if self._highlight in (None, b) else DIMMED)) for b, c in enumerate(SPEED_COLOURS)}

    def _build_all_chunks(self):
        if self.colour_mode_combo.currentData() == "lag" and self._sample_mode():
            # Nozzle path in grey under the jet path
            paths = build_toolpath_paths(self._preview_segments)
            for path in paths.values():
                item = QGraphicsPathItem(path)
                item.setPen(QPen(QColor(*NOZZLE_UNDERLAY), 0))
                self.preview_view.scene().addItem(item)
                self._underlay_items.append(item)
        total = len(self._active_lines())
        for start in range(0, total, PREVIEW_CHUNK):
            self._preview_chunk_items.append(self._add_path_items(start, min(start + PREVIEW_CHUNK, total)))

    def _add_path_items(self, start, stop, join_next=True):
        scene = self.preview_view.scene()
        if self._sample_mode():
            mode = self.colour_mode_combo.currentData()
            source = self._lag_samples if mode == "lag" else self._preview_samples
            # Include the next chunk's first point so consecutive chunks join up
            samples = source[start:stop + 1 if join_next else stop]
            if mode in ("speed", "lag"):
                v_min, v_max = self._lag_range if mode == "lag" else self._preview_speed_range
                bins = len(SPEED_COLOURS)
                width = max(v_max - v_min, 1e-9) / bins
                paths = build_sample_paths(samples, lambda v, a: min(bins - 1, max(0, int((v - v_min) / width))))
                colours = self._band_colours()
            else:
                paths = build_sample_paths(samples, lambda v, a: (a > 0) - (a < 0))
                colours = {k: QColor(*c) for k, (c, _label) in ACCEL_COLOURS.items()}
        else:
            paths = build_toolpath_paths(self._preview_segments[start:stop])
            colours = self._preview_colours
        items = []
        for key, path in paths.items():
            item = QGraphicsPathItem(path)
            item.setPen(QPen(colours.get(key, QColor("white")), 0))  # width 0 = cosmetic hairline
            scene.addItem(item)
            items.append(item)
        return items

    def _update_legend(self):
        mode = self.colour_mode_combo.currentData()
        if mode == "kind":
            self.speed_legend.setVisible(False)
            return
        self.speed_legend.setVisible(True)
        bands = len(SPEED_COLOURS)
        if mode == "lag":
            lag_range = self._lag_range if self._lag_samples is not None else None
            if lag_range is None and self._raster_loaded and self._lag_info is not None:
                lag_range = (self._lag_info["lag_min_mm"], self._lag_info["lag_max_mm"])
            if lag_range is None:
                self.speed_legend.set_message("No jet lag data - run with 'Lag prediction' ticked and Acceleration > 0")
            else:
                self.speed_legend.set_bands(np.linspace(lag_range[0], lag_range[1], bands + 1), "mm lag", decimals=3)
            return
        has_raster_data = self._raster_loaded and self._legend_info is not None
        if self._preview_samples is None and not has_raster_data:
            self.speed_legend.set_message("No speed data - run with Acceleration > 0 and 'Skip pixel coords' unticked")
        elif mode == "speed":
            if self._preview_samples is None:
                info = self._legend_info
                v_min, v_max, cts = info["speed_min_mm_s"], info["speed_max_mm_s"], info.get("cts_mm_s", 0.0)
            else:
                (v_min, v_max), cts = self._preview_speed_range, self._preview_cts
            self.speed_legend.set_bands(np.linspace(v_min, v_max, bands + 1) * 60, "mm/min", decimals=1,
                                        marker=cts * 60 if cts > 0 else None)
        else:
            self.speed_legend.set_entries([ACCEL_COLOURS[k] for k in (-1, 0, 1)])

    def _on_highlight_changed(self, band):
        self._highlight = band
        if self._preview_segments is None:
            if self._raster is not None:
                transform = self.preview_view.transform()
                self.preview_view.scene().clear()
                self._show_raster()
                self.preview_view.setTransform(transform)
            return
        self._rebuild_preview_items(keep_legend=True)

    def _rebuild_preview_items(self, *_args, keep_legend=False):
        # Re-colour the loaded toolpath (move type / speed / acceleration / jet lag) without
        # reloading it. A new colour mode clears the band highlight.
        if not keep_legend:
            self._highlight = None
            self.speed_legend.highlight = None
        self._update_legend()
        if self._preview_segments is None:
            if self._raster_loaded:
                # PNG preview: switch to the matching image
                transform = self.preview_view.transform()
                self.preview_view.scene().clear()
                self._load_raster_preview()
                self.preview_view.setTransform(transform)
                self._update_legend()
            return
        scene = self.preview_view.scene()
        for items in self._preview_chunk_items + [self._preview_partial_items, self._underlay_items]:
            for item in items:
                scene.removeItem(item)
        self._underlay_items = []
        self._preview_chunk_items = []
        self._preview_partial_items = []
        self._preview_partial_key = None
        self._build_all_chunks()
        self._update_preview_limit()

    def _load_raster_preview(self):
        # Move-type PNG from Plot_code(), or the speed / acceleration / jet lag PNGs drawn from
        # the pixel coords. Those are palette PNGs (pixel = colour band), kept in memory so a
        # band can be highlighted by changing the palette.
        suffix = {"speed": "_pixel_coords_speed.png", "accel": "_pixel_coords_accel.png",
                  "lag": "_lag.png"}.get(self.colour_mode_combo.currentData(), "_cv2_Image_output.png")
        image_path = self._output_dir() / f"{self._stem()}{suffix}"
        if not image_path.exists():
            image_path = self._output_dir() / f"{self._stem()}_cv2_Image_output.png"
        if not image_path.exists():
            return False
        self._raster_loaded = True
        self._raster = None
        try:
            with Image.open(image_path) as pil_image:
                if pil_image.mode == "P":
                    palette = np.asarray(pil_image.getpalette()[:768], dtype=np.uint8).reshape(-1, 3)
                    self._raster = (np.asarray(pil_image), palette)
        except (OSError, ValueError):
            self._raster = None
        if self._raster is not None:
            return self._show_raster()
        image = cv2.imread(str(image_path), cv2.IMREAD_UNCHANGED)
        if image is None:
            return False
        if len(image.shape) == 3:
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        return self._show_image(image)

    def _show_raster(self):
        # Colour the indexed PNG, dimming all bands but the highlighted one
        index_image, palette = self._raster
        lut = palette.copy()
        if self._highlight is not None and self.colour_mode_combo.currentData() in ("speed", "lag"):
            for b in range(len(SPEED_COLOURS)):
                if b != self._highlight:
                    lut[b + 1] = DIMMED
        return self._show_image(lut[index_image])

    def _show_image(self, image):
        # Show an RGB (or grey) image scaled down to the preview size
        preview_size = self.preview_view.size()
        if preview_size.width() <= 0 or preview_size.height() <= 0:
            preview_size = QSize(1400, 900)
        height, width = image.shape[:2]
        max_width = max(1, min(preview_size.width(), 1600))
        max_height = max(1, min(preview_size.height(), 1600))
        scale = min(max_width / width, max_height / height, 1.0)
        if scale < 1.0:
            target_width = max(1, int(width * scale))
            target_height = max(1, int(height * scale))
            image = cv2.resize(image, (target_width, target_height), interpolation=cv2.INTER_AREA)
        image = np.ascontiguousarray(image)
        if len(image.shape) == 2:
            qimage = QImage(image.data, image.shape[1], image.shape[0], image.strides[0], QImage.Format_Grayscale8)
        else:
            qimage = QImage(image.data, image.shape[1], image.shape[0], image.strides[0], QImage.Format_RGB888)
        if qimage.isNull():
            return False
        scene = self.preview_view.scene()
        pixmap_item = scene.addPixmap(QPixmap.fromImage(qimage.copy()))
        scene.setSceneRect(pixmap_item.boundingRect())
        self._fit_preview()
        return True

    def _fit_preview(self):
        scene = self.preview_view.scene()
        if scene.items():
            self.preview_view.fitInView(scene.sceneRect(), Qt.AspectRatioMode.KeepAspectRatio)

    def _invalidate_code_line_index(self):
        self._code_line_index = None

    def _editor_line_to_code_index(self, block):
        # The editor holds the saved file, which has comment lines (the ; header with time,
        # size and material, plus any comments added while editing) that Unlooper.py drops
        # before plotting. Map the editor line to the index of the last real code line at or
        # before it. Built lazily and cached until the text changes.
        if self._code_line_index is None:
            lines = self.code_editor.toPlainText().split("\n")
            flags = np.fromiter((is_code_line(line) for line in lines), dtype=bool, count=len(lines))
            self._code_line_index = np.cumsum(flags) - 1
        if len(self._code_line_index) == 0:
            return -1
        return int(self._code_line_index[min(block, len(self._code_line_index) - 1)])

    def _update_preview_limit(self):
        cursor_line = self.code_editor.textCursor().blockNumber()
        self.cursor_line_label.setText(f"Cursor line: {cursor_line + 1}")
        if self._preview_segments is None:
            return
        if self.hide_after_cursor_check.isChecked():
            # Show everything up to and including the code line the cursor is on
            self._set_preview_line_limit(self._editor_line_to_code_index(cursor_line))
        else:
            self._set_preview_line_limit(None)

    def _set_preview_line_limit(self, max_line):
        # Whole chunks are shown/hidden; only the chunk the cursor falls inside is rebuilt
        lines = self._active_lines()
        total = len(lines)
        if max_line is None:
            visible = total
        else:
            visible = int(np.searchsorted(lines, max_line, side="right"))
        full_chunks = visible // PREVIEW_CHUNK
        for index, items in enumerate(self._preview_chunk_items):
            show = index < full_chunks or visible == total
            for item in items:
                item.setVisible(show)

        partial_key = visible if (visible < total and visible % PREVIEW_CHUNK) else None
        if partial_key == self._preview_partial_key:
            return
        scene = self.preview_view.scene()
        for item in self._preview_partial_items:
            scene.removeItem(item)
        self._preview_partial_items = []
        self._preview_partial_key = partial_key
        if partial_key is not None:
            start = full_chunks * PREVIEW_CHUNK
            self._preview_partial_items = self._add_path_items(start, visible, join_next=False)

    def _load_code_editor(self):
        code_path = self._output_dir() / f"{self._stem()}_Unlooped_Code.txt"
        if code_path.exists():
            self.code_editor.setPlainText(code_path.read_text(encoding="utf-8"))
            self.replot_btn.setEnabled(True)
            self.replot_all_btn.setEnabled(True)

    def _capture_place(self):
        # Remember where the user is so a replot can put them back there. The editor
        # position is stored as a code-line index (not a raw editor line) because the
        # reprocessed file drops added comments and rewrites the ; header block.
        cursor = self.code_editor.textCursor()
        block = cursor.blockNumber()
        view = self.preview_view
        return {
            "code_index": self._editor_line_to_code_index(block),
            "column": cursor.positionInBlock(),
            "scroll_offset": block - self.code_editor.verticalScrollBar().value(),
            "transform": view.transform() if view.scene().items() else None,
            "centre": view.mapToScene(view.viewport().rect().center()),
        }

    def _restore_place(self, place):
        if self.code_editor.document().isEmpty():
            return
        self._code_line_index = None
        self._editor_line_to_code_index(0)  # rebuild the line map for the new text
        line_map = self._code_line_index
        if place["code_index"] < 0:
            block = 0
        else:
            # First editor line whose code index reaches the saved one = that code line
            block = min(int(np.searchsorted(line_map, place["code_index"], side="left")), len(line_map) - 1)
        text_block = self.code_editor.document().findBlockByNumber(block)
        cursor = QTextCursor(text_block)
        cursor.movePosition(QTextCursor.MoveOperation.Right, n=min(place["column"], text_block.length() - 1))
        self.code_editor.setTextCursor(cursor)
        self.code_editor.verticalScrollBar().setValue(max(0, block - place["scroll_offset"]))
        if place["transform"] is not None and self.preview_view.scene().items():
            self.preview_view.setTransform(place["transform"])
            self.preview_view.centerOn(place["centre"])

    def _replot_edited(self, keep_place=True):
        if self.process and self.process.state() != QProcess.ProcessState.NotRunning:
            return
        if not self.code_editor.toPlainText().strip() or not self._active_file:
            return
        self._pending_place = self._capture_place() if keep_place else None
        edit_dir = self.output_base_dir / ".unlooper_edits"
        edit_dir.mkdir(parents=True, exist_ok=True)
        # Don't stack suffixes (X_edited_edited...) on repeated replots
        stem = Path(self._active_file).stem
        if stem.endswith("_edited"):
            stem = stem[:-len("_edited")]
        edited_path = edit_dir / f"{stem}_edited.gcode"
        edited_path.write_text(self.code_editor.toPlainText(), encoding="utf-8")
        self.unloop_only_check.setChecked(False)
        self.input_files = [str(edited_path)]
        self.queue_list.clear()
        self.queue_list.addItem(edited_path.name)
        self._queue_index = -1
        self.log.clear()
        self._batch_cancelled = False
        self._run_next_file()

    # ── Output folder ────────────────────────────────────────────────────

    def _open_output_folder(self):
        folder = self._output_dir()
        if folder.exists():
            os.startfile(str(folder))

    def closeEvent(self, event):
        self._save_settings()
        if self.process and self.process.state() != QProcess.ProcessState.NotRunning:
            self.process.kill()
            self.process.waitForFinished(3000)
        event.accept()


def main():
    app = QApplication(sys.argv)
    app.setApplicationName("Unlooper")
    win = UnlooperWindow()
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
