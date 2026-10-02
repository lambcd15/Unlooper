"""Stage 6 - lag compensation: rewrite the G-code so the jet lands on the programmed path,
then run the whole pipeline on the rewritten file to see where the jet lands now.

Methods (variables["Lag_compensation"]):

  overshoot  - the ISBF method, Gcode_processing.py vector_angle ported so it writes the
               same G-code (isbf.py): at every corner the nozzle carries on past it by 0.85 x
               the lag, swings round it on an arc at the rapid speed and rejoins the next line
               that far along, dragging the trailing jet into the corner. The lag comes from
               the jet model on the compensated path as it is written (one command ahead, at
               constant speed); around G2/G3 arcs it is the fitted reset distance.
  pointwise  - the same with every line cut into points, so the lag is found and the nozzle
               placed after each point instead of once per command, whatever the command
               lengths. Point spacing > 0 cuts at that spacing. Point spacing 0 is adaptive:
               points one nozzle sample apart (feed x scatter_resolution) at each end of a
               line, growing towards the middle, and the lag model follows the nozzle exactly
               as written, moving as the planner will run it (acceleration, junction speeds);
               at each corner the overshoot is the one that, simulated through the swing,
               keeps the jet closest to the path.
  slowdown   - speed compensation: before each corner the nozzle slows to a set ratio of
               the CTS for just long enough (from the lag model) for the jet to catch up to
               within a tolerance, so the jet reaches the corner with it. The path is
               unchanged; only the feed rates are.
  hybrid     - the nozzle leads the jet along the path by exactly the lag (the lag model run
               backwards), the lag shed before sharp corners so the jump round them stays within
               the corner tolerance, capped round tight curves and where the planner can't keep
               up, then fitted to the simulated jet (hybrid.py). G1 moves only.
  hybrid_constant - the same lead with the jet at the programmed speed throughout, so the lag
               and the fibre diameter stay constant (as ISBF): the path is smoothed to the
               tightest turn the jet can make at that speed instead. G1 moves only.
  iterative  - model-driven: the nozzle path is corrected repeatedly with the real planner
               in the loop - plan the path (acceleration, junction deviation, jerk, corner
               rounding), simulate the jet, measure how far each jet point is from the
               programmed path, move the path points that put it there (one lag's travel
               earlier) by that error, repeat. Written as G1 moves at the original feed
               rates, dense only around corners and along arcs.

Options for overshoot / pointwise (off by default): time-preserving feeds
(Lag_comp_time_preserving) speed each command's moves up so they take the time the command
was programmed to take, and solve the path again at those feeds - the compensated nozzle path
is longer than the path it draws, so at the programmed feed the print takes longer, the jet
runs slower and the fibre comes out thicker. For the adaptive spacing there are also its own
lead factor (Lag_comp_adaptive_lead), the smallest turn swung round
(Lag_comp_adaptive_swing_deg) and swings kept no smaller than the fibre diameter limit's lag
(Lag_comp_adaptive_hold_swing).

Smooth swings (Lag_comp_blend_um; automatic for pointwise): the sharp kinks where an overshoot
line meets its swing arc, and the arc the next line, are each replaced by one small fillet arc
the machine can take at the feed, so the nozzle never brakes below the programmed speed - which
is what keeps the fibre at its diameter.

Steered curves (adaptive spacing; Lag_comp_adaptive_steer): turns gentler than the swing
threshold (30 degrees - a curve written as short lines) are not swung round. The nozzle carries
on at the feed and moves across onto each new line as fast as the machine allows, never slower
than the feed (isbf._g1_joins). How far ahead of the jet the nozzle is taken to be decides how
early it starts across; that lead is searched on the whole file. The solved points are thinned
to within THIN_UM of the solved path before writing (thin_lines). Arcs are left to ISBF's arc
joins unless Lag_comp_adaptive_arcs is "chords".

Correction passes (overshoot / pointwise): the compensated file is planned and simulated
exactly as processing it would (planner, corner rounding, 1 ms pixel coords, lag model) and
every join - a G1 corner's overshoot, an arc's reset distance - is tuned on its own by a
line search on how far the jet lands from the path around it. The file with the jet closest
to the path overall is kept.

The compensated file is Output/<name>/<name>_Lag_compensated.txt. It is then processed by
Unlooper.py in a child process (outputs in Output/<name>_Lag_compensated/) with the lag
model scored against the original programmed path, its console lines are passed on with a
"Compensated" prefix, and <name>_lag_compensation.png shows programmed path (black), jet
before (grey) and jet after compensation (green) as the original ISBF code drew them.
"""
import math
import os
import re
import subprocess
import sys
from types import SimpleNamespace

import cv2
import numpy as np

from .lag_model import REFERENCE_ENV, lag_steps, objective

try:
    from numba import njit
except ImportError:  # pragma: no cover - numba is optional, just slower without it
    njit = None
from .motion_planner import move_directions, plan_moves
from .path_reference import project, reference_polyline
from .pixel_coords import PNG_SHIFT, build_timeline, output_base, point_counts, to_png_pixels


METHODS = ("none", "overshoot", "pointwise", "slowdown", "iterative", "hybrid", "hybrid_constant")
CORNER_MIN_DEG = 5.0  # turns smaller than this are left alone (as vector_angle's angle bound)
MAX_ITERATIVE_POINTS = 4_000_000  # correction passes sample coarser than 1 ms beyond this
BLEND_ACCEL_FRACTION = 0.85  # smooth swings: the blend is taken within this x the acceleration limit
ARC_CHORD_DEG = 10.0  # adaptive spacing: arcs steered round as chords turning no more than this
ARC_CHORD_SAG_UM = 1.0  # ... and within this of the arc
THIN_UM = 3.0  # steered curves: written within this of the solved nozzle path
TIME_PRESERVING_ROUNDS = 3  # time-preserving feeds: rounds of setting the feeds and solving again
MAX_POINTWISE_POINTS = 2_000_000  # pointwise cuts the path coarser than the set spacing beyond this

# Colours of the before / after image, as the ISBF code drew them (RGB)
PROGRAMMED_RGB = (0, 0, 0)
JET_BEFORE_RGB = (150, 150, 150)
JET_AFTER_RGB = (25, 130, 25)


# --- G-code output ------------------------------------------------------------------------

class GcodeWriter:
    # Absolute G-code in mm (y up) from positions in µm with y flipped (the toolpath's frame)

    def __init__(self, scale, start):
        self.scale = scale
        self.lines = ["G90", "G21", "G17"]
        self.x, self.y = start

    def _xy(self, x, y):
        return f"X{x / self.scale:.5f} Y{-y / self.scale:.5f}"

    def g1(self, x, y, feed_mm_min):
        if abs(x - self.x) < 1e-4 and abs(y - self.y) < 1e-4:
            return
        self.lines.append(f"G1 {self._xy(x, y)} F{feed_mm_min:.2f}")
        self.x, self.y = x, y

    def arc(self, x, y, cx, cy, ccw, feed_mm_min):
        # ccw is counter-clockwise as seen in G-code coordinates (G3)
        i = (cx - self.x) / self.scale
        j = -(cy - self.y) / self.scale
        self.lines.append(f"G{3 if ccw else 2} {self._xy(x, y)} I{i:.5f} J{j:.5f} F{feed_mm_min:.2f}")
        self.x, self.y = x, y

    def dwell(self, seconds):
        self.lines.append(f"G4 P{seconds * 1000:.0f}")


def _moves(segments, variables):
    # The toolpath as moves with their geometry, feed (mm/min) and directions
    scale = variables["scale"]
    override = variables["Feedrate_override_mm_min"]
    moves = []
    for index, row in enumerate(segments):
        kind, x1, y1, x2, y2, cx, cy, sweep, line, feed, ocs = row[:11]
        length, u_in, u_out, radius = move_directions(kind, x1, y1, x2, y2, cx, cy, sweep, scale)
        if length <= 0:
            continue
        moves.append({"index": index, "kind": kind, "start": (x1, y1), "end": (x2, y2), "centre": (cx, cy),
                      "sweep": sweep, "feed": override if override > 0 else feed * 60.0, "ocs": int(ocs),
                      "length": length, "u_in": u_in, "u_out": u_out, "radius": radius})
    return moves


def _point_along(move, s_mm, scale):
    # Point s_mm along a move from its start
    x1, y1 = move["start"]
    if move["kind"] == 1:
        u = move["u_in"]
        return x1 + u[0] * s_mm * scale, y1 + u[1] * s_mm * scale
    cx, cy = move["centre"]
    r = move["radius"] * scale
    a = math.atan2(cy - y1, x1 - cx) + (1.0 if move["sweep"] > 0 else -1.0) * s_mm * scale / r
    return cx + r * math.cos(a), cy - r * math.sin(a)


def _emit_to(writer, move, x, y, feed):
    # From the writer's position (on the move) along the move to (x, y)
    if move["kind"] == 1:
        writer.g1(x, y, feed)
    else:
        writer.arc(x, y, move["centre"][0], move["centre"][1], move["sweep"] > 0, feed)


def _turn(prev, curr):
    # Turn between two moves in degrees (0 = straight on, 180 = reversal), and whether they meet
    u_out, u_in = prev["u_out"], curr["u_in"]
    dot = max(-1.0, min(1.0, u_out[0] * u_in[0] + u_out[1] * u_in[1]))
    meets = math.hypot(prev["end"][0] - curr["start"][0], prev["end"][1] - curr["start"][1]) < 0.01
    return math.degrees(math.acos(dot)), meets


# --- simulating a candidate file --------------------------------------------------------------

def _simulate_jet(nx, ny, model, lag0):
    # Jet contact points (µm) for nozzle positions (µm), starting lag0 behind the nozzle
    js, dt, a, b, eps = model
    gx, gy = nx / 1000.0, ny / 1000.0
    moved = np.flatnonzero(np.hypot(gx - gx[0], gy - gy[0]) > 0)
    if len(moved):
        dx, dy = gx[moved[0]] - gx[0], gy[moved[0]] - gy[0]
        norm = math.hypot(dx, dy)
        state = (gx[0] - dx / norm * lag0, gy[0] - dy / norm * lag0, lag0)
    else:
        state = (gx[0], gy[0], 0.0)
    n = len(gx)
    cx, cy, lag = np.empty(n), np.empty(n), np.empty(n)
    cx[0], cy[0], lag[0] = state
    lag_steps(gx[1:], gy[1:], *state, js, dt, a, b, eps, cx[1:], cy[1:], lag[1:])
    return cx * 1000.0, cy * 1000.0, lag


def _positions(cum_counts, t_start, duration, t_acc, t_cruise, v0, peak, length, d_acc, d_dec, is_arc, x1, y1, ux, uy,
               cx, cy, radius, a_start, direction, accel, dt, scale, out_x, out_y, out_m):
    # pixel_coords.sample_timeline's positions (the same points: every multiple of dt, and
    # the very end), for scoring - without the speed / acceleration it also works out
    m = 0
    n_moves = cum_counts.shape[0]
    for k in range(out_x.shape[0]):
        while m < n_moves - 1 and cum_counts[m] <= k:
            m += 1
        tau = min(max(k * dt - t_start[m], 0.0), duration[m])
        if tau < t_acc[m]:
            dist = v0[m] * tau + 0.5 * accel * tau * tau
        elif tau < t_acc[m] + t_cruise[m]:
            dist = d_acc[m] + peak[m] * (tau - t_acc[m])
        else:
            tau_dec = max(tau - t_acc[m] - t_cruise[m], 0.0)
            dist = length[m] - d_dec[m] + peak[m] * tau_dec - 0.5 * accel * tau_dec * tau_dec
        dist = min(max(dist, 0.0), length[m]) * scale
        if is_arc[m]:
            angle = a_start[m] + direction[m] * dist / max(radius[m], 1e-12)
            out_x[k] = cx[m] + radius[m] * math.cos(angle)
            out_y[k] = cy[m] - radius[m] * math.sin(angle)
        else:
            out_x[k] = x1[m] + ux[m] * dist
            out_y[k] = y1[m] + uy[m] * dist
        out_m[k] = m


if njit is not None:
    _positions = njit(cache=True, nogil=True)(_positions)


def _sample_positions(tl, accel, dt, scale):
    cum_counts = np.cumsum(point_counts(tl, dt))
    n = int(cum_counts[-1])
    seg_len_um = np.maximum(tl.length * scale, 1e-12)
    ux = np.where(tl.length > 0, (tl.x2 - tl.x1) / seg_len_um, 0.0)
    uy = np.where(tl.length > 0, (tl.y2 - tl.y1) / seg_len_um, 0.0)
    radius = np.hypot(tl.x1 - tl.cx, tl.y1 - tl.cy)
    a_start = np.arctan2(tl.cy - tl.y1, tl.x1 - tl.cx)
    direction = np.where(tl.sweep > 0, 1.0, -1.0)
    x, y, m = np.empty(n), np.empty(n), np.empty(n, dtype=np.int64)
    f = lambda v: np.ascontiguousarray(v, dtype=np.float64)
    _positions(cum_counts, f(tl.t_start), f(tl.duration), f(tl.t_acc), f(tl.t_cruise), f(tl.v0), f(tl.peak), f(tl.length),
               f(tl.d_acc), f(tl.d_dec), np.ascontiguousarray(tl.kind != 1), f(tl.x1), f(tl.y1), ux, uy, f(tl.cx), f(tl.cy),
               radius, a_start, direction, float(accel), float(dt), float(scale), x, y, m)
    return x, y, m


def _executed_jet(rows, dwells, variables, model_ab, dt):
    # Plan, round the corners and sample the rows exactly as a run of them would, then
    # simulate the jet. Returns the samples (µm), the row each belongs to, and the jet.
    from .corner_path import build_corner_segments
    accel = float(variables["Acceleration_mm_s2"])
    scale = variables["scale"]
    planned, _t, _b = plan_moves(rows, dwells, variables)
    corner_rows, corner_dwells, _dev = build_corner_segments(rows, planned, dwells, variables)
    corner_planned, _t, _b = plan_moves(corner_rows, corner_dwells, variables)
    tl = build_timeline(corner_rows, corner_planned, corner_dwells, accel)
    x, y, m = _sample_positions(tl, accel, dt, scale)
    owner = tl.owner[m]
    js, a, b = model_ab
    first_speed = corner_planned[0][3] if corner_planned else 0.0
    model = (js, dt / 60.0, a, b, max(first_speed * dt / 50.0, 1e-9))
    lag0 = max(objective(first_speed * 60.0 / js, a, b), 0.0) if first_speed > 0 else 0.0
    cx, cy, lag = _simulate_jet(x, y, model, lag0)
    return x, y, owner, cx, cy, lag, float(tl.t_end[-1])


class JetScorer:
    # Runs a candidate file (rows + dwells) through the planner and the lag model and
    # measures how far the jet lands from the programmed path. Scored from the moment the
    # jet first lands on the path in the first run scored (the original file): the start-up
    # tail (the jet starting a lag behind the first point) can't be compensated, and fixing
    # it in time scores every candidate over the same part of the print.

    def __init__(self, params, variables, stride=4):
        # stride: score every stride-th jet point (4 ms apart at the default 1 ms step; the
        # jet itself is still simulated at every step)
        self.stride = stride
        self.variables = variables
        js, _dt, a, b, _eps = params["Lag_model"]
        self.model_ab = (js, a, b)
        self.rx, self.ry, self.cum = reference_polyline(params["Preview_segments"])
        total_time = sum(p[1] / max(p[3], 1e-9) for p in params.get("Planned_moves", [])) or 1.0
        self.dt = max(float(variables["scatter_resolution"]), total_time / MAX_ITERATIVE_POINTS)
        self.start_cut = None

    def __call__(self, rows, dwells):
        x, y, owner, cx, cy, lag, duration = _executed_jet(rows, dwells, self.variables, self.model_ab, self.dt)
        if self.stride > 1:
            x, y, owner, cx, cy, lag = (v[::self.stride] for v in (x, y, owner, cx, cy, lag))
        cx, cy = np.ascontiguousarray(cx), np.ascontiguousarray(cy)
        n = len(x)
        d, qx, qy, s = np.empty(n), np.empty(n), np.empty(n), np.empty(n)
        project(cx, cy, self.rx, self.ry, self.cum, 0, 3000.0, d, qx, qy, s)
        if self.start_cut is None:
            landed = np.flatnonzero(d < 10.0)
            self.start_cut = int(landed[0]) if len(landed) else 0
        scored = d[min(self.start_cut, n - 1):]
        return SimpleNamespace(x=x, y=y, owner=owner, cx=cx, cy=cy, lag=lag, d=d, qx=qx, qy=qy,
                               s=np.maximum.accumulate(s), cumd=np.concatenate(([0.0], np.cumsum(d))), duration=duration,
                               mean=float(np.mean(scored)), rms=float(np.sqrt(np.mean(scored * scored))))


# --- overshoot (ISBF) and pointwise ------------------------------------------------------------

_GCODE_WORD = re.compile(r"([A-Z])\s*([-+]?(?:\d+\.?\d*|\.\d+))")


def rows_from_gcode(lines, scale):
    # Absolute G-code lines (as the compensation methods write them) -> toolpath rows as
    # params["Preview_segments"] (each line its own command) and G4 dwells, so a candidate
    # file can be simulated without writing and re-reading it. Feeds are modal, arcs take
    # I / J from the current position.
    rows, dwells = [], []
    x = y = 0.0
    feed = 0.0
    for line in lines:
        words = dict(_GCODE_WORD.findall(line.split(";")[0].upper()))
        if "G" not in words:
            continue
        g = int(float(words["G"]))
        if g == 4:
            dwells.append((len(rows), float(words.get("P", 0.0)) / 1000.0, 0, 0))
            continue
        if g not in (0, 1, 2, 3):
            continue
        feed = float(words.get("F", feed))
        nx, ny = float(words.get("X", x)), float(words.get("Y", y))
        x1, y1, x2, y2 = x * scale, -y * scale, nx * scale, -ny * scale
        n = len(rows)
        if g in (0, 1):
            if (x1, y1) != (x2, y2):
                rows.append((1, x1, y1, x2, y2, 0.0, 0.0, 0.0, n, feed / 60.0, n))
        else:
            cx, cy = x + float(words.get("I", 0.0)), y + float(words.get("J", 0.0))
            a1 = math.degrees(math.atan2(y - cy, x - cx))
            a2 = math.degrees(math.atan2(ny - cy, nx - cx))
            sweep = (a2 - a1) % 360.0 if g == 3 else -((a1 - a2) % 360.0)
            if sweep == 0.0:
                sweep = 360.0 if g == 3 else -360.0
            rows.append((g, x1, y1, x2, y2, cx * scale, -cy * scale, sweep, n, feed / 60.0, n))
        x, y = nx, ny
    return rows, dwells


def _time_preserving_feeds(lines, owners, base_feed, target_min, rapid, scale):
    # Feeds (mm/min, per program command / piece) that make each command's written moves take
    # the time the command was programmed to take: its moves' length at the feed, less what
    # its rapid swings take. A compensated nozzle path is longer than the path it draws
    # (overshoots; round a curve the lead sits on a bigger circle), and at the programmed feed
    # that extra length makes the print longer, the jet slower and the fibre thicker. Moves
    # only ever speed up, to the rapid speed at most.
    rows, _dwells = rows_from_gcode(lines, scale)
    move_lines = []
    x = y = 0.0
    for i, line in enumerate(lines):
        words = dict(_GCODE_WORD.findall(line.split(";")[0].upper()))
        if "G" not in words or int(float(words["G"])) not in (0, 1, 2, 3):
            continue
        g = int(float(words["G"]))
        nx, ny = float(words.get("X", x)), float(words.get("Y", y))
        if g in (0, 1) and (nx, ny) == (x, y):
            continue  # rows_from_gcode drops these
        move_lines.append(i)
        x, y = nx, ny
    n = len(base_feed)
    slow_len, rapid_time = np.zeros(n), np.zeros(n)
    for row, i in zip(rows, move_lines):
        owner = owners[i]
        if owner < 0 or owner >= n:
            continue
        kind, x1, y1, x2, y2, cx, cy, sweep = row[:8]
        length = (math.hypot(x2 - x1, y2 - y1) if kind == 1 else math.hypot(x1 - cx, y1 - cy) * math.radians(abs(sweep))) / scale
        if row[9] * 60.0 >= rapid - 1e-6:
            rapid_time[owner] += length / rapid
        else:
            slow_len[owner] += length
    left = np.maximum(target_min - rapid_time, 1e-9)
    return np.clip(np.where(slow_len > 0, slow_len / left, base_feed), base_feed, rapid)


def true_arcs(lines, tolerance_mm=0.0005):
    # Every G2 / G3 whose start and end are at different distances from its centre (ISBF's
    # shortened arcs, whose centre is moved by half the change) rewritten as a true arc: the
    # same start, end and direction, the radius the mean of the two, the centre on the same
    # side of the chord as it was. A controller that checks arcs (GRBL) stops on the others.
    out = []
    x = y = None
    for line in lines:
        words = dict(_GCODE_WORD.findall(line.split(";")[0].upper()))
        g = int(float(words["G"])) if "G" in words else None
        if g not in (0, 1, 2, 3) or not any(k in words for k in ("X", "Y")):
            out.append(line)
            continue
        nx, ny = float(words.get("X", x or 0.0)), float(words.get("Y", y or 0.0))
        if g in (2, 3) and x is not None:
            cx, cy = x + float(words.get("I", 0.0)), y + float(words.get("J", 0.0))
            r0, r1 = math.hypot(x - cx, y - cy), math.hypot(nx - cx, ny - cy)
            chord = math.hypot(nx - x, ny - y)
            if abs(r0 - r1) > tolerance_mm and chord > 1e-9:
                radius = max(0.5 * (r0 + r1), 0.5 * chord)
                mx, my = 0.5 * (x + nx), 0.5 * (y + ny)
                px, py = -(ny - y) / chord, (nx - x) / chord  # left of the chord
                side = 1.0 if (cx - mx) * px + (cy - my) * py >= 0 else -1.0
                h = math.sqrt(max(radius * radius - 0.25 * chord * chord, 0.0))
                ncx, ncy = mx + side * h * px, my + side * h * py
                feed = f" F{words['F']}" if "F" in words else ""
                line = f"G{g} X{nx:.5f} Y{ny:.5f} I{ncx - x:.5f} J{ncy - y:.5f}{feed}"
        out.append(line)
        x, y = nx, ny
    return out


def thin_lines(lines, tolerance_mm, feed_step, max_feed_jump=1.25):
    # Runs of G1 moves (the steered curves come out as a point every nozzle sample or so)
    # thinned to the fewest that stay within the tolerance of the written path (Douglas -
    # Peucker). A merged move gets the feed that keeps the time its moves took, in steps of
    # feed_step and never below the slowest of them. A run ends at anything that isn't a G1
    # move and where the feed jumps (a rapid).
    out = []
    run = []  # (x, y, feed) of consecutive G1 moves
    start = [None]
    pos = [None, None]
    feed = [0.0]

    def flush():
        if not run:
            return
        if len(run) < 3 or start[0] is None:
            out.extend(f"G1 X{x:.5f} Y{y:.5f} F{f:.2f}" for x, y, f in run)
            run.clear()
            return
        pts = np.array([start[0]] + [(x, y) for x, y, _f in run])
        feeds = np.array([f for _x, _y, f in run])
        seg = np.hypot(np.diff(pts[:, 0]), np.diff(pts[:, 1]))
        time = np.concatenate(([0.0], np.cumsum(seg / np.maximum(feeds, 1e-9))))
        dist = np.concatenate(([0.0], np.cumsum(seg)))
        keep = np.zeros(len(pts), dtype=bool)
        keep[0] = keep[-1] = True
        stack = [(0, len(pts) - 1)]
        while stack:
            i, j = stack.pop()
            if j <= i + 1:
                continue
            ax, ay = pts[i]
            dx, dy = pts[j] - pts[i]
            span = dx * dx + dy * dy
            px, py = pts[i + 1:j, 0] - ax, pts[i + 1:j, 1] - ay
            t = np.clip((px * dx + py * dy) / span, 0.0, 1.0) if span > 0 else np.zeros(len(px))
            d = np.hypot(px - t * dx, py - t * dy)
            k = int(np.argmax(d))
            if d[k] > tolerance_mm:
                keep[i + 1 + k] = True
                stack.append((i, i + 1 + k))
                stack.append((i + 1 + k, j))
        idx = np.flatnonzero(keep)
        for i, j in zip(idx[:-1], idx[1:]):
            f = feeds[i:j].min()
            if time[j] > time[i] and j > i + 1:
                f = max(f, feed_step * math.floor((dist[j] - dist[i]) / (time[j] - time[i]) / feed_step + 0.5))
            out.append(f"G1 X{pts[j, 0]:.5f} Y{pts[j, 1]:.5f} F{f:.2f}")
        run.clear()

    for line in lines:
        words = dict(_GCODE_WORD.findall(line.split(";")[0].upper()))
        g = int(float(words["G"])) if "G" in words else None
        moves = g in (0, 1, 2, 3) and any(k in words for k in ("X", "Y"))
        if moves:
            new_feed = float(words.get("F", feed[0]))
            nx, ny = float(words.get("X", pos[0] or 0.0)), float(words.get("Y", pos[1] or 0.0))
            if g == 1 and pos[0] is not None:
                last = run[-1][2] if run else new_feed
                if run and not (last / max_feed_jump <= new_feed <= last * max_feed_jump):
                    flush()
                if not run:
                    start[0] = (pos[0], pos[1])
                run.append((nx, ny, new_feed))
            else:
                flush()
                out.append(line)
            pos[0], pos[1], feed[0] = nx, ny, new_feed
        else:
            flush()
            out.append(line)
    flush()
    return out


def blend_kinks(lines, radius_mm, max_turn_deg=20.0, arcs=True):
    # Smooth swings: every sharp change of direction in the written path (an overshoot line
    # meeting its swing arc, a swing arc meeting the next line) replaced by a short blend - one
    # fillet arc (G2 / G3), or with arcs=False a few G1 chords turning ~15 degrees each. A
    # planner has to brake for a sharp kink (at jerk 5 mm/s a 90 degree one is taken at about
    # 3.5 mm/s) and, with the nozzle slower than programmed, the fibre thickens; through the
    # blend it keeps the feed. The blend stays within radius x (sqrt(2) - 1) of a 90 degree
    # kink (0.06 mm at 0.15 mm).
    items = []  # motion: [g, x0, y0, x1, y1, cx, cy, feed]; anything else: the line itself
    x = y = 0.0
    feed = 0.0
    for line in lines:
        words = dict(_GCODE_WORD.findall(line.split(";")[0].upper()))
        g = int(float(words["G"])) if "G" in words else None
        if g not in (0, 1, 2, 3) or not any(k in words for k in ("X", "Y")):
            items.append(line)
            continue
        feed = float(words.get("F", feed))
        nx, ny = float(words.get("X", x)), float(words.get("Y", y))
        if g in (0, 1) and abs(nx - x) < 1e-9 and abs(ny - y) < 1e-9:
            continue  # a G1 to where the nozzle already is only sets the feed (kept: it is modal here)
        cx, cy = (x + float(words.get("I", 0.0)), y + float(words.get("J", 0.0))) if g in (2, 3) else (0.0, 0.0)
        items.append([1 if g in (0, 1) else g, x, y, nx, ny, cx, cy, feed])
        x, y = nx, ny

    def length(m):
        g, x0, y0, x1, y1, cx, cy, _f = m
        if g == 1:
            return math.hypot(x1 - x0, y1 - y0)
        r = math.hypot(x0 - cx, y0 - cy)
        a0, a1 = math.atan2(y0 - cy, x0 - cx), math.atan2(y1 - cy, x1 - cx)
        sweep = (a1 - a0) % (2 * math.pi) if g == 3 else (a0 - a1) % (2 * math.pi)
        return r * (sweep if sweep > 1e-12 else 2 * math.pi)

    def tangent(m, at_end):
        g, x0, y0, x1, y1, cx, cy, _f = m
        if g == 1:
            d = math.hypot(x1 - x0, y1 - y0)
            return ((x1 - x0) / d, (y1 - y0) / d) if d > 0 else (0.0, 0.0)
        px, py = (x1, y1) if at_end else (x0, y0)
        r = math.hypot(px - cx, py - cy) or 1.0
        rx, ry = (px - cx) / r, (py - cy) / r
        return (-ry, rx) if g == 3 else (ry, -rx)

    def along(m, dist, from_end):
        # the point `dist` along the move from its start, or back from its end
        g, x0, y0, x1, y1, cx, cy, _f = m
        if g == 1:
            d = math.hypot(x1 - x0, y1 - y0) or 1.0
            t = (1.0 - dist / d) if from_end else dist / d
            return x0 + (x1 - x0) * t, y0 + (y1 - y0) * t
        r = math.hypot(x0 - cx, y0 - cy) or 1.0
        px, py = (x1, y1) if from_end else (x0, y0)
        sign = (1.0 if g == 3 else -1.0) * (-1.0 if from_end else 1.0)
        a = math.atan2(py - cy, px - cx) + sign * dist / r
        return cx + r * math.cos(a), cy + r * math.sin(a)

    moves = [i for i, m in enumerate(items) if isinstance(m, list)]
    lengths = {i: length(items[i]) for i in moves}
    blends = {}
    for a, b in zip(moves[:-1], moves[1:]):
        if b != a + 1 or lengths[a] <= 1e-9 or lengths[b] <= 1e-9:
            continue  # something between them (a dwell), or nothing to blend
        ta, tb = tangent(items[a], True), tangent(items[b], False)
        turn = math.degrees(math.acos(max(-1.0, min(1.0, ta[0] * tb[0] + ta[1] * tb[1]))))
        if turn <= max_turn_deg or turn >= 170.0:
            continue
        t = min(radius_mm * math.tan(math.radians(turn) / 2.0), 0.45 * lengths[a], 0.45 * lengths[b])
        if t <= 1e-5:
            continue
        jx, jy = items[a][3], items[a][4]
        pa, pb = along(items[a], t, True), along(items[b], t, False)
        n = int(math.ceil(turn / 15.0)) + 1
        pts = []
        for k in range(1, n + 1):
            u = k / n
            pts.append(((1 - u) ** 2 * pa[0] + 2 * u * (1 - u) * jx + u * u * pb[0],
                        (1 - u) ** 2 * pa[1] + 2 * u * (1 - u) * jy + u * u * pb[1]))
        fillet = None
        if arcs:
            # the circle leaving pa along the first move's direction and reaching pb
            ta2 = tangent(items[a], True) if items[a][0] == 1 else None
            if ta2 is None:
                # an arc shortened by t: its direction at pa
                g_, _x0, _y0, _x1, _y1, cxa, cya, _f = items[a]
                ra = math.hypot(pa[0] - cxa, pa[1] - cya) or 1.0
                rx, ry = (pa[0] - cxa) / ra, (pa[1] - cya) / ra
                ta2 = (-ry, rx) if g_ == 3 else (ry, -rx)
            chord = (pb[0] - pa[0], pb[1] - pa[1])
            across = chord[0] * -ta2[1] + chord[1] * ta2[0]  # chord . left normal
            if abs(across) > 1e-9:
                radius = (chord[0] ** 2 + chord[1] ** 2) / (2.0 * across)
                fillet = (3 if radius > 0 else 2, pa[0] - ta2[1] * radius, pa[1] + ta2[0] * radius)
        blends[a] = (pa, pts, min(items[a][7], items[b][7]), fillet)
    if not blends:
        return lines
    for a, (pa, pts, _f, _fillet) in blends.items():
        items[a][3], items[a][4] = pa
        items[a + 1][1], items[a + 1][2] = pts[-1]
    out = []
    for i, m in enumerate(items):
        if not isinstance(m, list):
            out.append(m)
            continue
        g, x0, y0, x1, y1, cx, cy, f = m
        if g == 1:
            out.append(f"G1 X{x1:.5f} Y{y1:.5f} F{f:.2f}")
        else:
            out.append(f"G{g} X{x1:.5f} Y{y1:.5f} I{cx - x0:.5f} J{cy - y0:.5f} F{f:.2f}")
        if i in blends:
            pa, pts, bf, fillet = blends[i]
            if fillet is not None:
                out.append(f"G{fillet[0]} X{pts[-1][0]:.5f} Y{pts[-1][1]:.5f} I{fillet[1] - pa[0]:.5f} J{fillet[2] - pa[1]:.5f} F{bf:.2f}")
            else:
                out.extend(f"G1 X{px:.5f} Y{py:.5f} F{bf:.2f}" for px, py in pts)
    return out


def _arcs_as_chords(lines, max_turn_deg, sag_mm):
    # The unlooped commands with every G2 / G3 written as short G1 chords (each turning no
    # more than max_turn_deg and within sag_mm of the arc), for the adaptive spacing to steer
    # round. Returns the new lines and, for each, the command it came from.
    from .isbf import _feed_of, _fmt, _tokens_cached
    out, owner = [], []
    for index, line in enumerate(lines):
        t = _tokens_cached(line)
        if t[-1] in ("0", "1"):
            out.append(line)
            owner.append(index)
            continue
        x1, y1 = float(t[3]), float(t[5])
        x0, y0, cx, cy = float(t[12]), float(t[13]), float(t[14]), float(t[15])
        r = math.hypot(x0 - cx, y0 - cy)
        a0, a1 = math.atan2(y0 - cy, x0 - cx), math.atan2(y1 - cy, x1 - cx)
        sweep = (a1 - a0) % (2 * math.pi) if t[-1] == "3" else -((a0 - a1) % (2 * math.pi))
        if sweep == 0:
            sweep = 2 * math.pi if t[-1] == "3" else -2 * math.pi
        step = math.radians(max_turn_deg)
        if r > sag_mm:
            step = min(step, 2.0 * math.acos(1.0 - sag_mm / r))
        n = max(1, int(math.ceil(abs(sweep) / step)))
        feed = _fmt(_feed_of(t))
        px, py = x0, y0
        for k in range(1, n + 1):
            ang = a0 + sweep * k / n
            nx, ny = (x1, y1) if k == n else (round(cx + r * math.cos(ang), 6), round(cy + r * math.sin(ang), 6))
            out.append(f"G1 X{_fmt(nx)} Y{_fmt(ny)} F{feed} ; {_fmt(px)} {_fmt(py)} 1")
            owner.append(index)
            px, py = nx, ny
    return out, owner


def _command_spans(params):
    # Where each command of the program starts and ends along the programmed path (µm)
    ends = reference_polyline(params["Preview_segments"], with_ends=True)[3]
    spans = {}
    for row_index, row in enumerate(params["Preview_segments"]):
        kind, x1, y1, x2, y2, cx, cy, sweep = row[:8]
        if kind == 1:
            length = math.hypot(x2 - x1, y2 - y1)
        else:
            length = math.hypot(x1 - cx, y1 - cy) * math.radians(abs(sweep))
        spans[int(row[10])] = (ends[row_index] - length, ends[row_index])
    return spans


def _local_mean(res, s0, s1):
    # Mean distance of the jet from the path over the jet points landing between s0 and s1
    lo, hi = np.searchsorted(res.s, (s0, s1))
    if hi <= lo:
        return None
    return float((res.cumd[hi] - res.cumd[lo]) / (hi - lo))


def _isbf_with_passes(params, variables, score, baseline, label, split_mm=None, mode="isbf", graded=False):
    # vector_angle (isbf.py) on the program - on its pieces for the pointwise method's fixed
    # spacing, or in its continuous mode for the adaptive spacing - then correction passes
    # with the planner and the lag model in the loop. Every join (G1 corner overshoot, arc
    # reset distance) is tuned on its own by a line search on how far the jet lands from
    # the path around that join: step, keep the step if the jet lands closer there,
    # otherwise reverse it (and halve it once both ways have been tried). Each pass
    # simulates the whole compensated file once with every join's step in it. The file with
    # the jet closest to the path overall is kept.
    from .isbf import Program, vector_angle
    scale = variables["scale"]
    js, _dt, a, b, _eps = params["Lag_model"]
    factor = float(variables["Lag_comp_overshoot_scale"])
    rapid = float(variables["Lag_comp_rapid_mm_min"])
    iterations = int(variables["Lag_comp_iterations"])
    commands = params["One_coordinate_system"]
    dwells = params.get("Dwells", [])
    chord_owner = None
    if mode == "continuous" and variables.get("Lag_comp_adaptive_arcs", "isbf") == "chords" and not isinstance(commands, Program):
        # Option: arcs steered round like any other curve, as chords, instead of ISBF's arc joins (which
        # do better on the files tried, in a fraction of the lines)
        commands, chord_owner = _arcs_as_chords(commands, ARC_CHORD_DEG, ARC_CHORD_SAG_UM / 1000.0)
        first_chord = {}
        for i, c in enumerate(chord_owner):
            first_chord.setdefault(c, i)
        dwells = [(d[0], d[1], first_chord.get(int(d[2]), len(commands))) + tuple(d[3:]) for d in dwells]
    program = Program(commands)
    owner = list(range(len(program)))
    if split_mm is not None or graded:
        # Every G1 cut into points (split_mm apart, or graded), each treated as a command
        program, owner_arr, first = program.split(split_mm, dt_s=float(variables["scatter_resolution"]))
        owner = owner_arr.tolist()
        dwells = [(d[0], d[1], int(first[int(d[2])]) if int(d[2]) < len(first) else len(program)) + tuple(d[3:])
                  for d in dwells]
    if chord_owner is not None:
        owner = [chord_owner[i] for i in owner]
    source = dict(params, One_coordinate_system=program, Dwells=dwells)
    solved = {}

    # Options for the adaptive spacing (all off unless set): its own lead factor, the smallest
    # turn it swings round, swings no smaller than the fibre diameter limit's lag, and
    # time-preserving feeds
    swing_min, min_overshoot, steer = 5.0, 0.0, False
    if mode == "continuous":
        factor = float(variables.get("Lag_comp_adaptive_lead", factor))
        swing_min = float(variables.get("Lag_comp_adaptive_swing_deg", 30.0))
        steer = bool(variables.get("Lag_comp_adaptive_steer", True))
        limit = float(variables.get("Lag_comp_diameter_limit_pct", 0.0))
        if variables.get("Lag_comp_adaptive_hold_swing", False) and limit > 0:
            speeds = [row[9] * 60.0 for row in params["Preview_segments"]]
            ratio = (float(np.median(speeds)) if speeds else js) / js
            min_overshoot = max(float(objective(ratio / (1.0 + limit / 100.0) ** 2, a, b)), 0.0)
    time_preserving = bool(variables.get("Lag_comp_time_preserving", False))
    base_feed = np.array(program.feed, dtype=np.float64)
    # Smooth swings: the radius the kinks are blended at. -1 = automatic: for the pointwise
    # method the smallest radius the machine can take at the feed without braking,
    # v^2 / (BLEND_ACCEL_FRACTION x acceleration); off for the ISBF method, whose output is
    # then as the original code wrote it. 0 = off.
    blend_um = float(variables.get("Lag_comp_blend_um", -1.0))
    if blend_um < 0:
        blend_mm = 0.0
        accel = float(variables.get("Acceleration_mm_s2", 0.0))
        if label == "pointwise" and accel > 0 and len(base_feed):
            blend_mm = (float(np.max(base_feed)) / 60.0) ** 2 / (BLEND_ACCEL_FRACTION * accel)
    else:
        blend_mm = blend_um / 1000.0
    is_line = np.asarray(program.is_g1)
    piece_mm = np.hypot(program.x - program.px, program.y - program.py)
    if (~is_line).any():
        # an arc's own length: from its row in the toolpath
        arc_len = {int(row[10]): math.hypot(row[1] - row[5], row[2] - row[6]) * math.radians(abs(row[7])) / scale
                   for row in params["Preview_segments"] if row[0] != 1}
        for i in np.flatnonzero(~is_line):
            piece_mm[i] = arc_len.get(owner[i], piece_mm[i])
    target_min = piece_mm / np.maximum(base_feed, 1e-9)
    feeds = [None]

    lead = [factor]
    fixes = bool(variables.get("Lag_comp_isbf_fixes", True))
    # With the swings blended the nozzle doesn't brake into or out of them: the corner solve
    # is told so (junctions passed at the move's own speed)
    solve_variables = variables
    if blend_mm > 0 and variables.get("Lag_comp_blend_solve", True):
        solve_variables = dict(variables, Junction_deviation_mm=1e6, Jerk_mm_s=0.0)

    def solve(extra, arc_extra, feed):
        return vector_angle(source, solve_variables, js, a, b, factor=lead[0], rapid=rapid, extra=extra, arc_extra=arc_extra,
                            mode=mode, fixed=solved, merge=split_mm is not None or graded, swing_min=swing_min,
                            min_overshoot=min_overshoot, feeds=feed, with_owner=True, steer=steer, fixes=fixes)

    def run(extra, arc_extra):
        lines_, corners_, arcs_, owners_ = solve(extra, arc_extra, feeds[0])
        if time_preserving and feeds[0] is None:
            # Solve, set the feeds that keep each command's time, and solve again with them
            # (the geometry depends on the lag, and the lag on the speed), a few times
            feed = base_feed
            for _round in range(TIME_PRESERVING_ROUNDS):
                new = _time_preserving_feeds(lines_, owners_, base_feed, target_min, rapid, 1.0)
                feed = 0.5 * (feed + new)
                lines_, corners_, arcs_, owners_ = solve(extra, arc_extra, feed)
            feeds[0] = feed  # the passes keep these feeds
        if fixes:
            lines_ = true_arcs(lines_)
        if steer:
            lines_ = thin_lines(lines_, float(variables.get("Lag_comp_thin_um", THIN_UM)) / 1000.0,
                                0.05 * float(np.max(base_feed)))
        if blend_mm > 0:
            lines_ = blend_kinks(lines_, blend_mm, arcs=bool(variables.get("Lag_comp_blend_arcs", True)))
        return lines_, corners_, arcs_

    lines, corners, arcs = run({}, {})
    if steer and iterations > 0 and "Lag_comp_adaptive_lead" not in variables:
        # Steered curves: how far ahead the nozzle is taken to be decides how early it starts
        # across for each turn, and the best value depends on the curve - found here on the
        # whole file (a few steps either way, halving), if it has gentle turns at all
        dx, dy = program.x - program.px, program.y - program.py
        turn = np.degrees(np.abs(np.arctan2(dx[:-1] * dy[1:] - dy[:-1] * dx[1:], dx[:-1] * dx[1:] + dy[:-1] * dy[1:])))
        gentle = is_line[:-1] & is_line[1:] & (turn > 1.0) & (turn <= swing_min)
        if gentle.sum() >= 10:
            best = score(*rows_from_gcode(lines, scale)).mean
            step = 0.08
            for _round in range(3):
                moved = False
                for trial in (lead[0] + step, lead[0] - step):
                    if not 0.4 <= trial <= 1.3:
                        continue
                    previous, lead[0] = lead[0], trial
                    t_lines, t_corners, t_arcs = run({}, {})
                    t_mean = score(*rows_from_gcode(t_lines, scale)).mean
                    if t_mean < best:
                        best, lines, corners, arcs, moved = t_mean, t_lines, t_corners, t_arcs, True
                        break
                    lead[0] = previous
                if not moved:
                    step *= 0.5
            print(f"Lag compensation ({label}): steered curves, lead = lag x", round(lead[0], 3), "- jet off the path by mean",
                  round(best, 2), "um")
    if mode == "continuous":
        # The passes tune on top of the first solve rather than solving every corner again
        solved.update({c[0]: c[4] for c in corners})
    print(f"Lag compensation ({label}): ISBF vector_angle ({mode}) on", len(source["One_coordinate_system"]), "commands,",
          len(corners), "corners,", len(arcs), "arc joins, overshoot = lag x", factor, ", swings at", rapid, "mm/min,",
          iterations, "correction passes")
    if iterations <= 0 or not (corners or arcs):
        return lines
    spans = _command_spans(params)
    last = len(owner) - 1

    def join_error(j, res):
        # Around a join: the command before it and the one after it
        first, second = spans.get(owner[j]), spans.get(owner[min(j + 1, last)])
        if first is None:
            return None
        return _local_mean(res, first[0], (second or first)[1])

    res = score(*rows_from_gcode(lines, scale))
    print(f"Lag compensation ({label}): uncompensated jet off the path by mean", round(baseline.mean, 2), "um; compensated",
          round(res.mean, 2), "um, rms", round(res.rms, 2), "um (after the jet first lands)")
    best_mean, best_lines = res.mean, lines
    # Per join: which table it adjusts, current value, error, step (mm), direction
    joins = {}
    for c in [c[0] for c in corners]:
        joins[("corner", c)] = [0.0, join_error(c, res), 0.1, 1.0]
    for j in arcs:
        joins[("arc", j)] = [0.0, join_error(j, res), 0.05, 1.0]
    joins = {key: v for key, v in joins.items() if v[1] is not None}
    for k in range(iterations):
        extra = {c: v[0] + v[3] * v[2] for (kind, c), v in joins.items() if kind == "corner"}
        arc_extra = {j: v[0] + v[3] * v[2] for (kind, j), v in joins.items() if kind == "arc"}
        c_lines, _c_corners, _c_arcs = run(extra, arc_extra)
        c_res = score(*rows_from_gcode(c_lines, scale))
        if c_res.mean < best_mean:
            best_mean, best_lines = c_res.mean, c_lines
        kept = 0
        for (kind, j), v in joins.items():
            err = join_error(j, c_res)
            if err is not None and err < v[1]:
                v[0] += v[3] * v[2]
                v[1] = err
                kept += 1
            else:
                v[3] = -v[3]
                if v[3] > 0:
                    v[2] *= 0.5  # tried both ways: finer steps
        print("Lag compensation progress:", str(int(100 * (k + 1) / iterations)) + "%")
        print(f"Lag compensation ({label}): pass", k + 1, "jet off the path by mean", round(c_res.mean, 2), "um, rms",
              round(c_res.rms, 2), f"um; closer at {kept}/{len(joins)} joins")
    # The joins' kept values together
    extra = {c: v[0] for (kind, c), v in joins.items() if kind == "corner"}
    arc_extra = {j: v[0] for (kind, j), v in joins.items() if kind == "arc"}
    f_lines = run(extra, arc_extra)[0]
    f_res = score(*rows_from_gcode(f_lines, scale))
    if f_res.mean < best_mean:
        best_mean, best_lines = f_res.mean, f_lines
    print(f"Lag compensation ({label}): kept values together mean", round(f_res.mean, 2), "um; using the best, mean",
          round(best_mean, 2), "um")
    return best_lines


def compensate_overshoot(params, variables, score, baseline):
    return _isbf_with_passes(params, variables, score, baseline, "overshoot")


def compensate_pointwise(params, variables, score, baseline):
    # Point spacing > 0: every line cut into points that far apart, each compensated as
    # vector_angle compensates a command. Point spacing 0 = adaptive: points one nozzle
    # sample apart at each end of a line, growing towards the middle (Program.split), run in
    # the continuous mode - the jet model follows the nozzle as written and each corner's
    # overshoot is solved on the lag model with the planner's speeds.
    spacing_um = float(variables["Lag_comp_point_spacing_um"])
    if spacing_um <= 0:
        return _isbf_with_passes(params, variables, score, baseline, "pointwise", graded=True, mode="continuous")
    spacing = spacing_um / variables["scale"]  # mm
    total = sum(m["length"] for m in _moves(params["Preview_segments"], variables))
    if total / spacing > MAX_POINTWISE_POINTS:
        spacing = total / MAX_POINTWISE_POINTS
        print("Lag compensation (pointwise): point spacing raised to", round(spacing * 1000, 2), "um for a path this long")
    return _isbf_with_passes(params, variables, score, baseline, "pointwise", split_mm=spacing)


# --- slow-down ---------------------------------------------------------------------------------

def catch_up_distance(model, lag0, v_slow_mm_min, tolerance_mm, limit_mm=50.0):
    # Distance (mm) the nozzle has to travel in a straight line at v_slow for the jet lag to
    # fall from lag0 to the tolerance, from the lag model
    js, dt, a, b, eps = model
    step = v_slow_mm_min * dt
    if step <= 0:
        return limit_mm
    n = 20000
    state = (-lag0, 0.0, lag0)
    travelled = 0
    out = np.empty(n), np.empty(n), np.empty(n)
    while travelled * step < limit_mm:
        gx = (np.arange(1, n + 1) + travelled) * step
        state = lag_steps(gx, np.zeros(n), *state, js, dt, a, b, eps, *out)
        below = np.flatnonzero(out[2] <= tolerance_mm)
        if len(below):
            return (travelled + below[0] + 1) * step
        travelled += n
    return limit_mm


def compensate_slowdown(params, variables, score=None, baseline=None):
    scale = variables["scale"]
    model = params["Lag_model"]
    cts = model[0]
    v_slow = float(variables["Lag_comp_slow_ratio"]) * cts
    tolerance = float(variables["Lag_comp_tolerance_mm"])
    lag_end = params["Lag_at_command_end"]
    moves = _moves(params["Preview_segments"], variables)
    dwells = sorted(params.get("Dwells", []), key=lambda d: d[0])
    writer = GcodeWriter(scale, (0.0, 0.0))
    cache = {}
    corners, slowed_mm = 0, 0.0
    for i, move in enumerate(moves):
        while dwells and dwells[0][0] <= move["index"]:
            writer.dwell(dwells.pop(0)[1])
        writer.g1(*move["start"], move["feed"])
        nxt = moves[i + 1] if i + 1 < len(moves) else None
        distance = 0.0
        if nxt is not None and v_slow < move["feed"]:
            turn, meets = _turn(move, nxt)
            lag = float(lag_end[move["ocs"]])
            if meets and turn > CORNER_MIN_DEG and lag > tolerance:
                key = round(lag, 4)
                if key not in cache:
                    cache[key] = catch_up_distance(model, lag, v_slow, tolerance)
                distance = min(cache[key], move["length"])
        if distance > 0:
            _emit_to(writer, move, *_point_along(move, move["length"] - distance, scale), move["feed"])
            _emit_to(writer, move, *move["end"], v_slow)
            corners += 1
            slowed_mm += distance
        else:
            _emit_to(writer, move, *move["end"], move["feed"])
    for d in dwells:
        writer.dwell(d[1])
    print("Lag compensation (slowdown):", corners, "corners slowed to", round(v_slow, 1), "mm/min over",
          round(slowed_mm, 2), "mm in total, jet within", tolerance, "mm at each corner")
    return writer.lines


# --- iterative ---------------------------------------------------------------------------------

def _dense_polyline(params, variables, lag_max):
    # The programmed path as a polyline the iterative method can bend: vertices every
    # `spacing` within a few lags of every corner and all along arcs (where the jet strays),
    # the straight runs in between left as single segments. Returns the vertices (µm), and
    # per segment its feed (mm/s) and G-code line, plus the dwells re-indexed to segments.
    scale = variables["scale"]
    moves = _moves(params["Preview_segments"], variables)
    spacing = max(lag_max / 40.0, 0.005)  # mm
    reach = 4.0 * lag_max + 0.5  # mm either side of a corner
    corner_after = [False] * len(moves)
    for i in range(len(moves) - 1):
        turn, meets = _turn(moves[i], moves[i + 1])
        corner_after[i] = (not meets) or turn > CORNER_MIN_DEG
    if not moves:
        return np.zeros((1, 2)), np.zeros(0), np.zeros(0), []
    vertices = [moves[0]["start"]]
    feeds, lines, first_seg = [], [], []
    for i, move in enumerate(moves):
        first_seg.append(len(feeds))
        length = move["length"]
        if move["kind"] != 1:
            stops = np.linspace(0.0, length, max(2, int(math.ceil(length / max(spacing * 4, 0.02))) + 1))[1:]
        else:
            near_start = i > 0 and corner_after[i - 1]
            near_end = corner_after[i] if i < len(moves) - 1 else False
            stops = set([length])
            if near_start:
                stops.update(np.arange(spacing, min(reach, length), spacing).tolist())
            if near_end:
                stops.update((length - np.arange(spacing, min(reach, length), spacing)).tolist())
            stops = sorted(s for s in stops if 0.0 < s <= length)
        if math.hypot(vertices[-1][0] - move["start"][0], vertices[-1][1] - move["start"][1]) > 0.01:
            vertices.append(move["start"])
            feeds.append(move["feed"] / 60.0)
            lines.append(params["Preview_segments"][move["index"]][8])
        for s_mm in stops:
            vertices.append(_point_along(move, s_mm, scale))
            feeds.append(move["feed"] / 60.0)
            lines.append(params["Preview_segments"][move["index"]][8])
    index_of_move = {move["index"]: k for k, move in enumerate(moves)}
    dwells = []
    for d in params.get("Dwells", []):
        k = next((index_of_move[j] for j in sorted(index_of_move) if j >= d[0]), None)
        dwells.append((first_seg[k] if k is not None else len(feeds),) + tuple(d[1:]))
    return np.asarray(vertices, dtype=np.float64), np.asarray(feeds), np.asarray(lines), dwells


def compensate_iterative(params, variables, score, baseline):
    # Iterative learning control on the nozzle path, with the real planner in the loop:
    # every pass plans the current path (acceleration, junction deviation, jerk, corner
    # rounding), simulates the jet on it and moves each vertex by the error of the jet
    # points it caused - those one lag's travel later - smoothed along the path.
    scale = variables["scale"]
    lag_max = float(params.get("Lag_range", (0.0, 1.0))[1]) or 1.0
    vertices, feeds, lines, dwells = _dense_polyline(params, variables, lag_max)
    score.stride = 1  # its corrections come from every jet point
    n_vert = len(vertices)

    def evaluate(v):
        rows = [(1, v[i, 0], v[i, 1], v[i + 1, 0], v[i + 1, 1], 0.0, 0.0, 0.0, lines[i], feeds[i], i) for i in range(len(feeds))]
        res = score(rows, dwells)
        return (res.x, res.y, res.owner, res.qx - res.cx, res.qy - res.cy, res.lag), res.rms, res.mean, res.duration

    state, rms, mean, duration = evaluate(vertices)
    print("Lag compensation (iterative):", n_vert, "path points,", len(state[0]), "samples every", round(score.dt * 1000, 3),
          "ms; start: jet off the path by mean", round(mean, 2), "um, rms", round(rms, 2), "um (after the jet first lands)")
    gain = 0.8
    iterations = int(variables["Lag_comp_iterations"])
    for k in range(iterations):
        x, y, owner, ex, ey, lag = state
        n = len(x)
        # The nozzle sample that put the jet at step i: one lag's travel earlier
        step = np.hypot(np.diff(x, append=x[-1]), np.diff(y, append=y[-1]))
        shift = np.round(np.nan_to_num(lag) * scale / np.maximum(step, 1e-6)).astype(np.int64)
        source = np.clip(np.arange(n) - np.minimum(shift, n), 0, n - 1)
        # Correction per nozzle sample, smoothed along the path (Gaussian, a third of the
        # largest lag): corrections that change sharply between neighbouring points would
        # put small zig-zags in the path, which the planner then slows down for
        count = np.bincount(source, minlength=n).astype(np.float64)
        sum_x = np.bincount(source, weights=ex, minlength=n)
        sum_y = np.bincount(source, weights=ey, minlength=n)
        moving = step[step > 0]
        sigma = max(lag_max * scale / 3.0, 100.0) / (float(np.median(moving)) if len(moving) else 1.0)
        half = int(min(max(3 * sigma, 1), n))
        kernel = np.exp(-0.5 * (np.arange(-half, half + 1) / max(sigma, 1e-9)) ** 2)
        weight_s = np.convolve(count, kernel, mode="same")
        corr_sx = np.convolve(sum_x, kernel, mode="same") / np.maximum(weight_s, 1e-9)
        corr_sy = np.convolve(sum_y, kernel, mode="same") / np.maximum(weight_s, 1e-9)
        # ... then onto the path vertices either side of each sample, weighted by position
        seg = np.clip(owner, 0, n_vert - 2)
        ax, ay = vertices[seg, 0], vertices[seg, 1]
        bx, by = vertices[seg + 1, 0], vertices[seg + 1, 1]
        seg_len = np.hypot(bx - ax, by - ay)
        f = np.clip(np.hypot(x - ax, y - ay) / np.maximum(seg_len, 1e-9), 0.0, 1.0)
        weight = np.bincount(seg, weights=1 - f, minlength=n_vert) + np.bincount(seg + 1, weights=f, minlength=n_vert)
        corr_x = np.bincount(seg, weights=(1 - f) * corr_sx, minlength=n_vert) + np.bincount(seg + 1, weights=f * corr_sx, minlength=n_vert)
        corr_y = np.bincount(seg, weights=(1 - f) * corr_sy, minlength=n_vert) + np.bincount(seg + 1, weights=f * corr_sy, minlength=n_vert)
        corr = np.column_stack((corr_x, corr_y)) / np.maximum(weight, 1e-9)[:, None]
        corr[0] = 0.0
        candidate = vertices + gain * corr
        c_state, c_rms, c_mean, c_duration = evaluate(candidate)
        if c_mean < mean:
            vertices, state, rms, mean, duration = candidate, c_state, c_rms, c_mean, c_duration
        else:
            gain *= 0.5
        print("Lag compensation progress:", str(int(100 * (k + 1) / max(iterations, 1))) + "%")
        print("Lag compensation (iterative): pass", k + 1, "jet off the path by mean", round(mean, 2), "um, rms",
              round(rms, 2), "um, gain", gain, ", time", round(duration, 2), "s")

    writer = GcodeWriter(scale, (0.0, 0.0))
    dwell_at = {}
    for d in dwells:
        dwell_at.setdefault(d[0], []).append(d[1])
    if len(feeds):
        writer.g1(vertices[0, 0], vertices[0, 1], feeds[0] * 60.0)
    for i in range(len(feeds)):
        for seconds in dwell_at.pop(i, []):
            writer.dwell(seconds)
        writer.g1(vertices[i + 1, 0], vertices[i + 1, 1], feeds[i] * 60.0)
    for rest in dwell_at.values():
        for seconds in rest:
            writer.dwell(seconds)
    return writer.lines


# --- running it -----------------------------------------------------------------------------

def compensate(params, variables):
    # Write the compensated G-code; returns its path (None if nothing to compensate)
    method = variables["Lag_compensation"]
    if method not in METHODS or method == "none":
        return None
    if "Lag_at_command_end" not in params:
        print("Lag compensation skipped - the lag model did not run (needs Acceleration > 0)")
        return None
    score = baseline = None
    if method != "slowdown":
        score = JetScorer(params, variables)
        baseline = score(params["Preview_segments"], params.get("Dwells", []))
    from .hybrid import compensate_hybrid, compensate_hybrid_constant
    lines = {"overshoot": compensate_overshoot, "pointwise": compensate_pointwise, "slowdown": compensate_slowdown,
             "iterative": compensate_iterative, "hybrid": compensate_hybrid,
             "hybrid_constant": compensate_hybrid_constant}[method](params, variables, score, baseline)
    js = params["Lag_model"][0]
    mark = comment_mark(params)
    header = [f"{mark} Lag compensated by Unlooper ({method}) from {params['Filename_only']}",
              f"{mark} CTS {js} mm/min, acceleration {variables['Acceleration_mm_s2']} mm/s^2, junction deviation "
              f"{variables['Junction_deviation_mm']} mm, jerk {variables.get('Jerk_mm_s', 0)} mm/s"]
    if method in ("hybrid", "hybrid_constant") and variables.get("Lag_comp_diameter_limit_pct", 0) > 0:
        header.append(f"{mark} Fibre diameter held within +{variables['Lag_comp_diameter_limit_pct']:g}% (jet at most "
                      f"{variables['Lag_comp_speed_change_pct']:.1f}% slower)")
    path = output_base(params) + "_Lag_compensated.txt"
    if variables.get("Mandrel_diameter_mm", 0) > 0:
        # Back onto the mandrel: Y (surface distance) as A, arcs as G1 chords
        from .mandrel import wrap_lines
        lines = wrap_lines(lines, float(variables["Mandrel_diameter_mm"]))
        header.append(f"{mark} Mandrel {variables['Mandrel_diameter_mm']} mm: A in degrees, F as surface speed")
    with open(path, "w") as f:
        f.write("\n".join(header + lines) + "\n")
    print("Lag compensated code saved:", path, "(" + str(len(lines)), "lines)")
    return path


def comment_mark(params):
    # The comment character the source file uses ('%' as in Mach3 / MEW files, else ';')
    lines = params.get("File_contents") or []
    uses_percent = any(ln.lstrip().startswith("%") for ln in lines[:200])
    uses_semicolon = any(";" in ln for ln in lines[:200])
    return "%" if uses_percent and not uses_semicolon else ";"


def export_gcode(src, dst, comments="%"):
    # Copy a compensated file for printing: comment lines in the controller's style ('%',
    # ';' or '' to drop them). Motion lines are unchanged.
    out = []
    with open(src) as f:
        for line in f.read().splitlines():
            stripped = line.lstrip()
            if stripped.startswith(("%", ";")):
                if comments:  # '%@' estimate lines follow the chosen style too
                    out.append(comments + stripped[1:])
                continue
            out.append(line)
    with open(dst, "w") as f:
        f.write("\n".join(out) + "\n")
    return dst


def compensated_samples_path(path):
    # Where the child run of the compensated file leaves its jet points for the before /
    # after image
    stem = os.path.splitext(os.path.basename(path))[0]
    return os.path.join("Output", stem, stem + "_jet_samples.npy")


def save_comparison_image(params, variables, compensated):
    # The ISBF before / after picture: programmed path black, jet before compensation grey,
    # jet after compensation green, on a white background with the 1 mm grid
    before = params.get("Lag_samples")
    if before is None or not len(before) or compensated is None or not len(compensated):
        return
    height, width = int(variables["Y_build"] * 100), int(variables["X_build"] * 100)
    image = np.full((height, width, 3), 255, dtype=np.uint8)
    for gx in range(100, width, 100):
        image[:, gx] = (220, 220, 220)
    for gy in range(100, height, 100):
        image[gy, :] = (220, 220, 220)
    thickness = max(int(variables["Line_width"]), 1)

    def draw(x, y, rgb):
        px, py = to_png_pixels(variables, np.asarray(x, dtype=np.float64), np.asarray(y, dtype=np.float64))
        pts = np.column_stack((px, py)).astype(np.int32)
        cv2.polylines(image, [pts], False, (rgb[2], rgb[1], rgb[0]), thickness, cv2.LINE_AA, PNG_SHIFT)

    rx, ry, _cum = reference_polyline(params["Preview_segments"])
    draw(rx, ry, PROGRAMMED_RGB)
    draw(before[:, 0], before[:, 1], JET_BEFORE_RGB)
    draw(compensated[:, 0], compensated[:, 1], JET_AFTER_RGB)
    out = output_base(params) + "_lag_compensation.png"
    cv2.imwrite(out, image)
    if variables.get("Mandrel_diameter_mm", 0) > 0:
        from .mandrel import render_comparison
        render_comparison(params, variables, before, compensated, output_base(params) + "_lag_compensation_mandrel.png")
    print("Lag compensation image saved:", out, "(black = programmed path, grey = jet before, green = jet after)")


def run_compensated(path, params, variables, script):
    # Process the compensated file with Unlooper.py (same settings, lag prediction on, no
    # further compensation), scoring its jet against this file's programmed path
    reference = output_base(params) + "_reference_segments.npy"
    np.save(reference, np.asarray(params["Preview_segments"], dtype=np.float64))
    args = [sys.executable, script, path, "0", str(variables["Feedrate_override_mm_min"]),
            str(variables["Material_Density_override"]), str(variables["Fibre_Diameter_override"]),
            variables["render_mode"], str(variables["Acceleration_mm_s2"]), str(variables["Junction_deviation_mm"]),
            "0" if variables["Generate_pixel_coords"] else "1", str(variables.get("Jerk_mm_s", 0)), "1",
            str(params["Lag_model"][0]), "0" if variables["high_speed"] else "1", "none", "3000", "0.85", "1.0", "0", "0", "20",
            "0", str(variables.get("Mandrel_diameter_mm", 0)), str(variables.get("Diameter_tolerance_pct", 5.0))]
    env = dict(os.environ, **{REFERENCE_ENV: os.path.abspath(reference), "PYTHONUNBUFFERED": "1", "PYTHONIOENCODING": "utf-8"})
    print("************** Lag compensated code **************")
    progress = re.compile(r"^(.+) progress:\s*(\d+)%$")
    process = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, env=env, text=True,
                               encoding="utf-8", errors="replace", bufsize=1)
    for raw in process.stdout:
        for line in re.split(r"[\r\n]", raw):
            line = line.strip()
            if not line or "it/s]" in line:
                continue
            m = progress.match(line)
            if m:
                print(f"Compensated {m.group(1).lower()} progress: {m.group(2)}%", flush=True)
            else:
                print("Compensated:", line, flush=True)
    process.wait()
    if process.returncode != 0:
        print("Compensated: run failed (exit code", str(process.returncode) + ")")
        return
    samples = compensated_samples_path(path)
    if os.path.exists(samples):
        save_comparison_image(params, variables, np.load(samples))
