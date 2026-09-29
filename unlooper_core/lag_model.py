"""Stage 5 - jet lag prediction: where the electrospun jet actually lands (the contact
point) as the nozzle follows the pixel coords.

A port of Ievgenii's model (ISBF Ievgenii/python_lag.py, compute_lag), itself a copy of
PathPrediction_v8_3_ng.m. Each time step the jet is pulled towards the nozzle:
  - ray      = nozzle position now - contact point one step ago
  - runaway  = |ray| - previous lag, speed ratio SR = runaway / (dt * CTS)
  - the lag if the jet were falling back to vertical at JetFallSpeed(previous lag) is
    |ray| - dt * JetFallSpeed; if SR > 1 the jet is dragged and the lag is at least the
    stationary lag for that speed ratio, Lag(SR) = a * SR^b - a (fitted to the measured
    lag / speed ratio data in lag_data/Lag_1.2b_fM.csv)
  - the contact point moves along the ray to sit `lag` behind the nozzle
The step is the pixel coords time step, and the nozzle positions come from the
corner-rounded path, so acceleration, deceleration and corner rounding all feed through.

What changed from python_lag.py (the maths per step is the same):
  - runs on the pixel coords as they are generated, block by block, instead of writing
    the pixel coords CSV, starting python_lag.py in a subprocess and reading it back
  - the per-step loop is compiled with numba (falls back to plain Python without it) and
    keeps only the current state instead of ~12 full-length arrays
  - the time step is the real pixel coords time step, so it holds with changing speeds
    (python_lag.py derived it from the first two points at one collector speed)
  - the jet starts `Lag(K0)` behind the nozzle along the first direction of travel, with
    K0 = first move's speed / CTS, and never ahead of it (K0 < 1 gives a negative Lag)
"""
import json
import math
import os
from pathlib import Path

import numpy as np

from .palette import SPEED_BANDS, SPEED_COLOURS, GRID_INDEX, NOZZLE_INDEX, band_of, band_edges, save_indexed_png
from .path_reference import DeviationTracker
from .pixel_coords import draw_runs, output_base, to_png_pixels

try:
    from numba import njit
except ImportError:  # pragma: no cover - numba is optional, just slower without it
    njit = None

CALIBRATION = Path(__file__).resolve().parent.parent / "lag_data" / "Lag_1.2b_fM.csv"
DEFAULT_CTS_MM_MIN = 230.0  # python_lag.py's default jet speed when none is given
LAG_LEVELS = 250  # fine lag levels while drawing; mapped to the colour bands at the end
# The path the jet is scored against. A lag-compensated run is scored against the
# original file's path (saved by lag_compensation.py and passed in this variable), not
# against its own compensated nozzle path.
REFERENCE_ENV = "UNLOOPER_REFERENCE_SEGMENTS"


def objective(x, a, b):
    # Stationary lag (mm) at speed ratio x
    return a * (x ** b) - a


def fit_lag_curve(path=CALIBRATION):
    from scipy.optimize import curve_fit
    data = np.loadtxt(path, delimiter=",", skiprows=1)
    popt, _ = curve_fit(objective, data[:, 0], data[:, 1], bounds=([-15, -1], [15, 0]))
    return float(popt[0]), float(popt[1])


def _nanmax(x, y):
    # np.maximum: NaN if either is NaN (as python_lag.py's arrays behave)
    if x != x or y != y:
        return math.nan
    return x if x > y else y


if njit is not None:
    _nanmax = njit(cache=True)(_nanmax)


def _lag_steps(gx, gy, cpx, cpy, lag, js, dt, a, b, eps, out_x, out_y, out_lag):
    # One step per nozzle position (gx, gy); state = contact point (cpx, cpy) and lag.
    # Same arithmetic as python_lag.compute_lag(), in mm and mm/min with dt in minutes.
    inv_b = 1.0 / b
    for i in range(gx.shape[0]):
        rx = gx[i] - cpx
        ry = gy[i] - cpy
        ray = math.sqrt(rx * rx + ry * ry)
        sr = (ray - lag) / (dt * js)
        base = (lag + a) / a
        fall_speed = js * base ** inv_b if base > 0 else math.nan
        falling = ray - dt * fall_speed
        if sr > 1:
            stationary = a * sr ** b - a
            if ray > stationary:
                new_lag = _nanmax(stationary, falling)
            else:
                new_lag = _nanmax(falling, 0.0)
        elif lag > eps:
            new_lag = _nanmax(falling, 0.0)
        else:
            new_lag = 0.0
        if ray > eps:
            f = new_lag / ray
            cpx = gx[i] - rx * f
            cpy = gy[i] - ry * f
        else:
            cpx = gx[i]
            cpy = gy[i]
        lag = new_lag
        out_x[i] = cpx
        out_y[i] = cpy
        out_lag[i] = lag
    return cpx, cpy, lag


lag_steps = njit(cache=True, nogil=True)(_lag_steps) if njit is not None else _lag_steps


class JetLagModel:
    # Consumer for generate_corner_path(): runs the model on each block of pixel coords,
    # draws the jet path, keeps the points the GUI needs, writes _lag.csv when the
    # lag-format files are on, records the lag at the end of every command (for the lag
    # compensation) and measures how far the jet lands from the programmed path.

    def __init__(self, params, variables):
        self.params, self.variables = params, variables
        self.a, self.b = fit_lag_curve()
        cts = float(variables.get("global_return_CTS", 0))
        if cts <= 0:
            cts = DEFAULT_CTS_MM_MIN
            print("Lag prediction: no CriticalTranslationSpeed in the file or override, using", cts, "mm/min")
        self.js = cts
        self.dt = float(variables["scatter_resolution"]) / 60.0  # minutes, as python_lag.py
        self.scale = variables["scale"]
        self.state = None
        self.lag_min, self.lag_max, self.lag_sum, self.points = math.inf, 0.0, 0.0, 0
        self.kept = []
        self.previous_keys = None
        self.out_base = output_base(params)
        self.csv = None
        if variables["high_speed"] == False:
            self.csv = open(self.out_base + "_lag.csv", "w")
            self.csv.write("X,Y,Lag\n")
        self.image = params.get("Lag_index_image")
        self.lag_at_command_end = np.full(len(params["One_coordinate_system"]), np.nan)
        reference = os.environ.get(REFERENCE_ENV)
        segments = np.load(reference) if reference and os.path.exists(reference) else params["Preview_segments"]
        self.deviation = DeviationTracker(np.asarray(segments, dtype=np.float64).tolist())
        print("Lag prediction: CTS", self.js, "mm/min, lag curve y = %.5f * (x ^ %.5f) - %.5f" % (self.a, self.b, self.a))

    def _start(self, b):
        # Jet Lag(K0) behind the nozzle along the first direction of travel
        planned = self.params["Corner_planned_moves"]
        first_speed = planned[0][3] if planned else 0.0
        k_max = max(p[3] for p in planned) * 60.0 / self.js if planned else 0.0
        # Fine level range for drawing: comfortably above the steady lag at top speed
        self.level_scale = max(3.0 * objective(max(k_max, 1.0), self.a, self.b), 0.5)
        k0 = first_speed * 60.0 / self.js
        lag0 = max(objective(k0, self.a, self.b), 0.0) if k0 > 0 else 0.0
        x0, y0 = b.x[0] / self.scale, b.y[0] / self.scale
        moved = np.flatnonzero(np.hypot(b.x - b.x[0], b.y - b.y[0]) > 0)
        if len(moved):
            dx, dy = b.x[moved[0]] / self.scale - x0, b.y[moved[0]] / self.scale - y0
            norm = math.hypot(dx, dy)
            cp = (x0 - dx / norm * lag0, y0 - dy / norm * lag0)
        else:
            cp = (x0, y0)
        nominal_spacing = first_speed * float(self.variables["scatter_resolution"])  # mm
        self.eps = max(nominal_spacing / 50.0, 1e-9)  # python_lag.py: EpsLaG = NMUL / 50
        self.state = (cp[0], cp[1], lag0)
        print("Lag prediction: speed ratio K0", round(k0, 3), ", initial lag", round(lag0, 4), "mm")

    def __call__(self, b, tl):
        if self.state is None:
            self._start(b)
        gx = b.x / self.scale
        gy = b.y / self.scale
        n = len(gx)
        cx, cy, lag = np.empty(n), np.empty(n), np.empty(n)
        if b.first == 0:
            # Point 0 is the starting state
            cx[0], cy[0], lag[0] = self.state
            self.state = lag_steps(gx[1:], gy[1:], *self.state, self.js, self.dt, self.a, self.b, self.eps, cx[1:], cy[1:], lag[1:])
        else:
            self.state = lag_steps(gx, gy, *self.state, self.js, self.dt, self.a, self.b, self.eps, cx, cy, lag)
        finite = lag[np.isfinite(lag)]
        if len(finite):
            self.lag_min = min(self.lag_min, float(finite.min()))
            self.lag_max = max(self.lag_max, float(finite.max()))
            self.lag_sum += float(finite.sum())
            self.points += len(finite)

        if self.csv is not None:
            # python_lag.py's output layout: contact point X, Y (mm, y up) and the lag
            rows = np.column_stack((cx, -cy, lag))
            self.csv.write("\n".join(f"{x:.6f},{y:.6f},{l:.6f}" for x, y, l in rows.tolist()) + "\n")

        level = np.clip(np.nan_to_num(lag / self.level_scale * LAG_LEVELS, nan=0.0).astype(np.int64), 0, LAG_LEVELS - 1) + 1
        cx_um, cy_um = cx * self.scale, cy * self.scale
        # Lag at the last point of each command (the corner it ends in)
        owner = tl.owner[b.m]
        ends = np.flatnonzero(np.append(owner[1:] != owner[:-1], True))
        self.lag_at_command_end[owner[ends]] = lag[ends]
        self.deviation.add(cx_um, cy_um)
        if self.image is not None:
            px, py = to_png_pixels(self.variables, cx_um, cy_um)
            if b.first > 0:
                # Join to the previous block's last point
                px = np.concatenate(([self.last_px[0]], px))
                py = np.concatenate(([self.last_px[1]], py))
                values = np.concatenate(([self.last_level], level))
            else:
                values = level
            draw_runs(self.image, values[:-1] if len(values) > 1 else values, px, py, int(self.variables["Line_width"]))
            self.last_px = (px[-1], py[-1])
            self.last_level = level[-1]

        # Points the GUI needs to draw the jet path: where the colour group, the direction
        # (3 degree steps) or the command changes, plus the first / last point
        heading = np.floor(np.degrees(np.arctan2(np.diff(cy_um, append=cy_um[-1]), np.diff(cx_um, append=cx_um[-1]))) / 3.0).astype(np.int64)
        keys = (b.m, level // 8, heading)
        keep = np.zeros(n, dtype=bool)
        for key in keys:
            keep[1:] |= key[1:] != key[:-1]
        keep[0] = self.previous_keys is None or any(int(key[0]) != p for key, p in zip(keys, self.previous_keys))
        keep[-1] = keep[-1] or b.last
        self.previous_keys = tuple(int(key[-1]) for key in keys)
        self.kept.append(np.column_stack((cx_um, cy_um, lag, tl.line[b.m]))[keep].astype(np.float32))

    def finish(self):
        params, variables = self.params, self.variables
        if self.csv is not None:
            self.csv.close()
            print("Lag positions saved:", self.out_base + "_lag.csv")
        if self.points == 0:
            return
        lag_min, lag_max = self.lag_min, self.lag_max
        print("Jet lag: mean", round(self.lag_sum / self.points, 4), "mm, range", round(lag_min, 4), "-", round(lag_max, 4), "mm")
        deviation = self.deviation.summary()
        if deviation is not None:
            print("Jet deviation from programmed path: mean", round(deviation[0], 2), "um, 95%", round(deviation[1], 1),
                  "um, max", round(deviation[2], 1), "um")
        # Commands with no points of their own take the lag of the command before
        lag_end = self.lag_at_command_end
        valid = np.isfinite(lag_end)
        if valid.any():
            filled = np.maximum.accumulate(np.where(valid, np.arange(len(lag_end)), -1))
            lag_end = np.where(filled >= 0, lag_end[np.maximum(filled, 0)], 0.0)
        params["Lag_at_command_end"] = np.nan_to_num(lag_end)
        params["Lag_model"] = (self.js, self.dt, self.a, self.b, self.eps)
        params["Lag_samples"] = np.concatenate(self.kept) if self.kept else np.zeros((0, 4), dtype=np.float32)
        params["Lag_range"] = (lag_min, lag_max)
        if os.environ.get(REFERENCE_ENV):
            # A lag-compensated run: leave the jet points for the parent's before / after image
            np.save(self.out_base + "_jet_samples.npy", params["Lag_samples"])
        with open(self.out_base + "_lag_legend.json", "w") as f:
            json.dump({"lag_min_mm": lag_min, "lag_max_mm": lag_max, "lag_bands": SPEED_BANDS,
                       "lag_band_edges_mm": band_edges(lag_min, lag_max), "cts_mm_min": self.js,
                       "lag_curve_a": self.a, "lag_curve_b": self.b}, f)
        if self.image is not None:
            # Fine levels -> the same 32 colour bands as the speed view, over the lag range found
            level_values = (np.arange(LAG_LEVELS) + 0.5) / LAG_LEVELS * self.level_scale
            lut = np.arange(256, dtype=np.uint8)
            lut[1:LAG_LEVELS + 1] = band_of(level_values, lag_min, lag_max) + 1
            lut[GRID_INDEX], lut[NOZZLE_INDEX] = GRID_INDEX, NOZZLE_INDEX
            image = lut[self.image]
            save_indexed_png(self.out_base + "_lag.png", image, SPEED_COLOURS, variables["Background_colour"])
            print("Jet lag image saved:", self.out_base + "_lag.png")
            params["Lag_index_image"] = None
