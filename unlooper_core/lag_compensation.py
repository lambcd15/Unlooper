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
  iterative  - model-driven: the nozzle path is corrected repeatedly with the real planner
               in the loop - plan the path (acceleration, junction deviation, jerk, corner
               rounding), simulate the jet, measure how far each jet point is from the
               programmed path, move the path points that put it there (one lag's travel
               earlier) by that error, repeat. Written as G1 moves at the original feed
               rates, dense only around corners and along arcs.

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


METHODS = ("none", "overshoot", "pointwise", "slowdown", "iterative", "hybrid")
CORNER_MIN_DEG = 5.0  # turns smaller than this are left alone (as vector_angle's angle bound)
MAX_ITERATIVE_POINTS = 4_000_000  # correction passes sample coarser than 1 ms beyond this
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
    program = Program(params["One_coordinate_system"])
    owner = list(range(len(program)))
    dwells = params.get("Dwells", [])
    if split_mm is not None or graded:
        # Every G1 cut into points (split_mm apart, or graded), each treated as a command
        program, owner_arr, first = program.split(split_mm, dt_s=float(variables["scatter_resolution"]))
        owner = owner_arr.tolist()
        dwells = [(d[0], d[1], int(first[int(d[2])]) if int(d[2]) < len(first) else len(program)) + tuple(d[3:])
                  for d in dwells]
    source = dict(params, One_coordinate_system=program, Dwells=dwells)
    solved = {}

    def run(extra, arc_extra):
        return vector_angle(source, variables, js, a, b, factor=factor, rapid=rapid, extra=extra, arc_extra=arc_extra,
                            mode=mode, fixed=solved, merge=split_mm is not None or graded)

    lines, corners, arcs = run({}, {})
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
    from .hybrid import compensate_hybrid
    lines = {"overshoot": compensate_overshoot, "pointwise": compensate_pointwise, "slowdown": compensate_slowdown,
             "iterative": compensate_iterative, "hybrid": compensate_hybrid}[method](params, variables, score, baseline)
    js = params["Lag_model"][0]
    header = [f"; Lag compensated by Unlooper ({method}) from {params['Filename_only']}",
              f"; CTS {js} mm/min, acceleration {variables['Acceleration_mm_s2']} mm/s^2, junction deviation "
              f"{variables['Junction_deviation_mm']} mm, jerk {variables.get('Jerk_mm_s', 0)} mm/s"]
    path = output_base(params) + "_Lag_compensated.txt"
    with open(path, "w") as f:
        f.write("\n".join(header + lines) + "\n")
    print("Lag compensated code saved:", path, "(" + str(len(lines)), "lines)")
    return path


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
            str(params["Lag_model"][0]), "0" if variables["high_speed"] else "1", "none"]
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
