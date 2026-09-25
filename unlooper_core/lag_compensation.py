"""Stage 6 - lag compensation: rewrite the G-code so the jet lands on the programmed path,
then run the whole pipeline on the rewritten file to see where the jet lands now.

Three methods (variables["Lag_compensation"]):

  overshoot  - the ISBF method (Gcode_processing.py vector_angle): at every corner the
               nozzle carries on past the corner by the lag, swings round the corner on an
               arc centred on it (at the rapid speed) and joins the next move the lag
               distance along it, so the trailing jet is dragged into the corner. The
               overshoot at each corner is the lag the model predicts there (times a scale
               factor), not a fixed fraction of one lag length.
  slowdown   - speed compensation: before each corner the nozzle slows to a set ratio of
               the CTS for just long enough (from the lag model) for the jet to catch up to
               within a tolerance, so the jet reaches the corner with it. The path is
               unchanged; only the feed rates are.
  iterative  - model-driven: the nozzle path is corrected repeatedly with the real planner
               in the loop - plan the path (acceleration, junction deviation, jerk, corner
               rounding), simulate the jet, measure how far each jet point is from the
               programmed path, move the path points that put it there (one lag's travel
               earlier) by that error, repeat. Written as G1 moves at the original feed
               rates, dense only around corners and along arcs.

The compensated file is Output/<name>/<name>_Lag_compensated.txt. It is then processed
by Unlooper.py in a child process (outputs in Output/<name>_Lag_compensated/) with the
lag model scored against the original programmed path, and its console lines are
passed on with a "Compensated" prefix.
"""
import math
import os
import re
import subprocess
import sys

import numpy as np

from .lag_model import REFERENCE_ENV, lag_steps, objective
from .motion_planner import move_directions, plan_moves
from .path_reference import project, reference_polyline
from .pixel_coords import build_timeline, output_base, sample_timeline


METHODS = ("none", "overshoot", "slowdown", "iterative")
CORNER_MIN_DEG = 5.0  # turns smaller than this are left alone (as vector_angle's angle bound)
MAX_ITERATIVE_POINTS = 4_000_000  # the iterative method samples coarser than 1 ms beyond this


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


def _dwells_before(dwells, index):
    return [d for d in dwells if d[0] <= index]


# --- the three methods --------------------------------------------------------------------

def compensate_overshoot(params, variables):
    scale = variables["scale"]
    rapid = float(variables["Lag_comp_rapid_mm_min"])
    factor = float(variables["Lag_comp_overshoot_scale"])
    lag_end = params["Lag_at_command_end"]
    moves = _moves(params["Preview_segments"], variables)
    dwells = sorted(params.get("Dwells", []), key=lambda d: d[0])
    writer = GcodeWriter(scale, (0.0, 0.0))
    trim = 0.0  # mm the current move starts in from its start (after a corner swing)
    corners = 0
    for i, move in enumerate(moves):
        while dwells and dwells[0][0] <= move["index"]:
            writer.dwell(dwells.pop(0)[1])
        if trim == 0.0:
            writer.g1(*move["start"], move["feed"])
        elif move["kind"] != 1:
            # The swing ended on the arc's tangent; step onto the arc itself
            writer.g1(*_point_along(move, trim, scale), move["feed"])
        nxt = moves[i + 1] if i + 1 < len(moves) else None
        overshoot = 0.0
        if nxt is not None:
            turn, meets = _turn(move, nxt)
            if meets and CORNER_MIN_DEG < turn < 180.0 - CORNER_MIN_DEG:
                overshoot = min(float(lag_end[move["ocs"]]) * factor, 0.9 * nxt["length"])
        _emit_to(writer, move, *move["end"], move["feed"])
        trim = 0.0
        if overshoot > 1e-4:
            # Carry on past the corner by the lag, then swing round it (centre = corner)
            vx, vy = move["end"]
            u_out, u_in = move["u_out"], nxt["u_in"]
            writer.g1(vx + u_out[0] * overshoot * scale, vy + u_out[1] * overshoot * scale, move["feed"])
            ccw = (u_out[0] * u_in[1] - u_out[1] * u_in[0]) < 0  # y is flipped
            writer.arc(vx + u_in[0] * overshoot * scale, vy + u_in[1] * overshoot * scale, vx, vy, ccw, rapid)
            trim = overshoot
            corners += 1
    for d in dwells:
        writer.dwell(d[1])
    print("Lag compensation (overshoot):", corners, "corners, rapid", rapid, "mm/min, overshoot = model lag x", factor)
    return writer.lines


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


def compensate_slowdown(params, variables):
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


def _executed_jet(vx, vy, feeds, lines, dwells, variables, model_ab, dt):
    # Plan, round the corners and sample the polyline exactly as a run of it would, then
    # simulate the jet. Returns the samples (µm), the segment each belongs to, and the jet.
    from .corner_path import build_corner_segments
    accel = float(variables["Acceleration_mm_s2"])
    scale = variables["scale"]
    rows = [(1, vx[i], vy[i], vx[i + 1], vy[i + 1], 0.0, 0.0, 0.0, lines[i], feeds[i], i) for i in range(len(feeds))]
    planned, _t, _b = plan_moves(rows, dwells, variables)
    corner_rows, corner_dwells, _dev = build_corner_segments(rows, planned, dwells, variables)
    corner_planned, _t, _b = plan_moves(corner_rows, corner_dwells, variables)
    tl = build_timeline(corner_rows, corner_planned, corner_dwells, accel)
    xs, ys, owners = [], [], []
    for block in sample_timeline(tl, accel, dt, scale):
        xs.append(block.x)
        ys.append(block.y)
        owners.append(tl.owner[block.m])
    x, y, owner = np.concatenate(xs), np.concatenate(ys), np.concatenate(owners)
    js, a, b = model_ab
    first_speed = corner_planned[0][3] if corner_planned else 0.0
    model = (js, dt / 60.0, a, b, max(first_speed * dt / 50.0, 1e-9))
    lag0 = max(objective(first_speed * 60.0 / js, a, b), 0.0) if first_speed > 0 else 0.0
    cx, cy, lag = _simulate_jet(x, y, model, lag0)
    return x, y, owner, cx, cy, lag, float(tl.t_end[-1])


def compensate_iterative(params, variables):
    # Iterative learning control on the nozzle path, with the real planner in the loop:
    # every pass plans the current path (acceleration, junction deviation, jerk, corner
    # rounding), simulates the jet on it and moves each vertex by the error of the jet
    # points it caused - those one lag's travel later - smoothed along the path.
    scale = variables["scale"]
    js, _dt, a, b, _eps = params["Lag_model"]
    lag_max = float(params.get("Lag_range", (0.0, 1.0))[1]) or 1.0
    vertices, feeds, lines, dwells = _dense_polyline(params, variables, lag_max)
    rx, ry, cum = reference_polyline(params["Preview_segments"])
    total_time = sum(p[1] / max(p[3], 1e-9) for p in params.get("Planned_moves", [])) or 1.0
    dt = max(float(variables["scatter_resolution"]), total_time / MAX_ITERATIVE_POINTS)
    n_vert = len(vertices)

    start_cut = []

    def evaluate(v):
        x, y, owner, cx, cy, lag, duration = _executed_jet(v[:, 0], v[:, 1], feeds, lines, dwells, variables, (js, a, b), dt)
        n = len(x)
        d, qx, qy = np.empty(n), np.empty(n), np.empty(n)
        project(cx, cy, rx, ry, cum, 0, 3000.0, d, qx, qy)
        # Score from the moment the jet first lands on the path in the original run: the
        # start-up tail (the jet starting a lag behind the first point) can't be compensated.
        # Fixed in time so every pass is scored over the same part of the print.
        if not start_cut:
            landed = np.flatnonzero(d < 10.0)
            start_cut.append(int(landed[0]) if len(landed) else 0)
        scored = d[min(start_cut[0], n - 1):]
        return (x, y, owner, qx - cx, qy - cy, lag), float(np.sqrt(np.mean(scored * scored))), float(np.mean(scored)), duration

    state, rms, mean, duration = evaluate(vertices)
    print("Lag compensation (iterative):", n_vert, "path points,", len(state[0]), "samples every", round(dt * 1000, 3),
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
        seg = owner
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
    lines = {"overshoot": compensate_overshoot, "slowdown": compensate_slowdown,
             "iterative": compensate_iterative}[method](params, variables)
    js = params["Lag_model"][0]
    header = [f"; Lag compensated by Unlooper ({method}) from {params['Filename_only']}",
              f"; CTS {js} mm/min, acceleration {variables['Acceleration_mm_s2']} mm/s^2, junction deviation "
              f"{variables['Junction_deviation_mm']} mm, jerk {variables.get('Jerk_mm_s', 0)} mm/s"]
    path = output_base(params) + "_Lag_compensated.txt"
    with open(path, "w") as f:
        f.write("\n".join(header + lines) + "\n")
    print("Lag compensated code saved:", path, "(" + str(len(lines)), "lines)")
    return path


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
