"""Stage 4b - the corner-rounded path: where the nozzle actually goes when the controller
keeps its speed through corners (constant-velocity mode), as pixel coords.

The planner (motion_planner.py) only decides how fast each corner is taken. A machine
that holds that speed through a corner can't follow the sharp corner exactly: within its
acceleration limit it can only turn on a radius of at least v^2 / a. So every corner taken
at speed v is replaced by an arc tangent to both moves:

    r = min(v^2 / a, junction-deviation radius)    (the arc junction deviation assumes)

cut in at distance r / tan(theta/2) from the corner (theta = the corner's inside angle),
never more than half of either move. The arc stays within r * (1/sin(theta/2) - 1) of the
sharp corner, which is at most the junction deviation. Corners taken from a stop, full
reversals and straight-on joins are left sharp.

The rounded path is then planned and sampled exactly like the sharp one, and written as
_corner_pixel_cords.csv / _corner_pixel_coords_motion.csv in the same layout as the pixel
coords, for the lag model. Where a corner touches a G2/G3 arc, the rounding is built on the
arc's tangent (the error is below cut-in^2 / (2 * arc radius), well under 1 um).
"""
import bisect
import math

from .motion_planner import junction_limits, move_directions, plan_moves
from .pixel_coords import LagFormatWriter, build_timeline, output_base, point_counts, sample_timeline


def _point_at(cx, cy, r, angle):
    # Qt / Preview_segments convention: point at angle a is (cx + r cos a, cy - r sin a)
    return cx + r * math.cos(angle), cy - r * math.sin(angle)


def build_corner_segments(segments, planned, dwells, variables):
    # Returns (segments of the rounded path, its dwells, corner stats). Rows use the same
    # layout as params["Preview_segments"]; the halves of each corner arc belong to the
    # command before / after the corner so the lag file keeps every command's points together.
    accel, deviation, _jerk, _min_speed = junction_limits(variables)
    scale = variables["scale"]
    moves = []
    for index, length, v0, _peak, _v1, _d_acc, _d_dec in planned:
        row = tuple(segments[index][:11])
        _length, u_in, u_out, radius = move_directions(*row[:8], scale)
        moves.append((row, length, v0, u_in, u_out, radius))
    n = len(moves)

    trim_start = [0.0] * n  # mm cut off the start / end of each move by its corners
    trim_end = [0.0] * n
    fillet = [None] * n  # corner arc at the start of move i: (x1, y1, cx, cy, r, sweep_rad, x2, y2)
    deviations = []
    for i in range(1, n):
        prev, curr = moves[i - 1], moves[i]
        speed = curr[2]  # planned speed through this corner
        px, py = prev[0][3], prev[0][4]
        if speed <= 0 or math.hypot(curr[0][1] - px, curr[0][2] - py) > 0.01:
            continue  # taken from a stop, or the moves don't meet
        u_out, u_in = prev[4], curr[3]
        cos_theta = -(u_out[0] * u_in[0] + u_out[1] * u_in[1])
        if cos_theta < -0.999999 or cos_theta > 0.999999:
            continue  # straight on / full reversal
        sin_half = math.sqrt(0.5 * (1.0 - cos_theta))
        cos_half = math.sqrt(0.5 * (1.0 + cos_theta))
        radius = speed * speed / accel
        if deviation > 0:
            radius = min(radius, deviation * sin_half / (1.0 - sin_half))
        cut = radius * cos_half / sin_half
        cut = min(cut, 0.5 * prev[1], 0.5 * curr[1])
        for move in (prev, curr):
            if move[5] > 0:
                cut = min(cut, move[5] / 4)  # keep the tangent approximation good on G2/G3
        if cut * scale < 1e-3:
            continue
        radius = cut * sin_half / cos_half
        trim_end[i - 1] = cut
        trim_start[i] = cut
        # Tangent points, and the centre on the inside of the turn
        cut_um, r_um = cut * scale, radius * scale
        x1, y1 = px - u_out[0] * cut_um, py - u_out[1] * cut_um
        x2, y2 = px + u_in[0] * cut_um, py + u_in[1] * cut_um
        turn = u_out[0] * u_in[1] - u_out[1] * u_in[0]
        nx, ny = (-u_out[1], u_out[0]) if turn > 0 else (u_out[1], -u_out[0])
        cx, cy = x1 + nx * r_um, y1 + ny * r_um
        a1 = math.atan2(cy - y1, x1 - cx)
        a2 = math.atan2(cy - y2, x2 - cx)
        sweep = (a2 - a1 + math.pi) % (2 * math.pi) - math.pi
        fillet[i] = (x1, y1, cx, cy, r_um, sweep, x2, y2)
        deviations.append(radius * (1.0 / sin_half - 1.0))

    out = []
    first_piece = []

    def add_fillet_half(i, second, row):
        x1, y1, cx, cy, r_um, sweep, x2, y2 = fillet[i]
        a1 = math.atan2(cy - y1, x1 - cx)
        mx, my = _point_at(cx, cy, r_um, a1 + sweep / 2)
        start, end = ((mx, my), (x2, y2)) if second else ((x1, y1), (mx, my))
        out.append((2, start[0], start[1], end[0], end[1], cx, cy, math.degrees(sweep / 2), row[8], row[9], row[10]))

    for i, (row, length, _v0, u_in, _u_out, radius) in enumerate(moves):
        first_piece.append(len(out))
        if fillet[i] is not None:
            add_fillet_half(i, True, row)
        kind, x1, y1, x2, y2, cx, cy, sweep = row[:8]
        if length - trim_start[i] - trim_end[i] > 1e-9:
            if kind == 1:
                ux, uy = (x2 - x1) / (length * scale), (y2 - y1) / (length * scale)
                x1, y1 = x1 + ux * trim_start[i] * scale, y1 + uy * trim_start[i] * scale
                x2, y2 = x2 - ux * trim_end[i] * scale, y2 - uy * trim_end[i] * scale
            else:
                r_um = radius * scale
                d = 1.0 if sweep > 0 else -1.0
                a1 = math.atan2(cy - y1, x1 - cx) + d * trim_start[i] * scale / r_um
                sweep = sweep - d * math.degrees((trim_start[i] + trim_end[i]) * scale / r_um)
                x1, y1 = _point_at(cx, cy, r_um, a1)
                x2, y2 = _point_at(cx, cy, r_um, a1 + math.radians(sweep))
            out.append((kind, x1, y1, x2, y2, cx, cy, sweep) + tuple(row[8:11]))
        if i + 1 < n and fillet[i + 1] is not None:
            add_fillet_half(i + 1, False, row)

    # Dwells: same place in the move order (a dwell is a stop, so it never has a corner arc)
    seg_of_move = [p[0] for p in planned]
    corner_dwells = []
    for dwell in dwells:
        j = bisect.bisect_left(seg_of_move, dwell[0])
        corner_dwells.append((first_piece[j] if j < n else len(out),) + tuple(dwell[1:]))
    return out, corner_dwells, deviations


def generate_corner_path(params, variables, consumers=()):
    # Build, plan and sample the corner-rounded path. Every block of pixel coords is passed
    # to each consumer (the lag model), and written out when the lag-format files are on.
    planned = params.get("Planned_moves", [])
    if not planned:
        return
    accel = float(variables["Acceleration_mm_s2"])
    dt = float(variables["scatter_resolution"])
    scale = variables["scale"]
    segments, dwells, deviations = build_corner_segments(params["Preview_segments"], planned, params.get("Dwells", []), variables)
    corners = len(planned) - 1
    if deviations:
        print("Corner rounding:", len(deviations), "of", corners, "corners rounded, deviation up to",
              round(max(deviations) * 1000, 3), "um (mean", str(round(sum(deviations) / len(deviations) * 1000, 3)) + " um)")
    else:
        print("Corner rounding: no corners rounded (all taken from a stop, straight on or reversals)")
    corner_planned, move_time, _below = plan_moves(segments, dwells, variables)
    params["Corner_segments"] = segments
    params["Corner_planned_moves"] = corner_planned
    if not corner_planned:
        return
    dwell_time = sum(d[1] for d in dwells)
    length = sum(p[1] for p in corner_planned)
    print("Corner path: length", round(length / 1000, 4), "m, time", round(move_time + dwell_time, 3), "s")

    tl = build_timeline(segments, corner_planned, dwells, accel)
    write_all = variables["high_speed"] == False
    out_base = output_base(params)
    if write_all:
        xy_file = open(out_base + "_corner_pixel_cords.csv", "w")
        writer = LagFormatWriter(xy_file, out_base + "_corner_pixel_coords_motion.csv",
                                 params["One_coordinate_system"], tl.owner, point_counts(tl, dt))
    for b in sample_timeline(tl, accel, dt, scale):
        if write_all:
            writer.write(b, tl.owner, tl.line)
        for consumer in consumers:
            consumer(b, tl)
        print("Corner path progress:", str(int(100 * (b.first + b.count) / b.total)) + "%")
    if write_all:
        writer.close()
        xy_file.close()
        print("Corner pixel coords saved (lag format):", b.total, "points,", out_base + "_corner_pixel_cords.csv")
