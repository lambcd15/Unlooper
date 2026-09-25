"""Stage 3 - machine dynamics: plan every move as an acceleration-limited trapezoid with
the corner speeds a Marlin-style controller would allow (junction deviation and/or
classic jerk), giving the acceleration-aware print time."""
import bisect
import math


def junction_limits(variables):
    # The corner-speed settings, in the order junction_speed() takes them
    return (float(variables["Acceleration_mm_s2"]), float(variables["Junction_deviation_mm"]),
            float(variables.get("Jerk_mm_s", 0.0)), float(variables["Minimum_planner_speed_mm_s"]))


def junction_speed(u_out, u_in, limit, accel, deviation, jerk, min_speed):
    # Fastest speed a move ending in direction u_out can hand over to one starting in
    # direction u_in (both unit vectors), never more than limit (the slower nominal speed).
    #
    # Two corner models, each switched off by setting it to 0. When both are on, the lower
    # speed wins:
    #   - junction deviation (Marlin 2, planner.cpp): the corner is treated as if rounded by
    #     an arc that stays within `deviation` of the sharp corner, and the speed is the
    #     fastest that arc can be taken at the acceleration limit: v = sqrt(a * r) with
    #     r = deviation * sin(theta/2) / (1 - sin(theta/2)). A full reversal gets Marlin's
    #     minimum planner speed.
    #   - classic jerk (Marlin 1 / CLASSIC_JERK, Mach3-style): each axis may change its
    #     velocity instantly by up to `jerk` mm/s, so v * |change in direction| <= jerk on
    #     both X and Y.
    # The two are linked: Marlin's suggested junction deviation for a printer tuned with
    # classic jerk is JD = 0.4 * jerk^2 / acceleration (5 mm/s at 1000 mm/s^2 -> 0.01 mm), which
    # is why they give similar corner speeds. In Marlin they're alternatives (CLASSIC_JERK
    # switches junction deviation off); here both can be applied together.
    # Both 0 = exact stop at every change of direction (only straight-on joins keep speed).
    cos_theta = -(u_out[0] * u_in[0] + u_out[1] * u_in[1])
    if cos_theta < -0.999999:
        return limit  # straight on
    if deviation <= 0 and jerk <= 0:
        return 0.0
    speed = limit
    if deviation > 0:
        if cos_theta > 0.999999:
            speed = min(speed, min_speed)  # full reversal
        else:
            sin_theta_d2 = math.sqrt(0.5 * (1.0 - cos_theta))
            speed = min(speed, math.sqrt(accel * deviation * sin_theta_d2 / (1.0 - sin_theta_d2)))
    if jerk > 0:
        change = max(abs(u_in[0] - u_out[0]), abs(u_in[1] - u_out[1]))
        if change > 0:
            speed = min(speed, jerk / change)
    return speed


def move_directions(kind, x1, y1, x2, y2, cx, cy, sweep, scale):
    # Length (mm), entry and exit unit directions, and radius (mm, 0 for a line) of one
    # toolpath segment. y is flipped (screen coordinates) as in params["Preview_segments"].
    if kind == 1:
        length = math.hypot(x2 - x1, y2 - y1) / scale
        if length <= 0:
            return 0.0, None, None, 0.0
        u = ((x2 - x1) / scale / length, (y2 - y1) / scale / length)
        return length, u, u, 0.0
    radius = math.hypot(x1 - cx, y1 - cy) / scale
    length = radius * math.radians(abs(sweep))
    if length <= 0:
        return 0.0, None, None, radius
    # Point at angle a is (cx + r cos a, cy - r sin a); travel is towards +a when sweep > 0
    a1 = math.atan2(cy - y1, x1 - cx)
    a2 = a1 + math.radians(sweep)
    d = 1.0 if sweep > 0 else -1.0
    return length, (-d * math.sin(a1), -d * math.cos(a1)), (-d * math.sin(a2), -d * math.cos(a2)), radius


def plan_moves(segments, dwells, variables):
    # Plan the moves in `segments` (rows as in params["Preview_segments"]), with the G4
    # `dwells` ((segments before, seconds, ...)) as full stops. Returns
    # (planned moves, time spent moving in s, path below the CTS in mm), where each planned
    # move is (segment index, length mm, v_entry, v_peak, v_exit, accel length, decel length).
    accel, deviation, jerk, min_speed = junction_limits(variables)
    scale = variables["scale"]
    override = variables["Feedrate_override_mm_min"] / 60.0

    # (segment index, length mm, nominal speed mm/s, entry direction, exit direction)
    moves = []
    for index, (kind, x1, y1, x2, y2, cx, cy, sweep, _line, feed, *_) in enumerate(segments):
        nominal = override if override > 0 else feed
        if nominal <= 0:
            continue
        length, u_in, u_out, radius = move_directions(kind, x1, y1, x2, y2, cx, cy, sweep, scale)
        if length <= 0:
            continue
        if kind != 1:
            # Arcs are capped where the centripetal acceleration v^2/r reaches the limit
            nominal = min(nominal, math.sqrt(accel * radius))
        moves.append((index, length, nominal, u_in, u_out))
    if not moves:
        return [], 0.0, 0.0

    n = len(moves)
    # entry[i] = speed at the start of move i, entry[n] = speed at the end of the last move.
    # The print starts and ends at rest.
    entry = ([0.0] + [junction_speed(moves[i - 1][4], moves[i][3], min(moves[i - 1][2], moves[i][2]),
                                     accel, deviation, jerk, min_speed) for i in range(1, n)] + [0.0])
    # A G4 dwell between two moves brings the machine to a stop
    dwell_at = sorted(d[0] for d in dwells if d[1] > 0)
    seg_of_move = [m[0] for m in moves]
    for k in dwell_at:
        i = bisect.bisect_left(seg_of_move, k)
        if 0 < i < n:
            entry[i] = 0.0
    for i in range(n - 1, -1, -1):
        entry[i] = min(entry[i], math.sqrt(entry[i + 1] ** 2 + 2 * accel * moves[i][1]))
    for i in range(n):
        entry[i + 1] = min(entry[i + 1], math.sqrt(entry[i] ** 2 + 2 * accel * moves[i][1]))

    total_time = 0.0
    cts = variables.get("global_return_CTS", 0) / 60.0
    below_cts = 0.0
    planned = []
    for i, (index, length, nominal, _u_in, _u_out) in enumerate(moves):
        v0, v1 = entry[i], entry[i + 1]
        d_acc = (nominal ** 2 - v0 ** 2) / (2 * accel)
        d_dec = (nominal ** 2 - v1 ** 2) / (2 * accel)
        if d_acc + d_dec > length:
            # Never reaches the programmed feed - accelerate straight into the deceleration
            peak = max(math.sqrt((2 * accel * length + v0 ** 2 + v1 ** 2) / 2), v0, v1)
            d_acc = min(max((peak ** 2 - v0 ** 2) / (2 * accel), 0.0), length)
            d_dec = length - d_acc
            total_time += (peak - v0) / accel + (peak - v1) / accel
        else:
            peak = nominal
            total_time += (peak - v0) / accel + (peak - v1) / accel + (length - d_acc - d_dec) / peak
        planned.append((index, length, v0, peak, v1, d_acc, d_dec))
        if cts > 0:
            # Speed along the accel phase is sqrt(v0^2 + 2as), so the part below the CTS is exact
            if peak < cts:
                below_cts += length
            else:
                below_cts += min(max((cts ** 2 - v0 ** 2) / (2 * accel), 0.0), d_acc)
                below_cts += min(max((cts ** 2 - v1 ** 2) / (2 * accel), 0.0), d_dec)
    return planned, total_time, below_cts


def plan_motion(params, variables):
    # Acceleration-aware estimate of the real machine speed, using the same model as
    # PrusaSlicer's time estimate for Marlin firmware:
    #   - every move is a trapezoid: accelerate at a constant rate, cruise, decelerate
    #   - the speed allowed through a corner comes from junction deviation and / or classic
    #     jerk (see junction_speed()), so the head keeps constant velocity unless a corner
    #     can't be taken at speed with minor rounding
    #   - a backward then forward pass makes sure every move can actually reach / slow to
    #     its neighbours' speeds within its length
    # Returns the total time (s) including G4 dwells, and stores the planned moves in
    # params["Planned_moves"] for the pixel coords and the corner path.
    dwells = params.get("Dwells", [])
    planned, move_time, below_cts = plan_moves(params["Preview_segments"], dwells, variables)
    params["Planned_moves"] = planned
    if not planned:
        return 0.0
    variables["Distance_below_CTS_mm"] = below_cts
    variables["Dwell_time_s"] = sum(d[1] for d in dwells)
    return move_time + variables["Dwell_time_s"]
