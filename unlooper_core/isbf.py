"""The ISBF lag compensation (Gcode_processing.py vector_angle / recompute_lag /
point_extraction), ported so it writes the same compensated G-code, plus a continuous
("adaptive") version that places every overshoot from the lag model itself.

vector_angle walks the unlooped code (params["One_coordinate_system"], absolute mm, y up)
one pair of commands at a time and rewrites each join so the trailing jet is dragged onto
the programmed path:

  G1 -> G1   carry on past the corner by 0.85 x the lag (Point 4), swing round the corner
             on an arc centred on it at the rapid feed and rejoin the next line the same
             distance along it (Point 5). Joins turning less than 5 degrees just carry on.
             Nothing is capped by the command lengths: a rejoin point further along than
             the next command runs on along the same line.
  G1 -> G2/3 extend the line by the lag, then arc round onto the start of the arc
  G2/3 -> G1 end the arc a lag early / late (centre moved to suit), then arc onto the line
  G2/3 -> G2/3  end the arc a lag early / late, rapid across to the next arc if they meet
             at an angle

The lag at a G1 corner comes from the jet model on the compensated path itself, as it is
built: after each join the new compensated moves plus the next original command (from
where the nozzle now is) are run through the lag model at constant speed, and the lag at
the end is used for the next corner. Around G2/G3 arcs the lag is the reset distance
fitted to the measured arc data (recompute_lag).

mode="continuous" (the pointwise method's adaptive spacing, on a Program cut into points)
keeps the geometry but drops the look-ahead: the jet model follows the compensated nozzle
path exactly as written, sample by sample, moving as the planner will run it (accelerating
at the limit, Marlin's junction speeds, the swing capped at sqrt(accel x radius)). At each
G1 corner the overshoot is the one that, simulated through the swing and back onto the
next line, keeps the jet closest to the path - a grid and golden-section search on the lag
model. Along a line the nozzle leads the jet by factor x its lag, but never past the next
corner (that overshoot is solved there).

What changed from vector_angle (the geometry is the same):
  - only the G1 -> G1 geometry that ran is kept: vector_angle also had Point 4 measured
    from the last Point 5, but forced Command_length_new = 0 so it never ran
  - the lag model runs on the moves directly instead of via line_reader / python_lag's
    arrays (same maths as lag_model.py), sampled every scatter_resolution at the feed rate
    with the remainder carried from move to move as doline() did; runs of G1 -> G1 joins
    are done in one compiled loop
  - G4 dwells are kept where they were (vector_angle dropped them)
  - `extra` / `arc_extra` add to the overshoot / arc reset distance at given joins, for the
    correction passes in lag_compensation.py
  - runs of G1 pieces along one line can be written as one move (`merge`)
"""
import math
import re
from functools import lru_cache

import numpy as np

from .lag_model import objective

try:
    from numba import njit
except ImportError:  # pragma: no cover - numba is optional, just slower without it
    njit = None


def _jit(f):
    return njit(cache=True, nogil=True)(f) if njit is not None else f


NUMBER_RE = re.compile(r"[^\W\d_]+|[-+]?(?:\d*\.*\d+)")
ROUND_NUM = 5
ANGLE_BOUND = 5
RAPID_FEEDRATE = 3000
LAG_LENGTH_REDUCTION_FACTOR = 0.85  # "0.85 used for a SR of 1.3 and 1.5, 1.0 used for max speed"


@lru_cache(maxsize=None)
def _tokens_cached(line):
    return tuple(NUMBER_RE.findall(line))


# --- vector_angle's helpers, as they were ------------------------------------------------------

def angle_calcualtion(point_1, point_2, point_3):
    # Angle at point_2 in degrees (0 - 180), signed as vector_angle signed it; nan if two
    # of the points coincide
    ax, ay = point_1[0] - point_2[0], point_1[1] - point_2[1]
    bx, by = point_3[0] - point_2[0], point_3[1] - point_2[1]
    sign = -1 if (by < 0 or ay < 0) else 1
    na, nb = math.hypot(ax, ay), math.hypot(bx, by)
    if na == 0 or nb == 0:
        return math.nan
    cosine = max(-1.0, min(1.0, (ax * bx + ay * by) / (na * nb)))
    return round(sign * math.degrees(math.acos(cosine)), 10)


def direction_of_point(a_x, a_y, b_x, b_y, p_x, p_y):
    # 1 if P is to the right of A -> B, -1 to the left, 0 on the line
    cross_product = (b_x - a_x) * (p_y - a_y) - (b_y - a_y) * (p_x - a_x)
    if cross_product > 0:
        return 1
    if cross_product < 0:
        return -1
    return 0


def point_extraction(command_array, point_num, lag_length):
    # Point 1 / 2 (from the current command) or Point 3 (from the next). For a line, Point 1
    # is its start, Point 2 its end and Point 3 the next line's end; for an arc, Point 1 /
    # Point 3 are a lag along the tangent from its end / start.
    point_1, point_2, point_3 = [0, 0], [0, 0], [0, 0]
    decimal_place = 10
    last = command_array[-1]
    if point_num == 2:
        return point_1, [float(command_array[3]), float(command_array[5])], point_3
    if last in ("0", "1"):
        if point_num == 1:
            return [float(command_array[8]), float(command_array[9])], point_2, point_3
        return point_1, point_2, [float(command_array[3]), float(command_array[5])]
    length = math.sqrt(float(command_array[7]) ** 2 + float(command_array[9]) ** 2)
    a_x = round(float(command_array[3]), decimal_place)
    a_y = round(float(command_array[5]), decimal_place)
    b_x = round(float(command_array[12]), decimal_place)
    b_y = round(float(command_array[13]), decimal_place)
    temp_x = round(float(command_array[14]), 5)
    temp_y = round(float(command_array[15]), 5)
    side = direction_of_point(a_x, a_y, b_x, b_y, temp_x, temp_y)
    diff = float(command_array[16])
    if last == "2" and (side == 0 or 179.999 < diff < 180.001):
        side = 1
    if last == "3" and (side == 0 or 179.999 < diff < 180.001):
        side = -1
    if not diff > 180:
        side = side * -1
    o_x, o_y = (a_x, a_y) if point_num == 1 else (b_x, b_y)
    # vector_angle tried the four sign combinations of the tangent offset and took the
    # first at 90 degrees to the radius on the right side. Only (+, +) and (-, -) are at
    # 90 degrees (the mixed ones only where they equal one of these), and none are with no
    # lag, so those two decide it.
    if lag_length != 0 and length != 0:
        for sgn in (1, -1):
            p_x = round(o_x + sgn * ((lag_length * (temp_y - o_y)) / length), decimal_place)
            p_y = round(o_y + sgn * ((lag_length * (o_x - temp_x)) / length), decimal_place)
            if direction_of_point(a_x, a_y, b_x, b_y, p_x, p_y) == side:
                angle = angle_calcualtion([temp_x, temp_y], [o_x, o_y], [p_x, p_y])
                if angle == angle and abs(round(angle, 5)) == 90:
                    return ([p_x, p_y], point_2, point_3) if point_num == 1 else (point_1, point_2, [p_x, p_y])
    return point_1, point_2, point_3


def _two_phase(y0, plateau, percent_fast, k_fast, k_slow, x):
    # GraphPad's two-phase decay, as the fits were made
    span_fast = (y0 - plateau) * percent_fast * 0.01
    span_slow = (y0 - plateau) * (100 - percent_fast) * 0.01
    return plateau + span_fast * math.exp(-k_fast * x) + span_slow * math.exp(-k_slow * x)


# Reset distance round an arc: B0 + B1 x angle + B2 x angle^2, each B a two-phase decay in
# the speed ratio whose parameters are quadratics in the pitch (recompute_lag)
_ARC_B = {
    "B0": {"Y0": (-3.768, -0.2116, -0.6346), "Plateau": (4.197, 1.293, -0.2974), "PercentFast": (47.01, 9.122, -0.8932),
           "KFast": (1.776, -0.5834, 0.2507), "KSlow": (0.12, 0.008734, 0.005446)},
    "B1": {"Y0": (-0.07572, 0.07423, -0.03469), "Plateau": (0.0327, -0.01374, 0.003273), "PercentFast": (68.42, -1.997, 3.551),
           "KFast": (2.499, -1.784, 0.8658), "KSlow": (0.1881, 0.05385, 0.02249)},
    "B2": {"Y0": (-0.000216, 0.0002131, -0.00009791), "Plateau": (0.00009078, -0.00003802, 0.00000905),
           "PercentFast": (67.57, -0.09092, 2.894), "KFast": (2.553, -1.912, 0.9167), "KSlow": (0.2006, 0.02626, 0.03219)},
}


def recompute_lag(command_array, speed_ratio):
    # Reset distance (mm) for the arc at this speed ratio (vector_angle only ever called
    # recompute_lag for arcs)
    return _recompute_lag(tuple(command_array[3:17]), speed_ratio)


@lru_cache(maxsize=65536)
def _recompute_lag(command_array, speed_ratio):
    # (command_array here starts at index 3 of vector_angle's: X, Y, I, J, F, prev, centre, Diff)
    command_array = ("", "", "") + command_array
    pitch = math.sqrt((float(command_array[12]) - float(command_array[3])) ** 2
                      + (float(command_array[13]) - float(command_array[5])) ** 2)
    arc_angle = float(command_array[16])
    b = {}
    for name, fit in _ARC_B.items():
        p = {key: c0 + c1 * pitch + c2 * pitch * pitch for key, (c0, c1, c2) in fit.items()}
        b[name] = _two_phase(p["Y0"], p["Plateau"], p["PercentFast"], p["KFast"], p["KSlow"], speed_ratio)
    b["B1"] = -b["B1"]
    return abs(b["B2"] * arc_angle ** 2 + b["B1"] * arc_angle + b["B0"])


def _lerp(t, p, q):
    # (1 - t) * p + t * q, rounded as vector_angle did
    return [round((1 - t) * p[0] + t * q[0], ROUND_NUM), round((1 - t) * p[1] + t * q[1], ROUND_NUM)]


def _dist(p, q):
    return math.sqrt((p[0] - q[0]) ** 2 + (p[1] - q[1]) ** 2)


def _fmt(v):
    # As vector_angle wrote numbers (str()), but never in exponent form (1e-05), which a
    # G-code reader can't take
    text = str(v)
    if "e" in text or "E" in text:
        text = f"{v:.6f}".rstrip("0").rstrip(".")
    return text


def _feed_of(command_array):
    if len(command_array) > 11 and command_array[10] == "F":
        return float(command_array[11])
    if len(command_array) > 7 and command_array[6] == "F":
        return float(command_array[7])
    return 0.0


# --- the jet model along the compensated moves (compiled) ---------------------------------------
#
# State array: [contact x, contact y, lag, nozzle x, nozzle y, s since the last sample, modal
# feed mm/min, nozzle speed mm/s]. vector_angle's moves put a nozzle position every dt_s
# along each move at its feed (no acceleration, as line_reader's pixel coords), the remainder
# carried from move to move like doline(). The continuous mode's moves (_sim_move) follow the
# planner instead: accelerate / decelerate at the acceleration limit into Marlin's junction
# speed at each end, as the compensated file will run. With
# `run` off the positions are read but the jet isn't moved (vector_angle only ran the lag
# model after a join into a G1). `trk` follows the jet near a corner while solving for its
# overshoot (see _track).

def _r(x, n):
    # round(x, n), as vector_angle rounded
    return round(x, n)


def _step(st, gx, gy, js, dt_min, a, b, eps):
    # One lag model step (lag_model._lag_steps for one nozzle position)
    rx = gx - st[0]
    ry = gy - st[1]
    ray = math.sqrt(rx * rx + ry * ry)
    lag = st[2]
    sr = (ray - lag) / (dt_min * js)
    base = (lag + a) / a
    fall_speed = js * base ** (1.0 / b) if base > 0 else math.nan
    falling = ray - dt_min * fall_speed
    if sr > 1:
        stationary = a * sr ** b - a
        if ray > stationary:
            new_lag = math.nan if (stationary != stationary or falling != falling) else max(stationary, falling)
        else:
            new_lag = math.nan if falling != falling else max(falling, 0.0)
    elif lag > eps:
        new_lag = math.nan if falling != falling else max(falling, 0.0)
    else:
        new_lag = 0.0
    if ray > eps:
        f = new_lag / ray
        st[0] = gx - rx * f
        st[1] = gy - ry * f
    else:
        st[0] = gx
        st[1] = gy
    st[2] = new_lag


def _track(trk, cx, cy):
    # trk: [corner x, y, incoming direction x, y, outgoing direction x, y, sum of distances
    # from the local path, samples]. The local path is the incoming line up to the corner
    # and the outgoing line from it.
    rx = cx - trk[0]
    ry = cy - trk[1]
    t_in = rx * trk[2] + ry * trk[3]  # > 0: past the corner along the incoming line
    if t_in <= 0:
        d_in = abs(rx * trk[3] - ry * trk[2])
    else:
        d_in = math.sqrt(rx * rx + ry * ry)
    t_out = rx * trk[4] + ry * trk[5]
    if t_out >= 0:
        d_out = abs(rx * trk[5] - ry * trk[4])
    else:
        d_out = math.sqrt(rx * rx + ry * ry)
    trk[6] += min(d_in, d_out)
    trk[7] += 1.0


def _sim_line(st, x, y, feed, dt_s, js, a, b, eps, run, trk, tracking):
    if feed <= 0:
        feed = st[6]
    st[6] = feed
    speed = feed / 60.0
    x0, y0 = st[3], st[4]
    length = math.sqrt((x - x0) ** 2 + (y - y0) ** 2)
    if length > 0 and speed > 0:
        total = length / speed
        t0 = dt_s - st[5]
        k = 0
        while True:
            t = t0 + k * dt_s
            if t > total + 1e-12:
                break
            if run:
                f = t * speed / length
                _step(st, x0 + (x - x0) * f, y0 + (y - y0) * f, js, dt_s / 60.0, a, b, eps)
                if tracking:
                    _track(trk, st[0], st[1])
            k += 1
        st[5] = (st[5] + total) % dt_s
    st[3] = x
    st[4] = y
    st[7] = speed


def _sim_arc(st, x, y, i, j, ccw, feed, dt_s, js, a, b, eps, run, trk, tracking):
    # G2 / G3 from the nozzle position: centre = start + (I, J); the end is taken as given
    if feed <= 0:
        feed = st[6]
    st[6] = feed
    speed = feed / 60.0
    cx, cy = st[3] + i, st[4] + j
    r = math.sqrt(i * i + j * j)
    a1 = math.atan2(st[4] - cy, st[3] - cx)
    a2 = math.atan2(y - cy, x - cx)
    two_pi = 2 * math.pi
    sweep = (a2 - a1) % two_pi if ccw else -((a1 - a2) % two_pi)
    if sweep == 0:
        sweep = two_pi if ccw else -two_pi
    length = abs(sweep) * r
    if length > 0 and speed > 0:
        total = length / speed
        t0 = dt_s - st[5]
        k = 0
        while True:
            t = t0 + k * dt_s
            if t > total + 1e-12:
                break
            if run:
                ang = a1 + sweep * t * speed / length
                _step(st, cx + r * math.cos(ang), cy + r * math.sin(ang), js, dt_s / 60.0, a, b, eps)
                if tracking:
                    _track(trk, st[0], st[1])
            k += 1
        st[5] = (st[5] + total) % dt_s
    st[3] = x
    st[4] = y
    st[7] = speed


def _junction_speed(uox, uoy, uix, uiy, limit, accel, deviation, jerk, min_speed):
    # motion_planner.junction_speed: fastest speed (mm/s) a move ending in direction u_out
    # can hand over to one starting in direction u_in
    cos_theta = -(uox * uix + uoy * uiy)
    if cos_theta < -0.999999:
        return limit
    if deviation <= 0 and jerk <= 0:
        return 0.0
    speed = limit
    if deviation > 0:
        if cos_theta > 0.999999:
            speed = min(speed, min_speed)
        else:
            sin_half = math.sqrt(0.5 * (1.0 - cos_theta))
            speed = min(speed, math.sqrt(accel * deviation * sin_half / (1.0 - sin_half)))
    if jerk > 0:
        change = max(abs(uix - uox), abs(uiy - uoy))
        if change > 0:
            speed = min(speed, jerk / change)
    return speed


def _sim_move(st, is_arc, x, y, cx, cy, ccw, v_nom, v_end, accel, dt_s, js, a, b, eps, trk, tracking):
    # A line (or an arc about (cx, cy)) from the nozzle position to (x, y), run as the
    # planner would: from the current speed towards v_nom, at most `accel`, arriving at no
    # more than v_end. Nozzle positions every dt_s go through the lag model.
    x0, y0 = st[3], st[4]
    if is_arc:
        r = math.sqrt((x0 - cx) ** 2 + (y0 - cy) ** 2)
        a1 = math.atan2(y0 - cy, x0 - cx)
        a2 = math.atan2(y - cy, x - cx)
        two_pi = 2 * math.pi
        sweep = (a2 - a1) % two_pi if ccw else -((a1 - a2) % two_pi)
        length = abs(sweep) * r
    else:
        r = 0.0
        a1 = 0.0
        sweep = 0.0
        length = math.sqrt((x - x0) ** 2 + (y - y0) ** 2)
    if length <= 0:
        st[3] = x
        st[4] = y
        return
    v = st[7]
    if accel <= 0:
        v = v_nom
    s = 0.0
    h = dt_s - st[5]
    while True:
        if accel > 0:
            v_allowed = math.sqrt(v_end * v_end + 2.0 * accel * max(length - s, 0.0))
            v_new = min(v_nom, v + accel * h, v_allowed)
            v_new = max(v_new, v - accel * h, 0.0)
        else:
            v_new = v_nom
        v_avg = max(0.5 * (v + v_new), 1e-9)
        ds = v_avg * h
        if s + ds >= length - 1e-12:
            tau = (length - s) / v_avg
            st[5] = (dt_s - h) + tau
            st[7] = v + (v_new - v) * tau / h if h > 0 else v_new
            break
        s += ds
        v = v_new
        if is_arc:
            ang = a1 + sweep * s / length
            gx = cx + r * math.cos(ang)
            gy = cy + r * math.sin(ang)
        else:
            gx = x0 + (x - x0) * s / length
            gy = y0 + (y - y0) * s / length
        _step(st, gx, gy, js, dt_s / 60.0, a, b, eps)
        if tracking:
            _track(trk, st[0], st[1])
        h = dt_s
    st[3] = x
    st[4] = y


def _corner_cost(st0, d, vx, vy, ux, uy, uix, uiy, ccw, feed, next_feed, rapid, rejoin, dt_s, js, a, b, eps, accel,
                 deviation, jerk, min_speed):
    # Mean distance (mm) of the jet from the path around a corner for an overshoot d: carry
    # on to the corner + d, swing round it, rejoin the next line and run `rejoin` mm along
    # it, moving as the planner would - the local version of what the scoring measures
    st = st0.copy()
    trk = np.zeros(8)
    trk[0] = vx
    trk[1] = vy
    trk[2] = ux
    trk[3] = uy
    trk[4] = uix
    trk[5] = uiy
    _swing(st, d, vx, vy, ux, uy, uix, uiy, ccw, feed, next_feed, rapid, dt_s, js, a, b, eps, accel, deviation,
           jerk, min_speed, trk, True)
    v_next = next_feed / 60.0
    _sim_move(st, False, vx + uix * (d + rejoin), vy + uiy * (d + rejoin), 0.0, 0.0, False, v_next, v_next, accel,
              dt_s, js, a, b, eps, trk, True)
    return trk[6] / max(trk[7], 1.0)


def _swing(st, d, vx, vy, ux, uy, uix, uiy, ccw, feed, next_feed, rapid, dt_s, js, a, b, eps, accel, deviation, jerk,
           min_speed, trk, tracking):
    # The nozzle to the corner + d along this line, then round the corner (centre = corner)
    # to the corner + d along the next line, with the planner's speeds: the swing is capped
    # at the rapid feed and at sqrt(accel x d), and its ends are ~90 degree junctions
    p4x = vx + ux * d
    p4y = vy + uy * d
    p5x = vx + uix * d
    p5y = vy + uiy * d
    v_arc = rapid / 60.0
    if accel > 0:
        v_arc = min(v_arc, math.sqrt(accel * d))
    sgn = 1.0 if ccw else -1.0
    # Tangents where the swing starts / ends (counter-clockwise = +90 degrees from the radius)
    t0x = -sgn * uy
    t0y = sgn * ux
    t1x = -sgn * uiy
    t1y = sgn * uix
    v_in = _junction_speed(ux, uy, t0x, t0y, min(feed / 60.0, v_arc), accel, deviation, jerk, min_speed)
    v_out = _junction_speed(t1x, t1y, uix, uiy, min(v_arc, next_feed / 60.0), accel, deviation, jerk, min_speed)
    _sim_move(st, False, p4x, p4y, 0.0, 0.0, False, feed / 60.0, v_in, accel, dt_s, js, a, b, eps, trk, tracking)
    if d > 1e-5:
        _sim_move(st, True, p5x, p5y, vx, vy, ccw, v_arc, v_out, accel, dt_s, js, a, b, eps, trk, tracking)


def _solve_overshoot(st, lo, vx, vy, ux, uy, uix, uiy, ccw, feed, next_feed, rapid, dt_s, js, a, b, eps, accel,
                     deviation, jerk, min_speed):
    # The overshoot with the jet closest to the path around the corner (_corner_cost): a
    # coarse grid from where the nozzle already is out to two lags past it, then a golden-
    # section search round the best grid point
    hi = lo + 2.0 * max(st[2], 0.1) + 0.5
    rejoin = 1.0 + st[2]
    n_grid = 8
    best_d = lo
    best_c = 1e300
    for g in range(n_grid + 1):
        dg = lo + (hi - lo) * g / n_grid
        cg = _corner_cost(st, dg, vx, vy, ux, uy, uix, uiy, ccw, feed, next_feed, rapid, rejoin, dt_s, js, a, b, eps,
                          accel, deviation, jerk, min_speed)
        if cg < best_c:
            best_c = cg
            best_d = dg
    g_lo = max(lo, best_d - (hi - lo) / n_grid)
    g_hi = min(hi, best_d + (hi - lo) / n_grid)
    ratio = 0.6180339887498949
    c1 = g_hi - ratio * (g_hi - g_lo)
    c2 = g_lo + ratio * (g_hi - g_lo)
    f1 = _corner_cost(st, c1, vx, vy, ux, uy, uix, uiy, ccw, feed, next_feed, rapid, rejoin, dt_s, js, a, b, eps,
                      accel, deviation, jerk, min_speed)
    f2 = _corner_cost(st, c2, vx, vy, ux, uy, uix, uiy, ccw, feed, next_feed, rapid, rejoin, dt_s, js, a, b, eps,
                      accel, deviation, jerk, min_speed)
    for _ in range(8):
        if f1 < f2:
            g_hi = c2
            c2 = c1
            f2 = f1
            c1 = g_hi - ratio * (g_hi - g_lo)
            f1 = _corner_cost(st, c1, vx, vy, ux, uy, uix, uiy, ccw, feed, next_feed, rapid, rejoin, dt_s, js, a, b,
                              eps, accel, deviation, jerk, min_speed)
        else:
            g_lo = c1
            c1 = c2
            f1 = f2
            c2 = g_lo + ratio * (g_hi - g_lo)
            f2 = _corner_cost(st, c2, vx, vy, ux, uy, uix, uiy, ccw, feed, next_feed, rapid, rejoin, dt_s, js, a, b,
                              eps, accel, deviation, jerk, min_speed)
    if best_c <= min(f1, f2):
        return best_d
    return c1 if f1 < f2 else c2


def _is_swing(p1x, p1y, p2x, p2y, p3x, p3y):
    # vector_angle's test: the angle at Point 2 between 5 and 175 degrees
    ax, ay = p1x - p2x, p1y - p2y
    bx, by = p3x - p2x, p3y - p2y
    na = math.sqrt(ax * ax + ay * ay)
    nb = math.sqrt(bx * bx + by * by)
    if na == 0 or nb == 0:
        return False
    c = min(1.0, max(-1.0, (ax * bx + ay * by) / (na * nb)))
    theta = math.degrees(math.acos(c))
    return ANGLE_BOUND < theta < 180 - ANGLE_BOUND


def _g1_joins(p, extra, fixed, st, lag_length, factor, rapid, dt_s, js, a, b, eps, mode, out, accel, deviation, jerk,
              min_speed):
    # A run of G1 -> G1 joins. p rows: P1 x, y, P2 x, y, P3 x, y (Point 1 - 3), feed of this
    # command, the next command as a relative move (dx, dy, feed). out rows: P4 x, y, arc
    # written, P5 x, y, G3, I, J, overshoot, swing. Returns the lag for the next join and
    # the lag used for the last one.
    lag_previous = lag_length
    trk = np.zeros(8)
    n = p.shape[0]
    # Continuous mode: where the next swing is along the run, so a lead taken along the
    # line never carries the nozzle past that corner (its overshoot is solved there)
    next_swing = np.full(n, -1, dtype=np.int64)
    upcoming = -1
    for k in range(n - 1, -1, -1):
        next_swing[k] = upcoming
        if _is_swing(p[k, 0], p[k, 1], p[k, 2], p[k, 3], p[k, 4], p[k, 5]):
            upcoming = k
    for k in range(n):
        p1x, p1y, p2x, p2y, p3x, p3y, feed_1 = p[k, 0], p[k, 1], p[k, 2], p[k, 3], p[k, 4], p[k, 5], p[k, 6]
        lag_previous = lag_length
        ax, ay = p1x - p2x, p1y - p2y
        bx, by = p3x - p2x, p3y - p2y
        na = math.sqrt(ax * ax + ay * ay)
        nb = math.sqrt(bx * bx + by * by)
        theta = 0.0
        if na > 0 and nb > 0:
            c = (ax * bx + ay * by) / (na * nb)
            c = min(1.0, max(-1.0, c))
            theta = math.degrees(math.acos(c))
        swing = ANGLE_BOUND < theta < 180 - ANGLE_BOUND
        ccw = ((p2x - p1x) * (p3y - p1y) - (p2y - p1y) * (p3x - p1x)) > 0
        out[k, 9] = 1.0 if swing else 0.0
        out[k, 5] = 1.0 if ccw else 0.0
        out[k, 2] = 0.0
        if mode == 0:
            # vector_angle
            d = lag_length * factor + extra[k]
            t4 = -d / na if na != 0 else 1.0
            p4x = _r((1 - t4) * p2x + t4 * p1x, ROUND_NUM)
            p4y = _r((1 - t4) * p2y + t4 * p1y, ROUND_NUM)
            out[k, 0] = p4x
            out[k, 1] = p4y
            out[k, 8] = d
            _sim_line(st, p4x, p4y, feed_1, dt_s, js, a, b, eps, True, trk, False)
            if swing:
                t5 = d / nb if nb != 0 else 1.0
                p5x = _r((1 - t5) * p2x + t5 * p3x, ROUND_NUM)
                p5y = _r((1 - t5) * p2y + t5 * p3y, ROUND_NUM)
                out[k, 3] = p5x
                out[k, 4] = p5y
                if lag_length != 0:
                    i = _r(p2x - p4x, ROUND_NUM)
                    j = _r(p2y - p4y, ROUND_NUM)
                    out[k, 2] = 1.0
                    out[k, 6] = i
                    out[k, 7] = j
                    _sim_arc(st, p5x, p5y, i, j, ccw, rapid, dt_s, js, a, b, eps, True, trk, False)
            # ... and the next command, from where the nozzle now is
            _sim_line(st, st[3] + p[k, 7], st[4] + p[k, 8], p[k, 9], dt_s, js, a, b, eps, True, trk, False)
            lag_length = _r(st[2], 4)
            continue
        # Continuous: the jet model follows the nozzle as written
        if na == 0:
            out[k, 0] = st[3]
            out[k, 1] = st[4]
            continue
        ux, uy = -ax / na, -ay / na
        x0 = (st[3] - p2x) * ux + (st[4] - p2y) * uy  # how far past the corner the nozzle already is
        if swing and nb > 0:
            uix, uiy = bx / nb, by / nb
            nf = p[k, 9]
            if fixed[k] == fixed[k]:
                d = fixed[k]  # solved before (a correction pass)
            else:
                d = _solve_overshoot(st, max(x0, 0.0), p2x, p2y, ux, uy, uix, uiy, ccw, feed_1, nf, rapid, dt_s, js, a,
                                     b, eps, accel, deviation, jerk, min_speed)
            d = max(d + extra[k], max(x0, 0.0))
            p4x = _r(p2x + ux * d, ROUND_NUM)
            p4y = _r(p2y + uy * d, ROUND_NUM)
            p5x = _r(p2x + uix * d, ROUND_NUM)
            p5y = _r(p2y + uiy * d, ROUND_NUM)
            out[k, 0] = p4x
            out[k, 1] = p4y
            out[k, 3] = p5x
            out[k, 4] = p5y
            out[k, 8] = d
            _swing(st, d, p2x, p2y, ux, uy, uix, uiy, ccw, feed_1, nf, rapid, dt_s, js, a, b, eps, accel, deviation,
                   jerk, min_speed, trk, False)
            st[3] = p5x if d > 1e-5 else p4x
            st[4] = p5y if d > 1e-5 else p4y
            st[6] = rapid if d > 1e-5 else feed_1
            if d > 1e-5:
                out[k, 2] = 1.0
                out[k, 6] = _r(p2x - p4x, ROUND_NUM)
                out[k, 7] = _r(p2y - p4y, ROUND_NUM)
        else:
            # No swing: pass the join where the nozzle is factor x the jet's lag beyond it,
            # checked at every sample (the first sample comes when the carry says)
            speed = feed_1 / 60.0
            qx, qy = st[3], st[4]
            x = x0
            cap = 1e300
            if next_swing[k] >= 0:
                j = next_swing[k]
                cap = (p[j, 2] - p2x) * ux + (p[j, 3] - p2y) * uy
            if speed > 0 and x < min(factor * st[2] + extra[k], cap):
                h = dt_s - st[5]
                v = st[7]
                limit = x0 + 100.0
                while True:
                    v_new = min(speed, v + accel * h) if accel > 0 else speed
                    ds = 0.5 * (v + v_new) * h
                    v = v_new
                    qx += ux * ds
                    qy += uy * ds
                    x += ds
                    _step(st, qx, qy, js, dt_s / 60.0, a, b, eps)
                    h = dt_s
                    if x >= min(factor * st[2] + extra[k], cap) or x >= limit:
                        break
                st[5] = 0.0
                st[7] = v
            p4x = _r(qx, ROUND_NUM)
            p4y = _r(qy, ROUND_NUM)
            st[3] = p4x
            st[4] = p4y
            st[6] = feed_1
            out[k, 0] = p4x
            out[k, 1] = p4y
            out[k, 8] = x
        lag_length = st[2]
    return lag_length, lag_previous


for _name in ("_r", "_step", "_track", "_sim_line", "_sim_arc", "_junction_speed", "_sim_move", "_swing",
              "_corner_cost", "_solve_overshoot", "_is_swing", "_g1_joins"):
    globals()[_name] = _jit(globals()[_name])


# --- the program vector_angle reads ---------------------------------------------------------------

class Program:
    # The unlooped commands as vector_angle reads them (One_coordinate_system lines: absolute
    # mm, y up, with the start / centre after the ';'), plus the numbers the G1 joins need as
    # arrays. split() cuts every G1 into short pieces for the pointwise method; a piece's
    # text and tokens are only made if an arc join asks for them.

    def __init__(self, lines=None):
        if lines is None:
            return
        self._lines = list(lines)
        self._toks = [_tokens_cached(line) for line in self._lines]
        self.n = len(self._toks)
        self.kind = np.array([int(t[-1]) for t in self._toks], dtype=np.int64)
        self.is_g1 = self.kind <= 1
        self.x = np.array([float(t[3]) for t in self._toks])
        self.y = np.array([float(t[5]) for t in self._toks])
        self.px = np.array([float(t[8] if g else t[12]) for t, g in zip(self._toks, self.is_g1)])
        self.py = np.array([float(t[9] if g else t[13]) for t, g in zip(self._toks, self.is_g1)])
        self.feed = np.array([_feed_of(t) for t in self._toks])

    def __len__(self):
        return self.n

    def tokens(self, i):
        t = self._toks[i]
        if t is None:
            t = _tokens_cached(self.text(i))
            self._toks[i] = t
        return t

    def text(self, i):
        line = self._lines[i]
        if line is None:
            line = (f"G1 X{_fmt(round(self.x[i], 6))} Y{_fmt(round(self.y[i], 6))} F{_fmt(self.feed[i])} ; "
                    f"{_fmt(round(self.px[i], 6))} {_fmt(round(self.py[i], 6))} {self.kind[i]}")
            self._lines[i] = line
        return line

    def split(self, spacing_mm=None, dt_s=None, growth=0.25, max_mm=1.0):
        # Every G1 cut into pieces (arcs stay whole). With spacing_mm the pieces are at most
        # that long; without it they are graded: one nozzle sample's travel at the feed
        # (feed x dt_s) at each end of the line, where the lag changes and the corner is
        # decided, each piece `growth` longer than the one before towards the middle, where
        # the lag has settled, up to max_mm. Returns the new program, the command each piece
        # came from and the first piece of each command.
        length = np.hypot(self.x - self.px, self.y - self.py)
        fractions = []
        for i in range(self.n):
            if not self.is_g1[i] or length[i] <= 0:
                fractions.append(np.ones(1))
                continue
            if spacing_mm is not None:
                n = max(1, int(math.ceil(length[i] / spacing_mm)))
                fractions.append(np.arange(1, n + 1) / n)
                continue
            h0 = max(self.feed[i] / 60.0 * dt_s, 1e-4)
            steps = np.minimum(h0 * (1.0 + growth) ** np.arange(64), max_mm)
            half = np.cumsum(steps)
            half = half[half < 0.5 * length[i]]
            cuts = np.concatenate((half, length[i] - half[::-1], [length[i]]))
            fractions.append(cuts / length[i])
        counts = np.array([len(f) for f in fractions], dtype=np.int64)
        idx = np.repeat(np.arange(self.n), counts)
        first = np.cumsum(counts) - counts
        frac = np.concatenate(fractions)
        frac_prev = np.where(np.arange(len(idx)) == first[idx], 0.0, np.roll(frac, 1))
        dx, dy = self.x[idx] - self.px[idx], self.y[idx] - self.py[idx]
        out = Program()
        out.n = len(idx)
        out.kind = self.kind[idx]
        out.is_g1 = self.is_g1[idx]
        out.feed = self.feed[idx]
        out.x = np.where(frac == 1.0, self.x[idx], self.px[idx] + dx * frac)
        out.y = np.where(frac == 1.0, self.y[idx], self.py[idx] + dy * frac)
        out.px = np.where(frac_prev == 0.0, self.px[idx], self.px[idx] + dx * frac_prev)
        out.py = np.where(frac_prev == 0.0, self.py[idx], self.py[idx] + dy * frac_prev)
        single = counts[idx] == 1
        out._lines = [self._lines[i] if one else None for i, one in zip(idx.tolist(), single.tolist())]
        out._toks = [self._toks[i] if one else None for i, one in zip(idx.tolist(), single.tolist())]
        return out, idx, first


# --- vector_angle ---------------------------------------------------------------------------------

def vector_angle(params, variables, js, a, b, factor=LAG_LENGTH_REDUCTION_FACTOR, rapid=RAPID_FEEDRATE,
                 extra=None, arc_extra=None, mode="isbf", fixed=None, merge=False):
    # Returns the lag-compensated G-code lines (vector_angle's lag_compensated_code, with the
    # consecutive duplicate lines removed as it did), the G1 corners it swung round -
    # (command index, Point 1, Point 2 = the corner, Point 3, overshoot mm) - and the arc
    # joins it compensated (command index of the arc before the join). `extra` /
    # `arc_extra` add mm to the overshoot / reset distance at given joins (by command index).
    # `fixed` gives the continuous mode's corner overshoots instead of solving them again.
    # params["One_coordinate_system"] may be a Program already (the pointwise method's pieces).
    # `merge`: write runs of G1 pieces along one line as one move.
    extra = extra or {}
    fixed = fixed or {}
    arc_extra = arc_extra or {}
    continuous = mode == "continuous"
    corners, arc_joins = [], []
    ocs = params["One_coordinate_system"]
    prog = ocs if isinstance(ocs, Program) else Program(ocs)
    n_cmd = len(prog)
    dwell_at = {}
    for d in params.get("Dwells", []):
        dwell_at.setdefault(int(d[2]), []).append(float(d[1]))
    dt_s = float(variables["scatter_resolution"])
    code = ["G90", "G21", "G17"]
    if n_cmd < 2:
        return code + [prog.text(i).split(";")[0].strip() for i in range(n_cmd)], corners, arc_joins
    is_g1 = prog.is_g1
    ex_all = np.zeros(n_cmd)
    for i, v in extra.items():
        ex_all[i] = v
    fx_all = np.full(n_cmd, np.nan)
    for i, v in fixed.items():
        fx_all[i] = v

    # Lag for the first command: the jet model along it at the file's feed rate (K0), jet
    # starting Lag(K0) behind the first point along the first direction of travel
    cs = round(float(variables.get("global_return_feedrate", 0.0)) * 60, 2) or float(prog.feed[0])
    start = (float(prog.px[0]), float(prog.py[0]))
    first_end = (float(prog.x[0]), float(prog.y[0]))
    eps = max(cs / 60.0 * dt_s / 50.0, 1e-9)
    lag0 = max(objective(cs / js, a, b), 0.0)
    dx, dy = first_end[0] - start[0], first_end[1] - start[1]
    norm = math.hypot(dx, dy) or 1.0
    initial = np.array([start[0] - dx / norm * lag0, start[1] - dy / norm * lag0, lag0, start[0], start[1], 0.0, cs,
                        cs / 60.0])
    accel = float(variables.get("Acceleration_mm_s2", 0.0))
    deviation = float(variables.get("Junction_deviation_mm", 0.0))
    jerk = float(variables.get("Jerk_mm_s", 0.0))
    min_speed = float(variables.get("Minimum_planner_speed_mm_s", 0.05))
    st = initial.copy()
    no_track = np.zeros(8)
    _feed_line(st, prog.text(0).split(";")[0], True, dt_s, js, a, b, eps, no_track)
    lag = _r(st[2], 4) if not continuous else st[2]
    # ... then the model starts again from the beginning for the compensated path
    st = initial.copy()

    lag_length = lag_previous = lag
    lower, upper = ANGLE_BOUND, 180 - ANGLE_BOUND
    command_array_1 = list(prog.tokens(0))
    written = len(code)
    line_number = 0
    while line_number < n_cmd - 1:
        for seconds in dwell_at.pop(line_number, ()):
            code.append(f"G4 P{seconds * 1000:.0f}")
            written = len(code)
            st[7] = 0.0  # a dwell is a stop

        if is_g1[line_number] and is_g1[line_number + 1]:
            # A run of G1 -> G1 joins, in one go
            end = line_number + 1
            while end < n_cmd - 1 and is_g1[end + 1] and end not in dwell_at:
                end += 1
            sl, nx = slice(line_number, end), slice(line_number + 1, end + 1)
            p = np.column_stack((prog.px[sl], prog.py[sl], prog.x[sl], prog.y[sl], prog.x[nx], prog.y[nx], prog.feed[sl],
                                 prog.x[nx] - prog.px[nx], prog.y[nx] - prog.py[nx], prog.feed[nx]))
            out = np.zeros((end - line_number, 10))
            lag_length, lag_previous = _g1_joins(p, ex_all[sl].copy(), fx_all[sl].copy(), st, lag_length, factor, rapid,
                                                 dt_s, js, a, b, eps, 1 if continuous else 0, out, accel, deviation,
                                                 jerk, min_speed)
            arc = out[:, 2] != 0
            keep = np.ones(len(out), dtype=bool)
            if merge and len(out) >= 3:
                # A piece's end on the straight line between its neighbours' (same feed, no
                # swing either side) needn't be written
                ax_, ay_ = out[:-2, 0], out[:-2, 1]
                bx_, by_ = out[1:-1, 0], out[1:-1, 1]
                cx_, cy_ = out[2:, 0], out[2:, 1]
                ac = np.hypot(cx_ - ax_, cy_ - ay_)
                cross = np.abs((cx_ - ax_) * (by_ - ay_) - (cy_ - ay_) * (bx_ - ax_))
                forward = (bx_ - ax_) * (cx_ - bx_) + (by_ - ay_) * (cy_ - by_) > 0
                same_feed = (p[:-2, 6] == p[1:-1, 6]) & (p[1:-1, 6] == p[2:, 6])
                keep[1:-1] = ~((cross <= 5e-5 * ac) & forward & same_feed & ~arc[:-2] & ~arc[1:-1])
            for k in np.flatnonzero(keep | arc).tolist():
                if keep[k]:
                    code.append(f"G1 X{_fmt(out[k, 0])} Y{_fmt(out[k, 1])} F{_fmt(p[k, 6])}")
                if arc[k]:
                    code.append(f"G{3 if out[k, 5] else 2} X{_fmt(out[k, 3])} Y{_fmt(out[k, 4])} I{_fmt(out[k, 6])} "
                                f"J{_fmt(out[k, 7])} F{_fmt(rapid)}")
            for k in np.flatnonzero(out[:, 9]).tolist():
                corners.append((line_number + k, [p[k, 0], p[k, 1]], [p[k, 2], p[k, 3]], [p[k, 4], p[k, 5]],
                                float(out[k, 8])))
            written = len(code)
            command_array_1 = list(prog.tokens(end))
            line_number = end
            continue

        # A join into or out of an arc (vector_angle's other branches)
        t1, t2 = prog.tokens(line_number), prog.tokens(line_number + 1)
        command_1_num = float(command_array_1[-1])
        feed_1 = _feed_of(command_array_1)
        speed_ratio = feed_1 / js
        command_array_2 = list(t2)
        command_2_num = float(command_array_2[-1])
        g1_1 = bool(is_g1[line_number])
        g1_2 = bool(is_g1[line_number + 1])
        p1 = point_extraction(t1, 1, lag_length)[0]
        p2 = point_extraction(t1, 2, lag_length)[1]
        p3 = point_extraction(t2, 3, lag_length)[2]
        angle = angle_calcualtion(p1, p2, p3)
        if angle != angle:
            angle = 0
        next_command_is_g1 = False

        if g1_1 and not g1_2:
            # G1 -> G2/3: the lag for the join is the arc's reset distance
            lag_length = recompute_lag(t2, speed_ratio) + arc_extra.get(line_number, 0.0)
            arc_joins.append(line_number)
            p1 = point_extraction(t1, 1, lag_length)[0]
            p3 = point_extraction(t2, 3, lag_length)[2]
            code.append(prog.text(line_number).split(";")[0].strip())
            if abs(angle) == 0:
                g1_line_length = _dist(p2, p3)
                p4 = _lerp(lag_length / g1_line_length - 2, p2, p3)
                code.append(f"G1 X{_fmt(p4[0])} Y{_fmt(p4[1])} F{_fmt(feed_1)}")
                code.append(f"G1 X{_fmt(p3[0])} Y{_fmt(p3[1])} F{_fmt(rapid)}")
            elif abs(angle) == 180:
                code.append(f"G1 X{_fmt(p3[0])} Y{_fmt(p3[1])} F{_fmt(feed_1)}")
            else:
                p4 = _lerp(lag_length / _dist(p1, p2) + 1, p1, p2)
                code.append(f"G1 X{_fmt(p4[0])} Y{_fmt(p4[1])} F{_fmt(feed_1)}")
                side = direction_of_point(*p1, *p2, *p3)
                i_ = round(p2[0] - p4[0], ROUND_NUM)
                j_ = round(p2[1] - p4[1], ROUND_NUM)
                end = f"X{_fmt(round(p3[0], ROUND_NUM))} Y{_fmt(round(p3[1], ROUND_NUM))} I{_fmt(i_)} J{_fmt(j_)}"
                if side == 1:
                    code.append(f"G3 {end} F{_fmt(rapid)}" if command_2_num == 2 else f"G2 {end}")
                else:
                    code.append(f"G2 {end} F{_fmt(rapid)}" if command_2_num == 2 else f"G3 {end}")
            # The arc's centre relative to its new start
            command_array_2[14] = round(float(command_array_2[14]) - p3[0], ROUND_NUM)
            command_array_2[15] = round(float(command_array_2[15]) - p3[1], ROUND_NUM)

        elif not g1_1:
            # G2/3 -> G1 or G2/3: end the arc a reset distance early / late
            lag_length = recompute_lag(t1, speed_ratio) + arc_extra.get(line_number, 0.0)
            arc_joins.append(line_number)
            p1 = point_extraction(t1, 1, lag_length)[0]
            p1p = point_extraction(t1, 1, lag_previous)[0]
            p3 = point_extraction(t2, 3, lag_length)[2]
            p4 = _lerp(-lag_length / _dist(p2, p1), p2, p1)
            p4_previous = _lerp(-lag_previous / _dist(p2, p1p), p2, p1p)
            command_array_1[14] = round(float(command_array_1[14]) - ((p4_previous[0] - p4[0]) / 2), ROUND_NUM)
            command_array_1[15] = round(float(command_array_1[15]) - ((p4_previous[1] - p4[1]) / 2), ROUND_NUM)
            arc = (f"G{int(command_1_num)} X{_fmt(round(p4[0], ROUND_NUM))} Y{_fmt(round(p4[1], ROUND_NUM))} "
                   f"I{_fmt(command_array_1[14])} J{_fmt(command_array_1[15])} F{_fmt(feed_1)}")
            code.append(arc)
            if g1_2:
                if abs(angle) != 180:
                    p5 = _lerp(lag_length / _dist(p3, p2), p2, p3)
                    side = direction_of_point(*p1, *p2, *p3)
                    i_ = round(p1[0] - p2[0], ROUND_NUM)
                    j_ = round(p1[1] - p2[1], ROUND_NUM)
                    code.append(f"G{3 if side == 1 else 2} X{_fmt(p5[0])} Y{_fmt(p5[1])} I{_fmt(i_)} J{_fmt(j_)} F{_fmt(rapid)}")
                next_command_is_g1 = True
            else:
                if command_2_num == command_1_num:
                    code.append(f"G1 X{_fmt(p1[0])} Y{_fmt(p1[1])}")
                elif lower < abs(angle) < upper:
                    code.append(f"G1 X{_fmt(round(p3[0], ROUND_NUM))} Y{_fmt(round(p3[1], ROUND_NUM))} F{_fmt(rapid)}")
                command_array_2[14] = round(float(command_array_2[14]) - p3[0], ROUND_NUM)
                command_array_2[15] = round(float(command_array_2[15]) - p3[1], ROUND_NUM)

        command_array_1 = command_array_2
        lag_previous = lag_length
        # The jet model on what was just written. vector_angle then read the next command
        # from where the nozzle now is (relative) and only ran the model after a join into a
        # G1; the continuous mode runs it on everything written and nothing else.
        run = True if continuous else next_command_is_g1
        for line in code[written:]:
            if line.startswith(("G0", "G1", "G2", "G3")):
                _feed_line(st, line, run, dt_s, js, a, b, eps, no_track)
        written = len(code)
        if not continuous:
            prev_x = float(t2[8] if t2[1] in ("0", "1") else t2[12])
            prev_y = float(t2[9] if t2[1] in ("0", "1") else t2[13])
            rel = (float(t2[3]) - prev_x, float(t2[5]) - prev_y)
            if t2[1] in ("0", "1"):
                _sim_line(st, st[3] + rel[0], st[4] + rel[1], float(t2[7]), dt_s, js, a, b, eps, run, no_track, False)
            else:
                _sim_arc(st, st[3] + rel[0], st[4] + rel[1], float(t2[7]), float(t2[9]), t2[1] == "3", float(t2[11]),
                         dt_s, js, a, b, eps, run, no_track, False)
            if next_command_is_g1:
                lag_length = _r(st[2], 4)
        elif next_command_is_g1:
            lag_length = st[2]
        line_number += 1

    for seconds in [s for rest in dwell_at.values() for s in rest]:
        code.append(f"G4 P{seconds * 1000:.0f}")
    code.append(prog.text(n_cmd - 1).split(";")[0].strip())
    # Delete consecutive duplicate lines (vector_angle dropped the very last line here too,
    # having just appended it; kept here so the path ends where the file does)
    out = [code[0]]
    for line in code[1:]:
        if line != out[-1]:
            out.append(line)
    return out, corners, arc_joins


_WORD = re.compile(r"([A-Z])\s*([-+]?(?:\d+\.?\d*|\.\d+))")


def _feed_line(st, line, run, dt_s, js, a, b, eps, trk):
    # A written G0 / G1 / G2 / G3 into the jet model, as line_reader read it (modal feed,
    # absolute ends, arcs with I / J from the current position)
    words = dict(_WORD.findall(line.upper()))
    g = words.get("G", "1")
    feed = float(words.get("F", 0.0))
    x, y = float(words.get("X", st[3])), float(words.get("Y", st[4]))
    if g in ("0", "1", "00", "01"):
        _sim_line(st, x, y, feed, dt_s, js, a, b, eps, run, trk, False)
    else:
        _sim_arc(st, x, y, float(words.get("I", 0.0)), float(words.get("J", 0.0)), g in ("3", "03"), feed,
                 dt_s, js, a, b, eps, run, trk, False)
