"""The hybrid lag compensation: the nozzle leads the jet along the path by exactly the lag,
with the lag shed before sharp corners, and the machine's planner and the lag model in the
loop. Written as G1 moves only, so it suits controllers that take no G2 / G3.

Why it works. In the lag model (lag_model.py) the jet always moves straight towards the
nozzle, at the speed whose stationary lag is its distance behind it (the fall speed): jet
speed and lag are tied, Lag = a * (v / CTS)^b - a. So for a jet to follow the path at speed v
the nozzle has to be that lag ahead of it along the path's direction:

    nozzle = jet target + Lag(v) x tangent of the path there

and the jet then moves exactly along the path. This is the model run backwards, and it is
what every part below builds on:

  - straight lines and waves: the nozzle leads by the lag along the tangent, smoothed over
    about half a program piece so the program's small chords (a sine written as 0.2 mm lines, an arc as
    chords) read as the curve they draw. The jet runs at the programmed feed. (What the
    point-by-point method did one point at a time, without its swings.)
  - sharp corners (turns over SHARP_DEG): the tangent flips, so the nozzle has to jump from
    one side of the corner to the other. The jump is short only if the lag is small there,
    so the lag is shed before the corner to what makes the jump the corner tolerance (from
    the slow-down method) and built up again after it (from the overshoot method's rapid
    swing, as a straight rapid move). Shedding: the nozzle slows to SHED of the jet's speed
    so the jet catches up; building: it speeds up to (1 + GROW) x.
  - tight curves: leading by the lag round a curve of radius R swings the nozzle round a
    circle of radius sqrt(R^2 + Lag^2), faster than the jet. The lag is capped so that stays
    within ACCEL_FRACTION of the acceleration and RAPID_FRACTION of the rapid speed.
  - the planner: the nozzle path as written is run through the machine's own planner
    (motion_planner.plan_moves); where it can't keep the requested feed over a stretch of
    path, the jet is planned slower there to match (PLANNER_FITS rounds).
  - correction passes (from the passes of the other methods, with the whole simulation -
    planner, corner rounding, 1 ms samples, lag model - in the loop): where the path bends
    within one lag ahead of the jet, timing matters, and wherever the simulated jet ran with
    less lag there than planned, the plan takes the lag it actually ran with. The file with
    the jet closest to the path is kept.

The cost is time at sharp corners: the jet has to slow nearly to the CTS to turn one
exactly, so a pattern made of short lines and sharp corners prints slower (lines and waves
barely do). Slowing the jet also changes the fibre diameter there.

Constant jet speed (compensate_hybrid_constant): the jet kept at the programmed speed, so
the lag and the fibre diameter stay the same everywhere, as in ISBF. Then the jet can only
turn as tightly as the nozzle can swing round it at the lag distance - radius r where a
nozzle circling at sqrt(r^2 + Lag^2) at the jet's angular rate v / r stays within the
acceleration and speed limits (about 0.8 mm at speed ratio 3, 1000 mm/s^2, 3000 mm/min).
So the path is smoothed over TURN_SMOOTHING x that radius - only where it bends tighter, or
everywhere, whichever lands the jet closer - and the nozzle leads by the lag along it.
Features smaller than that radius are rounded off: that is the price of a constant diameter.
"""
import math

import numpy as np

try:
    from numba import njit
except ImportError:  # pragma: no cover - numba is optional, just slower without it
    njit = None

SAMPLE_UM = 5.0  # the path is planned on points this far apart ...
MAX_SAMPLES = 3_000_000  # ... or further apart on a path too long for this many
SHARP_DEG = 30.0  # a turn this sharp is a corner (the lag is shed before it); gentler ones are curve
SMOOTH_FRACTION = 0.5  # the lead direction is the path direction averaged over this x the stretch's typical piece length ...
SMOOTH_MIN_UM = 10.0  # ... at least this ...
SMOOTH_MAX_UM = 150.0  # ... and at most this
SHED = 0.5  # while shedding lag the nozzle runs at least (1 - SHED) x the jet's speed
GROW = 3.0  # while building lag it runs at most (1 + GROW) x the jet's speed
ACCEL_FRACTION = 0.5  # round curves the nozzle's sideways acceleration stays within this x the limit
RAPID_FRACTION = 0.9  # and its speed within this x the rapid speed
PLANNER_FITS = 3  # rounds of matching the plan to the machine's planner
PLANNER_WINDOW_MM = 0.5  # the planner's time is compared over this much path either side ...
PLANNER_SHORTFALL = 0.9  # ... and taken as a limit where it is slower than this x asked
CONSTANT_ACCEL_FRACTION = 0.5  # constant jet speed: the nozzle's swing round the jet stays within this x the acceleration ...
CONSTANT_RAPID_FRACTION = 0.9  # ... and this x the rapid speed
TURN_SMOOTHING = 0.65  # constant jet speed: the path is smoothed over this x the jet's tightest turn radius
ARC_STEP_UM = 20.0  # the corner arcs are written as G1 chords this long at most
PASS_GAIN = 0.98  # the passes stop at the first one that isn't at least 2% closer than the best so far
SIM_MARGIN = 0.9  # a pass lowers the lag where the simulated jet ran with less than this x the plan
POSITION_TOL_UM = 1.0  # the nozzle path is written as G1 moves this close to it ...
SPEED_TOL = 0.05  # ... each within this fraction of one feed
MAX_CHORD = 20000  # samples in one G1 move at most


def _program_polyline(segments, variables):
    # The programmed path as a polyline (µm, y flipped, arcs split into chords within 0.5 µm),
    # the feed (mm/min) of each piece, and the vertex each row ends at
    scale = variables["scale"]
    override = variables["Feedrate_override_mm_min"]
    xs, ys, feeds, row_end = [], [], [], []
    for kind, x1, y1, x2, y2, cx, cy, sweep, _line, feed, *_ in segments:
        f = override if override > 0 else feed * 60.0
        if not xs:
            xs.append(x1)
            ys.append(y1)
        elif abs(xs[-1] - x1) > 0.01 or abs(ys[-1] - y1) > 0.01:
            xs.append(x1)  # a jump between rows is drawn as a line
            ys.append(y1)
            feeds.append(f)
        if kind == 1:
            xs.append(x2)
            ys.append(y2)
            feeds.append(f)
        else:
            r = math.hypot(x1 - cx, y1 - cy)
            step = 2.0 * math.acos(max(-1.0, min(1.0, 1.0 - 0.5 / r))) if r > 0.5 else math.pi / 2
            n = max(1, int(math.ceil(math.radians(abs(sweep)) / max(step, 1e-6))))
            a1 = math.atan2(cy - y1, x1 - cx)
            for k in range(1, n + 1):
                a = a1 + math.radians(sweep) * k / n
                xs.append(cx + r * math.cos(a))
                ys.append(cy - r * math.sin(a))
                feeds.append(f)
        row_end.append(len(xs) - 1)
    x, y, f = np.asarray(xs, dtype=np.float64), np.asarray(ys, dtype=np.float64), np.asarray(feeds, dtype=np.float64)
    # Zero-length pieces dropped
    keep = np.concatenate(([True], np.hypot(np.diff(x), np.diff(y)) > 1e-6))
    new_index = np.cumsum(keep) - 1
    return x[keep], y[keep], f[keep[1:]], new_index[np.asarray(row_end, dtype=np.int64)] if row_end else np.zeros(0, np.int64)


def _resample(x, y, f, ds, sharp_deg, out_vertex):
    # Points at most ds apart along each piece (the vertices kept). Per point: position,
    # direction of its piece, feed, which stretch it is on (stretches end at sharp corners)
    # and, at a sharp corner, the turn (degrees). out_vertex[i] = the point at vertex i.
    nseg = x.shape[0] - 1
    total = 0
    for i in range(nseg):
        total += max(1, int(math.ceil(math.hypot(x[i + 1] - x[i], y[i + 1] - y[i]) / ds)))
    n = total + 1
    px, py, ux, uy, fe = np.empty(n), np.empty(n), np.empty(n), np.empty(n), np.empty(n)
    stretch = np.empty(n, dtype=np.int64)
    turn = np.zeros(n)
    cos_sharp = math.cos(math.radians(sharp_deg))
    k = 0
    sid = 0
    px[0] = x[0]
    py[0] = y[0]
    out_vertex[0] = 0
    for i in range(nseg):
        dx = x[i + 1] - x[i]
        dy = y[i + 1] - y[i]
        length = math.hypot(dx, dy)
        m = max(1, int(math.ceil(length / ds)))
        vx = dx / length
        vy = dy / length
        if i == 0:
            ux[0] = vx
            uy[0] = vy
            fe[0] = f[0]
            stretch[0] = 0
        else:
            pdx = x[i] - x[i - 1]
            pdy = y[i] - y[i - 1]
            c = min(1.0, max(-1.0, (pdx * vx + pdy * vy) / math.hypot(pdx, pdy)))
            if c < cos_sharp:
                turn[k] = math.degrees(math.acos(c))
                sid += 1
        for j in range(1, m + 1):
            k += 1
            px[k] = x[i] + dx * j / m
            py[k] = y[i] + dy * j / m
            ux[k] = vx
            uy[k] = vy
            fe[k] = f[i]
            stretch[k] = sid
        out_vertex[i + 1] = k
    return px, py, ux, uy, fe, stretch, turn


def _lag_profile(lag, cap, seg_mm, shed, grow):
    # The lag along the path: at most the programmed feed's stationary lag and each point's
    # cap, rising at most `grow` and falling at most `shed` per mm of path
    n = lag.shape[0]
    for i in range(n):
        lag[i] = min(lag[i], cap[i])
    for i in range(1, n):
        lag[i] = min(lag[i], lag[i - 1] + seg_mm[i - 1] * grow)
    for i in range(n - 2, -1, -1):
        lag[i] = min(lag[i], lag[i + 1] + seg_mm[i] * shed)
    return lag


def _curvature_cap(kappa, grid_lag, grid_v, accel_limit, speed_limit):
    # Largest lag (on the grid) for which a nozzle leading by it round a curve of curvature
    # kappa (1/mm) keeps within the sideways acceleration and speed limits
    n = kappa.shape[0]
    out = np.empty(n)
    m = grid_lag.shape[0]
    for i in range(n):
        k = kappa[i]
        best = grid_lag[m - 1]
        if k > 1e-6:
            best = 0.0
            for j in range(m):
                lag = grid_lag[j]
                v = grid_v[j]
                f = math.sqrt(1.0 + (lag * k) ** 2)
                if v * v * k * f > accel_limit or v * f > speed_limit:
                    break
                best = lag
        out[i] = best
    return out


def _chord_ok(nx, ny, tt, i0, j, tol, speed_tol):
    # Points i0..j within tol of the chord between them, and moving along it within
    # speed_tol of its average speed
    ax, ay = nx[i0], ny[i0]
    dx, dy = nx[j] - ax, ny[j] - ay
    ll = dx * dx + dy * dy
    dur = tt[j] - tt[i0]
    vbar = math.sqrt(ll) / dur if dur > 0 else 0.0
    for q in range(i0 + 1, j):
        t = 0.0
        if ll > 0:
            t = min(1.0, max(0.0, ((nx[q] - ax) * dx + (ny[q] - ay) * dy) / ll))
        if math.hypot(nx[q] - ax - t * dx, ny[q] - ay - t * dy) > tol:
            return False
        dq = tt[q] - tt[q - 1]
        if dq > 0 and vbar > 0:
            v = math.hypot(nx[q] - nx[q - 1], ny[q] - ny[q - 1]) / dq
            if abs(v - vbar) > speed_tol * vbar:
                return False
    return True


def _decimate(nx, ny, tt, next_must, tol, speed_tol, max_chord):
    # The points kept as G1 ends: from each, the furthest point the chord reaches (galloping
    # then bisecting), never past a point that has to be kept (next_must)
    n = nx.shape[0]
    keep = np.empty(n, dtype=np.int64)
    m = 0
    keep[m] = 0
    m += 1
    i0 = 0
    while i0 < n - 1:
        limit = min(n - 1, i0 + max_chord, next_must[i0])
        good = i0 + 1
        bad = -1
        while good < limit:
            cand = min(limit, i0 + 2 * (good - i0))
            if _chord_ok(nx, ny, tt, i0, cand, tol, speed_tol):
                good = cand
            else:
                bad = cand
                break
        if bad > 0:
            while bad - good > 1:
                mid = (good + bad) // 2
                if _chord_ok(nx, ny, tt, i0, mid, tol, speed_tol):
                    good = mid
                else:
                    bad = mid
        keep[m] = good
        m += 1
        i0 = good
    return keep[:m]


if njit is not None:
    _resample = njit(cache=True)(_resample)
    _lag_profile = njit(cache=True)(_lag_profile)
    _curvature_cap = njit(cache=True)(_curvature_cap)
    _chord_ok = njit(cache=True)(_chord_ok)
    _decimate = njit(cache=True)(_decimate)


def _tightest_turn(v, lag, accel_limit, speed_limit):
    # Smallest radius (mm) a jet moving at v (mm/s) can turn on while the nozzle leads it by
    # `lag` (mm): the nozzle circles at radius sqrt(r^2 + lag^2), at the jet's angular rate
    # v / r, within the acceleration and speed limits
    if lag <= 0 or v <= 0:
        return 0.0
    lo, hi = 1e-4, 100.0
    for _ in range(60):
        r = math.sqrt(lo * hi)
        w = v / r
        rn = math.hypot(r, lag)
        if w * w * rn > accel_limit or w * rn > speed_limit:
            lo = r
        else:
            hi = r
    return hi


def stationary_lag(ratio, a, b):
    # Lag (mm) at speed ratio(s) `ratio` (0 at or below the CTS)
    ratio = np.asarray(ratio, dtype=np.float64)
    return np.where(ratio > 1.0, a * np.power(np.maximum(ratio, 1.0), b) - a, 0.0)


def speed_ratio(lag, a, b):
    # The speed ratio whose stationary lag is `lag` (the jet's fall speed / CTS)
    return np.power(np.maximum((np.asarray(lag, dtype=np.float64) + a) / a, 1e-12), 1.0 / b)


def _smooth_directions(ux, uy, stretch, piece_um, ds):
    # Directions averaged (Gaussian) within each stretch, never across a sharp corner, over
    # half the stretch's typical piece length: enough to read a sine written as 0.2 mm lines
    # as the curve, without flattening a curve written finely
    out_x, out_y = ux.copy(), uy.copy()
    bounds = np.flatnonzero(np.diff(stretch)) + 1
    for s0, s1 in zip(np.concatenate(([0], bounds)), np.concatenate((bounds, [len(ux)]))):
        count = s1 - s0
        if count < 3:
            continue
        sigma_um = min(max(SMOOTH_FRACTION * float(np.median(piece_um[s0:s1])), SMOOTH_MIN_UM), SMOOTH_MAX_UM)
        sigma_samples = sigma_um / ds
        half = int(3 * sigma_samples) + 1
        kernel = np.exp(-0.5 * (np.arange(-half, half + 1) / sigma_samples) ** 2)
        conv = lambda v: np.convolve(v, kernel)[half:half + count]
        weight = conv(np.ones(count))
        sx, sy = conv(ux[s0:s1]) / weight, conv(uy[s0:s1]) / weight
        norm = np.hypot(sx, sy)
        norm[norm == 0] = 1.0
        out_x[s0:s1], out_y[s0:s1] = sx / norm, sy / norm
    return out_x, out_y


class LeadPlan:
    # The programmed path sampled finely, the lag planned along it, and the nozzle path that
    # lag gives. build() writes it as G-code; planner_fit() and sim_fit() lower the lag where
    # the machine or the simulated jet showed the plan can't be kept.

    def __init__(self, params, variables, constant=False, smooth_all=False):
        self.params, self.variables = params, variables
        self.constant = constant
        js, _dt, a, b, _eps = params["Lag_model"]
        self.js, self.a, self.b = js, a, b
        self.scale = scale = variables["scale"]
        self.rapid = float(variables["Lag_comp_rapid_mm_min"])
        corner_mm = float(variables.get("Lag_comp_corner_um", 20.0)) / 1000.0
        # How much the jet may slow (%), which sets how much the fibre diameter may grow
        # (d / d0 = sqrt(v0 / v)): the lag is never planned below the floor speed's
        default = 0.0 if constant else 100.0
        self.speed_change = min(max(float(variables.get("Lag_comp_speed_change_pct", default)), 0.0), 100.0)
        x, y, f, row_end = _program_polyline(params["Preview_segments"], variables)
        total_um = float(np.hypot(np.diff(x), np.diff(y)).sum())
        ds = self.ds = max(SAMPLE_UM, total_um / MAX_SAMPLES)
        at_vertex = np.zeros(len(x), dtype=np.int64)
        px, py, ux, uy, fe, stretch, turn = _resample(x, y, f, ds, 181.0 if constant else SHARP_DEG, at_vertex)
        self.turn_radius = 0.0
        if constant:
            # The jet at the programmed speed everywhere, so the lag stays the same. Then it can
            # only turn as tightly as the nozzle can swing round it at the lag distance, so it
            # is steered along the path smoothed to that radius (no corners left to shed lag at)
            from scipy.ndimage import gaussian_filter1d
            turn[:] = 0.0  # no corners to jump at, reversals included: everything is turned
            # With a speed change allowed, the jet may slow to (1 - change) x its speed at
            # the tight bends only, where the smaller lag lets it turn tighter
            floor = max(1.0 - self.speed_change / 100.0, js / max(float(f.max()), 1e-9))
            feed = floor * float(f.max()) / 60.0
            self.floor_lag = float(stationary_lag(floor * f.max() / js, a, b))
            self.turn_radius = _tightest_turn(feed, self.floor_lag, CONSTANT_ACCEL_FRACTION * float(variables["Acceleration_mm_s2"]),
                                              CONSTANT_RAPID_FRACTION * self.rapid / 60.0)
            self.slow_at = None
            sigma = TURN_SMOOTHING * self.turn_radius * scale / ds
            if sigma > 0.5:
                # Only where the path bends tighter than that (corners, small arcs), its
                # curvature measured over a quarter of the radius, so gentler curves are kept;
                # or everywhere (smooth_all), which suits a path made of little but corners
                n = len(px)
                w = max(int(0.125 * self.turn_radius * scale / ds), 1)
                heading = np.unwrap(np.arctan2(uy, ux))
                bend = np.abs(heading[np.minimum(np.arange(n) + w, n - 1)] - heading[np.maximum(np.arange(n) - w, 0)])
                tight = bend / (2 * w * ds / scale) > 1.0 / self.turn_radius
                near = np.minimum(1.0, 6.0 * gaussian_filter1d(tight.astype(np.float64), sigma, mode="nearest"))
                self.slow_at = near > 0.05
                weight = 1.0 if smooth_all else near
                px = px + weight * (gaussian_filter1d(px, sigma, mode="nearest") - px)
                py = py + weight * (gaussian_filter1d(py, sigma, mode="nearest") - py)
                dx, dy = np.diff(px, append=px[-1]), np.diff(py, append=py[-1])
                dx[-1], dy[-1] = dx[-2], dy[-2]
                norm = np.maximum(np.hypot(dx, dy), 1e-12)
                ux, uy = dx / norm, dy / norm
        # A sharp corner is sampled twice: arriving (direction in, end of its stretch) and
        # leaving (direction out); the nozzle jumps between the two
        n = len(px)
        ins = np.flatnonzero(turn > 0)
        nxt = np.minimum(ins + 1, n - 1)
        px, py = np.insert(px, ins + 1, px[ins]), np.insert(py, ins + 1, py[ins])
        fe = np.insert(fe, ins + 1, fe[nxt])
        ux, uy = np.insert(ux, ins + 1, ux[nxt]), np.insert(uy, ins + 1, uy[nxt])
        stretch = np.insert(stretch, ins + 1, stretch[nxt])
        turn = np.insert(turn, ins + 1, 0.0)
        at_vertex = at_vertex + np.searchsorted(ins, at_vertex, side="left")
        self.corner_in = ins + np.arange(len(ins))
        self.corner_out = self.corner_in + 1
        self.px, self.py, self.fe = px, py, fe
        self.n = n = len(px)
        # length of the program piece each point lies on
        piece = np.hypot(np.diff(x), np.diff(y))
        owner = np.searchsorted(at_vertex, np.arange(n), side="left") - 1
        piece_um = piece[np.clip(owner, 0, len(piece) - 1)]
        self.tx, self.ty = _smooth_directions(ux, uy, stretch, piece_um, ds)
        self.lag_nominal = stationary_lag(fe / js, a, b)
        self.seg_mm = np.hypot(np.diff(px), np.diff(py)) / scale
        self.s_um = np.concatenate(([0.0], np.cumsum(self.seg_mm))) * scale
        # Caps: at sharp corners, a lag that makes the nozzle's jump the corner tolerance
        cap = np.full(n, np.inf)
        th = turn[self.corner_in]
        corner_cap = corner_mm / np.maximum(2.0 * np.sin(np.radians(th) / 2.0), 1e-6)
        cap[self.corner_in] = corner_cap
        cap[self.corner_out] = corner_cap
        # ... round tight curves, what the nozzle can follow
        angle = np.arctan2(self.ty, self.tx)
        turning = np.abs(np.angle(np.exp(1j * np.diff(angle))))
        k_raw = np.concatenate((turning / np.maximum(self.seg_mm, 1e-9), [0.0]))
        k_raw[self.corner_in] = 0.0
        w = max(int(SMOOTH_MAX_UM / ds), 1)
        kappa = np.convolve(k_raw, np.ones(2 * w + 1) / (2 * w + 1))[w:w + n]
        grid_lag = np.linspace(0.0, max(float(self.lag_nominal.max()), 0.01), 400)
        grid_v = js * speed_ratio(grid_lag, a, b) / 60.0
        accel_limit = ACCEL_FRACTION * float(variables["Acceleration_mm_s2"])
        cap = np.minimum(cap, _curvature_cap(kappa, grid_lag, grid_v, accel_limit, RAPID_FRACTION * self.rapid / 60.0))
        if constant:
            # Constant jet speed: the lag is never lowered (only ramped up at the start and
            # down at the end and at dwells, where the nozzle stops) - or, with a speed change
            # allowed, lowered to the floor speed's lag at the tight bends only
            cap = np.full(n, np.inf)
            if self.speed_change > 0 and self.slow_at is not None and len(self.slow_at) == n:
                cap[self.slow_at] = self.floor_lag
        # ... and none at the start, the end and every dwell, where the nozzle is at rest
        must = np.zeros(n, dtype=np.bool_)
        must[self.corner_in] = True
        must[self.corner_out] = True
        self.lag_floor = np.zeros(n)
        if not constant and self.speed_change < 100.0:
            self.lag_floor = stationary_lag((1.0 - self.speed_change / 100.0) * fe / js, a, b)
            cap = np.maximum(cap, self.lag_floor)
        cap[0] = cap[-1] = 0.0
        self.dwell_at = {}
        for d in params.get("Dwells", []):
            row = int(d[0])
            q = int(at_vertex[row_end[row - 1]]) if 0 < row <= len(row_end) else (0 if row <= 0 else n - 1)
            q = min(max(q, 0), n - 1)
            cap[q] = 0.0
            must[q] = True
            self.dwell_at.setdefault(q, []).append(float(d[1]))
        self.cap = cap
        idx = np.where(must, np.arange(n), n - 1)
        self.next_must = np.minimum.accumulate(idx[::-1])[::-1]
        # next_must[i] = first must point after i
        self.next_must = np.concatenate((self.next_must[1:], [n - 1]))

    def build(self):
        # The nozzle path for the current caps, as G-code lines. Also keeps the G1 moves as
        # rows (for the planner) and the samples each covers.
        from .lag_compensation import GcodeWriter
        scale, js, rapid = self.scale, self.js, self.rapid
        lag = _lag_profile(self.lag_nominal.copy(), self.cap, self.seg_mm, SHED, GROW)
        v = np.minimum(js * speed_ratio(lag, self.a, self.b), np.maximum(self.fe, js))  # jet speed, mm/min
        nx = self.px + lag * scale * self.tx
        ny = self.py + lag * scale * self.ty
        dt = np.zeros(self.n)
        dt[1:] = self.seg_mm / np.maximum(0.5 * (v[1:] + v[:-1]), 1e-9)  # minutes
        ci, co = self.corner_in, self.corner_out
        self.lag, self.v = lag, v
        dwell_at, next_must = self.dwell_at, self.next_must
        origin = np.arange(self.n)
        if len(ci):
            # Round each sharp corner as the ISBF paper does (Lamb et al. 2026, Fig. 2): the
            # nozzle, a lag past the vertex along the incoming line, swings round the vertex
            # on an arc of radius = the lag at the cornering speed onto the outgoing line.
            # With the lag shed to the corner tolerance this is a short hop; with it held
            # (a small speed change allowed) it is the paper's full-lag swing.
            radius = lag[ci] * scale
            a0 = np.arctan2(self.ty[ci], self.tx[ci])
            sweep = np.angle(np.exp(1j * (np.arctan2(self.ty[co], self.tx[co]) - a0)))
            pieces = np.maximum(np.ceil(np.abs(sweep) * radius / ARC_STEP_UM).astype(np.int64), 1)
            k = np.repeat(np.arange(len(ci)), pieces)
            first = np.cumsum(pieces) - pieces
            frac = (np.arange(len(k)) - first[k] + 1) / pieces[k]
            ang = a0[k] + sweep[k] * frac
            ax = self.px[ci][k] + radius[k] * np.cos(ang)
            ay = self.py[ci][k] + radius[k] * np.sin(ang)
            adt = (np.abs(sweep) * radius / scale / pieces / rapid)[k]
            at = np.repeat(co, pieces)
            dt[co] = 0.0  # the arc ends on the outgoing lead point
            nx, ny = np.insert(nx, at, ax), np.insert(ny, at, ay)
            dt = np.insert(dt, at, adt)
            origin = np.insert(origin, at, np.repeat(ci, pieces))
            shift = lambda q: q + np.searchsorted(at, q, side="right")
            dwell_at = {int(shift(q)): secs for q, secs in dwell_at.items()}
            must = np.zeros(len(nx), dtype=np.bool_)
            must[shift(ci)] = True
            must[shift(co)] = True
            for q in dwell_at:
                must[q] = True
            idx = np.where(must, np.arange(len(nx)), len(nx) - 1)
            next_must = np.minimum.accumulate(idx[::-1])[::-1]
            next_must = np.concatenate((next_must[1:], [len(nx) - 1]))
        tt = np.cumsum(dt)
        keep = _decimate(nx, ny, tt, next_must, POSITION_TOL_UM, SPEED_TOL, MAX_CHORD)
        writer = GcodeWriter(scale, (0.0, 0.0))
        writer.g1(nx[0], ny[0], v[0])
        self.last_xy = (nx, ny)
        rows, ranges = [], []
        for i0, i1 in zip(keep[:-1].tolist(), keep[1:].tolist()):
            for seconds in dwell_at.get(i0, ()):
                writer.dwell(seconds)
            dist = math.hypot(nx[i1] - nx[i0], ny[i1] - ny[i0]) / scale
            dur = tt[i1] - tt[i0]
            feed = min(max(dist / dur if dur > 0 else rapid, 1.0), rapid)
            writer.g1(nx[i1], ny[i1], feed)
            if dist > 1e-7:
                rows.append((1, nx[i0], ny[i0], nx[i1], ny[i1], 0.0, 0.0, 0.0, len(rows), feed / 60.0, len(rows)))
                ranges.append((origin[i0], origin[i1]))
        for seconds in dwell_at.get(len(nx) - 1, ()):
            writer.dwell(seconds)
        self.rows, self.ranges = rows, np.asarray(ranges, dtype=np.int64).reshape(-1, 2)
        self.plan_time = float(tt[-1] * 60.0)
        return writer.lines

    def _lower(self, where, lag):
        where = where.copy()
        where[self.corner_out] = False
        if not where.any():
            return 0
        self.cap = np.where(where, np.minimum(self.cap, np.maximum(lag, self.lag_floor)), self.cap)
        return int(where.sum())

    def planner_fit(self):
        # Where the machine's planner takes longer than asked over a stretch of path (so
        # ordinary acceleration between neighbouring feeds averages out), plan the jet
        # slower there by the same ratio
        from .motion_planner import plan_moves
        accel = float(self.variables["Acceleration_mm_s2"])
        planned, _t, _b = plan_moves(self.rows, [], self.variables)
        m = len(self.rows)
        if m == 0:
            return 0
        want, took, length = np.zeros(m), np.zeros(m), np.zeros(m)
        for index, ln, v0, peak, v1, d_acc, d_dec in planned:
            took[index] = (peak - v0) / accel + (peak - v1) / accel + max(ln - d_acc - d_dec, 0.0) / max(peak, 1e-9)
            want[index] = ln / max(self.rows[index][9], 1e-9)
            length[index] = ln
        cl, cw, ct = (np.concatenate(([0.0], np.cumsum(v))) for v in (length, want, took))
        mid = 0.5 * (cl[:-1] + cl[1:])
        k = np.arange(m)
        lo = np.minimum(np.searchsorted(cl, mid - PLANNER_WINDOW_MM), k)
        hi = np.maximum(np.minimum(np.searchsorted(cl, mid + PLANNER_WINDOW_MM), m), k + 1)
        ratio_row = (cw[hi] - cw[lo]) / np.maximum(ct[hi] - ct[lo], 1e-12)
        ratio = np.ones(self.n)
        for r in np.flatnonzero(ratio_row < PLANNER_SHORTFALL):
            i0, i1 = self.ranges[r]
            ratio[i0:i1 + 1] = np.minimum(ratio[i0:i1 + 1], max(ratio_row[r], 0.3))
        return self._lower(ratio < PLANNER_SHORTFALL, stationary_lag(ratio * self.v / self.js, self.a, self.b))

    def _timing_matters(self):
        # Points where the path turns within one lag ahead: there the jet's direction depends
        # on where the nozzle is along the path, so its timing matters (on a straight it
        # doesn't - a nozzle anywhere ahead on the line pulls the jet along the line)
        ahead = np.minimum(np.searchsorted(self.s_um, self.s_um + np.maximum(self.lag, 0.02) * self.scale), self.n - 1)
        dot = self.tx * self.tx[ahead] + self.ty * self.ty[ahead]
        corners = np.zeros(self.n + 1)
        corners[self.corner_in + 1] = 1
        corners = np.cumsum(corners)
        return (dot < math.cos(math.radians(2.0))) | (corners[ahead + 1] > corners[np.arange(self.n) + 1])

    def sim_fit(self, res, start_cut):
        # Where timing matters and the simulated jet ran with less lag than planned, plan the
        # lag it actually ran with
        s, lag = res.s[start_cut:], np.nan_to_num(res.lag[start_cut:])
        order = np.argsort(s, kind="stable")
        ran = np.interp(self.s_um, s[order], lag[order])
        return self._lower(self._timing_matters() & (ran < SIM_MARGIN * self.lag), ran)


def program_lines(params, variables):
    # The program itself as absolute G-code lines (lines and arcs as they are, with its dwells)
    from .lag_compensation import GcodeWriter
    scale = variables["scale"]
    override = variables["Feedrate_override_mm_min"]
    segments = params["Preview_segments"]
    dwells = {}
    for d in params.get("Dwells", []):
        dwells.setdefault(int(d[0]), []).append(float(d[1]))
    writer = GcodeWriter(scale, (0.0, 0.0))
    for i, (kind, x1, y1, x2, y2, cx, cy, sweep, _line, feed, *_) in enumerate(segments):
        for seconds in dwells.get(i, ()):
            writer.dwell(seconds)
        f = override if override > 0 else feed * 60.0
        if abs(writer.x - x1) > 0.01 or abs(writer.y - y1) > 0.01:
            writer.g1(x1, y1, f)
        if kind == 1:
            writer.g1(x2, y2, f)
        else:
            writer.arc(x2, y2, cx, cy, sweep > 0, f)
    for seconds in dwells.get(len(segments), ()):
        writer.dwell(seconds)
    return writer.lines


def compensate_hybrid_constant(params, variables, score, baseline):
    # The hybrid with the jet never slowing (0% speed change): constant fibre diameter
    return compensate_hybrid(params, dict(variables, Lag_comp_speed_change_pct=0.0), score, baseline)


def _hybrid_rounded(params, variables, score, baseline):
    # The hybrid with the jet at the programmed speed throughout, so the lag - and the fibre
    # diameter that goes with the jet speed - stay the same, as in ISBF. The nozzle leads by
    # the lag along the path smoothed to the tightest turn the jet can make at that speed.
    # Two ways to smooth (only the tight bends, or the whole path); the closer one is kept.
    from .lag_compensation import rows_from_gcode
    best = None
    for smooth_all in (False, True):
        plan = LeadPlan(params, variables, constant=True, smooth_all=smooth_all)
        lines = plan.build()
        res = score(*rows_from_gcode(lines, variables["scale"]))
        how = "the whole path" if smooth_all else "the tight bends"
        print("Lag compensation (hybrid, constant jet speed): lag", round(float(plan.lag_nominal.max()), 3),
              "mm throughout; tightest jet turn at full speed", round(plan.turn_radius, 3), "mm radius;", how,
              "smoothed over", round(TURN_SMOOTHING * plan.turn_radius, 3), f"mm: jet off the path by mean {res.mean:.2f} um, "
              f"print {res.duration:.1f} s")
        if best is None or res.mean < best[0]:
            best = (res.mean, lines)
    print(f"Lag compensation (hybrid, constant jet speed): uncompensated jet off the path by mean {baseline.mean:.2f} um; "
          f"using the closer, mean {best[0]:.2f} um")
    return best[1]


def compensate_hybrid(params, variables, score, baseline):
    # With the full speed change allowed (100%) the jet slows as far as each sharp corner
    # needs. With less, two ways of turning within that diameter limit are tried - the
    # paper's arc round each corner at the floor speed's lag, and rounding the path to the
    # jet's tightest turn at that speed - and the closer is kept.
    from .lag_compensation import rows_from_gcode
    change = float(variables.get("Lag_comp_speed_change_pct", 100.0))
    if change < 100.0:
        # Any of these keeps within the limit (the constant-speed rounding more than keeps it)
        scale = variables["scale"]
        candidates = [("corner arcs", _hybrid_shed(params, variables, score, baseline)),
                      ("rounded path", _hybrid_rounded(params, variables, score, baseline))]
        if change > 0:
            candidates.append(("rounded path at constant speed",
                               _hybrid_rounded(params, dict(variables, Lag_comp_speed_change_pct=0.0), score, baseline)))
        # ... and the program as it is: at a low speed ratio the lag is small, and holding
        # the diameter tight can cost more than the lag does
        candidates.append(("programmed path unchanged", program_lines(params, variables)))
        # Judge each on the simulated jet: the fibre diameter it really lays down (the machine
        # may slow the nozzle more than planned) and how far it lands from the path. The most
        # accurate one whose thickest fibre (95th percentile) is within the limit is kept; if
        # none is, the one that comes closest to the limit.
        limit = float(variables.get("Lag_comp_diameter_limit_pct", 0.0)) / 100.0
        if limit <= 0:
            limit = 1.0 / math.sqrt(max(1.0 - change / 100.0, 1e-6)) - 1.0
        feed = float(np.median([row[9] for row in params["Preview_segments"]])) if params["Preview_segments"] else 1.0
        results = []
        for name, lines in candidates:
            res = score(*rows_from_gcode(lines, scale))
            # fibre diameter from the nozzle's speed over the collector (as lag_model reports it)
            v = np.hypot(np.diff(res.x), np.diff(res.y)) / (score.dt * score.stride) / 1000.0
            thick = float(np.percentile(np.sqrt(feed / np.maximum(v[score.start_cut:], 1e-6)), 95)) if len(v) > score.start_cut else 1.0
            results.append((name, lines, res.mean, thick))
        ok = [r for r in results if r[3] <= 1.0 + limit + 0.005]
        best = min(ok, key=lambda r: r[2]) if ok else min(results, key=lambda r: (r[3], r[2]))
        print(f"Lag compensation (hybrid, fibre diameter within +{limit * 100:.1f}%): " +
              ", ".join(f"{name} {m:.2f} um (fibre up to x{t:.2f})" for name, _l, m, t in results) +
              f"; using the {best[0]}" + ("" if ok else " (none keeps within the limit on this machine; closest)"))
        return best[1]
    return _hybrid_shed(params, variables, score, baseline)


def _hybrid_shed(params, variables, score, baseline):
    from .lag_compensation import rows_from_gcode
    scale = variables["scale"]
    iterations = int(variables["Lag_comp_iterations"])
    plan = LeadPlan(params, variables)
    lines = plan.build()
    print("Lag compensation (hybrid):", plan.n, "path points", round(plan.ds, 1), "um apart,", len(plan.corner_in),
          "sharp corners (tolerance", variables.get("Lag_comp_corner_um", 20.0), "um),", len(plan.dwell_at), "dwells; jet may slow",
          f"{plan.speed_change:g}% (fibre up to x{1 / math.sqrt(max(1 - plan.speed_change / 100, 1 / max(float(plan.fe.max()) / plan.js, 1))):.2f})")
    for k in range(PLANNER_FITS):
        changed = plan.planner_fit()
        if changed == 0:
            break
        lines = plan.build()
    res = score(*rows_from_gcode(lines, scale))
    print(f"Lag compensation (hybrid): uncompensated jet off the path by mean {baseline.mean:.2f} um; compensated "
          f"{res.mean:.2f} um, rms {res.rms:.2f} um, print {baseline.duration:.1f} -> {res.duration:.1f} s")
    best_mean, best_lines = res.mean, lines
    for k in range(iterations):
        changed = plan.sim_fit(res, score.start_cut)
        print("Lag compensation progress:", str(int(100 * (k + 1) / iterations)) + "%")
        if changed == 0:
            print("Lag compensation (hybrid): pass", k + 1, "- nothing left to fit")
            break
        lines = plan.build()
        res = score(*rows_from_gcode(lines, scale))
        gained = res.mean < PASS_GAIN * best_mean
        if res.mean < best_mean:
            best_mean, best_lines = res.mean, lines
        print(f"Lag compensation (hybrid): pass {k + 1} fitted {changed} points; jet off the path by mean "
              f"{res.mean:.2f} um, rms {res.rms:.2f} um, print {res.duration:.1f} s")
        if not gained:
            # Each pass simulates the whole print again; stop once they stop paying
            print("Lag compensation (hybrid): no longer improving, stopping the passes")
            break
    print(f"Lag compensation (hybrid): using the best, mean {best_mean:.2f} um")
    return best_lines
