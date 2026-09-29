"""The programmed path as a polyline, and how far a point sequence (the jet) strays from it.

Used to score the jet path (how far the fibre lands from where the G-code put it) and by
the iterative lag compensation to find where each jet point should have landed.
"""
import math

import numpy as np

try:
    from numba import njit
except ImportError:  # pragma: no cover
    njit = None

ARC_CHORD_ERROR_UM = 0.5  # arcs are split finely enough that the chords stay this close
TIE_UM = 1.0  # passes this close count as the same distance (the later one is followed)
LOST_UM = 300.0  # an answer further off than this, that has stopped moving along the path ...
STUCK_UM = 1000.0  # ... while the points travel this far, means the search has lost the path
RESCAN = 10.0  # then look this many windows ahead for it
ADVANCE = 4.0  # a tie with a later pass is only taken this many times as far along the path as the point moved ...
ADVANCE_SLACK_UM = 100.0  # ... plus this
BACK_UM = 3000.0  # how far back along the path a clearly closer answer is still taken


def reference_polyline(segments, with_ends=False):
    # Rows as params["Preview_segments"] (µm, y flipped) -> (x, y, cumulative length) of the
    # path in order, arcs split into chords. A jump between segments is kept as a line.
    # with_ends also returns how far along the path (µm) each row ends.
    xs, ys = [], []
    end_point = []
    for kind, x1, y1, x2, y2, cx, cy, sweep, *_ in segments:
        if not xs or xs[-1] != x1 or ys[-1] != y1:
            xs.append(x1)
            ys.append(y1)
        if kind == 1:
            xs.append(x2)
            ys.append(y2)
            end_point.append(len(xs) - 1)
            continue
        r = math.hypot(x1 - cx, y1 - cy)
        step = 2.0 * math.acos(max(-1.0, min(1.0, 1.0 - ARC_CHORD_ERROR_UM / r))) if r > ARC_CHORD_ERROR_UM else math.pi / 2
        n = max(1, int(math.ceil(math.radians(abs(sweep)) / max(step, 1e-6))))
        a1 = math.atan2(cy - y1, x1 - cx)
        for k in range(1, n + 1):
            a = a1 + math.radians(sweep) * k / n
            xs.append(cx + r * math.cos(a))
            ys.append(cy - r * math.sin(a))
        end_point.append(len(xs) - 1)
    x = np.asarray(xs, dtype=np.float64)
    y = np.asarray(ys, dtype=np.float64)
    cum = np.concatenate(([0.0], np.cumsum(np.hypot(np.diff(x), np.diff(y)))))
    if with_ends:
        return x, y, cum, cum[np.asarray(end_point, dtype=np.int64)] if end_point else np.zeros(0)
    return x, y, cum


def _project(px, py, rx, ry, cum, k, window, out_d, out_qx, out_qy, out_s):
    # Nearest point on the reference polyline for each point (out_s = how far along the
    # path it is, in µm), searching from two segments
    # back to `window` µm ahead of the previous answer along the path. Keeps to the pass the
    # points are following even where the path crosses or retraces itself; where passes
    # overlap (within TIE_UM - a retrace), the later one wins so the search moves on with
    # the path rather than sticking to the earlier pass.
    # A jet that cuts across a stretch of path longer than the window (a small loop or
    # circle at a large lag) would leave the search stuck behind it, measuring every later
    # point against the wrong place. So when the answer is far off (LOST_UM) and has not
    # got any further along the path while the points travelled STUCK_UM, the search looks
    # RESCAN x further ahead and jumps there if it finds the path at least twice as close.
    # A tie only goes to a later pass that is at most ADVANCE x as far along the path as the
    # point moved (+ ADVANCE_SLACK_UM): where passes overlap, a point exactly on the path is
    # as close to a much later pass as to its own, and would otherwise jump ahead to it.
    # And where the path loops back close to itself, a point a little off the path can be
    # nearer a piece further on; the search then looks up to BACK_UM back along the path and
    # returns there if it finds the path there at least twice as close.
    nseg = rx.shape[0] - 1
    s = cum[k]
    furthest = s
    still = 0.0
    for i in range(px.shape[0]):
        reach = window
        tie_reach = window
        if i > 0:
            step = math.hypot(px[i] - px[i - 1], py[i] - py[i - 1])
            still += step
            tie_reach = ADVANCE * step + ADVANCE_SLACK_UM
        rescan = False
        best = 1e300
        best_k = k
        best_s = s
        bqx = rx[k]
        bqy = ry[k]
        j = k - 2 if k >= 2 else 0
        while j < nseg:
            if j > k + 1 and cum[j] - s > reach:
                if rescan or best <= LOST_UM or still < STUCK_UM:
                    break
                # Stuck: keep going further ahead, but only take a much closer answer
                rescan = True
                reach = window * RESCAN
                best = 0.5 * best - TIE_UM
            if j > k + 1 and cum[j] - s > reach:
                break
            ax = rx[j]
            ay = ry[j]
            dx = rx[j + 1] - ax
            dy = ry[j + 1] - ay
            ll = dx * dx + dy * dy
            t = 0.0
            if ll > 0:
                t = ((px[i] - ax) * dx + (py[i] - ay) * dy) / ll
                if t < 0:
                    t = 0.0
                elif t > 1:
                    t = 1.0
            qx = ax + t * dx
            qy = ay + t * dy
            d = math.sqrt((px[i] - qx) ** 2 + (py[i] - qy) ** 2)
            cand_s = cum[j] + t * (cum[j + 1] - cum[j])
            if d < best - TIE_UM or (d <= best + TIE_UM and cand_s - s <= tie_reach):
                if d < best:
                    best = d
                best_k = j
                best_s = cand_s
                bqx = qx
                bqy = qy
            j += 1
        # Ran ahead? A clearly closer answer up to BACK_UM back along the path
        j = k - 3
        back_best = 0.5 * best - TIE_UM
        while j >= 0 and cum[j + 1] > s - BACK_UM:
            ax = rx[j]
            ay = ry[j]
            dx = rx[j + 1] - ax
            dy = ry[j + 1] - ay
            ll = dx * dx + dy * dy
            t = 0.0
            if ll > 0:
                t = ((px[i] - ax) * dx + (py[i] - ay) * dy) / ll
                if t < 0:
                    t = 0.0
                elif t > 1:
                    t = 1.0
            qx = ax + t * dx
            qy = ay + t * dy
            d = math.sqrt((px[i] - qx) ** 2 + (py[i] - qy) ** 2)
            if d < back_best:
                back_best = d
                best = d
                best_k = j
                best_s = cum[j] + t * (cum[j + 1] - cum[j])
                bqx = qx
                bqy = qy
            j -= 1
        if best_s > furthest + 1.0:
            furthest = best_s
            still = 0.0
        k = best_k
        s = best_s
        out_d[i] = math.hypot(px[i] - bqx, py[i] - bqy)
        out_qx[i] = bqx
        out_qy[i] = bqy
        out_s[i] = best_s
    return k


project = njit(cache=True, nogil=True)(_project) if njit is not None else _project


class DeviationTracker:
    # Streams points (µm) against a reference polyline and keeps the mean, 95th percentile
    # (1 µm histogram) and maximum distance.

    def __init__(self, segments, window_um=3000.0):
        self.rx, self.ry, self.cum = reference_polyline(segments)
        self.k = 0
        self.window = window_um
        self.hist = np.zeros(20001, dtype=np.int64)
        self.total, self.count, self.max = 0.0, 0, 0.0

    def add(self, x, y):
        n = len(x)
        d, qx, qy, s = np.empty(n), np.empty(n), np.empty(n), np.empty(n)
        if len(self.rx) < 2:
            return d, qx, qy
        self.k = project(np.ascontiguousarray(x, dtype=np.float64), np.ascontiguousarray(y, dtype=np.float64),
                         self.rx, self.ry, self.cum, self.k, self.window, d, qx, qy, s)
        self.total += float(d.sum())
        self.count += n
        self.max = max(self.max, float(d.max()))
        self.hist += np.bincount(np.minimum(d.astype(np.int64), len(self.hist) - 1), minlength=len(self.hist))
        return d, qx, qy

    def summary(self):
        if self.count == 0:
            return None
        p95 = int(np.searchsorted(np.cumsum(self.hist), 0.95 * self.count))
        return self.total / self.count, float(p95), self.max
