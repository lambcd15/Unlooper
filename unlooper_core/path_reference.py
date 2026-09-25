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


def reference_polyline(segments):
    # Rows as params["Preview_segments"] (µm, y flipped) -> (x, y, cumulative length) of the
    # path in order, arcs split into chords. A jump between segments is kept as a line.
    xs, ys = [], []
    for kind, x1, y1, x2, y2, cx, cy, sweep, *_ in segments:
        if not xs or xs[-1] != x1 or ys[-1] != y1:
            xs.append(x1)
            ys.append(y1)
        if kind == 1:
            xs.append(x2)
            ys.append(y2)
            continue
        r = math.hypot(x1 - cx, y1 - cy)
        step = 2.0 * math.acos(max(-1.0, min(1.0, 1.0 - ARC_CHORD_ERROR_UM / r))) if r > ARC_CHORD_ERROR_UM else math.pi / 2
        n = max(1, int(math.ceil(math.radians(abs(sweep)) / max(step, 1e-6))))
        a1 = math.atan2(cy - y1, x1 - cx)
        for k in range(1, n + 1):
            a = a1 + math.radians(sweep) * k / n
            xs.append(cx + r * math.cos(a))
            ys.append(cy - r * math.sin(a))
    x = np.asarray(xs, dtype=np.float64)
    y = np.asarray(ys, dtype=np.float64)
    cum = np.concatenate(([0.0], np.cumsum(np.hypot(np.diff(x), np.diff(y)))))
    return x, y, cum


def _project(px, py, rx, ry, cum, k, window, out_d, out_qx, out_qy):
    # Nearest point on the reference polyline for each point, searching from two segments
    # back to `window` µm ahead of the previous answer along the path. Keeps to the pass the
    # points are following even where the path crosses or retraces itself; where passes
    # overlap (within TIE_UM - a retrace), the later one wins so the search moves on with
    # the path rather than sticking to the earlier pass.
    nseg = rx.shape[0] - 1
    s = cum[k]
    for i in range(px.shape[0]):
        best = 1e300
        best_k = k
        best_s = s
        bqx = rx[k]
        bqy = ry[k]
        j = k - 2 if k >= 2 else 0
        while j < nseg:
            if j > k + 1 and cum[j] - s > window:
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
            if d <= best + TIE_UM:
                if d < best:
                    best = d
                best_k = j
                best_s = cum[j] + t * (cum[j + 1] - cum[j])
                bqx = qx
                bqy = qy
            j += 1
        k = best_k
        s = best_s
        out_d[i] = math.hypot(px[i] - bqx, py[i] - bqy)
        out_qx[i] = bqx
        out_qy[i] = bqy
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
        d, qx, qy = np.empty(n), np.empty(n), np.empty(n)
        if len(self.rx) < 2:
            return d, qx, qy
        self.k = project(np.ascontiguousarray(x, dtype=np.float64), np.ascontiguousarray(y, dtype=np.float64),
                         self.rx, self.ry, self.cum, self.k, self.window, d, qx, qy)
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
