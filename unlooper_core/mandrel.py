"""Tubular (mandrel) printing: G-code whose second axis is the mandrel's rotation A (degrees)
instead of Y.

The jet lands on the mandrel's surface, so everything is worked out on the surface unrolled
flat: X along the tube, the surface distance round it as Y,

    Y (mm) = A (deg) x pi x D / 360        (D = mandrel diameter)

With a mandrel diameter set, reading a file turns every A into that Y (toolpath.py), so the
planner, the pixel coords, the lag model and the compensation all run on the unrolled
surface, and F is taken as the surface speed. wrap_lines() goes the other way - flat X / Y
G-code onto the tube, as X / A - which is how the compensated file is written for a mandrel
(arcs become G1 chords, as a controller can't arc between a linear and a rotary axis), and
can wrap any flat pattern onto a tube:

    python -m unlooper_core.mandrel wrap <flat file> <out file> <mandrel diameter mm>

render_cylinder() draws paths on the tube in 3D (front of the tube dark, back pale), for
the programmed path and the jet before / after compensation.

Shaped mandrels (4-axis files such as Ellipse_mouse.txt): the nozzle also moves in Z to keep
its distance to a surface whose radius changes along the tube (an ellipse end, a dome), often
with arcs in the X-Z plane (G18). trace_4axis() follows X, Z and A through the unlooped
commands - G90 / G91, G17 / G18 / G19 arcs by R or I / J / K - and puts each point at radius
R0 + Z and angle A, R0 being the mandrel's radius where the program starts (Z = 0).
render_4axis() draws that in 3D, with the mandrel body shaded from the path's own profile.
A move of A alone takes F in degrees / min (Mach3); F3944 deg/min on an 8.72 mm mandrel is
the same 300 mm/min surface speed as the X moves' F300.
"""
import math
import re
import sys

import cv2
import numpy as np

_WORD = re.compile(r"([A-Z])\s*([-+]?(?:\d+\.?\d*|\.\d+))")
CHORD_UM = 20.0  # arcs wrapped onto a mandrel become G1 chords this long at most


def degrees_to_mm(a_deg, diameter_mm):
    return a_deg * math.pi * diameter_mm / 360.0


def mm_to_degrees(y_mm, diameter_mm):
    return y_mm * 360.0 / (math.pi * diameter_mm)


def wrap_lines(lines, diameter_mm):
    # Flat absolute / relative G-code (X / Y in mm) onto a mandrel: Y becomes A in degrees.
    # G2 / G3 are split into G1 chords (followed in the plane, then wrapped). Other lines
    # are kept as they are.
    out = []
    absolute = True
    x = y = 0.0
    for line in lines:
        code = line.split(";")[0].strip().upper()
        words = _WORD.findall(code)
        gs = [int(float(v)) for k, v in words if k == "G"]
        if 90 in gs:
            absolute = True
        if 91 in gs:
            absolute = False
        motion = next((g for g in gs if g in (0, 1, 2, 3)), None)
        if motion is None or not any(k in ("X", "Y") for k, _ in words):
            out.append(line)
            continue
        vals = dict(words)
        nx = float(vals.get("X", x if absolute else 0.0))
        ny = float(vals.get("Y", y if absolute else 0.0))
        if not absolute:
            nx, ny = x + nx, y + ny
        feed = f" F{vals['F']}" if "F" in vals else ""
        g0 = "G0" if motion == 0 else "G1"
        if motion in (2, 3):
            cx, cy = x + float(vals.get("I", 0.0)), y + float(vals.get("J", 0.0))
            r = math.hypot(x - cx, y - cy)
            a1, a2 = math.atan2(y - cy, x - cx), math.atan2(ny - cy, nx - cx)
            sweep = (a2 - a1) % (2 * math.pi) if motion == 3 else -((a1 - a2) % (2 * math.pi))
            if sweep == 0:
                sweep = 2 * math.pi if motion == 3 else -2 * math.pi
            n = max(1, int(math.ceil(abs(sweep) * r * 1000.0 / CHORD_UM)))
            pts = [(cx + r * math.cos(a1 + sweep * k / n), cy + r * math.sin(a1 + sweep * k / n)) for k in range(1, n + 1)]
            pts[-1] = (nx, ny)
        else:
            pts = [(nx, ny)]
        px, py = x, y
        for k, (qx, qy) in enumerate(pts):
            if absolute:
                out.append(f"{g0} X{qx:.5f} A{mm_to_degrees(qy, diameter_mm):.5f}" + (feed if k == 0 else ""))
            else:
                out.append(f"{g0} X{qx - px:.5f} A{mm_to_degrees(qy - py, diameter_mm):.5f}" + (feed if k == 0 else ""))
            px, py = qx, qy
        x, y = nx, ny
    return out


def render_cylinder(paths, diameter_mm, out_png, x_range_mm, width_px=2400, tilt_deg=20.0, line_px=1):
    # Paths drawn on the tube in 3D. paths: [(x_um, y_um, rgb)] in the toolpath's frame
    # (y down; the surface distance round the tube is -y). The tube is seen from the side,
    # tilted towards the viewer; the far side is drawn pale behind the near side.
    r = diameter_mm / 2.0
    x0, x1 = x_range_mm
    margin = 0.06 * max(x1 - x0, 1e-6)
    ppm = width_px / (x1 - x0 + 2 * margin)
    tilt = math.radians(tilt_deg)
    height_px = int(2 * r * ppm * 1.25 + 2 * margin * ppm)
    image = np.full((height_px, width_px, 3), 255, dtype=np.uint8)
    cy = height_px / 2.0

    def project(x_um, y_um):
        theta = (-y_um / 1000.0) / r
        yy, zz = r * np.cos(theta), r * np.sin(theta)
        # tilt about the tube axis' perpendicular: the top of the tube leans towards the viewer
        v = yy * math.cos(tilt) - zz * math.sin(tilt)
        depth = yy * math.sin(tilt) + zz * math.cos(tilt)
        u = (x_um / 1000.0 - x0 + margin) * ppm
        return u, cy - v * ppm, depth

    # the tube's outline
    u0, u1 = margin * ppm, (x1 - x0 + margin) * ppm
    for sign in (-1, 1):
        cv2.line(image, (int(u0), int(cy + sign * r * ppm)), (int(u1), int(cy + sign * r * ppm)), (200, 200, 200), 1, cv2.LINE_AA)
    def dense(x_um, y_um):
        # Points close enough together (5 degrees of the tube at most) that a straight piece
        # of the unrolled path - a helix on the tube - is drawn round the tube, not across it
        x_um, y_um = np.asarray(x_um, dtype=np.float64), np.asarray(y_um, dtype=np.float64)
        s = np.concatenate(([0.0], np.cumsum(np.hypot(np.diff(x_um), np.diff(y_um)))))
        step = r * 1000.0 * math.radians(5.0)
        if s[-1] <= 0:
            return x_um, y_um
        t = np.union1d(s, np.arange(0.0, s[-1], step))
        return np.interp(t, s, x_um), np.interp(t, s, y_um)

    for back in (True, False):
        for x_um, y_um, rgb in paths:
            if len(x_um) < 2:
                continue
            u, v, depth = project(*dense(x_um, y_um))
            side = depth < 0 if back else depth >= 0
            colour = tuple(int(255 - (255 - c) * 0.25) for c in rgb) if back else tuple(rgb)
            # runs of points on this side
            edges = np.flatnonzero(np.diff(np.concatenate(([0], side.astype(np.int8), [0]))))
            for s0, s1 in zip(edges[::2], edges[1::2]):
                if s1 - s0 < 2:
                    continue
                pts = np.column_stack((u[s0:s1], v[s0:s1])) * 16
                cv2.polylines(image, [pts.astype(np.int32)], False, (colour[2], colour[1], colour[0]), line_px, cv2.LINE_AA, 4)
    cv2.imwrite(out_png, image)
    return out_png


def _arc_points(p0, p1, plane, clockwise, r=None, ijk=None, chord_mm=0.01):
    # Points along a G2 / G3 arc from p0 to p1 (dicts of X, Y, Z), in the plane's two axes
    a, b = {17: ("X", "Y"), 18: ("Z", "X"), 19: ("Y", "Z")}[plane]
    x0, y0, x1, y1 = p0[a], p0[b], p1[a], p1[b]
    if ijk is not None:
        off = {"X": ijk.get("I", 0.0), "Y": ijk.get("J", 0.0), "Z": ijk.get("K", 0.0)}
        cx, cy = x0 + off[a], y0 + off[b]
    else:
        # centre from the radius: R > 0 the short way round, R < 0 the long way
        dx, dy = x1 - x0, y1 - y0
        d = math.hypot(dx, dy)
        if d == 0 or r is None:
            return [p1]
        rr = abs(r)
        h = math.sqrt(max(rr * rr - d * d / 4.0, 0.0))
        mx, my = (x0 + x1) / 2.0, (y0 + y1) / 2.0
        # clockwise + short way: centre to the right of the chord
        side = 1.0 if (clockwise == (r > 0)) else -1.0
        cx, cy = mx + side * h * dy / d, my - side * h * dx / d
    rad = math.hypot(x0 - cx, y0 - cy)
    a0, a1 = math.atan2(y0 - cy, x0 - cx), math.atan2(y1 - cy, x1 - cx)
    sweep = a1 - a0
    if clockwise and sweep > 0:
        sweep -= 2 * math.pi
    if not clockwise and sweep < 0:
        sweep += 2 * math.pi
    n = max(2, int(math.ceil(abs(sweep) * max(rad, 1e-9) / chord_mm)))
    pts = []
    for k in range(1, n + 1):
        t = a0 + sweep * k / n
        q = dict(p0)
        q[a], q[b] = cx + rad * math.cos(t), cy + rad * math.sin(t)
        for ax in ("X", "Y", "Z", "A"):
            if ax not in (a, b):
                q[ax] = p0[ax] + (p1[ax] - p0[ax]) * k / n
        pts.append(q)
    return pts


def trace_4axis(lines, step_deg=3.0):
    # The unlooped program as points (X, Y, Z, A) with the move each came from; long
    # rotations are split every step_deg so a hoop is drawn round the mandrel
    pos = {"X": 0.0, "Y": 0.0, "Z": 0.0, "A": 0.0}
    absolute, plane, motion = False, 17, 1
    out = [dict(pos)]
    for line in lines:
        code = line.split(";")[0].split("%")[0].strip().upper()
        words = _WORD.findall(code)
        if not words:
            continue
        vals = {}
        for k, v in words:
            if k == "G":
                g = int(float(v))
                if g in (0, 1, 2, 3):
                    motion = g
                elif g == 90:
                    absolute = True
                elif g == 91:
                    absolute = False
                elif g in (17, 18, 19):
                    plane = g
            else:
                vals[k] = float(v)
        if not any(k in vals for k in ("X", "Y", "Z", "A")):
            continue
        new = dict(pos)
        for ax in ("X", "Y", "Z", "A"):
            if ax in vals:
                new[ax] = vals[ax] if absolute else pos[ax] + vals[ax]
        if motion in (2, 3):
            ijk = {k: vals[k] for k in ("I", "J", "K") if k in vals} or None
            pts = _arc_points(pos, new, plane, motion == 2, r=vals.get("R"), ijk=ijk)
        else:
            n = max(1, int(math.ceil(abs(new["A"] - pos["A"]) / step_deg)))
            pts = [{ax: pos[ax] + (new[ax] - pos[ax]) * k / n for ax in pos} for k in range(1, n + 1)]
        out.extend(pts)
        pos = new
    return out


def render_4axis(lines, diameter_mm, out_png, width_px=2400, tilt_deg=25.0, yaw_deg=35.0):
    # A shaped-mandrel program in 3D: each point at radius R0 + Z, angle A, along X, seen by
    # a camera tilted over the tube (tilt) and turned round towards its far end (yaw). The
    # mandrel body is the path's own profile (the largest radius at each X) swept round and
    # lit; the path is dark on the near side, pale where it runs behind the mandrel.
    pts = trace_4axis(lines)
    X = np.array([p["X"] for p in pts]); Z = np.array([p["Z"] for p in pts]); A = np.radians([p["A"] for p in pts])
    r0 = diameter_mm / 2.0
    R = np.maximum(r0 + Z, 0.0)
    tilt, yaw = math.radians(tilt_deg), math.radians(yaw_deg)

    def camera(x, r, a):
        # tube along x, the angle a measured from straight up; returns screen u, v and
        # whether the surface there faces the camera
        y, z = r * np.cos(a), r * np.sin(a)
        y1, z1 = y * math.cos(tilt) - z * math.sin(tilt), y * math.sin(tilt) + z * math.cos(tilt)
        u = x * math.cos(yaw) + z1 * math.sin(yaw)
        facing = (np.cos(a) * math.sin(tilt) + np.sin(a) * math.cos(tilt)) * math.cos(yaw) + 0.0
        return u, y1, facing

    x0, x1 = float(X.min()), float(X.max())
    rmax = float(R.max()) if len(R) else r0
    # the mandrel profile
    bins = np.linspace(x0, x1, 300)
    idx = np.clip(np.searchsorted(bins, X), 0, len(bins) - 1)
    prof = np.zeros(len(bins))
    np.maximum.at(prof, idx, R)
    filled = prof > 0
    prof = np.interp(np.arange(len(bins)), np.flatnonzero(filled), prof[filled]) if filled.any() else np.full(len(bins), r0)
    slope = np.gradient(prof, bins) if len(bins) > 1 else np.zeros(len(bins))  # dr / dx
    ring = np.linspace(-math.pi, math.pi, 73)

    def faces(x_slope, a):
        # outward normal of the surface of revolution (-r', cos a, sin a), towards the camera?
        nz1 = np.cos(a) * math.sin(tilt) + np.sin(a) * math.cos(tilt)
        return -x_slope * -math.sin(yaw) + nz1 * math.cos(yaw)
    gu, gv, _f = camera(bins[:, None], prof[:, None], ring[None, :])
    pu, pv, _f = camera(X, R, A)
    umin, umax = min(gu.min(), pu.min()), max(gu.max(), pu.max())
    vmin, vmax = min(gv.min(), pv.min()), max(gv.max(), pv.max())
    margin = 0.06 * (umax - umin)
    ppm = width_px / (umax - umin + 2 * margin)
    height_px = int((vmax - vmin + 2 * margin) * ppm)
    image = np.full((height_px, width_px, 3), 255, dtype=np.uint8)
    to_px = lambda u, v: np.column_stack(((u - umin + margin) * ppm, (vmax + margin - v) * ppm))
    # lit surface patches, back to front (painter's order by depth)
    light = np.array([0.35, 0.75, 0.55]); light /= np.linalg.norm(light)
    quads = []
    for i in range(len(bins) - 1):
        for j in range(len(ring) - 1):
            am = 0.5 * (ring[j] + ring[j + 1])
            ny, nz = math.cos(am), math.sin(am)
            ny1, nz1 = ny * math.cos(tilt) - nz * math.sin(tilt), ny * math.sin(tilt) + nz * math.cos(tilt)
            # distance towards the camera of the patch's centre
            depth = -0.5 * (bins[i] + bins[i + 1]) * math.sin(yaw) + prof[i] * nz1 * math.cos(yaw)
            if faces(0.5 * (slope[i] + slope[i + 1]), am) < -0.02:
                continue
            shade = 205 + 45 * max(0.0, ny1 * light[1] + nz1 * light[2])
            quad = to_px(np.array([gu[i, j], gu[i + 1, j], gu[i + 1, j + 1], gu[i, j + 1]]),
                         np.array([gv[i, j], gv[i + 1, j], gv[i + 1, j + 1], gv[i, j + 1]]))
            quads.append((depth, quad, int(min(shade, 250))))
    # the ends of the mandrel as discs: the far one behind the body, the near one in front
    near, far = (0, len(bins) - 1) if math.sin(yaw) >= 0 else (len(bins) - 1, 0)
    disc = lambda i: [(to_px(gu[i, :-1], gv[i, :-1]) * 16).astype(np.int32)]
    cv2.fillPoly(image, disc(far), (226, 226, 226), cv2.LINE_AA, 4)
    for _d, quad, shade in sorted(quads, key=lambda q: q[0]):
        cv2.fillConvexPoly(image, (quad * 16).astype(np.int32), (shade, shade, shade), cv2.LINE_AA, 4)
    cv2.fillPoly(image, disc(near), (232, 232, 232), cv2.LINE_AA, 4)
    # the path: behind the mandrel pale, in front dark
    facing = faces(np.interp(X, bins, slope), A)
    xy = to_px(pu, pv)
    for back, colour in ((True, (205, 190, 175)), (False, (120, 45, 20))):
        side = facing < 0 if back else facing >= 0
        edges = np.flatnonzero(np.diff(np.concatenate(([0], side.astype(np.int8), [0]))))
        for s0, s1 in zip(edges[::2], edges[1::2]):
            if s1 - s0 < 2:
                continue
            cv2.polylines(image, [(xy[s0:s1] * 16).astype(np.int32)], False, colour, 1, cv2.LINE_AA, 4)
    cv2.imwrite(out_png, image)
    return out_png


def _x_range(xs):
    lo = min(float(np.min(x)) for x in xs if len(x)) / 1000.0
    hi = max(float(np.max(x)) for x in xs if len(x)) / 1000.0
    return lo, max(hi, lo + 1e-3)


def render_program(params, variables):
    # <name>_mandrel.png: the programmed path on the tube, with the jet (lag prediction) if run.
    # A program that moves Z (a shaped mandrel) is drawn in full 4-axis 3D instead.
    from .path_reference import reference_polyline
    from .pixel_coords import output_base
    lines = params.get("Unlooped_contents") or []
    if any(re.search(r"\bZ\s*[-+]?[\d.]", ln.upper()) for ln in lines[:20000]):
        out = output_base(params) + "_mandrel.png"
        render_4axis(lines, variables["Mandrel_diameter_mm"], out)
        print("Mandrel image saved:", out, "(4-axis:", variables["Mandrel_diameter_mm"], "mm mandrel at Z = 0, radius R0 + Z)")
        return
    rx, ry, _c = reference_polyline(params["Preview_segments"])
    paths = [(rx, ry, (0, 0, 0))]
    jet = params.get("Lag_samples")
    if jet is not None and len(jet):
        paths.append((jet[:, 0].astype(np.float64), jet[:, 1].astype(np.float64), (150, 150, 150)))
    out = output_base(params) + "_mandrel.png"
    render_cylinder(paths, variables["Mandrel_diameter_mm"], out, _x_range([rx]))
    print("Mandrel image saved:", out, "(" + str(variables["Mandrel_diameter_mm"]), "mm mandrel; black = programmed path, grey = jet)")


def render_comparison(params, variables, before, after, out_png):
    # The before / after colours of the flat comparison image, on the tube
    from .path_reference import reference_polyline
    rx, ry, _c = reference_polyline(params["Preview_segments"])
    paths = [(rx, ry, (0, 0, 0)), (before[:, 0].astype(np.float64), before[:, 1].astype(np.float64), (150, 150, 150)),
             (after[:, 0].astype(np.float64), after[:, 1].astype(np.float64), (25, 130, 25))]
    render_cylinder(paths, variables["Mandrel_diameter_mm"], out_png, _x_range([rx]))


if __name__ == "__main__":
    if len(sys.argv) < 5 or sys.argv[1] != "wrap":
        sys.exit("python -m unlooper_core.mandrel wrap <flat file> <out file> <mandrel diameter mm>")
    with open(sys.argv[2]) as f:
        src = f.read().splitlines()
    wrapped = wrap_lines(src, float(sys.argv[4]))
    with open(sys.argv[3], "w") as f:
        f.write("\n".join(wrapped) + "\n")
    print("Wrapped", len(src), "lines onto a", sys.argv[4], "mm mandrel:", sys.argv[3])
