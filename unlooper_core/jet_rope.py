"""Stage 5 (alternative) - rope model of the jet: the jet as a chain of charged beads joined
by viscoelastic links, extruded from the nozzle and laid down on the moving collector.

Based on the bead model of Reneker, Yarin, Fong & Koombhongse, "Bending instability of
electrically charged liquid jets of polymer solutions in electrospinning", J. Appl. Phys. 87,
4531 (2000), Sec. IV F, adapted to melt electrowriting (MEW), where the jet is stable and
short and the collector moves under it:
  - beads of mass m_i (half the volume of each link either side, x density) carry charge
    e_i = q m_i and feel the field V0 / h pulling them to the collector (their Eq. 15) and
    gravity; q is set so the steady jet lands at the critical translation speed (below)
  - each link is a Maxwell element (their Eq. 12): d sigma/dt = G (dl/dt)/l - sigma/theta,
    G = mu / theta, pulling with pi a^2 sigma (Eq. 16), a^2 l fixed per link (Eq. 17), plus
    the capillary tension pi a alpha of the thread (surface tension)
  - Coulomb repulsion between every pair of airborne beads (Eq. 13), the force behind the
    bending instability of Sec. IV E
  - bending resistance at every joint, the same Maxwell law for the moment
    (dM/dt = G I dkappa/dt - M/theta, I = pi a^4 / 4), which gives the rope its coiling
    ("fluid-mechanical sewing machine") below the CTS. Reneker et al. used a surface tension
    term for this (Eq. 18); for a viscous thread the bending stiffness is what sets coiling
  - inertia: m_i d2r_i/dt2 = sum of the above (Eq. 20)
Extrusion: the link at the nozzle gains Q dt of volume every step (Q = flow rate), so new jet
is "paid out" like a rope; links longer than the local maximum are split in two and very
short ones merged. A bead that reaches the collector (z = 0) sticks there and moves with it:
the laid fibre. The jet's contact point is where it meets the collector.

For MEW the forces are viscous-dominated (jet Reynolds numbers ~1e-6), so the equations are
stiff; each step solves the links and joints implicitly for the bead velocities (one banded
symmetric system), with inertia, field, gravity and Coulomb forces in it, so the step can be
the pixel coords' 1 ms.

The steady vertical jet fixes the one unknown: the field's pull per unit mass. It is solved
from the 1D steady jet (mass, momentum, viscous and capillary tension) so the jet leaves the
nozzle at Q / nozzle area and lands at the CTS with no pull from the collector - the
definition of the CTS. Above the CTS the moving collector pulls the jet back (the lag);
below it the jet arrives faster than the collector takes it away and buckles into coils.
"""
import json
import math
from pathlib import Path

import numpy as np

try:
    from numba import njit
except ImportError:  # pragma: no cover - numba is optional, just much slower without it
    njit = None

K_COULOMB = 8.9875517923e9  # 1 / (4 pi eps0), N m^2 / C^2
G_EARTH = 9.81
CAPACITY = 600  # most beads in the jet at once
HB = 8  # half bandwidth of the velocity system: joints couple beads two apart, 3 unknowns each
CALIBRATION_FILE = Path(__file__).resolve().parent.parent / "lag_data" / "rope_calibration.json"

DEFAULTS = {
    "Rope_collector_distance_mm": 3.0,
    "Rope_start_diameter_um": 60.0,  # the jet where it leaves the Taylor cone; sets its starting speed
    "Rope_viscosity_Pa_s": 3000.0,  # elongational viscosity of the melt (Trouton: 3 x shear)
    "Rope_relaxation_ms": 1.0,  # Maxwell relaxation time theta = mu / G, at the nozzle
    # Cooling: the melt cools in flight, so viscosity and relaxation time rise with the time
    # since it left the nozzle, x exp(age / Rope_cooling_ms), up to Rope_solid_ratio (solid)
    "Rope_cooling_ms": 300.0,
    "Rope_solid_ratio": 1000.0,
    "Rope_surface_tension_N_m": 0.03,
    "Rope_coulomb": True,
    "Rope_link_mm": 0.15,  # longest link near the nozzle; 1/6 of this at the collector
    "Rope_snapshot_ms": 20.0,  # jet shape kept every this often for the 3D render (0 = none)
    "Fibre_diameter_um": 20.0,  # at the CTS, when the file gives none
    "Density_kg_m3": 1145.0,  # PCL, when the file gives none
    "Voltage_kV": 5.0,  # when the file gives none (only sets the bead charge for Coulomb)
}


def settings(variables):
    # Model settings: the variables' Rope_* values over the calibration file over the defaults
    s = dict(DEFAULTS)
    if CALIBRATION_FILE.exists():
        try:
            s.update(json.loads(CALIBRATION_FILE.read_text()).get("settings", {}))
        except (OSError, ValueError):
            pass
    for key in DEFAULTS:
        if key in variables and variables[key] not in (None, ""):
            s[key] = variables[key]
    if float(variables.get("Fibre_Diameter", 0) or 0) > 0:
        s["Fibre_diameter_um"] = float(variables["Fibre_Diameter"])
    if float(variables.get("Material_Density", 0) or 0) > 0:
        rho = float(variables["Material_Density"])
        s["Density_kg_m3"] = rho * 1000.0 if rho < 20 else rho  # g/cm3 or kg/m3
    if float(variables.get("Applied_Voltage", 0) or 0) > 0:
        v = float(variables["Applied_Voltage"])
        s["Voltage_kV"] = v / 1000.0 if v > 200 else v  # V or kV
    return s


# --- steady vertical jet ------------------------------------------------------------------

def viscosity_factor(age, tau, cap):
    # How much more viscous the melt is after age seconds in flight
    if tau <= 0.0:
        return 1.0
    x = age / tau
    return cap if x >= math.log(cap) else math.exp(x)


def steady_jet(H, v0, vc, Q, rho, mu, alpha, tau=0.0, cap=1.0):
    """Steady vertical jet from the nozzle (s = 0) to the collector (s = H), s downward.
    w = ln v; viscous tension mu(s) Q w', total tension T = mu Q w' + pi a alpha with
    a = sqrt(Q / (pi v)); momentum dT/ds = rho Q v w' - rho Q g / v. Lands at vc with T = 0.
    mu(s) = mu x viscosity_factor(age(s)), age(s) = time in flight = int ds / v, found by
    iterating from the uniform-viscosity jet.
    Returns g (field + gravity pull per unit mass, m/s^2) and the profile s, v, T, age."""
    from scipy.integrate import solve_ivp
    from scipy.optimize import brentq

    floor = math.log(v0) - 3.0  # far enough below the nozzle speed to know the field was too strong
    s_grid = np.linspace(0.0, H, 400)
    factor = np.ones_like(s_grid)

    def rhs(s, y, g):
        w, T = y
        v = math.exp(max(w, floor - 1.0))
        cap_t = math.pi * alpha * math.sqrt(Q / (math.pi * v))
        dw = (T - cap_t) / (mu * np.interp(s, s_grid, factor) * Q)
        return [dw, rho * Q * v * dw - rho * Q * g / v]

    def too_slow(s, y, g):
        return y[0] - floor
    too_slow.terminal = True

    def shoot(lg, dense=False):
        # Integrate up from the collector; the miss in ln v at the nozzle (-3 if the jet
        # had already slowed far below the nozzle speed on the way up)
        g = math.exp(lg)
        try:
            sol = solve_ivp(rhs, (H, 0.0), [math.log(vc), 0.0], args=(g,), method="LSODA",
                            rtol=1e-8, atol=1e-12, dense_output=dense, events=too_slow)
        except (ZeroDivisionError, OverflowError, ValueError):
            return (np.nan, None) if dense else np.nan
        if sol.status == 1:
            return (-3.0, sol) if dense else -3.0
        if not sol.success:
            return (np.nan, None) if dense else np.nan
        miss = sol.y[0, -1] - math.log(v0)
        return (miss, sol) if dense else miss

    g_last = None
    for _iteration in range(30):
        grid = np.linspace(math.log(1e-2), math.log(1e10), 49)
        misses = np.array([shoot(lg) for lg in grid])
        ok = np.flatnonzero(np.isfinite(misses[:-1]) & np.isfinite(misses[1:]) & (np.sign(misses[:-1]) != np.sign(misses[1:])))
        if not len(ok):
            raise ValueError("Rope model: no field strength lands the steady jet at the CTS - check the settings")
        lg = brentq(shoot, grid[ok[0]], grid[ok[0] + 1], xtol=1e-12)
        _miss, sol = shoot(lg, dense=True)
        y = sol.sol(s_grid)
        v = np.exp(y[0])
        inv = 1.0 / v
        age = np.concatenate(([0.0], np.cumsum(0.5 * (inv[1:] + inv[:-1]) * np.diff(s_grid))))
        g = math.exp(lg)
        if tau <= 0.0 or (g_last is not None and abs(g - g_last) < 1e-4 * g):
            break
        g_last = g
        new = np.array([viscosity_factor(a, tau, cap) for a in age])
        factor = np.exp(0.5 * np.log(factor) + 0.5 * np.log(new))  # damped
    return g, s_grid, v, y[1], age


# --- one implicit step ---------------------------------------------------------------------

def _add(ab, rhs, i, j, c, gi, gj, n, vz_top):
    # Add c gi gj^T to block (i, j) of the free-bead system (beads 1 .. n-2). Coupling to an
    # anchor (bead 0 on the collector, still; bead n-1 at the nozzle, moving) goes to the rhs.
    if i < 1 or i > n - 2:
        return
    if j < 1 or j > n - 2:
        if j == n - 1:
            dot = gj[0] * vz_top[0] + gj[1] * vz_top[1] + gj[2] * vz_top[2]
            for r in range(3):
                rhs[3 * (i - 1) + r] -= c * gi[r] * dot
        return
    for r in range(3):
        row = 3 * (i - 1) + r
        for t in range(3):
            col = 3 * (j - 1) + t
            if row >= col:
                ab[row - col, col] += c * gi[r] * gj[t]


def _add_block(ab, rhs, i, j, blk, n, vz_top):
    # Add the 3x3 block blk to block (i, j), as _add
    if i < 1 or i > n - 2:
        return
    if j < 1 or j > n - 2:
        if j == n - 1:
            for r in range(3):
                rhs[3 * (i - 1) + r] -= blk[r, 0] * vz_top[0] + blk[r, 1] * vz_top[1] + blk[r, 2] * vz_top[2]
        return
    for r in range(3):
        row = 3 * (i - 1) + r
        for t in range(3):
            col = 3 * (j - 1) + t
            if row >= col:
                ab[row - col, col] += blk[r, t]


def _cholesky_solve(ab, b, m):
    # Banded Cholesky (lower, ab[d, j] = K[j + d, j]) and solve, in place
    for j in range(m):
        s = ab[0, j]
        for k in range(max(0, j - HB), j):
            s -= ab[j - k, k] * ab[j - k, k]
        if s <= 0.0:
            s = 1e-300
        d = math.sqrt(s)
        ab[0, j] = d
        for i in range(j + 1, min(m, j + HB + 1)):
            s = ab[i - j, j]
            for k in range(max(0, i - HB), j):
                s -= ab[i - k, k] * ab[j - k, k]
            ab[i - j, j] = s / d
    for i in range(m):
        s = b[i]
        for k in range(max(0, i - HB), i):
            s -= ab[i - k, k] * b[k]
        b[i] = s / ab[0, i]
    for i in range(m - 1, -1, -1):
        s = b[i]
        for k in range(i + 1, min(m, i + HB + 1)):
            s -= ab[k - i, i] * b[k]
        b[i] = s / ab[0, i]


def _step(pos, vel, vol, sig, mom, age, n, dt, top, vtop, rho, mu, theta, alpha, g, q, coulomb, tau, cap, ab, rhs, gj, gv, el):
    # One implicit step of the bead chain. Beads 0 (laid, on the collector) and n-1 (nozzle)
    # are anchors; beads 1 .. n-2 are solved for. el[k] = (ux, uy, uz, l) of link k.
    nf = n - 2
    m = 3 * nf
    ab[:, :m] = 0.0
    rhs[:m] = 0.0
    # per link (el[k, 4..5]) and joint (gv[i, 2..3]) k_t = theta / (theta + dt) and
    # k_v = mu dt / (theta + dt), with the cooled viscosity and relaxation time
    for k in range(n - 1):
        f = viscosity_factor(0.5 * (age[k] + age[k + 1]), tau, cap)
        el[k, 4] = theta * f / (theta * f + dt)
        el[k, 5] = mu * f * dt / (theta * f + dt)
    for i in range(1, n - 1):
        f = viscosity_factor(age[i], tau, cap)
        gv[i, 2] = theta * f / (theta * f + dt)
        gv[i, 3] = mu * f * dt / (theta * f + dt)
    # inertia and body forces
    for i in range(1, n - 1):
        mi = rho * 0.5 * (vol[i - 1] + vol[i])
        for r in range(3):
            ab[0, 3 * (i - 1) + r] += mi / dt
            rhs[3 * (i - 1) + r] += mi / dt * vel[i, r]
        rhs[3 * (i - 1) + 2] -= mi * g
    # Coulomb repulsion between airborne beads (Reneker et al. Eq. 13)
    if coulomb and q > 0.0:
        for i in range(1, n - 1):
            ei = q * rho * 0.5 * (vol[i - 1] + vol[i])
            for j in range(i + 1, n - 1):
                ej = q * rho * 0.5 * (vol[j - 1] + vol[j])
                dx = pos[i, 0] - pos[j, 0]
                dy = pos[i, 1] - pos[j, 1]
                dz = pos[i, 2] - pos[j, 2]
                r2 = dx * dx + dy * dy + dz * dz + 1e-24
                f = K_COULOMB * ei * ej / (r2 * math.sqrt(r2))
                rhs[3 * (i - 1)] += f * dx
                rhs[3 * (i - 1) + 1] += f * dy
                rhs[3 * (i - 1) + 2] += f * dz
                rhs[3 * (j - 1)] -= f * dx
                rhs[3 * (j - 1) + 1] -= f * dy
                rhs[3 * (j - 1) + 2] -= f * dz
    # links: Maxwell tension + capillary tension
    u = np.empty(3)
    geo = np.empty((3, 3))
    neg = np.empty((3, 3))
    for k in range(n - 1):
        dx = pos[k + 1, 0] - pos[k, 0]
        dy = pos[k + 1, 1] - pos[k, 1]
        dz = pos[k + 1, 2] - pos[k, 2]
        l = math.sqrt(dx * dx + dy * dy + dz * dz) + 1e-15
        u[0], u[1], u[2] = dx / l, dy / l, dz / l
        el[k, 0], el[k, 1], el[k, 2], el[k, 3] = u[0], u[1], u[2], l
        area = vol[k] / l
        k_t, k_v = el[k, 4], el[k, 5]
        t0 = area * sig[k] * k_t + math.pi * alpha * math.sqrt(area / math.pi)
        c = area * k_v / l
        if k == 0 and u[0] * vel[1, 0] + u[1] * vel[1, 1] + u[2] * vel[1, 2] < 0.0:
            # the link down to the laid fibre is being laid as the jet comes down: it pulls
            # the jet back when the collector stretches it, but puts up no fight shortening
            t0 = 0.0
            c = 0.0
            el[0, 3] = -l  # flag: no stress update for it this step
        for r in range(3):
            if 1 <= k <= n - 2:
                rhs[3 * (k - 1) + r] += t0 * u[r]
            if 1 <= k + 1 <= n - 2:
                rhs[3 * k + r] -= t0 * u[r]
        _add(ab, rhs, k, k, c, u, u, n, vtop)
        _add(ab, rhs, k + 1, k + 1, c, u, u, n, vtop)
        _add(ab, rhs, k, k + 1, -c, u, u, n, vtop)
        _add(ab, rhs, k + 1, k, -c, u, u, n, vtop)
        if t0 != 0.0:
            # the tension turning with the link (geometric stiffness T / l (I - u u^T)),
            # implicit too: a taut thread of near-massless beads carries very fast
            # sideways waves that an explicit step can't follow. Under compression it is
            # negative (buckling); kept to a quarter of the lighter bead's m / dt so the
            # system stays solvable - the bending stiffness sets the buckling rate
            cg = dt * t0 / l
            if cg < 0.0:
                lightest = 1e300
                if 1 <= k <= n - 2:
                    lightest = min(lightest, rho * 0.5 * (vol[k - 1] + vol[k]))
                if 1 <= k + 1 <= n - 2:
                    lightest = min(lightest, rho * 0.5 * (vol[k] + vol[k + 1]))
                cg = max(cg, -0.25 * lightest / dt)
            for r in range(3):
                for t in range(3):
                    geo[r, t] = cg * ((1.0 if r == t else 0.0) - u[r] * u[t])
                    neg[r, t] = -geo[r, t]
            _add_block(ab, rhs, k, k, geo, n, vtop)
            _add_block(ab, rhs, k + 1, k + 1, geo, n, vtop)
            _add_block(ab, rhs, k, k + 1, neg, n, vtop)
            _add_block(ab, rhs, k + 1, k, neg, n, vtop)
    # joints: Maxwell bending moment on the curvature vector kappa = (u2 - u1) / lbar,
    # d(Mv)/dt = G I dkappa/dt - Mv / theta, force on bead a = -lbar J_a Mv with
    # J_a = dkappa/dr_a (J for the bead below P1 / (l1 lbar), above P2 / (l2 lbar),
    # middle minus their sum; P = I - u u^T). Smooth through straight and folded joints.
    blk = np.empty((3, 3))
    for i in range(1, n - 1):
        l1, l2 = abs(el[i - 1, 3]), abs(el[i, 3])
        lbar = 0.5 * (l1 + l2)
        area = 0.5 * (vol[i - 1] / l1 + vol[i] / l2)
        inertia = area * area / (4.0 * math.pi)
        gv[i, 0] = inertia
        gv[i, 1] = lbar
        for r in range(3):
            for t in range(3):
                eye = 1.0 if r == t else 0.0
                gj[i, 0, r, t] = (eye - el[i - 1, r] * el[i - 1, t]) / (l1 * lbar)
                gj[i, 2, r, t] = (eye - el[i, r] * el[i, t]) / (l2 * lbar)
                gj[i, 1, r, t] = -(gj[i, 0, r, t] + gj[i, 2, r, t])
        k_t, k_v = gv[i, 2], gv[i, 3]
        c = lbar * k_v * inertia
        for a in range(3):
            bead = i - 1 + a
            if 1 <= bead <= n - 2:
                for r in range(3):
                    f = 0.0
                    for t in range(3):
                        f += gj[i, a, r, t] * mom[i, t]
                    rhs[3 * (bead - 1) + r] -= lbar * k_t * f
            for b2 in range(3):
                for r in range(3):
                    for t in range(3):
                        acc = 0.0
                        for w in range(3):
                            acc += gj[i, a, r, w] * gj[i, b2, w, t]
                        blk[r, t] = c * acc
                _add_block(ab, rhs, bead, i - 1 + b2, blk, n, vtop)
    _cholesky_solve(ab, rhs, m)
    for i in range(1, n - 1):
        for r in range(3):
            vel[i, r] = rhs[3 * (i - 1) + r]
    vel[0, 0], vel[0, 1], vel[0, 2] = 0.0, 0.0, 0.0
    vel[n - 1, 0], vel[n - 1, 1], vel[n - 1, 2] = vtop[0], vtop[1], vtop[2]
    # new stresses and moments from the new velocities
    if el[0, 3] < 0.0:
        sig[0] = 0.0
        el[0, 3] = -el[0, 3]
    else:
        rate = 0.0
        for r in range(3):
            rate += el[0, r] * (vel[1, r] - vel[0, r])
        sig[0] = sig[0] * el[0, 4] + el[0, 5] * rate / el[0, 3]
    for k in range(1, n - 1):
        rate = 0.0
        for r in range(3):
            rate += el[k, r] * (vel[k + 1, r] - vel[k, r])
        sig[k] = sig[k] * el[k, 4] + el[k, 5] * rate / el[k, 3]
    for i in range(1, n - 1):
        for r in range(3):
            rate = 0.0
            for a in range(3):
                for t in range(3):
                    rate += gj[i, a, r, t] * vel[i - 1 + a, t]
            mom[i, r] = mom[i, r] * gv[i, 2] + gv[i, 3] * gv[i, 0] * rate
    for i in range(1, n - 1):
        for r in range(3):
            pos[i, r] += dt * vel[i, r]
    for r in range(3):
        pos[n - 1, r] = top[r]
    return n


def _remove(pos, vel, vol, sig, mom, age, n, k):
    # Remove bead k (not an anchor): links k-1 and k merge
    vk = vol[k - 1] + vol[k]
    sk = (sig[k - 1] * vol[k - 1] + sig[k] * vol[k]) / vk if vk > 0 else 0.0
    vol[k - 1] = vk
    sig[k - 1] = sk
    for i in range(k, n - 1):
        pos[i] = pos[i + 1]
        vel[i] = vel[i + 1]
        mom[i] = mom[i + 1]
        age[i] = age[i + 1]
    for i in range(k, n - 2):
        vol[i] = vol[i + 1]
        sig[i] = sig[i + 1]
    return n - 1


def _insert(pos, vel, vol, sig, mom, age, n, k):
    # Split link k (beads k, k+1) in two with a bead at its middle
    for i in range(n, k + 1, -1):
        pos[i] = pos[i - 1]
        vel[i] = vel[i - 1]
        mom[i] = mom[i - 1]
        age[i] = age[i - 1]
    for i in range(n - 1, k + 1, -1):
        vol[i] = vol[i - 1]
        sig[i] = sig[i - 1]
    for r in range(3):
        pos[k + 1, r] = 0.5 * (pos[k, r] + pos[k + 2, r])
        vel[k + 1, r] = 0.5 * (vel[k, r] + vel[k + 2, r])
    mom[k + 1] = 0.0
    age[k + 1] = 0.5 * (age[k] + age[k + 2])
    vol[k] *= 0.5
    vol[k + 1] = vol[k]
    sig[k + 1] = sig[k]
    return n + 1


def _remesh(pos, vel, vol, sig, mom, age, n, H, lmax):
    # Split links longer than the local maximum (lmax at the nozzle, lmax/6 at the collector),
    # merge ones under a quarter of it
    k = 0
    while k < n - 1 and n < CAPACITY - 1:
        dx = pos[k + 1, 0] - pos[k, 0]
        dy = pos[k + 1, 1] - pos[k, 1]
        dz = pos[k + 1, 2] - pos[k, 2]
        l = math.sqrt(dx * dx + dy * dy + dz * dz)
        zmid = max(0.0, 0.5 * (pos[k, 2] + pos[k + 1, 2]))
        local = lmax * (1.0 / 6.0 + (5.0 / 6.0) * min(1.0, zmid / H))
        if l > local:
            n = _insert(pos, vel, vol, sig, mom, age, n, k)
            continue
        if l < 0.25 * local and k + 1 <= n - 2 and n > 4:
            n = _remove(pos, vel, vol, sig, mom, age, n, k + 1)
            continue
        k += 1
    return n


def _land(pos, vel, vol, sig, mom, age, n, dep, nd):
    # Beads that reach the collector stick there. The lowest airborne bead becomes the new
    # laid anchor (its link to the old anchor is laid fibre); others are held at z = 0.
    while n > 4 and pos[1, 2] <= 0.0:
        if nd < dep.shape[0]:
            dep[nd, 0] = pos[1, 0]
            dep[nd, 1] = pos[1, 1]
            nd += 1
        for i in range(0, n - 1):
            pos[i] = pos[i + 1]
            vel[i] = vel[i + 1]
            mom[i] = mom[i + 1]
            age[i] = age[i + 1]
        for i in range(0, n - 2):
            vol[i] = vol[i + 1]
            sig[i] = sig[i + 1]
        n -= 1
        pos[0, 2] = 0.0
        vel[0, 0], vel[0, 1], vel[0, 2] = 0.0, 0.0, 0.0
        mom[0] = 0.0
    for i in range(1, n - 1):
        if pos[i, 2] < 0.0:
            pos[i, 2] = 0.0
            if vel[i, 2] < 0.0:
                vel[i, 2] = 0.0
    return n, nd


def _run(gx, gy, t, t_prev, x_prev, y_prev, pos, vel, vol, sig, mom, age, n, H, Q, lmax, rho, mu, theta, alpha, g, q,
         coulomb, tau, cap, out_cx, out_cy, dep, nd, snap_every, next_snap, snaps, snap_n, snap_t, ns, ab, rhs, gj, gv, el):
    # Drive the chain through nozzle positions (gx, gy) at times t (all SI). Writes the
    # contact point per position, laid bead positions (dep) and jet snapshots.
    top = np.zeros(3)
    vtop = np.zeros(3)
    for p in range(gx.shape[0]):
        span = t[p] - t_prev
        if span > 0.0:
            vx = (gx[p] - x_prev) / span
            vy = (gy[p] - y_prev) / span
            # substeps so no bead crosses more than a bottom link in one go
            vmax = math.hypot(vx, vy)
            for i in range(1, n - 1):
                s2 = math.sqrt(vel[i, 0] ** 2 + vel[i, 1] ** 2 + vel[i, 2] ** 2)
                if s2 > vmax:
                    vmax = s2
            sub = int(min(64, max(1, math.ceil(vmax * span / (lmax / 6.0)))))
            h = span / sub
            for s in range(sub):
                f = (s + 1.0) / sub
                top[0] = x_prev + (gx[p] - x_prev) * f
                top[1] = y_prev + (gy[p] - y_prev) * f
                top[2] = H
                vtop[0], vtop[1], vtop[2] = vx, vy, 0.0
                # extrusion: the link at the nozzle gains this step's flow
                old = vol[n - 2]
                vol[n - 2] = old + Q * h
                sig[n - 2] *= old / vol[n - 2]
                n = _step(pos, vel, vol, sig, mom, age, n, h, top, vtop, rho, mu, theta, alpha, g, q, coulomb, tau, cap,
                          ab, rhs, gj, gv, el)
                for i in range(n - 1):
                    age[i] += h
                age[n - 1] = 0.0
                n, nd = _land(pos, vel, vol, sig, mom, age, n, dep, nd)
                n = _remesh(pos, vel, vol, sig, mom, age, n, H, lmax)
            x_prev, y_prev, t_prev = gx[p], gy[p], t[p]
        # contact point: the laid anchor, slid towards the lowest airborne bead as it comes down
        ax, ay = pos[0, 0], pos[0, 1]
        dx, dy = pos[1, 0] - ax, pos[1, 1] - ay
        dxy = math.hypot(dx, dy)
        w = dxy / (dxy + max(pos[1, 2], 0.0) + 1e-15)
        out_cx[p] = ax + dx * w
        out_cy[p] = ay + dy * w
        if snap_every > 0.0 and t[p] >= next_snap and ns < snaps.shape[0]:
            for i in range(n):
                snaps[ns, i, 0] = pos[i, 0]
                snaps[ns, i, 1] = pos[i, 1]
                snaps[ns, i, 2] = pos[i, 2]
            snap_n[ns] = n
            snap_t[ns] = t[p]
            ns += 1
            next_snap = t[p] + snap_every
    return n, nd, ns, next_snap, t_prev, x_prev, y_prev


if njit is not None:
    viscosity_factor = njit(cache=True)(viscosity_factor)
    _add = njit(cache=True)(_add)
    _add_block = njit(cache=True)(_add_block)
    _cholesky_solve = njit(cache=True)(_cholesky_solve)
    _step = njit(cache=True)(_step)
    _remove = njit(cache=True)(_remove)
    _insert = njit(cache=True)(_insert)
    _remesh = njit(cache=True)(_remesh)
    _land = njit(cache=True)(_land)
    _run = njit(cache=True, nogil=True)(_run)


class RopeJet:
    """The bead-chain jet, driven by nozzle positions (mm, s). Keeps the laid fibre and,
    every Rope_snapshot_ms, the jet's shape for the 3D render."""

    def __init__(self, variables, cts_mm_min, x0_mm, y0_mm, quiet=False):
        s = settings(variables)
        self.s = s
        self.H = float(s["Rope_collector_distance_mm"]) / 1000.0
        self.vc = float(cts_mm_min) / 60.0 / 1000.0
        d = float(s["Fibre_diameter_um"]) * 1e-6
        self.Q = math.pi * d * d / 4.0 * self.vc
        dn = float(s["Rope_start_diameter_um"]) * 1e-6
        self.v0 = self.Q / (math.pi * dn * dn / 4.0)
        self.rho = float(s["Density_kg_m3"])
        self.mu = float(s["Rope_viscosity_Pa_s"])
        self.theta = float(s["Rope_relaxation_ms"]) / 1000.0
        self.alpha = float(s["Rope_surface_tension_N_m"])
        self.coulomb = bool(s["Rope_coulomb"])
        self.lmax = float(s["Rope_link_mm"]) / 1000.0
        self.tau = float(s["Rope_cooling_ms"]) / 1000.0
        self.cap = max(float(s["Rope_solid_ratio"]), 1.0)
        self.g, prof_s, prof_v, prof_T, prof_age = steady_jet(self.H, self.v0, self.vc, self.Q, self.rho, self.mu,
                                                              self.alpha, self.tau, self.cap)
        self.transit = float(prof_age[-1])
        field = float(s["Voltage_kV"]) * 1000.0 / self.H
        self.q = max(self.g - G_EARTH, 0.0) / field  # charge per unit mass, C/kg
        self.pos = np.zeros((CAPACITY, 3))
        self.vel = np.zeros((CAPACITY, 3))
        self.vol = np.zeros(CAPACITY)
        self.sig = np.zeros(CAPACITY)
        self.mom = np.zeros((CAPACITY, 3))
        self.age = np.zeros(CAPACITY)
        self._init_jet(prof_s, prof_v, prof_age, x0_mm / 1000.0, y0_mm / 1000.0)
        self.ab = np.zeros((HB + 1, 3 * CAPACITY))
        self.rhs = np.zeros(3 * CAPACITY)
        self.gj = np.zeros((CAPACITY, 3, 3, 3))
        self.gv = np.zeros((CAPACITY, 4))
        self.el = np.zeros((CAPACITY, 6))
        self.t_prev, self.x_prev, self.y_prev = 0.0, x0_mm / 1000.0, y0_mm / 1000.0
        self.started = False
        self.laid = [np.array([[x0_mm / 1000.0, y0_mm / 1000.0]])]
        self.snap_every = float(s["Rope_snapshot_ms"]) / 1000.0
        self.next_snap = 0.0
        self.snaps, self.snap_n, self.snap_t = [], [], []
        self.nozzle_track = []
        if not quiet:
            print(f"Rope jet model: {self.n} beads, height {self.H * 1000:g} mm, lands at {self.vc * 60000:.1f} mm/min, "
                  f"leaves the nozzle at {self.v0 * 60000:.3f} mm/min, field pull {self.g:.3g} m/s^2, "
                  f"charge {self.q * 1e3:.3g} mC/kg, viscosity {self.mu:g} Pa s, relaxation {self.theta * 1000:g} ms, "
                  f"cooling {self.tau * 1000:g} ms (x{self.cap:g} when solid), transit {self.transit:.2f} s")

    def _init_jet(self, prof_s, prof_v, prof_age, x0, y0):
        # The steady vertical jet, beads spaced to the local link length, nozzle at the top
        H, lmax = self.H, self.lmax
        s_pts = [H]
        while s_pts[-1] > 0:
            z = H - s_pts[-1]
            local = lmax * (1 / 6 + 5 / 6 * min(1.0, max(z, 0.0) / H)) * 0.9
            s_pts.append(max(s_pts[-1] - local, 0.0))
        s_pts = np.array(s_pts)  # collector first
        n = len(s_pts)
        if n > CAPACITY - 10:
            raise ValueError("Rope model: too many beads, raise Rope_link_mm")
        v = np.interp(s_pts, prof_s, prof_v)
        self.pos[:n, 0], self.pos[:n, 1], self.pos[:n, 2] = x0, y0, H - s_pts
        self.vel[:n, 2] = -v
        self.vel[0, 2] = 0.0
        self.vel[n - 1, 2] = 0.0
        dense = np.linspace(0, H, 4000)
        inv_v = np.interp(dense, prof_s, 1.0 / prof_v)
        cum = np.concatenate(([0.0], np.cumsum(0.5 * (inv_v[1:] + inv_v[:-1]) * np.diff(dense))))
        vol_at = np.interp(s_pts, dense, cum) * self.Q  # volume from the nozzle down to s
        self.vol[:n - 1] = np.abs(np.diff(vol_at))
        # stress: viscous part of the tension over the area, mu dv/ds
        dv = np.gradient(prof_v, prof_s)
        s_mid = 0.5 * (s_pts[1:] + s_pts[:-1])
        factor = np.array([viscosity_factor(a, self.tau, self.cap) for a in np.interp(s_mid, prof_s, prof_age)])
        self.sig[:n - 1] = self.mu * factor * np.interp(s_mid, prof_s, dv)
        self.age[:n] = np.interp(s_pts, prof_s, prof_age)
        self.n = n

    def run(self, gx_mm, gy_mm, t_s, record=True):
        """Advance through nozzle positions; returns the contact points (mm)."""
        gx = np.ascontiguousarray(gx_mm, dtype=np.float64) / 1000.0
        gy = np.ascontiguousarray(gy_mm, dtype=np.float64) / 1000.0
        t = np.ascontiguousarray(t_s, dtype=np.float64)
        if not self.started and len(t):
            self.t_prev = float(t[0])
            self.x_prev, self.y_prev = float(gx[0]), float(gy[0])
            self.next_snap = self.t_prev
            self.started = True
        npts = len(gx)
        cx, cy = np.empty(npts), np.empty(npts)
        dep = np.empty((npts * 8 + 256, 2))
        nsnap = int(((t[-1] - t[0]) / self.snap_every) + 3) if (record and self.snap_every > 0 and npts) else 0
        snaps = np.zeros((nsnap, CAPACITY, 3), dtype=np.float64)
        snap_n = np.zeros(nsnap, dtype=np.int64)
        snap_t = np.zeros(nsnap)
        self.n, nd, ns, self.next_snap, self.t_prev, self.x_prev, self.y_prev = _run(
            gx, gy, t, self.t_prev, self.x_prev, self.y_prev, self.pos, self.vel, self.vol, self.sig, self.mom, self.age,
            self.n, self.H, self.Q, self.lmax, self.rho, self.mu, self.theta, self.alpha, self.g, self.q, self.coulomb,
            self.tau, self.cap, cx, cy,
            dep, 0, self.snap_every if nsnap else 0.0, self.next_snap, snaps, snap_n, snap_t, 0,
            self.ab, self.rhs, self.gj, self.gv, self.el)
        if record:
            self.laid.append(dep[:nd].copy())
            for i in range(ns):
                k = int(snap_n[i])
                self.snaps.append(snaps[i, :k].astype(np.float32) * 1000.0)
                self.snap_t.append(float(snap_t[i]))
                j = min(np.searchsorted(t, snap_t[i]), npts - 1)
                self.nozzle_track.append((float(snap_t[i]), gx[j] * 1000.0, gy[j] * 1000.0))
        return cx * 1000.0, cy * 1000.0

    def save(self, path, programmed=None):
        """Jet snapshots and the laid fibre for the 3D render (mm)."""
        laid = np.concatenate(self.laid) * 1000.0 if self.laid else np.zeros((0, 2))
        counts = np.array([len(s) for s in self.snaps], dtype=np.int64)
        flat = np.concatenate(self.snaps) if self.snaps else np.zeros((0, 3), dtype=np.float32)
        np.savez_compressed(path, snap_t=np.array(self.snap_t), snap_counts=counts, snap_xyz=flat,
                            nozzle=np.array(self.nozzle_track), laid=laid.astype(np.float32),
                            height_mm=self.H * 1000.0, cts_mm_min=self.vc * 60000.0,
                            programmed=np.zeros((0, 4)) if programmed is None else np.asarray(programmed, dtype=np.float32))


def steady_lag(variables, cts_mm_min, ratio, seconds=None, dt=0.001):
    """Lag (mm) of a jet dragged in a straight line at ratio x CTS, once settled."""
    v = ratio * cts_mm_min / 60.0  # mm/s
    jet = RopeJet(variables, cts_mm_min, 0.0, 0.0, quiet=True)
    if seconds is None:
        # long enough for several jet transit times and to lay well past the lag
        seconds = 1.0 + 15.0 / v
    t = np.arange(1, int(seconds / dt) + 1) * dt
    x = v * t
    cx, cy = jet.run(x, np.zeros_like(x), t, record=False)
    tail = slice(int(len(t) * 0.75), None)
    return float(np.mean(x[tail] - cx[tail])), float(np.std(x[tail] - cx[tail]))
