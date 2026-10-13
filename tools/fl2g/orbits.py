# SPDX-License-Identifier: BSD-3-Clause
"""Broadcast-ephemeris satellite position / clock (independent numpy port of the standard algorithms,
structured like RTKLIB ephemeris.c eph2pos/geph2pos so the constants and branches can be compared line by line).

Kepler (GPS/GAL/BDS/QZS): IS-GPS-200 / Galileo OS-SIS-ICD / BDS-SIS-ICD with RTKLIB's per-system mu and omega_e,
the BDS GEO branch (prn <= 5 or >= 59: rotate by -5 deg about x then by omega_e*tk about z) and the relativistic
clock term -2 sqrt(mu A) e sinE / c^2. `toes` is the ephemeris reference time in the *system's own* week seconds
(BDS: BDT) and enters every system through omega_e*toes (DESIGN §12.3: a BDS toes 14 s off = 27 km).
GLONASS: RK4 on the PZ-90 equations (J2 + Earth rotation + broadcast luni-solar acceleration), 60 s steps,
clock -taun + gamn*(t - toe).
Eph dicts use RTKLIB field names (rinexnav.py builds them from RINEX, gnss_msgs.py from the bag); times are GPST s.
"""
import math

import numpy as np

CLIGHT = 299792458.0
MU = {"G": 3.9860050e14, "J": 3.9860050e14, "E": 3.986004418e14, "C": 3.986004418e14}
OMGE = {"G": 7.2921151467e-5, "J": 7.2921151467e-5, "E": 7.2921151467e-5, "C": 7.292115e-5}
MU_GLO = 3.9860044e14
OMGE_GLO = 7.292115e-5
J2_GLO = 1.0826257e-3
RE_GLO = 6378136.0
SIN_5 = -0.0871557427476582   # sin(-5 deg)
COS_5 = 0.9961946980917456    # cos(-5 deg)
RTOL_KEPLER = 1e-13
MAX_ITER_KEPLER = 30
TSTEP_GLO = 60.0


def kepler_pos_clk(eph, t):
    """Satellite ECEF position [m] and clock bias [s] at GPST t (float seconds) from a Kepler ephemeris."""
    sys, prn = eph["sys"], eph["prn"]
    mu, omge = MU[sys], OMGE[sys]
    A, e = eph["A"], eph["e"]
    if A <= 0.0:
        raise ValueError("ephemeris %s: A <= 0" % eph.get("sat"))
    tk = t - eph["toe"]
    M = eph["M0"] + (math.sqrt(mu / (A * A * A)) + eph["deln"]) * tk
    E = M
    Ek = 0.0
    for _ in range(MAX_ITER_KEPLER):
        Ek = E
        E -= (E - e * math.sin(E) - M) / (1.0 - e * math.cos(E))
        if abs(E - Ek) < RTOL_KEPLER:
            break
    sinE, cosE = math.sin(E), math.cos(E)
    u = math.atan2(math.sqrt(1.0 - e * e) * sinE, cosE - e) + eph["omg"]
    r = A * (1.0 - e * cosE)
    i = eph["i0"] + eph["idot"] * tk
    sin2u, cos2u = math.sin(2.0 * u), math.cos(2.0 * u)
    u += eph["cus"] * sin2u + eph["cuc"] * cos2u
    r += eph["crs"] * sin2u + eph["crc"] * cos2u
    i += eph["cis"] * sin2u + eph["cic"] * cos2u
    x, y = r * math.cos(u), r * math.sin(u)
    cosi = math.cos(i)
    if sys == "C" and (prn <= 5 or prn >= 59):
        O = eph["OMG0"] + eph["OMGd"] * tk - omge * eph["toes"]
        sinO, cosO = math.sin(O), math.cos(O)
        xg = x * cosO - y * cosi * sinO
        yg = x * sinO + y * cosi * cosO
        zg = y * math.sin(i)
        sino, coso = math.sin(omge * tk), math.cos(omge * tk)
        rs = (xg * coso + yg * sino * COS_5 + zg * sino * SIN_5,
              -xg * sino + yg * coso * COS_5 + zg * coso * SIN_5,
              -yg * SIN_5 + zg * COS_5)
    else:
        O = eph["OMG0"] + (eph["OMGd"] - omge) * tk - omge * eph["toes"]
        sinO, cosO = math.sin(O), math.cos(O)
        rs = (x * cosO - y * cosi * sinO, x * sinO + y * cosi * cosO, y * math.sin(i))
    tc = t - eph["toc"]
    dts = eph["af0"] + eph["af1"] * tc + eph["af2"] * tc * tc
    dts -= 2.0 * math.sqrt(mu * A) * e * sinE / (CLIGHT * CLIGHT)
    return np.array(rs), dts


def _glo_deq(x, acc):
    r2 = x[0] * x[0] + x[1] * x[1] + x[2] * x[2]
    xdot = np.zeros(6)
    if r2 <= 0.0:
        return xdot
    r3 = r2 * math.sqrt(r2)
    omg2 = OMGE_GLO * OMGE_GLO
    a = 1.5 * J2_GLO * MU_GLO * RE_GLO * RE_GLO / r2 / r3
    b = 5.0 * x[2] * x[2] / r2
    c = -MU_GLO / r3 - a * (1.0 - b)
    xdot[0:3] = x[3:6]
    xdot[3] = (c + omg2) * x[0] + 2.0 * OMGE_GLO * x[4] + acc[0]
    xdot[4] = (c + omg2) * x[1] - 2.0 * OMGE_GLO * x[3] + acc[1]
    xdot[5] = (c - 2.0 * a) * x[2] + acc[2]
    return xdot


def _glo_rk4(t, x, acc):
    k1 = _glo_deq(x, acc)
    k2 = _glo_deq(x + k1 * t / 2.0, acc)
    k3 = _glo_deq(x + k2 * t / 2.0, acc)
    k4 = _glo_deq(x + k3 * t, acc)
    return x + (k1 + 2.0 * k2 + 2.0 * k3 + k4) * t / 6.0


def glo_pos_clk(geph, t):
    """GLONASS ECEF (PZ-90 ~ WGS-84 at cm level, ignored as in DESIGN §1) position [m] and clock [s] at GPST t."""
    dt = t - geph["toe"]
    dts = -geph["taun"] + geph["gamn"] * dt
    x = np.array(list(geph["pos"]) + list(geph["vel"]), dtype=float)
    acc = np.array(geph["acc"], dtype=float)
    step = -TSTEP_GLO if dt < 0.0 else TSTEP_GLO
    while abs(dt) > 1e-9:
        if abs(dt) < TSTEP_GLO:
            step = dt
        x = _glo_rk4(step, x, acc)
        dt -= step
    return x[0:3].copy(), dts


def pos_clk(eph, t):
    if eph["sys"] == "R":
        return glo_pos_clk(eph, t)
    return kepler_pos_clk(eph, t)


def pos_vel_clk(eph, t, h=0.5):
    """Position, velocity (central difference over +-h, truncation ~1e-6 m/s for GNSS orbits), clock bias and drift.
    At t ~ 1.7e9 s a float64 time resolves 2**-22 s (2.4e-7 s): t +- h is exact for h = 0.5 but rounded for a step
    that is not a multiple of it (e.g. 0.05); dividing by the representable step t1 - t0 (exact) instead of 2h keeps
    the velocity at the truncation error for any h (dividing by 2h would err by up to |v| * 2.4e-7 / (2h))."""
    t0, t1 = t - h, t + h
    p0, c0 = pos_clk(eph, t0)
    p1, c1 = pos_clk(eph, t1)
    p, c = pos_clk(eph, t)
    dt = t1 - t0
    return p, (p1 - p0) / dt, c, (c1 - c0) / dt


GAL_INAV_BIT = 1 << 9          # RTKLIB-EX ephemeris.c seleph: I/NAV = code bit 9 (RINEX "data sources" E5b,E1 clock)
GAL_FNAV_BIT = 1 << 8          # F/NAV = code bit 8 (E5a,E1 clock)


def is_gal_inav(code):
    return bool(int(code or 0) & GAL_INAV_BIT)


def is_gal_fnav(code):
    return bool(int(code or 0) & GAL_FNAV_BIT)


class EphSelector:
    """The ephemeris the core would use at time t for a satellite: RTKLIB-EX 2.5.1 seleph/selgeph semantics
    (ephemeris.c:432-500) followed by the core's L0 health and age masks (DESIGN §10, A16):
      * Galileo with gal_sel = 1 (the core's default, `gal_prefer_inav` => setseleph(SYS_GAL, 1)): I/NAV records only
        (code bit 9); gal_sel = 2: F/NAV only (bit 8); 0: any. For Galileo a toe at or after t is never used
        (seleph: timediff(toe, time) >= 0 -> continue, "AOD <= 0");
      * the record with the smallest |t - toe| is selected (no health filter inside seleph); ties go to the later
        toe (seleph keeps the last of equal |dt| in its ttr/toe-sorted array);
      * the selected record must then be healthy (svh == 0) and |t - toe| <= max_age[system] (L0), otherwise the
        satellite has no usable ephemeris at t — an older healthy record is NOT substituted.
    `causal` restricts each record to t_record >= its record time (the causal "bag-only" availability, A1)."""

    def __init__(self, records, max_age, causal=False, gal_sel=1):
        self.by_sat = {}
        for r in records:
            self.by_sat.setdefault(r["sat"], []).append(r)
        self.max_age = max_age
        self.causal = causal
        self.gal_sel = gal_sel

    def candidates(self, sat, t, t_record=None):
        sys = sat[0]
        out = []
        for r in self.by_sat.get(sat, ()):
            if self.causal and t_record is not None and r.get("t_record") is not None and r["t_record"] > t_record:
                continue
            if sys == "E":
                if self.gal_sel == 1 and not is_gal_inav(r.get("code", 0)):
                    continue
                if self.gal_sel == 2 and not is_gal_fnav(r.get("code", 0)):
                    continue
                if r["toe"] - t >= 0.0:
                    continue
            out.append(r)
        return out

    def select(self, sat, t, t_record=None):
        cands = self.candidates(sat, t, t_record)
        if not cands:
            return None
        best = min(cands, key=lambda r: (abs(t - r["toe"]), -r["toe"]))
        if best.get("svh", 0) != 0 or abs(t - best["toe"]) > self.max_age.get(sat[0], 7200.0):
            return None
        return best


def eph_is_healthy(r):
    return r.get("svh", 0) == 0
