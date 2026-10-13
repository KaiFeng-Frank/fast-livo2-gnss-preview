# SPDX-License-Identifier: BSD-3-Clause
"""Orbit, time-scale, ephemeris-selection and least-squares helpers used by the audit and the time calibration.

The strong validation of the orbit code is on real data (audit checks E1/E3: bag vs RINEX records and adjacent
RINEX records agree to mm / m); these tests pin the conventions that data cannot isolate: the Earth-rotation sign
and the use of toes, Kepler's equation and the true anomaly, the BDS GEO branch, the GLONASS equations of motion,
the Doppler model (incl. Sagnac rate), the SPP mechanics (ISB, rejection) and the RTKLIB seleph semantics.

The least-squares forward model shares only the orbit/clock evaluation and the iono/trop models with the solver:
the range comes from rotating the satellite's emission-time ECEF position into the Earth frame at reception
(Rz(omega_e tau), light time iterated) — not from geodesy.geodist's first-order Sagnac formula; the range rate is a
numerical derivative of that rotated-frame range with the satellite at a fixed emission offset and the rotation
angle the flight time's own (the DESIGN §1 convention: the 1 - rho_dot/c emission-rate factor is neglected) — not
the solver's Sagnac-rate expression; the group-delay terms are written from the ICDs. A Sagnac or sign convention
error in the solver therefore no longer cancels (checked with a flipped rotation). This found the Sagnac-rate sign
of RTKLIB resdop / DESIGN §1 reversed (see gnsslsq). Residual model differences: second-order Sagnac (< 1 mm), the
solver's +-0.5 s satellite-velocity difference (~1e-5 m/s)."""
import math
import os
import tempfile
import unittest

import numpy as np

import _path  # noqa: F401
from fl2g import geodesy, gnsslsq, gnsstime, orbits, rinexnav, sats

OMGE = 7.2921151467e-5

RNX_HEADER = """     3.04           N: GNSS NAV DATA    M: MIXED            RINEX VERSION / TYPE
GPSA   2.7940D-08  2.2352D-08 -1.1921D-07  5.9605D-08       IONOSPHERIC CORR
GPSB   1.5155D+05 -1.9661D+05 -6.5536D+04  3.2768D+05       IONOSPHERIC CORR
    18                                                      LEAP SECONDS
                                                            END OF HEADER
"""
# public IGS broadcast records (BRDC00IGS_R_20250030000_01D_MN.rnx), used for the GEO / GLONASS geometry checks
C01 = """C01 2025 01 03 00 00 00-3.585217054930E-04 3.610445276080E-12 0.000000000000E+00
     1.000000000000E+00-5.314218750000E+02 3.764799676160E-09-6.810587366470E-01
    -1.718569546940E-05 4.776980495080E-04 1.979898661380E-05 6.493571779250E+03
     4.320000000000E+05 1.010484993460E-07-2.771391901680E+00-2.281740307810E-08
     5.904981198170E-02-5.999687500000E+02-2.239320449260E-01-2.838332513660E-09
    -6.593131773260E-10 0.000000000000E+00 9.910000000000E+02 0.000000000000E+00
     2.000000000000E+00 0.000000000000E+00-5.199999986160E-09-9.700000000000E-09
     4.320000000000E+05 0.000000000000E+00
"""
R01 = """R01 2025 01 02 23 45 00 1.098988577724E-04 9.094947017729E-13 4.302000000000E+05
     1.258341308594E+04 2.554470062256E+00-1.862645149231E-09 0.000000000000E+00
    -1.135215869141E+04-5.484800338745E-01 9.313225746155E-10 1.000000000000E+00
    -1.905884130859E+04 2.013564109802E+00 1.862645149231E-09 0.000000000000E+00
"""


def read_records(text):
    with tempfile.TemporaryDirectory() as d:
        p = os.path.join(d, "nav.rnx")
        with open(p, "w") as f:
            f.write(RNX_HEADER + text)
        return rinexnav.read_nav(p)


def circular_eph(prn=5, sys="G", A=26560e3, e=0.0, i0=0.96, OMG0=0.5, omg=0.3, M0=1.0, toe=None):
    toe = gnsstime.gpst(2350, 3600.0) if toe is None else toe
    w, tow = gnsstime.week_tow(toe)
    return {"sys": sys, "prn": prn, "sat": sats.satid(sys, prn), "A": A, "e": e, "i0": i0, "OMG0": OMG0, "omg": omg,
            "M0": M0, "deln": 0.0, "OMGd": 0.0, "idot": 0.0, "cuc": 0.0, "cus": 0.0, "crc": 0.0, "crs": 0.0, "cic": 0.0,
            "cis": 0.0, "toe": toe, "toes": tow, "toc": toe, "af0": 1e-4, "af1": 2e-12, "af2": 0.0, "svh": 0, "code": 0,
            "tgd0": 3e-9, "tgd1": 0.0, "week": w}


def ecef_to_eci(r, t, eph):
    """Undo the Earth rotation implied by the broadcast model: ECI-like = Rz(omega_e (tk + toes)) r."""
    a = OMGE * (t - eph["toe"] + eph["toes"])
    c, s = math.cos(a), math.sin(a)
    return np.array([c * r[0] - s * r[1], s * r[0] + c * r[1], r[2]])


class Orbits(unittest.TestCase):
    def test_time_scales(self):
        self.assertEqual(gnsstime.leap_at_utc(1735888008.0), 18)
        self.assertEqual(gnsstime.leap_at_utc(1483228799.0), 17)       # 2016-12-31T23:59:59Z
        self.assertEqual(gnsstime.leap_at_gpst(gnsstime.utc_to_gpst(1483228800.0)), 18)
        t = gnsstime.gpst(2342, 130626.1)
        self.assertAlmostEqual(gnsstime.gpst_to_utc(t), 1732537008.1, places=6)   # Dark01 first PVT (DATA_NOTES)
        w, tow = gnsstime.gpst_to_bdt_week_tow(t)
        self.assertEqual(w, 2342 - 1356)
        self.assertAlmostEqual(tow, 130626.1 - 14.0, places=6)
        self.assertAlmostEqual(gnsstime.bdt_to_gpst(w, tow), t, places=6)

    def test_kepler_rotation_sign_and_toes(self):
        eph = circular_eph()
        n = math.sqrt(orbits.MU["G"] / eph["A"] ** 3)
        normals = []
        for k in range(0, 7200, 600):
            t = eph["toe"] + k
            r, _ = orbits.kepler_pos_clk(eph, t)
            self.assertAlmostEqual(np.linalg.norm(r), eph["A"], delta=1e-6)
            r2, _ = orbits.kepler_pos_clk(eph, t + 1.0)
            ri, ri2 = ecef_to_eci(r, t, eph), ecef_to_eci(r2, t + 1.0, eph)
            nrm = np.cross(ri, ri2)
            normals.append(nrm / np.linalg.norm(nrm))
            # inertial angular rate of the satellite equals the mean motion (circular orbit)
            ang = math.acos(np.dot(ri, ri2) / (np.linalg.norm(ri) * np.linalg.norm(ri2)))
            self.assertAlmostEqual(ang, n, delta=1e-10)
        # the orbital plane is fixed in the de-rotated frame only if Omega decreases with omega_e (not increases)
        for v in normals[1:]:
            self.assertLess(np.linalg.norm(v - normals[0]), 1e-9)
        self.assertAlmostEqual(math.degrees(math.acos(normals[0][2])), math.degrees(eph["i0"]), places=6)

    def test_kepler_eccentric_vis_viva_and_clock(self):
        eph = circular_eph(e=0.02, M0=2.5)
        mu = orbits.MU["G"]
        for k in (0.0, 1234.5, 5000.0):
            t = eph["toe"] + k
            ri = [ecef_to_eci(orbits.kepler_pos_clk(eph, t + d)[0], t + d, eph) for d in (-0.5, 0.0, 0.5)]
            v = (ri[2] - ri[0]) / 1.0
            r = np.linalg.norm(ri[1])
            self.assertAlmostEqual(v @ v, mu * (2.0 / r - 1.0 / eph["A"]), delta=1e-3 * (v @ v) * 1e-3)
            _, dts = orbits.kepler_pos_clk(eph, t)
            # clock polynomial + relativity: the relativistic term is bounded by 2 sqrt(mu A) e / c^2
            poly = eph["af0"] + eph["af1"] * (t - eph["toc"])
            self.assertLess(abs(dts - poly), 2.0 * math.sqrt(mu * eph["A"]) * eph["e"] / orbits.CLIGHT ** 2 + 1e-15)
            self.assertGreater(abs(dts - poly), 0.0)

    def test_toes_week_seconds_enter_the_node(self):
        # the same GPST toe written with a toes 14 s off (a BDT/GPST label error) moves the satellite by ~omega_e*14*r
        eph = circular_eph(sys="C", prn=20, A=27906e3)
        bad = dict(eph, toes=eph["toes"] + 14.0)
        for k in (100.0, 1500.0, 4000.0):
            r1, _ = orbits.kepler_pos_clk(eph, eph["toe"] + k)
            r2, _ = orbits.kepler_pos_clk(bad, eph["toe"] + k)
            d = np.linalg.norm(r1 - r2)
            # a pure rotation about z by omega_e*14 s: chord 2 r_xy sin(alpha/2), up to 28.5 km (§12.3 "填错 27 km")
            alpha = orbits.OMGE["C"] * 14.0
            self.assertAlmostEqual(d, 2.0 * math.hypot(r1[0], r1[1]) * math.sin(alpha / 2.0), delta=1e-6)
            self.assertAlmostEqual(r1[2], r2[2], delta=1e-6)

    def test_bds_geo_branch(self):
        hdr, recs = read_records(C01)
        (c01,) = recs
        self.assertEqual(c01["toes"], 432000.0)
        self.assertAlmostEqual(c01["toe"] - gnsstime.bdt_to_gpst(991, 432000.0), 0.0, places=9)
        lons = []
        for k in (-7200.0, 0.0, 7200.0):
            r, _ = orbits.kepler_pos_clk(c01, c01["toe"] + k)
            lat, lon, h = geodesy.ecef2pos(r)
            self.assertAlmostEqual(np.linalg.norm(r) / 1e3, 42164.0, delta=100.0)
            self.assertLess(abs(math.degrees(lat)), 2.0)
            lons.append(math.degrees(lon))
        self.assertLess(max(lons) - min(lons), 0.5, lons)  # geostationary within the fit interval
        self.assertTrue(55.0 < lons[1] < 165.0, lons)       # inside the BDS GEO arc (58.75 E .. 160 E)
        # without the GEO branch the same elements give a non-geostationary track
        ngeo = dict(c01, prn=20)
        r0, _ = orbits.kepler_pos_clk(ngeo, c01["toe"])
        self.assertGreater(abs(math.degrees(geodesy.ecef2pos(r0)[0])), 2.0)

    def test_glonass_jacobi_integral_and_reversibility(self):
        hdr, recs = read_records(R01)
        (g,) = recs
        self.assertAlmostEqual(g["taun"], -1.098988577724e-04, places=15)     # RINEX stores -TauN
        self.assertEqual(g["frq"], 1)
        self.assertAlmostEqual(g["toe"], gnsstime.calendar_to_seconds(2025, 1, 2, 23, 45, 0) + 18.0, places=6)
        g0 = dict(g, acc=[0.0, 0.0, 0.0])

        def jacobi(x, v):
            r = np.linalg.norm(x)
            u_j2 = -0.5 * orbits.J2_GLO * orbits.MU_GLO * orbits.RE_GLO ** 2 / r ** 3 * (3.0 * x[2] ** 2 / r ** 2 - 1.0)
            return 0.5 * v @ v - 0.5 * orbits.OMGE_GLO ** 2 * (x[0] ** 2 + x[1] ** 2) - orbits.MU_GLO / r - u_j2

        x0 = np.array(g0["pos"])
        v0 = np.array(g0["vel"])
        j0 = jacobi(x0, v0)
        p1, _ = orbits.glo_pos_clk(g0, g0["toe"] + 900.0)
        p1b, _ = orbits.glo_pos_clk(g0, g0["toe"] + 900.5)
        p1a, _ = orbits.glo_pos_clk(g0, g0["toe"] + 899.5)
        j1 = jacobi(p1, p1b - p1a)
        self.assertLess(abs(j1 - j0) / abs(j0), 1e-7)
        self.assertAlmostEqual(np.linalg.norm(p1) / 1e3, 25500.0, delta=300.0)
        back = dict(g0, pos=list(p1), vel=list(p1b - p1a), toe=g0["toe"] + 900.0)
        p0, _ = orbits.glo_pos_clk(back, g0["toe"])
        self.assertLess(np.linalg.norm(p0 - x0), 0.05)
        _, dts = orbits.glo_pos_clk(g, g["toe"] + 100.0)
        self.assertAlmostEqual(dts, -g["taun"] + g["gamn"] * 100.0, places=18)

    def test_velocity_step_at_a_large_epoch(self):
        # the same orbit (same tk and toes) at t ~ 1.7e9 s and at a small epoch: a 0.05 s step is not representable
        # at 1.7e9 s, the velocity must not depend on it
        big = circular_eph(e=0.01, toe=gnsstime.gpst(2342, 130000.0))
        small = dict(big, toe=big["toe"] - gnsstime.gpst(2342, 0.0), toc=big["toc"] - gnsstime.gpst(2342, 0.0))
        for dt in (123.4567, 3333.3):
            _, v_big, _, _ = orbits.pos_vel_clk(big, big["toe"] + dt, h=0.05)
            _, v_small, _, _ = orbits.pos_vel_clk(small, small["toe"] + dt, h=0.05)
            self.assertLess(float(np.linalg.norm(v_big - v_small)), 1e-5)

    def test_rinex_header_iono_and_leap(self):
        hdr, _ = read_records("")
        self.assertEqual(hdr["leap_s"], 18)
        self.assertAlmostEqual(hdr["iono_gps_alpha"][0], 2.7940e-08)
        self.assertAlmostEqual(hdr["iono_gps_beta"][3], 3.2768e+05)


class EphSelection(unittest.TestCase):
    """orbits.EphSelector = RTKLIB-EX 2.5.1 seleph (ephemeris.c:432-468) + the core's L0 health/age masks (A16)."""
    AGE = {"G": 7200.0, "E": 7200.0, "C": 3600.0, "R": 1800.0}
    T = gnsstime.gpst(2342, 36000.0)

    def rec(self, sat, dt_toe, code=0, svh=0, t_record=None):
        return {"sat": sat, "sys": sat[0], "toe": self.T + dt_toe, "code": code, "svh": svh, "t_record": t_record}

    def test_galileo_inav_only_and_past_toe_only(self):
        inav, fnav = 517, 258                                   # RINEX data sources of I/NAV and F/NAV records
        recs = [self.rec("E11", -300.0, fnav), self.rec("E11", -600.0, inav), self.rec("E11", +60.0, inav)]
        sel = orbits.EphSelector(recs, self.AGE)
        self.assertEqual(sel.select("E11", self.T)["toe"], self.T - 600.0)        # nearer F/NAV and future I/NAV skipped
        self.assertEqual(orbits.EphSelector(recs, self.AGE, gal_sel=0).select("E11", self.T)["toe"], self.T - 300.0)
        self.assertEqual(orbits.EphSelector(recs, self.AGE, gal_sel=2).select("E11", self.T)["toe"], self.T - 300.0)
        # toe exactly at t is "AOD <= 0" for Galileo (timediff(toe, time) >= 0 -> skipped) ...
        self.assertIsNone(orbits.EphSelector([self.rec("E11", 0.0, inav)], self.AGE).select("E11", self.T))
        # ... while GPS takes the nearest toe on either side
        gps = [self.rec("G05", -900.0), self.rec("G05", +300.0)]
        self.assertEqual(orbits.EphSelector(gps, self.AGE).select("G05", self.T)["toe"], self.T + 300.0)

    def test_health_is_judged_on_the_selected_record(self):
        recs = [self.rec("G07", -3000.0), self.rec("G07", -100.0, svh=1)]
        self.assertIsNone(orbits.EphSelector(recs, self.AGE).select("G07", self.T))     # no fallback to the older one

    def test_ties_age_and_causality(self):
        recs = [self.rec("G09", -3600.0), self.rec("G09", +3600.0)]
        self.assertEqual(orbits.EphSelector(recs, self.AGE).select("G09", self.T)["toe"], self.T + 3600.0)   # later toe
        self.assertIsNone(orbits.EphSelector([self.rec("C21", -3700.0)], self.AGE).select("C21", self.T))  # L0 age (C: 1 h)
        late = [self.rec("G10", -60.0, t_record=500.0)]
        sel = orbits.EphSelector(late, self.AGE, causal=True)
        self.assertIsNone(sel.select("G10", self.T, t_record=499.0))
        self.assertIsNotNone(sel.select("G10", self.T, t_record=500.0))


def synthetic_constellation(t, rr, n=10, seed=0):
    """n circular-orbit ephemerides spread over the sky above rr at time t (GPS/GAL/BDS mix)."""
    rng = np.random.RandomState(seed)
    ephs = []
    systems = ["G", "G", "G", "E", "E", "C", "C", "G", "E", "C", "G", "E"]
    for k in range(n):
        sys = systems[k % len(systems)]
        e = circular_eph(prn=k + 1 if sys != "C" else 20 + k, sys=sys, A={"G": 26560e3, "E": 29600e3, "C": 27906e3}[sys],
                         i0=math.radians(55.0), OMG0=rng.uniform(0, 2 * math.pi), M0=rng.uniform(0, 2 * math.pi),
                         toe=t - 600.0)
        ephs.append(e)
    # keep only satellites above 15 degrees
    pos = geodesy.ecef2pos(rr)
    vis = []
    for e in ephs:
        r, _ = orbits.kepler_pos_clk(e, t)
        _, u = geodesy.geodist(r, rr)
        if geodesy.satazel(pos, u)[1] > math.radians(15.0):
            vis.append(e)
    return vis


class FakeSelector:
    def __init__(self, ephs):
        self.by = {e["sat"]: e for e in ephs}

    def select(self, sat, t, t_record=None):
        return self.by.get(sat)


def rot3(a):
    """Coordinates in a frame rotated by +a about z (the Earth frame after rotating by a)."""
    c, s_ = math.cos(a), math.sin(a)
    return np.array([[c, s_, 0.0], [-s_, c, 0.0], [0.0, 0.0, 1.0]])


def emission_range(eph, t_r, rr, rot_sign=1.0):
    """Light-time geometric range [m] and flight time [s] in the Earth frame at the true reception time t_r: the
    satellite's ECEF position at emission t_r - tau, rotated by the Earth rotation during the flight. rot_sign=-1
    builds the wrong-sign world used to show that the tests discriminate the convention."""
    tau = 0.075
    for _ in range(30):
        rs, _ = orbits.kepler_pos_clk(eph, t_r - tau)
        rho = float(np.linalg.norm(rot3(rot_sign * OMGE * tau) @ rs - rr))
        if abs(rho / sats.CLIGHT - tau) < 1e-15:
            break
        tau = rho / sats.CLIGHT
    return rho, tau


def range_rate_design(eph, t_r, rr, v, tau, rot_sign=1.0, h=0.05):
    """DESIGN §1 range rate: d/dt of the rotated-frame range with the satellite at the fixed emission offset tau and
    the Earth-rotation angle omega_e * rho / c of the current range (receiver moving at v, ECEF)."""
    def rho(t):
        rs, _ = orbits.kepler_pos_clk(eph, t - tau)
        rr_t = rr + v * (t - t_r)
        r = sats.CLIGHT * tau
        for _ in range(8):
            r = float(np.linalg.norm(rot3(rot_sign * OMGE * r / sats.CLIGHT) @ rs - rr_t))
        return r
    return (rho(t_r + h) - rho(t_r - h)) / (2.0 * h)


def group_delay_m(eph):
    """Group-delay term contained in the primary-signal pseudorange (single-frequency user), from the ICDs:
    GPS L1 C/A +c*T_GD (IS-GPS-200 20.3.3.3.3.2); Galileo E1 +c*BGD of the clock's frequency pair (F/NAV: E1,E5a =
    tgd0; I/NAV, code bit 9: E1,E5b = tgd1; OS-SIS-ICD 5.1.5); BDS B1I +c*TGD1 (tgd0; BDS-SIS-ICD 5.2.4.10)."""
    if eph["sys"] in ("G", "J"):
        return sats.CLIGHT * eph["tgd0"]
    if eph["sys"] == "E":
        return sats.CLIGHT * (eph["tgd1"] if int(eph["code"]) & (1 << 9) else eph["tgd0"])
    return sats.CLIGHT * eph["tgd0"]


class LeastSquares(unittest.TestCase):
    def setUp(self):
        self.rr = geodesy.pos2ecef(math.radians(34.03), math.radians(108.76), 400.0)
        # a small absolute GPST (float64 resolves 3e-11 s here instead of 2.4e-7 s at 1.7e9 s): the forward model's
        # numerical range rate must not be limited by time rounding (at 1.7e9 s it would be ~1e-3 m/s). Its time of
        # week puts the site (108.76 E) at ~02:00 local time: Klobuchar night floor, so the SPP weights are not
        # dominated by the iono variance term
        self.t = 240300.0
        # search seeds until a rich sky is visible (deterministic): >= 10 satellites, >= 4 GPS, >= 2 of every
        # system (a lone satellite of a system is absorbed by its ISB)
        for seed in range(200):
            ephs = synthetic_constellation(self.t, self.rr, n=24, seed=seed)
            count = {k: sum(e["sys"] == k for e in ephs) for k in "GEC"}
            if len(ephs) >= 10 and count["G"] >= 4 and min(count.values()) >= 2:
                break
        self.ephs = ephs
        self.sel = FakeSelector(ephs)

    def make_obs(self, clk=3000.0, isb=None, v=np.array([1.2, -0.4, 0.05]), d=45.0, iono=None, outlier=None, rot_sign=1.0):
        """Observations of the true world (see the module docstring). The epoch tag self.t is the receiver clock
        reading: true reception time t_r = self.t - clk/c, so P = c*(tag - satellite clock reading at emission)
        + group delay = rho + clk + isb - c*dts + I + T + group delay; -lambda*D = range rate + d - c*dts_dot."""
        isb = isb or {"G": 0.0, "E": 12.0, "C": -7.0}
        pos = geodesy.ecef2pos(self.rr)
        E = geodesy.enu_matrix(pos[0], pos[1])
        obs = []
        for e in self.ephs:
            f = sats.FREQ_L1 if e["sys"] != "C" else sats.FREQ_B1I
            t_r = self.t - clk / sats.CLIGHT
            rho, tau = emission_range(e, t_r, self.rr, rot_sign)
            rs, dts = orbits.kepler_pos_clk(e, t_r - tau)
            enu = E @ (rs - self.rr)
            el = math.asin(enu[2] / np.linalg.norm(enu))
            az = math.atan2(enu[0], enu[1]) % (2.0 * math.pi)
            ion = geodesy.ionmodel(self.t, iono, pos, az, el) * (sats.FREQ_L1 / f) ** 2
            trop = geodesy.tropmodel(pos, el)
            psr = rho + clk + isb[e["sys"]] - sats.CLIGHT * dts + ion + trop + group_delay_m(e)
            rate = range_rate_design(e, t_r, self.rr, v, tau, rot_sign)
            _, c0 = orbits.kepler_pos_clk(e, t_r - tau - 0.05)
            _, c1 = orbits.kepler_pos_clk(e, t_r - tau + 0.05)
            dopp = -(rate + d - sats.CLIGHT * (c1 - c0) / 0.1) / (sats.CLIGHT / f)
            if outlier and e["sat"] == outlier[0]:
                psr += outlier[1]
                dopp += outlier[2] / (sats.CLIGHT / f)
            obs.append({"sat": e["sat"], "sys": e["sys"], "prn": e["prn"], "freq": f, "psr": psr, "psr_ok": True,
                        "dopp": dopp, "cn0": 45.0})
        return obs

    def test_spp_recovers_position_clock_isb(self):
        obs = self.make_obs()
        sol = gnsslsq.spp(obs, self.t, self.sel, iono=None)
        self.assertIsNotNone(sol)
        # tolerances: the solver's first-order Sagnac term differs from the rotated-frame range by < 1 mm per satellite
        self.assertLess(np.linalg.norm(sol["rr"] - self.rr), 0.01)
        self.assertAlmostEqual(sol["clk_m"], 3000.0, delta=0.01)
        self.assertAlmostEqual(sol["isb"]["E"], 12.0, delta=0.01)
        self.assertAlmostEqual(sol["isb"]["C"], -7.0, delta=0.01)
        self.assertEqual(sol["excluded"], [])

    def test_spp_discriminates_the_earth_rotation_sign(self):
        # the same world with the rotation reversed moves the solution by metres: the test above is not tautological
        sol = gnsslsq.spp(self.make_obs(rot_sign=-1.0), self.t, self.sel, iono=None, max_reject=0)
        self.assertGreater(np.linalg.norm(sol["rr"] - self.rr), 1.0)

    def most_redundant(self, spp, systems):
        """(redundancy number, sat) of the satellite of `systems` whose residual carries the largest share of its own
        error: r_i = 1 - (W H (H^T W H)^-1 H^T)_ii with the solver's weights. An outlier is only detectable in
        proportion to r_i (a lone satellite of a system has r = 0: its ISB absorbs it), so the tests put it there."""
        pos = geodesy.ecef2pos(self.rr)
        rows, w, ids = [], [], []
        for e in self.ephs:
            rs, _ = orbits.kepler_pos_clk(e, self.t - 0.075)
            _, u = geodesy.geodist(rs, self.rr)
            az, el = geodesy.satazel(pos, u)
            sinel = max(math.sin(el), 0.1)
            if spp:
                f = sats.FREQ_L1 if e["sys"] != "C" else sats.FREQ_B1I
                ion = geodesy.ionmodel(self.t, None, pos, az, el) * (sats.FREQ_L1 / f) ** 2
                rows.append(list(-u) + [1.0, float(e["sys"] == "E"), float(e["sys"] == "C")])
                w.append(1.0 / (0.3 ** 2 + (0.3 / sinel) ** 2 + (0.5 * ion) ** 2 + (0.3 / (sinel + 0.1)) ** 2))
            else:
                rows.append(list(-u) + [1.0])
                w.append(1.0 / (0.05 * (1.0 + 1.0 / sinel) / 2.0) ** 2)
            ids.append((e["sat"], e["sys"]))
        H, W = np.array(rows), np.diag(w)
        r = 1.0 - np.diag(W @ H @ np.linalg.solve(H.T @ W @ H, H.T))
        best = max((float(r[i]), float(np.sqrt(1.0 / w[i])), sat) for i, (sat, sys_) in enumerate(ids) if sys_ in systems)
        return best

    def test_spp_rejects_a_30m_outlier(self):
        r, sigma, bad = self.most_redundant(True, "G")
        self.assertGreater(r * 30.0 / sigma, 8.0)          # expected normalised residual: twice the 4-sigma threshold
        obs = self.make_obs(outlier=(bad, 30.0, 0.0))
        sol = gnsslsq.spp(obs, self.t, self.sel)
        self.assertIn(bad, sol["excluded"])
        self.assertLess(np.linalg.norm(sol["rr"] - self.rr), 0.01)

    def test_doppler_ls_exact_and_sign(self):
        v = np.array([1.2, -0.4, 0.05])
        obs = self.make_obs(v=v, d=45.0)
        sol = gnsslsq.doppler_ls(obs, self.t, self.sel, self.rr)
        # 1e-4 m/s: the solver's +-0.5 s satellite velocity differs from the forward model's by ~1e-5 m/s; the
        # Sagnac-rate term it must reproduce is ~5e-3 m/s per satellite
        np.testing.assert_allclose(sol["v_ecef"], v, atol=1e-4)
        self.assertAlmostEqual(sol["drift_mps"], 45.0, delta=1e-4)
        self.assertLess(sol["rms_mps"], 1e-4)
        flipped = gnsslsq.doppler_ls(obs, self.t, self.sel, self.rr, doppler_sign=-1.0, max_reject=0)
        self.assertGreater(flipped["rms_mps"], 10.0)      # the audit's G11 separates the signs by orders of magnitude

    def test_sagnac_rate_sign_per_satellite(self):
        # the solver's model rate against the independent rotated-frame rate, satellite by satellite (moving receiver)
        v = np.array([12.0, -7.0, 1.5])
        rows = gnsslsq.doppler_rows(self.make_obs(v=v, d=0.0), self.t, self.sel, self.rr, cn0_min=0.0)
        self.assertGreaterEqual(len(rows), 8)
        worst = 0.0
        for sat, e_los, rs, vs, ddts, z, el in rows:
            model = gnsslsq.doppler_model_rate(e_los, rs, vs, self.rr, v)      # the rate solve_doppler fits
            worst = max(worst, abs((z + sats.CLIGHT * ddts) - model))
        self.assertLess(worst, 1e-4)
        # the Jacobian is the derivative of that rate (central difference in the receiver velocity)
        sat, e_los, rs, vs, ddts, z, el = rows[0]
        for k in range(3):
            dv = np.zeros(3)
            dv[k] = 1e-3
            num = (gnsslsq.doppler_model_rate(e_los, rs, vs, self.rr, v + dv) - gnsslsq.doppler_model_rate(e_los, rs, vs, self.rr, v - dv)) / 2e-3
            self.assertAlmostEqual(num, gnsslsq.doppler_model_jacobian(e_los, rs)[k], delta=1e-9)

    def test_doppler_ls_discriminates_the_sagnac_rate(self):
        v = np.array([1.2, -0.4, 0.05])
        sol = gnsslsq.doppler_ls(self.make_obs(v=v, d=45.0, rot_sign=-1.0), self.t, self.sel, self.rr, max_reject=0)
        self.assertGreater(float(np.linalg.norm(sol["v_ecef"] - v)), 1e-3)

    def test_doppler_ls_rejects_outlier_and_tolerates_position_error(self):
        v = np.array([-0.8, 0.3, 0.0])
        r, sigma, bad = self.most_redundant(False, "GEC")
        self.assertGreater(r * 2.0 / sigma, 8.0)
        obs = self.make_obs(v=v, outlier=(bad, 0.0, 2.0))
        sol = gnsslsq.doppler_ls(obs, self.t, self.sel, self.rr + np.array([8.0, -6.0, 5.0]))
        self.assertIn(bad, sol["excluded"])
        np.testing.assert_allclose(sol["v_ecef"], v, atol=0.01)

    def test_weighted_doppler_ls_free_and_fixed_drift(self):
        # the time calibration's two-pass solver: same model as solve_doppler, per-row sigmas, drift free or given
        v = np.array([1.2, -0.4, 0.05])
        rows = gnsslsq.doppler_rows(self.make_obs(v=v, d=45.0), self.t, self.sel, self.rr, cn0_min=0.0)
        sig = [0.1 + 0.05 * k for k in range(len(rows))]
        free = gnsslsq.solve_doppler_weighted(rows, sig, self.rr)
        np.testing.assert_allclose(free["v_ecef"], v, atol=1e-4)
        self.assertAlmostEqual(free["drift_mps"], 45.0, delta=1e-4)
        self.assertEqual(free["excluded"], [])
        fixed = gnsslsq.solve_doppler_weighted(rows, sig, self.rr, drift_mps=45.0)
        np.testing.assert_allclose(fixed["v_ecef"], v, atol=1e-4)
        self.assertEqual(fixed["Q"].shape, (3, 3))
        wrong = gnsslsq.solve_doppler_weighted(rows, sig, self.rr, drift_mps=46.0, max_reject=0)
        self.assertGreater(float(np.linalg.norm(wrong["v_ecef"] - v)), 0.1)     # a given drift is not re-estimated

    def test_fixing_the_drift_removes_the_vertical_clock_collinearity(self):
        # the six highest satellites only (the grass-field geometry: only near-zenith satellites are strong): the
        # vertical velocity column (-e_up) and the drift column (+1) are nearly collinear, so the vertical variance
        # with a free drift is many times the one with the drift given
        rows = gnsslsq.doppler_rows(self.make_obs(v=np.array([0.5, 0.3, 0.0]), d=45.0), self.t, self.sel, self.rr, cn0_min=0.0)
        high = sorted(rows, key=lambda r: -r[6])[:6]
        sig = [0.1] * len(high)
        free = gnsslsq.solve_doppler_weighted(high, sig, self.rr)
        fixed = gnsslsq.solve_doppler_weighted(high, sig, self.rr, drift_mps=45.0)
        pos = geodesy.ecef2pos(self.rr)
        up = geodesy.enu_matrix(pos[0], pos[1])[2]
        ratio = float(up @ free["Q"][:3, :3] @ up) / float(up @ fixed["Q"] @ up)
        self.assertGreater(ratio, 3.0, ratio)

    def test_weighted_doppler_ls_rejects_by_the_row_sigma(self):
        v = np.array([-0.8, 0.3, 0.0])
        r, sigma, bad = self.most_redundant(False, "GEC")
        rows = gnsslsq.doppler_rows(self.make_obs(v=v, outlier=(bad, 0.0, 2.0)), self.t, self.sel, self.rr, cn0_min=0.0)
        sol = gnsslsq.solve_doppler_weighted(rows, [0.05] * len(rows), self.rr)
        self.assertIn(bad, sol["excluded"])
        np.testing.assert_allclose(sol["v_ecef"], v, atol=0.01)
        # the same 2 m/s on a row whose own sigma is 1 m/s is a 2-sigma residual at most: kept
        sol = gnsslsq.solve_doppler_weighted(rows, [1.0 if x[0] == bad else 0.05 for x in rows], self.rr)
        self.assertNotIn(bad, sol["excluded"])


if __name__ == "__main__":
    unittest.main()
