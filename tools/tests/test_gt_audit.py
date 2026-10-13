# SPDX-License-Identifier: BSD-3-Clause
"""tools/gt_audit.py on a synthetic drive with a known answer: GT built as ENU at the first RTK fix (or as UTM) is
identified exactly, a GCJ-02 source is caught by the datum test, an 80 ms GT stamp delay is found by the odometry
chain and compensated (> 20 ms), an antenna ahead of the no-slip point makes the GT course LEAD the gyro by r_x/v
(the lead model recovers r_x and the clock lag without the odometry), an undetermined or ambiguous time base is a
FAIL, and the pre-registered trust mask flags the h_acc window and an RTK stall (> 0.5 s), each widened by +-2 s."""
import csv
import math
import tempfile
import unittest

import numpy as np

import _path  # noqa: F401
import synth
import gt_audit as ga
from fl2g import geodesy, gnsstime, report

LAT0, LON0, H0 = math.radians(34.0319), math.radians(108.7643), 394.0
T0_UTC = 1732537010.0
GT_DELAY = 0.080          # GT stamp = true UTC + 80 ms (host), IMU/odom headers = true UTC
TRANSPORT = 0.180         # host record time - true time, RTK fix and IMU alike (equal transport delays)


def enu_to_geo(e, n, u):
    r0 = geodesy.pos2ecef(LAT0, LON0, H0)
    E = geodesy.enu_matrix(LAT0, LON0)
    out = []
    for a, b, c in zip(e, n, u):
        out.append(geodesy.ecef2pos(r0 + E.T @ np.array([a, b, c])))
    return np.array(out)


class Drive:
    def __init__(self, gcj=False, utm=False, seed=0, pattern="burst", antenna_x=0.0):
        rng = np.random.RandomState(seed)
        self.m = synth.Motion(T0_UTC - 5.0, T0_UTC + 305.0)
        # RTK receiver: 5 Hz fixes, each published 3 times; "burst" = the M3DGR pattern (the copies' stamps within
        # ~20 us, see docs/dev/audit), "republish" = copies spaced at 15 Hz. Stall of 0.8 s at +200 s.
        k15 = np.arange(4500)
        t_upd = T0_UTC + (k15 // 3) * 0.2          # integer arithmetic: 3 rows per fix
        if pattern == "burst":
            t15 = t_upd + np.where(k15 % 3 == 2, -16e-6, 0.0)
        else:
            t15 = T0_UTC + k15 / 15.0
        stall = (t_upd >= T0_UTC + 200.0) & (t_upd < T0_UTC + 200.8)
        t_upd = t_upd.copy()
        t_upd[stall] = T0_UTC + 199.8
        x, y, yaw_u = self.m.pose(t_upd)
        # GT antenna antenna_x ahead of the IMU along the body x axis (the unicycle's no-slip point is the IMU)
        x, y = x + antenna_x * np.cos(yaw_u), y + antenna_x * np.sin(yaw_u)
        geo = enu_to_geo(x, y, np.zeros_like(x))
        lat, lon = np.degrees(geo[:, 0]), np.degrees(geo[:, 1])
        if gcj:
            lat, lon = geodesy.wgs84_to_gcj02(lat, lon)
        self.rtk = [{"stamp": float(t + GT_DELAY), "t_record": float(t + TRANSPORT), "lat": float(a), "lon": float(b), "alt": float(h),
                     "status": 0, "cov": [0.36] + [0.0] * 8, "cov_type": 1} for t, a, b, h in zip(t15, lat, lon, geo[:, 2])]
        # GT file rows as M3DGR builds them (ENU at the first fix of the source lat/lon), or UTM for the negative case
        la, lo, al = np.radians(lat), np.radians(lon), geo[:, 2]
        if utm:
            zone = geodesy.utm_zone(lon[0])
            Ee, Nn = geodesy.tm_forward(lat, lon, -183.0 + 6.0 * zone)
            xyz = np.stack([Ee - Ee[0], Nn - Nn[0], al - al[0]], axis=1)
        else:
            xyz = geodesy.geodetic_to_enu(la, lo, al, la[0], lo[0], al[0])
        self.gt = np.column_stack([t15 + GT_DELAY, np.round(xyz, 6), np.zeros((t15.size, 3)), np.ones(t15.size)])
        # 9Ps PVT at 10 Hz (true instants, GPST tags), 1 m noise; h_acc 2 m in [100, 110] s
        tp = T0_UTC + np.arange(0.0, 300.0, 0.1)
        xp, yp, _ = self.m.pose(tp)
        gp = enu_to_geo(xp + rng.normal(0, 0.7, tp.size), yp + rng.normal(0, 0.7, tp.size), rng.normal(0, 1.0, tp.size))
        sp = self.m.antenna_speed(tp, lever=np.zeros(3))
        yaw = np.interp(tp, self.m.t, self.m.yaw)
        self.pvt = []
        for k, t in enumerate(tp):
            w, tow = gnsstime.week_tow(gnsstime.utc_to_gpst(t))
            hacc = 2.0 if 100.0 <= t - T0_UTC <= 110.0 else 0.8
            self.pvt.append({"t_gpst": gnsstime.gpst(w, tow), "t_record": t + 0.05, "fix_type": 3, "lat": math.degrees(gp[k, 0]),
                             "lon": math.degrees(gp[k, 1]), "h_ell": gp[k, 2], "h_acc": hacc, "v_acc": 1.5, "num_sv": 20,
                             "vel_n": sp[k] * math.sin(yaw[k]), "vel_e": sp[k] * math.cos(yaw[k]), "vel_d": 0.0})
        # Avia IMU 200 Hz: gyro z = yaw rate (+ noise), accel = +1 g up (Livox units)
        ti = T0_UTC + np.arange(0.0, 300.0, 0.005)
        wz = np.interp(ti, self.m.t, self.m.w) + rng.normal(0, 0.01, ti.size)
        self.imu = np.column_stack([ti, np.zeros_like(ti), np.zeros_like(ti), wz, np.zeros_like(ti), np.zeros_like(ti), np.ones_like(ti),
                                    ti + TRANSPORT])
        to = T0_UTC + np.arange(0.0, 300.0, 0.05)
        self.odom = np.column_stack([to, np.interp(to, self.m.t, self.m.v), np.zeros_like(to), np.zeros_like(to), np.interp(to, self.m.t, self.m.w), to])


class GtAudit(unittest.TestCase):
    def run_all(self, drive):
        checks, rep = report.Checks(), {}
        gt, _, _ = ga.clean_gt(drive.gt)
        origin = ga.provenance(gt, drive.rtk, rep, checks)
        with tempfile.TemporaryDirectory() as d:
            comp, lag_utc = ga.time_base(gt, drive.imu, drive.odom, drive.pvt, rep, checks, False, d, rtk=drive.rtk)
            ga.datum_and_alignment(gt, drive.rtk, drive.pvt, origin, rep, checks, lag_utc)
            trusted = ga.trust_mask(gt, drive.pvt, lag_utc, rep, checks, d)
        return {c["id"]: c for c in checks.items}, rep, comp, lag_utc, trusted, gt

    def test_enu_provenance_time_base_mask(self):
        dr = Drive()
        c, rep, comp, lag_utc, trusted, gt = self.run_all(dr)
        self.assertEqual(c["P1"]["verdict"], "PASS", c["P1"]["evidence"])
        self.assertTrue(rep["provenance"]["best"].startswith("ENU"))
        self.assertLess(rep["provenance"]["hypotheses"][0]["rms_direct_m"], 1e-5)
        self.assertEqual(rep["provenance"]["gt_stamp_equals_rtk_header_1us"], 1.0)
        self.assertEqual(c["P2"]["verdict"], "PASS", c["P2"]["evidence"])
        self.assertEqual(c["P3"]["verdict"], "PASS", c["P3"]["evidence"])
        self.assertLess(abs(rep["alignment"]["fits"]["SE2"]["yaw_deg"]), 3.0 * rep["alignment"]["yaw_sd_deg_block_bootstrap"] + 0.05)
        self.assertLess(abs(rep["alignment"]["fits"]["Sim2"]["scale"] - 1.0), 0.01)
        # GT stamps are 80 ms late: GT(t) matches IMU(t - 80 ms) -> compensation -80 ms applied
        self.assertLess(abs(comp - (-GT_DELAY)), 0.005, rep["time_base"])
        self.assertLess(abs(rep["time_base"]["xcorr"]["gt_speed__odom_speed"]["lag_s"] + GT_DELAY), 0.01)
        self.assertLess(abs(rep["time_base"]["xcorr"]["odom_yaw_rate__imu_yaw_rate"]["lag_s"]), 0.005)
        self.assertTrue(rep["time_base"]["method"].startswith("chain"))
        self.assertLess(abs(lag_utc - GT_DELAY), 0.02)
        self.assertEqual(c["P4"]["verdict"], "WARN")          # WARN = compensation applied (> 20 ms)
        self.assertLess(abs(rep["time_base"]["record_time_based_lag_s"] + GT_DELAY), 0.005)
        # trust mask (TEST_PLAN §4): receiver quality (h_acc > 1 m in [100, 110] s) and GT defects (the 0.8 s RTK
        # stall after 199.8 s) both mask +-2 s around them
        t = gt[:, 0] - GT_DELAY - T0_UTC
        self.assertFalse(np.any(trusted[(t > 98.3) & (t < 111.7)]))
        self.assertTrue(np.all(trusted[(t > 20.0) & (t < 97.7)]))
        self.assertTrue(np.all(trusted[(t > 112.3) & (t < 197.0)]))
        self.assertAlmostEqual(rep["trust_mask"]["frac_receiver_criteria_unwidened"],
                               np.mean((t >= 100.0) & (t <= 110.0)), delta=0.002)
        self.assertFalse(np.any(trusted[(t > 198.0) & (t < 202.5)]))
        self.assertTrue(np.all(trusted[(t > 204.0) & (t < 290.0)]))
        self.assertEqual(rep["trust_mask"]["n_stalls"], 1)
        # the only speed "spikes" of the noise-free drive are the stall's freeze/catch-up at 199.8-201.0 s
        t0 = gt[0, 0] - GT_DELAY - T0_UTC
        for ts in rep["trust_mask"]["spike_times_rel"]:
            self.assertTrue(199.0 < ts + t0 < 202.0, ts)

    def test_republish_pattern_mask_and_time_base(self):
        dr = Drive(pattern="republish", seed=2)
        c, rep, comp, lag_utc, trusted, gt = self.run_all(dr)
        self.assertEqual(rep["trust_mask"]["n_stalls"], 1)
        t0 = gt[0, 0] - GT_DELAY - T0_UTC
        self.assertTrue(all(199.0 < ts + t0 < 202.0 for ts in rep["trust_mask"]["spike_times_rel"]), rep["trust_mask"]["spike_times_rel"])
        # republished copies lag the fix by 0/67/133 ms but the first copy carries the fix time: chain lag stays -80 ms
        self.assertLess(abs(comp - (-GT_DELAY)), 0.01, rep["time_base"])

    def test_without_odometry_the_time_base_is_undetermined(self):
        # no chain: FAIL and no compensation value at all (main() then withholds gt_audited.tum)
        dr = Drive(seed=4)
        dr.odom = np.zeros((0, 6))
        c, rep, comp, lag_utc, trusted, gt = self.run_all(dr)
        self.assertIsNone(comp)
        self.assertIsNone(rep["time_base"]["gt_to_imu_lag_s"])
        self.assertEqual(c["P4"]["verdict"], "FAIL", c["P4"]["evidence"])

    def test_chain_and_record_time_disagreement_is_a_fail(self):
        dr = Drive(seed=5)
        dr.imu[:, 7] -= 0.100                       # IMU now 100 ms faster to the host than the RTK fix
        c, rep, comp, lag_utc, trusted, gt = self.run_all(dr)
        self.assertLess(abs(rep["time_base"]["gt_to_imu_lag_s"] + GT_DELAY), 0.005)
        self.assertIsNone(comp)
        self.assertEqual(c["P4"]["verdict"], "FAIL", c["P4"]["evidence"])

    def test_antenna_ahead_makes_the_course_lead(self):
        # an antenna 0.3 m ahead of the no-slip point: the GT course leads the heading by r_x / v, so the raw direct
        # link sits AFTER the chain (positive difference); the lead model recovers r_x and the clock lag
        r_x = 0.3
        dr = Drive(seed=6, antenna_x=r_x)
        c, rep, comp, lag_utc, trusted, gt = self.run_all(dr)
        tb = rep["time_base"]
        self.assertLess(abs(comp + GT_DELAY), 0.005, tb)                 # the chain is not affected by the lead
        lead = tb["xcorr"]["gt_course_rate__imu_yaw_rate"]["lag_s"] - tb["gt_to_imu_lag_s"]
        lv = tb["lever_lead_fit"]
        self.assertGreater(lead, 0.5 * r_x * lv["inv_speed_mean"], (lead, lv))
        self.assertLess(lead, 1.5 * r_x * lv["inv_speed_mean"], (lead, lv))
        self.assertLess(abs(lv["r_x_m"] - r_x), 0.05, lv)
        self.assertLess(abs(lv["lag_s"] + GT_DELAY), 0.015, lv)
        self.assertGreater(lv["lag_no_lever_s"] - lv["lag_s"], 0.1, lv)   # ignoring the lever biases the lag late
        # no lever: the fit finds r_x ~ 0
        c0, rep0, comp0, _, _, _ = self.run_all(Drive(seed=6))
        self.assertLess(abs(rep0["time_base"]["lever_lead_fit"]["r_x_m"]), 0.03, rep0["time_base"]["lever_lead_fit"])

    def test_gcj02_source_is_caught(self):
        dr = Drive(gcj=True)
        c, rep, comp, lag_utc, trusted, gt = self.run_all(dr)
        self.assertEqual(c["P2"]["verdict"], "FAIL", c["P2"]["evidence"])
        self.assertGreater(rep["provenance"]["gcj02_absolute_shift_here_m"], 100.0)

    def test_utm_provenance_is_identified(self):
        dr = Drive(utm=True)
        c, rep, comp, lag_utc, trusted, gt = self.run_all(dr)
        self.assertTrue(rep["provenance"]["best"].startswith("UTM"), rep["provenance"]["hypotheses"])
        enu = [h for h in rep["provenance"]["hypotheses"] if h["hypothesis"].startswith("ENU at")][0]
        self.assertGreater(abs(enu["sim3_yaw_deg"]), 0.5)       # grid convergence ~1.3 deg at 108.8 E in zone 49
        self.assertGreater(enu["rms_direct_m"], 1.0)

    def test_replay_route_when_the_odometry_yaw_link_is_weak(self):
        # the dev-set failure mode (Grass02/Occlusion03/04: odometry yaw rate vs gyro peak ~0.7 < 0.8): P4 stays FAIL by
        # the pre-registered rule, the sub-threshold candidate is reported with its precision, and P4c recovers the lag
        # without the odometry (GT speed vs the replay speed, the estimator clock being the IMU header clock = UTC here)
        dr = Drive(seed=7)
        rng = np.random.RandomState(7)
        dr.odom[:, 4] += rng.normal(0.0, np.std(dr.odom[:, 4]), dr.odom.shape[0])        # yaw-rate peak ~0.7
        tt = np.arange(T0_UTC, T0_UTC + 300.0, 0.04)                                        # replay steps (estimator = UTC)
        vv = np.interp(tt, dr.m.t, dr.m.v)
        yaw = np.interp(tt, dr.m.t, dr.m.yaw)
        traj = {"t": tt, "v": np.column_stack([vv * np.cos(yaw), vv * np.sin(yaw), np.zeros_like(tt)])}
        checks, rep = report.Checks(), {}
        gt, _, _ = ga.clean_gt(dr.gt)
        with tempfile.TemporaryDirectory() as d:
            comp, _ = ga.time_base(gt, dr.imu, dr.odom, dr.pvt, rep, checks, False, d, rtk=dr.rtk, traj=traj)
        c = {x["id"]: x for x in checks.items}
        tb = rep["time_base"]
        self.assertLess(tb["xcorr"]["odom_yaw_rate__imu_yaw_rate"]["peak"], ga.MIN_PEAK)
        self.assertEqual(c["P4"]["verdict"], "FAIL", c["P4"]["evidence"])
        self.assertIsNone(comp)
        cand = tb["candidate_chain"]
        self.assertLess(abs(cand["lag_s"] + GT_DELAY), 3.0 * cand["sigma_s"], cand)       # honest about its precision
        self.assertIn("sub-threshold", c["P4"]["evidence"])
        self.assertEqual(c["P4c"]["verdict"], "INFO")
        self.assertNotIn("P4b", c)
        self.assertLess(abs(tb["replay_chain"]["lag_s"] + GT_DELAY), 0.01, tb["replay_chain"])
        for k, v in tb["xcorr"].items():                      # every link carries its delay precision; the clean ones
            self.assertTrue(np.isfinite(v["sigma_s"]), (k, v))   # are sharp, the degraded yaw link is not
            if v["peak"] > 0.99:
                self.assertLess(v["sigma_s"], 0.01, (k, v))
        self.assertGreater(tb["xcorr"]["odom_yaw_rate__imu_yaw_rate"]["sigma_s"], 0.02)

    def test_rows_outside_the_bag_are_never_trusted(self):
        dr = Drive(seed=8)
        gt, _, _ = ga.clean_gt(dr.gt)
        span = (T0_UTC, T0_UTC + 150.0)            # a truncated bag: GT continues 150 s past its end (Occlusion03-like)
        checks, rep = report.Checks(), {}
        with tempfile.TemporaryDirectory() as d:
            trusted = ga.trust_mask(gt, [p for p in dr.pvt if gnsstime.gpst_to_utc(p["t_gpst"]) < span[1]], GT_DELAY, rep, checks, d, bag_span=span)
        outside = gt[:, 0] > span[1] + 1.0
        self.assertFalse(np.any(trusted[outside]))
        self.assertEqual(rep["trust_mask"]["rows_outside_bag"], int(outside.sum()))
        inside = ~outside
        self.assertAlmostEqual(rep["trust_mask"]["trusted_frac"], float(np.mean(trusted[inside])), places=12)
        self.assertIn("inside the bag's record span", [c for c in checks.items if c["id"] == "P5"][0]["evidence"])

    def test_timebase_rule_on_the_drive(self):
        # AMENDMENTS A54 on real time_base() output: the chain when P4 passes; the record-time value when it fails
        dr = Drive(seed=9)
        c, rep, comp, lag_utc, trusted, gt = self.run_all(dr)
        tb = ga.decide_timebase(rep["time_base"], c["P4"]["verdict"])
        self.assertEqual((tb["method"], tb["confidence"]), ("chain", "high"))
        self.assertAlmostEqual(tb["applied_shift_s"], comp, places=12)
        dr.odom = np.zeros((0, 6))                               # no chain: P4 FAIL, the record-time value is used
        c, rep, comp, lag_utc, trusted, gt = self.run_all(dr)
        self.assertIsNone(comp)
        tb = ga.decide_timebase(rep["time_base"], c["P4"]["verdict"])
        self.assertEqual((tb["method"], tb["confidence"], tb["p4_verdict"]), ("record_time", "low", "FAIL"))
        self.assertLess(abs(tb["applied_shift_s"] + GT_DELAY), 0.005)
        self.assertEqual(tb["uncertainty_s"], ga.AGREE_S)
        self.assertIn("A54", c["P4"]["evidence"])

    def test_clean_gt_orders_and_dedups(self):
        a = np.array([[1.0, 0, 0, 0, 0, 0, 0, 1], [2.0, 1, 0, 0, 0, 0, 0, 1], [2.0, 1, 0, 0, 0, 0, 0, 1],
                      [1.9999, 1, 0, 0, 0, 0, 0, 1], [3.0, 2, 0, 0, 0, 0, 0, 1]])
        b, reordered, dup = ga.clean_gt(a)
        self.assertEqual(list(b[:, 0]), [1.0, 1.9999, 2.0, 3.0])
        self.assertEqual(dup, 1)
        self.assertGreater(reordered, 0)


if __name__ == "__main__":
    unittest.main()


class TimebaseRule(unittest.TestCase):
    """AMENDMENTS A54: P4 pass -> chain; P4 fail -> record-time value; neither -> 0 (flagged); > 20 ms compensated."""

    def tb(self, chain=None, rec=None, cand=None):
        return {"gt_to_imu_lag_s": chain, "record_time_based_lag_s": rec,
                "candidate_chain": None if cand is None else {"lag_s": cand, "sigma_s": 0.03, "peaks": [0.95, 0.7]},
                "xcorr": {"gt_speed__odom_speed": {"sigma_s": 0.012}, "odom_yaw_rate__imu_yaw_rate": {"sigma_s": 0.005}}}

    def test_chain(self):
        d = ga.decide_timebase(self.tb(chain=0.1071, rec=0.1097), "WARN")
        self.assertEqual((d["method"], d["confidence"], d["applied_shift_s"], d["flag"]), ("chain", "high", 0.1071, ""))
        self.assertAlmostEqual(d["uncertainty_s"], math.hypot(0.012, 0.005))
        d = ga.decide_timebase(self.tb(chain=0.0146, rec=0.0046), "PASS")         # Grass01: <= 20 ms, not shifted
        self.assertEqual((d["method"], d["offset_s"], d["applied_shift_s"]), ("chain", 0.0146, 0.0))

    def test_record_time_fallback(self):
        # Occlusion03-like: sub-threshold chain -122.3 ms, record time -111.7 ms (agree within 50 ms): medium
        d = ga.decide_timebase(self.tb(rec=-0.1117, cand=-0.1223), "FAIL")
        self.assertEqual((d["method"], d["confidence"], d["applied_shift_s"]), ("record_time", "medium", -0.1117))
        self.assertIn("P4 FAIL", d["flag"])
        # Grass02-like: sub-threshold chain +78.9 ms against record time -5.9 ms: low, and -5.9 ms is not shifted
        d = ga.decide_timebase(self.tb(rec=-0.0059, cand=0.0789), "FAIL")
        self.assertEqual((d["method"], d["confidence"], d["applied_shift_s"]), ("record_time", "low", 0.0))
        # a valid chain that contradicts the record time (ambiguous, P4 FAIL): record time, low
        d = ga.decide_timebase(self.tb(chain=0.020, rec=0.110, cand=0.020), "FAIL")
        self.assertEqual((d["method"], d["confidence"]), ("record_time", "low"))

    def test_nothing(self):
        d = ga.decide_timebase(self.tb(), "FAIL")
        self.assertEqual((d["method"], d["offset_s"], d["applied_shift_s"], d["confidence"]), ("none", 0.0, 0.0, "none"))
        self.assertTrue(d["flag"])

    def test_yaml(self):
        import yaml
        d = ga.decide_timebase(self.tb(rec=-0.1117, cand=-0.1223), "FAIL")
        with tempfile.TemporaryDirectory() as tmp:
            p = tmp + "/gt_timebase.yaml"
            ga.write_timebase_yaml(p, "Occlusion03", d)
            with open(p) as f:
                back = yaml.safe_load(f)
        self.assertEqual(back["sequence"], "Occlusion03")
        self.assertEqual((back["method"], back["applied_shift_s"], back["chain_s"]), ("record_time", -0.1117, None))
        self.assertEqual(back["sub_threshold_chain_s"], -0.1223)


class RtkJumps(unittest.TestCase):
    """AMENDMENTS A54: RTK jumps / spikes on a 5 Hz fix track; a jump and its return = displaced plateau."""

    def track(self, seed=0, noise_h=0.01, noise_v=0.02, t_len=300.0):
        rng = np.random.RandomState(seed)
        m = synth.Motion(T0_UTC, T0_UTC + t_len + 1.0)
        t = T0_UTC + np.arange(0.0, t_len, 0.2)
        x, y, _ = m.pose(t)
        p = np.column_stack([x, y, 0.02 * np.sin(2 * np.pi * (t - T0_UTC) / 40.0)])
        p += np.column_stack([rng.normal(0, noise_h, t.size), rng.normal(0, noise_h, t.size), rng.normal(0, noise_v, t.size)])
        return t, p

    def test_plateau_and_spike(self):
        t, p = self.track()
        rel = t - T0_UTC
        p[(rel >= 100.0) & (rel < 110.0)] += np.array([1.0, -2.0, -5.0])      # Grass01-like mode change, 10 s
        k = int(np.argmin(np.abs(rel - 200.0)))
        p[k] += np.array([0.8, 0.0, 0.9])                                        # one bad fix
        rj = ga.rtk_jumps(t, p)
        self.assertEqual(len(rj["jumps"]), 4, [(round(j["t"] - T0_UTC, 1), round(j["size_m"], 2)) for j in rj["jumps"]])
        pl, un = ga.rtk_levels(rj["jumps"], (t[0], t[-1]))
        self.assertEqual(un, [])
        self.assertEqual(len(pl), 2)
        self.assertAlmostEqual(pl[0]["t_from"] - T0_UTC, 99.8, delta=0.01)
        self.assertAlmostEqual(pl[0]["t_to"] - T0_UTC, 110.0, delta=0.01)
        self.assertAlmostEqual(pl[1]["t_to"] - pl[1]["t_from"], 0.4, delta=0.01)
        self.assertLess(pl[0]["residual_m"], 0.1)

    def test_noise_and_real_motion_are_not_jumps(self):
        # grass-like GT noise (3 cm) on the synthetic drive (speed 0.6-2 m/s, turns): no jump
        t, p = self.track(seed=3, noise_h=0.03, noise_v=0.03)
        self.assertEqual(ga.rtk_jumps(t, p)["jumps"], [])

    def test_unpaired_jump_masks_the_shorter_side(self):
        t, p = self.track(seed=4)
        p[(t - T0_UTC) >= 240.0] += np.array([0.0, 0.0, 1.5])
        rj = ga.rtk_jumps(t, p)
        pl, un = ga.rtk_levels(rj["jumps"], (t[0], t[-1]))
        self.assertEqual((len(pl), len(un)), (0, 1))
        self.assertEqual(un[0]["masked_side"], "after")
        self.assertAlmostEqual(un[0]["duration_s"], t[-1] - un[0]["jump"]["t"], delta=1e-9)
        # the same jump outside the span counts nothing inside it
        pl, un = ga.rtk_levels(rj["jumps"], (t[0], T0_UTC + 150.0))
        self.assertEqual(un[0]["duration_s"], 0.0)

    def test_trust_mask_bit_64_intervals_and_durations(self):
        dr = Drive(seed=11)
        gt, _, _ = ga.clean_gt(dr.gt)
        rel = gt[:, 0] - GT_DELAY - T0_UTC
        gt[(rel >= 150.0) & (rel < 158.0), 1:4] += np.array([0.5, 1.5, -3.0])
        checks, rep = report.Checks(), {}
        with tempfile.TemporaryDirectory() as d:
            trusted = ga.trust_mask(gt, dr.pvt, GT_DELAY, rep, checks, d, audited_shift_s=-0.08)
            with open(d + "/gt_mask_intervals.csv") as f:
                rows = list(csv.DictReader(f))
        tm = rep["trust_mask"]
        self.assertEqual(len(tm["rtk"]["plateaus"]), 1)
        inside = (rel > 148.3) & (rel < 159.7)
        self.assertFalse(np.any(trusted[inside]))
        # 8 s plateau (+ the two fix intervals around it) + 2 x 2 s
        self.assertAlmostEqual(tm["masked_s"]["rtk_jump_plateau"], 12.2, delta=0.3)
        self.assertGreaterEqual(tm["masked_s"]["ate_exclude"], tm["masked_s"]["rtk_jump_plateau"])
        ate = [r for r in rows if r["ate_exclude"] == "1" and "rtk_jump_plateau" in r["reasons"]]
        self.assertEqual(len(ate), 1)
        # the ATE rows carry only GT-defect bits: the h_acc window (100-110 s) is a separate, reported-only interval
        self.assertTrue(all(int(r["reason_bits"]) & ~ga.ATE_EXCLUDE_BITS == 0 for r in rows if r["ate_exclude"] == "1"))
        self.assertTrue(any("h_acc>1m" in r["reasons"] and r["ate_exclude"] == "0" for r in rows))
        # from the last undisplaced row before the jump (<= 0.2 s before +150 s) minus the 2 s widening
        self.assertTrue(150.0 - 0.2 - ga.MASK_WIDEN_S - 1e-3 <= float(ate[0]["t_from_gt"]) - GT_DELAY - T0_UTC <= 150.0 - ga.MASK_WIDEN_S + 1e-3)
        self.assertAlmostEqual(float(ate[0]["t_from_audited"]) - float(ate[0]["t_from_gt"]), -0.08, places=6)
        p7 = [c for c in checks.items if c["id"] == "P7"][0]
        self.assertEqual(p7["verdict"], "INFO")
        self.assertIn("plateau", p7["evidence"])
