# SPDX-License-Identifier: BSD-3-Clause
"""lever_floor.py (AMENDMENTS A75 section 3, A76): the e1o floor of a host IMU point from the GT track alone. Analytic
cases (a straight line has no floor, a uniform circle has |sqrt((R+b)^2 + a^2) - R|), the held heading at standstill,
the level-frame lever vector of both host LiDARs, the A2 guard, and the GT path through gtbase (time base, ATE mask)."""
import contextlib
import io
import json
import math
import os
import shutil
import tempfile
import unittest

import numpy as np

import _path  # noqa: F401
import evalcommon as ec
import lever_floor as lf
from test_gtbase import Audit


def circle(radius=10.0, n=1500, laps=2, hz=15.0):
    """A whole number of laps, uniformly sampled: the case with the closed-form floor |sqrt((R+b)^2 + a^2) - R|."""
    t = np.arange(n) / hz
    th = 2.0 * math.pi * laps * np.arange(n) / n
    return t, np.stack([radius * np.cos(th), radius * np.sin(th), np.zeros(n)], axis=1)


class Floor(unittest.TestCase):
    def test_a_nearly_straight_track_has_a_negligible_floor(self):
        t = np.arange(900) / 15.0
        p = np.stack([1.5 * t, 0.3 * np.sin(0.05 * t), 0.01 * t], axis=1)       # a gentle wiggle: heading within +-0.01 rad
        r = lf.floor_of_track(t, p, np.ones(len(t), bool), np.array([-0.2, 0.1, 0.4]))
        self.assertLess(r["floor_m"], 3e-3)

    def test_uniform_circle_matches_the_analytic_value(self):
        t, p = circle()
        for v in ((-0.176, -0.016, 0.401), (-0.2124, -0.1109, -0.0297), (0.0, 0.3, 0.0)):
            a, b = v[0], v[1]
            want = abs(math.hypot(10.0 + b, a) - 10.0)
            for kw in ({"k": 2}, {"win_s": 1.0}):
                r = lf.floor_of_track(t, p, np.ones(len(t), bool), np.array(v), **kw)
                self.assertAlmostEqual(r["floor_m"], want, delta=2e-4, msg=(v, kw))
        self.assertAlmostEqual(r["yaw_range_deg"], 720.0, delta=5.0)                # two laps

    def test_the_vertical_part_of_the_lever_does_not_matter(self):
        t, p = circle()
        keep = np.ones(len(t), bool)
        a = lf.floor_of_track(t, p, keep, np.array([-0.2, 0.1, 0.0]))["floor_m"]
        b = lf.floor_of_track(t, p, keep, np.array([-0.2, 0.1, 0.9]))["floor_m"]
        self.assertAlmostEqual(a, b, places=9)

    def test_heading_is_held_at_standstill(self):
        t = np.arange(400) / 15.0
        x = np.where(t < 8.0, 0.0, 1.5 * (t - 8.0))          # 8 s at rest, then driving along +y
        p = np.stack([np.zeros_like(t) + 0 * x, x, np.zeros_like(t)], axis=1)
        yaw, ok = lf.heading_from_track(t, p)
        self.assertFalse(ok[:100].any())
        self.assertTrue(ok[-50:].all())
        self.assertTrue(np.allclose(yaw, math.pi / 2, atol=1e-9))                      # leading rows take the first valid heading
        with self.assertRaises(ValueError):
            lf.heading_from_track(t, np.zeros_like(p))

    def test_perfect_track_is_the_antenna_minus_the_rotated_lever(self):
        p = np.zeros((3, 3))
        yaw = np.array([0.0, math.pi / 2, math.pi])
        q = lf.perfect_imu_track(p, yaw, np.array([-0.2, 0.1, 0.4]))
        np.testing.assert_allclose(q, [[0.2, -0.1, -0.4], [0.1, 0.2, -0.4], [-0.2, 0.1, -0.4]], atol=1e-12)


class LevelVector(unittest.TestCase):
    def test_avia_is_the_a58_lever_and_mid360_the_a77_fit_seen_from_the_mid360_imu(self):
        a = ec.lidar_profile("avia")
        np.testing.assert_allclose(lf.level_vector(a), [-0.176, -0.016, 0.401], atol=1e-15)
        m = ec.lidar_profile("mid360")
        v = lf.level_vector(m)
        R = np.asarray(m["R_level_from_imu"]).reshape(3, 3)
        self.assertLess(np.abs(R @ R.T - np.eye(3)).max(), 2e-6)
        self.assertAlmostEqual(np.linalg.det(R), 1.0, places=5)
        self.assertAlmostEqual(np.linalg.norm(v), np.linalg.norm(m["gt_lever_imu"]), places=6)       # a rotation
        np.testing.assert_allclose(v[:2], [-0.049, -0.152], atol=2e-6)                  # A77: the fitted level-frame components
        np.testing.assert_allclose(v[2], 0.401 - 0.4306717794, atol=2e-6)                # ... and the chain's vertical
        self.assertAlmostEqual(math.hypot(v[0], v[1]), 0.1597, places=4)
        self.assertAlmostEqual(math.degrees(math.acos((np.trace(R) - 1) / 2)), 31.81, places=2)       # the 31.8 degree tilt
        np.testing.assert_allclose(lf.level_vector(m, lever=[0, 0, 0]), 0.0, atol=1e-15)


    def test_a80_platforms(self):
        """SmartPNT mini: the I300's x down / y right / z back to the level frame (x forward, y left, z up); the antenna lever of the
        dataset's ground-truth files, (0, -0.120, +0.118) in the carrier frame x right / y forward / z up."""
        s = ec.lidar_profile("smartpnt_mini")
        R = np.asarray(s["R_level_from_imu"]).reshape(3, 3)
        np.testing.assert_allclose(R @ R.T, np.eye(3), atol=1e-12)
        self.assertAlmostEqual(np.linalg.det(R), 1.0)
        np.testing.assert_allclose(lf.level_vector(s), [-0.120, 0.0, 0.118], atol=1e-12)         # 12 cm behind, 11.8 cm above the IMU
        np.testing.assert_allclose(R @ np.array([0, 0, -1.0]), [1, 0, 0], atol=1e-15)             # the I300's -z is forward
        m = ec.lidar_profile("mars_avia")
        np.testing.assert_allclose(lf.level_vector(m), 0.0, atol=1e-15)


class FromFiles(unittest.TestCase):
    def test_sequence_floor_uses_the_time_base_and_the_ate_mask(self):
        au = Audit()
        try:
            n = len(au.clean)
            th = np.linspace(0.0, 2.0 * math.pi, n)
            au.clean[:, 1], au.clean[:, 2], au.clean[:, 3] = 10 * np.cos(th), 10 * np.sin(th), 0.0
            ec.write_tum(os.path.join(au.d, "gt_clean.tum"), au.clean, digits=6)
            au.clean = ec.read_tum(os.path.join(au.d, "gt_clean.tum"))
            au.timebase(sequence="Grass02", method="none", applied_shift_s=0.0)
            excl = np.zeros(n, bool)
            excl[40:60] = True
            au.mask(excl)
            v = np.array([-0.2, 0.1, 0.4])
            r = lf.sequence_floor(au.root, "Grass02", {"gt_kind": "rtk"}, v)
            self.assertEqual((r["rows"], r["masked_rows"], r["gt_rows"]), (n - 20, 20, n))
            self.assertAlmostEqual(r["floor_m"], abs(math.hypot(10.0 + 0.1, 0.2) - 10.0), delta=5e-3)
            self.assertEqual(len(r["gt_clean_sha256"]), 64)
            self.assertLessEqual(r["rule_min_m"], r["floor_m"])
            self.assertGreaterEqual(r["rule_max_m"], r["floor_m"])
        finally:
            au.close()

    def test_cli_guards_the_evaluation_set_and_writes_the_json(self):
        buf = io.StringIO()
        with contextlib.redirect_stderr(buf):
            self.assertEqual(lf.main(["--seqs", "Outdoor01", "--root", "/nonexistent"]), 3)
        self.assertIn("evaluation-set", buf.getvalue())
        buf = io.StringIO()
        with contextlib.redirect_stderr(buf):
            self.assertEqual(lf.main(["--seqs", "NoSuchSeq", "--root", "/nonexistent"]), 3)
        au = Audit()
        out = os.path.join(au.root, "floor.json")
        try:
            n = len(au.clean)
            th = np.linspace(0.0, 2.0 * math.pi, n)
            au.clean[:, 1], au.clean[:, 2], au.clean[:, 3] = 10 * np.cos(th), 10 * np.sin(th), 0.0
            ec.write_tum(os.path.join(au.d, "gt_clean.tum"), au.clean, digits=6)
            au.timebase(sequence="Grass02", method="none", applied_shift_s=0.0)
            au.mask(np.zeros(n, bool))
            with contextlib.redirect_stdout(io.StringIO()) as so:
                self.assertEqual(lf.main(["--seqs", "Grass02", "--root", au.root, "--lidar", "mid360", "--out", out]), 0)
            self.assertIn("lidar mid360", so.getvalue())
            with open(out) as f:
                d = json.load(f)
            self.assertEqual(d["lidar"], "mid360")
            self.assertEqual(d["sequences"][0]["seq"], "Grass02")
            self.assertGreater(d["sequences"][0]["floor_m"], 0.05)
        finally:
            au.close()


if __name__ == "__main__":
    unittest.main()
