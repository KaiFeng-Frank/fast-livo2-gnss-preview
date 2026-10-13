# SPDX-License-Identifier: BSD-3-Clause
"""imu_gt_check: the pure parts (gyro integral on the ground-truth up axis against the ground-truth heading, lag scan, static
window, local gravity) on synthetic signals with a known answer."""
import math
import unittest

import numpy as np

import _path  # noqa: F401
from smartpnt import imu_gt_check as chk

FS = 200.0


def rotations(psi):
    """I300 body (x down, y right, z back) -> ENU for a level platform with heading psi (clockwise from north)."""
    n = len(psi)
    R = np.zeros((n, 3, 3))
    s, c = np.sin(psi), np.cos(psi)
    R[:, :, 0] = np.stack([np.zeros(n), np.zeros(n), -np.ones(n)], axis=1)       # x_body = down
    R[:, :, 1] = np.stack([c, -s, np.zeros(n)], axis=1)                          # y_body = right
    R[:, :, 2] = np.stack([-s, -c, np.zeros(n)], axis=1)                         # z_body = back
    return R


def synthetic(shift=0.0, scale=1.0, seconds=600):
    """IMU stamps, gyro, ground-truth epoch times and attitudes. The true yaw rate (CCW positive) is a sum of two sinusoids; the
    ground-truth epoch labelled k is the heading at true time k + shift; the gyro reads `scale` times the true rate."""
    t = np.arange(0.0, seconds, 1.0 / FS)
    w = math.radians(12) * np.sin(2 * np.pi * t / 23.0) + math.radians(6) * np.sin(2 * np.pi * t / 7.0 + 1.0)
    psi = -np.concatenate([[0.0], np.cumsum(0.5 * (w[1:] + w[:-1]) / FS)])
    gt_t = np.arange(10.0, seconds - 10.0, 1.0)
    R_gt = rotations(np.interp(gt_t + shift, t, psi))
    R_imu = rotations(psi)
    g = np.einsum("nji,j->ni", R_imu, np.array([0.0, 0.0, 1.0])) * (scale * w)[:, None]
    return t, g, gt_t, R_gt


class HeadingCheck(unittest.TestCase):
    def test_consistent_signals(self):
        t, g, gt_t, R = synthetic()
        r = chk.heading_check(t, g, gt_t, R)
        self.assertAlmostEqual(r["heading_10s"]["slope_turning"], 1.0, delta=0.01)
        self.assertLess(r["heading_10s"]["residual_std_deg"], 0.2)
        self.assertAlmostEqual(r["lag_scan_1s"]["tau_star_s"], 0.0, delta=0.02)

    def test_gyro_scale_error_shows_in_the_slope(self):
        t, g, gt_t, R = synthetic(scale=1.03)
        r = chk.heading_check(t, g, gt_t, R)
        self.assertAlmostEqual(r["heading_10s"]["slope_turning"], 1 / 1.03, delta=0.005)

    def test_lag_sign_and_size(self):
        for shift in (0.15, -0.2):
            t, g, gt_t, R = synthetic(shift=shift)
            r = chk.heading_check(t, g, gt_t, R)
            self.assertAlmostEqual(r["lag_scan_1s"]["tau_star_s"], shift, delta=0.03)

    def test_too_few_turns_is_reported_not_guessed(self):
        t = np.arange(0.0, 100.0, 1.0 / FS)
        gt_t = np.arange(10.0, 90.0, 1.0)
        R = rotations(np.zeros(len(gt_t)))
        r = chk.heading_check(t, np.zeros((len(t), 3)), gt_t, R)
        self.assertIn("note", r["lag_scan_1s"])


class StaticWindow(unittest.TestCase):
    def test_finds_the_quiet_start(self):
        rng = np.random.RandomState(3)
        t = np.arange(0.0, 300.0, 1.0 / FS)
        a = np.tile([9.81, 0.0, 0.0], (len(t), 1)) + rng.normal(0, 0.01, (len(t), 3))
        g = rng.normal(0, 1e-4, (len(t), 3))
        moving = t >= 100.0
        a[moving] += 2.0 * np.sin(2 * np.pi * t[moving] / 3.0)[:, None]
        g[moving] += 0.3
        r = chk.static_window(t, g, a, math.radians(30.5), 20.0)
        self.assertEqual(r["window_s_from_imu_start"], [0, 100])
        self.assertAlmostEqual(r["acc_norm_median"], 9.81, delta=0.01)
        self.assertAlmostEqual(r["ratio"], 9.81 / chk.gravity(math.radians(30.5), 20.0), delta=1e-3)

    def test_no_quiet_stretch(self):
        t = np.arange(0.0, 60.0, 1.0 / FS)
        a = np.tile([9.81, 0.0, 0.0], (len(t), 1)) + np.sin(2 * np.pi * t / 1.7)[:, None]
        r = chk.static_window(t, np.zeros((len(t), 3)), a, 0.5, 0.0)
        self.assertIn("note", r)


class Gravity(unittest.TestCase):
    def test_values(self):
        self.assertAlmostEqual(chk.gravity(0.0, 0.0), 9.780327, places=6)
        self.assertAlmostEqual(chk.gravity(math.pi / 2, 0.0), 9.780327 * (1 + 0.0053024), places=6)
        self.assertAlmostEqual(chk.gravity(0.5, 1000.0) - chk.gravity(0.5, 0.0), -3.086e-3, places=6)


if __name__ == "__main__":
    unittest.main()
