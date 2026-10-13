# SPDX-License-Identifier: BSD-3-Clause
"""The pure parts of the data-only clock checks: time_check.lag_search / windowed (IMU against Doppler signals) and
cam_imu_toff.scan / peak (image shift against the gyro rotation integral). Synthetic signals with a known lag."""
import unittest

import numpy as np

import _path  # noqa: F401
from smartpnt import time_check as tcheck
from smartpnt import cam_imu_toff as ctoff


def smooth_noise(n, fs, sigma_s, seed):
    rng = np.random.RandomState(seed)
    x = tcheck.gauss(rng.normal(size=n), sigma_s * fs)
    return x / np.std(x)


class LagSearch(unittest.TestCase):
    def test_recovers_a_known_lag_and_sign(self):
        fs, n = 100.0, 40000
        t = np.arange(n) / fs
        a = smooth_noise(n, fs, 0.4, 1)
        for true in (0.37, -0.21, 0.0):
            b = np.interp(t - true, t, a, left=0.0, right=0.0)               # b(t) = a(t - true): the event is later in b by `true`
            m = np.ones(n, dtype=bool)
            r = tcheck.lag_search(t, a, m, b, m, 20.0, fs)
            self.assertAlmostEqual(r["fine_peak_lag_s"], true, delta=0.01)
            self.assertGreater(r["fine_peak_corr"], 0.95)
            self.assertAlmostEqual(r["coarse_peak_lag_s"], true, delta=0.06)

    def test_a_leap_second_shows_at_18_s(self):
        fs, n = 100.0, 60000
        t = np.arange(n) / fs
        a = smooth_noise(n, fs, 0.4, 2)
        b = np.interp(t - 18.0, t, a, left=0.0, right=0.0)
        m = np.ones(n, dtype=bool)
        r = tcheck.lag_search(t, a, m, b, m, 20.0, fs)
        self.assertAlmostEqual(r["coarse_peak_lag_s"], 18.0, delta=0.06)
        self.assertGreater(r["corr_at_lag_plus18"], 0.9)
        self.assertLess(abs(r["corr_at_lag_0"]), 0.3)

    def test_windowed_lag_is_stable_for_a_constant_offset(self):
        fs, n = 100.0, 60000
        t = np.arange(n) / fs
        a = smooth_noise(n, fs, 0.3, 3)
        b = np.interp(t - 0.12, t, a, left=0.0, right=0.0)
        m = np.ones(n, dtype=bool)
        w = tcheck.windowed(t, a, m, b, m, fs, 0.12)
        self.assertGreaterEqual(w["windows"], 5)
        self.assertAlmostEqual(w["lag_median_s"], 0.12, delta=0.01)
        self.assertLess(w["lag_mad_s"], 0.01)
        self.assertLess(abs(w["drift_ppm"]), 200.0)

    def test_masked_samples_are_ignored(self):
        fs, n = 100.0, 30000
        t = np.arange(n) / fs
        a = smooth_noise(n, fs, 0.4, 4)
        b = np.interp(t - 0.2, t, a, left=0.0, right=0.0)
        b[5000:9000] = 5.0 * np.random.RandomState(0).normal(size=4000)       # garbage under the mask
        m = np.ones(n, dtype=bool)
        mb = m.copy()
        mb[5000:9000] = False
        r = tcheck.lag_search(t, a, m, b, mb, 20.0, fs)
        self.assertAlmostEqual(r["fine_peak_lag_s"], 0.2, delta=0.01)


class CamImuOffset(unittest.TestCase):
    def test_scan_finds_the_injected_offset(self):
        fs = 200.0
        t = np.arange(0, 120.0, 1.0 / fs)
        rate = smooth_noise(len(t), fs, 0.05, 5) * 0.2                         # rad/s, fast enough to resolve milliseconds
        cum = np.concatenate([[0.0], np.cumsum(0.5 * (rate[1:] + rate[:-1]) * np.diff(t))])
        cam_t = np.arange(1.0, 110.0, 0.1)
        true = 0.031
        c0 = np.interp(cam_t + true, t, cum)
        shift = np.concatenate([[np.nan], np.diff(c0)])                         # shift[k + 1]: frame k -> k + 1
        pairs = np.arange(len(cam_t) - 1)
        deltas = np.arange(-0.1, 0.1 + 1e-9, 0.001)
        rho = ctoff.scan(cam_t, np.nan_to_num(shift), pairs, t, cum, deltas)
        d, r = ctoff.peak(deltas, rho)
        self.assertAlmostEqual(d, true, delta=0.002)
        self.assertGreater(r, 0.99)

    def test_negative_correlation_is_found_too(self):
        fs = 200.0
        t = np.arange(0, 120.0, 1.0 / fs)
        rate = smooth_noise(len(t), fs, 0.05, 6) * 0.2
        cum = np.concatenate([[0.0], np.cumsum(0.5 * (rate[1:] + rate[:-1]) * np.diff(t))])
        cam_t = np.arange(1.0, 110.0, 0.1)
        shift = -np.concatenate([[np.nan], np.diff(np.interp(cam_t - 0.012, t, cum))])
        pairs = np.arange(len(cam_t) - 1)
        deltas = np.arange(-0.1, 0.1 + 1e-9, 0.001)
        rho = ctoff.scan(cam_t, np.nan_to_num(shift), pairs, t, cum, deltas)
        d, r = ctoff.peak(deltas, rho)
        self.assertAlmostEqual(d, -0.012, delta=0.002)
        self.assertLess(r, -0.99)


if __name__ == "__main__":
    unittest.main()
