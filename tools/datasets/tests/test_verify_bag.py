# SPDX-License-Identifier: BSD-3-Clause
"""verify_bag.py pieces that the end-to-end test does not reach: the IMU-grid rule for start-up gaps and the JPEG size reader."""
import io
import contextlib
import unittest

import numpy as np

import _path  # noqa: F401
import verify_bag

NS = 10 ** 9


def run_grid(imu, first_sensor_ns):
    rep = verify_bag.Report()
    with contextlib.redirect_stdout(io.StringIO()):
        verify_bag.check_imu_grid(rep, imu, first_sensor_ns)
    return rep.checks[0]


def imu(gaps, n=442461, dt_median=0.005, dt_min=0.005):
    return {"n": n, "dt_median_s": dt_median, "dt_min_s": dt_min, "first_ns": 0, "gaps": gaps}


class TestImuGrid(unittest.TestCase):
    def test_start_up_gap_before_the_first_scan_is_reported_not_failed(self):
        # Data02_20240718: 52 records, 1.7449 s of nothing, the first scan two minutes later
        c = run_grid(imu([[250000000, 1994900000, 1.7449]]), 130 * NS)
        self.assertTrue(c["ok"], c)
        self.assertIn("start-up gap", c["detail"])
        self.assertIn("0 gap(s)", c["detail"])

    def test_gap_inside_the_replayed_span_fails(self):
        c = run_grid(imu([[500 * NS, 502 * NS, 2.0]]), 130 * NS)
        self.assertFalse(c["ok"], c)

    def test_small_gap_inside_is_counted_but_tolerated(self):
        c = run_grid(imu([[500 * NS, 500 * NS + 30000000, 0.03]]), 130 * NS)
        self.assertTrue(c["ok"], c)
        self.assertIn("1 gap(s)", c["detail"])

    def test_wrong_rate_or_nonincreasing_stamps_fail(self):
        self.assertFalse(run_grid(imu([], dt_median=0.01), 0)["ok"])
        self.assertFalse(run_grid(imu([], dt_min=0.0), 0)["ok"])


class TestJpegSize(unittest.TestCase):
    def test_sizes_without_decoding(self):
        try:
            import cv2
        except ImportError:
            raise unittest.SkipTest("cv2 not installed")
        for w, h in ((64, 32), (640, 512), (612, 512)):
            img = (np.random.RandomState(0).rand(h, w, 3) * 255).astype(np.uint8)
            data = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 90])[1].tobytes()
            self.assertEqual(verify_bag.jpeg_size(data), (w, h))


if __name__ == "__main__":
    unittest.main()
