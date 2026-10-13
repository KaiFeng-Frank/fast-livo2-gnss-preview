# SPDX-License-Identifier: BSD-3-Clause
"""smartpnt.imr_check on a synthetic IMR file and a slim bag built from it (units: gyro degrees in the IMR, radians in the bag)."""
import math
import os
import struct
import sys
import tempfile
import unittest

import numpy as np

import _path  # noqa: F401
from dscommon import bagmerge, rosbytes
from fl2g import gnsstime
from smartpnt import imr_check

NS = 10 ** 9


class TestImr(unittest.TestCase):
    def test_compare(self):
        try:
            import rosbags  # noqa: F401
        except ImportError:
            self.skipTest("rosbags not installed")
        d = tempfile.mkdtemp()
        gsf, asf = 4.656612873077393e-10, 2.270936965942383e-09
        hdr = bytearray(512)
        hdr[:8] = b"$IMURAW\x00"
        struct.pack_into("<iidddiid", hdr, 17, 1, 1, 200.0, gsf, asf, 2, 2, 0.0)
        n = 50
        tow0 = 28490.720667
        rng = np.random.RandomState(3)
        gyro_counts = rng.randint(-1000000, 1000000, size=(n, 3)).astype(np.int32)
        acc_counts = rng.randint(-10 ** 7, 10 ** 7, size=(n, 3)).astype(np.int32)
        body = b""
        for k in range(n):
            body += struct.pack("<d", tow0 + 0.005 * k) + struct.pack("<6i", *gyro_counts[k], *acc_counts[k])
        p_imr = os.path.join(d, "I300.imr")
        with open(p_imr, "wb") as f:
            f.write(bytes(hdr) + body)
        # slim bag: samples 2.. of the IMR (the bag lacks the first ones, as in the data), UTC stamps = GPST - 18 s, week 2322 (2024-07-07)
        conns = {"/imu_raw": bagmerge.Conn("/imu_raw", "sensor_msgs/msg/Imu", "6a62c6daae103f4ff57a132d6f95cec2", "Header header\n")}
        ev = []
        for k in range(2, n):
            gpst = gnsstime.gpst(2322, tow0 + 0.005 * k)
            t = int(round((gpst - 18.0) * NS))
            g = gyro_counts[k] * gsf * math.pi / 180.0 * 200.0                    # IMR degrees -> rad, x 200
            a = acc_counts[k] * asf * 200.0
            ev.append((t, "/imu_raw", rosbytes.build_imu(0, t, "imu", tuple(g), tuple(a))))
        p_bag = os.path.join(d, "s.bag")
        bagmerge.write_bag(p_bag, conns, iter(ev))
        res = imr_check.compare(p_bag, p_imr)
        self.assertEqual((res["bag_samples"], res["imr_samples"], res["matched_within_2us"]), (n - 2, n, n - 2))
        self.assertEqual((res["unmatched_bag_samples"], res["imr_samples_not_in_bag"]), (0, 2))
        self.assertLess(res["max_abs_time_difference_s"], 1e-6)
        self.assertLess(res["gyro_bag_minus_imr_deg_to_rad_max_abs"], 1e-9)
        self.assertLess(res["acc_bag_minus_imr_max_abs"], 1e-12)
        self.assertAlmostEqual(res["gyro_bag_over_imr_ratio_median"], math.pi / 180.0, places=6)
        self.assertEqual((res["imr_header"]["utc_or_gps"], res["imr_header"]["rate_hz"]), (2, 200.0))


if __name__ == "__main__":
    unittest.main()
