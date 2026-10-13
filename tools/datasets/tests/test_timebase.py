# SPDX-License-Identifier: BSD-3-Clause
"""dscommon.timebase: the stamp rule of the SmartPNT slim bags."""
import unittest

import _path  # noqa: F401
from dscommon import timebase
from fl2g import gnsstime

NS = 10 ** 9


class TestTimebase(unittest.TestCase):
    def test_real_example_data01_20240707(self):
        # IMU bag of Data01_20240707: first stamp 2014-07-03 07:54:50.725666046 (== I300 IMR GPST tow 28490.7257),
        # mini receiver RINEX first epoch 2024-07-07 07:54:40.6 GPST (a Sunday: week 2322)
        first = 1404374090 * NS + 725666046
        ref = gnsstime.gpst(2322, 28480.6)
        shift, gpst_first, leap = timebase.resolve_shift_ns(first, ref)
        self.assertEqual(leap, 18)
        self.assertEqual(shift, 315964782 * NS)
        self.assertEqual(gpst_first, gnsstime.gpst(2322, 28490.725666046))
        # UTC Unix of 2024-07-07 07:54:32.725666046
        self.assertEqual(first + shift, 1720338872 * NS + 725666046)

    def test_day_is_taken_from_the_reference_not_the_stamp_date(self):
        # stamp time of day 23:59:55, reference just after the next GPST midnight: the stream started the day before
        first = 1404374090 * NS - (28490 * NS) + 86395 * NS
        ref = gnsstime.gpst(2322, 86400.0 + 10.0)
        shift, gpst_first, _ = timebase.resolve_shift_ns(first, ref)
        self.assertEqual(gpst_first, gnsstime.gpst(2322, 86395.0))
        with self.assertRaises(ValueError):                                     # reference 6 h away from every candidate day
            timebase.resolve_shift_ns(first, ref + 6 * 3600.0 + 1.0)

    def test_unwrapper(self):
        w = timebase.Unwrapper()
        stamps = [86398 * NS, 86399 * NS, 0 * NS + 500, 1 * NS, 2 * NS]
        out = [w(s) for s in stamps]
        self.assertEqual(out, [86398 * NS, 86399 * NS, 86400 * NS + 500, 86401 * NS, 86402 * NS])
        self.assertEqual(w.wraps, 1)
        w2 = timebase.Unwrapper()
        self.assertEqual([w2(s) for s in (5 * NS, 6 * NS, 7 * NS)], [5 * NS, 6 * NS, 7 * NS])      # no wrap, no change

    def test_utc_ns_of_gpst(self):
        self.assertEqual(timebase.utc_ns_of_gpst(2322, 28480.6), 1720338862 * NS + 600000000)
        # leap second count of the date: 18 s since 2017
        self.assertEqual(timebase.utc_ns_of_gpst(2322, 18.0), 1720310400 * NS)                    # 2024-07-07T00:00:00Z


if __name__ == "__main__":
    unittest.main()
