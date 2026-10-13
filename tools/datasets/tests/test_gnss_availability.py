# SPDX-License-Identifier: BSD-3-Clause
"""gnss_check.availability: the fractions of epochs with at least n RAIM-qualified satellites, the stretches with fewer
than four of them and the spread of the per-window medians (synthetic 1 Hz counts with known structure)."""
import unittest

import numpy as np

import _path  # noqa: F401
from smartpnt import gnss_check as gc


class Availability(unittest.TestCase):
    def test_fractions_and_stretches(self):
        n = np.full(1000, 12.0)
        n[100:130] = 2                               # a 30 s stretch below 4
        n[500:505] = 3                               # a 5 s stretch
        n[900:] = 6
        t = 1000.0 + np.arange(1000)
        a = gc.availability(n, t)
        self.assertAlmostEqual(a["fraction_epochs_with_at_least"]["4"], 965 / 1000)
        self.assertAlmostEqual(a["fraction_epochs_with_at_least"]["10"], 865 / 1000)
        s = a["stretches_below_4"]
        self.assertEqual(s["count"], 2)
        self.assertAlmostEqual(s["longest_s"], 30.0)
        self.assertAlmostEqual(s["total_s"], 35.0)
        self.assertEqual(s["longest_five_start_s_duration_s"][0], [100.0, 30.0])

    def test_a_gap_in_the_time_stamps_splits_a_stretch(self):
        n = np.full(8, 2.0)
        t = np.array([0, 1, 2, 3, 103, 104, 105, 106], dtype=float)   # the receiver was silent for 100 s: two stretches, not one
        s = gc.availability(n, t)["stretches_below_4"]
        self.assertEqual(s["count"], 2)
        self.assertAlmostEqual(s["longest_s"], 4.0)

    def test_window_medians(self):
        n = np.concatenate([np.full(200, 20.0), np.full(100, 1.0)])
        t = np.arange(300.0)
        w = gc.availability(n, t)["window_100s_median"]
        self.assertEqual(w["windows"], 3)
        self.assertEqual(w["min"], 1.0)
        self.assertAlmostEqual(w["fraction_windows_below_4"], 1 / 3)

    def test_no_low_epochs(self):
        s = gc.availability(np.full(50, 9.0), np.arange(50.0))["stretches_below_4"]
        self.assertEqual((s["count"], s["longest_s"], s["total_s"]), (0, 0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
