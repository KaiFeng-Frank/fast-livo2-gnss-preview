# SPDX-License-Identifier: BSD-3-Clause
"""Unit tests of the DTW replica (A81 2 item 1): python3 -m unittest discover -s tools/calib/tests -p 'test_dtw_scan.py'

The first group re-states the C++ tests of livgoDtwLag (gnss/test/test_livgo.cpp: LivgoDtw.*, LivgoA80.Dtw*) on the same
synthetic series; the bit-level agreement with the C++ function on real series is checked by dtw_align.cpp + `compare`."""
import math
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import dtw_scan as D  # noqa: E402


def a80_speeds(t_end, dt, delay):
    """gnss_test a80Speeds: an aperiodic series, gnss = host delayed by delay(t)."""
    f = lambda x: math.sin(0.9 * x) + 0.7 * math.sin(0.37 * x + 1.0) + 0.5 * math.sin(1.9 * x + 0.3) + 0.3 * math.sin(3.1 * x)
    t, vh, vg = [], [], []
    ti = 0.0
    while ti < t_end:
        t.append(ti)
        vh.append(f(ti))
        vg.append(f(ti - delay(ti)))
        ti += dt
    return t, vh, vg


class CppTestsRestated(unittest.TestCase):
    def test_delayed_gnss_speed_comes_back_negative(self):
        delay = 0.3
        th = [0.1 * i for i in range(200)]
        vh = [math.sin(2.0 * math.pi * t / 5.0) for t in th]
        vg = [math.sin(2.0 * math.pi * (t - delay) / 5.0) for t in th]
        ok, lag = D.dtw_lag(th, vg, th, vh)
        self.assertTrue(ok)
        self.assertAlmostEqual(lag, -delay, delta=0.15)

    def test_aligned_series_stay_near_zero(self):
        t = [0.1 * i for i in range(200)]
        v = [math.sin(2.0 * math.pi * x / 5.0) + 0.2 * math.sin(2.0 * math.pi * x / 1.7) for x in t]
        ok, lag = D.dtw_lag(t, v, t, v)
        self.assertTrue(ok)
        self.assertAlmostEqual(lag, 0.0, delta=0.15)

    def test_default_parameters_equal_the_passed_defaults(self):
        t, vh, vg = a80_speeds(60.0, 0.1, lambda x: 0.3)
        a = D.dtw_lag(t, vg, t, vh)
        b = D.dtw_lag(t, vg, t, vh, 0.1, 40.0, 2.0)
        self.assertEqual(a, b)
        self.assertAlmostEqual(a[1], -0.3, delta=0.15)

    def test_grid_parameter_takes_effect(self):
        t, vh, vg = a80_speeds(40.0, 0.05, lambda x: 0.3)
        ok, lag = D.dtw_lag(t, vg, t, vh, 0.05)
        self.assertTrue(ok)
        self.assertAlmostEqual(lag, -0.3, delta=0.08)
        t, vh, vg = a80_speeds(120.0, 0.25, lambda x: 1.0)
        ok, lag = D.dtw_lag(t, vg, t, vh, 0.25, 60.0)
        self.assertTrue(ok)
        self.assertAlmostEqual(lag, -1.0, delta=0.3)

    def test_window_selects_the_tail_of_the_overlap(self):
        t, vh, vg = a80_speeds(90.0, 0.1, lambda x: 0.3 if x < 70.0 else -0.2)
        ok1, tail = D.dtw_lag(t, vg, t, vh, 0.1, 15.0)
        ok2, whole = D.dtw_lag(t, vg, t, vh, 0.1, 90.0)
        self.assertTrue(ok1 and ok2)
        self.assertAlmostEqual(tail, 0.2, delta=0.15)
        self.assertAlmostEqual(whole, -0.3, delta=0.15)

    def test_degenerate_parameters_are_refused(self):
        t, vh, vg = a80_speeds(60.0, 0.1, lambda x: 0.3)
        self.assertFalse(D.dtw_lag(t, vg, t, vh, 0.0)[0])
        self.assertFalse(D.dtw_lag(t, vg, t, vh, -0.1)[0])
        self.assertFalse(D.dtw_lag(t, vg, t, vh, 0.1, 1.0)[0])
        self.assertFalse(D.dtw_lag(t, vg, t, vh, 0.1, 40.0, 0.0)[0])
        self.assertFalse(D.dtw_lag(t, vg, t, vh, 0.1, float("nan"))[0])


class Structure(unittest.TestCase):
    def test_overlap_shorter_than_three_seconds_and_short_series_fail(self):
        t = [0.1 * i for i in range(25)]
        self.assertFalse(D.dtw_lag(t, t, t, t)[0])               # 2.4 s
        self.assertFalse(D.dtw_lag(t[:4], t[:4], t[:4], t[:4])[0])

    def test_lag_is_a_multiple_of_the_grid(self):
        t, vh, vg = a80_speeds(60.0, 0.1, lambda x: 0.27)
        for g in (0.05, 0.1, 0.2):
            ok, lag = D.dtw_lag(t, vg, t, vh, g, 40.0)
            self.assertTrue(ok)
            self.assertAlmostEqual(lag / g, round(lag / g), places=9)

    def test_constant_series_do_not_fail_and_return_the_band_edge(self):
        # the std guard (s <= 1e-6 -> 1.0) makes an exactly constant window a flat zero series: the function does not fail on no
        # motion, every cell costs 0, the strict `<` keeps the first end cell (j = n-1-W) and the diagonal path reads -W*dt = -band_s
        t = [0.1 * i for i in range(500)]
        ok, lag = D.dtw_lag(t, [1.0] * 500, t, [2.0] * 500)
        self.assertTrue(ok)
        self.assertEqual(lag, -2.0)

    def test_lround_halves_away_from_zero(self):
        self.assertEqual(D._lround(2.5), 3)
        self.assertEqual(D._lround(0.5), 1)
        self.assertEqual(D._lround(-0.5), -1)
        self.assertEqual(D._lround(19.999999999999996), 20)

    def test_band_of_the_default_grids(self):
        # W = lround(2 / dt): 20 samples at 0.1 s, 40 at 0.05 s, 10 at 0.2 s
        self.assertEqual([D._lround(2.0 / g) for g in (0.1, 0.05, 0.2)], [20, 40, 10])


class Online(unittest.TestCase):
    def test_first_success_uses_exactly_min_epochs_samples_and_is_one_shot(self):
        t, vh, vg = a80_speeds(120.0, 0.1, lambda x: 0.3)
        t, vh, vg = np.array(t), np.array(vh), np.array(vg)
        calls = []
        r = D.online_first_lag(t, vg, vh, 10, 400, 0.1, 40.0, sink=lambda sl, *a: calls.append((sl.start, sl.stop)))
        self.assertTrue(r["ok"])
        self.assertEqual(r["n"], 400)
        self.assertEqual(calls, [(10, 410)])
        self.assertAlmostEqual(r["lag"], -0.3, delta=0.15)

    def test_data_end_before_a_full_window_is_a_failure(self):
        t, vh, vg = a80_speeds(30.0, 0.1, lambda x: 0.0)
        r = D.online_first_lag(np.array(t), np.array(vg), np.array(vh), 0, 400, 0.1, 40.0)
        self.assertFalse(r["ok"])

    def test_window_longer_than_the_series_uses_the_last_window_s(self):
        t, vh, vg = a80_speeds(100.0, 0.1, lambda x: 0.3 if x < 70.0 else -0.2)
        t, vh, vg = np.array(t), np.array(vh), np.array(vg)
        r = D.online_first_lag(t, vg, vh, 0, 900, 0.1, 15.0)       # 90 s collected, only the last 15 s are warped
        self.assertTrue(r["ok"])
        self.assertAlmostEqual(r["lag"], 0.2, delta=0.15)


class Rules(unittest.TestCase):
    def _tab(self, mads):
        return {k: {"mad_s": v} for k, v in mads.items()}

    def test_smallest_mad_wins(self):
        m = {(g, w): 0.1 for g in D.GRIDS for w in D.WINDOWS}
        m[(0.1, 60.0)] = 0.02
        chosen, near, ties = D.apply_a81_rule(self._tab(m))
        self.assertEqual(chosen, (0.1, 60.0))

    def test_within_5ms_the_shorter_window_wins(self):
        m = {(g, w): 0.1 for g in D.GRIDS for w in D.WINDOWS}
        m[(0.1, 60.0)] = 0.020
        m[(0.1, 20.0)] = 0.024
        m[(0.1, 40.0)] = 0.026          # 6 ms above the smallest: not within
        chosen, near, ties = D.apply_a81_rule(self._tab(m))
        self.assertEqual(chosen, (0.1, 20.0))
        self.assertEqual(near, [(0.1, 20.0), (0.1, 60.0)])

    def test_two_grids_at_the_same_window_are_a_tie_the_rule_does_not_resolve(self):
        m = {(g, w): 0.1 for g in D.GRIDS for w in D.WINDOWS}
        m[(0.1, 20.0)] = 0.0
        m[(0.2, 20.0)] = 0.0
        chosen, near, ties = D.apply_a81_rule(self._tab(m))
        self.assertIsNone(chosen)
        self.assertEqual(ties, [(0.1, 20.0), (0.2, 20.0)])

    def test_mad_is_unscaled(self):
        self.assertEqual(D.mad([0.0, 0.0, 0.1, 0.2, 0.2]), 0.1)


class HostSide(unittest.TestCase):
    def _state(self, n=200, dt=0.1, yaw_rate=0.5, speed=3.0):
        t = dt * np.arange(n)
        yaw = yaw_rate * t
        R = np.zeros((n, 3, 3))
        R[:, 0, 0] = np.cos(yaw)
        R[:, 0, 1] = -np.sin(yaw)
        R[:, 1, 0] = np.sin(yaw)
        R[:, 1, 1] = np.cos(yaw)
        R[:, 2, 2] = 1.0
        v = np.zeros((n, 3))
        v[:, 0] = speed
        p = np.zeros((n, 3))
        p[:, 0] = speed * t
        return {"t": t, "R": R, "p": p, "v": v}

    def test_lever_term_equals_the_derivative_of_the_antenna_track(self):
        st = self._state()
        l = [-0.118, 0.0, 0.12]
        zb = D.host_antenna_velocity(st, l, "bridge")
        zt = D.host_antenna_velocity(st, l, "track")
        np.testing.assert_allclose(zb[5:-5], zt[5:-5], atol=2e-3)
        # |omega x l| for a yaw rate of 0.5 rad/s about z: 0.5 * 0.118 = 0.059 m/s
        self.assertGreater(np.linalg.norm(zb[50] - st["v"][50]), 0.05)

    def test_zero_lever_is_the_filter_velocity(self):
        st = self._state()
        np.testing.assert_array_equal(D.host_antenna_velocity(st, [0, 0, 0], "bridge"), st["v"])

    def test_speed_is_linear_between_cycles_and_nan_outside(self):
        st = self._state(speed=2.0)
        s = D.host_speed_at(st, [0, 0, 0], np.array([-1.0, 0.05, 1.234, 19.85, 25.0]))
        self.assertTrue(np.isnan(s[0]) and np.isnan(s[4]))
        np.testing.assert_allclose(s[1:4], 2.0)

    def test_rotvec_inverts_rotation(self):
        th = 0.3
        R = np.array([[math.cos(th), -math.sin(th), 0.0], [math.sin(th), math.cos(th), 0.0], [0.0, 0.0, 1.0]])
        np.testing.assert_allclose(D.rotvec(R), [0.0, 0.0, th], atol=1e-12)


class Tables(unittest.TestCase):
    def _rows(self):
        rows = []
        for seq, lags in (("a", [0.0, 0.0, 0.2, 0.0]), ("b", [0.0, -0.2, 0.0, 0.0])):
            for k, lag in enumerate(lags):
                for g in D.GRIDS:
                    for w in D.WINDOWS:
                        rows.append({"seq": seq, "k": k, "grid_s": g, "window_s": w, "ok": True, "lag_s": lag, "common_start": True, "host_std": 1.0,
                                     "start_rel_s": 60.0 * k, "start_t_rx": 0.0, "corr": 0.9, "gnss_std": 1.0, "t_success_rel_s": w})
        return rows

    def test_combo_table_pools_the_sequences(self):
        tab = D.combo_table(self._rows())
        e = tab[(0.1, 40.0)]
        self.assertEqual(e["n_windows"], 8)
        self.assertEqual(e["n_ok"], 8)
        self.assertEqual(e["median_s"], 0.0)
        self.assertEqual(e["mad_s"], 0.0)
        self.assertAlmostEqual(e["zero_share"], 6 / 8)
        self.assertAlmostEqual(e["abs_gt_0p1_share"], 2 / 8)
        self.assertEqual(sorted(e["per_seq"]), ["a", "b"])

    def test_failed_windows_are_counted_and_do_not_enter_the_mad(self):
        rows = self._rows()
        for r in rows:
            if r["seq"] == "a" and r["k"] == 2:
                r["ok"], r["lag_s"] = False, None
        e = D.combo_table(rows)[(0.1, 40.0)]
        self.assertEqual(e["n_windows"], 8)
        self.assertEqual(e["n_ok"], 7)
        self.assertAlmostEqual(e["fail_share"], 1 / 8)

    def test_non_common_starts_are_left_out_by_default(self):
        rows = self._rows()
        rows[0]["common_start"] = False
        self.assertEqual(D.combo_table(rows)[(0.05, 20.0)]["n_windows"], 7)
        self.assertEqual(D.combo_table(rows, common_only=False)[(0.05, 20.0)]["n_windows"], 8)

    def test_compare_variants_counts_identical_lags(self):
        a = self._rows()
        b = [dict(r) for r in a]
        for r in b:
            if r["seq"] == "b" and r["k"] == 1:
                r["lag_s"] = 0.3
        c = D.compare_variants(a, b)
        self.assertEqual(c["n"], len(a))
        self.assertAlmostEqual(c["same"], 1.0 - 12 / len(a))
        self.assertAlmostEqual(c["max_abs"], 0.5)


class Dump(unittest.TestCase):
    def test_hexfloat_round_trip_is_exact(self):
        for x in (0.1, 1720338906.3, 1e-300, 3.141592653589793):
            self.assertEqual(float.fromhex(float(x).hex()), x)


if __name__ == "__main__":
    unittest.main()
