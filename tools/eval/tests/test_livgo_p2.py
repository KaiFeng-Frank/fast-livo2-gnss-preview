# SPDX-License-Identifier: BSD-3-Clause
"""livgo_p2.py (AMENDMENTS A75 revision 1, prediction P2): the statistic, the registered thresholds and the verdict on
synthetic livgo_tdcp_gain.csv files — a live arm, a dead control, a dose response, the instrument self-checks."""
import contextlib
import io
import os
import tempfile
import unittest

import numpy as np

import _path  # noqa: F401
import livgo_p2 as p2


def write_run(root, seq, arm, seed, g_of_n, n_epochs=400, seed_rng=0, status0=0.0, phase=p2.TRACKING, inflate=0.0):
    """One livgo_tdcp_gain.csv; g_of_n(n_tdcp) -> the TDCP share of an epoch with that many accepted rows."""
    d = os.path.join(root, seq, arm, "s%d" % seed)
    os.makedirs(d)
    rng = np.random.default_rng(seed_rng)
    with open(os.path.join(d, p2.FILE), "w") as f:
        f.write(",".join(p2.COLUMNS) + "\n")
        for k in range(n_epochs):
            n = int(rng.integers(0, 9))
            prior = 1.2e-5
            no = prior * 0.8
            if n == 0:
                allv, status = no, 2
            else:
                allv, status = no * (1.0 - g_of_n(n)), (0 if rng.random() < status0 else 1)
            if rng.random() < inflate:
                allv = prior * 1.5
            tr_no = no if status != 0 else float("nan")
            f.write("%.9f,%d,0,6,%d,%.12e,%.12e,%.12e,%d\n" % (100.0 + 0.1 * k, phase, n, prior, allv, tr_no, status))
    return d


class Statistic(unittest.TestCase):
    def test_quantile_type7(self):
        self.assertAlmostEqual(p2.quantile([1, 2, 3, 4], 0.5), 2.5)
        self.assertAlmostEqual(p2.quantile([1, 2, 3, 4, 5], 0.05), 1.2)
        self.assertTrue(p2.quantile([], 0.5) != p2.quantile([], 0.5))   # nan

    def test_g_is_the_share_left_after_everything_but_tdcp(self):
        with tempfile.TemporaryDirectory() as root:
            d = write_run(root, "Grass01", p2.ARM_V2, 0, lambda n: 0.30)
            s = p2.summarise(p2.read_rows(os.path.join(d, p2.FILE)))
        self.assertAlmostEqual(s["g_median"], 0.30, places=9)
        self.assertGreater(s["coverage"], 0.8)          # n = 0 in about 1 epoch of 9
        self.assertLess(s["coverage"], 1.0)
        self.assertEqual(s["n_g"], s["n_with_tdcp"])

    def test_only_tracking_epochs_count(self):
        with tempfile.TemporaryDirectory() as root:
            d = write_run(root, "Grass01", p2.ARM_V2, 0, lambda n: 0.30, phase=7)   # OUTAGE
            s = p2.summarise(p2.read_rows(os.path.join(d, p2.FILE)))
        self.assertEqual(s["n_closing"], 0)
        self.assertEqual(s["n_g"], 0)

    def test_bad_header_is_refused(self):
        with tempfile.TemporaryDirectory() as root:
            path = os.path.join(root, "x.csv")
            with open(path, "w") as f:
                f.write("t_k,phase\n1,2\n")
            with self.assertRaises(ValueError):
                p2.read_rows(path)


class Verdict(unittest.TestCase):
    def build(self, root, v2_g, ctl_g, **kw):
        for i, seq in enumerate(("Grass01", "Grass02", "Z-Rough-Road01")):
            for seed in range(5):
                write_run(root, seq, p2.ARM_V2, seed, v2_g, seed_rng=10 * i + seed, **kw)
            write_run(root, seq, p2.ARM_CONTROL, 4, ctl_g, seed_rng=99 + i)

    def verdict(self, root):
        runs = p2.collect([root])
        return p2.verdict(runs), runs

    def test_live_arm_and_dead_control_is_met(self):
        with tempfile.TemporaryDirectory() as root:
            self.build(root, lambda n: 0.02 + 0.01 * n, lambda n: 1e-7)
            (v, why), runs = self.verdict(root)
        self.assertEqual(v, "MET", why)
        self.assertEqual(len(runs), 18)
        self.assertTrue(all(r["checks"].get("P2b") for r in runs if r["arm"] == p2.ARM_V2))

    def test_weak_arm_is_not_met(self):
        with tempfile.TemporaryDirectory() as root:
            self.build(root, lambda n: 0.002 + 0.0005 * n, lambda n: 1e-7)
            (v, why), _ = self.verdict(root)
        self.assertEqual(v, "NOT MET")
        self.assertTrue(any("P2a" in w for w in why))

    def test_no_dose_response_is_not_met(self):
        with tempfile.TemporaryDirectory() as root:
            self.build(root, lambda n: 0.05 if n <= 3 else 0.02, lambda n: 1e-7)
            (v, why), _ = self.verdict(root)
        self.assertEqual(v, "NOT MET")
        self.assertTrue(any("P2b" in w for w in why))

    def test_a_control_that_is_not_dead_leaves_P2_undecided(self):
        with tempfile.TemporaryDirectory() as root:
            self.build(root, lambda n: 0.05, lambda n: 0.05)
            (v, why), _ = self.verdict(root)
        self.assertEqual(v, "UNDECIDED")
        self.assertIn("negative control", why[0])

    def test_missing_control_is_undecided(self):
        with tempfile.TemporaryDirectory() as root:
            write_run(root, "Grass01", p2.ARM_V2, 0, lambda n: 0.05)
            (v, why), _ = self.verdict(root)
        self.assertEqual(v, "UNDECIDED")

    def test_counterfactual_failures_make_the_instrument_undecided(self):
        with tempfile.TemporaryDirectory() as root:
            self.build(root, lambda n: 0.05, lambda n: 1e-7, status0=0.2)
            (v, why), _ = self.verdict(root)
        self.assertEqual(v, "UNDECIDED")
        self.assertIn("instrument", why[0])

    def test_a_posterior_above_its_prior_is_an_instrument_failure(self):
        with tempfile.TemporaryDirectory() as root:
            self.build(root, lambda n: 0.05, lambda n: 1e-7, inflate=0.1)
            (v, why), _ = self.verdict(root)
        self.assertEqual(v, "UNDECIDED")

    def test_strata_below_the_minimum_are_not_evaluable(self):
        with tempfile.TemporaryDirectory() as root:
            write_run(root, "Grass01", p2.ARM_V2, 0, lambda n: 0.05, n_epochs=20)
            runs = p2.collect([root])
        self.assertIsNone(runs[0]["checks"]["P2b"])

    def test_report_and_json(self):
        with tempfile.TemporaryDirectory() as root:
            self.build(root, lambda n: 0.02 + 0.01 * n, lambda n: 1e-7)
            buf = io.StringIO()
            v = p2.report(p2.collect([root]), out=buf)
            out_json = os.path.join(root, "p2.json")
            with contextlib.redirect_stdout(io.StringIO()):
                rc = p2.main([root, "--json", out_json])
            self.assertTrue(os.path.exists(out_json))
        self.assertEqual(v, "MET")
        self.assertEqual(rc, 0)
        self.assertIn("P2: MET", buf.getvalue())


if __name__ == "__main__":
    unittest.main()
