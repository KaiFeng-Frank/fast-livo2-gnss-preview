# SPDX-License-Identifier: BSD-3-Clause
"""gt_stats: path length, coverage of the bag, the trust mask and the quality counts on a synthetic ground-truth directory
(a straight run east at 2 m/s, 101 epochs, ten of them masked)."""
import json
import os
import tempfile
import unittest

import numpy as np

import _path  # noqa: F401
from smartpnt import gt_stats

SEQ = "Synth_seq"


def write_sequence(root, n=101, masked=range(20, 30)):
    d = os.path.join(root, "smartpnt", SEQ)
    os.makedirs(os.path.join(d, "gt"))
    t0 = 1000.0
    with open(os.path.join(d, "gt", "gt_clean.tum"), "w") as f:
        for k in range(n):
            f.write("%.6f %.6f 0 0 0 0 0 1\n" % (t0 + k, 2.0 * k))
    with open(os.path.join(d, "gt", "gt_quality.csv"), "w") as f:
        f.write("t,Q,AmbStatus,sigma_3d_m,hdng_sd_deg,e_sep_m,n_sep_m,h_sep_m,speed_h_mps,n_sats\n")
        for k in range(n):
            bad = k in masked
            f.write("%.6f,%d,%s,%.4f,0.01,0,0,0,2.0,%d\n" % (t0 + k, 4 if bad else 1, "Float" if bad else "Fixed", 1.2 if bad else 0.02, 12))
    with open(os.path.join(d, "gt", "trust_mask.csv"), "w") as f:
        f.write("t,trusted,reason,ate_exclude\n")
        for k in range(n):
            bad = k in masked
            f.write("%.6f,%d,%d,%d\n" % (t0 + k, 0 if bad else 1, 2 if bad else 0, 2 if bad else 0))
    with open(os.path.join(d, "build.json"), "w") as f:
        json.dump({"bag": {"start_ns": int((t0 - 30) * 1e9), "end_ns": int((t0 + n - 1 + 50) * 1e9), "duration_s": n - 1 + 80},
                   "topics": {"/imu_raw": {"first_utc": "2024-07-01T00:00:00Z"}}}, f)
    with open(os.path.join(d, "gt", "gt_audit.json"), "w") as f:
        json.dump({"primary": "X_GroundTruth.ref", "header": {"program": "IE"}, "readme_check": {"label": "Ant00"}}, f)


class GtStats(unittest.TestCase):
    def test_numbers(self):
        with tempfile.TemporaryDirectory() as root:
            write_sequence(root)
            r = gt_stats.run(root, SEQ)
        self.assertEqual(r["epochs"], 101)
        self.assertAlmostEqual(r["path_horizontal_m"], 200.0, places=3)
        self.assertAlmostEqual(r["path_horizontal_trusted_m"], 2.0 * (100 - 11), places=3)        # 11 steps touch a masked epoch
        self.assertEqual((r["masked_epochs"], r["mask_stretches"], r["mask_stretch_len_s"]), (10, 1, [10]))
        self.assertEqual((r["trusted_epochs"], r["trusted_runs"], r["trusted_longest_run_s"]), (91, 2, 71))
        self.assertAlmostEqual(r["gt_start_after_bag_start_s"], 30.0, places=3)
        self.assertAlmostEqual(r["gt_end_before_bag_end_s"], 50.0, places=3)
        self.assertAlmostEqual(r["gt_span_over_bag"], 100.0 / 180.0, places=6)
        self.assertEqual(r["Q"], {1: 91, 4: 10})
        self.assertEqual(r["amb"], {"Fixed": 91, "Float": 10})
        self.assertAlmostEqual(r["sigma3d_trusted_m"]["max"], 0.02)
        self.assertAlmostEqual(r["sigma3d_all_m"]["max"], 1.2)
        self.assertAlmostEqual(r["speed_h_mps"]["median"], 2.0)
        self.assertEqual(r["speed_h_mps"]["fraction_standing"], 0.0)
        self.assertAlmostEqual(r["extent_m"]["east"][1], 200.0)
        self.assertEqual(r["reference"]["label"], "Ant00")

    def test_mismatched_quality_file_is_refused(self):
        with tempfile.TemporaryDirectory() as root:
            write_sequence(root)
            q = os.path.join(root, "smartpnt", SEQ, "gt", "gt_quality.csv")
            with open(q) as f:
                lines = f.read().splitlines()
            with open(q, "w") as f:
                f.write("\n".join(lines[:-1]) + "\n")
            with self.assertRaises(SystemExit):
                gt_stats.run(root, SEQ)

    def test_run_lengths(self):
        self.assertEqual(gt_stats.runs_of([1, 1, 0, 1, 0, 0, 1, 1, 1]), [2, 1, 3])
        self.assertEqual(gt_stats.runs_of([0, 0]), [])


if __name__ == "__main__":
    unittest.main()
