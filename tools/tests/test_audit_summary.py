# SPDX-License-Identifier: BSD-3-Clause
"""tools/audit_summary.py: the committed summary is a pure composition of the archived reports — one title, the GT
report demoted one level, the method-1 prior and (when it ran) the method-2 segments and A18 text, notes after the
title block, container paths shown as host paths."""
import json
import os
import tempfile
import unittest

import _path  # noqa: F401
import audit_summary as su

AUDIT_MD = "# I2 数据审计 — SeqX\n\n生成:bag `/r/data/m3dgr/bags/SeqX.bag`。\n\n总评:**PASS**。\n\n## 检查表\n\n| # | check |\n|---|---|\n"
GT_MD = "# GT 审计 — SeqX\n\n生成:x。\n\n## 检查表\n\n| P4 | ok |\n"
M1 = {"method1_record_time": {"leap_s": 18, "median_s": 0.0381, "mad_s": 0.002, "p5_s": 0.035, "p95_s": 0.041, "drift_ppm": 3.2,
                              "steps": [], "header_minus_record_ms": {"/livox/avia/imu": {"median_ms": -57.8, "p1_ms": -107.0, "p99_ms": -7.4, "drift_ppm": -9.6}}},
      "method1_estimator_clock_prior": {"gnss_record_low_median_s": 0.0454, "imu_record_minus_header_low_median_s": 0.0121,
                                        "upper_bound_s": 0.0339, "drift_ppm": -8.3, "steps": [], "latency_g_assumed_range_s": [0.0, 0.15],
                                        "offset_s": -0.0411, "sigma_s": 0.075},
      "yaml": "sequence: SeqX\ntool_version: 1\nleap_s: 18\nsegments:\n  - t_from_gpst_s: 0\n"}
M2 = {"method2_inputs": {"trajectory": "/r/runs/p2b-f/seqx_s0_a/traj_W.tum", "kind": "tum", "traj_samples": 4000, "gnss_epochs": 1700,
                         "est_speed_std": 0.31},
      "method2_windows": [{"used": True}, {"used": True}, {"used": False}], "method2_status": "ran",
      "segments": [{"t_from_gpst_s": 0.0, "offset_s": -0.0321, "drift_ppm": 1.5, "t_ref_gpst_s": 1737451300.0, "sigma_toff_s": 0.0021,
                    "tdcp_paused": False, "method": "xcorr", "peak_corr": 0.97, "n_windows": 2}],
      "method3_dtw": {"offset_s": -0.0330, "mad_s": 0.004, "xcorr_offset_s": -0.0321, "consistent": True}, "warnings": [],
      "yaml": "sequence: SeqX\ntool_version: 1\nleap_s: 18\nsegments:\n  - t_from_gpst_s: 0\n    offset_s: -0.0321\n"}


class Compose(unittest.TestCase):
    def archive(self, d, with_m2):
        os.makedirs(os.path.join(d, "gt"))
        os.makedirs(os.path.join(d, "time_calib"))
        with open(os.path.join(d, "audit.md"), "w") as f:
            f.write(AUDIT_MD)
        with open(os.path.join(d, "gt", "gt_audit.md"), "w") as f:
            f.write(GT_MD)
        with open(os.path.join(d, "time_calib", "SeqX_record_time.json"), "w") as f:
            json.dump(M1, f)
        if with_m2:
            with open(os.path.join(d, "time_calib", "SeqX.json"), "w") as f:
                json.dump(M2, f)

    def test_composition(self):
        with tempfile.TemporaryDirectory() as d:
            self.archive(d, True)
            text = su.compose(d, "SeqX", notes="- provenance line")
        lines = text.split("\n")
        self.assertEqual(sum(1 for l in lines if l.startswith("# ")), 1)
        self.assertIn("## GT 审计(tools/gt_audit.py)", text)
        self.assertIn("### 检查表\n\n| P4 | ok |", text)                  # GT headings demoted
        self.assertLess(text.index("## 本轮说明"), text.index("## 检查表"))   # notes right after the title block
        self.assertIn("`/work/fastlivo2_gnss/data/m3dgr/bags/SeqX.bag`", text)
        self.assertIn("`/work/fastlivo2_gnss/runs/p2b-f/seqx_s0_a/traj_W.tum`", text)
        self.assertIn("| 0 | -32.10 ms | 1.5 |", text)                      # method-2 segment row
        self.assertIn("offset_s: -0.0321", text)
        self.assertIn("先验 -41.1 ms、σ 75 ms", text)

    def test_method2_window_table_and_signal(self):
        r = dict(M2)
        r["method2_inputs"] = dict(M2["method2_inputs"], signal="horizontal antenna speed ...", up_source="state_trace gravity",
                                   doppler_ls={"cn0_min_dbhz": 15.0, "el_min_deg": 10.0, "epochs_with_4_rows": 1729, "pass2_epochs": 1729,
                                               "rows_used_median": 18.0})
        r["method2_windows"] = [{"t_from": 100.0, "used": True, "peak": 0.979, "offset": 0.0726, "sigma_pw": 0.0483, "gnss_coverage": 1.0},
                                {"t_from": 160.0, "used": False, "gnss_coverage": 1.0, "reason": "insufficient excitation"}]
        text = su.method2_text(r)
        self.assertIn("| 0 | 是 | 0.979 | 72.6 | 48.3 | 100% |  |", text)
        self.assertIn("| 60 | 否 | — | — | — | 100% | insufficient excitation |", text)
        self.assertIn("第二遍解出 1729 个", text)

    def test_without_method2(self):
        with tempfile.TemporaryDirectory() as d:
            self.archive(d, False)
            text = su.compose(d, "SeqX")
        self.assertNotIn("方法②", text)
        self.assertIn("record_time", text)


if __name__ == "__main__":
    unittest.main()
