# SPDX-License-Identifier: BSD-3-Clause
"""make_table.py: the hard-coded FAST-LIVGO Table II values against the paper's verbatim text, the main table layout
(E1, A52), the GT column (A54 time base + masked time), the supplementary tables (post30, E4) and the markers."""
import json
import os
import re
import tempfile
import unittest

import _path  # noqa: F401
import make_table as mt

LIVGO_TXT = "/workspace/fastlivo2_gnss_scout_20260924/livgo.txt"
# the values the task statement gives (FAST-LIVO2 / FAST-LIVGO columns for Varying-illu03/04/05, Outdoor01/04, Dark01/02)
TASK_LIVO2 = (1.44, 0.35, 0.16, 0.27, 0.25, 0.31, 0.18)
TASK_LIVGO = (0.23, 0.19, 0.09, 0.11, 0.13, 0.34, 0.27)


def parse_table2(text):
    """Header columns and the numeric rows of the flattened arXiv-HTML table text."""
    head = re.search(r"GNSS Denied Scenarios ((?:[A-Za-z-]+\d\d ){7})", text)
    cols = tuple(head.group(1).split())
    rows = {}
    for name in ("FAST-LIVO2", "Ours"):
        m = re.search(r"(?:^|\s)%s ((?:\d+\.\d\d ?){7})" % re.escape(name), text)
        rows[name] = tuple(float(x) for x in m.group(1).split())
    return cols, rows


class Table2(unittest.TestCase):
    def test_transcription_matches_verbatim_text(self):
        cols, rows = parse_table2(mt.TABLE2_VERBATIM)
        self.assertEqual(cols, mt.TABLE2_COLUMNS)
        self.assertEqual(rows["FAST-LIVO2"], mt.TABLE2["FAST-LIVO2"])
        self.assertEqual(rows["Ours"], mt.TABLE2["FAST-LIVGO"])

    def test_task_values(self):
        self.assertEqual(mt.TABLE2["FAST-LIVO2"], TASK_LIVO2)
        self.assertEqual(mt.TABLE2["FAST-LIVGO"], TASK_LIVGO)

    def test_verbatim_text_is_in_the_paper_copy(self):
        if not os.path.isfile(LIVGO_TXT):
            self.skipTest("paper copy %s not on this machine" % LIVGO_TXT)
        with open(LIVGO_TXT, encoding="utf-8", errors="replace") as f:
            paper = re.sub(r"\s+", " ", f.read())
        self.assertIn(mt.TABLE2_VERBATIM, paper)
        self.assertIn("arXiv:2606.19190v2", paper)
        cols, rows = parse_table2(paper[paper.index("TABLE II:"):])
        self.assertEqual(cols, mt.TABLE2_COLUMNS)
        self.assertEqual(rows["FAST-LIVO2"], mt.TABLE2["FAST-LIVO2"])
        self.assertEqual(rows["Ours"], mt.TABLE2["FAST-LIVGO"])


def read(path):
    with open(path, encoding="utf-8") as f:
        return f.read()


def write_ate(d, cells, blocked=None, sequences=None, name="ate.json"):
    p = os.path.join(d, name)
    with open(p, "w") as f:
        json.dump({"generated_utc": "2026-09-25T00:00:00Z", "metric": {"cmd": "evo_ape tum ... -a --t_max_diff 0.05"},
                   "matrices": [{"name": "m", "source_id": "abc"}], "cells": cells, "blocked": blocked or {},
                   "sequences": sequences or {}}, f)
    return p


def seq_rec(method="p4", shift=0.1071, masked=12.4, mask_status="applied", sha="g" * 64, estimate=None):
    return {"gt": {"method": method, "time_shift_applied_s": shift, "estimate_s": estimate, "source": "gt_timebase.yaml",
                   "gt_clean_sha256": sha},
            "mask": {"status": mask_status}, "e1_spans": [{"span": [0.0, 100.0], "length_s": 100.0, "masked_s": masked}],
            "post30_spans": [{"span": [60.0, 100.0], "length_s": 40.0, "masked_s": 2.5}]}


def full_cell(seq, arm, med, a55=False):
    blk = {"n_ok": 5, "n_runs": 5, "median_m": med, "min_m": med - 0.1, "max_m": med + 0.1, "spans": [[60.0, 100.0]]}
    return {"seq": seq, "arm": arm, "n_ok": 5, "n_runs": 5, "median_m": med, "min_m": med - 0.1, "max_m": med + 0.1,
            "post30": dict(blk, median_m=med / 2, min_m=med / 2 - 0.05, max_m=med / 2 + 0.05), "e4": dict(blk), "a55_flag": a55}


class Layout(unittest.TestCase):
    def test_main_without_results(self):
        with tempfile.TemporaryDirectory() as d:
            md = os.path.join(d, "t.md")
            self.assertEqual(mt.main(["--ate", write_ate(d, []), "--out-md", md, "--out-csv", os.path.join(d, "t.csv")]), 0)
            text = read(md)
        rows = [ln for ln in text.splitlines() if ln.startswith("| ") and not ln.startswith("| 序列")]
        self.assertEqual([r.split("|")[1].strip().rstrip("ᵈ") for r in rows], list(mt.TABLE2_COLUMNS))
        self.assertEqual(len(rows[0].split("|")), 10)                  # 8 columns, no difference/ratio column
        self.assertIn("| Outdoor01ᵈ |", text)
        self.assertIn("| 1.44 | 待跑 | 0.23 | 待跑 | 待跑 | — |", text)
        for needle in ("arXiv:2606.19190v2", "A27", "986b4b1", "48 px", "80 px", "262.5 s", "ArUco", "--t_max_diff 0.05", "A30",
                       "A52", "A54", "A55", "T_{N←W}", "A61", "traj_N_odom"):
            self.assertIn(needle, text)
        self.assertIn("GNSS-denial01", text)

    def test_dev_rehearsal_row(self):
        cell = {"seq": "Grass01", "arm": "U", "n_ok": 5, "n_runs": 5, "median_m": 0.3456, "min_m": 0.3012, "max_m": 0.4101}
        with tempfile.TemporaryDirectory() as d:
            md = os.path.join(d, "t.md")
            mt.main(["--ate", write_ate(d, [cell]), "--mode", "dev", "--digits", "3", "--out-md", md])
            text = read(md)
        self.assertIn("开发集演练版", text)
        self.assertIn("| Grass01 | 开发集 | — | 0.346 [0.301, 0.410] | — | 待跑 | 待跑 | — |", text)
        rows = [ln for ln in text.split("## 附表")[0].splitlines() if ln.startswith("| ") and not ln.startswith("| 序列")]
        self.assertEqual(len(rows), 1)                                  # evaluation sequences never appear in dev mode

    def test_gt_column_and_supplementary_tables(self):
        cells = [full_cell("Grass01", "U", 0.5), full_cell("Grass01", "T", 0.9, a55=True), full_cell("Grass02", "U", 0.3),
                 full_cell("Occlusion03", "U", 0.2)]
        occ = seq_rec(method="none", shift=0.0, mask_status="absent (audit predates A54)")
        occ["post30_spans"] = []
        seqs = {"Grass01": seq_rec(), "Grass02": seq_rec(method="record_time", shift=0.0, estimate=-0.0059, masked=0.0),
                "Occlusion03": occ}
        with tempfile.TemporaryDirectory() as d:
            p = write_ate(d, cells, sequences=seqs)
            md, cs = os.path.join(d, "t.md"), os.path.join(d, "s.csv")
            self.assertEqual(mt.main(["--ate", p, "--mode", "dev", "--out-md", md, "--out-csv", os.path.join(d, "t.csv"),
                                      "--out-csv-supp", cs]), 0)
            text = read(md)
            with open(cs) as f:
                supp = f.read().splitlines()
            with open(os.path.join(d, "t.csv")) as f:
                main_csv = f.read().splitlines()
        self.assertIn("| Grass01 | 开发集 | — | 0.50 [0.40, 0.60] | — | 0.90 [0.80, 1.00] † | — | P4 +107.1 ms;掩掉 12.4 s |", text)
        self.assertIn("| Grass02 | 开发集 | — | 0.30 [0.20, 0.40] | — | 待跑 | 待跑 | 记录时间旁证 -5.9 ms(≤ 20 ms,不补偿);掩掉 0.0 s |", text)
        self.assertIn("| Occlusion03 | 开发集 | — | 0.20 [0.10, 0.30] | — | 待跑 | 待跑 | 未定,按 0ᶠ;无掩码(旧审计) |", text)
        self.assertIn("## 附表 1:激活 30 s 后", text)
        self.assertIn("| 序列 | 时段 s(掩掉 s) | U | T |", text)
        self.assertIn("| Grass01 | 40.0 (2.5) | 0.25 [0.20, 0.30] | 0.45 [0.40, 0.50] † |", text)
        # a sequence where no GNSS run activated has no such span; a GNSS cell whose runs never activated says so
        self.assertIn("| Occlusion03 | — | 无 GNSS 激活 | — |", text)
        self.assertIn("## 附表 2:E4", text)
        self.assertIn("| Grass01 | 0.50 [0.40, 0.60] | 0.90 [0.80, 1.00] † |", text)
        self.assertIn("| Grass02 | 0.30 [0.20, 0.40] | — |", text)
        self.assertEqual(supp[0], "seq,metric,arm,median_m,min_m,max_m,n_ok,n_runs,spans")
        self.assertIn("Grass01,post30,T,0.45,0.4,0.5,5,5,60.000000-100.000000", supp)
        self.assertEqual(len(supp), 1 + 2 * 4)
        self.assertTrue(main_csv[1].startswith("Grass01,开发集,,0.5,0.4,0.6,5,5,,0.9,0.8,1.0,5,5,p4,,0.1071,gt_timebase.yaml,applied,12.400"))
        self.assertTrue(main_csv[2].endswith(",,,,,,record_time,-0.0059,0.0,gt_timebase.yaml,applied,0.000,,,,,"))   # + 4 empty t_odo

    def test_odometry_output_column(self):
        """A61 F13: the T arm's e1o block (traj_N_odom) in its own column, CSV columns appended, supplementary rows."""
        t = full_cell("Grass01", "T", 0.9)
        t["e1o"] = {"n_ok": 4, "n_runs": 5, "median_m": 0.12, "min_m": 0.1, "max_m": 0.2, "spans": [[0.0, 100.0]]}
        cells = [full_cell("Grass01", "U", 0.5), t]
        with tempfile.TemporaryDirectory() as d:
            p = write_ate(d, cells, sequences={"Grass01": seq_rec()})
            md, cs, ss = os.path.join(d, "t.md"), os.path.join(d, "t.csv"), os.path.join(d, "s.csv")
            self.assertEqual(mt.main(["--ate", p, "--mode", "dev", "--out-md", md, "--out-csv", cs, "--out-csv-supp", ss]), 0)
            text = read(md)
            with open(cs) as f:
                main_csv = f.read().splitlines()
            with open(ss) as f:
                supp = f.read().splitlines()
        self.assertIn("| Grass01 | 开发集 | — | 0.50 [0.40, 0.60] | — | 0.90 [0.80, 1.00] | 0.12 [0.10, 0.20] (n=4/5) | P4 +107.1 ms;掩掉 12.4 s |",
                      text)
        self.assertIn("我们的紧耦合·里程计输出(T,traj_N_odom)ᵍ", text)
        self.assertEqual(main_csv[0].split(",")[-4:], ["t_odo_median_m", "t_odo_min_m", "t_odo_max_m", "t_odo_n_ok"])
        self.assertTrue(main_csv[1].endswith(",0.12,0.1,0.2,4"))
        self.assertIn("Grass01,e1o,T,0.12,0.1,0.2,4,5,0.000000-100.000000", supp)
        self.assertFalse([x for x in supp if x.startswith("Grass01,e1o,U,")])          # U has no e1o block here

    def test_post30_never_activated(self):
        c = dict(full_cell("Grass02", "T", 0.9), gnss_arm=True)
        c["post30"] = {"n_ok": 0, "n_runs": 5, "n_never_activated": 5, "median_m": None, "min_m": None, "max_m": None}
        self.assertEqual(mt.post30_text(c, seq_rec(), 2), "未激活(0/5)")
        c["post30"]["n_never_activated"] = 2
        self.assertEqual(mt.post30_text(c, seq_rec(), 2), "无有效运行(0/5)")
        self.assertEqual(mt.post30_text(None, seq_rec(), 2), "—")

    def test_two_inputs_must_share_the_gt(self):
        with tempfile.TemporaryDirectory() as d:
            a = write_ate(d, [full_cell("Grass01", "U", 0.5)], sequences={"Grass01": seq_rec()}, name="a.json")
            b = write_ate(d, [full_cell("Grass01", "T", 0.5)], sequences={"Grass01": seq_rec(sha="h" * 64)}, name="b.json")
            with self.assertRaises(ValueError):
                mt.load_cells([a, b])

    def test_partial_cell_and_blocked(self):
        cell = {"seq": "Grass01", "arm": "U", "n_ok": 4, "n_runs": 5, "median_m": 1.0, "min_m": 0.5, "max_m": 2.0}
        self.assertEqual(mt.fmt_cell(cell, 2, "x"), "1.00 [0.50, 2.00] (n=4/5)")
        self.assertEqual(mt.fmt_cell(None, 2, "待跑"), "待跑")
        dup = dict(cell, n_ok=5, identical_seed_groups=[[0, 1]], n_distinct=4)
        self.assertEqual(mt.fmt_cell(dup, 2, "x"), "1.00 [0.50, 2.00] ‡4")
        self.assertEqual(mt.fmt_cell({"n_ok": 0, "n_runs": 5}, 2, "x"), "无有效运行(0/5)")
        with tempfile.TemporaryDirectory() as d:
            p = write_ate(d, [], {"Dark01": "Dark01: blocked: A2"})
            md = os.path.join(d, "t.md")
            mt.main(["--ate", p, "--out-md", md])
            self.assertIn("| Dark01 | GNSS Denied | 0.31 | 阻塞:blocked: A2 | 0.34 |", read(md))

    def test_duplicate_cells_rejected(self):
        c = {"seq": "Grass01", "arm": "U", "n_ok": 1, "n_runs": 1, "median_m": 1, "min_m": 1, "max_m": 1}
        with tempfile.TemporaryDirectory() as d:
            a = write_ate(d, [c])
            b = os.path.join(d, "b.json")
            os.replace(a, b)
            a = write_ate(d, [c])
            with self.assertRaises(ValueError):
                mt.load_cells([a, b])


if __name__ == "__main__":
    unittest.main()
