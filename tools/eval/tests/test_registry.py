# SPDX-License-Identifier: BSD-3-Clause
"""tools/eval/sequences.json and arms.json against the rulings they encode (A2, A26, A46) and the official
checksums (when a copy is reachable)."""
import os
import re
import unittest

import _path  # noqa: F401
import evalcommon as ec
import fetch_eval
import make_table

A46_EVAL = ("Varying-illu03", "Varying-illu04", "Varying-illu05", "Outdoor01", "Outdoor04", "Dark01", "Dark02",
            "GNSS-denial01", "GNSS-denial02")
DEV = ("Grass01", "Grass02", "Occlusion03", "Occlusion04", "Dynamic03", "Z-Rough-Road01")     # A2, A26, A69 item 3
A46_ARMS = ("U", "L-PVT", "T", "T-pr", "T-dop", "T-tdcp")


A80_DEV = ("Data01_20240707_100C_CAR_complex", "Data02_20240718_100C_CAR_complex", "HKisland_GNSS03")
A80_EVAL = ("Data01_20240719_100C_CAR_opensky", "Data00_20240703_100C_CAR_complex", "Data00_20240706_100C_CAR_complex")


class Registry(unittest.TestCase):
    def setUp(self):
        self.reg, self.all_seqs = ec.load_sequences()
        # the M3DGR entries (default dataset) carry the cloud / checksum fields; the A80 data have their own test below
        self.seqs = {s: i for s, i in self.all_seqs.items() if i.get("dataset", "m3dgr") == "m3dgr"}

    def test_every_a46_and_dev_sequence_is_registered_with_the_fl2g_class(self):
        for s in A46_EVAL:
            self.assertEqual(self.seqs[s]["class"], "evaluation", s)
            self.assertEqual(ec.seq_class(s), "evaluation", s)
        for s in DEV:
            self.assertEqual(self.seqs[s]["class"], "development", s)
            self.assertEqual(ec.seq_class(s), "development", s)
        self.assertEqual(set(self.seqs), set(A46_EVAL) | set(DEV))

    def test_a80_data_split_is_registered_and_agrees_with_fl2g(self):
        """AMENDMENTS A80 section 2 (written before any download): the split is fixed, evaluation days differ from the development days."""
        a80 = {s: i for s, i in self.all_seqs.items() if i.get("dataset", "m3dgr") != "m3dgr"}
        self.assertEqual(set(a80), set(A80_DEV) | set(A80_EVAL))
        for s in A80_DEV:
            self.assertEqual(a80[s]["class"], "development", s)
            self.assertEqual(ec.seq_class(s), "development", s)
        for s in A80_EVAL:
            self.assertEqual(a80[s]["class"], "evaluation", s)
            self.assertEqual(ec.seq_class(s), "evaluation", s)
        dev_days = {a80[s]["date_local"] for s in A80_DEV if a80[s]["dataset"] == "smartpnt_mini"}
        eval_days = {a80[s]["date_local"] for s in A80_EVAL}
        self.assertFalse(dev_days & eval_days)
        lidars = ec.load_lidars()[1]
        for s, i in a80.items():
            self.assertIn(i["lidar"], lidars, s)
            self.assertEqual(i["gt_kind"], "rtk", s)
            self.assertEqual(i["bag"], "%s/bags/%s.bag" % (lidars[i["lidar"]]["bag_dir"].split("/")[0], s), s)     # the profile's bag_dir
            self.assertEqual(i["gt_dir"], "audit/%s/gt" % s, s)
            self.assertIn(i["dataset"], ("smartpnt_mini", "mars_lvig"), s)
            self.assertIsInstance(i["duration_s"], int, s)
        for s in a80:
            if a80[s]["dataset"] == "smartpnt_mini":
                self.assertEqual(a80[s]["portal"], "00_MINI/" + s)
                self.assertRegex(a80[s]["mini_receiver"], r"^01_ROVE/0\d_mini/ROVE_0\d\.24o$")
        self.assertRegex(a80["HKisland_GNSS03"]["sha256"], r"^[0-9a-f]{64}$")

    def test_checksum_fields(self):
        keys = set()
        for s, i in self.seqs.items():
            self.assertRegex(i["sha256"], r"^[0-9A-F]{64}$", s)
            self.assertTrue(i["checksum_key"].endswith("\\" + s + ".bag"), s)
            keys.add(i["checksum_key"])
        self.assertEqual(len(keys), len(self.seqs))

    def test_cloud_layout(self):
        for s, i in self.seqs.items():
            self.assertIsNotNone(i["cloud"], s)
            if s == "GNSS-denial01":                           # separate share (README:399), imported 2026-09-26 (A68)
                self.assertEqual(i["cloud"], "/来自分享/GNSS-denial01")
                self.assertIn("alipan.com/s/", i["cloud_note"])
            else:
                self.assertTrue(i["cloud"].startswith("/来自分享/Dataset/"), s)
            if i["cloud_layout"] == "single":
                self.assertTrue(i["cloud"].endswith("/" + s + ".exe"), s)
            else:
                self.assertEqual(i["cloud_layout"], "parts", s)
                self.assertTrue(i["cloud"].endswith("/" + s), s)

    def test_gt_kinds_and_table2(self):
        for s, i in self.seqs.items():
            self.assertEqual(i["gt_kind"], "aruco" if s.startswith("GNSS-denial") else "rtk", s)
        cols = [s for s, i in self.seqs.items() if i.get("table2")]
        self.assertEqual(sorted(cols, key=lambda s: self.seqs[s]["table2"]["col"]), list(make_table.TABLE2_COLUMNS))
        for s in cols:
            self.assertEqual(self.seqs[s]["table2"]["group"], make_table.TABLE2_GROUP[s])

    def test_official_checksums_file_when_reachable(self):
        cands = [os.environ.get("FL2G_CHECKSUMS", ""), "/r/data/m3dgr/checksums.txt", "/work/fastlivo2_gnss/data/m3dgr/checksums.txt"]
        path = next((c for c in cands if c and os.path.isfile(c)), None)
        if not path:
            self.skipTest("no copy of the official checksums.txt on this machine")
        with open(path, encoding="utf-8-sig", errors="replace") as f:
            text = f.read()
        for s, i in self.seqs.items():
            self.assertEqual(fetch_eval.checksum_lookup(text, i["checksum_key"]), i["sha256"], s)
            for k, v in i.get("sha256_alt", {}).items():
                self.assertEqual(fetch_eval.checksum_lookup(text, k), v, s)


class Arms(unittest.TestCase):
    def test_a46_arms(self):
        allarms = ec.load_arms()
        # A58: development-set calibration arms (TEST_PLAN §3) are flagged and are never evaluation arms
        arms = {a: v for a, v in allarms.items() if not v.get("calibration")}
        self.assertEqual(tuple(arms), A46_ARMS)
        for a, v in allarms.items():
            if v.get("calibration"):
                self.assertTrue(os.path.isfile(os.path.join(_path.REPO, v["overlay"])), a)
                self.assertEqual(v["build"], "head", a)
        u = arms["U"]
        # A55: U runs on the HEAD build (the GNSS arms' binary) with gnss.enable false, not on a separate ref-R tree
        self.assertEqual((u["traj"], u["build"], u["overlay"], u["gnss_inputs"]), ("traj_W.tum", "head", "config/arms/U.yaml", False))
        self.assertEqual({arms[a]["build"] for a in arms}, {"head"})
        with open(os.path.join(_path.REPO, "config", "arms", "U.yaml")) as f:
            self.assertRegex(f.read(), r"(?m)^  enable: false$")
        for a in A46_ARMS[1:]:
            self.assertEqual(arms[a]["traj"], "traj_N.tum", a)
            self.assertEqual(arms[a]["build"], "head", a)
            self.assertTrue(arms[a]["gnss_inputs"], a)
            self.assertEqual(arms[a]["overlay"], "config/arms/%s.yaml" % a)
            self.assertTrue(re.match(r"^[A-Za-z0-9-]+$", a))

    def test_time_calib_attribute(self):
        # A81: original candidates and the two actual one-round recheck arms do not read the A18 calibration file.
        allarms = ec.load_arms()
        off = sorted(a for a, v in allarms.items() if v.get("time_calib") is False)
        self.assertEqual(off, ["LIVGO-literal", "LIVGO-literal-diag", "LIVGO-literal-paper", "LIVGO-literal-paper-diag",
                               "LIVGO-literal-recheck-mars-diag", "LIVGO-literal-recheck-smartpnt-diag"])
        for a, v in allarms.items():
            if "time_calib" in v:
                self.assertIsInstance(v["time_calib"], bool, a)
                self.assertTrue(v["gnss_inputs"], a)          # the attribute is about the GNSS inputs overlay
                self.assertTrue(v.get("calibration"), a)      # an evaluation arm keeps the default
        self.assertNotIn("time_calib", allarms["U"])

    def test_literal_diag_arms_differ_only_in_overlay_and_label(self):
        # A81: the diagnostic arm is the arm plus gnss_sats.csv; the registry entry carries no other difference
        allarms = ec.load_arms()
        rest = lambda v: {k: x for k, x in v.items() if k not in ("overlay", "label")}   # noqa: E731
        for base in ("LIVGO-literal", "LIVGO-literal-paper"):
            d, b = allarms[base + "-diag"], allarms[base]
            self.assertEqual(rest(d), rest(b), base)
            self.assertEqual(d["overlay"], "config/arms/%s-diag.yaml" % base)
            self.assertTrue(d["calibration"] and b["calibration"], base)


class Phase(unittest.TestCase):
    def test_guard(self):
        self.assertEqual(ec.check_phase("Grass01", False), "development")
        for s in ("Dark01", "Outdoor01_filtered", "SomethingElse"):
            with self.assertRaises(ec.PhaseError):
                ec.check_phase(s, False)
        self.assertEqual(ec.check_phase("Dark01", True), "evaluation")

    def test_seeds(self):
        self.assertEqual(ec.parse_seeds("0-4"), [0, 1, 2, 3, 4])
        self.assertEqual(ec.parse_seeds("3,1,1"), [1, 3])
        for bad in ("", "4-1", "a", "-1"):
            with self.assertRaises(ValueError):
                ec.parse_seeds(bad)


if __name__ == "__main__":
    unittest.main()
