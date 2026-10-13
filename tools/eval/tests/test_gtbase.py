# SPDX-License-Identifier: BSD-3-Clause
"""gtbase.py (AMENDMENTS A54): the gt_timebase.yaml contract (method, shift, optional shifted file), the trust_mask.csv
ATE-mask column, masked stretches, legacy pre-A54 audits, refusals."""
import json
import os
import shutil
import tempfile
import unittest
from unittest import mock

import numpy as np

import _path  # noqa: F401
import evalcommon as ec
import gtbase as gb

INFO = {"gt_kind": "rtk"}


def gt_rows(n=200, t0=1000.0, dt=0.2):
    t = t0 + dt * np.arange(n)
    a = np.zeros((n, 8))
    a[:, 0], a[:, 1], a[:, 2], a[:, 7] = t, np.arange(n) * 0.2, np.sin(np.arange(n) * 0.05), 1.0
    return a


class Audit:
    def __init__(self):
        self.root = tempfile.mkdtemp()
        self.d = os.path.join(self.root, "audit", "Grass02", "gt")
        os.makedirs(self.d)
        self.clean = gt_rows()
        with open(os.path.join(self.d, "gt_clean.tum"), "w") as f:
            for r in self.clean:
                f.write("%.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n" % tuple(r))
        self.clean = ec.read_tum(os.path.join(self.d, "gt_clean.tum"))        # the file's (rounded) values

    def mask(self, excluded, column="ate_exclude(bits 8|32|64)"):
        """trust_mask.csv as tools/gt_audit.py (P4-TIME, A54) writes it: the ATE column header carries its bits."""
        with open(os.path.join(self.d, "trust_mask.csv"), "w") as f:
            f.write("t,trusted,trusted_gt_criteria_only,\"reason_bits(1=h_acc>1m,2=num_sv<8,4=speed_spike,8=gap_or_stall,"
                    "64=rtk_jump_plateau: each within +-2 s;16=no_pvt_within_0.2s;32=outside_the_bag_record_span)\",%s\n" % column)
            for t, x in zip(self.clean[:, 0], excluded):
                f.write("%.6f,1,1,%d,%d\n" % (t, 64 if x else 0, int(x)))

    def timebase(self, **kv):
        """gt_timebase.yaml as tools/gt_audit.py's write_timebase_yaml writes it (JSON-quoted strings, repr floats, null)."""
        with open(os.path.join(self.d, "gt_timebase.yaml"), "w") as f:
            f.write("# GT stamp -> estimator clock (Avia IMU/LiDAR header) time base of Grass02, tools/gt_audit.py (tool_version 3)\n"
                    "# gt_audited.tum = gt_clean.tum with applied_shift_s added to every stamp (AMENDMENTS A54)\n")
            for k, v in kv.items():
                if v is None:
                    f.write("%s: null\n" % k)
                elif isinstance(v, str):
                    f.write("%s: %s\n" % (k, json.dumps(v, ensure_ascii=False)))
                else:
                    f.write("%s: %r\n" % (k, v))

    def audited(self, shift):
        a = self.clean.copy()
        a[:, 0] += shift
        ec.write_tum(os.path.join(self.d, "gt_audited.tum"), a, digits=6)

    def legacy(self, shift=None):
        out = {"gt_clean": "gt_clean.tum", "gt_audited": None, "time_shift_applied_s": None, "gt_audited_withheld": "P4 FAIL"}
        if shift is not None:
            a = self.clean.copy()
            a[:, 0] += shift
            ec.write_tum(os.path.join(self.d, "gt_audited.tum"), a, digits=6)
            out = {"gt_clean": "gt_clean.tum", "gt_audited": "gt_audited.tum", "time_shift_applied_s": shift}
        with open(os.path.join(self.d, "gt_audit.json"), "w") as f:
            json.dump({"tool_version": 2, "checks": [{"id": "P4", "verdict": "PASS" if shift is not None else "FAIL"}], "outputs": out}, f)

    def load(self):
        return gb.load_gt(self.root, "Grass02", INFO)

    def close(self):
        shutil.rmtree(self.root)


class Stretches(unittest.TestCase):
    def test_runs_and_mask_geometry(self):
        t = np.arange(10.0)
        ex = np.array([0, 1, 1, 0, 0, 1, 0, 1, 1, 1], dtype=bool)
        iv = gb.stretches_from_rows(t, ex)
        self.assertEqual(iv.tolist(), [[1.0, 2.0], [5.0, 5.0], [7.0, 9.0]])
        w = np.minimum(np.diff(t, append=t[-1]), 0.5)
        m = gb.Mask(iv, status="applied", rows_t=t[ex], rows_s=w[ex])
        self.assertEqual(m.inside(np.array([0.5, 1.0, 1.5, 2.0, 2.01, 5.0, 6.0, 8.0, 9.5])).tolist(),
                         [False, True, True, True, False, True, False, True, False])
        # gt_audit's accounting: each excluded row stands for min(dt to the next row, 0.5 s); the last row for 0
        self.assertAlmostEqual(m.seconds_in(0.0, 10.0), 0.5 * 5)
        self.assertAlmostEqual(m.seconds_in(1.5, 8.0), 0.5 * 4)                    # rows 2, 5, 7, 8
        self.assertEqual(m.stretches_in(3.0, 6.0), 1)
        self.assertEqual(gb.Mask().inside(np.array([1.0])).tolist(), [False])
        self.assertEqual(gb.stretches_from_rows(t, np.zeros(10, bool)).shape, (0, 2))

    def test_flat_yaml_without_pyyaml(self):
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, "x.yaml")
            with open(p, "w") as f:
                f.write("# c\nsequence: \"Grass02\"\nmethod: record_time   # comment\napplied_shift_s: -0.0059\n"
                        "gt_file: 'gt_audited.tum'\nconfidence: low\nok: true\nnothing: null\n"
                        "flag: \"P4 FAIL: record-time value (A54) # not a comment \\\"q\\\"\"\n"
                        "rule: \"applied shift = value if |value| > 20 ms else 0\"\n")
            want = {"sequence": "Grass02", "method": "record_time", "applied_shift_s": -0.0059, "gt_file": "gt_audited.tum",
                    "confidence": "low", "ok": True, "nothing": None, "flag": 'P4 FAIL: record-time value (A54) # not a comment "q"',
                    "rule": "applied shift = value if |value| > 20 ms else 0"}
            with mock.patch.dict("sys.modules", {"yaml": None}):
                self.assertEqual(gb.read_flat_yaml(p), want)
            try:
                import yaml  # noqa: F401
                self.assertEqual(gb.read_flat_yaml(p), want)            # the PyYAML path reads the same file identically
            except ImportError:
                pass
            with open(p, "a") as f:
                f.write("nested:\n  a: 1\n")
            with mock.patch.dict("sys.modules", {"yaml": None}):
                with self.assertRaises(ValueError):
                    gb.read_flat_yaml(p)


class LoadGt(unittest.TestCase):
    def setUp(self):
        self.a = Audit()

    def tearDown(self):
        self.a.close()

    def test_record_time_with_mask(self):
        ex = np.zeros(200, dtype=bool)
        ex[50:60] = True
        self.a.mask(ex)
        self.a.timebase(sequence="Grass02", method="record_time", offset_s=-0.0059, applied_shift_s=0.0, uncertainty_s=0.05,
                        confidence="low", flag="P4 FAIL: record-time value (A54)", p4_verdict="FAIL", chain_s=None)
        gt, prov, mask = self.a.load()
        self.assertTrue(np.array_equal(gt, self.a.clean))                       # |offset| <= 20 ms: not compensated
        self.assertEqual((prov["method"], prov["method_label"], prov["source"]), ("record_time", "记录时间旁证", "gt_timebase.yaml"))
        self.assertEqual((prov["estimate_s"], prov["uncertainty_s"], prov["confidence"], prov["p4_verdict"]), (-0.0059, 0.05, "low", "FAIL"))
        self.a.timebase(sequence="Grass02", method="record_time", offset_s=-0.1117, applied_shift_s=-0.1117)
        gt, prov, mask = self.a.load()
        self.assertTrue(np.allclose(gt[:, 0], self.a.clean[:, 0] - 0.1117, atol=1e-9, rtol=0))
        self.assertTrue(np.array_equal(gt[:, 1:], self.a.clean[:, 1:]))
        self.assertEqual(mask.status, "applied")
        self.assertEqual(mask.iv.shape, (1, 2))
        self.assertAlmostEqual(mask.iv[0, 0], self.a.clean[50, 0] - 0.1117, places=6)
        self.assertAlmostEqual(mask.seconds_in(0, 1e12), 10 * 0.2, places=6)          # 10 rows x 0.2 s (gt_audit's count)
        self.assertEqual(mask.record()["rows_excluded"], 10)
        self.assertEqual(mask.inside(gt[:, 0]).sum(), 10)

    def test_chain_with_the_audited_file_is_checked(self):
        self.a.mask(np.zeros(200, bool))
        self.a.audited(0.107)                                  # P4-TIME: gt_audited.tum is always written
        self.a.timebase(sequence="Grass02", method="chain", offset_s=0.1071, applied_shift_s=0.107, confidence="high")
        gt, prov, mask = self.a.load()
        self.assertEqual((prov["method"], prov["method_raw"], prov["gt_file"]), ("p4", "chain", "gt_audited.tum"))
        self.assertEqual(mask.iv.shape, (0, 2))
        self.a.timebase(sequence="Grass02", method="chain", applied_shift_s=0.100)       # a stale gt_audited.tum
        with self.assertRaises(ec.Blocked) as cm:
            self.a.load()
        self.assertIn("stale or edited", str(cm.exception))
        self.a.timebase(sequence="Grass02", method="chain", time_shift_applied_s=0.107)  # the alias key
        self.assertEqual(self.a.load()[1]["time_shift_applied_s"], 0.107)
        self.a.timebase(sequence="Grass02", method="chain", time_shift_applied_s=0.107, applied_shift_s=0.2)
        with self.assertRaises(ec.Blocked):
            self.a.load()

    def test_none_is_zero_and_flagged(self):
        self.a.mask(np.zeros(200, bool))
        self.a.timebase(sequence="Grass02", method="none", offset_s=0.0, applied_shift_s=0.0, confidence="none",
                        flag="no GT time-base estimate: 0 used (A54)")
        gt, prov, _ = self.a.load()
        self.assertTrue(np.array_equal(gt, self.a.clean))
        self.assertEqual((prov["method_label"], prov["flag"]), ("未定(0)", "no GT time-base estimate: 0 used (A54)"))
        self.a.timebase(sequence="Grass02", method="none", applied_shift_s=0.02)
        with self.assertRaises(ec.Blocked):
            self.a.load()

    def test_gpst_and_doppler_methods_of_the_a80_data(self):
        self.a.mask(np.zeros(200, bool))
        self.a.timebase(sequence="Grass02", method="gpst", offset_s=0.0, applied_shift_s=0.0, confidence="high")
        gt, prov, _ = self.a.load()
        self.assertEqual((prov["method"], prov["method_label"]), ("gpst", "GPST 同钟"))
        self.assertTrue(np.array_equal(gt, self.a.clean))
        self.a.timebase(sequence="Grass02", method="gpst", applied_shift_s=0.05)            # a same-clock GT needs no shift
        with self.assertRaises(ec.Blocked):
            self.a.load()
        self.a.audited(0.060)
        self.a.timebase(sequence="Grass02", method="xcorr", offset_s=0.0601, applied_shift_s=0.060, confidence="medium")
        self.assertEqual(self.a.load()[1]["method"], "xcorr")

    def test_refusals(self):
        self.a.mask(np.zeros(200, bool))
        for kv, why in (({"sequence": "Grass01", "method": "chain", "applied_shift_s": 0.0}, "for sequence"),
                        ({"sequence": "Grass02", "method": "chain_guess", "applied_shift_s": 0.0}, "method"),
                        ({"sequence": "Grass02", "method": "chain"}, "applied_shift_s"),
                        ({"sequence": "Grass02", "method": "chain", "applied_shift_s": None}, "applied_shift_s"),
                        ({"sequence": "Grass02", "method": "chain", "applied_shift_s": "abc"}, "applied_shift_s")):
            self.a.timebase(**kv)
            with self.assertRaises(ec.Blocked) as cm:
                self.a.load()
            self.assertIn(why, str(cm.exception))
        # the A54 mask column is required once the A54 record exists; rows must match gt_clean
        self.a.timebase(sequence="Grass02", method="chain", applied_shift_s=0.0)
        self.a.mask(np.zeros(200, bool), column="something_else")
        with self.assertRaises(ec.Blocked) as cm:
            self.a.load()
        self.assertIn("ate_exclude", str(cm.exception))
        with open(os.path.join(self.a.d, "trust_mask.csv"), "w") as f:
            f.write("t,ate_exclude\n1000.000000,0\n")
        with self.assertRaises(ec.Blocked) as cm:
            self.a.load()
        self.assertIn("do not match", str(cm.exception))
        os.remove(os.path.join(self.a.d, "trust_mask.csv"))
        with self.assertRaises(ec.Blocked):
            self.a.load()

    def test_mask_file_and_column_from_the_record(self):
        ex = np.zeros(200, dtype=bool)
        ex[10:12] = True
        self.a.mask(ex, column="rtk_jump")
        os.rename(os.path.join(self.a.d, "trust_mask.csv"), os.path.join(self.a.d, "ate_mask.csv"))
        self.a.timebase(sequence="Grass02", method="chain", applied_shift_s=0.0, ate_mask_file="ate_mask.csv", ate_mask_column="rtk_jump")
        _, _, mask = self.a.load()
        self.assertEqual((mask.record()["file"], mask.record()["column"], mask.record()["rows_excluded"]), ("ate_mask.csv", "rtk_jump", 2))

    def test_legacy_audits(self):
        self.a.legacy(shift=0.05)
        gt, prov, mask = self.a.load()
        self.assertTrue(prov["source"].startswith("legacy"))
        self.assertEqual(prov["method"], "p4")
        self.assertTrue(mask.status.startswith("absent"))
        self.assertTrue(np.allclose(gt[:, 0], self.a.clean[:, 0] + 0.05, atol=1e-9, rtol=0))
        self.a.legacy(shift=None)
        with self.assertRaises(ec.Blocked) as cm:
            self.a.load()
        self.assertIn("A54", str(cm.exception))

    def test_aruco_and_missing(self):
        with self.assertRaises(ec.Blocked):
            gb.load_gt(self.a.root, "GNSS-denial02", {"gt_kind": "aruco"})
        with self.assertRaises(ec.Blocked):
            gb.load_gt(self.a.root, "Grass01", INFO)


if __name__ == "__main__":
    unittest.main()
