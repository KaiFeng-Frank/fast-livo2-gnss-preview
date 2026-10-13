# SPDX-License-Identifier: BSD-3-Clause
"""A18 time-calibration file: writer output is inside the restricted subset the C++ TimeMap parses, values round
trip exactly, the reader rejects everything outside the subset, times are on the GTime::toSec origin (seconds since
1970 on the GPST scale; a 1980-origin value is refused), and the piecewise mapping gives hand-computed values."""
import math
import unittest

import _path  # noqa: F401
from fl2g import gnsstime
from fl2g import timecalib_yaml as ty


# GPST 2024-11-25 (week 2342, tow 130626) = 1732537026 s since 1970 (GTime::toSec); the same instant counted from
# 1980-01-06 would be 1416572226 s — the origin the A18 example value 1416000000 suggests
T0 = gnsstime.gpst(2342, 130626.0)
SEG1 = {"t_from_gpst_s": 0.0, "offset_s": -0.0387, "drift_ppm": 12.345678901234, "t_ref_gpst_s": 1732537000.0,
        "sigma_toff_s": 0.004, "tdcp_paused": False, "method": "xcorr", "peak_corr": 0.97, "window_scatter_ms": 2.1}
SEG2 = {"t_from_gpst_s": 1732537250.25, "offset_s": 0.0413, "drift_ppm": -3.0, "t_ref_gpst_s": 1732537300.0,
        "sigma_toff_s": 0.05, "tdcp_paused": True, "method": "config"}


class TimeCalibYaml(unittest.TestCase):
    def test_round_trip_exact(self):
        text = ty.dumps("Occlusion03", 18, [SEG2, SEG1])        # unsorted input is sorted by t_from
        d = ty.loads(text)
        self.assertEqual(list(d.keys()), ["sequence", "tool_version", "leap_s", "segments"])
        self.assertEqual(d["sequence"], "Occlusion03")
        self.assertEqual(d["leap_s"], 18)
        s1, s2 = d["segments"]
        for k in ("offset_s", "drift_ppm", "t_ref_gpst_s", "sigma_toff_s"):
            self.assertEqual(s1[k], SEG1[k], k)                  # repr round trip is exact (bit for bit)
            self.assertEqual(s2[k], SEG2[k], k)
        self.assertEqual(s1["t_from_gpst_s"], 0)                # first segment written as 0 (= -inf for the core)
        self.assertEqual(s2["t_from_gpst_s"], SEG2["t_from_gpst_s"])
        self.assertIs(s1["tdcp_paused"], False)
        self.assertIs(s2["tdcp_paused"], True)

    def test_layout_is_the_restricted_subset(self):
        text = ty.dumps("Dark01", 18, [SEG1])
        lines = text.rstrip("\n").split("\n")
        self.assertEqual(lines[:4], ["sequence: Dark01", "tool_version: 1", "leap_s: 18", "segments:"])
        self.assertTrue(lines[4].startswith("  - t_from_gpst_s: "))
        keys = [ln.strip().lstrip("- ").split(":")[0] for ln in lines[4:]]
        self.assertEqual(tuple(keys[:6]), ty.REQUIRED_SEGMENT_KEYS)   # the six keys the core needs, in order
        for ln in lines[5:]:
            self.assertTrue(ln.startswith("    ") and not ln.startswith("     "), ln)
        for bad in ("#", "{", "[", "&", "*", "'", '"', "\t"):
            self.assertNotIn(bad, text)
        for ln in lines:
            v = ln.split(": ", 1)[1] if ": " in ln else ""
            if v and v not in ("true", "false") and not v[0].isalpha():
                self.assertTrue(math.isfinite(float(v)), ln)             # strtod-parsable numbers

    def test_reader_rejects_outside_subset(self):
        good = ty.dumps("X", 18, [SEG1])
        bad_texts = [
            good.replace("segments:", "segments: []"),                         # flow syntax
            good + "# comment\n",                                              # comments
            good.replace("  - t_from_gpst_s", "\t- t_from_gpst_s"),            # tab
            good.replace("sequence: X\ntool_version: 1", "tool_version: 1\nsequence: X"),  # top-level order
            good.replace("    offset_s", "    offset"),                        # required key renamed
            good.replace("tdcp_paused: false", "tdcp_paused: 0"),              # not a boolean
            good.replace("sigma_toff_s: 0.004", "sigma_toff_s: nan"),          # non-finite
        ]
        for t in bad_texts:
            with self.assertRaises(ValueError, msg=t):
                ty.loads(t)
        two = ty.dumps("X", 18, [SEG1, SEG2])
        swapped = two.replace("t_from_gpst_s: 1732537250.25", "t_from_gpst_s: -5.0")
        ty.loads(swapped)  # second segment at -5 is still > -inf (first ignored)
        three = ty.dumps("X", 18, [SEG1, SEG2, dict(SEG2, t_from_gpst_s=1732537400.0)])
        with self.assertRaises(ValueError):
            ty.loads(three.replace("t_from_gpst_s: 1732537400.0", "t_from_gpst_s: 1732537100.0"))  # unsorted

    def test_writer_refuses_unsafe_values(self):
        with self.assertRaises(ValueError):
            ty.dumps("X", 18, [dict(SEG1, offset_s=float("inf"))])
        with self.assertRaises(ValueError):
            ty.dumps("X", 18, [dict(SEG1, method="a b")])
        with self.assertRaises(ValueError):
            ty.dumps("X", 18, [])
        self.assertIn("sequence: GNSS-denial01_x", ty.dumps("GNSS-denial01 x", 18, [SEG1]))   # sanitised name

    def test_time_origin_and_span_check(self):
        self.assertEqual(T0, 1732537026.0)                             # GTime::toSec origin (1970, GPST scale)
        span = (T0, T0 + 400.0)
        ty.dumps("X", 18, [SEG1, SEG2], span=span)                     # Unix-origin values pass
        bad = dict(SEG1, t_ref_gpst_s=SEG1["t_ref_gpst_s"] - gnsstime.GPST0_UNIX)      # the same instant from 1980
        with self.assertRaises(ValueError):
            ty.dumps("X", 18, [bad, SEG2], span=span)
        with self.assertRaises(ValueError):
            ty.dumps("X", 18, [SEG1, dict(SEG2, t_from_gpst_s=SEG2["t_from_gpst_s"] - gnsstime.GPST0_UNIX)], span=span)
        # t_ref does not matter without drift (the config fallback writes t_ref 0 with drift 0)
        ty.dumps("X", 18, [dict(SEG1, drift_ppm=0.0, t_ref_gpst_s=0.0)], span=span)

    def test_mapping_hand_computed(self):
        # t_est = t - leap + offset + drift*1e-6*(t - t_ref) of the segment containing t (first segment from -inf);
        # expected values worked out by hand from SEG1 (drift 12.5 ppm here) and SEG2
        d = ty.loads(ty.dumps("X", 18, [dict(SEG1, drift_ppm=12.5), SEG2]))
        cases = ((1732537100.0, 1732537081.96255),          # SEG1: -18 - 0.0387 + 12.5e-6*100
                 (1732537250.0, 1732537231.964425),         # SEG1 up to t_from of SEG2 (exclusive): + 12.5e-6*250
                 (1732537260.0, 1732537242.04142),          # SEG2: -18 + 0.0413 + (-3e-6)*(-40)
                 (1732536000.0, 1732535981.9488))           # before the data: still SEG1, + 12.5e-6*(-1000)
        for t, expected in cases:
            self.assertAlmostEqual(ty.evaluate(d, t), expected, delta=1e-6, msg=t)
        self.assertAlmostEqual(ty.evaluate(d, 1732537250.25), 1732537232.29144925, delta=1e-6)   # SEG2 from its t_from on: + 3e-6*49.75


if __name__ == "__main__":
    unittest.main()
