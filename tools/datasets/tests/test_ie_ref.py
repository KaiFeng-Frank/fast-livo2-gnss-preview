# SPDX-License-Identifier: BSD-3-Clause
"""dscommon.ie_ref and smartpnt.gt_convert: IE .ref parsing, attitude conventions, lever identification, GT conversion."""
import csv
import json
import os
import tempfile
import unittest

import numpy as np

import _path  # noqa: F401
from dscommon import ie_ref
from fl2g import geodesy, gnsstime
from smartpnt import gt_convert

COLS = ("Week GPSTime Latitude Longitude H-Ell Q AmbStatus Date GPSTime X-ECEF Y-ECEF Z-ECEF X-LL Y-LL Z-LL VX-ECEF VY-ECEF "
        "VZ-ECEF VEast VNorth VUp Heading Pitch Roll Cx11 Cx22 Cx33 Cx21 Cx31 Cx32 Vel-Cx11 Vel-Cx22 Vel-Cx33 Vel-Cx21 Vel-Cx31 "
        "Vel-Cx32 HdngSD PitchSD RollSD NS GP GL BD GA QZ HDOP VDOP PDOP iFlag L1Rms E-Sep N-Sep H-Sep HdngSep PtchSep RollSep "
        "TotSlpDst").split()
assert len(COLS) == 57


def make_epochs(n=40):
    t = 28640.0 + np.arange(n)
    lat, lon, h = np.radians(30.5) + 1e-6 * np.arange(n), np.radians(114.35) + 2e-6 * np.arange(n), 21.9 + 0.01 * np.arange(n)
    heading = np.linspace(10.0, 350.0, n)
    pitch = 5.0 * np.sin(np.arange(n) / 5.0)
    roll = 3.0 * np.cos(np.arange(n) / 7.0)
    return t, lat, lon, h, heading, pitch, roll


def write_ref(path, secondary, ant_from_imu, point):
    """A .ref whose positions are `point` (ECEF array n x 3)."""
    t, lat, lon, h, hd, pt, rl = make_epochs(len(point))
    lines = ["", "Project:     IEProj", "Source:      GNSS/INS Epochs(Smoothed TC Combined)", "Datum:       WGS84",
             "IMU to GNSS Antenna Lever Arms:", "             x=%.3f, y=%.3f, z=%.3f m (x-right, y-fwd, z-up)" % ant_from_imu,
             "Body to IMU Rotations:", "             xRot=180.000, yRot=0.000, zRot=-90.000 degrees"]
    if secondary is not None:
        lines += ["IMU->Secondary Sensor Lever Arms: ", "            x=%.3f, y=%.3f, z=%.3f m (x-right, y-fwd, z-up, IMU->SENSOR)" % secondary]
    lines += ["SD Scaling Settings:", "  Position: 1.0000", "", " ".join(COLS), " ".join(["(x)"] * 57)]
    for i in range(len(point)):
        la, lo, hh = geodesy.ecef2pos(point[i])
        row = ["2322", "%.3f" % t[i], "%.10f" % np.degrees(la), "%.10f" % np.degrees(lo), "%.3f" % hh, "1", "Fixed", "2024/07/07",
               "7:57:21.00", "%.3f" % point[i][0], "%.3f" % point[i][1], "%.3f" % point[i][2], "0", "0", "0", "0", "0", "0", "0.1", "0.2", "0",
               "%.9f" % hd[i], "%.9f" % pt[i], "%.9f" % rl[i], "1.0E-02", "1.0E-02", "1.0E-02", "0", "0", "0", "0", "0", "0", "0", "0", "0",
               "0.01", "0.01", "0.01", "20", "10", "5", "3", "2", "0", "1.0", "1.5", "1.2", "ZUPT", "0.1", "E228!", "0.01", "0.02", "0", "0", "0", "0"]
        assert len(row) == 57, len(row)
        lines.append(" ".join(row))
    lines.append("E228: No previous position available for relative vector computations (1 occurances)")
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")


class TestAttitude(unittest.TestCase):
    def test_conventions(self):
        R = ie_ref.attitude_matrices([90.0], [0.0], [0.0])[0]
        np.testing.assert_allclose(R[:, 1], [1, 0, 0], atol=1e-12)           # heading 90: forward = east
        np.testing.assert_allclose(R[:, 0], [0, -1, 0], atol=1e-12)          # right = south
        np.testing.assert_allclose(R[:, 2], [0, 0, 1], atol=1e-12)
        R = ie_ref.attitude_matrices([0.0], [10.0], [0.0])[0]
        self.assertAlmostEqual(R[2, 1], np.sin(np.radians(10.0)))            # pitch up: forward axis rises
        R = ie_ref.attitude_matrices([0.0], [0.0], [10.0])[0]
        self.assertAlmostEqual(R[2, 0], -np.sin(np.radians(10.0)))           # roll right-down: the right axis dips

    def test_i300_mount_matrix(self):
        Rm = ie_ref.R_BODY_FROM_I300
        np.testing.assert_allclose(Rm @ Rm.T, np.eye(3), atol=1e-15)
        self.assertAlmostEqual(np.linalg.det(Rm), 1.0)
        np.testing.assert_array_equal(Rm[:, 0], [0, 0, -1])                  # I300 x = down
        np.testing.assert_array_equal(Rm[:, 1], [1, 0, 0])                   # I300 y = right
        np.testing.assert_array_equal(Rm[:, 2], [0, -1, 0])                  # I300 z = back

    def test_quaternion_round_trip(self):
        R = ie_ref.attitude_matrices(np.linspace(0, 359, 12), np.linspace(-20, 20, 12), np.linspace(5, -5, 12))
        q = ie_ref.quat_from_matrices(R)
        from scipy_free_rot import quat_to_matrix
        for i in range(12):
            np.testing.assert_allclose(quat_to_matrix(q[i]), R[i], atol=1e-12)
        self.assertTrue(np.all(q[:, 3] >= 0))


class TestConvert(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.mkdtemp()
        n = 40
        t, lat, lon, h, hd, pt, rl = make_epochs(n)
        A = np.array([geodesy.pos2ecef(a, b, c) for a, b, c in zip(lat, lon, h)])          # antenna point
        R_nb = ie_ref.attitude_matrices(hd, pt, rl)
        l_rfu = np.array([0.0, -0.120, 0.118])                                             # I300 -> antenna in the carrier frame
        enu_to_ecef = [geodesy.enu_matrix(a, b).T for a, b in zip(lat, lon)]
        I = A - np.array([E @ (R @ l_rfu) for E, R in zip(enu_to_ecef, R_nb)])           # I300 point
        self.prim = os.path.join(self.dir, "ROVE_02_GroundTruth.ref")
        self.i300 = os.path.join(self.dir, "I300_GroundTruth.ref")
        write_ref(self.prim, (0.124, -0.128, 0.518), (-0.178, 0.285, 0.065), A)
        write_ref(self.i300, (0.124, -0.008, 0.400), (-0.178, 0.285, 0.065), I)

    def test_read_ref(self):
        r = ie_ref.read_ref(self.prim)
        self.assertEqual(r.rows, 40)
        self.assertEqual(r.skipped, 1)                                    # the E228 trailer line (the units line is skipped by position)
        self.assertEqual(r.header["secondary_sensor"], (0.124, -0.128, 0.518))
        self.assertEqual(r.header["imu_to_antenna"], (-0.178, 0.285, 0.065))
        self.assertEqual(r.header["body_to_imu_deg"], (180.0, 0.0, -90.0))
        self.assertEqual(r.col["iFlag"][0], "ZUPT")
        self.assertEqual(r.col["Q"].dtype.kind, "i")

    def test_convert_writes_the_gtbase_contract(self):
        out = os.path.join(self.dir, "gt")
        audit = gt_convert.convert(self.prim, self.i300, out, "SeqX", max_sigma3=0.5, max_q=3)
        np.testing.assert_allclose(audit["lever_i300_to_antenna_i300_frame"], [-0.118, 0.0, 0.120], atol=1e-9)
        self.assertLess(audit["i300_transfer_residual_m"]["max"], 2e-3)
        for f in ("gt_clean.tum", "gt_audited.tum", "gt_i300.tum", "gt_timebase.yaml", "trust_mask.csv", "gt_mask_intervals.csv",
                  "gt_quality.csv", "gt_audit.json"):
            self.assertTrue(os.path.isfile(os.path.join(out, f)), f)
        rows = np.loadtxt(os.path.join(out, "gt_clean.tum"))
        self.assertEqual(rows.shape, (40, 8))
        # UTC Unix = GPST - 18 s: week 2322 tow 28640 -> 2024-07-07 07:57:03
        self.assertAlmostEqual(rows[0, 0], gnsstime.gpst(2322, 28640.0) - 18.0, places=3)
        np.testing.assert_allclose(rows[0, 1:4], 0.0, atol=1e-6)                           # ENU origin = first epoch
        np.testing.assert_allclose(np.linalg.norm(rows[:, 4:], axis=1), 1.0, atol=1e-8)
        imu = np.loadtxt(os.path.join(out, "gt_i300.tum"))
        # the I300 point is where the I300 file says it is
        enu_i, _ = ie_ref.enu_positions(ie_ref.read_ref(self.i300), origin=audit["enu_origin_lat_lon_h_rad_rad_m"])
        np.testing.assert_allclose(imu[:, 1:4], enu_i, atol=2e-3)
        with open(os.path.join(out, "gt_timebase.yaml")) as f:
            txt = f.read()
        self.assertIn('method: "gpst"', txt)
        self.assertIn("applied_shift_s: 0.0", txt)
        with open(os.path.join(out, "trust_mask.csv")) as f:
            hdr = next(csv.reader(f))
        self.assertTrue(any(h.startswith("ate_exclude(") for h in hdr))
        self.assertEqual(hdr[0], "t")

    def test_gtbase_accepts_the_files(self):
        import gtbase
        out = os.path.join(self.dir, "gt_loader")
        gt_convert.convert(self.prim, self.i300, out, "SeqX", max_sigma3=0.5, max_q=3)
        gt, prov, mask = gtbase.load_gt(self.dir, "SeqX", {"gt_kind": "rtk"}, override=out)
        self.assertEqual((prov["method"], prov["time_shift_applied_s"], prov["method_label"]), ("gpst", 0.0, "GPST 同钟"))
        self.assertEqual(gt.shape, (40, 8))
        self.assertEqual(mask.record()["status"], "applied")

    def test_lever_mismatch_is_rejected(self):
        bad = os.path.join(self.dir, "bad.ref")
        write_ref(bad, (0.124, -0.008, 0.900), (-0.178, 0.285, 0.065), np.array([geodesy.pos2ecef(np.radians(30.5), np.radians(114.35), 21.9)] * 40))
        with self.assertRaises(ValueError):
            gt_convert.convert(self.prim, bad, os.path.join(self.dir, "gt2"), "SeqX")

    def test_readme_cross_check(self):
        txt = ("<!-- \nLeverArm(100C):\nto ROVE_01:-0.178   0.285  0.065(PwrPak7,Ant01)\nto ROVE_02: 0.124  -0.128  0.518(Septentrio,Ant00)\n -->\n<DataInfo/>")
        ok = gt_convert.readme_check(txt, "02", (0.124, -0.128, 0.518))
        self.assertTrue(ok["ok"] and ok["checked"])
        self.assertFalse(gt_convert.readme_check(txt, "02", (0.124, -0.128, 0.9))["ok"])
        self.assertFalse(gt_convert.readme_check(txt, "01", (-0.178, 0.285, 0.065))["ok"])                 # PwrPak7 / Ant01 is not the mini
        self.assertFalse(gt_convert.readme_check(txt, "07", (0, 0, 0))["checked"])
        txt2 = "LeverArm(100C):\nto ROVE_03:\t 0.000   -0.112    0.528 (Septentrio_1, Ant00)\n"
        self.assertTrue(gt_convert.readme_check(txt2, "03", (0.0, -0.112, 0.528))["ok"])
        out = os.path.join(self.dir, "gt3")
        rd = os.path.join(self.dir, "README.xml")
        with open(rd, "w") as f:
            f.write(txt)
        a = gt_convert.convert(self.prim, self.i300, out, "SeqX", readme=rd, mini_id="02")
        self.assertTrue(a["readme_check"]["ok"])

    def test_mask_widening(self):
        t = np.arange(20.0)
        sigma = np.full(20, 0.1)
        sigma[10] = 2.0
        q = np.ones(20, dtype=int)
        q[3] = 4
        ex, bits = gt_convert.mask_epochs(t, sigma, q, 0.5, 3)
        self.assertEqual(list(np.nonzero(ex)[0]), [1, 2, 3, 4, 5, 8, 9, 10, 11, 12])
        self.assertEqual(int(bits[10]), 1)
        self.assertEqual(int(bits[2]), 2)
        self.assertEqual(gt_convert.intervals(t, ex), [(1.0, 5.0), (8.0, 12.0)])


if __name__ == "__main__":
    unittest.main()
