# SPDX-License-Identifier: BSD-3-Clause
"""dscommon.rosbytes and smartpnt.slim: raw message editing, IMU scaling, cloud time normalisation, image slimming."""
import struct
import unittest

import numpy as np

import _path  # noqa: F401
from dscommon import rosbytes
from smartpnt import slim

NS = 10 ** 9


def make_cloud(stamp_ns, times, seq=7, frame="velodyne"):
    """VLP-16 style PointCloud2 (x y z intensity float32, ring uint16, time float32; point_step 22)."""
    n = len(times)
    pts = np.zeros(n, dtype=np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("i", "<f4"), ("ring", "<u2"), ("t", "<f4")]))
    pts["x"], pts["y"], pts["z"] = np.arange(n) * 0.1, np.arange(n) * 0.2, np.arange(n) * 0.3
    pts["i"], pts["ring"], pts["t"] = 10.0, np.arange(n) % 16, times
    data = pts.tobytes()
    fields = [("x", 0, 7, 1), ("y", 4, 7, 1), ("z", 8, 7, 1), ("intensity", 12, 7, 1), ("ring", 16, 4, 1), ("time", 18, 7, 1)]
    out = rosbytes.pack_header(seq, stamp_ns, frame) + struct.pack("<III", 1, n, len(fields))
    for name, off, dt, cnt in fields:
        out += struct.pack("<I", len(name)) + name.encode() + struct.pack("<IBI", off, dt, cnt)
    out += struct.pack("<BII", 0, 22, 22 * n) + struct.pack("<I", len(data)) + data + struct.pack("<B", 1)
    return out


class TestRosBytes(unittest.TestCase):
    def test_imu_round_trip(self):
        raw = rosbytes.build_imu(3, 1720338872 * NS + 725666046, b"imu", (0.1, -0.2, 0.3), (9.8, 0.0, -0.1))
        d = rosbytes.parse_imu(raw)
        self.assertEqual((d["seq"], d["stamp_ns"], d["frame_id"]), (3, 1720338872 * NS + 725666046, b"imu"))
        self.assertEqual((d["gyro"], d["acc"]), ((0.1, -0.2, 0.3), (9.8, 0.0, -0.1)))
        self.assertEqual(len(raw), 315)                                       # the size of the SmartPNT source messages
        with self.assertRaises(ValueError):
            rosbytes.parse_imu(raw + b"\0")

    def test_compressed_round_trip(self):
        raw = rosbytes.build_compressed(1, 5 * NS + 7, "cam00", "bgr8; jpeg compressed bgr8", b"\xff\xd8abc")
        d = rosbytes.parse_compressed(raw)
        self.assertEqual((d["stamp_ns"], d["frame_id"], d["format"], d["data"]), (5 * NS + 7, b"cam00", b"bgr8; jpeg compressed bgr8", b"\xff\xd8abc"))

    def test_cloud_parse_and_rewrite(self):
        t = np.linspace(0.0, 0.1, 50, dtype=np.float32)
        raw = make_cloud(100 * NS + 5, t)
        c = rosbytes.Cloud(raw)
        self.assertEqual((c.npoints, c.point_step, c.stamp_ns, c.frame_id), (50, 22, 100 * NS + 5, b"velodyne"))
        np.testing.assert_array_equal(c.column(raw, "time"), t)
        self.assertEqual(c.column(raw, "ring", "<u2")[17], 1)
        new_t = (t * 1e6).astype(np.float32)
        out = rosbytes.rewrite_cloud(raw, c, 101 * NS + 9, new_t)
        c2 = rosbytes.Cloud(out)
        self.assertEqual(c2.stamp_ns, 101 * NS + 9)
        np.testing.assert_array_equal(c2.column(out, "time"), new_t)
        for name in ("x", "y", "z", "intensity"):
            np.testing.assert_array_equal(c2.column(out, name), c.column(raw, name))
        np.testing.assert_array_equal(c2.column(out, "ring", "<u2"), c.column(raw, "ring", "<u2"))
        self.assertEqual(len(out), len(raw))

    def test_detect_time_unit(self):
        self.assertEqual([rosbytes.detect_time_unit(x) for x in (0.1, 100.0, 1.0e5, 1.0e8)], ["s", "ms", "us", "ns"])


class TestSlim(unittest.TestCase):
    def test_imu_rates(self):
        raw = rosbytes.build_imu(0, 1404374090 * NS + 725666000, b"junk", (1e-6, 2e-6, -3e-6), (-0.049, 0.001, 0.0004))
        stamp, out = slim.imu_to_rates(raw, 315964782 * NS)
        d = rosbytes.parse_imu(out)
        self.assertEqual(stamp, 1720338872 * NS + 725666000)
        self.assertEqual(d["frame_id"], b"imu")
        np.testing.assert_allclose(d["gyro"], (2e-4, 4e-4, -6e-4))
        np.testing.assert_allclose(d["acc"], (-9.8, 0.2, 0.08))
        self.assertEqual(d["orientation"], (0.0, 0.0, 0.0, 0.0))

    def test_lidar_head_and_tail_referenced_scans_give_the_same_cloud(self):
        base = 1000 * NS
        head = np.linspace(0.0, 0.1, 120, dtype=np.float32) + np.float32(0.008)   # offsets from the frame start
        tail = (head - np.float32(0.108))                                        # same instants, referenced to the frame end
        rule_h, rule_t = slim.LidarTimeRule(), slim.LidarTimeRule()
        rec_h, st_h, out_h, _ = slim.lidar_to_slim(make_cloud(base, head), 5 * NS, rule_h)
        rec_t, st_t, out_t, _ = slim.lidar_to_slim(make_cloud(base + int(0.108 * NS), tail), 5 * NS, rule_t)
        self.assertEqual(rule_h.unit, "s")
        self.assertLess(abs(st_h - st_t), 20)                                     # float32 rounding of the offsets only
        ch, ct = rosbytes.Cloud(out_h), rosbytes.Cloud(out_t)
        th, tt = ch.column(out_h, "time"), ct.column(out_t, "time")
        self.assertAlmostEqual(float(th[0]), 0.0, places=6)
        np.testing.assert_allclose(th, tt, atol=0.05)                             # microseconds
        self.assertAlmostEqual(float(th[-1]), 100000.0, delta=0.1)                # 0.1 s in us
        self.assertEqual(rec_h, st_h + int(round(float(th.max()) * 1e3)))         # record = frame end
        for name in ("x", "y", "z"):
            np.testing.assert_array_equal(ch.column(out_h, name), rosbytes.Cloud(make_cloud(base, head)).column(make_cloud(base, head), name))

    def test_lidar_absolute_point_time_is_preserved(self):
        t = np.array([0.02, 0.05, 0.09], dtype=np.float32)
        raw = make_cloud(2000 * NS, t)
        rule = slim.LidarTimeRule("s")
        rec, st, out, info = slim.lidar_to_slim(raw, 0, rule)
        us = rosbytes.Cloud(out).column(out, "time")
        for k in range(3):
            self.assertAlmostEqual((st - 2000 * NS) / 1e9 + float(us[k]) * 1e-6, float(t[k]), places=6)

    def test_lidar_unit_microseconds_input(self):
        t = (np.linspace(0, 0.1, 200) * 1e6).astype(np.float32)
        rule = slim.LidarTimeRule()
        _, _, out, _ = slim.lidar_to_slim(make_cloud(NS, t), 0, rule)
        self.assertEqual(rule.unit, "us")
        np.testing.assert_allclose(rosbytes.Cloud(out).column(out, "time"), t, atol=0.1)

    def test_lidar_nonmonotonic_scan_is_reordered(self):
        t = np.array([0.05, 0.01, 0.09, 0.03] * 30, dtype=np.float32)
        raw = make_cloud(NS, t)
        rule = slim.LidarTimeRule("s")
        _, _, out, _ = slim.lidar_to_slim(raw, 0, rule)
        c = rosbytes.Cloud(out)
        tt = c.column(out, "time")
        self.assertTrue(np.all(np.diff(tt) >= 0))
        self.assertEqual(rule.nonmonotonic, 1)
        # the x column moved with the time column (points were reordered as rows, not just their times)
        x0 = rosbytes.Cloud(raw).column(raw, "x")
        order = np.argsort(t, kind="stable")
        np.testing.assert_array_equal(c.column(out, "x"), x0[order])

    def test_image_slimmer_and_camera_message(self):
        import cv2
        yy, xx = np.mgrid[0:64, 0:128]
        img = np.stack([xx * 2, yy * 4, (xx + yy) * 1], axis=2).astype(np.uint8)       # smooth content: JPEG-friendly
        ok, enc = cv2.imencode(".jpg", img)
        sl = slim.ImageSlimmer((64, 32), 90)
        raw = rosbytes.build_compressed(5, 77 * NS, "cam00", "bgr8; jpeg compressed bgr8", enc.tobytes())
        stamp, out = slim.camera_to_slim(raw, 3 * NS, sl)
        d = rosbytes.parse_compressed(out)
        self.assertEqual((stamp, d["stamp_ns"], d["format"], d["frame_id"]), (80 * NS, 80 * NS, b"bgr8; jpeg compressed bgr8", b"cam00"))
        back = cv2.imdecode(np.frombuffer(d["data"], np.uint8), cv2.IMREAD_COLOR)
        self.assertEqual(back.shape, (32, 64, 3))
        ref = cv2.resize(cv2.imdecode(enc, cv2.IMREAD_COLOR), (64, 32), interpolation=cv2.INTER_LINEAR)
        self.assertLess(float(np.abs(back.astype(int) - ref.astype(int)).mean()), 8.0)       # JPEG loss only
        with self.assertRaises(ValueError):
            sl(b"not a jpeg")


if __name__ == "__main__":
    unittest.main()
