# SPDX-License-Identifier: BSD-3-Clause
"""mars.gt_convert / mars.build_slim topic rules and dscommon.layout."""
import json
import os
import struct
import sys
import tempfile
import unittest

import numpy as np

import _path  # noqa: F401
from dscommon import bagmerge, layout, rosbytes
from fl2g import geodesy

sys.path.insert(0, os.path.join(_path.DATASETS, "mars"))
import build_slim as mars_build  # noqa: E402
import gt_convert as mars_gt  # noqa: E402

NS = 10 ** 9


def navsatfix(stamp_ns, lat, lon, alt, status=2):
    cov = struct.pack("<9d", *([0.0] * 9))
    return (rosbytes.pack_header(0, stamp_ns, "gps") + struct.pack("<bH", status, 1) + struct.pack("<ddd", lat, lon, alt) + cov +
            struct.pack("<B", 0))


class TestMarsGt(unittest.TestCase):
    def make_bag(self, n=60):
        try:
            import rosbags  # noqa: F401
        except ImportError:
            self.skipTest("rosbags not installed")
        d = tempfile.mkdtemp()
        p = os.path.join(d, "mars.bag")
        conns = {mars_gt.TOPIC_POS: bagmerge.Conn(mars_gt.TOPIC_POS, "sensor_msgs/msg/NavSatFix", "2d3a8cd499b9b4a0249fb98fd05cfa48", "Header header\n"),
                 mars_gt.TOPIC_INFO: bagmerge.Conn(mars_gt.TOPIC_INFO, "std_msgs/msg/UInt8", "7c8164229e7d2c17eb95e9231617fdee", "uint8 data\n")}
        ev = []
        lat0, lon0 = np.radians(22.3), np.radians(114.1)
        for i in range(n):
            t = 1698000000 * NS + i * 200000000
            ev.append((t + 5, mars_gt.TOPIC_POS, navsatfix(t, np.degrees(lat0) + 1e-6 * i, np.degrees(lon0), 30.0 + 0.01 * i)))
            ev.append((t + 6, mars_gt.TOPIC_INFO, bytes([50 if not (20 <= i < 25) else 16])))
        bagmerge.write_bag(p, conns, iter(sorted(ev, key=lambda e: e[0])))
        return d, p

    def test_navsatfix_parse(self):
        st, la, lo, al, status = mars_gt.parse_navsatfix(navsatfix(5 * NS + 7, 22.5, 114.25, 31.5, 2))
        self.assertEqual((st, la, lo, al, status), (5 * NS + 7, 22.5, 114.25, 31.5, 2))

    def test_convert(self):
        d, p = self.make_bag()
        out = os.path.join(d, "gt")
        a = mars_gt.convert(p, out, "HKisland_GNSS03", shift_s=0.06, method="xcorr")
        self.assertEqual(a["epochs"], 60)
        self.assertAlmostEqual(a["rate_hz_median"], 5.0, places=3)
        rows = np.loadtxt(os.path.join(out, "gt_clean.tum"))
        self.assertAlmostEqual(rows[0, 0], 1698000000.0, places=3)                         # gt_clean: the header stamps as recorded
        aud = np.loadtxt(os.path.join(out, "gt_audited.tum"))
        np.testing.assert_allclose(aud[:, 0] - rows[:, 0], 0.06, atol=1e-6)                 # gt_audited = clean + applied shift (gtbase checks it)
        np.testing.assert_array_equal(aud[:, 1:], rows[:, 1:])
        np.testing.assert_allclose(rows[0, 1:4], 0.0, atol=1e-6)
        self.assertGreater(rows[59, 2], 6.0)                                                # 59e-6 deg north ~ 6.5 m
        np.testing.assert_array_equal(rows[:, 4:], np.tile([0, 0, 0, 1], (60, 1)))
        with open(os.path.join(out, "gt_timebase.yaml")) as f:
            txt = f.read()
        self.assertIn('method: "xcorr"', txt)
        self.assertIn("applied_shift_s: 0.060000", txt)
        mask = np.genfromtxt(os.path.join(out, "trust_mask.csv"), delimiter=",", skip_header=1)
        ex = mask[:, 3].astype(bool)
        self.assertTrue(ex[10:35].all() and not ex[9] and not ex[35])                              # bad epochs 20-24, +-2 s = +-10 epochs at 5 Hz
        self.assertEqual(a["excluded_epochs"], int(ex.sum()))

    def test_gtbase_accepts_the_files(self):
        """The evaluation's own loader (tools/eval/gtbase.py): clean + applied shift == audited, mask rows match, xcorr method known."""
        import evalcommon as ec
        import gtbase
        d, p = self.make_bag()
        out = os.path.join(d, "gt")
        mars_gt.convert(p, out, "HKisland_GNSS03", shift_s=-0.112, method="xcorr")
        gt, prov, mask = gtbase.load_gt(d, "HKisland_GNSS03", {"gt_kind": "rtk"}, override=out)
        self.assertEqual((prov["method"], prov["method_label"]), ("xcorr", "速度互相关"))
        self.assertAlmostEqual(prov["time_shift_applied_s"], -0.112)
        self.assertAlmostEqual(gt[0, 0], 1698000000.0 - 0.112, places=3)
        self.assertEqual(mask.record()["status"], "applied")

    def test_small_shift_is_not_applied(self):
        d, p = self.make_bag(10)
        mars_gt.convert(p, os.path.join(d, "gt"), "S", shift_s=0.01, method="xcorr")
        with open(os.path.join(d, "gt", "gt_timebase.yaml")) as f:
            self.assertIn("applied_shift_s: 0.000000", f.read())                              # A54's 20 ms rule


class TestMarsTopics(unittest.TestCase):
    def test_keep_rules(self):
        k = mars_build.keep
        for t in ("/livox/lidar", "/livox/imu", "/left_camera/image/compressed", "/ublox_driver/range_meas", "/ublox_driver/receiver_pvt",
                  "/dji_osdk_ros/rtk_position", "/dji_osdk_ros/time_sync_gps_utc"):
            self.assertTrue(k(t), t)
        for t in ("/dji_osdk_ros/imu", "/dji_osdk_ros/attitude", "/dji_osdk_ros/rc", "/livox/lidar_extra"):
            self.assertFalse(k(t), t)


class TestLayout(unittest.TestCase):
    def test_publish(self):
        root = tempfile.mkdtemp()
        bag = os.path.join(root, "smartpnt", "S1", "S1.bag")
        gt = os.path.join(root, "smartpnt", "S1", "gt")
        nav = os.path.join(root, "smartpnt", "S1", "raw", "brdm1890.24p")
        for p in (bag, nav, os.path.join(gt, "gt_clean.tum")):
            os.makedirs(os.path.dirname(p), exist_ok=True)
            open(p, "w").write("x")
        rec = layout.publish(root, "smartpnt", "S1", bag, gt, [nav], "ab" * 32, {"duration_s": 12.5})
        self.assertEqual(rec["brdc"], ["smartpnt/S1/raw/brdm1890.24p"])
        link = os.path.join(root, "smartpnt", "bags", "S1.bag")
        self.assertTrue(os.path.islink(link))
        self.assertFalse(os.path.isabs(os.readlink(link)))
        self.assertTrue(os.path.isfile(link))                                                 # resolves inside the root
        self.assertTrue(os.path.isfile(os.path.join(root, "audit", "S1", "gt", "gt_clean.tum")))
        with open(os.path.join(root, "smartpnt", "bags", "S1.fetch.json")) as f:
            self.assertEqual(json.load(f)["brdc"], rec["brdc"])
        with open(os.path.join(root, "smartpnt", "bags", "S1.bag.sha256")) as f:
            self.assertEqual(f.read(), "%s  S1.bag\n" % ("ab" * 32))
        layout.publish(root, "smartpnt", "S1", bag, gt, [nav], "cd" * 32)                      # idempotent
        with self.assertRaises(ValueError):
            layout.publish(root, "smartpnt", "S2", "/elsewhere/S2.bag", gt, [nav], "00" * 32)


if __name__ == "__main__":
    unittest.main()
