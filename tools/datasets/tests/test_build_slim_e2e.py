# SPDX-License-Identifier: BSD-3-Clause
"""smartpnt/build_slim.py end to end on a synthetic sequence served by a local portal: tree listing, Range downloads with
size checks, streaming LiDAR/camera bags (deleted behind), IMU/camera/LiDAR/GNSS conversion, record times, ground truth,
second-project-root layout, build.json. The raw files follow the real SmartPNT layouts and conventions found in the data
(bogus bag date with GPST time of day, increments in /imu_raw, LiDAR time in seconds from the frame start, RTCM3-style RINEX)."""
import http.server
import io
import json
import os
import re
import struct
import sys
import tempfile
import threading
import unittest

import numpy as np

import _path  # noqa: F401
from dscommon import bagmerge, rosbytes
from fl2g import geodesy
import test_gnsscomm as tg
import test_ie_ref as tir
import test_rinex as tr
from test_rosbytes_slim import make_cloud

NS = 10 ** 9
SEQ = "Data01_20240707_100C_CAR_complex"
BOGUS_DAY = 1404345600                       # 2014-07-03T00:00:00Z: the platform's wrong date, the time of day is GPST
IMU_TOD = 28490.725666                       # 2024-07-07 (a Sunday) 07:54:50.7 GPST
BASE = "00_MINI/%s/" % SEQ


def d19(x):
    return ("%19.12E" % x).replace("E", "E")


def nav_text():
    """RINEX 3.03 nav with one GPS and one GLONASS record (fixed-width 19-character fields)."""
    def rec(first, rows):
        out = first + "\n"
        for r in rows:
            out += "    " + "".join(d19(v) for v in r) + "\n"
        return out
    hdr = ("     3.03           N: GNSS NAV DATA    M: MIXED            RINEX VERSION / TYPE\n"
           "GPSA   1.4901E-08  2.2352E-08 -1.1921E-07 -1.1921E-07       IONOSPHERIC CORR\n"
           "GPSB   1.1674E+05  1.8022E+05 -1.3107E+05 -4.5875E+05       IONOSPHERIC CORR\n"
           "    18                                                      LEAP SECONDS\n"
           "                                                            END OF HEADER\n")
    gps = rec("G14 2024 07 07 08 00 00" + d19(5.8e-4) + d19(7e-12) + d19(0.0),
              [[74, 44.0, 5e-9, -1.1], [2e-6, 0.005, 6e-6, 5153.6], [28800.0, 9e-8, 1.97, -1e-7], [0.94, 249.0, -2.86, -8.3e-9],
               [5.7e-11, 1.0, 2322.0, 0.0], [2.0, 0.0, -7.9e-9, 586.0], [27000.0, 4.0, 0.0, 0.0]])
    glo = rec("R03 2024 07 07 07 45 00" + d19(1.0e-4) + d19(1e-12) + d19(26718.0),
              [[1270.0, 0.2388, 0.0, 0.0], [-21954.0, 1.7582, 0.0, 5.0], [-12831.0, -2.979, 0.0, 0.0]])
    return hdr + gps + glo


def rinex_obs_text():
    """12 s of 10 Hz GPS+BDS epochs from 07:54:48.0 GPST (overlapping the IMU, which starts 07:54:50.7)."""
    lines = [tr.HDR.replace("2024    07    07    07    54   40.6000000", "2024    07    07    07    54   48.0000000")]
    for k in range(120):
        sec = 48.0 + 0.1 * k
        mm, ss = 54 + int(sec // 60), sec % 60
        lines.append("> 2024 07 07 07 %02d %10.7f  0  2\n" % (mm, ss))
        lines.append(tr.sat("G", 5, [tr.fld(22030924.0 + k), tr.fld(115773294.0 + 130 * k, "1" if k == 0 else " "), tr.fld(-1291.2), tr.fld(33.0),
                                      tr.fld(22030923.0 + k), tr.fld(115773294.0), tr.fld(-1291.2), tr.fld(33.0),
                                      tr.fld(22030927.0 + k), tr.fld(90212988.0), tr.fld(-1006.1), tr.fld(31.0)]))
        lines.append(tr.sat("C", 29, [tr.fld(22354302.0 + k), tr.fld(116404718.0), tr.fld(-449.7), tr.fld(38.0)]))
    return "".join(lines)


class Portal(http.server.BaseHTTPRequestHandler):
    files = {}
    server_version = "nginx/1.18.0"
    sys_version = ""

    def log_message(self, *a):
        pass

    def _resolve(self):
        import urllib.parse
        p = urllib.parse.unquote(self.path.lstrip("/"))
        return self.files.get(p)

    def _head(self, status, length, body):
        self.send_response(status)
        self.send_header("Content-Length", str(length))
        self.send_header("Accept-Ranges", "bytes")
        self.send_header("ETag", '"69e91f60-%x"' % len(body))
        self.end_headers()

    def do_HEAD(self):
        b = self._resolve()
        if b is None:
            self.send_response(404)
            self.send_header("Content-Length", "0")
            self.end_headers()
            return
        self._head(200, len(b), b)

    def do_GET(self):
        b = self._resolve()
        if b is None:
            self.send_response(404)
            self.send_header("Content-Length", "0")
            self.end_headers()
            return
        r = self.headers.get("Range")
        if not r:
            self._head(200, len(b), b)
            self.wfile.write(b)
            return
        a, e = re.match(r"bytes=(\d+)-(\d+)", r).groups()
        a, e = int(a), min(int(e), len(b) - 1)
        self._head(206, e - a + 1, b)
        self.wfile.write(b[a:e + 1])


def human(n):
    if n < 1024:
        return "%d B" % n
    if n < 1 << 20:
        return "%.2f KB" % (n / 1024.0)
    return "%.2f MB" % (n / float(1 << 20))


class TestBuildSlim(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:
            import cv2  # noqa: F401
            from rosbags.rosbag1 import Reader  # noqa: F401
        except ImportError:
            raise unittest.SkipTest("rosbags / cv2 not installed")
        cls.tmp = tempfile.mkdtemp()
        cls.files = {}
        cls.make_raw()
        Portal.files = dict(cls.files)
        cls.srv = http.server.ThreadingHTTPServer(("127.0.0.1", 0), Portal)
        cls.port = cls.srv.server_address[1]
        threading.Thread(target=cls.srv.serve_forever, daemon=True).start()

    @classmethod
    def tearDownClass(cls):
        cls.srv.shutdown()

    @classmethod
    def put(cls, rel, data):
        cls.files[BASE + rel] = data

    @classmethod
    def bag_bytes(cls, conns, events):
        p = os.path.join(cls.tmp, "w.bag")
        bagmerge.write_bag(p, conns, iter(events))
        with open(p, "rb") as f:
            data = f.read()
        os.remove(p)
        return data

    @classmethod
    def make_raw(cls):
        imu_def = "Header header\n"
        imu_conn = {"/imu_raw": bagmerge.Conn("/imu_raw", "sensor_msgs/msg/Imu", "6a62c6daae103f4ff57a132d6f95cec2", imu_def)}
        ev = []
        for k in range(600):                                                       # 3 s of 200 Hz increments, bogus date
            t = int(round((BOGUS_DAY + IMU_TOD + 0.005 * k) * NS))
            ev.append((t, "/imu_raw", rosbytes.build_imu(0, t, "imu", (1e-6 * (k % 7), 0.0, -2e-6), (-0.049, 0.001, 0.0004))))
        cls.put("01_SINS/rosbag/imu_2024_07_07_07_56_18.bag", cls.bag_bytes(imu_conn, ev))
        lid_conn = {"/velodyne_points": bagmerge.Conn("/velodyne_points", "sensor_msgs/msg/PointCloud2", "1158d486dd51d683ce2f1be655c3c181", "Header header\n")}
        cam_conn = {"/cam00/image_raw/compressed": bagmerge.Conn("/cam00/image_raw/compressed", "sensor_msgs/msg/CompressedImage",
                                                                  "8f7a12909da2c9d3332d540a0977563f", "Header header\n")}
        import cv2
        yy, xx = np.mgrid[0:64, 0:128]
        img = np.stack([xx * 2, yy * 4, (xx + yy)], axis=2).astype(np.uint8)
        jpg = cv2.imencode(".jpg", img)[1].tobytes()
        for i in range(2):                                                         # two files each, 1.5 s per file
            t0 = BOGUS_DAY + 28498.0 + 1.5 * i
            lev, cev = [], []
            for k in range(15):
                st = int(round((t0 + 0.1008 * k) * NS))
                times = (np.linspace(0.0, 0.1, 120) + 0.0).astype(np.float32)
                lev.append((st, "/velodyne_points", make_cloud(st, times)))
                ct = int(round((t0 + 0.1 * k + 0.1) * NS))
                cev.append((ct, "/cam00/image_raw/compressed", rosbytes.build_compressed(0, ct, "cam00", "bgr8; jpeg compressed bgr8", jpg)))
            cls.put("03_LiDAR/2024-07-07-07-%02d-28_%d.bag" % (56 + i, i), cls.bag_bytes(lid_conn, lev))
            cls.put("02_CAM/cam00/2024-07-07-07-%02d-28_%d_00.bag" % (56 + i, i), cls.bag_bytes(cam_conn, cev))
        cls.put("00_GNSS/01_ROVE/02_mini/ROVE_02.24o", rinex_obs_text().encode())
        cls.put("00_GNSS/02_Product/brdm1890.24p", nav_text().encode())
        cls.put("06_Description/README.xml", b"<DataInfo/>")
        # ground truth: four .ref files of the same trajectory (antenna / I300 / 100C / PwrPak)
        n = 40
        t, lat, lon, h, hd, pt, rl = tir.make_epochs(n)
        A = np.array([geodesy.pos2ecef(a, b, c) for a, b, c in zip(lat, lon, h)])
        R_nb = tir.ie_ref.attitude_matrices(hd, pt, rl)
        l_rfu = np.array([0.0, -0.120, 0.118])
        E = [geodesy.enu_matrix(a, b).T for a, b in zip(lat, lon)]
        Ipt = A - np.array([e @ (r @ l_rfu) for e, r in zip(E, R_nb)])
        d = tempfile.mkdtemp()
        for name, sec, pts in (("ROVE_02", (0.124, -0.128, 0.518), A), ("I300", (0.124, -0.008, 0.400), Ipt)):
            tir.write_ref(os.path.join(d, name), sec, (-0.178, 0.285, 0.065), pts)
            with open(os.path.join(d, name), "rb") as f:
                cls.put("04_GroundTruth/%s_GroundTruth.ref" % name, f.read())

    def tree(self):
        files = [{"id": i + 1, "key": "k", "type": 1, "path": "/data/SmartPNT/SmartPNT/" + p, "name": "x", "fileSize": human(len(b))}
                 for i, (p, b) in enumerate(sorted(self.files.items()))]
        return {"msg": "success", "code": 0, "data": [{"type": 0, "path": "/data/SmartPNT/SmartPNT", "name": "SmartPNT", "children": files}]}

    def test_build(self):
        from smartpnt import build_slim, portal
        root = os.path.join(self.tmp, "fl2g")
        os.makedirs(os.path.join(root, "_portal"))
        with open(os.path.join(root, "_portal", "tree.json"), "w") as f:
            json.dump(self.tree(), f)
        ref = os.path.join(self.tmp, "ref.bag")
        defs = tg.synth_defs()
        rconns = {t: bagmerge.Conn(t, c["msgtype"], c["md5"], c["msgdef"]) for t, c in defs.items()}
        bagmerge.write_bag(ref, rconns, iter([(NS + i, t, b"\x00") for i, t in enumerate(sorted(defs))]))
        old = portal.OSS
        portal.OSS = "http://127.0.0.1:%d" % self.port
        try:
            rc = build_slim.main([SEQ, "--root", root, "--ref-bag", ref, "--image-size", "64x32", "--workers", "2", "--connections", "2",
                                  "--project-root", root])
        finally:
            portal.OSS = old
        self.assertEqual(rc, 0)
        seqdir = os.path.join(root, SEQ)
        # the acceptance checker (a second read-back path) passes on the finished bag, and fails on a wrong expectation
        import verify_bag
        out = os.path.join(self.tmp, "verify.json")
        vrc = verify_bag.main([os.path.join(seqdir, SEQ + ".bag"), "--ref-bag", ref, "--image-size", "64x32", "--camera-stride", "3",
                               "--lidar-stride", "2", "--json", out])
        with open(out) as f:
            vrep = json.load(f)
        self.assertEqual(vrc, 0, [c for c in vrep["checks"] if not c["ok"]])
        self.assertEqual(vrep["lidar"]["n"], 30)
        self.assertEqual(vrep["gnss"]["ephem"], 1)
        self.assertEqual(verify_bag.main([os.path.join(seqdir, SEQ + ".bag"), "--ref-bag", ref, "--image-size", "640x512", "--camera-stride", "3",
                                          "--lidar-stride", "2"]), 1)
        with open(os.path.join(seqdir, "build.json")) as f:
            rec = json.load(f)
        from rosbags.rosbag1 import Reader
        with Reader(os.path.join(seqdir, SEQ + ".bag")) as r:
            topics = {c.topic: c.msgcount for c in r.connections}
            msgs = {}
            for c, t, raw in r.messages():
                msgs.setdefault(c.topic, []).append((t, bytes(raw)))
        self.assertEqual(topics["/imu_raw"], 600)
        self.assertEqual(topics["/velodyne_points"], 30)
        self.assertEqual(topics["/cam00/image_raw/compressed"], 30)
        self.assertGreater(topics["/ublox_driver/range_meas"], 60)
        self.assertEqual(topics["/ublox_driver/ephem"], 1)
        self.assertEqual(topics["/ublox_driver/glo_ephem"], 1)
        self.assertEqual(topics["/ublox_driver/iono_params"], 1)
        # record times are non-decreasing across the whole bag (the writer's order)
        # time rule: 2014 bogus date + GPST tod -> UTC Unix of 2024-07-07 07:54:50.7257 GPST = 07:54:32.7257 UTC
        d0 = rosbytes.parse_imu(msgs["/imu_raw"][0][1])
        self.assertAlmostEqual(d0["stamp_ns"] / 1e9, 1720338872.725666, places=5)
        self.assertEqual(d0["frame_id"], b"imu")
        self.assertAlmostEqual(d0["acc"][0], -0.049 * 200.0, places=9)                     # increments x 200
        # LiDAR: stamp = first point (frame start), time in microseconds, record time = frame end
        c0 = rosbytes.Cloud(msgs["/velodyne_points"][0][1])
        tt = c0.column(msgs["/velodyne_points"][0][1], "time")
        self.assertAlmostEqual(float(tt[0]), 0.0)
        self.assertAlmostEqual(float(tt[-1]), 100000.0, delta=0.5)
        self.assertAlmostEqual(msgs["/velodyne_points"][0][0] / 1e9 - c0.stamp_ns / 1e9, 0.1, places=6)
        self.assertAlmostEqual(c0.stamp_ns / 1e9, 1720338872.725666 + 7.274334, places=3)   # tod 28498.0 -> UTC
        # camera: shrunk to the processing size, stamp as source (shifted), record = stamp + 50 ms
        import cv2
        cam = rosbytes.parse_compressed(msgs["/cam00/image_raw/compressed"][0][1])
        im = cv2.imdecode(np.frombuffer(cam["data"], np.uint8), cv2.IMREAD_COLOR)
        self.assertEqual(im.shape, (32, 64, 3))
        self.assertAlmostEqual(msgs["/cam00/image_raw/compressed"][0][0] - cam["stamp_ns"], 50 * 10 ** 6, delta=1)
        # build record
        tr_ = rec["time_rule"]
        self.assertEqual((tr_["leap_s"], tr_["shift_ns"]), (18, 315964782 * NS))
        self.assertEqual(rec["lidar_time"]["unit"], "s")
        self.assertEqual(rec["lidar_time"]["source_time_reference_scans"]["frame_start"], 30)
        self.assertEqual(rec["gt"]["epochs"], 40)
        np.testing.assert_allclose(rec["gt"]["lever_i300_to_antenna_i300_frame"], [-0.118, 0.0, 0.120], atol=1e-9)
        for k, v in rec["raw_files"].items():
            self.assertEqual(len(v["sha256"]), 64, k)
            if "/03_LiDAR/" in k or "/02_CAM/" in k or k.endswith(".24o") or "imu_2024" in k:
                self.assertFalse(v["kept"], k)                                          # streamed or small originals are deleted
        self.assertTrue(rec["raw_files"][BASE + "00_GNSS/02_Product/brdm1890.24p"]["kept"])
        self.assertTrue(os.path.isfile(rec["replay_inputs"]["nav_rinex"]))
        left = [os.path.join(dp, f) for dp, _, fs in os.walk(os.path.join(seqdir, "raw")) for f in fs]
        self.assertFalse([p for p in left if "LiDAR" in p or "cam00" in p or p.endswith(".bag")], left)
        # second project root layout
        self.assertTrue(os.path.isfile(os.path.join(root, "smartpnt", "bags", SEQ + ".bag")))
        self.assertTrue(os.path.isfile(os.path.join(root, "audit", SEQ, "gt", "gt_clean.tum")))
        with open(os.path.join(root, "smartpnt", "bags", SEQ + ".fetch.json")) as f:
            fj = json.load(f)
        self.assertTrue(fj["brdc"][0].endswith("brdm1890.24p"))
        self.assertEqual(rec["layout"]["bag_sha256"], rec["bag"]["sha256"])
        # a queue that reaches a finished sequence again does not touch it (no network needed: the check comes first)
        bag = os.path.join(seqdir, SEQ + ".bag")
        m0 = os.stat(bag).st_mtime_ns
        self.assertEqual(build_slim.main([SEQ, "--root", root, "--ref-bag", ref]), 0)
        self.assertEqual(os.stat(bag).st_mtime_ns, m0)
        self.assertTrue(build_slim.finished_build(seqdir, bag))
        # one build per sequence: a second process (here: a second open file description) is refused while the lock is held
        held = build_slim.acquire_build_lock(seqdir)
        try:
            with self.assertRaises(SystemExit):
                build_slim.main([SEQ, "--root", root, "--ref-bag", ref])
        finally:
            held.close()
        self.assertEqual(build_slim.main([SEQ, "--root", root, "--ref-bag", ref]), 0)        # released with the file
        with open(bag, "ab") as f:
            f.write(b"x")
        self.assertFalse(build_slim.finished_build(seqdir, bag))                            # a size change = not the recorded bag


if __name__ == "__main__":
    unittest.main()
