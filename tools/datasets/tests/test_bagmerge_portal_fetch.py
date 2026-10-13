# SPDX-License-Identifier: BSD-3-Clause
"""dscommon.bagmerge, smartpnt.portal and dscommon.httpfetch (against a local range-capable HTTP server)."""
import hashlib
import http.server
import json
import os
import re
import tempfile
import threading
import unittest

import _path  # noqa: F401
from dscommon import bagmerge, httpfetch
from smartpnt import portal

NS = 10 ** 9


class TestMerge(unittest.TestCase):
    def test_merge_orders_by_time_then_rank(self):
        imu = [(1, "/imu", b"i1"), (3, "/imu", b"i3"), (5, "/imu", b"i5")]
        lid = [(3, "/lidar", b"l3"), (4, "/lidar", b"l4")]
        gn = [(1, "/gnss", b"g1"), (5, "/gnss", b"g5")]
        out = list(bagmerge.merge([(bagmerge.RANK["imu"], imu), (bagmerge.RANK["lidar"], lid), (bagmerge.RANK["gnss"], gn)]))
        self.assertEqual([p for _, _, p in out], [b"i1", b"g1", b"i3", b"l3", b"l4", b"i5", b"g5"])   # IMU before gnss before lidar at a tie

    def test_monotonic_clamps(self):
        m = bagmerge.Monotonic([(1, "a", b"x"), (3, "a", b"y"), (2, "a", b"z"), (4, "a", b"w")], "s")
        self.assertEqual([t for t, _, _ in m], [1, 3, 3, 4])
        self.assertEqual(m.clamped, 1)

    def test_pmap_keeps_order(self):
        import time
        def f(x):
            time.sleep(0.002 * (x % 3))
            return x * x
        self.assertEqual(list(bagmerge.pmap(f, range(20), workers=4, window=6)), [x * x for x in range(20)])
        self.assertEqual(list(bagmerge.pmap(f, range(5), workers=1)), [0, 1, 4, 9, 16])

    def test_write_and_read_back(self):
        try:
            from rosbags.rosbag1 import Reader
        except ImportError:
            self.skipTest("rosbags not installed")
        d = tempfile.mkdtemp()
        p = os.path.join(d, "t.bag")
        defs = "string data\n"
        conns = {"/a": bagmerge.Conn("/a", "std_msgs/msg/String", "992ce8a1687cec8c8bd883ec73ca41d1", defs),
                 "/b": bagmerge.Conn("/b", "std_msgs/msg/String", "992ce8a1687cec8c8bd883ec73ca41d1", defs)}
        ev = [(NS + 5, "/a", b"\x02\x00\x00\x00hi"), (NS + 6, "/b", b"\x03\x00\x00\x00abc"), (NS + 9, "/a", b"\x01\x00\x00\x00z")]
        stats = bagmerge.write_bag(p, conns, iter(ev))
        self.assertEqual((stats["/a"]["n"], stats["/b"]["n"]), (2, 1))
        with Reader(p) as r:
            got = [(t, c.topic, bytes(raw)) for c, t, raw in r.messages()]
        self.assertEqual(got, [(t, tp, raw) for t, tp, raw in ev])


class TestUnindexed(unittest.TestCase):
    """A recording cut off leaves a bag without its index (SmartPNT 0707: the last LiDAR file). open_reader reindexes it."""

    def test_unindexed_bag_is_reindexed_and_noted(self):
        try:
            from rosbags.rosbag1 import Reader, ReaderError
        except ImportError:
            self.skipTest("rosbags not installed")
        d = tempfile.mkdtemp()
        p = os.path.join(d, "t.bag")
        conns = {"/a": bagmerge.Conn("/a", "std_msgs/msg/String", "992ce8a1687cec8c8bd883ec73ca41d1", "string data\n")}
        bagmerge.write_bag(p, conns, iter([(NS + 1, "/a", b"\x01\x00\x00\x00x"), (NS + 2, "/a", b"\x01\x00\x00\x00y")]))
        with open(p, "rb") as f:
            good = f.read()
        raw = bytearray(good)
        k = raw.find(b"index_pos=", 0, 4096)
        self.assertGreater(k, 0)
        raw[k + 10:k + 18] = b"\0" * 8                                          # index_pos = 0: "Bag is not indexed"
        with open(p, "wb") as f:
            f.write(bytes(raw))
        with self.assertRaises(ReaderError):
            Reader(p).open()
        calls = []

        def fake_reindex(path):                                                   # the ROS reindex needs ROS: restore the index instead
            calls.append(path)
            with open(path, "wb") as f:
                f.write(good)
            return 3

        old = bagmerge.reindex_in_place
        bagmerge.reindex_in_place = fake_reindex
        try:
            notes = []
            got = [(t, bytes(r)) for t, r, c in bagmerge.iter_bag(p, "/a", notes)]
        finally:
            bagmerge.reindex_in_place = old
        self.assertEqual(len(got), 2)
        self.assertEqual(calls, [p])
        self.assertEqual((notes[0]["file"], notes[0]["reindexed"], notes[0]["chunks"]), ("t.bag", True, 3))


def node(path, size, kind="file"):
    name = path.split("/")[-1]
    return {"id": abs(hash(path)) % (10 ** 9), "key": "k", "type": 1 if kind == "file" else 0,
            "path": "/data/SmartPNT/SmartPNT/" + path, "name": "%s  (%s)" % (name, size), "fileSize": size}


def tree_for(seqs):
    files = []
    for platform, seq, style in seqs:
        b = "%s/%s/" % (platform, seq)
        files += [node(b + "00_GNSS/02_Product/brdm1890.24p", "8.89 MB"), node(b + "01_SINS/rosbag/imu_2024_07_07_07_56_18.bag", "148.75 MB"),
                  node(b + "01_SINS/I300.imr", "12.76 MB"), node(b + "06_Description/Data_Description.pdf", "941.07 KB"),
                  node(b + "06_Description/README.xml", "1.36 KB")]
        for i in (0, 1, 2, 9, 10, 11):
            files += [node(b + "03_LiDAR/2024-07-07-07-%02d-28_%d.bag" % (56 + i, i), "270.00 MB"),
                      node(b + "02_CAM/cam00/2024-07-07-07-%02d-28_%d_00.bag" % (56 + i, i), "500.00 MB"),
                      node(b + "02_CAM/cam01/2024-07-07-07-%02d-28_%d_01.bag" % (56 + i, i), "500.00 MB")]
        if style == "early":
            files += [node(b + "00_GNSS/00_BASE/00_SGG/BASE.24o", "78.17 MB"), node(b + "00_GNSS/01_ROVE/01_PwrPak7/ROVE_01.24O", "6.69 MB"),
                      node(b + "00_GNSS/01_ROVE/02_mini/ROVE_02.24o", "66.35 MB")]
            files += [node(b + "04_GroundTruth/%s_GroundTruth.ref" % n, "1.08 MB") for n in ("100C", "I300", "ROVE_01", "ROVE_02")]
        else:
            files += [node(b + "00_GNSS/00_BASE/00_WUH2/BASE_WUH200CHN.24o", "109.68 MB"),
                      node(b + "00_GNSS/00_BASE/01_Qianxun/BASE_01.24O", "22.27 MB"),
                      node(b + "00_GNSS/01_ROVE/03_mini/ROVE_03.24o", "266.06 MB"), node(b + "00_GNSS/01_ROVE/04_Septentrio/ROVE_04.24o", "50.83 MB")]
            files += [node(b + "04_GroundTruth/%s_GroundTruth.ref" % n, "2.26 MB") for n in ("100C", "I300", "Mini", "ROVE")]
    return {"msg": "success", "code": 0, "data": [{"type": 0, "path": "/data/SmartPNT/SmartPNT", "name": "SmartPNT", "children": files}]}


class TestPortal(unittest.TestCase):
    def test_human_sizes(self):
        self.assertEqual(portal.human_to_bytes("78.17 MB")[0], 78.17 * (1 << 20))
        self.assertEqual(portal.human_to_bytes("689 B")[0], 689)
        with self.assertRaises(portal.PortalError):
            portal.human_to_bytes("lots")
        # the portal size is the server size rounded to 0.01 of its unit: 81965547 B is "78.17 MB"
        v, bound = portal.human_to_bytes("78.17 MB")
        self.assertLessEqual(abs(81965547 - v), bound)

    def test_layouts_of_both_styles(self):
        files = portal.index_files(tree_for([("00_MINI", "Data01_20240707_100C_CAR_complex", "early"),
                                             ("00_MINI", "Data01_20240719_100C_CAR_opensky", "late")]))
        e = portal.Layout(files, "00_MINI", "Data01_20240707_100C_CAR_complex")
        self.assertTrue(e.obs_mini.rel.endswith("02_mini/ROVE_02.24o"))
        self.assertEqual(e.mini_id, "02")
        self.assertTrue(e.gt_primary.rel.endswith("ROVE_02_GroundTruth.ref"))
        self.assertTrue(e.base.rel.endswith("00_SGG/BASE.24o"))
        self.assertEqual([x.rel.split("_")[-1] for x in e.lidar], ["0.bag", "1.bag", "2.bag", "9.bag", "10.bag", "11.bag"])   # numeric order
        self.assertEqual(len(e.cam), 6)
        self.assertTrue(all("/cam00/" in x.rel for x in e.cam))
        self.assertTrue(e.imr.rel.endswith("I300.imr"))
        l = portal.Layout(files, "00_MINI", "Data01_20240719_100C_CAR_opensky")
        self.assertTrue(l.obs_mini.rel.endswith("03_mini/ROVE_03.24o"))
        self.assertTrue(l.gt_primary.rel.endswith("Mini_GroundTruth.ref"))
        self.assertTrue(l.base.rel.endswith("01_Qianxun/BASE_01.24O"))
        self.assertIn(l.gt_primary, l.small_files())
        self.assertEqual(portal.Entry({"id": 1, "path": "/data/SmartPNT/SmartPNT/00_MINI/a b/c.bag", "type": 1, "fileSize": "1 KB"}).url(),
                         portal.OSS + "/00_MINI/a%20b/c.bag")

    def test_missing_and_ambiguous(self):
        files = portal.index_files(tree_for([("00_MINI", "S", "early")]))
        with self.assertRaises(portal.PortalError):
            portal.Layout(files, "00_MINI", "Nope")
        extra = dict(files)
        e = portal.Entry(node("00_MINI/S/00_GNSS/01_ROVE/05_mini/ROVE_05.24o", "1 MB"))
        extra[e.rel] = e
        with self.assertRaises(portal.PortalError):
            portal.Layout(extra, "00_MINI", "S")                                   # two mini receivers: refuse to guess


class RangeHandler(http.server.BaseHTTPRequestHandler):
    server_version = "nginx/1.18.0"           # the SmartPNT OSS is nginx; its ETag carries the size
    sys_version = ""
    payload = b""
    lie_about_size = False

    def log_message(self, *a):
        pass

    def _head(self, status, length, extra=None):
        self.send_response(status)
        self.send_header("Content-Length", str(length))
        self.send_header("Accept-Ranges", "bytes")
        size = len(self.payload)
        self.send_header("ETag", '"69e91f60-%x"' % (size + (1 if self.lie_about_size else 0)))
        for k, v in (extra or {}).items():
            self.send_header(k, v)
        self.end_headers()

    def do_HEAD(self):
        self._head(200, len(self.payload))

    def do_GET(self):
        r = self.headers.get("Range")
        if not r:
            self._head(200, len(self.payload))
            self.wfile.write(self.payload)
            return
        a, b = re.match(r"bytes=(\d+)-(\d+)", r).groups()
        a, b = int(a), min(int(b), len(self.payload) - 1)
        self._head(206, b - a + 1, {"Content-Range": "bytes %d-%d/%d" % (a, b, len(self.payload))})
        self.wfile.write(self.payload[a:b + 1])


class TestHttpFetch(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        RangeHandler.payload = bytes(bytearray((i * 7 + i // 251) & 0xFF for i in range(1234567)))
        cls.srv = http.server.ThreadingHTTPServer(("127.0.0.1", 0), RangeHandler)
        cls.url = "http://127.0.0.1:%d/f.bin" % cls.srv.server_address[1]
        threading.Thread(target=cls.srv.serve_forever, daemon=True).start()

    @classmethod
    def tearDownClass(cls):
        cls.srv.shutdown()

    def test_download_segments_and_hash(self):
        d = tempfile.mkdtemp()
        dest = os.path.join(d, "x", "f.bin")
        rec = httpfetch.download(self.url, dest, expect_size=1234567, connections=3, segment_bytes=100000)
        self.assertEqual((rec["size"], rec["reused"]), (1234567, False))
        self.assertEqual(rec["sha256"], hashlib.sha256(RangeHandler.payload).hexdigest())
        with open(dest, "rb") as f:
            self.assertEqual(f.read(), RangeHandler.payload)
        self.assertFalse(os.path.exists(dest + ".part"))
        again = httpfetch.download(self.url, dest, connections=2)
        self.assertTrue(again["reused"])

    def test_size_mismatch_is_an_error(self):
        with self.assertRaises(httpfetch.FetchError):
            httpfetch.download(self.url, os.path.join(tempfile.mkdtemp(), "f.bin"), expect_size=5)

    def test_etag_size(self):
        self.assertEqual(httpfetch.etag_size("69e91f60-4e2b1eb"), 0x4e2b1eb)
        self.assertIsNone(httpfetch.etag_size("W/abc"))
        self.assertIsNone(httpfetch.etag_size("69e91f60-4e2b1eb", "Apache/2.4"))             # another server: no claim about the size
        self.assertIsNone(httpfetch.etag_size("1ac3b-6538c6ea1d9a4", "nginx"))                # not the nginx <8 hex>-<size> shape
        RangeHandler.lie_about_size = True
        try:
            with self.assertRaises(httpfetch.FetchError):
                httpfetch.download(self.url, os.path.join(tempfile.mkdtemp(), "f.bin"))
        finally:
            RangeHandler.lie_about_size = False

    def test_rawstore_records_and_releases(self):
        old = portal.OSS
        portal.OSS = self.url.rsplit("/", 1)[0]
        try:
            e = portal.Entry({"id": 7, "path": "/data/SmartPNT/SmartPNT/f.bin", "type": 1, "fileSize": "1.18 MB"})
            d = tempfile.mkdtemp()
            st = portal.RawStore(d, connections=2)
            path = st.ensure(e)
            self.assertTrue(os.path.isfile(path))
            m = st.manifest["f.bin"]
            self.assertEqual((m["size"], m["kept"], m["portal_size"]), (1234567, True, "1.18 MB"))
            self.assertEqual(len(m["sha256"]), 64)
            st.release(e)
            self.assertFalse(os.path.exists(path))
            self.assertFalse(st.manifest["f.bin"]["kept"])
            st.save_manifest(os.path.join(d, "m.json"))
            with open(os.path.join(d, "m.json")) as f:
                self.assertIn("f.bin", json.load(f))
            bad = portal.Entry({"id": 7, "path": "/data/SmartPNT/SmartPNT/f.bin", "type": 1, "fileSize": "5.00 MB"})
            with self.assertRaises(portal.PortalError):
                st.ensure(bad)                                                    # portal size far from the server's
        finally:
            portal.OSS = old


if __name__ == "__main__":
    unittest.main()
