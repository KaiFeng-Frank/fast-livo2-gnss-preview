# SPDX-License-Identifier: BSD-3-Clause
"""tools/bag_filter.py on a synthetic bag: only the data-chain topics survive (IMPL_PLAN §2), every kept message is
byte-identical with an identical record time (ns), connection headers (type, md5sum, message_definition, callerid,
latching) are preserved per connection (checked with the ROS rosbag API, independent of rosbags), the time window
and --drop-gnss behave, and the .sha256 sidecar matches the file. Needs ROS Python + rosbags (project image)."""
import contextlib
import hashlib
import io
import os
import tempfile
import unittest

import _path  # noqa: F401

try:
    import genpy
    import rosbag
    import rospy
    import rosbags  # noqa: F401
    HAVE_ROS = True
except ImportError:
    HAVE_ROS = False


def _raw(msg):
    b = io.BytesIO()
    msg.serialize(b)
    return b.getvalue()


def make_bag(path):
    """Source bag written with rosbags (raw bytes) so that one topic can carry two connections (two publishers),
    which rosbag.Bag.write cannot produce; it is read back with the ROS rosbag API in the checks."""
    from rosbags.rosbag1 import Writer
    from sensor_msgs.msg import Imu, CompressedImage, NavSatFix
    from nav_msgs.msg import Odometry
    fake = genpy.dynamic.generate_dynamic("fl2g_test/Meas", "uint32 sat\nfloat64[] psr\nstring note\n")["fl2g_test/Meas"]
    t0 = 1735888008.0
    with Writer(path) as w:
        def conn(topic, cls, callerid, latching=0):
            pkg, name = cls._type.split("/")
            return w.add_connection(topic, "%s/msg/%s" % (pkg, name), msgdef=cls._full_text, md5sum=cls._md5sum,
                                    callerid=callerid, latching=latching)
        c_imu_a = conn("/livox/avia/imu", Imu, "/livox_lidar_publisher")
        c_imu_b = conn("/livox/avia/imu", Imu, "/other_publisher")
        c_imu_m = conn("/livox/mid360/imu", Imu, "/livox_lidar_publisher")
        c_img = conn("/camera/color/image_raw/compressed", CompressedImage, "/camera/realsense2_camera_manager")
        c_lla = conn("/ublox_driver/receiver_lla", NavSatFix, "/ublox_driver")
        c_rtk = conn("/mavros/global_position/raw/fix", NavSatFix, "/mavros")
        c_odom = conn("/odom", Odometry, "/wheeltec_robot")
        c_meas = conn("/ublox_driver/range_meas", fake, "/ublox_driver", 1)
        for k in range(200):
            t = rospy.Time.from_sec(t0 + 0.02 * k)
            ns = t.to_nsec()
            m = Imu()
            m.header.seq = k
            m.header.stamp = t
            m.angular_velocity.z = 0.001 * k
            w.write(c_imu_a if k % 2 == 0 else c_imu_b, ns, _raw(m))
            m2 = Imu()
            m2.header.seq = k
            m2.header.stamp = t
            m2.angular_velocity.z = 0.002 * k
            w.write(c_imu_m, ns, _raw(m2))
            if k % 10 == 0:
                c = CompressedImage()
                c.header.stamp = t
                c.format = "rgb8; jpeg compressed bgr8"
                c.data = bytes(range(256)) * 3
                w.write(c_img, ns, _raw(c))
                f = NavSatFix()
                f.header.stamp = t
                f.latitude = 34.0 + k * 1e-6
                w.write(c_lla, ns, _raw(f))
                w.write(c_rtk, ns, _raw(f))
                o = Odometry()
                o.header.stamp = t
                w.write(c_odom, ns, _raw(o))
                g = fake()
                g.sat = k
                g.psr = [2.2e7 + k, 2.3e7]
                g.note = "x" * k
                w.write(c_meas, ns, _raw(g))
    return t0


def stream(path, topics=None):
    out = []
    with rosbag.Bag(path) as b:
        for topic, msg, t in b.read_messages(topics=topics, raw=True):
            out.append((topic, t.to_nsec(), hashlib.sha256(msg[1]).hexdigest()))
    return out


def headers(path):
    with rosbag.Bag(path) as b:
        hs = []
        for c in b._get_connections():
            h = {k: (v.decode() if isinstance(v, bytes) else v) for k, v in c.header.items()
                 if k in ("topic", "type", "md5sum", "message_definition", "callerid", "latching")}
            hs.append(tuple(sorted(h.items())))
    return sorted(hs)


@unittest.skipUnless(HAVE_ROS, "blocked: ROS Python + rosbags (run tools/dev/run_tool_tests.sh, i.e. inside fastlivo2-gnss:noetic)")
class BagFilter(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.src = os.path.join(self.tmp.name, "in.bag")
        self.t0 = make_bag(self.src)

    def tearDown(self):
        self.tmp.cleanup()

    def run_filter(self, *extra):
        import bag_filter
        out = os.path.join(self.tmp.name, "out.bag")
        with contextlib.redirect_stdout(io.StringIO()):
            rc = bag_filter.main([self.src, out, "--force"] + list(extra))
        self.assertEqual(rc, 0)
        return out

    def test_keeps_data_chain_bytes_and_headers(self):
        out = self.run_filter()
        keep = ["/livox/avia/imu", "/camera/color/image_raw/compressed", "/ublox_driver/receiver_lla", "/ublox_driver/range_meas"]
        self.assertEqual(stream(out), stream(self.src, keep))
        with rosbag.Bag(out) as b:
            topics = set(b.get_type_and_topic_info().topics)
        self.assertEqual(topics, set(keep))
        src_h = [h for h in headers(self.src) if dict(h)["topic"] in keep]
        self.assertEqual(headers(out), sorted(src_h))
        callers = {dict(h)["callerid"] for h in headers(out) if dict(h)["topic"] == "/livox/avia/imu"}
        self.assertEqual(callers, {"/livox_lidar_publisher", "/other_publisher"})
        self.assertIn(("latching", "1"), [kv for h in headers(out) if dict(h)["topic"] == "/ublox_driver/range_meas" for kv in h])
        with open(out + ".sha256") as f:
            sha = f.read().split()[0]
        with open(out, "rb") as f:
            self.assertEqual(sha, hashlib.sha256(f.read()).hexdigest())

    def test_window_and_drop_gnss(self):
        out = self.run_filter("--start", "1.0", "--duration", "2.0", "--drop-gnss")
        got = stream(out)
        lo, hi = int(round((self.t0 + 1.0) * 1e9)), int(round((self.t0 + 3.0) * 1e9))
        exp = [s for s in stream(self.src, ["/livox/avia/imu", "/camera/color/image_raw/compressed"]) if lo <= s[1] < hi]
        self.assertEqual(got, exp)
        self.assertTrue(all(not s[0].startswith("/ublox_driver/") for s in got))
        self.assertGreater(len(got), 50)

    def test_lidar_mid360_keeps_the_mid360_pair_instead_of_the_avia_one(self):
        """A76: --lidar mid360 -> /livox/mid360/{lidar,imu}, the colour image and /ublox_driver/*; the Avia pair is dropped;
        the default (no --lidar) is the Avia pair, as before."""
        out = self.run_filter("--lidar", "mid360")
        keep = ["/livox/mid360/imu", "/camera/color/image_raw/compressed", "/ublox_driver/receiver_lla", "/ublox_driver/range_meas"]
        self.assertEqual(stream(out), stream(self.src, keep))
        with rosbag.Bag(out) as b:
            self.assertEqual(set(b.get_type_and_topic_info().topics), set(keep))
        src_h = [h for h in headers(self.src) if dict(h)["topic"] in keep]
        self.assertEqual(headers(out), sorted(src_h))
        explicit = stream(self.run_filter("--lidar", "avia"))
        default = stream(self.run_filter())
        self.assertEqual(explicit, default)
        self.assertNotIn("/livox/mid360/imu", {x[0] for x in default})

    def test_keep_extra_topic(self):
        out = self.run_filter("--keep", "/mavros/global_position/raw/fix")
        with rosbag.Bag(out) as b:
            self.assertIn("/mavros/global_position/raw/fix", b.get_type_and_topic_info().topics)


if __name__ == "__main__":
    unittest.main()
