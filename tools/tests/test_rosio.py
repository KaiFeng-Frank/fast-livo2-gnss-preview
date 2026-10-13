# SPDX-License-Identifier: BSD-3-Clause
"""rosio: the md5sum recomputed from a full message_definition equals the md5sum genpy compiled into the installed
ROS message packages (an independent oracle; covers constants, fixed/variable arrays, nested and array-of-message
fields, Header resolution), and the raw header parser agrees with genpy serialisation. Needs ROS Python (project
image); outside it the tests are skipped with the reason."""
import io
import unittest

import _path  # noqa: F401
from fl2g import rosio

try:
    import genpy  # noqa: F401
    import roslib.message
    HAVE_ROS = True
except ImportError:
    HAVE_ROS = False

TYPES = ("std_msgs/Header", "std_msgs/String", "sensor_msgs/Imu", "sensor_msgs/NavSatFix", "sensor_msgs/NavSatStatus",
         "sensor_msgs/CompressedImage", "sensor_msgs/PointCloud2", "nav_msgs/Odometry", "geometry_msgs/PoseWithCovarianceStamped",
         "diagnostic_msgs/DiagnosticArray", "visualization_msgs/MarkerArray")


@unittest.skipUnless(HAVE_ROS, "blocked: ROS Python (run tools/dev/run_tool_tests.sh, i.e. inside fastlivo2-gnss:noetic)")
class RosIo(unittest.TestCase):
    def test_md5_matches_genpy(self):
        checked = 0
        for t in TYPES:
            cls = roslib.message.get_message_class(t)
            if cls is None:
                continue
            self.assertEqual(rosio.ros_md5(t, cls._full_text), cls._md5sum, t)
            checked += 1
        self.assertGreaterEqual(checked, 8)

    def test_md5_detects_a_changed_definition(self):
        cls = roslib.message.get_message_class("sensor_msgs/NavSatFix")
        tampered = cls._full_text.replace("float64 altitude", "float32 altitude", 1)
        self.assertNotEqual(rosio.ros_md5("sensor_msgs/NavSatFix", tampered), cls._md5sum)
        tampered = cls._full_text.replace("int8 STATUS_FIX =      0", "int8 STATUS_FIX =      5", 1)
        self.assertNotEqual(rosio.ros_md5("sensor_msgs/NavSatFix", tampered), cls._md5sum)

    def test_parse_header(self):
        from sensor_msgs.msg import Imu
        m = Imu()
        m.header.seq = 7
        m.header.stamp.secs = 123
        m.header.stamp.nsecs = 456000000
        m.header.frame_id = "abc"
        m.angular_velocity.z = 0.25
        b = io.BytesIO()
        m.serialize(b)
        raw = b.getvalue()
        seq, st, frame, used = rosio.parse_header(raw)
        self.assertEqual((seq, frame, used), (7, "abc", 19))
        self.assertAlmostEqual(st, 123.456, places=9)
        import struct
        self.assertEqual(struct.unpack_from("<3d", raw, used + 104)[2], 0.25)     # the offset gt_audit reads the gyro at
        self.assertTrue(rosio.first_field_is_header("sensor_msgs/Imu", Imu._full_text))
        from std_msgs.msg import String
        self.assertFalse(rosio.first_field_is_header("std_msgs/String", String._full_text))

    def test_odom_twist_offsets(self):
        from nav_msgs.msg import Odometry
        import struct
        m = Odometry()
        m.header.frame_id = "odom_combined"
        m.child_frame_id = "base_footprint"
        m.twist.twist.linear.x = 0.75
        m.twist.twist.angular.z = -0.125
        b = io.BytesIO()
        m.serialize(b)
        raw = b.getvalue()
        _s, _t, _f, used = rosio.parse_header(raw)
        (n,) = struct.unpack_from("<I", raw, used)
        off = used + 4 + n + 56 + 288
        self.assertEqual(struct.unpack_from("<3d", raw, off)[0], 0.75)
        self.assertEqual(struct.unpack_from("<3d", raw, off + 24)[2], -0.125)


if __name__ == "__main__":
    unittest.main()
