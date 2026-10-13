# SPDX-License-Identifier: BSD-3-Clause
"""extrinsics.tilt_correction: the smallest rotation that makes the scans' ground normal and the accelerometer's up agree."""
import math
import unittest

import numpy as np

import _path  # noqa: F401
from smartpnt import extrinsics

R_A = np.array([[0, 0, -1], [0, -1, 0], [-1, 0, 0]], dtype=float)


def rot(axis, deg):
    k = np.asarray(axis, dtype=float)
    k = k / np.linalg.norm(k)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    a = math.radians(deg)
    return np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * (K @ K)


def report(R_il, tilt, up_l):
    """A static window whose accelerometer up is the LiDAR up mapped by R_il and then tilted by `tilt`."""
    up_i = tilt @ (R_il @ np.asarray(up_l, dtype=float))
    return {"up_in_lidar": list(up_l), "up_in_imu": list(up_i)}


class TiltCorrection(unittest.TestCase):
    def test_recovers_a_known_longitudinal_tilt(self):
        tilt = rot([0, 0, 1], 1.66)                        # about the I300 z axis (the vehicle's longitudinal axis)
        reps = [report(R_A, tilt, [0.0, 0.0, 1.0]), report(R_A, tilt, [0.006, -0.012, 0.99991])]
        R_t, before, after = extrinsics.tilt_correction(R_A, reps)
        self.assertAlmostEqual(math.degrees(math.acos((np.trace(R_t) - 1) / 2)), 1.66, places=2)
        self.assertTrue(all(abs(b - 1.66) < 0.05 for b in before))
        self.assertTrue(all(a < 0.02 for a in after))
        self.assertTrue(np.allclose(R_t @ R_t.T, np.eye(3), atol=1e-12))
        self.assertAlmostEqual(np.linalg.det(R_t), 1.0)

    def test_no_tilt_is_the_identity(self):
        R_t, before, after = extrinsics.tilt_correction(R_A, [report(R_A, np.eye(3), [0.0, 0.0, 1.0])])
        self.assertTrue(np.allclose(R_t, np.eye(3)))

    def test_windows_that_disagree_leave_a_residual_the_report_shows(self):
        reps = [report(R_A, rot([0, 0, 1], 1.0), [0.0, 0.0, 1.0]), report(R_A, rot([0, 0, 1], 2.0), [0.0, 0.0, 1.0])]
        R_t, before, after = extrinsics.tilt_correction(R_A, reps)
        self.assertAlmostEqual(math.degrees(math.acos((np.trace(R_t) - 1) / 2)), 1.5, places=2)
        self.assertTrue(all(abs(a - 0.5) < 0.05 for a in after))

    def test_the_rotation_axis_is_horizontal(self):
        # only the vertical is moved: the axis of the correction is perpendicular to the vertical it takes from one place to the other
        tilt = rot([0, 0, 1], 1.66)
        R_t, _, _ = extrinsics.tilt_correction(R_A, [report(R_A, tilt, [0.0, 0.0, 1.0])])
        a = R_A @ np.array([0.0, 0.0, 1.0])
        w, v = np.linalg.eig(R_t)
        axis = np.real(v[:, int(np.argmin(np.abs(w - 1.0)))])
        self.assertAlmostEqual(float(axis @ a), 0.0, places=6)
        self.assertTrue(np.allclose(R_t @ a, tilt @ a, atol=1e-9))


if __name__ == "__main__":
    unittest.main()
