# SPDX-License-Identifier: BSD-3-Clause
"""Unit tests of tools/calib/host_gt_lever.py (AMENDMENTS A77): python3 -m unittest discover -s tools/calib/tests"""
import math
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import host_gt_lever as H  # noqa: E402
import p4_calib as P4  # noqa: E402

# lidars.json R_level_from_imu of the MID-360 (calibration row avia -> mid360 transposed): IMU axes = R_AM @ level axes
R_LEVEL_FROM_IMU = np.array([0.849802, 0.0393196, 0.525632, -0.0398291, 0.999153, -0.0103484, -0.525594, -0.0121414, 0.850649]).reshape(3, 3)
R_AM = R_LEVEL_FROM_IMU.T


def rot(axis, a):
    c, s = math.cos(a), math.sin(a)
    return {"x": np.array([[1, 0, 0], [0, c, -s], [0, s, c]]), "y": np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]]),
            "z": np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])}[axis]


def quat_of(R):
    """x y z w of a rotation matrix (Shepperd)."""
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return [(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s]
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k]) * 2
    q = [0.0] * 4
    q[i] = 0.25 * s
    q[j] = (R[j, i] + R[i, j]) / s
    q[k] = (R[k, i] + R[i, k]) / s
    q[3] = (R[k, j] - R[j, k]) / s
    return q


def synth(R_il, v_level, n=4000, noise=0.0, seed=0):
    """A ground vehicle with varying curvature, small roll/pitch; the host IMU frame is the level frame rotated by R_il
    (x_imu = R_il x_level); the GT antenna is p + R_wb v in the host's world, seen through an unknown SE(3) and noise."""
    rng = np.random.default_rng(seed)
    t = np.arange(n) * 0.05
    psi = 2.0 * np.sin(0.2 * t) + 0.05 * t
    p = np.cumsum(1.2 * 0.05 * np.stack([np.cos(psi), np.sin(psi), 0 * psi], axis=1), axis=0)
    roll, pitch = 0.03 * np.sin(0.37 * t), 0.04 * np.sin(0.23 * t + 1.0)
    R_wb = np.array([rot("z", a) @ rot("y", b) @ rot("x", c) for a, b, c in zip(psi, pitch, roll)])
    R_t = np.array([R @ R_il.T for R in R_wb])                 # W <- IMU
    g_w = p + np.einsum("nij,j->ni", R_wb, np.asarray(v_level, float))
    Rg = rot("z", 0.7) @ rot("y", 0.3) @ rot("x", -0.2)
    g = g_w @ Rg.T + np.array([3.0, -2.0, 1.0]) + noise * rng.normal(size=g_w.shape)
    return {"t": t, "p": p, "R": R_t, "g": g}


class FitLevelLever(unittest.TestCase):
    def test_recovers_the_level_components_through_a_tilted_imu_frame(self):
        v = np.array([-0.049, -0.152, -0.0297])
        P = synth(R_AM, v, noise=0.002)
        l, vf, rms = H.fit_level_lever(P, R_AM, v[2])
        np.testing.assert_allclose(vf[:2], v[:2], atol=3e-4)
        np.testing.assert_allclose(l, R_AM @ vf, atol=1e-12)               # the IMU-frame vector is R_am [vx, vy, vz]
        self.assertLess(rms, 0.004)

    def test_noise_free_is_exact_and_the_held_vertical_only_moves_it_at_tilt_size(self):
        v = np.array([-0.2, 0.1, 0.4])
        P = synth(np.eye(3), v)
        l, vf, rms = H.fit_level_lever(P, np.eye(3), v[2])
        np.testing.assert_allclose(vf, v, atol=1e-7)
        self.assertLess(rms, 1e-7)
        l, vf, rms = H.fit_level_lever(P, np.eye(3), v[2] + 0.1)           # a wrong held vertical: roll/pitch (~2 deg) scale
        self.assertLess(np.abs(vf[:2] - v[:2]).max(), 0.1 * 0.05)

    def test_identity_split_is_p4_calibs_fit_lever_step_for_step(self):
        v = np.array([-0.176, -0.016, 0.4006])
        P = synth(np.eye(3), v, noise=0.01, seed=3)
        l_ref, rms_ref = P4.fit_lever(P, [0.0, 0.0, v[2]], iters=H.ITERS)
        l, vf, rms = H.fit_level_lever(P, np.eye(3), v[2])
        np.testing.assert_allclose(l, l_ref, atol=1e-12)
        self.assertAlmostEqual(rms, rms_ref, places=12)


class DataPaths(unittest.TestCase):
    def test_prep_est_interpolates_the_gt_and_drops_rows_in_a_gt_gap(self):
        tg = np.array([0.0, 0.2, 0.4, 0.6, 3.0, 3.2, 3.4])
        gt = np.stack([tg, 10 * tg, 0 * tg, 0 * tg, 0 * tg, 0 * tg, 0 * tg, 1 + 0 * tg], axis=1)
        te = np.array([-1.0, 0.3, 0.5, 1.5, 3.1, 5.0])
        est = np.stack([te, 9 + 0 * te, 9 + 0 * te, 9 + 0 * te, 0 * te, 0 * te, 0 * te, 1 + 0 * te], axis=1)
        d = H.prep_est(est, gt)
        np.testing.assert_allclose(d["t"], [0.3, 0.5, 3.1])               # 1.5 sits in the 2.4 s hole (> 0.5 s); -1 and 5 are outside
        np.testing.assert_allclose(d["g"][:, 0], [3.0, 5.0, 31.0])         # linear interpolation of x = 10 t
        np.testing.assert_allclose(d["R"][0], np.eye(3))

    def test_quaternion_convention_is_p4_calibs(self):
        R = rot("z", 0.4) @ rot("y", -0.2) @ rot("x", 0.1)
        np.testing.assert_allclose(P4.quat_to_R(quat_of(R)), R, atol=1e-12)


class Summary(unittest.TestCase):
    def test_a58_literal_seed_median_and_all_run_medians(self):
        runs = []
        for seq, vs in (("A", [(-0.10, 0.0), (-0.12, 0.0), (-0.20, 0.0)]), ("B", [(-0.30, 0.1), (-0.32, 0.1), (-0.34, 0.1)])):
            for i, (x, y) in enumerate(vs):
                runs.append({"seq": seq, "name": "%s%d" % (seq, i), "v": [x, y, 0.0]})
        s = H.summarise(runs)
        self.assertEqual(s["per_sequence_first_run"], {"A": [-0.10, 0.0], "B": [-0.30, 0.1]})
        np.testing.assert_allclose(s["per_sequence_seed_median"]["A"], [-0.12, 0.0])
        np.testing.assert_allclose(s["a58_literal_median_over_sequences"], [-0.20, 0.05])        # median of two = their mean
        np.testing.assert_allclose(s["seed_median_then_sequence_median"], [-0.22, 0.05])
        np.testing.assert_allclose(s["median_over_all_runs"], [-0.25, 0.05])
        self.assertEqual(s["runs_per_sequence"], {"A": 3, "B": 3})


if __name__ == "__main__":
    unittest.main()
