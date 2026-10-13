# SPDX-License-Identifier: BSD-3-Clause
"""Unit tests of the P4 calibration helpers (AMENDMENTS A58): python3 -m unittest discover -s tools/calib/tests"""
import math
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import p4_calib as P  # noqa: E402


class Helpers(unittest.TestCase):
    def test_kappa_is_the_ar1_sample_mean_inflation(self):
        self.assertAlmostEqual(P.kappa(10.0, 1.0), (1 + math.exp(-0.1)) / (1 - math.exp(-0.1)), places=12)
        self.assertEqual(P.kappa(0.0, 1.0), 1.0)

    def test_nnls_small_matches_the_active_set_solution(self):
        rng = np.random.default_rng(1)
        A = rng.random((40, 3))
        x_true = np.array([2.0, 0.0, 0.5])
        b = A @ x_true
        np.testing.assert_allclose(P.nnls_small(A, b), x_true, atol=1e-10)
        x = P.nnls_small(A, A @ np.array([1.0, -3.0, 1.0]))
        self.assertTrue(np.all(x >= 0))

    def test_epoch_ls_redundancy_sums_to_dof_and_snooping_removes_a_blunder(self):
        rng = np.random.default_rng(2)
        el = rng.uniform(15, 85, 12)
        az = rng.uniform(0, 360, 12)
        A = P.design("p", ["G%02d" % i for i in range(12)], el, az)
        nu = A @ np.array([1.0, -2.0, 0.5, 30.0]) + rng.normal(0, 1.0, 12)
        R = np.ones(12)
        e, r, act, w, dof = P.epoch_ls(nu, R, A, snoop=False)
        self.assertEqual(dof, 8)
        self.assertAlmostEqual(float(r.sum()), 8.0, places=9)
        nu[5] += 25.0
        e, r, act, w, dof = P.epoch_ls(nu, R, A, snoop=True)
        self.assertFalse(act[5])
        self.assertEqual(dof, 7)

    def test_tau_of_an_ar1_series(self):
        rng = np.random.default_rng(3)
        rho = math.exp(-1.0 / 5.0)
        x = np.zeros(20000)
        for k in range(1, len(x)):
            x[k] = rho * x[k - 1] + math.sqrt(1 - rho * rho) * rng.normal()
        acf = []
        for L in range(1, 41):
            acf.append({"lag": L, "lag_s": float(L), "rho": float(np.corrcoef(x[:-L], x[L:])[0, 1]), "pairs": len(x) - L})
        t = P.fit_tau(acf, 1.0)
        self.assertAlmostEqual(t["tau_efold_s"], 5.0, delta=0.5)
        self.assertAlmostEqual(t["kappa_integrated"], P.kappa(5.0, 1.0), delta=1.0)

    def test_changepoint_finds_the_tail_step(self):
        bins = [{"lo": lo, "hi": lo + 2, "n": 500, "tail": (0.05 if lo < 20 else 0.001)} for lo in range(12, 32, 2)]
        c = P.changepoint(bins)
        self.assertEqual(c["edge"], 20)
        self.assertTrue(c["accepted"])
        flat = [{"lo": lo, "hi": lo + 2, "n": 500, "tail": 0.002} for lo in range(12, 32, 2)]
        c = P.changepoint(flat)
        self.assertTrue(c is None or not c["accepted"])

    def test_allan_of_white_frequency_noise(self):
        rng = np.random.default_rng(4)
        t = np.arange(20000) * 0.1
        y = rng.normal(0, 0.1, t.size)
        av = P.allan_freq(t, y, 0.1)
        for a in av[:6]:     # white FM: σ²(τ) = σ²·τ0/τ
            self.assertAlmostEqual(a["avar"], 0.01 * 0.1 / a["tau_s"], delta=0.2 * 0.01 * 0.1 / a["tau_s"])


class FrameFixedLever(unittest.TestCase):
    """`frame --lever-fixed` (A78): a host whose IMU frame is tilted against the vertical gets its GT antenna lever from
    host_gt_lever.py (lidars.json gt_lever_imu) instead of the A58 per-sequence fit, which takes the first two IMU-frame
    components to be horizontal. The default path must stay as it was."""

    @classmethod
    def setUpClass(cls):
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        import test_host_gt_lever as T
        cls.T = T

    def _level_world_synth(self, v_level, n=4000, noise=0.01, seed=5):
        """test_host_gt_lever.synth with a GT frame whose z axis is the vertical of the host world (the alignment turns about z only),
        as it is for a ground vehicle on level ground; the level IMU frame (identity mount)."""
        T = self.T
        rng = np.random.default_rng(seed)
        t = np.arange(n) * 0.05
        psi = 2.0 * np.sin(0.2 * t) + 0.05 * t
        p = np.cumsum(1.2 * 0.05 * np.stack([np.cos(psi), np.sin(psi), 0 * psi], axis=1), axis=0)
        roll, pitch = 0.03 * np.sin(0.37 * t), 0.04 * np.sin(0.23 * t + 1.0)
        R_wb = np.array([T.rot("z", a) @ T.rot("y", b) @ T.rot("x", c) for a, b, c in zip(psi, pitch, roll)])
        g_w = p + np.einsum("nij,j->ni", R_wb, np.asarray(v_level, float))
        g = g_w @ T.rot("z", 0.7).T + np.array([3.0, -2.0, 1.0]) + noise * rng.normal(size=g_w.shape)
        return {"t": t, "p": p, "R": R_wb, "g": g}

    def _run(self, P, extra, tmp, alpha="heading"):
        import argparse
        import contextlib
        import io
        from unittest import mock
        out = os.path.join(tmp, "frame.json")
        args = argparse.Namespace(run=["S=/nowhere"], root="/nowhere", lever=[-0.31635, 0.10926, 0.4006], lever_fixed=extra, alpha=alpha, out=out)
        P = dict(P, gt_method="synthetic")
        with mock.patch.object(P4mod, "frame_prep", return_value=P), mock.patch("evalcommon.load_sequences", return_value=({}, {"S": {}})), \
                contextlib.redirect_stdout(io.StringIO()):
            P4mod.cmd_frame(args)
        with open(out) as f:
            return json.load(f)

    def test_fixed_lever_replaces_the_fit_and_the_default_is_unchanged(self):
        import tempfile
        v = np.array([-0.176, -0.016, 0.4006])
        Pd = self._level_world_synth(v)
        with tempfile.TemporaryDirectory() as tmp:
            d0 = self._run(Pd, None, tmp)
            self.assertNotIn("lever_arm_I_gt_fixed", d0)
            self.assertEqual(list(d0["lever_arm_I_gt_per_seq"]), ["S"])
            l_fit = d0["lever_arm_I_gt_used"]
            # the same lever held reproduces the default's per-trajectory fit bit for bit, and says it was held
            d1 = self._run(Pd, l_fit, tmp)
            self.assertTrue(d1["lever_arm_I_gt_fixed"])
            self.assertEqual(d1["lever_arm_I_gt_per_seq"], {})
            self.assertEqual(d1["lever_arm_I_gt_used"], l_fit)
            self.assertEqual(d1["trajectories"][0]["fit"], d0["trajectories"][0]["fit"])
            self.assertEqual(d1["quantiles"], d0["quantiles"])
            # a different held lever is used as given
            d2 = self._run(Pd, [-0.4, 0.2, 0.4006], tmp)
            self.assertEqual(d2["lever_arm_I_gt_used"], [-0.4, 0.2, 0.4006])
            self.assertNotEqual(d2["trajectories"][0]["fit"], d0["trajectories"][0]["fit"])
            # the turn measure is recorded when it is not the A58 one; it only moves the α column of the segment rows (the errors do
            # not depend on it), and on a level IMU by about a percent
            self.assertNotIn("alpha_measure", d0)
            d3 = self._run(Pd, l_fit, tmp, alpha="vertical")
            self.assertEqual(d3["alpha_measure"], "vertical")
            rows_h, _ = P4mod.frame_segments(Pd, np.array(l_fit))
            rows_v, _ = P4mod.frame_segments(Pd, np.array(l_fit), "vertical")
            np.testing.assert_array_equal(rows_h[:, :4], rows_v[:, :4])
            self.assertAlmostEqual(rows_v[:, 4].mean() / rows_h[:, 4].mean(), 1.0, delta=0.02)

    def test_tilted_imu_frame_with_the_right_lever_has_no_segment_error_and_a_wrong_one_does(self):
        T = self.T
        v_level = np.array([-0.05, -0.15, -0.03])
        Pd = T.synth(T.R_AM, v_level, noise=0.0, seed=2)               # MID-360-like IMU frame, exact GT antenna
        l_ok = T.R_AM @ v_level
        rows, path = P4mod.frame_segments(Pd, l_ok)
        self.assertGreater(len(rows), 50)
        self.assertLess(np.abs(rows[:, 1:4]).max(), 1e-6)
        rows_bad, _ = P4mod.frame_segments(Pd, l_ok + T.R_AM @ np.array([0.0, 0.3, 0.0]))     # 0.3 m off sideways
        self.assertGreater(np.abs(rows_bad[:, 1:3]).max(), 0.02)
        # the IMU-frame lever mixes the level components (that is why A58's "first two components are horizontal" fit does not
        # carry over to this host and the lever is held instead)
        self.assertGreater(np.linalg.norm(l_ok[:2] - v_level[:2]), 0.02)


class TurnMeasure(unittest.TestCase):
    """`turn_steps` (A78): the accumulated turn α that regresses q_ψr. 'heading' is A58's (heading of the IMU x axis), right when the
    IMU frame is level; 'vertical' is the rotation about the vertical, right for any mount."""

    @staticmethod
    def _orientations(R_mount, yaw_rate=0.3, roll_amp=0.15, pitch=0.05, dt=0.02, n=3000):
        """W <- IMU orientations of a vehicle that turns at a constant rate while rolling at 1 Hz: R_wi = R_w,level · R_mount with
        R_mount = lidars.json R_level_from_imu (the IMU frame's axes as seen from the level frame). True accumulated turn = yaw_rate (n-1) dt."""
        t = np.arange(n) * dt
        Rs = []
        for tt in t:
            c, s_ = math.cos(yaw_rate * tt), math.sin(yaw_rate * tt)
            Rz = np.array([[c, -s_, 0], [s_, c, 0], [0, 0, 1.0]])
            Ry = np.array([[math.cos(pitch), 0, math.sin(pitch)], [0, 1, 0], [-math.sin(pitch), 0, math.cos(pitch)]])
            ph = roll_amp * math.sin(2 * math.pi * tt)
            Rx = np.array([[1, 0, 0], [0, math.cos(ph), -math.sin(ph)], [0, math.sin(ph), math.cos(ph)]])
            Rs.append(Rz @ Ry @ Rx @ R_mount)
        return np.array(Rs), yaw_rate * (n - 1) * dt

    def test_level_imu_both_measures_are_the_true_turn(self):
        Rs, truth = self._orientations(np.eye(3))
        Rg = np.eye(3)
        h = P.turn_steps(Rg, Rs, "heading").sum()
        v = P.turn_steps(Rg, Rs, "vertical").sum()
        self.assertAlmostEqual(h / truth, 1.0, delta=0.01)
        self.assertAlmostEqual(v / truth, 1.0, delta=0.01)

    def test_tilted_imu_frame_the_heading_of_x_picks_up_the_roll_and_the_vertical_measure_does_not(self):
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        import test_host_gt_lever as T
        Rs, truth = self._orientations(T.R_LEVEL_FROM_IMU)          # the MID-360 mount (31.8 degrees)
        Rg = np.eye(3)
        h = P.turn_steps(Rg, Rs, "heading").sum()
        v = P.turn_steps(Rg, Rs, "vertical").sum()
        self.assertAlmostEqual(v / truth, 1.0, delta=0.01)
        self.assertGreater(h / truth, 1.3)
        # the global alignment (a rotation about any axis) does not change what the vertical measure counts when it keeps the vertical
        c, s_ = math.cos(0.7), math.sin(0.7)
        Rz = np.array([[c, -s_, 0], [s_, c, 0], [0, 0, 1.0]])
        np.testing.assert_allclose(P.turn_steps(Rz, Rs, "vertical"), P.turn_steps(np.eye(3), Rs, "vertical"), atol=1e-12)

    def test_unknown_measure_is_refused(self):
        with self.assertRaises(ValueError):
            P.turn_steps(np.eye(3), np.eye(3)[None].repeat(3, axis=0), "course")


import json  # noqa: E402
P4mod = P

if __name__ == "__main__":
    unittest.main()
