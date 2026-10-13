# SPDX-License-Identifier: BSD-3-Clause
"""eval_ate.py: evo's association/Umeyama re-implementation; E1 over the whole sequence with the A52 completion of the
GNSS arms (activation-time T_{N<-W} before ACTIVATE, traj_N after it, never-activated runs = host); the "30 s after
activation" span and E4; the A54 GT (time base record, ATE mask removing stretches from GT and estimates, legacy
audits); the instrument self-checks (heading of T_{N<-W}, ACTIVATE) and the A55/I11 protocol checks (one binary,
pre-activation host = U); per-cell statistics, seed collapse, the A2 guard (an evaluation sequence is refused before
any of its files is read) and — where evo works (the project image after tools/eval/setup_evo_env.sh) — agreement of
evo_ape with the numpy re-implementation on every metric."""
import csv
import json
import math
import os
import shutil
import subprocess
import tempfile
import unittest
from unittest import mock

import numpy as np

import _path  # noqa: F401
import e1traj
import eval_ate as ea
import evalcommon as ec

PYLIB = os.environ.get("FL2G_EVO_PYLIB", "/r/tools/eval_pylib/site")
B, D = "b" * 64, "d" * 64


def rot(yaw, pitch=0.0, roll=0.0):
    cz, sz, cy, sy, cx, sx = np.cos(yaw), np.sin(yaw), np.cos(pitch), np.sin(pitch), np.cos(roll), np.sin(roll)
    rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    return rz @ ry @ rx


def tum(t, p, r=None):
    a = np.zeros((len(t), 8))
    a[:, 0], a[:, 1:4] = t, p
    if r is None:
        a[:, 7] = 1.0
    else:
        a[:, 4:8] = e1traj.rot_to_quat(r)
    return a


def text_round(a):
    """The array a %.9f TUM file gives back (what the replay writes, what the tools read)."""
    return np.array([[float("%.9f" % x) for x in row] for row in a])


def gt_path(n=600, dt=0.2, t0=1000.0, seed=1):
    rng = np.random.default_rng(seed)
    t = t0 + dt * np.arange(n)
    v = np.cumsum(rng.normal(0, 0.05, (n, 3)), axis=0) * 0.1 + np.array([1.0, 0.3, 0.0])
    return t, np.cumsum(v * dt, axis=0)


class Math(unittest.TestCase):
    def test_association_matches_evo_rule(self):
        ref = np.array([0.0, 1.0, 2.0, 3.0])
        est = np.array([0.04, 0.96, 1.5, 2.051, 2.95, 3.02, 4.0])
        # est longer: every ref stamp takes its nearest est stamp if within max_diff; 2.0 has none (2.051 is 51 ms off),
        # 3.0 takes 3.02 (20 ms) over 2.95 (50 ms)
        self.assertEqual(ea.evo_associate(ref, est, 0.05), ([0, 1, 3], [0, 1, 5]))
        # ref longer: the est stamps are scanned instead (est 1.5 has no ref stamp within 50 ms)
        self.assertEqual(ea.evo_associate(ref, est[:3], 0.05), ([0, 1], [0, 1]))
        # the bound is inclusive and an exact tie takes the first minimum (np.argmin), as evo does
        self.assertEqual(ea.evo_associate(np.array([1.0]), np.array([0.75, 1.25, 5.0]), 0.25), ([0], [0]))
        # a shorter estimate is scanned: two estimate stamps may take the same reference stamp (evo allows it)
        self.assertEqual(ea.evo_associate(np.array([1.0, 9.0, 9.5]), np.array([0.75, 1.25]), 0.25), ([0, 0], [0, 1]))

    def test_umeyama_recovers_se3(self):
        rng = np.random.default_rng(3)
        x = rng.normal(size=(50, 3)) * 10
        r0, t0 = rot(0.7, 0.1, -0.2), np.array([5.0, -2.0, 1.0])
        r, t = ea.umeyama_se3(x, x @ r0.T + t0)
        self.assertLess(np.max(np.abs(r - r0)), 1e-12)
        self.assertLess(np.max(np.abs(t - t0)), 1e-10)

    def test_ate_invariant_to_se3_and_equal_to_noise(self):
        t, p = gt_path()
        gt = tum(t, p)
        r0, t0 = rot(1.2), np.array([10.0, 20.0, -3.0])
        est_t = np.arange(t[0] - 1.0, t[-1] + 1.0, 1 / 30.0)
        est_p = np.stack([np.interp(est_t, t, p[:, i]) for i in range(3)], axis=1)
        exact = ea.ate_numpy(gt, tum(est_t, (est_p - t0) @ r0), t[0], t[-1])
        self.assertLess(exact["rmse"], 1e-9)
        self.assertEqual(exact["n_pairs"], len(t))
        rng = np.random.default_rng(5)
        noisy = est_p + rng.normal(0, 0.1, est_p.shape)
        res = ea.ate_numpy(gt, tum(est_t, noisy), t[0], t[-1])
        self.assertGreater(res["rmse"], 0.14)                     # sqrt(3) * 0.1, minus what the alignment absorbs
        self.assertLess(res["rmse"], 0.19)
        cut = ea.ate_numpy(gt, tum(est_t, noisy), t[100], t[200])
        self.assertEqual(cut["n_ref_in_span"], 101)

    @staticmethod
    def line_gt(n=100, dt=1.0, t0=1000.0, speed=10.0):
        """A GT polyline with a turn every 20 rows (so a chord of two segments is shorter than their sum)."""
        t = t0 + dt * np.arange(n)
        heading = 0.5 * (np.arange(n) // 20)
        step = speed * dt * np.stack([np.cos(heading), np.sin(heading), np.zeros(n)], axis=1)
        return tum(t, np.cumsum(step, axis=0))

    def test_path_ratio_is_one_for_the_same_path_at_any_rigid_pose_and_follows_the_scale(self):
        gt = self.line_gt()
        est_t = np.arange(999.0, 1100.0, 0.1)
        est_p = np.stack([np.interp(est_t, gt[:, 0], gt[:, i]) for i in (1, 2, 3)], axis=1)
        r0, t0 = rot(1.1, 0.2, -0.3), np.array([30.0, -4.0, 8.0])
        same = ea.path_ratio(gt, tum(est_t, est_p @ r0.T + t0), gt[0, 0], gt[-1, 0])
        self.assertAlmostEqual(same["ratio"], 1.0, places=9)
        self.assertEqual((same["n_rows"], same["n_segments"], same["n_segments_skipped"]), (100, 99, 0))
        self.assertAlmostEqual(same["gt_len_m"], same["est_len_m"], places=6)
        longer = ea.path_ratio(gt, tum(est_t, 1.2 * est_p), gt[0, 0], gt[-1, 0])
        self.assertAlmostEqual(longer["ratio"], 1.2, places=9)
        frozen = ea.path_ratio(gt, tum(est_t, np.zeros_like(est_p)), gt[0, 0], gt[-1, 0])
        self.assertEqual(frozen["ratio"], 0.0)                                   # the pose that does not move (A81: outside the band)
        self.assertTrue(ea.path_band_flag(frozen["ratio"]))
        self.assertTrue(ea.path_band_flag(1.06) and ea.path_band_flag(0.949) and ea.path_band_flag(None))
        self.assertFalse(ea.path_band_flag(0.95) or ea.path_band_flag(1.0) or ea.path_band_flag(1.05))

    def test_path_ratio_uses_gt_rows_at_most_once_a_second(self):
        gt5 = self.line_gt(n=500, dt=0.2)                                        # a 5 Hz GT: thinned to every fifth row
        est_t = np.arange(999.0, 1101.0, 0.05)
        est_p = np.stack([np.interp(est_t, gt5[:, 0], gt5[:, i]) for i in (1, 2, 3)], axis=1)
        res = ea.path_ratio(gt5, tum(est_t, est_p), gt5[0, 0], gt5[-1, 0])
        self.assertEqual(res["n_rows"], 100)
        self.assertAlmostEqual(res["ratio"], 1.0, places=9)
        # the 1 Hz chord, not the 5 Hz path: a zig-zag between the 1 Hz rows is invisible to both sides
        zig = est_p + np.where((np.round(est_t * 5) % 5 != 0)[:, None], np.array([0.0, 3.0, 0.0]), 0.0)
        self.assertAlmostEqual(ea.path_ratio(gt5, tum(est_t, zig), gt5[0, 0], gt5[-1, 0])["ratio"], 1.0, places=9)

    def test_path_ratio_counts_only_segments_both_sides_have(self):
        gt = self.line_gt()
        est_t = np.arange(999.0, 1100.0, 0.1)
        est_p = np.stack([np.interp(est_t, gt[:, 0], gt[:, i]) for i in (1, 2, 3)], axis=1)
        # a masked stretch (GT rows 40..49 removed, as the A54 mask does): the two segments that would bridge it are not segments;
        # the estimate inside it is wild and must not count either
        keep = np.ones(len(gt), dtype=bool)
        keep[40:50] = False
        wild = est_p.copy()
        wild[(est_t >= 1040.0) & (est_t <= 1049.0)] *= 7.0
        res = ea.path_ratio(gt[keep], tum(est_t, wild), gt[0, 0], gt[-1, 0])
        self.assertEqual((res["n_rows"], res["n_segments"], res["n_segments_skipped"]), (90, 88, 1))
        self.assertAlmostEqual(res["ratio"], 1.0, places=9)
        # the estimate has a hole around GT row 60: both segments touching it are dropped from both paths
        hole = (est_t < 1059.9) | (est_t > 1060.1)
        res = ea.path_ratio(gt, tum(est_t[hole], est_p[hole]), gt[0, 0], gt[-1, 0])
        self.assertEqual((res["n_segments"], res["n_segments_skipped"]), (97, 2))
        self.assertAlmostEqual(res["ratio"], 1.0, places=9)
        # a span that holds no GT row, a still GT, an empty estimate
        with self.assertRaises(ValueError):
            ea.path_ratio(gt, tum(est_t, est_p), 5000.0, 6000.0)
        with self.assertRaises(ValueError):
            ea.path_ratio(gt, np.zeros((0, 8)), gt[0, 0], gt[-1, 0])
        still = tum(gt[:, 0], np.zeros((len(gt), 3)))
        self.assertIsNone(ea.path_ratio(still, tum(est_t, est_p), gt[0, 0], gt[-1, 0])["ratio"])


class LeverArm(unittest.TestCase):
    """E1' (TEST_PLAN §4, A64): the GT antenna lever arm estimated jointly with the SE(3) alignment."""

    @staticmethod
    def walk(n=1500, dt=0.1, tilt=0.03, seed=2):
        """A ground-vehicle IMU trajectory: turning (yaw sweeps several turns), roll/pitch wobbling by ~tilt rad."""
        rng = np.random.default_rng(seed)
        k = np.arange(n)
        yaw = 0.004 * k + 0.6 * np.sin(0.01 * k)
        pitch = tilt * np.sin(0.037 * k + 0.3)
        roll = tilt * np.cos(0.029 * k)
        rb = np.stack([rot(a, b, c) for a, b, c in zip(yaw, pitch, roll)])
        v = 1.2 * np.stack([np.cos(yaw), np.sin(yaw), 0.02 * np.sin(0.01 * k)], axis=1)
        p = np.cumsum(v * dt, axis=0) + rng.normal(0, 1e-4, (n, 3))
        return 100.0 + dt * k, p, rb

    def gt_of(self, t, p, rb, lever, noise, seed=4):
        r0, t0 = rot(0.9, 0.01, -0.02), np.array([-40.0, 12.0, 3.0])
        rng = np.random.default_rng(seed)
        y = (p + rb @ np.asarray(lever)) @ r0.T + t0 + rng.normal(0, noise, p.shape)
        return tum(t, y)

    def test_recovers_the_lever_arm_and_removes_its_floor(self):
        t, p, rb = self.walk()
        lever = np.array([-0.176, -0.016, 0.401])
        gt = self.gt_of(t, p, rb, lever, 0.01)
        est = tum(t, p, rb)
        plain = ea.ate_numpy(gt, est, t[0], t[-1])
        res = ea.ate_lever_numpy(gt, est, t[0], t[-1])
        self.assertGreater(plain["rmse"], 0.08)                   # the horizontal lever turning with the heading
        self.assertLess(res["rmse"], 0.02)                        # sqrt(3) * 1 cm noise, minus the 9 fitted parameters
        self.assertGreater(res["rmse"], 0.015)
        self.assertLess(np.max(np.abs(np.array(res["lever_m"][:2]) - lever[:2])), 2e-3)
        self.assertLess(abs(res["lever_m"][2] - lever[2]), 0.05)  # vertical: observable through the tilt wobble only
        self.assertGreater(res["lever_sigma_m"][2], 5 * res["lever_sigma_m"][0])
        self.assertAlmostEqual(res["rmse_lever0"], plain["rmse"], delta=1e-9)
        self.assertLess(abs(res["rmse_restart"] - res["rmse"]), 1e-9)
        self.assertEqual(ea.lever_problems(res, plain["rmse"]), [])
        self.assertTrue(res["converged"])
        ref = ea.ate_lever_numpy(gt, est, t[0], t[-1], lever_ref=lever)
        self.assertLess(ref["rmse_at_lever_ref"], 0.02)
        self.assertGreaterEqual(ref["rmse_at_lever_ref"], res["rmse"] - 1e-12)

    def test_no_lever_leaves_the_ate(self):
        t, p, rb = self.walk(seed=6)
        gt = self.gt_of(t, p, rb, [0.0, 0.0, 0.0], 0.05)
        est = tum(t, p, rb)
        plain = ea.ate_numpy(gt, est, t[0], t[-1])
        res = ea.ate_lever_numpy(gt, est, t[0], t[-1])
        self.assertLessEqual(res["rmse"], plain["rmse"])
        self.assertGreater(res["rmse"], 0.99 * plain["rmse"])     # nothing to absorb but noise
        self.assertLess(np.max(np.abs(res["lever_m"][:2])), 0.01)

    def test_planar_motion_leaves_the_vertical_unobservable(self):
        """Yaw only: R_i e_z = e_z, so the vertical lever is t's z; the fit converges from both starts to the same
        minimum, the horizontal lever is recovered and the vertical has no formal σ (or a huge one)."""
        t, p, rb = self.walk(tilt=0.0, seed=8)
        lever = np.array([0.2, -0.1, 0.4])
        gt = self.gt_of(t, p, rb, lever, 0.01)
        res = ea.ate_lever_numpy(gt, tum(t, p, rb), t[0], t[-1])
        self.assertLess(np.max(np.abs(np.array(res["lever_m"][:2]) - lever[:2])), 2e-3)
        self.assertLess(res["rmse"], 0.02)
        self.assertLess(abs(res["rmse_restart"] - res["rmse"]), 1e-9)
        self.assertTrue(res["lever_sigma_m"][2] is None or res["lever_sigma_m"][2] > 1.0)
        self.assertEqual(ea.lever_problems(res, None), [])

    def test_self_checks_catch_a_bad_fit(self):
        t, p, rb = self.walk(seed=9)
        gt = self.gt_of(t, p, rb, [-0.2, 0.0, 0.4], 0.01)
        res = ea.ate_lever_numpy(gt, tum(t, p, rb), t[0], t[-1])
        self.assertEqual(ea.lever_problems(res, res["rmse_lever0"]), [])
        bad = dict(res, rmse=res["rmse_lever0"] * 1.01)
        self.assertTrue(any("above its l = 0" in x for x in ea.lever_problems(bad, res["rmse_lever0"])))
        bad = dict(res, rmse_restart=res["rmse"] * 1.1)
        self.assertTrue(any("second start" in x for x in ea.lever_problems(bad, None)))
        self.assertTrue(any("numpy ATE" in x for x in ea.lever_problems(res, res["rmse_lever0"] + 0.01)))
        self.assertTrue(any("converge" in x for x in ea.lever_problems(dict(res, converged=False), None)))


# ---------------------------------------------------------------------------------------------------------------
class Fixture:
    """A project root with a GT audit for Grass01 (A54 record, or a legacy one) and a matrix `mx` with U runs and,
    optionally, T runs. Clocks: GT stamps t_gt; the estimator clock t = t_gt + shift (so the evaluation GT is
    gt_clean + shift). W frame: p_W = R_WN p_N + c_W; T's registration T_{N<-W}(t) drifts slowly after activation."""

    R_WN, C_W = rot(0.4), np.array([5.0, -3.0, 1.0])

    def __init__(self, shift=0.1, method="chain", mask=None, legacy=None):
        self.root = tempfile.mkdtemp()
        self.shift = shift
        t, p = gt_path(n=800)
        g = os.path.join(self.root, "audit", "Grass01", "gt")
        os.makedirs(g)
        ec.write_tum(os.path.join(g, "gt_clean.tum"), tum(t, p), digits=6)
        clean = ec.read_tum(os.path.join(g, "gt_clean.tum"))
        self.t, self.p = clean[:, 0], clean[:, 1:4]
        if legacy is None:
            # the A54 outputs of tools/gt_audit.py (P4-TIME format: gt_timebase.yaml, gt_audited.tum, ate_exclude column)
            with open(os.path.join(g, "gt_timebase.yaml"), "w") as f:
                f.write('# time base of Grass01\nsequence: "Grass01"\nmethod: "%s"\noffset_s: %r\napplied_shift_s: %r\n'
                        'uncertainty_s: 0.05\nconfidence: "high"\nflag: ""\n' % (method, shift, shift))
            aud = clean.copy()
            aud[:, 0] += shift
            ec.write_tum(os.path.join(g, "gt_audited.tum"), aud, digits=6)
            ex = np.zeros(len(t), dtype=bool) if mask is None else (self.t >= mask[0]) & (self.t <= mask[1])
            with open(os.path.join(g, "trust_mask.csv"), "w") as f:
                f.write("t,trusted,trusted_gt_criteria_only,reason_bits,ate_exclude(bits 8|32|64)\n")
                for a, x in zip(self.t, ex):
                    f.write("%.6f,1,1,0,%d\n" % (a, int(x)))
        else:
            aud = clean.copy()
            aud[:, 0] += shift
            if legacy == "pass":
                ec.write_tum(os.path.join(g, "gt_audited.tum"), aud, digits=6)
            with open(os.path.join(g, "gt_audit.json"), "w") as f:
                json.dump({"verdict": "WARN", "tool_version": 2, "checks": [{"id": "P4", "verdict": "WARN" if legacy == "pass" else "FAIL"}],
                           "outputs": {"gt_clean": "gt_clean.tum", "gt_audited": "gt_audited.tum" if legacy == "pass" else None,
                                       "time_shift_applied_s": shift if legacy == "pass" else None,
                                       "gt_audited_withheld": None if legacy == "pass" else "P4 FAIL"}}, f)
        self.runs, self.arms, self.u_rows, self.noise = [], {}, {}, {}

    # trajectories -------------------------------------------------------------------------------------------
    def host(self, seed, sig, spike=None, scale=1.0):
        rng = np.random.default_rng(100 + seed)
        te = np.arange(self.t[0] + self.shift + 0.1 * seed, self.t[-1] + self.shift - 2.0 + 0.1 * seed, 1 / 30.0)
        truth = np.stack([np.interp(te - self.shift, self.t, self.p[:, i]) for i in range(3)], axis=1)
        pn = truth * scale + rng.normal(0, sig, truth.shape)       # scale 1.0: bit-identical to truth + noise
        if spike is not None:
            pn[(te >= spike[0]) & (te <= spike[1])] += np.array([3.0, 0.0, 0.0])
        rb = np.stack([rot(0.2 * np.sin(0.05 * x), 0.02, -0.01) for x in te])
        return text_round(tum(te, pn @ self.R_WN.T + self.C_W, self.R_WN @ rb))

    def tnw(self, t, t_act):
        d = max(0.0, t - t_act)
        return rot(0.0002 * d) @ self.R_WN.T, -self.R_WN.T @ self.C_W + np.array([0.005 * d, 0.0, 0.0])

    def expected_e1(self, w, t_act, n_stamps):
        """E1 by the A52 rule computed here independently: activation-time T_{N<-W} before t_act, traj_N after."""
        out = w.copy()
        r0, t0 = self.tnw(t_act, t_act)
        pre = w[:, 0] < t_act
        out[pre, 1:4] = w[pre, 1:4] @ r0.T + t0
        for i in np.where(~pre)[0]:
            if w[i, 0] in n_stamps:
                r, tt = self.tnw(w[i, 0], t_act)
                out[i, 1:4] = r @ w[i, 1:4] + tt
        return out

    # runs ---------------------------------------------------------------------------------------------------
    def run_dir(self, arm, k):
        rel = "runs/eval/mx/Grass01/%s/s%d" % (arm, k)
        os.makedirs(os.path.join(self.root, rel), exist_ok=True)
        return rel, os.path.join(self.root, rel)

    def add_u(self, seeds=(0, 1, 2), failed=(), spike=None, build=B, scale=None):
        self.arms["U"] = {"traj": "traj_W.tum"}
        for k in seeds:
            rel, d = self.run_dir("U", k)
            ok = k not in failed
            if ok:
                sig = 0.02 * (k + 1)
                self.noise[k] = sig
                w = self.host(k, sig, spike, (scale or {}).get(k, 1.0))
                self.u_rows[k] = w
                ec.write_tum(os.path.join(d, "traj_W.tum"), np.vstack([w[:1], w[:1], w]))   # IMU init repeats a stamp
            self.runs.append({"seq": "Grass01", "arm": "U", "seed": k, "state": "done" if ok else "failed",
                              "attempts": [{"attempt": 1, "run_dir": rel, "complete": int(ok), "build_sha256": build,
                                            "config_sha256": "c" * 64}]})

    def add_t(self, seeds=(0,), act_after=30.0, never=(), psi_err_deg=0.0, perturb_pre=(), build=B, odom=()):
        self.arms["T"] = {"traj": "traj_N.tum"}
        self.t_act = {}
        for k in seeds:
            rel, d = self.run_dir("T", k)
            w = self.u_rows[k].copy()
            t_act = float(w[int(act_after * 30), 0])
            post = w[:, 0] >= t_act
            if k not in never:
                w[post, 1] += 0.002                               # GNSS moves the host a little after activation
            if k in perturb_pre:
                w[5, 2] += 1e-6
            ec.write_tum(os.path.join(d, "traj_W.tum"), w)
            w = ec.read_tum(os.path.join(d, "traj_W.tum"))
            events = ['t_k=%.9f t_est=0.000000000 gpst=2350:1.000000000 PHASE_CHANGE magnitude=0 detail="COLLECT"' % w[0, 0]]
            rows = []
            if k not in never:
                self.t_act[k] = t_act
                n = []
                for x in w[post]:
                    r, tt = self.tnw(x[0], t_act)
                    rw = e1traj.quat_to_rot(x[4:8])[0]
                    n.append(np.concatenate([[x[0]], r @ x[1:4] + tt, e1traj.rot_to_quat(r @ rw)[0]]))
                ec.write_tum(os.path.join(d, "traj_N.tum"), np.array(n))
                if k in odom:   # A61 F13 odometry output: here the frame of the activation step, held
                    r0, t0 = self.tnw(t_act, t_act)
                    o = [np.concatenate([[x[0]], r0 @ x[1:4] + t0, e1traj.rot_to_quat(r0 @ e1traj.quat_to_rot(x[4:8])[0])[0]])
                         for x in w[post]]
                    ec.write_tum(os.path.join(d, "traj_N_odom.tum"), np.array(o))
                events.append('t_k=%.9f t_est=0.000000000 gpst=2350:2.000000000 ACTIVATE magnitude=0.03 detail="x"' % t_act)
                psi = math.atan2(self.tnw(t_act, t_act)[0][1, 0], self.tnw(t_act, t_act)[0][0, 0]) + math.radians(psi_err_deg)
                rows = [(x, 0.0, 0.0) for x in w[:: 3, 0] if x < t_act] + [(t_act, psi, 0.03)] + \
                       [(x, psi, 0.03) for x in w[:: 3, 0] if x > t_act]
            else:
                rows = [(x, 0.0, 0.0) for x in w[:: 3, 0]]
            with open(os.path.join(d, "events.log"), "w") as f:
                f.write("\n".join(events) + "\n")
            with open(os.path.join(d, "gnss_epochs.csv"), "w") as f:
                f.write("week,tow,t_k,phase,yaw,yaw_sigma\n")
                for tk, y, ys in rows:
                    f.write("2350,0,%.9f,%s,%.17g,%.17g\n" % (tk, "TRACKING" if ys > 0 else "COLLECT", y, ys))
            self.runs.append({"seq": "Grass01", "arm": "T", "seed": k, "state": "done",
                              "attempts": [{"attempt": 1, "run_dir": rel, "complete": 1, "build_sha256": build,
                                            "config_sha256": "e" * 64}]})

    def write(self, name="mx", runs=None):
        man = {"name": name, "source_id": "abc", "image_digest": "sha256:x", "seeds": [0, 1, 2, 3], "counts": {},
               "updated_utc": "now", "arms": self.arms, "runs": self.runs if runs is None else runs}
        os.makedirs(os.path.join(self.root, "runs", "eval", name), exist_ok=True)
        with open(os.path.join(self.root, "runs", "eval", name, "matrix_manifest.json"), "w") as f:
            json.dump(man, f)

    def run(self, *extra):
        self.write()
        return ea.main(["--root", self.root, "--matrix", "runs/eval/mx", "--no-evo"] + list(extra))

    def result(self):
        return ec.load_json(os.path.join(self.root, "runs", "eval", "mx", "eval", "ate.json"))

    def run_row(self, arm, seed):
        return [x for x in self.result()["runs"] if x["arm"] == arm and x["seed"] == seed][0]

    def cell(self, arm):
        return [c for c in self.result()["cells"] if c["arm"] == arm][0]

    def evaldir(self, *p):
        return os.path.join(self.root, "runs", "eval", "mx", "eval", *p)

    def close(self):
        shutil.rmtree(self.root)


class Evaluate(unittest.TestCase):
    def test_u_cells_span_and_outputs(self):
        fx = Fixture()
        try:
            fx.add_u(seeds=(0, 1, 2, 3), failed=(3,))
            self.assertEqual(fx.run(), 0)
            r = fx.result()
            seq = r["sequences"]["Grass01"]
            self.assertEqual((seq["gt"]["method"], seq["gt"]["time_shift_applied_s"], seq["mask"]["status"]), ("p4", 0.1, "applied"))
            t0, t1 = seq["e1_spans"][0]["span"]
            # the whole sequence: latest start among runs (seed 2 starts 0.2 s later) .. earliest end, inside the GT span
            self.assertAlmostEqual(t0, fx.t[0] + 0.1 + 0.2, places=6)
            self.assertLess(t1, fx.t[-1] + 0.1 - 2.0 + 1e-6)
            self.assertEqual(seq["post30_spans"], [])
            cell = fx.cell("U")
            self.assertEqual((cell["n_ok"], cell["n_runs"], cell["seeds_missing"]), (3, 4, [3]))
            vals = sorted(cell["values_m"])
            self.assertEqual(cell["median_m"], vals[1])
            self.assertEqual((cell["min_m"], cell["max_m"]), (vals[0], vals[-1]))
            self.assertEqual(cell["e4"]["values_m"], cell["values_m"])           # U: E4 is its E1
            self.assertEqual(cell["post30"]["n_ok"], 0)
            for k in (0, 1, 2):
                x = fx.run_row("U", k)
                self.assertLess(abs(x["ate_rmse_m"] - np.sqrt(3) * fx.noise[k]), 0.35 * np.sqrt(3) * fx.noise[k])
                self.assertEqual(x["metrics"]["post30"]["status"], "no activated GNSS run in the span group")
            self.assertEqual(fx.run_row("U", 0)["est_rows_dup"], 2)
            self.assertTrue(os.path.isfile(fx.evaldir("Grass01", "U", "s0", "est_e1.tum")))
            self.assertTrue(os.path.isfile(fx.evaldir("Grass01", "gt_eval.tum")))
            with open(fx.evaldir("ate_cells.csv")) as f:
                self.assertEqual(f.read().splitlines()[1].split(",")[:4], ["Grass01", "U", "3", "4"])
        finally:
            fx.close()

    def test_e1_completes_gnss_runs_before_activation(self):
        """A52: E1 = traj_N from ACTIVATE on, before it traj_W mapped with the activation-time T_{N<-W}; a run that
        never activated is its host (= U's, bit for bit); post30 starts 30 s after the latest activation; E4 = traj_W."""
        fx = Fixture()
        try:
            fx.add_u(seeds=(0, 1))
            fx.add_t(seeds=(0, 1), never=(1,))
            self.assertEqual(fx.run(), 0)
            r = fx.result()
            t0, t1 = r["sequences"]["Grass01"]["e1_spans"][0]["span"]
            t_act = fx.t_act[0]
            t0_seed1 = fx.u_rows[1][0, 0]
            self.assertAlmostEqual(t0, t0_seed1, places=6)                    # whole sequence (seed 1 starts last)
            s0 = fx.run_row("T", 0)
            self.assertEqual((s0["activated"], s0["activation"]["t_act"]), (True, t_act))
            self.assertEqual((s0["checks"]["heading"]["status"], s0["checks"]["activate_event"]["status"]), ("ok", "ok"))
            self.assertEqual(s0["checks"]["pre_activation_vs_U"]["status"], "identical")
            # the scored file is the independent A52 construction
            w = ec.read_tum(os.path.join(fx.root, "runs/eval/mx/Grass01/T/s0/traj_W.tum"))
            n = ec.read_tum(os.path.join(fx.root, "runs/eval/mx/Grass01/T/s0/traj_N.tum"))
            want = fx.expected_e1(w, t_act, set(n[:, 0]))
            got = ec.read_tum(fx.evaldir("Grass01", "T", "s0", "est_e1.tum"))
            self.assertTrue(np.array_equal(got[:, 0], want[:, 0]))
            self.assertLess(np.max(np.abs(got[:, 1:4] - want[:, 1:4])), 2e-6)
            gt_eval = ec.read_tum(fx.evaldir("Grass01", "gt_eval.tum"))
            self.assertAlmostEqual(s0["ate_rmse_m"], ea.ate_numpy(gt_eval, want, t0, t1)["rmse"], places=5)
            # the E1 value is not the activation-onwards value (the transient before activation is in it)
            self.assertNotAlmostEqual(s0["ate_rmse_m"], ea.ate_numpy(gt_eval, want, t_act, t1)["rmse"], places=3)
            # post30: every arm on [t_act + 30, t1]
            p0, p1 = r["sequences"]["Grass01"]["post30_spans"][0]["span"]
            self.assertAlmostEqual(p0, t_act + 30.0, places=6)
            self.assertEqual(p1, t1)
            self.assertEqual(s0["metrics"]["post30"]["span"], [p0, p1])
            self.assertEqual(fx.run_row("U", 0)["metrics"]["post30"]["span"], [p0, p1])
            self.assertAlmostEqual(s0["metrics"]["post30"]["value"], ea.ate_numpy(gt_eval, want, p0, p1)["rmse"], places=5)
            # E4: T's own host trajectory on the E1 span
            e4 = ec.read_tum(fx.evaldir("Grass01", "T", "s0", "est_e4.tum"))
            self.assertTrue(np.array_equal(e4, w))
            self.assertAlmostEqual(s0["metrics"]["e4"]["value"], ea.ate_numpy(gt_eval, w, t0, t1)["rmse"], places=9)
            # never activated: the host, identical to U of the same seed
            s1, u1 = fx.run_row("T", 1), fx.run_row("U", 1)
            self.assertFalse(s1["activated"])
            self.assertEqual(s1["checks"]["pre_activation_vs_U"], {"status": "identical", "rows": int(fx.u_rows[1].shape[0])})
            self.assertEqual(s1["ate_rmse_m"], u1["ate_rmse_m"])
            self.assertEqual(s1["metrics"]["post30"]["status"], "never activated")
            c = fx.cell("T")
            self.assertEqual((c["n_ok"], c["n_activated"], c["post30"]["n_ok"], c["post30"]["n_never_activated"]), (2, 1, 1, 1))
            self.assertEqual((c["pre_activation_vs_U"], c["a55_flag"]), ({"identical": 2, "differs": 0, "no_U": 0}, False))
            self.assertAlmostEqual(c["activation_after_span_start_s"][0], t_act - t0, places=6)
            self.assertIs(r["sequences"]["Grass01"]["a55_same_build"], True)
            with open(fx.evaldir("ate_runs.csv")) as f:
                hdr = f.readline().strip().split(",")
            self.assertIn("ate_post30_m", hdr)
        finally:
            fx.close()

    def test_e1o_scores_the_odometry_output(self):
        """A61 F13: e1o = the A52 construction from traj_N_odom.tum on the e1 span; U: its e1; a GNSS run without the
        file (older than A61) has no e1o; a run that never activated is its host in both."""
        fx = Fixture()
        try:
            fx.add_u(seeds=(0, 1, 2))
            fx.add_t(seeds=(0, 1, 2), never=(2,), odom=(0,))
            self.assertEqual(fx.run(), 0)
            r = fx.result()
            t0, t1 = r["sequences"]["Grass01"]["e1_spans"][0]["span"]
            t_act = fx.t_act[0]
            w = ec.read_tum(os.path.join(fx.root, "runs/eval/mx/Grass01/T/s0/traj_W.tum"))
            r0, tt0 = fx.tnw(t_act, t_act)
            want = w.copy()
            want[:, 1:4] = w[:, 1:4] @ r0.T + tt0                   # the activation-time frame held over the whole run
            got = ec.read_tum(fx.evaldir("Grass01", "T", "s0", "est_e1o.tum"))
            self.assertTrue(np.array_equal(got[:, 0], want[:, 0]))
            self.assertLess(np.max(np.abs(got[:, 1:4] - want[:, 1:4])), 2e-6)
            gt_eval = ec.read_tum(fx.evaldir("Grass01", "gt_eval.tum"))
            s0 = fx.run_row("T", 0)
            self.assertEqual(s0["metrics"]["e1o"]["status"], "ok (numpy only)")
            self.assertAlmostEqual(s0["metrics"]["e1o"]["value"], ea.ate_numpy(gt_eval, want, t0, t1)["rmse"], places=5)
            self.assertNotAlmostEqual(s0["metrics"]["e1o"]["value"], s0["ate_rmse_m"], places=4)   # not the traj_N value
            self.assertIsNone(fx.run_row("T", 1)["metrics"]["e1o"]["value"])
            self.assertIn("older than AMENDMENTS A61", fx.run_row("T", 1)["metrics"]["e1o"]["status"])
            s2 = fx.run_row("T", 2)
            self.assertEqual(s2["metrics"]["e1o"]["value"], s2["ate_rmse_m"])                    # never activated: host
            u0 = fx.run_row("U", 0)
            self.assertEqual(u0["metrics"]["e1o"]["same_as"], "e1")
            c = fx.cell("T")
            self.assertEqual((c["e1o"]["n_ok"], c["e1o"]["seeds_ok"]), (2, [0, 2]))
            with open(fx.evaldir("ate_runs.csv")) as f:
                rows = list(csv.reader(f))
            self.assertEqual(rows[0][17], "ate_e1o_m")                # appended after the A52 columns (E1' follows it)
            with open(fx.evaldir("ate_cells.csv")) as f:
                self.assertEqual(f.readline().strip().split(",")[20:24], ["e1o_n_ok", "e1o_median_m", "e1o_min_m", "e1o_max_m"])
        finally:
            fx.close()

    def test_e1p_scores_both_outputs_with_the_lever_fit(self):
        """E1' (A64): e1p from est_e1.tum, e1op from est_e1o.tum, same spans, numpy only; ≤ the plain value; U's e1op is
        its e1p; the lever columns reach both csv files; a failing fit is an INSTRUMENT failure (exit 4)."""
        fx = Fixture()
        try:
            fx.add_u(seeds=(0, 1))
            fx.add_t(seeds=(0, 1), odom=(0, 1))
            self.assertEqual(fx.run("--lever-ref", "0.1", "0.0", "0.3"), 0)
            r = fx.result()
            t0, t1 = r["sequences"]["Grass01"]["e1_spans"][0]["span"]
            gt_eval = ec.read_tum(fx.evaldir("Grass01", "gt_eval.tum"))
            for arm, k in (("T", 0), ("U", 1)):
                x = fx.run_row(arm, k)
                m = x["metrics"]
                self.assertEqual(m["e1p"]["status"], "ok (numpy only)")
                self.assertLessEqual(m["e1p"]["value"], m["e1"]["value"] + 1e-12)
                want = ea.ate_lever_numpy(gt_eval, ec.read_tum(fx.evaldir("Grass01", arm, "s%d" % k, "est_e1.tum")), t0, t1)
                self.assertAlmostEqual(m["e1p"]["value"], want["rmse"], places=12)
                self.assertEqual(m["e1p"]["lever"]["lever_ref_m"], [0.1, 0.0, 0.3])
                self.assertEqual(m["e1p"]["span"], [t0, t1])
            t0m = fx.run_row("T", 0)["metrics"]
            want = ea.ate_lever_numpy(gt_eval, ec.read_tum(fx.evaldir("Grass01", "T", "s0", "est_e1o.tum")), t0, t1)
            self.assertAlmostEqual(t0m["e1op"]["value"], want["rmse"], places=12)
            self.assertEqual(fx.run_row("U", 0)["metrics"]["e1op"]["same_as"], "e1p")
            c = fx.cell("T")
            self.assertEqual((c["e1p"]["n_ok"], c["e1op"]["n_ok"], sorted(c["e1p"]["lever_m"])), (2, 2, ["0", "1"]))
            with open(fx.evaldir("ate_runs.csv")) as f:
                rows = list(csv.reader(f))
            self.assertEqual(rows[0][-9:], ["ate_e1p_m", "ate_e1op_m", "e1p_lever_x_m", "e1p_lever_y_m", "e1p_lever_z_m",
                                            "e1op_lever_x_m", "e1op_lever_y_m", "e1op_lever_z_m", "ate_e1c_m"])
            with open(fx.evaldir("ate_cells.csv")) as f:
                self.assertEqual(f.readline().strip().split(",")[-12:], ["e1p_n_ok", "e1p_median_m", "e1p_min_m", "e1p_max_m",
                                                                        "e1op_n_ok", "e1op_median_m", "e1op_min_m", "e1op_max_m",
                                                                        "e1c_n_ok", "e1c_median_m", "e1c_min_m", "e1c_max_m"])
            real = ea.lever_fit

            def worse(p, rb, y, l0, max_iter=ea.LEVER_MAX_ITER):
                out = real(p, rb, y, l0, max_iter)
                out["converged"] = False
                return out
            with mock.patch.object(ea, "lever_fit", side_effect=worse):
                self.assertEqual(fx.run(), 4)
            x = fx.run_row("T", 0)
            self.assertIsNone(x["metrics"]["e1p"]["value"])
            self.assertTrue(x["metrics"]["e1p"]["status"].startswith("INSTRUMENT: E1'"))
        finally:
            fx.close()

    def test_e1c_is_the_calibrated_lever_ate_and_the_lidar_picks_its_lever(self):
        """A76: e1c = rmse_at_lever_ref of the e1op record (e1o trajectory; U: its e1p), surfaced as a metric and as csv columns;
        the lever defaults to the host LiDAR's gt_lever_imu of tools/eval/lidars.json (manifest `lidar`, absent = Avia),
        --lever-ref overrides it, and a --lidar that contradicts the manifest is refused."""
        fx = Fixture()
        try:
            fx.add_u(seeds=(0, 1))
            fx.add_t(seeds=(0, 1), odom=(0, 1))
            self.assertEqual(fx.run(), 0)                      # no --lever-ref: the Avia profile (old manifest has no `lidar`)
            r = fx.result()
            avia = ec.lidar_profile("avia")["gt_lever_imu"]
            self.assertEqual(r["metric"]["lidar"], "avia")
            self.assertEqual(r["metric"]["lever_ref_m"], avia)
            self.assertIn("lidars.json", r["metric"]["lever_ref_source"])
            t0m = fx.run_row("T", 0)["metrics"]
            self.assertEqual(t0m["e1op"]["lever"]["lever_ref_m"], avia)
            self.assertEqual(t0m["e1c"]["value"], t0m["e1op"]["lever"]["rmse_at_lever_ref"])
            self.assertEqual(t0m["e1c"]["status"], "ok (numpy only)")
            u1 = fx.run_row("U", 1)["metrics"]
            self.assertEqual(u1["e1c"]["value"], u1["e1p"]["lever"]["rmse_at_lever_ref"])
            self.assertIn("same_as", u1["e1c"])
            c = fx.cell("T")
            self.assertEqual(c["e1c"]["n_ok"], 2)
            self.assertEqual(c["e1c"]["median_m"], float(np.median([fx.run_row("T", k)["metrics"]["e1c"]["value"] for k in (0, 1)])))
            # a manifest of the MID-360 matrix selects the MID-360 lever
            man = os.path.join(fx.root, "runs", "eval", "mx", "matrix_manifest.json")
            fx.write()
            with open(man) as f:
                d = json.load(f)
            d["lidar"] = "mid360"
            with open(man, "w") as f:
                json.dump(d, f)
            self.assertEqual(ea.main(["--root", fx.root, "--matrix", "runs/eval/mx", "--no-evo"]), 0)
            r = fx.result()
            mid = ec.lidar_profile("mid360")["gt_lever_imu"]
            self.assertEqual((r["metric"]["lidar"], r["metric"]["lever_ref_m"]), ("mid360", mid))
            self.assertEqual(r["matrices"][0]["lidar"], "mid360")
            self.assertNotEqual(mid, avia)
            # --lever-ref wins, --lidar must agree with the manifest
            self.assertEqual(ea.main(["--root", fx.root, "--matrix", "runs/eval/mx", "--no-evo", "--lever-ref", "0.1", "0", "0.3"]), 0)
            self.assertEqual(fx.result()["metric"]["lever_ref_source"], "--lever-ref")
            self.assertEqual(fx.result()["metric"]["lever_ref_m"], [0.1, 0.0, 0.3])
            self.assertEqual(ea.main(["--root", fx.root, "--matrix", "runs/eval/mx", "--no-evo", "--lidar", "avia"]), 3)
            self.assertEqual(ea.main(["--root", fx.root, "--matrix", "runs/eval/mx", "--no-evo", "--lidar", "mid360"]), 0)
        finally:
            fx.close()

    def test_heading_mismatch_is_an_instrument_failure(self):
        fx = Fixture()
        try:
            fx.add_u(seeds=(0,))
            fx.add_t(seeds=(0,), psi_err_deg=20.0)
            self.assertEqual(fx.run(), 4)
            s0 = fx.run_row("T", 0)
            self.assertEqual(s0["checks"]["heading"]["status"], "MISMATCH")
            self.assertTrue(s0["status"].startswith("INSTRUMENT (heading"))
            self.assertEqual(fx.cell("T")["n_ok"], 0)
        finally:
            fx.close()

    def test_a55_protocol_flags(self):
        fx = Fixture()
        try:
            fx.add_u(seeds=(0, 1))
            fx.add_t(seeds=(0, 1), perturb_pre=(0,), build=D)
            self.assertEqual(fx.run(), 0)                            # protocol flags, not instrument failures
            r = fx.result()
            self.assertIs(r["sequences"]["Grass01"]["a55_same_build"], False)
            self.assertEqual(fx.run_row("T", 0)["checks"]["pre_activation_vs_U"]["status"], "DIFFERS")
            self.assertEqual(fx.run_row("T", 1)["checks"]["pre_activation_vs_U"]["status"], "identical")
            self.assertEqual(fx.cell("T")["pre_activation_vs_U"]["differs"], 1)
            self.assertTrue(fx.cell("T")["a55_flag"] and fx.cell("U")["a55_flag"])
        finally:
            fx.close()

    def test_a54_mask_removes_the_stretch_from_both_sides(self):
        """A GT defect stretch (A54 mask) where the estimate is 3 m off: the ATE ignores it; the masked time inside the
        span is reported; neither the GT nor the estimate file has a row inside the stretch."""
        m0, m1 = 1080.0, 1090.0                                     # GT clock
        spike = (m0 + 0.1, m1 + 0.1)                                # estimator clock (shift 0.1 s)
        plain, masked = Fixture(), Fixture(mask=(m0, m1))
        try:
            for fx in (plain, masked):
                fx.add_u(seeds=(0,), spike=spike)
                self.assertEqual(fx.run(), 0)
            v_plain, v_masked = plain.run_row("U", 0)["ate_rmse_m"], masked.run_row("U", 0)["ate_rmse_m"]
            self.assertGreater(v_plain, 0.5)
            self.assertLess(v_masked, 0.1)
            seq = masked.result()["sequences"]["Grass01"]
            self.assertAlmostEqual(seq["e1_spans"][0]["masked_s"], 51 * 0.2, places=5)      # 51 rows, 0.2 s each
            self.assertEqual(seq["mask"]["rows_excluded"], 51)
            for p in (masked.evaldir("Grass01", "gt_eval.tum"), masked.evaldir("Grass01", "U", "s0", "est_e1.tum")):
                a = ec.read_tum(p)
                self.assertFalse(np.any((a[:, 0] >= spike[0]) & (a[:, 0] <= spike[1])))
        finally:
            plain.close()
            masked.close()

    def test_time_base_record_and_legacy_audits(self):
        fx = Fixture(shift=-0.0059, method="record_time")
        try:
            fx.add_u(seeds=(0,))
            self.assertEqual(fx.run(), 0)
            g = fx.result()["sequences"]["Grass01"]["gt"]
            self.assertEqual((g["method"], g["method_label"], g["time_shift_applied_s"]), ("record_time", "记录时间旁证", -0.0059))
            self.assertLess(fx.run_row("U", 0)["ate_rmse_m"], 0.1)
        finally:
            fx.close()
        fx = Fixture(legacy="pass")
        try:
            fx.add_u(seeds=(0,))
            self.assertEqual(fx.run(), 0)
            s = fx.result()["sequences"]["Grass01"]
            self.assertTrue(s["gt"]["source"].startswith("legacy") and s["mask"]["status"].startswith("absent"))
        finally:
            fx.close()
        fx = Fixture(legacy="fail")
        try:
            fx.add_u(seeds=(0,))
            self.assertEqual(fx.run(), 0)
            r = fx.result()
            self.assertIn("A54", r["blocked"]["Grass01"])
            self.assertEqual(r["cells"], [])
        finally:
            fx.close()

    def test_identical_seeds_are_flagged(self):
        fx = Fixture()
        try:
            fx.add_u(seeds=(0, 1, 2))
            shutil.copy(os.path.join(fx.root, "runs/eval/mx/Grass01/U/s0/traj_W.tum"), os.path.join(fx.root, "runs/eval/mx/Grass01/U/s1/traj_W.tum"))
            self.assertEqual(fx.run(), 0)
            cell = fx.cell("U")
            self.assertEqual(cell["identical_seed_groups"], [[0, 1]])
            self.assertEqual((cell["n_distinct"], cell["n_ok"]), (2, 3))
        finally:
            fx.close()

    def test_span_per_seed(self):
        fx = Fixture()
        try:
            fx.add_u(seeds=(0, 1, 2))
            self.assertEqual(fx.run("--span", "seed"), 0)
            self.assertEqual(len(fx.result()["sequences"]["Grass01"]["e1_spans"]), 3)
        finally:
            fx.close()

    def test_aruco_blocked(self):
        _, reg = ec.load_sequences()
        with self.assertRaises(ec.Blocked):
            ea.load_gt("/nonexistent", "GNSS-denial02", reg["GNSS-denial02"])

    def test_evaluation_sequence_is_never_read(self):
        fx = Fixture()
        real = ec.read_tum

        def guarded(path):
            if "Dark01" in path:
                raise AssertionError("A2: an evaluation-set file was read: %s" % path)
            return real(path)
        try:
            fx.add_u(seeds=(0,))
            rel = "runs/eval/mx/Dark01/U/s0"
            os.makedirs(os.path.join(fx.root, rel))
            shutil.copy(os.path.join(fx.root, "runs/eval/mx/Grass01/U/s0/traj_W.tum"), os.path.join(fx.root, rel, "traj_W.tum"))
            fx.runs.append({"seq": "Dark01", "arm": "U", "seed": 0, "state": "done",
                            "attempts": [{"attempt": 1, "run_dir": rel, "complete": 1}]})
            with mock.patch.object(ec, "read_tum", side_effect=guarded):
                self.assertEqual(fx.run(), 3)
            r = fx.result()
            self.assertIn("A2", r["blocked"]["Dark01"])
            self.assertTrue(all(x["status"].startswith("not scored") for x in r["runs"] if x["seq"] == "Dark01"))
            self.assertEqual([c["seq"] for c in r["cells"]], ["Grass01"])
        finally:
            fx.close()

    def test_same_run_in_two_matrices_is_refused(self):
        fx = Fixture()
        try:
            fx.add_u(seeds=(0,))
            fx.write()
            fx.write("my")
            rc = ea.main(["--root", fx.root, "--matrix", "runs/eval/mx", "--matrix", "runs/eval/my", "--no-evo"])
            self.assertEqual(rc, 3)
        finally:
            fx.close()

    def test_no_mask_scores_the_whole_sequence_records_it_and_keeps_the_masked_output(self):
        """A81 --no-mask: the A54 stretch stays in GT and estimate, the result equals that of a sequence without a mask, the
        record says what was not applied, and the masked output directory is untouched (the unmasked one is eval_nomask)."""
        m0, m1 = 1080.0, 1090.0
        spike = (m0 + 0.1, m1 + 0.1)
        fx, bare = Fixture(mask=(m0, m1)), Fixture()
        try:
            fx.add_u(seeds=(0,), spike=spike)
            bare.add_u(seeds=(0,), spike=spike)
            self.assertEqual(fx.run(), 0)
            masked = fx.run_row("U", 0)["ate_rmse_m"]
            self.assertEqual(bare.run(), 0)
            self.assertEqual(fx.run("--no-mask"), 0)
            nmdir = os.path.join(fx.root, "runs", "eval", "mx", "eval_nomask")
            nm = ec.load_json(os.path.join(nmdir, "ate.json"))
            whole = [x for x in nm["runs"] if x["arm"] == "U"][0]["ate_rmse_m"]
            self.assertLess(masked, 0.1)
            self.assertGreater(whole, 0.5)
            self.assertEqual(whole, bare.run_row("U", 0)["ate_rmse_m"])            # bit for bit what no mask gives
            self.assertEqual(fx.run_row("U", 0)["ate_rmse_m"], masked)              # the masked output was not overwritten
            self.assertTrue(nm["metric"]["no_mask"])
            self.assertFalse(fx.result()["metric"]["no_mask"])
            self.assertIn("--no-mask", nm["metric"]["gt"])
            seq = nm["sequences"]["Grass01"]
            self.assertTrue(seq["mask"]["status"].startswith("not applied (--no-mask"), seq["mask"]["status"])
            self.assertEqual((seq["mask"]["n_stretches"], seq["mask"]["total_s"]), (0, 0.0))
            self.assertEqual(seq["mask"]["not_applied"]["n_stretches"], 1)
            self.assertEqual(seq["mask"]["not_applied"]["rows_excluded"], 51)
            self.assertEqual(seq["e1_spans"][0]["masked_s"], 0.0)
            self.assertEqual(seq["e1_spans"][0]["masked_stretches"], 0)
            for p in (os.path.join(nmdir, "Grass01", "gt_eval.tum"), os.path.join(nmdir, "Grass01", "U", "s0", "est_e1.tum")):
                a = ec.read_tum(p)
                self.assertTrue(np.any((a[:, 0] >= spike[0]) & (a[:, 0] <= spike[1])), p)
            self.assertEqual(ec.read_tum(os.path.join(nmdir, "Grass01", "gt_eval.tum")).shape[0], len(fx.t))
            # a legacy audit (no mask at all) says so under not_applied and scores the same way
            leg = Fixture(legacy="pass")
            try:
                leg.add_u(seeds=(0,))
                self.assertEqual(leg.run("--no-mask"), 0)
                lm = ec.load_json(os.path.join(leg.root, "runs", "eval", "mx", "eval_nomask", "ate.json"))["sequences"]["Grass01"]["mask"]
                self.assertTrue(lm["status"].startswith("not applied (--no-mask"))
                self.assertEqual(lm["not_applied"]["status"], "absent (audit predates A54)")
            finally:
                leg.close()
            # an explicit --out is honoured as it was
            self.assertEqual(fx.run("--no-mask", "--out", os.path.join(fx.root, "elsewhere")), 0)
            self.assertTrue(os.path.isfile(os.path.join(fx.root, "elsewhere", "ate.json")))
        finally:
            fx.close()
            bare.close()

    def test_path_ratio_per_run_and_cell_and_the_band(self):
        """A81 divergence rule: estimated / ground-truth path at the GT times, per run and per cell, with the seeds outside
        [0.95, 1.05] listed; written to ate.json and to ate_path_runs.csv / ate_path_cells.csv."""
        fx = Fixture()
        try:
            fx.add_u(seeds=(0, 1, 2), scale={2: 1.2})
            self.assertEqual(fx.run(), 0)
            for k in (0, 1):
                x = fx.run_row("U", k)["path"]
                self.assertAlmostEqual(x["ratio"], 1.0, delta=0.01)
                self.assertEqual(x["n_segments_skipped"], 0)
                self.assertGreater(x["n_segments"], 100)
                self.assertAlmostEqual(x["est_len_m"] / x["gt_len_m"], x["ratio"], places=12)
            self.assertAlmostEqual(fx.run_row("U", 2)["path"]["ratio"], 1.2, delta=0.03)
            c = fx.cell("U")["path_ratio"]
            self.assertEqual(c["seeds_outside_band"], [2])
            self.assertEqual(sorted(c["values"]), ["0", "1", "2"])
            self.assertEqual((c["min"], c["max"]), (min(c["values"].values()), max(c["values"].values())))
            self.assertEqual(c["band"], [0.95, 1.05])
            self.assertEqual(fx.result()["metric"]["path_ratio"]["band"], [0.95, 1.05])
            with open(fx.evaldir("ate_path_runs.csv")) as f:
                rows = list(csv.DictReader(f))
            self.assertEqual([(x["arm"], x["seed"], x["outside_band"]) for x in rows], [("U", "0", "0"), ("U", "1", "0"), ("U", "2", "1")])
            self.assertAlmostEqual(float(rows[0]["path_ratio"]), fx.run_row("U", 0)["path"]["ratio"], places=5)
            self.assertEqual(int(rows[0]["segments"]), fx.run_row("U", 0)["path"]["n_segments"])
            with open(fx.evaldir("ate_path_cells.csv")) as f:
                rows = list(csv.DictReader(f))
            self.assertEqual(len(rows), 1)
            self.assertEqual((rows[0]["arm"], rows[0]["n_ok"], rows[0]["seeds_outside_band"]), ("U", "3", "2"))
            self.assertAlmostEqual(float(rows[0]["path_ratio_max"]), c["max"], places=5)
            # the two csv files the other tools read keep their columns (no path column in them)
            for name in ("ate_runs.csv", "ate_cells.csv"):
                with open(fx.evaldir(name)) as f:
                    self.assertFalse([h for h in next(csv.reader(f)) if h.startswith("path")], name)
        finally:
            fx.close()

    def test_evo_required_unless_no_evo(self):
        fx = Fixture()
        try:
            fx.add_u(seeds=(0,))
            fx.write()
            self.assertEqual(ea.main(["--root", fx.root, "--matrix", "runs/eval/mx", "--pylib", "/nonexistent"]), 3)
        finally:
            fx.close()


def evo_works():
    if not os.path.isdir(PYLIB) or not shutil.which("evo_ape"):
        return False
    env = dict(os.environ, PYTHONPATH=PYLIB, HOME=os.environ.get("HOME") or "/tmp")
    return subprocess.run(["evo_ape", "tum", "--help"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, env=env).returncode == 0


@unittest.skipUnless(evo_works(), "evo_ape with the scipy override (tools/eval/setup_evo_env.sh) not available here")
class EvoAgreement(unittest.TestCase):
    def test_evo_equals_numpy_on_every_metric(self):
        fx = Fixture(mask=(1080.0, 1090.0))
        try:
            fx.add_u(seeds=(0, 1))
            fx.add_t(seeds=(0, 1))
            fx.write()
            rc = ea.main(["--root", fx.root, "--matrix", "runs/eval/mx", "--pylib", PYLIB])
            r = fx.result()
            self.assertEqual(rc, 0, [x["status"] for x in r["runs"]])
            for x in r["runs"]:
                self.assertEqual(x["status"], "ok")
                for name in ("e1", "post30", "e4"):
                    m = x["metrics"][name]
                    self.assertEqual(m["status"], "ok", (x["arm"], x["seed"], name))
                    self.assertAlmostEqual(m["evo"]["rmse"], m["numpy"]["rmse"], delta=1e-9 * max(1.0, m["numpy"]["rmse"]))
                    self.assertIn("-a --t_max_diff 0.05 --t_start", m["evo_cmd"])
        finally:
            fx.close()


if __name__ == "__main__":
    unittest.main()
