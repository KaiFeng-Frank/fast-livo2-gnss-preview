# SPDX-License-Identifier: BSD-3-Clause
"""e1traj.py (AMENDMENTS A52): rotation helpers, T_{N<-W} from the two step-end outputs, the E1 assembly (fill before
activation with the activation-time T_{N<-W}, causal fill of post-activation gaps, never-activated runs), the heading
self-check against gnss_epochs.csv and the ACTIVATE/traj_N consistency check."""
import math
import os
import tempfile
import unittest

import numpy as np

import _path  # noqa: F401
import e1traj as e


def rot(yaw, pitch=0.0, roll=0.0):
    cz, sz, cy, sy, cx, sx = np.cos(yaw), np.sin(yaw), np.cos(pitch), np.sin(pitch), np.cos(roll), np.sin(roll)
    rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    return rz @ ry @ rx


def poses(t, p, r):
    a = np.zeros((len(t), 8))
    a[:, 0], a[:, 1:4], a[:, 4:8] = t, p, e.rot_to_quat(r)
    return a


def text_round(a):
    """What a %.9f TUM file gives back."""
    return np.array([[float("%.9f" % x) for x in row] for row in a])


def host(n=400, t0=100.0, dt=1 / 15.0, seed=2):
    rng = np.random.default_rng(seed)
    t = t0 + dt * np.arange(n)
    p = np.cumsum(rng.normal(0, 0.02, (n, 3)) + [0.05, 0.02, 0.0], axis=0)
    r = np.stack([rot(0.3 * np.sin(0.05 * k), 0.04, -0.03 + 0.001 * k) for k in range(n)])
    return poses(t, p, r)


def tnw_at(t, t_act):
    """A slowly drifting registration: yaw, a 3 deg tilt (a W frame that is not level), translation."""
    d = max(0.0, t - t_act)
    return rot(-2.1 + 0.002 * d, 0.05, 0.02), np.array([10.0 + 0.01 * d, -4.0, 0.5])


def global_rows(w, t_act, keep=None):
    rows = []
    for x in w:
        if x[0] < t_act or (keep is not None and not keep(x[0])):
            continue
        r, t = tnw_at(x[0], t_act)
        rw = e.quat_to_rot(x[4:8])[0]
        rows.append(np.concatenate([[x[0]], r @ x[1:4] + t, e.rot_to_quat(r @ rw)[0]]))
    return np.array(rows).reshape(-1, 8)


class Rotations(unittest.TestCase):
    def test_quaternion_round_trip_all_branches(self):
        rng = np.random.default_rng(0)
        q = rng.normal(size=(500, 4))
        q /= np.linalg.norm(q, axis=1, keepdims=True)
        q[q[:, 3] < 0] *= -1
        # rotations near 180 deg about each axis exercise the three non-trace branches
        q = np.vstack([q, np.array([[1, 0, 0, 1e-3], [0, 1, 0, 1e-3], [0, 0, 1, 1e-3]]) / np.linalg.norm([1, 0, 0, 1e-3])])
        r = e.quat_to_rot(q)
        self.assertLess(np.max(np.abs(r @ np.transpose(r, (0, 2, 1)) - np.eye(3))), 1e-12)
        self.assertLess(np.max(np.abs(e.rot_to_quat(r) - q)), 1e-12)
        self.assertLess(np.max(np.abs(e.quat_to_rot(e.rot_to_quat(r)) - r)), 1e-12)

    def test_azimuth_is_yaw_plus_second_order(self):
        """M = R_z(psi) T, T a tilt about any horizontal axis (what C(g) and Exp([phi, 0]) are): the azimuth of M's first
        column is psi within tilt^2/4 — the basis of the heading self-check's tolerance."""
        worst = 0.0
        for yaw in (-3.0, -2.1, 0.0, 1.3):
            for tilt in (0.0, 0.05, 0.1, 0.2):
                for beta in np.linspace(0, np.pi, 13):
                    a = np.array([np.cos(beta), np.sin(beta), 0.0])
                    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
                    t = np.eye(3) + np.sin(tilt) * k + (1 - np.cos(tilt)) * k @ k
                    d = abs(e.wrap_deg(e.azimuth_deg(rot(yaw) @ t) - math.degrees(yaw)))
                    self.assertLessEqual(d, math.degrees(tilt ** 2 / 4) * 1.01 + 1e-9)
                    worst = max(worst, d / max(math.degrees(tilt ** 2 / 4), 1e-12))
        self.assertGreater(worst, 0.9)                       # the bound is attained (at beta = 45 deg)


class Assemble(unittest.TestCase):
    def setUp(self):
        self.w = text_round(host())
        self.t_act = float(self.w[60, 0])

    def test_fill_before_activation_uses_the_activation_estimate(self):
        n = text_round(global_rows(self.w, self.t_act))
        e1, info = e.assemble_e1(self.w, n)
        self.assertTrue(info["activated"])
        self.assertEqual((info["t_act"], info["rows_fill_pre"], info["rows_fill_gap"], info["rows_traj_n"]),
                         (self.t_act, 60, 0, self.w.shape[0] - 60))
        self.assertTrue(np.array_equal(e1[:, 0], self.w[:, 0]))
        r0, t0 = tnw_at(self.t_act, self.t_act)
        expect_pre = self.w[:60, 1:4] @ r0.T + t0
        self.assertLess(np.max(np.abs(e1[:60, 1:4] - expect_pre)), 1e-6)          # 1e-9 text rounding x 12 m lever
        self.assertTrue(np.array_equal(e1[60:], n))                               # from activation on: traj_N itself
        # the derived registration is the one used to make traj_N (text rounding only)
        stamps, r_nw, t_nw = e.derive_tnw(self.w, n)
        for k in (0, 100, len(stamps) - 1):
            r, t = tnw_at(stamps[k], self.t_act)
            self.assertLess(np.max(np.abs(r_nw[k] - r)), 1e-7)
            self.assertLess(np.max(np.abs(t_nw[k] - t)), 1e-6)
        self.assertAlmostEqual(info["tnw_act"]["azimuth_deg"], e.azimuth_deg(r0), places=5)
        self.assertLess(info["tnw_orthonormality_err"], 1e-7)
        # orientations of the filled rows are T_{N<-W} R_{W<-I}
        rw = e.quat_to_rot(self.w[:60, 4:8])
        self.assertLess(np.max(np.abs(e.quat_to_rot(e1[:60, 4:8]) - r0 @ rw)), 1e-7)

    def test_gap_after_activation_is_filled_causally(self):
        tg0, tg1 = float(self.w[150, 0]), float(self.w[170, 0])
        n = text_round(global_rows(self.w, self.t_act, keep=lambda t: not (tg0 <= t <= tg1)))
        e1, info = e.assemble_e1(self.w, n)
        self.assertEqual(info["rows_fill_gap"], 21)
        self.assertEqual(info["gaps"], [[tg0, tg1, 21]])
        self.assertAlmostEqual(info["gap_s"], tg1 - tg0)
        r_prev, t_prev = tnw_at(float(self.w[149, 0]), self.t_act)       # the last registration before the gap
        expect = self.w[150:171, 1:4] @ r_prev.T + t_prev
        self.assertLess(np.max(np.abs(e1[150:171, 1:4] - expect)), 1e-6)
        r_now, t_now = tnw_at(float(self.w[160, 0]), self.t_act)          # not the (future) one of the gap itself
        self.assertGreater(np.max(np.abs(e1[160, 1:4] - (r_now @ self.w[160, 1:4] + t_now))), 1e-4)

    def test_never_activated_is_the_host(self):
        e1, info = e.assemble_e1(self.w, np.zeros((0, 8)))
        self.assertFalse(info["activated"])
        self.assertTrue(np.array_equal(e1, self.w))

    def test_no_common_stamp_is_an_error(self):
        n = global_rows(self.w, self.t_act)
        n[:, 0] += 0.001
        with self.assertRaises(ValueError):
            e.assemble_e1(self.w, n)


class Checks(unittest.TestCase):
    def write_epochs(self, d, rows):
        p = os.path.join(d, "gnss_epochs.csv")
        with open(p, "w") as f:
            f.write("week,tow,t_k,phase,yaw,yaw_sigma,committed\n")
            for tk, ph, y, ys in rows:
                f.write("2350,0,%.9f,%s,%.17g,%.17g,0\n" % (tk, ph, y, ys))
        return p

    def test_heading_check(self):
        r = rot(-2.1307, 0.06, 0.0)          # 3.4 deg tilt, as on the development set
        with tempfile.TemporaryDirectory() as d:
            rows = [(99.9, "COLLECT", 0.0, 0.0), (100.0, "COLLECT", -2.1307, 0.033), (100.1, "TRACKING", 1.0, 0.03)]
            p = self.write_epochs(d, rows)
            res = e.heading_check(p, 100.0, r)
            self.assertEqual(res["status"], "ok")
            self.assertEqual(res["epoch_t_k"], 100.0)                    # the activation row (phase still COLLECT)
            self.assertLess(abs(res["diff_deg"]), 0.1)
            bad = e.heading_check(self.write_epochs(d, [(100.0, "COLLECT", -2.1307 + math.radians(3.0), 0.033)]), 100.0, r)
            self.assertEqual(bad["status"], "MISMATCH")
            # an inverted registration (T_{W<-N} instead of T_{N<-W}) is caught
            self.assertEqual(e.heading_check(p, 100.0, r.T)["status"], "MISMATCH")
            self.assertTrue(e.heading_check(self.write_epochs(d, [(99.9, "COLLECT", 0.0, 0.0)]), 100.0, r)["status"].startswith("skipped"))
            self.assertTrue(e.heading_check(os.path.join(d, "none.csv"), 100.0, r)["status"].startswith("skipped"))

    def test_activation_event_check(self):
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, "events.log")
            with open(p, "w") as f:
                f.write('t_k=90.000000000 t_est=0.000000000 gpst=2350:1.000000000 PHASE_CHANGE magnitude=0 detail="x"\n')
                f.write('t_k=100.000000000 t_est=0.000000000 gpst=2350:2.000000000 ACTIVATE magnitude=0.03 detail="epochs_used=101"\n')
                f.write('t_k=150.000000000 t_est=0.000000000 gpst=2350:3.000000000 REINIT magnitude=0 detail=""\n')
            ev = e.read_events(p)
            self.assertEqual([k for k, _ in ev], ["PHASE_CHANGE", "ACTIVATE", "REINIT"])
            ok = e.activation_event_check(ev, 100.0)
            self.assertEqual((ok["status"], ok["n_activate"], ok["n_reinit"]), ("ok", 1, 1))
            self.assertTrue(e.activation_event_check(ev, 105.0)["status"].startswith("MISMATCH"))
            self.assertTrue(e.activation_event_check(ev, None)["status"].startswith("MISMATCH"))
            self.assertTrue(e.activation_event_check([("PHASE_CHANGE", 1.0)], 100.0)["status"].startswith("MISMATCH"))
            self.assertEqual(e.activation_event_check([("PHASE_CHANGE", 1.0)], None)["status"], "ok")
            self.assertTrue(e.activation_event_check(None, 100.0)["status"].startswith("skipped"))


if __name__ == "__main__":
    unittest.main()
