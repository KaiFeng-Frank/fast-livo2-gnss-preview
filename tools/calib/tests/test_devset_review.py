# SPDX-License-Identifier: BSD-3-Clause
"""Unit tests of the development-set review helpers (AMENDMENTS A62): python3 -m unittest discover -s tools/calib/tests"""
import math
import os
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import devset_review as D  # noqa: E402


class Guards(unittest.TestCase):
    def test_only_development_sequences(self):
        D.dev_only("/r/runs/eval/x/Grass01/T/s0")
        for bad in ("/r/runs/eval/x/Dark01/T/s0", "/r/runs/eval/x/Outdoor04/T/s0", "Varying-illu03", "SomethingElse"):
            with self.assertRaises(SystemExit):
                D.dev_only(bad)


class Parsing(unittest.TestCase):
    def test_event_line(self):
        line = ('t_k=1737278603.196575 t_est=0.000000000 gpst=0:0.000000000 FRAME_JUMP magnitude=0.0087 '
                'detail="host correction 0.009 m / 0.35 deg (chi2 65): registration widened by 0.009 m, 0.05 deg yaw"')
        m = D.EVENT_RE.match(line)
        self.assertIsNotNone(m)
        self.assertEqual(m.group(4), "FRAME_JUMP")
        self.assertAlmostEqual(float(m.group(5)), 0.0087)
        self.assertTrue(m.group(6).startswith("host correction"))
        self.assertEqual(D.DET_RE.search("G30 detectors=0x10 d4=0.2/1.3 cyc").group(1), "10")

    def test_anchor_yaml(self):
        txt = ("valid: true\nr0_ecef: [-1701852.37, 5010299.41, 3549842.80]\nR_EN:  # row-major, columns = east/north/up\n"
               "  - [-0.9468677, 0.1800100, -0.2665294]\n  - [-0.3216233, -0.5299545, 0.7846697]\n  - [0, 0.8287005, 0.5596922]\n"
               "lat_deg: 34.0\n")
        with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as f:
            f.write(txt)
        try:
            r0, R = D.read_anchor(f.name)
        finally:
            os.unlink(f.name)
        self.assertEqual(r0.shape, (3,))
        self.assertAlmostEqual(r0[1], 5010299.41)
        self.assertEqual(R.shape, (3, 3))
        self.assertAlmostEqual(R[2, 1], 0.8287005)
        np.testing.assert_allclose(R.T @ R, np.eye(3), atol=1e-6)


class TdcpLag(unittest.TestCase):
    """The lag statistic on synthetic chains: white innovations give ρ₁ ≈ 0; independent-pair residuals of chained pairs
    that share the junction phase with opposite signs give ρ₁ ≈ −σ_Φ²/(2σ_Φ² + o²)."""

    @staticmethod
    def chains(n_sats, n_pairs, shared, rng, sigma_phi=1.0, sigma_o=0.5):
        ep_t = np.arange(n_pairs) * 0.5
        acc = {}
        for s in range(n_sats):
            phi = rng.normal(0.0, sigma_phi, n_pairs + 1)
            o = rng.normal(0.0, sigma_o, n_pairs)
            if shared:
                e = phi[1:] - phi[:-1] + o
            else:
                e = rng.normal(0.0, math.sqrt(2 * sigma_phi ** 2 + sigma_o ** 2), n_pairs)
            acc["G%02d" % s] = {k: (float(e[k]), float(e[k]), 1.0, 45.0) for k in range(n_pairs)}
        return ep_t, acc

    def test_white_and_shared_junction(self):
        rng = np.random.default_rng(7)
        t, acc = self.chains(10, 2000, False, rng)
        st = D.lag_stats(t, acc, 1, 1.5, "w")
        self.assertLess(abs(st["rho"]), 0.03)
        t, acc = self.chains(10, 2000, True, rng)
        st = D.lag_stats(t, acc, 1, 1.5, "w")
        self.assertAlmostEqual(st["rho"], -1.0 / (2.0 + 0.25), delta=0.03)
        self.assertLess(abs(D.lag_stats(t, acc, 2, 1.5, "w")["rho"]), 0.03)

    def test_chain_model_reading(self):
        r1, nis = D.chain_model(28.0, 5.0, 28.0, 5.0, n=40000)
        self.assertLess(abs(r1), 0.03)
        self.assertAlmostEqual(nis, 1.0, delta=0.04)
        r1, nis = D.chain_model(28.0, 0.5, 28.0, 5.0, n=40000)      # per-pair term over-modelled: negative lag 1, NIS < 1
        self.assertLess(r1, -0.1)
        self.assertLess(nis, 0.9)
        r1, _ = D.chain_model(28.0, 5.0, 28.0, 5.0, rho_o=0.5, n=40000)  # correlated across pairs: positive lag 1
        self.assertGreater(r1, 0.1)

    def test_gap_breaks_the_chain(self):
        rng = np.random.default_rng(8)
        t, acc = self.chains(3, 200, True, rng)
        n_all = D.lag_stats(t, acc, 1, 1.5, "w")["pairs"]
        t2 = t.copy()
        t2[100:] += 10.0               # an outage after pair 99: the (99, 100) pairs are not consecutive links
        self.assertEqual(D.lag_stats(t2, acc, 1, 1.5, "w")["pairs"], n_all - 3)


if __name__ == "__main__":
    unittest.main()
