# SPDX-License-Identifier: BSD-3-Clause
"""dev_report.py: the χ² quantiles, the exact Wilcoxon signed-rank test, Holm and κ_AR helpers; the per-run GNSS
diagnostics read from the replay outputs (gnss_epochs.csv pooled per class, the summary.json cross-check, events,
κ from the manifest's config dump, gzip twins); the paired comparisons and their TEST_PLAN §4 verdicts; the self-checks
(instrument failures in ate.json, diagnostics-chain mismatch) and exit codes; the A2 guard; and one end-to-end run on
what eval_ate.py itself writes (figures included when matplotlib is available)."""
import gzip
import hashlib
import inspect
import json
import os
import random
import shutil
import tempfile
import unittest

import numpy as np

import _path  # noqa: F401
import dev_report as dr
import evalcommon as ec
from test_eval_ate import Fixture

try:
    import matplotlib  # noqa: F401
    HAVE_MPL = True
except Exception:  # noqa: BLE001
    HAVE_MPL = False
try:
    from scipy import stats as sps
    HAVE_SCIPY = True
except Exception:  # noqa: BLE001
    HAVE_SCIPY = False

REASONS = ["mask", "l1", "clock", "slip_d1", "slip_d2", "slip_d3", "slip_d4", "slip_d5", "slip_d6", "w", "nis", "majority",
           "step", "exclusive", "no_eph", "late", "below_min"]


def epoch_header():
    """gnss_epochs.csv header, the walk of diagnostics.cpp epochWalk (EpochRecord::csvHeader)."""
    h = ["week", "tow", "t_gpst", "t_est", "t_k", "delta", "arrival_latency", "cross_cycle_bridge", "phase", "degenerate",
         "livo_alarm", "schmidt", "observe_only", "n_gps", "n_glo", "n_gal", "n_bds", "n_qzs", "n_total_raw"]
    for c in dr.CLASSES:
        h += [c + "_cand", c + "_acc"] + [c + "_rej_" + r for r in REASONS] + [c + "_nis_prior", c + "_dof", c + "_nis_post",
                                                                            c + "_skipped"]
    h += ["f_p", "f_xi", "f_clk", "f_v", "tilt_x", "tilt_y", "tilt_sigma_x", "tilt_sigma_y", "yaw", "yaw_sigma", "trans_x",
          "trans_y", "trans_z", "trans_sigma_x", "trans_sigma_y", "trans_sigma_z", "clk_b", "clk_b_sigma", "clk_d",
          "clk_d_sigma", "isb_C", "isb_E", "isb_R", "td", "td_sigma", "spp_minus_filter_x", "spp_minus_filter_y",
          "spp_minus_filter_z", "spp_valid", "lambda_1", "lambda_2", "lambda_3", "u1_x", "u1_y", "u1_z"]
    h += ["host_sigma_%d" % i for i in range(19)]
    h += ["dp_host_norm", "dyaw", "dt_norm", "iterations", "committed", "rolled_back", "n_tdcp_pairs_open", "elapsed_ms",
          "psr_decim_skipped", "pvt_decim_skipped"]
    return h


# sha256 of ",".join(epoch_header()) (diagnostics.cpp epochWalk; the T-arm header, no LIVGO columns)
CPP_HEADER_SHA256 = "69f70c846f0b28c239693ccc7e882e9d71d3b83ccb09d5d4529cc9894c95604d"


def gnss_run(d, t_act, t_end, loose=False, nis=None, tdcp_acc=(5, 6), seed=0, summary_override=None, gz=False,
             events_extra=(), config=None):
    """Writes the GNSS outputs of one replay into run directory d: 10 Hz epochs from t_act − 5 s (COLLECT) to t_end,
    pseudorange at 1 Hz, Doppler every epoch, a TDCP pair closing every 6th epoch (or PVT rows at 1 Hz when loose),
    summary.json with the same totals (unless overridden), events.log and run_manifest.json. -> the class totals."""
    nis = nis or {}
    rng = random.Random(seed)
    hdr = epoch_header()
    tot = {c: {"cand": 0, "acc": 0, "nis": 0.0, "dof": 0} for c in dr.CLASSES}
    rows = []
    n_pre = 50                                            # 5 s of COLLECT before the activation
    n_track = int(round((t_end - t_act) / 0.1)) + 1
    for k in range(n_pre + n_track):
        j = k - n_pre                                     # epoch index since the activation
        r = {h: "0" for h in hdr}
        r["t_k"] = "%.9f" % (t_act + 0.1 * j)
        r["phase"] = "TRACKING" if j >= 0 else "COLLECT"
        if j >= 0:
            r["committed"] = "1"
            r["f_p"] = repr(4e-4 + 1e-4 * rng.random())
            r["yaw"], r["yaw_sigma"] = "0.1", "0.01"

            def cls(c, cand, acc, dof, scale):
                v = float(repr(sum(rng.gauss(0.0, 1.0) ** 2 for _ in range(dof)) * scale))
                r[c + "_cand"], r[c + "_acc"] = str(cand), str(acc)
                r[c + "_rej_mask"] = str(cand - acc)
                r[c + "_dof"], r[c + "_nis_prior"] = str(dof), repr(v)
                tot[c]["cand"] += cand
                tot[c]["acc"] += acc
                tot[c]["dof"] += dof
                tot[c]["nis"] += v
            if loose:
                if j % 10 == 0:
                    cls("pvt_pos", 3, 3, 3, nis.get("pvt_pos", 0.02))
                    cls("pvt_vel", 3, 3, 3, nis.get("pvt_vel", 0.3))
            else:
                if j % 10 == 0:
                    cls("psr", 8, 7, 7, nis.get("psr", 0.02))
                cls("dopp", 9, 9, 9, nis.get("dopp", 0.95))
                if j % 6 == 5:
                    cls("tdcp", tdcp_acc[1], tdcp_acc[0], tdcp_acc[0] - 1, nis.get("tdcp", 1.0))
        rows.append(r)
    path = os.path.join(d, "gnss_epochs.csv")
    text = ",".join(hdr) + "\n" + "".join(",".join(r[h] for h in hdr) + "\n" for r in rows)
    if gz:
        with gzip.open(path + ".gz", "wt") as f:
            f.write(text)
        if os.path.exists(path):
            os.remove(path)
    else:
        with open(path, "w") as f:
            f.write(text)
    summ = {"epochs_total": len(rows), "epochs_updated": sum(1 for r in rows if r["committed"] == "1"), "epochs_rolled_back": 0,
            "mean_nis_per_dof": {c: (tot[c]["nis"] / tot[c]["dof"] if tot[c]["dof"] else None) for c in dr.CLASSES},
            "acceptance_rate": {c: (tot[c]["acc"] / tot[c]["cand"] if tot[c]["cand"] else None) for c in dr.CLASSES},
            "tdcp_coverage": 0.0 if loose else 0.8, "elapsed_p50_ms": 1.0, "elapsed_p95_ms": 2.0, "elapsed_p99_ms": 3.0,
            "n_activate": 1, "n_reinit": 0, "n_reanchor": 0, "n_clock_jump": 0, "n_slip": 1, "n_late": 0}
    if summary_override:
        summary_override(summ)
    with open(os.path.join(d, "summary.json"), "w") as f:
        json.dump(summ, f)
    ev = ['t_k=%.9f t_est=0.000000000 gpst=2350:1.000000000 PHASE_CHANGE magnitude=0 detail="COLLECT"' % (t_act - 5.0),
          't_k=%.9f t_est=0.000000000 gpst=2350:2.000000000 ACTIVATE magnitude=0.0183 detail="epochs_used=40 '
          'sigma_yaw_deg=1.050 slots=6 dim=12 path=a51 sigma_t_max_m=2.500 redundancy=30 sats=9"' % t_act,
          't_k=%.9f t_est=%.9f gpst=2350:3.000000000 SLIP magnitude=1 detail="G07 D4"' % (t_act + 3.0, t_act + 2.9)]
    ev += list(events_extra)
    with open(os.path.join(d, "events.log"), "w") as f:
        f.write("\n".join(ev) + "\n")
    cfg = config if config is not None else {
        "laserMapping": {"gnss": {"enable": True, "mode": "loose_pvt" if loose else "tight",
                                  "noise": {"psr": {"tau_s": 33.1, "decim_s": 1.0}, "dopp": {"tau_s": 0.047},
                                            "pvt": {"tau_pos_s": 21.6, "tau_vel_s": 0.52, "decim_s": 1.0}}}},
        "rosdistro": "noetic"}
    with open(os.path.join(d, "run_manifest.json"), "w") as f:
        json.dump({"arm": "x", "config_dump": json.dumps(cfg, sort_keys=True)}, f)
    return tot


# ---------------------------------------------------------------------------------------------------------------
class Stats(unittest.TestCase):
    def test_epoch_header_mirrors_the_core(self):
        h = ",".join(epoch_header())
        self.assertEqual(len(epoch_header()), 198)
        self.assertEqual(hashlib.sha256(h.encode()).hexdigest(), CPP_HEADER_SHA256)

    def test_chi2_quantiles(self):
        for p, k, want in ((0.975, 1, 5.023886187), (0.025, 1, 0.000982069), (0.95, 3, 7.814727903), (0.5, 10, 9.341817766),
                           (0.975, 40, 59.34170668), (0.025, 40, 24.43304e0)):
            self.assertAlmostEqual(dr.chi2_ppf(p, k), want, delta=2e-6 * max(1.0, want))
        for k in (1, 2, 7, 33, 150):
            for p in (0.001, 0.3, 0.999):
                self.assertAlmostEqual(dr.chi2_cdf(dr.chi2_ppf(p, k), k), p, places=9)

    def test_wilcoxon_exact_small_cases(self):
        self.assertAlmostEqual(dr.wilcoxon_exact([1, 2, 3, 4, 5])["p"], 2 / 32)
        self.assertAlmostEqual(dr.wilcoxon_exact([-1, -2, -3, -4, -5])["p"], 2 / 32)
        self.assertAlmostEqual(dr.wilcoxon_exact([-1, 2, 3, 4, 5])["p"], 4 / 32)
        self.assertAlmostEqual(dr.wilcoxon_exact([1, 2, 3, 4, 5])["p_min"], 2 / 32)
        self.assertAlmostEqual(dr.wilcoxon_exact([0.1] * 10)["p"], 2 / 1024)          # all tied, all positive
        w = dr.wilcoxon_exact([0.0, 1.0, -1.0])                                         # zero dropped, tie ±1
        self.assertEqual((w["n"], w["w_plus"], w["p"]), (2, 1.5, 1.0))
        self.assertIsNone(dr.wilcoxon_exact([0.0, 0.0])["p"])

    @unittest.skipUnless(HAVE_SCIPY, "scipy not importable here")
    def test_wilcoxon_matches_scipy_exact(self):
        rng = np.random.default_rng(4)
        for n in (5, 7, 10, 12):
            for _ in range(20):
                d = rng.normal(0.2, 1.0, n)
                kw = {"method": "exact"} if "method" in inspect.signature(sps.wilcoxon).parameters else {"mode": "exact"}
                ref = sps.wilcoxon(d, alternative="two-sided", **kw)
                self.assertAlmostEqual(dr.wilcoxon_exact(d)["p"], ref.pvalue, places=12)

    def test_holm_and_kappa(self):
        adj = dr.holm([0.01, 0.04, 0.03, None])
        self.assertEqual(adj[3], None)
        for got, want in zip(adj[:3], (0.03, 0.06, 0.06)):
            self.assertAlmostEqual(got, want)
        self.assertAlmostEqual(dr.kappa_ar(33.1, 1.0), 66.2, delta=0.05)                # A58: κ_ρ = 66.2
        self.assertAlmostEqual(dr.kappa_ar(0.047, 0.1), 1.2704, delta=1e-3)            # κ_D 1.27
        self.assertAlmostEqual(dr.kappa_ar(21.6, 1.0), 43.2, delta=0.1)                # PVT position 43.3
        self.assertAlmostEqual(dr.kappa_ar(0.52, 1.0), 1.3425, delta=1e-3)             # PVT velocity 1.34
        self.assertEqual(dr.kappa_ar(0.0, 1.0), 1.0)
        self.assertIsNone(dr.kappa_ar(None, 1.0))


# ---------------------------------------------------------------------------------------------------------------
def synth_ate(root, values, seq="Grass01", instrument=None, span=(1000.0, 1100.0), gnss_dirs=True):
    """An ate.json as eval_ate.py writes it, with the given E1 values {arm: {seed: value}} (GNSS arms: traj_N)."""
    runs, cells = [], []
    for arm, vals in values.items():
        gnss = arm != "U"
        for seed, v in vals.items():
            rel = "runs/eval/mx/%s/%s/s%d" % (seq, arm, seed)
            os.makedirs(os.path.join(root, rel), exist_ok=True)
            status = "ok (numpy only)"
            if instrument and (arm, seed) == instrument:
                status = "INSTRUMENT (heading: MISMATCH); e1 ok (numpy only)"
            r = {"seq": seq, "arm": arm, "seed": seed, "run_dir": rel, "traj": "traj_N.tum" if gnss else "traj_W.tum",
                 "status": status, "e1_span": list(span), "ate_rmse_m": v, "build_sha256": "b" * 64, "config_sha256": "c" * 64,
                 "metrics": {"e1": {"value": v, "status": "ok (numpy only)"}, "post30": {"value": v, "status": "ok (numpy only)"},
                             "e4": {"value": v, "status": "ok (numpy only)"}}}
            if gnss:
                r.update(activated=True, activation={"t_act": span[0] + 12.0},
                         checks={"pre_activation_vs_U": {"status": "identical"}, "heading": {"status": "ok"}})
                if gnss_dirs:
                    gnss_run(os.path.join(root, rel), span[0] + 12.0, span[0] + 60.0, loose=arm == "L-PVT", seed=seed)
            runs.append(r)
        xs = sorted(vals.values())
        cells.append({"seq": seq, "arm": arm, "n_ok": len(xs), "n_runs": len(xs), "median_m": float(np.median(xs)),
                      "min_m": xs[0], "max_m": xs[-1], "n_distinct": len(xs), "identical_seed_groups": [],
                      "post30": {"median_m": None, "n_ok": 0}, "e4": {"median_m": None, "n_ok": 0}})
    ate = {"tool": "tools/eval/eval_ate.py", "generated_utc": "now", "root": root, "matrices": [{"name": "mx", "source_id": "abc"}],
           "metric": {"t_max_diff_s": 0.05}, "runs": runs, "cells": cells, "blocked": {},
           "sequences": {seq: {"gt": {"method": "p4"}, "mask": {}, "e1_spans": [{"span": list(span)}], "a55_same_build": True}}}
    path = os.path.join(root, "runs", "eval", "mx", "eval", "ate.json")
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        json.dump(ate, f)
    return path


class FacadeTotals(unittest.TestCase):
    """summary.json's TDCP acceptance skips the closing epochs whose pair members were all lost (gnss_fusion.cpp
    classStats returns before the totals when no row set was built); the check reports that difference, and nothing else."""

    def test_all_lost_closing_epochs_are_a_known_difference(self):
        d = tempfile.mkdtemp()
        try:
            gnss_run(d, 1000.0, 1030.0)
            path = os.path.join(d, "gnss_epochs.csv")
            with open(path) as f:
                lines = f.read().splitlines()
            hdr = lines[0].split(",")
            col = {h: i for i, h in enumerate(hdr)}
            # two TRACKING epochs get an all-lost TDCP closing (3 + 2 candidates, all 'mask', no rows, dof 0)
            done = 0
            for i in range(1, len(lines)):
                v = lines[i].split(",")
                if v[col["phase"]] == "TRACKING" and v[col["tdcp_cand"]] == "0" and done < 2:
                    n = 3 if done == 0 else 2
                    v[col["tdcp_cand"]], v[col["tdcp_rej_mask"]] = str(n), str(n)
                    lines[i] = ",".join(v)
                    done += 1
            with open(path, "w") as f:
                f.write("\n".join(lines) + "\n")
            kap = dr.class_kappas({"noise": {}})
            ep = dr.read_epochs(path, kap)
            self.assertEqual((ep["classes"]["tdcp"]["lost_epochs"], ep["classes"]["tdcp"]["lost_cand"]), (2, 5))
            summ = ec.load_json(os.path.join(d, "summary.json"))      # written from the totals before the edit
            st = dr.summary_check(summ, ep)
            self.assertTrue(st.startswith("ok (facade totals rule"), st)
            self.assertIn("2 all-lost closing epoch(s) (5 candidates)", st)
            summ["acceptance_rate"]["tdcp"] *= 0.99                     # anything else stays a mismatch
            self.assertTrue(dr.summary_check(summ, ep).startswith("MISMATCH"))
        finally:
            shutil.rmtree(d)


class Report(unittest.TestCase):
    def setUp(self):
        self.root = tempfile.mkdtemp()

    def tearDown(self):
        shutil.rmtree(self.root)

    def run_tool(self, ate, *extra):
        out = os.path.join(self.root, "rep")
        rc = dr.main(["--ate", ate, "--out", out, "--no-plots"] + list(extra))
        return rc, ec.load_json(os.path.join(out, "dev_report.json")) if os.path.isfile(os.path.join(out, "dev_report.json")) else None

    def test_five_seeds_can_only_tie_ten_seeds_can_win(self):
        u = {k: 1.0 + 0.1 * k for k in range(5)}
        t = {k: u[k] - 0.05 for k in range(5)}                    # T better on every seed
        rc, rep = self.run_tool(synth_ate(self.root, {"U": u, "T": t}))
        self.assertEqual(rc, 0)
        c = rep["sequences"]["Grass01"]["vs_u"][0]
        self.assertEqual((c["a"], c["n"], c["wins"]), ("T", 5, 5))
        self.assertAlmostEqual(c["median_diff_m"], -0.05)
        self.assertAlmostEqual(c["p"], 0.0625)
        self.assertTrue(c["verdict"].startswith("tie (n=5"))
        shutil.rmtree(self.root)
        os.makedirs(self.root)
        u = {k: 1.0 + 0.1 * k for k in range(10)}
        t = {k: u[k] - 0.05 for k in range(10)}
        worse = {k: u[k] + (0.2 if k < 9 else -0.01) for k in range(10)}   # 9/10 losses
        rc, rep = self.run_tool(synth_ate(self.root, {"U": u, "T": t, "T-tdcp": worse}))
        v = {c["a"]: c for c in rep["sequences"]["Grass01"]["vs_u"]}
        self.assertEqual(v["T"]["verdict"], "better")
        self.assertEqual(v["T-tdcp"]["verdict"], "worse")
        self.assertLess(v["T"]["p_holm"], 0.05)
        # contribution of TDCP: T against T-tdcp, T wins on every seed
        ctr = rep["sequences"]["Grass01"]["contributions"][0]
        self.assertEqual((ctr["name"], ctr["wins"], ctr["n"]), ("T vs T-tdcp", 10, 10))
        self.assertLess(ctr["median_diff_m"], 0.0)

    def test_gnss_diagnostics_pooled_and_cross_checked(self):
        ate = synth_ate(self.root, {"U": {0: 1.0, 1: 1.1}, "T": {0: 0.9, 1: 1.0}, "L-PVT": {0: 1.0, 1: 1.2}})
        rc, rep = self.run_tool(ate)
        self.assertEqual(rc, 0)
        self.assertTrue(rep["self_checks"]["instrument_all_pass"])
        runs = {(r["arm"], r["seed"]): r for r in rep["runs"]}
        g = runs[("T", 0)]["gnss"]
        self.assertEqual(g["summary_check"], "ok")
        self.assertTrue(g["config_found"])
        self.assertAlmostEqual(g["kappa"]["psr"], dr.kappa_ar(33.1, 1.0))
        self.assertAlmostEqual(g["kappa"]["dopp"], dr.kappa_ar(0.047, 0.1))
        self.assertEqual(g["kappa"]["tdcp"], 1.0)
        ps = g["epochs"]["classes"]["psr"]
        self.assertEqual((ps["cand"], ps["acc"]), (8 * 49, 7 * 49))
        self.assertAlmostEqual(ps["acc_rate"], 7 / 8)
        self.assertAlmostEqual(ps["nis_per_dof_x_kappa"], ps["nis_per_dof"] * g["kappa"]["psr"])
        self.assertEqual(g["events"]["SLIP"], 1)
        self.assertEqual(g["first_activate"]["path"], "a51")
        self.assertAlmostEqual(g["first_activate"]["sigma_yaw_deg"], 1.05)
        self.assertAlmostEqual(g["t_act_after_span_start_s"], 12.0)
        self.assertEqual(g["epochs"]["phases"]["COLLECT"], 50)
        self.assertLess(g["epochs"]["f_p_median"], 1e-3)
        # pooled cell: the two seeds' totals
        cell = [c for c in rep["cells"] if c["arm"] == "T"][0]
        tot = cell["gnss"]["classes"]["dopp"]
        self.assertEqual(tot["cand"], 2 * 9 * 481)
        self.assertEqual(cell["gnss"]["n_activated"], 2)
        # χ² band: Doppler NIS drawn from 0.95·χ²_9 ⇒ most epochs inside the two-sided band
        self.assertGreater(tot["band_frac"], 0.85)
        # the loose arm reports PVT classes only
        lp = [c for c in rep["cells"] if c["arm"] == "L-PVT"][0]["gnss"]["classes"]
        self.assertEqual(sorted(lp), ["pvt_pos", "pvt_vel"])
        self.assertAlmostEqual(lp["pvt_pos"]["kappa"], dr.kappa_ar(21.6, 1.0))
        # outputs
        out = os.path.join(self.root, "rep")
        for f in ("dev_report.md", "dev_report_runs.csv", "dev_report_cells.csv"):
            self.assertTrue(os.path.isfile(os.path.join(out, f)))
        with open(os.path.join(out, "dev_report.md"), encoding="utf-8") as f:
            md = f.read()
        self.assertIn("仪器自检:全过", md)
        self.assertIn("T vs L-PVT", md)

    def test_summary_mismatch_and_instrument_failures_fail_the_self_check(self):
        ate = synth_ate(self.root, {"U": {0: 1.0}, "T": {0: 0.9}})
        d = os.path.join(self.root, "runs/eval/mx/Grass01/T/s0")
        gnss_run(d, 1012.0, 1060.0, summary_override=lambda s: s["mean_nis_per_dof"].__setitem__("dopp", 0.5))
        rc, rep = self.run_tool(ate)
        self.assertEqual(rc, 4)
        self.assertEqual(rep["self_checks"]["diagnostics_chain"]["status"], "FAIL")
        self.assertIn("dopp nis/dof", rep["self_checks"]["diagnostics_chain"]["items"][0][1])
        shutil.rmtree(self.root)
        os.makedirs(self.root)
        rc, rep = self.run_tool(synth_ate(self.root, {"U": {0: 1.0}, "T": {0: 0.9}}, instrument=("T", 0)))
        self.assertEqual(rc, 4)
        self.assertEqual(rep["self_checks"]["ate_instrument"]["status"], "FAIL")
        self.assertFalse(rep["self_checks"]["instrument_all_pass"])

    def test_gzip_twin_flags_and_missing_config(self):
        ate = synth_ate(self.root, {"U": {0: 1.0}, "T": {0: 0.9}})
        d = os.path.join(self.root, "runs/eval/mx/Grass01/T/s0")
        # TDCP accepted in 2 of 6 rows (33 %) with NIS 2 per dof: the lock-up rule fires; Doppler NIS 3 per dof: D4 flag
        gnss_run(d, 1012.0, 1060.0, gz=True, tdcp_acc=(2, 6), nis={"tdcp": 2.0, "dopp": 3.0}, config={"rosdistro": "noetic"},
                 events_extra=['t_k=1030.000000000 t_est=0.000000000 gpst=2350:4.000000000 REANCHOR magnitude=0 detail=""'])
        rc, rep = self.run_tool(ate)
        self.assertEqual(rc, 0)                                   # flags are reported, they do not gate the exit code
        g = rep["runs"][1]["gnss"]
        self.assertFalse(g["config_found"])
        self.assertIsNone(g["kappa"]["psr"])
        self.assertEqual(g["epochs"]["classes"]["tdcp"]["cand"], 6 * 80)
        flags = " | ".join(f["flag"] for f in rep["flags"])
        self.assertIn("lock-up rule", flags)
        self.assertIn("dopp NIS/dof", flags)
        self.assertIn("REANCHOR", flags)

    def test_a2_guard_refuses_evaluation_sequences_before_reading(self):
        ate = synth_ate(self.root, {"U": {0: 1.0}, "T": {0: 0.9}}, seq="Outdoor01", gnss_dirs=False)
        rc, rep = self.run_tool(ate)
        self.assertEqual(rc, 3)
        self.assertIn("Outdoor01", rep["blocked"])
        self.assertEqual(rep["runs"], [])
        self.assertEqual(rep["cells"], [])


# ---------------------------------------------------------------------------------------------------------------
class EndToEnd(unittest.TestCase):
    """eval_ate.py's own output (Fixture of test_eval_ate: GT audit, U and T runs, --no-evo) + the GNSS files."""

    def test_on_eval_ate_output(self):
        fx = Fixture()
        try:
            fx.add_u(seeds=(0, 1, 2))
            fx.add_t(seeds=(0, 1, 2))
            self.assertEqual(fx.run(), 0)
            ate = os.path.join(fx.root, "runs", "eval", "mx", "eval", "ate.json")
            for k in (0, 1, 2):
                d = os.path.join(fx.root, "runs/eval/mx/Grass01/T/s%d" % k)
                gnss_run(d, fx.t_act[k], fx.t_act[k] + 60.0, seed=k)
            out = os.path.join(fx.root, "rep")
            argv = ["--ate", ate, "--out", out] + ([] if HAVE_MPL else ["--no-plots"])
            self.assertEqual(dr.main(argv), 0)
            rep = ec.load_json(os.path.join(out, "dev_report.json"))
            self.assertTrue(rep["self_checks"]["instrument_all_pass"])
            self.assertEqual(rep["self_checks"]["i11_pre_activation_equals_U"]["status"], "PASS")
            t = [c for c in rep["cells"] if c["arm"] == "T"][0]
            self.assertEqual((t["n_ok"], t["n_runs"], t["gnss"]["n_activated"]), (3, 3, 3))
            self.assertEqual(t["vs_u"]["n"], 3)
            act = t["gnss"]["t_act_median_s"]
            span0 = rep["sequences"]["Grass01"]["e1_spans"][0]["span"][0]
            self.assertAlmostEqual(act, float(np.median([fx.t_act[k] - span0 for k in (0, 1, 2)])), places=6)
            if HAVE_MPL:
                for f in ("traj_Grass01", "heat_global", "heat_odom", "heat_nis"):
                    self.assertIn(f, rep["figures"]["files"])
                    p = os.path.join(out, rep["figures"]["files"][f])
                    self.assertGreater(os.path.getsize(p), 10000)
        finally:
            fx.close()


if __name__ == "__main__":
    unittest.main()
