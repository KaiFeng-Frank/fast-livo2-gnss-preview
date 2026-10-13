#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Development-set report of a replay matrix (docs/NEXT_STEPS.md step 1: the material of AMENDMENTS A59).

    dev_report.py --ate runs/eval/<name>/eval/ate.json [--out DIR] [--u-arm U] [--t-arm T] [--no-plots]
                  [--alpha 0.05] [--win-frac 0.8] [--evaluation-phase] [--root ROOT]

Input: what eval_ate.py wrote (ate.json: runs, cells, spans; <eval>/<seq>/gt_eval.tum and
<eval>/<seq>/<arm>/s<k>/est_e1.tum, i.e. exactly what the ATE read) and, per complete run directory, the GNSS outputs
of the replay (DESIGN §15): gnss_epochs.csv, summary.json, events.log and run_manifest.json (its config_dump gives the
κ_AR of every measurement class). Nothing is replayed and no ATE is recomputed: E1/post30/E4 values come from ate.json.

Output (--out, default <eval>/dev_report):
  dev_report.md          the A59 draft: self-checks, E1 with paired comparisons against U, the GNSS diagnostics per
                         cell (activation, acceptance, NIS, TDCP coverage, gain share, events), the contribution of
                         each measurement class (T against T-pr / T-dop / T-tdcp, and against L-PVT), consistency flags
  dev_report.json        everything above with provenance; dev_report_runs.csv / dev_report_cells.csv
  fig/traj_<seq>.png     GT and the E1 trajectories of U, T and L-PVT for one seed (the one whose T value is the median),
                         each GNSS arm as its global output (traj_N, solid) and its odometry output (traj_N_odom, dashed;
                         AMENDMENTS A61 F13), top view after each trajectory's own SE(3) Umeyama alignment on the E1 span
                         (the ATE's), and the per-pose position error over time with T's activation marked and the GT
                         mask stretches (A54, removed from the ATE) shaded
  fig/heat_global.png    sequences x arms, global output: median over seeds of the paired ratio arm/U of E1 (left) and
                         E1' (right; TEST_PLAN §4, the GT antenna lever arm fitted), log2 colour, seed wins, median value
  fig/heat_odom.png      the same for the odometry output (e1o / e1op; U: its E1 / E1')
  fig/heat_nis.png       (sequence, arm) x class: pooled mean NIS/dof × κ_AR (white-equivalent upper bound) and the
                         acceptance rate
With --white-review (tools/calib/p4_calib.py review JSON of T-diag runs = T + gnss_sats.csv) the exact white-equivalent
NIS of AMENDMENTS A58 decision 1 is tabulated per sequence (pooled over the seeds of the review, with the range).

Metrics (from ate.json): e1 (A52, global output traj_N), e1o (odometry output traj_N_odom, A61), e1p / e1op (their E1',
TEST_PLAN §4: the GT antenna lever arm estimated jointly with the alignment, A64 diagnostic); post30 and E4 as side tables.
Statistics (TEST_PLAN §4, applied to the development set's seeds): per sequence and metric, every arm against U on the same seeds
(paired differences arm − U), exact two-sided Wilcoxon signed-rank p (zeros dropped, average ranks for ties), Holm
over the arms of the sequence; "better" = Holm p < alpha, at least win-frac of the pairs won and median difference < 0,
"worse" symmetric, otherwise "tie". With n pairs the smallest attainable p is 2/2^n (0.0625 for five seeds): the verdict
column then cannot leave "tie" and says so; counts, medians and p are reported regardless. The same test (own Holm
family) compares T with each ablation and with L-PVT (difference T − other: < 0 means the removed class helped).

NIS: the pooled Σ NIS_prior / Σ dof over a run's epochs is what the filter reports (= summary.json mean_nis_per_dof; the
tool checks that the two agree, an instrument check of the diagnostics chain). For the classes whose receiver term is
inflated by κ_AR (pseudorange, PVT, Doppler; AMENDMENTS A58 decision 1) the reported value is by construction about
marginal/κ; κ × reported is an upper bound of the white-equivalent NIS (κ multiplies the receiver term only). The exact
white-equivalent needs per-satellite rows (arm T-diag + tools/calib/p4_calib.py review). The per-epoch χ² band
fraction (DESIGN §5 D4: 95 % of the epochs inside the two-sided 95 % band) is reported for TDCP (κ = 1) and Doppler.
Consistency flags (reported, never gating the tool's exit code): Doppler and TDCP pooled NIS/dof in [0.8, 1.25]
(DESIGN §5 D4); TDCP acceptance >= 70 % and NIS/dof <= 1.5 (TEST_PLAN §3 lock-up rule); no REANCHOR/REINIT.

Exit code: 0; 4 when an instrument self-check failed (ate.json INSTRUMENT statuses, summary.json vs gnss_epochs.csv);
3 when a sequence was refused (A2) or the input is unusable.
A2: evaluation-set sequences are reported only with --evaluation-phase; the guard runs before any of their files is read.
Runs anywhere with Python >= 3.8 + numpy; matplotlib only for the figures (missing ⇒ the report says so).
"""
import argparse
import csv
import gzip
import json
import math
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402

import eval_ate as ea  # noqa: E402
import evalcommon as ec  # noqa: E402

CLASSES = ("psr", "dopp", "tdcp", "pvt_pos", "pvt_vel")
TIGHT_CLASSES = ("psr", "dopp", "tdcp")
LOOSE_CLASSES = ("pvt_pos", "pvt_vel")
ACTIVE_PHASES = ("TRACKING", "OUTAGE", "RECOVERY")
EVENT_KINDS = ("ACTIVATE", "REINIT", "REANCHOR", "REINIT_YAW", "ROLLBACK", "CLOCK_JUMP", "SLIP", "LATE_EPOCH", "TOFF_WARN",
               "LIVO_ALARM_ON", "DEGENERATE_ON", "OUTAGE_ON", "RECOVERY", "REPIVOT", "PSEUDOINVERSE", "TDCP_PAUSED",
               "MSGDEF_REJECT", "CONFIG_WARNING", "FRAME_JUMP")
ABLATIONS = {"T-pr": "psr", "T-dop": "dopp", "T-tdcp": "tdcp"}
# the four ATE views of the report: (ate.json metric, output, E1 or E1')
VIEWS = (("e1", "global", "E1"), ("e1p", "global", "E1'"), ("e1o", "odom", "E1"), ("e1op", "odom", "E1'"))
VIEW_ZH = {"e1": "全局 E1", "e1p": "全局 E1′", "e1o": "里程计 E1", "e1op": "里程计 E1′"}
_RUN_PATH = re.compile(r"/([^/]+)/([^/]+)/s(\d+)(?:\.retry\d+)?/?$")
NIS_BAND = (0.8, 1.25)            # DESIGN §5 D4
TDCP_MIN_ACCEPT = 0.70            # TEST_PLAN §3 lock-up rule
TDCP_MAX_NIS = 1.5
FP_NONDEGENERATE_MAX = 1e-3       # DESIGN §5 D3 pre-registration: median f_p < 1e-3 in non-degenerate stretches
DOPP_DT_S = 0.1                   # rows.cpp: κ_AR(τ_D, 0.1 s)
SUMMARY_REL_TOL = 1e-9
OK = ea.OK

# figure style (dataviz reference palette: fixed categorical slot per arm, neutral ink for GT, blue-grey-red diverging)
SURFACE, INK, INK2, MUTED, GRID, AXIS = "#fcfcfb", "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#c3c2b7"
ARM_COLOR = {"U": "#2a78d6", "T": "#eb6834", "L-PVT": "#1baf7a", "T-pr": "#eda100", "T-dop": "#e87ba4", "T-tdcp": "#008300"}
GT_COLOR = "#52514e"
DIV_LO, DIV_MID, DIV_HI = "#1c5cab", "#f0efec", "#e34948"
SEQ_LO, SEQ_HI = "#cde2fb", "#104281"


# ---------------------------------------------------------------------------------------------------------------
# small statistics (no scipy: the project image's scipy does not import next to its numpy, tools/eval/README)
def _gammainc_lower_reg(a, x):
    """Regularised lower incomplete gamma P(a, x) (series below a + 1, Lentz continued fraction above)."""
    if x <= 0.0:
        return 0.0
    lg = math.lgamma(a)
    if x < a + 1.0:
        term = s = 1.0 / a
        ap = a
        for _ in range(10000):
            ap += 1.0
            term *= x / ap
            s += term
            if abs(term) < abs(s) * 1e-16:
                break
        return min(1.0, s * math.exp(-x + a * math.log(x) - lg))
    tiny = 1e-300
    b = x + 1.0 - a
    c = 1.0 / tiny
    d = 1.0 / b
    h = d
    for i in range(1, 10000):
        an = -i * (i - a)
        b += 2.0
        d = an * d + b
        d = tiny if abs(d) < tiny else d
        c = b + an / c
        c = tiny if abs(c) < tiny else c
        d = 1.0 / d
        dl = d * c
        h *= dl
        if abs(dl - 1.0) < 1e-16:
            break
    return max(0.0, 1.0 - math.exp(-x + a * math.log(x) - lg) * h)


def chi2_cdf(x, k):
    return _gammainc_lower_reg(0.5 * k, 0.5 * x)


_CHI2_PPF = {}


def chi2_ppf(p, k):
    """Quantile of χ²_k by bisection on the CDF (monotone); cached per (p, k)."""
    key = (p, k)
    if key in _CHI2_PPF:
        return _CHI2_PPF[key]
    lo, hi = 0.0, max(10.0, k + 20.0 * math.sqrt(2.0 * k))
    while chi2_cdf(hi, k) < p:
        hi *= 2.0
    for _ in range(200):
        mid = 0.5 * (lo + hi)
        if chi2_cdf(mid, k) < p:
            lo = mid
        else:
            hi = mid
        if hi - lo <= 1e-12 * max(1.0, hi):
            break
    _CHI2_PPF[key] = 0.5 * (lo + hi)
    return _CHI2_PPF[key]


def wilcoxon_exact(d):
    """Two-sided exact Wilcoxon signed-rank test of paired differences d (zeros dropped, average ranks for ties).
    -> dict(n, w_plus, p, p_min): p is None when no non-zero difference is left; p_min = the smallest attainable p."""
    x = [float(v) for v in d if v != 0.0 and math.isfinite(v)]
    n = len(x)
    if n == 0:
        return {"n": 0, "w_plus": None, "p": None, "p_min": None}
    a = sorted(range(n), key=lambda i: abs(x[i]))
    rank2 = [0] * n                                    # 2 × average rank (integers even with ties)
    i = 0
    while i < n:
        j = i
        while j + 1 < n and abs(x[a[j + 1]]) == abs(x[a[i]]):
            j += 1
        for k in range(i, j + 1):
            rank2[a[k]] = i + j + 2                    # 2 × mean of ranks i+1 .. j+1
        i = j + 1
    w2 = sum(r for r, v in zip(rank2, x) if v > 0)
    total = sum(rank2)
    counts = {0: 1}                                    # null distribution of 2·W+ over the 2^n sign assignments
    for r in rank2:
        nxt = dict(counts)
        for s, c in counts.items():
            nxt[s + r] = nxt.get(s + r, 0) + c
        counts = nxt
    norm = float(2 ** n)
    lower = sum(c for s, c in counts.items() if s <= w2) / norm
    upper = sum(c for s, c in counts.items() if s >= w2) / norm
    p = min(1.0, 2.0 * min(lower, upper))
    extreme = sum(c for s, c in counts.items() if s == 0) / norm      # all signs negative (tie-free: 1 / 2^n)
    return {"n": n, "w_plus": w2 / 2.0, "p": p, "p_min": min(1.0, 2.0 * extreme), "rank_sum": total / 2.0}


def holm(pvals):
    """Holm step-down adjusted p-values (None stays None)."""
    idx = [i for i, p in enumerate(pvals) if p is not None]
    idx.sort(key=lambda i: pvals[i])
    out = [None] * len(pvals)
    m, run = len(idx), 0.0
    for r, i in enumerate(idx):
        run = max(run, min(1.0, (m - r) * pvals[i]))
        out[i] = run
    return out


def kappa_ar(tau_s, dt_s):
    """κ_AR(τ, Δt) = (1+ρ)/(1−ρ), ρ = exp(−Δt/τ) (quality.cpp NoiseModel::kappaAR; τ ≤ 0 or Δt ≤ 0 ⇒ 1)."""
    try:
        tau_s, dt_s = float(tau_s), float(dt_s)
    except (TypeError, ValueError):
        return None
    if not (tau_s > 0.0) or not (dt_s > 0.0):
        return 1.0
    rho = math.exp(-dt_s / tau_s)
    return 1.0 if rho >= 1.0 else (1.0 + rho) / (1.0 - rho)


def median(v):
    v = [x for x in v if x is not None and math.isfinite(x)]
    return float(np.median(v)) if v else None


# ---------------------------------------------------------------------------------------------------------------
# run files
def open_text(path):
    """A text file, or its .gz twin (run_matrix.py --gzip-after), or None."""
    if os.path.isfile(path):
        return open(path, newline="", errors="replace")
    if os.path.isfile(path + ".gz"):
        return gzip.open(path + ".gz", "rt", newline="", errors="replace")
    return None


def find_gnss_config(obj):
    """The `gnss` parameter subtree of a run's config_dump (the whole parameter server, node namespace inside)."""
    if isinstance(obj, dict):
        g = obj.get("gnss")
        if isinstance(g, dict) and ("noise" in g or "enable" in g):
            return g
        for k in sorted(obj):
            r = find_gnss_config(obj[k])
            if r is not None:
                return r
    return None


def class_kappas(gnss_cfg):
    """Nominal κ_AR per class from the run's gnss configuration (rows.cpp / gnss_fusion.cpp intervals)."""
    if not gnss_cfg:
        return {c: None for c in CLASSES}
    noise = gnss_cfg.get("noise") or {}
    psr, dopp, pvt = noise.get("psr") or {}, noise.get("dopp") or {}, noise.get("pvt") or {}
    dec = psr.get("decim_s", 1.0)
    dec = dec if isinstance(dec, (int, float)) and dec > 0 else 1.0
    pdec = pvt.get("decim_s", 1.0)
    pdec = pdec if isinstance(pdec, (int, float)) and pdec > 0 else 1.0
    return {"psr": kappa_ar(psr.get("tau_s"), dec), "dopp": kappa_ar(dopp.get("tau_s"), DOPP_DT_S), "tdcp": 1.0,
            "pvt_pos": kappa_ar(pvt.get("tau_pos_s"), pdec), "pvt_vel": kappa_ar(pvt.get("tau_vel_s"), pdec)}


_EVENT = re.compile(r"^t_k=(\S+) t_est=(\S+) gpst=(\S+) (\S+) magnitude=(\S+) detail=(.*)$")


def read_events(path):
    """events.log -> list of dict(kind, t_k, magnitude, detail); None when the file is missing."""
    f = open_text(path)
    if f is None:
        return None
    out = []
    with f:
        for ln in f:
            m = _EVENT.match(ln.rstrip("\n"))
            if not m:
                continue
            try:
                detail = json.loads(m.group(6))
            except ValueError:
                detail = m.group(6)
            try:
                mag = float(m.group(5))
            except ValueError:
                mag = None
            out.append({"kind": m.group(4), "t_k": float(m.group(1)), "magnitude": mag, "detail": detail})
    return out


def parse_activate(detail):
    """The facade's ACTIVATE detail (gnss_fusion.cpp): epochs_used, sigma_yaw_deg, path, sigma_t_max_m, redundancy, sats."""
    out = {}
    for k, v in re.findall(r"(\w+)=(\S+)", detail or ""):
        try:
            out[k] = int(v) if re.fullmatch(r"-?\d+", v) else float(v)
        except ValueError:
            out[k] = v
    return out


def _num(row, key, cast=float):
    try:
        return cast(row[key])
    except (KeyError, TypeError, ValueError):
        return None


def read_epochs(path, kappas):
    """gnss_epochs.csv -> per-run diagnostics (pooled per class, phases, gain share), or None when missing."""
    f = open_text(path)
    if f is None:
        return None
    cls = {c: {"cand": 0, "acc": 0, "nis_sum": 0.0, "dof": 0, "n_nis_epochs": 0, "n_in_band": 0, "rejected": {},
               "lost_epochs": 0, "lost_cand": 0} for c in CLASSES}
    phases, fp, fp_all = {}, [], []
    n_rows, n_committed, n_rolled = 0, 0, 0
    with f:
        rd = csv.DictReader(f)
        hdr = rd.fieldnames or []
        rej_cols = {c: [h for h in hdr if h.startswith(c + "_rej_")] for c in CLASSES}
        for row in rd:
            n_rows += 1
            ph = row.get("phase", "?")
            phases[ph] = phases.get(ph, 0) + 1
            committed = row.get("committed") == "1"
            n_committed += committed
            n_rolled += row.get("rolled_back") == "1"
            for c in CLASSES:
                s = cls[c]
                cand, acc = _num(row, c + "_cand", int), _num(row, c + "_acc", int)
                if cand is None:
                    continue
                s["cand"] += cand
                s["acc"] += acc or 0
                dof = _num(row, c + "_dof", int) or 0
                if dof > 0:
                    nis = _num(row, c + "_nis_prior")
                    s["nis_sum"] += nis
                    s["dof"] += dof
                    s["n_nis_epochs"] += 1
                    if chi2_ppf(0.025, dof) <= nis <= chi2_ppf(0.975, dof):
                        s["n_in_band"] += 1
                for h in rej_cols[c]:
                    v = _num(row, h, int)
                    if v:
                        r = h[len(c) + 5:]
                        s["rejected"][r] = s["rejected"].get(r, 0) + v
                # a closing epoch whose pair members were all lost before the rows (every candidate 'mask', none built):
                # the facade builds no TDCP row set there and its summary totals skip the epoch (FACADE_TOTALS_RULE)
                if cand and not acc and not dof and _num(row, c + "_rej_mask", int) == cand:
                    s["lost_epochs"] += 1
                    s["lost_cand"] += cand
            if committed:
                v = _num(row, "f_p")
                if v is not None and math.isfinite(v):
                    fp_all.append(v)
                    if row.get("degenerate") != "1":
                        fp.append(v)
    for c in CLASSES:
        s = cls[c]
        s["acc_rate"] = s["acc"] / s["cand"] if s["cand"] else None
        s["nis_per_dof"] = s["nis_sum"] / s["dof"] if s["dof"] else None
        s["kappa"] = kappas.get(c)
        s["nis_per_dof_x_kappa"] = (s["nis_per_dof"] * s["kappa"]) if (s["nis_per_dof"] is not None and s["kappa"]) else None
        s["band_frac"] = s["n_in_band"] / s["n_nis_epochs"] if s["n_nis_epochs"] else None
    return {"rows": n_rows, "committed": n_committed, "rolled_back": n_rolled, "phases": phases, "classes": cls,
            "active_epochs": sum(phases.get(p, 0) for p in ACTIVE_PHASES),
            "f_p_median": median(fp), "f_p_frac_below": (sum(1 for v in fp if v < FP_NONDEGENERATE_MAX) / len(fp)) if fp else None,
            "f_p_median_all": median(fp_all), "n_f_p": len(fp)}


FACADE_TOTALS_RULE = ("gnss_fusion.cpp classStats: the summary totals are added after `if (!s) return;`, so a TDCP "
                      "closing epoch with no row set (every pair member lost before the rows, with_tdcp false) is in "
                      "gnss_epochs.csv (candidates = 'mask' rejections) but not in summary.json's TDCP acceptance")


def _differs(mine, theirs):
    return (mine is None) != (theirs is None) or (mine is not None and abs(mine - theirs) > SUMMARY_REL_TOL * max(1.0, abs(theirs)))


def summary_check(summary, ep):
    """summary.json (the facade's totals) against the recomputation from gnss_epochs.csv: -> 'ok' | 'ok (facade totals …)' |
    'MISMATCH: …'. The TDCP acceptance is also compared under the facade's own totals rule (FACADE_TOTALS_RULE): a
    difference that is exactly the all-lost closing epochs is a known definitional one, reported, not a mismatch."""
    if summary is None or ep is None:
        return "skipped: %s missing" % ("summary.json" if summary is None else "gnss_epochs.csv")
    bad, notes = [], []
    nis = summary.get("mean_nis_per_dof") or {}
    acc = summary.get("acceptance_rate") or {}
    for c in CLASSES:
        s = ep["classes"][c]
        for name, mine, theirs in (("nis/dof", s["nis_per_dof"], nis.get(c)), ("acceptance", s["acc_rate"], acc.get(c))):
            if mine is None and theirs is None:
                continue
            if not _differs(mine, theirs):
                continue
            if name == "acceptance" and c == "tdcp" and s["lost_cand"] and s["cand"] > s["lost_cand"]:
                alt = s["acc"] / (s["cand"] - s["lost_cand"])
                if not _differs(alt, theirs):
                    notes.append("%s acceptance: summary %.6f = epochs without the %d all-lost closing epoch(s) (%d candidates); "
                                 "epochs %.6f" % (c, theirs, s["lost_epochs"], s["lost_cand"], mine))
                    continue
            bad.append("%s %s: epochs %s vs summary %s" % (c, name, mine, theirs))
    if bad:
        return "MISMATCH: " + "; ".join(bad[:4])
    return "ok" if not notes else "ok (facade totals rule: %s)" % "; ".join(notes)


def run_gnss(root, r, gnss_arm):
    """Per-run GNSS diagnostics from its directory (None for the host arm)."""
    if not gnss_arm or not r.get("run_dir"):
        return None
    rd = os.path.join(root, r["run_dir"])
    man, cfg = None, None
    mp = os.path.join(rd, "run_manifest.json")
    if os.path.isfile(mp):
        try:
            man = ec.load_json(mp)
            dump = man.get("config_dump")
            cfg = find_gnss_config(json.loads(dump) if isinstance(dump, str) and dump else dump)
        except ValueError:
            man = None
    kap = class_kappas(cfg)
    ep = read_epochs(os.path.join(rd, "gnss_epochs.csv"), kap)
    summary = None
    sp = os.path.join(rd, "summary.json")
    if os.path.isfile(sp):
        try:
            summary = ec.load_json(sp)
        except ValueError:
            summary = None
    ev = read_events(os.path.join(rd, "events.log"))
    counts = {k: 0 for k in EVENT_KINDS}
    act = None
    for e in ev or []:
        if e["kind"] in counts:                    # PHASE_CHANGE (every state-machine step) is not counted
            counts[e["kind"]] += 1
        if e["kind"] == "ACTIVATE" and act is None:
            act = dict(parse_activate(e["detail"] if isinstance(e["detail"], str) else ""), t_k=e["t_k"])
    span0 = (r.get("e1_span") or [None])[0]
    t_act = (r.get("activation") or {}).get("t_act")
    out = {"config_found": cfg is not None, "mode": (cfg or {}).get("mode"), "use": (cfg or {}).get("use"),
           "kappa": kap, "epochs": ep, "summary": summary, "summary_check": summary_check(summary, ep),
           "events": counts if ev is not None else None, "first_activate": act,
           "activated": r.get("activated"), "t_act_after_span_start_s": (t_act - span0) if (t_act is not None and span0 is not None) else None,
           "tdcp_coverage": (summary or {}).get("tdcp_coverage"),
           "elapsed_ms": {k: (summary or {}).get("elapsed_%s_ms" % k) for k in ("p50", "p95", "p99")}}
    return out


# ---------------------------------------------------------------------------------------------------------------
# comparisons
def paired(vals_a, vals_b):
    """{seed: value} x2 -> the paired sample (a − b over common seeds)."""
    seeds = sorted(set(vals_a) & set(vals_b))
    d = [vals_a[k] - vals_b[k] for k in seeds]
    ratio = [vals_a[k] / vals_b[k] for k in seeds if vals_b[k] > 0]
    return seeds, d, ratio


def compare(name, a, b, vals, win_frac):
    """Paired comparison of arm a against arm b on one sequence (difference a − b; a 'wins' when it is smaller)."""
    seeds, d, ratio = paired(vals.get(a, {}), vals.get(b, {}))
    w = wilcoxon_exact(d)
    wins = sum(1 for x in d if x < 0)
    losses = sum(1 for x in d if x > 0)
    return {"name": name, "a": a, "b": b, "seeds": seeds, "n": len(d), "wins": wins, "losses": losses,
            "median_diff_m": median(d), "median_ratio": median(ratio), "diffs_m": d, "p": w["p"], "p_min": w["p_min"],
            "need_wins": int(math.ceil(win_frac * len(d) - 1e-9)) if d else None}


def verdicts(comps, alpha):
    """Holm within the family, then the TEST_PLAN §4 rule on each comparison (in place)."""
    adj = holm([c["p"] for c in comps])
    for c, ph in zip(comps, adj):
        c["p_holm"] = ph
        if not c["n"]:
            c["verdict"] = "n/a"
            continue
        if c["p_min"] is not None and c["p_min"] >= alpha:
            c["verdict"] = "tie (n=%d: min p %.4g ≥ α)" % (c["n"], c["p_min"])
            continue
        sig = ph is not None and ph < alpha
        if sig and c["wins"] >= c["need_wins"] and c["median_diff_m"] < 0:
            c["verdict"] = "better"
        elif sig and c["losses"] >= c["need_wins"] and c["median_diff_m"] > 0:
            c["verdict"] = "worse"
        else:
            c["verdict"] = "tie"
    return comps


# ---------------------------------------------------------------------------------------------------------------
def build(args):
    ate_path = os.path.abspath(args.ate)
    ate = ec.load_json(ate_path)
    root = ec.project_root(args.root or ate.get("root"))
    eval_dir = os.path.dirname(ate_path)
    arm_order = list(ec.load_arms())
    rep = {"tool": "tools/eval/dev_report.py", "generated_utc": ec.utc_now(), "argv": sys.argv, "ate_json": ate_path,
           "ate_generated_utc": ate.get("generated_utc"), "root": root, "matrices": ate.get("matrices"), "metric": ate.get("metric"),
           "params": {"u_arm": args.u_arm, "t_arm": args.t_arm, "alpha": args.alpha, "win_frac": args.win_frac,
                      "nis_band": NIS_BAND, "tdcp_min_accept": TDCP_MIN_ACCEPT, "tdcp_max_nis": TDCP_MAX_NIS},
           "sequences": {}, "runs": [], "cells": [], "blocked": {}, "self_checks": {}, "figures": {}}
    seqs = []
    for seq in ate.get("sequences", {}):
        try:
            ec.check_phase(seq, args.evaluation_phase, "the development report")
        except ec.PhaseError as e:
            rep["blocked"][seq] = str(e)
            continue
        seqs.append(seq)
    for seq, why in (ate.get("blocked") or {}).items():
        rep["blocked"].setdefault(seq, "eval_ate: %s" % why)
    runs = [r for r in ate.get("runs", []) if r.get("seq") in seqs]

    def mval(m, name, ok):
        return m.get(name, {}).get("value") if ok and m.get(name, {}).get("status") in OK else None

    def lever(m, name, ok):
        return ((m.get(name) or {}).get("lever") or {}).get("lever_m") if mval(m, name, ok) is not None else None
    # per run
    for r in runs:
        gnss_arm = r.get("traj") != ea.HOST_TRAJ
        m = r.get("metrics") or {}
        ok = bool(m) and m.get("e1", {}).get("status") in OK and not str(r.get("status", "")).startswith("INSTRUMENT")
        row = {"seq": r["seq"], "arm": r["arm"], "seed": r["seed"], "run_dir": r.get("run_dir"), "gnss_arm": gnss_arm,
               "status": r.get("status"), "ok": ok, "e1_m": m.get("e1", {}).get("value") if ok else None,
               "post30_m": mval(m, "post30", ok), "e4_m": mval(m, "e4", ok),
               "e1o_m": mval(m, "e1o", ok),                                                   # A61 F13
               "e1p_m": mval(m, "e1p", ok), "e1op_m": mval(m, "e1op", ok),                    # E1' (A64)
               "e1p_lever_m": lever(m, "e1p", ok), "e1op_lever_m": lever(m, "e1op", ok),
               "activated": r.get("activated"), "config_sha256": r.get("config_sha256"), "build_sha256": r.get("build_sha256"),
               "pre_activation_vs_U": (r.get("checks") or {}).get("pre_activation_vs_U", {}).get("status")}
        row["gnss"] = run_gnss(root, r, gnss_arm) if r.get("run_dir") else None
        rep["runs"].append(row)
    # per cell and per sequence
    for seq in seqs:
        srun = [x for x in rep["runs"] if x["seq"] == seq]
        arms = sorted({x["arm"] for x in srun}, key=lambda a: arm_order.index(a) if a in arm_order else 99)
        by_metric, contrib_by = {}, {}
        for metric, _, _ in VIEWS:
            vals = {a: {x["seed"]: x[metric + "_m"] for x in srun if x["arm"] == a and x[metric + "_m"] is not None} for a in arms}
            by_metric[metric] = verdicts([compare("%s vs %s" % (a, args.u_arm), a, args.u_arm, vals, args.win_frac)
                                          for a in arms if a != args.u_arm], args.alpha) if args.u_arm in arms else []
            contrib = []
            if args.t_arm in arms:
                for a in arms:
                    if a in ABLATIONS:
                        contrib.append(compare("%s vs %s" % (args.t_arm, a), args.t_arm, a, vals, args.win_frac))
                if "L-PVT" in arms and args.t_arm != "L-PVT":
                    contrib.append(compare("%s vs L-PVT" % args.t_arm, args.t_arm, "L-PVT", vals, args.win_frac))
                verdicts(contrib, args.alpha)
            contrib_by[metric] = contrib
        ate_seq = ate["sequences"].get(seq, {})
        rep["sequences"][seq] = {"class": ec.seq_class(seq), "vs_u": by_metric["e1"], "contributions": contrib_by["e1"],
                                 "vs_u_by_metric": by_metric, "contributions_by_metric": contrib_by,
                                 "gt": ate_seq.get("gt"), "mask": ate_seq.get("mask"), "e1_spans": ate_seq.get("e1_spans"),
                                 "post30_spans": ate_seq.get("post30_spans"),
                                 "a55_same_build": ate_seq.get("a55_same_build"), "arms": arms}
        for a in arms:
            rs = [x for x in srun if x["arm"] == a]
            ac = next((c for c in ate.get("cells", []) if c["seq"] == seq and c["arm"] == a), {})
            cell = {"seq": seq, "arm": a, "gnss_arm": any(x["gnss_arm"] for x in rs), "n_runs": len(rs),
                    "n_ok": sum(1 for x in rs if x["ok"]), "e1": {k: ac.get(k) for k in ("median_m", "min_m", "max_m", "n_ok")},
                    "n_distinct": ac.get("n_distinct"), "identical_seed_groups": ac.get("identical_seed_groups"),
                    "vs_u": next((c for c in by_metric["e1"] if c["a"] == a), None),
                    "vs_u_by_metric": {k: next((c for c in v if c["a"] == a), None) for k, v in by_metric.items()}}
            for name in ("post30", "e4", "e1o", "e1p", "e1op"):
                cell[name] = {k: (ac.get(name) or {}).get(k) for k in ("median_m", "min_m", "max_m", "n_ok")}
            for name in ("e1p", "e1op"):
                lv = [x[name + "_lever_m"] for x in rs if x.get(name + "_lever_m")]
                cell[name]["lever_median_m"] = [float(np.median([v[i] for v in lv])) for i in range(3)] if lv else None
            if cell["gnss_arm"]:
                cell["gnss"] = cell_gnss([x["gnss"] for x in rs if x["gnss"]])
            rep["cells"].append(cell)
    rep["white"] = white_equivalent(args.white_review, seqs, args.evaluation_phase)
    rep["self_checks"] = self_checks(rep, ate, seqs)
    rep["flags"] = consistency_flags(rep, args.t_arm)
    return rep, eval_dir


def white_equivalent(paths, seqs, evaluation_phase):
    """Exact white-equivalent NIS (AMENDMENTS A58 decision 1) from tools/calib/p4_calib.py review JSONs of T-diag runs:
    per (sequence, arm) the dof-pooled value over the seeds and the per-seed range of the pseudorange and Doppler rows
    ('common' = clock-eliminated, the A58/A65 number) and the self-reported TDCP NIS (A62 ⑦). The run path gives the
    sequence, arm and seed; a sequence outside the report (or an evaluation sequence without --evaluation-phase) is
    skipped before its numbers are used."""
    out = {"files": list(paths or []), "cells": {}, "skipped": []}
    for path in paths or []:
        for rec in ec.load_json(path):
            m = _RUN_PATH.search(str(rec.get("run", "")))
            if not m:
                out["skipped"].append(str(rec.get("run")))
                continue
            seq, arm, seed = m.group(1), m.group(2), int(m.group(3))
            try:
                ec.check_phase(seq, evaluation_phase, "the development report")
            except ec.PhaseError:
                out["skipped"].append(str(rec.get("run")))
                continue
            if seq not in seqs:
                out["skipped"].append(str(rec.get("run")))
                continue
            we = rec.get("white_equivalent") or {}
            c = out["cells"].setdefault("%s/%s" % (seq, arm), {"seq": seq, "arm": arm, "seeds": [], "psr": [], "dopp": [], "tdcp": []})
            c["seeds"].append(seed)
            for key, cls in (("p", "psr"), ("d", "dopp")):
                x = (we.get(key) or {}).get("common") or {}
                if x.get("dof"):
                    c[cls].append((x["nis_per_dof"], x["dof"]))
            t = rec.get("tdcp") or {}
            if t.get("dof_sum"):
                c["tdcp"].append((t["pooled_nis_per_dof"], t["dof_sum"]))
    for c in out["cells"].values():
        for cls in ("psr", "dopp", "tdcp"):
            v = c[cls]
            c[cls] = {"pooled": sum(a * b for a, b in v) / sum(b for _, b in v) if v else None,
                      "min": min(a for a, _ in v) if v else None, "max": max(a for a, _ in v) if v else None,
                      "n": len(v), "dof": sum(b for _, b in v)}
        c["seeds"] = sorted(c["seeds"])
    return out


def cell_gnss(gs):
    """Seed aggregate of per-run GNSS diagnostics: pooled class totals, medians, event sums."""
    out = {"n": len(gs), "n_activated": sum(1 for g in gs if g.get("activated")),
           "t_act_median_s": median([g.get("t_act_after_span_start_s") for g in gs]),
           "t_act_all": [g.get("t_act_after_span_start_s") for g in gs if g.get("activated")],
           "sigma_yaw_act_median_deg": median([(g.get("first_activate") or {}).get("sigma_yaw_deg") for g in gs]),
           "act_paths": sorted({str((g.get("first_activate") or {}).get("path")) for g in gs if g.get("first_activate")}),
           "tdcp_coverage_median": median([g.get("tdcp_coverage") for g in gs]),
           "f_p_median": median([(g.get("epochs") or {}).get("f_p_median") for g in gs]),
           "summary_checks": sorted({g["summary_check"] for g in gs}), "classes": {}, "events": {}, "events_max": {},
           "mode": sorted({str(g.get("mode")) for g in gs})}
    for c in CLASSES:
        tot = {"cand": 0, "acc": 0, "nis_sum": 0.0, "dof": 0, "n_nis_epochs": 0, "n_in_band": 0}
        per_seed, kap = [], None
        for g in gs:
            s = ((g.get("epochs") or {}).get("classes") or {}).get(c)
            if not s:
                continue
            for k in tot:
                tot[k] += s[k]
            if s["nis_per_dof"] is not None:
                per_seed.append(s["nis_per_dof"])
            kap = s["kappa"] if s["kappa"] is not None else kap
        if not tot["cand"] and not tot["dof"]:
            continue
        nis = tot["nis_sum"] / tot["dof"] if tot["dof"] else None
        out["classes"][c] = {"cand": tot["cand"], "acc": tot["acc"], "acc_rate": tot["acc"] / tot["cand"] if tot["cand"] else None,
                             "nis_per_dof": nis, "nis_per_dof_min": min(per_seed) if per_seed else None,
                             "nis_per_dof_max": max(per_seed) if per_seed else None, "kappa": kap,
                             "nis_per_dof_x_kappa": nis * kap if (nis is not None and kap) else None,
                             "band_frac": tot["n_in_band"] / tot["n_nis_epochs"] if tot["n_nis_epochs"] else None,
                             "dof": tot["dof"]}
    for g in gs:
        for k, v in (g.get("events") or {}).items():
            out["events"][k] = out["events"].get(k, 0) + v
            out["events_max"][k] = max(out["events_max"].get(k, 0), v)
    return out


def self_checks(rep, ate, seqs):
    """The instrument part of the step-1 entry condition (NEXT_STEPS: 仪器自检全过)."""
    runs = rep["runs"]
    inst = [("%s/%s/s%d" % (r["seq"], r["arm"], r["seed"]), r["status"]) for r in runs if "INSTRUMENT" in str(r.get("status", ""))]
    missing = [("%s/%s/s%d" % (r["seq"], r["arm"], r["seed"]), r["status"]) for r in runs
               if not r["ok"] and "INSTRUMENT" not in str(r.get("status", ""))]
    diag = [("%s/%s/s%d" % (r["seq"], r["arm"], r["seed"]), r["gnss"]["summary_check"]) for r in runs
            if r["gnss"] and r["gnss"]["summary_check"] != "ok"]
    diag_bad = [x for x in diag if x[1].startswith("MISMATCH")]
    diag_rule = [x for x in diag if x[1].startswith("ok (facade totals rule")]
    a55 = {s: ate["sequences"].get(s, {}).get("a55_same_build") for s in seqs}
    pre = [("%s/%s/s%d" % (r["seq"], r["arm"], r["seed"]), r["pre_activation_vs_U"]) for r in runs
           if r["gnss_arm"] and r["ok"] and r["pre_activation_vs_U"] != "identical"]
    collapse = [("%s/%s" % (c["seq"], c["arm"]), c["identical_seed_groups"]) for c in rep["cells"] if c.get("identical_seed_groups")]
    checks = {
        "ate_instrument": {"status": "PASS" if not inst else "FAIL", "failures": inst},
        "diagnostics_chain": {"status": "FAIL" if diag_bad else ("PASS" if not diag else (
            "PASS (facade totals rule)" if len(diag_rule) == len(diag) else "PASS (with skips)")), "items": diag,
            "facade_totals_rule": FACADE_TOTALS_RULE if diag_rule else None},
        # None = not applicable (eval_ate: no U or no GNSS run with a build hash in the sequence)
        "a55_one_binary": {"status": "FAIL" if any(v is False for v in a55.values()) else "PASS", "per_sequence": a55},
        "i11_pre_activation_equals_U": {"status": "PASS" if not pre else "FAIL", "items": pre},
        "complete": {"status": "PASS" if not missing else "INCOMPLETE", "items": missing},
        "seed_collapse": {"status": "INFO", "items": collapse},
    }
    checks["instrument_all_pass"] = all(checks[k]["status"].startswith("PASS")
                                        for k in ("ate_instrument", "diagnostics_chain", "a55_one_binary", "i11_pre_activation_equals_U"))
    return checks


def consistency_flags(rep, t_arm):
    """DESIGN §5 D4 / TEST_PLAN §3 flags on the GNSS arms (reported; the explanation is the reader's job)."""
    flags = []
    for c in rep["cells"]:
        g = c.get("gnss")
        if not g:
            continue
        cl = g["classes"]
        for k in ("dopp", "tdcp"):
            x = (cl.get(k) or {}).get("nis_per_dof")
            if x is not None and not (NIS_BAND[0] <= x <= NIS_BAND[1]):
                flags.append({"seq": c["seq"], "arm": c["arm"], "flag": "%s NIS/dof %.3f outside [%.2f, %.2f] (D4)" % (k, x, NIS_BAND[0], NIS_BAND[1])})
        t = cl.get("tdcp")
        if t and t["acc_rate"] is not None and (t["acc_rate"] < TDCP_MIN_ACCEPT or (t["nis_per_dof"] or 0.0) > TDCP_MAX_NIS):
            flags.append({"seq": c["seq"], "arm": c["arm"], "flag": "TDCP acceptance %.1f%% / NIS/dof %s: lock-up rule (TEST_PLAN §3)" % (
                100.0 * t["acc_rate"], "%.3f" % t["nis_per_dof"] if t["nis_per_dof"] is not None else "—")})
        for k in ("REANCHOR", "REINIT"):
            if g["events"].get(k):
                flags.append({"seq": c["seq"], "arm": c["arm"], "flag": "%d × %s over %d seeds" % (g["events"][k], k, g["n"])})
        if g["n"] and g["n_activated"] < g["n"]:
            flags.append({"seq": c["seq"], "arm": c["arm"], "flag": "activated in %d of %d seeds" % (g["n_activated"], g["n"])})
        if g["f_p_median"] is not None and g["f_p_median"] >= FP_NONDEGENERATE_MAX:
            flags.append({"seq": c["seq"], "arm": c["arm"], "flag": "median f_p %.2g ≥ %.0e (D3 pre-registration)" % (g["f_p_median"], FP_NONDEGENERATE_MAX)})
    return flags


# ---------------------------------------------------------------------------------------------------------------
# output
def f3(x, fmt="%.3f"):
    return "—" if x is None else fmt % x


def pct(x):
    return "—" if x is None else "%.1f%%" % (100.0 * x)


def verdict_zh(v):
    if not v:
        return "—"
    if v.startswith("tie (n="):
        return "平局(" + v[5:-1].replace("min p", "最小可达 p") + ")"
    return {"better": "优于", "worse": "劣于", "tie": "平局", "n/a": "—"}.get(v, v)


def write_markdown(rep, path, figs):
    L = []
    add = L.append
    add("# 开发集评测报告(`tools/eval/dev_report.py`;AMENDMENTS A59 的素材)")
    add("")
    add("生成 %s;输入 `%s`(eval_ate %s)。矩阵:%s。" % (
        rep["generated_utc"], os.path.relpath(rep["ate_json"], rep["root"]) if rep["ate_json"].startswith(rep["root"]) else rep["ate_json"],
        rep.get("ate_generated_utc") or "?",
        ", ".join("%s(%s)" % (m.get("name"), (m.get("source_id") or "?")[:12]) for m in (rep.get("matrices") or [])) or "—"))
    add("")
    sc = rep["self_checks"]
    add("## 1 仪器自检(进入评测集的条件之一)")
    add("")
    add("| 检查 | 结果 | 说明 |")
    add("|---|---|---|")
    add("| ATE 仪器(evo 与 numpy 复算、T_{N←W} 航向、ACTIVATE 对 traj_N)| %s | %s |" % (
        sc["ate_instrument"]["status"], "; ".join("%s: %s" % x for x in sc["ate_instrument"]["failures"][:6]) or "ate.json 无 INSTRUMENT"))
    dc = sc["diagnostics_chain"]
    add("| 诊断链(summary.json 与 gnss_epochs.csv 重算一致)| %s | %s |" % (
        dc["status"], ("全部一致" if not dc["items"] else ("%d 个运行的 TDCP 接受率只差「整对丢失的收尾历元」(门面汇总规则,见下)" % len(dc["items"])
                                                     if dc.get("facade_totals_rule") and dc["status"].startswith("PASS") else
                                                     "; ".join("%s: %s" % x for x in dc["items"][:6])))))
    add("| A55 同一二进制 | %s | %s |" % (sc["a55_one_binary"]["status"], ", ".join(
        "%s=%s" % (k, {True: "同", False: "不同", None: "不适用"}[v]) for k, v in sc["a55_one_binary"]["per_sequence"].items())))
    add("| I11 激活前宿主 = 同种子 U | %s | %s |" % (
        sc["i11_pre_activation_equals_U"]["status"], "; ".join("%s: %s" % x for x in sc["i11_pre_activation_equals_U"]["items"][:6]) or "全部逐位相同"))
    add("| 完整性 | %s | %s |" % (sc["complete"]["status"], "; ".join("%s: %s" % x for x in sc["complete"]["items"][:6]) or "每格运行齐全"))
    add("| 种子塌缩(A30,信息)| — | %s |" % ("; ".join("%s %s" % (a, g) for a, g in sc["seed_collapse"]["items"]) or "无"))
    add("")
    add("**仪器自检:%s**。" % ("全过" if sc["instrument_all_pass"] else "未全过(见上表)"))
    if dc.get("facade_totals_rule"):
        add("")
        add("门面汇总规则:%s。本报告与 A58/A62/A65 的接受率都按 gnss_epochs.csv 计(含这些候选);summary.json 的值按门面规则去掉它们后逐位重合。"
            "涉及:%s。" % (dc["facade_totals_rule"], "; ".join(x[0] for x in dc["items"][:12]) + (" 等 %d 个" % len(dc["items"]) if len(dc["items"]) > 12 else "")))
    if rep["blocked"]:
        add("")
        add("未报告的序列:" + "; ".join("%s(%s)" % kv for kv in rep["blocked"].items()))
    add("")
    add("| 序列 | GT 时基(A54) | E1 时段 (s) | 其中掩掉 (s) |")
    add("|---|---|---|---|")
    for seq, s in rep["sequences"].items():
        gt = s.get("gt") or {}
        sp = (s.get("e1_spans") or [{}])
        shift = gt.get("time_shift_applied_s")
        add("| %s | %s%s | %s | %s |" % (seq, gt.get("method", "?"), "" if shift is None else "(%+.4f s)" % shift,
                                        "/".join(f3(x.get("length_s"), "%.1f") for x in sp),
                                        "/".join(f3(x.get("masked_s"), "%.1f") for x in sp)))
    add("")
    add("## 2 ATE:全局输出与里程计输出,E1 与 E1′(m,中位 [最小, 最大],种子数)")
    add("")
    add("| 序列 | 臂 | 全局 E1 | 全局 E1′ | 里程计 E1 | 里程计 E1′ | 对 U 胜/n:全局 E1 · E1′ · 里程计 E1 · E1′ |")
    add("|---|---|---|---|---|---|---|")
    for c in rep["cells"]:
        dup = " ‡%d" % c["n_distinct"] if c.get("identical_seed_groups") else ""
        cols = []
        for metric, _, _ in VIEWS:
            e = c.get(metric) or {}
            cols.append("%s [%s, %s] (%s)" % (f3(e.get("median_m")), f3(e.get("min_m")), f3(e.get("max_m")),
                                             e.get("n_ok") if e.get("n_ok") is not None else "—"))
        vs = []
        for metric, _, _ in VIEWS:
            v = (c.get("vs_u_by_metric") or {}).get(metric)
            vs.append("%d/%d" % (v["wins"], v["n"]) if v else "—")
        add("| %s | %s%s | %s | %s |" % (c["seq"], c["arm"], dup, " | ".join(cols), " · ".join(vs) if c["arm"] != rep["params"]["u_arm"] else "(基准)"))
    add("")
    add("全局 = GNSS 臂的 traj_N(激活前用激活时刻的 T_{N←W} 补齐,A52),U 为 traj_W;里程计 = traj_N_odom(A61 F13,配准固定在"
        "激活时刻的估计,之后的漂移修正全部保留;与 FAST-LIVGO「初始化后固定 W→ENU」同口径),U 为其 E1。E1′ = 同一文件、同一时段与"
        "关联,真值天线杆臂(3 参数,IMU 系)与 SE(3) 对齐联合估计(TEST_PLAN §4,A64;诊断列,主表仍为 E1)。‡m = 该格种子在时段内只有 m 条不同轨迹。")
    add("")
    add("### 2.1 对 U 的成对比较(同种子,差 = 臂 − U,负为好于 U)")
    add("")
    add("| 序列 | 臂 | 口径 | 胜 / n | 成对差中位 (m) | 比值中位 | p(Holm) | 判定 |")
    add("|---|---|---|---|---|---|---|---|")
    for seq, sq in rep["sequences"].items():
        for metric, _, _ in VIEWS:
            for v in (sq.get("vs_u_by_metric") or {}).get(metric) or []:
                add("| %s | %s | %s | %d / %d | %s | %s | %s | %s |" % (seq, v["a"], VIEW_ZH[metric], v["wins"], v["n"],
                                                               f3(v["median_diff_m"], "%+.3f"), f3(v["median_ratio"]),
                                                               f3(v["p_holm"], "%.3g"), verdict_zh(v["verdict"])))
    add("")
    add("判定按 TEST_PLAN §4(Holm 后 p < %g、≥ %.0f%% 种子胜、成对差中位同号;每个口径各自一族),种子数使最小可达 p ≥ α 时只能判平局。"
        % (rep["params"]["alpha"], 100 * rep["params"]["win_frac"]))
    add("")
    add("### 2.2 附表:激活 30 s 后(post30)、E4(各臂 traj_W),E1′ 拟合的杆臂")
    add("")
    add("| 序列 | 臂 | 激活 30 s 后 | E4 | E1′ 杆臂中位 全局 (x, y, z) | E1′ 杆臂中位 里程计 (x, y, z) |")
    add("|---|---|---|---|---|---|")
    for c in rep["cells"]:
        lg = (c.get("e1p") or {}).get("lever_median_m")
        lo = (c.get("e1op") or {}).get("lever_median_m")
        add("| %s | %s | %s [%s, %s] | %s [%s, %s] | %s | %s |" % (
            c["seq"], c["arm"], f3(c["post30"]["median_m"]), f3(c["post30"]["min_m"]), f3(c["post30"]["max_m"]),
            f3(c["e4"]["median_m"]), f3(c["e4"]["min_m"]), f3(c["e4"]["max_m"]),
            "(%s)" % ", ".join("%+.3f" % x for x in lg) if lg else "—", "(%s)" % ", ".join("%+.3f" % x for x in lo) if lo else "—"))
    add("")
    add("post30 = 该序列最晚一次激活 + 30 s 到 E1 时段末(所有臂同一时段;从未激活的运行无此值);E4 = 各臂自己的 traj_W(GNSS 是否改动宿主)。"
        "杆臂 = E1′ 拟合的真值天线在 IMU 系的位置;竖直分量只由横滚/俯仰的变化观测,形式 σ 见 ate.json。")
    add("")
    add("## 3 GNSS 诊断(每格,种子汇总)")
    add("")
    add("| 序列 | 臂 | 激活 | 激活时刻中位 (s) | σ_ψ 激活 (°) | 接受率 伪距/多普勒/TDCP | NIS/dof 自报 伪距/多普勒/TDCP | κ×NIS 上界 伪距(PVT 位置) | χ² 带内 多普勒/TDCP(PVT 速度) | TDCP 覆盖 | f_p 中位 | 事件 |")
    add("|---|---|---|---|---|---|---|---|---|---|---|---|")
    for c in rep["cells"]:
        g = c.get("gnss")
        if not g:
            continue
        cl = g["classes"]
        loose = any(k in cl for k in LOOSE_CLASSES) and not any(k in cl for k in TIGHT_CLASSES)
        keys = LOOSE_CLASSES if loose else TIGHT_CLASSES
        accs = "/".join(pct((cl.get(k) or {}).get("acc_rate")) for k in keys)
        niss = "/".join(f3((cl.get(k) or {}).get("nis_per_dof"), "%.3g") for k in keys)
        kx = f3((cl.get("pvt_pos" if loose else "psr") or {}).get("nis_per_dof_x_kappa"), "%.3g")
        band = "/".join(pct((cl.get(k) or {}).get("band_frac")) for k in (("pvt_vel",) if loose else ("dopp", "tdcp")))
        ev = ", ".join("%s %d" % (k, n) for k, n in g["events"].items() if n and k not in ("ACTIVATE",)) or "—"
        add("| %s | %s | %d/%d | %s | %s | %s%s | %s | %s | %s | %s | %s | %s |" % (
            c["seq"], c["arm"], g["n_activated"], g["n"], f3(g["t_act_median_s"], "%.1f"), f3(g["sigma_yaw_act_median_deg"], "%.2f"),
            accs, "(PVT 位置/速度)" if loose else "", niss, kx, band, pct(g["tdcp_coverage_median"]) if not loose else "—",
            f3(g["f_p_median"], "%.2g"), ev))
    add("")
    add("NIS/dof = 各种子 Σ NIS_prior / Σ dof 合并(= summary.json 的 mean_nis_per_dof);κ_AR>1 的类按构造约为「边际值/κ」,"
        "κ×自报是白等效 NIS 的上界(κ 只乘接收机项;精确值用 T-diag 臂 + `tools/calib/p4_calib.py review`,A58 裁定 1)。"
        "χ² 带内 = dof>0 的历元里先验 NIS 落在 χ²_dof 双侧 95% 带的比例(DESIGN §5 D4 要 ≥95%)。事件为各种子之和。")
    add("")
    add("| 序列 | 臂 | 激活路径 | 激活时刻(E1 时段起算,s)中位 [最小, 最大] | σ_ψ 激活 (°) |")
    add("|---|---|---|---|---|")
    for c in rep["cells"]:
        g = c.get("gnss")
        if not g:
            continue
        ta = [x for x in g.get("t_act_all") or [] if x is not None]
        add("| %s | %s | %s | %s | %s |" % (c["seq"], c["arm"], ", ".join(g["act_paths"]) or "未激活",
                                           "%s [%s, %s]" % (f3(g["t_act_median_s"], "%.1f"), f3(min(ta), "%.1f"), f3(max(ta), "%.1f")) if ta else "—",
                                           f3(g["sigma_yaw_act_median_deg"], "%.2f")))
    add("")
    wh = rep.get("white") or {}
    if wh.get("cells"):
        add("**白等效 NIS**(A58 裁定 1;`p4_calib.py review` 对 T-diag 运行 = T + gnss_sats.csv,估计与同种子 T 逐字节相同;"
            "伪距、多普勒逐历元消钟后的白等效,TDCP 取自报值(A62 ⑦);按 dof 合并 [逐种子最小, 最大]):")
        add("")
        add("| 序列 | 臂 | 种子 | 伪距 白等效 | 多普勒 白等效 | TDCP 自报 |")
        add("|---|---|---|---|---|---|")
        for k in sorted(wh["cells"]):
            c = wh["cells"][k]
            add("| %s | %s | %s | %s | %s | %s |" % (c["seq"], c["arm"], ",".join(map(str, c["seeds"])), *(
                "%s [%s, %s]" % (f3(c[x]["pooled"]), f3(c[x]["min"]), f3(c[x]["max"])) for x in ("psr", "dopp", "tdcp"))))
        add("")
    add("## 4 各类观测的贡献(%s 对消融臂与 L-PVT,全局与里程计两种输出)" % rep["params"]["t_arm"])
    add("")
    add("| 序列 | 比较 | 口径 | %s 胜 / n | 成对差中位 (m,%s − 对方) | 比值中位 | p(Holm) | 判定 |" % (rep["params"]["t_arm"], rep["params"]["t_arm"]))
    add("|---|---|---|---|---|---|---|---|")
    for seq, sq in rep["sequences"].items():
        for metric, _, _ in VIEWS:
            for v in (sq.get("contributions_by_metric") or {}).get(metric) or []:
                add("| %s | %s | %s | %d / %d | %s | %s | %s | %s |" % (seq, v["name"], VIEW_ZH[metric], v["wins"], v["n"],
                                                                   f3(v["median_diff_m"], "%+.3f"), f3(v["median_ratio"]),
                                                                   f3(v["p_holm"], "%.3g"), verdict_zh(v["verdict"])))
    add("")
    add("负的成对差 = %s 好于去掉该类的臂,即该类有贡献;TDCP 的贡献同时看 §3 的 TDCP 接受率、NIS 与覆盖。" % rep["params"]["t_arm"])
    add("")
    add("## 5 一致性标志(DESIGN §5 D3/D4、TEST_PLAN §3;报告项,不影响退出码)")
    add("")
    if rep["flags"]:
        for f in rep["flags"]:
            add("- %s / %s:%s" % (f["seq"], f["arm"], f["flag"]))
    else:
        add("- 无")
    add("")
    add("## 6 图")
    add("")
    if figs.get("error"):
        add("- 未生成:%s" % figs["error"])
    for k, p in sorted((figs.get("files") or {}).items()):
        add("- %s:`%s`" % (k, p))
    add("")
    add("## 7 口径")
    add("")
    add("- E1/E1′/post30/E4 取自 ate.json(eval_ate.py:A52 整条序列、A54 真值与掩码、A55 同一二进制、TEST_PLAN §4 E1′);本工具不重算 ATE。")
    add("- 轨迹图:每条轨迹用与 ATE 相同的 SE(3) Umeyama(不含尺度)在 E1 时段上各自对齐到真值后画俯视图(实线 = 全局输出,虚线 = 里程计输出);"
        "误差为对齐后的逐位姿位置误差;灰带 = A54 掩掉的真值时段(不计入 ATE)。热图:全局与里程计两版,各含 E1 与 E1′。")
    add("- 进入评测集的第二个条件(%s 相对 U 的表现能用 NIS、接受率、TDCP 贡献解释)是判断,不由本工具给出;本报告给出判断所需的数。" % rep["params"]["t_arm"])
    e5 = rep.get("e5")
    if e5:
        add("")
        add("## 8 E5 ArUco 起终点(GNSS-denial 另表,A56;不混入上面的 ATE)")
        add("")
        if e5.get("refused"):
            add("未附上数值(A2):" + " ".join(e5["refused"]))
        else:
            import eval_aruco
            for line in eval_aruco.markdown_lines(e5["result"], digits=3):
                add(line)
            add("")
            add("来源:`%s`。" % e5.get("source", ""))
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(L) + "\n")


def write_csvs(rep, out):
    with open(os.path.join(out, "dev_report_runs.csv"), "w", newline="") as f:
        w = csv.writer(f)
        hdr = ["seq", "arm", "seed", "ok", "e1_m", "post30_m", "e4_m", "activated", "t_act_after_span_start_s", "sigma_yaw_act_deg",
               "act_path", "tdcp_coverage", "f_p_median", "summary_check"]
        for c in CLASSES:
            hdr += ["%s_cand" % c, "%s_acc_rate" % c, "%s_nis_per_dof" % c, "%s_kappa" % c, "%s_band_frac" % c]
        hdr += ["ev_%s" % k.lower() for k in EVENT_KINDS]
        hdr += ["e1o_m"]   # A61 F13, appended
        hdr += ["e1p_m", "e1op_m", "e1p_lever_x_m", "e1p_lever_y_m", "e1p_lever_z_m", "e1op_lever_x_m", "e1op_lever_y_m",
                "e1op_lever_z_m"]   # E1' (A64), appended
        w.writerow(hdr)
        for r in rep["runs"]:
            g = r["gnss"] or {}
            ep = g.get("epochs") or {}
            act = g.get("first_activate") or {}
            row = [r["seq"], r["arm"], r["seed"], int(r["ok"]), ea._f(r["e1_m"]), ea._f(r["post30_m"]), ea._f(r["e4_m"]),
                   "" if r["activated"] is None else int(bool(r["activated"])), ea._f(g.get("t_act_after_span_start_s"), "%.3f"),
                   ea._f(act.get("sigma_yaw_deg"), "%.3f"), act.get("path", ""), ea._f(g.get("tdcp_coverage"), "%.4f"),
                   ea._f(ep.get("f_p_median"), "%.3g"), g.get("summary_check", "")]
            for c in CLASSES:
                s = (ep.get("classes") or {}).get(c) or {}
                row += [s.get("cand", ""), ea._f(s.get("acc_rate"), "%.4f"), ea._f(s.get("nis_per_dof"), "%.5g"),
                        ea._f(s.get("kappa"), "%.4g"), ea._f(s.get("band_frac"), "%.4f")]
            row += [(g.get("events") or {}).get(k, "") if g else "" for k in EVENT_KINDS]
            row += [ea._f(r.get("e1o_m"))]
            row += [ea._f(r.get("e1p_m")), ea._f(r.get("e1op_m"))]
            for k in ("e1p_lever_m", "e1op_lever_m"):
                row += [ea._f(x) for x in r[k]] if r.get(k) else ["", "", ""]
            w.writerow(row)
    with open(os.path.join(out, "dev_report_cells.csv"), "w", newline="") as f:
        w = csv.writer(f)
        hdr = ["seq", "arm", "n_ok", "n_runs", "e1_median_m", "e1_min_m", "e1_max_m", "vs_u_n", "vs_u_wins", "vs_u_median_diff_m",
               "vs_u_median_ratio", "vs_u_p", "vs_u_p_holm", "vs_u_verdict", "n_activated", "t_act_median_s", "tdcp_coverage_median",
               "f_p_median"]
        for c in CLASSES:
            hdr += ["%s_acc_rate" % c, "%s_nis_per_dof" % c, "%s_nis_x_kappa" % c, "%s_band_frac" % c]
        hdr += ["e1o_median_m", "e1o_min_m", "e1o_max_m", "e1o_n_ok"]   # A61 F13, appended
        for k in ("e1p", "e1op", "e1o"):                                  # E1' (A64) and the paired comparisons, appended
            hdr += ["%s_median_m" % k, "%s_min_m" % k, "%s_max_m" % k, "%s_n_ok" % k] if k != "e1o" else []
            hdr += ["%s_vs_u_wins" % k, "%s_vs_u_n" % k, "%s_vs_u_median_ratio" % k, "%s_vs_u_p_holm" % k]
        w.writerow(hdr)
        for c in rep["cells"]:
            v = c.get("vs_u") or {}
            g = c.get("gnss") or {}
            row = [c["seq"], c["arm"], c["n_ok"], c["n_runs"], ea._f(c["e1"]["median_m"]), ea._f(c["e1"]["min_m"]), ea._f(c["e1"]["max_m"]),
                   v.get("n", ""), v.get("wins", ""), ea._f(v.get("median_diff_m")), ea._f(v.get("median_ratio")),
                   ea._f(v.get("p"), "%.6g"), ea._f(v.get("p_holm"), "%.6g"), v.get("verdict", ""), g.get("n_activated", ""),
                   ea._f(g.get("t_act_median_s"), "%.3f"), ea._f(g.get("tdcp_coverage_median"), "%.4f"), ea._f(g.get("f_p_median"), "%.3g")]
            for k in CLASSES:
                s = (g.get("classes") or {}).get(k) or {}
                row += [ea._f(s.get("acc_rate"), "%.4f"), ea._f(s.get("nis_per_dof"), "%.5g"), ea._f(s.get("nis_per_dof_x_kappa"), "%.5g"),
                        ea._f(s.get("band_frac"), "%.4f")]
            o = c.get("e1o") or {}
            row += [ea._f(o.get("median_m")), ea._f(o.get("min_m")), ea._f(o.get("max_m")), o.get("n_ok", "") if o.get("n_ok") is not None else ""]
            for k in ("e1p", "e1op", "e1o"):
                if k != "e1o":
                    o = c.get(k) or {}
                    row += [ea._f(o.get("median_m")), ea._f(o.get("min_m")), ea._f(o.get("max_m")),
                            o.get("n_ok") if o.get("n_ok") is not None else ""]
                v = (c.get("vs_u_by_metric") or {}).get(k) or {}
                row += [v.get("wins", ""), v.get("n", ""), ea._f(v.get("median_ratio")), ea._f(v.get("p_holm"), "%.6g")]
            w.writerow(row)


# ---------------------------------------------------------------------------------------------------------------
# figures
def _mpl():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import LinearSegmentedColormap
    rc = {"figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE, "axes.edgecolor": AXIS,
          "axes.labelcolor": INK2, "axes.titlecolor": INK, "xtick.color": AXIS, "ytick.color": AXIS, "xtick.labelcolor": INK2,
          "ytick.labelcolor": INK2, "grid.color": GRID, "grid.linewidth": 0.8, "grid.linestyle": "-", "axes.grid": False,
          "font.size": 9, "legend.frameon": False, "axes.spines.top": False, "axes.spines.right": False,
          "lines.solid_capstyle": "round", "lines.solid_joinstyle": "round", "text.color": INK2}
    # the project image has focal's matplotlib 3.1: keys it does not know (titlecolor 3.2, labelcolor 3.4) are skipped
    plt.rcParams.update({k: v for k, v in rc.items() if k in plt.rcParams})
    div = LinearSegmentedColormap.from_list("fl2g_div", [DIV_LO, DIV_MID, DIV_HI])
    seqm = LinearSegmentedColormap.from_list("fl2g_seq", [SEQ_LO, SEQ_HI])
    return plt, div, seqm


def _text_on(rgba):
    """Ink or white on a filled cell, by the cell's relative luminance."""
    def lin(c):
        return c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4
    lum = 0.2126 * lin(rgba[0]) + 0.7152 * lin(rgba[1]) + 0.0722 * lin(rgba[2])
    return "#ffffff" if lum < 0.35 else INK


def _aligned(gt, est, t0, t1, max_diff):
    """est aligned to gt with the ATE's own association and SE(3) Umeyama on [t0, t1] -> (t, p_aligned, err at pairs)."""
    ref = gt[(gt[:, 0] >= t0) & (gt[:, 0] <= t1)]
    ir, ie = ea.evo_associate(ref[:, 0], est[:, 0], max_diff)
    if len(ir) < 3:
        return None
    y, x = ref[ir, 1:4], est[ie, 1:4]
    r, t = ea.umeyama_se3(x, y)
    inside = est[(est[:, 0] >= t0 - max_diff) & (est[:, 0] <= t1 + max_diff)]
    return {"t": inside[:, 0], "p": inside[:, 1:4] @ r.T + t, "t_pair": est[ie, 0], "err": np.linalg.norm(x @ r.T + t - y, axis=1)}


def _gapped(t, v, gap_s=1.0):
    """Copies of (t, v) with a NaN row wherever consecutive stamps are more than gap_s apart (a line is not drawn
    across a stretch the ATE removed)."""
    if len(t) < 2:
        return t, v
    cut = np.where(np.diff(t) > gap_s)[0]
    if cut.size == 0:
        return t, v
    t2 = np.insert(np.asarray(t, dtype=float), cut + 1, np.nan)
    v2 = np.insert(np.asarray(v, dtype=float), cut + 1, np.nan, axis=0)
    return t2, v2


def plot_traj(plt, rep, eval_dir, seq, out_dir, t_arm, u_arm, max_diff):
    runs = {(x["arm"], x["seed"]): x for x in rep["runs"] if x["seq"] == seq and x["ok"]}
    t_seeds = sorted(k[1] for k in runs if k[0] == t_arm and (u_arm, k[1]) in runs)
    if not t_seeds:
        t_seeds = sorted(k[1] for k in runs if k[0] == u_arm)
        if not t_seeds:
            return None
    vals = {k: runs[(t_arm, k)]["e1_m"] for k in t_seeds if (t_arm, k) in runs}
    if vals:
        med = float(np.median(list(vals.values())))
        seed = min(vals, key=lambda k: (abs(vals[k] - med), k))
    else:
        seed = t_seeds[0]
    gt_p = os.path.join(eval_dir, seq, "gt_eval.tum")
    if not os.path.isfile(gt_p):
        return None
    gt = ec.read_tum(gt_p)
    span = next((s["span"] for s in (rep["sequences"][seq].get("e1_spans") or [])), None)
    t0, t1 = (span[0], span[1]) if span else (gt[0, 0], gt[-1, 0])
    # (arm, file, label, line style, width): the global output solid, the odometry output (A61 F13) dashed
    series = [(u_arm, "est_e1.tum", "e1", u_arm, "-", 1.6)]
    for a in (t_arm, "L-PVT"):
        series += [(a, "est_e1.tum", "e1", "%s global" % a, "-", 1.6 if a == t_arm else 1.1),
                   (a, "est_e1o.tum", "e1o", "%s odometry" % a, (0, (4, 2)), 1.6 if a == t_arm else 1.1)]
    fig, (ax, bx) = plt.subplots(1, 2, figsize=(12.0, 5.0), gridspec_kw={"width_ratios": [1.0, 1.45]})
    g_in = gt[(gt[:, 0] >= t0) & (gt[:, 0] <= t1)]
    gt_t, gt_xy = _gapped(g_in[:, 0], g_in[:, 1:3])
    ax.plot(gt_xy[:, 0], gt_xy[:, 1], color=GT_COLOR, lw=2.4, alpha=0.35, label="GT (RTK antenna)", zorder=1)
    masked = [(a, b) for a, b in ((rep["sequences"][seq].get("mask") or {}).get("stretches") or []) if b >= t0 and a <= t1]
    for i, (a, b) in enumerate(masked):
        bx.axvspan(max(a, t0) - t0, min(b, t1) - t0, color=GRID, alpha=0.9, lw=0, zorder=0,
                   label="GT mask (A54, not scored)" if i == 0 else None)
    t_act = None
    drawn = 0
    for a, fname, metric, label, ls, lw in series:
        if (a, seed) not in runs:
            continue
        p = os.path.join(eval_dir, seq, a, "s%d" % seed, fname)
        if not os.path.isfile(p):
            continue
        res = _aligned(gt, ec.read_tum(p), t0, t1, max_diff)
        if res is None:
            continue
        x = runs[(a, seed)]
        v, vp = x.get(metric + "_m"), x.get(metric + "p_m")
        col = ARM_COLOR.get(a, INK2)
        tt, pp = _gapped(res["t"], res["p"][:, :2])
        ax.plot(pp[:, 0], pp[:, 1], color=col, lw=lw, ls=ls, label=label, zorder=2 + drawn)
        te, ee = _gapped(res["t_pair"], res["err"])
        bx.plot(te - t0, ee, color=col, lw=lw, ls=ls, zorder=2 + drawn,
                label="%s  E1 %s · E1' %s m" % (label, f3(v), f3(vp)))
        drawn += 1
        if a == t_arm:
            t_act = (x["gnss"] or {}).get("t_act_after_span_start_s")
    if t_act is not None:
        bx.axvline(t_act, color=MUTED, lw=1.0, zorder=1)
        bx.annotate("%s ACTIVATE" % t_arm, (t_act, 1.0), xycoords=("data", "axes fraction"), xytext=(4, -12),
                    textcoords="offset points", color=INK2, fontsize=8)
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xlabel("E (m)")
    ax.set_ylabel("N (m)")
    ax.grid(True)
    ax.set_title("%s, seed %d: top view (each SE(3)-aligned to GT)" % (seq, seed), fontsize=9, loc="left", color=INK)
    for a in (ax, bx):
        a.tick_params(labelcolor=INK2)
    ax.legend(loc="best", fontsize=7.5)
    bx.set_xlabel("time since E1 span start (s)")
    bx.set_ylabel("position error after alignment (m)")
    bx.set_ylim(bottom=0.0)
    bx.set_xlim(0.0, t1 - t0)
    bx.grid(True, axis="y")
    bx.set_title("per-pose error (E1 protocol): global output solid, odometry output dashed", fontsize=9, loc="left",
                 color=INK)
    bx.legend(loc="upper right", fontsize=7.5)
    fig.tight_layout()
    path = os.path.join(out_dir, "traj_%s.png" % seq)
    fig.savefig(path, dpi=130)
    plt.close(fig)
    return path


def plot_heat(plt, div, rep, out_dir, u_arm, metrics, name, what):
    """sequences x arms (U excluded): per metric panel, the median paired ratio arm/U over seeds (log2 colour), the
    arm's median value and the seeds won; row labels carry U's own medians."""
    seqs = list(rep["sequences"])
    arms = []
    for s_ in seqs:
        for a in rep["sequences"][s_]["arms"]:
            if a != u_arm and a not in arms:
                arms.append(a)
    if not seqs or not arms:
        return None
    arm_order = list(ec.load_arms())
    arms.sort(key=lambda a: arm_order.index(a) if a in arm_order else 99)
    cells = {(c["seq"], c["arm"]): c for c in rep["cells"]}
    fig, axes = plt.subplots(1, len(metrics), figsize=(1.32 * len(arms) * len(metrics) + 3.4, 0.72 * len(seqs) + 1.9),
                             sharey=True, squeeze=False)
    im = None
    for pi, (metric, label) in enumerate(metrics):
        ax = axes[0][pi]
        z = np.full((len(seqs), len(arms)), np.nan)
        ann = [["—"] * len(arms) for _ in seqs]
        for i, s_ in enumerate(seqs):
            for c in (rep["sequences"][s_].get("vs_u_by_metric") or {}).get(metric) or []:
                if c["a"] in arms and c["median_ratio"] is not None:
                    j = arms.index(c["a"])
                    z[i, j] = math.log2(c["median_ratio"])
                    cell = cells.get((s_, c["a"])) or {}
                    med = cell.get(metric, {}).get("median_m")
                    g = cell.get("gnss") or {}
                    if g.get("n") and not g.get("n_activated"):       # never activated: the run is its host, = U
                        ann[i][j] = "%s m\n= %s (not activated)" % (f3(med), u_arm)
                    else:
                        ann[i][j] = "%s m\n×%.2f  %d/%d" % (f3(med), c["median_ratio"], c["wins"], c["n"])
        im = ax.imshow(np.clip(z, -1.0, 1.0), cmap=div, vmin=-1.0, vmax=1.0, aspect="auto")
        for i in range(len(seqs)):
            for j in range(len(arms)):
                col = INK2 if np.isnan(z[i, j]) else _text_on(im.cmap(im.norm(np.clip(z[i, j], -1.0, 1.0))))
                ax.text(j, i, ann[i][j], ha="center", va="center", fontsize=7.5, color=col)
        ax.set_xticks(range(len(arms)))
        ax.set_xticklabels(arms)
        if pi == 0:
            # row label: the sequence and U's own medians of every panel's metric (the ratios' denominators)
            ax.set_yticks(range(len(seqs)))
            ax.set_yticklabels(["%s\n%s %s m" % (s_, u_arm, " · ".join(
                f3((cells.get((s_, u_arm)) or {}).get(m_, {}).get("median_m")) for m_, _ in metrics)) for s_ in seqs])
        ax.tick_params(length=0, labelcolor=INK2)
        for sp in ax.spines.values():
            sp.set_visible(False)
        ax.set_xticks(np.arange(-0.5, len(arms), 1), minor=True)
        ax.set_yticks(np.arange(-0.5, len(seqs), 1), minor=True)
        ax.grid(which="minor", color=SURFACE, linewidth=2.0)
        ax.tick_params(which="minor", length=0)
        ax.set_title(label, fontsize=9, loc="left", color=INK)
    cb = fig.colorbar(im, ax=axes[0].tolist(), fraction=0.025, pad=0.02, ticks=[-1, -0.5, 0, 0.5, 1])
    cb.ax.set_yticklabels(["×0.5", "×0.71", "×1", "×1.41", "×2"])
    cb.outline.set_visible(False)
    cb.set_label("ratio to %s (log; blue = better)" % u_arm, color=INK2)
    fig.suptitle("%s: median over seeds (m), median ratio to %s, seeds won / pairs" % (what, u_arm), x=0.01, ha="left",
                 fontsize=9.5, color=INK)
    path = os.path.join(out_dir, name)
    fig.savefig(path, dpi=130, bbox_inches="tight")
    plt.close(fig)
    return path


def plot_heat_nis(plt, div, seqm, rep, out_dir):
    """(sequence, GNSS arm) x class: pooled NIS/dof × κ_AR (the white-equivalent upper bound; = reported for κ = 1) and
    the acceptance rate. The reported values themselves are in the tables."""
    rows = [c for c in rep["cells"] if c.get("gnss") and c["gnss"]["classes"]]
    if not rows:
        return None
    cls = [k for k in CLASSES if any(k in c["gnss"]["classes"] for c in rows)]
    z = np.full((len(rows), len(cls)), np.nan)
    acc = np.full((len(rows), len(cls)), np.nan)
    kappas = {k: set() for k in cls}
    for i, c in enumerate(rows):
        for j, k in enumerate(cls):
            s = c["gnss"]["classes"].get(k)
            if not s:
                continue
            x = s["nis_per_dof_x_kappa"] if s["nis_per_dof_x_kappa"] is not None else s["nis_per_dof"]
            if x is not None and x > 0:
                z[i, j] = math.log10(x)
            if s["acc_rate"] is not None:
                acc[i, j] = s["acc_rate"]
            kappas[k].add(round(s["kappa"], 2) if s["kappa"] else None)
    col_labels = []
    for k in cls:
        ks = sorted(x for x in kappas[k] if x is not None)
        col_labels.append(k if not ks or ks == [1.0] else "%s\n(κ %s)" % (k, "/".join("%g" % x for x in ks)))
    labels = ["%s · %s" % (c["seq"], c["arm"]) for c in rows]
    fig, (ax, bx) = plt.subplots(1, 2, figsize=(1.9 * len(cls) + 4.4, 0.36 * len(rows) + 1.9), sharey=True)
    im = ax.imshow(np.clip(z, -1.0, 1.0), cmap=div, vmin=-1.0, vmax=1.0, aspect="auto")
    jm = bx.imshow(acc, cmap=seqm, vmin=0.0, vmax=1.0, aspect="auto")
    for i in range(len(rows)):
        for j in range(len(cls)):
            if not np.isnan(z[i, j]):
                ax.text(j, i, "%.3g" % (10 ** z[i, j]), ha="center", va="center", fontsize=7.5,
                        color=_text_on(im.cmap(im.norm(np.clip(z[i, j], -1.0, 1.0)))))
            if not np.isnan(acc[i, j]):
                bx.text(j, i, "%.0f%%" % (100 * acc[i, j]), ha="center", va="center", fontsize=7.5,
                        color=_text_on(jm.cmap(jm.norm(acc[i, j]))))
    for a, title in ((ax, "mean NIS/dof × κ_AR (white-equivalent upper bound; 1 = consistent)"), (bx, "acceptance rate")):
        a.set_xticks(range(len(cls)))
        a.set_xticklabels(col_labels)
        a.tick_params(length=0, labelcolor=INK2)
        for sp in a.spines.values():
            sp.set_visible(False)
        a.set_xticks(np.arange(-0.5, len(cls), 1), minor=True)
        a.set_yticks(np.arange(-0.5, len(rows), 1), minor=True)
        a.grid(which="minor", color=SURFACE, linewidth=2.0)
        a.tick_params(which="minor", length=0)
        a.set_title(title, fontsize=9, loc="left", color=INK)
    ax.set_yticks(range(len(rows)))
    ax.set_yticklabels(labels)
    cb = fig.colorbar(im, ax=ax, fraction=0.05, pad=0.02, ticks=[-1, -0.5, 0, 0.5, 1])
    cb.ax.set_yticklabels(["0.1", "0.32", "1", "3.2", "10"])
    cb.outline.set_visible(False)
    cb2 = fig.colorbar(jm, ax=bx, fraction=0.05, pad=0.02, ticks=[0, 0.5, 1])
    cb2.ax.set_yticklabels(["0%", "50%", "100%"])
    cb2.outline.set_visible(False)
    fig.tight_layout()
    path = os.path.join(out_dir, "heat_nis.png")
    fig.savefig(path, dpi=130)
    plt.close(fig)
    return path


def make_figures(rep, eval_dir, out, args):
    figs = {"files": {}}
    if args.no_plots:
        figs["error"] = "--no-plots"
        return figs
    try:
        plt, div, seqm = _mpl()
    except Exception as e:  # noqa: BLE001 - matplotlib missing or broken in the environment
        figs["error"] = "matplotlib unavailable (%s)" % e
        return figs
    fd = os.path.join(out, "fig")
    os.makedirs(fd, exist_ok=True)
    max_diff = (rep.get("metric") or {}).get("t_max_diff_s") or ec.T_MAX_DIFF_S
    for seq in rep["sequences"]:
        p = plot_traj(plt, rep, eval_dir, seq, fd, args.t_arm, args.u_arm, max_diff)
        if p:
            figs["files"]["traj_%s" % seq] = os.path.relpath(p, out)
    for name, metrics, what in (("heat_global.png", (("e1", "E1 (protocol: IMU vs RTK antenna)"), ("e1p", "E1' (GT antenna lever arm fitted)")),
                                 "Global output (traj_N; U: traj_W)"),
                                ("heat_odom.png", (("e1o", "E1 (protocol: IMU vs RTK antenna)"), ("e1op", "E1' (GT antenna lever arm fitted)")),
                                 "Odometry output (traj_N_odom, A61 F13; U: traj_W)")):
        p = plot_heat(plt, div, rep, fd, args.u_arm, metrics, name, what)
        if p:
            figs["files"][name[:-4]] = os.path.relpath(p, out)
    p = plot_heat_nis(plt, div, seqm, rep, fd)
    if p:
        figs["files"]["heat_nis"] = os.path.relpath(p, out)
    return figs


# ---------------------------------------------------------------------------------------------------------------
def build_parser():
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0], formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--ate", required=True, help="ate.json written by eval_ate.py")
    p.add_argument("--out", default="", help="output directory (default <dir of ate.json>/dev_report)")
    p.add_argument("--u-arm", default="U", help="the host reference arm")
    p.add_argument("--t-arm", default="T", help="the full tight-coupling arm (contributions are measured against it)")
    p.add_argument("--alpha", type=float, default=0.05, help="family-wise level of the Holm-corrected Wilcoxon tests")
    p.add_argument("--win-frac", type=float, default=0.8, help="fraction of seeds that must be won (TEST_PLAN §4: 8/10)")
    p.add_argument("--no-plots", action="store_true", help="tables only")
    p.add_argument("--white-review", action="append", default=[],
                   help="tools/calib/p4_calib.py review JSON of T-diag runs (repeatable): exact white-equivalent NIS table")
    p.add_argument("--aruco", default="", help="aruco.json from eval_aruco.py (TEST_PLAN E5). Appended as its own section; "
                   "not mixed into the ATE tables. An evaluation sequence's numbers are attached only with "
                   "--evaluation-phase (A2)")
    p.add_argument("--evaluation-phase", action="store_true", help="A2: required to report evaluation-set sequences")
    p.add_argument("--root", default=None, help="project root the run directories are relative to (default: ate.json's)")
    return p


def main(argv=None):
    args = build_parser().parse_args(argv)
    try:
        rep, eval_dir = build(args)
    except (OSError, ValueError, KeyError) as e:
        print("dev_report: BLOCKED: %s" % e, file=sys.stderr)
        return 3
    if args.aruco:
        import eval_aruco
        guarded = eval_aruco.guard_for_report(ec.load_json(args.aruco), args.evaluation_phase)
        guarded["source"] = args.aruco
        rep["e5"] = guarded
    out = args.out or os.path.join(eval_dir, "dev_report")
    os.makedirs(out, exist_ok=True)
    rep["figures"] = make_figures(rep, eval_dir, out, args)
    ec.atomic_write_json(os.path.join(out, "dev_report.json"), rep)
    write_csvs(rep, out)
    write_markdown(rep, os.path.join(out, "dev_report.md"), rep["figures"])
    sc = rep["self_checks"]
    print("dev_report: %d runs, %d cells, %d sequences -> %s" % (len(rep["runs"]), len(rep["cells"]), len(rep["sequences"]), out))
    print("dev_report: instrument self-checks %s; %d consistency flag(s); figures: %s" % (
        "all pass" if sc["instrument_all_pass"] else "NOT all pass", len(rep["flags"]),
        rep["figures"].get("error") or ", ".join(sorted(rep["figures"]["files"])) or "none"))
    for seq, why in rep["blocked"].items():
        print("dev_report: %s not reported: %s" % (seq, why))
    if not rep["sequences"]:
        return 3                        # nothing reportable (refused by A2 or blocked in eval_ate)
    if not sc["instrument_all_pass"]:
        return 4
    return 3 if rep["blocked"] else 0


if __name__ == "__main__":
    sys.exit(main())
