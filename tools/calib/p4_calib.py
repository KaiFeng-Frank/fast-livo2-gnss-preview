# SPDX-License-Identifier: BSD-3-Clause
"""P4 closed-form calibration of the M3DGR `gnss:` tree (TEST_PLAN §3, AMENDMENTS A58). numpy only; development set only.

  p4_calib.py noise    --run <T-obs run> --extract <p4_extract dir> [--run … --extract …] [--model <json>] --out <json>
  p4_calib.py gtres    --seq <seq> --run <T-obs run> --extract <dir> [--run-model <json>] --model <json> --root <root> --out <json>
  p4_calib.py gtspp    --seq <seq> --gtres <p4_gtres dir> [...] --model <json> --out <json>
  p4_calib.py clockfit --noise <noise json>
  p4_calib.py pvt      --seq <seq> --extract <dir> --calib <A18 yaml> [...] --root <project root> --out <json>
  p4_calib.py frame    --run <seq>=<U run> [...] --root <project root> [--lever-fixed X Y Z] [--alpha heading|vertical] --out <json>
  p4_calib.py review   --run <feedback run> [...] [--extract <dir>|- ...] [--model <json>] --out <json>

Model JSON (`--model`): the evaluated values of the configuration, {"psr": {sigma0_m, cn0_ref, k_std, tau_s}, "dopp":
{sigma0_mps, tau_s}, "scale": {psr, dopp, tdcp}, "masks": {cn0_psr, cn0_dopp, cn0_cp, max_psr_std_m, max_dopp_std_hz}};
missing keys keep the pre-calibration values the OBSERVE runs were made with.

Innovations come from the OBSERVE arm (T-obs: nothing committed, masks open). They carry the uncorrected receiver clock
and the registration frozen at ACTIVATE, so every class is reduced per epoch by a weighted LS that eliminates what the
filter carries as states — pseudorange: antenna position (3), receiver clock, one ISB per further system; Doppler:
antenna velocity (3), clock drift; TDCP: displacement over the pair (3), Δclk — with Baarda data snooping at the filter's
α = 1e-3 (w = 3.29). What is left is the measurement noise seen through each row's redundancy number r_i; the variance
factor of a model is Σe²/R / Σdof (the a-posteriori σ̂0² of the adjustment, i.e. the mean NIS/dof of those residuals).
Variance models are evaluated on their marginal (per-sample) variance: κ_AR (DESIGN §4.7) is the time-correlation
allowance for the sample mean and is not part of a single sample's variance (AMENDMENTS A48, A1-1).
"""
import argparse
import csv
import itertools
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "eval"))

# rows that reach L4 (the filter's NIS is over these after its own w/NIS rejections); earlier layers decided the others
L4_REASONS = {"none", "w", "nis", "majority", "below_min", "step"}
W_THR = 3.2905267314918945          # two-sided α = 1e-3 (DESIGN §10 L4)
DEG = math.pi / 180.0
MIN_EL_SIN = math.sin(5.0 * DEG)    # NoiseModel guard (quality.cpp)
TAIL_K = 4.0                        # tail statistic |w| > 4 (task: |ν|/σ > 4)
CN0_EDGES = np.arange(8.0, 46.0, 2.0)


# ------------------------------------------------------------------------------------------------ small helpers
def _jsonable(o):
    if hasattr(o, "item"):
        return o.item()
    if hasattr(o, "tolist"):
        return o.tolist()
    return str(o)


def read_csv(path, numeric=None):
    """CSV -> dict of columns (float arrays where every value parses, else lists of str)."""
    with open(path, newline="") as f:
        rd = csv.reader(f)
        head = next(rd)
        cols = [[] for _ in head]
        for row in rd:
            if len(row) != len(head):
                continue
            for i, v in enumerate(row):
                cols[i].append(v)
    out = {}
    for h, c in zip(head, cols):
        try:
            out[h] = np.array([float(v) for v in c])
        except ValueError:
            out[h] = c
    return out


def kappa(tau, dt):
    if not (tau > 0 and dt > 0):
        return 1.0
    r = math.exp(-dt / tau)
    return (1 + r) / (1 - r)


def nnls_small(A, b, w=None):
    """Exact NNLS for a handful of unknowns: every support set, LS on it, the best feasible one."""
    A = np.asarray(A, float)
    b = np.asarray(b, float)
    if w is not None:
        sw = np.sqrt(np.asarray(w, float))
        A, b = A * sw[:, None], b * sw
    n = A.shape[1]
    best = (np.zeros(n), float(b @ b))
    for k in range(1, n + 1):
        for sup in itertools.combinations(range(n), k):
            x_s, *_ = np.linalg.lstsq(A[:, sup], b, rcond=None)
            if np.all(x_s >= 0):
                x = np.zeros(n)
                x[list(sup)] = x_s
                r = b - A @ x
                c = float(r @ r)
                if c < best[1] - 1e-15 * max(1.0, best[1]):
                    best = (x, c)
    return best[0]


def los_enu(el_deg, az_deg):
    el, az = np.asarray(el_deg) * DEG, np.asarray(az_deg) * DEG
    return np.stack([np.cos(el) * np.sin(az), np.cos(el) * np.cos(az), np.sin(el)], axis=-1)


def el_term_psr(el_deg):
    s = np.maximum(np.sin(np.asarray(el_deg) * DEG), MIN_EL_SIN)
    return 1.0 + 1.0 / (s * s)


def el_factor_dopp(el_deg):
    return 0.5 * el_term_psr(el_deg)


def sys_of(sat):
    return sat[0]


def is_bds_geo(sat):
    if sat[0] != "C":
        return False
    p = int(sat[1:])
    return p <= 5 or p >= 59


# ------------------------------------------------------------------------------------------------ models
# The configuration a model is evaluated in (DESIGN §4.7 / quality.cpp); `old` = what the OBSERVE run used.
OLD = {"psr": {"sigma0_m": 0.3, "cn0_ref": 45.0, "k_std": 1.0, "tau_s": 10.0, "sys": {"G": 1.0, "E": 1.0, "C": 1.0, "R": 1.5, "J": 1.0},
               "geo": 2.0},
       "dopp": {"sigma0_mps": 0.05, "tau_s": 0.2},
       "scale": {"psr": 1.0, "dopp": 1.0, "tdcp": 1.0}}


def psr_rcv(m, el, cn0, psr_std):
    p = m["psr"]
    return p["sigma0_m"] ** 2 * el_term_psr(el) * 10.0 ** (np.maximum(0.0, p["cn0_ref"] - cn0) / 10.0) + (p["k_std"] * psr_std) ** 2


def psr_ssys(m, sats):
    p = m["psr"]
    return np.array([p["geo"] if is_bds_geo(s) else p["sys"].get(sys_of(s), 1.0) for s in sats])


def dopp_rcv(m, el, lam, dopp_std):
    s0 = np.maximum(lam * dopp_std, m["dopp"]["sigma0_mps"])
    return s0 * s0 * el_factor_dopp(el)


# ------------------------------------------------------------------------------------------------ data loading
def load_meas(extract_dir):
    d = read_csv(os.path.join(extract_dir, "meas_rows.csv"))
    key = {}
    for i in range(len(d["sat"])):
        key[(int(d["week"][i]), int(round(d["tow"][i] * 1000.0)), d["sat"][i])] = i
    return d, key


def load_run(run, extract_dir):
    """T-obs / T-diag run -> rows of every class with the receiver's own numbers joined in (TRACKING epochs only)."""
    ep = read_csv(os.path.join(run, "gnss_epochs.csv"))
    sa = read_csv(os.path.join(run, "gnss_sats.csv"))
    phase = {}
    epoch_info = {}
    for i in range(len(ep["phase"])):
        k = (int(ep["week"][i]), int(round(ep["tow"][i] * 1000.0)))
        phase[k] = ep["phase"][i]
        epoch_info[k] = (ep["t_est"][i], ep["clk_b"][i], ep["clk_d"][i], ep["t_k"][i])
    meas, mkey = load_meas(extract_dir) if extract_dir else (None, None)
    n = len(sa["sat"])
    rec = {"k": [], "t": [], "sat": [], "el": [], "az": [], "cn0": [], "psr_std": [], "dopp_std": [], "cp_std": [], "lam": [],
           "nu_p": [], "s_p": [], "rs_p": [], "nu_d": [], "s_d": [], "rs_d": [], "nu_c": [], "s_c": [], "rs_c": [], "clk_d": [], "clk_b": []}
    miss = 0
    for i in range(n):
        k = (int(sa["week"][i]), int(round(sa["tow"][i] * 1000.0)))
        if phase.get(k) != "TRACKING":
            continue
        if not (sa["sigma_psr"][i] > 0 or sa["sigma_dopp"][i] > 0 or sa["sigma_tdcp"][i] > 0):
            continue
        sat = sa["sat"][i]
        if mkey is not None:
            j = mkey.get((k[0], k[1], sat))
            if j is None:
                miss += 1
                continue
            rec["psr_std"].append(meas["psr_std_m"][j])
            rec["dopp_std"].append(meas["dopp_std_hz"][j])
            rec["cp_std"].append(meas["cp_std"][j])
            rec["lam"].append(299792458.0 / meas["freq_hz"][j])
        else:
            for c in ("psr_std", "dopp_std", "cp_std", "lam"):
                rec[c].append(float("nan"))
        rec["k"].append(k)
        rec["t"].append(epoch_info[k][0])
        rec["clk_b"].append(epoch_info[k][1])
        rec["clk_d"].append(epoch_info[k][2])
        rec["sat"].append(sat)
        for c, s in (("el", "el_deg"), ("az", "az_deg"), ("cn0", "cn0")):
            rec[c].append(sa[s][i])
        for cl, a, b, r in (("p", "innov_psr", "sigma_psr", "reason_psr"), ("d", "innov_dopp", "sigma_dopp", "reason_dopp"),
                            ("c", "innov_tdcp", "sigma_tdcp", "reason_tdcp")):
            rec["nu_" + cl].append(sa[a][i])
            rec["s_" + cl].append(sa[b][i])
            rec["rs_" + cl].append(sa[r][i])
    out = {c: (np.array(v) if c not in ("k", "sat", "rs_p", "rs_d", "rs_c") else v) for c, v in rec.items()}
    for cl in ("p", "d", "c"):
        out["l4_" + cl] = np.array([r in L4_REASONS for r in rec["rs_" + cl]], bool)
    out["n_unjoined"] = miss
    out["epochs"] = ep
    return out


# ------------------------------------------------------------------------------------------------ per-epoch LS
def epoch_ls(nu, R, A, s=1.0, snoop=True):
    """Weighted LS ν ≈ A x with R·s; Baarda snooping at W_THR. -> (e, r, active, w, dof)."""
    m = len(nu)
    act = np.ones(m, bool)
    e = np.zeros(m)
    r = np.zeros(m)
    w = np.zeros(m)
    dof = 0
    while True:
        idx = np.where(act)[0]
        Aa = A[idx]
        keep = np.any(np.abs(Aa) > 0, axis=0)
        Aa = Aa[:, keep]
        Wa = 1.0 / (s * R[idx])
        rank = np.linalg.matrix_rank(Aa * np.sqrt(Wa)[:, None]) if len(idx) else 0
        dof = len(idx) - rank
        if dof < 1:
            return e, r, act, w, 0
        N = (Aa * Wa[:, None]).T @ Aa
        Ninv = np.linalg.pinv(N)
        x = Ninv @ (Aa.T @ (Wa * nu[idx]))
        ea = nu[idx] - Aa @ x
        h = np.einsum("ij,jk,ik->i", Aa, Ninv, Aa) * Wa
        ra = np.clip(1.0 - h, 0.0, 1.0)
        wa = np.where(ra > 1e-6, ea * np.sqrt(Wa) / np.sqrt(np.maximum(ra, 1e-12)), 0.0)
        e[:] = 0.0
        r[:] = 0.0
        w[:] = 0.0
        e[idx], r[idx], w[idx] = ea, ra, wa
        if not snoop or dof < 2:
            return e, r, act, w, dof
        j = int(np.argmax(np.abs(wa)))
        if abs(wa[j]) <= W_THR:
            return e, r, act, w, dof
        act[idx[j]] = False


def design(cls, rows_sat, el, az):
    u = los_enu(el, az)
    if cls == "p":
        systems = []
        for s in rows_sat:
            g = "G" if sys_of(s) in ("G", "J") else sys_of(s)
            if g not in systems:
                systems.append(g)
        ref = "G" if "G" in systems else systems[0]
        extra = [g for g in systems if g != ref]
        A = np.zeros((len(rows_sat), 4 + len(extra)))
        A[:, :3] = -u
        A[:, 3] = 1.0
        for i, s in enumerate(rows_sat):
            g = "G" if sys_of(s) in ("G", "J") else sys_of(s)
            if g in extra:
                A[i, 4 + extra.index(g)] = 1.0
        return A
    A = np.zeros((len(rows_sat), 4))
    A[:, :3] = -u
    A[:, 3] = 1.0
    return A


def reduce_class(D, cls, R_model, s, sel=None, snoop=True, drift_only=False):
    """Per-epoch LS of one class over the selected rows -> dict of per-row e, r, w, active and per-epoch x0 (common term)."""
    nu = D["nu_" + cls]
    have = (D["s_" + cls] > 0) & np.isfinite(nu) & D["l4_" + cls]
    if sel is not None:
        have &= sel
    idx_all = np.where(have)[0]
    by_ep = {}
    for i in idx_all:
        by_ep.setdefault(D["k"][i], []).append(i)
    n = len(nu)
    E = np.zeros(n)
    Rr = np.zeros(n)
    Wt = np.zeros(n)
    act = np.zeros(n, bool)
    used = np.zeros(n, bool)
    dof_tot = 0
    common = []
    for k in sorted(by_ep):
        ii = np.array(by_ep[k])
        A = design(cls, [D["sat"][i] for i in ii], D["el"][ii], D["az"][ii])
        if drift_only:
            A = A[:, 3:4]
        if len(ii) < A.shape[1] + 1:
            continue
        e, r, a, w, dof = epoch_ls(nu[ii], R_model[ii], A, s=s, snoop=snoop)
        if dof < 1:
            continue
        E[ii], Rr[ii], Wt[ii], act[ii] = e, r, w, a
        used[ii] = True
        dof_tot += dof
        if drift_only or cls in ("p", "d"):
            aa = a
            Wa = 1.0 / (s * R_model[ii][aa])
            Aa = A[aa]
            N = (Aa * Wa[:, None]).T @ Aa
            x = np.linalg.pinv(N) @ (Aa.T @ (Wa * nu[ii][aa]))
            col = 0 if drift_only else 3
            var = np.linalg.pinv(N)[col, col]
            common.append((D["t"][ii[0]], x[col], var, D["clk_d"][ii[0]], D["clk_b"][ii[0]], D["ri"][ii[0]] if "ri" in D else 0))
    return {"e": E, "r": Rr, "w": Wt, "active": act & used, "used": used, "dof": dof_tot, "common": np.array(common)}


def variance_factor(red, R_model):
    a = red["active"]
    if red["dof"] < 1:
        return float("nan")
    return float(np.sum(red["e"][a] ** 2 / R_model[a]) / red["dof"])


def fit_factor(D, cls, R_model, sel=None, iters=6):
    """Iterate snooping and the pooled factor to a fixed point (the filter's L4 + NIS over accepted rows)."""
    s = 1.0
    red = reduce_class(D, cls, R_model, s, sel=sel, snoop=False)
    ok = red["used"] & (red["r"] > 0.05)
    # robust start (median of χ²₁): a mean-based start is inflated by gross errors and then shields them from the snooping
    s = float(np.median(red["e"][ok] ** 2 / (red["r"][ok] * R_model[ok])) / 0.45493642311957283)
    for _ in range(iters):
        red = reduce_class(D, cls, R_model, s, sel=sel, snoop=True)
        s_new = variance_factor(red, R_model)
        if abs(s_new / s - 1.0) < 1e-4:
            s = s_new
            break
        s = s_new
    return s, red


def binned(x, q, edges, act=None):
    """Mean of q per bin of x (q = e²/(r·R) → the per-bin variance factor), counts."""
    out = []
    for lo, hi in zip(edges[:-1], edges[1:]):
        m = (x >= lo) & (x < hi)
        if act is not None:
            m &= act
        if m.sum() >= 20:
            out.append({"lo": float(lo), "hi": float(hi), "n": int(m.sum()), "factor": float(np.mean(q[m]))})
    return out


def tail_bins(x, w, edges, used):
    out = []
    for lo, hi in zip(edges[:-1], edges[1:]):
        m = (x >= lo) & (x < hi) & used
        if m.sum() >= 20:
            out.append({"lo": float(lo), "hi": float(hi), "n": int(m.sum()), "tail": float(np.mean(np.abs(w[m]) > TAIL_K)),
                        "reject": float(np.mean(np.abs(w[m]) > W_THR))})
    return out


def knee(bins, key="tail"):
    """Break point of a hinge fit of log10(tail + 1e-4) vs the bin centre (flat above, linear below), count-weighted:
    the C/N0 (or std) where the tail stops rising."""
    if len(bins) < 4:
        return None
    x = np.array([(b["lo"] + b["hi"]) / 2 for b in bins])
    # Jeffreys estimate (k + ½)/(n + 1): no log(0), and a bin with few rows cannot pin the knee
    y = np.log10(np.array([(b[key] * b["n"] + 0.5) / (b["n"] + 1.0) for b in bins]))
    wt = np.array([b["n"] for b in bins], float)
    best = None
    for c in np.arange(x.min(), x.max() + 1e-9, 0.5):
        A = np.stack([np.ones_like(x), np.minimum(x - c, 0.0)], axis=1)
        sw = np.sqrt(wt)
        p, *_ = np.linalg.lstsq(A * sw[:, None], y * sw, rcond=None)
        sse = float(np.sum(wt * (y - A @ p) ** 2))
        if best is None or sse < best[0]:
            best = (sse, float(c), p.tolist())
    return {"knee": best[1], "level": best[2][0], "slope_per_unit": best[2][1]}


def changepoint(bins, key="tail", lower_side_worse=True):
    """Tail-rate change point: the bin edge c splitting the bins into two binomial rates (below / at-or-above c) with the
    largest likelihood, accepted when the likelihood ratio against one common rate passes χ²₁(0.999) = 10.83 and the rate
    on the `worse` side is the higher one. -> {"edge", "rate_below", "rate_above", "lr", "accepted"} or None."""
    if len(bins) < 2:
        return None
    k = np.array([round(b[key] * b["n"]) for b in bins], float)
    n = np.array([b["n"] for b in bins], float)

    def ll(kk, nn):
        if nn <= 0:
            return 0.0
        p = kk / nn
        return (kk * math.log(p) if kk > 0 else 0.0) + ((nn - kk) * math.log(1 - p) if nn - kk > 0 else 0.0)

    l0 = ll(k.sum(), n.sum())
    best = None
    for i in range(1, len(bins)):
        kb, nb, ka, na = k[:i].sum(), n[:i].sum(), k[i:].sum(), n[i:].sum()
        lr = 2.0 * (ll(kb, nb) + ll(ka, na) - l0)
        rb, ra = kb / nb, ka / na
        worse_ok = (rb > ra) if lower_side_worse else (ra > rb)
        if worse_ok and (best is None or lr > best["lr"]):
            best = {"edge": bins[i]["lo"], "rate_below": rb, "rate_above": ra, "lr": lr, "n_below": int(nb), "n_above": int(na)}
    if best:
        best["accepted"] = bool(best["lr"] >= 10.83)
    return best


def acf_series(D, red, cls, dt, lags):
    """Pooled per-satellite autocorrelation of the standardised residuals at lag k·dt (±0.3 dt)."""
    a = red["active"] & (red["r"] > 0.05)
    z = np.zeros_like(red["e"])
    Rm = red["Rm"]
    z[a] = red["e"][a] / np.sqrt(red["r"][a] * Rm[a])
    by_sat = {}
    for i in np.where(a)[0]:
        by_sat.setdefault(D["sat"][i], []).append(i)
    res = []
    for L in lags:
        num = s1 = s2 = 0.0
        npair = 0
        for s, ii in by_sat.items():
            ii = np.array(ii)
            t = D["t"][ii]
            o = np.argsort(t)
            ii, t = ii[o], t[o]
            j = np.searchsorted(t, t + L * dt - 0.3 * dt)
            ok = (j < len(t))
            jj = np.minimum(j, len(t) - 1)
            ok &= np.abs(t[jj] - t - L * dt) <= 0.3 * dt
            if not ok.any():
                continue
            x0, x1 = z[ii[ok]], z[ii[jj[ok]]]
            num += float(x0 @ x1)
            s1 += float(x0 @ x0)
            s2 += float(x1 @ x1)
            npair += int(ok.sum())
        res.append({"lag": L, "lag_s": L * dt, "rho": num / math.sqrt(s1 * s2) if npair else float("nan"), "pairs": npair})
    return res


def fit_tau(acf, dt):
    """AR(1) fit ρ(k) = exp(−k dt/τ) by LS on log ρ over the leading lags with ρ above its noise (2/√pairs), plus the
    κ-matching τ from the integrated autocorrelation (κ = 1 + 2Σρ(k))."""
    ks, ys = [], []
    for a in acf:
        if not (a["rho"] > 2.0 / math.sqrt(max(a["pairs"], 1))):
            break
        ks.append(a["lag_s"])
        ys.append(math.log(a["rho"]))
    tau_fit = None
    if ks:
        ks, ys = np.array(ks), np.array(ys)
        slope = float(np.sum(ks * ys) / np.sum(ks * ks))          # through the origin: ρ(0) = 1
        tau_fit = -1.0 / slope if slope < 0 else float("inf")
    integ = 1.0
    for a in acf:
        if not (a["rho"] > 2.0 / math.sqrt(max(a["pairs"], 1))):
            break
        integ += 2.0 * a["rho"]
    # Sokal's automatic window: the smallest lag window W with W >= 5·τ_int(W) (τ_int in samples = (κ_W − 1)/2 ... here the
    # window sum runs over every lag, sign included, up to W)
    kw, sokal = 1.0, None
    for a in acf:
        kw += 2.0 * (a["rho"] if math.isfinite(a["rho"]) else 0.0)
        if a["lag"] >= 5.0 * max((kw - 1.0) / 2.0, 0.0):
            sokal = {"window_lags": a["lag"], "kappa": kw}
            break
    tau_integ = dt / math.log((integ + 1.0) / (integ - 1.0)) if integ > 1.0 else 0.0
    rho1 = acf[0]["rho"] if acf else float("nan")
    tau_lag1 = -dt / math.log(rho1) if 0 < rho1 < 1 else 0.0
    # e-folding correlation time: the lag where ρ crosses 1/e (linear interpolation); a lag-1 correlation already below 1/e
    # (sub-sample correlation) -> the exponential through ρ(dt)
    efold = None
    prev = (0.0, 1.0)
    for a in acf:
        if not math.isfinite(a["rho"]):
            continue
        if a["rho"] <= math.exp(-1.0):
            if prev[0] == 0.0:
                efold = tau_lag1
            else:
                l0, r0 = prev
                efold = l0 + (r0 - math.exp(-1.0)) / (r0 - a["rho"]) * (a["lag_s"] - l0)
            break
        prev = (a["lag_s"], a["rho"])
    out = {"tau_efold_s": efold, "tau_ar1_fit_s": tau_fit, "tau_lag1_s": tau_lag1, "kappa_integrated": integ,
           "tau_from_integrated_s": tau_integ, "n_lags_used": len(ks) if hasattr(ks, "__len__") else 0}
    if sokal:
        k = sokal["kappa"]
        out["sokal"] = {"window_lags": sokal["window_lags"], "kappa": k,
                        "tau_s": dt / math.log((k + 1.0) / (k - 1.0)) if k > 1.0 else 0.0}
    return out


def allan_freq(t, y, tau0, max_frac=0.2):
    """Overlapping Allan variance of frequency samples y (m/s) at ~uniform tau0; gaps split the series."""
    t = np.asarray(t)
    y = np.asarray(y)
    o = np.argsort(t)
    t, y = t[o], y[o]
    seg = np.split(np.arange(len(t)), np.where(np.diff(t) > 2.5 * tau0)[0] + 1)
    T = t[-1] - t[0]
    out = []
    m = 1
    while m * tau0 <= max_frac * T:
        num = 0.0
        cnt = 0
        for s in seg:
            ys = y[s]
            if len(ys) < 2 * m + 1:
                continue
            c = np.concatenate([[0.0], np.cumsum(ys)])
            avg = (c[m:] - c[:-m]) / m
            d = avg[m:] - avg[:-m]
            num += float(d @ d)
            cnt += len(d)
        if cnt >= 10:
            out.append({"tau_s": m * tau0, "avar": num / (2.0 * cnt), "terms": cnt})
        m = m * 2 if m < 8 else int(round(m * 1.5))
    return out


# ------------------------------------------------------------------------------------------------ noise
def model_from(args):
    m = json.loads(json.dumps(OLD))
    if args.model:
        with open(args.model) as f:
            upd = json.load(f)
        for k, v in upd.items():
            if isinstance(v, dict) and k in m:
                m[k].update(v)
            else:
                m[k] = v
    return m


def components(D, m_old):
    """Per-row split of the logged R (OBSERVE run's model) into the receiver term (κ applied) and the rest."""
    k_p = kappa(m_old["psr"]["tau_s"], 1.0)
    k_d = kappa(m_old["dopp"]["tau_s"], 0.1)
    ss = psr_ssys(m_old, D["sat"])
    Rp = D["s_p"] ** 2
    rcv_p = ss ** 2 * psr_rcv(m_old, D["el"], D["cn0"], D["psr_std"])
    other_p = Rp / m_old["scale"]["psr"] - rcv_p * k_p
    Rd = D["s_d"] ** 2
    rcv_d = dopp_rcv(m_old, D["el"], D["lam"], D["dopp_std"])
    other_d = Rd / m_old["scale"]["dopp"] - rcv_d * k_d
    return {"ssys": ss, "other_p": other_p, "other_d": other_d, "k_p": k_p, "k_d": k_d, "rcv_p_old": rcv_p, "rcv_d_old": rcv_d,
            "scale_tdcp_run": m_old["scale"]["tdcp"]}


def marginal_R(D, C, m, cls):
    """Per-sample variance of a model (κ left out, scale left out) on the rows' own numbers."""
    if cls == "p":
        return C["ssys"] ** 2 * psr_rcv(m, D["el"], D["cn0"], D["psr_std"]) + np.maximum(C["other_p"], 0.0)
    if cls == "d":
        return dopp_rcv(m, D["el"], D["lam"], D["dopp_std"]) + np.maximum(C["other_d"], 0.0)
    return D["s_c"] ** 2 / C["scale_tdcp_run"]           # the run's own TDCP model, unscaled (factor = scale to use)


def vce(y, basis, forms, bins_x):
    """Variance components: E[e_i²/r_i] = Σ_k a_k basis_k,i (NNLS, weights 1/model², three reweightings) for each candidate
    form (a subset of the basis). Reports the coefficients, the Gaussian log-likelihood of the standardised residuals and the
    spread of the per-bin factor (the binned diagnosis of the fitted form) over each binning variable."""
    A = np.stack(basis, axis=1)
    out = {}
    for name, cols in forms.items():
        xs = np.zeros(A.shape[1])
        sub = nnls_small(A[:, cols], y)
        for _ in range(3):
            mod = np.maximum(A[:, cols] @ sub, 1e-9)
            sub = nnls_small(A[:, cols], y, 1.0 / mod ** 2)
        xs[cols] = sub
        mod = np.maximum(A @ xs, 1e-12)
        z = y / mod
        rec = {"coef": xs.tolist(), "loglik_gauss": float(-0.5 * np.sum(np.log(mod) + z)), "mean_z": float(np.mean(z))}
        for bname, (x, edges) in bins_x.items():
            b = binned(x, z, edges)
            f = np.array([v["factor"] for v in b]) if b else np.array([np.nan])
            rec["bins_" + bname] = [(v["lo"], v["n"], round(v["factor"], 3)) for v in b]
            rec["spread_" + bname] = [float(np.nanmin(f)), float(np.nanmax(f))]
        out[name] = rec
    return out


EL_EDGES = np.array([10, 15, 20, 30, 45, 60, 90.0])
PSR_STD_EDGES = np.array([0, 0.3, 0.7, 1.5, 3.0, 6.0, 12.0, 24.0, 48.0, 1e9])
DOPP_STD_EDGES = np.array([0, 0.3, 0.7, 1.5, 3.0, 6.0, 12.0, 24.0, 1e9])


def vce_psr(D, C, red, sel):
    """Pseudorange: basis f1 = ssys²·el_term·10^(−CN0/10) (↔ σ0²·10^(cn0_ref/10) below the knee), f2 = ssys²·psr_std²
    (↔ k_std²), f3 = the modelled ephemeris/atmosphere/bridge/time-offset variance (↔ its multiplier = noise.scale.psr)."""
    a = red["active"] & sel & (red["r"] > 0.05)
    y = red["e"][a] ** 2 / red["r"][a]
    basis = [C["ssys"][a] ** 2 * el_term_psr(D["el"][a]) * 10.0 ** (-D["cn0"][a] / 10.0), C["ssys"][a] ** 2 * D["psr_std"][a] ** 2,
             np.maximum(C["other_p"][a], 0.0)]
    forms = {"cn0_only": [0, 2], "std_only": [1, 2], "cn0_std": [0, 1, 2], "cn0_std_no_other": [0, 1]}
    return vce(y, basis, forms, {"cn0": (D["cn0"][a], CN0_EDGES), "el": (D["el"][a], EL_EDGES), "std": (D["psr_std"][a], PSR_STD_EDGES)})


def vce_dopp(D, C, red, sel):
    """Doppler: basis g1 = (λ·dopp_std)²·el_factor (↔ noise.scale.dopp), g2 = el_factor (↔ scale·σ_D0², the floor),
    g3 = the modelled bridge/clock/lever/time-offset/registration variance."""
    a = red["active"] & sel & (red["r"] > 0.05)
    y = red["e"][a] ** 2 / red["r"][a]
    ef = el_factor_dopp(D["el"][a])
    basis = [(D["lam"][a] * D["dopp_std"][a]) ** 2 * ef, ef, np.maximum(C["other_d"][a], 0.0)]
    forms = {"std_only": [0, 2], "std_floor": [0, 1, 2]}
    return vce(y, basis, forms, {"cn0": (D["cn0"][a], CN0_EDGES), "el": (D["el"][a], EL_EDGES), "std": (D["dopp_std"][a], DOPP_STD_EDGES)})


def fit_dopp_floor(D, C, red, sel):
    """σ_D0 of the configured form scale·max(λ·dopp_std, σ_D0)²·el_factor (+ the small modelled terms): the Gaussian ML over
    the per-epoch LS residuals (E[e²/r] = model), σ_D0 on a 1 mm/s grid, the scale in closed form for each σ_D0."""
    a = red["active"] & sel & (red["r"] > 0.05)
    y = red["e"][a] ** 2 / red["r"][a]
    x = D["lam"][a] * D["dopp_std"][a]
    ef = el_factor_dopp(D["el"][a])
    oth = np.maximum(C["other_d"][a], 0.0)
    best = None
    for s0 in np.arange(0.02, 0.301, 0.001):
        base = np.maximum(x, s0) ** 2 * ef + oth
        sc = float(np.mean(y / base))                   # ML scale of a Gaussian variance model with one multiplier
        ll = float(-0.5 * np.sum(np.log(sc * base) + y / (sc * base)))
        if best is None or ll > best[0]:
            best = (ll, float(s0), sc)
    base = np.maximum(x, best[1]) ** 2 * ef + oth
    z = y / (best[2] * base)
    return {"sigma0_mps": best[1], "scale": best[2], "loglik": best[0],
            "bins_cn0": [(v["lo"], v["n"], round(v["factor"], 3)) for v in binned(D["cn0"][a], z, CN0_EDGES)],
            "bins_std": [(v["lo"], v["n"], round(v["factor"], 3)) for v in binned(D["dopp_std"][a], z, DOPP_STD_EDGES)]}


def cmd_noise(args):
    m_old = json.loads(json.dumps(OLD))
    m = model_from(args)
    report = {"model": m, "runs": []}
    Ds = []
    for run, ex in zip(args.run, args.extract):
        D = load_run(run, ex)
        C = components(D, m_old)
        Ds.append((run, D, C))
        report["runs"].append({"run": run, "rows": int(len(D["sat"])), "unjoined": D["n_unjoined"],
                               "other_p_min_rel": float(np.nanmin(np.where(D["s_p"] > 0, C["other_p"] / np.maximum(D["s_p"] ** 2, 1e-12), np.inf))),
                               "other_d_min_rel": float(np.nanmin(np.where(D["s_d"] > 0, C["other_d"] / np.maximum(D["s_d"] ** 2, 1e-12), np.inf)))})
    # pool the runs (same receiver, same model): concatenate per-row arrays; epoch keys stay distinct per run
    def cat(key):
        return np.concatenate([D[key] for _, D, _ in Ds])
    D = {c: cat(c) for c in ("t", "el", "az", "cn0", "psr_std", "dopp_std", "cp_std", "lam", "nu_p", "s_p", "nu_d", "s_d", "nu_c", "s_c", "clk_d", "clk_b")}
    for cl in ("p", "d", "c"):
        D["l4_" + cl] = np.concatenate([Dd["l4_" + cl] for _, Dd, _ in Ds])
    D["sat"] = sum((Dd["sat"] for _, Dd, _ in Ds), [])
    D["k"] = sum(([(ri,) + k for k in Dd["k"]] for ri, (_, Dd, _) in enumerate(Ds)), [])
    # time axis distinct per run for the autocorrelation (runs are hours apart anyway; add a large offset per run)
    D["t"] = np.concatenate([Dd["t"] + 1e7 * ri for ri, (_, Dd, _) in enumerate(Ds)])
    D["ri"] = np.concatenate([np.full(len(Dd["sat"]), ri) for ri, (_, Dd, _) in enumerate(Ds)])
    C = {c: np.concatenate([Cc[c] for _, _, Cc in Ds]) for c in ("ssys", "other_p", "other_d", "rcv_p_old", "rcv_d_old")}
    C["k_p"], C["k_d"], C["scale_tdcp_run"] = Ds[0][2]["k_p"], Ds[0][2]["k_d"], Ds[0][2]["scale_tdcp_run"]
    sel_mask = {}
    masks = m.get("masks", {})
    sel_mask["p"] = (D["cn0"] >= masks.get("cn0_psr", 0.0)) & (D["psr_std"] <= masks.get("max_psr_std_m", 1e9))
    sel_mask["d"] = (D["cn0"] >= masks.get("cn0_dopp", 0.0)) & (D["dopp_std"] <= masks.get("max_dopp_std_hz", 1e9))
    sel_mask["c"] = (D["cn0"] >= masks.get("cn0_cp", 0.0))
    for cls, dt, lags in (("p", 1.0, range(1, 121)), ("d", 0.1, range(1, 31)), ("c", 0.5, range(1, 2))):
        Rm = marginal_R(D, C, m, cls)
        s, red = fit_factor(D, cls, Rm, sel=sel_mask[cls])
        red["Rm"] = Rm
        q = np.where(red["r"] > 0.05, red["e"] ** 2 / np.maximum(red["r"] * Rm, 1e-12), np.nan)
        a = red["active"] & (red["r"] > 0.05)
        rep = {"variance_factor": s, "dof": red["dof"], "rows_used": int(red["used"].sum()), "rows_active": int(red["active"].sum()),
               "reject_frac": float(1.0 - red["active"].sum() / max(red["used"].sum(), 1)),
               "bins_cn0": binned(D["cn0"], q / s, CN0_EDGES, a),
               "bins_el": binned(D["el"], q / s, EL_EDGES, a)}
        # tails on ALL rows (masks open, no selection): standardised with the fitted factor, no snooping
        red_all = reduce_class(D, cls, Rm, s, sel=None, snoop=False)
        rep["tails_cn0"] = tail_bins(D["cn0"], red_all["w"], CN0_EDGES, red_all["used"] & (red_all["r"] > 0.05))
        rep["tails_cn0_knee"] = knee(rep["tails_cn0"])
        if cls == "p":
            std_edges = PSR_STD_EDGES
            rep["bins_std"] = binned(D["psr_std"], q / s, std_edges, a)
            rep["tails_std"] = tail_bins(D["psr_std"], red_all["w"], std_edges, red_all["used"] & (red_all["r"] > 0.05))
            rep["vce"] = vce_psr(D, C, red, sel_mask["p"])
        if cls == "d":
            std_edges = DOPP_STD_EDGES
            rep["bins_std"] = binned(D["dopp_std"], q / s, std_edges, a)
            rep["tails_std"] = tail_bins(D["dopp_std"], red_all["w"], std_edges, red_all["used"] & (red_all["r"] > 0.05))
            rep["vce"] = vce_dopp(D, C, red, sel_mask["d"])
            rep["floor_ml"] = fit_dopp_floor(D, C, red, sel_mask["d"])
        if cls in ("p", "d"):
            acf = acf_series(D, red, cls, dt, list(lags))
            rep["acf"] = acf
            rep["tau"] = fit_tau(acf, dt)
        report[cls] = rep
        if cls == "d":
            # clock drift from the Doppler common term (drift-only reduction: lowest noise), per run -> Allan variance
            red1 = reduce_class(D, "d", Rm * s, 1.0, sel=sel_mask["d"], snoop=True, drift_only=True)
            cm = red1["common"]
            allan = []
            for ri in range(len(Ds)):
                c = cm[cm[:, 5] == ri]
                if len(c) < 100:
                    continue
                t = c[:, 0] - 1e7 * ri
                d_true = c[:, 3] + c[:, 1]
                tau0 = float(np.median(np.diff(np.sort(t))))
                av = allan_freq(t, d_true, tau0)
                allan.append({"run": Ds[ri][0], "tau0_s": tau0, "meas_var_mean": float(np.mean(c[:, 2])), "avar": av,
                              "drift_first_last": [float(np.median(d_true[:50])), float(np.median(d_true[-50:]))],
                              "span_s": float(t.max() - t.min())})
            report["clock"] = allan
    with open(args.out, "w") as f:
        json.dump(report, f, indent=1, default=_jsonable)
    print("wrote %s" % args.out)


# ------------------------------------------------------------------------------------------------ GT-referenced residuals
def load_model(path):
    m = json.loads(json.dumps(OLD))
    if path:
        with open(path) as f:
            upd = json.load(f)
        for k, v in upd.items():
            if isinstance(v, dict) and k in m:
                m[k].update(v)
            else:
                m[k] = v
    return m


def read_anchor(path):
    a = {"R_EN": []}
    with open(path) as f:
        for line in f:
            t = line.strip()
            if t.startswith("r0_ecef:"):
                a["r0"] = np.array(json.loads(t.split(":", 1)[1]))
            elif t.startswith("- ["):
                a["R_EN"].append(json.loads(t[2:]))
    a["R_EN"] = np.array(a["R_EN"])
    return a


def cmd_gtres(args):
    """Pseudorange/Doppler residuals of an OBSERVE run against the RTK truth instead of a per-epoch position fit: the
    frozen registration's error at each epoch, δa = a^N(filter) − a^N(truth), is removed from every pseudorange innovation
    (ν − uᵀδa) and only the receiver clock (+ ISB) is eliminated per epoch — usable with the 1–3 satellites per epoch of the
    occluded sequences, where a position fit has no redundancy. Truth: the audited GT (A54 time base and mask) of the
    C-RTK2HP antenna, moved to the 9Ps antenna with the fitted lever arms (frame subcommand)."""
    import gtbase
    import evalcommon as ec
    from fl2g import geodesy
    _, seqreg = ec.load_sequences()
    m_run = load_model(args.run_model)
    m = load_model(args.model)
    D = load_run(args.run, args.extract)
    C = components(D, m_run)
    gt, prov, mask = gtbase.load_gt(args.root, args.seq, seqreg[args.seq])
    o = json.load(open(os.path.join(args.root, "audit", args.seq, "gt", "gt_audit.json")))["provenance"]["origin"]
    anc = read_anchor(os.path.join(args.run, "anchor.yaml"))
    Mg = geodesy.enu_matrix(o["lat_deg"] * DEG, o["lon_deg"] * DEG)          # rows: E, N, U of the GT origin
    r0g = geodesy.pos2ecef(o["lat_deg"] * DEG, o["lon_deg"] * DEG, o["h_m"])
    ecef = r0g + gt[:, 1:4] @ Mg
    gtN = (ecef - anc["r0"]) @ anc["R_EN"]                                     # N frame (anchor ENU)
    trN = read_tum_np(os.path.join(args.run, "traj_N.tum"))
    antN = read_tum_np(os.path.join(args.run, "traj_ant_N.tum"))
    dl = np.array(args.lever_9ps) - np.array(args.lever_gt)
    tg = gt[:, 0]
    t = D["t"]
    j = np.clip(np.searchsorted(tg, t), 1, len(tg) - 1)
    okg = ((tg[j] - tg[j - 1]) <= 0.5) & (t >= tg[0]) & (t <= tg[-1]) & np.array([not mask.inside(x) for x in t])
    f = (t - tg[j - 1]) / np.maximum(tg[j] - tg[j - 1], 1e-9)
    g = gtN[j - 1] * (1 - f)[:, None] + gtN[j] * f[:, None]
    ja = np.clip(np.searchsorted(antN[:, 0], t), 1, len(antN) - 1)
    fa = (t - antN[ja - 1, 0]) / np.maximum(antN[ja, 0] - antN[ja - 1, 0], 1e-9)
    okg &= (t >= antN[0, 0]) & (t <= antN[-1, 0])
    a_f = antN[ja - 1, 1:4] * (1 - fa)[:, None] + antN[ja, 1:4] * fa[:, None]
    jr = np.clip(np.searchsorted(trN[:, 0], t), 0, len(trN) - 1)
    Rni = np.array([quat_to_R(q) for q in trN[jr, 4:8]])
    a_true = g + np.einsum("nij,j->ni", Rni, dl)
    da = a_f - a_true
    u = los_enu(D["el"], D["az"])
    D["nu_p"] = np.where(okg, D["nu_p"] - np.sum(u * da, axis=1), np.nan)
    rep = {"seq": args.seq, "run": args.run, "rows": int(len(t)), "gt_ok": int(okg.sum()),
           "frame_error_m": {"median_h": float(np.nanmedian(np.linalg.norm(da[okg, :2], axis=1))) if okg.any() else None,
                             "median_v": float(np.nanmedian(np.abs(da[okg, 2]))) if okg.any() else None}}
    mk = m.get("masks", {})
    for cls in ("p", "d"):
        Rm = marginal_R(D, C, m, cls) * m["scale"]["psr" if cls == "p" else "dopp"]
        sel = np.isfinite(D["nu_" + cls])
        if cls == "p":
            sel &= (D["cn0"] >= mk.get("cn0_psr", 0.0)) & (D["psr_std"] <= mk.get("max_psr_std_m", 1e9))
        else:
            sel &= (D["cn0"] >= mk.get("cn0_dopp", 0.0)) & (D["dopp_std"] <= mk.get("max_dopp_std_hz", 1e9))
        if cls == "p":
            D2 = dict(D)
            D2["l4_p"] = D["l4_p"] & sel
            red = reduce_class_clock_only(D2, Rm)
        else:
            red = reduce_class(D, "d", Rm, 1.0, sel=sel, snoop=False, drift_only=True)
        okr = red["used"] & (red["r"] > 0.05)
        w = np.where(okr, red["e"] / np.sqrt(np.maximum(red["r"] * Rm, 1e-12)), 0.0)
        q = np.where(okr, w ** 2, np.nan)
        # the factor over rows that a 3.29 w-test would keep (the filter's L4), and every row's tail
        keep = okr & (np.abs(w) <= W_THR)
        rec = {"factor_all": float(np.nansum(q[okr]) / max(red["dof"], 1)), "rows": int(okr.sum()), "dof": red["dof"],
               "factor_kept": float(np.mean(q[keep])) if keep.any() else None,
               "bins_cn0": binned(D["cn0"], q, CN0_EDGES, okr), "tails_cn0": tail_bins(D["cn0"], w, CN0_EDGES, okr)}
        rec["tails_cn0_knee"] = knee(rec["tails_cn0"])
        edges = PSR_STD_EDGES if cls == "p" else DOPP_STD_EDGES
        rec["tails_std"] = tail_bins(D["psr_std" if cls == "p" else "dopp_std"], w, edges, okr)
        rec["bins_std"] = binned(D["psr_std" if cls == "p" else "dopp_std"], q, edges, okr)
        if cls == "p":
            red["Rm"] = Rm
            red["active"] = keep.copy() if args.acf_kept else okr.copy()
            acf = acf_series(D, red, "p", 1.0, list(range(1, 121)))
            rec["acf"] = [a for a in acf if a["lag"] in (1, 2, 3, 5, 8, 12, 20, 30, 45, 60, 90, 120)]
            rec["tau"] = fit_tau(acf, 1.0)
        rep[cls] = rec
    with open(args.out, "w") as fo:
        json.dump(rep, fo, indent=1, default=_jsonable)
    print("wrote %s" % args.out)


def cmd_gtspp(args):
    """Residuals at the RTK truth (p4_gtres.py) of one or more sequences, under a model: per-epoch receiver clock (+ ISB)
    elimination for the pseudorange, clock-drift elimination for the Doppler; variance factor, per-bin factors and tails
    (|w| > 4, |w| > 3.29) over C/N0 and the receiver std, the knee of the tail over C/N0."""
    m = load_model(args.model)
    out = {"model": m, "seqs": {}}
    pooled = {"p": [], "d": []}
    for seq, d_ in zip(args.seq, args.gtres):
        g = read_csv(os.path.join(d_, "gt_residuals.csv"))
        n = len(g["sat"])
        sats = g["sat"]
        k = [(round(t, 3),) for t in g["t_gpst"]]
        el, az, cn0 = g["el_deg"], g["az_deg"], g["cn0"]
        ssys = psr_ssys(m, sats)
        other = g["var_ion"] + g["var_trop"] + g["var_eph"]
        Rp = m["scale"]["psr"] * (ssys ** 2 * psr_rcv(m, el, cn0, g["psr_std_m"]) + other)
        lam = 299792458.0 / np.maximum(g["freq_hz"], 1.0)
        Rd = m["scale"]["dopp"] * dopp_rcv(m, el, lam, g["dopp_std_hz"])
        D = {"k": k, "sat": sats, "el": el, "az": az, "t": g["t_est"], "clk_d": np.zeros(n), "clk_b": np.zeros(n),
             "nu_p": np.where(g["psr_epoch"] > 0, g["v_psr"], np.nan), "s_p": np.sqrt(Rp),
             "nu_d": g["v_dopp"], "s_d": np.sqrt(Rd), "l4_p": np.isfinite(g["v_psr"]) & (g["psr_epoch"] > 0),
             "l4_d": np.isfinite(g["v_dopp"])}
        masks = m.get("masks", {})
        D["l4_p"] &= (cn0 >= masks.get("cn0_psr", 0.0)) & (g["psr_std_m"] <= masks.get("max_psr_std_m", 1e9))
        D["l4_d"] &= (cn0 >= masks.get("cn0_dopp", 0.0)) & (g["dopp_std_hz"] <= masks.get("max_dopp_std_hz", 1e9))
        rec = {"rows_psr": int(D["l4_p"].sum()), "rows_dopp": int(D["l4_d"].sum())}
        for cls, Rm in (("p", Rp), ("d", Rd)):
            red = reduce_class_clock_only(D, Rm) if cls == "p" else reduce_class(D, "d", Rm, 1.0, snoop=False, drift_only=True)
            okr = red["used"] & (red["r"] > 0.05)
            w = np.where(okr, red["e"] / np.sqrt(np.maximum(red["r"] * Rm, 1e-12)), 0.0)
            q = np.where(okr, w ** 2, np.nan)
            keep = okr & (np.abs(w) <= W_THR)
            std = g["psr_std_m"] if cls == "p" else g["dopp_std_hz"]
            edges = PSR_STD_EDGES if cls == "p" else DOPP_STD_EDGES
            r = {"factor_all": float(np.nansum(q[okr]) / max(red["dof"], 1)), "factor_kept": float(np.mean(q[keep])) if keep.any() else None,
                 "rows": int(okr.sum()), "dof": red["dof"], "reject_frac": float(1.0 - keep.sum() / max(okr.sum(), 1)),
                 "bins_cn0": binned(cn0, q, CN0_EDGES, okr), "tails_cn0": tail_bins(cn0, w, CN0_EDGES, okr),
                 "bins_std": binned(std, q, edges, okr), "tails_std": tail_bins(std, w, edges, okr)}
            r["tails_cn0_knee"] = knee(r["tails_cn0"])
            r["cn0_change"] = changepoint(r["tails_cn0"])
            rec[cls] = r
            pooled[cls].append((cn0[okr], w[okr], std[okr], red["dof"], np.nansum(q[okr])))
        out["seqs"][seq] = rec
    for cls in ("p", "d"):
        if not pooled[cls]:
            continue
        c = np.concatenate([x[0] for x in pooled[cls]])
        w = np.concatenate([x[1] for x in pooled[cls]])
        sd = np.concatenate([x[2] for x in pooled[cls]])
        used = np.ones(len(c), bool)
        edges = PSR_STD_EDGES if cls == "p" else DOPP_STD_EDGES
        pr = {"factor_all": float(sum(x[4] for x in pooled[cls]) / max(sum(x[3] for x in pooled[cls]), 1)),
              "tails_cn0": tail_bins(c, w, CN0_EDGES, used), "tails_std": tail_bins(sd, w, edges, used),
              "bins_cn0": binned(c, w ** 2, CN0_EDGES, used)}
        pr["tails_cn0_knee"] = knee(pr["tails_cn0"])
        pr["cn0_change"] = changepoint(pr["tails_cn0"])
        pr["cn0_change_reject"] = changepoint(pr["tails_cn0"], key="reject")
        cut = pr["cn0_change"]["edge"] if pr["cn0_change"] and pr["cn0_change"]["accepted"] else -1.0
        above = c >= cut
        pr["std_tails_above_cn0_cut"] = {"cut": cut, "tails": tail_bins(sd, w, edges, above)}
        pr["std_change_above_cn0_cut"] = changepoint(pr["std_tails_above_cn0_cut"]["tails"], lower_side_worse=False)
        pr["std_change_all"] = changepoint(pr["tails_std"], lower_side_worse=False)
        out["pooled_" + cls] = pr
    with open(args.out, "w") as fo:
        json.dump(out, fo, indent=1, default=_jsonable)
    print("wrote %s" % args.out)


def read_tum_np(path):
    rows = []
    with open(path) as f:
        for line in f:
            p = line.split()
            if len(p) == 8:
                rows.append([float(x) for x in p])
    return np.array(rows)


# ------------------------------------------------------------------------------------------------ clock fit
def cmd_clockfit(args):
    rep = json.load(open(args.noise))
    rows = []
    for a in rep["clock"]:
        meas = a["meas_var_mean"] * a["tau0_s"]
        for v in a["avar"]:
            rows.append((v["tau_s"], v["avar"], meas))
    tau = np.array([r[0] for r in rows])
    av = np.array([r[1] for r in rows])
    A = np.stack([1.0 / tau, tau / 3.0], axis=1)
    x = nnls_small(A, av, 1.0 / av ** 2)
    meas = float(np.mean([r[2] for r in rows]))
    ratio = av / (A @ x)
    # minimal envelope: the two-state model A/τ + Bτ/3 (A = S_b, B = S_d) at or above every measured point, the binding
    # pair chosen by the smallest mean log over-bound (the DESIGN §3.1 "保守值" read on the data: a TCXO's thermal frequency
    # ramp makes the long-τ AVAR grow as τ², which no least-squares fit of the two-state model follows)
    cands = []
    n = len(tau)
    for i in range(n):
        cands.append((av[i] * tau[i], 0.0))
        cands.append((0.0, 3.0 * av[i] / tau[i]))
        for j in range(i + 1, n):
            M = np.array([[1.0 / tau[i], tau[i] / 3.0], [1.0 / tau[j], tau[j] / 3.0]])
            if abs(np.linalg.det(M)) < 1e-15:
                continue
            ab = np.linalg.solve(M, np.array([av[i], av[j]]))
            cands.append((ab[0], ab[1]))
    best = None
    for a_, b_ in cands:
        if a_ < 0 or b_ < 0:
            continue
        mod = a_ / tau + b_ * tau / 3.0
        if np.all(mod >= av * (1 - 1e-9)):
            c = float(np.mean(np.log(mod / av)))
            if best is None or c < best[0]:
                best = (c, a_, b_)
    out = {"white_fm_coef": float(x[0]), "S_d": float(x[1]), "meas_white_fm": meas, "S_b": float(max(x[0] - meas, 0.0)),
           "fit_ratio_min": float(ratio.min()), "fit_ratio_max": float(ratio.max()),
           "envelope_factor": float(ratio.max()), "points": [[float(t), float(v), float(r)] for t, v, r in zip(tau, av, ratio)],
           "envelope": {"S_b": float(best[1]), "S_d": float(best[2]), "mean_log_overbound": best[0],
                        "max_overbound": float(np.max((best[1] / tau + best[2] * tau / 3.0) / av))} if best else None}
    print(json.dumps(out, indent=1))


# ------------------------------------------------------------------------------------------------ pvt
def cmd_pvt(args):
    import gtbase
    import evalcommon as ec
    from fl2g import geodesy, timecalib_yaml, gnsstime
    _, seqreg = ec.load_sequences()
    out = {"seqs": []}
    allz = {"h": [], "v": [], "vel": []}
    acf_pool = {"h": [], "v": [], "vel": []}
    for seq, ex, cal in zip(args.seq, args.extract, args.calib):
        gt, prov, mask = gtbase.load_gt(args.root, seq, seqreg[seq])
        gj = json.load(open(os.path.join(args.root, "audit", seq, "gt", "gt_audit.json")))
        o = gj["provenance"]["origin"]
        p = read_csv(os.path.join(ex, "pvt.csv"))
        with open(cal) as f:
            calib = timecalib_yaml.loads(f.read())
        # 1 Hz: first message of each GPST second (A43), valid fixes only
        t_g = p["t_gpst"]
        sec = np.floor(t_g)
        first = np.r_[True, sec[1:] != sec[:-1]]
        ok = first & (p["valid_fix"] > 0) & (p["fix_type"] >= 3)
        te = np.array([timecalib_yaml.evaluate(calib, float(t)) for t in t_g[ok]])
        enu = geodesy.geodetic_to_enu(p["lat"][ok] * DEG, p["lon"][ok] * DEG, p["h_ell"][ok], o["lat_deg"] * DEG, o["lon_deg"] * DEG, o["h_m"])
        # GT at those instants (linear interpolation inside ≤0.5 s gaps, outside masked stretches)
        tg = gt[:, 0]
        j = np.searchsorted(tg, te)
        good = (j > 0) & (j < len(tg))
        j = np.clip(j, 1, len(tg) - 1)
        good &= (tg[j] - tg[j - 1]) <= 0.5
        good &= np.array([not mask.inside(t) for t in te])
        f = (te - tg[j - 1]) / np.maximum(tg[j] - tg[j - 1], 1e-9)
        g = gt[j - 1, 1:4] * (1 - f)[:, None] + gt[j, 1:4] * f[:, None]
        # GT velocity: central difference over ±0.5 s
        def gt_at(tt):
            jj = np.clip(np.searchsorted(tg, tt), 1, len(tg) - 1)
            ff = (tt - tg[jj - 1]) / np.maximum(tg[jj] - tg[jj - 1], 1e-9)
            return gt[jj - 1, 1:4] * (1 - ff)[:, None] + gt[jj, 1:4] * ff[:, None]
        vg = (gt_at(te + 0.5) - gt_at(te - 0.5)) / 1.0
        good &= np.array([not (mask.inside(t - 0.5) or mask.inside(t + 0.5)) for t in te])
        e = enu - g
        e = e[good]
        tt = te[good]
        h_acc, v_acc, vel_acc = p["h_acc"][ok][good], p["v_acc"][ok][good], p["vel_acc"][ok][good]
        vel = np.stack([p["vel_e"][ok], p["vel_n"][ok], -p["vel_d"][ok]], axis=1)[good]
        ev = vel - vg[good]
        e0 = e - np.mean(e, axis=0)          # the constant part is the registration's t, not measurement noise
        zh = e0[:, :2] / h_acc[:, None]
        zv = e0[:, 2] / v_acc
        zvel = ev / (vel_acc[:, None] * math.sqrt(2.0))
        rec = {"seq": seq, "n": int(len(tt)), "mean_err_enu": np.mean(e, axis=0).tolist(), "rms_err_demeaned": np.sqrt(np.mean(e0 ** 2, axis=0)).tolist(),
               "h_acc_median": float(np.median(h_acc)), "v_acc_median": float(np.median(v_acc)), "vel_acc_median": float(np.median(vel_acc)),
               "factor_h": float(np.mean(zh ** 2)), "factor_v": float(np.mean(zv ** 2)), "factor_vel": float(np.mean(zvel ** 2)),
               "factor_vel_h": float(np.mean(zvel[:, :2] ** 2)), "factor_vel_v": float(np.mean(zvel[:, 2] ** 2)),
               "gt_method": prov.get("method")}
        for name, z in (("h", zh), ("v", zv[:, None]), ("vel", zvel)):
            acf = []
            for L in range(1, 61):
                num = s1 = s2 = 0.0
                cnt = 0
                for c in range(z.shape[1]):
                    jj = np.searchsorted(tt, tt + L - 0.3)
                    okk = jj < len(tt)
                    jj = np.minimum(jj, len(tt) - 1)
                    okk &= np.abs(tt[jj] - tt - L) <= 0.3
                    x0, x1 = z[okk, c], z[jj[okk], c]
                    num += float(x0 @ x1)
                    s1 += float(x0 @ x0)
                    s2 += float(x1 @ x1)
                    cnt += int(okk.sum())
                acf.append({"lag": L, "lag_s": float(L), "rho": num / math.sqrt(s1 * s2) if cnt else float("nan"), "pairs": cnt})
            rec["acf_" + name] = [a for a in acf if a["lag"] in (1, 2, 3, 5, 10, 20, 30, 60)]
            rec["tau_" + name] = fit_tau(acf, 1.0)
            allz[name].append(z.ravel() ** 2)
            for a in acf:
                acf_pool[name].append(a)
        out["seqs"].append(rec)
    for name in allz:
        if allz[name]:
            out["pooled_factor_" + name] = float(np.mean(np.concatenate(allz[name])))
    # pooled autocorrelation: pair-count-weighted mean of the per-sequence ρ(k)
    for name, lst in acf_pool.items():
        by = {}
        for a in lst:
            if math.isfinite(a["rho"]) and a["pairs"]:
                n, sw = by.get(a["lag"], (0, 0.0))
                by[a["lag"]] = (n + a["pairs"], sw + a["rho"] * a["pairs"])
        pooled = [{"lag": k, "lag_s": float(k), "rho": sw / n, "pairs": n} for k, (n, sw) in sorted(by.items())]
        out["pooled_acf_" + name] = [a for a in pooled if a["lag"] in (1, 2, 3, 5, 10, 15, 20, 30, 60)]
        out["pooled_tau_" + name] = fit_tau(pooled, 1.0)
    with open(args.out, "w") as f:
        json.dump(out, f, indent=1, default=_jsonable)
    print("wrote %s" % args.out)


# ------------------------------------------------------------------------------------------------ frame q (DESIGN §3.2)
SEG_S = (10.0, 20.0, 50.0, 100.0, 200.0)


def quat_to_R(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def umeyama(src, dst):
    ms, md = src.mean(0), dst.mean(0)
    H = (src - ms).T @ (dst - md)
    U, S, Vt = np.linalg.svd(H)
    D = np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))])
    R = Vt.T @ D @ U.T
    return R, md - R @ ms


def yaw_align(src, dst):
    """Translation + rotation about z (4-DoF) of src onto dst."""
    ms, md = src.mean(0), dst.mean(0)
    a, b = src[:, :2] - ms[:2], dst[:, :2] - md[:2]
    c = float(np.sum(a[:, 0] * b[:, 0] + a[:, 1] * b[:, 1]))
    s = float(np.sum(a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0]))
    th = math.atan2(s, c)
    R = np.array([[math.cos(th), -math.sin(th), 0], [math.sin(th), math.cos(th), 0], [0, 0, 1.0]])
    return R, md - R @ ms


def frame_prep(root, seq, info, run):
    import gtbase
    import evalcommon as ec
    gt, prov, mask = gtbase.load_gt(root, seq, info)
    tr = ec.read_tum(os.path.join(run, "traj_W.tum"))
    tr, _, _ = ec.sanitize_tum(tr)
    tg, te = gt[:, 0], tr[:, 0]
    j = np.searchsorted(tg, te)
    ok = (j > 0) & (j < len(tg))
    j = np.clip(j, 1, len(tg) - 1)
    ok &= (tg[j] - tg[j - 1]) <= 0.5
    ok &= np.array([not mask.inside(t) for t in te])
    f = (te - tg[j - 1]) / np.maximum(tg[j] - tg[j - 1], 1e-9)
    g = gt[j - 1, 1:4] * (1 - f)[:, None] + gt[j, 1:4] * f[:, None]
    Rs = np.array([quat_to_R(row[4:8]) for row in tr[ok]])
    return {"t": te[ok], "p": tr[ok, 1:4], "R": Rs, "g": g[ok], "gt_method": prov.get("method"),
            "masked_s": float(mask.seconds_in(te[0], te[-1])) if hasattr(mask, "seconds_in") and len(te) else 0.0}


def fit_lever(P, l0, iters=12):
    """GT antenna (C-RTK2HP, unknown, audit P6) lever arm in the IMU frame, horizontal components, jointly with the global
    SE(3): g ≈ Rg (p + R l) + tg; l_z kept (a ground vehicle's rotations are about the vertical)."""
    l = np.array(l0, float)
    for _ in range(iters):
        a = P["p"] + np.einsum("nij,j->ni", P["R"], l)
        Rg, tg = umeyama(a, P["g"])
        res = P["g"] - (P["p"] @ Rg.T + tg)
        A = np.einsum("ij,njk->nik", Rg, P["R"])          # Rg R_t
        rhs = (res - A[:, :, 2] * l[2]).reshape(-1)
        M = A[:, :, :2].reshape(-1, 2)
        l[:2] = np.linalg.lstsq(M, rhs, rcond=None)[0]
    a = P["p"] + np.einsum("nij,j->ni", P["R"], l)
    Rg, tg = umeyama(a, P["g"])
    rms = float(np.sqrt(np.mean(np.sum((P["g"] - (a @ Rg.T + tg)) ** 2, axis=1))))
    return l, rms


def robust_var(x):
    return float(np.median(np.asarray(x) ** 2) / 0.45493642311957283)    # median of χ²₁


def turn_steps(Rg, Rs, measure="heading"):
    """|turn| of every step of the trajectory, the increments of the accumulated turn α (the regressor of q_ψr).
    'heading' (A58): the change of the heading of the IMU x axis in the GT frame, which is the vehicle's turn when the IMU frame is
    level (the Avia IMU: z within 2-3 degrees of the vertical). 'vertical' (A78): the rotation about the vertical between consecutive
    orientations, which is what the filter integrates (Δα = Σ|ω·up|dt, gnss_fusion.cpp) and does not depend on how the IMU is
    mounted: for a mount tilted against the vertical (the MID-360 IMU: 31.8 degrees) the heading of the x axis picks up the roll
    (tan 31.8° ≈ 0.62 of it). On a level IMU the two agree to better than 1 % (A78 record). Rs: (n,3,3) orientations IMU → world."""
    if measure == "heading":
        heading = np.array([math.atan2((Rg @ R)[1, 0], (Rg @ R)[0, 0]) for R in Rs])
        return np.abs(np.angle(np.exp(1j * np.diff(heading))))
    if measure == "vertical":
        Rw = np.einsum("ij,njk->nik", Rg, np.asarray(Rs, float))
        dR = Rw[1:] @ np.swapaxes(Rw[:-1], 1, 2)                      # world-frame increment of every step
        th = np.arccos(np.clip((np.trace(dR, axis1=1, axis2=2) - 1.0) / 2.0, -1.0, 1.0))
        k = np.ones_like(th)
        big = th > 1e-9
        k[big] = th[big] / np.sin(th[big])                            # rotation vector = k · (R − Rᵀ)^∨ / 2
        return np.abs(k * (dR[:, 1, 0] - dR[:, 0, 1]) / 2.0)
    raise ValueError("unknown turn measure %r" % (measure,))


def frame_segments(P, lever, turn="heading"):
    a_w = P["p"] + np.einsum("nij,j->ni", P["R"], lever)
    g, te = P["g"], P["t"]
    Rg, tgv = umeyama(a_w, g)
    a_g = a_w @ Rg.T + tgv
    dpsi = turn_steps(Rg, P["R"], turn)
    alpha = np.r_[0.0, np.cumsum(dpsi)]
    brk = np.r_[False, np.diff(te) > 0.5]          # masked stretches / gaps cut the path
    dp = np.linalg.norm(np.diff(g, axis=0), axis=1)
    dp[brk[1:]] = 0.0
    s_cum = np.r_[0.0, np.cumsum(dp)]
    cut = np.cumsum(brk)
    rows = []
    for S in SEG_S:
        for s0 in np.arange(0.0, s_cum[-1] - S, 1.0):
            i0 = int(np.searchsorted(s_cum, s0))
            i1 = int(np.searchsorted(s_cum, s0 + S))
            ia = int(np.searchsorted(s_cum, s0 + 0.1 * S))
            if i1 >= len(s_cum) or cut[i0] != cut[i1] or ia - i0 < 3:
                continue
            R4, t4 = yaw_align(a_g[i0:ia + 1], g[i0:ia + 1])      # DESIGN §3.2: first 10 % of the segment, translation + yaw
            err = g[i1] - (R4 @ a_g[i1] + t4)
            chord = g[i1, :2] - g[i0, :2]
            d = chord if np.linalg.norm(chord) >= 0.3 * S else g[i1, :2] - g[i1 - 1, :2]
            d = d / max(np.linalg.norm(d), 1e-9)
            rows.append((S, float(err[:2] @ d), float(err[:2] @ np.array([-d[1], d[0]])), float(err[2]), float(alpha[i1] - alpha[i0])))
    return np.array(rows), float(s_cum[-1])


def frame_fit(rows):
    """Per segment length: robust variances (median of χ²₁) of the along/lateral/vertical end errors; the lateral set is
    split at the median accumulated turn α of that length (straight vs turning halves), which separates q_ψ (per metre)
    from q_ψr (per radian of turn) — α grows with s, so on the length bins alone the two are collinear."""
    stats, lat_rows = [], []
    for S in SEG_S:
        mm = rows[:, 0] == S
        if mm.sum() < 10:
            continue
        stats.append({"s": S, "n": int(mm.sum()), "along2": robust_var(rows[mm, 1]), "lat2": robust_var(rows[mm, 2]),
                      "vert2": robust_var(rows[mm, 3]), "alpha": float(np.mean(rows[mm, 4]))})
        a = rows[mm, 4]
        med = np.median(a)
        for half in (a <= med, a > med):
            if half.sum() >= 5:
                lat_rows.append((S, robust_var(rows[mm, 2][half]), float(np.mean(a[half]))))
    if len(stats) < 3:
        return stats, None
    s = np.array([x["s"] for x in stats])
    ya, yv = (np.array([x[k] for x in stats]) for k in ("along2", "vert2"))
    lr = np.array(lat_rows)
    sl, yl, al = lr[:, 0], lr[:, 1], lr[:, 2]
    # along: c_a + q_th s ; lat: c_l + q_th s + q_ψ s³/3 + q_ψr ᾱ s²/3 ; vert: c_v + q_tv s + q_φ s³/3 (DESIGN §3.2; the constants
    # c_* take the end-point floor: GT noise, the alignment on the first 10 %, residual antenna offset). Relative weights.
    A = np.vstack([np.stack([np.ones_like(s), np.zeros_like(s), s, np.zeros_like(s), np.zeros_like(s)], axis=1),
                   np.stack([np.zeros_like(sl), np.ones_like(sl), sl, sl ** 3 / 3.0, al * sl ** 2 / 3.0], axis=1)])
    y = np.r_[ya, yl]
    x = nnls_small(A, y, 1.0 / np.maximum(y, 1e-8) ** 2)
    Av = np.stack([np.ones_like(s), s, s ** 3 / 3.0], axis=1)
    xv = nnls_small(Av, yv, 1.0 / np.maximum(yv, 1e-8) ** 2)
    fit = {"c_along": x[0], "c_lat": x[1], "q_th": x[2], "q_yaw": x[3], "q_yaw_per_rad": x[4], "c_vert": xv[0], "q_tv": xv[1], "q_tilt": xv[2]}
    return stats + [{"lat_half": [float(S), float(v), float(a)]} for S, v, a in lat_rows], {k: float(v) for k, v in fit.items()}


def cmd_frame(args):
    import evalcommon as ec
    _, seqreg = ec.load_sequences()
    runs = [spec.split("=", 1) for spec in args.run]
    preps = [(seq, run, frame_prep(args.root, seq, seqreg[seq], run)) for seq, run in runs]
    # pass 1: GT antenna lever arm per sequence (first run of each), the median is used for every trajectory
    lev = {}
    if args.lever_fixed is None:
        for seq, run, P in preps:
            if seq not in lev:
                l, rms = fit_lever(P, args.lever)
                lev[seq] = {"l_I": l.tolist(), "rms_m": rms}
        l_med = np.array(args.lever, float)
        l_med[:2] = np.median(np.array([v["l_I"][:2] for v in lev.values()]), axis=0)
    else:
        # A78: the GT antenna lever of a host whose IMU frame is not level is not fitted here (the fit above takes the first two
        # IMU-frame components to be horizontal); the host's own value, fitted in the level frame by host_gt_lever.py (A77) and
        # kept in tools/eval/lidars.json `gt_lever_imu`, is used for every trajectory of every sequence.
        l_med = np.array(args.lever_fixed, float)
    per_traj = []
    for seq, run, P in preps:
        rows, path = frame_segments(P, l_med, args.alpha)
        stats, fit = frame_fit(rows)
        per_traj.append({"seq": seq, "run": run, "segments": int(len(rows)), "path_m": path, "stats": stats, "fit": fit,
                         "gt_method": P["gt_method"]})
    out = {"lever_arm_I_9ps": list(args.lever), "lever_arm_I_gt_per_seq": lev, "lever_arm_I_gt_used": l_med.tolist(), "trajectories": per_traj}
    if args.lever_fixed is not None:
        out["lever_arm_I_gt_fixed"] = True
    if args.alpha != "heading":
        out["alpha_measure"] = args.alpha
    keys = ("q_th", "q_yaw", "q_yaw_per_rad", "q_tv", "q_tilt")
    fits = [t["fit"] for t in per_traj if t["fit"]]
    if fits:
        out["quantiles"] = {k: {"p50": float(np.percentile([f[k] for f in fits], 50)), "p75": float(np.percentile([f[k] for f in fits], 75)),
                                "max": float(np.max([f[k] for f in fits])), "n": len(fits)} for k in keys}
    with open(args.out, "w") as f:
        json.dump(out, f, indent=1, default=_jsonable)
    print("wrote %s" % args.out)


# ------------------------------------------------------------------------------------------------ review
def white_equivalent(D, m):
    """Per-row marginal (κ-free) variance of the rows a feedback run logged with model m: R − scale·rcv·(κ − 1)."""
    k_p = kappa(m["psr"]["tau_s"], 1.0)
    k_d = kappa(m["dopp"]["tau_s"], 0.1)
    rp = m["scale"]["psr"] * psr_ssys(m, D["sat"]) ** 2 * psr_rcv(m, D["el"], D["cn0"], D["psr_std"])
    rd = m["scale"]["dopp"] * dopp_rcv(m, D["el"], D["lam"], D["dopp_std"])
    return {"p": D["s_p"] ** 2 - rp * (k_p - 1.0), "d": D["s_d"] ** 2 - rd * (k_d - 1.0), "c": D["s_c"] ** 2,
            "kappa_p": k_p, "kappa_d": k_d}


def review_rows(run, extract, m):
    """κ-normalised statistics of a feedback run's accepted rows (reason none): per epoch the filter's prior innovations
    reduced by the common receiver clock (+ ISB) — the state whose prior uncertainty dominates HΣHᵀ — against the marginal
    variance (white-equivalent NIS/dof), and the same with the antenna position/velocity/displacement also eliminated."""
    D = load_run(run, extract)
    Wm = white_equivalent(D, m)
    out = {"kappa_psr": Wm["kappa_p"], "kappa_dopp": Wm["kappa_d"],
           # AMENDMENTS A62: the TDCP ("c") figures take the logged sigma_tdcp² = R as the marginal variance, which holds for
           # independent pairs only. With the A61 F11 phase states R leaves σ²_Φ of the shared epochs to the state, so these
           # figures are not a NIS; the TDCP review figure is the filter's self-reported NIS (A58 ruling 1).
           "tdcp_note": "c: valid only without phase states (A61 F11); use the self-reported TDCP NIS"}
    for cls in ("p", "d", "c"):
        acc = np.array([r == "none" for r in D["rs_" + cls]]) & (D["s_" + cls] > 0)
        D2 = dict(D)
        D2["l4_" + cls] = acc
        res = {}
        for mode in ("common", "full"):
            red = reduce_class(D2, cls, np.maximum(Wm[cls], 1e-12), 1.0, snoop=False, drift_only=(mode == "common" and cls != "p"))
            if mode == "common" and cls == "p":
                red = reduce_class_clock_only(D2, np.maximum(Wm[cls], 1e-12))
            res[mode] = {"nis_per_dof": variance_factor(red, np.maximum(Wm[cls], 1e-12)), "dof": red["dof"], "rows": int(red["used"].sum())}
        out[cls] = res
    return out


def reduce_class_clock_only(D, R_model):
    """Pseudorange rows reduced by the receiver clock and one ISB per further system only (position kept at the filter's)."""
    nu = D["nu_p"]
    have = (D["s_p"] > 0) & np.isfinite(nu) & D["l4_p"]
    by_ep = {}
    for i in np.where(have)[0]:
        by_ep.setdefault(D["k"][i], []).append(i)
    n = len(nu)
    E, Rr, act, used = np.zeros(n), np.zeros(n), np.zeros(n, bool), np.zeros(n, bool)
    dof_tot = 0
    for k in sorted(by_ep):
        ii = np.array(by_ep[k])
        A = design("p", [D["sat"][i] for i in ii], D["el"][ii], D["az"][ii])[:, 3:]
        if len(ii) < A.shape[1] + 1:
            continue
        e, r, a, w, dof = epoch_ls(nu[ii], R_model[ii], A, s=1.0, snoop=False)
        if dof < 1:
            continue
        E[ii], Rr[ii], act[ii], used[ii] = e, r, a, True
        dof_tot += dof
    return {"e": E, "r": Rr, "active": act & used, "used": used, "dof": dof_tot}


def cmd_review(args):
    out = []
    m = model_from(args) if args.model else None
    for ri, run in enumerate(args.run):
        ep = read_csv(os.path.join(run, "gnss_epochs.csv"))
        summ = json.load(open(os.path.join(run, "summary.json"))) if os.path.isfile(os.path.join(run, "summary.json")) else {}
        ev = open(os.path.join(run, "events.log")).read().splitlines() if os.path.isfile(os.path.join(run, "events.log")) else []
        trk = np.array([p in ("TRACKING", "RECOVERY") for p in ep["phase"]])
        rec = {"run": run, "summary": summ, "epochs": int(len(ep["phase"])), "tracking_epochs": int(trk.sum())}
        for k in ("psr", "dopp", "tdcp", "pvt_pos", "pvt_vel"):
            if k + "_nis_prior" not in ep:
                continue
            dof = ep[k + "_dof"]
            nis = ep[k + "_nis_prior"]
            use = trk & (dof > 0)
            cand = ep[k + "_cand"]
            acc = ep[k + "_acc"]
            rec[k] = {"pooled_nis_per_dof": float(nis[use].sum() / max(dof[use].sum(), 1)), "nis_sum": float(nis[use].sum()),
                      "dof_sum": int(dof[use].sum()),
                      "mean_epoch_nis_per_dof": float(np.mean(nis[use] / dof[use])) if use.any() else float("nan"),
                      "epochs": int(use.sum()), "candidates": int(cand[trk].sum()), "accepted": int(acc[trk].sum()),
                      "acceptance": float(acc[trk].sum() / max(cand[trk].sum(), 1))}
            rej = {}
            for c in ep:
                if c.startswith(k + "_rej_"):
                    v = int(np.sum(ep[c][trk]))
                    if v:
                        rej[c[len(k) + 5:]] = v
            rec[k]["rejected"] = rej
        kinds = {}
        for line in ev:
            parts = line.split()
            for p in parts:
                if p.isupper() and len(p) > 3 and p.replace("_", "").isalpha():
                    kinds[p] = kinds.get(p, 0) + 1
                    break
        rec["events"] = kinds
        rec["activate"] = [l for l in ev if "ACTIVATE" in l][:3]
        if m is not None and args.extract and ri < len(args.extract) and args.extract[ri] != "-" and \
                os.path.isfile(os.path.join(run, "gnss_sats.csv")):
            rec["white_equivalent"] = review_rows(run, args.extract[ri], m)
        out.append(rec)
    with open(args.out, "w") as f:
        json.dump(out, f, indent=1, default=_jsonable)
    print("wrote %s" % args.out)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    a = sub.add_parser("noise")
    a.add_argument("--run", action="append", required=True)
    a.add_argument("--extract", action="append", required=True)
    a.add_argument("--model")
    a.add_argument("--out", required=True)
    a = sub.add_parser("gtres")
    a.add_argument("--seq", required=True)
    a.add_argument("--run", required=True)
    a.add_argument("--extract", required=True)
    a.add_argument("--run-model", help="model the run was made with (default: the pre-calibration placeholders)")
    a.add_argument("--model", help="model to evaluate")
    a.add_argument("--root", required=True)
    a.add_argument("--lever-9ps", type=float, nargs=3, default=[-0.31635, 0.10926, 0.4006])
    a.add_argument("--lever-gt", type=float, nargs=3, default=[-0.1763, -0.0141, 0.4006])
    a.add_argument("--acf-kept", action="store_true", help="autocorrelation over the rows a 3.29 w-test keeps")
    a.add_argument("--out", required=True)
    a = sub.add_parser("gtspp")
    a.add_argument("--seq", action="append", required=True)
    a.add_argument("--gtres", action="append", required=True, help="p4_gtres.py output dir per --seq")
    a.add_argument("--model")
    a.add_argument("--out", required=True)
    a = sub.add_parser("clockfit")
    a.add_argument("--noise", required=True)
    a = sub.add_parser("pvt")
    a.add_argument("--seq", action="append", required=True)
    a.add_argument("--extract", action="append", required=True)
    a.add_argument("--calib", action="append", required=True)
    a.add_argument("--root", required=True)
    a.add_argument("--out", required=True)
    a = sub.add_parser("frame")
    a.add_argument("--run", action="append", required=True, help="<seq>=<U run dir>")
    a.add_argument("--root", required=True)
    a.add_argument("--lever", type=float, nargs=3, default=[-0.31635, 0.10926, 0.4006])
    a.add_argument("--lever-fixed", type=float, nargs=3, default=None, metavar=("X", "Y", "Z"),
                   help="GT antenna lever arm in the host IMU frame, held for every trajectory instead of fitted per sequence (A78; "
                        "tools/eval/lidars.json gt_lever_imu). Default: the A58 fit")
    a.add_argument("--alpha", choices=("heading", "vertical"), default="heading",
                   help="turn measure α of a segment (the regressor of q_ψr and the key of the lateral half split): 'heading' = the "
                        "A58 one (heading of the IMU x axis, right for a level IMU), 'vertical' = rotation about the vertical (A78; "
                        "right for any mount)")
    a.add_argument("--out", required=True)
    a = sub.add_parser("review")
    a.add_argument("--run", action="append", required=True)
    a.add_argument("--extract", action="append", help="p4_extract dir per --run ('-' = none): κ-normalised NIS from gnss_sats.csv")
    a.add_argument("--model")
    a.add_argument("--out", required=True)
    args = ap.parse_args(argv)
    for s in (getattr(args, "seq", None) if isinstance(getattr(args, "seq", None), list) else []):
        from fl2g.sequences import seq_class
        if seq_class(s) != "development":
            print("refusing %s: not a development-set sequence (AMENDMENTS A2)" % s)
            return 3
    if getattr(args, "seq", None) and isinstance(args.seq, str):
        from fl2g.sequences import seq_class
        if seq_class(args.seq) != "development":
            print("refusing %s: not a development-set sequence (AMENDMENTS A2)" % args.seq)
            return 3
    {"noise": cmd_noise, "gtres": cmd_gtres, "gtspp": cmd_gtspp, "clockfit": cmd_clockfit, "pvt": cmd_pvt, "frame": cmd_frame, "review": cmd_review}[args.cmd](args)
    return 0


if __name__ == "__main__":
    sys.exit(main())
