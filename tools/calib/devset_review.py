# SPDX-License-Identifier: BSD-3-Clause
"""Development-set consistency review of feedback runs (AMENDMENTS A58 review protocol, A61 ruling 3, A62).
numpy only; development set only (A2): every command refuses a run whose sequence is not a development sequence.

  devset_review.py events   --run <run> [...] --out <json>
  devset_review.py tdcp-acf --run <run> [...] --out <json>
  devset_review.py chain-model [--p-hat 28 --q-hat 5] --case <p,q,rho_o> [...]
  devset_review.py nees     --run <run> --seq <seq> --root <project root> [--extract <p4_extract dir> --calib <A18 yaml>]
                            [--lever-9ps x y z] [--lever-gt x y z] [--segments N] [--series <csv>] --out <json>
  devset_review.py same     --a <run> --b <run> [--u <U run of the same seed>] --out <json>

events   every event kind of events.log counted; ACTIVATE (time after the first traj_W stamp, path, σ_ψ, σ_t, sats),
         FRAME_JUMP / ROLLBACK / REANCHOR / REINIT lines, SLIP by detector bit (D1..D6), TOFF_WARN residuals.
tdcp-acf lag-1 / lag-2 correlation of the TDCP rows of one satellite over consecutive pair-closing epochs (A61 ruling 3②).
         Statistic: the filter's Baarda w of accepted rows (reason none, σ > 0) — the prior innovation standardised by the
         Δclk-projected S of its epoch (update.cpp gateClass). A consistent filter has white innovations: ρ ≈ 0 whatever the
         pairing. Chained pairs (A61 F10) without the phase states (F11) share φ(k1) with opposite signs, which a model of
         independent pairs leaves in the innovations as ρ₁ ≈ −σ_Φ²/(2σ_Φ² + o²); positive ρ₁ means errors correlated over
         more than one pair (carrier multipath, wind-up, rate) that σ_rate/σ_windup model as independent per pair.
         Secondary statistic: ν·sin(el) (the raw innovation on the elevation scale of σ_Φ(el) = σ_Φ0/sin el).
chain-model  the scalar chain that maps (lag-1, NIS) to "which variance is off" (A62): truth vs model phase / per-pair.
nees     the global antenna output (traj_ant_N.tum) against the A54 ground truth moved to the filter's N frame (anchor.yaml)
         and to the receiver's antenna (lever arms in the IMU frame, GT antenna fitted in A58); per axis z² = e²/σ² with
         σ = the registration translation's marginal σ_t logged in gnss_epochs.csv. The antenna's covariance is not written by
         the replay; a^N = M(a^W − o^W) + t, so at the pivot (d = a^N − t = 0) σ_t is exact and elsewhere
         Var(a_i) ∈ [(σ_t,i − σ_rot,i)², (σ_t,i + σ_rot,i)²] (Cauchy–Schwarz on the unlogged cross-covariance), σ_rot,i² =
         Σ_j ([d]×)_ij² σ_θ,j² with σ_θ = (tilt_x, tilt_y, yaw) marginals (tilt–yaw correlation ignored). Reported per time
         segment of the tracking span, with the error mean/RMS. With --extract/--calib the receiver's own PVT (first message
         of each GPST second, A43) is compared with the same ground truth over the same segments; parked windows of the
         host (speed < --still-mps for >= --still-min-s) are reported separately (A61 ruling 5). An offset shared by the
         receiver PVT and the filter would be a datum offset of the ground-truth frame, not a filter property (A62: on the
         development set the receiver PVT and the ground truth differ by metres in directions that change between
         sequences, so the PVT is not used to re-reference the ground truth). --series writes the compared epochs one per row
         (time since the first tracking epoch, error E/N/U, σ_t, σ_rot, parked flag) for figures; the JSON is unchanged.
same     byte identity of traj_W/traj_N/traj_N_odom/traj_ant_N/gnss_epochs/events between two runs; with --u, the traj_W
         lines of each run before its first ACTIVATE step against the U run of the same seed (TEST_PLAN I11, A55).
"""
import argparse
import hashlib
import json
import math
import os
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "eval"))

from p4_calib import read_csv, _jsonable  # noqa: E402
from fl2g.sequences import DEV_SEQUENCES  # noqa: E402,F401  (A2 classes: one source, tools/fl2g/sequences.py; A70 adds Z-Rough-Road01)

EVENT_RE = re.compile(r'^t_k=(\S+) t_est=(\S+) gpst=(\S+) (\w+) magnitude=(\S+)(?: detail="(.*)")?\s*$')
KV_RE = re.compile(r"(\w+)=([^\s]+)")
DET_RE = re.compile(r"detectors=0x([0-9a-fA-F]+)")


def dev_only(path_or_seq):
    """A2: refuse anything that is not a development sequence (the run directories carry the sequence name)."""
    s = str(path_or_seq)
    for e in ("Varying-illu", "Outdoor", "Dark0", "GNSS-denial"):
        if e.lower() in s.lower():
            raise SystemExit("devset_review: %s looks like an evaluation sequence (A2): refused" % s)
    if not any(d.lower() in s.lower() for d in DEV_SEQUENCES):
        raise SystemExit("devset_review: %s is not a development sequence (A2): refused" % s)


def read_events(run):
    p = os.path.join(run, "events.log")
    out = []
    if not os.path.isfile(p):
        return out
    with open(p) as f:
        for line in f:
            m = EVENT_RE.match(line.rstrip("\n"))
            if not m:
                continue
            out.append({"t_k": float(m.group(1)), "t_est": float(m.group(2)), "gpst": m.group(3), "kind": m.group(4),
                        "magnitude": float(m.group(5)), "detail": m.group(6) or ""})
    return out


def first_traj_stamp(run):
    p = os.path.join(run, "traj_W.tum")
    with open(p) as f:
        for line in f:
            if line.strip() and not line.startswith("#"):
                return float(line.split()[0])
    return float("nan")


def read_tum(path):
    a = np.loadtxt(path, ndmin=2)
    return a


def quat_to_R(q):
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


# ------------------------------------------------------------------------------------------------ events
def cmd_events(args):
    out = []
    for run in args.run:
        dev_only(run)
        ev = read_events(run)
        t0 = first_traj_stamp(run)
        kinds = {}
        for e in ev:
            kinds[e["kind"]] = kinds.get(e["kind"], 0) + 1
        rec = {"run": run, "t0_traj_W": t0, "counts": kinds}
        act = [e for e in ev if e["kind"] == "ACTIVATE"]
        rec["activate"] = [dict(t_after_start_s=e["t_k"] - t0, t_k=e["t_k"], magnitude=e["magnitude"],
                                **{k: v for k, v in KV_RE.findall(e["detail"])}) for e in act]
        for k in ("FRAME_JUMP", "ROLLBACK", "REANCHOR", "REINIT", "REINIT_YAW", "CLOCK_JUMP", "LIVO_ALARM", "REPIVOT"):
            rec[k.lower()] = [dict(t_after_start_s=e["t_k"] - t0, magnitude=e["magnitude"], detail=e["detail"])
                              for e in ev if e["kind"] == k]
        det = {"D%d" % i: 0 for i in range(1, 7)}
        slips = [e for e in ev if e["kind"] == "SLIP"]
        for e in slips:
            m = DET_RE.search(e["detail"])
            if m:
                b = int(m.group(1), 16)
                for i in range(6):
                    if b & (1 << i):
                        det["D%d" % (i + 1)] += 1
        rec["slip"] = {"n": len(slips), "by_detector": det}
        tw = [e for e in ev if e["kind"] == "TOFF_WARN"]
        align = [e for e in tw if e["detail"].startswith("ALIGN")]
        res = np.array([e["magnitude"] for e in tw if not e["detail"].startswith("ALIGN")]) * 1e3
        rec["toff_warn"] = {"n": len(tw), "n_align_check": len(align),
                            "align_magnitude_ms": [e["magnitude"] * 1e3 for e in align],
                            "n_residual": int(res.size),
                            "residual_ms": ({"min": float(res.min()), "median": float(np.median(res)), "max": float(res.max())}
                                            if res.size else None)}
        sp = os.path.join(run, "summary.json")
        rec["summary"] = json.load(open(sp)) if os.path.isfile(sp) else None
        out.append(rec)
    with open(args.out, "w") as f:
        json.dump(out, f, indent=1, default=_jsonable)
    print("wrote %s" % args.out)


# ------------------------------------------------------------------------------------------------ TDCP autocorrelation
def tdcp_rows(run):
    """-> (sorted closing-epoch times, {sat: {epoch index: (w, nu, sigma, el)}}, gaps) over accepted TDCP rows."""
    sa = read_csv(os.path.join(run, "gnss_sats.csv"))
    n = len(sa["sat"])
    t = sa["week"] * 604800.0 + sa["tow"]
    have = (sa["sigma_tdcp"] > 0) & np.isfinite(sa["innov_tdcp"])
    ep_t = np.unique(np.round(t[have], 3))
    idx = {v: i for i, v in enumerate(ep_t)}
    acc = {}
    reasons = sa["reason_tdcp"]
    n_rows = int(have.sum())
    n_acc = 0
    for i in range(n):
        if not have[i] or reasons[i] != "none":
            continue
        n_acc += 1
        k = idx[round(float(t[i]), 3)]
        acc.setdefault(sa["sat"][i], {})[k] = (float(sa["w_tdcp"][i]), float(sa["innov_tdcp"][i]), float(sa["sigma_tdcp"][i]),
                                               float(sa["el_deg"][i]))
    gaps = np.diff(ep_t) if ep_t.size > 1 else np.zeros(0)
    return ep_t, acc, gaps, n_rows, n_acc


def lag_stats(ep_t, acc, lag, max_step_s, which):
    xs, ys, sats = [], [], set()
    for sat, d in acc.items():
        for k, v in d.items():
            j = k + lag
            if j not in d:
                continue
            if np.any(np.diff(ep_t[k:j + 1]) > max_step_s):
                continue
            a, b = v, d[j]
            if which == "w":
                x, y = a[0], b[0]
            else:
                x, y = a[1] * math.sin(math.radians(a[3])), b[1] * math.sin(math.radians(b[3]))
            xs.append(x)
            ys.append(y)
            sats.add(sat)
    x, y = np.array(xs), np.array(ys)
    if x.size < 3:
        return {"pairs": int(x.size)}
    prod = x * y
    return {"pairs": int(x.size), "sats": len(sats), "rho": float(np.sum(prod) / math.sqrt(np.sum(x * x) * np.sum(y * y))),
            "pearson": float(np.corrcoef(x, y)[0, 1]), "mean_product": float(prod.mean()),
            "se_mean_product": float(prod.std(ddof=1) / math.sqrt(prod.size)), "mean_sq": float(np.mean(np.r_[x, y] ** 2))}


def cmd_tdcp_acf(args):
    out = []
    pool = {}
    for run in args.run:
        dev_only(run)
        ep_t, acc, gaps, n_rows, n_acc = tdcp_rows(run)
        rec = {"run": run, "closing_epochs": int(ep_t.size), "rows": n_rows, "accepted_rows": n_acc,
               "closing_gap_s": ({"p10": float(np.percentile(gaps, 10)), "median": float(np.median(gaps)),
                                  "p90": float(np.percentile(gaps, 90))} if gaps.size else None)}
        for which in ("w", "nu_sin_el"):
            for lag in (1, 2):
                rec["%s_lag%d" % (which, lag)] = lag_stats(ep_t, acc, lag, args.max_step_s, which)
        # by elevation of the earlier row (σ_Φ(el) ∝ 1/sin el dominates low; o² relatively larger high)
        for lo, hi in ((0.0, 30.0), (30.0, 90.1)):
            sub = {s: {k: v for k, v in d.items() if lo <= v[3] < hi} for s, d in acc.items()}
            full = {s: {k: acc[s][k] for k in acc[s]} for s in acc}
            xs = []
            for s, d in sub.items():
                for k in d:
                    if k + 1 in full[s] and not np.any(np.diff(ep_t[k:k + 2]) > args.max_step_s):
                        xs.append((d[k][0], full[s][k + 1][0]))
            if len(xs) >= 3:
                a = np.array(xs)
                rec["w_lag1_el_%d_%d" % (lo, min(hi, 90))] = {
                    "pairs": int(a.shape[0]), "rho": float(np.sum(a[:, 0] * a[:, 1]) / math.sqrt(np.sum(a[:, 0] ** 2) * np.sum(a[:, 1] ** 2)))}
        out.append(rec)
        for k in ("w_lag1", "w_lag2"):
            st = rec[k]
            if st.get("pairs", 0) >= 3:
                p = pool.setdefault(k, [0, 0.0])
                p[0] += st["pairs"]
                p[1] += st["rho"] * st["pairs"]
    res = {"runs": out, "pooled_pair_weighted": {k: {"pairs": v[0], "rho": v[1] / v[0]} for k, v in pool.items() if v[0]}}
    with open(args.out, "w") as f:
        json.dump(res, f, indent=1, default=_jsonable)
    print("wrote %s" % args.out)


def chain_model(p, q, p_hat, q_hat, rho_o=0.0, n=200000, seed=1):
    """Scalar chain behind the reading of tdcp-acf (A62): one satellite, pairs chained at a shared epoch, the phase noise of
    that epoch a state (A61 F10/F11). Truth: pair n measures ε(n+1) − ε(n) + o_n with Var ε = p, Var o = q, o AR(1) across
    pairs with ρ_o; the filter assumes p_hat, q_hat, o independent. -> (lag-1 correlation of the standardised innovations,
    mean NIS)."""
    rng = np.random.default_rng(seed)
    eps = rng.normal(0.0, math.sqrt(p), n + 1)
    w = rng.normal(0.0, math.sqrt(q), n)
    o = np.empty(n)
    o[0] = w[0]
    for i in range(1, n):
        o[i] = rho_o * o[i - 1] + math.sqrt(1.0 - rho_o * rho_o) * w[i]
    z = eps[1:] - eps[:-1] + o
    m, v = 0.0, p_hat                      # posterior of the shared epoch's phase (carried from the previous pair)
    zs = np.empty(n)
    nis = np.empty(n)
    for i in range(n):
        s = v + p_hat + q_hat              # H = [−1, +1] on [ε(n), ε(n+1)], R = q_hat
        nu = z[i] + m                      # z − (0 − m)
        zs[i] = nu / math.sqrt(s)
        nis[i] = nu * nu / s
        m, v = p_hat / s * nu, p_hat - p_hat * p_hat / s
    r1 = float(np.sum(zs[1:] * zs[:-1]) / math.sqrt(np.sum(zs[1:] ** 2) * np.sum(zs[:-1] ** 2)))
    return r1, float(nis.mean())


def cmd_chain_model(args):
    rows = []
    for spec in args.case:
        p, q, r = (float(x) for x in spec.split(","))
        r1, nis = chain_model(p, q, args.p_hat, args.q_hat, r)
        rows.append({"p": p, "q": q, "rho_o": r, "lag1": r1, "nis": nis})
        print("truth p=%.1f q=%.1f rho_o=%.2f (model p=%.1f q=%.1f): lag-1 %.3f, NIS %.3f" % (p, q, r, args.p_hat, args.q_hat, r1, nis))
    print("chained, phase not a state: -p/(2p+q) = %.3f" % (-args.p_hat / (2.0 * args.p_hat + args.q_hat)))


# ------------------------------------------------------------------------------------------------ NEES of the global output
def read_anchor(path):
    txt = open(path).read()
    def vec(key):
        m = re.search(r"^%s:\s*\[([^\]]*)\]" % key, txt, re.M)
        return np.array([float(v) for v in m.group(1).split(",")])
    r0 = vec("r0_ecef")
    rows = re.findall(r"^\s*-\s*\[([^\]]*)\]", txt, re.M)
    R = np.array([[float(v) for v in r.split(",")] for r in rows[:3]])
    return r0, R


def interp_rows(t_ref, a, t_q, max_gap):
    """Linear interpolation of the columns of a (first column = time) at t_q; NaN outside or across gaps > max_gap."""
    j = np.searchsorted(t_ref, t_q)
    ok = (j > 0) & (j < len(t_ref))
    j = np.clip(j, 1, len(t_ref) - 1)
    ok &= (t_ref[j] - t_ref[j - 1]) <= max_gap
    f = (t_q - t_ref[j - 1]) / np.maximum(t_ref[j] - t_ref[j - 1], 1e-12)
    v = a[j - 1] * (1 - f)[:, None] + a[j] * f[:, None]
    v[~ok] = np.nan
    return v


def seg_stats(e, s_c, s_lo, s_hi, sel):
    if not np.any(sel):
        return {"n": 0}
    e, s_c, s_lo, s_hi = e[sel], s_c[sel], s_lo[sel], s_hi[sel]
    z2c = (e / s_c) ** 2
    with np.errstate(divide="ignore"):
        z2max = (e / np.maximum(s_lo, 1e-9)) ** 2
    z2min = (e / s_hi) ** 2
    return {"n": int(e.shape[0]), "err_mean_m": e.mean(axis=0).tolist(), "err_rms_m": np.sqrt(np.mean(e ** 2, axis=0)).tolist(),
            "sigma_t_median_m": np.median(s_c, axis=0).tolist(),
            "sigma_rot_over_sigma_t_median": np.median((s_hi - s_c) / s_c, axis=0).tolist(),
            "z2_mean_sigma_t": z2c.mean(axis=0).tolist(), "z2_mean_range": [z2min.mean(axis=0).tolist(), z2max.mean(axis=0).tolist()],
            "nees3_over_3_sigma_t": float(z2c.sum(axis=1).mean() / 3.0)}


def cmd_nees(args):
    dev_only(args.seq)
    dev_only(args.run)
    import gtbase
    import evalcommon as ec
    from fl2g import geodesy
    _, seqreg = ec.load_sequences()
    gt, prov, mask = gtbase.load_gt(args.root, args.seq, seqreg[args.seq])
    gj = json.load(open(os.path.join(args.root, "audit", args.seq, "gt", "gt_audit.json")))
    o = gj["provenance"]["origin"]
    lat0, lon0, h0 = o["lat_deg"] * math.pi / 180, o["lon_deg"] * math.pi / 180, o["h_m"]
    E0 = geodesy.enu_matrix(lat0, lon0)
    r_o = geodesy.pos2ecef(lat0, lon0, h0)
    r0, R_EN = read_anchor(os.path.join(args.run, "anchor.yaml"))
    gt_ecef = r_o[None, :] + gt[:, 1:4] @ E0            # ENU (origin o) -> ECEF: r = r_o + E0ᵀ enu
    gt_N = (gt_ecef - r0[None, :]) @ R_EN                # ECEF -> N: a^N = R_ENᵀ (r − r0)
    ep = read_csv(os.path.join(args.run, "gnss_epochs.csv"))
    trk = np.array([p == "TRACKING" for p in ep["phase"]])
    ant = read_tum(os.path.join(args.run, "traj_ant_N.tum"))
    trN = read_tum(os.path.join(args.run, "traj_N.tum"))
    # one row per t_k (the last one written for that step)
    _, last = np.unique(ant[:, 0][::-1], return_index=True)
    ant = ant[len(ant) - 1 - last]
    _, lastN = np.unique(trN[:, 0][::-1], return_index=True)
    trN = trN[len(trN) - 1 - lastN]
    tk = ep["t_k"]
    use = trk & (ep["trans_sigma_x"] > 0)
    tk = tk[use]
    ja = np.searchsorted(ant[:, 0], tk)
    ok = (ja < len(ant))
    ja = np.minimum(ja, len(ant) - 1)
    ok &= np.abs(ant[ja, 0] - tk) < 1e-6
    jn = np.minimum(np.searchsorted(trN[:, 0], tk), len(trN) - 1)
    ok &= np.abs(trN[jn, 0] - tk) < 1e-6
    a_est = ant[ja, 1:4]
    trans = np.stack([ep["trans_x"][use], ep["trans_y"][use], ep["trans_z"][use]], axis=1)
    s_t = np.stack([ep["trans_sigma_x"][use], ep["trans_sigma_y"][use], ep["trans_sigma_z"][use]], axis=1)
    s_th = np.stack([ep["tilt_sigma_x"][use], ep["tilt_sigma_y"][use], ep["yaw_sigma"][use]], axis=1)
    d = a_est - trans
    # σ_rot,i² = Σ_j ([d]×)_ij² σ_θj²
    dx, dy, dz = d[:, 0], d[:, 1], d[:, 2]
    sx2, sy2, sz2 = s_th[:, 0] ** 2, s_th[:, 1] ** 2, s_th[:, 2] ** 2
    s_rot = np.sqrt(np.stack([dz ** 2 * sy2 + dy ** 2 * sz2, dz ** 2 * sx2 + dx ** 2 * sz2, dy ** 2 * sx2 + dx ** 2 * sy2], axis=1))
    # ground truth at t_k, moved to the receiver antenna: a_9ps = a_gt + R_NI (l_9ps − l_gt)
    g = interp_rows(gt[:, 0], gt_N, tk, 0.5)
    ok &= np.all(np.isfinite(g), axis=1)
    ok &= np.array([not mask.inside(float(t)) for t in tk]) if hasattr(mask, "inside") else True
    dl = np.array(args.lever_9ps) - np.array(args.lever_gt)
    g_ant = g.copy()
    for i in np.where(ok)[0]:
        g_ant[i] = g[i] + quat_to_R(trN[jn[i], 4:8]) @ dl
    e = a_est - g_ant
    t_act = tk[0]
    t_end = tk[-1]
    rec = {"run": args.run, "seq": args.seq, "gt": {k: prov.get(k) for k in ("method", "time_shift_applied_s", "gt_file")},
           "gt_origin": o, "tracking_epochs": int(use.sum()), "compared": int(ok.sum()), "t_first_tracking": float(t_act),
           "span_s": float(t_end - t_act), "pivot_distance_m": {"median": float(np.median(np.linalg.norm(d, axis=1))),
                                                             "max": float(np.max(np.linalg.norm(d, axis=1)))}}
    s_lo = np.abs(s_t - s_rot)
    s_hi = s_t + s_rot
    segs = []
    edges = np.linspace(t_act, t_end + 1e-6, args.segments + 1)
    for a, b in zip(edges[:-1], edges[1:]):
        sel = ok & (tk >= a) & (tk < b)
        st = seg_stats(e, s_t, s_lo, s_hi, sel)
        st["t_from_s"] = float(a - t_act)
        st["t_to_s"] = float(b - t_act)
        st["sigma_rot_median_m"] = np.median(s_rot[sel], axis=0).tolist() if np.any(sel) else None
        segs.append(st)
    rec["segments"] = segs
    near = ok & (np.linalg.norm(d, axis=1) < args.near_pivot_m)
    rec["near_pivot"] = seg_stats(e, s_t, s_lo, s_hi, near)
    rec["near_pivot"]["radius_m"] = args.near_pivot_m
    # stationary windows of the host (A61 ruling 5: the registration walk's path counts the host's speed jitter while
    # parked): speed over ±0.5 s below --still-mps for at least --still-min-s
    W = read_tum(os.path.join(args.run, "traj_W.tum"))
    _, iw = np.unique(W[:, 0], return_index=True)
    W = W[iw]
    tt = np.arange(W[0, 0], W[-1, 0], 0.5)
    P = np.stack([np.interp(tt, W[:, 0], W[:, 1 + k]) for k in range(3)], axis=1)
    spd = np.full(tt.shape, np.inf)
    spd[1:-1] = np.linalg.norm(P[2:] - P[:-2], axis=1) / 1.0
    wins, k0 = [], None
    for k in range(len(tt)):
        still = spd[k] < args.still_mps
        if still and k0 is None:
            k0 = k
        if (not still or k == len(tt) - 1) and k0 is not None:
            if tt[k] - tt[k0] >= args.still_min_s:
                wins.append((float(tt[k0]), float(tt[k])))
            k0 = None
    st_list = []
    in_still = np.zeros(tk.shape, bool)
    for a, b in wins:
        sel = ok & (tk >= a) & (tk <= b)
        in_still |= sel
        st = seg_stats(e, s_t, s_lo, s_hi, sel)
        st["t_from_s"], st["t_to_s"] = a - t_act, b - t_act
        if np.any(sel):
            ii = np.where(sel)[0]
            st["sigma_t_first_m"], st["sigma_t_last_m"] = s_t[ii[0]].tolist(), s_t[ii[-1]].tolist()
        st_list.append(st)
    rec["stationary"] = {"windows": st_list, "all": seg_stats(e, s_t, s_lo, s_hi, in_still),
                         "still_mps": args.still_mps, "still_min_s": args.still_min_s}
    if args.series:
        # one row per compared epoch (the rows every statistic above is computed from)
        with open(args.series, "w") as f:
            f.write("t_rel_s,t_k,err_e_m,err_n_m,err_u_m,sigma_t_e_m,sigma_t_n_m,sigma_t_u_m,sigma_rot_e_m,sigma_rot_n_m,"
                    "sigma_rot_u_m,parked\n")
            for i in np.where(ok)[0]:
                f.write("%.3f,%.6f,%s,%s,%d\n" % (tk[i] - t_act, tk[i], ",".join("%.4f" % v for v in e[i]),
                                                    ",".join("%.4f" % v for v in np.r_[s_t[i], s_rot[i]]), int(in_still[i])))
        rec["series"] = args.series
    # the receiver's own PVT over the same segments (datum check)
    if args.extract and args.calib:
        from fl2g import timecalib_yaml
        p = read_csv(os.path.join(args.extract, "pvt.csv"))
        calib = timecalib_yaml.loads(open(args.calib).read())
        t_g = p["t_gpst"]
        sec = np.floor(t_g)
        first = np.r_[True, sec[1:] != sec[:-1]]
        okp = first & (p["valid_fix"] > 0) & (p["fix_type"] >= 3)
        te = np.array([timecalib_yaml.evaluate(calib, float(t)) for t in t_g[okp]])
        pe = geodesy.geodetic_to_enu(p["lat"][okp] * math.pi / 180, p["lon"][okp] * math.pi / 180, p["h_ell"][okp], lat0, lon0, h0)
        p_N = ((r_o[None, :] + pe @ E0) - r0[None, :]) @ R_EN
        gp = interp_rows(gt[:, 0], gt_N, te, 0.5)
        okq = np.all(np.isfinite(gp), axis=1) & np.array([not mask.inside(float(t)) for t in te])
        # GT to the receiver antenna with the filter's attitude at the nearest step
        jq = np.clip(np.searchsorted(trN[:, 0], te), 0, len(trN) - 1)
        okq &= np.abs(trN[jq, 0] - te) < 0.1
        for i in np.where(okq)[0]:
            gp[i] = gp[i] + quat_to_R(trN[jq[i], 4:8]) @ dl
        ep_ = p_N - gp
        hacc, vacc = p["h_acc"][okp], p["v_acc"][okp]
        pv = []
        for a, b in zip(edges[:-1], edges[1:]):
            sel = okq & (te >= a) & (te < b)
            if not np.any(sel):
                pv.append({"n": 0})
                continue
            pv.append({"t_from_s": float(a - t_act), "t_to_s": float(b - t_act), "n": int(sel.sum()),
                       "err_mean_m": ep_[sel].mean(axis=0).tolist(), "err_rms_m": np.sqrt(np.mean(ep_[sel] ** 2, axis=0)).tolist(),
                       "h_acc_median": float(np.median(hacc[sel])), "v_acc_median": float(np.median(vacc[sel]))})
        rec["receiver_pvt"] = pv
        # filter minus receiver PVT at the PVT instants (both against nothing: the ground truth datum drops out)
        fa = interp_rows(ant[:, 0], ant[:, 1:4], te, 0.2)
        okf = np.all(np.isfinite(fa), axis=1) & (te >= t_act)
        fd = []
        for a, b in zip(edges[:-1], edges[1:]):
            sel = okf & (te >= a) & (te < b)
            if np.any(sel):
                dd = fa[sel] - p_N[sel]
                fd.append({"t_from_s": float(a - t_act), "t_to_s": float(b - t_act), "n": int(sel.sum()),
                           "mean_m": dd.mean(axis=0).tolist(), "rms_m": np.sqrt(np.mean(dd ** 2, axis=0)).tolist()})
            else:
                fd.append({"n": 0})
        rec["filter_minus_receiver_pvt"] = fd
    with open(args.out, "w") as f:
        json.dump(rec, f, indent=1, default=_jsonable)
    print("wrote %s" % args.out)


# ------------------------------------------------------------------------------------------------ byte identity
FILES = ("traj_W.tum", "traj_N.tum", "traj_N_odom.tum", "traj_ant_N.tum", "traj_ant_N_odom.tum", "gnss_epochs.csv", "events.log",
         "summary.json")


def sha(path):
    if not os.path.isfile(path):
        return None
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()


TIMING_COLUMNS = {"elapsed_ms"}                      # wall-clock columns: never bit-reproducible


def sha_csv_without(path, drop):
    """sha256 of a CSV with the named columns removed (None if the file is absent)."""
    if not os.path.isfile(path):
        return None
    h = hashlib.sha256()
    with open(path) as f:
        head = f.readline().rstrip("\n").split(",")
        keep = [i for i, c in enumerate(head) if c not in drop]
        h.update((",".join(head[i] for i in keep) + "\n").encode())
        for line in f:
            v = line.rstrip("\n").split(",")
            h.update((",".join(v[i] for i in keep if i < len(v)) + "\n").encode())
    return h.hexdigest()


def sha_json_without(path, prefixes=("elapsed_",)):
    if not os.path.isfile(path):
        return None
    d = json.load(open(path))
    d = {k: v for k, v in d.items() if not k.startswith(prefixes)}
    return hashlib.sha256(json.dumps(d, sort_keys=True).encode()).hexdigest()


def cmd_same(args):
    dev_only(args.a)
    dev_only(args.b)
    rec = {"a": args.a, "b": args.b, "files": {}}
    for fn in FILES:
        pa, pb = os.path.join(args.a, fn), os.path.join(args.b, fn)
        if fn == "gnss_epochs.csv":
            ha, hb = sha_csv_without(pa, TIMING_COLUMNS), sha_csv_without(pb, TIMING_COLUMNS)
            note = "without the wall-clock column elapsed_ms"
        elif fn == "summary.json":
            ha, hb = sha_json_without(pa), sha_json_without(pb)
            note = "without the wall-clock elapsed_* keys"
        else:
            ha, hb = sha(pa), sha(pb)
            note = ""
        state = "absent in both" if ha is None and hb is None else ("identical" if ha == hb else "different")
        rec["files"][fn] = {"a": ha, "b": hb, "identical": state == "identical", "state": state, "note": note}
    if args.u:
        dev_only(args.u)
        u = open(os.path.join(args.u, "traj_W.tum")).read().splitlines()
        rec["pre_activation_vs_U"] = {}
        for tag, run in (("a", args.a), ("b", args.b)):
            ev = [e for e in read_events(run) if e["kind"] == "ACTIVATE"]
            lines = open(os.path.join(run, "traj_W.tum")).read().splitlines()
            if ev:
                t_act = ev[0]["t_k"]
                pre = [l for l in lines if l.strip() and float(l.split()[0]) < t_act - 1e-9]
            else:
                pre = lines
            same = len(pre) <= len(u) and all(x == y for x, y in zip(pre, u))
            rec["pre_activation_vs_U"][tag] = {"lines": len(pre), "identical_to_U_prefix": bool(same),
                                               "first_difference": next((i for i, (x, y) in enumerate(zip(pre, u)) if x != y), None)}
    with open(args.out, "w") as f:
        json.dump(rec, f, indent=1, default=_jsonable)
    print("wrote %s" % args.out)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    a = sub.add_parser("events")
    a.add_argument("--run", action="append", required=True)
    a.add_argument("--out", required=True)
    a = sub.add_parser("tdcp-acf")
    a.add_argument("--run", action="append", required=True)
    a.add_argument("--max-step-s", type=float, default=1.5, help="consecutive closing epochs further apart break the chain")
    a.add_argument("--out", required=True)
    a = sub.add_parser("chain-model")
    a.add_argument("--p-hat", type=float, default=28.0, help="model phase variance of one epoch (mm², 45° elevation)")
    a.add_argument("--q-hat", type=float, default=5.0, help="model per-pair independent variance (mm²: rate, wind-up, time offset)")
    a.add_argument("--case", action="append", default=[], help="truth p,q,rho_o (repeatable)")
    a = sub.add_parser("nees")
    a.add_argument("--run", required=True)
    a.add_argument("--seq", required=True)
    a.add_argument("--root", required=True)
    a.add_argument("--extract")
    a.add_argument("--calib")
    a.add_argument("--lever-9ps", type=float, nargs=3, default=[-0.31635, 0.10926, 0.4006])
    a.add_argument("--lever-gt", type=float, nargs=3, default=[-0.1763, -0.0141, 0.4006])
    a.add_argument("--segments", type=int, default=3)
    a.add_argument("--near-pivot-m", type=float, default=10.0)
    a.add_argument("--still-mps", type=float, default=0.05, help="host speed below which a window counts as parked")
    a.add_argument("--still-min-s", type=float, default=5.0)
    a.add_argument("--series", help="also write the compared epochs as CSV (figures)")
    a.add_argument("--out", required=True)
    a = sub.add_parser("same")
    a.add_argument("--a", required=True)
    a.add_argument("--b", required=True)
    a.add_argument("--u")
    a.add_argument("--out", required=True)
    args = ap.parse_args(argv)
    {"events": cmd_events, "tdcp-acf": cmd_tdcp_acf, "chain-model": cmd_chain_model, "nees": cmd_nees, "same": cmd_same}[args.cmd](args)


if __name__ == "__main__":
    main()
