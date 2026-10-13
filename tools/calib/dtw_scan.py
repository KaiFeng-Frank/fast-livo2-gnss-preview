#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""A81 section 2 item 1: offline replica of the online DTW of the LIVGO-literal arm and the scan over its grid and window.

What it replicates (gnss/include/livo_gnss/livgo.hpp `livgoDtwLag`, gnss/src/gnss_fusion_livgo.cpp `livgoTryDtw`):

  * `dtw_lag`     livgoDtwLag, statement by statement (same operation order on IEEE doubles, same tie-breaking of the
                  recursion: diagonal, then up, then left, each on a strict `<`; same band indexing, same median element), so
                  the lag it returns is meant to be bit-identical to the C++ one. `tools/calib/dtw_align.cpp` runs the C++
                  function on exactly the series this tool hands to `dtw_lag` and `compare` checks the two result files.
  * `online_first_lag`  the one-shot behaviour of livgoTryDtw: every GNSS epoch whose Doppler least squares and host antenna
                  velocity exist pushes one sample (t_est, |v_GNSS|, |v_host|) onto the series; once the series holds at least
                  `dtw_min_epochs` samples, livgoDtw is tried before the next push, and the first success is the lag that
                  `TimeMap::addOffset` adds, once, for the whole run (`dtw_applied`).

Inputs, rebuilt from the data the way the C++ builds them (see docs/dev/A81_DTW_SCAN.md for the line-by-line argument):

  * GNSS speed  = |v_ecef| of the single-epoch Doppler least squares (C++ `RobustDopplerLs`, position from `RobustSpp`).
                  Here: tools/fl2g/gnsslsq.py `spp` and `solve_doppler` (same range-rate model, the same Sagnac-rate sign,
                  4 unknowns, 0/1 rejection; the C++ one adds an LMS stage and Huber weights) on the primary signal of each
                  satellite with a valid pseudorange (COLLECT is pseudorange-driven), C/N0 >= 35 dB-Hz and elevation >= 15 deg
                  (livgo/cn0_hard masks of the literal arm), from the bag's own messages plus the broadcast file.
  * time of the sample t_est = t_rx - b/c - 18 s (+ nothing: the literal arm reads no A18 file, A81 1 item 2).
  * host speed  = |z|, z = v + R [omega]x l, the antenna velocity of `ImuBridge::evaluate`, from the state trace of the U run
                  (cycle states at the LiDAR rate, linear in time between two cycles exactly as the bridge's constant
                  acceleration inside a cycle; omega from the attitude of the neighbouring cycles).

Sub-commands:  extract-gnss | validate | xcorr | motion | hostcheck | scan | select | report | synth | compare   (see --help of each).
Pure numpy; runs in the project image (python 3.8, rosbag) on the remote host only.
"""
import argparse
import csv
import gzip
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import numpy as np  # noqa: E402

CLIGHT = 299792458.0
LEAP_S = 18.0
INF = 1e300                      # kInf of livgoDtwLag

GRIDS = (0.05, 0.1, 0.2)         # A81 2 item 1
WINDOWS = (20.0, 40.0, 60.0, 120.0)
BAND_S = 2.0                     # LivgoDtwParams::band_s, not a configuration key
START_STEP_S = 60.0              # A81: one start every 60 s
TIE_S = 0.005                    # A81: "differences below 5 ms: the shorter window"
MAX_ATTEMPTS = 200               # tries after the first full window before a start is called a failure (see online_first_lag)


# ------------------------------------------------------------------------------------------------------------------------
# livgoDtwLag
# ------------------------------------------------------------------------------------------------------------------------
def _lround(x):
    """C lround: halves away from zero (python's round() is banker's)."""
    return int(math.floor(x + 0.5)) if x >= 0.0 else -int(math.floor(-x + 0.5))


def dtw_lag(tg, vg, th, vh, grid_s=0.1, window_s=40.0, band_s=2.0):
    """livgoDtwLag(tg, vg, th, vh, &lag, LivgoDtwParams{grid_s, window_s, band_s}) -> (ok, lag_s).
    Sequences are python lists (or arrays) of float; nothing is modified."""
    if len(tg) != len(vg) or len(th) != len(vh) or len(tg) < 5 or len(th) < 5:
        return False, None
    if not (math.isfinite(grid_s) and grid_s > 0.0 and math.isfinite(window_s) and math.isfinite(band_s) and band_s > 0.0):
        return False, None
    t0 = max(float(tg[0]), float(th[0]))
    t1 = min(float(tg[-1]), float(th[-1]))
    if not (t1 - t0 >= 3.0):
        return False, None
    dt = float(grid_s)
    n = int((t1 - t0) / dt)
    if n < 20:
        return False, None
    n_max = int(math.floor(window_s / dt + 1e-9))
    if n_max < 20:
        return False, None
    if n > n_max:
        n = n_max
    grid0 = t1 - n * dt

    def resample(t, v):
        t = [float(x) for x in t]
        v = [float(x) for x in v]
        m = len(t)
        o = [0.0] * n
        j = 0
        for i in range(n):
            ti = grid0 + (float(i) + 0.5) * dt
            while j + 1 < m and t[j + 1] < ti:
                j += 1
            if j + 1 >= m or not (t[j + 1] > t[j]):
                o[i] = v[j]
                continue
            u = min(1.0, max(0.0, (ti - t[j]) / (t[j + 1] - t[j])))
            o[i] = (1.0 - u) * v[j] + u * v[j + 1]
        return o

    def norm(a):
        m = 0.0
        for x in a:                              # sequential sums, as the C++ loops (no pairwise summation)
            m += x
        m /= float(len(a))
        s2 = 0.0
        for x in a:
            s2 += (x - m) * (x - m)
        s = math.sqrt(s2 / float(len(a)))
        if not (s > 1e-6):
            s = 1.0
        return [(x - m) / s for x in a]

    ag = norm(resample(tg, vg))
    ah = norm(resample(th, vh))

    W = max(1, _lround(band_s / dt))
    stride = 2 * W + 1
    cost = [INF] * (n * stride)
    pred = bytearray(n * stride)

    # row 0: j = 0 .. min(n-1, W);  k = j - (i - W)
    for j in range(0, min(n - 1, W) + 1):
        d = ag[0] - ah[j]
        cost[j + W] = d * d
    for i in range(1, n):
        j0 = max(0, i - W)
        j1 = min(n - 1, i + W)
        ai = ag[i]
        base = i * stride
        pbase = (i - 1) * stride
        for j in range(j0, j1 + 1):
            k = j - (i - W)
            p = base + k
            d = ai - ah[j]
            d = d * d
            best = INF
            how = 0
            # diag = at(i-1, j-1): same k when j-1 >= 0; up = at(i-1, j): k+1 < stride; left = at(i, j-1): k-1 >= 0, j-1 >= 0
            if j - 1 >= 0:
                c = cost[pbase + k]
                if c < best:
                    best = c
                    how = 1
            if k + 1 < stride:
                c = cost[pbase + k + 1]
                if c < best:
                    best = c
                    how = 2
            if k - 1 >= 0 and j - 1 >= 0:
                c = cost[p - 1]
                if c < best:
                    best = c
                    how = 3
            if best >= INF / 2:
                continue
            cost[p] = d + best
            pred[p] = how
    jbest = -1
    cbest = INF
    for j in range(max(0, n - 1 - W), min(n - 1, n - 1 + W) + 1):
        k = j - (n - 1 - W)
        if 0 <= k < stride:
            c = cost[(n - 1) * stride + k]
            if c < cbest:
                cbest = c
                jbest = j
    if jbest < 0:
        return False, None
    lags = []
    i, j = n - 1, jbest
    while i >= 0 and j >= 0:
        lags.append(float(j - i) * dt)
        k = j - (i - W)
        if k < 0 or k >= stride:
            break
        how = pred[i * stride + k]
        if how == 1:
            i -= 1
            j -= 1
        elif how == 2:
            i -= 1
        elif how == 3:
            j -= 1
        else:
            break
    if len(lags) < 10:
        return False, None
    lags.sort()
    lag = lags[len(lags) // 2]                   # std::nth_element(.., size/2): the element of that rank
    return (True, lag) if math.isfinite(lag) else (False, None)


# ------------------------------------------------------------------------------------------------------------------------
# the one-shot online behaviour
# ------------------------------------------------------------------------------------------------------------------------
def online_first_lag(t, vg, vh, start, min_epochs, grid_s, window_s, band_s=BAND_S, max_attempts=MAX_ATTEMPTS, sink=None):
    """livgoTryDtw from the sample `start` of the pushable series (t, vg, vh): the series grows one sample per push, the
    try runs when it holds >= min_epochs samples, the first success wins. A start whose first `max_attempts` tries all fail
    is reported as a failure (a bound of this tool: on the real data the function does not fail once a full window exists).
    `sink(series_slice, grid, window, band, ok, lag)` is called for every call of dtw_lag (for the C++ comparison).
    Returns dict(ok, lag, n, attempts, i_end)."""
    n_all = len(t)
    first = start + min_epochs
    if first > n_all:
        return {"ok": False, "lag": None, "n": n_all - start, "attempts": 0, "i_end": None, "why": "data end before a full window"}
    attempts = 0
    for end in range(first, min(n_all, first + max_attempts - 1) + 1):
        sl = slice(start, end)
        ok, lag = dtw_lag(t[sl], vg[sl], t[sl], vh[sl], grid_s, window_s, band_s)
        attempts += 1
        if sink is not None:
            sink(sl, grid_s, window_s, band_s, ok, lag)
        if ok:
            return {"ok": True, "lag": lag, "n": end - start, "attempts": attempts, "i_end": end, "why": ""}
    return {"ok": False, "lag": None, "n": min(n_all, first + max_attempts - 1) - start, "attempts": attempts, "i_end": None,
            "why": "no success in %d tries" % attempts}


def mad(x):
    x = np.asarray(x, dtype=float)
    if x.size == 0:
        return float("nan")
    return float(np.median(np.abs(x - np.median(x))))


# ------------------------------------------------------------------------------------------------------------------------
# GNSS side
# ------------------------------------------------------------------------------------------------------------------------
MAX_AGE = {"G": 7200.0, "E": 7200.0, "C": 3600.0, "R": 1800.0, "J": 7200.0}   # tools/datasets/smartpnt/gnss_check.py


def read_gnss(path):
    from fl2g import gnss_msgs, rosio
    rb = rosio.RawBag(path)
    epochs, eph, geph = [], [], []
    try:
        for topic, _datatype, raw, t_rec, pytype in rb.iter_raw(topics=[gnss_msgs.TOPIC_MEAS, gnss_msgs.TOPIC_EPHEM, gnss_msgs.TOPIC_GLO_EPHEM]):
            if topic == gnss_msgs.TOPIC_MEAS:
                epochs.append(gnss_msgs.meas_epoch(rb.deserialize(pytype, raw), t_rec))
            elif topic == gnss_msgs.TOPIC_EPHEM:
                eph.append(gnss_msgs.kepler_eph(rb.deserialize(pytype, raw), t_rec))
            else:
                geph.append(gnss_msgs.glo_eph(rb.deserialize(pytype, raw), t_rec))
    finally:
        rb.close()
    return epochs, eph, geph


def gnss_speed(epochs, eph, geph, nav_recs, nav_hdr, cn0_min=35.0, el_min_deg=15.0, leap_s=LEAP_S, offset_s=0.0, progress=0):
    """Per receiver epoch: SPP (clock for the time map) and the Doppler least squares speed under the masks of the literal arm.
    Returns dict of arrays (one entry per epoch of `epochs`)."""
    from fl2g import gnsslsq, obs as fobs, orbits
    sel = orbits.EphSelector(eph + geph + nav_recs, MAX_AGE, causal=False)
    ion = None
    a, b = nav_hdr.get("iono_gps_alpha"), nav_hdr.get("iono_gps_beta")
    if a and b:
        ion = [float(x or 0.0) for x in list(a) + list(b)]
    n = len(epochs)
    out = {k: np.full(n, np.nan) for k in ("t_rx", "t_est", "speed", "clk_m", "spp_rms", "vx", "vy", "vz", "px", "py", "pz")}
    out["n_rows"] = np.zeros(n, dtype=int)
    out["n_cand"] = np.zeros(n, dtype=int)
    out["spp_ok"] = np.zeros(n, dtype=bool)
    out["dls_ok"] = np.zeros(n, dtype=bool)
    last = None
    b_hint = 0.0
    for k, e in enumerate(epochs):
        t_rx = e["t_gpst"]
        out["t_rx"][k] = t_rx
        # COLLECT is pseudorange-driven (gnss_fusion_livgo.cpp geometryAt: `!sig->psrValid()` -> no row at all)
        obs = [o for o in fobs.ls_obs(e) if o["psr_ok"]]
        out["n_cand"][k] = sum(1 for o in obs if o["cn0"] >= cn0_min)
        sp = gnsslsq.spp(obs, t_rx, sel, iono=ion, x0=last, cn0_min=cn0_min, el_mask_deg=el_min_deg)
        if sp is not None and math.isfinite(sp["clk_m"]) and np.all(np.isfinite(sp["rr"])):
            out["spp_ok"][k] = True
            out["clk_m"][k] = sp["clk_m"]
            out["spp_rms"][k] = sp["rms_m"]
            out["px"][k], out["py"][k], out["pz"][k] = sp["rr"]
            if sp["rms_m"] < 100.0:
                last = sp["rr"]
            b_hint = sp["clk_m"]
            rows = gnsslsq.doppler_rows(obs, t_rx, sel, sp["rr"], cn0_min=cn0_min, el_mask_deg=el_min_deg)
            dv = gnsslsq.solve_doppler(rows, sp["rr"], min_rows=4)
            if dv is not None and np.all(np.isfinite(dv["v_ecef"])):
                out["dls_ok"][k] = True
                out["speed"][k] = float(np.linalg.norm(dv["v_ecef"]))
                out["vx"][k], out["vy"][k], out["vz"][k] = dv["v_ecef"]
                out["n_rows"][k] = dv["n"]
        # t_est = t_gpst,true - leap + offset, t_gpst,true = t_rcv - b/c   (b of this epoch's SPP, else the last one: b_hint)
        out["t_est"][k] = (t_rx - (out["clk_m"][k] if out["spp_ok"][k] else b_hint) / CLIGHT) - leap_s + offset_s
        if progress and (k + 1) % progress == 0:
            print("  epoch %d/%d spp %.3f dls %.3f" % (k + 1, n, out["spp_ok"][:k + 1].mean(), out["dls_ok"][:k + 1].mean()), flush=True)
    return out


def cmd_extract_gnss(a):
    from fl2g import rinexnav
    epochs, eph, geph = read_gnss(a.bag)
    hdr, recs = rinexnav.read_nav(a.nav)
    print("bag %s: %d epochs, %d kepler, %d glonass ephemerides; nav file %d records" % (a.bag, len(epochs), len(eph), len(geph), len(recs)), flush=True)
    res = gnss_speed(epochs, eph, geph, recs, hdr, a.cn0_min, a.el_min, progress=2000)
    meta = {"bag": a.bag, "nav": a.nav, "cn0_min_dbhz": a.cn0_min, "el_min_deg": a.el_min, "leap_s": LEAP_S, "offset_s": 0.0,
            "epochs": len(epochs), "spp_ok": int(res["spp_ok"].sum()), "dls_ok": int(res["dls_ok"].sum()),
            "tool": "dtw_scan.py extract-gnss"}
    np.savez(a.out, meta=json.dumps(meta), **res)
    print(json.dumps(meta))
    return 0


# ------------------------------------------------------------------------------------------------------------------------
# host side
# ------------------------------------------------------------------------------------------------------------------------
def rotvec(R):
    """Log map of a rotation matrix (angle < pi)."""
    c = max(-1.0, min(1.0, (np.trace(R) - 1.0) / 2.0))
    ang = math.acos(c)
    w = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    if ang < 1e-8:
        return 0.5 * w
    return w * (ang / (2.0 * math.sin(ang)))


def host_antenna_velocity(state, lever, mode="bridge"):
    """Antenna velocity per cycle of a state trace (t_k, R, p, v from fl2g.replay.load_state_trace).
      bridge  z = v + R (omega x l), omega = Log(R_{k-1}^T R_{k+1}) / (t_{k+1} - t_{k-1})      (ImuBridge::evaluate's z)
      vonly   z = v                                                                              (no lever term)
      track   z = d/dt (p + R l) by central differences of the neighbouring cycles
    """
    t, R, p, v = state["t"], state["R"], state["p"], state["v"]
    l = np.asarray(lever, dtype=float)
    n = len(t)
    if mode == "vonly" or not np.any(l):
        return v.copy()
    if mode == "track":
        y = p + np.einsum("nij,j->ni", R, l)
        z = np.empty_like(y)
        z[1:-1] = (y[2:] - y[:-2]) / (t[2:] - t[:-2])[:, None]
        z[0], z[-1] = z[1], z[-2]
        return z
    w = np.zeros((n, 3))
    for k in range(1, n - 1):
        w[k] = rotvec(R[k - 1].T @ R[k + 1]) / (t[k + 1] - t[k - 1])
    w[0], w[-1] = w[1], w[-2]
    return v + np.einsum("nij,nj->ni", R, np.cross(w, l))


def load_state(run_dir):
    from fl2g import replay
    p = os.path.join(run_dir, "state_trace.txt")
    if os.path.exists(p):
        return replay.load_state_trace(p)
    import tempfile
    pz = p + ".gz"
    with gzip.open(pz, "rb") as f, tempfile.NamedTemporaryFile("wb", suffix=".txt", dir=os.environ.get("TMPDIR")) as tmp:
        tmp.write(f.read())
        tmp.flush()
        return replay.load_state_trace(tmp.name)


def host_speed_at(state, lever, t_query, mode="bridge"):
    """|z| at the query times (t_est of the GNSS epochs), z linear between two cycles; NaN outside the trace."""
    t = state["t"]
    z = host_antenna_velocity(state, lever, mode)
    out = np.full(len(t_query), np.nan)
    inside = (t_query >= t[0]) & (t_query <= t[-1])
    tq = t_query[inside]
    j = np.clip(np.searchsorted(t, tq, side="right") - 1, 0, len(t) - 2)
    u = (tq - t[j]) / (t[j + 1] - t[j])
    zz = z[j] * (1.0 - u)[:, None] + z[j + 1] * u[:, None]
    out[inside] = np.linalg.norm(zz, axis=1)
    return out


def first_collect_epoch(gnss_epochs_csv):
    """(t_gpst, t_est, phase counts) of the first COLLECT epoch of an OBSERVE run's gnss_epochs.csv."""
    opener = gzip.open if gnss_epochs_csv.endswith(".gz") else open
    with opener(gnss_epochs_csv, "rt") as f:
        r = csv.reader(f)
        h = next(r)
        ix = {k: i for i, k in enumerate(h)}
        first = None
        counts = {}
        for row in r:
            if len(row) != len(h):
                continue
            ph = row[ix["phase"]]
            counts[ph] = counts.get(ph, 0) + 1
            if ph == "COLLECT" and first is None:
                first = (float(row[ix["t_gpst"]]), float(row[ix["t_est"]]))
    return first, counts


def pushable(gnss, state, lever, mode="bridge", t_from=None):
    """The samples the C++ would push (dop.ok, finite host velocity) as arrays t_rx, t_est, vg, vh, from t_from on (receiver time)."""
    vh = host_speed_at(state, lever, gnss["t_est"], mode)
    m = gnss["dls_ok"] & np.isfinite(vh) & np.isfinite(gnss["t_est"])
    if t_from is not None:
        m &= gnss["t_rx"] >= t_from - 1e-9
    return {"t_rx": gnss["t_rx"][m], "t_est": gnss["t_est"][m], "vg": gnss["speed"][m], "vh": vh[m]}


def xcorr_best_delay(t, vg, vh, search=1.0, step=0.001, smooth_s=0.0):
    """D* maximising corr(v_g(t), v_h(t - D)) over D in [-search, search] (GNSS delayed by D: the DTW lag is -D). Series are
    resampled to 10 Hz on the pushable samples, linear interpolation; returns (D*, corr at D*, corr at 0)."""
    t = np.asarray(t)
    lo, hi = t[0] + search + 1.0, t[-1] - search - 1.0
    if hi - lo < 20.0:
        return None
    grid = np.arange(lo, hi, 0.1)
    g = np.interp(grid, t, vg)
    best = None
    c0 = None
    for D in np.arange(-search, search + 1e-9, step):
        h = np.interp(grid - D, t, vh)
        c = float(np.corrcoef(g, h)[0, 1])
        if abs(D) < 1e-9:
            c0 = c
        if best is None or c > best[1]:
            best = (float(D), c)
    return best[0], best[1], c0


# ------------------------------------------------------------------------------------------------------------------------
# scan
# ------------------------------------------------------------------------------------------------------------------------
class Dump:
    """Writes the series and calls handed to dtw_lag, for tools/calib/dtw_align.cpp (hexfloat text)."""

    def __init__(self, path):
        self.f = open(path, "w")
        self.ids = {}
        self.calls = 0

    def series(self, key, t, vg, vh):
        if key in self.ids:
            return
        self.ids[key] = len(self.ids)
        self.f.write("SERIES %d %d\n" % (self.ids[key], len(t)))
        for a, b, c in zip(t, vg, vh):
            self.f.write("%s %s %s\n" % (float(a).hex(), float(b).hex(), float(c).hex()))

    def call(self, key, grid, window, band, ok, lag):
        self.calls += 1
        self.f.write("CALL %d %s %s %s %d %s\n" % (self.ids[key], float(grid).hex(), float(window).hex(), float(band).hex(), 1 if ok else 0,
                                                   float(lag).hex() if ok else "-"))

    def close(self):
        self.f.close()


def window_stats(vg, vh):
    return {"host_std": float(np.std(vh)), "host_min": float(np.min(vh)), "host_max": float(np.max(vh)),
            "gnss_std": float(np.std(vg)), "corr": float(np.corrcoef(vg, vh)[0, 1]) if np.std(vg) > 0 and np.std(vh) > 0 else float("nan")}


def scan_sequence(seq, s, t_collect0, step_s=START_STEP_S, grids=GRIDS, windows=WINDOWS, epoch_dt=0.1, dump=None):
    """s: pushable series (arrays t_rx, t_est, vg, vh) from t_collect0 on. Returns list of rows, one per
    (start k, grid, window). Starts: t_collect0 + k*step_s; the common start set is the one where the longest window fits."""
    t_rx, te, vg, vh = s["t_rx"], s["t_est"], s["vg"], s["vh"]
    t_end = t_rx[-1]
    t_long = max(windows)
    rows = []
    k = 0
    while True:
        t_start = t_collect0 + k * step_s
        feasible_all = t_start + t_long <= t_end
        if t_start + min(windows) > t_end:
            break
        i0 = int(np.searchsorted(t_rx, t_start - 1e-9))
        for w in windows:
            min_epochs = int(round(w / epoch_dt))
            key = (seq, k, w)
            sink = None
            if dump is not None:
                def sink(sl, grid, window, band, ok, lag, _key=key):
                    dump.series(_key + (sl.start, sl.stop), te[sl], vg[sl], vh[sl])
                    dump.call(_key + (sl.start, sl.stop), grid, window, band, ok, lag)
            for g in grids:
                r = online_first_lag(te, vg, vh, i0, min_epochs, g, w, BAND_S, sink=sink)
                row = {"seq": seq, "k": k, "start_t_rx": float(t_start), "start_rel_s": k * step_s, "grid_s": g, "window_s": w,
                       "min_epochs": min_epochs, "ok": r["ok"], "lag_s": r["lag"] if r["ok"] else "", "n_samples": r["n"],
                       "attempts": r["attempts"], "common_start": bool(t_start + t_long <= t_end), "why": r["why"]}
                if r["ok"]:
                    row["t_success_rel_s"] = float(t_rx[r["i_end"] - 1] - t_start)
                    row.update(window_stats(vg[i0:r["i_end"]], vh[i0:r["i_end"]]))
                rows.append(row)
        k += 1
    return rows


def cmd_scan(a):
    g = np.load(a.gnss, allow_pickle=False)
    gnss = {k: g[k] for k in g.files if k != "meta"}
    state = load_state(a.host_run)
    t0 = a.collect_t_rx
    s = pushable(gnss, state, a.lever, a.host_mode, t_from=t0)
    print("%s: %d pushable samples from t_rx %.3f (GNSS epochs %d, DLS ok %d)" % (a.seq, len(s["t_rx"]), t0, len(gnss["t_rx"]), int(gnss["dls_ok"].sum())), flush=True)
    dump = Dump(a.dump) if a.dump else None
    rows = scan_sequence(a.seq, s, t0, dump=dump, epoch_dt=a.epoch_dt)
    if dump:
        dump.close()
        print("dumped %d calls" % dump.calls)
    keys = ["seq", "k", "start_t_rx", "start_rel_s", "grid_s", "window_s", "min_epochs", "ok", "lag_s", "n_samples", "attempts", "common_start", "why",
            "t_success_rel_s", "host_std", "host_min", "host_max", "gnss_std", "corr"]
    with open(a.out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
        w.writeheader()
        for r in rows:
            w.writerow(r)
    print("wrote %d rows to %s" % (len(rows), a.out))
    return 0


# ------------------------------------------------------------------------------------------------------------------------
# synthetic cases for the C++ comparison (ties, constants, gaps, failures)
# ------------------------------------------------------------------------------------------------------------------------
def synthetic_cases(seed=0):
    """(name, t, vg, vh, grid, window) tuples chosen to hit every branch of livgoDtwLag: the delayed smooth series, a quantised walk
    (many equal costs: tie-breaking of the recursion and of the end cell), exactly constant and sub-guard-noise series (std guard),
    irregular time stamps with gaps (resampling pointer), and the refusals (overlap < 3 s, < 20 grid steps, short series)."""
    rng = np.random.RandomState(seed)
    out = []
    f = lambda x: math.sin(0.9 * x) + 0.7 * math.sin(0.37 * x + 1.0) + 0.5 * math.sin(1.9 * x + 0.3) + 0.3 * math.sin(3.1 * x)
    for g in GRIDS:
        for w in WINDOWS:
            t = [0.1 * i for i in range(int(1.2 * w / 0.1) + 5)]
            for delay in (-0.7, 0.0, 0.33, 1.2):
                for noise in (0.0, 0.05):
                    vh = [f(x) + noise * rng.randn() for x in t]
                    vg = [f(x - delay) + noise * rng.randn() for x in t]
                    out.append(("delay%+.2f_noise%g" % (delay, noise), t, vg, vh, g, w))
            q = np.cumsum(rng.choice([-1.0, 0.0, 1.0], size=len(t))).tolist()
            q2 = np.cumsum(rng.choice([-1.0, 0.0, 1.0], size=len(t))).tolist()
            out.append(("quantised_walk_pair", t, q, q2, g, w))
            out.append(("quantised_walk_same", t, q, q, g, w))
            out.append(("constant", t, [1.0] * len(t), [2.0] * len(t), g, w))
            out.append(("sub_guard_noise", t, (1e-8 * rng.randn(len(t))).tolist(), (1e-8 * rng.randn(len(t))).tolist(), g, w))
            tt, a, b = [], [], []
            x = 0.0
            while x < 1.2 * w:
                x += 0.1 if rng.rand() > 0.03 else 0.1 + rng.rand() * 1.5     # dropouts up to 1.5 s
                tt.append(x)
                a.append(f(x - 0.2) + 0.05 * rng.randn())
                b.append(f(x) + 0.05 * rng.randn())
            out.append(("gaps", tt, a, b, g, w))
    t = [0.1 * i for i in range(29)]
    out.append(("overlap_2.8s", t, [f(x) for x in t], [f(x) for x in t], 0.1, 40.0))
    t = [0.1 * i for i in range(300)]
    out.append(("window_below_20_steps", t, [f(x) for x in t], [f(x) for x in t], 0.1, 1.9))
    out.append(("four_samples", t[:4], [1.0] * 4, [1.0] * 4, 0.1, 40.0))
    out.append(("band_wider_than_series", t[:60], [f(x) for x in t[:60]], [f(x - 0.4) for x in t[:60]], 0.05, 40.0))
    return out


def cmd_synth(a):
    dump = Dump(a.dump)
    n_ok = n_fail = 0
    for k, (name, t, vg, vh, g, w) in enumerate(synthetic_cases(a.seed)):
        key = ("synth", k)
        dump.series(key, t, vg, vh)
        ok, lag = dtw_lag(t, vg, t, vh, g, w, BAND_S)
        dump.call(key, g, w, BAND_S, ok, lag)
        n_ok += 1 if ok else 0
        n_fail += 0 if ok else 1
    dump.close()
    print("synthetic cases: %d calls (%d ok, %d refused)" % (dump.calls, n_ok, n_fail))
    return 0


# ------------------------------------------------------------------------------------------------------------------------
# validation of the rebuilt time axis and epoch availability against an OBSERVE run of the C++ (gnss_epochs.csv)
# ------------------------------------------------------------------------------------------------------------------------
def read_obs_epochs(gnss_epochs_csv, cols=("t_gpst", "t_est", "t_k", "delta", "phase", "spp_valid")):
    opener = gzip.open if gnss_epochs_csv.endswith(".gz") else open
    out = {c: [] for c in cols}
    with opener(gnss_epochs_csv, "rt") as f:
        r = csv.reader(f)
        h = next(r)
        ix = {k: i for i, k in enumerate(h)}
        for row in r:
            if len(row) != len(h):
                continue
            for c in cols:
                v = row[ix[c]]
                out[c].append(v if c == "phase" else float(v))
    return {c: (np.array(v) if c != "phase" else np.array(v, dtype=object)) for c, v in out.items()}


def cmd_validate(a):
    from fl2g import timecalib_yaml
    g = np.load(a.gnss, allow_pickle=False)
    o = read_obs_epochs(a.obs_epochs)
    with open(a.calib) as f:
        calib = timecalib_yaml.loads(f.read())
    # match by receiver time tag (the C++ t_gpst column is the receiver epoch; both are GPST seconds on the 1970 axis)
    tr = g["t_rx"]
    j = np.searchsorted(tr, o["t_gpst"] - 1e-4)
    j = np.clip(j, 0, len(tr) - 1)
    hit = np.abs(tr[j] - o["t_gpst"]) < 1e-4
    out = {"obs_run": "/".join(os.path.abspath(a.obs_epochs).split("/")[-5:-1]), "obs_epochs": int(len(o["t_gpst"])), "matched_epochs": int(hit.sum()),
           "calib": os.path.basename(a.calib),
           "calib_segment": {k: calib["segments"][0][k] for k in ("offset_s", "drift_ppm", "t_ref_gpst_s")}}
    both = hit & (o["spp_valid"] > 0.5) & g["spp_ok"][j]
    t_true = tr[j] - g["clk_m"][j] / CLIGHT
    pred = np.array([timecalib_yaml.evaluate(calib, x) if np.isfinite(x) else np.nan for x in t_true])
    d = (o["t_est"] - pred)[both]
    out["t_est_residual_s"] = {"n": int(both.sum()), "median": float(np.median(d)), "mad": mad(d), "max_abs": float(np.max(np.abs(d))),
                               "p99_abs": float(np.percentile(np.abs(d), 99))} if d.size else None
    ph = o["phase"]
    for name, sel in (("COLLECT", ph == "COLLECT"), ("COLLECT+ALIGN+TRACKING", np.isin(ph, ["COLLECT", "ALIGN", "TRACKING"]))):
        m = sel & hit
        out["availability_" + name] = {"epochs": int(m.sum()), "cpp_spp_valid": float(np.mean(o["spp_valid"][m] > 0.5)) if m.any() else None,
                                       "py_spp_ok": float(np.mean(g["spp_ok"][j][m])) if m.any() else None,
                                       "py_dls_ok": float(np.mean(g["dls_ok"][j][m])) if m.any() else None,
                                       "both_spp": float(np.mean((o["spp_valid"][m] > 0.5) & g["spp_ok"][j][m])) if m.any() else None}
    print(json.dumps(out, indent=1))
    if a.out:
        with open(a.out, "w") as f:
            json.dump(out, f, indent=1)
    return 0


# ------------------------------------------------------------------------------------------------------------------------
# independent check: the lag between the two speed series by cross-correlation (1 ms steps), same series the DTW sees
# ------------------------------------------------------------------------------------------------------------------------
def cmd_xcorr(a):
    g = np.load(a.gnss, allow_pickle=False)
    gnss = {k: g[k] for k in g.files if k != "meta"}
    state = load_state(a.host_run)
    out = {"seq": a.seq, "modes": {}}
    for mode in ("bridge", "vonly", "track"):
        s = pushable(gnss, state, a.lever, mode, t_from=a.collect_t_rx)
        ent = {"n_samples": int(len(s["t_est"]))}
        whole = xcorr_best_delay(s["t_est"], s["vg"], s["vh"])
        ent["whole"] = {"delay_s": whole[0], "corr": whole[1], "corr_at_zero": whole[2], "dtw_lag_expected_s": -whole[0]} if whole else None
        # windows of a.window_s every a.window_s/2 on the pushable series
        ws = []
        t = s["t_est"]
        t0 = t[0]
        while t0 + a.window_s <= t[-1]:
            m = (t >= t0) & (t < t0 + a.window_s)
            r = xcorr_best_delay(t[m], s["vg"][m], s["vh"][m], search=a.search) if m.sum() > 300 else None
            if r:
                ws.append({"t0": float(t0 - t[0]), "delay_s": r[0], "corr": r[1], "host_std": float(np.std(s["vh"][m]))})
            t0 += a.window_s / 2
        good = [w for w in ws if w["corr"] >= a.min_corr]
        ent["windows"] = {"window_s": a.window_s, "n": len(ws), "n_corr_ge_min": len(good), "min_corr": a.min_corr,
                          "delay_median_s": float(np.median([w["delay_s"] for w in good])) if good else None,
                          "delay_mad_s": mad([w["delay_s"] for w in good]) if good else None,
                          "delay_p10_p90_s": [float(np.percentile([w["delay_s"] for w in good], q)) for q in (10, 90)] if good else None}
        ent["speed_corr_all"] = float(np.corrcoef(s["vg"], s["vh"])[0, 1])
        ent["speed_levels"] = {"host_median": float(np.median(s["vh"])), "host_p95": float(np.percentile(s["vh"], 95)), "gnss_median": float(np.median(s["vg"])),
                               "gnss_p95": float(np.percentile(s["vg"], 95)), "rms_diff": float(np.sqrt(np.mean((s["vg"] - s["vh"]) ** 2)))}
        out["modes"][mode] = ent
    print(json.dumps(out, indent=1))
    if a.out:
        with open(a.out, "w") as f:
            json.dump(out, f, indent=1)
    return 0


# ------------------------------------------------------------------------------------------------------------------------
# when does the motion start, and what do the series look like while the platform stands (the first online window)
# ------------------------------------------------------------------------------------------------------------------------
def cmd_motion(a):
    g = np.load(a.gnss, allow_pickle=False)
    gnss = {k: g[k] for k in g.files if k != "meta"}
    state = load_state(a.host_run)
    s = pushable(gnss, state, a.lever, "bridge", t_from=a.collect_t_rx)
    t = s["t_rx"] - a.collect_t_rx
    moving = s["vh"] >= a.speed
    first = None
    for i in range(len(t)):
        j = np.searchsorted(t, t[i] + a.hold_s)
        if j < len(t) and moving[i:j].all():
            first = float(t[i])
            break
    out = {"seq": a.seq, "pushable": int(len(t)), "span_s": float(t[-1]), "speed_threshold_mps": a.speed, "hold_s": a.hold_s, "first_motion_after_collect_s": first}
    for w in (20.0, 40.0, 60.0, 120.0):
        m = t < w
        out["first_%gs" % w] = {"host_speed_std": float(np.std(s["vh"][m])), "host_speed_max": float(np.max(s["vh"][m])), "gnss_speed_std": float(np.std(s["vg"][m])),
                                "gnss_speed_mean": float(np.mean(s["vg"][m])), "corr": float(np.corrcoef(s["vg"][m], s["vh"][m])[0, 1]) if np.std(s["vg"][m]) > 0 and np.std(s["vh"][m]) > 0 else None}
    still = (s["vh"] < 0.05)
    out["gnss_speed_when_host_below_0.05"] = {"n": int(still.sum()), "mean": float(np.mean(s["vg"][still])) if still.any() else None, "std": float(np.std(s["vg"][still])) if still.any() else None}
    print(json.dumps(out, indent=1))
    if a.out:
        with open(a.out, "w") as f:
            json.dump(out, f, indent=1)
    return 0


# ------------------------------------------------------------------------------------------------------------------------
# host failures: stretches where the U run's speed disagrees with the ground-truth speed (development sequences only, A2)
# ------------------------------------------------------------------------------------------------------------------------
def cmd_hostcheck(a):
    state = load_state(a.host_run)
    gt = np.loadtxt(os.path.join(a.gt_dir, "gt_clean.tum"))
    tg, p = gt[:, 0], gt[:, 1:4]
    v = np.zeros_like(p)
    v[1:-1] = (p[2:] - p[:-2]) / (tg[2:] - tg[:-2])[:, None]          # GT speed from the 1 Hz positions, central differences
    sg = np.linalg.norm(v, axis=1)
    sh = host_speed_at(state, a.lever, tg)
    ok = np.isfinite(sh)
    ok[0] = ok[-1] = False
    d = np.abs(sh - sg)
    idx = np.where(ok & (d > a.thresh))[0]
    intervals = []
    if idx.size:
        for g in np.split(idx, np.where(np.diff(idx) > a.merge)[0] + 1):
            intervals.append([float(tg[g[0]]), float(tg[g[-1]]), int(g.size), float(d[g].max())])
    out = {"seq": a.seq, "threshold_mps": a.thresh, "gt_samples": int(ok.sum()), "median_abs_diff": float(np.median(d[ok])), "p90_abs_diff": float(np.percentile(d[ok], 90)),
           "intervals_t_est": intervals, "host_path_m": float(np.sum(np.linalg.norm(np.diff(state["p"], axis=0), axis=1))),
           "gt_path_m": float(np.sum(np.linalg.norm(np.diff(p, axis=0), axis=1))), "host_t0": float(state["t"][0])}
    print(json.dumps(out, indent=1))
    if a.out:
        with open(a.out, "w") as f:
            json.dump(out, f, indent=1)
    return 0


# ------------------------------------------------------------------------------------------------------------------------
# selection (A81 2 item 1)
# ------------------------------------------------------------------------------------------------------------------------
def load_rows(paths):
    rows = []
    for p in paths:
        with open(p) as f:
            for r in csv.DictReader(f):
                rows.append({"seq": r["seq"], "k": int(r["k"]), "grid_s": float(r["grid_s"]), "window_s": float(r["window_s"]),
                             "ok": r["ok"] == "True", "lag_s": float(r["lag_s"]) if r["lag_s"] != "" else None,
                             "common_start": r["common_start"] == "True", "host_std": float(r["host_std"]) if r.get("host_std") else None,
                             "start_rel_s": float(r["start_rel_s"]), "start_t_rx": float(r["start_t_rx"]), "corr": float(r["corr"]) if r.get("corr") else None,
                             "gnss_std": float(r["gnss_std"]) if r.get("gnss_std") else None,
                             "t_success_rel_s": float(r["t_success_rel_s"]) if r.get("t_success_rel_s") else None})
    return rows


def combo_table(rows, seqs=None, common_only=True):
    """Per (grid, window): pooled MAD of the lags, median, failure share, per-sequence MAD."""
    out = {}
    for g in GRIDS:
        for w in WINDOWS:
            sel = [r for r in rows if r["grid_s"] == g and r["window_s"] == w and (not common_only or r["common_start"]) and (seqs is None or r["seq"] in seqs)]
            lags = np.array([r["lag_s"] for r in sel if r["ok"]])
            ent = {"grid_s": g, "window_s": w, "n_windows": len(sel), "n_ok": int(lags.size), "fail_share": 1.0 - lags.size / max(len(sel), 1),
                   "mad_s": mad(lags) if lags.size else float("nan"), "median_s": float(np.median(lags)) if lags.size else float("nan"),
                   "mean_s": float(np.mean(lags)) if lags.size else float("nan"),
                   "zero_share": float(np.mean(np.abs(lags) < 1e-9)) if lags.size else float("nan"),
                   "abs_gt_0p1_share": float(np.mean(np.abs(lags) > 0.1 + 1e-9)) if lags.size else float("nan"), "per_seq": {}}
            for sq in sorted({r["seq"] for r in sel}):
                l2 = np.array([r["lag_s"] for r in sel if r["seq"] == sq and r["ok"]])
                ent["per_seq"][sq] = {"n": int(l2.size), "mad_s": mad(l2) if l2.size else float("nan"),
                                      "median_s": float(np.median(l2)) if l2.size else float("nan")}
            out[(g, w)] = ent
    return out


def apply_a81_rule(table):
    """A81: the (grid, window) of the smallest pooled MAD; MADs within 5 ms of the smallest -> the shorter window.
    Returns (chosen (grid, window) or None, candidates within 5 ms sorted by window, residual ties)."""
    ok = {k: v for k, v in table.items() if math.isfinite(v["mad_s"])}
    if not ok:
        return None, [], []
    m0 = min(v["mad_s"] for v in ok.values())
    near = sorted([k for k, v in ok.items() if v["mad_s"] - m0 < TIE_S], key=lambda k: (k[1], k[0]))
    shortest = near[0][1]
    ties = [k for k in near if k[1] == shortest]
    # the rule names the window for ties of MAD; several grids at the same window and MAD within 5 ms are a tie the rule does not resolve
    return (ties[0] if len(ties) == 1 else None), near, ties


def cmd_select(a):
    rows = load_rows(a.csv)
    tab = combo_table(rows, common_only=not a.all_starts)
    chosen, near, ties = apply_a81_rule(tab)
    out = {"table": [dict(v, key="%g/%g" % k) for k, v in tab.items()], "chosen": chosen, "within_5ms_of_min": near, "residual_tie_at_shortest_window": ties}
    print(json.dumps(out, indent=1, default=str))
    if a.out:
        with open(a.out, "w") as f:
            json.dump(out, f, indent=1, default=str)
    return 0


# ------------------------------------------------------------------------------------------------------------------------
# report: markdown tables of the scan (docs/dev/A81_DTW_SCAN.md quotes them)
# ------------------------------------------------------------------------------------------------------------------------
SEQ_LABELS = (("d01", "Data01_20240707_100C_CAR_complex", "SP-0707"), ("d02", "Data02_20240718_100C_CAR_complex", "SP-0718"),
              ("mars", "HKisland_GNSS03", "MARS"))
STATIONARY_STD = 0.05           # diagnostic only (m/s): host speed std below it in the window = no motion to align on


def _f(x, nd=3):
    return "-" if x is None or (isinstance(x, float) and not math.isfinite(x)) else ("%.*f" % (nd, x))


def md_table(head, body):
    out = ["| " + " | ".join(head) + " |", "|" + "|".join("---" for _ in head) + "|"]
    for r in body:
        out.append("| " + " | ".join(str(c) for c in r) + " |")
    return "\n".join(out)


def compare_variants(a_rows, b_rows):
    kb = {(r["seq"], r["k"], r["grid_s"], r["window_s"]): r for r in b_rows}
    d = []
    n_both = n_ok_a = 0
    for r in a_rows:
        if not r["common_start"]:
            continue
        o = kb.get((r["seq"], r["k"], r["grid_s"], r["window_s"]))
        if o is None:
            continue
        n_ok_a += 1 if r["ok"] else 0
        if r["ok"] and o["ok"]:
            n_both += 1
            d.append(abs(r["lag_s"] - o["lag_s"]))
    d = np.array(d)
    if not d.size:
        return None
    return {"n": int(d.size), "same": float(np.mean(d < 1e-9)), "median_abs": float(np.median(d)), "p90_abs": float(np.percentile(d, 90)),
            "max_abs": float(np.max(d)), "ge_0p1": float(np.mean(d >= 0.1 - 1e-9))}


def cmd_report(a):
    D_ = a.dir
    rows = {}
    for L, seq, _ in SEQ_LABELS:
        rows[L] = load_rows([os.path.join(D_, "scan_%s.csv" % L)])
    allrows = [r for L, _, _ in SEQ_LABELS for r in rows[L]]
    out = []
    # T1: the pooled table
    tab = combo_table(allrows)
    chosen, near, ties = apply_a81_rule(tab)
    head = ["grid [s]", "window [s]", "min_epochs", "windows", "failed", "MAD [s]", "median [s]", "lag = 0", "abs(lag) > 0.1 s"] + ["MAD / median " + l for _, _, l in SEQ_LABELS]
    body = []
    for (g, w), v in tab.items():
        body.append([g, "%g" % w, int(round(w / 0.1)), v["n_windows"], "%d" % (v["n_windows"] - v["n_ok"]), _f(v["mad_s"]), _f(v["median_s"]), "%.0f %%" % (100 * v["zero_share"]),
                     "%.0f %%" % (100 * v["abs_gt_0p1_share"])] + ["%s / %s" % (_f(v["per_seq"][sq]["mad_s"]), _f(v["per_seq"][sq]["median_s"])) if sq in v["per_seq"] else "-" for _, sq, _ in SEQ_LABELS])
    out.append("### T1 pooled over the three development sequences, common start set (every 60 s; the 120 s window fits)\n\n" + md_table(head, body))
    out.append("\nA81 rule: smallest MAD %s s; within 5 ms of it: %s; chosen: %s" % (_f(min(v["mad_s"] for v in tab.values())), near, chosen))
    # the same for every feasible start of each window
    tab2 = combo_table(allrows, common_only=False)
    c2, near2, ties2 = apply_a81_rule(tab2)
    out.append("\nSame rule on every feasible start of each window: chosen %s (within 5 ms: %s)" % (c2, near2))
    # T1c: windows that do not overlap a host failure (diagnostic)
    bad, t0s = {}, {}
    for L, sq, lab in SEQ_LABELS:
        path = os.path.join(D_, "hostcheck_%s.json" % L)
        if os.path.exists(path):
            with open(path) as f:
                hj = json.load(f)
            bad[sq] = hj["intervals_t_est"]
            t0s[sq] = hj["host_t0"]
    if bad:
        keep = []
        n_drop = 0
        for L, sq, lab in SEQ_LABELS:
            iv = bad.get(sq, [])
            for r in rows[L]:
                w0 = r["start_t_rx"] - LEAP_S
                w1 = w0 + (r["t_success_rel_s"] if r["t_success_rel_s"] is not None else r["window_s"])
                if any(w0 <= b_[1] + 1.0 and w1 >= b_[0] - 1.0 for b_ in iv):
                    n_drop += 1
                    continue
                keep.append(r)
        t1c = combo_table(keep)
        c1c, near1c, _ = apply_a81_rule(t1c)
        head = ["grid [s]", "window [s]", "windows", "MAD [s]", "median [s]", "lag = 0", "abs(lag) > 0.1 s"] + ["MAD / median " + l for _, _, l in SEQ_LABELS]
        body = []
        for (g, w), v in t1c.items():
            body.append([g, "%g" % w, v["n_windows"], _f(v["mad_s"]), _f(v["median_s"]), "%.0f %%" % (100 * v["zero_share"]), "%.0f %%" % (100 * v["abs_gt_0p1_share"])] +
                        ["%s / %s" % (_f(v["per_seq"][sq]["mad_s"]), _f(v["per_seq"][sq]["median_s"])) if sq in v["per_seq"] else "-" for _, sq, _ in SEQ_LABELS])
        desc = "; ".join("%s: %s" % (lab, ", ".join("%.0f-%.0f s after the host start" % (b_[0] - t0s[sq], b_[1] - t0s[sq]) for b_ in bad.get(sq, [])) or "none")
                         for _, sq, lab in SEQ_LABELS if sq in bad)
        out.append("\n### T1c diagnostic: without the windows that overlap a host failure (host speed vs GT speed > 2 m/s; %s; %d window rows dropped over all combinations)\n\n" % (desc, n_drop)
                   + md_table(head, body) + "\n\nrule on these windows: chosen %s, within 5 ms %s" % (c1c, near1c))
    # T1b: moving windows only (diagnostic)
    mv = [r for r in allrows if r["ok"] and r["host_std"] is not None and r["host_std"] >= STATIONARY_STD]
    head = ["grid [s]", "window [s]", "windows ok", "moving", "stationary", "MAD moving [s]", "median moving [s]", "abs(lag) > 0.1 s moving", "MAD stationary [s]"]
    body = []
    for g in GRIDS:
        for w in WINDOWS:
            sel = [r for r in allrows if r["grid_s"] == g and r["window_s"] == w and r["common_start"] and r["ok"]]
            m1 = np.array([r["lag_s"] for r in sel if r["host_std"] is not None and r["host_std"] >= STATIONARY_STD])
            m0 = np.array([r["lag_s"] for r in sel if r["host_std"] is not None and r["host_std"] < STATIONARY_STD])
            body.append([g, "%g" % w, len(sel), m1.size, m0.size, _f(mad(m1)) if m1.size else "-", _f(float(np.median(m1))) if m1.size else "-",
                         "%.0f %%" % (100 * np.mean(np.abs(m1) > 0.1 + 1e-9)) if m1.size else "-", _f(mad(m0)) if m0.size else "-"])
    out.append("\n### T1b diagnostic split by the host speed std in the window (%.2f m/s; not part of the rule)\n\n" % STATIONARY_STD + md_table(head, body))
    # T2: first window (the one the online run uses)
    head = ["grid [s]", "window [s]"] + ["lag %s [s]" % l for _, _, l in SEQ_LABELS]
    body = []
    for g in GRIDS:
        for w in WINDOWS:
            cells = []
            for L, sq, _ in SEQ_LABELS:
                r = [x for x in rows[L] if x["k"] == 0 and x["grid_s"] == g and x["window_s"] == w]
                cells.append(_f(r[0]["lag_s"], 2) if r and r[0]["ok"] else ("fail" if r else "-"))
            body.append([g, "%g" % w] + cells)
    out.append("\n### T2 first window after the start of COLLECT (k = 0: the one the online run would apply)\n\n" + md_table(head, body))
    if chosen:
        out.append("\nFirst window at the chosen combination %s: fires after [s of COLLECT] -> %s" % (chosen, "; ".join(
            "%s %s s" % (lab, _f([x for x in rows[L] if x["k"] == 0 and (x["grid_s"], x["window_s"]) == chosen][0]["t_success_rel_s"], 1)) for L, _, lab in SEQ_LABELS)))
    head = ["sequence", "host speed std in the first 40 s [m/s]", "GNSS speed std [m/s]", "speed correlation", "windows k = 0..3: host std [m/s]"]
    body = []
    for L, sq, lab in SEQ_LABELS:
        r0 = [x for x in rows[L] if x["k"] == 0 and x["grid_s"] == 0.1 and x["window_s"] == 40.0][0]
        ks = [x for x in rows[L] if x["k"] <= 3 and x["grid_s"] == 0.1 and x["window_s"] == 40.0]
        body.append([lab, _f(r0["host_std"], 3), _f(r0["gnss_std"], 3), _f(r0["corr"], 2), ", ".join(_f(x["host_std"], 3) for x in ks)])
    out.append("\n" + md_table(head, body))
    head = ["sequence", "first sustained motion after COLLECT start [s]", "host speed std / GNSS speed std in the first 40 s [m/s]", "speed corr in the first 40 s", "GNSS speed mean while host < 0.05 m/s [m/s]"]
    body = []
    for L, sq, lab in SEQ_LABELS:
        path = os.path.join(D_, "motion_%s.json" % L)
        if not os.path.exists(path):
            continue
        with open(path) as f:
            m = json.load(f)
        f40 = m["first_40s"]
        body.append([lab, _f(m["first_motion_after_collect_s"], 1), "%s / %s" % (_f(f40["host_speed_std"], 3), _f(f40["gnss_speed_std"], 3)), _f(f40["corr"], 2),
                     _f(m["gnss_speed_when_host_below_0.05"]["mean"], 3)])
    if body:
        out.append("\n" + md_table(head, body))
    # T3: variants
    var = []
    for L, sq, lab in SEQ_LABELS:
        for suffix, name in (("_vonly", "host z = v (no lever term)"), ("_track", "host z = d/dt(p + R l)"), ("m", "GNSS masks C/N0 >= 20, el >= 10 deg")):
            path = os.path.join(D_, "scan_%s%s.csv" % (L, suffix))
            if os.path.exists(path):
                var.append((lab, name, load_rows([path]), rows[L]))
    if var:
        head = ["sequence", "variant", "windows compared", "identical lag", "median abs diff [s]", "p90 abs diff [s]", "max abs diff [s]", "diff >= 0.1 s"]
        body = []
        for lab, name, vr, base in var:
            c = compare_variants(base, vr)
            if c:
                body.append([lab, name, c["n"], "%.0f %%" % (100 * c["same"]), _f(c["median_abs"]), _f(c["p90_abs"]), _f(c["max_abs"]), "%.1f %%" % (100 * c["ge_0p1"])])
        out.append("\n### T3 input sensitivity: lags with a changed input against the primary input, same windows\n\n" + md_table(head, body))
        # the pooled table and the rule under each variant (a sequence without that variant file keeps its primary rows)
        variants = [("primary inputs", "")] + [(n_, sfx) for sfx, n_ in (("_vonly", "host z = v"), ("_track", "host z = d/dt(p + R l)"), ("m", "GNSS masks 20 / 10"))]
        mad_body, tail_body, rule_body = [], [], []
        for name, suffix in variants:
            rr = []
            for L, sq, lab in SEQ_LABELS:
                path = os.path.join(D_, "scan_%s%s.csv" % (L, suffix))
                rr += load_rows([path]) if (suffix and os.path.exists(path)) else rows[L]
            t3 = combo_table(rr)
            ch, nr, _ = apply_a81_rule(t3)
            mad_body.append([name] + [_f(t3[(g, w)]["mad_s"], 3) for g in GRIDS for w in WINDOWS])
            tail_body.append([name] + ["%.0f" % (100 * t3[(g, w)]["abs_gt_0p1_share"]) for g in GRIDS for w in WINDOWS])
            rule_body.append([name, ch, nr])
        head = ["variant"] + ["%g/%g" % (g, w) for g in GRIDS for w in WINDOWS]
        out.append("\nPooled MAD [s] per (grid/window) under each input variant:\n\n" + md_table(head, mad_body))
        out.append("\nShare of windows with abs(lag) > 0.1 s [%]:\n\n" + md_table(head, tail_body))
        out.append("\nA81 rule under each variant:\n\n" + md_table(["variant", "chosen", "within 5 ms of the smallest MAD"], rule_body))
    # T4: xcorr
    head = ["sequence", "whole series: delay [s] / corr", "windows 60 s with corr >= 0.9: n / median delay [s] / MAD [s]", "speed correlation over all samples", "host / GNSS median speed [m/s]"]
    body = []
    for L, sq, lab in SEQ_LABELS:
        path = os.path.join(D_, "xcorr_%s.json" % L)
        if not os.path.exists(path):
            continue
        with open(path) as f:
            x = json.load(f)["modes"]["bridge"]
        w = x["windows"]
        body.append([lab, "%s / %s" % (_f(x["whole"]["delay_s"], 3), _f(x["whole"]["corr"], 3)), "%d of %d / %s / %s" % (w["n_corr_ge_min"], w["n"], _f(w["delay_median_s"], 3), _f(w["delay_mad_s"], 3)),
                     _f(x["speed_corr_all"], 3), "%s / %s" % (_f(x["speed_levels"]["host_median"], 2), _f(x["speed_levels"]["gnss_median"], 2))])
    out.append("\n### T4 cross-correlation of the two series (1 ms steps): GNSS delayed by D <=> DTW lag -D\n\n" + md_table(head, body))
    # T5: validation of the time axis
    head = ["sequence", "OBSERVE run", "t_est residual median / max [us]", "COLLECT epochs: C++ SPP valid / python SPP ok / python Doppler LS ok"]
    body = []
    for L, sq, lab in SEQ_LABELS:
        path = os.path.join(D_, "validate_%s.json" % L)
        if not os.path.exists(path):
            continue
        with open(path) as f:
            v = json.load(f)
        r = v["t_est_residual_s"]
        c = v["availability_COLLECT"]
        body.append([lab, v.get("obs_run", "-"), "%.2f / %.1f" % (r["median"] * 1e6, r["max_abs"] * 1e6), "%s / %s / %s" % (_f(c["cpp_spp_valid"], 3), _f(c["py_spp_ok"], 3), _f(c["py_dls_ok"], 3))])
    out.append("\n### T5 rebuilt time axis and availability against an OBSERVE run of the C++\n\n" + md_table(head, body))
    # T6: per-sequence sample counts at the chosen combo
    if chosen:
        head = ["sequence", "windows", "lags (s) at the chosen (grid, window), in start order (F = no full window)"]
        body = []
        for L, sq, lab in SEQ_LABELS:
            rr = sorted([r for r in rows[L] if (r["grid_s"], r["window_s"]) == chosen], key=lambda r: r["k"])
            body.append([lab, len(rr), " ".join("F" if not r["ok"] else "%.1f" % r["lag_s"] for r in rr)])
        out.append("\n### T6 lags at the chosen combination %s\n\n" % (chosen,) + md_table(head, body))
    txt = "\n".join(out)
    print(txt)
    if a.out:
        with open(a.out, "w") as f:
            f.write(txt + "\n")
    return 0


# ------------------------------------------------------------------------------------------------------------------------
# comparison with the C++ results
# ------------------------------------------------------------------------------------------------------------------------
def cmd_compare(a):
    """dump (python calls with their results) vs the C++ results file 'RES <call index> <ok> <lag hex>'."""
    py = []
    with open(a.dump) as f:
        for ln in f:
            if ln.startswith("CALL "):
                _, sid, g, w, b, ok, lag = ln.split()
                py.append((int(sid), float.fromhex(g), float.fromhex(w), int(ok), float.fromhex(lag) if lag != "-" else None))
    cpp = []
    with open(a.cpp) as f:
        for ln in f:
            if ln.startswith("RES "):
                _, idx, ok, lag = ln.split()
                cpp.append((int(idx), int(ok), float.fromhex(lag) if lag != "-" else None))
    if len(py) != len(cpp):
        print("MISMATCH in count: python %d, C++ %d" % (len(py), len(cpp)))
        return 1
    n_bad_ok = n_bit = n_tol = 0
    worst = 0.0
    bad = []
    for i, ((sid, g, w, ok, lag), (idx, ok2, lag2)) in enumerate(zip(py, cpp)):
        if idx != i or ok != ok2:
            n_bad_ok += 1
            bad.append((i, "ok %d vs %d" % (ok, ok2)))
            continue
        if ok:
            d = abs(lag - lag2)
            worst = max(worst, d)
            if lag == lag2:
                n_bit += 1
            elif d <= 1e-9:
                n_tol += 1
            else:
                bad.append((i, "lag %.17g vs %.17g" % (lag, lag2)))
        else:
            n_bit += 1
    summary = {"calls": len(py), "bit_identical": n_bit, "within_1e-9": n_tol, "ok_flag_mismatch": n_bad_ok, "other_mismatch": len(bad) - n_bad_ok,
               "worst_abs_diff_s": worst, "first_bad": bad[:10]}
    print(json.dumps(summary, indent=1))
    return 0 if not bad else 1


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    sp = p.add_subparsers(dest="cmd", required=True)
    e = sp.add_parser("extract-gnss", help="bag + RINEX nav -> npz of per-epoch Doppler-LS speed and the time of each epoch")
    e.add_argument("--bag", required=True)
    e.add_argument("--nav", required=True)
    e.add_argument("--out", required=True)
    e.add_argument("--cn0-min", type=float, default=35.0)
    e.add_argument("--el-min", type=float, default=15.0)
    e.set_defaults(fn=cmd_extract_gnss)
    s = sp.add_parser("scan", help="scan one sequence: every 60 s a start, every (grid, window) a one-shot lag")
    s.add_argument("--seq", required=True)
    s.add_argument("--gnss", required=True, help="npz of extract-gnss")
    s.add_argument("--host-run", required=True, help="U run directory (state_trace.txt)")
    s.add_argument("--lever", type=float, nargs=3, required=True, help="antenna in the IMU frame [m] (gnss.frames.lever_arm_xyz)")
    s.add_argument("--host-mode", default="bridge", choices=["bridge", "vonly", "track"])
    s.add_argument("--collect-t-rx", type=float, required=True, help="receiver time of the first COLLECT epoch (OBSERVE run gnss_epochs.csv)")
    s.add_argument("--epoch-dt", type=float, default=0.1, help="receiver epoch interval [s]: dtw_min_epochs = window / this")
    s.add_argument("--out", required=True)
    s.add_argument("--dump", default="")
    s.set_defaults(fn=cmd_scan)
    c = sp.add_parser("select", help="A81 rule over the scan CSVs")
    c.add_argument("csv", nargs="+")
    c.add_argument("--all-starts", action="store_true", help="every feasible start of each window (default: the common start set)")
    c.add_argument("--out", default="")
    c.set_defaults(fn=cmd_select)
    v = sp.add_parser("validate", help="rebuilt t_est and epoch availability against an OBSERVE run's gnss_epochs.csv")
    v.add_argument("--gnss", required=True)
    v.add_argument("--obs-epochs", required=True)
    v.add_argument("--calib", required=True, help="the A18 file that run used (its t_est carries that map)")
    v.add_argument("--out", default="")
    v.set_defaults(fn=cmd_validate)
    x = sp.add_parser("xcorr", help="independent lag of the two speed series by cross-correlation, three host-velocity variants")
    x.add_argument("--seq", required=True)
    x.add_argument("--gnss", required=True)
    x.add_argument("--host-run", required=True)
    x.add_argument("--lever", type=float, nargs=3, required=True)
    x.add_argument("--collect-t-rx", type=float, required=True)
    x.add_argument("--window-s", type=float, default=60.0)
    x.add_argument("--search", type=float, default=1.0)
    x.add_argument("--min-corr", type=float, default=0.9)
    x.add_argument("--out", default="")
    x.set_defaults(fn=cmd_xcorr)
    rp = sp.add_parser("report", help="markdown tables from scan_*.csv, xcorr_*.json, validate_*.json of a directory")
    rp.add_argument("--dir", required=True)
    rp.add_argument("--out", default="")
    rp.set_defaults(fn=cmd_report)
    mo = sp.add_parser("motion", help="time of the first sustained motion after COLLECT starts, series statistics of the first windows")
    mo.add_argument("--seq", required=True)
    mo.add_argument("--gnss", required=True)
    mo.add_argument("--host-run", required=True)
    mo.add_argument("--lever", type=float, nargs=3, required=True)
    mo.add_argument("--collect-t-rx", type=float, required=True)
    mo.add_argument("--speed", type=float, default=0.5)
    mo.add_argument("--hold-s", type=float, default=5.0)
    mo.add_argument("--out", default="")
    mo.set_defaults(fn=cmd_motion)
    hk = sp.add_parser("hostcheck", help="stretches where the host speed disagrees with the ground-truth speed (development sequences)")
    hk.add_argument("--seq", required=True)
    hk.add_argument("--host-run", required=True)
    hk.add_argument("--gt-dir", required=True)
    hk.add_argument("--lever", type=float, nargs=3, required=True)
    hk.add_argument("--thresh", type=float, default=2.0, help="|host speed - GT speed| [m/s] that counts as a failure")
    hk.add_argument("--merge", type=int, default=5, help="GT samples (1 Hz) within which two failing samples form one interval")
    hk.add_argument("--out", default="")
    hk.set_defaults(fn=cmd_hostcheck)
    y = sp.add_parser("synth", help="write the synthetic cases (python results included) in the dump format of dtw_align.cpp")
    y.add_argument("--dump", required=True)
    y.add_argument("--seed", type=int, default=0)
    y.set_defaults(fn=cmd_synth)
    m = sp.add_parser("compare", help="python vs C++ lags on the same series")
    m.add_argument("--dump", required=True)
    m.add_argument("--cpp", required=True)
    m.set_defaults(fn=cmd_compare)
    a = p.parse_args(argv)
    return a.fn(a)


if __name__ == "__main__":
    sys.exit(main())
