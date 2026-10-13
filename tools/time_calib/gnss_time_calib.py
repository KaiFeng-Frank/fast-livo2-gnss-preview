#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Per-sequence offline GNSS↔estimator time calibration (DESIGN §7): offset, drift_ppm, sigma_toff.

Inputs: the reference commit R's GNSS-free deterministic replay (traj_W / state_trace) and the bag's GNSS topics.
1) record-time statistics (range_meas record time − GPST→UTC; LiDAR/IMU/image header − record time; step detection),
   combined into a prior of the estimator-clock offset;
2) main method: horizontal |v_W(t)| vs horizontal |v_GNSS| from the Doppler LS, resampled at 100 Hz, normalised
   cross-correlation over offset0 ± 1 s with parabolic refinement, 60 s sliding windows → offset per segment (drift
   from the method-1 prior; a free line (offset, drift_ppm) only without one), sigma from the window scatter / peak
   width / bootstrap; 3) DTW cross-check (warn if > 3 sigma apart); 4) quality checks.
Output: time_calib/<seq>.yaml (AMENDMENTS A18). Owner: WP-g.

Details fixed here (DESIGN §1/§7, AMENDMENTS A18):
  * mapping: t_est = t_gpst_true - leap + offset + drift_ppm*1e-6*(t_gpst - t_ref_gpst); every *_gpst_s value is
    GTime::toSec(): seconds since 1970-01-01 on the GPST scale, no leap seconds (315964800 + week*604800 + tow);
    the tool estimates offset at t_ref = the median GPST of the usable windows of a segment;
  * method 1 (record_time_stats, estimator_clock_prior): the bag record times are host-clock times, the estimator
    runs on the Livox header clock (S1/C4 of the audit: header = device time + constant). The GNSS record offset
    (host clock + GNSS transport latency) is moved onto the header clock with the Avia IMU's record − header at the
    same host instant, both at their lower envelope (transport jitter only adds delay):
        offset = [record_g − utc_g]_low − [record_imu − header_imu]_low − latency_g,min + latency_imu,min .
    A host-clock step moves both brackets and cancels; a header-clock step stays (the only kind that can split the
    estimator-time segments). The GNSS transport latency is not observable from record times; it is taken as
    uniform on [0, LAT_G_MAX] (推断: u-blox RAWX output within one 100 ms epoch + serial + driver), i.e. the prior
    is the upper bound minus LAT_G_MAX/2 with sigma LAT_G_MAX/2;
  * method 2 compares the *antenna* speed on both sides (method 2 signal, tool_version 3): the horizontal part of
    d/dt (p + R l) from traj_W (TUM, estimator time; "up" in W = minus the replay's own gravity estimate from
    state_trace.txt) and the horizontal (E, N) part of the Doppler-LS antenna velocity (GNSS epochs, GPST), each
    VECTOR smoothed with the same zero-phase Gaussian (SMOOTH_S = 0.25 s) on a 100 Hz grid before its norm is taken;
    norms are frame-free, so no registration is needed. The GNSS velocity comes from doppler_velocity_series (a
    weighted two-pass Doppler LS whose second pass fixes the smoothed clock drift). Why (development set, AMENDMENTS
    P4 record): the tool_version-2 signal — the 3-D norm of the per-epoch C/N0 >= 25 Doppler LS, norms smoothed —
    was dominated by the LS's own noise, not by the motion: on the grass field only near-zenith satellites reach
    25 dB-Hz, so vertical velocity and clock drift are nearly collinear (Doppler-LS vertical noise 1.6 m/s on Grass02,
    0.6 m/s on Grass01, against 0.06 m/s of real vertical motion), and the norm of a noisy vector is biased up by the
    noise (Grass02 |v| mean 1.36 m/s against 0.75). High-C/N0 rows agree with the replay's antenna velocity to
    0.10-0.11 m/s (robust, per row) on both sequences: there is no high-frequency antenna motion to blame. The
    platform's speed lives below ~0.2 Hz (94-96 % of its power), the Doppler-LS noise is white to the 5 Hz Nyquist
    frequency, hence a Gaussian of 0.25 s (amplitude -3 dB at 0.53 Hz) applied identically to both sides (zero phase:
    no lag bias). The per-window scatter is flat between 0.25 and 0.5 s (Grass01 10.5 / 12.2 ms) and grows with
    heavier smoothing (1 s: 23.7 ms), which widens the correlation peak;
  * windows need MIN_COVERAGE = 90 % GNSS data (grid samples within MAX_GAP_S = 0.25 s of a Doppler solution): a
    series with gaps (Occlusion03: 264 solvable epochs in runs of <= 6 s) is never correlated across its gaps;
  * windows: `--window` s long, `--step` s apart; a window is used when both speed series have std >= 0.1 m/s, the
    correlation peak is > 0.5 and not at the search edge. Each used window carries a peak-width precision
    (robust.tde_sigma: peak height + curvature width + duration, "σ_toff 取峰宽");
  * segments (A18 "台阶(>50 ms 持续)= 新段"): a jump from the running level larger than 50 ms AND than 3 sigma of the
    difference (the windows' peak-width precisions, split_groups/_tol; with 10-60 ms per window a bare 50 ms rule
    would cut segments on noise) starts a new segment only if it persists for >= MIN_WINDOWS consecutive windows, or
    coincides with a method-1 step of the estimator-clock prior (same sign, size within 50 ms), or runs to the end of
    the data (nothing after it restores the old level); otherwise the jumped windows are isolated outliers and dropped;
  * drift: a segment's drift is the method-1 prior's (the record-time series measures it to ~1 ppm; the windows,
    10-60 ms each, would fit noise as slope: Grass02 -943 ppm from 3 windows) and the windows fit only its level;
    without a method-1 prior the windows fit offset and drift (the tool_version-2 rule);
  * sigma_toff of a segment with >= MIN_WINDOWS windows = max(1 ms, window scatter about the line (dof-corrected),
    bootstrap s.d. of the line at t_ref, peak-width precision / sqrt(independent windows)); a segment with fewer
    windows cannot measure its own scatter: sigma = max(peak-width precision of one window, 50 ms). Window scatter
    > 5 ms is flagged;
  * method 3: banded DTW (±2 s) of the two speed profiles at 10 Hz after applying the method-2 offset; the median
    path offset is the DTW correction; |DTW - xcorr| > 3 sigma is reported as a warning (never used as the value);
  * what goes into the file (choose_segments, AMENDMENTS A53 "方法②可用时取其 σ_toff(更小)"): over every time span
    the more precise of method 2 and the method-1 prior — a method-2 segment whose sigma exceeds the prior's
    (LAT_G_MAX/2) is replaced over its span by the prior; no method-2 segment at all (no replay, or no usable
    window: insufficient excitation) ⇒ the method-1 prior (method `record_time`). Only when there is no method-1
    prior either (no IMU header clock) does the A18 method `config` keep the values given with --config-offset/
    --config-drift-ppm/--config-t-ref (sigma 50 ms); without them nothing is written (blocked);
  * TDCP (A53, tdcp_paused_for): the TDCP R already carries (|dv_pair| sigma_toff)^2 (DESIGN §4.7), so a time offset
    only hurts TDCP through the velocity change inside a pair and that is accounted for: tdcp_paused = false whenever
    the segment comes from method 1 or 2 with sigma_toff <= 0.1 s; paused only when both methods failed (method
    `config`, or a method-2 segment coarser than 0.1 s with no method-1 prior to fall back on).
"""
import argparse
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np  # noqa: E402

from fl2g import TOOL_VERSION, geodesy, gnss_msgs, gnsslsq, gnsstime, orbits, report, rinexnav, robust, rosio  # noqa: E402
from fl2g import timecalib_yaml  # noqa: E402
from fl2g import obs as obs_mod  # noqa: E402
from fl2g.replay import load_state_trace  # noqa: E402
from fl2g.sats import CLIGHT  # noqa: E402

DEFAULT_LEVER_AVIA = (-0.31635, 0.10926, 0.4006)      # l^I for the Avia IMU (DESIGN §14, A15; tools/m3dgr_calib.py)
MAX_EPH_AGE = {"G": 7200.0, "E": 7200.0, "C": 3600.0, "R": 1800.0, "J": 7200.0}
MIN_SPEED_STD = 0.1
STEP_S = 0.050                 # A18: a persistent step > 50 ms starts a new segment
MIN_WINDOWS = 3                # windows a segment needs to measure its own scatter / a jump to count as persistent
SIGMA_FALLBACK = 0.05          # A18 method config; DESIGN §7 insufficient excitation; floor of short segments
LAT_G_MAX = 0.150              # assumed upper bound of the GNSS measurement transport latency [s] (推断)
TDCP_SIGMA_MAX = 0.100         # A53: TDCP runs when method 1 or 2 gives sigma_toff <= 0.1 s
CALIBRATING_METHODS = ("xcorr", "record_time")    # method 2, method 1 (A53); `config` is neither
# method-2 signal (tool_version 3, module docstring "method 2 signal")
SMOOTH_S = 0.25                # zero-phase Gaussian on both velocity vectors before the horizontal norm
FS_HZ = 100.0                  # common resampling rate of both speed series
MAX_GAP_S = 0.25               # a GNSS grid sample counts as data within this distance of a Doppler solution
MIN_COVERAGE = 0.9             # fraction of a window that must be GNSS data
CN0_MIN_DOPP = 15.0            # Doppler rows of the velocity LS: C/N0 >= 15 dB-Hz ...
EL_MIN_DOPP = 10.0             # ... elevation >= 10 deg, weighted by lambda x the receiver's dopp_std
DRIFT_MEDIAN_HALF_S = 1.0      # clock drift: running median over +-1 s of the pass-1 solutions ...
DRIFT_SMOOTH_S = 1.0           # ... then a Gaussian of 1 s; pass 2 solves the velocity with that drift fixed
IMU_TOPIC = "/livox/avia/imu"
ENVELOPE_Q = 5.0               # lower-envelope percentile of the record-time offsets
ENVELOPE_HALF_S = 5.0          # half window of the running envelope [s]
EXIT_BLOCKED = 3


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0], formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("bag", help="input bag")
    p.add_argument("replay_dir", help="output directory of the R replay (traj_W.tum / state_trace.txt); '-' = none (method 1 only)")
    p.add_argument("out_yaml", help="time_calib/<seq>.yaml")
    p.add_argument("--offset0", type=float, default=None, help="search centre [s] (default: the method-1 estimator-clock prior)")
    p.add_argument("--window", type=float, default=60.0, help="sliding window length [s]")
    p.add_argument("--step", type=float, default=30.0, help="window step [s]")
    p.add_argument("--plot", action="store_true", help="write diagnostic plots next to out_yaml")
    p.add_argument("--seq", default="")
    p.add_argument("--nav-rinex", action="append", default=[], help="RINEX NAV merged with the bag ephemerides (A1)")
    p.add_argument("--lever-arm", type=float, nargs=3, default=DEFAULT_LEVER_AVIA, help="antenna in the IMU frame [m]")
    p.add_argument("--approx-xyz", type=float, nargs=3, default=None, metavar=("X", "Y", "Z"),
                   help="ECEF [m] of a place near the track (a few km are enough): bags without a receiver_pvt topic (A80 "
                        "SmartPNT/MARS slim bags) get one fixed-position PVT entry per epoch for the Doppler LS (it only needs the "
                        "local ENU frame and the line-of-sight geometry)")
    p.add_argument("--approx-from-gt", default="", metavar="GT_DIR",
                   help="like --approx-xyz with the ENU origin of GT_DIR/gt_audit.json (a location only, no trajectory is read)")
    p.add_argument("--traj", default="", help="trajectory file (default <replay_dir>/traj_W.tum)")
    p.add_argument("--max-window-sigma-s", type=float, default=None,
                   help="method-2 windows whose own peak-width precision is worse than this [s] are not used; off by default")
    p.add_argument("--level-ratio-max", type=float, default=None,
                   help="method-2 windows whose mean estimator speed and mean GNSS speed differ by more than this factor are not used "
                        "(the estimator is not following the motion: its correlation peak is no clock offset); off by default")
    p.add_argument("--report", default="", help="JSON report path (default next to out_yaml)")
    p.add_argument("--imu-topic", default=IMU_TOPIC, help="IMU on the estimator (header) clock for the method-1 prior")
    p.add_argument("--config-offset", type=float, default=None,
                   help="gnss.time.offset_s of the sequence config: kept (A18 method config) when method 2 finds no usable window")
    p.add_argument("--config-drift-ppm", type=float, default=0.0, help="gnss.time.drift_ppm of the sequence config")
    p.add_argument("--config-t-ref", type=float, default=0.0, help="gnss.time.t_ref_gpst_s of the sequence config")
    return p


# ------------------------------------------------------------------------------------------------------------
# method 1: record-time statistics
# ------------------------------------------------------------------------------------------------------------
def record_time_stats(t_gpst, t_record, extra_topics=None):
    """Host-clock view: record - GPST->UTC [s] of the GNSS epochs (host clock offset + GNSS transport latency) with a
    robust line and steps (> 50 ms persistent over 20 s)."""
    t_gpst = np.asarray(t_gpst, dtype=float)
    t_record = np.asarray(t_record, dtype=float)
    tu = np.array([gnsstime.gpst_to_utc(t) for t in t_gpst])
    d = t_record - tu
    slope, icpt = robust.theil_sen(tu - tu[0], d, max_pairs=100000)
    steps = robust.detect_steps(tu - tu[0], d, STEP_S, 20.0)
    out = {"n": int(d.size), "median_s": float(np.median(d)), "mad_s": robust.mad(d), "p5_s": float(np.percentile(d, 5)),
           "p95_s": float(np.percentile(d, 95)), "min_s": float(d.min()), "max_s": float(d.max()),
           "drift_ppm": slope * 1e6, "steps": [(s[0], s[1]) for s in steps], "leap_s": gnsstime.leap_at_gpst(t_gpst[0])}
    if extra_topics:
        out["header_minus_record_ms"] = extra_topics
    return out


def estimator_clock_prior(t_gpst, t_record, imu_header, imu_record, lat_g_max=LAT_G_MAX):
    """Method-1 prior of the estimator-clock offset (module docstring): upper bound series
    p(t) = [record_g - utc_g]_low - [record_imu - header_imu]_low at each GNSS epoch, its Theil-Sen line, and its
    persistent steps (header-clock steps; host-clock steps cancel). Returns None without IMU samples."""
    t_gpst = np.asarray(t_gpst, dtype=float)
    t_record = np.asarray(t_record, dtype=float)
    imu_header = np.asarray(imu_header, dtype=float)
    imu_record = np.asarray(imu_record, dtype=float)
    if t_gpst.size < 20 or imu_header.size < 20:
        return None
    tu = np.array([gnsstime.gpst_to_utc(t) for t in t_gpst])
    m1 = t_record - tu
    grid = np.arange(t_record.min(), t_record.max() + 1.0, 1.0)
    g_low = robust.running_percentile(t_record, m1, grid, ENVELOPE_HALF_S, ENVELOPE_Q)
    i_low = robust.running_percentile(imu_record, imu_record - imu_header, grid, ENVELOPE_HALF_S, ENVELOPE_Q)
    ok = np.isfinite(g_low) & np.isfinite(i_low)
    if ok.sum() < 2:
        return None
    ub = np.interp(t_record, grid[ok], g_low[ok] - i_low[ok])
    t_ref = float(np.median(t_gpst))
    slope, icpt = robust.theil_sen(t_gpst - t_ref, ub, max_pairs=100000)
    steps = robust.detect_steps(t_gpst - t_gpst[0], ub, STEP_S, 20.0)
    return {"n_epochs": int(t_gpst.size), "n_imu": int(imu_header.size), "t_ref_gpst_s": t_ref,
            "upper_bound_s": float(icpt), "upper_bound_median_s": float(np.median(ub)), "drift_ppm": slope * 1e6,
            "latency_g_assumed_range_s": [0.0, lat_g_max], "offset_s": float(icpt) - 0.5 * lat_g_max, "sigma_s": 0.5 * lat_g_max,
            "gnss_record_low_median_s": float(np.nanmedian(g_low)), "imu_record_minus_header_low_median_s": float(np.nanmedian(i_low)),
            "steps": [(float(t_gpst[0] + s[0]), float(s[1])) for s in steps], "series": (t_gpst, ub)}


def tdcp_paused_for(method, sigma_toff_s):
    """AMENDMENTS A53: TDCP is paused only when neither method 1 (record_time) nor method 2 (xcorr) calibrated the
    segment to sigma_toff <= 0.1 s. The TDCP R carries (|dv_pair| sigma_toff)^2 (DESIGN §4.7), so an honest sigma of
    that size is accounted for; method `config` (the sequence config's values kept) is not a calibration."""
    return not (method in CALIBRATING_METHODS and float(sigma_toff_s) <= TDCP_SIGMA_MAX)


def prior_segments(prior):
    """A18 segments from the method-1 prior (split at its steps): method record_time, sigma LAT_G_MAX/2 (75 ms), so
    TDCP runs (A53). Each piece gets its own Theil-Sen line of the upper-bound series."""
    tg, ub = prior["series"]
    cuts = [s[0] for s in prior["steps"]]
    edges = [-np.inf] + cuts + [np.inf]
    segs = []
    for a, b in zip(edges[:-1], edges[1:]):
        m = (tg >= a) & (tg < b)
        if m.sum() < 2:
            continue
        t_ref = float(np.median(tg[m]))
        slope, icpt = robust.theil_sen(tg[m] - t_ref, ub[m], max_pairs=100000)
        sigma = 0.5 * LAT_G_MAX
        segs.append({"t_from_gpst_s": 0.0 if not segs else float(a), "offset_s": float(icpt) - 0.5 * LAT_G_MAX,
                     "drift_ppm": slope * 1e6, "t_ref_gpst_s": t_ref, "sigma_toff_s": sigma,
                     "tdcp_paused": tdcp_paused_for("record_time", sigma), "method": "record_time"})
    return segs


# ------------------------------------------------------------------------------------------------------------
# method 2: speed cross-correlation
# ------------------------------------------------------------------------------------------------------------
def quat_to_R(qx, qy, qz, qw):
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    return np.array([[1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
                     [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
                     [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)]])


def load_traj(path):
    """TUM traj_W (t x y z qx qy qz qw) or state_trace.txt (hexfloat: cycle flag t_k R(9) p(3) v(3) ...)."""
    rows = []
    kind = "tum"
    with open(path) as f:
        for ln in f:
            s = ln.strip()
            if not s or s.startswith("#"):
                continue
            tok = s.split()
            vals = []
            for x in tok:
                try:
                    vals.append(float(x))
                except ValueError:
                    try:
                        vals.append(float.fromhex(x))
                    except ValueError:
                        vals.append(float("nan"))
            rows.append(vals)
    if not rows:
        raise ValueError("empty trajectory %s" % path)
    if os.path.basename(path).startswith("state_trace"):
        kind = "state_trace"
    return kind, rows


def antenna_track(kind, rows, lever):
    l = np.asarray(lever, dtype=float)
    t, a = [], []
    for r in rows:
        if kind == "tum":
            if len(r) < 8:
                continue
            R = quat_to_R(r[4], r[5], r[6], r[7])
            p = np.array(r[1:4])
            tt = r[0]
        else:
            if len(r) < 15:
                continue
            tt = r[2]
            R = np.array(r[3:12]).reshape(3, 3)
            p = np.array(r[12:15])
        t.append(tt)
        a.append(p + R @ l)
    t = np.array(t)
    a = np.array(a)
    order = np.argsort(t, kind="stable")
    t, a = t[order], a[order]
    keep = np.ones(t.size, dtype=bool)
    keep[1:] = np.diff(t) > 1e-6
    return t[keep], a[keep]


def speed_from_track(t, a, half_window=0.1):
    n = t.size
    v = np.full(n, np.nan)
    lo = np.searchsorted(t, t - half_window, side="left")
    hi = np.searchsorted(t, t + half_window, side="right") - 1
    for k in range(n):
        i, j = lo[k], hi[k]
        if j > i and t[j] - t[i] > 0.5 * half_window:
            v[k] = float(np.linalg.norm(a[j] - a[i]) / (t[j] - t[i]))
    ok = np.isfinite(v)
    return t[ok], v[ok]


def velocity_from_track(t, a, half_window=0.1):
    """Antenna velocity vectors (central differences over +-half_window, as speed_from_track)."""
    n = t.size
    v = np.full((n, 3), np.nan)
    lo = np.searchsorted(t, t - half_window, side="left")
    hi = np.searchsorted(t, t + half_window, side="right") - 1
    for k in range(n):
        i, j = lo[k], hi[k]
        if j > i and t[j] - t[i] > 0.5 * half_window:
            v[k] = (a[j] - a[i]) / (t[j] - t[i])
    ok = np.all(np.isfinite(v), axis=1)
    return t[ok], v[ok]


def vertical_in_w(replay_dir, v_w=None):
    """Unit 'up' of the estimator frame W (not levelled: W = the first IMU frame): minus the estimator's own gravity
    g (median over the replay's state_trace.txt, columns 24-26). Without a state trace, the normal of the plane of the
    antenna velocity vectors (smallest principal axis; a ground vehicle moves in the local horizontal), oriented to
    W's +z half-space. Returns (up, source)."""
    st = os.path.join(replay_dir, "state_trace.txt") if replay_dir not in ("", "-") else ""
    if st and os.path.exists(st):
        gm = np.median(load_state_trace(st)["g"], axis=0)
        if np.linalg.norm(gm) > 1.0:
            return -gm / np.linalg.norm(gm), "state_trace gravity (median g = %s)" % np.round(gm, 4).tolist()
    if v_w is None or len(v_w) < 10:
        return np.array([0.0, 0.0, 1.0]), "W z axis (no state trace, no velocities)"
    _, _, vt = np.linalg.svd(np.asarray(v_w) - np.mean(v_w, axis=0), full_matrices=False)
    up = vt[-1] if vt[-1][2] >= 0 else -vt[-1]
    return up / np.linalg.norm(up), "plane of the antenna velocities (no state trace)"


def horizontal_speed(t, v, up, sigma_s=SMOOTH_S, fs=FS_HZ, max_gap_s=MAX_GAP_S):
    """|horizontal part of the zero-phase Gaussian-smoothed velocity vector| on a uniform grid, and per grid sample
    whether a real sample lies within max_gap_s. Smoothing the VECTOR before the norm matters: the norm of a noisy
    vector is biased up by the noise (Grass02, tool_version 2: Doppler-LS |v| mean 1.36 m/s against the replay's
    0.75) and that bias follows the noise, not the motion; dropping the vertical matters as much (the Doppler-LS
    vertical velocity carried 1.6 m/s of noise on Grass02 against 0.06 m/s of real vertical motion)."""
    t = np.asarray(t, dtype=float)
    v = np.asarray(v, dtype=float)
    up = np.asarray(up, dtype=float) / np.linalg.norm(up)
    vh = v - np.outer(v @ up, up)
    e1 = np.cross(up, [1.0, 0.0, 0.0] if abs(up[0]) < 0.9 else [0.0, 1.0, 0.0])
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(up, e1)
    comp = [robust.smooth_uniform(t, vh @ e, fs, sigma_s) for e in (e1, e2)]
    g = comp[0][0]
    speed = np.hypot(comp[0][1], comp[1][1])
    idx = np.clip(np.searchsorted(t, g), 1, t.size - 1)
    near = np.minimum(np.abs(g - t[idx - 1]), np.abs(t[idx] - g))
    return g, speed, near <= max_gap_s


def xcorr_windows(t_est, s_est, t_gnss_gpst, s_gnss, leap_s, offset0, window, step, search=1.0, lag_step=0.0005, smooth_s=0.1,
                  gnss_valid=None, level_ratio_max=None, max_window_sigma_s=None):
    """Per-window offsets. For estimator time t, the GNSS UTC time is t - offset; the xcorr lag L of
    corr(s_est(t), s_gnss_utc(t + L)) therefore gives offset = -L (drift handled by the line fit).
    gnss_valid (optional, one flag per GNSS sample): samples that stand for real epochs (prepare_speeds: within
    MAX_GAP_S of a Doppler solution); a window needs MIN_COVERAGE of them, so gaps are never correlated as if they
    were data (Occlusion03: 264 usable epochs in runs of <= 6 s).
    level_ratio_max (optional, > 1; off by default so that earlier calibrations are unchanged): a window whose mean estimator
    speed differs from the mean GNSS speed by more than this factor is not used: the estimator is not following the motion there
    (A80 Data02: the host's pose freezes on an elevated road at 16 m/s, est/GNSS path 0.01), so the correlation peak of such a
    window measures its failure, not a clock offset.
    max_window_sigma_s (optional, off by default): a window whose own peak-width delay precision (robust.tde_sigma) is worse than
    this is not used: at constant speed (a straight road at 16 m/s) the speed profiles carry no timing, the peak is a plateau
    and its position scatters over hundreds of ms."""
    t_gnss_utc = np.asarray(t_gnss_gpst) - leap_s
    # shift GNSS onto the estimator axis with offset0 so the search is centred on 0
    t_g = t_gnss_utc + offset0
    valid = np.ones(t_g.size, dtype=bool) if gnss_valid is None else np.asarray(gnss_valid, dtype=bool)
    lo = max(t_est[0], t_g[0]) + search
    hi = min(t_est[-1], t_g[-1]) - search
    out = []
    w0 = lo
    while w0 + window <= hi + 1e-9:
        w1 = w0 + window
        me = (t_est >= w0) & (t_est <= w1)
        mg = (t_g >= w0) & (t_g <= w1)
        entry = {"t_from": w0, "t_to": w1, "used": False}
        coverage = float(np.mean(valid[mg])) if mg.any() else 0.0
        entry["gnss_coverage"] = coverage
        if coverage < MIN_COVERAGE:
            entry["reason"] = "GNSS velocity covers %.0f%% of the window (< %.0f%%)" % (100 * coverage, 100 * MIN_COVERAGE)
        elif (level_ratio_max and me.sum() > 20 and mg.sum() > 20 and (mg & valid).any()
              and not (1.0 / level_ratio_max <= np.mean(s_est[me]) / max(float(np.mean(s_gnss[mg & valid])), 1e-6) <= level_ratio_max)):
            entry["reason"] = "estimator and GNSS mean speeds differ by more than %gx (%.2f / %.2f m/s)" % (
                level_ratio_max, float(np.mean(s_est[me])), float(np.mean(s_gnss[mg & valid])))
        elif me.sum() > 20 and mg.sum() > 20 and np.std(s_est[me]) >= MIN_SPEED_STD and np.std(s_gnss[mg & valid]) >= MIN_SPEED_STD:
            r = robust.xcorr_offset(t_est, s_est, t_g, s_gnss, -search, search, 0.01, fs=100.0, t_from=w0, t_to=w1, fine_step=lag_step,
                                    smooth_s=smooth_s)
            if r is not None:
                entry.update({"lag": r["lag"], "peak": r["peak"], "width": r["width"], "curv_s": r["curv_s"],
                              "sigma_pw": robust.tde_sigma(r["peak"], r["curv_s"], r["duration_s"]), "at_edge": r["at_edge"]})
                if r["peak"] > 0.5 and not r["at_edge"] and max_window_sigma_s and entry["sigma_pw"] > max_window_sigma_s:
                    entry["reason"] = "peak-width precision %.0f ms is worse than %.0f ms" % (1e3 * entry["sigma_pw"], 1e3 * max_window_sigma_s)
                elif r["peak"] > 0.5 and not r["at_edge"]:
                    entry["used"] = True
                    entry["offset"] = offset0 - r["lag"]
                    tm = 0.5 * (w0 + w1)
                    entry["t_mid_est"] = tm
                    entry["t_mid_gpst"] = tm - entry["offset"] + leap_s
        else:
            entry["reason"] = "insufficient excitation (speed std < %.1f m/s) or data" % MIN_SPEED_STD
        out.append(entry)
        w0 += step
    return out


def _robust_line(tg, off, t_ref, min_windows, slope_fixed=None):
    """Theil-Sen line, windows with |residual| > max(10 ms, 4 robust sigma) rejected (e.g. a window straddling a
    host-clock step, whose correlation peak mixes both offsets), least-squares refit on the rest. With slope_fixed
    (the method-1 drift, fit_segments) only the level is fitted: median, the same rejection, mean of the rest."""
    keep = np.ones(tg.size, dtype=bool)
    if slope_fixed is not None:
        r0 = off - slope_fixed * (tg - t_ref)
        if tg.size >= min_windows:
            keep = np.abs(r0 - np.median(r0)) <= max(0.010, 4.0 * robust.robust_std(r0))
        return float(slope_fixed), float(np.mean(r0[keep]) if keep.any() else np.median(r0)), keep
    if tg.size >= min_windows:
        s, i = robust.theil_sen(tg - t_ref, off)
        r = off - (s * (tg - t_ref) + i)
        keep = np.abs(r) <= max(0.010, 4.0 * robust.robust_std(r))
    if keep.sum() >= min_windows:
        A = np.vstack([tg[keep] - t_ref, np.ones(int(keep.sum()))]).T
        (slope, icpt), *_ = np.linalg.lstsq(A, off[keep], rcond=None)
    else:
        slope, icpt = 0.0, float(np.median(off[keep] if keep.any() else off))
    return float(slope), float(icpt), keep


def _step_confirmed(confirm_steps, t_before, t_after, jump, step_s, window_s):
    """A method-1 estimator-clock step between the last window of the old level and the first of the new one
    (widened by half a window: a window's offset is its mid-time value), same sign, size within step_s."""
    for t_s, size in confirm_steps:
        if t_before - 0.5 * window_s <= t_s <= t_after + 0.5 * window_s and size * jump > 0.0 and abs(size - jump) <= step_s:
            return True
    return False


def _pw(w):
    """A window's own delay precision [s] (peak width, robust.tde_sigma); 0 when unknown."""
    v = w.get("sigma_pw", float("nan"))
    return float(v) if v is not None and np.isfinite(v) else 0.0


def _level(ws):
    """(level, sigma of the level) of a group: median of its last three windows; sigma = their median precision
    over sqrt(count) (the median of three costs little efficiency against the mean)."""
    last = ws[-3:]
    return float(np.median([w["offset"] for w in last])), float(np.median([_pw(w) for w in last])) / math.sqrt(len(last))


def _tol(step_s, *sigmas):
    """A difference counts as a jump only when it exceeds A18's step_s AND three times its own standard deviation:
    with a band-limited speed signal a window measures the offset to tens of ms (Grass01: 15-60 ms by peak width), and
    a 50 ms rule alone would cut segments on noise."""
    return max(step_s, 3.0 * math.sqrt(sum(x * x for x in sigmas)))


def split_groups(used, step_s=STEP_S, min_windows=MIN_WINDOWS, confirm_steps=(), window_s=60.0):
    """Groups of windows at one offset level (A18: "台阶(>50 ms 持续)= 新段"). A jump from the running level (median of
    the last three windows of the current group) that exceeds _tol (> step_s and > 3 sigma of the difference, from
    the windows' peak-width precisions) opens a new group only when the run of jumped windows (consecutive, mutually
    within _tol) is persistent (>= min_windows), confirmed by a method-1 step, or reaches the end of the data;
    otherwise its windows are isolated outliers. Returns (groups, isolated)."""
    groups, isolated = [[used[0]]], []
    n = len(used)
    k = 1
    while k < n:
        level, s_lvl = _level(groups[-1])
        if abs(used[k]["offset"] - level) <= _tol(step_s, _pw(used[k]), s_lvl):
            groups[-1].append(used[k])
            k += 1
            continue
        run = [used[k]]
        j = k + 1
        while j < n and abs(used[j]["offset"] - level) > _tol(step_s, _pw(used[j]), s_lvl) and \
                abs(used[j]["offset"] - float(np.median([w["offset"] for w in run]))) <= \
                _tol(step_s, _pw(used[j]), float(np.median([_pw(w) for w in run])) / math.sqrt(len(run))):
            run.append(used[j])
            j += 1
        jump = float(np.median([w["offset"] for w in run])) - level
        why = None
        if len(run) >= min_windows:
            why = "persistent (%d windows)" % len(run)
        elif _step_confirmed(confirm_steps, groups[-1][-1]["t_mid_gpst"], run[0]["t_mid_gpst"], jump, step_s, window_s):
            why = "confirmed by a method-1 step"
        elif j >= n:
            why = "reaches the end of the data (old level never restored)"
        if why:
            run[0]["segment_start"] = why
            groups.append(run)
        else:
            for w in run:
                w["isolated_jump"] = True
            isolated.extend(run)
        k = j
    return groups, isolated


def fit_segments(windows, min_windows=MIN_WINDOWS, step_s=STEP_S, seed=0, confirm_steps=(), window_s=60.0, drift_ppm_at=None):
    """Split at persistent offset jumps (split_groups), fit offset + drift per segment (robust), sigma (module doc).
    drift_ppm_at(t_gpst) (optional): the drift to use for the segment whose reference time is t — main passes the
    method-1 prior's drift, which the record-time series measures to ~1 ppm; the windows then only fit the level.
    Window offsets carry 10-60 ms each (band-limited speed signal), so a free slope over a few minutes is noise
    (Grass02, 3 windows over 90 s: -943 ppm), while the header-clock drift against GPST is a few ppm."""
    used = sorted([w for w in windows if w.get("used")], key=lambda w: w["t_mid_gpst"])
    if not used:
        return []
    groups, isolated = split_groups(used, step_s, min_windows, confirm_steps, window_s)
    segs = []
    rng = np.random.RandomState(seed)
    for g in groups:
        tg = np.array([w["t_mid_gpst"] for w in g])
        off = np.array([w["offset"] for w in g])
        t_ref = float(np.median(tg))
        fixed = None if drift_ppm_at is None else float(drift_ppm_at(t_ref)) * 1e-6
        slope, icpt, keep = _robust_line(tg, off, t_ref, min_windows, slope_fixed=fixed)
        resid = off[keep] - (slope * (tg[keep] - t_ref) + icpt)
        nk = int(keep.sum())
        n_par = 1 if fixed is not None else 2
        # per-window scatter about the fitted line, dof-corrected (one or two line parameters)
        scatter = float(np.sqrt(np.sum(resid ** 2) / (nk - n_par))) if nk > n_par else float("nan")
        boots = []
        if nk >= min_windows:
            A = np.vstack([tg[keep] - t_ref, np.ones(nk)]).T
            for _ in range(200):
                k = rng.randint(0, nk, nk)
                if fixed is not None:
                    boots.append(float(np.mean(off[keep][k] - fixed * (tg[keep][k] - t_ref))))
                    continue
                if np.ptp(A[k, 0]) <= 0:
                    continue
                (_s2, i2), *_ = np.linalg.lstsq(A[k], off[keep][k], rcond=None)
                boots.append(i2)
        bsd = float(np.std(boots)) if boots else float("nan")
        # peak-width precision: median window value; the segment value divides by the number of independent
        # (non-overlapping) windows its windows span
        pw = [w["sigma_pw"] for w, kk in zip(g, keep) if kk and np.isfinite(w.get("sigma_pw", float("nan")))]
        pw_window = float(np.median(pw)) if pw else float("nan")
        n_indep = max(1.0, (float(tg.max() - tg.min()) + window_s) / window_s)
        pw_seg = pw_window / math.sqrt(n_indep) if np.isfinite(pw_window) else float("nan")
        short = nk < min_windows
        if short:
            sigma = max(pw_window if np.isfinite(pw_window) else 0.0, SIGMA_FALLBACK)
        else:
            sigma = max([1e-3] + [v for v in (scatter, bsd, pw_seg) if np.isfinite(v)])
        t0, t1 = float(tg.min()), float(tg.max())
        n_iso = sum(1 for w in isolated if t0 <= w["t_mid_gpst"] <= t1)
        segs.append({"t_from_gpst_s": 0.0, "offset_s": icpt, "drift_ppm": slope * 1e6,
                     "t_ref_gpst_s": t_ref, "sigma_toff_s": sigma, "tdcp_paused": tdcp_paused_for("xcorr", sigma), "method": "xcorr",
                     "peak_corr": float(np.median([w["peak"] for w in g])), "window_scatter_ms": 1e3 * scatter if np.isfinite(scatter) else -1.0,
                     "n_windows": nk, "n_windows_rejected": int(tg.size - nk), "n_windows_isolated": n_iso,
                     "bootstrap_sd_ms": 1e3 * bsd if np.isfinite(bsd) else -1.0,
                     "peak_width_sigma_ms": 1e3 * (pw_window if short else pw_seg) if np.isfinite(pw_window) else -1.0,
                     "short_segment": short, "drift_source": "method1_prior" if fixed is not None else "window_fit"})
    # a later segment starts halfway between the last window of the previous group and the first of this one
    for k in range(1, len(segs)):
        a = max(w["t_mid_gpst"] for w in groups[k - 1])
        b = min(w["t_mid_gpst"] for w in groups[k])
        segs[k]["t_from_gpst_s"] = 0.5 * (a + b)
    return segs


def dtw_check(t_est, s_est, t_gnss_gpst, s_gnss, leap_s, offset, fs=10.0, band_s=2.0):
    """Method 3: DTW path offset after applying the xcorr offset; returns (dtw_offset, mad)."""
    t_g = np.asarray(t_gnss_gpst) - leap_s + offset
    lo, hi = max(t_est[0], t_g[0]), min(t_est[-1], t_g[-1])
    if hi - lo < 20.0:
        return None
    grid = np.arange(lo, hi, 1.0 / fs)
    a = np.interp(grid, t_est, s_est)
    b = np.interp(grid, t_g, s_gnss)
    path = robust.dtw_offsets(a, b, int(round(band_s * fs)))
    if not path:
        return None
    d = np.array([(j - i) / fs for i, j in path])
    # b is the GNSS series already shifted by `offset`; a residual shift L means est(t) ~ gnss(t + L): offset - L
    med = float(np.median(d))
    return offset - med, robust.mad(d)


# ------------------------------------------------------------------------------------------------------------
# GNSS side of method 2
# ------------------------------------------------------------------------------------------------------------
HEADER_TOPICS = ("/livox/avia/lidar", "/livox/avia/imu", "/camera/color/image_raw/compressed")


def read_inputs(bag_path, imu_topic=IMU_TOPIC):
    """One pass: GNSS epochs/ephemerides/PVT and (header, record) pairs of the LiDAR/IMU/image topics."""
    rb = rosio.RawBag(bag_path)
    eps, eph, geph, pvt = [], [], [], []
    hdr_topics = tuple(HEADER_TOPICS) + ((imu_topic,) if imu_topic not in HEADER_TOPICS else ())
    hdr = {t: [] for t in hdr_topics}
    topics = [gnss_msgs.TOPIC_MEAS, gnss_msgs.TOPIC_EPHEM, gnss_msgs.TOPIC_GLO_EPHEM, gnss_msgs.TOPIC_PVT] + list(hdr_topics)
    for topic, datatype, raw, t_rec, pytype in rb.iter_raw(topics=topics):
        if topic in hdr:
            hdr[topic].append((rosio.parse_header(raw)[1], t_rec))
            continue
        m = rb.deserialize(pytype, raw)
        if topic == gnss_msgs.TOPIC_MEAS:
            eps.append(gnss_msgs.meas_epoch(m, t_rec))
        elif topic == gnss_msgs.TOPIC_EPHEM:
            eph.append(gnss_msgs.kepler_eph(m, t_rec))
        elif topic == gnss_msgs.TOPIC_GLO_EPHEM:
            geph.append(gnss_msgs.glo_eph(m, t_rec))
        else:
            pvt.append(gnss_msgs.pvt(m, t_rec))
    rb.close()
    hstats = {}
    for t, v in hdr.items():
        if v:
            a = np.array(v)
            d = (a[:, 0] - a[:, 1]) * 1e3
            slope, _ = robust.theil_sen(a[:, 1] - a[0, 1], d, max_pairs=50000)
            hstats[t] = {"median_ms": float(np.median(d)), "mad_ms": robust.mad(d), "p1_ms": float(np.percentile(d, 1)),
                         "p99_ms": float(np.percentile(d, 99)), "drift_ppm": slope * 1e3, "n": int(d.size)}
    imu = np.array(hdr.get(imu_topic) or np.zeros((0, 2)))
    return eps, eph, geph, pvt, hstats, imu


def smooth_drift(t, d, median_half_s=DRIFT_MEDIAN_HALF_S, sigma_s=DRIFT_SMOOTH_S):
    """Robust smooth of the per-epoch clock drift: running median over +-median_half_s, then a Gaussian (sigma_s)
    over the available samples only (no interpolation across gaps). The receiver's oscillator drift changes slowly
    (TCXO), the per-epoch LS drift carries the geometry noise it shares with the vertical velocity."""
    t = np.asarray(t, dtype=float)
    d = np.asarray(d, dtype=float)
    med = np.array([np.median(d[(t >= x - median_half_s) & (t <= x + median_half_s)]) for x in t])
    out = np.empty_like(med)
    for k, x in enumerate(t):
        m = np.abs(t - x) <= 4.0 * sigma_s
        w = np.exp(-0.5 * ((t[m] - x) / sigma_s) ** 2)
        out[k] = float(np.sum(w * med[m]) / np.sum(w))
    return out


def dopp_std_from_cn0(cn0_dbhz, ref_dbhz=40.0, sigma_ref_hz=0.3, lo=0.1, hi=3.0):
    """Doppler std [Hz] for receivers that publish none (the A80 slim bags carry std = 0): 0.3 Hz at 40 dB-Hz, growing
    10^(-dC/20) with the C/N0 deficit (a tracking-loop scaling), clipped to [0.1, 3] Hz. Only the WEIGHTS of the Doppler LS
    depend on it; a bag whose receiver gives std values keeps them."""
    return float(min(hi, max(lo, sigma_ref_hz * 10.0 ** ((ref_dbhz - cn0_dbhz) / 20.0))))


def fixed_position_pvt(eps, xyz):
    """One PVT-like entry per GNSS epoch at a fixed ECEF position (bags without receiver_pvt, --approx-xyz)."""
    lat, lon, h = geodesy.ecef2pos(np.asarray(xyz, dtype=float))
    return [{"t_gpst": e["t_gpst"], "fix_type": 3, "lat": math.degrees(lat), "lon": math.degrees(lon), "h_ell": float(h)} for e in eps]


def doppler_velocity_series(eps, eph, geph, pvt, rnx):
    """GNSS side of method 2: antenna velocity (ENU at the receiver PVT position) per epoch, two passes:
      1. weighted LS of v and the clock drift d on the rows with C/N0 >= CN0_MIN_DOPP and el >= EL_MIN_DOPP,
         sigma_i = lambda_i x the receiver's dopp_std_i (its own Doppler std; on Grass01/02 it follows the rows'
         scatter about the replay's velocity with a near-constant factor ~0.5 across C/N0), 0/1 rejection at 4 sigma;
      2. d smoothed over time (smooth_drift) and the velocity re-solved with d fixed (>= 4 rows).
    Why pass 2: with the low-elevation satellites weak (C/N0 < 25 dB-Hz on the grass field), the usable geometry is
    zenith-heavy and the vertical velocity and d are nearly collinear; a free per-epoch d leaves the vertical
    velocity with 0.7-1.6 m/s of noise (Grass02; 0.19 m/s once d is smoothed and fixed). Returns
    (t_gpst, v_enu (n,3), info)."""
    keys = {(r["sat"], round(r["toe"])) for r in rnx}
    recs = list(rnx) + [r for r in eph + geph if (r["sat"], round(r["toe"])) not in keys]
    sel = orbits.EphSelector(recs, MAX_EPH_AGE)
    pt = np.array([p["t_gpst"] for p in pvt]) if pvt else np.zeros(0)
    ep_rows = []                                    # (t, rr, rows, sigmas)
    for e in eps:
        if not pt.size:
            break
        k = int(np.argmin(np.abs(pt - e["t_gpst"])))
        p = pvt[k]
        if p["fix_type"] < 2 or abs(pt[k] - e["t_gpst"]) > 1.0:
            continue
        rr = geodesy.pos2ecef(math.radians(p["lat"]), math.radians(p["lon"]), p["h_ell"])
        obs = obs_mod.ls_obs(e)
        if not any(o.get("dopp_std", 0.0) > 0.0 for o in obs):
            obs = [dict(o, dopp_std=dopp_std_from_cn0(o["cn0"])) for o in obs]      # std fields all zero (A80 slim bags)
        obs = [o for o in obs if o.get("dopp_std", 0.0) > 0.0]
        sig = {o["sat"]: CLIGHT / o["freq"] * o["dopp_std"] for o in obs if o.get("freq", 0.0) > 0.0}
        rows = gnsslsq.doppler_rows(obs, e["t_gpst"], sel, rr, cn0_min=CN0_MIN_DOPP, el_mask_deg=EL_MIN_DOPP)
        rows = [r for r in rows if r[0] in sig]
        if len(rows) >= 4:
            ep_rows.append((e["t_gpst"], rr, rows, [sig[r[0]] for r in rows]))
    t1, d1 = [], []
    for t, rr, rows, sg in ep_rows:
        s = gnsslsq.solve_doppler_weighted(rows, sg, rr)
        if s is not None:
            t1.append(t)
            d1.append(s["drift_mps"])
    info = {"epochs_with_4_rows": len(ep_rows), "pass1_epochs": len(t1), "cn0_min_dbhz": CN0_MIN_DOPP, "el_min_deg": EL_MIN_DOPP}
    if len(t1) < 2:
        return np.zeros(0), np.zeros((0, 3)), info
    t1, d1 = np.array(t1), np.array(d1)
    ds = smooth_drift(t1, d1)
    t_out, v_out, rows_used = [], [], []
    for t, rr, rows, sg in ep_rows:
        j = int(np.searchsorted(t1, t))
        near = [i for i in (j - 1, j) if 0 <= i < t1.size and abs(t1[i] - t) <= DRIFT_MEDIAN_HALF_S]
        if not near:
            continue                                # no drift estimate within 1 s: no epoch
        d = float(np.interp(t, t1, ds))
        s = gnsslsq.solve_doppler_weighted(rows, sg, rr, drift_mps=d)
        if s is not None:
            t_out.append(t)
            v_out.append(s["v_enu"])
            rows_used.append(s["n"])
    info.update({"pass2_epochs": len(t_out), "rows_used_median": float(np.median(rows_used)) if rows_used else 0.0,
                 "drift_mps_median": float(np.median(d1)), "drift_scatter_about_smooth_mps": float(robust.robust_std(d1 - ds))})
    return np.array(t_out), np.array(v_out).reshape(-1, 3), info


def write_plot(path, tg, tr, wins, segs, prior=None):
    """Diagnostics: method-1 record-time offsets (host clock) and the estimator-clock upper bound, and, when method 2
    ran, the per-window offsets with the fitted lines."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return False
    tu = np.array([gnsstime.gpst_to_utc(t) for t in tg])
    fig, ax = plt.subplots(2 if wins else 1, 1, figsize=(9, 5.5 if wins else 3), squeeze=False)
    ax[0, 0].plot(tg - tg[0], (np.asarray(tr) - tu) * 1e3, ".", ms=1.5, label="GNSS record - (GPST-leap) (host clock)")
    if prior is not None:
        pt, pu = prior["series"]
        ax[0, 0].plot(pt - tg[0], pu * 1e3, "-", lw=1, label="estimator-clock upper bound (IMU header, lower envelopes)")
    ax[0, 0].set_ylabel("[ms]")
    ax[0, 0].legend(fontsize=7)
    ax[0, 0].set_title("method 1: record-time offsets")
    if wins:
        if prior is not None:
            pt, pu = prior["series"]
            ax[1, 0].fill_between(pt - tg[0], 1e3 * (pu - LAT_G_MAX), 1e3 * pu, color="0.85", lw=0,
                                  label="method-1 prior range (GNSS latency 0..%.0f ms)" % (1e3 * LAT_G_MAX))
            ax[1, 0].plot(pt - tg[0], 1e3 * (pu - 0.5 * LAT_G_MAX), "--", color="0.5", lw=1, label="method-1 prior")
        used = [w for w in wins if w.get("used")]
        if used:
            x = np.array([w["t_mid_gpst"] for w in used]) - tg[0]
            y = np.array([w["offset"] for w in used]) * 1e3
            e = np.array([w.get("sigma_pw", float("nan")) for w in used]) * 1e3
            iso = np.array([bool(w.get("isolated_jump")) for w in used])
            ax[1, 0].errorbar(x[~iso], y[~iso], yerr=e[~iso], fmt="o", ms=4, capsize=2, label="window xcorr offsets (peak-width sigma)")
            if iso.any():
                ax[1, 0].plot(x[iso], y[iso], "x", ms=6, label="isolated jumps (dropped)")
        for sgm in segs:
            if sgm.get("method") == "xcorr":
                xx = np.linspace(0, tg[-1] - tg[0], 50)
                yy = 1e3 * (sgm["offset_s"] + sgm["drift_ppm"] * 1e-6 * (xx + tg[0] - sgm["t_ref_gpst_s"]))
                ax[1, 0].plot(xx, yy, "-", lw=1.2, color="C1", label="written segment (sigma %.0f ms)" % (1e3 * sgm["sigma_toff_s"]))
                ax[1, 0].fill_between(xx, yy - 1e3 * sgm["sigma_toff_s"], yy + 1e3 * sgm["sigma_toff_s"], color="C1", alpha=0.15, lw=0)
        ax[1, 0].set_xlabel("GPST - first epoch [s]")
        ax[1, 0].set_ylabel("offset [ms]")
        ax[1, 0].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)
    return True


# ------------------------------------------------------------------------------------------------------------
def prior_drift_at(prior):
    """drift_ppm_at(t) for fit_segments: the drift of the method-1 prior piece that covers t (None without a prior)."""
    if not prior:
        return None
    ps = prior_segments(prior)
    if not ps:
        return None
    starts = [-math.inf] + [p["t_from_gpst_s"] for p in ps[1:]]

    def at(t):
        k = max(i for i, a in enumerate(starts) if t >= a)
        return ps[k]["drift_ppm"]
    return at


def calibrate(t_est, s_est, t_gnss, s_gnss, leap_s, offset0, window, step, confirm_steps=(), min_windows=MIN_WINDOWS,
              smooth_s=0.1, gnss_valid=None, drift_ppm_at=None, level_ratio_max=None, max_window_sigma_s=None):
    """Methods 2 + 3 on prepared speed series (main: horizontal speeds already smoothed on a uniform grid, smooth_s=0,
    gnss_valid = the coverage flags of horizontal_speed, drift_ppm_at = prior_drift_at(prior)). Returns (segments,
    windows, dtw)."""
    wins = xcorr_windows(t_est, s_est, t_gnss, s_gnss, leap_s, offset0, window, step, smooth_s=smooth_s, gnss_valid=gnss_valid,
                         level_ratio_max=level_ratio_max, max_window_sigma_s=max_window_sigma_s)
    segs = fit_segments(wins, min_windows=min_windows, confirm_steps=confirm_steps, window_s=window, drift_ppm_at=drift_ppm_at)
    dtw = None
    if segs:
        seg = segs[0] if len(segs) == 1 else max(segs, key=lambda s: s["n_windows"])
        r = dtw_check(t_est, s_est, t_gnss, s_gnss, leap_s, seg["offset_s"])
        if r is not None:
            dtw = {"offset_s": r[0], "mad_s": r[1], "xcorr_offset_s": seg["offset_s"],
                   "consistent": abs(r[0] - seg["offset_s"]) <= 3.0 * seg["sigma_toff_s"]}
            for s in segs:
                s["dtw_offset_s"] = r[0]
                s["dtw_consistent"] = dtw["consistent"]
    return segs, wins, dtw


def config_segment(offset_s, drift_ppm, t_ref_gpst_s):
    """A18 method config: the sequence config's values kept (neither method calibrated), sigma 50 ms, TDCP paused."""
    return {"t_from_gpst_s": 0.0, "offset_s": float(offset_s), "drift_ppm": float(drift_ppm), "t_ref_gpst_s": float(t_ref_gpst_s),
            "sigma_toff_s": SIGMA_FALLBACK, "tdcp_paused": tdcp_paused_for("config", SIGMA_FALLBACK), "method": "config"}


def merge_with_prior(xsegs, psegs):
    """Per time span the more precise of method 2 and the method-1 prior (A53 "方法②可用时取其 σ_toff(更小)").

    xsegs / psegs: A18 segment lists, each sorted, the first covering -inf (t_from 0). A method-2 segment whose sigma
    is larger than that of the prior piece(s) under its span is replaced there by those piece(s), cut to the span.
    Returns (segments, replaced) with replaced = indices of the method-2 segments that were replaced."""
    def starts(segs):
        return [-math.inf] + [float(s["t_from_gpst_s"]) for s in segs[1:]]
    xs, ps = starts(xsegs), starts(psegs)
    xe = xs[1:] + [math.inf]
    pe = ps[1:] + [math.inf]
    out, replaced, last_prior = [], [], None
    for k, sg in enumerate(xsegs):
        a, b = xs[k], xe[k]
        under = [j for j in range(len(psegs)) if ps[j] < b and pe[j] > a]
        if under and all(float(sg["sigma_toff_s"]) > float(psegs[j]["sigma_toff_s"]) for j in under):
            replaced.append(k)
            for j in under:
                if last_prior == j:                    # the same prior piece continues across two replaced spans
                    continue
                piece = dict(psegs[j])
                piece["t_from_gpst_s"] = max(a, ps[j])
                piece["replaces_method2_segment"] = True
                out.append(piece)
                last_prior = j
            continue
        seg = dict(sg)
        seg["t_from_gpst_s"] = a
        out.append(seg)
        last_prior = None
    out[0]["t_from_gpst_s"] = 0.0                      # A18: the first segment covers -inf
    for s in out[1:]:
        s["t_from_gpst_s"] = float(s["t_from_gpst_s"])
    return out, replaced


def choose_segments(method2_ran, xcorr_segs, prior, config_offset=None, config_drift_ppm=0.0, config_t_ref=0.0,
                    no_replay_reason="ref-R (no GNSS-free replay trajectory)",
                    no_window_reason="insufficient excitation"):
    """What goes into the A18 file (module docstring, AMENDMENTS A53). Returns (segments, status, blocked or None):
      * method 2 produced segments -> per span the more precise of them and the method-1 prior (merge_with_prior):
        a cross-correlation that knows less than the record times is not a calibration (Grass02, tool_version 2:
        two windows with peaks ~0.5, sigma 533 ms, against the prior's 75 ms);
      * no method-2 segment (no replay trajectory, or no usable window: insufficient excitation) -> the method-1
        prior (method record_time, sigma 75 ms, TDCP running);
      * no method-1 prior either (no IMU header clock) -> method config with the given config values (TDCP paused);
        without them: blocked (nothing is written: the tool never labels a record-time number as the config value).
    `status` describes method 2 (it starts with "blocked:" when method 2 could not run)."""
    psegs = prior_segments(prior) if prior else []
    if xcorr_segs:
        if not psegs:
            return xcorr_segs, "ran", None
        segs, replaced = merge_with_prior(xcorr_segs, psegs)
        if not replaced:
            return segs, "ran", None
        sig = "/".join("%.0f" % (1e3 * xcorr_segs[k]["sigma_toff_s"]) for k in replaced)
        if len(replaced) == len(xcorr_segs):
            return (segs, "ran, but every method-2 segment is less precise than the method-1 prior (sigma %s ms > %.0f ms)"
                    " -> method-1 prior (record_time) kept" % (sig, 500.0 * LAT_G_MAX), None)
        return (segs, "ran; %d of %d method-2 segments less precise than the method-1 prior (sigma %s ms > %.0f ms) replaced by it"
                " over their spans" % (len(replaced), len(xcorr_segs), sig, 500.0 * LAT_G_MAX), None)
    if method2_ran:
        status = "ran: no usable window (%s)" % no_window_reason
    elif no_replay_reason.startswith("not run"):
        status = no_replay_reason                  # method 1 only on purpose (replay_dir '-')
    else:
        status = "blocked: %s" % no_replay_reason
    if psegs:
        return psegs, status + " -> method-1 prior (record_time)", None
    if config_offset is not None:
        return ([config_segment(config_offset, config_drift_ppm, config_t_ref)],
                status + " and no method-1 prior (no IMU header clock) -> A18 method config, sequence config values kept, TDCP paused", None)
    if method2_ran:
        return [], status, ("blocked: no usable method-2 window (%s), no method-1 prior (no IMU header clock) and no "
                            "--config-offset to keep (A18 method config)" % no_window_reason)
    return [], status, "blocked: %s and no IMU header clock for the method-1 prior" % no_replay_reason.replace("not run ", "")


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    seq = args.seq or os.path.splitext(os.path.basename(args.bag))[0]
    rep = {"tool": "gnss_time_calib.py", "tool_version": TOOL_VERSION, "sequence": seq, "bag": os.path.abspath(args.bag),
           "time_origin": "all *_gpst_s values = GTime::toSec(): seconds since 1970-01-01 on the GPST scale (no leap seconds)"}
    report_path = args.report or os.path.splitext(args.out_yaml)[0] + ".json"
    out_dir = os.path.dirname(os.path.abspath(args.out_yaml))
    os.makedirs(out_dir, exist_ok=True)
    eps, eph, geph, pvt, hstats, imu = read_inputs(args.bag, args.imu_topic)
    if not eps:
        print("gnss_time_calib: no range_meas epochs in %s" % args.bag, file=sys.stderr)
        return 2
    approx = args.approx_xyz
    if approx is None and args.approx_from_gt:
        with open(os.path.join(args.approx_from_gt, "gt_audit.json")) as f:
            approx = [float(x) for x in geodesy.pos2ecef(*json.load(f)["enu_origin_lat_lon_h_rad_rad_m"])]
    if approx is not None and not pvt:
        pvt = fixed_position_pvt(eps, approx)
        rep["pvt_source"] = "fixed position (ECEF %s): the bag has no receiver_pvt" % [round(float(x), 1) for x in approx]
    tg = np.array([e["t_gpst"] for e in eps])
    tr = np.array([e["t_record"] for e in eps])
    m1 = record_time_stats(tg, tr, hstats)
    rep["method1_record_time"] = m1
    leap = m1["leap_s"]
    prior = estimator_clock_prior(tg, tr, imu[:, 0], imu[:, 1]) if imu.size else None
    rep["method1_estimator_clock_prior"] = {k: v for k, v in prior.items() if k != "series"} if prior else None
    if args.offset0 is not None:
        offset0, how0 = args.offset0, "--offset0"
    elif prior:
        offset0, how0 = prior["offset_s"], "method-1 estimator-clock prior"
    else:
        offset0, how0 = m1["median_s"], "method-1 host-clock median (no %s in the bag)" % args.imu_topic
    rep["offset0"] = {"value_s": offset0, "source": how0}
    confirm = prior["steps"] if prior else []
    traj = args.traj or (os.path.join(args.replay_dir, "traj_W.tum") if args.replay_dir not in ("-", "") else "")
    xsegs, dtw, wins = [], None, []
    ran = bool(traj and os.path.exists(traj))
    if ran:
        kind, rows = load_traj(traj)
        t_a, a = antenna_track(kind, rows, args.lever_arm)
        t_v, v_w = velocity_from_track(t_a, a)
        up_w, up_src = vertical_in_w(args.replay_dir if args.replay_dir not in ("-", "") else os.path.dirname(traj), v_w)
        rnx = []
        for pth in args.nav_rinex:
            rnx += rinexnav.read_nav(pth)[1]
        t_g, v_g, dinfo = doppler_velocity_series(eps, eph, geph, pvt, rnx)
        rep["method2_inputs"] = {"trajectory": traj, "kind": kind, "traj_samples": int(t_v.size), "gnss_epochs": int(t_g.size),
                                 "doppler_ls": dinfo, "up_in_W": up_w.tolist(), "up_source": up_src, "smooth_s": SMOOTH_S,
                                 "signal": "horizontal antenna speed = |horizontal part of the zero-phase Gaussian-smoothed (%.2f s) "
                                           "velocity vector| on both sides, %g Hz grid" % (SMOOTH_S, FS_HZ)}
        if t_g.size > 100 and t_v.size > 100:
            tE, sE, _ = horizontal_speed(t_v, v_w, up_w)
            tG, sG, vG = horizontal_speed(t_g, v_g, np.array([0.0, 0.0, 1.0]))
            rep["method2_inputs"].update({"est_speed_std": float(np.std(sE)), "gnss_speed_std": float(np.std(sG[vG])),
                                          "gnss_coverage": float(np.mean(vG))})
            xsegs, wins, dtw = calibrate(tE, sE, tG, sG, leap, offset0, args.window, args.step, confirm_steps=confirm,
                                         smooth_s=0.0, gnss_valid=vG, drift_ppm_at=prior_drift_at(prior),
                                         level_ratio_max=args.level_ratio_max, max_window_sigma_s=args.max_window_sigma_s)
        else:
            rep["method2_inputs"]["note"] = ("too few Doppler-velocity epochs (%d; a solution needs >= 5 rows with C/N0 >= %g dB-Hz "
                                             "and el >= %g deg) or trajectory samples" % (t_g.size, CN0_MIN_DOPP, EL_MIN_DOPP))
        rep["method2_windows"] = wins
        rep["method3_dtw"] = dtw
    rep["method2_segments"] = xsegs                  # what the cross-correlation found, whatever goes into the file
    why = "insufficient excitation"
    if ran and not wins:
        why = rep["method2_inputs"].get("note", "no window fits the overlap of the two series")
    elif wins and not any(w.get("used") for w in wins):
        reasons = sorted({w.get("reason") or ("peak %.2f%s" % (w["peak"], " at the search edge" if w.get("at_edge") else "")
                                              if "peak" in w else "no correlation") for w in wins})
        why = "; ".join(reasons)
    no_replay = ("ref-R (no GNSS-free replay trajectory at %s)" % traj if traj else
                 "not run (no replay given, replay_dir '-': method 1 only)")
    segs, status2, blocked = choose_segments(ran, xsegs, prior, args.config_offset, args.config_drift_ppm, args.config_t_ref,
                                             no_replay, why)
    rep["method2_status"] = status2
    rep["tdcp_rule"] = ("AMENDMENTS A53: tdcp_paused = not (method in %s and sigma_toff_s <= %.3f s); per span the more precise of "
                        "method 2 and the method-1 prior" % (list(CALIBRATING_METHODS), TDCP_SIGMA_MAX))
    warnings = []
    for s in segs:
        if s.get("window_scatter_ms", 0) > 5.0:
            warnings.append("window scatter %.1f ms > 5 ms in segment from %.1f" % (s["window_scatter_ms"], s["t_from_gpst_s"]))
        if s.get("short_segment"):
            warnings.append("segment from %.1f has %d window(s): sigma %.0f ms%s" % (s["t_from_gpst_s"], s["n_windows"], 1e3 * s["sigma_toff_s"],
                                                                                    ", TDCP paused" if s["tdcp_paused"] else ""))
        if s["tdcp_paused"]:
            warnings.append("segment from %.1f: TDCP paused (method %s, sigma %.0f ms; A53 needs method 1 or 2 with sigma <= %.0f ms)" % (
                s["t_from_gpst_s"], s.get("method"), 1e3 * s["sigma_toff_s"], 1e3 * TDCP_SIGMA_MAX))
    n_iso = sum(1 for w in wins if w.get("isolated_jump"))
    if n_iso:
        warnings.append("%d isolated window(s) jumped > %.0f ms and were dropped" % (n_iso, 1e3 * STEP_S))
    if dtw and not dtw["consistent"]:
        warnings.append("DTW offset %.4f s differs from xcorr %.4f s by more than 3 sigma" % (dtw["offset_s"], dtw["xcorr_offset_s"]))
    rep["segments"] = segs
    rep["warnings"] = warnings
    if blocked:
        rep["status"] = blocked
        report.write_json(report_path, rep)
        print("gnss_time_calib: %s; nothing written to %s (report %s)" % (blocked, args.out_yaml, report_path), file=sys.stderr)
        return EXIT_BLOCKED
    text = timecalib_yaml.write(args.out_yaml, seq, leap, segs, TOOL_VERSION, span=(float(tg[0]), float(tg[-1])))
    rep["yaml"] = text
    if args.plot:
        rep["plot"] = os.path.splitext(args.out_yaml)[0] + ".png"
        write_plot(rep["plot"], tg, tr, wins, segs, prior)
    report.write_json(report_path, rep)
    print(text, end="")
    print("method 1: record - GPST->UTC median %.1f ms (MAD %.1f, drift %.1f ppm, host steps %s); estimator-clock prior %s; method 2: %s" % (
        1e3 * m1["median_s"], 1e3 * m1["mad_s"], m1["drift_ppm"], m1["steps"] or "none",
        ("%.1f ms (upper bound %.1f ms, drift %.1f ppm, sigma %.0f ms, steps %s)" % (
            1e3 * prior["offset_s"], 1e3 * prior["upper_bound_s"], prior["drift_ppm"], 1e3 * prior["sigma_s"], prior["steps"] or "none"))
        if prior else "n/a", status2))
    for w in warnings:
        print("WARNING: " + w)
    return 0


if __name__ == "__main__":
    sys.exit(main())
