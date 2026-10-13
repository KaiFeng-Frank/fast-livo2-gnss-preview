#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Ground-truth audit before evaluation (TEST_PLAN §4 gt_audit): frame/datum check, time base, trust mask.

Lat/lon vs 9Ps WGS-84 PVT (GCJ-02 offset detection); local XYZ vs PVT-ENU SE(3)/Sim(3) alignment residual structure;
GT time base by cross-correlation (> 20 ms compensated; undetermined or ambiguous => FAIL and no gt_audited.tum);
trust mask from 9Ps h_acc > 1 m or num_sv < 8, speed spikes > 3 sigma or gaps > 0.5 s, every criterion widened by
±2 s; 2HP lever arm unknown recorded as an ATE floor. Owner: WP-g.

Inputs are the GT file and raw sensor topics of the ORIGINAL bag (AMENDMENTS A2: on evaluation sequences no estimated
trajectory of this system or of FAST-LIVO2 is read; the optional P4c diagnostic reads a GNSS-free replay on development
sequences only):
  * the RTK receiver fix /mavros/global_position/raw/fix (C-RTK2HP, the GT source) when present: provenance of the
    GT file (which stamps, which projection) is then decided exactly;
  * the 9Ps receiver PVT /ublox_driver/receiver_pvt (WGS-84 by construction): datum test (GCJ-02 shifts are
    hundreds of metres, receiver-vs-RTK differences metres), SE(3)/Sim(3) residual structure, GPS time base;
  * /livox/avia/imu (the estimator's time base: LiDAR/IMU header time) and /odom for the GT time-base
    cross-correlation (course rate vs gyro yaw rate; speed vs wheel speed).
Outputs: gt_audit.md/json, trust_mask.csv (pre-registered mask + reasons), gt_mask_intervals.csv, gt_clean.tum
(sorted, duplicate rows removed), gt_timebase.yaml and gt_audited.tum (gt_clean with the time-base compensation
applied iff |value| > 20 ms). AMENDMENTS A54 (P4): the evaluation's GT time base is decided by a fixed rule
(decide_timebase: P4 pass -> the odometry chain; P4 fail -> the record-time value; neither -> 0, flagged), so
gt_audited.tum is always written and gt_timebase.yaml records the method and its confidence; the P4 verdict itself
is still the pre-registered one. The trust mask gains RTK jumps / displaced plateaus (bit 64, rtk_jumps).
"""
import argparse
import collections
import csv
import datetime
import json
import math
import os
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402

from fl2g import TOOL_VERSION, geodesy, gnss_msgs, gnsstime, report, robust, rosio  # noqa: E402
from fl2g.replay import load_state_trace  # noqa: E402
from fl2g.sequences import seq_class  # noqa: E402

RTK_TOPIC = "/mavros/global_position/raw/fix"
COMP_THRESHOLD_S = 0.020          # TEST_PLAN §4: > 20 ms is compensated
AGREE_S = 0.050                   # chain vs record-time plausibility: larger disagreement = ambiguous time base
MASK_WIDEN_S = 2.0                # TEST_PLAN §4 "前后 2 s", applied to every mask criterion


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0], formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("gt_file", help="M3DGR ground-truth file")
    p.add_argument("bag", help="bag with /ublox_driver/receiver_pvt")
    p.add_argument("out_dir", help="report directory")
    p.add_argument("--assume-gcj02", action="store_true", help="force the GCJ-02 → WGS-84 correction")
    p.add_argument("--seq", default="")
    p.add_argument("--rtk-topic", default=RTK_TOPIC)
    p.add_argument("--pvt-topic", default=gnss_msgs.TOPIC_PVT)
    p.add_argument("--imu-topic", default="/livox/avia/imu")
    p.add_argument("--odom-topic", default="/odom")
    p.add_argument("--archive-label", default="")
    p.add_argument("--no-plots", action="store_true")
    p.add_argument("--replay-dir", default="",
                   help="GNSS-free replay of this bag (state_trace.txt): enables the replay-route diagnostic P4c on "
                        "development sequences; never opened on other sequences (A2)")
    return p


# ------------------------------------------------------------------------------------------------------------
def load_tum(path):
    rows = []
    with open(path) as f:
        for ln in f:
            s = ln.strip()
            if not s or s.startswith("#"):
                continue
            v = s.replace(",", " ").split()
            if len(v) >= 8:
                rows.append([float(x) for x in v[:8]])
    return np.array(rows)


def clean_gt(a):
    """Stable sort by time; drop rows that repeat (t, pose) exactly; keep the first of equal stamps."""
    order = np.argsort(a[:, 0], kind="stable")
    b = a[order]
    keep = np.ones(b.shape[0], dtype=bool)
    keep[1:] = b[1:, 0] != b[:-1, 0]
    return b[keep], int(np.sum(order != np.arange(a.shape[0]))), int(np.sum(~keep))


def update_series(t, p):
    """Rows where the position changes (the RTK updates are slower than the 15 Hz republish): first occurrences."""
    keep = np.ones(t.size, dtype=bool)
    keep[1:] = np.any(np.abs(np.diff(p, axis=0)) > 0.0, axis=1)
    return t[keep], p[keep]


def velocity(t, p, half_window=0.2):
    """Central differences over +-half_window (non-uniform times)."""
    n = t.size
    v = np.full((n, p.shape[1]), np.nan)
    lo = np.searchsorted(t, t - half_window, side="left")
    hi = np.searchsorted(t, t + half_window, side="right") - 1
    for k in range(n):
        a, b = lo[k], hi[k]
        if b > a and t[b] - t[a] > 0.5 * half_window:
            v[k] = (p[b] - p[a]) / (t[b] - t[a])
    return v


def umeyama(src, dst, with_scale):
    """dst ~ s R src + t (least squares)."""
    mu_s, mu_d = src.mean(axis=0), dst.mean(axis=0)
    xs, xd = src - mu_s, dst - mu_d
    C = xd.T @ xs / src.shape[0]
    U, S, Vt = np.linalg.svd(C)
    D = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        D[2, 2] = -1.0
    R = U @ D @ Vt
    s = float(np.trace(np.diag(S) @ D) / (xs ** 2).sum() * src.shape[0]) if with_scale else 1.0
    t = mu_d - s * R @ mu_s
    return s, R, t


def rot_angles(R):
    yaw = math.degrees(math.atan2(R[1, 0], R[0, 0]))
    ang = math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(R) - 1.0) / 2.0))))
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, R[2, 2]))))
    return yaw, tilt, ang


# ------------------------------------------------------------------------------------------------------------
def read_bag(path, args):
    rb = rosio.RawBag(path)
    topics = [args.rtk_topic, args.pvt_topic, args.imu_topic, args.odom_topic]
    rtk, pvt, imu, odom = [], [], [], []
    for topic, datatype, raw, t_rec, pytype in rb.iter_raw(topics=topics):
        if topic == args.rtk_topic:
            rtk.append(gnss_msgs.navsatfix(rb.deserialize(pytype, raw), t_rec))
        elif topic == args.pvt_topic:
            pvt.append(gnss_msgs.pvt(rb.deserialize(pytype, raw), t_rec))
        elif topic == args.imu_topic:
            _seq, st, _f, used = rosio.parse_header(raw)
            g = struct.unpack_from("<3d", raw, used + 104)
            a = struct.unpack_from("<3d", raw, used + 200)
            imu.append((st, g[0], g[1], g[2], a[0], a[1], a[2], t_rec))
        elif topic == args.odom_topic:
            _seq, st, _f, used = rosio.parse_header(raw)
            (n,) = struct.unpack_from("<I", raw, used)
            off = used + 4 + n + 56 + 288
            lin = struct.unpack_from("<3d", raw, off)
            ang = struct.unpack_from("<3d", raw, off + 24)
            odom.append((st, lin[0], lin[1], lin[2], ang[2], t_rec))
    rb.close()
    return rtk, pvt, np.array(imu), np.array(odom)


def provenance(gt, rtk, rep, checks):
    """Which stamps and which projection produced the GT file (exact test against the RTK receiver fix)."""
    if not rtk:
        checks.add("P1", "GT provenance vs RTK fix topic", "SKIP", "RTK fix topic not in the bag")
        return None
    h = np.array([r["stamp"] for r in rtk])
    order = np.argsort(h, kind="stable")
    h = h[order]
    rtk = [rtk[i] for i in order]
    idx = np.clip(np.searchsorted(h, gt[:, 0]), 1, h.size - 1)
    left = np.abs(gt[:, 0] - h[idx - 1]) <= np.abs(gt[:, 0] - h[idx])
    j = np.where(left, idx - 1, idx)
    near = np.abs(gt[:, 0] - h[j])
    matched = near < 1e-6
    res = {"rtk_messages": len(rtk), "gt_rows": int(gt.shape[0]), "gt_stamp_equals_rtk_header_1us": float(np.mean(matched)),
           "gt_stamp_minus_rtk_header_s": robust.summary(gt[:, 0] - h[j])}
    rec = np.array([r["t_record"] for r in rtk])
    res["rtk_record_minus_header_ms"] = robust.summary((rec - h) * 1e3)
    st = collections.Counter(r["status"] for r in rtk)
    ct = collections.Counter(r["cov_type"] for r in rtk)
    sig = np.sqrt(np.array([r["cov"][0] for r in rtk]))
    res.update({"rtk_status_hist": {str(k): v for k, v in st.items()}, "rtk_cov_type_hist": {str(k): v for k, v in ct.items()},
                "rtk_sigma_e_m": robust.summary(sig), "rtk_sigma_e_values": sorted({round(float(x), 4) for x in sig})[:20]})
    lat_q = np.array([r["lat"] for r in rtk]) * 1e7
    lon_q = np.array([r["lon"] for r in rtk]) * 1e7
    res["latlon_on_1e-7_deg_grid"] = float(np.mean((np.abs(lat_q - np.round(lat_q)) < 1e-3) & (np.abs(lon_q - np.round(lon_q)) < 1e-3)))
    pos = np.array([[r["lat"], r["lon"], r["alt"]] for r in rtk])
    same = np.all(pos[1:] == pos[:-1], axis=1)
    runs, cur = [], 1
    for sflag in same:
        if sflag:
            cur += 1
        else:
            runs.append(cur)
            cur = 1
    runs.append(cur)
    res["copies_per_fix_median"] = float(np.median(runs))
    res["rtk_note"] = ("NavSatFix from mavros: status is STATUS_FIX for every fix_type >= 3 and the covariance comes from HDOP, so "
                       "RTK fixed/float cannot be told apart from this topic; lat/lon are MAVLink degE7 integers (1e-7 deg = 1.1 cm N, "
                       "0.9 cm E) when latlon_on_1e-7_deg_grid ~ 1")
    if matched.sum() < 10:
        rep["provenance"] = res
        checks.add("P1", "GT stamps = RTK fix header stamps", "FAIL", "only %d GT rows share a stamp with the RTK topic" % int(matched.sum()))
        return None
    lat = np.radians(np.array([rtk[k]["lat"] for k in j[matched]]))
    lon = np.radians(np.array([rtk[k]["lon"] for k in j[matched]]))
    alt = np.array([rtk[k]["alt"] for k in j[matched]])
    g = gt[matched, 1:4]
    k0 = int(np.argmin(gt[matched, 0]))
    hyps = {}
    enu = geodesy.geodetic_to_enu(lat, lon, alt, lat[k0], lon[k0], alt[k0])
    hyps["ENU at the first GT fix (exact, WGS-84)"] = enu
    zone = geodesy.utm_zone(math.degrees(lon[k0]))
    lon0 = -183.0 + 6.0 * zone
    E, N = geodesy.tm_forward(np.degrees(lat), np.degrees(lon), lon0)
    hyps["UTM zone %d (E, N, alt) minus first" % zone] = np.stack([E - E[k0], N - N[k0], alt - alt[k0]], axis=1)
    R0 = geodesy.RE_WGS84
    hyps["equirectangular (R=6378137 m) minus first"] = np.stack(
        [(lon - lon[k0]) * math.cos(lat[k0]) * R0, (lat - lat[k0]) * R0, alt - alt[k0]], axis=1)
    la_g, lo_g = geodesy.wgs84_to_gcj02(np.degrees(lat), np.degrees(lon))
    hyps["ENU of GCJ-02-shifted lat/lon"] = geodesy.geodetic_to_enu(np.radians(la_g), np.radians(lo_g), alt, math.radians(la_g[k0]), math.radians(lo_g[k0]), alt[k0])
    table = []
    best = None
    for name, H in hyps.items():
        direct = g - H
        rms_direct = float(np.sqrt(np.mean(np.sum(direct ** 2, axis=1))))
        s, R, t = umeyama(H, g, True)
        fit = g - (s * (R @ H.T).T + t)
        rms_fit = float(np.sqrt(np.mean(np.sum(fit ** 2, axis=1))))
        yaw, tilt, ang = rot_angles(R)
        row = {"hypothesis": name, "rms_direct_m": rms_direct, "max_direct_m": float(np.max(np.linalg.norm(direct, axis=1))),
               "sim3_scale": s, "sim3_yaw_deg": yaw, "sim3_tilt_deg": tilt, "sim3_rms_m": rms_fit}
        table.append(row)
        if best is None or rms_direct < best["rms_direct_m"]:
            best = row
    res["hypotheses"] = table
    res["best"] = best["hypothesis"]
    res["origin"] = {"lat_deg": math.degrees(lat[k0]), "lon_deg": math.degrees(lon[k0]), "h_m": float(alt[k0])}
    # how much GCJ-02 would distort the local geometry over this trajectory (relevance of the datum question)
    gcj_local = hyps["ENU of GCJ-02-shifted lat/lon"] - enu
    res["gcj02_local_distortion_m"] = float(np.max(np.linalg.norm(gcj_local - gcj_local.mean(axis=0), axis=1)))
    la0, lo0 = geodesy.wgs84_to_gcj02(math.degrees(lat[k0]), math.degrees(lon[k0]))
    dn = (float(la0) - math.degrees(lat[k0])) * math.radians(1.0) * geodesy.RE_WGS84
    de = (float(lo0) - math.degrees(lon[k0])) * math.radians(1.0) * geodesy.RE_WGS84 * math.cos(lat[k0])
    res["gcj02_absolute_shift_here_m"] = math.hypot(de, dn)
    rep["provenance"] = res
    # the GT file has 6 decimals and its source lat/lon are degE7 integers (1 cm); the producer's ENU arithmetic may
    # differ from ours at the sub-mm..mm level, a different projection shows up at the decimetre level (UTM ~0.5-2 m)
    exact = best["rms_direct_m"] < 5e-3
    checks.add("P1", "GT provenance: stamps and projection reproduced from the RTK fix (C-RTK2HP)", "PASS" if exact else "WARN",
               "%.1f%% of GT stamps equal %s header stamps (1 us); best hypothesis '%s' reproduces GT to %.2e m RMS (max %.2e m); other hypotheses: %s" % (
                   100 * res["gt_stamp_equals_rtk_header_1us"], RTK_TOPIC, best["hypothesis"], best["rms_direct_m"], best["max_direct_m"],
                   "; ".join("%s %.3g m" % (r["hypothesis"], r["rms_direct_m"]) for r in table if r is not best)))
    return {"lat0": lat[k0], "lon0": lon[k0], "h0": alt[k0], "exact": exact, "best": best["hypothesis"]}


def datum_and_alignment(gt, rtk, pvt, origin, rep, checks, lag_gt_minus_utc):
    """GCJ-02 test (absolute) and SE(3)/Sim(3) of GT vs the 9Ps PVT in ENU (structure of the residual)."""
    if not pvt:
        checks.add("P2", "datum: GT source vs 9Ps WGS-84 PVT", "SKIP", "no receiver PVT")
        return
    tp = np.array([gnsstime.gpst_to_utc(p["t_gpst"]) for p in pvt])
    lat = np.radians([p["lat"] for p in pvt])
    lon = np.radians([p["lon"] for p in pvt])
    h = np.array([p["h_ell"] for p in pvt])
    fix = np.array([p["fix_type"] >= 3 for p in pvt])
    res = {}
    # absolute datum test: RTK lat/lon (GT source) vs PVT lat/lon at the same instants
    if rtk:
        th = np.array([r["stamp"] for r in rtk])
        rl = np.radians([r["lat"] for r in rtk])
        ro = np.radians([r["lon"] for r in rtk])
        ra = np.array([r["alt"] for r in rtk])
        t_utc_rtk = th - lag_gt_minus_utc
        la = np.interp(tp, t_utc_rtk, rl)
        lo = np.interp(tp, t_utc_rtk, ro)
        al = np.interp(tp, t_utc_rtk, ra)
        ok = fix & (tp > t_utc_rtk[0]) & (tp < t_utc_rtk[-1])
        E = np.array([geodesy.enu_matrix(a, b) for a, b in zip(lat[ok], lon[ok])])
        d = np.array([Ei @ (geodesy.pos2ecef(a, b, c) - geodesy.pos2ecef(x, y, z)) for Ei, a, b, c, x, y, z in
                      zip(E, la[ok], lo[ok], al[ok], lat[ok], lon[ok], h[ok])])
        horiz = np.hypot(d[:, 0], d[:, 1]) if d.size else np.zeros(0)
        res["rtk_minus_pvt_horizontal_m"] = robust.summary(horiz)
        res["rtk_minus_pvt_up_m"] = robust.summary(d[:, 2]) if d.size else {}
        shift = rep.get("provenance", {}).get("gcj02_absolute_shift_here_m")
        res["gcj02_shift_here_m"] = shift
        if horiz.size:
            med = float(np.median(horiz))
            if shift and med > 0.5 * shift:
                verdict, word = "FAIL", "GCJ-02-like offset"
            else:
                verdict, word = "PASS", "WGS-84 (no GCJ-02 offset)"
            checks.add("P2", "datum: RTK fix (GT source) lat/lon vs 9Ps WGS-84 PVT", verdict,
                       "median horizontal difference %.1f m vs a GCJ-02 shift of %.0f m at this location ⇒ %s" % (med, shift or float("nan"), word))
    # structure of GT (local XYZ) vs PVT in ENU at the GT origin
    if origin is None:
        lat0, lon0, h0 = lat[0], lon[0], h[0]
    else:
        lat0, lon0, h0 = origin["lat0"], origin["lon0"], origin["h0"]
    enu = geodesy.geodetic_to_enu(lat, lon, h, lat0, lon0, h0)
    tg = gt[:, 0] - lag_gt_minus_utc
    ok = fix & (tp > tg[0]) & (tp < tg[-1])
    if ok.sum() < 30:
        checks.add("P3", "GT vs PVT-ENU SE(3)/Sim(3) residual structure", "SKIP", "fewer than 30 overlapping PVT epochs")
        rep["alignment"] = res
        return
    g = np.stack([np.interp(tp[ok], tg, gt[:, k]) for k in (1, 2, 3)], axis=1)
    src = enu[ok]
    tt = tp[ok]
    # horizontal SE(2)/Sim(2): a ground trajectory is nearly planar, so a 3-D fit's tilt is unobservable noise
    def fit2(src2, dst2, with_scale):
        """2-D Umeyama (proper rotation only; a 3-D fit on planar data could return an in-plane reflection)."""
        ms, md = src2.mean(axis=0), dst2.mean(axis=0)
        xs, xd = src2 - ms, dst2 - md
        U, S, Vt = np.linalg.svd(xd.T @ xs / len(src2))
        D = np.diag([1.0, 1.0 if np.linalg.det(U @ Vt) > 0 else -1.0])
        R_ = U @ D @ Vt
        s_ = float(np.trace(np.diag(S) @ D) / np.mean(np.sum(xs ** 2, axis=1))) if with_scale else 1.0
        t_ = md - s_ * R_ @ ms
        return s_, math.degrees(math.atan2(R_[1, 0], R_[0, 0])), t_
    fits = {}
    for name, ws in (("SE2", False), ("Sim2", True)):
        sc, yaw, t2 = fit2(src[:, :2], g[:, :2], ws)
        c, s_ = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
        pred = sc * (src[:, :2] @ np.array([[c, s_], [-s_, c]])) + t2
        r = g[:, :2] - pred
        A = np.column_stack([src[:, 0], src[:, 1], np.ones(src.shape[0])])
        coef_e, *_ = np.linalg.lstsq(A, r[:, 0], rcond=None)
        coef_n, *_ = np.linalg.lstsq(A, r[:, 1], rcond=None)
        fits[name] = {"scale": sc, "yaw_deg": yaw, "t_en": t2, "rms_e_m": float(np.sqrt(np.mean(r[:, 0] ** 2))),
                      "rms_n_m": float(np.sqrt(np.mean(r[:, 1] ** 2))),
                      "trend_per_km": [float(coef_e[0] * 1e3), float(coef_e[1] * 1e3), float(coef_n[0] * 1e3), float(coef_n[1] * 1e3)]}
    fits["vertical_offset_m"] = float(np.median(g[:, 2] - src[:, 2]))
    # block bootstrap (30 s blocks: PVT errors are correlated over tens of seconds) for honest yaw/scale sigmas
    rng = np.random.RandomState(0)
    blocks = np.floor((tt - tt[0]) / 30.0).astype(int)
    ub = np.unique(blocks)
    ys, ss = [], []
    for _ in range(200):
        pick = rng.choice(ub, ub.size, replace=True)
        idx = np.concatenate([np.where(blocks == b_)[0] for b_ in pick])
        if idx.size < 30:
            continue
        sc, yaw, _t = fit2(src[idx, :2], g[idx, :2], True)
        ys.append(yaw)
        ss.append(sc)
    spread = float(np.sqrt(np.mean(np.sum((src[:, :2] - src[:, :2].mean(axis=0)) ** 2, axis=1))))
    yaw_sd = float(np.std(ys)) if ys else float("nan")
    sc_sd = float(np.std(ss)) if ss else float("nan")
    res.update({"n": int(ok.sum()), "fits": fits, "horizontal_spread_m": spread, "yaw_sd_deg_block_bootstrap": yaw_sd,
                "scale_sd_block_bootstrap": sc_sd, "blocks": int(ub.size)})
    rep["alignment"] = res
    y = fits["SE2"]["yaw_deg"]
    sc = fits["Sim2"]["scale"]
    if not np.isfinite(yaw_sd) or yaw_sd > 1.0 or ub.size < 5:
        verdict = "SKIP"
        head = "not decisive: 9Ps PVT noise vs trajectory extent (horizontal spread %.0f m, %d x 30 s blocks) gives yaw σ %.2f° > 1°; the frame is decided by P1" % (
            spread, ub.size, yaw_sd)
    else:
        # the question is "ENU or a rotated/scaled projection": UTM would show ~1.25 deg / 1.25e-4 here, GCJ-02 only a
        # 1e-4 local distortion; tolerances are the larger of 3 sigma and 0.5 deg / 0.5 % (PVT multipath is correlated)
        ok_frame = abs(y) < max(0.5, 3.0 * yaw_sd) and abs(sc - 1.0) < max(5e-3, 3.0 * sc_sd)
        verdict = "PASS" if ok_frame else "WARN"
        head = "SE(2) yaw %+.3f° (block-bootstrap σ %.3f°), Sim(2) scale %.5f (σ %.5f)" % (y, yaw_sd, sc, sc_sd)
    checks.add("P3", "GT (local XYZ) vs 9Ps PVT in ENU at the GT origin: horizontal SE(2)/Sim(2) structure (ENU, no rotation/scale expected)", verdict,
               head + "; residual trend dE/dE %+.1f dN/dN %+.1f m/km; n=%d (residual magnitudes are the 9Ps PVT noise, not a result, A2)" % (
                   fits["SE2"]["trend_per_km"][0], fits["SE2"]["trend_per_km"][3], int(ok.sum())))


MIN_PEAK = 0.8


def _xc(t_a, a, t_b, b):
    r = robust.xcorr_offset(t_a, a, t_b, b, -1.0, 1.0, 0.01)
    if not r:
        return None
    # sigma_s: delay precision from the peak alone (robust.tde_sigma, Carter's high-SNR bound) — reported beside the
    # peak so that a sub-threshold link can be judged by what it measures, not only by the pre-registered 0.8 rule
    return {"lag_s": r["lag"], "peak": r["peak"], "width_s": r["width"], "at_edge": r["at_edge"],
            "sigma_s": robust.tde_sigma(r["peak"], r["curv_s"], r["duration_s"]),
            "valid": bool(r["peak"] >= MIN_PEAK and not r["at_edge"])}


def gt_kinematics(gt):
    tu, pu = update_series(gt[:, 0], gt[:, 1:4])
    v = velocity(tu, pu, 0.25)
    speed = np.linalg.norm(v[:, :2], axis=1)
    course = np.unwrap(np.arctan2(v[:, 1], v[:, 0]))
    ok = np.isfinite(speed) & (speed > 0.3)
    rate = np.zeros(tu.size)
    for k in range(1, tu.size - 1):
        if ok[k - 1] and ok[k + 1]:
            rate[k] = (course[k + 1] - course[k - 1]) / (tu[k + 1] - tu[k - 1])
    return tu, pu, speed, rate, ok


LEVER_V_MIN = 0.5                 # m/s: the antenna-lead term r_x/v is only fitted where 1/v is bounded


def lever_lead_fit(tu, rate, ok, speed, t_imu, yaw_rate, lags=None, smooth_s=0.15):
    """GT course rate against the gyro with the antenna-lead term (diagnostic of P4).

    For a ground vehicle whose no-slip point (instantaneous centre of the lateral motion) is at the IMU, an antenna
    r_x ahead of it moves sideways at omega*r_x, so its course is heading + atan(r_x omega / v) and
        course rate = omega + r_x d/dt(omega / v)  ~  omega(t + r_x / v)          (small angles)
    i.e. the course LEADS the heading by r_x / v (speed dependent, so it cannot be a clock offset). Fitted on moving
    samples (v > 0.5 m/s): course_rate(t) = omega(t + L) + r_x d/dt[omega(t + L) / v(t)], least squares in r_x for
    every L of a 5 ms grid, the L with the smallest residual refined by a parabola. L estimates the GT -> IMU clock lag
    without the odometry (independent of the chain); lag_no_lever_s is the same fit with r_x = 0 (the raw direct
    link). The gyro is smoothed (Gaussian, smooth_s) to the bandwidth of the GT course rate (5 Hz fixes, 0.5 s
    central differences), zero phase on both sides so no lag is introduced."""
    lags = np.arange(-1.0, 1.0 + 1e-9, 0.005) if lags is None else lags
    m = ok & np.isfinite(speed) & (speed > LEVER_V_MIN) & np.isfinite(rate)
    if m.sum() < 50 or t_imu.size < 100:
        return None
    tg, wz = robust.smooth_uniform(t_imu, yaw_rate, 100.0, smooth_s)
    # dv/dt on the contiguous GT series, before the slow samples are masked out (no differences across gaps)
    vdot_all = np.gradient(np.nan_to_num(speed), tu) if tu.size > 2 else np.zeros_like(tu)
    t = tu[m]
    c = rate[m]
    v = speed[m]
    vdot = vdot_all[m]
    wdot_g = np.gradient(wz, tg)
    inside = (t + lags.min() >= tg[0]) & (t + lags.max() <= tg[-1])
    t, c, v, vdot = t[inside], c[inside], v[inside], vdot[inside]
    if t.size < 50:
        return None
    rss, rx_all, rss0 = [], [], []
    for L in lags:
        w = np.interp(t + L, tg, wz)
        wd = np.interp(t + L, tg, wdot_g)
        x = wd / v - w * vdot / (v * v)             # d/dt(omega/v)
        y = c - w
        xx = float(x @ x)
        rx = float(x @ y) / xx if xx > 0.0 else 0.0
        r = y - rx * x
        rss.append(float(r @ r))
        rx_all.append(rx)
        rss0.append(float(y @ y))

    def refine(vals):
        vals = np.asarray(vals)
        k = int(np.argmin(vals))
        L = float(lags[k])
        if 0 < k < lags.size - 1:
            y0, y1, y2 = vals[k - 1], vals[k], vals[k + 1]
            den = y0 - 2.0 * y1 + y2
            if den > 0:
                L += 0.5 * (y0 - y2) / den * float(lags[1] - lags[0])
        return L, k

    L, k = refine(rss)
    L0, _ = refine(rss0)
    return {"lag_s": L, "r_x_m": float(np.interp(L, lags, rx_all)), "lag_no_lever_s": L0, "n": int(t.size),
            "speed_median_mps": float(np.median(v)), "inv_speed_mean": float(np.mean(1.0 / v)),
            "rms_resid_rad_s": float(math.sqrt(rss[k] / t.size)), "rms_resid_no_lever_rad_s": float(math.sqrt(min(rss0) / t.size)),
            "at_edge": bool(k == 0 or k == lags.size - 1)}


def time_base(gt, imu, odom, pvt, rep, checks, plots, out_dir, rtk=None, traj=None):
    """Offset between GT stamps and the Avia IMU header clock (= the estimator's LiDAR/IMU time base).

    Primary: a chain through the wheel odometry, whose speed matches the GT speed and whose yaw rate matches the gyro
    with high correlation: GT(t) ~ odom(t + L1) (speed) and odom(t) ~ IMU(t + L2) (yaw rate) => GT(t) ~ IMU(t + L1 + L2).
    A link counts only with peak correlation >= 0.8 and the peak inside +-1 s. The chain's own risk is a delay
    difference between the odometry's speed and yaw-rate channels (it adds to L1 + L2); two independent estimates
    bound it: the record-time plausibility (RTK and IMU transport delays assumed equal) must agree within 50 ms or the
    time base is ambiguous (FAIL), and the antenna-lead fit (lever_lead_fit, no odometry) is reported beside it.
    The raw direct link (GT course rate vs gyro) is not a clock estimate: the antenna ahead of the no-slip point makes
    the GT course LEAD the heading by r_x / v (Dark01/Outdoor01: direct - chain = +0.31/+0.20 s, consistent with a
    lead, not a lag). GPST relation: GT speed vs 9Ps PVT speed (UTC).
    The P4 verdict is the pre-registered rule above (TEST_PLAN §4, A40) and nothing below changes it. Diagnostics for
    the maintainer's decision when it fails: every link's delay precision (tde_sigma); the sub-threshold candidate
    chain; P4c = GT speed vs the speed of a GNSS-free replay on the estimator clock (traj; the caller passes it on
    development sequences only, A2). There is deliberately no GNSS route through the receiver PVT: its velocity is
    filtered and lags the replay's by 0.14-0.19 s on Grass02, so "GT speed vs PVT speed" measures that filter, not a
    clock (the GT - UTC value below is only used to line up the PVT positions for P2/P3/P5, where 0.2 s is immaterial).
    Returns (compensation [s] or None when the time base is undetermined/ambiguous, GT - UTC [s])."""
    tu, pu, speed, rate, ok = gt_kinematics(gt)
    res = {"gt_update_rate_hz": float(1.0 / np.median(np.diff(tu))) if tu.size > 2 else None, "moving_fraction": float(np.mean(ok)),
           "min_peak": MIN_PEAK}
    results = {}
    yaw_rate = None
    lever = None
    if imu.size:
        up = imu[:, 4:7].mean(axis=0)
        up /= np.linalg.norm(up)
        yaw_rate = imu[:, 1:4] @ up
        res["imu_up_axis_in_imu_frame"] = up.tolist()
        valid = np.interp(imu[:, 0], tu, ok.astype(float)) > 0.99
        r = _xc(tu, np.where(ok, rate, 0.0), imu[:, 0], np.where(valid, yaw_rate, 0.0))
        if r:
            results["gt_course_rate__imu_yaw_rate"] = r
        lever = lever_lead_fit(tu, rate, ok, speed, imu[:, 0], yaw_rate)
    res["lever_lead_fit"] = lever
    if odom.size:
        r = _xc(tu, np.nan_to_num(speed), odom[:, 0], np.abs(odom[:, 1]))
        if r:
            results["gt_speed__odom_speed"] = r
        if yaw_rate is not None:
            r = _xc(odom[:, 0], odom[:, 4], imu[:, 0], yaw_rate)
            if r:
                results["odom_yaw_rate__imu_yaw_rate"] = r
    if pvt:
        tp = np.array([gnsstime.gpst_to_utc(p["t_gpst"]) for p in pvt])
        ps = np.array([math.hypot(p["vel_n"], p["vel_e"]) for p in pvt])
        r = _xc(tu, np.nan_to_num(speed), tp, ps)
        if r:
            results["gt_speed__pvt_speed_utc"] = r
    res["xcorr"] = results
    l1, l2 = results.get("gt_speed__odom_speed"), results.get("odom_yaw_rate__imu_yaw_rate")
    direct = results.get("gt_course_rate__imu_yaw_rate")
    lag, how = None, None
    if l1 and l2 and l1["valid"] and l2["valid"]:
        lag, how = l1["lag_s"] + l2["lag_s"], "chain GT->odom (speed) + odom->IMU (yaw rate)"
    res["gt_to_imu_lag_s"] = lag
    res["method"] = how
    # independent plausibility: if the RTK fix and the IMU sample of one instant reach the host with similar delays,
    # the lag ~ median(record - stamp | RTK) - median(record - stamp | IMU)
    rec_based = None
    if rtk and imu.size and imu.shape[1] >= 8:
        rec_based = float(np.median([r["t_record"] - r["stamp"] for r in rtk]) - np.median(imu[:, 7] - imu[:, 0]))
    res["record_time_based_lag_s"] = rec_based
    lever_txt = ""
    if lever:
        lever_txt = "; antenna-lead fit (no odometry) L %+.1f ms, r_x %+.2f m (median speed %.2f m/s, n=%d; r_x = 0 gives %+.1f ms)" % (
            1e3 * lever["lag_s"], lever["r_x_m"], lever["speed_median_mps"], lever["n"], 1e3 * lever["lag_no_lever_s"])
    # the chain of the two odometry links when at least one misses the peak rule (reported, never used)
    cand = None
    if lag is None and l1 and l2 and not (l1["at_edge"] or l2["at_edge"]):
        cand = {"lag_s": l1["lag_s"] + l2["lag_s"], "sigma_s": _hyp(l1.get("sigma_s"), l2.get("sigma_s")),
                "peaks": [l1["peak"], l2["peak"]]}
    res["candidate_chain"] = cand
    comp = None
    if lag is None:
        verdict = "FAIL"
        ev = ("no valid correlation chain (peaks: %s): GT time base undetermined by the pre-registered rule; AMENDMENTS A54: "
              "the record-time value is used instead (gt_timebase.yaml)" % {k: round(v["peak"], 2) for k, v in results.items()})
        if cand:
            ev += ("; for the record, the chain of the sub-threshold links would give %+.1f ms (delay precision %.1f ms, "
                   "peaks %.2f/%.2f)" % (1e3 * cand["lag_s"], 1e3 * cand["sigma_s"], cand["peaks"][0], cand["peaks"][1]))
        if rec_based is not None:
            ev += "; record-time plausibility %+.1f ms" % (1e3 * rec_based)
        ev += lever_txt
    elif rec_based is not None and abs(lag - rec_based) > AGREE_S:
        verdict = "FAIL"
        ev = ("chain %+.1f ms and record-time plausibility %+.1f ms disagree by more than %.0f ms: GT time base ambiguous; "
              "AMENDMENTS A54: the record-time value is used (gt_timebase.yaml)" % (1e3 * lag, 1e3 * rec_based, 1e3 * AGREE_S)) + lever_txt
    else:
        comp = lag if abs(lag) > COMP_THRESHOLD_S else 0.0
        verdict = "PASS" if comp == 0.0 else "WARN"
        ev = "GT stamp -> Avia IMU header clock: %+.1f ms by %s%s" % (
            1e3 * lag, how, "; > 20 ms: gt_audited.tum shifted by it" if comp else "; <= 20 ms: no compensation")
        if rec_based is not None:
            ev += "; record-time plausibility %+.1f ms (assumes equal transport delays; agreement within %.0f ms required)" % (
                1e3 * rec_based, 1e3 * AGREE_S)
        ev += lever_txt
        if direct:
            ev += "; raw direct course-rate link %+.1f ms (peak %.2f): %+.1f ms against the chain = antenna lead r_x/v, not a clock term" % (
                1e3 * direct["lag_s"], direct["peak"], 1e3 * (direct["lag_s"] - lag))
    res["compensation_s"] = comp
    rep["time_base"] = res
    links = "; ".join("%s %+.1f ms (peak %.3f, delay precision %s ms)" % (
        k, 1e3 * v["lag_s"], v["peak"], "%.1f" % (1e3 * v["sigma_s"]) if np.isfinite(v.get("sigma_s", float("nan"))) else "—")
        for k, v in results.items())
    checks.add("P4", "GT time base vs the estimator clock (Avia IMU header) by cross-correlation; > 20 ms compensated", verdict, ev + "; links: " + links)
    pv = results.get("gt_speed__pvt_speed_utc")
    lag_utc = -pv["lag_s"] if pv and pv["valid"] else 0.0
    res["gt_minus_utc_s"] = lag_utc if pv and pv["valid"] else None
    # ---- diagnostics (no verdict): independent routes from the GT stamps to the estimator clock ----
    refs = []
    if lag is not None:
        refs.append(("odometry chain", lag))
    elif cand:
        refs.append(("sub-threshold odometry chain", cand["lag_s"]))
    if rec_based is not None:
        refs.append(("record-time plausibility", rec_based))

    def against(x):
        return "; ".join("%+.1f ms against the %s" % (1e3 * (x - v), n) for n, v in refs) or "no other estimate"
    if traj is not None:
        title = "diagnostic: GT time base via the replay = GT speed vs |v| of the GNSS-free replay (estimator clock; development set only)"
        r = _xc(tu, np.nan_to_num(speed), traj["t"], np.linalg.norm(traj["v"], axis=1))
        if r and r["valid"]:
            res["replay_chain"] = r
            checks.add("P4c", title, "INFO", "GT stamp -> estimator clock %+.1f ms (peak %.3f, delay precision %.1f ms); %s" % (
                1e3 * r["lag_s"], r["peak"], 1e3 * r["sigma_s"], against(r["lag_s"])))
        else:
            res["replay_chain"] = r
            checks.add("P4c", title, "SKIP", "no valid peak (%s)" % ("peak %.2f%s" % (r["peak"], ", at the search edge" if r["at_edge"] else "") if r else "no overlap"))
    return comp, lag_utc


def _hyp(a, b):
    a = a if a is not None and np.isfinite(a) else float("nan")
    b = b if b is not None and np.isfinite(b) else float("nan")
    return float(math.hypot(a, b))


def decide_timebase(tb, p4_verdict):
    """AMENDMENTS A54: the GT time base the evaluation uses, by a fixed rule on what time_base() measured.

      * P4 PASS/WARN (the pre-registered chain rule, A40) -> method `chain`: the odometry chain's value;
      * P4 FAIL (no valid chain, or chain and record time disagree by > 50 ms) -> method `record_time`: the record-time
        plausibility (median record - stamp of the RTK fix minus that of the Avia IMU; equal transport delays assumed);
      * neither available -> method `none`: 0, flagged for the table.
    The value is GT stamp -> estimator clock; the applied shift follows TEST_PLAN §4 (> 20 ms compensated, else 0), the
    same for every method. The replay route P4c is never used (development sequences only, and it is the estimator's
    own output). Confidence: chain = high (two correlations >= 0.8 and the record-time cross-check within 50 ms);
    record_time = medium when a sub-threshold chain agrees within 50 ms, low otherwise (and when a valid chain
    contradicts it); none = none. uncertainty_s: chain = the two links' peak-width precisions combined (Carter bound;
    conservative for the smooth GT speed, e.g. 114 ms on Grass01) capped at 50 ms, the agreement with the record-time
    value that P4 PASS requires; record_time = 50 ms, that same agreement bound (A40; observed 2.6-18.5 ms on the six
    audited sequences with a chain or sub-threshold chain)."""
    chain = tb.get("gt_to_imu_lag_s")
    rec = tb.get("record_time_based_lag_s")
    cand = tb.get("candidate_chain")
    xc = tb.get("xcorr", {})
    out = {"rule": "AMENDMENTS A54: P4 pass -> chain; P4 fail -> record-time plausibility; neither -> 0 (flagged); "
                   "applied shift = value if |value| > %.0f ms else 0" % (1e3 * COMP_THRESHOLD_S),
           "p4_verdict": p4_verdict, "chain_s": chain, "record_time_s": rec,
           "sub_threshold_chain_s": cand["lag_s"] if cand else None,
           "replay_route_p4c_s": (tb.get("replay_chain") or {}).get("lag_s") if (tb.get("replay_chain") or {}).get("valid") else None}
    if p4_verdict in ("PASS", "WARN") and chain is not None:
        l1, l2 = xc.get("gt_speed__odom_speed", {}), xc.get("odom_yaw_rate__imu_yaw_rate", {})
        carter = _hyp(l1.get("sigma_s"), l2.get("sigma_s"))
        out.update({"method": "chain", "offset_s": float(chain), "confidence": "high",
                    "uncertainty_s": min(carter, AGREE_S) if np.isfinite(carter) else AGREE_S,
                    "uncertainty_basis": "peak-width precisions of the two chain links combined (Carter bound, %.1f ms), capped by "
                                         "the 50 ms agreement with the record-time value that P4 PASS requires" % (1e3 * carter), "flag": ""})
    elif rec is not None:
        corroborated = cand is not None and abs(cand["lag_s"] - rec) <= AGREE_S
        contradicted = chain is not None and abs(chain - rec) > AGREE_S
        out.update({"method": "record_time", "offset_s": float(rec),
                    "confidence": "medium" if corroborated and not contradicted else "low",
                    "uncertainty_s": AGREE_S,
                    "uncertainty_basis": "the agreement P4 requires between the record-time value and a chain (A40)",
                    "flag": "P4 FAIL: record-time value (A54)"})
    else:
        out.update({"method": "none", "offset_s": 0.0, "confidence": "none", "uncertainty_s": None,
                    "uncertainty_basis": "no estimate", "flag": "no GT time-base estimate: 0 used (A54)"})
    out["applied_shift_s"] = out["offset_s"] if abs(out["offset_s"]) > COMP_THRESHOLD_S else 0.0
    return out


def write_timebase_yaml(path, seq, tb):
    """gt_timebase.yaml: flat `key: value` lines (strings quoted, null for missing numbers), readable with any YAML
    parser; the same dict is in gt_audit.json under "timebase"."""
    keys = ("method", "offset_s", "applied_shift_s", "uncertainty_s", "confidence", "flag", "p4_verdict", "chain_s",
            "record_time_s", "sub_threshold_chain_s", "replay_route_p4c_s", "uncertainty_basis", "rule")
    L = ["# GT stamp -> estimator clock (Avia IMU/LiDAR header) time base of %s, tools/gt_audit.py (tool_version %d)" % (seq, TOOL_VERSION),
         "# gt_audited.tum = gt_clean.tum with applied_shift_s added to every stamp (AMENDMENTS A54)",
         "sequence: %s" % json.dumps(seq)]
    for k in keys:
        v = tb.get(k)
        if v is None or (isinstance(v, float) and not np.isfinite(v)):
            L.append("%s: null" % k)
        elif isinstance(v, str):
            L.append("%s: %s" % (k, json.dumps(v, ensure_ascii=False)))
        else:
            L.append("%s: %r" % (k, float(v)))
    with open(path, "w") as f:
        f.write("\n".join(L) + "\n")


SPEED_SIGMA_FLOOR = 0.03


def fix_series(t, p, min_dt=0.15):
    """One row per RTK fix: a row is a new fix when its value changed or >= min_dt passed since the last kept row, so
    republished copies (M3DGR: three per 5 Hz fix, stamps within ~20 ms; or a 15 Hz republish) collapse, while a parked
    robot's unchanged fixes are kept (unlike update_series)."""
    keep = np.zeros(t.size, dtype=bool)
    keep[0] = True
    last = 0
    for k in range(1, t.size):
        if np.any(p[k] != p[last]) or t[k] - t[last] >= min_dt:
            keep[k] = True
            last = k
    return t[keep], p[keep]


def near_any(t, t_events, half):
    """Rows of t within +-half of any event time (sorted search, no quadratic loop)."""
    t_events = np.sort(np.asarray(t_events, dtype=float))
    if t_events.size == 0:
        return np.zeros(np.asarray(t).size, dtype=bool)
    idx = np.searchsorted(t_events, t)
    left = np.abs(t - t_events[np.clip(idx - 1, 0, t_events.size - 1)])
    right = np.abs(t_events[np.clip(idx, 0, t_events.size - 1)] - t)
    return np.minimum(left, right) <= half


JUMP_MIN_M = 0.5                  # RTK jump floor: 2.5 m/s of velocity change within one 0.2 s fix interval
JUMP_K = 10.0                     # ... and 10 robust sigma of the sequence's own fix-to-fix residuals
JUMP_NEIGHBOURS = 3               # fix intervals on each side for the local velocity (median)
PAIR_TOL = 0.5                    # a return jump cancels an open one within half the larger jump's size


def rtk_jumps(t, p, neighbours=JUMP_NEIGHBOURS):
    """RTK position discontinuities (AMENDMENTS A54 "RTK 跳变/毛刺"): for every fix interval k the residual
    r_k = dp_k - v_local * dt_k, v_local = median velocity of the `neighbours` intervals on each side (robust to the
    jump itself). A jump: |r_h| > max(JUMP_MIN_M, JUMP_K sigma_h) or |r_v| > max(JUMP_MIN_M, JUMP_K sigma_v), sigma
    from the sequence's own residuals (horizontal: median |r_h| / 1.1774, the Rayleigh median; vertical: median
    |r_v| / 0.6745). 0.5 m in one 0.2 s interval means 2.5 m/s of velocity change (>= 6 m/s^2 sustained), which the
    ground platform cannot do; on the development set the residuals' sigma is 0.01-0.03 m and the RTK mode changes
    are 0.8-6 m (Grass01 2.8 m horizontal / 6.1 m vertical at +203 s, back at +217 s; Grass02 0.8 / 1.9 m at +109 s,
    back at +115 s). t, p: the rows where the position changes (update_series), so that a frozen fix's catch-up is
    divided by the whole freeze. Returns dict(sigma_h, sigma_v, thr_h, thr_v, jumps)."""
    dt = np.diff(t)
    ok = dt > 1e-6
    v = np.zeros((dt.size, 3))
    v[ok] = np.diff(p, axis=0)[ok] / dt[ok, None]
    n = dt.size
    r = np.zeros((n, 3))
    for k in range(n):
        nb = [j for j in range(k - neighbours, k + neighbours + 1) if j != k and 0 <= j < n and ok[j]]
        if nb and ok[k]:
            r[k] = (v[k] - np.median(v[nb], axis=0)) * dt[k]
    rh = np.hypot(r[:, 0], r[:, 1])
    rv = np.abs(r[:, 2])
    sig_h = float(np.median(rh) / 1.1774) if n else float("nan")
    sig_v = float(np.median(rv) / 0.6745) if n else float("nan")
    thr_h = max(JUMP_MIN_M, JUMP_K * sig_h) if np.isfinite(sig_h) else JUMP_MIN_M
    thr_v = max(JUMP_MIN_M, JUMP_K * sig_v) if np.isfinite(sig_v) else JUMP_MIN_M
    jumps = [{"t": float(t[k + 1]), "t_before": float(t[k]), "d_enu_m": [float(x) for x in r[k]], "size_m": float(np.linalg.norm(r[k])),
              "h_m": float(rh[k]), "v_m": float(r[k, 2])}
             for k in range(n) if rh[k] > thr_h or rv[k] > thr_v]
    return {"sigma_h_m": sig_h, "sigma_v_m": sig_v, "thr_h_m": thr_h, "thr_v_m": thr_v, "n_intervals": int(n), "jumps": jumps}


def rtk_levels(jumps, span, tol=PAIR_TOL):
    """Pair the jumps into displaced plateaus: in time order, a jump whose vector cancels the most recent open jump
    (|d_open + d| <= tol * max(|d_open|, |d|)) closes it — the GT between the two sits on a displaced level (an RTK
    solution-mode change and its recovery; a spike of one fix is the shortest plateau); otherwise it opens one.
    A jump left open at the end splits the GT into two levels with no way to tell which is right: the side that is
    shorter within `span` (the bag's record span) is taken as the displaced one. Returns (plateaus, unpaired) with
    plateaus = [{t_from, t_to, jumps}] (t_from = the time of the last good fix before the first jump, t_to = the
    first fix after the return jump)."""
    stack, plateaus, unpaired = [], [], []
    for j in jumps:
        d = np.array(j["d_enu_m"])
        if stack:
            o = stack[-1]
            do = np.array(o["d_enu_m"])
            if np.linalg.norm(do + d) <= tol * max(np.linalg.norm(do), np.linalg.norm(d)):
                stack.pop()
                plateaus.append({"t_from": o["t_before"], "t_to": j["t"], "duration_s": j["t"] - o["t_before"], "jumps": [o, j],
                                 "residual_m": float(np.linalg.norm(do + d))})
                continue
        stack.append(j)
    lo, hi = float(span[0]), float(span[1])
    for j in stack:
        before = max(0.0, min(j["t_before"], hi) - lo)          # the two sides, clipped to the span
        after = max(0.0, hi - max(j["t"], lo))
        side = (lo, j["t"]) if before <= after else (j["t_before"], hi)
        unpaired.append({"jump": j, "masked_side": "before" if before <= after else "after", "t_from": side[0], "t_to": side[1],
                         "duration_s": min(before, after)})
    return plateaus, unpaired


MASK_BITS = {1: "h_acc>1m", 2: "num_sv<8", 4: "speed_spike", 8: "gap_or_stall", 16: "no_pvt_within_0.2s",
             32: "outside_the_bag_record_span", 64: "rtk_jump_plateau"}
ATE_EXCLUDE_BITS = 8 | 32 | 64    # GT defects: gaps/stalls, no sensor data, RTK jumps/plateaus (A54); see trust_mask


def mask_intervals(t, reason, bits):
    """Runs of consecutive GT rows with (reason & bits) != 0 as [t_first, t_last] (GT stamps)."""
    m = (reason & bits) != 0
    out = []
    k = 0
    while k < t.size:
        if m[k]:
            j = k
            while j + 1 < t.size and m[j + 1]:
                j += 1
            out.append((float(t[k]), float(t[j])))
            k = j + 1
        else:
            k += 1
    return out


def trust_mask(gt, pvt, lag_gt_minus_utc, rep, checks, out_dir, bag_span=None, audited_shift_s=0.0):
    """TEST_PLAN §4 mask: 9Ps h_acc > 1 m or num_sv < 8; GT speed spike > 3 sigma or gaps > 0.5 s; "前后 2 s" read as
    applying to every criterion (a receiver-quality flag is widened like a GT defect: receiver filters report a
    degradation late and recover late). The receiver criteria without widening are reported beside it.
    GT rows outside the bag's record span (bit 32; e.g. Occlusion03's GT runs past its truncated bag, A26) have no
    sensor data at all: they are never trusted, and every fraction is taken over the rows inside the span (GT stamps
    and record times are both host-clock times).
    AMENDMENTS A54 adds bit 64: RTK jumps and the displaced plateaus between a jump and its return (rtk_jumps /
    rtk_levels), widened by the same 2 s. Masked durations are reported per criterion in seconds of GT time inside the
    bag span. gt_mask_intervals.csv lists every masked run of rows with its bits, in GT stamps and in the audited time
    base (+ audited_shift_s), and marks ate_exclude for the GT-defect bits (8 gaps/stalls, 32 no sensor data, 64 RTK
    jumps: "掩码内的时段不计入 ATE", A54); the receiver-quality bits (1, 2) and the speed-spike bit (4) are reported
    but not marked — on the development set they mask 100 % (h_acc of a standalone receiver) and 54-59 % (real grass
    motion) and are not ruled on (INTERFACE_CHANGES WP-g P2b)."""
    t = gt[:, 0]
    n = t.size
    reason = np.zeros(n, dtype=int)            # bit0 h_acc, bit1 num_sv, bit2 speed spike, bit3 gap/stall, bit4 no PVT
    raw_rx = np.zeros(n, dtype=bool)
    if pvt:
        tp = np.array([gnsstime.gpst_to_utc(p["t_gpst"]) for p in pvt]) + lag_gt_minus_utc
        hacc = np.array([p["h_acc"] for p in pvt])
        nsv = np.array([p["num_sv"] for p in pvt])
        idx = np.clip(np.searchsorted(tp, t), 1, tp.size - 1)
        j = np.where(np.abs(t - tp[idx - 1]) <= np.abs(t - tp[idx]), idx - 1, idx)
        near = np.abs(t - tp[j]) < 0.2
        raw_rx = near & ((hacc[j] > 1.0) | (nsv[j] < 8))
        reason |= np.where(near_any(t, tp[hacc > 1.0], MASK_WIDEN_S), 1, 0)
        reason |= np.where(near_any(t, tp[nsv < 8], MASK_WIDEN_S), 2, 0)
        reason |= np.where(~near, 16, 0)
    tf, pf = fix_series(t, gt[:, 1:4])
    sp = np.full(tf.size, np.nan)
    sp[1:-1] = np.linalg.norm(pf[2:, :2] - pf[:-2, :2], axis=1) / (tf[2:] - tf[:-2])
    okv = np.isfinite(sp)
    spikes_t = []
    sig = float("nan")
    if okv.sum() > 20:
        v = sp[okv]
        med = np.array([np.median(v[max(0, k - 3):k + 4]) for k in range(v.size)])
        resid = v - med
        moving = med > 0.1
        # sigma over moving samples (stationary samples have zero residual), floored at the degE7 quantisation
        # of the source (1e-7 deg ~ 1.1 cm over a 0.4 s difference baseline -> ~0.03 m/s)
        sig = robust.robust_std(resid[moving]) if moving.sum() > 20 else robust.robust_std(resid)
        bad = np.abs(resid) > 3.0 * max(sig, SPEED_SIGMA_FLOOR)
        spikes_t = tf[okv][bad]
    gaps_t = [(t[i], t[i + 1]) for i in np.where(np.diff(t) > 0.5)[0]]
    # stall: the fix value frozen for > 0.5 s right after the antenna was moving (> 0.2 m/s); a parked robot's
    # quantised position legitimately repeats, so only frozen-while-moving counts
    stalls = []
    k = 0
    while k < tf.size - 1:
        j = k
        while j + 1 < tf.size and np.all(pf[j + 1] == pf[k]):
            j += 1
        if tf[j] - tf[k] > 0.5 and k >= 2 and np.isfinite(sp[k - 1]) and sp[k - 1] > 0.2:
            stalls.append((tf[k], tf[min(j + 1, tf.size - 1)]))
        k = j + 1
    for ts in spikes_t:
        reason |= np.where(np.abs(t - ts) <= MASK_WIDEN_S, 4, 0)
    for a, b in gaps_t + stalls:
        reason |= np.where((t >= a - MASK_WIDEN_S) & (t <= b + MASK_WIDEN_S), 8, 0)
    inside = np.ones(n, dtype=bool)
    if bag_span is not None:
        inside = (t >= bag_span[0] - 1.0) & (t <= bag_span[1] + 1.0)
        reason |= np.where(inside, 0, 32)
    # A54: RTK jumps / plateaus on the positions that change (update_series): a stall's catch-up then spans the whole
    # freeze (real motion, bit 8), not one fix interval
    tu, pu = update_series(t, gt[:, 1:4])
    rj = rtk_jumps(tu, pu)
    span = (float(bag_span[0]), float(bag_span[1])) if bag_span is not None else (float(t[0]), float(t[-1]))
    plateaus, unpaired = rtk_levels(rj["jumps"], span)
    rtk_iv = [(p_["t_from"], p_["t_to"]) for p_ in plateaus] + [(u["t_from"], u["t_to"]) for u in unpaired]
    for a, b in rtk_iv:
        reason |= np.where((t >= a - MASK_WIDEN_S) & (t <= b + MASK_WIDEN_S), 64, 0)
    trusted = reason & (1 | 2 | 4 | 8 | 32 | 64) == 0
    with open(os.path.join(out_dir, "trust_mask.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "trusted", "trusted_gt_criteria_only", "reason_bits(1=h_acc>1m,2=num_sv<8,4=speed_spike,8=gap_or_stall,64=rtk_jump_plateau: each within +-2 s;16=no_pvt_within_0.2s;32=outside_the_bag_record_span)",
                    "ate_exclude(bits 8|32|64)"])
        for a, b, c in zip(t, trusted, reason):
            w.writerow(["%.6f" % a, int(b), int(c & (4 | 8 | 32 | 64) == 0), int(c), int(c & ATE_EXCLUDE_BITS != 0)])
    trusted_gt_only = reason & (4 | 8 | 32 | 64) == 0
    m = inside if inside.any() else np.ones(n, dtype=bool)
    # durations [s] of GT time inside the bag span: each row stands for the time to the next row (gaps > 0.5 s
    # count as 0.5 s: a gap is masked, but its length is not GT coverage)
    dt_row = np.minimum(np.diff(t, append=t[-1]), 0.5)
    dur = lambda bits: float(np.sum(dt_row[m & ((reason & bits) != 0)]))
    durations = {name: dur(bit) for bit, name in MASK_BITS.items() if bit != 32}
    durations.update({"span_s": float(np.sum(dt_row[m])), "ate_exclude": dur(ATE_EXCLUDE_BITS & ~32),
                      "pre_registered_all": dur(1 | 2 | 4 | 8 | 64)})
    # intervals per group, so that a criterion that masks everything (h_acc on a standalone receiver) does not swallow
    # the GT-defect intervals the evaluation has to cut out: ate_exclude=1 rows = runs of the bits 8|32|64 only,
    # ate_exclude=0 rows = runs of the reported-only bits 1|2|4|16
    iv = []
    for bits, ate in ((ATE_EXCLUDE_BITS, 1), (1 | 2 | 4 | 16, 0)):
        for a, b in mask_intervals(t, reason, bits):
            rb = int(np.bitwise_or.reduce(reason[(t >= a) & (t <= b)] & bits))
            iv.append((a, b, rb, ate))
    with open(os.path.join(out_dir, "gt_mask_intervals.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_from_gt", "t_to_gt", "t_from_audited", "t_to_audited", "reason_bits", "reasons", "ate_exclude"])
        for a, b, rb, ate in sorted(iv, key=lambda x: (x[0], -x[3])):
            w.writerow(["%.6f" % a, "%.6f" % b, "%.6f" % (a + audited_shift_s), "%.6f" % (b + audited_shift_s), rb,
                        "+".join(nm for bit, nm in MASK_BITS.items() if rb & bit), ate])
    res = {"rows": int(n), "rows_outside_bag": int(np.sum(~inside)), "bag_span": list(bag_span) if bag_span is not None else None,
           "trusted_frac": float(np.mean(trusted[m])), "trusted_frac_gt_criteria_only": float(np.mean(trusted_gt_only[m])),
           "speed_sigma_mps": sig, "speed_sigma_floor_mps": SPEED_SIGMA_FLOOR,
           "frac_h_acc_gt_1m": float(np.mean((reason & 1 > 0)[m])), "frac_num_sv_lt_8": float(np.mean((reason & 2 > 0)[m])),
           "frac_receiver_criteria_unwidened": float(np.mean(raw_rx[m])), "widen_s": MASK_WIDEN_S,
           "frac_speed_spike": float(np.mean((reason & 4 > 0)[m])), "frac_gap_or_stall": float(np.mean((reason & 8 > 0)[m])),
           "frac_no_pvt": float(np.mean((reason & 16 > 0)[m])), "n_speed_spikes": int(len(spikes_t)), "n_gaps": len(gaps_t), "n_stalls": len(stalls),
           "stalls": [(float(a - t[0]), float(b - a)) for a, b in stalls[:20]], "gaps": [(float(a - t[0]), float(b - a)) for a, b in gaps_t[:20]],
           "spike_times_rel": [float(x - t[0]) for x in spikes_t[:50]],
           "frac_rtk_jump_plateau": float(np.mean((reason & 64 > 0)[m])), "masked_s": durations,
           "rtk": {"sigma_h_m": rj["sigma_h_m"], "sigma_v_m": rj["sigma_v_m"], "thr_h_m": rj["thr_h_m"], "thr_v_m": rj["thr_v_m"],
                   "n_fix_intervals": rj["n_intervals"], "jumps": [dict(j, t_rel=j["t"] - t[0]) for j in rj["jumps"]],
                   "plateaus": [{"t_from": p_["t_from"], "t_to": p_["t_to"], "t_from_rel": p_["t_from"] - t[0], "duration_s": p_["duration_s"],
                                 "return_residual_m": p_["residual_m"], "jump_sizes_m": [p_["jumps"][0]["size_m"], p_["jumps"][1]["size_m"]]}
                                for p_ in plateaus],
                   "unpaired": [{"t": u["jump"]["t"], "t_rel": u["jump"]["t"] - t[0], "size_m": u["jump"]["size_m"], "masked_side": u["masked_side"],
                                 "masked_from": u["t_from"], "masked_to": u["t_to"]} for u in unpaired]},
           "ate_exclude_bits": ATE_EXCLUDE_BITS, "audited_shift_s": audited_shift_s}
    if pvt:
        res["pvt_h_acc_m"] = robust.summary([p["h_acc"] for p in pvt])
        res["pvt_num_sv"] = robust.summary([p["num_sv"] for p in pvt])
    rep["trust_mask"] = res
    verdict = "INFO" if res["trusted_frac"] > 0.5 else "WARN"
    checks.add("P5", "pre-registered trust mask (9Ps h_acc > 1 m or num_sv < 8; speed spikes > 3σ; gaps/stalls > 0.5 s; each ±2 s)", verdict,
               "trusted %.1f%% (GT-derived criteria alone: %.1f%%); masked by h_acc>1 m %.1f%%, num_sv<8 %.1f%% (receiver criteria before the ±2 s widening %.1f%%), speed spikes %.1f%% (%d; σ %.3f m/s, floor %.2f), gaps/stalls %.1f%% (%d/%d)%s" % (
                   100 * res["trusted_frac"], 100 * res["trusted_frac_gt_criteria_only"], 100 * res["frac_h_acc_gt_1m"], 100 * res["frac_num_sv_lt_8"],
                   100 * res["frac_receiver_criteria_unwidened"],
                   100 * res["frac_speed_spike"], res["n_speed_spikes"], sig if np.isfinite(sig) else float("nan"), SPEED_SIGMA_FLOOR,
                   100 * res["frac_gap_or_stall"], res["n_gaps"], res["n_stalls"],
                   ("; fractions over the %d GT rows inside the bag's record span (%d rows after/before it: no sensor data, never trusted)" % (
                       int(np.sum(inside)), res["rows_outside_bag"])) if res["rows_outside_bag"] else "") + (
                   "; the h_acc criterion masks most rows because the 9Ps is a standalone (non-RTK) receiver: median h_acc %.2f m — the pre-registered threshold needs a decision" % res["pvt_h_acc_m"]["median"]
                   if pvt and res["frac_h_acc_gt_1m"] > 0.5 else ""))
    rk = res["rtk"]
    desc = "; ".join(["plateau +%.1f..+%.1f s (%.1f s; jumps %.2f/%.2f m, return residual %.2f m)" % (
        p_["t_from_rel"], p_["t_from_rel"] + p_["duration_s"], p_["duration_s"], p_["jump_sizes_m"][0], p_["jump_sizes_m"][1],
        p_["return_residual_m"]) for p_ in rk["plateaus"]] + ["UNPAIRED jump at +%.1f s (%.2f m): the %s side (%.1f s) taken as displaced" % (
        u["t_rel"], u["size_m"], u["masked_side"], u["masked_to"] - u["masked_from"]) for u in rk["unpaired"]])
    checks.add("P7", "RTK jumps / spikes (A54): fix-to-fix residual against the local velocity > max(0.5 m, 10σ); a jump and its return = displaced plateau, masked ±2 s",
               "WARN" if rk["unpaired"] else "INFO",
               "%d jump(s) (thresholds h %.2f / v %.2f m; residual σ h %.3f / v %.3f m over %d fix intervals)%s; masked %.1f s of %.1f s (%.2f%%); "
               "ATE-excluded (gaps/stalls + RTK) %.1f s" % (
                   len(rk["jumps"]), rk["thr_h_m"], rk["thr_v_m"], rk["sigma_h_m"], rk["sigma_v_m"], rk["n_fix_intervals"],
                   (": " + desc) if desc else "", durations["rtk_jump_plateau"], durations["span_s"],
                   100.0 * durations["rtk_jump_plateau"] / max(durations["span_s"], 1e-9), durations["ate_exclude"]))
    return trusted


def write_tum(path, a):
    with open(path, "w") as f:
        for r in a:
            f.write("%.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n" % tuple(r))


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    os.makedirs(args.out_dir, exist_ok=True)
    seq = args.seq or os.path.splitext(os.path.basename(args.gt_file))[0]
    checks = report.Checks()
    rep = {"tool": "gt_audit.py", "tool_version": TOOL_VERSION, "sequence": seq, "gt_file": os.path.abspath(args.gt_file),
           "bag": os.path.abspath(args.bag), "archive_dir": args.archive_label or os.path.abspath(args.out_dir),
           "generated_utc": datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")}
    raw = load_tum(args.gt_file)
    gt, n_reordered, n_dup = clean_gt(raw)
    d = np.diff(raw[:, 0])
    rep["format"] = {"rows": int(raw.shape[0]), "rows_clean": int(gt.shape[0]), "rows_out_of_order": n_reordered,
                     "rows_duplicate_stamp_dropped": n_dup, "backward_steps": int(np.sum(d < 0)),
                     "quat_identity_frac": float(np.mean(np.all(np.abs(raw[:, 4:8] - [0, 0, 0, 1]) < 1e-9, axis=1)))}
    checks.add("P0", "GT file format and cleaning", "INFO",
               "%d rows; %d backward stamp steps; %d rows reordered and %d duplicate-stamp rows dropped in gt_clean.tum; orientation identity on %.1f%% rows (position-only GT)" % (
                   raw.shape[0], rep["format"]["backward_steps"], n_reordered, n_dup, 100 * rep["format"]["quat_identity_frac"]))
    rtk, pvt, imu, odom = read_bag(args.bag, args)
    rb = rosio.RawBag(args.bag)
    bag_span = rb.start_end()
    rb.close()
    rep["inputs"] = {"rtk_msgs": len(rtk), "pvt_msgs": len(pvt), "imu_msgs": int(imu.shape[0]) if imu.size else 0, "odom_msgs": int(odom.shape[0]) if odom.size else 0,
                     "bag_record_span": list(bag_span), "sequence_class": seq_class(seq)}
    traj = None
    if args.replay_dir:
        if seq_class(seq) == "development":
            traj = load_state_trace(os.path.join(args.replay_dir, "state_trace.txt"))
            rep["inputs"]["replay_dir"] = os.path.abspath(args.replay_dir)
        else:
            rep["inputs"]["replay_dir"] = "ignored (A2: not a development sequence)"
    origin = provenance(gt, rtk, rep, checks)
    comp, lag_gt_minus_utc = time_base(gt, imu, odom, pvt, rep, checks, not args.no_plots, args.out_dir, rtk=rtk, traj=traj)
    p4 = [c["verdict"] for c in checks.items if c["id"] == "P4"]
    tb = decide_timebase(rep["time_base"], p4[0] if p4 else None)
    rep["timebase"] = tb
    datum_and_alignment(gt, rtk, pvt, origin, rep, checks, lag_gt_minus_utc)
    trusted = trust_mask(gt, pvt, lag_gt_minus_utc, rep, checks, args.out_dir, bag_span=bag_span, audited_shift_s=tb["applied_shift_s"])
    if args.assume_gcj02:
        rep["gcj02_correction"] = "requested but not applicable: the GT file is a local XYZ frame; correct the lat/lon source instead"
    write_tum(os.path.join(args.out_dir, "gt_clean.tum"), gt)
    # AMENDMENTS A54: the time base always has a value by rule (chain / record time / 0); gt_audited.tum = gt_clean.tum
    # + the applied shift in every case, gt_timebase.yaml says which rule gave it (the P4 verdict is unchanged)
    out = gt.copy()
    out[:, 0] += tb["applied_shift_s"]
    write_tum(os.path.join(args.out_dir, "gt_audited.tum"), out)
    write_timebase_yaml(os.path.join(args.out_dir, "gt_timebase.yaml"), seq, tb)
    tm = rep["trust_mask"]
    rep["outputs"] = {"gt_clean": "gt_clean.tum", "gt_audited": "gt_audited.tum", "time_shift_applied_s": tb["applied_shift_s"],
                      "timebase_method": tb["method"], "timebase_confidence": tb["confidence"], "gt_timebase": "gt_timebase.yaml",
                      "trust_mask": "trust_mask.csv", "mask_intervals": "gt_mask_intervals.csv", "trusted_frac": float(np.mean(trusted)),
                      "ate_exclude_s": tm["masked_s"]["ate_exclude"], "rtk_jump_plateau_s": tm["masked_s"]["rtk_jump_plateau"],
                      "span_in_bag_s": tm["masked_s"]["span_s"]}
    checks.add("P4r", "GT time base used by the evaluation (AMENDMENTS A54 rule)", "INFO" if tb["method"] == "chain" else "WARN",
               "method %s: %+.1f ms (applied shift %+.1f ms, > 20 ms rule), confidence %s, uncertainty %s (%s)%s" % (
                   tb["method"], 1e3 * tb["offset_s"], 1e3 * tb["applied_shift_s"], tb["confidence"],
                   "%.1f ms" % (1e3 * tb["uncertainty_s"]) if tb["uncertainty_s"] is not None and np.isfinite(tb["uncertainty_s"]) else "—",
                   tb["uncertainty_basis"], ("; " + tb["flag"]) if tb["flag"] else ""))
    checks.add("P6", "2HP antenna lever arm", "INFO",
               "unknown (the C-RTK2HP antenna is not in calibration.md): an ATE floor of the order of the antenna separation on the top plate (~0.1-0.3 m, README Fig. 1); TEST_PLAN E1' estimates it jointly")
    rep["checks"] = checks.items
    rep["verdict"] = checks.worst()
    report.write_json(os.path.join(args.out_dir, "gt_audit.json"), rep)
    write_md(os.path.join(args.out_dir, "gt_audit.md"), seq, rep, checks)
    print("gt_audit %s: %s %s -> %s" % (seq, checks.worst(), checks.counts(), args.out_dir))
    return 0


def write_md(path, seq, rep, checks):
    L = ["# GT 审计 — %s" % seq, "",
         "生成:`tools/gt_audit.py`(tool_version %d),%s。输入:GT 文件与原始 bag 的 RTK 定位话题、9Ps 接收机 PVT、Avia IMU、轮速计%s。" % (
             TOOL_VERSION, rep["generated_utc"], "".join(x for x in (
                 (";诊断 P4c 另读 GNSS-free 回放 `%s`(仅开发集,A2)" % rep["inputs"]["replay_dir"]) if str(rep.get("inputs", {}).get("replay_dir", "")).startswith("/") else "")) or
             "(A2:不读任何估计轨迹)"), "",
         "总评:**%s**(%s)。远端归档:`%s`。" % (checks.worst(), ", ".join("%s %d" % kv for kv in sorted(checks.counts().items())), rep["archive_dir"]), "",
         "## 检查表", "", checks.markdown(), ""]
    pv = rep.get("provenance")
    if pv and pv.get("hypotheses"):
        L += ["## GT 生成方式假设(对 RTK 定位逐点复算)", "",
              report.table(("hypothesis", "direct RMS [m]", "direct max [m]", "Sim3 scale", "Sim3 yaw [°]", "Sim3 RMS [m]"),
                           [(h["hypothesis"], h["rms_direct_m"], h["max_direct_m"], h["sim3_scale"], h["sim3_yaw_deg"], h["sim3_rms_m"]) for h in pv["hypotheses"]]),
              "", "GCJ-02 在本地的绝对平移 %.0f m,轨迹范围内局部畸变 %.3f m。" % (pv.get("gcj02_absolute_shift_here_m", float("nan")), pv.get("gcj02_local_distortion_m", float("nan"))), ""]
    tb = rep.get("time_base", {})
    if tb.get("xcorr"):
        L += ["## 时基互相关(GT 戳 t 对应其它时钟 t + lag)", "",
              report.table(("reference", "lag [ms]", "peak corr", "peak width [ms]", "delay precision [ms]"),
                           [(k, 1e3 * v["lag_s"], v["peak"], 1e3 * v["width_s"], 1e3 * v.get("sigma_s", float("nan"))) for k, v in tb["xcorr"].items()]), ""]
    lv = tb.get("lever_lead_fit")
    if lv:
        L += ["## 天线超前模型(P4 诊断,不经轮速计)", "",
              "航向角速率 = ω(t+L) + r_x·d/dt(ω/v):L = %+.1f ms,r_x = %+.2f m(车速中位 %.2f m/s,n = %d);r_x 置 0 时 L = %+.1f ms。"
              "天线在无侧滑点前方 r_x 处时 GT 航向**领先**车头 r_x/v,原始直接链路相对链式结果为正即此效应,不是钟差。" % (
                  1e3 * lv["lag_s"], lv["r_x_m"], lv["speed_median_mps"], lv["n"], 1e3 * lv["lag_no_lever_s"]), ""]
    t_b = rep.get("timebase")
    if t_b:
        ms = lambda v: "—" if v is None or (isinstance(v, float) and not np.isfinite(v)) else "%+.1f" % (1e3 * v)
        L += ["## 评测用 GT 时基(AMENDMENTS A54,`gt_timebase.yaml`)", "",
              "规则:P4 通过 ⇒ 轮速计链;P4 失败 ⇒ 记录时间旁证;两者都没有 ⇒ 0 并标记;补偿按 TEST_PLAN §4(|值| > 20 ms 才补偿)。",
              "",
              report.table(("method", "value [ms]", "applied shift [ms]", "confidence", "uncertainty [ms]", "P4", "chain [ms]",
                            "record time [ms]", "sub-threshold chain [ms]", "P4c replay (dev only, not used) [ms]"),
                           [(t_b["method"], ms(t_b["offset_s"]), ms(t_b["applied_shift_s"]), t_b["confidence"],
                             "—" if t_b["uncertainty_s"] is None or not np.isfinite(t_b["uncertainty_s"]) else "%.1f" % (1e3 * t_b["uncertainty_s"]),
                             t_b["p4_verdict"] or "—", ms(t_b["chain_s"]), ms(t_b["record_time_s"]), ms(t_b["sub_threshold_chain_s"]),
                             ms(t_b["replay_route_p4c_s"]))]), ""]
    tm = rep.get("trust_mask", {})
    rk = tm.get("rtk")
    if rk is not None:
        ms_ = tm["masked_s"]
        L += ["## RTK 跳变/毛刺与被掩时长(AMENDMENTS A54)", "",
              "逐定位差分对局部速度(前后各 %d 个定位区间的中位速度)的残差;跳变 = 水平 > %.2f m 或垂直 > %.2f m(max(0.5 m, 10σ),"
              "σ 水平 %.3f / 垂直 %.3f m);一对相互抵消的跳变之间为错位平台,连同前后 2 s 掩掉(bit 64)。" % (
                  JUMP_NEIGHBOURS, rk["thr_h_m"], rk["thr_v_m"], rk["sigma_h_m"], rk["sigma_v_m"]), ""]
        if rk["jumps"]:
            L += [report.table(("t − GT 起点 [s]", "ΔE [m]", "ΔN [m]", "ΔU [m]", "‖Δ‖ [m]"),
                               [(j["t_rel"], j["d_enu_m"][0], j["d_enu_m"][1], j["d_enu_m"][2], j["size_m"]) for j in rk["jumps"]]), ""]
        else:
            L += ["无跳变。", ""]
        L += [report.table(("criterion", "masked [s]", "of bag span [%]"),
                           [(k, v, 100.0 * v / max(ms_["span_s"], 1e-9)) for k, v in ms_.items() if k != "span_s"]),
              "", "bag 区间内 GT 时长 %.1f s;`ate_exclude` = 间断/冻结 + RTK 跳变平台(bit 8|64;bag 区间外 bit 32 本就不参评),"
              "区间见 `gt_mask_intervals.csv`(GT 戳与审计后时基两列)。h_acc/num_sv/速度尖峰(bit 1/2/4)只报告,不标 ATE 排除(未裁定)。" % ms_["span_s"], ""]
    o = rep["outputs"]
    L += ["## 产物", "", "`gt_clean.tum`、`gt_audited.tum`(= gt_clean + %.3f s,方法 %s)、`gt_timebase.yaml`、`trust_mask.csv`、"
          "`gt_mask_intervals.csv`、`gt_audit.json`。" % (o["time_shift_applied_s"], o["timebase_method"]), ""]
    with open(path, "w") as f:
        f.write("\n".join(L))


if __name__ == "__main__":
    sys.exit(main())
