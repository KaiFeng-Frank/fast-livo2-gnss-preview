#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""GNSS content check of a slim bag (run in the project image; ROS1 python API).

  gnss_check.py stats <bag> --nav <brdm.24p> (--approx-xyz X Y Z | --gt-dir <seq>/gt) [--max-epochs N]
  gnss_check.py dev   <bag> --nav <brdm.24p> --gt-dir <seq>/gt [--approx-xyz X Y Z] [--max-epochs N]

stats  (every class of sequence; format level only, AMENDMENTS A2): epochs and rate, satellites per epoch by system and
       the A80 RAIM count (primary-signal C/N0 >= 35 dB-Hz and elevation >= 15 deg), C/N0 percentiles, carrier-valid and
       slip-flag fractions of the primary signal, the share of satellite-epochs with a healthy ephemeris from the bag alone
       (causal, the in-bag messages) and with the broadcast file; per-system satellites per epoch (median, p10, p90) and the
       availability of the RAIM count over time (fraction of 1 Hz epochs with at least 4/6/8/10, the stretches below 4,
       the spread of the 100 s window medians).
dev    (development sequences only; refused otherwise, A2: no GNSS-only solution of an evaluation sequence is compared
       with anything): 1 Hz single-point positioning against the ground-truth antenna (ENU -> ECEF with the origin of
       gt_audit.json), and the lag between the Doppler-LS horizontal speed and the ground-truth horizontal speed (both GPST,
       so the lag must be ~0: it verifies the epoch time tags and the GPST->UTC rule of the slim bag end to end).
"""
import argparse
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(os.path.dirname(HERE)))
sys.path.insert(0, os.path.dirname(HERE))

import numpy as np  # noqa: E402

from fl2g import geodesy, gnss_msgs, gnsslsq, gnsstime, obs as fobs, orbits, rinexnav, robust, rosio  # noqa: E402
from fl2g.sequences import seq_class  # noqa: E402

MAX_AGE = {"G": 7200.0, "E": 7200.0, "C": 3600.0, "R": 1800.0, "J": 7200.0}
LEAP_S = 18


def read_gnss(path, max_epochs=None):
    rb = rosio.RawBag(path)
    epochs, eph, geph, iono = [], [], [], None
    try:
        for topic, datatype, raw, t_rec, pytype in rb.iter_raw(topics=list(gnss_msgs.GNSS_TOPICS)):
            if topic == gnss_msgs.TOPIC_MEAS:
                if max_epochs is None or len(epochs) < max_epochs:
                    epochs.append(gnss_msgs.meas_epoch(rb.deserialize(pytype, raw), t_rec))
            elif topic == gnss_msgs.TOPIC_EPHEM:
                eph.append(gnss_msgs.kepler_eph(rb.deserialize(pytype, raw), t_rec))
            elif topic == gnss_msgs.TOPIC_GLO_EPHEM:
                geph.append(gnss_msgs.glo_eph(rb.deserialize(pytype, raw), t_rec))
            elif topic == gnss_msgs.TOPIC_IONO and iono is None:
                iono = gnss_msgs.iono(rb.deserialize(pytype, raw), t_rec)
    finally:
        rb.close()
    return epochs, eph, geph, iono


def nav_records(path):
    hdr, recs = rinexnav.read_nav(path)
    return hdr, recs


def availability(nraim, t, thresholds=(4, 6, 8, 10), low=4, window_s=100.0):
    """Availability of the RAIM-qualified satellites (C/N0 >= 35 dB-Hz, elevation >= 15 deg) over time, from the 1 Hz counts
    `nraim` stamped `t` [s]: the fraction of epochs with at least n of them, the stretches with fewer than `low` (a stretch
    lasts from its first to its last epoch plus one epoch interval), and the spread of the per-window medians. Format level
    only (a count of signals above a threshold, no solution of any kind)."""
    n = np.asarray(nraim, dtype=float)
    t = np.asarray(t, dtype=float)
    dt = float(np.median(np.diff(t))) if t.size > 1 else 1.0
    out = {"fraction_epochs_with_at_least": {str(k): float(np.mean(n >= k)) for k in thresholds}}
    bad = n < low
    runs, i = [], 0
    while i < n.size:
        if bad[i]:
            j = i
            while j + 1 < n.size and bad[j + 1] and (t[j + 1] - t[j]) <= 2.5 * dt:
                j += 1
            runs.append((float(t[i] - t[0]), float(t[j] - t[i] + dt)))
            i = j + 1
        else:
            i += 1
    runs.sort(key=lambda r: -r[1])
    out["stretches_below_%d" % low] = {"count": len(runs), "total_s": float(sum(r[1] for r in runs)),
                                       "longest_s": runs[0][1] if runs else 0.0,
                                       "longest_five_start_s_duration_s": [[round(s, 1), round(d, 1)] for s, d in runs[:5]]}
    w = np.floor((t - t[0]) / window_s).astype(int)
    med = np.array([np.median(n[w == k]) for k in np.unique(w) if np.sum(w == k) >= 0.5 * window_s / dt])
    out["window_%gs_median" % window_s] = {"windows": int(med.size), "p10": float(np.percentile(med, 10)) if med.size else None,
                                           "min": float(med.min()) if med.size else None,
                                           "fraction_windows_below_%d" % low: float(np.mean(med < low)) if med.size else None}
    return out


def stats(epochs, eph, geph, nav_recs, approx_xyz):
    pos = geodesy.ecef2pos(np.array(approx_xyz))
    sel_bag = orbits.EphSelector(eph + geph, MAX_AGE, causal=True)
    sel_all = orbits.EphSelector(eph + geph + nav_recs, MAX_AGE, causal=False)
    cn0, nsat, nraim, nsys, per_sys = [], [], [], {}, []
    cp_ok = cp_tot = lli_tot = cpflag = cpflag0 = 0
    healthy_bag = healthy_all = sat_epochs = 0
    el_cache = {}
    for k, e in enumerate(epochs):
        obs = fobs.ls_obs(e)
        nsat.append(len(obs))
        cnt = {}
        n35 = 0
        for o in obs:
            cn0.append(o["cn0"])
            nsys[o["sys"]] = nsys.get(o["sys"], 0) + 1
            cnt[o["sys"]] = cnt.get(o["sys"], 0) + 1
            cp_tot += 1
            cp_ok += 1 if (o["status"] & 2) and o["cp"] != 0.0 else 0
            if o["status"] & 2:
                cpflag += 1
                cpflag0 += 1 if o["cp"] == 0.0 else 0
            lli_tot += 1 if (o["lli"] & 3) else 0
            if k % 10 == 0:                                 # 1 Hz subsample for the ephemeris and elevation work
                sat_epochs += 1
                t = e["t_gpst"]
                s = sel_all.select(o["sat"], t)
                healthy_all += 1 if s is not None else 0
                healthy_bag += 1 if sel_bag.select(o["sat"], t, e["t_record"]) is not None else 0
                if s is not None and o["cn0"] >= 35.0:
                    try:
                        rs, _ = orbits.pos_clk(s, t - 0.075)
                        r, ev = geodesy.geodist(rs, np.array(approx_xyz))
                        _, el = geodesy.satazel(pos, ev)
                        if math.degrees(el) >= 15.0:
                            n35 += 1
                    except Exception:
                        pass
        per_sys.append(cnt)
        if k % 10 == 0:
            nraim.append(n35)
    t_all = np.array([e["t_gpst"] for e in epochs])
    dt = np.diff(t_all)
    out = {"epochs": len(epochs), "span_s": float(t_all[-1] - t_all[0]) if len(epochs) > 1 else 0.0,
           "epoch_dt_s": {"median": float(np.median(dt)), "p1": float(np.percentile(dt, 1)), "p99": float(np.percentile(dt, 99)),
                          "gaps_gt_1s": int(np.sum(dt > 1.0)), "max_gap_s": float(dt.max())} if dt.size else None,
           "primary_sats_per_epoch": robust.summary(nsat) if hasattr(robust, "summary") else {"mean": float(np.mean(nsat))},
           "primary_signals_by_system": nsys,
           "primary_sats_per_epoch_by_system": {s: {"median": float(np.median([c.get(s, 0) for c in per_sys])),
                                                    "p10": float(np.percentile([c.get(s, 0) for c in per_sys], 10)),
                                                    "p90": float(np.percentile([c.get(s, 0) for c in per_sys], 90))} for s in sorted(nsys)},
           "cn0_primary_dbhz": {"p10": float(np.percentile(cn0, 10)), "median": float(np.median(cn0)), "p90": float(np.percentile(cn0, 90))},
           "carrier_valid_fraction_primary": cp_ok / max(cp_tot, 1),
           "carrier_flag_set_but_zero_phase_fraction": cpflag0 / max(cpflag, 1), "lli_flagged_fraction_primary": lli_tot / max(cp_tot, 1),
           "raim_sats_c35_el15_per_epoch(1Hz)": {"mean": float(np.mean(nraim)), "median": float(np.median(nraim)),
                                                 "p10": float(np.percentile(nraim, 10)), "epochs_with_ge4": float(np.mean(np.array(nraim) >= 4))},
           "raim_availability(1Hz)": availability(nraim, t_all[::10]),
           "healthy_ephemeris_fraction_bag_only_causal": healthy_bag / max(sat_epochs, 1),
           "healthy_ephemeris_fraction_with_broadcast_file": healthy_all / max(sat_epochs, 1),
           "ephemeris_messages": {"kepler": len(eph), "glonass": len(geph)}}
    return out


def gt_ecef(gt_dir):
    tum = np.loadtxt(os.path.join(gt_dir, "gt_clean.tum"))
    with open(os.path.join(gt_dir, "gt_audit.json")) as f:
        org = json.load(f)["enu_origin_lat_lon_h_rad_rad_m"]
    r0 = geodesy.pos2ecef(*org)
    E = geodesy.enu_matrix(org[0], org[1])
    ecef = r0 + tum[:, 1:4] @ E
    return tum[:, 0], ecef, tum


def dev_checks(epochs, eph, geph, nav_recs, iono_hdr, gt_dir, approx_xyz):
    sel = orbits.EphSelector(eph + geph + nav_recs, MAX_AGE, causal=False)
    ion = None
    a, b = iono_hdr.get("iono_gps_alpha"), iono_hdr.get("iono_gps_beta")
    if a and b:
        ion = [float(x or 0.0) for x in list(a) + list(b)]
    tg, ecef, tum = gt_ecef(gt_dir)
    errs, speeds_g, t_speed = [], [], []
    last = np.array(approx_xyz)
    for e in epochs:
        t_utc = e["t_gpst"] - LEAP_S
        if t_utc < tg[0] - 1 or t_utc > tg[-1] + 1:
            continue
        obs = fobs.ls_obs(e)
        sp = gnsslsq.spp(obs, e["t_gpst"], sel, iono=ion, x0=last, cn0_min=20.0)
        if not sp or sp["rms_m"] > 30.0:
            continue
        last = sp["rr"]
        i = np.searchsorted(tg, t_utc)
        if 0 < i < len(tg):
            f = (t_utc - tg[i - 1]) / (tg[i] - tg[i - 1])
            gx = ecef[i - 1] * (1 - f) + ecef[i] * f
            d = sp["rr"] - gx
            lat, lon, _ = geodesy.ecef2pos(gx)
            enu = geodesy.enu_matrix(lat, lon) @ d
            errs.append((t_utc, enu[0], enu[1], enu[2]))
        dv = gnsslsq.doppler_ls(obs, e["t_gpst"], sel, sp["rr"], cn0_min=25.0)
        if dv:
            v = dv["v_enu"]
            speeds_g.append(math.hypot(v[0], v[1]))
            t_speed.append(t_utc)
    out = {"spp_epochs": len(errs)}
    if errs:
        a = np.array(errs)
        h = np.hypot(a[:, 1], a[:, 2])
        out["spp_error_vs_gt_antenna_m"] = {"horizontal_median": float(np.median(h)), "horizontal_p95": float(np.percentile(h, 95)),
                                            "up_median_abs": float(np.median(np.abs(a[:, 3]))), "bias_enu_median": [float(np.median(a[:, k])) for k in (1, 2, 3)]}
    # lag between the Doppler-LS horizontal speed and the GT horizontal speed (GT velocity from the position differences)
    vgt = np.hypot(np.gradient(tum[:, 1], tum[:, 0]), np.gradient(tum[:, 2], tum[:, 0]))
    if len(t_speed) > 200:
        ts = np.array(t_speed)
        sg = np.array(speeds_g)
        grid = np.arange(max(ts[0], tg[0]) + 1, min(ts[-1], tg[-1]) - 1, 0.1)
        s_dop = np.interp(grid, ts, sg)
        k = int(round(0.5 / 0.1))
        s_dop = np.convolve(s_dop, np.ones(2 * k + 1) / (2 * k + 1), mode="same")
        best = None
        curve = []
        for lag in np.arange(-3.0, 3.0001, 0.1):
            s_gt = np.interp(grid + lag, tg, vgt)
            m = slice(40, -40)
            c = float(np.corrcoef(s_dop[m], s_gt[m])[0, 1])
            curve.append((round(float(lag), 2), c))
            if best is None or c > best[1]:
                best = (float(lag), c)
        out["doppler_speed_vs_gt_speed"] = {"best_lag_s": best[0], "corr_at_best": best[1],
                                            "corr_at_zero": dict(curve).get(0.0), "note": "gt(t + lag) matches doppler(t); GT 1 Hz, so the lag is good to ~0.1 s"}
    return out


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("cmd", choices=["stats", "dev"])
    p.add_argument("bag")
    p.add_argument("--nav", required=True)
    p.add_argument("--approx-xyz", type=float, nargs=3, default=None, help="ECEF [m]; default: the ENU origin of --gt-dir/gt_audit.json (a location only)")
    p.add_argument("--gt-dir", default=None)
    p.add_argument("--seq", default="", help="sequence name for the A2 class check (default: the bag's file name)")
    p.add_argument("--max-epochs", type=int, default=None)
    p.add_argument("--out", default=None)
    a = p.parse_args(argv)
    seq = a.seq or os.path.basename(a.bag).replace(".bag", "")
    if a.approx_xyz is None:
        if not a.gt_dir:
            raise SystemExit("give --approx-xyz or --gt-dir (its ENU origin is used as the approximate position)")
        with open(os.path.join(a.gt_dir, "gt_audit.json")) as f:
            a.approx_xyz = [float(x) for x in geodesy.pos2ecef(*json.load(f)["enu_origin_lat_lon_h_rad_rad_m"])]
    epochs, eph, geph, iono = read_gnss(a.bag, a.max_epochs)
    hdr, recs = nav_records(a.nav)
    res = {"sequence": seq, "class": seq_class(seq)}
    res["stats"] = stats(epochs, eph, geph, recs, a.approx_xyz)
    if a.cmd == "dev":
        if seq_class(seq) != "development":
            raise SystemExit("A2: %s is not a development sequence; `dev` computes GNSS-only solutions" % seq)
        res["dev"] = dev_checks(epochs, eph, geph, recs, hdr, a.gt_dir, a.approx_xyz)
    txt = json.dumps(res, indent=1, default=str)
    if a.out:
        with open(a.out, "w") as f:
            f.write(txt + "\n")
    print(txt)
    return 0


if __name__ == "__main__":
    sys.exit(main())
