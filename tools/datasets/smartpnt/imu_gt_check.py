#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""The IMU stream of a slim bag against the ground-truth attitude (run in the project image; rosbags). No estimator and no GNSS
solution is involved, so every class of sequence may run it (AMENDMENTS A2): it checks that the ground truth belongs to the
recording, that the IMU scale is right and how well the two clocks agree.

  imu_gt_check.py <seq> [--root /data2/fl2g] [--gt-dir D] [--bag B] [--out report.json]

  heading     the gyro projected on the ground-truth 'up' axis and integrated, against the ground-truth heading change over 10 s
              intervals: the slope on the turning intervals (IMU scale, ~1), the residual, the sums over the sequence;
  lag scan    the same on 1 s intervals with the ground-truth epochs shifted by tau: the minimum of the residual sum of squares,
              parabolic peak. tau > 0: the epoch labelled t in the ground truth belongs to IMU time t + tau;
  static      the longest quiet stretch in the first 200 s of the IMU stream: |accelerometer| against the local gravity
              (Somigliana + free air at the ground-truth origin) and the gyro mean (bias).
"""
import argparse
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, os.path.join(TOOLS, "eval"))

UP = np.array([0.0, 0.0, 1.0])
FORWARD_I300 = np.array([0.0, 0.0, -1.0])      # I300 axes: x down, y right, z back


def gravity(lat, h):
    """Local gravity [m/s^2]: Somigliana + free air; lat [rad], h [m]."""
    s = math.sin(lat) ** 2
    s2 = math.sin(2 * lat) ** 2
    return 9.780327 * (1 + 0.0053024 * s - 0.0000058 * s2) - 3.086e-6 * h


def gyro_integral_on_up(t, g, gt_t, R_gt):
    """Cumulative integral of the gyro rate about the ground-truth up axis (CCW positive), sampled on the IMU stamps `t`;
    the up axis is the one of the nearest ground-truth epoch. R_gt: (n,3,3) body -> ENU."""
    idx = np.clip(np.searchsorted(gt_t, t), 1, len(gt_t) - 1)
    up_b = np.einsum("nji,j->ni", R_gt[idx], UP)
    wup = np.einsum("ni,ni->n", g, up_b)
    return np.concatenate([[0.0], np.cumsum(0.5 * (wup[1:] + wup[:-1]) * np.diff(t))])


def heading_check(t, g, gt_t, R_gt, forward=FORWARD_I300):
    """-> dict with the 10 s heading comparison and the 1 s lag scan (see the module docstring)."""
    fwd = R_gt @ forward
    hd = np.unwrap(np.arctan2(fwd[:, 0], fwd[:, 1]))                   # heading, clockwise from north
    cum = gyro_integral_on_up(t, g, gt_t, R_gt)

    def intervals(step, tau=0.0):
        ci = np.interp(gt_t + tau, t, cum)
        i0 = np.arange(0, len(gt_t) - step, step)
        return -(hd[i0 + step] - hd[i0]), ci[i0 + step] - ci[i0]

    dh, dc = intervals(10)
    big = np.abs(dh) > np.radians(5)
    res = dc - dh
    out = {"heading_10s": {
        "intervals": int(len(dh)), "turning_intervals_over_5deg": int(big.sum()),
        "slope_turning": float(np.sum(dh[big] * dc[big]) / np.sum(dc[big] ** 2)) if big.any() else None,
        "residual_median_deg": float(np.degrees(np.median(res))), "residual_mean_deg": float(np.degrees(res.mean())),
        "residual_std_deg": float(np.degrees(res.std())),
        "sum_gyro_deg": float(np.degrees(dc.sum())), "sum_gt_deg": float(np.degrees(dh.sum())),
        "abs_sum_gyro_deg": float(np.degrees(np.abs(dc).sum())), "abs_sum_gt_deg": float(np.degrees(np.abs(dh).sum()))}}
    taus = np.arange(-1.5, 1.5001, 0.02)
    dh1, _ = intervals(1)
    turn = np.abs(dh1) > np.radians(1.5)
    if turn.sum() < 30:
        out["lag_scan_1s"] = {"turning_intervals_over_1.5deg_per_s": int(turn.sum()), "note": "too few turning intervals"}
        return out
    rss = np.array([float(np.sum((intervals(1, tau)[1][turn] - dh1[turn]) ** 2)) for tau in taus])
    k = int(np.argmin(rss))
    tau_star = float(taus[k])
    if 0 < k < len(taus) - 1:
        y0, y1, y2 = rss[k - 1], rss[k], rss[k + 1]
        den = y0 - 2 * y1 + y2
        if den > 0:
            tau_star = float(taus[k] + 0.5 * (y0 - y2) / den * (taus[1] - taus[0]))
    i0 = int(np.argmin(np.abs(taus)))
    out["lag_scan_1s"] = {"turning_intervals_over_1.5deg_per_s": int(turn.sum()), "tau_star_s": tau_star, "rss_min": float(rss.min()),
                          "rss_at_0": float(rss[i0]), "rss_ratio_0_over_min": float(rss[i0] / rss.min()),
                          "rss_at_tau_minus1s_plus1s": [float(rss[np.argmin(np.abs(taus + 1.0))]), float(rss[np.argmin(np.abs(taus - 1.0))])]}
    return out


def static_window(t, g, a, lat, h, search_s=200, min_len_s=10, acc_std_max=0.05, gyro_max=0.02):
    """The longest run of quiet 1 s blocks within `search_s` of the first sample, and the readings in it."""
    nb = int(t[-1] - t[0])
    blk = (t - t[0]).astype(int)
    first = min(search_s, nb)
    ok = np.zeros(first, bool)
    for i in range(first):
        m = blk == i
        if m.sum() < 20:
            continue
        gm = g[m].mean(axis=0)
        ok[i] = (np.linalg.norm(a[m], axis=1).std() < acc_std_max and np.linalg.norm(gm) < gyro_max
                 and np.linalg.norm(g[m] - gm, axis=1).max() < gyro_max)
    best, cur = (0, 0), None
    for i in range(first + 1):
        if i < first and ok[i]:
            cur = i if cur is None else cur
        else:
            if cur is not None and i - cur > best[1] - best[0]:
                best = (cur, i)
            cur = None
    w0, w1 = best
    if w1 - w0 < min_len_s:
        return {"note": "no quiet stretch of >= %d s in the first %d s" % (min_len_s, search_s), "longest_s": int(w1 - w0)}
    m = (blk >= w0) & (blk < w1)
    an = np.linalg.norm(a[m], axis=1)
    gl = gravity(lat, h)
    return {"window_s_from_imu_start": [w0, w1], "acc_norm_median": float(np.median(an)), "acc_norm_std": float(an.std()),
            "local_gravity": float(gl), "ratio": float(np.median(an) / gl),
            "gyro_mean_rad_s": [float(x) for x in g[m].mean(axis=0)], "acc_mean_xyz": [float(x) for x in a[m].mean(axis=0)]}


def read_imu(bag, topic="/imu_raw"):
    from dscommon import rosbytes
    from rosbags.rosbag1 import Reader
    t, g, a = [], [], []
    with Reader(bag) as r:
        conns = [c for c in r.connections if c.topic == topic]
        for c, ts, raw in r.messages(connections=conns):
            d = rosbytes.parse_imu(raw)
            t.append(d["stamp_ns"] / 1e9)
            g.append(d["gyro"])
            a.append(d["acc"])
    return np.array(t), np.array(g), np.array(a)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("seq")
    p.add_argument("--root", default="/data2/fl2g")
    p.add_argument("--gt-dir", default=None)
    p.add_argument("--bag", default=None)
    p.add_argument("--out", default=None)
    a = p.parse_args(argv)
    import e1traj
    import evalcommon as ec
    gt_dir = a.gt_dir or os.path.join(a.root, "smartpnt", a.seq, "gt")
    bag = a.bag or os.path.join(a.root, "smartpnt", a.seq, a.seq + ".bag")
    gti = ec.read_tum(os.path.join(gt_dir, "gt_i300.tum"))
    with open(os.path.join(gt_dir, "gt_audit.json")) as f:
        lat, lon, h0 = json.load(f)["enu_origin_lat_lon_h_rad_rad_m"]
    t, g, acc = read_imu(bag)
    res = {"sequence": a.seq, "imu_samples": int(len(t))}
    res.update(heading_check(t, g, gti[:, 0], e1traj.quat_to_rot(gti[:, 4:8])))
    res["static_start"] = static_window(t, g, acc, lat, h0)
    txt = json.dumps(res, indent=1)
    if a.out:
        with open(a.out, "w") as f:
            f.write(txt + "\n")
    print(txt)
    return 0


if __name__ == "__main__":
    sys.exit(main())
