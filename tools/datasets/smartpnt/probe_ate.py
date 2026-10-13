#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Score of one diagnostic replay (tools/datasets/probe_replay.sh) of a development sequence against the ground truth
(A2: development sequences only): the SE(3) ATE of the IMU trajectory moved to the ground-truth antenna by the calibrated lever
(tools/eval/lidars.json gt_lever_imu; the same association, span and Umeyama as tools/eval/eval_ate.py's e1c), the plain ATE of
the IMU point (e1o's form), and the travel direction in the body frame against the ground truth's (the yaw of the LiDAR -> IMU
extrinsic: a wrong yaw puts the direction of travel along another IMU axis).

  probe_ate.py --run <run dir> --seq <seq> [--root /data2/fl2g] [--lever X Y Z] [--span T0 T1] [--out report.json]

numpy only (the image's scipy is broken against its numpy).
"""
import argparse
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(TOOLS, "eval"))

import numpy as np  # noqa: E402

import e1traj  # noqa: E402
import eval_ate as ea  # noqa: E402
import evalcommon as ec  # noqa: E402
import gtbase  # noqa: E402


def travel_direction(gt, gti, est, ir, ie, min_speed=0.4):
    """Mean unit direction of travel in the body frame, from consecutive associated pairs ~1 s apart: the ground truth's from the
    I300 attitude of gt_i300.tum and the antenna positions, the estimate's from its own attitude and positions."""
    y, p = gt[ir, 1:4], est[ie, 1:4]
    tg = gt[ir, 0]
    rg = e1traj.quat_to_rot(gti[np.searchsorted(gti[:, 0], tg - 1e-3).clip(0, len(gti) - 1), 4:8])
    re = e1traj.quat_to_rot(est[ie, 4:8])
    dg, de, bg, be = [], [], [], []
    for i in range(len(ir) - 1):
        dt = tg[i + 1] - tg[i]
        if not (0.8 < dt < 1.2):
            continue
        d1, d2 = y[i + 1] - y[i], p[i + 1] - p[i]
        if np.hypot(d1[0], d1[1]) / dt < min_speed or np.linalg.norm(d2) / dt < min_speed:
            continue
        bg.append(rg[i].T @ (d1 / np.linalg.norm(d1)))
        be.append(re[i].T @ (d2 / np.linalg.norm(d2)))
    if len(bg) < 5:
        return None
    mg, me = np.mean(bg, axis=0), np.mean(be, axis=0)
    ng, ne = np.linalg.norm(mg), np.linalg.norm(me)
    ug, ue = mg / ng, me / ne
    ang = math.degrees(math.acos(max(-1.0, min(1.0, float(ug @ ue)))))
    return {"n_pairs": len(bg), "gt_body_dir": [round(float(v), 4) for v in ug], "est_body_dir": [round(float(v), 4) for v in ue],
            "angle_deg": ang, "concentration_gt": float(ng), "concentration_est": float(ne)}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--run", required=True)
    ap.add_argument("--seq", required=True)
    ap.add_argument("--root", default=None)
    ap.add_argument("--lever", type=float, nargs=3, default=None, help="antenna in the IMU frame (default: lidars.json of the sequence's profile)")
    ap.add_argument("--span", type=float, nargs=2, default=None, help="UTC Unix span; default the run's own time span")
    ap.add_argument("--traj", default="traj_W.tum")
    ap.add_argument("--min-speed", type=float, default=0.4, help="travel-direction pairs need a horizontal speed above this [m/s]")
    ap.add_argument("--out", default="")
    a = ap.parse_args(argv)
    root = ec.project_root(a.root)
    seqs = ec.load_sequences()[1]
    if a.seq not in seqs:
        raise SystemExit("%s is not in tools/eval/sequences.json" % a.seq)
    if ec.seq_class(a.seq) != "development":
        raise SystemExit("A2: %s is not a development sequence" % a.seq)
    lever = a.lever
    if lever is None:
        prof = seqs[a.seq].get("lidar") or "smartpnt_mini"
        lever = ec.lidar_profile(prof)["gt_lever_imu"]
    gt, prov, mask = gtbase.load_gt(root, a.seq, seqs[a.seq])
    run = a.run if os.path.isabs(a.run) else os.path.join(root, a.run)
    est = ec.read_tum(os.path.join(run, a.traj))
    est, _, _ = ec.sanitize_tum(est)
    t0, t1 = a.span or (float(est[0, 0]), float(est[-1, 0]))
    g = gt[(gt[:, 0] >= t0) & (gt[:, 0] <= t1)]
    out = {"run": run, "sequence": a.seq, "lever_m": list(lever), "span_utc": [t0, t1], "est_rows": int(est.shape[0]),
           "est_path_m": float(np.sum(np.linalg.norm(np.diff(est[:, 1:4], axis=0), axis=1)))}
    # masked stretches of the ground truth (A54) are removed on both sides as eval_ate does
    g, e2 = g[~mask.inside(g[:, 0])], est[~mask.inside(est[:, 0])]
    out["masked_stretches"] = int(mask.iv.shape[0])
    out["gt_rows_used"] = int(g.shape[0])
    plain = ea.ate_numpy(g, e2, t0, t1)
    out["ate_imu_point_rmse_m"] = plain["rmse"]
    out["pairs"] = plain["n_pairs"]
    ir, ie = ea.evo_associate(g[:, 0], e2[:, 0], ec.T_MAX_DIFF_S)
    y, p = g[ir, 1:4], e2[ie, 1:4]
    rb = e1traj.quat_to_rot(e2[ie, 4:8])
    x = p + rb @ np.asarray(lever, dtype=float)
    r, t = ea.umeyama_se3(x, y)
    err = np.linalg.norm(x @ r.T + t - y, axis=1)
    out["ate_lever_rmse_m"] = float(np.sqrt(np.mean(err ** 2)))
    out["ate_lever_median_m"] = float(np.median(err))
    out["ate_lever_max_m"] = float(np.max(err))
    out["gt_path_m"] = float(np.sum(np.linalg.norm(np.diff(y, axis=0), axis=1)))
    # travel direction needs the GT attitude of the I300 frame
    gi = os.path.join(root, "audit", a.seq, "gt", "gt_i300.tum")
    if os.path.isfile(gi):
        gti = ec.read_tum(gi)
        gti[:, 0] += prov.get("time_shift_applied_s", 0.0)
        out["travel_direction"] = travel_direction(g, gti, e2, ir, ie, a.min_speed)
    txt = json.dumps(out, indent=1)
    if a.out:
        with open(a.out, "w") as f:
            f.write(txt + "\n")
    print(txt)
    return 0


if __name__ == "__main__":
    sys.exit(main())
