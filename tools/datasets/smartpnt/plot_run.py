#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Trajectory and error-over-time figure of one replay of a development sequence (A2: development sequences only), with the
numbers that go with it: the SE(3) Umeyama-aligned antenna track of the estimate (IMU trajectory + calibrated lever, the e1c form
of tools/eval/eval_ate.py) against the ground-truth antenna, the error at every associated epoch, the masked ground-truth stretches
(A54) in grey.

  plot_run.py --run <run dir> --seq <seq> --out <png> [--root /data2/fl2g] [--lever X Y Z] [--traj traj_W.tum] [--json out.json]

The alignment is fitted on the associated, unmasked epochs only (as the ATE); the whole estimated track is drawn with it.
Also reports the path lengths, the ATE per path length, and the largest error with its time. numpy + matplotlib.
"""
import argparse
import json
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


def main(argv=None):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--run", required=True)
    ap.add_argument("--seq", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--root", default=None)
    ap.add_argument("--lever", type=float, nargs=3, default=None)
    ap.add_argument("--traj", default="traj_W.tum")
    ap.add_argument("--title", default="")
    ap.add_argument("--json", default="")
    a = ap.parse_args(argv)
    root = ec.project_root(a.root)
    seqs = ec.load_sequences()[1]
    if ec.seq_class(a.seq) != "development":
        raise SystemExit("A2: %s is not a development sequence" % a.seq)
    lever = a.lever if a.lever is not None else ec.lidar_profile(seqs[a.seq].get("lidar") or "smartpnt_mini")["gt_lever_imu"]
    gt, prov, mask = gtbase.load_gt(root, a.seq, seqs[a.seq])
    run = a.run if os.path.isabs(a.run) else os.path.join(root, a.run)
    est, _, _ = ec.sanitize_tum(ec.read_tum(os.path.join(run, a.traj)))
    t0, t1 = float(est[0, 0]), float(est[-1, 0])
    g = gt[(gt[:, 0] >= t0) & (gt[:, 0] <= t1)]
    g_keep = g[~mask.inside(g[:, 0])]
    e_keep = est[~mask.inside(est[:, 0])]
    ir, ie = ea.evo_associate(g_keep[:, 0], e_keep[:, 0], ec.T_MAX_DIFF_S)
    y = g_keep[ir, 1:4]
    rb = e1traj.quat_to_rot(e_keep[ie, 4:8])
    x = e_keep[ie, 1:4] + rb @ np.asarray(lever, dtype=float)
    R, t = ea.umeyama_se3(x, y)
    err = np.linalg.norm(x @ R.T + t - y, axis=1)
    # whole estimated antenna track
    rb_all = e1traj.quat_to_rot(est[:, 4:8])
    ant = (est[:, 1:4] + rb_all @ np.asarray(lever, dtype=float)) @ R.T + t
    path_est = float(np.sum(np.linalg.norm(np.diff(est[:, 1:4], axis=0), axis=1)))
    path_gt = float(np.sum(np.linalg.norm(np.diff(g[:, 1:4], axis=0), axis=1)))
    res = {"run": run, "sequence": a.seq, "lever_m": list(lever), "span_utc": [t0, t1], "duration_s": t1 - t0,
           "pairs": int(len(ir)), "gt_rows_in_span": int(g.shape[0]), "gt_rows_masked": int(g.shape[0] - g_keep.shape[0]),
           "path_est_m": path_est, "path_gt_m": path_gt, "ate_rmse_m": float(np.sqrt(np.mean(err ** 2))),
           "ate_median_m": float(np.median(err)), "ate_max_m": float(np.max(err)), "t_of_max_s": float(g_keep[ir, 0][int(np.argmax(err))] - t0),
           "ate_over_path_percent": float(100.0 * np.sqrt(np.mean(err ** 2)) / max(path_gt, 1e-9))}
    tb = g_keep[ir, 0] - t0
    res["error_by_60s_bin_median_m"] = {"%d" % b: float(np.median(err[(tb >= b) & (tb < b + 60)])) for b in range(0, int(t1 - t0) + 1, 60)
                                        if np.any((tb >= b) & (tb < b + 60))}
    fig, ax = plt.subplots(1, 2, figsize=(13, 5.2), gridspec_kw={"width_ratios": [1, 1.25]})
    ax[0].plot(g[:, 1], g[:, 2], "-", color="0.15", lw=2.0, label="ground truth antenna (%d rows)" % g.shape[0])
    ax[0].plot(ant[:, 0], ant[:, 1], "-", color="tab:orange", lw=1.0, label="estimate (IMU + lever, aligned)")
    ax[0].set_aspect("equal", adjustable="datalim")
    ax[0].set_xlabel("E [m]")
    ax[0].set_ylabel("N [m]")
    ax[0].legend(fontsize=8, loc="best")
    ax[0].set_title(a.title or ("%s  (%s)" % (a.seq, os.path.basename(run))), fontsize=9)
    tt = g_keep[ir, 0] - t0
    for lo, hi in mask.iv:
        if hi >= t0 and lo <= t1:
            ax[1].axvspan(max(lo, t0) - t0, min(hi, t1) - t0, color="0.88", lw=0)
    ax[1].plot(tt, err, "-", color="tab:red", lw=1.0)
    ax[1].set_xlabel("time since the view start [s]")
    ax[1].set_ylabel("antenna position error after SE(3) alignment [m]")
    ax[1].set_title("RMSE %.2f m over %d epochs, path %.0f m (GT %.0f m); grey = masked GT" % (res["ate_rmse_m"], len(ir), path_est, path_gt), fontsize=9)
    ax[1].set_xlim(0, t1 - t0)
    fig.tight_layout()
    fig.savefig(a.out, dpi=110)
    if a.json:
        with open(a.json, "w") as f:
            json.dump(res, f, indent=1)
    print(json.dumps(res, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
