#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Ground-truth antenna lever arm of a host IMU, and the lever arms derived from it (AMENDMENTS A77). numpy only.

  host_gt_lever.py fit    --root <project root> --level-from-imu <9 numbers> --vz <m> [--run SEQ=<run dir> ...]
                          [--est SEQ=<est_e1.tum>:<gt_eval.tum> ...] --out <json>
  host_gt_lever.py derive --calibration <calibration.md> --lever-ref X Y Z --vxy VX VY [--out <json>]

fit
  A58's lever fit (tools/calib/p4_calib.py `frame`, fit_lever): the C-RTK 2HP antenna's lever arm l in the host IMU frame,
  fitted jointly with the global SE(3) of the host's own trajectory against the audited GT (g ~ Rg (p + R l) + tg), the two
  horizontal components free, the vertical one held. A58 did this in the Avia IMU frame, whose z axis is vertical to 2-3
  degrees, so "the first two components" were the horizontal ones. The MID-360 IMU frame is tilted 31.8 degrees against the
  vertical, so the split is made in the LEVEL frame instead (lidars.json `R_level_from_imu` takes IMU-frame free vectors to
  the Avia IMU axes): l = R_imu_from_level [vx, vy, vz], (vx, vy) free, vz held. With R_imu_from_level = I this is
  p4_calib.fit_lever step for step (test_host_gt_lever.py asserts it). Of the three level components only (vx, vy) are
  observable for a ground vehicle (the vertical one is a constant offset the alignment absorbs; the held value fixes
  where on that null line the IMU-frame vector sits), so the fitted (vx, vy) do not depend on vz beyond a tilt-sized
  second-order term (the report gives the sensitivity). Inputs per run: either a run directory (traj_W.tum, read as
  `frame` reads it: tools/eval/gtbase GT with time base and ATE mask, sanitised, GT gap <= 0.5 s) or eval_ate's own
  est_e1.tum + gt_eval.tum (the masked trajectory and GT eval_ate scored, for runs whose directories are gone).
  A58 took the first run of every sequence and the median over the sequences; this tool reports that, the median over
  the seeds of every sequence, and the median over all runs.

derive
  A77 decisions 2-3: with the fitted level components (vx, vy) of the GT antenna seen from the MID-360 IMU,
    * GT antenna in the MID-360 IMU frame        l_gt  = R_am [vx, vy, vz_chain]        (R_am = calibration row avia -> mid360,
                                                  the same rotation as lidars.json's R_level_from_imu transposed; vz_chain =
                                                  the chain's level-frame height of the antenna over the MID-360 IMU =
                                                  lever_ref z - the MID-360 IMU's height over the Avia IMU),
    * 9Ps antenna in the MID-360 IMU frame       l_9ps = l_gt + R_am d,   d = (9Ps - GT antenna) in the Avia IMU axes
                                                  (A15's 9Ps lever minus the A58 GT lever, vertical difference zero: A58 holds
                                                  the GT vertical at the 9Ps height); only the chain's ROTATION is used, never
                                                  its translation (which the data contradict, A76 record, section six),
    * 9Ps antenna in the MID-360 LiDAR frame     l_L = R_IL^T (l_9ps - t_IL)  (config gnss.frames.lever_arm_frame = lidar;
                                                  extrinsic_R = I, t_IL = the row mid360 -> mid360_imu).
  Self-checks: with the chain's own (vx, vy) the construction returns the chain values the repository had before A77
  (lidars.json and config lever to 1e-9), distances are preserved, the rotation is a rotation.
"""
import argparse
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
for _p in (HERE, TOOLS, os.path.join(TOOLS, "eval")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import m3dgr_calib as mc  # noqa: E402
import p4_calib as P4  # noqa: E402

ITERS = 12                        # as p4_calib.fit_lever
GT_GAP_S = 0.5                    # as p4_calib.frame_prep


# ------------------------------------------------------------------------------------------------------------- fit
def fit_level_lever(P, R_imu_from_level, vz, iters=ITERS):
    """A58's lever fit with the free/held split in the level frame. P: dict with p (n,3), R (n,3,3), g (n,3).
    -> (l in the IMU frame, v = (vx, vy, vz) in the level frame, rms of the aligned residual at the lever point)."""
    Ril = np.asarray(R_imu_from_level, float).reshape(3, 3)
    B = Ril[:, :2]
    c = Ril[:, 2] * vz
    l = Ril @ np.array([0.0, 0.0, vz])
    for _ in range(iters):
        a = P["p"] + np.einsum("nij,j->ni", P["R"], l)
        Rg, tg = P4.umeyama(a, P["g"])
        res = P["g"] - (P["p"] @ Rg.T + tg)
        A = np.einsum("ij,njk->nik", Rg, P["R"])               # Rg R_t
        rhs = (res - np.einsum("nij,j->ni", A, c)).reshape(-1)
        M = np.einsum("nij,jk->nik", A, B).reshape(-1, 2)
        v = np.linalg.lstsq(M, rhs, rcond=None)[0]
        l = B @ v + c
    a = P["p"] + np.einsum("nij,j->ni", P["R"], l)
    Rg, tg = P4.umeyama(a, P["g"])
    rms = float(np.sqrt(np.mean(np.sum((P["g"] - (a @ Rg.T + tg)) ** 2, axis=1))))
    return l, np.array([v[0], v[1], vz]), rms


def prep_est(est, gt):
    """The `frame` data path for eval_ate's masked est_e1.tum / gt_eval.tum arrays (rows t x y z qx qy qz qw)."""
    tg, te = gt[:, 0], est[:, 0]
    j = np.searchsorted(tg, te)
    ok = (j > 0) & (j < len(tg))
    j = np.clip(j, 1, len(tg) - 1)
    ok &= (tg[j] - tg[j - 1]) <= GT_GAP_S
    f = (te - tg[j - 1]) / np.maximum(tg[j] - tg[j - 1], 1e-9)
    g = gt[j - 1, 1:4] * (1 - f)[:, None] + gt[j, 1:4] * f[:, None]
    Rs = np.array([P4.quat_to_R(row[4:8]) for row in est[ok]])
    return {"t": te[ok], "p": est[ok, 1:4], "R": Rs, "g": g[ok]}


def load_est(est_path, gt_path):
    import evalcommon as ec
    return prep_est(ec.read_tum(est_path), ec.read_tum(gt_path))


def load_run(root, seq, run_dir):
    import evalcommon as ec
    _, seqreg = ec.load_sequences()
    return P4.frame_prep(root, seq, seqreg[seq], run_dir)


def median_pair(vals):
    return [float(np.median([v[0] for v in vals])), float(np.median([v[1] for v in vals]))]


def summarise(per_run, order=None):
    """per_run: list of dicts {seq, name, v:[vx, vy, vz], l:[...]}; -> A58-literal (first run of each sequence, median over
    sequences), median over the seeds of each sequence then over sequences, and over all runs."""
    seqs = order or sorted({r["seq"] for r in per_run})
    by = {s: [r for r in per_run if r["seq"] == s] for s in seqs}
    first = {s: by[s][0]["v"][:2] for s in seqs}
    med = {s: median_pair([r["v"][:2] for r in by[s]]) for s in seqs}
    spread = {s: [[float(min(r["v"][i] for r in by[s])), float(max(r["v"][i] for r in by[s]))] for i in (0, 1)] for s in seqs}
    return {"sequences": seqs, "runs_per_sequence": {s: len(by[s]) for s in seqs},
            "per_sequence_first_run": first, "per_sequence_seed_median": med, "per_sequence_seed_range": spread,
            "a58_literal_median_over_sequences": median_pair(list(first.values())),
            "seed_median_then_sequence_median": median_pair(list(med.values())),
            "median_over_all_runs": median_pair([r["v"][:2] for r in per_run])}


# ----------------------------------------------------------------------------------------------------------- derive
def derive_a77(rows, lever_ref, vxy, l9_avia=None):
    """A77 decisions 2-3 (see the module docstring). rows = m3dgr_calib.parse_calibration(...)[0]."""
    lever_ref = np.asarray(lever_ref, float)
    res = mc.derive(rows, lever_ref)
    failed = [c for c in res["checks"] if not c[1]]
    if failed:
        raise ValueError("calibration self-checks failed: %s" % failed)
    lv = res["lever"]
    R_am = rows[("avia", "mid360")][0]                          # IMU axes = R_am @ level axes
    R_mi, t_mi = rows[("mid360", "mid360_imu")]
    if not np.allclose(R_mi, np.eye(3)):
        raise ValueError("mid360 -> mid360_imu is not a pure translation: the lidar-frame conversion below assumes it")
    chain_level = R_am.T @ lv["gt_mid360_imu"]                  # the chain's own level-frame vector, IMU origin -> GT antenna
    vz = float(chain_level[2])
    v = np.array([float(vxy[0]), float(vxy[1]), vz])
    l_gt = R_am @ v
    l9 = lv["l_avia_imu"] if l9_avia is None else np.asarray(l9_avia, float)
    d = l9 - lever_ref
    d_h = np.array([d[0], d[1], 0.0])                           # same height: A58 holds the GT vertical at the 9Ps height
    l_9ps = l_gt + R_am @ d_h
    l_9ps_lidar = l_9ps - t_mi
    out = {"lever_ref_avia_imu": lever_ref.tolist(), "l9ps_avia_imu": np.asarray(l9).tolist(),
           "offset_9ps_minus_gt_avia_axes": d_h.tolist(), "offset_vertical_dropped_m": float(d[2]),
           "vz_chain_level_m": vz, "chain_level_vector": chain_level.tolist(), "level_vector_used": v.tolist(),
           "gt_lever_mid360_imu": l_gt.tolist(), "l9ps_mid360_imu": l_9ps.tolist(),
           "l9ps_mid360_lidar": l_9ps_lidar.tolist(), "t_I_from_L": t_mi.tolist(),
           "chain": {"gt_mid360_imu": lv["gt_mid360_imu"].tolist(), "l9ps_mid360_imu": lv["l_mid360_imu"].tolist(),
                     "l9ps_mid360_lidar": lv["p_ant_mid360"].tolist(),
                     "l9ps_level_from_mid360_imu": (R_am.T @ lv["l_mid360_imu"]).tolist()},
           "l9ps_level_from_mid360_imu": (R_am.T @ l_9ps).tolist()}
    # self-checks
    chk = []
    # 1) the chain's own level components return the chain values (the construction is the chain when nothing is fitted)
    back = R_am @ chain_level
    e1 = float(np.max(np.abs(back - lv["gt_mid360_imu"])))
    back9 = back + R_am @ d_h
    e2 = float(np.max(np.abs(back9 + R_am @ np.array([0.0, 0.0, d[2]]) - lv["l_mid360_imu"])))
    chk.append(("chain level components reproduce the chain GT lever (R_am (R_am^T l) = l)", e1 < 5e-6, "max |d| %.1e m" % e1))
    chk.append(("chain components + offset reproduce the chain 9Ps lever up to the dropped vertical offset", e2 < 5e-6,
                "max |d| %.1e m (vertical offset dropped: %+.4f m)" % (e2, d[2])))
    # 2) rigid: GT -> 9Ps distance is |d_h| whatever the fitted components
    dist = float(np.linalg.norm(l_9ps - l_gt))
    chk.append(("|9Ps - GT antenna| = |offset|", abs(dist - float(np.linalg.norm(d_h))) < 5e-6, "%.7f m vs %.7f m" % (dist, np.linalg.norm(d_h))))
    # 3) the held vertical is the chain's: level z of the result
    zlev = float((R_am.T @ l_gt)[2])
    chk.append(("level-frame height of the GT antenna over the MID-360 IMU = chain value", abs(zlev - vz) < 5e-6, "%.7f vs %.7f" % (zlev, vz)))
    # 4) the lidar-frame conversion returns the IMU-frame lever through the extrinsics (l^I = R_IL l^L + t_IL)
    e4 = float(np.max(np.abs(R_mi @ l_9ps_lidar + t_mi - l_9ps)))
    chk.append(("l^I = R_IL l^L + t_IL returns the 9Ps lever in the IMU frame", e4 < 1e-12, "max |d| %.1e m" % e4))
    # 5) the chain's own 9Ps lidar-frame lever goes through the same conversion (config value before A77)
    e5 = float(np.max(np.abs((lv["l_mid360_imu"] - t_mi) - lv["p_ant_mid360"])))
    chk.append(("the same conversion applied to the chain 9Ps lever gives the config value of before A77", e5 < 1e-9, "max |d| %.1e m" % e5))
    out["checks"] = [{"name": n, "ok": bool(ok), "detail": d_} for n, ok, d_ in chk]
    if not all(ok for _, ok, _ in chk):
        raise ValueError("A77 derivation self-checks failed: %s" % [c for c in chk if not c[1]])
    return out


def _parse_pairs(items, what):
    out = []
    for it in items or []:
        if "=" not in it:
            raise SystemExit("%s: %r is not SEQ=PATH" % (what, it))
        k, v = it.split("=", 1)
        out.append((k, v))
    return out


def cmd_fit(a):
    Ril = np.array(a.level_from_imu, float).reshape(3, 3).T          # R_imu_from_level
    per_run = []
    for seq, run in _parse_pairs(a.run, "--run"):
        Pd = load_run(a.root, seq, run)
        l, v, rms = fit_level_lever(Pd, Ril, a.vz)
        per_run.append({"seq": seq, "name": run, "source": "traj_W.tum", "n": int(len(Pd["t"])), "v": v.tolist(), "l_imu": l.tolist(), "rms_m": rms})
    for seq, spec in _parse_pairs(a.est, "--est"):
        est, gt = spec.split(":", 1)
        Pd = load_est(est, gt)
        l, v, rms = fit_level_lever(Pd, Ril, a.vz)
        per_run.append({"seq": seq, "name": est, "source": "est_e1.tum", "n": int(len(Pd["t"])), "v": v.tolist(), "l_imu": l.tolist(), "rms_m": rms})
    out = {"level_from_imu": list(a.level_from_imu), "vz_held_m": a.vz, "per_run": per_run, "summary": summarise(per_run)}
    with open(a.out, "w") as f:
        json.dump(out, f, indent=1)
    s = out["summary"]
    for q in per_run:
        print("%-16s %-60s n=%5d  level (%+.4f, %+.4f)  rms %.4f" % (q["seq"], q["name"][-60:], q["n"], q["v"][0], q["v"][1], q["rms_m"]))
    for k in ("a58_literal_median_over_sequences", "seed_median_then_sequence_median", "median_over_all_runs"):
        print("%-36s (%+.4f, %+.4f)" % (k, s[k][0], s[k][1]))
    print("wrote %s" % a.out)
    return 0


def cmd_derive(a):
    with open(a.calibration) as f:
        rows = mc.parse_calibration(f.read())[0]
    res = derive_a77(rows, a.lever_ref, a.vxy)
    for k in ("level_vector_used", "gt_lever_mid360_imu", "l9ps_mid360_imu", "l9ps_mid360_lidar", "l9ps_level_from_mid360_imu"):
        print("%-30s %s" % (k, mc._v(res[k])))
    print("self-checks: %d, all passed" % len(res["checks"]))
    if a.out:
        with open(a.out, "w") as f:
            json.dump(res, f, indent=1)
        print("wrote %s" % a.out)
    return 0


def build_parser():
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0], formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)
    f = sub.add_parser("fit")
    f.add_argument("--root", default="/r")
    f.add_argument("--level-from-imu", type=float, nargs=9, required=True, metavar="R", help="row-major R_level_from_imu (lidars.json)")
    f.add_argument("--vz", type=float, required=True, help="the held level-frame vertical component (m)")
    f.add_argument("--run", action="append", default=[], help="SEQ=<run dir with traj_W.tum>")
    f.add_argument("--est", action="append", default=[], help="SEQ=<est_e1.tum>:<gt_eval.tum>")
    f.add_argument("--out", required=True)
    f.set_defaults(fn=cmd_fit)
    d = sub.add_parser("derive")
    d.add_argument("--calibration", required=True)
    d.add_argument("--lever-ref", type=float, nargs=3, required=True, metavar=("X", "Y", "Z"))
    d.add_argument("--vxy", type=float, nargs=2, required=True, metavar=("VX", "VY"), help="fitted level-frame horizontal components of the GT antenna seen from the MID-360 IMU")
    d.add_argument("--out", default="")
    d.set_defaults(fn=cmd_derive)
    return p


def main(argv=None):
    a = build_parser().parse_args(argv)
    return a.fn(a)


if __name__ == "__main__":
    sys.exit(main())
