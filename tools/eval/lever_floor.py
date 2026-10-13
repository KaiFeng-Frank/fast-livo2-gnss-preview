#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""The e1o floor of a host IMU point, from the ground truth alone (AMENDMENTS A75 section 3, A76).

    lever_floor.py [--lidar avia|mid360] [--seqs S1,S2,...] [--evaluation-phase] [--root DIR] [--out FILE.json]

e1o (and E1) compares the estimator's IMU point directly with the RTK antenna, without a lever arm. The antenna sits a few
decimetres from the IMU and turns with the heading, so even a perfect IMU trajectory has an ATE that no SE(3) alignment
removes. This tool computes that floor: the ATE (SE(3) Umeyama without scale, the e1o procedure) of the perfect IMU
trajectory, built from the GT antenna track only,

    p_IMU(t) = p_ant(t) - Rz(yaw(t)) v,        v = R_level_from_imu * gt_lever_imu       (lidars.json of the host LiDAR)

* heading: the GT has no attitude, so the yaw of the level frame is the direction of the horizontal GT velocity, a central
  difference over +-k GT rows (default k = 2); below vmin (0.2 m/s) the last valid heading is held (the first valid one
  before the first); roll and pitch are zero (ground vehicle; the Avia IMU frame is the level frame, gravity within 2-3
  degrees of its +z at the start of Grass01 and Outdoor01);
* v is the vector from the host IMU origin to the GT antenna in the level frame's axes. For the Avia IMU that is the A58
  lever (-0.176, -0.016, 0.401). The MID-360 IMU is a different point of the same vehicle: 0.036 m ahead of, 0.095 m to the
  left of and 0.431 m above the Avia IMU by the calibration chain, and its frame is tilted 31.8 degrees against the
  vertical, so v is the lidars.json lever rotated back to the level axes (R_level_from_imu). Since A77 that lever is the
  development-set fit of tools/calib/host_gt_lever.py on the MID-360 runs (horizontally 0.160 m: level (-0.049, -0.152)),
  not the chain's translation (which would give 0.240 m and disagrees with the data, A76 record section six). Only the
  horizontal part of v matters for the floor (the vertical part is a constant offset the alignment absorbs);
* the GT as the scoring sees it: gt_clean.tum shifted by the A54 time base, the A54 ate_exclude stretches removed.

The default heading rule is the one that reproduces A75's Avia table (ten sequences within 0.0015 m); the spread over other
reasonable rules (a 0.5 / 1 / 2 s window) is printed beside it, because the rule matters most on sequences with long
stops and spins (Grass01). The floor says what the e1o of a method could at best be, not what the method gets; the
calibrated-lever ATE (e1c) has no such floor beyond the lever arm's own error.
A2: this reads GT files only (no estimate, no replay), but the evaluation-set sequences still need --evaluation-phase.
Python >= 3.8, numpy only.
"""
import argparse
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402

import evalcommon as ec  # noqa: E402
import gtbase  # noqa: E402
from eval_ate import umeyama_se3  # noqa: E402

A75_SEQS = ("Varying-illu03", "Varying-illu04", "Varying-illu05", "Outdoor01", "Outdoor04", "Dark01", "Dark02",
            "Grass01", "Grass02", "Z-Rough-Road01")
RULES = (("k2", {"k": 2}), ("w0.5s", {"win_s": 0.5}), ("w1s", {"win_s": 1.0}), ("w2s", {"win_s": 2.0}))
DEFAULT_K = 2
DEFAULT_VMIN = 0.2


def heading_from_track(t, p, k=DEFAULT_K, win_s=None, vmin=DEFAULT_VMIN):
    """-> (yaw per row (rad), valid flag per row). Direction of the horizontal velocity: a central difference over +-k rows,
    or over a +-win_s/2 window; the last valid heading is held where the speed is below vmin."""
    n = len(t)
    if win_s is None:
        i0 = np.clip(np.arange(n) - k, 0, n - 1)
        i1 = np.clip(np.arange(n) + k, 0, n - 1)
    else:
        i0 = np.clip(np.searchsorted(t, t - win_s / 2.0), 0, n - 1)
        i1 = np.clip(np.searchsorted(t, t + win_s / 2.0) - 1, 0, n - 1)
        same = i1 <= i0
        i1 = np.where(same, np.clip(i0 + 1, 0, n - 1), i1)
        i0 = np.where(same & (i1 == i0), np.clip(i0 - 1, 0, n - 1), i0)
    dt = np.maximum(t[i1] - t[i0], 1e-6)
    v = (p[i1, :2] - p[i0, :2]) / dt[:, None]
    speed = np.hypot(v[:, 0], v[:, 1])
    yaw = np.arctan2(v[:, 1], v[:, 0])
    ok = speed > vmin
    if not ok.any():
        raise ValueError("the track never exceeds %.2f m/s: no heading" % vmin)
    idx = np.where(ok, np.arange(n), -1)
    np.maximum.accumulate(idx, out=idx)
    idx[idx < 0] = int(np.argmax(ok))
    return yaw[idx], ok


def perfect_imu_track(p_ant, yaw, v):
    """p_IMU = p_ant - Rz(yaw) v (the level frame turns about the vertical only)."""
    c, s = np.cos(yaw), np.sin(yaw)
    out = np.array(p_ant, dtype=float)
    out[:, 0] -= c * v[0] - s * v[1]
    out[:, 1] -= s * v[0] + c * v[1]
    out[:, 2] -= v[2]
    return out


def floor_of_track(t, p, keep, v, k=DEFAULT_K, win_s=None, vmin=DEFAULT_VMIN):
    """-> dict(floor_m, yaw_range_deg, ...) of the perfect IMU track against the antenna track on the rows `keep`."""
    yaw, ok = heading_from_track(t, p, k, win_s, vmin)
    p_imu = perfect_imu_track(p, yaw, np.asarray(v, dtype=float))
    a, b = p_imu[keep], p[keep]
    r, tt = umeyama_se3(a, b)
    e = np.linalg.norm(a @ r.T + tt - b, axis=1)
    uy = np.unwrap(yaw[keep])
    return {"floor_m": float(np.sqrt(np.mean(e ** 2))), "max_err_m": float(e.max()), "rows": int(keep.sum()),
            "yaw_range_deg": float(np.degrees(uy.max() - uy.min())), "moving_frac": float(np.mean(ok[keep]))}


def level_vector(profile, lever=None):
    """The vector from the host IMU origin to the GT antenna in the level frame's axes (see the module docstring)."""
    l = np.asarray(lever if lever is not None else profile["gt_lever_imu"], dtype=float)
    R = np.asarray(profile["R_level_from_imu"], dtype=float).reshape(3, 3)
    return R @ l


def sequence_floor(root, seq, info, v, k=DEFAULT_K, vmin=DEFAULT_VMIN, rules=RULES):
    gt, prov, mask = gtbase.load_gt(root, seq, info)
    t, p = gt[:, 0], gt[:, 1:4]
    keep = ~mask.inside(t)
    main = floor_of_track(t, p, keep, v, k=k, vmin=vmin)
    spread = {}
    for name, kw in rules:
        try:
            spread[name] = floor_of_track(t, p, keep, v, vmin=vmin, **kw)["floor_m"]
        except ValueError:
            spread[name] = None
    vals = [x for x in spread.values() if x is not None] + [main["floor_m"]]
    main.update(seq=seq, rule="k%d" % k, vmin_mps=vmin, rule_floors_m=spread, rule_min_m=min(vals), rule_max_m=max(vals),
                gt_rows=int(len(t)), masked_rows=int((~keep).sum()), gt_time_shift_s=prov.get("time_shift_applied_s"),
                gt_clean_sha256=prov.get("gt_clean_sha256"))
    return main


def build_parser():
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0], formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--lidar", default=None, choices=sorted(ec.load_lidars()[1]),
                   help="host LiDAR profile of tools/eval/lidars.json (default %s)" % ec.load_lidars()[0])
    p.add_argument("--seqs", default=",".join(A75_SEQS), help="comma list (default: the ten sequences of A75's table)")
    p.add_argument("--lever", type=float, nargs=3, default=None, metavar=("X", "Y", "Z"),
                   help="override the profile's gt_lever_imu (host IMU frame, m)")
    p.add_argument("--heading-k", type=int, default=DEFAULT_K, help="central-difference half width in GT rows (default %d)" % DEFAULT_K)
    p.add_argument("--vmin", type=float, default=DEFAULT_VMIN, help="speed below which the heading is held (m/s)")
    p.add_argument("--evaluation-phase", action="store_true", help="A2: required for evaluation-set sequences")
    p.add_argument("--root", default=None, help="project root holding audit/<seq>/gt (default FL2G_ROOT or the remote default)")
    p.add_argument("--out", default="", help="write the full result as JSON")
    return p


def main(argv=None):
    args = build_parser().parse_args(argv)
    root = ec.project_root(args.root)
    try:
        prof = ec.lidar_profile(args.lidar)
        _, reg = ec.load_sequences()
        seqs = ec.split_list(args.seqs)
        for s in seqs:                                    # A2 and the registry before any file is read
            if s not in reg:
                raise ec.Blocked("%s is not in tools/eval/sequences.json" % s)
            ec.check_phase(s, args.evaluation_phase, "the GT-only floor")
        v = level_vector(prof, args.lever)
        rows = []
        for s in seqs:
            rows.append(sequence_floor(root, s, reg[s], v, k=args.heading_k, vmin=args.vmin))
    except (ec.Blocked, ec.PhaseError, ValueError) as e:
        print("lever_floor: BLOCKED: %s" % e, file=sys.stderr)
        return 3
    lever = list(args.lever) if args.lever else list(prof["gt_lever_imu"])
    print("lever_floor: lidar %s, GT lever in its IMU frame %s, vector in the level frame (%.4f, %.4f, %.4f), horizontal norm %.4f m" % (
        prof["name"], lever, v[0], v[1], v[2], math.hypot(v[0], v[1])))
    print("%-16s %9s %9s %9s %9s %8s %7s" % ("sequence", "floor m", "rules min", "rules max", "yaw range", "rows", "masked"))
    for r in rows:
        print("%-16s %9.4f %9.4f %9.4f %8.0f° %8d %7d" % (r["seq"], r["floor_m"], r["rule_min_m"], r["rule_max_m"],
                                                          r["yaw_range_deg"], r["rows"], r["masked_rows"]))
    if args.out:
        ec.atomic_write_json(args.out, {"tool": "tools/eval/lever_floor.py", "generated_utc": ec.utc_now(), "argv": sys.argv,
                                        "lidar": prof["name"], "gt_lever_imu": lever, "R_level_from_imu": prof["R_level_from_imu"],
                                        "level_vector_m": [float(x) for x in v], "heading_k": args.heading_k, "vmin_mps": args.vmin,
                                        "sequences": rows})
    return 0


if __name__ == "__main__":
    sys.exit(main())
