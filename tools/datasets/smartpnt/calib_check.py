#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""LiDAR <-> IMU tilt check on a slim bag from the data alone (no estimator, no ground truth).

  calib_check.py gravity <bag> [--t-from S] [--t-to S] [--out report.json]

Over a stationary stretch the accelerometer mean gives the up direction in the IMU frame (specific force), and the
dominant plane of every LiDAR scan (RANSAC over the points 2-25 m away, normal oriented towards the sensor) gives it in
the LiDAR frame. For a rigid LiDAR -> IMU rotation R (p_imu = R p_lidar) R up_L = up_I. The report lists, for each of the
24 axis-aligned rotations (the candidates of mini_LiDAR.yaml are among them), the angle between R up_L and up_I; the
rotations below 10 degrees are the tilt-consistent ones, which leaves the yaw about the vertical axis (four candidates)
to be decided by the LIO replay (README, "extrinsic selection").

The scipy of the project image is broken against its numpy (np.typeDict), so this tool needs numpy only.
"""
import argparse
import itertools
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import numpy as np  # noqa: E402

from dscommon import rosbytes  # noqa: E402

NS = 1e9


def cube_rotations():
    """The 24 proper rotations whose entries are 0 or +-1."""
    out = []
    for perm in itertools.permutations(range(3)):
        for signs in itertools.product((1, -1), repeat=3):
            R = np.zeros((3, 3))
            for i, (j, s) in enumerate(zip(perm, signs)):
                R[i, j] = s
            if np.linalg.det(R) > 0:
                out.append(R)
    return out


def ground_normal(xyz, rng, iters=200, thr=0.08):
    """(normal towards the sensor, inlier fraction) of the largest plane among the points 2..25 m away, or None."""
    r = np.linalg.norm(xyz, axis=1)
    p = xyz[(r > 2.0) & (r < 25.0)]
    if p.shape[0] < 300:
        return None
    best, bn, bd = 0, None, None
    for _ in range(iters):
        a, b, c = p[rng.choice(p.shape[0], 3, replace=False)]
        n = np.cross(b - a, c - a)
        nn = np.linalg.norm(n)
        if nn < 1e-6:
            continue
        n = n / nn
        d = -n @ a
        cnt = int(np.sum(np.abs(p @ n + d) < thr))
        if cnt > best:
            best, bn, bd = cnt, n, d
    if bn is None:
        return None
    inl = p[np.abs(p @ bn + bd) < thr]
    c = inl.mean(axis=0)
    _, _, vt = np.linalg.svd(inl - c, full_matrices=False)
    n = vt[2]
    if n @ (-c) < 0:
        n = -n
    return n, best / p.shape[0]


def read_window(path, t_from, t_to):
    from rosbags.rosbag1 import Reader
    scans, imu = [], []
    with Reader(path) as r:
        t0 = r.start_time
        conns = [c for c in r.connections if c.topic in ("/velodyne_points", "/imu_raw")]
        start = t0 + int(t_from * NS)
        stop = None if t_to is None else t0 + int(t_to * NS) + 1
        for c, t, raw in r.messages(connections=conns, start=start, stop=stop):       # only the window is read (bags are 10-20 GB)
            ts = (t - t0) / NS
            if ts < t_from or (t_to is not None and ts > t_to):
                continue
            if c.topic == "/imu_raw":
                d = rosbytes.parse_imu(raw)
                imu.append(list(d["gyro"]) + list(d["acc"]))
            else:
                cl = rosbytes.Cloud(raw)
                a = np.frombuffer(raw, dtype=np.uint8, count=cl.data_len, offset=cl.data_off).reshape(-1, cl.point_step)
                xyz = np.stack([np.ascontiguousarray(a[:, o:o + 4]).view("<f4")[:, 0] for o in (0, 4, 8)], axis=1).astype(float)
                scans.append(xyz[np.isfinite(xyz).all(axis=1)])
    return scans, np.array(imu)


def analyse(path, t_from, t_to, log=lambda m: None):
    scans, imu = read_window(path, t_from, t_to)
    log("%d scans, %d imu samples" % (len(scans), len(imu)))
    gyro, acc = imu[:, :3], imu[:, 3:]
    gyro_rms = float(np.sqrt(np.mean(np.sum((gyro - gyro.mean(axis=0)) ** 2, axis=1))))
    acc_mean = acc.mean(axis=0)
    rep = {"window_s": [t_from, t_to], "imu_samples": int(len(imu)), "scans": len(scans),
           "gyro_mean_rad_s": gyro.mean(axis=0).tolist(), "gyro_rms_about_mean_rad_s": gyro_rms,
           "acc_mean_m_s2": acc_mean.tolist(), "acc_norm_m_s2": float(np.linalg.norm(acc_mean)),
           "acc_std_m_s2": acc.std(axis=0).tolist()}
    fI = acc_mean / np.linalg.norm(acc_mean)
    rep["up_in_imu"] = fI.tolist()
    rng = np.random.RandomState(1)
    ups = []
    for s in scans:
        g = ground_normal(s, rng)
        if g is not None and g[1] > 0.15:
            ups.append(g[0])
    if len(ups) < 5:
        raise SystemExit("ground plane found in only %d scans" % len(ups))
    ups = np.array(ups)
    upL = ups.mean(axis=0)
    upL /= np.linalg.norm(upL)
    spread = np.degrees(np.arccos(np.clip(ups @ upL, -1, 1)))
    rep["up_in_lidar"] = upL.tolist()
    rep["up_in_lidar_scans_used"] = len(ups)
    rep["up_in_lidar_spread_deg"] = {"median": float(np.median(spread)), "p95": float(np.percentile(spread, 95))}
    cands = []
    for R in cube_rotations():
        ang = float(np.degrees(np.arccos(np.clip((R @ upL) @ fI, -1, 1))))
        cands.append((ang, R))
    cands.sort(key=lambda x: x[0])
    rep["tilt_consistent_cube_rotations"] = [{"R_imu_lidar": R.astype(int).tolist(), "up_angle_deg": a} for a, R in cands if a < 10.0]
    rep["best_up_angle_deg"] = cands[0][0]
    # smallest rotation taking up_L to up_I (the residual tilt of the platform's LiDAR mounting with the cube rotation applied)
    return rep


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("cmd", choices=["gravity"])
    p.add_argument("bag")
    p.add_argument("--t-from", type=float, default=0.0, help="seconds after the bag's first message")
    p.add_argument("--t-to", type=float, default=None)
    p.add_argument("--out", default=None)
    a = p.parse_args(argv)
    rep = analyse(a.bag, a.t_from, a.t_to, lambda m: sys.stderr.write(m + "\n"))
    txt = json.dumps(rep, indent=1)
    if a.out:
        with open(a.out, "w") as f:
            f.write(txt + "\n")
    print(txt)
    return 0


if __name__ == "__main__":
    sys.exit(main())
