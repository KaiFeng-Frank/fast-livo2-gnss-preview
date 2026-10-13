#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Camera <-> LiDAR extrinsic check on a slim bag from the data alone (no estimator, no ground truth): the LiDAR scans of a
stationary stretch are accumulated, projected into the camera frame with the configured Rcl/Pcl and the Kalibr intrinsics, and
scored against the image; the same projection is repeated with small rotations of the camera about its own axes.

  overlay_check.py <slim bag> --out-dir <dir> [--config config/smartpnt_mini.yaml --camera config/smartpnt_mini_camera.yaml]
                   [--t-from S --t-to S]   (seconds after the first IMU stamp; default: the longest still stretch of the first 150 s)

Scores (higher = better aligned), as functions of the rotation offset (yaw = about the camera's vertical axis, pitch = about its
horizontal axis, roll = about its optical axis; offsets are applied to the camera-frame points):
  edge    the mean image-gradient magnitude (Gaussian-smoothed) at the projected LiDAR depth-edge points (a range jump of more than
          0.5 m against both azimuth neighbours of the same ring, the nearer point of the jump); object borders have a depth jump
          in the LiDAR and an intensity jump in the image;
  mi      the mutual information between the LiDAR intensity and the image grey value at the projected points (all points).
The report gives, per score and per frame, the offset of the maximum and the score ratio at zero offset against the maximum.
The overlay PNGs (grey image, points coloured by range; the second picture of each pair is the same projection with 3 degrees
of yaw error for comparison) are written to --out-dir for inspection. numpy and cv2 only.
"""
import argparse
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import numpy as np  # noqa: E402

from dscommon import rosbytes  # noqa: E402


def read_yaml_flat(path):
    import yaml
    with open(path) as f:
        return yaml.safe_load(f)


def read_stream(bag, topics, t_from_ns=None, t_to_ns=None):
    from rosbags.rosbag1 import Reader
    with Reader(bag) as r:
        conns = [c for c in r.connections if c.topic in topics]
        for c, ts, raw in r.messages(connections=conns, start=t_from_ns, stop=t_to_ns):
            yield c.topic, ts, raw


def still_window(bag, search_s=150.0, min_s=8.0):
    """Longest stretch (>= min_s) in the first search_s seconds with a small gyroscope norm and accelerometer scatter. -> (t0_ns, t1_ns)."""
    t, g, a = [], [], []
    t_first = None
    for topic, ts, raw in read_stream(bag, ["/imu_raw"]):
        d = rosbytes.parse_imu(raw)
        if t_first is None:
            t_first = d["stamp_ns"]
        if d["stamp_ns"] - t_first > search_s * 1e9:
            break
        t.append(d["stamp_ns"])
        g.append(d["gyro"])
        a.append(d["acc"])
    t = (np.array(t) - t_first) / 1e9
    g, a = np.array(g), np.array(a)
    n = int(round(1.0 / np.median(np.diff(t))))              # 1 s blocks
    nb = len(t) // n
    still = []
    for k in range(nb):
        s = slice(k * n, (k + 1) * n)
        still.append(np.linalg.norm(g[s].mean(axis=0)) < 0.004 and np.linalg.norm(a[s].std(axis=0)) < 0.08)
    best, cur = (0, 0), None
    for k, v in enumerate(still + [False]):
        if v and cur is None:
            cur = k
        if not v and cur is not None:
            if k - cur > best[1] - best[0]:
                best = (cur, k)
            cur = None
    if best[1] - best[0] < min_s:
        raise SystemExit("no still stretch of %g s in the first %g s" % (min_s, search_s))
    return t_first + int(best[0] * 1e9), t_first + int(best[1] * 1e9), float(best[0]), float(best[1])


def project(pc, fx, fy, cx, cy, dist, w, h):
    """Camera-frame points (n x 3) -> pixels (u, v) with radtan distortion, and the in-image mask."""
    z = pc[:, 2]
    ok = z > 0.5
    x = np.where(ok, pc[:, 0] / np.where(ok, z, 1), 0)
    y = np.where(ok, pc[:, 1] / np.where(ok, z, 1), 0)
    k1, k2, p1, p2 = dist
    r2 = x * x + y * y
    rad = 1 + k1 * r2 + k2 * r2 * r2
    xd = x * rad + 2 * p1 * x * y + p2 * (r2 + 2 * x * x)
    yd = y * rad + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y
    u, v = fx * xd + cx, fy * yd + cy
    ok &= (u >= 0) & (u < w - 1) & (v >= 0) & (v < h - 1)
    return u, v, ok


def rot_xyz(ax, ay, az):
    """Rotation by small angles [rad] about the camera x (pitch), y (yaw), z (roll) axes."""
    cx_, sx_, cy_, sy_, cz_, sz_ = math.cos(ax), math.sin(ax), math.cos(ay), math.sin(ay), math.cos(az), math.sin(az)
    Rx = np.array([[1, 0, 0], [0, cx_, -sx_], [0, sx_, cx_]])
    Ry = np.array([[cy_, 0, sy_], [0, 1, 0], [-sy_, 0, cy_]])
    Rz = np.array([[cz_, -sz_, 0], [sz_, cz_, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def mutual_information(a, b, bins=16):
    ia = np.clip((a * bins / 256.0).astype(int), 0, bins - 1)
    ib = np.clip((b * bins / 256.0).astype(int), 0, bins - 1)
    j = np.bincount(ia * bins + ib, minlength=bins * bins).reshape(bins, bins).astype(float)
    j /= j.sum()
    pa, pb = j.sum(axis=1), j.sum(axis=0)
    nz = j > 0
    return float(np.sum(j[nz] * np.log(j[nz] / np.outer(pa, pb)[nz])))


def depth_edges(xyz, ring):
    """Points whose range jumps by more than 0.5 m against BOTH azimuth neighbours of the same ring (the nearer side of an object border)."""
    rng = np.linalg.norm(xyz, axis=1)
    az = np.arctan2(xyz[:, 1], xyz[:, 0])
    idx = []
    for r in np.unique(ring):
        s = np.nonzero(ring == r)[0]
        s = s[np.argsort(az[s])]
        d = rng[s]
        for_edge = (np.abs(d[1:-1] - d[:-2]) > 0.5) & (np.abs(d[1:-1] - d[2:]) > 0.5) & (d[1:-1] < d[:-2]) & (d[1:-1] < d[2:])
        idx.append(s[1:-1][for_edge])
    return np.concatenate(idx) if idx else np.zeros(0, dtype=int)


def main(argv=None):
    import cv2
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("bag")
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--config", default=os.path.join(HERE, "..", "..", "..", "config", "smartpnt_mini.yaml"))
    ap.add_argument("--camera", default=os.path.join(HERE, "..", "..", "..", "config", "smartpnt_mini_camera.yaml"))
    ap.add_argument("--t-from", type=float, default=None)
    ap.add_argument("--t-to", type=float, default=None)
    ap.add_argument("--frames", type=int, default=3)
    ap.add_argument("--max-scans", type=int, default=80)
    ap.add_argument("--range-max-deg", type=float, default=3.0, help="half range of the rotation scan [deg]")
    ap.add_argument("--step-deg", type=float, default=0.25)
    ap.add_argument("--seq", default="")
    a = ap.parse_args(argv)
    os.makedirs(a.out_dir, exist_ok=True)
    seq = a.seq or os.path.basename(a.bag).replace(".bag", "")
    ex = read_yaml_flat(a.config)["extrin_calib"]
    Rcl, Pcl = np.array(ex["Rcl"], dtype=float).reshape(3, 3), np.array(ex["Pcl"], dtype=float)
    cam = read_yaml_flat(a.camera)
    sc = float(cam.get("scale", 1.0))
    fx, fy, cx, cy = [float(cam[k]) * sc for k in ("cam_fx", "cam_fy", "cam_cx", "cam_cy")]
    dist = [float(cam[k]) for k in ("cam_d0", "cam_d1", "cam_d2", "cam_d3")]
    W, H = int(cam["cam_width"] * sc), int(cam["cam_height"] * sc)
    if a.t_from is None:
        t0_ns, t1_ns, s0, s1 = still_window(a.bag)
    else:
        from rosbags.rosbag1 import Reader
        with Reader(a.bag) as r:
            first = next(r.messages(connections=[c for c in r.connections if c.topic == "/imu_raw"]))[1]
        t0_ns, t1_ns, s0, s1 = int(first + a.t_from * 1e9), int(first + a.t_to * 1e9), a.t_from, a.t_to
    # LiDAR scans and camera frames in the window
    clouds, frames = [], []
    for topic, ts, raw in read_stream(a.bag, ["/velodyne_points", "/cam00/image_raw/compressed"], t0_ns, t1_ns):
        if topic == "/velodyne_points":
            if len(clouds) < a.max_scans:
                c = rosbytes.Cloud(raw)
                xyz = np.stack([c.column(raw, n) for n in ("x", "y", "z")], axis=1)
                it = c.column(raw, "intensity")
                rg = c.column(raw, "ring", "<u2")
                ok = np.isfinite(xyz).all(axis=1) & (np.linalg.norm(xyz, axis=1) > 1.5)
                clouds.append((xyz[ok], it[ok], rg[ok]))
        else:
            frames.append(rosbytes.parse_compressed(raw)["data"])
    if not clouds or not frames:
        raise SystemExit("no scans (%d) or frames (%d) in the window" % (len(clouds), len(frames)))
    xyz = np.concatenate([c[0] for c in clouds])
    inten = np.concatenate([c[1] for c in clouds])
    edge_pts = np.concatenate([c[0][depth_edges(c[0], c[2])] for c in clouds])
    out = {"sequence": seq, "bag": os.path.abspath(a.bag), "window_s_after_first_imu": [s0, s1], "scans": len(clouds),
           "points": int(len(xyz)), "depth_edge_points": int(len(edge_pts)), "image_size": [W, H],
           "Rcl": Rcl.tolist(), "Pcl": Pcl.tolist(), "intrinsics_at_image_size": [fx, fy, cx, cy], "frames": []}
    offs = np.arange(-a.range_max_deg, a.range_max_deg + 1e-9, a.step_deg)
    sel = np.linspace(0, len(frames) - 1, a.frames).round().astype(int)
    for fi, k in enumerate(sel):
        img = cv2.imdecode(np.frombuffer(frames[k], np.uint8), cv2.IMREAD_GRAYSCALE)
        if img is None or img.shape != (H, W):
            raise SystemExit("frame %d is not %dx%d" % (k, W, H))
        gx = cv2.Sobel(img.astype(np.float32), cv2.CV_32F, 1, 0, ksize=3)
        gy = cv2.Sobel(img.astype(np.float32), cv2.CV_32F, 0, 1, ksize=3)
        grad = cv2.GaussianBlur(np.sqrt(gx * gx + gy * gy), (0, 0), 2.0)
        pcam_all = xyz @ Rcl.T + Pcl
        pcam_edge = edge_pts @ Rcl.T + Pcl

        def edge_score(R):
            u, v, ok = project(pcam_edge @ R.T, fx, fy, cx, cy, dist, W, H)
            return float(grad[v[ok].astype(int), u[ok].astype(int)].mean()) if ok.sum() > 20 else float("nan")

        def mi_score(R):
            u, v, ok = project(pcam_all @ R.T, fx, fy, cx, cy, dist, W, H)
            return mutual_information(inten[ok], img[v[ok].astype(int), u[ok].astype(int)].astype(float)) if ok.sum() > 500 else float("nan")

        rec = {"frame": int(k)}
        for name, fn in (("edge", edge_score), ("mi", mi_score)):
            res = {}
            for axis, mk in (("yaw", lambda d: rot_xyz(0, math.radians(d), 0)), ("pitch", lambda d: rot_xyz(math.radians(d), 0, 0)),
                             ("roll", lambda d: rot_xyz(0, 0, math.radians(d)))):
                s = np.array([fn(mk(d)) for d in offs])
                k0 = int(np.argmin(np.abs(offs)))
                kb = int(np.nanargmax(s))
                if 0 < kb < len(s) - 1 and np.isfinite(s[kb - 1]) and np.isfinite(s[kb + 1]):
                    dd = s[kb - 1] - 2 * s[kb] + s[kb + 1]
                    best = float(offs[kb] + (0.5 * (s[kb - 1] - s[kb + 1]) / dd * (offs[1] - offs[0]) if dd < 0 else 0.0))
                else:
                    best = float(offs[kb])
                res[axis] = {"best_offset_deg": best, "score_at_zero": float(s[k0]), "score_at_best": float(s[kb]),
                             "zero_over_best": float(s[k0] / s[kb]), "score_at_minus_range": float(s[0]), "score_at_plus_range": float(s[-1])}
            rec[name] = res
        # overlay pictures: calibrated, and 3 degrees of yaw error
        for tag, R in (("calibrated", np.eye(3)), ("yaw3deg", rot_xyz(0, math.radians(3.0), 0))):
            u, v, ok = project(pcam_all @ R.T, fx, fy, cx, cy, dist, W, H)
            z = (pcam_all @ R.T)[ok, 2]
            col = cv2.applyColorMap(np.clip((z - 2) / 28 * 255, 0, 255).astype(np.uint8).reshape(-1, 1), cv2.COLORMAP_JET).reshape(-1, 3)
            pic = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            pic[v[ok].astype(int), u[ok].astype(int)] = col
            ue, ve, oke = project(pcam_edge @ R.T, fx, fy, cx, cy, dist, W, H)
            for uu, vv in zip(ue[oke].astype(int), ve[oke].astype(int)):
                cv2.circle(pic, (uu, vv), 2, (255, 255, 255), -1)
            p = os.path.join(a.out_dir, "overlay_%s_f%d_%s.png" % (seq, fi, tag))
            cv2.imwrite(p, pic)
            rec.setdefault("pictures", []).append(p)
        rec["points_in_image_calibrated"] = int(project(pcam_all, fx, fy, cx, cy, dist, W, H)[2].sum())
        out["frames"].append(rec)
    with open(os.path.join(a.out_dir, "overlay_check_%s.json" % seq), "w") as f:
        json.dump(out, f, indent=1)
    print(json.dumps(out, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
