#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Camera <-> IMU time offset of a slim bag from the data alone (no estimator, no GNSS, no ground truth: allowed on every class).

  cam_imu_toff.py <slim bag> --camchain <Kalibr camchain yaml> [--t-from S --t-to S] [--out report.json]

The rotation between two consecutive frames shifts the far field of the image by fx * dtheta (small angles): the global shift of
the upper part of the image (above the road, buildings and trees; phase correlation) is compared with the gyroscope's rotation
integral about the camera's vertical axis (gyro rotated into the camera frame with the Kalibr R_cam_imu) over the same interval
shifted by a trial offset delta: the camera stamp t_cam stands for the IMU time t_cam + delta (the meaning of Kalibr's
timeshift_cam_imu and of the host's img_time_offset). rho(delta) = corr(image shift, gyro integral) over the frame pairs with a
usable phase correlation peak; the report gives the delta of the extremum (parabolic), its correlation, and a block bootstrap
(30 s blocks) of the delta. The pitch channel (dy against the rotation about the camera's horizontal axis) is reported as a check.
Seconds are counted from the first IMU stamp of the bag. numpy and cv2 only.
"""
import argparse
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)

import numpy as np  # noqa: E402

from dscommon import rosbytes  # noqa: E402
import extrinsics  # noqa: E402


def read_window(bag, t_from, t_to):
    from rosbags.rosbag1 import Reader
    import cv2
    imu_t, imu_w, cam_t, cam_img = [], [], [], []
    with Reader(bag) as r:
        first = None
        for c, ts, raw in r.messages(connections=[x for x in r.connections if x.topic == "/imu_raw"]):
            first = ts
            break
        start, stop = first + int(t_from * 1e9), first + int(t_to * 1e9)
        conns = [x for x in r.connections if x.topic in ("/imu_raw", "/cam00/image_raw/compressed")]
        for c, ts, raw in r.messages(connections=conns, start=start, stop=stop):
            if c.topic == "/imu_raw":
                d = rosbytes.parse_imu(raw)
                imu_t.append(d["stamp_ns"] / 1e9)
                imu_w.append(d["gyro"])
            else:
                d = rosbytes.parse_compressed(raw)
                im = cv2.imdecode(np.frombuffer(d["data"], np.uint8), cv2.IMREAD_GRAYSCALE)
                im = cv2.resize(im, (im.shape[1] // 2, im.shape[0] // 2), interpolation=cv2.INTER_AREA)
                cam_t.append(d["stamp_ns"] / 1e9)
                cam_img.append(im)
    return np.array(imu_t), np.array(imu_w), np.array(cam_t), cam_img


def frame_shifts(imgs, top_fraction=0.4):
    """Phase-correlation shift (dx, dy) [px of the half-size image] and response between consecutive frames, upper part only."""
    import cv2
    h = imgs[0].shape[0]
    rows = slice(int(0.03 * h), int(top_fraction * h))
    win = cv2.createHanningWindow((imgs[0].shape[1], rows.stop - rows.start), cv2.CV_32F)
    out = []
    prev = None
    for im in imgs:
        cur = im[rows].astype(np.float32)
        if prev is not None:
            (dx, dy), resp = cv2.phaseCorrelate(prev, cur, win)
            out.append((dx, dy, resp))
        else:
            out.append((np.nan, np.nan, 0.0))
        prev = cur
    return np.array(out)


def peak(deltas, rho):
    k = int(np.nanargmax(np.abs(rho)))
    d = deltas[k]
    if 0 < k < len(rho) - 1:
        a, b, c = np.abs(rho[k - 1]), np.abs(rho[k]), np.abs(rho[k + 1])
        den = a - 2 * b + c
        if den < 0:
            d = deltas[k] + 0.5 * (a - c) / den * (deltas[1] - deltas[0])
    return float(d), float(rho[k])


def scan(cam_t, shift, pairs, cum_t, cum, deltas):
    """rho(delta) over the frame pairs `pairs` (index k = the interval frame k -> frame k+1; repeated indices count repeatedly)."""
    rho = np.full(len(deltas), np.nan)
    if len(pairs) < 30:
        return rho
    x = shift[pairs + 1]
    x = x - x.mean()
    for i, dl in enumerate(deltas):
        v = np.interp(cam_t[pairs + 1] + dl, cum_t, cum) - np.interp(cam_t[pairs] + dl, cum_t, cum)
        y = v - v.mean()
        den = np.sqrt(float(x @ x) * float(y @ y))
        if den > 0:
            rho[i] = float(x @ y) / den
    return rho


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("bag")
    ap.add_argument("--camchain", required=True)
    ap.add_argument("--t-from", type=float, required=True)
    ap.add_argument("--t-to", type=float, required=True)
    ap.add_argument("--min-response", type=float, default=0.05)
    ap.add_argument("--range-ms", type=float, default=120.0)
    ap.add_argument("--out", default="")
    a = ap.parse_args(argv)
    cam = extrinsics.parse_kalibr_cam(a.camchain)
    R_ci = cam["T_cam_imu"][:3, :3]
    fx = cam["intrinsics"][0] * 0.5 * 0.5                      # raw 1280 -> slim 640 -> the half-size images used here
    imu_t, imu_w, cam_t, imgs = read_window(a.bag, a.t_from, a.t_to)
    sh = frame_shifts(imgs)
    dt_cam = np.diff(cam_t)
    valid = np.isfinite(sh[:, 0]) & (sh[:, 2] >= a.min_response)
    valid[1:] &= (np.abs(dt_cam - 0.1) < 0.02)
    w_c = (imu_w - imu_w.mean(axis=0)) @ R_ci.T                 # rotation rate in the camera frame (x right, y down, z forward), bias removed
    cumsum = np.cumsum(0.5 * (w_c[1:] + w_c[:-1]) * np.diff(imu_t)[:, None], axis=0)
    cum3 = np.vstack([np.zeros((1, 3)), cumsum])
    deltas = np.arange(-a.range_ms, a.range_ms + 1e-9, 1.0) * 1e-3
    rep = {"bag": os.path.abspath(a.bag), "window_s_after_first_imu": [a.t_from, a.t_to], "frames": int(len(cam_t)),
           "pairs_valid": int(valid[1:].sum()), "fx_px_halfsize": fx, "R_cam_imu": R_ci.tolist(),
           "kalibr_timeshift_cam_imu_s": cam["timeshift_cam_imu"], "min_response": a.min_response}
    # yaw: rotation about the camera's vertical (y) axis moves the image horizontally; pitch: about x, vertically
    pairs = np.nonzero(valid[1:])[0]                          # pair k: frame k -> k+1 (valid[k + 1] describes the shift into frame k+1)
    for name, ch, comp in (("yaw", 1, 0), ("pitch", 0, 1)):
        x = sh[:, comp] / fx                                    # rad (index k + 1: the shift from frame k to k + 1)
        rho = scan(cam_t, x, pairs, imu_t, cum3[:, ch], deltas)
        d, r = peak(deltas, rho)
        blocks = np.floor((cam_t[pairs] - cam_t[0]) / 30.0).astype(int)
        ub = np.unique(blocks)
        rng = np.random.RandomState(0)
        bs = []
        for _ in range(40):
            pick = rng.choice(ub, len(ub), replace=True)
            idx = np.concatenate([pairs[blocks == b] for b in pick])
            rr = scan(cam_t, x, idx, imu_t, cum3[:, ch], deltas[::2])
            if np.isfinite(rr).any():
                bs.append(peak(deltas[::2], rr)[0])
        rep[name] = {"best_delta_s": d, "rho_at_best": r, "rho_at_zero": float(rho[int(round(a.range_ms))]),
                     "rho_at_kalibr_shift": float(np.interp(cam["timeshift_cam_imu"], deltas, np.nan_to_num(rho))),
                     "bootstrap_delta_s": {"median": float(np.median(bs)) if bs else None, "p16": float(np.percentile(bs, 16)) if bs else None,
                                           "p84": float(np.percentile(bs, 84)) if bs else None, "n": len(bs)},
                     "curve_every_10ms": {"%+d" % int(round(dl * 1e3)): float(rho[i]) for i, dl in enumerate(deltas) if int(round(dl * 1e3)) % 10 == 0}}
    txt = json.dumps(rep, indent=1)
    if a.out:
        with open(a.out, "w") as f:
            f.write(txt + "\n")
    print(txt)
    return 0


if __name__ == "__main__":
    sys.exit(main())
