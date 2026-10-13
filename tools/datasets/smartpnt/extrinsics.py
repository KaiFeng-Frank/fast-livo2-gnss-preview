#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Host extrinsics of the SmartPNT mini platform: Kalibr camera-IMU chain + a LiDAR->IMU choice -> FAST-LIVO2's
extrin_calib block (extrinsic_R/T: p_imu = R p_lidar + T; Rcl/Pcl: p_cam = Rcl p_lidar + Pcl) and the camera yaml.

Inputs from the dataset: 03_config/cam_calibration/extrinsic_parameters/camchain-imucam-2024-06-26-02-19-12_0.yaml
(Kalibr, /cam00/image_raw with /imu_raw: T_cam_imu, pinhole radtan intrinsics, timeshift_cam_imu; t_imu = t_cam + shift)
and 03_config/mini_LiDAR.yaml (LIO-SAM style lidar<->IMU candidates, which one belongs to which IMU is NOT stated in
the file: calib_check.py decides from data). The LIO-SAM `extrinsicTrans` is the IMU origin in LiDAR axes
(imuPreintegration.cpp: lidarPose.compose(Pose3(Rot3(), extTrans)) = IMU pose); FAST-LIVO2 wants the LiDAR origin in
the IMU frame, T = -R_imu_lidar * extTrans.
"""
import argparse
import json
import sys

import numpy as np


def parse_kalibr_cam(path):
    """Minimal reader of a Kalibr camchain yaml (cam0 only): dict(T_cam_imu 4x4, intrinsics, distortion, resolution, shift)."""
    import yaml
    with open(path) as f:
        d = yaml.safe_load(f)["cam0"]
    return {"T_cam_imu": np.array(d["T_cam_imu"], dtype=float), "intrinsics": [float(x) for x in d["intrinsics"]],
            "distortion": [float(x) for x in d["distortion_coeffs"]], "resolution": [int(x) for x in d["resolution"]],
            "timeshift_cam_imu": float(d["timeshift_cam_imu"]), "model": d["camera_model"], "dist_model": d["distortion_model"],
            "rostopic": d["rostopic"]}


def compose_lidar_camera(T_cam_imu, R_imu_lidar, t_imu_lidar):
    """Rcl, Pcl with p_cam = Rcl p_lidar + Pcl."""
    R_ci, t_ci = T_cam_imu[:3, :3], T_cam_imu[:3, 3]
    return R_ci @ R_imu_lidar, R_ci @ t_imu_lidar + t_ci


def liosam_to_fastlivo(ext_rot, ext_trans, rot_is_imu_to_lidar=True):
    """LIO-SAM extrinsicRot/Trans -> (R_imu_lidar, t_imu_lidar). LIO-SAM rotates IMU measurements into the LiDAR frame
    with extRot, so extRot = R_lidar_imu and R_imu_lidar = extRot^T; extTrans is the IMU origin in LiDAR axes."""
    R = np.array(ext_rot, dtype=float).reshape(3, 3)
    R_il = R.T if rot_is_imu_to_lidar else R
    t_li = np.array(ext_trans, dtype=float)
    return R_il, -R_il @ t_li


def tilt_correction(R_il, gravity_reports):
    """The smallest rotation R_t with R_t (R_il up_L) = up_I on average over the static windows of `gravity_reports`
    (calib_check.py gravity JSONs: up_in_lidar from the ground plane of the scans, up_in_imu from the accelerometer).
    Returns (R_t, per-window residual angles [deg] before, after). The yaw about the vertical is untouched: only the
    tilt, which gravity observes, is corrected."""
    a_list, b_list = [], []
    for rep in gravity_reports:
        a = R_il @ np.array(rep["up_in_lidar"], dtype=float)
        b = np.array(rep["up_in_imu"], dtype=float)
        a_list.append(a / np.linalg.norm(a))
        b_list.append(b / np.linalg.norm(b))
    am, bm = np.mean(a_list, axis=0), np.mean(b_list, axis=0)
    am, bm = am / np.linalg.norm(am), bm / np.linalg.norm(bm)
    v = np.cross(am, bm)
    c = float(am @ bm)
    s = float(np.linalg.norm(v))
    if s < 1e-12:
        return np.eye(3), [], []
    k = v / s
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    R_t = np.eye(3) + s * K + (1 - c) * (K @ K)
    ang = lambda x, y: float(np.degrees(np.arccos(np.clip(x @ y, -1, 1))))
    return R_t, [ang(a, b) for a, b in zip(a_list, b_list)], [ang(R_t @ a, b) for a, b in zip(a_list, b_list)]


def fmt_list(v, per_row=None):
    v = [float(x) for x in np.asarray(v).reshape(-1)]
    return "[" + ", ".join("%.10g" % x for x in v) + "]"


def yaml_block(R_il, t_il, Rcl, Pcl):
    return ("extrin_calib:\n  extrinsic_T: %s\n  extrinsic_R: %s\n  Rcl: %s\n  Pcl: %s\n" %
            (fmt_list(t_il), fmt_list(R_il), fmt_list(Rcl), fmt_list(Pcl)))


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("camchain")
    p.add_argument("--ext-rot", type=float, nargs=9, default=[0, 0, -1, 0, -1, 0, -1, 0, 0],
                   help="LIO-SAM extrinsicRot (default: the active entry of mini_LiDAR.yaml)")
    p.add_argument("--ext-trans", type=float, nargs=3, default=[0.0, 0.0, 0.03411])
    p.add_argument("--gravity-check", action="append", default=[],
                   help="calib_check.py gravity JSON of a static window (repeatable): the LiDAR -> IMU tilt is corrected so that the "
                        "scans' ground normal and the accelerometer's up agree on average (the mount is not an exact axis permutation)")
    a = p.parse_args(argv)
    cam = parse_kalibr_cam(a.camchain)
    R_il, t_il = liosam_to_fastlivo(a.ext_rot, a.ext_trans)
    if a.gravity_check:
        reps = [json.load(open(f)) for f in a.gravity_check]
        R_t, before, after = tilt_correction(R_il, reps)
        sys.stderr.write("tilt correction: angle %.3f deg; per-window residual before %s deg, after %s deg\n" % (
            float(np.degrees(np.arccos(np.clip((np.trace(R_t) - 1) / 2, -1, 1)))), np.round(before, 3).tolist(), np.round(after, 3).tolist()))
        R_il = R_t @ R_il
        t_il = -R_il @ np.array(a.ext_trans, dtype=float)        # extTrans is the IMU origin in LiDAR axes: unchanged
    Rcl, Pcl = compose_lidar_camera(cam["T_cam_imu"], R_il, t_il)
    sys.stdout.write(yaml_block(R_il, t_il, Rcl, Pcl))
    print(json.dumps({"timeshift_cam_imu": cam["timeshift_cam_imu"], "intrinsics": cam["intrinsics"], "distortion": cam["distortion"]}))
    return 0


if __name__ == "__main__":
    sys.exit(main())
