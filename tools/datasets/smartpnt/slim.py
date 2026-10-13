# SPDX-License-Identifier: BSD-3-Clause
"""Message converters of the SmartPNT slim bag: IMU increments -> rates, camera JPEG shrinking, VLP-16 cloud time
normalisation, and the arrival model that gives every message its record time.

Everything here is a pure function of one raw message (plus the sequence's constant time shift), so it is unit tested
without any bag. Conventions (A80, see tools/datasets/README.md):

IMU       /imu_raw carries per-sample increments (delta angle [rad], delta velocity [m/s] over one 5 ms sample, the
          IMR header says 200 Hz; a static sample reads |dv| = 0.04908 = g * 0.005). The slim bag stores rates:
          value * 200. Orientation unknown (covariance[0] = -1), covariances unknown (0), frame_id `imu`.
camera    JPEG (1280x1024, 3 channels) -> cv2.resize INTER_LINEAR to the size the host would process (the config's
          cam_width*scale x cam_height*scale: the host's own processFrame resize, so the intrinsics of the camera file
          stay the raw-resolution calibration) -> JPEG again. Same format string as the source.
LiDAR     points untouched except the per-point `time` column: absolute time of a point = header stamp + time (seconds),
          whichever way the driver references the scan (frame start or frame tail). The slim cloud has header stamp =
          first point's time and time = microseconds since then, the unit the host's velodyne handler expects
          (preprocess.cpp: curvature = time / 1000 is milliseconds).
record    time of every message = the instant its data is complete (arrival model): IMU stamp; LiDAR stamp + scan
          duration; camera stamp + CAMERA_LATENCY; GNSS epoch + GNSS_LATENCY. The sources themselves carry no arrival
          times (bag time = header stamp), the replay delivers in record order (DESIGN §13).
"""
import numpy as np

from dscommon import rosbytes

IMU_RATE_HZ = 200.0
CAMERA_LATENCY_NS = 50 * 1000 * 1000        # Grass01's D435i: record - stamp = 45.8 ms
GNSS_LATENCY_NS = 80 * 1000 * 1000          # Grass01's F9P: range_meas record - GPST->UTC = 79.3 ms
IMU_FRAME_ID = b"imu"


def imu_to_rates(raw, shift_ns, seq=0, unwrap=None):
    """Source Imu bytes -> (stamp_ns, slim Imu bytes)."""
    d = rosbytes.parse_imu(raw)
    stamp = d["stamp_ns"] if unwrap is None else unwrap(d["stamp_ns"])
    stamp += shift_ns
    gyro = [v * IMU_RATE_HZ for v in d["gyro"]]
    acc = [v * IMU_RATE_HZ for v in d["acc"]]
    return stamp, rosbytes.build_imu(seq, stamp, IMU_FRAME_ID, gyro, acc)


class ImageSlimmer(object):
    """Decode -> resize -> encode. `size` = (width, height) of the output, quality = JPEG quality."""

    def __init__(self, size, quality=95):
        import cv2
        self.cv2 = cv2
        self.size = (int(size[0]), int(size[1]))
        self.quality = int(quality)

    def __call__(self, jpeg_bytes):
        cv2 = self.cv2
        img = cv2.imdecode(np.frombuffer(jpeg_bytes, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            raise ValueError("JPEG decode failed")
        out = cv2.resize(img, self.size, interpolation=cv2.INTER_LINEAR)
        ok, enc = cv2.imencode(".jpg", out, [int(cv2.IMWRITE_JPEG_QUALITY), self.quality])
        if not ok:
            raise ValueError("JPEG encode failed")
        return enc.tobytes()


def camera_to_slim(raw, shift_ns, slimmer, seq=0, unwrap=None):
    """Source CompressedImage bytes -> (stamp_ns, slim CompressedImage bytes)."""
    d = rosbytes.parse_compressed(raw)
    stamp = d["stamp_ns"] if unwrap is None else unwrap(d["stamp_ns"])
    stamp += shift_ns
    return stamp, rosbytes.build_compressed(seq, stamp, d["frame_id"], d["format"], slimmer(d["data"]))


class LidarTimeRule(object):
    """Unit of the per-point time column, fixed on the first scan with enough points and checked on every scan."""

    def __init__(self, unit=None):
        self.unit = unit
        self.reference = {"frame_start": 0, "frame_end": 0, "mixed": 0}      # how the source references the time column
        self.scans = 0
        self.flips = 0               # scans whose magnitude implies another unit than the fixed one
        self.nonmonotonic = 0
        self.span_min = None
        self.span_max = None

    def seconds(self, t_col):
        """float32 column -> float64 seconds; fixes/checks the unit."""
        mx = float(np.max(np.abs(t_col))) if t_col.size else 0.0
        if self.unit is None and t_col.size >= 100:
            self.unit = rosbytes.detect_time_unit(mx)
        elif self.unit is not None and t_col.size >= 100 and rosbytes.detect_time_unit(mx) != self.unit:
            self.flips += 1
        return t_col.astype(np.float64) * rosbytes.UNIT_TO_S[self.unit or "s"]


def lidar_to_slim(raw, shift_ns, rule, unwrap=None):
    """Source PointCloud2 bytes -> (record_ns, stamp_ns, slim cloud bytes, info).

    New stamp = old stamp + time.min() (the first point), new time = (time - time.min()) in microseconds, all else
    byte-identical (a scan whose time column is not non-decreasing is reordered by time first)."""
    c = rosbytes.Cloud(raw)
    stamp0 = c.stamp_ns if unwrap is None else unwrap(c.stamp_ns)
    t = rule.seconds(c.column(raw, "time", "<f4"))
    if t.size == 0:
        return None
    tmin, tmax = float(t.min()), float(t.max())
    rule.reference["frame_start" if tmin >= -1e-6 else "frame_end" if tmax <= 1e-6 else "mixed"] += 1
    rule.scans += 1
    rule.span_min = (tmax - tmin) if rule.span_min is None else min(rule.span_min, tmax - tmin)
    rule.span_max = (tmax - tmin) if rule.span_max is None else max(rule.span_max, tmax - tmin)
    stamp = stamp0 + int(round(tmin * 1e9)) + shift_ns
    t_us = ((t - tmin) * 1e6).astype(np.float32)
    body = raw
    if np.any(np.diff(t_us) < 0):
        rule.nonmonotonic += 1
        order = np.argsort(t_us, kind="stable")
        buf = bytearray(raw)
        a = np.frombuffer(buf, dtype=np.uint8, count=c.data_len, offset=c.data_off).reshape(-1, c.point_step)
        a[:] = a[order]
        t_us = t_us[order]
        body = bytes(buf)
    out = rosbytes.rewrite_cloud(body, c, stamp, t_us)
    record = stamp + int(round(float(t_us.max()) * 1e3))
    return record, stamp, out, {"npoints": int(t.size), "span_s": tmax - tmin}
