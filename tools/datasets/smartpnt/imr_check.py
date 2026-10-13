#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""The slim bag's IMU against the platform's own I300.imr (format level: nothing of an estimator is read, allowed on every class
of sequence): the record times, the units of the increments and the number of samples.

  imr_check.py <slim bag> <I300.imr> [--out report.json]

IMR (Waypoint/NovAtel `$IMURAW`, little endian): 512-byte header (data rate 200 Hz, gyro and accelerometer scale factors, delta-theta /
delta-v flags, time = GPS seconds of week, `iUtcOrGpsTime`, `iRcvTimeOrCorrTime`), then 32-byte records: double time, three gyro and
three accelerometer int32. Findings on the first development sequence: the IMR gyro unit is DEGREES per sample (counts x scale
factor), the bag's /imu_raw increments are radians per sample (bag = IMR x pi/180 to 1e-9), the accelerometer increments are
m/s per sample in both, and the bag stamps are the IMR times with the date replaced. The slim bag stores rates (increments x 200).
"""
import argparse
import json
import math
import os
import struct
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import numpy as np  # noqa: E402

from dscommon import rosbytes  # noqa: E402
from fl2g import gnsstime  # noqa: E402


def read_imr(path):
    with open(path, "rb") as f:
        h = f.read(512)
        body = f.read()
    if h[:8] != b"$IMURAW\x00":
        raise ValueError("%s: not an IMR file" % path)
    dT, dV, rate, gsf, asf, utc_gps, rcv_corr, bias = struct.unpack_from("<iidddiid", h, 17)
    n = len(body) // 32
    a = np.frombuffer(body[:n * 32], dtype=np.dtype([("t", "<f8"), ("v", "<i4", 6)]))
    return {"rate_hz": rate, "delta_theta": dT, "delta_v": dV, "gyro_sf": gsf, "acc_sf": asf, "utc_or_gps": utc_gps,
            "rcv_or_corr": rcv_corr, "time_bias": bias, "t": a["t"].copy(), "gyro": a["v"][:, :3] * gsf, "acc": a["v"][:, 3:] * asf}


def read_slim_imu(bag):
    from rosbags.rosbag1 import Reader
    t, g, ac = [], [], []
    with Reader(bag) as r:
        conns = [c for c in r.connections if c.topic == "/imu_raw"]
        for c, ts, raw in r.messages(connections=conns):
            d = rosbytes.parse_imu(raw)
            t.append(d["stamp_ns"])
            g.append(d["gyro"])
            ac.append(d["acc"])
    return np.array(t, dtype=np.int64), np.array(g) / 200.0, np.array(ac) / 200.0


def compare(bag, imr_path):
    imr = read_imr(imr_path)
    t_ns, g, ac = read_slim_imu(bag)
    leap = gnsstime.leap_at_gpst(t_ns[0] / 1e9 + 18)
    gpst = t_ns / 1e9 + leap                                   # seconds since the Unix epoch on the GPST scale
    tow = (gpst - gnsstime.GPST0_UNIX) % gnsstime.WEEK_S
    idx = np.searchsorted(imr["t"], tow - 1e-4)
    idx = np.clip(idx, 0, imr["t"].size - 1)
    dt = tow - imr["t"][idx]
    ok = np.abs(dt) < 2e-6
    out = {"bag_samples": int(t_ns.size), "imr_samples": int(imr["t"].size), "matched_within_2us": int(ok.sum()),
           "max_abs_time_difference_s": float(np.max(np.abs(dt[ok]))) if ok.any() else None,
           "imr_header": {k: imr[k] for k in ("rate_hz", "delta_theta", "delta_v", "gyro_sf", "acc_sf", "utc_or_gps", "rcv_or_corr", "time_bias")},
           "unmatched_bag_samples": int((~ok).sum()), "imr_samples_not_in_bag": int(imr["t"].size - ok.sum())}
    if ok.any():
        gi, ai = imr["gyro"][idx[ok]], imr["acc"][idx[ok]]
        out["gyro_bag_minus_imr_deg_to_rad_max_abs"] = float(np.max(np.abs(g[ok] - gi * math.pi / 180.0)))
        out["gyro_bag_over_imr_ratio_median"] = float(np.median(np.abs(g[ok][:, 1:]) / np.maximum(np.abs(gi[:, 1:]), 1e-12)))
        out["acc_bag_minus_imr_max_abs"] = float(np.max(np.abs(ac[ok] - ai)))
    return out


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("bag")
    p.add_argument("imr")
    p.add_argument("--out", default=None)
    a = p.parse_args(argv)
    res = compare(a.bag, a.imr)
    txt = json.dumps(res, indent=1)
    if a.out:
        with open(a.out, "w") as f:
            f.write(txt + "\n")
    print(txt)
    return 0


if __name__ == "__main__":
    sys.exit(main())
