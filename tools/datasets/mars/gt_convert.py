#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""MARS-LVIG ground truth: the DJI M300 RTK's position (`/dji_osdk_ros/rtk_position`, sensor_msgs/NavSatFix, WGS84, 5 Hz,
position only) -> the files tools/eval/gtbase.py reads (gt_clean.tum, gt_audited.tum, gt_timebase.yaml, trust_mask.csv),
plus gt_audit.json with the ENU origin (tools/datasets/smartpnt/gnss_check.py reads it).

Facts (survey of the bag, 2026-10-02): the RTK position carries a header stamp on the host clock and no orientation; the
GNSS antenna of the RTK is not the u-blox F9P's; the stamp is aligned to the GNSS time only to about 0.1 s (lag check against
the Doppler speed: best lag -0.06 s). `--shift-s` adds a measured shift to every stamp (A54's 20 ms rule: compensate only when
|shift| > 0.02 s); the method in gt_timebase.yaml says how the shift was found (`none` = 0, flagged; `xcorr` = the speed
cross-correlation against the estimator's or the receiver's speed).
The mask excludes epochs whose rtk_info_position status is not the best one seen (RTK fixed) and +-2 s around them.
"""
import argparse
import json
import os
import struct
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import numpy as np  # noqa: E402

from dscommon import rosbytes  # noqa: E402
from fl2g import geodesy  # noqa: E402

TOPIC_POS = "/dji_osdk_ros/rtk_position"
TOPIC_INFO = "/dji_osdk_ros/rtk_info_position"
MASK_WIDEN_S = 2.0


def parse_navsatfix(raw):
    """-> (stamp_ns, lat, lon, alt, status) of a NavSatFix message."""
    seq, stamp, fid, off = rosbytes.parse_header(raw)
    status, service = struct.unpack_from("<bH", raw, off)
    lat, lon, alt = struct.unpack_from("<ddd", raw, off + 3)
    return stamp, lat, lon, alt, status


def read_gt(bag):
    from rosbags.rosbag1 import Reader
    pos, info = [], []
    with Reader(bag) as r:
        conns = [c for c in r.connections if c.topic in (TOPIC_POS, TOPIC_INFO)]
        for c, t, raw in r.messages(connections=conns):
            if c.topic == TOPIC_POS:
                st, la, lo, al, status = parse_navsatfix(raw)
                pos.append((st, t, la, lo, al, status))
            else:
                info.append((t, raw[-1]))                      # std_msgs/UInt8: one byte
    return pos, info


def convert(bag, out_dir, seq, shift_s=0.0, method="none", mask_widen=MASK_WIDEN_S):
    os.makedirs(out_dir, exist_ok=True)
    pos, info = read_gt(bag)
    if not pos:
        raise SystemExit("no %s messages" % TOPIC_POS)
    a = np.array([(p[0] / 1e9, p[1] / 1e9, p[2], p[3], p[4], p[5]) for p in pos])
    stamp = a[:, 0]
    order = np.argsort(stamp, kind="stable")
    a = a[order]
    keep = np.concatenate([[True], np.diff(a[:, 0]) > 0])
    a = a[keep]
    lat, lon, alt = np.radians(a[:, 2]), np.radians(a[:, 3]), a[:, 4]
    enu = geodesy.geodetic_to_enu(lat, lon, alt, lat[0], lon[0], alt[0])
    t_clean = a[:, 0]                                       # gt_clean.tum: the RTK header stamps as recorded
    applied = shift_s if abs(shift_s) > 0.02 else 0.0       # A54's 20 ms rule
    t = t_clean + applied                                    # gt_audited.tum: on the estimator clock
    # rtk_info_position: status code per message; nearest in record time
    it = np.array([x[0] / 1e9 for x in info])
    iv = np.array([x[1] for x in info])
    if iv.size:
        j = np.clip(np.searchsorted(it, a[:, 1]), 0, it.size - 1)
        status = iv[j]
        best = int(np.bincount(iv.astype(int)).argmax()) if False else int(iv.max())
        bad = status != best
    else:
        status = np.zeros(t.size, dtype=int)
        bad = np.zeros(t.size, dtype=bool)
    wide = np.zeros(t.size, dtype=bool)
    for i in np.nonzero(bad)[0]:
        lo_i = np.searchsorted(t, t[i] - mask_widen, side="left")
        hi_i = np.searchsorted(t, t[i] + mask_widen, side="right")
        wide[lo_i:hi_i] = True
    for name, tt in (("gt_clean.tum", t_clean), ("gt_audited.tum", t)):
        with open(os.path.join(out_dir, name), "w") as f:
            for k in range(tt.size):
                f.write("%.6f %.6f %.6f %.6f 0 0 0 1\n" % (tt[k], enu[k, 0], enu[k, 1], enu[k, 2]))
    with open(os.path.join(out_dir, "gt_timebase.yaml"), "w") as f:
        f.write('# MARS-LVIG: the RTK header stamp is on the host clock, aligned to GNSS time to about 0.1 s (survey); `method` says how\n'
                '# applied_shift_s was found (none = not measured, 0)\n'
                'sequence: "%s"\nmethod: "%s"\noffset_s: %.6f\napplied_shift_s: %.6f\nuncertainty_s: 0.05\n'
                'confidence: "%s"\nflag: "%s"\n' % (seq, method, shift_s, applied,
                                                    "medium" if method != "none" else "low",
                                                    "" if method != "none" else "time base not measured"))
    with open(os.path.join(out_dir, "trust_mask.csv"), "w") as f:
        f.write("t,trusted,rtk_info_position,ate_exclude(rtk status != %d, +-%.1f s)\n" % (int(status.max()) if status.size else 0, mask_widen))
        for i in range(t.size):
            f.write("%.6f,%d,%d,%d\n" % (t_clean[i], 0 if wide[i] else 1, int(status[i]), 1 if wide[i] else 0))
    org = (float(lat[0]), float(lon[0]), float(alt[0]))
    audit = {"sequence": seq, "epochs": int(t.size), "t_first": float(t[0]), "t_last": float(t[-1]), "applied_shift_s": applied,
             "rate_hz_median": float(1.0 / np.median(np.diff(t))), "enu_origin_lat_lon_h_rad_rad_m": org,
             "rtk_status_counts": {int(k): int(v) for k, v in zip(*np.unique(status, return_counts=True))},
             "excluded_epochs": int(wide.sum()), "shift_s": shift_s, "method": method,
             "note": "position only, orientation identity; gt_clean = NavSatFix header stamps, gt_audited = + applied_shift_s"}
    with open(os.path.join(out_dir, "gt_audit.json"), "w") as f:
        json.dump(audit, f, indent=1)
        f.write("\n")
    return audit


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("bag")
    p.add_argument("out_dir")
    p.add_argument("--seq", default="HKisland_GNSS03")
    p.add_argument("--shift-s", type=float, default=0.0)
    p.add_argument("--method", default="none")
    a = p.parse_args(argv)
    print(json.dumps(convert(a.bag, a.out_dir, a.seq, a.shift_s, a.method), indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
