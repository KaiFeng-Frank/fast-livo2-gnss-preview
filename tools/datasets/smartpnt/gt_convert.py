#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""SmartPNT-MSF ground truth (NovAtel Inertial Explorer `.ref`) -> the files tools/eval/gtbase.py reads.

Reference point (A80 §5: "estimated poses are moved to the ground-truth reference point with the dataset's lever
arm"): the mini platform's GNSS antenna, phase centre Ant00 of the Septentrio mosaic-X5. Data facts, found in the files
(see dscommon/ie_ref.py): `Mini_GroundTruth.ref` (sequences 0718/0719) and `ROVE_<mini id>_GroundTruth.ref` (0702-0707)
are the same smoothed TC GNSS/INS trajectory transferred to that antenna, `I300_GroundTruth.ref` to the mini's IMU (the
HGuide I300 whose /imu_raw is the host's IMU). The vector I300 -> antenna is the difference of the two files'
`IMU->Secondary Sensor Lever Arms` ((0, -0.120, +0.118) m in the carrier frame x right / y forward / z up), and the data
agree with it to 0.4 mm (std of the rotated position difference) - which also validates the attitude convention.

Outputs (<out>/):
  gt_clean.tum, gt_audited.tum   t x y z qx qy qz qw: UTC Unix seconds (GPST - 18 s, the stamp convention of the slim
                                 bag), ENU metres of the antenna point relative to the first epoch, orientation of the
                                 I300 body frame (x down, y right, z back) in ENU. gt_audited == gt_clean (shift 0)
  gt_i300.tum                    same trajectory at the I300 point (clean + R * (-l_I300))
  gt_timebase.yaml               method gpst, applied_shift_s 0: the GT is stamped in GPST and the slim bag's stamps are
                                 the same instants (IMU stamps == I300 IMR GPST to the microsecond, dscommon/timebase.py)
  trust_mask.csv                 t, then reasons and `ate_exclude(...)`: epochs whose 1-sigma position uncertainty
                                 sqrt(Cx11+Cx22+Cx33) exceeds --max-sigma-3d or whose Q exceeds --max-q, widened by
                                 +-MASK_WIDEN_S (the rule of tools/gt_audit.py)
  gt_mask_intervals.csv          the masked stretches
  gt_quality.csv                 per epoch Q, AmbStatus, sigma_3d, forward/backward separation, speed, satellites
  gt_audit.json                  levers, checks (lever, origin, epochs, gaps, time span)
"""
import argparse
import csv
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import numpy as np  # noqa: E402

from dscommon import ie_ref  # noqa: E402

MASK_WIDEN_S = 2.0
LEVER_TOL_M = 0.002


def mask_epochs(t, sigma3, q, max_sigma3, max_q, widen=MASK_WIDEN_S):
    """-> (exclude bool array, reason bits array): 1 = sigma_3d above the limit, 2 = Q above the limit, each widened by
    +-widen seconds; the bits of the widened mask are the OR of the original epochs inside the window."""
    bits = np.zeros(t.size, dtype=int)
    bits |= np.where(sigma3 > max_sigma3, 1, 0)
    bits |= np.where(q > max_q, 2, 0)
    wide = np.zeros(t.size, dtype=int)
    for i in np.nonzero(bits)[0]:
        lo = np.searchsorted(t, t[i] - widen, side="left")
        hi = np.searchsorted(t, t[i] + widen, side="right")
        wide[lo:hi] |= bits[i]
    return wide != 0, wide


def intervals(t, ex):
    out = []
    i, n = 0, t.size
    while i < n:
        if ex[i]:
            j = i
            while j + 1 < n and ex[j + 1]:
                j += 1
            out.append((float(t[i]), float(t[j])))
            i = j + 1
        else:
            i += 1
    return out


def parse_readme_levers(text):
    """`to ROVE_NN: x y z (label)` lines of the 06_Description/README.xml comment -> {id: ((x, y, z), label)} (lever from the 100C IMU)."""
    import re
    out = {}
    for m in re.finditer(r"to\s+ROVE_(\d+)\s*:\s*(-?[\d.]+)\s+(-?[\d.]+)\s+(-?[\d.]+)\s*\(([^)]*)\)", text):
        out[m.group(1)] = ((float(m.group(2)), float(m.group(3)), float(m.group(4))), m.group(5).strip())
    return out


def readme_check(readme_text, mini_id, secondary_sensor):
    """Does the dataset's own description say that ROVE_<mini id> is the mini's Septentrio antenna Ant00 at the lever the .ref header carries?"""
    lev = parse_readme_levers(readme_text)
    if mini_id not in lev:
        return {"checked": False, "reason": "no `to ROVE_%s:` line in README.xml" % mini_id}
    v, label = lev[mini_id]
    d = float(np.linalg.norm(np.array(v) - np.array(secondary_sensor)))
    return {"checked": True, "label": label, "readme_lever": list(v), "ref_header_lever": list(secondary_sensor), "difference_m": d,
            "label_names_ant00": "Ant00" in label, "ok": d < 1e-3 and "Ant00" in label}


def convert(primary, i300, out_dir, seq, max_sigma3=0.5, max_q=3, readme=None, mini_id=None):
    """Convert one sequence's ground truth. `primary`: the mini-antenna .ref, `i300`: the I300 .ref. Returns the audit dict."""
    os.makedirs(out_dir, exist_ok=True)
    a = ie_ref.read_ref(primary)
    b = ie_ref.read_ref(i300)
    if "WGS84" not in a.header.get("datum", ""):
        raise ValueError("%s: datum %r is not WGS84" % (primary, a.header.get("datum")))
    sec_a, sec_b = a.header.get("secondary_sensor"), b.header.get("secondary_sensor")
    if sec_a is None or sec_b is None:
        raise ValueError("the .ref headers carry no IMU->Secondary Sensor Lever Arms")
    l_rfu = np.array(sec_a) - np.array(sec_b)
    l_i300 = ie_ref.R_BODY_FROM_I300.T @ l_rfu
    mean, std = ie_ref.check_lever(a, b, l_rfu)
    lever_err = float(np.linalg.norm(mean - l_rfu))
    if lever_err > LEVER_TOL_M or float(np.max(std)) > LEVER_TOL_M:
        raise ValueError("lever check failed: data %s (std %s) vs header %s" % (mean, std, l_rfu))
    rows, origin, Rn_i = ie_ref.build_poses(a)
    imu_rows = ie_ref.transfer_point(rows, Rn_i, -l_i300)
    # the I300 file's own positions in the same ENU frame: residual of the transfer
    enu_b, _ = ie_ref.enu_positions(b, origin)
    res = np.linalg.norm(imu_rows[:, 1:4] - enu_b, axis=1)
    t = rows[:, 0]
    sigma3 = ie_ref.sigma_3d(a)
    q = a.col["Q"]
    ex, bits = mask_epochs(t, sigma3, q, max_sigma3, max_q)
    ie_ref.write_tum(os.path.join(out_dir, "gt_clean.tum"), rows)
    ie_ref.write_tum(os.path.join(out_dir, "gt_audited.tum"), rows)
    ie_ref.write_tum(os.path.join(out_dir, "gt_i300.tum"), imu_rows)
    with open(os.path.join(out_dir, "gt_timebase.yaml"), "w") as f:
        f.write("# GT stamp -> slim-bag clock: the .ref epochs are GPST (Week, GPSTime); the slim bag's stamps are UTC Unix = GPST - leap\n"
                "# and the IMU stamps equal the I300 IMR GPST (dscommon/timebase.py), so the GT needs no shift beyond the leap\n"
                "# second already applied to gt_clean.tum\n"
                'sequence: "%s"\nmethod: "gpst"\noffset_s: 0.0\napplied_shift_s: 0.0\nuncertainty_s: 0.001\n'
                'confidence: "high"\nflag: ""\n'
                'rule: "SmartPNT .ref epochs are GPST; sensor stamps are GPST time of day on a wrong date (A80 data tools)"\n' % seq)
    reason_hdr = "reason_bits(1=sigma_3d>%.3f m,2=Q>%d: each within +-%.1f s)" % (max_sigma3, max_q, MASK_WIDEN_S)
    with open(os.path.join(out_dir, "trust_mask.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "trusted", reason_hdr, "ate_exclude(bits 1|2)"])
        for i in range(t.size):
            w.writerow(["%.6f" % t[i], 0 if ex[i] else 1, int(bits[i]), 1 if ex[i] else 0])
    iv = intervals(t, ex)
    with open(os.path.join(out_dir, "gt_mask_intervals.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_from", "t_to", "ate_exclude"])
        for lo, hi in iv:
            w.writerow(["%.6f" % lo, "%.6f" % hi, 1])
    speed = np.hypot(a.col["VEast"], a.col["VNorth"])
    with open(os.path.join(out_dir, "gt_quality.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "Q", "AmbStatus", "sigma_3d_m", "hdng_sd_deg", "e_sep_m", "n_sep_m", "h_sep_m", "speed_h_mps", "n_sats"])
        for i in range(t.size):
            w.writerow(["%.6f" % t[i], int(q[i]), a.col["AmbStatus"][i], "%.4f" % sigma3[i], "%.4f" % a.col["HdngSD"][i],
                        "%.4f" % a.col["E-Sep"][i], "%.4f" % a.col["N-Sep"][i], "%.4f" % a.col["H-Sep"][i],
                        "%.3f" % speed[i], int(a.col["NS"][i])])
    dt = np.diff(t)
    rc = None
    if readme and mini_id and os.path.isfile(readme):
        with open(readme, errors="replace") as f:
            rc = readme_check(f.read(), mini_id, sec_a)
    audit = {
        "readme_check": rc,
        "sequence": seq, "primary": os.path.basename(primary), "i300": os.path.basename(i300),
        "epochs": int(t.size), "t_first_utc": float(t[0]), "t_last_utc": float(t[-1]), "dt_unique_s": sorted(set(np.round(dt, 3).tolist()))[:6],
        "skipped_lines": [a.skipped, b.skipped],
        "lever_i300_to_antenna_carrier_rfu": l_rfu.tolist(), "lever_i300_to_antenna_i300_frame": l_i300.tolist(),
        "lever_check_mean_rfu": mean.tolist(), "lever_check_std_m": std.tolist(),
        "i300_transfer_residual_m": {"max": float(res.max()), "median": float(np.median(res))},
        "enu_origin_lat_lon_h_rad_rad_m": [float(x) for x in origin],
        "quality": {"Q": {int(k): int(v) for k, v in zip(*np.unique(q, return_counts=True))},
                    "amb": {k: a.col["AmbStatus"].count(k) for k in sorted(set(a.col["AmbStatus"]))},
                    "sigma_3d_median_m": float(np.median(sigma3)), "sigma_3d_p95_m": float(np.percentile(sigma3, 95)),
                    "sigma_3d_max_m": float(sigma3.max())},
        "mask": {"max_sigma_3d_m": max_sigma3, "max_q": max_q, "widen_s": MASK_WIDEN_S, "excluded_epochs": int(ex.sum()),
                 "stretches": len(iv)},
        "header": {k: v for k, v in a.header.items()},
        "orientation": "I300 body (x down, y right, z back) -> ENU; R_n_i300 = Rz(-heading) Rx(pitch) Ry(roll) * R_body_from_i300",
    }
    with open(os.path.join(out_dir, "gt_audit.json"), "w") as f:
        json.dump(audit, f, indent=1, sort_keys=True)
        f.write("\n")
    return audit


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("primary", help="Mini_GroundTruth.ref or ROVE_<id>_GroundTruth.ref (mini antenna)")
    p.add_argument("i300", help="I300_GroundTruth.ref")
    p.add_argument("out_dir")
    p.add_argument("--seq", required=True)
    p.add_argument("--max-sigma-3d", type=float, default=0.5)
    p.add_argument("--max-q", type=int, default=3)
    a = p.parse_args(argv)
    audit = convert(a.primary, a.i300, a.out_dir, a.seq, a.max_sigma_3d, a.max_q)
    print(json.dumps({k: audit[k] for k in ("epochs", "lever_i300_to_antenna_i300_frame", "i300_transfer_residual_m", "quality", "mask")},
                     indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
