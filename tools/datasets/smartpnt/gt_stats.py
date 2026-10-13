#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Ground-truth-only statistics of a SmartPNT sequence: coverage of the bag, quality (Q, AmbStatus, sigma_3d), the A80 trust
mask, path length, speed regime and extent. Nothing but the converted ground-truth directory and the bag's build record is
read: no estimator, no GNSS solution, so every class of sequence may run it (AMENDMENTS A2).

  gt_stats.py <seq> [<seq> ...] [--root /data2/fl2g] [--out stats.json]

Reads <root>/smartpnt/<seq>/{build.json, gt/gt_clean.tum, gt/gt_quality.csv, gt/trust_mask.csv, gt/gt_audit.json}.
numpy only.
"""
import argparse
import csv
import json
import math
import os
import sys

import numpy as np

STATIC_MPS = 0.3          # horizontal speed under which the platform counts as standing still


def pct(a, q):
    return float(np.percentile(a, q)) if len(a) else None


def runs_of(flags):
    """Lengths of the maximal runs of True in a boolean sequence."""
    out, cur = [], 0
    for f in flags:
        if f:
            cur += 1
        elif cur:
            out.append(cur)
            cur = 0
    if cur:
        out.append(cur)
    return out


def analyse(gt, build, audit):
    """gt: dict of arrays t [s], p (n,3) ENU antenna positions [m], Q (int), amb (list of str), sigma (m), speed (m/s), nsat,
    excluded (bool: the trust mask's ate_exclude); build: the parsed build.json; audit: the parsed gt_audit.json."""
    t, p = gt["t"], gt["p"]
    excl = gt["excluded"]
    trusted = ~excl
    dt = np.diff(t)
    step_h = np.hypot(np.diff(p[:, 0]), np.diff(p[:, 1]))
    step_3 = np.linalg.norm(np.diff(p, axis=0), axis=1)
    both = trusted[1:] & trusted[:-1]
    b0, b1 = build["bag"]["start_ns"] / 1e9, build["bag"]["end_ns"] / 1e9
    spd, sig, q = gt["speed"], gt["sigma"], gt["Q"]
    mask_runs = runs_of(excl)
    trusted_runs = runs_of(trusted)
    return {
        "bag_start_utc": build["topics"]["/imu_raw"]["first_utc"], "bag_duration_s": build["bag"]["duration_s"],
        "gt_first_s": float(t[0]), "gt_last_s": float(t[-1]),
        "gt_start_after_bag_start_s": float(t[0] - b0), "gt_end_before_bag_end_s": float(b1 - t[-1]),
        "gt_span_s": float(t[-1] - t[0]), "gt_span_over_bag": float((t[-1] - t[0]) / (b1 - b0)),
        "epochs": int(len(t)), "dt_unique_s": sorted(set(np.round(dt, 3).tolist())),
        "Q": {int(k): int((q == k).sum()) for k in sorted(set(q.tolist()))},
        "amb": {k: gt["amb"].count(k) for k in sorted(set(gt["amb"]))},
        "sigma3d_all_m": {"median": pct(sig, 50), "p95": pct(sig, 95), "max": float(sig.max())},
        "sigma3d_trusted_m": {"median": pct(sig[trusted], 50), "p95": pct(sig[trusted], 95), "max": float(sig[trusted].max())}
        if trusted.any() else None,
        "Q_trusted": {int(k): int((q[trusted] == k).sum()) for k in sorted(set(q[trusted].tolist()))},
        "fixed_fraction_trusted": float(np.mean([a == "Fixed" for a, tr in zip(gt["amb"], trusted) if tr])) if trusted.any() else None,
        "masked_epochs": int(excl.sum()), "masked_fraction": float(excl.mean()), "mask_stretches": len(mask_runs),
        "mask_stretch_len_s": sorted(mask_runs, reverse=True),
        "trusted_epochs": int(trusted.sum()), "trusted_runs": len(trusted_runs),
        "trusted_longest_run_s": int(max(trusted_runs)) if trusted_runs else 0,
        "path_horizontal_m": float(step_h.sum()), "path_3d_m": float(step_3.sum()),
        "path_horizontal_trusted_m": float(step_h[both].sum()),
        "speed_h_mps": {"median": pct(spd, 50), "p5": pct(spd, 5), "p95": pct(spd, 95), "max": float(spd.max()),
                        "mean_moving": float(spd[spd > STATIC_MPS].mean()) if (spd > STATIC_MPS).any() else None,
                        "fraction_standing": float(np.mean(spd < STATIC_MPS)),
                        "fraction_above_10": float(np.mean(spd > 10.0)), "fraction_above_15": float(np.mean(spd > 15.0))},
        "extent_m": {"east": [float(p[:, 0].min()), float(p[:, 0].max())], "north": [float(p[:, 1].min()), float(p[:, 1].max())],
                     "up": [float(p[:, 2].min()), float(p[:, 2].max())],
                     "max_dist_from_start": float(np.hypot(p[:, 0], p[:, 1]).max()),
                     "end_to_start_horizontal": float(math.hypot(p[-1, 0], p[-1, 1]))},
        "gt_n_sats": {"median": pct(gt["nsat"], 50), "p10": pct(gt["nsat"], 10), "min": float(gt["nsat"].min())},
        "reference": {"file": audit.get("primary"), "label": audit.get("readme_check", {}).get("label"),
                      "program": audit.get("header", {}).get("program"), "process_info": audit.get("header", {}).get("process_info"),
                      "source": audit.get("header", {}).get("source"), "solfile": audit.get("header", {}).get("solfile")},
    }


def load(gt_dir):
    tum = np.loadtxt(os.path.join(gt_dir, "gt_clean.tum"))
    with open(os.path.join(gt_dir, "gt_quality.csv")) as f:
        rows = list(csv.DictReader(f))
    with open(os.path.join(gt_dir, "trust_mask.csv")) as f:
        mrows = list(csv.reader(f))[1:]
    t = tum[:, 0]
    tq = np.array([float(r["t"]) for r in rows])
    if len(rows) != len(t) or not np.allclose(tq, t, atol=1e-3):
        raise SystemExit("gt_quality.csv does not match the epochs of gt_clean.tum")
    if len(mrows) != len(t):
        raise SystemExit("trust_mask.csv does not match the epochs of gt_clean.tum")
    return {"t": t, "p": tum[:, 1:4], "Q": np.array([int(r["Q"]) for r in rows]), "amb": [r["AmbStatus"] for r in rows],
            "sigma": np.array([float(r["sigma_3d_m"]) for r in rows]), "speed": np.array([float(r["speed_h_mps"]) for r in rows]),
            "nsat": np.array([float(r["n_sats"]) for r in rows]), "excluded": np.array([int(r[3]) != 0 for r in mrows])}


def run(root, seq):
    d = os.path.join(root, "smartpnt", seq)
    with open(os.path.join(d, "build.json")) as f:
        build = json.load(f)
    with open(os.path.join(d, "gt", "gt_audit.json")) as f:
        audit = json.load(f)
    out = {"sequence": seq}
    out.update(analyse(load(os.path.join(d, "gt")), build, audit))
    return out


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("seq", nargs="+")
    p.add_argument("--root", default="/data2/fl2g")
    p.add_argument("--out", default=None)
    a = p.parse_args(argv)
    txt = json.dumps({s: run(a.root, s) for s in a.seq}, indent=1)
    if a.out:
        with open(a.out, "w") as f:
            f.write(txt + "\n")
    print(txt)
    return 0


if __name__ == "__main__":
    sys.exit(main())
