#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Recompute synthetic antenna position ATE from exported, timestamp-matched samples."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


def aligned_rmse(est, truth):
    """Proper rigid least-squares alignment, with no scale estimation."""
    x = est - est.mean(axis=0)
    y = truth - truth.mean(axis=0)
    u, _, vt = np.linalg.svd(y.T @ x / len(x))
    reflection = np.eye(3)
    reflection[-1, -1] = np.linalg.det(u @ vt)
    rotation = u @ reflection @ vt
    return float(np.sqrt(np.mean(np.sum((x @ rotation.T - y) ** 2, axis=1))))


def score(directory):
    summary = json.loads((directory / "summary.json").read_text())
    truth = np.loadtxt(directory / "truth.tum", ndmin=2)
    if len(truth) < 3 or not np.isfinite(truth).all():
        raise ValueError("need at least three finite reference samples")
    if len(truth) != summary["samples"]:
        raise ValueError("sample count disagrees with summary")
    if not summary["activated"] or min(summary["accepted_rows"].values()) <= 0:
        raise ValueError("GNSS activation or accepted measurement rows missing")
    report = {
        "schema": "fastlivo.gnss.preview.score.v1",
        "synthetic_only": True,
        "metric": "antenna position ATE RMSE; SE(3) rigid alignment; no scale",
        "sampling": "same timestamps, from activation + 5 s, 1 Hz",
        "samples": len(truth),
        "activation_s": summary["activation_s"],
        "accepted_rows": summary["accepted_rows"],
        "ate_rmse_m": {},
        "input_sha256": {},
    }
    for name in ("host", "global", "odometry"):
        trajectory = np.loadtxt(directory / (name + ".tum"), ndmin=2)
        if trajectory.shape != truth.shape or not np.array_equal(trajectory[:, 0], truth[:, 0]):
            raise ValueError(f"{name}: sample timestamps differ from reference")
        if not np.isfinite(trajectory).all():
            raise ValueError(f"{name}: non-finite trajectory")
        value = aligned_rmse(trajectory[:, 1:4], truth[:, 1:4])
        if not np.isclose(value, summary["ate_rmse_m"][name], rtol=1e-7, atol=1e-8):
            raise ValueError(f"{name}: NumPy and C++ ATE disagree")
        report["ate_rmse_m"][name] = value
    for name in ("synthetic.nav", "config.txt", "truth.tum", "host.tum", "global.tum", "odometry.tum", "summary.json"):
        report["input_sha256"][name] = hashlib.sha256((directory / name).read_bytes()).hexdigest()
    (directory / "score.json").write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    print(f"Synthetic GNSS activated at {summary['activation_s']:.1f} s; {len(truth)} paired samples")
    for name, value in report["ate_rmse_m"].items():
        print(f"  {name} antenna ATE, SE(3), no scale: {value:.6f} m")
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    score(parser.parse_args().directory)
