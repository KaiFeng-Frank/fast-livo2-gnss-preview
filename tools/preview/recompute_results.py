#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Check published research aggregates; optionally rescore a supplied M3DGR GT.

Default: recompute A79's final rigid-fit algebra from paired-point sufficient
statistics, and A81's pooled noise gates from NIS sums and degrees of freedom.
This does not redo timestamp association, GT preprocessing or sensor replay.

--gt: rescore the included Varying-illu03 generated estimate poses against an
independently obtained, unshifted original/clean GT TUM. The GT's normalized
identity must match the recorded experiment. No project metric code is imported.
"""

import argparse
import gzip
import hashlib
import io
import json
import math
from pathlib import Path
import sys

import numpy as np


def read_json(path):
    return json.loads(path.read_text(encoding="utf-8"))


def require(condition, message):
    if not condition:
        raise ValueError(message)


def close(actual, expected, tolerance, label):
    require(math.isfinite(actual), label + ": nonfinite value")
    require(abs(actual - expected) <= tolerance,
            "%s: %.17g differs from %.17g" % (label, actual, expected))


def verify_published_inputs(root, summary):
    for item in summary["historical_provenance"]["base_and_overlay_snapshots"]:
        require(hashlib.sha256((root / item["path"]).read_bytes()).hexdigest() == item["sha256"],
                "Historical configuration snapshot identity mismatch")
    example = read_json(root / "results/a79-varying-illu03/example.json")
    require(example["ground_truth_included"] is False, "Unexpected bundled-GT claim")
    for item in example["generated_estimates"]:
        path = root / item["path"]
        require(hashlib.sha256(path.read_bytes()).hexdigest() == item["gzip_sha256"],
                "Compressed estimate identity mismatch")
        with gzip.open(path, "rb") as stream:
            raw = stream.read()
        require(len(raw) == item["uncompressed_bytes"]
                and hashlib.sha256(raw).hexdigest() == item["uncompressed_sha256"],
                "Uncompressed estimate identity mismatch")


def aggregate_ate(summary, statistics):
    require(statistics["kind"] == "aggregate_scoring_algebra_check",
            "Unexpected A79 statistics kind")
    rows = {row["sequence"]: row for row in summary["sequences"]}
    seen = set()
    largest_delta = 0.0
    for item in statistics["records"]:
        key = (item["sequence"], item["arm"], item["metric"])
        require(key not in seen, "Duplicate A79 statistics record")
        seen.add(key)
        count = item["paired_rows"]
        require(isinstance(count, int) and count > 2, "Invalid paired count")
        cross = np.asarray(item["centered_reference_estimate_product_sum_m2"], dtype=float)
        require(cross.shape == (3, 3) and np.isfinite(cross).all(), "Invalid cross product")
        u, singular, vt = np.linalg.svd(cross)
        require(np.count_nonzero(singular > np.finfo(float).eps) >= 2,
                "Degenerate rigid fit")
        orientation = 1.0 if np.linalg.det(u) * np.linalg.det(vt) >= 0 else -1.0
        residual_sum = (item["centered_estimate_squared_norm_sum_m2"]
                        + item["centered_reference_squared_norm_sum_m2"]
                        - 2 * float(singular[0] + singular[1] + orientation * singular[2]))
        require(residual_sum >= -1e-7, "Negative residual energy")
        rmse = math.sqrt(max(residual_sum, 0.0) / count)
        expected = rows[key[0]][key[1]][key[2] + "_m"]
        close(expected, item["expected_rmse_m"], 1e-14, "Summary/statistics identity")
        close(rmse, expected, 2e-8, "/".join(key))
        largest_delta = max(largest_delta, abs(rmse - expected))
    expected_keys = {(seq, arm, metric) for seq in rows
                     for arm in ("U", "T") for metric in ("e1c", "e1o")}
    require(seen == expected_keys and len(rows) == 7, "Incomplete A79 statistics")
    met = {}
    for metric in ("e1c", "e1o"):
        passing = [row for row in rows.values()
                   if row["T"][metric + "_m"] <= row["paper_fast_livgo_target_m"]]
        require(len(passing) == 2 and all(not r["gnss_activated"] for r in passing),
                "A79 target/activation conclusion changed")
        met[metric] = len(passing)
    active = [row for row in rows.values() if row["gnss_activated"]]
    improved = {metric: sum(row["T"][metric + "_m"] < row["U"][metric + "_m"]
                            for row in active) for metric in ("e1c", "e1o")}
    require(len(active) == 5 and improved == {"e1c": 2, "e1o": 1},
            "A79 improvement conclusion changed")
    return {"verification": "paired-point aggregate algebra only",
            "verified_ate_values": len(seen), "maximum_rmse_delta_m": largest_delta,
            "targets_met": met, "targets_met_with_gnss_active": 0,
            "active_sequences": len(active), "active_sequences_improved": improved}


def aggregate_noise(record):
    require(record["kind"] == "aggregate_gate_check_not_receiver_row_reproduction",
            "Unexpected A81 statistics kind")
    lower, upper = record["d4_closed_interval"]
    require((lower, upper) == (0.8, 1.25), "Registered D4 interval changed")
    checks = []
    for item in record["gates"]:
        require(item["dof_sum"] > 0, "Invalid noise dof")
        factor = item["nis_sum"] / item["dof_sum"]
        passed = lower <= factor <= upper
        close(factor, item["expected_pooled_nis_per_dof"], 1e-12, "Pooled noise factor")
        require(passed == item["expected_d4_pass"], "Noise gate conclusion changed")
        checks.append({"dataset": item["dataset"], "channel": item["channel"],
                       "nis_per_dof": factor, "d4_pass": passed})
    require({(c["dataset"], c["channel"]) for c in checks}
            == {(d, c) for d in ("smartpnt_mini", "mars_lvig") for c in ("dopp", "tdcp")},
            "Incomplete A81 gate table")
    per_sequence = record["smartpnt_independent_doppler_by_sequence"]
    pooled = math.fsum(row["doppler_nis_sum"] for row in per_sequence) / sum(
        row["doppler_dof_sum"] for row in per_sequence)
    smart = next(c for c in checks if c["dataset"] == "smartpnt_mini" and c["channel"] == "dopp")
    close(pooled, smart["nis_per_dof"], 1e-12, "Independent SmartPNT pool")
    require(sum(not c["d4_pass"] for c in checks) == 1 and not smart["d4_pass"],
            "A81 registered stop changed")
    downstream = record["downstream"]
    require(downstream["evaluation_split_sealed"]
            and not any(value for name, value in downstream.items()
                        if name != "evaluation_split_sealed"),
            "A81 downstream state changed")
    return {"verification": "recorded NIS/dof aggregates only", "checks": checks,
            "research_gate": "FAIL (expected recorded outcome)",
            "frozen_configuration_exists": False, "final_evaluation_run": False}


def load_tum(path):
    poses = np.loadtxt(path, comments="#", ndmin=2)
    require(poses.shape[1] == 8 and len(poses) > 2 and np.isfinite(poses).all(),
            "Malformed or nonfinite TUM input")
    require(np.all(np.linalg.norm(poses[:, 4:], axis=1) > 0), "Zero quaternion")
    poses = poses[np.argsort(poses[:, 0], kind="stable")]
    # Match gt_audit.clean_gt: retain the first row at a repeated timestamp.
    keep = np.r_[True, np.diff(poses[:, 0]) != 0]
    return poses[keep]


def quaternion_rotations(q):
    q = q / np.linalg.norm(q, axis=1)[:, None]
    xyz, w = q[:, :3], q[:, 3]
    basis = np.broadcast_to(np.eye(3), (len(q), 3, 3))
    cross = np.cross(xyz[:, None, :], basis)
    images = basis + 2 * w[:, None, None] * cross + 2 * np.cross(xyz[:, None, :], cross)
    return images.transpose(0, 2, 1)


def nearest_pairs(ref_time, est_time, max_diff):
    # Evo's shorter trajectory is scanned; ties choose the first (earlier) row.
    est_longer = len(est_time) > len(ref_time)
    shorter, longer = (ref_time, est_time) if est_longer else (est_time, ref_time)
    right = np.clip(np.searchsorted(longer, shorter), 0, len(longer) - 1)
    left = np.clip(right - 1, 0, len(longer) - 1)
    indices = np.where(abs(longer[left] - shorter) <= abs(longer[right] - shorter), left, right)
    valid = abs(longer[indices] - shorter) <= max_diff
    short_indices = np.flatnonzero(valid)
    return (short_indices, indices[valid]) if est_longer else (indices[valid], short_indices)


def rigid_rmse(x, y):
    xc, yc = x - x.mean(0), y - y.mean(0)
    u, singular, vt = np.linalg.svd(yc.T @ xc)
    require(np.count_nonzero(singular > np.finfo(float).eps) >= 2, "Degenerate rigid fit")
    signs = np.eye(3)
    if np.linalg.det(u) * np.linalg.det(vt) < 0:
        signs[2, 2] = -1
    rotation = u @ signs @ vt
    residual = xc @ rotation.T - yc
    return float(np.sqrt(np.mean(np.sum(residual ** 2, axis=1))))


def external_gt_ate(repo_root, gt_path):
    record = read_json(repo_root / "results/a79-varying-illu03/example.json")
    gt = load_tum(gt_path)
    canonical = "".join(" ".join("%.6f" % v for v in row) + "\n" for row in gt).encode("ascii")
    require(len(gt) == record["clean_ground_truth_rows"]
            and hashlib.sha256(canonical).hexdigest() == record["clean_ground_truth_canonical_sha256"],
            "GT identity mismatch. Supply the original unshifted Varying-illu03 GT TUM; "
            "gt_audited.tum already has a clock shift and is not the required input.")
    # The historical auditor serialized gt_clean.tum to six decimal places
    # before evaluation. Use that same identity-checked numeric input even when
    # the original upstream file has additional precision.
    gt = np.loadtxt(io.StringIO(canonical.decode("ascii")), ndmin=2)
    gt[:, 0] += record["timebase"]["applied_shift_s"]
    start, end = record["scoring_span_estimator_time_s"]
    gt = gt[(gt[:, 0] >= start) & (gt[:, 0] <= end)]
    require(record["excluded_intervals_estimator_time_s"] == [], "Unexpected example mask")
    lever = np.asarray(record["fixed_gt_antenna_lever_imu_m"])
    output = {}
    for item in record["generated_estimates"]:
        path = repo_root / item["path"]
        est = load_tum(path)
        ir, ie = nearest_pairs(gt[:, 0], est[:, 0], record["association_max_diff_s"])
        require(len(ir) > 2, "Insufficient GT/estimate pairs")
        y, x = gt[ir, 1:4], est[ie, 1:4]
        lifted = x + np.einsum("nij,j->ni", quaternion_rotations(est[ie, 4:]), lever)
        metrics = {"e1o_m": rigid_rmse(x, y), "e1c_m": rigid_rmse(lifted, y)}
        for name, value in metrics.items():
            close(value, record["expected_metrics_m"][item["arm"]][name],
                  record["numeric_tolerance_m"], item["arm"] + "/" + name)
        output[item["arm"]] = {**metrics, "paired_rows": len(ir)}
    return {"verification": "full scoring of archived estimate poses against identity-checked external GT",
            "sequence": record["sequence"], "sensor_replay_performed": False, "metrics": output}


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--gt", type=Path, help="Original, unshifted M3DGR Varying-illu03 GT TUM")
    parser.add_argument("--json", action="store_true", help="Print a machine-readable verification result")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    summary = read_json(root / "results/a79_summary.json")
    verify_published_inputs(root, summary)
    result = {"a79": aggregate_ate(summary, read_json(root / "results/a79_se3_statistics.json")),
              "a81": aggregate_noise(read_json(root / "results/a81_noise_recheck.json"))}
    if args.gt:
        result["external_gt"] = external_gt_ate(root, args.gt)
    if args.json:
        print(json.dumps(result, indent=2))
    else:
        print("A79: 28 aggregate ATE values checked; 2/7 paper targets met, both GNSS inactive.")
        print("A79 active sequences: e1c improves 2/5; e1o improves 1/5.")
        smart = next(c for c in result["a81"]["checks"]
                     if c["dataset"] == "smartpnt_mini" and c["channel"] == "dopp")
        print("A81: SmartPNT Doppler %.12f > 1.25; recorded D4 rejection reproduced." % smart["nis_per_dof"])
        if "external_gt" in result:
            for arm, metrics in result["external_gt"]["metrics"].items():
                print("Varying-illu03 %s: e1c %.9f m, e1o %.9f m (%d pairs)." %
                      (arm, metrics["e1c_m"], metrics["e1o_m"], metrics["paired_rows"]))
        else:
            print("Aggregate verification passed. Full trajectory scoring requires --gt; no GT is bundled.")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (ValueError, OSError, KeyError) as exc:
        print("Verification failed: " + str(exc), file=sys.stderr)
        sys.exit(1)
