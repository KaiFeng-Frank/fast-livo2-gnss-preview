#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""E5: GNSS-denial start/end error, the M3DGR `ArUco_evaluate.py` definition (TEST_PLAN E5, A56).

    eval_aruco.py --matrix runs/eval/<name> [--matrix ...] [--out DIR] [--seqs GNSS-denial01,GNSS-denial02]
                  [--gt SEQ=path] [--evaluation-phase]
    eval_aruco.py --reference GTGNSS-denial01.txt (--traj FILE | --traj-dir DIR) --out DIR [--seq NAME]

The official script (sjtuyinjie/M3DGR `ArUco_evaluate.py`, fetched from the repository; it is not in this tree)
does not run evo's APE. evo is only its TUM reader. The metric, copied from that script, is:

  * reference: rotation = file lines 4–6 (3×3), translation = lines 8–10 (one number each), bag time = the last
    line split on ':' with the 's' removed. On the M3DGR GNSS-denial files this is the ArUco start-to-end
    relative pose (three leading blank lines, a blank line, three translation lines, a blank line, `bag_time`).
  * estimate: T_rel = inv(T_first) @ T_end. T_end is not always the last row. Walking backward from the
    penultimate pose, the end index moves to each pose that `np.allclose`s the pose at the current end index
    and stops at the first that does not — the first pose of the trailing frozen run (a trajectory that stops
    updating). Poses are 4×4, file order, not time-sorted and not stamp-collapsed (unlike eval_ate.py).
  * translation error (m) = ||t_ref − t_rel||₂. This is the number the M3DGR README says the paper uses.
  * rotation error = ||R_ref − R_rel||_F (Frobenius). The official column header says "Rotation error(m)";
    the quantity is dimensionless. It is not a geodesic angle.
  * RMSE = sqrt((rotation_error² + translation_error²) / 2). Metres and a dimensionless number are mixed,
    as in the official script.
  * tracking rate (%) = min(100, 100 · (t_end − t_start) / bag_time), with those two stamps. There is no
    temporal association and no SE(3) alignment. A trajectory whose stamps do not overlap [0, bag_time] is
    still scored; bag_time only scales the tracking rate.

TEST_PLAN E5 asks for both outputs. Matrix mode scores `traj_W.tum` and `traj_N.tum` as the official script
would score each file on its own (raw files: the E1 activation stitch of e1traj.py is not applied). U has no
`traj_N.tum`; that output is reported absent and does not drop the W score.

Differences from the official script (the arithmetic above is the same):
  * no evo / colorama / tabulate import. A TUM row (t x y z qx qy qz qw) becomes an SE(3) matrix the way
    evo 1.31.1 does: roll qx qy qz qw so w is first, then `transformations.quaternion_matrix` (Christoph
    Gohlke, the copy evo vendors), translation written into that matrix. The test compares this with evo
    itself when evo imports.
  * the official pre-check rejects a file if any line does not split into 8 tokens (so a '#' comment or a
    blank line rejects the file). evo's own reader would skip '#'. This tool follows the official pre-check.
    Whitespace is collapsed by `split()`, which is the pre-check. evo's csv reader splits on every space, so
    a double space can make evo fail a file this pre-check accepts; replay TUM files are single-spaced.
  * full precision in JSON/CSV. The official table prints two decimals and sorts on those rounded strings.
    `--sort` for `--traj-dir` sorts on the full values (translation, unless --sort rotation|rmse).
  * matrix mode reads only traj_W.tum and traj_N.tum. `--traj-dir` walks a directory for `.txt` and `.tum`,
    as the official script does, so other TUM files in that tree are scored too.
  * A2: a registered evaluation sequence is scored only with `--evaluation-phase`, and nothing of it is read
    before that check. `--reference` without `--seq` is a calculator (the unit tests); `--seq` applies A2.

Output (--out, default <first matrix>/eval): aruco.json, aruco_runs.csv, aruco_cells.csv. make_table.py
`--aruco` renders the A56 separate table from aruco.json; dev_report.py `--aruco` appends the same numbers.
"""
import argparse
import csv
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402

import evalcommon as ec  # noqa: E402

# evo 1.31.1 evo/core/transformations.py (Christoph Gohlke, public domain): quaternion_matrix's epsilon.
_EPS = float(np.finfo(float).eps * 4.0)
OUTPUTS = (("W", "traj_W.tum"), ("N", "traj_N.tum"))
METRIC_KEYS = ("translation_m", "rotation_frobenius", "rmse", "tracking_rate_pct")


def quaternion_matrix(quaternion):
    """4×4 rotation from a (w, x, y, z) quaternion. evo's `transformations.quaternion_matrix`."""
    q = np.array(quaternion[:4], dtype=np.float64, copy=True)
    nq = float(np.dot(q, q))
    if nq < _EPS:
        return np.identity(4)
    q *= math.sqrt(2.0 / nq)
    q = np.outer(q, q)
    return np.array((
        (1.0 - q[2, 2] - q[3, 3], q[1, 2] - q[3, 0], q[1, 3] + q[2, 0], 0.0),
        (q[1, 2] + q[3, 0], 1.0 - q[1, 1] - q[3, 3], q[2, 3] - q[1, 0], 0.0),
        (q[1, 3] - q[2, 0], q[2, 3] + q[1, 0], 1.0 - q[1, 1] - q[2, 2], 0.0),
        (0.0, 0.0, 0.0, 1.0),
    ), dtype=np.float64)


def pose_from_tum_row(row):
    """One TUM row -> 4×4, evo 1.31.1 `xyz_quat_wxyz_to_se3_poses` for that row (wxyz after np.roll)."""
    xyz = np.asarray(row[1:4], dtype=np.float64)
    quat_wxyz = np.roll(np.asarray(row[4:8], dtype=np.float64), 1)
    pose = quaternion_matrix(quat_wxyz)
    pose[:3, 3] = xyz
    return pose


def check_tum_format(path):
    """Official `check_tum_format`: every line, blank ones included, must split into exactly 8 tokens.

    An empty file returns True there (the loop never runs) and the reader then fails. Same here: (True, "").
    """
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        for i, line in enumerate(f, 1):
            if len(line.strip().split()) != 8:
                return False, "%s:%d does not have 8 entries" % (path, i)
    return True, ""


def read_tum_poses(path):
    """-> (list of 4×4, stamps ndarray) in file order, or raise ValueError (official skip reasons)."""
    ok, why = check_tum_format(path)
    if not ok:
        raise ValueError("invalid TUM (official check_tum_format): %s" % why)
    rows = []
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            tok = line.strip().split()
            if tok:
                rows.append([float(x) for x in tok])
    if not rows:
        raise ValueError("no poses")
    poses = [pose_from_tum_row(r) for r in rows]
    stamps = np.array([r[0] for r in rows], dtype=float)
    return poses, stamps


def read_reference_transform(path):
    """Official `read_reference_transform`. -> (R 3×3, t 3, bag_time_s). Raises ec.Blocked if the layout is short.

    Line numbers are the official 0-based indexes: rotation lines[3:6], translation lines[7:10] as three
    scalar lines, bag time = last line. A missing rotation block, translation block or bag time is the
    'start/end marker missing' failure (the file is not two poses; it is one relative pose plus a duration).
    """
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        lines = f.readlines()
    try:
        rotation = np.array([[float(v) for v in lines[3].split()],
                             [float(v) for v in lines[4].split()],
                             [float(v) for v in lines[5].split()]], dtype=float)
        translation = np.array([float(lines[7].strip()), float(lines[8].strip()), float(lines[9].strip())], dtype=float)
        bag = float(lines[-1].strip().split(":")[1].replace("s", "").strip())
    except (IndexError, ValueError) as e:
        raise ec.Blocked("%s: ArUco reference is missing the rotation block (lines 4-6), the translation block "
                         "(lines 8-10) or the bag time (last line): %s" % (path, e))
    if rotation.shape != (3, 3):
        raise ec.Blocked("%s: rotation block is not 3x3 (got %s)" % (path, rotation.shape))
    return rotation, translation, float(bag)


def reference_record(path, rotation, translation, bag):
    tr = float(np.trace(rotation))
    angle = float(np.degrees(np.arccos(float(np.clip((tr - 1.0) / 2.0, -1.0, 1.0)))))
    return {"path": path, "sha256": ec.sha256_file(path), "rotation": rotation.tolist(),
            "translation_m": [float(x) for x in translation], "bag_time_s": bag,
            "translation_norm_m": float(np.linalg.norm(translation)), "rotation_angle_deg": angle,
            "rotation_det": float(np.linalg.det(rotation))}


def trailing_endpoint_index(poses):
    """Official end index: first pose of the trailing run of `np.allclose`-identical 4×4 poses."""
    last = len(poses) - 1
    for i in range(len(poses) - 2, -1, -1):
        if not np.allclose(poses[i], poses[last]):
            break
        last = i
    return last


def score_poses(rotation, translation, bag_time_s, poses, stamps):
    """Official arithmetic on poses the caller already built. Raises ValueError if there is no pose or bag time is 0.

    One pose is not an error: the official loop leaves the end index at 0, so the relative pose is the identity
    and the tracking rate is 0. Callers that want a distinct endpoint inspect `n_poses`.
    """
    if len(poses) == 0:
        raise ValueError("no poses")
    if not bag_time_s:
        raise ValueError("bag time is 0")
    last = trailing_endpoint_index(poses)
    relative = np.linalg.inv(poses[0]) @ poses[last]
    est_r, est_t = relative[:3, :3], relative[:3, 3]
    rotation_error = float(np.linalg.norm(rotation - est_r))
    translation_error = float(np.linalg.norm(translation - est_t))
    rmse = float(np.sqrt((rotation_error ** 2 + translation_error ** 2) / 2.0))
    duration = float(stamps[last] - stamps[0])
    rate = (duration / float(bag_time_s)) * 100.0
    if rate > 100.0:
        rate = 100.0
    return {"translation_error_m": translation_error, "rotation_error": rotation_error, "rmse": rmse,
            "tracking_rate_pct": float(rate), "eval_duration_s": duration, "end_index": int(last),
            "n_poses": int(len(poses)), "t_start": float(stamps[0]), "t_end": float(stamps[last]),
            "single_pose": len(poses) == 1}


def score_file(rotation, translation, bag_time_s, path):
    poses, stamps = read_tum_poses(path)
    return score_poses(rotation, translation, bag_time_s, poses, stamps)


def _stat(vals):
    if not vals:
        return {"n_ok": 0, "median": None, "min": None, "max": None}
    a = np.asarray(vals, dtype=float)
    return {"n_ok": int(a.size), "median": float(np.median(a)), "min": float(np.min(a)), "max": float(np.max(a))}


def locate_gt(root, seq, info, override=None):
    """Copied GT (`data/m3dgr/gt/<basename>`, fetch_eval.py) or the raw share path. Raises ec.Blocked."""
    if info.get("gt_kind") != "aruco":
        raise ec.Blocked("%s: gt_kind is %s, not aruco (E5 does not score an RTK trajectory)" % (seq, info.get("gt_kind")))
    if override:
        if not os.path.isfile(override):
            raise ec.Blocked("%s: GT override not found: %s" % (seq, override))
        return override
    reg, _ = ec.load_sequences()
    name = os.path.basename(info["gt"])
    candidates = [os.path.join(root, "data", "m3dgr", "gt", name),
                  os.path.join(root, reg["gt_raw_root"], info["gt"])]
    for p in candidates:
        if os.path.isfile(p):
            return p
    raise ec.Blocked("%s: ArUco GT not found (tried %s)" % (seq, ", ".join(candidates)))


def _metric_doc():
    return {"name": "E5", "source": "M3DGR ArUco_evaluate.py",
            "translation_error_m": "||t_ref - t_rel||_2, T_rel = inv(T_first) @ T_end",
            "rotation_error": "Frobenius ||R_ref - R_rel|| (official header says metres; the quantity is dimensionless)",
            "rmse": "sqrt((rotation_error**2 + translation_error**2) / 2)",
            "tracking_rate_pct": "min(100, 100 * (t_end - t_start) / bag_time); no temporal association",
            "end_pose": "first pose of the trailing np.allclose-identical run",
            "frames": "matrix mode scores raw traj_W.tum and traj_N.tum; no E1 stitch, no Umeyama"}


def _empty_result(mode):
    return {"tool": "tools/eval/eval_aruco.py", "generated_utc": ec.utc_now(), "argv": sys.argv,
            "metric": _metric_doc(), "mode": mode, "matrices": [], "sequences": {}, "runs": [], "cells": [],
            "blocked": {}}


def _cells_from_runs(runs):
    """Group ok runs by (seq, arm, output). Absent/invalid outputs stay as cells with n_ok 0 and a note."""
    groups = {}
    for r in runs:
        if r.get("output") is None:
            continue
        groups.setdefault((r["seq"], r["arm"], r["output"]), []).append(r)
    cells = []
    arm_order = list(ec.load_arms())
    _, reg = ec.load_sequences()
    seq_order = list(reg)

    def key(item):
        seq, arm, output = item
        return (seq_order.index(seq) if seq in seq_order else 999, arm_order.index(arm) if arm in arm_order else 999,
                0 if output == "W" else 1, output)
    for (seq, arm, output), rs in sorted(groups.items(), key=lambda kv: key(kv[0])):
        ok = [r for r in rs if r.get("status") == "ok"]
        notes = []
        if not ok:
            if all(r.get("status") == "absent" for r in rs):
                notes.append("无 %s" % rs[0].get("file", "trajectory"))
            elif rs:
                notes.append(rs[0].get("status") or "无有效运行")
        cell = {"seq": seq, "arm": arm, "output": output, "file": rs[0].get("file"), "n_runs": len(rs), "n_ok": len(ok),
                "note": notes[0] if notes else None,
                "translation_m": _stat([r["translation_error_m"] for r in ok]),
                "rotation_frobenius": _stat([r["rotation_error"] for r in ok]),
                "rmse": _stat([r["rmse"] for r in ok]),
                "tracking_rate_pct": _stat([r["tracking_rate_pct"] for r in ok])}
        cells.append(cell)
    return cells


def _score_one(rotation, translation, bag, path, base):
    row = dict(base)
    row["file_path"] = path
    try:
        row.update(score_file(rotation, translation, bag, path))
        row["status"] = "ok"
        if row["single_pose"]:
            row["note"] = "single pose: relative transform is the identity and the tracking rate is 0 (official loop)"
    except (OSError, ValueError) as e:
        row["status"] = str(e)
    return row


def iter_traj_dir(directory):
    """Official discovery: os.walk, files ending in .txt or .tum."""
    found = []
    for root, _, files in os.walk(directory):
        for name in files:
            if name.endswith(".txt") or name.endswith(".tum"):
                found.append(os.path.join(root, name))
    return found


def evaluate_files(reference, trajs, out_dir, seq=None, evaluation_phase=False):
    if seq:
        ec.check_phase(seq, evaluation_phase, "scoring")
    rotation, translation, bag = read_reference_transform(reference)
    result = _empty_result("files")
    result["sequences"][seq or os.path.basename(reference)] = reference_record(reference, rotation, translation, bag)
    runs = []
    for path in trajs:
        base = {"seq": seq or os.path.basename(reference), "arm": "file", "seed": None, "output": "file",
                "file": os.path.basename(path), "matrix": None, "run_dir": None}
        if not os.path.isfile(path):
            runs.append(dict(base, status="not found: %s" % path, file_path=path))
            continue
        runs.append(_score_one(rotation, translation, bag, path, base))
    if len(runs) > 1:
        runs.sort(key=lambda r: (r.get("translation_error_m") is None, r.get("translation_error_m") or 0.0))
    result["runs"] = runs
    result["cells"] = _cells_from_runs(runs) if seq else []
    _write(out_dir, result)
    return result


def evaluate_matrix(args):
    root = ec.project_root(args.root)
    _, reg = ec.load_sequences()
    import eval_ate  # local: same run discovery as E1, not imported at module load
    runs, _, provenance = eval_ate.collect_runs(root, args.matrix, set(ec.split_list(args.arms)) if args.arms else None,
                                                 set(ec.split_list(args.seqs)) if args.seqs else None)
    out_dir = args.out or os.path.join(root if not os.path.isabs(args.matrix[0]) else "", args.matrix[0], "eval")
    result = _empty_result("matrix")
    result["root"] = root
    result["matrices"] = provenance
    overrides = dict(x.split("=", 1) for x in args.gt)
    seqs = sorted({r["seq"] for r in runs}, key=lambda s: list(reg).index(s) if s in reg else 999)
    scored = []
    refused = []
    refs = {}
    for seq in seqs:
        info = reg.get(seq)
        try:
            ec.check_phase(seq, args.evaluation_phase, "scoring")
        except ec.PhaseError as e:
            refused.append(str(e))
            result["blocked"][seq] = str(e)
            continue
        if info is None:
            result["blocked"][seq] = "%s: not in tools/eval/sequences.json" % seq
            continue
        try:
            path = locate_gt(root, seq, info, overrides.get(seq))
            rotation, translation, bag = read_reference_transform(path)
        except ec.Blocked as e:
            result["blocked"][seq] = str(e)
            continue
        refs[seq] = (rotation, translation, bag)
        result["sequences"][seq] = reference_record(path, rotation, translation, bag)
    for r in runs:
        if r["seq"] not in refs:
            r["status"] = "not scored: %s" % result["blocked"].get(r["seq"], "?")
            r["output"] = None
            scored.append(r)
            continue
        rotation, translation, bag = refs[r["seq"]]
        if not r["run_dir"]:
            for output, fname in OUTPUTS:
                scored.append({"matrix": r["matrix"], "seq": r["seq"], "arm": r["arm"], "seed": r["seed"],
                               "state": r["state"], "run_dir": None, "output": output, "file": fname,
                               "status": "no complete run (%s)" % r["state"]})
            continue
        rd = os.path.join(root, r["run_dir"])
        for output, fname in OUTPUTS:
            path = os.path.join(rd, fname)
            base = {"matrix": r["matrix"], "seq": r["seq"], "arm": r["arm"], "seed": r["seed"], "state": r["state"],
                    "run_dir": r["run_dir"], "output": output, "file": fname, "build_sha256": r.get("build_sha256")}
            if not os.path.isfile(path) or os.path.getsize(path) == 0:
                scored.append(dict(base, status="absent"))
                continue
            scored.append(_score_one(rotation, translation, bag, path, base))
    result["runs"] = scored
    result["cells"] = _cells_from_runs(scored)
    _write(out_dir, result)
    _report(result, out_dir)
    return 3 if refused else 0


def _f(x, fmt="%.9f"):
    return fmt % x if isinstance(x, (int, float)) and x is not None else ""


def _write(out_dir, result):
    os.makedirs(out_dir, exist_ok=True)
    ec.atomic_write_json(os.path.join(out_dir, "aruco.json"), result)
    with open(os.path.join(out_dir, "aruco_runs.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["seq", "arm", "seed", "output", "file", "matrix", "run_dir", "status", "translation_error_m",
                    "rotation_error", "rmse", "tracking_rate_pct", "eval_duration_s", "end_index", "n_poses",
                    "t_start", "t_end", "single_pose"])
        for r in result["runs"]:
            w.writerow([r.get("seq"), r.get("arm"), "" if r.get("seed") is None else r.get("seed"), r.get("output"),
                        r.get("file"), r.get("matrix") or "", r.get("run_dir") or "", r.get("status"),
                        _f(r.get("translation_error_m")), _f(r.get("rotation_error")), _f(r.get("rmse")),
                        _f(r.get("tracking_rate_pct")), _f(r.get("eval_duration_s")),
                        "" if r.get("end_index") is None else r.get("end_index"), r.get("n_poses") or "",
                        _f(r.get("t_start")), _f(r.get("t_end")),
                        "" if "single_pose" not in r else int(bool(r.get("single_pose")))])
    with open(os.path.join(out_dir, "aruco_cells.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["seq", "arm", "output", "n_ok", "n_runs", "note",
                    "translation_median_m", "translation_min_m", "translation_max_m",
                    "rotation_median", "rotation_min", "rotation_max",
                    "rmse_median", "rmse_min", "rmse_max",
                    "tracking_median_pct", "tracking_min_pct", "tracking_max_pct"])
        for c in result["cells"]:
            def g(block, name):
                return _f((c.get(block) or {}).get(name))
            w.writerow([c["seq"], c["arm"], c["output"], c["n_ok"], c["n_runs"], c.get("note") or "",
                        g("translation_m", "median"), g("translation_m", "min"), g("translation_m", "max"),
                        g("rotation_frobenius", "median"), g("rotation_frobenius", "min"), g("rotation_frobenius", "max"),
                        g("rmse", "median"), g("rmse", "min"), g("rmse", "max"),
                        g("tracking_rate_pct", "median"), g("tracking_rate_pct", "min"), g("tracking_rate_pct", "max")])


def _report(result, out_dir):
    n_ok = sum(1 for r in result["runs"] if r.get("status") == "ok")
    print("eval_aruco: %d runs, %d ok, %d cells -> %s" % (len(result["runs"]), n_ok, len(result["cells"]), out_dir))
    for seq, why in result["blocked"].items():
        print("eval_aruco: %s not scored: %s" % (seq, why))


def fmt_block(block, digits):
    if not block or block.get("median") is None:
        return "—"
    return "%s [%s, %s]" % (ec.fmt_m(block["median"], digits), ec.fmt_m(block["min"], digits), ec.fmt_m(block["max"], digits))


def fmt_cell_translation(cell, digits):
    """Translation cell of the separate table. None cell -> the caller says 待评测."""
    if not cell or not cell.get("n_ok"):
        return (cell or {}).get("note") or "无有效运行(0/%d)" % ((cell or {}).get("n_runs") or 0)
    s = fmt_block(cell["translation_m"], digits)
    if cell["n_ok"] < cell.get("n_runs", cell["n_ok"]):
        s += " (n=%d/%d)" % (cell["n_ok"], cell["n_runs"])
    return s


def markdown_lines(result, digits=2):
    """Body of the A56 separate table (no heading). make_table and dev_report both use this."""
    cells = {(c["seq"], c["arm"], c["output"]): c for c in result.get("cells") or []}
    blocked = result.get("blocked") or {}
    lines = ["平移误差 ‖t_gt − t_rel‖₂(m),旋转误差 ‖R_gt − R_rel‖_F(无量纲;官方表头写作 Rotation error(m)),"
             "RMSE = sqrt((旋转² + 平移²) / 2),跟踪率 = min(100, 100·(t_end − t_start) / bag_time)。"
             "T_rel = inv(T_first) @ T_end,终点是末尾连续相同位姿段的第一帧。不做 SE(3) 对齐,不按时间关联。"
             "W = traj_W.tum,N = traj_N.tum(原始文件)。单元格为种子中位数 [最小, 最大]。",
             "",
             "| 序列 | 臂 | 输出 | 平移误差 (m) | 旋转误差 (Frobenius) | RMSE | 跟踪率 (%) |",
             "|---|---|---|---|---|---|---|"]
    if not cells and not blocked:
        lines.append("| — | — | — | 无有效运行 | — | — | — |")
        return lines
    order = []
    for c in result.get("cells") or []:
        order.append((c["seq"], c["arm"], c["output"]))
    blocked_only = []
    for seq in blocked:
        if not any(s == seq for s, _, _ in order):
            blocked_only.append(seq)
    for seq, arm, output in order:
        cell = cells[(seq, arm, output)]
        lines.append("| %s | %s | %s | %s | %s | %s | %s |" % (
            seq, arm, output, fmt_cell_translation(cell, digits), fmt_block(cell["rotation_frobenius"], digits),
            fmt_block(cell["rmse"], digits), fmt_block(cell["tracking_rate_pct"], digits)))
    for seq in blocked_only:
        why = blocked[seq]
        if why.startswith(seq + ": "):
            why = why[len(seq) + 2:]
        lines.append("| %s | — | — | 阻塞:%s | — | — | — |" % (seq, why or "未评分"))
    return lines


def guard_for_report(result, evaluation_phase):
    """A2: an evaluation sequence's E5 numbers are not attached to a report outside the evaluation phase.

    -> {"refused": [...]} with no numeric payload, or {"result": result}.
    """
    seqs = set(result.get("sequences") or []) | set(result.get("blocked") or [])
    for c in result.get("cells") or []:
        seqs.add(c["seq"])
    for r in result.get("runs") or []:
        if r.get("seq"):
            seqs.add(r["seq"])
    refused = []
    for seq in sorted(seqs):
        try:
            ec.check_phase(seq, evaluation_phase, "reporting E5")
        except ec.PhaseError as e:
            refused.append(str(e))
    if refused:
        return {"refused": refused}
    return {"result": result}


def build_parser():
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0], formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--matrix", action="append", default=[], help="runs/eval/<name> (repeatable)")
    p.add_argument("--reference", default="", help="one ArUco GT file (official positional reference)")
    p.add_argument("--traj", action="append", default=[], help="one TUM file (repeatable; file mode)")
    p.add_argument("--traj-dir", default="", help="directory walked like ArUco_evaluate.py (.txt and .tum)")
    p.add_argument("--seq", default="", help="sequence name for file mode (A2 applies)")
    p.add_argument("--out", default="", help="output directory (matrix default: <first matrix>/eval)")
    p.add_argument("--arms", default="", help="restrict matrix mode to these arms")
    p.add_argument("--seqs", default="", help="restrict matrix mode to these sequences")
    p.add_argument("--gt", action="append", default=[], help="SEQ=path GT override (matrix mode)")
    p.add_argument("--sort", choices=("translation", "rotation", "rmse"), default="translation",
                   help="file/directory mode only: sort runs by this full-precision error")
    p.add_argument("--evaluation-phase", action="store_true", help="A2: required to score evaluation-set sequences")
    p.add_argument("--root", default=None)
    return p


def _sort_key(name):
    field = {"translation": "translation_error_m", "rotation": "rotation_error", "rmse": "rmse"}[name]
    return field


def main(argv=None):
    args = build_parser().parse_args(argv)
    try:
        if args.matrix:
            return evaluate_matrix(args)
        if not args.reference or not (args.traj or args.traj_dir):
            print("eval_aruco: give --matrix, or --reference with --traj/--traj-dir", file=sys.stderr)
            return 3
        if not args.out:
            print("eval_aruco: file mode needs --out", file=sys.stderr)
            return 3
        trajs = list(args.traj)
        if args.traj_dir:
            if not os.path.isdir(args.traj_dir):
                print("eval_aruco: BLOCKED: not a directory: %s" % args.traj_dir, file=sys.stderr)
                return 3
            trajs.extend(iter_traj_dir(args.traj_dir))
        result = evaluate_files(args.reference, trajs, args.out, seq=args.seq or None,
                                evaluation_phase=args.evaluation_phase)
        field = _sort_key(args.sort)
        result["runs"].sort(key=lambda r: (r.get(field) is None, r.get(field) if r.get(field) is not None else 0.0))
        _write(args.out, result)
        _report(result, args.out)
        return 0
    except ec.PhaseError as e:
        print("eval_aruco: BLOCKED: %s" % e, file=sys.stderr)
        return 3
    except ec.Blocked as e:
        print("eval_aruco: BLOCKED: %s" % e, file=sys.stderr)
        return 3
    except (OSError, ValueError) as e:
        print("eval_aruco: BLOCKED: %s" % e, file=sys.stderr)
        return 3


if __name__ == "__main__":
    sys.exit(main())
