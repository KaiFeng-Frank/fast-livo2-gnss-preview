# SPDX-License-Identifier: BSD-3-Clause
"""eval_aruco.py (TEST_PLAN E5): the M3DGR ArUco_evaluate.py definition on synthetic poses with a known answer,
agreement with a transcription of that script (and with evo's TUM reader when evo imports), the real GNSS-denial
reference layout, and the boundary cases (missing marker block, no endpoint, a comment line, stamps that do not
overlap the bag, A2 before any read)."""
import json
import os
import sys
import tempfile
import unittest

import numpy as np

import _path  # noqa: F401
import eval_aruco as ea
import evalcommon as ec
import make_table as mt

# Exact bytes of data/m3dgr/gt/GTGNSS-denial01.txt and GTGNSS-denial02.txt (release evidence-remote-archive-20260927,
# 01-records). The relative pose the official script reads; not a trajectory.
GT01 = (
    "\n\n\n"
    " 0.99942978  0.00177122  0.03371918\n"
    " -0.00160895  0.999987   -0.00483919\n"
    " -0.03372731  0.00478218  0.99941963\n"
    "\n"
    "  0.00762752\n"
    "  0.00258951\n"
    " -0.04513938\n"
    "\n"
    "bag_time: 609s"
)
GT02 = (
    "\n\n\n"
    "  0.97611445 -0.03604806  0.21424546\n"
    "  0.02047815  0.99701417  0.07445389\n"
    " -0.21628968 -0.06828816  0.97393814\n"
    "\n"
    " -0.23633796\n"
    " -0.01319976\n"
    " -0.03049959\n"
    "\n"
    "bag_time: 750s"
)


def official_read_reference(path):
    """Line-for-line the official read_reference_transform (sjtuyinjie/M3DGR ArUco_evaluate.py)."""
    with open(path, "r", encoding="utf-8") as f:
        lines = f.readlines()
    rotation_matrix = np.array([
        [float(value) for value in lines[3].strip().split()],
        [float(value) for value in lines[4].strip().split()],
        [float(value) for value in lines[5].strip().split()],
    ])
    translation_vector = np.array([
        float(lines[7].strip()),
        float(lines[8].strip()),
        float(lines[9].strip()),
    ])
    ref_total_time = float(lines[-1].strip().split(":")[1].replace("s", "").strip())
    return rotation_matrix, translation_vector, ref_total_time


def official_score(ref_rotation, ref_translation, ref_total_time, poses, stamps):
    """The official per-file loop, without the printer. Poses are the 4x4 list evo would hand it."""
    last_pose_index = len(poses) - 1
    for i in range(len(poses) - 2, -1, -1):
        if not np.allclose(poses[i], poses[last_pose_index]):
            break
        last_pose_index = i
    est_first_pose = poses[0]
    est_last_pose = poses[last_pose_index]
    est_relative_transform = np.linalg.inv(est_first_pose) @ est_last_pose
    est_rotation = est_relative_transform[:3, :3]
    est_translation = est_relative_transform[:3, 3]
    rotation_error = np.linalg.norm(ref_rotation - est_rotation)
    translation_error = np.linalg.norm(ref_translation - est_translation)
    rmse_error = np.sqrt((rotation_error ** 2 + translation_error ** 2) / 2)
    start_time = stamps[0]
    end_time = stamps[last_pose_index]
    eval_total_time = end_time - start_time
    tracking_rate = (eval_total_time / ref_total_time) * 100
    if tracking_rate > 100:
        tracking_rate = 100
    return (float(translation_error), float(rotation_error), float(rmse_error), float(tracking_rate),
            int(last_pose_index))


def se3(r, t):
    pose = np.eye(4)
    pose[:3, :3] = r
    pose[:3, 3] = t
    return pose


def write_gt(path, text):
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write(text)


def write_tum(path, rows):
    """rows: (t, xyz (3,), quat xyzw (4,)). Single spaces, as the replay writer does."""
    with open(path, "w", encoding="utf-8") as f:
        for t, xyz, q in rows:
            f.write("%.9f %.9f %.9f %.9f %.9f %.9f %.9f %.9f\n" % ((t,) + tuple(xyz) + tuple(q)))


def quat_of(r):
    from scipy.spatial.transform import Rotation
    return Rotation.from_matrix(np.asarray(r, dtype=float)).as_quat()


class OfficialAgreement(unittest.TestCase):
    def test_real_reference_matches_the_official_reader_and_a68(self):
        with tempfile.TemporaryDirectory() as d:
            for name, text, bag, trans_m, angle_deg in (
                    ("a.txt", GT01, 609.0, 0.046, 1.96), ("b.txt", GT02, 750.0, 0.239, 13.2)):
                path = os.path.join(d, name)
                write_gt(path, text)
                got = ea.read_reference_transform(path)
                ref = official_read_reference(path)
                np.testing.assert_allclose(got[0], ref[0], atol=0.0)
                np.testing.assert_allclose(got[1], ref[1], atol=0.0)
                self.assertEqual(got[2], ref[2])
                self.assertEqual(got[2], bag)
                rec = ea.reference_record(path, *got)
                self.assertAlmostEqual(rec["translation_norm_m"], trans_m, delta=0.001)
                self.assertAlmostEqual(rec["rotation_angle_deg"], angle_deg, delta=0.15)
                self.assertAlmostEqual(rec["rotation_det"], 1.0, delta=1e-6)

    def test_known_relative_pose_and_the_transcribed_loop(self):
        r = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
        t_ref = np.array([1.0, 2.0, 3.0])
        # identity, then the reference itself: both errors are 0 and the rate is 100 (duration == bag time)
        poses = [se3(np.eye(3), np.zeros(3)), se3(r, t_ref)]
        stamps = np.array([10.0, 619.0])
        got = ea.score_poses(r, t_ref, 609.0, poses, stamps)
        trans, rot, rmse, rate, end = official_score(r, t_ref, 609.0, poses, stamps)
        self.assertEqual(end, 1)
        self.assertAlmostEqual(got["translation_error_m"], 0.0, delta=1e-12)
        self.assertAlmostEqual(got["rotation_error"], 0.0, delta=1e-12)
        self.assertEqual((got["translation_error_m"], got["rotation_error"], got["rmse"], got["tracking_rate_pct"],
                          got["end_index"]), (trans, rot, rmse, rate, end))
        # a pure +0.3 m along x in the relative translation; rotation unchanged
        shifted = [poses[0], se3(r, t_ref + np.array([0.3, 0.0, 0.0]))]
        got = ea.score_poses(r, t_ref, 609.0, shifted, stamps)
        trans, rot, rmse, rate, end = official_score(r, t_ref, 609.0, shifted, stamps)
        self.assertAlmostEqual(trans, 0.3, delta=1e-12)
        self.assertAlmostEqual(rot, 0.0, delta=1e-12)
        self.assertAlmostEqual(rmse, 0.3 / np.sqrt(2.0), delta=1e-12)
        self.assertEqual(rate, 100.0)                       # (619-10)=609, exactly the bag, not capped by a '> 100'
        self.assertEqual((got["translation_error_m"], got["rotation_error"], got["rmse"], got["tracking_rate_pct"]),
                         (trans, rot, rmse, rate))

    def test_frozen_tail_uses_allclose_and_file_order(self):
        r = np.eye(3)
        base = se3(r, np.array([1.0, 0.0, 0.0]))
        # index 2 differs; 3 and 4 match index... the walk compares each pose to the current end, so a 1e-9
        # translation (inside atol 1e-8) is frozen and a 1e-4 translation is not
        frozen = se3(r, np.array([1.0, 0.0, 1e-9]))
        moved = se3(r, np.array([1.0, 0.0, 1e-4]))
        poses = [se3(r, np.zeros(3)), moved, base, frozen, frozen]
        stamps = np.array([0.0, 100.0, 10.0, 20.0, 50.0])   # not time-sorted: duration is stamp[end]-stamp[0], not the max
        got = ea.score_poses(r, np.zeros(3), 40.0, poses, stamps)
        trans, rot, rmse, rate, end = official_score(r, np.zeros(3), 40.0, poses, stamps)
        self.assertEqual(end, 2)                            # pose 2 is the first of the trailing frozen run
        self.assertEqual(got["end_index"], 2)
        self.assertAlmostEqual(got["eval_duration_s"], 10.0, delta=0.0)
        self.assertAlmostEqual(rate, 25.0, delta=1e-12)     # 10/40, and the later stamp 50 is not the end
        self.assertAlmostEqual(got["translation_error_m"], trans, delta=0.0)
        self.assertAlmostEqual(got["tracking_rate_pct"], rate, delta=0.0)
        # longer than the bag: capped at 100, not at 100.something
        long = ea.score_poses(r, np.zeros(3), 40.0, [se3(r, np.zeros(3)), se3(r, np.array([0.2, 0.0, 0.0]))],
                              np.array([0.0, 80.0]))
        self.assertEqual(long["tracking_rate_pct"], 100.0)
        self.assertAlmostEqual(long["translation_error_m"], 0.2, delta=1e-12)

    def test_tum_roundtrip_matches_the_transcription(self):
        r = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
        t_ref = np.array([0.4, -0.2, 0.1])
        with tempfile.TemporaryDirectory() as d:
            gt, tum = os.path.join(d, "gt.txt"), os.path.join(d, "est.tum")
            write_gt(gt, GT01)
            q = quat_of(r)
            write_tum(tum, [(1000.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
                            (1000.0 + 100.0, tuple(t_ref + np.array([0.3, 0.0, 0.0])), tuple(q)),
                            (1000.0 + 609.0, tuple(t_ref + np.array([0.3, 0.0, 0.0])), tuple(q))])
            ref_r, ref_t, bag = ea.read_reference_transform(gt)
            poses, stamps = ea.read_tum_poses(tum)
            self.assertEqual(ea.trailing_endpoint_index(poses), 1)     # the last row repeats the previous pose
            got = ea.score_file(ref_r, ref_t, bag, tum)
            trans, rot, rmse, rate, end = official_score(ref_r, ref_t, bag, poses, stamps)
            self.assertEqual(got["end_index"], end)
            self.assertAlmostEqual(got["translation_error_m"], trans, delta=0.0)
            self.assertAlmostEqual(got["rotation_error"], rot, delta=0.0)
            self.assertAlmostEqual(got["rmse"], rmse, delta=0.0)
            self.assertAlmostEqual(got["tracking_rate_pct"], rate, delta=0.0)
            # the relative translation is t_ref+0.3 against the denial01 translation, so it is not ~0
            self.assertGreater(got["translation_error_m"], 0.2)
            out = os.path.join(d, "out")
            self.assertEqual(ea.main(["--reference", gt, "--traj", tum, "--out", out, "--sort", "translation"]), 0)
            with open(os.path.join(out, "aruco.json"), encoding="utf-8") as f:
                result = json.load(f)
            self.assertAlmostEqual(result["runs"][0]["translation_error_m"], got["translation_error_m"], delta=0.0)
            with open(os.path.join(out, "aruco_runs.csv"), encoding="utf-8") as f:
                csv = f.read().splitlines()
            self.assertIn("translation_error_m", csv[0])
            self.assertTrue(csv[1].startswith(os.path.basename(gt) + ",file,"))


def evo_reader():
    try:
        from evo.tools import file_interface
        return file_interface
    except ImportError:
        extra = "/workspace/storage/fastlivo2_gnss_e5/py"
        if os.path.isdir(extra) and extra not in sys.path:
            sys.path.insert(0, extra)
        try:
            from evo.tools import file_interface
            return file_interface
        except ImportError:
            return None


class EvoPoses(unittest.TestCase):
    def test_poses_match_evo_and_the_official_loop(self):
        reader = evo_reader()
        if reader is None:
            self.skipTest("evo is not importable (the transcribed loop above is the reference)")
        r = np.array([[0.36, -0.8, 0.48], [0.8, 0.6, 0.0], [-0.48, 0.384, 0.8]])
        with tempfile.TemporaryDirectory() as d:
            tum = os.path.join(d, "est.tum")
            write_tum(tum, [(1.0, (0.1, 0.2, 0.3), (0.0, 0.0, 0.0, 1.0)),
                            (2.5, (1.0, -0.4, 0.2), tuple(quat_of(r))),
                            (4.0, (1.0, -0.4, 0.2), tuple(quat_of(r)))])
            poses, stamps = ea.read_tum_poses(tum)
            traj = reader.read_tum_trajectory_file(tum)
            self.assertEqual(len(poses), len(traj.poses_se3))
            for ours, theirs in zip(poses, traj.poses_se3):
                np.testing.assert_allclose(ours, theirs, atol=1e-12)
            np.testing.assert_allclose(stamps, traj.timestamps, atol=0.0)
            ref_r = np.eye(3)
            ref_t = np.zeros(3)
            got = ea.score_poses(ref_r, ref_t, 10.0, poses, stamps)
            trans, rot, rmse, rate, end = official_score(ref_r, ref_t, 10.0, list(traj.poses_se3), traj.timestamps)
            self.assertEqual(got["end_index"], end)
            self.assertAlmostEqual(got["translation_error_m"], trans, delta=1e-12)
            self.assertAlmostEqual(got["rotation_error"], rot, delta=1e-12)
            self.assertAlmostEqual(got["rmse"], rmse, delta=1e-12)
            self.assertAlmostEqual(got["tracking_rate_pct"], rate, delta=1e-12)


class Boundaries(unittest.TestCase):
    def test_missing_marker_block(self):
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "short.txt")
            write_gt(path, "only\nthree\nlines\n")
            with self.assertRaises(ec.Blocked) as cm:
                ea.read_reference_transform(path)
            self.assertIn("rotation block", str(cm.exception))
            write_gt(path, GT01.replace("bag_time: 609s", ""))
            with self.assertRaises(ec.Blocked) as cm:
                ea.read_reference_transform(path)
            self.assertIn("bag time", str(cm.exception))

    def test_no_endpoint_comment_line_and_time_outside_the_bag(self):
        with tempfile.TemporaryDirectory() as d:
            gt = os.path.join(d, "gt.txt")
            write_gt(gt, "\n\n\n1 0 0\n0 1 0\n0 0 1\n\n0\n0\n0\n\nbag_time: 609s")
            empty = os.path.join(d, "empty.tum")
            write_gt(empty, "")
            one = os.path.join(d, "one.tum")
            write_tum(one, [(5.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))])
            comment = os.path.join(d, "comment.tum")
            with open(comment, "w", encoding="utf-8") as f:
                f.write("# a comment, which evo would skip and the official pre-check rejects\n")
                f.write("0 0 0 0 0 0 0 1\n1 1 0 0 0 0 0 1\n")
            # stamps sit far from [0, bag_time]; the official metric still scores the relative pose
            outside = os.path.join(d, "outside.tum")
            write_tum(outside, [(1.0e9, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
                                (1.0e9 + 10.0, (0.0, 0.0, 0.4), (0.0, 0.0, 0.0, 1.0))])
            ref_r, ref_t, bag = ea.read_reference_transform(gt)
            with self.assertRaises(ValueError) as cm:
                ea.score_file(ref_r, ref_t, bag, empty)
            self.assertIn("no poses", str(cm.exception))
            one_score = ea.score_file(ref_r, ref_t, bag, one)
            poses, stamps = ea.read_tum_poses(one)
            trans, rot, rmse, rate, end = official_score(ref_r, ref_t, bag, poses, stamps)
            self.assertEqual(end, 0)
            self.assertEqual(one_score["tracking_rate_pct"], 0.0)
            self.assertAlmostEqual(one_score["translation_error_m"], trans, delta=0.0)
            self.assertTrue(one_score["single_pose"])
            with self.assertRaises(ValueError) as cm:
                ea.read_tum_poses(comment)
            self.assertIn("check_tum_format", str(cm.exception))
            out_score = ea.score_file(ref_r, ref_t, bag, outside)
            self.assertEqual(out_score["status"] if "status" in out_score else "ok", "ok")
            self.assertAlmostEqual(out_score["translation_error_m"], 0.4, delta=1e-12)
            self.assertAlmostEqual(out_score["tracking_rate_pct"], 100.0 * 10.0 / 609.0, delta=1e-12)
            # stamps are ~1e9 s, nowhere near the 609 s bag; the metric still returns the relative-pose error
            self.assertNotIn("overlap", json.dumps(out_score))
            self.assertGreater(out_score["t_start"], 1e8)

    def test_a2_blocks_before_the_gt_is_opened(self):
        with tempfile.TemporaryDirectory() as d:
            mdir = os.path.join(d, "runs", "eval", "mx")
            os.makedirs(mdir)
            with open(os.path.join(mdir, "matrix_manifest.json"), "w", encoding="utf-8") as f:
                json.dump({"name": "mx", "arms": {"U": {"traj": "traj_W.tum"}},
                           "runs": [{"seq": "GNSS-denial01", "arm": "U", "seed": 0, "state": "done",
                                     "attempts": [{"attempt": 1, "complete": 1,
                                                   "run_dir": "runs/eval/mx/GNSS-denial01/U/s0"}]}]}, f)
            rc = ea.main(["--root", d, "--matrix", "runs/eval/mx", "--out", os.path.join(d, "out")])
            self.assertEqual(rc, 3)
            with open(os.path.join(d, "out", "aruco.json"), encoding="utf-8") as f:
                result = json.load(f)
            self.assertIn("A2", result["blocked"]["GNSS-denial01"])
            self.assertNotIn("not found", result["blocked"]["GNSS-denial01"])
            self.assertEqual(result["sequences"], {})

    def test_matrix_scores_w_and_n_and_make_table_keeps_them_out_of_the_ate_columns(self):
        with tempfile.TemporaryDirectory() as d:
            gt_dir = os.path.join(d, "data", "m3dgr", "gt")
            os.makedirs(gt_dir)
            write_gt(os.path.join(gt_dir, "GTGNSS-denial01.txt"),
                     "\n\n\n1 0 0\n0 1 0\n0 0 1\n\n0\n0\n0\n\nbag_time: 100s")
            mdir = os.path.join(d, "runs", "eval", "mx")
            runs = []
            for seed, dx in ((0, 0.0), (1, 0.3)):
                rel = "runs/eval/mx/GNSS-denial01/U/s%d" % seed
                rd = os.path.join(d, rel)
                os.makedirs(rd)
                write_tum(os.path.join(rd, "traj_W.tum"),
                          [(0.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
                           (100.0, (dx, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))])
                runs.append({"seq": "GNSS-denial01", "arm": "U", "seed": seed, "state": "done",
                             "attempts": [{"attempt": 1, "complete": 1, "run_dir": rel}]})
            rel_t = "runs/eval/mx/GNSS-denial01/T/s0"
            rd = os.path.join(d, rel_t)
            os.makedirs(rd)
            write_tum(os.path.join(rd, "traj_W.tum"),
                      [(0.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
                       (50.0, (0.1, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))])
            write_tum(os.path.join(rd, "traj_N.tum"),
                      [(0.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
                       (50.0, (0.4, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))])
            runs.append({"seq": "GNSS-denial01", "arm": "T", "seed": 0, "state": "done",
                         "attempts": [{"attempt": 1, "complete": 1, "run_dir": rel_t}]})
            os.makedirs(mdir, exist_ok=True)
            with open(os.path.join(mdir, "matrix_manifest.json"), "w", encoding="utf-8") as f:
                json.dump({"name": "mx", "arms": {"U": {"traj": "traj_W.tum"}, "T": {"traj": "traj_N.tum"}},
                           "runs": runs}, f)
            out = os.path.join(d, "out")
            self.assertEqual(ea.main(["--root", d, "--matrix", "runs/eval/mx", "--out", out, "--evaluation-phase"]), 0)
            with open(os.path.join(out, "aruco.json"), encoding="utf-8") as f:
                result = json.load(f)
            cells = {(c["arm"], c["output"]): c for c in result["cells"]}
            self.assertAlmostEqual(cells[("U", "W")]["translation_m"]["median"], 0.15, delta=1e-12)
            self.assertEqual(cells[("U", "W")]["n_ok"], 2)
            self.assertEqual(cells[("U", "N")]["n_ok"], 0)
            self.assertIn("traj_N.tum", cells[("U", "N")]["note"])
            self.assertAlmostEqual(cells[("T", "W")]["translation_m"]["median"], 0.1, delta=1e-12)
            self.assertAlmostEqual(cells[("T", "N")]["translation_m"]["median"], 0.4, delta=1e-12)
            self.assertAlmostEqual(cells[("T", "W")]["tracking_rate_pct"]["median"], 50.0, delta=1e-12)
            ate = os.path.join(d, "ate.json")
            with open(ate, "w", encoding="utf-8") as f:
                json.dump({"generated_utc": "2026-09-30T00:00:00Z", "metric": {"cmd": "evo_ape"},
                           "matrices": [], "cells": [], "blocked": {}, "sequences": {}}, f)
            md, csv_ate, csv_e5 = os.path.join(d, "t.md"), os.path.join(d, "t.csv"), os.path.join(d, "e5.csv")
            self.assertEqual(mt.main(["--ate", ate, "--aruco", os.path.join(out, "aruco.json"), "--out-md", md,
                                      "--out-csv", csv_ate, "--out-csv-aruco", csv_e5]), 0)
            with open(md, encoding="utf-8") as f:
                text = f.read()
            self.assertIn("## GNSS-denial 另表:E5", text)
            self.assertIn("| GNSS-denial01 | U | W | 0.15 [0.00, 0.30] |", text)
            self.assertIn("| GNSS-denial01 | U | N | 无 traj_N.tum |", text)
            self.assertIn("| GNSS-denial01 | T | N | 0.40 [0.40, 0.40] |", text)
            self.assertNotIn("待评测(ArUco_evaluate.py)", text)
            with open(csv_ate, encoding="utf-8") as f:
                ate_csv = f.read()
            self.assertNotIn("translation_median", ate_csv.splitlines()[0])
            self.assertIn("Varying-illu03", ate_csv)
            with open(csv_e5, encoding="utf-8") as f:
                e5 = f.read().splitlines()
            self.assertTrue(e5[0].startswith("seq,arm,output,"))
            self.assertTrue(any(row.startswith("GNSS-denial01,U,W,2,2,") for row in e5))
            guarded = ea.guard_for_report(result, evaluation_phase=False)
            self.assertIn("refused", guarded)
            self.assertNotIn("result", guarded)
            self.assertIn("A2", guarded["refused"][0])
            self.assertIn("result", ea.guard_for_report(result, evaluation_phase=True))


if __name__ == "__main__":
    unittest.main()
