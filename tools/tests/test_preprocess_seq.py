# SPDX-License-Identifier: BSD-3-Clause
"""tools/time_calib/preprocess_seq.py (the one-key per-sequence preprocessing) on a fake project root: the A2 guard,
input resolution (bag, GT, BRDC of the UTC day), the replay step handed to run_matrix, the three container steps with
their arguments (P4c replay route on development sequences only), archiving of earlier outputs, the summary."""
import contextlib
import io
import json
import os
import sys
import tempfile
import unittest

import _path  # noqa: F401
import preprocess_seq as pp

FAKE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "fake_tool_docker.py")


def write(path, text):
    with open(path, "w") as f:
        f.write(text)


def read(path):
    with open(path) as f:
        return f.read()


class Preprocess(unittest.TestCase):
    def root(self, d, seq, brdc):
        for rel in ("data/m3dgr/filtered/%s.bag" % seq, "data/m3dgr/gt/%s.txt" % seq, "data/brdc/%s" % brdc,
                    "wt/head/ws/devel/setup.bash", "runs/r1/replay_stats.json"):
            os.makedirs(os.path.dirname(os.path.join(d, rel)), exist_ok=True)
            write(os.path.join(d, rel), "{}" if rel.endswith(".json") else "x")
        with open(os.path.join(d, "runs/r1/run_status.json"), "w") as f:
            json.dump({"complete": 1}, f)
        os.makedirs(os.path.join(d, "audit", seq, "time_calib"))
        write(os.path.join(d, "audit", seq, "time_calib", "%s.yaml" % seq), "old")

    def run_pp(self, d, args):
        log = os.path.join(d, "calls.jsonl")
        os.environ["FAKE_TOOL_LOG"] = log
        out, err = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            rc = pp.main(args + ["--root", d, "--devel", "head:ws/devel", "--docker-cmd", "%s %s" % (sys.executable, FAKE)])
        calls = [json.loads(x) for x in read(log).splitlines()] if os.path.exists(log) else []
        return rc, out.getvalue(), err.getvalue(), calls

    def test_development_sequence(self):
        with tempfile.TemporaryDirectory() as d:
            self.root(d, "Grass02", "BRDC00IGS_R_20250210000_01D_MN.rnx")
            rc, out, err, calls = self.run_pp(d, ["Grass02", "--replay-dir", "runs/r1"])
            self.assertEqual(rc, 0, err)
            self.assertEqual(len(calls), 3)
            m1, m2, gt = (c["script"] for c in calls)
            self.assertIn("gnss_time_calib.py /r/data/m3dgr/filtered/Grass02.bag - /r/audit/Grass02/time_calib/Grass02_record_time.yaml", m1)
            self.assertIn("gnss_time_calib.py /r/data/m3dgr/filtered/Grass02.bag /r/runs/r1 /r/audit/Grass02/time_calib/Grass02.yaml", m2)
            self.assertIn("--nav-rinex /r/data/brdc/BRDC00IGS_R_20250210000_01D_MN.rnx", m2)
            self.assertIn("gt_audit.py /r/data/m3dgr/gt/Grass02.txt /r/data/m3dgr/filtered/Grass02.bag /r/audit/Grass02/gt", gt)
            self.assertIn("--replay-dir /r/runs/r1", gt)                      # P4c on the development set
            rec = json.loads(read(os.path.join(d, "audit/Grass02/preprocess.json")))
            self.assertEqual(rec["time_calib"]["segments"][0]["method"], "xcorr")
            self.assertEqual(rec["gt"]["timebase"]["method"], "record_time")
            prev = rec["archived_previous"]
            self.assertEqual(len(prev), 1)
            self.assertEqual(read(os.path.join(d, prev[0], "Grass02.yaml")), "old")   # earlier file kept
            self.assertIn("xcorr +41.2 ms sigma 30.5 ms", out)

    def test_evaluation_sequence_needs_the_phase_and_gets_no_replay_route(self):
        with tempfile.TemporaryDirectory() as d:
            self.root(d, "Dark01", "BRDC00IGS_R_20243300000_01D_MN.rnx")
            rc, out, err, calls = self.run_pp(d, ["Dark01", "--replay-dir", "runs/r1"])
            self.assertEqual(rc, pp.EXIT_BLOCKED)
            self.assertIn("A2", err)
            self.assertEqual(calls, [])
            rc, out, err, calls = self.run_pp(d, ["Dark01", "--replay-dir", "runs/r1", "--evaluation-phase"])
            self.assertEqual(rc, 0, err)
            self.assertNotIn("--replay-dir", calls[2]["script"])             # no estimated trajectory near the GT (A2)
            self.assertIn("BRDC00IGS_R_20243300000_01D_MN.rnx", calls[1]["script"])   # 2024-11-25 local = UTC day 330

    def test_replay_step_and_missing_inputs(self):
        with tempfile.TemporaryDirectory() as d:
            self.root(d, "Outdoor01", "BRDC00IGS_R_20250030000_01D_MN.rnx")
            with contextlib.redirect_stdout(io.StringIO()) as o:
                rc = pp.main(["Outdoor01", "--evaluation-phase", "--dry-run", "--root", d, "--devel", "head:ws/devel"])
            self.assertEqual(rc, 0)
            plan = json.loads(o.getvalue())
            rm = plan["commands"][0]
            for part in ("run_matrix.py run --name preproc_Outdoor01 --arms U --seqs Outdoor01 --seeds 0 --devel U=head:ws/devel",
                         "--bag Outdoor01=data/m3dgr/filtered/Outdoor01.bag", "--evaluation-phase"):
                self.assertIn(part, rm)
            self.assertEqual(plan["replay_dir"], "runs/eval/preproc_Outdoor01/Outdoor01/U/s0")
            os.remove(os.path.join(d, "data/brdc/BRDC00IGS_R_20250030000_01D_MN.rnx"))
            with contextlib.redirect_stderr(io.StringIO()) as e:
                rc = pp.main(["Outdoor01", "--evaluation-phase", "--dry-run", "--root", d, "--devel", "head:ws/devel"])
            self.assertEqual(rc, pp.EXIT_BLOCKED)
            self.assertIn("BRDC", e.getvalue())


if __name__ == "__main__":
    unittest.main()
