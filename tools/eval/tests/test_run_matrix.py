# SPDX-License-Identifier: BSD-3-Clause
"""run_matrix.py with a stand-in docker/fetch: scheduling under the slot cap, retry once, resume without repeating
finished runs, per-run wrapper launch for overlay arms (U included: A55, HEAD with config/arms/U.yaml), one build per
matrix (A55), rolling fetch/drop within --max-bags, timeouts, A2."""
import json
import os
import shutil
import sys
import tempfile
import threading
import time
import unittest
import xml.etree.ElementTree as ET
from unittest import mock

import _path
import evalcommon as ec
import run_matrix as rm

HERE = os.path.dirname(os.path.abspath(__file__))
PY = sys.executable


def read(p):
    with open(p, encoding="utf-8") as f:
        return f.read()


class Env:
    """A throw-away project root: two remote builds (ref, head), bags, BRDC, an A18 file."""

    def __init__(self, bags=("Grass01",)):
        self.root = tempfile.mkdtemp()
        self.state = os.path.join(self.root, ".fake")
        for wt in ("refwt", "headwt"):
            d = os.path.join(self.root, "wt", wt, "ws", "devel")
            os.makedirs(d)
            with open(os.path.join(d, "setup.bash"), "w") as f:
                f.write("true\n")
            with open(os.path.join(d, "BUILD_ID"), "w") as f:
                f.write("abc1234 0 eval-profile\n")
        ov = os.path.join(self.root, "wt", "headwt", "ws", "src", "FAST-LIVO2", "config", "arms")
        os.makedirs(ov)
        with open(os.path.join(ov, "T.yaml"), "w") as f:
            f.write("gnss:\n  enable: true\n  mode: tight\n")
        with open(os.path.join(ov, "U.yaml"), "w") as f:
            f.write("gnss:\n  enable: false\n")
        with open(os.path.join(ov, "T-pr.yaml"), "w") as f:
            f.write("gnss:\n  enable: true\noutput:\n  dir: /tmp\n")
        fd = os.path.join(self.root, "data", "m3dgr", "filtered")
        os.makedirs(fd)
        for b in bags:
            with open(os.path.join(fd, b + ".bag"), "w") as f:
                f.write("bag")
            with open(os.path.join(fd, b + ".bag.sha256"), "w") as f:
                f.write("%s  %s.bag\n" % ("a" * 64, b))
        os.makedirs(os.path.join(self.root, "data", "brdc"))
        with open(os.path.join(self.root, "data", "brdc", "BRDC00IGS_R_20250190000_01D_MN.rnx"), "w") as f:
            f.write("x")
        tc = os.path.join(self.root, "audit", "Grass01", "time_calib")
        os.makedirs(tc)
        for n in ("Grass01_record_time.yaml", "Grass01.yaml"):
            with open(os.path.join(tc, n), "w") as f:
                f.write("sequence: Grass01\n")

    def args(self, *extra, **cfg):
        cfg.setdefault("state", self.state)
        cfg.setdefault("track", os.path.join(self.state, "track.json"))
        os.makedirs(self.state, exist_ok=True)
        os.environ["FAKE_DOCKER"] = json.dumps(cfg)
        if not os.path.isfile(self.meminfo):
            self.set_available_gib(64)
        os.makedirs(self.proc, exist_ok=True)
        return ["run", "--root", self.root, "--docker-cmd", "%s %s" % (PY, os.path.join(HERE, "fake_docker.py")),
                "--manifest-check", os.path.join(HERE, "fake_manifest_check.py"), "--poll-s", "0.05",
                "--meminfo", self.meminfo, "--proc-root", self.proc,
                "--devel", "ref=refwt:ws/devel", "--devel", "head=headwt:ws/devel"] + list(extra)

    @property
    def meminfo(self):
        return os.path.join(self.root, ".meminfo")

    @property
    def proc(self):
        return os.path.join(self.root, ".proc")

    def set_available_gib(self, gib):
        """The host's MemAvailable as the runner reads it (a fake /proc/meminfo; written atomically)."""
        tmp = self.meminfo + ".tmp"
        with open(tmp, "w") as f:
            f.write("MemTotal:       65011712 kB\nMemFree:        1000000 kB\nMemAvailable:   %d kB\n" % int(gib * 1024 * 1024))
        os.replace(tmp, self.meminfo)

    def invocations(self):
        p = os.path.join(self.state, "invocations.log")
        return [json.loads(ln) for ln in read(p).splitlines()] if os.path.isfile(p) else []

    def manifest(self, name):
        return ec.load_json(os.path.join(self.root, "runs", "eval", name, "matrix_manifest.json"))

    def close(self):
        shutil.rmtree(self.root)


class Scheduling(unittest.TestCase):
    def setUp(self):
        self.e = Env()

    def tearDown(self):
        self.e.close()

    def test_runs_retry_once_and_resume(self):
        a = self.e.args("--name", "m1", "--arms", "U", "--seqs", "Grass01", "--seeds", "0-3", "--jobs", "2",
                        fail_first=["Grass01/U/s1"], sleep=0.3)
        self.assertEqual(rm.main(a), 0)
        m = self.e.manifest("m1")
        self.assertEqual(m["counts"], {"waiting": 0, "running": 0, "done": 4, "failed": 0})
        s1 = [r for r in m["runs"] if r["seed"] == 1][0]
        self.assertEqual([at["run_dir"].rsplit("/", 1)[1] for at in s1["attempts"]], ["s1", "s1.retry1"])
        self.assertEqual([at["complete"] for at in s1["attempts"]], [0, 1])
        self.assertEqual(s1["attempts"][0]["status"]["node_exit_code"], "-11")
        for r in m["runs"]:
            st = ec.load_json(os.path.join(self.e.root, r["attempts"][-1]["run_dir"], "run_status.json"))
            self.assertEqual(st, {"complete": 1, "roslaunch_rc": 0, "node_exit_code": "0", "done_line": 1, "replay_stats": 1,
                                  "manifest_ok": 1, "cycles": 17})
            self.assertEqual(r["attempts"][-1]["build_sha256"], "b" * 64)
        self.assertLessEqual(ec.load_json(os.path.join(self.e.state, "track.json"))["peak"], 2)
        inv = self.e.invocations()
        self.assertEqual(len(inv), 5)
        s0 = [i for i in inv if i["rel"].endswith("/s0")][0]
        # A55: U = the HEAD build with config/arms/U.yaml (gnss.enable false) through the wrapper; no GNSS inputs
        self.assertIn("source /r/wt/headwt/ws/devel/setup.bash", s0["inner"])
        self.assertIn("/r/runs/eval/m1/Grass01/U/s0/replay.launch", s0["inner"])
        launch = ET.parse(os.path.join(self.e.root, "runs/eval/m1/Grass01/U/s0/replay.launch")).getroot()
        self.assertEqual([x.get("file") for x in launch.findall("rosparam")], ["/r/wt/headwt/ws/src/FAST-LIVO2/config/arms/U.yaml"])
        self.assertIn("seed:=0 ", s0["inner"])
        self.assertIn("output_dir:=/r/runs/eval/m1/Grass01/U/s0 ", s0["inner"])
        self.assertIn("bag_sha256:=" + "a" * 64, s0["inner"])
        self.assertEqual(sorted(i["rel"].rsplit("/", 1)[1] for i in inv), ["s0", "s1", "s1.retry1", "s2", "s3"])
        # resume: nothing is repeated, the hashes of the finished runs are read back from their directories
        self.assertEqual(rm.main(a), 0)
        self.assertEqual(len(self.e.invocations()), 5)
        m = self.e.manifest("m1")
        self.assertEqual(m["counts"]["done"], 4)
        self.assertTrue(all(r["attempts"][-1]["build_sha256"] == "b" * 64 for r in m["runs"]))

    def test_one_orchestrator_per_matrix(self):
        import fcntl
        d = os.path.join(self.e.root, "runs", "eval", "m9")
        os.makedirs(d)
        fd = os.open(os.path.join(d, ".matrix.lock"), os.O_RDWR | os.O_CREAT)
        fcntl.flock(fd, fcntl.LOCK_EX)
        try:
            self.assertEqual(rm.main(self.e.args("--name", "m9", "--arms", "U", "--seqs", "Grass01", "--seeds", "0")), 3)
            self.assertEqual(self.e.invocations(), [])
        finally:
            os.close(fd)

    def test_second_failure_is_final(self):
        a = self.e.args("--name", "m2", "--arms", "U", "--seqs", "Grass01", "--seeds", "0", "--retries", "0",
                        fail_first=["Grass01/U/s0"])
        self.assertEqual(rm.main(a), 1)
        self.assertEqual(self.e.manifest("m2")["counts"]["failed"], 1)

    def test_timeout_kills_and_retries(self):
        a = self.e.args("--name", "m3", "--arms", "U", "--seqs", "Grass01", "--seeds", "0", "--timeout-factor", "0",
                        "--timeout-base", "0.5", "--retries", "1", hang=["Grass01/U/s0"])
        self.assertEqual(rm.main(a), 1)
        r = self.e.manifest("m3")["runs"][0]
        self.assertEqual(len(r["attempts"]), 2)
        self.assertTrue(all(at["killed"].startswith("timeout") for at in r["attempts"]))

    def test_gnss_arm_gets_wrapper_launch_with_overlays(self):
        with open(os.path.join(self.e.root, "data", "m3dgr", "filtered", "Grass01.fetch.json"), "w") as f:
            json.dump({"brdc": ["data/brdc/BRDC00IGS_R_20250190000_01D_MN.rnx"]}, f)
        a = self.e.args("--name", "m4", "--arms", "T", "--seqs", "Grass01", "--seeds", "0")
        self.assertEqual(rm.main(a), 0)
        m = self.e.manifest("m4")
        run = os.path.join(self.e.root, m["runs"][0]["attempts"][0]["run_dir"])
        launch = ET.parse(os.path.join(run, "replay.launch")).getroot()
        inc = launch.find("include")
        self.assertEqual(inc.get("file"), "$(find fast_livo)/launch/m3dgr_avia_replay.launch")
        self.assertEqual(inc.get("pass_all_args"), "true")
        loads = [x.get("file") for x in launch.findall("rosparam")]
        self.assertEqual(loads, ["/r/wt/headwt/ws/src/FAST-LIVO2/config/arms/T.yaml", "/r/runs/eval/m4/overlays/Grass01.gnss_inputs.yaml"])
        ov = read(os.path.join(self.e.root, "runs", "eval", "m4", "overlays", "Grass01.gnss_inputs.yaml"))
        self.assertIn('nav_rinex: ["/r/data/brdc/BRDC00IGS_R_20250190000_01D_MN.rnx"]', ov)
        self.assertIn('calib_file: "/r/audit/Grass01/time_calib/Grass01.yaml"', ov)     # method 2 preferred over the prior
        self.assertIn("/r/runs/eval/m4/Grass01/T/s0/replay.launch", self.e.invocations()[0]["inner"])
        self.assertEqual(m["arms"]["T"]["overlay"], "wt/headwt/ws/src/FAST-LIVO2/config/arms/T.yaml")
        self.assertEqual(len(m["arms"]["T"]["overlay_sha256"]), 64)

    def _literal_overlay(self, name="LIVGO-literal"):
        ov = os.path.join(self.e.root, "wt", "headwt", "ws", "src", "FAST-LIVO2", "config", "arms", name + ".yaml")
        with open(ov, "w") as f:
            f.write("gnss:\n  enable: true\n  mode: tight\n")

    def _brdc_record(self):
        with open(os.path.join(self.e.root, "data", "m3dgr", "filtered", "Grass01.fetch.json"), "w") as f:
            json.dump({"brdc": ["data/brdc/BRDC00IGS_R_20250190000_01D_MN.rnx"]}, f)

    def test_time_calib_false_arm_gets_the_inputs_overlay_without_the_a18_file(self):
        # A81: arms.json "time_calib": false (LIVGO-literal) = nav_rinex only; T of the same matrix keeps the A18 file
        self._literal_overlay()
        self._brdc_record()
        a = self.e.args("--name", "m20", "--arms", "T,LIVGO-literal", "--seqs", "Grass01", "--seeds", "0")
        self.assertEqual(rm.main(a), 0)
        m = self.e.manifest("m20")
        self.assertIs(m["arms"]["T"]["time_calib"], True)
        self.assertIs(m["arms"]["LIVGO-literal"]["time_calib"], False)
        loads = {}
        for r in m["runs"]:
            run = os.path.join(self.e.root, r["attempts"][0]["run_dir"])
            launch = ET.parse(os.path.join(run, "replay.launch")).getroot()
            loads[r["arm"]] = [x.get("file") for x in launch.findall("rosparam")]
            self.assertEqual(r["state"], "done")
        self.assertEqual(loads["T"], ["/r/wt/headwt/ws/src/FAST-LIVO2/config/arms/T.yaml", "/r/runs/eval/m20/overlays/Grass01.gnss_inputs.yaml"])
        self.assertEqual(loads["LIVGO-literal"], ["/r/wt/headwt/ws/src/FAST-LIVO2/config/arms/LIVGO-literal.yaml",
                                                  "/r/runs/eval/m20/overlays/Grass01.gnss_inputs_nocalib.yaml"])
        odir = os.path.join(self.e.root, "runs", "eval", "m20", "overlays")
        with_calib = read(os.path.join(odir, "Grass01.gnss_inputs.yaml"))
        without = read(os.path.join(odir, "Grass01.gnss_inputs_nocalib.yaml"))
        self.assertIn('calib_file: "/r/audit/Grass01/time_calib/Grass01.yaml"', with_calib)
        self.assertIn('nav_rinex: ["/r/data/brdc/BRDC00IGS_R_20250190000_01D_MN.rnx"]', without)
        self.assertNotIn("calib_file", without)
        self.assertNotIn("time:", without)
        s = m["sequences"]["Grass01"]
        self.assertEqual(s["gnss_inputs"]["calib_file"], "audit/Grass01/time_calib/Grass01.yaml")
        self.assertEqual(s["gnss_inputs_nocalib"]["calib_file"], None)
        self.assertEqual(s["gnss_inputs_nocalib"]["calib_kind"], None)
        self.assertEqual(s["gnss_inputs_nocalib"]["nav_rinex"], s["gnss_inputs"]["nav_rinex"])

    def test_time_calib_false_arm_alone_writes_only_the_nocalib_overlay_and_others_are_unchanged(self):
        self._literal_overlay()
        self._brdc_record()
        a = self.e.args("--name", "m21", "--arms", "LIVGO-literal-diag", "--seqs", "Grass01", "--seeds", "0")
        self._literal_overlay("LIVGO-literal-diag")
        self.assertEqual(rm.main(a), 0)
        odir = os.path.join(self.e.root, "runs", "eval", "m21", "overlays")
        self.assertEqual(sorted(os.listdir(odir)), ["Grass01.gnss_inputs_nocalib.yaml"])
        # a matrix without a time_calib false arm writes what it wrote before: one inputs overlay, no nocalib key anywhere
        a = self.e.args("--name", "m22", "--arms", "T", "--seqs", "Grass01", "--seeds", "0")
        self.assertEqual(rm.main(a), 0)
        odir = os.path.join(self.e.root, "runs", "eval", "m22", "overlays")
        self.assertEqual(sorted(os.listdir(odir)), ["Grass01.gnss_inputs.yaml"])
        m = self.e.manifest("m22")
        self.assertNotIn("gnss_inputs_nocalib", m["sequences"]["Grass01"])
        self.assertNotIn("overlay_seq_nocalib", m["sequences"]["Grass01"])

    def test_time_calib_must_be_a_boolean(self):
        arms = ec.load_arms()
        arms["T"] = dict(arms["T"], time_calib="no")
        with mock.patch.object(ec, "load_arms", return_value=arms):
            a = self.e.args("--name", "m23", "--arms", "T", "--seqs", "Grass01", "--seeds", "0")
            self.assertEqual(rm.main(a), 3)
        self.assertEqual(self.e.invocations(), [])

    def test_one_build_per_matrix(self):
        # U on another build than the GNSS arm (the P3 layout: U = ref-R binary) is refused (A55) ...
        a = self.e.args("--name", "m10", "--arms", "U,T", "--seqs", "Grass01", "--seeds", "0", "--devel", "U=refwt:ws/devel")
        self.assertEqual(rm.main(a), 3)
        self.assertEqual(self.e.invocations(), [])
        # ... unless asked for explicitly (diagnostic matrix; recorded in the manifest)
        ov = os.path.join(self.e.root, "wt", "refwt", "ws", "src", "FAST-LIVO2", "config", "arms")
        os.makedirs(ov)
        with open(os.path.join(ov, "U.yaml"), "w") as f:
            f.write("gnss:\n  enable: false\n")
        with open(os.path.join(self.e.root, "data", "m3dgr", "filtered", "Grass01.fetch.json"), "w") as f:
            json.dump({"brdc": ["data/brdc/BRDC00IGS_R_20250190000_01D_MN.rnx"]}, f)
        a = self.e.args("--name", "m11", "--arms", "U,T", "--seqs", "Grass01", "--seeds", "0", "--devel", "U=refwt:ws/devel",
                        "--allow-mixed-builds")
        self.assertEqual(rm.main(a), 0)
        self.assertTrue(self.e.manifest("m11")["allow_mixed_builds"])
        # the same --devel for every arm: accepted
        a = self.e.args("--name", "m12", "--arms", "U,T", "--seqs", "Grass01", "--seeds", "0")
        self.assertEqual(rm.main(a), 0)
        self.assertFalse(self.e.manifest("m12")["allow_mixed_builds"])
        self.assertEqual({r["arm"]: r["state"] for r in self.e.manifest("m12")["runs"]}, {"U": "done", "T": "done"})

    def test_overlay_with_reserved_keys_is_refused(self):
        a = self.e.args("--name", "m5", "--arms", "T-pr", "--seqs", "Grass01", "--seeds", "0")
        self.assertEqual(rm.main(a), 3)
        self.assertEqual(self.e.invocations(), [])

    def test_missing_overlay_and_devel_are_problems(self):
        a = self.e.args("--name", "m6", "--arms", "T-dop", "--seqs", "Grass01", "--seeds", "0")
        self.assertEqual(rm.main(a), 3)
        a = ["plan", "--root", self.e.root, "--name", "m6", "--arms", "U", "--seqs", "Grass01"]
        self.assertEqual(rm.main(a), 3)          # no --devel for build head

    def test_evaluation_sequence_refused(self):
        a = self.e.args("--name", "m7", "--arms", "U", "--seqs", "Dark01", "--seeds", "0")
        self.assertEqual(rm.main(a), 3)
        self.assertFalse(os.path.exists(os.path.join(self.e.root, "runs")))
        # The terminal recheck arms remain development-only even with an explicit evaluation-phase flag.
        self._brdc_record()
        for arm in ("LIVGO-literal-recheck-smartpnt-diag", "LIVGO-literal-recheck-mars-diag"):
            with self.subTest(arm=arm):
                self._literal_overlay(arm)
                a = self.e.args("--name", "m7", "--arms", arm, "--seqs", "Grass01", "--seeds", "0",
                                "--evaluation-phase")
                matrix = rm.Matrix(rm.build_parser().parse_args(a))
                self.assertIn("arm %s is a development-set calibration arm (A58), not an evaluation arm" % arm,
                              matrix.resolve())
                self.assertIs(matrix.arm_info[arm]["time_calib"], False)
                self.assertEqual(rm.main(a), 3)
                self.assertFalse(os.path.exists(os.path.join(self.e.root, "runs")))
                self.assertEqual(self.e.invocations(), [])

    def test_jobs_cap(self):
        a = self.e.args("--name", "m8", "--arms", "U", "--seqs", "Grass01", "--seeds", "0", "--jobs", "9")
        self.assertEqual(rm.main(a), 3)


class Memory(unittest.TestCase):
    """README "内存" (2026-09-25 incident): three replays by default, a MemAvailable gate before every launch, a
    per-container memory limit and OOM preference, a brake that kills the youngest replay, peak RSS per attempt."""

    def setUp(self):
        self.e = Env()

    def tearDown(self):
        self.e.close()

    def events(self, name):
        return read(os.path.join(self.e.root, "runs", "eval", name, "matrix_events.log"))

    def test_default_three_jobs_docker_memory_options_and_peak_rss(self):
        a = self.e.args("--name", "mm1", "--arms", "U", "--seqs", "Grass01", "--seeds", "0-5", "--mem-sample-s", "0.02",
                        sleep=0.5, proc=self.e.proc, hwm_kib=3 * 1024 * 1024)
        self.assertEqual(rm.build_parser().parse_args(a).jobs, 3)
        self.assertEqual(rm.main(a), 0)
        self.assertEqual(ec.load_json(os.path.join(self.e.state, "track.json"))["peak"], 3)
        argv = self.e.invocations()[0]["argv"]
        i = argv.index("--memory")
        self.assertEqual(argv[i:i + 4], ["--memory", "16384m", "--memory-swap", "16384m"])
        self.assertEqual(argv[argv.index("--oom-score-adj") + 1], "1000")
        self.assertLess(argv.index("--oom-score-adj"), argv.index(rm.build_parser().parse_args(a).image))   # a docker option
        m = self.e.manifest("mm1")
        self.assertEqual(m["jobs_max"], 3)
        self.assertEqual(m["memory"], {"min_avail_gib": 16.0, "brake_avail_gib": 6.0, "mem_limit_gib": 16.0, "oom_score_adj": 1000,
                                       "sample_s": 0.02, "max_brakes_per_run": rm.MAX_BRAKES_PER_RUN})
        for r in m["runs"]:
            at = r["attempts"][0]
            self.assertEqual(at["mem_available_at_start_gib"], 64.0)
            self.assertEqual(at["peak_rss_gib"], 3.0)
            self.assertEqual(ec.load_json(os.path.join(self.e.root, at["run_dir"], "run_status.json"))["peak_rss_gib"], 3.0)
        # no limit / no OOM preference on request
        a = self.e.args("--name", "mm1b", "--arms", "U", "--seqs", "Grass01", "--seeds", "0", "--mem-limit-gb", "0")
        self.assertEqual(rm.main(a), 0)
        self.assertNotIn("--memory", self.e.invocations()[-1]["argv"])

    def test_gate_waits_for_memory_before_launching(self):
        self.e.set_available_gib(10)
        a = self.e.args("--name", "mm2", "--arms", "U", "--seqs", "Grass01", "--seeds", "0-1", "--jobs", "2")
        opened = []

        def opener():
            time.sleep(0.8)
            opened.append(time.time())
            self.e.set_available_gib(40)
        th = threading.Thread(target=opener)
        th.start()
        self.assertEqual(rm.main(a), 0)
        th.join()
        inv = self.e.invocations()
        self.assertEqual(len(inv), 2)
        self.assertTrue(all(i["t"] >= opened[0] for i in inv))
        ev = self.events("mm2")
        self.assertIn("memory gate closed: MemAvailable 10.0 GiB < 16.0 GiB", ev)
        self.assertIn("memory gate open: MemAvailable 40.0 GiB", ev)

    def test_brake_kills_the_youngest_and_requeues_it_without_using_a_retry(self):
        a = self.e.args("--name", "mm3", "--arms", "U", "--seqs", "Grass01", "--seeds", "0-1", "--jobs", "2", "--retries", "0",
                        "--brake-interval-s", "5", hang_first=["Grass01/U/s1"], sleep=2.0)
        evlog = os.path.join(self.e.root, "runs", "eval", "mm3", "matrix_events.log")

        def squeeze():
            t_end = time.time() + 20
            while time.time() < t_end and len(self.e.invocations()) < 2:
                time.sleep(0.02)
            time.sleep(0.3)
            self.e.set_available_gib(4)
            while time.time() < t_end and "memory brake" not in (read(evlog) if os.path.isfile(evlog) else ""):
                time.sleep(0.01)
            self.e.set_available_gib(40)
        th = threading.Thread(target=squeeze)
        th.start()
        self.assertEqual(rm.main(a), 0)
        th.join()
        m = self.e.manifest("mm3")
        self.assertEqual(m["counts"], {"waiting": 0, "running": 0, "done": 2, "failed": 0})
        s0, s1 = sorted(m["runs"], key=lambda r: r["seed"])
        self.assertEqual(len(s0["attempts"]), 1)
        self.assertEqual([at["run_dir"].rsplit("/", 1)[1] for at in s1["attempts"]], ["s1", "s1.retry1"])
        self.assertTrue(s1["attempts"][0]["memory_brake"])
        self.assertTrue(s1["attempts"][0]["killed"].startswith("memory brake: MemAvailable 4.0 GiB < 6.0 GiB"))
        st = ec.load_json(os.path.join(self.e.root, s1["attempts"][0]["run_dir"], "run_status.json"))
        self.assertEqual((st["complete"], st["memory_brake"]), (0, 1))
        self.assertIn("killing the youngest replay Grass01/U/s1 attempt 1", self.events("mm3"))
        # resume: nothing repeated; the braked attempt is read back as such
        n = len(self.e.invocations())
        self.assertEqual(rm.main(a), 0)
        self.assertEqual(len(self.e.invocations()), n)
        self.assertTrue(self.e.manifest("mm3")["runs"][1]["attempts"][0]["memory_brake"])

    def test_resume_counts_ordinary_failures_but_not_brakes(self):
        for name, st in (("mm4", {"complete": 0, "killed": "memory brake: x", "memory_brake": 1}), ("mm5", {"complete": 0})):
            d = os.path.join(self.e.root, "runs", "eval", name, "Grass01", "U", "s0")
            os.makedirs(d)
            with open(os.path.join(d, "run_status.json"), "w") as f:
                json.dump(st, f)
        a = self.e.args("--name", "mm4", "--arms", "U", "--seqs", "Grass01", "--seeds", "0", "--retries", "0")
        self.assertEqual(rm.main(a), 0)
        self.assertEqual([i["rel"].rsplit("/", 1)[1] for i in self.e.invocations()], ["s0.retry1"])
        a = self.e.args("--name", "mm5", "--arms", "U", "--seqs", "Grass01", "--seeds", "0", "--retries", "0")
        self.assertEqual(rm.main(a), 1)
        self.assertEqual(self.e.manifest("mm5")["counts"]["failed"], 1)
        self.assertEqual(len(self.e.invocations()), 1)

    def test_thresholds_are_checked(self):
        a = self.e.args("--name", "mm6", "--arms", "U", "--seqs", "Grass01", "--seeds", "0", "--brake-avail-gb", "20")
        self.assertEqual(rm.main(a), 3)
        a = self.e.args("--name", "mm6", "--arms", "U", "--seqs", "Grass01", "--seeds", "0", "--meminfo", "/nonexistent/meminfo")
        self.assertEqual(rm.main(a), 3)
        self.assertEqual(self.e.invocations(), [])

    def test_disk_gate_and_gzip_after(self):
        calls = []

        def fake_free(path):
            calls.append(path)
            return 1.0 if len(calls) <= 3 else 100.0
        a = self.e.args("--name", "mm7", "--arms", "U", "--seqs", "Grass01", "--seeds", "0",
                        "--gzip-after", "console.log,state_trace.txt,absent.csv")
        with mock.patch.object(rm, "free_disk_gib", side_effect=fake_free):
            self.assertEqual(rm.main(a), 0)
        self.assertIn("disk gate closed: 1.0 GiB free under %s < 8.0 GiB" % self.e.root, self.events("mm7"))
        m = self.e.manifest("mm7")
        self.assertEqual(m["disk"], {"min_free_gib": 8.0, "gzip_after": ["console.log", "state_trace.txt", "absent.csv"],
                                     "extra": []})
        at = m["runs"][0]["attempts"][0]
        self.assertEqual(at["gzipped"], ["console.log", "state_trace.txt"])
        run = os.path.join(self.e.root, at["run_dir"])
        import gzip
        self.assertFalse(os.path.exists(os.path.join(run, "state_trace.txt")))
        with gzip.open(os.path.join(run, "state_trace.txt.gz"), "rt") as f:
            self.assertEqual(f.read(), "# header\n" + "row\n" * 17)
        with gzip.open(os.path.join(run, "console.log.gz"), "rt") as f:
            self.assertIn("fastlivo_replay: done", f.read())
        self.assertEqual(ec.load_json(os.path.join(run, "run_status.json"))["cycles"], 17)     # counted before the gzip
        self.assertEqual(rm.main(a), 0)                                                        # resume reads run_status.json only
        self.assertEqual(len(self.e.invocations()), 1)
        a = self.e.args("--name", "mm8", "--arms", "U", "--seqs", "Grass01", "--seeds", "0", "--gzip-after", "run_status.json")
        self.assertEqual(rm.main(a), 3)

    def test_replay_memory_maps_each_node_to_its_run(self):
        proc = self.e.proc

        def mk(pid, comm, cmdline, environ, rss, hwm):
            d = os.path.join(proc, str(pid))
            os.makedirs(d)
            with open(os.path.join(d, "comm"), "w") as f:
                f.write(comm + "\n")
            with open(os.path.join(d, "cmdline"), "wb") as f:
                f.write(b"\0".join(x.encode() for x in cmdline) + b"\0")
            with open(os.path.join(d, "environ"), "wb") as f:
                f.write(b"\0".join(x.encode() for x in environ) + b"\0")
            with open(os.path.join(d, "status"), "w") as f:
                f.write("Name:\t%s\nVmHWM:\t%d kB\nVmRSS:\t%d kB\n" % (comm, hwm, rss))
        mk(101, "fastlivo_replay", ["/r/wt/h/ws/devel/lib/fast_livo/fastlivo_replay", "--arm", "T", "__name:=laserMapping",
                                    "__log:=/r/runs/eval/m/Grass01/T/s0/roslog/uuid/laserMapping-2.log"], [], 2048, 4096)
        mk(102, "fastlivo_replay", ["fastlivo_replay"], ["HOME=/x", "ROS_LOG_DIR=/r/runs/eval/m/Grass01/U/s1.retry1/roslog"], 10, 20)
        mk(103, "rosmaster", ["rosmaster", "__log:=/r/runs/eval/m/Grass01/T/s0/roslog/uuid/master.log"], [], 99999, 99999)
        mk(104, "fastlivo_replay", ["fastlivo_replay"], [], 5, 5)          # no run directory: ignored
        os.makedirs(os.path.join(proc, "self"))                          # non-numeric entries are skipped
        self.assertEqual(rm.replay_memory(proc), {"runs/eval/m/Grass01/T/s0": (2048, 4096),
                                                 "runs/eval/m/Grass01/U/s1.retry1": (10, 20)})
        self.assertEqual(rm.replay_memory(os.path.join(proc, "missing")), {})
        self.assertIsNone(rm.mem_available_gib(os.path.join(proc, "missing")))
        self.e.set_available_gib(17.5)
        self.assertAlmostEqual(rm.mem_available_gib(self.e.meminfo), 17.5)

    def test_defaults_stay_remote_and_local_profile_is_opt_in(self):
        """Remote defaults are unchanged. --profile local-dev replaces them only for flags absent from argv."""
        bare = rm.build_parser().parse_args(["plan", "--name", "p", "--arms", "U", "--seqs", "Grass01"])
        self.assertEqual(bare.jobs, 3)
        self.assertEqual(bare.min_avail_gb, 16.0)
        self.assertEqual(bare.brake_avail_gb, 6.0)
        self.assertEqual(bare.mem_limit_gb, 16.0)
        self.assertEqual(bare.docker_cmd, "sudo -n docker")
        self.assertEqual(bare.docker_user, "1000:1000")
        self.assertEqual(bare.extra_disk, [])
        self.assertIsNone(bare.profile)
        a = ["plan", "--profile", "local-dev", "--name", "p", "--arms", "U", "--seqs", "Grass01", "--jobs", "1"]
        args = rm.build_parser().parse_args(a)
        rm.apply_profile(args, a)
        self.assertEqual(args.jobs, 1)                       # explicit flag wins
        self.assertEqual(args.docker_cmd, "docker")
        self.assertEqual(args.docker_user, "%d:%d" % (os.getuid(), os.getgid()))
        self.assertEqual(args.min_avail_gb, 10.0)
        self.assertEqual(args.brake_avail_gb, 2.5)
        self.assertEqual(args.mem_limit_gb, 10.0)
        self.assertEqual(args.min_free_disk_gb, 8.0)
        self.assertEqual(args.extra_disk, ["/=5"])
        ev = rm.build_parser().parse_args(["plan", "--profile", "local-eval", "--name", "p", "--arms", "U", "--seqs", "Grass01"])
        rm.apply_profile(ev, ["plan", "--profile", "local-eval", "--name", "p", "--arms", "U", "--seqs", "Grass01"])
        self.assertEqual((ev.jobs, ev.min_avail_gb, ev.brake_avail_gb, ev.mem_limit_gb), (1, 21.5, 1.0, 22.0))
        self.assertEqual(rm.main(["plan", "--profile", "no-such-host", "--name", "p", "--arms", "U", "--seqs", "Grass01",
                                  "--root", self.e.root, "--devel", "head=headwt:ws/devel"]), 2)

    def test_local_dev_profile_launches_with_host_user_and_ten_gib_limit(self):
        a = self.e.args("--profile", "local-dev", "--name", "ld1", "--arms", "U", "--seqs", "Grass01", "--seeds", "0",
                        "--extra-disk", "/=0")
        self.assertEqual(rm.main(a), 0)
        argv = self.e.invocations()[0]["argv"]
        i = argv.index("--memory")
        self.assertEqual(argv[i:i + 4], ["--memory", "10240m", "--memory-swap", "10240m"])
        self.assertEqual(argv[argv.index("-u") + 1], "%d:%d" % (os.getuid(), os.getgid()))
        self.assertEqual(argv[argv.index("--cpus") + 1], "4")
        self.assertEqual(argv[argv.index("ionice") + 1], "-c3")
        m = self.e.manifest("ld1")
        self.assertEqual(m["jobs_max"], 2)
        self.assertEqual(m["profile"], "local-dev")
        self.assertEqual(m["docker_user"], "%d:%d" % (os.getuid(), os.getgid()))
        self.assertEqual(m["memory"]["min_avail_gib"], 10.0)
        self.assertEqual(m["memory"]["brake_avail_gib"], 2.5)
        self.assertEqual(m["memory"]["mem_limit_gib"], 10.0)
        self.assertEqual(m["disk"]["extra"], ["/=0"])          # explicit --extra-disk wins over the profile

    def test_build_root_is_where_devel_is_read_and_mounted(self):
        """exFAT project roots cannot hold catkin's symlinks; FL2G_BUILD_ROOT is the devel the host and the container share."""
        broot = os.path.join(self.e.root, "buildroot")
        devel = os.path.join(broot, "wt", "headwt", "ws", "devel")
        os.makedirs(devel)
        with open(os.path.join(devel, "setup.bash"), "w") as f:
            f.write("true\n")
        with open(os.path.join(devel, "BUILD_ID"), "w") as f:
            f.write("abc1234 0 eval-profile\n")
        # the copy under the project root is the empty mount point, not the build
        shutil.rmtree(os.path.join(self.e.root, "wt", "headwt", "ws", "devel"))
        os.makedirs(os.path.join(self.e.root, "wt", "headwt", "ws", "devel"))
        a = self.e.args("--name", "br1", "--arms", "U", "--seqs", "Grass01", "--seeds", "0", "--jobs", "1")
        with mock.patch.dict(os.environ, {"FL2G_BUILD_ROOT": broot}):
            self.assertEqual(rm.main(a), 0)
        argv = self.e.invocations()[0]["argv"]
        mount = "-v"
        pairs = [argv[i + 1] for i, x in enumerate(argv) if x == mount]
        self.assertIn("%s:/r/wt/headwt/ws/devel" % devel, pairs)
        self.assertEqual(argv[argv.index("-u") + 1], "1000:1000")


class Rolling(unittest.TestCase):
    def test_fetch_one_at_a_time_and_drop(self):
        e = Env(bags=())
        try:
            with mock.patch.dict(os.environ, {"FAKE_MAX_BAGS": "1"}):
                a = e.args("--name", "r1", "--arms", "U", "--seqs", "Grass01,Grass02", "--seeds", "0-1", "--jobs", "4",
                           "--fetch", "--max-bags", "1", "--drop-after",
                           "--fetch-cmd", "%s %s" % (PY, os.path.join(HERE, "fake_fetch.py")))
                self.assertEqual(rm.main(a), 0)
            calls = [ln.split()[:2] for ln in read(os.path.join(e.root, "fetch_calls.log")).splitlines()]
            self.assertEqual(calls, [["fetch", "Grass01"], ["drop", "Grass01"], ["fetch", "Grass02"], ["drop", "Grass02"]])
            self.assertFalse(os.path.exists(os.path.join(e.root, "fetch_violation")))
            m = e.manifest("r1")
            self.assertEqual(m["counts"]["done"], 4)
            self.assertTrue(all(s["dropped"] and s["fetched_by_us"] for s in m["sequences"].values()))
        finally:
            e.close()


class HostLidar(unittest.TestCase):
    """A76: --lidar mid360 selects the replay launch, the bag directory and the fetch command line; the default is the
    pre-A76 behaviour; the manifest records the lidar (eval_ate.py reads it)."""

    def test_default_is_avia_and_the_manifest_says_so(self):
        e = Env()
        try:
            self.assertEqual(rm.main(e.args("--name", "l0", "--arms", "U", "--seqs", "Grass01", "--seeds", "0")), 0)
            m = e.manifest("l0")
            self.assertEqual((m["lidar"], m["lidar_launch"], m["lidar_config"]),
                             ("avia", "m3dgr_avia_replay.launch", "config/m3dgr_avia.yaml"))
            self.assertEqual(m["sequences"]["Grass01"]["bag"], "data/m3dgr/filtered/Grass01.bag")
        finally:
            e.close()

    def test_mid360_replays_its_own_bag_with_its_own_launch(self):
        e = Env(bags=())
        try:
            fd = os.path.join(e.root, "data", "m3dgr", "filtered_mid360")
            os.makedirs(fd)
            with open(os.path.join(fd, "Grass01.bag"), "w") as f:
                f.write("bag")
            with open(os.path.join(fd, "Grass01.bag.sha256"), "w") as f:
                f.write("%s  Grass01.bag\n" % ("b" * 64))
            self.assertEqual(rm.main(e.args("--name", "l1", "--arms", "U", "--seqs", "Grass01", "--seeds", "0", "--lidar", "mid360")), 0)
            m = e.manifest("l1")
            self.assertEqual((m["lidar"], m["lidar_launch"], m["lidar_config"]),
                             ("mid360", "m3dgr_mid360_replay.launch", "config/m3dgr_mid360.yaml"))
            self.assertEqual(m["sequences"]["Grass01"]["bag"], "data/m3dgr/filtered_mid360/Grass01.bag")
            self.assertEqual(m["sequences"]["Grass01"]["bag_sha256"], "b" * 64)
            run = os.path.join(e.root, m["runs"][0]["attempts"][0]["run_dir"])
            launch = ET.parse(os.path.join(run, "replay.launch")).getroot()          # U always goes through the wrapper (A55)
            self.assertEqual(launch.find("include").get("file"), "$(find fast_livo)/launch/m3dgr_mid360_replay.launch")
            self.assertIn("config/m3dgr_mid360.yaml", read(os.path.join(run, "replay.launch")))
            inner = e.invocations()[0]["inner"]
            self.assertIn("bag:=/r/data/m3dgr/filtered_mid360/Grass01.bag", inner)
            self.assertNotIn("m3dgr_avia", inner)
            # the Avia bag directory is not consulted: a MID-360 matrix with only an Avia bag is a problem, not a silent fallback
            e2 = Env()
            try:
                self.assertEqual(rm.main(e2.args("--name", "l2", "--arms", "U", "--seqs", "Grass01", "--seeds", "0", "--lidar", "mid360")), 3)
            finally:
                e2.close()
        finally:
            e.close()

    def test_mid360_fetch_and_drop_carry_the_lidar_option(self):
        e = Env(bags=())
        try:
            a = e.args("--name", "l3", "--arms", "U", "--seqs", "Grass01", "--seeds", "0", "--lidar", "mid360", "--jobs", "2",
                       "--fetch", "--max-bags", "1", "--drop-after", "--fetch-cmd", "%s %s" % (PY, os.path.join(HERE, "fake_fetch.py")))
            self.assertEqual(rm.main(a), 0)
            calls = read(os.path.join(e.root, "fetch_calls.log")).splitlines()
            self.assertEqual([c.split()[:2] for c in calls], [["fetch", "Grass01"], ["drop", "Grass01"]])
            for c in calls:
                self.assertIn("--lidar mid360 ", c + " ")
            self.assertFalse(os.path.isdir(os.path.join(e.root, "data", "m3dgr", "filtered", "Grass01.bag")))
            # the Avia command line is what it was: no --lidar at all
            e3 = Env(bags=())
            try:
                a = e3.args("--name", "l4", "--arms", "U", "--seqs", "Grass01", "--seeds", "0", "--fetch", "--max-bags", "1",
                            "--fetch-cmd", "%s %s" % (PY, os.path.join(HERE, "fake_fetch.py")))
                self.assertEqual(rm.main(a), 0)
                self.assertNotIn("--lidar", read(os.path.join(e3.root, "fetch_calls.log")))
            finally:
                e3.close()
        finally:
            e.close()

    def test_wrapper_launch_defaults_are_the_avia_files(self):
        w = rm.wrapper_launch(["/r/a.yaml"])
        self.assertIn("launch/m3dgr_avia_replay.launch", w)
        self.assertIn("config/m3dgr_avia.yaml", w)
        w = rm.wrapper_launch(["/r/a.yaml"], "m3dgr_mid360_replay.launch", "m3dgr_mid360.yaml")
        self.assertEqual(ET.fromstring(w).find("include").get("file"), "$(find fast_livo)/launch/m3dgr_mid360_replay.launch")
        stock = ET.parse(os.path.join(_path.REPO, "launch", "m3dgr_mid360_replay.launch")).getroot()     # same arguments as the Avia one
        want = [(x.get("name"), x.get("default")) for x in stock.findall("arg")]
        self.assertEqual([(x.get("name"), x.get("default")) for x in ET.fromstring(w).findall("arg")], want)


class Pieces(unittest.TestCase):
    def test_wrapper_declares_exactly_the_stock_launch_args(self):
        stock = ET.parse(os.path.join(_path.REPO, "launch", "m3dgr_avia_replay.launch")).getroot()
        want = [(x.get("name"), x.get("default")) for x in stock.findall("arg")]
        got = [(x.get("name"), x.get("default")) for x in ET.fromstring(rm.wrapper_launch(["/r/a.yaml"])).findall("arg")]
        self.assertEqual(got, want)

    def test_slots(self):
        root = tempfile.mkdtemp()
        try:
            fds = []
            for i in range(rm.N_SLOTS):
                s, fd = rm.try_acquire_slot(root)
                self.assertEqual(s, i)
                fds.append(fd)
            self.assertEqual(rm.try_acquire_slot(root), (None, None))
            self.assertEqual(os.path.basename(rm.slot_paths(root)[0]), ".replay.lock")   # slot 0 = replay_run.sh's lock
            os.close(fds[3])
            s, fd = rm.try_acquire_slot(root)
            self.assertEqual(s, 3)
            for f in fds[:3] + fds[4:] + [fd]:
                os.close(f)
        finally:
            shutil.rmtree(root)

    def test_console_verdict(self):
        self.assertEqual(ec.console_verdict("x\nfastlivo_replay: done\n[a] process has finished cleanly\n"), ("0", 1))
        self.assertEqual(ec.console_verdict("[a] process has died [pid 3, exit code -11, cmd"), ("-11", 0))
        self.assertEqual(ec.console_verdict("process has finished cleanly\nprocess has died [pid 3, exit code 2,"), ("2", 0))
        self.assertEqual(ec.console_verdict(""), (None, 0))

    def test_seq_overlay_is_yaml(self):
        import yaml
        d = yaml.safe_load(rm.seq_overlay_text("Grass01", ["/r/data/brdc/a.rnx", "/r/data/brdc/b.rnx"], "/r/audit/x.yaml"))
        self.assertEqual(d, {"gnss": {"nav_rinex": ["/r/data/brdc/a.rnx", "/r/data/brdc/b.rnx"], "time": {"calib_file": "/r/audit/x.yaml"}}})
        d = yaml.safe_load(rm.seq_overlay_text("Grass01", ["/r/a"], None))
        self.assertEqual(d, {"gnss": {"nav_rinex": ["/r/a"]}})


if __name__ == "__main__":
    unittest.main()
