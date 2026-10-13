#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""One-key per-sequence preprocessing before the evaluation (P4-TIME): the U-arm replay, the DESIGN §7 time
calibration and the TEST_PLAN §4 GT audit of one sequence, written into the archive audit/<seq>/ that
tools/eval/run_matrix.py (gnss.time.calib_file) and tools/eval/eval_ate.py (gt_audited.tum) read.

    python3 tools/time_calib/preprocess_seq.py <seq> --devel <worktree>:<devel dir> [--evaluation-phase] [...]

Runs on the remote host from a synced tree (e.g. through tools/eval/launch_remote.sh ... start preproc-<seq> -- ...):
  1. A2 guard: an evaluation-set (or unknown) sequence needs --evaluation-phase (tools/eval/evalcommon.check_phase).
     Nothing here computes or prints an accuracy: the time calibration compares the replay's speed with the GNSS
     Doppler speed, the GT audit compares the GT with raw sensor topics (its replay route P4c is used on development
     sequences only), and the summary holds calibration values, not errors;
  2. the U-arm replay (AMENDMENTS A55: the HEAD eval build, gnss.enable false, seed 0) with tools/eval/run_matrix.py
     (matrix preproc_<seq>; an existing complete run is reused);
  3. in the project image, under the host build lock (as tools/dev/remote_tool.sh): gnss_time_calib.py without the
     replay (method 1 -> time_calib/<seq>_record_time.yaml) and with it (methods 1 + 2 ->
     time_calib/<seq>.yaml, AMENDMENTS A53), then gt_audit.py (-> gt/, AMENDMENTS A54: gt_timebase.yaml,
     gt_audited.tum, trust_mask.csv, gt_mask_intervals.csv);
  4. audit/<seq>/preprocess.json: provenance (source id, replay run and build_sha256, inputs) and the resulting
     calibration values. Earlier archive files are moved to audit/<seq>/.previous/<UTC time>/ first.
Inputs: the bag (default data/m3dgr/filtered/<seq>.bag, else data/m3dgr/bags/<seq>.bag; the GT audit needs the
RTK fix and /odom topics, which fetch_eval keeps in the filtered bag), data/m3dgr/gt/<seq>.txt, the IGS BRDC of the
bag's UTC day(s) (the fetch record's list, else by the registry date). Owner: WP-g. Host side: Python >= 3.8, stdlib.
"""
import argparse
import datetime
import fcntl
import json
import os
import shlex
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
REPO = os.path.dirname(TOOLS)
sys.path.insert(0, os.path.join(TOOLS, "eval"))

import evalcommon as ec  # noqa: E402

EXIT_BLOCKED = 3


def build_parser():
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0], formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("seq")
    p.add_argument("--devel", required=True, help="<worktree>:<devel dir> of the HEAD eval build for the U arm (A55)")
    p.add_argument("--evaluation-phase", action="store_true", help="A2: required for evaluation-set sequences")
    p.add_argument("--bag", default="", help="bag relative to the root (default data/m3dgr/filtered/<seq>.bag, else data/m3dgr/bags/<seq>.bag)")
    p.add_argument("--gt", default="", help="GT file relative to the root (default data/m3dgr/gt/<seq>.txt)")
    p.add_argument("--replay-dir", default="", help="use this existing complete U-arm replay (relative to the root) instead of running one")
    p.add_argument("--skip-gt", action="store_true", help="time calibration only")
    p.add_argument("--root", default=None)
    p.add_argument("--image", default=ec.IMAGE_DEFAULT)
    p.add_argument("--docker-cmd", default="sudo -n docker", help=argparse.SUPPRESS)
    p.add_argument("--docker-user", default="1000:1000", help=argparse.SUPPRESS)
    p.add_argument("--profile", default="", help="forwarded to run_matrix.py (tools/eval/host_profiles.json)")
    p.add_argument("--dry-run", action="store_true", help="print the plan and the commands, run nothing")
    return p


class Pre:
    def __init__(self, a):
        self.a = a
        self.root = ec.project_root(a.root)
        self.seq = a.seq
        self.reg, self.seqs = ec.load_sequences()
        self.cmds = []

    def p(self, *rel):
        return os.path.join(self.root, *rel)

    def rel_in_container(self, path):
        return "/r/" + os.path.relpath(os.path.realpath(path), os.path.realpath(self.root))

    # ---------------------------------------------------------------- inputs
    def inputs(self):
        s = self.seq
        if s not in self.seqs:
            raise ec.Blocked("%s is not in tools/eval/sequences.json" % s)
        cls = ec.check_phase(s, self.a.evaluation_phase, "the time calibration and GT audit (they replay the sequence)")
        bag = self.a.bag
        if not bag:
            for c in ("data/m3dgr/filtered/%s.bag" % s, "data/m3dgr/bags/%s.bag" % s):
                if os.path.isfile(self.p(c)):
                    bag = c
                    break
        if not bag or not os.path.isfile(self.p(bag)):
            raise ec.Blocked("%s: no bag (fetch it with tools/eval/fetch_eval.sh)" % s)
        gt = self.a.gt or "data/m3dgr/gt/%s.txt" % s
        if not self.a.skip_gt and self.seqs[s].get("gt_kind") == "rtk" and not os.path.isfile(self.p(gt)):
            raise ec.Blocked("%s: GT file %s missing" % (s, gt))
        nav = []
        rec = self.p("data", "m3dgr", "filtered", s + ".fetch.json")
        if os.path.isfile(rec):
            nav = [x for x in (ec.load_json(rec).get("brdc") or []) if os.path.isfile(self.p(x))]
        if not nav:
            d = datetime.date.fromisoformat(self.seqs[s]["date_local"])
            for dd in (d - datetime.timedelta(days=1), d):        # local UTC+8 date: the UTC day is the same or the day before
                f = "data/brdc/BRDC00IGS_R_%d%03d0000_01D_MN.rnx" % (dd.year, dd.timetuple().tm_yday)
                if os.path.isfile(self.p(f)):
                    nav.append(f)
        if not nav:
            raise ec.Blocked("%s: no IGS BRDC in data/brdc for its day (fetch_eval downloads it)" % s)
        wt, _, dv = self.a.devel.partition(":")
        ec.safe_name(wt, "worktree")
        if not dv or not os.path.isfile(self.p("wt", wt, dv, "setup.bash")):
            raise ec.Blocked("--devel %s: %s missing (build the HEAD eval tree first)" % (self.a.devel, self.p("wt", wt, dv, "setup.bash")))
        self.cls, self.bag, self.gt, self.nav, self.wt, self.dv = cls, bag, gt, nav, wt, dv
        return {"class": cls, "bag": bag, "gt": gt, "nav_rinex": nav, "devel": self.a.devel}

    # ---------------------------------------------------------------- replay
    def replay(self):
        if self.a.replay_dir:
            d = self.a.replay_dir
            st = self.p(d, "run_status.json")
            if not (os.path.isfile(st) and int(ec.load_json(st).get("complete", 0)) == 1):
                raise ec.Blocked("--replay-dir %s is not a complete run" % d)
            return d
        name = "preproc_%s" % self.seq.replace("-", "_")
        cmd = [sys.executable, os.path.join(TOOLS, "eval", "run_matrix.py"), "run", "--name", name, "--arms", "U", "--seqs", self.seq,
               "--seeds", "0", "--devel", "U=%s" % self.a.devel, "--bag", "%s=%s" % (self.seq, self.bag), "--jobs", "1",
               "--root", self.root, "--image", self.a.image]
        if self.cls != "development":
            cmd.append("--evaluation-phase")
        if self.a.profile:
            cmd += ["--profile", self.a.profile]
        cmd += ["--docker-cmd", self.a.docker_cmd, "--docker-user", self.a.docker_user]
        self.cmds.append(" ".join(shlex.quote(c) for c in cmd))
        if self.a.dry_run:
            return "runs/eval/%s/%s/U/s0" % (name, self.seq)
        r = subprocess.run(cmd)
        man = self.p("runs", "eval", name, "matrix_manifest.json")
        if not os.path.isfile(man):
            raise ec.Blocked("run_matrix wrote no manifest (rc %d)" % r.returncode)
        for run in ec.load_json(man)["runs"]:
            if run["seq"] == self.seq and run["arm"] == "U" and run["seed"] == 0:
                done = [x for x in run["attempts"] if x.get("complete")]
                if done:
                    return done[-1]["run_dir"]
        raise ec.Blocked("the U-arm replay of %s did not complete (runs/eval/%s)" % (self.seq, name))

    # ---------------------------------------------------------------- container steps
    def container(self, script, log):
        cmd = self.a.docker_cmd.split() + ["run", "--rm", "--init", "-u", self.a.docker_user, "-v", "%s:/r" % self.root,
                                           "-e", "HOME=/tmp", "-e", "MPLCONFIGDIR=/tmp", "-e", "OMP_NUM_THREADS=1",
                                           "-e", "OPENBLAS_NUM_THREADS=1", "-e", "PYTHONDONTWRITEBYTECODE=1",
                                           "-w", self.rel_in_container(REPO), self.a.image,
                                           "nice", "-n", "19", "ionice", "-c3", "bash", "-c", "source /opt/ros/noetic/setup.bash; " + script]
        self.cmds.append(script)
        if self.a.dry_run:
            return 0
        with open(os.path.join(self.adir, log), "w") as lf, open(self.p(".build.lock"), "a") as lk:
            fcntl.flock(lk, fcntl.LOCK_EX)            # serialised with builds and remote_tool.sh runs (DEVELOPING §2)
            r = subprocess.run(cmd, stdout=lf, stderr=subprocess.STDOUT)
        return r.returncode

    def archive_previous(self):
        """Move earlier outputs of this tool chain to audit/<seq>/.previous/<UTC>/ (never overwrite silently)."""
        stamp = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        moved = []
        for sub in ("time_calib", "gt", "preprocess.json"):
            src = os.path.join(self.adir, sub)
            if os.path.exists(src):
                dst = os.path.join(self.adir, ".previous", stamp, sub)
                moved.append(os.path.relpath(dst, self.root))
                if not self.a.dry_run:
                    os.makedirs(os.path.dirname(dst), exist_ok=True)
                    shutil.move(src, dst)
        return moved

    def run(self):
        s = self.seq
        info = self.inputs()
        self.adir = self.p("audit", s)
        replay = self.replay()
        rq = lambda x: shlex.quote("/r/" + x)
        nav = " ".join("--nav-rinex %s" % rq(x) for x in self.nav)
        tc_dir = "/r/audit/%s/time_calib" % s
        moved = self.archive_previous()
        if not self.a.dry_run:
            os.makedirs(os.path.join(self.adir, "time_calib"), exist_ok=True)
            os.makedirs(os.path.join(self.adir, "gt"), exist_ok=True)
        rc = {}
        rc["time_calib_method1"] = self.container(
            "python3 tools/time_calib/gnss_time_calib.py %s - %s/%s_record_time.yaml --seq %s --plot" % (rq(self.bag), tc_dir, s, shlex.quote(s)),
            "time_calib/%s_record_time.log" % s)
        rc["time_calib"] = self.container(
            "python3 tools/time_calib/gnss_time_calib.py %s %s %s/%s.yaml --seq %s %s --plot" % (rq(self.bag), rq(replay), tc_dir, s, shlex.quote(s), nav),
            "time_calib/%s.log" % s)
        if not self.a.skip_gt and self.seqs[s].get("gt_kind") == "rtk":
            extra = " --replay-dir %s" % rq(replay) if self.cls == "development" else ""     # P4c: development set only (A2)
            rc["gt_audit"] = self.container(
                "python3 tools/gt_audit.py %s %s /r/audit/%s/gt --seq %s --archive-label %s%s" % (
                    rq(self.gt), rq(self.bag), s, shlex.quote(s), shlex.quote(os.path.join(ec.DEFAULT_ROOT, "audit", s, "gt")), extra),
                "gt/gt_audit.log")
        out = {"tool": "preprocess_seq.py", "sequence": s, "utc": ec.utc_now(), "source": source_id(), "inputs": info,
               "replay_dir": replay, "archived_previous": moved, "return_codes": rc, "commands": self.cmds}
        if self.a.dry_run:
            print(json.dumps(out, indent=1))
            return 0
        out.update(summarise(self.root, s, replay))
        ec.atomic_write_json(os.path.join(self.adir, "preprocess.json"), out)
        print(summary_line(out))
        return 0 if all(v == 0 for v in rc.values()) else 1


def source_id():
    try:
        r = subprocess.run(["git", "-C", REPO, "describe", "--always", "--dirty"], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                           universal_newlines=True, timeout=10)
        if r.stdout.strip():
            return r.stdout.strip()
    except Exception:  # noqa: BLE001
        pass
    p = os.path.join(TOOLS, "eval", ".source_id")
    if os.path.isfile(p):
        with open(p) as f:
            return f.read().strip()
    return "unknown"


def summarise(root, seq, replay):
    """Calibration values of the archive (no accuracy): time-calibration segments, GT time base, masked durations."""
    out = {}
    st = os.path.join(root, replay, "replay_stats.json")
    if os.path.isfile(st):
        r = ec.load_json(st)
        out["replay"] = {k: r.get(k) for k in ("build_sha256", "gnss_enable", "cycles", "complete")}
    tc = os.path.join(root, "audit", seq, "time_calib", "%s.json" % seq)
    if os.path.isfile(tc):
        r = ec.load_json(tc)
        out["time_calib"] = {"status": r.get("method2_status"),
                             "segments": [{k: sg.get(k) for k in ("t_from_gpst_s", "offset_s", "drift_ppm", "sigma_toff_s", "tdcp_paused",
                                                                   "method", "peak_corr", "n_windows")} for sg in r.get("segments") or []],
                             "method1_prior_offset_s": (r.get("method1_estimator_clock_prior") or {}).get("offset_s")}
    gj = os.path.join(root, "audit", seq, "gt", "gt_audit.json")
    if os.path.isfile(gj):
        r = ec.load_json(gj)
        tm = r.get("trust_mask", {})
        out["gt"] = {"verdict": r.get("verdict"), "timebase": r.get("timebase"), "masked_s": tm.get("masked_s"),
                     "rtk_plateaus": len((tm.get("rtk") or {}).get("plateaus", [])),
                     "rtk_unpaired": len((tm.get("rtk") or {}).get("unpaired", []))}
    return out


def summary_line(out):
    tc = out.get("time_calib", {})
    segs = tc.get("segments") or []
    g = out.get("gt", {})
    tb = g.get("timebase") or {}
    ms = g.get("masked_s") or {}
    return ("preprocess %s: time calibration %s (%s); GT time base %s %s ms (applied %s ms, %s); ATE-excluded %.1f s, RTK plateaus %s" % (
        out["sequence"],
        "; ".join("%s %+.1f ms sigma %.1f ms%s" % (sg["method"], 1e3 * sg["offset_s"], 1e3 * sg["sigma_toff_s"], " TDCP paused" if sg["tdcp_paused"] else "")
                  for sg in segs) or "none", tc.get("status"),
        tb.get("method"), "%+.1f" % (1e3 * tb["offset_s"]) if tb.get("offset_s") is not None else "—",
        "%+.1f" % (1e3 * tb["applied_shift_s"]) if tb.get("applied_shift_s") is not None else "—", tb.get("confidence"),
        ms.get("ate_exclude", float("nan")), g.get("rtk_plateaus")))


def main(argv=None):
    a = build_parser().parse_args(argv)
    try:
        return Pre(a).run()
    except (ec.PhaseError, ec.Blocked, ValueError) as e:
        print("preprocess_seq: BLOCKED: %s" % e, file=sys.stderr)
        return EXIT_BLOCKED


if __name__ == "__main__":
    sys.exit(main())
