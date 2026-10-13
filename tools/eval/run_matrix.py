#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Batch offline replays for the evaluation (P3-D, P4-EVAL): arms (A46) x sequences x seeds (A30, k = 0..4).

    run_matrix.py plan   --name N --arms U,T --seqs Grass01 [--seeds 0-4] --devel head=<wt>:<devel> [...]
    run_matrix.py run    (same arguments)            replays; resumable (a finished run is never repeated)
    run_matrix.py status --name N                    counts only (no trajectory values, A2)

Every run follows the conventions of docs/dev/ref_r_gates/replay_run.sh, reproduced here because that runner holds
one host-wide lock (one replay at a time, the A7 rule) whereas A41 allows up to 8 concurrent single-threaded
offline replays:
  * one container per run (`docker run --rm -u 1000:1000` by default, `--docker-user` / a host profile can
    select another uid, `nice -n 19` inside, OMP_NUM_THREADS=1, FL2G_HOST),
    ROS_LOG_DIR/ROS_HOME and the upstream Log/ directory bound into the run directory (ROOT_DIR is compiled in),
    a private roscore (each container has its own network namespace), BUILD_ID + image digest + bag sha256 passed
    to the manifest, a run directory that is never reused;
  * completion = the same four conditions: replay_stats.json says "complete": true, the console holds
    "fastlivo_replay: done", the node's own exit code is 0, and docs/dev/ref_r_gates/manifest_check.py --run passes;
    run_status.json/runner.log/console.log are written in the same format (hash_runs.sh reads them);
  * concurrency: at most --jobs containers of this runner, default 3 (README "内存": on 2026-09-25 eight concurrent
    Occlusion04 replays exhausted the 62 GB host near the end of the bag and froze it); a run holds one of 8
    host-wide slot locks for its whole life (slot 0 IS replay_run.sh's $ROOT/.replay.lock, slots 1..7 are
    $ROOT/.replay.slot<i>.lock), so this runner, other instances of it and replay_run.sh together never exceed 8
    replays. The lock is an open file description inherited by the docker client: it is released only when the
    container's client exits;
  * memory (same README section): a replay is launched only while the host's MemAvailable (the `available` column of
    `free -g`) is >= --min-avail-gb (16 GiB), otherwise the runner waits; each container gets --memory = --memory-swap
    = --mem-limit-gb (a runaway replay dies inside its own cgroup instead of taking the host down) and
    --oom-score-adj (a global OOM picks a replay, never sshd, the docker clients or another project's training);
    below --brake-avail-gb while running, the youngest replay is killed and queued again (a memory brake does not
    use up --retries). Every attempt records the peak RSS of its fastlivo_replay process (VmHWM, sampled every
    --mem-sample-s) and the lowest MemAvailable seen while it ran;
  * disk: no launch while the root's file system has less than --min-free-disk-gb free; --gzip-after compresses the
    named bulky outputs of a complete run in place (<file> -> <file>.gz, after the completion check has read them);
  * a run that is not complete is retried once (--retries) in a fresh directory <run>.retry<n>; every attempt is
    kept and recorded;
  * arms with an overlay (arms.json; P3-B's config/arms/<arm>.yaml of the build that runs the arm) and arms that
    need the per-sequence GNSS inputs (gnss.nav_rinex, A38/A45; gnss.time.calib_file, A18) run a wrapper launch
    written into the run directory: it includes launch/m3dgr_avia_replay.launch with pass_all_args and loads the
    overlays after it, so their keys win (roslaunch keeps the last setting of a parameter; yaml dicts are unrolled
    to leaves). An arm with neither would use the stock launch unchanged (none since A55: U loads config/arms/U.yaml);
    A81: an arm registered with "time_calib": false in arms.json gets the nav_rinex overlay without gnss.time.calib_file
    (overlays/<seq>.gnss_inputs_nocalib.yaml); the manifest's arms[arm].time_calib and sequences[seq].gnss_inputs_nocalib
    say so, every other arm is unchanged;
  * one binary per matrix (A55): U runs on the HEAD eval build with gnss.enable false (G-zero-channel: bit-identical
    to ref-R), the same build as the GNSS arms, so that seeds (A30) mean the same for every arm; arms resolving to
    different --devel builds are refused unless --allow-mixed-builds (diagnostics only; eval_ate.py flags such pairs).
Host LiDAR (--lidar avia|mid360, A76; profiles in tools/eval/lidars.json): selects the replay launch
(launch/m3dgr_<lidar>_replay.launch, which loads config/m3dgr_<lidar>.yaml before the arm overlays), the default bag
directory (data/m3dgr/filtered for Avia, data/m3dgr/filtered_mid360 for the MID-360; --bag still overrides one sequence) and
what --fetch/--drop-after hand to fetch_eval.sh (--lidar, so the MID-360 bags are filtered with the MID-360 topics into their own
directory). The manifest records `lidar`; eval_ate.py reads it to pick the lever arm of the calibrated-lever scoring. Default
avia: the launch, the bag paths and the fetch command lines are exactly what they were before A76.

Rolling fetch (--fetch): sequences whose bag is missing are fetched one at a time with tools/eval/fetch_eval.sh while
other sequences replay; at most --max-bags filtered bags exist at once (fetched + fetching); with --drop-after a
sequence's filtered bag is deleted once all its runs are final. A fetch that ends with EX_TEMPFAIL (disk) is retried
after the next drop.
The manifest runs/eval/<name>/matrix_manifest.json is rewritten atomically after every event.
A2: evaluation-set sequences need --evaluation-phase. Host side: Python >= 3.8, stdlib only.
Host profiles (tools/eval/host_profiles.json, `--profile`): named overrides of the remote defaults for a machine
that is not the 62 GB build host. Explicit flags win. The defaults themselves (sudo -n docker, jobs 3, 16 GiB
gates, uid 1000:1000) stay what the remote README states. When FL2G_BUILD_ROOT is set and differs from the
project root, devel/build trees are read from there and bind-mounted onto the same /r/wt/... paths (exFAT
project roots cannot store the symlinks catkin puts in devel).
"""
import argparse
import fcntl
import json
import os
import re
import shlex
import signal
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import evalcommon as ec  # noqa: E402

N_SLOTS = ec.MAX_CONCURRENT_REPLAYS
JOBS_DEFAULT = 3                 # README "内存": the 2026-09-25 incident (8 concurrent replays froze the 62 GB host)
MIN_AVAIL_GIB = 16.0             # launch gate on MemAvailable
BRAKE_AVAIL_GIB = 6.0            # kill the youngest replay below this while running
MEM_LIMIT_GIB = 16.0             # docker --memory/--memory-swap per replay (README: measured peak RSS and margin)
OOM_SCORE_ADJ = 1000             # replays are the first victims of a global OOM
MAX_BRAKES_PER_RUN = 5           # a run the brake killed this often is given up (termination guarantee)
MIN_FREE_DISK_GIB = 8.0          # launch gate on the root's file system (the host's /data is shared with other projects)
ROS_PORT = 11411
EX_TEMPFAIL = 75
STOCK_LAUNCH_ARGS = (("bag", None), ("output_dir", None), ("start_offset", "0.0"), ("seed", "0"), ("duration", "-1.0"),
                     ("seq", "m3dgr"), ("arm", "U"), ("git_describe", "unknown"), ("git_dirty", "unknown"),
                     ("image_digest", ""), ("bag_sha256", ""), ("prefix", ""), ("extra_args", ""))
RESERVED_OVERLAY_KEYS = ("replay", "output")     # the runner's contract with the stock launch; an overlay may not touch them


# ---------------------------------------------------------------------------------------------------------------
def slot_paths(root):
    return [os.path.join(root, ".replay.lock")] + [os.path.join(root, ".replay.slot%d.lock" % i) for i in range(1, N_SLOTS)]


def try_acquire_slot(root):
    """-> (slot index, fd) of a free host-wide replay slot, or (None, None)."""
    for i, p in enumerate(slot_paths(root)):
        fd = os.open(p, os.O_RDWR | os.O_CREAT, 0o664)
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            return i, fd
        except OSError:
            os.close(fd)
    return None, None


def overlay_top_keys(path):
    """Top-level keys of a yaml overlay (PyYAML when present, else the unindented 'key:' lines)."""
    with open(path, encoding="utf-8") as f:
        text = f.read()
    try:
        import yaml  # noqa: F401
        d = yaml.safe_load(text) or {}
        if not isinstance(d, dict):
            raise ValueError("%s is not a mapping" % path)
        return sorted(d.keys())
    except ImportError:
        keys = []
        for ln in text.splitlines():
            if ln and not ln[0].isspace() and not ln.startswith("#") and ":" in ln:
                keys.append(ln.split(":", 1)[0].strip())
        return sorted(set(keys))


def wrapper_launch(overlays_in_container, launch="m3dgr_avia_replay.launch", config="m3dgr_avia.yaml"):
    """The per-run launch: stock replay launch + overlays loaded after it (later settings of a key win)."""
    L = ["<launch>",
         "  <!-- generated by tools/eval/run_matrix.py: launch/%s + overlays (loaded after" % launch,
         "       config/%s, so their keys win; roslaunch unrolls yaml dicts into leaf parameters) -->" % config]
    for name, default in STOCK_LAUNCH_ARGS:
        L.append('  <arg name="%s"%s />' % (name, "" if default is None else ' default="%s"' % default))
    L.append('  <include file="$(find fast_livo)/launch/%s" pass_all_args="true" />' % launch)
    for p in overlays_in_container:
        L.append('  <rosparam command="load" file="%s" />' % p)
    L.append("</launch>")
    return "\n".join(L) + "\n"


def seq_overlay_text(seq, nav_rinex, calib_file):
    """Per-sequence GNSS inputs (paths inside the container)."""
    L = ["# generated by tools/eval/run_matrix.py for %s: per-sequence GNSS inputs (A38/A45 nav_rinex, A18 time calibration)" % seq,
         "gnss:",
         "  nav_rinex: [%s]" % ", ".join('"%s"' % x for x in nav_rinex)]
    if calib_file:
        L += ["  time:", '    calib_file: "%s"' % calib_file]
    return "\n".join(L) + "\n"


def run_dir_rel(name, seq, arm, seed, attempt):
    base = "runs/eval/%s/%s/%s/s%d" % (name, seq, arm, seed)
    return base if attempt == 1 else "%s.retry%d" % (base, attempt - 1)


def brakes(job):
    """Attempts of a job that the memory brake killed."""
    return sum(1 for at in job["attempts"] if at.get("memory_brake"))


def counted_attempts(job):
    """Attempts that use up --retries: every launched attempt except the ones the memory brake killed."""
    return sum(1 for at in job["attempts"] if at.get("attempt", 0) > 0 and not at.get("memory_brake"))


def read_first_token(path):
    with open(path) as f:
        return f.read().split()[0]


def json_hash_field(path, key):
    """A hash-valued string field of a run's json file (never a trajectory value)."""
    try:
        with open(path) as f:
            v = json.load(f).get(key)
        return v if isinstance(v, str) else None
    except (OSError, ValueError):
        return None


# ---------------------------------------------------------------------------------------------------------------
# memory (README "内存")
def mem_available_gib(path="/proc/meminfo"):
    """MemAvailable in GiB (what `free -g` prints as available, before its rounding), or None when unreadable."""
    try:
        with open(path) as f:
            for ln in f:
                if ln.startswith("MemAvailable:"):
                    return int(ln.split()[1]) / (1024.0 * 1024.0)
    except (OSError, ValueError, IndexError):
        return None
    return None


_RUN_OF_NODE = re.compile(r"/r/(runs/eval/[^\s\x00]+?)/roslog(?:/|\x00|$)")


def replay_memory(proc_root="/proc"):
    """-> {run directory relative to the root: (VmRSS KiB, VmHWM KiB)} of the running fastlivo_replay processes. The run
    directory comes from the node's ROS log argument (__log:=/r/<run>/roslog/...) or its ROS_LOG_DIR (/r/<run>/roslog);
    VmHWM is the kernel's peak resident set of the process. Processes that vanish while being read are skipped."""
    out = {}
    try:
        pids = [p for p in os.listdir(proc_root) if p.isdigit()]
    except OSError:
        return out
    for pid in pids:
        base = os.path.join(proc_root, pid)
        try:
            with open(os.path.join(base, "comm")) as f:
                if f.read().strip() != "fastlivo_replay":
                    continue
            with open(os.path.join(base, "cmdline"), "rb") as f:
                text = f.read().decode("utf-8", "replace")
            m = _RUN_OF_NODE.search(text)
            if not m:
                with open(os.path.join(base, "environ"), "rb") as f:
                    m = _RUN_OF_NODE.search(f.read().decode("utf-8", "replace"))
            if not m:
                continue
            rss = hwm = None
            with open(os.path.join(base, "status")) as f:
                for ln in f:
                    if ln.startswith("VmRSS:"):
                        rss = int(ln.split()[1])
                    elif ln.startswith("VmHWM:"):
                        hwm = int(ln.split()[1])
        except (OSError, ValueError, IndexError):
            continue
        if hwm is not None:
            prev = out.get(m.group(1))
            if prev is None or hwm > prev[1]:
                out[m.group(1)] = (rss or 0, hwm)
    return out


def free_disk_gib(path):
    st = os.statvfs(path)
    return st.f_bavail * st.f_frsize / (1024.0 ** 3)


_BUILD_REL = ("build", "devel", "build_vg", "devel_vg")
_CORE_REL = ("build_core", "build_core_vg")


def _split_rel(rel):
    parts = []
    for piece in rel:
        parts.extend([p for p in str(piece).split("/") if p])
    return parts


def _is_build_tree(parts):
    """wt/<name>/build_core[...] or wt/<name>/ws/{build,devel}[...] — the trees that may live off the project root."""
    if len(parts) < 3 or parts[0] != "wt" or not re.match(r"^[A-Za-z0-9_-]+$", parts[1]):
        return False
    if parts[2] in _CORE_REL:
        return True
    return len(parts) >= 4 and parts[2] == "ws" and parts[3] in _BUILD_REL


def host_project_path(root, *rel):
    """Join rel onto the project root, except build/devel trees when FL2G_BUILD_ROOT points somewhere else."""
    parts = _split_rel(rel)
    broot = os.environ.get("FL2G_BUILD_ROOT")
    if broot and os.path.abspath(broot) != os.path.abspath(root) and _is_build_tree(parts):
        return os.path.join(broot, *parts)
    return os.path.join(root, *parts) if parts else root


def build_volume_args(root):
    """`-v` arguments: the project root at /r, plus each build tree from FL2G_BUILD_ROOT over /r/wt/..."""
    args = ["-v", "%s:/r" % root]
    broot = os.environ.get("FL2G_BUILD_ROOT")
    if not broot or os.path.abspath(broot) == os.path.abspath(root):
        return args
    wt_root = os.path.join(broot, "wt")
    if not os.path.isdir(wt_root):
        return args
    for name in sorted(os.listdir(wt_root)):
        if not re.match(r"^[A-Za-z0-9_-]+$", name):
            continue
        for rel in ("build_core", "build_core_vg", "ws/build", "ws/devel", "ws/build_vg", "ws/devel_vg"):
            host = os.path.join(wt_root, name, *rel.split("/"))
            if not os.path.isdir(host):
                continue
            os.makedirs(os.path.join(root, "wt", name, *rel.split("/")), exist_ok=True)
            args += ["-v", "%s:/r/wt/%s/%s" % (host, name, rel)]
    return args


def flag_in_argv(argv, flag):
    for a in argv:
        if a == flag or a.startswith(flag + "="):
            return True
    return False


def resolve_docker_user(spec):
    if spec == "host":
        return "%d:%d" % (os.getuid(), os.getgid())
    if not re.match(r"^[0-9]+:[0-9]+$", spec or ""):
        raise ValueError("--docker-user must be UID:GID or 'host', not %r" % spec)
    return spec


def load_profile(name):
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "host_profiles.json")
    with open(path, encoding="utf-8") as f:
        data = json.load(f)
    profiles = data.get("profiles") or {}
    if name not in profiles:
        raise ValueError("unknown --profile %s (known: %s)" % (name, ", ".join(sorted(profiles)) or "none"))
    return profiles[name]


# Profile keys that replace the program defaults only when the matching flag is absent from argv.
_PROFILE_FLAGS = (
    ("--jobs", "jobs", int),
    ("--min-avail-gb", "min_avail_gb", float),
    ("--brake-avail-gb", "brake_avail_gb", float),
    ("--mem-limit-gb", "mem_limit_gb", float),
    ("--oom-score-adj", "oom_score_adj", int),
    ("--cpus", "cpus", float),
    ("--min-free-disk-gb", "min_free_disk_gb", float),
    ("--docker-cmd", "docker_cmd", None),
    ("--docker-user", "docker_user", None),
    ("--image", "image", None),
)


def apply_profile(args, argv):
    """Apply tools/eval/host_profiles.json. Explicit argv flags win. docker_user 'host' becomes the current uid:gid."""
    if argv is None:
        argv = sys.argv[1:]
    if getattr(args, "profile", None):
        prof = load_profile(args.profile)
        for flag, attr, cast in _PROFILE_FLAGS:
            if attr in prof and not flag_in_argv(argv, flag):
                setattr(args, attr, prof[attr] if cast is None else cast(prof[attr]))
        if "extra_disk" in prof and not flag_in_argv(argv, "--extra-disk"):
            args.extra_disk = list(prof["extra_disk"])
    if hasattr(args, "docker_user"):
        args.docker_user = resolve_docker_user(args.docker_user)
    return args


def gzip_in_place(path):
    """<path> -> <path>.gz (level 6, streamed); the original is removed only once the .gz is complete."""
    import gzip
    import shutil
    tmp = path + ".gz.tmp"
    with open(path, "rb") as fi, open(tmp, "wb") as raw:
        with gzip.GzipFile(filename=os.path.basename(path), mode="wb", fileobj=raw, compresslevel=6,
                           mtime=int(os.stat(path).st_mtime)) as fo:
            shutil.copyfileobj(fi, fo, 1 << 20)
    os.replace(tmp, path + ".gz")
    os.remove(path)


# ---------------------------------------------------------------------------------------------------------------
class Matrix:
    def __init__(self, args):
        self.a = args
        self.root = ec.project_root(args.root)
        self.name = ec.safe_name(args.name, "matrix name")
        self.mdir_rel = "runs/eval/%s" % self.name
        self.mdir = os.path.join(self.root, self.mdir_rel)
        self.arms_reg = ec.load_arms()
        self.lidar = ec.lidar_profile(getattr(args, "lidar", None))        # A76 host LiDAR profile
        self.reg, self.seq_reg = ec.load_sequences()
        self.docker = shlex.split(args.docker_cmd)
        self.stop = False
        self.changed = True
        self.events = None
        self.gate_closed = False         # memory gate state (events only on a change)
        self.last_brake = 0.0
        self.last_sample = 0.0

    # ------------------------------------------------------------------ inputs
    def p(self, *rel):
        return os.path.join(self.root, *rel)

    def host_path(self, *rel):
        return host_project_path(self.root, *rel)

    def volume_args(self):
        return build_volume_args(self.root)

    def resolve(self):
        a = self.a
        self.arms = ec.split_list(a.arms)
        self.seqs = ec.split_list(a.seqs)
        self.seeds = ec.parse_seeds(a.seeds)
        if a.jobs < 1 or a.jobs > N_SLOTS:
            raise ValueError("--jobs must be 1..%d (A41)" % N_SLOTS)
        if not 0.0 <= a.brake_avail_gb < a.min_avail_gb:
            raise ValueError("--brake-avail-gb must be >= 0 and below --min-avail-gb (the gate reopens above the brake)")
        if a.mem_limit_gb < 0:
            raise ValueError("--mem-limit-gb must be >= 0 (0 = no per-container limit)")
        for spec in a.extra_disk:
            path, sep, gib = spec.partition("=")
            if not sep or not path:
                raise ValueError("--extra-disk %r: expected PATH=GiB" % spec)
            try:
                need = float(gib)
            except ValueError:
                raise ValueError("--extra-disk %r: GiB must be a number" % spec)
            if need < 0:
                raise ValueError("--extra-disk %r: GiB must be >= 0" % spec)
            if not os.path.isdir(path):
                raise ValueError("--extra-disk %r: %s is not a directory" % (spec, path))
        self.gzip_after = ec.split_list(a.gzip_after) if a.gzip_after else []
        for g in self.gzip_after:
            if "/" in g or g in ("", ".", "..") or g in ("run_status.json", "replay_stats.json", "run_manifest.json", "runner.log"):
                raise ValueError("--gzip-after %r: a plain output file name of the run directory (not a runner record)" % g)
        problems, self.arm_info, self.seq_info = [], {}, {}
        if mem_available_gib(a.meminfo) is None:
            problems.append("cannot read MemAvailable from %s (the memory gate needs it)" % a.meminfo)
        devel = {}
        for d in a.devel:
            k, _, v = d.partition("=")
            wt, _, dv = v.partition(":")
            if not k or not wt or not dv:
                raise ValueError("--devel %r: expected <build or arm>=<worktree>:<devel dir>" % d)
            devel[k] = (ec.safe_name(wt, "worktree"), dv)
        for arm in self.arms:
            if arm not in self.arms_reg:
                raise ValueError("arm %s is not in tools/eval/arms.json (A46 arms: %s)" % (arm, ", ".join(self.arms_reg)))
            r = self.arms_reg[arm]
            if r.get("calibration") and a.evaluation_phase:
                problems.append("arm %s is a development-set calibration arm (A58), not an evaluation arm" % arm)
            wt, dv = devel.get(arm) or devel.get(r["build"]) or (None, None)
            # A81: "time_calib": false in the arm's registry entry = the arm does not read the A18 time-calibration file (its
            # time alignment is whatever the arm does online); absent or true = the A18 file is loaded when it exists.
            tc = r.get("time_calib", True)
            if not isinstance(tc, bool):
                raise ValueError("arm %s: arms.json time_calib must be true or false, got %r" % (arm, tc))
            info = {"traj": r["traj"], "build": r["build"], "gnss_inputs": bool(r.get("gnss_inputs")), "time_calib": tc,
                    "wt": wt, "devel": dv, "overlay": None, "overlay_sha256": None, "build_id": None}
            if not wt:
                problems.append("arm %s: no --devel for build '%s'" % (arm, r["build"]))
            else:
                setup = self.host_path("wt", wt, dv, "setup.bash")
                if not os.path.isfile(setup):
                    problems.append("arm %s: %s missing (build the %s tree first)" % (arm, setup, r["build"]))
                bid = self.host_path("wt", wt, dv, "BUILD_ID")
                if os.path.isfile(bid):
                    with open(bid) as f:
                        info["build_id"] = f.read().strip()
                if r.get("overlay"):
                    ov = self.p("wt", wt, os.path.dirname(dv), "src", "FAST-LIVO2", r["overlay"])
                    if not os.path.isfile(ov):
                        problems.append("arm %s: overlay %s missing (P3-B: config/arms/**)" % (arm, ov))
                    else:
                        bad = [k for k in overlay_top_keys(ov) if k in RESERVED_OVERLAY_KEYS]
                        if bad:
                            problems.append("arm %s: overlay %s sets reserved keys %s" % (arm, ov, bad))
                        info["overlay"] = os.path.relpath(ov, self.root)
                        info["overlay_sha256"] = ec.sha256_file(ov)
            self.arm_info[arm] = info
        builds = sorted({(i["wt"], i["devel"]) for i in self.arm_info.values() if i["wt"]})
        if len(builds) > 1 and not a.allow_mixed_builds:
            problems.append("A55: every arm of a matrix runs on one build (U = HEAD with gnss.enable false); arms resolve to %s "
                            "-- give one --devel head=<worktree>:<devel dir>, or --allow-mixed-builds for a diagnostic matrix"
                            % ", ".join("%s:%s" % b for b in builds))
        bags = {}
        for b in a.bag:
            k, _, v = b.partition("=")
            if not k or not v or os.path.isabs(v) or ".." in v.split("/"):
                raise ValueError("--bag %r: expected <seq>=<path relative to the project root>" % b)
            bags[k] = v
        for seq in self.seqs:
            if seq not in self.seq_reg:
                raise ValueError("sequence %s is not in tools/eval/sequences.json" % seq)
            cls = ec.check_phase(seq, a.evaluation_phase, "replaying")
            bag = bags.get(seq, "%s/%s.bag" % (self.lidar["bag_dir"], seq))
            s = {"class": cls, "bag": bag, "ready": os.path.isfile(self.p(bag)), "fetch": None, "fetched_by_us": False,
                 "dropped": False, "blocked": None, "tempfail_at": None, "bag_sha256": None,
                 "duration_s": self.seq_reg[seq].get("duration_s", 600), "overlay_seq": None}
            if not s["ready"] and not a.fetch:
                problems.append("%s: bag %s missing (fetch it with tools/eval/fetch_eval.sh or pass --fetch)" % (seq, bag))
            if not s["ready"] and a.fetch and bags.get(seq):
                problems.append("%s: --fetch writes %s/%s.bag; drop the --bag override" % (seq, self.lidar["bag_dir"], seq))
            self.seq_info[seq] = s
        return problems

    def gnss_inputs(self, seq):
        """nav_rinex (fetch record, else the BRDC days of the registry date) and the A18 file (xcorr, else prior)."""
        rec = self.p(*self.lidar["bag_dir"].split("/"), seq + ".fetch.json")
        nav = []
        if os.path.isfile(rec):
            nav = ec.load_json(rec).get("brdc") or []
        if not nav:
            import datetime
            d = datetime.date.fromisoformat(self.seq_reg[seq]["date_local"])
            for dd in (d - datetime.timedelta(days=1), d):   # local UTC+8 date: the UTC day is the same or the day before
                f = "data/brdc/BRDC00IGS_R_%d%03d0000_01D_MN.rnx" % (dd.year, dd.timetuple().tm_yday)
                if os.path.isfile(self.p(f)):
                    nav.append(f)
        if not nav:
            raise ec.Blocked("%s: no BRDC file (A38: gnss.nav_rinex is required)" % seq)
        calib = None
        for c in ("audit/%s/time_calib/%s.yaml" % (seq, seq), "audit/%s/time_calib/%s_record_time.yaml" % (seq, seq)):
            if os.path.isfile(self.p(c)):
                calib = c
                break
        return ["/r/" + x for x in nav], ("/r/" + calib if calib else None), calib

    # ------------------------------------------------------------------ jobs
    def build_jobs(self):
        self.jobs = []
        for seq in self.seqs:
            for arm in self.arms:
                for seed in self.seeds:
                    self.jobs.append({"seq": seq, "arm": arm, "seed": seed, "state": "waiting", "attempts": [],
                                      "proc": None, "fd": None})
        # resume: attempts already on disk (a finished run is never repeated; an unfinished one counts as failed)
        for j in self.jobs:
            n = 1
            while True:
                rel = run_dir_rel(self.name, j["seq"], j["arm"], j["seed"], n)
                d = self.p(rel)
                if not os.path.isdir(d):
                    break
                st = os.path.join(d, "run_status.json")
                complete, brake = 0, False
                if os.path.isfile(st):
                    sj = ec.load_json(st)
                    complete, brake = int(sj.get("complete", 0)), bool(sj.get("memory_brake"))
                att = {"attempt": n, "run_dir": rel, "complete": complete, "resumed": True,
                       "status": ec.load_json(st) if os.path.isfile(st) else "no run_status.json (interrupted)",
                       "build_sha256": json_hash_field(os.path.join(d, "replay_stats.json"), "build_sha256"),
                       "config_sha256": json_hash_field(os.path.join(d, "run_manifest.json"), "config_sha256")}
                if brake:
                    att["memory_brake"] = True
                j["attempts"].append(att)
                if complete:
                    j["state"] = "done"
                    break
                n += 1
            if j["state"] != "done" and (counted_attempts(j) > self.a.retries or brakes(j) >= MAX_BRAKES_PER_RUN):
                j["state"] = "failed"

    def bag_sha(self, bag_abs):
        side = bag_abs + ".sha256"
        if os.path.isfile(side):
            return read_first_token(side)
        st = os.stat(bag_abs)
        cache = self.p("runs", ".bag_sha256", "%s.%d-%d.sha256" % (os.path.basename(bag_abs), st.st_size, int(st.st_mtime)))
        if os.path.isfile(cache):
            return read_first_token(cache)
        h = ec.sha256_file(bag_abs)
        os.makedirs(os.path.dirname(cache), exist_ok=True)
        with open(cache + ".tmp.%d" % os.getpid(), "w") as f:
            f.write(h + "\n")
        os.replace(cache + ".tmp.%d" % os.getpid(), cache)
        return h

    def image_digest(self):
        try:
            r = subprocess.run(self.docker + ["image", "inspect", "--format", "{{.Id}}", self.a.image], stdout=subprocess.PIPE,
                               stderr=subprocess.DEVNULL, universal_newlines=True, timeout=60)
            return r.stdout.strip()
        except Exception:  # noqa: BLE001
            return ""

    def launch(self, j, slot, fd):
        a, seq, arm, seed = self.a, j["seq"], j["arm"], j["seed"]
        s, ai = self.seq_info[seq], self.arm_info[arm]
        n = len(j["attempts"]) + 1
        rel = run_dir_rel(self.name, seq, arm, seed, n)
        run = self.p(rel)
        if os.path.exists(os.path.join(run, "replay_stats.json")):
            raise RuntimeError("refusing to reuse %s" % run)
        wt, devel = ai["wt"], ai["devel"]
        gd, gdirty = "unknown", "unknown"
        if ai["build_id"]:
            tok = ai["build_id"].split()
            gd, gdirty = tok[0], (tok[1] if len(tok) > 1 else "unknown")
        if s["bag_sha256"] is None:
            s["bag_sha256"] = self.bag_sha(self.p(s["bag"]))
        overlays = []
        if ai["overlay"]:
            overlays.append("/r/" + ai["overlay"])
        if ai["gnss_inputs"]:
            # A81: an arm registered with time_calib false gets its own per-sequence overlay without gnss.time.calib_file
            # (<seq>.gnss_inputs_nocalib.yaml, recorded under sequences[seq].gnss_inputs_nocalib); the others keep the
            # <seq>.gnss_inputs.yaml they always had, so a matrix without such an arm writes what it wrote before.
            calib_on = ai["time_calib"]
            okey, ikey, stem = (("overlay_seq", "gnss_inputs", "gnss_inputs") if calib_on
                                else ("overlay_seq_nocalib", "gnss_inputs_nocalib", "gnss_inputs_nocalib"))
            if s.get(okey) is None:
                nav, calib_c, calib_rel = self.gnss_inputs(seq)
                if not calib_on:
                    calib_c = calib_rel = None
                p = os.path.join(self.mdir, "overlays", "%s.%s.yaml" % (seq, stem))
                os.makedirs(os.path.dirname(p), exist_ok=True)
                with open(p, "w") as f:
                    f.write(seq_overlay_text(seq, nav, calib_c))
                s[okey] = os.path.relpath(p, self.root)
                s[ikey] = {"nav_rinex": nav, "calib_file": calib_rel,
                           "calib_kind": None if not calib_rel else ("prior (record_time, sigma 75 ms, TDCP paused)"
                                                                     if calib_rel.endswith("_record_time.yaml") else "method 2 (xcorr)")}
            overlays.append("/r/" + s[okey])
        for sub in ("Log/result", "Log/pcd", "Log/image", "Log/Colmap/sparse/0", "roslog"):
            os.makedirs(os.path.join(run, sub), exist_ok=True)
        if overlays:
            with open(os.path.join(run, "replay.launch"), "w") as f:
                f.write(wrapper_launch(overlays, self.lidar["replay_launch"], os.path.basename(self.lidar["config"])))
            target = "/r/%s/replay.launch" % rel
        else:
            target = "fast_livo %s" % self.lidar["replay_launch"]
        seqname = "%s_%s_s%d" % (seq, arm, seed) + ("" if n == 1 else "_r%d" % (n - 1))
        logmount = os.path.join(os.path.dirname(devel), "src", "FAST-LIVO2", "Log")
        cname = "fl2g-eval-%s-%s-%s-s%d-a%d-%d" % (self.name, seq, arm, seed, n, int(time.time()) % 100000)
        inner = ("source /r/wt/%s/%s/setup.bash && exec roslaunch -p %d %s bag:=%s output_dir:=%s seed:=%d duration:=%s "
                 "seq:=%s arm:=%s git_describe:=%s git_dirty:=%s image_digest:=%s bag_sha256:=%s") % (
            wt, devel, ROS_PORT, target, shlex.quote("/r/" + s["bag"]), shlex.quote("/r/" + rel), seed,
            shlex.quote(str(a.duration)), shlex.quote(seqname), shlex.quote(arm), shlex.quote(gd), shlex.quote(gdirty),
            shlex.quote(self.digest), shlex.quote(s["bag_sha256"]))
        mem = []
        if a.mem_limit_gb > 0:          # no swap for the container: --memory-swap = --memory
            lim = "%dm" % int(round(a.mem_limit_gb * 1024))
            mem += ["--memory", lim, "--memory-swap", lim]
        if a.oom_score_adj is not None:
            mem += ["--oom-score-adj", str(int(a.oom_score_adj))]
        if a.cpus:
            mem += ["--cpus", "%g" % a.cpus]
        cmd = self.docker + ["run", "--rm", "-u", a.docker_user, "--name", cname] + mem + [
                             "-e", "HOME=/r/wt/%s/home" % wt, "-e", "OMP_NUM_THREADS=1", "-e", "FL2G_HOST=%s" % ec.host_name(),
                             "-e", "ROS_LOG_DIR=/r/%s/roslog" % rel, "-e", "ROS_HOME=/r/%s/roslog" % rel,
                             ] + self.volume_args() + [
                             "-v", "%s/Log:/r/wt/%s/%s" % (run, wt, logmount), "-w", "/r/%s" % rel,
                             a.image, "nice", "-n", "19", "ionice", "-c3", "bash", "-c", inner]
        avail = mem_available_gib(a.meminfo)
        with open(os.path.join(run, "runner.log"), "a") as f:
            f.write("[run_matrix %s] start %s wt=%s devel=%s bag=%s seed=%d dur=%s build=%s/%s slot=%d launch=%s container=%s "
                    "mem_available=%.1fGiB docker_mem=%s\n" % (
                        time.strftime("%T"), seqname, wt, devel, s["bag"], seed, a.duration, gd, gdirty, slot, target, cname,
                        avail if avail is not None else -1.0, " ".join(mem) or "none"))
        con = open(os.path.join(run, "console.log"), "w")
        try:
            proc = subprocess.Popen(cmd, stdout=con, stderr=subprocess.STDOUT, pass_fds=(fd,), start_new_session=True)
        finally:
            con.close()
            os.close(fd)                   # the docker client holds the slot lock from here on (or nobody, on failure)
        att = {"attempt": n, "run_dir": rel, "slot": slot, "container": cname, "launch": target,
               "overlays": [o[3:] for o in overlays], "start_utc": ec.utc_now(), "t0": time.time(),
               "docker_memory_args": mem, "mem_available_at_start_gib": None if avail is None else round(avail, 2),
               "peak_rss_gib": None, "min_mem_available_gib": None if avail is None else round(avail, 2)}
        j["attempts"].append(att)
        j["proc"], j["state"] = proc, "running"
        self.event("start %s/%s/s%d attempt %d slot %d (MemAvailable %.1f GiB)" % (seq, arm, seed, n, slot,
                                                                                  avail if avail is not None else -1.0))

    def sample_memory(self, avail):
        """Peak RSS (VmHWM) of every running attempt's fastlivo_replay and the lowest MemAvailable while it runs."""
        mem = replay_memory(self.a.proc_root)
        for j in self.jobs:
            if j["state"] != "running":
                continue
            att = j["attempts"][-1]
            if avail is not None:
                old = att["min_mem_available_gib"]
                att["min_mem_available_gib"] = round(avail if old is None else min(avail, old), 2)
                if old is None or old - att["min_mem_available_gib"] >= 0.5:
                    self.changed = True           # the manifest is rewritten on a visible change only
            m = mem.get(att["run_dir"])
            if m:
                old = att["peak_rss_gib"] or 0.0
                att["peak_rss_gib"] = round(max(m[1] / (1024.0 * 1024.0), old), 3)
                if att["peak_rss_gib"] - old >= 0.1:
                    self.changed = True

    def finalize(self, j, rc, killed_reason=None, brake=False):
        att = j["attempts"][-1]
        run = self.p(att["run_dir"])
        with open(os.path.join(run, "console.log"), errors="replace") as f:
            text = f.read()
        node_rc, done_line = ec.console_verdict(text)
        stats = ec.stats_complete(run)
        manifest_ok = 0
        if stats:
            r = subprocess.run(["python3", self.manifest_check, "--run", run], stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                               universal_newlines=True)
            with open(os.path.join(run, "runner.log"), "a") as f:
                f.write(r.stdout)
            manifest_ok = int(r.returncode == 0)
        cycles = ec.count_cycles(run)
        complete = int(stats == 1 and done_line == 1 and node_rc == "0" and manifest_ok == 1)
        status = {"complete": complete, "roslaunch_rc": rc, "node_exit_code": node_rc or "unknown", "done_line": done_line,
                  "replay_stats": stats, "manifest_ok": manifest_ok, "cycles": cycles}
        if killed_reason:
            status["killed"] = killed_reason
        if brake and not complete:
            status["memory_brake"] = 1       # read back on resume: this attempt does not use up --retries
        if att.get("peak_rss_gib") is not None:
            status["peak_rss_gib"] = att["peak_rss_gib"]   # extra keys only when they carry something (replay_run.sh format)
        with open(os.path.join(run, "run_status.json"), "w") as f:
            f.write(json.dumps(status) + "\n")
        with open(os.path.join(run, "runner.log"), "a") as f:
            f.write("[run_matrix %s] end %s roslaunch_rc=%s node_rc=%s done_line=%d stats=%d manifest_ok=%d cycles=%d complete=%d "
                    "peak_rss=%sGiB min_mem_available=%sGiB%s\n" % (
                        time.strftime("%T"), att["run_dir"], rc, node_rc or "unknown", done_line, stats, manifest_ok, cycles, complete,
                        att.get("peak_rss_gib"), att.get("min_mem_available_gib"), (" killed=" + killed_reason) if killed_reason else ""))
        att.update(status=status, complete=complete, end_utc=ec.utc_now(), wall_s=round(time.time() - att.pop("t0"), 1),
                   killed=killed_reason, build_sha256=json_hash_field(os.path.join(run, "replay_stats.json"), "build_sha256"),
                   config_sha256=json_hash_field(os.path.join(run, "run_manifest.json"), "config_sha256"))
        if brake and not complete:
            att["memory_brake"] = True
        if complete and self.gzip_after:
            done = []
            for g in self.gzip_after:
                p = os.path.join(run, g)
                if os.path.isfile(p) and not os.path.exists(p + ".gz"):
                    try:
                        gzip_in_place(p)
                        done.append(g)
                    except OSError as e:
                        self.event("gzip %s failed (kept as is): %s" % (p, e))
            att["gzipped"] = done
        j["proc"] = None
        if complete:
            j["state"] = "done"
        elif self.stop:
            j["state"] = "failed"
        elif att.get("memory_brake") and brakes(j) < MAX_BRAKES_PER_RUN:
            j["state"] = "waiting"
        elif not att.get("memory_brake") and counted_attempts(j) <= self.a.retries:
            j["state"] = "waiting"
        else:
            j["state"] = "failed"
        self.event("end %s/%s/s%d attempt %d complete=%d -> %s" % (j["seq"], j["arm"], j["seed"], att["attempt"], complete, j["state"]))

    # ------------------------------------------------------------------ rolling fetch
    def fetch_cmd(self, sub, seq):
        base = shlex.split(self.a.fetch_cmd) if self.a.fetch_cmd else ["bash", os.path.join(ec.HERE, "fetch_eval.sh")]
        cmd = base + ["--root", self.root] + (["--lidar", self.lidar["name"]] if self.lidar["name"] != ec.load_lidars()[0] else []) + [sub, seq]
        if sub == "fetch":
            if self.a.evaluation_phase:
                cmd.append("--evaluation-phase")
            if self.a.i2:
                cmd.append("--i2")
            cmd += self.a.fetch_arg
        return cmd

    def manage_fetch(self):
        for seq, s in self.seq_info.items():
            pr = s["fetch"]
            if pr is not None and pr.poll() is not None:
                rc = pr.returncode
                s["fetch"] = None
                if rc == 0 and os.path.isfile(self.p(s["bag"])):
                    s["ready"], s["fetched_by_us"] = True, True
                    self.event("fetch %s done" % seq)
                elif rc == EX_TEMPFAIL and s.get("tempfails", 0) < 6:
                    s["tempfail_at"] = time.time()
                    s["tempfails"] = s.get("tempfails", 0) + 1
                    self.event("fetch %s: tempfail %d (disk); retried after the next drop or in 10 min" % (seq, s["tempfails"]))
                else:
                    s["blocked"] = "fetch failed (rc %d, see %s/fetch_%s.log)" % (rc, self.mdir_rel, seq)
                    self.event("fetch %s FAILED rc %d" % (seq, rc))
                    for j in self.jobs:
                        if j["seq"] == seq and j["state"] == "waiting":
                            j["state"] = "failed"
                            j["attempts"].append({"attempt": 0, "error": s["blocked"]})
        if not self.a.fetch or any(s["fetch"] is not None for s in self.seq_info.values()):
            return
        present = sum(1 for s in self.seq_info.values() if s["ready"] and not s["dropped"])
        if present >= self.a.max_bags:
            return
        for seq in self.seqs:
            s = self.seq_info[seq]
            if s["ready"] or s["dropped"] or s["blocked"]:
                continue
            if not any(j["seq"] == seq and j["state"] == "waiting" for j in self.jobs):
                continue
            if s["tempfail_at"] and time.time() - s["tempfail_at"] < 600 and not self.dropped_since(s["tempfail_at"]):
                return
            log = open(os.path.join(self.mdir, "fetch_%s.log" % seq), "a")
            s["fetch"] = subprocess.Popen(self.fetch_cmd("fetch", seq), stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            log.close()
            self.event("fetch %s started" % seq)
            return

    def dropped_since(self, t):
        return any(t_drop > t for t_drop in self.drop_times)

    def manage_drop(self):
        if not self.a.drop_after:
            return
        for seq, s in self.seq_info.items():
            if s["dropped"] or not s["ready"] or not (s["fetched_by_us"] or self.a.drop_any):
                continue
            if all(j["state"] in ("done", "failed") for j in self.jobs if j["seq"] == seq):
                r = subprocess.run(self.fetch_cmd("drop", seq), stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True)
                s["dropped"], s["ready"] = True, False
                self.drop_times.append(time.time())
                self.event("drop %s rc %d: %s" % (seq, r.returncode, r.stdout.strip()[-200:]))

    # ------------------------------------------------------------------ bookkeeping
    def event(self, msg):
        line = "[run_matrix %s] %s" % (time.strftime("%F %T"), msg)
        print(line, flush=True)
        if self.events:
            self.events.write(line + "\n")
            self.events.flush()
        self.changed = True

    def manifest(self):
        jobs = []
        for j in self.jobs:
            jobs.append({"seq": j["seq"], "arm": j["arm"], "seed": j["seed"], "state": j["state"],
                         "attempts": [{k: v for k, v in at.items() if k != "t0"} for at in j["attempts"]]})
        seqs = {}
        for seq, s in self.seq_info.items():
            seqs[seq] = {k: v for k, v in s.items() if k not in ("fetch",)}
            seqs[seq]["fetching"] = s["fetch"] is not None
        return {"tool": "tools/eval/run_matrix.py", "manifest_version": 1, "name": self.name, "root": self.root,
                "host": ec.host_name(), "created_utc": self.created, "updated_utc": ec.utc_now(), "argv": sys.argv,
                "source_id": self.source_id, "image": self.a.image, "image_digest": self.digest,
                "lidar": self.lidar["name"], "lidar_config": self.lidar["config"], "lidar_launch": self.lidar["replay_launch"],
                "evaluation_phase": bool(self.a.evaluation_phase), "seeds": self.seeds, "duration_s": self.a.duration,
                "retries": self.a.retries, "jobs_max": self.a.jobs, "allow_mixed_builds": bool(self.a.allow_mixed_builds),
                "memory": {"min_avail_gib": self.a.min_avail_gb, "brake_avail_gib": self.a.brake_avail_gb,
                           "mem_limit_gib": self.a.mem_limit_gb or None, "oom_score_adj": self.a.oom_score_adj,
                           "sample_s": self.a.mem_sample_s, "max_brakes_per_run": MAX_BRAKES_PER_RUN},
                "disk": {"min_free_gib": self.a.min_free_disk_gb, "gzip_after": self.gzip_after,
                         "extra": list(self.a.extra_disk)},
                "docker_user": self.a.docker_user, "profile": self.a.profile,
                "manifest_check": self.manifest_check, "arms": self.arm_info, "sequences": seqs, "runs": jobs,
                "counts": {st: sum(1 for j in self.jobs if j["state"] == st) for st in ("waiting", "running", "done", "failed")}}

    def save(self):
        if self.changed:
            ec.atomic_write_json(os.path.join(self.mdir, "matrix_manifest.json"), self.manifest())
            self.changed = False

    # ------------------------------------------------------------------ memory (README "内存")
    def memory_gate(self):
        """True when a replay may be launched now: MemAvailable >= --min-avail-gb (read afresh before every launch) and
        the root's file system has >= --min-free-disk-gb free."""
        avail = mem_available_gib(self.a.meminfo)
        if avail is None or avail < self.a.min_avail_gb:
            if self.gate_closed != "memory":
                self.gate_closed = "memory"
                self.event("memory gate closed: MemAvailable %s GiB < %.1f GiB; no launch until it is back above" % (
                    "?" if avail is None else "%.1f" % avail, self.a.min_avail_gb))
            return False
        blocks = []
        disk = free_disk_gib(self.root)
        if disk < self.a.min_free_disk_gb:
            blocks.append((disk, self.root, self.a.min_free_disk_gb))
        for spec in self.a.extra_disk:
            path, _, gib = spec.partition("=")
            free = free_disk_gib(path)
            need = float(gib)
            if free < need:
                blocks.append((free, path, need))
        if blocks:
            free, path, need = blocks[0]
            if self.gate_closed != "disk":
                self.gate_closed = "disk"
                self.event("disk gate closed: %.1f GiB free under %s < %.1f GiB; no launch until space is back" % (
                    free, path, need))
            return False
        if self.gate_closed:
            self.gate_closed = False
            self.event("memory gate open: MemAvailable %.1f GiB, %.1f GiB free on disk" % (avail, disk))
        return True

    def memory_brake(self, avail, now):
        """Below --brake-avail-gb while replays run: kill the youngest one (least work lost) and queue it again."""
        running = [j for j in self.jobs if j["state"] == "running"]
        if self.stop or avail is None or avail >= self.a.brake_avail_gb or not running:
            return
        if now - self.last_brake < self.a.brake_interval_s:
            return                        # give the kernel time to hand back the killed replay's memory
        j = max(running, key=lambda x: x["attempts"][-1]["t0"])
        att = j["attempts"][-1]
        reason = "memory brake: MemAvailable %.1f GiB < %.1f GiB" % (avail, self.a.brake_avail_gb)
        self.event("%s: killing the youngest replay %s/%s/s%d attempt %d" % (reason, j["seq"], j["arm"], j["seed"], att["attempt"]))
        subprocess.run(self.docker + ["kill", att["container"]], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        try:
            j["proc"].wait(timeout=120)
        except subprocess.TimeoutExpired:
            j["proc"].kill()
            j["proc"].wait()
        self.finalize(j, j["proc"].returncode if j["proc"] else -9, killed_reason=reason, brake=True)
        self.last_brake = now

    # ------------------------------------------------------------------ main loop
    def run(self):
        a = self.a
        os.makedirs(self.mdir, exist_ok=True)
        # one orchestrator per matrix: a second one would take the first one's running attempts for interrupted ones
        self.lock_fd = os.open(os.path.join(self.mdir, ".matrix.lock"), os.O_RDWR | os.O_CREAT, 0o664)
        try:
            fcntl.flock(self.lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            print("run_matrix: matrix %s is already being run by another process" % self.name, file=sys.stderr)
            return 3
        self.events = open(os.path.join(self.mdir, "matrix_events.log"), "a")
        old = os.path.join(self.mdir, "matrix_manifest.json")
        self.created = ec.load_json(old).get("created_utc") if os.path.isfile(old) else ec.utc_now()
        self.source_id = source_id()
        self.digest = self.image_digest()
        self.manifest_check = a.manifest_check or os.path.join(ec.REPO, "docs", "dev", "ref_r_gates", "manifest_check.py")
        self.drop_times = []
        self.build_jobs()
        self.event("matrix %s: %d jobs (%s done, %s failed from earlier invocations); arms %s; seqs %s; seeds %s; jobs <= %d" % (
            self.name, len(self.jobs), sum(j["state"] == "done" for j in self.jobs), sum(j["state"] == "failed" for j in self.jobs),
            ",".join(self.arms), ",".join(self.seqs), self.seeds, a.jobs))

        def on_signal(signum, _frame):
            self.stop = True
            self.event("signal %d: stopping (running containers are killed)" % signum)
        signal.signal(signal.SIGTERM, on_signal)
        signal.signal(signal.SIGINT, on_signal)
        while True:
            self.manage_fetch()
            now = time.time()
            avail = mem_available_gib(a.meminfo)
            if now - self.last_sample >= a.mem_sample_s:
                self.sample_memory(avail)
                self.last_sample = now
            self.memory_brake(avail, now)
            running = [j for j in self.jobs if j["state"] == "running"]
            for j in running:
                rc = j["proc"].poll()
                att = j["attempts"][-1]
                if rc is None:
                    limit = a.timeout_factor * self.seq_info[j["seq"]]["duration_s"] + a.timeout_base
                    if self.stop or time.time() - att["t0"] > limit:
                        reason = "stop" if self.stop else "timeout %.0f s" % limit
                        subprocess.run(self.docker + ["kill", att["container"]], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                        try:
                            j["proc"].wait(timeout=120)
                        except subprocess.TimeoutExpired:
                            j["proc"].kill()
                            j["proc"].wait()
                        self.finalize(j, j["proc"].returncode if j["proc"] else -9, killed_reason=reason)
                    continue
                self.finalize(j, rc)
            if not self.stop:
                n_run = sum(1 for j in self.jobs if j["state"] == "running")
                for j in self.jobs:
                    if n_run >= a.jobs:
                        break
                    if j["state"] != "waiting" or not self.seq_info[j["seq"]]["ready"]:
                        continue
                    if not self.memory_gate():
                        break
                    slot, fd = try_acquire_slot(self.root)
                    if slot is None:
                        break
                    try:
                        self.launch(j, slot, fd)
                    except (ec.Blocked, OSError, RuntimeError) as e:
                        try:
                            os.close(fd)
                        except OSError:
                            pass
                        j["state"] = "failed"
                        j["attempts"].append({"attempt": 0, "error": "%s: %s" % (type(e).__name__, e)})
                        self.event("blocked %s/%s/s%d: %s" % (j["seq"], j["arm"], j["seed"], e))
                        continue
                    n_run += 1
            self.manage_drop()
            self.save()
            busy = any(j["state"] == "running" for j in self.jobs) or any(s["fetch"] is not None for s in self.seq_info.values())
            pending = any(j["state"] == "waiting" for j in self.jobs)
            if not busy and (self.stop or not pending):
                break
            if not busy and pending and not a.fetch and not any(self.seq_info[j["seq"]]["ready"] for j in self.jobs if j["state"] == "waiting"):
                break
            time.sleep(a.poll_s)
        self.changed = True
        self.save()
        c = self.manifest()["counts"]
        self.event("matrix %s finished: done %d, failed %d, waiting %d" % (self.name, c["done"], c["failed"], c["waiting"]))
        self.events.close()
        self.events = None
        os.close(self.lock_fd)
        return 0 if c["failed"] == 0 and c["waiting"] == 0 else 1


def source_id():
    p = os.path.join(ec.HERE, ".source_id")
    if os.path.isfile(p):
        with open(p) as f:
            return f.read().strip()
    try:
        r = subprocess.run(["git", "-C", ec.REPO, "describe", "--always", "--dirty"], stdout=subprocess.PIPE,
                           stderr=subprocess.DEVNULL, universal_newlines=True, timeout=10)
        return r.stdout.strip() or "unknown"
    except Exception:  # noqa: BLE001
        return "unknown"


def cmd_status(args):
    root = ec.project_root(args.root)
    p = os.path.join(root, "runs", "eval", ec.safe_name(args.name, "matrix name"), "matrix_manifest.json")
    m = ec.load_json(p)
    print("matrix %s (%s .. %s): %s" % (m["name"], m["created_utc"], m["updated_utc"], m["counts"]))
    for r in m["runs"]:
        last = r["attempts"][-1] if r["attempts"] else {}
        print("  %-15s %-7s s%d  %-8s attempts %d  %s" % (r["seq"], r["arm"], r["seed"], r["state"], len(r["attempts"]),
                                                       last.get("run_dir", last.get("error", ""))))
    return 0


def build_parser():
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0], formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd")
    for name in ("plan", "run"):
        s = sub.add_parser(name)
        s.add_argument("--name", required=True, help="matrix name -> runs/eval/<name>/")
        s.add_argument("--arms", required=True, help="comma list from tools/eval/arms.json (A46: U,L-PVT,T,T-pr,T-dop,T-tdcp)")
        s.add_argument("--seqs", required=True, help="comma list from tools/eval/sequences.json")
        s.add_argument("--seeds", default="0-4", help="A30/A46 seeds (default 0-4)")
        s.add_argument("--devel", action="append", default=[], help="<build|arm>=<worktree>:<devel dir>, e.g. head=p3i-strict:ws/devel")
        s.add_argument("--allow-mixed-builds", action="store_true",
                       help="A55 exception: let the arms of this matrix run on different builds (diagnostics only)")
        s.add_argument("--bag", action="append", default=[], help="<seq>=<bag relative to the root> (default <bag dir of --lidar>/<seq>.bag, "
                       "data/m3dgr/filtered/<seq>.bag for Avia)")
        s.add_argument("--lidar", default=None, choices=sorted(ec.load_lidars()[1]),
                       help="host LiDAR profile of tools/eval/lidars.json (A76): replay launch + config, bag directory, fetch "
                            "topics; default %s" % ec.load_lidars()[0])
        s.add_argument("--jobs", type=int, default=JOBS_DEFAULT,
                       help="concurrent replays of this runner (1..8, A41; default %d: README '内存', 2026-09-25 incident)" % JOBS_DEFAULT)
        s.add_argument("--min-avail-gb", type=float, default=MIN_AVAIL_GIB,
                       help="launch a replay only while MemAvailable >= this many GiB (free -g 'available'), else wait")
        s.add_argument("--brake-avail-gb", type=float, default=BRAKE_AVAIL_GIB,
                       help="below this MemAvailable (GiB) while running, kill the youngest replay and queue it again")
        s.add_argument("--mem-limit-gb", type=float, default=MEM_LIMIT_GIB,
                       help="docker --memory = --memory-swap per replay in GiB (0 = none)")
        s.add_argument("--oom-score-adj", type=int, default=OOM_SCORE_ADJ,
                       help="docker --oom-score-adj of every replay (a global OOM kills a replay first)")
        s.add_argument("--cpus", type=float, default=None,
                       help="docker --cpus quota (CFS; not --cpuset-cpus). Unset on the remote host; local profiles set 4")
        s.add_argument("--mem-sample-s", type=float, default=2.0, help="peak-RSS sampling period of the running replays")
        s.add_argument("--min-free-disk-gb", type=float, default=MIN_FREE_DISK_GIB,
                       help="launch a replay only while the root's file system has this many GiB free")
        s.add_argument("--gzip-after", default="",
                       help="comma list of bulky output files of a run (e.g. console.log,filter_cov.csv,state_trace.txt,"
                            "timing.csv) gzipped in place once the run is complete")
        s.add_argument("--meminfo", default="/proc/meminfo", help=argparse.SUPPRESS)
        s.add_argument("--proc-root", default="/proc", help=argparse.SUPPRESS)
        s.add_argument("--brake-interval-s", type=float, default=30.0, help=argparse.SUPPRESS)
        s.add_argument("--duration", type=float, default=-1.0, help="replay duration in s (-1 = whole bag)")
        s.add_argument("--retries", type=int, default=1)
        s.add_argument("--timeout-factor", type=float, default=8.0, help="kill a run after factor x bag duration + 900 s")
        s.add_argument("--fetch", action="store_true", help="rolling fetch of missing bags (fetch_eval.sh)")
        s.add_argument("--fetch-arg", action="append", default=[], help="extra argument for fetch_eval.sh fetch (repeatable)")
        s.add_argument("--i2", action="store_true", help="fetch: run the I2 audit on the unfiltered bag")
        s.add_argument("--max-bags", type=int, default=2, help="filtered bags on disk at once (fetched + fetching)")
        s.add_argument("--drop-after", action="store_true", help="drop a fetched sequence's filtered bag when its runs are final")
        s.add_argument("--drop-any", action="store_true", help="--drop-after also for bags this runner did not fetch")
        s.add_argument("--evaluation-phase", action="store_true", help="A2: required for evaluation-set sequences")
        s.add_argument("--image", default=ec.IMAGE_DEFAULT)
        s.add_argument("--root", default=None)
        s.add_argument("--docker-cmd", default="sudo -n docker", help=argparse.SUPPRESS)
        s.add_argument("--docker-user", default="1000:1000",
                       help="docker -u UID:GID (default 1000:1000, the remote host); 'host' = the current user")
        s.add_argument("--profile", default=None,
                       help="named overrides in tools/eval/host_profiles.json (explicit flags win; defaults stay the remote ones)")
        s.add_argument("--extra-disk", action="append", default=[],
                       help="additional launch gate PATH=GiB, besides --min-free-disk-gb on the project root")
        s.add_argument("--manifest-check", default="", help=argparse.SUPPRESS)
        s.add_argument("--poll-s", type=float, default=2.0, help=argparse.SUPPRESS)
        s.add_argument("--fetch-cmd", default="", help=argparse.SUPPRESS)
        s.add_argument("--timeout-base", type=float, default=900.0, help=argparse.SUPPRESS)
    s = sub.add_parser("status")
    s.add_argument("--name", required=True)
    s.add_argument("--root", default=None)
    return p


def main(argv=None):
    args = build_parser().parse_args(argv)
    if args.cmd == "status":
        return cmd_status(args)
    if args.cmd not in ("plan", "run"):
        build_parser().print_help()
        return 2
    try:
        apply_profile(args, argv)
    except ValueError as e:
        print("run_matrix: %s" % e, file=sys.stderr)
        return 2
    m = Matrix(args)
    try:
        problems = m.resolve()
    except (ec.PhaseError, ec.Blocked, ValueError) as e:
        print("run_matrix: BLOCKED: %s" % e, file=sys.stderr)
        return 3
    for pr in problems:
        print("run_matrix: problem: %s" % pr, file=sys.stderr)
    if args.cmd == "plan":
        m.build_jobs()
        for j in m.jobs:
            print("%-15s %-7s s%d  %-8s %s" % (j["seq"], j["arm"], j["seed"], j["state"],
                                             run_dir_rel(m.name, j["seq"], j["arm"], j["seed"], len(j["attempts"]) + 1)))
        for arm, i in m.arm_info.items():
            print("arm %-7s build %-4s %s:%s build_id=%s overlay=%s traj=%s" % (arm, i["build"], i["wt"], i["devel"], i["build_id"], i["overlay"], i["traj"]))
        print("lidar %s: launch %s (config %s), bags in %s" % (m.lidar["name"], m.lidar["replay_launch"], m.lidar["config"], m.lidar["bag_dir"]))
        return 3 if problems else 0
    if problems:
        return 3
    return m.run()


if __name__ == "__main__":
    sys.exit(main())
