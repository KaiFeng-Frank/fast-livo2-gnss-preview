#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Stand-in for `sudo -n docker` in test_run_matrix.py: `image inspect`, `kill <name>` and `run ... bash -c <inner>`.
A run writes what fastlivo_replay + roslaunch would (console lines, replay_stats.json, run_manifest.json, state_trace,
traj_W, and traj_N for GNSS arms) into the -w directory mapped back from /r. Behaviour from FAKE_DOCKER (json): fail_first = list of
"<seq>/<arm>/s<k>" whose first attempt dies; hang = list that never finishes; hang_first = list whose first attempt never
finishes; sleep = seconds per run; track = file recording the peak number of concurrent runs. Every `run` logs its full
argv (the docker options included) to invocations.log."""
import fcntl
import json
import os
import re
import signal
import sys
import time

cfg = json.loads(os.environ.get("FAKE_DOCKER", "{}"))
argv = sys.argv[1:]
if argv[:2] == ["image", "inspect"]:
    print("sha256:fakeimage")
    sys.exit(0)
if argv[:1] == ["kill"]:
    pidf = os.path.join(cfg["state"], argv[1] + ".pid")
    if os.path.isfile(pidf):
        os.kill(int(open(pidf).read()), signal.SIGTERM)
    sys.exit(0)
assert argv[0] == "run", argv
root = [argv[i + 1].split(":/r")[0] for i, a in enumerate(argv) if a == "-v" and argv[i + 1].endswith(":/r")][0]
name = argv[argv.index("--name") + 1]
wdir = argv[argv.index("-w") + 1]
run = root + wdir[2:]
inner = argv[-1]
kv = dict(re.findall(r"(\w+):=(\S+)", inner))
rel = wdir[3:]
m = re.search(r"runs/eval/[^/]+/([^/]+)/([^/]+)/s(\d+)(\.retry\d+)?$", rel)
key = "%s/%s/s%s" % (m.group(1), m.group(2), m.group(3))
first = m.group(4) is None
os.makedirs(cfg["state"], exist_ok=True)
with open(os.path.join(cfg["state"], name + ".pid"), "w") as f:
    f.write(str(os.getpid()))
with open(os.path.join(cfg["state"], "invocations.log"), "a") as f:
    f.write(json.dumps({"key": key, "rel": rel, "inner": inner, "fds": sorted(os.listdir("/proc/self/fd")), "argv": argv,
                        "t": time.time()}) + "\n")


def bump(delta):
    p = cfg.get("track")
    if not p:
        return
    with open(p, "a+") as f:
        fcntl.flock(f, fcntl.LOCK_EX)
        f.seek(0)
        d = json.loads(f.read() or '{"now": 0, "peak": 0}')
        d["now"] += delta
        d["peak"] = max(d["peak"], d["now"])
        f.seek(0)
        f.truncate()
        f.write(json.dumps(d))


# a stand-in for the replay node in a fake /proc (cfg "proc"): comm, cmdline with the node's ROS log argument, VmHWM
proc_dir = None
if cfg.get("proc"):
    proc_dir = os.path.join(cfg["proc"], str(os.getpid()))
    os.makedirs(proc_dir, exist_ok=True)
    with open(os.path.join(proc_dir, "comm"), "w") as f:
        f.write("fastlivo_replay\n")
    with open(os.path.join(proc_dir, "cmdline"), "wb") as f:
        f.write(b"fastlivo_replay\0__name:=laserMapping\0__log:=/r/" + rel.encode() + b"/roslog/u/laserMapping-2.log\0")
    with open(os.path.join(proc_dir, "status"), "w") as f:
        f.write("Name:\tfastlivo_replay\nVmHWM:\t%d kB\nVmRSS:\t%d kB\n" % (cfg.get("hwm_kib", 1048576), cfg.get("hwm_kib", 1048576) // 2))

bump(1)
stop = []
signal.signal(signal.SIGTERM, lambda *_: stop.append(1))
try:
    t_end = time.time() + cfg.get("sleep", 0.1)
    while time.time() < t_end or key in cfg.get("hang", []) or (first and key in cfg.get("hang_first", [])):
        if stop:
            print("[laserMapping-1] process has died [pid 42, exit code -15, cmd ...]")
            sys.exit(1)
        time.sleep(0.02)
    if first and key in cfg.get("fail_first", []):
        print("[laserMapping-1] process has died [pid 42, exit code -11, cmd ...]")
        sys.exit(0)
    seed = int(kv["seed"])
    arm = kv["arm"]
    # every run writes the host trajectory; a GNSS arm also the N-frame one from a fake activation on (row 50)
    for traj in ("traj_W.tum",) + (() if arm == "U" else ("traj_N.tum",)):
        with open(os.path.join(run, traj), "w") as f:
            for i in range(200 if traj == "traj_W.tum" else 150):
                j = i if traj == "traj_W.tum" else i + 50
                t = 1000.0 + 0.1 * seed + 0.05 * j
                f.write("%.9f %.9f %.9f 0.0 0 0 0 1\n" % (t, 0.01 * j + 0.001 * seed, 0.002 * j))
    with open(os.path.join(run, "state_trace.txt"), "w") as f:
        f.write("# header\n" + "row\n" * 17)
    with open(os.path.join(run, "run_manifest.json"), "w") as f:
        json.dump({"config_sha256": "c" * 64, "arm": arm, "seed": seed}, f)
    with open(os.path.join(run, "replay_stats.json"), "w") as f:
        f.write('{\n  "complete": true,\n  "build_sha256": "%s"\n}\n' % ("b" * 64))
    print("fastlivo_replay: done cycles=17")
    print("[laserMapping-1] process has finished cleanly")
finally:
    bump(-1)
    if proc_dir:
        import shutil
        shutil.rmtree(proc_dir, ignore_errors=True)
