#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Stand-in for fetch_eval.sh in test_run_matrix.py: `--root R fetch SEQ ...` creates the filtered bag (and checks
that no more than FAKE_MAX_BAGS filtered bags exist afterwards), `--root R drop SEQ` removes it; every call is logged."""
import glob
import json
import os
import sys
import time

a = sys.argv[1:]
root = a[a.index("--root") + 1]
lidar = a[a.index("--lidar") + 1] if "--lidar" in a else "avia"          # A76: a global option before the subcommand
rest = [x for i, x in enumerate(a) if x != "--lidar" and (i == 0 or a[i - 1] != "--lidar")]
cmd, seq = rest[rest.index("--root") + 2], rest[rest.index("--root") + 3]
fd = os.path.join(root, "data", "m3dgr", "filtered" if lidar == "avia" else "filtered_" + lidar)
os.makedirs(fd, exist_ok=True)
with open(os.path.join(root, "fetch_calls.log"), "a") as f:
    f.write("%s %s %s\n" % (cmd, seq, " ".join(a)))
if cmd == "fetch":
    time.sleep(0.2)
    with open(os.path.join(fd, seq + ".bag"), "w") as f:
        f.write("bag")
    with open(os.path.join(fd, seq + ".bag.sha256"), "w") as f:
        f.write("%s  %s.bag\n" % ("a" * 64, seq))
    with open(os.path.join(fd, seq + ".fetch.json"), "w") as f:
        json.dump({"status": "done", "brdc": ["data/brdc/BRDC00IGS_R_20250190000_01D_MN.rnx"]}, f)
    n = len(glob.glob(os.path.join(fd, "*.bag")))
    if n > int(os.environ.get("FAKE_MAX_BAGS", "99")):
        with open(os.path.join(root, "fetch_violation"), "w") as f:
            f.write("%d bags present" % n)
elif cmd == "drop":
    for x in (seq + ".bag", seq + ".bag.sha256"):
        p = os.path.join(fd, x)
        if os.path.exists(p):
            os.remove(p)
