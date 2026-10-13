#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0-only
# p3b_i14_remote.sh <wt> — inside the project container (tools/dev/remote_tool.sh: host flock, nice 19, -u 1000:1000):
# build the catkin test targets of wt/<wt>/ws and run the P3-B rostest (I14 + glue unit tests). rostest --text exits 0
# even when a gtest case fails, so the verdict is read from the gtest XML it writes.
set -Eeuo pipefail
WT=${1:?wt}
# remote_tool.sh pins OMP_NUM_THREADS=1 for the Python tools. catkin_make re-runs CMake, whose ProcessorCount() honours
# OMP_NUM_THREADS through nproc: left set, it would reconfigure the tree with MP_PROC_NUM=1 and without MP_EN and
# rebuild the libraries the tests link — the default-profile build would no longer pass the compile-command gate
# (remote_build.sh never sets it for the same reason).
unset OMP_NUM_THREADS
source /opt/ros/noetic/setup.bash
cd "/r/wt/$WT/ws"
if ! catkin_make -j"${FL2G_JOBS:-10}" tests > "/r/wt/$WT/home/p3b_tests_build.log" 2>&1; then
  tail -40 "/r/wt/$WT/home/p3b_tests_build.log"; echo "catkin_make tests failed"; exit 2
fi
grep -E "warning|error" "/r/wt/$WT/home/p3b_tests_build.log" | grep -i "test_p3b_glue" | head -20 || true
source devel/setup.bash
export ROS_HOME="/r/wt/$WT/home/.ros" ROS_LOG_DIR="/r/wt/$WT/home/.ros/log"
XML="$ROS_HOME/test_results/fast_livo/rosunit-test_p3b_glue.xml"
rm -f "$XML"
OMP_NUM_THREADS=1 rostest --text fast_livo p3b_glue.test || true
python3 - "$XML" <<'PY'
import sys, xml.etree.ElementTree as E
r = E.parse(sys.argv[1]).getroot()
a = {k: int(r.get(k, 0)) for k in ("tests", "failures", "errors", "skipped", "disabled")}
cases = [(c.get("classname"), c.get("name"), "FAIL" if c.find("failure") is not None else "SKIP" if c.find("skipped") is not None else "ok")
         for c in r.iter("testcase")]
for cl, n, v in cases: print("  %-8s %s.%s" % (v, cl, n))
print("test_p3b_glue:", a)
sys.exit(1 if a["failures"] or a["errors"] or a["tests"] == 0 else 0)
PY
