#!/usr/bin/env bash
# SPDX-License-Identifier: BSD-3-Clause
# probe_replay.sh — one diagnostic replay of an A80 slim bag with free overlays, a start offset and a duration (the extrinsic
# and clock probes of the development sequences). Not an evaluation run: tools/eval/run_matrix.py is the evaluation path, and the
# arms of tools/eval/arms.json are what is scored; this script exists because run_matrix has no start offset and no overlay of
# its own (LIO-only, other extrinsic candidates). It starts the container exactly as run_matrix.py does (same user, nice 19 / idle
# IO inside the container, OMP_NUM_THREADS=1, private roscore, upstream Log/ bound into the run directory, build trees of
# FL2G_BUILD_ROOT over /r/wt/...), at most --cpus cores and --memory, so a probe never takes more than that.
#
#   probe_replay.sh <name> <bag_rel> <replay_launch> <start_offset_s> <duration_s> [overlay.yaml ...]
#     name           run directory <root>/runs/probe/<name> (never reused)
#     bag_rel        bag relative to the root, e.g. smartpnt/bags/Data02_20240718_100C_CAR_complex.bag
#     replay_launch  launch file of the host profile (smartpnt_mini_replay.launch | mars_lvig_replay.launch)
#     overlay.yaml   absolute path as seen from the HOST (under the root), loaded after the profile's config in the order given
#   environment: FL2G_ROOT (project root, default /data2/fl2g), FL2G_BUILD_ROOT (default /work/fastlivo2_gnss), WT (build tree
#   name, default a80-data), PORT (private roscore port, default 11511), CPUS (default 2), MEM (default 12g), IMAGE, SEQ, ARM
set -Eeuo pipefail
NAME=${1:?name}; BAG=${2:?bag_rel}; LAUNCH=${3:?replay_launch}; START=${4:?start_offset_s}; DUR=${5:?duration_s}; shift 5
ROOT=${FL2G_ROOT:-/data2/fl2g}
BROOT=${FL2G_BUILD_ROOT:-/work/fastlivo2_gnss}
WT=${WT:-a80-data}
PORT=${PORT:-11511}
CPUS=${CPUS:-2}
MEM=${MEM:-12g}
IMAGE=${IMAGE:-fastlivo2-gnss:noetic}
SEQ=${SEQ:-probe}
ARM=${ARM:-U}
[[ "$NAME" =~ ^[A-Za-z0-9_.-]+$ ]] || { echo "bad name" >&2; exit 2; }
[[ "$BAG" =~ ^[A-Za-z0-9_./-]+$ ]] || { echo "bad bag path" >&2; exit 2; }
[ -f "$ROOT/$BAG" ] || { echo "bag $ROOT/$BAG missing" >&2; exit 2; }
RUN_REL=runs/probe/$NAME
RUN=$ROOT/$RUN_REL
[ -e "$RUN/replay_stats.json" ] && { echo "refusing to reuse $RUN" >&2; exit 2; }
mkdir -p "$RUN/Log/result" "$RUN/Log/pcd" "$RUN/Log/image" "$RUN/Log/Colmap/sparse/0" "$RUN/roslog"
{
  echo '<launch>'
  echo '  <arg name="bag" /> <arg name="output_dir" /> <arg name="start_offset" default="0.0" /> <arg name="seed" default="0" /> <arg name="duration" default="-1.0" /> <arg name="seq" default="probe" /> <arg name="arm" default="U" /> <arg name="git_describe" default="unknown" /> <arg name="git_dirty" default="unknown" /> <arg name="image_digest" default="" /> <arg name="bag_sha256" default="" /> <arg name="prefix" default="" /> <arg name="extra_args" default="" />'
  echo "  <include file=\"\$(find fast_livo)/launch/$LAUNCH\" pass_all_args=\"true\" />"
  for o in "$@"; do
    echo "  <rosparam command=\"load\" file=\"/r/${o#"$ROOT"/}\" />"
  done
  echo '</launch>'
} > "$RUN/wrap.launch"
VOL=(-v "$ROOT:/r")
for rel in build_core ws/build ws/devel; do
  [ -d "$BROOT/wt/$WT/$rel" ] && [ "$(realpath "$BROOT")" != "$(realpath "$ROOT")" ] && { mkdir -p "$ROOT/wt/$WT/$rel"; VOL+=(-v "$BROOT/wt/$WT/$rel:/r/wt/$WT/$rel"); }
done
SHA=unknown
[ -f "$ROOT/$BAG.sha256" ] && SHA=$(cut -d' ' -f1 "$ROOT/$BAG.sha256")
INNER="source /r/wt/$WT/ws/devel/setup.bash && exec roslaunch -p $PORT /r/$RUN_REL/wrap.launch bag:=/r/$BAG output_dir:=/r/$RUN_REL seed:=0 start_offset:=$START duration:=$DUR seq:=$SEQ arm:=$ARM bag_sha256:=$SHA"
echo "[probe_replay $(date +%T)] $NAME bag=$BAG start=$START dur=$DUR overlays=$* cpus=$CPUS" >> "$RUN/runner.log"
set +e
sudo -n docker run --rm -u 1000:1000 --name "fl2g-probe-$NAME" --cpus "$CPUS" --memory "$MEM" --memory-swap "$MEM" --oom-score-adj 1000 \
  -e "HOME=/r/wt/$WT/home" -e OMP_NUM_THREADS=1 -e "ROS_LOG_DIR=/r/$RUN_REL/roslog" -e "ROS_HOME=/r/$RUN_REL/roslog" \
  "${VOL[@]}" -v "$RUN/Log:/r/wt/$WT/ws/src/FAST-LIVO2/Log" -w "/r/$RUN_REL" "$IMAGE" nice -n 19 ionice -c3 bash -c "$INNER" \
  > "$RUN/console.log" 2>&1 &
DPID=$!
# resident memory of the container every 15 s (the replay's visual map grows with the path, see the README): <run>/mem.csv "epoch_s,usage"
while kill -0 "$DPID" 2>/dev/null; do
  u=$(sudo -n docker stats --no-stream --format '{{.MemUsage}}' "fl2g-probe-$NAME" 2>/dev/null | cut -d/ -f1)
  [ -n "$u" ] && echo "$(date +%s),$u" >> "$RUN/mem.csv"
  sleep 15
done
wait "$DPID"
RC=$?
set -e
if grep -q '"complete": *true' "$RUN/replay_stats.json" 2>/dev/null && grep -q 'fastlivo_replay: done' "$RUN/console.log"; then
  echo "[probe_replay $(date +%T)] $NAME complete (docker rc $RC)" | tee -a "$RUN/runner.log"
else
  echo "[probe_replay $(date +%T)] $NAME INCOMPLETE (docker rc $RC), see $RUN/console.log" | tee -a "$RUN/runner.log"
  exit 1
fi
