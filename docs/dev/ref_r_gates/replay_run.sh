#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0-only
# replay_run.sh — one deterministic replay on the remote host (WP-f, ref-R gates; DESIGN §13 "Log 隔离").
#
#   replay_run.sh <wt_name> <devel_subdir> <run_rel> <bag_rel> <seed> <duration_s> [roslaunch arg ...]
#     wt_name       remote worktree under $FL2G_ROOT/wt (e.g. p2b-f-strict)
#     devel_subdir  catkin devel dir inside the worktree (ws/devel, ws/devel_vg, ...)
#     run_rel       run directory relative to $FL2G_ROOT (e.g. runs/p2b-f/grass02_s0_a); created, never reused
#     bag_rel       bag relative to $FL2G_ROOT (e.g. data/m3dgr/bags/Grass02.bag)
#     seed          k → the k-th LiDAR frame (record order) opens the view (AMENDMENTS A30)
#     duration_s    -1 = whole bag
#     extra         e.g. prefix:='valgrind --tool=memcheck ...'
#
# Discipline: one container per run, -u 1000:1000, `nice -n 19` inside the container, OMP_NUM_THREADS=1,
# private roscore port, ROS_LOG_DIR/ROS_HOME and the upstream Log/ directory bound into the run directory
# (ROOT_DIR is burnt into liblaser_mapping at compile time, CMakeLists.txt:40). Host-wide replay slots: the container
# runs under `flock -x` of one of the 8 slot locks of tools/eval/run_matrix.py (AMENDMENTS A41: at most 8 concurrent
# single-threaded offline replays) — FL2G_REPLAY_SLOT=k (0..7) picks slot k, default 0 = $FL2G_ROOT/.replay.lock
# (the former single A7 lock, so an unchanged caller still runs one at a time); slot k>0 = $FL2G_ROOT/.replay.slot<k>.lock.
# A second runner on the same slot waits. Evaluation-set bags: hashes and line counts only (A2).
#
# Bag identity: <bag>.sha256 next to the bag when present; otherwise sha256sum once (nice 19) into a cache
# runs/.bag_sha256/<bag>.<size>-<mtime>.sha256 — without it the binary hashes the multi-GB bag itself on every run,
# which under valgrind takes longer than the replay.
#
# Completion (review m2 + P2b): roslaunch returns 0 even when the node dies, so the run is complete only when
#   (1) replay_stats.json exists and says "complete": true (it is written last),
#   (2) the console holds the "fastlivo_replay: done" line,
#   (3) the node's own exit code, parsed from the roslaunch console, is 0,
#   (4) run_manifest.json is strict JSON with unique keys and carries every replay statistic (manifest_check.py
#       --run: ManifestInfo::extra_json merge, AMENDMENTS A40).
# run_status.json records all four plus the cycle count; the script exits 0 only when all four hold.
set -Eeuo pipefail
WT=${1:?wt_name}; DEVEL=${2:?devel_subdir}; RUN_REL=${3:?run_rel}; BAG_REL=${4:?bag_rel}; SEED=${5:?seed}; DUR=${6:?duration_s}; shift 6
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=${FL2G_ROOT:?set FL2G_ROOT to the project directory}
IMAGE=${FL2G_IMAGE:-fastlivo2-gnss:noetic}
PORT=${FL2G_ROS_PORT:-11411}
[[ "$WT" =~ ^[A-Za-z0-9_-]+$ ]] || { echo "bad worktree name" >&2; exit 2; }
[[ "$RUN_REL" =~ ^runs/[A-Za-z0-9_./-]+$ ]] || { echo "run_rel must be runs/<...>" >&2; exit 2; }
[[ "$DEVEL" =~ ^[A-Za-z0-9_/-]+$ ]] || { echo "bad devel dir" >&2; exit 2; }
[[ "$BAG_REL" =~ ^[A-Za-z0-9_./-]+$ ]] || { echo "bad bag path" >&2; exit 2; }
SLOT=${FL2G_REPLAY_SLOT:-0}
[[ "$SLOT" =~ ^[0-7]$ ]] || { echo "FL2G_REPLAY_SLOT must be 0..7 (A41)" >&2; exit 2; }
if [ "$SLOT" = 0 ]; then LOCKF="$ROOT/.replay.lock"; else LOCKF="$ROOT/.replay.slot$SLOT.lock"; fi
RUN="$ROOT/$RUN_REL"
[ -e "$RUN/replay_stats.json" ] && { echo "refusing to reuse $RUN (replay_stats.json exists)" >&2; exit 2; }
[ -f "$ROOT/$BAG_REL" ] || { echo "bag $ROOT/$BAG_REL missing" >&2; exit 2; }
NAME=$(basename "$RUN_REL")
mkdir -p "$RUN/Log/result" "$RUN/Log/pcd" "$RUN/Log/image" "$RUN/Log/Colmap/sparse/0" "$RUN/roslog"

# Build identity: written by build_eval.sh / gates_local.sh next to the devel tree; the binary additionally
# self-reports build_sha256 (hashes of the executable and its fast_livo shared objects) in replay_stats.json and
# run_manifest.json.
GIT_DESCRIBE=unknown; GIT_DIRTY=unknown
if [ -f "$ROOT/wt/$WT/$DEVEL/BUILD_ID" ]; then
  read -r GIT_DESCRIBE GIT_DIRTY _ < "$ROOT/wt/$WT/$DEVEL/BUILD_ID"
fi
DIGEST=$(sudo -n docker image inspect --format '{{.Id}}' "$IMAGE" 2>/dev/null || echo "")
if [ -f "$ROOT/$BAG_REL.sha256" ]; then
  BAG_SHA=$(cut -d' ' -f1 "$ROOT/$BAG_REL.sha256")
else
  CACHE="$ROOT/runs/.bag_sha256/$(basename "$BAG_REL").$(stat -c '%s-%Y' "$ROOT/$BAG_REL").sha256"
  if [ ! -s "$CACHE" ]; then
    mkdir -p "$(dirname "$CACHE")"
    nice -n 19 sha256sum "$ROOT/$BAG_REL" | cut -d' ' -f1 > "$CACHE.tmp.$$" && mv "$CACHE.tmp.$$" "$CACHE"
  fi
  BAG_SHA=$(cat "$CACHE")
fi
[[ "$BAG_SHA" =~ ^[0-9a-fA-F]{64}$ ]] || { echo "bag sha256 unavailable for $BAG_REL" >&2; exit 2; }
LOGMOUNT=${LOGMOUNT:-$(dirname "$DEVEL")/src/FAST-LIVO2/Log}   # the source tree whose ROOT_DIR the binary carries

ARGS=""; [ $# -gt 0 ] && ARGS=$(printf ' %q' "$@")   # each extra roslaunch argument stays one word inside bash -c (printf with no args would emit '')
{
  echo "[replay_run $(date +%T)] start $NAME wt=$WT devel=$DEVEL bag=$BAG_REL seed=$SEED dur=$DUR build=$GIT_DESCRIBE/$GIT_DIRTY slot=$SLOT args=$ARGS"
} >> "$RUN/runner.log"
ENVX=()
[ -n "${RCF:-}" ] && ENVX+=(-e "ROSCONSOLE_CONFIG_FILE=$RCF")   # optional: silence rosconsole
set +e
flock -x "$LOCKF" sudo -n docker run --rm -u 1000:1000 --name "fl2g-replay-$WT-$NAME" "${ENVX[@]}" \
  -e HOME="/r/wt/$WT/home" -e OMP_NUM_THREADS=1 -e "FL2G_HOST=$(hostname)" \
  -e ROS_LOG_DIR="/r/$RUN_REL/roslog" -e ROS_HOME="/r/$RUN_REL/roslog" \
  -v "$ROOT:/r" -v "$RUN/Log:/r/wt/$WT/$LOGMOUNT" -w "/r/$RUN_REL" "$IMAGE" \
  nice -n 19 bash -c "source /r/wt/$WT/$DEVEL/setup.bash && exec roslaunch -p $PORT fast_livo m3dgr_avia_replay.launch \
    bag:=/r/$BAG_REL output_dir:=/r/$RUN_REL seed:=$SEED duration:=$DUR seq:=$NAME \
    git_describe:=$GIT_DESCRIBE git_dirty:=$GIT_DIRTY image_digest:=$DIGEST bag_sha256:=$BAG_SHA$ARGS" \
  > "$RUN/console.log" 2>&1
rc=$?
set -e

# ---- completion criterion ----
node_rc=""
if grep -q "process has finished cleanly" "$RUN/console.log"; then node_rc=0; fi
m=$(grep -o "process has died \[pid [0-9]*, exit code -\?[0-9]*" "$RUN/console.log" | head -1 || true)
[ -n "$m" ] && node_rc=${m##*exit code }
done_line=0; grep -q "fastlivo_replay: done" "$RUN/console.log" && done_line=1
stats=0; [ -f "$RUN/replay_stats.json" ] && grep -q '"complete": true' "$RUN/replay_stats.json" && stats=1
cycles=-1; [ -f "$RUN/state_trace.txt" ] && cycles=$(( $(wc -l < "$RUN/state_trace.txt") - 1 ))
manifest_ok=0
if [ "$stats" = 1 ] && python3 "$HERE/manifest_check.py" --run "$RUN" >> "$RUN/runner.log" 2>&1; then manifest_ok=1; fi
complete=0; [ "$stats" = 1 ] && [ "$done_line" = 1 ] && [ "$node_rc" = "0" ] && [ "$manifest_ok" = 1 ] && complete=1
cat > "$RUN/run_status.json" <<JSON
{"complete": $complete, "roslaunch_rc": $rc, "node_exit_code": "${node_rc:-unknown}", "done_line": $done_line, "replay_stats": $stats, "manifest_ok": $manifest_ok, "cycles": $cycles}
JSON
echo "[replay_run $(date +%T)] end $NAME roslaunch_rc=$rc node_rc=${node_rc:-unknown} done_line=$done_line stats=$stats manifest_ok=$manifest_ok cycles=$cycles complete=$complete" >> "$RUN/runner.log"
[ "$complete" = 1 ]
