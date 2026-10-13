#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0-only
# p3b_gates.sh — local driver of the P3-B gates (DESIGN §13 四道门, TEST_PLAN I4/I5/I14). Heavy steps run on the remote
# host through tools/dev/remote_build.sh / remote_tool.sh (host flock, nice 19 in the container, -u 1000:1000) or
# through p3b_gate_remote.sh (WP-f's replay runner: one container per run, host replay lock). Hashes and counts only.
#
#   p3b_gates.sh <local_tree> <wt> build-eval          remote_build.sh --ros --strict (eval profile) + devel/BUILD_ID
#   p3b_gates.sh <local_tree> <wt> hook-diff           local: git diff ref-R..HEAD of the upstream files — the frozen ones
#                                                      untouched, IMU_Processing.* / LIVMapper.* exactly the registered hook
#                                                      lines (ros/test/gates/hook_lines.expected)
#   p3b_gates.sh <local_tree> <wt> zero   [prefix]     G-零通道 (runs now)
#   p3b_gates.sh <local_tree> <wt> bypass [prefix]     G-旁路   (blocked: facade until P3-A2 is merged)
#   p3b_gates.sh <local_tree> <wt> nodata [prefix]     G-无数据 (blocked: facade until P3-A2 is merged)
#   p3b_gates.sh <local_tree> <wt> nodata-filtered [prefix]   G-无数据 on a bag without GNSS topics (bag_filter.py --drop-gnss)
#   p3b_gates.sh <local_tree> <wt> determinism [prefix]       G-确定性 with GNSS on (T arm, two runs, must ACTIVATE)
#   p3b_gates.sh <local_tree> <wt> i14                 catkin_make tests + rostest ros/test/p3b_glue.test (I14 and the
#                                                      glue unit tests); needs the P3-B CMake lines (INTERFACE_CHANGES P3-B)
# <wt> is the remote worktree of the eval-profile build (e.g. p3-b-strict). Default prefix: p3-b/<wt>.
set -Eeuo pipefail
LOCAL=${1:?local tree}; WT=${2:?wt}; STEP=${3:?step}; PFX=${4:-p3-b/$WT}
REMOTE=${FL2G_REMOTE:?set FL2G_REMOTE=user@host}; PORT=${FL2G_PORT:-22}
ROOT=${FL2G_ROOT:?set FL2G_ROOT to the project directory}
SSH=(ssh -p "$PORT" "$REMOTE")
[[ "$WT" =~ ^[A-Za-z0-9_-]+$ ]] || { echo "bad wt name" >&2; exit 2; }
LOCAL=$(realpath -e "$LOCAL")
say() { echo "[p3b_gates $(date +%T)] $*"; }
case "$STEP" in
  build-eval)
    id=$(git -C "$LOCAL" rev-parse --short HEAD); dirty=$( [ -n "$(git -C "$LOCAL" status --porcelain)" ] && echo 1 || echo 0 )
    "$LOCAL/tools/dev/remote_build.sh" "$LOCAL" "$WT" --ros --strict
    "${SSH[@]}" "echo '$id $dirty eval-profile' > '$ROOT/wt/$WT/ws/devel/BUILD_ID'" ;;
  hook-diff)
    cd "$LOCAL"
    frozen=(include/common_lib.h include/voxel_map.h src/voxel_map.cpp include/vio.h src/vio.cpp include/frame.h src/frame.cpp
            include/visual_point.h src/visual_point.cpp include/preprocess.h src/preprocess.cpp src/main.cpp include/feature.h include/utils)
    bad=0
    if [ -n "$(git diff --stat ref-R HEAD -- "${frozen[@]}")" ]; then echo "frozen upstream files changed since ref-R:"; git diff --stat ref-R HEAD -- "${frozen[@]}"; bad=1
    else echo "frozen upstream files (common_lib.h, voxel_map.*, vio.*, frame.*, visual_point.*, preprocess.*, main.cpp, feature.h, utils/): unchanged since ref-R"; fi
    hooks=(include/IMU_Processing.h src/IMU_Processing.cpp include/LIVMapper.h src/LIVMapper.cpp)
    got=$(git diff -U0 ref-R HEAD -- "${hooks[@]}" | grep -E '^[+-]' | grep -vE '^(\+\+\+|---) ' || true)
    removed=$(printf '%s\n' "$got" | grep -c '^-' || true)
    echo "hook files: $(printf '%s\n' "$got" | grep -c '^+' || true) added lines, $removed removed lines relative to ref-R"
    [ "$removed" -eq 0 ] || { echo "upstream lines were changed or removed"; bad=1; }
    if diff <(printf '%s\n' "$got") "$LOCAL/ros/test/gates/hook_lines.expected" > /dev/null; then echo "added lines = registered hook lines (hook_lines.expected)"
    else echo "added lines differ from the registered hook lines:"; diff <(printf '%s\n' "$got") "$LOCAL/ros/test/gates/hook_lines.expected" | head -40; bad=1; fi
    echo "===== hook-diff: $([ "$bad" = 0 ] && echo PASS || echo FAIL)"
    exit $bad ;;
  zero|bypass|nodata|nodata-filtered|determinism)
    say "sync + remote gate $STEP on wt/$WT (prefix $PFX)"
    rsync -a -e "ssh -p $PORT" "$LOCAL/ros/test/gates/" "$REMOTE:$ROOT/wt/$WT/ws/src/FAST-LIVO2/ros/test/gates/"
    "${SSH[@]}" "bash '$ROOT/wt/$WT/ws/src/FAST-LIVO2/ros/test/gates/p3b_gate_remote.sh' '$STEP' '$WT' '$PFX'" ;;
  i14)
    say "sync + catkin_make tests + rostest p3b_glue.test in wt/$WT (remote_tool.sh: flock, nice 19)"
    "$LOCAL/tools/dev/remote_tool.sh" "$LOCAL" "$WT" -- bash ros/test/gates/p3b_i14_remote.sh "$WT" ;;
  *) echo "unknown step $STEP" >&2; exit 2 ;;
esac
