#!/usr/bin/env bash
# SPDX-License-Identifier: BSD-3-Clause
# remote_test.sh — run ctest of the ROS-free core build on the remote host (after remote_build.sh --core).
#
#   tools/dev/remote_test.sh <name> [--vg] [ctest arguments...]     e.g.  tools/dev/remote_test.sh skel -R test_u01 -V
#
#   --vg   use the valgrind build wt/<name>/build_core_vg of `remote_build.sh --core --vg` (AMENDMENTS A29) instead
#          of wt/<name>/build_core. memcheck (the core CMake has no CTest memcheck setup) through remote_tool.sh, e.g.
#            tools/dev/remote_tool.sh <tree> <name> -- valgrind --error-exitcode=9 /r/wt/<name>/build_core_vg/test_u01_time
#
# Same discipline as remote_build.sh: nice -n 19 inside the container, serialised by the host-side flock,
# project-specific FL2G_* environment names (B1/B4).
set -Eeuo pipefail
NAME=${1:?worktree name}; shift || true
[[ "$NAME" =~ ^[A-Za-z0-9_-]+$ ]] || { echo "remote_test: bad worktree name '$NAME'" >&2; exit 2; }
BUILD=build_core
if [ "${1:-}" = "--vg" ]; then BUILD=build_core_vg; shift; fi
REMOTE=${FL2G_REMOTE:?set FL2G_REMOTE=user@host}
PORT=${FL2G_PORT:-22}
IMAGE=${FL2G_IMAGE:-fastlivo2-gnss:noetic}
ROOT=${FL2G_ROOT:?set FL2G_ROOT to the project directory}
LOCK=$ROOT/.build.lock
ARGS=""; [ $# -gt 0 ] && ARGS=$(printf ' %q' "$@")   # printf with no arguments would emit one '' argument
ssh -p "$PORT" "$REMOTE" "flock -x '$LOCK' sudo -n docker run --rm -u 1000:1000 -e HOME=/r/wt/$NAME/home -e OMP_NUM_THREADS=1 -v $ROOT:/r -w /r/wt/$NAME/$BUILD $IMAGE nice -n 19 bash -c 'ctest --output-on-failure$ARGS'"
