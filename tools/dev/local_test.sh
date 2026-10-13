#!/usr/bin/env bash
# SPDX-License-Identifier: BSD-3-Clause
# local_test.sh — ctest of a core build produced by local_build.sh --core.
#
#   tools/dev/local_test.sh <name> [--vg] [ctest arguments...]
#
# Same discipline as local_build.sh: flock, in-container nice -n 19 and ionice -c3, docker --cpus,
# docker -u $(id -u):$(id -g), no sudo. CTest itself is one process at a time.
# OMP_NUM_THREADS=1 (a build must not set that: it changes nproc and the compile-command gate).
set -Eeuo pipefail
FL2G_TAG=local_test
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# shellcheck source=local_common.sh
source "$HERE/local_common.sh"

NAME=${1:?worktree name}; shift || true
fl2g_check_name "$NAME"
BUILD=build_core
if [ "${1:-}" = "--vg" ]; then BUILD=build_core_vg; shift; fi
fl2g_init_layout
fl2g_require_image
FL2G_NAME=$NAME
fl2g_prepare_mounts "$NAME"
HOST_BUILD=$FL2G_BUILD_ROOT/wt/$NAME/$BUILD
[ -d "$HOST_BUILD" ] || fl2g_die "no build at $HOST_BUILD (run local_build.sh --core first)"
ARGS=""; [ $# -gt 0 ] && ARGS=$(printf ' %q' "$@")
fl2g_lock
docker run --rm --cpus "$FL2G_CPUS" -u "$FL2G_DOCKER_USER" \
  -e HOME="/r/wt/$NAME/home" -e OMP_NUM_THREADS=1 \
  "${FL2G_MOUNTS[@]}" -w "/r/wt/$NAME/$BUILD" "$FL2G_IMAGE" \
  nice -n 19 ionice -c3 bash -c "ctest --output-on-failure$ARGS"
