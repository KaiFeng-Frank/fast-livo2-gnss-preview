#!/usr/bin/env bash
# SPDX-License-Identifier: BSD-3-Clause
# local_tool.sh — run one command in the project image on this machine (the local counterpart of remote_tool.sh).
#
#   tools/dev/local_tool.sh <local_tree> <name> [--no-sync] [--timeout SECONDS] -- <command ...>
#
# cwd in the container is the synced tree /r/wt/<name>/ws/src/FAST-LIVO2. /r is $FL2G_ROOT
# (bags under /r/data, audit under /r/audit). Build trees are bind-mounted from $FL2G_BUILD_ROOT
# when that differs from $FL2G_ROOT (see local_common.sh).
#
# Discipline: the run holds $FL2G_ROOT/.build.lock; nice -n 19 and ionice -c3 run inside the container;
# numpy/BLAS are pinned to one thread; docker -u $(id -u):$(id -g); no sudo. The container is named
# fl2g-tool-<name> and started with --init. Orphans (name fl2g-tool-*) are removed only after the lock
# is held, same rule as remote_tool.sh.
set -Eeuo pipefail
FL2G_TAG=local_tool
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# shellcheck source=local_common.sh
source "$HERE/local_common.sh"

LOCAL=${1:?local tree}; NAME=${2:?worktree name}; shift 2
SYNC=1; TIMEOUT=7200
while [ $# -gt 0 ]; do
  case "$1" in
    --no-sync) SYNC=0; shift ;;
    --timeout) TIMEOUT=${2:?seconds}; shift 2 ;;
    --) shift; break ;;
    *) fl2g_die "unknown option $1 (the command follows --)" ;;
  esac
done
[ $# -gt 0 ] || fl2g_die "no command given after --"
fl2g_check_name "$NAME"
[[ "$TIMEOUT" =~ ^[0-9]+$ ]] || fl2g_die "timeout must be an integer number of seconds"
LOCAL=$(realpath -e "$LOCAL") || fl2g_die "local tree does not exist"
fl2g_check_tree "$LOCAL"
fl2g_init_layout
fl2g_require_image
FL2G_NAME=$NAME
mkdir -p "$FL2G_ROOT/wt/$NAME/ws/src/FAST-LIVO2" "$FL2G_ROOT/wt/$NAME/home/.mpl" "$FL2G_ROOT/wt/$NAME/home/.ros" "$FL2G_ROOT/audit"
if [ "$SYNC" = 1 ]; then
  fl2g_sync_tree "$LOCAL" "$NAME"
fi
fl2g_prepare_mounts "$NAME"
CMD=$(printf ' %q' "$@")
B64=$(printf '%s' "$CMD" | base64 -w0)
CNAME="fl2g-tool-$NAME"
echo "[local_tool $(date +%T)] waiting for the build lock, then:$CMD"
fl2g_lock
orphans=$(docker ps -q --filter "name=^fl2g-tool-" </dev/null || true)
if [ -n "$orphans" ]; then
  echo "[local_tool] killing orphaned tool container(s) (lock was free): $(docker ps --format '{{.Names}}' --filter 'name=^fl2g-tool-' </dev/null | tr '\n' ' ')" >&2
  # shellcheck disable=SC2086
  docker kill $orphans </dev/null >/dev/null || true
fi
docker rm -f "$CNAME" </dev/null >/dev/null 2>&1 || true
docker run --rm --init --cpus "$FL2G_CPUS" --name "$CNAME" -u "$FL2G_DOCKER_USER" \
  -e HOME="/r/wt/$NAME/home" -e MPLCONFIGDIR="/r/wt/$NAME/home/.mpl" -e ROS_HOME="/r/wt/$NAME/home/.ros" \
  -e OMP_NUM_THREADS=1 -e OPENBLAS_NUM_THREADS=1 -e MKL_NUM_THREADS=1 -e PYTHONDONTWRITEBYTECODE=1 \
  -e FL2G_CMD_B64="$B64" \
  "${FL2G_MOUNTS[@]}" -w "/r/wt/$NAME/ws/src/FAST-LIVO2" "$FL2G_IMAGE" \
  nice -n 19 ionice -c3 timeout "$TIMEOUT" bash -c 'source /opt/ros/noetic/setup.bash; eval "$(printf %s "$FL2G_CMD_B64" | base64 -d)"'
