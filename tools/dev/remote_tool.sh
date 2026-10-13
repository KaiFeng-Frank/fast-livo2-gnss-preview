#!/usr/bin/env bash
# SPDX-License-Identifier: BSD-3-Clause
# remote_tool.sh — run one of the Python tools (tools/**.py) or the tool unit tests on the remote host, inside the
# project image, with the same discipline as remote_build.sh.
#
#   tools/dev/remote_tool.sh <local_tree> <name> [--no-sync] [--timeout SECONDS] -- <command ...>
#
#   <local_tree>  this repository (checked like remote_build.sh: package.xml name fast_livo, gnss/CMakeLists.txt)
#   <name>        remote worktree name, ^[A-Za-z0-9_-]+$ -> /work/fastlivo2_gnss/wt/<name>/ws/src/FAST-LIVO2
#   <command>     run by bash in the container with cwd = the synced tree; /r = /work/fastlivo2_gnss
#                 (bags under /r/data, the audit archive under /r/audit)
#
# Discipline (docs/dev/DEVELOPING.md "远端纪律"):
#   * the whole run holds `flock -x /work/fastlivo2_gnss/.build.lock`: tool runs are serialised with every
#     build/test on the host (one heavy job at a time next to the GPU training);
#   * `nice -n 19` and `ionice -c3` run INSIDE the container (a nice on the docker client does not reach the job);
#     numpy/BLAS are pinned to one thread; the bag readers drop their own pages from the page cache as they go;
#   * the container is named fl2g-tool-<name> and started with --init. Orphans are removed only AFTER the lock is
#     held: every tool run holds the lock for its whole life, so a fl2g-tool-* container still running while we hold
#     it belongs to a dead session (killed ssh: the flock holder died, the container kept running outside the lock)
#     and is killed; a live same-name job is never killed, the second call just waits for the lock;
#   * a waiter whose client left before it got the lock (ssh killed, Ctrl-C) does not run: it checks that its
#     parent sshd still exists once the lock is held (the job was synced from an older tree);
#   * docker as -u 1000:1000 (no root-owned artefacts); /work/livo_sem is never touched.
set -Eeuo pipefail

die() { echo "remote_tool: $*" >&2; exit 2; }

LOCAL=${1:?local tree}; NAME=${2:?worktree name}; shift 2
SYNC=1; TIMEOUT=7200
while [ $# -gt 0 ]; do
  case "$1" in
    --no-sync) SYNC=0; shift ;;
    --timeout) TIMEOUT=${2:?seconds}; shift 2 ;;
    --) shift; break ;;
    *) die "unknown option $1 (the command follows --)" ;;
  esac
done
[ $# -gt 0 ] || die "no command given after --"
[[ "$NAME" =~ ^[A-Za-z0-9_-]+$ ]] || die "worktree name '$NAME' must match ^[A-Za-z0-9_-]+\$"
[[ "$TIMEOUT" =~ ^[0-9]+$ ]] || die "timeout must be an integer number of seconds"
LOCAL=$(realpath -e "$LOCAL") || die "local tree does not exist"
[ -f "$LOCAL/package.xml" ] && grep -q '<name>fast_livo</name>' "$LOCAL/package.xml" || die "$LOCAL is not the fast_livo checkout"
[ -f "$LOCAL/gnss/CMakeLists.txt" ] || die "$LOCAL/gnss/CMakeLists.txt missing"

REMOTE=${FL2G_REMOTE:?set FL2G_REMOTE=user@host}
PORT=${FL2G_PORT:-22}
IMAGE=${FL2G_IMAGE:-fastlivo2-gnss:noetic}
[[ "$IMAGE" =~ ^[A-Za-z0-9_./:-]+$ ]] || die "bad image name"
ROOT=${FL2G_ROOT:?set FL2G_ROOT to the project directory}
LOCK=$ROOT/.build.lock
WT=$ROOT/wt/$NAME
WS=$WT/ws
SSH=(ssh -p "$PORT" "$REMOTE")
CNAME="fl2g-tool-$NAME"

"${SSH[@]}" "set -e; r=\$(realpath -m '$WT'); case \"\$r\" in '$ROOT/wt/'*) ;; *) echo 'refusing worktree outside wt/' >&2; exit 3;; esac; mkdir -p '$WS/src/FAST-LIVO2' '$WT/home/.mpl' '$WT/home/.ros' '$ROOT/audit'"

if [ "$SYNC" = 1 ]; then
  echo "[remote_tool $(date +%T)] rsync $LOCAL -> $REMOTE:$WS/src/FAST-LIVO2"
  rsync -a --delete -e "ssh -p $PORT" \
    --exclude '.git' --exclude '.claude' --exclude 'Log/' --exclude 'pics/' --exclude 'Supplementary/' \
    --exclude '*.pcd' --exclude '*.bag' --exclude '*.pdf' \
    --exclude 'build/' --exclude 'build_core/' --exclude 'devel/' --exclude '__pycache__/' \
    "$LOCAL/" "$REMOTE:$WS/src/FAST-LIVO2/"
fi

CMD=$(printf ' %q' "$@")
# the command travels base64-encoded so that no quoting inside it can break the ssh/docker/bash layers
B64=$(printf '%s' "$CMD" | base64 -w0)
echo "[remote_tool $(date +%T)] waiting for the build lock, then:$CMD"
# All arguments are shell-safe by construction (NAME/IMAGE regex-checked, TIMEOUT digits, B64 base64, fixed paths),
# so they travel as positional parameters of a remote `bash -s`; the script itself comes on stdin, unexpanded.
"${SSH[@]}" bash -s -- "$LOCK" "$CNAME" "$ROOT" "$NAME" "$IMAGE" "$TIMEOUT" "$B64" <<'REMOTE'
set -Eeuo pipefail
LOCK=$1 CNAME=$2 ROOT=$3 NAME=$4 IMAGE=$5 TIMEOUT=$6 B64=$7
P0=$PPID                     # this ssh session's sshd: it exits when the client goes (ssh killed, Ctrl-C)
exec 9>>"$LOCK"
flock -x 9
# a waiter whose client left while it waited must not run: its command was synced with an older tree and would
# overwrite the outputs of a newer run (observed: a killed local run kept waiting on the host)
kill -0 "$P0" 2>/dev/null || { echo "[remote_tool] the client left while this job waited for the lock: not run" >&2; exit 4; }
# lock held: any tool container still running is an orphan of a dead session (see the header)
orphans=$(sudo -n docker ps -q --filter "name=^fl2g-tool-" </dev/null || true)
if [ -n "$orphans" ]; then
  echo "[remote_tool] killing orphaned tool container(s) (lock was free): $(sudo -n docker ps --format '{{.Names}}' --filter "name=^fl2g-tool-" </dev/null | tr '\n' ' ')" >&2
  sudo -n docker kill $orphans </dev/null >/dev/null || true
fi
sudo -n docker rm -f "$CNAME" </dev/null >/dev/null 2>&1 || true
sudo -n docker run --rm --init --name "$CNAME" -u 1000:1000 \
  -e HOME=/r/wt/$NAME/home -e MPLCONFIGDIR=/r/wt/$NAME/home/.mpl -e ROS_HOME=/r/wt/$NAME/home/.ros \
  -e OMP_NUM_THREADS=1 -e OPENBLAS_NUM_THREADS=1 -e MKL_NUM_THREADS=1 -e PYTHONDONTWRITEBYTECODE=1 \
  -e FL2G_CMD_B64="$B64" -v "$ROOT":/r -w /r/wt/$NAME/ws/src/FAST-LIVO2 "$IMAGE" \
  nice -n 19 ionice -c3 timeout "$TIMEOUT" bash -c 'source /opt/ros/noetic/setup.bash; eval "$(printf %s "$FL2G_CMD_B64" | base64 -d)"' </dev/null
REMOTE
