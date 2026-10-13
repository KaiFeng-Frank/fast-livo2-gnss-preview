#!/usr/bin/env bash
# SPDX-License-Identifier: BSD-3-Clause
# launch_remote.sh — local helper of the P3-D tools: sync this tree to a remote worktree and run a long job there
# (fetch, replay matrix) as a transient systemd user unit, which survives the ssh session and the local session.
#
#   launch_remote.sh <local_tree> <wt_name> sync
#   launch_remote.sh <local_tree> <wt_name> start <unit> -- <command ...>   cwd = the synced tree; log <root>/logs/p3d_<unit>.log
#   launch_remote.sh - - status <unit>      |  logs <unit> [lines]  |  stop <unit>
#
# The unit gets Nice=19 / IOSchedulingClass=idle and, because systemd-run passes no caller environment,
# --setenv=ALIYUNPAN_CONFIG_DIR=<root>/tools/apcfg (required by aliyunpan) and FL2G_ROOT. Use a worktree name of its
# own for long jobs (e.g. p3-d-run): re-syncing a tree while a job runs from it would change the scripts under it.
# The sync writes tools/eval/.source_id (git describe --always --dirty + full hash) so every record names its code.
# FL2G_SEED_WT=<worktree> seeds a new remote tree from an existing one on the host before the (then small) network sync.
set -Eeuo pipefail
LOCAL=${1:?local tree or -}; NAME=${2:?worktree name or -}; CMD=${3:?sync|start|status|logs|stop}; shift 3
REMOTE=${FL2G_REMOTE:?set FL2G_REMOTE=user@host}
PORT=${FL2G_PORT:-22}
ROOT=${FL2G_ROOT:?set FL2G_ROOT to the project directory}
SSH=(ssh -p "$PORT" "$REMOTE")
unit() { [[ "$1" =~ ^[A-Za-z0-9_.-]+$ ]] || { echo "bad unit name $1" >&2; exit 2; }; echo "fl2g-p3d-$1"; }
sync_tree() {
  [[ "$NAME" =~ ^[A-Za-z0-9_-]+$ ]] || { echo "bad worktree name" >&2; exit 2; }
  LOCAL=$(realpath -e "$LOCAL")
  [ -f "$LOCAL/package.xml" ] && grep -q '<name>fast_livo</name>' "$LOCAL/package.xml" || { echo "$LOCAL is not the fast_livo tree" >&2; exit 2; }
  local WS=$ROOT/wt/$NAME/ws
  "${SSH[@]}" "mkdir -p '$WS/src/FAST-LIVO2' '$ROOT/wt/$NAME/home' '$ROOT/logs'"
  # FL2G_SEED_WT=<other worktree>: copy that remote tree first (host-local), so that only the delta crosses the network
  # (the link to the host can drop to ~100 KB/s; a fresh 20 MB tree then takes most of an hour)
  if [ -n "${FL2G_SEED_WT:-}" ]; then
    [[ "$FL2G_SEED_WT" =~ ^[A-Za-z0-9_-]+$ ]] || { echo "bad FL2G_SEED_WT" >&2; exit 2; }
    "${SSH[@]}" "rsync -a --delete '$ROOT/wt/$FL2G_SEED_WT/ws/src/FAST-LIVO2/' '$WS/src/FAST-LIVO2/'"
  fi
  rsync -a --delete -e "ssh -p $PORT" \
    --exclude '.git' --exclude '.claude' --exclude 'Log/' --exclude 'pics/' --exclude 'Supplementary/' \
    --exclude '*.pcd' --exclude '*.bag' --exclude '*.pdf' --exclude 'build/' --exclude 'build_core/' --exclude 'devel/' \
    --exclude '__pycache__/' "$LOCAL/" "$REMOTE:$WS/src/FAST-LIVO2/"
  local id; id="$(git -C "$LOCAL" describe --always --dirty 2>/dev/null || echo unknown) $(git -C "$LOCAL" rev-parse HEAD 2>/dev/null || echo unknown)"
  "${SSH[@]}" "echo '$id' > '$WS/src/FAST-LIVO2/tools/eval/.source_id'"
  echo "[launch_remote] synced $LOCAL -> $REMOTE:$WS/src/FAST-LIVO2 ($id)"
}
case "$CMD" in
  sync) sync_tree ;;
  start)
    U=$(unit "${1:?unit}"); shift
    [ "${1:-}" = "--" ] && shift
    [ $# -gt 0 ] || { echo "no command after --" >&2; exit 2; }
    sync_tree
    TREE=$ROOT/wt/$NAME/ws/src/FAST-LIVO2
    LOG=$ROOT/logs/p3d_${U#fl2g-p3d-}.log
    C=$(printf ' %q' "$@")
    "${SSH[@]}" "systemd-run --user --unit='$U' --collect -p Nice=19 -p IOSchedulingClass=idle \
      -p StandardOutput=append:'$LOG' -p StandardError=append:'$LOG' --working-directory='$TREE' \
      --setenv=ALIYUNPAN_CONFIG_DIR='$ROOT/tools/apcfg' --setenv=FL2G_ROOT='$ROOT' --setenv=PYTHONUNBUFFERED=1 \
      --setenv=PATH=/usr/local/bin:/usr/bin:/bin $C && echo '[launch_remote] started unit $U, log $LOG'" ;;
  status) U=$(unit "${1:?unit}"); "${SSH[@]}" "systemctl --user status '$U' --no-pager 2>&1 | head -15 || true" ;;
  logs) U=$(unit "${1:?unit}"); N=${2:-40}; [[ "$N" =~ ^[0-9]+$ ]] || exit 2; "${SSH[@]}" "tail -n $N '$ROOT/logs/p3d_${U#fl2g-p3d-}.log'" ;;
  stop) U=$(unit "${1:?unit}"); "${SSH[@]}" "systemctl --user stop '$U'" ;;
  *) echo "unknown command $CMD" >&2; exit 2 ;;
esac
