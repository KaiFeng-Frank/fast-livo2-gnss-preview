#!/usr/bin/env bash
# SPDX-License-Identifier: BSD-3-Clause
# local_build.sh — build on this machine, inside docker, with the same discipline as remote_build.sh.
#
#   tools/dev/local_build.sh <local_tree> <name> [--ros|--core|--all] [--strict] [--vg] [--cmake-arg ARG]...
#                                                 [--clean] [--no-sync]
#
# <name> is a directory under $FL2G_ROOT/wt, not a git worktree. The git checkout stays where it is;
# sources are rsynced into the project root (default: the Extreme SSD) the way remote_build.sh rsynced
# them to the build host. See tools/dev/local_common.sh for FL2G_ROOT / FL2G_BUILD_ROOT / FL2G_IMAGE.
#
# Discipline (docs/dev/DEVELOPING.md, docs/dev/LOCAL_HOST.md): in-container nice -n 19 and ionice -c3,
# docker --cpus=${FL2G_CPUS:-4} (a quota, not --cpuset-cpus), -j at most 4, one build at a time under
# flock on $FL2G_ROOT/.build.lock, docker -u $(id -u):$(id -g), no sudo. Formal catkin/eval builds
# belong on the remote build host; this script is the local smoke path.
# Configure stamp: <build dir>/.fl2g_configure, same rule as remote_build.sh (a changed option list wipes
# that build directory, catkin together with its devel space).
#
# rpg_vikit: pristine tree at $FL2G_VIKIT_SRC (xuankuzcr@6c886c8) plus third_party/patches/rpg_vikit/*.patch.
# A --core build does not need it. --ros/--all refuse to configure until that tree exists. Clone it on a
# filesystem that supports symlinks (the SSD is exFAT): the build root's src/rpg_vikit, not the SSD copy,
# unless FL2G_ROOT itself can hold a git checkout.
set -Eeuo pipefail
FL2G_TAG=local_build
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# shellcheck source=local_common.sh
source "$HERE/local_common.sh"

LOCAL=${1:?local tree}; NAME=${2:?worktree name}; shift 2
MODE=--all; STRICT=0; VG=0; CLEAN=0; SYNC=1; EXTRA=()
while [ $# -gt 0 ]; do
  case "$1" in
    --ros|--core|--all) MODE=$1 ;;
    --strict) STRICT=1 ;;
    --vg) VG=1; STRICT=1 ;;
    --clean) CLEAN=1 ;;
    --no-sync) SYNC=0 ;;
    --cmake-arg) [ $# -ge 2 ] || fl2g_die "--cmake-arg needs a value"; EXTRA+=("$2"); shift ;;
    --cmake-arg=*) EXTRA+=("${1#--cmake-arg=}") ;;
    *) fl2g_die "unknown option $1" ;;
  esac
  shift
done
for a in ${EXTRA[@]+"${EXTRA[@]}"}; do
  [ -n "$a" ] || fl2g_die "--cmake-arg: empty argument"
  case "$a" in *$'\n'*|*$'\r'*) fl2g_die "--cmake-arg: line breaks are not allowed" ;; esac
done

fl2g_check_name "$NAME"
LOCAL=$(realpath -e "$LOCAL") || fl2g_die "local tree does not exist"
fl2g_check_tree "$LOCAL"
[ -f "$LOCAL/tools/dev/local_build.sh" ] || fl2g_die "$LOCAL/tools/dev/local_build.sh missing"
fl2g_init_layout
fl2g_check_root_floor
fl2g_require_image
if [ "$FL2G_IMAGE" != "fastlivo2-gnss:noetic" ]; then
  echo "[local_build] NOTE: FL2G_IMAGE=$FL2G_IMAGE is not fastlivo2-gnss:noetic; do not treat this build as an evaluation build" >&2
fi

FL2G_NAME=$NAME
SUF=""; [ "$VG" = 1 ] && SUF=_vg
WT=$FL2G_ROOT/wt/$NAME
WS=$WT/ws
C_CORE_BUILD=/r/wt/$NAME/build_core$SUF
C_ROS_BUILD=/r/wt/$NAME/ws/build$SUF
C_ROS_DEVEL=/r/wt/$NAME/ws/devel$SUF
mkdir -p "$WS/src/FAST-LIVO2" "$WT/home"

if [ "$SYNC" = 1 ]; then
  fl2g_sync_tree "$LOCAL" "$NAME"
fi

FL2G_NEED_VIKIT=0
if [ "$MODE" = --ros ] || [ "$MODE" = --all ]; then FL2G_NEED_VIKIT=1; fi
if [ -d "$FL2G_VIKIT_SRC" ] || [ "$FL2G_NEED_VIKIT" = 1 ]; then
  if [ ! -d "$FL2G_VIKIT_SRC" ]; then
    fl2g_die "pristine rpg_vikit is not at $FL2G_VIKIT_SRC. Clone xuankuzcr/rpg_vikit@6c886c8 onto a symlink-capable disk (default $FL2G_BUILD_ROOT/src/rpg_vikit) or set FL2G_VIKIT_SRC. --core does not need it"
  fi
  PATCHDIR="$WS/src/FAST-LIVO2/third_party/patches/rpg_vikit"
  STAMP="$WS/src/.vikit_patches_applied"
  want=$( (ls "$PATCHDIR"/*.patch 2>/dev/null || true) | sort | xargs -r sha256sum | sha256sum | cut -c1-16)
  have=$(cat "$STAMP" 2>/dev/null || echo none)
  if [ ! -d "$WS/src/rpg_vikit" ] || [ "$want" != "$have" ]; then
    echo "[local_build] refreshing rpg_vikit from $FL2G_VIKIT_SRC (patches $want)"
    rm -rf "$WS/src/rpg_vikit"
    mkdir -p "$WS/src/rpg_vikit"
    rsync -aL --delete --exclude '.git' "$FL2G_VIKIT_SRC/" "$WS/src/rpg_vikit/"
    for p in $(ls "$PATCHDIR"/*.patch 2>/dev/null | sort); do
      echo "[local_build] applying vikit patch $(basename "$p")"
      patch -p1 -d "$WS/src/rpg_vikit" -i "$p"
    done
    echo "$want" > "$STAMP"
    rm -rf "$FL2G_BUILD_ROOT/wt/$NAME/ws/build" "$FL2G_BUILD_ROOT/wt/$NAME/ws/devel" \
           "$FL2G_BUILD_ROOT/wt/$NAME/ws/build_vg" "$FL2G_BUILD_ROOT/wt/$NAME/ws/devel_vg"
  fi
else
  echo "[local_build] rpg_vikit not present; skipped (--core does not need it)"
fi

if [ "$CLEAN" = 1 ]; then
  if [ "$VG" = 1 ]; then
    echo "[local_build] clean build dirs (--vg)"
    rm -rf "$FL2G_BUILD_ROOT/wt/$NAME/ws/build_vg" "$FL2G_BUILD_ROOT/wt/$NAME/ws/devel_vg" \
           "$FL2G_BUILD_ROOT/wt/$NAME/build_core_vg"
  else
    echo "[local_build] clean build dirs"
    rm -rf "$FL2G_BUILD_ROOT/wt/$NAME/ws/build" "$FL2G_BUILD_ROOT/wt/$NAME/ws/devel" \
           "$FL2G_BUILD_ROOT/wt/$NAME/build_core"
  fi
fi

fl2g_prepare_mounts "$NAME"

# Delete a build tree. When FL2G_BUILD_ROOT is bind-mounted onto /r/wt/<name>/..., that path is a mount
# point and `rm -rf` of the path itself fails with "Device or resource busy"; delete the contents instead.
wipe_tree() {
  local b=$1
  [ -e "$b" ] || return 0
  rm -rf "$b" 2>/dev/null && return 0
  find "$b" -mindepth 1 -delete
}

stamp_or_wipe() {   # <build dir> <devel dir or ""> <stamp text>   (runs in the container)
  local b=$1 d=$2 want=$3 have
  have=$(cat "$b/.fl2g_configure" 2>/dev/null || echo "<no stamp>")
  if [ "$have" != "$want" ] && { [ -e "$b" ] || { [ -n "$d" ] && [ -e "$d" ]; }; }; then
    echo "[local_build] configure options differ from those $b was configured with -> wiping $b${d:+ and $d}"
    wipe_tree "$b"
    [ -n "$d" ] && wipe_tree "$d"
  fi
  mkdir -p "$b"
  printf '%s' "$want" > "$b/.fl2g_configure"
}

run_in_container() {
  local b64
  b64=$(printf '%s' "$1" | base64 -w0)
  docker run --rm --cpus "$FL2G_CPUS" -u "$FL2G_DOCKER_USER" \
    -e HOME="/r/wt/$NAME/home" -e FL2G_SCRIPT_B64="$b64" \
    "${FL2G_MOUNTS[@]}" -w "/r/wt/$NAME" "$FL2G_IMAGE" \
    nice -n 19 ionice -c3 bash -c 'eval "$(printf %s "$FL2G_SCRIPT_B64" | base64 -d)"'
}

fl2g_lock

if [ "$MODE" = --core ] || [ "$MODE" = --all ]; then
  CORE_OPTS=(-DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DLIVO_GNSS_BUILD_TESTS=ON)
  [ "$STRICT" = 1 ] && CORE_OPTS+=(-DLIVO_GNSS_STRICT_FP=ON)
  [ "$VG" = 1 ] && CORE_OPTS+=(-DCMAKE_CXX_FLAGS=-mno-avx)
  CORE_OPTS+=(${EXTRA[@]+"${EXTRA[@]}"})
  CORE_STAMP=$(printf '%s\n' "image=$FL2G_IMAGE" "${CORE_OPTS[@]}")
  echo "[local_build $(date +%T)] core build in wt/$NAME/build_core$SUF (flock, nice 19 in-container, -j$FL2G_JOBS): cmake $(fl2g_q "${CORE_OPTS[@]}")"
  run_in_container "set -e
$(declare -f wipe_tree)
$(declare -f stamp_or_wipe)
stamp_or_wipe $(fl2g_q "$C_CORE_BUILD" "" "$CORE_STAMP")
cmake -S /r/wt/$NAME/ws/src/FAST-LIVO2/gnss -B $(fl2g_q "$C_CORE_BUILD") $(fl2g_q "${CORE_OPTS[@]}")
make -C $(fl2g_q "$C_CORE_BUILD") -j$FL2G_JOBS
cd $(fl2g_q "$C_CORE_BUILD") && ctest --output-on-failure"
fi

if [ "$MODE" = --ros ] || [ "$MODE" = --all ]; then
  ROS_OPTS=(-DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DSophus_DIR=/opt/Sophus/build)
  if [ "$STRICT" = 1 ]; then
    CXXF=-ffp-contract=off; [ "$VG" = 1 ] && CXXF="-ffp-contract=off -mno-avx"
    ROS_OPTS+=(-DFASTLIVO_STRICT_FP=ON -DFASTLIVO_OMP=OFF "-DCMAKE_CXX_FLAGS=$CXXF")
  fi
  ROS_OPTS+=(${EXTRA[@]+"${EXTRA[@]}"})
  DIRS=()
  [ "$VG" = 1 ] && DIRS=(--build "$C_ROS_BUILD" "-DCATKIN_DEVEL_PREFIX=$C_ROS_DEVEL")
  ROS_STAMP=$(printf '%s\n' "image=$FL2G_IMAGE" ${DIRS[@]+"${DIRS[@]}"} "${ROS_OPTS[@]}")
  echo "[local_build $(date +%T)] catkin_make in wt/$NAME/ws/build$SUF (flock, nice 19 in-container, -j$FL2G_JOBS): $(fl2g_q ${DIRS[@]+"${DIRS[@]}"} "${ROS_OPTS[@]}")"
  run_in_container "set -e
$(declare -f wipe_tree)
$(declare -f stamp_or_wipe)
stamp_or_wipe $(fl2g_q "$C_ROS_BUILD" "$C_ROS_DEVEL" "$ROS_STAMP")
source /opt/ros/noetic/setup.bash; cd /r/wt/$NAME/ws; catkin_make --force-cmake -j$FL2G_JOBS $(fl2g_q ${DIRS[@]+"${DIRS[@]}"} "${ROS_OPTS[@]}")"
fi
echo "[local_build $(date +%T)] done: $WT (build files under $FL2G_BUILD_ROOT/wt/$NAME)"
df -P / | awk 'NR==2{printf "[local_build] / now has %.2f GiB free\n", $4/1024/1024}'
