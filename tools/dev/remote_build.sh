#!/usr/bin/env bash
# SPDX-License-Identifier: BSD-3-Clause
# remote_build.sh — sync the local tree to the remote build host and build there inside docker.
#
#   tools/dev/remote_build.sh <local_tree> <name> [--ros|--core|--all] [--strict] [--vg] [--cmake-arg ARG]...
#                                                  [--clean] [--no-sync]
#
#   <local_tree>  local FAST-LIVO2 checkout (this repository; checked: package.xml name fast_livo, gnss/CMakeLists.txt)
#   <name>        remote worktree name, ^[A-Za-z0-9_-]+$ -> /work/fastlivo2_gnss/wt/<name>/{ws,build_core,home}
#   --core        cmake -S gnss (ROS-free core + rtklib + gnss_io + all gtest targets) && make -j${FL2G_JOBS:-10} && ctest
#   --ros         catkin_make -j${FL2G_JOBS:-10} of the whole fast_livo package (fastlivo_mapping + fastlivo_replay)
#   --all         both (default)
#   --strict      eval profile: FASTLIVO_STRICT_FP=ON FASTLIVO_OMP=OFF (core: LIVO_GNSS_STRICT_FP=ON) and, for
#                 catkin, CMAKE_CXX_FLAGS=-ffp-contract=off so the vikit packages get the flag too (WP-f's vikit patch
#                 makes vikit append to CMAKE_CXX_FLAGS instead of overriding it, AMENDMENTS A22)
#   --vg          valgrind build (AMENDMENTS A29): the --strict profile plus -mno-avx in CMAKE_CXX_FLAGS, in its own
#                 directories wt/<name>/build_core_vg and wt/<name>/ws/{build_vg,devel_vg} (the default/--strict
#                 directories are never touched). valgrind 3.15 in the image decodes neither AVX-512 nor the VEX
#                 encodings -march=native emits on the build host; an explicit -mno-avx wins over -march=native and
#                 also disables AVX2/AVX-512 (REF_R_GATES §1). For memcheck only: this build's numbers (ctest goldens,
#                 trajectories) are never evaluation numbers. ctest: tools/dev/remote_test.sh <name> --vg ...
#   --cmake-arg A one argument passed verbatim to cmake (--core) and to catkin_make (--ros); repeatable (AMENDMENTS
#                 A40), e.g. --cmake-arg -DLIVO_RTKLIB_APPLY_PATCHES=OFF. It comes after the profile's own options,
#                 so a -D of a variable the profile sets overrides the profile (a CMAKE_CXX_FLAGS given here replaces
#                 the -ffp-contract=off of --strict: repeat that flag inside it when you need it).
#   --clean       remove the remote build directories of the selected profile first (default/--strict: ws/build,
#                 ws/devel, build_core; --vg: ws/build_vg, ws/devel_vg, build_core_vg)
#   --no-sync     skip the rsync (build what is already on the remote)
#
# Configure stamp (what makes --cmake-arg safe): CMake caches every -D, so an option given to one run would stay in
# force for every later run on the same directory without being visible on its command line (a golden-file build with
# -DLIVO_RTKLIB_APPLY_PATCHES=OFF followed by a plain build would test the unpatched library against its own golden;
# a --strict run followed by a default one would keep the eval profile). Every build directory therefore records the
# image and the exact option list it was configured with (<build dir>/.fl2g_configure). When the next build's list
# differs, or the directory predates the stamp, that build directory (catkin: build and devel space) is wiped before
# configuring; the same options give the usual incremental build. The stamp check runs inside the build container,
# under the host lock.
#
# Remote discipline (docs/dev/DEVELOPING.md, verified B1):
#   * `nice -n 19` runs INSIDE the container as the process that forks cmake/make/ctest (a nice on the docker
#     client does not reach the build; verified NI=19 on the compiler processes only this way);
#   * one build at a time on the host: everything runs under `flock -x /work/fastlivo2_gnss/.build.lock`
#     (A41: the GPU training uses ~1 core; builds are serialised by flock and run at nice 19, so -j10 is safe);
#   * no --cpuset-cpus: it changes nproc, hence MP_PROC_NUM, hence the compile-command gate;
#   * docker as -u 1000:1000, never root-owned artefacts; /work/livo_sem is never touched.
# The container's commands travel base64-encoded in an environment variable (as in remote_tool.sh), so a
# --cmake-arg with spaces or quotes reaches cmake as exactly one argument through the ssh/flock/docker/bash layers.
# Environment: FL2G_REMOTE (user@host), FL2G_PORT, FL2G_IMAGE — project-specific names so another project's
# REMOTE/PORT cannot redirect this script (B4).
set -Eeuo pipefail

die() { echo "remote_build: $*" >&2; exit 2; }

LOCAL=${1:?local tree}; NAME=${2:?worktree name}; shift 2
MODE=--all; STRICT=0; VG=0; CLEAN=0; SYNC=1; EXTRA=()
while [ $# -gt 0 ]; do
  case "$1" in
    --ros|--core|--all) MODE=$1 ;;
    --strict) STRICT=1 ;;
    --vg) VG=1; STRICT=1 ;;
    --clean) CLEAN=1 ;;
    --no-sync) SYNC=0 ;;
    --cmake-arg) [ $# -ge 2 ] || die "--cmake-arg needs a value"; EXTRA+=("$2"); shift ;;
    --cmake-arg=*) EXTRA+=("${1#--cmake-arg=}") ;;
    *) die "unknown option $1" ;;
  esac
  shift
done
for a in ${EXTRA[@]+"${EXTRA[@]}"}; do
  [ -n "$a" ] || die "--cmake-arg: empty argument"
  case "$a" in *$'\n'*|*$'\r'*) die "--cmake-arg: line breaks are not allowed" ;; esac
done

# ---- argument checks (B4): the remote rm -rf / rsync --delete targets are built from these ----
[[ "$NAME" =~ ^[A-Za-z0-9_-]+$ ]] || die "worktree name '$NAME' must match ^[A-Za-z0-9_-]+\$"
LOCAL=$(realpath -e "$LOCAL") || die "local tree does not exist"
[ -f "$LOCAL/package.xml" ] || die "$LOCAL/package.xml missing: not the FAST-LIVO2 checkout"
grep -q '<name>fast_livo</name>' "$LOCAL/package.xml" || die "$LOCAL/package.xml is not package fast_livo"
[ -f "$LOCAL/gnss/CMakeLists.txt" ] || die "$LOCAL/gnss/CMakeLists.txt missing"
[ -f "$LOCAL/tools/dev/remote_build.sh" ] || die "$LOCAL/tools/dev/remote_build.sh missing"

REMOTE=${FL2G_REMOTE:?set FL2G_REMOTE=user@host}
PORT=${FL2G_PORT:-22}
IMAGE=${FL2G_IMAGE:-fastlivo2-gnss:noetic}
[[ "$IMAGE" =~ ^[A-Za-z0-9_./:-]+$ ]] || die "bad image name"
ROOT=${FL2G_ROOT:?set FL2G_ROOT to the project directory}
LOCK=$ROOT/.build.lock
WT=$ROOT/wt/$NAME
WS=$WT/ws
SSH=(ssh -p "$PORT" "$REMOTE")
SUF=""; [ "$VG" = 1 ] && SUF=_vg
# container paths (ROOT is mounted at /r)
C_CORE_BUILD=/r/wt/$NAME/build_core$SUF
C_ROS_BUILD=/r/wt/$NAME/ws/build$SUF
C_ROS_DEVEL=/r/wt/$NAME/ws/devel$SUF

say() { echo "[remote_build $(date +%T)] $*"; }
q() { if [ $# -gt 0 ]; then printf '%q ' "$@"; fi; }   # shell-quote each argument for the container's bash

# Runs in the container before configuring (see "Configure stamp" above).
stamp_or_wipe() {   # <build dir> <devel dir or ""> <stamp text>
  local b=$1 d=$2 want=$3 have
  have=$(cat "$b/.fl2g_configure" 2>/dev/null || echo "<no stamp>")
  if [ "$have" != "$want" ] && { [ -e "$b" ] || { [ -n "$d" ] && [ -e "$d" ]; }; }; then
    echo "[remote_build] configure options differ from those $b was configured with -> wiping $b${d:+ and $d}"
    rm -rf "$b" ${d:+"$d"}
  fi
  mkdir -p "$b"
  printf '%s' "$want" > "$b/.fl2g_configure"
}

# Runs <script> (bash) in the project image: under the host flock, `nice -n 19` inside the container, as 1000:1000.
# The remote side waits for the lock in a `bash -s` whose parent is this ssh session's sshd; a waiter whose client
# is gone by the time it gets the lock (ssh killed, Ctrl-C) exits instead of building. Without that check a stale
# waiter would later wipe and rebuild the directory with its old options (configure stamp) behind the next build.
run_in_container() {
  local b64
  b64=$(printf '%s' "$1" | base64 -w0)
  "${SSH[@]}" bash -s -- "$LOCK" "$NAME" "$ROOT" "$IMAGE" "$b64" <<'REMOTE_RUN'
set -Eeuo pipefail
LOCK=$1 NAME=$2 ROOT=$3 IMAGE=$4 B64=$5
P0=$PPID
exec 9>>"$LOCK"
flock -x 9
kill -0 "$P0" 2>/dev/null || { echo "[remote_build] the client left while this build waited for the lock: not run" >&2; exit 4; }
sudo -n docker run --rm -u 1000:1000 -e HOME="/r/wt/$NAME/home" -e FL2G_SCRIPT_B64="$B64" -v "$ROOT":/r -w "/r/wt/$NAME" "$IMAGE" \
  nice -n 19 bash -c 'eval "$(printf %s "$FL2G_SCRIPT_B64" | base64 -d)"' </dev/null
REMOTE_RUN
}

# ---- 0. remote guard: the worktree must resolve under $ROOT/wt (belt and braces after the regex) ----
"${SSH[@]}" "set -e; r=\$(realpath -m '$WT'); case \"\$r\" in '$ROOT/wt/'*) ;; *) echo 'refusing worktree outside wt/' >&2; exit 3;; esac; mkdir -p '$WS/src/FAST-LIVO2' '$WT/home' '$WS/src/FAST-LIVO2/Log'"

# ---- 1. sync sources (never .git, never Log/, never pics/Supplementary (m18), never local build dirs) ----
if [ "$SYNC" = 1 ]; then
  say "rsync $LOCAL -> $REMOTE:$WS/src/FAST-LIVO2"
  rsync -a --delete -e "ssh -p $PORT" \
    --exclude '.git' --exclude '.claude' --exclude 'Log/' --exclude 'pics/' --exclude 'Supplementary/' \
    --exclude '*.pcd' --exclude '*.bag' --exclude '*.pdf' \
    --exclude 'build/' --exclude 'build_core/' --exclude 'devel/' --exclude '__pycache__/' \
    "$LOCAL/" "$REMOTE:$WS/src/FAST-LIVO2/"
fi

# ---- 2. rpg_vikit: pristine copy (xuankuzcr@6c886c8) + third_party/patches/rpg_vikit/*.patch in name order.
#         Re-copied whenever the applied patch set differs from the current one (stamp file). ----
"${SSH[@]}" bash -s <<REMOTE_SCRIPT
set -Eeuo pipefail
WS='$WS'; ROOT='$ROOT'
PATCHDIR="\$WS/src/FAST-LIVO2/third_party/patches/rpg_vikit"
STAMP="\$WS/src/.vikit_patches_applied"
want=\$( (ls "\$PATCHDIR"/*.patch 2>/dev/null || true) | sort | xargs -r sha256sum | sha256sum | cut -c1-16)
have=\$(cat "\$STAMP" 2>/dev/null || echo none)
if [ ! -d "\$WS/src/rpg_vikit" ] || [ "\$want" != "\$have" ]; then
  rm -rf "\$WS/src/rpg_vikit"
  cp -a "\$ROOT/src/rpg_vikit" "\$WS/src/rpg_vikit"
  rm -rf "\$WS/src/rpg_vikit/.git"
  for p in \$(ls "\$PATCHDIR"/*.patch 2>/dev/null | sort); do
    echo "[remote_build] applying vikit patch \$(basename "\$p")"
    patch -p1 -d "\$WS/src/rpg_vikit" -i "\$p"
  done
  echo "\$want" > "\$STAMP"
  # a vikit source change invalidates the catkin build trees (default/--strict and --vg)
  rm -rf "\$WS/build" "\$WS/devel" "\$WS/build_vg" "\$WS/devel_vg"
fi
REMOTE_SCRIPT

if [ "$CLEAN" = 1 ]; then
  if [ "$VG" = 1 ]; then
    say "clean remote build dirs (--vg)"
    "${SSH[@]}" "rm -rf '$WS/build_vg' '$WS/devel_vg' '$WT/build_core_vg'"
  else
    say "clean remote build dirs"
    "${SSH[@]}" "rm -rf '$WS/build' '$WS/devel' '$WT/build_core'"
  fi
fi

# No OMP_NUM_THREADS here: CMake's ProcessorCount() calls `nproc`, which honours OMP_NUM_THREADS and would turn
# the upstream MP_EN/MP_PROC_NUM branch into the single-core one (compile-command gate failure). Runs, not
# builds, set OMP_NUM_THREADS=1.

# ---- 3. core: ROS-free build of gnss/ (rtklib + gnss_io + all tests) ----
if [ "$MODE" = --core ] || [ "$MODE" = --all ]; then
  CORE_OPTS=(-DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DLIVO_GNSS_BUILD_TESTS=ON)
  [ "$STRICT" = 1 ] && CORE_OPTS+=(-DLIVO_GNSS_STRICT_FP=ON)
  [ "$VG" = 1 ] && CORE_OPTS+=(-DCMAKE_CXX_FLAGS=-mno-avx)
  CORE_OPTS+=(${EXTRA[@]+"${EXTRA[@]}"})
  CORE_STAMP=$(printf '%s\n' "image=$IMAGE" "${CORE_OPTS[@]}")
  say "core build in ${C_CORE_BUILD#/r/} (flock, nice 19 in-container, -j${FL2G_JOBS:-10}): cmake $(q "${CORE_OPTS[@]}")"
  run_in_container "set -e
$(declare -f stamp_or_wipe)
stamp_or_wipe $(q "$C_CORE_BUILD" "" "$CORE_STAMP")
cmake -S /r/wt/$NAME/ws/src/FAST-LIVO2/gnss -B $(q "$C_CORE_BUILD") $(q "${CORE_OPTS[@]}")
make -C $(q "$C_CORE_BUILD") -j${FL2G_JOBS:-10}
cd $(q "$C_CORE_BUILD") && ctest --output-on-failure"
fi

# ---- 4. ros: catkin_make of the whole package ----
if [ "$MODE" = --ros ] || [ "$MODE" = --all ]; then
  # Sophus a621ff2 is built in-tree in the image (/opt/Sophus/build holds SophusConfig.cmake), same as ws_upstream.
  ROS_OPTS=(-DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DSophus_DIR=/opt/Sophus/build)
  if [ "$STRICT" = 1 ]; then
    CXXF=-ffp-contract=off; [ "$VG" = 1 ] && CXXF="-ffp-contract=off -mno-avx"
    ROS_OPTS+=(-DFASTLIVO_STRICT_FP=ON -DFASTLIVO_OMP=OFF "-DCMAKE_CXX_FLAGS=$CXXF")
  fi
  ROS_OPTS+=(${EXTRA[@]+"${EXTRA[@]}"})
  DIRS=()   # default/--strict: catkin's own ws/build and ws/devel (the command line of the compile-command gate)
  [ "$VG" = 1 ] && DIRS=(--build "$C_ROS_BUILD" "-DCATKIN_DEVEL_PREFIX=$C_ROS_DEVEL")
  ROS_STAMP=$(printf '%s\n' "image=$IMAGE" ${DIRS[@]+"${DIRS[@]}"} "${ROS_OPTS[@]}")
  say "catkin_make in ${C_ROS_BUILD#/r/} (flock, nice 19 in-container, -j${FL2G_JOBS:-10}): $(q ${DIRS[@]+"${DIRS[@]}"} "${ROS_OPTS[@]}")"
  run_in_container "set -e
$(declare -f stamp_or_wipe)
stamp_or_wipe $(q "$C_ROS_BUILD" "$C_ROS_DEVEL" "$ROS_STAMP")
source /opt/ros/noetic/setup.bash; cd /r/wt/$NAME/ws; catkin_make --force-cmake -j${FL2G_JOBS:-10} $(q ${DIRS[@]+"${DIRS[@]}"} "${ROS_OPTS[@]}")"
fi
say "done: $WT"
