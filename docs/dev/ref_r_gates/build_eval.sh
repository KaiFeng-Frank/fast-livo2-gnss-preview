#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0-only
# build_eval.sh — eval-profile catkin build into a separate build/devel pair on the remote host (WP-f, ref-R gates).
# Its remaining use is the valgrind variant (AMENDMENTS A29) until WP-g's `remote_build.sh --vg` is merged; the
# eval binary itself comes from `tools/dev/remote_build.sh <tree> <name> --ros --strict` (A28 made that path build).
#
#   build_eval.sh <wt_name> <ws_subdir> <build_subdir> <devel_subdir> <build_id> [extra CXX flags]
#     e.g. build_eval.sh p2b-f-strict ws build_vg devel_vg "abc1234 0" -mno-avx    valgrind variant
#
# Flags = remote_build.sh --strict's catkin step exactly (FASTLIVO_STRICT_FP=ON FASTLIVO_OMP=OFF,
# CMAKE_CXX_FLAGS=-ffp-contract=off; the `-include omp.h` of the upstream TUs now comes from the top-level CMake,
# A28) plus the extra flags. Same discipline: host-side flock, `nice -n 19` inside the container, -j2,
# -u 1000:1000, no --cpuset-cpus.
#
# valgrind variant: valgrind 3.15 (image) decodes neither AVX-512 nor (for one VEX encoding it meets in
# generateVisualMapPoints) AVX; `-march=native` on the Xeon Gold 6248R emits both, so the memcheck build adds
# -mno-avx (which also disables AVX2/AVX-512; explicit -mno-* survives the preceding -march=native). Only for
# memcheck, never for evaluation numbers (A29).
set -Eeuo pipefail
WT=${1:?wt_name}; WS=${2:?ws_subdir}; BUILD=${3:?build_subdir}; DEVEL=${4:?devel_subdir}; BUILD_ID=${5:?build_id}; shift 5
EXTRA="$*"
ROOT=${FL2G_ROOT:?set FL2G_ROOT to the project directory}
IMAGE=${FL2G_IMAGE:-fastlivo2-gnss:noetic}
LOCK=$ROOT/.build.lock
[[ "$WT" =~ ^[A-Za-z0-9_-]+$ ]] || { echo "bad worktree name" >&2; exit 2; }
[[ "$WS$BUILD$DEVEL" =~ ^[A-Za-z0-9_-]+$ ]] || { echo "bad ws/build/devel names" >&2; exit 2; }
[[ "$EXTRA" =~ ^[A-Za-z0-9_=\ -]*$ ]] || { echo "extra flags limited to [A-Za-z0-9_= -]" >&2; exit 2; }
[ -d "$ROOT/wt/$WT/$WS/src/FAST-LIVO2" ] || { echo "$ROOT/wt/$WT/$WS/src/FAST-LIVO2 missing (sync first)" >&2; exit 2; }
COMMON="-DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DSophus_DIR=/opt/Sophus/build -DFASTLIVO_STRICT_FP=ON -DFASTLIVO_OMP=OFF"
FLAGS="-ffp-contract=off${EXTRA:+ $EXTRA}"
LOG="$ROOT/wt/$WT/home/build_${WS}_${BUILD}.log"
mkdir -p "$ROOT/wt/$WT/home"
echo "[build_eval $(date +%T)] $WT/$WS build=$BUILD devel=$DEVEL flags='$FLAGS' id='$BUILD_ID'" | tee -a "$LOG"
DOCKER="sudo -n docker run --rm -u 1000:1000 -e HOME=/r/wt/$WT/home -v $ROOT:/r -w /r/wt/$WT $IMAGE nice -n 19 bash -c"
set +e
flock -x "$LOCK" $DOCKER "set -e; source /opt/ros/noetic/setup.bash; cd /r/wt/$WT/$WS; catkin_make --force-cmake -j2 --build $BUILD -DCATKIN_DEVEL_PREFIX=/r/wt/$WT/$WS/$DEVEL $COMMON '-DCMAKE_CXX_FLAGS=$FLAGS'" >> "$LOG" 2>&1
rc=$?
set -e
echo "[build_eval $(date +%T)] rc=$rc" | tee -a "$LOG"
if [ "$rc" = 0 ]; then
  # BUILD_ID next to the devel tree: "<git describe> <dirty 0|1> <flags>"; replay_run.sh forwards it. The
  # binary's self-reported build_sha256 is the authoritative identity; this label is for humans.
  echo "$BUILD_ID $FLAGS" > "$ROOT/wt/$WT/$WS/$DEVEL/BUILD_ID"
fi
exit $rc
