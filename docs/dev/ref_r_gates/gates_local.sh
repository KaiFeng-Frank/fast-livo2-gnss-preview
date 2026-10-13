#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0-only
# gates_local.sh — local driver of the ref-R gates (WP-f). Every heavy step runs on the remote host through
# tools/dev/remote_build.sh (sync + builds) or the remote-side scripts of this directory (which the sync copies to
# wt/<name>/ws/src/FAST-LIVO2/docs/dev/ref_r_gates/). Steps are independent commands so a failed one can be
# repeated; each prints the exact remote command it runs. The two profiles live in two remote worktrees (as the
# maintainer's maint / maint-strict): <name> for the default profile, <name>-strict for the eval profile.
#
#   gates_local.sh <local_tree> <wt_name> build-default        ① remote_build.sh --ros --clean (default profile)
#   gates_local.sh <local_tree> <wt_name> compile-gate         ② compare_compile_commands.py --include-vikit (default build)
#   gates_local.sh <local_tree> <wt_name> build-eval           ① remote_build.sh --ros --strict --clean (eval profile)
#   gates_local.sh <local_tree> <wt_name> vg-build             eval + -mno-avx → ws/build_vg, ws/devel_vg (A29; build_eval.sh
#                                                              until WP-g's `remote_build.sh --vg` is merged)
#   gates_local.sh <local_tree> <wt_name> manifest-check [devel_subdir=ws/devel]
#                                                              writeManifest + extra_json cases (A40) against the built library
#   gates_local.sh <local_tree> <wt_name> seed-gate <devel_subdir> <run_prefix> [bag_rel] [n_seeds=10] [seed_dur=60] [pair_dur=-1]
#                                                              ③ seed 0 twice bit-identical + ④ I11 seeds pairwise distinct
#   gates_local.sh <local_tree> <wt_name> valgrind <devel_subdir> <run_rel> [bag_rel] [duration=60]
#                                                              ⑤ memcheck (detached on the remote; replay lock held)
#   gates_local.sh <local_tree> <wt_name> vg-report <run_rel>  per-origin table of a finished valgrind run
# <wt_name> is the remote worktree the step works on (e.g. p2b-f for build-default/compile-gate, p2b-f-strict for
# the rest). Default bag: the development set Grass02 (A26); evaluation-set bags (Dark01, ...) only for hash
# comparisons (A2).
set -Eeuo pipefail
LOCAL=${1:?local tree}; NAME=${2:?wt name}; STEP=${3:?step}; shift 3
REMOTE=${FL2G_REMOTE:?set FL2G_REMOTE=user@host}; PORT=${FL2G_PORT:-22}
ROOT=${FL2G_ROOT:?set FL2G_ROOT to the project directory}
IMAGE=${FL2G_IMAGE:-fastlivo2-gnss:noetic}
RG="$ROOT/wt/$NAME/ws/src/FAST-LIVO2/docs/dev/ref_r_gates"
SSH=(ssh -p "$PORT" "$REMOTE")
BAG_DEFAULT=data/m3dgr/bags/Grass02.bag
[[ "$NAME" =~ ^[A-Za-z0-9_-]+$ ]] || { echo "bad wt name" >&2; exit 2; }
LOCAL=$(realpath -e "$LOCAL")
id=$(git -C "$LOCAL" rev-parse --short HEAD 2>/dev/null || echo unknown)
dirty=$( [ -n "$(git -C "$LOCAL" status --porcelain 2>/dev/null)" ] && echo 1 || echo 0 )
say() { echo "[gates_local $(date +%T)] $*"; }
case "$STEP" in
  build-default)
    say "remote_build.sh $LOCAL $NAME --ros --clean"
    "$LOCAL/tools/dev/remote_build.sh" "$LOCAL" "$NAME" --ros --clean
    "${SSH[@]}" "echo '$id $dirty default-profile' > '$ROOT/wt/$NAME/ws/devel/BUILD_ID'" ;;
  compile-gate)
    say "compare_compile_commands.py --include-vikit (ws_upstream vs wt/$NAME/ws, default profile)"
    "${SSH[@]}" "cd $ROOT && python3 wt/$NAME/ws/src/FAST-LIVO2/tools/dev/compare_compile_commands.py ws_upstream/build/compile_commands.json wt/$NAME/ws/build/compile_commands.json --head-root /r/wt/$NAME/ws/src --include-vikit" ;;
  build-eval)
    say "remote_build.sh $LOCAL $NAME --ros --strict --clean"
    "$LOCAL/tools/dev/remote_build.sh" "$LOCAL" "$NAME" --ros --strict --clean
    "${SSH[@]}" "echo '$id $dirty eval-profile' > '$ROOT/wt/$NAME/ws/devel/BUILD_ID'" ;;
  vg-build)
    say "build_eval.sh $NAME ws build_vg devel_vg -mno-avx"
    "${SSH[@]}" "bash '$RG/build_eval.sh' '$NAME' ws build_vg devel_vg '$id $dirty' -mno-avx" ;;
  manifest-check)
    DEVEL=${1:-ws/devel}
    [[ "$DEVEL" =~ ^[A-Za-z0-9_/-]+$ ]] || { echo "bad devel dir" >&2; exit 2; }
    OUT=runs/$NAME/manifest_check
    SRC=/r/wt/$NAME/ws/src/FAST-LIVO2
    say "manifest_check.cpp against $DEVEL/lib/liblivo_gnss_ros.so → $OUT (flock, nice 19)"
    "${SSH[@]}" "rm -rf '$ROOT/$OUT' && mkdir -p '$ROOT/$OUT' && flock -x '$ROOT/.build.lock' sudo -n docker run --rm -u 1000:1000 -e HOME=/r/wt/$NAME/home -v $ROOT:/r -w /r/$OUT $IMAGE nice -n 19 bash -c 'set -e; source /r/wt/$NAME/$DEVEL/setup.bash; g++ -std=c++17 -O1 -Wall -Wextra -I$SRC/ros/include -I$SRC/gnss/include -I/usr/include/eigen3 $SRC/docs/dev/ref_r_gates/manifest_check.cpp -L/r/wt/$NAME/$DEVEL/lib -Wl,-rpath,/r/wt/$NAME/$DEVEL/lib -llivo_gnss_ros -llivo_gnss -o /r/$OUT/manifest_check && /r/$OUT/manifest_check /r/$OUT/cases' && python3 '$RG/manifest_check.py' --cases '$ROOT/$OUT/cases'" ;;
  seed-gate)
    DEVEL=${1:?devel_subdir}; PFX=${2:?run_prefix}; BAG=${3:-$BAG_DEFAULT}; N=${4:-10}; SD=${5:-60}; PD=${6:--1}
    say "seed_gate.sh $NAME $DEVEL $PFX $BAG $N $SD $PD (sequential, one replay at a time)"
    "${SSH[@]}" "bash '$RG/seed_gate.sh' '$NAME' '$DEVEL' '$PFX' '$BAG' '$N' '$SD' '$PD'" ;;
  valgrind)
    DEVEL=${1:?devel_subdir}; RUN=${2:?run_rel}; BAG=${3:-$BAG_DEFAULT}; DUR=${4:-60}
    say "valgrind memcheck on $DEVEL, first $DUR s → $RUN (detached on the remote, ~45–75 min for 60 s)"
    "${SSH[@]}" "nohup bash '$RG/replay_run.sh' '$NAME' '$DEVEL' '$RUN' '$BAG' 0 '$DUR' 'prefix:=valgrind --tool=memcheck --track-origins=yes --leak-check=no --num-callers=40 --error-limit=no --log-file=/r/$RUN/valgrind.log' > '$ROOT/wt/$NAME/home/valgrind_$(basename "$RUN").out' 2>&1 &" ;;
  vg-report)
    RUN=${1:?run_rel}
    "${SSH[@]}" "cat '$ROOT/$RUN/run_status.json' 2>/dev/null; python3 '$RG/vg_origin.py' '$ROOT/$RUN/valgrind.log'" ;;
  *) echo "unknown step $STEP" >&2; exit 2 ;;
esac
