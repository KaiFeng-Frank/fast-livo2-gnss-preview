#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0-only
# seed_gate.sh — G-determinism (③) + I11 seed sub-gate (④) on one build, run sequentially on the remote host.
#   seed_gate.sh <wt_name> <devel_subdir> <run_prefix> <bag_rel> [n_seeds=10] [seed_duration_s=60] [pair_duration_s=-1]
# Produces runs/<run_prefix>_s0_a and _s0_b (seed 0 twice over pair_duration_s, -1 = whole bag → must be
# bit-identical) and runs/<run_prefix>_sd<k> (k = 0..n-1, first <seed_duration_s> s → state_trace pairwise
# distinct; n = 0 runs the pair only, e.g. the hash-only Dark01 check), then the hash report. One replay at a time
# (replay_run.sh holds the host-wide replay lock). Exit status 0 only when every run is complete, the pair is
# bit-identical and (n ≥ 2) all seed pairs are distinct.
set -Eeuo pipefail
WT=${1:?}; DEVEL=${2:?}; PFX=${3:?}; BAG=${4:?}; N=${5:-10}; SD=${6:-60}; PD=${7:--1}
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=${FL2G_ROOT:?set FL2G_ROOT to the project directory}
bad=0
for tag in a b; do
  bash "$HERE/replay_run.sh" "$WT" "$DEVEL" "runs/${PFX}_s0_$tag" "$BAG" 0 "$PD" || { echo "run ${PFX}_s0_$tag incomplete"; bad=1; }
done
for ((k = 0; k < N; k++)); do
  bash "$HERE/replay_run.sh" "$WT" "$DEVEL" "runs/${PFX}_sd$k" "$BAG" "$k" "$SD" || { echo "run sd$k incomplete"; bad=1; }
done
echo "===== bitwise (seed 0 twice, duration $PD s; -1 = whole bag)"
bash "$HERE/hash_runs.sh" "$ROOT/runs/${PFX}_s0_a" "$ROOT/runs/${PFX}_s0_b" | tee "$ROOT/runs/${PFX}_bitwise.txt"
# pass = all seven products identical (state_trace, traj_W, filter_cov, mat_pre, mat_out, imu, Log/result) and the
# pair ran the same binary and configuration
grep -q ': line \|: missing\|DIFFERENT' "$ROOT/runs/${PFX}_bitwise.txt" && bad=1
[ "$(grep -c ': identical' "$ROOT/runs/${PFX}_bitwise.txt")" -eq 7 ] || bad=1
if [ "$N" -ge 2 ]; then
  echo "===== pairwise distinct (seeds 0..$((N - 1)), first $SD s)"
  dirs=(); for ((k = 0; k < N; k++)); do dirs+=("$ROOT/runs/${PFX}_sd$k"); done
  bash "$HERE/pairwise_distinct.sh" "${dirs[@]}" | tee "$ROOT/runs/${PFX}_pairwise.txt" || bad=1
fi
echo "===== seed_gate $PFX: $([ $bad = 0 ] && echo PASS || echo FAIL)"
exit $bad
