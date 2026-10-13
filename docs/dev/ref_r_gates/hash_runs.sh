#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0-only
# hash_runs.sh — sha256 of the compared products of replay runs and, for the first two runs, the first
# differing line of each file. Prints hashes, line numbers and counts only (AMENDMENTS A2: no trajectory value
# of an evaluation-set sequence is ever displayed).
#   hash_runs.sh <run_dir> [<run_dir> ...]
# Pair lines (first two runs): "<file>: identical" or "<file>: line <n>" (first differing line), for the seven
# compared products, then "identity: build_sha256 <same|DIFFERENT>, config_sha256 <same|DIFFERENT>".
set -Eeuo pipefail
bsha() { { grep -o '"build_sha256": "[0-9a-f]*' "$1/replay_stats.json" 2>/dev/null || true; } | cut -d'"' -f4; }
csha() { { grep -o '"config_sha256": "[0-9a-f]*' "$1/run_manifest.json" 2>/dev/null || true; } | cut -d'"' -f4; }
evo() { local f; for f in "$1"/Log/result/*.txt; do if [ -f "$f" ]; then echo "$f"; break; fi; done; return 0; }
for D in "$@"; do
  st=$(cat "$D/run_status.json" 2>/dev/null || echo "?")
  echo "== $D"
  echo "   status=$st"
  echo "   offset=$({ grep -o '"offset_effective_s": [0-9.]*' "$D/replay_stats.json" 2>/dev/null || true; } | cut -d' ' -f2)  build_sha256=$(bsha "$D" | cut -c1-16)  config_sha256=$(csha "$D" | cut -c1-16)"
  for f in state_trace.txt traj_W.tum filter_cov.csv Log/imu.txt Log/mat_pre.txt Log/mat_out.txt; do
    [ -f "$D/$f" ] && printf "   %-18s %s  (%s lines)\n" "$f" "$(sha256sum "$D/$f" | cut -c1-64)" "$(wc -l < "$D/$f")"
  done
  e=$(evo "$D"); [ -n "$e" ] && printf "   %-18s %s  (%s lines)\n" "Log/result" "$(sha256sum "$e" | cut -c1-64)" "$(wc -l < "$e")"
done
if [ $# -ge 2 ]; then
  A=$1; B=$2
  echo "== pair ($A vs $B):"
  cmpf() {   # <label> <fileA> <fileB>
    if [ ! -f "$2" ] || [ ! -f "$3" ]; then echo "   $1: missing"
    elif cmp -s "$2" "$3"; then echo "   $1: identical"
    else echo "   $1: $(LC_ALL=C cmp "$2" "$3" 2>&1 | grep -o 'line [0-9]*' || echo 'line ? (length)')"; fi
  }
  for f in state_trace.txt traj_W.tum filter_cov.csv Log/mat_pre.txt Log/mat_out.txt Log/imu.txt; do cmpf "$f" "$A/$f" "$B/$f"; done
  cmpf "Log/result" "$(evo "$A")" "$(evo "$B")"
  echo "   identity: build_sha256 $([ -n "$(bsha "$A")" ] && [ "$(bsha "$A")" = "$(bsha "$B")" ] && echo same || echo DIFFERENT), config_sha256 $([ -n "$(csha "$A")" ] && [ "$(csha "$A")" = "$(csha "$B")" ] && echo same || echo DIFFERENT)"
fi
exit 0
