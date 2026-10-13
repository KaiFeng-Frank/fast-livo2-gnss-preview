#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0-only
# pairwise_distinct.sh — I11 seed sub-gate: every pair of the given runs must have a different state_trace.txt
# (sha256), and the report lists for each pair the first differing line (a number, not a value; A2).
#   pairwise_distinct.sh <run_dir> <run_dir> [...]
set -Eeuo pipefail
n=$#; dirs=("$@"); bad=0
declare -A H
for d in "${dirs[@]}"; do H[$d]=$(sha256sum "$d/state_trace.txt" | cut -c1-64); done
for ((i = 0; i < n; i++)); do
  for ((j = i + 1; j < n; j++)); do
    a=${dirs[i]}; b=${dirs[j]}
    if [ "${H[$a]}" = "${H[$b]}" ]; then echo "SAME     $(basename "$a") $(basename "$b")"; bad=1
    else echo "distinct $(basename "$a") $(basename "$b")  first diff: $(LC_ALL=C cmp "$a/state_trace.txt" "$b/state_trace.txt" 2>&1 | grep -o 'line [0-9]*')"; fi
  done
done
[ $bad = 0 ] && echo "RESULT: all $((n * (n - 1) / 2)) pairs distinct" || echo "RESULT: FAIL (identical pair)"
exit $bad
