#!/usr/bin/env bash
# SPDX-License-Identifier: BSD-3-Clause
# Core software checks + a fully invented GNSS activation/scoring example. No ROS/data downloads.
set -Eeuo pipefail
PREVIEW_REPO=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
PREVIEW_BUILD_DIR=${PREVIEW_BUILD_DIR:-"$PREVIEW_REPO/build_preview"}
PREVIEW_PYTHON=${PREVIEW_PYTHON:-python3}
PREVIEW_JOBS=${PREVIEW_JOBS:-2}
if [[ ! "$PREVIEW_JOBS" =~ ^[1-9][0-9]*$ ]]; then
  printf 'PREVIEW_JOBS must be a positive integer\n' >&2
  exit 2
fi
mkdir -p "$PREVIEW_BUILD_DIR"
PREVIEW_BUILD_DIR=$(cd "$PREVIEW_BUILD_DIR" && pwd)
PREVIEW_REPORT_DIR="$PREVIEW_BUILD_DIR/reports/$(date -u +%Y%m%dT%H%M%S)-$$"
mkdir -p "$PREVIEW_REPORT_DIR/core"
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
cmake -S "$PREVIEW_REPO/tools/preview" -B "$PREVIEW_BUILD_DIR" \
  -DCMAKE_BUILD_TYPE=Release -DLIVO_GNSS_STRICT_FP=ON
cmake --build "$PREVIEW_BUILD_DIR" --parallel "$PREVIEW_JOBS"
PREVIEW_CORE_RC=0
GTEST_OUTPUT="xml:$PREVIEW_REPORT_DIR/core/" \
  ctest --test-dir "$PREVIEW_BUILD_DIR" --output-on-failure --parallel 1 || PREVIEW_CORE_RC=$?
"$PREVIEW_PYTHON" "$PREVIEW_REPO/tools/preview/summarize_core.py" "$PREVIEW_REPORT_DIR/core"
if [ "$PREVIEW_CORE_RC" -ne 0 ]; then exit "$PREVIEW_CORE_RC"; fi
"$PREVIEW_PYTHON" "$PREVIEW_REPO/tools/preview/run_python_tests.py" "$PREVIEW_REPORT_DIR"
"$PREVIEW_PYTHON" "$PREVIEW_REPO/tools/preview/recompute_results.py"
PREVIEW_DEMO_DIR="$PREVIEW_REPORT_DIR/synthetic"
"$PREVIEW_PYTHON" "$PREVIEW_REPO/tools/preview/make_nav.py" "$PREVIEW_DEMO_DIR/synthetic.nav"
"$PREVIEW_BUILD_DIR/preview_demo" "$PREVIEW_DEMO_DIR/synthetic.nav" "$PREVIEW_DEMO_DIR"
"$PREVIEW_PYTHON" "$PREVIEW_REPO/tools/preview/score_demo.py" "$PREVIEW_DEMO_DIR"
printf '\nPreview checks completed. Reports and synthetic trajectories: %s\n' "$PREVIEW_REPORT_DIR"
