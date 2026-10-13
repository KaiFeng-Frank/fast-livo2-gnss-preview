#!/usr/bin/env bash
# SPDX-License-Identifier: BSD-3-Clause
# run_tool_tests.sh — the WP-g Python tool tests (tools/tests, unittest) inside fastlivo2-gnss:noetic on the remote
# host, through remote_tool.sh (flock on the build lock, nice 19 / ionice idle in the container).
#
#   tools/dev/run_tool_tests.sh <local_tree> <name> [--no-sync] [unittest pattern, default test_*.py]
set -Eeuo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
LOCAL=${1:?local tree}; NAME=${2:?worktree name}; shift 2
OPTS=()
if [ "${1:-}" = "--no-sync" ]; then OPTS+=(--no-sync); shift; fi
PATTERN=${1:-test_*.py}
exec "$HERE/remote_tool.sh" "$LOCAL" "$NAME" "${OPTS[@]}" --timeout 1800 -- python3 -m unittest discover -s tools/tests -p "$PATTERN" -v
