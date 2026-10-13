#!/usr/bin/env bash
# SPDX-License-Identifier: BSD-3-Clause
# fetch_eval.sh — rolling fetch of M3DGR sequences on the remote host (P3-D). The pipeline itself is fetch_eval.py
# (download -> unpack -> sha256 -> reindex/repair -> GT/BRDC -> audits -> bag_filter -> delete the unfiltered bag);
# this entry point only fixes the environment a systemd unit does not inherit and checks the host tools.
#
#   fetch_eval.sh plan   <seq>...                       listing + disk plan, downloads nothing
#   fetch_eval.sh fetch  <seq> [--evaluation-phase] ...  one sequence (A2: evaluation sequences need the switch)
#   fetch_eval.sh drop   <seq>...                       delete filtered bags whose runs are done
#   fetch_eval.sh status
#
# Long fetches run as a transient user unit so that they survive the ssh session; systemd-run does not pass the
# caller's environment, so the aliyunpan configuration MUST be given with --setenv:
#   systemd-run --user --unit=fl2g-fetch-<seq> --collect -p Nice=19 -p IOSchedulingClass=idle \
#     --setenv=ALIYUNPAN_CONFIG_DIR=/work/fastlivo2_gnss/tools/apcfg --setenv=FL2G_ROOT=${FL2G_ROOT:?set FL2G_ROOT to the project directory} \
#     bash <tree>/tools/eval/fetch_eval.sh fetch <seq>
# (tools/eval/launch_remote.sh does exactly this). Without the variable aliyunpan falls back to ~/.config and is
# not logged in; the default below repeats the project path so that a manual call works too.
set -Eeuo pipefail
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
export FL2G_ROOT=${FL2G_ROOT:?set FL2G_ROOT to the project directory}
export ALIYUNPAN_CONFIG_DIR=${ALIYUNPAN_CONFIG_DIR:-$FL2G_ROOT/tools/apcfg}
for t in python3 7z sha256sum sudo nice ionice curl gunzip; do
  command -v "$t" >/dev/null || { echo "fetch_eval.sh: $t not found on this host" >&2; exit 2; }
done
[ -d "$ALIYUNPAN_CONFIG_DIR" ] || { echo "fetch_eval.sh: ALIYUNPAN_CONFIG_DIR=$ALIYUNPAN_CONFIG_DIR does not exist" >&2; exit 2; }
exec python3 -u "$HERE/fetch_eval.py" "$@"
