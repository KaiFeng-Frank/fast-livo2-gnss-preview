#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0-only
# p3b_gate_remote.sh — remote side of the P3-B replay gates (DESIGN §13 四道门; TEST_PLAN I4/I5). Runs on the remote host
# from the synced tree wt/<wt>/ws/src/FAST-LIVO2/ros/test/gates/, drives WP-f's runner docs/dev/ref_r_gates/replay_run.sh
# (own container per run, host replay lock, nice 19 in the container, OMP_NUM_THREADS=1, Log/ bound per run) and
# compares products with hashes and line numbers only.
#
#   p3b_gate_remote.sh zero   <wt> <prefix>   G-零通道: HEAD with the shipped config (gnss.enable false) vs ref-R
#   p3b_gate_remote.sh bypass <wt> <prefix>   G-旁路:   enable true, update_enable false (OBSERVE) vs ref-R, host columns
#   p3b_gate_remote.sh nodata <wt> <prefix>   G-无数据: enable true, the bag's GNSS topics never reach the bridge
#   p3b_gate_remote.sh nodata-filtered <wt> <prefix>   G-无数据 on a bag without the GNSS topics (tools/bag_filter.py
#                                              --drop-gnss, made once per prefix): enable true, 7 products vs ref-R
#   p3b_gate_remote.sh determinism <wt> <prefix>       G-确定性 with GNSS on (the T arm): two runs, every product —
#                                              state_trace with x_G/diag Q_G, traj_N, gnss_epochs.csv without its
#                                              wall-clock column, events.log — bit-identical; the run must ACTIVATE
# (nodata-filtered and determinism: added by the maintainer in the P3 integration, AMENDMENTS「P3 集成记录」.)
#
# Every gate replays the development-set bag Grass02 (A26) whole, seed 0, eval profile (<wt> = an --strict worktree).
# The ref-R run is produced once per prefix from the ref-R eval build wt/${FL2G_REF_WT:-p2b-f-strict}/ws/devel
# (REF_R_GATES §1: build 35ee6c5, build_sha256 f453a0fb…) and its hashes are checked against the recorded ref-R pair
# (REF_R_GATES §3a) first, so a drift of the machine or image shows up as a ref-R failure, not as a gate verdict.
# bypass/nodata need the GNSS core (P3-A2 facade): until it is merged the HEAD run stops at start-up with
# "gnss configuration rejected: GNSS core: not implemented" and the gate reports "blocked: facade".
set -Eeuo pipefail
GATE=${1:?gate}; WT=${2:?wt}; PFX=${3:?prefix}
ROOT=${FL2G_ROOT:?set FL2G_ROOT to the project directory}
REF_WT=${FL2G_REF_WT:-p2b-f-strict}
[[ "$WT" =~ ^[A-Za-z0-9_-]+$ && "$REF_WT" =~ ^[A-Za-z0-9_-]+$ ]] || { echo "bad worktree name" >&2; exit 2; }
[[ "$PFX" =~ ^[A-Za-z0-9_./-]+$ ]] || { echo "bad prefix" >&2; exit 2; }
HERE=$(cd "$(dirname "$0")" && pwd)
RG=$(cd "$HERE/../../../docs/dev/ref_r_gates" && pwd)
BAG=data/m3dgr/bags/Grass02.bag
NAV=/r/data/brdc/BRDC00IGS_R_20250210000_01D_MN.rnx          # Grass02 = 2025-01-21 (DOY 021), A26
CALIB=/r/audit/Grass02/time_calib/Grass02.yaml                 # WP-g time calibration (A18; record_time prior)
REF=runs/$PFX/refR_grass02
mkdir -p "$ROOT/runs/$PFX"

# Recorded ref-R hashes of Grass02 seed 0 whole bag (REF_R_GATES §3a).
declare -A REC=(
  [state_trace.txt]=2affc187b225df978dcdd32f3c4a41b8d5613994a147fcd3084bcdafc430e2a8
  [traj_W.tum]=35b515f0fd8cdee6264c2e8904331827edfe2a62e127f1beef6c3cad3f81e4d7
  [filter_cov.csv]=7cc694ea626b486fd063e8d7e6cefcc88facdf9bd92e0e967ff04622a7f19385
  [Log/imu.txt]=0b4f7d4b6adc817df29177dc5b8d6067b0b5d5aa5292a5189b8685b52efd9fbc
  [Log/mat_pre.txt]=be8085dd37463720ab1977d262140a850fa4093b9f3fe0755094a95eb4eacac5
  [Log/mat_out.txt]=ca44f0d81c8e9b230a2fea2e99953f8e7a28e99501e37f27b254888b7c71ae83
  [Log/result]=86dfe9620852488241e09af3c2e77d7cade9a94481812ac2a380db76cf24a2c6
)
evo() { local f; for f in "$1"/Log/result/*.txt; do [ -f "$f" ] && { echo "$f"; return 0; }; done; echo "$1/Log/result/<none>"; }
path_of() { if [ "$2" = Log/result ]; then evo "$1"; else echo "$1/$2"; fi; }
bad=0
say() { echo "[p3b_gate $(date +%T)] $*"; }

# ---- ref-R, once per prefix ----
if [ ! -f "$ROOT/$REF/run_status.json" ]; then
  say "ref-R replay ($REF_WT) -> $REF"
  bash "$RG/replay_run.sh" "$REF_WT" ws/devel "$REF" "$BAG" 0 -1 || { echo "ref-R run incomplete"; exit 3; }
fi
for f in "${!REC[@]}"; do
  h=$(sha256sum "$(path_of "$ROOT/$REF" "$f")" | cut -c1-64)
  if [ "$h" = "${REC[$f]}" ]; then echo "   ref-R $f = recorded ref-R ($h)"; else echo "   ref-R $f DIFFERS from the recorded ref-R ($h)"; bad=1; fi
done
[ "$bad" = 0 ] || { echo "===== ref-R does not reproduce REF_R_GATES §3a on this host/image: no gate verdict"; exit 3; }

overlay() {   # <file> <enable> <update_enable>
  cat > "$1" <<YAML
# P3-B $GATE gate overlay (written by p3b_gate_remote.sh): the T arm with the gate's switches and Grass02's inputs.
gnss:
  enable: $2
  update_enable: $3
  mode: tight
  update_slot: pre_lio
  use: {psr: true, dopp: true, tdcp: true, pvt_vel: true}
  time: {calib_file: $CALIB}
YAML
}

run_head() {   # <run_rel> [extra replay args ...]
  local run=$1; shift
  local extra=""; [ $# -gt 0 ] && extra="extra_args:=$*"
  say "HEAD replay ($WT) -> $run ${extra}"
  if [ -n "$extra" ]; then bash "$RG/replay_run.sh" "$WT" ws/devel "$run" "$BAG" 0 -1 "$extra"
  else bash "$RG/replay_run.sh" "$WT" ws/devel "$run" "$BAG" 0 -1; fi
}

blocked_if_facade() {   # <run_dir>: a HEAD run that could not start because the core is still a stub
  if grep -q "GNSS core: not implemented" "$1/console.log" 2>/dev/null; then
    echo "===== $GATE: blocked: facade (P3-A2 GnssFusion is still the skeleton stub; the run refused to start as designed)"
    exit 5
  fi
}

case "$GATE" in
  zero)
    RUN=runs/$PFX/zero_grass02
    run_head "$RUN" || { echo "HEAD run incomplete"; exit 4; }
    bash "$RG/hash_runs.sh" "$ROOT/$REF" "$ROOT/$RUN" | tee "$ROOT/runs/$PFX/zero_hashes.txt"
    # 7 products byte-identical; the identity line differs by construction (other binary, config gained the gnss: tree)
    n=$(grep -c ': identical' "$ROOT/runs/$PFX/zero_hashes.txt" || true)
    grep -q ': line \|: missing' "$ROOT/runs/$PFX/zero_hashes.txt" && bad=1
    [ "$n" -eq 7 ] || bad=1
    grep -q '"gnss_enable": false' "$ROOT/$RUN/replay_stats.json" || { echo "   HEAD run did not report gnss_enable false"; bad=1; }
    ;;
  bypass|nodata)
    OV=$ROOT/runs/$PFX/${GATE}_overlay.yaml
    if [ "$GATE" = bypass ]; then overlay "$OV" true false; else overlay "$OV" true true; fi
    RUN=runs/$PFX/${GATE}_grass02
    EXTRA=(--param-file "/r/runs/$PFX/${GATE}_overlay.yaml" --nav-rinex "$NAV")
    [ "$GATE" = nodata ] && EXTRA+=(--no-gnss-messages)
    if ! run_head "$RUN" "${EXTRA[@]}"; then
      blocked_if_facade "$ROOT/$RUN"
      if grep -q '"gnss_failed": true' "$ROOT/$RUN/replay_stats.json" 2>/dev/null; then
        echo "   the GNSS core failed during the run: the bridge's fail-safe switched GNSS off, fastlivo_replay exit 4 ([gnss] lines in console.log, GNSS_FAILSAFE in events.log)"
        grep -m3 '\[gnss\].*GNSS disabled' "$ROOT/$RUN/console.log" || true
        echo "===== $GATE: FAIL"; exit 1
      fi
      echo "HEAD run incomplete"; exit 4
    fi
    grep -q '"gnss_enable": true' "$ROOT/$RUN/replay_stats.json" || { echo "   GNSS was not enabled in the HEAD run"; bad=1; }
    grep -q '"gnss_failed": false' "$ROOT/$RUN/replay_stats.json" || { echo "   the GNSS core failed during the run (fail-safe)"; bad=1; }
    if [ "$GATE" = bypass ]; then
      routed=$(grep -o '"gnss_msgs_routed": [0-9]*' "$ROOT/$RUN/replay_stats.json" | grep -o '[0-9]*$')
      [ "${routed:-0}" -gt 0 ] || { echo "   no GNSS message reached the bridge"; bad=1; }
      ep=$(( $(wc -l < "$ROOT/$RUN/gnss_epochs.csv") - 1 ))
      echo "   gnss_msgs_routed=$routed, gnss_epochs.csv rows=$ep"
      [ "$ep" -gt 0 ] || { echo "   the core processed no epoch (OBSERVE must still compute)"; bad=1; }
      # P3 integration: the gate is only as strong as the augmented block it exercises — OBSERVE must go through
      # ACTIVATE so that H1/H2 run on live slots (otherwise the hooks only ever see an empty block).
      na=$(grep -c ' ACTIVATE ' "$ROOT/$RUN/events.log" || true)
      nt=$(awk -F, 'NR==1{for(i=1;i<=NF;i++) if($i=="phase") c=i; next} $c=="TRACKING"' "$ROOT/$RUN/gnss_epochs.csv" | wc -l)
      echo "   ACTIVATE events=$na, TRACKING epochs=$nt (observe_only)"
      [ "${na:-0}" -gt 0 ] || { echo "   OBSERVE never activated: H1/H2 never ran on a live augmented block (vacuous bypass)"; bad=1; }
      # host part only: state_trace columns 1-47 (x_G/diag Q_G may follow), filter_cov without postGNSS rows
      for pair in "state_trace.txt:cut -d' ' -f1-47" "filter_cov.csv:awk -F, 'NR==1||\$2==\"prior\"||\$2==\"postLIO\"||\$2==\"postVIO\"'"; do
        f=${pair%%:*}; cmd=${pair#*:}
        if cmp -s <(eval "$cmd" "$ROOT/$REF/$f") <(eval "$cmd" "$ROOT/$RUN/$f"); then echo "   $f (host part): identical"
        else echo "   $f (host part): DIFFERENT"; bad=1; fi
      done
      for f in traj_W.tum Log/imu.txt Log/mat_pre.txt Log/mat_out.txt Log/result; do
        if cmp -s "$(path_of "$ROOT/$REF" "$f")" "$(path_of "$ROOT/$RUN" "$f")"; then echo "   $f: identical"; else echo "   $f: DIFFERENT"; bad=1; fi
      done
    else
      ep=$(( $(wc -l < "$ROOT/$RUN/gnss_epochs.csv") - 1 ))
      [ "$ep" -eq 0 ] || { echo "   the core processed $ep epochs without data"; bad=1; }
      bash "$RG/hash_runs.sh" "$ROOT/$REF" "$ROOT/$RUN" | tee "$ROOT/runs/$PFX/nodata_hashes.txt"
      n=$(grep -c ': identical' "$ROOT/runs/$PFX/nodata_hashes.txt" || true)
      grep -q ': line \|: missing' "$ROOT/runs/$PFX/nodata_hashes.txt" && bad=1
      [ "$n" -eq 7 ] || bad=1
    fi
    ;;
  nodata-filtered)
    # G-无数据 with the GNSS topics physically absent from the bag (TEST_PLAN I5), GNSS enabled with full updates.
    FB=runs/$PFX/Grass02_nognss.bag
    if [ ! -f "$ROOT/$FB" ]; then
      say "tools/bag_filter.py --drop-gnss -> $FB"
      sudo -n docker run --rm -u 1000:1000 -e HOME=/tmp -v "$ROOT:/r" "${FL2G_IMAGE:-fastlivo2-gnss:noetic}" nice -n 19 \
        python3 "/r/wt/$WT/ws/src/FAST-LIVO2/tools/bag_filter.py" "/r/$BAG" "/r/$FB" --drop-gnss > "$ROOT/runs/$PFX/bag_filter.log" 2>&1 \
        || { tail -5 "$ROOT/runs/$PFX/bag_filter.log"; echo "bag_filter failed"; exit 3; }
    fi
    OV=$ROOT/runs/$PFX/nodata_filtered_overlay.yaml; overlay "$OV" true true
    RUN=runs/$PFX/nodata_filtered_grass02
    say "HEAD replay ($WT) on the GNSS-free bag -> $RUN"
    bash "$RG/replay_run.sh" "$WT" ws/devel "$RUN" "$FB" 0 -1 "extra_args:=--param-file /r/runs/$PFX/nodata_filtered_overlay.yaml --nav-rinex $NAV" \
      || { echo "HEAD run incomplete"; exit 4; }
    grep -q '"gnss_enable": true' "$ROOT/$RUN/replay_stats.json" || { echo "   GNSS was not enabled in the HEAD run"; bad=1; }
    grep -q '"gnss_failed": false' "$ROOT/$RUN/replay_stats.json" || { echo "   the GNSS core failed during the run (fail-safe)"; bad=1; }
    routed=$(grep -o '"gnss_msgs_routed": [0-9]*' "$ROOT/$RUN/replay_stats.json" | grep -o '[0-9]*$')
    echo "   gnss_msgs_routed=$routed (the bag has no GNSS topic)"
    [ "${routed:-1}" -eq 0 ] || bad=1
    ep=$(( $(wc -l < "$ROOT/$RUN/gnss_epochs.csv") - 1 ))
    [ "$ep" -eq 0 ] || { echo "   the core processed $ep epochs without data"; bad=1; }
    bash "$RG/hash_runs.sh" "$ROOT/$REF" "$ROOT/$RUN" | tee "$ROOT/runs/$PFX/nodata_filtered_hashes.txt"
    n=$(grep -c ': identical' "$ROOT/runs/$PFX/nodata_filtered_hashes.txt" || true)
    grep -q ': line \|: missing' "$ROOT/runs/$PFX/nodata_filtered_hashes.txt" && bad=1
    [ "$n" -eq 7 ] || bad=1
    ;;
  determinism)
    # G-确定性 with GNSS on: the T arm (full updates) twice on the same binary, bag and seed.
    OV=$ROOT/runs/$PFX/determinism_overlay.yaml; overlay "$OV" true true
    for s in a b; do
      run_head "runs/$PFX/det_grass02_$s" --param-file "/r/runs/$PFX/determinism_overlay.yaml" --nav-rinex "$NAV" \
        || { echo "HEAD run $s incomplete"; exit 4; }
    done
    A=$ROOT/runs/$PFX/det_grass02_a; B=$ROOT/runs/$PFX/det_grass02_b
    bash "$RG/hash_runs.sh" "$A" "$B" | tee "$ROOT/runs/$PFX/determinism_hashes.txt"
    n=$(grep -c ': identical' "$ROOT/runs/$PFX/determinism_hashes.txt" || true)
    grep -q ': line \|: missing' "$ROOT/runs/$PFX/determinism_hashes.txt" && bad=1
    [ "$n" -eq 7 ] || bad=1
    grep -q 'build_sha256 same, config_sha256 same' "$ROOT/runs/$PFX/determinism_hashes.txt" || bad=1
    # GNSS products: gnss_epochs.csv without its wall-clock column (elapsed_ms), summary.json without elapsed_*
    noelapsed() { awk -F, 'NR==1{for(i=1;i<=NF;i++) if($i=="elapsed_ms") c=i} {if(c){$c=""}; print}' OFS=, "$1"; }
    for f in traj_N.tum traj_ant_N.tum traj_N_geo.csv anchor.yaml events.log; do
      if [ ! -f "$A/$f" ] || [ ! -f "$B/$f" ]; then echo "   $f: missing"; bad=1
      elif cmp -s "$A/$f" "$B/$f"; then echo "   $f: identical ($(wc -l < "$A/$f") lines)"
      else echo "   $f: $(LC_ALL=C cmp "$A/$f" "$B/$f" 2>&1 | grep -o 'line [0-9]*' || echo 'length')"; bad=1; fi
    done
    if cmp -s <(noelapsed "$A/gnss_epochs.csv") <(noelapsed "$B/gnss_epochs.csv"); then
      echo "   gnss_epochs.csv (without elapsed_ms): identical ($(wc -l < "$A/gnss_epochs.csv") lines)"
    else echo "   gnss_epochs.csv (without elapsed_ms): DIFFERENT"; bad=1; fi
    if cmp -s <(grep -v '"elapsed_p' "$A/summary.json") <(grep -v '"elapsed_p' "$B/summary.json"); then echo "   summary.json (without elapsed_*): identical"
    else echo "   summary.json (without elapsed_*): DIFFERENT"; bad=1; fi
    na=$(grep -c ' ACTIVATE ' "$A/events.log" || true)
    xg=$(awk 'NR>1{print NF}' "$A/state_trace.txt" | sort -n | tail -1)
    echo "   ACTIVATE events=$na, widest state_trace row=$xg columns (47 host + x_G + diag Q_G)"
    [ "${na:-0}" -gt 0 ] || { echo "   the T arm never activated: the gate would only cover the waiting phases"; bad=1; }
    for d in "$A" "$B"; do grep -q '"gnss_failed": false' "$d/replay_stats.json" || { echo "   $d: GNSS fail-safe"; bad=1; }; done
    ;;
  *) echo "unknown gate $GATE" >&2; exit 2 ;;
esac
echo "===== $GATE: $([ "$bad" = 0 ] && echo PASS || echo FAIL)"
exit $bad
