#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Continue the existing development runs; expand only on direct FAST-LIVGO evidence.

The frozen policy is tools/eval/livgo/livgo_policy.json, checked against livgo_policy.sha256.json
beside it. Run state stays under $FL2G_ROOT/runs/<FL2G_LIVGO_DIR> (see recover.py). Set FL2G_PROFILE
(local-dev / local-eval) so run_matrix uses this machine's memory gates; without it the historical
remote gates (32 GiB available, 28 GiB container cap) are passed through.
"""
import fcntl
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import recover as base

HERE = Path(__file__).resolve().parent
D, R = base.D, base.R
POLICY = HERE / "livgo_policy.json"
POLICY_SHA = HERE / "livgo_policy.sha256.json"
LIVGO_IN_CONTAINER = base.C + "/tools/eval/livgo"


def proc(pid):
    try:
        p = Path('/proc') / str(pid)
        v = (p / 'stat').read_text().rsplit(')', 1)[1].split()
        return dict(state=v[0], start_ticks=v[19], cmdline=(p/'cmdline').read_bytes().replace(b'\0', b' ').decode())
    except (FileNotFoundError, ProcessLookupError):
        return None


def same_process(record):
    p = proc(record['pid'])
    return p if p and p['start_ticks'] == record['start_ticks'] else None


def ensure_frozen():
    freeze = base.load(D / 'calibration_freeze.json')
    assert base.sha(base.S / 'config/m3dgr_avia.yaml') == freeze['config_sha256']
    build = base.load(D / 'build_check.json')
    assert base.sha(R / 'wt' / base.WT / 'ws/devel/lib/fast_livo/fastlivo_replay') == build['replay_sha256']
    assert base.sha(POLICY) == base.load(POLICY_SHA)["sha256"]


def completed(name, n):
    p = R / 'runs/eval' / name / 'matrix_manifest.json'
    if not p.exists():
        return False
    return base.load(p)['counts'] == dict(waiting=0, running=0, done=n, failed=0)


def takeover():
    record = base.load(D / 'controller_takeover.json')
    worker = same_process(record['worker'])
    if worker and worker['state'] != 'Z':
        base.status('running_corrected_development_matrix', total=60, target='FAST-LIVGO',
                    controller='fl2g-a70-livgo-chain', automatic_270_disabled=True)
        while worker and worker['state'] != 'Z':
            time.sleep(10)
            worker = same_process(record['worker'])
    old = same_process(record['controller'])
    if old:
        assert old['state'] == 'T', 'Previous coordinator was resumed unexpectedly'
        assert 'recover.py' in old['cmdline']
        # The live replay worker has exited. Only the suspended, obsolete coordinator is terminated.
        os.kill(record['controller']['pid'], signal.SIGKILL)
    base.write(D / 'controller_takeover_complete.json', dict(previous_worker_finished=True,
               previous_controller_replaced=True, development_replays_preserved=True))
    if not completed('a70_gate_strict', 60):
        # Existing run_matrix is resumable: successful jobs remain untouched; failed attempts stay archived.
        base.matrix('a70_gate_strict', ['U', 'T', 'T-dop', 'L-PVT'], base.DEV)


def matrix(name, arms, seeds):
    ensure_frozen()
    policy = base.load(POLICY)
    expected = len(policy['targets_m']) * len(arms) * len(seeds)
    if completed(name, expected):
        return
    args = ['python3', '-u', 'tools/eval/run_matrix.py', 'run', '--name', name,
            '--arms', ','.join(arms), '--seqs', ','.join(policy['targets_m']),
            '--seeds', ','.join(map(str, seeds)), '--devel', 'head=' + base.WT + ':ws/devel',
            '--root', str(R), '--retries', '1', '--evaluation-phase',
            '--gzip-after', 'console.log,filter_cov.csv,state_trace.txt,timing.csv']
    profile = os.environ.get("FL2G_PROFILE")
    if profile:
        args += ["--profile", profile]
        if os.environ.get("FL2G_DOCKER_CMD"):
            args += ["--docker-cmd", os.environ["FL2G_DOCKER_CMD"]]
        if os.environ.get("FL2G_DOCKER_USER"):
            args += ["--docker-user", os.environ["FL2G_DOCKER_USER"]]
    else:
        # Historical remote host. This machine cannot meet a 32 GiB MemAvailable gate; set FL2G_PROFILE.
        args += ['--jobs', '2', '--min-avail-gb', '32', '--brake-avail-gb', '8',
                 '--mem-limit-gb', '28', '--min-free-disk-gb', '20']
    base.run(args, name)
    assert completed(name, expected), 'Incomplete matrix ' + name


def analyze(names, label):
    ensure_frozen()
    out = base.CD + '/' + label
    (D / label).mkdir(exist_ok=True)
    args = ['python3', 'tools/eval/eval_ate.py', '--out', out + '/eval', '--root', '/r',
            '--lever-ref', '-0.176', '-0.016', '0.401', '--evaluation-phase',
            '--seqs', ','.join(base.load(POLICY)['targets_m'])]
    for name in names:
        args += ['--matrix', '/r/runs/eval/' + name]
    base.container(args, label + '_ate')
    base.container(['python3', 'tools/eval/dev_report.py', '--ate', out + '/eval/ate.json',
                    '--out', out + '/report', '--root', '/r', '--evaluation-phase'], label + '_report')
    base.container(['python3', 'tools/eval/make_table.py', '--ate', out + '/eval/ate.json',
                    '--mode', 'main', '--out-md', out + '/table.md', '--out-csv', out + '/table.csv',
                    '--out-csv-supp', out + '/table_supp.csv'], label + '_table')


def compare(label, seeds):
    ensure_frozen()
    return base.container(['python3', LIVGO_IN_CONTAINER + '/compare_livgo.py',
            '--ate', base.CD + '/' + label + '/eval/ate.json', '--policy', LIVGO_IN_CONTAINER + '/livgo_policy.json',
            '--seeds', ','.join(map(str, seeds)), '--out', base.CD + '/' + label + '/livgo_verdict.json'],
            label + '_livgo', (0, 2, 3))


def main():
    ensure_frozen()
    takeover()
    base.status('checking_development_determinism', target='FAST-LIVGO')
    if not completed('a70_gate_strict_repeat', 1):
        base.matrix('a70_gate_strict_repeat', ['T'], ['Grass02'], seeds='0')
    base.status('analyzing_development_reliability', target='FAST-LIVGO')
    base.analyze('a70_gate_strict')
    rc = base.container(['python3', LIVGO_IN_CONTAINER + '/judge.py'], 'judge', (0, 2))
    if rc:
        base.status('development_gate_failed', evaluation_started=False, livgo_target_established=False)
        return

    policy = base.load(POLICY)
    base.status('running_livgo_screen', total=14, seeds=policy['screen_seeds'], target='FAST-LIVGO Table II')
    matrix('a71_livgo_screen', ['U', 'T'], policy['screen_seeds'])
    analyze(['a71_livgo_screen'], 'livgo_screen')
    rc = compare('livgo_screen', policy['screen_seeds'])
    if rc:
        base.status('livgo_screen_target_missed' if rc == 2 else 'livgo_comparison_invalid',
                    verdict=str(D/'livgo_screen/livgo_verdict.json'), expanded=False,
                    next_step='Report the per-sequence gap; keep evaluation parameters frozen. No automatic full matrix.')
        return

    base.status('confirming_livgo_target', additional_runs=56, total_main_runs=70)
    matrix('a71_livgo_confirm', ['U', 'T'], policy['confirmation_seeds'])
    main_matrices = ['a71_livgo_screen', 'a71_livgo_confirm']
    analyze(main_matrices, 'livgo_main')
    rc = compare('livgo_main', policy['main_seeds'])
    if rc:
        base.status('livgo_main_target_missed' if rc == 2 else 'livgo_comparison_invalid',
                    verdict=str(D/'livgo_main/livgo_verdict.json'), expanded=False)
        return

    # The user's target, not merely non-harm to FAST-LIVO2, is now supported by all five seeds.
    base.status('livgo_main_target_met_running_ablations', additional_runs=140,
                evidence=str(D/'livgo_main/livgo_verdict.json'), claim='paper values; no same-environment LIVGO reproduction')
    matrix('a71_livgo_ablations', policy['extra_comparison_arms'], policy['main_seeds'])
    analyze(main_matrices + ['a71_livgo_ablations'], 'livgo_full')
    rc = compare('livgo_full', policy['main_seeds'])
    base.status('livgo_main_and_ablations_complete' if rc == 0 else 'livgo_full_comparison_needs_review',
                total_main_and_ablation_runs=210, verdict=str(D/'livgo_full/livgo_verdict.json'),
                auxiliary_aruco='Pending existing E5 implementation; the extra 60 ArUco runs have not been started.')


if __name__ == '__main__':
    lock = (D / '.livgo_chain.lock').open('a')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    log = (D / 'livgo_chain.log').open('a', buffering=1)
    os.dup2(log.fileno(), 1)
    os.dup2(log.fileno(), 2)
    try:
        main()
    except Exception as exc:
        import traceback
        previous = base.load(D/'status.json') if (D/'status.json').exists() else {}
        base.status('error', error=str(exc), previous_stage=previous.get('stage'), target='FAST-LIVGO')
        traceback.print_exc()
        sys.exit(1)
