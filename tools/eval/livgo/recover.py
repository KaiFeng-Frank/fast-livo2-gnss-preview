#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Resume A69 with the prescribed build and calibration. Host-side, durable systemd job.

Paths come from the environment so the same file runs on the project root of whatever host is in use
(FL2G_ROOT, mounted at /r inside the container). FL2G_WT is the build-tree name under wt/ (historical
default a70p1-strict). FL2G_DOCKER_CMD defaults to the old remote `sudo -n docker`; this machine sets
`docker`. FL2G_CPUS, when set, is a docker --cpus quota (this machine: 4).
"""
import datetime
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import shlex
import subprocess
import sys
import time

HERE = Path(__file__).resolve().parent
R = Path(os.environ["FL2G_ROOT"]) if os.environ.get("FL2G_ROOT") else Path("/r")
_LIVGO_DIR = os.environ.get("FL2G_LIVGO_DIR", "a70_recovery_20260927")
D = R / "runs" / _LIVGO_DIR
WT = os.environ.get("FL2G_WT", "a70p1-strict")
S = R / "wt" / WT / "ws/src/FAST-LIVO2"
C = "/r/wt/" + WT + "/ws/src/FAST-LIVO2"
CD = "/r/runs/" + _LIVGO_DIR
DEV = ['Grass01', 'Grass02', 'Z-Rough-Road01']
EVAL = ['Varying-illu03', 'Varying-illu04', 'Varying-illu05', 'Outdoor01', 'Outdoor04', 'Dark01', 'Dark02', 'GNSS-denial01', 'GNSS-denial02']


def load(p):
    return json.loads(Path(p).read_text())


def write(p, value):
    p = Path(p)
    tmp = p.with_suffix(p.suffix + '.tmp')
    tmp.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')
    tmp.replace(p)


def status(stage, **extra):
    v = dict(stage=stage, updated_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(), **extra)
    write(D / 'status.json', v)
    print(json.dumps(v), flush=True)


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def run(args, label, allowed=(0,)):
    print(label, args, flush=True)
    with (D / (label + '.log')).open('a') as log:
        rc = subprocess.run([str(x) for x in args], cwd=S, stdout=log, stderr=subprocess.STDOUT).returncode
    if rc not in allowed:
        raise RuntimeError('%s exited %s; see %s.log' % (label, rc, label))
    return rc


def _docker_user():
    spec = os.environ.get("FL2G_DOCKER_USER", "1000:1000")
    if spec == "host":
        return "%d:%d" % (os.getuid(), os.getgid())
    return spec


def container(args, label, allowed=(0,)):
    cmd = shlex.split(os.environ.get("FL2G_DOCKER_CMD", "sudo -n docker")) + [
        "run", "--rm", "--init", "-u", _docker_user()]
    if os.environ.get("FL2G_CPUS"):
        cmd += ["--cpus", os.environ["FL2G_CPUS"]]
    cmd += ['-e', 'HOME=/r/wt/' + WT + '/home', '-e', 'MPLCONFIGDIR=/r/wt/' + WT + '/home/.mpl',
            '-e', 'OMP_NUM_THREADS=1', '-e', 'OPENBLAS_NUM_THREADS=1', '-e', 'MKL_NUM_THREADS=1',
            '-e', 'PYTHONDONTWRITEBYTECODE=1', '-v', str(R) + ':/r', '-w', C,
            os.environ.get("FL2G_IMAGE", "fastlivo2-gnss:noetic"), 'nice', '-n', '19', 'ionice', '-c3'] + args
    with (R / '.build.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        return run(cmd, label, allowed)


def active(unit):
    return subprocess.run(['systemctl', '--user', 'is-active', '--quiet', unit]).returncode == 0


def calibration_inputs():
    out = []
    for seq in DEV:
        for seed in range(5):
            matrix = 'a66' if seq != 'Z-Rough-Road01' else ('preproc_Z_Rough_Road01' if seed == 0 else 'a70_zrr_u')
            base = R / 'runs/eval' / matrix
            m = load(base / 'matrix_manifest.json')
            assert 'eval-profile' in m['arms']['U']['build_id'], matrix
            p = base / seq / 'U' / ('s%d' % seed)
            st = load(p / 'run_status.json')
            assert st['complete'] and int(st['node_exit_code']) == 0 and st['manifest_ok'], str(p)
            out.append(dict(seq=seq, seed=seed, run=str(p.relative_to(R)),
                            trajectory_sha256=sha(p / 'traj_W.tum'),
                            build_id=m['arms']['U']['build_id']))
    return out


def matrix(name, arms, seqs, seeds='0-4', evaluation=False):
    args = ['python3', '-u', 'tools/eval/run_matrix.py', 'run', '--name', name, '--arms', ','.join(arms),
            '--seqs', ','.join(seqs), '--seeds', seeds, '--devel', 'head=' + WT + ':ws/devel',
            '--root', str(R), '--jobs', '2' if evaluation else '3',
            '--min-avail-gb', '32' if evaluation else '24', '--brake-avail-gb', '8',
            '--mem-limit-gb', '28' if evaluation else '24', '--min-free-disk-gb', '20',
            '--gzip-after', 'console.log,filter_cov.csv,state_trace.txt,timing.csv', '--retries', '1']
    for seq in seqs:
        if seq in ('Grass01', 'Grass02'):
            args += ['--bag', seq + '=data/m3dgr/bags/' + seq + '.bag']
    if evaluation:
        args += ['--evaluation-phase']
    run(args, name)
    m = load(R / 'runs/eval' / name / 'matrix_manifest.json')
    n = len(arms) * len(seqs) * (5 if seeds == '0-4' else 1)
    assert m['counts'] == dict(waiting=0, running=0, done=n, failed=0), m['counts']


def analyze(name, evaluation=False, diagnostic=False):
    path = '/r/runs/eval/' + name
    out = CD + '/' + name
    (D / name).mkdir(exist_ok=True)
    args = ['python3', 'tools/eval/eval_ate.py', '--matrix', path, '--out', out + '/eval',
            '--lever-ref', '-0.176', '-0.016', '0.401', '--root', '/r']
    if evaluation:
        args += ['--evaluation-phase', '--seqs', ','.join(EVAL[:7])]
    permitted = (0, 3, 4) if diagnostic else (0,)
    container(args, name + '_ate', permitted)
    if not evaluation:
        container(['python3', 'tools/eval/dev_report.py', '--ate', out + '/eval/ate.json',
                   '--out', out + '/dev_report', '--root', '/r'], name + '_report', permitted)
    container(['python3', 'tools/eval/make_table.py', '--ate', out + '/eval/ate.json',
               '--mode', 'main' if evaluation else 'dev', '--out-md', out + '/table.md',
               '--out-csv', out + '/table.csv', '--out-csv-supp', out + '/table_supp.csv'], name + '_table')


def main():
    status('waiting_for_eval_build')
    while active('fl2g-a70-strict-build.service'):
        time.sleep(15)
    result = subprocess.check_output(['systemctl', '--user', 'show', 'fl2g-a70-strict-build.service',
                                      '-p', 'Result', '--value'], text=True).strip()
    assert result == 'success', result
    cache = (R / 'wt' / WT / 'ws/build/CMakeCache.txt').read_text()
    for expected in ('FASTLIVO_STRICT_FP:BOOL=ON', 'FASTLIVO_OMP:BOOL=OFF', 'CMAKE_CXX_FLAGS:STRING=-ffp-contract=off'):
        assert expected in cache, expected
    bid = R / 'wt' / WT / 'ws/devel/BUILD_ID'
    assert 'eval-profile' in bid.read_text()
    # The 245 compiled source files were compared with local 5e1adb9 before the build.
    identity = load(D / 'source_check.json')
    assert identity['differences'] == [] and identity['compiled_source_files'] == 245
    for rel, digest in identity['sha256'].items():
        assert sha(S / rel) == digest, rel
    testlog = (R / 'wt' / WT / 'build_core/Testing/Temporary/LastTest.log').read_text()
    assert testlog.count('Test Passed.') == 20 and 'Test Failed.' not in testlog
    write(D / 'build_check.json', dict(build_id=bid.read_text().strip(), core_test_groups_passed=20,
          replay_sha256=sha(R / 'wt' / WT / 'ws/devel/lib/fast_livo/fastlivo_replay'),
          compile_profile='eval-profile', compiled_source_files=245))

    status('recalibrating_from_15_development_trajectories')
    inputs = calibration_inputs()
    write(D / 'calibration_inputs.json', inputs)
    args = ['python3', 'tools/calib/p4_calib.py', 'frame', '--root', '/r', '--out', CD + '/frame_open_s0-4.json']
    for item in inputs:
        args += ['--run', item['seq'] + '=/r/' + item['run']]
    container(args, 'frame_calibration')
    cal = load(D / 'frame_open_s0-4.json')
    assert len(cal['trajectories']) == 15
    vals = {key: cal['quantiles'][key]['p75'] for key in ('q_tilt', 'q_tv')}
    assert all(cal['quantiles'][key]['n'] == 15 for key in vals)
    config = S / 'config/m3dgr_avia.yaml'
    before = config.read_text()
    backup = D / 'm3dgr_avia.before.yaml'
    if not backup.exists():
        backup.write_text(before)
    after = before
    for key, value in vals.items():
        after, n = re.subn(r'(' + key + r'_per_m:\s*)[^,\s}]+', lambda m: m[1] + format(value, '.17g'), after)
        assert n == 1
    config.write_text(after)
    (D / 'm3dgr_avia.frozen.yaml').write_text(after)
    bid.write_text('5e1adb9 1 eval-profile a70-open-dev-15-trajectories\n')
    write(D / 'calibration_freeze.json', dict(method='A58 RPE-distance NNLS; p75 over 3 development sequences x 5 seeds',
          values=vals, config_sha256=sha(config), calibration_sha256=sha(D / 'frame_open_s0-4.json'),
          calibration_inputs=inputs, build_id=bid.read_text().strip()))

    status('waiting_for_previous_diagnostic_matrix', values=vals)
    while active('fl2g-a70-gate.service'):
        time.sleep(15)
    # Old runs remain untouched and are labelled diagnostic only in their companion directory.
    old = load(R / 'runs/eval/a70_gate/matrix_manifest.json')['counts']
    if old == dict(waiting=0, running=0, done=60, failed=0):
        status('archiving_previous_diagnostic_results')
        analyze('a70_gate', diagnostic=True)

    status('running_corrected_development_matrix', total=60)
    matrix('a70_gate_strict', ['U', 'T', 'T-dop', 'L-PVT'], DEV)
    status('checking_determinism')
    matrix('a70_gate_strict_repeat', ['T'], ['Grass02'], seeds='0')
    status('analyzing_corrected_development_matrix')
    analyze('a70_gate_strict')
    verdict = container(['python3', CD + '/judge.py'], 'judge', (0, 2))
    if verdict == 2:
        status('development_gate_failed', verdict=str(D / 'gate_verdict.json'), evaluation_started=False)
        return
    status('development_gate_passed_starting_evaluation', total=270)
    assert sha(config) == load(D / 'calibration_freeze.json')['config_sha256']
    matrix('a71_eval', ['U', 'L-PVT', 'T', 'T-pr', 'T-dop', 'T-tdcp'], EVAL, evaluation=True)
    status('generating_evaluation_ate_tables')
    analyze('a71_eval', evaluation=True)
    status('evaluation_replays_and_ate_tables_complete', total=270,
           remaining='GNSS-denial01/02 ArUco E5 endpoint evaluation is not implemented in the existing tools; it is not ATE.')


if __name__ == '__main__':
    D.mkdir(exist_ok=True)
    # User's 2026-09-27 target supersedes the original U-only automatic expansion.
    livgo = HERE / "recover_livgo.py"
    if (HERE / "livgo_policy.json").is_file() and livgo.is_file():
        os.execv(sys.executable, [sys.executable, str(livgo)])
    # Lock the continuation itself, separately from the project build lock and replay slots.
    lock = (D / '.chain.lock').open('a')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    log = (D / 'chain.log').open('a', buffering=1)
    os.dup2(log.fileno(), 1)
    os.dup2(log.fileno(), 2)
    try:
        main()
    except Exception as exc:
        import traceback
        previous = load(D / 'status.json') if (D / 'status.json').exists() else {}
        status('error', error=str(exc), previous_stage=previous.get('stage'))
        traceback.print_exc()
        sys.exit(1)
