#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""A69 gate: all 3 sequence medians <= U + 5 mm and all T tilt epochs within own sigma.

The tilt reference is the first TRACKING epoch, exactly as runs/a66/a66_tables.py.
No evaluation-set trajectory is read here. Missing evidence is a failure, never a pass.
"""
import csv
import datetime
import gzip
import hashlib
import json
import math
import os
from pathlib import Path
import sys

R = Path(os.environ["FL2G_ROOT"]) if os.environ.get("FL2G_ROOT") else Path("/r")
_LIVGO_DIR = os.environ.get("FL2G_LIVGO_DIR", "a70_recovery_20260927")
WT = os.environ.get("FL2G_WT", "a70p1-strict")
D = R / "runs" / _LIVGO_DIR
M = R / "runs/eval/a70_gate_strict"
DEV = ['Grass01', 'Grass02', 'Z-Rough-Road01']
errors = []
checks = {}


def load(p):
    return json.loads(Path(p).read_text())


def require(ok, label):
    checks[label] = bool(ok)
    if not ok:
        errors.append(label)


def digest(p):
    p = Path(p)
    opener = open
    if not p.exists():
        p = Path(str(p) + '.gz')
        opener = gzip.open
    h = hashlib.sha256()
    with opener(p, 'rb') as f:
        for b in iter(lambda: f.read(1024 * 1024), b''):
            h.update(b)
    return h.hexdigest()


def normalized(p):
    if p.name == 'summary.json':
        v = {k: v for k, v in load(p).items() if not k.startswith('elapsed_')}
        return hashlib.sha256(json.dumps(v, sort_keys=True).encode()).hexdigest()
    if p.name == 'gnss_epochs.csv':
        h = hashlib.sha256()
        with p.open() as f:
            rd = csv.DictReader(f)
            keep = [k for k in rd.fieldnames if k != 'elapsed_ms']
            for row in rd:
                h.update(json.dumps([row[k] for k in keep]).encode())
        return h.hexdigest()
    return digest(p)


def main():
    manifest = load(M / 'matrix_manifest.json')
    def directories(m):
        result = {}
        for r in m['runs']:
            successful = [a for a in r['attempts'] if a.get('complete')]
            if successful:
                result[r['seq'], r['arm'], r['seed']] = R / successful[-1]['run_dir']
        return result
    paths = directories(manifest)
    require(manifest['counts'] == dict(done=60, running=0, waiting=0, failed=0), 'all_60_runs_completed')
    require(all('eval-profile' in v['build_id'] for v in manifest['arms'].values()), 'eval_build_for_all_arms')
    report = load(D / 'a70_gate_strict/dev_report/dev_report.json')
    require(report['self_checks']['instrument_all_pass'], 'development_report_instrument_checks')
    freeze = load(D / 'calibration_freeze.json')
    config = R / "wt" / WT / "ws/src/FAST-LIVO2/config/m3dgr_avia.yaml"
    require(digest(config) == freeze['config_sha256'], 'frozen_configuration_unchanged')

    # Reusing old U runs for calibration is valid only if all 15 host trajectories agree with this build.
    for item in freeze['calibration_inputs']:
        p = paths[item['seq'], 'U', item['seed']] / 'traj_W.tum'
        require(digest(p) == item['trajectory_sha256'], 'calibration_host_identity/' + item['seq'] + '/s%d' % item['seed'])

    # REF_R_GATES §3a recorded hashes, same machine and image, all seven frozen host products.
    refs = {
        'state_trace.txt': '2affc187b225df978dcdd32f3c4a41b8d5613994a147fcd3084bcdafc430e2a8',
        'traj_W.tum': '35b515f0fd8cdee6264c2e8904331827edfe2a62e127f1beef6c3cad3f81e4d7',
        'filter_cov.csv': '7cc694ea626b486fd063e8d7e6cefcc88facdf9bd92e0e967ff04622a7f19385',
        'Log/imu.txt': '0b4f7d4b6adc817df29177dc5b8d6067b0b5d5aa5292a5189b8685b52efd9fbc',
        'Log/mat_pre.txt': 'be8085dd37463720ab1977d262140a850fa4093b9f3fe0755094a95eb4eacac5',
        'Log/mat_out.txt': 'ca44f0d81c8e9b230a2fea2e99953f8e7a28e99501e37f27b254888b7c71ae83',
    }
    u = paths['Grass02', 'U', 0]
    for f, value in refs.items():
        require(digest(u / f) == value, 'zero_channel/' + f)
    logs = list((u / 'Log/result').glob('*.txt'))
    require(len(logs) == 1 and digest(logs[0]) == '86dfe9620852488241e09af3c2e77d7cade9a94481812ac2a380db76cf24a2c6', 'zero_channel/Log/result')

    original = paths['Grass02', 'T', 0]
    repeat = directories(load(R / 'runs/eval/a70_gate_strict_repeat/matrix_manifest.json'))['Grass02', 'T', 0]
    for f in ['traj_W.tum', 'traj_N.tum', 'traj_N_odom.tum', 'traj_ant_N.tum', 'traj_ant_N_odom.tum',
              'gnss_epochs.csv', 'events.log', 'summary.json', 'state_trace.txt', 'filter_cov.csv']:
        require(normalized(original / f) == normalized(repeat / f), 'determinism/' + f)

    ate = load(D / 'a70_gate_strict/eval/ate.json')
    cells = {(c['seq'], c['arm']): c for c in ate['cells']}
    metrics, tilt = {}, {}
    for seq in DEV:
        c, u = cells[seq, 'T'], cells[seq, 'U']
        t_metric, u_metric = c['e1op'], u['e1p']
        require(t_metric['n_ok'] == 5 and u_metric['n_ok'] == 5 and c['n_activated'] == 5, 'five_valid_seeds/' + seq)
        t, b = t_metric['median_m'], u_metric['median_m']
        require(t is not None and b is not None and math.isfinite(t) and math.isfinite(b) and t - b <= .005,
                'odometry_E1prime/' + seq)
        metrics[seq] = dict(T_median_m=t, U_median_m=b, difference_m=t-b if t is not None and b is not None else None,
                            tolerance_m=.005)
        for seed in range(5):
            with (paths[seq, 'T', seed] / 'gnss_epochs.csv').open() as f:
                rows = [r for r in csv.DictReader(f) if r['phase'] == 'TRACKING']
            key = seq + '/s%d' % seed
            require(bool(rows), 'tracking_epochs/' + key)
            ratios = [[], []]
            for axis, letter in enumerate('xy'):
                first = float(rows[0]['tilt_' + letter])
                for r in rows:
                    value, sig = float(r['tilt_' + letter]), float(r['tilt_sigma_' + letter])
                    if not all(math.isfinite(x) for x in (value, sig)) or sig <= 0:
                        raise ValueError('Invalid tilt evidence in ' + key)
                    ratios[axis].append(abs(value - first) / sig)
            maxima = [max(v) for v in ratios]
            tilt[key] = dict(epochs=len(rows), max_abs_delta_over_sigma=maxima,
                             within_one_sigma_fraction=[sum(x <= 1 for x in v)/len(v) for v in ratios])
            require(max(maxima) <= 1 + 1e-12, 'tilt_within_sigma/' + key)
    result = dict(verdict='PASS' if not errors else 'FAIL', checked_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
                  basis='AMENDMENTS A69 item 5; tilt reference is first TRACKING epoch, following A67 tables',
                  checks=checks, failures=errors, odometry=metrics, tilt=tilt, evaluation_data_read=False)
    (D / 'gate_verdict.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    print(json.dumps({k: result[k] for k in ['verdict', 'failures', 'odometry']}))
    return 2 if errors else 0


if __name__ == '__main__':
    sys.exit(main())
