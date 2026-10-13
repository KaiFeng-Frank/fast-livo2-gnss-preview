#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Compare odometry E1 with the seven published FAST-LIVGO Table II values.

The policy is frozen before reading evaluation results. E1-prime is never used to
pass this check. Matching a paper value is not a same-environment reproduction.
"""
import argparse
import datetime
import hashlib
import json
import math
from pathlib import Path
import statistics


def compare(ate, policy, seeds):
    invalid, rows = [], []
    required_seeds = sorted(seeds)
    if required_seeds not in (policy['screen_seeds'], policy['main_seeds']):
        invalid.append('Seeds must be the predeclared screen or complete five-seed set')
    metric = ate.get('metric', {})
    if policy['metric'] != 'e1o' or policy['tolerance_m'] != 0:
        invalid.append('Unexpected comparison policy')
    if metric.get('alignment') != policy['alignment'] or metric.get('span') != 'sequence':
        invalid.append('Expected whole-sequence SE(3) alignment without scale')
    if metric.get('t_max_diff_s') != policy['t_max_diff_s'] or not metric.get('evo_version'):
        invalid.append('Expected evo validation at the frozen association tolerance')
    if ate.get('blocked'):
        invalid.append('Some requested sequences could not be scored')
    cell_list = ate.get('cells', [])
    cells = {(c['seq'], c['arm']): c for c in cell_list}
    if len(cells) != len(cell_list):
        invalid.append('Duplicate sequence/arm cells')
    for seq, target in policy['targets_m'].items():
        values = {}
        seq_info = ate.get('sequences', {}).get(seq, {})
        if seq_info.get('a55_same_build') is not True:
            invalid.append(seq + ': U and T must use the same verified binary')
        for arm in ('U', 'T'):
            c = cells.get((seq, arm))
            if c is None:
                invalid.append(seq + '/' + arm + ': missing cell')
                continue
            block = c.get('e1o', {})
            if (c.get('n_runs') != len(seeds) or c.get('n_ok') != len(seeds)
                    or block.get('n_ok') != len(seeds)
                    or sorted(c.get('seeds_ok', [])) != required_seeds
                    or sorted(block.get('seeds_ok', [])) != required_seeds):
                invalid.append(seq + '/' + arm + ': missing, extra, or invalid seed')
            if c.get('a55_flag') or c.get('numpy_only') or c.get('identical_seed_groups'):
                invalid.append(seq + '/' + arm + ': instrument or seed identity check failed')
            v = [r for r in ate.get('runs', []) if r.get('seq') == seq and r.get('arm') == arm]
            if sorted(r.get('seed') for r in v) != required_seeds:
                invalid.append(seq + '/' + arm + ': per-run evidence does not match the planned seeds')
            per_seed = {}
            for r in v:
                m = r.get('metrics', {}).get('e1o', {})
                value = m.get('value')
                if (r.get('status') != 'ok' or m.get('status') != 'ok'
                        or not isinstance(value, (float, int)) or not math.isfinite(value) or value < 0):
                    invalid.append(seq + '/' + arm + ': invalid per-run odometry ATE')
                else:
                    per_seed[r['seed']] = value
            median = block.get('median_m')
            if (not isinstance(median, (float, int)) or not math.isfinite(median)
                    or len(per_seed) != len(seeds)
                    or not math.isclose(median, statistics.median(per_seed.values()), rel_tol=1e-12, abs_tol=1e-12)):
                invalid.append(seq + '/' + arm + ': median disagrees with per-run evidence')
                continue
            values[arm] = dict(median_m=median, per_seed_m=per_seed)
        if set(values) == {'U', 'T'}:
            value = values['T']['median_m']
            rows.append(dict(sequence=seq, paper_livgo_m=target, U_m=values['U']['median_m'], T_m=value,
                             delta_to_paper_m=value-target, ratio_to_paper=value/target,
                             target_met=value <= target, runs=values,
                             gt_provenance=seq_info.get('gt'), mask=seq_info.get('mask')))
    all_targets_met = len(rows) == 7 and all(r['target_met'] for r in rows)
    verdict = 'INVALID_COMPARISON' if invalid else ('PAPER_TARGET_MET' if all_targets_met else 'PAPER_TARGET_MISSED')
    return dict(verdict=verdict, stage='screen' if len(seeds) == 1 else 'five_seed_confirmation',
                metric='e1o', expected_seeds=required_seeds, paper=policy['paper'],
                paper_url=policy['paper_url'], tolerance_m=0, invalid=invalid, rows=rows,
                all_seven_targets_met=all_targets_met and not invalid,
                same_environment_livgo_reproduction=False, claim_limit=policy['claim_limit'],
                rule=policy['target_rule'])


def frozen_policy_sha256(policy_path):
    """Hash the policy. If livgo_policy.sha256.json sits beside it, the bytes must match that freeze."""
    digest = hashlib.sha256(policy_path.read_bytes()).hexdigest()
    side = policy_path.with_name("livgo_policy.sha256.json")
    if side.is_file():
        expected = json.loads(side.read_text())["sha256"]
        if digest != expected:
            raise SystemExit("compare_livgo: %s sha256 %s does not match frozen %s" % (policy_path, digest, expected))
    return digest


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--ate', required=True)
    ap.add_argument('--policy', required=True)
    ap.add_argument('--seeds', required=True, help='comma-separated, predeclared seeds')
    ap.add_argument('--out', required=True)
    args = ap.parse_args()
    ate_path, policy_path = Path(args.ate), Path(args.policy)
    policy_sha = frozen_policy_sha256(policy_path)
    result = compare(json.loads(ate_path.read_text()), json.loads(policy_path.read_text()),
                     [int(s) for s in args.seeds.split(',')])
    result.update(checked_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
                  ate_sha256=hashlib.sha256(ate_path.read_bytes()).hexdigest(),
                  policy_sha256=policy_sha)
    out = Path(args.out)
    out.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    lines = ['# FAST-LIVGO Table II comparison', '', 'Verdict: ' + result['verdict'], '',
             'Metric: whole-sequence odometry E1 (e1o), SE(3) without scale. Seeds: ' + args.seeds + '.', '',
             '| Sequence | FAST-LIVGO paper (m) | U (m) | T (m) | T − paper (m) | Target met |',
             '|---|---:|---:|---:|---:|---|']
    for row in result['rows']:
        lines.append('| {sequence} | {paper_livgo_m:.2f} | {U_m:.6f} | {T_m:.6f} | {delta_to_paper_m:+.6f} | {target_met} |'.format(**row))
    lines += ['', result['claim_limit'], '', 'Validation issues: ' + ('; '.join(result['invalid']) or 'none')]
    out.with_suffix('.md').write_text('\n'.join(lines) + '\n')
    print(json.dumps(dict(verdict=result['verdict'], invalid=result['invalid'], targets_met=sum(r['target_met'] for r in result['rows']))))
    return {'PAPER_TARGET_MET': 0, 'PAPER_TARGET_MISSED': 2, 'INVALID_COMPARISON': 3}[result['verdict']]


if __name__ == '__main__':
    raise SystemExit(main())
