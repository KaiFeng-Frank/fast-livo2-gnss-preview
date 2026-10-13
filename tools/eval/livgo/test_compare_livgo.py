# SPDX-License-Identifier: BSD-3-Clause
"""Guard the decision to spend compute on the full comparison matrix."""
import copy
import hashlib
import json
from pathlib import Path
import statistics
import unittest

from compare_livgo import compare

POLICY = json.loads(Path(__file__).with_name('livgo_policy.json').read_text())


def evidence(seeds=(0,), t_ratio=0.9):
    ate = dict(metric=dict(alignment=POLICY['alignment'], span='sequence',
                           t_max_diff_s=0.05, evo_version='1.31.1'),
               blocked={}, sequences={}, cells=[], runs=[])
    for seq, target in POLICY['targets_m'].items():
        ate['sequences'][seq] = dict(a55_same_build=True, gt={'source': 'fixture'}, mask={})
        for arm, ratio in [('U', 3), ('T', t_ratio)]:
            vals = [target * ratio * (1 + (seed-2) * 0.001) for seed in seeds]
            ate['cells'].append(dict(seq=seq, arm=arm, n_runs=len(seeds), n_ok=len(seeds),
                seeds_ok=list(seeds), a55_flag=False, numpy_only=False, identical_seed_groups=[],
                e1o=dict(n_ok=len(seeds), seeds_ok=list(seeds), median_m=statistics.median(vals))))
            for seed, value in zip(seeds, vals):
                ate['runs'].append(dict(seq=seq, arm=arm, seed=seed, status='ok',
                    metrics=dict(e1o=dict(status='ok', value=value), e1op=dict(status='ok', value=0.001))))
    return ate


class ComparisonGate(unittest.TestCase):
    def test_policy_file_matches_frozen_sha256(self):
        raw = Path(__file__).with_name("livgo_policy.json").read_bytes()
        frozen = json.loads(Path(__file__).with_name("livgo_policy.sha256.json").read_text())
        self.assertEqual(hashlib.sha256(raw).hexdigest(), frozen["sha256"])

    def verdict(self, data, seeds=(0,)):
        return compare(data, POLICY, list(seeds))['verdict']

    def test_screen_and_all_five_seeds_can_pass(self):
        for seeds in [(0,), (0, 1, 2, 3, 4)]:
            with self.subTest(seeds=seeds):
                self.assertEqual(self.verdict(evidence(seeds), seeds), 'PAPER_TARGET_MET')

    def test_improving_livo2_with_small_lever_fitted_error_does_not_pass(self):
        self.assertEqual(self.verdict(evidence(t_ratio=1.2)), 'PAPER_TARGET_MISSED')

    def test_one_bad_sequence_cannot_be_averaged_away(self):
        data = evidence(t_ratio=0.1)
        seq = next(iter(POLICY['targets_m']))
        value = POLICY['targets_m'][seq] * 1.01
        next(c for c in data['cells'] if c['seq'] == seq and c['arm'] == 'T')['e1o']['median_m'] = value
        next(r for r in data['runs'] if r['seq'] == seq and r['arm'] == 'T')['metrics']['e1o']['value'] = value
        self.assertEqual(self.verdict(data), 'PAPER_TARGET_MISSED')

    def test_missing_runs_and_sequences_are_invalid(self):
        for key in ['cells', 'runs']:
            data = evidence()
            data[key].pop()
            self.assertEqual(self.verdict(data), 'INVALID_COMPARISON')

    def test_wrong_alignment_and_instrument_failures_are_invalid(self):
        data = evidence()
        mutations = [lambda d: d['metric'].update(alignment='Sim(3)'),
                     lambda d: d['cells'][1].update(a55_flag=True),
                     lambda d: d['cells'][1].update(numpy_only=True),
                     lambda d: d['cells'][1].update(identical_seed_groups=[[0, 1]]),
                     lambda d: d['sequences'][next(iter(POLICY['targets_m']))].update(a55_same_build=False)]
        for mutation in mutations:
            bad = copy.deepcopy(data)
            mutation(bad)
            self.assertEqual(self.verdict(bad), 'INVALID_COMPARISON')

    def test_cannot_pick_the_best_seed_or_report_a_false_median(self):
        self.assertEqual(self.verdict(evidence((3,)), (3,)), 'INVALID_COMPARISON')
        seeds = (0, 1, 2, 3, 4)
        data = evidence(seeds)
        data['cells'][1]['e1o']['median_m'] = 0.001
        self.assertEqual(self.verdict(data, seeds), 'INVALID_COMPARISON')

    def test_missing_seed_cannot_pass_confirmation(self):
        data = evidence((0, 1, 2, 3))
        self.assertEqual(self.verdict(data, (0, 1, 2, 3, 4)), 'INVALID_COMPARISON')

    def test_duplicate_cells_and_nan_are_invalid(self):
        data = evidence()
        data['cells'].append(copy.deepcopy(data['cells'][0]))
        self.assertEqual(self.verdict(data), 'INVALID_COMPARISON')
        data = evidence()
        data['runs'][1]['metrics']['e1o']['value'] = float('nan')
        self.assertEqual(self.verdict(data), 'INVALID_COMPARISON')


if __name__ == '__main__':
    unittest.main()
