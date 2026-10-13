# SPDX-License-Identifier: BSD-3-Clause
"""No expensive stage may start on a failed or invalid target comparison."""
import json
from pathlib import Path
import unittest
from unittest.mock import patch

import recover_livgo as runner

POLICY = json.loads(Path(__file__).with_name('livgo_policy.json').read_text())


class ContinuationGate(unittest.TestCase):
    def execute(self, development_rc, comparison_rcs):
        with patch.object(runner, 'ensure_frozen'), patch.object(runner, 'takeover'), \
             patch.object(runner, 'completed', return_value=True), \
             patch.object(runner.base, 'status'), patch.object(runner.base, 'analyze'), \
             patch.object(runner.base, 'container', return_value=development_rc), \
             patch.object(runner.base, 'load', return_value=POLICY), \
             patch.object(runner, 'analyze'), patch.object(runner, 'matrix') as matrix, \
             patch.object(runner, 'compare', side_effect=comparison_rcs):
            runner.main()
            return matrix.call_args_list

    def test_development_failure_starts_no_evaluation(self):
        self.assertEqual(self.execute(2, []), [])

    def test_screen_failure_or_invalidity_stops_after_fourteen(self):
        for rc in (2, 3):
            calls = self.execute(0, [rc])
            self.assertEqual(len(calls), 1)
            self.assertEqual(calls[0].args, ('a71_livgo_screen', ['U', 'T'], [0]))

    def test_confirmation_failure_never_starts_ablations(self):
        calls = self.execute(0, [0, 2])
        self.assertEqual(len(calls), 2)
        self.assertEqual(calls[1].args, ('a71_livgo_confirm', ['U', 'T'], [1, 2, 3, 4]))

    def test_passing_path_is_fourteen_then_fifty_six_then_one_forty(self):
        calls = self.execute(0, [0, 0, 0])
        self.assertEqual([7 * len(c.args[1]) * len(c.args[2]) for c in calls], [14, 56, 140])
        self.assertTrue(all(c.args[0] != 'a71_eval' for c in calls))


if __name__ == '__main__':
    unittest.main()
