# SPDX-License-Identifier: BSD-3-Clause
import csv
import gzip
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import a80_noise_readout as R
import p4_calib as P


class Readout(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.run = self.root / "run"
        self.extract = self.root / "extract"
        self.run.mkdir()
        self.extract.mkdir()
        self.config = {"evo": {"seq_name": "Data01_20240707_100C_CAR_complex_LIVGO-literal-diag_s0"},
            "replay": {"duration_s": -1, "start_offset_s": 0},
            "gnss": {"livgo": {"noise_mode": "model", "enable": True, "paper_init": True,
                "frame_exact": True, "freeze_yaw": True, "cn0_hard": True, "primary_only": True,
                "tdcp_raim_both_ends": True, "dtw_online": True, "dtw_at_init": True, "single_init": True,
                "init_travel_m": 50, "init_tilt_sigma_max_deg": .2, "dtw_grid_s": .1,
                "cn0_min_dbhz": 35, "el_min_deg": 15, "lambda_th": 0, "lambda_source": "lio"},
                "output": {"sat_csv": True}, "time": {"estimate_online": False, "calib_file": ""}, "noise": {
                "psr": {"sys_factor": {"G": 1}}, "dopp": {"sigma0_mps": .05, "tau_s": .2},
                "scale": {"psr": 1, "dopp": .2, "tdcp": .882}}}}
        self.manifest()
        (self.run / "replay_stats.json").write_text(json.dumps({"complete": True}))
        self.csv(self.run / "gnss_epochs.csv.gz", [
            {"week": 2300, "tow": 1, "phase": "TRACKING", "tdcp_dof": 2, "tdcp_nis_prior": 2, "tdcp_acc": 2, "tdcp_cand": 3},
            {"week": 2300, "tow": 2, "phase": "RECOVERY", "tdcp_dof": 2, "tdcp_nis_prior": 6, "tdcp_acc": 3, "tdcp_cand": 4}])
        self.sigma = math.sqrt(1 + .2 * .05**2 * (R.kappa(.2, .1) - 1))
        self.csv(self.run / "gnss_sats.csv.gz", [
            {"week": 2300, "tow": 1, "sat": "G01", "sigma_dopp": self.sigma, "innov_dopp": -1, "reason_dopp": "none", "el_deg": 90},
            {"week": 2300, "tow": 1, "sat": "G02", "sigma_dopp": self.sigma, "innov_dopp": 1, "reason_dopp": "none", "el_deg": 90},
            {"week": 2300, "tow": 1, "sat": "G03", "sigma_dopp": self.sigma, "innov_dopp": 30, "reason_dopp": "w", "el_deg": 90}])
        self.csv(self.extract / "meas_rows.csv.gz", [
            {"week": 2300, "tow": 1, "sat": "G01", "freq_hz": 1575420000, "dopp_std_hz": 0},
            {"week": 2300, "tow": 1, "sat": "G02", "freq_hz": 1575420000, "dopp_std_hz": 0}])

    def tearDown(self):
        self.temp.cleanup()

    def manifest(self):
        (self.run / "run_manifest.json").write_text(json.dumps({"config_dump": json.dumps(self.config), "seed": 0,
                                                                "arm": "LIVGO-literal-diag"}))

    def csv(self, path, records):
        opener = gzip.open if str(path).endswith(".gz") else open
        with opener(path, "wt", newline="") as file:
            writer = csv.DictWriter(file, fieldnames=list(records[0]))
            writer.writeheader()
            writer.writerows(records)

    def test_streamed_gzip_readout_keeps_white_doppler_and_self_reported_phase_tdcp_distinct(self):
        result = R.one_run(self.run, self.extract, False)
        self.assertAlmostEqual(result["doppler_white_common"]["nis_sum"], 2)
        self.assertEqual(result["doppler_white_common"]["dof_sum"], 1)
        self.assertEqual(result["doppler_white_common"]["rows"], 2)
        self.assertEqual(result["tdcp_self_reported"]["nis_sum"], 8)
        self.assertEqual(result["tdcp_self_reported"]["dof_sum"], 4)
        self.assertEqual(result["tdcp_self_reported"]["accepted"], 5)
        self.assertEqual(result["p4_model"]["psr"]["sys"], {"G": 1})
        self.assertNotIn("sys_factor", result["p4_model"]["psr"])
        self.assertTrue(all(len(item["sha256"]) == 64 for item in result["inputs"]))

    def test_common_clock_matches_existing_weighted_ls_with_heterogeneous_variance(self):
        rng = np.random.default_rng(98)
        innovations = 6 + rng.normal(size=19)
        variance = np.exp(rng.normal(size=19))
        residual, _, active, _, dof = P.epoch_ls(innovations, variance, np.ones((19, 1)), snoop=False)
        expected = np.sum(residual[active]**2 / variance[active])
        actual, actual_dof = R.common_clock_stat(innovations, variance)
        self.assertEqual(actual_dof, dof)
        self.assertAlmostEqual(actual, float(expected), places=11)

    def test_smartpnt_pool_is_dof_weighted_and_mars_kept_separate(self):
        first = R.one_run(self.run, self.extract, False)
        second = json.loads(json.dumps(first))
        second["sequence"] = "Data02_20240718_100C_CAR_complex"
        first["doppler_white_common"].update(nis_sum=20, dof_sum=10)
        second["doppler_white_common"].update(nis_sum=45, dof_sum=90)
        mars = json.loads(json.dumps(first))
        mars.update(sequence="HKisland_GNSS03", dataset="mars_lvig")
        mars["doppler_white_common"].update(nis_sum=11, dof_sum=10)
        groups = R.pool([first, second, mars], "proposal")
        self.assertAlmostEqual(groups["smartpnt_mini"]["statistics"]["dopp"]["pooled_factor"], .65)
        self.assertAlmostEqual(groups["smartpnt_mini"]["statistics"]["dopp"]["proposed_scale"], .13)
        self.assertAlmostEqual(groups["mars_lvig"]["statistics"]["dopp"]["pooled_factor"], 1.1)

    def test_evaluation_manifest_refused_before_reading_any_csv(self):
        self.config["evo"]["seq_name"] = "Data01_20240719_100C_CAR_opensky"
        self.manifest()
        with self.assertRaisesRegex(ValueError, "non-A80-development"):
            R.one_run(self.run, self.extract, False)

    def test_partial_and_incomplete_runs_cannot_be_full_calibration(self):
        self.config["replay"]["duration_s"] = 900
        self.manifest()
        with self.assertRaisesRegex(ValueError, "partial replay"):
            R.one_run(self.run, self.extract, False)
        self.assertTrue(R.one_run(self.run, self.extract, True)["partial"])
        (self.run / "replay_stats.json").write_text('{"complete":false}')
        with self.assertRaisesRegex(ValueError, "incomplete replay"):
            R.one_run(self.run, self.extract, True)

    def test_accepted_rows_require_receiver_join_and_valid_white_variance(self):
        self.csv(self.extract / "meas_rows.csv.gz", [
            {"week": 2300, "tow": 1, "sat": "G01", "freq_hz": 1575420000, "dopp_std_hz": 0}])
        with self.assertRaisesRegex(ValueError, "satellite missing"):
            R.one_run(self.run, self.extract, False)
        self.csv(self.extract / "meas_rows.csv.gz", [
            {"week": 2300, "tow": 1, "sat": "G01", "freq_hz": 1575420000, "dopp_std_hz": 100},
            {"week": 2300, "tow": 1, "sat": "G02", "freq_hz": 1575420000, "dopp_std_hz": 0}])
        with self.assertRaisesRegex(ValueError, "white-equivalent"):
            R.one_run(self.run, self.extract, False)

    def test_recheck_requires_registered_scale_and_does_not_offer_another_update(self):
        initial = R.one_run(self.run, self.extract, False)
        previous = {"groups": R.pool([initial], "proposal")}
        with self.assertRaisesRegex(ValueError, "registered proposal"):
            R.pool([initial], "recheck", previous)
        recheck = json.loads(json.dumps(initial))
        for kind in ("dopp", "tdcp"):
            recheck["p4_model"]["scale"][kind] = previous["groups"]["smartpnt_mini"]["statistics"][kind]["proposed_scale"]
        output = R.pool([recheck], "recheck", previous)
        self.assertTrue(all(value["proposed_scale"] is None for value in output["smartpnt_mini"]["statistics"].values()))

    def test_disordered_receiver_csv_is_rejected(self):
        self.csv(self.extract / "meas_rows.csv.gz", [
            {"week": 2300, "tow": 2, "sat": "G01", "freq_hz": 1575420000, "dopp_std_hz": 0},
            {"week": 2300, "tow": 1, "sat": "G02", "freq_hz": 1575420000, "dopp_std_hz": 0}])
        with self.assertRaisesRegex(ValueError, "monotonically"):
            list(R.groups(self.extract / "meas_rows.csv.gz"))

    def test_calibration_cannot_use_another_seed_arm_or_enabled_time_calibration(self):
        path = self.run / "run_manifest.json"
        for field, value in (("seed", 1), ("arm", "T-diag"), ("arm", "LIVGO-literal-recheck-mars-diag")):
            self.manifest()
            doc = json.loads(path.read_text())
            doc[field] = value
            doc["config_dump"] = json.dumps({**self.config, "evo": {"seq_name":
                "Data01_20240707_100C_CAR_complex_%s_s%s" % (doc["arm"], doc["seed"])}})
            path.write_text(json.dumps(doc))
            with self.assertRaisesRegex(ValueError, "seed-0 registered literal"):
                R.one_run(self.run, self.extract, False)
        self.config["gnss"]["time"]["estimate_online"] = True
        self.manifest()
        with self.assertRaisesRegex(ValueError, "time calibration disabled"):
            R.one_run(self.run, self.extract, False)

    def test_a81_or_noncollection_model_runs_cannot_calibrate_a82(self):
        for key, value in (("single_init", False), ("dtw_at_init", False), ("lambda_th", 1.0)):
            old = self.config["gnss"]["livgo"][key]
            self.config["gnss"]["livgo"][key] = value
            self.manifest()
            with self.assertRaisesRegex(ValueError, "A82 literal diagnostic"):
                R.one_run(self.run, self.extract, False)
            self.config["gnss"]["livgo"][key] = old

    def test_actual_runner_sequence_identity_is_exact_and_development_only(self):
        self.assertEqual(R.sequence_identity({"arm": "LIVGO-literal-diag", "seed": 0}, self.config),
                         "Data01_20240707_100C_CAR_complex")
        self.config["evo"]["seq_name"] = "Data01_20240707_100C_CAR_complex_wrong-arm_s0"
        self.manifest()
        with self.assertRaisesRegex(ValueError, "unregistered replay sequence identity"):
            R.one_run(self.run, self.extract, False)


if __name__ == "__main__":
    unittest.main()
