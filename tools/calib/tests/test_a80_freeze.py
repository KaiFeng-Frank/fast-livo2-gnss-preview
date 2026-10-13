# SPDX-License-Identifier: BSD-3-Clause
"""Synthetic scratch fixtures verify refusal boundaries; they are not experiment evidence."""
import copy
import csv
import hashlib
import json
from pathlib import Path
import shutil
import sys
import tempfile
import unittest

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import a80_freeze as F
import a80_noise_readout as N

REPO = Path(__file__).resolve().parents[3]


class Freeze(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.repo = self.root / "repo"
        self.repo.mkdir()
        names = ["tools/eval/arms.json", "tools/eval/lidars.json", *F.BASE_CONFIGS.values(),
                 "config/arms/U.yaml", *("config/arms/" + name + ".yaml" for name in F.ORIGINALS)]
        for name in names:
            target = self.repo / name
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(REPO / name, target)
        self.binaries = []
        for name in ("lib/fast_livo/fastlivo_replay", "lib/liblivo_gnss.so"):
            path = self.root / "dummy_build" / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text("synthetic nonexecuted build fixture: " + name)
            self.binaries.append(N.fingerprint(path))
        self.build_files = {item["path"]: item["sha256"] for item in self.binaries}
        self.build_sha = hashlib.sha256("".join(
            str(Path(item["path"]).relative_to(self.root / "dummy_build/lib")) + " " + item["sha256"] + "\n"
            for item in self.binaries).encode()).hexdigest()
        self.gt = {}
        for sequence in N.DEVELOPMENT:
            directory = self.root / "audit" / sequence / "gt"
            directory.mkdir(parents=True)
            (directory / "gt_clean.tum").write_text("99 0 0 0 0 0 0 1\n200 1 0 0 0 0 0 1\n")
            (directory / "trust_mask.csv").write_text("t,ate_exclude\n99,0\n200,0\n")
            (directory / "gt_timebase.yaml").write_text("sequence: " + sequence + "\nmethod: gpst\napplied_shift_s: 0\n")
            self.gt[sequence] = {"class": "development", "mask": {
                "status": "applied", "file": "trust_mask.csv", "column": "ate_exclude",
                "sha256": N.fingerprint(directory / "trust_mask.csv")["sha256"]}, "gt": {
                "dir": str(directory.relative_to(self.root)), "source": "gt_timebase.yaml",
                "gt_clean_sha256": N.fingerprint(directory / "gt_clean.tum")["sha256"],
                "timebase_sha256": N.fingerprint(directory / "gt_timebase.yaml")["sha256"]}}
        self.proposal_path = self.root / "proposal.json"
        self.recheck_path = self.root / "recheck.json"
        initial = [self.noise_run(seq, False) for seq in N.DEVELOPMENT]
        self.proposal = {"schema": "a80-model-noise-readout/v1", "stage": "proposal", "development_only": True,
                         "runs": initial, "groups": N.pool(initial, "proposal")}
        self.write(self.proposal_path, self.proposal)
        recheck = [self.noise_run(seq, True) for seq in N.DEVELOPMENT]
        self.recheck = {"schema": "a80-model-noise-readout/v1", "stage": "recheck", "development_only": True,
                       "runs": recheck, "previous": N.fingerprint(self.proposal_path),
                       "groups": N.pool(recheck, "recheck", self.proposal)}
        self.write(self.recheck_path, self.recheck)
        self.rows = []
        self.candidates = {dataset: {"model": "LIVGO-literal-recheck-" + ("smartpnt" if dataset == "smartpnt_mini" else "mars") + "-diag",
                                      "flat": "LIVGO-literal-paper-diag"} for dataset in F.DATASETS}
        for index, (seq, dataset) in enumerate(N.DEVELOPMENT.items()):
            self.rows.append(self.make_run(seq, "U", self.configuration(seq, "U"), 10))
            model_cfg = self.configuration(seq, "LIVGO-literal-diag")
            model_cfg["gnss"]["noise"]["scale"].update({kind: self.proposal["groups"][dataset]["statistics"][kind]["proposed_scale"]
                                                       for kind in ("dopp", "tdcp")})
            self.rows.append(self.make_run(seq, self.candidates[dataset]["model"], model_cfg, 7 + index))
            self.rows.append(self.make_run(seq, self.candidates[dataset]["flat"],
                                      self.configuration(seq, "LIVGO-literal-paper-diag"), 9.5))
        self.reports = self.reports_for(self.rows, "selection")
        self.selection_path = self.root / "selection.json"
        self.selection = {"ate_files": self.reports, "candidate_arms": self.candidates, "binary_files": self.binaries}
        self.write(self.selection_path, self.selection)
        self.originals = F.protected(self.repo)

    def tearDown(self):
        self.temp.cleanup()

    @staticmethod
    def write(path, data):
        path.write_text(json.dumps(data, indent=2) + "\n")

    def configuration(self, sequence, arm):
        base = yaml.safe_load((self.repo / F.BASE_CONFIGS[N.DEVELOPMENT[sequence]]).read_text())
        config = F.merged(base, yaml.safe_load((self.repo / "config/arms" / (arm + ".yaml")).read_text()))
        config["evo"]["seq_name"] = sequence
        config["replay"].update(enable=True, duration_s=-1, start_offset_s=0, seed=0)
        return config

    def make_run(self, sequence, arm, config, ate, matrix="selected"):
        config = copy.deepcopy(config)
        config["evo"]["seq_name"] = sequence + "_" + arm + "_s0"
        directory = self.root / "runs" / matrix / sequence / arm / "s0"
        directory.mkdir(parents=True, exist_ok=True)
        manifest = {"arm": arm, "seed": 0, "config_dump": json.dumps(config), "config_sha256": F.config_digest(config),
                    "build_sha256": self.build_sha, "build_files": self.build_files}
        self.write(directory / "run_manifest.json", manifest)
        self.write(directory / "replay_stats.json", {"complete": True})
        (directory / "events.log").write_text("t_k=100 t_est=100 gpst=2300:100 ACTIVATE magnitude=0 detail=fixture\n")
        (directory / "traj_N.tum").write_text("100.25 0 0 0 0 0 0 1\n200 1 0 0 0 0 0 1\n")
        values = {list(N.DEVELOPMENT)[0]: [10, 20, 30], list(N.DEVELOPMENT)[1]: [40, 50, 60],
                  "HKisland_GNSS03": [100, 200, 300]}[sequence]
        with (directory / "livgo_hessian.csv").open("w", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(["kind", "t_k", "lambda_bar", "window_n"])
            writer.writerows([["hessian", 99, 1, 9], ["hessian", 129.99, 1, 10],
                              ["hessian", 130.05, 1, 9], *[["hessian", 130.10 + i * 10, v, 10] for i, v in enumerate(values)]])
        return {"seq": sequence, "arm": arm, "seed": 0, "state": "done", "status": "ok",
                "run_dir": str(directory.relative_to(self.root)), "build_sha256": self.build_sha,
                "config_sha256": manifest["config_sha256"], "activated": True,
                "activation": {"t_act": 100.25}, "checks": {"activate_event": {
                    "status": "ok", "n_activate": 1, "n_reinit": 0, "first_activate_t_k": 100,
                    "first_traj_n": 100.25, "lag_s": .25}}, "path": {"ratio": 1},
                "metrics": {name: {"value": ate, "status": "ok (numpy only)"} for name in ("e1", "e1c")}}

    def noise_run(self, seq, recheck):
        dataset = N.DEVELOPMENT[seq]
        config = self.configuration(seq, "LIVGO-literal-diag")
        arm = "LIVGO-literal-diag"
        if recheck:
            arm = "LIVGO-literal-recheck-" + ("smartpnt" if dataset == "smartpnt_mini" else "mars") + "-diag"
            config["gnss"]["noise"]["scale"].update({kind: self.proposal["groups"][dataset]["statistics"][kind]["proposed_scale"]
                                                       for kind in ("dopp", "tdcp")})
        row = self.make_run(seq, arm, config, 1, "noise-recheck" if recheck else "noise-initial")
        directory = self.root / row["run_dir"]
        dof = 20 if seq.startswith("Data02") else 10
        return {"sequence": seq, "dataset": dataset, "partial": False, "seed": 0, "arm": arm,
                "actual_noise_config": config["gnss"]["noise"], "p4_model": N.model_from_config(config),
                "doppler_white_common": {"nis_sum": dof * (1 if recheck else 1.2), "dof_sum": dof},
                "tdcp_self_reported": {"nis_sum": dof * (1 if recheck else .9), "dof_sum": dof},
                "inputs": [N.fingerprint(directory / name) for name in ("run_manifest.json", "replay_stats.json")]}

    def reports_for(self, rows, suffix):
        specs = []
        lidars = N.strict_json(self.repo / "tools/eval/lidars.json")["lidars"]
        for dataset in F.DATASETS:
            chosen = [row for row in rows if N.DEVELOPMENT[row["seq"]] == dataset]
            lidar = F.LIDARS[dataset]
            path = self.root / (suffix + "-" + dataset + ".json")
            self.write(path, {"tool": "tools/eval/eval_ate.py", "root": str(self.root), "blocked": {},
                "metric": {"no_mask": False, "u_arm": "U", "t_max_diff_s": .05, "alignment": "SE(3) Umeyama, no scale",
                           "span": "sequence", "lidar": lidar, "lever_ref_m": lidars[lidar]["gt_lever_imu"],
                           "lever_ref_source": "tools/eval/lidars.json: " + lidar + " gt_lever_imu"},
                "sequences": {row["seq"]: self.gt[row["seq"]] for row in chosen}, "runs": chosen})
            specs.append(N.fingerprint(path))
        return specs

    def refresh_selection(self):
        self.selection["ate_files"] = self.reports_for(self.rows, "selection")
        self.write(self.selection_path, self.selection)

    def prepare(self):
        return F.prepare(self.repo, self.root, self.proposal_path, self.recheck_path,
                         self.selection_path, self.root / "prepared")

    def final_rows(self, prepared):
        rows = []
        for seq, dataset in N.DEVELOPMENT.items():
            arm = F.DEV_ARMS[dataset]
            config = copy.deepcopy(prepared["overlays"][arm]["effective_config"])
            config["evo"]["seq_name"] = seq
            rows.append(self.make_run(seq, arm, config, 1, "final-development"))
        return rows

    def promote(self, rows):
        evidence = self.root / "development.json"
        self.write(evidence, {"ate_files": self.reports_for(rows, "final-development")})
        return F.promote(self.repo, self.root, self.root / "prepared/freeze.json", evidence, self.root / "promoted")

    def test_prepare_uses_global_median3_dataset_type7_and_only_permitted_overlay_changes(self):
        result = self.prepare()
        self.assertEqual(result["winner"], "model")
        self.assertAlmostEqual(result["selection_median3"]["model"], .8)
        self.assertAlmostEqual(result["lambda"]["smartpnt_mini"]["lambda_th"], 12.5)
        self.assertAlmostEqual(result["lambda"]["mars_lvig"]["lambda_th"], 110)
        self.assertFalse(result["evaluation_authorized"])
        for dataset, arm in F.DEV_ARMS.items():
            info = result["overlays"][arm]
            self.assertTrue(info["registry"]["calibration"])
            base = yaml.safe_load((self.repo / "config/arms/LIVGO-literal.yaml").read_text())
            frozen = yaml.safe_load(Path(info["artifact"]["path"]).read_text())
            changed = {key for key, value in F.leaves(frozen).items() if F.leaves(base).get(key) != value}
            self.assertEqual(changed, {"gnss/livgo/lambda_th", "gnss/noise/scale/dopp", "gnss/noise/scale/tdcp"})
        F.verify_protected(self.repo, self.originals)

    def test_flat_winner_does_not_add_any_model_noise_scales(self):
        for row in self.rows:
            if row["arm"] == "LIVGO-literal-paper-diag":
                row["metrics"]["e1c"]["value"] = 5
        self.refresh_selection()
        result = self.prepare()
        self.assertEqual(result["winner"], "flat")
        for info in result["overlays"].values():
            frozen = yaml.safe_load(Path(info["artifact"]["path"]).read_text())
            self.assertNotIn("scale", frozen["gnss"]["noise"])

    def test_prepare_enforces_d4_before_a_better_flat_candidate_can_win(self):
        self.recheck["groups"]["mars_lvig"]["statistics"]["dopp"]["nis_sum"] = 100
        self.write(self.recheck_path, self.recheck)
        with self.assertRaisesRegex(ValueError, "D4 band"):
            self.prepare()

    def test_official_scores_reject_override_lever_unmasked_and_wrong_span(self):
        for key, value in (("lever_ref_source", "--lever-ref"), ("lever_ref_m", [0, 0, 0]),
                           ("no_mask", True), ("span", "seed")):
            path = Path(self.reports[0]["path"])
            report = N.strict_json(path)
            old = copy.deepcopy(report)
            report["metric"][key] = value
            self.write(path, report)
            with self.assertRaises(ValueError):
                F.read_reports([N.fingerprint(path)], self.repo, self.root)
            self.write(path, old)

    def test_truth_mask_hash_and_development_only_sources_are_bound(self):
        path = self.root / "audit" / list(N.DEVELOPMENT)[0] / "gt/trust_mask.csv"
        path.write_text(path.read_text().replace("99,0", "99,1"))
        with self.assertRaisesRegex(ValueError, "SHA256 changed"):
            self.prepare()

    def test_eval_scores_cannot_be_used_as_development_selection(self):
        path = Path(self.reports[0]["path"])
        report = N.strict_json(path)
        report["runs"][0]["seq"] = "Data01_20240719_100C_CAR_opensky"
        self.write(path, report)
        with self.assertRaisesRegex(ValueError, "nondevelopment"):
            F.read_reports([N.fingerprint(path)], self.repo, self.root)

    def test_same_build_baseline_cannot_change_host_configuration(self):
        row = next(row for row in self.rows if row["arm"] == "U")
        path = self.root / row["run_dir"] / "run_manifest.json"
        doc = N.strict_json(path)
        config = json.loads(doc["config_dump"])
        config["lio"]["voxel_size"] = 99
        doc.update(config_dump=json.dumps(config), config_sha256=F.config_digest(config))
        row["config_sha256"] = doc["config_sha256"]
        self.write(path, doc)
        self.refresh_selection()
        with self.assertRaisesRegex(ValueError, "lio/voxel_size"):
            self.prepare()

    def test_lambda_uses_real_event_clock_and_ignores_short_windows(self):
        result = self.prepare()
        sources = [spec for spec in result["inputs"] if "eligible_samples" in spec]
        self.assertEqual([spec["eligible_samples"] for spec in sources], [3, 3, 3])
        self.assertTrue(all(spec["trajectory_lag_s"] == .25 for spec in sources))
        self.assertTrue(all(spec["t_activate"] == 100 for spec in sources))

    def test_lambda_rejects_multiple_activation_or_a_different_clock(self):
        row = next(row for row in self.rows if "recheck" in row["arm"])
        path = self.root / row["run_dir"] / "events.log"
        path.write_text(path.read_text() + "t_k=160 t_est=160 gpst=2300:160 ACTIVATE magnitude=0 detail=fixture\n")
        with self.assertRaisesRegex(ValueError, "single ACTIVATE"):
            self.prepare()

    def test_lambda_rejects_an_event_on_an_unrelated_timebase(self):
        row = next(row for row in self.rows if "recheck" in row["arm"])
        path = self.root / row["run_dir"] / "events.log"
        path.write_text("t_k=1000000 t_est=100 gpst=2300:100 ACTIVATE magnitude=0 detail=fixture\n")
        with self.assertRaisesRegex(ValueError, "clock checks disagree"):
            self.prepare()

    def test_time_calibration_file_cannot_hide_in_a_hash_excluded_identity_field(self):
        row = next(row for row in self.rows if row["arm"] == "LIVGO-literal-paper-diag")
        path = self.root / row["run_dir"] / "run_manifest.json"
        doc = N.strict_json(path)
        config = json.loads(doc["config_dump"])
        old_sha = F.config_digest(config)
        config["gnss"]["time"]["calib_file"] = "/unregistered/map.yaml"
        self.assertEqual(F.config_digest(config), old_sha)
        doc["config_dump"] = json.dumps(config)
        self.write(path, doc)
        with self.assertRaisesRegex(ValueError, "time calibration disabled"):
            self.prepare()

    def test_noise_source_cannot_use_a_different_host_with_legitimate_updated_file_hashes(self):
        row = self.proposal["runs"][0]
        spec = next(spec for spec in row["inputs"] if Path(spec["path"]).name == "run_manifest.json")
        path = Path(spec["path"])
        doc = N.strict_json(path)
        config = json.loads(doc["config_dump"])
        config["lio"]["voxel_size"] = 99
        doc.update(config_dump=json.dumps(config), config_sha256=F.config_digest(config))
        self.write(path, doc)
        spec.update(N.fingerprint(path))
        self.write(self.proposal_path, self.proposal)
        self.recheck["previous"] = N.fingerprint(self.proposal_path)
        self.write(self.recheck_path, self.recheck)
        with self.assertRaisesRegex(ValueError, "lio/voxel_size"):
            self.prepare()

    def test_all_linked_binary_files_must_be_pinned(self):
        self.selection["binary_files"][1]["build_path"] = "/unrelated/liblivo_gnss.so"
        self.write(self.selection_path, self.selection)
        with self.assertRaisesRegex(ValueError, "every actual linked build file"):
            self.prepare()

    def test_promote_after_three_complete_checks_preserves_exact_overlay_bytes(self):
        prepared = self.prepare()
        F.install(self.repo, prepared)
        promoted = self.promote(self.final_rows(prepared))
        self.assertTrue(promoted["evaluation_authorized"])
        self.assertEqual(set(promoted["overlays"]), {"LIVGO-literal-final"})
        final = promoted["overlays"]["LIVGO-literal-final"]
        smart = prepared["overlays"][F.DEV_ARMS["smartpnt_mini"]]
        self.assertEqual(Path(final["artifact"]["path"]).read_bytes(), Path(smart["artifact"]["path"]).read_bytes())
        self.assertFalse(final["registry"]["calibration"])
        F.install(self.repo, promoted)
        F.verify_protected(self.repo, self.originals)

    def test_final_development_bad_path_or_missing_sequence_cannot_unlock_evaluation(self):
        prepared = self.prepare()
        rows = self.final_rows(prepared)
        rows[0]["path"]["ratio"] = .94
        with self.assertRaisesRegex(ValueError, "path ratio"):
            self.promote(rows)
        with self.assertRaisesRegex(ValueError, "exactly one seed-0 score"):
            self.promote(rows[1:])

    def test_promotion_rejects_new_extra_configuration_even_with_a_self_consistent_hash(self):
        prepared = self.prepare()
        rows = self.final_rows(prepared)
        row = rows[0]
        path = self.root / row["run_dir"] / "run_manifest.json"
        doc = N.strict_json(path)
        config = json.loads(doc["config_dump"])
        config["gnss"]["unregistered_future_switch"] = True
        doc.update(config_dump=json.dumps(config), config_sha256=F.config_digest(config))
        row["config_sha256"] = doc["config_sha256"]
        self.write(path, doc)
        with self.assertRaisesRegex(ValueError, "configuration SHA256 differs"):
            self.promote(rows)

    def test_promotion_rejects_changed_binary_or_original_candidate_unlock(self):
        prepared = self.prepare()
        rows = self.final_rows(prepared)
        binary = Path(self.binaries[0]["path"])
        original = binary.read_bytes()
        binary.write_bytes(original + b"changed")
        with self.assertRaisesRegex(ValueError, "SHA256 changed"):
            self.promote(rows)
        binary.write_bytes(original)
        path = self.repo / "tools/eval/arms.json"
        registry = N.strict_json(path)
        registry["arms"]["LIVGO-literal"]["calibration"] = False
        self.write(path, registry)
        with self.assertRaisesRegex(ValueError, "unlocked"):
            self.promote(rows)

    def test_install_checks_conflicts_before_writing_any_new_arm(self):
        prepared = self.prepare()
        registry_path = self.repo / "tools/eval/arms.json"
        before = registry_path.read_bytes()
        conflict = self.repo / prepared["overlays"][F.DEV_ARMS["mars_lvig"]]["registry"]["overlay"]
        conflict.write_text("conflicting prior arm\n")
        with self.assertRaisesRegex(ValueError, "existing overlay"):
            F.install(self.repo, prepared)
        self.assertEqual(registry_path.read_bytes(), before)
        self.assertFalse((self.repo / prepared["overlays"][F.DEV_ARMS["smartpnt_mini"]]["registry"]["overlay"]).exists())


if __name__ == "__main__":
    unittest.main()
