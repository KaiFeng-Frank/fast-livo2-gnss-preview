#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Prepare development-only A80 frozen arms, then promote after final dev checks.

prepare --repo REPO --root DATA --proposal noise-proposal.json --recheck noise-recheck.json
        --selection selection.json --out-dir DIR [--install]
promote --repo REPO --root DATA --prepared DIR/freeze.json --dev-evidence dev.json
        --out-dir DIR [--install]

selection.json contains ate_files:[{path,sha256}], candidate_arms:{DATASET:{model,flat}},
and binary_files:[{path,sha256,build_path}] (all replay build_files; build_path is
the container path recorded in the manifest, when it differs from local path).
dev.json contains ate_files:[{path,sha256}] for the three final development runs.
All scores come from official eval_ate ate.json files. Lambda is computed from
the selected complete seed-0 development runs, not supplied as a free number.
Nothing is installed unless --install is explicit. Existing candidate overlays
and registry entries are immutable; no evaluation run is ever launched here.
"""
import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import re
import statistics

import yaml

import a80_noise_readout as N

DATASETS = ("smartpnt_mini", "mars_lvig")
DEV_ARMS = {"smartpnt_mini": "LIVGO-literal-frozen-smartpnt", "mars_lvig": "LIVGO-literal-frozen-mars"}
BASE_CONFIGS = {"smartpnt_mini": "config/smartpnt_mini.yaml", "mars_lvig": "config/mars_lvig_gnss.yaml"}
LIDARS = {"smartpnt_mini": "smartpnt_mini", "mars_lvig": "mars_avia"}
ORIGINALS = ("LIVGO-literal", "LIVGO-literal-diag", "LIVGO-literal-paper", "LIVGO-literal-paper-diag")
OK = {"ok", "ok (numpy only)"}
IDENTITY = {"replay/bag", "replay/seed", "replay/start_offset_s", "replay/duration_s", "output/dir", "evo/seq_name",
            "gnss/nav_rinex", "gnss/time/calib_file", "gnss/rinex/obs", "gnss/rinex/nav"}


def reference(spec):
    actual = N.fingerprint(spec["path"])
    if actual["sha256"] != spec.get("sha256"):
        raise ValueError("evidence SHA256 changed: " + spec["path"])
    return actual


def merged(base, overlay):
    result = copy.deepcopy(base)
    for key, value in overlay.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = merged(result[key], value)
        else:
            result[key] = copy.deepcopy(value)
    return result


def leaves(value, prefix=""):
    result = {}
    for key, item in value.items():
        path = prefix + "/" + key if prefix else key
        if isinstance(item, dict):
            result.update(leaves(item, path))
        else:
            result[path] = item
    return result


def check_effective(expected, actual, diagnostic=False):
    want, have = leaves(expected), leaves(actual)
    for key, value in want.items():
        if key in IDENTITY or (diagnostic and key == "gnss/output/sat_csv"):
            continue
        if key not in have or value != have[key] or isinstance(value, bool) != isinstance(have[key], bool):
            raise ValueError("effective configuration differs at " + key)


def canonical_config(value, prefix=""):
    """The replay's sorted canonical JSON, %.17g floats, run identity removed."""
    if isinstance(value, dict):
        entries = []
        for key in sorted(value):
            path = prefix + "/" + key if prefix else key
            if path not in IDENTITY:
                entries.append(json.dumps(key, ensure_ascii=False) + ":" + canonical_config(value[key], path))
        return "{" + ",".join(entries) + "}"
    if isinstance(value, list):
        return "[" + ",".join(canonical_config(v, prefix) for v in value) + "]"
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, int):
        return str(value)
    if isinstance(value, float):
        return format(N.finite(value, "configuration number"), ".17g")
    if isinstance(value, str):
        return json.dumps(value, ensure_ascii=False)
    raise ValueError("unsupported canonical configuration value")


def config_digest(config):
    return hashlib.sha256(canonical_config(config).encode()).hexdigest()


def runtime_base(repo, dataset):
    base = yaml.safe_load((Path(repo) / BASE_CONFIGS[dataset]).read_text())
    # Both registered replay launches set this after loading the live-node YAML.
    base["replay"]["enable"] = True
    return base


def protected(repo):
    registry = N.strict_json(Path(repo) / "tools/eval/arms.json")["arms"]
    result = {}
    for name in ORIGINALS:
        entry = registry[name]
        if entry.get("calibration") is not True or entry.get("time_calib") is not False:
            raise ValueError("original development candidate has been unlocked: " + name)
        result[name] = {"entry": copy.deepcopy(entry), "overlay": N.fingerprint(Path(repo) / entry["overlay"])}
    return result


def verify_protected(repo, record):
    current = protected(repo)
    for name in ORIGINALS:
        if current[name]["entry"] != record[name]["entry"] or any(
                current[name]["overlay"][field] != record[name]["overlay"][field] for field in ("sha256", "bytes")):
            raise ValueError("original candidate registry/overlay changed after prepare")


def read_reports(specs, repo, root):
    lidar_path = Path(repo) / "tools/eval/lidars.json"
    lidars = N.strict_json(lidar_path)["lidars"]
    reports, provenance = [], [N.fingerprint(lidar_path)]
    for spec in specs:
        provenance.append(reference(spec))
        report = N.strict_json(spec["path"])
        metric = report.get("metric", {})
        if report.get("tool") != "tools/eval/eval_ate.py" or report.get("blocked"):
            raise ValueError("need an unblocked official eval_ate report")
        if metric.get("no_mask") is not False or metric.get("u_arm") != "U" or metric.get("t_max_diff_s") != .05 or \
                metric.get("alignment") != "SE(3) Umeyama, no scale" or metric.get("span") != "sequence":
            raise ValueError("score must use the registered masked e1c association")
        if any(row["seq"] not in N.DEVELOPMENT for row in report["runs"]):
            raise ValueError("refusing evaluation/nondevelopment scores in freeze evidence")
        if Path(report.get("root", "")).resolve() != Path(root).resolve():
            raise ValueError("score root differs from the evidence root")
        datasets = {N.DEVELOPMENT[row["seq"]] for row in report["runs"]}
        if len(datasets) != 1:
            raise ValueError("one official report must have one registered LiDAR profile")
        lidar = LIDARS[next(iter(datasets))]
        if metric.get("lidar") != lidar or metric.get("lever_ref_m") != lidars[lidar]["gt_lever_imu"] or \
                metric.get("lever_ref_source") != "tools/eval/lidars.json: %s gt_lever_imu" % lidar:
            raise ValueError("e1c does not use the registered dataset lever arm")
        for sequence in {row["seq"] for row in report["runs"]}:
            seqinfo = report.get("sequences", {}).get(sequence, {})
            mask, gt = seqinfo.get("mask", {}), seqinfo.get("gt", {})
            expected = Path("audit") / sequence / "gt"
            if seqinfo.get("class") != "development" or mask.get("status") != "applied" or \
                    mask.get("file") != "trust_mask.csv" or mask.get("column") != "ate_exclude" or \
                    gt.get("source") != "gt_timebase.yaml" or Path(gt.get("dir", "")) != expected:
                raise ValueError("score does not use the registered development truth mask/timebase")
            directory = Path(root) / expected
            timebase = yaml.safe_load((directory / "gt_timebase.yaml").read_text())
            if timebase.get("sequence") not in (None, sequence) or \
                    timebase.get("ate_mask_file", "trust_mask.csv") != "trust_mask.csv" or \
                    timebase.get("ate_mask_column", "ate_exclude") != "ate_exclude":
                raise ValueError("development timebase changes the registered truth mask")
            for filename, digest in (("trust_mask.csv", mask.get("sha256")),
                                     ("gt_timebase.yaml", gt.get("timebase_sha256")),
                                     ("gt_clean.tum", gt.get("gt_clean_sha256"))):
                provenance.append(reference({"path": str(directory / filename), "sha256": digest}))
        reports.extend(report["runs"])
    return reports, provenance


def find_run(records, sequence, arm):
    found = [row for row in records if (row["seq"], row["arm"], row["seed"]) == (sequence, arm, 0)]
    if len(found) != 1:
        raise ValueError("need exactly one seed-0 score: %s/%s" % (sequence, arm))
    row = found[0]
    if row.get("state") != "done" or not row.get("run_dir") or "INSTRUMENT" in row.get("status", ""):
        raise ValueError("score run is not complete or its instrument failed")
    for name in ("e1", "e1c"):
        score = row.get("metrics", {}).get(name, {})
        if score.get("status") not in OK:
            raise ValueError("invalid official score: " + name)
        N.finite(score["value"], name + " ATE", nonnegative=True)
    for key in ("build_sha256", "config_sha256"):
        value = row.get(key)
        if not isinstance(value, str) or len(value) != 64 or any(c not in "0123456789abcdef" for c in value):
            raise ValueError("score lacks a valid " + key)
    return row


def run_config(root, row):
    directory = Path(root) / row["run_dir"]
    manifest_path, stats_path = directory / "run_manifest.json", directory / "replay_stats.json"
    manifest, stats = N.strict_json(manifest_path), N.strict_json(stats_path)
    config = manifest["config_dump"]
    config = json.loads(config, parse_int=lambda text: -0.0 if text == "-0" else int(text)) if isinstance(config, str) else config
    N.time_calibration_disabled(config)
    if stats.get("complete") is not True or manifest.get("seed") != 0 or manifest.get("arm") != row["arm"]:
        raise ValueError("final/development evidence must be complete seed 0")
    if N.sequence_identity(manifest, config) != row["seq"] or config["replay"]["duration_s"] > 0 or \
            config["replay"]["start_offset_s"] != 0:
        raise ValueError("freeze requires whole development sequences with no start offset")
    for key in ("build_sha256", "config_sha256"):
        if manifest.get(key) != row[key]:
            raise ValueError("run manifest and official score disagree: " + key)
    if config_digest(config) != manifest["config_sha256"]:
        raise ValueError("actual effective configuration fails its recorded canonical SHA256")
    return config, directory, [N.fingerprint(manifest_path), N.fingerprint(stats_path)]


def verify_build(directory, binaries, expected):
    files = N.strict_json(directory / "run_manifest.json").get("build_files", {})
    executables = [path for path in files if Path(path).name == "fastlivo_replay"]
    if len(executables) != 1 or not any(Path(path).name == "liblivo_gnss.so" for path in files):
        raise ValueError("run lacks replay/GNSS build-file provenance")
    executable = executables[0]
    prefix = Path(executable).parent.parent
    order = [executable] + sorted(path for path in files if path != executable)
    try:
        listing = "".join(str(Path(path).relative_to(prefix)) + " " + files[path] + "\n" for path in order)
    except ValueError:
        raise ValueError("recorded build files are outside the replay build prefix")
    if hashlib.sha256(listing.encode()).hexdigest() != expected:
        raise ValueError("recorded build-file listing does not match build_sha256")
    supplied = {spec.get("build_path", spec["path"]): spec["sha256"] for spec in binaries}
    if files != supplied:
        raise ValueError("pin every actual linked build file and its recorded build_path")


def checked_noise(proposal_path, recheck_path):
    proposal, recheck = N.strict_json(proposal_path), N.strict_json(recheck_path)
    if proposal.get("schema") != "a80-model-noise-readout/v1" or recheck.get("schema") != proposal["schema"] or \
            proposal.get("stage") != "proposal" or recheck.get("stage") != "recheck":
        raise ValueError("need the one registered noise proposal and its recheck")
    if recheck.get("previous", {}).get("sha256") != N.fingerprint(proposal_path)["sha256"]:
        raise ValueError("noise recheck is not bound to this proposal")
    scales = {}
    for dataset in DATASETS:
        first, second = proposal["groups"][dataset], recheck["groups"][dataset]
        if not first.get("full_replays") or not second.get("full_replays") or \
                not first.get("complete_development_coverage") or not second.get("complete_development_coverage"):
            raise ValueError("noise calibration must cover full development sequences")
        if first["noise_without_calibrated_scales"] != second["noise_without_calibrated_scales"]:
            raise ValueError("noise recheck changed more than the two scales")
        scales[dataset] = {}
        for kind in ("dopp", "tdcp"):
            before, after = first["statistics"][kind], second["statistics"][kind]
            expected = before["current_scale"] * before["nis_sum"] / before["dof_sum"]
            if not math.isclose(before["proposed_scale"], expected, rel_tol=1e-12) or \
                    not math.isclose(after["current_scale"], expected, rel_tol=1e-9, abs_tol=1e-12):
                raise ValueError("recheck does not use the one closed-form scale proposal")
            factor = after["nis_sum"] / after["dof_sum"]
            if not math.isfinite(factor) or not N.D4[0] <= factor <= N.D4[1]:
                raise ValueError("noise recheck failed the registered D4 band")
            scales[dataset][kind] = N.finite(after["current_scale"], kind + " scale", positive=True)
    for document in (proposal, recheck):
        if document.get("development_only") is not True:
            raise ValueError("noise readout is not development-only")
        if {row["sequence"] for row in document["runs"]} != set(N.DEVELOPMENT) or any(row.get("partial") for row in document["runs"]):
            raise ValueError("noise readout misses full development coverage")
        for row in document["runs"]:
            for spec in row["inputs"]:
                reference(spec)
            manifests = [spec for spec in row["inputs"] if Path(spec["path"]).name == "run_manifest.json"]
            if len(manifests) != 1:
                raise ValueError("noise readout lacks one actual run manifest")
            manifest = N.strict_json(manifests[0]["path"])
            config = manifest["config_dump"]
            config = json.loads(config, parse_int=lambda text: -0.0 if text == "-0" else int(text)) if isinstance(config, str) else config
            if N.sequence_identity(manifest, config) != row["sequence"] or config["replay"]["duration_s"] > 0 or \
                    config["replay"]["start_offset_s"] != 0:
                raise ValueError("noise source is not the full registered development sequence")
            N.literal_source(manifest, config, row["sequence"])
            if row.get("seed") != 0 or row.get("arm") != manifest["arm"] or \
                    row["actual_noise_config"] != config["gnss"]["noise"] or row["p4_model"] != N.model_from_config(config):
                raise ValueError("noise readout and actual model-source manifest disagree")
            completions = [spec for spec in row["inputs"] if Path(spec["path"]).name == "replay_stats.json"]
            if len(completions) != 1 or N.strict_json(completions[0]["path"]).get("complete") is not True or \
                    config_digest(config) != manifest.get("config_sha256"):
                raise ValueError("noise source is incomplete or its configuration hash is invalid")
        if N.pool(document["runs"], document["stage"], proposal if document is recheck else None) != document["groups"]:
            raise ValueError("noise group statistics do not match the pooled per-run readout")
    return scales, [N.fingerprint(proposal_path), N.fingerprint(recheck_path)]


def lambda_samples(root, selected):
    result = {dataset: [] for dataset in DATASETS}
    sources = []
    for row in selected:
        config, directory, provenance = run_config(root, row)
        if config["gnss"]["livgo"]["lambda_th"] != 0:
            raise ValueError("lambda collection run did not use lambda_th=0")
        sources.extend(provenance)
        if row.get("activated") is not True:
            continue
        event_path = directory / "events.log"
        events = []
        for line in event_path.read_text().splitlines():
            match = re.match(r"^t_k=(\S+) t_est=\S+ gpst=\S+ (\S+) magnitude=", line)
            if match:
                events.append((match.group(2), N.finite(match.group(1), "event t_k")))
        acts = [time for kind, time in events if kind == "ACTIVATE"]
        if len(acts) != 1 or any(kind == "REINIT" for kind, _ in events):
            raise ValueError("lambda collection requires the A82 single ACTIVATE without REINIT")
        activation = acts[0]
        trajectory = directory / "traj_N.tum"
        with trajectory.open() as source:
            first = next((N.finite(line.split()[0], "first traj_N time") for line in source
                          if line.strip() and not line.lstrip().startswith("#")), None)
        scored = N.finite(row["activation"]["t_act"], "scored activation time")
        check = row.get("checks", {}).get("activate_event", {})
        if first is None or abs(first - scored) > 2e-6 or not -1e-6 <= first - activation <= 1 or \
                check.get("status") != "ok" or check.get("n_activate") != 1 or check.get("n_reinit") != 0 or \
                abs(N.finite(check.get("first_activate_t_k"), "reported ACTIVATE") - activation) > 2e-6:
            raise ValueError("ACTIVATE, traj_N and official activation clock checks disagree")
        window = config["gnss"]["livgo"].get("hessian_window", 10)
        path = N.choose_csv(directory, "livgo_hessian.csv")
        values, times = [], []
        for item in N.rows(path):
            time = N.finite(item["t_k"], "Hessian t_k")
            times.append(time)
            if item["kind"] != "hessian" or time < activation + 30 or int(item["window_n"]) < window:
                continue
            value = float(item["lambda_bar"])
            if math.isfinite(value):
                values.append(N.finite(value, "lambda_bar", nonnegative=True))
        result[N.DEVELOPMENT[row["seq"]]].extend(values)
        if not times or not min(times) <= activation <= max(times):
            raise ValueError("Hessian t_k does not cover the ACTIVATE clock")
        sources.extend([N.fingerprint(event_path), N.fingerprint(trajectory)])
        sources.append({**N.fingerprint(path), "sequence": row["seq"], "t_activate": activation,
                        "activation_clock": "events.log ACTIVATE t_k = livgo_hessian.csv t_k",
                        "first_traj_n_t_k": first, "trajectory_lag_s": first - activation,
                        "hessian_t_k_range": [min(times), max(times)],
                        "activation_wait_s": 30, "full_window": window, "eligible_samples": len(values)})
    thresholds = {}
    for dataset, values in result.items():
        if not values:
            raise ValueError("no eligible lambda samples for " + dataset)
        ordered = sorted(values)
        index = (len(ordered) - 1) * .05
        lower, fraction = int(math.floor(index)), index - math.floor(index)
        value = ordered[lower] if lower == len(ordered) - 1 else ordered[lower] + fraction * (ordered[lower + 1] - ordered[lower])
        thresholds[dataset] = {"lambda_th": value, "samples": len(values), "quantile": .05, "method": "Hyndman-Fan type 7"}
    return thresholds, sources


def registry_entry(name, calibration):
    return {"traj": "traj_N.tum", "overlay": "config/arms/" + name + ".yaml", "build": "head",
            "gnss_inputs": True, "time_calib": False, "calibration": calibration,
            "label": "A81/A82 frozen " + ("development check; never an evaluation arm" if calibration else "SmartPNT final evaluation arm")}


def save_artifacts(directory, record, overlays):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=False)
    for name, text in overlays.items():
        (directory / (name + ".yaml")).write_text(text)
        record["overlays"][name]["artifact"] = N.fingerprint(directory / (name + ".yaml"))
    (directory / "arms_append.json").write_text(json.dumps({name: data["registry"] for name, data in record["overlays"].items()}, indent=2) + "\n")
    (directory / "freeze.json").write_text(json.dumps(record, indent=2, allow_nan=False) + "\n")
    return record


def install(repo, record):
    verify_protected(repo, record["original_candidates"])
    path = Path(repo) / "tools/eval/arms.json"
    registry = N.strict_json(path)
    for name, data in record["overlays"].items():
        reference(data["artifact"])
        target = Path(repo) / data["registry"]["overlay"]
        if name in registry["arms"] and registry["arms"][name] != data["registry"]:
            raise ValueError("refusing an existing registry entry: " + name)
        if target.exists() and N.fingerprint(target)["sha256"] != data["artifact"]["sha256"]:
            raise ValueError("refusing an existing overlay file: " + str(target))
    # All conflicts are checked before any installation. Originals are never written.
    for name, data in record["overlays"].items():
        target = Path(repo) / data["registry"]["overlay"]
        target.write_bytes(Path(data["artifact"]["path"]).read_bytes())
        registry["arms"][name] = data["registry"]
    temporary = path.with_name(path.name + ".a80-freeze-tmp")
    temporary.write_text(json.dumps(registry, indent=2) + "\n")
    temporary.replace(path)
    verify_protected(repo, record["original_candidates"])


def prepare(repo, root, proposal_path, recheck_path, selection_path, out_dir):
    originals = protected(repo)
    scales, inputs = checked_noise(proposal_path, recheck_path)
    selection = N.strict_json(selection_path)
    inputs.append(N.fingerprint(selection_path))
    records, reports = read_reports(selection["ate_files"], repo, root)
    inputs.extend(reports)
    binaries = [{**reference(spec), "build_path": spec.get("build_path", spec["path"])}
                for spec in selection["binary_files"]]
    if len(binaries) < 2:
        raise ValueError("pin the replay executable and GNSS library")
    ratios, chosen_rows, builds = {"model": [], "flat": []}, {"model": [], "flat": []}, set()
    runtime_configs = {mode: {dataset: [] for dataset in DATASETS} for mode in ("model", "flat")}
    model_by_sequence = {}
    u_overlay_path = Path(repo) / N.strict_json(Path(repo) / "tools/eval/arms.json")["arms"]["U"]["overlay"]
    inputs.append(N.fingerprint(u_overlay_path))
    u_overlay = yaml.safe_load(u_overlay_path.read_text())
    for sequence, dataset in N.DEVELOPMENT.items():
        baseline = find_run(records, sequence, "U")
        u = N.finite(baseline["metrics"]["e1c"]["value"], "U ATE", positive=True)
        base = runtime_base(repo, dataset)
        u_config, directory, provenance = run_config(root, baseline)
        check_effective(merged(base, u_overlay), u_config)
        u_host = {key: value for key, value in leaves(u_config).items() if key not in IDENTITY and not key.startswith("gnss/")}
        verify_build(directory, binaries, baseline["build_sha256"])
        inputs.extend(provenance)
        builds.add(baseline["build_sha256"])
        for mode in ("model", "flat"):
            row = find_run(records, sequence, selection["candidate_arms"][dataset][mode])
            config, directory, provenance = run_config(root, row)
            verify_build(directory, binaries, row["build_sha256"])
            inputs.extend(provenance)
            if config["gnss"]["livgo"]["noise_mode"] != mode or config["gnss"]["livgo"]["lambda_th"] != 0:
                raise ValueError("candidate mode or collection lambda differs")
            base_name = "LIVGO-literal" if mode == "model" else "LIVGO-literal-paper"
            overlay = yaml.safe_load((Path(repo) / originals[base_name]["entry"]["overlay"]).read_text())
            if mode == "model":
                overlay["gnss"]["noise"]["scale"] = scales[dataset]
            check_effective(merged(base, overlay), config, diagnostic=True)
            candidate_host = {key: value for key, value in leaves(config).items() if key not in IDENTITY and not key.startswith("gnss/")}
            if candidate_host != u_host:
                raise ValueError("U and noise candidates differ in effective host configuration")
            runtime_configs[mode][dataset].append(config)
            if mode == "model":
                model_by_sequence[sequence] = config
            builds.add(row["build_sha256"])
            ratios[mode].append(1.0 if row.get("activated") is False else row["metrics"]["e1c"]["value"] / u)
            chosen_rows[mode].append(row)
    if len(builds) != 1:
        raise ValueError("U and candidates must use one identical build")
    for document_path in (proposal_path, recheck_path):
        for row in N.strict_json(document_path)["runs"]:
            manifest_spec = next(spec for spec in row["inputs"] if Path(spec["path"]).name == "run_manifest.json")
            manifest = N.strict_json(manifest_spec["path"])
            if manifest.get("build_sha256") != next(iter(builds)):
                raise ValueError("noise calibration and selection builds differ")
            verify_build(Path(manifest_spec["path"]).parent, binaries, next(iter(builds)))
            config = manifest["config_dump"]
            config = json.loads(config, parse_int=lambda text: -0.0 if text == "-0" else int(text)) if isinstance(config, str) else config
            dataset = N.DEVELOPMENT[row["sequence"]]
            base = runtime_base(repo, dataset)
            overlay = yaml.safe_load((Path(repo) / originals["LIVGO-literal-diag"]["entry"]["overlay"]).read_text())
            if document_path == recheck_path:
                overlay["gnss"]["noise"]["scale"] = scales[dataset]
            check_effective(merged(base, overlay), config)
            normalized = copy.deepcopy(config)
            normalized["gnss"]["noise"]["scale"].update(scales[dataset])
            actual_leaves = {k: v for k, v in leaves(normalized).items() if k not in IDENTITY}
            selected_leaves = {k: v for k, v in leaves(model_by_sequence[row["sequence"]]).items() if k not in IDENTITY}
            if actual_leaves != selected_leaves:
                raise ValueError("noise calibration source differs from the registered model selection configuration")
    medians = {mode: statistics.median(values) for mode, values in ratios.items()}
    if medians["model"] == medians["flat"]:
        raise ValueError("noise candidates tie; record an explicit decision before preparing")
    winner = min(medians, key=medians.get)
    thresholds, lambda_inputs = lambda_samples(root, chosen_rows[winner])
    record = {"schema": "a80-freeze/v1", "stage": "prepared_development_only", "evaluation_authorized": False,
              "winner": winner, "selection_ratios": ratios, "selection_median3": medians,
              "calibrated_model_scales": scales, "lambda": thresholds, "build_sha256": next(iter(builds)),
              "binary_files": binaries, "original_candidates": originals, "inputs": inputs + lambda_inputs,
              "source_configs": {}, "overlays": {}}
    texts = {}
    selected_base = "LIVGO-literal" if winner == "model" else "LIVGO-literal-paper"
    for dataset in DATASETS:
        base_path = Path(repo) / BASE_CONFIGS[dataset]
        record["source_configs"][dataset] = N.fingerprint(base_path)
        overlay = yaml.safe_load((Path(repo) / originals[selected_base]["entry"]["overlay"]).read_text())
        if winner == "model":
            overlay["gnss"]["noise"]["scale"] = scales[dataset]
        overlay["gnss"]["livgo"]["lambda_th"] = thresholds[dataset]["lambda_th"]
        name = DEV_ARMS[dataset]
        texts[name] = "# A81/A82 frozen values; the arm registry controls the experiment stage.\n" + yaml.safe_dump(overlay, sort_keys=True)
        selected_configs = runtime_configs[winner][dataset]
        if len({config_digest(config) for config in selected_configs}) != 1:
            raise ValueError("selected same-dataset runtime configurations differ")
        effective = merged(selected_configs[0], overlay)
        effective["gnss"]["output"]["sat_csv"] = False
        record["overlays"][name] = {"dataset": dataset, "registry": registry_entry(name, True),
                                    "effective_config": effective, "expected_config_sha256": config_digest(effective)}
    return save_artifacts(out_dir, record, texts)


def promote(repo, root, prepared_path, evidence_path, out_dir):
    record = N.strict_json(prepared_path)
    if record.get("schema") != "a80-freeze/v1" or record.get("stage") != "prepared_development_only":
        raise ValueError("promotion requires a prepared development-only freeze")
    verify_protected(repo, record["original_candidates"])
    for spec in record["binary_files"] + list(record["source_configs"].values()) + record["inputs"]:
        reference(spec)
    for data in record["overlays"].values():
        reference(data["artifact"])
    evidence = N.strict_json(evidence_path)
    rows, scores = read_reports(evidence["ate_files"], repo, root)
    checks = []
    config_hashes = {dataset: set() for dataset in DATASETS}
    for sequence, dataset in N.DEVELOPMENT.items():
        arm = DEV_ARMS[dataset]
        row = find_run(rows, sequence, arm)
        ratio = N.finite(row.get("path", {}).get("ratio"), "development path ratio", positive=True)
        if not .95 <= ratio <= 1.05:
            raise ValueError("final development path ratio failed the registered band")
        if row["build_sha256"] != record["build_sha256"]:
            raise ValueError("final development build differs from the frozen build")
        config, _, inputs = run_config(root, row)
        verify_build(Path(root) / row["run_dir"], record["binary_files"], record["build_sha256"])
        check_effective(record["overlays"][arm]["effective_config"], config)
        if row["config_sha256"] != record["overlays"][arm]["expected_config_sha256"]:
            raise ValueError("final development configuration SHA256 differs from prepared values")
        config_hashes[dataset].add(row["config_sha256"])
        checks.append({"sequence": sequence, "seed": 0, "arm": arm, "path_ratio": ratio,
                       "config_sha256": row["config_sha256"], "build_sha256": row["build_sha256"], "inputs": inputs})
    if any(len(values) != 1 for values in config_hashes.values()):
        raise ValueError("same-dataset final development configurations differ")
    name = "LIVGO-literal-final"
    smart = record["overlays"][DEV_ARMS["smartpnt_mini"]]
    text = Path(smart["artifact"]["path"]).read_text()
    promoted = {"schema": "a80-freeze/v1", "stage": "promoted_after_final_development", "evaluation_authorized": True,
                "prepared": N.fingerprint(prepared_path), "dev_evidence": N.fingerprint(evidence_path),
                "winner": record["winner"], "lambda": record["lambda"], "calibrated_model_scales": record["calibrated_model_scales"],
                "build_sha256": record["build_sha256"], "binary_files": record["binary_files"],
                "original_candidates": record["original_candidates"], "final_development_checks": checks,
                "score_files": scores, "config_sha256_by_dataset": {k: next(iter(v)) for k, v in config_hashes.items()},
                "overlays": {name: {"dataset": "smartpnt_mini", "registry": registry_entry(name, False),
                                      "effective_config": smart["effective_config"],
                                      "same_bytes_as_frozen_development_sha256": smart["artifact"]["sha256"]}}}
    return save_artifacts(out_dir, promoted, {name: text})


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    commands = parser.add_subparsers(dest="operation", required=True)
    for operation in ("prepare", "promote"):
        sub = commands.add_parser(operation)
        for name in ("repo", "root", "out-dir"):
            sub.add_argument("--" + name, required=True)
        sub.add_argument("--install", action="store_true")
        if operation == "prepare":
            for name in ("proposal", "recheck", "selection"):
                sub.add_argument("--" + name, required=True)
        else:
            sub.add_argument("--prepared", required=True)
            sub.add_argument("--dev-evidence", required=True)
    args = parser.parse_args(argv)
    if args.operation == "prepare":
        result = prepare(args.repo, args.root, args.proposal, args.recheck, args.selection, args.out_dir)
    else:
        result = promote(args.repo, args.root, args.prepared, args.dev_evidence, args.out_dir)
    if args.install:
        install(args.repo, result)
    print(json.dumps({"stage": result["stage"], "evaluation_authorized": result["evaluation_authorized"],
                      "winner": result["winner"], "arms": list(result["overlays"]), "out_dir": args.out_dir, "installed": args.install}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
