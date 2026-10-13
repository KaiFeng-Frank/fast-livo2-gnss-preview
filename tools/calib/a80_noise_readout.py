#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Read A80 development-only model-noise evidence without changing any run.

  a80_noise_readout.py --run DIR --extract DIR [--run DIR --extract DIR ...] --out JSON
  a80_noise_readout.py --stage recheck --previous proposal.json ... --out recheck.json

CSV or CSV.gz is accepted. Doppler uses the A58/A62 p4_calib.review common-clock
white-equivalent statistic (accepted TRACKING rows, no extra snooping); TDCP uses
the filter's own pooled prior NIS/dof in TRACKING/RECOVERY, because chained phase
states make a row-only TDCP variance statistic invalid. SmartPNT's two development
sequences are pooled by dof, and MARS is kept separate. A proposal is one variance
scale update: scale_new = scale_old * pooled_factor. Recheck never proposes a
second update, changes a gate, writes a configuration, or starts an evaluation.
"""
import argparse
import csv
import gzip
import hashlib
import itertools
import json
import math
from pathlib import Path

DEVELOPMENT = {
    "Data01_20240707_100C_CAR_complex": "smartpnt_mini",
    "Data02_20240718_100C_CAR_complex": "smartpnt_mini",
    "HKisland_GNSS03": "mars_lvig",
}
D4 = (0.8, 1.25)
DT_DOPP = 0.1  # p4_calib.white_equivalent / core DoppTerms nominal receiver interval.


def strict_json(path):
    def pairs(items):
        result = {}
        for key, value in items:
            if key in result:
                raise ValueError("duplicate JSON key: " + key)
            result[key] = value
        return result
    return json.loads(Path(path).read_text(), object_pairs_hook=pairs)


def choose_csv(directory, name):
    candidates = [Path(directory) / name, Path(directory) / (name + ".gz")]
    existing = [p for p in candidates if p.is_file()]
    if len(existing) != 1:
        raise ValueError("need exactly one CSV or CSV.gz: " + str(candidates[0]))
    return existing[0]


def rows(path):
    opener = gzip.open if str(path).endswith(".gz") else open
    with opener(path, "rt", newline="") as source:
        reader = csv.DictReader(source)
        for index, row in enumerate(reader, 2):
            if None in row or any(v is None for v in row.values()):
                raise ValueError("malformed CSV row %s:%d" % (path, index))
            yield row


def epoch_key(row):
    week, tow = float(row["week"]), float(row["tow"])
    if not math.isfinite(week) or not math.isfinite(tow) or int(week) != week:
        raise ValueError("invalid receiver epoch")
    return int(week), int(round(tow * 1000.0))


def groups(path):
    last = None
    for key, batch in itertools.groupby(rows(path), epoch_key):
        if last is not None and key < last:
            raise ValueError("CSV receiver epochs must be monotonically ordered: " + str(path))
        last = key
        yield key, list(batch)


class EpochJoin:
    def __init__(self, path):
        self.iterator = iter(groups(path))
        self.current = next(self.iterator, None)

    def at(self, key):
        while self.current is not None and self.current[0] < key:
            self.current = next(self.iterator, None)
        return self.current[1] if self.current is not None and self.current[0] == key else None


def finite(value, label, *, positive=False, nonnegative=False):
    value = float(value)
    if not math.isfinite(value) or (positive and value <= 0) or (nonnegative and value < 0):
        raise ValueError("invalid " + label)
    return value


def model_from_config(config):
    gnss = config["gnss"]
    if gnss["livgo"].get("noise_mode", "model") != "model":
        raise ValueError("this readout is only for the model-noise candidate")
    noise = gnss["noise"]
    # Use actual manifest values, not p4_calib.OLD's M3DGR placeholders.
    model = {"psr": dict(noise["psr"]), "dopp": dict(noise["dopp"]), "scale": dict(noise["scale"])}
    model["psr"]["sys"] = model["psr"].pop("sys_factor")
    model["psr"]["geo"] = gnss.get("systems", {}).get("bds_geo", {}).get("psr_scale", 2.0)
    finite(model["dopp"]["sigma0_mps"], "sigma0_mps", positive=True)
    finite(model["dopp"]["tau_s"], "dopp tau_s", nonnegative=True)
    for kind in ("dopp", "tdcp"):
        finite(model["scale"][kind], kind + " scale", positive=True)
    return model


def kappa(tau, dt):
    return 1.0 if tau <= 0 or dt / tau > 700 else 1.0 + 2.0 / math.expm1(dt / tau)


def common_clock_stat(innovations, variances):
    """Weighted elimination of the one common drift column, no row rejection."""
    if len(innovations) < 2:
        return 0.0, 0
    weights = [1.0 / r for r in variances]
    mean = math.fsum(w * nu for w, nu in zip(weights, innovations)) / math.fsum(weights)
    nis = math.fsum((nu - mean) ** 2 / r for nu, r in zip(innovations, variances))
    return nis, len(innovations) - 1


def self_tdcp(path):
    nis, dof, epochs, accepted, candidates = 0.0, 0, 0, 0, 0
    for row in rows(path):
        if row["phase"] not in ("TRACKING", "RECOVERY"):
            continue
        accepted += int(row["tdcp_acc"])
        candidates += int(row["tdcp_cand"])
        d = finite(row["tdcp_dof"], "TDCP dof", nonnegative=True)
        if int(d) != d:
            raise ValueError("noninteger TDCP dof")
        if d > 0:
            nis += finite(row["tdcp_nis_prior"], "TDCP prior NIS", nonnegative=True)
            dof += int(d)
            epochs += 1
    return {"nis_sum": nis, "dof_sum": dof, "epochs": epochs,
            "accepted": accepted, "candidates": candidates,
            "pooled_nis_per_dof": nis / dof if dof else None}


def white_doppler(sats_path, epochs_path, meas_path, model):
    epoch_join, meas_join = EpochJoin(epochs_path), EpochJoin(meas_path)
    kapp = kappa(model["dopp"]["tau_s"], DT_DOPP)
    total_nis, total_dof, used_rows, used_epochs, singles = 0.0, 0, 0, 0, 0
    min_white = None
    for key, sat_rows in groups(sats_path):
        ep_rows = epoch_join.at(key)
        if ep_rows is None:
            raise ValueError("diagnostic receiver epoch missing from gnss_epochs")
        if ep_rows[-1]["phase"] != "TRACKING":
            continue
        accepted = [row for row in sat_rows if row["reason_dopp"] == "none" and
                    finite(row["sigma_dopp"], "logged sigma_dopp", nonnegative=True) > 0]
        if not accepted:
            continue
        meas_rows = meas_join.at(key)
        if meas_rows is None:
            raise ValueError("accepted diagnostic receiver epoch missing from meas_rows")
        # Match p4_calib.load_meas: the last row for an epoch/satellite wins.
        meas = {row["sat"]: row for row in meas_rows}
        innovations, variances = [], []
        for row in accepted:
            if row["sat"] not in meas:
                raise ValueError("accepted diagnostic satellite missing from meas_rows")
            observation = meas[row["sat"]]
            freq = finite(observation["freq_hz"], "receiver frequency", positive=True)
            rcv_std = finite(observation["dopp_std_hz"], "receiver Doppler std", nonnegative=True)
            el = finite(row["el_deg"], "elevation") * math.pi / 180.0
            sin_el = max(math.sin(el), math.sin(5.0 * math.pi / 180.0))
            sigma0 = max(299792458.0 / freq * rcv_std, model["dopp"]["sigma0_mps"])
            rcv_variance = sigma0 ** 2 * 0.5 * (1.0 + 1.0 / sin_el ** 2)
            white = float(row["sigma_dopp"]) ** 2 - model["scale"]["dopp"] * rcv_variance * (kapp - 1.0)
            finite(white, "nonpositive white-equivalent Doppler variance", positive=True)
            min_white = white if min_white is None else min(min_white, white)
            innovations.append(finite(row["innov_dopp"], "Doppler innovation"))
            variances.append(max(white, 1e-12))  # Same numerical floor as p4_calib.review_rows, after validity check.
        value, dof = common_clock_stat(innovations, variances)
        if dof:
            total_nis += value
            total_dof += dof
            used_rows += len(innovations)
            used_epochs += 1
        else:
            singles += 1
    return {"nis_sum": total_nis, "dof_sum": total_dof, "rows": used_rows, "epochs": used_epochs,
            "single_row_epochs_skipped": singles, "kappa_dopp": kapp, "nominal_receiver_dt_s": DT_DOPP,
            "minimum_white_variance": min_white, "pooled_nis_per_dof": total_nis / total_dof if total_dof else None}


def fingerprint(path):
    digest = hashlib.sha256()
    with open(path, "rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return {"path": str(Path(path).absolute()), "bytes": Path(path).stat().st_size, "sha256": digest.hexdigest()}


def time_calibration_disabled(config):
    time = config["gnss"].get("time", {})
    if time.get("estimate_online") is not False or time.get("calib_file") != "":
        raise ValueError("evidence must keep time calibration disabled")


def sequence_identity(manifest, config):
    """run_matrix uses <sequence>_<arm>_s<seed> for the upstream EVO filename."""
    name = config["evo"]["seq_name"]
    for sequence in DEVELOPMENT:
        if name == "%s_%s_s%s" % (sequence, manifest.get("arm"), manifest.get("seed")):
            return sequence
    raise ValueError("refusing non-A80-development or unregistered replay sequence identity: " + str(name))


def literal_source(manifest, config, sequence):
    """The registered A82 model diagnostic source, before any statistic is read."""
    dataset = DEVELOPMENT[sequence]
    permitted = {"LIVGO-literal-diag", "LIVGO-literal-recheck-" + dataset.replace("_mini", "").replace("_lvig", "") + "-diag"}
    if manifest.get("seed") != 0 or manifest.get("arm") not in permitted:
        raise ValueError("noise evidence requires seed-0 registered literal model diagnostic arms")
    gnss = config["gnss"]
    literal = {"enable": True, "paper_init": True, "frame_exact": True, "freeze_yaw": True,
               "cn0_hard": True, "primary_only": True, "tdcp_raim_both_ends": True,
               "dtw_online": True, "dtw_at_init": True, "single_init": True,
               "init_travel_m": 50, "init_tilt_sigma_max_deg": .2, "dtw_grid_s": .1,
               "cn0_min_dbhz": 35, "el_min_deg": 15, "lambda_th": 0,
               "lambda_source": "lio", "noise_mode": "model"}
    if any(gnss.get("livgo", {}).get(key) != value or
           isinstance(gnss.get("livgo", {}).get(key), bool) != isinstance(value, bool)
           for key, value in literal.items()) or gnss.get("output", {}).get("sat_csv") is not True:
        raise ValueError("noise evidence is not the registered A82 literal diagnostic configuration")
    time_calibration_disabled(config)


def one_run(run, extract, allow_partial):
    run, extract = Path(run), Path(extract)
    manifest_path, stats_path = run / "run_manifest.json", run / "replay_stats.json"
    manifest, stats = strict_json(manifest_path), strict_json(stats_path)
    config = manifest["config_dump"]
    config = json.loads(config) if isinstance(config, str) else config
    sequence = sequence_identity(manifest, config)
    literal_source(manifest, config, sequence)
    if stats.get("complete") is not True:
        raise ValueError("refusing an incomplete replay: " + str(run))
    partial = float(config["replay"]["duration_s"]) > 0 or float(config["replay"]["start_offset_s"]) != 0
    if partial and not allow_partial:
        raise ValueError("partial replay is only a smoke readout; use --allow-partial explicitly")
    model = model_from_config(config)
    epochs, sats, meas = [choose_csv(directory, name) for directory, name in
                          ((run, "gnss_epochs.csv"), (run, "gnss_sats.csv"), (extract, "meas_rows.csv"))]
    inputs = (manifest_path, stats_path, epochs, sats, meas)
    before = {p: (p.stat().st_size, p.stat().st_mtime_ns, p.stat().st_ctime_ns) for p in inputs}
    dopp = white_doppler(sats, epochs, meas, model)
    tdcp = self_tdcp(epochs)
    hashes = [fingerprint(p) for p in inputs]
    if any(before[p] != (p.stat().st_size, p.stat().st_mtime_ns, p.stat().st_ctime_ns) for p in inputs):
        raise ValueError("input changed during readout")
    if dopp["dof_sum"] <= 0 or tdcp["dof_sum"] <= 0:
        raise ValueError("Doppler and TDCP each need positive degrees of freedom")
    return {"sequence": sequence, "evo_seq_name": config["evo"]["seq_name"],
            "dataset": DEVELOPMENT[sequence], "run": str(run), "extract": str(extract),
            "partial": partial, "seed": manifest.get("seed"), "arm": manifest.get("arm"),
            "config_sha256": manifest.get("config_sha256"), "git_describe": manifest.get("git_describe"),
            "actual_noise_config": config["gnss"]["noise"], "p4_model": model,
            "doppler_white_common": dopp, "tdcp_self_reported": tdcp, "inputs": hashes}


def pool(records, stage, previous=None):
    result = {}
    for dataset in sorted({record["dataset"] for record in records}):
        members = [record for record in records if record["dataset"] == dataset]
        if len({record["sequence"] for record in members}) != len(members):
            raise ValueError("duplicate development sequence within a readout")
        if any(record["actual_noise_config"] != members[0]["actual_noise_config"] for record in members):
            raise ValueError("cannot pool different noise configurations for " + dataset)
        entry = {"sequences": [record["sequence"] for record in members],
                 "complete_development_coverage": {r["sequence"] for r in members} ==
                     {s for s, group in DEVELOPMENT.items() if group == dataset},
                 "full_replays": all(not r["partial"] for r in members), "statistics": {}}
        unchanged_noise = json.loads(json.dumps(members[0]["actual_noise_config"]))
        for kind in ("dopp", "tdcp"):
            del unchanged_noise["scale"][kind]
        entry["noise_without_calibrated_scales"] = unchanged_noise
        if stage == "recheck" and unchanged_noise != previous["groups"][dataset]["noise_without_calibrated_scales"]:
            raise ValueError("recheck changed noise parameters beyond the two registered scales")
        for kind, field in (("dopp", "doppler_white_common"), ("tdcp", "tdcp_self_reported")):
            nis = math.fsum(record[field]["nis_sum"] for record in members)
            dof = sum(record[field]["dof_sum"] for record in members)
            factor = nis / dof if dof else None
            old = float(members[0]["p4_model"]["scale"][kind])
            if stage == "recheck":
                expected = previous["groups"][dataset]["statistics"][kind]["proposed_scale"]
                if expected is None or not math.isclose(old, expected, rel_tol=1e-9, abs_tol=1e-12):
                    raise ValueError("recheck scale does not match the one registered proposal")
            entry["statistics"][kind] = {"nis_sum": nis, "dof_sum": dof, "pooled_factor": factor,
                "current_scale": old, "proposed_scale": old * factor if stage == "proposal" and factor is not None else None,
                "d4_pass": factor is not None and D4[0] <= factor <= D4[1]}
        entry["d4_all_pass"] = all(s["d4_pass"] for s in entry["statistics"].values())
        result[dataset] = entry
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--run", action="append", required=True)
    parser.add_argument("--extract", action="append", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--allow-partial", action="store_true")
    parser.add_argument("--stage", choices=("proposal", "recheck"), default="proposal")
    parser.add_argument("--previous")
    args = parser.parse_args(argv)
    if len(args.run) != len(args.extract):
        parser.error("one --extract is required per --run")
    if (args.stage == "recheck") != bool(args.previous):
        parser.error("--previous is required only for the one recheck")
    previous = strict_json(args.previous) if args.previous else None
    if previous is not None and previous.get("stage") != "proposal":
        parser.error("recheck requires a proposal, not another recheck")
    records = [one_run(run, extract, args.allow_partial) for run, extract in zip(args.run, args.extract)]
    pooled = pool(records, args.stage, previous)
    output = {"schema": "a80-model-noise-readout/v1", "stage": args.stage, "development_only": True,
              "d4_interval": list(D4), "automatic_evaluation_authorization": False,
              "calibration_rounds_allowed": 1, "scale_is_variance_multiplier": True,
              "method": "A58/A62 common-clock white-equivalent Doppler; filter self-reported prior TDCP NIS",
              "previous": fingerprint(args.previous) if args.previous else None,
              "runs": records, "groups": pooled,
              "ready_for_external_freeze_review": args.stage == "recheck" and set(pooled) == {"smartpnt_mini", "mars_lvig"}
                  and all(g["complete_development_coverage"] and g["full_replays"] and g["d4_all_pass"] for g in pooled.values())}
    path = Path(args.out)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(output, indent=2, allow_nan=False) + "\n")
    print(json.dumps({"out": str(path), "stage": args.stage, "groups": pooled,
                      "ready_for_external_freeze_review": output["ready_for_external_freeze_review"]}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
