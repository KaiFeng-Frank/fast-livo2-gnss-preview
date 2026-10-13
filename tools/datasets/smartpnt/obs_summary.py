#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""What the HOST read from a converted bag: the satellites per epoch, C/N0, the carrier-valid fraction and the ephemeris
coverage, from the diagnostics of an OBSERVE run (arm T-obs: gnss.enable true, update_enable false, gnss.output.sat_csv true;
tools/eval/arms.json), set against the format-level statistics of tools/datasets/smartpnt/gnss_check.py. The run's own files
gnss_epochs.csv and gnss_sats.csv are read; nothing is replayed and no ground truth is used (A2: development runs only).

  obs_summary.py --run <T-obs run dir> [--check <gnss_check.json>] [--out summary.json]

Per-satellite rows are the host's: every primary-signal record that passed the bag decoding and the L0 masks (all open in T-obs).
numpy not needed (csv only).
"""
import argparse
import collections
import csv
import json
import math
import os
import statistics
import sys


def pct(v, q):
    v = sorted(v)
    if not v:
        return None
    k = (len(v) - 1) * q / 100.0
    lo, hi = int(math.floor(k)), int(math.ceil(k))
    return v[lo] + (v[hi] - v[lo]) * (k - lo)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--run", required=True)
    ap.add_argument("--check", default="", help="gnss_check.json of the same sequence, for the side-by-side")
    ap.add_argument("--out", default="")
    a = ap.parse_args(argv)
    out = {"run": a.run, "complete": os.path.isfile(os.path.join(a.run, "replay_stats.json"))}
    cols = {}
    with open(os.path.join(a.run, "gnss_epochs.csv"), newline="") as f:
        rd = csv.DictReader(f)
        rows = [r for r in rd if None not in r.values() and all(v != "" for v in r.values())]    # a run that was stopped may end in a cut line
    tot = [int(float(r["n_total_raw"])) for r in rows]
    out["epochs"] = {"n": len(rows), "phase_counts": dict(collections.Counter(r["phase"] for r in rows)),
                     "records_per_epoch": {"min": min(tot), "p10": pct(tot, 10), "median": pct(tot, 50), "p90": pct(tot, 90), "max": max(tot), "mean": statistics.mean(tot)},
                     "records_per_epoch_by_system_median": {k: pct([int(float(r["n_" + k])) for r in rows], 50) for k in ("gps", "glo", "gal", "bds", "qzs")}}
    cn0, el, cp, psr, dop = [], [], 0, 0, 0
    n = 0
    per_epoch = collections.defaultdict(lambda: [0, 0, 0, 0])      # records, raim-like, cp usable, psr usable
    reasons = collections.Counter()
    by_sys = collections.defaultdict(lambda: [0, 0, 0])            # rows, cp usable, no_eph
    with open(os.path.join(a.run, "gnss_sats.csv"), newline="") as f:
        for r in csv.DictReader(f):
            if None in r.values():
                continue
            n += 1
            c, e = float(r["cn0"]), float(r["el_deg"])
            k = (r["week"], r["tow"])
            cn0.append(c)
            el.append(e)
            cu, pu, du = r["cp_usable"] in ("1", "true"), r["psr_usable"] in ("1", "true"), r["dopp_usable"] in ("1", "true")
            cp += cu
            psr += pu
            dop += du
            pe = per_epoch[k]
            pe[0] += 1
            pe[1] += (c >= 35.0 and e >= 15.0)
            pe[2] += cu
            pe[3] += pu
            reasons[r["reason_psr"]] += 1
            s = by_sys[r["sat"][:1]]
            s[0] += 1
            s[1] += cu
            s[2] += (r["reason_psr"] == "no_eph")
    out["sat_rows"] = {"n": n, "cn0_dbhz": {"p10": pct(cn0, 10), "median": pct(cn0, 50), "p90": pct(cn0, 90)},
                       "elevation_deg_median": pct(el, 50), "carrier_usable_fraction": cp / max(n, 1),
                       "psr_usable_fraction": psr / max(n, 1), "doppler_usable_fraction": dop / max(n, 1),
                       "psr_reason_counts": dict(reasons.most_common(12)),
                       "no_ephemeris_fraction": reasons.get("no_eph", 0) / max(n, 1),
                       "by_system": {k: {"rows": v[0], "carrier_usable_fraction": v[1] / max(v[0], 1), "no_ephemeris_fraction": v[2] / max(v[0], 1)}
                                     for k, v in sorted(by_sys.items())},
                       "per_epoch": {"epochs": len(per_epoch), "records_median": pct([v[0] for v in per_epoch.values()], 50),
                                     "raim_c35_el15_median": pct([v[1] for v in per_epoch.values()], 50),
                                     "raim_c35_el15_p10": pct([v[1] for v in per_epoch.values()], 10),
                                     "carrier_usable_median": pct([v[2] for v in per_epoch.values()], 50)}}
    if a.check:
        with open(a.check) as f:
            st = json.load(f)["stats"]
        out["format_level_gnss_check"] = {"primary_sats_per_epoch_median": st["primary_sats_per_epoch"]["median"],
                                          "cn0_primary_dbhz": st["cn0_primary_dbhz"],
                                          "carrier_valid_fraction_primary": st["carrier_valid_fraction_primary"],
                                          "raim_sats_c35_el15_per_epoch_1Hz": st["raim_sats_c35_el15_per_epoch(1Hz)"],
                                          "healthy_ephemeris_fraction_with_broadcast_file": st["healthy_ephemeris_fraction_with_broadcast_file"],
                                          "healthy_ephemeris_fraction_bag_only_causal": st["healthy_ephemeris_fraction_bag_only_causal"]}
    txt = json.dumps(out, indent=1)
    if a.out:
        with open(a.out, "w") as f:
            f.write(txt + "\n")
    print(txt)
    return 0


if __name__ == "__main__":
    sys.exit(main())
