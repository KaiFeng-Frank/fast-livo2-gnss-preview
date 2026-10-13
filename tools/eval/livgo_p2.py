# SPDX-License-Identifier: BSD-3-Clause
"""Prediction P2 of AMENDMENTS A75 (as rewritten in A75 revision 1): do the TDCP rows of LIVGO-v2 act on the host?

    livgo_p2.py <matrix dir> [<matrix dir> ...] [--json OUT]

Reads <matrix>/<sequence>/<arm>/s<seed>/livgo_tdcp_gain.csv (written by the LIVGO facade with livgo/tdcp_gain_log; arms
LIVGO-v2, LIVGO-v2-paper and the negative control LIVGO-legacy-p2). One row per committed epoch that closed a TDCP pair:

    t_k, phase, n_psr, n_dopp, n_tdcp, trp_prior, trp_all, trp_noT, status

trp_* are traces of the host position block (m²) of the prior, of the epoch's posterior and of the posterior of the same epoch
without its TDCP rows (a second filter update on copies of the state, same prior, same gates). status 1 = counterfactual
computed, 0 = it rolled back, 2 = no accepted TDCP row (trp_noT = trp_all).

    g = 1 − trp_all / trp_noT      per epoch with phase TRACKING, n_tdcp ≥ 1 and status 1: the share of the position variance
                                   that is left after everything but TDCP which the TDCP rows remove in addition.

The criteria below were written down before any run of the instrument existed (AMENDMENTS A75 revision 1, the P2 text):

    P2a  alive     LIVGO-v2: the median g of every run is ≥ ALIVE_MIN (1e-2).
    P2b  dose      LIVGO-v2: in every run in which both strata (n_tdcp 1–3, n_tdcp ≥ 4) have ≥ STRATUM_MIN (30) epochs,
                   the median g of the ≥ 4 stratum is larger than that of the 1–3 stratum. A run without both strata is
                   "not evaluable": neither pass nor fail.
    P2c  control   LIVGO-legacy-p2 (the legacy arm, whose TDCP rows the translation variance absorbs): the median g of every
                   run is < DEAD_MAX (1e-3). If the control fails the instrument cannot tell dead rows from live ones, and
                   P2a is not read.
    instrument     per run, the share of status 0 among the epochs with n_tdcp ≥ 1 is ≤ STATUS0_MAX (0.05), and the posterior
                   position trace is not above the prior's (relative 1e-6) in ≥ 99 % of the rows; else P2 is "undecided".

P2 holds iff P2a and P2c hold in every run, P2b wherever evaluable, and the instrument checks pass. LIVGO-v2-paper is reported
with the same numbers and no pass/fail. Coverage (the share of closing epochs with n_tdcp ≥ 1) is reported, not predicted.
"""
import argparse
import csv
import glob
import json
import math
import os
import sys

ALIVE_MIN = 1e-2
DEAD_MAX = 1e-3
STRATUM_MIN = 30
STATUS0_MAX = 0.05
INFLATE_TOL = 1e-6
INFLATE_MAX_FRAC = 0.01
TRACKING = 6            # int(Phase::TRACKING)
ARM_V2, ARM_PAPER, ARM_CONTROL = "LIVGO-v2", "LIVGO-v2-paper", "LIVGO-legacy-p2"
FILE = "livgo_tdcp_gain.csv"
COLUMNS = ["t_k", "phase", "n_psr", "n_dopp", "n_tdcp", "trp_prior", "trp_all", "trp_noT", "status"]


def quantile(xs, q):
    """Hyndman–Fan type 7 (the percentile the A73/A75 calibrations use)."""
    s = sorted(xs)
    if not s:
        return float("nan")
    h = (len(s) - 1) * q
    lo = int(math.floor(h))
    hi = min(lo + 1, len(s) - 1)
    return s[lo] + (h - lo) * (s[hi] - s[lo])


def median(xs):
    return quantile(xs, 0.5)


def read_rows(path):
    with open(path, newline="") as f:
        r = csv.reader(f)
        head = next(r)
        if head != COLUMNS:
            raise ValueError("%s: unexpected header %s" % (path, ",".join(head)))
        rows = []
        for line in r:
            if not line:
                continue
            rows.append({"t_k": float(line[0]), "phase": int(line[1]), "n_psr": int(line[2]), "n_dopp": int(line[3]),
                         "n_tdcp": int(line[4]), "prior": float(line[5]), "all": float(line[6]), "no": float(line[7]),
                         "status": int(line[8])})
    return rows


def summarise(rows):
    """Per-run numbers. `rows` as read_rows returns them."""
    trk = [r for r in rows if r["phase"] == TRACKING]
    with_t = [r for r in trk if r["n_tdcp"] >= 1]
    ok = [r for r in with_t if r["status"] == 1 and r["no"] > 0.0 and math.isfinite(r["no"])]
    g = [1.0 - r["all"] / r["no"] for r in ok]
    lo = [1.0 - r["all"] / r["no"] for r in ok if 1 <= r["n_tdcp"] <= 3]
    hi = [1.0 - r["all"] / r["no"] for r in ok if r["n_tdcp"] >= 4]
    n_with = len(with_t)
    s0 = sum(1 for r in with_t if r["status"] == 0)
    infl = sum(1 for r in trk if r["status"] != 0 and r["prior"] > 0.0 and r["all"] > r["prior"] * (1.0 + INFLATE_TOL))
    return {
        "n_closing": len(trk), "n_with_tdcp": n_with, "coverage": (n_with / len(trk)) if trk else float("nan"),
        "n_g": len(g), "g_median": median(g), "g_p10": quantile(g, 0.1), "g_p90": quantile(g, 0.9),
        "n_lo": len(lo), "g_median_1to3": median(lo), "n_hi": len(hi), "g_median_ge4": median(hi),
        "status0_frac": (s0 / n_with) if n_with else float("nan"),
        "inflate_frac": (infl / len(trk)) if trk else float("nan"),
    }


def run_checks(arm, s):
    """The registered checks of one run -> {name: True|False|None} (None: not evaluable)."""
    c = {}
    inst = (s["n_g"] > 0 and s["status0_frac"] <= STATUS0_MAX and s["inflate_frac"] <= INFLATE_MAX_FRAC)
    c["instrument"] = bool(inst)
    if arm == ARM_V2:
        c["P2a"] = bool(s["n_g"] > 0 and s["g_median"] >= ALIVE_MIN)
        if s["n_lo"] >= STRATUM_MIN and s["n_hi"] >= STRATUM_MIN:
            c["P2b"] = bool(s["g_median_ge4"] > s["g_median_1to3"])
        else:
            c["P2b"] = None
    elif arm == ARM_CONTROL:
        c["P2c"] = bool(s["n_g"] > 0 and s["g_median"] < DEAD_MAX)
    return c


def verdict(runs):
    """runs: list of dicts {seq, arm, seed, summary, checks}. -> (verdict string, why list)."""
    why = []
    v2 = [r for r in runs if r["arm"] == ARM_V2]
    ctl = [r for r in runs if r["arm"] == ARM_CONTROL]
    if not v2:
        return "NO DATA", ["no LIVGO-v2 run"]
    if not ctl:
        return "UNDECIDED", ["no LIVGO-legacy-p2 control run: P2c missing"]
    if any(not r["checks"]["instrument"] for r in v2 + ctl):
        bad = [r["seq"] + "/" + r["arm"] + "/s" + str(r["seed"]) for r in v2 + ctl if not r["checks"]["instrument"]]
        return "UNDECIDED", ["instrument check failed: " + ", ".join(bad)]
    if any(not r["checks"]["P2c"] for r in ctl):
        bad = [r["seq"] + "/s" + str(r["seed"]) for r in ctl if not r["checks"]["P2c"]]
        return "UNDECIDED", ["negative control not dead (median g >= %g): %s" % (DEAD_MAX, ", ".join(bad))]
    fail_a = [r["seq"] + "/s" + str(r["seed"]) for r in v2 if not r["checks"]["P2a"]]
    fail_b = [r["seq"] + "/s" + str(r["seed"]) for r in v2 if r["checks"]["P2b"] is False]
    if fail_a:
        why.append("P2a fails (median g < %g): %s" % (ALIVE_MIN, ", ".join(fail_a)))
    if fail_b:
        why.append("P2b fails (no dose response): %s" % ", ".join(fail_b))
    return ("NOT MET" if why else "MET"), why


def collect(matrices):
    runs = []
    for m in matrices:
        for p in sorted(glob.glob(os.path.join(m, "*", "*", "s*", FILE))):
            parts = os.path.normpath(p).split(os.sep)
            seed_dir, arm, seq = parts[-2], parts[-3], parts[-4]
            seed = seed_dir[1:].split(".")[0]
            s = summarise(read_rows(p))
            runs.append({"matrix": m, "seq": seq, "arm": arm, "seed": seed, "summary": s, "checks": run_checks(arm, s)})
    return runs


def fmt(x):
    return "nan" if x != x else ("%.3g" % x)


def report(runs, out=sys.stdout):
    out.write("run (sequence/arm/seed)                     closing  with_TDCP  cover   median g   p10..p90        1-3 rows   >=4 rows   s0     checks\n")
    for r in runs:
        s = r["summary"]
        chk = " ".join("%s=%s" % (k, {True: "ok", False: "FAIL", None: "n/e"}[v]) for k, v in sorted(r["checks"].items()))
        out.write("%-42s %7d %9d %6s %10s %s..%s %10s %10s %6s  %s\n" % (
            "%s/%s/s%s" % (r["seq"], r["arm"], r["seed"]), s["n_closing"], s["n_with_tdcp"], fmt(s["coverage"]), fmt(s["g_median"]),
            fmt(s["g_p10"]), fmt(s["g_p90"]), fmt(s["g_median_1to3"]), fmt(s["g_median_ge4"]), fmt(s["status0_frac"]), chk))
    v, why = verdict(runs)
    out.write("P2: %s\n" % v)
    for w in why:
        out.write("  - %s\n" % w)
    return v


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("matrix", nargs="+", help="runs/eval/<name> (repeatable)")
    ap.add_argument("--json", default="", help="write the per-run numbers and the verdict here")
    a = ap.parse_args(argv)
    runs = collect(a.matrix)
    v = report(runs)
    if a.json:
        _, why = verdict(runs)
        with open(a.json, "w") as f:
            json.dump({"verdict": v, "why": why, "thresholds": {"alive_min": ALIVE_MIN, "dead_max": DEAD_MAX, "stratum_min": STRATUM_MIN,
                                                                 "status0_max": STATUS0_MAX},
                       "runs": runs}, f, indent=1, sort_keys=True)
    return 0 if v in ("MET", "NOT MET") else 2


if __name__ == "__main__":
    sys.exit(main())
