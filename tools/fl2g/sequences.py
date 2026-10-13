# SPDX-License-Identifier: BSD-3-Clause
"""M3DGR sequence classes (AMENDMENTS A2, A26): what a tool may compute on a sequence depends on its class.

evaluation  : format/decoding audits and bitwise comparisons only; no estimated trajectory is read, no GNSS-only
              solution is compared with anything (A2);
development : everything, including calibration-grade judgements (TEST_PLAN §3);
unknown     : treated like evaluation."""

EVAL_SEQUENCES = ("Varying-illu03", "Varying-illu04", "Varying-illu05", "Outdoor01", "Outdoor04", "Dark01", "Dark02",
                  "GNSS-denial01", "GNSS-denial02",
                  "Data01_20240719_100C_CAR_opensky", "Data00_20240703_100C_CAR_complex",            # A80: SmartPNT-MSF evaluation
                  "Data00_20240706_100C_CAR_complex")                                              # sequences (different days from the dev ones)
DEV_SEQUENCES = ("Occlusion03", "Occlusion04", "Dynamic03", "Grass01", "Grass02",     # A2 + A26 (Grass01)
                 "Z-Rough-Road01",                                                 # A69 item 3 / A70
                 "Data01_20240707_100C_CAR_complex", "Data02_20240718_100C_CAR_complex",           # A80: SmartPNT-MSF development
                 "HKisland_GNSS03")                                                               # A80: MARS-LVIG pipeline sequence


def seq_class(name):
    """A2 class of a sequence name; a derived name (e.g. "Outdoor01_filtered") keeps its sequence's class."""
    low = name.lower()
    if any(e.lower() in low for e in EVAL_SEQUENCES):
        return "evaluation"
    if any(d.lower() in low for d in DEV_SEQUENCES):
        return "development"
    return "unknown"
