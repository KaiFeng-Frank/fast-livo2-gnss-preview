#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Write invented GPS ephemerides for the preview simulator; no real data is used."""
import argparse
from datetime import datetime, timezone
import math
from pathlib import Path

EPOCH = datetime(2025, 1, 21, 8, 0, tzinfo=timezone.utc)  # calendar expressed in GPST
GPS_ORIGIN = datetime(1980, 1, 6, tzinfo=timezone.utc)


def make_nav(path):
    gps_seconds = int((EPOCH - GPS_ORIGIN).total_seconds())
    week, tow = divmod(gps_seconds, 604800)
    lines = []

    def header(payload, label):
        lines.append(payload.ljust(60) + label)

    def numbers(values):
        return "".join(f"{value:19.12E}" for value in values)

    header("     3.04           N: GNSS NAV DATA    G: GPS", "RINEX VERSION / TYPE")
    header("preview make_nav.py  synthetic", "PGM / RUN BY / DATE")
    header("Invented 24-satellite constellation; no receiver or IGS data", "COMMENT")
    header("Only a software smoke example; not a real-data benchmark", "COMMENT")
    header("GPSA" + "".join(f"{v:12.4E}" for v in (2.5e-8, 1.0e-8, 0.0, 0.0)), "IONOSPHERIC CORR")
    header("GPSB" + "".join(f"{v:12.4E}" for v in (9.0e4, 0.0, 0.0, 0.0)), "IONOSPHERIC CORR")
    header("    18", "LEAP SECONDS")
    header("", "END OF HEADER")
    # Six equally spaced orbital planes, four satellites per plane. These
    # idealized orbital parameters are authored here, not copied from a NAV file.
    for prn in range(1, 25):
        plane, slot = divmod(prn - 1, 4)
        mean_anomaly = slot * math.pi / 2 + plane * math.pi / 12
        ascending_node = plane * math.pi / 3
        lines.append(f"G{prn:02d} {EPOCH:%Y %m %d %H %M %S}" + numbers((0.0, 0.0, 0.0)))
        for row in (
            (1.0, 0.0, 0.0, mean_anomaly),
            (0.0, 0.005, 0.0, math.sqrt(26560000.0)),
            (float(tow), 0.0, ascending_node, 0.0),
            (math.radians(55), 0.0, 0.0, -8.0e-9),
            (0.0, 1.0, float(week), 0.0),
            (2.0, 0.0, 0.0, 1.0),
            (float(tow), 4.0, 0.0, 0.0),
        ):
            lines.append("    " + numbers(row))
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="ascii")
    print(f"Wrote {path.name}: 24 invented GPS ephemerides, week {week}, TOW {tow}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    make_nav(parser.parse_args().output)
