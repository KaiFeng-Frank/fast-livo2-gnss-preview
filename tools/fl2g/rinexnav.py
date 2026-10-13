# SPDX-License-Identifier: BSD-3-Clause
"""RINEX 3.0x navigation reader (GPS, Galileo, BeiDou, QZSS, GLONASS) for the offline tools.

Records come back as dicts with RTKLIB eph_t/geph_t field names and absolute times in GPST seconds
(gnsstime.py). Per RINEX 3.04 §A: GPS/QZS epochs are GPST, GAL epochs GST (== GPST seconds, week aligned to the
GPS week), BDS epochs BDT (GPST - 14 s), GLONASS epochs UTC; RINEX stores -TauN for GLONASS (RTKLIB rinex.c
decode_geph negates it back). Units: SI (GLONASS km -> m).
"""
from . import gnsstime
from .sats import satid

_NORBIT = {"G": 7, "E": 7, "C": 7, "J": 7, "I": 7, "R": 3, "S": 3}


def _num(text):
    text = text.strip()
    if not text:
        return None
    return float(text.replace("D", "E").replace("d", "e"))


def _fields(line, first):
    """Four 19-character fields; the first line of a record carries three after the epoch."""
    if first:
        cols = ((23, 42), (42, 61), (61, 80))
    else:
        cols = ((4, 23), (23, 42), (42, 61), (61, 80))
    return [_num(line[a:b]) if len(line) > a else None for a, b in cols]


def _z(v):
    return 0.0 if v is None else v


def read_nav(path):
    """Return (header, records). header: version, iono_gps (alpha, beta), leap_s; records: list of dicts."""
    with open(path, "r", errors="replace") as f:
        lines = f.read().splitlines()
    header = {"version": None, "iono_gps_alpha": None, "iono_gps_beta": None, "leap_s": None, "path": path, "iono": []}
    i = 0
    while i < len(lines):
        ln = lines[i]
        label = ln[60:].strip()
        if label == "RINEX VERSION / TYPE":
            header["version"] = float(ln[:9])
        elif label == "IONOSPHERIC CORR":
            kind = ln[:4]
            vals = [_num(ln[5 + 12 * k:5 + 12 * (k + 1)]) for k in range(4)]
            # RINEX 3.04: time mark (A1, col 54) and transmitting SV (I2, cols 56-57) for BDS/QZS/IRN sets
            header["iono"].append({"kind": kind, "vals": vals, "mark": ln[54:55].strip(), "sv": ln[56:58].strip()})
            if kind == "GPSA":
                header["iono_gps_alpha"] = vals
            elif kind == "GPSB":
                header["iono_gps_beta"] = vals
        elif label == "LEAP SECONDS":
            try:
                header["leap_s"] = int(ln[:6])
            except ValueError:
                pass
        elif label == "END OF HEADER":
            i += 1
            break
        i += 1
    version = header["version"] or 3.04
    nglo = 4 if version >= 3.05 else 3
    recs = []
    while i < len(lines):
        ln = lines[i]
        if len(ln) < 23 or ln[0] not in _NORBIT or not ln[1:3].strip().isdigit():
            i += 1
            continue
        sys = ln[0]
        prn = int(ln[1:3])
        norb = nglo if sys == "R" else _NORBIT[sys]
        if i + norb >= len(lines):
            break
        try:
            ep = (int(ln[4:8]), int(ln[9:11]), int(ln[12:14]), int(ln[15:17]), int(ln[18:20]), float(ln[21:23]))
            data = _fields(ln, True)
            for k in range(1, norb + 1):
                data += _fields(lines[i + k], False)
        except (ValueError, IndexError):
            i += 1
            continue
        i += norb + 1
        t_epoch = gnsstime.calendar_to_seconds(*ep)
        if sys in ("G", "E", "C", "J"):
            recs.append(_kepler(sys, prn, t_epoch, data))
        elif sys == "R":
            recs.append(_glo(prn, t_epoch, data, header))
    return header, recs


def _kepler(sys, prn, t_epoch, d):
    d = [_z(x) for x in d] + [0.0] * max(0, 31 - len(d))
    week = int(round(d[21]))
    toes = d[11]
    if sys == "C":
        toc = t_epoch - gnsstime.BDT_MINUS_GPST_S            # BDT calendar -> GPST
        toe = gnsstime.bdt_to_gpst(week, toes)
        ttr = gnsstime.bdt_to_gpst(week, d[27])
    else:
        toc = t_epoch                                         # GPST / GST calendar
        toe = gnsstime.gpst(week, toes)
        ttr = gnsstime.gpst(week, d[27])
    # a toe near a week boundary can make ttr week-ambiguous; keep ttr within half a week of toe
    if ttr - toe > 302400.0:
        ttr -= gnsstime.WEEK_S
    elif toe - ttr > 302400.0:
        ttr += gnsstime.WEEK_S
    r = {
        "sys": sys, "prn": prn, "sat": satid(sys, prn), "source": "rinex",
        "toc": toc, "toe": toe, "ttr": ttr, "toes": toes, "week": week,
        "af0": d[0], "af1": d[1], "af2": d[2],
        "iode": int(round(d[3])), "crs": d[4], "deln": d[5], "M0": d[6],
        "cuc": d[7], "e": d[8], "cus": d[9], "A": d[10] * d[10],
        "cic": d[12], "OMG0": d[13], "cis": d[14],
        "i0": d[15], "crc": d[16], "omg": d[17], "OMGd": d[18],
        "idot": d[19], "code": int(round(d[20])), "flag": int(round(d[22])),
        "sva_m": d[23], "svh": int(round(d[24])),
        "tgd0": d[25], "tgd1": 0.0, "iodc": 0, "fit": 0.0, "aodc": 0,
    }
    if sys in ("G", "J"):
        r["iodc"] = int(round(d[26]))
        r["fit"] = d[28]
    elif sys == "E":
        r["tgd1"] = d[26]              # BGD E5b/E1 (tgd0 = BGD E5a/E1)
    elif sys == "C":
        r["tgd1"] = d[26]              # TGD2 B2/B3 (tgd0 = TGD1 B1/B3)
        r["aodc"] = int(round(d[28]))
    return r


def _glo(prn, t_epoch_utc, d, header):
    d = [_z(x) for x in d] + [0.0] * max(0, 16 - len(d))
    leap = header.get("leap_s") or gnsstime.leap_at_utc(t_epoch_utc)
    toe = t_epoch_utc + leap
    return {
        "sys": "R", "prn": prn, "sat": satid("R", prn), "source": "rinex",
        "toe": toe, "taun": -d[0], "gamn": d[1], "tof_raw": d[2],
        "pos": [d[3] * 1e3, d[7] * 1e3, d[11] * 1e3],
        "vel": [d[4] * 1e3, d[8] * 1e3, d[12] * 1e3],
        "acc": [d[5] * 1e3, d[9] * 1e3, d[13] * 1e3],
        "svh": int(round(d[6])), "frq": int(round(d[10])), "age": int(round(d[14])),
        "sva_m": None,
    }
