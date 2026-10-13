# SPDX-License-Identifier: BSD-3-Clause
"""RINEX 3 observations / navigation records -> gnss_comm ROS1 message bytes.

The inverse of tools/gnss_bag_export.py (message -> RINEX); the field semantics are exactly those the exporter and the
C++ reader (gnss_io/src/gnss_comm_mapping.cpp) agree on:

  GnssMeasMsg      one epoch; per satellite the signals of the RTKLIB-EX slots (rinex.Selection) that the signal table
                   accepts (GPS L1/L2/L5, GAL E1/E5b/E5a, BDS B1I/B2, GLO G1/G2), primary (L1-class) first.
                   freqs [Hz], CN0 [dB-Hz], psr [m], cp [cycles], dopp [Hz] (RINEX sign = gnss_comm sign: positive for an
                   approaching satellite), LLI = RINEX LLI bit 0 | (2 if the phase is flagged half-cycle), status bits
                   0 psr valid / 1 cp valid / 2 half-cycle resolved / 3 half-cycle subtracted (never set from RINEX).
                   The std fields are 0: a RINEX file has no receiver std, 0 means "not available" to the core
                   (quality.cpp StdUnitJudge leaves the unit unknown and the noise-model floor applies).
  GnssEphemMsg     GPS / GAL / BDS broadcast set: toe/toc/ttr (week, tow) in GPST (BDS: BDT + 14 s), week = GPS week of toe,
                   toe_tow = GPST time of week of toe, ura = SV accuracy in metres (A16 "metres" semantics), tgd0/tgd1 as
                   the RINEX record (GPS TGD; GAL BGD E5a/E1, E5b/E1; BDS TGD1, TGD2), iodc = AODC for BDS.
  GnssGloEphemMsg  GLONASS: ttr/toe GPST, freqo, iode = tb index (15-min slots of the Moscow day), age, positions in m,
                   tau_n = -(RINEX SV clock bias), delta_tau_n = 0 and ura = 0 (neither is in a RINEX 3.0x file).
  iono_params      StampedFloat64Array with the GPSA/GPSB Klobuchar set of the NAV header.

Message definitions and md5 sums are NOT written down here (docs/LICENSING.md): they are read from the connection
headers of a reference bag (the M3DGR bag that already carries the gnss_comm topics) and used verbatim; the field layout
this module packs is checked against them field by field before anything is written.
"""
import math
import struct

from fl2g import gnsstime, rosio
from fl2g.sats import signal_from_frequency

from . import rinex

TOPIC_MEAS = "/ublox_driver/range_meas"
TOPIC_EPHEM = "/ublox_driver/ephem"
TOPIC_GLO_EPHEM = "/ublox_driver/glo_ephem"
TOPIC_IONO = "/ublox_driver/iono_params"
SHORT = {TOPIC_MEAS: "range_meas", TOPIC_EPHEM: "ephem", TOPIC_GLO_EPHEM: "glo_ephem", TOPIC_IONO: "iono_params"}

# gnss_comm numbering (DESIGN §12.3): GPS 1-32, GLO 33-59, GAL 60-97, BDS 98-160
SAT_OFFSET = {"G": 0, "R": 32, "E": 59, "C": 97}
SAT_MAX_PRN = {"G": 32, "R": 27, "E": 38, "C": 63}
BDS_B1I_GNSS_COMM_CODE = 47            # ublox_driver labels BDS B1I with this code number (observed in the M3DGR bags)

# ---------------------------------------------------------------------------------------------------------------
# field tables = the wire layout we pack; each is compared with the reference bag's own definition (check_layout)
# ---------------------------------------------------------------------------------------------------------------
OBS_FIELDS = (("time", "time"), ("uint32", "sat"), ("float64[]", "freqs"), ("float64[]", "CN0"), ("uint8[]", "LLI"),
              ("uint8[]", "code"), ("float64[]", "psr"), ("float64[]", "psr_std"), ("float64[]", "cp"),
              ("float64[]", "cp_std"), ("float64[]", "dopp"), ("float64[]", "dopp_std"), ("uint8[]", "status"))
EPHEM_FIELDS = (("uint32", "sat"), ("time", "ttr"), ("time", "toe"), ("time", "toc"), ("float64", "toe_tow"),
                ("uint32", "week"), ("uint32", "iode"), ("uint32", "iodc"), ("uint32", "health"), ("uint32", "code")) + \
    tuple(("float64", n) for n in ("ura", "A", "e", "i0", "omg", "OMG0", "M0", "delta_n", "OMG_dot", "i_dot", "cuc", "cus",
                                   "crc", "crs", "cic", "cis", "af0", "af1", "af2", "tgd0", "tgd1", "A_dot", "n_dot"))
GLO_FIELDS = (("uint32", "sat"), ("time", "ttr"), ("time", "toe"), ("int32", "freqo"), ("uint32", "iode"),
              ("uint32", "health"), ("uint32", "age")) + \
    tuple(("float64", n) for n in ("ura", "pos_x", "pos_y", "pos_z", "vel_x", "vel_y", "vel_z", "acc_x", "acc_y", "acc_z",
                                   "tau_n", "gamma", "delta_tau_n"))

_SCALAR = {"uint8": "B", "int8": "b", "uint32": "I", "int32": "i", "uint64": "Q", "int64": "q", "float32": "f", "float64": "d"}


def _flatten_expected(fields, prefix=""):
    out = []
    for t, n in fields:
        if t == "time":
            out += [("uint32", prefix + n + ".week"), ("float64", prefix + n + ".tow")]
        else:
            out.append((t, prefix + n))
    return out


def expected_layout(short):
    """Flattened (type, name) list this module writes for a gnss_comm topic (names with a `meas.` prefix inside GnssMeasMsg)."""
    if short == "range_meas":
        return _flatten_expected(OBS_FIELDS, "meas.")
    if short == "ephem":
        return _flatten_expected(EPHEM_FIELDS)
    if short == "glo_ephem":
        return _flatten_expected(GLO_FIELDS)
    if short == "iono_params":
        return [("std_msgs/Header", "header"), ("float64[]", "data")]
    raise KeyError(short)


def flatten_definition(datatype, msgdef):
    """Flattened (type, name) list of a connection's message_definition (nested messages expanded, arrays of messages
    keep the array name as prefix), for comparison with expected_layout."""
    defs = rosio.split_msgdef(msgdef, _norm(datatype))
    top = _norm(datatype)
    out = []

    def walk(tname, prefix):
        pkg = tname.split("/")[0]
        _, fields = rosio.parse_spec(defs[tname], pkg)
        for ftype, name in fields:
            bare = ftype[:ftype.find("[")] if "[" in ftype else ftype
            if bare in rosio.BUILTINS or bare == "std_msgs/Header":
                out.append((ftype, prefix + name))
            else:
                walk(bare, prefix + name + ".")

    walk(top, "")
    return out


def _norm(t):
    parts = t.split("/")
    return parts[0] + "/" + parts[2] if len(parts) == 3 and parts[1] == "msg" else t


class ReferenceDefs(object):
    """Connection headers of the gnss_comm topics of a reference bag: {topic: dict(msgtype, md5, msgdef, callerid, latching)}."""

    def __init__(self, conns):
        self.conns = conns

    def get(self, topic):
        return self.conns[topic]


def load_reference_defs(bag_path):
    """Read the gnss_comm connection headers from a reference bag (an M3DGR bag). Verifies that each md5 matches its
    definition and that the layout equals the one this module packs; raises ValueError otherwise."""
    from rosbags.rosbag1 import Reader
    found = {}
    with Reader(bag_path) as r:
        for c in r.connections:
            if c.topic in SHORT and c.topic not in found:
                ext = c.ext
                found[c.topic] = {"msgtype": c.msgtype, "md5": c.digest, "msgdef": getattr(c.msgdef, "data", c.msgdef),
                                  "callerid": getattr(ext, "callerid", None), "latching": getattr(ext, "latching", None)}
    missing = [t for t in SHORT if t not in found]
    if missing:
        raise ValueError("reference bag %s lacks the gnss_comm topics %s" % (bag_path, missing))
    refs = ReferenceDefs(found)
    check_definitions(refs)
    return refs


def check_definitions(refs):
    for topic, c in refs.conns.items():
        short = SHORT[topic]
        md5 = rosio.ros_md5(_norm(c["msgtype"]), c["msgdef"])
        if md5 != c["md5"]:
            raise ValueError("%s: md5 %s does not match its message_definition (%s)" % (topic, c["md5"], md5))
        got = flatten_definition(c["msgtype"], c["msgdef"])
        exp = expected_layout(short)
        if got != exp:
            diff = [(i, a, b) for i, (a, b) in enumerate(zip(got, exp)) if a != b][:4]
            raise ValueError("%s: definition layout differs from the packed layout (%d vs %d entries; first differences %s)"
                             % (topic, len(got), len(exp), diff))


# ---------------------------------------------------------------------------------------------------------------
# packing
# ---------------------------------------------------------------------------------------------------------------
def _pack_value(t, v, out):
    if t == "time":
        out.append(struct.pack("<Id", int(v[0]), float(v[1])))
    elif t in _SCALAR:
        out.append(struct.pack("<" + _SCALAR[t], v))
    elif t.endswith("[]"):
        e = t[:-2]
        n = len(v)
        if e == "uint8":
            out.append(struct.pack("<I", n) + bytes(bytearray(int(x) & 0xFF for x in v)))
        else:
            out.append(struct.pack("<I%d%s" % (n, _SCALAR[e]), n, *v))
    else:
        raise KeyError(t)


def pack_fields(fields, values):
    out = []
    for t, n in fields:
        _pack_value(t, values[n], out)
    return b"".join(out)


def pack_meas(obs_list):
    """obs_list: [dict per satellite with the OBS_FIELDS names] -> GnssMeasMsg bytes."""
    parts = [struct.pack("<I", len(obs_list))]
    for o in obs_list:
        parts.append(pack_fields(OBS_FIELDS, o))
    return b"".join(parts)


def pack_ephem(e):
    return pack_fields(EPHEM_FIELDS, e)


def pack_glo_ephem(g):
    return pack_fields(GLO_FIELDS, g)


def pack_iono(values, stamp_ns, seq=0, frame_id=""):
    sec, nsec = divmod(int(stamp_ns), 10 ** 9)
    fid = frame_id.encode()
    return struct.pack("<III", seq, sec, nsec) + struct.pack("<I", len(fid)) + fid + struct.pack("<I8d", 8, *values)


# ---------------------------------------------------------------------------------------------------------------
# observations
# ---------------------------------------------------------------------------------------------------------------
def gnss_comm_sat(sysl, prn):
    """gnss_comm satellite number, or None when the system/prn has none (QZSS, SBAS, IRNSS, prn out of range)."""
    off = SAT_OFFSET.get(sysl)
    if off is None or not 1 <= prn <= SAT_MAX_PRN[sysl]:
        return None
    return off + prn


def chosen_code_number(sysl, sig):
    if sysl == "C" and sig.band == "2":
        return BDS_B1I_GNSS_COMM_CODE
    return rinex.CODE_NUMBER.get(sig.code, 0)


def sat_signals(sysl, prn, vals, selection, stats=None):
    """Accepted signals of one satellite record: slot signals filtered by the signal table, one per signal code,
    primary (L1-class) first. Returns [(Signal, status, lli_out)]."""
    out = []
    seen = set()
    for s in selection.signals(sysl, prn, vals):
        code, verdict, _ = signal_from_frequency(sysl, s.freq_hz)
        if verdict != "Accepted":
            if stats is not None:
                key = "rejected_" + verdict
                stats[key] = stats.get(key, 0) + 1
            continue
        if code in seen:
            continue
        seen.add(code)
        st = 0
        if s.psr != 0.0:
            st |= 1
        lli = 0
        if s.cp != 0.0:
            st |= 2
            half_flag = bool(s.lli & 2)
            if not half_flag:
                st |= 4
            lli = (s.lli & 1) | (2 if half_flag else 0)
        out.append((s, code, st, lli))
    primary = {"G": "L1C", "R": "L1C", "E": "L1X", "C": "L2I"}.get(sysl)
    out.sort(key=lambda x: 0 if x[1] == primary else 1)          # stable: other slots keep their order
    return out


def epoch_to_obs(epoch, selection, stats=None):
    """rinex.Epoch -> [OBS dict] (one per satellite that has at least one accepted signal)."""
    obs = []
    for sysl, prn, vals in epoch.sats:
        sat = gnss_comm_sat(sysl, prn)
        if sat is None:
            if stats is not None:
                stats["skipped_" + sysl] = stats.get("skipped_" + sysl, 0) + 1
            continue
        sigs = sat_signals(sysl, prn, vals, selection, stats)
        if not sigs:
            continue
        obs.append({
            "time": (epoch.week, epoch.tow), "sat": sat,
            "freqs": [s.freq_hz for s, _, _, _ in sigs],
            "CN0": [s.cn0 for s, _, _, _ in sigs],
            "LLI": [l for _, _, _, l in sigs],
            "code": [chosen_code_number(sysl, s) for s, _, _, _ in sigs],
            "psr": [s.psr for s, _, _, _ in sigs], "psr_std": [0.0] * len(sigs),
            "cp": [s.cp for s, _, _, _ in sigs], "cp_std": [0.0] * len(sigs),
            "dopp": [s.dopp for s, _, _, _ in sigs], "dopp_std": [0.0] * len(sigs),
            "status": [st for _, _, st, _ in sigs],
        })
    return obs


# ---------------------------------------------------------------------------------------------------------------
# navigation
# ---------------------------------------------------------------------------------------------------------------
def _wt(t_gpst):
    w, tow = gnsstime.week_tow(t_gpst)
    return int(w), float(tow)


def kepler_fields(r):
    """fl2g.rinexnav Kepler record (G/E/C) -> GnssEphemMsg field dict, or None for systems without a gnss_comm number."""
    sat = gnss_comm_sat(r["sys"], r["prn"])
    if sat is None or r["sys"] not in ("G", "E", "C"):
        return None
    toe, toc, ttr = _wt(r["toe"]), _wt(r["toc"]), _wt(r["ttr"])
    iodc = r["aodc"] if r["sys"] == "C" else r["iodc"]
    return {
        "sat": sat, "ttr": ttr, "toe": toe, "toc": toc, "toe_tow": toe[1], "week": toe[0],
        "iode": int(r["iode"]) & 0xFFFFFFFF, "iodc": int(iodc) & 0xFFFFFFFF, "health": int(r["svh"]) & 0xFFFFFFFF,
        "code": int(r["code"]) & 0xFFFFFFFF if r["sys"] != "C" else 0,
        "ura": float(r["sva_m"]), "A": r["A"], "e": r["e"], "i0": r["i0"], "omg": r["omg"], "OMG0": r["OMG0"],
        "M0": r["M0"], "delta_n": r["deln"], "OMG_dot": r["OMGd"], "i_dot": r["idot"], "cuc": r["cuc"], "cus": r["cus"],
        "crc": r["crc"], "crs": r["crs"], "cic": r["cic"], "cis": r["cis"], "af0": r["af0"], "af1": r["af1"],
        "af2": r["af2"], "tgd0": r["tgd0"], "tgd1": r["tgd1"], "A_dot": 0.0, "n_dot": 0.0,
    }


def glo_fields(r, leap_s=None):
    """fl2g.rinexnav GLONASS record -> GnssGloEphemMsg field dict."""
    sat = gnss_comm_sat("R", r["prn"])
    if sat is None:
        return None
    toe_gpst = r["toe"]
    leap = leap_s if leap_s is not None else gnsstime.leap_at_gpst(toe_gpst)
    toe_utc = toe_gpst - leap
    # message frame time: only its time of day is used (the day-of-week part of the brdm files does not follow the
    # RINEX definition: Sunday records carry day 4); the instant with that time of day nearest to toe
    tod = float(r["tof_raw"]) % 86400.0
    day0 = math.floor(toe_utc / 86400.0) * 86400.0
    tof_utc = min((day0 + k * 86400.0 + tod for k in (-1, 0, 1)), key=lambda x: abs(x - toe_utc))
    tb = int(round(((toe_utc + 3 * 3600.0) % 86400.0) / 900.0))
    return {
        "sat": sat, "ttr": _wt(tof_utc + leap), "toe": _wt(toe_gpst), "freqo": int(r["frq"]), "iode": tb,
        "health": int(r["svh"]) & 0xFFFFFFFF, "age": int(r["age"]) & 0xFFFFFFFF, "ura": 0.0,
        "pos_x": r["pos"][0], "pos_y": r["pos"][1], "pos_z": r["pos"][2],
        "vel_x": r["vel"][0], "vel_y": r["vel"][1], "vel_z": r["vel"][2],
        "acc_x": r["acc"][0], "acc_y": r["acc"][1], "acc_z": r["acc"][2],
        "tau_n": r["taun"], "gamma": r["gamn"], "delta_tau_n": 0.0,
    }


def nav_messages(records, t_lo_gpst, t_hi_gpst, margin_s=7200.0):
    """Ephemeris messages of the records whose toe lies in [t_lo - margin, t_hi + margin] (GPST seconds), de-duplicated
    on (satellite, toe, iode). Returns [(t_tx_gpst, topic, field dict)] sorted by transmission time."""
    out = []
    seen = set()
    for r in records:
        if not (t_lo_gpst - margin_s <= r["toe"] <= t_hi_gpst + margin_s):
            continue
        key = (r["sys"], r["prn"], round(r["toe"], 3), r.get("iode"))
        if key in seen:
            continue
        seen.add(key)
        if r["sys"] == "R":
            f = glo_fields(r)
            if f:
                out.append((gnsstime.gpst(*f["ttr"]), TOPIC_GLO_EPHEM, f))
        else:
            f = kepler_fields(r)
            if f:
                out.append((gnsstime.gpst(*f["ttr"]), TOPIC_EPHEM, f))
    out.sort(key=lambda x: (x[0], x[2]["sat"]))
    return out


def iono_values(header):
    a, b = header.get("iono_gps_alpha"), header.get("iono_gps_beta")
    if not a or not b:
        return None
    return [float(x or 0.0) for x in list(a) + list(b)]
