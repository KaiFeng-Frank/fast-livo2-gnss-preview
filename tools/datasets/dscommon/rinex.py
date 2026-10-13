# SPDX-License-Identifier: BSD-3-Clause
"""RINEX 3.0x observation reader (streaming) and the per-signal slot selection of RTKLIB-EX.

Why our own reader: the converters write gnss_comm messages (rinex2gnsscomm.py), so the signal choice must be the one
the repository's RINEX path (RTKLIB readrnxt in gnss_io/src/rinex_source.cpp) makes, otherwise the bag path and the
RINEX path would carry different signals. The rules are those of the vendored RTKLIB-EX 2.5.1:
  * every system has frequency slots (rtkcmn.c code2freq_*: GPS L1/L2/L5, GLO G1/G2/G3, GAL E1/E5b/E5a/E6/E5ab,
    BDS B1I/B2/B2a/B3/B1C/B2ab) and the RINEX path keeps the first NFREQ = 3 slots;
  * within a slot the observation code is chosen per observable (C, L, D, S) by the priority string of rtkcmn.c
    codepris (first character = highest priority), among the codes the header lists for the system.
Time: epochs come back as (GPS week, time of week [s]) in GPST whatever the file's time system (GPS, GAL, BDT, GLO/UTC).
"""
import datetime
import re
from collections import namedtuple

from fl2g import gnsstime
from fl2g.sats import FREQ_L1, rtk_sat2freq

NFREQ = 3                                   # rtklib NFREQ of the vendored build (DESIGN §12.1)

# rtkcmn.c codepris[sys][freq index]; first character wins.
CODEPRI = {
    "G": ("CPYWMNSLX", "CPYWMNDLSX", "IQX"),
    "R": ("CPABX", "CPABX", "IQX"),
    "E": ("CABXZ", "XIQ", "XIQ", "ABCXZ", "IQX"),
    "J": ("CLSXZBE", "LSX", "IQXDPZ", "LSXEZ"),
    "S": ("C", "IQX"),
    "C": ("IQX", "IQXDPZ", "DPX", "IQXDPZA", "DPXSLZAN", "DPX"),
}
# rtkcmn.c code2freq_*: band digit of the RINEX observation code -> frequency index (slot)
SLOT_OF_BAND = {
    "G": {"1": 0, "2": 1, "5": 2},
    "R": {"1": 0, "2": 1, "3": 2},
    "E": {"1": 0, "7": 1, "5": 2, "6": 3, "8": 4},
    "J": {"1": 0, "2": 1, "5": 2, "6": 3},
    "S": {"1": 0, "5": 1},
    "C": {"2": 0, "7": 1, "5": 2, "6": 3, "1": 4, "8": 5},
}

# rtkcmn.c obscodes (RTKLIB-EX 2.5.1): observation code string -> CODE_* number (used for the gnss_comm `code` field)
OBSCODES = ("", "1C", "1P", "1W", "1Y", "1M", "1N", "1S", "1L", "1E", "1A", "1B", "1X", "1Z", "2C", "2D", "2S", "2L",
            "2X", "2P", "2W", "2Y", "2M", "2N", "5I", "5Q", "5X", "7I", "7Q", "7X", "6A", "6B", "6C", "6X", "6Z", "6S",
            "6L", "8I", "8Q", "8X", "2I", "2Q", "6I", "6Q", "3I", "3Q", "3X", "1I", "1Q", "5A", "5B", "5C", "9A", "9B",
            "9C", "9X", "1D", "5D", "5P", "5Z", "6E", "7D", "7P", "7Z", "8D", "8P", "4A", "4B", "4X", "6D", "6P")
CODE_NUMBER = {c: i for i, c in enumerate(OBSCODES) if c}

GPS_EPOCH = datetime.date(1980, 1, 6)

Epoch = namedtuple("Epoch", "week tow flag sats")          # sats: [(sys letter, prn, {obs type: (value, lli, ssi)})]
Signal = namedtuple("Signal", "slot band code psr cp lli dopp cn0 freq_hz")


class RinexError(ValueError):
    pass


class ObsHeader(object):
    """What the converters need of a RINEX 3 OBS header."""

    def __init__(self):
        self.version = None
        self.sat_system = ""
        self.obs_types = {}            # system letter -> [observation type strings]
        self.glo_fcn = {}              # GLONASS prn -> frequency channel number
        self.first_obs = None          # (y, m, d, hh, mm, sec) as written
        self.time_system = "GPS"
        self.interval = None
        self.leap_s = None
        self.receiver = ""
        self.antenna = ""
        self.approx_xyz = None
        self.sig_strength_unit = ""
        self.program = ""
        self.comments = []

    def systems(self):
        return sorted(self.obs_types)


def gpst_from_calendar(y, m, d, hh, mm, ss, time_system="GPS", leap_s=None):
    """(week, tow) of calendar fields given in `time_system` (GPS | GAL | BDT | GLO | UTC)."""
    days = (datetime.date(int(y), int(m), int(d)) - GPS_EPOCH).days
    week, dow = divmod(days, 7)
    tow = dow * 86400.0 + hh * 3600.0 + mm * 60.0 + ss
    ts = time_system.upper()
    if ts in ("GPS", "GAL", ""):
        shift = 0.0
    elif ts in ("BDT", "BDS"):
        shift = -gnsstime.BDT_MINUS_GPST_S
    elif ts in ("GLO", "UTC"):
        shift = float(leap_s) if leap_s else float(gnsstime.leap_at_utc(gnsstime.calendar_to_seconds(y, m, d, hh, mm, ss)))
    else:
        raise RinexError("unsupported RINEX time system %r" % time_system)
    tow += shift
    while tow >= 604800.0:
        tow -= 604800.0
        week += 1
    while tow < 0.0:
        tow += 604800.0
        week -= 1
    return int(week), tow


def _label(line):
    return line[60:].strip()


def read_header(f):
    """Parse header lines from the file object `f` up to END OF HEADER."""
    h = ObsHeader()
    pending = None            # (system, remaining count) of a multi-line SYS / # / OBS TYPES block
    glo_pairs = []
    while True:
        line = f.readline()
        if not line:
            raise RinexError("end of file inside the RINEX header")
        line = line.rstrip("\r\n")
        lab = _label(line)
        body = line[:60]
        if lab == "RINEX VERSION / TYPE":
            h.version = float(body[:9])
            if body[20:21] not in ("O",):
                raise RinexError("not an observation file: %r" % body[20:40])
            h.sat_system = body[40:41]
        elif lab == "PGM / RUN BY / DATE":
            h.program = body[:20].strip()
        elif lab == "SYS / # / OBS TYPES":
            toks = body.split()
            if body[0] != " ":
                sysl, cnt, types = toks[0], int(toks[1]), toks[2:]
                h.obs_types[sysl] = list(types)
                pending = (sysl, cnt)
            else:
                if pending is None:
                    raise RinexError("continuation of SYS / # / OBS TYPES without a first line")
                h.obs_types[pending[0]].extend(toks)
        elif lab == "TIME OF FIRST OBS":
            toks = body.split()
            h.first_obs = tuple([int(t) for t in toks[:5]] + [float(toks[5])])
            h.time_system = toks[6] if len(toks) > 6 else "GPS"
        elif lab == "INTERVAL":
            try:
                h.interval = float(body[:10])
            except ValueError:
                pass
        elif lab == "LEAP SECONDS":
            try:
                h.leap_s = int(body[:6])
            except ValueError:
                pass
        elif lab == "GLONASS SLOT / FRQ #":
            glo_pairs.extend(re.findall(r"R(\d\d)\s+(-?\d+)", body))
        elif lab == "REC # / TYPE / VERS":
            h.receiver = body[20:40].strip()
        elif lab == "ANT # / TYPE":
            h.antenna = body[20:40].strip()
        elif lab == "APPROX POSITION XYZ":
            try:
                h.approx_xyz = tuple(float(x) for x in body.split()[:3])
            except ValueError:
                pass
        elif lab == "SIGNAL STRENGTH UNIT":
            h.sig_strength_unit = body.strip()
        elif lab == "COMMENT":
            h.comments.append(body.strip())
        elif lab == "END OF HEADER":
            break
    for prn, k in glo_pairs:
        h.glo_fcn[int(prn)] = int(k)
    if h.version is None or not h.obs_types:
        raise RinexError("RINEX header without version or observation types")
    if h.version < 3.0:
        raise RinexError("RINEX %.2f: only 3.x is supported" % h.version)
    return h


def _num(field):
    s = field[:14].strip()
    if not s:
        return None
    try:
        v = float(s)
    except ValueError:
        return None
    return v


def iter_epochs(f, header, wanted=None):
    """Yield Epoch records from `f` (positioned after the header). `wanted`: {system: set of observation types to
    parse}; None parses all types the header lists. Event epochs (flag > 2) are skipped with their records."""
    types = header.obs_types
    idx = {}
    for sysl, tl in types.items():
        sel = [(k, t) for k, t in enumerate(tl) if wanted is None or t in wanted.get(sysl, ())]
        idx[sysl] = sel
    ts = header.time_system
    line = f.readline()
    while line:
        if line[0] != ">":
            line = f.readline()
            continue
        try:
            y, mo, d = int(line[2:6]), int(line[7:9]), int(line[10:12])
            hh, mi = int(line[13:15]), int(line[16:18])
            ss = float(line[18:29])
            flag = int(line[31:32])
            nsat = int(line[32:35])
        except ValueError:
            raise RinexError("bad epoch record: %r" % line[:40])
        week, tow = gpst_from_calendar(y, mo, d, hh, mi, ss, ts, header.leap_s)
        sats = []
        if flag > 2:
            for _ in range(nsat):
                f.readline()
            line = f.readline()
            continue
        for _ in range(nsat):
            sl = f.readline()
            if len(sl) < 3:
                continue
            sysl = sl[0]
            sel = idx.get(sysl)
            if sel is None:
                continue
            try:
                prn = int(sl[1:3])
            except ValueError:
                continue
            vals = {}
            n = len(sl)
            for k, t in sel:
                a = 3 + 16 * k
                if a >= n:
                    continue
                fld = sl[a:a + 16]
                v = _num(fld)
                if v is None:
                    continue
                lli = int(fld[14]) if len(fld) > 14 and fld[14] in "0123456789" else 0
                ssi = int(fld[15]) if len(fld) > 15 and fld[15] in "0123456789" else 0
                vals[t] = (v, lli, ssi)
            if vals:
                sats.append((sysl, prn, vals))
        yield Epoch(week, tow, flag, sats)
        line = f.readline()


class Selection(object):
    """Per system and slot the observation type of each observable, chosen like RTKLIB's set_index (per
    observable, highest codepris priority among the codes the header lists)."""

    def __init__(self, header, nslots=NFREQ):
        self.header = header
        self.nslots = nslots
        self.slots = {}            # system -> {slot: {'C': 'C1C', 'L': 'L1C', 'D': 'D1C', 'S': 'S1C'}}
        self.band = {}             # system -> {slot: band digit}
        for sysl, tl in header.obs_types.items():
            pri = CODEPRI.get(sysl)
            if pri is None:
                continue
            best = {}
            for t in tl:
                if len(t) != 3 or t[0] not in "CLDS":
                    continue
                band, attr = t[1], t[2]
                slot = SLOT_OF_BAND[sysl].get(band)
                if slot is None or slot >= nslots or slot >= len(pri):
                    continue
                p = pri[slot].find(attr)
                if p < 0:
                    continue
                key = (slot, t[0])
                if key not in best or p < best[key][0]:
                    best[key] = (p, t, band)
            per = {}
            bands = {}
            for (slot, kind), (p, t, band) in best.items():
                per.setdefault(slot, {})[kind] = t
                bands[slot] = band
            self.slots[sysl] = per
            self.band[sysl] = bands

    def wanted(self):
        """{system: set of observation types} to hand to iter_epochs."""
        out = {}
        for sysl, per in self.slots.items():
            out[sysl] = set(t for d in per.values() for t in d.values())
        return out

    def signals(self, sysl, prn, vals, glo_fcn=None):
        """Signals of one satellite record in slot order. A slot is present when it has a pseudorange, a phase or a
        Doppler; the frequency comes from the band's nominal value (GLONASS: the channel from the header or glo_fcn)."""
        out = []
        per = self.slots.get(sysl, {})
        for slot in sorted(per):
            d = per[slot]
            c = vals.get(d.get("C")) if "C" in d else None
            l = vals.get(d.get("L")) if "L" in d else None
            dp = vals.get(d.get("D")) if "D" in d else None
            s = vals.get(d.get("S")) if "S" in d else None
            if not any(x is not None and x[0] != 0.0 for x in (c, l, dp)):
                continue
            band = self.band[sysl][slot]
            fcn = None
            if sysl == "R":
                fcn = (glo_fcn or {}).get(prn, self.header.glo_fcn.get(prn))
            attr = (d.get("C") or d.get("L") or d.get("D") or d.get("S"))[2]
            freq = rtk_sat2freq(sysl, "L%s%s" % (band, attr), fcn)
            if not freq:
                continue
            out.append(Signal(slot, band, (d.get("C") or "")[1:], c[0] if c and c[0] != 0.0 else 0.0,
                              l[0] if l and l[0] != 0.0 else 0.0, l[1] if l else 0,
                              dp[0] if dp and dp[0] != 0.0 else 0.0, s[0] if s else 0.0, freq))
        return out


def open_obs(path, wanted=None, nslots=NFREQ):
    """(file object, header, Selection). The caller closes the file; iterate with iter_epochs(f, header, sel.wanted())."""
    f = open(path, "r", errors="replace")
    try:
        header = read_header(f)
    except Exception:
        f.close()
        raise
    return f, header, Selection(header, nslots)
