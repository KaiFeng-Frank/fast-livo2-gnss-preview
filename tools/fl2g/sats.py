# SPDX-License-Identifier: BSD-3-Clause
"""Satellite numbering and the freqs -> signal table (DESIGN §12.3).

gnss_comm numbering (functional fact of the data): GPS 1-32, GLO 33-59 (prn = sat-32), GAL 60-97 (prn = sat-59),
BDS 98-160 (prn = sat-97). The frequency table mirrors gnss/include/livo_gnss/signal_table.hpp (the single C++
copy); tools/tests/test_audit.py (SignalTableMirror) parses the header and pins the two together value by value.
"""
import math

CLIGHT = 299792458.0

SYS_NAMES = {"G": "GPS", "R": "GLO", "E": "GAL", "C": "BDS", "J": "QZS", "S": "SBS", "I": "IRN"}

FREQ_L1 = 1575.42e6
FREQ_L2 = 1227.60e6
FREQ_L5 = 1176.45e6
FREQ_E5B = 1207.14e6
FREQ_B1I = 1561.098e6
FREQ_G1 = 1602.0e6
DFRQ_G1 = 0.5625e6
FREQ_G2 = 1246.0e6
DFRQ_G2 = 0.4375e6
FREQ_TOL_HZ = 1.0e3        # signal_table.hpp kFreqTolHz


def from_gnss_comm(sat):
    """gnss_comm sat number -> (system letter, prn); (None, 0) outside the ranges."""
    sat = int(sat)
    if 1 <= sat <= 32:
        return "G", sat
    if 33 <= sat <= 59:
        return "R", sat - 32
    if 60 <= sat <= 97:
        return "E", sat - 59
    if 98 <= sat <= 160:
        return "C", sat - 97
    return None, 0


def satid(sys, prn):
    return "%s%02d" % (sys, int(prn))


def parse_satid(text):
    text = text.strip()
    return text[0], int(text[1:])


def is_bds_geo(prn):
    """BDS GEO by PRN, as RTKLIB ephemeris.c:265 (prn <= 5 || prn >= 59)."""
    return prn <= 5 or prn >= 59


def signal_from_frequency(sys, freq_hz):
    """(code name, verdict, glo_k) exactly as livo_gnss::signalFromFrequency (signal_table.hpp).

    Verdicts: Accepted, RejectedBdsB1C, RejectedUnknownFreq, RejectedGloChannel, RejectedSystem."""
    near = lambda f0: abs(freq_hz - f0) < FREQ_TOL_HZ
    if sys in ("G", "J"):
        if near(FREQ_L1):
            return "L1C", "Accepted", 0
        if near(FREQ_L2):
            return "L2X", "Accepted", 0
        if near(FREQ_L5):
            return "L5X", "Accepted", 0
        return "Unknown", "RejectedUnknownFreq", 0
    if sys == "E":
        if near(FREQ_L1):
            return "L1X", "Accepted", 0
        if near(FREQ_E5B):
            return "L7X", "Accepted", 0
        if near(FREQ_L5):
            return "L5Q", "Accepted", 0
        return "Unknown", "RejectedUnknownFreq", 0
    if sys == "C":
        if near(FREQ_B1I):
            return "L2I", "Accepted", 0
        if near(FREQ_E5B):
            return "L7I", "Accepted", 0
        if near(FREQ_L1):
            return "Unknown", "RejectedBdsB1C", 0
        return "Unknown", "RejectedUnknownFreq", 0
    if sys == "R":
        k1 = (freq_hz - FREQ_G1) / DFRQ_G1
        k2 = (freq_hz - FREQ_G2) / DFRQ_G2
        r1, r2 = round(k1), round(k2)
        if abs(k1 - r1) * DFRQ_G1 < FREQ_TOL_HZ and -7 <= r1 <= 6:
            return "L1C", "Accepted", int(r1)
        if abs(k2 - r2) * DFRQ_G2 < FREQ_TOL_HZ and -7 <= r2 <= 6:
            return "L2C", "Accepted", int(r2)
        return "Unknown", "RejectedGloChannel", 0
    return "Unknown", "RejectedSystem", 0


# RTKLIB-EX 2.5.1 code2freq_GPS/GLO/GAL/QZS/BDS (rtkcmn.c:611-700): the carrier depends only on the band digit of the
# observation code (code2obs: CODE_L1C -> "1C" -> '1'), which is the second character of a livo_gnss SignalCode name.
# GLONASS G1/G2 add the channel (sat2freq takes it from the satellite's first ephemeris in nav). Pinned against the
# vendored C source by tools/tests/test_audit.py (Sat2FreqMirror).
FREQ_L6 = 1278.75e6
FREQ_E5AB = 1191.795e6
FREQ_B3 = 1268.52e6
FREQ_G3 = 1202.025e6
FREQ_G1A = 1600.995e6
FREQ_G2A = 1248.06e6
RTK_BAND_FREQ = {
    "G": {"1": FREQ_L1, "2": FREQ_L2, "5": FREQ_L5},
    "J": {"1": FREQ_L1, "2": FREQ_L2, "5": FREQ_L5, "6": FREQ_L6},
    "E": {"1": FREQ_L1, "7": FREQ_E5B, "5": FREQ_L5, "6": FREQ_L6, "8": FREQ_E5AB},
    "C": {"2": FREQ_B1I, "7": FREQ_E5B, "5": FREQ_L5, "6": FREQ_B3, "1": FREQ_L1, "8": FREQ_E5AB},
    "R": {"3": FREQ_G3, "4": FREQ_G1A, "6": FREQ_G2A},
}


def rtk_sat2freq(sys, code_name, fcn=None):
    """RTKLIB-EX sat2freq(sat, code, nav) for a livo_gnss SignalCode name ("L1C", "L2I", ...) [Hz]; 0.0 is RTKLIB's
    error value (unknown band, or GLONASS G1/G2 without a channel in [-7, 6], i.e. no ephemeris of the satellite)."""
    if not code_name or len(code_name) < 3 or code_name[0] != "L":
        return 0.0
    band = code_name[1]
    if sys == "R" and band in ("1", "2"):
        if fcn is None or not -7 <= int(fcn) <= 6:
            return 0.0
        return FREQ_G1 + DFRQ_G1 * int(fcn) if band == "1" else FREQ_G2 + DFRQ_G2 * int(fcn)
    return RTK_BAND_FREQ.get(sys, {}).get(band, 0.0)


PRIMARY_CODES = {"G": "L1C", "J": "L1C", "R": "L1C", "E": "L1X", "C": "L2I"}


def is_primary(sys, code):
    """The L1-class signal of a system: the one used for psr/Doppler/TDCP (§12.3 "每星只取主信号")."""
    return PRIMARY_CODES.get(sys) == code


def wavelength(freq_hz):
    return CLIGHT / freq_hz if freq_hz > 0 else float("nan")


def glo_k_from_freq(freq_hz):
    """GLONASS channel from a G1 frequency, or None when off-grid."""
    k = (freq_hz - FREQ_G1) / DFRQ_G1
    r = round(k)
    if abs(k - r) * DFRQ_G1 < FREQ_TOL_HZ and -7 <= r <= 6:
        return int(r)
    return None


def finite(x):
    return x is not None and not (isinstance(x, float) and (math.isnan(x) or math.isinf(x)))
