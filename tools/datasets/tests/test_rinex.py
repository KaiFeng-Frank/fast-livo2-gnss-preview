# SPDX-License-Identifier: BSD-3-Clause
"""dscommon.rinex: header, epochs, time systems and the RTKLIB-EX slot selection, on a synthetic RINEX 3.03 file."""
import io
import unittest

import _path  # noqa: F401
from dscommon import rinex

HDR = """\
     3.03           OBSERVATION DATA    M: Mixed            RINEX VERSION / TYPE
RTKCONV 2.4.3 b34                       20240707 092446 UTC PGM / RUN BY / DATE
 -2267243.7013  5009554.6034  3221114.7928                  APPROX POSITION XYZ
G   12 C1C L1C D1C S1C C1W L1W D1W S1W C2W L2W D2W S2W      SYS / # / OBS TYPES
R    8 C1C L1C D1C S1C C2C L2C D2C S2C                      SYS / # / OBS TYPES
E    8 C1C L1C D1C S1C C7Q L7Q D7Q S7Q                      SYS / # / OBS TYPES
C   12 C2I L2I D2I S2I C7I L7I D7I S7I C1P L1P D1P S1P      SYS / # / OBS TYPES
S    4 C1C L1C D1C S1C                                      SYS / # / OBS TYPES
  2024    07    07    07    54   40.6000000     GPS         TIME OF FIRST OBS
  7 R07  5 R08  6                                           GLONASS SLOT / FRQ #
                                                            END OF HEADER
"""


def fld(v, lli=" ", ssi=" "):
    if v is None:
        return " " * 16
    return "%14.3f%s%s" % (v, lli, ssi)


def sat(sysl, prn, vals):
    return "%s%02d" % (sysl, prn) + "".join(vals) + "\n"


BODY = (
    "> 2024 07 07 07 54 40.6000000  0  6\n" +
    # GPS: C1C L1C D1C S1C | C1W L1W D1W S1W | C2W L2W D2W S2W
    sat("G", 5, [fld(22030924.229), fld(115773294.540, "1"), fld(-1291.267), fld(33.0),
                 fld(22030923.979), fld(115773294.540, "1"), fld(-1291.267), fld(33.0),
                 fld(22030927.553), fld(90212988.205, "2"), fld(-1006.167), fld(9.0)]) +
    # GLO R07 (fcn 5): C1C L1C D1C S1C | C2C L2C D2C S2C
    sat("R", 7, [fld(20634818.680), fld(110459767.203), fld(-1617.399), fld(46.0),
                 fld(20634826.275), fld(85913352.588), fld(-1257.573), fld(35.0)]) +
    # GAL E04: only E1
    sat("E", 4, [fld(24556523.215), fld(129045399.730), fld(-1559.848), fld(37.0)]) +
    # BDS C29: B1I and B2I, and a B1C phase-only block
    sat("C", 29, [fld(22354302.708), fld(116404718.468), fld(-449.722), fld(38.0),
                  fld(22354310.0), fld(86000000.0), fld(-300.0), fld(30.0),
                  fld(22354311.0), fld(116404700.0), fld(-449.0), fld(36.0)]) +
    sat("S", 28, [fld(37680430.420), fld(198011995.139), fld(234.731), fld(36.0)]) +
    sat("J", 3, [fld(1.0)]) +
    "> 2024 07 07 07 54 40.7000000  4  1\n"
    "COMMENT after an event flag\n"
    "> 2024 07 07 07 54 40.8000000  0  1\n" +
    sat("G", 5, [fld(22030973.762), fld(115773552.690), fld(-1290.658), fld(33.0)])
)


class TestRinex(unittest.TestCase):
    def setUp(self):
        self.f = io.StringIO(HDR + BODY)
        self.h = rinex.read_header(self.f)

    def test_header(self):
        h = self.h
        self.assertEqual(h.version, 3.03)
        self.assertEqual(h.obs_types["G"][:4], ["C1C", "L1C", "D1C", "S1C"])
        self.assertEqual(len(h.obs_types["C"]), 12)
        self.assertEqual(h.first_obs, (2024, 7, 7, 7, 54, 40.6))
        self.assertEqual(h.time_system, "GPS")
        self.assertEqual(h.glo_fcn, {7: 5, 8: 6})

    def test_time_conversion(self):
        # Sunday 2024-07-07: GPS week 2322, tow = time of day
        self.assertEqual(rinex.gpst_from_calendar(2024, 7, 7, 7, 54, 40.6), (2322, 28480.6))
        self.assertEqual(rinex.gpst_from_calendar(2024, 7, 6, 23, 59, 59.0)[0], 2321)
        # BDT is 14 s behind GPST, GLONASS/UTC 18 s behind
        self.assertEqual(rinex.gpst_from_calendar(2024, 7, 7, 0, 0, 0.0, "BDT"), (2322, 14.0))
        self.assertEqual(rinex.gpst_from_calendar(2024, 7, 7, 0, 0, 0.0, "GLO"), (2322, 18.0))
        # crossing the week boundary through the shift
        self.assertEqual(rinex.gpst_from_calendar(2024, 7, 6, 23, 59, 50.0, "UTC"), (2322, 8.0))
        with self.assertRaises(rinex.RinexError):
            rinex.gpst_from_calendar(2024, 7, 7, 0, 0, 0.0, "XYZ")

    def test_epochs(self):
        sel = rinex.Selection(self.h)
        eps = list(rinex.iter_epochs(self.f, self.h, sel.wanted()))
        self.assertEqual(len(eps), 2)                       # the event epoch (flag 4) and its record line are skipped
        e0 = eps[0]
        self.assertEqual((e0.week, e0.flag), (2322, 0))
        self.assertAlmostEqual(e0.tow, 28480.6, 6)
        self.assertEqual(eps[1].tow, 28480.8)
        systems = [s[0] for s in e0.sats]
        self.assertEqual(systems, ["G", "R", "E", "C", "S"])  # J has no OBS TYPES entry in the header: dropped
        g5 = e0.sats[0][2]
        self.assertEqual(g5["C1C"], (22030924.229, 0, 0))
        self.assertEqual(g5["L1C"], (115773294.540, 1, 0))   # LLI bit 0
        self.assertEqual(g5["L2W"][1], 2)                    # LLI bit 1 (half cycle)

    def test_slot_selection_follows_rtklib_priority(self):
        sel = rinex.Selection(self.h)
        g = sel.slots["G"]
        self.assertEqual(g[0], {"C": "C1C", "L": "L1C", "D": "D1C", "S": "S1C"})      # C beats W in band 1
        self.assertEqual(g[1]["C"], "C2W")
        self.assertEqual(sel.slots["E"][1]["C"], "C7Q")
        self.assertEqual(sel.slots["C"][0]["C"], "C2I")                                 # BDS band 2 = B1I = slot 0
        self.assertEqual(sel.slots["C"][1]["C"], "C7I")
        self.assertNotIn(4, sel.slots["C"])                                             # B1C (band 1) is slot 4: beyond NFREQ
        self.assertEqual(set(sel.wanted()["C"]), {"C2I", "L2I", "D2I", "S2I", "C7I", "L7I", "D7I", "S7I"})

    def test_signals_frequencies(self):
        sel = rinex.Selection(self.h)
        eps = list(rinex.iter_epochs(self.f, self.h, sel.wanted()))
        by = {(s, p): v for s, p, v in eps[0].sats}
        g = sel.signals("G", 5, by[("G", 5)])
        self.assertEqual([x.freq_hz for x in g], [1575.42e6, 1227.60e6])
        self.assertEqual((g[0].psr, g[0].cp, g[0].dopp, g[0].cn0, g[0].lli), (22030924.229, 115773294.540, -1291.267, 33.0, 1))
        r = sel.signals("R", 7, by[("R", 7)])
        self.assertEqual([x.freq_hz for x in r], [1602e6 + 5 * 0.5625e6, 1246e6 + 5 * 0.4375e6])
        c = sel.signals("C", 29, by[("C", 29)])
        self.assertEqual([x.freq_hz for x in c], [1561.098e6, 1207.14e6])
        self.assertEqual(sel.signals("E", 4, by[("E", 4)])[0].freq_hz, 1575.42e6)

    def test_missing_glonasss_channel_drops_the_slot(self):
        sel = rinex.Selection(self.h)
        eps = list(rinex.iter_epochs(self.f, self.h, sel.wanted()))
        by = {(s, p): v for s, p, v in eps[0].sats}
        self.assertEqual(sel.signals("R", 9, by[("R", 7)]), [])                         # no channel known for R09
        self.assertEqual(len(sel.signals("R", 9, by[("R", 7)], glo_fcn={9: -2})), 2)

    def test_rejects_rinex2_and_nonobs(self):
        bad = HDR.replace("     3.03", "     2.11")
        with self.assertRaises(rinex.RinexError):
            rinex.read_header(io.StringIO(bad))
        nav = HDR.replace("OBSERVATION DATA    M: Mixed ", "N: GNSS NAV DATA    M: Mixed ")
        with self.assertRaises(rinex.RinexError):
            rinex.read_header(io.StringIO(nav))


if __name__ == "__main__":
    unittest.main()
