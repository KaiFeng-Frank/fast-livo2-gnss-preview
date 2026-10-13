# SPDX-License-Identifier: BSD-3-Clause
"""dscommon.gnsscomm: wire layout against the known md5 sums, message packing, RINEX -> gnss_comm field mapping, and
the round trip through tools/gnss_bag_export.py (the inverse conversion).

The md5 sums of the four gnss_comm types are functional identifiers (they depend only on field types and names); the
definitions used here are written from this module's own field tables without any comment text (docs/LICENSING.md).
"""
import io
import os
import unittest

import _path  # noqa: F401
from dscommon import gnsscomm, rinex
from fl2g import rosio
import test_rinex as tr

MD5 = {"range_meas": "0eafeb530a3e6637722c4e296734c119", "ephem": "b4346f99e431c870e392a438684c0fe8",
       "glo_ephem": "6893dee0d8b1c9325a6693f3488e0144", "iono_params": "fb60495edd59d3fcf90e173153ae8a9a"}
SEP = "=" * 80


def _lines(fields):
    out = []
    for t, n in fields:
        out.append("%s %s" % ("GnssTimeMsg" if t == "time" else t, n))
    return "\n".join(out)


def synth_defs():
    """{topic: connection dict} with definitions generated from the field tables."""
    time_def = "uint32 week\nfloat64 tow"
    obs = _lines(gnsscomm.OBS_FIELDS)
    meas = "GnssObsMsg[] meas\n%s\nMSG: gnss_comm/GnssObsMsg\n%s\n%s\nMSG: gnss_comm/GnssTimeMsg\n%s\n" % (SEP, obs, SEP, time_def)
    eph = "%s\n%s\nMSG: gnss_comm/GnssTimeMsg\n%s\n" % (_lines(gnsscomm.EPHEM_FIELDS), SEP, time_def)
    glo = "%s\n%s\nMSG: gnss_comm/GnssTimeMsg\n%s\n" % (_lines(gnsscomm.GLO_FIELDS), SEP, time_def)
    hdr = ("Header header\nfloat64[] data\n%s\nMSG: std_msgs/Header\nuint32 seq\ntime stamp\nstring frame_id\n" % SEP)
    types = {"range_meas": ("gnss_comm/msg/GnssMeasMsg", meas), "ephem": ("gnss_comm/msg/GnssEphemMsg", eph),
             "glo_ephem": ("gnss_comm/msg/GnssGloEphemMsg", glo), "iono_params": ("gnss_comm/msg/StampedFloat64Array", hdr)}
    topics = {"range_meas": gnsscomm.TOPIC_MEAS, "ephem": gnsscomm.TOPIC_EPHEM, "glo_ephem": gnsscomm.TOPIC_GLO_EPHEM,
              "iono_params": gnsscomm.TOPIC_IONO}
    return {topics[k]: {"msgtype": t, "md5": MD5[k], "msgdef": d, "callerid": None, "latching": None} for k, (t, d) in types.items()}


class TestLayout(unittest.TestCase):
    def test_md5_of_generated_definitions(self):
        for topic, c in synth_defs().items():
            short = gnsscomm.SHORT[topic]
            self.assertEqual(rosio.ros_md5(gnsscomm._norm(c["msgtype"]), c["msgdef"]), MD5[short], short)

    def test_check_definitions_accepts_and_rejects(self):
        refs = gnsscomm.ReferenceDefs(synth_defs())
        gnsscomm.check_definitions(refs)
        bad = synth_defs()
        c = bad[gnsscomm.TOPIC_EPHEM]
        c["msgdef"] = c["msgdef"].replace("float64 A_dot\n", "")           # layout (and md5) change
        with self.assertRaises(ValueError):
            gnsscomm.check_definitions(gnsscomm.ReferenceDefs(bad))
        bad = synth_defs()
        bad[gnsscomm.TOPIC_MEAS]["md5"] = "0" * 32                           # md5 not matching its definition
        with self.assertRaises(ValueError):
            gnsscomm.check_definitions(gnsscomm.ReferenceDefs(bad))

    @unittest.skipUnless(os.path.isfile(_path.REFERENCE_BAG), "no M3DGR reference bag on this machine")
    def test_reference_bag_definitions_match(self):
        refs = gnsscomm.load_reference_defs(_path.REFERENCE_BAG)
        for topic, c in refs.conns.items():
            self.assertEqual(c["md5"], MD5[gnsscomm.SHORT[topic]])


class TestMapping(unittest.TestCase):
    def setUp(self):
        f = io.StringIO(tr.HDR + tr.BODY)
        self.h = rinex.read_header(f)
        self.sel = rinex.Selection(self.h)
        self.eps = list(rinex.iter_epochs(f, self.h, self.sel.wanted()))

    def test_epoch_to_obs(self):
        stats = {}
        obs = gnsscomm.epoch_to_obs(self.eps[0], self.sel, stats)
        by = {o["sat"]: o for o in obs}
        self.assertEqual(sorted(by), [5, 39, 63, 126])             # G05, R07 (32+7), E04 (59+4), C29 (97+29); S and J dropped
        self.assertEqual(stats.get("skipped_S"), 1)
        g = by[5]
        self.assertEqual(g["time"], (2322, 28480.6))
        self.assertEqual(g["freqs"], [1575.42e6, 1227.60e6])
        self.assertEqual(g["CN0"], [33.0, 9.0])
        self.assertEqual(g["psr"], [22030924.229, 22030927.553])
        self.assertEqual(g["dopp"], [-1291.267, -1006.167])
        # L1: RINEX LLI 1 -> LLI 1, status psr|cp|half-cycle-resolved (7); L2: LLI 2 (half cycle) -> LLI 2, no bit 2 (3)
        self.assertEqual((g["LLI"], g["status"]), ([1, 2], [7, 3]))
        self.assertEqual(g["code"], [1, 20])                         # RTKLIB-EX codes 1C, 2W
        self.assertEqual(g["psr_std"] + g["cp_std"] + g["dopp_std"], [0.0] * 6)
        bds = by[126]
        self.assertEqual(bds["freqs"], [1561.098e6, 1207.14e6])
        self.assertEqual(bds["code"], [gnsscomm.BDS_B1I_GNSS_COMM_CODE, 27])
        glo = by[39]
        self.assertEqual(glo["freqs"], [1604.8125e6, 1248.1875e6])

    def test_primary_first_when_secondary_listed_first(self):
        h = rinex.read_header(io.StringIO(tr.HDR.replace("E    8 C1C L1C D1C S1C C7Q L7Q D7Q S7Q",
                                                       "E    8 C7Q L7Q D7Q S7Q C1C L1C D1C S1C")))
        sel = rinex.Selection(h)
        vals = {"C7Q": (1e7, 0, 0), "L7Q": (5e7, 0, 0), "D7Q": (-10.0, 0, 0), "S7Q": (30.0, 0, 0),
                "C1C": (2e7, 0, 0), "L1C": (9e7, 0, 0), "D1C": (-12.0, 0, 0), "S1C": (40.0, 0, 0)}
        sigs = gnsscomm.sat_signals("E", 4, vals, sel)
        self.assertEqual([c for _, c, _, _ in sigs], ["L1X", "L7X"])

    def test_signal_without_phase_has_no_cp_bits(self):
        vals = {"C1C": (2e7, 0, 0), "D1C": (-12.0, 0, 0), "S1C": (40.0, 0, 0)}
        (s, code, st, lli), = gnsscomm.sat_signals("G", 3, vals, self.sel)
        self.assertEqual((st, lli, s.cp), (1, 0, 0.0))

    def test_round_trip_through_the_exporter(self):
        """RINEX -> gnss_comm bytes -> decode -> tools/gnss_bag_export.py RINEX -> our reader: the values survive."""
        try:
            from rosbags.typesys import Stores, get_typestore, get_types_from_msg
        except ImportError:
            self.skipTest("rosbags not installed")
        import gnss_bag_export as exp
        defs = synth_defs()
        ts = get_typestore(Stores.ROS1_NOETIC)
        for c in defs.values():
            ts.register(get_types_from_msg(c["msgdef"], c["msgtype"]))
        obs = gnsscomm.epoch_to_obs(self.eps[0], self.sel)
        msg = ts.deserialize_ros1(gnsscomm.pack_meas(obs), defs[gnsscomm.TOPIC_MEAS]["msgtype"])
        self.assertEqual(len(msg.meas), 4)
        self.assertEqual(float(msg.meas[0].psr[0]), 22030924.229)
        text, _, n = exp.rinex_from_slice({"range_meas": [(0, msg)]}, "T", None, "M")
        self.assertEqual(n, 1)
        f = io.StringIO(text)
        h = rinex.read_header(f)
        sel = rinex.Selection(h)
        back = list(rinex.iter_epochs(f, h, sel.wanted()))[0]
        got = {(s, p): v for s, p, v in back.sats}
        self.assertAlmostEqual(got[("G", 5)]["C1C"][0], 22030924.229, 3)
        self.assertAlmostEqual(got[("G", 5)]["L1C"][0], 115773294.540, 3)
        self.assertAlmostEqual(got[("G", 5)]["D2X"][0], -1006.167, 3)          # exporter names the L2 band 2X
        self.assertEqual(got[("G", 5)]["L1C"][1], 1)                           # LLI bit 0 survives
        self.assertEqual(got[("G", 5)]["L2X"][1], 2)                           # half-cycle flag survives


class TestNavigation(unittest.TestCase):
    def rec(self, **kw):
        r = {"sys": "G", "prn": 14, "source": "rinex", "toc": 1720342800.0, "toe": 1720342800.0, "ttr": 1720337000.0,
             "toes": 21600.0, "week": 2322, "af0": 5.8e-4, "af1": 7e-12, "af2": 0.0, "iode": 74, "crs": 44.0, "deln": 5e-9,
             "M0": -1.1, "cuc": 2e-6, "e": 0.005, "cus": 6e-6, "A": 26560650.0, "cic": 9e-8, "OMG0": 1.97, "cis": -1e-7,
             "i0": 0.94, "crc": 249.0, "omg": -2.86, "OMGd": -8.3e-9, "idot": 5.7e-11, "code": 1, "flag": 0, "sva_m": 2.0,
             "svh": 0, "tgd0": -7.9e-9, "tgd1": 0.0, "iodc": 586, "fit": 4.0, "aodc": 0}
        r.update(kw)
        return r

    def test_kepler_fields(self):
        f = gnsscomm.kepler_fields(self.rec())
        self.assertEqual(f["sat"], 14)
        self.assertEqual(f["toe"], (2322, 32400.0))
        self.assertEqual((f["toe_tow"], f["week"]), (f["toe"][1], f["toe"][0]))
        self.assertEqual((f["iode"], f["iodc"], f["health"], f["code"]), (74, 586, 0, 1))
        self.assertEqual((f["ura"], f["A"], f["tgd0"], f["A_dot"], f["n_dot"]), (2.0, 26560650.0, -7.9e-9, 0.0, 0.0))
        e = gnsscomm.kepler_fields(self.rec(sys="E", prn=7, tgd1=3.7e-9, code=513))
        self.assertEqual((e["sat"], e["tgd1"], e["code"]), (59 + 7, 3.7e-9, 513))
        c = gnsscomm.kepler_fields(self.rec(sys="C", prn=12, aodc=3, iodc=0, code=0))
        self.assertEqual((c["sat"], c["iodc"], c["code"]), (97 + 12, 3, 0))
        self.assertIsNone(gnsscomm.kepler_fields(self.rec(sys="J", prn=1)))

    def test_glo_fields(self):
        toe_gpst = 1720342800.0 + 22500.0 - 21600.0 + 0.0     # some instant
        r = {"sys": "R", "prn": 3, "toe": toe_gpst, "taun": -1.0e-4, "gamn": 1e-12, "tof_raw": 367218.0,
             "pos": [1.0e6, 2.0e6, 3.0e6], "vel": [1.0, 2.0, 3.0], "acc": [0.0, 0.0, 0.0], "svh": 0, "frq": 5, "age": 0}
        f = gnsscomm.glo_fields(r)
        self.assertEqual((f["sat"], f["freqo"], f["tau_n"], f["delta_tau_n"], f["ura"]), (35, 5, -1.0e-4, 0.0, 0.0))
        # ttr = the instant with tof's time of day (367218 mod 86400 = 21618 s UTC) nearest to toe (UTC)
        t_toe_utc = toe_gpst - 18
        t_ttr_utc = gnsscomm.gnsstime.gpst(*f["ttr"]) - 18
        self.assertEqual(t_ttr_utc % 86400, 21618.0)
        self.assertLess(abs(t_ttr_utc - t_toe_utc), 43200)
        self.assertEqual(f["iode"], int(round(((t_toe_utc + 10800) % 86400) / 900)))

    def test_nav_messages_window_and_dedup(self):
        a = self.rec(toe=1000.0 + 1.7e9)
        b = dict(a)
        far = self.rec(prn=15, toe=1.7e9 + 40000.0)
        msgs = gnsscomm.nav_messages([a, b, far], 1.7e9, 1.7e9 + 3600.0)
        self.assertEqual(len(msgs), 1)                                          # duplicate dropped, far-away record out
        self.assertEqual(msgs[0][1], gnsscomm.TOPIC_EPHEM)

    def test_pack_sizes(self):
        f = gnsscomm.kepler_fields(self.rec())
        self.assertEqual(len(gnsscomm.pack_ephem(f)), 4 + 3 * 12 + 8 + 5 * 4 + 23 * 8)
        self.assertEqual(len(gnsscomm.pack_iono(list(range(8)), 1700000000 * 10 ** 9 + 5)), 12 + 4 + 4 + 64)


if __name__ == "__main__":
    unittest.main()
