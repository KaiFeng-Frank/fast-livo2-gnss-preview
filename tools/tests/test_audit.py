# SPDX-License-Identifier: BSD-3-Clause
"""Decision logic of tools/m3dgr_audit.py on synthetic inputs with known answers: std-unit grids (§12.6-3), URA
semantics (A16), the phase-rate vs Doppler sign check (§12.6-2), the signal table mirror of signal_table.hpp,
GLONASS channel consistency, ephemeris coverage (A1), the iono family and ephemeris-stream checks, time_pulse_info
(A19: integer-second offsets), bags without GNSS epochs, and the A2 rule that self-check 5 is never computed on a
non-development sequence. Fixtures are synthetic or public IGS broadcast values (never evaluation-set data)."""
import math
import os
import re
import tempfile
import unittest

import numpy as np

import _path
from fl2g import gnsstime, report, sats
import m3dgr_audit as au


class Col:
    """Minimal stand-in for m3dgr_audit.Collected."""

    def __init__(self, epochs=(), eph=(), geph=(), pvt=(), iono=()):
        self.epochs = list(epochs)
        self.eph = list(eph)
        self.geph = list(geph)
        self.pvt = list(pvt)
        self.iono = list(iono)
        self.tp = []
        self.lla = []


def slot(freq, psr=2.2e7, cp=0.0, dopp=0.0, cn0=40.0, lli=0, status=1, psr_std=0.32, cp_std=0.004, dopp_std=0.032, code=1):
    return {"freq": freq, "cn0": cn0, "lli": lli, "code": code, "psr": psr, "psr_std": psr_std, "cp": cp, "cp_std": cp_std,
            "dopp": dopp, "dopp_std": dopp_std, "status": status}


def epoch(t, sats_):
    return {"t_gpst": t, "t_record": gnsstime.gpst_to_utc(t) + 0.04, "sats": sats_, "n_time_tags": 1}


def sat(sys, prn, slots):
    return {"sat_raw": 0, "sys": sys, "prn": prn, "sat": sats.satid(sys, prn), "t_gpst": 0.0, "slots": slots,
            "array_lengths": (len(slots),) * 11}


class StdUnits(unittest.TestCase):
    def test_ublox_grids(self):
        lam = sats.wavelength(sats.FREQ_L1)
        vals = {"psr": [0.01 * 2 ** n for n in (3, 4, 5, 9, 10)] * 20,
                "cp": [0.004 * n for n in (1, 2, 3, 15)] * 25,
                "dopp": [0.002 * 2 ** n for n in (4, 5, 12)] * 30}
        lams = [lam] * 100
        vals["dopp"] = vals["dopp"][:100]
        j = au.judge_std_units(vals, lams)
        self.assertEqual(j["psr"]["unit"], "metres")
        self.assertEqual(j["cp"]["unit"], "cycles")
        self.assertEqual(j["dopp"]["unit"], "hertz")
        self.assertAlmostEqual(j["cp"]["frac_invalid_marker"], 0.25)

    def test_cp_std_in_metres_and_ambiguous(self):
        lam = sats.wavelength(sats.FREQ_L1)
        j = au.judge_std_units({"psr": [0.32], "cp": [0.004 * n * lam for n in (1, 2, 5, 7)], "dopp": [0.032] * 4}, [lam] * 4)
        self.assertEqual(j["cp"]["unit"], "metres")
        j = au.judge_std_units({"psr": [0.3, 0.7], "cp": [0.004, 0.0055], "dopp": [0.032, 0.05]}, [lam] * 2)
        self.assertEqual(j["psr"]["unit"], "ambiguous")
        self.assertEqual(j["cp"]["unit"], "ambiguous")


class UraSemantics(unittest.TestCase):
    def rows(self, pairs, sysc="G"):
        return [{"sat": "%s%02d" % (sysc, k + 1), "matched": True, "ura_raw": u, "rnx_sva_m": m} for k, (u, m) in enumerate(pairs)]

    def decide(self, rows):
        checks = report.Checks()
        rep = {}
        au.ura_semantics(rows, Col(), checks, rep)
        return rep["ura_semantics"], checks.items[-1]["verdict"]

    def test_index(self):
        r, v = self.decide(self.rows([(0.0, 2.0), (0.0, 2.0), (1.0, 2.8), (2.0, 4.0)]))
        self.assertEqual(r["config_value"], "index")
        self.assertEqual(v, "PASS")

    def test_metres(self):
        r, v = self.decide(self.rows([(2.0, 2.0), (2.8, 2.8), (4.0, 4.0), (2.0, 2.0)]))
        self.assertEqual(r["config_value"], "metres")

    def test_metres_by_icd_formula(self):
        # 2^(1+n/2) m (2.83 for n=1) against RINEX nominal 2.8: metres within 3 %
        r, v = self.decide(self.rows([(2.0, 2.0), (2.0 ** 1.5, 2.8), (4.0, 4.0)]))
        self.assertEqual(r["config_value"], "metres")

    def test_conflicting_systems_are_undecided(self):
        rows = self.rows([(0.0, 2.0), (0.0, 2.0)], "G") + self.rows([(3.12, 3.12), (3.12, 3.12)], "E")
        r, v = self.decide(rows)
        self.assertEqual(r["decision"], {"G": "index", "E": "metres"})
        self.assertIsNone(r["config_value"])
        self.assertEqual(v, "FAIL")

    def test_galileo_sisa_index(self):
        # SISA index 107 = 3.12 m
        self.assertAlmostEqual(au.sisa_value(107), 3.12)
        self.assertEqual(au.sisa_index(3.12), 107)
        r, v = self.decide(self.rows([(107.0, 3.12), (107.0, 3.12), (100.0, 2.0)], "E"))
        self.assertEqual(r["decision"]["E"], "index")


class SignCheck(unittest.TestCase):
    def make(self, doppler_sign=1.0, n=300):
        """Carrier phase that integrates the Doppler with the design sign (lambda dPhi/dt = -lambda D) plus clock drift."""
        lam = sats.wavelength(sats.FREQ_L1)
        t0 = gnsstime.gpst(2350, 1000.0)
        eps = []
        state = {}
        rng = np.random.RandomState(0)
        rates = {p: rng.uniform(-700, 700) for p in range(1, 7)}        # range rate [m/s]
        for k in range(n):
            t = t0 + 0.1 * k
            ss = []
            for p, rr in rates.items():
                rr_t = rr + 0.5 * np.sin(0.05 * k + p) + 120.0              # + receiver clock drift (common)
                D = -rr_t / lam
                phi = state.get(p, 1.0e6) + (-D * 0.1)                       # dPhi/dt = -D
                state[p] = phi
                ss.append(sat("G", p, [slot(sats.FREQ_L1, psr=2.2e7 + rr_t * 0.1 * k, cp=phi, dopp=doppler_sign * D, status=7)]))
            eps.append(epoch(t, ss))
        return Col(eps)

    def test_design_sign_passes(self):
        checks, rep = report.Checks(), {}
        au.analyse_sign(self.make(), checks, rep)
        self.assertEqual(checks.items[-1]["verdict"], "PASS", checks.items[-1]["evidence"])
        self.assertAlmostEqual(rep["sign_check"]["slope"], 1.0, delta=1e-3)

    def test_flipped_doppler_fails(self):
        checks, rep = report.Checks(), {}
        au.analyse_sign(self.make(doppler_sign=-1.0), checks, rep)
        self.assertEqual(checks.items[-1]["verdict"], "FAIL")
        self.assertIn("SIGN FLIPPED", checks.items[-1]["evidence"])


class SignalTableMirror(unittest.TestCase):
    def test_python_table_equals_signal_table_hpp(self):
        path = os.path.join(_path.REPO, "gnss", "include", "livo_gnss", "signal_table.hpp")
        with open(path) as f:
            text = f.read()
        body = text[text.index("inline SignalCode signalFromFrequency"):text.index("inline double nominalFrequencyHz")]
        cases = {"G": "case Sys::GPS:", "E": "case Sys::GAL:", "C": "case Sys::BDS:"}
        for sysc, marker in cases.items():
            seg = body[body.index(marker):]
            seg = seg[:seg.index("return done(SignalCode::Unknown, SignalVerdict::RejectedUnknownFreq)")]
            pairs = re.findall(r"near\(([0-9.]+)e6\)\) return done\(SignalCode::(\w+), SignalVerdict::(\w+)\)", seg)
            self.assertTrue(pairs, sysc)
            for f, code, verdict in pairs:
                py = sats.signal_from_frequency(sysc, float(f) * 1e6)
                self.assertEqual((py[0], py[1]), (code, verdict), (sysc, f))
        for k in range(-7, 7):
            self.assertEqual(sats.signal_from_frequency("R", 1602e6 + k * 0.5625e6), ("L1C", "Accepted", k))
            self.assertEqual(sats.signal_from_frequency("R", 1246e6 + k * 0.4375e6), ("L2C", "Accepted", k))
        self.assertEqual(sats.signal_from_frequency("R", 1602e6 + 0.3e6)[1], "RejectedGloChannel")
        self.assertEqual(sats.signal_from_frequency("C", 1575.42e6)[1], "RejectedBdsB1C")
        self.assertEqual(sats.from_gnss_comm(33), ("R", 1))
        self.assertEqual(sats.from_gnss_comm(97), ("E", 38))
        self.assertEqual(sats.from_gnss_comm(98), ("C", 1))
        self.assertEqual(sats.from_gnss_comm(160), ("C", 63))
        self.assertEqual(sats.from_gnss_comm(161), (None, 0))


class GloChannelAndCoverage(unittest.TestCase):
    def test_glo_channel_mismatch(self):
        t = gnsstime.gpst(2350, 1000.0)
        eps = [epoch(t, [sat("R", 5, [slot(1602e6 + 3 * 0.5625e6)]), sat("R", 6, [slot(1602e6 - 2 * 0.5625e6)])])]
        geph = [{"sys": "R", "sat": "R05", "frq": 3}, {"sys": "R", "sat": "R06", "frq": 1}]
        checks, rep = report.Checks(), {}
        au.analyse_glo_channel(Col(eps, geph=geph), [], checks, rep)
        self.assertEqual(checks.items[-1]["verdict"], "FAIL")
        self.assertEqual(rep["glo_channel"]["mismatch_slots"], {"R06": 1})

    def test_coverage_bag_only_is_causal(self):
        t0 = gnsstime.gpst(2350, 3600.0)
        eps = [epoch(t0 + k, [sat("G", p, [slot(sats.FREQ_L1)]) for p in range(1, 7)]) for k in range(60)]
        rec0 = eps[0]["t_record"]
        eph = [{"sys": "G", "prn": p, "sat": sats.satid("G", p), "toe": t0, "svh": 0, "t_record": rec0 + (30.0 if p > 3 else -1.0)}
               for p in range(1, 7)]
        rnx = [{"sys": "G", "prn": p, "sat": sats.satid("G", p), "toe": t0, "svh": 0} for p in range(1, 7)]
        checks, rep = report.Checks(), {}
        import tempfile
        with tempfile.TemporaryDirectory() as d:
            au.analyse_coverage(Col(eps, eph=eph), rnx, checks, rep, d, False)
        cov = rep["eph_coverage"]
        # 3 satellites from the start, 6 after their ephemerides arrive at +30 s; RINEX covers everything
        self.assertAlmostEqual(cov["sat_epoch_fraction"]["bag_only"], (3 * 60 + 3 * 30) / 360.0, delta=0.02)
        self.assertEqual(cov["sat_epoch_fraction"]["bag_rinex"], 1.0)
        self.assertAlmostEqual(cov["first_epoch_ge5_healthy_eph_s"]["bag_only"], 30.0, delta=1.01)
        self.assertEqual(cov["first_epoch_ge5_healthy_eph_s"]["bag_rinex"], 0.0)


class IonoAndEphemStream(unittest.TestCase):
    # GPSA/GPSB and one BDSA/BDSB set as printed in a public IGS BRDC header (the same GPS values as test_gnss.RNX_HEADER)
    GPS = [2.7940e-08, 2.2352e-08, -1.1921e-07, 5.9605e-08, 1.5155e+05, -1.9661e+05, -6.5536e+04, 3.2768e+05]
    BDS = [1.8626e-08, 3.7998e-07, -2.0862e-06, 2.6226e-06, 1.9866e+05, -7.2090e+05, 1.4418e+06, 6.5536e+04]
    # synthetic bag set: another upload of the BDS D1 family, a few broadcast LSB away from BDS (tens from GPS)
    BDS_LIKE = [b + k * l for b, k, l in zip(BDS, (3, -2, 4, 1, -3, 2, -1, 2), au.KLOB_LSB)]

    def header(self):
        return {"iono_gps_alpha": self.GPS[:4], "iono_gps_beta": self.GPS[4:],
                "iono": [{"kind": "GPSA", "vals": self.GPS[:4], "mark": "", "sv": ""},
                         {"kind": "GPSB", "vals": self.GPS[4:], "mark": "", "sv": ""},
                         {"kind": "BDSA", "vals": self.BDS[:4], "mark": "W", "sv": "02"},
                         {"kind": "BDSB", "vals": self.BDS[4:], "mark": "W", "sv": "02"}]}

    def run_iono(self, data):
        iono = [{"t_record": 100.0 + 3.0 * k, "stamp": None, "data": list(data)} for k in range(20)]
        checks, rep = report.Checks(), {}
        au.analyse_iono(Col(iono=iono), self.header(), checks, rep)
        return checks.items[-1], rep["iono"]

    def test_gps_set_passes(self):
        c, r = self.run_iono(self.GPS)
        self.assertEqual(c["verdict"], "PASS", c["evidence"])
        self.assertEqual(r["classification"][0]["class"], "GPS")
        self.assertEqual(r["gps_klobuchar_source"], "rinex_header")

    def test_bds_set_is_classified_as_a_diagnostic(self):
        # A38: a set of the BDS D1 family (the kind the audit found in Grass02/Outdoor01) is classified but never used:
        # the verdict follows the RINEX GPSA/GPSB set, which the core takes
        c, r = self.run_iono(self.BDS_LIKE)
        self.assertEqual(c["verdict"], "PASS", c["evidence"])
        self.assertEqual(r["classification"][0]["class"], "BDS-like")
        self.assertLess(r["classification"][0]["dist_bds_lsb"], 4.0)
        self.assertGreater(r["classification"][0]["dist_gps_lsb"], 10.0)
        self.assertIn("not used", c["evidence"])
        self.assertEqual(r["gps_klobuchar_source"], "rinex_header")

    def test_without_rinex_gps_set_warns_even_with_a_gps_like_bag_set(self):
        # the core never takes the bag's set (A38): no RINEX GPSA/GPSB means I = 0 whatever the bag carries
        iono = [{"t_record": 100.0 + 3.0 * k, "stamp": None, "data": list(self.GPS)} for k in range(5)]
        checks, rep = report.Checks(), {}
        au.analyse_iono(Col(iono=iono), None, checks, rep)
        self.assertEqual(checks.items[-1]["verdict"], "WARN", checks.items[-1]["evidence"])
        self.assertTrue(rep["iono"]["gps_klobuchar_source"].startswith("none"))
        checks, rep = report.Checks(), {}
        au.analyse_iono(Col(), self.header(), checks, rep)
        self.assertEqual(checks.items[-1]["verdict"], "PASS", checks.items[-1]["evidence"])

    def test_malformed_bag_set_is_a_warning_only(self):
        iono = [{"t_record": 100.0, "stamp": None, "data": list(self.GPS[:6])}]
        checks, rep = report.Checks(), {}
        au.analyse_iono(Col(iono=iono), self.header(), checks, rep)
        self.assertEqual(checks.items[-1]["verdict"], "WARN")

    def test_ephemeris_copy_conflict_is_explained_not_failed(self):
        import test_gnss
        base = test_gnss.circular_eph(prn=16, sys="C", A=42170e3, e=0.0078)
        base.update({"iode": 1, "iodc": 0, "svh": 0, "code": 0, "ura_raw": 2.0, "tgd0": -2.8e-9, "tgd1": 4.6e-9,
                     "raw_toe_tow_field": base["toes"] + 14.0, "raw_week_field": base["week"] + 1356})
        copies = []
        for k in range(5):
            b = dict(base, t_record=1000.0 + 30 * k, source="bag")
            if k == 3:
                b["af1"] = -1.428e-9                  # corrupted clock under the same (sat, toe, IODE), as C16 in Outdoor01
            copies.append(b)
        rnx = dict(base, source="rinex", sva_m=2.0)
        checks, rep = report.Checks(), {}
        au.analyse_ephemerides(Col(eph=copies), {}, [rnx], checks, rep, base["toe"], base["toe"] + 600)
        v = {c["id"]: c for c in checks.items}
        self.assertEqual(v["E1"]["verdict"], "PASS", v["E1"]["evidence"])
        self.assertEqual(v["E7"]["verdict"], "WARN", v["E7"]["evidence"])
        self.assertEqual(rep["eph_compare"]["conflicts"][0]["fields"], ["af1"])
        # without a good copy the same corruption is a FAIL
        checks, rep = report.Checks(), {}
        au.analyse_ephemerides(Col(eph=[copies[3]]), {}, [rnx], checks, rep, base["toe"], base["toe"] + 600)
        self.assertEqual({c["id"]: c for c in checks.items}["E1"]["verdict"], "FAIL")


class NoBagEphemerides(unittest.TestCase):
    def test_e1_is_skip_without_bag_ephemerides(self):
        # Occlusion03/04, Dynamic03 carry no ephem topic (A26): nothing to compare is not a mismatch
        import test_gnss
        rnx = dict(test_gnss.circular_eph(prn=3), source="rinex", sva_m=2.0, iode=1, svh=0, code=0)
        checks, rep = report.Checks(), {}
        au.analyse_ephemerides(Col(), {}, [rnx], checks, rep, rnx["toe"], rnx["toe"] + 600)
        v = {c["id"]: c["verdict"] for c in checks.items}
        self.assertEqual(v["E1"], "SKIP")
        self.assertEqual(v["E4"], "SKIP")


class TimePulse(unittest.TestCase):
    """A19: time_pulse_info is absent in Dark01/Outdoor01; the code path for when it is present is exercised with an
    introspected layout (time = GnssTimeMsg, utc_based, time_sys) and injected integer-second errors. Each message
    announces the next 1 Hz pulse 0.3 s ahead and reaches the host with the range_meas transport latency (40 ms)."""
    LAT_MS = 40.0

    def col(self, err_s=0.0, n=60, utc_based=0, time_sys=1, lead=0.3, repeat=1, numbers="gpst", jitter=0.0):
        c = Col()
        rng = np.random.RandomState(1)
        t0 = gnsstime.gpst(2350, 1000.0)
        for k in range(n):
            tp = t0 + k + 1.0                                   # true GPST of the announced pulse
            rec = gnsstime.gpst_to_utc(tp) - lead + 1e-3 * self.LAT_MS + rng.uniform(-jitter, jitter)
            shown = tp + err_s                                  # what the receiver/driver writes (err = its error)
            if numbers == "utc":
                shown = gnsstime.gpst_to_utc(shown)
            elif numbers == "bdt":
                shown = shown - 14.0
            w, tow = gnsstime.week_tow(shown)
            if numbers == "bdt":
                w -= 1356
            for r in range(repeat):
                c.tp.append({"t_record": rec + 0.1 * r, "msg": {"time": {"week": w, "tow": tow}, "utc_based": utc_based, "time_sys": time_sys}})
        return c

    def run_tp(self, col):
        checks, rep = report.Checks(), {}
        au.analyse_time_pulse(col, checks, rep, np.array([self.LAT_MS] * 10))
        return checks.items[-1], rep["time_pulse"]

    def test_consistent_pulses_pass(self):
        c, r = self.run_tp(self.col())
        self.assertEqual(c["verdict"], "PASS", c["evidence"])
        self.assertEqual((r["gross_interval_errors"], r["integer_second_offset_k"]), (0, 0))
        self.assertAlmostEqual(r["lead_s"]["median"], 0.3, delta=1e-6)

    def test_one_second_errors_fail(self):
        for err, k in ((+1.0, 1), (-1.0, -1)):
            c, r = self.run_tp(self.col(err_s=err))
            self.assertEqual(c["verdict"], "FAIL", c["evidence"])
            self.assertEqual(r["integer_second_offset_k"], k)
            self.assertIn("leap second", c["evidence"])

    def test_one_second_error_near_the_window_edge_is_still_caught(self):
        for lead in (0.02, 0.97):
            for err in (+1.0, -1.0):
                c, r = self.run_tp(self.col(err_s=err, lead=lead))
                self.assertEqual(c["verdict"], "FAIL", (lead, err, c["evidence"]))

    def test_lead_straddling_the_window_edge_is_ambiguous(self):
        # the message sent right after a pulse announces the next one ~1 s ahead: with 20 ms jitter the lead straddles
        # the edge of [0, 1) s, so a one-second error could hide: WARN, never PASS
        c, r = self.run_tp(self.col(lead=0.995, jitter=0.02))
        self.assertEqual(c["verdict"], "WARN", c["evidence"])
        self.assertGreater(r["frac_messages_other_k"], 0.1)

    def test_utc_numbers_read_as_gpst_fail(self):
        c, r = self.run_tp(self.col(numbers="utc", utc_based=0))
        self.assertEqual(c["verdict"], "FAIL", c["evidence"])
        self.assertEqual(r["integer_second_offset_k"], -18)

    def test_declared_time_bases_are_honoured(self):
        c, r = self.run_tp(self.col(numbers="utc", utc_based=1))
        self.assertEqual(c["verdict"], "PASS", c["evidence"])
        self.assertEqual(r["interpretation"], {"UTC": 60})
        c, r = self.run_tp(self.col(numbers="bdt", time_sys=0x20))
        self.assertEqual(c["verdict"], "PASS", c["evidence"])
        self.assertEqual(r["interpretation"], {"BDT": 60})

    def test_repeated_announcements_are_not_interval_errors(self):
        c, r = self.run_tp(self.col(repeat=3))
        self.assertEqual(c["verdict"], "PASS", c["evidence"])
        self.assertEqual(r["repeated_announcements"], 120)
        self.assertEqual(r["gross_interval_errors"], 0)

    def test_absent_topic_skips(self):
        checks, rep = report.Checks(), {}
        au.analyse_time_pulse(Col(), checks, rep, np.array([]))
        self.assertEqual(checks.items[-1]["verdict"], "SKIP")

    def col5(self, err_s=0.0, n=100, send_after=0.06, per_pulse_only=False, host_step_at=None):
        """The Grass01/02 layout: a pulse every 5 s, UTC-based numbers, one announcement per second (send_after s past
        the second) of the next pulse, so the leads walk 4.94, 3.94, ..., 0.94 s."""
        c = Col()
        t0 = gnsstime.gpst(2350, 1000.0)
        for k in range(n):
            send = t0 + k + send_after
            pulse = t0 + 5.0 * (math.floor((k + send_after) / 5.0) + 1)
            if per_pulse_only and (pulse - send) < 4.0:
                continue                                       # one announcement per pulse, 4.94 s ahead only
            rec = gnsstime.gpst_to_utc(send) + 1e-3 * self.LAT_MS + (0.3 if host_step_at is not None and k >= host_step_at else 0.0)
            w, tow = gnsstime.week_tow(gnsstime.gpst_to_utc(pulse + err_s))
            c.tp.append({"t_record": rec, "msg": {"time": {"week": w, "tow": tow}, "utc_based": 1, "time_sys": 0}})
        return c

    def test_five_second_pulse_train(self):
        c, r = self.run_tp(self.col5())
        self.assertEqual(c["verdict"], "PASS", c["evidence"])
        self.assertEqual((r["integer_second_offset_k"], r["lead_integer_parts"]), (0, [0, 1, 2, 3, 4]))
        self.assertEqual(r["host_steps"], [])
        for err, k in ((+1.0, 1), (-1.0, -1)):                 # the announcements cover the window: +-1 s are caught
            c, r = self.run_tp(self.col5(err_s=err))
            self.assertEqual(c["verdict"], "FAIL", (err, c["evidence"]))
            self.assertEqual(r["integer_second_offset_k"], k)
        c, r = self.run_tp(self.col5(host_step_at=60))         # a 0.3 s host-clock step is a step, the sawtooth is not
        self.assertEqual(c["verdict"], "PASS", c["evidence"])
        self.assertEqual(len(r["host_steps"]), 1)
        self.assertAlmostEqual(r["host_steps"][0][1], -300.0, delta=5.0)
        c, r = self.run_tp(self.col5(per_pulse_only=True))      # only 4.94 s-ahead announcements: +1 s is undetectable
        self.assertEqual(c["verdict"], "WARN", c["evidence"])
        self.assertIn("not decisive", c["evidence"])


class NoEpochs(unittest.TestCase):
    """A bag without range_meas (GNSS-denial, bag_filter --drop-gnss): every analysis completes, nothing crashes."""

    def test_all_analyses_survive(self):
        import tempfile
        col = Col()
        checks, rep = report.Checks(), {}
        with tempfile.TemporaryDirectory() as d:
            au.analyse_epochs(col, checks, rep)
            au.analyse_signals(col, checks, rep)
            au.analyse_sign(col, checks, rep)
            au.analyse_glo_channel(col, [], checks, rep)
            au.analyse_sat2freq(col, [], checks, rep)
            au.analyse_ephemerides(col, None, [], checks, rep, 0.0, 0.0)
            au.analyse_coverage(col, [], checks, rep, d, False)
            au.analyse_iono(col, None, checks, rep)
            lat = au.analyse_latency(col, checks, rep, d, False)
            au.analyse_time_pulse(col, checks, rep, lat)
            au.analyse_ls(col, [], None, checks, rep, 1.0, evaluation=False)
            rep["conclusions"] = au.conclusions(rep, checks)
        v = {c["id"]: c["verdict"] for c in checks.items}
        for cid in ("G3", "G4", "G5", "G6", "G7", "G8", "G14", "E5", "L1", "L3", "G11", "G12a", "G12b", "T1"):
            self.assertEqual(v.get(cid), "SKIP", (cid, v))


class OneHzDecimation(unittest.TestCase):
    """G15: the A43 1 Hz rule (first epoch of every GPST second) selects one epoch per second whatever the receiver's
    grid phase (PASS); the retired DESIGN §4.3 rule (|tow - round(tow)| < 5 ms) is reported beside it: on Grass01's grid,
    7 ms before the whole seconds, it would select nothing. The tools' own 1 Hz subsample (first epoch of each second)
    yields the coverage timeline either way."""

    def analyse(self, phase):
        t0 = gnsstime.gpst(2350, 3600.0) + phase
        eps = [epoch(t0 + 0.1 * k, [sat("G", p, [slot(sats.FREQ_L1)]) for p in range(1, 7)]) for k in range(300)]
        rnx = [{"sys": "G", "prn": p, "sat": sats.satid("G", p), "toe": t0, "svh": 0} for p in range(1, 7)]
        checks, rep = report.Checks(), {}
        au.analyse_epochs(Col(eps), checks, rep)
        with tempfile.TemporaryDirectory() as d:
            au.analyse_coverage(Col(eps), rnx, checks, rep, d, False)
        return {c["id"]: c for c in checks.items}, rep

    def test_aligned_grid(self):
        c, rep = self.analyse(0.0)
        self.assertEqual(c["G15"]["verdict"], "PASS", c["G15"]["evidence"])
        self.assertEqual(rep["eph_coverage"]["sat_epoch_fraction"]["bag_rinex"], 1.0)

    def test_grid_seven_ms_before_the_second(self):
        c, rep = self.analyse(-0.007)
        self.assertEqual(c["G15"]["verdict"], "PASS", c["G15"]["evidence"])
        self.assertIn("would select an epoch in 0.0%", c["G15"]["evidence"])
        self.assertEqual(rep["epochs"]["a43_epochs_selected"], rep["epochs"]["a43_seconds_with_data"])
        self.assertAlmostEqual(rep["epochs"]["grid_offset_s"], -0.007, delta=1e-6)
        self.assertEqual(rep["epochs"]["frac_seconds_with_whole_second_epoch"], 0.0)
        # the coverage timeline does not depend on the grid phase
        self.assertEqual(rep["eph_coverage"]["sat_epoch_fraction"]["bag_rinex"], 1.0)
        self.assertGreaterEqual(len(rep["eph_coverage"]["per_system"]), 1)


class DataDiscipline(unittest.TestCase):
    """A2: self-check 5 compares GNSS-only solutions with the receiver PVT — never on a non-development sequence."""

    def test_sequence_classes(self):
        self.assertEqual(au.seq_class("Outdoor01"), "evaluation")
        self.assertEqual(au.seq_class("Outdoor01_filtered"), "evaluation")
        self.assertEqual(au.seq_class("gnss-denial02"), "evaluation")
        for s in ("Occlusion03", "Occlusion04", "Dynamic03", "Grass01", "Grass02", "Z-Rough-Road01"):      # A2 + A26 + A69
            self.assertEqual(au.seq_class(s), "development", s)
        self.assertEqual(au.seq_class("synthetic"), "unknown")
        self.assertEqual(au.seq_class("Grass03"), "unknown")

    def col(self):
        t = gnsstime.gpst(2350, 1000.0)
        eps = [epoch(t + k, [sat("G", p, [slot(sats.FREQ_L1, dopp=100.0 * p)]) for p in range(1, 7)]) for k in range(5)]
        pvt = [{"t_gpst": t + k, "t_record": 0.0, "fix_type": 3, "lat": 34.0, "lon": 108.7, "h_ell": 400.0, "h_acc": 1.0,
                "num_sv": 12, "vel_n": 0.0, "vel_e": 1.0, "vel_d": 0.0} for k in range(5)]
        return Col(eps, pvt=pvt)

    def test_self_check_5_is_not_computed_off_the_development_set(self):
        calls = []
        orig = au.gnsslsq.spp

        def spy(*a, **k):
            calls.append(1)
            return None
        au.gnsslsq.spp = spy
        try:
            checks, rep = report.Checks(), {}
            au.analyse_ls(self.col(), [], None, checks, rep, 1.0, evaluation=True)
            self.assertEqual(calls, [])                         # no SPP at all
            v = {c["id"]: c for c in checks.items}
            for cid in ("G12a", "G12b"):
                self.assertEqual(v[cid]["verdict"], "SKIP")
                self.assertIn("A2", v[cid]["evidence"])
            self.assertNotIn("spp_vs_pvt", rep["ls"])
            checks, rep = report.Checks(), {}
            au.analyse_ls(self.col(), [], None, checks, rep, 1.0, evaluation=False)
            self.assertGreater(len(calls), 0)                   # the development path does compute it
            self.assertIn("spp_vs_pvt", rep["ls"])
        finally:
            au.gnsslsq.spp = orig


class TopicRequirements(unittest.TestCase):
    """C2 after A38: the replay core topics and range_meas are required (FAIL when missing); the bag ephemerides and
    the receiver PVT only add options (WARN: RINEX NAV is an input of every sequence, A1/A26); iono_params and
    time_pulse_info carry no requirement at all (A38: diagnostics / A19 only)."""

    def c2(self, missing):
        conns = [{"topic": t, "datatype": "x/Y", "md5sum": "0", "msg_def": "", "callerid": "", "latching": "0", "count": 1}
                 for t in au.REQUIRED_TOPICS if t not in missing]
        checks, rep = report.Checks(), {}
        au.analyse_connections(conns, checks, rep)
        return {c["id"]: c for c in checks.items}["C2"]

    def test_verdicts(self):
        self.assertEqual(self.c2(())["verdict"], "PASS")
        c = self.c2((au.gnss_msgs.TOPIC_TP, au.gnss_msgs.TOPIC_IONO))              # Dark01-like
        self.assertEqual(c["verdict"], "PASS", c["evidence"])
        c = self.c2((au.gnss_msgs.TOPIC_EPHEM, au.gnss_msgs.TOPIC_GLO_EPHEM, au.gnss_msgs.TOPIC_IONO))   # Occlusion03-like
        self.assertEqual(c["verdict"], "WARN", c["evidence"])
        self.assertIn("RINEX", c["evidence"])
        self.assertEqual(self.c2((au.gnss_msgs.TOPIC_PVT,))["verdict"], "WARN")
        for t in au.CORE_TOPICS:
            self.assertEqual(self.c2((t,))["verdict"], "FAIL", t)


class PerSatelliteSign(unittest.TestCase):
    """§12.6-2 says 每星: a satellite whose Doppler has the wrong scale fails even when the pooled fit passes; a
    satellite whose range rate hardly varies (GEO-like) is reported, not judged."""

    def col(self, bad_scale=None, n=600):
        lam = sats.wavelength(sats.FREQ_L1)
        t0 = gnsstime.gpst(2350, 1000.0)
        base = {1: -600.0, 2: 300.0, 3: 50.0, 4: 700.0}
        trend = {1: 0.2, 2: -0.25, 3: 0.0, 4: 0.3}            # m/s per s; PRN 3 has no trend (GEO-like)
        phi, psr, eps = {}, {}, []
        for k in range(n):
            ss = []
            for p in base:
                rate = base[p] + trend[p] * 0.1 * k + 0.3 * np.sin(0.05 * k + p) + 120.0     # + common clock drift
                D = -rate / lam
                phi[p] = phi.get(p, 1.0e6) - D * 0.1           # dPhi/dt = -D
                psr[p] = psr.get(p, 2.2e7) + rate * 0.1
                dopp = D * (bad_scale if bad_scale and p == 4 else 1.0)
                ss.append(sat("G", p, [slot(sats.FREQ_L1, psr=psr[p], cp=phi[p], dopp=dopp, status=7)]))
            eps.append(epoch(t0 + 0.1 * k, ss))
        return Col(eps)

    def test_all_satellites_consistent(self):
        checks, rep = report.Checks(), {}
        au.analyse_sign(self.col(), checks, rep)
        self.assertEqual(checks.items[-1]["verdict"], "PASS", checks.items[-1]["evidence"])
        ps = rep["sign_check"]["per_sat"]
        self.assertEqual(sorted(k for k, v in ps.items() if v["decisive"]), ["G01", "G02", "G04"])
        self.assertFalse(ps["G03"]["decisive"])                # 0.2 m/s spread: corr is not a criterion there

    def test_one_satellite_with_a_wrong_scale_fails(self):
        checks, rep = report.Checks(), {}
        au.analyse_sign(self.col(bad_scale=0.9), checks, rep)
        c = checks.items[-1]
        self.assertEqual(c["verdict"], "FAIL", c["evidence"])
        self.assertIn("FAILING: G04", c["evidence"])
        self.assertLess(abs(rep["sign_check"]["per_sat"]["G04"]["slope"] - 0.9), 0.01)


class Sat2FreqMirror(unittest.TestCase):
    """A35-C8: the audit's sat2freq mirror equals RTKLIB-EX code2freq_* / sat2freq of the vendored source, and the
    check flags a GLONASS channel that disagrees with the ephemeris while a satellite without one stays unverifiable."""

    def test_band_tables_equal_the_vendored_rtklib(self):
        src = os.path.join(_path.REPO, "third_party", "rtklib", "upstream", "src")
        with open(os.path.join(src, "rtklib.h")) as f:
            macros = {m.group(1): float(m.group(2)) for m in re.finditer(r"#define\s+(FREQ\w+|DFRQ\w+)\s+([0-9.Ee+-]+)", f.read())}
        with open(os.path.join(src, "rtkcmn.c")) as f:
            text = f.read()
        systems = {"GPS": "G", "GLO": "R", "GAL": "E", "QZS": "J", "BDS": "C"}
        for name, sysc in systems.items():
            body = text[text.index("static int code2freq_%s(" % name):]
            body = body[:body.index("\n}\n")]
            cases = dict(re.findall(r"case '(\d)':\s*\*freq=(\w+);", body))
            self.assertTrue(cases or sysc == "R", name)
            for band, macro in cases.items():
                self.assertEqual(sats.RTK_BAND_FREQ[sysc][band], macros[macro], (name, band, macro))
            self.assertEqual(set(sats.RTK_BAND_FREQ[sysc]), set(cases), name)
            if sysc == "R":
                self.assertIn("*freq=FREQ1_GLO+DFRQ1_GLO*fcn;", body)
                self.assertIn("*freq=FREQ2_GLO+DFRQ2_GLO*fcn;", body)
                self.assertEqual((sats.FREQ_G1, sats.DFRQ_G1, sats.FREQ_G2, sats.DFRQ_G2),
                                 (macros["FREQ1_GLO"], macros["DFRQ1_GLO"], macros["FREQ2_GLO"], macros["DFRQ2_GLO"]))
        # every accepted code of the frequency table maps back through code2freq (non-GLONASS: no channel involved)
        for sysc, f in (("G", sats.FREQ_L1), ("G", sats.FREQ_L2), ("G", sats.FREQ_L5), ("E", sats.FREQ_L1), ("E", sats.FREQ_E5B),
                        ("E", sats.FREQ_L5), ("C", sats.FREQ_B1I), ("C", sats.FREQ_E5B)):
            code, verdict, _ = sats.signal_from_frequency(sysc, f)
            self.assertEqual((verdict, sats.rtk_sat2freq(sysc, code)), ("Accepted", f), (sysc, code))
        self.assertEqual(sats.rtk_sat2freq("R", "L1C", None), 0.0)          # RTKLIB: no channel -> error value 0
        self.assertEqual(sats.rtk_sat2freq("R", "L2C", 3), sats.FREQ_G2 + 3 * sats.DFRQ_G2)

    def test_check_flags_a_channel_mismatch_only(self):
        t = gnsstime.gpst(2350, 1000.0)
        eps = [epoch(t, [sat("G", 3, [slot(sats.FREQ_L1), slot(sats.FREQ_L2)]), sat("E", 11, [slot(sats.FREQ_L1), slot(1227.60e6)]),
                         sat("C", 21, [slot(sats.FREQ_B1I), slot(sats.FREQ_E5B)]),
                         sat("R", 5, [slot(1602e6 + 3 * 0.5625e6)]), sat("R", 7, [slot(1602e6 + 1 * 0.5625e6)])])]
        rnx = [{"sys": "R", "sat": "R05", "frq": 3}]
        checks, rep = report.Checks(), {}
        au.analyse_sat2freq(Col(eps), rnx, checks, rep)
        c = checks.items[-1]
        self.assertEqual(c["verdict"], "PASS", c["evidence"])
        self.assertEqual(rep["sat2freq"]["unverifiable_sats"], {"R07": 1})          # no ephemeris: RTKLIB returns 0
        self.assertEqual(rep["sat2freq"]["per_system"]["GAL"]["not_accepted"], 1)   # the mislabelled E5b slot (G3)
        eps[0]["sats"].append(sat("R", 6, [slot(1602e6 - 2 * 0.5625e6)]))
        checks, rep = report.Checks(), {}
        au.analyse_sat2freq(Col(eps, geph=[{"sys": "R", "sat": "R06", "frq": 1}]), rnx, checks, rep)
        self.assertEqual(checks.items[-1]["verdict"], "FAIL")
        self.assertEqual(rep["sat2freq"]["per_system"]["GLO"]["mismatch"], 1)


class LeverArm(unittest.TestCase):
    """DESIGN §12.6-7 (G13) on a synthetic drive with a known antenna: W is the replay's world frame (tilted 2 deg,
    yawed 40 deg from ENU), the state trace carries R, v, b_g and g in W, the gyro the body yaw rate, and the Doppler
    rows are made with the true antenna velocity v + R (omega x l_true) through the solver's model rate (the rate model
    itself is pinned by test_gnss). With l_true = l the regression coefficient is ~0 (PASS), with -l ~-2 (FAIL: sign
    flipped), without a lever ~-1 (WARN)."""
    LEAP, OFF = 18.0, 0.030
    T0 = 240300.0

    def build(self, l_true, seed=3, noise=0.03, dur=150.0):
        import test_gnss
        import synth
        rng = np.random.RandomState(seed)
        rr = au.geodesy.pos2ecef(math.radians(34.03), math.radians(108.76), 400.0)
        E = au.geodesy.enu_matrix(math.radians(34.03), math.radians(108.76))
        for s_ in range(200):
            ephs = test_gnss.synthetic_constellation(self.T0, rr, n=24, seed=s_)
            if len(ephs) >= 9:
                break
        sel = test_gnss.FakeSelector(ephs)
        t_est_of = lambda t: t - self.LEAP + self.OFF
        e0, e1 = t_est_of(self.T0) - 3.0, t_est_of(self.T0 + dur) + 3.0
        m = synth.Motion(e0, e1)
        ca, sa = math.cos(math.radians(40.0)), math.sin(math.radians(40.0))
        tilt = math.radians(2.0)
        R_NW = np.array([[ca, -sa, 0], [sa, ca, 0], [0, 0, 1.0]]) @ np.array([[1, 0, 0], [0, math.cos(tilt), -math.sin(tilt)], [0, math.sin(tilt), math.cos(tilt)]])
        R_WN = R_NW.T

        def state(t):
            _, _, yaw = m.pose(t)
            v = np.interp(t, m.t, m.v)
            R_NI = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1.0]])
            return R_NI, np.array([v * math.cos(yaw), v * math.sin(yaw), 0.0]), np.array([0.0, 0.0, np.interp(t, m.t, m.w)])

        ts = synth.estimator_times(e0 + 0.5, e1 - 0.5)
        Rs, vs_ = [], []
        for t in ts:
            R_NI, vN, _ = state(t)
            Rs.append(R_WN @ R_NI)
            vs_.append(R_WN @ vN)
        n = ts.size
        traj = {"t": ts, "R": np.array(Rs), "p": np.zeros((n, 3)), "v": np.array(vs_), "bg": np.zeros((n, 3)),
                "g": np.tile(R_WN @ np.array([0.0, 0.0, -9.81]), (n, 1))}
        tg = np.arange(e0, e1, 0.005)
        gyro = np.column_stack([tg, np.zeros_like(tg), np.zeros_like(tg), np.interp(tg, m.t, m.w) + rng.normal(0, 0.005, tg.size)])
        samples = []
        pos = au.geodesy.ecef2pos(rr)
        for k in range(int(dur * 10)):
            t = self.T0 + 0.1 * k
            R_NI, vN, om = state(t_est_of(t))
            wE = E.T @ (vN + R_NI @ np.cross(om, l_true))
            obs = [{"sat": e["sat"], "sys": e["sys"], "prn": e["prn"], "freq": sats.FREQ_L1, "psr": 2.2e7, "psr_ok": False,
                    "dopp": 1.0, "cn0": 45.0} for e in ephs]
            rows = []
            for s in au.gnsslsq.sat_states(obs, t, sel, need_vel=True):
                _, e_los = au.geodesy.geodist(s["rs"], rr)
                az, el = au.geodesy.satazel(pos, e_los)
                if el < math.radians(10.0):
                    continue
                z = au.gnsslsq.doppler_model_rate(e_los, s["rs"], s["vs"], rr, wE) + 45.0 - sats.CLIGHT * s["ddts"] + rng.normal(0, noise)
                rows.append((s["obs"]["sat"], e_los, s["rs"], s["vs"], s["ddts"], z, el))
            samples.append({"t_gpst": t, "rr": rr, "rows": rows})
        return samples, traj, gyro, t_est_of

    def run_case(self, scale):
        samples, traj, gyro, t_est_of = self.build(scale * np.array(au.DEFAULT_LEVER_AVIA))
        res = au.lever_arm_test(samples, traj, gyro, t_est_of, au.DEFAULT_LEVER_AVIA)
        return res, au.lever_verdict(res)

    def test_true_lever_passes(self):
        res, (verdict, txt) = self.run_case(1.0)
        self.assertEqual(verdict, "PASS", txt)
        self.assertLess(abs(res["k"]), 0.1, txt)
        self.assertLess(res["hypotheses"]["l"]["rms_turn_mps"], 0.04, txt)          # = the injected 0.03 m/s noise
        self.assertGreater(res["hypotheses"]["-l"]["rms_turn_mps"], 2.0 * res["hypotheses"]["l"]["rms_turn_mps"], txt)

    def test_flipped_lever_fails(self):
        res, (verdict, txt) = self.run_case(-1.0)
        self.assertEqual(verdict, "FAIL", txt)
        self.assertLess(abs(res["k"] + 2.0), 0.15, txt)

    def test_no_lever_is_inconclusive(self):
        res, (verdict, txt) = self.run_case(0.0)
        self.assertEqual(verdict, "WARN", txt)
        self.assertLess(abs(res["k"] + 1.0), 0.15, txt)

    def test_state_trace_reader(self):
        R = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
        def line(cycle, flag, t, v):
            vals = [t] + list(R.ravel()) + [1.0, 2.0, 3.0] + list(v) + [1e-3, 2e-3, 3e-3] + [0.0] * 3 + [0.1, 0.2, -9.8] + [1.0] + [0.5] * 19
            return "%d %d " % (cycle, flag) + " ".join(float(x).hex() for x in vals)
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, "state_trace.txt")
            with open(p, "w") as f:
                f.write("# cycle flag t_k R(9,row-major) p(3) v(3) b_g(3) b_a(3) g(3) tau diagP(19) x_G(n) diagQ_G(n)  [%a hexfloat]\n")
                f.write(line(10, 0, 100.0, (1.0, 0.0, 0.0)) + "\n")          # LIO of the image time 100.0
                f.write(line(11, 1, 100.0, (1.5, 0.0, 0.0)) + "\n")          # VIO of the same time: this one is kept
                f.write(line(16, 0, 100.1, (2.0, 0.0, 0.0)) + "\n")          # cycle "16" read as 16, not as hex
            tr = au.load_state_trace(p)
        np.testing.assert_array_equal(tr["t"], [100.0, 100.1])
        np.testing.assert_array_equal(tr["v"][:, 0], [1.5, 2.0])
        np.testing.assert_array_equal(tr["R"][0], R)
        np.testing.assert_array_equal(tr["g"][1], [0.1, 0.2, -9.8])
        np.testing.assert_array_equal(tr["bg"][0], [1e-3, 2e-3, 3e-3])

    def test_gating(self):
        import argparse
        args = argparse.Namespace(replay_dir="/nonexistent/replay", time_calib="", imu_topic=au.IMU_TOPIC,
                                  lever_arm=list(au.DEFAULT_LEVER_AVIA), imu_time_offset=0.0)
        checks, rep = report.Checks(), {}
        au.analyse_lever_arm(Col(), [], args, checks, rep, evaluation=True)           # A2: never opened
        self.assertEqual(checks.items[-1]["verdict"], "SKIP")
        self.assertIn("A2", checks.items[-1]["evidence"])
        checks, rep = report.Checks(), {}
        au.analyse_lever_arm(Col(), [], args, checks, rep, evaluation=False)
        self.assertEqual(checks.items[-1]["verdict"], "SKIP")
        self.assertTrue(checks.items[-1]["evidence"].startswith("blocked: ref-R"), checks.items[-1]["evidence"])
        self.assertNotIn("lever_arm", rep)


if __name__ == "__main__":
    unittest.main()
