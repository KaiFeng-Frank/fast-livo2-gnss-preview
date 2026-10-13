# SPDX-License-Identifier: BSD-3-Clause
"""gnss_time_calib.py methods on synthetic data with a known answer.

I10 (TEST_PLAN §2), tool part: the same data with the GNSS time tags shifted by ±50 ms, analysed with one fixed
search centre, must move the estimated offset by the injection to < 2 ms (differential, noise cancels); over noise
seeds the absolute error is < 2 ms in RMS and never outside 3 reported sigma. The real-data I10 run is blocked on the
ref-R replay (the development set is on the remote since A26). Also: drift in ppm; A18 segmentation (a persistent
> 50 ms step splits, isolated jumped windows do not, a short trailing run becomes a low-confidence segment, a short
run confirmed by a method-1 step splits); sigma of short segments; insufficient excitation; the DTW cross-check;
method 1 (host-clock record-time statistics, and the estimator-clock prior that cancels host-clock steps) and the
A18 fallbacks."""
import contextlib
import io
import json
import math
import os
import tempfile
import unittest

import numpy as np

import _path  # noqa: F401
import synth
import gnss_time_calib as tc
from fl2g import gnsstime, timecalib_yaml

LEAP = 18
T0_GPST = gnsstime.gpst(2360, 300000.0)     # a 2025 epoch (leap 18 s)


def make_case(offset_fn, seed=0, noise=0.03, t_len=400.0, lever=synth.LEVER, speed=None):
    """GNSS epochs every 0.1 s in GPST; est time = gpst - leap + offset_fn(gpst). Returns tool inputs + truth."""
    rng = np.random.RandomState(seed)
    t_gpst = T0_GPST + np.arange(0.0, t_len, 0.1)
    t_est_of_gnss = t_gpst - LEAP + offset_fn(t_gpst)
    e0 = t_est_of_gnss[0] - 5.0
    e1 = t_est_of_gnss[-1] + 5.0
    kw = {} if speed is None else {"speed": speed}
    m = synth.Motion(e0, e1, **kw)
    times = synth.estimator_times(e0, e1, seed=seed + 1)
    rows = m.tum_rows(times)
    s_g = m.antenna_speed(t_est_of_gnss, lever) + rng.normal(0.0, noise, t_gpst.size)
    return rows, t_gpst, s_g, m


def run_tool(rows, t_gpst, s_g, offset0, confirm_steps=()):
    t_a, a = tc.antenna_track("tum", [list(r) for r in rows], synth.LEVER)
    t_e, s_e = tc.speed_from_track(t_a, a)
    return tc.calibrate(t_e, s_e, t_gpst, s_g, LEAP, offset0, 60.0, 30.0, confirm_steps=confirm_steps)


def windows(offsets, dt=30.0, peak=0.95, sigma_pw=0.012):
    """Used windows as xcorr_windows makes them (mid-times dt apart) for the segmentation logic."""
    return [{"used": True, "offset": float(o), "t_mid_gpst": T0_GPST + dt * k, "peak": peak, "sigma_pw": sigma_pw}
            for k, o in enumerate(offsets)]


class TimeCalibMethod2(unittest.TestCase):
    def run_case(self, off, drift_ppm=0.0, offset0_err=0.3, seed=0, noise=0.03):
        tref_true = T0_GPST + 200.0
        fn = lambda t: off + drift_ppm * 1e-6 * (t - tref_true)
        rows, t_gpst, s_g, _ = make_case(fn, seed=seed, noise=noise)
        segs, wins, dtw = run_tool(rows, t_gpst, s_g, off + offset0_err)
        self.assertEqual(len(segs), 1, segs)
        return segs[0], fn, dtw

    def test_noise_free_is_unbiased(self):
        for off in (+0.050, -0.050):
            s, fn, dtw = self.run_case(off, noise=0.0, offset0_err=0.3 if off > 0 else -0.4)
            self.assertLess(abs(s["offset_s"] - fn(s["t_ref_gpst_s"])), 0.0005, s)
            self.assertLess(abs(s["drift_ppm"]), 2.0, s)
            self.assertLess(abs(dtw["offset_s"] - s["offset_s"]), 0.02, dtw)

    def test_noisy_offset_within_reported_sigma(self):
        # 3 cm/s white Doppler-speed noise at 10 Hz: per-window precision ~4 ms (Cramer-Rao for this motion);
        # the estimate must lie within 3 reported sigma and the sigma must be honest (>= 1 ms, < 10 ms)
        for seed, off in ((0, 0.050), (3, -0.050), (5, 0.012)):
            s, fn, dtw = self.run_case(off, seed=seed, offset0_err=0.3 if off > 0 else -0.4)
            err = s["offset_s"] - fn(s["t_ref_gpst_s"])
            self.assertLess(abs(err), 3.0 * s["sigma_toff_s"], (err, s["sigma_toff_s"]))
            self.assertLess(abs(err), 0.005, err)
            self.assertTrue(0.001 <= s["sigma_toff_s"] < 0.010, s["sigma_toff_s"])
            self.assertGreater(s["peak_corr"], 0.9)
            self.assertFalse(s["tdcp_paused"])
            self.assertGreater(s["peak_width_sigma_ms"], 0.0)
            self.assertTrue(dtw["consistent"], dtw)

    def test_i10_injection_moves_the_estimate_by_the_injection(self):
        # I10: the GNSS time tags of the same data are relabelled t_gpst + inj (the receiver clock "late" by inj):
        # true offset becomes off - inj. The search centre offset0 is one fixed prior for every run and is NOT told
        # about the injection, so the correlator really sees shifted input.
        off = 0.030
        rows, t_gpst, s_g, _ = make_case(lambda t: off + 0.0 * t, seed=13)
        offset0 = off + 0.3
        base, wb, _ = run_tool(rows, t_gpst, s_g, offset0)
        lag_b = float(np.median([w["lag"] for w in wb if w.get("used")]))
        for inj in (+0.050, -0.050):
            segs, wi, _ = run_tool(rows, t_gpst + inj, s_g, offset0)
            self.assertEqual(len(segs), 1, segs)
            lag_i = float(np.median([w["lag"] for w in wi if w.get("used")]))
            # the raw correlation lags moved by +inj (offset = offset0 - lag): the input did change ...
            self.assertLess(abs((lag_i - lag_b) - inj), 0.002, (lag_i, lag_b, inj))
            # ... and the calibrated offset moved by -inj to < 2 ms
            moved = segs[0]["offset_s"] - base[0]["offset_s"]
            self.assertLess(abs(moved + inj), 0.002, "moved %.5f s for an injection of %+.3f s" % (moved, inj))
            self.assertLess(abs(segs[0]["offset_s"] - (off - inj)), 3.0 * segs[0]["sigma_toff_s"])

    def test_absolute_error_statistics_over_seeds(self):
        # absolute accuracy is noise limited (~1.5 ms RMS for 400 s at 3 cm/s): a single seed can exceed 2 ms, so the
        # claim is statistical — RMS over 12 seeds < 2 ms, every seed inside 3 reported sigma, sigma not optimistic
        errs, zs = [], []
        for seed in range(12):
            off = 0.030 + 0.010 * (seed % 3 - 1)
            rows, t_gpst, s_g, _ = make_case(lambda t, o=off: o + 0.0 * t, seed=100 + seed, noise=0.03)
            segs, _, _ = run_tool(rows, t_gpst, s_g, off + 0.3)
            self.assertEqual(len(segs), 1, (seed, segs))
            e = segs[0]["offset_s"] - off
            errs.append(e)
            zs.append(e / segs[0]["sigma_toff_s"])
        errs, zs = np.array(errs), np.array(zs)
        self.assertLess(float(np.max(np.abs(zs))), 3.0, zs)
        self.assertLess(float(np.sqrt(np.mean(zs ** 2))), 1.0, zs)
        self.assertLess(float(np.sqrt(np.mean(errs ** 2))), 0.002, errs)

    def test_drift(self):
        s, fn, _ = self.run_case(0.020, drift_ppm=40.0, noise=0.0, seed=5)
        self.assertLess(abs(s["drift_ppm"] - 40.0), 2.0, s["drift_ppm"])
        self.assertLess(abs(s["offset_s"] - fn(s["t_ref_gpst_s"])), 0.0005)
        s, fn, _ = self.run_case(0.020, drift_ppm=40.0, seed=5)
        self.assertLess(abs(s["drift_ppm"] - 40.0), 25.0, s["drift_ppm"])

    def test_step_splits_segments(self):
        t_step = T0_GPST + 190.0
        fn = lambda t: np.where(t < t_step, 0.030, 0.110)
        rows, t_gpst, s_g, _ = make_case(fn, seed=7)
        segs, wins, dtw = run_tool(rows, t_gpst, s_g, 0.07)
        self.assertEqual(len(segs), 2, [(s["t_from_gpst_s"], s["offset_s"]) for s in segs])
        self.assertLess(abs(segs[0]["offset_s"] - 0.030), 0.005, segs[0])
        self.assertLess(abs(segs[1]["offset_s"] - 0.110), 0.005, segs[1])
        self.assertLess(abs(segs[1]["t_from_gpst_s"] - t_step), 30.0)
        self.assertFalse(segs[0]["short_segment"] or segs[1]["short_segment"])
        text = timecalib_yaml.dumps("Synthetic", LEAP, segs, span=(float(t_gpst[0]), float(t_gpst[-1])))
        back = timecalib_yaml.loads(text)
        self.assertEqual(len(back["segments"]), 2)
        self.assertEqual(back["segments"][1]["offset_s"], segs[1]["offset_s"])

    def test_low_excitation_gives_no_xcorr_segment(self):
        const = lambda t: 1.0 + 0.0 * t
        rows, t_gpst, s_g, _ = make_case(lambda t: 0.05 + 0.0 * t, seed=9, speed=const)
        segs, wins, dtw = run_tool(rows, t_gpst, s_g, 0.05)
        self.assertEqual(segs, [], "constant speed must not produce an xcorr calibration")
        self.assertEqual([w for w in wins if w.get("used")], [])

    def test_wrong_lever_sign_still_recovers_offset(self):
        # the |v| correlation is dominated by the speed profile; the lever arm only matters on turns
        rows, t_gpst, s_g, _ = make_case(lambda t: 0.04 + 0.0 * t, seed=11, lever=-synth.LEVER, noise=0.0)
        segs, wins, dtw = run_tool(rows, t_gpst, s_g, 0.0)
        self.assertLess(abs(segs[0]["offset_s"] - 0.04), 0.005)


class Segmentation(unittest.TestCase):
    """A18 "台阶(>50 ms 持续)= 新段" on window offsets with a known answer."""

    def noisy(self, levels, seed=0):
        rng = np.random.RandomState(seed)
        return [lv + rng.normal(0.0, 0.002) for lv in levels]

    def test_single_jumped_window_is_an_outlier_not_a_segment(self):
        lv = [0.030] * 12
        lv[6] = 0.110                                   # one spurious correlation peak, neighbours back at 30 ms
        w = windows(self.noisy(lv))
        segs = tc.fit_segments(w)
        self.assertEqual(len(segs), 1, [(s["offset_s"], s["n_windows"]) for s in segs])
        self.assertTrue(w[6].get("isolated_jump"))
        self.assertEqual(segs[0]["n_windows"], 11)
        self.assertEqual(segs[0]["n_windows_isolated"], 1)
        self.assertLess(abs(segs[0]["offset_s"] - 0.030), 0.003)
        self.assertFalse(segs[0]["tdcp_paused"])
        self.assertLess(segs[0]["sigma_toff_s"], 0.010)

    def test_two_window_jump_in_the_middle_is_dropped(self):
        lv = [0.030] * 5 + [0.110, 0.112] + [0.030] * 5
        segs = tc.fit_segments(windows(self.noisy(lv, 1)))
        self.assertEqual(len(segs), 1)
        self.assertEqual(segs[0]["n_windows_isolated"], 2)

    def test_persistent_step_splits(self):
        lv = [0.030] * 6 + [0.110] * 4
        segs = tc.fit_segments(windows(self.noisy(lv, 2)))
        self.assertEqual(len(segs), 2)
        self.assertEqual([s["n_windows"] for s in segs], [6, 4])
        self.assertAlmostEqual(segs[1]["t_from_gpst_s"], T0_GPST + 30.0 * 5.5)
        self.assertFalse(segs[1]["tdcp_paused"])

    def test_short_trailing_run_is_a_low_confidence_segment(self):
        # nothing after the jump restores the old level: a segment, but it cannot measure its own scatter; its sigma
        # is floored at 50 ms, which is <= 0.1 s, so TDCP runs (A53: the TDCP R carries (|dv_pair| sigma_toff)^2)
        lv = [0.030] * 6 + [0.110, 0.111]
        segs = tc.fit_segments(windows(self.noisy(lv, 3)))
        self.assertEqual(len(segs), 2)
        self.assertFalse(segs[0]["short_segment"])
        self.assertTrue(segs[1]["short_segment"])
        self.assertGreaterEqual(segs[1]["sigma_toff_s"], 0.05)
        self.assertFalse(segs[1]["tdcp_paused"])
        self.assertLess(abs(segs[1]["offset_s"] - 0.1105), 0.004)

    def test_short_segment_sigma_takes_the_peak_width_when_larger(self):
        for pw, paused in ((0.080, False), (0.150, True)):      # A53: TDCP paused only above sigma 0.1 s
            w = windows(self.noisy([0.030] * 6, 4)) + windows([0.530], sigma_pw=pw)
            w[-1]["t_mid_gpst"] = T0_GPST + 30.0 * 6
            segs = tc.fit_segments(w)
            self.assertEqual(len(segs), 2)
            self.assertAlmostEqual(segs[1]["sigma_toff_s"], pw)
            self.assertIs(segs[1]["tdcp_paused"], paused)

    def test_drift_from_the_method1_prior(self):
        # three noisy windows with a spurious trend (Grass02 in the P4 run: 72.6 / 36.6 / -13.9 ms, 30 s apart): a free
        # line fits the noise as ~-1400 ppm; with the method-1 drift the windows only fit the level
        offs = [0.0726, 0.0366, -0.0139]
        free = tc.fit_segments(windows(offs, sigma_pw=0.045))
        self.assertLess(free[0]["drift_ppm"], -500.0)
        self.assertEqual(free[0]["drift_source"], "window_fit")
        seg = tc.fit_segments(windows(offs, sigma_pw=0.045), drift_ppm_at=lambda t: 0.6)[0]
        self.assertAlmostEqual(seg["drift_ppm"], 0.6, places=9)
        self.assertEqual(seg["drift_source"], "method1_prior")
        self.assertAlmostEqual(seg["offset_s"], float(np.mean(offs)), delta=1e-4)
        self.assertAlmostEqual(seg["window_scatter_ms"], 1e3 * float(np.std(offs, ddof=1)), delta=0.1)
        self.assertGreaterEqual(seg["sigma_toff_s"], float(np.std(offs, ddof=1)) - 1e-4)
        # the prior's piece is chosen by time (a header-clock step splits it)
        t = T0_GPST + np.arange(0.0, 300.0, 0.1)
        at = tc.prior_drift_at({"series": (t, np.where(t < T0_GPST + 150.0, 0.02 + 5e-6 * (t - T0_GPST), -0.10)),
                                "steps": [(T0_GPST + 150.0, -0.12)]})
        self.assertAlmostEqual(at(T0_GPST + 20.0), 5.0, delta=0.01)
        self.assertAlmostEqual(at(T0_GPST + 200.0), 0.0, delta=0.01)
        self.assertIsNone(tc.prior_drift_at(None))

    def test_a_jump_within_three_sigma_is_not_a_step(self):
        # imprecise windows (a band-limited speed signal measures the offset to tens of ms): a 170 ms difference
        # against a window of sigma 80 ms (3 sigma = 240 ms) is noise, not an A18 step
        w = windows(self.noisy([0.030] * 6, 4)) + windows([0.200], sigma_pw=0.080)
        w[-1]["t_mid_gpst"] = T0_GPST + 30.0 * 6
        segs = tc.fit_segments(w)
        self.assertEqual(len(segs), 1)
        # noisy windows around one level (sigma 40 ms each, scatter ~40 ms) stay one segment
        rng = np.random.RandomState(9)
        w = windows(list(0.050 + rng.normal(0.0, 0.040, 12)), sigma_pw=0.040)
        segs = tc.fit_segments(w)
        self.assertEqual(len(segs), 1, [(round(x["offset_s"], 3), x["n_windows"]) for x in segs])
        self.assertEqual(segs[0]["n_windows_isolated"], 0)

    def test_short_run_confirmed_by_a_method1_step(self):
        lv = [0.030] * 5 + [0.110, 0.112] + [0.300] * 4
        t_ab = T0_GPST + 30.0 * 4.5                      # between the last 30 ms window and the first 110 ms one
        for steps, n_expected in (((), 2), (((t_ab, 0.080),), 3), (((t_ab, -0.080),), 2), (((t_ab + 200.0, 0.080),), 2)):
            segs = tc.fit_segments(windows(self.noisy(lv, 5)), confirm_steps=steps)
            self.assertEqual(len(segs), n_expected, (steps, [(round(s["offset_s"], 3), s["n_windows"]) for s in segs]))
            if n_expected == 3:
                self.assertTrue(segs[1]["short_segment"])
                self.assertGreaterEqual(segs[1]["sigma_toff_s"], 0.05)
                self.assertFalse(segs[1]["tdcp_paused"])

    def test_leading_short_group_is_low_confidence(self):
        segs = tc.fit_segments(windows(self.noisy([0.200] + [0.030] * 8, 6)))
        self.assertEqual(len(segs), 2)
        self.assertTrue(segs[0]["short_segment"])
        self.assertEqual(segs[0]["sigma_toff_s"], tc.SIGMA_FALLBACK)
        self.assertFalse(segs[1]["short_segment"])


class TimeCalibMethod1(unittest.TestCase):
    def test_record_time_median_and_step(self):
        rng = np.random.RandomState(4)
        t_gpst = T0_GPST + np.arange(0.0, 400.0, 0.1)
        t_utc = np.array([gnsstime.gpst_to_utc(t) for t in t_gpst])
        lat = 0.040 + rng.exponential(0.004, t_gpst.size)
        lat[(t_gpst - T0_GPST) > 200.0] += 0.080                     # persistent host-clock step
        spikes = rng.choice(t_gpst.size, 20, replace=False)
        lat[spikes] += 0.3                                            # isolated latency spikes must not split
        st = tc.record_time_stats(t_gpst, t_utc + lat)
        self.assertEqual(len(st["steps"]), 1, st["steps"])
        self.assertLess(abs(st["steps"][0][0] - 200.0), 5.0)
        self.assertLess(abs(st["steps"][0][1] - 0.080), 0.01)
        self.assertEqual(st["leap_s"], 18)

    def test_estimator_clock_prior_cancels_host_steps_keeps_header_steps(self):
        # host clock: +20 ms, stepping to +100 ms at 150 s; estimator (Livox header) clock: -50 ms, stepping to
        # -170 ms at 300 s; GNSS latency 60 ms + exponential jitter; IMU delivered in 100 ms bursts (latency 5..105 ms)
        rng = np.random.RandomState(8)
        u0 = gnsstime.gpst_to_utc(T0_GPST)
        host = lambda u: np.where(u - u0 < 150.0, 0.020, 0.100)
        dev = lambda u: np.where(u - u0 < 300.0, -0.050, -0.170)
        t_gpst = T0_GPST + np.arange(0.0, 400.0, 0.1)
        u_g = t_gpst - LEAP
        rec_g = u_g + 0.060 + rng.exponential(0.004, u_g.size) + host(u_g)
        u_i = u0 + np.arange(0.0, 400.0, 0.005)
        rec_i = np.ceil((u_i - u0) / 0.1) * 0.1 + u0 + 0.005 + host(u_i)
        hdr_i = u_i + dev(u_i)
        m1 = tc.record_time_stats(t_gpst, rec_g)
        self.assertEqual(len(m1["steps"]), 1)                                # the host step shows in method 1 ...
        pr = tc.estimator_clock_prior(t_gpst, rec_g, hdr_i, rec_i)
        self.assertEqual(len(pr["steps"]), 1, pr["steps"])                   # ... but not in the estimator-clock prior
        self.assertLess(abs(pr["steps"][0][0] - (T0_GPST + 300.0)), 10.0)
        self.assertLess(abs(pr["steps"][0][1] + 0.120), 0.010)
        segs = tc.prior_segments(pr)
        self.assertEqual(len(segs), 2)
        # upper bound = header-clock offset + GNSS latency floor - IMU latency floor (~60 - 10 ms); prior = ub - 75 ms
        for s, dev_off in zip(segs, (-0.050, -0.170)):
            self.assertLess(abs(s["offset_s"] - (dev_off + 0.050 - 0.5 * tc.LAT_G_MAX)), 0.006, s)
            self.assertEqual((s["method"], s["sigma_toff_s"], s["tdcp_paused"]), ("record_time", 0.5 * tc.LAT_G_MAX, False))
            self.assertLess(abs(s["drift_ppm"]), 20.0)
        # the true offset (header clock - UTC) lies within 1 sigma of the prior: [ub - LAT_G_MAX, ub] covers latency 0..150 ms
        self.assertLess(abs(segs[0]["offset_s"] - (-0.050)), segs[0]["sigma_toff_s"])

    def test_estimator_clock_prior_needs_imu(self):
        t_gpst = T0_GPST + np.arange(0.0, 100.0, 0.1)
        self.assertIsNone(tc.estimator_clock_prior(t_gpst, t_gpst - LEAP + 0.05, [], []))

    def test_prior_segments_write_and_read_back(self):
        # the method-1 file (no usable method 2): record_time, sigma 75 ms, TDCP running (A53)
        t = T0_GPST + np.arange(0.0, 200.0, 0.1)
        segs = tc.prior_segments({"series": (t, np.full(t.size, 0.0363)), "steps": []})
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, "x.yaml")
            timecalib_yaml.write(p, "Dark01", 18, segs, span=(T0_GPST, T0_GPST + 200.0))
            with open(p) as f:
                back = timecalib_yaml.loads(f.read())
        s = back["segments"][0]
        self.assertEqual((s["method"], s["sigma_toff_s"], s["tdcp_paused"]), ("record_time", 0.075, False))
        self.assertAlmostEqual(s["offset_s"], 0.0363 - 0.075, places=12)


def xseg(t_from, sigma, offset=0.030):
    return {"t_from_gpst_s": t_from, "offset_s": offset, "drift_ppm": 0.0, "t_ref_gpst_s": T0_GPST + 50.0, "sigma_toff_s": sigma,
            "tdcp_paused": tc.tdcp_paused_for("xcorr", sigma), "method": "xcorr"}


class Fallbacks(unittest.TestCase):
    """What goes into the A18 file (AMENDMENTS A53): per span the more precise of method 2 and the method-1 prior;
    the method-1 prior when method 2 gives nothing; method config (the sequence config's values, TDCP paused) only
    without a method-1 prior; TDCP paused only when neither method calibrated to sigma <= 0.1 s."""

    def prior(self, steps=()):
        t = T0_GPST + np.arange(0.0, 300.0, 0.1)
        ub = np.full(t.size, 0.095)
        for t_s, size in steps:
            ub = np.where(t >= t_s, ub + size, ub)
        return {"series": (t, ub), "steps": list(steps)}

    def test_a53_rule(self):
        self.assertFalse(tc.tdcp_paused_for("record_time", 0.075))
        self.assertFalse(tc.tdcp_paused_for("xcorr", 0.004))
        self.assertFalse(tc.tdcp_paused_for("xcorr", 0.100))
        self.assertTrue(tc.tdcp_paused_for("xcorr", 0.101))
        self.assertTrue(tc.tdcp_paused_for("config", 0.05))

    def test_method2_segments_win(self):
        segs, status, blocked = tc.choose_segments(True, [xseg(0.0, 0.004)], self.prior(), config_offset=0.1)
        self.assertEqual((status, blocked), ("ran", None))
        self.assertEqual([(s["method"], s["sigma_toff_s"], s["tdcp_paused"]) for s in segs], [("xcorr", 0.004, False)])

    def test_imprecise_method2_keeps_the_prior(self):
        # tool_version 2 on Grass02: two windows with peaks ~0.5 gave sigma 533 ms > the prior's 75 ms: the prior stays
        segs, status, blocked = tc.choose_segments(True, [xseg(0.0, 0.533)], self.prior())
        self.assertIsNone(blocked)
        self.assertEqual([(s["method"], s["tdcp_paused"]) for s in segs], [("record_time", False)])
        self.assertIn("less precise than the method-1 prior", status)
        self.assertTrue(segs[0]["replaces_method2_segment"])

    def test_merge_per_span(self):
        # method 2: precise / coarse / precise; the coarse middle span takes the prior, the others stay
        t1, t2 = T0_GPST + 100.0, T0_GPST + 200.0
        x = [xseg(0.0, 0.004, 0.030), xseg(t1, 0.300, 0.500), xseg(t2, 0.006, 0.031)]
        segs, status, blocked = tc.choose_segments(True, x, self.prior())
        self.assertEqual([s["method"] for s in segs], ["xcorr", "record_time", "xcorr"])
        self.assertEqual([s["t_from_gpst_s"] for s in segs], [0.0, t1, t2])
        self.assertTrue(all(not s["tdcp_paused"] for s in segs))
        self.assertIn("1 of 3 method-2 segments", status)
        # two coarse spans in a row are covered by ONE piece of the prior (no duplicate segment)
        x = [xseg(0.0, 0.004), xseg(t1, 0.300), xseg(t2, 0.200)]
        segs, _, _ = tc.choose_segments(True, x, self.prior())
        self.assertEqual([(s["method"], s["t_from_gpst_s"]) for s in segs], [("xcorr", 0.0), ("record_time", t1)])
        # a prior with a header-clock step inside the replaced span contributes both of its pieces, cut to the span
        t_step = T0_GPST + 150.0
        segs, _, _ = tc.choose_segments(True, [xseg(0.0, 0.004), xseg(t1, 0.300), xseg(t2, 0.006)], self.prior([(t_step, -0.120)]))
        self.assertEqual([(s["method"], s["t_from_gpst_s"]) for s in segs],
                         [("xcorr", 0.0), ("record_time", t1), ("record_time", t_step), ("xcorr", t2)])
        self.assertLess(abs((segs[2]["offset_s"] - segs[1]["offset_s"]) + 0.120), 1e-9)
        text = timecalib_yaml.dumps("Synthetic", LEAP, segs, span=(T0_GPST, T0_GPST + 300.0))
        self.assertEqual(len(timecalib_yaml.loads(text)["segments"]), 4)

    def test_no_method2_window_uses_the_prior(self):
        segs, status, blocked = tc.choose_segments(True, [], self.prior(), config_offset=-0.0123, no_window_reason="GNSS velocity covers 0% of the window")
        self.assertIsNone(blocked)
        self.assertEqual([(s["method"], s["sigma_toff_s"], s["tdcp_paused"]) for s in segs], [("record_time", 0.075, False)])
        self.assertIn("GNSS velocity covers", status)

    def test_without_a_prior_config_values_are_kept_with_tdcp_paused(self):
        segs, status, blocked = tc.choose_segments(True, [], None, config_offset=-0.0123, config_drift_ppm=2.5, config_t_ref=T0_GPST)
        self.assertIsNone(blocked)
        s = segs[0]
        self.assertEqual((s["method"], s["offset_s"], s["drift_ppm"], s["t_ref_gpst_s"]), ("config", -0.0123, 2.5, T0_GPST))
        self.assertEqual((s["sigma_toff_s"], s["tdcp_paused"]), (0.05, True))

    def test_nothing_at_all_is_blocked(self):
        segs, status, blocked = tc.choose_segments(True, [], None)
        self.assertEqual(segs, [])
        self.assertTrue(blocked.startswith("blocked:"), blocked)

    def test_coarse_method2_without_a_prior_pauses_tdcp(self):
        segs, status, blocked = tc.choose_segments(True, [xseg(0.0, 0.3)], None)
        self.assertEqual((segs[0]["method"], segs[0]["tdcp_paused"]), ("xcorr", True))

    def test_no_replay_uses_the_prior_or_blocks(self):
        segs, status, blocked = tc.choose_segments(False, [], self.prior())
        self.assertIsNone(blocked)
        self.assertEqual([(s["method"], s["tdcp_paused"]) for s in segs], [("record_time", False)])
        self.assertAlmostEqual(segs[0]["offset_s"], 0.095 - 0.5 * tc.LAT_G_MAX)
        self.assertTrue(status.startswith("blocked: ref-R"))
        segs, status, blocked = tc.choose_segments(False, [], None)
        self.assertEqual(segs, [])
        self.assertTrue(blocked.startswith("blocked:"))


class HorizontalSpeedSignal(unittest.TestCase):
    """Method-2 signal (tool_version 3): horizontal speed from vector-smoothed velocities on both sides. A GNSS velocity
    with Grass02-like noise (white, 0.4 m/s horizontal, 1.6 m/s vertical per epoch) defeats the tool_version-2 signal
    (3-D norm, norms smoothed) and not the new one."""

    def series(self, seed=0, off=0.050, sig_h=0.4, sig_v=1.6, gap=None):
        rng = np.random.RandomState(seed)
        t_gpst = T0_GPST + np.arange(0.0, 300.0, 0.1)
        t_est_g = t_gpst - LEAP + off
        m = synth.Motion(t_est_g[0] - 5.0, t_est_g[-1] + 5.0)
        yaw_w = 0.7                                   # W is rotated against ENU and not level: norms are frame-free
        c, s_ = math.cos(yaw_w), math.sin(yaw_w)
        tilt = np.array([[1.0, 0.0, 0.0], [0.0, math.cos(0.05), -math.sin(0.05)], [0.0, math.sin(0.05), math.cos(0.05)]])
        Rw = tilt @ np.array([[c, -s_, 0.0], [s_, c, 0.0], [0.0, 0.0, 1.0]])
        te = synth.estimator_times(m.t[0], m.t[-1], seed=seed + 1)
        vw = m.antenna_velocity(te) @ Rw.T
        up_w = Rw @ np.array([0.0, 0.0, 1.0])
        vg = m.antenna_velocity(t_est_g) + np.column_stack([rng.normal(0, sig_h, t_gpst.size), rng.normal(0, sig_h, t_gpst.size),
                                                            rng.normal(0, sig_v, t_gpst.size)])
        if gap is not None:
            keep = np.ones(t_gpst.size, dtype=bool)
            for a, b in gap:
                keep &= ~((t_gpst >= T0_GPST + a) & (t_gpst < T0_GPST + b))
            t_gpst, vg = t_gpst[keep], vg[keep]
        return te, vw, up_w, t_gpst, vg, off

    def test_new_signal_recovers_the_offset_where_the_old_one_fails(self):
        te, vw, up_w, tg, vg, off = self.series()
        # tool_version 2: 3-D norms, smoothed as norms (smooth_s 0.1)
        s_old = np.linalg.norm(vg, axis=1)
        e_old = np.linalg.norm(vw, axis=1)
        segs_old, wins_old, _ = tc.calibrate(te, e_old, tg, s_old, LEAP, off + 0.2, 60.0, 30.0)
        peaks_old = [w["peak"] for w in wins_old if "peak" in w]
        self.assertLess(float(np.median(peaks_old)), 0.6, peaks_old)
        # tool_version 3
        tE, sE, _ = tc.horizontal_speed(te, vw, up_w)
        tG, sG, vG = tc.horizontal_speed(tg, vg, np.array([0.0, 0.0, 1.0]))
        segs, wins, dtw = tc.calibrate(tE, sE, tG, sG, LEAP, off + 0.2, 60.0, 30.0, smooth_s=0.0, gnss_valid=vG)
        self.assertEqual(len(segs), 1, segs)
        self.assertGreater(min(w["peak"] for w in wins if w.get("used")), 0.9)
        self.assertLess(abs(segs[0]["offset_s"] - off), 3.0 * segs[0]["sigma_toff_s"])
        self.assertLess(abs(segs[0]["offset_s"] - off), 0.015)
        self.assertFalse(segs[0]["tdcp_paused"])

    def test_i10_injection_on_the_new_signal(self):
        # I10 (tool part) on the tool_version-3 signal: relabelling the GNSS epochs by +-50 ms moves the calibrated
        # offset by -+50 ms to < 2 ms (same noise realisation: the differential is exact up to resampling)
        te, vw, up_w, tg, vg, off = self.series(seed=8)
        tE, sE, _ = tc.horizontal_speed(te, vw, up_w)
        base = None
        for inj in (0.0, +0.050, -0.050):
            tG, sG, vG = tc.horizontal_speed(tg + inj, vg, np.array([0.0, 0.0, 1.0]))
            segs, _, _ = tc.calibrate(tE, sE, tG, sG, LEAP, off + 0.2, 60.0, 30.0, smooth_s=0.0, gnss_valid=vG,
                                      drift_ppm_at=lambda t: 0.0)
            self.assertEqual(len(segs), 1, segs)
            if base is None:
                base = segs[0]["offset_s"]
                continue
            self.assertLess(abs((segs[0]["offset_s"] - base) + inj), 0.002, (segs[0]["offset_s"], base, inj))

    def test_vertical_from_state_trace_or_velocity_plane(self):
        te, vw, up_w, tg, vg, off = self.series(seed=3)
        up, src = tc.vertical_in_w("", vw)
        self.assertLess(np.linalg.norm(up - up_w), 1e-6, (up, up_w))
        self.assertIn("plane", src)
        with tempfile.TemporaryDirectory() as d:
            g = -9.81 * up_w
            row = ["0", "2", float(1000.0).hex()] + [float(x).hex() for x in np.eye(3).ravel()] + ["0x0p+0"] * 12 + [float(x).hex() for x in g]
            with open(os.path.join(d, "state_trace.txt"), "w") as f:
                f.write("# header\n" + " ".join(row) + "\n")
            up2, src2 = tc.vertical_in_w(d, None)
        self.assertLess(np.linalg.norm(up2 - up_w), 1e-12)
        self.assertIn("state_trace", src2)

    def test_gaps_are_never_correlated(self):
        # GNSS velocity only in short runs (Occlusion03-like): no window reaches the coverage, none is used
        runs = [(k * 20.0 + 6.0, k * 20.0 + 20.0) for k in range(15)]
        te, vw, up_w, tg, vg, off = self.series(seed=5, sig_h=0.05, sig_v=0.1, gap=runs)
        tE, sE, _ = tc.horizontal_speed(te, vw, up_w)
        tG, sG, vG = tc.horizontal_speed(tg, vg, np.array([0.0, 0.0, 1.0]))
        segs, wins, _ = tc.calibrate(tE, sE, tG, sG, LEAP, off, 60.0, 30.0, smooth_s=0.0, gnss_valid=vG)
        self.assertEqual(segs, [])
        self.assertTrue(wins and all(w["gnss_coverage"] < tc.MIN_COVERAGE for w in wins), [w["gnss_coverage"] for w in wins])
        self.assertTrue(all("covers" in w["reason"] for w in wins))

    def test_smooth_drift_is_robust(self):
        t = T0_GPST + np.arange(0.0, 60.0, 0.1)
        d = 40.0 + 0.02 * (t - t[0]) + np.random.RandomState(2).normal(0, 0.8, t.size)
        d[100] += 50.0                                                     # one wild epoch
        ds = tc.smooth_drift(t, d)
        truth = 40.0 + 0.02 * (t - t[0])
        self.assertLess(float(np.max(np.abs(ds - truth)[20:-20])), 0.5)


class MainEndToEnd(unittest.TestCase):
    """main() with the bag reader and the Doppler-velocity series replaced by synthetic data: trajectory file ->
    method 2 (horizontal vector-smoothed speeds) -> A18 file (span-checked), and the fallbacks: low excitation keeps
    the method-1 prior (TDCP running, A53); without the IMU header clock the config value (TDCP paused) or blocked."""

    def run_main(self, speed=None, extra=(), with_imu=True, noise_v=1.6):
        off = 0.030
        rng = np.random.RandomState(21)
        t_gpst = T0_GPST + np.arange(0.0, 400.0, 0.1)
        t_est_g = t_gpst - LEAP + off
        kw = {} if speed is None else {"speed": speed}
        m = synth.Motion(t_est_g[0] - 5.0, t_est_g[-1] + 5.0, **kw)
        rows = m.tum_rows(synth.estimator_times(m.t[0], m.t[-1], seed=22))
        vg = m.antenna_velocity(t_est_g) + np.column_stack([rng.normal(0, 0.3, t_gpst.size), rng.normal(0, 0.3, t_gpst.size),
                                                            rng.normal(0, noise_v, t_gpst.size)])
        u = np.array([gnsstime.gpst_to_utc(t) for t in t_gpst])
        eps = [{"t_gpst": float(t), "t_record": float(x) + 0.045} for t, x in zip(t_gpst, u)]
        ti = u[0] + np.arange(0.0, u[-1] - u[0], 0.005)
        imu = np.column_stack([ti + off, ti + 0.010]) if with_imu else np.zeros((0, 2))   # header = UTC + 30 ms
        orig = (tc.read_inputs, tc.doppler_velocity_series)
        tc.read_inputs = lambda bag, topic=None: (eps, [], [], [], {}, imu)
        tc.doppler_velocity_series = lambda *a: (t_gpst, vg, {"synthetic": True})
        try:
            with tempfile.TemporaryDirectory() as d:
                with open(os.path.join(d, "traj_W.tum"), "w") as f:
                    for r in rows:
                        f.write(" ".join("%.9f" % v for v in r) + "\n")
                out = os.path.join(d, "tc", "Synthetic.yaml")
                with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
                    code = tc.main(["dummy.bag", d, out, "--seq", "Synthetic"] + list(extra))
                text = None
                if os.path.exists(out):
                    with open(out) as f:
                        text = f.read()
                with open(os.path.splitext(out)[0] + ".json") as f:
                    rep = json.load(f)
        finally:
            tc.read_inputs, tc.doppler_velocity_series = orig
        return code, text, rep, off

    def test_method2_writes_a_calibration(self):
        code, text, rep, off = self.run_main()
        self.assertEqual(code, 0)
        back = timecalib_yaml.loads(text)
        seg = back["segments"][0]
        self.assertEqual((seg["method"], seg["tdcp_paused"]), ("xcorr", False))
        self.assertLess(abs(seg["offset_s"] - off), 3.0 * seg["sigma_toff_s"])
        self.assertLess(seg["sigma_toff_s"], 0.075)
        self.assertEqual(rep["method2_status"], "ran")
        self.assertIn("plane", rep["method2_inputs"]["up_source"])
        # the method-1 prior: upper bound = GNSS record floor - IMU record floor = 45 - (10 - 30) = 65 ms
        self.assertLess(abs(rep["method1_estimator_clock_prior"]["upper_bound_s"] - 0.065), 0.002)

    def test_low_excitation_keeps_the_method1_prior(self):
        code, text, rep, off = self.run_main(speed=lambda t: 1.0 + 0.0 * t, noise_v=0.0)
        self.assertEqual(code, 0)
        seg = timecalib_yaml.loads(text)["segments"][0]
        self.assertEqual((seg["method"], seg["sigma_toff_s"], seg["tdcp_paused"]), ("record_time", 0.075, False))
        self.assertIn("no usable window", rep["method2_status"])

    def test_low_excitation_without_imu_keeps_the_config_value_or_blocks(self):
        code, text, rep, off = self.run_main(speed=lambda t: 1.0 + 0.0 * t, noise_v=0.0, with_imu=False)
        self.assertEqual(code, tc.EXIT_BLOCKED)
        self.assertIsNone(text)
        self.assertTrue(rep["status"].startswith("blocked:"))
        code, text, rep, off = self.run_main(speed=lambda t: 1.0 + 0.0 * t, noise_v=0.0, with_imu=False, extra=("--config-offset", "-0.0123"))
        self.assertEqual(code, 0)
        seg = timecalib_yaml.loads(text)["segments"][0]
        self.assertEqual((seg["method"], seg["offset_s"], seg["sigma_toff_s"], seg["tdcp_paused"]), ("config", -0.0123, 0.05, True))


class Plot(unittest.TestCase):
    def test_plot_is_written(self):
        rows, t_gpst, s_g, _ = make_case(lambda t: 0.05 + 0.0 * t, seed=0)
        segs, wins, dtw = run_tool(rows, t_gpst, s_g, 0.3)
        t_utc = np.array([gnsstime.gpst_to_utc(t) for t in t_gpst])
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, "x.png")
            ok = tc.write_plot(p, t_gpst, t_utc + 0.04, wins, segs)
            if ok:
                self.assertGreater(os.path.getsize(p), 1000)


class TrajectoryReaders(unittest.TestCase):
    def test_state_trace_hexfloat(self):
        R = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])      # yaw 90 deg
        p = np.array([1.5, -2.25, 0.125])
        vals = [7, 1, 1000.5] + list(R.ravel()) + list(p) + [0.1, 0.2, 0.3]
        line = " ".join(float(v).hex() if i >= 2 else str(v) for i, v in enumerate(vals))
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "state_trace.txt")
            with open(path, "w") as f:
                f.write(line + "\n")
            kind, rows = tc.load_traj(path)
        self.assertEqual(kind, "state_trace")
        t, a = tc.antenna_track(kind, rows, synth.LEVER)
        self.assertEqual(t[0], 1000.5)
        np.testing.assert_allclose(a[0], p + R @ synth.LEVER, atol=0, rtol=0)

    def test_tum_quaternion(self):
        yaw = 0.7
        rows = [[5.0, 1.0, 2.0, 3.0, 0.0, 0.0, np.sin(yaw / 2), np.cos(yaw / 2)]]
        t, a = tc.antenna_track("tum", rows, synth.LEVER)
        c, s = np.cos(yaw), np.sin(yaw)
        R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        np.testing.assert_allclose(a[0], np.array([1.0, 2.0, 3.0]) + R @ synth.LEVER, atol=1e-12)


class A80Options(unittest.TestCase):
    """The A80 additions (all off or inert by default): C/N0-based Doppler weights for receivers without std fields, a fixed
    position for bags without receiver_pvt, and the two method-2 window validity options."""

    def test_doppler_std_from_cn0_is_monotonic_and_clipped(self):
        v = [tc.dopp_std_from_cn0(c) for c in (10.0, 25.0, 35.0, 40.0, 50.0, 60.0)]
        self.assertTrue(all(a >= b for a, b in zip(v, v[1:])))
        self.assertAlmostEqual(tc.dopp_std_from_cn0(40.0), 0.3)
        self.assertEqual(tc.dopp_std_from_cn0(5.0), 3.0)
        self.assertEqual(tc.dopp_std_from_cn0(80.0), 0.1)

    def test_fixed_position_pvt_entries(self):
        from fl2g import geodesy
        xyz = geodesy.pos2ecef(math.radians(30.5), math.radians(114.3), 25.0)
        eps = [{"t_gpst": T0_GPST + 0.1 * k} for k in range(3)]
        pvt = tc.fixed_position_pvt(eps, xyz)
        self.assertEqual(len(pvt), 3)
        self.assertAlmostEqual(pvt[1]["lat"], 30.5, places=6)
        self.assertAlmostEqual(pvt[1]["lon"], 114.3, places=6)
        self.assertAlmostEqual(pvt[1]["h_ell"], 25.0, places=2)
        self.assertEqual(pvt[1]["t_gpst"], eps[1]["t_gpst"])
        self.assertEqual(pvt[1]["fix_type"], 3)

    def case(self, scale=1.0):
        rows, t_gpst, s_g, m = make_case(lambda t: 0.02 + 0.0 * t, seed=3)
        t_a, a = tc.antenna_track("tum", [list(r) for r in rows], synth.LEVER)
        t_e, s_e = tc.speed_from_track(t_a, a)
        return t_e, s_e * scale, t_gpst, s_g

    def test_options_default_off_change_nothing(self):
        t_e, s_e, t_g, s_g = self.case()
        a = tc.calibrate(t_e, s_e, t_g, s_g, LEAP, 0.02, 60.0, 30.0)
        b = tc.calibrate(t_e, s_e, t_g, s_g, LEAP, 0.02, 60.0, 30.0, level_ratio_max=None, max_window_sigma_s=None)
        self.assertEqual([w.get("offset") for w in a[1]], [w.get("offset") for w in b[1]])
        self.assertEqual([w.get("used") for w in a[1]], [w.get("used") for w in b[1]])

    def test_level_ratio_rejects_windows_the_estimator_does_not_follow(self):
        t_e, s_e, t_g, s_g = self.case(scale=0.6)                     # an estimator that moves at 60 % of the real speed
        free = tc.calibrate(t_e, s_e, t_g, s_g, LEAP, 0.02, 60.0, 30.0)[1]
        gated = tc.calibrate(t_e, s_e, t_g, s_g, LEAP, 0.02, 60.0, 30.0, level_ratio_max=1.25)[1]
        self.assertTrue(any(w.get("used") for w in free))              # the correlation alone cannot tell
        self.assertFalse(any(w.get("used") for w in gated))
        self.assertTrue(all("differ" in w.get("reason", "") for w in gated))
        t_e, s_e, t_g, s_g = self.case(scale=1.0)
        ok = tc.calibrate(t_e, s_e, t_g, s_g, LEAP, 0.02, 60.0, 30.0, level_ratio_max=1.25)[1]
        self.assertTrue(any(w.get("used") for w in ok))

    def test_window_sigma_limit_drops_imprecise_windows(self):
        t_e, s_e, t_g, s_g = self.case()
        free = tc.calibrate(t_e, s_e, t_g, s_g, LEAP, 0.02, 60.0, 30.0)[1]
        used = [w for w in free if w.get("used")]
        self.assertTrue(used)
        lim = float(np.median([w["sigma_pw"] for w in used]))
        cut = tc.calibrate(t_e, s_e, t_g, s_g, LEAP, 0.02, 60.0, 30.0, max_window_sigma_s=lim)[1]
        n_cut = sum(1 for w in cut if w.get("used"))
        self.assertLess(n_cut, len(used))
        self.assertTrue(all(w["sigma_pw"] <= lim for w in cut if w.get("used")))
        self.assertTrue(any("precision" in w.get("reason", "") for w in cut))


if __name__ == "__main__":
    unittest.main()
