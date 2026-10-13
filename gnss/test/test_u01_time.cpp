// SPDX-License-Identifier: BSD-3-Clause
// test_u01_time.cpp — TEST_PLAN §1 U1 (DESIGN §1 "时间", §7; AMENDMENTS A18, A40). Owner: WP-a.
// Criterion: GNSS time-system conversions are bit-equal to the RTKLIB functions and to known epochs; the t_est map
// has the documented bases and signs; the calibration file (A18) is parsed and segmented as specified, its *_gpst_s
// values are read on the GTime::toSec() scale (1970 origin, GPST, no leap seconds; A18/A40) and, given the data span,
// a file on another origin is refused (A40).
#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include "internal/rtk_adapter.hpp"
#include "internal/time_map.hpp"
#include "rtklib.h"   // WP-a test files may include it (OWNERSHIP §1): the reference implementation itself

using namespace livo_gnss;

namespace {

GTime ep(double y, double mo, double d, double h, double mi, double s) {
  const double e[6] = {y, mo, d, h, mi, s};
  return rtk::epoch2time(e);
}
gtime_t toG(GTime t) {
  gtime_t g;
  g.time = static_cast<time_t>(t.sec);
  g.sec = t.frac;
  return g;
}
// Bitwise equality of two GTime (integer part and the bits of the fraction).
::testing::AssertionResult sameBits(const GTime& a, const GTime& b) {
  if (a.sec == b.sec && std::memcmp(&a.frac, &b.frac, sizeof(double)) == 0) return ::testing::AssertionSuccess();
  char buf[160];
  std::snprintf(buf, sizeof(buf), "(%lld, %a) vs (%lld, %a)", static_cast<long long>(a.sec), a.frac, static_cast<long long>(b.sec), b.frac);
  return ::testing::AssertionFailure() << buf;
}
::testing::AssertionResult sameBits(const GTime& a, const gtime_t& b) {
  GTime c;
  c.sec = static_cast<int64_t>(b.time);
  c.frac = b.sec;
  return sameBits(a, c);
}
bool sameDouble(double a, double b) { return std::memcmp(&a, &b, sizeof(double)) == 0; }

// Random GPST instants 1990–2035 with arbitrary fractions (the conversions must not touch the fraction bits).
std::vector<GTime> randomTimes(int n, unsigned seed) {
  std::mt19937_64 rng(seed);
  std::uniform_int_distribution<int64_t> s(631152000LL, 2051222400LL);
  std::uniform_real_distribution<double> f(0.0, 1.0);
  std::vector<GTime> v;
  for (int i = 0; i < n; ++i) {
    GTime t;
    t.sec = s(rng);
    t.frac = f(rng);
    v.push_back(t);
  }
  return v;
}

Config::Time timeCfg(Config::Time::Base base, double offset, double drift_ppm, double t_ref) {
  Config::Time c;
  c.base = base;
  c.offset_s = offset;
  c.drift_ppm = drift_ppm;
  c.t_ref_gpst_s = t_ref;
  return c;
}

constexpr double kGpsEpochUnix = 315964800.0;   // 1980-01-06 00:00:00 GPST on the GTime::toSec() scale

std::string tmpPath(const std::string& name) { return ::testing::TempDir() + "/livo_u01_" + name; }
void writeFile(const std::string& path, const std::string& text) {
  std::ofstream o(path);
  o << text;
}

}  // namespace

// GPST↔UTC via the RTKLIB leap table; 2026 must give 18 s, bit-equal to rtk::gpst2utc
TEST(U01Time, Gpst2UtcLeapTableByDate2026Is18s) {
  // 2026 (the evaluation year) and the 2024/2025 dataset dates: 18 s, exactly.
  EXPECT_EQ(rtk::leapSecondsAt(ep(2026, 6, 1, 12, 0, 0)), 18.0);
  EXPECT_EQ(rtk::leapSecondsAt(ep(2024, 11, 25, 3, 0, 0)), 18.0);
  EXPECT_EQ(rtk::leapSecondsAt(ep(2025, 1, 3, 6, 0, 0)), 18.0);
  // By date, not a constant: the 2017-01-01 step (UTC 2017-01-01 00:00:00 = GPST 00:00:18) and 2015-07-01 (17 s).
  EXPECT_EQ(rtk::leapSecondsAt(ep(2017, 1, 1, 0, 0, 17)), 17.0);
  EXPECT_EQ(rtk::leapSecondsAt(ep(2017, 1, 1, 0, 0, 18)), 18.0);
  EXPECT_EQ(rtk::leapSecondsAt(ep(2015, 6, 30, 0, 0, 0)), 16.0);
  EXPECT_EQ(rtk::leapSecondsAt(ep(2016, 1, 1, 0, 0, 0)), 17.0);
  EXPECT_EQ(rtk::leapSecondsAt(ep(1981, 1, 1, 0, 0, 0)), 0.0);
  // Known epoch: GPST 2025-01-03 06:00:18.25 is UTC 06:00:00.25.
  EXPECT_TRUE(sameBits(rtk::gpst2utc(ep(2025, 1, 3, 6, 0, 18.25)), ep(2025, 1, 3, 6, 0, 0.25)));
  EXPECT_TRUE(sameBits(rtk::utc2gpst(ep(2025, 1, 3, 6, 0, 0.25)), ep(2025, 1, 3, 6, 0, 18.25)));
  // The adapter is the RTKLIB function, bit for bit (GTime ↔ gtime_t conversion loses nothing).
  for (const GTime& t : randomTimes(2000, 7)) {
    EXPECT_TRUE(sameBits(rtk::gpst2utc(t), ::gpst2utc(toG(t))));
    EXPECT_TRUE(sameBits(rtk::utc2gpst(t), ::utc2gpst(toG(t))));
    EXPECT_TRUE(sameDouble(rtk::leapSecondsAt(t), ::timediff(toG(t), ::gpst2utc(toG(t)))));
  }
  // TimeMap: "auto" uses the table; an integer string overrides it; garbage is a configuration error.
  Config::Time c;
  EXPECT_EQ(TimeMap(c).leapSeconds(ep(2026, 6, 1, 0, 0, 0)), 18.0);
  c.leap = "17";
  EXPECT_EQ(TimeMap(c).leapSeconds(ep(2026, 6, 1, 0, 0, 0)), 17.0);
  c.leap = "18.5";
  EXPECT_THROW(TimeMap{c}, std::invalid_argument);
  c.leap = "eighteen";
  EXPECT_THROW(TimeMap{c}, std::invalid_argument);
}

// BDT = GPST − 14 s, week difference 1356 (bdt2gpst/gpst2bdt round trip)
TEST(U01Time, BdtOffset14sAndWeek1356) {
  for (const GTime& t : randomTimes(2000, 11)) {
    const GTime b = rtk::gpst2bdt(t);
    EXPECT_TRUE(sameBits(b, ::gpst2bdt(toG(t))));            // RTKLIB bit for bit
    EXPECT_TRUE(sameBits(rtk::bdt2gpst(b), ::bdt2gpst(toG(b))));
    EXPECT_EQ(b.sec, t.sec - 14);
    // timeadd() forms (frac − 14) + 14: the fraction may move by an ulp of 14 (1.8e-15 s), never more.
    EXPECT_NEAR(b.frac, t.frac, 4e-15);
    EXPECT_NEAR(rtk::bdt2gpst(b).diff(t), 0.0, 4e-15);
  }
  // Mid-week: BDT week = GPS week − 1356, BDT tow = GPS tow − 14.
  int wg = 0, wb = 0;
  const GTime t = rtk::gpst2time(2347, 446400.5);   // 2025-01-03 04:00:00.5 GPST
  const double tow_g = rtk::time2gpst(t, &wg);
  const double tow_b = rtk::time2bdt(rtk::gpst2bdt(t), &wb);
  EXPECT_EQ(wg, 2347);
  EXPECT_EQ(wb, 2347 - 1356);
  EXPECT_EQ(tow_g, 446400.5);
  EXPECT_EQ(tow_b, 446386.5);
  EXPECT_TRUE(sameBits(rtk::bdt2gpst(rtk::bdt2time(wb, tow_b)), t));
  // BDT epoch: 2006-01-01 00:00:00 BDT = GPST 00:00:14 = GPS week 1356 tow 14.
  EXPECT_TRUE(sameBits(rtk::bdt2gpst(rtk::bdt2time(0, 0.0)), rtk::gpst2time(1356, 14.0)));
  // Within the first 14 s of a GPS week the BDT week is still the previous one (week roll-over of BDT).
  const GTime t2 = rtk::gpst2time(2347, 10.0);
  EXPECT_EQ(rtk::time2bdt(rtk::gpst2bdt(t2), &wb), 604796.0);
  EXPECT_EQ(wb, 2347 - 1357);
}

// GST seconds equal GPST, week difference 1024
TEST(U01Time, GstWeekOffset1024) {
  for (int w : {1024, 2047, 2048, 2347, 2400}) {
    for (double tow : {0.0, 0.25, 302400.125, 604799.75}) {
      const GTime g = rtk::gpst2time(w, tow);
      EXPECT_TRUE(sameBits(rtk::gst2time(w - 1024, tow), g));
      int wg = 0, we = 0;
      EXPECT_TRUE(sameDouble(rtk::time2gst(g, &we), rtk::time2gpst(g, &wg)));
      EXPECT_EQ(we, wg - 1024);
    }
  }
  for (const GTime& t : randomTimes(500, 13)) {
    int we = 0, wr = 0;
    EXPECT_TRUE(sameDouble(rtk::time2gst(t, &we), ::time2gst(toG(t), &wr)));
    EXPECT_EQ(we, wr);
  }
}

// GLONASST = UTC + 3 h through utc2gpst
TEST(U01Time, GlonassTimeIsUtcPlus3h) {
  // Known epoch: GPST 2025-01-03 06:00:18 = UTC 06:00:00 = GLONASST 09:00:00 (Moscow time, no leap offset of its own).
  const GTime gpst = ep(2025, 1, 3, 6, 0, 18);
  const GTime glot = rtk::timeadd(rtk::gpst2utc(gpst), 10800.0);
  EXPECT_TRUE(sameBits(glot, ep(2025, 1, 3, 9, 0, 0)));
  // Inverse used by the RINEX/UBX GLONASS paths (rinex.c decode_geph: toe = utc2gpst(toc_utc)).
  EXPECT_TRUE(sameBits(rtk::utc2gpst(rtk::timeadd(glot, -10800.0)), gpst));
  // Across the 2017 leap second the GLONASST−GPST offset changes from −(3 h − 17 s) to −(3 h − 18 s).
  const GTime before = ep(2016, 12, 31, 12, 0, 0), after = ep(2017, 1, 2, 12, 0, 0);
  EXPECT_EQ(rtk::timediff(rtk::timeadd(rtk::gpst2utc(before), 10800.0), before), 10800.0 - 17.0);
  EXPECT_EQ(rtk::timediff(rtk::timeadd(rtk::gpst2utc(after), 10800.0), after), 10800.0 - 18.0);
}

// week/tow round trip across a week boundary is exact
TEST(U01Time, WeekRollover) {
  int w = 0;
  const GTime end = rtk::gpst2time(2347, 604799.5);
  const GTime next = rtk::timeadd(end, 1.0);
  EXPECT_TRUE(sameBits(next, rtk::gpst2time(2348, 0.5)));
  EXPECT_EQ(rtk::time2gpst(next, &w), 0.5);
  EXPECT_EQ(w, 2348);
  EXPECT_TRUE(sameBits(rtk::gpst2time(2347, 604800.0), rtk::gpst2time(2348, 0.0)));
  EXPECT_TRUE(sameBits(rtk::gpst2time(2348, -1.0), rtk::gpst2time(2347, 604799.0)));
  // Historical 10-bit roll-overs are only a matter of the counter: week 1024 = 1999-08-22, 2048 = 2019-04-07.
  EXPECT_TRUE(sameBits(rtk::gpst2time(1024, 0.0), ep(1999, 8, 22, 0, 0, 0)));
  EXPECT_TRUE(sameBits(rtk::gpst2time(2048, 0.0), ep(2019, 4, 7, 0, 0, 0)));
  // GTime::add / GTime::diff (types.hpp, m2) are RTKLIB timeadd/timediff bit for bit, and so are the wrappers.
  std::mt19937_64 rng(17);
  std::uniform_real_distribution<double> dt(-1.0e6, 1.0e6);
  for (const GTime& t : randomTimes(3000, 19)) {
    const double d = dt(rng);
    EXPECT_TRUE(sameBits(t.add(d), ::timeadd(toG(t), d)));
    EXPECT_TRUE(sameBits(rtk::timeadd(t, d), ::timeadd(toG(t), d)));
    const GTime u = t.add(d);
    EXPECT_TRUE(sameDouble(u.diff(t), ::timediff(toG(u), toG(t))));
    EXPECT_TRUE(sameDouble(rtk::timediff(u, t), ::timediff(toG(u), toG(t))));
    int wa = 0, wb = 0;
    EXPECT_TRUE(sameDouble(rtk::time2gpst(t, &wa), ::time2gpst(toG(t), &wb)));
    EXPECT_EQ(wa, wb);
    double e1[6], e2[6];
    rtk::time2epoch(t, e1);
    ::time2epoch(toG(t), e2);
    for (int i = 0; i < 6; ++i) EXPECT_TRUE(sameDouble(e1[i], e2[i]));
    EXPECT_TRUE(sameBits(rtk::epoch2time(e1), ::epoch2time(e2)));
  }
  EXPECT_EQ(rtk::timeStr(ep(2025, 1, 3, 6, 0, 18.25), 3), "2025/01/03 06:00:18.250");
}

// t_est mapping for base gpst/utc/host with offset and drift signs (DESIGN §1)
TEST(U01Time, EstimatorMappingThreeBases) {
  // Known epoch: GPST 2025-01-03 06:00:18.25 = UTC 06:00:00.25 = Unix 1735884000.25 (all exactly representable).
  const GTime t = ep(2025, 1, 3, 6, 0, 18.25);
  const double unix_utc = 1735884000.25, unix_gpst = 1735884018.25;
  using B = Config::Time::Base;
  EXPECT_EQ(TimeMap(timeCfg(B::Utc, 0.0, 0.0, 0.0)).toEstimator(t), unix_utc);
  EXPECT_EQ(TimeMap(timeCfg(B::Host, 0.0, 0.0, 0.0)).toEstimator(t), unix_utc);
  EXPECT_EQ(TimeMap(timeCfg(B::Gpst, 0.0, 0.0, 0.0)).toEstimator(t), unix_gpst);   // no leap second on a GPST timeline
  // offset: t_est = ... + offset (a host clock 38.7 ms behind the GNSS time: offset −0.0387, A4 Dark01).
  const double ulp = 2.4e-7;   // double resolution at 1.7e9 s
  EXPECT_NEAR(TimeMap(timeCfg(B::Host, -0.0387, 0.0, 0.0)).toEstimator(t), unix_utc - 0.0387, ulp);
  EXPECT_NEAR(TimeMap(timeCfg(B::Gpst, 0.125, 0.0, 0.0)).toEstimator(t), unix_gpst + 0.125, 0.0);
  // drift: + drift_ppm·1e-6·(t_gpst − t_ref); t_ref on the GTime::toSec() scale (A18/A40), here 1000 s before t.
  const double t_ref = t.toSec() - 1000.0;
  EXPECT_NEAR(TimeMap(timeCfg(B::Host, 0.0, 10.0, t_ref)).toEstimator(t), unix_utc + 0.01, ulp);
  EXPECT_NEAR(TimeMap(timeCfg(B::Host, 0.0, -10.0, t_ref)).toEstimator(t), unix_utc - 0.01, ulp);
  EXPECT_NEAR(TimeMap(timeCfg(B::Host, 0.0, 10.0, t_ref + 2000.0)).toEstimator(t), unix_utc - 0.01, ulp);
  // The inverse maps back within the double resolution of t_est, for every base and with drift.
  for (B b : {B::Gpst, B::Utc, B::Host}) {
    const TimeMap m(timeCfg(b, -0.0387, 12.5, t_ref));
    for (double dt : {-3600.0, -0.5, 0.0, 0.3, 7200.0}) {
      const GTime g = t.add(dt);
      EXPECT_NEAR(m.toGpst(m.toEstimator(g)).diff(g), 0.0, 5e-7) << static_cast<int>(b) << " " << dt;
    }
  }
  // A leap-second boundary inside the mapped interval: UTC timeline jumps back by one second at 2017-01-01.
  const TimeMap mu(timeCfg(B::Utc, 0.0, 0.0, 0.0));
  EXPECT_EQ(mu.toEstimator(ep(2017, 1, 1, 0, 0, 18)) - mu.toEstimator(ep(2017, 1, 1, 0, 0, 16)), 1.0);
}

// t_true = t_rcv − b/c (positive b moves the true time earlier)
TEST(U01Time, ReceiverClockBiasCorrection) {
  const GTime t_rcv = ep(2025, 1, 3, 6, 0, 18.5);
  const double b = rtk::kCLight * 1e-3;   // receiver clock 1 ms ahead
  const GTime t_true = TimeMap::receiverToTrue(t_rcv, b);
  EXPECT_NEAR(t_true.diff(t_rcv), -1e-3, 1e-15);
  EXPECT_TRUE(sameBits(t_true, ::timeadd(toG(t_rcv), -b / CLIGHT)));
  EXPECT_NEAR(TimeMap::receiverToTrue(t_rcv, -b).diff(t_rcv), 1e-3, 1e-15);
  EXPECT_TRUE(sameBits(TimeMap::receiverToTrue(t_rcv, 0.0), t_rcv));
  const TimeMap m(timeCfg(Config::Time::Base::Host, -0.0387, 0.0, 0.0));
  EXPECT_EQ(m.receiverToEstimator(t_rcv, b), m.toEstimator(t_true));
  EXPECT_NEAR(m.receiverToEstimator(t_rcv, b) - m.toEstimator(t_rcv), -1e-3, 5e-7);
}

// isWholeSecond accepts |tow − round(tow)| < 5 ms only
TEST(U01Time, WholeSecondDecimation) {
  const GTime base = rtk::gpst2time(2347, 446400.0);
  EXPECT_TRUE(TimeMap::isWholeSecond(base));
  EXPECT_TRUE(TimeMap::isWholeSecond(base.add(0.004)));
  EXPECT_FALSE(TimeMap::isWholeSecond(base.add(0.006)));
  EXPECT_TRUE(TimeMap::isWholeSecond(base.add(0.996)));
  EXPECT_FALSE(TimeMap::isWholeSecond(base.add(0.994)));
  EXPECT_FALSE(TimeMap::isWholeSecond(base.add(0.5)));
  EXPECT_FALSE(TimeMap::isWholeSecond(base.add(0.1)));   // 10 Hz epochs: exactly one in ten passes
  int n = 0;
  for (int k = 0; k < 100; ++k) n += TimeMap::isWholeSecond(base.add(0.1 * k)) ? 1 : 0;
  EXPECT_EQ(n, 10);
  EXPECT_TRUE(TimeMap::isWholeSecond(base.add(0.009), 0.01));
  EXPECT_FALSE(TimeMap::isWholeSecond(base.add(0.009)));
}

// A18: restricted-YAML calibration file, piecewise segments (first one from −∞), extra keys ignored, errors reported
TEST(U01Time, CalibrationFileSegments) {
  const GTime t0 = ep(2025, 1, 3, 6, 0, 18.25);
  const double g0 = t0.toSec();   // A18/A40: *_gpst_s = GTime::toSec()
  char text[2048];
  std::snprintf(text, sizeof(text),
                "sequence: Occlusion03\n"
                "tool_version: 1\n"
                "leap_s: 18\n"
                "segments:\n"
                "  - t_from_gpst_s: 0\n"
                "    offset_s: -0.0387\n"
                "    drift_ppm: 0.0\n"
                "    t_ref_gpst_s: %.3f\n"
                "    sigma_toff_s: 0.004\n"
                "    tdcp_paused: false\n"
                "    method: xcorr          # xcorr | record_time | config\n"
                "    peak_corr: 0.97\n"
                "    window_scatter_ms: 2.1\n"
                "  - t_from_gpst_s: %.3f\n"
                "    offset_s: 0.0466\n"
                "    drift_ppm: 2.0\n"
                "    t_ref_gpst_s: %.3f\n"
                "    sigma_toff_s: 0.05\n"
                "    tdcp_paused: true\n"
                "    method: config\n",
                g0, g0 + 250.0, g0 + 250.0);
  const std::string path = tmpPath("calib_ok.yaml");
  writeFile(path, text);
  TimeMap m(timeCfg(Config::Time::Base::Host, 9.0, 0.0, 0.0));
  std::string why;
  ASSERT_TRUE(m.loadCalibFile(path, &why)) << why;
  ASSERT_EQ(m.segments().size(), 2u);
  EXPECT_EQ(m.offset(t0), -0.0387);
  EXPECT_EQ(m.sigmaToff(t0), 0.004);
  EXPECT_FALSE(m.tdcpPaused(t0));
  EXPECT_EQ(m.offset(t0.add(-1.0e6)), -0.0387);   // the first segment extends to −∞
  EXPECT_EQ(m.offset(t0.add(249.999)), -0.0387);
  EXPECT_EQ(m.offset(t0.add(250.0)), 0.0466);      // [t_from, next)
  EXPECT_EQ(m.driftPpm(t0.add(300.0)), 2.0);
  EXPECT_TRUE(m.tdcpPaused(t0.add(300.0)));
  EXPECT_NEAR(m.toEstimator(t0), 1735884000.25 - 0.0387, 2.4e-7);
  EXPECT_NEAR(m.toEstimator(t0.add(350.0)), 1735884000.25 + 350.0 + 0.0466 + 2e-6 * 100.0, 2.4e-7);

  // Errors: each one names the problem and leaves the map unchanged.
  const std::vector<std::pair<std::string, std::string>> bad = {
      {"missing_key", "segments:\n  - t_from_gpst_s: 0\n    offset_s: 0.1\n    drift_ppm: 0\n    t_ref_gpst_s: 0\n    tdcp_paused: false\n"},
      {"not_number", "segments:\n  - t_from_gpst_s: 0\n    offset_s: abc\n    drift_ppm: 0\n    t_ref_gpst_s: 0\n    sigma_toff_s: 0.1\n    tdcp_paused: false\n"},
      {"not_bool", "segments:\n  - t_from_gpst_s: 0\n    offset_s: 0\n    drift_ppm: 0\n    t_ref_gpst_s: 0\n    sigma_toff_s: 0.1\n    tdcp_paused: maybe\n"},
      {"unsorted", "segments:\n  - t_from_gpst_s: 100\n    offset_s: 0\n    drift_ppm: 0\n    t_ref_gpst_s: 0\n    sigma_toff_s: 0.1\n    tdcp_paused: false\n"
                   "  - t_from_gpst_s: 50\n    offset_s: 0\n    drift_ppm: 0\n    t_ref_gpst_s: 0\n    sigma_toff_s: 0.1\n    tdcp_paused: false\n"},
      {"flow", "segments: [{t_from_gpst_s: 0}]\n"},
      {"tab", "segments:\n\t- t_from_gpst_s: 0\n"},
      {"empty", "sequence: x\n"},
      {"leap", std::string("leap_s: 17\nsegments:\n  - t_from_gpst_s: 0\n    offset_s: 0\n    drift_ppm: 0\n    t_ref_gpst_s: ") +
                   std::to_string(g0) + "\n    sigma_toff_s: 0.1\n    tdcp_paused: false\n"},
  };
  for (const auto& b : bad) {
    const std::string p = tmpPath("calib_" + b.first + ".yaml");
    writeFile(p, b.second);
    std::string w;
    EXPECT_FALSE(m.loadCalibFile(p, &w)) << b.first;
    EXPECT_FALSE(w.empty()) << b.first;
    EXPECT_EQ(m.segments().size(), 2u) << b.first;
  }
  std::string w;
  EXPECT_FALSE(m.loadCalibFile(tmpPath("does_not_exist.yaml"), &w));
  // setSegments sorts; the first t_from is irrelevant (−∞).
  TimeMap::Segment a, b;
  a.t_from_gpst_s = g0 + 10.0;
  a.offset_s = 0.2;
  b.t_from_gpst_s = 123.0;
  b.offset_s = 0.1;
  m.setSegments({a, b});
  EXPECT_EQ(m.offset(t0), 0.1);
  EXPECT_EQ(m.offset(t0.add(11.0)), 0.2);
  EXPECT_THROW(m.setSegments({}), std::invalid_argument);

  // A drift needs its reference epoch: t_ref = 0 is 1970 and would multiply the drift by ~1.7e9 s (review note).
  const std::string drift_no_ref = tmpPath("calib_drift_no_ref.yaml");
  writeFile(drift_no_ref, "segments:\n  - t_from_gpst_s: 0\n    offset_s: 0\n    drift_ppm: 1.5\n    t_ref_gpst_s: 0\n"
                          "    sigma_toff_s: 0.1\n    tdcp_paused: false\n");
  std::string w2;
  EXPECT_FALSE(m.loadCalibFile(drift_no_ref, &w2));
  EXPECT_NE(w2.find("t_ref_gpst_s"), std::string::npos) << w2;
  EXPECT_THROW(TimeMap{timeCfg(Config::Time::Base::Host, 0.0, 1.5, 0.0)}, std::invalid_argument);
  EXPECT_NO_THROW(TimeMap{timeCfg(Config::Time::Base::Host, 0.0, 0.0, 0.0)});   // the §14 defaults stay valid
}

// setOffset (frozen skeleton convenience): back to one segment from −∞ with the given offset/drift/t_ref; σ_toff and
// the TDCP pause of the former first segment carry over; the map equals a TimeMap built from the same config values.
TEST(U01Time, SetOffsetSingleSegment) {
  const GTime t0 = ep(2025, 1, 3, 6, 0, 18.25);
  const double g0 = t0.toSec();
  TimeMap m(timeCfg(Config::Time::Base::Host, 9.0, 0.0, 0.0));
  TimeMap::Segment a, b;
  a.t_from_gpst_s = 123.0;   // irrelevant (first segment starts at −∞)
  a.offset_s = 0.1;
  a.sigma_toff_s = 0.004;
  a.tdcp_paused = true;
  b.t_from_gpst_s = g0 + 100.0;
  b.offset_s = 0.7;
  b.drift_ppm = 3.0;
  b.t_ref_gpst_s = g0 + 100.0;
  b.sigma_toff_s = 0.02;
  m.setSegments({a, b});
  ASSERT_EQ(m.segments().size(), 2u);
  EXPECT_EQ(m.offset(t0.add(200.0)), 0.7);

  const double t_ref = g0 - 1000.0;
  m.setOffset(-0.0387, 2.0, t_ref);
  ASSERT_EQ(m.segments().size(), 1u);
  const TimeMap::Segment& s = m.segments().front();
  EXPECT_EQ(s.t_from_gpst_s, 0.0);   // t_from reset
  EXPECT_EQ(s.offset_s, -0.0387);
  EXPECT_EQ(s.drift_ppm, 2.0);
  EXPECT_EQ(s.t_ref_gpst_s, t_ref);
  EXPECT_EQ(s.sigma_toff_s, 0.004);  // carried over from the former first segment
  EXPECT_TRUE(s.tdcp_paused);
  // One segment everywhere, including where the second segment used to start.
  for (double dt : {-1.0e6, 0.0, 99.0, 100.0, 200.0, 1.0e5}) {
    EXPECT_EQ(m.offset(t0.add(dt)), -0.0387) << dt;
    EXPECT_EQ(m.driftPpm(t0.add(dt)), 2.0) << dt;
  }
  const TimeMap ref(timeCfg(Config::Time::Base::Host, -0.0387, 2.0, t_ref));
  for (double dt : {-3600.0, 0.0, 250.0, 7200.0}) {
    const double a_est = m.toEstimator(t0.add(dt)), b_est = ref.toEstimator(t0.add(dt));
    EXPECT_TRUE(sameDouble(a_est, b_est)) << dt;
  }
  // t_est = t_utc + offset + drift·1e-6·(t_gps − t_ref): 1000 s after t_ref, 2 ppm → +2 ms.
  EXPECT_NEAR(m.toEstimator(t0), 1735884000.25 - 0.0387 + 2e-3, 2.4e-7);
}

// A18/A40: every *_gpst_s value (config t_ref, file t_from/t_ref) is GTime::toSec() — 1970-01-01 origin, GPST scale, no
// leap seconds. A18's example 1733040083 is GPST week 2343, tow 28883 (Occlusion03's first range_meas epoch). A value on
// the 1980-01-06 origin is 315964800 s early; with a 3 ppm drift that moves the map by 3e-6·315964800 ≈ 948 s, which
// the origin check (A40, data span ± 1 day, same rule as tools/fl2g/timecalib_yaml.py check_span) refuses.
TEST(U01Time, CalibrationTimeOriginIsGTimeToSec) {
  using B = Config::Time::Base;
  const GTime t0 = rtk::gpst2time(2343, 28883.0);
  EXPECT_EQ(t0.toSec(), 1733040083.0);
  EXPECT_EQ(t0.toSec(), kGpsEpochUnix + 2343.0 * 604800.0 + 28883.0);
  EXPECT_EQ(rtk::leapSecondsAt(t0), 18.0);
  // The map on the toSec scale: 1000 s after t_ref, 3 ppm → +3 ms (t_est = UTC = GPST − 18 s, offset 0).
  const TimeMap m(timeCfg(B::Host, 0.0, 3.0, t0.toSec()));
  EXPECT_NEAR(m.toEstimator(t0.add(1000.0)) - (1733040083.0 + 1000.0 - 18.0), 3e-3, 2.4e-7);
  EXPECT_NEAR(m.toEstimator(t0) - (1733040083.0 - 18.0), 0.0, 2.4e-7);
  // The same t_ref written on the 1980 origin: the drift term is off by 948 s.
  const TimeMap w(timeCfg(B::Host, 0.0, 3.0, t0.toSec() - kGpsEpochUnix));
  EXPECT_NEAR(w.toEstimator(t0) - m.toEstimator(t0), 3e-6 * kGpsEpochUnix, 1e-6);
  EXPECT_GT(w.toEstimator(t0) - m.toEstimator(t0), 947.0);

  // checkTimeOrigin on segments (e.g. the config's single segment): span = the data, Occlusion03's 242 s (A26).
  const GTime b = t0, e = t0.add(242.0);
  std::string why;
  EXPECT_TRUE(TimeMap::checkTimeOrigin(m.segments(), b, e, &why)) << why;
  EXPECT_FALSE(TimeMap::checkTimeOrigin(w.segments(), b, e, &why));
  EXPECT_NE(why.find("t_ref_gpst_s"), std::string::npos) << why;
  EXPECT_NE(why.find("1970"), std::string::npos) << why;
  // A drift-free segment's t_ref is never used, so it is not checked (the tool writes whatever it had).
  EXPECT_TRUE(TimeMap::checkTimeOrigin(TimeMap(timeCfg(B::Host, 0.0, 0.0, 0.0)).segments(), b, e, &why)) << why;
  // ± 1 day inclusive, on both sides; the first segment's t_from (−∞) is never checked.
  auto seg = [](double t_from, double drift, double t_ref) {
    TimeMap::Segment s;
    s.t_from_gpst_s = t_from;
    s.drift_ppm = drift;
    s.t_ref_gpst_s = t_ref;
    return s;
  };
  const double lo = t0.toSec() - 86400.0, hi = e.toSec() + 86400.0;
  EXPECT_TRUE(TimeMap::checkTimeOrigin({seg(0.0, 1.0, lo), seg(hi, 1.0, hi)}, b, e, &why)) << why;
  EXPECT_FALSE(TimeMap::checkTimeOrigin({seg(0.0, 1.0, lo - 0.5), seg(hi, 1.0, hi)}, b, e, &why));
  EXPECT_FALSE(TimeMap::checkTimeOrigin({seg(0.0, 0.0, 0.0), seg(hi + 0.5, 0.0, 0.0)}, b, e, &why));
  EXPECT_NE(why.find("t_from_gpst_s"), std::string::npos) << why;
  EXPECT_FALSE(TimeMap::checkTimeOrigin({seg(0.0, 0.0, 0.0), seg(t0.toSec() + 60.0 - kGpsEpochUnix, 0.0, 0.0)}, b, e, &why));
  EXPECT_TRUE(TimeMap::checkTimeOrigin({seg(123.0, 0.0, 0.0)}, b, e, &why)) << why;   // a lone first segment: −∞
  EXPECT_FALSE(TimeMap::checkTimeOrigin(m.segments(), e, b, &why));                     // span ends before it begins

  // The file level: the same two-segment calibration on the right origin and on the 1980 one (no leap_s key, so the
  // leap-table cross-check cannot catch it: only the span check can).
  auto calib = [](double first_t_ref, double t_from2, double t_ref2) {
    char text[1024];
    std::snprintf(text, sizeof(text),
                  "sequence: Occlusion03\ntool_version: 1\nsegments:\n"
                  "  - t_from_gpst_s: 0\n    offset_s: 0.0125\n    drift_ppm: 1.5\n    t_ref_gpst_s: %.1f\n"
                  "    sigma_toff_s: 0.004\n    tdcp_paused: false\n"
                  "  - t_from_gpst_s: %.1f\n    offset_s: 0.0625\n    drift_ppm: 0.0\n    t_ref_gpst_s: %.1f\n"
                  "    sigma_toff_s: 0.05\n    tdcp_paused: true\n",
                  first_t_ref, t_from2, t_ref2);
    return std::string(text);
  };
  const std::string ok_path = tmpPath("origin_ok.yaml"), bad_path = tmpPath("origin_1980.yaml");
  writeFile(ok_path, calib(t0.toSec(), t0.toSec() + 120.0, t0.toSec() + 120.0));
  writeFile(bad_path, calib(t0.toSec() - kGpsEpochUnix, t0.toSec() + 120.0 - kGpsEpochUnix, t0.toSec() + 120.0 - kGpsEpochUnix));
  TimeMap f(timeCfg(B::Host, 9.0, 0.0, 0.0));
  ASSERT_TRUE(f.loadCalibFile(ok_path, &why, b, e)) << why;
  ASSERT_EQ(f.segments().size(), 2u);
  EXPECT_EQ(f.offset(t0.add(119.0)), 0.0125);
  EXPECT_EQ(f.offset(t0.add(120.0)), 0.0625);   // the segment boundary sits where the data are
  EXPECT_NEAR(f.toEstimator(t0.add(100.0)) - (1733040083.0 + 100.0 - 18.0 + 0.0125), 1.5e-4, 5e-7);
  // The 1980-origin file is parsed fine by the span-less overload (it cannot know), and the map is wrong: the second
  // segment's start reads as 2014, so every epoch of the data falls into it ...
  TimeMap g(timeCfg(B::Host, 9.0, 0.0, 0.0));
  ASSERT_TRUE(g.loadCalibFile(bad_path, &why)) << why;
  EXPECT_EQ(g.offset(t0), 0.0625);
  EXPECT_EQ(f.offset(t0), 0.0125);
  // ... and a drift reference on the 1980 origin is off by 1.5 ppm × 315964800 s = 474 s.
  const std::string bad1_path = tmpPath("origin_1980_single.yaml");
  writeFile(bad1_path, "segments:\n  - t_from_gpst_s: 0\n    offset_s: 0.0125\n    drift_ppm: 1.5\n    t_ref_gpst_s: " +
                           std::to_string(t0.toSec() - kGpsEpochUnix) + "\n    sigma_toff_s: 0.004\n    tdcp_paused: false\n");
  TimeMap h(timeCfg(B::Host, 9.0, 0.0, 0.0));
  ASSERT_TRUE(h.loadCalibFile(bad1_path, &why)) << why;
  EXPECT_NEAR(h.toEstimator(t0) - f.toEstimator(t0), 1.5e-6 * kGpsEpochUnix, 1e-6);
  EXPECT_FALSE(TimeMap(h).loadCalibFile(bad1_path, &why, b, e));
  // With the span both files are refused, and the map stays as it was.
  EXPECT_FALSE(f.loadCalibFile(bad_path, &why, b, e));
  EXPECT_NE(why.find(bad_path), std::string::npos) << why;
  EXPECT_NE(why.find("GTime::toSec()"), std::string::npos) << why;
  ASSERT_EQ(f.segments().size(), 2u);
  EXPECT_EQ(f.segments()[0].t_ref_gpst_s, t0.toSec());   // unchanged
  // Parse errors are reported as before by the span overload too.
  EXPECT_FALSE(f.loadCalibFile(tmpPath("does_not_exist.yaml"), &why, b, e));
  EXPECT_EQ(f.segments()[0].t_ref_gpst_s, t0.toSec());
}

// The file exactly as tools/fl2g/timecalib_yaml.py dumps() writes it (A18 subset; WP-g request in INTERFACE_CHANGES):
// informational keys after the six A18 keys (method, peak_corr, window_scatter_ms, n_windows*, bootstrap_sd_ms,
// peak_width_sigma_ms, short_segment, dtw_*), the first t_from as the integer 0, numbers in Python repr (-1e-05,
// 1733040083.0), booleans true/false. The informational keys are skipped (A18 "其余忽略"), every literal is accepted, and
// the six values are the literals' doubles bit for bit. Text produced with dumps("Occlusion03", 18, segments,
// span=(1733040083, 1733040325)); the numbers are illustrative, not an Occlusion03 calibration.
TEST(U01Time, CalibrationFileFromTimeCalibTool) {
  const std::string text =
      "sequence: Occlusion03\n"
      "tool_version: 1\n"
      "leap_s: 18\n"
      "segments:\n"
      "  - t_from_gpst_s: 0\n"
      "    offset_s: 0.0125\n"
      "    drift_ppm: -1e-05\n"
      "    t_ref_gpst_s: 1733040083.0\n"
      "    sigma_toff_s: 0.0021\n"
      "    tdcp_paused: false\n"
      "    method: xcorr\n"
      "    peak_corr: 0.97\n"
      "    window_scatter_ms: 2.1\n"
      "    n_windows: 4\n"
      "    n_windows_rejected: 0\n"
      "    n_windows_isolated: 0\n"
      "    bootstrap_sd_ms: 1.25\n"
      "    peak_width_sigma_ms: 0.35\n"
      "    short_segment: false\n"
      "    dtw_offset_s: 0.0131\n"
      "    dtw_consistent: true\n"
      "  - t_from_gpst_s: 1733040203.0\n"
      "    offset_s: 0.0625\n"
      "    drift_ppm: 3.0\n"
      "    t_ref_gpst_s: 1733040203.5\n"
      "    sigma_toff_s: 0.05\n"
      "    tdcp_paused: true\n"
      "    method: config\n"
      "    n_windows: 2\n"
      "    short_segment: true\n";
  const std::string path = tmpPath("calib_tool.yaml");
  writeFile(path, text);
  const GTime t0 = rtk::gpst2time(2343, 28883.0);
  TimeMap m(timeCfg(Config::Time::Base::Host, 9.0, 0.0, 0.0));
  std::string why;
  ASSERT_TRUE(m.loadCalibFile(path, &why, t0, t0.add(242.0))) << why;
  ASSERT_EQ(m.segments().size(), 2u);
  const TimeMap::Segment& a = m.segments()[0];
  const TimeMap::Segment& c = m.segments()[1];
  EXPECT_TRUE(sameDouble(a.t_from_gpst_s, 0.0));
  EXPECT_TRUE(sameDouble(a.offset_s, 0.0125));
  EXPECT_TRUE(sameDouble(a.drift_ppm, -1e-05));
  EXPECT_TRUE(sameDouble(a.t_ref_gpst_s, 1733040083.0));
  EXPECT_TRUE(sameDouble(a.sigma_toff_s, 0.0021));
  EXPECT_FALSE(a.tdcp_paused);
  EXPECT_TRUE(sameDouble(c.t_from_gpst_s, 1733040203.0));
  EXPECT_TRUE(sameDouble(c.offset_s, 0.0625));
  EXPECT_TRUE(sameDouble(c.drift_ppm, 3.0));
  EXPECT_TRUE(sameDouble(c.t_ref_gpst_s, 1733040203.5));
  EXPECT_TRUE(sameDouble(c.sigma_toff_s, 0.05));
  EXPECT_TRUE(c.tdcp_paused);
  // Segment selection at the second segment's start (t0 + 120 s) and the drift sign of the first one.
  EXPECT_EQ(m.offset(t0.add(119.5)), 0.0125);
  EXPECT_EQ(m.offset(t0.add(120.0)), 0.0625);
  EXPECT_TRUE(m.tdcpPaused(t0.add(200.0)));
  EXPECT_NEAR(m.toEstimator(t0.add(100.0)) - (1733040083.0 + 100.0 - 18.0 + 0.0125), -1e-11 * 100.0, 5e-7);
  EXPECT_NEAR(m.toEstimator(t0.add(160.5)) - (1733040083.0 + 160.5 - 18.0 + 0.0625), 3e-6 * 40.0, 5e-7);
  // The span-less overload reads the same file identically.
  TimeMap n(timeCfg(Config::Time::Base::Host, 9.0, 0.0, 0.0));
  ASSERT_TRUE(n.loadCalibFile(path, &why)) << why;
  ASSERT_EQ(n.segments().size(), 2u);
  EXPECT_TRUE(sameDouble(n.segments()[1].t_ref_gpst_s, c.t_ref_gpst_s));
}
