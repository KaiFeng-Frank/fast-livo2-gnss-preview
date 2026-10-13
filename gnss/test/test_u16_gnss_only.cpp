// SPDX-License-Identifier: BSD-3-Clause
// test_u16_gnss_only.cpp — TEST_PLAN §1 U16 (AMENDMENTS A9-1): the self-written GNSS-only estimators on
// synthetic geometry (rtk::SatState is only a struct here; models_fixed keeps rtk_adapter out of the loop), plus the
// Doppler model's Sagnac-rate sign on real broadcast orbits (AMENDMENTS A36; WP-a's rtk_adapter and the dev-set
// fixture's IGS BRDC). The Doppler forward models never restate the rate formula under test: they differentiate the
// pseudorange model numerically (a forward model sharing the code's formula shares its sign error).
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <random>
#include <set>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "internal/gnss_only.hpp"
#include "internal/quality.hpp"
#include "internal/rtk_adapter.hpp"
#include "../../gnss_io/test/fixture_discovery.hpp"   // WP-e's one discovery rule (A24): realFixtures(), fixtureBrdc()

using namespace livo_gnss;

namespace {

constexpr double kC = 299792458.0;
constexpr double kOmgE = 7.2921151467e-5;
constexpr double kDeg = M_PI / 180.0;

Eigen::Vector3d geo2ecef(double lat, double lon, double h) {
  const double a = 6378137.0, f = 1.0 / 298.257223563, e2 = f * (2.0 - f);
  const double sl = std::sin(lat), cl = std::cos(lat);
  const double N = a / std::sqrt(1.0 - e2 * sl * sl);
  return Eigen::Vector3d((N + h) * cl * std::cos(lon), (N + h) * cl * std::sin(lon), (N * (1.0 - e2) + h) * sl);
}
Eigen::Matrix3d enuRot(double lat, double lon) {
  const double sl = std::sin(lat), cl = std::cos(lat), so = std::sin(lon), co = std::cos(lon);
  Eigen::Matrix3d R;
  R.col(0) = Eigen::Vector3d(-so, co, 0.0);
  R.col(1) = Eigen::Vector3d(-sl * co, -sl * so, cl);
  R.col(2) = Eigen::Vector3d(cl * co, cl * so, sl);
  return R;
}
// The pseudorange's Sagnac term (geodist, rtkcmn.c:3612; DESIGN §1): the one Earth-rotation expression the test writes.
double sagnacTerm(const Eigen::Vector3d& rs, const Eigen::Vector3d& rr) { return kOmgE * (rs.x() * rr.y() - rs.y() * rr.x()) / kC; }
double rangeSagnac(const Eigen::Vector3d& rs, const Eigen::Vector3d& rr, Eigen::Vector3d* e) {
  const Eigen::Vector3d d = rs - rr;
  const double r = d.norm();
  if (e) *e = d / r;
  return r + sagnacTerm(rs, rr);
}
// Sagnac rate as the central difference of the Sagnac term along the straight-line motion r_s + v_s τ, r_A + v_A τ
// (bilinear in τ ⇒ exact up to rounding, ~1e-15 m/s at h = 1 s) — AMENDMENTS A36 without restating its formula.
double sagnacRateNumeric(const Eigen::Vector3d& rs, const Eigen::Vector3d& vs, const Eigen::Vector3d& rr, const Eigen::Vector3d& vr) {
  const double h = 1.0;
  return (sagnacTerm(rs + vs * h, rr + vr * h) - sagnacTerm(rs - vs * h, rr - vr * h)) / (2.0 * h);
}
// Range rate of the Doppler forward model: e·(v_s − v_A) plus the numerical Sagnac rate.
double rangeRate(const Eigen::Vector3d& rs, const Eigen::Vector3d& vs, const Eigen::Vector3d& rr, const Eigen::Vector3d& vr) {
  Eigen::Vector3d e;
  rangeSagnac(rs, rr, &e);
  return e.dot(vs - vr) + sagnacRateNumeric(rs, vs, rr, vr);
}

SatId sat(Sys sys, int prn, int rtk) {
  SatId s;
  s.sys = sys;
  s.prn = prn;
  s.rtk_sat = rtk;
  return s;
}

struct Scene {
  Eigen::Vector3d r_true;
  Eigen::Matrix3d R_EN;
  double b_true = 1234.5, isb_gal = 3.2, isb_bds = -7.7;
  std::vector<SppInput> in;
};

// n satellites spread in azimuth with elevations 15–80°, 2.3e7 m away, 3 km/s across the line of sight.
Scene makeScene(int n, unsigned seed, bool multi_sys) {
  Scene sc;
  const double lat = 30.0 * kDeg, lon = 110.0 * kDeg;
  sc.r_true = geo2ecef(lat, lon, 50.0);
  sc.R_EN = enuRot(lat, lon);
  std::mt19937_64 rng(seed);
  std::uniform_real_distribution<double> uni(0.0, 1.0);
  for (int i = 0; i < n; ++i) {
    SppInput s;
    Sys sys = Sys::GPS;
    if (multi_sys) {
      if (i % 5 == 3) sys = Sys::GAL;
      if (i % 5 == 4) sys = Sys::BDS;
    }
    s.sat = sat(sys, i + 1, i + 1);
    // elevations spread evenly over 15–80° (a well-conditioned VDOP in every scene), azimuths random
    const double el = (15.0 + 65.0 * (n > 1 ? static_cast<double>((i * 7) % n) / (n - 1) : 0.5)) * kDeg;
    const double az = 2.0 * M_PI * (i + uni(rng)) / n;
    const Eigen::Vector3d los_N(std::cos(el) * std::sin(az), std::cos(el) * std::cos(az), std::sin(el));
    const Eigen::Vector3d los_E = sc.R_EN * los_N;
    s.state.ok = true;
    s.state.r_ecef = sc.r_true + los_E * 2.3e7;
    Eigen::Vector3d v = Eigen::Vector3d(uni(rng) - 0.5, uni(rng) - 0.5, uni(rng) - 0.5).cross(los_E);
    s.state.v_ecef = v.normalized() * 3000.0;
    s.state.dts = 1e-4 * (uni(rng) - 0.5);
    s.state.ddts = 1e-11 * (uni(rng) - 0.5);
    s.el_rad = el;
    s.az_rad = az;
    s.cn0 = 32.0 + 16.0 * std::sin(el);
    s.lambda_m = kC / 1575.42e6;
    s.trop_m = 2.3 / std::sin(el);
    s.ion_m = 1.5 / std::sin(el);
    s.var_psr = 1.0;
    s.var_dopp = 0.0025;
    s.psr_usable = true;
    s.dopp_usable = true;
    s.models_fixed = true;
    sc.in.push_back(s);
  }
  return sc;
}

void fillPseudoranges(Scene& sc, double sigma, std::mt19937_64& rng) {
  std::normal_distribution<double> gauss(0.0, 1.0);
  for (auto& s : sc.in) {
    const double rho = rangeSagnac(s.state.r_ecef, sc.r_true, nullptr);
    const double isb = s.sat.sys == Sys::GAL ? sc.isb_gal : (s.sat.sys == Sys::BDS ? sc.isb_bds : 0.0);
    s.P_corr = rho + sc.b_true + isb - kC * s.state.dts + s.ion_m + s.trop_m + sigma * gauss(rng);
  }
}

void fillDoppler(Scene& sc, const Eigen::Vector3d& v_ant_ecef, double d_true, double sigma, std::mt19937_64& rng) {
  std::normal_distribution<double> gauss(0.0, 1.0);
  for (auto& s : sc.in) {
    const double rate = rangeRate(s.state.r_ecef, s.state.v_ecef, sc.r_true, v_ant_ecef);
    s.dopp_hz = -(rate + d_true - kC * s.state.ddts + sigma * gauss(rng)) / s.lambda_m;
  }
}

}  // namespace

// noise-free synthetic epoch: position/clock/ISB recovered to < 1e-6 m, GDOP as computed from the geometry
TEST(U16GnssOnly, NoiseFreeSolutionExact) {
  Config cfg;
  NoiseModel noise(cfg);
  RobustSpp spp(cfg, noise);
  Scene sc = makeScene(10, 1, true);
  std::mt19937_64 rng(2);
  fillPseudoranges(sc, 0.0, rng);
  for (bool warm : {false, true}) {
    SppSolution sol;
    std::vector<SppInput> in = sc.in;
    ASSERT_TRUE(spp.solve(GTime::fromSec(1.4e9), in, nullptr, warm, sc.r_true + Eigen::Vector3d(100.0, -50.0, 30.0), 0.0, &sol));
    EXPECT_TRUE(sol.ok);
    EXPECT_LT((sol.r_ecef - sc.r_true).norm(), 1e-6) << (warm ? "warm" : "cold");
    EXPECT_NEAR(sol.b_m, sc.b_true, 1e-6);
    EXPECT_TRUE(sol.isb_est[1]);
    EXPECT_TRUE(sol.isb_est[2]);
    EXPECT_FALSE(sol.isb_est[0]);
    EXPECT_NEAR(sol.isb_m[1], sc.isb_gal, 1e-6);
    EXPECT_NEAR(sol.isb_m[2], sc.isb_bds, 1e-6);
    EXPECT_EQ(sol.n_used, 10);
    EXPECT_EQ(sol.n_rejected, 0);
    EXPECT_LT(sol.sigma0, 1e-6);
    // GDOP from the geometry: (HᵀH)⁻¹ with H = [−e, 1]
    Eigen::MatrixXd H(10, 4);
    for (int i = 0; i < 10; ++i) {
      Eigen::Vector3d e;
      rangeSagnac(sc.in[i].state.r_ecef, sc.r_true, &e);
      H.row(i) << -e.transpose(), 1.0;
    }
    const Eigen::Matrix4d Q = (H.transpose() * H).inverse();
    EXPECT_NEAR(sol.gdop, std::sqrt(Q.trace()), 1e-6);
    EXPECT_NEAR(sol.pdop, std::sqrt(Q.topLeftCorner<3, 3>().trace()), 1e-6);
    // formal covariance floored at 3² m² (§8) and consistent with the geometry
    for (int k = 0; k < 3; ++k) EXPECT_GE(sol.Q(k, k), 9.0 - 1e-9);
  }
  // too few satellites: refused
  {
    Scene few = makeScene(4, 3, false);
    fillPseudoranges(few, 0.0, rng);
    SppSolution sol;
    std::vector<SppInput> in = few.in;
    EXPECT_FALSE(spp.solve(GTime::fromSec(1.4e9), in, nullptr, false, Eigen::Vector3d::Zero(), 0.0, &sol));
  }
}

// self-written GNSS-only LS must reject an injected +30 m single-satellite blunder
TEST(U16GnssOnly, RobustLsRejects30mSingleOutlier) {
  Config cfg;
  NoiseModel noise(cfg);
  RobustSpp spp(cfg, noise);
  std::mt19937_64 rng(11);
  const int trials = 300;
  // (a) GPS-only, 9 satellites: the blunder must be in `rejected` every time and never a clean satellite
  {
    std::uniform_int_distribution<int> pick(0, 8);
    int rejected_right = 0, clean_rejected = 0;
    double max_err = 0.0;
    for (int t = 0; t < trials; ++t) {
      Scene sc = makeScene(9, 100 + t, false);
      fillPseudoranges(sc, 0.5, rng);
      const int bad = pick(rng);
      sc.in[bad].P_corr += 30.0;
      SppSolution sol;
      ASSERT_TRUE(spp.solve(GTime::fromSec(1.4e9), sc.in, nullptr, false, Eigen::Vector3d::Zero(), 0.0, &sol));
      bool bad_out = false;
      for (const auto& s : sol.rejected) {
        if (s.rtk_sat == sc.in[bad].sat.rtk_sat) bad_out = true; else ++clean_rejected;
      }
      if (bad_out) ++rejected_right;
      max_err = std::max(max_err, (sol.r_ecef - sc.r_true).norm());
    }
    EXPECT_EQ(rejected_right, trials) << "+30 m blunder must always be rejected";
    EXPECT_LE(clean_rejected, trials / 50);
    EXPECT_LT(max_err, 6.0) << "position error after rejection (σ = 0.5 m, DOP ≲ 3)";
  }
  // (b) four systems, 10 satellites (2 GAL, 2 BDS with their own ISBs): a blunder on a GPS satellite is
  // rejected; a blunder on one of a 2-satellite system is not identifiable (the two residuals are equal and
  // opposite through the ISB) — the safety property is that it never enters the solution: one of the two is
  // rejected and the other, now alone with its ISB, is dropped.
  {
    int gps_ok = 0, gps_trials = 0, isb_ok = 0, isb_trials = 0;
    double max_err = 0.0;
    for (int t = 0; t < trials; ++t) {
      Scene sc = makeScene(10, 700 + t, true);
      fillPseudoranges(sc, 0.5, rng);
      const int bad = t % 10;
      sc.in[bad].P_corr += 30.0;
      SppSolution sol;
      ASSERT_TRUE(spp.solve(GTime::fromSec(1.4e9), sc.in, nullptr, false, Eigen::Vector3d::Zero(), 0.0, &sol));
      bool in_used = false, in_rejected = false;
      for (const auto& s : sol.used) in_used |= (s.rtk_sat == sc.in[bad].sat.rtk_sat);
      for (const auto& s : sol.rejected) in_rejected |= (s.rtk_sat == sc.in[bad].sat.rtk_sat);
      if (sc.in[bad].sat.sys == Sys::GPS) {
        ++gps_trials;
        if (in_rejected) ++gps_ok;
      } else {
        ++isb_trials;
        if (!in_used) ++isb_ok;
      }
      max_err = std::max(max_err, (sol.r_ecef - sc.r_true).norm());
    }
    EXPECT_EQ(gps_ok, gps_trials);
    EXPECT_EQ(isb_ok, isb_trials);
    EXPECT_LT(max_err, 8.0);
  }
}

// two simultaneous NLOS satellites detected ≥ 95 % of trials
TEST(U16GnssOnly, RobustLsDetectsTwoNlos) {
  Config cfg;
  NoiseModel noise(cfg);
  RobustSpp spp(cfg, noise);
  std::mt19937_64 rng(21);
  const int trials = 500;
  int both = 0, any_clean = 0;
  for (int t = 0; t < trials; ++t) {
    Scene sc = makeScene(10, 500 + t, false);   // GPS-only: identifiability is the property under test
    fillPseudoranges(sc, 0.5, rng);
    const int b1 = t % 10, b2 = (t * 7 + 3) % 10 == b1 ? (b1 + 1) % 10 : (t * 7 + 3) % 10;
    sc.in[b1].P_corr += 20.0;
    sc.in[b2].P_corr += 35.0;
    SppSolution sol;
    ASSERT_TRUE(spp.solve(GTime::fromSec(1.4e9), sc.in, nullptr, false, Eigen::Vector3d::Zero(), 0.0, &sol));
    bool r1 = false, r2 = false, clean = false;
    for (const auto& s : sol.rejected) {
      if (s.rtk_sat == sc.in[b1].sat.rtk_sat) r1 = true;
      else if (s.rtk_sat == sc.in[b2].sat.rtk_sat) r2 = true;
      else clean = true;
    }
    if (r1 && r2) ++both;
    if (clean) ++any_clean;
  }
  const double rate = static_cast<double>(both) / trials;
  std::cout << "[U16] two simultaneous NLOS (+20/+35 m) both rejected in " << rate << " of trials, a clean satellite rejected in "
            << static_cast<double>(any_clean) / trials << "\n";
  EXPECT_GE(rate, 0.95) << "two NLOS detected in " << rate;
  EXPECT_LE(any_clean, trials / 20);
}

// Doppler LS: velocity and clock drift recovered; Sagnac-rate and lever terms present (§1 Doppler equation)
TEST(U16GnssOnly, DopplerLsRecoversVelocityAndDrift) {
  Config cfg;
  NoiseModel noise(cfg);
  RobustDopplerLs dls(cfg, noise);
  Scene sc = makeScene(9, 5, false);
  std::mt19937_64 rng(6);
  // antenna velocity = IMU velocity + R[ω]×l (the lever term is what the receiver sees)
  const Eigen::Vector3d v_imu_N(1.2, -0.4, 0.05), omega(0.0, 0.0, 1.0), lever(-0.316, 0.109, 0.401);
  const Eigen::Vector3d w_ant_N = v_imu_N + omega.cross(lever);
  const Eigen::Vector3d w_ant_E = sc.R_EN * w_ant_N;
  const double d_true = 0.7;
  fillDoppler(sc, w_ant_E, d_true, 0.0, rng);
  DopplerSolution sol;
  ASSERT_TRUE(dls.solve(GTime::fromSec(1.4e9), sc.in, sc.r_true, &sol));
  EXPECT_LT((sol.v_ecef - w_ant_E).norm(), 1e-6);
  EXPECT_NEAR(sol.d_mps, d_true, 1e-6);
  EXPECT_EQ(sol.n_used, 9);
  // the solution is the antenna velocity, not the IMU velocity: the lever term (0.4 m/s) is resolved
  EXPECT_NEAR((sol.v_ecef - sc.R_EN * v_imu_N).norm(), omega.cross(lever).norm(), 1e-6);
  // Sagnac-rate term present: dropping it — or flipping its sign (A36: 2× the term) — would leave a bias of that size
  double sagnac_max = 0.0;
  for (const auto& s : sc.in) sagnac_max = std::max(sagnac_max, std::fabs(sagnacRateNumeric(s.state.r_ecef, s.state.v_ecef, sc.r_true, w_ant_E)));
  EXPECT_GT(sagnac_max, 1e-3);   // mm/s-level term (ω_e·v_s·r/c), far above the 1e-6 accuracy asserted above
  // noisy + one 3 m/s blunder: rejected, velocity within a few cm/s
  Scene sn = makeScene(9, 7, false);
  fillDoppler(sn, w_ant_E, d_true, 0.05, rng);
  sn.in[4].dopp_hz -= 3.0 / sn.in[4].lambda_m;
  ASSERT_TRUE(dls.solve(GTime::fromSec(1.4e9), sn.in, sn.r_true, &sol));
  ASSERT_EQ(sol.rejected.size(), 1u);
  EXPECT_EQ(sol.rejected[0].rtk_sat, sn.in[4].sat.rtk_sat);
  EXPECT_EQ(sol.n_used, 8);
  // error against the solver's own formal covariance (unscaled by σ̂0): < 4σ in 3-D
  const double sigma_v = std::sqrt(sol.Q.topLeftCorner<3, 3>().trace()) / std::max(sol.sigma0, 1e-9);
  EXPECT_LT((sol.v_ecef - w_ant_E).norm(), 4.0 * sigma_v) << "formal σ_v = " << sigma_v;
  EXPECT_LT((sol.v_ecef - w_ant_E).norm(), 0.25);
}

// clock-only LS with a fixed antenna position: b, d and MAD² variances
TEST(U16GnssOnly, ClockOnlyLs) {
  Scene sc = makeScene(9, 8, true);
  std::mt19937_64 rng(9);
  fillPseudoranges(sc, 0.5, rng);
  const Eigen::Vector3d v_E = sc.R_EN * Eigen::Vector3d(1.0, 0.5, 0.0);
  fillDoppler(sc, v_E, 0.7, 0.05, rng);
  // one gross pseudorange and one gross Doppler: the medians must not move by more than the noise
  sc.in[2].P_corr += 30.0;
  sc.in[5].dopp_hz += 5.0 / sc.in[5].lambda_m;
  ClockOnlyResult r = clockOnlyLs(sc.in, sc.r_true, v_E);
  ASSERT_TRUE(r.ok);
  EXPECT_NEAR(r.b_m, sc.b_true, 1.0);
  EXPECT_NEAR(r.d_mps, 0.7, 0.1);
  EXPECT_GT(r.var_b, 0.0);
  EXPECT_LT(std::sqrt(r.var_b), 3.0);
  EXPECT_GT(r.var_d, 0.0);
  EXPECT_LT(std::sqrt(r.var_d), 0.3);
  EXPECT_EQ(r.n_used, 6);   // GPS-only for b (ISBs of GAL/BDS excluded)
  // MAD² scaling: with an exactly known spread the variance equals (1.4826·MAD)²
  std::vector<SppInput> in;
  for (int i = 0; i < 5; ++i) {
    SppInput s = sc.in[0];
    s.sat = sat(Sys::GPS, i + 1, i + 1);
    s.dopp_usable = false;
    const double rho = rangeSagnac(s.state.r_ecef, sc.r_true, nullptr);
    s.P_corr = rho + 100.0 - kC * s.state.dts + s.ion_m + s.trop_m + (i - 2) * 0.4;   // b_i = 100 + {−0.8,−0.4,0,0.4,0.8}
    in.push_back(s);
  }
  r = clockOnlyLs(in, sc.r_true, Eigen::Vector3d::Zero());
  ASSERT_TRUE(r.ok);
  EXPECT_NEAR(r.b_m, 100.0, 1e-9);
  EXPECT_NEAR(r.var_b, (1.4826 * 0.4) * (1.4826 * 0.4), 1e-7);
  // shifted antenna prediction: the clock absorbs the LOS-projected shift (diagnostic, not a failure)
  r = clockOnlyLs(in, sc.r_true + Eigen::Vector3d(0.5, 0.0, 0.0), Eigen::Vector3d::Zero());
  EXPECT_LT(std::fabs(r.b_m - 100.0), 0.6);
  // no GPS/QZSS code at all: b would be biased by the unknown ISBs (3.2 / −7.7 m here) ⇒ refused (review m7)
  {
    std::vector<SppInput> no_gps;
    for (const auto& s : sc.in)
      if (s.sat.sys != Sys::GPS) no_gps.push_back(s);
    ASSERT_GE(no_gps.size(), 3u);
    EXPECT_FALSE(clockOnlyLs(no_gps, sc.r_true, v_E).ok);
  }
}

// C/N0–elevation NLOS pre-screen flags the low-C/N0 outliers only
TEST(U16GnssOnly, NlosPrescreen) {
  Config cfg;
  std::mt19937_64 rng(13);
  std::normal_distribution<double> gauss(0.0, 1.0);
  // (a) noise-free: exactly the two suspects (−12 / −14 dB below the 32 + 16·sin(el) trend) are flagged
  {
    Scene sc = makeScene(10, 900, false);
    sc.in[2].cn0 -= 12.0;
    sc.in[6].cn0 -= 14.0;
    EXPECT_EQ(RobustSpp::nlosPrescreen(sc.in, cfg.init.nlos_cn0_margin_db), 2);
    for (int i = 0; i < 10; ++i) {
      EXPECT_EQ(sc.in[i].psr_usable, !(i == 2 || i == 6)) << "sat " << i;
      EXPECT_EQ(sc.in[i].dopp_usable, !(i == 2 || i == 6)) << "sat " << i;
    }
  }
  // (b) 1 dB C/N0 noise, 200 trials: ≥ 98 % of the suspects flagged (12 − 8 = 4 dB margin against ~1.4 dB of
  // combined measurement + fit noise), ≤ 1 % of the clean satellites flagged
  int flagged_bad = 0, flagged_good = 0;
  for (int t = 0; t < 200; ++t) {
    Scene sc = makeScene(10, 900 + t, false);
    for (auto& s : sc.in) s.cn0 += 1.0 * gauss(rng);
    const int b1 = t % 10, b2 = (b1 + 4) % 10;
    sc.in[b1].cn0 -= 12.0;
    sc.in[b2].cn0 -= 14.0;
    RobustSpp::nlosPrescreen(sc.in, cfg.init.nlos_cn0_margin_db);
    for (int i = 0; i < 10; ++i) {
      const bool bad = (i == b1 || i == b2);
      if (!sc.in[i].psr_usable) {
        if (bad) ++flagged_bad; else ++flagged_good;
        EXPECT_FALSE(sc.in[i].dopp_usable);
      }
    }
  }
  std::cout << "[U16] NLOS prescreen: suspects flagged " << flagged_bad << "/400, clean flagged " << flagged_good << "/1600\n";
  EXPECT_GE(flagged_bad, 392);
  EXPECT_LE(flagged_good, 16);
  // fewer than four usable satellites: nothing is decided
  Scene sc = makeScene(3, 1, false);
  sc.in[0].cn0 -= 20.0;
  EXPECT_EQ(RobustSpp::nlosPrescreen(sc.in, cfg.init.nlos_cn0_margin_db), 0);
  EXPECT_TRUE(sc.in[0].psr_usable);
}

// ---------------------------------------------------------------------------------------------
// AMENDMENTS A36 on real broadcast orbits for the GNSS-only Doppler estimators (RobustDopplerLs, clockOnlyLs).
// (The same numerical truth as U7's DopplerSagnacRateSignOnRealEphemeris; test files are self-contained.)
// ---------------------------------------------------------------------------------------------
namespace {
// Site of the dev-set fixture (Grass02 receiver_pvt, first record); only a real geometry is needed, not this point.
constexpr double kSiteLatDeg = 34.0339415, kSiteLonDeg = 108.7614865, kSiteH = 388.05;

std::string devSetBrdc() {
  for (const std::string& f : livo_gnss_io::testsupport::realFixtures(LIVO_GNSS_TEST_DATA_DIR)) {
    const std::string b = livo_gnss_io::testsupport::fixtureBrdc(f);
    if (!b.empty()) return b;
  }
  return std::string();
}

GTime epochInside(const rtk::NavStore& nav) {   // 1234.5 s after the median GPS toe: inside every system's window
  std::vector<GTime> toes;
  for (const rtk::EphKey& k : nav.coverage())
    if (k.sat.sys == Sys::GPS) toes.push_back(k.toe);
  if (toes.empty()) return GTime();
  std::sort(toes.begin(), toes.end());
  return toes[toes.size() / 2].add(1234.5);
}

double primaryLambda(const SatId& id, const rtk::NavStore& nav) {
  double f = 1575.42e6;
  if (id.sys == Sys::GLO) {
    const int k = nav.gloFcn(id.prn);
    f = 1602.0e6 + (k > -100 ? k : 0) * 0.5625e6;
  } else if (id.sys == Sys::BDS) {
    f = 1561.098e6;
  }
  return kC / f;
}

// Numerical Sagnac rate along the broadcast trajectory r_s(t) = SatSolver::ephPos(t) and a straight antenna path:
// d/dτ[geodist(r_s(t_tx+τ), r_A+v_Aτ) − ‖r_s(t_tx+τ) − (r_A+v_Aτ)‖] by central differences (h = 0.5 s); geodist − ‖·‖
// is exactly the Sagnac term, so the orbit's curvature cancels and only rounding (≲ 1e-8 m/s) is left. `rcv_share`:
// the same with the satellite held at r_s(t_tx).
bool sagnacTruth(const rtk::SatSolver& solver, const SatId& id, const GTime& t_tx, const Eigen::Vector3d& r_A,
                 const Eigen::Vector3d& v_A, double* sagnac, double* rcv_share, Eigen::Vector3d* rs0) {
  const double h = 0.5;
  Eigen::Vector3d rp, rm;
  double dts = 0.0, var = 0.0;
  if (!solver.ephPos(id, t_tx, rs0, &dts, &var) || !solver.ephPos(id, t_tx.add(h), &rp, &dts, &var) ||
      !solver.ephPos(id, t_tx.add(-h), &rm, &dts, &var))
    return false;
  auto rate = [&](const Eigen::Vector3d& sp, const Eigen::Vector3d& sm) {
    const Eigen::Vector3d ap = r_A + v_A * h, am = r_A - v_A * h;
    const Eigen::Vector3d dp = sp - ap, dm = sm - am;
    const double rho_dot = (rtk::geodist(sp, ap, nullptr) - rtk::geodist(sm, am, nullptr)) / (2.0 * h);
    return rho_dot - (dp - dm).dot(dp + dm) / (dp.norm() + dm.norm()) / (2.0 * h);
  };
  *sagnac = rate(rp, rm);
  *rcv_share = rate(*rs0, *rs0);
  return true;
}
}  // namespace

// Every satellite of the dev-set fixture's same-day IGS BRDC above 10° at the fixture's site. −λD carries the
// numerical Sagnac rate plus the estimator's own geometric rate e·(v_s − v_A) (same satellite velocity), so per
// satellite clockOnlyLs — fed that satellite's Doppler alone — returns d exactly iff its Sagnac rate equals the
// derivative (1e-6 m/s); RobustDopplerLs recovers v_A and d from all of them (1e-6). Parked antenna (satellite share
// only) and 30 m/s (antenna share ~1e-4 m/s as well). The resdop sign would miss by 2|rate| per satellite.
TEST(U16GnssOnly, DopplerSagnacRateSignOnRealEphemeris) {
  const std::string brdc = devSetBrdc();
  if (brdc.empty()) GTEST_SKIP() << "blocked: dev-set fixture with brdc/*.rnx (A24) under " << LIVO_GNSS_TEST_DATA_DIR;
  rtk::NavStore nav;
  std::string why;
  ASSERT_GT(nav.loadRinexNav(brdc, true, &why), 0) << brdc << ": " << why;
  const rtk::SatSolver solver(nav);
  const GTime t_rx = epochInside(nav);
  ASSERT_TRUE(t_rx.valid()) << "no GPS record in " << brdc;
  const Eigen::Vector3d r_A = geo2ecef(kSiteLatDeg * kDeg, kSiteLonDeg * kDeg, kSiteH);
  const Eigen::Matrix3d R_EN = enuRot(kSiteLatDeg * kDeg, kSiteLonDeg * kDeg);
  std::set<int> seen;
  std::vector<SatId> ids;
  for (const rtk::EphKey& k : nav.coverage())
    if (seen.insert(k.sat.rtk_sat).second) ids.push_back(k.sat);
  Config cfg;
  NoiseModel noise(cfg);
  RobustDopplerLs dls(cfg, noise);
  const double d_true = 0.37;
  for (const Eigen::Vector3d& v_N : {Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(24.0, -18.0, 0.2)}) {
    const Eigen::Vector3d v_A = R_EN * v_N;
    std::vector<SppInput> in;
    std::vector<double> truth, rcv_truth;
    int gps_index = -1;
    for (const SatId& id : ids) {
      rtk::SatState st;
      double P = 2.2e7;
      bool ok = true;
      for (int it = 0; it < 3 && ok; ++it) {   // a pseudorange consistent with the geometry: the real transmit time
        ok = solver.satState(id, t_rx, P, &st) && st.ok;
        if (ok) P = rtk::geodist(st.r_ecef, r_A, nullptr) - kC * st.dts;
      }
      if (!ok) continue;
      const Eigen::Vector3d e = (st.r_ecef - r_A).normalized();
      if (std::asin(e.dot(R_EN.col(2))) < 10.0 * kDeg) continue;
      double sag = 0.0, rcv = 0.0;
      Eigen::Vector3d rs0;
      ASSERT_TRUE(sagnacTruth(solver, id, st.t_tx, r_A, v_A, &sag, &rcv, &rs0)) << satIdStringOf(id);
      ASSERT_LT((rs0 - st.r_ecef).norm(), 1e-6) << satIdStringOf(id) << ": trajectory misses the estimator's satellite state";
      SppInput s;
      s.sat = id;
      s.state = st;
      s.lambda_m = primaryLambda(id, nav);
      s.dopp_hz = -(e.dot(st.v_ecef - v_A) + sag + d_true - kC * st.ddts) / s.lambda_m;
      s.var_dopp = 0.0025;
      s.dopp_usable = true;
      s.psr_usable = false;
      s.P_corr = P;
      if (gps_index < 0 && id.sys == Sys::GPS) gps_index = static_cast<int>(in.size());
      in.push_back(s);
      truth.push_back(sag);
      rcv_truth.push_back(rcv);
    }
    ASSERT_GE(in.size(), 12u) << "satellites above 10° at the fixture site, " << brdc;
    ASSERT_GE(gps_index, 0);
    // clockOnlyLs needs one GPS code for b (review m7); it contributes no Doppler here
    SppInput b_anchor = in[static_cast<size_t>(gps_index)];
    b_anchor.psr_usable = true;
    b_anchor.dopp_usable = false;
    int n_sat_share = 0, n_rcv_share = 0;
    double max_err = 0.0, max_rate = 0.0;
    std::set<Sys> systems;
    for (size_t q = 0; q < in.size(); ++q) {
      const ClockOnlyResult r = clockOnlyLs({b_anchor, in[q]}, r_A, v_A);
      ASSERT_TRUE(r.ok);
      const double err = std::fabs(r.d_mps - d_true);   // = |numerical Sagnac rate − the estimator's|
      EXPECT_LT(err, 1e-6) << satIdStringOf(in[q].sat) << " at |v_A| = " << v_N.norm() << " m/s: d/dt of ρ(t) gives Sagnac rate "
                           << truth[q] << " m/s";
      max_err = std::max(max_err, err);
      max_rate = std::max(max_rate, std::fabs(truth[q]));
      if (std::fabs(truth[q] - rcv_truth[q]) > 1e-4) ++n_sat_share;
      if (std::fabs(rcv_truth[q]) > 1e-4) ++n_rcv_share;
      systems.insert(in[q].sat.sys);
    }
    DopplerSolution sol;
    ASSERT_TRUE(dls.solve(t_rx, in, r_A, &sol));
    EXPECT_LT((sol.v_ecef - v_A).norm(), 1e-6) << "|v_A| = " << v_N.norm();
    EXPECT_NEAR(sol.d_mps, d_true, 1e-6);
    EXPECT_EQ(sol.n_used, static_cast<int>(in.size()));
    std::cout << "[U16] A36 on real orbits, |v_A| = " << v_N.norm() << " m/s: " << in.size() << " satellites of " << systems.size()
              << " systems; max |Sagnac rate| " << max_rate << " m/s, max |estimator − d/dt ρ(t)| " << max_err
              << " m/s; satellite share > 1e-4 on " << n_sat_share << ", antenna share > 1e-4 on " << n_rcv_share << "\n";
    EXPECT_GE(n_sat_share, 8) << "too few satellites discriminate the sign of the satellite-velocity share";
    if (v_N.norm() > 0.0) {
      EXPECT_GE(n_rcv_share, 3) << "too few satellites discriminate the sign of the antenna-velocity share";
    }
    EXPECT_GE(systems.size(), 3u);
  }
}
