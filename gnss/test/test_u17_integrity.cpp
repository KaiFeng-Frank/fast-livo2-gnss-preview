// SPDX-License-Identifier: BSD-3-Clause
// test_u17_integrity.cpp — TEST_PLAN §1 U17 (AMENDMENTS A9-2; P3-A1): the external judges of DESIGN §10 L7 and the
// degeneracy flag (IntegrityMonitor), the Schmidt switching they drive, and the outage/recovery machine of §11/§10 L6
// (OutageManager) with its two state actions (L6 REANCHOR, RECOVERY clock reset).
//   U17Integrity.VelocityAlarmHysteresis     LIVO_ALARM after 5 bad epochs (d² > χ²₁(0.999) and > 0.2 m/s), clear after
//                                            10 consistent ones; speeds, so a pure heading error cannot raise it
//   U17Integrity.HeadingJudge                20 epochs > max(5°, 3σ) ⇒ REINIT_YAW; slow/alarm epochs are no evidence
//   U17Integrity.DegeneracyIndexHysteresis   λ₁ of the translation Schur complement: < 400 ×10 sets, > 800 ×10 clears
//   U17Integrity.SchmidtSwitching            alarm on ⇒ the update leaves the host bit-exact; alarm off ⇒ it writes it
//   U17Outage.StateMachine                   OUTAGE after 1 s, RECOVERY after 1 s of ≥4 sats, reacquisition window,
//                                            clock-reset request, REINIT after 60 s only with a lost frame
//   U17Outage.L6Lockout                      20 consistent-but-rejected epochs ⇒ REANCHOR; with the Doppler ⇒ REINIT
//   U17Outage.ReanchorIsConsistentAugmentation, RecoveryClockResetIsConsistent   the two actions are consistent
//                                            augmentations (§2.4): the new states' only independent part is their noise
#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <random>
#include <vector>

#include <Eigen/Dense>

#include "internal/clock.hpp"
#include "internal/integrity.hpp"
#include "internal/outage.hpp"
#include "internal/rows.hpp"
#include "internal/rtk_adapter.hpp"
#include "internal/update.hpp"

using namespace livo_gnss;

namespace {

constexpr double kDeg = M_PI / 180.0;
constexpr double kC = 299792458.0;
constexpr double kOmgE = 7.2921151467e-5;
const double kNaN = std::numeric_limits<double>::quiet_NaN();

Eigen::MatrixXd randomPd(int n, double scale, std::mt19937_64& rng) {
  std::normal_distribution<double> g(0.0, 1.0);
  Eigen::MatrixXd A(n, n);
  for (int i = 0; i < n; ++i)
    for (int j = 0; j < n; ++j) A(i, j) = g(rng);
  return scale * (A * A.transpose() / n + 0.5 * Eigen::MatrixXd::Identity(n, n));
}

SatId satOf(Sys sys, int prn) {
  SatId s;
  s.sys = sys;
  s.prn = prn;
  s.rtk_sat = satNoOf(sys, prn);
  return s;
}

}  // namespace

// =============================================================================================
// L7 velocity alarm
// =============================================================================================
TEST(U17Integrity, VelocityAlarmHysteresis) {
  Config cfg;
  IntegrityMonitor mon(cfg.integrity);
  const Eigen::Matrix3d cov = Eigen::Matrix3d::Identity() * 1e-4;   // σ 1 cm/s per axis on both sides
  const Eigen::Vector3d vG(1.5, 0.2, 0.05);
  auto scaled = [&](double ds) { return Eigen::Vector3d(vG * ((vG.norm() + ds) / vG.norm())); };
  const Eigen::Vector3d bad = scaled(-0.3), good = scaled(0.01);
  auto feed = [&](const Eigen::Vector3d& w) { mon.onVelocity(vG, cov, w, cov, 0.01); };
  for (int i = 0; i < 4; ++i) {
    feed(bad);
    EXPECT_FALSE(mon.flags().livo_alarm) << i;
  }
  feed(good);   // a consistent epoch restarts the count
  for (int i = 0; i < 4; ++i) feed(bad);
  EXPECT_FALSE(mon.flags().livo_alarm);
  EXPECT_FALSE(mon.flags().alarm_changed);
  feed(bad);
  EXPECT_TRUE(mon.flags().livo_alarm) << "5 consecutive bad epochs";
  EXPECT_TRUE(mon.flags().alarm_changed);
  EXPECT_GT(mon.flags().d2_velocity, 10.0);
  mon.clearEdges();
  EXPECT_FALSE(mon.flags().alarm_changed);
  EXPECT_TRUE(mon.flags().livo_alarm);
  for (int i = 0; i < 9; ++i) feed(good);
  EXPECT_TRUE(mon.flags().livo_alarm) << "9 consistent epochs are not enough";
  feed(bad);   // restarts the clearing count
  for (int i = 0; i < 9; ++i) feed(good);
  EXPECT_TRUE(mon.flags().livo_alarm);
  EXPECT_FALSE(mon.flags().alarm_changed);
  feed(good);
  EXPECT_FALSE(mon.flags().livo_alarm) << "10 consecutive consistent epochs";
  EXPECT_TRUE(mon.flags().alarm_changed);
  mon.clearEdges();

  // the two conditions of a bad epoch: d² > χ²₁(1 − α) AND |Δ| > min_err_mps
  const double thr = chi2Quantile(1, 1.0 - cfg.integrity.velocity_alarm.alpha);
  EXPECT_NEAR(thr, 10.827566170662733, 1e-9);
  {
    IntegrityMonitor m2(cfg.integrity);
    const Eigen::Vector3d w_small = scaled(-0.15);   // d² huge but |Δ| < 0.2 m/s
    for (int i = 0; i < 10; ++i) m2.onVelocity(vG, cov, w_small, cov, 0.01);
    EXPECT_GT(m2.flags().d2_velocity, thr);
    EXPECT_FALSE(m2.flags().livo_alarm);
    // |Δ| = 0.3 m/s and d² just below / just above the threshold (Var along v = u·(2Σ)·u with Σ isotropic)
    const Eigen::Vector3d w3 = scaled(-0.3);
    for (double f : {0.99, 1.01}) {
      IntegrityMonitor m3(cfg.integrity);
      const double var_each = 0.3 * 0.3 / (thr * f) / 2.0;
      const Eigen::Matrix3d c3 = Eigen::Matrix3d::Identity() * var_each;
      for (int i = 0; i < 5; ++i) m3.onVelocity(vG, c3, w3, c3, 0.01);
      EXPECT_EQ(m3.flags().livo_alarm, f > 1.0) << "d² = " << m3.flags().d2_velocity << " vs " << thr;
    }
    // a heading error alone (same speed) is not a speed failure
    IntegrityMonitor m4(cfg.integrity);
    const Eigen::Vector3d w_rot = Eigen::AngleAxisd(20.0 * kDeg, Eigen::Vector3d::UnitZ()) * vG;
    for (int i = 0; i < 10; ++i) m4.onVelocity(vG, cov, w_rot, cov, 0.01);
    EXPECT_FALSE(m4.flags().livo_alarm);
    // disabled
    Config c5;
    c5.integrity.velocity_alarm.enable = false;
    IntegrityMonitor m5(c5.integrity);
    for (int i = 0; i < 10; ++i) m5.onVelocity(vG, cov, bad, cov, 0.01);
    EXPECT_FALSE(m5.flags().livo_alarm);
  }
}

// =============================================================================================
// L7 heading judge
// =============================================================================================
TEST(U17Integrity, HeadingJudge) {
  Config cfg;
  const Eigen::Matrix3d cov = Eigen::Matrix3d::Identity() * 1e-4;
  const Eigen::Vector3d vG(1.2, 0.9, 0.0);   // 1.5 m/s
  auto rot = [](const Eigen::Vector3d& v, double deg) { return Eigen::Vector3d(Eigen::AngleAxisd(deg * kDeg, Eigen::Vector3d::UnitZ()) * v); };
  {
    IntegrityMonitor mon(cfg.integrity);
    for (int i = 0; i < 19; ++i) {
      mon.onVelocity(vG, cov, rot(vG, 10.0), cov, 0.5 * kDeg);
      EXPECT_FALSE(mon.flags().reinit_yaw) << i;
    }
    EXPECT_NEAR(mon.flags().heading_err_deg, -10.0, 1e-9);
    mon.onVelocity(vG, cov, rot(vG, 10.0), cov, 0.5 * kDeg);
    EXPECT_TRUE(mon.flags().reinit_yaw) << "20 consecutive epochs > max(5°, 3σ)";
    mon.clearEdges();
    EXPECT_FALSE(mon.flags().reinit_yaw);
    // the count restarted: another 19 do not fire
    for (int i = 0; i < 19; ++i) mon.onVelocity(vG, cov, rot(vG, 10.0), cov, 0.5 * kDeg);
    EXPECT_FALSE(mon.flags().reinit_yaw);
  }
  {
    // 3σ dominates: σ_ψ = 4° ⇒ 3σ ≈ 12° > 10°: never
    IntegrityMonitor mon(cfg.integrity);
    for (int i = 0; i < 40; ++i) mon.onVelocity(vG, cov, rot(vG, 10.0), cov, 4.0 * kDeg);
    EXPECT_FALSE(mon.flags().reinit_yaw);
    // 5° floor: 4° errors with a tiny σ never count
    IntegrityMonitor m2(cfg.integrity);
    for (int i = 0; i < 40; ++i) m2.onVelocity(vG, cov, rot(vG, 4.0), cov, 0.01 * kDeg);
    EXPECT_FALSE(m2.flags().reinit_yaw);
  }
  {
    // slow epochs are no evidence: 10 bad, 5 slow (0.3 m/s), 10 bad ⇒ fires on the 20th bad one; a good one resets
    IntegrityMonitor mon(cfg.integrity);
    const Eigen::Vector3d slow = vG * (0.3 / vG.norm());
    for (int i = 0; i < 10; ++i) mon.onVelocity(vG, cov, rot(vG, 10.0), cov, 0.5 * kDeg);
    for (int i = 0; i < 5; ++i) mon.onVelocity(slow, cov, rot(slow, 30.0), cov, 0.5 * kDeg);
    for (int i = 0; i < 9; ++i) mon.onVelocity(vG, cov, rot(vG, 10.0), cov, 0.5 * kDeg);
    EXPECT_FALSE(mon.flags().reinit_yaw);
    mon.onVelocity(vG, cov, rot(vG, 10.0), cov, 0.5 * kDeg);
    EXPECT_TRUE(mon.flags().reinit_yaw);
    mon.clearEdges();
    for (int i = 0; i < 15; ++i) mon.onVelocity(vG, cov, rot(vG, 10.0), cov, 0.5 * kDeg);
    mon.onVelocity(vG, cov, vG, cov, 0.5 * kDeg);   // consistent
    for (int i = 0; i < 15; ++i) mon.onVelocity(vG, cov, rot(vG, 10.0), cov, 0.5 * kDeg);
    EXPECT_FALSE(mon.flags().reinit_yaw);
  }
  {
    // while LIVO_ALARM is on the LIVO velocity is the suspect: heading epochs do not count
    IntegrityMonitor mon(cfg.integrity);
    const Eigen::Vector3d fast = vG * (2.0 / vG.norm());
    for (int i = 0; i < 5; ++i) mon.onVelocity(vG, cov, rot(fast, 10.0), cov, 0.5 * kDeg);   // speed error 0.5 m/s ⇒ alarm
    ASSERT_TRUE(mon.flags().livo_alarm);
    for (int i = 0; i < 30; ++i) mon.onVelocity(vG, cov, rot(fast, 10.0), cov, 0.5 * kDeg);
    EXPECT_FALSE(mon.flags().reinit_yaw);
  }
}

// =============================================================================================
// degeneracy index
// =============================================================================================
TEST(U17Integrity, DegeneracyIndexHysteresis) {
  Config cfg;
  std::mt19937_64 rng(17);
  const Eigen::MatrixXd Pm_d = randomPd(HostIdx::dim, 1e-2, rng);
  const HostMat Pm = Pm_d;
  // LIO information gain Y = HᵀR⁻¹H on [θ, p] (6 columns) with a chosen translation Schur complement S: Y_θθ = A,
  // Y_θp = B, Y_pp = S + BᵀA⁻¹B ⇒ Schur(Y) = S exactly.
  const Eigen::Matrix3d A = randomPd(3, 2e5, rng);
  std::normal_distribution<double> g(0.0, 1.0);
  Eigen::Matrix3d Bm;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) Bm(i, j) = 3e3 * g(rng);
  const Eigen::Matrix3d U = Eigen::Quaterniond(g(rng), g(rng), g(rng), g(rng)).normalized().toRotationMatrix();
  auto postFor = [&](double lambda1) {
    const Eigen::Matrix3d S = U * Eigen::Vector3d(lambda1, 5e3, 9e3).asDiagonal() * U.transpose();
    Eigen::MatrixXd Y = Eigen::MatrixXd::Zero(HostIdx::dim, HostIdx::dim);
    Y.block<3, 3>(0, 0) = A;
    Y.block<3, 3>(0, 3) = Bm;
    Y.block<3, 3>(3, 0) = Bm.transpose();
    Y.block<3, 3>(3, 3) = S + Bm.transpose() * A.inverse() * Bm;
    const Eigen::MatrixXd Ppost = (Pm_d.inverse() + Y).inverse();
    return HostMat(Ppost);
  };
  IntegrityMonitor mon(cfg.integrity);
  // λ₁ and u₁ are the Schur complement's
  mon.onLioCovariance(Pm, postFor(1500.0));
  EXPECT_NEAR(mon.flags().lambda(0), 1500.0, 1e-4 * 1500.0);
  EXPECT_NEAR(mon.flags().lambda(1), 5e3, 1e-4 * 5e3);
  EXPECT_NEAR(mon.flags().lambda(2), 9e3, 1e-4 * 9e3);
  EXPECT_NEAR(std::fabs(mon.flags().u1.dot(U.col(0))), 1.0, 1e-6);
  EXPECT_FALSE(mon.flags().degenerate);
  mon.reset();
  for (int i = 0; i < 9; ++i) {
    mon.onLioCovariance(Pm, postFor(300.0));
    EXPECT_FALSE(mon.flags().degenerate) << i;
  }
  mon.onLioCovariance(Pm, postFor(1000.0));   // one good frame restarts the count
  for (int i = 0; i < 9; ++i) mon.onLioCovariance(Pm, postFor(300.0));
  EXPECT_FALSE(mon.flags().degenerate);
  mon.onLioCovariance(Pm, postFor(300.0));
  EXPECT_TRUE(mon.flags().degenerate) << "10 frames with λ₁ < 400 m⁻²";
  EXPECT_TRUE(mon.flags().degenerate_changed);
  mon.clearEdges();
  for (int i = 0; i < 30; ++i) mon.onLioCovariance(Pm, postFor(600.0));   // inside the hysteresis band
  EXPECT_TRUE(mon.flags().degenerate);
  for (int i = 0; i < 9; ++i) mon.onLioCovariance(Pm, postFor(1000.0));
  EXPECT_TRUE(mon.flags().degenerate);
  EXPECT_FALSE(mon.flags().degenerate_changed);
  mon.onLioCovariance(Pm, postFor(1000.0));
  EXPECT_FALSE(mon.flags().degenerate) << "10 frames with λ₁ > 800 m⁻²";
  EXPECT_TRUE(mon.flags().degenerate_changed);
  // the host's (I − G)P⁻ is not exactly symmetric: symmetrised, same answer
  HostMat asym = postFor(300.0);
  asym(3, 4) += 1e-12;
  IntegrityMonitor m2(cfg.integrity);
  m2.onLioCovariance(Pm, asym);
  EXPECT_NEAR(m2.flags().lambda(0), 300.0, 1e-3 * 300.0);
}

// =============================================================================================
// Schmidt switching: the alarm drives JointUpdater::Options::schmidt
// =============================================================================================
TEST(U17Integrity, SchmidtSwitching) {
  Config cfg;
  std::mt19937_64 rng(5);
  CoreState core;
  core.P = randomPd(HostIdx::dim, 1e-3, rng);
  AugBlock aug(cfg);
  Eigen::MatrixXd F3 = Eigen::MatrixXd::Zero(3, HostIdx::dim);
  F3.block<3, 3>(0, HostIdx::pos) = Eigen::Matrix3d::Identity();
  aug.addSlot(Slot::Trans, 0, "trans", Eigen::Vector3d(1.0, 2.0, 3.0), F3, Eigen::MatrixXd(3, 0), Eigen::Matrix3d::Identity(), core.P);
  const int N = HostIdx::dim + aug.dim();
  JointUpdater::RowFactory factory = [&](const CoreState& x, const AugBlock& a, std::vector<RowSet>* sets) {
    RowSet s;
    s.cls = MeasClass::PvtPos;
    for (int k = 0; k < 3; ++k) {
      MeasRow row;
      row.cls = MeasClass::PvtPos;
      row.sat = satOf(Sys::GPS, k + 1);
      row.H = Eigen::RowVectorXd::Zero(N);
      row.H(HostIdx::pos + k) = 1.0;
      row.H(HostIdx::dim + k) = 1.0;
      row.h = x.p(k) + a.x()(k);
      row.z = row.h + 0.3;
      row.nu = row.z - row.h;
      row.R = 0.5;
      s.rows.push_back(row);
    }
    sets->push_back(s);
  };
  IntegrityMonitor mon(cfg.integrity);
  const Eigen::Matrix3d cv = Eigen::Matrix3d::Identity() * 1e-4;
  const Eigen::Vector3d vG(1.5, 0.0, 0.0), w_bad(1.0, 0.0, 0.0);
  for (int i = 0; i < 5; ++i) mon.onVelocity(vG, cv, w_bad, cv, 0.01);
  ASSERT_TRUE(mon.flags().livo_alarm);
  JointUpdater upd(cfg);
  {
    JointUpdater::Options opt;
    opt.schmidt = mon.flags().livo_alarm;
    const CoreState c0 = core;
    const Eigen::VectorXd xg0 = aug.x();
    const auto res = upd.update(core, aug, factory, opt);
    ASSERT_TRUE(res.committed) << res.rollback_reason;
    EXPECT_EQ(std::memcmp(c0.P.data(), core.P.data(), sizeof(double) * HostIdx::dim * HostIdx::dim), 0) << "host P written in Schmidt mode";
    EXPECT_EQ(std::memcmp(c0.p.data(), core.p.data(), sizeof(double) * 3), 0);
    EXPECT_EQ(std::memcmp(c0.R.data(), core.R.data(), sizeof(double) * 9), 0);
    EXPECT_GT((aug.x() - xg0).norm(), 1e-3) << "the frame block must still take the update";
  }
  for (int i = 0; i < 10; ++i) mon.onVelocity(vG, cv, vG, cv, 0.01);
  ASSERT_FALSE(mon.flags().livo_alarm);
  {
    JointUpdater::Options opt;
    opt.schmidt = mon.flags().livo_alarm;
    const CoreState c0 = core;
    const auto res = upd.update(core, aug, factory, opt);
    ASSERT_TRUE(res.committed) << res.rollback_reason;
    EXPECT_GT((core.p - c0.p).norm(), 1e-6) << "alarm cleared: the host is updated again";
    EXPECT_NE(std::memcmp(c0.P.data(), core.P.data(), sizeof(double) * HostIdx::dim * HostIdx::dim), 0);
  }
}

// =============================================================================================
// OutageManager (DESIGN §11)
// =============================================================================================
TEST(U17Outage, StateMachine) {
  // Times on an exact binary grid (dt = 0.125 s) so that "more than 1 s" and "1 s" are decided without rounding.
  Config cfg;
  const double dt = 0.125;
  OutageManager om(cfg.outage, cfg.rejection, true);
  om.setReacqEpochs(cfg.iterate.reacq_epochs);
  double t = 100.0;
  om.onActivated(t);
  // reacquisition window after ACTIVATE: 5 epochs report 5..1, then 0
  for (int i = 0; i < 8; ++i) {
    t += dt;
    const OutageDecision d = om.step(t, true, 9, true, kNaN, 0.0, 1.0, 0.01, 1.0);
    EXPECT_EQ(d.reacq_epochs_left, std::max(0, 5 - i)) << i;
    EXPECT_EQ(d.phase, Phase::TRACKING);
  }
  // cycles without an epoch are not an outage until more than 1 s has passed since the last valid one
  const double t_valid = t;
  for (int i = 1; i <= 8; ++i) {
    t = t_valid + dt * i;
    const OutageDecision d = om.step(t, false, 0, false, kNaN, kNaN, 1.0, 0.01, 1.0);
    EXPECT_EQ(d.phase, Phase::TRACKING) << i;
  }
  int entered = 0;
  t = t_valid + 9 * dt;
  {
    const OutageDecision d = om.step(t, true, 0, false, kNaN, kNaN, 1.0, 0.01, 1.0);   // an epoch without any L0-valid satellite
    EXPECT_TRUE(d.entered_outage);
    EXPECT_EQ(d.phase, Phase::OUTAGE);
    EXPECT_EQ(d.outage_duration_s, 9 * dt);
    entered += d.entered_outage;
  }
  // 1–3 satellites: valid epochs, but no recovery
  for (int i = 0; i < 30; ++i) {
    t += dt;
    const OutageDecision d = om.step(t, true, 3, false, kNaN, kNaN, 1.0, 0.01, 2000.0);
    EXPECT_EQ(d.phase, Phase::OUTAGE);
    entered += d.entered_outage;
  }
  EXPECT_EQ(entered, 1);
  // ≥ 4 satellites for 1 s; a thin epoch in between restarts the settling
  for (int i = 0; i < 5; ++i) om.step(t += dt, true, 5, true, kNaN, kNaN, 1.0, 0.01, 2000.0);
  om.step(t += dt, true, 3, true, kNaN, kNaN, 1.0, 0.01, 2000.0);
  EXPECT_EQ(om.phase(), Phase::OUTAGE);
  const double t_settle = t + dt;
  OutageDecision rec;
  for (int i = 0; i < 40; ++i) {
    t += dt;
    rec = om.step(t, true, 5, true, kNaN, kNaN, 1.0, 0.01, 2000.0);
    if (rec.phase != Phase::OUTAGE) break;
  }
  EXPECT_EQ(t - t_settle, cfg.outage.recover_settle_s) << "RECOVERY after ≥4 satellites for recover_settle_s";
  EXPECT_TRUE(rec.start_recovery);
  EXPECT_TRUE(rec.left_outage);
  EXPECT_TRUE(rec.clock_reset_needed) << "σ_b = 2000 m > 1e3 m";
  EXPECT_EQ(rec.phase, Phase::RECOVERY);
  EXPECT_EQ(rec.reacq_epochs_left, 5);
  for (int i = 0; i < 4; ++i) {
    const OutageDecision d = om.step(t += dt, true, 5, true, kNaN, kNaN, 1.0, 0.01, 1.0);
    EXPECT_EQ(d.reacq_epochs_left, 4 - i);
    EXPECT_EQ(d.phase, i < 3 ? Phase::RECOVERY : Phase::TRACKING) << i;
  }
  EXPECT_EQ(om.phase(), Phase::TRACKING);
  // RECOVERY without the clock request when σ_b is small
  {
    OutageManager o2(cfg.outage, cfg.rejection, true);
    o2.onActivated(0.0);
    double tt = 0.0;
    o2.step(tt += 1.5, false, 0, false, kNaN, kNaN, 1.0, 0.01, 50.0);
    ASSERT_EQ(o2.phase(), Phase::OUTAGE);
    bool reset = true;
    for (int i = 0; i < 12 && o2.phase() == Phase::OUTAGE; ++i) reset = o2.step(tt += dt, true, 6, true, kNaN, kNaN, 1.0, 0.01, 50.0).clock_reset_needed;
    EXPECT_EQ(o2.phase(), Phase::RECOVERY);
    EXPECT_FALSE(reset);
  }
  // REINIT: only after 60 s of outage AND with a lost frame (σ_t > 10 m or σ_ψ > 0.1 rad)
  for (int variant = 0; variant < 3; ++variant) {
    OutageManager o3(cfg.outage, cfg.rejection, true);
    o3.onActivated(0.0);
    const double st = variant == 1 ? 20.0 : 1.0, sp = variant == 2 ? 0.2 : 0.01;
    double tt = 0.0;
    double t_reinit = -1.0;
    for (int i = 1; i <= 800; ++i) {
      tt = dt * i;
      const OutageDecision d = o3.step(tt, false, 0, false, kNaN, kNaN, st, sp, 1.0);
      if (d.reinit && t_reinit < 0.0) t_reinit = tt;
    }
    if (variant == 0) {
      EXPECT_LT(t_reinit, 0.0) << "a frame that is still known is kept through a 100 s outage";
      EXPECT_EQ(o3.phase(), Phase::OUTAGE);
    } else {
      EXPECT_EQ(t_reinit, 60.0 + dt) << "variant " << variant;
      o3.onReinitDone(tt);
      EXPECT_EQ(o3.phase(), Phase::TRACKING);
      EXPECT_EQ(o3.reacqEpochsLeft(), 0);
    }
  }
}

TEST(U17Outage, L6Lockout) {
  Config cfg;
  // pseudoranges at 1 Hz: the nine Doppler-only epochs in between carry NaN for the code class (no evidence)
  auto run = [&](bool l6, int n_bad_psr, double dopp_frac, bool break_at_10, OutageDecision* last) {
    OutageManager om(cfg.outage, cfg.rejection, l6);
    om.onActivated(0.0);
    double t = 0.0;
    int reanchors = 0, reinits = 0;
    for (int s = 0; s < n_bad_psr; ++s) {
      for (int e = 0; e < 10; ++e) {
        t += 0.1;
        double fp = kNaN;
        if (e == 0) fp = (break_at_10 && s == 10) ? 0.1 : 0.8;
        const OutageDecision d = om.step(t, true, 8, true, fp, dopp_frac, 1.0, 0.01, 1.0);
        reanchors += d.reanchor;
        reinits += d.reinit;
        if (d.reanchor || d.reinit) *last = d;
      }
    }
    return std::make_pair(reanchors, reinits);
  };
  OutageDecision last;
  EXPECT_EQ(run(true, 19, 0.0, false, &last).first, 0) << "19 locked-out code epochs";
  const auto r20 = run(true, 20, 0.0, false, &last);
  EXPECT_EQ(r20.first, 1) << "20 consecutive code epochs with L1 consistent and L4 > 50 %";
  EXPECT_EQ(r20.second, 0);
  EXPECT_TRUE(last.reanchor);
  EXPECT_FALSE(last.reanchor_yaw);
  EXPECT_EQ(last.reacq_epochs_left, 0) << "the reacquisition window starts with the next cycle";
  EXPECT_EQ(run(true, 29, 0.0, true, &last).first, 0) << "a clean code epoch restarts the count";
  EXPECT_EQ(run(true, 40, 0.0, false, &last).first, 2);
  EXPECT_EQ(run(false, 40, 0.0, false, &last).first, 0) << "T−pr arm: L6 off";
  // the Doppler locked out as well ⇒ ψ is wrong too ⇒ REINIT (reanchor_yaw), not a plain REANCHOR
  const auto rd = run(true, 20, 0.9, false, &last);
  EXPECT_EQ(rd.first, 0);
  EXPECT_EQ(rd.second, 1);
  EXPECT_TRUE(last.reanchor_yaw && last.reinit);
  // L1 not consistent (< 5 satellites / GNSS-only LS failed) is not a lock-out
  {
    OutageManager om(cfg.outage, cfg.rejection, true);
    om.onActivated(0.0);
    int n = 0;
    for (int s = 0; s < 40; ++s) n += om.step(1.0 + s, true, 4, false, 0.9, kNaN, 1.0, 0.01, 1.0).reanchor;
    EXPECT_EQ(n, 0);
  }
  // after REANCHOR the next epochs run the fixed three iterations
  {
    OutageManager om(cfg.outage, cfg.rejection, true);
    om.setReacqEpochs(5);
    om.onActivated(0.0);
    double t = 0.0;
    for (int i = 0; i < 6; ++i) om.step(t += 0.1, true, 8, true, kNaN, 0.0, 1.0, 0.01, 1.0);   // spend the ACTIVATE window
    for (int s = 0; s < 20; ++s) om.step(t += 1.0, true, 8, true, 0.8, kNaN, 1.0, 0.01, 1.0);
    EXPECT_EQ(om.step(t += 0.1, true, 8, true, kNaN, 0.0, 1.0, 0.01, 1.0).reacq_epochs_left, 5);
  }
}

// ---------------------------------------------------------------------------------------------
// The two actions: synthetic geometry around a real-looking site, a full registration/clock/ISB block
// ---------------------------------------------------------------------------------------------
namespace {

struct Scene {
  Config cfg;
  Leveling lev;
  AnchorFrame anchor;
  CoreState core;
  BridgeResult br;
  Eigen::Vector3d lever = Eigen::Vector3d(-0.31635, 0.10926, 0.4006);
  Eigen::Vector3d pivot = Eigen::Vector3d(3.0, -2.0, 0.5);
  std::vector<SppInput> in;
  Eigen::Vector3d r_true = Eigen::Vector3d::Zero(), v_true = Eigen::Vector3d::Zero();
};

// The block [tilt, yaw, trans, clk, isb_E, td] with a random joint covariance, host P random; the bridge is δ = 0.05 s
// with a noiseless snapshot (R_k = R etc.).
void buildScene(Scene* s, AugBlock* aug, uint32_t seed, bool with_td) {
  std::mt19937_64 rng(seed);
  std::normal_distribution<double> g(0.0, 1.0);
  s->cfg.time.estimate_online = with_td;
  Geodetic geo;
  geo.lat_rad = 34.03 * kDeg;
  geo.lon_rad = 108.76 * kDeg;
  geo.h_m = 388.0;
  s->anchor.geo = geo;
  s->anchor.r0_ecef = rtk::pos2ecef(geo);
  s->anchor.R_EN = rtk::enuRotation(geo);
  s->anchor.valid = true;
  s->core.R = Eigen::AngleAxisd(0.4, Eigen::Vector3d(0.1, 0.2, 1.0).normalized()).toRotationMatrix();
  s->core.p = Eigen::Vector3d(10.0, 4.0, 0.3);
  s->core.v = Eigen::Vector3d(1.1, 0.7, 0.02);
  s->core.g = Eigen::AngleAxisd(0.05, Eigen::Vector3d::UnitX()).toRotationMatrix() * Eigen::Vector3d(0.0, 0.0, -9.81);
  {
    const HostMat P = randomPd(HostIdx::dim, 1e-4, rng);
    s->core.P = 0.5 * (P + P.transpose());   // exactly symmetric (the kept-block comparisons below are bitwise)
  }
  s->lev.setRef(s->core.g);
  s->br.ok = true;
  s->br.delta = 0.05;
  s->br.t_k = 1000.0;
  s->br.t_g = 999.95;
  s->br.R_k = s->core.R;
  s->br.g_snap = s->core.g;
  s->br.omega_g = Eigen::Vector3d(0.01, -0.02, 0.1);
  s->br.a_bar = s->core.R.transpose() * (Eigen::Vector3d(0.1, 0.0, 0.0) - s->core.g);
  s->br.D = Eigen::Vector3d(0.001, -0.002, 0.0);
  s->br.E = Eigen::Vector3d(0.005, 0.0, 0.001);
  // registration/clock block with random joint covariance and host correlation
  const int n = 2 + 1 + 3 + 2 + 1 + (with_td ? 1 : 0);
  Eigen::VectorXd x0(n);
  const double vals[10] = {1e-4, -2e-4, 0.8, 5.0, -3.0, 1.0, 1234.5, 40.0, 3.0, 0.002};
  for (int i = 0; i < n; ++i) x0(i) = vals[i];
  aug->addSlot(Slot::Tilt, 0, "tilt", x0.segment<2>(0), Eigen::MatrixXd::Zero(2, HostIdx::dim), Eigen::MatrixXd(0, 0),
               Eigen::Matrix2d::Identity() * 2.25e-8, s->core.P);
  aug->addSlot(Slot::Yaw, 0, "yaw", x0.segment<1>(2), Eigen::MatrixXd::Zero(1, HostIdx::dim), Eigen::MatrixXd(0, 0),
               Eigen::MatrixXd::Constant(1, 1, 1e-4), s->core.P);
  Eigen::MatrixXd Ft = Eigen::MatrixXd::Zero(3, HostIdx::dim);
  Ft.block<3, 3>(0, HostIdx::pos) = -Eigen::Matrix3d::Identity();
  aug->addSlot(Slot::Trans, 0, "trans", x0.segment<3>(3), Ft, Eigen::MatrixXd(0, 0), randomPd(3, 4.0, rng), s->core.P);
  aug->addSlot(Slot::Clk, 0, "clk", x0.segment<2>(6), Eigen::MatrixXd::Zero(2, HostIdx::dim), Eigen::MatrixXd(0, 0),
               randomPd(2, 9.0, rng), s->core.P);
  aug->addSlot(Slot::Isb, sysBit(Sys::GAL), "isb_E", x0.segment<1>(8), Eigen::MatrixXd::Zero(1, HostIdx::dim), Eigen::MatrixXd(0, 0),
               Eigen::MatrixXd::Constant(1, 1, 4.0), s->core.P);
  if (with_td)
    aug->addSlot(Slot::Td, 0, "td", x0.segment<1>(9), Eigen::MatrixXd::Zero(1, HostIdx::dim), Eigen::MatrixXd(0, 0),
                 Eigen::MatrixXd::Constant(1, 1, 1e-4), s->core.P);
  // correlate the block a little (a PSD joint: Σ ← Σ + small rank-2 update over [host | aug])
  {
    const int N = HostIdx::dim + aug->dim();
    Eigen::MatrixXd S = aug->jointCov(s->core.P);
    Eigen::VectorXd u(N);
    for (int i = 0; i < N; ++i) u(i) = g(rng) * std::sqrt(S(i, i)) * 0.3;
    S += u * u.transpose();
    S = 0.5 * (S + S.transpose()).eval();
    HostMat P;
    aug->setFromJoint(S, &P);
    s->core.P = P;
  }
  // satellites: 10 GPS/GAL/BDS at spread az/el, 2.2e7 m
  const Eigen::Matrix3d& REN = s->anchor.R_EN;
  s->r_true = s->anchor.r0_ecef + REN * Eigen::Vector3d(12.0, 3.0, 0.4);
  s->v_true = REN * Eigen::Vector3d(1.2, 0.6, 0.0);
  const double azel[10][2] = {{10, 70}, {80, 40}, {150, 25}, {200, 55}, {260, 15}, {320, 35}, {30, 20}, {110, 60}, {240, 30}, {300, 75}};
  const Sys sys[10] = {Sys::GPS, Sys::GPS, Sys::GPS, Sys::GPS, Sys::GPS, Sys::GAL, Sys::GAL, Sys::GAL, Sys::BDS, Sys::BDS};
  for (int i = 0; i < 10; ++i) {
    const double az = azel[i][0] * kDeg, el = azel[i][1] * kDeg;
    const Eigen::Vector3d uN(std::cos(el) * std::sin(az), std::cos(el) * std::cos(az), std::sin(el));
    SppInput x;
    x.sat = satOf(sys[i], 3 + i);
    x.state.ok = true;
    x.state.r_ecef = s->r_true + REN * uN * 2.2e7;
    x.state.v_ecef = REN * Eigen::Vector3d(g(rng), g(rng), g(rng)) * 2000.0;
    x.state.dts = 1e-4 * g(rng);
    x.state.ddts = 1e-11 * g(rng);
    x.el_rad = el;
    x.az_rad = az;
    x.lambda_m = kC / 1575.42e6;
    const double b = 1300.0, d = 38.0, iota = sys[i] == Sys::GAL ? 3.0 : sys[i] == Sys::BDS ? -7.0 : 0.0;
    Eigen::Vector3d e;
    const Eigen::Vector3d dd = x.state.r_ecef - s->r_true;
    e = dd.normalized();
    const double rho = dd.norm() + kOmgE * (x.state.r_ecef.x() * s->r_true.y() - x.state.r_ecef.y() * s->r_true.x()) / kC;
    x.P_corr = rho + b + iota - kC * x.state.dts + 0.7 * g(rng);
    const double rate = e.dot(x.state.v_ecef - s->v_true) +
                        kOmgE / kC * (x.state.v_ecef.x() * s->r_true.y() + x.state.r_ecef.x() * s->v_true.y() -
                                      x.state.v_ecef.y() * s->r_true.x() - x.state.r_ecef.y() * s->v_true.x());
    x.dopp_hz = -(rate + d - kC * x.state.ddts + 0.03 * g(rng)) / x.lambda_m;
    x.var_psr = 1.0 + 0.3 / std::pow(std::sin(el), 2);
    x.var_dopp = 0.0025;
    x.psr_usable = x.dopp_usable = true;
    x.models_fixed = true;
    s->in.push_back(x);
  }
}

AntennaJacobians jacobiansAt(const Scene& s, const AugBlock& aug) {
  const NoiseModel nm(s.cfg);
  const RowBuilder rb(s.cfg, nm);
  LinPoint lp;
  lp.x = &s.core;
  lp.bridge = &s.br;
  lp.aug = &aug;
  lp.leveling = &s.lev;
  lp.anchor = &s.anchor;
  lp.lever_I = s.lever;
  lp.pivot_W = s.pivot;
  lp.gravity_est_en = true;
  lp.delta = s.br.delta;
  return rb.antennaJacobians(lp);
}

// Columns of the aug slots of `kinds` in the current layout.
std::vector<int> colsOf(const AugBlock& aug, std::initializer_list<Slot> kinds) {
  std::vector<int> c;
  for (const SlotInfo& si : aug.slots())
    for (Slot k : kinds)
      if (si.kind == k)
        for (int i = 0; i < si.dim; ++i) c.push_back(HostIdx::dim + si.offset + i);
  return c;
}

}  // namespace

TEST(U17Outage, ReanchorIsConsistentAugmentation) {
  for (bool with_td : {false, true}) {
    Scene s;
    AugBlock aug(s.cfg);
    buildScene(&s, &aug, 3, with_td);
    NoiseModel nm(s.cfg);
    RobustSpp spp_solver(s.cfg, nm);
    RobustDopplerLs dls(s.cfg, nm);
    SppSolution spp;
    ASSERT_TRUE(spp_solver.solve(GTime(), s.in, nullptr, false, Eigen::Vector3d::Zero(), 0.0, &spp));
    DopplerSolution dop;
    ASSERT_TRUE(dls.solve(GTime(), s.in, spp.r_ecef, &dop));
    const AntennaJacobians J0 = jacobiansAt(s, aug);
    // bookkeeping before: the slots that must not change
    const std::vector<int> keep0 = colsOf(aug, {Slot::Tilt, Slot::Yaw, Slot::Isb, Slot::Td});
    const Eigen::MatrixXd S0 = aug.jointCov(s.core.P);
    Eigen::VectorXd xkeep0(keep0.size());
    for (size_t i = 0; i < keep0.size(); ++i) xkeep0(i) = aug.x()(keep0[i] - HostIdx::dim);
    // a clone present ⇒ refused, nothing changed
    {
      AugBlock a2 = aug;
      a2.addSlot(Slot::Clone, 7, "clone_7", J0.a_N, J0.J_a.leftCols(HostIdx::dim), J0.J_a.rightCols(a2.dim()), Eigen::Matrix3d::Identity() * 1e-6,
                 s.core.P);
      const Eigen::VectorXd x_before = a2.x();
      std::string why;
      EXPECT_FALSE(reanchorFromSpp(&a2, jacobiansAt(s, a2), s.anchor, spp, dop, s.in, s.br.delta, s.cfg, s.core.P, &why));
      EXPECT_NE(why.find("clone"), std::string::npos) << why;
      EXPECT_EQ(a2.x(), x_before);
    }
    std::string why;
    ASSERT_TRUE(reanchorFromSpp(&aug, J0, s.anchor, spp, dop, s.in, s.br.delta, s.cfg, s.core.P, &why)) << why;
    // 1) the global antenna takes the SPP value …
    const AntennaJacobians J1 = jacobiansAt(s, aug);
    const Eigen::Vector3d aN_spp = s.anchor.R_EN.transpose() * (spp.r_ecef - s.anchor.r0_ecef);
    EXPECT_LT((J1.a_N - aN_spp).norm(), 1e-9);
    // … with the SPP covariance, uncorrelated with the host and with every kept state; a^N–b' carries the SPP's own
    // position–clock correlation. Reference: the SPP LS normal matrix built here from the rows (ISBs as nuisance).
    std::vector<const SppInput*> used;
    bool has_gal = false, has_bds = false;
    for (const SppInput& x : s.in)
      for (const SatId& u : spp.used)
        if (u == x.sat) {
          used.push_back(&x);
          has_gal |= x.sat.sys == Sys::GAL;
          has_bds |= x.sat.sys == Sys::BDS;
        }
    const int pc = 4 + (has_gal ? 1 : 0) + (has_bds ? 1 : 0);
    Eigen::MatrixXd Hs = Eigen::MatrixXd::Zero(static_cast<int>(used.size()), pc);
    Eigen::VectorXd ws(static_cast<int>(used.size()));
    for (size_t i = 0; i < used.size(); ++i) {
      const SppInput& x = *used[i];
      const Eigen::Vector3d e = (x.state.r_ecef - spp.r_ecef).normalized();
      Hs.row(i).head<4>() << -e.x() - kOmgE / kC * x.state.r_ecef.y(), -e.y() + kOmgE / kC * x.state.r_ecef.x(), -e.z(), 1.0;
      if (x.sat.sys == Sys::GAL) Hs(i, 4) = 1.0;
      if (x.sat.sys == Sys::BDS) Hs(i, has_gal ? 5 : 4) = 1.0;
      ws(i) = 1.0 / x.var_psr;
    }
    std::cout << "[U17] reanchor (td " << with_td << "): SPP used " << used.size() << "/10, σ̂0 " << spp.sigma0 << "\n";
    Eigen::MatrixXd Qs = (Hs.transpose() * ws.asDiagonal() * Hs).inverse() * std::max(1.0, spp.sigma0 * spp.sigma0);
    for (int i = 0; i < 3; ++i) Qs(i, i) = std::max(Qs(i, i), 9.0);
    const Eigen::Matrix3d Saa = s.anchor.R_EN.transpose() * Qs.topLeftCorner<3, 3>() * s.anchor.R_EN;
    const Eigen::MatrixXd S1 = aug.jointCov(s.core.P);
    const Eigen::Matrix3d V = J1.J_a * S1 * J1.J_a.transpose();
    EXPECT_LT((V - Saa).norm(), 1e-9 * Saa.norm()) << "V\n" << V << "\nSPP\n" << Saa;
    const Eigen::MatrixXd Xh = J1.J_a * S1.leftCols(HostIdx::dim);
    EXPECT_LT(Xh.cwiseAbs().maxCoeff(), 1e-9 * std::sqrt(Saa.norm() * s.core.P.norm()));
    const std::vector<int> keep1 = colsOf(aug, {Slot::Tilt, Slot::Yaw, Slot::Isb, Slot::Td});
    ASSERT_EQ(keep1.size(), keep0.size());
    for (size_t i = 0; i < keep1.size(); ++i) {
      const Eigen::Vector3d c = J1.J_a * S1.col(keep1[i]);
      EXPECT_LT(c.cwiseAbs().maxCoeff(), 1e-9 * std::sqrt(Saa.norm() * S1(keep1[i], keep1[i]))) << "kept state " << i;
      EXPECT_EQ(aug.x()(keep1[i] - HostIdx::dim), xkeep0(i)) << "kept state value " << i;
      for (size_t j = 0; j < keep1.size(); ++j) EXPECT_EQ(S1(keep1[i], keep1[j]), S0(keep0[i], keep0[j]));
    }
    const int oc = HostIdx::dim + aug.find(Slot::Clk);
    const Eigen::Vector3d Vab = J1.J_a * S1.col(oc);
    const Eigen::Vector3d Sab = s.anchor.R_EN.transpose() * Qs.block<3, 1>(0, 3);
    EXPECT_LT((Vab - Sab).norm(), 1e-9 * std::sqrt(Saa.norm() * Qs(3, 3)));
    // 2) the clock: b' = b_SPP + δ d, d' = d_LS
    EXPECT_NEAR(aug.clockBias(), spp.b_m + s.br.delta * dop.d_mps, 1e-9);
    EXPECT_NEAR(aug.clockDrift(), dop.d_mps, 1e-12);
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(S1);
    EXPECT_GE(es.eigenvalues().minCoeff(), -1e-9 * es.eigenvalues().maxCoeff());
  }
}

TEST(U17Outage, RecoveryClockResetIsConsistent) {
  Scene s;
  AugBlock aug(s.cfg);
  buildScene(&s, &aug, 5, true);
  const AntennaJacobians J0 = jacobiansAt(s, aug);
  const Eigen::MatrixXd S0 = aug.jointCov(s.core.P);
  const std::vector<int> rest0 = colsOf(aug, {Slot::Tilt, Slot::Yaw, Slot::Trans, Slot::Isb, Slot::Td});
  ClockOnlyResult co;
  std::string why;
  ASSERT_TRUE(recoveryClockReset(&aug, J0, s.anchor, s.in, s.br.delta, s.cfg, s.core.P, &co, &why)) << why;
  ASSERT_TRUE(co.ok);
  EXPECT_NEAR(aug.clockBias(), co.b_m + s.br.delta * co.d_mps, 1e-9);
  EXPECT_NEAR(aug.clockDrift(), co.d_mps, 1e-12);
  // the new clock is F·x + n: the combination b' − F_b·x (resp. d' − F_d·x) has only its own noise — zero covariance with
  // every other state (host and aug) — and that noise is the clock-only LS MAD² variance moved by δ.
  const Eigen::Matrix3d& REN = s.anchor.R_EN;
  const Eigen::Vector3d r_pred = s.anchor.r0_ecef + REN * J0.a_N;
  Eigen::Vector3d eb = Eigen::Vector3d::Zero(), ed = Eigen::Vector3d::Zero();
  int nb = 0, nd = 0;
  for (const SppInput& x : s.in) {
    const Eigen::Vector3d e = (x.state.r_ecef - r_pred).normalized();
    if (x.sat.sys == Sys::GPS) {
      eb += e;
      ++nb;
    }
    ed += e;
    ++nd;
  }
  eb /= nb;
  ed /= nd;
  const Eigen::MatrixXd S1 = aug.jointCov(s.core.P);
  const int oc = HostIdx::dim + aug.find(Slot::Clk);
  const std::vector<int> rest1 = colsOf(aug, {Slot::Tilt, Slot::Yaw, Slot::Trans, Slot::Isb, Slot::Td});
  ASSERT_EQ(rest1.size(), rest0.size());
  // F over the new layout: host columns + the rest (the clk columns of J_a/J_w are zero)
  const int N1 = static_cast<int>(S1.rows());
  Eigen::RowVectorXd Fb = Eigen::RowVectorXd::Zero(N1), Fd = Eigen::RowVectorXd::Zero(N1);
  const Eigen::RowVectorXd fb0 = eb.transpose() * REN * J0.J_a, fd0 = ed.transpose() * REN * J0.J_w;
  Fb.head(HostIdx::dim) = fb0.head(HostIdx::dim);
  Fd.head(HostIdx::dim) = fd0.head(HostIdx::dim);
  for (size_t i = 0; i < rest1.size(); ++i) {
    Fb(rest1[i]) = fb0(rest0[i]);
    Fd(rest1[i]) = fd0(rest0[i]);
  }
  Eigen::RowVectorXd zb = -(Fb + s.br.delta * Fd), zd = -Fd;
  zb(oc) += 1.0;
  zd(oc + 1) += 1.0;
  const Eigen::RowVectorXd cb = zb * S1, cd = zd * S1;
  for (int c = 0; c < N1; ++c) {
    if (c == oc || c == oc + 1) continue;
    EXPECT_LT(std::fabs(cb(c)), 1e-9 * std::sqrt(S1(c, c) * (co.var_b + 1.0))) << "b' noise correlated with column " << c;
    EXPECT_LT(std::fabs(cd(c)), 1e-9 * std::sqrt(S1(c, c) * (co.var_d + 1e-6))) << "d' noise correlated with column " << c;
  }
  const double vb = cb.dot(zb);
  const double vd = cd.dot(zd);
  ClockModel cm(s.cfg.process.clock);
  const Eigen::Matrix2d Qc = cm.Q(s.br.delta, 0.0, 0.0);
  EXPECT_NEAR(vb, co.var_b + s.br.delta * s.br.delta * co.var_d + Qc(0, 0), 1e-9 * (co.var_b + 1.0));
  EXPECT_NEAR(vd, co.var_d + Qc(1, 1), 1e-9 * (co.var_d + 1e-6));
  // every other state untouched
  for (size_t i = 0; i < rest1.size(); ++i)
    for (size_t j = 0; j < rest1.size(); ++j) EXPECT_EQ(S1(rest1[i], rest1[j]), S0(rest0[i], rest0[j]));
  // without a GPS code the reset is refused and nothing changes
  {
    Scene s2;
    AugBlock a2(s2.cfg);
    buildScene(&s2, &a2, 6, false);
    for (SppInput& x : s2.in)
      if (x.sat.sys == Sys::GPS) x.psr_usable = false;
    const Eigen::VectorXd xb = a2.x();
    std::string w2;
    EXPECT_FALSE(recoveryClockReset(&a2, jacobiansAt(s2, a2), s2.anchor, s2.in, s2.br.delta, s2.cfg, s2.core.P, nullptr, &w2));
    EXPECT_EQ(a2.x(), xb);
  }
}
