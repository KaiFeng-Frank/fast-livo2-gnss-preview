// SPDX-License-Identifier: BSD-3-Clause
// test_u09_augblock.cpp — TEST_PLAN §1 U9 (AugBlock, ClockModel, ClockJumpDetector; DESIGN §2.2–§2.4, §3, §10 L2,
// AMENDMENTS A9-2, A11, A34). Owner: WP-b.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <random>
#include <stdexcept>
#include <vector>

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <Eigen/QR>

#include "internal/aug_block.hpp"
#include "internal/clock.hpp"
#include "internal/leveling.hpp"

using namespace livo_gnss;

namespace {

std::mt19937& rng() {
  static std::mt19937 g(2026);
  return g;
}
double gauss() {
  static std::normal_distribution<double> nd(0.0, 1.0);
  return nd(rng());
}
Eigen::MatrixXd gaussian(int r, int c) {
  Eigen::MatrixXd M(r, c);
  for (int i = 0; i < r; ++i)
    for (int j = 0; j < c; ++j) M(i, j) = gauss();
  return M;
}
// Random SPD with eigenvalues in [scale/10, scale] (condition ≤ 10) so that 1e-12 comparisons are meaningful.
Eigen::MatrixXd randomSpd(int n, double scale = 1.0) {
  Eigen::HouseholderQR<Eigen::MatrixXd> qr(gaussian(n, n));
  Eigen::MatrixXd U = qr.householderQ();
  Eigen::VectorXd lam(n);
  std::uniform_real_distribution<double> ud(-1.0, 0.0);
  for (int i = 0; i < n; ++i) lam(i) = scale * std::pow(10.0, ud(rng()));
  Eigen::MatrixXd S = U * lam.asDiagonal() * U.transpose();
  return 0.5 * (S + S.transpose()).eval();
}
bool bitEqual(const Eigen::MatrixXd& a, const Eigen::MatrixXd& b) {
  if (a.rows() != b.rows() || a.cols() != b.cols()) return false;
  return a.size() == 0 || std::memcmp(a.data(), b.data(), sizeof(double) * a.size()) == 0;
}
double minEig(const Eigen::MatrixXd& S) {
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(0.5 * (S + S.transpose()).eval());
  return es.eigenvalues().minCoeff();
}

// J_l(w) by its power series Σ_{k≥0} [w]×^k/(k+1)!, independent of the closed form in Leveling::tiltLeftJacobian;
// 25 terms leave < |w|²⁵/26! (< 1e-19 for |w| ≤ 2).
Eigen::Matrix3d leftJacobianSeries(const Eigen::Vector3d& w) {
  const Eigen::Matrix3d K = skew(w);
  Eigen::Matrix3d term = Eigen::Matrix3d::Identity(), sum = Eigen::Matrix3d::Identity();
  for (int k = 1; k <= 25; ++k) {
    term = (term * K / static_cast<double>(k + 1)).eval();
    sum += term;
  }
  return sum;
}
// Exp([φ_x, φ_y, 0]) through Eigen's AngleAxis, independent of Leveling::expTilt.
Eigen::Matrix3d expTiltRef(const Eigen::Vector2d& phi) {
  const double th = phi.norm();
  if (th == 0.0) return Eigen::Matrix3d::Identity();
  return Eigen::AngleAxisd(th, Eigen::Vector3d(phi.x() / th, phi.y() / th, 0.0)).toRotationMatrix();
}
// Fourth-order central difference at 0: [f(−2h) − 8f(−h) + 8f(h) − f(2h)]/(12h), truncation h⁴|f⁽⁵⁾|/30, rounding
// ≈ 1.5ε|f|/h. For |f| ≈ 100 m (a 100 m lever) and h = 1e-3 rad both stay near 1e-11, so the T and J columns can be
// held to 1e-9 — a two-point difference at h = 1e-6 already has a 1e-8 rounding floor there.
template <typename F>
auto cd4(const F& f, double h) -> decltype(f(0.0)) {
  return (f(-2.0 * h) - 8.0 * f(-h) + 8.0 * f(h) - f(2.0 * h)) / (12.0 * h);
}
constexpr double kHAngle = 1e-3;   // cd4 step for ψ and φ [rad]
constexpr double kHGrav = 1e-2;    // cd4 step for g [m/s²] (C(g) varies on the scale ‖g‖ ≈ 9.8)

// Registration and δg columns of J_a at δ = 0 as the exact derivatives of a^N = M(ψ, φ, g)(a^W − o^W) + t (DESIGN
// §4.2 with AMENDMENTS A34): δψ ↦ [e_z]× M y, δφ ↦ Ξ(y) J_l(φ̃) E_xy, δg ↦ Ξ(y) Exp(φ̃) [e_z]× G_⊥ (gravity_est_en),
// Ξ(y) = −R_z(ψ)[Exp(φ̃) C(g) y]×, y = a^W − o^W. J_l and Exp are built here independently of Leveling (series,
// AngleAxis), and RepivotKeepsGlobalAntenna checks these columns against central differences of a^N before it uses
// them, so the invariance test is about T against the true derivative, not about T against a shared approximation.
struct RegJac {
  Eigen::Vector3d a_N;
  Eigen::Matrix<double, 3, Eigen::Dynamic> J;   // 3 × (19 + n)
};
RegJac antennaJacobian(const AugBlock& aug, const Leveling& lev, const CoreState& x, const Eigen::Vector3d& lever,
                       const Eigen::Vector3d& pivot, bool gravity_est_en) {
  RegJac out;
  const int n = aug.dim();
  out.J.setZero(3, HostIdx::dim + n);
  const double psi = aug.yaw();
  const Eigen::Vector2d phi = aug.tilt();
  const Eigen::Matrix3d Rz = Leveling::Rz(psi);
  const Eigen::Matrix3d ExpPhi = expTiltRef(phi);
  const Eigen::Matrix3d Jl = leftJacobianSeries(Eigen::Vector3d(phi.x(), phi.y(), 0.0));
  const Eigen::Matrix3d Cg = lev.C(x.g);
  const Eigen::Matrix3d M = Rz * ExpPhi * Cg;
  const Eigen::Vector3d a_W = x.p + x.R * lever;
  const Eigen::Vector3d y = a_W - pivot;
  out.a_N = lev.M(psi, phi, x.g) * y + aug.trans();   // the value through the production M()
  const Eigen::Matrix3d Xi = -Rz * skew(Eigen::Vector3d(ExpPhi * Cg * y));
  out.J.block<3, 3>(0, HostIdx::rot) = -M * x.R * skew(lever);
  out.J.block<3, 3>(0, HostIdx::pos) = M;
  if (gravity_est_en) out.J.block<3, 3>(0, HostIdx::grav) = Xi * ExpPhi * lev.epsJacobian(x.g);
  const int it = aug.find(Slot::Tilt), iy = aug.find(Slot::Yaw), itr = aug.find(Slot::Trans);
  if (it >= 0) {
    const Eigen::Matrix3d XiJl = Xi * Jl;
    out.J.block<3, 2>(0, HostIdx::dim + it) = XiJl.leftCols<2>();
  }
  if (iy >= 0) out.J.block<3, 1>(0, HostIdx::dim + iy) = Eigen::Vector3d::UnitZ().cross(Eigen::Vector3d(M * y));
  if (itr >= 0) out.J.block<3, 3>(0, HostIdx::dim + itr).setIdentity();
  return out;
}

CoreState randomCore() {
  CoreState x;
  x.R = so3Exp(Eigen::Vector3d(0.4 * gauss(), 0.4 * gauss(), 2.0 * gauss()));
  x.p = 5.0 * Eigen::Vector3d(gauss(), gauss(), gauss());
  x.v = Eigen::Vector3d(gauss(), gauss(), gauss());
  x.bg = 1e-3 * Eigen::Vector3d(gauss(), gauss(), gauss());
  x.ba = 1e-2 * Eigen::Vector3d(gauss(), gauss(), gauss());
  x.g = Eigen::Vector3d(0.08, -0.15, -9.79);
  return x;
}

// Builds a block with the registration + clock slots and an arbitrary joint covariance.
void makeBlock(const Config& cfg, AugBlock* aug, HostMat* P, double yaw = 0.7, Eigen::Vector2d tilt = Eigen::Vector2d(1e-4, -2e-4),
               Eigen::Vector3d trans = Eigen::Vector3d(10.0, -5.0, 2.0)) {
  aug->clear();
  const HostMat P0 = randomSpd(HostIdx::dim).cast<double>();
  const Eigen::MatrixXd none;
  auto zeroF = [](int d) { return Eigen::MatrixXd(Eigen::MatrixXd::Zero(d, HostIdx::dim)); };
  aug->addSlot(Slot::Tilt, 0, "tilt", tilt, zeroF(2), none, Eigen::MatrixXd::Identity(2, 2), P0);
  aug->addSlot(Slot::Yaw, 0, "yaw", Eigen::VectorXd::Constant(1, yaw), zeroF(1), none, Eigen::MatrixXd::Identity(1, 1), P0);
  aug->addSlot(Slot::Trans, 0, "trans", trans, zeroF(3), none, Eigen::MatrixXd::Identity(3, 3), P0);
  aug->addSlot(Slot::Clk, 0, "clk", Eigen::Vector2d(120.0, 0.4), zeroF(2), none, Eigen::MatrixXd::Identity(2, 2), P0);
  (void)cfg;
  const int n = aug->dim();
  const Eigen::MatrixXd S = randomSpd(HostIdx::dim + n);
  aug->setFromJoint(S, P);
}

}  // namespace

// under the pivot-coupled walk the current point mean and variance are unchanged
TEST(U09AugBlock, PivotNoiseLeavesCurrentPointInvariant) {
  Config cfg;
  AugBlock aug(cfg);
  HostMat P;
  makeBlock(cfg, &aug, &P);
  Leveling lev;
  const Eigen::Vector3d g_W(0.08, -0.15, -9.79);
  lev.setRef(g_W);
  const Eigen::Vector3d a_W(30.0, 40.0, 1.0), pivot(1.0, 2.0, 0.5);
  const Eigen::Matrix3d M = lev.M(aug.yaw(), aug.tilt(), g_W);
  const double ds = 2.0, dalpha = 0.3, dt = 0.1, lambda = 0.19, wz_dt = 0.3;
  Eigen::MatrixXd Phi, Qp;
  aug.processModel(ds, dalpha, dt, M, a_W, pivot, lambda, wz_dt, &Phi, &Qp);
  const int n = aug.dim();
  ASSERT_EQ(Phi.rows(), n);
  ASSERT_EQ(Qp.rows(), n);
  EXPECT_LT((Qp - Qp.transpose()).cwiseAbs().maxCoeff(), 1e-18);
  EXPECT_GE(minEig(Qp), -1e-18);

  // ∂m/∂ξ of the current global point m = M(a^W − o^W) + t, exact at φ ≠ 0: the tilt column is Ξ J_l(φ̃) E_xy (J_l by
  // its power series here, independent of Leveling). §3.2's walk has rotation increment Δr = R_z J_l(φ̃) E_xy w_φ + e_z w_ψ
  // (AMENDMENTS A45 / R6); its φ = 0 form R_z E_xy w_φ would leave a residue of up to (½|φ̃||u|)² q_φ Δs in the current
  // point (≈ 8e-11 of rot_scale below, far above the 1e-13 tolerance; block (6) shows it at a larger tilt).
  // δθ/δp are irrelevant here.
  const Eigen::Vector3d u = M * (a_W - pivot);
  const Eigen::Matrix3d Rz = Leveling::Rz(aug.yaw());
  const Eigen::Matrix3d ExpPhi = Leveling::expTilt(aug.tilt());
  const Eigen::Matrix3d Jl = leftJacobianSeries(Eigen::Vector3d(aug.tilt().x(), aug.tilt().y(), 0.0));
  const Eigen::Matrix3d Xi = -Rz * skew(Eigen::Vector3d(ExpPhi * lev.C(g_W) * (a_W - pivot)));
  Eigen::MatrixXd Jm = Eigen::MatrixXd::Zero(3, n);
  Jm.block<3, 2>(0, aug.find(Slot::Tilt)) = (Xi * Jl).leftCols<2>();
  Jm.block<3, 1>(0, aug.find(Slot::Yaw)) = Eigen::Vector3d::UnitZ().cross(u);
  Jm.block<3, 3>(0, aug.find(Slot::Trans)).setIdentity();
  // (1) only the translation white noise reaches the current point: diag(q_th, q_th, q_tv)·Δs
  const Eigen::Matrix3d Pm = Jm * Qp * Jm.transpose();
  Eigen::Matrix3d expect = Eigen::Matrix3d::Zero();
  expect(0, 0) = cfg.process.frame.q_th_per_m * ds;
  expect(1, 1) = cfg.process.frame.q_th_per_m * ds;
  expect(2, 2) = cfg.process.frame.q_tv_per_m * ds;
  // the rotation-noise contributions cancel: their magnitude before cancellation is ~|u|²·q_ψ·Δs
  const double rot_scale = u.squaredNorm() * (cfg.process.frame.q_yaw_per_m * ds + cfg.process.frame.q_yaw_per_rad * dalpha);
  EXPECT_LT((Pm - expect).cwiseAbs().maxCoeff(), 1e-13 * rot_scale + 1e-20) << "rotation noise leaked into the current point";
  // (2) with the translation noise off the current point's variance is exactly untouched
  Config cfg0 = cfg;
  cfg0.process.frame.q_th_per_m = 0.0;
  cfg0.process.frame.q_tv_per_m = 0.0;
  AugBlock aug0(cfg0);
  HostMat P0;
  makeBlock(cfg0, &aug0, &P0, aug.yaw(), aug.tilt(), aug.trans());
  Eigen::MatrixXd Phi0, Qp0;
  aug0.processModel(ds, dalpha, dt, M, a_W, pivot, lambda, wz_dt, &Phi0, &Qp0);
  EXPECT_LT((Jm * Qp0 * Jm.transpose()).cwiseAbs().maxCoeff(), 1e-13 * rot_scale + 1e-20);
  EXPECT_GT(Qp0.diagonal().segment<3>(aug0.find(Slot::Trans)).minCoeff(), 0.0);   // t still walks (coupled to Δr)
  // (3) nonlinear: applying a sampled rotation increment with Δt = [u]×Δr, Δr = R_z J_l(φ̃) E_xy w_φ + e_z w_ψ (R6),
  //     moves m only at second order in Δr — no O(|φ̃|·|Δr|) term left (the bound before R6 had to allow one)
  for (int k = 0; k < 20; ++k) {
    const Eigen::Vector2d w_phi = 1e-4 * Eigen::Vector2d(gauss(), gauss());
    const double w_psi = 1e-4 * gauss();
    const Eigen::Vector3d dr = (Rz * Jl).leftCols<2>() * w_phi + Eigen::Vector3d::UnitZ() * w_psi;
    const Eigen::Vector3d t_new = aug.trans() + u.cross(dr);
    const Eigen::Matrix3d M_new = lev.M(aug.yaw() + w_psi, aug.tilt() + w_phi, g_W);
    const Eigen::Vector3d m_old = M * (a_W - pivot) + aug.trans();
    const Eigen::Vector3d m_new = M_new * (a_W - pivot) + t_new;
    EXPECT_LT((m_new - m_old).norm(), 20.0 * u.norm() * dr.squaredNorm() + 1e-12) << "sample " << k;
  }
  // (4) mean propagation: Φ is identity on the registration, the clock block is [[1, dt], [0, 1]]
  EXPECT_TRUE((Phi.topLeftCorner(6, 6) - Eigen::MatrixXd::Identity(6, 6)).isZero(0.0));
  const int ic = aug.find(Slot::Clk);
  EXPECT_EQ(Phi(ic, ic + 1), dt);
  ClockModel cm(cfg.process.clock);
  cm.windup_epoch_dt_s = aug.windupEpochDt();
  EXPECT_TRUE((Qp.block<2, 2>(ic, ic) - cm.Q(dt, wz_dt / dt, lambda)).isZero(0.0));
  // ... and against the independent Van Loan reference with the effective (wind-up-bounded) S_b
  {
    const double w = (wz_dt / dt) * lambda / (2.0 * M_PI);
    const double sb_eff = std::max(cfg.process.clock.S_b, w * w * aug.windupEpochDt());
    const Eigen::Matrix2d Qv = ClockModel::QVanLoan(dt, sb_eff, cfg.process.clock.S_d);
    EXPECT_LT((Qp.block<2, 2>(ic, ic) - Qv).cwiseAbs().maxCoeff(), 1e-12 * Qv.cwiseAbs().maxCoeff());
  }
  const Eigen::VectorXd x_before = aug.x();
  aug.propagate(HostMat::Identity(), Phi, Qp);
  EXPECT_NEAR(aug.clockBias(), x_before(ic) + dt * x_before(ic + 1), 1e-12);
  EXPECT_EQ(aug.clockDrift(), x_before(ic + 1));
  EXPECT_EQ(aug.yaw(), x_before(aug.find(Slot::Yaw)));
  // (5) Δs = Δα = Δt = 0 (standstill) adds no noise at all
  Eigen::MatrixXd Phi1, Qp1;
  aug.processModel(0.0, 0.0, 0.0, M, a_W, pivot, lambda, 0.0, &Phi1, &Qp1);
  EXPECT_TRUE(Qp1.isZero(0.0));
  EXPECT_TRUE(Phi1.isIdentity(0.0));
  // (6) AMENDMENTS A45 / R6 at a large tilt residual (|φ| = 0.05, a long walk; |u| ≈ 48 m), where the φ = 0 form of the
  //     walk is far from exact: (1)'s exact invariance holds at the same 1e-13·rot_scale, the φ = 0 linearisation of
  //     the current point sees a leak ≥ 1e3× that tolerance, and in the nonlinear check (3) the φ = 0 increment
  //     R_z E_xy w_φ misses the second-order bound that the exact increment meets (w_φ ⟂ φ, where Exp(φ̃ + w̃) and
  //     Exp(w̃)Exp(φ̃) differ at first order). No random draws from the file's generator (later tests keep theirs).
  {
    const Eigen::Vector2d tilt_big(0.03, -0.04);
    AugBlock ab(cfg);
    const HostMat Pi = HostMat::Identity();
    const Eigen::MatrixXd none;
    auto zF = [](int d) { return Eigen::MatrixXd(Eigen::MatrixXd::Zero(d, HostIdx::dim)); };
    ab.addSlot(Slot::Tilt, 0, "tilt", tilt_big, zF(2), none, Eigen::MatrixXd::Identity(2, 2), Pi);
    ab.addSlot(Slot::Yaw, 0, "yaw", Eigen::VectorXd::Constant(1, 0.7), zF(1), none, Eigen::MatrixXd::Identity(1, 1), Pi);
    ab.addSlot(Slot::Trans, 0, "trans", Eigen::Vector3d(10.0, -5.0, 2.0), zF(3), none, Eigen::MatrixXd::Identity(3, 3), Pi);
    const Eigen::Matrix3d Mb = lev.M(ab.yaw(), ab.tilt(), g_W);
    Eigen::MatrixXd Phib, Qb;
    ab.processModel(ds, dalpha, dt, Mb, a_W, pivot, lambda, wz_dt, &Phib, &Qb);
    const Eigen::Vector3d ub = Mb * (a_W - pivot);
    const Eigen::Matrix3d Rzb = Leveling::Rz(ab.yaw());
    const Eigen::Matrix3d Jlb = leftJacobianSeries(Eigen::Vector3d(tilt_big.x(), tilt_big.y(), 0.0));
    const Eigen::Matrix3d Xib = -Rzb * skew(Eigen::Vector3d(Leveling::expTilt(tilt_big) * lev.C(g_W) * (a_W - pivot)));
    Eigen::MatrixXd Jmb = Eigen::MatrixXd::Zero(3, ab.dim());
    Jmb.block<3, 2>(0, ab.find(Slot::Tilt)) = (Xib * Jlb).leftCols<2>();
    Jmb.block<3, 1>(0, ab.find(Slot::Yaw)) = Eigen::Vector3d::UnitZ().cross(ub);
    Jmb.block<3, 3>(0, ab.find(Slot::Trans)).setIdentity();
    Eigen::MatrixXd Jm0 = Jmb;   // the φ = 0 linearisation of the current point (§3.2 before R6)
    Jm0.block<3, 2>(0, ab.find(Slot::Tilt)) = Xib.leftCols<2>();
    const double rot_b = ub.squaredNorm() * (cfg.process.frame.q_yaw_per_m * ds + cfg.process.frame.q_yaw_per_rad * dalpha);
    const double leak_exact = (Jmb * Qb * Jmb.transpose() - expect).cwiseAbs().maxCoeff();
    const double leak_first = (Jm0 * Qb * Jm0.transpose() - expect).cwiseAbs().maxCoeff();
    EXPECT_LT(leak_exact, 1e-13 * rot_b + 1e-20) << "rotation noise leaked into the current point at |φ| = 0.05 (A45/R6)";
    EXPECT_GT(leak_first, 1e3 * (1e-13 * rot_b + 1e-20)) << "the φ = 0 linearisation is indistinguishable at |φ| = 0.05";
    const Eigen::Vector2d w_phi = 1e-5 * Eigen::Vector2d(0.8, 0.6);   // ⟂ tilt_big
    const double w_psi = 0.3e-5;
    const Eigen::Vector3d dr_exact = (Rzb * Jlb).leftCols<2>() * w_phi + Eigen::Vector3d::UnitZ() * w_psi;
    const Eigen::Vector3d dr_first = Rzb.leftCols<2>() * w_phi + Eigen::Vector3d::UnitZ() * w_psi;
    const Eigen::Matrix3d Mb_new = lev.M(ab.yaw() + w_psi, ab.tilt() + w_phi, g_W);
    auto moved = [&](const Eigen::Vector3d& dr) { return (Mb_new * (a_W - pivot) + ab.trans() + ub.cross(dr) - (ub + ab.trans())).norm(); };
    const double bound = 20.0 * ub.norm() * dr_exact.squaredNorm() + 1e-12;
    EXPECT_LT(moved(dr_exact), bound);
    EXPECT_GT(moved(dr_first), 10.0 * bound) << "the φ = 0 increment meets the second-order bound at |φ| = 0.05";
    std::printf("[U9 R6] |phi| 0.05, |u| %.1f m: current-point leak exact %.1e / phi=0 form %.1e (tol %.1e); nonlinear "
                "move exact %.1e / phi=0 increment %.1e (bound %.1e)\n",
                ub.norm(), leak_exact, leak_first, 1e-13 * rot_b, moved(dr_exact), moved(dr_first), bound);
  }
}

// two-state clock Q equals the Van Loan matrix exponential
TEST(U09AugBlock, ClockQMatchesVanLoan) {
  struct Case {
    double dt, S_b, S_d;
  };
  const std::vector<Case> cases{{0.1, 0.1, 0.1}, {1.0, 0.05, 2.0}, {0.033, 3.0, 0.01}, {5.0, 0.1, 0.1}, {0.001, 1e-3, 1e-4}, {30.0, 0.2, 0.3}};
  for (const Case& c : cases) {
    Config::Process::Clock cc;
    cc.S_b = c.S_b;
    cc.S_d = c.S_d;
    cc.S_b_min_windup = false;
    const ClockModel cm(cc);
    const Eigen::Matrix2d Q = cm.Q(c.dt, 0.0, 0.0);
    const Eigen::Matrix2d Qv = ClockModel::QVanLoan(c.dt, c.S_b, c.S_d);
    const double scale = Q.cwiseAbs().maxCoeff();
    EXPECT_LT((Q - Qv).cwiseAbs().maxCoeff(), 1e-12 * scale) << "dt " << c.dt;
    EXPECT_NEAR(Q(0, 1), Q(1, 0), 0.0);
    Eigen::Matrix2d Phi_expect;
    Phi_expect << 1.0, c.dt, 0.0, 1.0;
    EXPECT_TRUE((cm.Phi(c.dt) - Phi_expect).isZero(0.0));
    // the wind-up bound only raises S_b, only when it binds, and by (|ω_z|λ/2π)²·Δt_epoch — the epoch interval, not
    // the step this Q is discretised over
    cc.S_b_min_windup = true;
    for (double epoch_dt : {0.1, 1.0}) {
      ClockModel cw(cc);
      cw.windup_epoch_dt_s = epoch_dt;
      const double omega = 3.0, lambda = 0.1903;
      const Eigen::Matrix2d Qw = cw.Q(c.dt, omega, lambda);
      const double w = omega * lambda / (2.0 * M_PI);
      const double sb_eff = std::max(c.S_b, w * w * epoch_dt);
      EXPECT_NEAR(Qw(0, 0), sb_eff * c.dt + c.S_d * c.dt * c.dt * c.dt / 3.0, 1e-15 * (1.0 + Qw(0, 0))) << "dt " << c.dt << " epoch " << epoch_dt;
      EXPECT_NEAR(Qw(1, 1), Q(1, 1), 0.0);
      EXPECT_NEAR(Qw(0, 1), Q(0, 1), 0.0);
      EXPECT_TRUE((cw.Q(c.dt, 0.0, lambda) - Q).isZero(0.0));
      // the Van Loan reference with the bounded PSD is the same matrix
      EXPECT_LT((Qw - ClockModel::QVanLoan(c.dt, sb_eff, c.S_d)).cwiseAbs().maxCoeff(), 1e-12 * Qw.cwiseAbs().maxCoeff());
    }
  }
  EXPECT_TRUE(ClockModel(Config::Process::Clock{}).Q(0.0, 1.0, 0.19).isZero(0.0));
  EXPECT_TRUE(ClockModel(Config::Process::Clock{}).Q(-1.0, 1.0, 0.19).isZero(0.0));
  EXPECT_EQ(ClockModel(Config::Process::Clock{}).windup_epoch_dt_s, 0.1);   // nominal M3DGR epoch interval

  // The property the bound exists for (DESIGN §2.2): over one epoch the clock variance accumulated through the LIO
  // steps that tile it is at least the squared wind-up of that epoch, whatever the LIO rate. 15 Hz LIO against a
  // 10 Hz epoch: the epoch (0.1 s) is tiled by steps of 1/15 s and 1/30 s with different rates.
  {
    Config::Process::Clock cc;
    cc.S_b = 1e-9;   // so that the bound is what carries the variance
    cc.S_d = 0.1;
    cc.S_b_min_windup = true;
    ClockModel cw(cc);
    const double epoch = 0.1, lambda = 0.1903;
    cw.windup_epoch_dt_s = epoch;
    const std::vector<double> steps{1.0 / 15.0, 1.0 / 30.0};
    const std::vector<double> omegas{1.0, 0.4};
    double q00 = 0.0, windup = 0.0, q00_lio_rule = 0.0;
    for (size_t i = 0; i < steps.size(); ++i) {
      q00 += cw.Q(steps[i], omegas[i], lambda)(0, 0) - cc.S_d * std::pow(steps[i], 3) / 3.0;   // S_b part only
      windup += omegas[i] * steps[i] * lambda / (2.0 * M_PI);
      const double w = omegas[i] * lambda / (2.0 * M_PI);
      q00_lio_rule += std::max(cc.S_b, w * w * steps[i]) * steps[i];   // the rejected "bound with Δt_LIO" rule
    }
    EXPECT_GE(q00, windup * windup * (1.0 - 1e-12));
    EXPECT_LT(q00_lio_rule, windup * windup);   // the per-step rule would have under-covered the epoch (≈ 61 % here)
    // one uniform-rate epoch of constant |ω_z| gives exactly the squared wind-up
    double q_uni = 0.0;
    for (double s : {0.025, 0.025, 0.05}) q_uni += cw.Q(s, 2.0, lambda)(0, 0) - cc.S_d * s * s * s / 3.0;
    const double wu = 2.0 * epoch * lambda / (2.0 * M_PI);
    EXPECT_NEAR(q_uni, wu * wu, 1e-12 * wu * wu);
  }
}

// after add/remove the remaining covariance is bit-identical
TEST(U09AugBlock, AddRemoveSlotLeavesRestBitIdentical) {
  Config cfg;
  AugBlock aug(cfg);
  HostMat P;
  makeBlock(cfg, &aug, &P);
  const int n = aug.dim();
  const Eigen::VectorXd x0 = aug.x();
  const Eigen::MatrixXd C0 = aug.C(), Q0 = aug.Q();
  const std::vector<SlotInfo> slots0 = aug.slots();
  // consistent augmentation of a 3-dim clone c = f(x_host, x_G) + n
  const Eigen::MatrixXd Fh = gaussian(3, HostIdx::dim), Fa = gaussian(3, n), N = randomSpd(3, 1e-4);
  const Eigen::Vector3d c0(1.0, 2.0, 3.0);
  const int idx = aug.addSlot(Slot::Clone, 17, "clone_17", c0, Fh, Fa, N, P);
  ASSERT_EQ(aug.dim(), n + 3);
  EXPECT_EQ(aug.find(Slot::Clone, 17), n);
  EXPECT_EQ(aug.slots()[idx].name, "clone_17");
  EXPECT_TRUE(bitEqual(aug.x().head(n), x0));
  EXPECT_TRUE(bitEqual(aug.C().leftCols(n), C0));
  EXPECT_TRUE(bitEqual(aug.Q().topLeftCorner(n, n), Q0));
  EXPECT_TRUE(bitEqual(aug.x().tail(3), c0));
  // the new blocks against the explicit joint construction Σ' = F Σ Fᵀ + N with F = [[I, 0], [Fh, Fa]]
  Eigen::MatrixXd S0(HostIdx::dim + n, HostIdx::dim + n);
  S0 << P, C0, C0.transpose(), Q0;
  Eigen::MatrixXd F(HostIdx::dim + n + 3, HostIdx::dim + n);
  F.setZero();
  F.topLeftCorner(HostIdx::dim + n, HostIdx::dim + n).setIdentity();
  F.bottomLeftCorner(3, HostIdx::dim) = Fh;
  F.bottomRightCorner(3, n) = Fa;
  Eigen::MatrixXd S1 = F * S0 * F.transpose();
  S1.bottomRightCorner(3, 3) += N;
  const Eigen::MatrixXd S_aug = aug.jointCov(P);
  EXPECT_LT((S_aug - S1).cwiseAbs().maxCoeff(), 1e-13 * S1.cwiseAbs().maxCoeff());
  EXPECT_LT((S_aug - S_aug.transpose()).cwiseAbs().maxCoeff(), 1e-15 * S1.cwiseAbs().maxCoeff());
  // remove it again: bit-identical to before
  aug.removeSlot(idx);
  ASSERT_EQ(aug.dim(), n);
  EXPECT_TRUE(bitEqual(aug.x(), x0));
  EXPECT_TRUE(bitEqual(aug.C(), C0));
  EXPECT_TRUE(bitEqual(aug.Q(), Q0));
  ASSERT_EQ(aug.slots().size(), slots0.size());
  for (size_t i = 0; i < slots0.size(); ++i) {
    EXPECT_EQ(aug.slots()[i].offset, slots0[i].offset);
    EXPECT_EQ(aug.slots()[i].dim, slots0[i].dim);
    EXPECT_EQ(aug.slots()[i].name, slots0[i].name);
  }
  // remove a middle slot (yaw, 1 dim between tilt and trans): the survivors keep their bits, offsets shift
  const int iy = aug.find(Slot::Yaw);
  int yaw_idx = -1;
  for (size_t i = 0; i < aug.slots().size(); ++i)
    if (aug.slots()[i].kind == Slot::Yaw) yaw_idx = static_cast<int>(i);
  aug.removeSlot(yaw_idx);
  ASSERT_EQ(aug.dim(), n - 1);
  EXPECT_FALSE(aug.has(Slot::Yaw));
  EXPECT_EQ(aug.find(Slot::Trans), iy);   // shifted down by one
  std::vector<int> keep;
  for (int i = 0; i < n; ++i)
    if (i != iy) keep.push_back(i);
  for (size_t a = 0; a < keep.size(); ++a) {
    EXPECT_EQ(aug.x()(a), x0(keep[a]));
    for (int r = 0; r < HostIdx::dim; ++r) EXPECT_EQ(aug.C()(r, a), C0(r, keep[a]));
    for (size_t b = 0; b < keep.size(); ++b) EXPECT_EQ(aug.Q()(a, b), Q0(keep[a], keep[b]));
  }
  EXPECT_THROW(aug.removeSlot(99), std::out_of_range);
  EXPECT_THROW(aug.addSlot(Slot::Tilt, 0, "tilt", Eigen::Vector2d::Zero(), Eigen::MatrixXd::Zero(2, HostIdx::dim), Eigen::MatrixXd(),
                           Eigen::MatrixXd::Identity(2, 2), P),
               std::invalid_argument);   // duplicate
}

// Re-pivot with A11's full T and A34's exact columns: the global antenna position and its covariance change by < 1e-9,
// φ ≠ 0 included (σ_DOV-sized tilts and two long-walk sizes on the closed-form branch of Exp/J_l). The reference
// Jacobian is first checked against central differences of a^N, the t rows of T against central differences of
// t' = t + M(ψ, φ, g)(o' − o), all at 1e-9; A11's φ = 0 columns (Ξ(o')E_xy, Ξ(o')[e_z]×G_⊥) must fail the same
// criteria at every φ ≠ 0, so the tolerance separates the exact T from the first-order one.
TEST(U09AugBlock, RepivotKeepsGlobalAntenna) {
  Config cfg;
  const std::vector<Eigen::Vector2d> tilts{Eigen::Vector2d::Zero(),         Eigen::Vector2d(1.2e-4, -0.8e-4), Eigen::Vector2d(-2.1e-4, 1.4e-4),
                                           Eigen::Vector2d(0.5e-4, 2.6e-4), Eigen::Vector2d(-1.7e-4, -1.9e-4), Eigen::Vector2d(3e-3, -2e-3),
                                           Eigen::Vector2d(0.04, 0.03)};
  for (size_t trial = 0; trial < tilts.size(); ++trial) {
    AugBlock aug(cfg);
    HostMat P;
    const Eigen::Vector2d tilt = tilts[trial];
    makeBlock(cfg, &aug, &P, 2.0 * gauss(), tilt, Eigen::Vector3d(50.0 * gauss(), 50.0 * gauss(), 5.0 * gauss()));
    Leveling lev;
    CoreState x = randomCore();
    lev.setRef(x.g);   // g = g_ref: the ε Jacobian is the exact derivative of C(·) here (leveling.hpp)
    const Eigen::Vector3d lever(-0.31635, 0.10926, 0.4006);
    const Eigen::Vector3d pivot_old = x.p + Eigen::Vector3d(80.0 * gauss(), 80.0 * gauss(), 3.0 * gauss());   // ~100 m away
    const Eigen::Vector3d a_W = x.p + x.R * lever;
    const bool gest = (trial % 2 == 0);
    const double psi = aug.yaw();
    const Eigen::Vector3d t_before = aug.trans();
    const int it = aug.find(Slot::Tilt), iy = aug.find(Slot::Yaw), itr = aug.find(Slot::Trans);

    const RegJac before = antennaJacobian(aug, lev, x, lever, pivot_old, gest);
    // (0) the reference is the true derivative: its ψ, φ, g columns against central differences of the global antenna
    //     a^N(ψ, φ, g) = M(ψ, φ, g)(a^W − o) + t
    {
      auto aN = [&](double ps, const Eigen::Vector2d& ph, const Eigen::Vector3d& g) {
        return Eigen::Vector3d(lev.M(ps, ph, g) * (a_W - pivot_old) + t_before);
      };
      EXPECT_LT((before.a_N - aN(psi, tilt, x.g)).norm(), 1e-12);
      const Eigen::Vector3d n_psi = cd4([&](double s) { return aN(psi + s, tilt, x.g); }, kHAngle);
      EXPECT_LT((n_psi - before.J.col(HostIdx::dim + iy)).norm(), 1e-9 * (1.0 + n_psi.norm())) << "trial " << trial;
      for (int k = 0; k < 2; ++k) {
        const Eigen::Vector3d num = cd4([&](double s) { Eigen::Vector2d p = tilt; p(k) += s; return aN(psi, p, x.g); }, kHAngle);
        EXPECT_LT((num - before.J.col(HostIdx::dim + it + k)).norm(), 1e-9 * (1.0 + num.norm())) << "trial " << trial << " tilt col " << k;
      }
      for (int k = 0; gest && k < 3; ++k) {
        const Eigen::Vector3d num = cd4([&](double s) { Eigen::Vector3d g = x.g; g(k) += s; return aN(psi, tilt, g); }, kHGrav);
        EXPECT_LT((num - before.J.col(HostIdx::grav + k)).norm(), 1e-9 * (1.0 + num.norm())) << "trial " << trial << " g col " << k;
      }
    }
    const Eigen::MatrixXd S_before = aug.jointCov(P);
    const Eigen::Matrix3d cov_before = before.J * S_before * before.J.transpose();
    const Eigen::VectorXd xg_before = aug.x();

    const Eigen::MatrixXd T_aug = aug.repivot(pivot_old, a_W, lev, x.g, P, gest);
    const int n = aug.dim();
    ASSERT_EQ(T_aug.rows(), n);
    ASSERT_EQ(T_aug.cols(), HostIdx::dim + n);

    // (1) U9: global antenna position and covariance unchanged (< 1e-9 absolute and 1e-12 relative)
    const RegJac after = antennaJacobian(aug, lev, x, lever, a_W, gest);
    const Eigen::MatrixXd S_after = aug.jointCov(P);
    const Eigen::Matrix3d cov_after = after.J * S_after * after.J.transpose();
    EXPECT_LT((after.a_N - before.a_N).norm(), 1e-9) << "trial " << trial;
    EXPECT_LT((cov_after - cov_before).cwiseAbs().maxCoeff(), 1e-9) << "trial " << trial;
    EXPECT_LT((cov_after - cov_before).cwiseAbs().maxCoeff(), 1e-12 * cov_before.cwiseAbs().maxCoeff()) << "trial " << trial;
    // Σ' = T Σ Tᵀ with the returned aug rows of T (host rows = [I 0]), and the identity behind (1): J' T = J
    Eigen::MatrixXd T(HostIdx::dim + n, HostIdx::dim + n);
    T.setZero();
    T.topLeftCorner(HostIdx::dim, HostIdx::dim).setIdentity();
    T.bottomRows(n) = T_aug;
    const Eigen::MatrixXd S_ref = T * S_before * T.transpose();
    EXPECT_LT((S_after - S_ref).cwiseAbs().maxCoeff(), 1e-13 * S_ref.cwiseAbs().maxCoeff()) << "trial " << trial;
    EXPECT_LT((after.J * T - before.J).cwiseAbs().maxCoeff(), 1e-12 * before.J.cwiseAbs().maxCoeff()) << "trial " << trial;
    EXPECT_TRUE(bitEqual(S_after.topLeftCorner(HostIdx::dim, HostIdx::dim), P));   // P is the host's
    // only t moved: t' = t + M(o' − o)
    for (int i = 0; i < n; ++i)
      if (i < itr || i >= itr + 3) { EXPECT_EQ(aug.x()(i), xg_before(i)); }
    const Eigen::Matrix3d M = lev.M(aug.yaw(), aug.tilt(), x.g);
    EXPECT_LT((aug.trans() - (xg_before.segment<3>(itr) + M * (a_W - pivot_old))).norm(), 1e-12);

    // (2) T's t rows against central differences of t'(ψ, φ, g) = t + M(ψ, φ, g)(o' − o): every column to 1e-9,
    //     φ ≠ 0 included (A34: Ξ(o') J_l(φ̃) E_xy and Ξ(o') Exp(φ̃) [e_z]× G_⊥)
    const Eigen::Vector3d d = a_W - pivot_old;
    auto tprime = [&](double ps, const Eigen::Vector2d& ph, const Eigen::Vector3d& g) { return Eigen::Vector3d(lev.M(ps, ph, g) * d); };
    std::vector<Eigen::Vector3d> num_phi(2), num_g(3);
    double worst_col = 0.0, cov11_dev = 0.0;   // reported below: relative column error of T, A11 covariance change
    {
      const Eigen::Vector3d num = cd4([&](double s) { return tprime(psi + s, tilt, x.g); }, kHAngle);
      const Eigen::Vector3d ana = T_aug.block<3, 1>(itr, HostIdx::dim + iy);
      EXPECT_LT((num - ana).norm(), 1e-9 * (1.0 + num.norm())) << "trial " << trial << " yaw col";
      worst_col = std::max(worst_col, (num - ana).norm() / (1.0 + num.norm()));
    }
    for (int k = 0; k < 2; ++k) {
      num_phi[k] = cd4([&](double s) { Eigen::Vector2d p = tilt; p(k) += s; return tprime(psi, p, x.g); }, kHAngle);
      const Eigen::Vector3d ana = T_aug.block<3, 1>(itr, HostIdx::dim + it + k);
      EXPECT_LT((num_phi[k] - ana).norm(), 1e-9 * (1.0 + num_phi[k].norm())) << "trial " << trial << " tilt col " << k;
      worst_col = std::max(worst_col, (num_phi[k] - ana).norm() / (1.0 + num_phi[k].norm()));
    }
    for (int k = 0; k < 3; ++k) {
      num_g[k] = cd4([&](double s) { Eigen::Vector3d g = x.g; g(k) += s; return tprime(psi, tilt, g); }, kHGrav);
      const Eigen::Vector3d ana = T_aug.block<3, 1>(itr, HostIdx::grav + k);
      if (gest) {
        EXPECT_LT((num_g[k] - ana).norm(), 1e-9 * (1.0 + num_g[k].norm())) << "trial " << trial << " g col " << k;
        worst_col = std::max(worst_col, (num_g[k] - ana).norm() / (1.0 + num_g[k].norm()));
      } else {
        EXPECT_TRUE(ana.isZero(0.0));
      }
    }

    // (3) discrimination: A11's φ = 0 columns give the same T at φ = 0 (J_l(0) = Exp(0) = I exactly; compared to 1e-15
    //     relative, not bitwise, because this TU and aug_block.cpp may contract FMAs differently in the default
    //     profile) and fail (1) and (2) by orders of magnitude at every φ ≠ 0 here
    {
      const Eigen::Matrix3d Rz = Leveling::Rz(psi);
      const Eigen::Matrix3d Xi_d = -Rz * skew(Eigen::Vector3d(Leveling::expTilt(tilt) * lev.C(x.g) * d));
      Eigen::MatrixXd T11 = T;
      T11.block<3, 2>(HostIdx::dim + itr, HostIdx::dim + it) = Xi_d.leftCols<2>();
      if (gest) T11.block<3, 3>(HostIdx::dim + itr, HostIdx::grav) = Xi_d * lev.epsJacobian(x.g);
      if (tilt.isZero(0.0)) {
        EXPECT_LT((T11 - T).cwiseAbs().maxCoeff(), 1e-15 * T.cwiseAbs().maxCoeff()) << "A34's columns must reduce to A11's at φ = 0";
      } else {
        const Eigen::Matrix3d cov11 = after.J * (T11 * S_before * T11.transpose()) * after.J.transpose();
        EXPECT_GT((cov11 - cov_before).cwiseAbs().maxCoeff(), 1e3 * 1e-9) << "trial " << trial << ": A11's T passes the covariance criterion";
        cov11_dev = (cov11 - cov_before).cwiseAbs().maxCoeff();
        for (int k = 0; k < 2; ++k)
          EXPECT_GT((num_phi[k] - Xi_d.col(k)).norm(), 1e2 * 1e-9 * (1.0 + num_phi[k].norm())) << "trial " << trial << " tilt col " << k;
        for (int k = 0; gest && k < 3; ++k) {
          const Eigen::Vector3d a11 = Xi_d * lev.epsJacobian(x.g).col(k);
          EXPECT_GT((num_g[k] - a11).norm(), 1e2 * 1e-9 * (1.0 + num_g[k].norm())) << "trial " << trial << " g col " << k;
        }
      }
    }
    std::printf("[U9 repivot] trial %zu |phi| %.1e gest %d: T col rel err %.1e, cov change %.1e (rel %.1e), A11 cov change %.1e\n", trial,
                tilt.norm(), gest ? 1 : 0, worst_col, (cov_after - cov_before).cwiseAbs().maxCoeff(),
                (cov_after - cov_before).cwiseAbs().maxCoeff() / cov_before.cwiseAbs().maxCoeff(), cov11_dev);
    // T is the identity everywhere else
    Eigen::MatrixXd T_id = T_aug;
    T_id.block(itr, 0, 3, HostIdx::dim + n).setZero();
    T_id.block<3, 3>(itr, HostIdx::dim + itr).setIdentity();
    Eigen::MatrixXd I_aug = Eigen::MatrixXd::Zero(n, HostIdx::dim + n);
    I_aug.rightCols(n).setIdentity();
    EXPECT_TRUE((T_id - I_aug).isZero(0.0));
    // the joint stays PSD
    EXPECT_GE(minEig(S_after), -1e-12 * S_after.cwiseAbs().maxCoeff());
  }
  // without a trans slot re-pivot is the identity
  AugBlock empty(cfg);
  Leveling lev;
  lev.setRef(Eigen::Vector3d(0, 0, -9.8));
  const Eigen::MatrixXd T0 = empty.repivot(Eigen::Vector3d::Zero(), Eigen::Vector3d(1, 2, 3), lev, Eigen::Vector3d(0, 0, -9.8), HostMat::Identity(), true);
  EXPECT_EQ(T0.rows(), 0);
}

// parking and consistent (re)augmentation
TEST(U09AugBlock, ParkActivateConsistentAugmentation) {
  Config cfg;
  AugBlock aug(cfg);
  const HostMat P = randomSpd(HostIdx::dim).cast<double>();
  EXPECT_EQ(aug.dim(), 0);                 // parked = absent
  EXPECT_FALSE(aug.has(Slot::Trans));
  EXPECT_EQ(aug.jointCov(P).rows(), HostIdx::dim);
  EXPECT_TRUE(bitEqual(aug.jointCov(P), P));
  // explicit joint construction alongside
  Eigen::MatrixXd S = P;
  auto augment = [&](const Eigen::MatrixXd& Fh, const Eigen::MatrixXd& Fa, const Eigen::MatrixXd& N) {
    const int n = static_cast<int>(S.rows()) - HostIdx::dim;
    const int d = static_cast<int>(N.rows());
    Eigen::MatrixXd F = Eigen::MatrixXd::Zero(HostIdx::dim + n + d, HostIdx::dim + n);
    F.topLeftCorner(HostIdx::dim + n, HostIdx::dim + n).setIdentity();
    F.bottomLeftCorner(d, HostIdx::dim) = Fh;
    if (n > 0 && Fa.size() > 0) F.bottomRightCorner(d, n) = Fa;
    Eigen::MatrixXd S2 = F * S * F.transpose();
    S2.bottomRightCorner(d, d) += N;
    S = S2;
  };
  const Eigen::MatrixXd none;
  // ACTIVATE (§8): φ = 0 + n_φ (σ_DOV), ψ from GN (σ_ψ), t = a^N_SPP − M(y − o) with −J_x P and −J_ψ σ_ψ² couplings,
  // clock from the LS block
  const double s_dov = cfg.frames.sigma_dov_rad;
  aug.addSlot(Slot::Tilt, 0, "tilt", Eigen::Vector2d::Zero(), Eigen::MatrixXd::Zero(2, HostIdx::dim), none, s_dov * s_dov * Eigen::MatrixXd::Identity(2, 2), P);
  augment(Eigen::MatrixXd::Zero(2, HostIdx::dim), none, s_dov * s_dov * Eigen::MatrixXd::Identity(2, 2));
  const double s_psi = 0.02;
  aug.addSlot(Slot::Yaw, 0, "yaw", Eigen::VectorXd::Constant(1, 1.1), Eigen::MatrixXd::Zero(1, HostIdx::dim), none, s_psi * s_psi * Eigen::MatrixXd::Identity(1, 1), P);
  augment(Eigen::MatrixXd::Zero(1, HostIdx::dim), none, s_psi * s_psi * Eigen::MatrixXd::Identity(1, 1));
  const Eigen::MatrixXd Jx = gaussian(3, HostIdx::dim);
  Eigen::MatrixXd Fa_t = Eigen::MatrixXd::Zero(3, 3);
  Fa_t.col(2) = Eigen::Vector3d(0.3, -0.7, 0.0);   // −J_ψ
  const Eigen::MatrixXd cov_spp = randomSpd(3, 9.0);
  aug.addSlot(Slot::Trans, 0, "trans", Eigen::Vector3d(3.0, 4.0, 5.0), -Jx, Fa_t, cov_spp, P);
  augment(-Jx, Fa_t, cov_spp);
  const Eigen::MatrixXd cov_clk = randomSpd(2, 4.0);
  aug.addSlot(Slot::Clk, 0, "clk", Eigen::Vector2d(100.0, 0.1), Eigen::MatrixXd::Zero(2, HostIdx::dim), Eigen::MatrixXd::Zero(2, 6), cov_clk, P);
  augment(Eigen::MatrixXd::Zero(2, HostIdx::dim), Eigen::MatrixXd::Zero(2, 6), cov_clk);
  ASSERT_EQ(aug.dim(), 8);
  EXPECT_LT((aug.jointCov(P) - S).cwiseAbs().maxCoeff(), 1e-13 * S.cwiseAbs().maxCoeff());
  EXPECT_GE(minEig(aug.jointCov(P)), -1e-12 * S.cwiseAbs().maxCoeff());
  // the couplings are where they should be: C's trans columns = −P Jxᵀ, Q's trans/yaw block = −J_ψ σ_ψ²
  const int itr = aug.find(Slot::Trans), iy = aug.find(Slot::Yaw);
  EXPECT_LT((aug.C().middleCols<3>(itr) - (-P * Jx.transpose())).cwiseAbs().maxCoeff(), 1e-13);
  EXPECT_LT((aug.Q().block<3, 1>(itr, iy) - Fa_t.col(2) * s_psi * s_psi).cwiseAbs().maxCoeff(), 1e-16);
  EXPECT_EQ(aug.slots()[2].name, "trans");
  EXPECT_EQ(aug.slots()[2].offset, 3);
  EXPECT_EQ(aug.slots()[3].kind, Slot::Clk);
  EXPECT_EQ(aug.find(Slot::Clone, 5), -1);
  double v, s;
  EXPECT_FALSE(aug.isb(Sys::BDS, &v, &s));
  EXPECT_EQ(v, 0.0);
  // ISB slot for BDS, tagged with the system bit
  aug.addSlot(Slot::Isb, static_cast<int>(sysBit(Sys::BDS)), "isb_C", Eigen::VectorXd::Constant(1, 2.5), Eigen::MatrixXd::Zero(1, HostIdx::dim), none,
              Eigen::MatrixXd::Constant(1, 1, 900.0), P);
  augment(Eigen::MatrixXd::Zero(1, HostIdx::dim), none, Eigen::MatrixXd::Constant(1, 1, 900.0));
  EXPECT_TRUE(aug.isb(Sys::BDS, &v, &s));
  EXPECT_EQ(v, 2.5);
  EXPECT_EQ(s, 30.0);
  EXPECT_FALSE(aug.isb(Sys::GAL, &v, &s));
  EXPECT_LT((aug.jointCov(P) - S).cwiseAbs().maxCoeff(), 1e-13 * S.cwiseAbs().maxCoeff());
  // PARK the registration (REINIT drops frame + clock slots): marginalisation, remaining entries untouched
  const Eigen::MatrixXd S_full = aug.jointCov(P);
  int trans_idx = -1;
  for (size_t i = 0; i < aug.slots().size(); ++i)
    if (aug.slots()[i].kind == Slot::Trans) trans_idx = static_cast<int>(i);
  aug.removeSlot(trans_idx);
  EXPECT_FALSE(aug.has(Slot::Trans));
  EXPECT_EQ(aug.dim(), 6);
  EXPECT_TRUE(aug.trans().isZero(0.0));   // absent slot reads as zero
  std::vector<int> keep{0, 1, 2, 6, 7, 8};   // tilt(2) yaw(1) [trans removed] clk(2) isb(1) in joint indices +19
  for (size_t a = 0; a < keep.size(); ++a)
    for (size_t b = 0; b < keep.size(); ++b) EXPECT_EQ(aug.Q()(a, b), S_full(HostIdx::dim + keep[a], HostIdx::dim + keep[b]));
  for (size_t a = 0; a < keep.size(); ++a)
    for (int r = 0; r < HostIdx::dim; ++r) EXPECT_EQ(aug.C()(r, a), S_full(r, HostIdx::dim + keep[a]));
  // RE-ACTIVATE with a new consistent augmentation (REANCHOR, L6): consistent again
  {
    Eigen::MatrixXd S_m(HostIdx::dim + 6, HostIdx::dim + 6);   // marginal of the explicit joint
    std::vector<int> jk{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 25, 26, 27};
    for (size_t a = 0; a < jk.size(); ++a)
      for (size_t b = 0; b < jk.size(); ++b) S_m(a, b) = S(jk[a], jk[b]);
    S = S_m;
  }
  const Eigen::MatrixXd Jx2 = gaussian(3, HostIdx::dim), Fa2 = gaussian(3, 6), N2 = randomSpd(3, 2.0);
  aug.addSlot(Slot::Trans, 0, "trans", Eigen::Vector3d(1.0, 1.0, 1.0), Jx2, Fa2, N2, P);
  augment(Jx2, Fa2, N2);
  EXPECT_EQ(aug.find(Slot::Trans), 6);   // appended at the end, not at its old place
  EXPECT_LT((aug.jointCov(P) - S).cwiseAbs().maxCoeff(), 1e-13 * S.cwiseAbs().maxCoeff());
  EXPECT_GE(minEig(aug.jointCov(P)), -1e-12 * S.cwiseAbs().maxCoeff());
  // setFromJoint round trip
  HostMat P2;
  const Eigen::MatrixXd S_rt = aug.jointCov(P);
  aug.setFromJoint(S_rt, &P2);
  EXPECT_TRUE(bitEqual(P2, P));
  EXPECT_TRUE(bitEqual(aug.jointCov(P2), S_rt));
  EXPECT_THROW(aug.setFromJoint(Eigen::MatrixXd::Identity(5, 5), &P2), std::invalid_argument);
  // wrapYaw keeps (−π, π]
  aug.x()(aug.find(Slot::Yaw)) = 4.0;
  aug.wrapYaw();
  EXPECT_NEAR(aug.yaw(), 4.0 - 2 * M_PI, 1e-15);
  aug.x()(aug.find(Slot::Yaw)) = -M_PI;
  aug.wrapYaw();
  EXPECT_EQ(aug.yaw(), M_PI);
  aug.x()(aug.find(Slot::Yaw)) = M_PI;
  aug.wrapYaw();
  EXPECT_EQ(aug.yaw(), M_PI);
  aug.clear();
  EXPECT_EQ(aug.dim(), 0);
  EXPECT_TRUE(aug.slots().empty());
}

// 1 ms clock jumps (k = ±1, ±2) are detected by ClockJumpDetector; non-integer > 1 km asks for a reset (moved from U12, A9-2)
TEST(U09AugBlock, ClockJumpDetectorPlusMinus1And2ms) {
  Config cfg;
  const ClockJumpDetector det(cfg.rejection);
  const double c_ms = ClockJumpDetector::kMsInMetres;
  EXPECT_NEAR(c_ms, 299792.458, 1e-9);
  auto innovations = [&](double offset, int n_sats, bool with_outlier) {
    std::vector<double> v(n_sats);
    for (int i = 0; i < n_sats; ++i) v[i] = offset + 0.8 * gauss();
    if (with_outlier) v[n_sats / 3] += 50000.0;   // one NLOS satellite must not move the verdict
    return v;
  };
  for (int k : {-2, -1, 1, 2}) {
    for (int n_sats : {4, 5, 8, 13}) {
      for (bool outlier : {false, true}) {
        const ClockJumpVerdict v = det.detect(innovations(k * c_ms, n_sats, outlier));
        EXPECT_TRUE(v.jump) << "k " << k << " n " << n_sats;
        EXPECT_EQ(v.k_ms, k) << "k " << k << " n " << n_sats;
        EXPECT_FALSE(v.needs_reset);
        EXPECT_NEAR(v.median_innov_m, k * c_ms, 5.0);
      }
    }
  }
  // no jump
  for (int n_sats : {1, 2, 8}) {
    const ClockJumpVerdict v = det.detect(innovations(0.0, n_sats, false));
    EXPECT_FALSE(v.jump);
    EXPECT_FALSE(v.needs_reset);
    EXPECT_EQ(v.k_ms, 0);
  }
  // inside the 1 km window around k·c·1 ms it is still the integer jump
  {
    const ClockJumpVerdict v = det.detect(innovations(c_ms + 900.0, 8, false));
    EXPECT_TRUE(v.jump);
    EXPECT_EQ(v.k_ms, 1);
    EXPECT_FALSE(v.needs_reset);
  }
  // |m| < 1 km, k = 0: neither
  {
    const ClockJumpVerdict v = det.detect(innovations(600.0, 8, false));
    EXPECT_FALSE(v.jump);
    EXPECT_FALSE(v.needs_reset);
  }
  // non-integer millisecond, |m| > 1 km: reset
  for (double off : {0.5 * c_ms, -1.5 * c_ms, 3000.0, -2500.0, 10.0 * c_ms + 5000.0}) {
    const ClockJumpVerdict v = det.detect(innovations(off, 8, true));
    EXPECT_FALSE(v.jump) << off;
    EXPECT_TRUE(v.needs_reset) << off;
    EXPECT_EQ(v.k_ms, 0);
  }
  // empty / non-finite input: nothing
  EXPECT_FALSE(det.detect({}).jump);
  EXPECT_FALSE(det.detect({}).needs_reset);
  {
    const ClockJumpVerdict v = det.detect({std::nan(""), c_ms, c_ms + 1.0, std::nan("")});
    EXPECT_TRUE(v.jump);
    EXPECT_EQ(v.k_ms, 1);
  }
  // a configured threshold is respected
  Config::Rejection rej = cfg.rejection;
  rej.clock_jump_km = 0.5;
  const ClockJumpDetector det2(rej);
  EXPECT_TRUE(det2.detect(innovations(600.0, 8, false)).needs_reset);
  EXPECT_FALSE(det2.detect(innovations(c_ms + 900.0, 8, false)).jump);
  EXPECT_TRUE(det2.detect(innovations(c_ms + 900.0, 8, false)).needs_reset);

  // L2 actions on the block
  AugBlock aug(cfg);
  HostMat P;
  makeBlock(cfg, &aug, &P);
  const Eigen::VectorXd x0 = aug.x();
  const Eigen::MatrixXd C0 = aug.C(), Q0 = aug.Q();
  const int ic = aug.find(Slot::Clk);
  applyIntegerMsJump(&aug, -2);
  EXPECT_EQ(aug.clockBias(), x0(ic) - 2.0 * c_ms);
  EXPECT_EQ(aug.clockDrift(), x0(ic + 1));
  EXPECT_TRUE(bitEqual(aug.C(), C0));
  EXPECT_TRUE(bitEqual(aug.Q(), Q0));
  applyIntegerMsJump(&aug, 0);
  EXPECT_EQ(aug.clockBias(), x0(ic) - 2.0 * c_ms);
  Eigen::Matrix2d cov;
  cov << 25.0, 0.5, 0.5, 0.04;
  resetClockStates(&aug, 12.5, -0.3, cov);
  EXPECT_EQ(aug.clockBias(), 12.5);
  EXPECT_EQ(aug.clockDrift(), -0.3);
  EXPECT_TRUE(aug.C().middleCols<2>(ic).isZero(0.0));
  EXPECT_TRUE((aug.Q().block<2, 2>(ic, ic) - cov).isZero(0.0));
  const int n = aug.dim();
  for (int i = 0; i < n; ++i) {
    const bool clk_i = (i == ic || i == ic + 1);
    if (!clk_i) { EXPECT_EQ(aug.x()(i), x0(i)); }
    for (int r = 0; r < HostIdx::dim; ++r)
      if (!clk_i) { EXPECT_EQ(aug.C()(r, i), C0(r, i)); }
    for (int j = 0; j < n; ++j) {
      const bool clk_j = (j == ic || j == ic + 1);
      if (clk_i != clk_j) { EXPECT_EQ(aug.Q()(i, j), 0.0); }
      if (!clk_i && !clk_j) { EXPECT_EQ(aug.Q()(i, j), Q0(i, j)); }
    }
  }
  // without a clock slot both are no-ops
  AugBlock none_aug(cfg);
  applyIntegerMsJump(&none_aug, 1);
  resetClockStates(&none_aug, 1.0, 1.0, cov);
  EXPECT_EQ(none_aug.dim(), 0);
}

// A partial registration block (design: never, §8 activates tilt/yaw/trans together) must not receive the rotation
// noise of an absent slot through the [u]× coupling into t
TEST(U09AugBlock, PartialFrameSlotsCarryNoRotationNoise) {
  Config cfg;
  cfg.process.frame.q_th_per_m = 0.0;
  cfg.process.frame.q_tv_per_m = 0.0;   // only the rotation sources are left
  const HostMat P = randomSpd(HostIdx::dim).cast<double>();
  const Eigen::MatrixXd none;
  Leveling lev;
  const Eigen::Vector3d g_W(0.08, -0.15, -9.79);
  lev.setRef(g_W);
  const Eigen::Vector3d a_W(100.0, 0.0, 0.0), pivot(0.0, 0.0, 0.0);   // |u| = 100 m lever
  const Eigen::Matrix3d M = lev.M(0.0, Eigen::Vector2d::Zero(), g_W);
  const double ds = 1.0, dalpha = 0.5;
  // (1) trans only: nothing walks
  {
    AugBlock aug(cfg);
    aug.addSlot(Slot::Trans, 0, "trans", Eigen::Vector3d::Zero(), Eigen::MatrixXd::Zero(3, HostIdx::dim), none, Eigen::MatrixXd::Identity(3, 3), P);
    Eigen::MatrixXd Phi, Qp;
    aug.processModel(ds, dalpha, 0.1, M, a_W, pivot, 0.19, 0.0, &Phi, &Qp);
    EXPECT_TRUE(Qp.isZero(0.0)) << Qp;
  }
  // (2) yaw + trans: only w_ψ exists, Q_t = q_ψ ([u]×e_z)([u]×e_z)ᵀ, Q_ψt = q_ψ ([u]×e_z)ᵀ
  {
    AugBlock aug(cfg);
    aug.addSlot(Slot::Yaw, 0, "yaw", Eigen::VectorXd::Zero(1), Eigen::MatrixXd::Zero(1, HostIdx::dim), none, Eigen::MatrixXd::Identity(1, 1), P);
    aug.addSlot(Slot::Trans, 0, "trans", Eigen::Vector3d::Zero(), Eigen::MatrixXd::Zero(3, HostIdx::dim), none, Eigen::MatrixXd::Identity(3, 3), P);
    Eigen::MatrixXd Phi, Qp;
    aug.processModel(ds, dalpha, 0.1, M, a_W, pivot, 0.19, 0.0, &Phi, &Qp);
    const double q_psi = cfg.process.frame.q_yaw_per_m * ds + cfg.process.frame.q_yaw_per_rad * dalpha;
    const Eigen::Vector3d u = M * (a_W - pivot);
    const Eigen::Vector3d c = u.cross(Eigen::Vector3d::UnitZ());   // [u]× e_z
    Eigen::MatrixXd expect = Eigen::MatrixXd::Zero(4, 4);
    expect(0, 0) = q_psi;
    expect.block<3, 1>(1, 0) = q_psi * c;
    expect.block<1, 3>(0, 1) = q_psi * c.transpose();
    expect.block<3, 3>(1, 1) = q_psi * c * c.transpose();
    EXPECT_LT((Qp - expect).cwiseAbs().maxCoeff(), 1e-15 * expect.cwiseAbs().maxCoeff());
    // the current point is still invariant: [e_z]×u · w_ψ cancels against Δt
    Eigen::MatrixXd Jm = Eigen::MatrixXd::Zero(3, 4);
    Jm.col(0) = Eigen::Vector3d::UnitZ().cross(u);
    Jm.rightCols<3>().setIdentity();
    EXPECT_LT((Jm * Qp * Jm.transpose()).cwiseAbs().maxCoeff(), 1e-13 * u.squaredNorm() * q_psi);
  }
  // (3) tilt + trans (no yaw): the ψ source is dropped, the φ sources stay
  {
    AugBlock aug(cfg);
    aug.addSlot(Slot::Tilt, 0, "tilt", Eigen::Vector2d::Zero(), Eigen::MatrixXd::Zero(2, HostIdx::dim), none, Eigen::MatrixXd::Identity(2, 2), P);
    aug.addSlot(Slot::Trans, 0, "trans", Eigen::Vector3d::Zero(), Eigen::MatrixXd::Zero(3, HostIdx::dim), none, Eigen::MatrixXd::Identity(3, 3), P);
    Eigen::MatrixXd Phi, Qp;
    aug.processModel(ds, dalpha, 0.1, M, a_W, pivot, 0.19, 0.0, &Phi, &Qp);
    const double q_phi = cfg.process.frame.q_tilt_per_m * ds;
    const Eigen::Matrix3d U = skew(Eigen::Vector3d(M * (a_W - pivot)));
    Eigen::Matrix<double, 5, 2> J;
    J.setZero();
    J.topLeftCorner<2, 2>().setIdentity();
    J.bottomLeftCorner<3, 2>() = U.leftCols<2>();   // R_z(0) E_xy = E_xy
    const Eigen::MatrixXd expect = q_phi * J * J.transpose();
    EXPECT_LT((Qp - expect).cwiseAbs().maxCoeff(), 1e-15 * expect.cwiseAbs().maxCoeff());
  }
}

// The tilt residual φ has no dead zone: M and repivot use Leveling::expTilt (no 1e-5 cut-off), so a φ below the
// host's so3Exp threshold moves M by exactly its first-order amount and central differences work at any step
TEST(U09AugBlock, TiltExpHasNoDeadZone) {
  Leveling lev;
  const Eigen::Vector3d g(0.08, -0.15, -9.79);
  lev.setRef(g);
  const double psi = 0.3;
  const Eigen::Matrix3d Rz = Leveling::Rz(psi), Cg = lev.C(g);
  const Eigen::Matrix3d M0 = lev.M(psi, Eigen::Vector2d::Zero(), g);
  EXPECT_TRUE(M0 == Rz * Cg);
  // the probe case of the review (φ = (8e-6, −5e-6), |y| = 100 m): the 1 mm shift is present, not dropped
  {
    const Eigen::Vector2d phi(8e-6, -5e-6);
    const Eigen::Matrix3d M1 = lev.M(psi, phi, g);
    EXPECT_FALSE(M1 == M0);
    const Eigen::Matrix3d dM_first = Rz * skew(Eigen::Vector3d(phi.x(), phi.y(), 0.0)) * Cg;
    EXPECT_LT((M1 - M0 - dM_first).cwiseAbs().maxCoeff(), 0.6 * phi.squaredNorm());   // second order: ½[φ̃]²
    const Eigen::Vector3d y(100.0, 0.0, 0.0);
    EXPECT_NEAR(((M1 - M0) * y).norm(), (dM_first * y).norm(), 1e-8);   // ½|φ̃|²·|y| ≈ 4.5e-9 second order
    EXPECT_GT(((M1 - M0) * y).norm(), 4.5e-4);
  }
  // expTilt: exactly the identity at 0, orthonormal with det +1 across seven decades, equal to so3Exp above its cut-off
  EXPECT_TRUE(Leveling::expTilt(Eigen::Vector2d::Zero()) == Eigen::Matrix3d::Identity());
  for (double a : {1e-9, 1e-7, 1e-5, 9.99e-5, 1.01e-4, 1e-3, 1e-2, 0.5, 2.0}) {
    const Eigen::Vector2d phi(a * 0.6, -a * 0.8);
    const Eigen::Matrix3d E = Leveling::expTilt(phi);
    EXPECT_LT((E * E.transpose() - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff(), 4e-15) << a;
    EXPECT_NEAR(E.determinant(), 1.0, 4e-15) << a;
    // independent construction (Eigen AngleAxis, angle a about the unit axis (0.6, −0.8, 0))
    const Eigen::Matrix3d Eref = Eigen::AngleAxisd(a, Eigen::Vector3d(0.6, -0.8, 0.0)).toRotationMatrix();
    EXPECT_LT((E - Eref).cwiseAbs().maxCoeff(), 4e-15) << a;
    if (a > 1e-5) { EXPECT_LT((E - so3Exp(Eigen::Vector3d(phi.x(), phi.y(), 0.0))).cwiseAbs().maxCoeff(), 4e-15) << a; }
    // continuity across the Taylor/closed-form switch and the first-order term for small angles
    if (a <= 1e-2) {
      EXPECT_LT((E - Eigen::Matrix3d::Identity() - skew(Eigen::Vector3d(phi.x(), phi.y(), 0.0))).cwiseAbs().maxCoeff(), 0.52 * a * a + 0.2 * a * a * a + 1e-17) << a;
    }
  }
  // central differences of M through φ = 0 at steps 1e-6 and 1e-8 match Rz [e_k]× C(g) (a step of 1e-8 was a
  // flat zero with so3Exp)
  for (double h : {1e-6, 1e-8}) {
    for (int k = 0; k < 2; ++k) {
      Eigen::Vector2d e = Eigen::Vector2d::Zero();
      e(k) = h;
      const Eigen::Matrix3d num = (lev.M(psi, e, g) - lev.M(psi, -e, g)) / (2.0 * h);
      Eigen::Vector3d ek = Eigen::Vector3d::Zero();
      ek(k) = 1.0;
      const Eigen::Matrix3d ana = Rz * skew(ek) * Cg;
      EXPECT_LT((num - ana).cwiseAbs().maxCoeff(), 1e-6) << "h " << h << " col " << k;
    }
  }
  // repivot with a tiny tilt: t' = t + M d uses the same expTilt, so the tilt is not ignored there either
  {
    Config cfg;
    AugBlock aug(cfg);
    HostMat P;
    const Eigen::Vector2d phi(8e-6, -5e-6);
    makeBlock(cfg, &aug, &P, psi, phi, Eigen::Vector3d::Zero());
    CoreState x = randomCore();
    x.g = g;
    const Eigen::Vector3d pivot_old = x.p - Eigen::Vector3d(100.0, 0.0, 0.0);
    aug.repivot(pivot_old, x.p, lev, g, P, true);
    const Eigen::Vector3d d = x.p - pivot_old;
    EXPECT_LT((aug.trans() - lev.M(psi, phi, g) * d).norm(), 1e-12);
    EXPECT_GT((aug.trans() - Rz * Cg * d).norm(), 4.5e-4);   // differs from the φ = 0 result by the ~0.5 mm the probe found missing
  }
}

// Leveling::tiltLeftJacobian (A34) is J_l of SO(3) at φ̃ = [φ_x, φ_y, 0]: equal to the power series across the Taylor /
// closed-form switch, and to its defining property Exp(φ̃ + δφ̃) = Exp(J_l(φ̃) δφ̃) Exp(φ̃), i.e. the x/y columns of J_l
// are vee((∂Exp(φ̃ + s e_k)/∂s) Exp(φ̃)ᵀ) (central differences through the independent AngleAxis Exp)
TEST(U09AugBlock, TiltLeftJacobianIsExact) {
  EXPECT_TRUE(Leveling::tiltLeftJacobian(Eigen::Vector2d::Zero()) == Eigen::Matrix3d::Identity());
  for (double a : {1e-9, 1e-7, 1e-5, 9.99e-5, 1.01e-4, 1.5e-4, 1e-3, 1e-2, 0.5, 2.0}) {
    const Eigen::Vector2d phi(a * 0.6, -a * 0.8);
    const Eigen::Matrix3d J = Leveling::tiltLeftJacobian(phi);
    // a few ulps (entries are O(1); the series and the closed form round differently, with or without FMA contraction)
    EXPECT_LT((J - leftJacobianSeries(Eigen::Vector3d(phi.x(), phi.y(), 0.0))).cwiseAbs().maxCoeff(), 4e-15 * (1.0 + a * a)) << a;
    const Eigen::Matrix3d E0t = expTiltRef(phi).transpose();
    for (int k = 0; k < 2; ++k) {
      const Eigen::Matrix3d dE = cd4([&](double s) { Eigen::Vector2d p = phi; p(k) += s; return expTiltRef(p); }, kHAngle);
      const Eigen::Matrix3d W = dE * E0t;   // = [J_l e_k]× up to O(h⁴)
      EXPECT_LT((W + W.transpose()).cwiseAbs().maxCoeff(), 1e-11) << a << " col " << k;
      const Eigen::Vector3d num(0.5 * (W(2, 1) - W(1, 2)), 0.5 * (W(0, 2) - W(2, 0)), 0.5 * (W(1, 0) - W(0, 1)));
      EXPECT_LT((num - J.col(k)).norm(), 1e-11) << a << " col " << k;
    }
    // the correction over A11's φ = 0 form is first order, J_l − I = ½[φ̃]× + O(|φ̃|²) (entrywise ≤ θ²/6 for small θ,
    // plus rounding of the diagonal, which sits at 1 with ulp 2.2e-16): what A34 adds to the tilt column
    if (a <= 1e-2) {
      EXPECT_LT((J - Eigen::Matrix3d::Identity() - 0.5 * skew(Eigen::Vector3d(phi.x(), phi.y(), 0.0))).cwiseAbs().maxCoeff(), a * a / 6.0 + 1e-15) << a;
    }
  }
}

// Leveling sanity (the U7 ε checks are WP-c's; these pin the behaviour of setRef/C/M used above)
TEST(U09AugBlock, LevelingLevelsGravityWithoutYaw) {
  Leveling lev;
  EXPECT_FALSE(lev.hasRef());
  const Eigen::Vector3d g(0.6, -0.9, -9.75);
  lev.setRef(g);
  EXPECT_TRUE(lev.hasRef());
  const Eigen::Vector3d down(0.0, 0.0, -1.0);
  EXPECT_LT((lev.C(g) * g.normalized() - down).norm(), 1e-15);
  EXPECT_LT((lev.ref() - lev.C(g)).cwiseAbs().maxCoeff(), 1e-15);   // C_ref levels the g it was set from
  // any other g is levelled too, and the update rotation has a horizontal axis (no yaw leak)
  for (int k = 0; k < 20; ++k) {
    const Eigen::Vector3d g2 = g + 0.05 * Eigen::Vector3d(gauss(), gauss(), gauss());
    const Eigen::Matrix3d C2 = lev.C(g2);
    EXPECT_LT((C2 * g2.normalized() - down).norm(), 1e-14);
    const Eigen::Vector3d ax = so3Log(Eigen::Matrix3d(C2 * lev.ref().transpose()));
    EXPECT_LT(std::abs(ax.z()), 1e-14 * (1.0 + ax.norm()));
    EXPECT_LT((C2 * C2.transpose() - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff(), 1e-14);
    // refreshing the reference with g2 keeps C(g2) and keeps yaw (the composed axis stays horizontal)
    Leveling lev2 = lev;
    lev2.setRef(g2);
    EXPECT_LT((lev2.ref() - C2).cwiseAbs().maxCoeff(), 1e-15);
  }
  // ε Jacobian: third row exactly zero; central difference of Log(C(g+hδ)C(g)ᵀ) matches (C_ref set from g)
  const Eigen::Matrix3d Je = lev.epsJacobian(g);
  EXPECT_TRUE(Je.row(2).isZero(0.0));
  const double h = 1e-6;
  for (int k = 0; k < 3; ++k) {
    Eigen::Vector3d e = Eigen::Vector3d::Zero();
    e(k) = h;
    const Eigen::Vector3d num = (so3Log(Eigen::Matrix3d(lev.C(g + e) * lev.C(g).transpose())) - so3Log(Eigen::Matrix3d(lev.C(g - e) * lev.C(g).transpose()))) / (2 * h);
    EXPECT_LT((num - Je.col(k)).norm(), 1e-6 * (1.0 + num.norm())) << "g col " << k;
    EXPECT_LT(std::abs(num.z()), 1e-9) << "g col " << k;   // ε_z: second order only
  }
  EXPECT_LT((lev.Gperp(g) * g).norm(), 1e-13);   // G_⊥ annihilates ĝ
  // M = R_z(ψ) Exp(φ̃) C(g); Rz is a proper rotation about e_z
  const Eigen::Matrix3d Rz = Leveling::Rz(0.3);
  EXPECT_NEAR(Rz(0, 0), std::cos(0.3), 0.0);
  EXPECT_NEAR(Rz(1, 0), std::sin(0.3), 0.0);
  EXPECT_NEAR(Rz(0, 1), -std::sin(0.3), 0.0);
  EXPECT_EQ(Rz(2, 2), 1.0);
  const Eigen::Matrix3d M = lev.M(0.3, Eigen::Vector2d(1e-3, -2e-3), g);
  EXPECT_LT((M - Rz * so3Exp(Eigen::Vector3d(1e-3, -2e-3, 0.0)) * lev.C(g)).cwiseAbs().maxCoeff(), 1e-14);
  EXPECT_LT((Leveling::minimalRotation(Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY()) * Eigen::Vector3d::UnitX() - Eigen::Vector3d::UnitY()).norm(), 1e-15);
}
