// SPDX-License-Identifier: BSD-3-Clause
// test_u10_updater.cpp — TEST_PLAN §1 U10: linear-Gaussian == batch LS; Joseph == standard form and PSD;
// Householder ≡ single-difference + whitening; χ² thresholds; bit-exact rollback; conditional iteration.
// The cases that drive JointUpdater::update() run against WP-b's AugBlock (jointCov/setFromJoint/addSlot).
#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <random>
#include <stdexcept>
#include <string>

#include <Eigen/Dense>

#include "internal/update.hpp"
#include "internal/rows.hpp"

using namespace livo_gnss;

namespace {

constexpr double kDeg = M_PI / 180.0;

SatId sat(int i) {
  SatId s;
  s.sys = Sys::GPS;
  s.prn = i;
  s.rtk_sat = i;
  return s;
}

Eigen::MatrixXd randomPd(int n, double scale, std::mt19937_64& rng) {
  std::normal_distribution<double> g(0.0, 1.0);
  Eigen::MatrixXd A(n, n);
  for (int i = 0; i < n; ++i)
    for (int j = 0; j < n; ++j) A(i, j) = g(rng);
  return scale * (A * A.transpose()) / n + Eigen::MatrixXd::Identity(n, n) * scale * 1e-3;
}

// Independent slots (F_host = 0, F_aug = 0, N = prior): trans(3) + clk(2).
void addTransClk(AugBlock* aug, const HostMat& P, const Eigen::Vector3d& t0, double b0, double d0, double var_t, double var_b, double var_d) {
  Eigen::MatrixXd F0 = Eigen::MatrixXd::Zero(3, HostIdx::dim);
  aug->addSlot(Slot::Trans, 0, "trans", t0, F0, Eigen::MatrixXd(3, 0), Eigen::Matrix3d::Identity() * var_t, P);
  Eigen::MatrixXd F1 = Eigen::MatrixXd::Zero(2, HostIdx::dim);
  Eigen::Vector2d c0(b0, d0);
  Eigen::Matrix2d Nc = Eigen::Matrix2d::Zero();
  Nc(0, 0) = var_b;
  Nc(1, 1) = var_d;
  aug->addSlot(Slot::Clk, 0, "clk", c0, F1, Eigen::MatrixXd::Zero(2, aug->dim()), Nc, P);
}

// Additive coordinates of the linear toy: [p | v | x_G].
Eigen::VectorXd linCoords(const CoreState& x, const AugBlock& a) {
  Eigen::VectorXd v = Eigen::VectorXd::Zero(HostIdx::dim + a.dim());
  v.segment<3>(HostIdx::pos) = x.p;
  v.segment<3>(HostIdx::vel) = x.v;
  v.tail(a.dim()) = a.x();
  return v;
}

std::vector<double> snapshotBytes(const CoreState& c, const AugBlock& a) {
  std::vector<double> v;
  auto push = [&](const double* p, int n) { v.insert(v.end(), p, p + n); };
  push(c.R.data(), 9);
  push(c.p.data(), 3);
  push(c.v.data(), 3);
  push(c.bg.data(), 3);
  push(c.ba.data(), 3);
  push(c.g.data(), 3);
  push(&c.inv_expo, 1);
  push(c.P.data(), HostIdx::dim * HostIdx::dim);
  push(a.x().data(), static_cast<int>(a.x().size()));
  push(a.Q().data(), static_cast<int>(a.Q().size()));
  push(a.C().data(), static_cast<int>(a.C().size()));
  return v;
}
bool sameBytes(const std::vector<double>& a, const std::vector<double>& b) {
  return a.size() == b.size() && std::memcmp(a.data(), b.data(), a.size() * sizeof(double)) == 0;
}

}  // namespace

// χ² quantiles match tabulated values (exact inverse regularised gamma, not Wilson–Hilferty)
TEST(U10Updater, Chi2Thresholds) {
  EXPECT_NEAR(chi2Quantile(1, 0.95), 3.841458821, 1e-8);
  EXPECT_NEAR(chi2Quantile(1, 0.999), 10.82756617, 1e-7);
  EXPECT_NEAR(chi2Quantile(2, 0.999), 13.81551056, 1e-7);
  EXPECT_NEAR(chi2Quantile(8, 0.999), 26.12448156, 1e-7);
  EXPECT_NEAR(chi2Quantile(10, 0.95), 18.30703805, 1e-7);
  EXPECT_NEAR(chi2Quantile(30, 0.99), 50.89218131, 1e-7);
  EXPECT_NEAR(chi2Quantile(100, 0.5), 99.33412923, 1e-7);
  EXPECT_NEAR(chi2Quantile(3, 0.001), 0.0242975858157, 1e-10);
  EXPECT_NEAR(normalQuantile(0.9995), 3.290526731, 1e-8);   // two-sided α = 1e-3
  EXPECT_NEAR(normalQuantile(0.975), 1.959963985, 1e-8);
  EXPECT_NEAR(normalQuantile(0.5), 0.0, 1e-12);
  EXPECT_NEAR(normalQuantile(0.001), -3.090232306, 1e-8);
  EXPECT_TRUE(std::isnan(chi2Quantile(0, 0.5)));
}

// Joseph form equals the standard form for the optimal gain and stays PSD/symmetric for a suboptimal one
TEST(U10Updater, JosephEqualsStandardAndPsd) {
  std::mt19937_64 rng(3);
  std::normal_distribution<double> g(0.0, 1.0);
  const int N = 25, m = 6;
  Eigen::MatrixXd Sigma = randomPd(N, 0.5, rng);
  Eigen::MatrixXd H(m, N);
  for (int i = 0; i < m; ++i)
    for (int j = 0; j < N; ++j) H(i, j) = g(rng);
  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(m, m);
  for (int i = 0; i < m; ++i) R(i, i) = 0.01 + 0.1 * std::fabs(g(rng));
  Eigen::MatrixXd S1 = Sigma;
  const Eigen::MatrixXd K = JointUpdater::josephUpdate(S1, H, R, Eigen::MatrixXd());
  const Eigen::MatrixXd S = H * Sigma * H.transpose() + R;
  const Eigen::MatrixXd K_ref = Sigma * H.transpose() * S.inverse();
  EXPECT_LT((K - K_ref).norm() / K_ref.norm(), 1e-10);
  const Eigen::MatrixXd S_std = (Eigen::MatrixXd::Identity(N, N) - K_ref * H) * Sigma;
  EXPECT_LT((S1 - S_std).norm() / S_std.norm(), 1e-9);
  EXPECT_LT((S1 - S1.transpose()).norm(), 1e-14 * S1.norm());
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(S1);
  EXPECT_GE(es.eigenvalues().minCoeff(), -1e-12 * es.eigenvalues().maxCoeff());
  // suboptimal gain (Schmidt-like: first 19 rows zeroed, rest scaled): Joseph remains PSD, standard form does not have to
  Eigen::MatrixXd K2 = 0.7 * K_ref;
  K2.topRows(HostIdx::dim).setZero();
  Eigen::MatrixXd S2 = Sigma;
  JointUpdater::josephUpdate(S2, H, R, K2);
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es2(S2);
  EXPECT_GE(es2.eigenvalues().minCoeff(), -1e-12 * es2.eigenvalues().maxCoeff());
  EXPECT_LT((S2 - S2.transpose()).norm(), 1e-14 * S2.norm());
  // the exact Joseph value for a suboptimal K is what the covariance must be (consistency, not optimality)
  const Eigen::MatrixXd IKH = Eigen::MatrixXd::Identity(N, N) - K2 * H;
  const Eigen::MatrixXd S2_ref = IKH * Sigma * IKH.transpose() + K2 * R * K2.transpose();
  EXPECT_LT((S2 - S2_ref).norm() / S2_ref.norm(), 1e-12);
  // posterior NIS of the optimal update equals the prior NIS for a linear single step (r_post = R S⁻¹ r)
  Eigen::VectorXd r(m);
  for (int i = 0; i < m; ++i) r(i) = g(rng);
  const Eigen::VectorXd r_post = r - H * (K_ref * r);
  const double nis_prior = r.dot(S.ldlt().solve(r));
  const double nis_post = JointUpdater::posteriorNis(r_post, H, R, S1);
  EXPECT_NEAR(nis_post, nis_prior, 1e-8 * nis_prior);
}

// Householder projection solution equals single-difference + whitening
TEST(U10Updater, HouseholderEqualsSingleDifferenceWhitening) {
  std::mt19937_64 rng(5);
  std::normal_distribution<double> g(0.0, 1.0);
  std::uniform_real_distribution<double> u(0.5, 2.0);
  const int N = 22, m = 7;
  for (int variant = 0; variant < 2; ++variant) {
    RowSet in;
    in.cls = MeasClass::Tdcp;
    Eigen::MatrixXd H(m, N), R = Eigen::MatrixXd::Zero(m, m);
    Eigen::VectorXd nu(m);
    for (int i = 0; i < m; ++i) {
      MeasRow r;
      r.cls = MeasClass::Tdcp;
      r.sat = sat(i + 1);
      r.pair_id = 9;
      r.H = Eigen::RowVectorXd(N);
      for (int j = 0; j < N; ++j) r.H(j) = g(rng);
      r.nu = 5.0 + 0.01 * g(rng);   // common Δclk of 5 m plus noise
      r.R = 1e-5 * u(rng);
      H.row(i) = r.H;
      nu(i) = r.nu;
      R(i, i) = r.R;
      in.rows.push_back(r);
    }
    if (variant == 1) {
      // full R with a common bridge term (rank-1 correlation) as tdcpRows() produces
      Eigen::VectorXd c(m);
      for (int i = 0; i < m; ++i) c(i) = 3e-3 * g(rng);
      R += c * c.transpose();
      in.R_full = R;
      in.has_full_R = true;
    }
    RowSet out;
    double dclk = 0.0;
    ASSERT_TRUE(RowBuilder::projectOutCommonClock(in, &out, &dclk));
    ASSERT_EQ(static_cast<int>(out.rows.size()), m - 1);
    EXPECT_NEAR(dclk, 5.0, 0.05);
    Eigen::MatrixXd Hp(m - 1, N);
    Eigen::VectorXd np(m - 1);
    for (int i = 0; i < m - 1; ++i) {
      Hp.row(i) = out.rows[i].H;
      np(i) = out.rows[i].nu;
      EXPECT_DOUBLE_EQ(out.rows[i].R, 1.0);
    }
    // single differences against satellite 0, whitened by (D R Dᵀ)⁻¹
    Eigen::MatrixXd D = Eigen::MatrixXd::Zero(m - 1, m);
    for (int i = 0; i < m - 1; ++i) {
      D(i, 0) = -1.0;
      D(i, i + 1) = 1.0;
    }
    const Eigen::MatrixXd Rsd = D * R * D.transpose();
    const Eigen::MatrixXd Wsd = Rsd.inverse();
    const Eigen::MatrixXd Hsd = D * H;
    const Eigen::VectorXd nsd = D * nu;
    const Eigen::MatrixXd info_p = Hp.transpose() * Hp, info_sd = Hsd.transpose() * Wsd * Hsd;
    const Eigen::VectorXd vec_p = Hp.transpose() * np, vec_sd = Hsd.transpose() * Wsd * nsd;
    EXPECT_LT((info_p - info_sd).norm() / info_sd.norm(), 1e-10) << "variant " << variant;
    EXPECT_LT((vec_p - vec_sd).norm() / vec_sd.norm(), 1e-10) << "variant " << variant;
    // the projected rows are exactly Δclk-free: adding a common constant changes nothing
    RowSet in2 = in;
    for (auto& r : in2.rows) r.nu += 123.456;
    RowSet out2;
    ASSERT_TRUE(RowBuilder::projectOutCommonClock(in2, &out2, nullptr));
    for (int i = 0; i < m - 1; ++i) EXPECT_NEAR(out2.rows[i].nu, out.rows[i].nu, 1e-9);
    // unit variance: (Qᵀ W) R (Qᵀ W)ᵀ = I on the kept rows ⇔ the whitening is consistent with R
    // (checked through the information equality above) and rejected rows are excluded from the projection
    RowSet in3 = in;
    in3.rows[2].reason = RejectReason::WTest;
    RowSet out3;
    ASSERT_TRUE(RowBuilder::projectOutCommonClock(in3, &out3, nullptr));
    EXPECT_EQ(static_cast<int>(out3.rows.size()), m - 2);
  }
  // fewer than two rows: no projection
  RowSet one;
  one.cls = MeasClass::Tdcp;
  one.rows.push_back(MeasRow());
  one.rows[0].H = Eigen::RowVectorXd::Ones(3);
  one.rows[0].R = 1.0;
  RowSet o;
  EXPECT_FALSE(RowBuilder::projectOutCommonClock(one, &o, nullptr));
}

// linear-Gaussian case equals the closed-form batch LS (information form)
TEST(U10Updater, LinearGaussianEqualsBatchLs) {
  Config cfg;
  std::mt19937_64 rng(17);
  std::normal_distribution<double> g(0.0, 1.0);
  CoreState core;
  core.p = Eigen::Vector3d(10.0, -4.0, 2.0);
  core.v = Eigen::Vector3d(1.0, 0.2, -0.1);
  core.P = randomPd(HostIdx::dim, 1e-2, rng);
  AugBlock aug(cfg);
  addTransClk(&aug, core.P, Eigen::Vector3d(1.0, 2.0, 3.0), 100.0, 0.5, 0.5, 4.0, 0.01);
  const int N = HostIdx::dim + aug.dim();
  ASSERT_EQ(N, 24);
  const Eigen::MatrixXd Sigma = aug.jointCov(core.P);
  const Eigen::VectorXd x_prior = linCoords(core, aug);
  // truth = a draw from the prior on the additive coordinates (rotation/bias columns untouched)
  Eigen::VectorXd x_true = x_prior;
  {
    Eigen::LLT<Eigen::MatrixXd> llt(Sigma);
    Eigen::VectorXd e(N);
    for (int i = 0; i < N; ++i) e(i) = g(rng);
    const Eigen::VectorXd d = llt.matrixL() * e;
    x_true.segment<3>(HostIdx::pos) += d.segment<3>(HostIdx::pos);
    x_true.segment<3>(HostIdx::vel) += d.segment<3>(HostIdx::vel);
    x_true.tail(aug.dim()) += d.tail(aug.dim());
  }
  const int m = 7;
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(m, N);
  Eigen::VectorXd z(m);
  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(m, m);
  for (int i = 0; i < m; ++i) {
    for (int j : {3, 4, 5, 7, 8, 9, 19, 20, 21, 22, 23}) H(i, j) = g(rng);
    R(i, i) = 0.01;
    z(i) = H.row(i).dot(x_true) + 0.1 * g(rng);
  }
  JointUpdater::RowFactory factory = [&](const CoreState& x, const AugBlock& a, std::vector<RowSet>* sets) {
    RowSet s;
    s.cls = MeasClass::Psr;
    const Eigen::VectorXd xv = linCoords(x, a);
    for (int i = 0; i < m; ++i) {
      MeasRow r;
      r.cls = MeasClass::Psr;
      r.sat = sat(i + 1);
      r.H = H.row(i);
      r.z = z(i);
      r.h = H.row(i).dot(xv);
      r.nu = r.z - r.h;
      r.R = R(i, i);
      s.rows.push_back(r);
    }
    sets->push_back(s);
  };
  JointUpdater upd(cfg);
  JointUpdater::Options opt;
  opt.disable_l4_l5 = true;
  const JointUpdater::Result res = upd.update(core, aug, factory, opt);
  ASSERT_TRUE(res.committed);
  EXPECT_EQ(res.iterations, 1);
  // batch LS over the full 24-D error state with the prior as pseudo-observation (information form):
  // δx = (Σ⁻¹ + HᵀR⁻¹H)⁻¹ HᵀR⁻¹ ν. The random prior correlates every block, so the rotation and bias blocks
  // move too; the filter applies δθ through ⊞ (R = I·Exp(δθ) here).
  const Eigen::MatrixXd Sinv = Sigma.inverse();
  const Eigen::MatrixXd Sigma_post = (Sinv + H.transpose() * R.inverse() * H).inverse();
  const Eigen::VectorXd nu = z - H * x_prior;
  const Eigen::VectorXd dx_batch = Sigma_post * (H.transpose() * R.inverse() * nu);
  const CoreState prior_state;   // identity rotation, zero biases/gravity (as constructed above)
  const HostVec dx_host_filt = core.boxminus(prior_state) - HostVec::Unit(HostIdx::pos) * 0.0;
  HostVec dx_host_ref = dx_batch.head<HostIdx::dim>();
  // the prior means of p and v were non-zero: boxminus already removed them (prior_state has zero p/v), so
  // compare the increments block by block
  EXPECT_LT((core.p - Eigen::Vector3d(10.0, -4.0, 2.0) - dx_batch.segment<3>(HostIdx::pos)).norm(), 1e-9);
  EXPECT_LT((core.v - Eigen::Vector3d(1.0, 0.2, -0.1) - dx_batch.segment<3>(HostIdx::vel)).norm(), 1e-9);
  EXPECT_LT((so3Log(core.R) - dx_batch.segment<3>(HostIdx::rot)).norm(), 1e-9);
  EXPECT_LT((core.bg - dx_batch.segment<3>(HostIdx::bg)).norm(), 1e-9);
  EXPECT_LT((core.ba - dx_batch.segment<3>(HostIdx::ba)).norm(), 1e-9);
  EXPECT_LT((core.g - dx_batch.segment<3>(HostIdx::grav)).norm(), 1e-9);
  EXPECT_NEAR(core.inv_expo - 1.0, dx_batch(HostIdx::expo), 1e-9);
  EXPECT_LT((aug.x() - x_prior.tail(aug.dim()) - dx_batch.tail(aug.dim())).norm(), 1e-9);
  (void)dx_host_filt;
  (void)dx_host_ref;
  const Eigen::MatrixXd Sigma_filt = aug.jointCov(core.P);
  EXPECT_LT((Sigma_filt - Sigma_post).norm() / Sigma_post.norm(), 1e-9);
  EXPECT_LT((Sigma_filt - Sigma_filt.transpose()).norm(), 1e-14 * Sigma_filt.norm());
}

// after an L5 rollback (and in observe-only mode) the state is bit-identical
TEST(U10Updater, RollbackIsBitExact) {
  Config cfg;
  std::mt19937_64 rng(23);
  CoreState core;
  core.R = so3Exp(Eigen::Vector3d(0.1, -0.2, 0.3));
  core.p = Eigen::Vector3d(3.0, 2.0, 1.0);
  core.v = Eigen::Vector3d(0.5, 0.1, 0.0);
  core.bg = Eigen::Vector3d(1e-3, -2e-3, 3e-3);
  core.ba = Eigen::Vector3d(0.01, 0.02, -0.03);
  core.g = Eigen::Vector3d(0.1, 0.0, -9.8);
  core.inv_expo = 1.7;
  core.P = randomPd(HostIdx::dim, 1e-2, rng);
  core.P.block<3, 3>(HostIdx::pos, HostIdx::pos) = Eigen::Matrix3d::Identity();
  AugBlock aug(cfg);
  addTransClk(&aug, core.P, Eigen::Vector3d(0.3, 0.2, 0.1), 50.0, 0.1, 1.0, 9.0, 0.01);
  const int N = HostIdx::dim + aug.dim();
  double innov = 1.0;
  JointUpdater::RowFactory factory = [&](const CoreState& x, const AugBlock& a, std::vector<RowSet>* sets) {
    (void)a;
    RowSet s;
    s.cls = MeasClass::PvtPos;
    MeasRow r;
    r.cls = MeasClass::PvtPos;
    r.H = Eigen::RowVectorXd::Zero(N);
    r.H(HostIdx::pos) = 1.0;
    r.h = x.p(0);
    r.z = x.p(0) + innov;
    r.nu = innov;
    r.R = 1e-6;
    s.rows.push_back(r);
    sets->push_back(s);
  };
  JointUpdater upd(cfg);
  const std::vector<double> before = snapshotBytes(core, aug);
  // (a) host step 1 m with a claimed prior σ of 1 cm ⇒ L5 "host_step" rollback
  {
    JointUpdater::Options opt;
    opt.sigma_prior_pos_m = 0.01;
    const JointUpdater::Result res = upd.update(core, aug, factory, opt);
    EXPECT_FALSE(res.committed);
    EXPECT_TRUE(res.rolled_back);
    EXPECT_EQ(res.rollback_reason, "host_step");
    EXPECT_NEAR(res.dp_host_norm, 1.0, 1e-3);
    EXPECT_TRUE(sameBytes(before, snapshotBytes(core, aug)));
  }
  // (b) NaN innovation ⇒ rollback "nan"
  {
    innov = std::nan("");
    JointUpdater::Options opt;
    opt.disable_l4_l5 = true;
    const JointUpdater::Result res = upd.update(core, aug, factory, opt);
    EXPECT_FALSE(res.committed);
    EXPECT_TRUE(res.rolled_back);
    EXPECT_EQ(res.rollback_reason, "nan");
    EXPECT_TRUE(sameBytes(before, snapshotBytes(core, aug)));
    innov = 1.0;
  }
  // (c) observe-only computes everything and commits nothing
  {
    JointUpdater::Options opt;
    opt.observe_only = true;
    const JointUpdater::Result res = upd.update(core, aug, factory, opt);
    EXPECT_FALSE(res.committed);
    EXPECT_FALSE(res.rolled_back);
    EXPECT_NEAR(res.dp_host_norm, 1.0, 1e-3);
    EXPECT_TRUE(sameBytes(before, snapshotBytes(core, aug)));
  }
  // (d) the same update commits when the prior σ is honest (1 m): the state changes
  {
    JointUpdater::Options opt;
    opt.sigma_prior_pos_m = 1.0;
    const JointUpdater::Result res = upd.update(core, aug, factory, opt);
    EXPECT_TRUE(res.committed);
    EXPECT_FALSE(sameBytes(before, snapshotBytes(core, aug)));
    EXPECT_NEAR(core.p(0), 3.0 + 1.0, 1e-5);
  }
}

// L5 posterior NIS: two classes that each pass their own gate (w = 3.2 < 3.29, NIS 10.24 < χ²₁(0.999) = 10.83)
// but whose joint posterior NIS 20.48 exceeds χ²₂(0.999) = 13.82 ⇒ "post_nis" rollback, state bit-identical;
// at w = 2.5 (joint 12.5) the same update commits.
TEST(U10Updater, PosteriorNisRollbackIsBitExact) {
  Config cfg;
  CoreState core;
  core.P = HostMat::Identity() * 1e-2;
  AugBlock aug(cfg);
  addTransClk(&aug, core.P, Eigen::Vector3d(0.3, 0.2, 0.1), 50.0, 0.1, 1.0, 9.0, 0.01);
  const int N = HostIdx::dim + aug.dim();
  const double R = 1e-2, S = 1e-2 + R;   // Σ_p0p0 = Σ_v0v0 = 1e-2, C = 0
  double k = 3.2;
  JointUpdater::RowFactory factory = [&](const CoreState& x, const AugBlock& a, std::vector<RowSet>* sets) {
    (void)a;
    RowSet sp, sv;
    sp.cls = MeasClass::PvtPos;
    sv.cls = MeasClass::PvtVel;
    MeasRow rp, rv;
    rp.cls = MeasClass::PvtPos;
    rp.H = Eigen::RowVectorXd::Zero(N);
    rp.H(HostIdx::pos) = 1.0;
    rp.h = x.p(0);
    rp.nu = k * std::sqrt(S);
    rp.z = rp.h + rp.nu;
    rp.R = R;
    rv.cls = MeasClass::PvtVel;
    rv.H = Eigen::RowVectorXd::Zero(N);
    rv.H(HostIdx::vel) = 1.0;
    rv.h = x.v(0);
    rv.nu = k * std::sqrt(S);
    rv.z = rv.h + rv.nu;
    rv.R = R;
    sp.rows.push_back(rp);
    sv.rows.push_back(rv);
    sets->push_back(sp);
    sets->push_back(sv);
  };
  JointUpdater upd(cfg);
  const std::vector<double> before = snapshotBytes(core, aug);
  JointUpdater::Options opt;
  JointUpdater::Result res = upd.update(core, aug, factory, opt);
  EXPECT_FALSE(res.committed);
  EXPECT_TRUE(res.rolled_back);
  EXPECT_EQ(res.rollback_reason, "post_nis");
  EXPECT_EQ(res.gates[static_cast<size_t>(MeasClass::PvtPos)].n_out, 1);
  EXPECT_EQ(res.gates[static_cast<size_t>(MeasClass::PvtVel)].n_out, 1);
  EXPECT_NEAR(res.nis_post, 2.0 * k * k, 1e-6);   // linear single step: posterior NIS = prior joint NIS
  EXPECT_GT(res.nis_post, chi2Quantile(2, 0.999));
  EXPECT_TRUE(sameBytes(before, snapshotBytes(core, aug)));
  k = 2.5;
  res = upd.update(core, aug, factory, opt);
  EXPECT_TRUE(res.committed);
  EXPECT_NEAR(res.nis_post, 2.0 * k * k, 1e-6);
  EXPECT_FALSE(sameBytes(before, snapshotBytes(core, aug)));
}

// L5 "not_psd": a malformed row (negative variance from a broken producer) passes the gate (S = 0.5 > 0, ν = 0)
// but the Joseph update yields Σ⁺_p0p0 = (1 − 2)²·1 + 2·(−0.5)·2 = −1 ⇒ rollback, state bit-identical.
TEST(U10Updater, NotPsdRollbackIsBitExact) {
  Config cfg;
  CoreState core;
  core.P = HostMat::Identity() * 1e-2;
  core.P.block<3, 3>(HostIdx::pos, HostIdx::pos) = Eigen::Matrix3d::Identity();
  AugBlock aug(cfg);
  addTransClk(&aug, core.P, Eigen::Vector3d(0.3, 0.2, 0.1), 50.0, 0.1, 1.0, 9.0, 0.01);
  const int N = HostIdx::dim + aug.dim();
  JointUpdater::RowFactory factory = [&](const CoreState& x, const AugBlock& a, std::vector<RowSet>* sets) {
    (void)a;
    RowSet s;
    s.cls = MeasClass::PvtPos;
    MeasRow r;
    r.cls = MeasClass::PvtPos;
    r.H = Eigen::RowVectorXd::Zero(N);
    r.H(HostIdx::pos) = 1.0;
    r.h = x.p(0);
    r.z = r.h;
    r.nu = 0.0;
    r.R = -0.5;
    s.rows.push_back(r);
    sets->push_back(s);
  };
  JointUpdater upd(cfg);
  const std::vector<double> before = snapshotBytes(core, aug);
  JointUpdater::Options opt;
  const JointUpdater::Result res = upd.update(core, aug, factory, opt);
  EXPECT_FALSE(res.committed);
  EXPECT_TRUE(res.rolled_back);
  EXPECT_EQ(res.rollback_reason, "not_psd");
  EXPECT_EQ(res.gates[static_cast<size_t>(MeasClass::PvtPos)].n_out, 1);   // the gate itself let the row through
  EXPECT_TRUE(sameBytes(before, snapshotBytes(core, aug)));
}

// L5 "not_psd" up to rounding (AMENDMENTS A64; A62 ⑤). A stopped clone made without noise (the Origin slot, A61 F13) is an
// exact linear function of other states, so the joint covariance has exact zero directions, and a Joseph Σ⁺ keeps them at
// the rounding level. The pre-A64 test (Eigen LDLT: info() == Success and isPositive()) rejected such a matrix either way:
// an exact zero pivot followed by a non-zero one fails info() (Eigen picks LDLT pivots on the unreduced diagonal), a
// rounding-level negative pivot fails isPositive(). posteriorPsd() takes the eigenvalues of the Jacobi-scaled Σ against
// −1e-9. Mixed units as in the joint covariance (rad², m², clock m²), the clone an exact copy of three states.
TEST(U10Updater, PosteriorPsdToleratesExactZeroDirectionsOnly) {
  auto preA64 = [](const Eigen::MatrixXd& M) {
    Eigen::LDLT<Eigen::MatrixXd> l(M);
    return l.info() == Eigen::Success && l.isPositive() && M.diagonal().minCoeff() >= 0.0;
  };
  std::mt19937_64 rng(31);
  Eigen::VectorXd s(6);
  s << 1e-3, 2e-2, 1.0, 15.0, 15.0, 300.0;
  const Eigen::MatrixXd B = s.asDiagonal() * randomPd(6, 1.0, rng) * s.asDiagonal();
  Eigen::MatrixXd F = Eigen::MatrixXd::Zero(9, 6);
  F.topRows(6).setIdentity();
  F.block(6, 2, 3, 3).setIdentity();   // clone = states 2..4, no noise
  Eigen::MatrixXd S = F * B * F.transpose();
  S = 0.5 * (S + S.transpose()).eval();
  EXPECT_TRUE(JointUpdater::posteriorPsd(S));
  EXPECT_FALSE(preA64(S));   // exact zero pivots
  auto shrinkClone = [&](double rel) {
    Eigen::MatrixXd T = S;
    for (int k = 0; k < 3; ++k) T(6 + k, 6 + k) *= 1.0 - rel;   // each zero direction becomes −rel·Σ_cc
    return T;
  };
  // rounding-level negatives in the zero directions: accepted
  const Eigen::MatrixXd Sr = shrinkClone(1e-13);
  EXPECT_TRUE(JointUpdater::posteriorPsd(Sr));
  EXPECT_FALSE(preA64(Sr));
  // a real negative direction (1e-6 of the variance) is still not PSD, and neither is 1e-13 with a tolerance of 0
  EXPECT_FALSE(JointUpdater::posteriorPsd(shrinkClone(1e-6)));
  EXPECT_FALSE(JointUpdater::posteriorPsd(Sr, 0.0));
  // negative or non-finite diagonal, non-square
  Eigen::MatrixXd Sd = S;
  Sd(0, 0) = -1e-30;
  EXPECT_FALSE(JointUpdater::posteriorPsd(Sd));
  Sd(0, 0) = std::nan("");
  EXPECT_FALSE(JointUpdater::posteriorPsd(Sd));
  EXPECT_FALSE(JointUpdater::posteriorPsd(Eigen::MatrixXd::Zero(3, 4)));
  // a zero variance with a zero row is PSD; with a non-zero cross term it is not
  Eigen::MatrixXd Sz = Eigen::MatrixXd::Zero(10, 10);
  Sz.topLeftCorner(9, 9) = S;
  EXPECT_TRUE(JointUpdater::posteriorPsd(Sz));
  Sz(9, 0) = Sz(0, 9) = 1e-6;
  EXPECT_FALSE(JointUpdater::posteriorPsd(Sz));
}

// The activation-epoch case of A62 ⑤ through update(): an Origin-like stopped copy of [t; b] (F_aug on the trans and clock
// slots, N = 0) and a strong first update of t (σ 15 m → about 1 m) must commit; the copy moves with t, so the posterior
// keeps the exact zero directions (to rounding) and both variances equal.
TEST(U10Updater, StoppedCloneUpdateCommits) {
  Config cfg;
  std::mt19937_64 rng(37);
  CoreState core;
  core.P = randomPd(HostIdx::dim, 1e-4, rng);
  AugBlock aug(cfg);
  addTransClk(&aug, core.P, Eigen::Vector3d(4.0, -7.0, 1.5), 30.0, 0.2, 225.0, 400.0, 0.04);
  const int it = aug.find(Slot::Trans), ic = aug.find(Slot::Clk);
  ASSERT_GE(it, 0);
  ASSERT_GE(ic, 0);
  Eigen::MatrixXd Fa = Eigen::MatrixXd::Zero(4, aug.dim());
  Fa.block<3, 3>(0, it).setIdentity();
  Fa(3, ic) = 1.0;
  Eigen::VectorXd x0(4);
  x0 << aug.x().segment<3>(it), aug.x()(ic);
  aug.addSlot(Slot::Origin, 0, "origin", x0, Eigen::MatrixXd::Zero(4, HostIdx::dim), Fa, Eigen::Matrix4d::Zero(), core.P);
  const int io = aug.find(Slot::Origin);
  ASSERT_GE(io, 0);
  const int N = HostIdx::dim + aug.dim();
  // a position fix of p + t (the antenna at the pivot), 0.8–2.1 m from the prior: z fixed, rows rebuilt at each iterate
  const Eigen::Vector3d z = core.p + aug.x().segment<3>(it) + Eigen::Vector3d(0.8, -1.3, 2.1);
  JointUpdater::RowFactory factory = [&, z](const CoreState& x, const AugBlock& a, std::vector<RowSet>* sets) {
    RowSet set;
    set.cls = MeasClass::PvtPos;
    for (int k = 0; k < 3; ++k) {
      MeasRow r;
      r.cls = MeasClass::PvtPos;
      r.H = Eigen::RowVectorXd::Zero(N);
      r.H(HostIdx::pos + k) = 1.0;
      r.H(HostIdx::dim + it + k) = 1.0;
      r.h = x.p(k) + a.x()(it + k);
      r.z = z(k);
      r.nu = r.z - r.h;
      r.R = 1.0;
      set.rows.push_back(r);
    }
    sets->push_back(set);
  };
  JointUpdater upd(cfg);
  JointUpdater::Options opt;
  const JointUpdater::Result res = upd.update(core, aug, factory, opt);
  EXPECT_TRUE(res.committed) << res.rollback_reason;
  EXPECT_FALSE(res.rolled_back);
  const Eigen::MatrixXd Sigma = aug.jointCov(core.P);
  EXPECT_TRUE(JointUpdater::posteriorPsd(Sigma));
  for (int k = 0; k < 3; ++k) {
    EXPECT_LT(Sigma(HostIdx::dim + it + k, HostIdx::dim + it + k), 2.0);   // strong update: 225 m² → ~1 m²
    EXPECT_NEAR(aug.x()(io + k), aug.x()(it + k), 1e-9);                     // the copy moved with t
    EXPECT_NEAR(Sigma(HostIdx::dim + io + k, HostIdx::dim + io + k), Sigma(HostIdx::dim + it + k, HostIdx::dim + it + k), 1e-9);
  }
  EXPECT_NEAR(aug.x()(io + 3), aug.x()(ic), 1e-9);
  // the exact zero directions t − copy stay at the rounding level
  Eigen::RowVectorXd v = Eigen::RowVectorXd::Zero(N);
  v(HostIdx::dim + it) = 1.0;
  v(HostIdx::dim + io) = -1.0;
  EXPECT_LT(std::fabs(v.dot(Sigma * v.transpose())), 1e-9);
}

// L7 Schmidt mode: host mean and P bit-identical (also with the host's asymmetric (I−G)P, DESIGN §16-11), C and
// Q_G equal to the Schmidt–Kalman closed form (Joseph with the host rows of K zeroed) — review m1. The third case
// (host σ_p = 1 m, Q_t = 1e-4, R = 0.01) has R − HΣ⁺Hᵀ indefinite for the Schmidt gain: L5 must use the general
// Cov(r_post) = (I−HK)S(I−HK)ᵀ and commit, not roll back.
TEST(U10Updater, SchmidtModeLeavesHostBitExact) {
  struct Case {
    bool asym, big_host_p;
  };
  for (const Case& c : {Case{false, false}, Case{true, false}, Case{false, true}}) {
    Config cfg;
    std::mt19937_64 rng(5);
    CoreState core;
    core.P = randomPd(HostIdx::dim, 1e-3, rng);
    if (c.asym) core.P(3, 4) += 1e-9;
    if (c.big_host_p) core.P.block<3, 3>(HostIdx::pos, HostIdx::pos) = Eigen::Matrix3d::Identity();
    AugBlock aug(cfg);
    Eigen::MatrixXd F3 = Eigen::MatrixXd::Zero(3, HostIdx::dim);
    F3.block<3, 3>(0, HostIdx::pos) = Eigen::Matrix3d::Identity();   // t correlated with the host position
    aug.addSlot(Slot::Trans, 0, "trans", Eigen::Vector3d(1.0, 2.0, 3.0), F3, Eigen::MatrixXd(3, 0),
                Eigen::Matrix3d::Identity() * (c.big_host_p ? 1e-4 : 1.0), core.P);
    const int N = HostIdx::dim + aug.dim();
    const int m = 3;
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(m, N);
    Eigen::VectorXd r = Eigen::VectorXd::Constant(m, 0.3);
    const Eigen::MatrixXd R = Eigen::MatrixXd::Identity(m, m) * (c.big_host_p ? 0.01 : 0.5);
    for (int k = 0; k < m; ++k) {
      H(k, HostIdx::pos + k) = 1.0;
      H(k, HostIdx::dim + k) = 1.0;
    }
    JointUpdater::RowFactory factory = [&](const CoreState& x, const AugBlock& a, std::vector<RowSet>* sets) {
      RowSet s;
      s.cls = MeasClass::PvtPos;
      for (int k = 0; k < m; ++k) {
        MeasRow row;
        row.cls = MeasClass::PvtPos;
        row.sat = sat(k + 1);
        row.H = H.row(k);
        row.h = x.p(k) + a.x()(k);
        row.nu = r(k);
        row.z = row.h + row.nu;
        row.R = R(k, k);
        s.rows.push_back(row);
      }
      sets->push_back(s);
    };
    const CoreState c0 = core;
    const Eigen::MatrixXd Sigma0 = aug.jointCov(core.P);
    const Eigen::VectorXd xg0 = aug.x();
    JointUpdater upd(cfg);
    JointUpdater::Options opt;
    opt.schmidt = true;   // L4/L5 stay on: the post-check must accept a Schmidt update
    const JointUpdater::Result res = upd.update(core, aug, factory, opt);
    ASSERT_TRUE(res.committed) << "asym " << c.asym << " big_host_p " << c.big_host_p << " rollback " << res.rollback_reason;
    EXPECT_EQ(res.dx_host.norm(), 0.0);
    EXPECT_EQ(std::memcmp(c0.P.data(), core.P.data(), sizeof(double) * HostIdx::dim * HostIdx::dim), 0) << "host P rewritten (asym " << c.asym << ")";
    EXPECT_EQ(std::memcmp(c0.R.data(), core.R.data(), sizeof(double) * 9), 0);
    EXPECT_EQ(std::memcmp(c0.p.data(), core.p.data(), sizeof(double) * 3), 0);
    EXPECT_EQ(std::memcmp(c0.v.data(), core.v.data(), sizeof(double) * 3), 0);
    EXPECT_EQ(std::memcmp(c0.bg.data(), core.bg.data(), sizeof(double) * 3), 0);
    EXPECT_EQ(std::memcmp(c0.ba.data(), core.ba.data(), sizeof(double) * 3), 0);
    EXPECT_EQ(std::memcmp(c0.g.data(), core.g.data(), sizeof(double) * 3), 0);
    EXPECT_EQ(c0.inv_expo, core.inv_expo);
    // Schmidt–Kalman closed form on the augmented block
    const Eigen::MatrixXd S = H * Sigma0 * H.transpose() + R;
    Eigen::MatrixXd K = Sigma0 * H.transpose() * S.inverse();
    K.topRows(HostIdx::dim).setZero();
    const Eigen::MatrixXd IKH = Eigen::MatrixXd::Identity(N, N) - K * H;
    Eigen::MatrixXd Sp = IKH * Sigma0 * IKH.transpose() + K * R * K.transpose();
    Sp = 0.5 * (Sp + Sp.transpose()).eval();
    const int n = aug.dim();
    // With the asymmetric P the closed form is ambiguous at the 1e-9 asymmetry (ΣHᵀ vs (HΣ)ᵀ): compare at that scale.
    const double tol = c.asym ? 1e-7 : 1e-10;
    EXPECT_LT((aug.x() - (xg0 + K.bottomRows(n) * r)).norm(), tol);
    EXPECT_LT((aug.Q() - Sp.bottomRightCorner(n, n)).norm() / Sp.bottomRightCorner(n, n).norm(), tol);
    EXPECT_LT((aug.C() - Sp.topRightCorner(HostIdx::dim, n)).norm() / Sp.topRightCorner(HostIdx::dim, n).norm(), tol);
    EXPECT_GT((aug.Q() - Sigma0.bottomRightCorner(n, n)).norm(), 1e-6);   // the block did move
  }
}

// AMENDMENTS A66, Options::consider: the Doppler rows do not update the tilt φ.
// A linear toy over [host 19 | φ 2 | ψ 1 | t 3 | clk 2] with every block correlated (random host prior, slots augmented
// from the host): 4 pseudorange rows on host position/t/b, 5 Doppler rows on host velocity/attitude/φ/ψ/d; gates off
// (exactness), then on (L5 must commit). Checked:
//   (a) the gain is K* = ΣHᵀS⁻¹ except φ's rows = [Σ_φ H_oᵀ S_oo⁻¹ | 0 on the Doppler rows]; mean and Σ⁺ (Joseph of that
//       gain) as committed; φ moved by K_φo ν_o only;
//   (b) the same state follows from processing the pseudorange rows first with the full gain and the Doppler rows next
//       with φ's gain zeroed (the constrained optimum is that sequential filter — an independent construction);
//   (c) minimum variance: any other gain from the pseudorange rows onto φ gives a larger tr Σ⁺_φφ;
//   (d) L5: the general-form posterior NIS (I − HK)S(I − HK)ᵀ of this gain equals r̃ᵀS⁻¹r̃, and a gated update commits
//       with the same state as the ungated one;
//   (e) φ's σ stays in the Doppler rows' S: the L4 statistics equal those without the consider entry;
//   (f) Doppler rows only: φ's mean is bit-identical to the prior and its variance does not shrink;
//   (g) bit-identical to no entry when the epoch has no Doppler row, or the block has no tilt slot.
TEST(U10Updater, ConsiderColumnsKeepTiltAndAreTheConstrainedOptimum) {
  Config cfg;
  std::mt19937_64 rng(66);
  std::normal_distribution<double> g(0.0, 1.0);
  CoreState core0;
  core0.P = randomPd(HostIdx::dim, 1e-3, rng);
  auto build = [&](bool with_tilt, AugBlock* aug) {
    auto rnd = [&](int r, int c, double s) {
      Eigen::MatrixXd M(r, c);
      for (int i = 0; i < r; ++i)
        for (int j = 0; j < c; ++j) M(i, j) = s * g(rng);
      return M;
    };
    if (with_tilt)
      aug->addSlot(Slot::Tilt, 0, "tilt", Eigen::Vector2d(1e-3, -2e-3), rnd(2, HostIdx::dim, 0.3), Eigen::MatrixXd(2, 0),
                   Eigen::Matrix2d::Identity() * 4e-6, core0.P);
    aug->addSlot(Slot::Yaw, 0, "yaw", Eigen::VectorXd::Constant(1, 0.01), rnd(1, HostIdx::dim, 0.3),
                 Eigen::MatrixXd::Zero(1, aug->dim()), Eigen::MatrixXd::Identity(1, 1) * 1e-6, core0.P);
    aug->addSlot(Slot::Trans, 0, "trans", Eigen::Vector3d(1.0, -2.0, 0.5), rnd(3, HostIdx::dim, 1.0),
                 Eigen::MatrixXd::Zero(3, aug->dim()), Eigen::Matrix3d::Identity() * 0.5, core0.P);
    Eigen::Matrix2d Nc = Eigen::Matrix2d::Zero();
    Nc(0, 0) = 4.0;
    Nc(1, 1) = 0.01;
    aug->addSlot(Slot::Clk, 0, "clk", Eigen::Vector2d(30.0, 0.2), Eigen::MatrixXd::Zero(2, HostIdx::dim),
                 Eigen::MatrixXd::Zero(2, aug->dim()), Nc, core0.P);
  };
  AugBlock aug0(cfg);
  build(true, &aug0);
  const int ip = aug0.find(Slot::Tilt), iy = aug0.find(Slot::Yaw), it = aug0.find(Slot::Trans), ic = aug0.find(Slot::Clk);
  ASSERT_EQ(ip, 0);
  const int n = aug0.dim(), N = HostIdx::dim + n;
  const Eigen::MatrixXd Sigma0 = aug0.jointCov(core0.P);
  // rows: pseudorange on p, t, b; Doppler on v, θ, φ, ψ, d
  const int mo = 4, md = 5, m = mo + md;
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(m, N);
  for (int i = 0; i < mo; ++i) {
    for (int k = 0; k < 3; ++k) {
      H(i, HostIdx::pos + k) = g(rng);
      H(i, HostIdx::dim + it + k) = H(i, HostIdx::pos + k);
    }
    H(i, HostIdx::dim + ic) = 1.0;
  }
  for (int i = mo; i < m; ++i) {
    for (int k = 0; k < 3; ++k) {
      H(i, HostIdx::vel + k) = g(rng);
      H(i, HostIdx::rot + k) = 0.1 * g(rng);
    }
    H(i, HostIdx::dim + ip) = 1.5 * g(rng);
    H(i, HostIdx::dim + ip + 1) = 1.5 * g(rng);
    H(i, HostIdx::dim + iy) = 1.5 * g(rng);
    H(i, HostIdx::dim + ic + 1) = 1.0;
  }
  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(m, m);
  for (int i = 0; i < mo; ++i) R(i, i) = 0.5 + 0.2 * i;
  for (int i = mo; i < m; ++i) R(i, i) = 0.01 + 0.002 * i;
  R(mo, mo + 1) = R(mo + 1, mo) = 0.003;   // the Doppler set carries a full R (A61 F8's common term)
  Eigen::VectorXd nu(m);
  {
    const Eigen::MatrixXd S = H * Sigma0 * H.transpose() + R;
    Eigen::LLT<Eigen::MatrixXd> L(S);
    Eigen::VectorXd e(m);
    for (int i = 0; i < m; ++i) e(i) = g(rng);
    nu = L.matrixL() * e;
    nu *= 0.5;   // a typical innovation (no row near the L4 thresholds, checked below)
  }
  // factory: a linear toy — the rows' innovation at an iterate is ν − H(x_i ⊟ x⁻) (the updater iterates at most once
  // more here and must land on the same point)
  auto factoryFor = [&](const CoreState& xc0, const Eigen::VectorXd& xg0, bool psr, bool dopp) {
    return JointUpdater::RowFactory([&, xc0, xg0, psr, dopp](const CoreState& x, const AugBlock& a, std::vector<RowSet>* sets) {
      Eigen::VectorXd dx(N);
      dx.head<HostIdx::dim>() = x.boxminus(xc0);
      dx.tail(n) = a.x() - xg0;
      if (psr) {
        RowSet s;
        s.cls = MeasClass::Psr;
        for (int i = 0; i < mo; ++i) {
          MeasRow r;
          r.cls = MeasClass::Psr;
          r.sat = sat(i + 1);
          r.H = H.row(i);
          r.nu = nu(i) - H.row(i).dot(dx);
          r.R = R(i, i);
          s.rows.push_back(r);
        }
        sets->push_back(s);
      }
      if (dopp) {
        RowSet s;
        s.cls = MeasClass::Dopp;
        s.has_full_R = true;
        s.R_full = R.block(mo, mo, md, md);
        for (int i = mo; i < m; ++i) {
          MeasRow r;
          r.cls = MeasClass::Dopp;
          r.sat = sat(i + 1);
          r.H = H.row(i);
          r.nu = nu(i) - H.row(i).dot(dx);
          r.R = R(i, i);
          s.rows.push_back(r);
        }
        sets->push_back(s);
      }
    });
  };
  JointUpdater upd(cfg);
  JointUpdater::Options opt;
  opt.disable_l4_l5 = true;
  opt.consider.push_back({MeasClass::Dopp, Slot::Tilt});
  // ---- (a) closed form ----
  const Eigen::MatrixXd S = H * Sigma0 * H.transpose() + R;
  Eigen::MatrixXd K = Sigma0 * H.transpose() * S.inverse();
  const Eigen::MatrixXd Soo = S.topLeftCorner(mo, mo);
  const Eigen::MatrixXd Kphi_o = Sigma0.middleRows(HostIdx::dim + ip, 2) * H.topRows(mo).transpose() * Soo.inverse();
  K.middleRows(HostIdx::dim + ip, 2).setZero();
  K.block(HostIdx::dim + ip, 0, 2, mo) = Kphi_o;
  const Eigen::MatrixXd IKH = Eigen::MatrixXd::Identity(N, N) - K * H;
  Eigen::MatrixXd Sp = IKH * Sigma0 * IKH.transpose() + K * R * K.transpose();
  Sp = 0.5 * (Sp + Sp.transpose()).eval();
  const Eigen::VectorXd dx_ref = K * nu;
  CoreState core = core0;
  AugBlock aug = aug0;
  const JointUpdater::Result res = upd.update(core, aug, factoryFor(core0, aug0.x(), true, true), opt);
  ASSERT_TRUE(res.committed) << res.rollback_reason;
  EXPECT_LT((aug.x() - aug0.x() - dx_ref.tail(n)).norm(), 1e-10 * dx_ref.norm());
  EXPECT_LT((core.boxminus(core0) - dx_ref.head<HostIdx::dim>()).norm(), 1e-10 * dx_ref.norm());
  EXPECT_LT((aug.x().segment<2>(ip) - aug0.x().segment<2>(ip) - Kphi_o * nu.head(mo)).norm(), 1e-12);
  const Eigen::MatrixXd Sigma1 = aug.jointCov(core.P);
  EXPECT_LT((Sigma1 - Sp).norm() / Sp.norm(), 1e-10);
  // the other columns keep the joint optimum: e.g. ψ and d moved by K*'s rows
  const Eigen::MatrixXd Kstar = Sigma0 * H.transpose() * S.inverse();
  EXPECT_NEAR(aug.x()(iy) - aug0.x()(iy), Kstar.row(HostIdx::dim + iy).dot(nu), 1e-12);
  EXPECT_NEAR(aug.x()(ic + 1) - aug0.x()(ic + 1), Kstar.row(HostIdx::dim + ic + 1).dot(nu), 1e-12);
  // ---- (b) sequential: pseudorange rows with the full gain, then Doppler rows with φ's gain zeroed ----
  {
    Eigen::MatrixXd Sg = Sigma0;
    Eigen::VectorXd x = Eigen::VectorXd::Zero(N);
    const Eigen::MatrixXd Ho = H.topRows(mo), Hd = H.bottomRows(md);
    const Eigen::MatrixXd Ro = R.topLeftCorner(mo, mo), Rd = R.bottomRightCorner(md, md);
    const Eigen::MatrixXd K1 = Sg * Ho.transpose() * (Ho * Sg * Ho.transpose() + Ro).inverse();
    x += K1 * (nu.head(mo) - Ho * x);
    Eigen::MatrixXd I1 = Eigen::MatrixXd::Identity(N, N) - K1 * Ho;
    Sg = I1 * Sg * I1.transpose() + K1 * Ro * K1.transpose();
    Eigen::MatrixXd K2 = Sg * Hd.transpose() * (Hd * Sg * Hd.transpose() + Rd).inverse();
    K2.middleRows(HostIdx::dim + ip, 2).setZero();
    x += K2 * (nu.tail(md) - Hd * x);
    Eigen::MatrixXd I2 = Eigen::MatrixXd::Identity(N, N) - K2 * Hd;
    Sg = I2 * Sg * I2.transpose() + K2 * Rd * K2.transpose();
    EXPECT_LT((x - dx_ref).norm(), 1e-10 * dx_ref.norm());
    EXPECT_LT((Sg - Sp).norm() / Sp.norm(), 1e-10);
  }
  // ---- (c) minimum variance of φ among the gains that use only the pseudorange rows ----
  {
    auto trPhi = [&](const Eigen::MatrixXd& Kp) {
      Eigen::MatrixXd Kx = K;
      Kx.block(HostIdx::dim + ip, 0, 2, mo) = Kp;
      const Eigen::MatrixXd A = Eigen::MatrixXd::Identity(N, N) - Kx * H;
      const Eigen::MatrixXd P = A * Sigma0 * A.transpose() + Kx * R * Kx.transpose();
      return P.block<2, 2>(HostIdx::dim + ip, HostIdx::dim + ip).trace();
    };
    const double best = trPhi(Kphi_o);
    for (int trial = 0; trial < 5; ++trial) {
      Eigen::MatrixXd dK(2, mo);
      for (int i = 0; i < 2; ++i)
        for (int j = 0; j < mo; ++j) dK(i, j) = 1e-3 * g(rng);
      EXPECT_GT(trPhi(Kphi_o + dK), best);
    }
    // the joint optimum's own φ rows restricted to the pseudorange columns (the naive "zero the Doppler entries") is worse
    EXPECT_GT(trPhi(Kstar.block(HostIdx::dim + ip, 0, 2, mo)), best);
  }
  // ---- (d) L5 on: commits the same state; general-form posterior NIS = r̃ᵀS⁻¹r̃ ----
  {
    const Eigen::MatrixXd G = Eigen::MatrixXd::Identity(m, m) - H * K;
    const Eigen::VectorXd r_post = G * nu;
    const Eigen::MatrixXd C = G * S * G.transpose();
    const double nis_general = r_post.dot(C.ldlt().solve(r_post));
    const double nis_prior = nu.dot(S.ldlt().solve(nu));
    EXPECT_NEAR(nis_general, nis_prior, 1e-7 * nis_prior);
    CoreState c2 = core0;
    AugBlock a2 = aug0;
    JointUpdater::Options o2;
    o2.consider = opt.consider;
    o2.sigma_prior_pos_m = 1.0;
    const JointUpdater::Result r2 = upd.update(c2, a2, factoryFor(core0, aug0.x(), true, true), o2);
    ASSERT_TRUE(r2.committed) << r2.rollback_reason;
    ASSERT_EQ(r2.gates[static_cast<size_t>(MeasClass::Psr)].n_out, mo);
    ASSERT_EQ(r2.gates[static_cast<size_t>(MeasClass::Dopp)].n_out, md);
    EXPECT_NEAR(r2.nis_post, nis_prior, 1e-9 * nis_prior);
    EXPECT_LT((a2.x() - aug.x()).norm(), 1e-12);
    // ---- (e) the gates' statistics do not see the consider entry ----
    CoreState c3 = core0;
    AugBlock a3 = aug0;
    JointUpdater::Options o3;
    o3.sigma_prior_pos_m = 1.0;
    const JointUpdater::Result r3 = upd.update(c3, a3, factoryFor(core0, aug0.x(), true, true), o3);
    ASSERT_TRUE(r3.committed);
    for (MeasClass c : {MeasClass::Psr, MeasClass::Dopp}) {
      EXPECT_EQ(r2.gates[static_cast<size_t>(c)].nis, r3.gates[static_cast<size_t>(c)].nis);
      EXPECT_EQ(r2.gates[static_cast<size_t>(c)].n_out, r3.gates[static_cast<size_t>(c)].n_out);
    }
    EXPECT_NE((a3.x().segment<2>(ip) - a2.x().segment<2>(ip)).norm(), 0.0);   // without the entry φ takes the Doppler gain
  }
  // ---- (f) Doppler rows only: φ's mean is the prior's to the bit, its variance does not shrink ----
  {
    CoreState c4 = core0;
    AugBlock a4 = aug0;
    const JointUpdater::Result r4 = upd.update(c4, a4, factoryFor(core0, aug0.x(), false, true), opt);
    ASSERT_TRUE(r4.committed);
    EXPECT_EQ(std::memcmp(a4.x().data() + ip, aug0.x().data() + ip, 2 * sizeof(double)), 0);
    EXPECT_NE(a4.x()(iy), aug0.x()(iy));   // ψ still moves
    const Eigen::MatrixXd Sg4 = a4.jointCov(c4.P);
    // Σ⁺_φφ = Σ_φφ exactly in exact arithmetic (K_φ = 0 ⇒ the φ block of (I − KH)Σ(I − KH)ᵀ + KRKᵀ is Σ_φφ)
    EXPECT_LT((Sg4.block<2, 2>(HostIdx::dim + ip, HostIdx::dim + ip) - Sigma0.block<2, 2>(HostIdx::dim + ip, HostIdx::dim + ip)).norm(),
              1e-15);
  }
  // ---- (g) bit-identical to no entry: no Doppler row in the epoch; no tilt slot in the block ----
  {
    CoreState c5 = core0, c6 = core0;
    AugBlock a5 = aug0, a6 = aug0;
    JointUpdater::Options plain;
    plain.disable_l4_l5 = true;
    upd.update(c5, a5, factoryFor(core0, aug0.x(), true, false), opt);
    upd.update(c6, a6, factoryFor(core0, aug0.x(), true, false), plain);
    EXPECT_TRUE(sameBytes(snapshotBytes(c5, a5), snapshotBytes(c6, a6)));
  }
  {
    AugBlock b0(cfg);
    build(false, &b0);
    ASSERT_LT(b0.find(Slot::Tilt), 0);
    const int nb = b0.dim(), Nb = HostIdx::dim + nb;
    JointUpdater::RowFactory f = [&](const CoreState& x, const AugBlock& a, std::vector<RowSet>* sets) {
      (void)x;
      (void)a;
      RowSet s;
      s.cls = MeasClass::Dopp;
      for (int i = mo; i < m; ++i) {
        MeasRow r;
        r.cls = MeasClass::Dopp;
        r.sat = sat(i + 1);
        r.H = Eigen::RowVectorXd(Nb);   // [host | ψ t clk]: the toy's columns without the tilt's
        r.H.head<HostIdx::dim>() = H.row(i).head<HostIdx::dim>();
        r.H.tail(nb) = H.row(i).tail(nb);
        r.nu = nu(i);
        r.R = R(i, i);
        s.rows.push_back(r);
      }
      sets->push_back(s);
    };
    CoreState c7 = core0, c8 = core0;
    AugBlock a7 = b0, a8 = b0;
    JointUpdater::Options plain;
    plain.disable_l4_l5 = true;
    upd.update(c7, a7, f, opt);
    upd.update(c8, a8, f, plain);
    EXPECT_TRUE(sameBytes(snapshotBytes(c7, a7), snapshotBytes(c8, a8)));
    EXPECT_FALSE(sameBytes(snapshotBytes(c7, a7), snapshotBytes(core0, b0)));   // the update did happen
  }
}

// AMENDMENTS A69/A70, Options::project: the accepted Doppler rows projected, as one group in the metric of their R, onto
// the orthogonal complement of their tilt columns. The consider toy above (all blocks correlated, the Doppler set with a
// full R), 4 pseudorange + 6 Doppler rows, gates off unless stated. Checked:
//   (a) projection(): rank 2, m − 2 rows T H, T r with T R Tᵀ = I and the tilt columns zero (1e-12 of the whitened ones);
//   (b) their information is the batch least-squares elimination of φ as a free nuisance parameter: TᵀT-weighted
//       Hᵀ(·)H = J_oo − J_oφ J_φφ⁻¹ J_φo on the other columns (J = HᵀR⁻¹H), zero on φ's;
//   (c) the committed update equals (c1) a Kalman update with the complement basis taken independently (SVD) — the result
//       does not depend on the basis — and (c2) the information-form batch solution with the Doppler rows' φ replaced by
//       a separate parameter without prior, marginalised;
//   (d) with the gates on it commits the same state; the L4 statistics equal those without the entry (the gates judge the
//       rows before the projection); L5's posterior NIS is r̃ᵀS⁻¹r̃ of the projected system;
//   (e) the channel of A67: φ uncorrelated with everything a priori, Doppler rows only. Without an entry φ moves and the
//       host becomes correlated with φ; with the A66 consider entry φ stays but the correlation is made all the same, and
//       a following host update (H2) moves φ; projected, neither happens (φ, its row of Σ and the H2 step stay at rounding);
//   (f) project + consider on a Doppler-only epoch: φ's mean bit-identical to the prior;
//   (g) bit-identical to no entry without Doppler rows, without a tilt slot, and when the tilt columns are zero (rank 0);
//   (h) exactly proportional tilt columns (level motion): rank 1, one row dropped, the information is the elimination of
//       the one combination the rows see — a Householder basis of the two columns would drop one more (information lost);
//   (i) two Doppler rows (m ≤ rank): nothing kept — bit-identical to the pseudorange rows alone; Doppler only ⇒ no update;
//   (j) an R that is not positive definite ⇒ rollback "proj_failed", state bit-identical;
//   (k) the relinearised iterates (forced three) land on the single-step state.
TEST(U10Updater, ProjectedDopplerRowsAreTiltFreeAndKeepTheRest) {
  Config cfg;
  std::mt19937_64 rng(70);
  std::normal_distribution<double> g(0.0, 1.0);
  CoreState core0;
  core0.P = randomPd(HostIdx::dim, 1e-3, rng);
  auto rnd = [&](int r, int c, double s) {
    Eigen::MatrixXd M(r, c);
    for (int i = 0; i < r; ++i)
      for (int j = 0; j < c; ++j) M(i, j) = s * g(rng);
    return M;
  };
  // tilt_mode: 0 correlated tilt slot first, 1 no tilt slot, 2 tilt slot uncorrelated with everything (added last)
  auto build = [&](int tilt_mode, AugBlock* aug) {
    if (tilt_mode == 0)
      aug->addSlot(Slot::Tilt, 0, "tilt", Eigen::Vector2d(1e-3, -2e-3), rnd(2, HostIdx::dim, 0.3), Eigen::MatrixXd(2, 0),
                   Eigen::Matrix2d::Identity() * 4e-6, core0.P);
    aug->addSlot(Slot::Yaw, 0, "yaw", Eigen::VectorXd::Constant(1, 0.01), rnd(1, HostIdx::dim, 0.3),
                 Eigen::MatrixXd::Zero(1, aug->dim()), Eigen::MatrixXd::Identity(1, 1) * 1e-6, core0.P);
    aug->addSlot(Slot::Trans, 0, "trans", Eigen::Vector3d(1.0, -2.0, 0.5), rnd(3, HostIdx::dim, 1.0),
                 Eigen::MatrixXd::Zero(3, aug->dim()), Eigen::Matrix3d::Identity() * 0.5, core0.P);
    Eigen::Matrix2d Nc = Eigen::Matrix2d::Zero();
    Nc(0, 0) = 4.0;
    Nc(1, 1) = 0.01;
    aug->addSlot(Slot::Clk, 0, "clk", Eigen::Vector2d(30.0, 0.2), Eigen::MatrixXd::Zero(2, HostIdx::dim),
                 Eigen::MatrixXd::Zero(2, aug->dim()), Nc, core0.P);
    if (tilt_mode == 2)
      aug->addSlot(Slot::Tilt, 0, "tilt", Eigen::Vector2d(1e-3, -2e-3), Eigen::MatrixXd::Zero(2, HostIdx::dim),
                   Eigen::MatrixXd::Zero(2, aug->dim()), Eigen::Matrix2d::Identity() * 4e-6, core0.P);
  };
  AugBlock aug0(cfg);
  build(0, &aug0);
  const int ip = aug0.find(Slot::Tilt), iy = aug0.find(Slot::Yaw), it = aug0.find(Slot::Trans), ic = aug0.find(Slot::Clk);
  ASSERT_EQ(ip, 0);
  const int n = aug0.dim(), N = HostIdx::dim + n;
  const std::vector<int> tcols = {HostIdx::dim + ip, HostIdx::dim + ip + 1};
  const Eigen::MatrixXd Sigma0 = aug0.jointCov(core0.P);
  const int mo = 4, md = 6;
  // rows over the layout of aug0 ([host | φ ψ t clk]); `dopp_rows` builds md Doppler rows with the given tilt columns
  Eigen::MatrixXd Ho = Eigen::MatrixXd::Zero(mo, N);
  for (int i = 0; i < mo; ++i) {
    for (int k = 0; k < 3; ++k) {
      Ho(i, HostIdx::pos + k) = g(rng);
      Ho(i, HostIdx::dim + it + k) = Ho(i, HostIdx::pos + k);
    }
    Ho(i, HostIdx::dim + ic) = 1.0;
  }
  Eigen::MatrixXd Hd = Eigen::MatrixXd::Zero(md, N);
  for (int i = 0; i < md; ++i) {
    for (int k = 0; k < 3; ++k) {
      Hd(i, HostIdx::vel + k) = g(rng);
      Hd(i, HostIdx::rot + k) = 0.1 * g(rng);
    }
    Hd(i, HostIdx::dim + ip) = 1.5 * g(rng);
    Hd(i, HostIdx::dim + ip + 1) = 1.5 * g(rng);
    Hd(i, HostIdx::dim + iy) = 1.5 * g(rng);
    Hd(i, HostIdx::dim + ic + 1) = 1.0;
  }
  Eigen::MatrixXd Ro = Eigen::MatrixXd::Zero(mo, mo), Rd = Eigen::MatrixXd::Zero(md, md);
  for (int i = 0; i < mo; ++i) Ro(i, i) = 0.5 + 0.2 * i;
  for (int i = 0; i < md; ++i) Rd(i, i) = 0.01 + 0.002 * i;
  Rd(0, 1) = Rd(1, 0) = 0.003;   // the Doppler set carries a full R (A61 F8's common term)
  Rd(2, 4) = Rd(4, 2) = -0.002;
  Eigen::VectorXd nuo(mo), nud(md);
  {
    Eigen::MatrixXd H(mo + md, N), R = Eigen::MatrixXd::Zero(mo + md, mo + md);
    H << Ho, Hd;
    R.topLeftCorner(mo, mo) = Ro;
    R.bottomRightCorner(md, md) = Rd;
    Eigen::LLT<Eigen::MatrixXd> L(Eigen::MatrixXd(H * Sigma0 * H.transpose() + R));
    Eigen::VectorXd e(mo + md);
    for (int i = 0; i < mo + md; ++i) e(i) = g(rng);
    Eigen::VectorXd nu = L.matrixL() * e;
    nu *= 0.5;   // a typical innovation (no row near the L4 thresholds)
    nuo = nu.head(mo);
    nud = nu.tail(md);
  }
  struct Block {
    MeasClass cls;
    Eigen::MatrixXd H, R;
    Eigen::VectorXd nu;
  };
  // factory of a linear toy: the innovation at an iterate is ν − H(x_i ⊟ x⁻)
  auto factoryOf = [&](const CoreState& xc0, const Eigen::VectorXd& xg0, const std::vector<Block>& blocks) {
    const int Nb = HostIdx::dim + static_cast<int>(xg0.size());
    return JointUpdater::RowFactory([xc0, xg0, blocks, Nb](const CoreState& x, const AugBlock& a, std::vector<RowSet>* sets) {
      Eigen::VectorXd dx(Nb);
      dx.head<HostIdx::dim>() = x.boxminus(xc0);
      dx.tail(Nb - HostIdx::dim) = a.x() - xg0;
      int id = 1;
      for (const Block& b : blocks) {
        RowSet s;
        s.cls = b.cls;
        s.has_full_R = true;
        s.R_full = b.R;
        for (int i = 0; i < b.H.rows(); ++i) {
          MeasRow r;
          r.cls = b.cls;
          r.sat = sat(id++);
          r.H = b.H.row(i);
          r.nu = b.nu(i) - b.H.row(i).dot(dx);
          r.R = b.R(i, i);
          s.rows.push_back(r);
        }
        sets->push_back(s);
      }
    });
  };
  const Block psr{MeasClass::Psr, Ho, Ro, nuo}, dop{MeasClass::Dopp, Hd, Rd, nud};
  JointUpdater upd(cfg);
  JointUpdater::Options plain, proj;
  plain.disable_l4_l5 = proj.disable_l4_l5 = true;
  proj.project.push_back({MeasClass::Dopp, Slot::Tilt});
  auto whiten = [](const Eigen::MatrixXd& R) { return Eigen::MatrixXd(Eigen::LLT<Eigen::MatrixXd>(R).matrixL()); };
  // the independent complement basis of (c1): left singular vectors of the whitened tilt columns beyond their rank
  auto svdT = [&](const Eigen::MatrixXd& H, const Eigen::MatrixXd& R, int rank) {
    const Eigen::MatrixXd L = whiten(R);
    const Eigen::MatrixXd Li = L.triangularView<Eigen::Lower>().solve(Eigen::MatrixXd::Identity(R.rows(), R.rows()));
    Eigen::MatrixXd F(H.rows(), 2);
    F << (Li * H).col(tcols[0]), (Li * H).col(tcols[1]);
    Eigen::JacobiSVD<Eigen::MatrixXd> sv(F, Eigen::ComputeFullU);
    return Eigen::MatrixXd(sv.matrixU().rightCols(H.rows() - rank).transpose() * Li);
  };
  // ---- (a) the helper ----
  Eigen::MatrixXd Hp;
  Eigen::VectorXd rp;
  int rank = -1;
  ASSERT_TRUE(JointUpdater::projection(Hd, nud, Rd, tcols, &Hp, &rp, &rank));
  ASSERT_EQ(rank, 2);
  ASSERT_EQ(Hp.rows(), md - 2);
  {
    // T from the same construction on [H | I] (its extra columns are T itself)
    Eigen::MatrixXd HI(md, N + md);
    HI << Hd, Eigen::MatrixXd::Identity(md, md);
    Eigen::MatrixXd HpI;
    Eigen::VectorXd rpI;
    int rI = -1;
    ASSERT_TRUE(JointUpdater::projection(HI, nud, Rd, tcols, &HpI, &rpI, &rI));
    ASSERT_EQ(rI, 2);
    const Eigen::MatrixXd T = HpI.rightCols(md);
    EXPECT_LT((T * Rd * T.transpose() - Eigen::MatrixXd::Identity(md - 2, md - 2)).norm(), 1e-12) << "kept rows not unit-variance";
    EXPECT_LT((T * Hd - Hp).norm(), 1e-12 * Hp.norm());
    EXPECT_LT((T * nud - rp).norm(), 1e-12 * rp.norm());
    const Eigen::MatrixXd Fw = whiten(Rd).triangularView<Eigen::Lower>().solve(Eigen::MatrixXd(Hd.middleCols(tcols[0], 2)));
    EXPECT_LT(Hp.middleCols(tcols[0], 2).norm(), 1e-12 * Fw.norm()) << "the kept rows still see the tilt";
    EXPECT_GT(Fw.norm(), 1.0);
  }
  // ---- (b) information = elimination of φ as a free nuisance ----
  {
    const Eigen::MatrixXd J = Hd.transpose() * Rd.ldlt().solve(Hd);
    const Eigen::MatrixXd Jp = Hp.transpose() * Hp;
    std::vector<int> oth;
    for (int k = 0; k < N; ++k)
      if (k != tcols[0] && k != tcols[1]) oth.push_back(k);
    const int no = static_cast<int>(oth.size());
    Eigen::MatrixXd Joo(no, no), Jof(no, 2), Jff(2, 2), Jpoo(no, no);
    for (int a = 0; a < no; ++a) {
      for (int b = 0; b < no; ++b) {
        Joo(a, b) = J(oth[a], oth[b]);
        Jpoo(a, b) = Jp(oth[a], oth[b]);
      }
      for (int b = 0; b < 2; ++b) Jof(a, b) = J(oth[a], tcols[b]);
    }
    for (int a = 0; a < 2; ++a)
      for (int b = 0; b < 2; ++b) Jff(a, b) = J(tcols[a], tcols[b]);
    const Eigen::MatrixXd schur = Joo - Jof * Jff.inverse() * Jof.transpose();
    EXPECT_LT((Jpoo - schur).norm(), 1e-10 * schur.norm()) << "the kept information is not the elimination of φ";
    EXPECT_LT(Jp.middleCols(tcols[0], 2).norm(), 1e-12 * J.norm());
    EXPECT_GT((Joo - schur).norm(), 1e-3 * schur.norm()) << "the toy's rows must have something along φ to lose";
  }
  // ---- (c) the committed update ----
  CoreState core = core0;
  AugBlock aug = aug0;
  const JointUpdater::Result res = upd.update(core, aug, factoryOf(core0, aug0.x(), {psr, dop}), proj);
  ASSERT_TRUE(res.committed) << res.rollback_reason;
  Eigen::VectorXd dx_upd(N);
  dx_upd.head<HostIdx::dim>() = core.boxminus(core0);
  dx_upd.tail(n) = aug.x() - aug0.x();
  const Eigen::MatrixXd Sigma1 = aug.jointCov(core.P);
  {
    // (c1) Kalman update with an independent complement basis
    const Eigen::MatrixXd T2 = svdT(Hd, Rd, 2);
    Eigen::MatrixXd H(mo + md - 2, N), R = Eigen::MatrixXd::Zero(mo + md - 2, mo + md - 2);
    H << Ho, T2 * Hd;
    R.topLeftCorner(mo, mo) = Ro;
    R.bottomRightCorner(md - 2, md - 2).setIdentity();
    Eigen::VectorXd nu(mo + md - 2);
    nu << nuo, T2 * nud;
    const Eigen::MatrixXd S = H * Sigma0 * H.transpose() + R;
    const Eigen::MatrixXd K = Sigma0 * H.transpose() * S.inverse();
    const Eigen::VectorXd dx_ref = K * nu;
    const Eigen::MatrixXd Sp = Sigma0 - K * S * K.transpose();
    EXPECT_LT((dx_upd - dx_ref).norm(), 1e-10 * dx_ref.norm());
    EXPECT_LT((Sigma1 - Sp).norm(), 1e-10 * Sp.norm());
    // (c2) information-form batch: [x; φ'] with φ' in the Doppler rows instead of φ, no prior on φ', φ' marginalised
    const Eigen::MatrixXd Lam0 = Sigma0.inverse();
    Eigen::MatrixXd Hdx = Hd;
    Hdx.middleCols(tcols[0], 2).setZero();
    const Eigen::MatrixXd Fd = Hd.middleCols(tcols[0], 2);
    const Eigen::MatrixXd Rdi = Rd.inverse(), Roi = Ro.inverse();
    const Eigen::MatrixXd Lxx = Lam0 + Ho.transpose() * Roi * Ho + Hdx.transpose() * Rdi * Hdx;
    const Eigen::MatrixXd Lxf = Hdx.transpose() * Rdi * Fd, Lff = Fd.transpose() * Rdi * Fd;
    const Eigen::VectorXd ex = Ho.transpose() * Roi * nuo + Hdx.transpose() * Rdi * nud, ef = Fd.transpose() * Rdi * nud;
    const Eigen::MatrixXd Lm = Lxx - Lxf * Lff.inverse() * Lxf.transpose();
    const Eigen::VectorXd em = ex - Lxf * Lff.inverse() * ef;
    const Eigen::MatrixXd Sb = Lm.inverse();
    const Eigen::VectorXd dxb = Sb * em;
    EXPECT_LT((dx_upd - dxb).norm(), 1e-7 * dxb.norm()) << "not the batch LS with φ a free nuisance in the Doppler rows";
    EXPECT_LT((Sigma1 - Sb).norm(), 1e-7 * Sb.norm());
    // φ moved (through its prior correlations), by the optimal gain of the projected system
    EXPECT_GT((aug.x().segment<2>(ip) - aug0.x().segment<2>(ip)).norm(), 1e-9);
  }
  // ---- (d) gates on ----
  {
    JointUpdater::Options gp = proj, gq;
    gp.disable_l4_l5 = false;
    gp.sigma_prior_pos_m = gq.sigma_prior_pos_m = 1.0;
    CoreState c2 = core0, c3 = core0;
    AugBlock a2 = aug0, a3 = aug0;
    const JointUpdater::Result r2 = upd.update(c2, a2, factoryOf(core0, aug0.x(), {psr, dop}), gp);
    const JointUpdater::Result r3 = upd.update(c3, a3, factoryOf(core0, aug0.x(), {psr, dop}), gq);
    ASSERT_TRUE(r2.committed) << r2.rollback_reason;
    ASSERT_TRUE(r3.committed) << r3.rollback_reason;
    ASSERT_EQ(r2.gates[static_cast<size_t>(MeasClass::Dopp)].n_out, md);
    for (MeasClass c : {MeasClass::Psr, MeasClass::Dopp}) {
      EXPECT_EQ(r2.gates[static_cast<size_t>(c)].nis, r3.gates[static_cast<size_t>(c)].nis);
      EXPECT_EQ(r2.gates[static_cast<size_t>(c)].n_out, r3.gates[static_cast<size_t>(c)].n_out);
      EXPECT_EQ(r2.gates[static_cast<size_t>(c)].w, r3.gates[static_cast<size_t>(c)].w);
    }
    EXPECT_LT((a2.x() - aug.x()).norm(), 1e-12);
    const Eigen::MatrixXd T2 = svdT(Hd, Rd, 2);
    Eigen::MatrixXd H(mo + md - 2, N), R = Eigen::MatrixXd::Zero(mo + md - 2, mo + md - 2);
    H << Ho, T2 * Hd;
    R.topLeftCorner(mo, mo) = Ro;
    R.bottomRightCorner(md - 2, md - 2).setIdentity();
    Eigen::VectorXd nu(mo + md - 2);
    nu << nuo, T2 * nud;
    const double nis = nu.dot((H * Sigma0 * H.transpose() + R).ldlt().solve(nu));
    EXPECT_NEAR(r2.nis_post, nis, 1e-9 * nis);
  }
  // ---- (e) the H2 channel of A67 ----
  {
    AugBlock b0(cfg);
    build(2, &b0);   // φ uncorrelated with everything
    const int jp = b0.find(Slot::Tilt), jy = b0.find(Slot::Yaw), jt = b0.find(Slot::Trans), jc = b0.find(Slot::Clk);
    ASSERT_EQ(jy, 0);
    const int nb = b0.dim(), Nb = HostIdx::dim + nb;
    // the same Doppler rows on b0's layout
    Eigen::MatrixXd Hb = Eigen::MatrixXd::Zero(md, Nb);
    Hb.leftCols(HostIdx::dim) = Hd.leftCols(HostIdx::dim);
    Hb.col(HostIdx::dim + jp) = Hd.col(HostIdx::dim + ip);
    Hb.col(HostIdx::dim + jp + 1) = Hd.col(HostIdx::dim + ip + 1);
    Hb.col(HostIdx::dim + jy) = Hd.col(HostIdx::dim + iy);
    Hb.middleCols(HostIdx::dim + jt, 3) = Hd.middleCols(HostIdx::dim + it, 3);
    Hb.middleCols(HostIdx::dim + jc, 2) = Hd.middleCols(HostIdx::dim + ic, 2);
    const Block dopb{MeasClass::Dopp, Hb, Rd, nud};
    const Eigen::MatrixXd Sb0 = b0.jointCov(core0.P);
    ASSERT_EQ(Sb0.block(HostIdx::dim + jp, 0, 2, HostIdx::dim + jp).norm(), 0.0);
    // then a host update that pins the velocity (the LIO's role) and H2
    auto afterH2 = [&](const JointUpdater::Options& o, Eigen::Vector2d* dphi_update, double* corr, Eigen::Vector2d* dphi_h2) {
      CoreState c = core0;
      AugBlock a = b0;
      const JointUpdater::Result r = upd.update(c, a, factoryOf(core0, b0.x(), {dopb}), o);
      EXPECT_TRUE(r.committed) << r.rollback_reason;
      *dphi_update = a.x().segment<2>(jp) - b0.x().segment<2>(jp);
      *corr = a.C().middleCols(jp, 2).norm();
      Eigen::MatrixXd Hl = Eigen::MatrixXd::Zero(3, HostIdx::dim);
      Hl.middleCols(HostIdx::vel, 3).setIdentity();
      const HostMat Pm = c.P;
      const Eigen::MatrixXd Kl = Pm * Hl.transpose() * (Hl * Pm * Hl.transpose() + 1e-6 * Eigen::Matrix3d::Identity()).inverse();
      const HostVec dxl = Kl * Eigen::Vector3d(0.05, -0.03, 0.02);
      HostMat Pp = Pm - Kl * Hl * Pm;
      Pp = 0.5 * (Pp + Pp.transpose()).eval();
      bool pinv = false;
      const Eigen::Vector2d before = a.x().segment<2>(jp);
      EXPECT_TRUE(a.conditionOnHostUpdate(Pm, Pp, dxl, &pinv));
      *dphi_h2 = a.x().segment<2>(jp) - before;
    };
    Eigen::Vector2d du, dh;
    double corr = 0.0;
    JointUpdater::Options cons = plain;
    cons.consider.push_back({MeasClass::Dopp, Slot::Tilt});
    // (σ_φ = 2e-3 rad; this toy: the joint gain moves φ by 3.3e-6, makes ‖C_hφ‖ = 2.2e-6, and H2 then moves φ by 6.6e-5 —
    // twenty times the update's own step, A67's 8–12× on the development set; consider: 0, 2.2e-6, 6.6e-5)
    const double sphi = 2e-3;
    afterH2(plain, &du, &corr, &dh);   // the joint gain: φ moves and the host–φ correlation is made
    EXPECT_GT(du.norm(), 1e-4 * sphi);
    EXPECT_GT(corr, 1e-8);
    EXPECT_GT(dh.norm(), 1e-4 * sphi);
    afterH2(cons, &du, &corr, &dh);    // A66 consider: φ stays in the update, the correlation is made all the same
    EXPECT_EQ(du.norm(), 0.0);
    EXPECT_GT(corr, 1e-8) << "consider no longer leaves the correlation (A67's channel)";
    EXPECT_GT(dh.norm(), 1e-4 * sphi) << "H2 no longer hands the Doppler content to φ under consider";
    const double corr_cons = corr, dh_cons = dh.norm();
    afterH2(proj, &du, &corr, &dh);    // projected: no φ content, no correlation, nothing for H2 to pass on
    EXPECT_LT(du.norm(), 1e-12 * sphi);
    EXPECT_LT(corr, 1e-12 * corr_cons);
    EXPECT_LT(dh.norm(), 1e-12 * dh_cons);
    CoreState c4 = core0;
    AugBlock a4 = b0;
    upd.update(c4, a4, factoryOf(core0, b0.x(), {dopb}), proj);
    EXPECT_LT((a4.Q().block<2, 2>(jp, jp) - Sb0.block<2, 2>(HostIdx::dim + jp, HostIdx::dim + jp)).norm(), 1e-15)
        << "φ's variance changed without φ content in the rows";
  }
  // ---- (f) project + consider on a Doppler-only epoch: φ bit-identical ----
  {
    JointUpdater::Options pc = proj;
    pc.consider.push_back({MeasClass::Dopp, Slot::Tilt});
    CoreState c5 = core0;
    AugBlock a5 = aug0;
    const JointUpdater::Result r5 = upd.update(c5, a5, factoryOf(core0, aug0.x(), {dop}), pc);
    ASSERT_TRUE(r5.committed);
    EXPECT_EQ(std::memcmp(a5.x().data() + ip, aug0.x().data() + ip, 2 * sizeof(double)), 0);
    EXPECT_NE(a5.x()(iy), aug0.x()(iy));
  }
  // ---- (g) bit-identical to no entry: no Doppler rows; no tilt slot; zero tilt columns (rank 0) ----
  {
    CoreState c6 = core0, c7 = core0;
    AugBlock a6 = aug0, a7 = aug0;
    upd.update(c6, a6, factoryOf(core0, aug0.x(), {psr}), proj);
    upd.update(c7, a7, factoryOf(core0, aug0.x(), {psr}), plain);
    EXPECT_TRUE(sameBytes(snapshotBytes(c6, a6), snapshotBytes(c7, a7)));
    AugBlock b1(cfg);
    build(1, &b1);
    ASSERT_LT(b1.find(Slot::Tilt), 0);
    const int nb = b1.dim(), Nb = HostIdx::dim + nb;
    Eigen::MatrixXd Hb(md, Nb);
    Hb.leftCols(HostIdx::dim) = Hd.leftCols(HostIdx::dim);
    Hb.rightCols(nb) = Hd.rightCols(nb);   // [ψ t clk] follow the tilt in aug0's layout
    const Block dopb{MeasClass::Dopp, Hb, Rd, nud};
    CoreState c8 = core0, c9 = core0;
    AugBlock a8 = b1, a9 = b1;
    upd.update(c8, a8, factoryOf(core0, b1.x(), {dopb}), proj);
    upd.update(c9, a9, factoryOf(core0, b1.x(), {dopb}), plain);
    EXPECT_TRUE(sameBytes(snapshotBytes(c8, a8), snapshotBytes(c9, a9)));
    EXPECT_FALSE(sameBytes(snapshotBytes(c8, a8), snapshotBytes(core0, b1)));
    Eigen::MatrixXd Hz = Hd;
    Hz.middleCols(tcols[0], 2).setZero();
    const Block dopz{MeasClass::Dopp, Hz, Rd, nud};
    Eigen::MatrixXd Hzp;
    Eigen::VectorXd rzp;
    int rz = -1;
    ASSERT_TRUE(JointUpdater::projection(Hz, nud, Rd, tcols, &Hzp, &rzp, &rz));
    EXPECT_EQ(rz, 0);
    CoreState c10 = core0, c11 = core0;
    AugBlock a10 = aug0, a11 = aug0;
    upd.update(c10, a10, factoryOf(core0, aug0.x(), {psr, dopz}), proj);
    upd.update(c11, a11, factoryOf(core0, aug0.x(), {psr, dopz}), plain);
    EXPECT_TRUE(sameBytes(snapshotBytes(c10, a10), snapshotBytes(c11, a11)));
  }
  // ---- (h) proportional tilt columns: rank 1 ----
  {
    Eigen::MatrixXd H1 = Hd;
    H1.col(tcols[1]) = 0.5 * H1.col(tcols[0]);
    Eigen::MatrixXd H1p;
    Eigen::VectorXd r1p;
    int r1 = -1;
    ASSERT_TRUE(JointUpdater::projection(H1, nud, Rd, tcols, &H1p, &r1p, &r1));
    ASSERT_EQ(r1, 1);
    ASSERT_EQ(H1p.rows(), md - 1);
    const Eigen::MatrixXd L = whiten(Rd);
    const Eigen::MatrixXd Fw = L.triangularView<Eigen::Lower>().solve(Eigen::MatrixXd(H1.middleCols(tcols[0], 2)));
    EXPECT_LT(H1p.middleCols(tcols[0], 2).norm(), 1e-12 * Fw.norm());
    // information: the elimination of the one combination φ₁ + ½φ₂ the rows see (column c₁)
    const Eigen::MatrixXd J = H1.transpose() * Rd.ldlt().solve(H1);
    const Eigen::MatrixXd Jp = H1p.transpose() * H1p;
    const Eigen::VectorXd j1 = J.col(tcols[0]);
    Eigen::MatrixXd schur = J - j1 * j1.transpose() / J(tcols[0], tcols[0]);
    for (int k : tcols) {
      schur.row(k).setZero();
      schur.col(k).setZero();
    }
    Eigen::MatrixXd Jpo = Jp;
    for (int k : tcols) {
      Jpo.row(k).setZero();
      Jpo.col(k).setZero();
    }
    EXPECT_LT((Jpo - schur).norm(), 1e-10 * schur.norm());
    // a Householder basis of the two columns takes out a second direction as well: less information
    Eigen::HouseholderQR<Eigen::MatrixXd> qr(Fw);
    const Eigen::MatrixXd Q = qr.householderQ() * Eigen::MatrixXd::Identity(md, md);
    const Eigen::MatrixXd Hh = Q.rightCols(md - 2).transpose() * L.triangularView<Eigen::Lower>().solve(H1);
    EXPECT_GT(Jp.trace() - (Hh.transpose() * Hh).trace(), 1e-3 * Jp.trace()) << "the rank-1 case lost nothing to a 2-D basis?";
  }
  // ---- (i) m ≤ rank: two Doppler rows keep nothing ----
  {
    const Block dop2{MeasClass::Dopp, Hd.topRows(2), Rd.topLeftCorner(2, 2), nud.head(2)};
    Eigen::MatrixXd H2p;
    Eigen::VectorXd r2p;
    int r2 = -1;
    ASSERT_TRUE(JointUpdater::projection(dop2.H, dop2.nu, dop2.R, tcols, &H2p, &r2p, &r2));
    EXPECT_EQ(r2, 2);
    EXPECT_EQ(H2p.rows(), 0);
    CoreState c12 = core0, c13 = core0;
    AugBlock a12 = aug0, a13 = aug0;
    upd.update(c12, a12, factoryOf(core0, aug0.x(), {psr, dop2}), proj);
    upd.update(c13, a13, factoryOf(core0, aug0.x(), {psr}), plain);
    EXPECT_TRUE(sameBytes(snapshotBytes(c12, a12), snapshotBytes(c13, a13)));
    CoreState c14 = core0;
    AugBlock a14 = aug0;
    const JointUpdater::Result r14 = upd.update(c14, a14, factoryOf(core0, aug0.x(), {dop2}), proj);
    EXPECT_FALSE(r14.committed);
    EXPECT_FALSE(r14.rolled_back);
    EXPECT_EQ(r14.iterations, 0);
    EXPECT_TRUE(sameBytes(snapshotBytes(c14, a14), snapshotBytes(core0, aug0)));
  }
  // ---- (j) R not positive definite ----
  {
    Eigen::MatrixXd Rbad = Rd;
    Rbad(3, 3) = -0.001;
    const Block dopbad{MeasClass::Dopp, Hd, Rbad, nud};
    CoreState c15 = core0;
    AugBlock a15 = aug0;
    const JointUpdater::Result r15 = upd.update(c15, a15, factoryOf(core0, aug0.x(), {psr, dopbad}), proj);
    EXPECT_TRUE(r15.rolled_back);
    EXPECT_EQ(r15.rollback_reason, "proj_failed");
    EXPECT_TRUE(sameBytes(snapshotBytes(c15, a15), snapshotBytes(core0, aug0)));
  }
  // ---- (k) forced three iterations land on the single step ----
  {
    JointUpdater::Options p3 = proj;
    p3.force_three_iterations = true;
    CoreState c16 = core0;
    AugBlock a16 = aug0;
    const JointUpdater::Result r16 = upd.update(c16, a16, factoryOf(core0, aug0.x(), {psr, dop}), p3);
    ASSERT_TRUE(r16.committed);
    EXPECT_EQ(r16.iterations, 3);
    EXPECT_LT((a16.x() - aug.x()).norm(), 1e-12 * (aug.x() - aug0.x()).norm() + 1e-15);
    EXPECT_LT((c16.boxminus(core0) - core.boxminus(core0)).norm(), 1e-10 * dx_upd.head<HostIdx::dim>().norm());
  }
}

// the σ_ψ / Δψ iteration rule triggers and the iterates converge on a yaw-leverage problem
TEST(U10Updater, ConditionalIterationConverges) {
  // range rows to far beacons from an antenna 100 m from the pivot: a^N = R_z(ψ)(y − o) + t
  const Eigen::Vector3d y_rel(100.0, 0.0, 0.0);
  std::vector<Eigen::Vector3d> beacons;
  for (int i = 0; i < 6; ++i) {
    const double el = (20.0 + 10.0 * i) * kDeg, az = (37.0 + 61.0 * i) * kDeg;
    beacons.emplace_back(2.0e7 * std::cos(el) * std::sin(az), 2.0e7 * std::cos(el) * std::cos(az), 2.0e7 * std::sin(el));
  }
  auto runCase = [&](double sigma_psi_deg, double psi_true_deg, int max_iter, bool force3, std::vector<double>* psi_calls,
                     double* psi_out) {
    Config cfg;
    cfg.iterate.max_iter = max_iter;
    CoreState core;
    core.P = HostMat::Identity() * 1e-8;
    AugBlock aug(cfg);
    Eigen::MatrixXd F1 = Eigen::MatrixXd::Zero(1, HostIdx::dim);
    Eigen::VectorXd psi0(1);
    psi0(0) = 0.0;
    Eigen::MatrixXd Npsi(1, 1);
    Npsi(0, 0) = (sigma_psi_deg * kDeg) * (sigma_psi_deg * kDeg);
    aug.addSlot(Slot::Yaw, 0, "yaw", psi0, F1, Eigen::MatrixXd(1, 0), Npsi, core.P);
    Eigen::MatrixXd F3 = Eigen::MatrixXd::Zero(3, HostIdx::dim);
    aug.addSlot(Slot::Trans, 0, "trans", Eigen::Vector3d::Zero(), F3, Eigen::MatrixXd::Zero(3, 1), Eigen::Matrix3d::Identity() * 1e-4, core.P);
    const int yaw_off = aug.find(Slot::Yaw), trans_off = aug.find(Slot::Trans);
    const int N = HostIdx::dim + aug.dim();
    const double psi_true = psi_true_deg * kDeg;
    const Eigen::Vector3d a_true = Leveling::Rz(psi_true) * y_rel;
    psi_calls->clear();
    JointUpdater::RowFactory factory = [&](const CoreState& x, const AugBlock& a, std::vector<RowSet>* sets) {
      (void)x;
      const double psi = a.x()(yaw_off);
      const Eigen::Vector3d t = a.x().segment<3>(trans_off);
      const Eigen::Vector3d a_N = Leveling::Rz(psi) * y_rel + t;
      psi_calls->push_back(psi);
      RowSet s;
      s.cls = MeasClass::Psr;
      const Eigen::Vector3d ez(0.0, 0.0, 1.0);
      for (size_t i = 0; i < beacons.size(); ++i) {
        MeasRow r;
        r.cls = MeasClass::Psr;
        r.sat = sat(static_cast<int>(i) + 1);
        const Eigen::Vector3d u = (beacons[i] - a_N).normalized();
        r.z = (beacons[i] - a_true).norm();
        r.h = (beacons[i] - a_N).norm();
        r.nu = r.z - r.h;
        r.H = Eigen::RowVectorXd::Zero(N);
        r.H(HostIdx::dim + yaw_off) = -u.dot(ez.cross(Eigen::Vector3d(a_N - t)));
        r.H.segment<3>(HostIdx::dim + trans_off) = -u.transpose();
        r.R = 1e-4;
        s.rows.push_back(r);
      }
      sets->push_back(s);
    };
    JointUpdater upd(cfg);
    JointUpdater::Options opt;
    opt.disable_l4_l5 = true;
    opt.force_three_iterations = force3;
    const JointUpdater::Result res = upd.update(core, aug, factory, opt);
    EXPECT_TRUE(res.committed);
    *psi_out = aug.x()(yaw_off);
    return res.iterations;
  };
  std::vector<double> calls;
  double psi3 = 0.0, psi1 = 0.0, psi_small = 0.0, psi_forced = 0.0;
  // σ_ψ = 5° > 2° and a 3° truth: three linearisations, errors shrinking, final error ≪ single step
  EXPECT_EQ(runCase(5.0, 3.0, 3, false, &calls, &psi3), 3);
  ASSERT_EQ(calls.size(), 3u);
  const double e0 = std::fabs(calls[0] - 3.0 * kDeg), e1 = std::fabs(calls[1] - 3.0 * kDeg), e2 = std::fabs(calls[2] - 3.0 * kDeg);
  EXPECT_LT(e1, e0);
  EXPECT_LT(e2, e1);
  EXPECT_LT(std::fabs(psi3 - 3.0 * kDeg), 1e-5);
  EXPECT_EQ(runCase(5.0, 3.0, 1, false, &calls, &psi1), 1);
  EXPECT_GT(std::fabs(psi1 - 3.0 * kDeg), std::fabs(psi3 - 3.0 * kDeg) * 10.0);
  // σ_ψ = 1° and a 0.05° truth: the rule does not trigger (single step)
  EXPECT_EQ(runCase(1.0, 0.05, 3, false, &calls, &psi_small), 1);
  EXPECT_LT(std::fabs(psi_small - 0.05 * kDeg), 1e-5);
  // reacquisition window: forced three iterations regardless — also on a problem that converges in one step
  // (the 1e-12 early exit is for the "at most 3" rule only, §4.8-3; review n1)
  EXPECT_EQ(runCase(1.0, 0.05, 3, true, &calls, &psi_forced), 3);
  EXPECT_EQ(calls.size(), 3u);
  EXPECT_EQ(runCase(1.0, 0.0, 3, true, &calls, &psi_forced), 3);
  EXPECT_EQ(calls.size(), 3u);
}
