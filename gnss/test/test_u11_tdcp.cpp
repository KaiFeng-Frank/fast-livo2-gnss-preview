// SPDX-License-Identifier: BSD-3-Clause
// test_u11_tdcp.cpp — TEST_PLAN §1 U11: the 3-D stochastic clone (DESIGN §4.5, §6) on a linear toy problem.
//   * two-epoch toy: clone + propagate + TDCP update == batch smoother over (p_k0, p_k1);
//   * 1000-run Monte-Carlo: the clone filter and an independently written full delayed-state KF agree, and
//     the mean NEES of the position error is within ±10 % of its dimension (3).
// Runs against WP-b's AugBlock (addSlot/propagate/removeSlot/jointCov/setFromJoint).
#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <random>
#include <stdexcept>
#include <string>

#include <Eigen/Dense>

#include "internal/rows.hpp"
#include "internal/aug_block.hpp"
#include "internal/update.hpp"

using namespace livo_gnss;

namespace {

SatId sat(int i) {
  SatId s;
  s.sys = Sys::GPS;
  s.prn = i;
  s.rtk_sat = i;
  return s;
}

Eigen::MatrixXd cloneFhost() {
  Eigen::MatrixXd F = Eigen::MatrixXd::Zero(3, HostIdx::dim);
  F.block<3, 3>(0, HostIdx::pos) = Eigen::Matrix3d::Identity();
  return F;
}

std::vector<Eigen::Vector3d> losSet(int n, std::mt19937_64& rng) {
  std::uniform_real_distribution<double> u(0.0, 1.0);
  std::vector<Eigen::Vector3d> v;
  for (int i = 0; i < n; ++i) {
    const double el = (15.0 + 65.0 * u(rng)) * M_PI / 180.0, az = 2.0 * M_PI * u(rng);
    v.emplace_back(std::cos(el) * std::sin(az), std::cos(el) * std::cos(az), std::sin(el));
  }
  return v;
}

// TDCP-like rows: z = −u·(p1 − c) + Δclk + n ; H = [−u at pos | +u at clone]; class Tdcp so update() projects Δclk.
JointUpdater::RowFactory tdcpFactory(const std::vector<Eigen::Vector3d>& u, const Eigen::VectorXd& z, double var, int clone_off,
                                     int N) {
  return [&, var, clone_off, N](const CoreState& x, const AugBlock& a, std::vector<RowSet>* sets) {
    RowSet s;
    s.cls = MeasClass::Tdcp;
    const Eigen::Vector3d c = a.x().segment<3>(clone_off);
    for (size_t j = 0; j < u.size(); ++j) {
      MeasRow r;
      r.cls = MeasClass::Tdcp;
      r.sat = sat(static_cast<int>(j) + 1);
      r.pair_id = 1;
      r.z = z(static_cast<int>(j));
      r.h = -u[j].dot(x.p - c);
      r.nu = r.z - r.h;
      r.H = Eigen::RowVectorXd::Zero(N);
      r.H.segment<3>(HostIdx::pos) = -u[j].transpose();
      r.H.segment<3>(HostIdx::dim + clone_off) = u[j].transpose();
      r.R = var;
      s.rows.push_back(r);
    }
    sets->push_back(s);
  };
}

// Reference: batch/joint Gaussian over X = [p1; p0] with single-difference elimination of Δclk.
struct JointRef {
  Eigen::VectorXd X;     // 6
  Eigen::MatrixXd P;     // 6×6
};
JointRef jointUpdate(const JointRef& prior, const std::vector<Eigen::Vector3d>& u, const Eigen::VectorXd& z, double var) {
  const int m = static_cast<int>(u.size());
  Eigen::MatrixXd A(m, 6);
  for (int j = 0; j < m; ++j) {
    A.row(j).head<3>() = -u[j].transpose();
    A.row(j).tail<3>() = u[j].transpose();
  }
  Eigen::MatrixXd D = Eigen::MatrixXd::Zero(m - 1, m);
  for (int j = 0; j < m - 1; ++j) {
    D(j, 0) = -1.0;
    D(j, j + 1) = 1.0;
  }
  const Eigen::MatrixXd Ad = D * A;
  const Eigen::VectorXd zd = D * z;
  const Eigen::MatrixXd Rd = var * D * D.transpose();
  const Eigen::MatrixXd S = Ad * prior.P * Ad.transpose() + Rd;
  const Eigen::MatrixXd K = prior.P * Ad.transpose() * S.inverse();
  JointRef post;
  post.X = prior.X + K * (zd - Ad * prior.X);
  const Eigen::MatrixXd IKH = Eigen::MatrixXd::Identity(6, 6) - K * Ad;
  post.P = IKH * prior.P * IKH.transpose() + K * Rd * K.transpose();
  return post;
}

}  // namespace

// two-epoch linear toy problem equals the batch smoother
TEST(U11Tdcp, TwoEpochLinearToyEqualsBatchSmoother) {
  Config cfg;
  std::mt19937_64 rng(31);
  std::normal_distribution<double> g(0.0, 1.0);
  CoreState core;
  core.p = Eigen::Vector3d(1.0, 2.0, 3.0);
  core.P = HostMat::Identity() * 1e-6;
  Eigen::Matrix3d P0;
  P0 << 4e-4, 1e-4, 0.0, 1e-4, 3e-4, 5e-5, 0.0, 5e-5, 9e-4;
  core.P.block<3, 3>(HostIdx::pos, HostIdx::pos) = P0;
  const Eigen::Matrix3d Qw = Eigen::Matrix3d::Identity() * 0.01;
  const Eigen::Matrix3d SigmaD = Eigen::Matrix3d::Identity() * 1e-8;
  // truth
  const Eigen::Vector3d p0_true = core.p + Eigen::Vector3d(0.01, -0.005, 0.02);
  const Eigen::Vector3d c_true = p0_true + Eigen::Vector3d(1e-4, -1e-4, 5e-5);
  const Eigen::Vector3d p1_true = p0_true + Eigen::Vector3d(0.08, -0.05, 0.12);
  // clone at k0 (consistent augmentation), then propagate the host prior
  AugBlock aug(cfg);
  const int slot = aug.addSlot(Slot::Clone, 1, "clone_1", core.p, cloneFhost(), Eigen::MatrixXd(3, 0), SigmaD, core.P);
  const int clone_off = aug.find(Slot::Clone, 1);
  ASSERT_GE(clone_off, 0);
  ASSERT_EQ(aug.slots()[slot].dim, 3);
  core.P.block<3, 3>(HostIdx::pos, HostIdx::pos) += Qw;
  aug.propagate(HostMat::Identity(), Eigen::MatrixXd::Identity(3, 3), Eigen::MatrixXd::Zero(3, 3));
  const int N = HostIdx::dim + aug.dim();
  // rows
  const auto u = losSet(5, rng);
  const double var = 25e-6, dclk = 0.7;
  Eigen::VectorXd z(5);
  for (int j = 0; j < 5; ++j) z(j) = -u[j].dot(p1_true - c_true) + dclk + std::sqrt(var) * g(rng);
  JointUpdater upd(cfg);
  JointUpdater::Options opt;
  opt.disable_l4_l5 = true;
  const JointUpdater::Result res = upd.update(core, aug, tdcpFactory(u, z, var, clone_off, N), opt);
  ASSERT_TRUE(res.committed);
  // reference joint Gaussian over (p1, c): prior mean (p̂0, p̂0), cov [[P0+Qw, P0], [P0, P0+Σ_D]]
  JointRef prior;
  prior.X.resize(6);
  prior.X << Eigen::Vector3d(1.0, 2.0, 3.0), Eigen::Vector3d(1.0, 2.0, 3.0);
  prior.P.resize(6, 6);
  prior.P.topLeftCorner<3, 3>() = P0 + Qw;
  prior.P.topRightCorner<3, 3>() = P0;
  prior.P.bottomLeftCorner<3, 3>() = P0;
  prior.P.bottomRightCorner<3, 3>() = P0 + SigmaD;
  const JointRef post = jointUpdate(prior, u, z, var);
  EXPECT_LT((core.p - post.X.head<3>()).norm(), 1e-9);
  EXPECT_LT((aug.x().segment<3>(clone_off) - post.X.tail<3>()).norm(), 1e-9);
  const Eigen::Matrix3d Ppp = core.P.block<3, 3>(HostIdx::pos, HostIdx::pos);
  EXPECT_LT(((Ppp - post.P.topLeftCorner<3, 3>()).norm() / post.P.topLeftCorner<3, 3>().norm()), 1e-9);
  const Eigen::Matrix3d Qcc = aug.Q().block<3, 3>(clone_off, clone_off);
  EXPECT_LT(((Qcc - post.P.bottomRightCorner<3, 3>()).norm() / post.P.bottomRightCorner<3, 3>().norm()), 1e-9);
  const Eigen::Matrix3d Cpc = aug.C().block<3, 3>(HostIdx::pos, clone_off);
  EXPECT_LT(((Cpc - post.P.topRightCorner<3, 3>()).norm() / post.P.topRightCorner<3, 3>().norm()), 1e-9);
  // deleting the clone leaves the host posterior bit-identical
  const HostMat P_before = core.P;
  aug.removeSlot(slot);
  EXPECT_EQ(aug.dim(), 0);
  EXPECT_EQ(std::memcmp(P_before.data(), core.P.data(), sizeof(double) * HostIdx::dim * HostIdx::dim), 0);
}

// clone version vs full delayed-state filter, 1000 MC runs, mean NEES within ±10 % of the dimension
TEST(U11Tdcp, CloneVsFullDelayedState1000McNees) {
  Config cfg;
  std::mt19937_64 rng(1000);
  std::normal_distribution<double> g(0.0, 1.0);
  const int runs = 1000, pairs = 4, m = 5;
  const Eigen::Matrix3d P0 = Eigen::Matrix3d::Identity() * 2.5e-3;
  const Eigen::Matrix3d Qw = Eigen::Matrix3d::Identity() * 1e-2;
  const Eigen::Matrix3d SigmaD = Eigen::Matrix3d::Identity() * 1e-6;
  const double var = 25e-6;
  double nees_clone = 0.0, nees_ref = 0.0, max_diff = 0.0;
  int count = 0;
  JointUpdater upd(cfg);
  JointUpdater::Options opt;
  opt.disable_l4_l5 = true;
  for (int r = 0; r < runs; ++r) {
    CoreState core;
    core.p = Eigen::Vector3d(0.0, 0.0, 0.0);
    core.P = HostMat::Identity() * 1e-9;
    core.P.block<3, 3>(HostIdx::pos, HostIdx::pos) = P0;
    Eigen::Vector3d p_true = core.p + P0.llt().matrixL() * Eigen::Vector3d(g(rng), g(rng), g(rng));
    AugBlock aug(cfg);
    // reference delayed-state filter: X = [p1; c]
    JointRef ref;
    for (int k = 0; k < pairs; ++k) {
      // clone (bridge noise Σ_D on the clone value)
      const Eigen::Vector3d c_true = p_true + SigmaD.llt().matrixL() * Eigen::Vector3d(g(rng), g(rng), g(rng));
      const int slot = aug.addSlot(Slot::Clone, k + 1, "clone", core.p, cloneFhost(), Eigen::MatrixXd(3, aug.dim()), SigmaD, core.P);
      const int clone_off = aug.find(Slot::Clone, k + 1);
      ASSERT_GE(clone_off, 0);
      ref.X.resize(6);
      ref.X << core.p, core.p;
      ref.P.resize(6, 6);
      const Eigen::Matrix3d Pp = core.P.block<3, 3>(HostIdx::pos, HostIdx::pos);
      ref.P.topLeftCorner<3, 3>() = Pp;
      ref.P.topRightCorner<3, 3>() = Pp;
      ref.P.bottomLeftCorner<3, 3>() = Pp;
      ref.P.bottomRightCorner<3, 3>() = Pp + SigmaD;
      // propagate (random walk of the truth, prior mean unchanged)
      p_true += Qw.llt().matrixL() * Eigen::Vector3d(g(rng), g(rng), g(rng));
      core.P.block<3, 3>(HostIdx::pos, HostIdx::pos) += Qw;
      aug.propagate(HostMat::Identity(), Eigen::MatrixXd::Identity(aug.dim(), aug.dim()), Eigen::MatrixXd::Zero(aug.dim(), aug.dim()));
      ref.P.topLeftCorner<3, 3>() += Qw;
      // TDCP pair
      const auto u = losSet(m, rng);
      const double dclk = 3.0 * g(rng);
      Eigen::VectorXd z(m);
      for (int j = 0; j < m; ++j) z(j) = -u[j].dot(p_true - c_true) + dclk + std::sqrt(var) * g(rng);
      const int N = HostIdx::dim + aug.dim();
      const JointUpdater::Result res = upd.update(core, aug, tdcpFactory(u, z, var, clone_off, N), opt);
      ASSERT_TRUE(res.committed);
      ref = jointUpdate(ref, u, z, var);
      // compare and score
      max_diff = std::max(max_diff, (core.p - ref.X.head<3>()).norm());
      const Eigen::Vector3d e = p_true - core.p;
      const Eigen::Matrix3d Pc = core.P.block<3, 3>(HostIdx::pos, HostIdx::pos);
      nees_clone += e.dot(Pc.ldlt().solve(e));
      const Eigen::Vector3d er = p_true - ref.X.head<3>();
      const Eigen::Matrix3d Pr = ref.P.topLeftCorner<3, 3>();
      nees_ref += er.dot(Pr.ldlt().solve(er));
      ++count;
      aug.removeSlot(slot);
      ASSERT_EQ(aug.dim(), 0);
    }
  }
  nees_clone /= count;
  nees_ref /= count;
  EXPECT_LT(max_diff, 1e-8) << "clone filter vs delayed-state reference";
  EXPECT_GT(nees_clone, 2.7) << "mean NEES " << nees_clone;
  EXPECT_LT(nees_clone, 3.3) << "mean NEES " << nees_clone;
  EXPECT_GT(nees_ref, 2.7);
  EXPECT_LT(nees_ref, 3.3);
}
