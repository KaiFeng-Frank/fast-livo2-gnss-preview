// SPDX-License-Identifier: BSD-3-Clause
// test_u08_hooks.cpp — TEST_PLAN §1 U8: the split hooks (host 19-dim filter + AugBlock H1/H2) against an explicit
// (19+n)-dim joint filter (DESIGN §2.4; D3 §3.4 experiment 2; R1 "K_G = P_GH (P⁻)⁻¹ K_H, independent of H_i").
// Owner: WP-b. Relative difference < 1e-12 on C, Q_G, x_G; Σ stays PSD.
//
// Host update forms mirrored here:
//   LIO-like  voxel_map.cpp:468-490  K_1 = (HᵀR⁻¹H + P⁻¹)⁻¹, K = K_1 HᵀR⁻¹, G = K H (6 columns), P⁺ = (I − G) P⁻
//   VIO-like  vio.cpp:1655-1667/800  K_1 = (HᵀH + (P/σ²)⁻¹)⁻¹, K = K_1 Hᵀ, G = K H (7 columns), several layers,
//             P⁺ = P⁻ − G P⁻ with the G of the last layer, whose iterate is not a fixed point
// Both iterate in the error-state form x_{i+1} = x⁻ ⊞ K_i[ν_i + H_i(x_i ⊟ x⁻)] (DESIGN §4.8 step 2), in which the joint
// filter's augmented increment is exactly A·(x⁺ ⊟ x⁻). The upstream code writes the same iteration as
// x_i ⊞ (K ν + vec − G vec), vec = x⁻ ⊟ x_i, which differs from it by second-order terms of the SO(3) retraction
// (BCH: ½[x⁻ ⊟ x_i, δ]) — the same order as the IEKF's own linearisation error; the last case quantifies it.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <random>
#include <string>
#include <vector>

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <Eigen/LU>
#include <Eigen/QR>

#include "internal/aug_block.hpp"
#include "internal/leveling.hpp"

using namespace livo_gnss;

namespace {

constexpr int kH = HostIdx::dim;

std::mt19937& rng() {
  static std::mt19937 g(808);
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
// SPD with condition ≤ 10 so that the information-form (host) and covariance-form (joint) gains agree to ~1e-14.
Eigen::MatrixXd randomSpd(int n, double scale = 1.0) {
  Eigen::HouseholderQR<Eigen::MatrixXd> qr(gaussian(n, n));
  Eigen::MatrixXd U = qr.householderQ();
  Eigen::VectorXd lam(n);
  std::uniform_real_distribution<double> ud(-1.0, 0.0);
  for (int i = 0; i < n; ++i) lam(i) = scale * std::pow(10.0, ud(rng()));
  Eigen::MatrixXd S = U * lam.asDiagonal() * U.transpose();
  return 0.5 * (S + S.transpose()).eval();
}
Eigen::MatrixXd sym(const Eigen::MatrixXd& S) { return 0.5 * (S + S.transpose()).eval(); }
double relDiff(const Eigen::MatrixXd& a, const Eigen::MatrixXd& b) {
  const double s = std::max(a.cwiseAbs().maxCoeff(), b.cwiseAbs().maxCoeff());
  return (a - b).cwiseAbs().maxCoeff() / (s > 0.0 ? s : 1.0);
}
double minEigRatio(const Eigen::MatrixXd& S) {
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(sym(S));
  return es.eigenvalues().minCoeff() / es.eigenvalues().cwiseAbs().maxCoeff();
}
CoreState randomCore() {
  CoreState x;
  x.R = so3Exp(Eigen::Vector3d(0.4 * gauss(), 0.4 * gauss(), 2.0 * gauss()));
  x.p = 5.0 * Eigen::Vector3d(gauss(), gauss(), gauss());
  x.v = Eigen::Vector3d(gauss(), gauss(), gauss());
  x.bg = 1e-3 * Eigen::Vector3d(gauss(), gauss(), gauss());
  x.ba = 1e-2 * Eigen::Vector3d(gauss(), gauss(), gauss());
  x.g = Eigen::Vector3d(0.08, -0.15, -9.79);
  x.inv_expo = 1.0 + 0.1 * gauss();
  return x;
}

// Joint state (host ⊞ on the manifold, additive augmented block).
struct JointState {
  CoreState host;
  Eigen::VectorXd xG;
  JointState boxplus(const Eigen::VectorXd& d) const {
    JointState y;
    y.host = host.boxplus(d.head<kH>());
    y.xG = xG + d.tail(xG.size());
    return y;
  }
  Eigen::VectorXd boxminus(const JointState& o) const {
    Eigen::VectorXd d(kH + xG.size());
    d.head<kH>() = host.boxminus(o.host);
    d.tail(xG.size()) = xG - o.xG;
    return d;
  }
};

// Synthetic host-only measurement: h(x) = H_c · (x ⊟ x_ref).head(c), c ∈ {6, 7}; linear in the error state about
// x_ref, mildly nonlinear in x through ⊟ on SO(3).
struct Meas {
  Eigen::MatrixXd Hc;   // m × c
  CoreState x_ref;
  Eigen::VectorXd z;
  Eigen::MatrixXd R;    // m × m
  int c() const { return static_cast<int>(Hc.cols()); }
  Eigen::VectorXd innov(const CoreState& x) const { return z - Hc * x.boxminus(x_ref).head(c()); }
  Eigen::MatrixXd Hfull(int n_total) const {
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(Hc.rows(), n_total);
    H.leftCols(c()) = Hc;
    return H;
  }
};
Meas makeMeas(int m, int c, const CoreState& x_prior, double z_scale, double r_scale) {
  Meas ms;
  ms.Hc = gaussian(m, c);
  HostVec d = HostVec::Zero();
  for (int i = 0; i < kH; ++i) d(i) = 0.05 * gauss();
  ms.x_ref = x_prior.boxplus(d);
  ms.z = z_scale * gaussian(m, 1);
  ms.R = Eigen::MatrixXd::Zero(m, m);
  for (int i = 0; i < m; ++i) ms.R(i, i) = r_scale * (0.5 + std::abs(gauss()));
  return ms;
}

// Host gain in the upstream information form (LIO: general R; VIO: R = σ²I gives the same expression).
Eigen::MatrixXd hostGainInfoForm(const HostMat& P, const Meas& ms) {
  const int c = ms.c();
  const Eigen::MatrixXd Rinv = ms.R.inverse();
  HostMat HTH = HostMat::Zero();
  HTH.topLeftCorner(c, c) = ms.Hc.transpose() * Rinv * ms.Hc;
  const HostMat K1 = (HTH + P.inverse()).inverse();
  return K1.leftCols(c) * ms.Hc.transpose() * Rinv;   // 19 × m
}
// Joint gain in covariance form (never inverts Σ).
Eigen::MatrixXd jointGain(const Eigen::MatrixXd& Sigma, const Eigen::MatrixXd& Hfull, const Eigen::MatrixXd& R) {
  const Eigen::MatrixXd S = Hfull * Sigma * Hfull.transpose() + R;
  Eigen::LDLT<Eigen::MatrixXd> ldlt(sym(S));
  return ldlt.solve(Hfull * Sigma).transpose();   // Σ Hᵀ S⁻¹
}

struct Rig {
  Config cfg;
  AugBlock aug;
  HostMat P;
  CoreState x;
  int n = 0;
  Rig() : aug(cfg) {
    const Eigen::MatrixXd none;
    auto zeroF = [](int d) { return Eigen::MatrixXd(Eigen::MatrixXd::Zero(d, kH)); };
    const HostMat P0 = HostMat::Identity();
    aug.addSlot(Slot::Tilt, 0, "tilt", Eigen::Vector2d(1e-4, -2e-4), zeroF(2), none, Eigen::MatrixXd::Identity(2, 2), P0);
    aug.addSlot(Slot::Yaw, 0, "yaw", Eigen::VectorXd::Constant(1, 0.7), zeroF(1), none, Eigen::MatrixXd::Identity(1, 1), P0);
    aug.addSlot(Slot::Trans, 0, "trans", Eigen::Vector3d(10.0, -5.0, 2.0), zeroF(3), none, Eigen::MatrixXd::Identity(3, 3), P0);
    aug.addSlot(Slot::Clk, 0, "clk", Eigen::Vector2d(120.0, 0.4), zeroF(2), none, Eigen::MatrixXd::Identity(2, 2), P0);
    aug.addSlot(Slot::Isb, static_cast<int>(sysBit(Sys::BDS)), "isb_C", Eigen::VectorXd::Constant(1, 3.0), zeroF(1), none,
                Eigen::MatrixXd::Identity(1, 1), P0);
    aug.addSlot(Slot::Clone, 3, "clone_3", Eigen::Vector3d(1.0, 2.0, 3.0), zeroF(3), none, Eigen::MatrixXd::Identity(3, 3), P0);
    n = aug.dim();
    const Eigen::MatrixXd S = randomSpd(kH + n);
    aug.setFromJoint(S, &P);
    x = randomCore();
  }
  JointState joint() const { return JointState{x, aug.x()}; }
  void compare(const Eigen::MatrixXd& Sigma_ref, const JointState& js_ref, double tol, const char* what) const {
    const Eigen::MatrixXd Sigma = aug.jointCov(P);
    EXPECT_LT(relDiff(Sigma.topLeftCorner(kH, kH), Sigma_ref.topLeftCorner(kH, kH)), tol) << what << ": P";
    EXPECT_LT(relDiff(aug.C(), Sigma_ref.topRightCorner(kH, n)), tol) << what << ": C";
    EXPECT_LT(relDiff(aug.Q(), sym(Sigma_ref.bottomRightCorner(n, n))), tol) << what << ": Q_G";
    EXPECT_LT(relDiff(aug.x(), js_ref.xG), tol) << what << ": x_G";
    EXPECT_LT(x.boxminus(js_ref.host).cwiseAbs().maxCoeff(), tol) << what << ": host mean";
    EXPECT_GE(minEigRatio(Sigma), -1e-12) << what << ": PSD";
  }
};

// One propagation cycle on both sides. Returns nothing; updates s (host P + hook) and (Sigma, js) (joint).
void propagateBoth(Rig& s, Eigen::MatrixXd& Sigma, JointState& js, int steps) {
  HostMat F_tot = HostMat::Identity();
  for (int k = 0; k < steps; ++k) {
    // IMU-like transition: rotation/velocity/position couplings of IMU_Processing.cpp:382-392 with random inputs
    const double dt = 0.005;
    HostMat F = HostMat::Identity();
    const Eigen::Vector3d w(0.3 * gauss(), 0.3 * gauss(), 0.5 * gauss()), f(0.5 * gauss(), 0.5 * gauss(), 9.8);
    F.block<3, 3>(0, 0) = so3Exp(Eigen::Vector3d(-w * dt));
    F.block<3, 3>(0, 10) = -Eigen::Matrix3d::Identity() * dt;
    F.block<3, 3>(3, 7) = Eigen::Matrix3d::Identity() * dt;
    F.block<3, 3>(7, 0) = -s.x.R * skew(f) * dt;
    F.block<3, 3>(7, 13) = -s.x.R * dt;
    F.block<3, 3>(7, 16) = Eigen::Matrix3d::Identity() * dt;
    HostMat Qw = HostMat::Zero();
    Qw.block<3, 3>(0, 0) = 1e-4 * dt * dt * Eigen::Matrix3d::Identity();
    Qw.block<3, 3>(7, 7) = 1e-2 * dt * dt * Eigen::Matrix3d::Identity();
    Qw.block<3, 3>(10, 10) = 1e-6 * dt * dt * Eigen::Matrix3d::Identity();
    Qw.block<3, 3>(13, 13) = 1e-5 * dt * dt * Eigen::Matrix3d::Identity();
    Qw(6, 6) = 1e-4 * dt * dt;
    s.P = F * s.P * F.transpose() + Qw;
    F_tot = F * F_tot;
    Eigen::MatrixXd Fj = Eigen::MatrixXd::Identity(kH + s.n, kH + s.n);
    Fj.topLeftCorner(kH, kH) = F;
    Eigen::MatrixXd Qj = Eigen::MatrixXd::Zero(kH + s.n, kH + s.n);
    Qj.topLeftCorner(kH, kH) = Qw;
    Sigma = Fj * Sigma * Fj.transpose() + Qj;
  }
  Leveling lev;
  lev.setRef(s.x.g);
  const Eigen::Matrix3d M = lev.M(s.aug.yaw(), s.aug.tilt(), s.x.g);
  Eigen::MatrixXd Phi, Qp;
  s.aug.processModel(1.3, 0.2, steps * 0.005, M, s.x.p + Eigen::Vector3d(0.3, 0.1, 0.4), Eigen::Vector3d(2.0, 1.0, 0.0), 0.19, 0.2, &Phi, &Qp);
  s.aug.propagate(F_tot, Phi, Qp);
  Eigen::MatrixXd Pj = Eigen::MatrixXd::Identity(kH + s.n, kH + s.n);
  Pj.bottomRightCorner(s.n, s.n) = Phi;
  Eigen::MatrixXd Qj = Eigen::MatrixXd::Zero(kH + s.n, kH + s.n);
  Qj.bottomRightCorner(s.n, s.n) = Qp;
  Sigma = Pj * Sigma * Pj.transpose() + Qj;
  js.xG = Phi * js.xG;
}

// Host iteration (error-state form) with the given per-iteration measurements; returns P⁺ = (I − G_L) P⁻ (LIO form) or
// P⁻ − G_L P⁻ (VIO form: identical algebra, kept separate to mirror the two code sites).
struct HostResult {
  CoreState x_post;
  HostMat P_post;
};
HostResult hostIterate(const CoreState& x_prior, const HostMat& P_prior, const std::vector<Meas>& layers, bool vio_form) {
  CoreState xi = x_prior;
  HostMat G = HostMat::Zero();
  for (const Meas& ms : layers) {
    const Eigen::MatrixXd K = hostGainInfoForm(P_prior, ms);
    const Eigen::VectorXd nu = ms.innov(xi);
    const HostVec dxi = xi.boxminus(x_prior);
    const Eigen::VectorXd delta = K * (nu + ms.Hc * dxi.head(ms.c()));
    xi = x_prior.boxplus(delta);
    G.setZero();
    G.leftCols(ms.c()) = K * ms.Hc;
  }
  HostResult r;
  r.x_post = xi;
  r.P_post = vio_form ? HostMat(P_prior - G * P_prior) : HostMat((HostMat::Identity() - G) * P_prior);
  return r;
}
struct JointResult {
  JointState js;
  Eigen::MatrixXd Sigma;
};
JointResult jointIterate(const JointState& prior, const Eigen::MatrixXd& Sigma_prior, const std::vector<Meas>& layers) {
  const int N = static_cast<int>(Sigma_prior.rows());
  JointState ji = prior;
  Eigen::MatrixXd KH = Eigen::MatrixXd::Zero(N, N);
  for (const Meas& ms : layers) {
    const Eigen::MatrixXd H = ms.Hfull(N);
    const Eigen::MatrixXd K = jointGain(Sigma_prior, H, ms.R);
    const Eigen::VectorXd nu = ms.innov(ji.host);
    const Eigen::VectorXd delta = K * (nu + H * ji.boxminus(prior));
    ji = prior.boxplus(delta);
    KH = K * H;
  }
  JointResult r;
  r.js = ji;
  r.Sigma = Sigma_prior - KH * Sigma_prior;
  return r;
}

void updateBoth(Rig& s, Eigen::MatrixXd& Sigma, JointState& js, const std::vector<Meas>& layers, bool vio_form, bool* pinv) {
  const HostMat P_prior = s.P;
  const CoreState x_prior = s.x;
  const HostResult h = hostIterate(x_prior, P_prior, layers, vio_form);
  const JointResult j = jointIterate(js, Sigma, layers);
  s.x = h.x_post;
  s.P = h.P_post;
  ASSERT_TRUE(s.aug.conditionOnHostUpdate(P_prior, h.P_post, h.x_post.boxminus(x_prior), pinv));
  Sigma = j.Sigma;
  js = j.js;
  // the hook wraps ψ into (−π, π] (H2 ends with wrapYaw()); do the same on the reference
  const int iy = s.aug.find(Slot::Yaw);
  double& psi = js.xG(iy);
  psi -= 2.0 * M_PI * std::floor((psi + M_PI) / (2.0 * M_PI));
  if (psi <= -M_PI) psi += 2.0 * M_PI;
}

}  // namespace

// H1 split propagation equals the (19+n)-dim joint filter, rel. diff < 1e-12
TEST(U08Hooks, PropagationEqualsJoint) {
  Rig s;
  Eigen::MatrixXd Sigma = s.aug.jointCov(s.P);
  JointState js = s.joint();
  propagateBoth(s, Sigma, js, 20);
  s.compare(Sigma, js, 1e-12, "propagation");
  // a second cycle on top (C is no longer block-structured)
  propagateBoth(s, Sigma, js, 20);
  s.compare(Sigma, js, 1e-12, "propagation ×2");
  // the clock mean moved by d·Δt, the registration mean did not
  EXPECT_NEAR(s.aug.clockBias(), 120.0 + 2 * 0.1 * 0.4, 1e-12);
  EXPECT_EQ(s.aug.yaw(), 0.7);
}

// H2 after a LIO-like update (6 columns, (I−G)P⁻) equals the joint filter
TEST(U08Hooks, LioLikeUpdate6Columns) {
  Rig s;
  Eigen::MatrixXd Sigma = s.aug.jointCov(s.P);
  JointState js = s.joint();
  propagateBoth(s, Sigma, js, 10);
  // one LIO update: 12 point-to-plane rows on the first 6 columns, 3 iterations at the same H (voxel_map re-linearises
  // H at each iterate; the hook does not care which), the last one not iterated to convergence
  std::vector<Meas> layers;
  const Meas m0 = makeMeas(12, 6, s.x, 0.3, 0.05);
  for (int i = 0; i < 3; ++i) layers.push_back(m0);
  bool pinv = true;
  const CoreState x_before = s.x;
  const Eigen::VectorXd xG_before = s.aug.x();
  const Eigen::MatrixXd C_before = s.aug.C();
  updateBoth(s, Sigma, js, layers, false, &pinv);
  EXPECT_FALSE(pinv);
  s.compare(Sigma, js, 1e-12, "LIO-like");
  // not vacuous: the host mean, x_G and C all moved
  EXPECT_GT(s.x.boxminus(x_before).norm(), 1e-3);
  EXPECT_GT((s.aug.x() - xG_before).norm(), 1e-4);
  EXPECT_GT(relDiff(s.aug.C(), C_before), 1e-3);
  // another cycle: propagate + LIO-like again, still exact
  propagateBoth(s, Sigma, js, 10);
  layers.clear();
  layers.push_back(makeMeas(20, 6, s.x, 0.3, 0.05));
  updateBoth(s, Sigma, js, layers, false, &pinv);
  s.compare(Sigma, js, 1e-12, "LIO-like ×2");
}

// H2 after a VIO-like update (7 columns, multiple layers, final G at a non-fixed point) equals the joint filter
TEST(U08Hooks, VioLikeUpdateMultiLayerFinalG) {
  Rig s;
  Eigen::MatrixXd Sigma = s.aug.jointCov(s.P);
  JointState js = s.joint();
  propagateBoth(s, Sigma, js, 10);
  // three pyramid layers, each its own H (patch Jacobians change with the level) and residual, one iteration each;
  // the covariance is written once with the G of the last layer (vio.cpp:800) at an iterate that is not a fixed point
  std::vector<Meas> layers;
  for (int l = 0; l < 3; ++l) {
    Meas ms = makeMeas(16 + 8 * l, 7, s.x, 0.4, 0.1);
    ms.R = 0.1 * Eigen::MatrixXd::Identity(ms.z.size(), ms.z.size());   // img_point_cov · I
    layers.push_back(ms);
  }
  bool pinv = true;
  updateBoth(s, Sigma, js, layers, true, &pinv);
  EXPECT_FALSE(pinv);
  s.compare(Sigma, js, 1e-12, "VIO-like");
  // the last iterate is indeed not a fixed point: one more step at the same layer would still move the state
  {
    const Meas& last = layers.back();
    const Eigen::MatrixXd K = hostGainInfoForm(s.P, last);   // any gain; only the innovation matters
    const Eigen::VectorXd nu = last.innov(s.x);
    EXPECT_GT((K * nu).norm(), 1e-6);
  }
  // LIO-like then VIO-like in the same cycle (LIVO: handleLIO H2, handleVIO H2 with the LIO posterior as prior)
  propagateBoth(s, Sigma, js, 10);
  updateBoth(s, Sigma, js, {makeMeas(12, 6, s.x, 0.3, 0.05)}, false, &pinv);
  std::vector<Meas> vio;
  for (int l = 0; l < 2; ++l) {
    Meas ms = makeMeas(20, 7, s.x, 0.4, 0.1);
    ms.R = 0.1 * Eigen::MatrixXd::Identity(ms.z.size(), ms.z.size());
    vio.push_back(ms);
  }
  updateBoth(s, Sigma, js, vio, true, &pinv);
  s.compare(Sigma, js, 1e-12, "LIO+VIO cycle");
}

// P⁺ ≠ (I−G)P⁻ case is still exact
TEST(U08Hooks, PostNotEqualIGPprior) {
  Rig s;
  Eigen::MatrixXd Sigma = s.aug.jointCov(s.P);
  JointState js = s.joint();
  propagateBoth(s, Sigma, js, 10);
  // a deliberately sub-optimal gain (0.7·K) with the Joseph form: P⁺ is then neither (I−KH)P⁻ nor (I−G)P⁻
  const Meas ms = makeMeas(10, 6, s.x, 0.3, 0.05);
  const HostMat P_prior = s.P;
  const CoreState x_prior = s.x;
  const Eigen::MatrixXd K = 0.7 * hostGainInfoForm(P_prior, ms);
  const Eigen::VectorXd nu = ms.innov(x_prior);
  const HostVec delta = K * nu;
  const CoreState x_post = x_prior.boxplus(delta);
  HostMat IKH = HostMat::Identity();
  IKH.leftCols(6) -= K * ms.Hc;
  const HostMat P_post = IKH * P_prior * IKH.transpose() + K * ms.R * K.transpose();
  ASSERT_GT(relDiff(P_post, IKH * P_prior), 1e-6);   // the case is what its name says
  bool pinv = true;
  ASSERT_TRUE(s.aug.conditionOnHostUpdate(P_prior, P_post, x_post.boxminus(x_prior), &pinv));
  EXPECT_FALSE(pinv);
  s.x = x_post;
  s.P = P_post;
  // joint Joseph with the same scaled gain (computed from Σ alone: 0.7·Σ Hᵀ S⁻¹ = [K; A K] by construction)
  const int N = kH + s.n;
  const Eigen::MatrixXd H = ms.Hfull(N);
  const Eigen::MatrixXd Kj = 0.7 * jointGain(Sigma, H, ms.R);
  const Eigen::MatrixXd IKHj = Eigen::MatrixXd::Identity(N, N) - Kj * H;
  Sigma = IKHj * Sigma * IKHj.transpose() + Kj * ms.R * Kj.transpose();
  js = js.boxplus(Kj * nu);
  s.compare(Sigma, js, 1e-12, "Joseph, sub-optimal gain");
}

// Σ stays positive semi-definite through the hooks
TEST(U08Hooks, JointCovarianceStaysPsd) {
  Rig s;
  Eigen::MatrixXd Sigma = s.aug.jointCov(s.P);
  JointState js = s.joint();
  bool pinv = false;
  for (int cycle = 0; cycle < 6; ++cycle) {
    propagateBoth(s, Sigma, js, 8);
    EXPECT_GE(minEigRatio(s.aug.jointCov(s.P)), -1e-12) << "cycle " << cycle << " after H1";
    updateBoth(s, Sigma, js, {makeMeas(12, 6, s.x, 0.3, 0.05)}, false, &pinv);
    EXPECT_GE(minEigRatio(s.aug.jointCov(s.P)), -1e-12) << "cycle " << cycle << " after H2(LIO)";
    std::vector<Meas> vio;
    for (int l = 0; l < 2; ++l) {
      Meas ms = makeMeas(16, 7, s.x, 0.4, 0.1);
      ms.R = 0.1 * Eigen::MatrixXd::Identity(ms.z.size(), ms.z.size());
      vio.push_back(ms);
    }
    updateBoth(s, Sigma, js, vio, true, &pinv);
    EXPECT_GE(minEigRatio(s.aug.jointCov(s.P)), -1e-12) << "cycle " << cycle << " after H2(VIO)";
    EXPECT_LT((s.aug.Q() - s.aug.Q().transpose()).cwiseAbs().maxCoeff(), 0.0 + 1e-300) << "Q_G symmetrised";
    // every cycle is exact at the TEST_PLAN tolerance from the state it started in; the joint reference is then
    // re-seeded from the split state so that rounding of one cycle does not accumulate into the next comparison
    s.compare(Sigma, js, 1e-12, (std::string("cycle ") + std::to_string(cycle)).c_str());
    Sigma = s.aug.jointCov(s.P);
    js = s.joint();
  }
}

// A singular or slightly indefinite prior (zero-variance exposure state) is conditioned through the pseudo-inverse
TEST(U08Hooks, SingularPriorPseudoInverse) {
  for (int variant = 0; variant < 2; ++variant) {
    Rig s;
    // exposure state τ (index 6) with no variance and no cross-covariance: the joint stays PSD, P⁻ is singular
    s.P.row(HostIdx::expo).setZero();
    s.P.col(HostIdx::expo).setZero();
    s.P(HostIdx::expo, HostIdx::expo) = (variant == 0) ? 0.0 : -1e-30;   // 1: rounding-level indefinite
    s.aug.C().row(HostIdx::expo).setZero();
    Eigen::MatrixXd Sigma = s.aug.jointCov(s.P);
    JointState js = s.joint();
    Meas ms = makeMeas(12, 6, s.x, 0.3, 0.05);   // 6 columns never touch index 6
    // host update via the covariance-form gain (P⁻¹ does not exist): K = P Hᵀ S⁻¹
    const HostMat P_prior = s.P;
    const CoreState x_prior = s.x;
    Eigen::MatrixXd Hh = Eigen::MatrixXd::Zero(ms.z.size(), kH);
    Hh.leftCols(6) = ms.Hc;
    const Eigen::MatrixXd K = jointGain(P_prior, Hh, ms.R);
    const Eigen::VectorXd nu = ms.innov(x_prior);
    const CoreState x_post = x_prior.boxplus(K * nu);
    const HostMat P_post = (HostMat::Identity() - K * Hh) * P_prior;
    bool pinv = false;
    ASSERT_TRUE(s.aug.conditionOnHostUpdate(P_prior, P_post, x_post.boxminus(x_prior), &pinv));
    std::printf("[U08] singular prior variant %d: pseudo-inverse used = %d\n", variant, pinv ? 1 : 0);
    if (variant == 1) { EXPECT_TRUE(pinv); }   // a negative pivot cannot pass the LDLT positivity check
    s.x = x_post;
    s.P = P_post;
    const int N = kH + s.n;
    const Eigen::MatrixXd H = ms.Hfull(N);
    const Eigen::MatrixXd Kj = jointGain(Sigma, H, ms.R);
    Sigma = Sigma - Kj * H * Sigma;
    js = js.boxplus(Kj * nu);
    s.compare(Sigma, js, 1e-12, variant == 0 ? "singular prior" : "indefinite prior");
    // row 6 of C⁺ = P⁺(6,6)·Aᵀ(6,:): exactly 0 for the singular prior, ~1e-30·1e-17 for the indefinite one
    EXPECT_LT(s.aug.C().row(HostIdx::expo).cwiseAbs().maxCoeff(), 1e-25);
  }
}

// Documentation case: the upstream writes the iteration as x_i ⊞ (Kν + vec − G vec) instead of x⁻ ⊞ K[ν + H(x_i ⊟ x⁻)].
// The two retractions differ at second order on SO(3), so the hook (which conditions on the host's actual x⁺) and a
// joint filter written in the upstream form agree only to that order — the same order as the IEKF's linearisation.
TEST(U08Hooks, UpstreamRetractionFormSecondOrderOnly) {
  Rig s;
  Eigen::MatrixXd Sigma = s.aug.jointCov(s.P);
  JointState js = s.joint();
  propagateBoth(s, Sigma, js, 10);
  // two iterations with *different* linearisations so the second increment is not negligible
  std::vector<Meas> layers{makeMeas(12, 6, s.x, 0.02, 0.05), makeMeas(12, 6, s.x, 0.02, 0.05)};
  const HostMat P_prior = s.P;
  const CoreState x_prior = s.x;
  CoreState xi = x_prior;
  HostMat G = HostMat::Zero();
  double first_step = 0.0;
  for (size_t it = 0; it < layers.size(); ++it) {
    const Meas& ms = layers[it];
    const Eigen::MatrixXd K = hostGainInfoForm(P_prior, ms);
    const Eigen::VectorXd nu = ms.innov(xi);
    const HostVec vec = x_prior.boxminus(xi);
    G.setZero();
    G.leftCols(6) = K * ms.Hc;
    const HostVec solution = K * nu + vec - G.leftCols(6) * vec.head<6>();
    if (it == 0) first_step = solution.head<3>().norm();
    xi = xi.boxplus(solution);
  }
  const HostMat P_post = (HostMat::Identity() - G) * P_prior;
  bool pinv = false;
  ASSERT_TRUE(s.aug.conditionOnHostUpdate(P_prior, P_post, xi.boxminus(x_prior), &pinv));
  // joint filter in the upstream form
  const int N = kH + s.n;
  JointState ji = js;
  for (const Meas& ms : layers) {
    const Eigen::MatrixXd H = ms.Hfull(N);
    const Eigen::MatrixXd K = jointGain(Sigma, H, ms.R);
    const Eigen::VectorXd nu = ms.innov(ji.host);
    const Eigen::VectorXd vec = js.boxminus(ji);
    const Eigen::VectorXd solution = K * nu + vec - K * (H * vec);
    ji = ji.boxplus(solution);
  }
  const double d_xG = relDiff(s.aug.x(), ji.xG);
  const double d_host = xi.boxminus(ji.host).cwiseAbs().maxCoeff();
  std::printf("[U08] upstream retraction form: first rotation step %.2e rad, x_G rel. diff %.2e, host diff %.2e\n", first_step, d_xG, d_host);
  // host means coincide up to rounding (same iteration on both sides); x_G differs at second order in the steps
  EXPECT_LT(d_host, 1e-12);
  EXPECT_LT(d_xG, 1e-3);
}
