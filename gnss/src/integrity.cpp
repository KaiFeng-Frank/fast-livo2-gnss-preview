// SPDX-License-Identifier: BSD-3-Clause
// IntegrityMonitor (DESIGN §10 L7, degeneracy flag). Owner: P3-A1. Tests: U17, U15.
//
// External judges that only set flags (the host P is never touched here; R1 rejected D2's P_vv/P_pp surgery):
//   * velocity alarm (L7): the GNSS-only Doppler *speed* against the LIVO-predicted antenna speed (χ²₁, DESIGN §10: speeds
//     are frame-free, so the judge does not depend on ψ — the heading judge below owns that). Bad epoch: d² > χ²₁(1−α)
//     and |Δspeed| > min_err_mps. n_consec consecutive bad epochs ⇒ LIVO_ALARM on (the facade then runs the updater in
//     Schmidt mode: host columns of K zeroed, JointUpdater::Options::schmidt); n_clear consecutive good epochs ⇒ off.
//   * heading (L7): horizontal direction of the two velocities, both ≥ min_speed; error > max(min_err_deg, 3σ) with σ² =
//     σ_ψ² + the two directions' own variances, n_consec consecutive times ⇒ REINIT_YAW (one-shot edge, cleared by
//     clearEdges()). Epochs below min_speed, and epochs while LIVO_ALARM is on (the LIVO velocity is the suspect), are
//     no evidence either way.
//   * degeneracy index: information gain of the LIO update Y = (P⁺)⁻¹ − (P⁻)⁻¹, its [θ, p] block, the translation Schur
//     complement S = Y_pp − Y_pθ Y_θθ⁺ Y_θp; λ₁ < lambda_enter for `window` consecutive frames ⇒ DEGENERATE, λ₁ >
//     lambda_exit for `window` consecutive frames ⇒ clear. The floor (degeneracy.floor_enable) is applied by the caller.
#include "internal/integrity.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <Eigen/Cholesky>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include "internal/update.hpp"   // chi2Quantile

namespace livo_gnss {

namespace {

constexpr double kDeg = M_PI / 180.0;

// Inverse of a symmetric PSD matrix restricted to the index set `idx`: Jacobi scaling + LDLT, eigen pseudo-inverse
// (relative cut 1e-12) when the LDLT is not positive.
Eigen::MatrixXd pinvSym(const Eigen::MatrixXd& A) {
  const int n = static_cast<int>(A.rows());
  Eigen::VectorXd d(n);
  for (int i = 0; i < n; ++i) d(i) = (A(i, i) > 0.0 && std::isfinite(A(i, i))) ? 1.0 / std::sqrt(A(i, i)) : 1.0;
  const Eigen::MatrixXd As = d.asDiagonal() * (0.5 * (A + A.transpose())) * d.asDiagonal();
  Eigen::LDLT<Eigen::MatrixXd> ldlt(As);
  if (ldlt.info() == Eigen::Success && ldlt.isPositive()) {
    const Eigen::MatrixXd inv = ldlt.solve(Eigen::MatrixXd::Identity(n, n));
    if (inv.allFinite() && (As * inv - Eigen::MatrixXd::Identity(n, n)).cwiseAbs().maxCoeff() < 1e-8)
      return d.asDiagonal() * inv * d.asDiagonal();
  }
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(As);
  const Eigen::VectorXd lam = es.eigenvalues();
  const double lmax = lam.cwiseAbs().maxCoeff();
  Eigen::VectorXd li = Eigen::VectorXd::Zero(n);
  for (int i = 0; i < n; ++i) li(i) = (lam(i) > 1e-12 * lmax) ? 1.0 / lam(i) : 0.0;
  return d.asDiagonal() * es.eigenvectors() * li.asDiagonal() * es.eigenvectors().transpose() * d.asDiagonal();
}

// Direction variance [rad²] of a horizontal velocity from its 3×3 covariance: σ² = n̂ᵀΣ_hh n̂ / |v_h|², n̂ ⟂ v_h.
double headingVar(const Eigen::Vector3d& v, const Eigen::Matrix3d& cov) {
  const Eigen::Vector2d vh(v.x(), v.y());
  const double s2 = vh.squaredNorm();
  if (!(s2 > 0.0)) return 0.0;
  const Eigen::Vector2d nh(-vh.y() / std::sqrt(s2), vh.x() / std::sqrt(s2));
  return nh.dot(cov.topLeftCorner<2, 2>() * nh) / s2;
}

}  // namespace

IntegrityMonitor::IntegrityMonitor(const Config::Integrity& cfg) : cfg_(cfg) {}

void IntegrityMonitor::onVelocity(const Eigen::Vector3d& v_gnss_N, const Eigen::Matrix3d& cov_gnss, const Eigen::Vector3d& w_N,
                                  const Eigen::Matrix3d& cov_w, double sigma_yaw_rad) {
  // ---- velocity (speed) alarm ----
  {
    const Config::Integrity::VelocityAlarm& va = cfg_.velocity_alarm;
    const double sg = v_gnss_N.norm(), sw = w_N.norm();
    const Eigen::Vector3d sum = v_gnss_N + w_N;
    const Eigen::Matrix3d S = cov_gnss + cov_w;
    bool bad = false;
    double d2 = 0.0;
    if (sum.norm() > 0.0) {
      // Var(|v_G| − |w|) to first order: both speeds vary along their own direction; with the two directions close the
      // common unit vector of v_G + w is the projection (exact when parallel).
      const Eigen::Vector3d u = sum.normalized();
      const double var = u.dot(S * u);
      const double diff = sg - sw;
      d2 = var > 0.0 ? diff * diff / var : (diff != 0.0 ? std::numeric_limits<double>::infinity() : 0.0);
      bad = d2 > chi2Quantile(1, 1.0 - va.alpha) && std::fabs(diff) > va.min_err_mps;
    }
    flags_.d2_velocity = d2;
    if (va.enable) {
      if (bad) {
        ++n_bad_vel_;
        n_good_vel_ = 0;
      } else {
        ++n_good_vel_;
        n_bad_vel_ = 0;
      }
      if (!flags_.livo_alarm && n_bad_vel_ >= va.n_consec) {
        flags_.livo_alarm = true;
        flags_.alarm_changed = true;
        n_good_vel_ = 0;
      } else if (flags_.livo_alarm && n_good_vel_ >= va.n_clear) {
        flags_.livo_alarm = false;
        flags_.alarm_changed = true;
        n_bad_vel_ = 0;
      }
    }
  }
  // ---- heading judge ----
  {
    const Config::Integrity::Heading& hc = cfg_.heading;
    const Eigen::Vector2d gh(v_gnss_N.x(), v_gnss_N.y()), wh(w_N.x(), w_N.y());
    if (hc.enable && !flags_.livo_alarm && gh.norm() >= hc.min_speed && wh.norm() >= hc.min_speed) {
      double err = std::atan2(gh.y(), gh.x()) - std::atan2(wh.y(), wh.x());
      err -= 2.0 * M_PI * std::floor((err + M_PI) / (2.0 * M_PI));
      flags_.heading_err_deg = err / kDeg;
      const double sigma = std::sqrt(sigma_yaw_rad * sigma_yaw_rad + headingVar(v_gnss_N, cov_gnss) + headingVar(w_N, cov_w));
      const double thr = std::max(hc.min_err_deg * kDeg, 3.0 * sigma);
      if (std::fabs(err) > thr) ++n_bad_heading_; else n_bad_heading_ = 0;
      if (n_bad_heading_ >= hc.n_consec) {
        flags_.reinit_yaw = true;   // one-shot: the caller redoes ψ (REINIT) and clears the edge
        n_bad_heading_ = 0;
      }
    }
  }
}

void IntegrityMonitor::onLioCovariance(const HostMat& P_prior, const HostMat& P_post) {
  // Restrict to the states with a prior variance (a never-estimated state has a zero row/column and no information).
  std::vector<int> idx;
  for (int i = 0; i < HostIdx::dim; ++i)
    if (P_prior(i, i) > 0.0 && std::isfinite(P_prior(i, i))) idx.push_back(i);
  for (int i = 0; i < 6; ++i)
    if (std::find(idx.begin(), idx.end(), i) == idx.end()) return;   // θ or p without variance: nothing to judge
  const int n = static_cast<int>(idx.size());
  Eigen::MatrixXd Pm(n, n), Pp(n, n);
  for (int a = 0; a < n; ++a)
    for (int b = 0; b < n; ++b) {
      Pm(a, b) = P_prior(idx[a], idx[b]);
      Pp(a, b) = 0.5 * (P_post(idx[a], idx[b]) + P_post(idx[b], idx[a]));   // the host's (I−G)P⁻ is not symmetric
    }
  const Eigen::MatrixXd Y = pinvSym(Pp) - pinvSym(Pm);
  // θ, p are the first six indices of idx (0..5 all present)
  const Eigen::Matrix3d Ytt = 0.5 * (Y.block<3, 3>(0, 0) + Y.block<3, 3>(0, 0).transpose());
  const Eigen::Matrix3d Ypp = 0.5 * (Y.block<3, 3>(3, 3) + Y.block<3, 3>(3, 3).transpose());
  const Eigen::Matrix3d Ypt = 0.5 * (Y.block<3, 3>(3, 0) + Y.block<3, 3>(0, 3).transpose());
  const Eigen::Matrix3d Ytt_inv = pinvSym(Ytt);
  Eigen::Matrix3d S = Ypp - Ypt * Ytt_inv * Ypt.transpose();
  S = 0.5 * (S + S.transpose()).eval();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(S);
  if (es.info() != Eigen::Success || !es.eigenvalues().allFinite()) return;
  flags_.lambda = es.eigenvalues();   // ascending: λ₁ first
  flags_.u1 = es.eigenvectors().col(0);
  const Config::Integrity::Degeneracy& dg = cfg_.degeneracy;
  const double l1 = flags_.lambda(0);
  if (l1 < dg.lambda_enter) ++n_low_lambda_; else n_low_lambda_ = 0;
  if (l1 > dg.lambda_exit) ++n_high_lambda_; else n_high_lambda_ = 0;
  if (!flags_.degenerate && n_low_lambda_ >= dg.window) {
    flags_.degenerate = true;
    flags_.degenerate_changed = true;
    n_high_lambda_ = 0;
  } else if (flags_.degenerate && n_high_lambda_ >= dg.window) {
    flags_.degenerate = false;
    flags_.degenerate_changed = true;
    n_low_lambda_ = 0;
  }
}

void IntegrityMonitor::reset() { flags_ = IntegrityFlags(); n_bad_vel_ = n_good_vel_ = n_bad_heading_ = n_low_lambda_ = n_high_lambda_ = 0; }
void IntegrityMonitor::clearEdges() {
  flags_.alarm_changed = false;
  flags_.degenerate_changed = false;
  flags_.reinit_yaw = false;
}

}  // namespace livo_gnss
