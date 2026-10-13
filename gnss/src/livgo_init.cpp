// SPDX-License-Identifier: BSD-3-Clause
#include "livo_gnss/livgo_init.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>

#include "livo_gnss/core_state.hpp"

namespace livo_gnss {

bool livgoInitLeastSquares(const std::vector<LivgoInitSample>& samples, const Eigen::Matrix3d& R0,
                           const Eigen::Vector3d& p0, const Eigen::Vector3d& lever0, LivgoInitEstimate* out) {
  if (!out) return false;
  *out = LivgoInitEstimate();
  out->n = static_cast<int>(samples.size());
  if (out->n < 10) {
    out->note = "fewer than 10 GNSS positions in the initialisation window";
    return false;
  }
  for (const LivgoInitSample& s : samples) {
    if (!s.p_I.allFinite() || !s.R_I.allFinite() || !s.p_r.allFinite()) {
      out->note = "non-finite pose or GNSS position";
      return false;
    }
  }
  Eigen::Matrix3d R = R0;
  Eigen::Vector3d p = p0;
  Eigen::Vector3d t = lever0;
  if (!R.allFinite() || !p.allFinite() || !t.allFinite()) {
    out->note = "non-finite start";
    return false;
  }
  // Rotation and translation first, lever held at the mechanical start: a wrong yaw otherwise trades
  // against a runaway lever. The joint steps afterwards use 1e-6·trace damping so a rank-deficient
  // lever direction stays at that start; a step that raises the cost is rejected and the damping grows.
  auto accumulate = [&](Eigen::Matrix<double, 9, 9>* H, Eigen::Matrix<double, 9, 1>* g, double* sse) {
    H->setZero();
    g->setZero();
    *sse = 0.0;
    for (const LivgoInitSample& s : samples) {
      const Eigen::Vector3d q = s.p_I + s.R_I * t;
      const Eigen::Vector3d pred = R * q + p;
      const Eigen::Vector3d r = s.p_r - pred;
      *sse += r.squaredNorm();
      Eigen::Matrix<double, 3, 9> J;
      J.block<3, 3>(0, 0) = skew(pred - p);                 // ∂r/∂δφ, left perturbation of ^E R_G
      J.block<3, 3>(0, 3) = -Eigen::Matrix3d::Identity();   // ∂r/∂p
      J.block<3, 3>(0, 6) = -(R * s.R_I);                   // ∂r/∂t
      H->noalias() += J.transpose() * J;
      g->noalias() -= J.transpose() * r;                    // (JᵀJ) dx = −Jᵀ r
    }
  };
  double mu_scale = 1e-6;
  double sse = 0.0;
  Eigen::Matrix<double, 9, 9> H = Eigen::Matrix<double, 9, 9>::Zero();
  bool joint = false;
  for (int iter = 0; iter < 30; ++iter) {
    Eigen::Matrix<double, 9, 1> g;
    accumulate(&H, &g, &sse);
    bool accepted = false;
    double mu_try = mu_scale;
    for (int damp = 0; damp < 10; ++damp) {
      const double tr = H.trace();
      const double mu = mu_try * ((tr > 0.0 && std::isfinite(tr)) ? tr : 1.0);
      Eigen::Matrix<double, 9, 9> Hd = H;
      Hd.diagonal().array() += mu;
      const Eigen::LDLT<Eigen::Matrix<double, 9, 9>> ldlt(Hd);
      if (ldlt.info() != Eigen::Success) {
        mu_try *= 10.0;
        continue;
      }
      Eigen::Matrix<double, 9, 1> dx = ldlt.solve(g);
      if (!joint) dx.tail<3>().setZero();
      if (!dx.allFinite()) {
        mu_try *= 10.0;
        continue;
      }
      const Eigen::Matrix3d R_try = so3Exp(dx.head<3>()) * R;
      const Eigen::Vector3d p_try = p + dx.segment<3>(3);
      const Eigen::Vector3d t_try = t + dx.tail<3>();
      double sse_try = 0.0;
      for (const LivgoInitSample& s : samples) {
        const Eigen::Vector3d q = s.p_I + s.R_I * t_try;
        sse_try += (s.p_r - (R_try * q + p_try)).squaredNorm();
      }
      if (sse_try <= sse * (1.0 + 1e-12)) {
        R = R_try;
        p = p_try;
        t = t_try;
        sse = sse_try;
        accepted = true;
        ++out->iterations;
        mu_scale = std::max(1e-6, mu_try / 3.0);
        if (!joint && dx.head<6>().norm() < 1e-8) joint = true;
        if (joint && dx.norm() < 1e-10) iter = 30;
        break;
      }
      mu_try *= 10.0;
    }
    if (!accepted) {
      if (!joint) {
        joint = true;
        continue;
      }
      break;
    }
  }
  {
    Eigen::Matrix<double, 9, 1> g;
    accumulate(&H, &g, &sse);
    out->lambda_min = Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 9, 9>>(H).eigenvalues()(0);
    const double tr = H.trace();
    const double mu = 1e-6 * ((tr > 0.0 && std::isfinite(tr)) ? tr : 1.0);
    Eigen::Matrix<double, 9, 9> Hd = H;
    Hd.diagonal().array() += mu;
    const Eigen::LDLT<Eigen::Matrix<double, 9, 9>> ldlt(Hd);
    if (ldlt.info() == Eigen::Success) out->cov = ldlt.solve(Eigen::Matrix<double, 9, 9>::Identity());
  }
  const double dof = std::max(1.0, 3.0 * static_cast<double>(out->n) - 9.0);
  const double sigma2 = sse / dof;
  out->cov *= sigma2;
  out->R = R;
  out->p = p;
  out->lever = t;
  out->rms_m = std::sqrt(sse / (3.0 * static_cast<double>(out->n)));
  out->ok = R.allFinite() && p.allFinite() && t.allFinite() && std::isfinite(out->rms_m);
  if (!out->ok) out->note = "non-finite solution";
  return out->ok;
}

void LivgoInitHistory::add(const LivgoInitEpoch& e, const Eigen::Vector3d& g_W) {
  if (!e.p_I.allFinite() || !e.R_I.allFinite() || !e.r_ecef.allFinite()) return;
  Eigen::Vector3d up = -g_W;   // the gravity points down; only its direction is used
  const double n = up.norm();
  if (std::isfinite(n) && n > 1e-6) up /= n; else up = Eigen::Vector3d::UnitZ();
  if (!have_origin_) {
    origin_ = e.p_I;
    have_origin_ = true;
  } else {
    const Eigen::Vector3d d = e.p_I - origin_;
    const Eigen::Vector3d dh = d - d.dot(up) * up;
    travel_m_ = std::max(travel_m_, dh.norm());
  }
  epochs_.push_back(e);
  while (epochs_.size() > capacity_) epochs_.pop_front();
}

void LivgoInitHistory::clear() {
  epochs_.clear();
  track_.clear();   // A82
  travel_m_ = 0.0;
  have_origin_ = false;
}

std::vector<LivgoInitSample> LivgoInitHistory::samples(const Eigen::Vector3d& r0_ecef, const Eigen::Matrix3d& R_EN) const {
  std::vector<LivgoInitSample> out;
  out.reserve(epochs_.size());
  for (const LivgoInitEpoch& e : epochs_) {
    LivgoInitSample s;
    s.p_I = e.p_I;
    s.R_I = e.R_I;
    s.p_r = R_EN.transpose() * (e.r_ecef - r0_ecef);
    if (!s.p_r.allFinite()) continue;
    out.push_back(s);
  }
  return out;
}

LivgoTiltDecision livgoTiltDecision(const LivgoInitEstimate& est, double max_deg) {
  LivgoTiltDecision d;
  const double rad2deg = 180.0 / M_PI;
  if (!std::isfinite(est.cov(0, 0)) || !std::isfinite(est.cov(1, 1))) {
    d.sigma_deg = std::numeric_limits<double>::quiet_NaN();
  } else {
    const double sx = std::sqrt(std::max(0.0, est.cov(0, 0))) * rad2deg;
    const double sy = std::sqrt(std::max(0.0, est.cov(1, 1))) * rad2deg;
    d.sigma_deg = std::max(sx, sy);
  }
  if (!(max_deg > 0.0)) return d;
  d.use_ls_tilt = std::isfinite(d.sigma_deg) && d.sigma_deg <= max_deg;
  return d;
}

// ---- AMENDMENTS A82 (livgo.dtw_at_init) ----
void LivgoInitHistory::addTrack(const LivgoHostPose& p) {
  if (!std::isfinite(p.t) || !p.R.allFinite() || !p.p.allFinite() || !p.v.allFinite()) return;
  if (!track_.empty() && !(p.t > track_.back().t)) return;
  track_.push_back(p);
  while (track_.size() > capacity_) track_.pop_front();
}

LivgoSpeedSeries LivgoInitHistory::speedSeries() const {
  LivgoSpeedSeries s;
  s.t.reserve(epochs_.size());
  s.v_gnss.reserve(epochs_.size());
  s.v_host.reserve(epochs_.size());
  for (const LivgoInitEpoch& e : epochs_) {
    if (!std::isfinite(e.t_est) || !std::isfinite(e.v_gnss) || !std::isfinite(e.v_host)) continue;
    s.t.push_back(e.t_est);
    s.v_gnss.push_back(e.v_gnss);
    s.v_host.push_back(e.v_host);
  }
  return s;
}

void livgoInterpolateHostPose(const LivgoHostPose& a, const LivgoHostPose& b, double t, Eigen::Matrix3d* R, Eigen::Vector3d* p) {
  const double h = b.t - a.t;
  if (!(h > 0.0)) {
    if (R) *R = a.R;
    if (p) *p = a.p;
    return;
  }
  const double s = std::min(1.0, std::max(0.0, (t - a.t) / h));
  const double s2 = s * s, s3 = s2 * s;
  const double h00 = 2.0 * s3 - 3.0 * s2 + 1.0, h10 = s3 - 2.0 * s2 + s, h01 = -2.0 * s3 + 3.0 * s2, h11 = s3 - s2;
  if (p) *p = h00 * a.p + (h10 * h) * a.v + h01 * b.p + (h11 * h) * b.v;
  if (R) {
    const Eigen::Vector3d w = so3Log(Eigen::Matrix3d(a.R.transpose() * b.R));
    *R = a.R * so3Exp(s * w);
  }
}

LivgoRepairResult LivgoInitHistory::repairPairing(double lag_s) {
  LivgoRepairResult res;
  std::deque<LivgoInitEpoch> kept;
  for (const LivgoInitEpoch& src : epochs_) {
    LivgoInitEpoch e = src;
    if (!std::isfinite(e.t_est) || !std::isfinite(lag_s) || track_.size() < 2) {
      ++res.dropped;
      continue;
    }
    const double tau = e.t_est + lag_s;
    if (tau < track_.front().t || tau > track_.back().t) {
      ++res.dropped;
      continue;
    }
    // the first sample at or after tau; it exists because tau <= track_.back().t
    const auto it = std::lower_bound(track_.begin(), track_.end(), tau, [](const LivgoHostPose& a, double t) { return a.t < t; });
    if (it->t == tau) {   // a sample at exactly that time (this includes tau == front().t, so below `it` is never begin())
      e.p_I = it->p;
      e.R_I = it->R;
      ++res.repaired;
      kept.push_back(e);
      continue;
    }
    const LivgoHostPose& b = *it;
    const LivgoHostPose& a = *(it - 1);
    const double gap = b.t - a.t;
    if (gap > kLivgoRepairMaxGapS) {
      ++res.dropped;
      continue;
    }
    livgoInterpolateHostPose(a, b, tau, &e.R_I, &e.p_I);
    res.max_gap_s = std::max(res.max_gap_s, gap);
    ++res.repaired;
    kept.push_back(e);
  }
  epochs_.swap(kept);
  return res;
}

}  // namespace livo_gnss
