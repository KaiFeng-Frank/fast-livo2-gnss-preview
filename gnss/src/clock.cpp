// SPDX-License-Identifier: BSD-3-Clause
// clock (DESIGN §2.2, §3.1, §10 L2). Owner: WP-b. Tests: U9 (Q vs Van Loan, k = ±1, ±2 ms detection), U12 (rows.cpp half).
#include "internal/clock.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include "internal/aug_block.hpp"

namespace livo_gnss {

namespace {

// Matrix exponential of a 4×4 by scaling and squaring + Taylor series summed to convergence in double.
// Deliberately generic (no use of the nilpotent structure of the Van Loan matrix) so that QVanLoan() is an
// independent reference for the closed-form Q of ClockModel::Q (U9).
Eigen::Matrix4d expm4(const Eigen::Matrix4d& M) {
  const double norm_inf = M.cwiseAbs().rowwise().sum().maxCoeff();
  int s = 0;
  if (norm_inf > 0.5) s = static_cast<int>(std::ceil(std::log2(norm_inf / 0.5)));
  const Eigen::Matrix4d A = M / std::ldexp(1.0, s);
  Eigen::Matrix4d term = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d E = Eigen::Matrix4d::Identity();
  for (int k = 1; k <= 40; ++k) {
    term = (term * A) / static_cast<double>(k);
    E += term;
    if (term.cwiseAbs().maxCoeff() <= 1e-300) break;
  }
  for (int i = 0; i < s; ++i) E = E * E;
  return E;
}

}  // namespace

ClockModel::ClockModel(const Config::Process::Clock& cfg) : S_b(cfg.S_b), S_d(cfg.S_d), windup_bound(cfg.S_b_min_windup) {}

Eigen::Matrix2d ClockModel::Phi(double dt) const {
  Eigen::Matrix2d P;
  P << 1.0, dt, 0.0, 1.0;
  return P;
}

Eigen::Matrix2d ClockModel::Q(double dt, double omega_z_abs, double lambda_m) const {
  // Groves two-state clock: ḃ = d + w_b (PSD S_b), ḋ = w_d (PSD S_d). Exact discretisation over dt
  // (DESIGN §2.2; not the upstream σ²·dt² form, IMU_Processing.cpp:395-399).
  Eigen::Matrix2d Q = Eigen::Matrix2d::Zero();
  if (!(dt > 0.0)) return Q;
  double sb = S_b;
  if (windup_bound && omega_z_abs > 0.0 && lambda_m > 0.0) {
    // Phase wind-up over one epoch interval (|ω_z|·Δt_epoch·λ/2π metres) must be absorbable by the clock increment
    // of that interval: √(S_b Δt_epoch) ≥ |ω_z| Δt_epoch λ/2π  ⇔  S_b ≥ (|ω_z| λ/2π)² Δt_epoch (R2-13). The bound is
    // on the PSD and therefore uses Δt_epoch, not the LIO step dt this Q is discretised over (clock.hpp).
    const double w = omega_z_abs * lambda_m / (2.0 * M_PI);
    sb = std::max(sb, w * w * std::max(windup_epoch_dt_s, 0.0));
  }
  const double dt2 = dt * dt, dt3 = dt2 * dt;
  Q << sb * dt + S_d * dt3 / 3.0, S_d * dt2 / 2.0, S_d * dt2 / 2.0, S_d * dt;
  return Q;
}

Eigen::Matrix2d ClockModel::QVanLoan(double dt, double S_b, double S_d) {
  // Van Loan (1978): for ẋ = A x + w, E[w wᵀ] = W δ(τ), build M = [[−A, W], [0, Aᵀ]]·dt and
  // exp(M) = [[·, Φ⁻¹Q], [0, Φᵀ]]  ⇒  Q = Φ · (upper-right block) = (lower-right)ᵀ · (upper-right).
  Eigen::Matrix2d A;
  A << 0.0, 1.0, 0.0, 0.0;
  Eigen::Matrix2d W = Eigen::Matrix2d::Zero();
  W(0, 0) = S_b;
  W(1, 1) = S_d;
  Eigen::Matrix4d M = Eigen::Matrix4d::Zero();
  M.block<2, 2>(0, 0) = -A;
  M.block<2, 2>(0, 2) = W;
  M.block<2, 2>(2, 2) = A.transpose();
  M *= dt;
  const Eigen::Matrix4d E = expm4(M);
  const Eigen::Matrix2d Phi = E.block<2, 2>(2, 2).transpose();
  return Phi * E.block<2, 2>(0, 2);
}

ClockJumpDetector::ClockJumpDetector(const Config::Rejection& cfg) : jump_threshold_m_(cfg.clock_jump_km * 1000.0) {}

ClockJumpVerdict ClockJumpDetector::detect(const std::vector<double>& psr_innovations_m) const {
  ClockJumpVerdict v;
  std::vector<double> s;
  s.reserve(psr_innovations_m.size());
  for (double x : psr_innovations_m)
    if (std::isfinite(x)) s.push_back(x);
  if (s.empty()) return v;
  // Median (robust against a few NLOS/outlier satellites): a u-blox clock steer moves every pseudorange by
  // the same k·c·1 ms, a single bad satellite does not move the median.
  std::sort(s.begin(), s.end());
  const size_t n = s.size();
  const double m = (n % 2 == 1) ? s[n / 2] : 0.5 * (s[n / 2 - 1] + s[n / 2]);
  v.median_innov_m = m;
  const double k = std::round(m / kMsInMetres);
  // |k| ≥ 1e6 ms (3e11 m) is no receiver clock steer of any kind: the non-integer path, decided before the int
  // conversion so that the conversion stays defined for any finite m.
  if (!(std::abs(k) < 1.0e6)) {
    v.needs_reset = true;
    return v;
  }
  const int ki = static_cast<int>(k);
  if (ki != 0 && std::abs(m - k * kMsInMetres) < jump_threshold_m_) {
    v.jump = true;
    v.k_ms = ki;
  } else if (std::abs(m) > jump_threshold_m_) {
    v.needs_reset = true;
  }
  return v;
}

void applyIntegerMsJump(AugBlock* aug, int k_ms) {
  if (!aug || k_ms == 0) return;
  const int i = aug->find(Slot::Clk);
  if (i < 0) return;
  aug->x()(i) += static_cast<double>(k_ms) * ClockJumpDetector::kMsInMetres;
}

void resetClockStates(AugBlock* aug, double b_m, double d_mps, const Eigen::Matrix2d& cov_bd) {
  if (!aug) return;
  const int i = aug->find(Slot::Clk);
  if (i < 0) return;
  aug->x()(i) = b_m;
  aug->x()(i + 1) = d_mps;
  aug->C().middleCols<2>(i).setZero();
  Eigen::MatrixXd& Q = aug->Q();
  Q.middleRows<2>(i).setZero();
  Q.middleCols<2>(i).setZero();
  Q.block<2, 2>(i, i) = 0.5 * (cov_bd + cov_bd.transpose());
}

}  // namespace livo_gnss
