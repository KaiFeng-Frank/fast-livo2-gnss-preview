// SPDX-License-Identifier: BSD-3-Clause
// clock — receiver clock two-state model and clock-jump handling (DESIGN §2.2, §3.1, §10 L2):
//   Φ = [[1, Δt], [0, 1]],  Q = [[S_bΔt + S_dΔt³/3, S_dΔt²/2], [S_dΔt²/2, S_dΔt]]  (Groves; U9 vs Van Loan)
//   S_b lower bound: √(S_b Δt_epoch) ≥ |ω_z| Δt_epoch λ/2π (phase wind-up absorbed by the clock increment, R2-13),
//   i.e. S_b ≥ (|ω_z| λ/2π)² Δt_epoch — a bound on the PSD, applied per LIO step with that step's |ω_z| but with the
//   epoch interval Δt_epoch (ClockModel::windup_epoch_dt_s), never with the LIO step length
//   L2: m = median pseudorange innovation; |m − k·c·1 ms| < 1 km (k ≠ 0) ⇒ b += k·c·1 ms and λΔΦ gets
//   k·c·1 ms removed so TDCP stays usable; |m| > 1 km non-integer ⇒ clock-only LS reset (b, d; MAD² variance,
//   C clock rows zeroed). Event CLOCK_JUMP.
// Owner: WP-b. Tests: test_u09_augblock.cpp (Q vs Van Loan; k = ±1, ±2 ms detection — moved from U12, A9).
// The "TDCP still usable after subtracting k·c·1 ms" half of that check is a rows.cpp property and stays in U12.
#pragma once

#include <vector>

#include <Eigen/Core>

#include "livo_gnss/config.hpp"

namespace livo_gnss {

class AugBlock;   // the L2 actions below only take a pointer; aug_block.hpp is included by clock.cpp

struct ClockModel {
  explicit ClockModel(const Config::Process::Clock& cfg);
  Eigen::Matrix2d Phi(double dt) const;
  // Q for the step with the wind-up lower bound applied when enabled (`omega_z_abs` [rad/s], λ [m]).
  Eigen::Matrix2d Q(double dt, double omega_z_abs, double lambda_m) const;
  // Reference implementation via Van Loan (matrix exponential) for U9.
  static Eigen::Matrix2d QVanLoan(double dt, double S_b, double S_d);
  double S_b = 0.1, S_d = 0.1;
  bool windup_bound = true;
  // Δt_epoch of the wind-up bound S_b ≥ (|ω_z|λ/2π)²·Δt_epoch (file header; DESIGN §2.2). With the bound applied
  // per LIO step using that step's |ω_z|, the clock variance accumulated over one epoch is
  // Σ_i max(S_b, (|ω_z,i|λ/2π)²Δt_epoch)·Δt_i ≥ (λ/2π)²·(Σ_i |ω_z,i|Δt_i)² (Cauchy–Schwarz), i.e. at least the squared
  // wind-up of that epoch at any LIO rate; with the LIO step length in place of Δt_epoch it would only be
  // Δt_LIO/Δt_epoch of that. Default = nominal M3DGR u-blox epoch interval (10 Hz, DESIGN §12 audit 1); the glue may
  // set the measured interval through AugBlock::setWindupEpochDt. Appended by WP-b (INTERFACE_CHANGES).
  double windup_epoch_dt_s = 0.1;
};

struct ClockJumpVerdict {
  bool jump = false;
  int k_ms = 0;                 // integer-millisecond jump (0 = none)
  double median_innov_m = 0.0;
  bool needs_reset = false;     // |m| > threshold but not an integer ms
};

class ClockJumpDetector {
 public:
  explicit ClockJumpDetector(const Config::Rejection& cfg);
  // Pseudorange innovations (before any gate) of the current epoch.
  ClockJumpVerdict detect(const std::vector<double>& psr_innovations_m) const;
  static constexpr double kMsInMetres = 299792.458;   // c · 1 ms

 private:
  double jump_threshold_m_ = 1000.0;
};

// ---- L2 actions on the augmented block (appended by WP-b, INTERFACE_CHANGES.md; DESIGN §10 L2) ----
// Both are no-ops when the clk slot is absent.
// Integer-millisecond jump: b += k·c·1 ms. Covariance untouched: the receiver re-steered its own clock by a
// known amount, nothing else about the estimate changed.
void applyIntegerMsJump(AugBlock* aug, int k_ms);
// Clock-only LS reset (|m| > threshold, non-integer): (b, d) replaced by the independent estimate with
// covariance `cov_bd` (MAD² on the diagonal from ClockOnlyLs); the clk columns of C and the clk cross-covariances
// inside Q_G are zeroed because the new estimate is independent of everything else ("C 钟行清零").
void resetClockStates(AugBlock* aug, double b_m, double d_mps, const Eigen::Matrix2d& cov_bd);

}  // namespace livo_gnss
