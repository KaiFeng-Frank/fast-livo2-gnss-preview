// SPDX-License-Identifier: BSD-3-Clause
// SlipDetector D1–D6 (DESIGN §10 L3). Owner: WP-c. Tests: U12.
//
// Each detector is a 0/1 verdict on one satellite of one TDCP pair; any detector firing drops the
// satellite from the pair (membership, never a weight). The D4 threshold carries the j_max Δt³/(12λ)
// term of DESIGN §10 (trapezoidal integration error of the Doppler under a bounded jerk), which is
// what keeps D4 honest on the longer pairs of a recovery.
#include "internal/slip.hpp"

#include <cmath>

namespace livo_gnss {

namespace {
// Two-sided w-test threshold at α = 1e-3 (Baarda, same level as L4, DESIGN §10): Φ⁻¹(1 − 5e-4) = 3.2905.
constexpr double kWThreshold = 3.2905267314919255;
}  // namespace

SlipDetector::SlipDetector(const Config::Slip& cfg) : cfg_(cfg) {}

double SlipDetector::d4Threshold(double dt_s, double sigma_dopp_hz, double lambda_m) const {
  // max(dopp_tol_cyc, 4σ_D·Δt + j_max·Δt³/(12λ)) [cycles]. σ_D in Hz → 4σ_DΔt cycles; the jerk term is the
  // trapezoid error of ∫D dt in metres divided by λ.
  const double jerk_term = (lambda_m > 0.0) ? cfg_.jerk_max * dt_s * dt_s * dt_s / (12.0 * lambda_m) : 0.0;
  return std::max(cfg_.dopp_tol_cyc, 4.0 * sigma_dopp_hz * dt_s + jerk_term);
}

SlipVerdict SlipDetector::check(const SlipInput& in) const {
  SlipVerdict v;
  if (!in.k0 || !in.k1) {
    v.slip = true;          // no observation at one end: not a usable pair member
    return v;
  }
  const SignalObs& a = *in.k0;
  const SignalObs& b = *in.k1;

  // D1: loss-of-lock indicator on the interval (k0, k1]: the end epoch's LLI (bit0 loss, bit1 half-cycle
  // slip) or any intermediate epoch's LLI tracked by the caller. k0's own LLI refers to the epoch before
  // the pair started and does not concern this pair.
  if (cfg_.lli && (((b.lli & 0x3) != 0) || in.lli_between)) v.detectors |= 1u << 0;

  // D2: half-cycle status changed between the two ends (bit2 valid / bit3 subtracted): 0.5-cycle slip.
  if (cfg_.half_cycle) {
    const uint8_t mask = kStatusHalfCycValid | kStatusHalfCycSub;
    if (((a.status ^ b.status) & mask) != 0) v.detectors |= 1u << 1;
  }

  // D3: pair too long (ephemeris / atmosphere drift and the D4 jerk term both grow with Δt).
  if (in.dt_s > cfg_.max_gap_s) v.detectors |= 1u << 2;

  // D4: Doppler-integrated phase. dΦ/dt = −D (§1: λ·dΦ/dt = −λD) ⇒ ΔΦ + (D_k1 + D_k0)Δt/2 ≈ 0. An L2 integer-ms
  // clock jump moves Φ by k·c·1 ms/λ but not D (a rate), so it is removed from ΔΦ first (review m5).
  if (a.doppValid() && b.doppValid() && in.dt_s > 0.0) {
    const double jump_cyc = in.lambda_m > 0.0 ? in.clock_jump_m / in.lambda_m : 0.0;
    const double dphi = b.cp_cyc - a.cp_cyc - jump_cyc;
    v.d4_value_cyc = std::fabs(dphi + 0.5 * (a.dopp_hz + b.dopp_hz) * in.dt_s);
    v.d4_threshold_cyc = d4Threshold(in.dt_s, in.sigma_dopp_hz, in.lambda_m);
    if (v.d4_value_cyc > v.d4_threshold_cyc) v.detectors |= 1u << 3;
  }

  // D5: innovation against the LIVO prediction: w-test AND the absolute floor (the mm-level prediction
  // would otherwise flag atmospheric noise; the floor keeps D5 for ≥ 5 cm effects, e.g. 1 cycle = 19 cm).
  if (in.innov_valid && in.innov_sigma_m > 0.0) {
    const double w = std::fabs(in.innov_pred_m) / in.innov_sigma_m;
    if (w > kWThreshold && std::fabs(in.innov_pred_m) > cfg_.innov_abs_m) v.detectors |= 1u << 4;
  }

  // D6: dual-frequency geometry-free combination λ1Φ1 − λ2Φ2 [m] changes by more than gf_tol_m.
  if (in.k0_second && in.k1_second && in.lambda_m > 0.0 && in.lambda2_m > 0.0 && in.k0_second->cpValid() &&
      in.k1_second->cpValid() && a.cpValid() && b.cpValid()) {
    const double gf0 = in.lambda_m * a.cp_cyc - in.lambda2_m * in.k0_second->cp_cyc;
    const double gf1 = in.lambda_m * b.cp_cyc - in.lambda2_m * in.k1_second->cp_cyc;
    v.d6_value_m = std::fabs(gf1 - gf0);
    if (v.d6_value_m > cfg_.gf_tol_m) v.detectors |= 1u << 5;
  }

  v.slip = v.detectors != 0;
  return v;
}

}  // namespace livo_gnss
