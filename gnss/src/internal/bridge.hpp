// SPDX-License-Identifier: BSD-3-Clause
// ImuBridge — IMU bridging between a GNSS epoch time t_g ∈ (t_{k−1}, t_k] and the LIO cycle end t_k
// (DESIGN §4.1). Samples are recorded by the host inside UndistortPcl's propagation loop (start sample
// {prop_beg_time, R, p, v, angvel_last} then one per IMU interval after :430). The ring buffer keeps at least
// `bridge_ring_cycles` cycles and every cycle with t_k ≥ t_latest − (max_bridge_s + 0.1 s) (retention by duration,
// AMENDMENTS A33); late epochs chain relative motion across cycles (δ ≤ max_bridge_s).
// Interpolation inside (t_i, t_{i+1}] uses constant a_{i+1} (incl. g) and ω_{i+1}, τ = t_g − t_i:
//   p = p_i + v_i τ + ½ a τ²,  v = v_i + a τ,  R = R_i Exp(ω τ)    (== IMU_Processing.cpp:412-421, U14)
// Output for any iterate x (AMENDMENTS A10, the U7 reference model): with δθ = Log(R̂_kᵀ R),
//   δa = −R̂_k[ā_b]× δθ − R̂_k (b_a − b̂_a^snap) + (g − ĝ^snap)         (first-order change of a = R f + g)
//   D(x) = D̂ − ½ δ² δa,   E(x) = Ê + δ δa
//   p_g = p − v δ − D(x),  R_g = R ΔR̂ᵀ,  v_g = v − E(x),  ω_g = ω̂_g + b̂_g^snap − b_g
// so the §4.2 columns δθ (−½δ²MR̂[ā_b]×), δb_a (−½δ²MR̂), δg (½δ²M) and their J_w counterparts are the
// exact derivatives of evaluate(); a central difference through evaluate() reproduces them (U7).
// Owner: WP-b. Tests: test_u14_bridge.cpp (δ ≤ 0.1 s error < 0.1 mm; cross-cycle; sample-exact match; A33 retention
// rule and δ = 0.28 s across 15 Hz cycles).
#pragma once

#include <deque>
#include <vector>

#include <Eigen/Core>

#include "livo_gnss/config.hpp"
#include "livo_gnss/core_state.hpp"

namespace livo_gnss {

struct BridgeResult {
  bool ok = false;
  double t_g = 0.0, t_k = 0.0;
  double delta = 0.0;                                   // δ = t_k − t_g ≥ 0
  bool cross_cycle = false;                             // t_g before the current cycle start
  Eigen::Matrix3d dR = Eigen::Matrix3d::Identity();     // ΔR̂ = R̂_gᵀ R̂_k
  Eigen::Vector3d D = Eigen::Vector3d::Zero();          // D̂ = p̂_k − p̂_g − v̂_k δ
  Eigen::Vector3d E = Eigen::Vector3d::Zero();          // Ê = v̂_k − v̂_g
  Eigen::Vector3d omega_g = Eigen::Vector3d::Zero();    // ω̂_g: rate of the interval (t_i, t_{i+1}] with t_{i+1} > t_g
                                                        // (the interval *starting* at a sample instant; the last one at t_k)
  Eigen::Vector3d bg_snap = Eigen::Vector3d::Zero();    // b̂_g of the cycle holding t_g (ω̂_g was corrected with it)
  Eigen::Vector3d ba_snap = Eigen::Vector3d::Zero();    // b̂_a of the cycle ending at t_k (M1; δa of A10 is taken about it)
  Eigen::Vector3d g_snap = Eigen::Vector3d::Zero();     // ĝ^W of the cycle ending at t_k (M1)
  Eigen::Matrix3d R_k = Eigen::Matrix3d::Identity();    // R̂_k: attitude at the cycle end (last sample) (M1)
  Eigen::Vector3d a_bar = Eigen::Vector3d::Zero();      // ā_b: time-mean of BridgeSample::f_body over (t_g, t_k] (§4.2 δθ column)
  // Bridge position noise Σ_D (§4.1), W frame. query() fills the IMU white-noise term acc_n²δ³/3 plus (2 mm)² per
  // crossed cycle and NOT the (½δ²(σ_ba+gσ_θ))² term of §4.1: after A10 the (σ_ba, σ_θ, σ_g) sensitivity of D/E is
  // carried by the δθ/δb_a/δg columns of J_a/J_w through Σ, so adding it here would count it twice (≈0.15 mm at
  // δ = 0.1 s either way). This deviation from §4.1/§4.7 is registered as a request for an AMENDMENTS ruling
  // (INTERFACE_CHANGES, WP-b); if the maintainer keeps the §4.1 term, the caller adds sigmaD(δ, 0, σ_ba, σ_θ, g, 0)
  // with its own σ_ba/σ_θ from P⁻ (query() has no access to P⁻), and the Doppler row keeps §4.7's (δ(σ_ba+gσ_θ))².
  Eigen::Matrix3d Sigma_D = Eigen::Matrix3d::Zero();
  double var_v_bridge = 0.0;                            // acc_n²·δ: white-noise velocity term (same ruling as Sigma_D)
  // Appended (AMENDMENTS A61 F8; INTERFACE_CHANGES): per-axis variance of ω̂_g's gyro white noise. ω̂_g is the host's
  // angvel_avr of one IMU interval, the mean of its two raw samples, each with gyr_n²/Δt_s ⇒ gyr_n²/(2Δt_s).
  double var_omega = 0.0;
  // Nominal state at t_g expressed through the latest snapshot: R̂_k ΔR̂ᵀ, p̂_k − v̂_k δ − D̂, v̂_k − Ê (equals the direct
  // interpolation when t_g lies in the current cycle; for diagnostics / initialisation antenna samples):
  Eigen::Matrix3d R_g_hat = Eigen::Matrix3d::Identity();
  Eigen::Vector3d p_g_hat = Eigen::Vector3d::Zero(), v_g_hat = Eigen::Vector3d::Zero();
};

// Antenna quantities evaluated at an iterate x (§4.1 "y", "z").
struct AntennaAtEpoch {
  Eigen::Matrix3d R_g = Eigen::Matrix3d::Identity();
  Eigen::Vector3d p_g = Eigen::Vector3d::Zero(), v_g = Eigen::Vector3d::Zero(), omega_g = Eigen::Vector3d::Zero();
  Eigen::Vector3d y = Eigen::Vector3d::Zero();   // a^W = p_g + R_g l
  Eigen::Vector3d z = Eigen::Vector3d::Zero();   // ȧ^W = v_g + R_g [ω_g]× l
};

class ImuBridge {
 public:
  explicit ImuBridge(const Config& cfg);

  // Store one LIO cycle (t_prev, t_k]. `samples[0]` must be the cycle start (t == t_prev within 1e-9).
  // Precondition: t_k > t_prev, sample times non-decreasing, and t_prev == t_k of the previously pushed cycle (within
  // 1e-9; the host guarantees it, IMU_Processing.cpp:248/:470); a violation throws std::invalid_argument and leaves
  // the ring untouched (a glue bug, never silently corrupts the ring or chains across a gap). VIO cycles (no samples)
  // must not be pushed. The three snapshots are the host biases/gravity the samples were propagated with
  // (state_inout at UndistortPcl entry). Afterwards the oldest cycles are dropped while more than
  // `bridge_ring_cycles` are held and the oldest one ends before t_k − (max_bridge_s + 0.1 s) (A33; a cycle ending on
  // that horizon within 1e-9 s is kept), so every epoch with δ ≤ max_bridge_s stays bridgeable at any LIO rate.
  void pushCycle(const std::vector<BridgeSample>& samples, double t_prev, double t_k, const Eigen::Vector3d& bg_snap,
                 const Eigen::Vector3d& ba_snap, const Eigen::Vector3d& g_snap);
  // Bridge from t_g to the latest cycle end. Fails when t_g > t_k (epoch for the next cycle),
  // δ > max_bridge_s (LATE_EPOCH) or t_g older than the ring buffer (after A33 only before the first pushed cycle).
  bool query(double t_g, BridgeResult* out) const;
  double latestCycleEnd() const;
  double oldestTime() const;
  int cyclesStored() const;

  // Pure interpolation between two consecutive samples (U14 checks it against the host propagation).
  static void interpolate(const BridgeSample& a, const BridgeSample& b, double t, Eigen::Matrix3d* R,
                          Eigen::Vector3d* p, Eigen::Vector3d* v);
  // Apply the bridge to an iterate: antenna position/velocity in W with lever arm l^I, using the
  // first-order D(x)/E(x) of the file header (A10). `gravity_est_en=false` freezes the δg term.
  static AntennaAtEpoch evaluate(const CoreState& x, const BridgeResult& br, const Eigen::Vector3d& lever_I,
                                 bool gravity_est_en = true);
  // Σ_D from §4.1: (acc_n² δ³/3 + (½ δ² (σ_ba + g σ_θ))²) I, plus (2 mm)² per crossed cycle.
  static Eigen::Matrix3d sigmaD(double delta, double acc_n, double sigma_ba, double sigma_theta, double g, int crossed_cycles);

 private:
  struct Cycle {
    double t_prev = 0.0, t_k = 0.0;
    Eigen::Vector3d bg_snap = Eigen::Vector3d::Zero(), ba_snap = Eigen::Vector3d::Zero(), g_snap = Eigen::Vector3d::Zero();
    std::vector<BridgeSample> samples;
  };
  const Config& cfg_;
  std::deque<Cycle> ring_;
};

}  // namespace livo_gnss
