// SPDX-License-Identifier: BSD-3-Clause
// IntegrityMonitor — external judges that only set flags, never touch the host P (DESIGN §10 L7 and the
// degeneracy row):
//   * velocity alarm: GNSS-only Doppler speed vs LIVO predicted speed, d² > χ²₁(0.999) and |Δ| > 0.2 m/s
//     for 5 consecutive epochs ⇒ LIVO_ALARM (updates switch to Schmidt: host columns of K zeroed) until
//     10 consecutive consistent epochs;
//   * heading: horizontal velocity direction error > max(5°, 3σ) for 20 epochs ⇒ REINIT_YAW;
//   * degeneracy index: λ₁ of the translation Schur complement of the LIO information gain
//     (P⁺)⁻¹ − (P⁻)⁻¹ [θ,p] block; < 400 m⁻² for 10 frames ⇒ DEGENERATE (hysteresis 800); floor only when
//     degeneracy.floor_enable (probe arm T+floor) — the floor is applied by the caller, not here.
// Speeds (not velocities) for the alarm: χ²₁ as §10 L7 writes it, frame-free, so a yaw error cannot raise it (the heading
// judge owns ψ). reinit_yaw is a one-shot edge like alarm_changed/degenerate_changed: set on the epoch the heading count
// reaches n_consec, cleared by clearEdges(). The heading judge counts only epochs with both horizontal speeds ≥ min_speed
// and no LIVO_ALARM. The degeneracy index needs both hysteresis counts (`window` frames below lambda_enter to set,
// `window` frames above lambda_exit to clear). integrity.cpp has the details.
// Owner: P3-A1 (OWNERSHIP §2c). Tests: test_u17_integrity.cpp (alarm hysteresis, heading, degeneracy index, Schmidt
// switching; A9), test_u15_mc.cpp.
#pragma once

#include <Eigen/Core>

#include "livo_gnss/config.hpp"
#include "livo_gnss/core_state.hpp"
#include "livo_gnss/diagnostics.hpp"

namespace livo_gnss {

struct IntegrityFlags {
  bool livo_alarm = false;
  bool reinit_yaw = false;
  bool degenerate = false;
  bool alarm_changed = false, degenerate_changed = false;   // edge for events
  double d2_velocity = 0.0;          // last velocity test statistic
  double heading_err_deg = 0.0;
  Eigen::Vector3d lambda = Eigen::Vector3d::Zero();         // λ₁..₃
  Eigen::Vector3d u1 = Eigen::Vector3d::Zero();
};

class IntegrityMonitor {
 public:
  explicit IntegrityMonitor(const Config::Integrity& cfg);

  // Velocity judge input: GNSS-only Doppler velocity (N frame) with covariance vs LIVO-predicted
  // antenna velocity w^N with its covariance. Call once per epoch that has a Doppler LS solution.
  void onVelocity(const Eigen::Vector3d& v_gnss_N, const Eigen::Matrix3d& cov_gnss, const Eigen::Vector3d& w_N,
                  const Eigen::Matrix3d& cov_w, double sigma_yaw_rad);
  // Degeneracy judge input: host prior/posterior covariance of the LIO update (H2 LIO call site).
  void onLioCovariance(const HostMat& P_prior, const HostMat& P_post);
  void reset();
  const IntegrityFlags& flags() const { return flags_; }
  // Clear the edge bits after the events were logged.
  void clearEdges();

 private:
  Config::Integrity cfg_;
  IntegrityFlags flags_;
  int n_bad_vel_ = 0, n_good_vel_ = 0, n_bad_heading_ = 0, n_low_lambda_ = 0, n_high_lambda_ = 0;
};

}  // namespace livo_gnss
