// SPDX-License-Identifier: BSD-3-Clause
// TimeOffsetMonitor — DESIGN §7 "在线:多普勒残差对 u_jᵀα_A 的滑窗回归斜率作残余时偏诊断,>3σ 告警不自动改值",
// given its own module in AMENDMENTS A17 (the verifier found it declared nowhere; U12 and I10 need it).
// A residual time offset δt_off between the GNSS and the estimator timeline makes every Doppler innovation
// carry −(u_jᵀ α_A^N)·δt_off (antenna acceleration along the line of sight). Over a sliding window the
// weighted regression ν_j = s·x_j + e_j, x_j = u_jᵀα_A^N, gives ŝ = −δt_off and its σ; |ŝ| > n_sigma·σ and
// |ŝ| > min_abs_s ⇒ TOFF_WARN (flag + event only; the mapping is never changed online).
// Owner: WP-c. Tests: test_u12_gating.cpp (TimeOffsetStep: an injected ±50 ms step is flagged within one window).
#pragma once

#include <deque>
#include <vector>

#include "livo_gnss/config.hpp"

namespace livo_gnss {

struct TimeOffsetSample {
  double x = 0.0;      // u_jᵀ α_A^N [m/s²]
  double nu = 0.0;     // Doppler innovation [m/s] (accepted rows only)
  double var = 1.0;    // its S_ii [m²/s²] (weight 1/var)
};

struct TimeOffsetEstimate {
  bool valid = false;  // enough samples and excitation (Σ w x² > 0)
  double offset_s = 0.0;   // −ŝ
  double sigma_s = 0.0;
  int n = 0;
  double t_from = 0.0, t_to = 0.0;
  bool warn = false;
};

class TimeOffsetMonitor {
 public:
  explicit TimeOffsetMonitor(const Config::Time& cfg);
  // One call per epoch that had a Doppler update; `t_k` in estimator time. Samples older than window_s are dropped.
  void addEpoch(double t_k, const std::vector<TimeOffsetSample>& rows);
  TimeOffsetEstimate estimate() const;
  void reset();
  int epochs() const { return static_cast<int>(window_.size()); }

 private:
  struct Epoch {
    double t = 0.0;
    std::vector<TimeOffsetSample> rows;
  };
  Config::Time cfg_;
  std::deque<Epoch> window_;
};

}  // namespace livo_gnss
