// SPDX-License-Identifier: BSD-3-Clause
// TimeOffsetMonitor (DESIGN §7, AMENDMENTS A17). Owner: WP-c. Tests: U12 (TimeOffsetStep).
//
// Weighted linear regression of accepted Doppler innovations on the antenna acceleration along the line
// of sight, ν_j = c + s·x_j + e_j, x_j = u_jᵀα_A^N, over a sliding window. The intercept c absorbs a
// common clock-drift error so that it cannot masquerade as a time offset; ŝ = −δt_off. The monitor only
// raises a flag (TOFF_WARN): the time mapping is frozen per sequence (§7) and is never changed online.
#include "internal/time_offset.hpp"

#include <cmath>

namespace livo_gnss {

namespace {
constexpr int kMinSamples = 10;
}

TimeOffsetMonitor::TimeOffsetMonitor(const Config::Time& cfg) : cfg_(cfg) {}

void TimeOffsetMonitor::addEpoch(double t_k, const std::vector<TimeOffsetSample>& rows) {
  Epoch e;
  e.t = t_k;
  e.rows.reserve(rows.size());
  for (const auto& r : rows) {
    if (!(r.var > 0.0) || !std::isfinite(r.x) || !std::isfinite(r.nu)) continue;
    e.rows.push_back(r);
  }
  window_.push_back(std::move(e));
  const double t_min = t_k - cfg_.toff_monitor.window_s;
  while (!window_.empty() && window_.front().t < t_min) window_.pop_front();
}

TimeOffsetEstimate TimeOffsetMonitor::estimate() const {
  TimeOffsetEstimate est;
  if (window_.empty()) return est;
  est.t_from = window_.front().t;
  est.t_to = window_.back().t;
  // Weighted normal equations for [c, s] with weights 1/var.
  double sw = 0.0, swx = 0.0, swxx = 0.0, swy = 0.0, swxy = 0.0;
  int n = 0;
  for (const auto& e : window_) {
    for (const auto& r : e.rows) {
      const double w = 1.0 / r.var;
      sw += w;
      swx += w * r.x;
      swxx += w * r.x * r.x;
      swy += w * r.nu;
      swxy += w * r.x * r.nu;
      ++n;
    }
  }
  est.n = n;
  if (n < kMinSamples) return est;
  const double det = sw * swxx - swx * swx;   // = sw · Σw(x − x̄)²
  const double excitation = det / sw;         // Σ w (x − x̄)²: no acceleration ⇒ no observability of s
  if (!(excitation > 1e-12 * sw)) return est;
  const double s = (sw * swxy - swx * swy) / det;
  const double c = (swy - s * swx) / sw;
  // a-posteriori unit variance; inflate the formal σ_s when the residuals are larger than the model says
  double chi2 = 0.0;
  for (const auto& e : window_) {
    for (const auto& r : e.rows) {
      const double res = r.nu - c - s * r.x;
      chi2 += res * res / r.var;
    }
  }
  const double s0sq = (n > 2) ? std::max(1.0, chi2 / static_cast<double>(n - 2)) : 1.0;
  const double var_s = s0sq * sw / det;
  est.valid = true;
  est.offset_s = -s;
  est.sigma_s = std::sqrt(var_s);
  est.warn = cfg_.toff_monitor.enable && std::fabs(s) > cfg_.toff_monitor.n_sigma * est.sigma_s &&
             std::fabs(s) > cfg_.toff_monitor.min_abs_s;
  return est;
}

void TimeOffsetMonitor::reset() { window_.clear(); }

}  // namespace livo_gnss
