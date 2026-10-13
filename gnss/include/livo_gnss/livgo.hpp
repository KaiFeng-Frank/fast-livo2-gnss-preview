// SPDX-License-Identifier: BSD-3-Clause
// Online speed-magnitude DTW for the FAST-LIVGO arm (arXiv:2606.19190 v2, §III-C1).
// The paper does not publish a grid, a band, or a minimum length. This uses a 10 Hz grid (the
// receiver epoch rate), a Sakoe–Chiba band of 2 s (wider than the 0.03–0.15 s offline dtw_offset
// already stored in the M3DGR time-calibration audits), and refuses overlaps shorter than 3 s.
// The returned lag is added to TimeMap's offset: a positive GNSS delay (the same motion appears
// later on the GNSS stamps) comes back negative, so estimator time of those stamps moves earlier.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <set>
#include <string>
#include <vector>

#include "livo_gnss/config.hpp"
#include "livo_gnss/types.hpp"

namespace livo_gnss {

// AMENDMENTS A80 (LIVGO-literal): the grid, the window and the band of livgoDtwLag as arguments. The defaults are the numbers the
// function had before (10 Hz grid, last 40 s of the overlap, Sakoe–Chiba band 2 s), so a caller that passes nothing is unchanged.
//   grid_s    resampling step of the two speed series [s]
//   window_s  the DTW runs on at most the last window_s seconds of the overlap of the two series (n_max = window_s / grid_s samples)
//   band_s    Sakoe–Chiba band half-width [s] (the band in samples is round(band_s / grid_s))
// The overlap must be at least 3 s and the grid at least 20 samples long, whatever the window.
struct LivgoDtwParams {
  double grid_s = 0.1;
  double window_s = 40.0;
  double band_s = 2.0;
};

// Parameters the paper does not put on the shared Config, so sizeof(Config) and the T-arm object
// code stay as they were. The ROS glue fills this from gnss/livgo/* and passes it only to GnssFusionLivgo.
struct LivgoConfig {
  bool enable = false;
  // III-C2 solves ^E R_G in SO(3). false keeps activate() (yaw, phi = 0, the configured lever) as the control.
  bool paper_init = true;
  // T-dualrej: only the IV-E Hessian switch. Pseudorange, the frame random walk, DTW and the paper init stay as T.
  bool dual_only = false;
  double el_min_deg = 15.0;          // paper: elevation < 15° excluded
  double cn0_min_dbhz = 35.0;        // paper reference; not applied as a hard mask on this receiver
  // Paper λ_th is not given. 0 means every non-negative window mean is Mode A, so a collection run can
  // record the distribution. The scored yaml sets the registered open-set percentile (see the analysis note).
  double lambda_th = 0.0;
  int hessian_window = 10;
  int dtw_min_epochs = 30;
  std::string hessian_log;           // livgo_hessian.csv; empty disables the trace

  // ---- LIVGO-v2 (AMENDMENTS A75 §6). Every default below is the behaviour before A75, so LIVGO-legacy.yaml and the
  // other existing arms are unchanged byte for byte. ----
  // The registration tilt and translation are exact constants after ACTIVATE (no variance, no covariance, no process
  // noise: they leave every S). The yaw stays a constant *state* — activation variance, no process noise, updated by the
  // GNSS rows and no longer a consider parameter. A fixed-anchor TDCP row then shares the registration and the host
  // gravity with its anchor (the anchor is the k0 antenna in W, mapped through the iterate's registration), see
  // internal/livgo_anchor.hpp. false = the pin of the legacy arm (marginal variances kept, cross terms dropped).
  bool frame_exact = false;
  // Which Hessian block drives the dual mode: "min" = the weaker of the LIO and VIO translation blocks (legacy),
  // "lio" = the LiDAR translation block only. Read by the host glue (src/LIVMapper.cpp), not by the facade.
  enum class LambdaSource { Min, Lio };
  LambdaSource lambda_source = LambdaSource::Min;
  // false: no online DTW; the time map keeps the offline calibration only.
  bool dtw_online = true;
  // Row noise of the Doppler and TDCP rows. Model = the A58 noise model (elevation, C/N0, receiver std, kappa_AR,
  // noise.scale, sigma_toff). Flat = one fixed sigma per row (flat_sigma_*) plus the terms of our own time architecture
  // that no state column carries — the IMU bridge between the epoch instant and the filter instant (TDCP: the k1
  // antenna position noise u1' M Sigma_D M' u1 and its cross terms between satellites; Doppler: acc_n^2 delta, the gyro
  // noise through the lever arm with its cross terms, and the clock-drift term S_d delta), all unscaled.
  // Dropped in flat mode: elevation, C/N0, receiver std, kappa_AR, noise.scale, sigma_toff, TDCP rate and wind-up terms,
  // the Doppler lever-arm sigma and registration-drift terms. The GNSS-only estimators (RAIM, DTW, initialisation) keep the
  // model.
  enum class NoiseMode { Model, Flat };
  NoiseMode noise_mode = NoiseMode::Model;
  double flat_sigma_tdcp_m = 0.001;   // paper: "sub-millimetre to millimetre" relative constraint
  double flat_sigma_dopp_mps = 0.05;

  // ---- AMENDMENTS A80 (LIVGO-literal). Every default below is the behaviour before A80: LIVGO-legacy, LIVGO-v2 and the other
  // existing arms are unchanged byte for byte. ----
  // frame_exact only: the yaw is an exact constant too after ACTIVATE (the frame is solved once and then frozen: tilt, yaw and
  // translation are numbers, none is a state with a variance). false = A75: the yaw stays a constant state updated by the GNSS rows.
  bool freeze_yaw = false;
  // paper_init only. > 0: the first activation waits until the host trajectory has moved this far horizontally (the farthest
  // horizontal distance from the first collected epoch; "horizontal" = orthogonal to the host gravity), and the eq. (2)-(3)
  // least squares then runs once on every synchronous epoch collected up to that moment (the epochs that passed the section-8 SPP
  // test, each with the host pose at its time) instead of the initializer's sliding window. 0 = the sliding window (A73).
  double init_travel_m = 0.0;
  // paper_init only. > 0: when the formal tilt sigma of the least squares (the larger of the two horizontal rotation axes, deg)
  // exceeds this, the tilt is taken from the host gravity (phi = 0) and only the least squares' yaw, translation and lever are
  // kept; the event log says which branch ran and the sigma. 0 = no check, the least-squares tilt is always taken.
  double init_tilt_sigma_max_deg = 0.0;
  // true: the C/N0 masks of the pseudorange, Doppler and carrier rows are all cn0_min_dbhz and the elevation mask is el_min_deg
  // (the paper's hard RAIM pre-screen). false: the masks of the yaml (A58 floors); el_min_deg then only raises a looser mask.
  bool cn0_hard = false;
  // true: a satellite whose first signal is not its system's primary one (GPS/QZS/GLO L1C, GAL L1X, BDS L2I = B1I) carries no row
  // at all. The I/O layer puts the primary signal first but keeps a satellite tracked only on a secondary band at sig[0], and the
  // core's L0 only refuses the geometry-free-only codes; the paper uses the main frequency of each system only.
  bool primary_only = false;
  // The online DTW's grid and window (LivgoDtwParams; the band stays at 2 s). dtw_min_epochs (above) is its minimum sample count.
  double dtw_grid_s = 0.1;
  double dtw_window_s = 40.0;

  // ---- AMENDMENTS A81 (development-set calibration of LIVGO-literal). Default = the behaviour before A81. ----
  // true: a satellite that the GNSS-only robust least squares (the arm's RAIM) rejected at the epoch k0 that opens a TDCP pair does
  // not enter the pair. The closing end k1 already refuses a satellite the k1 epoch's least squares rejected (RejectReason::L1);
  // without this switch the start end looks at the carrier mask only, so "rejected satellites are not observations" held at one end.
  bool tdcp_raim_both_ends = false;

  // ---- AMENDMENTS A82 (the initialisation in the paper's order; LIVGO-literal). Default = the behaviour before A82. ----
  // true (needs dtw_online, paper_init and init_travel_m > 0): the online DTW's "after dtw_min_epochs epochs" trigger is replaced by one
  // DTW on every synchronous epoch of the frame least squares' history, run when the travel gate opens, right before that least
  // squares (the paper's order: DTW, compensate the time offset, then eq. (2)-(3)). The grid is dtw_grid_s; the window is the whole
  // history (dtw_window_s and dtw_min_epochs are not read); the band stays 2 s. The result is taken only when the acceptance test of
  // the author's public code passes (livgoDtwInit, below); otherwise the offset stays 0. When taken, the lag goes into the GNSS time
  // map and the history's host poses are re-paired at the compensated times before the least squares.
  bool dtw_at_init = false;
  // true (full arm): after the first ACTIVATE no re-initialisation runs (REINIT_YAW, the outage/L6/L7 re-initialisation, the H2
  // conditioning fallback); the frame stays as the first least squares left it until the end of the run. Every suppressed trigger is
  // an event REINIT_SUPPRESSED.
  bool single_init = false;


  // ---- AMENDMENTS A75 revision 1 (prediction P2). Observer only: with tdcp_gain_log the facade runs every epoch with a closing
  // TDCP pair a second time on copies of its state, without the TDCP rows, and writes the host position variance of the two
  // posteriors to tdcp_gain_log_path (livgo_tdcp_gain.csv). Nothing in the filter reads it; false = nothing computed, no file. ----
  bool tdcp_gain_log = false;
  std::string tdcp_gain_log_path;    // filled by the ROS glue (output dir) when tdcp_gain_log is true
};

// AMENDMENTS A80 livgo.primary_only: the primary (main-frequency) signal of each system, the one gnss_io/signal_order.hpp puts first
// (GPS/QZS/GLO L1C, GAL L1X, BDS L2I = B1I). Written out here because the core does not depend on gnss_io.
inline bool livgoIsPrimarySignal(Sys sys, SignalCode code) {
  switch (sys) {
    case Sys::GPS:
    case Sys::QZS:
    case Sys::GLO: return code == SignalCode::L1C;
    case Sys::GAL: return code == SignalCode::L1X;
    case Sys::BDS: return code == SignalCode::L2I;
    default: return false;
  }
}

// The L0 masks the LIVGO facade runs with. cn0_hard (A80): the C/N0 masks of the pseudorange, Doppler and carrier rows are all
// cn0_min_dbhz and the elevation mask is el_min_deg — assigned, the yaml's values are replaced. Without it (every arm before A80)
// the C/N0 masks are the yaml's and el_min_deg only raises a looser elevation mask.
inline void livgoApplyMasks(const LivgoConfig& l, Config::Masks* m) {
  if (l.cn0_hard) {
    m->cn0_psr = m->cn0_dopp = m->cn0_cp = l.cn0_min_dbhz;
    m->el_deg = l.el_min_deg;
  } else if (m->el_deg < l.el_min_deg) {
    m->el_deg = l.el_min_deg;
  }
}

// AMENDMENTS A81 livgo.tdcp_raim_both_ends: the members of the TDCP pair that opens at an epoch, as indices into that epoch's
// satellites. `usable[i]` is what the pair always required of a start member (geometry available and the carrier mask passed);
// `rtk_sat[i]` the satellite's id and `l1_rejected` the ids the epoch's robust least squares rejected. With the switch on a
// rejected satellite is no member; with it off the result is the usable satellites in order, as before A81.
inline std::vector<int> livgoTdcpStartMembers(bool both_ends, const std::vector<char>& usable, const std::vector<int>& rtk_sat,
                                              const std::set<int>& l1_rejected) {
  std::vector<int> out;
  for (size_t i = 0; i < usable.size() && i < rtk_sat.size(); ++i) {
    if (!usable[i]) continue;
    if (both_ends && l1_rejected.count(rtk_sat[i])) continue;
    out.push_back(static_cast<int>(i));
  }
  return out;
}

inline bool livgoDtwLag(const std::vector<double>& tg, const std::vector<double>& vg,
                        const std::vector<double>& th, const std::vector<double>& vh, double* lag_add_s,
                        const LivgoDtwParams& prm = LivgoDtwParams()) {
  if (!lag_add_s || tg.size() != vg.size() || th.size() != vh.size() || tg.size() < 5 || th.size() < 5) return false;
  if (!(std::isfinite(prm.grid_s) && prm.grid_s > 0.0 && std::isfinite(prm.window_s) && std::isfinite(prm.band_s) && prm.band_s > 0.0))
    return false;
  const double t0 = std::max(tg.front(), th.front());
  const double t1 = std::min(tg.back(), th.back());
  if (!(t1 - t0 >= 3.0)) return false;
  const double dt = prm.grid_s;
  int n = static_cast<int>((t1 - t0) / dt);
  if (n < 20) return false;
  const int n_max = static_cast<int>(std::floor(prm.window_s / dt + 1e-9));   // default 40 s / 0.1 s = 400
  if (n_max < 20) return false;
  if (n > n_max) n = n_max;   // last window_s of the overlap; the init window is the part that matters
  const double grid0 = t1 - n * dt;
  auto resample = [&](const std::vector<double>& t, const std::vector<double>& v, std::vector<double>* o) {
    o->assign(static_cast<size_t>(n), 0.0);
    size_t j = 0;
    for (int i = 0; i < n; ++i) {
      const double ti = grid0 + (static_cast<double>(i) + 0.5) * dt;
      while (j + 1 < t.size() && t[j + 1] < ti) ++j;
      if (j + 1 >= t.size() || !(t[j + 1] > t[j])) {
        (*o)[static_cast<size_t>(i)] = v[j];
        continue;
      }
      const double u = std::min(1.0, std::max(0.0, (ti - t[j]) / (t[j + 1] - t[j])));
      (*o)[static_cast<size_t>(i)] = (1.0 - u) * v[j] + u * v[j + 1];
    }
  };
  std::vector<double> ag, ah;
  resample(tg, vg, &ag);
  resample(th, vh, &ah);
  auto norm = [](std::vector<double>* a) {
    double m = 0.0;
    for (double x : *a) m += x;
    m /= static_cast<double>(a->size());
    double s2 = 0.0;
    for (double x : *a) s2 += (x - m) * (x - m);
    double s = std::sqrt(s2 / static_cast<double>(a->size()));
    if (!(s > 1e-6)) s = 1.0;
    for (double& x : *a) x = (x - m) / s;
  };
  norm(&ag);
  norm(&ah);

  const int W = std::max(1, static_cast<int>(std::lround(prm.band_s / dt)));   // default 2 s at 10 Hz = 20
  const int stride = 2 * W + 1;
  const double kInf = 1e300;
  std::vector<double> cost(static_cast<size_t>(n * stride), kInf);
  std::vector<uint8_t> pred(static_cast<size_t>(n * stride), 0);
  auto at = [&](int i, int j) -> int {
    const int k = j - (i - W);
    if (j < 0 || j >= n || k < 0 || k >= stride) return -1;
    return i * stride + k;
  };
  auto dist = [&](int i, int j) {
    const double d = ag[static_cast<size_t>(i)] - ah[static_cast<size_t>(j)];
    return d * d;
  };
  for (int j = 0; j < n && j <= W; ++j) {
    const int p = at(0, j);
    if (p >= 0) cost[static_cast<size_t>(p)] = dist(0, j);
  }
  for (int i = 1; i < n; ++i) {
    const int j0 = std::max(0, i - W);
    const int j1 = std::min(n - 1, i + W);
    for (int j = j0; j <= j1; ++j) {
      const int p = at(i, j);
      if (p < 0) continue;
      const double d = dist(i, j);
      double best = kInf;
      uint8_t how = 0;
      const int diag = at(i - 1, j - 1);
      const int up = at(i - 1, j);
      const int left = at(i, j - 1);
      if (diag >= 0 && cost[static_cast<size_t>(diag)] < best) { best = cost[static_cast<size_t>(diag)]; how = 1; }
      if (up >= 0 && cost[static_cast<size_t>(up)] < best) { best = cost[static_cast<size_t>(up)]; how = 2; }
      if (left >= 0 && cost[static_cast<size_t>(left)] < best) { best = cost[static_cast<size_t>(left)]; how = 3; }
      if (best >= kInf / 2) continue;
      cost[static_cast<size_t>(p)] = d + best;
      pred[static_cast<size_t>(p)] = how;
    }
  }
  int jbest = -1;
  double cbest = kInf;
  const int j0 = std::max(0, n - 1 - W);
  const int j1 = std::min(n - 1, n - 1 + W);
  for (int j = j0; j <= j1; ++j) {
    const int p = at(n - 1, j);
    if (p >= 0 && cost[static_cast<size_t>(p)] < cbest) { cbest = cost[static_cast<size_t>(p)]; jbest = j; }
  }
  if (jbest < 0) return false;
  std::vector<double> lags;
  lags.reserve(static_cast<size_t>(n));
  for (int i = n - 1, j = jbest; i >= 0 && j >= 0;) {
    lags.push_back(static_cast<double>(j - i) * dt);
    const int p = at(i, j);
    if (p < 0) break;
    const uint8_t how = pred[static_cast<size_t>(p)];
    if (how == 1) { --i; --j; }
    else if (how == 2) { --i; }
    else if (how == 3) { --j; }
    else break;
  }
  if (lags.size() < 10) return false;
  std::nth_element(lags.begin(), lags.begin() + static_cast<std::ptrdiff_t>(lags.size() / 2), lags.end());
  *lag_add_s = lags[lags.size() / 2];
  return std::isfinite(*lag_add_s);
}

// =====================================================================================================================================
// AMENDMENTS A82: the time alignment in the paper's order, with the acceptance test of the author's public code
// (FAST-LIVO2-RTK, src/optimization.cpp: normalizeVelocitySeries, interpolateLinear, scoreVelocityOffset, and the acceptance block of
// optimization::initialAlign). The functions below are ports of those, same operations in the same order.
//
// SIGN. `lag` is what livgoDtwLag returns and what the facade adds to the GNSS time map (t_est' = t_est + lag): the GNSS sample stamped
// t carries the motion the host had at t + lag, so a GNSS stream that is late by d comes back as lag = -d. The author's `fine_offset`
// is the same number with the same use: scoreVelocityOffset reads the GNSS series at (host time - fine_offset), it is largest when
// gps(t_host - f) = host(t_host), i.e. host time = GNSS stamp + f, and initialAlign then does gps.stamp += f. So fine_offset == lag,
// no sign change (pinned by LivgoA82.LagAndAuthorsFineOffsetAreTheSameNumber).
//
// One difference to the author's series, by A82's wording ("the two speed series are defined as in the online DTW"): the author's
// builders (buildVelocitySeriesFromSamples) smooth each series with a centred 5-sample mean before normalising; these series are the
// raw speed magnitudes the online DTW already uses, not smoothed.
// =====================================================================================================================================
constexpr double kLivgoSeriesMinStd = 0.05;       // author: normalizeVelocitySeries, stddev < 0.05 m/s refuses the series
constexpr size_t kLivgoSeriesMinSamples = 20;     // author: normalizeVelocitySeries, fewer than 20 samples refuses it
constexpr int kLivgoScoreMinPairs = 30;           // author: scoreVelocityOffset, fewer than 30 pairs scores -inf
constexpr double kLivgoScoreMin = 0.25;           // author: initialAlign, best_velocity_score > 0.25
constexpr double kLivgoScoreGain = 0.05;          // author: initialAlign, (best - zero) > 0.05
constexpr double kLivgoLagMaxS = 10.0;            // author: initialAlign, |fine_time_offset| <= 10 s

// Population standard deviation as normalizeVelocitySeries computes it (sequential sums); NaN for an empty series.
inline double livgoSeriesStd(const std::vector<double>& v) {
  if (v.empty()) return std::numeric_limits<double>::quiet_NaN();
  double sum = 0.0;
  for (double x : v) sum += x;
  const double mean = sum / static_cast<double>(v.size());
  double sq_sum = 0.0;
  for (double x : v) {
    const double d = x - mean;
    sq_sum += d * d;
  }
  return std::sqrt(sq_sum / static_cast<double>(v.size()));
}

// Author's normalizeVelocitySeries: false (series untouched) for fewer than 20 samples or a non-finite or < 0.05 standard deviation;
// otherwise the series is replaced by its z-scores. `sd_out` (if given) receives the standard deviation whenever the series is not
// empty, accepted or not.
inline bool livgoNormalizeSeries(std::vector<double>* v, double* sd_out = nullptr) {
  if (sd_out) *sd_out = std::numeric_limits<double>::quiet_NaN();
  if (!v || v->empty()) return false;
  const double sd = livgoSeriesStd(*v);
  if (sd_out) *sd_out = sd;
  if (v->size() < kLivgoSeriesMinSamples) return false;
  if (!std::isfinite(sd) || sd < kLivgoSeriesMinStd) return false;
  double sum = 0.0;
  for (double x : *v) sum += x;
  const double mean = sum / static_cast<double>(v->size());
  for (double& x : *v) x = (x - mean) / sd;
  return true;
}

// Author's interpolateLinear (times ascending): false for fewer than two samples, t outside [front, back], or a zero-width interval.
inline bool livgoInterpolateLinear(const std::vector<double>& t, const std::vector<double>& v, double tq, double* out) {
  if (t.size() < 2 || t.size() != v.size() || tq < t.front() || tq > t.back()) return false;
  auto upper = std::upper_bound(t.begin(), t.end(), tq);
  if (upper == t.begin()) {
    *out = v.front();
    return true;
  }
  if (upper == t.end()) {
    *out = v.back();
    return true;
  }
  const size_t i = static_cast<size_t>(upper - t.begin());
  const double t0 = t[i - 1];
  const double t1 = t[i];
  if (t1 <= t0) return false;
  const double a = (tq - t0) / (t1 - t0);
  *out = v[i - 1] * (1.0 - a) + v[i] * a;
  return true;
}

// Author's scoreVelocityOffset on two already normalised series: for each host ("slam") sample the GNSS series is read at
// (host time - fine_offset); the score is sum(s g) / sqrt(sum(s s) sum(g g)) over the pairs found. Fewer than 30 pairs (or an
// empty sum) is -inf. `pair_count` receives the pairs.
inline double livgoScoreVelocityOffset(const std::vector<double>& slam_t, const std::vector<double>& slam_v,
                                       const std::vector<double>& gps_t, const std::vector<double>& gps_v, double fine_offset,
                                       int* pair_count = nullptr) {
  double sum_sg = 0.0, sum_ss = 0.0, sum_gg = 0.0;
  int pairs = 0;
  for (size_t i = 0; i < slam_t.size() && i < slam_v.size(); ++i) {
    double gps_value = 0.0;
    if (!livgoInterpolateLinear(gps_t, gps_v, slam_t[i] - fine_offset, &gps_value)) continue;
    const double slam_value = slam_v[i];
    sum_sg += slam_value * gps_value;
    sum_ss += slam_value * slam_value;
    sum_gg += gps_value * gps_value;
    ++pairs;
  }
  if (pair_count) *pair_count = pairs;
  if (pairs < kLivgoScoreMinPairs || sum_ss <= 1e-12 || sum_gg <= 1e-12) return -std::numeric_limits<double>::infinity();
  return sum_sg / std::sqrt(sum_ss * sum_gg);
}

// The acceptance block of the author's initialAlign, on the three numbers it reads: the lag (fine_time_offset), the score at it
// (best_velocity_score) and the zero-lag score. Taken only if the lag is finite and |lag| <= 10 s, the score is finite and > 0.25, and
// the zero-lag score is not finite (-inf: fewer than 30 pairs) or lower by more than 0.05. (The author also takes the lag when the two
// clocks' epochs differ by whole seconds; that case does not exist here, host and GNSS stamps share one clock.) `reason` receives which
// condition refused, or "adopted".
inline bool livgoDtwAccept(double lag, double score_lag, double score_zero, const char** reason = nullptr) {
  auto why = [&](const char* s) {
    if (reason) *reason = s;
  };
  if (!std::isfinite(lag) || std::fabs(lag) > kLivgoLagMaxS) {
    why("lag_out_of_range");
    return false;
  }
  if (!(std::isfinite(score_lag) && score_lag > kLivgoScoreMin)) {
    why("score_at_lag_not_above_0.25");
    return false;
  }
  const bool improved = !std::isfinite(score_zero) || (score_lag - score_zero) > kLivgoScoreGain;
  if (!improved) {
    why("no_gain_over_zero_lag");
    return false;
  }
  why("adopted");
  return true;
}

// What the DTW at the frame initialisation found, and whether the acceptance test took it.
struct LivgoDtwInit {
  bool dtw_ok = false;                                     // livgoDtwLag returned a lag
  double lag_s = std::numeric_limits<double>::quiet_NaN(); // the DTW lag (the number the time map would get)
  int n = 0;                                               // samples of each series
  double sd_gnss = std::numeric_limits<double>::quiet_NaN(), sd_host = std::numeric_limits<double>::quiet_NaN();   // m/s
  double score_lag = std::numeric_limits<double>::quiet_NaN(), score_zero = std::numeric_limits<double>::quiet_NaN();
  int pairs_lag = 0, pairs_zero = 0;
  bool adopted = false;                                    // all conditions held
  double applied_lag_s = 0.0;                              // lag_s when adopted, else 0
  const char* reason = "";                                 // why not adopted ("adopted" when it was)
};

// One DTW on a speed history (A82 one 1 and 2). `t` are the epochs' estimator times (ascending, shared by both series), `v_gnss` the GNSS
// Doppler speed magnitudes and `v_host` the host antenna speed magnitudes at them. The DTW runs on the whole history (window = the
// span), grid `grid_s`, band 2 s. Adopted only if: DTW produced a lag; both series have at least 20 samples and a standard deviation of
// at least 0.05 m/s; |lag| <= 10 s; the author's score at the lag is > 0.25 and (zero-lag score non-finite or) exceeds the zero-lag
// score by more than 0.05. The host series plays the author's "slam" role, the GNSS series the "gps" role.
inline LivgoDtwInit livgoDtwInit(const std::vector<double>& t, const std::vector<double>& v_gnss, const std::vector<double>& v_host,
                                 double grid_s) {
  LivgoDtwInit r;
  r.n = static_cast<int>(t.size());
  if (t.empty() || v_gnss.size() != t.size() || v_host.size() != t.size()) {
    r.reason = "no_series";
    return r;
  }
  LivgoDtwParams prm;
  prm.grid_s = grid_s;
  prm.window_s = (t.back() - t.front()) + 2.0 * grid_s;   // every sample of the history; band_s stays the default 2 s
  double lag = 0.0;
  r.dtw_ok = livgoDtwLag(t, v_gnss, t, v_host, &lag, prm);
  if (r.dtw_ok) r.lag_s = lag;
  std::vector<double> g = v_gnss, h = v_host;
  const bool ok_g = livgoNormalizeSeries(&g, &r.sd_gnss);
  const bool ok_h = livgoNormalizeSeries(&h, &r.sd_host);
  if (!r.dtw_ok) {
    r.reason = "dtw_failed";
    return r;
  }
  if (!ok_g || !ok_h) {
    r.reason = "series_short_or_flat";
    return r;
  }
  r.score_lag = livgoScoreVelocityOffset(t, h, t, g, lag, &r.pairs_lag);
  r.score_zero = livgoScoreVelocityOffset(t, h, t, g, 0.0, &r.pairs_zero);
  r.adopted = livgoDtwAccept(lag, r.score_lag, r.score_zero, &r.reason);
  if (r.adopted) r.applied_lag_s = lag;
  return r;
}

// AMENDMENTS A82 livgo.single_init: the re-initialisation branches of the facade, behind one gate. Before the first ACTIVATE (or with
// the switch off) every branch is allowed; afterwards, with the switch on, none is: allow() returns false and counts the trigger.
enum class LivgoReinitBranch : int { HeadingYaw = 0, Outage = 1, ConditioningFailed = 2, Count = 3 };

class LivgoReinitGate {
 public:
  void setSingleInit(bool on) { single_init_ = on; }
  void onActivated() { activated_ = true; }
  bool singleInit() const { return single_init_; }
  bool activated() const { return activated_; }
  // true: the re-initialisation may run. false: suppressed (and counted).
  bool allow(LivgoReinitBranch b) {
    if (!single_init_ || !activated_) return true;
    ++n_[static_cast<size_t>(b)];
    ++total_;
    return false;
  }
  int suppressed(LivgoReinitBranch b) const { return n_[static_cast<size_t>(b)]; }
  int total() const { return total_; }
  static const char* name(LivgoReinitBranch b) {
    switch (b) {
      case LivgoReinitBranch::HeadingYaw: return "heading_yaw";
      case LivgoReinitBranch::Outage: return "outage";
      case LivgoReinitBranch::ConditioningFailed: return "conditioning_failed";
      default: return "?";
    }
  }

 private:
  bool single_init_ = false;
  bool activated_ = false;
  int n_[static_cast<size_t>(LivgoReinitBranch::Count)] = {0, 0, 0};
  int total_ = 0;
};

}  // namespace livo_gnss
