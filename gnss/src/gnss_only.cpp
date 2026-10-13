// SPDX-License-Identifier: BSD-3-Clause
// gnss_only (DESIGN §8 COLLECT, §10 L1, §11 RECOVERY clock reset). Owner: WP-c (A9-1). Tests: U16, U13 (P3).
//
// The only place where a robust kernel exists. Every decision is still 0/1 membership: satellites are used
// or rejected, never continuously down-weighted in what leaves this file. The robust core is
//   plain Gauss–Newton (cold start supported) → least-median-of-squares consensus over minimal 4-satellite
//   subsets (high breakdown; the gross outliers it excludes are re-admitted when they are consistent with the
//   consensus solution) → IRLS-Huber on the consensus with the a-priori σ → iterated Baarda w-test.
// Why the LMS stage in front of the Huber IRLS the design names: Huber's objective is convex, so its
// M-estimate is the same from any start and it is leverage-blind — a 30 m blunder on a 15–23° satellite
// moves the solution by ~35 m and the w-test then throws out clean satellites (verified in U16). Only a
// high-breakdown membership decision removes that; it costs ≤ 495 4×4 solves per epoch (≤ 12 base satellites).
// RTKLIB pntpos is not a judge here (its χ² return is commented out, pntpos.c:384; raim_fde is single-fault).
#include "internal/gnss_only.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <numeric>
#include <random>

#include <Eigen/Dense>

namespace livo_gnss {

namespace {

constexpr double kCLight = 299792458.0;
constexpr double kOmgE = 7.2921151467e-5;
constexpr double kFreqL1 = 1.57542e9;
constexpr double kRelHumi = 0.7;
constexpr double kHuberK = 1.345;                    // 95 % efficiency at the normal
constexpr double kWThreshold = 3.2905267314919255;   // Baarda α = 1e-3 two-sided (same level as L4)
constexpr double kConsensus = 3.0;                   // LMS consensus cut on the robust scale
constexpr double kMinEarthRadius = 6.0e6;            // below this the iterate is not a place on Earth yet
constexpr double kMadToSigma = 1.4826;

double rangeWithSagnac(const Eigen::Vector3d& rs, const Eigen::Vector3d& rr, Eigen::Vector3d* e) {
  const Eigen::Vector3d d = rs - rr;
  const double r = d.norm();
  if (e) *e = d / r;
  return r + kOmgE * (rs.x() * rr.y() - rs.y() * rr.x()) / kCLight;   // == geodist rtkcmn.c:3599
}

double median(std::vector<double> v) {
  if (v.empty()) return 0.0;
  std::sort(v.begin(), v.end());
  const size_t n = v.size();
  return (n % 2 == 1) ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
}
double mad(const std::vector<double>& v, double med) {
  std::vector<double> d;
  d.reserve(v.size());
  for (double x : v) d.push_back(std::fabs(x - med));
  return median(d);
}

// ISB column index of a system in the SPP state ([GLO, GAL, BDS, QZS] order of SppSolution::isb_m); −1 = GPS clock.
int isbIndex(Sys sys, bool qzs_as_gps) {
  switch (sys) {
    case Sys::GLO: return 0;
    case Sys::GAL: return 1;
    case Sys::BDS: return 2;
    case Sys::QZS: return qzs_as_gps ? -1 : 3;
    default: return -1;
  }
}

bool allModelsFixed(const std::vector<SppInput>& in) {
  if (in.empty()) return true;
  for (const auto& s : in)
    if (!s.models_fixed) return false;
  return true;
}

// Weighted normal equations. Returns false when singular.
bool solveNormal(const Eigen::MatrixXd& H, const Eigen::VectorXd& v, const Eigen::VectorXd& w, Eigen::VectorXd* dx,
                 Eigen::MatrixXd* Qx) {
  const Eigen::MatrixXd Nm = H.transpose() * w.asDiagonal() * H;
  Eigen::LDLT<Eigen::MatrixXd> ldlt(Nm);
  if (ldlt.info() != Eigen::Success || !ldlt.isPositive()) return false;
  *dx = ldlt.solve(H.transpose() * (w.asDiagonal() * v));
  if (Qx) *Qx = ldlt.solve(Eigen::MatrixXd::Identity(Nm.rows(), Nm.cols()));
  return dx->allFinite();
}

// Baarda w-test on the residuals of a weighted LS fit: w_i = v_i / √(Q_v)_ii with Q_v = R − H Q_x Hᵀ.
// Returns the index of the largest |w| over the threshold, or −1. Rows with (near) zero redundancy cannot
// be tested and are skipped.
int worstResidual(const Eigen::MatrixXd& H, const Eigen::VectorXd& v, const Eigen::VectorXd& var, const Eigen::MatrixXd& Qx,
                  double* w_max) {
  int worst = -1;
  *w_max = 0.0;
  for (int i = 0; i < v.size(); ++i) {
    const double qv = var(i) - H.row(i) * Qx * H.row(i).transpose();
    if (!(qv > 1e-6 * var(i))) continue;
    const double w = std::fabs(v(i)) / std::sqrt(qv);
    if (w > *w_max) {
      *w_max = w;
      worst = i;
    }
  }
  return worst;
}

// Least median of squares over minimal 4-satellite subsets of the base system (columns 0..3 = position +
// clock, or velocity + drift), linearised at the current point. sys_of[i] is the ISB index of row i (−1 =
// base); the ISB columns of the returned increment are the per-system medians of the subset residuals.
// Deterministic: exhaustive up to 1000 subsets, seeded sampling beyond. `score` = h-th smallest squared
// standardised residual, h = ⌊m/2⌋ + 2 (Rousseeuw's order statistic for 4 parameters).
bool lmsFit(const Eigen::MatrixXd& H, const Eigen::VectorXd& v, const Eigen::VectorXd& var, const std::vector<int>& sys_of,
            const int isb_col[4], int p, Eigen::VectorXd* dx_out, double* score_out) {
  const int m = static_cast<int>(H.rows());
  std::vector<int> base;
  for (int i = 0; i < m; ++i)
    if (sys_of[i] < 0) base.push_back(i);
  const int nb = static_cast<int>(base.size());
  if (nb < 4 || m < 5) return false;
  const int h = m / 2 + 2;
  double best = std::numeric_limits<double>::infinity();
  Eigen::VectorXd best_dx;
  std::vector<double> u2(m), res(m);
  auto evaluate = [&](int a, int b, int c, int d) {
    Eigen::Matrix4d A;
    Eigen::Vector4d y;
    const int rows[4] = {base[a], base[b], base[c], base[d]};
    for (int k = 0; k < 4; ++k) {
      A.row(k) = H.row(rows[k]).head<4>();
      y(k) = v(rows[k]);
    }
    Eigen::FullPivLU<Eigen::Matrix4d> lu(A);
    if (!lu.isInvertible()) return;
    const Eigen::Vector4d dx4 = lu.solve(y);
    std::vector<double> per_sys[4];
    for (int i = 0; i < m; ++i) {
      res[i] = v(i) - H.row(i).head<4>().dot(dx4);
      if (sys_of[i] >= 0) per_sys[sys_of[i]].push_back(res[i]);
    }
    double isb[4] = {0.0, 0.0, 0.0, 0.0};
    for (int k = 0; k < 4; ++k)
      if (!per_sys[k].empty()) isb[k] = median(per_sys[k]);
    for (int i = 0; i < m; ++i) {
      const double r = res[i] - (sys_of[i] >= 0 ? isb[sys_of[i]] : 0.0);
      u2[i] = r * r / var(i);
    }
    std::vector<double> sorted = u2;
    std::nth_element(sorted.begin(), sorted.begin() + (h - 1), sorted.end());
    const double score = sorted[h - 1];
    if (score < best) {
      best = score;
      best_dx = Eigen::VectorXd::Zero(p);
      best_dx.head<4>() = dx4;
      for (int k = 0; k < 4; ++k)
        if (isb_col[k] >= 0) best_dx(isb_col[k]) = isb[k];
    }
  };
  const long n_subsets = static_cast<long>(nb) * (nb - 1) * (nb - 2) * (nb - 3) / 24;
  if (n_subsets <= 1000) {
    for (int a = 0; a < nb; ++a)
      for (int b = a + 1; b < nb; ++b)
        for (int c = b + 1; c < nb; ++c)
          for (int d = c + 1; d < nb; ++d) evaluate(a, b, c, d);
  } else {
    std::mt19937 rng(0x5eed);
    std::uniform_int_distribution<int> pick(0, nb - 1);
    for (int n = 0; n < 1000; ++n) {
      const int a = pick(rng), b = pick(rng), c = pick(rng), d = pick(rng);
      if (a == b || a == c || a == d || b == c || b == d || c == d) continue;
      evaluate(a, b, c, d);
    }
  }
  if (!std::isfinite(best)) return false;
  *dx_out = best_dx;
  *score_out = best;
  return true;
}

using LinFn = std::function<void(const Eigen::VectorXd&, const std::vector<int>&, Eigen::MatrixXd*, Eigen::VectorXd*, Eigen::VectorXd*)>;
using SysFn = std::function<int(int)>;

// The robust core shared by RobustSpp and RobustDopplerLs (file header). On return `act` holds the
// consensus set, `excluded` the satellites the LMS stage removed (and that failed re-admission), x the
// IRLS-Huber solution on `act`, (H, v, var) its residual rows and Qx the base-weight covariance.
bool robustCore(const LinFn& lin, const SysFn& sysOf, const int isb_col[4], int p, std::vector<int>* act,
                std::vector<int>* excluded, Eigen::VectorXd* x, int* iterations, Eigen::MatrixXd* H, Eigen::VectorXd* v,
                Eigen::VectorXd* var, Eigen::MatrixXd* Qx) {
  auto gn = [&](bool huber, int max_it) -> bool {
    for (int it = 0; it < max_it; ++it) {
      lin(*x, *act, H, v, var);
      Eigen::VectorXd w = var->cwiseInverse();
      if (huber) {
        for (int a = 0; a < v->size(); ++a) {
          const double ua = std::fabs((*v)(a)) / std::sqrt((*var)(a));
          if (ua > kHuberK) w(a) *= kHuberK / ua;
        }
      }
      Eigen::VectorXd dx;
      if (!solveNormal(*H, *v, w, &dx, nullptr)) return false;
      *x += dx;
      if (iterations) ++*iterations;
      if (dx.head<4>().norm() < 1e-4 && (!huber || it >= 1)) break;
    }
    return true;
  };
  excluded->clear();
  if (!gn(false, 30)) return false;
  lin(*x, *act, H, v, var);
  std::vector<int> sys_of(act->size());
  for (size_t a = 0; a < act->size(); ++a) sys_of[a] = sysOf((*act)[a]);
  Eigen::VectorXd dxl;
  double score = 0.0;
  if (lmsFit(*H, *v, *var, sys_of, isb_col, p, &dxl, &score)) {
    const Eigen::VectorXd x_lms = *x + dxl;
    lin(x_lms, *act, H, v, var);
    const int m = static_cast<int>(act->size());
    // robust scale with Rousseeuw's small-sample factor; never below the a-priori σ (scale 1)
    const double s = std::max(1.0, kMadToSigma * (1.0 + 5.0 / std::max(1, m - p)) * std::sqrt(score));
    std::vector<int> keep, drop;
    for (int a = 0; a < m; ++a) {
      const double u = std::fabs((*v)(a)) / std::sqrt((*var)(a));
      (u <= kConsensus * s ? keep : drop).push_back((*act)[a]);
    }
    *x = x_lms;
    if (static_cast<int>(keep.size()) >= p + 1 && !drop.empty()) {
      *act = keep;
      *excluded = drop;
    }
  }
  auto finish = [&]() -> bool {
    if (!gn(true, 40)) return false;
    lin(*x, *act, H, v, var);
    Eigen::VectorXd dx;
    return solveNormal(*H, *v, var->cwiseInverse(), &dx, Qx);   // step not applied: keep the robust point
  };
  if (!finish()) return false;
  // Re-admission: an excluded satellite whose residual against the consensus solution is consistent with
  // its predicted variance (var + H Q_x Hᵀ, it is not in the fit) was only a victim of the 4-satellite LMS
  // solution's own noise.
  if (!excluded->empty()) {
    std::vector<int> all = *act;
    all.insert(all.end(), excluded->begin(), excluded->end());
    Eigen::MatrixXd Ha;
    Eigen::VectorXd va, vara;
    lin(*x, all, &Ha, &va, &vara);
    std::vector<int> readmit, still;
    for (size_t k = act->size(); k < all.size(); ++k) {
      const int kk = static_cast<int>(k);
      const double q = vara(kk) + Ha.row(kk) * (*Qx) * Ha.row(kk).transpose();
      if (std::fabs(va(kk)) / std::sqrt(q) <= kWThreshold) readmit.push_back(all[k]); else still.push_back(all[k]);
    }
    if (!readmit.empty()) {
      act->insert(act->end(), readmit.begin(), readmit.end());
      *excluded = still;
      if (!finish()) return false;
    }
  }
  return true;
}

}  // namespace

// ---------------------------------------------------------------------------------------------
// RobustSpp
// ---------------------------------------------------------------------------------------------
RobustSpp::RobustSpp(const Config& cfg, const NoiseModel& noise) : cfg_(cfg), noise_(noise) {}

int RobustSpp::nlosPrescreen(std::vector<SppInput>& in, double margin_db) {
  // C/N0 against sin(el): Siegel's repeated-median line (50 % breakdown; Theil–Sen's 29 % is exceeded by
  // two low-elevation suspects among ten satellites) so that the suspects themselves do not bend the trend;
  // slope clamped ≥ 0 (C/N0 never decreases with elevation). Suspects lose psr and dopp membership
  // (§8 COLLECT "初始化期剔除").
  std::vector<std::pair<double, double>> pts;   // (sin el, cn0)
  for (const auto& s : in) {
    if (s.psr_usable && s.cn0 > 0.0) pts.emplace_back(std::sin(s.el_rad), s.cn0);
  }
  if (pts.size() < 4) return 0;
  std::vector<double> slopes;
  for (size_t i = 0; i < pts.size(); ++i) {
    std::vector<double> si;
    for (size_t j = 0; j < pts.size(); ++j) {
      const double dx = pts[j].first - pts[i].first;
      if (j == i || std::fabs(dx) < 0.05) continue;
      si.push_back((pts[j].second - pts[i].second) / dx);
    }
    if (!si.empty()) slopes.push_back(median(si));
  }
  const double slope = slopes.empty() ? 0.0 : std::max(0.0, median(slopes));
  std::vector<double> icpt;
  for (const auto& p : pts) icpt.push_back(p.second - slope * p.first);
  const double a = median(icpt);
  int n = 0;
  for (auto& s : in) {
    if (!s.psr_usable || !(s.cn0 > 0.0)) continue;
    const double expected = a + slope * std::sin(s.el_rad);
    if (s.cn0 < expected - margin_db) {
      s.psr_usable = false;
      s.dopp_usable = false;
      ++n;
    }
  }
  return n;
}

bool RobustSpp::solve(GTime t, std::vector<SppInput>& in, const Klobuchar* ion, bool x0_valid, const Eigen::Vector3d& r0,
                      double b0, SppSolution* out) const {
  if (!out) return false;
  *out = SppSolution();
  out->t = t;
  const bool fixed = allModelsFixed(in);
  const bool qzs_as_gps = cfg_.systems.qzs_as_gps;
  const SysFn sysOf = [&](int i) { return isbIndex(in[i].sat.sys, qzs_as_gps); };

  std::vector<int> cand;                        // indices into `in`
  for (int i = 0; i < static_cast<int>(in.size()); ++i) {
    const SppInput& s = in[i];
    if (s.psr_usable && s.state.ok && s.P_corr > 0.0 && std::isfinite(s.P_corr)) cand.push_back(i);
  }
  // ISB columns for the systems present (one per non-GPS system, fixed order GLO, GAL, BDS, QZS).
  int isb_col[4] = {-1, -1, -1, -1};
  int p = 4;
  // A non-GPS system with a single usable satellite adds one ISB unknown for one equation: that satellite
  // carries no information on position/clock and cannot be tested (a blunder on it vanishes into its ISB).
  // Such satellites are left out of the robust LS (neither used nor rejected).
  auto pruneLoneSystems = [&](std::vector<int>* act) {
    int cnt[4] = {0, 0, 0, 0};
    for (int i : *act) {
      const int k = sysOf(i);
      if (k >= 0) ++cnt[k];
    }
    act->erase(std::remove_if(act->begin(), act->end(),
                              [&](int i) {
                                const int k = sysOf(i);
                                return k >= 0 && cnt[k] < 2;
                              }),
               act->end());
  };
  auto assignCols = [&](const std::vector<int>& active) {
    bool present[4] = {false, false, false, false};
    for (int i : active) {
      const int k = sysOf(i);
      if (k >= 0) present[k] = true;
    }
    p = 4;
    for (int k = 0; k < 4; ++k) isb_col[k] = present[k] ? p++ : -1;
  };
  pruneLoneSystems(&cand);
  assignCols(cand);
  if (static_cast<int>(cand.size()) < std::max(p, cfg_.init.min_sats)) {
    out->ok = false;
    return false;
  }

  std::vector<int> active = cand;
  std::vector<int> rejected;
  Eigen::VectorXd x = Eigen::VectorXd::Zero(p);
  if (x0_valid) {
    x.head<3>() = r0;
    x(3) = b0;
  }
  Geodetic geo;
  bool geo_valid = false;

  // Evaluate residuals/design at x for a satellite set; refreshes the atmospheric models unless fixed.
  const LinFn linearise = [&](const Eigen::VectorXd& xx, const std::vector<int>& act, Eigen::MatrixXd* H, Eigen::VectorXd* v,
                              Eigen::VectorXd* var) {
    const int m = static_cast<int>(act.size());
    H->setZero(m, p);
    v->resize(m);
    var->resize(m);
    const Eigen::Vector3d r = xx.head<3>();
    const bool on_earth = r.norm() > kMinEarthRadius;
    if (!fixed && on_earth) {
      geo = rtk::ecef2pos(r);
      geo_valid = true;
    }
    for (int a = 0; a < m; ++a) {
      SppInput& s = in[act[a]];
      Eigen::Vector3d e;
      const double rho = rangeWithSagnac(s.state.r_ecef, r, &e);
      if (!fixed && on_earth) {
        rtk::satAzel(geo, e, &s.az_rad, &s.el_rad);
        s.trop_m = rtk::tropModel(t, geo, s.az_rad, s.el_rad, kRelHumi);
        if (ion && ion->valid() && s.lambda_m > 0.0) {
          const double f = kCLight / s.lambda_m;
          s.ion_m = rtk::ionModelGpsL1(t, *ion, geo, s.az_rad, s.el_rad) * (kFreqL1 / f) * (kFreqL1 / f);
        } else {
          s.ion_m = 0.0;
        }
      }
      const int k = sysOf(act[a]);
      const double iota = (k >= 0 && isb_col[k] >= 0) ? xx(isb_col[k]) : 0.0;
      const double h = rho + xx(3) + iota - kCLight * s.state.dts + s.ion_m + s.trop_m;
      (*v)(a) = s.P_corr - h;
      H->row(a).head<3>() = -e.transpose();
      (*H)(a, 3) = 1.0;
      if (k >= 0 && isb_col[k] >= 0) (*H)(a, isb_col[k]) = 1.0;
      (*var)(a) = s.var_psr > 0.0 ? s.var_psr : 1.0;
    }
  };

  Eigen::MatrixXd H, Qx;
  Eigen::VectorXd v, var;
  for (int guard = 0; guard < 64; ++guard) {
    assignCols(active);
    if (static_cast<int>(active.size()) < p) {
      out->ok = false;
      return false;
    }
    if (static_cast<int>(x.size()) != p) {
      Eigen::VectorXd xn = Eigen::VectorXd::Zero(p);
      xn.head<4>() = x.head<4>();
      x = xn;
    }
    std::vector<int> excluded;
    if (!robustCore(linearise, sysOf, isb_col, p, &active, &excluded, &x, &out->iterations, &H, &v, &var, &Qx)) {
      out->ok = false;
      return false;
    }
    if (!excluded.empty()) {
      rejected.insert(rejected.end(), excluded.begin(), excluded.end());
      const size_t before = active.size();
      pruneLoneSystems(&active);
      if (active.size() != before) continue;   // ISB layout changed: refit
    }
    // Iterated w-test: drop the worst satellite while it fails and redundancy allows another test afterwards.
    const int m = static_cast<int>(active.size());
    if (m - p < 2) break;
    double w_max = 0.0;
    const int worst = worstResidual(H, v, var, Qx, &w_max);
    if (worst < 0 || w_max <= kWThreshold) break;
    rejected.push_back(active[worst]);
    active.erase(active.begin() + worst);
    pruneLoneSystems(&active);   // the partner of a rejected 2-satellite system carries no information any more
  }

  // Solution bookkeeping
  const int m = static_cast<int>(active.size());
  const Eigen::VectorXd w = var.cwiseInverse();
  const double dof = static_cast<double>(m - p);
  const double s0sq = dof > 0.0 ? (v.dot(w.asDiagonal() * v)) / dof : 1.0;
  out->ok = true;
  out->r_ecef = x.head<3>();
  out->b_m = x(3);
  for (int k = 0; k < 4; ++k) {
    out->isb_est[k] = isb_col[k] >= 0;
    out->isb_m[k] = isb_col[k] >= 0 ? x(isb_col[k]) : 0.0;
  }
  out->x = x;
  out->Q = Qx * s0sq;
  for (int k = 0; k < 3; ++k) {
    const double floor_var = cfg_.init.spp_sigma_floor_m * cfg_.init.spp_sigma_floor_m;
    if (out->Q(k, k) < floor_var) out->Q(k, k) = floor_var;   // §8 ALIGN: max(formal·σ̂0², 3² m²)
  }
  out->sigma0 = std::sqrt(s0sq);
  // DOPs from the unit-weight 4-column geometry (position + clock, RTKLIB dops(); rotation invariant).
  {
    Eigen::MatrixXd H4(m, 4);
    for (int a = 0; a < m; ++a) {
      H4.row(a).head<3>() = H.row(a).head<3>();
      H4(a, 3) = 1.0;
    }
    const Eigen::Matrix4d Q4 = (H4.transpose() * H4).inverse();
    out->gdop = std::sqrt(Q4.trace());
    out->pdop = std::sqrt(Q4.topLeftCorner<3, 3>().trace());
  }
  out->n_used = m;
  out->n_rejected = static_cast<int>(rejected.size());
  out->used.clear();
  out->residuals.clear();
  for (int a = 0; a < m; ++a) {
    out->used.push_back(in[active[a]].sat);
    out->residuals.push_back(v(a));
  }
  for (int i : rejected) out->rejected.push_back(in[i].sat);
  if (!fixed) {
    if (!geo_valid) geo = rtk::ecef2pos(out->r_ecef);
    out->geo = geo;
  }
  return true;
}

// ---------------------------------------------------------------------------------------------
// RobustDopplerLs: [v(3), d] from range rates (linear given the position)
// ---------------------------------------------------------------------------------------------
RobustDopplerLs::RobustDopplerLs(const Config& cfg, const NoiseModel& noise) : cfg_(cfg), noise_(noise) {}

bool RobustDopplerLs::solve(GTime t, const std::vector<SppInput>& in, const Eigen::Vector3d& r_ecef, DopplerSolution* out) const {
  if (!out) return false;
  *out = DopplerSolution();
  out->t = t;
  std::vector<int> active;
  for (int i = 0; i < static_cast<int>(in.size()); ++i) {
    const SppInput& s = in[i];
    if (s.dopp_usable && s.state.ok && s.lambda_m > 0.0 && s.dopp_hz != 0.0 && std::isfinite(s.dopp_hz) &&
        s.state.v_ecef.norm() > 0.0)
      active.push_back(i);
  }
  const int p = 4;
  if (static_cast<int>(active.size()) < p) return false;
  std::vector<int> rejected;
  Eigen::VectorXd x = Eigen::VectorXd::Zero(p);

  const LinFn linearise = [&](const Eigen::VectorXd& xx, const std::vector<int>& act, Eigen::MatrixXd* H, Eigen::VectorXd* v,
                              Eigen::VectorXd* var) {
    const int m = static_cast<int>(act.size());
    H->setZero(m, p);
    v->resize(m);
    var->resize(m);
    const Eigen::Vector3d vr = xx.head<3>();
    for (int a = 0; a < m; ++a) {
      const SppInput& s = in[act[a]];
      Eigen::Vector3d e;
      rangeWithSagnac(s.state.r_ecef, r_ecef, &e);
      const Eigen::Vector3d& rs = s.state.r_ecef;
      const Eigen::Vector3d& vs = s.state.v_ecef;
      // rate = d/dt[‖r_s − r‖ + (ω_e/c)(x_s y − y_s x)] = e·(v_s − v) + (ω_e/c)(v_sx y + x_s v_y − v_sy x − y_s v_x):
      // the Sagnac rate with the derivative's sign (AMENDMENTS A36; RTKLIB resdop pntpos.c:580-581 has its negative).
      const double rate = e.dot(vs - vr) + kOmgE / kCLight * (vs.x() * r_ecef.y() + rs.x() * vr.y() - vs.y() * r_ecef.x() - rs.y() * vr.x());
      const double h = rate + xx(3) - kCLight * s.state.ddts;
      (*v)(a) = -s.lambda_m * s.dopp_hz - h;
      H->row(a).head<3>() = -e.transpose();
      (*H)(a, 0) -= kOmgE / kCLight * rs.y();   // exact Sagnac-rate derivative ∂rate/∂v (kept: it is free here)
      (*H)(a, 1) += kOmgE / kCLight * rs.x();
      (*H)(a, 3) = 1.0;
      (*var)(a) = s.var_dopp > 0.0 ? s.var_dopp : 1.0;
    }
  };
  const SysFn sysOf = [](int) { return -1; };
  const int no_isb[4] = {-1, -1, -1, -1};

  Eigen::MatrixXd H, Qx;
  Eigen::VectorXd v, var;
  for (int guard = 0; guard < 64; ++guard) {
    std::vector<int> excluded;
    if (!robustCore(linearise, sysOf, no_isb, p, &active, &excluded, &x, nullptr, &H, &v, &var, &Qx)) return false;
    rejected.insert(rejected.end(), excluded.begin(), excluded.end());
    const int m = static_cast<int>(active.size());
    if (m - p < 2) break;
    double w_max = 0.0;
    const int worst = worstResidual(H, v, var, Qx, &w_max);
    if (worst < 0 || w_max <= kWThreshold) break;
    rejected.push_back(active[worst]);
    active.erase(active.begin() + worst);
  }
  const int m = static_cast<int>(active.size());
  const Eigen::VectorXd w = var.cwiseInverse();
  const double dof = static_cast<double>(m - p);
  const double s0sq = dof > 0.0 ? v.dot(w.asDiagonal() * v) / dof : 1.0;
  out->ok = true;
  out->v_ecef = x.head<3>();
  out->d_mps = x(3);
  out->x = x;
  out->Q = Qx * s0sq;
  out->sigma0 = std::sqrt(s0sq);
  out->n_used = m;
  out->n_rejected = static_cast<int>(rejected.size());
  for (int a = 0; a < m; ++a) out->used.push_back(in[active[a]].sat);
  for (int i : rejected) out->rejected.push_back(in[i].sat);
  return true;
}

// ---------------------------------------------------------------------------------------------
// Clock-only LS with the antenna fixed to the LIVO prediction (L2 non-integer jump, RECOVERY).
// Medians of the per-satellite clock estimates; MAD-based variances (DESIGN §10 L2).
// ---------------------------------------------------------------------------------------------
ClockOnlyResult clockOnlyLs(const std::vector<SppInput>& in, const Eigen::Vector3d& r_ecef_pred, const Eigen::Vector3d& v_ecef_pred) {
  ClockOnlyResult res;
  std::vector<double> bs, ds, var_b_single, var_d_single;
  for (const auto& s : in) {
    if (!s.state.ok) continue;
    Eigen::Vector3d e;
    const double rho = rangeWithSagnac(s.state.r_ecef, r_ecef_pred, &e);
    // b from GPS (and QZSS on the GPS clock) only: the other systems' codes carry their ISB (see below).
    if (s.psr_usable && s.P_corr > 0.0 && (s.sat.sys == Sys::GPS || s.sat.sys == Sys::QZS)) {
      bs.push_back(s.P_corr - (rho - kCLight * s.state.dts + s.ion_m + s.trop_m));
      var_b_single.push_back(s.var_psr > 0.0 ? s.var_psr : 1.0);
    }
    if (s.dopp_usable && s.lambda_m > 0.0 && s.dopp_hz != 0.0 && s.state.v_ecef.norm() > 0.0) {
      const Eigen::Vector3d& rs = s.state.r_ecef;
      const Eigen::Vector3d& vs = s.state.v_ecef;
      // Range rate with the derivative-sign Sagnac rate (AMENDMENTS A36), as in RobustDopplerLs and rows.cpp.
      const double rate = e.dot(vs - v_ecef_pred) + kOmgE / kCLight * (vs.x() * r_ecef_pred.y() + rs.x() * v_ecef_pred.y() -
                                                                      vs.y() * r_ecef_pred.x() - rs.y() * v_ecef_pred.x());
      ds.push_back(-s.lambda_m * s.dopp_hz - (rate - kCLight * s.state.ddts));
      var_d_single.push_back(s.var_dopp > 0.0 ? s.var_dopp : 1.0);
    }
  }
  // b is the GPS clock (§2.2 ι_G ≡ 0). The other systems' pseudoranges carry their ISB, which SppInput does not
  // provide, so without a GPS/QZSS code the reset is refused rather than biased by metres to tens of metres
  // (review m7).
  if (bs.empty()) return res;
  res.ok = true;
  res.n_used = static_cast<int>(bs.size());
  res.b_m = median(bs);
  if (bs.size() >= 3) {
    const double s = kMadToSigma * mad(bs, res.b_m);
    res.var_b = std::max(s * s, 1e-4);
  } else {
    res.var_b = median(var_b_single);
  }
  if (!ds.empty()) {
    res.d_mps = median(ds);
    if (ds.size() >= 3) {
      const double s = kMadToSigma * mad(ds, res.d_mps);
      res.var_d = std::max(s * s, 1e-8);
    } else {
      res.var_d = median(var_d_single);
    }
  }
  return res;
}

}  // namespace livo_gnss
