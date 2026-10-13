// SPDX-License-Identifier: BSD-3-Clause
// Initializer (DESIGN §8; AMENDMENTS A35 ISB slots, A43 1 Hz rule, A51 LIO-aided multi-epoch path). Owner: P3-A1, A51 by
// P4-INIT. Tests: U13.
//
// Frames and conventions are those of DESIGN §1: M = R_{N←W} = R_z(ψ)·Exp([φ,0])·C(g), a^N = M(a^W − o^W) + t,
// the host error state is right-perturbed on R and additive elsewhere, ψ is left-perturbed, x_G is additive.
// At ACTIVATE φ = 0 (prior σ_DOV), so M = R_z(ψ)·C(g) throughout this file.
//
// ALIGN in four steps over the COLLECT window (samples k_0 … k_a, all satellites already through L0, the robust
// GNSS-only SPP/Doppler LS and the NLOS pre-screen):
//   1. N frame: r_0 = component-wise median of the window's SPP positions (first ALIGN only; REINIT keeps N).
//   2. Time: residual GNSS-vs-host time offset from the speed series, |v_GNSS| − |v_host| = c + τ·d|v_host|/dt — the
//      small-lag form of §7's cross-correlation of |v| (the only quantities that need no ψ). Diagnostic only: τ beyond
//      3·√(σ_τ² + σ_toff²) sets offset_warn (TOFF_WARN), nothing is changed.
//   3. Yaw: 2D Procrustes of the Doppler-LS velocities against the levelled host antenna velocities gives ψ₀; a
//      Gauss–Newton LS on the raw Doppler rows (−λD = e·(v_s − R_EN R_z(ψ) C w^W_j) + Sagnac rate + d_j − c·δṫ_s, the
//      per-epoch drifts d_j eliminated per epoch) with an iterated Baarda w-test refines it; σ_ψ² = max(1, σ̂0²)/N_ψψ.
//      max(1, ·): the a-priori Doppler variances already carry κ_AR (time correlation, §4.7) that post-fit scatter of
//      correlated rows cannot show; scaling below one would undo it.
//   4. Translation/clock/ISB: ψ fixed, the raw pseudoranges of the 1 Hz epochs of the window (A43: the first epoch of
//      every GPST second — the rate the a-priori κ_AR(τ_ρ, 1 s) of the pseudorange variance is written for) in one LS
//      with shared t and ι_k and one clock b_j per epoch (eliminated per epoch), iterated w-test, σ̂ ≤ 5 m. Its
//      sensitivities to ψ and φ (the lever terms M(y_j − o) of every epoch j ≠ k_a) are the J_trans_yaw/J_trans_tilt of
//      the consistent augmentation; the host enters only through the pivot, e_t = −J_a,host·e_x (the same shift of
//      every y_j is absorbed by t, so b and ι do not depend on e_x).
// A51 adds a second ALIGN path for windows of 1–3-satellite epochs (the block comment above LaEpoch below; tryAlign()
// tries §8 first when both are armed).
// ACTIVATE is the consistent augmentation (§2.4) of the new block y = [φ, ψ, t, b, d, ι…, (td)]:
//   e_y = F_host·e_x + G·n,  n = [n_φ, n_ψ, n_batch, n_ιunobs, n_td] independent,
// Cov(y) = F_host P F_hostᵀ + G Σ_n Gᵀ, Cov(y, x) = F_host P. AugBlock::addSlot() writes the F_host parts slot by slot;
// the noise cross-blocks between different new slots are added afterwards (addSlot takes each slot's own noise only).
#include "internal/initializer.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <sstream>

#include <Eigen/Cholesky>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include "internal/clock.hpp"
#include "internal/quality.hpp"
#include "internal/rows.hpp"
#include "internal/rtk_adapter.hpp"

namespace livo_gnss {

namespace {

constexpr double kCLight = 299792458.0;              // rtklib.h CLIGHT
constexpr double kOmgE = 7.2921151467e-5;            // rtklib.h OMGE
constexpr double kDeg = M_PI / 180.0;
constexpr double kWThreshold = 3.2905267314919255;   // Baarda α = 1e-3 two-sided (the L1/L4 level)
constexpr double kIsbUnobservedSigma = 30.0;         // AMENDMENTS A35: a system without data gets ι = 0, σ = 30 m
constexpr double kMaxRejectFrac = 0.3;               // the iterated w-test may drop at most this share of the rows
constexpr double kMaxPsrSigmaM = 5.0;                // §8 ALIGN "σ̂ ≤ 5 m"
constexpr double kMinSpeedStdMps = 0.1;              // §7: speed std < 0.1 m/s ⇒ time check not possible

using View = std::vector<const CollectSample*>;

std::string fmt(double v, int prec = 3) {
  std::ostringstream os;
  os.setf(std::ios::fixed);
  os.precision(prec);
  os << v;
  return os.str();
}

double wrapPi(double a) {
  if (!std::isfinite(a)) return a;
  a -= 2.0 * M_PI * std::floor((a + M_PI) / (2.0 * M_PI));
  if (a <= -M_PI) a += 2.0 * M_PI;
  return a;
}

double median(std::vector<double> v) {
  if (v.empty()) return 0.0;
  std::sort(v.begin(), v.end());
  const size_t n = v.size();
  return (n % 2 == 1) ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
}

// Geometric range with the Sagnac term, identical to RTKLIB geodist (rtkcmn.c:3599); e = unit LOS receiver → satellite.
double rangeWithSagnac(const Eigen::Vector3d& rs, const Eigen::Vector3d& rr, Eigen::Vector3d* e) {
  const Eigen::Vector3d d = rs - rr;
  const double r = d.norm();
  if (e) *e = d / r;
  return r + kOmgE * (rs.x() * rr.y() - rs.y() * rr.x()) / kCLight;
}

// ∂ρ/∂r_A (row, ECEF) of rangeWithSagnac: −eᵀ plus the Sagnac term's (ω_e/c)(−y_s, x_s, 0).
Eigen::RowVector3d rangeGradient(const Eigen::Vector3d& rs, const Eigen::Vector3d& e) {
  return Eigen::RowVector3d(-e.x() - kOmgE / kCLight * rs.y(), -e.y() + kOmgE / kCLight * rs.x(), -e.z());
}

bool contains(const std::vector<SatId>& v, const SatId& s) {
  for (const SatId& x : v)
    if (x == s) return true;
  return false;
}

// AlignResult ISB order: 0 = BDS (ι_C), 1 = GAL (ι_E), 2 = GLO (ι_R), 3 = QZS (own clock only without qzs_as_gps).
int isbIndexOf(Sys s, bool qzs_as_gps) {
  switch (s) {
    case Sys::BDS: return 0;
    case Sys::GAL: return 1;
    case Sys::GLO: return 2;
    case Sys::QZS: return qzs_as_gps ? -1 : 3;
    default: return -1;
  }
}
Sys isbSys(int k) {
  static const Sys s[4] = {Sys::BDS, Sys::GAL, Sys::GLO, Sys::QZS};
  return s[k];
}
const char* isbSlotName(int k) {
  static const char* n[4] = {"isb_C", "isb_E", "isb_R", "isb_J"};
  return n[k];
}
// gnss_only's SppSolution::isb_m order is [GLO, GAL, BDS, QZS].
int sppIsbIndex(int k) {
  static const int m[4] = {2, 1, 0, 3};
  return m[k];
}
// A35: a slot for every enabled system whose code enters the filter (tight coupling with pseudorange rows). QZSS is only
// ever enabled on the GPS clock (QualityGate::systemEnabled), GLONASS code only with glo_psr (§2.2 "ι_R 不建").
bool isbSlotEnabled(const Config& cfg, int k) {
  if (cfg.mode == Config::Mode::LoosePvt || !cfg.use.psr) return false;
  switch (k) {
    case 0: return cfg.systems.bds;
    case 1: return cfg.systems.gal;
    case 2: return cfg.systems.glo && cfg.systems.glo_psr;
    default: return false;
  }
}
// Code rows the batch may use: base clock (GPS, QZSS on it) or a system whose ISB the batch can estimate.
bool batchCodeSystem(const Config& cfg, Sys s, int* k) {
  *k = -1;
  if (s == Sys::GPS) return cfg.systems.gps;
  if (s == Sys::QZS) return cfg.systems.gps && cfg.systems.qzs_as_gps;
  const int kk = isbIndexOf(s, cfg.systems.qzs_as_gps);
  if (kk < 0 || kk > 2) return false;
  if (kk == 0 && !cfg.systems.bds) return false;
  if (kk == 1 && !cfg.systems.gal) return false;
  if (kk == 2 && !(cfg.systems.glo && cfg.systems.glo_psr)) return false;
  *k = kk;
  return true;
}

// Unit vector of gravity in W from the bridge snapshot (zero when not available).
Eigen::Vector3d gravityDir(const BridgeResult& br) {
  const double n = br.g_snap.norm();
  return n > 1.0 ? Eigen::Vector3d(br.g_snap / n) : Eigen::Vector3d::Zero();
}
Eigen::Vector3d horizontal(const Eigen::Vector3d& v, const Eigen::Vector3d& gdir) { return v - v.dot(gdir) * gdir; }

struct MotionStats {
  int moving = 0;
  double path = 0.0;
};
template <class It>
MotionStats motionOf(It begin, It end, double min_hspeed) {
  MotionStats m;
  const CollectSample* prev = nullptr;
  for (It it = begin; it != end; ++it) {
    const CollectSample& s = **it;
    const Eigen::Vector3d gd = gravityDir(s.bridge);
    if (horizontal(s.host_ant.z, gd).norm() >= min_hspeed) ++m.moving;
    if (prev) m.path += horizontal(Eigen::Vector3d(s.host_ant.y - prev->host_ant.y), gd).norm();
    prev = &s;
  }
  return m;
}

// ---------------------------------------------------------------------------------------------
// Step 2: time check
// ---------------------------------------------------------------------------------------------
struct TimeCheck {
  bool valid = false;
  double tau = 0.0, sigma = 0.0;
};

TimeCheck timeCheck(const View& w) {
  TimeCheck tc;
  std::vector<double> t, sg, sh;
  for (const CollectSample* s : w) {
    if (!s->dop.ok) continue;
    t.push_back(s->t_est);
    sg.push_back(s->dop.v_ecef.norm());
    sh.push_back(s->host_ant.z.norm());
  }
  const int n = static_cast<int>(t.size());
  if (n < 10) return tc;
  double mean = 0.0;
  for (double v : sh) mean += v;
  mean /= n;
  double var = 0.0;
  for (double v : sh) var += (v - mean) * (v - mean);
  if (std::sqrt(var / (n - 1)) < kMinSpeedStdMps) return tc;
  std::vector<double> x, y;
  for (int j = 1; j + 1 < n; ++j) {
    const double dt = t[j + 1] - t[j - 1];
    if (!(dt > 0.0)) continue;
    x.push_back((sh[j + 1] - sh[j - 1]) / dt);   // d|v_host|/dt
    y.push_back(sg[j] - sh[j]);                   // |v_GNSS| − |v_host|
  }
  const int m = static_cast<int>(x.size());
  if (m < 8) return tc;
  double xm = 0.0, ym = 0.0;
  for (int j = 0; j < m; ++j) { xm += x[j]; ym += y[j]; }
  xm /= m;
  ym /= m;
  double sxx = 0.0, sxy = 0.0;
  for (int j = 0; j < m; ++j) {
    sxx += (x[j] - xm) * (x[j] - xm);
    sxy += (x[j] - xm) * (y[j] - ym);
  }
  if (!(sxx > 1e-12)) return tc;
  tc.tau = sxy / sxx;
  double ssr = 0.0;
  for (int j = 0; j < m; ++j) {
    const double r = y[j] - ym - tc.tau * (x[j] - xm);
    ssr += r * r;
  }
  tc.sigma = std::sqrt(ssr / std::max(1, m - 2) / sxx);
  tc.valid = std::isfinite(tc.tau) && std::isfinite(tc.sigma);
  return tc;
}

// ---------------------------------------------------------------------------------------------
// Step 3: yaw from raw Doppler
// ---------------------------------------------------------------------------------------------
struct DoppRow {
  int epoch = 0;
  const SppInput* in = nullptr;
  Eigen::Vector3d r_A = Eigen::Vector3d::Zero();   // antenna ECEF (SPP of the epoch; LOS/Sagnac only)
  Eigen::Vector3d z_lev = Eigen::Vector3d::Zero(); // C(g)·w^W_j: levelled host antenna velocity
  double var = 1.0;
  bool active = true;
};

bool yawFitView(const View& w, double yaw0, const Eigen::Matrix3d& C, const Eigen::Matrix3d& R_EN, Initializer::YawFit* fit,
                std::string* why) {
  auto fail = [&](const std::string& s) {
    if (why) *why = s;
    return false;
  };
  std::vector<DoppRow> rows;
  int n_epochs = 0;
  for (size_t j = 0; j < w.size(); ++j) {
    const CollectSample& s = *w[j];
    const Eigen::Vector3d z_lev = C * s.host_ant.z;
    bool any = false;
    for (const SppInput& in : s.raw) {
      if (!in.dopp_usable || !in.state.ok || !(in.lambda_m > 0.0) || in.dopp_hz == 0.0 || !std::isfinite(in.dopp_hz)) continue;
      if (!(in.state.v_ecef.norm() > 0.0)) continue;
      if (s.dop.ok && contains(s.dop.rejected, in.sat)) continue;   // the epoch's robust Doppler LS judged it an outlier
      DoppRow r;
      r.epoch = static_cast<int>(j);
      r.in = &in;
      r.r_A = s.spp.r_ecef;
      r.z_lev = z_lev;
      r.var = in.var_dopp > 0.0 ? in.var_dopp : 1.0;
      rows.push_back(r);
      any = true;
    }
    if (any) ++n_epochs;
  }
  const int m_all = static_cast<int>(rows.size());
  if (m_all < 3) return fail("yaw: fewer than 3 Doppler rows in the window");
  const int n_ep = static_cast<int>(w.size());
  std::vector<double> v(m_all), a(m_all);
  std::vector<double> W(n_ep), abar(n_ep), vbar(n_ep);
  double psi = yaw0, N = 0.0;
  int rejected = 0, iterations = 0;
  auto linearise = [&](double p) {
    const Eigen::Matrix3d Rz = Leveling::Rz(p);
    for (int i = 0; i < m_all; ++i) {
      const DoppRow& r = rows[i];
      const SppInput& in = *r.in;
      const Eigen::Vector3d wN = Rz * r.z_lev;
      const Eigen::Vector3d rdot = R_EN * wN;
      Eigen::Vector3d e;
      rangeWithSagnac(in.state.r_ecef, r.r_A, &e);
      const Eigen::Vector3d& rs = in.state.r_ecef;
      const Eigen::Vector3d& vs = in.state.v_ecef;
      // range rate with the derivative-sign Sagnac rate (AMENDMENTS A36), as rows.cpp / gnss_only.cpp
      const double rate = e.dot(vs - rdot) +
                          kOmgE / kCLight * (vs.x() * r.r_A.y() + rs.x() * rdot.y() - vs.y() * r.r_A.x() - rs.y() * rdot.x());
      v[i] = -in.lambda_m * in.dopp_hz - (rate - kCLight * in.state.ddts);   // residual without d_j
      // ∂rate/∂ṙ_A = −eᵀ + (ω_e/c)(−y_s, x_s, 0); ∂ṙ_A/∂ψ = R_EN [e_z]× w^N (R_z(ψ+δψ) = Exp(e_z δψ) R_z(ψ))
      const Eigen::RowVector3d g(-e.x() - kOmgE / kCLight * rs.y(), -e.y() + kOmgE / kCLight * rs.x(), -e.z());
      a[i] = g.dot(R_EN * Eigen::Vector3d::UnitZ().cross(wN));
    }
  };
  auto reduce = [&]() {
    std::fill(W.begin(), W.end(), 0.0);
    std::fill(abar.begin(), abar.end(), 0.0);
    std::fill(vbar.begin(), vbar.end(), 0.0);
    for (int i = 0; i < m_all; ++i) {
      if (!rows[i].active) continue;
      const double wi = 1.0 / rows[i].var;
      W[rows[i].epoch] += wi;
      abar[rows[i].epoch] += wi * a[i];
      vbar[rows[i].epoch] += wi * v[i];
    }
    for (int j = 0; j < n_ep; ++j)
      if (W[j] > 0.0) {
        abar[j] /= W[j];
        vbar[j] /= W[j];
      }
    double Nn = 0.0, rhs = 0.0;
    for (int i = 0; i < m_all; ++i) {
      if (!rows[i].active) continue;
      const int j = rows[i].epoch;
      const double wi = 1.0 / rows[i].var;
      Nn += wi * (a[i] - abar[j]) * (a[i] - abar[j]);
      rhs += wi * (a[i] - abar[j]) * (v[i] - vbar[j]);
    }
    return std::make_pair(Nn, rhs);
  };
  for (int round = 0; round <= m_all; ++round) {
    for (int it = 0; it < 30; ++it) {
      linearise(psi);
      const auto nr = reduce();
      N = nr.first;
      if (!(N > 0.0) || !std::isfinite(N)) return fail("yaw: no heading information in the Doppler rows");
      const double dpsi = nr.second / N;
      psi += dpsi;
      ++iterations;
      if (std::fabs(dpsi) < 1e-12) break;
    }
    linearise(psi);
    N = reduce().first;
    // Iterated Baarda w-test on the post-fit residuals r_i = v_i − d̂_j (d̂_j = v̄_j at the solution).
    double w_max = 0.0;
    int worst = -1;
    for (int i = 0; i < m_all; ++i) {
      if (!rows[i].active) continue;
      const int j = rows[i].epoch;
      const double r = v[i] - vbar[j];
      const double qv = rows[i].var - ((a[i] - abar[j]) * (a[i] - abar[j]) / N + 1.0 / W[j]);
      if (!(qv > 1e-9 * rows[i].var)) continue;   // no redundancy (single row of its epoch)
      const double wv = std::fabs(r) / std::sqrt(qv);
      if (wv > w_max) {
        w_max = wv;
        worst = i;
      }
    }
    if (worst < 0 || w_max <= kWThreshold) break;
    rows[worst].active = false;
    ++rejected;
    if (rejected > kMaxRejectFrac * m_all) return fail("yaw: more than 30 % of the Doppler rows rejected");
  }
  // post-fit unit weight: m rows, one d_j per epoch with rows, one ψ
  double ssr = 0.0;
  int m = 0, ep_used = 0;
  std::vector<int> ep_rows(n_ep, 0);
  for (int i = 0; i < m_all; ++i) {
    if (!rows[i].active) continue;
    const int j = rows[i].epoch;
    const double r = v[i] - vbar[j];
    ssr += r * r / rows[i].var;
    ++m;
    ++ep_rows[j];
  }
  for (int j = 0; j < n_ep; ++j)
    if (ep_rows[j] > 0) ++ep_used;
  const int dof = m - ep_used - 1;
  if (dof < 1) return fail("yaw: no redundancy in the Doppler rows");
  const double s0sq = ssr / dof;
  fit->yaw = wrapPi(psi);
  fit->sigma_yaw = std::sqrt(std::max(1.0, s0sq) / N);
  fit->sigma0 = std::sqrt(s0sq);
  fit->rows = m;
  fit->epochs = ep_used;
  fit->rejected = rejected;
  fit->iterations = iterations;
  (void)n_epochs;
  return std::isfinite(fit->yaw) && std::isfinite(fit->sigma_yaw);
}

// ---------------------------------------------------------------------------------------------
// Step 4: batch pseudorange LS for t, ι (shared) and b_j (per epoch)
// ---------------------------------------------------------------------------------------------
struct PsrRow {
  int epoch = 0;                                    // index into the decimated epoch list
  const SppInput* in = nullptr;
  int k = -1;                                       // ISB index (−1 = base clock)
  double var = 1.0;
  bool active = true;
  // at the current linearisation
  double v = 0.0;                                   // P_corr − h(θ) without b_j
  Eigen::RowVector3d g_t = Eigen::RowVector3d::Zero();   // ∂h/∂t
  double h_psi = 0.0;                               // ∂h/∂ψ
  Eigen::RowVector2d h_phi = Eigen::RowVector2d::Zero(); // ∂h/∂φ
};

struct BatchResult {
  bool ok = false;
  std::string why;
  Eigen::Vector3d t = Eigen::Vector3d::Zero();
  double isb[4] = {0, 0, 0, 0};
  bool observed[4] = {false, false, false, false};
  std::vector<int> col_k;                           // θ = [t, ι_{col_k[0]}, …]
  Eigen::MatrixXd Ninv;                             // unit-weight (a-priori variances) inverse normal matrix of θ
  double scale = 1.0;                               // max(1, σ̂0²)
  double s0sq = 0.0, rms_m = 0.0;
  int rows = 0, rejected = 0, epochs = 0;
  // clock epoch j_b (the newest decimated epoch with rows)
  int jb = -1;
  double b_jb = 0.0, invW_jb = 0.0;
  Eigen::VectorXd gbar_jb;
  double hpsi_bar_jb = 0.0;
  Eigen::RowVector2d hphi_bar_jb = Eigen::RowVector2d::Zero();
  Eigen::VectorXd dtheta_dpsi;
  Eigen::MatrixXd dtheta_dphi;
};

BatchResult batchLs(const Config& cfg, const View& ep, const std::vector<Eigen::Vector3d>& y_rel, const Eigen::Matrix3d& M,
                    const Eigen::Matrix3d& Rz, const Eigen::Matrix3d& C, const AnchorFrame& anchor, const Eigen::Vector3d& t0,
                    const double isb0[4]) {
  BatchResult out;
  const int J = static_cast<int>(ep.size());
  std::vector<PsrRow> rows;
  for (int j = 0; j < J; ++j) {
    const CollectSample& s = *ep[j];
    for (const SppInput& in : s.raw) {
      if (!in.psr_usable || !in.state.ok || !(in.P_corr > 0.0) || !std::isfinite(in.P_corr)) continue;
      if (contains(s.spp.rejected, in.sat)) continue;   // L1 outlier of its own epoch
      int k = -1;
      if (!batchCodeSystem(cfg, in.sat.sys, &k)) continue;
      PsrRow r;
      r.epoch = j;
      r.in = &in;
      r.k = k;
      r.var = in.var_psr > 0.0 ? in.var_psr : 1.0;
      rows.push_back(r);
    }
  }
  const int m_all = static_cast<int>(rows.size());
  if (m_all < 4) {
    out.why = "trans: fewer than 4 pseudorange rows in the 1 Hz epochs";
    return out;
  }
  // Each epoch's rows share ψ-dependent geometry terms: precompute per-epoch lever sensitivities.
  std::vector<Eigen::Vector3d> dA_dpsi(J);                  // ∂a^N_j/∂ψ = [e_z]× M y_rel
  std::vector<Eigen::Matrix<double, 3, 2>> dA_dphi(J);      // ∂a^N_j/∂φ = Ξ(y_rel) E_xy, Ξ(v) = −R_z [C v]×
  for (int j = 0; j < J; ++j) {
    dA_dpsi[j] = Eigen::Vector3d::UnitZ().cross(Eigen::Vector3d(M * y_rel[j]));
    const Eigen::Matrix3d Xi = -Rz * skew(Eigen::Vector3d(C * y_rel[j]));
    dA_dphi[j] = Xi.leftCols<2>();
  }

  Eigen::Vector3d t = t0;
  double isb[4];
  for (int k = 0; k < 4; ++k) isb[k] = isb0[k];
  int rejected = 0;
  std::vector<double> W(J), vbar(J), hpsibar(J);
  std::vector<Eigen::RowVector2d> hphibar(J);
  std::vector<int> col_of(4, -1);
  std::vector<int> col_k;
  Eigen::MatrixXd N, Ninv;
  Eigen::VectorXd rhs;
  std::vector<Eigen::VectorXd> gbar(J);
  int p = 3;

  auto linearise = [&]() {
    for (PsrRow& r : rows) {
      const SppInput& in = *r.in;
      const Eigen::Vector3d aN = M * y_rel[r.epoch] + t;
      const Eigen::Vector3d rA = anchor.r0_ecef + anchor.R_EN * aN;
      Eigen::Vector3d e;
      const double rho = rangeWithSagnac(in.state.r_ecef, rA, &e);
      const double iota = r.k >= 0 ? isb[r.k] : 0.0;
      const double h = rho + iota - kCLight * in.state.dts + in.ion_m + in.trop_m;
      r.v = in.P_corr - h;
      const Eigen::RowVector3d gE = rangeGradient(in.state.r_ecef, e);
      r.g_t = gE * anchor.R_EN;
      r.h_psi = r.g_t.dot(dA_dpsi[r.epoch]);
      r.h_phi = r.g_t * dA_dphi[r.epoch];
    }
  };
  // Observability of the ISBs: ι_k is seen when an epoch holds rows of system k together with rows of the base clock
  // or of a system already seen (transitively). Rows of an unseen system only determine b_j + ι_k and would bias the
  // epoch clock (b is the GPS clock, ι_G ≡ 0): they are left out.
  auto observability = [&](bool seen[4]) {
    for (int k = 0; k < 4; ++k) seen[k] = false;
    bool changed = true;
    while (changed) {
      changed = false;
      for (int j = 0; j < J; ++j) {
        bool has_base = false, has_seen = false, sys[4] = {false, false, false, false};
        for (const PsrRow& r : rows) {
          if (!r.active || r.epoch != j) continue;
          if (r.k < 0) has_base = true; else { sys[r.k] = true; if (seen[r.k]) has_seen = true; }
        }
        if (!(has_base || has_seen)) continue;
        for (int k = 0; k < 4; ++k)
          if (sys[k] && !seen[k]) {
            seen[k] = true;
            changed = true;
          }
      }
    }
  };
  auto reduce = [&](bool with_rhs) {
    N.setZero(p, p);
    rhs.setZero(p);
    for (int j = 0; j < J; ++j) {
      W[j] = 0.0;
      vbar[j] = 0.0;
      hpsibar[j] = 0.0;
      hphibar[j].setZero();
      gbar[j].setZero(p);
    }
    auto gvec = [&](const PsrRow& r) {
      Eigen::VectorXd g = Eigen::VectorXd::Zero(p);
      g.head<3>() = r.g_t.transpose();
      if (r.k >= 0 && col_of[r.k] >= 0) g(col_of[r.k]) = 1.0;
      return g;
    };
    for (const PsrRow& r : rows) {
      if (!r.active) continue;
      const double wi = 1.0 / r.var;
      W[r.epoch] += wi;
      vbar[r.epoch] += wi * r.v;
      hpsibar[r.epoch] += wi * r.h_psi;
      hphibar[r.epoch] += wi * r.h_phi;
      gbar[r.epoch] += wi * gvec(r);
    }
    for (int j = 0; j < J; ++j) {
      if (!(W[j] > 0.0)) continue;
      vbar[j] /= W[j];
      hpsibar[j] /= W[j];
      hphibar[j] /= W[j];
      gbar[j] /= W[j];
    }
    for (const PsrRow& r : rows) {
      if (!r.active) continue;
      const double wi = 1.0 / r.var;
      const Eigen::VectorXd gd = gvec(r) - gbar[r.epoch];
      N.noalias() += wi * gd * gd.transpose();
      if (with_rhs) rhs.noalias() += wi * gd * (r.v - vbar[r.epoch]);
    }
  };
  auto solveN = [&](Eigen::VectorXd* dx) -> bool {
    // Jacobi-scaled LDLT; the reduced normal matrix must be positive definite (t and every kept ι observable).
    Eigen::VectorXd d = N.diagonal();
    for (int i = 0; i < p; ++i) {
      if (!(d(i) > 0.0) || !std::isfinite(d(i))) return false;
      d(i) = 1.0 / std::sqrt(d(i));
    }
    const Eigen::MatrixXd Ns = d.asDiagonal() * N * d.asDiagonal();
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(0.5 * (Ns + Ns.transpose()));
    if (es.info() != Eigen::Success) return false;
    const double lmin = es.eigenvalues().minCoeff(), lmax = es.eigenvalues().maxCoeff();
    if (!(lmin > 1e-10 * lmax)) return false;
    Ninv = d.asDiagonal() * es.eigenvectors() * es.eigenvalues().cwiseInverse().asDiagonal() * es.eigenvectors().transpose() *
           d.asDiagonal();
    if (dx) *dx = Ninv * rhs;
    return Ninv.allFinite();
  };

  for (int round = 0; round <= m_all; ++round) {
    bool seen[4];
    observability(seen);
    col_k.clear();
    std::fill(col_of.begin(), col_of.end(), -1);
    p = 3;
    for (int k = 0; k < 4; ++k)
      if (seen[k]) {
        col_of[k] = p++;
        col_k.push_back(k);
      }
    for (PsrRow& r : rows)
      if (r.active && r.k >= 0 && !seen[r.k]) r.active = false;   // unobservable system: not a rejection
    bool any_base = false;
    for (const PsrRow& r : rows)
      if (r.active && r.k < 0) any_base = true;
    if (!any_base) {
      out.why = "trans: no GPS pseudorange in the 1 Hz epochs (b is the GPS clock)";
      return out;
    }
    // Gauss–Newton on θ = [t, ι_seen]
    bool conv = false;
    for (int it = 0; it < 20; ++it) {
      linearise();
      reduce(true);
      Eigen::VectorXd dx;
      if (!solveN(&dx)) {
        out.why = "trans: singular batch normal matrix (t or an ISB unobservable)";
        return out;
      }
      t += dx.head<3>();
      for (size_t c = 0; c < col_k.size(); ++c) isb[col_k[c]] += dx(3 + static_cast<int>(c));
      // 1 µm: the ranges are ~2e7 m, so steps below ~1e-8 m are rounding and a tighter test would never be met
      if (dx.norm() < 1e-6) {
        conv = true;
        break;
      }
    }
    linearise();
    reduce(false);
    if (!solveN(nullptr)) {
      out.why = "trans: singular batch normal matrix";
      return out;
    }
    if (!conv) {
      out.why = "trans: batch LS did not converge";
      return out;
    }
    // Iterated Baarda w-test: r_i = v_i − b̂_j, Q_v = var_i − [(g_i − ḡ_j)ᵀ N⁻¹ (g_i − ḡ_j) + 1/W_j].
    double w_max = 0.0;
    int worst = -1;
    for (int i = 0; i < m_all; ++i) {
      const PsrRow& r = rows[i];
      if (!r.active) continue;
      Eigen::VectorXd g = Eigen::VectorXd::Zero(p);
      g.head<3>() = r.g_t.transpose();
      if (r.k >= 0 && col_of[r.k] >= 0) g(col_of[r.k]) = 1.0;
      const Eigen::VectorXd gd = g - gbar[r.epoch];
      const double qv = r.var - (gd.dot(Ninv * gd) + 1.0 / W[r.epoch]);
      if (!(qv > 1e-9 * r.var)) continue;
      const double wv = std::fabs(r.v - vbar[r.epoch]) / std::sqrt(qv);
      if (wv > w_max) {
        w_max = wv;
        worst = i;
      }
    }
    if (worst < 0 || w_max <= kWThreshold) break;
    rows[worst].active = false;
    ++rejected;
    if (rejected > kMaxRejectFrac * m_all) {
      out.why = "trans: more than 30 % of the pseudorange rows rejected";
      return out;
    }
  }

  // post-fit statistics: m rows − one clock per epoch with rows − dim θ
  double ssr_w = 0.0, ssr = 0.0;
  int m = 0, ep_used = 0;
  std::vector<int> ep_rows(J, 0);
  for (const PsrRow& r : rows) {
    if (!r.active) continue;
    const double res = r.v - vbar[r.epoch];
    ssr_w += res * res / r.var;
    ssr += res * res;
    ++m;
    ++ep_rows[r.epoch];
  }
  for (int j = 0; j < J; ++j)
    if (ep_rows[j] > 0) ++ep_used;
  const int dof = m - ep_used - p;
  if (dof < 1) {
    out.why = "trans: no redundancy in the pseudorange rows (" + std::to_string(m) + " rows, " + std::to_string(ep_used) +
              " epochs)";
    return out;
  }
  out.s0sq = ssr_w / dof;
  out.rms_m = std::sqrt(ssr / dof);
  out.scale = std::max(1.0, out.s0sq);
  out.rows = m;
  out.rejected = rejected;
  out.epochs = ep_used;
  out.t = t;
  for (int k = 0; k < 4; ++k) {
    out.observed[k] = col_of[k] >= 0;
    out.isb[k] = out.observed[k] ? isb[k] : 0.0;
  }
  out.col_k = col_k;
  out.Ninv = Ninv;
  // clock epoch: the newest epoch with rows
  for (int j = J - 1; j >= 0; --j)
    if (ep_rows[j] > 0) {
      out.jb = j;
      break;
    }
  const int jb = out.jb;
  out.b_jb = vbar[jb];
  out.invW_jb = 1.0 / W[jb];
  out.gbar_jb = gbar[jb];
  out.hpsi_bar_jb = hpsibar[jb];
  out.hphi_bar_jb = hphibar[jb];
  // sensitivities of θ̂ to ψ and φ: N dθ = −Σ w (g − ḡ_j) ∂h/∂(ψ, φ)
  Eigen::VectorXd spsi = Eigen::VectorXd::Zero(p);
  Eigen::MatrixXd sphi = Eigen::MatrixXd::Zero(p, 2);
  for (const PsrRow& r : rows) {
    if (!r.active) continue;
    const double wi = 1.0 / r.var;
    Eigen::VectorXd g = Eigen::VectorXd::Zero(p);
    g.head<3>() = r.g_t.transpose();
    if (r.k >= 0 && col_of[r.k] >= 0) g(col_of[r.k]) = 1.0;
    const Eigen::VectorXd gd = g - gbar[r.epoch];
    spsi += wi * gd * r.h_psi;
    sphi += wi * gd * r.h_phi;
  }
  out.dtheta_dpsi = -Ninv * spsi;
  out.dtheta_dphi = -Ninv * sphi;
  out.ok = out.t.allFinite() && out.Ninv.allFinite();
  if (!out.ok) out.why = "trans: non-finite batch solution";
  return out;
}

// Formal covariance of the drift of one epoch's Doppler LS, recomputed from its rows with the a-priori variances
// (RobustDopplerLs::Q is scaled by σ̂0², which is 0 for an exact fit): Var(d) = [(HᵀWH)⁻¹]_dd · max(1, σ̂0²).
double dopplerDriftVariance(const CollectSample& s) {
  const DopplerSolution& d = s.dop;
  Eigen::Matrix4d Nm = Eigen::Matrix4d::Zero();
  int m = 0;
  for (const SppInput& in : s.raw) {
    if (!contains(d.used, in.sat)) continue;
    if (!in.state.ok) continue;
    Eigen::Vector3d e;
    rangeWithSagnac(in.state.r_ecef, s.spp.r_ecef, &e);
    Eigen::RowVector4d h;
    h << -e.x() - kOmgE / kCLight * in.state.r_ecef.y(), -e.y() + kOmgE / kCLight * in.state.r_ecef.x(), -e.z(), 1.0;
    const double w = 1.0 / (in.var_dopp > 0.0 ? in.var_dopp : 1.0);
    Nm.noalias() += w * h.transpose() * h;
    ++m;
  }
  if (m < 4) return std::numeric_limits<double>::quiet_NaN();
  Eigen::FullPivLU<Eigen::Matrix4d> lu(Nm);
  if (!lu.isInvertible()) return std::numeric_limits<double>::quiet_NaN();
  const Eigen::Matrix4d Q = lu.inverse();
  return Q(3, 3) * std::max(1.0, d.sigma0 * d.sigma0);
}

View viewOf(const std::vector<CollectSample>& v) {
  View w;
  w.reserve(v.size());
  for (const CollectSample& s : v) w.push_back(&s);
  return w;
}

// ---------------------------------------------------------------------------------------------
// Shared by both ALIGN paths: the lever arm, the pivot o^W = a^W at k_a and ∂t/∂δx = −J_a,host there (§8 ACTIVATE).
// ---------------------------------------------------------------------------------------------
struct PivotFrame {
  Eigen::Vector3d lever = Eigen::Vector3d::Zero();
  Eigen::Vector3d pivot_W = Eigen::Vector3d::Zero();
  Eigen::Matrix<double, 3, HostIdx::dim> J_trans_host = Eigen::Matrix<double, 3, HostIdx::dim>::Zero();
};

// l^I from the facade's own evaluation (y = p_g + R_g l, ImuBridge::evaluate) and o^W = a^W at k_a for the current host.
Eigen::Vector3d leverOf(const CollectSample& last) { return last.host_ant.R_g.transpose() * (last.host_ant.y - last.host_ant.p_g); }
Eigen::Vector3d pivotOf(const Config& cfg, bool gravity_est_en, const CoreState& core, const CollectSample& last) {
  const bool grav = gravity_est_en && cfg.frames.gnss_updates_gravity;
  return ImuBridge::evaluate(core, last.bridge, leverOf(last), grav).y;
}

PivotFrame pivotAt(const Config& cfg, const Leveling& leveling, bool gravity_est_en, const CoreState& core,
                   const CollectSample& last, const AnchorFrame& anc, const Eigen::Matrix3d& Rz) {
  PivotFrame pf;
  pf.lever = leverOf(last);
  pf.pivot_W = pivotOf(cfg, gravity_est_en, core, last);   // o^W = current a^W (§8)
  NoiseModel noise(cfg);
  RowBuilder rb(cfg, noise);
  LinPoint lp;
  lp.x = &core;
  lp.bridge = &last.bridge;
  lp.aug = nullptr;             // ψ = φ = 0 there: M = C(g); R_z(ψ) is applied below (every host column is M·(…))
  lp.leveling = &leveling;
  lp.anchor = &anc;
  lp.lever_I = pf.lever;
  lp.pivot_W = pf.pivot_W;
  lp.gravity_est_en = gravity_est_en;
  lp.delta = last.bridge.delta;
  const AntennaJacobians J = rb.antennaJacobians(lp);
  pf.J_trans_host = -(Rz * J.J_a.leftCols<HostIdx::dim>());
  return pf;
}

// =============================================================================================
// AMENDMENTS A51: LIO-aided multi-epoch initialisation (the second ALIGN path)
//
// Rows (window k_0 … k_a, every sample that has a bridge, whatever its number of satellites):
//   code  (1 Hz per satellite):  P_corr = ρ(r_s, r_A,j) + b + d·τ_j + ½a·τ_j² + ι_s − c·δt_s + I + T
//   Dopp  (every epoch):         −λD   = e·(v_s − ṙ_A,j) + Sagnac rate + d_j − c·δṫ_s     (d_j eliminated per epoch)
//   r_A,j = r_0 + R_EN (M y_j + t),  ṙ_A,j = R_EN M w_j,  M = R_z(ψ)·C(g) (φ = 0),  y_j = a^W_j − o^W,  τ_j = t_g,j − t_k
// θ = [ψ, t(3), b, d, a, ι_k of every system with code rows]; the host's relative motion (y_j, w_j) is known, its bridge
// noise Σ_D enters the row variances (u·MΣ_DMᵀ·u, acc_n²δ). Stages: (A) a coarse static-antenna code LS for the position
// and clock (cold start from the satellites' mean direction: a few-satellite window never had an SPP fix), which also
// fixes the working N frame when none is fixed yet; (B) ψ₀ in closed form from the Doppler rows, which are linear in
// (cos ψ, sin ψ) once each epoch is centred on its mean (e·R_EN R_z(ψ) c = α(u_x c_x + u_y c_y) + β(u_y c_x − u_x c_y) +
// u_z c_z); (C) the joint Gauss–Newton on θ with the atmosphere, elevation and §4.7 variances re-evaluated at the antenna
// positions, elevation mask set once at the start, iterated Baarda w-test.
// =============================================================================================
constexpr double kFreqL1 = 1.57542e9;
constexpr double kErrIonStd = 5.0;       // rows.cpp / pntpos.c ERR_ION: σ_ion without Klobuchar (§8 WAIT_NAV)
constexpr double kErrSaas = 0.3;         // rows.cpp / pntpos.c ERR_SAAS
constexpr double kRelHumi = 0.7;         // rows.cpp / pntpos.c REL_HUMI
constexpr double kEarthRadius = 6371000.0;
constexpr double kLaMaxGapS = 30.0;      // clock-continuity check: the same satellite seen again within this interval

struct LaEpoch {
  const CollectSample* s = nullptr;
  Eigen::Vector3d y = Eigen::Vector3d::Zero();      // a^W_j − o^W
  Eigen::Vector3d w = Eigen::Vector3d::Zero();      // w^W_j
  double tau = 0.0;                                  // t_g,j − t_k (clock model time)
  bool code = false;                                 // first epoch of its GPST second (A43)
  bool models_given = false;                         // no receiver stds: variances/atmosphere as delivered
  // at the current iterate
  Eigen::Vector3d aN = Eigen::Vector3d::Zero(), wN = Eigen::Vector3d::Zero();
  Eigen::Vector3d rA = Eigen::Vector3d::Zero(), rdot = Eigen::Vector3d::Zero();
};

struct LaRow {
  int ep = 0, idx = 0;                               // epoch, index into CollectSample::raw
  bool code = false;                                 // pseudorange (true) or Doppler
  int k = -1;                                        // ISB index of a code row (−1 = base clock)
  bool active = true;                                // membership (elevation mask, w-test)
  bool rejected = false;
  double var = 1.0, ion = 0.0, trop = 0.0, el = 0.5 * M_PI;
  double v = 0.0;                                    // residual z − h(θ)
  Eigen::RowVectorXd h;                              // ∂h/∂θ
  Eigen::RowVector2d hphi = Eigen::RowVector2d::Zero();   // ∂h/∂φ at φ = 0
  const SppInput& in() const;
  const CollectSample* s = nullptr;
};
const SppInput& LaRow::in() const { return s->raw[static_cast<size_t>(idx)]; }

// Clock and ψ modelling of the joint LS. A51 as written is {dopp_epoch_d = false, code_quad = false}: one drift d shared
// by every Doppler row and the linear code clock b + d·τ, "the residual covered by κ_AR". On the dev set that is not
// what the receiver does: its drift wanders by 0.1–0.25 m/s within a minute (Grass02 tracking, 61.1 → 59.8 m/s over
// 160 s), a slow common-mode error that κ_AR(τ_D = 0.2 s) does not cover and that is as large as the heading signal of a
// walking-speed platform (v·|u_h| ≈ 0.5 m/s), so the shared d drags ψ. The implemented model therefore gives every
// epoch's Doppler rows their own drift d_j, eliminated per epoch (ψ then comes from the line-of-sight differences within
// an epoch, exactly §8's yaw fit "每历元 d_j 投影掉"), and the code rows the quadratic clock b + d·τ + ½a·τ² (A51's
// "二次模型"; the remaining wander of b is metres, inside the code noise, i.e. κ_AR(τ_ρ) as A51 says). `code_psi` = false
// (diagnostics only) takes the lever-arm ψ sensitivity out of the code rows: ψ from the Doppler rows alone.
struct LaModel {
  bool dopp_epoch_d = true;
  bool code_quad = true;
  bool code_psi = true;   // false: the code rows' lever arms keep the rotation LaProblem::psi_code (diagnostics)
};

struct LaDoppEpoch {   // weighted means of one epoch's active Doppler rows (per-epoch drift elimination)
  double W = 0.0, vbar = 0.0;
  Eigen::VectorXd hbar;
  Eigen::RowVector2d hphibar = Eigen::RowVector2d::Zero();
  int n = 0;
};

struct LaProblem {
  const Config* cfg = nullptr;
  const NoiseModel* noise = nullptr;
  std::vector<LaEpoch> ep;
  std::vector<LaRow> rows;
  AnchorFrame anc;
  Eigen::Matrix3d C = Eigen::Matrix3d::Identity();
  double sigma_toff = 0.0;
  LaModel model;
  // unknowns θ = [ψ, t(3), b, d, (a), ι_{col_k[0]}, …]: b, d, a = the code clock at t_k, its drift and drift rate
  std::vector<int> col_k;
  int col_of[4] = {-1, -1, -1, -1};
  int col_a = -1;
  int p = 6;
  double psi = 0.0, b = 0.0, d = 0.0, a = 0.0;
  double psi_code = 0.0;   // rotation of the code rows' lever arms when !model.code_psi
  Eigen::Vector3d t = Eigen::Vector3d::Zero();
  double isb[4] = {0, 0, 0, 0};
  // last normal equations (active rows) and the per-epoch Doppler means they were reduced with
  Eigen::MatrixXd N, Ninv;
  Eigen::VectorXd rhs;
  std::vector<LaDoppEpoch> de;
};

bool rowUsable(const SppInput& in, bool code) {
  if (!in.state.ok) return false;
  if (code) return in.psr_usable && in.P_corr > 0.0 && std::isfinite(in.P_corr);
  return in.dopp_usable && in.lambda_m > 0.0 && in.dopp_hz != 0.0 && std::isfinite(in.dopp_hz) && in.state.v_ecef.norm() > 0.0;
}

// A sample can serve the A51 window when it holds at least one usable row.
bool hasLaRows(const Config& cfg, const CollectSample& s) {
  for (const SppInput& in : s.raw) {
    int k = -1;
    if (rowUsable(in, true) && batchCodeSystem(cfg, in.sat.sys, &k)) return true;
    if (rowUsable(in, false)) return true;
  }
  return false;
}

// Receiver clock continuity between a new sample and the window (A51's clock model b(τ) = b + d·τ has no jumps): the
// pseudorange of a satellite seen again within kLaMaxGapS must follow its Doppler, P_new ≈ P_old − λ(D_old + D_new)/2·Δτ
// (−λD is the total range rate incl. both clock drifts). A millisecond jump moves every pseudorange by ~3e5 m; the
// integration error over 30 s is ~10² m. Returns the median discontinuity over the satellites compared (0 when none).
double clockDiscontinuity(const std::deque<CollectSample>& win, const CollectSample& s, int* compared) {
  std::vector<double> jumps;
  for (const SppInput& a : s.raw) {
    if (!rowUsable(a, true) || !(a.lambda_m > 0.0) || !std::isfinite(a.dopp_hz) || a.dopp_hz == 0.0) continue;
    for (auto it = win.rbegin(); it != win.rend(); ++it) {
      const double dt = s.t_rx.diff(it->t_rx);
      if (dt > kLaMaxGapS) break;
      const SppInput* b = nullptr;
      for (const SppInput& x : it->raw)
        if (x.sat == a.sat) {
          b = &x;
          break;
        }
      if (!b) continue;
      if (!rowUsable(*b, true) || !std::isfinite(b->dopp_hz) || b->dopp_hz == 0.0) break;
      const double pred = b->P_corr - a.lambda_m * 0.5 * (a.dopp_hz + b->dopp_hz) * dt;
      jumps.push_back(a.P_corr - pred);
      break;
    }
  }
  if (compared) *compared = static_cast<int>(jumps.size());
  return jumps.empty() ? 0.0 : median(jumps);
}

// Solves the Jacobi-scaled normal equations; false when not positive definite (a parameter without information).
bool solveNormals(const Eigen::MatrixXd& N, const Eigen::VectorXd& rhs, Eigen::MatrixXd* Ninv, Eigen::VectorXd* dx) {
  const int p = static_cast<int>(N.rows());
  Eigen::VectorXd dg = N.diagonal();
  for (int i = 0; i < p; ++i) {
    if (!(dg(i) > 0.0) || !std::isfinite(dg(i))) return false;
    dg(i) = 1.0 / std::sqrt(dg(i));
  }
  const Eigen::MatrixXd Ns = dg.asDiagonal() * N * dg.asDiagonal();
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(0.5 * (Ns + Ns.transpose()));
  if (es.info() != Eigen::Success) return false;
  const double lmin = es.eigenvalues().minCoeff(), lmax = es.eigenvalues().maxCoeff();
  if (!(lmin > 1e-10 * lmax)) return false;
  *Ninv = dg.asDiagonal() * es.eigenvectors() * es.eigenvalues().cwiseInverse().asDiagonal() * es.eigenvectors().transpose() *
          dg.asDiagonal();
  if (dx) *dx = (*Ninv) * rhs;
  return Ninv->allFinite() && (!dx || dx->allFinite());
}

// ---- stage A: coarse static-antenna code LS for position, clock and ISBs (cold start supported) ----
struct CoarseFix {
  bool ok = false;
  Eigen::Vector3d r = Eigen::Vector3d::Zero();
  double b = 0.0, d = 0.0, rms = 0.0;
  double isb[4] = {0, 0, 0, 0};
  int iterations = 0;
};

CoarseFix laCoarse(const LaProblem& P, bool have_start, const Eigen::Vector3d& r_start, std::string* why) {
  CoarseFix f;
  std::vector<const LaRow*> rows;
  for (const LaRow& r : P.rows)
    if (r.code && r.active) rows.push_back(&r);
  int col_of[4] = {-1, -1, -1, -1};
  int p = 5;
  for (const LaRow* r : rows)
    if (r->k >= 0 && col_of[r->k] < 0) col_of[r->k] = p++;
  const int m = static_cast<int>(rows.size());
  if (m < p + 1) {
    *why = "coarse fix: " + std::to_string(m) + " code rows for " + std::to_string(p) + " unknowns";
    return f;
  }
  Eigen::Vector3d r = r_start;
  if (!have_start) {
    // Cold start: every satellite with a row is above the receiver's horizon, so their mean direction from the Earth's
    // centre is within a few tens of degrees of the receiver's zenith (the Gauss–Newton below converges from there).
    Eigen::Vector3d u = Eigen::Vector3d::Zero();
    for (const LaRow* rr : rows) u += rr->in().state.r_ecef.normalized();
    if (!(u.norm() > 0.0)) {
      *why = "coarse fix: no satellite direction";
      return f;
    }
    r = kEarthRadius * u.normalized();
  }
  Eigen::VectorXd x = Eigen::VectorXd::Zero(p);
  x.head<3>() = r;
  {
    std::vector<double> b0;
    for (const LaRow* rr : rows) {
      const SppInput& in = rr->in();
      b0.push_back(in.P_corr + kCLight * in.state.dts - rangeWithSagnac(in.state.r_ecef, r, nullptr));
    }
    x(3) = median(b0);
  }
  bool conv = false;
  Eigen::MatrixXd N(p, p), Ninv;
  Eigen::VectorXd rhs(p), dx;
  for (int it = 0; it < 60; ++it) {
    N.setZero();
    rhs.setZero();
    const Eigen::Vector3d rr = x.head<3>();
    for (const LaRow* row : rows) {
      const SppInput& in = row->in();
      Eigen::Vector3d e;
      const double rho = rangeWithSagnac(in.state.r_ecef, rr, &e);
      const double iota = row->k >= 0 ? x(col_of[row->k]) : 0.0;
      const double v = in.P_corr - (rho + x(3) + x(4) * P.ep[row->ep].tau + iota - kCLight * in.state.dts);
      Eigen::VectorXd h = Eigen::VectorXd::Zero(p);
      h.head<3>() = rangeGradient(in.state.r_ecef, e).transpose();
      h(3) = 1.0;
      h(4) = P.ep[row->ep].tau;
      if (row->k >= 0) h(col_of[row->k]) = 1.0;
      N.noalias() += h * h.transpose();   // unit weights: a start point, the variances come in stage C
      rhs.noalias() += h * v;
    }
    if (!solveNormals(N, rhs, &Ninv, &dx)) {
      *why = "coarse fix: singular code geometry (position, clock and ISBs not separable)";
      return f;
    }
    x += dx;
    ++f.iterations;
    if (dx.head<3>().norm() < 1e-3 && std::fabs(dx(3)) < 1e-3) {
      conv = true;
      break;
    }
  }
  if (!conv || !x.allFinite()) {
    *why = "coarse fix: no convergence";
    return f;
  }
  f.r = x.head<3>();
  const double rn = f.r.norm();
  if (!(rn > 6.2e6 && rn < 6.6e6)) {
    *why = "coarse fix off the Earth (|r| = " + fmt(rn / 1e3, 0) + " km)";
    return f;
  }
  double ssr = 0.0;
  for (const LaRow* row : rows) {
    const SppInput& in = row->in();
    const double iota = row->k >= 0 ? x(col_of[row->k]) : 0.0;
    const double v = in.P_corr - (rangeWithSagnac(in.state.r_ecef, f.r, nullptr) + x(3) + x(4) * P.ep[row->ep].tau + iota -
                                  kCLight * in.state.dts);
    ssr += v * v;
  }
  f.rms = std::sqrt(ssr / std::max(1, m - p));
  f.b = x(3);
  f.d = x(4);
  for (int k = 0; k < 4; ++k) f.isb[k] = col_of[k] >= 0 ? x(col_of[k]) : 0.0;
  f.ok = true;
  return f;
}

// ---- stage B: ψ₀ and d₀ from the Doppler rows, linear in (α, β, d) = (cos ψ, sin ψ, d) at a known position ----
// With per-epoch drifts (LaModel::dopp_epoch_d) the unknown d is one per epoch: every epoch's rows are centred on their
// weighted mean before the 2-unknown fit of (α, β); *d0 is then left unchanged.
bool laYawStart(const LaProblem& P, const Eigen::Vector3d& r, double* psi0, double* d0, std::string* why) {
  if (P.model.dopp_epoch_d) {
    struct Acc {
      double W = 0.0, a = 0.0, b = 0.0, y = 0.0;
      std::vector<std::array<double, 4>> rows;   // (w, −A, −B, target)
    };
    std::vector<Acc> acc(P.ep.size());
    for (const LaRow& row : P.rows) {
      if (row.code || !row.active) continue;
      const SppInput& in = row.in();
      Eigen::Vector3d e;
      rangeWithSagnac(in.state.r_ecef, r, &e);
      const Eigen::Vector3d& vs = in.state.v_ecef;
      const Eigen::Vector3d u = P.anc.R_EN.transpose() * e;
      const Eigen::Vector3d c = P.C * P.ep[row.ep].w;
      const double y = -in.lambda_m * in.dopp_hz - (e.dot(vs) + kOmgE / kCLight * (vs.x() * r.y() - vs.y() * r.x())) +
                       kCLight * in.state.ddts + u.z() * c.z();
      const double A = u.x() * c.x() + u.y() * c.y();
      const double B = u.y() * c.x() - u.x() * c.y();
      const double wgt = 1.0 / (row.var > 0.0 ? row.var : 1.0);
      Acc& q = acc[static_cast<size_t>(row.ep)];
      q.W += wgt;
      q.a += wgt * (-A);
      q.b += wgt * (-B);
      q.y += wgt * y;
      q.rows.push_back({wgt, -A, -B, y});
    }
    Eigen::Matrix2d N = Eigen::Matrix2d::Zero();
    Eigen::Vector2d rhs = Eigen::Vector2d::Zero();
    int m = 0;
    for (const Acc& q : acc) {
      if (q.rows.size() < 2) continue;   // one row: its epoch drift absorbs it
      const double ma = q.a / q.W, mb = q.b / q.W, my = q.y / q.W;
      for (const auto& x : q.rows) {
        const Eigen::Vector2d h(x[1] - ma, x[2] - mb);
        N.noalias() += x[0] * h * h.transpose();
        rhs.noalias() += x[0] * h * (x[3] - my);
        ++m;
      }
    }
    if (m < 3) {
      *why = "yaw start: fewer than 3 Doppler rows in epochs with two or more Doppler satellites";
      return false;
    }
    Eigen::MatrixXd Ninv;
    Eigen::VectorXd sol;
    if (!solveNormals(N, rhs, &Ninv, &sol) || !(std::hypot(sol(0), sol(1)) > 0.0)) {
      *why = "yaw start: the within-epoch Doppler differences carry no heading information";
      return false;
    }
    *psi0 = std::atan2(sol(1), sol(0));
    return true;
  }
  Eigen::Matrix3d N = Eigen::Matrix3d::Zero();
  Eigen::Vector3d rhs = Eigen::Vector3d::Zero();
  int m = 0;
  for (const LaRow& row : P.rows) {
    if (row.code || !row.active) continue;
    const SppInput& in = row.in();
    Eigen::Vector3d e;
    rangeWithSagnac(in.state.r_ecef, r, &e);
    const Eigen::Vector3d& vs = in.state.v_ecef;
    const Eigen::Vector3d u = P.anc.R_EN.transpose() * e;
    const Eigen::Vector3d c = P.C * P.ep[row.ep].w;
    // −λD − e·v_s − (ω/c)(ẋ_s y_A − ẏ_s x_A) + c·δṫ_s = −(α A + β B + u_z c_z) + d   (the ṙ_A part of the Sagnac rate,
    // ~1e-5 m/s, is left to stage C)
    const double y = -in.lambda_m * in.dopp_hz - (e.dot(vs) + kOmgE / kCLight * (vs.x() * r.y() - vs.y() * r.x())) +
                     kCLight * in.state.ddts;
    const double A = u.x() * c.x() + u.y() * c.y();
    const double B = u.y() * c.x() - u.x() * c.y();
    const Eigen::Vector3d h(-A, -B, 1.0);
    const double wgt = 1.0 / (row.var > 0.0 ? row.var : 1.0);
    N.noalias() += wgt * h * h.transpose();
    rhs.noalias() += wgt * h * (y + u.z() * c.z());
    ++m;
  }
  if (m < 3) {
    *why = "yaw start: fewer than 3 Doppler rows";
    return false;
  }
  Eigen::MatrixXd Ninv;
  Eigen::VectorXd sol;
  if (!solveNormals(N, rhs, &Ninv, &sol)) {
    *why = "yaw start: the Doppler rows carry no heading information (host velocity ~0 or one direction)";
    return false;
  }
  if (!(std::hypot(sol(0), sol(1)) > 0.0)) {
    *why = "yaw start: degenerate (cos ψ, sin ψ) fit";
    return false;
  }
  *psi0 = std::atan2(sol(1), sol(0));
  *d0 = sol(2);
  return true;
}

// ---- stage C pieces ----
void laLayout(LaProblem& P) {
  bool present[4] = {false, false, false, false};
  for (const LaRow& r : P.rows)
    if (r.code && r.active && r.k >= 0) present[r.k] = true;
  P.col_k.clear();
  P.p = 6;
  P.col_a = P.model.code_quad ? P.p++ : -1;
  for (int k = 0; k < 4; ++k) {
    P.col_of[k] = present[k] ? P.p++ : -1;
    if (present[k]) P.col_k.push_back(k);
  }
}

// Antenna position/velocity of every epoch at the current θ (φ = 0).
void laAntennas(LaProblem& P) {
  const Eigen::Matrix3d M = Leveling::Rz(P.psi) * P.C;
  for (LaEpoch& e : P.ep) {
    e.aN = M * e.y + P.t;
    e.wN = M * e.w;
    e.rA = P.anc.r0_ecef + P.anc.R_EN * e.aN;
    e.rdot = P.anc.R_EN * e.wN;
  }
}

// Elevation, atmosphere and §4.7 variances of every row at the current antenna positions. `membership`: also apply the
// L0 elevation mask (once, at the first evaluation of stage C: the membership then stays fixed while θ converges).
void laModels(LaProblem& P, bool membership) {
  laAntennas(P);
  const Config& cfg = *P.cfg;
  const double el_mask = cfg.masks.el_deg * kDeg;
  const Eigen::Matrix3d M = Leveling::Rz(P.psi) * P.C;
  std::vector<Geodetic> geo(P.ep.size());
  for (size_t j = 0; j < P.ep.size(); ++j) geo[j] = rtk::ecef2pos(P.ep[j].rA);
  for (LaRow& row : P.rows) {
    if (row.rejected) continue;
    const LaEpoch& E = P.ep[row.ep];
    const CollectSample& s = *E.s;
    const SppInput& in = row.in();
    Eigen::Vector3d e;
    rangeWithSagnac(in.state.r_ecef, E.rA, &e);
    double az = 0.0, el = 0.0;
    rtk::satAzel(geo[row.ep], e, &az, &el);
    const bool fixed = in.models_fixed;
    if (fixed) el = in.el_rad;
    row.el = el;
    if (membership) row.active = el >= el_mask - 1e-12;
    const Eigen::Vector3d u = P.anc.R_EN.transpose() * e;
    if (row.code) {
      const double uw = u.dot(E.wN) * P.sigma_toff;
      const double toff_term = uw * uw;
      const double bridge_term = u.dot(M * s.bridge.Sigma_D * M.transpose() * u);
      if (fixed || E.models_given) {
        row.ion = in.ion_m;
        row.trop = in.trop_m;
        row.var = (in.var_psr > 0.0 ? in.var_psr : 1.0) + bridge_term + toff_term;
      } else {
        const double f = kCLight / in.lambda_m;
        const double kk = (kFreqL1 / f) * (kFreqL1 / f);
        double var_ion = kErrIonStd * kErrIonStd;
        row.ion = 0.0;
        if (s.ion_valid && s.ion.valid()) {
          const double i1 = rtk::ionModelGpsL1(s.t_rx, s.ion, geo[row.ep], az, el);
          row.ion = i1 * kk;
          var_ion = (cfg.noise.psr.err_brdci * i1 * kk) * (cfg.noise.psr.err_brdci * i1 * kk);
        }
        row.trop = rtk::tropModel(s.t_rx, geo[row.ep], az, el, kRelHumi);
        const double se = std::sin(el) + 0.1;
        NoiseModel::PsrTerms pt;
        pt.el_rad = el;
        pt.cn0 = in.cn0;
        pt.psr_std_m = s.psr_std_m[static_cast<size_t>(row.idx)];
        pt.sys = in.sat.sys;
        pt.bds_geo = QualityGate::isBdsGeo(in.sat);
        pt.var_eph = in.state.var_eph;
        pt.var_ion = var_ion;
        pt.var_trop = (kErrSaas / se) * (kErrSaas / se);
        pt.bridge_term = bridge_term;
        pt.clock_bridge = 0.0;   // the clock model is evaluated at the epoch's own time
        pt.toff_term = toff_term;
        pt.dt_since_last_s = cfg.noise.psr.decim_s > 0.0 ? cfg.noise.psr.decim_s : 1.0;   // §4.7 κ_AR(τ_ρ, 1 s)
        row.var = P.noise->psrVar(pt);
      }
    } else {
      if (fixed || E.models_given) {
        row.var = (in.var_dopp > 0.0 ? in.var_dopp : 1.0) + s.bridge.var_v_bridge;
      } else {
        NoiseModel::DoppTerms dt;
        dt.el_rad = el;
        dt.lambda_m = in.lambda_m;
        dt.dopp_std_hz = s.dopp_std_hz[static_cast<size_t>(row.idx)];
        dt.delta_s = 0.0;                      // d is modelled over the window, not bridged from t_k
        dt.var_v_bridge = s.bridge.var_v_bridge;
        dt.omega_norm = s.host_ant.omega_g.norm();
        dt.ant_acc_norm = 0.0;                 // not available per sample; (‖α‖σ_toff)² ≲ 1e-3 m²/s²
        dt.v_norm = E.w.norm();
        dt.dt_since_last_s = 0.1;              // §4.7 κ_AR(τ_D, 0.1 s)
        row.var = P.noise->doppVar(dt);
      }
    }
    if (!(row.var > 0.0) || !std::isfinite(row.var)) row.active = false;
  }
}

// Residuals and Jacobians of every active row at the current θ (models fixed).
void laLinearise(LaProblem& P) {
  laAntennas(P);
  const Eigen::Matrix3d Rz = Leveling::Rz(P.psi);
  const Eigen::Matrix3d M = Rz * P.C;
  const Eigen::Vector3d ez = Eigen::Vector3d::UnitZ();
  const Eigen::Matrix3d& REN = P.anc.R_EN;
  for (LaRow& row : P.rows) {
    if (!row.active) continue;
    const LaEpoch& E = P.ep[row.ep];
    const SppInput& in = row.in();
    const Eigen::Vector3d& rs = in.state.r_ecef;
    Eigen::Vector3d e;
    const double rho = rangeWithSagnac(rs, E.rA, &e);
    // ∂a^N/∂ψ = [e_z]× M y, ∂a^N/∂φ = Ξ(y) E_xy with Ξ(v) = −R_z [C v]× (φ = 0)
    const Eigen::Vector3d dA_dpsi = ez.cross(Eigen::Vector3d(M * E.y));
    const Eigen::Matrix<double, 3, 2> dA_dphi = (-Rz * skew(Eigen::Vector3d(P.C * E.y))).leftCols<2>();
    row.h = Eigen::RowVectorXd::Zero(P.p);
    if (row.code && !P.model.code_psi) {
      // diagnostics: the lever arm rotated with the frozen ψ_code (the rows then carry no ψ information at all)
      const Eigen::Vector3d aN = Leveling::Rz(P.psi_code) * P.C * E.y + P.t;
      const Eigen::Vector3d rA = P.anc.r0_ecef + P.anc.R_EN * aN;
      const double rho_c = rangeWithSagnac(rs, rA, &e);
      const double iota = row.k >= 0 && P.col_of[row.k] >= 0 ? P.isb[row.k] : 0.0;
      const double clk = P.b + P.d * E.tau + (P.col_a >= 0 ? 0.5 * P.a * E.tau * E.tau : 0.0);
      row.v = in.P_corr - (rho_c + clk + iota - kCLight * in.state.dts + row.ion + row.trop);
      const Eigen::RowVector3d gN = rangeGradient(rs, e) * REN;
      row.h.segment<3>(1) = gN;
      row.h(4) = 1.0;
      row.h(5) = E.tau;
      if (P.col_a >= 0) row.h(P.col_a) = 0.5 * E.tau * E.tau;
      if (row.k >= 0 && P.col_of[row.k] >= 0) row.h(P.col_of[row.k]) = 1.0;
      row.hphi.setZero();
    } else if (row.code) {
      const double iota = row.k >= 0 && P.col_of[row.k] >= 0 ? P.isb[row.k] : 0.0;
      const double clk = P.b + P.d * E.tau + (P.col_a >= 0 ? 0.5 * P.a * E.tau * E.tau : 0.0);
      const double h = rho + clk + iota - kCLight * in.state.dts + row.ion + row.trop;
      row.v = in.P_corr - h;
      const Eigen::RowVector3d gN = rangeGradient(rs, e) * REN;   // ∂ρ/∂a^N
      row.h(0) = gN.dot(dA_dpsi);
      row.h.segment<3>(1) = gN;
      row.h(4) = 1.0;
      row.h(5) = E.tau;
      if (P.col_a >= 0) row.h(P.col_a) = 0.5 * E.tau * E.tau;
      if (row.k >= 0 && P.col_of[row.k] >= 0) row.h(P.col_of[row.k]) = 1.0;
      row.hphi = gN * dA_dphi;
    } else {
      const Eigen::Vector3d& vs = in.state.v_ecef;
      const Eigen::Vector3d& rA = E.rA;
      const Eigen::Vector3d& rd = E.rdot;
      const Eigen::Vector3d dv = vs - rd;
      const double rho_g = (rs - rA).norm();
      const double rate = e.dot(dv) + kOmgE / kCLight * (vs.x() * rA.y() + rs.x() * rd.y() - vs.y() * rA.x() - rs.y() * rd.x());
      // the shared drift d, or none: the epoch's own d_j is eliminated in laNormals()
      const double h = rate + (P.model.dopp_epoch_d ? 0.0 : P.d) - kCLight * in.state.ddts;
      row.v = -in.lambda_m * in.dopp_hz - h;
      // ∂rate/∂ṙ_A = −eᵀ + (ω_e/c)(−y_s, x_s, 0); ∂rate/∂r_A = −(Δv − e(e·Δv))ᵀ/ρ + (ω_e/c)(−ẏ_s, ẋ_s, 0)
      const Eigen::RowVector3d gD = rangeGradient(rs, e) * REN;
      const Eigen::RowVector3d gP =
          (Eigen::RowVector3d(-(dv - e * e.dot(dv)).transpose() / rho_g) +
           Eigen::RowVector3d(-kOmgE / kCLight * vs.y(), kOmgE / kCLight * vs.x(), 0.0)) *
          REN;
      const Eigen::Vector3d dW_dpsi = ez.cross(E.wN);
      const Eigen::Matrix<double, 3, 2> dW_dphi = (-Rz * skew(Eigen::Vector3d(P.C * E.w))).leftCols<2>();
      row.h(0) = gD.dot(dW_dpsi) + gP.dot(dA_dpsi);
      row.h.segment<3>(1) = gP;
      if (!P.model.dopp_epoch_d) row.h(5) = 1.0;
      row.hphi = gD * dW_dphi + gP * dA_dphi;
    }
  }
}

// Per-epoch weighted means of the active Doppler rows (per-epoch drift elimination), at the current linearisation.
void laDoppMeans(LaProblem& P) {
  P.de.assign(P.ep.size(), LaDoppEpoch());
  if (!P.model.dopp_epoch_d) return;
  for (LaDoppEpoch& d : P.de) d.hbar = Eigen::VectorXd::Zero(P.p);
  for (const LaRow& r : P.rows) {
    if (!r.active || r.code) continue;
    LaDoppEpoch& d = P.de[static_cast<size_t>(r.ep)];
    const double w = 1.0 / r.var;
    d.W += w;
    d.vbar += w * r.v;
    d.hbar += w * r.h.transpose();
    d.hphibar += w * r.hphi;
    ++d.n;
  }
  for (LaDoppEpoch& d : P.de)
    if (d.W > 0.0) {
      d.vbar /= d.W;
      d.hbar /= d.W;
      d.hphibar /= d.W;
    }
}

// A row as it enters the normal equations: Doppler rows with the epoch drift eliminated are centred on their epoch.
struct LaReduced {
  Eigen::RowVectorXd h;
  double v = 0.0, q0 = 0.0;   // q0: variance removed by the elimination (1/W_j), for the w-test
  Eigen::RowVector2d hphi = Eigen::RowVector2d::Zero();
};
LaReduced laReduce(const LaProblem& P, const LaRow& r) {
  LaReduced x;
  x.h = r.h;
  x.v = r.v;
  x.hphi = r.hphi;
  if (!r.code && P.model.dopp_epoch_d) {
    const LaDoppEpoch& d = P.de[static_cast<size_t>(r.ep)];
    x.h -= d.hbar.transpose();
    x.v -= d.vbar;
    x.hphi -= d.hphibar;
    x.q0 = 1.0 / d.W;
  }
  return x;
}

bool laNormals(LaProblem& P, bool with_rhs) {
  laDoppMeans(P);
  P.N.setZero(P.p, P.p);
  P.rhs.setZero(P.p);
  for (const LaRow& row : P.rows) {
    if (!row.active) continue;
    const LaReduced x = laReduce(P, row);
    const double w = 1.0 / row.var;
    P.N.noalias() += w * x.h.transpose() * x.h;
    if (with_rhs) P.rhs.noalias() += w * x.h.transpose() * x.v;
  }
  return solveNormals(P.N, P.rhs, &P.Ninv, nullptr);
}

void laApply(LaProblem& P, const Eigen::VectorXd& dx) {
  P.psi += dx(0);
  P.t += dx.segment<3>(1);
  P.b += dx(4);
  P.d += dx(5);
  if (P.col_a >= 0) P.a += dx(P.col_a);
  for (size_t c = 0; c < P.col_k.size(); ++c) P.isb[P.col_k[c]] += dx(P.col_of[P.col_k[c]]);
}

// Weighted cost at the current linearisation (Doppler rows with the epoch drift eliminated: centred on their epoch).
double laCost(LaProblem& P) {
  laDoppMeans(P);
  double c = 0.0;
  for (const LaRow& r : P.rows)
    if (r.active) {
      const double v = (!r.code && P.model.dopp_epoch_d) ? r.v - P.de[static_cast<size_t>(r.ep)].vbar : r.v;
      c += v * v / r.var;
    }
  return c;
}

// Gauss–Newton at fixed models with a backtracking step (ψ enters the Doppler rows through cos ψ, sin ψ and the code rows
// through the lever arms: with σ_ψ of tens of degrees the undamped step can overshoot). Converged when every component
// of the step is below 1e-7 of its own formal σ (scale-free: an absolute limit such as 1e-12 rad is below the rounding
// of 2000-row normal equations when σ_ψ is 0.3 rad), or when no step along the Gauss–Newton direction lowers the cost
// any more and the step is already below 1e-3 σ (the rounding floor). The layout follows the active code rows (a system
// that lost every row loses its ι).
bool laGaussNewton(LaProblem& P, std::string* why) {
  laLayout(P);
  laLinearise(P);
  if (!laNormals(P, true)) {
    *why = "singular normal matrix (ψ, t, b, d or an ISB without information)";
    return false;
  }
  double cost = laCost(P);
  double last_rel = std::numeric_limits<double>::infinity();
  for (int it = 0; it < 60; ++it) {
    const Eigen::VectorXd dx = P.Ninv * P.rhs;
    double rel = 0.0;
    for (int i = 0; i < P.p; ++i) rel = std::max(rel, std::fabs(dx(i)) / std::sqrt(std::max(P.Ninv(i, i), 1e-300)));
    // θ before the step (only the parameters move; the rows are relinearised from them)
    const double psi0 = P.psi, b0 = P.b, d0 = P.d, a0 = P.a;
    const Eigen::Vector3d t0 = P.t;
    double isb0[4];
    std::copy(P.isb, P.isb + 4, isb0);
    auto restore = [&]() {
      P.psi = psi0;
      P.b = b0;
      P.d = d0;
      P.a = a0;
      P.t = t0;
      std::copy(isb0, isb0 + 4, P.isb);
    };
    double step = 1.0;
    bool better = false;
    for (int h = 0; h < 12; ++h, step *= 0.5) {
      laApply(P, step * dx);
      laLinearise(P);
      const double c = laCost(P);
      if (c <= cost * (1.0 + 1e-12) + 1e-12) {
        cost = c;
        better = true;
        break;
      }
      restore();
    }
    if (!better) {
      restore();
      laLinearise(P);
      if (rel < 1e-3 && laNormals(P, true)) return true;   // at the rounding floor of the cost
      *why = "joint LS did not converge (no descent; step " + fmt(rel, 4) + " σ)";
      return false;
    }
    if (!laNormals(P, true)) {
      *why = "singular normal matrix (ψ, t, b, d or an ISB without information)";
      return false;
    }
    if (rel * step < 1e-7) return true;   // residuals, Jacobians and N⁻¹ are at the solution (w-test, covariance)
    last_rel = rel * step;
  }
  // A flat cost (σ_ψ of tens of degrees) converges linearly: steps below 1e-3 σ are accepted (the rounding-floor rule).
  if (last_rel < 1e-3) return true;
  *why = "joint LS did not converge in 60 iterations (last step " + fmt(last_rel, 4) + " σ)";
  return false;
}

}  // namespace

// =============================================================================================
// state machine
// =============================================================================================
Initializer::Initializer(const Config& cfg, const Leveling& leveling) : cfg_(cfg), leveling_(leveling) {}

void Initializer::setPhase(Phase p) {
  phase_ = p;
  retry_wait_ = 0;
  la_attempted_ = false;   // A51: the next armed window is tried at once (REINIT, tests)
}

void Initializer::onLioFrame(bool imu_initialised, bool lidar_map_inited) {
  imu_ok_ = imu_initialised;
  map_ok_ = lidar_map_inited;
  // "≥50 LIO frames" counts frames with the host ready; a host re-initialisation starts the count again.
  if (imu_ok_ && map_ok_) ++lio_frames_; else lio_frames_ = 0;
  switch (phase_) {
    case Phase::OFF:
    case Phase::OBSERVE:
      phase_ = Phase::WAIT_CORE;
      if (coreReady()) phase_ = Phase::WAIT_NAV;
      break;
    case Phase::WAIT_CORE:
      if (coreReady()) phase_ = Phase::WAIT_NAV;
      break;
    case Phase::WAIT_NAV:
    case Phase::COLLECT:
    case Phase::ALIGN:
      if (!(imu_ok_ && map_ok_)) {   // host lost its initialisation: the windows' W frame is gone
        phase_ = Phase::WAIT_CORE;
        window_.clear();
        la_window_.clear();
      }
      break;
    default:   // TRACKING/OUTAGE/RECOVERY belong to the OutageManager
      break;
  }
}

bool Initializer::coreReady() const { return imu_ok_ && map_ok_ && lio_frames_ >= cfg_.init.min_livo_frames; }

// WAIT_NAV (§8 "≥5 颗星有健康星历", AMENDMENTS A51): one satellite with healthy ephemeris is enough to start collecting —
// the A51 path uses epochs of 1–3 satellites; the §8 path still takes only samples whose SPP used ≥ init.min_sats.
void Initializer::onNavCount(int n_healthy) {
  n_healthy_ = n_healthy;
  if (phase_ == Phase::WAIT_NAV && n_healthy_ >= 1) phase_ = Phase::COLLECT;
}

void Initializer::addSample(CollectSample&& s) {
  auto refuse = [&](const std::string& w) {
    ++n_refused_;
    last_refusal_ = w;
  };
  if (phase_ != Phase::COLLECT && phase_ != Phase::ALIGN) return refuse(std::string("not collecting (phase ") + phaseName(phase_) + ")");
  if (!s.t_rx.valid()) return refuse("invalid epoch time");
  if (has_consumed_ && !(t_consumed_ < s.t_rx)) return refuse("epoch at or before the last initialisation epoch k_a");
  // §8 window: a robust SPP with ≥ min_sats and GDOP < max_gdop.
  std::string why8;
  if (!window_.empty() && !(window_.back().t_rx < s.t_rx)) why8 = "epoch time not after the previous sample";
  else if (!s.spp.ok) why8 = "GNSS-only SPP failed";
  else if (s.spp.n_used < cfg_.init.min_sats)
    why8 = "SPP with " + std::to_string(s.spp.n_used) + " satellites < init.min_sats " + std::to_string(cfg_.init.min_sats);
  else if (!(s.spp.gdop < cfg_.init.max_gdop)) why8 = "GDOP " + fmt(s.spp.gdop, 2) + " ≥ init.max_gdop";
  else if (!s.bridge.ok) why8 = "no IMU bridge to the epoch";
  else if (!s.host_ant.y.allFinite() || !s.host_ant.z.allFinite()) why8 = "host antenna not finite";
  // A51 window: a bridge and at least one usable row, whatever the number of satellites.
  std::string why_la;
  if (!s.bridge.ok) why_la = "no IMU bridge to the epoch";
  else if (!s.host_ant.y.allFinite() || !s.host_ant.z.allFinite()) why_la = "host antenna not finite";
  else if (!la_window_.empty() && !(la_window_.back().t_rx < s.t_rx)) why_la = "epoch time not after the previous sample";
  else if (!hasLaRows(cfg_, s)) why_la = "no usable pseudorange or Doppler row";
  const bool to8 = why8.empty(), to_la = why_la.empty();
  if (!to8 && !to_la) return refuse(why8 == why_la ? why8 : why8 + "; lio-aided window: " + why_la);
  if (to_la) {
    // A receiver clock jump breaks the window's clock model b + d·τ: the window restarts at the jump.
    int compared = 0;
    if (!la_window_.empty() && std::fabs(clockDiscontinuity(la_window_, s, &compared)) > kLaClockJumpM) {
      la_window_.clear();
      ++la_restarts_;
    }
    if (to8) la_window_.push_back(s); else la_window_.push_back(std::move(s));
    while (la_window_.size() > 1 && la_window_.back().t_rx.diff(la_window_.front().t_rx) > kLaMaxSpanS + 1e-9) la_window_.pop_front();
  }
  if (to8) {
    window_.push_back(std::move(s));
    const double span = maxWindowSpan();
    while (window_.size() > 1 && window_.back().t_rx.diff(window_.front().t_rx) > span + 1e-9) window_.pop_front();
    if (retry_wait_ > 0) --retry_wait_;
  }
  if (phase_ == Phase::COLLECT && ((retry_wait_ == 0 && windowReady() && movingEnough()) || laArmed())) phase_ = Phase::ALIGN;
}

bool Initializer::windowReady() const {
  return static_cast<int>(window_.size()) >= cfg_.init.min_epochs && windowSpan() >= cfg_.init.min_window_s - 1e-9;
}

bool Initializer::movingEnough() const {
  std::vector<const CollectSample*> w;
  for (const CollectSample& s : window_) w.push_back(&s);
  const MotionStats m = motionOf(w.begin(), w.end(), cfg_.init.min_hspeed_mps);
  return m.moving >= cfg_.init.min_moving_epochs && m.path >= cfg_.init.min_path_m;
}

void Initializer::clearWindow() {
  window_.clear();
  la_window_.clear();
}

int Initializer::laWindowSize() const { return static_cast<int>(la_window_.size()); }
double Initializer::laWindowSpan() const {
  if (la_window_.size() < 2) return 0.0;
  return la_window_.back().t_rx.diff(la_window_.front().t_rx);
}
int Initializer::laRestarts() const { return la_restarts_; }

bool Initializer::laArmed() const {
  if (static_cast<int>(la_window_.size()) < cfg_.init.min_epochs) return false;
  if (laWindowSpan() < cfg_.init.min_window_s - 1e-9) return false;
  if (la_attempted_ && la_window_.back().t_rx.diff(la_last_attempt_) < kLaRetryS - 1e-9) return false;
  std::vector<const CollectSample*> w;
  for (const CollectSample& s : la_window_) w.push_back(&s);
  const MotionStats m = motionOf(w.begin(), w.end(), cfg_.init.min_hspeed_mps);
  return m.moving >= cfg_.init.min_moving_epochs && m.path >= cfg_.init.min_path_m;
}

double Initializer::windowSpan() const {
  if (window_.size() < 2) return 0.0;
  return window_.back().t_rx.diff(window_.front().t_rx);
}

double Initializer::maxWindowSpan() const { return kWindowSpanFactor * std::max(cfg_.init.min_window_s, 1.0); }
void Initializer::setGravityEstimated(bool en) { gravity_est_en_ = en; }
void Initializer::setAnchor(const AnchorFrame& anchor) {
  anchor_ = anchor;
  anchor_fixed_ = anchor.valid;
}
bool Initializer::anchorFixed() const { return anchor_fixed_; }
const AnchorFrame& Initializer::anchorFrame() const { return anchor_; }
bool Initializer::consumedByInit(const GTime& t_rx) const { return has_consumed_ && !(t_consumed_ < t_rx); }
int Initializer::samplesRefused() const { return n_refused_; }
const std::string& Initializer::lastRefusal() const { return last_refusal_; }

// =============================================================================================
// ALIGN pieces
// =============================================================================================
double Initializer::procrustesYaw(const std::vector<Eigen::Vector2d>& v_gnss, const std::vector<Eigen::Vector2d>& w_host) {
  // ψ₀ with R(ψ)·w ≈ v in the least-squares sense (2D orthogonal Procrustes): tan ψ = Σ(w × v)_z / Σ w·v.
  double s = 0.0, c = 0.0;
  const size_t n = std::min(v_gnss.size(), w_host.size());
  for (size_t i = 0; i < n; ++i) {
    const Eigen::Vector2d& a = w_host[i];
    const Eigen::Vector2d& b = v_gnss[i];
    s += a.x() * b.y() - a.y() * b.x();
    c += a.dot(b);
  }
  if (s == 0.0 && c == 0.0) return 0.0;
  return std::atan2(s, c);
}

bool Initializer::yawGaussNewton(const std::vector<CollectSample>& window, double yaw0, const Eigen::Matrix3d& C_level,
                                 const Eigen::Matrix3d& R_EN, YawFit* fit, std::string* why) {
  YawFit tmp;
  const bool ok = yawFitView(viewOf(window), yaw0, C_level, R_EN, &tmp, why);
  if (fit) *fit = tmp;
  return ok;
}

bool Initializer::yawGaussNewton(const std::vector<CollectSample>& window, double yaw0, double* yaw, double* sigma_yaw, std::string* why) {
  if (window.empty()) {
    if (why) *why = "yaw: empty window";
    return false;
  }
  const CollectSample& last = window.back();
  const Eigen::Vector3d gd = gravityDir(last.bridge);
  if (gd.squaredNorm() == 0.0) {
    if (why) *why = "yaw: no gravity snapshot in the last sample";
    return false;
  }
  const Eigen::Matrix3d C = Leveling::minimalRotation(gd, Eigen::Vector3d(0.0, 0.0, -1.0));
  const Eigen::Matrix3d R_EN = rtk::enuRotation(rtk::ecef2pos(last.spp.r_ecef));
  YawFit fit;
  if (!yawGaussNewton(window, yaw0, C, R_EN, &fit, why)) return false;
  if (yaw) *yaw = fit.yaw;
  if (sigma_yaw) *sigma_yaw = fit.sigma_yaw;
  return true;
}

// ALIGN: the §8 window first when it is armed (or when neither path is: its checks then say why), the A51 window when it
// is armed and §8 did not solve. A success fixes the N frame and consumes every collected epoch up to k_a; a failure
// returns to COLLECT (§8 retries after kRetrySamples more samples, A51 after kLaRetryS).
AlignResult Initializer::tryAlign(const CoreState& core, double offline_offset_s, double sigma_toff_s) {
  const bool la_armed = laArmed();
  const bool spp_armed = windowReady() && movingEnough();
  AlignResult a8, al;
  bool tried8 = false, tried_la = false;
  auto commit = [&](AlignResult& a) {
    anchor_ = a.anchor;
    anchor_fixed_ = true;
    GTime last = a.t_activate;
    if (!window_.empty() && last < window_.back().t_rx) last = window_.back().t_rx;
    if (!la_window_.empty() && last < la_window_.back().t_rx) last = la_window_.back().t_rx;
    t_consumed_ = last;
    has_consumed_ = true;
    window_.clear();
    la_window_.clear();
    phase_ = Phase::ALIGN;
    retry_wait_ = 0;
    la_attempted_ = false;
    return a;
  };
  if (spp_armed || !la_armed) {
    tried8 = true;
    a8 = alignSppWindow(core, offline_offset_s, sigma_toff_s);
    if (a8.ok) return commit(a8);
  }
  if (la_armed) {
    tried_la = true;
    la_last_attempt_ = la_window_.back().t_rx;
    la_attempted_ = true;
    al = alignLioAided(core, offline_offset_s, sigma_toff_s);
    if (al.ok) return commit(al);
  }
  AlignResult f = tried8 ? a8 : al;
  if (tried8 && tried_la) f.reason = a8.reason + "; " + al.reason;
  f.ok = false;
  if (phase_ == Phase::ALIGN) phase_ = Phase::COLLECT;
  if (tried8) retry_wait_ = kRetrySamples;
  return f;
}

AlignResult Initializer::alignSppWindow(const CoreState& core, double offline_offset_s, double sigma_toff_s) const {
  AlignResult a;
  a.path = "spp_window";
  a.sigma_toff_s = sigma_toff_s;
  a.offset_check_s = offline_offset_s;
  auto fail = [&](const std::string& why) {
    a.ok = false;
    a.reason = why;
    return a;
  };
  if (window_.empty()) return fail("empty window");
  View w;
  for (const CollectSample& s : window_) w.push_back(&s);
  const MotionStats ms = motionOf(w.begin(), w.end(), cfg_.init.min_hspeed_mps);
  a.moving_epochs = ms.moving;
  a.path_m = ms.path;
  a.epochs_used = static_cast<int>(w.size());
  if (!windowReady())
    return fail("window not ready: " + std::to_string(window_.size()) + " epochs over " + fmt(windowSpan(), 1) + " s (need " +
                std::to_string(cfg_.init.min_epochs) + " and " + fmt(cfg_.init.min_window_s, 1) + " s)");
  if (!(ms.moving >= cfg_.init.min_moving_epochs && ms.path >= cfg_.init.min_path_m))
    return fail("insufficient motion: " + std::to_string(ms.moving) + " epochs with v_h ≥ " + fmt(cfg_.init.min_hspeed_mps, 2) +
                " m/s, path " + fmt(ms.path, 2) + " m (need " + std::to_string(cfg_.init.min_moving_epochs) + " and " +
                fmt(cfg_.init.min_path_m, 1) + " m); no alignment at standstill");
  if (!leveling_.hasRef()) return fail("leveling reference not set: call Leveling::setRef(core.g) before tryAlign (DESIGN §1)");
  const CollectSample& last = *w.back();
  const Eigen::Matrix3d C = leveling_.C(core.g);

  // ---- 1. N frame ----
  if (anchor_fixed_) {
    a.anchor = anchor_;
  } else {
    std::vector<double> xs, ys, zs;
    for (const CollectSample* s : w) {
      xs.push_back(s->spp.r_ecef.x());
      ys.push_back(s->spp.r_ecef.y());
      zs.push_back(s->spp.r_ecef.z());
    }
    a.anchor.r0_ecef = Eigen::Vector3d(median(xs), median(ys), median(zs));
    a.anchor.geo = rtk::ecef2pos(a.anchor.r0_ecef);
    a.anchor.R_EN = rtk::enuRotation(a.anchor.geo);
    a.anchor.t_fixed = last.t_rx;
    a.anchor.valid = true;
  }
  const AnchorFrame& anc = a.anchor;

  // ---- 2. time check (§7, diagnostic) ----
  {
    const TimeCheck tc = timeCheck(w);
    a.offset_check_valid = tc.valid;
    if (tc.valid) {
      a.offset_check_s = offline_offset_s + tc.tau;
      a.offset_check_sigma_s = tc.sigma;
      const double s = std::sqrt(tc.sigma * tc.sigma + sigma_toff_s * sigma_toff_s);
      a.offset_warn = std::fabs(tc.tau) > 3.0 * s;
    }
  }

  // ---- 3. yaw ----
  {
    std::vector<Eigen::Vector2d> vg, wh;
    for (const CollectSample* s : w) {
      if (!s->dop.ok) continue;
      const Eigen::Vector3d vN = anc.R_EN.transpose() * s->dop.v_ecef;
      const Eigen::Vector3d wl = C * s->host_ant.z;
      vg.emplace_back(vN.x(), vN.y());
      wh.emplace_back(wl.x(), wl.y());
    }
    if (vg.size() < 3) return fail("yaw: fewer than 3 Doppler LS velocities in the window");
    const double yaw0 = procrustesYaw(vg, wh);
    YawFit fit;
    std::string why;
    if (!yawFitView(w, yaw0, C, anc.R_EN, &fit, &why)) return fail(why);
    a.yaw = fit.yaw;
    a.sigma_yaw = fit.sigma_yaw;
    a.sigma0_dopp = fit.sigma0;
    a.rows_dopp = fit.rows;
    a.rejected_dopp = fit.rejected;
    if (a.sigma_yaw > cfg_.init.yaw_sigma_max_deg * kDeg)
      return fail("yaw: σ_ψ " + fmt(a.sigma_yaw / kDeg, 2) + "° > init.yaw_sigma_max_deg " + fmt(cfg_.init.yaw_sigma_max_deg, 2) + "°");
  }
  const Eigen::Matrix3d Rz = Leveling::Rz(a.yaw);
  const Eigen::Matrix3d M = Rz * C;

  // ---- pivot, lever arm, host Jacobian at k_a (current host state) ----
  // l^I from the facade's own evaluation: y = p_g + R_g l (ImuBridge::evaluate).
  const Eigen::Vector3d lever = last.host_ant.R_g.transpose() * (last.host_ant.y - last.host_ant.p_g);
  const bool grav = gravity_est_en_ && cfg_.frames.gnss_updates_gravity;
  const AntennaAtEpoch ant = ImuBridge::evaluate(core, last.bridge, lever, grav);
  a.pivot_W = ant.y;   // o^W = current a^W (§8)
  {
    NoiseModel noise(cfg_);
    RowBuilder rb(cfg_, noise);
    LinPoint lp;
    lp.x = &core;
    lp.bridge = &last.bridge;
    lp.aug = nullptr;             // ψ = φ = 0 there: M = C(g); R_z(ψ) is applied below (every host column is M·(…))
    lp.leveling = &leveling_;
    lp.anchor = &anc;
    lp.lever_I = lever;
    lp.pivot_W = a.pivot_W;
    lp.gravity_est_en = gravity_est_en_;
    lp.delta = last.bridge.delta;
    const AntennaJacobians J = rb.antennaJacobians(lp);
    a.J_trans_host = -(Rz * J.J_a.leftCols<HostIdx::dim>());
  }

  // ---- 4. translation / clock / ISB (1 Hz epochs, A43) ----
  View ep;
  {
    int64_t last_sec = std::numeric_limits<int64_t>::min();
    for (const CollectSample* s : w) {
      if (s->t_rx.sec == last_sec) continue;   // the first epoch of each GPST second
      last_sec = s->t_rx.sec;
      ep.push_back(s);
    }
  }
  std::vector<Eigen::Vector3d> y_rel;
  for (const CollectSample* s : ep) y_rel.push_back(s->host_ant.y - a.pivot_W);
  Eigen::Vector3d t0;
  double isb0[4] = {0, 0, 0, 0};
  {
    std::vector<double> tx, ty, tz;
    std::vector<double> ib[4];
    for (size_t j = 0; j < ep.size(); ++j) {
      const Eigen::Vector3d tj = anc.R_EN.transpose() * (ep[j]->spp.r_ecef - anc.r0_ecef) - M * y_rel[j];
      tx.push_back(tj.x());
      ty.push_back(tj.y());
      tz.push_back(tj.z());
      for (int k = 0; k < 4; ++k)
        if (ep[j]->spp.isb_est[sppIsbIndex(k)]) ib[k].push_back(ep[j]->spp.isb_m[sppIsbIndex(k)]);
    }
    t0 = Eigen::Vector3d(median(tx), median(ty), median(tz));
    for (int k = 0; k < 4; ++k) isb0[k] = median(ib[k]);
  }
  const BatchResult br = batchLs(cfg_, ep, y_rel, M, Rz, C, anc, t0, isb0);
  if (!br.ok) return fail(br.why);
  a.rows_psr = br.rows;
  a.rejected_psr = br.rejected;
  a.epochs_psr = br.epochs;
  a.sigma0_psr = std::sqrt(br.s0sq);
  a.rms_psr_m = br.rms_m;
  if (br.rms_m > kMaxPsrSigmaM) return fail("trans: post-fit pseudorange σ̂ " + fmt(br.rms_m, 2) + " m > 5 m");

  // Drift from the newest epoch with a Doppler LS solution (§8 "d 取 k_a 多普勒 LS").
  const CollectSample* sd = nullptr;
  for (auto it = w.rbegin(); it != w.rend(); ++it)
    if ((*it)->dop.ok) {
      sd = *it;
      break;
    }
  if (!sd) return fail("clock: no Doppler LS drift in the window");
  const double var_dD = dopplerDriftVariance(*sd);
  if (!(var_dD >= 0.0) || !std::isfinite(var_dD)) return fail("clock: Doppler LS drift covariance not available");

  // ---- assemble the noise model over n = [t(3), b(t_k), d(t_k), ι_observed…] ----
  const int p = 3 + static_cast<int>(br.col_k.size());
  const int k_obs = static_cast<int>(br.col_k.size());
  const int dimn = 5 + k_obs;
  a.t_k = last.bridge.t_k;
  const double delta_b = std::max(0.0, a.t_k - ep[br.jb]->bridge.t_g);
  const double delta_d = std::max(0.0, a.t_k - sd->bridge.t_g);
  const double d_D = sd->dop.d_mps;
  const Eigen::MatrixXd Sth = br.Ninv * br.scale;                            // Σ_θ
  const Eigen::VectorXd Sthb = -(br.Ninv * br.gbar_jb) * br.scale;           // Cov(θ, b_jb)
  const double var_bjb = (br.invW_jb + br.gbar_jb.dot(br.Ninv * br.gbar_jb)) * br.scale;
  ClockModel cm(cfg_.process.clock);
  const Eigen::Matrix2d Qc = cm.Q(std::max(delta_b, delta_d), 0.0, 0.0);
  // index map of θ into n: t → 0..2, ι_c → 5 + c
  auto nIndex = [&](int th) { return th < 3 ? th : 5 + (th - 3); };
  Eigen::MatrixXd Sn = Eigen::MatrixXd::Zero(dimn, dimn);
  for (int i = 0; i < p; ++i) {
    for (int j = 0; j < p; ++j) Sn(nIndex(i), nIndex(j)) = Sth(i, j);
    Sn(nIndex(i), 3) = Sn(3, nIndex(i)) = Sthb(i);
  }
  Sn(3, 3) = var_bjb + delta_b * delta_b * var_dD + Qc(0, 0);
  Sn(3, 4) = Sn(4, 3) = delta_b * var_dD + Qc(0, 1);
  Sn(4, 4) = var_dD + Qc(1, 1);
  a.cov_noise_formal = Sn;
  const double floor2 = cfg_.init.spp_sigma_floor_m * cfg_.init.spp_sigma_floor_m;   // §8 max(formal·σ̂0², 3² m²)
  for (int i = 0; i < 3; ++i) Sn(i, i) = std::max(Sn(i, i), floor2);
  Eigen::VectorXd Jy = Eigen::VectorXd::Zero(dimn);
  Eigen::MatrixXd Jf = Eigen::MatrixXd::Zero(dimn, 2);
  for (int i = 0; i < p; ++i) {
    Jy(nIndex(i)) = br.dtheta_dpsi(i);
    Jf.row(nIndex(i)) = br.dtheta_dphi.row(i);
  }
  // b̂_jb = mean_jb(P − h0) ⇒ ∂b̂/∂ψ = −h̄_ψ,jb − ḡ_jbᵀ ∂θ̂/∂ψ (same for φ); b(t_k) = b_jb + δ_b d does not add any.
  Jy(3) = -br.hpsi_bar_jb - br.gbar_jb.dot(br.dtheta_dpsi);
  Jf.row(3) = -br.hphi_bar_jb - br.gbar_jb.transpose() * br.dtheta_dphi;

  a.cov_noise = Sn;
  a.J_noise_yaw = Jy;
  a.J_noise_tilt = Jf;
  a.trans = br.t;
  a.cov_trans = Sn.topLeftCorner<3, 3>();
  a.J_trans_yaw = Jy.head<3>();
  a.J_trans_tilt = Jf.topRows<3>();
  a.b = br.b_jb + delta_b * d_D;
  a.d = d_D;
  a.var_d = Sn(4, 4);
  a.cov_clock_isb = Eigen::MatrixXd::Zero(1 + k_obs, 1 + k_obs);
  a.cov_clock_isb(0, 0) = Sn(3, 3);
  for (int c = 0; c < k_obs; ++c) {
    a.cov_clock_isb(0, 1 + c) = a.cov_clock_isb(1 + c, 0) = Sn(3, 5 + c);
    for (int e = 0; e < k_obs; ++e) a.cov_clock_isb(1 + c, 1 + e) = Sn(5 + c, 5 + e);
  }
  for (int k = 0; k < 4; ++k) {
    a.isb_observed[k] = br.observed[k];
    a.isb_present[k] = isbSlotEnabled(cfg_, k);
    a.isb[k] = br.observed[k] ? br.isb[k] : 0.0;
  }
  a.t_activate = last.t_rx;
  // diagnostics shared with A51: the largest formal σ of t (post-fit scaled, with its ψ part, before the floor)
  a.sigma_trans_max_m =
      std::sqrt((a.cov_noise_formal.topLeftCorner<3, 3>() + a.J_trans_yaw * a.sigma_yaw * a.sigma_yaw * a.J_trans_yaw.transpose())
                    .diagonal()
                    .maxCoeff());
  a.redundancy = br.rows - br.epochs - p;
  {
    std::vector<SatId> sats;
    for (const CollectSample* s : w)
      for (const SatId& id : s->spp.used)
        if (!contains(sats, id)) sats.push_back(id);
    a.n_sats = static_cast<int>(sats.size());
    a.window_s = windowSpan();
  }
  a.ok = true;
  return a;   // tryAlign() fixes the N frame and consumes the window's epochs
}

// AMENDMENTS A51 (see the block comment above LaEpoch for the model and the stages).
AlignResult Initializer::alignLioAided(const CoreState& core, double offline_offset_s, double sigma_toff_s) const {
  AlignResult a;
  a.path = "lio_aided";
  a.sigma_toff_s = sigma_toff_s;
  a.offset_check_s = offline_offset_s;
  auto fail = [&](const std::string& why) {
    a.ok = false;
    a.reason = "lio-aided: " + why;
    return a;
  };
  if (la_window_.empty()) return fail("empty window");
  View w;
  for (const CollectSample& s : la_window_) w.push_back(&s);
  const MotionStats ms = motionOf(w.begin(), w.end(), cfg_.init.min_hspeed_mps);
  a.moving_epochs = ms.moving;
  a.path_m = ms.path;
  a.epochs_used = static_cast<int>(w.size());
  a.window_s = laWindowSpan();
  if (!leveling_.hasRef()) return fail("leveling reference not set: call Leveling::setRef(core.g) before tryAlign (DESIGN §1)");
  const CollectSample& last = *w.back();
  const Eigen::Matrix3d C = leveling_.C(core.g);
  const double t_k = last.bridge.t_k;

  // ---- rows of the window; o^W = the host antenna at k_a ----
  NoiseModel noise(cfg_);
  LaProblem P;
  P.cfg = &cfg_;
  P.noise = &noise;
  P.C = C;
  P.sigma_toff = sigma_toff_s;
  const Eigen::Vector3d pivot0 = pivotOf(cfg_, gravity_est_en_, core, last);
  {
    // Code rows at 1 Hz per satellite (A51 "全部通过 L0 的伪距" with the κ_AR(τ_ρ, 1 s) of §4.7/A43): every satellite
    // contributes its first L0-passed pseudorange of each GPST second, whichever epoch of that second carries it — the
    // first epoch of a second alone (A43's epoch rule, A1-2) misses the satellites a 1–3-satellite sky shows only in the
    // later epochs of the second.
    std::vector<std::pair<int, int64_t>> code_taken;   // (satellite, GPST second)
    for (const CollectSample* s : w) {
      LaEpoch E;
      E.s = s;
      E.y = s->host_ant.y - pivot0;
      E.w = s->host_ant.z;
      E.tau = s->bridge.t_g - t_k;
      E.models_given = s->psr_std_m.size() != s->raw.size() || s->dopp_std_hz.size() != s->raw.size();
      const int j = static_cast<int>(P.ep.size());
      P.ep.push_back(E);
      for (size_t i = 0; i < s->raw.size(); ++i) {
        const SppInput& in = s->raw[i];
        int k = -1;
        const std::pair<int, int64_t> key(in.sat.rtk_sat, s->t_rx.sec);
        if (rowUsable(in, true) && batchCodeSystem(cfg_, in.sat.sys, &k) && !(s->spp.ok && contains(s->spp.rejected, in.sat)) &&
            std::find(code_taken.begin(), code_taken.end(), key) == code_taken.end()) {
          code_taken.push_back(key);
          P.ep[static_cast<size_t>(j)].code = true;
          LaRow r;
          r.ep = j;
          r.idx = static_cast<int>(i);
          r.s = s;
          r.code = true;
          r.k = k;
          P.rows.push_back(r);
        }
        if (rowUsable(in, false) && !(s->dop.ok && contains(s->dop.rejected, in.sat))) {
          LaRow r;
          r.ep = j;
          r.idx = static_cast<int>(i);
          r.s = s;
          r.code = false;
          P.rows.push_back(r);
        }
      }
    }
  }
  // Necessary conditions before any solve (the "星数×运动几何" of A51 in words): the GPS clock b needs GPS/QZSS code;
  // t(3) and b need ≥ 4 distinct lines of sight, one more per ISB (the lines of sight hardly turn in 60 s); ψ and d need
  // Doppler rows; the redundancy rows − unknowns ≥ kLaMinRedundancy.
  auto census = [&](bool code, std::vector<SatId>* sats, bool sys_present[4], bool* base) {
    for (int k = 0; k < 4; ++k) sys_present[k] = false;
    *base = false;
    int m = 0;
    for (const LaRow& r : P.rows) {
      if (!r.active || r.code != code) continue;
      ++m;
      if (!contains(*sats, r.in().sat)) sats->push_back(r.in().sat);
      if (r.k >= 0) sys_present[r.k] = true; else *base = true;
    }
    return m;
  };
  {
    std::vector<SatId> code_sats, dopp_sats;
    bool sysp[4], base = false, sysd[4], based = false;
    const int mc = census(true, &code_sats, sysp, &base);
    const int md = census(false, &dopp_sats, sysd, &based);
    int kc = 0;
    for (int k = 0; k < 4; ++k) kc += sysp[k] ? 1 : 0;
    std::string list;
    for (const SatId& s : code_sats) list += " " + rtk::satIdString(s.rtk_sat);
    a.rows_psr = mc;
    a.rows_dopp = md;
    a.n_sats = static_cast<int>(code_sats.size());
    const std::string census_txt = std::to_string(mc) + " code rows from " + std::to_string(code_sats.size()) + " satellites (" +
                                   (list.empty() ? std::string(" none") : list) + " ) and " + std::to_string(md) + " Doppler rows from " +
                                   std::to_string(dopp_sats.size()) + " satellites over " + fmt(a.window_s, 1) + " s";
    if (!base) return fail("no GPS/QZSS pseudorange in the window (b is the GPS clock): " + census_txt);
    if (static_cast<int>(code_sats.size()) < 4 + kc)
      return fail("too few distinct code satellites for t, b" + std::string(kc ? " and the ISBs" : "") + " (need " + std::to_string(4 + kc) +
                  "): " + census_txt);
    // Doppler rows that still carry ψ once each epoch's drift is removed: n_j − 1 per epoch with n_j rows
    std::vector<int> per_ep(P.ep.size(), 0);
    for (const LaRow& r : P.rows)
      if (!r.code) ++per_ep[static_cast<size_t>(r.ep)];
    int md_eff = 0, n_de = 0;
    for (int n : per_ep) {
      md_eff += std::max(0, n - 1);
      n_de += n > 0 ? 1 : 0;
    }
    if (md_eff < 3)
      return fail("fewer than 3 Doppler differences within epochs for ψ (" + std::to_string(md_eff) +
                  "; every epoch's clock drift is its own unknown): " + census_txt);
    const int p = 7 + kc;
    const int red = mc + md - n_de - p;
    if (red < kLaMinRedundancy)
      return fail("redundancy " + std::to_string(red) + " < " + std::to_string(kLaMinRedundancy) + ": " + census_txt);
  }

  // ---- stage A: coarse position/clock; the working N frame ----
  std::string why;
  bool have_start = false;
  Eigen::Vector3d r_start = Eigen::Vector3d::Zero();
  for (auto it = w.rbegin(); it != w.rend(); ++it)
    if ((*it)->spp.ok) {
      r_start = (*it)->spp.r_ecef;
      have_start = true;
      break;
    }
  if (!have_start && anchor_fixed_) {
    r_start = anchor_.r0_ecef;
    have_start = true;
  }
  const CoarseFix cf = laCoarse(P, have_start, r_start, &why);
  if (!cf.ok) return fail(why);
  if (anchor_fixed_) {
    a.anchor = anchor_;
  } else {
    a.anchor.r0_ecef = cf.r;
    a.anchor.geo = rtk::ecef2pos(cf.r);
    a.anchor.R_EN = rtk::enuRotation(a.anchor.geo);
    a.anchor.t_fixed = last.t_rx;
    a.anchor.valid = true;
  }
  P.anc = a.anchor;
  Eigen::Vector3d ybar = Eigen::Vector3d::Zero();
  {
    int n = 0;
    for (const LaEpoch& E : P.ep)
      if (E.code) {
        ybar += E.y;
        ++n;
      }
    if (n > 0) ybar /= n;
  }
  P.b = cf.b;
  P.d = cf.d;
  for (int k = 0; k < 4; ++k) P.isb[k] = cf.isb[k];
  P.psi = 0.0;
  P.t = P.anc.R_EN.transpose() * (cf.r - P.anc.r0_ecef) - C * ybar;
  laModels(P, true);   // elevation mask and variances at the coarse position (the unknown ψ moves the epochs by < 2|y|)

  // ---- stage B: ψ₀, d₀ ----
  double psi0 = 0.0, d0 = cf.d;
  if (!laYawStart(P, cf.r, &psi0, &d0, &why)) return fail(why);
  P.psi = psi0;
  P.d = d0;
  P.a = 0.0;
  P.t = P.anc.R_EN.transpose() * (cf.r - P.anc.r0_ecef) - Leveling::Rz(psi0) * C * ybar;

  // ---- stage C: joint Gauss–Newton with the models re-evaluated, then the iterated w-test ----
  int m_all = 0;
  for (const LaRow& r : P.rows) m_all += r.active ? 1 : 0;
  int rejected = 0, rejected_psr = 0, rejected_dopp = 0;
  bool clean = false;
  for (int pass = 0; pass < 4 && !clean; ++pass) {
    for (int outer = 0; outer < 5; ++outer) {
      const Eigen::Vector3d t_before = P.t;
      const double psi_before = P.psi;
      laModels(P, false);
      if (!laGaussNewton(P, &why)) return fail(why);
      if ((P.t - t_before).norm() < 0.01 && std::fabs(wrapPi(P.psi - psi_before)) < 1e-6) break;
    }
    clean = true;
    for (int round = 0; round <= m_all; ++round) {
      // Baarda w on every active row: Q_v = var − h N⁻¹ hᵀ
      double w_max = 0.0;
      int worst = -1;
      for (size_t i = 0; i < P.rows.size(); ++i) {
        const LaRow& r = P.rows[i];
        if (!r.active) continue;
        const LaReduced x = laReduce(P, r);   // Doppler: centred on its epoch, Q_v loses 1/W_j
        const double qv = r.var - x.q0 - (x.h * P.Ninv * x.h.transpose()).value();
        if (!(qv > 1e-9 * r.var)) continue;   // no redundancy for this row (e.g. the only Doppler row of its epoch)
        const double wv = std::fabs(x.v) / std::sqrt(qv);
        if (wv > w_max) {
          w_max = wv;
          worst = static_cast<int>(i);
        }
      }
      if (worst < 0 || w_max <= kWThreshold) break;
      clean = false;
      P.rows[worst].active = false;
      P.rows[worst].rejected = true;
      ++rejected;
      (P.rows[worst].code ? rejected_psr : rejected_dopp)++;
      if (rejected > kMaxRejectFrac * m_all) return fail("more than 30 % of the rows rejected by the w-test");
      if (!laGaussNewton(P, &why)) return fail(why + " after the w-test");
    }
  }
  if (!clean) return fail("the w-test did not settle");

  // ---- solvability (0/1) and the result ----
  int m = 0, mc = 0, md = 0;
  double ssr = 0.0, ssr_code = 0.0, ssr_w_code = 0.0;
  std::vector<SatId> used_sats;
  std::vector<int64_t> code_secs;
  int n_de = 0;
  for (const LaDoppEpoch& d : P.de) n_de += d.n > 0 ? 1 : 0;
  if (!P.model.dopp_epoch_d) n_de = 0;
  for (const LaRow& r : P.rows) {
    if (!r.active) continue;
    ++m;
    const double v = laReduce(P, r).v;
    ssr += v * v / r.var;
    if (r.code) {
      ++mc;
      ssr_code += r.v * r.v;
      ssr_w_code += r.v * r.v / r.var;
      if (code_secs.empty() || code_secs.back() != P.ep[r.ep].s->t_rx.sec) code_secs.push_back(P.ep[r.ep].s->t_rx.sec);
    } else {
      ++md;
    }
    if (!contains(used_sats, r.in().sat)) used_sats.push_back(r.in().sat);
  }
  const int p = P.p;
  const int dof = m - p - n_de;   // one eliminated drift per epoch with Doppler rows
  a.rows_psr = mc;
  a.rows_dopp = md;
  a.rejected_psr = rejected_psr;
  a.rejected_dopp = rejected_dopp;
  a.epochs_psr = static_cast<int>(code_secs.size());
  a.redundancy = dof;
  a.n_sats = static_cast<int>(used_sats.size());
  if (dof < kLaMinRedundancy) return fail("redundancy " + std::to_string(dof) + " < " + std::to_string(kLaMinRedundancy) + " after the w-test");
  const double s0sq = ssr / dof;
  const double scale = std::max(1.0, s0sq);   // A1-1
  // Per-class unit weights (diagnostics: how conservative each a-priori variance model is on this data; the class
  // shares of the redundancy are taken proportional to their row counts)
  const double dof_frac = static_cast<double>(dof) / (m - n_de);
  a.sigma0_psr = mc > 0 ? std::sqrt(ssr_w_code / (mc * dof_frac)) : 0.0;
  a.sigma0_dopp = md > n_de ? std::sqrt((ssr - ssr_w_code) / ((md - n_de) * dof_frac)) : 0.0;
  a.rms_psr_m = mc > 0 ? std::sqrt(ssr_code / mc) : 0.0;
  const Eigen::MatrixXd S = P.Ninv * scale;   // Σ_θ, θ = [ψ, t, b, d, (a), ι…]
  const double var_psi = S(0, 0);
  const double sigma_psi = std::sqrt(var_psi);
  const double sigma_t = std::sqrt(S.block<3, 3>(1, 1).diagonal().maxCoeff());
  a.sigma_trans_max_m = sigma_t;
  std::string code_list;
  {
    std::vector<SatId> cs;
    for (const LaRow& r : P.rows)
      if (r.active && r.code && !contains(cs, r.in().sat)) cs.push_back(r.in().sat);
    for (const SatId& x : cs) code_list += " " + rtk::satIdString(x.rtk_sat);
  }
  // Diagnostics (events.log): ψ of the same rows under A51 as written (one drift shared by all Doppler rows, linear code
  // clock) and from the Doppler rows alone (code rows without their lever-arm ψ sensitivity), next to this model's.
  auto variant = [&](const LaModel& mdl) -> std::string {
    LaProblem Q = P;   // same rows and memberships, started from this solution
    Q.model = mdl;
    Q.psi_code = P.psi;
    if (!mdl.code_quad) Q.a = 0.0;
    std::string w;
    if (!laGaussNewton(Q, &w)) return "(" + w + ")";
    return fmt(wrapPi(Q.psi) / kDeg, 2) + "±" + fmt(std::sqrt(Q.Ninv(0, 0) * scale) / kDeg, 2) + "°";
  };
  LaModel ruled;
  ruled.dopp_epoch_d = false;
  ruled.code_quad = false;
  LaModel dopp_only = P.model;
  dopp_only.code_psi = false;
  const std::string psi_txt = "ψ " + fmt(wrapPi(P.psi) / kDeg, 2) + "° [A51 as written " + variant(ruled) + ", Doppler only " +
                              variant(dopp_only) + "]";
  const std::string stats = psi_txt + "; " + std::to_string(md) + " Doppler / " + std::to_string(mc) + " code rows (" + std::to_string(a.epochs_psr) +
                            " s; code sats" + code_list + ") from " + std::to_string(a.n_sats) + " satellites, redundancy " +
                            std::to_string(dof) + ", σ̂0 " + fmt(std::sqrt(s0sq), 3) + " (code " + fmt(a.sigma0_psr, 3) + ", Doppler " +
                            fmt(a.sigma0_dopp, 3) + ", code RMS " + fmt(a.rms_psr_m, 1) + " m), " + std::to_string(rejected) +
                            " rejected, path " + fmt(ms.path, 1) + " m, " + std::to_string(ms.moving) + " moving epochs";
  if (sigma_psi > cfg_.init.yaw_sigma_max_deg * kDeg)
    return fail("σ_ψ " + fmt(sigma_psi / kDeg, 2) + "° > init.yaw_sigma_max_deg " + fmt(cfg_.init.yaw_sigma_max_deg, 2) + "° (σ_t " +
                fmt(sigma_t, 2) + " m; " + stats + ")");
  if (sigma_t > kLaMaxTransSigmaM)
    return fail("σ_t " + fmt(sigma_t, 2) + " m > " + fmt(kLaMaxTransSigmaM, 1) + " m (σ_ψ " + fmt(sigma_psi / kDeg, 2) + "°; " + stats + ")");

  // ∂θ̂/∂φ: the LS was solved at φ = 0 while the true φ has the prior σ_DOV ⇒ e_θ = n_θ + J_φ e_φ, J_φ = −N⁻¹ Σ w hᵀ h_φ
  // (reduced rows: the eliminated drifts take their share of h_φ too).
  Eigen::MatrixXd Sphi = Eigen::MatrixXd::Zero(p, 2);
  for (const LaRow& r : P.rows)
    if (r.active) {
      const LaReduced x = laReduce(P, r);
      Sphi.noalias() += (1.0 / r.var) * x.h.transpose() * x.hphi;
    }
  const Eigen::MatrixXd Jphi = -P.Ninv * Sphi;

  // The new state y = [ψ, t(3), b(t_k), d(t_k), ι_observed…] as a linear map of the LS errors: e_y = A e_θ (+ ε̄_J in d).
  // b(t_k) = b of the code clock (τ = 0 at t_k). d(t_k): with per-epoch drifts the drift of the newest epoch J with
  // Doppler rows, d̂_J = v̄_J(θ̂) ⇒ e_d = −h̄_J e_θ + ε̄_J, Var ε̄_J = 1/W_J, ε̄_J ⟂ e_θ (the reduction removed epoch means),
  // ∂/∂φ: −(h̄φ_J + h̄_J J_φ) — the §8 path's "d 取 k_a 多普勒 LS"; with the shared drift, d itself.
  const int k_obs = static_cast<int>(P.col_k.size());
  const int ny = 6 + k_obs;
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(ny, p);
  A(0, 0) = 1.0;
  A.block<3, 3>(1, 1).setIdentity();
  A(4, 4) = 1.0;
  int j_d = -1;
  double t_d = t_k;
  if (P.model.dopp_epoch_d) {
    for (int j = static_cast<int>(P.de.size()) - 1; j >= 0; --j)
      if (P.de[static_cast<size_t>(j)].n > 0) {
        j_d = j;
        break;
      }
    if (j_d < 0) return fail("no Doppler epoch left for the clock drift");
    A.row(5) = -P.de[static_cast<size_t>(j_d)].hbar.transpose();
    t_d = P.ep[static_cast<size_t>(j_d)].s->bridge.t_g;
  } else {
    A(5, 5) = 1.0;
  }
  for (int c = 0; c < k_obs; ++c) A(6 + c, P.col_of[P.col_k[static_cast<size_t>(c)]]) = 1.0;
  Eigen::MatrixXd Sy = A * S * A.transpose();
  Eigen::MatrixXd Jy = A * Jphi;
  double d_hat = P.d;
  if (j_d >= 0) {
    const LaDoppEpoch& dj = P.de[static_cast<size_t>(j_d)];
    Sy(5, 5) += scale / dj.W;
    Jy.row(5) -= dj.hphibar;
    d_hat = dj.vbar;
  }
  // Map into activate()'s noise model over n = [t(3), b(t_k), d(t_k), ι_observed…]: n = K·n_ψ + n_⊥ with
  // K = Cov(n, ψ)/σ_ψ² (the regression on ψ's own noise) and cov_noise = Cov(n_⊥) = the Schur complement.
  const int dimn = 5 + k_obs;
  const Eigen::VectorXd K = Sy.block(1, 0, dimn, 1) / var_psi;
  Eigen::MatrixXd Sn = Sy.block(1, 1, dimn, dimn) - K * var_psi * K.transpose();
  const Eigen::MatrixXd Jf = Jy.bottomRows(dimn);
  // b refers to t_k; the newest code/Doppler epochs are δ before it: the clock process noise of that bridge (as the §8
  // path adds), for d from the Doppler epoch J it was taken at.
  const double delta = std::max(0.0, t_k - std::min(t_d, last.bridge.t_g));
  ClockModel cm(cfg_.process.clock);
  const Eigen::Matrix2d Qc = cm.Q(delta, 0.0, 0.0);
  Sn(3, 3) += Qc(0, 0);
  Sn(3, 4) += Qc(0, 1);
  Sn(4, 3) += Qc(1, 0);
  Sn(4, 4) += Qc(1, 1);
  Sn = 0.5 * (Sn + Sn.transpose());
  a.cov_noise_formal = Sn;
  const double floor2 = cfg_.init.spp_sigma_floor_m * cfg_.init.spp_sigma_floor_m;   // §8 max(formal·σ̂0², 3² m²)
  for (int i = 0; i < 3; ++i) Sn(i, i) = std::max(Sn(i, i), floor2);

  // pivot and ∂t/∂δx at k_a with the solved yaw (identical pivot to the one the rows were built on)
  const Eigen::Matrix3d Rz = Leveling::Rz(P.psi);
  const PivotFrame pf = pivotAt(cfg_, leveling_, gravity_est_en_, core, last, P.anc, Rz);
  a.pivot_W = pf.pivot_W;
  a.J_trans_host = pf.J_trans_host;

  // §7 online time check (diagnostic; only epochs that had a Doppler LS velocity can take part)
  {
    const TimeCheck tc = timeCheck(w);
    a.offset_check_valid = tc.valid;
    if (tc.valid) {
      a.offset_check_s = offline_offset_s + tc.tau;
      a.offset_check_sigma_s = tc.sigma;
      const double s = std::sqrt(tc.sigma * tc.sigma + sigma_toff_s * sigma_toff_s);
      a.offset_warn = std::fabs(tc.tau) > 3.0 * s;
    }
  }

  a.yaw = wrapPi(P.psi);
  a.sigma_yaw = sigma_psi;
  a.J_yaw_tilt = Jy.row(0);
  a.cov_noise = Sn;
  a.J_noise_yaw = K;
  a.J_noise_tilt = Jf;
  a.trans = P.t;
  a.cov_trans = Sn.topLeftCorner<3, 3>();
  a.J_trans_yaw = K.head<3>();
  a.J_trans_tilt = Jf.topRows<3>();
  a.b = P.b;
  a.d = d_hat;
  a.var_d = Sn(4, 4);
  a.cov_clock_isb = Eigen::MatrixXd::Zero(1 + k_obs, 1 + k_obs);
  a.cov_clock_isb(0, 0) = Sn(3, 3);
  for (int c = 0; c < k_obs; ++c) {
    a.cov_clock_isb(0, 1 + c) = a.cov_clock_isb(1 + c, 0) = Sn(3, 5 + c);
    for (int e = 0; e < k_obs; ++e) a.cov_clock_isb(1 + c, 1 + e) = Sn(5 + c, 5 + e);
  }
  for (int k = 0; k < 4; ++k) {
    a.isb_observed[k] = P.col_of[k] >= 0;
    a.isb_present[k] = isbSlotEnabled(cfg_, k);
    a.isb[k] = a.isb_observed[k] ? P.isb[k] : 0.0;
  }
  a.t_k = t_k;
  a.t_activate = last.t_rx;
  a.ok = true;
  return a;
}

bool Initializer::activate(const AlignResult& a, const CoreState& core, AugBlock* aug, std::string* why) const {
  auto fail = [&](const std::string& s) {
    if (why) *why = s;
    return false;
  };
  if (!a.ok) return fail("activate: alignment result is not ok");
  if (!aug) return fail("activate: no augmented block");
  for (const SlotInfo& s : aug->slots()) {
    if (s.kind != Slot::Isb) return fail("activate: slot already present: " + s.name + " (REINIT must drop frame/clock/td/clone slots)");
  }
  int k_obs = 0;
  for (int k = 0; k < 4; ++k)
    if (a.isb_observed[k]) ++k_obs;
  const int dimn = 5 + k_obs;
  if (a.cov_noise.rows() != dimn || a.cov_noise.cols() != dimn || a.J_noise_yaw.size() != dimn || a.J_noise_tilt.rows() != dimn ||
      a.J_noise_tilt.cols() != 2)
    return fail("activate: inconsistent noise model in the alignment result");
  const bool tight = cfg_.mode != Config::Mode::LoosePvt;
  const bool td = cfg_.time.estimate_online;

  // New components in slot order: tilt(2), yaw(1), trans(3), [clk(2)], [isb_k(1) …], [td(1)].
  struct NewSlot {
    Slot kind;
    int tag;
    std::string name;
    int dim;
    Eigen::VectorXd x0;
    int row = 0;   // first row in the stacked new state y
  };
  std::vector<NewSlot> ns;
  auto push = [&](Slot kind, int tag, const std::string& name, const Eigen::VectorXd& x0) {
    NewSlot s{kind, tag, name, static_cast<int>(x0.size()), x0, 0};
    s.row = ns.empty() ? 0 : ns.back().row + ns.back().dim;
    ns.push_back(s);
  };
  push(Slot::Tilt, 0, "tilt", Eigen::Vector2d::Zero());
  push(Slot::Yaw, 0, "yaw", (Eigen::VectorXd(1) << wrapPi(a.yaw)).finished());
  push(Slot::Trans, 0, "trans", a.trans);
  std::vector<int> isb_new;   // k of each new ISB slot, in order
  if (tight) {
    push(Slot::Clk, 0, "clk", Eigen::Vector2d(a.b, a.d));
    for (int k = 0; k < 4; ++k)
      if (a.isb_present[k]) {
        push(Slot::Isb, static_cast<int>(sysBit(isbSys(k))), isbSlotName(k),
             (Eigen::VectorXd(1) << (a.isb_observed[k] ? a.isb[k] : 0.0)).finished());
        isb_new.push_back(k);
      }
  }
  if (td) push(Slot::Td, 0, "td", Eigen::VectorXd::Zero(1));
  const int m = ns.back().row + ns.back().dim;

  // Independent noise sources: [n_φ(2), n_ψ(1), n(dimn), n_ιunobs(…), n_td(0/1)].
  int n_unobs = 0;
  for (int k : isb_new)
    if (!a.isb_observed[k]) ++n_unobs;
  const int n_src = 3 + dimn + n_unobs + (td ? 1 : 0);
  Eigen::MatrixXd Ssrc = Eigen::MatrixXd::Zero(n_src, n_src);
  const double sdov = cfg_.frames.sigma_dov_rad;
  const double slev = cfg_.frames.sigma_lev_rad;
  // A61 F1: deflection of the vertical plus the tilt the map has walked since it was levelled.
  // σ_lev: C(g) is treated as exact by the host (g has no process noise and a 1e-5 prior). The
  // leveling error lives here, not in Cov(φ, g): that cross-covariance cannot be consistent with
  // the host's g variance, which is far smaller than the accelerometer zero-g offset.
  Ssrc(0, 0) = Ssrc(1, 1) =
      sdov * sdov + slev * slev + cfg_.process.frame.q_tilt_per_m * std::max(0.0, a.map_path_m);
  Ssrc(2, 2) = a.sigma_yaw * a.sigma_yaw;
  Ssrc.block(3, 3, dimn, dimn) = 0.5 * (a.cov_noise + a.cov_noise.transpose());
  int src = 3 + dimn;
  for (int i = 0; i < n_unobs; ++i, ++src) Ssrc(src, src) = kIsbUnobservedSigma * kIsbUnobservedSigma;
  const int src_td = src;
  if (td) {
    const double s = a.sigma_toff_s > 0.0 ? a.sigma_toff_s : cfg_.time.sigma_toff_s;
    Ssrc(src_td, src_td) = s * s;
  }
  // G: y rows × sources. The ι observed at position c of the noise vector sits at 5 + c.
  Eigen::MatrixXd G = Eigen::MatrixXd::Zero(m, n_src);
  Eigen::MatrixXd Fh = Eigen::MatrixXd::Zero(m, HostIdx::dim);
  const NewSlot& s_tilt = ns[0];
  const NewSlot& s_yaw = ns[1];
  const NewSlot& s_trans = ns[2];
  G.block<2, 2>(s_tilt.row, 0).setIdentity();
  G(s_yaw.row, 2) = 1.0;
  G.block<1, 2>(s_yaw.row, 0) = a.J_yaw_tilt;   // A51 joint LS: ψ̂ depends on φ too (zero for §8)
  auto mapNoiseRow = [&](int yrow, int nrow) {   // y component driven by e_φ, e_ψ and n(nrow)
    G.block<1, 2>(yrow, 0) = a.J_noise_tilt.row(nrow);
    G(yrow, 2) = a.J_noise_yaw(nrow);
    G(yrow, 3 + nrow) = 1.0;
  };
  for (int i = 0; i < 3; ++i) mapNoiseRow(s_trans.row + i, i);
  Fh.block(s_trans.row, 0, 3, HostIdx::dim) = a.J_trans_host;
  if (tight) {
    const NewSlot& s_clk = ns[3];
    mapNoiseRow(s_clk.row, 3);
    mapNoiseRow(s_clk.row + 1, 4);
    int c_unobs = 0;
    for (size_t q = 0; q < isb_new.size(); ++q) {
      const int k = isb_new[q];
      const NewSlot& s = ns[4 + q];
      if (a.isb_observed[k]) {
        // position of k among the observed ISBs (noise order = isb order)
        int c = 0;
        for (int kk = 0; kk < k; ++kk)
          if (a.isb_observed[kk]) ++c;
        mapNoiseRow(s.row, 5 + c);
      } else {
        G(s.row, 3 + dimn + c_unobs) = 1.0;
        ++c_unobs;
      }
    }
  }
  if (td) G(ns.back().row, src_td) = 1.0;
  const Eigen::MatrixXd GSG = G * Ssrc * G.transpose();
  if (!GSG.allFinite() || !Fh.allFinite()) return fail("activate: non-finite augmentation");
  {
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(0.5 * (GSG + GSG.transpose()));
    if (es.info() != Eigen::Success) return fail("activate: eigen decomposition of the new-slot noise failed");
    const double lmax = std::max(es.eigenvalues().maxCoeff(), 0.0);
    if (es.eigenvalues().minCoeff() < -1e-12 * std::max(lmax, 1.0)) return fail("activate: new-slot noise is not positive semidefinite");
  }

  // ISB slots kept through a REINIT are replaced by the new estimate (their information is not reused: the batch saw the
  // same ι again). Removing a slot is the covariance-form marginalisation, exact for everything else (U9).
  for (int k : isb_new) {
    const int tag = static_cast<int>(sysBit(isbSys(k)));
    for (int i = 0; i < static_cast<int>(aug->slots().size()); ++i) {
      if (aug->slots()[i].kind == Slot::Isb && aug->slots()[i].tag == tag) {
        aug->removeSlot(i);
        break;
      }
    }
  }
  // Consistent augmentation, slot by slot: addSlot() adds F_host P F_hostᵀ, the host cross-covariance F_host P and
  // F_host C_old against the slots already there (incl. the new ones before it) plus this slot's own noise block …
  std::vector<int> offs;
  const Eigen::MatrixXd no_aug(0, 0);
  for (const NewSlot& s : ns) {
    const Eigen::MatrixXd Fs = Fh.block(s.row, 0, s.dim, HostIdx::dim);
    const Eigen::MatrixXd Ns = GSG.block(s.row, s.row, s.dim, s.dim);
    aug->addSlot(s.kind, s.tag, s.name, s.x0, Fs, no_aug, Ns, core.P);
    offs.push_back(aug->find(s.kind, s.tag));
  }
  // … and the noise shared between different new slots (t with ψ/φ, b/ι with t, ψ, φ) goes in here.
  Eigen::MatrixXd& Q = aug->Q();
  for (size_t u = 0; u < ns.size(); ++u) {
    for (size_t v = 0; v < ns.size(); ++v) {
      if (u == v) continue;
      Q.block(offs[u], offs[v], ns[u].dim, ns[v].dim) += GSG.block(ns[u].row, ns[v].row, ns[u].dim, ns[v].dim);
    }
  }
  aug->symmetrize();
  aug->wrapYaw();
  return true;
}

}  // namespace livo_gnss
