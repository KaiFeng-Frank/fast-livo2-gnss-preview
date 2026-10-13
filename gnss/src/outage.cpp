// SPDX-License-Identifier: BSD-3-Clause
// OutageManager (DESIGN §11, §10 L6) and the two state actions it asks for (REANCHOR, RECOVERY clock reset).
// Owner: P3-A1. Tests: U17 (U17Outage), U15.
//
// Everything here is either a 0/1 decision on counters and timers or a consistent augmentation (§2.4): a replaced slot is
// removed (covariance-form marginalisation, exact for every other entry, U9) and re-added as y = F·x + n with its
// dependence F on the remaining joint state and an independent noise n. No covariance is inflated, clamped or ramped.
#include "internal/outage.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <Eigen/Dense>

#include "internal/clock.hpp"

namespace livo_gnss {

namespace {

constexpr double kCLight = 299792458.0;
constexpr double kOmgE = 7.2921151467e-5;

double rangeWithSagnac(const Eigen::Vector3d& rs, const Eigen::Vector3d& rr, Eigen::Vector3d* e) {
  const Eigen::Vector3d d = rs - rr;
  const double r = d.norm();
  if (e) *e = d / r;
  return r + kOmgE * (rs.x() * rr.y() - rs.y() * rr.x()) / kCLight;
}

Eigen::RowVector3d rangeGradient(const Eigen::Vector3d& rs, const Eigen::Vector3d& e) {
  return Eigen::RowVector3d(-e.x() - kOmgE / kCLight * rs.y(), -e.y() + kOmgE / kCLight * rs.x(), -e.z());
}

bool contains(const std::vector<SatId>& v, const SatId& s) {
  for (const SatId& x : v)
    if (x == s) return true;
  return false;
}

// ISB column of a system in gnss_only's SPP state ([GLO, GAL, BDS, QZS]); −1 = GPS clock.
int sppIsbIndex(Sys sys, bool qzs_as_gps) {
  switch (sys) {
    case Sys::GLO: return 0;
    case Sys::GAL: return 1;
    case Sys::BDS: return 2;
    case Sys::QZS: return qzs_as_gps ? -1 : 3;
    default: return -1;
  }
}

// Column map of the aug block after deleting the slots whose offsets/dims are listed: old column → new column (−1 = gone).
std::vector<int> columnMapWithout(int n, const std::vector<std::pair<int, int>>& removed) {
  std::vector<int> map(n);
  int c = 0;
  for (int i = 0; i < n; ++i) {
    bool gone = false;
    for (const auto& r : removed)
      if (i >= r.first && i < r.first + r.second) gone = true;
    map[i] = gone ? -1 : c++;
  }
  return map;
}

int slotIndexOf(const AugBlock& aug, Slot kind) {
  for (int i = 0; i < static_cast<int>(aug.slots().size()); ++i)
    if (aug.slots()[i].kind == kind) return i;
  return -1;
}

bool psdOk(const Eigen::MatrixXd& S) {
  if (!S.allFinite()) return false;
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(0.5 * (S + S.transpose()));
  if (es.info() != Eigen::Success) return false;
  const double lmax = std::max(es.eigenvalues().maxCoeff(), 0.0);
  return es.eigenvalues().minCoeff() >= -1e-12 * std::max(lmax, 1.0);
}

}  // namespace

// =============================================================================================
// OutageManager
// =============================================================================================
OutageManager::OutageManager(const Config::Outage& cfg, const Config::Rejection& rej, bool l6_enabled)
    : cfg_(cfg), rej_(rej), l6_enabled_(l6_enabled) {}

void OutageManager::setReacqEpochs(int n) { reacq_epochs_ = std::max(0, n); }

OutageDecision OutageManager::step(double t_k, bool epoch_arrived, int n_valid_sats, bool l1_consistent, double l4_reject_frac_psr,
                                   double l4_reject_frac_dopp, double sigma_t_m, double sigma_yaw_rad, double sigma_b_m) {
  OutageDecision d;
  const bool valid = epoch_arrived && n_valid_sats > 0;
  if (valid) t_last_valid_ = t_k;
  // reacquisition window: this cycle's epochs run the fixed three iterations (§4.8-3, §11)
  auto consumeReacq = [&]() {
    if (epoch_arrived && reacq_left_ > 0) {
      d.reacq_epochs_left = reacq_left_;
      --reacq_left_;
    }
  };
  switch (phase_) {
    case Phase::TRACKING:
    case Phase::RECOVERY: {
      if (!valid && t_k - t_last_valid_ > cfg_.outage_s) {
        // §11 OUTAGE: no L0-valid epoch for outage_s. Clones are parked by the caller; nothing else changes here.
        phase_ = Phase::OUTAGE;
        t_outage_start_ = t_last_valid_;
        settling_ = false;
        reacq_left_ = 0;
        n_lockout_ = n_lockout_dopp_ = 0;
        d.entered_outage = true;
        break;
      }
      consumeReacq();
      if (phase_ == Phase::RECOVERY && reacq_left_ == 0) phase_ = Phase::TRACKING;
      // L6: consecutive epochs (of the class, NaN = no candidates = no evidence) with a consistent GNSS-only solution
      // but a rejecting L4 — the filter disagrees with every satellite at once, so its frame/clock are what is wrong.
      if (l6_enabled_ && epoch_arrived) {
        if (std::isfinite(l4_reject_frac_psr)) {
          if (l1_consistent && l4_reject_frac_psr > rej_.max_reject_frac) ++n_lockout_; else n_lockout_ = 0;
        }
        if (std::isfinite(l4_reject_frac_dopp)) {
          if (l1_consistent && l4_reject_frac_dopp > rej_.max_reject_frac) ++n_lockout_dopp_; else n_lockout_dopp_ = 0;
        }
        if (n_lockout_ >= rej_.lockout_epochs) {
          if (n_lockout_dopp_ >= rej_.lockout_epochs) {
            // the Doppler rejects too: ψ is wrong as well and only COLLECT→ALIGN can redo it (§11 "或 L6/L7 触发")
            d.reanchor_yaw = true;
            d.reinit = true;
          } else {
            d.reanchor = true;
            reacq_left_ = reacq_epochs_;
          }
          n_lockout_ = n_lockout_dopp_ = 0;
        }
      }
      break;
    }
    case Phase::OUTAGE: {
      const double dur = t_k - t_outage_start_;
      if (dur > cfg_.reinit_after_s && (sigma_t_m > cfg_.reinit_sigma_t_m || sigma_yaw_rad > cfg_.reinit_sigma_psi_rad)) {
        d.reinit = true;   // the frame has walked too far to be reacquired: COLLECT→ALIGN (phase kept until onReinitDone)
        break;
      }
      if (epoch_arrived && n_valid_sats >= cfg_.recover_min_sats) {
        if (!settling_) {
          settling_ = true;
          t_settle_start_ = t_k;
        }
      } else if (epoch_arrived || t_k - t_last_valid_ > cfg_.outage_s) {
        settling_ = false;   // a thin epoch, or the sky closed again
      }
      if (settling_ && t_k - t_settle_start_ >= cfg_.recover_settle_s) {
        phase_ = Phase::RECOVERY;
        settling_ = false;
        n_lockout_ = n_lockout_dopp_ = 0;
        d.left_outage = true;
        d.start_recovery = true;
        d.clock_reset_needed = sigma_b_m > kClockResetSigmaB;
        reacq_left_ = reacq_epochs_;
        consumeReacq();
      }
      break;
    }
    default:
      break;
  }
  d.phase = phase_;
  if (phase_ == Phase::OUTAGE) d.outage_duration_s = t_k - t_outage_start_;
  return d;
}

void OutageManager::onActivated(double t_k) {
  phase_ = Phase::TRACKING;
  t_last_valid_ = t_outage_start_ = t_settle_start_ = t_k;
  settling_ = false;
  n_lockout_ = n_lockout_dopp_ = 0;
  reacq_left_ = reacq_epochs_;
}

void OutageManager::onReinitDone(double t_k) {
  phase_ = Phase::TRACKING;   // dormant until the next onActivated(): the Initializer owns the phase meanwhile
  t_last_valid_ = t_outage_start_ = t_settle_start_ = t_k;
  settling_ = false;
  n_lockout_ = n_lockout_dopp_ = 0;
  reacq_left_ = 0;
}

// =============================================================================================
// L6 REANCHOR
// =============================================================================================
bool reanchorFromSpp(AugBlock* aug, const AntennaJacobians& J, const AnchorFrame& anchor, const SppSolution& spp,
                     const DopplerSolution& dop, const std::vector<SppInput>& in, double delta_s, const Config& cfg,
                     const HostMat& P_host, std::string* why) {
  auto fail = [&](const std::string& s) {
    if (why) *why = s;
    return false;
  };
  if (!aug) return fail("reanchor: no augmented block");
  if (!spp.ok || !dop.ok) return fail("reanchor: GNSS-only SPP / Doppler LS not available");
  if (!anchor.valid) return fail("reanchor: no N frame");
  for (const SlotInfo& s : aug->slots())
    if (s.kind == Slot::Clone || s.kind == Slot::Phase) return fail("reanchor: park the TDCP clones first (" + s.name + ")");
  const int n = aug->dim();
  const int i_t = aug->find(Slot::Trans), i_c = aug->find(Slot::Clk);
  if (i_t < 0 || i_c < 0) return fail("reanchor: trans/clk slot absent");
  if (J.J_a.cols() != HostIdx::dim + n || J.J_w.cols() != HostIdx::dim + n) return fail("reanchor: Jacobians do not match the aug layout");

  // Formal SPP covariance of [r, b] (ISBs of the SPP as nuisance), a-priori weights, scaled by max(1, σ̂0²).
  const bool qzs = cfg.systems.qzs_as_gps;
  int cols[4] = {-1, -1, -1, -1}, p = 4;
  for (const SppInput& s : in)
    if (contains(spp.used, s.sat)) {
      const int k = sppIsbIndex(s.sat.sys, qzs);
      if (k >= 0 && cols[k] < 0) cols[k] = p++;
    }
  Eigen::MatrixXd Nm = Eigen::MatrixXd::Zero(p, p);
  int m = 0;
  for (const SppInput& s : in) {
    if (!contains(spp.used, s.sat) || !s.state.ok) continue;
    Eigen::Vector3d e;
    rangeWithSagnac(s.state.r_ecef, spp.r_ecef, &e);
    Eigen::RowVectorXd h = Eigen::RowVectorXd::Zero(p);
    h.head<3>() = rangeGradient(s.state.r_ecef, e);
    h(3) = 1.0;
    const int k = sppIsbIndex(s.sat.sys, qzs);
    if (k >= 0) h(cols[k]) = 1.0;
    Nm.noalias() += h.transpose() * h / (s.var_psr > 0.0 ? s.var_psr : 1.0);
    ++m;
  }
  if (m < p) return fail("reanchor: SPP rows missing from the inputs");
  Eigen::FullPivLU<Eigen::MatrixXd> lu(Nm);
  if (!lu.isInvertible()) return fail("reanchor: singular SPP geometry");
  Eigen::MatrixXd Qs = lu.inverse() * std::max(1.0, spp.sigma0 * spp.sigma0);
  Qs = 0.5 * (Qs + Qs.transpose()).eval();
  const double floor2 = cfg.init.spp_sigma_floor_m * cfg.init.spp_sigma_floor_m;
  for (int i = 0; i < 3; ++i) Qs(i, i) = std::max(Qs(i, i), floor2);   // §8 max(formal·σ̂0², 3² m²)
  const Eigen::Matrix3d& REN = anchor.R_EN;
  const Eigen::Matrix3d S_aa = REN.transpose() * Qs.topLeftCorner<3, 3>() * REN;
  const Eigen::Vector3d S_ab = REN.transpose() * Qs.block<3, 1>(0, 3);
  const double S_bb = Qs(3, 3);
  // Doppler-LS drift variance of the epoch (same recomputation).
  Eigen::Matrix4d Nd = Eigen::Matrix4d::Zero();
  int md = 0;
  for (const SppInput& s : in) {
    if (!contains(dop.used, s.sat) || !s.state.ok) continue;
    Eigen::Vector3d e;
    rangeWithSagnac(s.state.r_ecef, spp.r_ecef, &e);
    Eigen::RowVector4d h;
    h.head<3>() = rangeGradient(s.state.r_ecef, e);
    h(3) = 1.0;
    Nd.noalias() += h.transpose() * h / (s.var_dopp > 0.0 ? s.var_dopp : 1.0);
    ++md;
  }
  if (md < 4) return fail("reanchor: Doppler rows missing from the inputs");
  Eigen::FullPivLU<Eigen::Matrix4d> lud(Nd);
  if (!lud.isInvertible()) return fail("reanchor: singular Doppler geometry");
  const double var_d = lud.inverse()(3, 3) * std::max(1.0, dop.sigma0 * dop.sigma0);

  const double dl = std::max(0.0, delta_s);
  ClockModel cm(cfg.process.clock);
  const Eigen::Matrix2d Qc = cm.Q(dl, 0.0, 0.0);
  Eigen::Matrix2d N_clk;
  N_clk << S_bb + dl * dl * var_d + Qc(0, 0), dl * var_d + Qc(0, 1), dl * var_d + Qc(0, 1), var_d + Qc(1, 1);
  const Eigen::Vector3d aN_spp = REN.transpose() * (spp.r_ecef - anchor.r0_ecef);
  const Eigen::Vector3d t_new = aug->trans() + (aN_spp - J.a_N);
  const Eigen::Vector2d clk_new(spp.b_m + dl * dop.d_mps, dop.d_mps);
  if (!psdOk(S_aa) || !psdOk(N_clk) || !t_new.allFinite() || !clk_new.allFinite()) return fail("reanchor: non-finite / indefinite SPP noise");

  // e_t' = n − (J_a e without the t columns): F over [host | aug] in the old layout, the t (and clk) columns dropped.
  const int o_t = i_t, o_c = i_c;
  const std::vector<int> map = columnMapWithout(n, {{o_t, 3}, {o_c, 2}});
  const int n_new = n - 5;
  Eigen::MatrixXd F_host = -J.J_a.leftCols<HostIdx::dim>();
  Eigen::MatrixXd F_aug = Eigen::MatrixXd::Zero(3, n_new);
  for (int c = 0; c < n; ++c)
    if (map[c] >= 0) F_aug.col(map[c]) = -J.J_a.col(HostIdx::dim + c);

  // apply: drop t and clk (marginalisation), re-add consistently, then the SPP position/clock noise between them
  aug->removeSlot(slotIndexOf(*aug, Slot::Trans));
  aug->removeSlot(slotIndexOf(*aug, Slot::Clk));
  aug->addSlot(Slot::Trans, 0, "trans", t_new, F_host, F_aug, S_aa, P_host);
  aug->addSlot(Slot::Clk, 0, "clk", clk_new, Eigen::MatrixXd::Zero(2, HostIdx::dim), Eigen::MatrixXd(0, 0), N_clk, P_host);
  const int nt = aug->find(Slot::Trans), nc = aug->find(Slot::Clk);
  aug->Q().block<3, 1>(nt, nc) += S_ab;
  aug->Q().block<1, 3>(nc, nt) += S_ab.transpose();
  aug->symmetrize();
  return true;
}

// =============================================================================================
// RECOVERY clock reset (σ_b > 1e3 m)
// =============================================================================================
bool recoveryClockReset(AugBlock* aug, const AntennaJacobians& J, const AnchorFrame& anchor, const std::vector<SppInput>& in,
                        double delta_s, const Config& cfg, const HostMat& P_host, ClockOnlyResult* result, std::string* why) {
  auto fail = [&](const std::string& s) {
    if (why) *why = s;
    return false;
  };
  if (!aug) return fail("clock reset: no augmented block");
  if (!anchor.valid) return fail("clock reset: no N frame");
  const int n = aug->dim();
  const int i_c = aug->find(Slot::Clk);
  if (i_c < 0) return fail("clock reset: clk slot absent");
  if (J.J_a.cols() != HostIdx::dim + n || J.J_w.cols() != HostIdx::dim + n) return fail("clock reset: Jacobians do not match the aug layout");
  const Eigen::Matrix3d& REN = anchor.R_EN;
  const Eigen::Vector3d r_pred = anchor.r0_ecef + REN * J.a_N;
  const Eigen::Vector3d v_pred = REN * J.w_N;
  const ClockOnlyResult co = clockOnlyLs(in, r_pred, v_pred);
  if (result) *result = co;
  if (!co.ok) return fail("clock reset: clock-only LS needs a GPS/QZSS code");
  // Mean lines of sight of exactly the satellites clockOnlyLs uses for b and for d.
  Eigen::Vector3d eb = Eigen::Vector3d::Zero(), ed = Eigen::Vector3d::Zero();
  int nb = 0, nd = 0;
  for (const SppInput& s : in) {
    if (!s.state.ok) continue;
    Eigen::Vector3d e;
    rangeWithSagnac(s.state.r_ecef, r_pred, &e);
    if (s.psr_usable && s.P_corr > 0.0 && (s.sat.sys == Sys::GPS || s.sat.sys == Sys::QZS)) {
      eb += e;
      ++nb;
    }
    if (s.dopp_usable && s.lambda_m > 0.0 && s.dopp_hz != 0.0 && s.state.v_ecef.norm() > 0.0) {
      ed += e;
      ++nd;
    }
  }
  if (nb == 0 || nd == 0) return fail("clock reset: needs GPS/QZSS codes and Dopplers");
  eb /= nb;
  ed /= nd;
  // b_j = P − ρ(r_pred) + … with ρ(r_true + e_r) ≈ ρ − eᵀe_r ⇒ e_b = n_b + ēᵀ e_r, e_r = R_EN·J_a·e; d likewise with J_w.
  const Eigen::RowVectorXd Fb = eb.transpose() * REN * J.J_a;
  const Eigen::RowVectorXd Fd = ed.transpose() * REN * J.J_w;
  const double dl = std::max(0.0, delta_s);
  ClockModel cm(cfg.process.clock);
  const Eigen::Matrix2d Qc = cm.Q(dl, 0.0, 0.0);
  Eigen::Matrix2d N_clk;
  N_clk << co.var_b + dl * dl * co.var_d + Qc(0, 0), dl * co.var_d + Qc(0, 1), dl * co.var_d + Qc(0, 1), co.var_d + Qc(1, 1);
  if (!psdOk(N_clk)) return fail("clock reset: indefinite clock noise");
  Eigen::MatrixXd F(2, HostIdx::dim + n);
  F.row(0) = Fb + dl * Fd;   // b(t_k) = b(t_g) + δ·d
  F.row(1) = Fd;
  const std::vector<int> map = columnMapWithout(n, {{i_c, 2}});
  Eigen::MatrixXd F_aug = Eigen::MatrixXd::Zero(2, n - 2);
  for (int c = 0; c < n; ++c)
    if (map[c] >= 0) F_aug.col(map[c]) = F.col(HostIdx::dim + c);
  const Eigen::MatrixXd F_host = F.leftCols<HostIdx::dim>();
  if (!F.allFinite()) return fail("clock reset: non-finite sensitivity");
  aug->removeSlot(slotIndexOf(*aug, Slot::Clk));
  aug->addSlot(Slot::Clk, 0, "clk", Eigen::Vector2d(co.b_m + dl * co.d_mps, co.d_mps), F_host, F_aug, N_clk, P_host);
  aug->symmetrize();
  return true;
}

}  // namespace livo_gnss
