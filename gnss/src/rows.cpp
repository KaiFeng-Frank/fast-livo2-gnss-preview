// SPDX-License-Identifier: BSD-3-Clause
// rows (DESIGN §4.2–§4.6). Owner: WP-c. Tests: U7, U10, U11.
//
// Conventions (DESIGN §1): ν = z − ĥ(x̂), δx = Kν; host right perturbation on R, additive elsewhere; ψ left
// perturbation (commutes with R_z); increments of the augmented block are additive. The column layout is
// [host 19 | x_G slots in AugBlock order]. Every Jacobian here is the exact derivative of the nonlinear
// model that ImuBridge::evaluate() + Leveling::M() implement (AMENDMENTS A10), so U7 can check them by
// manifold central differences; the §4.2 table is the special case of a first iterate (δθ_x = 0, φ = 0).
#include "internal/rows.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <numeric>

#include <Eigen/Cholesky>
#include <Eigen/Dense>

namespace livo_gnss {

namespace {

constexpr double kCLight = 299792458.0;        // rtklib.h CLIGHT
constexpr double kOmgE = 7.2921151467e-5;      // rtklib.h OMGE
constexpr double kFreqL1 = 1.57542e9;
constexpr double kErrIonStd = 5.0;             // pntpos.c ERR_ION: σ_ion when no Klobuchar (§8 WAIT_NAV)
constexpr double kErrSaas = 0.3;               // pntpos.c ERR_SAAS
constexpr double kRelHumi = 0.7;               // pntpos.c REL_HUMI

// Geometric range with the Sagnac correction, identical to RTKLIB geodist (rtkcmn.c:3599) without the
// ‖rs‖ < RE guard; kept local so the TDCP k0 range and the tests do not depend on rtk_adapter (WP-a).
double rangeWithSagnac(const Eigen::Vector3d& rs, const Eigen::Vector3d& rr, Eigen::Vector3d* e) {
  const Eigen::Vector3d d = rs - rr;
  const double r = d.norm();
  if (e) *e = d / r;
  return r + kOmgE * (rs.x() * rr.y() - rs.y() * rr.x()) / kCLight;
}

// SO(3) left Jacobian J_l(θ) and inverse right Jacobian J_r⁻¹(θ) (Barfoot 2017 eqs. 7.36 / 7.43a).
Eigen::Matrix3d so3LeftJacobian(const Eigen::Vector3d& th) {
  const double a = th.norm();
  const Eigen::Matrix3d K = skew(th);
  if (a < 1e-7) return Eigen::Matrix3d::Identity() + 0.5 * K;
  const double a2 = a * a;
  return Eigen::Matrix3d::Identity() + (1.0 - std::cos(a)) / a2 * K + (a - std::sin(a)) / (a2 * a) * K * K;
}
Eigen::Matrix3d so3RightJacobianInv(const Eigen::Vector3d& th) {
  const double a = th.norm();
  const Eigen::Matrix3d K = skew(th);
  if (a < 1e-7) return Eigen::Matrix3d::Identity() + 0.5 * K;
  const double a2 = a * a;
  return Eigen::Matrix3d::Identity() + 0.5 * K + (1.0 / a2 - (1.0 + std::cos(a)) / (2.0 * a * std::sin(a))) * K * K;
}

// GLONASS channel k from the delivered frequency for the (f_L1/f)² ionosphere scaling.
double ionoScale(double freq_hz) {
  if (!(freq_hz > 0.0)) return 1.0;
  const double r = kFreqL1 / freq_hz;
  return r * r;
}

struct SlotView {
  int tilt = -1, yaw = -1, trans = -1, clk = -1, clone = -1, td = -1;
  Eigen::Vector2d phi = Eigen::Vector2d::Zero();
  double psi = 0.0;
  Eigen::Vector3d t = Eigen::Vector3d::Zero();
  double b = 0.0, d = 0.0;
};

// Reads the registration/clock values straight from x_G at the slot offsets (the slot layout is
// AugBlock's contract: tilt = 2, yaw = 1, trans = 3, clk = [b, d]).
SlotView readSlots(const AugBlock* aug) {
  SlotView s;
  if (!aug) return s;
  s.tilt = aug->find(Slot::Tilt);
  s.yaw = aug->find(Slot::Yaw);
  s.trans = aug->find(Slot::Trans);
  s.clk = aug->find(Slot::Clk);
  s.td = aug->find(Slot::Td);
  const Eigen::VectorXd& x = aug->x();
  if (s.tilt >= 0) s.phi = x.segment<2>(s.tilt);
  if (s.yaw >= 0) s.psi = x(s.yaw);
  if (s.trans >= 0) s.t = x.segment<3>(s.trans);
  if (s.clk >= 0) {
    s.b = x(s.clk);
    s.d = x(s.clk + 1);
  }
  return s;
}

inline void addColumn(Eigen::RowVectorXd& H, int col, double v) {
  if (col >= 0 && col < H.size()) H(col) += v;
}

}  // namespace

// ---------------------------------------------------------------------------------------------
// AMENDMENTS A43: 1 Hz selection of the pseudorange / L-PVT epochs — per GPST bin ⌊t/decim_s⌋ the first epoch to arrive.
// It replaces DESIGN §4.3's |tow − round(tow)| < 5 ms, which assumed a receiver grid on the whole tenths: the M3DGR
// grids sit −7 … +5 ms off them per recording (audit G15), so that rule took no epoch at all on Grass01 and
// Occlusion03/04 and dropped 13 % of the seconds on Dynamic03, silently.
// ---------------------------------------------------------------------------------------------
EpochDecimator::EpochDecimator(double decim_s) {
  // validateConfig() requires a positive decim_s; a direct caller with anything else gets the 1 s of DESIGN §14.
  decim_s_ = (std::isfinite(decim_s) && decim_s > 0.0) ? decim_s : 1.0;
  const double r = std::round(decim_s_);
  whole_s_ = (r >= 1.0 && decim_s_ == r) ? static_cast<int64_t>(r) : 0;
}

int64_t EpochDecimator::binOf(const GTime& t) const {
  // GTime keeps frac in [0, 1); fold anything else into the integer part first.
  const double w = std::floor(t.frac);
  const int64_t sec = t.sec + static_cast<int64_t>(w);
  const double frac = t.frac - w;
  if (whole_s_ > 0) {
    // ⌊(sec + frac)/W⌋ = ⌊sec/W⌋ for an integer W ≥ 1 and 0 ≤ frac < 1: exact at any magnitude of sec (≈ 1.7e9 here,
    // where a double sum would carry 2.4e-7 s of rounding right at the bin edges). Floor division for negative sec too.
    int64_t q = sec / whole_s_;
    if (sec % whole_s_ != 0 && sec < 0) --q;
    return q;
  }
  return static_cast<int64_t>(std::floor((static_cast<double>(sec) + frac) / decim_s_));
}

EpochDecimator::Verdict EpochDecimator::offer(const GTime& t) {
  // The same epoch again (relinearisation, or rows built after the arrival-time offer): its first verdict, uncounted.
  for (auto it = recent_.rbegin(); it != recent_.rend(); ++it) {
    if (it->first == t) return it->second;
  }
  Verdict v;
  if (!t.valid() || !std::isfinite(t.frac)) {
    v = Verdict::NoTime;                  // cannot be binned: no row (not a decimation skip, not counted)
  } else {
    const int64_t b = binOf(t);
    if (!has_bin_ || b > last_bin_) {
      v = Verdict::Take;
      has_bin_ = true;
      last_bin_ = b;
      ++n_take_;
    } else if (b == last_bin_) {
      v = Verdict::SameBin;
      ++n_same_;
    } else {
      v = Verdict::Stale;                 // a bin before the latest one taken: arrived out of order
      ++n_stale_;
    }
  }
  recent_.emplace_back(t, v);
  if (recent_.size() > kRecent) recent_.pop_front();
  return v;
}

void EpochDecimator::reset() {
  has_bin_ = false;
  last_bin_ = 0;
  recent_.clear();
  n_take_ = n_same_ = n_stale_ = 0;
}

RowBuilder::RowBuilder(const Config& cfg, const NoiseModel& noise)
    : cfg_(cfg), noise_(noise), psr_decim_(cfg.noise.psr.decim_s), pvt_decim_(cfg.noise.pvt.decim_s) {}

EpochDecimator::Verdict RowBuilder::offerPsrEpoch(const GTime& t_rx) const { return psr_decim_.offer(t_rx); }
EpochDecimator::Verdict RowBuilder::offerPvtEpoch(const GTime& t) const { return pvt_decim_.offer(t); }

// ---------------------------------------------------------------------------------------------
// §4.2 J_a / J_w
// ---------------------------------------------------------------------------------------------
AntennaJacobians RowBuilder::antennaJacobians(const LinPoint& lp) const {
  AntennaJacobians out;
  const CoreState& x = *lp.x;
  const BridgeResult& br = *lp.bridge;
  const Eigen::Vector3d& l = lp.lever_I;
  const int n = lp.aug ? lp.aug->dim() : 0;
  const int N = HostIdx::dim + n;
  const double d = br.delta;

  const AntennaAtEpoch ae = ImuBridge::evaluate(x, br, l, lp.gravity_est_en);
  const SlotView s = readSlots(lp.aug);
  const Eigen::Matrix3d M = lp.leveling->M(s.psi, s.phi, x.g);
  const Eigen::Matrix3d Rz = Leveling::Rz(s.psi);
  const Eigen::Vector3d y_rel = ae.y - lp.pivot_W;      // y − o^W
  out.M = M;
  out.a_N = M * y_rel + s.t;
  out.w_N = M * ae.z;
  // Antenna acceleration in W: a_g ≈ R_g ā_b + g plus the centripetal lever term R_g[ω]×[ω]×l (ω̇ ignored).
  const Eigen::Matrix3d Wg = skew(ae.omega_g);
  const Eigen::Vector3d acc_W = ae.R_g * br.a_bar + x.g + ae.R_g * (Wg * (Wg * l));
  out.alpha_N = M * acc_W;

  Eigen::Matrix<double, 3, Eigen::Dynamic>& Ja = out.J_a;
  Eigen::Matrix<double, 3, Eigen::Dynamic>& Jw = out.J_w;
  Ja.setZero(3, N);
  Jw.setZero(3, N);

  // Iterate attitude relative to the bridge snapshot: δa depends on δθ_x = Log(R̂_kᵀ R) (A10); its
  // derivative w.r.t. a right perturbation of R is J_r⁻¹(δθ_x) (identity on the first iterate).
  const Eigen::Vector3d dth_x = so3Log(Eigen::Matrix3d(br.R_k.transpose() * x.R));
  const Eigen::Matrix3d Jri = so3RightJacobianInv(dth_x);
  const Eigen::Matrix3d MR = M * x.R;
  const Eigen::Matrix3d MRk_abar = M * br.R_k * skew(br.a_bar) * Jri;
  const Eigen::Vector3d dRt_l = br.dR.transpose() * l;
  const Eigen::Vector3d dRt_wl = br.dR.transpose() * (Wg * l);

  // δθ: y = p − vδ − D(x) + R ΔR̂ᵀ l ;  z = v − E(x) + R ΔR̂ᵀ [ω_g]× l
  Ja.block<3, 3>(0, HostIdx::rot) = -MR * skew(dRt_l) - 0.5 * d * d * MRk_abar;
  Jw.block<3, 3>(0, HostIdx::rot) = -MR * skew(dRt_wl) + d * MRk_abar;
  // δp, δv
  Ja.block<3, 3>(0, HostIdx::pos) = M;
  Ja.block<3, 3>(0, HostIdx::vel) = -d * M;
  Jw.block<3, 3>(0, HostIdx::vel) = M;
  // δb_g: ω_g = ω̂_g + b̂_g − b_g ⇒ ∂([ω_g]×l)/∂b_g = +[l]×  (y does not depend on b_g in the A10 model)
  Jw.block<3, 3>(0, HostIdx::bg) = M * ae.R_g * skew(l);
  // δb_a: δa = … − R̂_k(b_a − b̂_a) ⇒ ∂y/∂b_a = −½δ² R̂_k, ∂z/∂b_a = +δ R̂_k
  Ja.block<3, 3>(0, HostIdx::ba) = -0.5 * d * d * M * br.R_k;
  Jw.block<3, 3>(0, HostIdx::ba) = d * M * br.R_k;
  // Ξ(y) = −R_z(ψ)[Exp(φ̃)C(g)(y − o^W)]× = −R_z [R_zᵀ M (y − o^W)]× ; same for z.
  const Eigen::Matrix3d Xi_y = -Rz * skew(Eigen::Vector3d(Rz.transpose() * (M * y_rel)));
  const Eigen::Matrix3d Xi_z = -Rz * skew(Eigen::Vector3d(Rz.transpose() * (M * ae.z)));
  const Eigen::Vector3d phi_t(s.phi(0), s.phi(1), 0.0);
  // The same Exp as Leveling::M() (no 1e-5 dead zone, unlike the host's so3Exp), so this column is the exact
  // derivative of the M() the rows are linearised about.
  const Eigen::Matrix3d Exp_phi = Leveling::expTilt(s.phi);
  // δg: C(g+δg) ≈ Exp(ε)C(g), ε = [e_z]×G_⊥ δg (D1 §2.2); M' = R_z Exp(φ̃) Exp(ε) C ⇒ Ξ(y)·Exp(φ̃)·[e_z]×G_⊥
  // (reduces to the §4.2 entry at φ = 0), plus the ½δ²M / −δM terms of evaluate() (A10). ε is the exact
  // derivative of C(·) only at g = g_ref (error ≈ 0.59·∠(g, g_ref), review n5); onStepEnd() refreshes C_ref
  // with the final g every step, so the first iterate of an epoch always linearises at ∠ = 0.
  if (lp.gravity_est_en && cfg_.frames.gnss_updates_gravity) {
    const Eigen::Matrix3d Eps = lp.leveling->epsJacobian(x.g);   // [e_z]× G_⊥, third row zero
    Ja.block<3, 3>(0, HostIdx::grav) = Xi_y * Exp_phi * Eps + 0.5 * d * d * M;
    Jw.block<3, 3>(0, HostIdx::grav) = Xi_z * Exp_phi * Eps - d * M;
  }
  // δφ (additive on φ): Exp(φ̃ + δφ̃) = Exp(J_l(φ̃) δφ̃) Exp(φ̃) ⇒ Ξ(·) J_l(φ̃) E_xy
  if (s.tilt >= 0) {
    const Eigen::Matrix3d Jl = so3LeftJacobian(phi_t);
    const Eigen::Matrix3d XyJl = Xi_y * Jl;
    const Eigen::Matrix3d XzJl = Xi_z * Jl;
    Ja.block<3, 2>(0, HostIdx::dim + s.tilt) = XyJl.leftCols<2>();
    Jw.block<3, 2>(0, HostIdx::dim + s.tilt) = XzJl.leftCols<2>();
  }
  // δψ: R_z(ψ + δψ) = Exp(e_z δψ) R_z(ψ) exactly ⇒ [e_z]×(a^N − t), [e_z]× w^N
  if (s.yaw >= 0) {
    const Eigen::Vector3d ez(0.0, 0.0, 1.0);
    Ja.col(HostIdx::dim + s.yaw) = ez.cross(Eigen::Vector3d(M * y_rel));
    Jw.col(HostIdx::dim + s.yaw) = ez.cross(out.w_N);
  }
  // δt
  if (s.trans >= 0) Ja.block<3, 3>(0, HostIdx::dim + s.trans) = Eigen::Matrix3d::Identity();
  // δt_d (optional): −w^N / −α_A^N (§4.2)
  if (s.td >= 0) {
    Ja.col(HostIdx::dim + s.td) = -out.w_N;
    Jw.col(HostIdx::dim + s.td) = -out.alpha_N;
  }
  return out;
}

// ---------------------------------------------------------------------------------------------
// Geometry of one satellite at the antenna position (production path through rtk_adapter, WP-a).
// One body for the live ephemeris store and for a frozen snapshot: only the satellite-state evaluator differs.
// ---------------------------------------------------------------------------------------------
namespace {

using SatStateFn = std::function<bool(double pseudorange_m, rtk::SatState* out)>;

bool geometryImpl(const SatObs& obs, const SatStateFn& satState, const Eigen::Vector3d& r_A_ecef, const Geodetic& rcv_geo,
                  const Klobuchar* ion, GTime t, double clock_bias_m, double err_brdci, SatGeometry* out) {
  const SignalObs* sig = obs.primary();
  if (!sig || !out) return false;
  rtk::SatState st;
  if (sig->psrValid()) {
    if (!satState(sig->psr_m, &st) || !st.ok) return false;
  } else {
    // Carrier-only epoch (§1): transmit time from ρ̂ + b̂ − c·δt_s, fixed-point iterated from a coarse guess
    // (each pass reduces the t_tx error by ~c/v_s ≈ 1e5).
    double Pg = 2.2e7 + clock_bias_m;
    for (int it = 0; it < 6; ++it) {
      if (!satState(Pg, &st) || !st.ok) return false;
      const double rho = rangeWithSagnac(st.r_ecef, r_A_ecef, nullptr);
      const double Pn = rho + clock_bias_m - kCLight * st.dts;
      const bool done = std::fabs(Pn - Pg) < 1e-4;
      Pg = Pn;
      if (done) break;
    }
    if (!satState(Pg, &st) || !st.ok) return false;
  }
  out->sat = obs.sat;
  out->state = st;
  out->range = rtk::geodist(st.r_ecef, r_A_ecef, &out->e_ecef);
  if (!(out->range > 0.0)) return false;
  // `rcv_geo` is the receiver: az/el and both atmospheric models want the true antenna height and pierce point.
  // u_N here is the ENU of that receiver point (diagnostics); the rows use the anchor's R_EN (review m3).
  rtk::satAzel(rcv_geo, out->e_ecef, &out->az_rad, &out->el_rad);
  out->u_N = rtk::enuRotation(rcv_geo).transpose() * out->e_ecef;
  out->lambda_m = sig->freq_hz > 0.0 ? kCLight / sig->freq_hz : 0.0;
  const double k = ionoScale(sig->freq_hz);
  if (ion && ion->valid()) {
    const double i1 = rtk::ionModelGpsL1(t, *ion, rcv_geo, out->az_rad, out->el_rad);
    out->ion_m = i1 * k;                                                   // (f_L1/f)² (rescode :324)
    out->var_ion = (err_brdci * i1) * (err_brdci * i1) * k * k;
  } else {
    out->ion_m = 0.0;                                                      // §8 WAIT_NAV: I = 0, σ_ion = 5 m
    out->var_ion = kErrIonStd * kErrIonStd;
  }
  out->trop_m = rtk::tropModel(t, rcv_geo, out->az_rad, out->el_rad, kRelHumi);
  const double se = std::sin(out->el_rad) + 0.1;
  out->var_trop = (kErrSaas / se) * (kErrSaas / se);                       // tropcorr :263
  return out->el_rad > 0.0;
}

}  // namespace

bool RowBuilder::geometry(const SatObs& obs, const rtk::SatSolver& solver, const Eigen::Vector3d& r_A_ecef,
                          const Geodetic& rcv_geo, const Klobuchar* ion, GTime t, double clock_bias_m,
                          SatGeometry* out) const {
  const SatStateFn fn = [&](double P, rtk::SatState* st) { return solver.satState(obs.sat, t, P, st); };
  return geometryImpl(obs, fn, r_A_ecef, rcv_geo, ion, t, clock_bias_m, cfg_.noise.psr.err_brdci, out);
}

bool RowBuilder::geometryFromSnapshot(const SatObs& obs, const rtk::EphSnapshot& snap, const Eigen::Vector3d& r_A_ecef,
                                      const Geodetic& rcv_geo, const Klobuchar* ion, GTime t, double clock_bias_m,
                                      SatGeometry* out) const {
  if (!snap.valid()) return false;
  const SatStateFn fn = [&](double P, rtk::SatState* st) { return rtk::SatSolver::satStateFromSnapshot(snap, t, P, st); };
  return geometryImpl(obs, fn, r_A_ecef, rcv_geo, ion, t, clock_bias_m, cfg_.noise.psr.err_brdci, out);
}

// ---------------------------------------------------------------------------------------------
// §4.3 pseudorange rows (1 Hz: the first epoch to arrive in each GPST second, AMENDMENTS A43)
// ---------------------------------------------------------------------------------------------
void RowBuilder::pseudorangeRows(const ObsEpoch& epoch, const std::vector<SatGeometry>& geo,
                                 const std::vector<MaskVerdict>& masks, const LinPoint& lp, const AntennaJacobians& J,
                                 RowSet* out) const {
  out->cls = MeasClass::Psr;
  out->has_full_R = false;
  if (!cfg_.use.psr) return;
  // A43: the epoch's first offer decides (an arrival-time offerPsrEpoch() or this call); its relinearisations repeat
  // the decision. Every other epoch of the second builds no code row. κ_AR below keeps the nominal decim_s spacing.
  if (!psr_decim_.admit(epoch.t_rx)) return;
  const SlotView s = readSlots(lp.aug);
  if (s.clk < 0 || !lp.anchor) return;         // tight coupling needs the receiver clock (§2.2) and the N frame
  const int N = static_cast<int>(J.J_a.cols());
  const double delta = lp.bridge ? lp.bridge->delta : lp.delta;
  const Eigen::Matrix3d MSDMt = lp.bridge ? Eigen::Matrix3d(J.M * lp.bridge->Sigma_D * J.M.transpose()) : Eigen::Matrix3d::Zero();
  const Eigen::Matrix3d RENt = lp.anchor->R_EN.transpose();   // N = fixed ENU at the anchor (§1): the frame of J_a
  const size_t n = std::min(epoch.sats.size(), std::min(geo.size(), masks.size()));
  for (size_t i = 0; i < n; ++i) {
    if (!masks[i].psr || !geo[i].state.ok) continue;
    const SatObs& so = epoch.sats[i];
    const SignalObs* sig = so.primary();
    if (!sig || !sig->psrValid()) continue;
    const SatGeometry& g = geo[i];
    // P_corr (§4.3, AMENDMENTS A37): the broadcast group delay of the primary signal from the record the satellite
    // state was evaluated with, through the single rule rtk::codeBiasMetres() for every system (Galileo E1 takes the
    // BGD of that record's own clock — I/NAV BGD(E1,E5b), F/NAV BGD(E1,E5a) — not RTKLIB-EX prange()'s choice keyed
    // on the global selection). NaN (a secondary signal as primary, an unknown Galileo data source) ⇒ no code row.
    const double code_bias_m = rtk::codeBiasMetres(so.sat, g.state, sig->code);
    if (!std::isfinite(code_bias_m)) continue;
    const Eigen::Vector3d u = RENt * g.e_ecef;
    // ISB slot of the system (tag = system bit); QZSS rides on the GPS clock (ι_G ≡ 0). A system without its
    // slot has an unknown ι_s: its code rows would enter biased by metres, so they are not built (review m8);
    // the initialiser/glue creates the slot (§2.2, σ = 30 m for an unobserved system) before the code is used.
    int isb_col = -1;
    if (so.sat.sys != Sys::GPS && !(so.sat.sys == Sys::QZS && cfg_.systems.qzs_as_gps)) {
      isb_col = lp.aug ? lp.aug->find(Slot::Isb, sysBit(so.sat.sys)) : -1;
      if (isb_col < 0) continue;
    }
    const double iota = isb_col >= 0 ? lp.aug->x()(isb_col) : 0.0;
    MeasRow r;
    r.cls = MeasClass::Psr;
    r.sat = so.sat;
    r.el_rad = g.el_rad;
    r.az_rad = g.az_rad;
    r.cn0 = sig->cn0_dbhz;
    r.z = sig->psr_m - code_bias_m;                                                         // P_corr
    r.h = g.range + s.b - delta * s.d + iota - kCLight * g.state.dts + g.trop_m + g.ion_m;   // §4.3
    r.nu = r.z - r.h;
    r.H = Eigen::RowVectorXd::Zero(N);
    r.H = -u.transpose() * J.J_a;
    addColumn(r.H, HostIdx::dim + s.clk, 1.0);
    addColumn(r.H, HostIdx::dim + s.clk + 1, -delta);
    if (isb_col >= 0) addColumn(r.H, HostIdx::dim + isb_col, 1.0);
    NoiseModel::PsrTerms t;
    t.el_rad = g.el_rad;
    t.cn0 = sig->cn0_dbhz;
    t.psr_std_m = sig->psr_std_m;
    t.sys = so.sat.sys;
    t.bds_geo = QualityGate::isBdsGeo(so.sat);
    t.var_eph = g.state.var_eph;
    t.var_ion = g.var_ion;
    t.var_trop = g.var_trop;
    t.bridge_term = u.dot(MSDMt * u);
    t.clock_bridge = cfg_.process.clock.S_b * delta + cfg_.process.clock.S_d * delta * delta * delta / 3.0;
    const double uw = u.dot(J.w_N) * cfg_.time.sigma_toff_s;
    t.toff_term = uw * uw;
    t.dt_since_last_s = cfg_.noise.psr.decim_s > 0.0 ? cfg_.noise.psr.decim_s : 1.0;
    r.R = noise_.psrVar(t);
    out->rows.push_back(std::move(r));
  }
}

// ---------------------------------------------------------------------------------------------
// §4.4 Doppler rows (full rate; satellites on an open TDCP pair are excluded)
// ---------------------------------------------------------------------------------------------
void RowBuilder::dopplerRows(const ObsEpoch& epoch, const std::vector<SatGeometry>& geo, const std::vector<MaskVerdict>& masks,
                             const LinPoint& lp, const AntennaJacobians& J, const std::vector<int>& tdcp_excluded_sats,
                             RowSet* out) const {
  out->cls = MeasClass::Dopp;
  out->has_full_R = false;
  if (!cfg_.use.dopp) return;
  const SlotView s = readSlots(lp.aug);
  if (s.clk < 0 || !lp.anchor) return;
  const int N = static_cast<int>(J.J_a.cols());
  const Eigen::Matrix3d& REN = lp.anchor->R_EN;
  const Eigen::Vector3d r_A = lp.anchor->r0_ecef + REN * J.a_N;
  const Eigen::Vector3d rdot_A = REN * J.w_N;
  const CoreState& x = *lp.x;
  const double delta = lp.bridge ? lp.bridge->delta : lp.delta;
  const double omega_norm = lp.bridge ? (lp.bridge->omega_g + lp.bridge->bg_snap - x.bg).norm() : 0.0;
  // A61 F8: the antenna velocity takes ω̂_g, the mean of two raw gyro samples, through the lever arm — one white-noise
  // velocity error (cov var_ω·R_g[l]×[l]×ᵀR_gᵀ, W) shared by every satellite of the epoch. In N: Σ_ω,v = M(·)Mᵀ; each row
  // gets u_jᵀΣ_ω,v u_j (DoppTerms::var_omega_lever) and the pairs u_iᵀΣ_ω,v u_j (full R, like the TDCP bridge term).
  Eigen::Matrix3d Sov = Eigen::Matrix3d::Zero();
  if (lp.bridge && lp.bridge->var_omega > 0.0) {
    const Eigen::Matrix3d Rg = x.R * lp.bridge->dR.transpose();
    const Eigen::Matrix3d L = skew(lp.lever_I);
    const Eigen::Matrix3d MRL = J.M * Rg * L;
    Sov = lp.bridge->var_omega * MRL * MRL.transpose();
  }
  std::vector<Eigen::Vector3d> us;
  const size_t n = std::min(epoch.sats.size(), std::min(geo.size(), masks.size()));
  for (size_t i = 0; i < n; ++i) {
    if (!masks[i].dopp || !geo[i].state.ok) continue;
    const SatObs& so = epoch.sats[i];
    const SignalObs* sig = so.primary();
    if (!sig || !sig->doppValid()) continue;
    if (std::find(tdcp_excluded_sats.begin(), tdcp_excluded_sats.end(), so.sat.rtk_sat) != tdcp_excluded_sats.end()) continue;
    const SatGeometry& g = geo[i];
    if (!(g.lambda_m > 0.0)) continue;
    const Eigen::Vector3d& rs = g.state.r_ecef;
    const Eigen::Vector3d& vs = g.state.v_ecef;
    const Eigen::Vector3d u = REN.transpose() * g.e_ecef;   // anchor ENU, the frame of J_w (review m3)
    // Range rate = time derivative of the pseudorange geometry ρ = ‖r_s − r_A‖ + (ω_e/c)(x_s y_A − y_s x_A) (geodist, §1):
    //   ρ̇ = e·(v_s − ṙ_A) + (ω_e/c)(ẋ_s y_A + x_s ẏ_A − ẏ_s x_A − y_s ẋ_A).
    // The Sagnac rate carries the derivative's sign (AMENDMENTS A36). RTKLIB-EX resdop (pntpos.c:580-581) and the
    // original DESIGN §1 have its negative (≤ 5 mm/s per satellite, systematic); resdop is a reference only. U7/U16
    // check the sign against a numerical time derivative of ρ(t) along real broadcast orbits.
    const double rate = g.e_ecef.dot(vs - rdot_A) +
                        kOmgE / kCLight * (vs.x() * r_A.y() + rs.x() * rdot_A.y() - vs.y() * r_A.x() - rs.y() * rdot_A.x());
    MeasRow r;
    r.cls = MeasClass::Dopp;
    r.sat = so.sat;
    r.el_rad = g.el_rad;
    r.az_rad = g.az_rad;
    r.cn0 = sig->cn0_dbhz;
    r.z = -g.lambda_m * sig->dopp_hz;
    r.h = rate + s.d - kCLight * g.state.ddts;
    r.nu = r.z - r.h;
    r.H = Eigen::RowVectorXd::Zero(N);
    r.H = -u.transpose() * J.J_w;
    addColumn(r.H, HostIdx::dim + s.clk + 1, 1.0);
    // §4.7 as ruled in AMENDMENTS A31/A35-C6 (see NoiseModel::doppVar): no (δ(σ_ba + gσ_θ))² bridge term, so
    // DoppTerms::sigma_ba/sigma_theta/g stay at their defaults — that sensitivity reaches S through the δθ/δb_a/δg
    // columns of J_w. The bridge's white-noise velocity term IS added (AMENDMENTS A45 / C10): var_v_bridge = acc_n²δ
    // per axis (ImuBridge::query), isotropic, so u_jᵀM(var_v_bridge·I)Mᵀu_j = var_v_bridge for the unit u_j and the
    // rotation M; it is the IMU noise between t_g and t_k, which no state column carries (no double count).
    NoiseModel::DoppTerms t;
    t.el_rad = g.el_rad;
    t.lambda_m = g.lambda_m;
    t.dopp_std_hz = sig->dopp_std_hz;
    t.delta_s = delta;
    t.omega_norm = omega_norm;
    t.ant_acc_norm = J.alpha_N.norm();
    t.v_norm = J.w_N.norm();
    t.dt_since_last_s = 0.1;                    // §4.7 "κ_AR(τ_D, 0.1 s)"
    t.var_v_bridge = lp.bridge ? lp.bridge->var_v_bridge : 0.0;
    t.var_omega_lever = u.dot(Sov * u);
    r.R = noise_.doppVar(t);
    out->rows.push_back(std::move(r));
    us.push_back(u);
  }
  if (!Sov.isZero(0.0) && !out->rows.empty()) {   // A61 F8: the shared ω̂_g term off the diagonal
    const int mr = static_cast<int>(out->rows.size());
    out->R_full = Eigen::MatrixXd::Zero(mr, mr);
    for (int i = 0; i < mr; ++i)
      for (int k = 0; k < mr; ++k)
        out->R_full(i, k) = (i == k) ? out->rows[static_cast<size_t>(i)].R
                                     : us[static_cast<size_t>(i)].dot(Sov * us[static_cast<size_t>(k)]) * cfg_.noise.scale.dopp;
    out->has_full_R = true;
  }
}

// ---------------------------------------------------------------------------------------------
// §4.5 TDCP rows (one per satellite, Δclk still inside ν; projection is a separate step)
// ---------------------------------------------------------------------------------------------
void RowBuilder::tdcpRows(const TdcpPairInput& pair, const std::vector<SatGeometry>& geo_k1, const LinPoint& lp,
                          const AntennaJacobians& J, RowSet* out) const {
  out->cls = MeasClass::Tdcp;
  out->has_full_R = false;
  if (!cfg_.use.tdcp || !lp.anchor) return;
  const int N = static_cast<int>(J.J_a.cols());
  const Eigen::Matrix3d& REN = lp.anchor->R_EN;
  const Eigen::Vector3d r_c = lp.anchor->r0_ecef + REN * pair.c_hat_N;   // ECEF of the clone at this iterate
  const Eigen::Matrix3d MSDMt = lp.bridge ? Eigen::Matrix3d(J.M * lp.bridge->Sigma_D * J.M.transpose()) : Eigen::Matrix3d::Zero();
  const size_t m = pair.sats.size();
  std::vector<Eigen::Vector3d> u1s;
  u1s.reserve(m);
  for (size_t j = 0; j < m; ++j) {
    if (j >= geo_k1.size() || j >= pair.phi_k0_cyc.size() || j >= pair.phi_k1_cyc.size() || j >= pair.dts_k0.size() ||
        j >= pair.dts_k1.size() || j >= pair.rs_k0.size())
      break;
    const SatGeometry& g1 = geo_k1[j];
    if (!g1.state.ok || !(g1.lambda_m > 0.0)) continue;
    // R2-11: both ends use the ephemeris frozen at k0. When the caller hands over the snapshots, a k1 state that
    // was evaluated with another ephemeris (toe differs) is refused instead of mixed (review m4).
    if (j < pair.eph_snap.size() && pair.eph_snap[j].valid() && g1.state.toe != pair.eph_snap[j].toe()) continue;
    // k0 range recomputed at the current clone iterate ĉ_i (m3) with the frozen ephemeris position.
    const double rho0 = rangeWithSagnac(pair.rs_k0[j], r_c, nullptr);
    const Eigen::Vector3d u1 = REN.transpose() * g1.e_ecef;   // anchor ENU, the frame of J_a (review m3)
    MeasRow r;
    r.cls = MeasClass::Tdcp;
    r.sat = pair.sats[j];
    r.pair_id = pair.pair_id;
    r.el_rad = g1.el_rad;
    r.az_rad = g1.az_rad;
    r.z = g1.lambda_m * (pair.phi_k1_cyc[j] - pair.phi_k0_cyc[j]) - pair.clock_jump_m;
    r.h = (g1.range - rho0) - kCLight * (pair.dts_k1[j] - pair.dts_k0[j]);
    // A61 F2: the carrier's model delays change over the pair (+T − I; k1 at this iterate's antenna, k0 cached at k0)
    if (j < pair.trop_k0_m.size() && std::isfinite(pair.trop_k0_m[j])) r.h += g1.trop_m - pair.trop_k0_m[j];
    if (j < pair.ion_k0_m.size() && std::isfinite(pair.ion_k0_m[j])) r.h -= g1.ion_m - pair.ion_k0_m[j];
    // A61 F11: phase-noise states of the shared epochs (ε_k1 − ε_k0 of the carrier difference)
    const int c0 = j < pair.eta_k0_col.size() ? pair.eta_k0_col[j] : -1;
    const int c1 = j < pair.eta_k1_col.size() ? pair.eta_k1_col[j] : -1;
    const bool eta0 = c0 >= 0 && HostIdx::dim + c0 < N && lp.aug && c0 < lp.aug->dim();
    const bool eta1 = c1 >= 0 && HostIdx::dim + c1 < N && lp.aug && c1 < lp.aug->dim();
    if (eta0) r.h -= lp.aug->x()(c0);
    if (eta1) r.h += lp.aug->x()(c1);
    r.nu = r.z - r.h;                                        // = Δclk + H δx + ε
    r.H = Eigen::RowVectorXd::Zero(N);
    r.H = -u1.transpose() * J.J_a;
    // AMENDMENTS A64 (A61 ruling 5, A62 ③): the clone column takes the k1 line of sight too, so a row is linearised in
    // the displacement over the pair alone, −u1ᵀ(δa_k1 − δc). The exact k0 line of sight adds (u0 − u1)ᵀδc (u turns by
    // ~1e-4 per pair with the satellite), through which a chain of pairs builds float-PPP-like absolute position
    // information: the simulator overstates it by ~25 % (A61 ⑥), and on M3DGR the broadcast-orbit and atmosphere-rate
    // errors bias it (A62 ③: Grass01's global output pulled 1.5 m north, z² 4.6). h stays the exact ρ_k1(a) − ρ_k0(ĉ).
    if (pair.clone_slot >= 0 && HostIdx::dim + pair.clone_slot + 2 < N) {
      r.H.segment<3>(HostIdx::dim + pair.clone_slot) += u1.transpose();
    }
    if (eta0) r.H(HostIdx::dim + c0) -= 1.0;
    if (eta1) r.H(HostIdx::dim + c1) += 1.0;
    NoiseModel::TdcpTerms t;
    t.lambda_m = g1.lambda_m;
    t.el_k0_rad = j < pair.el_k0_rad.size() ? pair.el_k0_rad[j] : g1.el_rad;
    t.el_k1_rad = j < pair.el_k1_rad.size() ? pair.el_k1_rad[j] : g1.el_rad;
    t.cp_std_k0_m = j < pair.cp_std_k0_m.size() ? pair.cp_std_k0_m[j] : 0.0;
    t.cp_std_k1_m = j < pair.cp_std_k1_m.size() ? pair.cp_std_k1_m[j] : 0.0;
    t.bridge_term_k1 = u1.dot(MSDMt * u1);
    t.dt_pair_s = pair.dt_pair_s;
    t.dv_pair_norm = pair.dv_pair.norm();
    t.phase_k0_state = eta0;
    t.phase_k1_state = eta1;
    r.R = noise_.tdcpVar(t);
    out->rows.push_back(std::move(r));
    u1s.push_back(u1);
  }
  // The k1 bridge noise is common to every satellite of the epoch: R = diag + u_iᵀMΣ_DMᵀu_j (§4.8 "桥接满阵").
  const int mr = static_cast<int>(out->rows.size());
  if (mr > 0) {
    out->R_full = Eigen::MatrixXd::Zero(mr, mr);
    for (int i = 0; i < mr; ++i) {
      for (int j = 0; j < mr; ++j) {
        out->R_full(i, j) = (i == j) ? out->rows[i].R : u1s[i].dot(MSDMt * u1s[j]) * cfg_.noise.scale.tdcp;
      }
    }
    out->has_full_R = true;
  }
}

// ---------------------------------------------------------------------------------------------
// §4.5 Householder projection of the common Δclk
// ---------------------------------------------------------------------------------------------
bool RowBuilder::projectOutCommonClock(const RowSet& in, RowSet* out, double* robust_dclk) {
  std::vector<int> idx;
  for (int i = 0; i < static_cast<int>(in.rows.size()); ++i) {
    if (in.rows[i].reason == RejectReason::NotRejected) idx.push_back(i);
  }
  const int m = static_cast<int>(idx.size());
  if (robust_dclk) {
    // Weighted median (weights 1/R_i) of the raw innovations: the robust Δclk used by the per-satellite gate.
    *robust_dclk = 0.0;
    if (m > 0) {
      std::vector<std::pair<double, double>> vw;
      double wsum = 0.0;
      for (int i : idx) {
        const double w = in.rows[i].R > 0.0 ? 1.0 / in.rows[i].R : 1.0;
        vw.emplace_back(in.rows[i].nu, w);
        wsum += w;
      }
      std::sort(vw.begin(), vw.end(), [](const std::pair<double, double>& a, const std::pair<double, double>& b) { return a.first < b.first; });
      double acc = 0.0;
      for (const auto& p : vw) {
        acc += p.second;
        if (acc >= 0.5 * wsum) {
          *robust_dclk = p.first;
          break;
        }
      }
    }
  }
  if (!out) return m >= 2;
  out->cls = MeasClass::Tdcp;
  out->rows.clear();
  out->R_full.resize(0, 0);
  out->has_full_R = false;
  if (m < 2) return false;
  const int N = static_cast<int>(in.rows[idx[0]].H.size());
  Eigen::MatrixXd R(m, m);
  Eigen::MatrixXd H(m, N);
  Eigen::VectorXd nu(m);
  for (int a = 0; a < m; ++a) {
    nu(a) = in.rows[idx[a]].nu;
    H.row(a) = in.rows[idx[a]].H;
    for (int b = 0; b < m; ++b) {
      R(a, b) = in.has_full_R ? in.R_full(idx[a], idx[b]) : (a == b ? in.rows[idx[a]].R : 0.0);
    }
  }
  // Whitening W = L⁻¹ with R = LLᵀ (equals R^{−1/2} for diagonal R; a full R keeps unit variances too).
  Eigen::LLT<Eigen::MatrixXd> llt(R);
  if (llt.info() != Eigen::Success) return false;
  const Eigen::MatrixXd L = llt.matrixL();
  const Eigen::MatrixXd Hw = L.triangularView<Eigen::Lower>().solve(H);
  const Eigen::VectorXd nuw = L.triangularView<Eigen::Lower>().solve(nu);
  const Eigen::VectorXd o = L.triangularView<Eigen::Lower>().solve(Eigen::VectorXd::Ones(m));
  // Householder reflector Q (symmetric, orthogonal) with Qᵀo = ∓‖o‖e_1; rows 2..m of Qᵀ[·] are orthogonal
  // to o, i.e. exactly the part of the whitened measurements that carries no Δclk.
  Eigen::VectorXd v = o;
  const double alpha = (o(0) >= 0.0 ? 1.0 : -1.0) * o.norm();
  v(0) += alpha;
  const double vtv = v.squaredNorm();
  if (!(vtv > 0.0)) return false;
  const Eigen::MatrixXd Hp = Hw - v * ((2.0 / vtv) * (v.transpose() * Hw));
  const Eigen::VectorXd nup = nuw - v * ((2.0 / vtv) * v.dot(nuw));
  for (int a = 1; a < m; ++a) {
    MeasRow r;
    r.cls = MeasClass::Tdcp;
    r.pair_id = in.rows[idx[0]].pair_id;
    r.sat = SatId();                       // a projected row is a combination of satellites
    r.z = nup(a);
    r.h = 0.0;
    r.nu = nup(a);
    r.H = Hp.row(a);
    r.R = 1.0;
    r.accepted = true;
    out->rows.push_back(std::move(r));
  }
  return true;
}

// ---------------------------------------------------------------------------------------------
// §4.6 loose-coupling rows (L-PVT arm)
// ---------------------------------------------------------------------------------------------
void RowBuilder::pvtRows(const ReceiverPvt& pvt, const LinPoint& lp, const AntennaJacobians& J, double dt_since_last_s,
                         RowSet* pos_out, RowSet* vel_out) const {
  if (pos_out) { pos_out->cls = MeasClass::PvtPos; pos_out->has_full_R = false; }
  if (vel_out) { vel_out->cls = MeasClass::PvtVel; vel_out->has_full_R = false; }
  // A43: the pseudorange rule for the receiver's solutions (bins of noise.pvt.decim_s over ReceiverPvt::t) — the first
  // solution to arrive in its GPST second, fix or not, decides that second; its relinearisations repeat the decision.
  if (!pvt_decim_.admit(pvt.t)) return;
  if (!lp.anchor || !pvt.valid_fix || (pvt.fix_type != 3 && pvt.fix_type != 4)) return;
  const int N = static_cast<int>(J.J_a.cols());
  const double deg = M_PI / 180.0;
  if (pos_out) {
    Geodetic g;
    g.lat_rad = pvt.lat_deg * deg;
    g.lon_rad = pvt.lon_deg * deg;
    g.h_m = pvt.h_ell_m;                                                    // ellipsoidal height (§4.6)
    const Eigen::Vector3d r_pvt = rtk::pos2ecef(g);
    const Eigen::Vector3d z_p = lp.anchor->R_EN.transpose() * (r_pvt - lp.anchor->r0_ecef);
    const Eigen::Vector3d var = noise_.pvtPosVar(pvt.h_acc_m, pvt.v_acc_m, dt_since_last_s);
    for (int k = 0; k < 3; ++k) {
      MeasRow r;
      r.cls = MeasClass::PvtPos;
      r.z = z_p(k);
      r.h = J.a_N(k);
      r.nu = r.z - r.h;
      r.H = Eigen::RowVectorXd::Zero(N);
      r.H = J.J_a.row(k);
      r.R = var(k);
      pos_out->rows.push_back(std::move(r));
    }
  }
  if (vel_out && cfg_.use.pvt_vel) {
    const Eigen::Vector3d z_v(pvt.vel_ned(1), pvt.vel_ned(0), -pvt.vel_ned(2));   // NED → ENU
    const Eigen::Vector3d var = noise_.pvtVelVar(pvt.vel_acc_mps, dt_since_last_s);
    for (int k = 0; k < 3; ++k) {
      MeasRow r;
      r.cls = MeasClass::PvtVel;
      r.z = z_v(k);
      r.h = J.w_N(k);
      r.nu = r.z - r.h;
      r.H = Eigen::RowVectorXd::Zero(N);
      r.H = J.J_w.row(k);
      r.R = var(k);
      vel_out->rows.push_back(std::move(r));
    }
  }
}

// ---------------------------------------------------------------------------------------------
// §2.4 clone creation: c = a^N(x̂_k0) ⇒ F_host = ∂a^N/∂δx_host, F_aug = ∂a^N/∂δx_G, N = M Σ_D,k0 Mᵀ
// ---------------------------------------------------------------------------------------------
void RowBuilder::cloneJacobians(const AntennaJacobians& J, const Eigen::Matrix3d& Sigma_D_k0, Eigen::MatrixXd* F_host,
                                Eigen::MatrixXd* F_aug, Eigen::MatrixXd* N) {
  const int n = static_cast<int>(J.J_a.cols()) - HostIdx::dim;
  if (F_host) *F_host = J.J_a.leftCols(HostIdx::dim);
  if (F_aug) *F_aug = (n > 0) ? Eigen::MatrixXd(J.J_a.rightCols(n)) : Eigen::MatrixXd(3, 0);
  if (N) *N = J.M * Sigma_D_k0 * J.M.transpose();
}

}  // namespace livo_gnss
