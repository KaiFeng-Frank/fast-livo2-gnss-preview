// SPDX-License-Identifier: BSD-3-Clause
// quality (DESIGN §10 L0, §4.7, §12.6-3). Owner: WP-c. Tests: U5 (units), U12 (masks).
//
// Everything here is either a 0/1 membership decision (masks) or a variance (noise model). No gate in
// this file ever scales a residual: continuous down-weighting is excluded by DESIGN §0-4.
#include "internal/quality.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <set>

#include "livo_gnss/signal_table.hpp"
#include "internal/rtk_adapter.hpp"   // rtk::svhHealthy (the one health rule, AMENDMENTS A37); no rtklib.h

namespace livo_gnss {

namespace {

constexpr double kCLight = 299792458.0;
constexpr double kDeg2Rad = M_PI / 180.0;

// Parse a RINEX-style id ("G07", "C59", also "G7") into (sys, prn). Local so the mask never depends on
// rtk_adapter (WP-a) and works on synthetic SatIds.
bool parseSatIdString(const std::string& s, Sys* sys, int* prn) {
  if (s.size() < 2) return false;
  const char c = static_cast<char>(std::toupper(static_cast<unsigned char>(s[0])));
  Sys sy = Sys::Unknown;
  switch (c) {
    case 'G': sy = Sys::GPS; break;
    case 'R': sy = Sys::GLO; break;
    case 'E': sy = Sys::GAL; break;
    case 'J': sy = Sys::QZS; break;
    case 'C': sy = Sys::BDS; break;
    case 'I': sy = Sys::IRN; break;
    case 'S': sy = Sys::SBS; break;
    default: return false;
  }
  char* end = nullptr;
  const long v = std::strtol(s.c_str() + 1, &end, 10);
  if (end == s.c_str() + 1 || *end != '\0' || v <= 0) return false;
  *sys = sy;
  *prn = static_cast<int>(v);
  return true;
}

inline bool isNaNOrInf(double v) { return !std::isfinite(v); }

// Grid tests of the ingest self-check 3 (§12.6): value = base·2ⁿ or value = base·n within a relative
// tolerance that separates neighbouring grid points (1e-3 of the step).
bool onPow2Grid(double v, double base) {
  if (!(v > 0.0) || !(base > 0.0)) return false;
  const double n = std::log2(v / base);
  const double r = std::round(n);
  return std::fabs(n - r) < 1e-3 && r >= -1.0 && r <= 40.0;
}
bool onLinearGrid(double v, double step) {
  if (!(v > 0.0) || !(step > 0.0)) return false;
  const double n = v / step;
  const double r = std::round(n);
  return r >= 1.0 && std::fabs(n - r) < 1e-3;
}

}  // namespace

// ---------------------------------------------------------------------------------------------
// TrackCounter
// ---------------------------------------------------------------------------------------------
TrackCounter::TrackCounter(const Config::Masks& m) : masks_(m) {}

void TrackCounter::tick(const ObsEpoch& epoch) {
  // Epoch-stream hole: "consecutive" means consecutive receiver epochs, and epochs the host never received (A3's
  // 0.5 s gap where every topic is missing) must not be bridged by the counters. Nominal interval = running median.
  if (t_prev_.valid() && epoch.t_rx.valid()) {
    const double dt = epoch.t_rx.diff(t_prev_);
    if (dt > 0.0) {
      if (intervals_.size() >= 3) {
        std::vector<double> s = intervals_;
        std::nth_element(s.begin(), s.begin() + s.size() / 2, s.end());
        const double nominal = s[s.size() / 2];
        if (nominal > 0.0 && dt > kGapFactor * nominal) {
          psr_.clear();
          cp_.clear();
        }
      }
      intervals_.push_back(dt);
      if (intervals_.size() > 20) intervals_.erase(intervals_.begin());
    }
  }
  if (epoch.t_rx.valid()) t_prev_ = epoch.t_rx;
  std::set<int> present;
  for (const auto& so : epoch.sats) {
    if (!so.sat.valid()) continue;
    const int key = so.sat.rtk_sat;
    present.insert(key);
    const SignalObs* sig = so.primary();
    // Pseudorange arc: consecutive epochs with a valid code measurement.
    if (sig && sig->psrValid() && std::isfinite(sig->psr_m)) {
      psr_[key] += 1;
    } else {
      psr_[key] = 0;
    }
    // Carrier arc: consecutive epochs with valid carrier, resolved half cycle and no loss of lock.
    // An LLI bit (loss / half-cycle slip) starts a new arc at this epoch (count 1), it does not keep the old one.
    if (sig && sig->cpValid() && std::isfinite(sig->cp_cyc) && (sig->status & kStatusHalfCycValid)) {
      if (sig->lli & 0x3) {
        cp_[key] = 1;
      } else {
        cp_[key] += 1;
      }
    } else {
      cp_[key] = 0;
    }
    last_seen_[key] = epoch.t_rx;
  }
  // Absence resets both arcs (no locktime field in gnss_comm, DESIGN §10 L0).
  for (auto it = psr_.begin(); it != psr_.end();) {
    if (!present.count(it->first)) it = psr_.erase(it); else ++it;
  }
  for (auto it = cp_.begin(); it != cp_.end();) {
    if (!present.count(it->first)) it = cp_.erase(it); else ++it;
  }
}

int TrackCounter::psrCount(const SatId& sat) const {
  auto it = psr_.find(sat.rtk_sat);
  return it == psr_.end() ? 0 : it->second;
}
int TrackCounter::cpCount(const SatId& sat) const {
  auto it = cp_.find(sat.rtk_sat);
  return it == cp_.end() ? 0 : it->second;
}
bool TrackCounter::psrOk(const SatId& sat) const { return psrCount(sat) >= masks_.track_psr; }
bool TrackCounter::cpOk(const SatId& sat) const { return cpCount(sat) >= masks_.track_cp; }
void TrackCounter::reset() {
  psr_.clear();
  cp_.clear();
  last_seen_.clear();
  t_prev_ = GTime();
  intervals_.clear();
}

// ---------------------------------------------------------------------------------------------
// QualityGate — L0 masks. Each check sets exactly one bit for one defect so U12 can verify that every
// MaskBit fires on its own synthetic trigger and nothing else.
// ---------------------------------------------------------------------------------------------
QualityGate::QualityGate(const Config& cfg) : cfg_(cfg) {}

bool QualityGate::isBdsGeo(const SatId& sat) {
  return sat.sys == Sys::BDS && (sat.prn <= 5 || sat.prn >= 59);   // ephemeris.c:265 GEO branch
}

bool QualityGate::systemEnabled(Sys sys) const {
  switch (sys) {
    case Sys::GPS: return cfg_.systems.gps;
    case Sys::GLO: return cfg_.systems.glo;
    case Sys::GAL: return cfg_.systems.gal;
    case Sys::BDS: return cfg_.systems.bds;
    case Sys::QZS: return cfg_.systems.qzs_as_gps && cfg_.systems.gps;   // QZSS rides on the GPS clock (§2.2)
    default: return false;                                               // SBS / IRN / Unknown never used
  }
}

bool QualityGate::excluded(const SatId& sat) const {
  for (const auto& s : cfg_.exclude_sats) {
    Sys sys;
    int prn;
    if (parseSatIdString(s, &sys, &prn) && sys == sat.sys && prn == sat.prn) return true;
  }
  return false;
}

MaskVerdict QualityGate::evaluate(const MaskInput& in) const {
  MaskVerdict v;
  const SatObs* obs = in.obs;
  if (!obs) return v;
  const SignalObs* sig = obs->primary();
  if (!sig) {
    v.reasons |= kMaskCodeUnknown;
    return v;
  }
  // Every threshold below is the configured one (AMENDMENTS A44: the C/N0 masks are receiver-dependent; M3DGR runs with
  // placeholders 15/12/18 dB-Hz until the P4 calibration, DESIGN §14's 30/25/28 only as the Config defaults).
  const Config::Masks& m = cfg_.masks;
  const SatId& sat = obs->sat;

  // --- satellite-level defects: kill every row ---
  uint32_t sat_reasons = kMaskNone;
  if (isNaNOrInf(sig->psr_m) || isNaNOrInf(sig->cp_cyc) || isNaNOrInf(sig->dopp_hz) || isNaNOrInf(sig->cn0_dbhz) ||
      isNaNOrInf(sig->freq_hz)) {
    sat_reasons |= kMaskNaN;
  }
  // freqs → CODE_* table (single copy, signal_table.hpp). BDS 1575.42 gets its own counter (§12.3).
  {
    SignalVerdict sv = SignalVerdict::Accepted;
    const SignalCode code = signalFromFrequency(sat.sys, sig->freq_hz, nullptr, &sv);
    if (sv == SignalVerdict::RejectedBdsB1C) {
      sat_reasons |= kMaskBdsB1C;
    } else if (code == SignalCode::Unknown || sig->code == SignalCode::Unknown || isGeometryFreeOnly(sig->code)) {
      sat_reasons |= kMaskCodeUnknown;
    }
  }
  if (!systemEnabled(sat.sys) || (isBdsGeo(sat) && !cfg_.systems.bds_geo.use)) sat_reasons |= kMaskSystemOff;
  if (excluded(sat)) sat_reasons |= kMaskExcluded;
  if (sat.sys == Sys::GAL && (sat.prn == 37 || sat.prn == 38)) sat_reasons |= kMaskGalPrn;
  if (in.el_deg < m.el_deg) sat_reasons |= kMaskElevation;
  if (!in.has_eph) {
    sat_reasons |= kMaskNoEph;
  } else {
    // Broadcast health as RTKLIB satexclude() reads it (AMENDMENTS A37): QZSS ignores bit 0 (set on every QZSS record
    // of the IGS BRDC), GLONASS uses the Bn/RINEX-3.05 flag layout, every other system needs svh = 0. A literal
    // svh ≠ 0 would mask all QZSS and the healthy RINEX ≥ 3.05 GLONASS satellites.
    if (!rtk::svhHealthy(sat.sys, in.svh)) sat_reasons |= kMaskHealth;
    // QZSS uses its own key 'J' (A15); a missing key is treated as "no limit known" → masked. Age is |t − toe|
    // (ephHealthy); a signed age from another caller is folded rather than masked (review n2).
    const auto it = m.eph_age_s.find(sysChar(sat.sys));
    if (it == m.eph_age_s.end() || std::fabs(in.eph_age_s) > it->second) sat_reasons |= kMaskEphAge;
  }
  v.reasons |= sat_reasons;
  if (sat_reasons != kMaskNone) return v;   // psr/dopp/cp all false

  // --- pseudorange row ---
  bool psr = true;
  if (sat.sys == Sys::GLO && !cfg_.systems.glo_psr) {
    v.reasons |= kMaskSystemOff;              // GLONASS code off by default (IFB, R2-5); Doppler/TDCP stay
    psr = false;
  }
  if (!sig->psrValid()) {
    v.reasons |= kMaskStatusPsr;
    psr = false;
  } else if (sig->psr_m < m.psr_min_m || sig->psr_m > m.psr_max_m) {
    v.reasons |= kMaskPsrRange;
    psr = false;
  }
  if (sig->cn0_dbhz < m.cn0_psr) { v.reasons |= kMaskCn0Psr; psr = false; }
  if (sig->psr_std_m > m.max_psr_std_m) { v.reasons |= kMaskStdPsr; psr = false; }
  if (!in.psr_track_ok) { v.reasons |= kMaskTrackPsr; psr = false; }
  v.psr = psr;

  // --- Doppler row ---
  bool dopp = true;
  const double lambda = sig->freq_hz > 0.0 ? kCLight / sig->freq_hz : 0.0;
  if (!sig->doppValid() || lambda <= 0.0) {
    v.reasons |= kMaskRangeRate;              // no usable range rate at all
    dopp = false;
  } else if (std::fabs(lambda * sig->dopp_hz) >= m.max_range_rate_mps) {
    v.reasons |= kMaskRangeRate;
    dopp = false;
  }
  if (sig->cn0_dbhz < m.cn0_dopp) { v.reasons |= kMaskCn0Dopp; dopp = false; }
  if (sig->dopp_std_hz > m.max_dopp_std_hz) { v.reasons |= kMaskStdDopp; dopp = false; }
  v.dopp = dopp;

  // --- carrier (TDCP) row ---
  bool cp = true;
  if (!sig->cpValid()) { v.reasons |= kMaskStatusCp; cp = false; }
  if (!(sig->status & kStatusHalfCycValid)) { v.reasons |= kMaskHalfCycle; cp = false; }
  if (sig->cn0_dbhz < m.cn0_cp) { v.reasons |= kMaskCn0Cp; cp = false; }
  if (sig->cp_std_m > m.max_cp_std_m) { v.reasons |= kMaskStdCp; cp = false; }
  if (!in.cp_track_ok) { v.reasons |= kMaskTrackCp; cp = false; }
  v.cp = cp;
  return v;
}

// ---------------------------------------------------------------------------------------------
// NoiseModel — DESIGN §4.7 (TDCP dimension per AMENDMENTS A14)
// ---------------------------------------------------------------------------------------------
NoiseModel::NoiseModel(const Config& c) : cfg(c) {}

double NoiseModel::kappaAR(double tau_s, double dt_s) {
  // Variance inflation of the sample mean of an AR(1) process relative to white noise:
  // κ = (1+ρ)/(1−ρ), ρ = exp(−Δt/τ). τ ≤ 0 or Δt → ∞ means white (κ = 1); Δt ≤ 0 is meaningless → κ = 1.
  if (!(tau_s > 0.0) || !(dt_s > 0.0)) return 1.0;
  const double rho = std::exp(-dt_s / tau_s);
  if (rho >= 1.0) return 1.0;
  return (1.0 + rho) / (1.0 - rho);
}

double NoiseModel::psrVar(const PsrTerms& t) const {
  const Config::Noise::Psr& p = cfg.noise.psr;
  // s_sys: G/E/C(/J) = 1, R = 1.5 (config), C GEO = psr_scale (§4.7 "C_GEO=2").
  double s_sys = 1.0;
  const auto it = p.sys_factor.find(sysChar(t.sys));
  if (it != p.sys_factor.end()) s_sys = it->second;
  if (t.bds_geo) s_sys = cfg.systems.bds_geo.psr_scale;
  const double sin_el = std::max(std::sin(t.el_rad), std::sin(5.0 * kDeg2Rad));   // MIN_EL guard as RTKLIB varerr
  const double el_term = 1.0 + 1.0 / (sin_el * sin_el);
  const double cn0_term = std::pow(10.0, std::max(0.0, p.cn0_ref - t.cn0) / 10.0);
  // receiver term: σ_P0²(1+1/sin²el)·10^{max(0,45−CN0)/10} + (k_std·psr_std)², then κ_AR for the multipath
  // time correlation (the κ applies to the receiver noise only; ephemeris/atmosphere/bridge are added after).
  const double rcv = p.sigma0_m * p.sigma0_m * el_term * cn0_term + (p.k_std * t.psr_std_m) * (p.k_std * t.psr_std_m);
  const double kappa = kappaAR(p.tau_s, t.dt_since_last_s);
  double var = s_sys * s_sys * rcv * kappa;
  var += t.var_eph + t.var_ion + t.var_trop;          // σ_eph² + var_ion + var_trop (§4.3)
  var += t.bridge_term;                               // u_jᵀ M Σ_D Mᵀ u_j
  var += t.clock_bridge;                              // S_b δ + S_d δ³/3 (clock state is at t_k, epoch at t_k − δ)
  var += t.toff_term;                                 // (u_jᵀ w^N σ_toff)²
  return var * cfg.noise.scale.psr;
}

double NoiseModel::doppVar(const DoppTerms& t) const {
  if (flat.on) return flat.sigma_dopp_mps * flat.sigma_dopp_mps + t.var_v_bridge + t.var_omega_lever + cfg.process.clock.S_d * t.delta_s;
  const Config::Noise::Dopp& d = cfg.noise.dopp;
  const double sin_el = std::max(std::sin(t.el_rad), std::sin(5.0 * kDeg2Rad));
  // Receiver term (AMENDMENTS A35-C6): σ_D² = max(λ·dopp_std, σ_D0)²·(1 + 1/sin²el)/2 — the elevation enters as a
  // variance factor, like the pseudorange's, normalised to 1 at the zenith (DESIGN §4.7 squared the factor: a typo
  // that made σ_D ∝ 1/sin²el). Time-correlated ⇒ κ_AR(τ_D, Δt) multiplies this term only (A35-C5).
  const double s0 = std::max(t.lambda_m * t.dopp_std_hz, d.sigma0_mps);
  const double el_factor = 0.5 * (1.0 + 1.0 / (sin_el * sin_el));
  const double kappa = kappaAR(d.tau_s, t.dt_since_last_s);
  double var = s0 * s0 * el_factor * kappa;
  // No bridge (δ(σ_ba + gσ_θ))² term (AMENDMENTS A31): that sensitivity of the bridged velocity is carried by the
  // δθ/δb_a/δg columns of J_w through HΣHᵀ; adding it here would count it twice. DoppTerms::sigma_ba/sigma_theta/g
  // are therefore not read. The bridge's IMU white-noise velocity term acc_n²δ is added (AMENDMENTS A45 / C10): no state
  // column carries the accelerometer noise between t_g and t_k, so it is not in HΣHᵀ (the Doppler twin of the
  // u_jᵀMΣ_DMᵀu_j term of the pseudorange/TDCP rows). Remaining §4.7 terms: clock drift over the bridge S_d δ; lever
  // arm (‖ω‖σ_l)²; time offset (‖α_A‖σ_toff)²; registration drift rate (0.01‖v‖)².
  var += t.var_v_bridge;
  var += t.var_omega_lever;                                // A61 F8: gyro noise of ω̂_g through the lever arm
  var += cfg.process.clock.S_d * t.delta_s;
  const double vl = t.omega_norm * cfg.frames.lever_arm_sigma;
  var += vl * vl;
  const double vt = t.ant_acc_norm * cfg.time.sigma_toff_s;
  var += vt * vt;
  const double vv = 0.01 * t.v_norm;
  var += vv * vv;
  return var * cfg.noise.scale.dopp;
}

double NoiseModel::doppMarginalVar(double lambda_m, double dopp_std_hz, double el_rad) const {
  const Config::Noise::Dopp& d = cfg.noise.dopp;
  const double sin_el = std::max(std::sin(el_rad), std::sin(5.0 * kDeg2Rad));
  const double s0 = std::max(lambda_m * dopp_std_hz, d.sigma0_mps);
  return s0 * s0 * 0.5 * (1.0 + 1.0 / (sin_el * sin_el)) * cfg.noise.scale.dopp;
}

double NoiseModel::sigmaPhase(double cp_std_m, double el_rad) const {
  // σ_Φ = max(cp_std_m, σ_Φ0 / sin el), all in metres (A14).
  const double sin_el = std::max(std::sin(el_rad), std::sin(5.0 * kDeg2Rad));
  return std::max(cp_std_m, cfg.noise.tdcp.sigma0_m / sin_el);
}

double NoiseModel::tdcpVar(const TdcpTerms& t) const {
  if (flat.on) return flat.sigma_tdcp_m * flat.sigma_tdcp_m + t.bridge_term_k1;
  const Config::Noise::Tdcp& c = cfg.noise.tdcp;
  const double s0 = sigmaPhase(t.cp_std_k0_m, t.el_k0_rad);
  const double s1 = sigmaPhase(t.cp_std_k1_m, t.el_k1_rad);
  // A14: phase term σ²_Φ(k0) + σ²_Φ(k1) with NO λ² (σ_Φ already in metres). A61 F11: an end whose phase noise is a
  // state contributes through its column instead.
  double var = (t.phase_k0_state ? 0.0 : s0 * s0) + (t.phase_k1_state ? 0.0 : s1 * s1);
  var += t.bridge_term_k1;                                  // u_jᵀ M Σ_{D,k1} Mᵀ u_j (k0 term lives in Σ_cc)
  const double vr = c.sigma_rate_mps * t.dt_pair_s;         // ionosphere/orbit residual rate
  var += vr * vr;
  var += c.sigma_windup_m * c.sigma_windup_m;               // non-common phase wind-up
  const double vt = t.dv_pair_norm * cfg.time.sigma_toff_s; // (‖Δv_pair‖ σ_toff)² (R1-6)
  var += vt * vt;
  if (t.phase_k0_state || t.phase_k1_state) var = std::max(var, kTdcpVarFloorFrac * (s0 * s0 + s1 * s1));
  return var * cfg.noise.scale.tdcp;
}

Eigen::Vector3d NoiseModel::pvtPosVar(double h_acc_m, double v_acc_m, double dt_since_last_s) const {
  // §4.6: R = diag(h_acc², h_acc², v_acc²)·κ_AR(τ_pvt) (× pvt.scale).
  const double k = kappaAR(cfg.noise.pvt.tau_pos_s, dt_since_last_s) * cfg.noise.pvt.scale;
  return Eigen::Vector3d(h_acc_m * h_acc_m * k, h_acc_m * h_acc_m * k, v_acc_m * v_acc_m * k);
}

Eigen::Vector3d NoiseModel::pvtVelVar(double vel_acc_mps, double dt_since_last_s) const {
  // §4.6: velocity (vel_acc)²·2, time-correlated with τ_vel (× pvt.scale).
  const double k = kappaAR(cfg.noise.pvt.tau_vel_s, dt_since_last_s) * cfg.noise.pvt.scale;
  const double v = vel_acc_mps * vel_acc_mps * 2.0 * k;
  return Eigen::Vector3d(v, v, v);
}

// ---------------------------------------------------------------------------------------------
// StdUnitJudge — ingest self-check 3 (§12.6): the receiver std fields of gnss_comm come either as the
// u-blox RXM-RAWX indices (prStdev/cpStdev/doStdev: 0.01·2ⁿ m, 0.004·n cycles, 0.002·2ⁿ Hz) already
// converted by the driver, or as the raw indices. The decision is made by the grid the values fall on.
// ---------------------------------------------------------------------------------------------
namespace {
constexpr int kJudgeMinSamples = 50;
constexpr double kJudgeMajority = 0.9;
bool looksLikeIndex(double v) { return v >= 0.0 && v <= 15.0 && std::fabs(v - std::round(v)) < 1e-9; }
}  // namespace

void StdUnitJudge::observe(const ObsEpoch& epoch) {
  for (const auto& so : epoch.sats) {
    const SignalObs* sig = so.primary();
    if (!sig) continue;
    const double lambda = sig->freq_hz > 0.0 ? kCLight / sig->freq_hz : 0.0;
    ++n_;
    // psr: {0.01·2ⁿ} m vs raw index 0..15
    if (sig->psr_std_raw > 0.0) {
      if (onPow2Grid(sig->psr_std_raw, 0.01)) psr_hist_[0] += 1;
      if (looksLikeIndex(sig->psr_std_raw)) psr_hist_[1] += 1;
      psr_hist_[2] += 1;   // total
    }
    // cp: cycles grid {0.004·n} vs metres grid {0.004·n·λ} vs raw index
    if (sig->cp_std_raw > 0.0) {
      if (onLinearGrid(sig->cp_std_raw, 0.004)) cp_hist_cyc_[0] += 1;
      if (lambda > 0.0 && onLinearGrid(sig->cp_std_raw, 0.004 * lambda)) cp_hist_m_[0] += 1;
      if (looksLikeIndex(sig->cp_std_raw)) cp_hist_cyc_[1] += 1;
      cp_hist_cyc_[2] += 1;   // total
    }
    // dopp: {0.002·2ⁿ} Hz vs raw index
    if (sig->dopp_std_raw > 0.0) {
      if (onPow2Grid(sig->dopp_std_raw, 0.002)) dopp_hist_[0] += 1;
      if (looksLikeIndex(sig->dopp_std_raw)) dopp_hist_[1] += 1;
      dopp_hist_[2] += 1;
    }
  }
  auto frac = [](const std::map<int, int>& h, int key) {
    const auto t = h.find(2);
    const auto k = h.find(key);
    if (t == h.end() || t->second < kJudgeMinSamples || k == h.end()) return 0.0;
    return static_cast<double>(k->second) / static_cast<double>(t->second);
  };
  if (psr_ == StdUnit::Unknown) {
    // An integer index 0..15 is also on the 0.01·2ⁿ grid only for n=0 (0.01·1 ≠ integer) — the two
    // hypotheses are disjoint except v=1 (n≈6.64, not on grid), so majority decides.
    if (frac(psr_hist_, 1) >= kJudgeMajority) psr_ = StdUnit::Index;
    else if (frac(psr_hist_, 0) >= kJudgeMajority) psr_ = StdUnit::Metres;
  }
  if (cp_ == StdUnit::Unknown) {
    const auto tot = cp_hist_cyc_.find(2);
    if (tot != cp_hist_cyc_.end() && tot->second >= kJudgeMinSamples) {
      const double f_cyc = static_cast<double>(cp_hist_cyc_[0]) / tot->second;
      const double f_m = static_cast<double>(cp_hist_m_[0]) / tot->second;
      const double f_idx = static_cast<double>(cp_hist_cyc_[1]) / tot->second;
      if (f_idx >= kJudgeMajority) cp_ = StdUnit::Index;
      else if (f_m >= kJudgeMajority && f_m > f_cyc) cp_ = StdUnit::Metres;
      else if (f_cyc >= kJudgeMajority) cp_ = StdUnit::Cycles;
    }
  }
  if (dopp_ == StdUnit::Unknown) {
    if (frac(dopp_hist_, 1) >= kJudgeMajority) dopp_ = StdUnit::Index;
    else if (frac(dopp_hist_, 0) >= kJudgeMajority) dopp_ = StdUnit::Hertz;
  }
}

bool StdUnitJudge::decided() const { return psr_ != StdUnit::Unknown && cp_ != StdUnit::Unknown && dopp_ != StdUnit::Unknown; }
void StdUnitJudge::set(StdUnit psr, StdUnit cp, StdUnit dopp) { psr_ = psr; cp_ = cp; dopp_ = dopp; }

void StdUnitJudge::apply(ObsEpoch* epoch) const {
  if (!epoch) return;
  for (auto& so : epoch->sats) {
    for (auto& sig : so.sig) {
      const double lambda = sig.freq_hz > 0.0 ? kCLight / sig.freq_hz : 0.0;
      sig.psr_std_unit = psr_;
      sig.cp_std_unit = cp_;
      sig.dopp_std_unit = dopp_;
      switch (psr_) {
        case StdUnit::Metres: sig.psr_std_m = sig.psr_std_raw; break;
        case StdUnit::Index: sig.psr_std_m = sig.psr_std_raw > 0.0 ? 0.01 * std::pow(2.0, sig.psr_std_raw) : 0.0; break;
        default: sig.psr_std_m = 0.0; break;   // unknown → noise-model floor applies
      }
      switch (cp_) {
        case StdUnit::Metres: sig.cp_std_m = sig.cp_std_raw; break;
        case StdUnit::Cycles: sig.cp_std_m = sig.cp_std_raw * lambda; break;   // λ only appears here (A14)
        case StdUnit::Index: sig.cp_std_m = 0.004 * sig.cp_std_raw * lambda; break;
        default: sig.cp_std_m = 0.0; break;
      }
      switch (dopp_) {
        case StdUnit::Hertz: sig.dopp_std_hz = sig.dopp_std_raw; break;
        case StdUnit::Index: sig.dopp_std_hz = sig.dopp_std_raw > 0.0 ? 0.002 * std::pow(2.0, sig.dopp_std_raw) : 0.0; break;
        default: sig.dopp_std_hz = 0.0; break;
      }
    }
  }
}

}  // namespace livo_gnss
