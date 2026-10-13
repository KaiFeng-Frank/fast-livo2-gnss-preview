// SPDX-License-Identifier: BSD-3-Clause
// quality — L0 signal masks (DESIGN §10 L0), measurement noise model (§4.7) with the AR(1) sample-mean
// factor κ_AR, and the receiver-std unit judgement of the ingest self-check (§12.6-3).
// Everything here is 0/1 membership or a variance; no continuous down-weighting (§0-4).
// Owner: WP-c. Tests: test_u07_jacobians.cpp (noise partials not needed), test_u12_gating.cpp (masks),
// test_u05_corrections.cpp (unit judgement against the fixture).
#pragma once

#include <array>
#include <cstdint>
#include <map>
#include <vector>

#include <Eigen/Core>

#include "livo_gnss/config.hpp"
#include "livo_gnss/types.hpp"

namespace livo_gnss {

// Reasons a signal fails L0 (bit mask; several may apply).
enum MaskBit : uint32_t {
  kMaskNone = 0,
  kMaskNaN = 1u << 0,
  kMaskPsrRange = 1u << 1,       // psr ∉ [1.9e7, 4.5e7] m
  kMaskRangeRate = 1u << 2,      // |λD| ≥ 2 km/s
  kMaskElevation = 1u << 3,      // el < el_deg
  kMaskCn0Psr = 1u << 4,
  kMaskCn0Dopp = 1u << 5,
  kMaskCn0Cp = 1u << 6,
  kMaskStatusPsr = 1u << 7,      // status bit0
  kMaskStatusCp = 1u << 8,       // bit1
  kMaskHalfCycle = 1u << 9,      // bit2
  kMaskHealth = 1u << 10,        // !rtk::svhHealthy(sys, svh) (RTKLIB satexclude reading, AMENDMENTS A37)
  kMaskEphAge = 1u << 11,
  kMaskTrackPsr = 1u << 12,      // < track_psr consecutive epochs
  kMaskTrackCp = 1u << 13,
  kMaskExcluded = 1u << 14,      // exclude_sats
  kMaskSystemOff = 1u << 15,     // system disabled / GLO psr off
  kMaskBdsB1C = 1u << 16,        // BDS 1575.42 rejected
  kMaskGalPrn = 1u << 17,        // E37/E38
  kMaskStdPsr = 1u << 18,        // psr_std > max
  kMaskStdDopp = 1u << 19,
  kMaskStdCp = 1u << 20,
  kMaskNoEph = 1u << 21,
  kMaskCodeUnknown = 1u << 22,   // freqs → CODE_* rejected
};

struct MaskInput {
  const SatObs* obs = nullptr;   // primary signal is obs->sig[0]
  double el_deg = 0.0, az_deg = 0.0;
  bool has_eph = false;
  // Raw broadcast svh of the selected record (rtk::SatState::svh, unprocessed). The gate applies the system rules
  // itself through rtk::svhHealthy() (AMENDMENTS A37: QZSS bit 0 ignored, GLONASS Bn/RINEX-3.05 layout, others 0);
  // callers must NOT pre-filter it. Negative = no ephemeris (unhealthy).
  int svh = 0;
  // Ephemeris age |t − toe| as rtk::SatSolver::ephHealthy returns it; a signed value is folded by |·| (review n2).
  double eph_age_s = 0.0;
  bool psr_track_ok = false, cp_track_ok = false;   // from TrackCounter
  bool is_bds_geo = false;
};

struct MaskVerdict {
  bool psr = false, dopp = false, cp = false;       // usable for the respective row
  uint32_t reasons = kMaskNone;
};

// Consecutive-epoch tracking counters (no locktime field in gnss_comm, §10 L0).
class TrackCounter {
 public:
  explicit TrackCounter(const Config::Masks& m);
  // Advance with the new epoch: a satellite present with a valid psr/cp increments, absence or an
  // LLI/status break resets. Must be called once per epoch before evaluate(). A hole in the epoch stream itself
  // (interval > kGapFactor × the running median interval, e.g. the 0.5 s gap of AMENDMENTS A3) restarts every
  // arc (review n3); the median makes this rate-independent (10 Hz receiver or 1 Hz RINEX).
  void tick(const ObsEpoch& epoch);
  int psrCount(const SatId& sat) const;
  int cpCount(const SatId& sat) const;
  bool psrOk(const SatId& sat) const;
  bool cpOk(const SatId& sat) const;
  void reset();
  static constexpr double kGapFactor = 2.5;

 private:
  Config::Masks masks_;
  std::map<int, int> psr_, cp_;       // keyed by rtk_sat
  std::map<int, GTime> last_seen_;
  // Appended by WP-c (INTERFACE_CHANGES.md, review n3): epoch-stream gap detection.
  GTime t_prev_;                      // previous tick's epoch time (invalid before the first tick)
  std::vector<double> intervals_;     // last ≤ 20 positive epoch intervals [s] (median = nominal rate)
};

class QualityGate {
 public:
  explicit QualityGate(const Config& cfg);
  MaskVerdict evaluate(const MaskInput& in) const;
  static bool isBdsGeo(const SatId& sat);          // prn ≤ 5 || prn ≥ 59 (ephemeris.c:265)
  bool systemEnabled(Sys sys) const;
  bool excluded(const SatId& sat) const;

 private:
  const Config& cfg_;
};

// Variances of §4.7. Every term is documented in the .cpp next to the formula it implements.
struct NoiseModel {
  explicit NoiseModel(const Config& cfg);

  // κ_AR(τ, Δt) = (1+ρ)/(1−ρ), ρ = exp(−Δt/τ): variance inflation of an AR(1) sample mean.
  static double kappaAR(double tau_s, double dt_s);

  struct PsrTerms {
    double el_rad = 0.0, cn0 = 0.0, psr_std_m = 0.0;
    Sys sys = Sys::GPS;
    bool bds_geo = false;
    double var_eph = 0.0, var_ion = 0.0, var_trop = 0.0;
    double bridge_term = 0.0;            // u_jᵀ M Σ_D Mᵀ u_j
    double clock_bridge = 0.0;           // S_b δ + S_d δ³/3
    double toff_term = 0.0;              // (u_jᵀ w^N σ_toff)²
    double dt_since_last_s = 1.0;        // for κ_AR
  };
  double psrVar(const PsrTerms& t) const;

  // σ_D² = max(λ·dopp_std, σ_D0)²·(1+1/sin²el)/2·κ_AR(τ_D, Δt) + var_v_bridge + S_d δ + (‖ω‖σ_l)² + (‖α_A‖σ_toff)²
  //        + (0.01‖v‖)²   (DESIGN §4.7 as ruled in AMENDMENTS A31, A35-C5/C6 and A45 (C10)).
  struct DoppTerms {
    double el_rad = 0.0, lambda_m = 0.0, dopp_std_hz = 0.0;
    double delta_s = 0.0;                // δ bridge gap
    // Not read since AMENDMENTS A31 (the (δ(σ_ba+gσ_θ))² term is carried by J_w's δθ/δb_a/δg columns); kept because
    // the struct layout is frozen.
    double sigma_ba = 0.0, sigma_theta = 0.0, g = 9.81;
    double omega_norm = 0.0, ant_acc_norm = 0.0, v_norm = 0.0;
    double dt_since_last_s = 0.1;
    // Appended by P3-C (AMENDMENTS A45 / C10; INTERFACE_CHANGES): the bridge's IMU white-noise velocity variance
    // u_jᵀM(var_v_bridge·I)Mᵀu_j = var_v_bridge = acc_n²·δ (BridgeResult::var_v_bridge); outside κ_AR (A35-C5).
    double var_v_bridge = 0.0;
    // Appended (AMENDMENTS A61 F8; INTERFACE_CHANGES): u_jᵀΣ_ω,v u_j, the gyro white noise of ω̂_g through the lever arm
    // (BridgeResult::var_omega; RowBuilder::dopplerRows puts the cross terms into the full R); outside κ_AR.
    double var_omega_lever = 0.0;
  };
  double doppVar(const DoppTerms& t) const;

  // TDCP variance (A14 corrects DESIGN §4.7): σ_Φ is in metres, so the phase term is σ²_Φ(k0) + σ²_Φ(k1)
  // with NO λ² factor; λ only converts cp_std given in cycles into metres (StdUnitJudge::apply).
  struct TdcpTerms {
    double lambda_m = 0.0;               // informational (already applied to cp_std_*_m)
    double el_k0_rad = 0.0, el_k1_rad = 0.0;
    double cp_std_k0_m = 0.0, cp_std_k1_m = 0.0;
    double bridge_term_k1 = 0.0;         // u_jᵀ M Σ_{D,k1} Mᵀ u_j (k0 term lives in Σ_cc)
    double dt_pair_s = 0.5;
    double dv_pair_norm = 0.0;           // ‖Δv_pair‖ for the σ_toff term (R1-6)
    // Appended (AMENDMENTS A61 F11; INTERFACE_CHANGES): the phase noise of that end is a state (Slot::Phase) whose column
    // the row carries, so its σ²_Φ leaves R. R then keeps kTdcpVarFloorFrac of the phase variance it gave up, only so that
    // it stays positive when every other term vanishes (a model-matched scene): an absolute floor is noise the scene does
    // not have, and over thousands of chained pairs even (0.1 mm)² made the filter measurably conservative (A61).
    bool phase_k0_state = false, phase_k1_state = false;
  };
  double tdcpVar(const TdcpTerms& t) const;
  static constexpr double kTdcpVarFloorFrac = 1e-6;

  // Loose-coupling rows (§4.6): R = diag(h_acc², h_acc², v_acc²)·κ_AR(τ_pvt) ; velocity (vel_acc)²·2.
  Eigen::Vector3d pvtPosVar(double h_acc_m, double v_acc_m, double dt_since_last_s) const;
  Eigen::Vector3d pvtVelVar(double vel_acc_mps, double dt_since_last_s) const;

  // Receiver std (metres) with the σ floor rules of §4.7: σ_Φ = max(cp_std_m, σ_Φ0/sin el).
  double sigmaPhase(double cp_std_m, double el_rad) const;

  const Config& cfg;

  // Appended (AMENDMENTS A75, livgo/noise_mode = flat; INTERFACE_CHANGES): fixed row noise for the FAST-LIVGO-v2 paper
  // variant. Off by default: every other arm never sets it, and doppVar()/tdcpVar() then run the §4.7 model unchanged.
  // On: a Doppler row has variance sigma_dopp_mps² + the terms of the IMU bridge that no state column carries
  // (var_v_bridge = acc_n²δ, var_omega_lever = u_jᵀΣ_ω,v u_j, the clock-drift term S_d δ) and a TDCP row sigma_tdcp_m² +
  // bridge_term_k1 (u1ᵀMΣ_{D,k1}Mᵀu1); no elevation, C/N0 or receiver-std dependence, no κ_AR, no noise.scale, no
  // σ_toff, no TDCP rate/wind-up term, no lever-arm or registration-drift term. The bridge cross terms between rows
  // (RowBuilder's R_full) are scaled by noise.scale: the owner of a flat model hands the builder a Config whose scale
  // is 1 (GnssFusionLivgo does). Pseudorange variance is not affected.
  struct Flat {
    bool on = false;
    double sigma_dopp_mps = 0.0, sigma_tdcp_m = 0.0;
  };
  Flat flat;

  // Appended (AMENDMENTS A61 F4; INTERFACE_CHANGES): the single-sample (marginal) Doppler variance of the receiver term,
  // max(λ·dopp_std, σ_D0)²·(1 + 1/sin²el)/2·scale.dopp [m²/s²] — doppVar()'s receiver term without κ_AR (which describes
  // the variance of a time average, A58 ruling 1) and without the bridge/lever/time-offset terms. Used by the D4 slip
  // threshold, which tests one Doppler pair, not an average.
  double doppMarginalVar(double lambda_m, double dopp_std_hz, double el_rad) const;
};

// Ingest self-check 3: decide whether psr_std/cp_std/dopp_std are on the {0.01·2ⁿ} / {0.004·n or 0.004·n/λ}
// / {0.002·2ⁿ} grids, and rewrite the *_m/_hz members of every SignalObs accordingly.
class StdUnitJudge {
 public:
  void observe(const ObsEpoch& epoch);
  bool decided() const;
  StdUnit psrUnit() const { return psr_; }
  StdUnit cpUnit() const { return cp_; }
  StdUnit doppUnit() const { return dopp_; }
  // Force a decision (config / tests).
  void set(StdUnit psr, StdUnit cp, StdUnit dopp);
  // Convert raw std → metres/Hz in place using the judged units and the signal wavelength.
  void apply(ObsEpoch* epoch) const;
  int samples() const { return n_; }

 private:
  StdUnit psr_ = StdUnit::Unknown, cp_ = StdUnit::Unknown, dopp_ = StdUnit::Unknown;
  int n_ = 0;
  std::map<int, int> psr_hist_, cp_hist_m_, cp_hist_cyc_, dopp_hist_;   // grid hits
};

}  // namespace livo_gnss
