// SPDX-License-Identifier: BSD-3-Clause
// OutageManager — TRACKING ⇄ OUTAGE → RECOVERY → REINIT and the L6 lock-out recovery (DESIGN §11, §10 L6):
//   OUTAGE: 1 s without an L0-valid epoch; clones parked; ξ walks by path, clocks by time (no floors).
//   RECOVERY: ≥4 sats stable for 1 s; σ_b > 1e3 m ⇒ clock-only LS consistent reset; first 5 epochs fixed 3
//             iterations; L4 active throughout; no artificial clamps or R ramps.
//   REINIT: OUTAGE > 60 s and (σ_t > 10 m or σ_ψ > 0.1 rad), or L6/L7 trigger ⇒ drop frame+clock slots,
//           back to COLLECT→ALIGN; global output may jump at the event.
//   L6: 20 consecutive epochs with L1 consistent (≥5 sats) but L4 rejecting >50 % ⇒ re-augment t and clock
//       from SPP (REANCHOR); Doppler systematically rejected too ⇒ redo ψ. Disabled in the T−pr arm.
// Owner: P3-A1 (OWNERSHIP §2c). Tests: test_u17_integrity.cpp (U17Outage: state machine, L6, the two actions below),
// test_u15_mc.cpp (outage 120 s scenario).
//
// Decision semantics of step() (outage.cpp has the details, INTERFACE_CHANGES P3-A1 the contract):
//   * "valid epoch" = an epoch with ≥ 1 satellite through L0. OUTAGE starts when no valid epoch arrived for more than
//     outage_s; the timer starts at the last valid epoch. Leaving it needs recover_min_sats in every arriving epoch for
//     recover_settle_s (cycles without an epoch neither break nor extend the settling).
//   * RECOVERY lasts the reacquisition window (setReacqEpochs, = iterate.reacq_epochs) and then returns to TRACKING;
//     reacq_epochs_left > 0 in a decision means "force three iterations for this cycle's epochs" (also after ACTIVATE
//     and REANCHOR).
//   * L6 counts only epochs whose class had candidates (NaN fraction = no evidence either way); with 1 Hz pseudoranges
//     20 epochs are 20 s. Pseudorange lock-out alone ⇒ reanchor (reanchorFromSpp below); with the Doppler locked out
//     as well ⇒ reanchor_yaw and reinit (ψ can only be redone by COLLECT→ALIGN).
//   * reinit is raised in OUTAGE (duration > reinit_after_s and σ too large) or by L6 with the Doppler; the phase stays
//     where it is until the caller has dropped the slots and called onReinitDone(); step() must not be called again
//     before the next onActivated().
#pragma once

#include <string>
#include <vector>

#include "livo_gnss/config.hpp"
#include "livo_gnss/core_state.hpp"
#include "livo_gnss/diagnostics.hpp"
#include "livo_gnss/types.hpp"
#include "aug_block.hpp"
#include "gnss_only.hpp"
#include "rows.hpp"

namespace livo_gnss {

struct OutageDecision {
  Phase phase = Phase::TRACKING;
  bool entered_outage = false, left_outage = false;
  bool start_recovery = false;
  bool clock_reset_needed = false;     // σ_b > 1e3 m at recovery
  bool reinit = false;
  bool reanchor = false;               // L6 (t + clock)
  bool reanchor_yaw = false;           // L6 with Doppler also rejected
  int reacq_epochs_left = 0;           // fixed-3-iteration window
  double outage_duration_s = 0.0;
};

class OutageManager {
 public:
  OutageManager(const Config::Outage& cfg, const Config::Rejection& rej, bool l6_enabled);

  // Per LIO step with the epoch bookkeeping of this cycle. `n_valid_sats` = sats passing L0 (0 when no
  // epoch arrived), `l1_consistent` = GNSS-only LS OK with ≥5 sats, `l4_reject_frac_psr/dopp` = fraction
  // rejected by L4 this epoch (NaN when the class had no candidates).
  OutageDecision step(double t_k, bool epoch_arrived, int n_valid_sats, bool l1_consistent,
                      double l4_reject_frac_psr, double l4_reject_frac_dopp, double sigma_t_m, double sigma_yaw_rad,
                      double sigma_b_m);
  void onActivated(double t_k);        // resets timers; starts the reacquisition window
  void onReinitDone(double t_k);
  Phase phase() const { return phase_; }
  double lastValidTime() const { return t_last_valid_; }
  int lockoutCount() const { return n_lockout_; }

 private:
  Config::Outage cfg_;
  Config::Rejection rej_;
  bool l6_enabled_ = true;
  Phase phase_ = Phase::TRACKING;
  double t_last_valid_ = 0.0, t_outage_start_ = 0.0, t_settle_start_ = 0.0;
  int n_lockout_ = 0, n_lockout_dopp_ = 0, reacq_left_ = 0;

 public:
  // ---- appended by P3-A1 (INTERFACE_CHANGES) ----
  static constexpr double kClockResetSigmaB = 1.0e3;   // DESIGN §11 "σ_b > 1e3 m"
  // Length of the fixed-3-iteration window after ACTIVATE / RECOVERY / REANCHOR (cfg.iterate.reacq_epochs, default 5;
  // the constructor gets no Iterate config).
  void setReacqEpochs(int n);
  int reacqEpochsLeft() const { return reacq_left_; }
  int dopplerLockoutCount() const { return n_lockout_dopp_; }
  double outageStart() const { return t_outage_start_; }

 private:
  int reacq_epochs_ = 5;
  bool settling_ = false;
};

// ---- appended by P3-A1 (INTERFACE_CHANGES): the actions behind OutageDecision::reanchor and ::clock_reset_needed ----
// L6 REANCHOR (DESIGN §10 L6 "以 SPP 重新一致增广 t 与钟"): the trans and clk slots are replaced by the epoch's GNSS-only
// solution with a consistent augmentation: t' = t̂ + (a^N_SPP − â^N), e_t' = n_SPP − (J_a·e with the t columns removed),
// so the global antenna takes the SPP value and covariance while every other state keeps its own; (b', d') are the SPP
// clock and the epoch's Doppler-LS drift, moved from the epoch to t_k by δ = t_k − t_g (Φ(δ) plus the clock process noise),
// with the SPP position–clock covariance kept between t' and b'. The formal covariances are recomputed from `in` (the
// epoch's SppInputs as RobustSpp::solve left them) because SppSolution::Q/DopplerSolution::Q are scaled by σ̂0² (zero for
// an exact fit); scale max(1, σ̂0²), position floored at init.spp_sigma_floor_m. Preconditions: trans and clk slots
// present, no clone slot (park the TDCP pairs first), spp.ok and dop.ok; `J` = RowBuilder::antennaJacobians at the
// epoch over the current aug layout. On failure nothing is changed.
bool reanchorFromSpp(AugBlock* aug, const AntennaJacobians& J, const AnchorFrame& anchor, const SppSolution& spp,
                     const DopplerSolution& dop, const std::vector<SppInput>& in, double delta_s, const Config& cfg,
                     const HostMat& P_host, std::string* why);
// RECOVERY with σ_b > 1e3 m (DESIGN §11 "以预测天线位置做只解钟 LS 一致重置"): clockOnlyLs() at the predicted antenna
// (anchor + R_EN·J.a_N, R_EN·J.w_N) as a consistent reset. The per-satellite clock estimates depend on the predicted
// position through their lines of sight, so e_b = n_b + ē_bᵀR_EN·J_a·e and e_d = n_d + ē_dᵀR_EN·J_w·e (ē = mean LOS of
// the satellites used, ECEF); the clk slot is re-augmented with these rows, moved to t_k by δ, noise = the MAD² variances.
// (L2's non-integer jump keeps WP-b's resetClockStates, "C 钟行清零".) Needs a GPS/QZSS code and a Doppler; on failure
// nothing is changed. `result` (optional) receives the clock-only LS.
bool recoveryClockReset(AugBlock* aug, const AntennaJacobians& J, const AnchorFrame& anchor, const std::vector<SppInput>& in,
                        double delta_s, const Config& cfg, const HostMat& P_host, ClockOnlyResult* result, std::string* why);

}  // namespace livo_gnss
