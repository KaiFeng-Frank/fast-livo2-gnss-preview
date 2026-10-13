// SPDX-License-Identifier: BSD-3-Clause
// Diagnostics records emitted by the core (DESIGN §15): every column of gnss_epochs.csv, gnss_sats.csv,
// filter_cov.csv and every event of events.log is a member here. The core only fills structs and calls
// the sink; file formats live in ros/src/outputs.cpp and gnss/src/diagnostics.cpp (CSV row/header).
#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "livo_gnss/export.hpp"
#include "livo_gnss/types.hpp"

namespace livo_gnss {

// State machine phases (DESIGN §8).
enum class Phase : uint8_t { OFF = 0, OBSERVE, WAIT_CORE, WAIT_NAV, COLLECT, ALIGN, TRACKING, OUTAGE, RECOVERY };
LIVO_GNSS_API const char* phaseName(Phase p);

// Measurement classes (rows of §4.3–§4.6). New classes go before Count.
enum class MeasClass : uint8_t { Psr = 0, Dopp, Tdcp, PvtPos, PvtVel, Count };
LIVO_GNSS_API const char* measClassName(MeasClass c);

// Why a candidate row did not make it into the update (§10 layers). One counter per reason.
// Enums with a `Count` sentinel: new values are inserted *before* Count (OWNERSHIP §3-1).
// (`NotRejected`, not `None`: X11 `#define None 0L`.)
enum class RejectReason : uint8_t {
  NotRejected = 0,
  Mask,        // L0
  L1,          // GNSS-only robust LS outlier
  Clock,       // L2 clock-jump handling removed the row
  SlipD1, SlipD2, SlipD3, SlipD4, SlipD5, SlipD6,   // L3 detectors
  WTest,       // L4 Baarda w
  Nis,         // L4 epoch NIS
  Majority,    // L4 >50% rejected → class skipped this epoch
  Step,        // L5 post-check rollback
  Exclusive,   // Doppler excluded on TDCP pair endpoints (§4.4)
  NoEph,       // no healthy ephemeris
  Late,        // δ > max_bridge_s
  BelowMin,    // L4: fewer than min_meas rows left after rejection → the class is not updated (A35; appended P2b)
  Count
};
LIVO_GNSS_API const char* rejectReasonName(RejectReason r);

// Per-class counters for one epoch.
struct LIVO_GNSS_API ClassStats {
  int candidates = 0;
  int accepted = 0;
  std::array<int, static_cast<size_t>(RejectReason::Count)> rejected{};   // indexed by RejectReason
  double nis_prior = 0.0;   // ν_0ᵀ S_0⁻¹ ν_0 over the accepted rows
  int dof = 0;
  double nis_post = 0.0;    // rᵀ(R − HΣ⁺Hᵀ)⁻¹ r (L5)
  bool skipped = false;     // class not updated this epoch (Majority / disabled / no rows)
};

// One row of gnss_epochs.csv (DESIGN §15). Column order is fixed by csvHeader() in diagnostics.cpp.
struct LIVO_GNSS_API EpochRecord {
  // time
  WeekTow gpst;             // epoch receiver time (week, tow)
  double t_gpst = 0.0;      // same as seconds (GTime::toSec)
  double t_est = 0.0;       // mapped estimator time (§1)
  double t_k = 0.0;         // LIO cycle end used for the update
  double delta = 0.0;       // δ = t_k − t_g [s] (§4.1)
  double arrival_latency = 0.0;   // t_record − t_est [s]
  bool cross_cycle_bridge = false;
  // state machine / flags
  Phase phase = Phase::OFF;
  bool degenerate = false, livo_alarm = false, schmidt = false, observe_only = false;
  // satellites per system (after L0)
  int n_gps = 0, n_glo = 0, n_gal = 0, n_bds = 0, n_qzs = 0, n_total_raw = 0;
  // per-class stats
  std::array<ClassStats, static_cast<size_t>(MeasClass::Count)> cls{};
  // gain shares (§5 D3): ‖K_p z‖/‖K z‖ etc.
  double f_p = 0.0, f_xi = 0.0, f_clk = 0.0, f_v = 0.0;
  // registration and clock states with σ
  Eigen::Vector2d tilt = Eigen::Vector2d::Zero(), tilt_sigma = Eigen::Vector2d::Zero();
  double yaw = 0.0, yaw_sigma = 0.0;
  Eigen::Vector3d trans = Eigen::Vector3d::Zero(), trans_sigma = Eigen::Vector3d::Zero();
  double clk_b = 0.0, clk_b_sigma = 0.0, clk_d = 0.0, clk_d_sigma = 0.0;
  double isb_C = 0.0, isb_E = 0.0, isb_R = 0.0;      // NaN when the slot does not exist
  double td = 0.0, td_sigma = 0.0;                   // NaN when estimate_online = false
  // SPP vs filter antenna position (N frame) [m]
  Eigen::Vector3d spp_minus_filter = Eigen::Vector3d::Zero();
  bool spp_valid = false;
  // degeneracy index (§10 last row): eigenvalues of the translation Schur complement + principal axis
  Eigen::Vector3d lambda = Eigen::Vector3d::Zero();
  Eigen::Vector3d u1 = Eigen::Vector3d::Zero();
  // host σ = √diag(P) after the update (19)
  Eigen::Matrix<double, 19, 1> host_sigma = Eigen::Matrix<double, 19, 1>::Zero();
  // update magnitudes
  double dp_host_norm = 0.0, dyaw = 0.0, dt_norm = 0.0;
  int iterations = 0;
  bool committed = false, rolled_back = false;
  int n_tdcp_pairs_open = 0;
  double elapsed_ms = 0.0;
  // Appended in the P3 integration (AMENDMENTS A43, INTERFACE_CHANGES P3-C C12): cumulative number of epochs the 1 Hz
  // rule skipped so far (not the first epoch of their GPST second, or older than the last second taken) — pseudorange
  // epochs of the tight arms and receiver solutions of the L-PVT arm. Last two columns of gnss_epochs.csv.
  uint64_t psr_decim_skipped = 0, pvt_decim_skipped = 0;

  static std::string csvHeader();
  std::string csvRow() const;     // %.17g for deterministic columns
};

// One row of gnss_sats.csv (optional, DESIGN §15).
struct LIVO_GNSS_API SatRecord {
  WeekTow gpst;
  SatId sat;
  double el_deg = 0.0, az_deg = 0.0, cn0 = 0.0;
  bool psr_usable = false, dopp_usable = false, cp_usable = false;
  double sigma_psr = 0.0, sigma_dopp = 0.0, sigma_tdcp = 0.0;
  double innov_psr = 0.0, innov_dopp = 0.0, innov_tdcp = 0.0;
  double w_psr = 0.0, w_dopp = 0.0, w_tdcp = 0.0;
  RejectReason reason_psr = RejectReason::NotRejected, reason_dopp = RejectReason::NotRejected, reason_tdcp = RejectReason::NotRejected;
  uint8_t slip_detectors = 0;     // bit i-1 = detector Di fired
  bool nlos_suspect = false;

  static std::string csvHeader();
  std::string csvRow() const;
};

// One row of filter_cov.csv (D1/D3 diagnostics). stage: prior | postLIO | postVIO | postGNSS.
struct LIVO_GNSS_API CovRecord {
  double t_k = 0.0;
  std::string stage;
  int effct_feat_num = -1;        // host-only quantity; -1 when unknown (postGNSS rows)
  Eigen::Matrix<double, 19, 1> host_sigma = Eigen::Matrix<double, 19, 1>::Zero();
  Eigen::Vector3d pos_eig = Eigen::Vector3d::Zero();
  Eigen::Vector3d rot_eig = Eigen::Vector3d::Zero();
  double min_diag_prior = 0.0;    // min diag(P⁻) and condition number (R3-16)
  double cond_prior = 0.0;
  Eigen::VectorXd aug_sigma;      // √diag(Q_G), empty when no slots
  // Eigenvectors (columns, same order as pos_eig / rot_eig) — DESIGN §5 D1 asks for the position block's
  // principal axes, not only its eigenvalues (m20).
  Eigen::Matrix3d pos_eigvec = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d rot_eigvec = Eigen::Matrix3d::Identity();

  static std::string csvHeader();
  std::string csvRow() const;
};

// events.log (DESIGN §15). New kinds go before Count.
enum class EventKind : uint8_t {
  ACTIVATE = 0,
  REANCHOR,          // L6
  REINIT,            // §11
  REINIT_YAW,        // L7 heading
  CLOCK_JUMP,        // L2
  SLIP,              // L3
  LATE_EPOCH,        // δ > max_bridge_s
  TOFF_WARN,         // §7 residual time-offset diagnostics
  LIVO_ALARM_ON, LIVO_ALARM_OFF,   // L7
  DEGENERATE_ON, DEGENERATE_OFF,
  REPIVOT,           // §2.3
  ROLLBACK,          // L5
  MSGDEF_REJECT,     // gnss_io layout mismatch (§12.2)
  OUTAGE_ON, OUTAGE_OFF,
  RECOVERY,
  PSEUDOINVERSE,     // H2 fallback to eigen-decomposition pseudo-inverse (§2.4)
  TDCP_PAUSED,       // insufficient excitation (§7)
  PHASE_CHANGE,      // generic state-machine transition
  CONFIG_WARNING,
  FRAME_JUMP,        // appended (AMENDMENTS A61 F9): a host update taken as a jump of the map frame
  REINIT_SUPPRESSED, // appended (AMENDMENTS A82 livgo.single_init): a re-initialisation trigger that was not acted on (magnitude = running total, detail names the branch)
  Count
};
LIVO_GNSS_API const char* eventKindName(EventKind k);

struct LIVO_GNSS_API Event {
  EventKind kind = EventKind::PHASE_CHANGE;
  double t_k = 0.0;              // estimator time of the cycle
  double t_est = 0.0;            // epoch time, 0 if not epoch-bound
  WeekTow gpst;
  double magnitude = 0.0;        // e.g. jump size [m], clock jump k [ms], rollback NIS
  std::string detail;            // free text (satellite id, reason)

  std::string logLine() const;   // one line of events.log
};

// End-of-run summary.json (DESIGN §15).
struct LIVO_GNSS_API Summary {
  int epochs_total = 0, epochs_updated = 0, epochs_rolled_back = 0;
  std::array<double, static_cast<size_t>(MeasClass::Count)> mean_nis_per_dof{};
  std::array<double, static_cast<size_t>(MeasClass::Count)> acceptance_rate{};
  double tdcp_coverage = 0.0;    // fraction of time with an open TDCP pair
  double elapsed_p50_ms = 0.0, elapsed_p95_ms = 0.0, elapsed_p99_ms = 0.0;
  int n_activate = 0, n_reinit = 0, n_reanchor = 0, n_clock_jump = 0, n_slip = 0, n_late = 0;
  int n_msgdef_reject = 0, n_bds_b1c_reject = 0, n_gal_prn_gt36 = 0, n_glo_fcn_mismatch = 0;

  std::string toJson() const;
};

// Receives everything above. Implemented by ros/src/outputs.cpp (files) and by tests (in-memory).
class LIVO_GNSS_API DiagnosticsSink {
 public:
  virtual ~DiagnosticsSink() = default;
  virtual void onEpoch(const EpochRecord&) {}
  virtual void onSat(const SatRecord&) {}
  virtual void onCov(const CovRecord&) {}
  virtual void onEvent(const Event&) {}
  virtual void onSummary(const Summary&) {}
};

}  // namespace livo_gnss
