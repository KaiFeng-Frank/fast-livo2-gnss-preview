// SPDX-License-Identifier: BSD-3-Clause
// GnssFusionLivgo facade: the DESIGN §9 schedule over the internal modules. Owner: P3-A2 (OWNERSHIP §2c).
//
// Per LIO cycle (update_slot = pre_lio):
//   onLioStep   bridge ring ← the cycle's samples; host readiness (WAIT_CORE); H1 (AugBlock::propagate with Δs, Δα from
//               the bridge samples and the A32 wind-up Δt_epoch = running median epoch interval); then every queued
//               measurement whose estimator time is ≤ t_k, in GPST order: the state machine (WAIT_NAV → COLLECT →
//               ALIGN/ACTIVATE) or, once active, L0 → L1 → L2 → L3 → [L4 → H3 → L5 inside JointUpdater] → TDCP
//               pairing / clones → diagnostics; the outage / lock-out manager (§11, L6) runs once per cycle after that.
//   onCoreUpdated  H2 after every host update; the LIO one also feeds the degeneracy judge.
//   onPostVio   update_slot = post_vio (AMENDMENTS A12): the epochs onLioStep held back are processed here, on the
//               posterior of the cycle's last host update. Both slots consume the same epochs.
//   onStepEnd   C_ref refresh, re-pivot check (§2.3), rate-limited output pose (§11).
//
// Rules this file implements itself (the modules below are called through their frozen internal headers):
//   * time: t_est = TimeMap(t_rx − b̂/c) (DESIGN §1); an epoch is consumed by the first cycle with t_est ≤ t_k, bridged
//     across cycles while δ ≤ max_bridge_s (§4.1), otherwise dropped as LATE_EPOCH; non-monotone/duplicate epochs are
//     dropped at ingestion (§10 L0 "历元时间单调/去重");
//   * AMENDMENTS A43: pseudorange and PVT rows use the first epoch of every GPST second (⌊t / decim_s⌋ buckets),
//     independent of the receiver's epoch phase; the update's RowBuilder is only ever given the selected epochs, so the
//     1 Hz rule inside rows.cpp (P3-C's EpochDecimator, same decim_s) sees one epoch per bin and takes it; the skipped
//     epochs are counted here (gnss_epochs.csv psr_decim_skipped / pvt_decim_skipped). L2 builds the code innovations of
//     every epoch with a scratch RowBuilder per epoch. The Config copy read by the RowBuilder and the noise model carries
//     the σ_toff of the epoch's time-map segment (a calibration file overrides the configured value, config.hpp);
//   * TDCP (§4.5): non-overlapping pairs k0 → k1 = first epoch ≥ interval_s after k0 (half an epoch interval of
//     tolerance), the next pair from k1 itself (noise.tdcp.chain_pairs, AMENDMENTS A61 F10; k1 + 1 without it), the
//     first after ACTIVATE from k_a + 1; the clone c = a^N(x̂_k0) is
//     augmented consistently after k0's update (RowBuilder::cloneJacobians), removed after k1's update, parked on
//     OUTAGE/REINIT/REANCHOR or when k1 never comes within max_span_s; both ends use the ephemeris frozen at k0; the
//     Doppler rows of the pair's satellites are left out at both end epochs (§4.4, exclusive_with_tdcp);
//   * AMENDMENTS A70 (A69 ruling): in the joint update the accepted Doppler rows of the epoch are projected onto the
//     orthogonal complement of their φ columns (noise.dopp.project_tilt, default on → JointUpdater::Options::project):
//     no tilt content is left in them, so they neither move φ nor make the host correlated with it;
//   * AMENDMENTS A66 (an option since A70, default off): the Doppler rows treat the registration tilt φ as a consider
//     parameter (noise.dopp.consider_tilt → JointUpdater::Options::consider): no gain from them onto φ, φ's σ still in
//     their S;
//   * AMENDMENTS A75 (LIVGO-v2, every switch default off = the arm as before): livgo.frame_exact makes the registration tilt
//     and translation exact constants after ACTIVATE (zero variance and covariance, AugBlock::pinRegistrationExact) and the
//     yaw an unpinned constant state that the GNSS rows keep estimating (no consider entry); a fixed-anchor TDCP row then
//     takes the registration and the host gravity as states it shares with its anchor (livgo_anchor.hpp);
//     livgo.noise_mode = flat gives the Doppler/TDCP rows one fixed σ plus the IMU-bridge terms (quality.hpp NoiseModel::Flat);
//     livgo.dtw_online = false drops the online DTW. TDCP pairs of consecutive receiver epochs are noise.tdcp.interval_s 0.1 /
//     max_span_s 0.15 with chain_pairs: a pair closes at the first epoch ≥ interval_s − ½ epoch_dt after k0 and opens the next
//     one there, a gap beyond max_span_s parks the chain (the arm yaml, no extra code);
//   * AMENDMENTS A80 (LIVGO-literal, every switch default off = the arm as before): livgo.freeze_yaw (with frame_exact) pins the
//     yaw as well, so the registration is three exact numbers; livgo.init_travel_m holds the first activation back until the host
//     has moved that far horizontally and then solves the eq. (2)-(3) least squares once on every synchronous epoch collected
//     (LivgoInitHistory) instead of on the initializer's sliding window; livgo.init_tilt_sigma_max_deg takes the tilt from the host
//     gravity when the least squares' formal tilt sigma is above it; livgo.cn0_hard sets the three C/N0 masks and the elevation
//     mask to the paper's values; livgo.primary_only drops a satellite whose first signal is not its system's primary one;
//     livgo.dtw_grid_s / dtw_window_s / dtw_min_epochs are the online DTW's grid, window and minimum sample count;
//   * AMENDMENTS A82 (LIVGO-literal initialisation in the paper's order, both switches default off): livgo.dtw_at_init makes the DTW once,
//     when the travel gate opens, on the whole history of the frame least squares and takes its lag only if the author's acceptance test
//     passes (the host poses of the history are then re-paired at the compensated times, from a pose track kept during COLLECT);
//     livgo.single_init lets the first ACTIVATE stand: REINIT_YAW, the outage/L6/L7 re-initialisation and the H2 fallback are only counted
//     (event REINIT_SUPPRESSED);
//   * ISB slots (A35): after ACTIVATE every enabled system that carries code rows (GAL, BDS; GLO only with glo_psr)
//     gets its slot, σ = 30 m when the alignment did not observe it; the td slot likewise when estimate_online;
//   * OBSERVE (update_enable = false): everything is computed (state machine, activation, H1/H2 on the augmented
//     block, rows, gates, the joint update in JointUpdater's observe_only form) but nothing is committed — the host is
//     never written, and neither are the measurement-driven corrections of x_G (H3, L2 clock corrections, L6
//     re-anchoring). As a last line of defence onLioStep/onPostVio compare the host against its entry copy and restore
//     it (event CONFIG_WARNING) — the bypass gate (G-旁路) must never depend on that.
//   * gnss.enable = false: the constructor validates the configuration and returns; every method is a no-op that
//     touches neither the host state nor any global state (no RTKLIB selection, no module is built).
#include "livo_gnss/gnss_fusion_livgo.hpp"
#include "livo_gnss/livgo.hpp"
#include "livo_gnss/livgo_init.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <deque>
#include <limits>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "internal/aug_block.hpp"
#include "internal/bridge.hpp"
#include "internal/clock.hpp"
#include "internal/ephem_store.hpp"
#include "internal/gnss_only.hpp"
#include "internal/initializer.hpp"
#include "internal/integrity.hpp"
#include "internal/leveling.hpp"
#include "internal/livgo_anchor.hpp"
#include "internal/livgo_gain.hpp"
#include "internal/outage.hpp"
#include "internal/quality.hpp"
#include "internal/rows.hpp"
#include "internal/rtk_adapter.hpp"
#include "internal/slip.hpp"
#include "internal/time_map.hpp"
#include "internal/time_offset.hpp"
#include "internal/update.hpp"

namespace livo_gnss {

namespace {

constexpr double kCLight = 299792458.0;
constexpr double kFreqL1 = 1.57542e9;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kIsbUnobservedSigmaM = 30.0;   // AMENDMENTS A35: ISB of a system the alignment did not observe
constexpr size_t kIntervalWindow = 21;          // epochs in the running-median epoch interval (A32)
constexpr double kDefaultEpochDt = 0.1;         // nominal M3DGR receiver interval until measured (A32)
constexpr double kTimeSlackS = 1e-6;            // t_est − t_k below this is rounding of the b̂/c term, not "next cycle"

using Clock = std::chrono::steady_clock;

double msSince(Clock::time_point t0) { return std::chrono::duration<double, std::milli>(Clock::now() - t0).count(); }

// Bit-for-bit equality of every host member (memcmp: −0 ≠ +0 and NaN == NaN, unlike operator==) — the OBSERVE guard.
bool sameBits(const CoreState& a, const CoreState& b) {
  auto eq = [](const double* x, const double* y, size_t n) { return std::memcmp(x, y, n * sizeof(double)) == 0; };
  return eq(a.R.data(), b.R.data(), 9) && eq(a.p.data(), b.p.data(), 3) && eq(a.v.data(), b.v.data(), 3) &&
         eq(a.bg.data(), b.bg.data(), 3) && eq(a.ba.data(), b.ba.data(), 3) && eq(a.g.data(), b.g.data(), 3) &&
         eq(&a.inv_expo, &b.inv_expo, 1) && eq(a.P.data(), b.P.data(), HostIdx::dim * HostIdx::dim);
}

// hᵀΣh of one measurement row (1 × N) without mixing row/column vectors in dot().
double quadForm(const Eigen::RowVectorXd& h, const Eigen::MatrixXd& S) { return (h * S * h.transpose()).value(); }

WeekTow weekTow(const GTime& t) {
  WeekTow w;
  if (!t.valid()) return w;
  w.tow = rtk::time2gpst(t, &w.week);
  return w;
}

// A43 bucket of a GPST instant: ⌊(GTime::toSec()) / decim_s⌋, exact integer arithmetic for integral decim_s (the GPST
// week boundary is a whole second on this scale, so ⌊tow⌋ changes exactly where ⌊sec⌋ does).
int64_t decimBucket(const GTime& t, double decim_s) {
  if (!(decim_s > 0.0)) return t.sec;
  const double r = std::round(decim_s);
  if (r >= 1.0 && std::fabs(decim_s - r) < 1e-9) {
    const int64_t d = static_cast<int64_t>(r);
    int64_t q = t.sec / d;
    if ((t.sec % d) < 0) --q;
    return q;
  }
  return static_cast<int64_t>(std::floor(t.toSec() / decim_s));
}

double median(std::vector<double> v) {
  if (v.empty()) return kNaN;
  std::sort(v.begin(), v.end());
  const size_t n = v.size();
  return (n % 2 == 1) ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
}

double sqrtPos(double v) { return (v > 0.0 && std::isfinite(v)) ? std::sqrt(v) : 0.0; }

// Index of the satellite in an epoch, −1 when absent.
int findSat(const ObsEpoch& ep, const SatId& s) {
  for (size_t i = 0; i < ep.sats.size(); ++i)
    if (ep.sats[i].sat.rtk_sat == s.rtk_sat) return static_cast<int>(i);
  return -1;
}

// Second-frequency signal of a satellite usable for the geometry-free slip check (D6), −1 when none.
int secondSignal(const SatObs& so) {
  const SignalObs* p = so.primary();
  if (!p) return -1;
  for (size_t k = 1; k < so.sig.size(); ++k)
    if (so.sig[k].cpValid() && std::fabs(so.sig[k].freq_hz - p->freq_hz) > 1e3) return static_cast<int>(k);
  return -1;
}

int signalWithCode(const SatObs& so, SignalCode code) {
  for (size_t k = 1; k < so.sig.size(); ++k)
    if (so.sig[k].code == code) return static_cast<int>(k);
  return -1;
}

RejectReason slipReason(uint8_t detectors) {
  static const RejectReason kByBit[6] = {RejectReason::SlipD1, RejectReason::SlipD2, RejectReason::SlipD3,
                                         RejectReason::SlipD4, RejectReason::SlipD5, RejectReason::SlipD6};
  for (int b = 0; b < 6; ++b)
    if (detectors & (1u << b)) return kByBit[b];
  return RejectReason::NotRejected;
}

size_t ci(MeasClass c) { return static_cast<size_t>(c); }
size_t ri(RejectReason r) { return static_cast<size_t>(r); }

}  // namespace

// =============================================================================================
// Impl
// =============================================================================================
struct GnssFusionLivgo::Impl {
  explicit Impl(const Config& c) : cfg(c), cfg_rows(c) {}
  ~Impl() {
    if (livgo_log) std::fclose(livgo_log);
    if (gain_log) std::fclose(gain_log);
  }

  // ---------------- configuration and host-derived settings ----------------
  Config cfg;
  // Copy read by RowBuilder and NoiseModel: time.sigma_toff_s = the σ_toff of the time-map segment of the epoch being
  // processed (a calibration file overrides the configured value).
  Config cfg_rows;
  // A75 livgo.noise_mode = flat: Config copy and noise model of the filter rows (noise.scale 1 so the bridge cross terms of
  // R_full are unscaled like the diagonal; σ_toff is not used by flat rows). Declared before `rows`, which refers to them.
  // The GNSS-only estimators (RAIM, DTW, initialisation) keep `noise`.
  Config cfg_flat;
  std::unique_ptr<NoiseModel> noise_flat;
  bool enabled = false;
  DiagnosticsSink* sink = nullptr;
  RawDecoder* decoder = nullptr;
  Eigen::Vector3d lever_I = Eigen::Vector3d::Zero();
  bool extrinsics_set = false;
  SlamMode slam_mode = SlamMode::LIVO;
  bool host_gravity_est = true, host_expo_est = true;
  std::vector<Event> pending_events;   // raised before a sink exists (construction-time warnings)

  // ---------------- modules (built only when enabled) ----------------
  std::unique_ptr<rtk::NavStore> nav;
  std::unique_ptr<EphemStore> ephem;
  std::unique_ptr<rtk::SatSolver> solver;
  std::unique_ptr<TimeMap> tmap;
  std::unique_ptr<ImuBridge> bridge;
  Leveling leveling;
  std::unique_ptr<AugBlock> aug;
  std::unique_ptr<NoiseModel> noise;
  std::unique_ptr<RowBuilder> rows;
  std::unique_ptr<JointUpdater> updater;
  std::unique_ptr<QualityGate> gate;
  std::unique_ptr<TrackCounter> track;
  StdUnitJudge std_judge;
  std::unique_ptr<SlipDetector> slip;
  std::unique_ptr<ClockJumpDetector> jump;
  std::unique_ptr<RobustSpp> spp;
  std::unique_ptr<RobustDopplerLs> dls;
  std::unique_ptr<TimeOffsetMonitor> toff;
  std::unique_ptr<Initializer> init;
  std::unique_ptr<OutageManager> outage;
  std::unique_ptr<IntegrityMonitor> integ;

  // ---------------- measurement queue ----------------
  struct Pending {
    bool is_pvt = false;
    ObsEpoch obs;
    ReceiverPvt pvt;
    GTime t;             // GPST ordering key (receiver tag / navigation epoch)
    uint64_t seq = 0;    // arrival order, ties
  };
  std::vector<Pending> queue;          // sorted by (t, seq)
  std::vector<Pending> held;           // post_vio: consumed by onLioStep, processed by onPostVio (A12)
  uint64_t arrival_seq = 0;
  GTime last_consumed_obs, last_consumed_pvt;   // monotonicity (§10 L0)

  // ---------------- filter state ----------------
  Phase phase = Phase::OFF;
  bool active = false;                 // registration slots exist
  // FAST-LIVGO arm. This class is constructed only when the arm is on.
  LivgoConfig liv;
  Eigen::VectorXd livgo_x_pin;
  bool livgo_pinned = false;
  bool livgo_degraded = false;
  // Full LIVGO (frame freeze, no pseudorange in the filter, DTW, paper init). dual_only keeps T's update and only
  // gates L4/L5 on the Hessian window.
  bool livgoFull() const { return liv.enable && !liv.dual_only; }
  // A75 switches (only meaningful for the full arm).
  bool frameExact() const { return livgoFull() && liv.frame_exact; }
  bool dtwOnline() const { return livgoFull() && liv.dtw_online; }
  bool flatNoise() const { return livgoFull() && liv.noise_mode == LivgoConfig::NoiseMode::Flat; }
  // A80 switches (only meaningful for the full arm).
  bool yawExact() const { return frameExact() && liv.freeze_yaw; }
  bool travelInit() const { return livgoFull() && liv.paper_init && liv.init_travel_m > 0.0; }
  // A81 switch (only meaningful for the full arm).
  bool raimBothEnds() const { return livgoFull() && liv.tdcp_raim_both_ends; }
  // A82 switches (only meaningful for the full arm). dtw_at_init replaces the online DTW's "after dtw_min_epochs epochs" trigger by one DTW at
  // the travel gate, so it needs the DTW itself (dtw_online) and the gate with its history (paper_init, init_travel_m > 0); without them it is
  // ignored (and said so at construction).
  bool dtwAtInit() const { return dtwOnline() && liv.dtw_at_init && travelInit(); }
  bool singleInit() const { return livgoFull() && liv.single_init; }
  bool a82Default() const { return !liv.dtw_at_init && !liv.single_init; }
  bool a80Default() const {
    const LivgoConfig d;
    return !liv.freeze_yaw && !(liv.init_travel_m > 0.0) && !(liv.init_tilt_sigma_max_deg > 0.0) && !liv.cn0_hard && !liv.primary_only &&
           !liv.tdcp_raim_both_ends && liv.dtw_grid_s == d.dtw_grid_s && liv.dtw_window_s == d.dtw_window_s &&
           liv.dtw_min_epochs == d.dtw_min_epochs;
  }
  LivgoInitHistory livgo_hist;         // A80: every synchronous epoch collected since the last activation (travel_init only)
  int livgo_switches = 0;
  std::vector<double> livgo_lambda_win;
  double livgo_lambda = 0.0;
  double livgo_lambda_bar = 0.0;
  FILE* livgo_log = nullptr;
  FILE* gain_log = nullptr;            // A75 revision 1: livgo_tdcp_gain.csv (observer, off by default)
  std::deque<CollectSample> livgo_snap_spp, livgo_snap_la;
  std::string livgo_init_note;
  std::vector<double> dtw_tg, dtw_vg, dtw_th, dtw_vh;
  bool dtw_applied = false;
  // A82: livgo.dtw_at_init (the one DTW of the run, at the travel gate) and livgo.single_init (the re-initialisation gate)
  bool dtw_init_done = false;
  LivgoDtwInit dtw_init;                // its result (the acceptance test's numbers)
  LivgoRepairResult dtw_init_repair;    // what taking the lag did to the history
  LivgoReinitGate reinit_gate;
  AnchorFrame anchor;                  // N frame, fixed at the first ACTIVATE and never changed (§1)
  Eigen::Vector3d pivot_W = Eigen::Vector3d::Zero();
  int host_lio_updates = 0;
  double t_k_last = kNaN;
  Eigen::Vector3d g_last = Eigen::Vector3d::Zero();   // host gravity of the last step (registration() M)
  std::string last_align_reason;
  uint64_t epoch_index = 0;            // obs epochs consumed so far (1-based index of the current one)
  uint64_t activate_index = 0;         // epoch index of k_a
  GTime last_epoch_t;
  double last_epoch_t_est = 0.0, last_valid_t_est = kNaN;
  int n_sats_last = 0;
  std::deque<double> intervals;        // positive receiver epoch intervals (A32 median)
  double epoch_dt = kDefaultEpochDt;
  int64_t psr_bucket = std::numeric_limits<int64_t>::min();
  int64_t pvt_bucket = std::numeric_limits<int64_t>::min();
  uint64_t psr_decim_skipped = 0, pvt_decim_skipped = 0;   // A43 skips (gnss_epochs.csv, C12)
  GTime last_pvt_used;
  SppSolution last_spp;
  bool have_spp = false;
  Klobuchar ubx_iono;                  // A45: GPS Klobuchar decoded from the UBX subframes
  bool have_ubx_iono = false;
  int reacq_left = 0;                  // fixed-3-iteration window (ACTIVATE / RECOVERY)
  double map_path_m = 0.0;             // A61 F1: host path since the first LIO cycle
  // A61 F13 odometry output: origin_a0/psi0 = the Origin slot's [a^N; ψ] when it was made; T_off (N_odo ← N) carries the
  // odometry frame across REINIT / REANCHOR; odo_G (N_odo ← W, last step end) is the mapping a new origin continues.
  Eigen::Vector3d origin_a0 = Eigen::Vector3d::Zero();
  double origin_psi0 = 0.0;
  Eigen::Matrix3d odo_off_R = Eigen::Matrix3d::Identity(), odo_G_R = Eigen::Matrix3d::Identity();
  Eigen::Vector3d odo_off_t = Eigen::Vector3d::Zero(), odo_G_t = Eigen::Vector3d::Zero();
  bool odo_have_G = false;
  bool pending_clock_reset = false, pending_reanchor = false;
  bool toff_warned = false, tdcp_paused_reported = false, origin_checked = false, lever_warned = false, freq_checked = false;
  bool span_warned = false;            // A75 revision 1: epoch interval above noise.tdcp.max_span_s (reported once)

  // ---------------- TDCP pairing (§4.5) ----------------
  struct PairMember {
    SatId sat;
    SignalObs k0;                      // primary signal at k0
    SignalObs k0_second;               // second-frequency signal at k0 (D6)
    bool has_second = false;
    rtk::EphSnapshot snap;             // ephemeris frozen at k0 (R2-11)
    double dts_k0 = 0.0, psr_k0 = 0.0, el_k0 = 0.0;
    Eigen::Vector3d rs_k0 = Eigen::Vector3d::Zero();
    double trop_k0 = 0.0, ion_k0 = 0.0;   // A61 F2: model delays at k0 (ion_k0 NaN when k0 had no Klobuchar)
    bool lli_between = false;          // loss of lock / absence on an intermediate epoch (D1)
    int eta = -1;                      // A61 F11: component of the pair's Slot::Phase (its k0 phase noise), −1 none
  };
  struct TdcpPair {
    int id = -1;
    GTime t_k0;
    uint64_t index_k0 = 0;
    std::vector<PairMember> members;
    double clock_jump_m = 0.0;         // L2 integer-ms jumps inside (k0, k1]
    Eigen::Vector3d w_N_k0 = Eigen::Vector3d::Zero();
    bool fixed_anchor = false;         // A-FA ablation: no clone slot, ĉ frozen
    Eigen::Vector3d c_fixed = Eigen::Vector3d::Zero();
    double b_k0 = 0.0;
    LivgoAnchor anchor_k0;             // livgo/frame_exact: the k0 antenna in W (host posterior + bridge of k0)
  };
  bool pair_open = false;
  TdcpPair pair;
  int next_pair_id = 1;
  uint64_t last_k1_index = 0;

  // ---------------- per-cycle bookkeeping for the outage manager ----------------
  struct Book {
    bool epoch_arrived = false;
    int n_valid = 0;
    bool l1_consistent = false;
    double frac_psr = kNaN, frac_dopp = kNaN;
  } book;

  // ---------------- smoothed output (§11 presentation layer) ----------------
  bool smooth_valid = false;
  double smooth_t = 0.0;
  Eigen::Matrix3d smooth_M = Eigen::Matrix3d::Identity();
  Eigen::Vector3d smooth_b = Eigen::Vector3d::Zero();
  Eigen::Matrix3d smooth_R_NI = Eigen::Matrix3d::Identity();
  Eigen::Vector3d smooth_t_NI = Eigen::Vector3d::Zero();

  // ---------------- totals (summary.json) ----------------
  struct Totals {
    int epochs_total = 0, epochs_updated = 0, epochs_rolled_back = 0;
    std::array<double, static_cast<size_t>(MeasClass::Count)> nis{}, dof{}, cand{}, acc{};
    double tdcp_open_s = 0.0, active_s = 0.0;
    std::vector<double> elapsed;
    int n_activate = 0, n_reinit = 0, n_reanchor = 0, n_clock_jump = 0, n_slip = 0, n_late = 0;
    int n_msgdef_reject = 0, n_bds_b1c = 0, n_gal_prn = 0, n_glo_fcn = 0;
  } tot;
  bool summary_sent = false;

  // =========================================================================================
  // small helpers
  // =========================================================================================
  void emit(EventKind k, double t_k, double t_est, const GTime& g, double mag, const std::string& detail) {
    Event e;
    e.kind = k;
    e.t_k = std::isfinite(t_k) ? t_k : 0.0;
    e.t_est = t_est;
    e.gpst = weekTow(g);
    e.magnitude = mag;
    e.detail = detail;
    switch (k) {
      case EventKind::ACTIVATE: ++tot.n_activate; break;
      case EventKind::REINIT: ++tot.n_reinit; break;
      case EventKind::REANCHOR: ++tot.n_reanchor; break;
      case EventKind::CLOCK_JUMP: ++tot.n_clock_jump; break;
      case EventKind::SLIP: ++tot.n_slip; break;
      case EventKind::LATE_EPOCH: ++tot.n_late; break;
      case EventKind::MSGDEF_REJECT: ++tot.n_msgdef_reject; break;
      default: break;
    }
    if (sink) sink->onEvent(e); else if (pending_events.size() < 64) pending_events.push_back(e);
  }

  void setPhase(Phase p, double t_k, const GTime& g, const std::string& why) {
    if (p == phase) return;
    const std::string d = std::string(phaseName(phase)) + "->" + phaseName(p) + (why.empty() ? "" : " " + why);
    phase = p;
    emit(EventKind::PHASE_CHANGE, t_k, 0.0, g, 0.0, d);
  }

  // The initializer moves WAIT_CORE → WAIT_NAV → COLLECT → ALIGN by itself (INTERFACE_CHANGES P3-A1); the facade only
  // tells it a phase it has not reached on its own, because setPhase() also resets its ALIGN retry wait.
  void syncInitPhase(Phase p) {
    if (init->phase() != p) init->setPhase(p);
  }

  bool tight() const { return cfg.mode != Config::Mode::LoosePvt; }
  bool observe() const { return !cfg.update_enable; }
  bool repivotGravity() const { return host_gravity_est && cfg.frames.gnss_updates_gravity; }

  // Effective GPS Klobuchar: RINEX BRDC GPSA/GPSB (A38), else the UBX subframe set (A45), else none (I = 0, σ_ion = 5 m).
  const Klobuchar* iono(Klobuchar* buf) const {
    if (ephem && ephem->iono(buf) && buf->valid()) return buf;
    if (have_ubx_iono && ubx_iono.valid()) {
      *buf = ubx_iono;
      return buf;
    }
    return nullptr;
  }

  // Receiver clock b̂ [m] at the receiver tag of a new epoch, for the time map only (|b| < c·1 ms, DESIGN §1).
  double clockHint() const {
    if (active && aug && aug->has(Slot::Clk)) return aug->clockBias();
    return have_spp ? last_spp.b_m : 0.0;
  }

  double tEstOf(const Pending& p) const {
    if (p.is_pvt) return tmap->toEstimator(p.pvt.t);
    double t = tmap->receiverToEstimator(p.obs.t_rx, clockHint());
    if (p.obs.source == Source::Rinex) t += cfg.rinex.latency_s;   // §12.4: a file epoch enters at t_est + latency
    return t;
  }

  LinPoint linPoint(const CoreState& x, const AugBlock& a, const BridgeResult& br) const {
    LinPoint lp;
    lp.x = &x;
    lp.bridge = &br;
    lp.aug = &a;
    lp.leveling = &leveling;
    lp.anchor = &anchor;
    lp.lever_I = lever_I;
    lp.pivot_W = pivot_W;
    lp.gravity_est_en = host_gravity_est;
    lp.exposure_est_en = host_expo_est;
    lp.delta = br.delta;
    return lp;
  }

  // A bridge of zero length at the host's current state (global pose / antenna queries, clone of the present).
  static BridgeResult nowBridge(const CoreState& x) {
    BridgeResult br;
    br.ok = true;
    br.delta = 0.0;
    br.R_k = x.R;
    br.bg_snap = x.bg;
    br.ba_snap = x.ba;
    br.g_snap = x.g;
    br.R_g_hat = x.R;
    br.p_g_hat = x.p;
    br.v_g_hat = x.v;
    return br;
  }

  int slotIndex(Slot kind, int tag) const {
    const auto& s = aug->slots();
    for (size_t k = 0; k < s.size(); ++k)
      if (s[k].kind == kind && s[k].tag == tag) return static_cast<int>(k);
    return -1;
  }
  void removeSlotIf(Slot kind, int tag) {
    const int k = slotIndex(kind, tag);
    if (k >= 0) aug->removeSlot(k);
  }
  void removeAllOf(Slot kind) {
    for (int k = static_cast<int>(aug->slots().size()) - 1; k >= 0; --k)
      if (aug->slots()[k].kind == kind) aug->removeSlot(k);
  }

  double slotSigma(Slot kind, int tag = 0, int comp = 0) const {
    const int o = aug ? aug->find(kind, tag) : -1;
    return o < 0 ? 0.0 : sqrtPos(aug->Q()(o + comp, o + comp));
  }
  double transSigma() const {
    const int o = aug ? aug->find(Slot::Trans) : -1;
    if (o < 0) return 0.0;
    return sqrtPos(aug->Q().block<3, 3>(o, o).diagonal().maxCoeff());
  }

  // =========================================================================================
  // per-epoch geometry at an iterate
  // =========================================================================================
  struct EpochGeo {
    AntennaJacobians J;
    Eigen::Vector3d r_A = Eigen::Vector3d::Zero(), v_A = Eigen::Vector3d::Zero();
    Geodetic rcv;
    std::vector<SatGeometry> geo;   // aligned with ep.sats; state.ok = false where unavailable
    std::vector<char> ok;
  };

  // `fallback` (nullable, the prior's geometry): only the satellites that had a geometry at the prior are evaluated, and
  // a later iterate that fails for one of them reuses the prior geometry, so the row structure is the same at every
  // iterate (JointUpdater keeps the iteration-0 membership only for an identical structure).
  void epochGeometry(const ObsEpoch& ep, const CoreState& x, const AugBlock& a, const BridgeResult& br, const Klobuchar* ion,
                     const EpochGeo* fallback, EpochGeo* out) const {
    const LinPoint lp = linPoint(x, a, br);
    out->J = rows->antennaJacobians(lp);
    out->r_A = anchor.r0_ecef + anchor.R_EN * out->J.a_N;
    out->v_A = anchor.R_EN * out->J.w_N;
    out->rcv = rtk::ecef2pos(out->r_A);
    const int ic = a.find(Slot::Clk);
    const double b_g = ic >= 0 ? a.x()(ic) - br.delta * a.x()(ic + 1) : 0.0;
    const size_t n = ep.sats.size();
    out->geo.assign(n, SatGeometry());
    out->ok.assign(n, 0);
    for (size_t i = 0; i < n; ++i) {
      if (fallback && !fallback->ok[i]) continue;
      SatGeometry g;
      if (rows->geometry(ep.sats[i], *solver, out->r_A, out->rcv, ion, ep.t_rx, b_g, &g)) {
        out->geo[i] = g;
        out->ok[i] = 1;
      } else if (fallback) {
        out->geo[i] = fallback->geo[i];
        out->ok[i] = 1;
      }
    }
  }

  // L0 membership of every satellite (§10 L0) with the geometry's elevation; GLONASS channel cross-check (§12.3).
  void masksFor(const ObsEpoch& ep, const std::vector<SatGeometry>& geo, const std::vector<char>& ok, bool el_known,
                std::vector<MaskVerdict>* masks, bool count = true) {
    masks->assign(ep.sats.size(), MaskVerdict());
    for (size_t i = 0; i < ep.sats.size(); ++i) {
      const SatObs& so = ep.sats[i];
      const SignalObs* sig = so.primary();
      MaskInput mi;
      mi.obs = &so;
      mi.has_eph = ok[i] && geo[i].state.ok;
      mi.el_deg = el_known && mi.has_eph ? geo[i].el_rad * 180.0 / M_PI : 90.0;
      mi.az_deg = mi.has_eph ? geo[i].az_rad * 180.0 / M_PI : 0.0;
      mi.svh = mi.has_eph ? geo[i].state.svh : -1;
      mi.eph_age_s = mi.has_eph ? std::fabs(ep.t_rx.diff(geo[i].state.toe)) : 0.0;
      mi.psr_track_ok = track->psrOk(so.sat);
      mi.cp_track_ok = track->cpOk(so.sat);
      mi.is_bds_geo = QualityGate::isBdsGeo(so.sat);
      MaskVerdict v = gate->evaluate(mi);
      if (livgoFull() && liv.primary_only && sig && !livgoIsPrimarySignal(so.sat.sys, sig->code)) {   // A80: main frequency only
        v.psr = v.dopp = v.cp = false;
        v.reasons |= kMaskCodeUnknown;
      }
      if (so.sat.sys == Sys::GLO && mi.has_eph && sig && !ephem->gloFcnConsistent(so.sat, sig->freq_hz)) {
        v.psr = v.dopp = v.cp = false;
        v.reasons |= kMaskNoEph;
        if (count) ++tot.n_glo_fcn;
      }
      if (count && (v.reasons & kMaskBdsB1C)) ++tot.n_bds_b1c;
      if (count && (v.reasons & kMaskGalPrn)) ++tot.n_gal_prn;
      (*masks)[i] = v;
    }
  }

  // SppInput rows for the GNSS-only estimators (L1, COLLECT, clock-only LS); `idx` maps back to ep.sats.
  void sppInputs(const ObsEpoch& ep, const std::vector<SatGeometry>& geo, const std::vector<char>& ok,
                 const std::vector<MaskVerdict>& masks, std::vector<SppInput>* in, std::vector<int>* idx) const {
    in->clear();
    idx->clear();
    for (size_t i = 0; i < ep.sats.size(); ++i) {
      if (!ok[i] || !geo[i].state.ok) continue;
      const SatObs& so = ep.sats[i];
      const SignalObs* sig = so.primary();
      if (!sig || !(sig->freq_hz > 0.0)) continue;
      const SatGeometry& g = geo[i];
      SppInput s;
      s.sat = so.sat;
      s.state = g.state;
      const double cb = rtk::codeBiasMetres(so.sat, g.state, sig->code);
      s.psr_usable = masks[i].psr && sig->psrValid() && std::isfinite(cb);
      s.P_corr = s.psr_usable ? sig->psr_m - cb : 0.0;
      s.dopp_hz = sig->dopp_hz;
      s.lambda_m = kCLight / sig->freq_hz;
      s.dopp_usable = masks[i].dopp && sig->doppValid();
      s.cn0 = sig->cn0_dbhz;
      s.el_rad = g.el_rad;
      s.az_rad = g.az_rad;
      s.ion_m = g.ion_m;
      s.trop_m = g.trop_m;
      NoiseModel::PsrTerms pt;
      pt.el_rad = g.el_rad;
      pt.cn0 = sig->cn0_dbhz;
      pt.psr_std_m = sig->psr_std_m;
      pt.sys = so.sat.sys;
      pt.bds_geo = QualityGate::isBdsGeo(so.sat);
      pt.var_eph = g.state.var_eph;
      pt.var_ion = g.var_ion;
      pt.var_trop = g.var_trop;
      pt.dt_since_last_s = 1.0;
      s.var_psr = noise->psrVar(pt);
      NoiseModel::DoppTerms dt;
      dt.el_rad = g.el_rad;
      dt.lambda_m = s.lambda_m;
      dt.dopp_std_hz = sig->dopp_std_hz;
      s.var_dopp = noise->doppVar(dt);
      in->push_back(s);
      idx->push_back(static_cast<int>(i));
    }
  }

  // =========================================================================================
  // ingestion
  // =========================================================================================
  void enqueue(Pending&& p) {
    // §10 L0 "历元时间单调/去重": an epoch at or before the last consumed one, or a second copy of a queued one, is dropped.
    const GTime& last = p.is_pvt ? last_consumed_pvt : last_consumed_obs;
    if (last.valid() && !(last < p.t)) {
      emit(EventKind::LATE_EPOCH, t_k_last, 0.0, p.t, 0.0, p.is_pvt ? "pvt not after the last consumed one" : "epoch not after the last consumed one");
      return;
    }
    for (const Pending& q : queue) {
      if (q.is_pvt == p.is_pvt && q.t == p.t) {
        emit(EventKind::LATE_EPOCH, t_k_last, 0.0, p.t, 0.0, "duplicate epoch");
        return;
      }
    }
    p.seq = ++arrival_seq;
    auto it = std::upper_bound(queue.begin(), queue.end(), p, [](const Pending& a, const Pending& b) {
      const double d = a.t.diff(b.t);
      return d < 0.0 || (d == 0.0 && a.seq < b.seq);
    });
    queue.insert(it, std::move(p));
  }

  // Pops every queued measurement available at t_k (t_est ≤ t_k), in GPST order. A measurement not yet due blocks the
  // later ones so that the processing order stays the GPST order.
  std::vector<Pending> popDue(double t_k) {
    std::vector<Pending> out;
    size_t k = 0;
    for (; k < queue.size(); ++k) {
      if (tEstOf(queue[k]) > t_k + kTimeSlackS) break;
    }
    out.reserve(k);
    for (size_t i = 0; i < k; ++i) out.push_back(std::move(queue[i]));
    queue.erase(queue.begin(), queue.begin() + static_cast<long>(k));
    return out;
  }

  // =========================================================================================
  // cycle
  // =========================================================================================
  void beginStep(CoreState& core, const HostMat& F_tot, const std::vector<BridgeSample>& samples, double t_prev, double t_k,
                 StepReport* rep) {
    if (!lever_warned && cfg.frames.lever_arm_frame == Config::Frames::LeverArmFrame::Lidar && !extrinsics_set) {
      lever_warned = true;
      emit(EventKind::CONFIG_WARNING, t_k, 0.0, GTime(), 0.0,
           "lever_arm_frame = lidar but setExtrinsics() was never called: lever arm taken as given in the IMU frame");
    }
    if (!leveling.hasRef()) leveling.setRef(core.g);
    // Bridge ring (§4.1): the samples were propagated with the biases/gravity of core (propagation does not change them).
    try {
      bridge->pushCycle(samples, t_prev, t_k, core.bg, core.ba, core.g);
    } catch (const std::invalid_argument& e) {
      // Non-contiguous cycle (a skipped LIO step in the glue): late epochs cannot be chained across the gap. Restart
      // the ring with this cycle; a malformed cycle leaves the ring empty (its epochs become LATE_EPOCH).
      bridge.reset(new ImuBridge(cfg));
      emit(EventKind::CONFIG_WARNING, t_k, 0.0, GTime(), 0.0, std::string("bridge ring restarted: ") + e.what());
      try {
        bridge->pushCycle(samples, t_prev, t_k, core.bg, core.ba, core.g);
      } catch (const std::invalid_argument&) {
      }
    }
    // WAIT_CORE (§8): a LIO cycle with IMU samples implies the IMU is initialised; the map exists once the host has
    // reported an H2(LIO) (its early-return paths never call it).
    init->onLioFrame(true, host_lio_updates > 0);
    if (init->phase() == Phase::WAIT_CORE && !livgo_hist.empty()) livgo_hist.clear();   // A80: the host lost its W frame
    if (phase == Phase::WAIT_CORE && init->coreReady()) {
      syncInitPhase(Phase::WAIT_NAV);
      setPhase(Phase::WAIT_NAV, t_k, GTime(), "");
    }
    // Degeneracy floor (§10 last row; probe arm T+floor only, floor_enable default off).
    if (active && !observe() && cfg.integrity.degeneracy.floor_enable && integ->flags().degenerate) {
      const Eigen::Vector3d u = integ->flags().u1;
      if (u.norm() > 0.5) {
        const Eigen::Vector3d un = u.normalized();
        Eigen::Matrix3d Ppp = core.P.block<3, 3>(HostIdx::pos, HostIdx::pos);
        const double var_u = un.dot(Ppp * un);
        const double floor2 = cfg.integrity.degeneracy.sigma_floor_m * cfg.integrity.degeneracy.sigma_floor_m;
        if (var_u < floor2) {
          core.P.block<3, 3>(HostIdx::pos, HostIdx::pos) += (floor2 - var_u) * un * un.transpose();
          rep->core_modified = true;
        }
      }
    }
    // A61 F1: host path since the first LIO cycle (the map's origin), for the tilt prior of an activation.
    for (size_t i = 1; i < samples.size(); ++i) map_path_m += (samples[i].p - samples[i - 1].p).norm();
    // H1 (§2.4): Φ_G, Q_G(Δs, Δα, Δt) from the bridge samples, then C ← F_tot C Φ_Gᵀ.
    if (aug->dim() > 0) {
      double ds = 0.0, dalpha = 0.0;
      const Eigen::Vector3d up = -core.g.normalized();
      for (size_t i = 1; i < samples.size(); ++i) {
        const double dt = samples[i].t - samples[i - 1].t;
        ds += (samples[i].p - samples[i - 1].p).norm();
        // yaw path: vertical component of the world angular rate of the interval (t_{i−1}, t_i] (constant ω_i)
        if (dt > 0.0) dalpha += std::fabs((samples[i - 1].R * samples[i].omega).dot(up)) * dt;
      }
      aug->setWindupEpochDt(epoch_dt);   // A32
      const Eigen::Matrix3d M_reg = leveling.M(aug->yaw(), aug->tilt(), core.g);
      const Eigen::Vector3d a_W = core.p + core.R * lever_I;
      Eigen::MatrixXd Phi, Qp;
      aug->processModel(ds, dalpha, t_k - t_prev, M_reg, a_W, pivot_W, kCLight / kFreqL1, dalpha, &Phi, &Qp);
      aug->propagate(F_tot, Phi, Qp);
      if (pair_open) tot.tdcp_open_s += t_k - t_prev;
      tot.active_s += t_k - t_prev;
    }
    t_k_last = t_k;
  }

  void processBatch(CoreState& core, double t_k, std::vector<Pending>& batch, StepReport* rep) {
    book = Book();
    for (Pending& p : batch) {
      if (p.is_pvt) {
        last_consumed_pvt = p.t;
        consumePvt(core, p.pvt, t_k, rep);
      } else {
        last_consumed_obs = p.t;
        consumeObs(core, p.obs, t_k, rep);
      }
    }
    if (active) outageStep(t_k);
  }

  // =========================================================================================
  // observation epochs
  // =========================================================================================
  void consumeObs(CoreState& core, ObsEpoch& ep, double t_k, StepReport* rep) {
    const Clock::time_point t0 = Clock::now();
    ++epoch_index;
    ++tot.epochs_total;
    ++rep->epochs_processed;
    // bookkeeping shared by every phase: arcs, std units, epoch interval (A32)
    track->tick(ep);
    if (!std_judge.decided()) std_judge.observe(ep);
    std_judge.apply(&ep);
    if (last_epoch_t.valid()) {
      const double dt = ep.t_rx.diff(last_epoch_t);
      if (dt > 0.0) {
        intervals.push_back(dt);
        if (intervals.size() > kIntervalWindow) intervals.pop_front();
        if (intervals.size() >= 3) epoch_dt = median(std::vector<double>(intervals.begin(), intervals.end()));
      }
    }
    last_epoch_t = ep.t_rx;
    cfg_rows.time.sigma_toff_s = tmap->sigmaToff(ep.t_rx);   // σ_toff of this epoch's time-map segment (§4.7)
    // A61 F12: with a td slot (time.estimate_online) the offset is a state whose columns (−w^N, −α^N) every row carries;
    // the σ_toff terms of R would count it a second time, and — the offset being one constant — as if it were new noise
    // on every row. σ_toff remains the slot's prior.
    if (aug && aug->has(Slot::Td)) cfg_rows.time.sigma_toff_s = 0.0;
    if (!origin_checked && tmap) {
      origin_checked = true;
      std::string why;
      if (!TimeMap::checkTimeOrigin(tmap->segments(), ep.t_rx, ep.t_rx, &why))
        emit(EventKind::CONFIG_WARNING, t_k, 0.0, ep.t_rx, 0.0, "time map: " + why);
    }
    if (!tdcp_paused_reported && tmap->tdcpPaused(ep.t_rx)) {
      tdcp_paused_reported = true;
      emit(EventKind::TDCP_PAUSED, t_k, 0.0, ep.t_rx, 0.0, "time calibration segment marks TDCP paused (§7)");
    }
    if (!freq_checked && phase != Phase::WAIT_CORE) freqCrossCheck(ep, t_k);

    EpochRecord rec;
    rec.gpst = weekTow(ep.t_rx);
    rec.t_gpst = ep.t_rx.toSec();
    rec.t_k = t_k;
    rec.phase = phase;
    rec.observe_only = observe();
    rec.n_total_raw = static_cast<int>(ep.sats.size());

    if (phase == Phase::WAIT_CORE) {
      rec.t_est = tmap->receiverToEstimator(ep.t_rx, clockHint());
    } else if (phase == Phase::WAIT_NAV) {
      rec.t_est = tmap->receiverToEstimator(ep.t_rx, clockHint());
      int n = 0;
      for (const SatObs& so : ep.sats) n += ephem->healthy(so.sat, ep.t_rx, nullptr) ? 1 : 0;
      init->onNavCount(n);
      // AMENDMENTS A51 (P4-INIT): the initializer decides when COLLECT starts (one healthy satellite: the LIO-aided path
      // uses epochs of 1–3 satellites; the §8 window still takes only SPP samples with ≥ init.min_sats).
      if (init->phase() == Phase::COLLECT) {
        setPhase(Phase::COLLECT, t_k, ep.t_rx, std::to_string(n) + " satellites with healthy ephemeris");
        collectEpoch(core, ep, t_k, rec, rep);
      }
    } else if (phase == Phase::COLLECT || phase == Phase::ALIGN) {
      collectEpoch(core, ep, t_k, rec, rep);
    } else {
      activeEpoch(core, ep, t_k, rec, rep);
    }
    last_epoch_t_est = rec.t_est;
    rec.elapsed_ms = msSince(t0);
    if (active || rec.committed || rec.rolled_back) tot.elapsed.push_back(rec.elapsed_ms);
    fillStateColumns(core, &rec);
    if (sink) sink->onEpoch(rec);
  }

  // AMENDMENTS A35-C8: the one-time check that the delivered carrier frequencies agree with RTKLIB sat2freq (the λ the
  // rows use comes from the message; a mismatch means a mapping error upstream, not a measurement). Done on the first
  // epoch that has satellites with a known frequency in the store; reported once.
  void freqCrossCheck(const ObsEpoch& ep, double t_k) {
    int checked = 0, bad = 0;
    std::string which;
    for (const SatObs& so : ep.sats) {
      const SignalObs* sig = so.primary();
      if (!sig || !(sig->freq_hz > 0.0)) continue;
      const double f = rtk::sat2freq(so.sat.rtk_sat, sig->code, *nav);
      if (!(f > 0.0)) continue;
      ++checked;
      if (std::fabs(f - sig->freq_hz) > 1e3) {
        ++bad;
        if (which.size() < 64) which += " " + satIdStringOf(so.sat);
      }
    }
    if (checked == 0) return;
    freq_checked = true;
    if (bad > 0)
      emit(EventKind::CONFIG_WARNING, t_k, 0.0, ep.t_rx, bad,
           "sat2freq disagrees with the delivered frequency on " + std::to_string(bad) + "/" + std::to_string(checked) +
               " satellites (A35-C8):" + which);
  }

  // Registration / clock / host columns of an epoch record (after the epoch's processing).
  void fillStateColumns(const CoreState& core, EpochRecord* r) const {
    r->degenerate = integ ? integ->flags().degenerate : false;
    r->livo_alarm = integ ? integ->flags().livo_alarm : false;
    r->schmidt = active && r->livo_alarm;
    if (integ) {
      r->lambda = integ->flags().lambda;
      r->u1 = integ->flags().u1;
    }
    for (int i = 0; i < HostIdx::dim; ++i) r->host_sigma(i) = sqrtPos(core.P(i, i));
    r->isb_C = r->isb_E = r->isb_R = kNaN;
    r->td = r->td_sigma = kNaN;
    r->n_tdcp_pairs_open = pair_open ? 1 : 0;
    r->psr_decim_skipped = psr_decim_skipped;
    r->pvt_decim_skipped = pvt_decim_skipped;
    if (!aug || aug->dim() == 0) return;
    const int ot = aug->find(Slot::Tilt), oy = aug->find(Slot::Yaw), oz = aug->find(Slot::Trans), oc = aug->find(Slot::Clk);
    const Eigen::VectorXd& x = aug->x();
    const Eigen::MatrixXd& Q = aug->Q();
    if (ot >= 0) {
      r->tilt = x.segment<2>(ot);
      r->tilt_sigma = Eigen::Vector2d(sqrtPos(Q(ot, ot)), sqrtPos(Q(ot + 1, ot + 1)));
    }
    if (oy >= 0) {
      r->yaw = x(oy);
      r->yaw_sigma = sqrtPos(Q(oy, oy));
    }
    if (oz >= 0) {
      r->trans = x.segment<3>(oz);
      for (int k = 0; k < 3; ++k) r->trans_sigma(k) = sqrtPos(Q(oz + k, oz + k));
    }
    if (oc >= 0) {
      r->clk_b = x(oc);
      r->clk_b_sigma = sqrtPos(Q(oc, oc));
      r->clk_d = x(oc + 1);
      r->clk_d_sigma = sqrtPos(Q(oc + 1, oc + 1));
    }
    double v = 0.0, s = 0.0;
    if (aug->isb(Sys::BDS, &v, &s)) r->isb_C = v;
    if (aug->isb(Sys::GAL, &v, &s)) r->isb_E = v;
    if (aug->isb(Sys::GLO, &v, &s)) r->isb_R = v;
    const int od = aug->find(Slot::Td);
    if (od >= 0) {
      r->td = x(od);
      r->td_sigma = sqrtPos(Q(od, od));
    }
  }

  void countSystems(const ObsEpoch& ep, const std::vector<MaskVerdict>& masks, EpochRecord* r) {
    int n_valid = 0;
    for (size_t i = 0; i < ep.sats.size() && i < masks.size(); ++i) {
      if (!(masks[i].psr || masks[i].dopp || masks[i].cp)) continue;
      ++n_valid;
      switch (ep.sats[i].sat.sys) {
        case Sys::GPS: ++r->n_gps; break;
        case Sys::GLO: ++r->n_glo; break;
        case Sys::GAL: ++r->n_gal; break;
        case Sys::BDS: ++r->n_bds; break;
        case Sys::QZS: ++r->n_qzs; break;
        default: break;
      }
    }
    n_sats_last = n_valid;
  }

  // ---------------- COLLECT (§8) ----------------
  void collectEpoch(CoreState& core, const ObsEpoch& ep, double t_k, EpochRecord& rec, StepReport* rep) {
    Klobuchar ion_buf;
    const Klobuchar* ion = iono(&ion_buf);
    // Receiver position for the elevation mask: the last GNSS-only fix (none before the first one, then the SPP is
    // solved twice: without and with the elevation mask).
    const bool hint = have_spp;
    const Eigen::Vector3d r_hint = hint ? last_spp.r_ecef : Eigen::Vector3d::Zero();
    const double b_hint = hint ? last_spp.b_m : 0.0;
    std::vector<SatGeometry> geo(ep.sats.size());
    std::vector<char> ok(ep.sats.size(), 0);
    auto geometryAt = [&](const Eigen::Vector3d& r, bool known) {
      const Geodetic rcv = known ? rtk::ecef2pos(r) : Geodetic();
      for (size_t i = 0; i < ep.sats.size(); ++i) {
        ok[i] = 0;
        const SatObs& so = ep.sats[i];
        const SignalObs* sig = so.primary();
        if (!sig || !sig->psrValid()) continue;   // COLLECT is pseudorange-driven: the transmit time needs P
        SatGeometry g;
        if (known) {
          if (!rows->geometry(so, *solver, r, rcv, ion, ep.t_rx, b_hint, &g)) continue;
        } else {
          if (!solver->satState(so.sat, ep.t_rx, sig->psr_m, &g.state) || !g.state.ok) continue;
          g.sat = so.sat;
          g.el_rad = M_PI / 2.0;
          g.lambda_m = sig->freq_hz > 0.0 ? kCLight / sig->freq_hz : 0.0;
          g.var_ion = 25.0;
          g.var_trop = 0.0;
        }
        geo[i] = g;
        ok[i] = 1;
      }
    };
    geometryAt(r_hint, hint);
    std::vector<MaskVerdict> masks;
    masksFor(ep, geo, ok, hint, &masks, /*count=*/hint);
    std::vector<SppInput> in;
    std::vector<int> idx;
    sppInputs(ep, geo, ok, masks, &in, &idx);
    RobustSpp::nlosPrescreen(in, cfg.init.nlos_cn0_margin_db);   // §8 COLLECT NLOS pre-screen
    SppSolution sol;
    bool sok = spp->solve(ep.t_rx, in, ion, hint, r_hint, b_hint, &sol);
    if (sok && !hint) {
      geometryAt(sol.r_ecef, true);
      masksFor(ep, geo, ok, true, &masks);
      sppInputs(ep, geo, ok, masks, &in, &idx);
      RobustSpp::nlosPrescreen(in, cfg.init.nlos_cn0_margin_db);
      SppSolution s2;
      sok = spp->solve(ep.t_rx, in, ion, true, sol.r_ecef, sol.b_m, &s2);
      sol = s2;
    }
    countSystems(ep, masks, &rec);
    DopplerSolution dop;
    if (sok) {
      dls->solve(ep.t_rx, in, sol.r_ecef, &dop);
      last_spp = sol;
      have_spp = true;
    }
    rec.spp_valid = sok;
    // FAST-LIVGO: one DTW on the speeds collected so far, before this epoch's estimator time is read.
    // The call is absent from T's path (the enable test), so this function's off-arm body stays a branch
    // the compiler can leave out of the numeric schedule when the flag is a constant false. It is not.
    // A82: with livgo.dtw_at_init the DTW is made once at the travel gate (livgoDtwAtInit below), not after dtw_min_epochs epochs.
    if (dtwOnline() && !dtwAtInit()) livgoTryDtw(t_k, ep.t_rx);   // A75: livgo.dtw_online = false keeps the offline calibration only
    const double t_est = tmap->receiverToEstimator(ep.t_rx, sok ? sol.b_m : b_hint);
    rec.t_est = t_est;
    if (ep.t_record != 0.0) rec.arrival_latency = ep.t_record - t_est;
    BridgeResult br;
    if (!bridge->query(std::min(t_est, t_k), &br)) {
      lateEpoch(ep, t_est, t_k, &rec);
      return;
    }
    rec.delta = br.delta;
    rec.cross_cycle_bridge = br.cross_cycle;
    // AMENDMENTS A51 (P4-INIT): every epoch with a bridge and at least one prepared row goes to the initializer, SPP or
    // not; it keeps the §8 window (SPP ok, ≥ init.min_sats, GDOP < init.max_gdop) and the LIO-aided window itself. The
    // LIO-aided path re-evaluates elevation, atmosphere and the §4.7 variances at its own antenna positions, so it gets
    // the Klobuchar in force and the receiver stds of each row's primary signal (aligned with `in` through `idx`).
    // A82 livgo.dtw_at_init: every epoch with a bridge leaves the host pose it sees in the pose track (also an epoch with no usable satellite,
    // so the track has no holes the history's epochs do not have); a DTW lag re-pairs the history with this track. Until the one DTW of
    // the run has been made.
    if (dtwAtInit() && !dtw_init_done) {
      const AntennaAtEpoch ha = ImuBridge::evaluate(core, br, lever_I, host_gravity_est);
      LivgoHostPose hp;
      hp.t = t_est;
      hp.R = ha.R_g;
      hp.p = ha.p_g;
      hp.v = ha.v_g;
      livgo_hist.addTrack(hp);
    }
    if (in.empty()) return;
    CollectSample s;
    s.t_rx = ep.t_rx;
    s.t_est = t_est;
    s.spp = sol;
    s.dop = dop;
    s.raw = in;
    s.host_ant = ImuBridge::evaluate(core, br, lever_I, host_gravity_est);
    if (dtwOnline() && !dtwAtInit() && !dtw_applied && dop.ok && s.host_ant.z.allFinite() && dop.v_ecef.allFinite()) {
      dtw_tg.push_back(t_est);
      dtw_vg.push_back(dop.v_ecef.norm());
      dtw_th.push_back(t_est);
      dtw_vh.push_back(s.host_ant.z.norm());
    }
    s.bridge = br;
    if (ion) {
      s.ion = *ion;
      s.ion_valid = true;
    }
    s.psr_std_m.reserve(idx.size());
    s.dopp_std_hz.reserve(idx.size());
    for (int i : idx) {
      const SignalObs* sig = ep.sats[static_cast<size_t>(i)].primary();
      s.psr_std_m.push_back(sig ? sig->psr_std_m : 0.0);
      s.dopp_std_hz.push_back(sig ? sig->dopp_std_hz : 0.0);
    }
    // A80 init_travel_m: the synchronous epoch (SPP position + host pose at its time) joins the history the frame least squares will
    // run on. The §8 SPP test is the initializer's own (SPP ok, init.min_sats, init.max_gdop, a bridge, a finite antenna).
    if (travelInit() && s.spp.ok && s.spp.n_used >= cfg.init.min_sats && s.spp.gdop < cfg.init.max_gdop && s.bridge.ok &&
        s.host_ant.p_g.allFinite() && s.host_ant.R_g.allFinite()) {
      LivgoInitEpoch he;
      he.p_I = s.host_ant.p_g;
      he.R_I = s.host_ant.R_g;
      he.r_ecef = s.spp.r_ecef;
      // A82: what the DTW at the frame initialisation needs of the epoch (the same two speed magnitudes the online DTW uses; NaN when the
      // epoch has no Doppler solution, and such an epoch is then not in the DTW's series but stays in the least squares).
      he.t_est = t_est;
      if (dop.ok && dop.v_ecef.allFinite()) he.v_gnss = dop.v_ecef.norm();
      if (s.host_ant.z.allFinite()) he.v_host = s.host_ant.z.norm();
      livgo_hist.add(he, core.g);
    }
    init->addSample(std::move(s));
    // ALIGN (§8): the initializer moves COLLECT → ALIGN itself once the window is ready, the motion sufficient and its
    // retry wait after a failed attempt has run out (INTERFACE_CHANGES P3-A1); the facade only acts on that decision and
    // never overrides the initializer's phase around an attempt (setPhase() would reset that wait). The attempt runs
    // in this LIO cycle with `core` at its t_k; the facade's own phase stays COLLECT until it succeeds, and a failure is
    // logged once per distinct reason.
    if (init->phase() != Phase::ALIGN) return;
    // A80 init_travel_m: the initializer keeps collecting (it stays in ALIGN and its windows slide); no attempt is made, and no
    // window is copied, until the host has moved init_travel_m horizontally.
    if (travelInit() && livgo_hist.travelM() < liv.init_travel_m) return;
    // A82 livgo.dtw_at_init: the gate is open. The paper's order: DTW on the history of the least squares, compensate, then the least
    // squares. Once per run (an attempt that fails below and is retried at the next epoch does not repeat it).
    if (dtwAtInit() && !dtw_init_done) livgoDtwAtInit(t_k, ep.t_rx);
    if (!leveling.hasRef()) leveling.setRef(core.g);
    if (livgoFull()) {
      livgo_snap_spp = init->sppWindow();
      livgo_snap_la = init->laWindow();
    }
    const AlignResult a = init->tryAlign(core, tmap->offset(ep.t_rx), tmap->sigmaToff(ep.t_rx));
    if (a.offset_warn)
      emit(EventKind::TOFF_WARN, t_k, t_est, ep.t_rx, a.offset_check_s, "ALIGN: online offset check disagrees with the offline calibration");
    if (!a.ok) {
      if (a.reason != last_align_reason) emit(EventKind::PHASE_CHANGE, t_k, t_est, ep.t_rx, 0.0, "ALIGN failed: " + a.reason);
      last_align_reason = a.reason;
      return;
    }
    last_align_reason.clear();
    activateFrom(a, core, t_k, ep.t_rx, rep);
  }

  bool activateFrom(const AlignResult& a_in, const CoreState& core, double t_k, const GTime& g, StepReport* rep) {
    std::string why;
    aug->clear();
    AlignResult a = a_in;
    a.map_path_m = map_path_m;   // A61 F1
    const bool ok = init->activate(a, core, aug.get(), &why);
    if (ok && (!aug->has(Slot::Yaw) || !aug->has(Slot::Trans))) why = "activate() left no yaw/trans slot";
    if (ok && !anchor.valid && !a.anchor.valid) why = "alignment carries no valid anchor";
    if (!ok || !why.empty()) {
      aug->clear();
      init->setPhase(Phase::COLLECT);
      setPhase(Phase::COLLECT, t_k, g, "activate: " + why);
      return false;
    }
    if (!anchor.valid) {
      anchor = a.anchor;
    } else if (a.anchor.valid && (a.anchor.r0_ecef != anchor.r0_ecef || a.anchor.R_EN != anchor.R_EN)) {
      reexpressInAnchor(a.anchor);   // N is fixed at the first ACTIVATE and never changes (§1)
    }
    pivot_W = a.pivot_W;
    if (!tight()) {
      // L-PVT arm (§4.6): no clock slots.
      removeAllOf(Slot::Isb);
      removeSlotIf(Slot::Clk, 0);
    } else {
      if (!aug->has(Slot::Clk)) {
        // The alignment's clock block (b, d) when activate() did not write it: independent of the rest.
        const double var_b = (a.cov_clock_isb.rows() > 0 && a.cov_clock_isb(0, 0) > 0.0) ? a.cov_clock_isb(0, 0) : 1e4;
        const double var_d = a.var_d > 0.0 ? a.var_d : 1.0;
        Eigen::Matrix2d N = Eigen::Matrix2d::Zero();
        N(0, 0) = var_b;
        N(1, 1) = var_d;
        aug->addSlot(Slot::Clk, 0, "clk", Eigen::Vector2d(a.b, a.d), Eigen::MatrixXd::Zero(2, HostIdx::dim),
                     Eigen::MatrixXd::Zero(2, aug->dim()), N, core.P);
        emit(EventKind::CONFIG_WARNING, t_k, 0.0, g, 0.0, "activate() left no clock slot: added from the alignment");
      }
      ensureIsbSlots(core);
    }
    if (cfg.time.estimate_online && !aug->has(Slot::Td)) {
      Eigen::MatrixXd N(1, 1);
      N(0, 0) = cfg.time.sigma_toff_s * cfg.time.sigma_toff_s;
      aug->addSlot(Slot::Td, 0, "td", Eigen::VectorXd::Zero(1), Eigen::MatrixXd::Zero(1, HostIdx::dim),
                   Eigen::MatrixXd::Zero(1, aug->dim()), N, core.P);
    }
    if (livgoFull() && liv.paper_init) livgoApplyPaperInit(a, core);
    else if (livgoFull() && aug) {
      // Control: the pre-LIVGO activate() solution. Tilt stays the prior (phi = 0); yaw and the lever are not
      // overwritten by the position least squares.
      const double rad2deg = 180.0 / M_PI;
      const Eigen::Vector2d phi = aug->has(Slot::Tilt) ? aug->tilt() : Eigen::Vector2d::Zero();
      char b[320];
      std::snprintf(b, sizeof b, " | livgo_init=legacy yaw_deg=%.3f tilt_deg=%.3f,%.3f lever=%.3f,%.3f,%.3f",
                    aug->yaw() * rad2deg, phi.x() * rad2deg, phi.y() * rad2deg, lever_I.x(), lever_I.y(), lever_I.z());
      livgo_init_note = b;
    } else if (liv.dual_only) {
      livgo_init_note = " | dual_rej";
    }
    odoOrigin(core);   // A61 F13
    active = true;
    if (livgoFull() && aug) {
      livgo_x_pin = aug->x();
      livgo_pinned = true;
      livgoPin();
    }
    activate_index = epoch_index;
    reacq_left = cfg.iterate.reacq_epochs;
    pending_clock_reset = pending_reanchor = false;
    closePair(false);
    outage->onActivated(t_k);
    init->setPhase(Phase::TRACKING);
    setPhase(Phase::TRACKING, t_k, g, "");
    char d[240];
    std::snprintf(d, sizeof d, "epochs_used=%d sigma_yaw_deg=%.3f slots=%d dim=%d path=%s sigma_t_max_m=%.3f redundancy=%d sats=%d",
                  a.epochs_used, a.sigma_yaw * 180.0 / M_PI, static_cast<int>(aug->slots().size()), aug->dim(),
                  a.path.empty() ? "-" : a.path.c_str(), a.sigma_trans_max_m, a.redundancy, a.n_sats);
    emit(EventKind::ACTIVATE, t_k, 0.0, g, a.sigma_yaw, std::string(d) + livgo_init_note + livgoV2Note() + livgoA80Note() + livgoA82Note());
    reinit_gate.onActivated();   // A82 livgo.single_init: from here on no re-initialisation runs
    // A75 revision 1: the frame-jump widening (A61 F9) is not armed under frame_exact (frameJumpProbe); say so when it was asked for.
    if (frameExact() && cfg.integrity.frame_jump.enable)
      emit(EventKind::CONFIG_WARNING, t_k, 0.0, g, 0.0, "integrity.frame_jump is not armed under livgo/frame_exact (no registration process noise)");
    rep->activated = true;
    smooth_valid = false;
    return true;
  }

  // AMENDMENTS A35: every enabled system with code rows gets its ISB slot (σ = 30 m when unobserved at ACTIVATE).
  void ensureIsbSlots(const CoreState& core) {
    struct S {
      Sys sys;
      bool on;
      const char* name;
    };
    const S list[3] = {{Sys::GAL, cfg.systems.gal, "isb_E"}, {Sys::BDS, cfg.systems.bds, "isb_C"},
                       {Sys::GLO, cfg.systems.glo && cfg.systems.glo_psr, "isb_R"}};
    for (const S& s : list) {
      if (!s.on || aug->has(Slot::Isb, sysBit(s.sys))) continue;
      Eigen::MatrixXd N(1, 1);
      N(0, 0) = kIsbUnobservedSigmaM * kIsbUnobservedSigmaM;
      aug->addSlot(Slot::Isb, sysBit(s.sys), s.name, Eigen::VectorXd::Zero(1), Eigen::MatrixXd::Zero(1, HostIdx::dim),
                   Eigen::MatrixXd::Zero(1, aug->dim()), N, core.P);
    }
  }

  // A re-activation (REINIT) aligned in its own anchor N' is re-expressed in the fixed N: a^N = Q a^N' + d with
  // Q = R_ENᵀ R'_EN, d = R_ENᵀ(r0' − r0) ⇒ M = Q M', t = Q t' + d. R_z(ψ)Exp(φ̃) = Q R_z(ψ')Exp(φ̃') is split back into
  // twist (ψ) and horizontal swing (φ); the covariance is mapped with Q on t and identity on (φ, ψ), exact to first
  // order in |Q − I| (~1e-4 per km between the two anchors).
  void reexpressInAnchor(const AnchorFrame& fresh) {
    const Eigen::Matrix3d Q = anchor.R_EN.transpose() * fresh.R_EN;
    const Eigen::Vector3d d = anchor.R_EN.transpose() * (fresh.r0_ecef - anchor.r0_ecef);
    const int oy = aug->find(Slot::Yaw), oz = aug->find(Slot::Trans), ot = aug->find(Slot::Tilt);
    if (oy < 0 || oz < 0) return;
    Eigen::VectorXd& x = aug->x();
    const Eigen::Vector2d tilt = ot >= 0 ? Eigen::Vector2d(x.segment<2>(ot)) : Eigen::Vector2d::Zero();
    const Eigen::Matrix3d A = Q * Leveling::Rz(x(oy)) * Leveling::expTilt(tilt);
    const double psi = std::atan2(A(1, 0) - A(0, 1), A(0, 0) + A(1, 1));
    const Eigen::Vector3d phi = so3Log(Eigen::Matrix3d(Leveling::Rz(psi).transpose() * A));
    x(oy) = psi;
    if (ot >= 0) x.segment<2>(ot) = phi.head<2>();
    x.segment<3>(oz) = Q * Eigen::Vector3d(x.segment<3>(oz)) + d;
    Eigen::MatrixXd T = Eigen::MatrixXd::Identity(aug->dim(), aug->dim());
    T.block<3, 3>(oz, oz) = Q;
    aug->C() = aug->C() * T.transpose();
    aug->Q() = T * aug->Q() * T.transpose();
    aug->symmetrize();
    aug->wrapYaw();
  }

  // ---------------- late / unbridgeable epoch ----------------
  void lateEpoch(const ObsEpoch& ep, double t_est, double t_k, EpochRecord* rec) {
    const double delta = t_k - t_est;
    rec->delta = delta;
    for (const SatObs& so : ep.sats) {
      const SignalObs* s = so.primary();
      if (!s) continue;
      if (s->psrValid()) {
        ++rec->cls[ci(MeasClass::Psr)].candidates;
        ++rec->cls[ci(MeasClass::Psr)].rejected[ri(RejectReason::Late)];
      }
      if (s->doppValid()) {
        ++rec->cls[ci(MeasClass::Dopp)].candidates;
        ++rec->cls[ci(MeasClass::Dopp)].rejected[ri(RejectReason::Late)];
      }
    }
    for (auto& c : rec->cls) c.skipped = true;
    char d[96];
    std::snprintf(d, sizeof d, "delta=%.4f s (max_bridge_s %.3f)", delta, cfg.time.max_bridge_s);
    emit(EventKind::LATE_EPOCH, t_k, t_est, ep.t_rx, delta, d);
  }

  // =========================================================================================
  // TRACKING / OUTAGE / RECOVERY epoch (§9 inner loop)
  // =========================================================================================
  void activeEpoch(CoreState& core, ObsEpoch& ep, double t_k, EpochRecord& rec, StepReport* rep) {
    const bool obs_only = observe();
    Klobuchar ion_buf;
    const Klobuchar* ion = iono(&ion_buf);
    // ---- time and bridge ----
    const double t_est = tmap->receiverToEstimator(ep.t_rx, clockHint());
    rec.t_est = t_est;
    if (ep.t_record != 0.0) rec.arrival_latency = ep.t_record - t_est;
    BridgeResult br;
    const double t_g = (t_est > t_k && t_est <= t_k + kTimeSlackS) ? t_k : t_est;
    if (!bridge->query(t_g, &br)) {
      lateEpoch(ep, t_est, t_k, &rec);
      return;
    }
    rec.delta = br.delta;
    rec.cross_cycle_bridge = br.cross_cycle;
    book.epoch_arrived = true;

    // ---- prior geometry and L0 ----
    EpochGeo g0;
    epochGeometry(ep, core, *aug, br, ion, nullptr, &g0);
    std::vector<MaskVerdict> masks;
    masksFor(ep, g0.geo, g0.ok, true, &masks);
    countSystems(ep, masks, &rec);
    book.n_valid = std::max(book.n_valid, n_sats_last);
    if (n_sats_last > 0) last_valid_t_est = t_est;
    std::vector<SppInput> in;
    std::vector<int> in_idx;
    sppInputs(ep, g0.geo, g0.ok, masks, &in, &in_idx);
    const int oc = aug->find(Slot::Clk);
    const double b_g = oc >= 0 ? aug->x()(oc) - br.delta * aug->x()(oc + 1) : clockHint();

    // ---- RECOVERY clock reset (§11: σ_b > 1e3 m ⇒ clock-only LS at the predicted antenna) ----
    if (pending_clock_reset && oc >= 0) {
      pending_clock_reset = false;
      if (observe()) {
        emit(EventKind::CLOCK_JUMP, t_k, t_est, ep.t_rx, 0.0, "recovery clock reset: observe_only, not applied");
      } else {
        // P3-A1 (outage.hpp): clock-only LS at the predicted antenna as a consistent reset (e_b, e_d through the mean
        // line of sight), moved to t_k by δ.
        ClockOnlyResult cr;
        std::string why;
        const bool okr = livo_gnss::recoveryClockReset(aug.get(), g0.J, anchor, in, br.delta, cfg, core.P, &cr, &why);
        emit(EventKind::CLOCK_JUMP, t_k, t_est, ep.t_rx, okr ? cr.b_m : 0.0,
             okr ? std::string("recovery clock reset") : "recovery clock reset failed: " + why);
      }
    }

    // ---- L1: GNSS-only robust SPP / Doppler LS (§10 L1) ----
    SppSolution sol;
    const bool sok = spp->solve(ep.t_rx, in, ion, true, g0.r_A, b_g, &sol);
    DopplerSolution dop;
    dls->solve(ep.t_rx, in, g0.r_A, &dop);
    std::set<int> l1_rejected;
    if (sok) {
      for (const SatId& s : sol.rejected) l1_rejected.insert(s.rtk_sat);
      last_spp = sol;
      have_spp = true;
      rec.spp_valid = true;
      rec.spp_minus_filter = anchor.R_EN.transpose() * (sol.r_ecef - g0.r_A);
      if (sol.n_used >= cfg.init.min_sats) book.l1_consistent = true;
    }

    // ---- L6 re-anchor (§10 L6: t and clock re-augmented from the SPP) ----
    if (pending_reanchor && sok) {
      pending_reanchor = false;
      if (obs_only) {
        emit(EventKind::REANCHOR, t_k, t_est, ep.t_rx, 0.0, "observe_only: not applied");
      } else {
        closePair(true);   // P3-A1 precondition: no clone slot
        std::string why;
        const AntennaJacobians Jr = rows->antennaJacobians(linPoint(core, *aug, br));
        if (dop.ok && livo_gnss::reanchorFromSpp(aug.get(), Jr, anchor, sol, dop, in, br.delta, cfg, core.P, &why)) {
          reacq_left = cfg.iterate.reacq_epochs;
          odoOrigin(core);   // A61 F13: t lost its history; the odometry output continues from the last step end
          emit(EventKind::REANCHOR, t_k, t_est, ep.t_rx, 0.0, "L6: trans and clock re-augmented from the SPP");
        } else {
          emit(EventKind::REANCHOR, t_k, t_est, ep.t_rx, 0.0,
               "L6 re-anchor not applied: " + (dop.ok ? why : std::string("no Doppler LS")));
        }
        g0 = EpochGeo();
        epochGeometry(ep, core, *aug, br, ion, nullptr, &g0);
      }
    }

    // ---- L2: clock jump on the median pseudorange innovation (every epoch, §10 L2) ----
    const LinPoint lp0 = linPoint(core, *aug, br);
    if (tight() && aug->has(Slot::Clk)) {
      RowSet ps;
      // L2 needs the code innovations of every epoch, the update only those of the 1 Hz epochs (A43). A scratch
      // RowBuilder per epoch keeps L2 free of any 1 Hz selection state a RowBuilder carries (P3-C's EpochDecimator: a
      // fresh builder takes the epoch it is first given) and keeps L2's calls out of the update builder's state.
      const RowBuilder l2rows(cfg_rows, *noise);
      l2rows.pseudorangeRows(ep, g0.geo, masks, lp0, g0.J, &ps);
      std::vector<double> nus;
      for (const MeasRow& r : ps.rows)
        if (!l1_rejected.count(r.sat.rtk_sat)) nus.push_back(r.nu);
      const ClockJumpVerdict v = jump->detect(nus);
      if (v.jump) {
        const double m = v.k_ms * ClockJumpDetector::kMsInMetres;
        if (pair_open) pair.clock_jump_m += m;   // a fact about the measurements, recorded in every mode
        char d[96];
        std::snprintf(d, sizeof d, "k_ms=%d median_innov_m=%.3f%s", v.k_ms, v.median_innov_m, obs_only ? " observe_only" : "");
        emit(EventKind::CLOCK_JUMP, t_k, t_est, ep.t_rx, v.k_ms, d);
        if (!obs_only) {
          applyIntegerMsJump(aug.get(), v.k_ms);
          // The step moved the true reception instant t_rx − b/c by k ms: re-map this epoch with the corrected b̂ so its
          // rows are built at the right antenna position (later epochs get it through clockHint()).
          BridgeResult br2;
          const double t_est2 = tmap->receiverToEstimator(ep.t_rx, clockHint());
          const double t_g2 = (t_est2 > t_k && t_est2 <= t_k + kTimeSlackS) ? t_k : t_est2;
          if (bridge->query(t_g2, &br2)) {
            br = br2;
            rec.t_est = t_est2;
            rec.delta = br.delta;
            rec.cross_cycle_bridge = br.cross_cycle;
            g0 = EpochGeo();
            epochGeometry(ep, core, *aug, br, ion, nullptr, &g0);
          }
        }
      } else if (v.needs_reset) {
        clockReset(in, g0, br, t_k, t_est, ep.t_rx, "non_integer_jump");
      }
    }

    // ---- which rows this epoch carries ----
    const int64_t bucket = decimBucket(ep.t_rx, cfg.noise.psr.decim_s);
    const bool psr_epoch = tight() && cfg.use.psr && !livgoFull() && bucket > psr_bucket;   // A43; full LIVGO keeps code out of the filter
    if (tight() && cfg.use.psr && !livgoFull() && !psr_epoch) ++psr_decim_skipped;
    if (tight()) psr_bucket = std::max(psr_bucket, bucket);
    const bool dopp_on = tight() && cfg.use.dopp;
    const bool tdcp_on = tight() && cfg.use.tdcp;
    const double since_k0 = pair_open ? ep.t_rx.diff(pair.t_k0) : 0.0;
    if (pair_open && since_k0 > cfg.noise.tdcp.max_span_s) closePair(true);   // k1 never came (gap): park
    // A75 revision 1: a receiver interval above max_span_s parks every pair before its k1 — no TDCP row, and no other trace.
    // Said once, for the LIVGO arms only (the legacy arm keeps the default 1.5 s span, far above any interval of the development set).
    if (livgoFull() && tdcp_on && !span_warned && intervals.size() >= 3 && epoch_dt > cfg.noise.tdcp.max_span_s) {
      span_warned = true;
      char d[160];
      std::snprintf(d, sizeof d, "receiver epoch interval %.3f s exceeds noise.tdcp.max_span_s %.3f s: no TDCP pair can close", epoch_dt,
                    cfg.noise.tdcp.max_span_s);
      emit(EventKind::CONFIG_WARNING, t_k, t_est, ep.t_rx, epoch_dt, d);
    }
    const bool closing = pair_open && since_k0 >= cfg.noise.tdcp.interval_s - 0.5 * epoch_dt;
    if (pair_open && !closing) markIntermediate(ep);
    // A61 F10: with chain_pairs the next pair opens at this pair's k1 (after the update, once the old clone is gone)
    const bool chain = cfg.noise.tdcp.chain_pairs && closing;
    bool starting = tdcp_on && (!pair_open || chain) && epoch_index > activate_index && epoch_index > last_k1_index &&
                    phase != Phase::OUTAGE && !tmap->tdcpPaused(ep.t_rx);
    std::vector<int> start_members;   // indices into ep.sats
    if (starting) {
      // A81 tdcp_raim_both_ends: a satellite this epoch's robust least squares rejected opens no pair (the closing end already
      // refuses one it rejected at k1). Off: the usable satellites in order, as before.
      std::vector<char> usable(ep.sats.size());
      std::vector<int> sat_ids(ep.sats.size());
      for (size_t i = 0; i < ep.sats.size(); ++i) {
        usable[i] = g0.ok[i] && masks[i].cp ? 1 : 0;
        sat_ids[i] = ep.sats[i].sat.rtk_sat;
      }
      start_members = livgoTdcpStartMembers(raimBothEnds(), usable, sat_ids, l1_rejected);
      if (static_cast<int>(start_members.size()) < std::max(2, cfg.rejection.min_meas.tdcp)) starting = false;
    }
    // A61 F11: the phase noise of this epoch is shared by the closing pair (+ε) and the one opening here (−ε). It becomes
    // a state before the update (one component per start member, prior σ²_Φ·scale.tdcp, independent of everything), the
    // closing rows carry its column, the next pair's rows will too; openPair() adopts it (tag = the next pair's id).
    std::vector<int> eta_next(ep.sats.size(), -1);   // ep.sats index → component of the new Phase slot
    const int eta_next_tag = next_pair_id;
    if (starting && chain && cfg.noise.tdcp.phase_states && !cfg.ablation.tdcp_fixed_anchor) {
      const int d = static_cast<int>(start_members.size());
      Eigen::MatrixXd N = Eigen::MatrixXd::Zero(d, d);
      for (int k = 0; k < d; ++k) {
        const int i = start_members[static_cast<size_t>(k)];
        const double s = noise->sigmaPhase(ep.sats[i].sig[0].cp_std_m, g0.geo[i].el_rad);
        N(k, k) = s * s * cfg.noise.scale.tdcp;
        eta_next[static_cast<size_t>(i)] = k;
      }
      removeSlotIf(Slot::Phase, eta_next_tag);
      aug->addSlot(Slot::Phase, eta_next_tag, "phase_" + std::to_string(eta_next_tag), Eigen::VectorXd::Zero(d),
                   Eigen::MatrixXd::Zero(d, HostIdx::dim), Eigen::MatrixXd::Zero(d, aug->dim()), N, core.P);
      g0.J = rows->antennaJacobians(linPoint(core, *aug, br));   // the layout grew: J over [host | x_G] again
    }

    // ---- L3: TDCP pair end k1 (§10 L3 D1–D6 before any gate) ----
    RowBuilder::TdcpPairInput tin;
    std::vector<int> tin_ep;             // tin.sats[j] ↔ ep.sats[tin_ep[j]]
    std::vector<int> tin_member;         // ↔ pair.members[...]
    std::vector<RejectReason> tin_reason;
    std::vector<uint8_t> tin_detectors;
    std::vector<int> tin_eta0, tin_eta1;   // A61 F11: components of the old / new Phase slot, −1 none
    int tdcp_lost = 0;
    const Eigen::MatrixXd Sigma0 = aug->jointCov(core.P);
    if (closing) {
      tin.pair_id = pair.id;
      tin.t_k0 = pair.t_k0;
      tin.t_k1 = ep.t_rx;
      tin.dt_pair_s = ep.t_rx.diff(pair.t_k0);
      tin.clock_bias_k0_m = pair.b_k0;
      tin.clock_jump_m = pair.clock_jump_m;
      for (size_t k = 0; k < pair.members.size(); ++k) {
        const PairMember& m = pair.members[k];
        const int i = findSat(ep, m.sat);
        if (i < 0 || !g0.ok[i] || !masks[i].cp) {
          ++tdcp_lost;
          continue;
        }
        const SignalObs& s1 = ep.sats[i].sig[0];
        tin.sats.push_back(m.sat);
        tin.phi_k0_cyc.push_back(m.k0.cp_cyc);
        tin.phi_k1_cyc.push_back(s1.cp_cyc);
        tin.rho_k0.push_back(0.0);
        tin.dts_k0.push_back(m.dts_k0);
        tin.dts_k1.push_back(0.0);                      // filled per iterate from the k1 geometry
        tin.u_k0.push_back(Eigen::Vector3d::Zero());
        tin.cp_std_k0_m.push_back(m.k0.cp_std_m);
        tin.el_k0_rad.push_back(m.el_k0);
        tin.eph_snap.push_back(m.snap);
        tin.psr_k0.push_back(m.psr_k0);
        tin.rs_k0.push_back(m.rs_k0);
        tin.cp_std_k1_m.push_back(s1.cp_std_m);
        tin.el_k1_rad.push_back(g0.geo[i].el_rad);
        tin.trop_k0_m.push_back(m.trop_k0);
        tin.ion_k0_m.push_back(ion ? m.ion_k0 : std::numeric_limits<double>::quiet_NaN());   // both ends or neither
        tin_ep.push_back(i);
        tin_member.push_back(static_cast<int>(k));
        tin_reason.push_back(livgoFull() && l1_rejected.count(m.sat.rtk_sat) ? RejectReason::L1
                                                                              : RejectReason::NotRejected);
        tin_detectors.push_back(0);
        tin_eta0.push_back(m.eta);
        tin_eta1.push_back(eta_next[static_cast<size_t>(i)]);
      }
    }
    // k1 geometry with the ephemeris frozen at k0, at an iterate
    auto k1Geometry = [&](const EpochGeo& gi, const AugBlock& a, std::vector<SatGeometry>* out) {
      out->assign(tin.sats.size(), SatGeometry());
      const int oc_i = a.find(Slot::Clk);
      const double b_i = oc_i >= 0 ? a.x()(oc_i) - br.delta * a.x()(oc_i + 1) : b_g;
      for (size_t j = 0; j < tin.sats.size(); ++j) {
        SatGeometry g;
        if (rows->geometryFromSnapshot(ep.sats[tin_ep[j]], tin.eph_snap[j], gi.r_A, gi.rcv, ion, ep.t_rx, b_i, &g)) (*out)[j] = g;
      }
    };
    // frame_exact: ĉ(x) = the k0 antenna mapped through this iterate's registration and gravity (livgo_anchor.hpp), and the
    // row's columns for those shared states made relative to it. `Jc` is returned for shareAnchor() at the same iterate.
    const bool share_anchor = frameExact() && pair_open && pair.fixed_anchor && pair.anchor_k0.valid;
    auto fillPairIterate = [&](const CoreState& xi, const EpochGeo& gi, const AugBlock& a, const std::vector<SatGeometry>& geo_k1,
                               RowBuilder::TdcpPairInput* t, AntennaJacobians* Jc) {
      *t = tin;
      t->clone_slot = pair.fixed_anchor ? -1 : a.find(Slot::Clone, pair.id);
      t->c_hat_N = t->clone_slot >= 0 ? Eigen::Vector3d(a.x().segment<3>(t->clone_slot)) : pair.c_fixed;
      if (share_anchor) {
        *Jc = livgoAnchorAt(*rows, linPoint(xi, a, br), pair.anchor_k0);
        t->c_hat_N = Jc->a_N;
      }
      for (size_t j = 0; j < t->sats.size(); ++j) t->dts_k1[j] = geo_k1[j].state.ok ? geo_k1[j].state.dts : 0.0;
      t->dv_pair = gi.J.w_N - pair.w_N_k0;
      // A61 F11: phase-noise columns (the slots exist from before the update, their offsets are fixed during it)
      const int o0 = a.find(Slot::Phase, pair.id), o1 = a.find(Slot::Phase, eta_next_tag);
      t->eta_k0_col.assign(t->sats.size(), -1);
      t->eta_k1_col.assign(t->sats.size(), -1);
      for (size_t j = 0; j < t->sats.size() && j < tin_eta0.size(); ++j) {
        if (o0 >= 0 && tin_eta0[j] >= 0) t->eta_k0_col[j] = o0 + tin_eta0[j];
        if (o1 >= 0 && tin_eta1[j] >= 0) t->eta_k1_col[j] = o1 + tin_eta1[j];
      }
    };
    // H ← H + u₁ᵀJ_c on the states the k1 antenna and its anchor share (rows in the order of tin.sats, matched by satellite).
    auto shareAnchor = [&](RowSet* rs, const std::vector<SatGeometry>& gk1, const AntennaJacobians& Jc, const AugBlock& a) {
      const std::vector<int> cols = livgoSharedAnchorColumns(a, repivotGravity());
      for (MeasRow& r : rs->rows) {
        for (size_t j = 0; j < tin.sats.size() && j < gk1.size(); ++j) {
          if (tin.sats[j].rtk_sat != r.sat.rtk_sat) continue;
          livgoShareAnchorColumns(&r.H, Eigen::Vector3d(anchor.R_EN.transpose() * gk1[j].e_ecef), Jc, cols);
          break;
        }
      }
    };
    if (closing && !tin.sats.empty()) {
      std::vector<SatGeometry> gk1;
      k1Geometry(g0, *aug, &gk1);
      RowBuilder::TdcpPairInput t0;
      AntennaJacobians Jc0;
      fillPairIterate(core, g0, *aug, gk1, &t0, &Jc0);
      RowSet prior_rows;
      rows->tdcpRows(t0, gk1, lp0, g0.J, &prior_rows);
      if (share_anchor) shareAnchor(&prior_rows, gk1, Jc0, *aug);
      double dclk = 0.0;
      RowBuilder::projectOutCommonClock(prior_rows, nullptr, &dclk);
      for (size_t j = 0; j < tin.sats.size(); ++j) {
        const PairMember& m = pair.members[tin_member[j]];
        const SatObs& so1 = ep.sats[tin_ep[j]];
        SlipInput si;
        si.k0 = &m.k0;
        si.k1 = &so1.sig[0];
        // D4 integrates the Doppler over the true interval: a k-ms receiver clock step inside the pair shifts the true
        // reception instant of k1 by k ms (ΔΦ carries the step itself, removed through clock_jump_m).
        si.dt_s = tin.dt_pair_s - pair.clock_jump_m / kCLight;
        si.lambda_m = so1.sig[0].freq_hz > 0.0 ? kCLight / so1.sig[0].freq_hz : 0.0;
        if (m.has_second) {
          const int k2 = signalWithCode(so1, m.k0_second.code);
          if (k2 >= 0 && so1.sig[k2].cpValid()) {
            si.k0_second = &m.k0_second;
            si.k1_second = &so1.sig[k2];
            si.lambda2_m = m.k0_second.freq_hz > 0.0 ? kCLight / m.k0_second.freq_hz : 0.0;
          }
        }
        // D4 (AMENDMENTS A61 F4): 4σΔt with σ the standard deviation of the statistic ΔΦ + (D_k0 + D_k1)Δt/2 itself, from
        // the calibrated models — the carrier σ_Φ(el)²·scale.tdcp at both ends, the single-sample Doppler variance at both
        // ends, and the receiver clock's walk inside the pair, S_bΔt + S_dΔt³/12 (Δb minus the trapezoid of the drift; the
        // same for every satellite) — expressed as the equivalent σ_D [Hz] the threshold takes. (Before A61: the zenith
        // σ_D0/λ, which over-covered high satellites and under-covered low ones, ~1 false slip per 450 m in U15.)
        si.sigma_dopp_hz = std::max(so1.sig[0].dopp_std_hz, si.lambda_m > 0.0 ? cfg.noise.dopp.sigma0_mps / si.lambda_m : 0.0);
        if (si.lambda_m > 0.0 && si.dt_s > 0.0) {
          const double el1 = gk1[j].state.ok ? gk1[j].el_rad : m.el_k0;
          const double sp0 = noise->sigmaPhase(m.k0.cp_std_m, m.el_k0), sp1 = noise->sigmaPhase(so1.sig[0].cp_std_m, el1);
          const double vd0 = noise->doppMarginalVar(si.lambda_m, m.k0.dopp_std_hz, m.el_k0);
          const double vd1 = noise->doppMarginalVar(si.lambda_m, so1.sig[0].dopp_std_hz, el1);
          const double dt = si.dt_s;
          const double var_clk = cfg.process.clock.S_b * dt + cfg.process.clock.S_d * dt * dt * dt / 12.0;
          const double var_stat = (sp0 * sp0 + sp1 * sp1) * cfg.noise.scale.tdcp + 0.25 * dt * dt * (vd0 + vd1) + var_clk;
          si.sigma_dopp_hz = std::sqrt(var_stat) / (si.lambda_m * si.dt_s);
        }
        si.lli_between = m.lli_between;
        si.clock_jump_m = pair.clock_jump_m;
        for (size_t q = 0; q < prior_rows.rows.size(); ++q) {
          const MeasRow& r = prior_rows.rows[q];
          if (r.sat.rtk_sat != m.sat.rtk_sat) continue;
          const double s_ii = quadForm(r.H, Sigma0) +
                              (prior_rows.has_full_R ? prior_rows.R_full(static_cast<int>(q), static_cast<int>(q)) : r.R);
          si.innov_pred_m = r.nu - dclk;
          si.innov_sigma_m = sqrtPos(s_ii);
          si.innov_valid = si.innov_sigma_m > 0.0;
          break;
        }
        const SlipVerdict sv = slip->check(si);
        tin_detectors[j] = sv.detectors;
        if (sv.slip) {
          tin_reason[j] = slipReason(sv.detectors);
          char d[160];
          std::snprintf(d, sizeof d, "%s detectors=0x%02x d4=%.3f/%.3f cyc d6=%.4f m", satIdStringOf(m.sat).c_str(), sv.detectors,
                        sv.d4_value_cyc, sv.d4_threshold_cyc, sv.d6_value_m);
          emit(EventKind::SLIP, t_k, t_est, ep.t_rx, sv.detectors, d);
        }
      }
    }

    // ---- Doppler rows left out at the pair's end epochs (§4.4) ----
    std::vector<int> excluded;
    if (cfg.noise.dopp.exclusive_with_tdcp) {
      if (closing)
        for (const SatId& s : tin.sats) excluded.push_back(s.rtk_sat);
      if (starting)
        for (int i : start_members) excluded.push_back(ep.sats[i].sat.rtk_sat);
    }
    // FAST-LIVGO RAIM: a satellite the SPP rejected does not enter Doppler or TDCP either.
    if (livgoFull())
      for (int id : l1_rejected) excluded.push_back(id);

    // ---- the joint update (L4 → H3 → L5 inside JointUpdater, §4.8) ----
    const bool with_tdcp = closing && !tin.sats.empty();
    bool tdcp_in_factory = true;   // false only for the P2 counterfactual below (A75 revision 1)
    const JointUpdater::RowFactory factory = [&](const CoreState& x, const AugBlock& a, std::vector<RowSet>* sets) {
      EpochGeo gi;
      epochGeometry(ep, x, a, br, ion, &g0, &gi);
      const LinPoint lp = linPoint(x, a, br);
      if (psr_epoch) {
        RowSet s;
        rows->pseudorangeRows(ep, gi.geo, masks, lp, gi.J, &s);
        for (MeasRow& r : s.rows)
          if (l1_rejected.count(r.sat.rtk_sat)) r.reason = RejectReason::L1;
        sets->push_back(std::move(s));
      }
      if (dopp_on) {
        RowSet s;
        rows->dopplerRows(ep, gi.geo, masks, lp, gi.J, excluded, &s);
        sets->push_back(std::move(s));
      }
      if (with_tdcp && tdcp_in_factory) {
        std::vector<SatGeometry> gk1;
        k1Geometry(gi, a, &gk1);
        RowBuilder::TdcpPairInput ti;
        AntennaJacobians Jci;
        fillPairIterate(x, gi, a, gk1, &ti, &Jci);
        RowSet s;
        rows->tdcpRows(ti, gk1, lp, gi.J, &s);
        if (share_anchor) shareAnchor(&s, gk1, Jci, a);
        for (MeasRow& r : s.rows) {
          for (size_t j = 0; j < tin.sats.size(); ++j)
            if (tin.sats[j].rtk_sat == r.sat.rtk_sat && tin_reason[j] != RejectReason::NotRejected) r.reason = tin_reason[j];
        }
        sets->push_back(std::move(s));
      }
    };
    JointUpdater::Options opt;
    opt.force_three_iterations = reacq_left > 0;
    opt.schmidt = integ->flags().livo_alarm;
    opt.disable_l4_l5 = cfg.ablation.disable_l4_l5 || cfg.mode == Config::Mode::LivgoProbeNorej;
    // FAST-LIVGO mode B (paper IV-E): the windowed translation λ_min is below λ_th, so the chi-square
    // against the LIVO prior is off. Mode A keeps L4/L5. Until the window is full the mean is undefined
    // and the stricter branch (Mode A) stays on.
    if (liv.enable && livgo_degraded) opt.disable_l4_l5 = true;
    if (liv.enable) livgoLogEpoch(t_k, !opt.disable_l4_l5);
    opt.observe_only = obs_only;
    // The legacy arm freezes the whole registration as consider parameters. With frame_exact tilt and translation have no
    // variance (nothing to consider: they are in no S) and the yaw is estimated, so no entry at all.
    if (livgoFull() && !liv.frame_exact && aug->has(Slot::Tilt)) {
      const MeasClass cls_pin[2] = {MeasClass::Dopp, MeasClass::Tdcp};
      const Slot slots_pin[3] = {Slot::Tilt, Slot::Yaw, Slot::Trans};
      for (MeasClass c : cls_pin)
        for (Slot s : slots_pin) opt.consider.push_back({c, s});
    }
    // A70 (default): the Doppler rows projected orthogonal to the tilt φ's columns — no φ content left in them.
    if (dopp_on && cfg.noise.dopp.project_tilt) opt.project.push_back({MeasClass::Dopp, Slot::Tilt});
    // A66 (option since A70): the Doppler rows do not update the tilt φ (consider parameter); φ's σ stays in their S.
    if (dopp_on && cfg.noise.dopp.consider_tilt) opt.consider.push_back({MeasClass::Dopp, Slot::Tilt});
    const HostMat P_prior = core.P;
    // A75 revision 1 (P2): the epoch without its TDCP rows, on copies of the state before the update (observer only).
    const bool gain_on = livgoFull() && liv.tdcp_gain_log && !liv.tdcp_gain_log_path.empty() && with_tdcp && !obs_only;
    std::unique_ptr<CoreState> gain_core0;
    std::unique_ptr<AugBlock> gain_aug0;
    if (gain_on) {
      gain_core0.reset(new CoreState(core));
      gain_aug0.reset(new AugBlock(*aug));
    }
    JointUpdater::Result res;
    const bool any_rows = psr_epoch || dopp_on || with_tdcp;
    if (any_rows) res = updater->update(core, *aug, factory, opt);
    if (gain_on && res.committed) {
      const int n_tdcp = res.gates[ci(MeasClass::Tdcp)].n_out;
      double tr_no = std::numeric_limits<double>::quiet_NaN();
      int status = 2;   // 2: no accepted TDCP row — the epoch is its own counterfactual
      const double tr_all = livgoPosTrace(core.P);
      if (n_tdcp > 0) {
        tdcp_in_factory = false;
        status = livgoCounterfactualPosTrace(*updater, *gain_core0, *gain_aug0, factory, opt, &tr_no) ? 1 : 0;
        tdcp_in_factory = true;
      } else {
        tr_no = tr_all;
      }
      livgoLogGain(t_k, static_cast<int>(phase), res.gates[ci(MeasClass::Psr)].n_out, res.gates[ci(MeasClass::Dopp)].n_out, n_tdcp,
                   livgoPosTrace(P_prior), tr_all, tr_no, status);
    }
    if (livgoFull()) livgoPin();
    if (any_rows && reacq_left > 0) --reacq_left;
    if (res.committed && !obs_only) {
      ++tot.epochs_updated;
      rep->epochs_committed += 1;
      if (!opt.schmidt) rep->core_modified = true;
    }
    if (res.rolled_back) {
      ++tot.epochs_rolled_back;
      rep->epochs_rolled_back += 1;
      emit(EventKind::ROLLBACK, t_k, t_est, ep.t_rx, res.nis_post, res.rollback_reason);
    }
    rec.committed = res.committed && !obs_only;
    rec.rolled_back = res.rolled_back;
    rec.iterations = res.iterations;
    rec.f_p = res.f_p;
    rec.f_xi = res.f_xi;
    rec.f_clk = res.f_clk;
    rec.f_v = res.f_v;
    rec.dp_host_norm = res.dp_host_norm;
    rec.dyaw = res.dyaw;
    rec.dt_norm = res.dt_norm;

    // ---- per-class statistics (gnss_epochs.csv) ----
    classStats(ep, g0, masks, l1_rejected, excluded, psr_epoch, dopp_on, with_tdcp, closing, tdcp_lost, res, &rec);

    // ---- time-offset monitor on the accepted Doppler rows (§7, A17) ----
    if (dopp_on) toffMonitor(res, g0, Sigma0, t_k, t_est, ep.t_rx);

    // ---- integrity judge (L7): GNSS-only Doppler velocity vs the LIVO-predicted antenna velocity ----
    if (dop.ok) {
      const Eigen::Matrix3d RtE = anchor.R_EN.transpose();
      const Eigen::Vector3d v_gnss = RtE * dop.v_ecef;
      const Eigen::Matrix3d cov_gnss = RtE * dop.Q.topLeftCorner<3, 3>() * RtE.transpose();
      const Eigen::MatrixXd Jw = g0.J.J_w;
      const Eigen::Matrix3d cov_w = Jw * Sigma0 * Jw.transpose();
      integ->onVelocity(v_gnss, cov_gnss, g0.J.w_N, cov_w, slotSigma(Slot::Yaw));
      integrityEdges(t_k, t_est, ep.t_rx);
    }

    // ---- L4 rejection shares for the lock-out detector (L6) ----
    for (MeasClass c : {MeasClass::Psr, MeasClass::Dopp}) {
      const GateResult& gr = res.gates[ci(c)];
      if (gr.n_in <= 0) continue;
      const double f = static_cast<double>(gr.n_in - gr.n_out) / gr.n_in;
      (c == MeasClass::Psr ? book.frac_psr : book.frac_dopp) = f;
    }

    // ---- TDCP pair bookkeeping (§4.5): close at k1, open at k0, both after the update ----
    if (closing) {
      closePair(false);
      last_k1_index = epoch_index;
    }
    if (starting && active) openPair(core, ep, g0, masks, start_members, br, t_k);
    // A61 F11: a Phase slot made for a pair that did not open is marginalised (its rows already used it)
    if (aug->has(Slot::Phase, eta_next_tag) && !(pair_open && pair.id == eta_next_tag)) removeSlotIf(Slot::Phase, eta_next_tag);

    // ---- postGNSS covariance row (filter_cov.csv, D1/D3) ----
    if (rec.committed && sink) covRecord(core, P_prior, t_k);
    if (cfg.output.sat_csv && sink) satRecords(ep, g0, masks, l1_rejected, excluded, tin, tin_detectors, res);
  }

  // ---------------- L2 / RECOVERY clock-only reset ----------------
  void clockReset(const std::vector<SppInput>& in, const EpochGeo& g0, const BridgeResult& br, double t_k, double t_est,
                  const GTime& g, const char* why) {
    const ClockOnlyResult c = clockOnlyLs(in, g0.r_A, g0.v_A);
    const int oc = aug->find(Slot::Clk);
    if (!c.ok || oc < 0) {
      emit(EventKind::CLOCK_JUMP, t_k, t_est, g, 0.0, std::string(why) + ": clock-only LS failed (no GPS/QZSS code)");
      return;
    }
    const bool have_d = c.var_d > 0.0;
    const double d = have_d ? c.d_mps : aug->x()(oc + 1);
    const double var_d = have_d ? c.var_d : aug->Q()(oc + 1, oc + 1);
    // The LS solves the clock at the epoch (t_g); the slot holds it at t_k = t_g + δ.
    Eigen::Matrix2d cov;
    cov << c.var_b + br.delta * br.delta * var_d, br.delta * var_d, br.delta * var_d, var_d;
    char dd[128];
    std::snprintf(dd, sizeof dd, "%s: b=%.3f m d=%.4f m/s n=%d%s", why, c.b_m + br.delta * d, d, c.n_used,
                  observe() ? " observe_only" : "");
    emit(EventKind::CLOCK_JUMP, t_k, t_est, g, c.b_m - aug->x()(oc), dd);
    if (!observe()) resetClockStates(aug.get(), c.b_m + br.delta * d, d, cov);
  }

  // ---------------- TDCP pairs ----------------
  void markIntermediate(const ObsEpoch& ep) {
    for (PairMember& m : pair.members) {
      const int i = findSat(ep, m.sat);
      if (i < 0) {
        m.lli_between = true;
        continue;
      }
      const SignalObs& s = ep.sats[i].sig[0];
      if (!s.cpValid() || (s.lli & 0x3)) m.lli_between = true;
    }
  }

  void openPair(const CoreState& core, const ObsEpoch& ep, const EpochGeo& g0, const std::vector<MaskVerdict>& masks,
                const std::vector<int>& members, const BridgeResult& br, double t_k) {
    (void)masks;
    TdcpPair p;
    p.id = next_pair_id++;
    p.t_k0 = ep.t_rx;
    p.index_k0 = epoch_index;
    p.fixed_anchor = cfg.ablation.tdcp_fixed_anchor;
    const int oc = aug->find(Slot::Clk);
    p.b_k0 = oc >= 0 ? aug->x()(oc) - br.delta * aug->x()(oc + 1) : 0.0;
    Klobuchar ion_buf_k0;
    const bool have_ion = iono(&ion_buf_k0) != nullptr;   // the k0 geometry was built with it (A61 F2)
    const bool have_eta = aug->has(Slot::Phase, p.id);   // A61 F11: made before the update in the members' order
    for (size_t k = 0; k < members.size(); ++k) {
      const int i = members[k];
      const SatObs& so = ep.sats[i];
      const SignalObs& s = so.sig[0];
      PairMember m;
      m.sat = so.sat;
      m.k0 = s;
      m.eta = have_eta ? static_cast<int>(k) : -1;
      const int k2 = secondSignal(so);
      if (k2 >= 0) {
        m.k0_second = so.sig[k2];
        m.has_second = true;
      }
      // transmit time with the measured code (the predicted one for a carrier-only epoch), ephemeris frozen here
      const double P = s.psrValid() ? s.psr_m : g0.geo[i].range + p.b_k0 - kCLight * g0.geo[i].state.dts;
      rtk::SatState st;
      if (!solver->satState(so.sat, ep.t_rx, P, &st, &m.snap) || !st.ok) continue;
      m.dts_k0 = st.dts;
      m.rs_k0 = st.r_ecef;
      m.psr_k0 = P;
      m.el_k0 = g0.geo[i].el_rad;
      m.trop_k0 = g0.geo[i].trop_m;
      m.ion_k0 = have_ion ? g0.geo[i].ion_m : std::numeric_limits<double>::quiet_NaN();
      p.members.push_back(m);
    }
    if (static_cast<int>(p.members.size()) < std::max(2, cfg.rejection.min_meas.tdcp)) return;
    // clone c = a^N(x̂_k0) at the posterior of k0's update (§2.4 consistent augmentation, Σ_cc carries the k0 bridge noise)
    const AntennaJacobians J = rows->antennaJacobians(linPoint(core, *aug, br));
    p.w_N_k0 = J.w_N;
    if (p.fixed_anchor) {
      p.c_fixed = J.a_N;
      if (frameExact()) {   // A75: keep what ĉ(x) = a^N(x_k0; registration, g) needs (livgo_anchor.hpp)
        p.anchor_k0.valid = true;
        p.anchor_k0.core = core;
        p.anchor_k0.br = br;
      }
    } else {
      Eigen::MatrixXd F_host, F_aug, N;
      RowBuilder::cloneJacobians(J, br.Sigma_D, &F_host, &F_aug, &N);
      aug->addSlot(Slot::Clone, p.id, "clone_" + std::to_string(p.id), J.a_N, F_host, F_aug, N, core.P);
    }
    pair = std::move(p);
    pair_open = true;
    (void)t_k;
  }

  // Closes the open pair and drops its clone (parked or consumed).
  void closePair(bool parked) {
    if (parked) removeAllOf(Slot::Phase);   // A61 F11: nothing continues from a parked pair
    if (!pair_open) return;
    if (!pair.fixed_anchor) removeSlotIf(Slot::Clone, pair.id);
    removeSlotIf(Slot::Phase, pair.id);
    pair_open = false;
    pair = TdcpPair();
  }

  // ---------------- AMENDMENTS A61 F9: map frame jumps ----------------
  // A host update (LIO/VIO) that moves the pose far outside the spread of its own correction — for a linear update
  // Cov(x⁺ − x⁻) = P⁻ − P⁺, so χ²₆ of the pose part against that block — AND by a large absolute amount is, most likely,
  // a jump of the map frame (the LIO locked onto an older part of its map): information about the frame, not about the
  // IMU prediction. H2 conditions x_G on it all the same (the joint covariance stays exact); frameJumpApply() then widens
  // the registration by the size of the correction — translation along the correction mapped to N (plus a tenth of it
  // isotropically), heading by the rotation's vertical part, tilt by its horizontal part in the levelled frame (again along
  // it, plus a tenth isotropically) — so that the next GNSS rows move ξ
  // to where the antenna really is. The open TDCP pair is kept on purpose: its clone was taken before the jump, so the
  // pair measures the frame change at millimetre level. No deterministic compensation: a host that absorbs a jump over
  // several cycles overshoots (its velocity is corrected too), and only the frame part of its corrections should move ξ —
  // which only the GNSS can tell.
  struct FrameJumpProbe {
    bool armed = false;
    Eigen::Vector3d dp_W = Eigen::Vector3d::Zero(), dth = Eigen::Vector3d::Zero();   // host correction (W; body)
    double chi2 = 0.0;
  };
  FrameJumpProbe frameJumpProbe(const CoreState& prior, const CoreState& post) const {
    FrameJumpProbe fp;
    const Config::Integrity::FrameJump& fj = cfg.integrity.frame_jump;
    // A75 revision 1: with frame_exact the registration has no process noise and tilt and translation are exact constants; the
    // widening below would put variance back into both (and into the yaw) between a pin and the next S. Not armed there.
    if (!fj.enable || !active || observe() || frameExact() || !aug->has(Slot::Yaw) || !aug->has(Slot::Trans) || !leveling.hasRef())
      return fp;
    const Eigen::Vector3d dth = so3Log(Eigen::Matrix3d(prior.R.transpose() * post.R));
    const Eigen::Vector3d dp = post.p - prior.p;
    if (dp.norm() < fj.min_translation_m && dth.norm() < fj.min_rotation_rad) return fp;
    Eigen::Matrix<double, 6, 6> S;
    const int ir = HostIdx::rot, ip = HostIdx::pos;
    const HostMat dP = prior.P - post.P;
    S.block<3, 3>(0, 0) = dP.block<3, 3>(ir, ir);
    S.block<3, 3>(0, 3) = dP.block<3, 3>(ir, ip);
    S.block<3, 3>(3, 0) = dP.block<3, 3>(ip, ir);
    S.block<3, 3>(3, 3) = dP.block<3, 3>(ip, ip);
    Eigen::Matrix<double, 6, 1> d;
    d << dth, dp;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> es(0.5 * (S + S.transpose()));
    if (es.info() != Eigen::Success) return fp;
    const Eigen::Matrix<double, 6, 1> v = es.eigenvectors().transpose() * d;
    double chi2 = 0.0;
    for (int i = 0; i < 6; ++i) chi2 += v(i) * v(i) / std::max(es.eigenvalues()(i), 1e-12);
    if (chi2 < fj.chi2) return fp;
    fp.armed = true;
    fp.dp_W = dp;
    fp.dth = dth;
    fp.chi2 = chi2;
    return fp;
  }
  void frameJumpApply(const FrameJumpProbe& fp, const CoreState& post) {
    if (!fp.armed) return;
    const int iy = aug->find(Slot::Yaw), it = aug->find(Slot::Trans), iphi = aug->find(Slot::Tilt);
    if (iy < 0 || it < 0) return;
    const Eigen::Vector2d tilt = aug->tilt();
    const Eigen::Matrix3d M = leveling.M(aug->yaw(), tilt, post.g);
    const Eigen::Vector3d dN = M * fp.dp_W;                          // the correction in N
    const Eigen::Vector3d up_W = -post.g.normalized();
    const Eigen::Vector3d rot_W = post.R * fp.dth;                   // body rotation vector in W
    const double dyaw = rot_W.dot(up_W);
    // the tilt is the rotation after the levelling C(g): its horizontal part, along its own direction (a tenth of it
    // isotropically, as for the translation)
    const Eigen::Vector2d h = Eigen::Vector3d(leveling.C(post.g) * rot_W).head<2>();
    Eigen::Matrix<double, 6, 6> W = Eigen::Matrix<double, 6, 6>::Zero();   // (w_φ, w_ψ, w_t0)
    if (iphi >= 0) W.block<2, 2>(0, 0) = h * h.transpose() + 0.1 * h.squaredNorm() * Eigen::Matrix2d::Identity();
    W(2, 2) = dyaw * dyaw;
    W.block<3, 3>(3, 3) = dN * dN.transpose() + 0.01 * dN.squaredNorm() * Eigen::Matrix3d::Identity();
    // Mapped like the registration walk (AugBlock::processModel): the rotation turns about the current antenna point, where
    // the host measured the jump, so the translation at the pivot moves with it by [u]×Δr, u = M(a^W − o^W) (up to
    // pivot_distance_m); widened independently, a rotation of the frame would leave the pivot's translation as sure as
    // before.
    const Eigen::Vector3d a_W = post.p + post.R * lever_I;
    const Eigen::Matrix3d U = skew(Eigen::Vector3d(M * (a_W - pivot_W)));
    const Eigen::Matrix3d RzJl = Leveling::Rz(aug->yaw()) * Leveling::tiltLeftJacobian(tilt);
    Eigen::Matrix<double, 6, 6> J = Eigen::Matrix<double, 6, 6>::Identity();
    J.block<3, 2>(3, 0) = U * RzJl.leftCols<2>();
    J.block<3, 1>(3, 2) = U.col(2);
    const Eigen::Matrix<double, 6, 6> Qxi = J * W * J.transpose();
    const int col[6] = {iphi, iphi < 0 ? -1 : iphi + 1, iy, it, it + 1, it + 2};
    Eigen::MatrixXd& Q = aug->Q();
    for (int a = 0; a < 6; ++a) {
      if (col[a] < 0) continue;
      for (int b = 0; b < 6; ++b)
        if (col[b] >= 0) Q(col[a], col[b]) += Qxi(a, b);
    }
    char d[200];
    std::snprintf(d, sizeof d, "host correction %.3f m / %.2f deg (chi2 %.0f): registration widened by %.3f m, %.2f deg yaw", fp.dp_W.norm(),
                  fp.dth.norm() * 180.0 / M_PI, fp.chi2, dN.norm(), std::fabs(dyaw) * 180.0 / M_PI);
    emit(EventKind::FRAME_JUMP, t_k_last, 0.0, GTime(), dN.norm(), d);
  }

  // ---------------- diagnostics helpers ----------------
  void classStats(const ObsEpoch& ep, const EpochGeo& g0, const std::vector<MaskVerdict>& masks, const std::set<int>& l1,
                  const std::vector<int>& excluded, bool psr_epoch, bool dopp_on, bool with_tdcp, bool closing, int tdcp_lost,
                  const JointUpdater::Result& res, EpochRecord* rec) {
    ClassStats& cp = rec->cls[ci(MeasClass::Psr)];
    ClassStats& cd = rec->cls[ci(MeasClass::Dopp)];
    ClassStats& ct = rec->cls[ci(MeasClass::Tdcp)];
    auto isExcluded = [&](int rtk_sat) { return std::find(excluded.begin(), excluded.end(), rtk_sat) != excluded.end(); };
    int psr_built = 0, dopp_built = 0;
    const RowSet* gp = nullptr;
    const RowSet* gd = nullptr;
    const RowSet* gt = nullptr;
    for (const RowSet& s : res.gated) {
      if (s.cls == MeasClass::Psr) gp = &s;
      if (s.cls == MeasClass::Dopp) gd = &s;
      if (s.cls == MeasClass::Tdcp) gt = &s;
    }
    if (gp) psr_built = static_cast<int>(gp->rows.size());
    if (gd) dopp_built = static_cast<int>(gd->rows.size());
    // candidates and the reasons decided before the rows (NoEph, Mask, L1, Exclusive)
    int psr_noeph = 0, psr_cand = 0, dopp_noeph = 0, dopp_cand = 0, dopp_excl = 0;
    for (size_t i = 0; i < ep.sats.size(); ++i) {
      const SignalObs* s = ep.sats[i].primary();
      if (!s) continue;
      if (s->psrValid()) {
        ++psr_cand;
        if (!g0.ok[i]) ++psr_noeph;
      }
      if (s->doppValid()) {
        ++dopp_cand;
        if (!g0.ok[i]) ++dopp_noeph;
        else if (masks[i].dopp && isExcluded(ep.sats[i].sat.rtk_sat)) ++dopp_excl;
      }
    }
    (void)l1;
    if (psr_epoch) {
      cp.candidates = psr_cand;
      cp.rejected[ri(RejectReason::NoEph)] += psr_noeph;
      cp.rejected[ri(RejectReason::Mask)] += std::max(0, psr_cand - psr_noeph - psr_built);
    } else {
      cp.skipped = true;   // not the first epoch of its GPST second (A43) or no code rows in this arm
    }
    if (dopp_on) {
      cd.candidates = dopp_cand;
      cd.rejected[ri(RejectReason::NoEph)] += dopp_noeph;
      cd.rejected[ri(RejectReason::Exclusive)] += dopp_excl;
      cd.rejected[ri(RejectReason::Mask)] += std::max(0, dopp_cand - dopp_noeph - dopp_excl - dopp_built);
    } else {
      cd.skipped = true;
    }
    if (closing) {
      ct.candidates = tdcp_lost + (gt ? static_cast<int>(gt->rows.size()) : 0);
      ct.rejected[ri(RejectReason::Mask)] += tdcp_lost;
    } else {
      ct.skipped = true;
    }
    (void)with_tdcp;
    const bool committed = res.committed && !observe();
    auto fromGated = [&](const RowSet* s, MeasClass c, ClassStats* cs) {
      const GateResult& gr = res.gates[ci(c)];
      cs->nis_prior = gr.nis;
      cs->dof = gr.dof;
      if (gr.class_skipped) cs->skipped = true;
      if (!s) return;
      for (const MeasRow& r : s->rows) {
        if (r.accepted && r.reason == RejectReason::NotRejected) {
          if (res.rolled_back) ++cs->rejected[ri(RejectReason::Step)];
          else ++cs->accepted;
        } else if (r.reason != RejectReason::NotRejected) {
          ++cs->rejected[ri(r.reason)];
        }
      }
      if (committed && cs->accepted > 0) cs->nis_post = res.nis_post;   // joint L5 statistic (one per update)
      if (cs->accepted == 0) cs->skipped = true;
      tot.cand[ci(c)] += cs->candidates;
      tot.acc[ci(c)] += cs->accepted;
      if (gr.dof > 0) {
        tot.nis[ci(c)] += gr.nis;
        tot.dof[ci(c)] += gr.dof;
      }
    };
    if (psr_epoch) fromGated(gp, MeasClass::Psr, &cp);
    if (dopp_on) fromGated(gd, MeasClass::Dopp, &cd);
    if (closing) fromGated(gt, MeasClass::Tdcp, &ct);
    rec->cls[ci(MeasClass::PvtPos)].skipped = true;
    rec->cls[ci(MeasClass::PvtVel)].skipped = true;
  }

  void toffMonitor(const JointUpdater::Result& res, const EpochGeo& g0, const Eigen::MatrixXd& Sigma0, double t_k, double t_est,
                   const GTime& g) {
    std::vector<TimeOffsetSample> smp;
    for (const RowSet& s : res.gated) {
      if (s.cls != MeasClass::Dopp) continue;
      for (const MeasRow& r : s.rows) {
        if (!(r.accepted && r.reason == RejectReason::NotRejected)) continue;
        Eigen::Vector3d u = Eigen::Vector3d::Zero();
        for (size_t i = 0; i < g0.geo.size(); ++i)
          if (g0.ok[i] && g0.geo[i].sat.rtk_sat == r.sat.rtk_sat) u = anchor.R_EN.transpose() * g0.geo[i].e_ecef;
        TimeOffsetSample t;
        t.x = u.dot(g0.J.alpha_N);
        t.nu = r.nu;
        t.var = quadForm(r.H, Sigma0) + r.R;
        smp.push_back(t);
      }
    }
    if (smp.empty()) return;
    toff->addEpoch(t_k, smp);
    const TimeOffsetEstimate e = toff->estimate();
    if (e.warn && !toff_warned) {
      char d[128];
      std::snprintf(d, sizeof d, "residual time offset %.4f s (sigma %.4f s, n=%d)", e.offset_s, e.sigma_s, e.n);
      emit(EventKind::TOFF_WARN, t_k, t_est, g, e.offset_s, d);
    }
    toff_warned = e.warn;
  }

  void integrityEdges(double t_k, double t_est, const GTime& g) {
    const IntegrityFlags f = integ->flags();
    if (f.alarm_changed)
      emit(f.livo_alarm ? EventKind::LIVO_ALARM_ON : EventKind::LIVO_ALARM_OFF, t_k, t_est, g, f.d2_velocity,
           f.livo_alarm ? "GNSS updates switch to Schmidt (host columns of K zeroed)" : "");
    if (f.degenerate_changed)
      emit(f.degenerate ? EventKind::DEGENERATE_ON : EventKind::DEGENERATE_OFF, t_k, t_est, g, f.lambda(0), "");
    integ->clearEdges();
    if (f.reinit_yaw && active) {
      char w[96];
      std::snprintf(w, sizeof w, "L7 heading judge, heading error %.3f deg", f.heading_err_deg);
      // A82 livgo.single_init: the frame the first activation left stays; the trigger is only counted (REINIT_SUPPRESSED)
      if (reinitSuppressed(t_k, LivgoReinitBranch::HeadingYaw, w)) return;
      emit(EventKind::REINIT_YAW, t_k, t_est, g, f.heading_err_deg, "L7 heading judge: yaw redone through COLLECT/ALIGN");
      reinit(t_k, "REINIT_YAW");
    }
  }

  void covRecord(const CoreState& core, const HostMat& P_prior, double t_k) {
    CovRecord c;
    c.t_k = t_k;
    c.stage = "postGNSS";
    c.effct_feat_num = -1;
    for (int i = 0; i < HostIdx::dim; ++i) c.host_sigma(i) = sqrtPos(core.P(i, i));
    const Eigen::Matrix3d Pp = core.P.block<3, 3>(HostIdx::pos, HostIdx::pos);
    const Eigen::Matrix3d Pr = core.P.block<3, 3>(HostIdx::rot, HostIdx::rot);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> ep(0.5 * (Pp + Pp.transpose()));
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> er(0.5 * (Pr + Pr.transpose()));
    c.pos_eig = ep.eigenvalues();
    c.pos_eigvec = ep.eigenvectors();
    c.rot_eig = er.eigenvalues();
    c.rot_eigvec = er.eigenvectors();
    c.min_diag_prior = P_prior.diagonal().minCoeff();
    Eigen::SelfAdjointEigenSolver<HostMat> es(0.5 * (P_prior + P_prior.transpose()), Eigen::EigenvaluesOnly);
    const double lmin = es.eigenvalues().minCoeff(), lmax = es.eigenvalues().maxCoeff();
    c.cond_prior = lmin > 0.0 ? lmax / lmin : std::numeric_limits<double>::infinity();
    c.aug_sigma = aug->sigma();
    sink->onCov(c);
  }

  void satRecords(const ObsEpoch& ep, const EpochGeo& g0, const std::vector<MaskVerdict>& masks, const std::set<int>& l1,
                  const std::vector<int>& excluded, const RowBuilder::TdcpPairInput& tin, const std::vector<uint8_t>& det,
                  const JointUpdater::Result& res) {
    const WeekTow wt = weekTow(ep.t_rx);
    for (size_t i = 0; i < ep.sats.size(); ++i) {
      const SatObs& so = ep.sats[i];
      const SignalObs* sig = so.primary();
      SatRecord s;
      s.gpst = wt;
      s.sat = so.sat;
      if (sig) s.cn0 = sig->cn0_dbhz;
      if (g0.ok[i]) {
        s.el_deg = g0.geo[i].el_rad * 180.0 / M_PI;
        s.az_deg = g0.geo[i].az_rad * 180.0 / M_PI;
      }
      s.psr_usable = masks[i].psr;
      s.dopp_usable = masks[i].dopp;
      s.cp_usable = masks[i].cp;
      const RejectReason pre = !g0.ok[i] ? RejectReason::NoEph : RejectReason::Mask;
      s.reason_psr = masks[i].psr ? (l1.count(so.sat.rtk_sat) ? RejectReason::L1 : RejectReason::NotRejected) : pre;
      s.reason_dopp = masks[i].dopp ? (std::find(excluded.begin(), excluded.end(), so.sat.rtk_sat) != excluded.end()
                                           ? RejectReason::Exclusive
                                           : RejectReason::NotRejected)
                                    : pre;
      for (size_t j = 0; j < tin.sats.size(); ++j)
        if (tin.sats[j].rtk_sat == so.sat.rtk_sat) s.slip_detectors = det[j];
      for (const RowSet& set : res.gated) {
        const GateResult& gr = res.gates[ci(set.cls)];
        for (size_t q = 0; q < set.rows.size(); ++q) {
          const MeasRow& r = set.rows[q];
          if (r.sat.rtk_sat != so.sat.rtk_sat) continue;
          const double w = q < gr.w.size() ? gr.w[q] : 0.0;
          if (set.cls == MeasClass::Psr) {
            s.sigma_psr = sqrtPos(r.R);
            s.innov_psr = r.nu;
            s.w_psr = w;
            s.reason_psr = r.reason;
          } else if (set.cls == MeasClass::Dopp) {
            s.sigma_dopp = sqrtPos(r.R);
            s.innov_dopp = r.nu;
            s.w_dopp = w;
            s.reason_dopp = r.reason;
          } else if (set.cls == MeasClass::Tdcp) {
            s.sigma_tdcp = sqrtPos(r.R);
            s.innov_tdcp = r.nu;
            s.w_tdcp = w;
            s.reason_tdcp = r.reason;
          }
        }
      }
      sink->onSat(s);
    }
  }

  // =========================================================================================
  // receiver PVT (L-PVT arm, §4.6)
  // =========================================================================================
  void consumePvt(CoreState& core, const ReceiverPvt& pvt, double t_k, StepReport* rep) {
    if (!active || tight()) return;   // tight arms keep the receiver solution for self-checks only
    const Clock::time_point t0 = Clock::now();
    EpochRecord rec;
    rec.gpst = weekTow(pvt.t);
    rec.t_gpst = pvt.t.toSec();
    rec.t_k = t_k;
    rec.phase = phase;
    rec.observe_only = observe();
    const double t_est = tmap->toEstimator(pvt.t);
    rec.t_est = t_est;
    if (pvt.t_record != 0.0) rec.arrival_latency = pvt.t_record - t_est;
    for (auto& c : rec.cls) c.skipped = true;
    cfg_rows.time.sigma_toff_s = tmap->sigmaToff(pvt.t);
    if (aug && aug->has(Slot::Td)) cfg_rows.time.sigma_toff_s = 0.0;   // A61 F12, as for the epochs
    const int64_t bucket = decimBucket(pvt.t, cfg.noise.pvt.decim_s);
    BridgeResult br;
    const double t_g = (t_est > t_k && t_est <= t_k + kTimeSlackS) ? t_k : t_est;
    if (!bridge->query(t_g, &br)) {
      char d[96];
      std::snprintf(d, sizeof d, "pvt delta=%.4f s", t_k - t_est);
      emit(EventKind::LATE_EPOCH, t_k, t_est, pvt.t, t_k - t_est, d);
      fillStateColumns(core, &rec);
      if (sink) sink->onEpoch(rec);
      return;
    }
    rec.delta = br.delta;
    rec.cross_cycle_bridge = br.cross_cycle;
    if (bucket <= pvt_bucket) {   // A43: first PVT of each GPST second
      ++pvt_decim_skipped;
      fillStateColumns(core, &rec);
      if (sink) sink->onEpoch(rec);
      return;
    }
    pvt_bucket = bucket;
    const double dt_since = last_pvt_used.valid() ? std::max(pvt.t.diff(last_pvt_used), 1e-3) : cfg.noise.pvt.decim_s;
    last_pvt_used = pvt.t;
    const JointUpdater::RowFactory factory = [&](const CoreState& x, const AugBlock& a, std::vector<RowSet>* sets) {
      const LinPoint lp = linPoint(x, a, br);
      const AntennaJacobians J = rows->antennaJacobians(lp);
      RowSet pos, vel;
      rows->pvtRows(pvt, lp, J, dt_since, &pos, &vel);
      sets->push_back(std::move(pos));
      sets->push_back(std::move(vel));
    };
    JointUpdater::Options opt;
    opt.force_three_iterations = reacq_left > 0;
    opt.schmidt = integ->flags().livo_alarm;
    opt.disable_l4_l5 = cfg.ablation.disable_l4_l5;
    opt.observe_only = observe();
    const HostMat P_prior = core.P;
    const JointUpdater::Result res = updater->update(core, *aug, factory, opt);
    if (reacq_left > 0) --reacq_left;
    book.epoch_arrived = true;
    if (pvt.valid_fix) {
      book.n_valid = std::max(book.n_valid, pvt.num_sv);
      last_valid_t_est = t_est;
    }
    if (res.committed && !observe()) {
      ++tot.epochs_updated;
      rep->epochs_committed += 1;
      if (!opt.schmidt) rep->core_modified = true;
    }
    if (res.rolled_back) {
      ++tot.epochs_rolled_back;
      rep->epochs_rolled_back += 1;
      emit(EventKind::ROLLBACK, t_k, t_est, pvt.t, res.nis_post, res.rollback_reason);
    }
    rec.committed = res.committed && !observe();
    rec.rolled_back = res.rolled_back;
    rec.iterations = res.iterations;
    rec.f_p = res.f_p;
    rec.f_xi = res.f_xi;
    rec.f_v = res.f_v;
    rec.dp_host_norm = res.dp_host_norm;
    rec.dyaw = res.dyaw;
    rec.dt_norm = res.dt_norm;
    for (const RowSet& s : res.gated) {
      ClassStats& cs = rec.cls[ci(s.cls)];
      const GateResult& gr = res.gates[ci(s.cls)];
      cs.skipped = gr.class_skipped;
      cs.candidates = static_cast<int>(s.rows.size());
      cs.nis_prior = gr.nis;
      cs.dof = gr.dof;
      for (const MeasRow& r : s.rows) {
        if (r.accepted && r.reason == RejectReason::NotRejected) {
          if (res.rolled_back) ++cs.rejected[ri(RejectReason::Step)];
          else ++cs.accepted;
        } else if (r.reason != RejectReason::NotRejected) {
          ++cs.rejected[ri(r.reason)];
        }
      }
      if (rec.committed && cs.accepted > 0) cs.nis_post = res.nis_post;
      if (cs.accepted == 0) cs.skipped = true;
      tot.cand[ci(s.cls)] += cs.candidates;
      tot.acc[ci(s.cls)] += cs.accepted;
      if (gr.dof > 0) {
        tot.nis[ci(s.cls)] += gr.nis;
        tot.dof[ci(s.cls)] += gr.dof;
      }
    }
    if (rec.committed && sink) covRecord(core, P_prior, t_k);
    rec.elapsed_ms = msSince(t0);
    tot.elapsed.push_back(rec.elapsed_ms);
    fillStateColumns(core, &rec);
    if (sink) sink->onEpoch(rec);
  }

  // =========================================================================================
  // outage / recovery / re-initialisation (§11, L6)
  // =========================================================================================
  void outageStep(double t_k) {
    const OutageDecision d = outage->step(t_k, book.epoch_arrived, book.n_valid, book.l1_consistent, book.frac_psr, book.frac_dopp,
                                          transSigma(), slotSigma(Slot::Yaw), slotSigma(Slot::Clk));
    if (d.reinit) {
      // A82 livgo.single_init: not re-initialised. The manager is told it is done (it would otherwise raise the same decision every
      // cycle and never leave OUTAGE: its reinit branch skips the recovery logic), so it starts over in TRACKING with fresh timers.
      if (reinitSuppressed(t_k, LivgoReinitBranch::Outage, "outage/L6/L7 re-initialisation")) {
        outage->onReinitDone(t_k);
        if (active) setPhase(Phase::TRACKING, t_k, GTime(), "re-initialisation suppressed");
        return;
      }
      reinit(t_k, "outage/L6/L7 re-initialisation");
      return;
    }
    if (d.entered_outage) {
      closePair(true);
      emit(EventKind::OUTAGE_ON, t_k, 0.0, GTime(), d.outage_duration_s, "");
    }
    if (d.left_outage) emit(EventKind::OUTAGE_OFF, t_k, 0.0, GTime(), d.outage_duration_s, "");
    if (d.start_recovery) {
      reacq_left = cfg.iterate.reacq_epochs;
      if (d.clock_reset_needed) pending_clock_reset = true;
      emit(EventKind::RECOVERY, t_k, 0.0, GTime(), d.outage_duration_s, d.clock_reset_needed ? "clock-only reset pending" : "");
    }
    if (d.reanchor_yaw) {
      emit(EventKind::REANCHOR, t_k, 0.0, GTime(), 0.0, "L6 with Doppler rejected too: yaw redone through COLLECT/ALIGN");
      reinit(t_k, "L6 yaw");
      return;
    }
    if (d.reanchor) pending_reanchor = true;
    if (active) {
      const Phase p = (d.phase == Phase::TRACKING || d.phase == Phase::OUTAGE || d.phase == Phase::RECOVERY) ? d.phase : Phase::TRACKING;
      setPhase(p, t_k, GTime(), "");
    }
  }

  void reinit(double t_k, const std::string& why) {
    closePair(true);
    if (integ->flags().livo_alarm) emit(EventKind::LIVO_ALARM_OFF, t_k, 0.0, GTime(), 0.0, "cleared by REINIT");
    aug->clear();
    active = false;
    pending_clock_reset = pending_reanchor = false;
    reacq_left = 0;
    integ->reset();
    toff->reset();
    toff_warned = false;
    init->clearWindow();
    livgo_hist.clear();   // A80: a new collection episode (its travel gate starts again)
    init->setPhase(Phase::COLLECT);
    outage->onReinitDone(t_k);
    emit(EventKind::REINIT, t_k, 0.0, GTime(), 0.0, why);
    setPhase(Phase::COLLECT, t_k, GTime(), "");
  }

  // =========================================================================================
  // global pose
  // =========================================================================================
  // Registration Jacobians of M v (§4.2 columns, host right perturbation / ψ left / φ additive with J_l / δg through ε).
  Eigen::Matrix3d M(const CoreState& core) const { return leveling.M(aug->yaw(), aug->tilt(), core.g); }

  // J (6 × (19+n)) of [δθ_N; δp^N] for R_NI = M R, p^N = M(p − o^W) + t (δθ_N: left perturbation in N).
  Eigen::MatrixXd poseJacobian(const CoreState& core) const {
    const int n = aug->dim();
    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, HostIdx::dim + n);
    const double psi = aug->yaw();
    const Eigen::Vector2d tilt = aug->tilt();
    const Eigen::Matrix3d Rz = Leveling::Rz(psi);
    const Eigen::Matrix3d Mm = leveling.M(psi, tilt, core.g);
    const Eigen::Matrix3d ExpPhi = Leveling::expTilt(tilt);
    const Eigen::Matrix3d Jl = Leveling::tiltLeftJacobian(tilt);
    const Eigen::Vector3d v = core.p - pivot_W;
    const Eigen::Matrix3d Xi = -Rz * skew(Eigen::Vector3d(Rz.transpose() * (Mm * v)));
    J.block<3, 3>(0, HostIdx::rot) = Mm * core.R;
    J.block<3, 3>(3, HostIdx::pos) = Mm;
    if (repivotGravity()) {
      const Eigen::Matrix3d Eps = leveling.epsJacobian(core.g);
      J.block<3, 3>(0, HostIdx::grav) = Rz * ExpPhi * Eps;
      J.block<3, 3>(3, HostIdx::grav) = Xi * ExpPhi * Eps;
    }
    const int ot = aug->find(Slot::Tilt), oy = aug->find(Slot::Yaw), oz = aug->find(Slot::Trans);
    if (ot >= 0) {
      const Eigen::Matrix3d RzJl = Rz * Jl, XiJl = Xi * Jl;
      J.block<3, 2>(0, HostIdx::dim + ot) = RzJl.leftCols<2>();
      J.block<3, 2>(3, HostIdx::dim + ot) = XiJl.leftCols<2>();
    }
    if (oy >= 0) {
      J.block<3, 1>(0, HostIdx::dim + oy) = Eigen::Vector3d::UnitZ();
      J.block<3, 1>(3, HostIdx::dim + oy) = Eigen::Vector3d::UnitZ().cross(Eigen::Vector3d(Mm * v));
    }
    if (oz >= 0) J.block<3, 3>(3, HostIdx::dim + oz) = Eigen::Matrix3d::Identity();
    return J;
  }

  // ---------------- AMENDMENTS A61 F13: odometry output ----------------
  // T_odo = T_off ∘ T_corr with T_corr = [R_z(ψ₀(made) − ψ̂₀), a₀(made) − R_z(·)â₀] from the Origin slot (identity without
  // it): the global output re-expressed as if the registration's estimate at the origin's instant had been exact.
  void odomCorrection(Eigen::Matrix3d* Rc, Eigen::Vector3d* tc) const {
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t = Eigen::Vector3d::Zero();
    const int oo = aug ? aug->find(Slot::Origin) : -1;
    if (oo >= 0) {
      const double dpsi = std::remainder(origin_psi0 - aug->x()(oo + 3), 2.0 * M_PI);
      R = Leveling::Rz(dpsi);
      t = origin_a0 - R * Eigen::Vector3d(aug->x().segment<3>(oo));
    }
    *Rc = odo_off_R * R;
    *tc = odo_off_R * t + odo_off_t;
  }
  // A new origin at this instant — ACTIVATE, and after a REANCHOR (which re-augments t without its history): the
  // stopped clone [a^N; ψ] = [a^N(x̂); ψ̂] (consistent augmentation, no noise), and T_off such that the odometry mapping
  // N_odo ← W continues from the last step end's (T_corr = I now, so T_off = G ∘ ξ̂⁻¹ with ξ̂ = (M, t − M o^W)).
  void odoOrigin(const CoreState& core) {
    removeAllOf(Slot::Origin);
    const int oy = aug->find(Slot::Yaw);
    if (!cfg.output.odom_output || oy < 0 || !aug->has(Slot::Trans) || !leveling.hasRef()) return;
    const AntennaJacobians J = rows->antennaJacobians(linPoint(core, *aug, nowBridge(core)));
    const int n = aug->dim();
    Eigen::MatrixXd F_host = Eigen::MatrixXd::Zero(4, HostIdx::dim), F_aug = Eigen::MatrixXd::Zero(4, n);
    F_host.topRows(3) = J.J_a.leftCols(HostIdx::dim);
    F_aug.topRows(3) = J.J_a.rightCols(n);
    F_aug(3, oy) = 1.0;
    Eigen::VectorXd x0(4);
    x0 << J.a_N, aug->x()(oy);
    aug->addSlot(Slot::Origin, 0, "origin", x0, F_host, F_aug, Eigen::Matrix4d::Zero(), core.P);
    origin_a0 = J.a_N;
    origin_psi0 = aug->x()(oy);
    if (odo_have_G) {
      const Eigen::Matrix3d Mm = M(core);
      odo_off_R = odo_G_R * Mm.transpose();
      odo_off_t = odo_G_t - odo_off_R * (aug->trans() - Mm * pivot_W);
    }
  }
  // Step end: the odometry mapping N_odo ← W of this state, G = T_odo ∘ ξ̂ (what a later origin continues from).
  void odoRefresh(const CoreState& core) {
    if (!active || !aug->has(Slot::Yaw) || !aug->has(Slot::Trans) || !leveling.hasRef()) return;
    Eigen::Matrix3d Rc;
    Eigen::Vector3d tc;
    odomCorrection(&Rc, &tc);
    const Eigen::Matrix3d Mm = M(core);
    odo_G_R = Rc * Mm;
    odo_G_t = Rc * (aug->trans() - Mm * pivot_W) + tc;
    odo_have_G = true;
  }

  void updateSmooth(const CoreState& core, double t_k) {
    const Eigen::Matrix3d Mm = M(core);
    const Eigen::Vector3d b = aug->trans() - Mm * pivot_W;   // x^N = M x^W + b
    if (!smooth_valid) {
      smooth_M = Mm;
      smooth_b = b;
      smooth_valid = true;
    } else {
      const double dt = std::max(0.0, t_k - smooth_t);
      Eigen::Vector3d eR = so3Log(Eigen::Matrix3d(Mm * smooth_M.transpose()));
      const double wmax = cfg.output.smooth_yaw_dps * M_PI / 180.0 * dt;
      if (eR.norm() > wmax) eR *= wmax / eR.norm();
      smooth_M = so3Exp(eR) * smooth_M;
      Eigen::Vector3d ep = (Mm * core.p + b) - (smooth_M * core.p + smooth_b);
      const double vmax = cfg.output.smooth_trans_mps * dt;
      if (ep.norm() > vmax) ep *= vmax / ep.norm();
      smooth_b += ep;
    }
    smooth_t = t_k;
    smooth_R_NI = smooth_M * core.R;
    smooth_t_NI = smooth_M * core.p + smooth_b;
  }

  // Provenance of the A75 switches for events.log; empty when every one is at its pre-A75 value, so the ACTIVATE line of the
  // legacy arms does not change.
  std::string livgoV2Note() const {
    if (!livgoFull()) return std::string();
    if (!liv.frame_exact && liv.noise_mode == LivgoConfig::NoiseMode::Model && liv.dtw_online &&
        liv.lambda_source == LivgoConfig::LambdaSource::Min)
      return std::string();
    char b[200];
    std::snprintf(b, sizeof b, " | livgo_v2 frame_exact=%d noise=%s dtw_online=%d lambda=%s", liv.frame_exact ? 1 : 0,
                  liv.noise_mode == LivgoConfig::NoiseMode::Flat ? "flat" : "model", liv.dtw_online ? 1 : 0,
                  liv.lambda_source == LivgoConfig::LambdaSource::Lio ? "lio" : "min");
    std::string r(b);
    if (liv.noise_mode == LivgoConfig::NoiseMode::Flat) {
      std::snprintf(b, sizeof b, " sigma_tdcp_m=%.6g sigma_dopp_mps=%.6g", liv.flat_sigma_tdcp_m, liv.flat_sigma_dopp_mps);
      r += b;
    }
    return r;
  }

  // Provenance of the A80 switches for events.log; empty when every one is at its pre-A80 value (legacy and v2 ACTIVATE lines unchanged).
  std::string livgoA80Note() const {
    if (!livgoFull() || a80Default()) return std::string();
    char b[400];
    std::snprintf(b, sizeof b,
                  " | livgo_a80 freeze_yaw=%d init_travel_m=%.6g init_tilt_sigma_max_deg=%.6g cn0_hard=%d cn0_min_dbhz=%.6g el_min_deg=%.6g "
                  "primary_only=%d dtw_grid_s=%.6g dtw_window_s=%.6g dtw_min_epochs=%d tdcp_raim_both_ends=%d",
                  yawExact() ? 1 : 0, liv.init_travel_m, liv.init_tilt_sigma_max_deg, liv.cn0_hard ? 1 : 0, liv.cn0_min_dbhz, liv.el_min_deg,
                  liv.primary_only ? 1 : 0, liv.dtw_grid_s, liv.dtw_window_s, liv.dtw_min_epochs, raimBothEnds() ? 1 : 0);
    return std::string(b);
  }

  // Provenance of the A82 switches for events.log and, with dtw_at_init, what the one DTW found and did; empty when both switches are at
  // their pre-A82 value (the ACTIVATE lines of every earlier arm do not change).
  std::string livgoA82Note() const {
    if (!livgoFull() || a82Default()) return std::string();
    char b[640];
    std::snprintf(b, sizeof b, " | livgo_a82 dtw_at_init=%d single_init=%d", dtwAtInit() ? 1 : 0, singleInit() ? 1 : 0);
    std::string r(b);
    if (dtwAtInit() && dtw_init_done) {
      std::snprintf(b, sizeof b,
                    " | livgo_dtw_init lag_s=%.4f adopted=%d applied_lag_s=%.4f score_lag=%.4f score_zero=%.4f sd_gnss=%.4f sd_host=%.4f n=%d "
                    "pairs_lag=%d pairs_zero=%d repaired=%d dropped=%d max_gap_s=%.3f reason=%s",
                    dtw_init.lag_s, dtw_init.adopted ? 1 : 0, dtw_init.applied_lag_s, dtw_init.score_lag, dtw_init.score_zero, dtw_init.sd_gnss,
                    dtw_init.sd_host, dtw_init.n, dtw_init.pairs_lag, dtw_init.pairs_zero, dtw_init_repair.repaired, dtw_init_repair.dropped,
                    dtw_init_repair.max_gap_s, dtw_init.reason);
      r += b;
    }
    return r;
  }

  __attribute__((noinline)) void livgoPin() {
    if (!liv.enable || !livgo_pinned || !aug) return;
    if (frameExact()) aug->pinRegistrationExact(livgo_x_pin, yawExact());   // A75: tilt, translation exact; yaw a state unless A80 freeze_yaw
    else aug->pinRegistration(livgo_x_pin);
  }

  void livgoApplyPaperInit(const AlignResult& a, const CoreState& core) {
    livgo_init_note.clear();
    // A80 init_travel_m: the least squares runs on every synchronous epoch collected (livgo_hist) in the facade's fixed N frame;
    // otherwise on the initializer's window in the alignment's own frame (A73: the two are the same frame at the first activation).
    const bool travel = travelInit();
    const AnchorFrame& fr = travel ? anchor : a.anchor;
    std::vector<LivgoInitSample> samples;
    if (travel) {
      if (fr.valid) samples = livgo_hist.samples(fr.r0_ecef, fr.R_EN);
    } else {
      const std::deque<CollectSample>& src =
          (a.path == "lio_aided" && static_cast<int>(livgo_snap_la.size()) >= 10) ? livgo_snap_la
          : (static_cast<int>(livgo_snap_spp.size()) >= 10 ? livgo_snap_spp : livgo_snap_la);
      samples.reserve(src.size());
      for (const CollectSample& s : src) {
        if (!s.spp.ok || !s.host_ant.p_g.allFinite() || !s.host_ant.R_g.allFinite() || !a.anchor.valid) continue;
        LivgoInitSample e;
        e.p_I = s.host_ant.p_g;
        e.R_I = s.host_ant.R_g;
        e.p_r = a.anchor.R_EN.transpose() * (s.spp.r_ecef - a.anchor.r0_ecef);
        if (!e.p_r.allFinite()) continue;
        samples.push_back(e);
      }
    }
    const double travel_m = livgo_hist.travelM();
    const int oy = aug->find(Slot::Yaw), oz = aug->find(Slot::Trans), ot = aug->find(Slot::Tilt);
    auto done = [&](const std::string& note) {
      livgo_init_note = note;
      livgo_snap_spp.clear();
      livgo_snap_la.clear();
      livgo_hist.clear();   // A80: the epochs are consumed
    };
    if (static_cast<int>(samples.size()) < 10 || oy < 0 || oz < 0 || !leveling.hasRef() || !fr.valid) {
      done(" | livgo_ls=skipped n=" + std::to_string(samples.size()));
      return;
    }
    const Eigen::Vector2d tilt0 = ot >= 0 ? aug->tilt() : Eigen::Vector2d::Zero();
    const Eigen::Matrix3d R0 = leveling.M(aug->yaw(), tilt0, core.g);
    Eigen::Vector3d p_acc = Eigen::Vector3d::Zero();
    for (const LivgoInitSample& s : samples) p_acc += s.p_r - R0 * (s.p_I + s.R_I * lever_I);
    const Eigen::Vector3d p0 = p_acc / static_cast<double>(samples.size());
    LivgoInitEstimate est;
    if (!livgoInitLeastSquares(samples, R0, p0, lever_I, &est)) {
      done(" | livgo_ls=failed " + est.note);
      return;
    }
    const Eigen::Matrix3d C = leveling.C(core.g);
    const Eigen::Matrix3d Q = est.R * C.transpose();
    const double psi = std::atan2(Q(1, 0) - Q(0, 1), Q(0, 0) + Q(1, 1));
    Eigen::Vector3d phi = so3Log(Eigen::Matrix3d(Leveling::Rz(psi).transpose() * Q));
    const Eigen::Matrix3d R_rec = leveling.M(psi, phi.head<2>(), core.g);
    const double recon = (R_rec - est.R).norm();
    // A80 init_tilt_sigma_max_deg: a tilt the least squares cannot determine is not taken; the host gravity gives phi = 0 and only
    // the least squares' yaw, translation (the pivot position below does not depend on the tilt) and lever are kept.
    const Eigen::Vector2d phi_ls = phi.head<2>();
    const LivgoTiltDecision td = livgoTiltDecision(est, liv.init_tilt_sigma_max_deg);
    if (!td.use_ls_tilt) phi.setZero();
    const LivgoInitSample& last = samples.back();
    // The pivot is the antenna at the activation epoch. The window's last sample is that epoch's own sample; the history's last
    // epoch need not be (the epoch can fail the SPP test the history keeps), so with a history the host state at k_a is used.
    const Eigen::Vector3d pivot = travel ? Eigen::Vector3d(core.p + core.R * est.lever) : Eigen::Vector3d(last.p_I + last.R_I * est.lever);
    aug->x()(oy) = psi;
    if (ot >= 0) aug->x().segment<2>(ot) = phi.head<2>();
    aug->x().segment<3>(oz) = est.p + est.R * pivot;
    aug->wrapYaw();
    lever_I = est.lever;
    pivot_W = pivot;
    const double rad2deg = 180.0 / M_PI;
    const double sx = std::sqrt(std::max(0.0, est.cov(0, 0))) * rad2deg;
    const double sy = std::sqrt(std::max(0.0, est.cov(1, 1))) * rad2deg;
    const double sl = std::sqrt(std::max(0.0, est.cov.block<3, 3>(6, 6).diagonal().maxCoeff()));
    char b[512];
    std::snprintf(b, sizeof b,
                  " | livgo_ls n=%d rms_m=%.3f tilt_deg=%.3f,%.3f sigma_rot_deg=%.3f,%.3f "
                  "lever=%.3f,%.3f,%.3f sigma_lever_m=%.3f lambda_min=%.3g recon=%.3g",
                  est.n, est.rms_m, phi_ls.x() * rad2deg, phi_ls.y() * rad2deg, sx, sy, est.lever.x(), est.lever.y(),
                  est.lever.z(), sl, est.lambda_min, recon);
    std::string note(b);
    if (travel || liv.init_tilt_sigma_max_deg > 0.0) {
      // The branch and the formal sigma it was decided on (A80): tilt_branch=ls keeps the least-squares tilt (printed above),
      // tilt_branch=gravity sets phi = 0 (the printed tilt_deg is what the least squares gave and was not used).
      std::snprintf(b, sizeof b, " | livgo_init_a80 tilt_branch=%s tilt_sigma_deg=%.4f tilt_sigma_max_deg=%.4f travel_m=%.1f epochs=%d",
                    td.use_ls_tilt ? "ls" : "gravity", td.sigma_deg, liv.init_tilt_sigma_max_deg, travel_m, est.n);
      note += b;
    }
    done(note);
  }

  void livgoEnsureLog() {
    if (livgo_log || liv.hessian_log.empty()) return;
    livgo_log = std::fopen(liv.hessian_log.c_str(), "w");
    if (livgo_log) std::fprintf(livgo_log, "kind,t_k,lambda_min,lambda_bar,window_n,degraded,chi2\n");
  }

  // A75 revision 1: one row per committed epoch that closed a TDCP pair. trp_* = trace of the host position block (m²) of the prior
  // (the state the GNSS update started from), of the posterior of the epoch, and of the posterior of the epoch without its TDCP
  // rows. status 1 = counterfactual computed, 0 = it rolled back (trp_noT NaN), 2 = no accepted TDCP row (trp_noT = trp_all).
  void livgoLogGain(double t_k, int ph, int n_psr, int n_dopp, int n_tdcp, double tr_prior, double tr_all, double tr_no, int status) {
    if (!gain_log) {
      gain_log = std::fopen(liv.tdcp_gain_log_path.c_str(), "w");
      if (gain_log) std::fprintf(gain_log, "t_k,phase,n_psr,n_dopp,n_tdcp,trp_prior,trp_all,trp_noT,status\n");
    }
    if (!gain_log) return;
    std::fprintf(gain_log, "%.9f,%d,%d,%d,%d,%.12e,%.12e,%.12e,%d\n", t_k, ph, n_psr, n_dopp, n_tdcp, tr_prior, tr_all, tr_no, status);
  }

  void livgoNoteLambda(double lambda_min) {
    if (!liv.enable || !std::isfinite(lambda_min)) return;
    livgo_lambda = lambda_min;
    livgo_lambda_win.push_back(lambda_min);
    const int w = std::max(1, liv.hessian_window);
    if (static_cast<int>(livgo_lambda_win.size()) > w) livgo_lambda_win.erase(livgo_lambda_win.begin());
    const bool full = static_cast<int>(livgo_lambda_win.size()) >= w;
    if (full) {
      double m = 0.0;
      for (double r : livgo_lambda_win) m += r;
      livgo_lambda_bar = m / static_cast<double>(livgo_lambda_win.size());
      const bool deg = livgo_lambda_bar < liv.lambda_th;
      if (deg != livgo_degraded) {
        livgo_degraded = deg;
        ++livgo_switches;
      }
    }
    livgoEnsureLog();
    if (livgo_log) {
      std::fprintf(livgo_log, "hessian,%.9f,%.9g,%.9g,%d,%d,\n", t_k_last, livgo_lambda,
                   full ? livgo_lambda_bar : std::numeric_limits<double>::quiet_NaN(),
                   static_cast<int>(livgo_lambda_win.size()), livgo_degraded ? 1 : 0);
    }
  }

  void livgoLogEpoch(double t_k, bool chi2_on) {
    livgoEnsureLog();
    if (!livgo_log) return;
    std::fprintf(livgo_log, "epoch,%.9f,,,,%d,%d\n", t_k, livgo_degraded ? 1 : 0, chi2_on ? 1 : 0);
  }

  __attribute__((noinline)) void livgoTryDtw(double t_k, const GTime& g) {
    if (!liv.enable || dtw_applied || !tmap) return;
    if (static_cast<int>(dtw_tg.size()) < liv.dtw_min_epochs) return;
    if (dtw_tg.size() != dtw_vg.size() || dtw_th.size() != dtw_vh.size()) return;
    double lag = 0.0;
    LivgoDtwParams dp;   // A80: the grid and window are livgo/dtw_grid_s and dtw_window_s (defaults = the earlier constants)
    dp.grid_s = liv.dtw_grid_s;
    dp.window_s = liv.dtw_window_s;
    if (!livgoDtwLag(dtw_tg, dtw_vg, dtw_th, dtw_vh, &lag, dp)) return;
    tmap->addOffset(lag);
    dtw_applied = true;
    char d[96];
    std::snprintf(d, sizeof d, "DTW residual lag added to the time map: %.4f s (n=%zu)", lag, dtw_tg.size());
    emit(EventKind::TOFF_WARN, t_k, 0.0, g, lag, d);
  }

  // AMENDMENTS A82 livgo.dtw_at_init: the one DTW of the run, made when the travel gate opens, on every synchronous epoch of the frame
  // least squares' history (the epochs with both speeds). The acceptance test of the author's code (livgoDtwInit) decides; a lag that
  // is taken goes into the GNSS time map and the history's host poses are re-paired at the compensated times before the least squares.
  // Not taken: the offset stays 0 and nothing else changes. The result is in ACTIVATE's detail; TOFF_WARN says it as soon as it is made.
  void livgoDtwAtInit(double t_k, const GTime& g) {
    dtw_init_done = true;
    const LivgoSpeedSeries sr = livgo_hist.speedSeries();
    dtw_init = livgoDtwInit(sr.t, sr.v_gnss, sr.v_host, liv.dtw_grid_s);
    dtw_init_repair = LivgoRepairResult();
    if (dtw_init.adopted) {
      tmap->addOffset(dtw_init.applied_lag_s);
      dtw_init_repair = livgo_hist.repairPairing(dtw_init.applied_lag_s);
    }
    char d[320];
    std::snprintf(d, sizeof d,
                  "DTW at init: lag %.4f s %s (n=%d, score %.3f, zero-lag score %.3f, sd %.3f/%.3f m/s, %s); history re-paired %d, dropped %d",
                  dtw_init.lag_s, dtw_init.adopted ? "taken" : "not taken", dtw_init.n, dtw_init.score_lag, dtw_init.score_zero, dtw_init.sd_gnss,
                  dtw_init.sd_host, dtw_init.reason, dtw_init_repair.repaired, dtw_init_repair.dropped);
    emit(EventKind::TOFF_WARN, t_k, 0.0, g, dtw_init.applied_lag_s, d);
  }

  // AMENDMENTS A82 livgo.single_init: false = the re-initialisation may run. true = it was suppressed (counted, and an event says which branch).
  bool reinitSuppressed(double t_k, LivgoReinitBranch b, const std::string& what) {
    if (reinit_gate.allow(b)) return false;
    char d[200];
    std::snprintf(d, sizeof d, "single_init: branch=%s suppressed=%d total=%d | ", LivgoReinitGate::name(b), reinit_gate.suppressed(b),
                  reinit_gate.total());
    emit(EventKind::REINIT_SUPPRESSED, t_k, 0.0, GTime(), static_cast<double>(reinit_gate.total()), std::string(d) + what);
    return true;
  }
};

// =============================================================================================
// public interface
// =============================================================================================
GnssFusionLivgo::GnssFusionLivgo(const Config& cfg, const LivgoConfig& livgo) : impl_(new Impl(cfg)) {
  const std::vector<std::string> problems = validateConfig(cfg);
  if (!problems.empty()) {
    std::string msg = "GnssFusionLivgo: invalid configuration:";
    for (const auto& p : problems) msg += "\n  " + p;
    throw std::invalid_argument(msg);
  }
  Impl& m = *impl_;
  m.liv = livgo;
  if (m.liv.enable && m.liv.noise_mode == LivgoConfig::NoiseMode::Flat &&
      !(std::isfinite(m.liv.flat_sigma_tdcp_m) && m.liv.flat_sigma_tdcp_m > 0.0 && std::isfinite(m.liv.flat_sigma_dopp_mps) &&
        m.liv.flat_sigma_dopp_mps > 0.0))
    throw std::invalid_argument("GnssFusionLivgo: livgo.noise_mode = flat needs flat_sigma_tdcp_m > 0 and flat_sigma_dopp_mps > 0");
  if (m.liv.enable) {
    // A80: the switches that take a number must get a usable one (a typo must not run a different arm silently).
    auto finiteNonNeg = [](double v) { return std::isfinite(v) && v >= 0.0; };
    if (!finiteNonNeg(m.liv.init_travel_m)) throw std::invalid_argument("GnssFusionLivgo: livgo.init_travel_m must be finite and >= 0");
    if (!finiteNonNeg(m.liv.init_tilt_sigma_max_deg))
      throw std::invalid_argument("GnssFusionLivgo: livgo.init_tilt_sigma_max_deg must be finite and >= 0");
    if (!(std::isfinite(m.liv.dtw_grid_s) && m.liv.dtw_grid_s > 0.0 && std::isfinite(m.liv.dtw_window_s) &&
          m.liv.dtw_window_s / m.liv.dtw_grid_s >= 20.0 - 1e-9))
      throw std::invalid_argument("GnssFusionLivgo: livgo.dtw_grid_s must be > 0 and livgo.dtw_window_s at least 20 grid steps");
    if (m.liv.cn0_hard && !(std::isfinite(m.liv.cn0_min_dbhz) && m.liv.cn0_min_dbhz >= 0.0 && m.liv.cn0_min_dbhz <= 70.0 &&
                            std::isfinite(m.liv.el_min_deg) && m.liv.el_min_deg >= 0.0 && m.liv.el_min_deg < 90.0))
      throw std::invalid_argument("GnssFusionLivgo: livgo.cn0_hard needs cn0_min_dbhz in [0,70] and el_min_deg in [0,90)");
  }
  m.reinit_gate.setSingleInit(m.livgoFull() && m.liv.single_init);   // A82
  m.enabled = cfg.enable;
  if (m.enabled && m.liv.enable && !m.liv.dual_only) {
    // Paper update set: Doppler + fixed-anchor TDCP. Pseudorange stays available to the SPP/RAIM path
    // (use.psr only gates the filter rows). Elevation is raised to the paper floor when the overlay is looser.
    m.cfg.use.psr = false;
    m.cfg.ablation.tdcp_fixed_anchor = true;
    m.cfg.noise.dopp.project_tilt = false;
    m.cfg.noise.dopp.consider_tilt = false;
    // Frozen W→ENU: the registration random walk is off. Done on the config copy so AugBlock::processModel
    // itself is unchanged when the arm is off (that function is on T's numeric path).
    m.cfg.process.frame.q_tilt_per_m = 0.0;
    m.cfg.process.frame.q_yaw_per_m = 0.0;
    m.cfg.process.frame.q_yaw_per_rad = 0.0;
    m.cfg.process.frame.q_th_per_m = 0.0;
    m.cfg.process.frame.q_tv_per_m = 0.0;
    m.cfg.process.frame.q_time = 0.0;
    livgoApplyMasks(m.liv, &m.cfg.masks);   // A80 cn0_hard: the paper's RAIM pre-screen as written; otherwise el_min_deg only raises
    // Without cn0_hard C/N0 is not raised to livgo.cn0_min_dbhz. On Grass01 every range_meas CN0 is below 35 dB-Hz
    // (max 32, median about 26), so a hard floor of 35 admits no satellite and COLLECT never
    // initialises. The paper's wording is "much lower than 35". This receiver keeps the frozen
    // A58 floors (the development-set tail change points already in the base yaml).
    m.cfg_rows = m.cfg;
  }
  if (!m.enabled) return;   // zero channel (DESIGN §13): nothing is built, nothing global is touched

  // A43 is applied by the facade (the first epoch of every ⌊t/decim_s⌋ bin reaches the update's RowBuilder); the
  // RowBuilder's own EpochDecimator (rows.cpp) bins with the same decim_s and so takes every epoch it is given, and κ_AR
  // keeps the nominal decim_s spacing (P3 integration: the former decim_s = 0 work-around is gone with the 5 ms grid).
  m.lever_I = Eigen::Vector3d(cfg.frames.lever_arm_xyz[0], cfg.frames.lever_arm_xyz[1], cfg.frames.lever_arm_xyz[2]);
  m.nav.reset(new rtk::NavStore());
  m.ephem.reset(new EphemStore(m.cfg, *m.nav));   // sets RTKLIB's Galileo I/NAV selection (A16)
  m.solver.reset(new rtk::SatSolver(*m.nav));
  m.tmap.reset(new TimeMap(m.cfg.time));
  if (!m.cfg.time.calib_file.empty()) {
    std::string why;
    if (!m.tmap->loadCalibFile(m.cfg.time.calib_file, &why)) throw std::invalid_argument("GnssFusionLivgo: time calibration: " + why);
  }
  m.bridge.reset(new ImuBridge(m.cfg));
  m.aug.reset(new AugBlock(m.cfg));
  m.noise.reset(new NoiseModel(m.cfg_rows));
  m.rows.reset(new RowBuilder(m.cfg_rows, *m.noise));
  if (m.flatNoise()) {
    // A75 livgo.noise_mode = flat: the filter rows (not RAIM, DTW or the initialisation, which keep `noise`) take one σ per
    // class plus the IMU-bridge terms; the Config handed to the builder has noise.scale 1 so its bridge cross terms (R_full)
    // are unscaled like the diagonal.
    m.cfg_flat = m.cfg_rows;
    m.cfg_flat.noise.scale.dopp = 1.0;
    m.cfg_flat.noise.scale.tdcp = 1.0;
    m.noise_flat.reset(new NoiseModel(m.cfg_flat));
    m.noise_flat->flat.on = true;
    m.noise_flat->flat.sigma_dopp_mps = m.liv.flat_sigma_dopp_mps;
    m.noise_flat->flat.sigma_tdcp_m = m.liv.flat_sigma_tdcp_m;
    m.rows.reset(new RowBuilder(m.cfg_flat, *m.noise_flat));
  }
  m.updater.reset(new JointUpdater(m.cfg));
  m.gate.reset(new QualityGate(m.cfg));
  m.track.reset(new TrackCounter(m.cfg.masks));
  m.slip.reset(new SlipDetector(m.cfg.slip));
  m.jump.reset(new ClockJumpDetector(m.cfg.rejection));
  m.spp.reset(new RobustSpp(m.cfg, *m.noise));
  m.dls.reset(new RobustDopplerLs(m.cfg, *m.noise));
  m.toff.reset(new TimeOffsetMonitor(m.cfg.time));
  m.init.reset(new Initializer(m.cfg, m.leveling));
  m.outage.reset(new OutageManager(m.cfg.outage, m.cfg.rejection, /*l6_enabled=*/m.cfg.use.psr));   // L6 off in T−pr
  m.integ.reset(new IntegrityMonitor(m.cfg.integrity));
  m.outage->setReacqEpochs(m.cfg.iterate.reacq_epochs);   // P3-A1: the RECOVERY window length
  for (const std::string& path : m.cfg.nav_rinex) {
    std::string why;
    const int n = m.ephem->loadRinex(path, &why);
    if (n < 0) throw std::invalid_argument("GnssFusionLivgo: nav_rinex " + path + ": " + why);
  }
  m.init->setPhase(Phase::WAIT_CORE);
  m.phase = Phase::WAIT_CORE;
  if (m.cfg.mode == Config::Mode::LivgoProbe || m.cfg.mode == Config::Mode::LivgoProbeNorej)
    m.emit(EventKind::CONFIG_WARNING, 0.0, 0.0, GTime(), 0.0,
           "mode livgo_probe*: the FAST-LIVGO probe arms are not part of the evaluation (AMENDMENTS A46); run as tight + ablation flags");
  if (m.cfg.ablation.frame_fixed_after_init && !m.liv.enable)
    m.emit(EventKind::CONFIG_WARNING, 0.0, 0.0, GTime(), 0.0, "ablation.frame_fixed_after_init is not implemented (A46): ignored");
  // A80: a switch that the arm as configured cannot honour is said once at construction instead of being ignored silently.
  if (m.liv.enable) {
    auto warn = [&](const std::string& what) {
      m.emit(EventKind::CONFIG_WARNING, 0.0, 0.0, GTime(), 0.0, "livgo: " + what + ": ignored");
    };
    if (m.liv.dual_only && (!m.a80Default() || !m.a82Default())) warn("the A80/A81/A82 switches need the full arm, not livgo.dual_only");
    if (!m.liv.dual_only) {
      if (m.liv.freeze_yaw && !m.liv.frame_exact) warn("freeze_yaw without frame_exact");
      if (!m.liv.paper_init && m.liv.init_travel_m > 0.0) warn("init_travel_m without paper_init");
      if (!m.liv.paper_init && m.liv.init_tilt_sigma_max_deg > 0.0) warn("init_tilt_sigma_max_deg without paper_init");
      // A82: the DTW at the frame initialisation needs the DTW and the travel gate whose history it runs on
      if (m.liv.dtw_at_init && !m.liv.dtw_online) warn("dtw_at_init without dtw_online");
      if (m.liv.dtw_at_init && (!m.liv.paper_init || !(m.liv.init_travel_m > 0.0))) warn("dtw_at_init without paper_init and init_travel_m > 0");
    }
  }
}

GnssFusionLivgo::~GnssFusionLivgo() = default;

void GnssFusionLivgo::setExtrinsics(const Eigen::Matrix3d& R_IL, const Eigen::Vector3d& t_IL) {
  Impl& m = *impl_;
  if (!m.enabled) return;
  const Eigen::Vector3d l(m.cfg.frames.lever_arm_xyz[0], m.cfg.frames.lever_arm_xyz[1], m.cfg.frames.lever_arm_xyz[2]);
  // DESIGN §1 "A": l^I = R_{I←L} l^L + t_{I←L} for a lever arm given in the LiDAR frame (§14: [-0.358,0.086,0.429] → l^I).
  m.lever_I = (m.cfg.frames.lever_arm_frame == Config::Frames::LeverArmFrame::Lidar) ? Eigen::Vector3d(R_IL * l + t_IL) : l;
  m.extrinsics_set = true;
}

bool GnssFusionLivgo::setSlamMode(SlamMode mode) {
  Impl& m = *impl_;
  if (mode == SlamMode::ONLY_LO) return false;   // no IMU bridge (§9)
  m.slam_mode = mode;
  return true;
}

void GnssFusionLivgo::setHostFlags(bool gravity_est_en, bool exposure_est_en) {
  Impl& m = *impl_;
  m.host_gravity_est = gravity_est_en;
  m.host_expo_est = exposure_est_en;
  if (m.init) m.init->setGravityEstimated(gravity_est_en);   // P3-A1: δg column of J_trans_host
}

void GnssFusionLivgo::setDiagnosticsSink(DiagnosticsSink* sink) {
  Impl& m = *impl_;
  if (!m.enabled) return;
  m.sink = sink;
  if (sink) {
    for (const Event& e : m.pending_events) sink->onEvent(e);
    m.pending_events.clear();
  }
}

void GnssFusionLivgo::setRawDecoder(RawDecoder* decoder) {
  if (!impl_->enabled) return;
  impl_->decoder = decoder;
}

bool GnssFusionLivgo::registerMessageDefinition(Source src, const std::string& datatype, const std::string& md5, const std::string& msg_def) {
  Impl& m = *impl_;
  if (!m.enabled) return true;
  std::string why;
  const bool ok = m.decoder && m.decoder->registerMessageDefinition(src, datatype, md5, msg_def, &why);
  if (!ok) {
    if (!m.decoder) why = "no raw decoder registered";
    m.emit(EventKind::MSGDEF_REJECT, m.t_k_last, 0.0, GTime(), 0.0, std::string(sourceName(src)) + " " + datatype + ": " + why);
  }
  return ok;
}

void GnssFusionLivgo::feedBytes(Source src, const uint8_t* data, size_t size, double t_record) {
  Impl& m = *impl_;
  if (!m.enabled || !m.decoder) return;
  std::string why;
  m.decoder->decode(src, data, size, t_record, *this, &why);   // malformed buffers are counted by the decoder (U2)
}

void GnssFusionLivgo::pushObservation(ObsEpoch&& epoch) {
  Impl& m = *impl_;
  if (!m.enabled || !epoch.t_rx.valid()) return;
  Impl::Pending p;
  p.is_pvt = false;
  p.t = epoch.t_rx;
  p.obs = std::move(epoch);
  m.enqueue(std::move(p));
}

void GnssFusionLivgo::pushEphemeris(const KeplerEph& eph) {
  Impl& m = *impl_;
  if (!m.enabled) return;
  m.ephem->addBag(eph);
}

void GnssFusionLivgo::pushGloEphemeris(const GloEph& eph) {
  Impl& m = *impl_;
  if (!m.enabled) return;
  m.ephem->addBag(eph);
}

void GnssFusionLivgo::pushIono(const Klobuchar& iono) {
  Impl& m = *impl_;
  if (!m.enabled) return;
  if (iono.source == Source::UbxStream) {
    // AMENDMENTS A45: the GPS subframe set decoded from UBX is the real GPS Klobuchar.
    m.ubx_iono = iono;
    m.have_ubx_iono = iono.valid();
  } else {
    m.ephem->setIonoFromBag(iono);   // A38: gnss_comm iono_params are a diagnostic only
  }
}

void GnssFusionLivgo::pushPvt(const ReceiverPvt& pvt) {
  Impl& m = *impl_;
  if (!m.enabled || !pvt.t.valid() || m.tight()) return;   // tight arms: the receiver solution is not a measurement
  Impl::Pending p;
  p.is_pvt = true;
  p.t = pvt.t;
  p.pvt = pvt;
  m.enqueue(std::move(p));
}

int GnssFusionLivgo::loadNavRinex(const std::string& path, std::string* why) {
  Impl& m = *impl_;
  if (!m.enabled) {
    if (why) *why = "gnss.enable = false";
    return 0;
  }
  return m.ephem->loadRinex(path, why);
}

StepReport GnssFusionLivgo::onLioStep(CoreState& core, const HostMat& F_tot, const std::vector<BridgeSample>& bridge, double t_prev,
                                 double t_k) {
  StepReport rep;
  Impl& m = *impl_;
  if (!m.enabled) return rep;
  // VIO cycles (no IMU samples, zero propagation) and malformed calls are no-ops (§9 "钩子空操作").
  if (bridge.empty() || !(t_k > t_prev)) return rep;
  const Clock::time_point t0 = Clock::now();
  const CoreState entry = core;
  m.beginStep(core, F_tot, bridge, t_prev, t_k, &rep);
  std::vector<Impl::Pending> due = m.popDue(t_k);
  if (m.cfg.update_slot == Config::UpdateSlot::PostVio) {
    // A12: only H1 and bookkeeping here; the epochs of this cycle wait for onPostVio.
    for (auto& p : due) m.held.push_back(std::move(p));
  } else {
    m.processBatch(core, t_k, due, &rep);
  }
  m.g_last = core.g;
  if (m.observe() && !sameBits(entry, core)) {
    core = entry;
    m.emit(EventKind::CONFIG_WARNING, t_k, 0.0, GTime(), 0.0, "observe_only: a host write was suppressed (bug)");
  }
  if (m.observe()) {
    rep.core_modified = false;
  } else if (!sameBits(entry, core)) {
    rep.core_modified = true;   // whatever rewrote the host copy, the glue must write it back (it only does on this flag)
  }
  rep.elapsed_ms = msSince(t0);
  return rep;
}

void GnssFusionLivgo::setLivgoLambdaMin(double lambda_min) {
  Impl& m = *impl_;
  if (!m.enabled) return;
  m.livgoNoteLambda(lambda_min);
}

void GnssFusionLivgo::onCoreUpdated(const CoreState& prior, const CoreState& post, UpdateSource src) {
  Impl& m = *impl_;
  if (!m.enabled) return;
  // Pin before H2. The test is on livgo_pinned, which stays false for every arm except LIVGO after ACTIVATE,
  // so T never calls into pinRegistration.
  if (m.livgo_pinned) m.livgoPin();
  if (src == UpdateSource::LIO) {
    ++m.host_lio_updates;
    m.integ->onLioCovariance(prior.P, post.P);   // degeneracy judge (§10 last row)
    const IntegrityFlags f = m.integ->flags();
    if (f.degenerate_changed)
      m.emit(f.degenerate ? EventKind::DEGENERATE_ON : EventKind::DEGENERATE_OFF, m.t_k_last, 0.0, GTime(), f.lambda(0), "");
    // Every edge is consumed right after the call that raised it (the velocity-alarm edge in the epoch loop), so
    // clearing both here cannot lose one.
    m.integ->clearEdges();
  }
  if (m.aug->dim() == 0) return;
  // A61 F9: is this update a jump of the map frame? (decided on the state before H2 touches x_G)
  const Impl::FrameJumpProbe fjp = m.frameJumpProbe(prior, post);
  // H2 (§2.4): exact conditioning of x_G, C, Q_G on the host's own update.
  const HostVec dx = post.boxminus(prior);
  bool pinv = false;
  if (!m.aug->conditionOnHostUpdate(prior.P, post.P, dx, &pinv)) {
    m.emit(EventKind::PSEUDOINVERSE, m.t_k_last, 0.0, GTime(), 0.0, "H2 conditioning failed: re-initialising");
    // A82 livgo.single_init: not re-initialised; the augmented block keeps its state of before this update (conditionOnHostUpdate
    // returns false before it writes anything), so this one host update is not conditioned into it.
    if (m.active && !m.reinitSuppressed(m.t_k_last, LivgoReinitBranch::ConditioningFailed, "H2 conditioning failed")) m.reinit(m.t_k_last, "H2 conditioning failed");
    return;
  }
  if (pinv) m.emit(EventKind::PSEUDOINVERSE, m.t_k_last, 0.0, GTime(), 0.0, src == UpdateSource::LIO ? "H2(LIO)" : "H2(VIO)");
  m.frameJumpApply(fjp, post);
}

StepReport GnssFusionLivgo::onPostVio(CoreState& core, double t_k) {
  StepReport rep;
  Impl& m = *impl_;
  if (!m.enabled || m.cfg.update_slot != Config::UpdateSlot::PostVio) return rep;
  const Clock::time_point t0 = Clock::now();
  const CoreState entry = core;
  std::vector<Impl::Pending> batch;
  batch.swap(m.held);
  m.processBatch(core, t_k, batch, &rep);
  if (m.observe() && !sameBits(entry, core)) {
    core = entry;
    m.emit(EventKind::CONFIG_WARNING, t_k, 0.0, GTime(), 0.0, "observe_only: a host write was suppressed (bug)");
  }
  if (m.observe()) {
    rep.core_modified = false;
  } else if (!sameBits(entry, core)) {
    rep.core_modified = true;   // whatever rewrote the host copy, the glue must write it back (it only does on this flag)
  }
  rep.elapsed_ms = msSince(t0);
  return rep;
}

void GnssFusionLivgo::onStepEnd(const CoreState& core, double t_k) {
  Impl& m = *impl_;
  if (!m.enabled) return;
  // C_ref follows the final gravity of the step (§1); M is unchanged at this g, only its δg linearisation moves.
  m.leveling.setRef(core.g);
  m.g_last = core.g;
  if (!m.active || !m.aug->has(Slot::Trans)) return;
  // §2.3 re-pivot, right after the refresh so the δg column of T is exact (aug_block.hpp). The smoothed output keeps
  // x^N = M x^W + b with b = t − M o^W, which a re-pivot leaves unchanged.
  const Eigen::Vector3d a_W = core.p + core.R * m.lever_I;
  const double dist = (a_W - m.pivot_W).norm();
  if (dist > m.cfg.frames.pivot_distance_m) {
    m.aug->repivot(m.pivot_W, a_W, m.leveling, core.g, core.P, m.repivotGravity());
    m.pivot_W = a_W;
    // Re-pivot rewrites the translation state but leaves M unchanged. The LIVGO pin must follow that
    // reparameterisation, or the next pin would restore the pre-pivot mean against the new pivot.
    if (m.livgo_pinned) m.livgo_x_pin = m.aug->x();
    // frame_exact: the re-pivot ties the translation to the yaw (∂t'/∂δψ = [e_z]×M(o' − o)) and to the host gravity; the exact
    // constant is the translation at the *new* pivot, so it is pinned now, before the next GNSS epoch's S is built (the legacy
    // arm pins at the next update/H2, which for pre_lio is after that epoch).
    if (m.frameExact() && m.livgo_pinned) m.livgoPin();
    m.emit(EventKind::REPIVOT, t_k, 0.0, GTime(), dist, "");
  }
  m.updateSmooth(core, t_k);
  m.odoRefresh(core);   // A61 F13
}

bool GnssFusionLivgo::globalPose(const CoreState& core, Eigen::Matrix3d* R_NI, Eigen::Vector3d* t_NI, Eigen::Matrix<double, 6, 6>* cov) const {
  const Impl& m = *impl_;
  if (!m.enabled || !m.active || !m.aug->has(Slot::Yaw) || !m.aug->has(Slot::Trans) || !m.leveling.hasRef()) return false;
  const Eigen::Matrix3d Mm = m.M(core);
  if (R_NI) *R_NI = Mm * core.R;
  if (t_NI) *t_NI = Mm * (core.p - m.pivot_W) + m.aug->trans();
  if (cov) {
    const Eigen::MatrixXd J = m.poseJacobian(core);
    const Eigen::MatrixXd S = m.aug->jointCov(core.P);
    Eigen::Matrix<double, 6, 6> c = J * S * J.transpose();
    *cov = 0.5 * (c + c.transpose());
  }
  return true;
}

bool GnssFusionLivgo::antennaN(const CoreState& core, Eigen::Vector3d* a_N, Eigen::Matrix3d* cov) const {
  const Impl& m = *impl_;
  if (!m.enabled || !m.active || !m.aug->has(Slot::Trans) || !m.leveling.hasRef()) return false;
  const BridgeResult br = Impl::nowBridge(core);
  const AntennaJacobians J = m.rows->antennaJacobians(m.linPoint(core, *m.aug, br));
  if (a_N) *a_N = J.a_N;
  if (cov) {
    const Eigen::MatrixXd S = m.aug->jointCov(core.P);
    const Eigen::Matrix3d c = J.J_a * S * J.J_a.transpose();
    *cov = 0.5 * (c + c.transpose());
  }
  return true;
}

bool GnssFusionLivgo::globalPoseOdom(const CoreState& core, Eigen::Matrix3d* R_NI, Eigen::Vector3d* t_NI) const {
  Eigen::Matrix3d R;
  Eigen::Vector3d t;
  if (!globalPose(core, &R, &t, nullptr)) return false;
  Eigen::Matrix3d Rc;
  Eigen::Vector3d tc;
  impl_->odomCorrection(&Rc, &tc);
  if (R_NI) *R_NI = Rc * R;
  if (t_NI) *t_NI = Rc * t + tc;
  return true;
}

bool GnssFusionLivgo::antennaOdom(const CoreState& core, Eigen::Vector3d* a_N) const {
  Eigen::Vector3d a;
  if (!antennaN(core, &a, nullptr)) return false;
  Eigen::Matrix3d Rc;
  Eigen::Vector3d tc;
  impl_->odomCorrection(&Rc, &tc);
  if (a_N) *a_N = Rc * a + tc;
  return true;
}

bool GnssFusionLivgo::globalPoseSmooth(Eigen::Matrix3d* R_NI, Eigen::Vector3d* t_NI) const {
  const Impl& m = *impl_;
  if (!m.enabled || !m.active || !m.smooth_valid) return false;
  if (R_NI) *R_NI = m.smooth_R_NI;
  if (t_NI) *t_NI = m.smooth_t_NI;
  return true;
}

bool GnssFusionLivgo::anchor(Geodetic* geo) const {
  const Impl& m = *impl_;
  if (!m.enabled || !m.anchor.valid) return false;
  if (geo) *geo = m.anchor.geo;
  return true;
}

bool GnssFusionLivgo::anchor(AnchorFrame* frame) const {
  const Impl& m = *impl_;
  if (!m.enabled || !m.anchor.valid) return false;
  if (frame) *frame = m.anchor;
  return true;
}

bool GnssFusionLivgo::registration(Registration* xi) const {
  const Impl& m = *impl_;
  if (!m.enabled || !m.active) return false;
  if (!xi) return true;
  Registration r;
  r.active = true;
  r.tilt = m.aug->tilt();
  r.yaw = m.aug->yaw();
  r.trans = m.aug->trans();
  r.pivot_w = m.pivot_W;
  // M at the gravity of the last step (onStepEnd / onLioStep); identity before the first one.
  r.M = (m.leveling.hasRef() && m.g_last.norm() > 0.0) ? m.leveling.M(r.yaw, r.tilt, m.g_last) : Eigen::Matrix3d::Identity();
  const int ot = m.aug->find(Slot::Tilt), oy = m.aug->find(Slot::Yaw), oz = m.aug->find(Slot::Trans);
  const int idx[6] = {ot, ot < 0 ? -1 : ot + 1, oy, oz, oz < 0 ? -1 : oz + 1, oz < 0 ? -1 : oz + 2};
  for (int a = 0; a < 6; ++a)
    for (int b = 0; b < 6; ++b) r.cov(a, b) = (idx[a] >= 0 && idx[b] >= 0) ? m.aug->Q()(idx[a], idx[b]) : 0.0;
  *xi = r;
  return true;
}

bool GnssFusionLivgo::lastSpp(Eigen::Vector3d* r_ecef, Eigen::Matrix3d* cov, GTime* t) const {
  const Impl& m = *impl_;
  if (!m.enabled || !m.have_spp) return false;
  if (r_ecef) *r_ecef = m.last_spp.r_ecef;
  if (cov) *cov = m.last_spp.Q.rows() >= 3 ? Eigen::Matrix3d(m.last_spp.Q.topLeftCorner<3, 3>()) : Eigen::Matrix3d::Zero();
  if (t) *t = m.last_spp.t;
  return true;
}

FusionStatus GnssFusionLivgo::status() const {
  const Impl& m = *impl_;
  FusionStatus s;
  if (!m.enabled) return s;
  s.phase = m.phase;
  s.active = m.active;
  s.degenerate = m.integ->flags().degenerate;
  s.livo_alarm = m.integ->flags().livo_alarm;
  s.schmidt = m.active && s.livo_alarm;
  s.n_slots = static_cast<int>(m.aug->slots().size());
  s.aug_dim = m.aug->dim();
  s.n_sats_last = m.n_sats_last;
  s.last_epoch = m.last_epoch_t;
  s.last_epoch_t_est = m.last_epoch_t_est;
  s.epochs_processed = m.tot.epochs_total;
  s.epochs_committed = m.tot.epochs_updated;
  s.epochs_rolled_back = m.tot.epochs_rolled_back;
  s.sigma_yaw_rad = m.slotSigma(Slot::Yaw);
  s.sigma_t_m = m.transSigma();
  s.sigma_b_m = m.slotSigma(Slot::Clk);
  s.tdcp_pairs_open = m.pair_open ? 1 : 0;
  s.outage_s = (m.active && std::isfinite(m.last_valid_t_est) && std::isfinite(m.t_k_last)) ? std::max(0.0, m.t_k_last - m.last_valid_t_est) : 0.0;
  return s;
}

void GnssFusionLivgo::augmentedState(Eigen::VectorXd* x_G, Eigen::VectorXd* diag_Q_G, std::vector<std::string>* slots) const {
  const Impl& m = *impl_;
  if (!m.enabled) {
    if (x_G) x_G->resize(0);
    if (diag_Q_G) diag_Q_G->resize(0);
    if (slots) slots->clear();
    return;
  }
  if (x_G) *x_G = m.aug->x();
  if (diag_Q_G) *diag_Q_G = m.aug->Q().diagonal();
  if (slots) {
    slots->clear();
    for (const SlotInfo& s : m.aug->slots()) slots->push_back(s.name);   // one name per slot, components consecutive in x_G
  }
}

const Config& GnssFusionLivgo::config() const { return impl_->cfg; }

Summary GnssFusionLivgo::summary() const {
  const Impl& m = *impl_;
  Summary s;
  if (!m.enabled) return s;
  const Impl::Totals& t = m.tot;
  s.epochs_total = t.epochs_total;
  s.epochs_updated = t.epochs_updated;
  s.epochs_rolled_back = t.epochs_rolled_back;
  for (size_t c = 0; c < s.mean_nis_per_dof.size(); ++c) {
    s.mean_nis_per_dof[c] = t.dof[c] > 0.0 ? t.nis[c] / t.dof[c] : kNaN;
    s.acceptance_rate[c] = t.cand[c] > 0.0 ? t.acc[c] / t.cand[c] : kNaN;
  }
  s.tdcp_coverage = t.active_s > 0.0 ? t.tdcp_open_s / t.active_s : 0.0;
  if (!t.elapsed.empty()) {
    std::vector<double> e = t.elapsed;
    std::sort(e.begin(), e.end());
    auto q = [&](double p) { return e[std::min(e.size() - 1, static_cast<size_t>(std::floor(p * (e.size() - 1) + 0.5)))]; };
    s.elapsed_p50_ms = q(0.50);
    s.elapsed_p95_ms = q(0.95);
    s.elapsed_p99_ms = q(0.99);
  }
  s.n_activate = t.n_activate;
  s.n_reinit = t.n_reinit;
  s.n_reanchor = t.n_reanchor;
  s.n_clock_jump = t.n_clock_jump;
  s.n_slip = t.n_slip;
  s.n_late = t.n_late;
  s.n_msgdef_reject = t.n_msgdef_reject;
  s.n_bds_b1c_reject = t.n_bds_b1c;
  s.n_gal_prn_gt36 = t.n_gal_prn + m.ephem->rejectedGalPrn();
  s.n_glo_fcn_mismatch = t.n_glo_fcn + m.ephem->rejectedGloFcn();
  return s;
}

void GnssFusionLivgo::finish() {
  Impl& m = *impl_;
  if (!m.enabled || m.summary_sent) return;
  m.summary_sent = true;
  if (m.sink) m.sink->onSummary(summary());
}

}  // namespace livo_gnss
