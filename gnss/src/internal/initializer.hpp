// SPDX-License-Identifier: BSD-3-Clause
// Initializer — state machine OFF→OBSERVE→WAIT_CORE→WAIT_NAV→COLLECT→ALIGN→TRACKING (DESIGN §8):
//   WAIT_CORE: !imu_need_init && lidar_map_inited && ≥50 LIO frames; WAIT_NAV: ≥5 sats with healthy eph;
//   COLLECT: window ≥30 epochs and ≥10 s of robust SPP + Doppler LS, paired with bridged antenna samples
//            a^W_j, w^W_j from the host;
//   ALIGN time: cross-correlation check of the offline offset (>3σ ⇒ TOFF_WARN, no auto change);
//   ALIGN yaw: ≥30 epochs with v_h ≥ 0.5 m/s and ≥5 m displacement; 2D Procrustes ψ₀ then GN-LS on raw
//              Doppler with per-epoch d_j projected out; post-fit scaled σ_ψ ≤ 3°;
//   ALIGN trans/clock/ISB: ψ fixed; batch LS on raw pseudoranges (shared t, ι; per-epoch b_j) with
//              iterated w-rejection, σ̂ ≤ 5 m; r_0 = robust window mean (N fixed); o^W = current a^W;
//   ACTIVATE: consistent augmentation of φ (σ_DOV), ψ (GN), t (with cross-cov −J_x P, −J_ψ σ_ψ²),
//              b/ι (LS block), d (Doppler LS at k_a); epochs used here are never re-used as updates.
// Owner: P3-A1 (OWNERSHIP §2c). Tests: test_u13_init.cpp (known ψ, r_0, t, b, d, ι recovered;
// still/insufficient excitation keeps waiting; no double counting).
//
// Protocol with the facade (P3-A2; initializer.cpp has the details, INTERFACE_CHANGES P3-A1 the contract):
//   * OFF and OBSERVE are start phases: the first onLioFrame() moves either to WAIT_CORE. "Only compute, never write"
//     (update_enable = false) is the updater's observe_only option and EpochRecord::observe_only; it does not stop
//     this machine (the bypass gate must run H1/H2/H3 on a live block). WAIT_CORE → WAIT_NAV in onLioFrame(),
//     WAIT_NAV → COLLECT in onNavCount(), COLLECT → ALIGN in addSample() once windowReady() && movingEnough().
//   * phase() == ALIGN: call Leveling::setRef(core.g) if the leveling has no reference yet, then tryAlign() and, when
//     it is ok, activate() — both in the LIO cycle whose onLioStep() added the last sample (k_a), with `core` = the host
//     state at that cycle's t_k (the state the sample's bridge ends on, before any GNSS update of the cycle). A failed
//     tryAlign() returns to COLLECT (the window keeps sliding; a new attempt after ≥ kRetrySamples more samples); an ok
//     one clears the window (its epochs are consumed, consumedByInit()). After activate() the caller sets TRACKING
//     (activate() is const) — or COLLECT when activate() failed — and must not update with epochs at or before
//     AlignResult::t_activate. REINIT = the caller drops frame/clock/td/clone slots and calls setPhase(COLLECT);
//     the N frame (anchor) is kept.
//   * CollectSample: spp/dop from RobustSpp/RobustDopplerLs on the L0-masked, NLOS-prescreened inputs; `raw` = those
//     inputs as solve() left them (el/az/ion_m/trop_m at the SPP solution); host_ant = ImuBridge::evaluate(core, bridge,
//     l^I) of the cycle that received the epoch; bridge = ImuBridge::query(t_est) of that cycle.
//   * AMENDMENTS A51 (P4-INIT): the facade hands over EVERY COLLECT epoch that has a bridge — also those whose SPP failed
//     or used < init.min_sats (spp.ok = false then) — with CollectSample::ion/ion_valid/psr_std_m/dopp_std_hz filled;
//     addSample() sorts it into the §8 window (SPP ok, ≥ min_sats, GDOP < max_gdop) and/or the LIO-aided window (≥ 1
//     usable row). WAIT_NAV → COLLECT at one healthy satellite: the facade follows phase() after onNavCount(). An
//     AlignResult says which path solved (`path`). A facade that still feeds only SPP samples gets the §8 behaviour plus
//     the A51 path on those samples.
#pragma once

#include <deque>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "livo_gnss/config.hpp"
#include "livo_gnss/core_state.hpp"
#include "livo_gnss/diagnostics.hpp"
#include "livo_gnss/types.hpp"
#include "aug_block.hpp"
#include "bridge.hpp"
#include "gnss_only.hpp"
#include "leveling.hpp"

namespace livo_gnss {

struct CollectSample {
  GTime t_rx;
  double t_est = 0.0;
  SppSolution spp;
  DopplerSolution dop;
  std::vector<SppInput> raw;          // raw rows kept for the batch LS
  AntennaAtEpoch host_ant;            // bridged host antenna (W frame) at t_est
  BridgeResult bridge;

  // ---- appended by P4-INIT (AMENDMENTS A51; INTERFACE_CHANGES) ----
  // Inputs of the LIO-aided multi-epoch path that the §8 path does not need. That path re-evaluates elevation/azimuth,
  // ionosphere, troposphere and the §4.7 variances of every raw row at its own antenna positions: an epoch of 1–3
  // satellites never had a GNSS-only fix to evaluate them at (the facade's rows then carry el = 90°, no atmosphere).
  //   ion / ion_valid        the GPS Klobuchar in force for the epoch (A38/A45); not valid ⇒ I = 0, σ_ion = 5 m (§8)
  //   psr_std_m, dopp_std_hz the receiver-reported std of each raw row's primary signal after the unit judgement
  //                          (§12.6-3), aligned with `raw`; empty ⇒ the raw rows' var_psr/var_dopp, el and ion/trop are
  //                          used as delivered (so are rows with SppInput::models_fixed).
  Klobuchar ion;
  bool ion_valid = false;
  std::vector<double> psr_std_m, dopp_std_hz;
};

struct AlignResult {
  bool ok = false;
  std::string reason;                 // failure reason for events.log
  double yaw = 0.0, sigma_yaw = 0.0;
  Eigen::Vector3d trans = Eigen::Vector3d::Zero();                 // t relative to pivot_W (= a^N at k_a)
  Eigen::Matrix3d cov_trans = Eigen::Matrix3d::Zero();             // noise part of t (see cov_noise), floored
  Eigen::Matrix<double, 3, HostIdx::dim> J_trans_host = Eigen::Matrix<double, 3, HostIdx::dim>::Zero();   // ∂t/∂δx = −J_a,host
  Eigen::Vector3d J_trans_yaw = Eigen::Vector3d::Zero();           // ∂t/∂ψ of the batch LS
  double b = 0.0, d = 0.0;            // receiver clock bias [m] and drift [m/s] at t_k (the activation cycle end)
  Eigen::MatrixXd cov_clock_isb;      // [b, ι_C, ι_E, (ι_R)] block from the batch LS (observed ISBs only, order below)
  double isb[4] = {0, 0, 0, 0};       // order: 0 = BDS (ι_C), 1 = GAL (ι_E), 2 = GLO (ι_R), 3 = QZS
  bool isb_present[4] = {false, false, false, false};   // a slot is created for this system
  double var_d = 0.0;
  AnchorFrame anchor;
  Eigen::Vector3d pivot_W = Eigen::Vector3d::Zero();
  double offset_check_s = 0.0;        // online cross-correlation vs offline offset
  bool offset_warn = false;
  int epochs_used = 0;
  GTime t_activate;                   // k_a: updates start at k_a + 1

  // ---- appended by P3-A1 (INTERFACE_CHANGES) ----
  // isb_observed[k]: ι_k estimated by the batch LS; a present but unobserved system gets ι = 0, σ = 30 m (A35).
  bool isb_observed[4] = {false, false, false, false};
  // The complete noise model that activate() applies, over n = [t(3), b(t_k), d(t_k), ι_k for each observed k in the
  // order of isb[]]:  e_new = J_trans_host·e_x (t rows only) + J_noise_yaw·e_ψ + J_noise_tilt·e_φ + n,
  // n ~ N(0, cov_noise). cov_noise holds the batch (t, b, ι) covariance (post-fit scaled by max(1, σ̂0²), t diagonal
  // floored at init.spp_sigma_floor_m²), the Doppler-LS drift variance, b's propagation from its epoch to t_k and the
  // clock process noise over that interval; cov_trans / cov_clock_isb / var_d above are views of it.
  Eigen::MatrixXd cov_noise;
  Eigen::VectorXd J_noise_yaw;                                     // (5 + k_obs)
  Eigen::MatrixXd J_noise_tilt;                                    // (5 + k_obs) × 2
  Eigen::Matrix<double, 3, 2> J_trans_tilt = Eigen::Matrix<double, 3, 2>::Zero();
  double t_k = 0.0;                   // estimator time the clock values refer to (the last sample's bridge t_k)
  double sigma_toff_s = 0.0;          // td slot prior σ when time.estimate_online (DESIGN §2.2)
  // Fit diagnostics (events.log, U13).
  double sigma0_dopp = 0.0, sigma0_psr = 0.0, rms_psr_m = 0.0;
  int rows_dopp = 0, rows_psr = 0, rejected_dopp = 0, rejected_psr = 0, epochs_psr = 0;
  int moving_epochs = 0;
  double path_m = 0.0;
  bool offset_check_valid = false;    // enough speed excitation for the §7 time check
  double offset_check_sigma_s = 0.0;
  Eigen::MatrixXd cov_noise_formal;   // cov_noise before the §8 3 m floor on t (diagnostics: is the floor binding?)

  // ---- appended by P4-INIT (AMENDMENTS A51; INTERFACE_CHANGES) ----
  std::string path;                   // which initialisation solved: "spp_window" (§8) or "lio_aided" (A51)
  // ∂ψ̂/∂φ of the A51 joint LS (ψ is estimated together with t, b, d, ι there); zero for the §8 yaw fit (no φ term).
  // activate() puts it into ψ's row, so the new block's (φ, ψ, t, b, d, ι) covariance is the joint LS's
  // Σ_θ + J_φ σ_DOV² J_φᵀ exactly. With it, J_noise_tilt holds the total ∂/∂φ of each n component and J_noise_yaw the
  // regression of n on ψ's own noise (cov_noise is then the Schur complement, i.e. the part independent of ψ).
  Eigen::RowVector2d J_yaw_tilt = Eigen::RowVector2d::Zero();
  // A51 diagnostics (events.log / U13): rows − unknowns after the w-test, distinct satellites with rows, window span,
  // largest formal σ of t (post-fit scaled, before the §8 floor).
  int redundancy = 0, n_sats = 0;
  double window_s = 0.0, sigma_trans_max_m = 0.0;

  // ---- appended (AMENDMENTS A61 F1; INTERFACE_CHANGES) ----
  // Host path from the map origin to this activation [m], filled by the caller (the facade counts it from its first
  // LIO cycle). activate() widens the tilt prior to σ_DOV² + q_tilt·map_path_m per axis: the map's tilt has walked with
  // the path since the LIO levelled it at start-up (DESIGN §3.2's own walk; before A61 the prior held σ_DOV only, 11σ_DOV
  // short of A58's q_φ after 20 m). 0 ⇒ σ_DOV² (the old prior).
  double map_path_m = 0.0;
};

class Initializer {
 public:
  Initializer(const Config& cfg, const Leveling& leveling);

  Phase phase() const { return phase_; }
  void setPhase(Phase p);             // used by REINIT (back to COLLECT) and OBSERVE
  // Host readiness (WAIT_CORE); counts LIO frames internally.
  void onLioFrame(bool imu_initialised, bool lidar_map_inited);
  bool coreReady() const;
  // WAIT_NAV: number of satellites with healthy ephemeris at t.
  void onNavCount(int n_healthy);
  // COLLECT: one epoch's GNSS-only solutions + bridged host antenna.
  void addSample(CollectSample&& s);
  bool windowReady() const;           // ≥ min_epochs and ≥ min_window_s
  bool movingEnough() const;          // ≥ min_moving_epochs with v_h ≥ min_hspeed and path ≥ min_path_m
  // ALIGN: runs time check, yaw, trans/clock/ISB. On failure keeps the window sliding and reports why.
  AlignResult tryAlign(const CoreState& core, double offline_offset_s, double sigma_toff_s);
  // ACTIVATE: writes the slots into `aug` with consistent augmentation (needs the host P at k_a).
  bool activate(const AlignResult& a, const CoreState& core, AugBlock* aug, std::string* why) const;
  void clearWindow();
  int windowSize() const { return static_cast<int>(window_.size()); }
  // Read-only views so the LIVGO arm can copy the window before tryAlign() consumes it.
  const std::deque<CollectSample>& sppWindow() const { return window_; }
  const std::deque<CollectSample>& laWindow() const { return la_window_; }
  double windowSpan() const;

  // Pieces exposed for U13:
  static double procrustesYaw(const std::vector<Eigen::Vector2d>& v_gnss, const std::vector<Eigen::Vector2d>& w_host);
  static bool yawGaussNewton(const std::vector<CollectSample>& window, double yaw0, double* yaw, double* sigma_yaw, std::string* why);

 private:
  const Config& cfg_;
  const Leveling& leveling_;
  Phase phase_ = Phase::OFF;
  int lio_frames_ = 0;
  bool imu_ok_ = false, map_ok_ = false;
  int n_healthy_ = 0;
  std::deque<CollectSample> window_;

 public:
  // ---- appended by P3-A1 (INTERFACE_CHANGES) ----
  // The window slides: samples older than maxWindowSpan() before the newest one are dropped.
  static constexpr double kWindowSpanFactor = 3.0;   // maxWindowSpan() = kWindowSpanFactor · init.min_window_s
  static constexpr int kRetrySamples = 10;           // samples between two ALIGN attempts after a failure (≈ 1 s)
  double maxWindowSpan() const;
  // Host flag gravity_est_en (GnssFusion::setHostFlags): the δg column of J_trans_host, as LinPoint::gravity_est_en.
  void setGravityEstimated(bool en);
  // The N frame is fixed by the first successful tryAlign() and reused by every later one (REINIT keeps N, DESIGN §1,
  // §11). setAnchor() fixes it explicitly (restart, tests).
  void setAnchor(const AnchorFrame& anchor);
  bool anchorFixed() const;
  const AnchorFrame& anchorFrame() const;
  // DESIGN §8 "初始化用过的历元不再作更新": true for every epoch at or before the last k_a (AlignResult::t_activate).
  bool consumedByInit(const GTime& t_rx) const;
  // addSample() refusals (phase, time order, consumed epoch, SPP not ok / too few satellites / GDOP, no bridge).
  int samplesRefused() const;
  const std::string& lastRefusal() const;
  // Yaw fit with explicit frames: C_level = C(g) of the leveling in use, R_EN of the N frame. tryAlign() uses this with
  // leveling_.C(core.g) and the anchor; the frozen yawGaussNewton() above has no frame arguments and takes
  // C = minimal rotation of the last sample's bridge g_snap onto −e_z (a fresh Leveling::setRef) and R_EN at the last
  // sample's SPP position — equal to the facade's frames only while its C_ref comes from that g.
  struct YawFit {
    double yaw = 0.0, sigma_yaw = 0.0;   // σ post-fit scaled by max(1, σ̂0²)
    double sigma0 = 0.0;
    int rows = 0, epochs = 0, rejected = 0, iterations = 0;
  };
  static bool yawGaussNewton(const std::vector<CollectSample>& window, double yaw0, const Eigen::Matrix3d& C_level,
                             const Eigen::Matrix3d& R_EN, YawFit* fit, std::string* why);

 private:
  bool gravity_est_en_ = true;
  AnchorFrame anchor_;
  bool anchor_fixed_ = false;
  GTime t_consumed_;
  bool has_consumed_ = false;
  int n_refused_ = 0;
  std::string last_refusal_;
  int retry_wait_ = 0;

 public:
  // ---- appended by P4-INIT (AMENDMENTS A51; INTERFACE_CHANGES) ----
  // LIO-aided multi-epoch initialisation, a second ALIGN path next to §8 (whichever is solvable first activates; §8 is
  // tried first when both are armed). Its window keeps every COLLECT sample that has a bridge and at least one usable
  // pseudorange or Doppler row — whatever the number of satellites, SPP or not — over at most kLaMaxSpanS. One joint
  // batch LS over all of the window's L0-passed rows (pseudoranges at 1 Hz per satellite: the first L0-passed one of each
  // satellite in each GPST second; Dopplers at full rate) with the host's bridged antenna positions/velocities a^W_j,
  // w^W_j as known relative motion (their bridge noise Σ_D in the row variances) solves θ = [ψ, t(3), b(t_k), d, a,
  // ι of every system with code rows]: code rows on the quadratic clock b + d·τ + ½a·τ² (τ = t_g,j − t_k), every epoch's
  // Doppler rows on their own clock drift d_j, eliminated per epoch (the receiver's drift wander is not white; see
  // initializer.cpp and INTERFACE_CHANGES P4-INIT I-11), φ = 0 with its σ_DOV prior mapped through ∂θ̂/∂φ; d(t_k) is the
  // eliminated drift of the newest Doppler epoch. Solvable (0/1): redundancy ≥ kLaMinRedundancy, σ_ψ ≤
  // init.yaw_sigma_max_deg, every σ_t ≤ kLaMaxTransSigmaM (formal, post-fit scaled by max(1, σ̂0²) as A1-1), and the
  // iterated Baarda w-test (α = 1e-3, ≤ 30 % of the rows) ends with no row above the threshold. Armed like §8's window
  // (init.min_epochs samples over init.min_window_s, init.min_moving_epochs at ≥ init.min_hspeed_mps and
  // init.min_path_m: no alignment at standstill) and at most once per kLaRetryS of GPST. ACTIVATE is the same consistent
  // augmentation (activate()). WAIT_NAV → COLLECT now needs one satellite with healthy ephemeris (the §8 path still
  // needs init.min_sats per sample).
  static constexpr double kLaMaxSpanS = 60.0;       // window span cap: the linear clock model's validity (A51)
  static constexpr int kLaMinRedundancy = 5;        // rows − unknowns (A51)
  static constexpr double kLaMaxTransSigmaM = 5.0;  // formal σ of each t component (A51)
  static constexpr double kLaRetryS = 1.0;          // GPST seconds between two A51 attempts
  static constexpr double kLaClockJumpM = 1.0e4;    // Doppler-predicted pseudorange discontinuity that restarts the
                                                    // A51 window (receiver clock jump: the linear clock model breaks)
  int laWindowSize() const;
  double laWindowSpan() const;
  bool laArmed() const;                             // window/motion minimums met and kLaRetryS since the last attempt
  int laRestarts() const;                           // A51 windows restarted at a receiver clock discontinuity

 private:
  std::deque<CollectSample> la_window_;
  GTime la_last_attempt_;
  bool la_attempted_ = false;
  int la_restarts_ = 0;
  // The two ALIGN paths. Neither touches the phase, the retry wait, the windows or the anchor: tryAlign() commits.
  AlignResult alignSppWindow(const CoreState& core, double offline_offset_s, double sigma_toff_s) const;
  AlignResult alignLioAided(const CoreState& core, double offline_offset_s, double sigma_toff_s) const;
};

}  // namespace livo_gnss
