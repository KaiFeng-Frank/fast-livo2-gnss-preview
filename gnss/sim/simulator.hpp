// SPDX-License-Identifier: BSD-3-Clause
// livo_gnss_sim — ROS-free Monte-Carlo simulator for U15, the consistency validation of the joint filter (TEST_PLAN §1
// U15; criteria and scope as ruled in AMENDMENTS A60). Owner: P3-E.
//
// One run = one scene generated from a seed, replayed through a synthetic host and the real GnssFusion facade:
//   * truth: a ground trajectory of `path_m` (1 km) at 1.1–1.9 m/s with turns, defined by integrating noise-free IMU
//     samples with FAST-LIVO2's own propagation scheme (so the host bridge interpolates it exactly); real broadcast
//     geometry from the development-set fixture's IGS BRDC at its site and day;
//   * map drift: the registration T_{N←W} is a random walk driven by the distance travelled, rotations applied about
//     the current antenna point (DESIGN §3.2, the filter's own process model with its q), from the map origin on (the
//     activation tilt prior carries the walk so far since A61 F1; `drift_from_activation` for the older assumption).
//     Two truths (A60):
//     - drift in ξ (default, the design's model): the host's map-frame trajectory is the smooth IMU trajectory and the
//       world trajectory is T_{N←W_k}·x^W — the drift lives only in the registration, the LIO never sees it;
//     - drift seen by the LIO (`drift_in_host`): the world trajectory is smooth and the host's map-frame pose absorbs
//       the drift increments (the LIO innovation carries them) — the host adds matching process noise;
//   * gravity: tilted from the ellipsoid normal by a per-run deflection of the vertical drawn from the filter's σ_DOV
//     prior, so the tilt truth at activation is what the prior says;
//   * host: an error-state Kalman filter written like IMU_Processing.cpp (midpoint IMU integration, the upstream F_x,
//     F_tot accumulation, bridge samples recorded in the propagation loop) with process noise from the simulated IMU
//     (white acc_n/gyr_n, bias random walks) and from the drift, corrected once per cycle by the "LIO": the true pose
//     in the current map frame plus mm-level noise (6-column H on δθ, δp);
//   * receiver: pseudorange, Doppler and carrier of GPS/Galileo/BeiDou/GLONASS with a two-state clock random walk,
//     inter-system biases, Klobuchar and Saastamoinen delays, group delays, integer ambiguities;
//   * measurement noise: in the model-matched scenarios (group A) white noise with exactly the variance the filter
//     assigns (the core's NoiseModel on the core's RowBuilder geometry) for every term the scene does not produce
//     physically — the IMU-noise and clock-walk terms of the bridge are produced physically — and a carrier with the
//     atmospheric delays (the TDCP model carries their change over a pair since A61 F2); in the realistic scenario
//     (group B) Gauss–Markov multipath, white noise and NLOS outliers; group C (model mismatch) is group A's noise with a
//     map that violates the model. Every run checks that the generated pseudoranges equal the filter's own measurement
//     model at the truth (RunResult::model_psr_max_abs_m).
// Scenarios (TEST_PLAN U15): open sky, urban multipath, 120 s outage, clock jumps, cycle slips, 4 systems + ISB,
// few satellites (3 then 2), long standstill, revisit snap Δ, deterministic yaw drift, map ghosting; A61 open sky with
// A58's errors; A66 low-elevation per-satellite Doppler biases.
// Every random draw comes from the run's seeded engine: a run is reproducible bit for bit on one build; `noise_sign`
// negates every Gaussian draw (antithetic twin: the pair average isolates what does not depend on the noise, A60 F2).
// Runs are single-threaded and not to be run concurrently: RTKLIB's ephemeris selection is process-wide state.
#pragma once

#include <array>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "livo_gnss/config.hpp"
#include "livo_gnss/diagnostics.hpp"
#include "livo_gnss/types.hpp"

namespace livo_gnss {
namespace sim {

enum class Scenario : uint8_t {
  OpenSky = 0, UrbanMultipath, Outage120s, ClockJumps, CycleSlips, FourSystemsIsb, FewSats, LongStill,
  RevisitSnap, YawDriftMismatch, MapGhosting,
  // Appended (A61): open sky with the errors the A58 calibration describes — per-satellite pseudorange errors with the
  // calibrated marginal variance, 75 % Gauss–Markov (τ_ρ) and 25 % constant per satellite pass, white Doppler and
  // carrier at the calibrated levels, the carrier's and the Doppler's atmospheric terms — and the frozen A58 filter tree.
  // Group B; the ATE proxy of the FAST-LIVGO comparison.
  OpenSkyRealistic,
  // Appended (A66): open sky (the same scene draw for draw) whose satellites carry a constant Doppler bias while below
  // dopp_bias_el_deg (A59: low-elevation, low-C/N0 satellites of the development set, 0.15–0.55 m/s after removing the
  // common mode, present at rest too, invisible to the NIS). Group A's noise with a violation of the model: group C.
  LowElevDopplerBias,
  Count
};
const char* scenarioName(Scenario s);
bool scenarioFromName(const std::string& name, Scenario* s);

// What a scenario is allowed to claim (AMENDMENTS A60): A = the scene follows the filter's model, NEES and NIS must
// sit in the χ² bands; B = realistic time-correlated errors covered by κ_AR, the filter must not be over-confident;
// C = the map violates the model, the filter must stay bounded and recover.
enum class Group : uint8_t { ModelMatched = 0, Realistic, Mismatch };
Group scenarioGroup(Scenario s);

struct SimParams {
  Scenario scenario = Scenario::OpenSky;
  uint32_t seed = 0;
  std::string nav_rinex;                 // broadcast ephemerides for generator and filter (the fixture's IGS BRDC)
  // Optional ENU trajectory (TUM: t x y z ..., metres, z up). When set, the truth follows this path
  // instead of the procedural plan. Test/diagnostic only; an empty string leaves every scene unchanged.
  std::string replay_enu_tum;
  Geodetic site;                         // start of the trajectory = origin of the true ENU frame N*
  GTime t0;                              // GPST of the scene start; the host clock is t0 − 18 s (time.base host)
  double path_m = 1000.0;                // distance travelled (TEST_PLAN U15: 1 km)
  double speed_mid_mps = 1.5, speed_amp_mps = 0.4;   // speed = mid ± amp (1.1–1.9 m/s)
  double turn_rate_rps = 0.15;           // mean |ψ̇| of a turn (raised-cosine profile, peak twice that)
  double imu_dt = 0.005, lio_dt = 0.1, gnss_dt = 0.1;
  // IMU white noise densities (M3DGR Avia Kalibr; the filter's imu_bridge carries the same) and bias random walks
  double acc_n = 0.0208, gyr_n = 0.00322;
  double acc_w = 4.0e-4, gyr_w = 2.0e-5;
  // LIO stand-in: the true pose in the current map frame plus white noise (mm-level)
  double lio_rot_sigma = 2.0e-4, lio_pos_sigma = 0.002;
  Eigen::Vector3d lever_I = Eigen::Vector3d(-0.316, 0.109, 0.401);   // antenna in the IMU frame (M3DGR)
  Eigen::Vector3d mount_rpy = Eigen::Vector3d(0.02, -0.015, 0.0);    // IMU mounting rotation (small tilt)
  // Constant error of the host's gravity direction at init [rad], (x, y) = tilt of ĝ away from the
  // true specific force. The covariance is not widened: this is the BMI088 zero-g bias the host
  // absorbs into g and then holds. Zero leaves every draw unchanged.
  Eigen::Vector2d level_bias = Eigen::Vector2d::Zero();
  // Constant body-frame accelerometer bias [m/s²] added on top of the small random walk, and absorbed
  // into the host's g at init (b_a estimate stays 0). Zero leaves every scene unchanged.
  // host_turnon_prior: replace the tight b_a/g prior with applyBmi088TurnonPrior. Default false.
  Eigen::Vector3d turnon_ba = Eigen::Vector3d::Zero();
  bool host_turnon_prior = false;
  double clk_b0_m = 3000.0, clk_d0_mps = 45.0;   // receiver clock at the start
  double epoch_latency_s = 0.04;         // receiver epoch available this long after its instant (host clock)
  double processing_lag_s = 0.05;        // epochs that arrived up to this long after t_k are fed before the hook
  double eval_dt_s = 1.0;                // NEES sampling interval
  double settle_s = 5.0;                 // NEES/NIS are taken from this long after ACTIVATE on
  bool fixed_anchor = false;             // filter with A-FA's tdcp_fixed_anchor (DESIGN §6 comparison)
  bool drift_in_host = false;            // world smooth, drift absorbed by the host's map-frame pose (A60 study)
  // When the registration random walk starts: at drift_start_s (default 0 = the map origin, the physical truth; since
  // AMENDMENTS A61 F1 the activation tilt prior carries the walk up to the activation). drift_from_activation: the walk
  // starts at the activation instant of a first pass over the same scene instead (every draw before it is identical, so
  // the second pass activates at the same cycle) — the pre-A61 design assumption, kept for studies.
  bool drift_from_activation = false;
  double drift_start_s = 0.0;
  // Deflection of the vertical: the true gravity in N* is tilted by a per-run draw from N(0, σ_DOV²) per axis (the
  // filter's tilt prior). false: gravity along the ellipsoid normal (tilt truth 0 at the origin).
  bool draw_dov = true;
  // Carrier phase with the troposphere/ionosphere delays: 1 (default) = with — since AMENDMENTS A61 F2 the TDCP model
  // carries their change over a pair; 0 = without; −1 = by group (realistic only; the pre-A61 study setting).
  int carrier_atmo = 1;
  double noise_sign = 1.0;               // −1: antithetic run, every Gaussian draw negated (variance reduction/bias checks)
  // OpenSkyRealistic's C/N0 = realistic_cn0_base_db + 14·sin el [dB-Hz]. 18 (AMENDMENTS A61) reads like M3DGR's receiver
  // (A58: median 25 dB-Hz on the accepted code rows, carrier phase only from 25 dB-Hz, i.e. above ~30° here); 28 is the
  // A60 study value (every satellite above the carrier mask, code σ about 3 times smaller).
  double realistic_cn0_base_db = 18.0;
  // A GNSS↔host time offset the time map does not know: every epoch is generated at the host instant τ + δ (δ drawn per
  // run from N(0, toff_sigma_s²) on a stream of its own, so the other draws are those of δ = 0). 0 = none (A61 F12 study).
  double toff_sigma_s = 0.0;
  // One broadcast ephemeris per satellite for the generator and the filter (AMENDMENTS A61), valid over the whole scene:
  // Galileo the latest before its start, the other systems the one nearest its middle (a temporary RINEX file of the
  // run). With the whole file the generator's truth is the ephemeris selected at each epoch, so a
  // satellite jumps by the difference of two consecutive ephemerides whenever the selection switches (Galileo every
  // 10 min), while the filter's TDCP evaluates both ends of a pair with the ephemeris frozen at k0 (R2-11): the pair
  // across a switch carried a centimetre-level error of the scene's own making. false: the file as it is.
  bool single_ephemeris = true;
  // LowElevDopplerBias (AMENDMENTS A66/A67): every satellite carries a constant Doppler bias b_j = ±U(dopp_bias_lo_mps,
  // dopp_bias_hi_mps) [m/s] (sign and size drawn per satellite on a stream of their own, so the rest of the scene is
  // OpenSky's draw for draw) added to its range rate while its elevation is below dopp_bias_el_deg. While biased, a
  // satellite is described as A59's were: it reports dopp_bias_cn0_db (20 dB-Hz: below the carrier mask of A58, 25, so no
  // TDCP row; its code σ follows the reported C/N0) and a receiver Doppler std of dopp_bias_std_hz (4 Hz: σ 0.56–1.45 m/s
  // from 10° to 30°; A59 measured 0.7–3 m/s, so the bias sits inside the row's σ and the NIS does not show it). Every term
  // is drawn at the filter's own variance (group A's noise); the biases are the only violation. dopp_bias_cn0_db = 45 and
  // dopp_bias_std_hz = 0 give the biased rows group A's σ (the bias is then 1–5σ; a study setting, A67).
  double dopp_bias_lo_mps = 0.15, dopp_bias_hi_mps = 0.55, dopp_bias_el_deg = 30.0;
  double dopp_bias_cn0_db = 20.0, dopp_bias_std_hz = 4.0;
};

// The filter configuration of a scene: the frozen M3DGR tree (AMENDMENTS A58) with the simulator's IMU, lever arm,
// clock PSD and drift q, and — for the model-matched scenarios — a noise model whose every term the scene realises
// exactly: κ_AR = 1 (τ → 0) with A58's κ folded into the receiver σ0 (the same per-row weights as the calibrated tree),
// σ_toff and the TDCP rate/wind-up terms → 0, no lever-arm term (the simulated lever arm is exact). Group B keeps κ_AR
// with the multipath time constant. `fixed_anchor` sets the A-FA flags.
Config simConfig(const SimParams& p);

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

// One evaluation instant of a run (step end, after the LIO update and onStepEnd).
struct NeesSample {
  double t = 0.0;                        // seconds since the scene start
  double nees_xi = kNaN;                 // registration ξ = (φ, ψ, t), 6 dof, Registration::cov
  double nees_ant = kNaN;                // global antenna position a^N, 3 dof, antennaN() covariance
  double nees_b = kNaN, nees_d = kNaN;   // receiver clock bias / drift, 1 dof each
  double nees_isb = kNaN;                // sum over the ISB slots present (dof = n_isb)
  int n_isb = 0;
  Eigen::Vector3d ant_err_N = Eigen::Vector3d::Zero();   // filter − truth in the filter's N frame [m]
  double yaw_err = 0.0, yaw_sigma = 0.0;                  // ψ̂ − ψ*, σ_ψ [rad]
  Eigen::Vector2d tilt = Eigen::Vector2d::Zero();         // φ̂ [rad]
  Eigen::Vector2d tilt_sigma = Eigen::Vector2d::Zero();   // marginal σ_φ [rad]
  // per-component squared normalised errors e_i² / C_ii (marginal, 1 dof each): ξ = (φx, φy, ψ, tx, ty, tz), antenna E/N/U
  std::array<double, 6> z2_xi{{kNaN, kNaN, kNaN, kNaN, kNaN, kNaN}};
  std::array<double, 3> z2_ant{{kNaN, kNaN, kNaN}};
  // signed normalised errors e_i / σ_i (filter − truth) of the same components and of the clock (b, d)
  std::array<double, 6> zs_xi{{kNaN, kNaN, kNaN, kNaN, kNaN, kNaN}};
  std::array<double, 3> zs_ant{{kNaN, kNaN, kNaN}};
  double zs_b = kNaN, zs_d = kNaN;
  double err_b = kNaN, sigma_b = kNaN;   // clock bias error and σ [m]
  Eigen::Vector3d sigma_ant = Eigen::Vector3d::Constant(kNaN);   // antenna σ (E, N, U) [m]
  double zs_isbE = kNaN;
  bool mismatch_window = false;          // group C: inside/after the model violation (excluded from group-A style checks)
  // Appended (AMENDMENTS A61): inside a window where the LIO pose itself is wrong (map ghosting). ξ maps the host's own
  // map coordinates to N, so there it absorbs the LIO's error (which keeps the antenna right) and differs from the true
  // registration by exactly that error — no GNSS row can tell a wrong LIO pose from a moved map frame.
  bool host_deceived = false;
};

struct ClassNis {
  double nis = 0.0;                      // Σ NIS_prior over the epochs
  long dof = 0;                          // Σ dof
  long epochs = 0;                       // epochs with dof > 0
  long in_band = 0;                      // epochs whose NIS lies in the two-sided 95 % χ²_dof interval
  double meanPerDof() const { return dof > 0 ? nis / static_cast<double>(dof) : kNaN; }
  double inBandFrac() const { return epochs > 0 ? static_cast<double>(in_band) / static_cast<double>(epochs) : kNaN; }
};

// Row counts of one class after ACTIVATE + settle (ClassStats summed over the epochs).
struct RowCounts {
  long candidates = 0, accepted = 0;
  std::array<long, static_cast<size_t>(RejectReason::Count)> rejected{};
};

// A satellite flagged by the slip detectors (collected when the filter's output.sat_csv is on: the per-satellite records).
struct SlipFlag {
  double t_gpst = 0.0;                   // epoch GPST [s] on the GTime::toSec() scale
  int sat = 0;                           // rtk_sat
  double el_deg = 0.0, innov_m = 0.0, sigma_m = 0.0, w = 0.0;   // TDCP innovation against the LIVO prediction (D5 input)
  uint8_t detectors = 0;                 // bit i−1 = Di fired
};

struct RunResult {
  bool no_fixture = false;               // the BRDC file is missing (the caller SKIPs)
  std::string error;                     // non-empty: the run failed (exception text)
  bool activated = false;
  double t_activate = kNaN;              // seconds since the scene start
  double duration_s = 0.0;
  std::vector<NeesSample> nees;
  std::array<ClassNis, 3> nis{};         // psr, dopp, tdcp (MeasClass order)
  Summary summary;
  std::array<int, static_cast<size_t>(EventKind::Count)> events{};
  int n_epochs = 0, n_sats_mean = 0;
  std::string activate_detail;           // the ACTIVATE event's detail (initialiser path, formal σ, redundancy)
  // Trajectories at the NEES instants after ACTIVATE + settle (A61 ATE proxy): the true antenna (N*), the host's
  // antenna in its map frame (what FAST-LIVO2 alone outputs, the U arm's traj_W) and the filter's global antenna
  // (traj_N). ate_u / ate_t: their position ATE RMSE after SE(3) Umeyama alignment without scale (evo_ape -a, as M3DGR
  // and FAST-LIVGO report) over the same instants.
  struct TrajSample {
    double t = 0.0;
    Eigen::Vector3d truth = Eigen::Vector3d::Zero(), host_W = Eigen::Vector3d::Zero(), est_N = Eigen::Vector3d::Zero();
    Eigen::Vector3d est_odo = Eigen::Vector3d::Zero();   // GnssFusion::antennaOdom (A61 F13)
  };
  std::vector<TrajSample> traj;
  double ate_u = kNaN, ate_t = kNaN;
  double ate_o = kNaN;                   // the same for the odometry output (traj_ant_N_odom)
  std::array<RowCounts, 3> rows{};       // psr, dopp, tdcp
  std::vector<SlipFlag> slip_flags;      // only with cfg.output.sat_csv (study aid)
  std::vector<std::string> event_log;    // FRAME_JUMP / REANCHOR / REINIT / LIVO_ALARM_ON / ROLLBACK events with their details
  double drift_start_s = kNaN;           // when the registration random walk started (scene time)
  Eigen::Vector2d dov = Eigen::Vector2d::Zero();   // the run's deflection of the vertical [rad]
  double toff_true_s = 0.0;                        // the run's time offset δ (SimParams::toff_sigma_s)
  // Generator self-check (group A/C): |P_generated − h_psr(truth)| over every pseudorange, the filter's own psr model
  // (RowBuilder geometry + §4.3 corrections) evaluated at the true antenna and clock. Anything above ~1 mm means the
  // scene no longer follows the filter's measurement model.
  double model_psr_max_abs_m = kNaN;
  double t_mismatch = kNaN;              // group C: when the model violation starts
  // synthetic-host self-check: mean NEES of its pose [δθ, δp] (6 dof) against the truth in the map frame, and the mean
  // marginal z² of every host block (θ, p, v, b_g, b_a, g; 3 dof each, divided by 3 — 1 when consistent)
  double host_nees_mean = kNaN;
  std::array<double, 6> host_block_z2{{kNaN, kNaN, kNaN, kNaN, kNaN, kNaN}};
  // Last synthetic-host sample (t>10 s): angle between estimated and true g [deg], and that sample's g z²/3.
  double host_g_err_deg = kNaN, host_g_z2_end = kNaN;
  std::string describe() const;
};

RunResult runScenario(const SimParams& p, const Config& cfg);

// Position ATE RMSE of `est` against `ref` (same length, paired) after the least-squares rigid alignment of est onto ref
// (Umeyama, rotation + translation, no scale). NaN for fewer than 3 pairs.
double ateSe3(const std::vector<Eigen::Vector3d>& est, const std::vector<Eigen::Vector3d>& ref);

// χ² helpers (regularised incomplete gamma; no external dependency)
double chi2Cdf(double x, double k);
double chi2Quantile(double p, double k);
struct Band {
  double lo = 0.0, hi = 0.0;
};
// Two-sided (1 − alpha) band of the average of n independent χ²_dof variables: [χ²_{n·dof}(α/2), χ²_{n·dof}(1 − α/2)] / n.
Band chi2MeanBand(int n, int dof, double alpha = 0.05);

// Consistency over the runs of one scenario. NEES: at every evaluation time the runs present give the average NEES
// (ANEES) and its band for that number of runs (Bar-Shalom); reported are the fraction of times inside the band and the
// time-averaged ANEES / dof. NIS: pooled Σ NIS / Σ dof and the fraction of epochs inside their own 95 % interval.
struct NeesStat {
  int dof = 0;
  int n_times = 0;                       // evaluation times with at least `min_runs` runs
  double anees_per_dof = kNaN;           // time average of ANEES(t) / dof
  double in_band_frac = kNaN;            // fraction of those times with ANEES(t) inside chi2MeanBand(n_runs(t), dof)
  double first_third = kNaN, last_third = kNaN;   // ANEES/dof averaged over the first / last third of the times
};
struct ConsistencyStats {
  int runs = 0, activated = 0, failed = 0;
  NeesStat xi, ant, b, d;
  std::array<ClassNis, 3> nis{};
  // group-A criteria (AMENDMENTS A60): every NEES kind with anees_per_dof in [nees_lo, nees_hi] and in_band_frac ≥
  // min_in_band; every NIS class with dof > 0 and mean in [nis_lo, nis_hi] and in-band fraction ≥ nis_min_in_band.
  bool withinBounds(double nees_lo = 0.8, double nees_hi = 1.25, double min_in_band = 0.8, double nis_lo = 0.85,
                    double nis_hi = 1.15, double nis_min_in_band = 0.9, std::string* why = nullptr) const;
  std::string describe() const;
};
// `exclude_mismatch`: leave out NEES samples flagged mismatch_window. `min_runs`: evaluation times need that many runs.
ConsistencyStats aggregate(const std::vector<RunResult>& runs, bool exclude_mismatch = true, int min_runs = 2);

}  // namespace sim
}  // namespace livo_gnss
