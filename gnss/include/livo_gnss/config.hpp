// SPDX-License-Identifier: BSD-3-Clause
// Configuration of the GNSS core: a 1:1 mirror of the `gnss:` YAML tree in DESIGN §14 plus
// AMENDMENTS A1 (`nav_rinex`), A6/A15 (Avia IMU defaults, QZSS keys), A16 (`ephem:`), A17 (`time.toff_monitor`).
// Defaults are the design's placeholders; starred (*) values are frozen after the development-set
// calibration (TEST_PLAN §3) and their sha256 goes into the preregistration.
// Keys that are not listed here keep the upstream (FAST-LIVO2) behaviour and are read by the ROS glue.
#pragma once

#include <map>
#include <string>
#include <vector>

#include "livo_gnss/export.hpp"

namespace livo_gnss {

struct LIVO_GNSS_API Config {
  // ---- top level ----
  bool enable = false;                 // false: bit-identical to reference commit R (zero-channel gate)
  bool update_enable = true;           // false: OBSERVE (compute, never write the host; bypass gate)
  enum class Mode { Tight, LoosePvt, LivgoProbe, LivgoProbeNorej };
  Mode mode = Mode::Tight;
  enum class UpdateSlot { PreLio, PostVio };
  UpdateSlot update_slot = UpdateSlot::PreLio;   // §9; PostVio only for the I13 equivalence check

  struct Use {
    bool psr = true, dopp = true, tdcp = true, pvt_vel = true;
  } use;

  enum class SourceKind { GnssCommRos1, Rinex, UbxStream };
  SourceKind source = SourceKind::GnssCommRos1;

  struct Topics {
    std::string meas = "/ublox_driver/range_meas";
    std::string ephem = "/ublox_driver/ephem";
    std::string glo_ephem = "/ublox_driver/glo_ephem";
    std::string iono = "/ublox_driver/iono_params";
    std::string pvt = "/ublox_driver/receiver_pvt";
    std::string tp = "/ublox_driver/time_pulse_info";
  } topics;

  struct Rinex {
    std::string obs, nav;
    double latency_s = 0.0;            // epochs enter the event stream at t_est + latency_s (§12.4)
  } rinex;

  // AMENDMENTS A1: broadcast ephemerides merged from RINEX NAV files (IGS BRDC); bag ephemerides only
  // fill the (sat, toe) entries the RINEX files lack. Active together with source=GnssCommRos1.
  std::vector<std::string> nav_rinex;

  struct Systems {
    bool gps = true, glo = true, gal = true, bds = true;
    bool qzs_as_gps = true;            // QZSS observations enter the GPS clock (ι_G ≡ 0)
    bool glo_psr = false;              // GLONASS pseudorange off by default (IFB, R2-5); dopp/TDCP still used
    struct BdsGeo {
      bool use = true;
      double psr_scale = 2.0;          // s_sys for C GEO
    } bds_geo;
  } systems;

  std::vector<std::string> exclude_sats;   // RINEX ids, e.g. "C59", "G07"

  struct Frames {
    enum class LeverArmFrame { Lidar, Imu };
    LeverArmFrame lever_arm_frame = LeverArmFrame::Lidar;
    double lever_arm_xyz[3] = {-0.358, 0.086, 0.429};  // antenna in lever_arm_frame [m]; -> l^I via extR/extT
    double lever_arm_sigma = 0.03;     // σ_l [m], enters R only (§4.7)
    double pivot_distance_m = 100.0;   // re-pivot threshold ‖a^W−o^W‖ (§2.3)
    double sigma_dov_rad = 1.5e-4;     // σ_DOV tilt prior (§2.2)
    // Optional leveling term in φ's activation prior, independent of the host g block.
    // Candidate from the Avia IMU (BMI088, user manual v1.0; BST-BMI088-DS001-13 Table 4):
    // hypot(atan(20 mg), 0.5°) = 0.021824 rad. Left at 0: U15 OpenSky with a held 0.7° g-direction
    // error does not move φ under the 0.03° prior (gate ratio 0.34, TDCP NIS/dof 1.01), and turning
    // this term on makes φ move and then fail |Δφ|/σ. Not fit to any trajectory.
    double sigma_lev_rad = 0.0;
    bool gnss_updates_gravity = true;  // δg column of J_a/J_w (only if host gravity_est_en)
  } frames;

  struct Time {
    enum class Base { Gpst, Utc, Host };
    Base base = Base::Host;            // M3DGR = host
    std::string leap = "auto";         // "auto" = RTKLIB leap table by date; or an integer string
    double offset_s = 0.0;             // t_est = t_gpst − leap + offset + drift·(t_gpst − t_ref) (§1)
    double drift_ppm = 0.0;
    double t_ref_gpst_s = 0.0;         // GTime::toSec() scale: Unix-epoch origin, GPST, no leap seconds (A18, A40)
    std::string calib_file;            // tools/time_calib/<seq>.yaml overrides offset/drift/sigma
    double sigma_toff_s = 0.05;        // σ_toff, into R (§4.7)
    bool estimate_online = false;      // td slot (§2.2)
    double td_rw_s2_per_s = 1.0e-8;
    double max_bridge_s = 0.3;         // δ ≤ 0.3 s, older epochs dropped and counted (§4.1)
    int bridge_ring_cycles = 4;        // minimum cycles kept; every cycle with t_k ≥ t_latest − (max_bridge_s + 0.1 s)
                                       // is kept as well (retention by duration, AMENDMENTS A33)
    // §7 online residual time-offset diagnostic (TimeOffsetMonitor, AMENDMENTS A17): sliding-window
    // regression of Doppler innovations on u_jᵀα_A; |slope| > n_sigma·σ and > min_abs_s ⇒ TOFF_WARN.
    struct ToffMonitor {
      bool enable = true;
      double window_s = 30.0;
      double min_abs_s = 0.005;
      double n_sigma = 3.0;
    } toff_monitor;
  } time;

  struct ImuBridge {                   // AMENDMENTS A6/A15: Avia is the main platform, so Avia values are the defaults
    double acc_n = 0.0208;             // M3DGR Kalibr, Avia IMU (MID-360: 0.0115)
    double gyr_n = 0.00322;            // (MID-360: 0.00133)
  } imu_bridge;

  struct Masks {                       // L0 (§10)
    double el_deg = 10.0;
    double cn0_psr = 30.0, cn0_dopp = 25.0, cn0_cp = 28.0;
    int track_psr = 10, track_cp = 3;  // consecutive-epoch counts (no locktime field)
    // QZSS ('J') carries its own key: operator[] on a missing key would read 0 s (m16). QZSS is treated as GPS.
    std::map<char, double> eph_age_s = {{'G', 7200.0}, {'E', 7200.0}, {'C', 3600.0}, {'R', 1800.0}, {'J', 7200.0}};
    double max_psr_std_m = 5.0, max_dopp_std_hz = 1.0, max_cp_std_m = 0.02;
    double psr_min_m = 1.9e7, psr_max_m = 4.5e7;  // §10 L0 range gate
    double max_range_rate_mps = 2000.0;           // |λD| < 2 km/s
  } masks;

  struct Noise {                       // §4.7
    struct Psr {
      double sigma0_m = 0.3, cn0_ref = 45.0, k_std = 1.0;
      double tau_s = 10.0;             // * κ_AR time constant
      double decim_s = 1.0;            // 1 Hz decimation: first epoch to arrive in each ⌊t/decim_s⌋ GPST bin (AMENDMENTS A43)
      double err_brdci = 0.5;          // Klobuchar variance factor (0.5 → 0.7 candidate)
      std::map<char, double> sys_factor = {{'G', 1.0}, {'E', 1.0}, {'C', 1.0}, {'R', 1.5}, {'J', 1.0}};
    } psr;
    struct Dopp {
      double sigma0_mps = 0.05;        // *
      double tau_s = 0.2;              // *
      bool exclusive_with_tdcp = true; // §4.4
      // Appended (AMENDMENTS A66; INTERFACE_CHANGES): the Doppler rows treat the registration tilt φ as a consider
      // parameter — their gain onto φ is zero (φ's mean does not move with them) while φ's uncertainty still enters their
      // S through H, and the joint covariance takes the Schmidt (Joseph) form of that gain. ψ, d and the host columns keep
      // the ordinary gain. Reason (A59): per-satellite Doppler biases of low-elevation, low-C/N0 satellites (0.15–0.55 m/s,
      // invisible to the NIS) reached φ through ∂w^N/∂φ, pushed it ~2σ and made the vertical of the odometry output worse
      // than the host's. Default false since AMENDMENTS A69/A70 (project_tilt below replaces it): the rows still update the
      // host, whose error then correlates with φ, and the next LIO update hands their content to φ through H2 (A67);
      // kept as an option (with project_tilt it also zeroes φ's gain through its prior correlations). Both false = the
      // pre-A66 joint gain.
      bool consider_tilt = false;
      // Appended (AMENDMENTS A69/A70; INTERFACE_CHANGES): the accepted Doppler rows of an epoch are projected, as one
      // group in the metric of their R, onto the orthogonal complement of their φ columns (JointUpdater::Options::project;
      // the TDCP Δclk construction, m rows → m − 2 unit-variance rows). The kept rows do not depend on φ: they neither
      // update it nor make the host correlated with it, so φ is decided by the position-domain rows (pseudorange, TDCP)
      // and its process model; their information on the host, ψ and d is the original rows' with φ a free nuisance
      // parameter. The cost: what the rows said along φ's directions (level motion: the vertical velocity, which the LIO
      // measures better) is dropped. false = no projection.
      bool project_tilt = true;
    } dopp;
    struct Tdcp {
      double sigma0_m = 0.004;         // * σ_Φ0
      double interval_s = 0.5;         // pair length m
      double sigma_rate_mps = 0.002;   // *
      double sigma_windup_m = 0.002;
      double max_span_s = 1.5;         // D3 slip detector
      // Appended (AMENDMENTS A61 F10; INTERFACE_CHANGES): the next pair opens at the closing epoch k1 itself (the k1 of
      // one pair is the k0 of the next) instead of at k1 + 1, so the pairs tile the time axis. Over the gap epoch the
      // frame's translation drift is seen by no row (Doppler does not observe it). Consecutive rows of a satellite then
      // share φ(k1) with opposite signs; the filter ignores that correlation — exact for one pair, conservative for a sum
      // of pairs, optimistic only for differences of consecutive pairs (A61). false = DESIGN §4.5 as written.
      bool chain_pairs = true;
      // Appended (AMENDMENTS A61 F11; INTERFACE_CHANGES): with chain_pairs, the phase noise of the shared epoch becomes a
      // state (one per satellite, Slot::Phase) instead of a term of both pairs' R, so consecutive pairs are exactly
      // correlated and a run of pairs measures the displacement with the noise of its two end epochs only — the carrier's
      // continuity, which independent pairs throw away (the accumulated TDCP error of n pairs grows like n·2σ_Φ² with
      // them). Ignored without chain_pairs.
      bool phase_states = true;
    } tdcp;
    struct Pvt {
      double tau_pos_s = 30.0, tau_vel_s = 1.0;  // *
      double decim_s = 1.0;            // same rule as noise.psr.decim_s (A43)
      double scale = 1.5;
    } pvt;
    struct Scale {
      double psr = 1.0, dopp = 1.0, tdcp = 1.0;  // * NIS≈1 scalar calibration
    } scale;
  } noise;

  struct Process {                     // §3
    struct Clock {
      double S_b = 0.1;                // * [m²/s]
      double S_d = 0.1;                // * [m²/s³]
      bool S_b_min_windup = true;      // lower bound √(S_bΔt) ≥ |ω_z|Δt·λ/2π (R2-13)
    } clock;
    double isb_q = 1.0e-4;             // [m²/s]
    struct Frame {
      double q_tilt_per_m = 1.0e-8;    // * [rad²/m]
      double q_yaw_per_m = 1.0e-7;     // * [rad²/m]
      double q_yaw_per_rad = 1.0e-5;   // * [rad²/rad]
      double q_th_per_m = 1.0e-4;      // * [m²/m]
      double q_tv_per_m = 1.0e-4;      // * [m²/m]
      double q_time = 0.0;             // per-second term, 0 = no drift at rest
    } frame;
  } process;

  struct Init {                        // §8
    int min_livo_frames = 50;
    int min_sats = 5;
    double max_gdop = 5.0;
    int min_epochs = 30;
    double min_window_s = 10.0;
    double min_hspeed_mps = 0.5;
    int min_moving_epochs = 30;
    double min_path_m = 5.0;
    double yaw_sigma_max_deg = 3.0;
    double spp_sigma_floor_m = 3.0;
    double nlos_cn0_margin_db = 8.0;
  } init;

  struct Rejection {                   // §10 L4/L5/L6
    double alpha_w = 1.0e-3;           // Baarda w-test level (3.29σ)
    double alpha_nis = 1.0e-3;
    double max_reject_frac = 0.5;
    struct MinMeas {
      int psr = 1, dopp = 1, tdcp = 2;
    } min_meas;
    bool post_check = true;            // L5
    double core_step_max_m = 0.05;     // ‖δp‖ > max(3σ_prior, 5 cm) with σ_prior < 5 cm → rollback
    double clock_jump_km = 1.0;        // L2
    int lockout_epochs = 20;           // L6
  } rejection;

  struct Slip {                        // §10 L3 D1–D6
    bool lli = true, half_cycle = true;
    double max_gap_s = 1.5;
    double dopp_tol_cyc = 0.5;
    double jerk_max = 1.0;             // j_max [m/s³] for the D4 long-gap term
    double innov_abs_m = 0.05;
    double gf_tol_m = 0.05;
  } slip;

  struct Integrity {                   // §10 L7 + degeneracy flag
    struct VelocityAlarm {
      bool enable = true;
      double alpha = 1.0e-3;
      double min_err_mps = 0.2;
      int n_consec = 5, n_clear = 10;
    } velocity_alarm;
    struct Heading {
      bool enable = true;
      double min_speed = 0.5;
      int n_consec = 20;
      double min_err_deg = 5.0;
    } heading;
    struct Degeneracy {
      double lambda_enter = 400.0, lambda_exit = 800.0;
      int window = 10;
      bool floor_enable = false;       // T+floor probe arm only (R3-12)
      double sigma_floor_m = 0.05;
    } degeneracy;
    // Appended (AMENDMENTS A61 F9; INTERFACE_CHANGES): a host update (LIO/VIO) whose pose correction lies far outside
    // its own predicted spread — χ²₆ of (x⁺ ⊟ x⁻) against P⁻ − P⁺ at least `chi2` — AND is large in absolute terms
    // (translation ≥ min_translation_m or rotation ≥ min_rotation_rad) is taken as a jump of the map frame (the LIO locked
    // onto an older part of its map), not as information about the IMU prediction: after H2 the registration covariance
    // grows by the size of that move (translation along it, heading by its vertical part, tilt by its horizontal part,
    // mapped like the registration walk) and an open TDCP pair is kept, its clone predating the jump (event FRAME_JUMP).
    // The absolute floor keeps ordinary corrections of an over-confident host out.
    // Default off (AMENDMENTS A64): on M3DGR, FAST-LIVO2's ordinary LIO corrections are about 5× the spread its own
    // P⁻ − P⁺ predicts, so the χ²₆ premise fails on the real host and ~4 % of the LIO cycles were taken as jumps (A62 ①).
    // The simulator's revisit and ghosting scenes (U15, whose host stand-in is consistent) still switch it on.
    struct FrameJump {
      bool enable = false;
      double chi2 = 35.9;              // χ²₆(1 − 1e-6)
      double min_translation_m = 0.03;
      double min_rotation_rad = 0.005;
    } frame_jump;
  } integrity;

  struct Iterate {                     // §4.8 (3)
    double yaw_sigma_deg = 2.0;
    double dyaw_deg = 0.1;
    int max_iter = 3;
    int reacq_epochs = 5;              // fixed 3 iterations for the first N epochs after ACTIVATE/RECOVERY
  } iterate;

  struct Outage {                      // §11
    double outage_s = 1.0;
    int recover_min_sats = 4;
    double recover_settle_s = 1.0;
    double reinit_after_s = 60.0;
    double reinit_sigma_t_m = 10.0;
    double reinit_sigma_psi_rad = 0.1;
  } outage;

  struct Ablation {                    // arms (IMPL_PLAN 8)
    bool frame_fixed_after_init = false;   // A-FA
    bool tdcp_fixed_anchor = false;        // A-FA
    bool disable_l4_l5 = false;            // T−reject
  } ablation;

  struct Output {
    std::string frame_id = "enu";
    double smooth_trans_mps = 0.2;     // /gnss/global_odometry_smooth rate limits (§11)
    double smooth_yaw_dps = 2.0;
    bool sat_csv = false;              // gnss_sats.csv
    // Appended (AMENDMENTS A61 F13; INTERFACE_CHANGES): keep the Origin slot ([a^N; ψ] at ACTIVATE, 4 states) behind
    // GnssFusion::globalPoseOdom() / traj_N_odom.tum. false: no slot, the odometry output equals the global one.
    bool odom_output = true;
  } output;

  // AMENDMENTS A16 (M5/m7): ephemeris ingest semantics that only the offline audit can decide.
  struct Ephem {
    // Meaning of GnssEphemMsg.ura as delivered by the receiver driver: decided by ingest self-check
    // §12.6-4 (tools/m3dgr_audit.py vs the same-day RINEX SV accuracy) and written into the sequence
    // YAML. `Unknown` with source = gnss_comm_ros1 is rejected by validateConfig (no silent default).
    enum class UraSemantics { Unknown = 0, Index, Metres };
    UraSemantics ura_semantics = UraSemantics::Unknown;
    // Galileo: keep I/NAV (E1/E5b, the source of the E1 observations we use) when both I/NAV and F/NAV
    // share a toe in the merged BRDC file (RTKLIB setseleph(SYS_GAL, 1)); false = RTKLIB default (closest toe).
    bool gal_prefer_inav = true;
  } ephem;
};

// Returns human-readable problems (empty = OK). Checked by GnssFusion's constructor; the ROS glue
// prints them and refuses to start when non-empty.
LIVO_GNSS_API std::vector<std::string> validateConfig(const Config& c);

// Canonical dump of every field; the caller hashes it with sha256 for the preregistration (TEST_PLAN §3).
// Same field order as this header so two builds agree byte for byte.
LIVO_GNSS_API std::string canonicalDump(const Config& c);

}  // namespace livo_gnss
