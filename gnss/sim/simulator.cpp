// SPDX-License-Identifier: BSD-3-Clause
// livo_gnss_sim — U15 Monte-Carlo simulator (TEST_PLAN §1 U15, AMENDMENTS A60). Owner: P3-E. See simulator.hpp.
#include "simulator.hpp"
#include "host_turnon_prior.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <random>
#include <set>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <Eigen/Cholesky>
#include <Eigen/Dense>

#include "internal/ephem_store.hpp"
#include "internal/leveling.hpp"
#include "internal/quality.hpp"
#include "internal/rows.hpp"
#include "internal/rtk_adapter.hpp"
#include "livo_gnss/core_state.hpp"
#include "livo_gnss/gnss_fusion.hpp"

namespace livo_gnss {
namespace sim {

namespace {

constexpr double kC = 299792458.0;
constexpr double kOmgE = 7.2921151467e-5;
constexpr double kGrav = 9.81;          // FAST-LIVO2 G_m_s2
constexpr double kDeg = M_PI / 180.0;
constexpr double kLeapS = 18.0;         // GPST − UTC in 2025 (time.base host: t_est = t_gpst − leap + offset)
constexpr double kElMaskRad = 10.0 * kDeg;
constexpr double kHumi = 0.7;           // rows.cpp REL_HUMI
constexpr double kFreqL1 = 1575.42e6;

using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;

Mat3 Rz(double a) { return Leveling::Rz(a); }

double wrapPi(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a <= -M_PI) a += 2.0 * M_PI;
  return a;
}

double smoothstep(double x) {
  if (x <= 0.0) return 0.0;
  if (x >= 1.0) return 1.0;
  return x * x * (3.0 - 2.0 * x);
}
double smoothstepDeriv(double x) { return (x <= 0.0 || x >= 1.0) ? 0.0 : 6.0 * x * (1.0 - x); }

// ---------------------------------------------------------------------------------------------
// Motion plan: speed s(t) and yaw rate ψ̇(t) as smooth analytic functions (raised-cosine turns, a sinusoidal speed
// modulation, an optional standstill window) plus a small vertical undulation.
// ---------------------------------------------------------------------------------------------
struct Turn {
  double t0 = 0.0, dur = 1.0, angle = 0.0;
};

struct MotionPlan {
  double speed_mid = 1.5, speed_amp = 0.4, Ts = 90.0, speed_phase = 0.0;
  double Az = 0.3, Tz = 60.0, z_phase = 0.0;
  double still_t0 = -1.0, still_dur = 0.0, ramp = 6.0;
  std::vector<Turn> turns;

  // Standstill window W(t) ∈ [0, 1] (1 = moving) and its derivative.
  void window(double t, double* w, double* wd) const {
    *w = 1.0;
    *wd = 0.0;
    if (still_t0 < 0.0) return;
    const double t1 = still_t0 + ramp, t2 = t1 + still_dur, t3 = t2 + ramp;
    if (t <= still_t0 || t >= t3) return;
    if (t < t1) {
      *w = 1.0 - smoothstep((t - still_t0) / ramp);
      *wd = -smoothstepDeriv((t - still_t0) / ramp) / ramp;
    } else if (t < t2) {
      *w = 0.0;
    } else {
      *w = smoothstep((t - t2) / ramp);
      *wd = smoothstepDeriv((t - t2) / ramp) / ramp;
    }
  }
  void eval(double t, double* s, double* sd, double* yr, double* vz, double* az) const {
    double w, wd;
    window(t, &w, &wd);
    const double om = 2.0 * M_PI / Ts;
    const double base = speed_mid + speed_amp * std::sin(om * t + speed_phase);
    *s = w * base;
    *sd = wd * base + w * speed_amp * om * std::cos(om * t + speed_phase);
    double r = 0.0;
    for (const Turn& tr : turns) {
      if (t < tr.t0 || t > tr.t0 + tr.dur) continue;
      r += tr.angle / tr.dur * (1.0 - std::cos(2.0 * M_PI * (t - tr.t0) / tr.dur));
    }
    *yr = r * w;   // no turning at standstill (turns are never scheduled there; w only guards the ramps)
    const double oz = 2.0 * M_PI / Tz;
    *vz = w * Az * oz * std::cos(oz * t + z_phase);
    *az = wd * Az * oz * std::cos(oz * t + z_phase) - w * Az * oz * oz * std::sin(oz * t + z_phase);
  }
};

// ---------------------------------------------------------------------------------------------
// Truth on the IMU grid: the host's own propagation scheme (IMU_Processing.cpp :352-421) applied to noise-free IMU
// samples, so the bridge interpolation reproduces it exactly at any instant.
// ---------------------------------------------------------------------------------------------
struct Truth {
  double dt = 0.005;
  int n = 0;                           // samples 0..n
  std::vector<Mat3> R;                 // R_{N*←I}(t_i)
  std::vector<Vec3> p, v;              // in N*
  std::vector<Vec3> a, w;              // interval (i−1, i]: world acceleration (incl. nothing else) and body rate ω̄
  std::vector<Vec3> f_ideal, w_ideal;  // ideal IMU samples at t_i (body frame)
  std::vector<double> s_cum, alpha_cum;   // path length and |yaw| rotation up to t_i

  double time(int i) const { return i * dt; }
  int interval(double tau) const {
    int i = static_cast<int>(std::ceil(tau / dt - 1e-9));
    return std::max(1, std::min(i, n));
  }
  void at(double tau, Mat3* Ro, Vec3* po, Vec3* vo, Vec3* ao = nullptr, Vec3* wo = nullptr) const {
    const int i = interval(tau);
    const double s = tau - time(i - 1);
    *Ro = R[i - 1] * so3Exp(w[i] * s);
    *po = p[i - 1] + v[i - 1] * s + 0.5 * a[i] * s * s;
    *vo = v[i - 1] + a[i] * s;
    if (ao) *ao = a[i];
    if (wo) *wo = w[i];
  }
  double pathAt(double tau) const {
    const int i = interval(tau);
    const double fr = (tau - time(i - 1)) / dt;
    return s_cum[i - 1] + fr * (s_cum[i] - s_cum[i - 1]);
  }
};

// Two-state receiver clock b [m], d [m/s] and the ISB random walks on the IMU grid.
struct ClockTruth {
  double dt = 0.005;
  std::vector<double> b, d;
  std::vector<double> isb_E, isb_C, isb_R;
  // Anchored at the nearest grid point: the filter references its clock at t_k (a grid point) and models only the
  // random walk over δ = t_k − t_g (here ~10 µs); anchoring at the previous point would put one whole 5 ms random-walk
  // step (≈ 8.5 mm/s of drift at A58's S_d) between every epoch and its t_k, an error the model does not have.
  int nearest(double tau) const {
    return std::max(0, std::min(static_cast<int>(std::lround(tau / dt)), static_cast<int>(b.size()) - 1));
  }
  double bAt(double tau) const {
    const int i = nearest(tau);
    return b[i] + d[i] * (tau - i * dt);
  }
  double dAt(double tau) const { return d[nearest(tau)]; }
  static double lin(const std::vector<double>& x, double dt, double tau) {
    const int i = std::max(0, std::min(static_cast<int>(std::floor(tau / dt)), static_cast<int>(x.size()) - 2));
    const double f = (tau - i * dt) / dt;
    return x[i] + f * (x[i + 1] - x[i]);
  }
};

// One generated receiver epoch.
struct GenEpoch {
  ObsEpoch epoch;
  double arrival = 0.0;   // host time at which the message is available
};

struct SatTrack {
  SatId id;
  int ambiguity = 0;
  double mp_psr = 0.0, mp_dopp = 0.0, mp_cp = 0.0;   // Gauss–Markov states (group B)
  double bias_psr = 0.0;                             // OpenSkyRealistic: constant part of the pass (unit variance)
  double prev_atmo = kNaN, prev_tau = kNaN;          // OpenSkyRealistic: T − I of the last epoch (Doppler rate)
  double nlos_left = 0.0, nlos_bias = 0.0;           // group B NLOS episode
  double next_slip = 1e300;                          // CycleSlips
  double dopp_bias = 0.0;                            // LowElevDopplerBias: constant range-rate bias below the elevation [m/s]
};

// A fresh distribution per draw: std::normal_distribution caches its second Box–Muller value, and a cached value
// must never travel from one run's engine into another's (runs are reproducible per seed).
// Sign of every Gaussian draw of the run in progress (SimParams::noise_sign; runs are single-threaded).
thread_local double g_noise_sign = 1.0;
double gauss(std::mt19937_64& rng) {
  std::normal_distribution<double> n(0.0, 1.0);
  return g_noise_sign * n(rng);
}
double uniform(std::mt19937_64& rng, double a, double b) { return std::uniform_real_distribution<double>(a, b)(rng); }

// Diagnostics sink of a run: NIS per class after activation + settle, events, summary.
class RunSink final : public DiagnosticsSink {
 public:
  explicit RunSink(double settle_s) : settle_s_(settle_s) {}
  void onEpoch(const EpochRecord& r) override {
    ++epochs;
    if (!(t_act_ == t_act_) || r.t_k < t_act_ + settle_s_) return;   // NaN until ACTIVATE
    for (int c = 0; c < 3; ++c) {
      const ClassStats& cs = r.cls[static_cast<size_t>(c)];
      rows[static_cast<size_t>(c)].candidates += cs.candidates;
      rows[static_cast<size_t>(c)].accepted += cs.accepted;
      for (size_t q = 0; q < cs.rejected.size(); ++q) rows[static_cast<size_t>(c)].rejected[q] += cs.rejected[q];
    }
    if (!(r.phase == Phase::TRACKING || r.phase == Phase::RECOVERY)) return;
    for (int c = 0; c < 3; ++c) {
      const ClassStats& cs = r.cls[static_cast<size_t>(c)];
      if (cs.dof <= 0 || !std::isfinite(cs.nis_prior)) continue;
      ClassNis& n = nis[static_cast<size_t>(c)];
      n.nis += cs.nis_prior;
      n.dof += cs.dof;
      ++n.epochs;
      if (cs.nis_prior >= quantile(0.025, cs.dof) && cs.nis_prior <= quantile(0.975, cs.dof)) ++n.in_band;
    }
  }
  void onEvent(const Event& e) override {
    ++events[static_cast<size_t>(e.kind)];
    if (e.kind == EventKind::FRAME_JUMP || e.kind == EventKind::REANCHOR || e.kind == EventKind::REINIT ||
        e.kind == EventKind::LIVO_ALARM_ON || e.kind == EventKind::ROLLBACK)
      event_log.push_back(std::string(eventKindName(e.kind)) + " t_k " + std::to_string(e.t_k) + " " + e.detail +
                          (e.kind == EventKind::ROLLBACK ? " (posterior NIS " + std::to_string(e.magnitude) + ")" : ""));
    if (e.kind == EventKind::ACTIVATE && !(t_act_ == t_act_)) {
      t_act_ = e.t_k;
      activate_detail = e.detail;
    }
  }
  void onSat(const SatRecord& s) override {
    if (s.slip_detectors == 0) return;
    SlipFlag f;
    f.t_gpst = s.gpst.week * 604800.0 + s.gpst.tow + 315964800.0;   // GTime::toSec() scale (GPS epoch in Unix seconds)
    f.sat = s.sat.rtk_sat;
    f.el_deg = s.el_deg;
    f.innov_m = s.innov_tdcp;
    f.sigma_m = s.sigma_tdcp;
    f.w = s.w_tdcp;
    f.detectors = s.slip_detectors;
    slip_flags.push_back(f);
  }
  void onSummary(const Summary& s) override { summary = s; }

  std::array<ClassNis, 3> nis{};
  std::array<int, static_cast<size_t>(EventKind::Count)> events{};
  Summary summary;
  int epochs = 0;
  std::string activate_detail;
  std::array<RowCounts, 3> rows{};
  std::vector<SlipFlag> slip_flags;
  std::vector<std::string> event_log;
  double tAct() const { return t_act_; }

 private:
  double quantile(double p, int dof) {
    const auto key = std::make_pair(p, dof);
    auto it = cache_.find(key);
    if (it != cache_.end()) return it->second;
    const double q = chi2Quantile(p, dof);
    cache_[key] = q;
    return q;
  }
  double settle_s_;
  double t_act_ = kNaN;
  std::map<std::pair<double, int>, double> cache_;
};

// ---------------------------------------------------------------------------------------------
// The world of one run.
// ---------------------------------------------------------------------------------------------
class Scene {
 public:
  // `drift_start_s`: scene time from which the registration random walk runs (+∞: never).
  Scene(const SimParams& p, const Config& cfg, double drift_start_s)
      : p_(p), cfg_(cfg), rng_(mixSeed(p)), drift_start_s_(drift_start_s) {}

  // Returns false when the BRDC file is missing.
  bool build(std::string* why) {
    if (p_.nav_rinex.empty()) {
      *why = "no nav_rinex";
      return false;
    }
    gen_cfg_ = cfg_;   // generator's own ephemeris store (same file, same selection rules as the filter)
    gen_cfg_.enable = true;
    nav_.reset(new rtk::NavStore());
    ephem_.reset(new EphemStore(gen_cfg_, *nav_));
    if (ephem_->loadRinex(p_.nav_rinex, why) <= 0) return false;
    have_ion_ = ephem_->iono(&ion_);
    solver_.reset(new rtk::SatSolver(*nav_));
    noise_.reset(new NoiseModel(cfg_));
    rows_.reset(new RowBuilder(cfg_, *noise_));
    r0_ = rtk::pos2ecef(p_.site);
    R_EN_ = rtk::enuRotation(p_.site);
    t_host0_ = static_cast<double>(p_.t0.sec) + p_.t0.frac - kLeapS;
    if (p_.draw_dov) {   // deflection of the vertical (first draws of the run)
      const double sd = cfg_.frames.sigma_dov_rad;
      dov_ = Eigen::Vector2d(sd * gauss(rng_), sd * gauss(rng_));
      gN_ = so3Exp(Vec3(dov_(0), dov_(1), 0.0)) * Vec3(0.0, 0.0, -kGrav);
    }
    if (p_.toff_sigma_s > 0.0) {   // its own stream: every other draw is that of a run without the offset
      std::mt19937_64 r2(mixSeed(p_) ^ 0x7f4a7c159e3779b9ULL);
      toff_ = p_.toff_sigma_s * gauss(r2);   // gauss() applies noise_sign
    }
    planMotion();
    integrateTruth();
    makeImu();
    makeClock();
    makeDrift();
    pickSatellites();
    makeEpochs();
    return true;
  }

  // ---- accessors for the runner ----
  const SimParams& params() const { return p_; }
  double duration() const { return T_; }
  double hostT0() const { return t_host0_; }
  int cycles() const { return K_; }
  const Truth& truth() const { return tr_; }
  const SimParams& simParams() const { return p_; }
  const std::vector<Vec3>& imuF() const { return f_meas_; }
  const std::vector<Vec3>& imuW() const { return w_meas_; }
  const std::vector<GenEpoch>& epochs() const { return epochs_; }
  // ξ_true(k) = T_{N*←W_k}: maps the host's map-frame coordinates of cycle k to N*. Drift in ξ (default): it applies to
  // the whole cycle (t_{k−1}, t_k] — the filter's H1 adds the increment before the cycle's GNSS update. Drift seen by
  // the LIO: it applies from the LIO update of cycle k on (the world is smooth; the host pose absorbs the increment).
  const Mat3& regM(int k) const { return M_[static_cast<size_t>(k)]; }
  const Vec3& regT(int k) const { return tau_[static_cast<size_t>(k)]; }
  // Host state (map-frame coordinates the LIO measures) at τ, on cycle k's map frame.
  struct HostState {
    Mat3 R = Mat3::Identity();
    Vec3 p = Vec3::Zero(), v = Vec3::Zero(), a = Vec3::Zero(), w = Vec3::Zero();
  };
  HostState hostCoords(int k, double tau) const {
    Mat3 Rs;
    Vec3 ps, vs, as, ws;
    tr_.at(tau, &Rs, &ps, &vs, &as, &ws);
    HostState h;
    h.w = ws;
    if (p_.drift_in_host) {
      const Mat3& M = regM(k);
      h.R = M.transpose() * Rs;
      h.p = M.transpose() * (ps - regT(k));
      h.v = M.transpose() * vs;
      h.a = M.transpose() * as;
    } else {
      const Mat3 A = Hm_[static_cast<size_t>(k)] * M_[0].transpose();
      h.R = A * Rs;
      h.p = A * (ps - tau_[0]) + Ht_[static_cast<size_t>(k)];
      h.v = A * vs;
      h.a = A * as;
    }
    return h;
  }
  // World (N*) antenna of a host state on cycle k's registration: position, velocity, acceleration.
  void antennaWorld(int k, const HostState& h, Vec3* pos, Vec3* vel, Vec3* acc) const {
    const Mat3& M = regM(k);
    const Vec3 l = p_.lever_I;
    if (pos) *pos = M * (h.p + h.R * l) + regT(k);
    if (vel) *vel = M * (h.v + h.R * h.w.cross(l));
    if (acc) *acc = M * (h.a + h.R * h.w.cross(h.w.cross(l)));
  }
  // Gravity in the host's map-frame coordinates of cycle k.
  Vec3 gravityHost(int k) const {
    if (p_.drift_in_host) return regM(k).transpose() * gN_;
    return Hm_[static_cast<size_t>(k)] * M_[0].transpose() * gN_;
  }
  // Cycle whose GNSS update consumes an epoch received at τ (epochs are processed at the first t_k ≥ τ).
  int cycleOf(double tau) const { return std::max(1, std::min(K_ + 1, static_cast<int>(std::ceil(tau / p_.lio_dt - 1e-9)))); }
  // LIO measurement offset of cycle k in the map frame (group C ghosting), zero otherwise
  const Vec3& ghost(int k) const { return ghost_[static_cast<size_t>(k)]; }
  const Vec3& r0() const { return r0_; }
  const Eigen::Vector2d& dov() const { return dov_; }
  double toff() const { return toff_; }
  double modelPsrMaxAbs() const { return model_psr_max_abs_; }
  const Mat3& R_EN() const { return R_EN_; }
  double bAt(double tau) const { return clk_.bAt(tau); }
  double dAt(double tau) const { return clk_.dAt(tau); }
  double isbAt(char sys, double tau) const {
    const std::vector<double>& x = sys == 'E' ? clk_.isb_E : sys == 'C' ? clk_.isb_C : clk_.isb_R;
    return ClockTruth::lin(x, clk_.dt, tau);
  }
  double bg0Sigma() const { return 2.0e-4; }
  double ba0Sigma() const { return 5.0e-3; }
  Vec3 bgTrue(int i) const { return bg_[static_cast<size_t>(i)]; }
  Vec3 baTrue(int i) const { return ba_[static_cast<size_t>(i)]; }
  double tMismatch() const { return t_mismatch_; }
  bool inMismatch(double tau) const;
  bool hostDeceived(double tau) const;
  int meanSats() const { return mean_sats_; }
  std::mt19937_64& rng() { return rng_; }

 private:
  static uint64_t mixSeed(const SimParams& p) {
    // LowElevDopplerBias is OpenSky's scene draw for draw (A66): its biases come from a stream of their own
    const Scenario s = p.scenario == Scenario::LowElevDopplerBias ? Scenario::OpenSky : p.scenario;
    return 0x9E3779B97F4A7C15ull * (static_cast<uint64_t>(p.seed) + 1) ^ (static_cast<uint64_t>(s) << 32) ^
           (p.fixed_anchor ? 0x51ED270B27Aull : 0ull);
  }

  void planMotion();
  void integrateTruth();
  void integrateReplay();
  void makeImu();
  void makeClock();
  void makeDrift();
  void pickSatellites();
  void makeEpochs();
  bool fewSatsAllowed(double tau, const SatId& id, const std::vector<std::pair<double, int>>& gps_by_el) const;

  SimParams p_;
  const Config& cfg_;
  Config gen_cfg_;   // EphemStore keeps a reference: must outlive it
  std::mt19937_64 rng_;
  double drift_start_s_ = 0.0;
  double model_psr_max_abs_ = 0.0;   // generator self-check (RunResult::model_psr_max_abs_m)
  Vec3 gN_ = Vec3(0.0, 0.0, -kGrav);   // true gravity in N* (tilted by the run's deflection of the vertical)
  Eigen::Vector2d dov_ = Eigen::Vector2d::Zero();
  double toff_ = 0.0;   // true GNSS↔host time offset δ: epoch generated at the host instant τ + δ
  std::unique_ptr<rtk::NavStore> nav_;
  std::unique_ptr<EphemStore> ephem_;
  std::unique_ptr<rtk::SatSolver> solver_;
  std::unique_ptr<NoiseModel> noise_;
  std::unique_ptr<RowBuilder> rows_;
  Klobuchar ion_;
  bool have_ion_ = false;
  Vec3 r0_ = Vec3::Zero();
  Mat3 R_EN_ = Mat3::Identity();
  double t_host0_ = 0.0;
  MotionPlan plan_;
  double T_ = 0.0;
  int K_ = 0;
  double psi0_ = 0.0;
  Mat3 R_mount_ = Mat3::Identity();
  Truth tr_;
  std::vector<Vec3> f_meas_, w_meas_, bg_, ba_;
  ClockTruth clk_;
  std::vector<Mat3> M_, Hm_;
  std::vector<Vec3> tau_, ghost_, Ht_;   // Hm_/Ht_: host-coordinate offset (drift in ξ; RevisitSnap re-localisation)
  double t_mismatch_ = kNaN;
  std::vector<std::pair<double, double>> ghost_windows_;
  std::vector<SatTrack> sats_;
  std::vector<GenEpoch> epochs_;
  int mean_sats_ = 0;
};

void Scene::planMotion() {
  plan_.speed_mid = p_.speed_mid_mps;
  plan_.speed_amp = p_.speed_amp_mps;
  plan_.speed_phase = uniform(rng_, -M_PI, M_PI);
  plan_.z_phase = uniform(rng_, -M_PI, M_PI);
  psi0_ = uniform(rng_, -M_PI, M_PI);
  R_mount_ = so3Exp(p_.mount_rpy);
  // Duration: the time the moving part needs for path_m at the mean speed, plus the standstill.
  const double t_move = p_.path_m / p_.speed_mid_mps;
  if (p_.scenario == Scenario::LongStill) {
    plan_.still_t0 = 0.45 * t_move;
    plan_.still_dur = 120.0;
  }
  // Turns: a straight start (initialisation needs ≥ 10 s of motion), then straight / turn alternating.
  double t = 25.0;
  const double t_end_guess = t_move + (plan_.still_t0 >= 0.0 ? plan_.still_dur + 2.0 * plan_.ramp : 0.0) + 60.0;
  while (t < t_end_guess) {
    const double ang = (uniform(rng_, 0.0, 1.0) < 0.5 ? -1.0 : 1.0) * uniform(rng_, 60.0, 150.0) * kDeg;
    const double dur = std::fabs(ang) / p_.turn_rate_rps;
    const bool overlaps_still = plan_.still_t0 >= 0.0 && t + dur > plan_.still_t0 - 1.0 &&
                                t < plan_.still_t0 + plan_.still_dur + 2.0 * plan_.ramp + 1.0;
    if (!overlaps_still) plan_.turns.push_back({t, dur, ang});
    t += (overlaps_still ? 5.0 : dur) + uniform(rng_, 20.0, 60.0);
  }
}

void Scene::integrateReplay() {
  std::ifstream in(p_.replay_enu_tum);
  if (!in) throw std::runtime_error("sim: cannot read replay trajectory " + p_.replay_enu_tum);
  std::vector<double> tt;
  std::vector<Vec3> pp;
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::istringstream ss(line);
    double t, x, y, z;
    if (!(ss >> t >> x >> y >> z)) continue;
    tt.push_back(t);
    pp.emplace_back(x, y, z);
  }
  if (tt.size() < 10) throw std::runtime_error("sim: replay trajectory has fewer than 10 poses");
  const double t0 = tt.front();
  for (double& t : tt) t -= t0;
  const double dt = p_.imu_dt;
  T_ = std::floor(tt.back() / p_.lio_dt) * p_.lio_dt;
  if (T_ < p_.lio_dt) throw std::runtime_error("sim: replay trajectory shorter than one LIO cycle");
  K_ = static_cast<int>(std::floor(T_ / p_.lio_dt + 1e-9)) - 1;
  const int n = static_cast<int>(std::lround(T_ / dt));
  auto at = [&](double t) {
    if (t <= tt.front()) return pp.front();
    if (t >= tt.back()) return pp.back();
    const auto it = std::upper_bound(tt.begin(), tt.end(), t);
    const int i1 = static_cast<int>(it - tt.begin());
    const int i0 = i1 - 1;
    const double a = (t - tt[static_cast<size_t>(i0)]) / (tt[static_cast<size_t>(i1)] - tt[static_cast<size_t>(i0)]);
    Vec3 out = pp[static_cast<size_t>(i0)] * (1.0 - a) + pp[static_cast<size_t>(i1)] * a;
    return out;
  };
  tr_.dt = dt;
  tr_.n = n;
  tr_.R.assign(n + 1, Mat3::Identity());
  tr_.p.assign(n + 1, Vec3::Zero());
  tr_.v.assign(n + 1, Vec3::Zero());
  tr_.a.assign(n + 1, Vec3::Zero());
  tr_.w.assign(n + 1, Vec3::Zero());
  tr_.f_ideal.assign(n + 1, Vec3::Zero());
  tr_.w_ideal.assign(n + 1, Vec3::Zero());
  tr_.s_cum.assign(n + 1, 0.0);
  tr_.alpha_cum.assign(n + 1, 0.0);
  std::vector<Vec3> pos(n + 1), vel(n + 1), acc(n + 1);
  std::vector<double> yaw(n + 1, 0.0), yr(n + 1, 0.0);
  for (int i = 0; i <= n; ++i) pos[static_cast<size_t>(i)] = at(i * dt);
  for (int i = 1; i <= n; ++i) vel[static_cast<size_t>(i)] = (pos[static_cast<size_t>(i)] - pos[static_cast<size_t>(i - 1)]) / dt;
  vel[0] = vel[1];
  for (int i = 1; i <= n; ++i) acc[static_cast<size_t>(i)] = (vel[static_cast<size_t>(i)] - vel[static_cast<size_t>(i - 1)]) / dt;
  acc[0] = acc[1];
  for (int i = 0; i <= n; ++i) {
    const double ve = vel[static_cast<size_t>(i)](0), vn = vel[static_cast<size_t>(i)](1);
    if (ve * ve + vn * vn > 0.05 * 0.05) yaw[static_cast<size_t>(i)] = std::atan2(ve, vn);
    else if (i > 0) yaw[static_cast<size_t>(i)] = yaw[static_cast<size_t>(i - 1)];
  }
  for (int i = 1; i <= n; ++i) {
    double d = yaw[static_cast<size_t>(i)] - yaw[static_cast<size_t>(i - 1)];
    if (d > M_PI) d -= 2.0 * M_PI;
    if (d < -M_PI) d += 2.0 * M_PI;
    yr[static_cast<size_t>(i)] = d / dt;
  }
  const Vec3& g_N = gN_;
  for (int i = 0; i <= n; ++i) {
    const Mat3 Ri = Rz(yaw[static_cast<size_t>(i)]) * R_mount_;
    tr_.R[static_cast<size_t>(i)] = Ri;
    tr_.p[static_cast<size_t>(i)] = pos[static_cast<size_t>(i)] - pos[0];
    tr_.v[static_cast<size_t>(i)] = vel[static_cast<size_t>(i)];
    tr_.a[static_cast<size_t>(i)] = acc[static_cast<size_t>(i)];
    tr_.w[static_cast<size_t>(i)] = R_mount_.transpose() * Vec3(0.0, 0.0, yr[static_cast<size_t>(i)]);
    tr_.w_ideal[static_cast<size_t>(i)] = tr_.w[static_cast<size_t>(i)];
    tr_.f_ideal[static_cast<size_t>(i)] = Ri.transpose() * (acc[static_cast<size_t>(i)] - g_N);
    if (i > 0) {
      tr_.s_cum[static_cast<size_t>(i)] = tr_.s_cum[static_cast<size_t>(i - 1)] + (tr_.p[static_cast<size_t>(i)] - tr_.p[static_cast<size_t>(i - 1)]).norm();
      tr_.alpha_cum[static_cast<size_t>(i)] = tr_.alpha_cum[static_cast<size_t>(i - 1)] + std::fabs(yr[static_cast<size_t>(i)]) * dt;
    }
  }
}

void Scene::integrateTruth() {
  if (!p_.replay_enu_tum.empty()) {
    integrateReplay();
    return;
  }
  const double dt = p_.imu_dt;
  // Find the duration by the path length (moving part) on the IMU grid.
  double s_path = 0.0, tt = 0.0;
  while (s_path < p_.path_m) {
    double s, sd, yr, vz, az;
    plan_.eval(tt, &s, &sd, &yr, &vz, &az);
    s_path += s * dt;
    tt += dt;
    if (tt > 1e5) throw std::runtime_error("sim: path never reaches path_m");
  }
  T_ = std::floor((tt + 1.0) / p_.lio_dt) * p_.lio_dt;
  K_ = static_cast<int>(std::floor(T_ / p_.lio_dt + 1e-9)) - 1;
  const int n = static_cast<int>(std::lround(T_ / dt));
  tr_.dt = dt;
  tr_.n = n;
  tr_.R.assign(n + 1, Mat3::Identity());
  tr_.p.assign(n + 1, Vec3::Zero());
  tr_.v.assign(n + 1, Vec3::Zero());
  tr_.a.assign(n + 1, Vec3::Zero());
  tr_.w.assign(n + 1, Vec3::Zero());
  tr_.f_ideal.assign(n + 1, Vec3::Zero());
  tr_.w_ideal.assign(n + 1, Vec3::Zero());
  tr_.s_cum.assign(n + 1, 0.0);
  tr_.alpha_cum.assign(n + 1, 0.0);
  const Vec3& g_N = gN_;
  // ideal samples: heading by the trapezoid rule of ψ̇ (the same rule the host's midpoint rotation integrates)
  double psi = psi0_, yr_prev = 0.0;
  std::vector<double> yaw_rate(n + 1, 0.0);
  for (int i = 0; i <= n; ++i) {
    double s, sd, yr, vz, az;
    plan_.eval(i * dt, &s, &sd, &yr, &vz, &az);
    if (i > 0) psi += 0.5 * (yr_prev + yr) * dt;
    yr_prev = yr;
    yaw_rate[i] = yr;
    const double c = std::cos(psi), sn = std::sin(psi);
    const Vec3 a_N(sd * c - s * yr * sn, sd * sn + s * yr * c, az);
    const Mat3 Ri = Rz(psi) * R_mount_;
    tr_.w_ideal[i] = R_mount_.transpose() * Vec3(0.0, 0.0, yr);
    tr_.f_ideal[i] = Ri.transpose() * (a_N - g_N);
    if (i == 0) {
      tr_.R[0] = Ri;
      tr_.p[0] = Vec3::Zero();
      tr_.v[0] = Vec3(s * c, s * sn, vz);
    }
  }
  for (int i = 1; i <= n; ++i) {
    const Vec3 wb = 0.5 * (tr_.w_ideal[i - 1] + tr_.w_ideal[i]);
    const Vec3 fb = 0.5 * (tr_.f_ideal[i - 1] + tr_.f_ideal[i]);
    tr_.R[i] = tr_.R[i - 1] * so3Exp(wb * dt);
    const Vec3 a = tr_.R[i] * fb + g_N;
    tr_.p[i] = tr_.p[i - 1] + tr_.v[i - 1] * dt + 0.5 * a * dt * dt;
    tr_.v[i] = tr_.v[i - 1] + a * dt;
    tr_.a[i] = a;
    tr_.w[i] = wb;
    tr_.s_cum[i] = tr_.s_cum[i - 1] + (tr_.p[i] - tr_.p[i - 1]).norm();
    tr_.alpha_cum[i] = tr_.alpha_cum[i - 1] + std::fabs(0.5 * (yaw_rate[i - 1] + yaw_rate[i])) * dt;
  }
  tr_.a[0] = tr_.a[1];
  tr_.w[0] = tr_.w[1];
}

void Scene::makeImu() {
  const int n = tr_.n;
  const double dt = p_.imu_dt;
  f_meas_.assign(n + 1, Vec3::Zero());
  w_meas_.assign(n + 1, Vec3::Zero());
  bg_.assign(n + 1, Vec3::Zero());
  ba_.assign(n + 1, Vec3::Zero());
  Vec3 bg(gauss(rng_), gauss(rng_), gauss(rng_)), ba(gauss(rng_), gauss(rng_), gauss(rng_));
  bg *= bg0Sigma();
  ba *= ba0Sigma();
  ba += p_.turnon_ba;
  const double sa = p_.acc_n / std::sqrt(dt), sg = p_.gyr_n / std::sqrt(dt);
  const double wa = p_.acc_w * std::sqrt(dt), wg = p_.gyr_w * std::sqrt(dt);
  for (int i = 0; i <= n; ++i) {
    if (i > 0) {
      bg += wg * Vec3(gauss(rng_), gauss(rng_), gauss(rng_));
      ba += wa * Vec3(gauss(rng_), gauss(rng_), gauss(rng_));
    }
    bg_[i] = bg;
    ba_[i] = ba;
    f_meas_[i] = tr_.f_ideal[i] + ba + sa * Vec3(gauss(rng_), gauss(rng_), gauss(rng_));
    w_meas_[i] = tr_.w_ideal[i] + bg + sg * Vec3(gauss(rng_), gauss(rng_), gauss(rng_));
  }
}

void Scene::makeClock() {
  const int n = tr_.n;
  const double dt = p_.imu_dt;
  const double Sb = cfg_.process.clock.S_b, Sd = cfg_.process.clock.S_d;
  Eigen::Matrix2d Q;
  Q << Sb * dt + Sd * dt * dt * dt / 3.0, Sd * dt * dt / 2.0, Sd * dt * dt / 2.0, Sd * dt;
  const Eigen::Matrix2d L = Q.llt().matrixL();
  clk_.dt = dt;
  clk_.b.assign(n + 1, 0.0);
  clk_.d.assign(n + 1, 0.0);
  clk_.isb_E.assign(n + 1, 0.0);
  clk_.isb_C.assign(n + 1, 0.0);
  clk_.isb_R.assign(n + 1, 0.0);
  std::vector<double> jumps;
  if (p_.scenario == Scenario::ClockJumps) jumps = {0.3 * T_, 0.5 * T_, 0.7 * T_};
  double b = p_.clk_b0_m, d = p_.clk_d0_mps;
  double iE = 4.0, iC = -6.0, iR = 2.0;
  const double si = std::sqrt(cfg_.process.isb_q * dt);
  size_t next_jump = 0;
  for (int i = 0; i <= n; ++i) {
    if (i > 0) {
      const Eigen::Vector2d w = L * Eigen::Vector2d(gauss(rng_), gauss(rng_));
      b = b + d * dt + w(0);
      d = d + w(1);
      iE += si * gauss(rng_);
      iC += si * gauss(rng_);
      iR += si * gauss(rng_);
      if (next_jump < jumps.size() && i * dt >= jumps[next_jump]) {
        b += kC * 1e-3;   // u-blox style integer-millisecond clock step (DESIGN §10 L2)
        ++next_jump;
      }
    }
    clk_.b[i] = b;
    clk_.d[i] = d;
    clk_.isb_E[i] = iE;
    clk_.isb_C[i] = iC;
    clk_.isb_R[i] = iR;
  }
}

bool Scene::inMismatch(double tau) const {
  if (scenarioGroup(p_.scenario) != Group::Mismatch) return false;
  if (p_.scenario == Scenario::MapGhosting) {
    for (const auto& w : ghost_windows_)
      if (tau >= w.first && tau < w.second + 30.0) return true;
    return false;
  }
  return std::isfinite(t_mismatch_) && tau >= t_mismatch_;
}

bool Scene::hostDeceived(double tau) const {
  for (const auto& w : ghost_windows_)
    if (tau >= w.first && tau < w.second) return true;
  return false;
}

void Scene::makeDrift() {
  // Registration truth ξ(k) = T_{N*←W_k}, k = 0..K+1 (DESIGN §3.2 process at the filter's q; group C adds its violation).
  const Config::Process::Frame& q = cfg_.process.frame;
  const int K = K_ + 1;
  M_.assign(K + 1, Mat3::Identity());
  tau_.assign(K + 1, Vec3::Zero());
  Hm_.assign(K + 1, Mat3::Identity());
  Ht_.assign(K + 1, Vec3::Zero());
  ghost_.assign(K + 1, Vec3::Zero());
  M_[0] = tr_.R[0];     // W0 = the IMU frame at start-up (FAST-LIVO2)
  tau_[0] = tr_.p[0];
  const double k_yaw_det = 4.0e-5;                  // YawDriftMismatch: deterministic 4e-5 rad/m (2.3° per km)
  const double t_old = 0.15 * T_, t_snap = 0.65 * T_;
  int k_old = -1;
  if (p_.scenario == Scenario::YawDriftMismatch) t_mismatch_ = 0.3 * T_;
  if (p_.scenario == Scenario::RevisitSnap) t_mismatch_ = t_snap;
  if (p_.scenario == Scenario::LowElevDopplerBias) t_mismatch_ = 0.0;   // the biases are there from the start
  if (p_.scenario == Scenario::MapGhosting) {
    ghost_windows_ = {{0.45 * T_, 0.45 * T_ + 20.0}, {0.65 * T_, 0.65 * T_ + 20.0}};
    t_mismatch_ = ghost_windows_.front().first;
  }
  for (int k = 1; k <= K; ++k) {
    const double t1 = std::min(k * p_.lio_dt, T_), t0 = (k - 1) * p_.lio_dt;
    const int i1 = std::min(tr_.n, static_cast<int>(std::lround(t1 / tr_.dt)));
    const int i0 = std::min(tr_.n, static_cast<int>(std::lround(t0 / tr_.dt)));
    // what the filter sums over its bridge samples; the draws are made whether or not the walk has started, so the scene
    // before the start is the same draw for draw whatever the start (drift_from_activation's two passes)
    const bool walk = t1 > drift_start_s_ + 1e-9;
    const double ds = walk ? tr_.s_cum[i1] - tr_.s_cum[i0] : 0.0;
    const double da = walk ? tr_.alpha_cum[i1] - tr_.alpha_cum[i0] : 0.0;
    Eigen::Vector2d wphi(gauss(rng_), gauss(rng_));
    wphi *= std::sqrt(q.q_tilt_per_m * ds);
    double wpsi = std::sqrt(q.q_yaw_per_m * ds + q.q_yaw_per_rad * std::max(0.0, da)) * gauss(rng_);
    if (p_.scenario == Scenario::YawDriftMismatch && t1 >= t_mismatch_) wpsi += k_yaw_det * (tr_.s_cum[i1] - tr_.s_cum[i0]);
    const Vec3 wt(std::sqrt(q.q_th_per_m * ds) * gauss(rng_), std::sqrt(q.q_th_per_m * ds) * gauss(rng_),
                  std::sqrt(q.q_tv_per_m * ds) * gauss(rng_));
    const Vec3 dr(wphi(0), wphi(1), wpsi);
    Hm_[k] = Hm_[k - 1];
    Ht_[k] = Ht_[k - 1];
    // current antenna point (at t_k, on the previous registration): stays put under the rotation (DESIGN §3.2)
    Vec3 m;
    {
      M_[k] = M_[k - 1];
      tau_[k] = tau_[k - 1];
      antennaWorld(k - 1, hostCoords(k - 1, t1), &m, nullptr, nullptr);
    }
    const Mat3 E = so3Exp(dr);
    M_[k] = E * M_[k - 1];
    tau_[k] = E * (tau_[k - 1] - m) + m + wt;
    if (p_.scenario == Scenario::RevisitSnap) {
      if (k_old < 0 && t1 >= t_old) k_old = k;
      if (k_old >= 0 && t1 >= t_snap && t1 - p_.lio_dt < t_snap) {
        // relocalised against the old map: the registration returns to ξ(k_old); the world does not move, so the
        // host's map-frame coordinates jump (H ← ξ_old⁻¹ ∘ ξ(k) ∘ H) — or, drift seen by the LIO, simply follow ξ_old
        if (!p_.drift_in_host) {
          const Mat3 Mo = M_[k_old];
          const Vec3 To = tau_[k_old];
          const Mat3 Hm = Mo.transpose() * M_[k] * Hm_[k];
          const Vec3 Ht = Mo.transpose() * (M_[k] * Ht_[k] + tau_[k] - To);
          Hm_[k] = Hm;
          Ht_[k] = Ht;
        }
        M_[k] = M_[k_old];
        tau_[k] = tau_[k_old];
      }
    }
    if (p_.scenario == Scenario::MapGhosting) {
      for (const auto& w : ghost_windows_)
        if (t1 >= w.first && t1 < w.second) ghost_[k] = Vec3(0.4, -0.2, 0.0);   // the LIO snaps to a ghost copy
    }
  }
}

void Scene::pickSatellites() {
  std::vector<Sys> systems = {Sys::GPS, Sys::GAL};
  if (p_.scenario == Scenario::FourSystemsIsb) systems = {Sys::GPS, Sys::GAL, Sys::BDS, Sys::GLO};
  if (p_.scenario == Scenario::UrbanMultipath || p_.scenario == Scenario::OpenSkyRealistic) systems = {Sys::GPS, Sys::GAL, Sys::BDS};
  std::set<int> seen;
  // A66: the Doppler biases on a stream of their own (uniform draws: the antithetic twin keeps them)
  std::mt19937_64 rb(mixSeed(p_) ^ 0x5bd1e995a66d0b1aULL);
  for (const rtk::EphKey& k : nav_->coverage()) {
    if (!seen.insert(k.sat.rtk_sat).second) continue;
    if (std::find(systems.begin(), systems.end(), k.sat.sys) == systems.end()) continue;
    if (k.sat.sys == Sys::GLO && nav_->gloFcn(k.sat.prn) < -7) continue;
    rtk::SatState st;
    if (!solver_->satState(k.sat, p_.t0, 2.2e7, &st) || !st.ok || !rtk::svhHealthy(k.sat.sys, st.svh)) continue;
    SatTrack s;
    s.id = k.sat;
    s.ambiguity = std::uniform_int_distribution<int>(-2000000, 2000000)(rng_);
    if (p_.scenario == Scenario::OpenSkyRealistic) {
      s.bias_psr = gauss(rng_);
      s.mp_psr = gauss(rng_);   // stationary start of the Gauss–Markov part
    }
    if (p_.scenario == Scenario::CycleSlips) s.next_slip = 0.2 * T_ + uniform(rng_, 0.0, 60.0);
    if (p_.scenario == Scenario::LowElevDopplerBias) {
      const double sgn = uniform(rb, 0.0, 1.0) < 0.5 ? -1.0 : 1.0;
      s.dopp_bias = sgn * uniform(rb, p_.dopp_bias_lo_mps, p_.dopp_bias_hi_mps);
    }
    sats_.push_back(s);
  }
}

bool Scene::fewSatsAllowed(double tau, const SatId& id, const std::vector<std::pair<double, int>>& gps_by_el) const {
  if (p_.scenario != Scenario::FewSats) return true;
  const double a = 0.4 * T_;
  int keep = 0;
  if (tau >= a && tau < a + 60.0) keep = 3;
  else if (tau >= a + 60.0 && tau < a + 90.0) keep = 2;
  if (keep == 0) return true;
  for (int i = 0; i < std::min(keep, static_cast<int>(gps_by_el.size())); ++i)
    if (gps_by_el[static_cast<size_t>(i)].second == id.rtk_sat) return true;
  return false;
}

void Scene::makeEpochs() {
  const Group grp = scenarioGroup(p_.scenario);
  // The carrier's delays (+T − I): their change over a TDCP pair (up to ~2 mm / 0.5 s at low elevation) is in the TDCP
  // model since A61 F2; −1 keeps the pre-A61 study setting (realistic group only).
  const bool atmo = p_.carrier_atmo < 0 ? grp == Group::Realistic : p_.carrier_atmo > 0;
  const int per_s = static_cast<int>(std::lround(1.0 / p_.gnss_dt));
  const double out0 = 0.4 * T_, out1 = out0 + 120.0;
  double last_arrival = -1e300;
  long n_sats_total = 0;
  for (int j = 0;; ++j) {
    GTime t_rx = p_.t0;
    t_rx.sec += j / per_s;
    t_rx.frac = static_cast<double>(j % per_s) / per_s;
    const double tau_rx = t_rx.diff(p_.t0);
    if (tau_rx > T_ + 1.0) break;
    double tau = tau_rx - clk_.bAt(std::max(0.0, tau_rx)) / kC;
    tau = tau_rx - clk_.bAt(std::max(0.0, tau)) / kC;
    if (tau < 0.05 || tau > T_ - 0.3) continue;
    if (p_.scenario == Scenario::Outage120s && tau >= out0 && tau < out1) continue;
    // The world antenna: the host's map-frame state on the registration of the cycle that consumes this epoch (drift in
    // ξ); with the drift seen by the LIO the same expression is the smooth world trajectory.
    const int kc = cycleOf(tau);
    const HostState hs = hostCoords(p_.drift_in_host ? std::max(0, kc - 1) : kc, tau + toff_);
    Vec3 antN, antV, antA;
    antennaWorld(p_.drift_in_host ? std::max(0, kc - 1) : kc, hs, &antN, &antV, &antA);
    const Vec3& wI = hs.w;
    const Vec3 rA = r0_ + R_EN_ * antN, vA = R_EN_ * antV;
    const Geodetic geo = rtk::ecef2pos(rA);
    const double b = clk_.bAt(tau), d = clk_.dAt(tau);
    GenEpoch ge;
    ge.epoch.t_rx = t_rx;
    ge.epoch.source = Source::GnssCommMeas;
    ge.epoch.seq = static_cast<uint32_t>(j);
    // few-satellites window: GPS by elevation (computed first)
    std::vector<std::pair<double, int>> gps_by_el;
    struct Pre {
      size_t idx;
      rtk::SatState st;
      Vec3 e;
      double rho, az, el;
    };
    std::vector<Pre> pre;
    for (size_t s = 0; s < sats_.size(); ++s) {
      const SatId& id = sats_[s].id;
      rtk::SatState st;
      Vec3 e = Vec3::Zero();
      double P = 2.2e7, rho = 0.0;
      bool ok = true;
      for (int it = 0; it < 3 && ok; ++it) {
        ok = solver_->satState(id, t_rx, P, &st) && st.ok;
        if (!ok) break;
        rho = rtk::geodist(st.r_ecef, rA, &e);
        P = rho + b - kC * st.dts;
      }
      if (!ok || !rtk::svhHealthy(id.sys, st.svh)) continue;
      double az = 0.0, el = 0.0;
      rtk::satAzel(geo, e, &az, &el);
      if (el < kElMaskRad) continue;
      pre.push_back({s, st, e, rho, az, el});
      if (id.sys == Sys::GPS) gps_by_el.emplace_back(-el, id.rtk_sat);
    }
    std::sort(gps_by_el.begin(), gps_by_el.end());
    for (auto& g : gps_by_el) g.first = -g.first;
    for (const Pre& q : pre) {
      SatTrack& trk = sats_[q.idx];
      const SatId& id = trk.id;
      if (!fewSatsAllowed(tau, id, gps_by_el)) continue;
      const double f = id.sys == Sys::GLO ? 1602.0e6 + nav_->gloFcn(id.prn) * 0.5625e6 : id.sys == Sys::BDS ? 1561.098e6 : kFreqL1;
      const double lam = kC / f;
      const SignalCode code = id.sys == Sys::GAL ? SignalCode::L1X : id.sys == Sys::BDS ? SignalCode::L2I : SignalCode::L1C;
      const double trop = rtk::tropModel(t_rx, geo, q.az, q.el, kHumi);
      const double iono = have_ion_ ? rtk::ionModelGpsL1(t_rx, ion_, geo, q.az, q.el) * (kFreqL1 / f) * (kFreqL1 / f) : 0.0;
      const double isb = id.sys == Sys::GAL ? isbAt('E', tau) : id.sys == Sys::BDS ? isbAt('C', tau) : id.sys == Sys::GLO ? isbAt('R', tau) : 0.0;
      const double cb = rtk::codeBiasMetres(id, q.st, code);
      if (!std::isfinite(cb)) continue;
      const Vec3& rs = q.st.r_ecef;
      const Vec3& vs = q.st.v_ecef;
      // range rate with the derivative-sign Sagnac rate (AMENDMENTS A36)
      const double rate = q.e.dot(vs - vA) + kOmgE / kC * (vs.x() * rA.y() + rs.x() * vA.y() - vs.y() * rA.x() - rs.y() * vA.x());
      double P = q.rho + b + isb - kC * q.st.dts + trop + iono + cb;
      double rr = rate + d - kC * q.st.ddts;                       // −λD
      double L = q.rho + b - kC * q.st.dts + (atmo ? trop - iono : 0.0);   // carrier [m] without the ambiguity
      const double tstep = p_.gnss_dt;
      // A66: below dopp_bias_el_deg the satellite's Doppler carries its constant bias (and the configured receiver std)
      const bool low_bias = p_.scenario == Scenario::LowElevDopplerBias && q.el < p_.dopp_bias_el_deg * kDeg;
      // C/N0: 45 dB-Hz in the model-matched groups; sky-dependent in the realistic ones (M3DGR's receiver reads low)
      const double cn0 = p_.scenario == Scenario::OpenSkyRealistic ? p_.realistic_cn0_base_db + 14.0 * std::sin(q.el)
                         : grp == Group::Realistic                 ? 30.0 + 18.0 * std::sin(q.el)
                         : low_bias                                ? p_.dopp_bias_cn0_db
                                                                   : 45.0;
      uint8_t lli = 0;
      if (grp != Group::Realistic) {
        // Model-matched: the variance of every term the scene does not realise physically (A60).
        SatObs so;
        so.sat = id;
        so.t_rx = t_rx;
        SignalObs sig;
        sig.freq_hz = f;
        sig.code = code;
        sig.psr_m = P;
        sig.status = kStatusPsrValid;
        so.sig.push_back(sig);
        SatGeometry g;
        if (!rows_->geometry(so, *solver_, rA, geo, have_ion_ ? &ion_ : nullptr, t_rx, b, &g)) continue;
        NoiseModel::PsrTerms pt;
        pt.el_rad = g.el_rad;
        pt.cn0 = cn0;   // 45 dB-Hz, or the reported C/N0 of A66's biased satellites
        pt.psr_std_m = 0.0;
        pt.sys = id.sys;
        pt.bds_geo = QualityGate::isBdsGeo(id);
        pt.var_eph = g.state.var_eph;
        pt.var_ion = g.var_ion;
        pt.var_trop = g.var_trop;
        pt.dt_since_last_s = cfg_.noise.psr.decim_s > 0.0 ? cfg_.noise.psr.decim_s : 1.0;
        const double var_p = noise_->psrVar(pt);
        {   // self-check: the filter's psr model (rows.cpp §4.3) at the true antenna and clock
          const double h = g.range + b + isb - kC * g.state.dts + g.trop_m + g.ion_m + rtk::codeBiasMetres(id, g.state, code);
          model_psr_max_abs_ = std::max(model_psr_max_abs_, std::fabs(P - h));
        }
        NoiseModel::DoppTerms dtm;
        dtm.el_rad = g.el_rad;
        dtm.lambda_m = lam;
        dtm.dopp_std_hz = low_bias ? p_.dopp_bias_std_hz : 0.0;   // A66: the biased rows' receiver std (0 = group A)
        dtm.omega_norm = wI.norm();
        dtm.ant_acc_norm = antA.norm();
        dtm.v_norm = antV.norm();
        dtm.dt_since_last_s = 0.1;
        const double var_d = noise_->doppVar(dtm);
        const double sig_cp = noise_->sigmaPhase(0.0, g.el_rad) * std::sqrt(cfg_.noise.scale.tdcp);
        P += std::sqrt(var_p) * gauss(rng_);
        rr += std::sqrt(var_d) * gauss(rng_);
        L += sig_cp * gauss(rng_);
      } else if (p_.scenario == Scenario::OpenSkyRealistic) {
        // A58's error picture (AMENDMENTS A58 table): the marginal code variance of the calibrated model's receiver term,
        // σ0²(1 + 1/sin²el)·10^{(45−C/N0)/10}·scale.psr, 75 % Gauss–Markov with τ_ρ and 25 % constant over the pass (the
        // 0.2–0.3 autocorrelation plateau A58 saw to 120 s); the single-sample Doppler variance σ_D0²(1 + 1/sin²el)/2·scale
        // (τ_D = 0.047 s: white at 10 Hz) plus the atmospheric rate the receiver's Doppler carries; the carrier white at
        // σ_Φ0/sin el·√scale.tdcp.
        const Config::Noise& nz = cfg_.noise;
        const double s_el = std::max(std::sin(q.el), std::sin(5.0 * kDeg));
        const double var_p = nz.psr.sigma0_m * nz.psr.sigma0_m * (1.0 + 1.0 / (s_el * s_el)) *
                              std::pow(10.0, std::max(0.0, nz.psr.cn0_ref - cn0) / 10.0) * nz.scale.psr;
        const double rho_p = std::exp(-tstep / std::max(nz.psr.tau_s, 1e-3));
        trk.mp_psr = rho_p * trk.mp_psr + std::sqrt(1.0 - rho_p * rho_p) * gauss(rng_);
        P += std::sqrt(var_p) * (std::sqrt(0.75) * trk.mp_psr + std::sqrt(0.25) * trk.bias_psr);
        const double var_d = nz.dopp.sigma0_mps * nz.dopp.sigma0_mps * 0.5 * (1.0 + 1.0 / (s_el * s_el)) * nz.scale.dopp;
        rr += std::sqrt(var_d) * gauss(rng_);
        const double atmo_now = trop - iono;
        if (std::isfinite(trk.prev_atmo) && tau - trk.prev_tau > 0.0 && tau - trk.prev_tau < 0.5)
          rr += (atmo_now - trk.prev_atmo) / (tau - trk.prev_tau);
        trk.prev_atmo = atmo_now;
        trk.prev_tau = tau;
        L += nz.tdcp.sigma0_m / s_el * std::sqrt(nz.scale.tdcp) * gauss(rng_);
      } else {
        // Realistic: white + Gauss–Markov multipath (τ = 30 s, σ = 1.5 m code), NLOS episodes, elevation scaling.
        const double elf = std::sqrt(0.5 * (1.0 + 1.0 / std::pow(std::max(std::sin(q.el), 0.1), 2)));
        const double rho_mp = std::exp(-tstep / 30.0), rho_dp = std::exp(-tstep / 1.0);
        trk.mp_psr = rho_mp * trk.mp_psr + std::sqrt(1.0 - rho_mp * rho_mp) * 1.5 * gauss(rng_);
        trk.mp_dopp = rho_dp * trk.mp_dopp + std::sqrt(1.0 - rho_dp * rho_dp) * 0.03 * gauss(rng_);
        trk.mp_cp = rho_mp * trk.mp_cp + std::sqrt(1.0 - rho_mp * rho_mp) * 0.005 * gauss(rng_);
        if (trk.nlos_left <= 0.0 && q.el < 35.0 * kDeg && uniform(rng_, 0.0, 1.0) < tstep / 300.0) {
          trk.nlos_left = uniform(rng_, 10.0, 30.0);
          trk.nlos_bias = uniform(rng_, 5.0, 40.0);
        }
        double nlos = 0.0;
        if (trk.nlos_left > 0.0) {
          nlos = trk.nlos_bias;
          trk.nlos_left -= tstep;
        }
        P += elf * (0.3 * gauss(rng_) + trk.mp_psr) + nlos;
        rr += elf * (0.05 * gauss(rng_) + trk.mp_dopp);
        L += elf * (0.003 * gauss(rng_) + trk.mp_cp);
      }
      if (p_.scenario == Scenario::CycleSlips && tau >= trk.next_slip) {
        const int k = std::uniform_int_distribution<int>(1, 20)(rng_) * (uniform(rng_, 0.0, 1.0) < 0.5 ? -1 : 1);
        trk.ambiguity += k;
        if (uniform(rng_, 0.0, 1.0) < 0.5) lli = 1;   // half flagged by the receiver (D1), half left to D4/D5/L4
        trk.next_slip = tau + uniform(rng_, 40.0, 80.0);
      }
      if (low_bias) rr += trk.dopp_bias;
      SatObs so;
      so.sat = id;
      so.t_rx = t_rx;
      SignalObs sig;
      sig.freq_hz = f;
      sig.code = code;
      sig.psr_m = P;
      sig.dopp_hz = -rr / lam;
      if (low_bias && p_.dopp_bias_std_hz > 0.0) {
        sig.dopp_std_hz = p_.dopp_bias_std_hz;
        sig.dopp_std_unit = StdUnit::Hertz;
      }
      sig.cp_cyc = L / lam + trk.ambiguity;
      sig.cn0_dbhz = cn0;
      sig.lli = lli;
      sig.status = kStatusPsrValid | kStatusCpValid | kStatusHalfCycValid;
      so.sig.push_back(sig);
      ge.epoch.sats.push_back(so);
    }
    const double t_est = t_host0_ + tau;
    ge.arrival = std::max(last_arrival, t_est + p_.epoch_latency_s);
    last_arrival = ge.arrival;
    ge.epoch.t_record = ge.arrival;
    n_sats_total += static_cast<long>(ge.epoch.sats.size());
    epochs_.push_back(std::move(ge));
  }
  mean_sats_ = epochs_.empty() ? 0 : static_cast<int>(n_sats_total / static_cast<long>(epochs_.size()));
}

// ---------------------------------------------------------------------------------------------
// The synthetic host: FAST-LIVO2's error-state filter reduced to IMU propagation + a pose ("LIO") update.
// ---------------------------------------------------------------------------------------------
class Host {
 public:
  Host(const Scene& sc, const SimParams& p, const Config& cfg) : sc_(sc), p_(p), cfg_(cfg) {}

  void init(std::mt19937_64& rng) {
    const Truth& tr = sc_.truth();
    const Mat3 M0 = sc_.regM(0);
    // truth in W0 (the IMU frame at start: R = I, p = 0)
    CoreState t;
    t.R = M0.transpose() * tr.R[0];
    t.p = M0.transpose() * (tr.p[0] - sc_.regT(0));
    t.v = M0.transpose() * tr.v[0];
    t.bg = sc_.bgTrue(0);
    t.ba = sc_.baTrue(0);
    t.g = sc_.gravityHost(0);
    x_.P = HostMat::Zero();
    const double s_rot = 1e-3, s_pos = 1e-2, s_vel = 2e-2, s_g = 5e-3;
    HostVec sd = HostVec::Zero();
    for (int i = 0; i < 3; ++i) {
      sd(HostIdx::rot + i) = s_rot;
      sd(HostIdx::pos + i) = s_pos;
      sd(HostIdx::vel + i) = s_vel;
      sd(HostIdx::bg + i) = sc_.bg0Sigma();
      sd(HostIdx::ba + i) = sc_.ba0Sigma();
      sd(HostIdx::grav + i) = s_g;
    }
    sd(HostIdx::expo) = 1e-3;
    HostVec e = HostVec::Zero();
    for (int i = 0; i < HostIdx::dim; ++i) {
      x_.P(i, i) = sd(i) * sd(i);
      if (i == HostIdx::bg || i == HostIdx::bg + 1 || i == HostIdx::bg + 2 || i == HostIdx::ba || i == HostIdx::ba + 1 ||
          i == HostIdx::ba + 2)
        continue;   // the bias truth is already a draw from the same prior; the estimate starts at 0
      e(i) = sd(i) * gauss(rng);
    }
    CoreState est = t;
    est.bg.setZero();
    est.ba.setZero();
    est.inv_expo = 1.0;
    est.P = x_.P;
    // x̂ = truth ⊟ e  ⇒  truth = x̂ ⊞ e (right perturbation on R, additive elsewhere)
    est.R = t.R * so3Exp(-e.segment<3>(HostIdx::rot));
    est.p = t.p - e.segment<3>(HostIdx::pos);
    est.v = t.v - e.segment<3>(HostIdx::vel);
    est.g = t.g - e.segment<3>(HostIdx::grav);
    if (p_.level_bias.squaredNorm() > 0.0) {
      const Vec3 w(p_.level_bias(0), p_.level_bias(1), 0.0);
      est.g = so3Exp(w) * est.g;
    }
    if (p_.turnon_ba.squaredNorm() > 0.0) {
      est.ba.setZero();
      est.g = t.g - t.R * p_.turnon_ba;
      if (p_.host_turnon_prior) applyBmi088TurnonPrior(&est.P, est.g);
    }
    x_ = est;
    acc_s_last_ = x_.R * (sc_.imuF()[0] - x_.ba) + x_.g;
    angvel_last_ = sc_.imuW()[0] - x_.bg;
  }

  // IMU propagation over cycle k: (t_{k−1}, t_k] on the IMU grid; bridge samples and F_tot as IMU_Processing.cpp.
  void propagate(int k, double t_host_prev, double t_host_k, std::vector<BridgeSample>* bridge, HostMat* F_tot, double* ds,
                 double* dalpha) {
    const double dt = p_.imu_dt;
    const int per = static_cast<int>(std::lround(p_.lio_dt / dt));
    bridge->clear();
    BridgeSample s0;
    s0.t = t_host_prev;
    s0.a_world = acc_s_last_;
    s0.omega = angvel_last_;
    s0.R = x_.R;
    s0.p = x_.p;
    s0.v = x_.v;
    s0.f_body = x_.R.transpose() * (acc_s_last_ - x_.g);
    bridge->push_back(s0);
    F_tot->setIdentity();
    const std::vector<Vec3>& F = sc_.imuF();
    const std::vector<Vec3>& W = sc_.imuW();
    const double qg = p_.gyr_n * p_.gyr_n * dt, qa = p_.acc_n * p_.acc_n * dt;
    const double qbg = p_.gyr_w * p_.gyr_w * dt, qba = p_.acc_w * p_.acc_w * dt;
    const Vec3 up = -x_.g.normalized();
    *ds = 0.0;
    *dalpha = 0.0;
    for (int j = 1; j <= per; ++j) {
      const int i = (k - 1) * per + j;
      const Vec3 w_avr = 0.5 * (W[i - 1] + W[i]) - x_.bg;
      const Vec3 f_avr = 0.5 * (F[i - 1] + F[i]) - x_.ba;
      HostMat Fx = HostMat::Identity();
      Fx.block<3, 3>(HostIdx::rot, HostIdx::rot) = so3Exp(-w_avr * dt);
      Fx.block<3, 3>(HostIdx::rot, HostIdx::bg) = -Mat3::Identity() * dt;
      Fx.block<3, 3>(HostIdx::pos, HostIdx::vel) = Mat3::Identity() * dt;
      Fx.block<3, 3>(HostIdx::vel, HostIdx::rot) = -x_.R * skew(f_avr) * dt;
      Fx.block<3, 3>(HostIdx::vel, HostIdx::ba) = -x_.R * dt;
      Fx.block<3, 3>(HostIdx::vel, HostIdx::grav) = Mat3::Identity() * dt;
      HostMat Q = HostMat::Zero();
      for (int a = 0; a < 3; ++a) {
        Q(HostIdx::rot + a, HostIdx::rot + a) = qg;
        Q(HostIdx::vel + a, HostIdx::vel + a) = qa;   // R diag(q) Rᵀ = q I (isotropic)
        Q(HostIdx::bg + a, HostIdx::bg + a) = qbg;
        Q(HostIdx::ba + a, HostIdx::ba + a) = qba;
      }
      x_.P = Fx * x_.P * Fx.transpose() + Q;
      *F_tot = Fx * (*F_tot);
      const Mat3 R_prev = x_.R;
      const Vec3 p_prev = x_.p;
      x_.R = x_.R * so3Exp(w_avr * dt);
      const Vec3 a = x_.R * f_avr + x_.g;
      x_.p = x_.p + x_.v * dt + 0.5 * a * dt * dt;
      x_.v = x_.v + a * dt;
      angvel_last_ = w_avr;
      acc_s_last_ = a;
      BridgeSample s;
      s.t = t_host_prev + j * dt;
      s.a_world = a;
      s.omega = w_avr;
      s.R = x_.R;
      s.p = x_.p;
      s.v = x_.v;
      s.f_body = f_avr;
      bridge->push_back(s);
      *ds += (x_.p - p_prev).norm();
      *dalpha += std::fabs((R_prev * w_avr).dot(up)) * dt;
    }
    bridge->back().t = t_host_k;   // the cycle end the hook is called with, to the bit (times are ~1.7e9 s)
  }

  // Process noise of the map-frame jump W_{k−1} → W_k (DESIGN §3.2 model, exact linear map onto the host error).
  void addDriftNoise(double ds, double dalpha) {
    const Config::Process::Frame& q = cfg_.process.frame;
    const Vec3 up = -x_.g.normalized();
    const Mat3 P_v = up * up.transpose(), P_h = Mat3::Identity() - P_v;
    const Mat3 Sr = q.q_tilt_per_m * ds * P_h + (q.q_yaw_per_m * ds + q.q_yaw_per_rad * dalpha) * P_v;
    const Mat3 Sw = q.q_th_per_m * ds * P_h + q.q_tv_per_m * ds * P_v;
    Eigen::Matrix<double, HostIdx::dim, 6> G = Eigen::Matrix<double, HostIdx::dim, 6>::Zero();
    const Vec3 m = x_.p + x_.R * p_.lever_I;
    G.block<3, 3>(HostIdx::rot, 0) = -x_.R.transpose();
    G.block<3, 3>(HostIdx::pos, 0) = skew(x_.p - m);
    G.block<3, 3>(HostIdx::pos, 3) = -Mat3::Identity();
    G.block<3, 3>(HostIdx::vel, 0) = skew(x_.v);
    G.block<3, 3>(HostIdx::grav, 0) = skew(x_.g);
    Eigen::Matrix<double, 6, 6> S = Eigen::Matrix<double, 6, 6>::Zero();
    S.block<3, 3>(0, 0) = Sr;
    S.block<3, 3>(3, 3) = Sw;
    x_.P += G * S * G.transpose();
    x_.P = 0.5 * (x_.P + x_.P.transpose());
  }

  // "LIO": pose measurement in the current map frame, rows [δθ | δp] (right perturbation), (I − KH)P as upstream.
  void lioUpdate(const Mat3& zR, const Vec3& zp) {
    Eigen::Matrix<double, 6, HostIdx::dim> H = Eigen::Matrix<double, 6, HostIdx::dim>::Zero();
    H.block<3, 3>(0, HostIdx::rot).setIdentity();
    H.block<3, 3>(3, HostIdx::pos).setIdentity();
    Eigen::Matrix<double, 6, 6> Rm = Eigen::Matrix<double, 6, 6>::Zero();
    for (int i = 0; i < 3; ++i) {
      Rm(i, i) = p_.lio_rot_sigma * p_.lio_rot_sigma;
      Rm(3 + i, 3 + i) = p_.lio_pos_sigma * p_.lio_pos_sigma;
    }
    const CoreState prior = x_;
    CoreState xi = x_;
    for (int it = 0; it < 2; ++it) {   // iterated (the rotation residual is not linear); converges in one step
      Eigen::Matrix<double, 6, 1> r;
      r.segment<3>(0) = so3Log(Mat3(xi.R.transpose() * zR));
      r.segment<3>(3) = zp - xi.p;
      const HostVec dx0 = xi.boxminus(prior);
      const Eigen::Matrix<double, 6, 6> S = H * prior.P * H.transpose() + Rm;
      const Eigen::Matrix<double, HostIdx::dim, 6> K = prior.P * H.transpose() * S.ldlt().solve(Eigen::Matrix<double, 6, 6>::Identity());
      const HostVec dx = K * (r + H * dx0);
      xi = prior.boxplus(dx);
      if (it == 1) {
        const HostMat IKH = HostMat::Identity() - K * H;
        HostMat P = IKH * prior.P;
        xi.P = 0.5 * (P + P.transpose());
      }
    }
    x_ = xi;
  }

  CoreState& x() { return x_; }

 private:
  const Scene& sc_;
  const SimParams& p_;
  const Config& cfg_;
  CoreState x_;
  Vec3 acc_s_last_ = Vec3::Zero(), angvel_last_ = Vec3::Zero();
};

// Slot index of a named augmented-block slot (augmentedState(): one name per slot, components consecutive).
int slotDim(const std::string& n) {
  if (n == "clk" || n == "tilt") return 2;
  if (n == "trans" || n.compare(0, 6, "clone_") == 0) return 3;
  return 1;   // yaw, isb_*, td
}
int slotIndex(const std::vector<std::string>& names, const std::string& want) {
  int o = 0;
  for (const std::string& n : names) {
    if (n == want) return o;
    o += slotDim(n);
  }
  return -1;
}

double neesOf(const Eigen::VectorXd& e, const Eigen::MatrixXd& C) {
  Eigen::LDLT<Eigen::MatrixXd> ldlt(0.5 * (C + C.transpose()));
  if (ldlt.info() != Eigen::Success || !(ldlt.vectorD().array() > 0.0).all()) return kNaN;
  return e.dot(ldlt.solve(e));
}

}  // namespace

// =============================================================================================
// public
// =============================================================================================
const char* scenarioName(Scenario s) {
  switch (s) {
    case Scenario::OpenSky: return "open_sky";
    case Scenario::UrbanMultipath: return "urban_multipath";
    case Scenario::Outage120s: return "outage_120s";
    case Scenario::ClockJumps: return "clock_jumps";
    case Scenario::CycleSlips: return "cycle_slips";
    case Scenario::FourSystemsIsb: return "four_systems_isb";
    case Scenario::FewSats: return "few_sats";
    case Scenario::LongStill: return "long_still";
    case Scenario::RevisitSnap: return "revisit_snap";
    case Scenario::YawDriftMismatch: return "yaw_drift_mismatch";
    case Scenario::MapGhosting: return "map_ghosting";
    case Scenario::OpenSkyRealistic: return "open_sky_realistic";
    case Scenario::LowElevDopplerBias: return "low_el_doppler_bias";
    default: return "?";
  }
}

bool scenarioFromName(const std::string& name, Scenario* s) {
  for (int i = 0; i < static_cast<int>(Scenario::Count); ++i) {
    if (name == scenarioName(static_cast<Scenario>(i))) {
      if (s) *s = static_cast<Scenario>(i);
      return true;
    }
  }
  return false;
}

Group scenarioGroup(Scenario s) {
  switch (s) {
    case Scenario::UrbanMultipath:
    case Scenario::OpenSkyRealistic: return Group::Realistic;
    case Scenario::RevisitSnap:
    case Scenario::YawDriftMismatch:
    case Scenario::MapGhosting:
    case Scenario::LowElevDopplerBias: return Group::Mismatch;
    default: return Group::ModelMatched;
  }
}

Config simConfig(const SimParams& p) {
  Config c;
  c.enable = true;
  c.update_enable = true;
  c.mode = Config::Mode::Tight;
  c.update_slot = Config::UpdateSlot::PreLio;
  c.source = Config::SourceKind::GnssCommRos1;
  c.nav_rinex = {p.nav_rinex};
  c.ephem.ura_semantics = Config::Ephem::UraSemantics::Metres;
  c.systems.glo = true;
  c.frames.lever_arm_frame = Config::Frames::LeverArmFrame::Imu;
  for (int i = 0; i < 3; ++i) c.frames.lever_arm_xyz[i] = p.lever_I(i);
  c.imu_bridge.acc_n = p.acc_n;
  c.imu_bridge.gyr_n = p.gyr_n;
  // AMENDMENTS A58 (frozen M3DGR calibration): masks, noise shape, clock PSD, frame q.
  c.masks.cn0_psr = 20.0;
  c.masks.cn0_dopp = 12.0;
  c.masks.cn0_cp = 25.0;
  c.masks.max_psr_std_m = 41.0;
  c.masks.max_dopp_std_hz = 16.4;
  c.noise.psr.sigma0_m = 0.773;
  c.noise.psr.k_std = 0.0;
  c.noise.dopp.sigma0_mps = 0.127;
  c.process.clock.S_b = 1.0e-3;
  c.process.clock.S_d = 1.43e-2;
  c.process.frame.q_tilt_per_m = 1.44e-7;
  c.process.frame.q_yaw_per_m = 2.78e-8;
  c.process.frame.q_yaw_per_rad = 0.0;
  c.process.frame.q_th_per_m = 4.58e-5;
  c.process.frame.q_tv_per_m = 8.57e-4;
  // the rest of A58's noise tree (config/m3dgr_avia.yaml)
  c.noise.psr.tau_s = 33.1;
  c.noise.dopp.tau_s = 0.047;
  c.noise.scale.psr = 0.0178;
  c.noise.scale.dopp = 0.2118;
  c.noise.scale.tdcp = 0.882;
  c.noise.tdcp.sigma0_m = 0.004;
  c.noise.tdcp.sigma_rate_mps = 0.002;
  c.noise.tdcp.sigma_windup_m = 0.002;
  c.frames.lever_arm_sigma = 0.03;
  c.time.sigma_toff_s = 0.05;
  if (p.scenario == Scenario::OpenSkyRealistic) {
    // the frozen A58 tree as it is (its errors are what the scene draws)
  } else if (scenarioGroup(p.scenario) == Group::Realistic) {
    // κ_AR carries the multipath time correlation; σ0 matches the simulated marginal code variance at the zenith
    // (0.3² + 1.5² m² over the elevation factor 2): the design's treatment of correlated errors.
    c.noise.psr.sigma0_m = std::sqrt((0.09 + 2.25) / 2.0);
    c.noise.psr.tau_s = 30.0;
    c.noise.dopp.sigma0_mps = std::sqrt(0.05 * 0.05 + 0.03 * 0.03);
    c.noise.dopp.tau_s = 1.0;
    c.noise.scale.psr = c.noise.scale.dopp = c.noise.scale.tdcp = 1.0;
    c.frames.lever_arm_sigma = 0.0;
    c.time.sigma_toff_s = 1e-3;
  } else {
    // Model-matched (A60): every term the scene does not produce physically is drawn by the simulator with exactly
    // this model's variance; the terms it cannot draw are switched off: κ_AR = 1 (τ → 0) with A58's κ folded into the
    // receiver σ0 (κ multiplies only the receiver term, so every term keeps the calibrated tree's per-row weight — A61;
    // folding κ into the scale instead inflated the ephemeris/atmosphere terms 66-fold), σ_toff → 0, TDCP rate /
    // wind-up → 0 (the carrier's atmospheric change is modelled since A61 F2), no lever-arm uncertainty (the simulated
    // lever arm is the configured one).
    c.noise.psr.tau_s = 1e-3;
    c.noise.dopp.tau_s = 1e-3;
    c.noise.psr.sigma0_m = 0.773 * std::sqrt(NoiseModel::kappaAR(33.1, 1.0));      // A58 σ0·√κ_ρ(33.1 s, 1 s)
    c.noise.dopp.sigma0_mps = 0.127 * std::sqrt(NoiseModel::kappaAR(0.047, 0.1));  // A58 σ_D0·√κ_D(0.047 s, 0.1 s)
    c.noise.tdcp.sigma_rate_mps = 1e-9;
    c.noise.tdcp.sigma_windup_m = 0.0;
    c.frames.lever_arm_sigma = 0.0;
    c.time.sigma_toff_s = 1e-6;
  }
  c.ablation.tdcp_fixed_anchor = p.fixed_anchor;
  c.ablation.frame_fixed_after_init = p.fixed_anchor;   // validateConfig pairs the two (A-FA)
  if (p.fixed_anchor) c.noise.tdcp.chain_pairs = false;   // A-FA as A60 measured it: DESIGN §4.5's pairing (A61)
  // A64: the map-frame jump detection (A61 F9) is off by default — FAST-LIVO2's corrections do not follow its own
  // P⁻ − P⁺ (A62 ①). The scenes whose map jumps by construction keep it on, as A61 measured them (the host stand-in is
  // consistent, so the other scenes never trigger it: A61 counted 0 FRAME_JUMP there).
  if (p.scenario == Scenario::RevisitSnap || p.scenario == Scenario::MapGhosting) c.integrity.frame_jump.enable = true;
  return c;
}

std::string RunResult::describe() const {
  std::ostringstream o;
  o << (error.empty() ? "" : "ERROR " + error + "; ") << "activated " << activated << " at " << t_activate << " s of "
    << duration_s << " s, " << nees.size() << " NEES samples, " << n_epochs << " epochs (" << n_sats_mean
    << " sats), NIS psr/dopp/tdcp " << nis[0].meanPerDof() << "/" << nis[1].meanPerDof() << "/" << nis[2].meanPerDof()
    << ", host pose NEES/6 " << host_nees_mean / 6.0 << " (blocks th/p/v/bg/ba/g " << host_block_z2[0] << "/" << host_block_z2[1]
    << "/" << host_block_z2[2] << "/" << host_block_z2[3] << "/" << host_block_z2[4] << "/" << host_block_z2[5] << "); events:";
  for (size_t i = 0; i < events.size(); ++i)
    if (events[i]) o << " " << eventKindName(static_cast<EventKind>(i)) << "=" << events[i];
  return o.str();
}

namespace {

// Days since 1970-01-01 of a proleptic Gregorian date (H. Hinnant's days_from_civil).
int64_t daysFromCivil(int y, int m, int d) {
  y -= m <= 2;
  const int64_t era = (y >= 0 ? y : y - 399) / 400;
  const int64_t yoe = y - era * 400;
  const int64_t doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  const int64_t doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return era * 146097 + doe - 719468;
}

// A copy of the RINEX 3 navigation file `src` with one ephemeris epoch per satellite, valid over the whole scene
// [t0, t0 + 2(t_mid − t0)] (GTime::toSec() scale; the records' own time systems differ from GPST by ≤ 18 s, which does
// not matter here): Galileo the latest epoch before t0 (selectEph() takes a Galileo record only once its toe has passed),
// the other systems the epoch nearest t_mid (validity ±2 h GPS, the ±1 h/±30 min age masks of BDS/GLONASS). Every record
// of that epoch is kept (Galileo's I/NAV and F/NAV share it), the header as it is.
bool writeSingleEphemerisNav(const std::string& src, double t0, double t_mid, const std::string& dst, std::string* why) {
  std::ifstream in(src);
  if (!in) {
    *why = "cannot open " + src;
    return false;
  }
  struct Rec {
    std::string sat;
    double t = 0.0;
    std::vector<std::string> lines;
  };
  std::vector<std::string> header;
  std::vector<Rec> recs;
  std::string line;
  bool in_header = true;
  while (std::getline(in, line)) {
    if (in_header) {
      header.push_back(line);
      if (line.find("END OF HEADER") != std::string::npos) in_header = false;
      continue;
    }
    if (!line.empty() && line[0] != ' ') {   // first line of a record: "G01 2025 01 21 00 00 00 ..."
      int y = 0, mo = 0, d = 0, h = 0, mi = 0, s = 0;
      if (line.size() < 23 || std::sscanf(line.c_str() + 3, "%d %d %d %d %d %d", &y, &mo, &d, &h, &mi, &s) != 6) {
        *why = src + ": unreadable record " + line.substr(0, 23);
        return false;
      }
      Rec r;
      r.sat = line.substr(0, 3);
      r.t = static_cast<double>(daysFromCivil(y, mo, d)) * 86400.0 + h * 3600.0 + mi * 60.0 + s;
      r.lines.push_back(line);
      recs.push_back(std::move(r));
    } else if (!recs.empty()) {
      recs.back().lines.push_back(line);
    }
  }
  const double version = header.empty() ? 0.0 : std::atof(header.front().substr(0, 9).c_str());
  if (in_header || !(version >= 3.0 && version < 4.0)) {
    *why = src + ": not a RINEX 3 navigation file";
    return false;
  }
  std::map<std::string, double> chosen;
  for (const Rec& r : recs) {
    const bool gal = r.sat[0] == 'E';
    if (gal && !(r.t <= t0 - 1.0)) continue;
    const auto it = chosen.find(r.sat);
    const bool better = it == chosen.end() || (gal ? r.t > it->second : std::fabs(r.t - t_mid) < std::fabs(it->second - t_mid));
    if (better) chosen[r.sat] = r.t;
  }
  std::ofstream out(dst);
  for (const std::string& l : header) out << l << '\n';
  for (const Rec& r : recs) {
    const auto it = chosen.find(r.sat);
    if (it != chosen.end() && r.t == it->second)
      for (const std::string& l : r.lines) out << l << '\n';
  }
  out.close();
  if (!out) {
    *why = "cannot write " + dst;
    return false;
  }
  return true;
}

// Middle of the scene on the GTime::toSec() scale (its length follows from the path; the long stop adds 2 min).
double sceneMidSec(const SimParams& p) {
  const double t_move = p.path_m / std::max(p.speed_mid_mps, 0.1);
  return p.t0.toSec() + 30.0 + 0.5 * (t_move + (p.scenario == Scenario::LongStill ? 150.0 : 0.0));
}

// A file of this process that is removed with the object.
struct TempFile {
  std::string path;
  TempFile() {
    static std::atomic<unsigned> counter{0};
    const long long stamp = static_cast<long long>(std::chrono::steady_clock::now().time_since_epoch().count());
    path = (std::filesystem::temp_directory_path() /
            ("livo_gnss_sim_nav_" + std::to_string(stamp) + "_" + std::to_string(counter++) + ".rnx"))
               .string();
  }
  ~TempFile() { std::remove(path.c_str()); }
  TempFile(const TempFile&) = delete;
  TempFile& operator=(const TempFile&) = delete;
};

// One pass over the scene of `p`: the registration random walk runs from `drift_start_s` (scene time) on;
// `stop_at_activation` returns at the cycle the filter activates (first pass of drift_from_activation).
RunResult runPass(const SimParams& p, const Config& cfg, double drift_start_s, bool stop_at_activation) {
  RunResult res;
  res.drift_start_s = drift_start_s;
  Scene sc(p, cfg, drift_start_s);
  std::string why;
  try {
    if (!sc.build(&why)) {
      res.no_fixture = true;
      res.error = why;
      return res;
    }
  } catch (const std::exception& e) {
    res.error = std::string("scene: ") + e.what();
    return res;
  }
  res.duration_s = sc.duration();
  res.t_mismatch = sc.tMismatch();
  res.n_sats_mean = sc.meanSats();
  res.dov = sc.dov();
  res.toff_true_s = sc.toff();
  res.model_psr_max_abs_m = scenarioGroup(p.scenario) == Group::Realistic ? kNaN : sc.modelPsrMaxAbs();
  RunSink sink(p.settle_s);
  try {
    GnssFusion fusion(cfg);
    fusion.setDiagnosticsSink(&sink);
    fusion.setExtrinsics(Mat3::Identity(), Vec3::Zero());
    fusion.setSlamMode(SlamMode::ONLY_LIO);
    fusion.setHostFlags(/*gravity_est_en=*/true, /*exposure_est_en=*/false);
    Host host(sc, p, cfg);
    host.init(sc.rng());
    const double th0 = sc.hostT0();
    const int eval_every = std::max(1, static_cast<int>(std::lround(p.eval_dt_s / p.lio_dt)));
    size_t next_epoch = 0;
    std::vector<BridgeSample> bridge;
    HostMat F_tot;
    double host_nees_sum = 0.0;
    long host_nees_n = 0;
    std::array<double, 6> host_block_sum{};
    std::array<long, 6> host_block_n{};
    for (int k = 1; k <= sc.cycles(); ++k) {
      const double t_prev = th0 + (k - 1) * p.lio_dt, t_k = th0 + k * p.lio_dt;
      const double tau_k = k * p.lio_dt;
      double ds = 0.0, dalpha = 0.0;
      host.propagate(k, t_prev, t_k, &bridge, &F_tot, &ds, &dalpha);
      while (next_epoch < sc.epochs().size() && sc.epochs()[next_epoch].arrival <= t_k + p.processing_lag_s) {
        ObsEpoch e = sc.epochs()[next_epoch].epoch;
        fusion.pushObservation(std::move(e));
        ++next_epoch;
      }
      {
        CoreState c = host.x();
        const StepReport r = fusion.onLioStep(c, F_tot, bridge, t_prev, t_k);
        if (r.core_modified) host.x() = c;
      }
      // drift seen by the LIO: the map frame moves under the host between its GNSS and LIO updates — the host learns it
      // through matching process noise and the LIO update (drift in ξ: nothing to do, the LIO never sees it)
      if (p.drift_in_host) host.addDriftNoise(ds, dalpha);
      const CoreState prior = host.x();
      const Scene::HostState hk = sc.hostCoords(k, tau_k);
      const Mat3& M = sc.regM(k);
      const Vec3& Tk = sc.regT(k);
      const Mat3 R_W = hk.R;
      const Vec3 p_W = hk.p;
      const Vec3 nr(gauss(sc.rng()), gauss(sc.rng()), gauss(sc.rng())), np(gauss(sc.rng()), gauss(sc.rng()), gauss(sc.rng()));
      host.lioUpdate(R_W * so3Exp(p.lio_rot_sigma * nr), p_W + sc.ghost(k) + p.lio_pos_sigma * np);
      const CoreState post = host.x();
      fusion.onCoreUpdated(prior, post, UpdateSource::LIO);
      fusion.onStepEnd(post, t_k);
      // synthetic-host self-check (outside group C's ghost windows)
      if (tau_k > 10.0 && sc.ghost(k).isZero()) {
        Eigen::Matrix<double, 6, 1> e;
        e.segment<3>(0) = so3Log(Mat3(post.R.transpose() * R_W));
        e.segment<3>(3) = p_W - post.p;
        const double hn = neesOf(e, post.P.block<6, 6>(0, 0));
        if (std::isfinite(hn)) {
          host_nees_sum += hn;
          ++host_nees_n;
        }
        const int ii = std::min(static_cast<int>(sc.imuF().size()) - 1, static_cast<int>(std::lround(tau_k / p.imu_dt)));
        const Vec3 g_true = sc.gravityHost(k);
        const Vec3 eb[6] = {e.segment<3>(0), e.segment<3>(3), hk.v - post.v, sc.bgTrue(ii) - post.bg, sc.baTrue(ii) - post.ba,
                            g_true - post.g};
        res.host_g_err_deg = std::atan2(g_true.cross(post.g).norm(), g_true.dot(post.g)) * 180.0 / 3.14159265358979323846;
        const double gz = neesOf(g_true - post.g, post.P.block<3, 3>(HostIdx::grav, HostIdx::grav));
        if (std::isfinite(gz)) res.host_g_z2_end = gz / 3.0;
        const int off[6] = {HostIdx::rot, HostIdx::pos, HostIdx::vel, HostIdx::bg, HostIdx::ba, HostIdx::grav};
        for (int b = 0; b < 6; ++b) {
          const double z = neesOf(eb[b], post.P.block<3, 3>(off[b], off[b]));
          if (std::isfinite(z)) {
            host_block_sum[static_cast<size_t>(b)] += z / 3.0;
            ++host_block_n[static_cast<size_t>(b)];
          }
        }
      }
      const FusionStatus st = fusion.status();
      if (st.active && !res.activated) {
        res.activated = true;
        res.t_activate = std::isfinite(sink.tAct()) ? sink.tAct() - th0 : tau_k;
        if (stop_at_activation) break;
      }
      if (!res.activated || k % eval_every != 0 || tau_k < res.t_activate + p.settle_s || !st.active) continue;
      NeesSample smp;
      smp.t = tau_k;
      smp.mismatch_window = sc.inMismatch(tau_k);
      smp.host_deceived = sc.hostDeceived(tau_k);
      AnchorFrame anc;
      if (!fusion.anchor(&anc)) continue;
      // global antenna position
      {
        Vec3 aN;
        Mat3 cov;
        if (fusion.antennaN(post, &aN, &cov)) {
          Vec3 a_true;
          sc.antennaWorld(k, hk, &a_true, nullptr, nullptr);
          const Vec3 a_true_E = sc.r0() + sc.R_EN() * a_true;
          const Vec3 a_true_N = anc.R_EN.transpose() * (a_true_E - anc.r0_ecef);
          smp.ant_err_N = aN - a_true_N;
          smp.nees_ant = neesOf(smp.ant_err_N, cov);
          RunResult::TrajSample ts;
          ts.t = tau_k;
          ts.truth = a_true;
          ts.host_W = post.p + post.R * p.lever_I;
          ts.est_N = aN;
          if (!fusion.antennaOdom(post, &ts.est_odo)) ts.est_odo = aN;
          res.traj.push_back(ts);
          for (int i = 0; i < 3; ++i) {
            smp.z2_ant[static_cast<size_t>(i)] = smp.ant_err_N(i) * smp.ant_err_N(i) / cov(i, i);
            smp.zs_ant[static_cast<size_t>(i)] = smp.ant_err_N(i) / std::sqrt(cov(i, i));
            smp.sigma_ant(i) = std::sqrt(cov(i, i));
          }
        }
      }
      // registration ξ = (φ, ψ, t) in the filter's own parameterisation (DESIGN §1), truth from T_{N*←W_k}
      {
        Registration xi;
        if (fusion.registration(&xi) && xi.active) {
          const Mat3 R_NNs = anc.R_EN.transpose() * sc.R_EN();
          const Mat3 M_N = R_NNs * M;
          const Vec3 tau_N = R_NNs * Tk + anc.R_EN.transpose() * (sc.r0() - anc.r0_ecef);
          const Vec3 t_star = tau_N + M_N * xi.pivot_w;
          const Mat3 C_ref = Leveling::expTilt(xi.tilt).transpose() * Rz(xi.yaw).transpose() * xi.M;
          // The levelling of the truth: the host's gravity estimate, the reference of the filter's own φ (DESIGN §1,
          // M = R_z(ψ)·Exp(φ̃)·C(ĝ)) — since A61; before, the true gravity. The two agree to the host's gravity error,
          // which a consistent host keeps far below σ_φ; after a map-frame tilt (the revisit snap) the stand-in host's ĝ
          // lags the new gravity for minutes with its covariance unchanged (host g-block NEES 50–2400), and against the
          // true gravity ξ inherited that host inconsistency while the antenna, on the joint covariance, stayed consistent.
          const Vec3 g_W = post.g;
          const Mat3 C_true = Leveling::minimalRotation((C_ref * g_W.normalized()).normalized(), Vec3(0.0, 0.0, -1.0)) * C_ref;
          const Mat3 Q = M_N * C_true.transpose();             // = R_z(ψ*) Exp([φ*, 0])
          const Vec3 u = Q.transpose() * Vec3::UnitZ();
          const Mat3 S = Leveling::minimalRotation(Vec3::UnitZ(), u).transpose();
          const Vec3 phi = so3Log(S);
          const Mat3 Rzs = Q * S.transpose();
          const double psi_star = std::atan2(Rzs(1, 0), Rzs(0, 0));
          Eigen::Matrix<double, 6, 1> e;
          e(0) = phi(0) - xi.tilt(0);
          e(1) = phi(1) - xi.tilt(1);
          e(2) = wrapPi(psi_star - xi.yaw);
          e.segment<3>(3) = t_star - xi.trans;
          smp.nees_xi = neesOf(e, xi.cov);
          for (int i = 0; i < 6; ++i) {
            smp.z2_xi[static_cast<size_t>(i)] = e(i) * e(i) / xi.cov(i, i);
            smp.zs_xi[static_cast<size_t>(i)] = -e(i) / std::sqrt(xi.cov(i, i));
          }
          smp.yaw_err = -e(2);
          smp.yaw_sigma = std::sqrt(std::max(0.0, xi.cov(2, 2)));
          smp.tilt = xi.tilt;
          smp.tilt_sigma(0) = std::sqrt(std::max(0.0, xi.cov(0, 0)));
          smp.tilt_sigma(1) = std::sqrt(std::max(0.0, xi.cov(1, 1)));
        }
      }
      // clock and ISB slots
      {
        Eigen::VectorXd xg, dq;
        std::vector<std::string> names;
        fusion.augmentedState(&xg, &dq, &names);
        const int oc = slotIndex(names, "clk");
        if (oc >= 0) {
          const double eb = xg(oc) - sc.bAt(tau_k), ed = xg(oc + 1) - sc.dAt(tau_k);
          smp.nees_b = dq(oc) > 0.0 ? eb * eb / dq(oc) : kNaN;
          smp.nees_d = dq(oc + 1) > 0.0 ? ed * ed / dq(oc + 1) : kNaN;
          smp.zs_b = dq(oc) > 0.0 ? eb / std::sqrt(dq(oc)) : kNaN;
          smp.zs_d = dq(oc + 1) > 0.0 ? ed / std::sqrt(dq(oc + 1)) : kNaN;
          smp.err_b = eb;
          smp.sigma_b = dq(oc) > 0.0 ? std::sqrt(dq(oc)) : kNaN;
        }
        double ni = 0.0;
        int nn = 0;
        for (char s : {'E', 'C'}) {
          const int oi = slotIndex(names, std::string("isb_") + s);
          if (oi < 0 || !(dq(oi) > 0.0)) continue;
          const double ei = xg(oi) - sc.isbAt(s, tau_k);
          if (s == 'E') smp.zs_isbE = ei / std::sqrt(dq(oi));
          ni += ei * ei / dq(oi);
          ++nn;
        }
        if (nn > 0) {
          smp.nees_isb = ni;
          smp.n_isb = nn;
        }
      }
      res.nees.push_back(smp);
    }
    fusion.finish();
    res.summary = sink.summary;
    res.nis = sink.nis;
    res.events = sink.events;
    res.n_epochs = sink.epochs;
    res.activate_detail = sink.activate_detail;
    if (res.traj.size() >= 3) {
      std::vector<Vec3> tr, hw, en, eo;
      for (const RunResult::TrajSample& s : res.traj) {
        tr.push_back(s.truth);
        hw.push_back(s.host_W);
        en.push_back(s.est_N);
        eo.push_back(s.est_odo);
      }
      res.ate_u = ateSe3(hw, tr);
      res.ate_t = ateSe3(en, tr);
      res.ate_o = ateSe3(eo, tr);
    }
    res.rows = sink.rows;
    res.slip_flags = sink.slip_flags;
    res.event_log = sink.event_log;
    res.host_nees_mean = host_nees_n > 0 ? host_nees_sum / static_cast<double>(host_nees_n) : kNaN;
    for (size_t b = 0; b < 6; ++b)
      res.host_block_z2[b] = host_block_n[b] > 0 ? host_block_sum[b] / static_cast<double>(host_block_n[b]) : kNaN;
  } catch (const std::exception& e) {
    res.error = e.what();
  }
  return res;
}

}  // namespace

RunResult runScenario(const SimParams& p_in, const Config& cfg_in) {
  struct SignGuard {
    explicit SignGuard(double s) { g_noise_sign = s < 0.0 ? -1.0 : 1.0; }
    ~SignGuard() { g_noise_sign = 1.0; }
  } sign_guard(p_in.noise_sign);
  SimParams p = p_in;
  Config cfg = cfg_in;
  std::unique_ptr<TempFile> nav;
  if (p.single_ephemeris && !p.nav_rinex.empty()) {
    nav.reset(new TempFile());
    std::string why;
    if (!writeSingleEphemerisNav(p.nav_rinex, p.t0.toSec(), sceneMidSec(p), nav->path, &why)) {
      RunResult r;
      r.error = "single_ephemeris: " + why;
      return r;
    }
    for (std::string& f : cfg.nav_rinex)
      if (f == p.nav_rinex) f = nav->path;
    p.nav_rinex = nav->path;
  }
  if (!p.drift_from_activation) return runPass(p, cfg, p.drift_start_s, false);
  const double never = std::numeric_limits<double>::infinity();
  RunResult first = runPass(p, cfg, never, true);
  if (first.no_fixture || !first.error.empty() || !first.activated) return first;   // never activated: a full drift-free run
  RunResult r = runPass(p, cfg, first.t_activate, false);
  if (r.error.empty() && !(r.activated && std::fabs(r.t_activate - first.t_activate) < 1e-6)) {
    std::ostringstream os;
    os << "drift_from_activation: second pass activated at " << r.t_activate << " s, first pass at " << first.t_activate << " s";
    r.error = os.str();
  }
  return r;
}

double ateSe3(const std::vector<Eigen::Vector3d>& est, const std::vector<Eigen::Vector3d>& ref) {
  const size_t n = std::min(est.size(), ref.size());
  if (n < 3) return kNaN;
  Vec3 me = Vec3::Zero(), mr = Vec3::Zero();
  for (size_t i = 0; i < n; ++i) {
    me += est[i];
    mr += ref[i];
  }
  me /= static_cast<double>(n);
  mr /= static_cast<double>(n);
  Mat3 H = Mat3::Zero();
  for (size_t i = 0; i < n; ++i) H += (est[i] - me) * (ref[i] - mr).transpose();
  Eigen::JacobiSVD<Mat3> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Mat3 D = Mat3::Identity();
  if ((svd.matrixV() * svd.matrixU().transpose()).determinant() < 0.0) D(2, 2) = -1.0;
  const Mat3 R = svd.matrixV() * D * svd.matrixU().transpose();
  const Vec3 t = mr - R * me;
  double sq = 0.0;
  for (size_t i = 0; i < n; ++i) sq += (R * est[i] + t - ref[i]).squaredNorm();
  return std::sqrt(sq / static_cast<double>(n));
}

// ---------------------------------------------------------------------------------------------
// χ²
// ---------------------------------------------------------------------------------------------
namespace {
double gammaincLower(double a, double x) {
  if (x <= 0.0) return 0.0;
  const double lg = std::lgamma(a);
  if (x < a + 1.0) {
    double term = 1.0 / a, s = term, ap = a;
    for (int i = 0; i < 100000; ++i) {
      ap += 1.0;
      term *= x / ap;
      s += term;
      if (std::fabs(term) < std::fabs(s) * 1e-16) break;
    }
    return std::min(1.0, s * std::exp(-x + a * std::log(x) - lg));
  }
  const double tiny = 1e-300;
  double b = x + 1.0 - a, c = 1.0 / tiny, d = 1.0 / b, h = d;
  for (int i = 1; i < 100000; ++i) {
    const double an = -i * (i - a);
    b += 2.0;
    d = an * d + b;
    if (std::fabs(d) < tiny) d = tiny;
    c = b + an / c;
    if (std::fabs(c) < tiny) c = tiny;
    d = 1.0 / d;
    const double dl = d * c;
    h *= dl;
    if (std::fabs(dl - 1.0) < 1e-16) break;
  }
  return std::max(0.0, 1.0 - std::exp(-x + a * std::log(x) - lg) * h);
}
}  // namespace

double chi2Cdf(double x, double k) { return gammaincLower(0.5 * k, 0.5 * x); }

double chi2Quantile(double p, double k) {
  double lo = 0.0, hi = std::max(10.0, k + 20.0 * std::sqrt(2.0 * k));
  while (chi2Cdf(hi, k) < p) hi *= 2.0;
  for (int i = 0; i < 300; ++i) {
    const double mid = 0.5 * (lo + hi);
    if (chi2Cdf(mid, k) < p) lo = mid;
    else hi = mid;
    if (hi - lo <= 1e-12 * std::max(1.0, hi)) break;
  }
  return 0.5 * (lo + hi);
}

Band chi2MeanBand(int n, int dof, double alpha) {
  Band b;
  const double k = static_cast<double>(n) * dof;
  b.lo = chi2Quantile(0.5 * alpha, k) / n;
  b.hi = chi2Quantile(1.0 - 0.5 * alpha, k) / n;
  return b;
}

// ---------------------------------------------------------------------------------------------
// aggregation
// ---------------------------------------------------------------------------------------------
namespace {
NeesStat neesStat(const std::vector<RunResult>& runs, int dof, double NeesSample::*field, bool exclude_mismatch, int min_runs) {
  NeesStat s;
  s.dof = dof;
  std::map<long, std::pair<double, int>> at;   // time bin (ms) → (Σ NEES, runs)
  for (const RunResult& r : runs) {
    if (!r.error.empty()) continue;
    for (const NeesSample& x : r.nees) {
      if (exclude_mismatch && x.mismatch_window) continue;
      const double v = x.*field;
      if (!std::isfinite(v)) continue;
      auto& a = at[std::lround(x.t * 1000.0)];
      a.first += v;
      ++a.second;
    }
  }
  std::vector<double> anees;
  int inside = 0;
  std::map<int, Band> bands;
  for (const auto& kv : at) {
    const int n = kv.second.second;
    if (n < min_runs) continue;
    const double m = kv.second.first / n;
    anees.push_back(m / dof);
    auto it = bands.find(n);
    if (it == bands.end()) it = bands.emplace(n, chi2MeanBand(n, dof)).first;
    if (m >= it->second.lo && m <= it->second.hi) ++inside;
  }
  s.n_times = static_cast<int>(anees.size());
  if (anees.empty()) return s;
  double sum = 0.0;
  for (double v : anees) sum += v;
  s.anees_per_dof = sum / anees.size();
  s.in_band_frac = static_cast<double>(inside) / anees.size();
  const size_t third = std::max<size_t>(1, anees.size() / 3);
  double a = 0.0, b = 0.0;
  for (size_t i = 0; i < third; ++i) a += anees[i];
  for (size_t i = anees.size() - third; i < anees.size(); ++i) b += anees[i];
  s.first_third = a / third;
  s.last_third = b / third;
  return s;
}
}  // namespace

ConsistencyStats aggregate(const std::vector<RunResult>& runs, bool exclude_mismatch, int min_runs) {
  ConsistencyStats c;
  c.runs = static_cast<int>(runs.size());
  for (const RunResult& r : runs) {
    if (!r.error.empty()) {
      ++c.failed;
      continue;
    }
    if (r.activated) ++c.activated;
    for (int k = 0; k < 3; ++k) {
      c.nis[static_cast<size_t>(k)].nis += r.nis[static_cast<size_t>(k)].nis;
      c.nis[static_cast<size_t>(k)].dof += r.nis[static_cast<size_t>(k)].dof;
      c.nis[static_cast<size_t>(k)].epochs += r.nis[static_cast<size_t>(k)].epochs;
      c.nis[static_cast<size_t>(k)].in_band += r.nis[static_cast<size_t>(k)].in_band;
    }
  }
  c.xi = neesStat(runs, 6, &NeesSample::nees_xi, exclude_mismatch, min_runs);
  c.ant = neesStat(runs, 3, &NeesSample::nees_ant, exclude_mismatch, min_runs);
  c.b = neesStat(runs, 1, &NeesSample::nees_b, exclude_mismatch, min_runs);
  c.d = neesStat(runs, 1, &NeesSample::nees_d, exclude_mismatch, min_runs);
  return c;
}

bool ConsistencyStats::withinBounds(double nees_lo, double nees_hi, double min_in_band, double nis_lo, double nis_hi,
                                    double nis_min_in_band, std::string* why) const {
  std::ostringstream o;
  bool ok = failed == 0 && activated == runs && runs > 0;
  if (!ok) o << "runs " << runs << ", activated " << activated << ", failed " << failed << "; ";
  const std::pair<const char*, const NeesStat*> kinds[] = {{"xi", &xi}, {"ant", &ant}, {"clk_b", &b}, {"clk_d", &d}};
  for (const auto& k : kinds) {
    const NeesStat& s = *k.second;
    if (s.n_times == 0 || !(s.anees_per_dof >= nees_lo && s.anees_per_dof <= nees_hi) || !(s.in_band_frac >= min_in_band)) {
      ok = false;
      o << k.first << " ANEES/dof " << s.anees_per_dof << " in-band " << s.in_band_frac << " (" << s.n_times << " times); ";
    }
  }
  const char* cn[3] = {"psr", "dopp", "tdcp"};
  for (int k = 0; k < 3; ++k) {
    const ClassNis& n = nis[static_cast<size_t>(k)];
    if (n.dof == 0) continue;
    if (!(n.meanPerDof() >= nis_lo && n.meanPerDof() <= nis_hi) || !(n.inBandFrac() >= nis_min_in_band)) {
      ok = false;
      o << cn[k] << " NIS/dof " << n.meanPerDof() << " in-band " << n.inBandFrac() << "; ";
    }
  }
  if (why) *why = o.str();
  return ok;
}

std::string ConsistencyStats::describe() const {
  std::ostringstream o;
  o.precision(4);
  o << "runs " << runs << " (activated " << activated << ", failed " << failed << ")";
  const std::pair<const char*, const NeesStat*> kinds[] = {{"xi", &xi}, {"ant", &ant}, {"clk_b", &b}, {"clk_d", &d}};
  for (const auto& k : kinds)
    o << "; " << k.first << " ANEES/dof " << k.second->anees_per_dof << " (first/last third " << k.second->first_third << "/"
      << k.second->last_third << ", in band " << k.second->in_band_frac << " of " << k.second->n_times << ")";
  const char* cn[3] = {"psr", "dopp", "tdcp"};
  for (int k = 0; k < 3; ++k)
    o << "; " << cn[k] << " NIS/dof " << nis[static_cast<size_t>(k)].meanPerDof() << " (in band "
      << nis[static_cast<size_t>(k)].inBandFrac() << ", dof " << nis[static_cast<size_t>(k)].dof << ")";
  return o.str();
}

}  // namespace sim
}  // namespace livo_gnss
