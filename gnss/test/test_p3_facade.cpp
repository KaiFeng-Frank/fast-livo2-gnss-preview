// SPDX-License-Identifier: BSD-3-Clause
// test_p3_facade.cpp — GnssFusion facade (P3-A2, OWNERSHIP §2c): the DESIGN §9 schedule end to end.
//
// Scene: a synthetic host (FAST-LIVO2 stand-in: 19-dim state propagated through IMU bridge samples, corrected every
// cycle by a mm-level LIO pose/velocity measurement taken in a map frame W whose translation random-walks by path length
// — the "drifting map" of DESIGN §0-2) and a synthetic receiver on real broadcast geometry: the dev-set fixture's
// same-day IGS BRDC (Grass02, 2025-01-21), the fixture's site, GPS + Galileo + BeiDou code/Doppler/carrier and GLONASS
// Doppler/carrier, a drifting receiver clock, ISBs, Klobuchar and Saastamoinen delays.
//
//   EnableFalseTouchesNothing        gnss.enable = false: every entry point is a no-op; the host trajectory is bit-identical
//                                    to the host-only run, the sink hears nothing, RTKLIB's global Galileo selection is
//                                    untouched (the zero channel, DESIGN §13)
//   RawBytesRouteThroughTheDecoder   feedBytes → RawDecoder → the facade's queue; MSGDEF_REJECT; duplicate epochs refused
//   InvalidConfigThrows              the constructor refuses a configuration validateConfig() rejects
//   ActivatesTracksAndPairsTdcp      WAIT_CORE → WAIT_NAV → COLLECT → ACTIVATE → TRACKING; commits; global antenna/attitude
//                                    error; TDCP pairs non-overlapping with clones added at k0 and removed at k1, Doppler
//                                    left out at both ends; ISB slots for every enabled code system (A35); summary
//   ObserveModeHostBitIdentical      update_enable = false: the host trajectory is bit-identical to the host-only run while
//                                    the GNSS side still activates and computes its updates (bypass gate G-旁路)
//   NoGnssDataHostBitIdentical       enable = true without any GNSS message: host bit-identical, block empty (G-无数据)
//   OneHzRuleFirstEpochOfEachSecond  AMENDMENTS A43 with receiver epochs at +5 ms / −7 ms / 0 ms phase
//   LateEpochsBridgedOrDropped       a stalled stream: δ ≤ max_bridge_s bridged across cycles, δ > max_bridge_s LATE_EPOCH
//   PostVioSlotConsumesSameEpochs    AMENDMENTS A12: post_vio consumes the same epochs as pre_lio
//   OutageParksPairsAndRecovers      §11: a 2.5 s gap ⇒ OUTAGE, the open pair's clone is parked, RECOVERY, pairs resume
//   IntegerMsClockJumpDetectedAndBridgedByTdcp  §10 L2: a +1 ms receiver clock step inside a TDCP pair
//   LoosePvtArm                      §4.6 L-PVT: raw-observation initialisation, then 1 Hz receiver PVT rows only
//   AblationArmsDropExactlyTheirClass  T−pr / T−dop / T−tdcp (A46)
//
// The cases that run the state machine call P3-A1's initializer/outage/integrity; while those are the skeleton stubs
// (std::logic_error "not implemented: …") the case reports `blocked: P3-A1` instead of passing. The first three cases
// never reach P3-A1 code and run in any tree. Thresholds on accuracy are deliberately loose: they check the facade's
// orchestration (activation, commits, frames), not P3-A1's alignment quality or the noise model's calibration.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <map>
#include <random>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "livo_gnss/gnss_fusion.hpp"
#include "internal/ephem_store.hpp"
#include "internal/rtk_adapter.hpp"
#include "../../gnss_io/test/fixture_discovery.hpp"   // WP-e's one discovery rule (A24)

using namespace livo_gnss;

namespace {

constexpr double kC = 299792458.0;
constexpr double kOmgE = 7.2921151467e-5;
constexpr double kDeg = M_PI / 180.0;
constexpr double kG = 9.81;
// Site and day of the dev-set fixture Grass02 (receiver_pvt, first record) — any site/day covered by the BRDC would do.
constexpr double kSiteLatDeg = 34.0339415, kSiteLonDeg = 108.7614865, kSiteH = 388.05;
constexpr int64_t kFixtureT0 = 1421486476 + 315964800;   // fixture t0 (1980 origin in the name) on the GTime::toSec() scale

std::string grass02Brdc() {
  for (const std::string& f : livo_gnss_io::testsupport::realFixtures(LIVO_GNSS_TEST_DATA_DIR)) {
    if (livo_gnss_io::testsupport::baseName(f).compare(0, 8, "Grass02_") != 0) continue;
    return livo_gnss_io::testsupport::fixtureBrdc(f);
  }
  return std::string();
}

bool isStub(const std::exception& e) { return std::string(e.what()).rfind("not implemented", 0) == 0; }

// gtest 1.10 does not print a skip message: the reason goes to stdout as well so the ctest log shows it.
#define P3_SKIP(msg)                                                  \
  do {                                                                \
    const std::string p3_skip_msg_ = (msg);                           \
    std::printf("[ BLOCKED  ] %s\n", p3_skip_msg_.c_str());           \
    GTEST_SKIP() << p3_skip_msg_;                                     \
  } while (0)

// Rotation over τ at constant body rate ω in the host's form (utils/so3_math.h Exp(angvel, dt); bridge.cpp expRate).
Eigen::Matrix3d expRate(const Eigen::Vector3d& w, double tau) {
  const double n = w.norm();
  if (n > 1e-7) {
    const Eigen::Matrix3d K = skew(w / n);
    const double a = n * tau;
    return Eigen::Matrix3d::Identity() + std::sin(a) * K + (1.0 - std::cos(a)) * K * K;
  }
  return Eigen::Matrix3d::Identity();
}

std::vector<double> hostBytes(const CoreState& c) {
  std::vector<double> v;
  auto push = [&](const double* p, int n) { v.insert(v.end(), p, p + n); };
  push(c.R.data(), 9);
  push(c.p.data(), 3);
  push(c.v.data(), 3);
  push(c.bg.data(), 3);
  push(c.ba.data(), 3);
  push(c.g.data(), 3);
  push(&c.inv_expo, 1);
  push(c.P.data(), HostIdx::dim * HostIdx::dim);
  return v;
}

// Bitwise host equality (memcmp: -0.0 and NaN payloads count as differences).
bool sameHost(const CoreState& a, const CoreState& b) {
  const std::vector<double> x = hostBytes(a), y = hostBytes(b);
  return x.size() == y.size() && std::memcmp(x.data(), y.data(), x.size() * sizeof(double)) == 0;
}

class Collect final : public DiagnosticsSink {
 public:
  void onEpoch(const EpochRecord& r) override { epochs.push_back(r); }
  void onSat(const SatRecord&) override { ++sats; }
  void onCov(const CovRecord& c) override { covs.push_back(c); }
  void onEvent(const Event& e) override { events.push_back(e); }
  void onSummary(const Summary& s) override {
    summary = s;
    ++summaries;
  }
  int count(EventKind k) const {
    int n = 0;
    for (const Event& e : events) n += e.kind == k ? 1 : 0;
    return n;
  }
  bool anyDetail(EventKind k, const std::string& needle) const {
    for (const Event& e : events)
      if (e.kind == k && e.detail.find(needle) != std::string::npos) return true;
    return false;
  }
  std::vector<EpochRecord> epochs;
  std::vector<CovRecord> covs;
  std::vector<Event> events;
  Summary summary;
  int summaries = 0, sats = 0;
};

// ---------------------------------------------------------------------------------------------
// Truth: IMU on a circle in the true ENU frame N* at the site, integrated with piecewise-constant acceleration and body
// rate over dt (so the truth is exactly what the bridge's constant-a/ω interpolation reproduces).
// ---------------------------------------------------------------------------------------------
struct Truth {
  double dt = 0.01;
  Eigen::Vector3d w_I = Eigen::Vector3d::Zero();
  std::vector<Eigen::Matrix3d> R;   // R_{N*←I}(k dt)
  std::vector<Eigen::Vector3d> p, v;
  std::vector<Eigen::Vector3d> a;   // acceleration of interval (k−1, k] in N*; a[0] unused

  void build(double duration, double speed, double radius, const Eigen::Matrix3d& R_mount) {
    const double om = speed / radius;
    w_I = R_mount.transpose() * Eigen::Vector3d(0.0, 0.0, om);
    const int n = static_cast<int>(std::ceil(duration / dt)) + 2;
    R.assign(n, Eigen::Matrix3d::Identity());
    p.assign(n, Eigen::Vector3d::Zero());
    v.assign(n, Eigen::Vector3d::Zero());
    a.assign(n, Eigen::Vector3d::Zero());
    R[0] = R_mount;
    v[0] = Eigen::Vector3d(speed, 0.0, 0.0);
    for (int k = 1; k < n; ++k) {
      const double tm = (k - 0.5) * dt;
      a[k] = radius * om * om * Eigen::Vector3d(-std::sin(om * tm), std::cos(om * tm), 0.0);
      R[k] = R[k - 1] * expRate(w_I, dt);
      p[k] = p[k - 1] + v[k - 1] * dt + 0.5 * a[k] * dt * dt;
      v[k] = v[k - 1] + a[k] * dt;
    }
  }
  // State at relative time τ ∈ [0, (n−1)dt]: the interval (k−1, k] containing τ, constant a[k] and w_I.
  void at(double tau, Eigen::Matrix3d* Ro, Eigen::Vector3d* po, Eigen::Vector3d* vo, Eigen::Vector3d* ao = nullptr) const {
    int k = static_cast<int>(std::ceil(tau / dt - 1e-9));
    k = std::max(1, std::min(k, static_cast<int>(p.size()) - 1));
    const double s = tau - (k - 1) * dt;
    *Ro = R[k - 1] * expRate(w_I, s);
    *po = p[k - 1] + v[k - 1] * s + 0.5 * a[k] * s * s;
    *vo = v[k - 1] + a[k] * s;
    if (ao) *ao = a[k];
  }
};

// ---------------------------------------------------------------------------------------------
// Scenario options and results
// ---------------------------------------------------------------------------------------------
struct Stall {
  double from = 0.0, to = 0.0, latency = 0.0;   // relative host time window of the receiver epochs, their latency
};

struct Options {
  double duration_s = 55.0;
  double lio_dt = 0.1, imu_dt = 0.01, gnss_dt = 0.1;
  double epoch_phase_s = 0.0;   // receiver epochs at k·0.1 s + phase (A43 cases)
  double speed = 1.5, radius = 12.0;
  uint64_t seed = 7;
  bool call_fusion = true;      // false: the host-only reference run
  double latency_s = 0.04, processing_lag_s = 0.05;
  std::vector<Stall> stalls;
  std::vector<std::pair<double, double>> gaps;   // relative time windows without any receiver epoch (outage)
  double clock_jump_at_s = -1.0;                 // receiver clock steps by +1 ms here (u-blox style), < 0: never
  bool push_epochs = true;                       // false: GNSS enabled but no GNSS message ever arrives (G-无数据)
  bool enable = true, update_enable = true;
  Config::Mode mode = Config::Mode::Tight;
  bool use_psr = true, use_dopp = true, use_tdcp = true;   // T−pr / T−dop / T−tdcp arms
  bool chain_pairs = true;                                 // noise.tdcp.chain_pairs (AMENDMENTS A61 F10)
  Config::UpdateSlot slot = Config::UpdateSlot::PreLio;
};

struct Result {
  bool blocked = false;
  std::string blocked_why;
  bool no_fixture = false;
  Collect sink;
  std::vector<std::vector<double>> host_bytes;   // host state after every cycle
  FusionStatus status;
  Summary summary;
  // accuracy at the end
  bool have_global = false;
  Eigen::Vector3d ant_err_enu = Eigen::Vector3d::Zero();
  Eigen::Matrix3d ant_cov = Eigen::Matrix3d::Zero();
  double att_err_deg = 0.0;
  // slots
  std::set<std::string> slot_names;
  int max_clone_slots = 0, max_phase_slots = 0;   // A61 F10/F11: slots of each kind at a cycle end, maximum
  int clone_mismatch = 0;       // cycles where status().tdcp_pairs_open disagrees with a clone slot being present
  int report_mismatch = 0;      // onLioStep/onPostVio calls whose StepReport::core_modified disagrees with the host copy
  int host_writes = 0;          // calls that rewrote the host (applied, like the glue, only on core_modified)
  std::vector<std::pair<double, int>> pair_open_by_cycle;
  int gal_sel_before = -1, gal_sel_after = -1;
  double t_host0 = 0.0;           // host time of the scene start (relative time τ = t − t_host0)
  std::string brdc;
};

// ---------------------------------------------------------------------------------------------
// The world: truth, host, receiver, and the driving loop.
// ---------------------------------------------------------------------------------------------
class World {
 public:
  explicit World(const Options& o) : o_(o), rng_(o.seed) {}

  Result run() {
    Result res;
    const std::string brdc = grass02Brdc();
    if (brdc.empty()) {
      res.no_fixture = true;
      return res;
    }
    // --- receiver-side ephemerides (the generator's own copy) ---
    cfg_gen_.enable = true;
    cfg_gen_.ephem.ura_semantics = Config::Ephem::UraSemantics::Metres;
    res.gal_sel_before = static_cast<int>(rtk::galileoNavSelection());
    nav_.reset(new rtk::NavStore());
    std::string why;
    if (!o_.enable) {
      // The zero channel must not touch RTKLIB's global state: load the NAV file without an EphemStore (which would
      // set the Galileo selection) and restore the selection after the generator's own queries.
      if (nav_->loadRinexNav(brdc, true, &why) <= 0) throw std::runtime_error(why);
    } else {
      ephem_.reset(new EphemStore(cfg_gen_, *nav_));
      if (ephem_->loadRinex(brdc, &why) <= 0) throw std::runtime_error(why);
      ephem_->iono(&ion_);
    }
    solver_.reset(new rtk::SatSolver(*nav_));
    // --- scene ---
    site_geo_.lat_rad = kSiteLatDeg * kDeg;
    site_geo_.lon_rad = kSiteLonDeg * kDeg;
    site_geo_.h_m = kSiteH;
    r0_ = rtk::pos2ecef(site_geo_);
    R_EN_ = rtk::enuRotation(site_geo_);
    T0_.sec = kFixtureT0 + 1000;   // inside the BRDC crop (toc ∈ [t0 − 2 h, t0 + 3 h]) for every system
    T0_.frac = 0.0;
    t_host0_ = static_cast<double>(T0_.sec) - 18.0;   // time.base = host: t_est = t_gpst − leap (18 s) + offset (0)
    res.t_host0 = t_host0_;
    res.brdc = brdc;
    lever_ = Eigen::Vector3d(-0.316, 0.109, 0.401);
    const Eigen::Matrix3d R_mount = so3Exp(Eigen::Vector3d(0.02, -0.015, 0.0));
    truth_.dt = o_.imu_dt;
    truth_.build(o_.duration_s + 1.0, o_.speed, o_.radius, R_mount);
    M_W_ = truth_.R[0];   // W = the IMU frame at start-up (FAST-LIVO2)
    g_W_ = M_W_.transpose() * Eigen::Vector3d(0.0, 0.0, -kG);
    pickSatellites();
    generateEpochs();

    // --- fusion ---
    Config cfg;
    cfg.enable = o_.enable;
    cfg.update_enable = o_.update_enable;
    cfg.update_slot = o_.slot;
    cfg.source = Config::SourceKind::GnssCommRos1;
    cfg.ephem.ura_semantics = Config::Ephem::UraSemantics::Metres;
    cfg.nav_rinex = {brdc};
    cfg.frames.lever_arm_frame = Config::Frames::LeverArmFrame::Imu;
    for (int i = 0; i < 3; ++i) cfg.frames.lever_arm_xyz[i] = lever_(i);
    cfg.systems.glo = true;
    cfg.mode = o_.mode;
    cfg.use.psr = o_.use_psr;
    cfg.use.dopp = o_.use_dopp;
    cfg.use.tdcp = o_.use_tdcp;
    cfg.noise.tdcp.chain_pairs = o_.chain_pairs;
    std::unique_ptr<GnssFusion> fusion;
    if (o_.call_fusion) {
      fusion.reset(new GnssFusion(cfg));
      fusion->setDiagnosticsSink(&res.sink);
      fusion->setExtrinsics(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
      fusion->setSlamMode(SlamMode::ONLY_LIO);
      fusion->setHostFlags(/*gravity_est_en=*/true, /*exposure_est_en=*/false);
    }

    // --- host initial state (W = IMU frame at start: identity pose) ---
    CoreState x;
    x.R = Eigen::Matrix3d::Identity();
    x.p = Eigen::Vector3d::Zero();
    x.v = M_W_.transpose() * truth_.v[0];
    x.g = g_W_;
    x.inv_expo = 1.0;
    x.P = HostMat::Zero();
    const double s_rot = 1e-4, s_pos = 1e-3, s_vel = 1e-2;
    for (int i = 0; i < 3; ++i) {
      x.P(HostIdx::rot + i, HostIdx::rot + i) = s_rot * s_rot;
      x.P(HostIdx::pos + i, HostIdx::pos + i) = s_pos * s_pos;
      x.P(HostIdx::vel + i, HostIdx::vel + i) = s_vel * s_vel;
      x.P(HostIdx::bg + i, HostIdx::bg + i) = 1e-8;
      x.P(HostIdx::ba + i, HostIdx::ba + i) = 1e-6;
      x.P(HostIdx::grav + i, HostIdx::grav + i) = 1e-6;
    }
    x.P(HostIdx::expo, HostIdx::expo) = 1e-6;
    Eigen::Vector3d drift = Eigen::Vector3d::Zero();
    std::normal_distribution<double> nrm(0.0, 1.0);
    size_t next_epoch = 0;
    const int steps = static_cast<int>(std::floor(o_.duration_s / o_.lio_dt + 1e-9));
    const int per = static_cast<int>(std::lround(o_.lio_dt / o_.imu_dt));
    double t_prev = t_host0_;
    try {
      for (int k = 1; k <= steps; ++k) {
        const double t_k = t_host0_ + k * o_.lio_dt;
        // bridge samples: the host posterior propagated through the true increments of the cycle
        std::vector<BridgeSample> smp;
        smp.reserve(per + 1);
        {
          BridgeSample s0;
          s0.t = t_prev;
          s0.R = x.R;
          s0.p = x.p;
          s0.v = x.v;
          s0.omega = truth_.w_I;
          const int kt = (k - 1) * per + 1;
          s0.a_world = M_W_.transpose() * truth_.a[kt];
          s0.f_body = x.R.transpose() * (s0.a_world - g_W_);
          smp.push_back(s0);
          for (int i = 1; i <= per; ++i) {
            const BridgeSample& q = smp.back();
            const Eigen::Vector3d aW = M_W_.transpose() * truth_.a[(k - 1) * per + i];
            BridgeSample s;
            s.t = (i == per) ? t_k : t_prev + i * o_.imu_dt;
            const double dt = s.t - q.t;
            s.omega = truth_.w_I;
            s.a_world = aW;
            s.R = q.R * expRate(truth_.w_I, dt);
            s.p = q.p + q.v * dt + 0.5 * aW * dt * dt;
            s.v = q.v + aW * dt;
            s.f_body = s.R.transpose() * (aW - g_W_);
            smp.push_back(s);
          }
        }
        CoreState prior = x;
        prior.R = smp.back().R;
        prior.p = smp.back().p;
        prior.v = smp.back().v;
        for (int i = 0; i < 3; ++i) {
          prior.P(HostIdx::rot + i, HostIdx::rot + i) += 1e-8;
          prior.P(HostIdx::pos + i, HostIdx::pos + i) += 1e-6;
          prior.P(HostIdx::vel + i, HostIdx::vel + i) += 1e-4;
        }
        // receiver epochs that have arrived by the time the host processes this cycle
        while (next_epoch < epochs_.size() && epochs_[next_epoch].arrival <= t_k + o_.processing_lag_s) {
          if (fusion && o_.push_epochs) {
            ObsEpoch e = epochs_[next_epoch].epoch;
            fusion->pushObservation(std::move(e));
            fusion->pushPvt(epochs_[next_epoch].pvt);   // the receiver's own solution (a measurement only in L-PVT)
          }
          ++next_epoch;
        }
        // Like the glue (P3-B, INTERFACE_CHANGES #5 ③): the host takes the facade's copy only when core_modified says so.
        if (fusion) {
          CoreState c = prior;
          const StepReport r = fusion->onLioStep(c, HostMat::Identity(), smp, t_prev, t_k);
          applyReport(r, c, &prior, &res);
        }
        // LIO: pose + velocity in the drifting map (translation random walk by path length, q = 1e-4 m²/m)
        const double tau = t_k - t_host0_;
        Eigen::Matrix3d R_N;
        Eigen::Vector3d p_N, v_N;
        truth_.at(tau, &R_N, &p_N, &v_N);
        const double ds = o_.speed * o_.lio_dt;
        for (int i = 0; i < 3; ++i) drift(i) += std::sqrt(1e-4 * ds) * nrm(rng_);
        const Eigen::Matrix3d R_Wt = M_W_.transpose() * R_N;
        const Eigen::Vector3d p_Wt = M_W_.transpose() * (p_N - truth_.p[0]) + drift;
        const Eigen::Vector3d v_Wt = M_W_.transpose() * v_N;
        const double sR = 2e-4, sP = 2e-3, sV = 5e-3;
        const Eigen::Matrix3d zR = R_Wt * so3Exp(Eigen::Vector3d(sR * nrm(rng_), sR * nrm(rng_), sR * nrm(rng_)));
        const Eigen::Vector3d zp = p_Wt + sP * Eigen::Vector3d(nrm(rng_), nrm(rng_), nrm(rng_));
        const Eigen::Vector3d zv = v_Wt + sV * Eigen::Vector3d(nrm(rng_), nrm(rng_), nrm(rng_));
        CoreState post = prior;
        lioUpdate(zR, zp, zv, sR, sP, sV, &post);
        if (fusion) fusion->onCoreUpdated(prior, post, UpdateSource::LIO);
        if (fusion && o_.slot == Config::UpdateSlot::PostVio) {
          CoreState c = post;
          const StepReport r = fusion->onPostVio(c, t_k);
          applyReport(r, c, &post, &res);
        }
        if (fusion) fusion->onStepEnd(post, t_k);
        x = post;
        res.host_bytes.push_back(hostBytes(x));
        if (fusion && o_.enable) {
          Eigen::VectorXd xg, dq;
          std::vector<std::string> names;
          fusion->augmentedState(&xg, &dq, &names);
          bool clone = false;
          int n_clone = 0, n_phase = 0;
          for (const std::string& n : names) {
            const bool is_clone = n.compare(0, 6, "clone_") == 0, is_phase = n.compare(0, 6, "phase_") == 0;
            res.slot_names.insert(is_clone ? std::string("clone") : is_phase ? std::string("phase") : n);
            clone = clone || is_clone;
            n_clone += is_clone ? 1 : 0;
            n_phase += is_phase ? 1 : 0;
          }
          res.max_clone_slots = std::max(res.max_clone_slots, n_clone);
          res.max_phase_slots = std::max(res.max_phase_slots, n_phase);
          const FusionStatus st = fusion->status();
          if ((st.tdcp_pairs_open > 0) != clone) ++res.clone_mismatch;
          res.pair_open_by_cycle.emplace_back(tau, st.tdcp_pairs_open);
        }
        t_prev = t_k;
      }
      if (fusion) {
        res.status = fusion->status();
        res.summary = fusion->summary();
        fusion->finish();
        fusion->finish();   // idempotent
        // global accuracy at the end
        Eigen::Vector3d aN;
        Eigen::Matrix3d cov;
        AnchorFrame anc;
        Eigen::Matrix3d R_NI;
        Eigen::Vector3d t_NI;
        if (fusion->antennaN(x, &aN, &cov) && fusion->anchor(&anc) && fusion->globalPose(x, &R_NI, &t_NI, nullptr)) {
          res.have_global = true;
          const double tau = t_prev - t_host0_;
          Eigen::Matrix3d R_N;
          Eigen::Vector3d p_N, v_N;
          truth_.at(tau, &R_N, &p_N, &v_N);
          const Eigen::Vector3d a_true_E = r0_ + R_EN_ * (p_N + R_N * lever_);
          const Eigen::Vector3d a_est_E = anc.r0_ecef + anc.R_EN * aN;
          res.ant_err_enu = R_EN_.transpose() * (a_est_E - a_true_E);
          res.ant_cov = cov;
          // attitude: R_{N←I} estimated vs the truth expressed in the filter's N (the two ENU frames differ by ~1e-7 rad)
          const Eigen::Matrix3d R_true_N = anc.R_EN.transpose() * R_EN_ * R_N;
          res.att_err_deg = so3Log(Eigen::Matrix3d(R_true_N.transpose() * R_NI)).norm() / kDeg;
        }
      }
    } catch (const std::logic_error& e) {
      if (!isStub(e)) throw;
      res.blocked = true;
      res.blocked_why = e.what();
    }
    res.gal_sel_after = static_cast<int>(rtk::galileoNavSelection());
    return res;
  }

 private:
  static void applyReport(const StepReport& r, const CoreState& facade_copy, CoreState* host, Result* res) {
    if (r.core_modified != !sameHost(facade_copy, *host)) ++res->report_mismatch;
    if (r.core_modified) {
      *host = facade_copy;
      ++res->host_writes;
    }
  }

  struct GenEpoch {
    ObsEpoch epoch;
    ReceiverPvt pvt;
    double arrival = 0.0;   // host time at which the message is available
    double t_est = 0.0;
  };

  // Host LIO update with rows [rot | pos | vel] (right perturbation on R), (I − KH)P as upstream.
  void lioUpdate(const Eigen::Matrix3d& zR, const Eigen::Vector3d& zp, const Eigen::Vector3d& zv, double sR, double sP, double sV,
                 CoreState* x) const {
    Eigen::Matrix<double, 9, HostIdx::dim> H = Eigen::Matrix<double, 9, HostIdx::dim>::Zero();
    H.block<3, 3>(0, HostIdx::rot).setIdentity();
    H.block<3, 3>(3, HostIdx::pos).setIdentity();
    H.block<3, 3>(6, HostIdx::vel).setIdentity();
    Eigen::Matrix<double, 9, 1> nu;
    nu.segment<3>(0) = so3Log(Eigen::Matrix3d(x->R.transpose() * zR));
    nu.segment<3>(3) = zp - x->p;
    nu.segment<3>(6) = zv - x->v;
    Eigen::Matrix<double, 9, 9> Rm = Eigen::Matrix<double, 9, 9>::Zero();
    for (int i = 0; i < 3; ++i) {
      Rm(i, i) = sR * sR;
      Rm(3 + i, 3 + i) = sP * sP;
      Rm(6 + i, 6 + i) = sV * sV;
    }
    const Eigen::Matrix<double, 9, 9> S = H * x->P * H.transpose() + Rm;
    const Eigen::Matrix<double, HostIdx::dim, 9> K = x->P * H.transpose() * S.inverse();
    const HostVec dx = K * nu;
    *x = x->boxplus(dx);
    const HostMat IKH = HostMat::Identity() - K * H;
    HostMat P = IKH * x->P;
    x->P = 0.5 * (P + P.transpose());
  }

  // Antenna truth (ECEF position/velocity) at relative host time τ.
  void antenna(double tau, Eigen::Vector3d* rA, Eigen::Vector3d* vA) const {
    Eigen::Matrix3d R;
    Eigen::Vector3d p, v;
    truth_.at(tau, &R, &p, &v);
    *rA = r0_ + R_EN_ * (p + R * lever_);
    *vA = R_EN_ * (v + R * truth_.w_I.cross(lever_));
  }

  double freqOf(const SatId& s) const {
    if (s.sys == Sys::GLO) return 1602.0e6 + nav_->gloFcn(s.prn) * 0.5625e6;
    return s.sys == Sys::BDS ? 1561.098e6 : 1575.42e6;
  }
  static SignalCode codeOf(Sys s) {
    switch (s) {
      case Sys::GAL: return SignalCode::L1X;
      case Sys::BDS: return SignalCode::L2I;
      default: return SignalCode::L1C;
    }
  }

  void pickSatellites() {
    std::set<int> seen;
    const Eigen::Vector3d rA = r0_;
    for (const rtk::EphKey& k : nav_->coverage()) {
      if (!seen.insert(k.sat.rtk_sat).second) continue;
      const SatId id = k.sat;
      if (id.sys == Sys::GLO && nav_->gloFcn(id.prn) < -7) continue;
      bool ok = true;
      for (double tt : {0.0, o_.duration_s + 1.0}) {
        rtk::SatState st;
        if (!solver_->satState(id, T0_.add(tt), 2.2e7, &st) || !st.ok || !rtk::svhHealthy(id.sys, st.svh)) {
          ok = false;
          break;
        }
        Eigen::Vector3d e;
        rtk::geodist(st.r_ecef, rA, &e);
        double az = 0.0, el = 0.0;
        rtk::satAzel(site_geo_, e, &az, &el);
        if (el < 15.0 * kDeg) ok = false;
      }
      if (ok) sats_.push_back(id);
    }
    std::uniform_int_distribution<int> amb(-2000000, 2000000);
    for (size_t i = 0; i < sats_.size(); ++i) ambiguity_.push_back(amb(rng_));
  }

  void generateEpochs() {
    std::normal_distribution<double> nrm(0.0, 1.0);
    const double b0 = 3000.0, d0 = 45.0;   // receiver clock [m], drift [m/s]
    const int n = static_cast<int>(std::floor(o_.duration_s / o_.gnss_dt));
    double last_arrival = -1e300;
    for (int j = 0; j < n; ++j) {
      // receiver tag t_rx = T0 + j·0.1 + phase (receiver time); the true reception instant is t_rx − b/c
      GTime t_rx = T0_;
      t_rx.sec += j / 10;
      t_rx.frac = (j % 10) / 10.0 + o_.epoch_phase_s;
      if (t_rx.frac < 0.0) {
        t_rx.frac += 1.0;
        t_rx.sec -= 1;
      }
      const double tau_rx = t_rx.diff(T0_);
      const double jump = (o_.clock_jump_at_s >= 0.0 && tau_rx >= o_.clock_jump_at_s) ? kC * 1e-3 : 0.0;
      const double b = b0 + d0 * tau_rx + jump;
      const double tau = tau_rx - b / kC;
      if (tau < 0.05 || tau > o_.duration_s - 0.05) continue;
      bool in_gap = false;
      for (const auto& g : o_.gaps) in_gap = in_gap || (tau >= g.first && tau < g.second);
      if (in_gap) continue;
      Eigen::Vector3d rA, vA;
      antenna(tau, &rA, &vA);
      const Geodetic geo = rtk::ecef2pos(rA);
      GenEpoch ge;
      ge.epoch.t_rx = t_rx;
      ge.epoch.source = Source::GnssCommMeas;
      ge.epoch.seq = static_cast<uint32_t>(j);
      for (size_t s = 0; s < sats_.size(); ++s) {
        const SatId& id = sats_[s];
        const double f = freqOf(id);
        const double lam = kC / f;
        const SignalCode code = codeOf(id.sys);
        rtk::SatState st;
        double P = 2.2e7, rho = 0.0, trop = 0.0, iono = 0.0;
        Eigen::Vector3d e;
        bool ok = true;
        for (int it = 0; it < 4 && ok; ++it) {
          ok = solver_->satState(id, t_rx, P, &st) && st.ok;
          if (!ok) break;
          rho = rtk::geodist(st.r_ecef, rA, &e);
          double az = 0.0, el = 0.0;
          rtk::satAzel(geo, e, &az, &el);
          trop = rtk::tropModel(t_rx, geo, az, el, 0.7);
          iono = ion_.valid() ? rtk::ionModelGpsL1(t_rx, ion_, geo, az, el) * (1575.42e6 / f) * (1575.42e6 / f) : 0.0;
          const double isb = id.sys == Sys::GAL ? 4.0 : id.sys == Sys::BDS ? -6.0 : 0.0;
          const double cb = rtk::codeBiasMetres(id, st, code);
          P = rho + b - kC * st.dts + trop + iono + isb + (std::isfinite(cb) ? cb : 0.0);
        }
        if (!ok) continue;
        const Eigen::Vector3d& rs = st.r_ecef;
        const Eigen::Vector3d& vs = st.v_ecef;
        // range rate with the derivative-sign Sagnac rate (AMENDMENTS A36)
        const double rate = e.dot(vs - vA) + kOmgE / kC * (vs.x() * rA.y() + rs.x() * vA.y() - vs.y() * rA.x() - rs.y() * vA.x());
        SatObs so;
        so.sat = id;
        so.t_rx = t_rx;
        SignalObs sig;
        sig.freq_hz = f;
        sig.code = code;
        sig.psr_m = P + 0.3 * nrm(rng_);
        sig.dopp_hz = -(rate + d0 - kC * st.ddts + 0.02 * nrm(rng_)) / lam;
        sig.cp_cyc = (rho + b - kC * st.dts + trop - iono + 0.002 * nrm(rng_)) / lam + ambiguity_[s];
        sig.cn0_dbhz = 45.0;
        sig.status = kStatusPsrValid | kStatusCpValid | kStatusHalfCycValid;
        so.sig.push_back(sig);
        ge.epoch.sats.push_back(so);
      }
      ge.t_est = t_host0_ + tau;
      {
        // receiver PVT at the true epoch instant: antenna position (0.5 m horizontal, 1 m vertical noise) and velocity
        ReceiverPvt& pv = ge.pvt;
        pv.t = T0_.add(tau);
        pv.fix_type = 3;
        pv.valid_fix = true;
        pv.num_sv = static_cast<int>(ge.epoch.sats.size());
        const Eigen::Vector3d nE = R_EN_ * Eigen::Vector3d(0.5 * nrm(rng_), 0.5 * nrm(rng_), 1.0 * nrm(rng_));
        const Geodetic g = rtk::ecef2pos(rA + nE);
        pv.lat_deg = g.lat_rad / kDeg;
        pv.lon_deg = g.lon_rad / kDeg;
        pv.h_ell_m = g.h_m;
        pv.h_acc_m = 0.5;
        pv.v_acc_m = 1.0;
        const Eigen::Vector3d vN = R_EN_.transpose() * vA + 0.03 * Eigen::Vector3d(nrm(rng_), nrm(rng_), nrm(rng_));
        pv.vel_ned = Eigen::Vector3d(vN(1), vN(0), -vN(2));
        pv.vel_acc_mps = 0.05;
        pv.source = Source::GnssCommPvt;
      }
      double lat = o_.latency_s;
      for (const Stall& st : o_.stalls)
        if (tau >= st.from && tau < st.to) lat = st.latency;
      ge.arrival = std::max(last_arrival, ge.t_est + lat);
      last_arrival = ge.arrival;
      ge.epoch.t_record = ge.arrival;
      ge.pvt.t_record = ge.arrival;
      epochs_.push_back(std::move(ge));
    }
  }

  Options o_;
  std::mt19937_64 rng_;
  Config cfg_gen_;
  std::unique_ptr<rtk::NavStore> nav_;
  std::unique_ptr<EphemStore> ephem_;
  std::unique_ptr<rtk::SatSolver> solver_;
  Klobuchar ion_;
  Geodetic site_geo_;
  Eigen::Vector3d r0_ = Eigen::Vector3d::Zero();
  Eigen::Matrix3d R_EN_ = Eigen::Matrix3d::Identity();
  GTime T0_;
  double t_host0_ = 0.0;
  Eigen::Vector3d lever_ = Eigen::Vector3d::Zero();
  Truth truth_;
  Eigen::Matrix3d M_W_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d g_W_ = Eigen::Vector3d::Zero();
  std::vector<SatId> sats_;
  std::vector<int> ambiguity_;
  std::vector<GenEpoch> epochs_;
};

bool activeRecord(const EpochRecord& r) {
  return r.phase == Phase::TRACKING || r.phase == Phase::OUTAGE || r.phase == Phase::RECOVERY;
}

// What the run did, for a failing assertion: records per phase, COLLECT fixes, satellites after L0, the events.
std::string explain(const Result& r) {
  std::map<std::string, int> per_phase;
  int spp_ok = 0, late = 0;
  double sats = 0.0;
  for (const EpochRecord& e : r.sink.epochs) {
    ++per_phase[phaseName(e.phase)];
    spp_ok += e.spp_valid ? 1 : 0;
    late += e.cls[static_cast<size_t>(MeasClass::Dopp)].rejected[static_cast<size_t>(RejectReason::Late)] > 0 ? 1 : 0;
    sats += e.n_gps + e.n_gal + e.n_bds + e.n_glo;
  }
  std::string s = "records:";
  for (const auto& kv : per_phase) s += " " + kv.first + "=" + std::to_string(kv.second);
  s += "; spp_valid=" + std::to_string(spp_ok) + " late=" + std::to_string(late) +
       " mean sats after L0=" + std::to_string(r.sink.epochs.empty() ? 0.0 : sats / r.sink.epochs.size());
  s += "; events:";
  int shown = 0;
  for (const Event& e : r.sink.events) {
    if (shown++ >= 25) break;
    s += "\n  " + std::string(eventKindName(e.kind)) + " " + e.detail;
  }
  return s;
}

}  // namespace

// ---------------------------------------------------------------------------------------------
// gnss.enable = false: the zero channel (DESIGN §13).
// ---------------------------------------------------------------------------------------------
TEST(P3Facade, EnableFalseTouchesNothing) {
  rtk::setGalileoNavSelection(rtk::GalNavSelection::Any);
  Options ref;
  ref.call_fusion = false;
  ref.enable = false;
  ref.duration_s = 12.0;
  Options off = ref;
  off.call_fusion = true;
  const Result a = World(ref).run();
  if (a.no_fixture) P3_SKIP(std::string("blocked: dev-set fixture Grass02 with brdc/ under ") + LIVO_GNSS_TEST_DATA_DIR);
  const Result b = World(off).run();
  ASSERT_FALSE(b.blocked) << "enable = false must not reach any P3-A1 code: " << b.blocked_why;
  ASSERT_EQ(a.host_bytes.size(), b.host_bytes.size());
  for (size_t k = 0; k < a.host_bytes.size(); ++k)
    ASSERT_EQ(std::memcmp(a.host_bytes[k].data(), b.host_bytes[k].data(), a.host_bytes[k].size() * sizeof(double)), 0)
        << "host state differs after cycle " << k;
  EXPECT_TRUE(b.sink.epochs.empty());
  EXPECT_TRUE(b.sink.events.empty());
  EXPECT_TRUE(b.sink.covs.empty());
  EXPECT_EQ(b.sink.summaries, 0);
  EXPECT_EQ(b.status.phase, Phase::OFF);
  EXPECT_FALSE(b.status.active);
  EXPECT_EQ(b.status.epochs_processed, 0);
  EXPECT_FALSE(b.have_global);
  EXPECT_EQ(b.gal_sel_before, static_cast<int>(rtk::GalNavSelection::Any));
  EXPECT_EQ(b.gal_sel_after, static_cast<int>(rtk::GalNavSelection::Any)) << "enable = false touched RTKLIB's Galileo selection";

  // Direct calls on a disabled facade: no-ops, nothing modified.
  Config cfg;   // enable = false by default
  GnssFusion f(cfg);
  Collect sink;
  f.setDiagnosticsSink(&sink);
  CoreState core;
  core.p = Eigen::Vector3d(1.0, 2.0, 3.0);
  core.P = HostMat::Identity() * 0.25;
  const std::vector<double> before = hostBytes(core);
  std::vector<BridgeSample> smp(2);
  smp[0].t = 10.0;
  smp[1].t = 10.1;
  const StepReport r1 = f.onLioStep(core, HostMat::Identity(), smp, 10.0, 10.1);
  f.onCoreUpdated(core, core, UpdateSource::LIO);
  const StepReport r2 = f.onPostVio(core, 10.1);
  f.onStepEnd(core, 10.1);
  const uint8_t junk[16] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
  f.feedBytes(Source::GnssCommMeas, junk, sizeof junk, 10.0);
  EXPECT_TRUE(f.registerMessageDefinition(Source::GnssCommMeas, "gnss_comm/GnssMeasMsg", "*", ""));
  ObsEpoch ep;
  ep.t_rx.sec = kFixtureT0;
  f.pushObservation(std::move(ep));
  f.pushPvt(ReceiverPvt());
  f.pushIono(Klobuchar());
  f.pushEphemeris(KeplerEph());
  f.pushGloEphemeris(GloEph());
  std::string why;
  EXPECT_EQ(f.loadNavRinex("/nonexistent", &why), 0);
  f.finish();
  EXPECT_EQ(hostBytes(core), before);
  EXPECT_EQ(r1.epochs_processed + r2.epochs_processed, 0);
  EXPECT_FALSE(r1.core_modified || r2.core_modified);
  Eigen::Matrix3d R;
  Eigen::Vector3d t;
  EXPECT_FALSE(f.globalPose(core, &R, &t, nullptr));
  EXPECT_FALSE(f.antennaN(core, &t, &R));
  EXPECT_FALSE(f.globalPoseSmooth(&R, &t));
  Geodetic g;
  EXPECT_FALSE(f.anchor(&g));
  Registration xi;
  EXPECT_FALSE(f.registration(&xi));
  EXPECT_FALSE(f.lastSpp(&t, &R, nullptr));
  Eigen::VectorXd xg, dq;
  std::vector<std::string> names;
  f.augmentedState(&xg, &dq, &names);
  EXPECT_EQ(xg.size(), 0);
  EXPECT_TRUE(names.empty());
  EXPECT_TRUE(sink.events.empty() && sink.epochs.empty() && sink.summaries == 0);
  EXPECT_EQ(rtk::galileoNavSelection(), rtk::GalNavSelection::Any);
  // and the discriminating counterpart: enabling the core does set the A16 selection
  Config on;
  on.enable = true;
  on.ephem.ura_semantics = Config::Ephem::UraSemantics::Metres;
  on.nav_rinex = {b.brdc};
  GnssFusion f_on(on);
  EXPECT_EQ(rtk::galileoNavSelection(), rtk::GalNavSelection::INav);
}

// ---------------------------------------------------------------------------------------------
// Raw bytes (DESIGN §12.2): feedBytes() goes through the registered RawDecoder with the facade itself as the sink;
// message-definition refusals raise MSGDEF_REJECT; a second copy of a queued epoch is refused (§10 L0 去重).
// Needs no state machine, so it runs before P3-A1 lands.
// ---------------------------------------------------------------------------------------------
namespace {
class MockDecoder final : public RawDecoder {
 public:
  bool registerMessageDefinition(Source src, const std::string& datatype, const std::string& md5, const std::string& msg_def,
                                 std::string* why) override {
    (void)src;
    (void)md5;
    (void)msg_def;
    registered.push_back(datatype);
    if (datatype == "bad/Layout") {
      if (why) *why = "flattened layout differs from the expectation table";
      return false;
    }
    return true;
  }
  bool decode(Source src, const uint8_t* data, size_t size, double t_record, MeasurementSink& sink, std::string* why) override {
    (void)data;
    ++calls;
    last_src = src;
    last_size = size;
    last_t = t_record;
    sink_addr = &sink;
    if (size == 0) {
      if (why) *why = "empty buffer";
      return false;
    }
    ObsEpoch e;
    e.t_rx.sec = kFixtureT0;
    e.t_rx.frac = 0.5;
    e.t_record = t_record;
    e.source = Source::GnssCommMeas;
    sink.pushObservation(std::move(e));
    return true;
  }
  int calls = 0;
  Source last_src = Source::Synthetic;
  size_t last_size = 0;
  double last_t = 0.0;
  const MeasurementSink* sink_addr = nullptr;
  std::vector<std::string> registered;
};
}  // namespace

TEST(P3Facade, RawBytesRouteThroughTheDecoder) {
  const std::string brdc = grass02Brdc();
  if (brdc.empty()) P3_SKIP(std::string("blocked: dev-set fixture Grass02 with brdc/ under ") + LIVO_GNSS_TEST_DATA_DIR);
  Config cfg;
  cfg.enable = true;
  cfg.ephem.ura_semantics = Config::Ephem::UraSemantics::Metres;
  cfg.nav_rinex = {brdc};
  GnssFusion f(cfg);
  Collect sink;
  f.setDiagnosticsSink(&sink);
  EXPECT_FALSE(f.registerMessageDefinition(Source::GnssCommMeas, "gnss_comm/GnssMeasMsg", "*", "")) << "no decoder registered";
  EXPECT_EQ(sink.count(EventKind::MSGDEF_REJECT), 1);
  const uint8_t bytes[5] = {1, 2, 3, 4, 5};
  f.feedBytes(Source::GnssCommMeas, bytes, sizeof bytes, 100.0);   // no decoder: ignored, no crash
  MockDecoder dec;
  f.setRawDecoder(&dec);
  EXPECT_TRUE(f.registerMessageDefinition(Source::GnssCommMeas, "gnss_comm/GnssMeasMsg", "*", "def"));
  EXPECT_FALSE(f.registerMessageDefinition(Source::GnssCommEphem, "bad/Layout", "*", "def"));
  EXPECT_EQ(sink.count(EventKind::MSGDEF_REJECT), 2);
  EXPECT_TRUE(sink.anyDetail(EventKind::MSGDEF_REJECT, "expectation table"));
  f.feedBytes(Source::GnssCommMeas, bytes, sizeof bytes, 123.5);
  EXPECT_EQ(dec.calls, 1);
  EXPECT_EQ(dec.last_src, Source::GnssCommMeas);
  EXPECT_EQ(dec.last_size, sizeof bytes);
  EXPECT_EQ(dec.last_t, 123.5);
  EXPECT_EQ(dec.sink_addr, static_cast<const MeasurementSink*>(&f)) << "decoded messages must land in the facade's queue";
  f.feedBytes(Source::GnssCommMeas, bytes, sizeof bytes, 123.6);   // the same epoch again: refused at ingestion
  EXPECT_EQ(sink.count(EventKind::LATE_EPOCH), 1);
  EXPECT_TRUE(sink.anyDetail(EventKind::LATE_EPOCH, "duplicate"));
  f.feedBytes(Source::GnssCommMeas, bytes, 0, 124.0);   // malformed buffer: the decoder refuses, nothing queued
  EXPECT_EQ(dec.calls, 3);
  EXPECT_EQ(sink.count(EventKind::LATE_EPOCH), 1);
  ReceiverPvt pvt;
  pvt.t.sec = kFixtureT0;
  f.pushPvt(pvt);   // tight arm: the receiver solution is not a measurement, never queued
  f.pushPvt(pvt);
  EXPECT_EQ(sink.count(EventKind::LATE_EPOCH), 1);
  EXPECT_EQ(f.status().phase, Phase::WAIT_CORE);
  EXPECT_EQ(f.status().epochs_processed, 0) << "nothing is consumed before a LIO step";
  f.finish();
  f.finish();
  EXPECT_EQ(sink.summaries, 1);
  EXPECT_EQ(sink.summary.n_msgdef_reject, 2);
  EXPECT_EQ(sink.summary.n_late, 1);
}

// The constructor refuses what validateConfig() refuses (header contract).
TEST(P3Facade, InvalidConfigThrows) {
  Config c;
  c.enable = true;   // gnss_comm source with URA semantics unknown: A16 refuses
  EXPECT_THROW(GnssFusion f(c), std::invalid_argument);
  Config d;
  d.enable = true;
  d.ephem.ura_semantics = Config::Ephem::UraSemantics::Metres;
  d.nav_rinex = {"/nonexistent/brdc.rnx"};   // an unreadable NAV file is a configuration error
  EXPECT_THROW(GnssFusion g(d), std::invalid_argument);
}

// ---------------------------------------------------------------------------------------------
// End to end: activation, tracking, TDCP pairing, ISB slots, diagnostics.
// ---------------------------------------------------------------------------------------------
TEST(P3Facade, ActivatesTracksAndPairsTdcp) {
  Options o;
  const Result r = World(o).run();
  if (r.no_fixture) P3_SKIP(std::string("blocked: dev-set fixture Grass02 with brdc/ under ") + LIVO_GNSS_TEST_DATA_DIR);
  if (r.blocked) P3_SKIP("blocked: P3-A1 (" + r.blocked_why + ")");
  const Collect& s = r.sink;
  ASSERT_EQ(s.count(EventKind::ACTIVATE), 1) << "the scenario activates once\n" << explain(r);
  ASSERT_TRUE(r.status.active);
  EXPECT_EQ(r.status.phase, Phase::TRACKING);
  // the phases in order, never going back
  {
    const Phase order[] = {Phase::WAIT_CORE, Phase::WAIT_NAV, Phase::COLLECT, Phase::TRACKING};
    size_t pos = 0;
    for (const EpochRecord& e : s.epochs) {
      size_t q = pos;
      while (q < 4 && order[q] != e.phase) ++q;
      ASSERT_LT(q, 4u) << "unexpected phase " << phaseName(e.phase);
      pos = q;
    }
    EXPECT_EQ(pos, 3u);
  }
  int committed = 0, psr_acc = 0, dopp_acc = 0, tdcp_acc = 0, rolled = 0, k1 = 0, excl_at_k1 = 0, k1_open = 0;
  double last_k1 = -1e300, min_k1_gap = 1e300;
  std::map<int64_t, int> psr_per_second;
  for (const EpochRecord& e : s.epochs) {
    if (!activeRecord(e)) continue;
    committed += e.committed ? 1 : 0;
    rolled += e.rolled_back ? 1 : 0;
    psr_acc += e.cls[static_cast<size_t>(MeasClass::Psr)].accepted;
    dopp_acc += e.cls[static_cast<size_t>(MeasClass::Dopp)].accepted;
    tdcp_acc += e.cls[static_cast<size_t>(MeasClass::Tdcp)].accepted;
    if (e.cls[static_cast<size_t>(MeasClass::Psr)].candidates > 0) ++psr_per_second[static_cast<int64_t>(std::floor(e.t_gpst))];
    if (e.cls[static_cast<size_t>(MeasClass::Tdcp)].candidates > 0) {
      ++k1;
      k1_open += e.n_tdcp_pairs_open;
      excl_at_k1 += e.cls[static_cast<size_t>(MeasClass::Dopp)].rejected[static_cast<size_t>(RejectReason::Exclusive)] > 0 ? 1 : 0;
      if (last_k1 > -1e299) min_k1_gap = std::min(min_k1_gap, e.t_gpst - last_k1);
      last_k1 = e.t_gpst;
    }
    // every active record carries the registration and host columns
    EXPECT_TRUE(std::isfinite(e.yaw) && e.yaw_sigma > 0.0);
    EXPECT_GT(e.host_sigma(HostIdx::pos), 0.0);
  }
  std::printf("[P3] epochs %zu, committed %d, rolled back %d, accepted psr/dopp/tdcp %d/%d/%d, TDCP pair ends %d, min gap %.3f s\n",
              s.epochs.size(), committed, rolled, psr_acc, dopp_acc, tdcp_acc, k1, min_k1_gap);
  EXPECT_GT(committed, 150);
  EXPECT_LE(rolled, committed / 20);
  EXPECT_GT(psr_acc, 100);
  EXPECT_GT(dopp_acc, 1000);
  EXPECT_GT(tdcp_acc, 50);
  // TDCP (§4.5 with AMENDMENTS A61 F10/F11): non-overlapping pairs k0 → k1 = k0 + 0.5 s chained (the next k0 is k1), the
  // closing clone gone at k1 (one clone, one phase-noise slot at most at a cycle end); Doppler out at k1
  EXPECT_GT(k1, 20);
  EXPECT_GE(k1_open, k1 - 2) << "the next pair opens at its predecessor's end epoch (chain_pairs)";
  EXPECT_GE(min_k1_gap, 0.45) << "pairs overlap";
  EXPECT_EQ(r.max_clone_slots, 1) << "a pair's clone must be removed at its end epoch";
  EXPECT_EQ(r.max_phase_slots, 1) << "the closing pair's phase-noise slot must be removed at its end epoch";
  EXPECT_TRUE(r.slot_names.count("phase"));
  EXPECT_EQ(excl_at_k1, k1) << "Doppler rows of the pair's satellites must be left out at k1";
  EXPECT_EQ(r.clone_mismatch, 0) << "status().tdcp_pairs_open disagrees with the clone slots";
  // The glue writes the host back only on StepReport::core_modified: it must say exactly whether the copy was rewritten.
  EXPECT_EQ(r.report_mismatch, 0) << "StepReport::core_modified disagrees with the host copy";
  EXPECT_GT(r.host_writes, 100);
  EXPECT_TRUE(r.slot_names.count("clone"));
  // A35: ISB slots for GAL and BDS (code systems), none for GLONASS (glo_psr = false)
  EXPECT_TRUE(r.slot_names.count("isb_E"));
  EXPECT_TRUE(r.slot_names.count("isb_C"));
  EXPECT_FALSE(r.slot_names.count("isb_R"));
  // A43: at most one code epoch per GPST second
  for (const auto& kv : psr_per_second) EXPECT_EQ(kv.second, 1) << "second " << kv.first;
  // global quality at the end (the filter's own N frame mapped back to ECEF against the true antenna)
  ASSERT_TRUE(r.have_global);
  std::printf("[P3] antenna error ENU %.3f %.3f %.3f m (sigma %.3f %.3f %.3f), attitude %.3f deg\n", r.ant_err_enu(0),
              r.ant_err_enu(1), r.ant_err_enu(2), std::sqrt(r.ant_cov(0, 0)), std::sqrt(r.ant_cov(1, 1)), std::sqrt(r.ant_cov(2, 2)),
              r.att_err_deg);
  EXPECT_LT(r.ant_err_enu.head<2>().norm(), 2.0);
  EXPECT_LT(std::fabs(r.ant_err_enu(2)), 4.0);
  EXPECT_LT(r.att_err_deg, 2.0);
  for (int i = 0; i < 3; ++i) EXPECT_GT(r.ant_cov(i, i), 0.0);
  // diagnostics: summary once, postGNSS covariance rows per commit, NIS in a sane band
  EXPECT_EQ(s.summaries, 1);
  EXPECT_EQ(static_cast<int>(s.covs.size()), committed);
  for (const CovRecord& c : s.covs) EXPECT_EQ(c.stage, "postGNSS");
  EXPECT_EQ(s.summary.epochs_updated, committed);
  EXPECT_EQ(s.summary.n_activate, 1);
  EXPECT_GT(s.summary.tdcp_coverage, 0.3);
  for (MeasClass c : {MeasClass::Psr, MeasClass::Dopp, MeasClass::Tdcp}) {
    const double nis = s.summary.mean_nis_per_dof[static_cast<size_t>(c)];
    std::printf("[P3] mean NIS/dof %s %.3f, acceptance %.3f\n", measClassName(c), nis, s.summary.acceptance_rate[static_cast<size_t>(c)]);
    // The scene's errors are white noise below the design's variance model (no orbit/iono/multipath error is simulated),
    // so only a sane positive range is asserted here; NIS calibration is TEST_PLAN §3 on the development set.
    EXPECT_GT(nis, 0.0);
    EXPECT_LT(nis, 20.0);
  }
  EXPECT_FALSE(s.anyDetail(EventKind::CONFIG_WARNING, "host write"));
}

// noise.tdcp.chain_pairs = false (DESIGN §4.5 as written, AMENDMENTS A61 F10): the next pair from k1 + 1, so no pair is
// open at the end of its predecessor's k1, and no phase-noise state is ever made (F11 needs the shared epoch).
TEST(P3Facade, UnchainedPairsStartAfterTheirPredecessorsEnd) {
  Options o;
  o.chain_pairs = false;
  const Result r = World(o).run();
  if (r.no_fixture) P3_SKIP(std::string("blocked: dev-set fixture Grass02 with brdc/ under ") + LIVO_GNSS_TEST_DATA_DIR);
  if (r.blocked) P3_SKIP("blocked: P3-A1 (" + r.blocked_why + ")");
  int k1 = 0, k1_open = 0;
  double last_k1 = -1e300, min_k1_gap = 1e300;
  for (const EpochRecord& e : r.sink.epochs) {
    if (!activeRecord(e) || e.cls[static_cast<size_t>(MeasClass::Tdcp)].candidates <= 0) continue;
    ++k1;
    k1_open += e.n_tdcp_pairs_open;
    if (last_k1 > -1e299) min_k1_gap = std::min(min_k1_gap, e.t_gpst - last_k1);
    last_k1 = e.t_gpst;
  }
  EXPECT_GT(k1, 20);
  EXPECT_EQ(k1_open, 0) << "a pair's clone must be removed at its end epoch";
  EXPECT_GE(min_k1_gap, 0.55) << "pairs overlap or start at their predecessor's end epoch";
  EXPECT_EQ(r.max_clone_slots, 1);
  EXPECT_FALSE(r.slot_names.count("phase"));
  EXPECT_EQ(r.clone_mismatch, 0);
}

// ---------------------------------------------------------------------------------------------
// OBSERVE (update_enable = false): the host is bit-identical to the host-only run (bypass gate), GNSS still computed.
// ---------------------------------------------------------------------------------------------
TEST(P3Facade, ObserveModeHostBitIdentical) {
  Options ref;
  ref.call_fusion = false;
  ref.duration_s = 40.0;
  Options obs = ref;
  obs.call_fusion = true;
  obs.update_enable = false;
  const Result a = World(ref).run();
  if (a.no_fixture) P3_SKIP(std::string("blocked: dev-set fixture Grass02 with brdc/ under ") + LIVO_GNSS_TEST_DATA_DIR);
  const Result b = World(obs).run();
  if (b.blocked) P3_SKIP("blocked: P3-A1 (" + b.blocked_why + ")");
  ASSERT_EQ(a.host_bytes.size(), b.host_bytes.size());
  for (size_t k = 0; k < a.host_bytes.size(); ++k)
    ASSERT_EQ(std::memcmp(a.host_bytes[k].data(), b.host_bytes[k].data(), a.host_bytes[k].size() * sizeof(double)), 0)
        << "OBSERVE wrote the host at cycle " << k;
  // ... while the GNSS side ran: activation, and updates computed (gates passed) but never committed
  ASSERT_EQ(b.sink.count(EventKind::ACTIVATE), 1) << explain(b);
  int computed = 0, committed = 0;
  for (const EpochRecord& e : b.sink.epochs) {
    EXPECT_TRUE(e.observe_only);
    if (!activeRecord(e)) continue;
    committed += e.committed ? 1 : 0;
    computed += e.cls[static_cast<size_t>(MeasClass::Dopp)].accepted > 0 ? 1 : 0;
  }
  EXPECT_EQ(committed, 0);
  EXPECT_GT(computed, 50);
  EXPECT_EQ(b.report_mismatch, 0);
  EXPECT_EQ(b.host_writes, 0) << "OBSERVE reported core_modified";
  EXPECT_TRUE(b.sink.covs.empty()) << "no postGNSS covariance row without a commit";
  EXPECT_FALSE(b.sink.anyDetail(EventKind::CONFIG_WARNING, "host write")) << "the bit identity must not rely on the guard";
}

// ---------------------------------------------------------------------------------------------
// GNSS enabled but no GNSS message at all (the G-无数据 gate's precondition): the host is bit-identical to the host-only
// run and the augmented block stays empty.
// ---------------------------------------------------------------------------------------------
TEST(P3Facade, NoGnssDataHostBitIdentical) {
  Options ref;
  ref.call_fusion = false;
  ref.duration_s = 20.0;
  Options on = ref;
  on.call_fusion = true;
  on.push_epochs = false;
  const Result a = World(ref).run();
  if (a.no_fixture) P3_SKIP(std::string("blocked: dev-set fixture Grass02 with brdc/ under ") + LIVO_GNSS_TEST_DATA_DIR);
  const Result b = World(on).run();
  if (b.blocked) P3_SKIP("blocked: P3-A1 (" + b.blocked_why + ")");
  ASSERT_EQ(a.host_bytes.size(), b.host_bytes.size());
  for (size_t k = 0; k < a.host_bytes.size(); ++k)
    ASSERT_EQ(std::memcmp(a.host_bytes[k].data(), b.host_bytes[k].data(), a.host_bytes[k].size() * sizeof(double)), 0)
        << "host state differs after cycle " << k;
  EXPECT_TRUE(b.sink.epochs.empty());
  EXPECT_TRUE(b.sink.covs.empty());
  EXPECT_EQ(b.host_writes, 0);
  EXPECT_EQ(b.report_mismatch, 0);
  EXPECT_FALSE(b.status.active);
  EXPECT_EQ(b.status.aug_dim, 0);
  EXPECT_EQ(b.status.phase, Phase::WAIT_NAV) << "the host is ready (≥ 50 LIO frames), no navigation data ever came";
  for (const Event& e : b.sink.events) EXPECT_EQ(e.kind, EventKind::PHASE_CHANGE) << e.detail;
}

// ---------------------------------------------------------------------------------------------
// A43: pseudorange rows at the first epoch of every GPST second, whatever the receiver's epoch phase.
// ---------------------------------------------------------------------------------------------
TEST(P3Facade, OneHzRuleFirstEpochOfEachSecond) {
  for (double phase : {0.005, -0.007, 0.0}) {
    Options o;
    o.duration_s = 36.0;
    o.epoch_phase_s = phase;
    const Result r = World(o).run();
    if (r.no_fixture) P3_SKIP(std::string("blocked: dev-set fixture Grass02 with brdc/ under ") + LIVO_GNSS_TEST_DATA_DIR);
    if (r.blocked) P3_SKIP("blocked: P3-A1 (" + r.blocked_why + ")");
    std::map<int64_t, std::vector<const EpochRecord*>> by_second;
    for (const EpochRecord& e : r.sink.epochs)
      if (activeRecord(e) && e.delta <= 0.3) by_second[static_cast<int64_t>(std::floor(e.t_gpst))].push_back(&e);
    ASSERT_GT(by_second.size(), 8u) << "phase " << phase << "\n" << explain(r);
    int seconds_with_code = 0;
    for (const auto& kv : by_second) {
      int with_code = 0;
      const EpochRecord* first = kv.second.front();
      for (const EpochRecord* e : kv.second) {
        if (e->t_gpst < first->t_gpst) first = e;
        with_code += e->cls[static_cast<size_t>(MeasClass::Psr)].candidates > 0 ? 1 : 0;
      }
      EXPECT_EQ(with_code, 1) << "phase " << phase << " second " << kv.first;
      EXPECT_GT(first->cls[static_cast<size_t>(MeasClass::Psr)].candidates, 0)
          << "phase " << phase << ": the code epoch of second " << kv.first << " is not its first epoch";
      seconds_with_code += with_code > 0 ? 1 : 0;
    }
    EXPECT_EQ(seconds_with_code, static_cast<int>(by_second.size())) << "phase " << phase;
  }
}

// ---------------------------------------------------------------------------------------------
// A stalled GNSS stream: epochs δ ≤ max_bridge_s behind are bridged across cycles; older ones are LATE_EPOCH.
// ---------------------------------------------------------------------------------------------
TEST(P3Facade, LateEpochsBridgedOrDropped) {
  Options o;
  o.duration_s = 45.0;
  o.stalls = {Stall{30.0, 30.6, 0.20}, Stall{38.0, 38.6, 0.65}};
  const Result r = World(o).run();
  if (r.no_fixture) P3_SKIP(std::string("blocked: dev-set fixture Grass02 with brdc/ under ") + LIVO_GNSS_TEST_DATA_DIR);
  if (r.blocked) P3_SKIP("blocked: P3-A1 (" + r.blocked_why + ")");
  ASSERT_EQ(r.sink.count(EventKind::ACTIVATE), 1) << explain(r);
  double t_act = 0.0;
  for (const Event& e : r.sink.events)
    if (e.kind == EventKind::ACTIVATE) t_act = e.t_k;
  ASSERT_LT(t_act - r.sink.epochs.front().t_k, 28.0) << "activation must precede the first stall";
  int bridged = 0, late_records = 0, committed = 0;
  for (const EpochRecord& e : r.sink.epochs) {
    const double tau = e.t_est - r.t_host0;
    if (tau >= 30.05 && tau < 30.55 && activeRecord(e)) {
      EXPECT_TRUE(e.cross_cycle_bridge) << "tau " << tau << " delta " << e.delta;
      EXPECT_GT(e.delta, 0.1);
      EXPECT_LE(e.delta, 0.3);
      EXPECT_EQ(e.cls[static_cast<size_t>(MeasClass::Dopp)].rejected[static_cast<size_t>(RejectReason::Late)], 0);
      ++bridged;
      committed += e.committed ? 1 : 0;   // a TDCP start epoch may carry no row at all (its Doppler rows are left out)
    }
    if (tau >= 38.05 && tau < 38.55) {
      EXPECT_GT(e.delta, 0.3) << "tau " << tau;
      EXPECT_GT(e.cls[static_cast<size_t>(MeasClass::Dopp)].rejected[static_cast<size_t>(RejectReason::Late)], 0);
      ++late_records;
    }
  }
  EXPECT_GE(bridged, 4) << "stalled epochs within max_bridge_s must be bridged, not dropped";
  EXPECT_GE(committed, 2) << "bridged epochs must still update";
  EXPECT_GE(late_records, 4);
  EXPECT_GE(r.sink.count(EventKind::LATE_EPOCH), late_records);
  EXPECT_EQ(r.summary.n_late, r.sink.count(EventKind::LATE_EPOCH));
}

// ---------------------------------------------------------------------------------------------
// A12: update_slot = post_vio consumes exactly the epochs pre_lio consumes (only the update instant differs).
// ---------------------------------------------------------------------------------------------
TEST(P3Facade, PostVioSlotConsumesSameEpochs) {
  Options pre;
  pre.duration_s = 32.0;
  Options post = pre;
  post.slot = Config::UpdateSlot::PostVio;
  const Result a = World(pre).run();
  if (a.no_fixture) P3_SKIP(std::string("blocked: dev-set fixture Grass02 with brdc/ under ") + LIVO_GNSS_TEST_DATA_DIR);
  if (a.blocked) P3_SKIP("blocked: P3-A1 (" + a.blocked_why + ")");
  const Result b = World(post).run();
  ASSERT_FALSE(b.blocked);
  ASSERT_EQ(a.sink.epochs.size(), b.sink.epochs.size());
  for (size_t i = 0; i < a.sink.epochs.size(); ++i) {
    EXPECT_EQ(a.sink.epochs[i].t_gpst, b.sink.epochs[i].t_gpst) << i;
    EXPECT_EQ(a.sink.epochs[i].t_k, b.sink.epochs[i].t_k) << i;
  }
  EXPECT_EQ(a.sink.count(EventKind::ACTIVATE), 1) << explain(a);
  EXPECT_EQ(b.sink.count(EventKind::ACTIVATE), 1) << explain(b);
  int ca = 0, cb = 0;
  for (const EpochRecord& e : a.sink.epochs) ca += e.committed ? 1 : 0;
  for (const EpochRecord& e : b.sink.epochs) cb += e.committed ? 1 : 0;
  EXPECT_GT(ca, 50);
  EXPECT_GT(cb, 50);
  EXPECT_EQ(b.report_mismatch, 0) << "post_vio: StepReport::core_modified disagrees with the host copy";
  EXPECT_GT(b.host_writes, 50);
}

// ---------------------------------------------------------------------------------------------
// §11 OUTAGE: 2.5 s without any epoch parks the open TDCP pair (its clone leaves x_G), then RECOVERY; pairs and commits
// resume afterwards.
// ---------------------------------------------------------------------------------------------
TEST(P3Facade, OutageParksPairsAndRecovers) {
  Options o;
  o.duration_s = 45.0;
  o.gaps = {{33.0, 35.5}};
  const Result r = World(o).run();
  if (r.no_fixture) P3_SKIP(std::string("blocked: dev-set fixture Grass02 with brdc/ under ") + LIVO_GNSS_TEST_DATA_DIR);
  if (r.blocked) P3_SKIP("blocked: P3-A1 (" + r.blocked_why + ")");
  ASSERT_EQ(r.sink.count(EventKind::ACTIVATE), 1) << explain(r);
  EXPECT_GE(r.sink.count(EventKind::OUTAGE_ON), 1) << explain(r);
  EXPECT_GE(r.sink.count(EventKind::RECOVERY) + r.sink.count(EventKind::OUTAGE_OFF), 1) << explain(r);
  EXPECT_EQ(r.sink.count(EventKind::REINIT), 0);
  int open_late_in_gap = 0;
  for (const auto& c : r.pair_open_by_cycle)
    if (c.first >= 34.6 && c.first < 35.5) open_late_in_gap += c.second;
  EXPECT_EQ(open_late_in_gap, 0) << "the open pair must be parked (clone removed) once the outage is declared";
  int committed_after = 0, pairs_after = 0;
  for (const EpochRecord& e : r.sink.epochs) {
    if (e.t_est - r.t_host0 < 37.5) continue;
    committed_after += e.committed ? 1 : 0;
    pairs_after += e.cls[static_cast<size_t>(MeasClass::Tdcp)].accepted > 0 ? 1 : 0;
  }
  EXPECT_GT(committed_after, 30);
  EXPECT_GT(pairs_after, 5);
  EXPECT_EQ(r.clone_mismatch, 0);
}

// ---------------------------------------------------------------------------------------------
// §10 L2: a +1 ms receiver clock step (≈ 300 km in code and carrier, nothing in Doppler) is detected once on the
// median code innovation, b is corrected by k·c·1 ms, and the TDCP pair spanning the step stays usable (the step is
// removed from λΔΦ and from the D4 Doppler-integration check).
// ---------------------------------------------------------------------------------------------
TEST(P3Facade, IntegerMsClockJumpDetectedAndBridgedByTdcp) {
  // The scene is deterministic up to the step, so a jump-free run tells where the TDCP pairs are; the step is then put
  // in the middle of a pair (k1 − 0.25 s): detected on an intermediate epoch, it must be carried to that pair's end.
  Options base;
  base.duration_s = 40.0;
  const Result r0 = World(base).run();
  if (r0.no_fixture) P3_SKIP(std::string("blocked: dev-set fixture Grass02 with brdc/ under ") + LIVO_GNSS_TEST_DATA_DIR);
  if (r0.blocked) P3_SKIP("blocked: P3-A1 (" + r0.blocked_why + ")");
  double k1 = -1.0;
  for (const EpochRecord& e : r0.sink.epochs) {
    const double tau = e.t_est - r0.t_host0;
    if (tau > 28.0 && activeRecord(e) && e.cls[static_cast<size_t>(MeasClass::Tdcp)].candidates > 0) {
      k1 = tau;
      break;
    }
  }
  ASSERT_GT(k1, 0.0) << "no TDCP pair end after 28 s\n" << explain(r0);
  Options o = base;
  o.clock_jump_at_s = k1 - 0.25;
  const Result r = World(o).run();
  ASSERT_FALSE(r.blocked);
  ASSERT_EQ(r.sink.count(EventKind::ACTIVATE), 1) << explain(r);
  int jumps = 0;
  double k_ms = 0.0, t_jump = 0.0;
  for (const Event& e : r.sink.events)
    if (e.kind == EventKind::CLOCK_JUMP) {
      ++jumps;
      k_ms = e.magnitude;
      t_jump = e.t_est - r.t_host0;
    }
  ASSERT_EQ(jumps, 1) << explain(r);
  EXPECT_EQ(k_ms, 1.0);
  EXPECT_NEAR(t_jump, k1 - 0.2, 0.06) << "detected at the first epoch after the step, inside the pair";
  int slips = 0, tdcp_after = 0, committed_after = 0, at_k1 = -1;
  for (const EpochRecord& e : r.sink.epochs) {
    const double tau = e.t_est - r.t_host0;
    if (tau < k1 - 0.3) continue;
    const ClassStats& t = e.cls[static_cast<size_t>(MeasClass::Tdcp)];
    for (RejectReason rr : {RejectReason::SlipD1, RejectReason::SlipD2, RejectReason::SlipD3, RejectReason::SlipD4, RejectReason::SlipD5})
      slips += t.rejected[static_cast<size_t>(rr)];
    if (std::fabs(tau - k1) < 0.02) at_k1 = t.accepted;
    tdcp_after += t.accepted;
    committed_after += e.committed ? 1 : 0;
  }
  EXPECT_EQ(slips, 0) << "the pair spanning the step must not see a slip detector fire (k1 " << k1 << ")\n" << explain(r);
  EXPECT_GT(at_k1, 0) << "the pair spanning the step must still deliver accepted TDCP rows at its end";
  EXPECT_GT(tdcp_after, 20);
  EXPECT_GT(committed_after, 50);
  EXPECT_EQ(r.sink.count(EventKind::ROLLBACK), 0);
}

// ---------------------------------------------------------------------------------------------
// L-PVT arm (§4.6, mode loose_pvt): initialised from the raw observations like T, then updated only by the receiver's
// own PVT at 1 Hz (A43); no clock or ISB slot, no code/Doppler/TDCP row.
// ---------------------------------------------------------------------------------------------
TEST(P3Facade, LoosePvtArm) {
  Options o;
  o.duration_s = 40.0;
  o.mode = Config::Mode::LoosePvt;
  const Result r = World(o).run();
  if (r.no_fixture) P3_SKIP(std::string("blocked: dev-set fixture Grass02 with brdc/ under ") + LIVO_GNSS_TEST_DATA_DIR);
  if (r.blocked) P3_SKIP("blocked: P3-A1 (" + r.blocked_why + ")");
  ASSERT_EQ(r.sink.count(EventKind::ACTIVATE), 1) << explain(r);
  EXPECT_FALSE(r.slot_names.count("clk")) << "L-PVT has no clock slot";
  EXPECT_FALSE(r.slot_names.count("isb_E") || r.slot_names.count("isb_C"));
  EXPECT_FALSE(r.slot_names.count("clone"));
  int pvt_records = 0, pvt_pos = 0, pvt_vel = 0, raw_rows = 0;
  std::map<int64_t, int> per_second;
  for (const EpochRecord& e : r.sink.epochs) {
    if (!activeRecord(e)) continue;
    const ClassStats& pp = e.cls[static_cast<size_t>(MeasClass::PvtPos)];
    const ClassStats& pv = e.cls[static_cast<size_t>(MeasClass::PvtVel)];
    if (pp.candidates > 0) {
      ++pvt_records;
      ++per_second[static_cast<int64_t>(std::floor(e.t_gpst))];
    }
    pvt_pos += pp.accepted;
    pvt_vel += pv.accepted;
    for (MeasClass c : {MeasClass::Psr, MeasClass::Dopp, MeasClass::Tdcp}) raw_rows += e.cls[static_cast<size_t>(c)].accepted;
  }
  EXPECT_GT(pvt_records, 10);
  EXPECT_GT(pvt_pos, 30);
  EXPECT_GT(pvt_vel, 30);
  EXPECT_EQ(raw_rows, 0) << "the loose arm must not use raw observations after ACTIVATE";
  for (const auto& kv : per_second) EXPECT_EQ(kv.second, 1) << "A43: one PVT per GPST second, second " << kv.first;
  EXPECT_EQ(r.report_mismatch, 0) << "L-PVT: StepReport::core_modified disagrees with the host copy";
  EXPECT_GT(r.host_writes, 10);
  ASSERT_TRUE(r.have_global);
  EXPECT_LT(r.ant_err_enu.head<2>().norm(), 2.0);
  EXPECT_LT(r.att_err_deg, 2.0);
}

// ---------------------------------------------------------------------------------------------
// Ablation arms T−pr / T−dop / T−tdcp (A46): the initialisation is unchanged (it always uses SPP and Doppler LS, §8
// "消融"), the disabled class never enters an update, the others do.
// ---------------------------------------------------------------------------------------------
TEST(P3Facade, AblationArmsDropExactlyTheirClass) {
  for (int arm = 0; arm < 3; ++arm) {
    Options o;
    o.duration_s = 32.0;
    o.use_psr = arm != 0;
    o.use_dopp = arm != 1;
    o.use_tdcp = arm != 2;
    const Result r = World(o).run();
    if (r.no_fixture) P3_SKIP(std::string("blocked: dev-set fixture Grass02 with brdc/ under ") + LIVO_GNSS_TEST_DATA_DIR);
    if (r.blocked) P3_SKIP("blocked: P3-A1 (" + r.blocked_why + ")");
    ASSERT_EQ(r.sink.count(EventKind::ACTIVATE), 1) << "arm " << arm << "\n" << explain(r);
    int acc[3] = {0, 0, 0}, committed = 0;
    for (const EpochRecord& e : r.sink.epochs) {
      if (!activeRecord(e)) continue;
      committed += e.committed ? 1 : 0;
      for (int c = 0; c < 3; ++c) acc[c] += e.cls[static_cast<size_t>(c)].accepted;
    }
    for (int c = 0; c < 3; ++c) {
      if (c == arm) {
        EXPECT_EQ(acc[c], 0) << "arm " << arm << ": disabled class " << measClassName(static_cast<MeasClass>(c));
      } else {
        EXPECT_GT(acc[c], 0) << "arm " << arm << ": class " << measClassName(static_cast<MeasClass>(c));
      }
    }
    // Without Doppler (10 Hz epochs) only the 1 Hz code epochs and the TDCP pair endpoints carry rows: far fewer commits.
    EXPECT_GT(committed, 25) << "arm " << arm;
    if (arm == 2) {
      EXPECT_FALSE(r.slot_names.count("clone")) << "T−tdcp opens no pair";
    }
  }
}
