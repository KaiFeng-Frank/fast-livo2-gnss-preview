// SPDX-License-Identifier: BSD-3-Clause
// test_u13_init.cpp — TEST_PLAN §1 U13 (P3-A1): the initialisation state machine and ALIGN/ACTIVATE (DESIGN §8).
//
// Synthetic trajectory + real ephemerides: a ground vehicle (static, a smooth start, then a 12 m-radius arc on a 2 %
// grade at 1.5 m/s) is flown in a known N frame at the dev-set fixture's site; the W frame of the host is tilted and
// yawed against N with a known offset; pseudoranges and Dopplers of every healthy satellite of the dev-set fixture's
// same-day IGS BRDC (A24) above 10° are generated with a known receiver clock (bias + drift) and known ISBs; the Sagnac
// rate of the Doppler is taken by numerical differentiation of ρ(t) along the broadcast orbit (never the estimator's
// formula, WP-g's rule for Doppler forward models). The host side is the real ImuBridge fed with 200 Hz truth samples.
// A test-local "facade" (onLioFrame ×50, onNavCount, RobustSpp + RobustDopplerLs per epoch, addSample, then
// tryAlign/activate when the machine reaches ALIGN) drives the Initializer exactly as the protocol in initializer.hpp says.
//
//   RecoversKnownPsiR0TBDIsb           noise-free: ψ, t, b, d, ι to machine/model precision; r_0 = robust window mean;
//                                      the activated block is a consistent augmentation (global antenna covariance =
//                                      GNSS-only covariance for any host P, zero cross-covariance with the host); noisy:
//                                      every error inside 4σ of the activated covariance; an enabled system without data
//                                      gets ι = 0, σ = 30 m (A35); the L-PVT arm gets no clock/ISB slots
//   StillOrInsufficientExcitationKeepsWaiting   standstill, 0.3 m/s creep and a 2 s start never leave COLLECT
//   InitEpochsNotDoubleCounted          epochs up to k_a are consumed and refused afterwards, REINIT keeps N
//   TimeCheckFlagsOffset                §7 online check: 0 lag passes, a 150 ms lag raises offset_warn
//   ProcrustesYawExact                  the 2D Procrustes start on exactly rotated velocities
// AMENDMENTS A51 (P4-INIT), the LIO-aided multi-epoch path: every epoch sees only 1–3 satellites (a rotating subset of
// 5 GPS + 2 Galileo satellites of the real sky, so no epoch ever has an SPP), 70 s on the arc:
//   LioAidedFewSatellitesRecoversPsiTBD noise-free: ψ, t, b, d, ι_E to model precision through path "lio_aided" with
//                                      the N frame fixed to the truth, and from a cold start (no fix at all: the window
//                                      fixes N); consistent augmentation as U13-1; a 1 ms receiver clock jump restarts
//                                      the window and the alignment after it is exact again
//   LioAidedIsStatisticallyConsistent  20 noisy runs: NEES of [ψ, t, b, d, ι_E] against the reported joint covariance is χ²₇
//   LioAidedInsufficientGeometryKeepsWaiting  3 distinct satellites in the window, or a standstill, never activate
// The test-local facade follows the A51 protocol: every epoch with a bridge is handed over (SPP or not).
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <map>
#include <random>
#include <set>
#include <string>
#include <tuple>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "internal/initializer.hpp"
#include "internal/quality.hpp"
#include "internal/rows.hpp"
#include "internal/rtk_adapter.hpp"
#include "internal/update.hpp"   // chi2Quantile
#include "../../gnss_io/test/fixture_discovery.hpp"   // WP-e's one discovery rule (A24): realFixtures(), fixtureBrdc()

using namespace livo_gnss;

namespace {

constexpr double kC = 299792458.0;
constexpr double kDeg = M_PI / 180.0;
constexpr double kGrav = 9.80665;
// Site of the dev-set fixture (Grass02 receiver_pvt, first record; as U16): only a real sky is needed, not this point.
constexpr double kSiteLatDeg = 34.0339415, kSiteLonDeg = 108.7614865, kSiteH = 388.05;
const Eigen::Vector3d kLever(-0.31635, 0.10926, 0.4006);   // DESIGN §14 l^I (Avia)
constexpr double kH0 = 5000.0;                              // host time of τ = 0 (estimator timeline)

std::string devSetBrdc() {
  for (const std::string& f : livo_gnss_io::testsupport::realFixtures(LIVO_GNSS_TEST_DATA_DIR)) {
    const std::string b = livo_gnss_io::testsupport::fixtureBrdc(f);
    if (!b.empty()) return b;
  }
  return std::string();
}

GTime epochInside(const rtk::NavStore& nav) {   // a whole second ≈ 1234 s after the median GPS toe
  std::vector<GTime> toes;
  for (const rtk::EphKey& k : nav.coverage())
    if (k.sat.sys == Sys::GPS) toes.push_back(k.toe);
  if (toes.empty()) return GTime();
  std::sort(toes.begin(), toes.end());
  GTime t = toes[toes.size() / 2].add(1234.0);
  t.frac = 0.0;
  return t;
}

Eigen::Matrix3d rotAxis(const Eigen::Vector3d& axis, double ang) {
  return Eigen::AngleAxisd(ang, axis.normalized()).toRotationMatrix();
}

double yawOf(const Eigen::Matrix3d& R) { return std::atan2(R(1, 0), R(0, 0)); }

double wrap(double a) {
  a -= 2.0 * M_PI * std::floor((a + M_PI) / (2.0 * M_PI));
  return a;
}

// Numerical Sagnac rate (U16's construction): d/dτ[geodist − ‖·‖] along r_s(t) = SatSolver::ephPos(t) and a straight
// antenna path, central differences h = 0.5 s; geodist − ‖·‖ is exactly the Sagnac term, so the orbit curvature cancels.
bool sagnacTruth(const rtk::SatSolver& solver, const SatId& id, const GTime& t_tx, const Eigen::Vector3d& r_A,
                 const Eigen::Vector3d& v_A, double* sagnac) {
  const double h = 0.5;
  Eigen::Vector3d r0, rp, rm;
  double dts = 0.0, var = 0.0;
  if (!solver.ephPos(id, t_tx, &r0, &dts, &var) || !solver.ephPos(id, t_tx.add(h), &rp, &dts, &var) ||
      !solver.ephPos(id, t_tx.add(-h), &rm, &dts, &var))
    return false;
  const Eigen::Vector3d ap = r_A + v_A * h, am = r_A - v_A * h;
  const Eigen::Vector3d dp = rp - ap, dm = rm - am;
  const double rho_dot = (rtk::geodist(rp, ap, nullptr) - rtk::geodist(rm, am, nullptr)) / (2.0 * h);
  *sagnac = rho_dot - (dp - dm).dot(dp + dm) / (dp.norm() + dm.norm()) / (2.0 * h);
  return true;
}

// ---------------------------------------------------------------------------------------------
// Synthetic world
// ---------------------------------------------------------------------------------------------
struct Params {
  double t_static = 4.0, t_ramp = 3.0, speed = 1.5, radius = 12.0, grade = 0.02, heading0 = 0.7;
  double duration = 40.0;
  double yaw_W = 1.234;                               // yaw part of R_{N←W}
  Eigen::Vector3d tilt_axis = Eigen::Vector3d(1.0, 0.5, 0.0);
  double tilt_W = 3.0 * kDeg;                         // the W frame is the IMU frame at start: not level (§1)
  Eigen::Vector3d T_NW = Eigen::Vector3d(2.5, -1.0, 0.7);
  double b0 = 12345.678, d0 = 45.25;                  // receiver clock [m], drift [m/s]
  double isb_C = -7.5, isb_E = 3.2;
  bool use_gps = true, use_gal = true, use_bds = true, use_glo = true;
  bool noise = false;
  uint32_t seed = 7;
  double psr_white = 0.8, psr_bias = 0.5, dopp_white = 0.05;
  double lag_s = 0.0;                                 // GNSS epochs labelled lag_s early on the host timeline
  bool var_matches_noise = false;                     // a-priori code variance = the white noise actually drawn
  double epoch_phase = 0.037;                         // GNSS epoch τ_k = 0.1 k + phase (not on the LIO grid)
  // A51 few-satellite sky: a pool of the first pool_gps GPS and pool_gal Galileo satellites above 15° (0, 0 = every
  // satellite); epoch k sees 1 + (k mod 3) of them, consecutive in the pool from index k mod |pool|.
  int pool_gps = 0, pool_gal = 0;
  double jump_tau = -1.0, jump_m = 0.0;               // receiver clock jump of jump_m from τ = jump_tau on
  // Clock-drift random walk (PSD drift_rw [m²/s³]; the receiver of M3DGR wanders ±0.1–0.25 m/s per minute): d(τ) = d0 +
  // w(τ), b(τ) = b0 + d0 τ + ∫w. Deterministic from wander_seed.
  double drift_rw = 0.0;
  uint32_t wander_seed = 99;
  // Real-data rows (A51's re-evaluation branch): the code carries Saastamoinen troposphere and the BRDC Klobuchar
  // ionosphere; the rows are delivered as the facade builds them without any fix (el = 90°, no atmosphere, models not
  // fixed) and the sample carries the Klobuchar and the receiver stds (psr_std_m, dopp_std_hz).
  bool atmosphere = false;
  double psr_std_m = 0.5, dopp_std_hz = 0.2;
};

struct Truth {
  Eigen::Matrix3d R_NI = Eigen::Matrix3d::Identity();
  Eigen::Vector3d p = Eigen::Vector3d::Zero(), v = Eigen::Vector3d::Zero(), a = Eigen::Vector3d::Zero();
  Eigen::Vector3d omega = Eigen::Vector3d::Zero();   // body rate (I)
};

class World {
 public:
  World(const Params& p, const rtk::NavStore& nav) : p_(p), nav_(nav), solver_(nav), rng_(p.seed) {
    T0_ = epochInside(nav);
    Geodetic g;
    g.lat_rad = kSiteLatDeg * kDeg;
    g.lon_rad = kSiteLonDeg * kDeg;
    g.h_m = kSiteH;
    anchor_true_.geo = g;
    anchor_true_.r0_ecef = rtk::pos2ecef(g);
    anchor_true_.R_EN = rtk::enuRotation(g);
    anchor_true_.t_fixed = T0_;
    anchor_true_.valid = true;
    mount_ = rotAxis(Eigen::Vector3d(0.3, -1.0, 0.2), 2.0 * kDeg);
    R_NW_ = Leveling::Rz(p.yaw_W) * rotAxis(p.tilt_axis, p.tilt_W);
    g_W_ = R_NW_.transpose() * Eigen::Vector3d(0.0, 0.0, -kGrav);
    // satellites of the file above 5° at the start (re-checked at 10° per epoch)
    std::set<int> seen;
    for (const rtk::EphKey& k : nav.coverage()) {
      if (!seen.insert(k.sat.rtk_sat).second) continue;
      const Sys s = k.sat.sys;
      if (!((s == Sys::GPS && p.use_gps) || (s == Sys::GAL && p.use_gal) || (s == Sys::BDS && p.use_bds) ||
            (s == Sys::GLO && p.use_glo)))
        continue;
      rtk::SatState st;
      if (!solver_.satState(k.sat, T0_, 2.2e7, &st) || !st.ok || !rtk::svhHealthy(s, st.svh)) continue;
      Eigen::Vector3d e;
      rtk::geodist(st.r_ecef, anchor_true_.r0_ecef, &e);
      double az = 0.0, el = 0.0;
      rtk::satAzel(anchor_true_.geo, e, &az, &el);
      if (el < 5.0 * kDeg) continue;
      sats_.push_back(k.sat);
      bias_[k.sat.rtk_sat] = (p.noise && p.psr_bias > 0.0) ? std::normal_distribution<double>(0.0, p.psr_bias)(rng_) : 0.0;
      if (el >= 15.0 * kDeg) {
        if (s == Sys::GPS && n_pool_gps_ < p.pool_gps) {
          pool_.push_back(k.sat);
          ++n_pool_gps_;
        } else if (s == Sys::GAL && n_pool_gal_ < p.pool_gal) {
          pool_.push_back(k.sat);
          ++n_pool_gal_;
        }
      }
    }
    buildWander();
    have_klob_ = nav.iono(&klob_) && klob_.valid();
  }
  bool atmosphere() const { return p_.atmosphere; }
  bool haveKlobuchar() const { return have_klob_; }
  const Klobuchar& klobuchar() const { return klob_; }
  const Params& params() const { return p_; }
  // clock wander table (0.01 s grid)
  void buildWander() {
    if (!(p_.drift_rw > 0.0)) return;
    std::mt19937 rng(p_.wander_seed);
    std::normal_distribution<double> n(0.0, 1.0);
    const double dt = 0.01;
    const int N = static_cast<int>((p_.duration + 10.0) / dt) + 2;
    wander_w_.assign(N, 0.0);
    wander_b_.assign(N, 0.0);
    for (int i = 1; i < N; ++i) {
      wander_w_[i] = wander_w_[i - 1] + std::sqrt(p_.drift_rw * dt) * n(rng);
      wander_b_[i] = wander_b_[i - 1] + 0.5 * (wander_w_[i] + wander_w_[i - 1]) * dt;
    }
  }
  double wanderAt(const std::vector<double>& v, double tau) const {
    if (v.empty() || tau <= 0.0) return 0.0;
    const double x = tau / 0.01;
    const int i = std::min(static_cast<int>(x), static_cast<int>(v.size()) - 2);
    const double f = x - i;
    return v[i] * (1.0 - f) + v[i + 1] * f;
  }
  int poolSize() const { return static_cast<int>(pool_.size()); }
  bool fewSats() const { return p_.pool_gps > 0 || p_.pool_gal > 0; }
  // Satellites visible at epoch k in the few-satellite sky.
  bool visible(int k, const SatId& id) const {
    if (!fewSats()) return true;
    const int L = static_cast<int>(pool_.size());
    if (L == 0) return false;
    const int n = 1 + k % 3;
    for (int i = 0; i < n && i < L; ++i)
      if (pool_[(k + i) % L] == id) return true;
    return false;
  }

  const GTime& T0() const { return T0_; }
  const AnchorFrame& anchorTrue() const { return anchor_true_; }
  const Eigen::Matrix3d& R_NW() const { return R_NW_; }
  const Eigen::Vector3d& gW() const { return g_W_; }
  int nSats() const { return static_cast<int>(sats_.size()); }

  // Vehicle (IMU) truth in N at τ seconds after start.
  Truth truth(double tau) const {
    Truth t;
    const double V = p_.speed, Ts = p_.t_static, Tr = p_.t_ramp, Rc = p_.radius;
    double s = 0.0, v = 0.0, vd = 0.0;
    if (tau >= Ts && tau < Ts + Tr) {
      const double u = tau - Ts;
      v = 0.5 * V * (1.0 - std::cos(M_PI * u / Tr));
      vd = 0.5 * V * M_PI / Tr * std::sin(M_PI * u / Tr);
      s = 0.5 * V * (u - Tr / M_PI * std::sin(M_PI * u / Tr));
    } else if (tau >= Ts + Tr) {
      v = V;
      s = 0.5 * V * Tr + V * (tau - Ts - Tr);
    }
    const double h = p_.heading0 + s / Rc, hd = v / Rc;
    t.p = Eigen::Vector3d(Rc * (std::sin(h) - std::sin(p_.heading0)), Rc * (-std::cos(h) + std::cos(p_.heading0)), p_.grade * s);
    t.v = Eigen::Vector3d(v * std::cos(h), v * std::sin(h), p_.grade * v);
    t.a = Eigen::Vector3d(vd * std::cos(h) - v * hd * std::sin(h), vd * std::sin(h) + v * hd * std::cos(h), p_.grade * vd);
    t.R_NI = Leveling::Rz(h) * mount_;
    t.omega = mount_.transpose() * Eigen::Vector3d(0.0, 0.0, hd);
    return t;
  }
  Eigen::Vector3d antennaN(double tau) const {
    const Truth t = truth(tau);
    return t.p + t.R_NI * kLever;
  }
  Eigen::Vector3d antennaVelN(double tau) const {
    const Truth t = truth(tau);
    return t.v + t.R_NI * t.omega.cross(kLever);
  }
  double clockBias(double tau) const {
    return p_.b0 + p_.d0 * tau + wanderAt(wander_b_, tau) + (p_.jump_tau >= 0.0 && tau >= p_.jump_tau ? p_.jump_m : 0.0);
  }
  double clockDrift(double tau) const { return p_.d0 + wanderAt(wander_w_, tau); }

  // Host state (W frame) at host time t_host, with the given covariance.
  CoreState host(double t_host, const HostMat& P) const {
    const Truth t = truth(t_host - kH0);
    CoreState c;
    c.R = R_NW_.transpose() * t.R_NI;
    c.p = R_NW_.transpose() * (t.p - p_.T_NW);
    c.v = R_NW_.transpose() * t.v;
    c.bg.setZero();
    c.ba.setZero();
    c.g = g_W_;
    c.P = P;
    return c;
  }
  BridgeSample sample(double t_host, double t_mid) const {
    BridgeSample s;
    const Truth t = truth(t_host - kH0), m = truth(t_mid - kH0);
    s.t = t_host;
    s.R = R_NW_.transpose() * t.R_NI;
    s.p = R_NW_.transpose() * (t.p - p_.T_NW);
    s.v = R_NW_.transpose() * t.v;
    s.a_world = R_NW_.transpose() * m.a;   // kinematic acceleration R·f + g of the interval
    s.omega = m.omega;
    s.f_body = (R_NW_.transpose() * m.R_NI).transpose() * (s.a_world - g_W_);
    return s;
  }
  // 500 Hz: the GNSS epochs (x.x37 s) sit at IMU interval midpoints, where the bridge's per-interval rate ω(mid) is the
  // truth's ω(t_g) — during the speed ramp ω varies and any other phase leaves a ~1e-6 rad yaw model error in this world.
  std::vector<BridgeSample> cycle(double t0, double t1) const {
    std::vector<BridgeSample> out;
    const int n = 50;
    for (int i = 0; i <= n; ++i) {
      const double ti = t0 + (t1 - t0) * i / n;
      const double tm = i == 0 ? ti : t0 + (t1 - t0) * (i - 0.5) / n;
      out.push_back(sample(ti, tm));
    }
    return out;
  }

  double epochTau(int k) const { return 0.1 * k + p_.epoch_phase; }
  double epochTest(int k) const { return epochTau(k) + kH0 - p_.lag_s; }

  // GNSS epoch k: satellite inputs as the facade would hand them to RobustSpp (L0 passed, models fixed at the truth).
  struct Epoch {
    GTime t_rx;
    double tau = 0.0, t_est = 0.0;
    std::vector<SppInput> in;
  };
  Epoch epoch(int k) {
    Epoch ep;
    ep.tau = epochTau(k);
    ep.t_est = epochTest(k);
    const double b = clockBias(ep.tau);
    ep.t_rx = T0_.add(ep.tau + b / kC);   // receiver tag = true GPST + b/c (§1 t_gpst,true = t_rcv − b/c)
    const Eigen::Vector3d rA = anchor_true_.r0_ecef + anchor_true_.R_EN * antennaN(ep.tau);
    const Eigen::Vector3d vA = anchor_true_.R_EN * antennaVelN(ep.tau);
    const Geodetic gA = rtk::ecef2pos(rA);
    std::normal_distribution<double> nrm(0.0, 1.0);
    for (const SatId& id : sats_) {
      if (!visible(k, id)) continue;
      rtk::SatState st;
      double P = 2.2e7;
      bool ok = true;
      const double iota = id.sys == Sys::BDS ? p_.isb_C : id.sys == Sys::GAL ? p_.isb_E : 0.0;   // GLO code unused
      for (int it = 0; it < 4 && ok; ++it) {
        ok = solver_.satState(id, ep.t_rx, P, &st) && st.ok;
        if (ok) P = rtk::geodist(st.r_ecef, rA, nullptr) + b + iota - kC * st.dts;
      }
      if (!ok || !rtk::svhHealthy(id.sys, st.svh)) continue;
      Eigen::Vector3d e;
      rtk::geodist(st.r_ecef, rA, &e);
      double az = 0.0, el = 0.0;
      rtk::satAzel(gA, e, &az, &el);
      if (el < 10.0 * kDeg) continue;
      double sag = 0.0;
      if (!sagnacTruth(solver_, id, st.t_tx, rA, vA, &sag)) continue;
      double f = 1575.42e6;
      if (id.sys == Sys::BDS) f = 1561.098e6;
      if (id.sys == Sys::GLO) {
        const int ch = nav_.gloFcn(id.prn);
        if (ch <= -100) continue;
        f = 1602.0e6 + ch * 0.5625e6;
      }
      const double lambda = kC / f;
      const double se = std::sin(el);
      SppInput s;
      s.sat = id;
      s.state = st;
      s.lambda_m = lambda;
      s.cn0 = 45.0;
      s.el_rad = el;
      s.az_rad = az;
      s.var_psr = p_.var_matches_noise ? p_.psr_white * p_.psr_white * 0.5 * (1.0 + 1.0 / (se * se)) : 0.8 * 0.8 * (1.0 + 1.0 / (se * se));
      s.var_dopp = 0.05 * 0.05 * 0.5 * (1.0 + 1.0 / (se * se));
      const double wp = p_.noise ? p_.psr_white * std::sqrt(0.5 * (1.0 + 1.0 / (se * se))) * nrm(rng_) : 0.0;
      const double wd = p_.noise ? p_.dopp_white * std::sqrt(0.5 * (1.0 + 1.0 / (se * se))) * nrm(rng_) : 0.0;
      s.P_corr = P + (p_.noise ? bias_[id.rtk_sat] : 0.0) + wp;
      const double rate = e.dot(st.v_ecef - vA) + sag;
      s.dopp_hz = -(rate + clockDrift(ep.tau) - kC * st.ddts + wd) / lambda;
      s.psr_usable = id.sys != Sys::GLO;   // GLONASS code off by default (glo_psr = false, §10 L0)
      s.dopp_usable = true;
      s.models_fixed = true;               // ion/trop = 0 in this world
      if (p_.atmosphere) {
        // the code of this world carries troposphere and ionosphere at the true antenna; the rows go out like the
        // facade's no-fix rows (el 90°, no corrections, a-priori variances of that guess)
        const double k = (1575.42e6 / f) * (1575.42e6 / f);
        s.P_corr += rtk::tropModel(ep.t_rx, gA, az, el, 0.7) + (have_klob_ ? rtk::ionModelGpsL1(ep.t_rx, klob_, gA, az, el) * k : 0.0);
        s.models_fixed = false;
        s.el_rad = 0.5 * M_PI;
        s.az_rad = 0.0;
        s.ion_m = s.trop_m = 0.0;
      }
      ep.in.push_back(s);
    }
    return ep;
  }

 private:
  Params p_;
  const rtk::NavStore& nav_;
  rtk::SatSolver solver_;
  std::mt19937 rng_;
  GTime T0_;
  AnchorFrame anchor_true_;
  Eigen::Matrix3d mount_, R_NW_;
  Eigen::Vector3d g_W_;
  std::vector<SatId> sats_;
  std::map<int, double> bias_;
  std::vector<SatId> pool_;
  int n_pool_gps_ = 0, n_pool_gal_ = 0;
  std::vector<double> wander_w_, wander_b_;
  Klobuchar klob_;
  bool have_klob_ = false;
};

HostMat makeHostP(uint32_t seed) {
  std::mt19937 rng(seed);
  std::normal_distribution<double> n(0.0, 1.0);
  Eigen::Matrix<double, 19, 19> B;
  for (int i = 0; i < 19; ++i)
    for (int j = 0; j < 19; ++j) B(i, j) = n(rng);
  Eigen::Matrix<double, 19, 1> s;
  s << 2e-3, 2e-3, 3e-3, 0.02, 0.02, 0.03, 0.01, 0.02, 0.02, 0.02, 1e-4, 1e-4, 1e-4, 0.02, 0.02, 0.02, 0.01, 0.01, 0.01;
  const HostMat K = 0.5 * HostMat::Identity() + 0.5 * B * B.transpose() / 19.0;
  return s.asDiagonal() * K * s.asDiagonal();
}

// ---------------------------------------------------------------------------------------------
// The test-local facade (initializer.hpp protocol)
// ---------------------------------------------------------------------------------------------
struct RunOut {
  bool activated = false;
  AlignResult align;
  std::string activate_why;
  std::vector<std::string> align_failures;
  CoreState core_ka;                 // host state of the activation cycle
  BridgeResult bridge_ka;            // the last sample's bridge
  double tau_ka = 0.0;               // true time of k_a
  std::vector<GTime> fed;            // every epoch handed to addSample
  std::vector<double> fed_tau;
  Phase final_phase = Phase::OFF;
  int max_window = 0;
  int spp_epochs = 0;                // epochs whose GNSS-only SPP succeeded
  int max_la_window = 0;
  double max_la_span = 0.0;
};

// Runs the machine over [0, duration] (or until ACTIVATE when stop_at_activation).
RunOut runFacade(World& world, const Config& cfg, Leveling& lev, Initializer& init, AugBlock& aug, const HostMat& P, double duration,
                 bool stop_at_activation = true, bool fix_true_anchor = false) {
  RunOut out;
  if (fix_true_anchor) init.setAnchor(world.anchorTrue());
  for (int f = 0; f < cfg.init.min_livo_frames; ++f) init.onLioFrame(true, true);
  NoiseModel noise(cfg);
  RobustSpp spp_solver(cfg, noise);
  RobustDopplerLs dop_solver(cfg, noise);
  ImuBridge bridge(cfg);
  int k = 0;
  double t_prev = kH0;
  for (int m = 1; t_prev - kH0 < duration; ++m) {
    const double t_k = kH0 + 0.1 * m;
    bridge.pushCycle(world.cycle(t_prev, t_k), t_prev, t_k, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), world.gW());
    const CoreState core = world.host(t_k, P);
    for (;;) {
      if (!(world.epochTest(k) <= t_k)) break;
      if (world.epochTest(k) <= t_prev) {
        ++k;
        continue;
      }
      const World::Epoch ep_peek = world.epoch(k++);
      init.onNavCount(static_cast<int>(ep_peek.in.size()));
      CollectSample s;
      s.t_rx = ep_peek.t_rx;
      s.t_est = ep_peek.t_est;
      s.raw = ep_peek.in;
      RobustSpp::nlosPrescreen(s.raw, cfg.init.nlos_cn0_margin_db);
      // A51 protocol: every epoch with a bridge goes to the initializer, SPP or not (it sorts the sample into its windows)
      if (spp_solver.solve(s.t_rx, s.raw, nullptr, false, Eigen::Vector3d::Zero(), 0.0, &s.spp)) {
        dop_solver.solve(s.t_rx, s.raw, s.spp.r_ecef, &s.dop);
        ++out.spp_epochs;
      }
      if (!bridge.query(s.t_est, &s.bridge)) continue;
      if (world.atmosphere()) {   // what the facade hands over for A51's re-evaluation (INTERFACE_CHANGES P4-INIT)
        s.ion = world.klobuchar();
        s.ion_valid = world.haveKlobuchar();
        s.psr_std_m.assign(s.raw.size(), world.params().psr_std_m);
        s.dopp_std_hz.assign(s.raw.size(), world.params().dopp_std_hz);
      }
      s.host_ant = ImuBridge::evaluate(core, s.bridge, kLever, true);
      const BridgeResult br_copy = s.bridge;
      out.fed.push_back(s.t_rx);
      out.fed_tau.push_back(ep_peek.tau);
      init.addSample(std::move(s));
      out.max_window = std::max(out.max_window, init.windowSize());
      out.max_la_window = std::max(out.max_la_window, init.laWindowSize());
      out.max_la_span = std::max(out.max_la_span, init.laWindowSpan());
      if (init.phase() == Phase::ALIGN) {
        if (!lev.hasRef()) lev.setRef(core.g);
        AlignResult a = init.tryAlign(core, 0.0, 0.02);
        if (!a.ok) {
          out.align_failures.push_back(a.reason);
          continue;
        }
        std::string why;
        if (init.activate(a, core, &aug, &why)) {
          init.setPhase(Phase::TRACKING);
          out.activated = true;
          out.align = a;
          out.core_ka = core;
          out.bridge_ka = br_copy;
          out.tau_ka = ep_peek.tau;
          if (stop_at_activation) {
            out.final_phase = init.phase();
            return out;
          }
        } else {
          out.activate_why = why;
          init.setPhase(Phase::COLLECT);
        }
      }
    }
    t_prev = t_k;
  }
  out.final_phase = init.phase();
  return out;
}

double qOf(const AugBlock& aug, Slot s, int tag, int i = 0) {
  const int o = aug.find(s, tag);
  return o < 0 ? std::numeric_limits<double>::quiet_NaN() : aug.Q()(o + i, o + i);
}
double xOf(const AugBlock& aug, Slot s, int tag, int i = 0) {
  const int o = aug.find(s, tag);
  return o < 0 ? std::numeric_limits<double>::quiet_NaN() : aug.x()(o + i);
}

}  // namespace

double tiltPriorVar(const livo_gnss::Config& cfg) {
  return cfg.frames.sigma_dov_rad * cfg.frames.sigma_dov_rad + cfg.frames.sigma_lev_rad * cfg.frames.sigma_lev_rad;
}

// =============================================================================================
// U13-1: known ψ, r_0, t, b, d, ι recovered (noise-free to model precision, noisy inside 4σ); consistent augmentation
// =============================================================================================
TEST(U13Init, RecoversKnownPsiR0TBDIsb) {
  const std::string brdc = devSetBrdc();
  if (brdc.empty()) GTEST_SKIP() << "blocked: dev-set fixture with brdc/*.rnx (A24) under " << LIVO_GNSS_TEST_DATA_DIR;
  rtk::NavStore nav;
  std::string why;
  ASSERT_GT(nav.loadRinexNav(brdc, true, &why), 0) << brdc << ": " << why;

  // ---------- noise-free, N frame fixed to the truth: every estimate to model precision ----------
  {
    Params p;
    World world(p, nav);
    ASSERT_GE(world.nSats(), 15) << "too few satellites above 5° in " << brdc;
    Config cfg;
    Leveling lev;
    Initializer init(cfg, lev);
    AugBlock aug(cfg);
    const HostMat P = makeHostP(1);
    const RunOut r = runFacade(world, cfg, lev, init, aug, P, 40.0, true, true);
    ASSERT_TRUE(r.activated) << "no activation; last ALIGN failure: " << (r.align_failures.empty() ? "-" : r.align_failures.back())
                             << "; activate: " << r.activate_why << "; refusals " << init.samplesRefused() << " (" << init.lastRefusal() << ")";
    const AlignResult& a = r.align;
    EXPECT_TRUE(r.align_failures.empty()) << r.align_failures.front();
    EXPECT_EQ(init.phase(), Phase::TRACKING);
    // truth in the (fixed, true) N frame
    const Eigen::Matrix3d C = lev.C(world.gW());
    const Eigen::Matrix3d MC = world.R_NW() * C.transpose();
    ASSERT_LT((MC.col(2) - Eigen::Vector3d::UnitZ()).norm(), 1e-12) << "R_NW·Cᵀ must be a pure yaw";
    const double psi_true = yawOf(MC);
    const Eigen::Vector3d t_true = world.antennaN(r.tau_ka);
    const double b_true = world.clockBias(a.t_k - kH0);
    std::cout << "[U13] noise-free: activation at τ = " << r.tau_ka << " s, window " << a.epochs_used << " epochs, "
              << a.rows_dopp << " Doppler rows, " << a.rows_psr << " code rows over " << a.epochs_psr << " 1 Hz epochs; moving "
              << a.moving_epochs << ", path " << a.path_m << " m\n"
              << "[U13]   ψ err " << wrap(a.yaw - psi_true) << " rad (σ " << a.sigma_yaw << "), |t err| " << (a.trans - t_true).norm()
              << " m, b err " << a.b - b_true << " m, d err " << a.d - p.d0 << " m/s, ι_C err " << a.isb[0] - p.isb_C << ", ι_E err "
              << a.isb[1] - p.isb_E << " m\n";
    EXPECT_LT(std::fabs(wrap(a.yaw - psi_true)), 2e-7);
    EXPECT_LT((a.trans - t_true).norm(), 1e-5);
    EXPECT_NEAR(a.b, b_true, 1e-5);
    EXPECT_NEAR(a.d, p.d0, 1e-6);
    EXPECT_TRUE(a.isb_observed[0] && a.isb_observed[1]);
    EXPECT_FALSE(a.isb_observed[2]) << "GLONASS code is off by default: no ι_R";
    EXPECT_NEAR(a.isb[0], p.isb_C, 1e-5);
    EXPECT_NEAR(a.isb[1], p.isb_E, 1e-5);
    EXPECT_TRUE(a.offset_check_valid);
    EXPECT_LT(std::fabs(a.offset_check_s), 1e-3);
    EXPECT_FALSE(a.offset_warn);
    EXPECT_LE(a.sigma_yaw, cfg.init.yaw_sigma_max_deg * kDeg);
    EXPECT_EQ(a.anchor.r0_ecef, world.anchorTrue().r0_ecef);   // setAnchor() kept
    // the activated block: slot values = the alignment
    EXPECT_NEAR(xOf(aug, Slot::Yaw, 0), a.yaw, 0.0);
    EXPECT_EQ(xOf(aug, Slot::Tilt, 0), 0.0);
    EXPECT_NEAR(xOf(aug, Slot::Clk, 0, 0), a.b, 0.0);
    EXPECT_NEAR(xOf(aug, Slot::Clk, 0, 1), a.d, 0.0);
    EXPECT_NEAR(xOf(aug, Slot::Isb, sysBit(Sys::BDS)), a.isb[0], 0.0);
    EXPECT_NEAR(xOf(aug, Slot::Isb, sysBit(Sys::GAL)), a.isb[1], 0.0);
    EXPECT_FALSE(aug.has(Slot::Isb, sysBit(Sys::GLO)));
    EXPECT_FALSE(aug.has(Slot::Td));
    EXPECT_NEAR(std::sqrt(qOf(aug, Slot::Tilt, 0)), std::sqrt(tiltPriorVar(cfg)), 1e-12);
    EXPECT_NEAR(std::sqrt(qOf(aug, Slot::Yaw, 0)), a.sigma_yaw, 1e-15);

    // Consistent augmentation (§2.4, §8 ACTIVATE): at k_a the global antenna a^N = M(y − o) + t equals the GNSS-only
    // estimate and its covariance is the GNSS-only one — J_tψσ_ψ²J_tψᵀ + J_tφσ_DOV²J_tφᵀ + the batch noise — for any host
    // P: the host enters t with −J_a,host and cancels. Two host covariances, same answer; zero cross-covariance.
    NoiseModel nm(cfg);
    RowBuilder rb(cfg, nm);
    auto antennaCheck = [&](const CoreState& core) {
      LinPoint lp;
      lp.x = &core;
      lp.bridge = &r.bridge_ka;
      lp.aug = &aug;
      lp.leveling = &lev;
      lp.anchor = &a.anchor;
      lp.lever_I = kLever;
      lp.pivot_W = a.pivot_W;
      lp.gravity_est_en = true;
      lp.delta = r.bridge_ka.delta;
      const AntennaJacobians J = rb.antennaJacobians(lp);
      const Eigen::MatrixXd S = aug.jointCov(core.P);
      const Eigen::Matrix3d V = J.J_a * S * J.J_a.transpose();
      const Eigen::MatrixXd X = J.J_a * S.leftCols(HostIdx::dim);
      return std::make_tuple(J.a_N, V, X);
    };
    const auto [aN, V, X] = antennaCheck(r.core_ka);
    const Eigen::Matrix3d Vexp = a.cov_trans + a.J_trans_yaw * a.sigma_yaw * a.sigma_yaw * a.J_trans_yaw.transpose() +
                                 a.J_trans_tilt * tiltPriorVar(cfg) * a.J_trans_tilt.transpose();
    EXPECT_LT((aN - a.trans).norm(), 1e-9) << "y(core) = o at k_a ⇒ a^N = t";
    EXPECT_LT((V - Vexp).norm(), 1e-9 * Vexp.norm()) << "V\n" << V << "\nexpected\n" << Vexp;
    EXPECT_LT(X.cwiseAbs().maxCoeff(), 1e-9 * std::sqrt(Vexp.norm() * P.norm())) << "global antenna correlated with the host";
    // the same block under another host covariance: augment again from scratch
    {
      Leveling lev2 = lev;
      AugBlock aug2(cfg);
      CoreState core2 = r.core_ka;
      core2.P = makeHostP(99) * 25.0;
      ASSERT_TRUE(init.activate(a, core2, &aug2, &why)) << why;
      LinPoint lp;
      lp.x = &core2;
      lp.bridge = &r.bridge_ka;
      lp.aug = &aug2;
      lp.leveling = &lev2;
      lp.anchor = &a.anchor;
      lp.lever_I = kLever;
      lp.pivot_W = a.pivot_W;
      lp.delta = r.bridge_ka.delta;
      const AntennaJacobians J2 = rb.antennaJacobians(lp);
      const Eigen::Matrix3d V2 = J2.J_a * aug2.jointCov(core2.P) * J2.J_a.transpose();
      EXPECT_LT((V2 - Vexp).norm(), 1e-9 * Vexp.norm());
      // PSD joint covariance
      Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(aug2.jointCov(core2.P));
      EXPECT_GE(es.eigenvalues().minCoeff(), -1e-9 * es.eigenvalues().maxCoeff());
    }
    // cross-covariance t–ψ of the block = J_tψ σ_ψ² (the design's "−J_ψσ_ψ²" with the batch's own sensitivity)
    const int ot = aug.find(Slot::Trans), oy = aug.find(Slot::Yaw);
    EXPECT_LT((aug.Q().block<3, 1>(ot, oy) - a.J_trans_yaw * a.sigma_yaw * a.sigma_yaw).norm(), 1e-15 + 1e-12 * a.J_trans_yaw.norm());
    // host cross-covariance of t = −J_a,host P (C column of t)
    EXPECT_LT((aug.C().middleCols<3>(ot) - (P * a.J_trans_host.transpose())).norm(), 1e-12 * (P.norm() * a.J_trans_host.norm() + 1e-30));
  }

  // ---------- noise-free, N frame from the window (first activation fixes r_0 = robust mean of the SPPs) ----------
  {
    Params p;
    World world(p, nav);
    Config cfg;
    Leveling lev;
    Initializer init(cfg, lev);
    AugBlock aug(cfg);
    const RunOut r = runFacade(world, cfg, lev, init, aug, makeHostP(2), 40.0);
    ASSERT_TRUE(r.activated);
    const AlignResult& a = r.align;
    ASSERT_TRUE(init.anchorFixed());
    // r_0 = component-wise median of the window's (exact) SPP positions: inside the window's bounding box
    Eigen::Vector3d lo = Eigen::Vector3d::Constant(1e300), hi = Eigen::Vector3d::Constant(-1e300);
    for (double tau : r.fed_tau) {
      if (tau > r.tau_ka) continue;
      if (tau < r.tau_ka - init.maxWindowSpan() - 1e-6) continue;
      const Eigen::Vector3d rA = world.anchorTrue().r0_ecef + world.anchorTrue().R_EN * world.antennaN(tau);
      lo = lo.cwiseMin(rA);
      hi = hi.cwiseMax(rA);
    }
    for (int i = 0; i < 3; ++i) {
      EXPECT_GE(a.anchor.r0_ecef(i), lo(i) - 1e-6);
      EXPECT_LE(a.anchor.r0_ecef(i), hi(i) + 1e-6);
    }
    // truth expressed in the estimated N frame: a^N = R_ENᵀ(a^E − r_0), ψ from R_EN,estᵀ R_EN,true R_NW Cᵀ (the verticals
    // of the two ENU frames differ by ~|r_0 − site|/R_E ≈ 1e-6 rad; that tilt is left to φ and limits t to ~1e-5·|y − o|)
    const Eigen::Matrix3d C = lev.C(world.gW());
    const Eigen::Matrix3d Mt = a.anchor.R_EN.transpose() * world.anchorTrue().R_EN * world.R_NW() * C.transpose();
    const Eigen::Vector3d aE = world.anchorTrue().r0_ecef + world.anchorTrue().R_EN * world.antennaN(r.tau_ka);
    const Eigen::Vector3d t_true = a.anchor.R_EN.transpose() * (aE - a.anchor.r0_ecef);
    EXPECT_LT(std::fabs(wrap(a.yaw - yawOf(Mt))), 1e-6);
    EXPECT_LT((a.trans - t_true).norm(), 2e-3);
    EXPECT_NEAR(a.b, world.clockBias(a.t_k - kH0), 2e-3);
    EXPECT_NEAR(a.isb[0], p.isb_C, 2e-3);
    EXPECT_NEAR(a.isb[1], p.isb_E, 2e-3);
  }

  // ---------- realistic noise: every error inside 4σ of the activated covariance ----------
  for (uint32_t seed : {11u, 12u, 13u}) {
    Params p;
    p.noise = true;
    p.seed = seed;
    World world(p, nav);
    Config cfg;
    Leveling lev;
    Initializer init(cfg, lev);
    AugBlock aug(cfg);
    const RunOut r = runFacade(world, cfg, lev, init, aug, makeHostP(seed), 40.0, true, true);
    ASSERT_TRUE(r.activated) << "seed " << seed << ": " << (r.align_failures.empty() ? r.activate_why : r.align_failures.back());
    const AlignResult& a = r.align;
    const Eigen::Matrix3d C = lev.C(world.gW());
    const double psi_true = yawOf(world.R_NW() * C.transpose());
    const Eigen::Vector3d t_true = world.antennaN(r.tau_ka);
    const double b_true = world.clockBias(a.t_k - kH0);
    const double s_psi = std::sqrt(qOf(aug, Slot::Yaw, 0));
    const Eigen::Vector3d s_t(std::sqrt(a.cov_trans(0, 0)), std::sqrt(a.cov_trans(1, 1)), std::sqrt(a.cov_trans(2, 2)));
    std::cout << "[U13] seed " << seed << ": ψ err " << wrap(a.yaw - psi_true) / kDeg << "° (σ " << s_psi / kDeg << "°, σ̂0 "
              << a.sigma0_dopp << "), t err " << (a.trans - t_true).transpose() << " m (σ " << s_t.transpose() << ", unfloored "
              << a.cov_noise_formal.diagonal().head<3>().cwiseSqrt().transpose() << "; σ̂ "
              << a.rms_psr_m << " m, " << a.rejected_psr << " rejected), b err " << a.b - b_true << " m (σ "
              << std::sqrt(qOf(aug, Slot::Clk, 0, 0)) << "), d err " << a.d - p.d0 << " (σ " << std::sqrt(qOf(aug, Slot::Clk, 0, 1))
              << "), ι_C/E err " << a.isb[0] - p.isb_C << " / " << a.isb[1] - p.isb_E << "\n";
    EXPECT_LT(s_psi, 1.0 * kDeg);
    EXPECT_LT(std::fabs(wrap(a.yaw - psi_true)), 4.0 * s_psi);
    for (int i = 0; i < 3; ++i) EXPECT_LT(std::fabs(a.trans(i) - t_true(i)), 4.0 * s_t(i)) << "axis " << i;
    EXPECT_LT((a.trans - t_true).norm(), 2.0) << "code-level accuracy expected";
    EXPECT_LT(std::fabs(a.b - b_true), 4.0 * std::sqrt(qOf(aug, Slot::Clk, 0, 0)));
    EXPECT_LT(std::fabs(a.d - p.d0), 4.0 * std::sqrt(qOf(aug, Slot::Clk, 0, 1)));
    EXPECT_LT(std::fabs(a.isb[0] - p.isb_C), 4.0 * std::sqrt(qOf(aug, Slot::Isb, sysBit(Sys::BDS))));
    EXPECT_LT(std::fabs(a.isb[1] - p.isb_E), 4.0 * std::sqrt(qOf(aug, Slot::Isb, sysBit(Sys::GAL))));
    EXPECT_GE(s_t.minCoeff(), cfg.init.spp_sigma_floor_m - 1e-12) << "§8: SPP position variance floored at 3² m²";
  }

  // ---------- an enabled system without data: ι = 0, σ = 30 m (A35); L-PVT: no clock/ISB slots ----------
  {
    Params p;
    p.use_bds = false;
    World world(p, nav);
    Config cfg;
    Leveling lev;
    Initializer init(cfg, lev);
    AugBlock aug(cfg);
    const RunOut r = runFacade(world, cfg, lev, init, aug, makeHostP(3), 40.0, true, true);
    ASSERT_TRUE(r.activated);
    EXPECT_FALSE(r.align.isb_observed[0]);
    EXPECT_TRUE(r.align.isb_present[0]);
    EXPECT_EQ(xOf(aug, Slot::Isb, sysBit(Sys::BDS)), 0.0);
    EXPECT_NEAR(std::sqrt(qOf(aug, Slot::Isb, sysBit(Sys::BDS))), 30.0, 1e-12);
    EXPECT_NEAR(r.align.isb[1], p.isb_E, 1e-5);
  }
  {
    Params p;
    World world(p, nav);
    Config cfg;
    cfg.mode = Config::Mode::LoosePvt;
    Leveling lev;
    Initializer init(cfg, lev);
    AugBlock aug(cfg);
    const RunOut r = runFacade(world, cfg, lev, init, aug, makeHostP(4), 40.0, true, true);
    ASSERT_TRUE(r.activated);
    EXPECT_TRUE(aug.has(Slot::Tilt) && aug.has(Slot::Yaw) && aug.has(Slot::Trans));
    EXPECT_FALSE(aug.has(Slot::Clk));
    EXPECT_EQ(aug.dim(), 6);
  }
}

// =============================================================================================
// U13-1b: statistical consistency of ALIGN (20 seeds, white noise whose variance the a-priori model states exactly, a
// deterministic receiver clock): the NEES of e = [ψ, t, b, d, ι_C, ι_E] against the covariance the alignment reports
// (J_yaw σ_ψ² J_yawᵀ + J_tilt σ_DOV² J_tiltᵀ + the unfloored batch noise) is χ²₈ on average.
// =============================================================================================
TEST(U13Init, AlignmentIsStatisticallyConsistent) {
  const std::string brdc = devSetBrdc();
  if (brdc.empty()) GTEST_SKIP() << "blocked: dev-set fixture with brdc/*.rnx (A24) under " << LIVO_GNSS_TEST_DATA_DIR;
  rtk::NavStore nav;
  std::string why;
  ASSERT_GT(nav.loadRinexNav(brdc, true, &why), 0) << brdc << ": " << why;
  const int runs = 20, dim = 8;
  double nees_sum = 0.0, nees_psi = 0.0;
  Eigen::VectorXd err_sum = Eigen::VectorXd::Zero(dim);
  for (int run = 0; run < runs; ++run) {
    Params p;
    p.noise = true;
    p.seed = 100 + run;
    p.psr_bias = 0.0;
    p.var_matches_noise = true;
    World world(p, nav);
    Config cfg;
    cfg.process.clock.S_b = cfg.process.clock.S_d = 1e-12;   // the world's clock is deterministic
    Leveling lev;
    Initializer init(cfg, lev);
    AugBlock aug(cfg);
    const RunOut r = runFacade(world, cfg, lev, init, aug, makeHostP(run), 40.0, true, true);
    ASSERT_TRUE(r.activated) << "run " << run;
    const AlignResult& a = r.align;
    ASSERT_EQ(a.cov_noise_formal.rows(), 7) << "t, b, d, ι_C, ι_E";
    const Eigen::Matrix3d C = lev.C(world.gW());
    Eigen::VectorXd e(dim);
    e(0) = wrap(a.yaw - yawOf(world.R_NW() * C.transpose()));
    e.segment<3>(1) = a.trans - world.antennaN(r.tau_ka);
    e(4) = a.b - world.clockBias(a.t_k - kH0);
    e(5) = a.d - p.d0;
    e(6) = a.isb[0] - p.isb_C;
    e(7) = a.isb[1] - p.isb_E;
    Eigen::MatrixXd S = Eigen::MatrixXd::Zero(dim, dim);
    const double s2 = a.sigma_yaw * a.sigma_yaw, sd2 = tiltPriorVar(cfg);
    S(0, 0) = s2;
    S.block(1, 0, 7, 1) = a.J_noise_yaw * s2;
    S.block(0, 1, 1, 7) = a.J_noise_yaw.transpose() * s2;
    S.block(1, 1, 7, 7) = a.J_noise_yaw * s2 * a.J_noise_yaw.transpose() + a.J_noise_tilt * sd2 * a.J_noise_tilt.transpose() + a.cov_noise_formal;
    const double nees = e.dot(S.ldlt().solve(e));
    nees_sum += nees;
    nees_psi += e(0) * e(0) / s2;
    for (int i = 0; i < dim; ++i) err_sum(i) += e(i) / std::sqrt(S(i, i));
  }
  const double mean = nees_sum / runs;
  const double lo = chi2Quantile(runs * dim, 0.005) / runs, hi = chi2Quantile(runs * dim, 0.995) / runs;
  std::cout << "[U13] ALIGN consistency over " << runs << " runs: mean NEES " << mean << " (χ²₈ 99 % band of the mean [" << lo << ", "
            << hi << "]), mean ψ NEES " << nees_psi / runs << ", mean normalised errors " << (err_sum / runs).transpose() << "\n";
  EXPECT_GE(mean, lo);
  EXPECT_LE(mean, hi);
  // no component biased: the mean normalised error of each is inside ±3/√runs
  for (int i = 0; i < dim; ++i) EXPECT_LT(std::fabs(err_sum(i) / runs), 3.0 / std::sqrt(static_cast<double>(runs))) << "component " << i;
}

// =============================================================================================
// U13-2: standstill / insufficient excitation keeps the machine in COLLECT
// =============================================================================================
TEST(U13Init, StillOrInsufficientExcitationKeepsWaiting) {
  const std::string brdc = devSetBrdc();
  if (brdc.empty()) GTEST_SKIP() << "blocked: dev-set fixture with brdc/*.rnx (A24) under " << LIVO_GNSS_TEST_DATA_DIR;
  rtk::NavStore nav;
  std::string why;
  ASSERT_GT(nav.loadRinexNav(brdc, true, &why), 0) << brdc << ": " << why;
  struct Case {
    const char* name;
    double t_static, speed;
    double duration;
  };
  // standstill for 45 s; a 0.3 m/s creep (below min_hspeed 0.5); a start 2 s before the end (< 30 moving epochs, < 5 m)
  const Case cases[] = {{"standstill", 100.0, 1.5, 45.0}, {"creep 0.3 m/s", 1.0, 0.3, 45.0}, {"2 s of motion", 40.0, 1.5, 42.0}};
  for (const Case& c : cases) {
    Params p;
    p.t_static = c.t_static;
    p.speed = c.speed;
    p.t_ramp = 1.0;
    World world(p, nav);
    Config cfg;
    Leveling lev;
    Initializer init(cfg, lev);
    AugBlock aug(cfg);
    const RunOut r = runFacade(world, cfg, lev, init, aug, makeHostP(5), c.duration);
    EXPECT_FALSE(r.activated) << c.name;
    EXPECT_EQ(init.phase(), Phase::COLLECT) << c.name;
    EXPECT_TRUE(r.align_failures.empty()) << c.name << ": ALIGN must not even be tried";
    EXPECT_TRUE(init.windowReady()) << c.name;
    EXPECT_FALSE(init.movingEnough()) << c.name;
    EXPECT_EQ(aug.dim(), 0) << c.name;
    // the window slides: never longer than maxWindowSpan()
    EXPECT_LE(init.windowSpan(), init.maxWindowSpan() + 1e-6) << c.name;
    // tryAlign called anyway refuses and says why
    lev.setRef(world.gW());
    const AlignResult a = init.tryAlign(world.host(kH0 + c.duration, makeHostP(5)), 0.0, 0.02);
    EXPECT_FALSE(a.ok) << c.name;
    EXPECT_NE(a.reason.find("insufficient motion"), std::string::npos) << c.name << ": " << a.reason;
    std::cout << "[U13] " << c.name << ": " << a.reason << "\n";
  }
  // too short a window: 30 epochs are there after 3 s but the 10 s span is not
  {
    Params p;
    p.t_static = 0.0;
    p.t_ramp = 0.5;
    World world(p, nav);
    Config cfg;
    Leveling lev;
    Initializer init(cfg, lev);
    AugBlock aug(cfg);
    const RunOut r = runFacade(world, cfg, lev, init, aug, makeHostP(6), 8.0);
    EXPECT_FALSE(r.activated);
    EXPECT_GE(init.windowSize(), cfg.init.min_epochs);
    EXPECT_FALSE(init.windowReady());
    EXPECT_EQ(init.phase(), Phase::COLLECT);
  }
  // the machine before COLLECT: WAIT_CORE until 50 host frames, WAIT_NAV until one healthy satellite (A51)
  {
    Config cfg;
    Leveling lev;
    Initializer init(cfg, lev);
    EXPECT_EQ(init.phase(), Phase::OFF);
    init.onLioFrame(false, false);
    EXPECT_EQ(init.phase(), Phase::WAIT_CORE);
    for (int f = 0; f < cfg.init.min_livo_frames - 1; ++f) init.onLioFrame(true, true);
    EXPECT_EQ(init.phase(), Phase::WAIT_CORE);
    init.onLioFrame(true, true);
    EXPECT_EQ(init.phase(), Phase::WAIT_NAV);
    init.onNavCount(0);
    EXPECT_EQ(init.phase(), Phase::WAIT_NAV);
    init.onNavCount(1);   // A51: one healthy satellite starts COLLECT (the §8 window still needs min_sats per sample)
    EXPECT_EQ(init.phase(), Phase::COLLECT);
    init.onLioFrame(false, true);   // host lost its initialisation
    EXPECT_EQ(init.phase(), Phase::WAIT_CORE);
    CollectSample s;
    init.addSample(std::move(s));
    EXPECT_EQ(init.samplesRefused(), 1);
    EXPECT_EQ(init.windowSize(), 0);
    Initializer obs(cfg, lev);
    obs.setPhase(Phase::OBSERVE);   // OBSERVE is a start phase: the machine runs, the updater does not write
    obs.onLioFrame(true, true);
    EXPECT_EQ(obs.phase(), Phase::WAIT_CORE);
  }
}

// =============================================================================================
// U13-3: epochs used by the initialisation are not re-used as updates
// =============================================================================================
TEST(U13Init, InitEpochsNotDoubleCounted) {
  const std::string brdc = devSetBrdc();
  if (brdc.empty()) GTEST_SKIP() << "blocked: dev-set fixture with brdc/*.rnx (A24) under " << LIVO_GNSS_TEST_DATA_DIR;
  rtk::NavStore nav;
  std::string why;
  ASSERT_GT(nav.loadRinexNav(brdc, true, &why), 0) << brdc << ": " << why;
  Params p;
  World world(p, nav);
  Config cfg;
  Leveling lev;
  Initializer init(cfg, lev);
  AugBlock aug(cfg);
  const RunOut r = runFacade(world, cfg, lev, init, aug, makeHostP(7), 40.0);
  ASSERT_TRUE(r.activated);
  const AlignResult& a = r.align;
  // k_a is the last epoch handed over; every epoch up to it is consumed, none after it
  ASSERT_FALSE(r.fed.empty());
  EXPECT_EQ(a.t_activate, r.fed.back());
  for (const GTime& t : r.fed) EXPECT_TRUE(init.consumedByInit(t));
  EXPECT_FALSE(init.consumedByInit(a.t_activate.add(0.1)));
  EXPECT_EQ(init.windowSize(), 0) << "the window is spent at a successful ALIGN";
  // an activation on a block that still carries a frame is refused (REINIT must drop the slots first)
  {
    std::string w2;
    EXPECT_FALSE(init.activate(a, r.core_ka, &aug, &w2));
    EXPECT_NE(w2.find("already present"), std::string::npos) << w2;
  }
  // REINIT: slots dropped, back to COLLECT; the old epochs are refused, new ones collected; N is kept
  const AnchorFrame n_before = init.anchorFrame();
  aug.clear();
  init.setPhase(Phase::COLLECT);
  const int refused0 = init.samplesRefused();
  {
    CollectSample old;
    old.t_rx = a.t_activate;
    old.spp.ok = true;
    old.spp.n_used = 20;
    old.spp.gdop = 1.5;
    old.bridge.ok = true;
    init.addSample(std::move(old));
    EXPECT_EQ(init.samplesRefused(), refused0 + 1);
    EXPECT_NE(init.lastRefusal().find("k_a"), std::string::npos) << init.lastRefusal();
    EXPECT_EQ(init.windowSize(), 0);
    CollectSample fresh;
    fresh.t_rx = a.t_activate.add(0.1);
    fresh.spp.ok = true;
    fresh.spp.n_used = 20;
    fresh.spp.gdop = 1.5;
    fresh.bridge.ok = true;
    init.addSample(std::move(fresh));
    EXPECT_EQ(init.windowSize(), 1);
  }
  EXPECT_TRUE(init.anchorFixed());
  EXPECT_EQ(init.anchorFrame().r0_ecef, n_before.r0_ecef);
}

// =============================================================================================
// U13-4 (§7 time check in ALIGN): a residual GNSS/host lag is estimated from the speed series; > 3σ ⇒ offset_warn
// =============================================================================================
TEST(U13Init, TimeCheckFlagsOffset) {
  const std::string brdc = devSetBrdc();
  if (brdc.empty()) GTEST_SKIP() << "blocked: dev-set fixture with brdc/*.rnx (A24) under " << LIVO_GNSS_TEST_DATA_DIR;
  rtk::NavStore nav;
  std::string why;
  ASSERT_GT(nav.loadRinexNav(brdc, true, &why), 0) << brdc << ": " << why;
  for (double lag : {0.0, 0.15}) {
    Params p;
    p.lag_s = lag;
    p.noise = true;
    p.seed = 21;
    p.psr_bias = 0.0;
    World world(p, nav);
    Config cfg;
    Leveling lev;
    Initializer init(cfg, lev);
    AugBlock aug(cfg);
    const RunOut r = runFacade(world, cfg, lev, init, aug, makeHostP(8), 40.0, true, true);
    ASSERT_TRUE(r.activated) << "lag " << lag << ": " << (r.align_failures.empty() ? r.activate_why : r.align_failures.back());
    const AlignResult& a = r.align;
    ASSERT_TRUE(a.offset_check_valid);
    std::cout << "[U13] time check, injected lag " << lag << " s: online residual " << a.offset_check_s << " ± "
              << a.offset_check_sigma_s << " s, warn " << a.offset_warn << "\n";
    // the labels are lag_s early ⇒ the GNSS epoch really happened lag_s after its label: +lag
    EXPECT_LT(std::fabs(a.offset_check_s - lag), 5.0 * a.offset_check_sigma_s + 0.01);
    EXPECT_EQ(a.offset_warn, lag > 0.0);
  }
}

// =============================================================================================
// U13-5: the Procrustes start is exact on rotated velocities
// =============================================================================================
TEST(U13Init, ProcrustesYawExact) {
  std::mt19937 rng(3);
  std::normal_distribution<double> n(0.0, 1.0);
  for (double psi : {0.0, 0.3, -1.2, 3.0, -3.1}) {
    std::vector<Eigen::Vector2d> w, v;
    const Eigen::Rotation2Dd R(psi);
    for (int i = 0; i < 20; ++i) {
      w.emplace_back(n(rng), n(rng));
      v.push_back(R * w.back());
    }
    EXPECT_NEAR(wrap(Initializer::procrustesYaw(v, w) - psi), 0.0, 1e-12) << psi;
  }
}

// =============================================================================================
// A51 (P4-INIT): the LIO-aided multi-epoch path on a few-satellite sky
// =============================================================================================
namespace {

// The few-satellite sky of A51's U13 case: 5 GPS + 2 Galileo satellites of the fixture's real broadcast orbits, each
// epoch seeing 1–3 of them (so the GNSS-only SPP never solves), on the 12 m arc at 1.5 m/s after a 4 s standstill.
Params fewSatParams() {
  Params p;
  p.pool_gps = 5;
  p.pool_gal = 2;
  p.use_bds = false;   // BDS slot enabled but unobserved (σ = 30 m), GLONASS code off
  p.use_glo = false;
  p.duration = 70.0;
  return p;
}

// Joint covariance of e = [ψ, t, b, d, ι_obs…] that an A51 alignment reports (tilt prior mapped through J_φ): with the
// unfloored batch noise (the estimator's own statistics) or with the §8-floored one activate() augments with.
Eigen::MatrixXd laJointCov(const AlignResult& a, double sdov, bool floored = false) {
  const Eigen::MatrixXd& Sn = floored ? a.cov_noise : a.cov_noise_formal;
  const int dn = static_cast<int>(Sn.rows());
  const int dim = 1 + dn;
  const double s2 = a.sigma_yaw * a.sigma_yaw, sd2 = sdov * sdov;
  Eigen::MatrixXd S = Eigen::MatrixXd::Zero(dim, dim);
  S(0, 0) = s2 + a.J_yaw_tilt.squaredNorm() * sd2;
  const Eigen::VectorXd cross = a.J_noise_yaw * s2 + a.J_noise_tilt * sd2 * a.J_yaw_tilt.transpose();
  S.block(1, 0, dn, 1) = cross;
  S.block(0, 1, 1, dn) = cross.transpose();
  S.block(1, 1, dn, dn) = a.J_noise_yaw * s2 * a.J_noise_yaw.transpose() + a.J_noise_tilt * sd2 * a.J_noise_tilt.transpose() + Sn;
  return S;
}

}  // namespace

TEST(U13Init, LioAidedFewSatellitesRecoversPsiTBD) {
  const std::string brdc = devSetBrdc();
  if (brdc.empty()) GTEST_SKIP() << "blocked: dev-set fixture with brdc/*.rnx (A24) under " << LIVO_GNSS_TEST_DATA_DIR;
  rtk::NavStore nav;
  std::string why;
  ASSERT_GT(nav.loadRinexNav(brdc, true, &why), 0) << brdc << ": " << why;

  // ---------- noise-free, N frame fixed to the truth ----------
  {
    const Params p = fewSatParams();
    World world(p, nav);
    ASSERT_EQ(world.poolSize(), 7) << "the fixture's sky must offer 5 GPS + 2 Galileo satellites above 15°";
    Config cfg;
    Leveling lev;
    Initializer init(cfg, lev);
    AugBlock aug(cfg);
    const HostMat P = makeHostP(31);
    const RunOut r = runFacade(world, cfg, lev, init, aug, P, p.duration, true, true);
    ASSERT_TRUE(r.activated) << "no activation; last ALIGN failure: " << (r.align_failures.empty() ? "-" : r.align_failures.back())
                             << "; activate: " << r.activate_why;
    const AlignResult& a = r.align;
    EXPECT_EQ(r.spp_epochs, 0) << "1–3 satellites per epoch: the §8 path never has a sample";
    EXPECT_EQ(a.path, "lio_aided");
    EXPECT_EQ(init.phase(), Phase::TRACKING);
    EXPECT_LE(r.max_la_span, Initializer::kLaMaxSpanS + 1e-9);
    const Eigen::Matrix3d C = lev.C(world.gW());
    const double psi_true = yawOf(world.R_NW() * C.transpose());
    const Eigen::Vector3d t_true = world.antennaN(r.tau_ka);
    const double b_true = world.clockBias(a.t_k - kH0);
    std::cout << "[U13] A51 noise-free: activation at τ = " << r.tau_ka << " s after " << r.align_failures.size()
              << " failed attempts (first: " << (r.align_failures.empty() ? "-" : r.align_failures.front()) << ")\n"
              << "[U13]   window " << a.epochs_used << " epochs / " << a.window_s << " s, " << a.rows_dopp << " Doppler + " << a.rows_psr
              << " code rows from " << a.n_sats << " satellites, redundancy " << a.redundancy << ", σ_ψ " << a.sigma_yaw / kDeg
              << "°, σ_t,max " << a.sigma_trans_max_m << " m\n"
              << "[U13]   ψ err " << wrap(a.yaw - psi_true) << " rad, |t err| " << (a.trans - t_true).norm() << " m, b err " << a.b - b_true
              << " m, d err " << a.d - p.d0 << " m/s, ι_E err " << a.isb[1] - p.isb_E << " m\n";
    EXPECT_LE(a.sigma_yaw, cfg.init.yaw_sigma_max_deg * kDeg);
    EXPECT_LE(a.sigma_trans_max_m, Initializer::kLaMaxTransSigmaM);
    EXPECT_GE(a.redundancy, Initializer::kLaMinRedundancy);
    EXPECT_LT(std::fabs(wrap(a.yaw - psi_true)), 2e-7);
    EXPECT_LT((a.trans - t_true).norm(), 1e-5);
    EXPECT_NEAR(a.b, b_true, 1e-5);
    EXPECT_NEAR(a.d, p.d0, 1e-6);
    EXPECT_TRUE(a.isb_observed[1]) << "ι_E is seen through the clock model, not through simultaneous GPS rows";
    EXPECT_NEAR(a.isb[1], p.isb_E, 1e-5);
    EXPECT_FALSE(a.isb_observed[0]);
    EXPECT_TRUE(a.isb_present[0]);
    EXPECT_NEAR(std::sqrt(qOf(aug, Slot::Isb, sysBit(Sys::BDS))), 30.0, 1e-12);
    EXPECT_EQ(a.anchor.r0_ecef, world.anchorTrue().r0_ecef);
    // slot values = the alignment
    EXPECT_EQ(xOf(aug, Slot::Yaw, 0), a.yaw);
    EXPECT_EQ(xOf(aug, Slot::Clk, 0, 0), a.b);
    EXPECT_EQ(xOf(aug, Slot::Clk, 0, 1), a.d);
    EXPECT_EQ(xOf(aug, Slot::Isb, sysBit(Sys::GAL)), a.isb[1]);
    // every epoch handed over up to k_a is consumed
    EXPECT_EQ(a.t_activate, r.fed.back());
    for (const GTime& t : r.fed) EXPECT_TRUE(init.consumedByInit(t));
    EXPECT_EQ(init.laWindowSize(), 0);

    // Consistent augmentation (§8 ACTIVATE, A51 "同 §8"): the block's (φ, ψ, t, b, d, ι) covariance is the joint LS's
    // Σ_θ + J_φσ_DOV²J_φᵀ; the global antenna at k_a has the GNSS-only covariance for any host P and no host correlation.
    NoiseModel nm(cfg);
    RowBuilder rb(cfg, nm);
    const double sd2 = tiltPriorVar(cfg);
    const Eigen::MatrixXd Sj = laJointCov(a, std::sqrt(sd2), true);
    const Eigen::Matrix3d Vexp = Sj.block<3, 3>(1, 1);
    auto globalAntenna = [&](const CoreState& core, const AugBlock& blk, const Leveling& lv) {
      LinPoint lp;
      lp.x = &core;
      lp.bridge = &r.bridge_ka;
      lp.aug = &blk;
      lp.leveling = &lv;
      lp.anchor = &a.anchor;
      lp.lever_I = kLever;
      lp.pivot_W = a.pivot_W;
      lp.gravity_est_en = true;
      lp.delta = r.bridge_ka.delta;
      const AntennaJacobians J = rb.antennaJacobians(lp);
      const Eigen::MatrixXd S = blk.jointCov(core.P);
      return std::make_tuple(J.a_N, Eigen::Matrix3d(J.J_a * S * J.J_a.transpose()), Eigen::MatrixXd(J.J_a * S.leftCols(HostIdx::dim)));
    };
    {
      const auto [aN, V, X] = globalAntenna(r.core_ka, aug, lev);
      EXPECT_LT((aN - a.trans).norm(), 1e-9) << "y(core) = o at k_a ⇒ a^N = t";
      EXPECT_LT((V - Vexp).norm(), 1e-9 * Vexp.norm()) << "V\n" << V << "\nexpected\n" << Vexp;
      EXPECT_LT(X.cwiseAbs().maxCoeff(), 1e-9 * std::sqrt(Vexp.norm() * P.norm())) << "global antenna correlated with the host";
    }
    {
      AugBlock aug2(cfg);
      CoreState core2 = r.core_ka;
      core2.P = makeHostP(77) * 16.0;
      ASSERT_TRUE(init.activate(a, core2, &aug2, &why)) << why;
      const auto [aN2, V2, X2] = globalAntenna(core2, aug2, lev);
      EXPECT_LT((V2 - Vexp).norm(), 1e-9 * Vexp.norm());
      Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(aug2.jointCov(core2.P));
      EXPECT_GE(es.eigenvalues().minCoeff(), -1e-9 * es.eigenvalues().maxCoeff());
    }
    // (φ, ψ, t) block of the new slots = the joint LS covariance mapped through J_φ
    const int ot = aug.find(Slot::Trans), oy = aug.find(Slot::Yaw), ophi = aug.find(Slot::Tilt);
    EXPECT_NEAR(aug.Q()(oy, oy), Sj(0, 0), 1e-15 + 1e-12 * Sj(0, 0));
    const Eigen::Vector3d q_ty = aug.Q().block<3, 1>(ot, oy), s_ty = Sj.block<3, 1>(1, 0);
    EXPECT_LT((q_ty - s_ty).norm(), 1e-15 + 1e-12 * s_ty.norm());
    const Eigen::RowVector2d q_yphi = aug.Q().block<1, 2>(oy, ophi);
    EXPECT_LT((q_yphi - a.J_yaw_tilt * sd2).norm(), 1e-20 + 1e-12 * a.J_yaw_tilt.norm() * sd2);
  }

  // ---------- noise-free cold start: no fix ever, the window fixes N ----------
  {
    const Params p = fewSatParams();
    World world(p, nav);
    Config cfg;
    Leveling lev;
    Initializer init(cfg, lev);
    AugBlock aug(cfg);
    const RunOut r = runFacade(world, cfg, lev, init, aug, makeHostP(32), p.duration);
    ASSERT_TRUE(r.activated) << (r.align_failures.empty() ? r.activate_why : r.align_failures.back());
    const AlignResult& a = r.align;
    EXPECT_EQ(a.path, "lio_aided");
    ASSERT_TRUE(init.anchorFixed());
    // r_0 = the coarse static fix of the window: within the few tens of metres of the arc
    EXPECT_LT((a.anchor.r0_ecef - world.anchorTrue().r0_ecef).norm(), 100.0);
    // truth in the estimated N frame (the two ENU verticals differ by |r_0 − site|/R_E ≲ 2e-5 rad, left to φ)
    const Eigen::Matrix3d C = lev.C(world.gW());
    const Eigen::Matrix3d Mt = a.anchor.R_EN.transpose() * world.anchorTrue().R_EN * world.R_NW() * C.transpose();
    const Eigen::Vector3d aE = world.anchorTrue().r0_ecef + world.anchorTrue().R_EN * world.antennaN(r.tau_ka);
    const Eigen::Vector3d t_true = a.anchor.R_EN.transpose() * (aE - a.anchor.r0_ecef);
    std::cout << "[U13] A51 cold start: |r_0 − site| " << (a.anchor.r0_ecef - world.anchorTrue().r0_ecef).norm() << " m, ψ err "
              << wrap(a.yaw - yawOf(Mt)) << " rad, |t err| " << (a.trans - t_true).norm() << " m\n";
    EXPECT_LT(std::fabs(wrap(a.yaw - yawOf(Mt))), 5e-5);
    EXPECT_LT((a.trans - t_true).norm(), 5e-3);
    EXPECT_NEAR(a.b, world.clockBias(a.t_k - kH0), 5e-3);
    EXPECT_NEAR(a.d, p.d0, 1e-4);
    EXPECT_NEAR(a.isb[1], p.isb_E, 5e-3);
  }

  // ---------- a 1 ms receiver clock jump inside the window: restart, then exact again ----------
  {
    Params p = fewSatParams();
    p.jump_tau = 8.0;   // inside the first window (without the jump the path activates at τ ≈ 10 s)
    p.jump_m = kC * 1e-3;
    World world(p, nav);
    Config cfg;
    Leveling lev;
    Initializer init(cfg, lev);
    AugBlock aug(cfg);
    const RunOut r = runFacade(world, cfg, lev, init, aug, makeHostP(33), p.duration, true, true);
    ASSERT_TRUE(r.activated) << (r.align_failures.empty() ? r.activate_why : r.align_failures.back());
    const AlignResult& a = r.align;
    EXPECT_EQ(init.laRestarts(), 1);
    EXPECT_GE(r.tau_ka, p.jump_tau + cfg.init.min_window_s) << "the window restarted at the jump";
    const Eigen::Matrix3d C = lev.C(world.gW());
    std::cout << "[U13] A51 clock jump at τ = " << p.jump_tau << " s: restarts " << init.laRestarts() << ", activation at τ = " << r.tau_ka
              << " s, b err " << a.b - world.clockBias(a.t_k - kH0) << " m\n";
    EXPECT_LT(std::fabs(wrap(a.yaw - yawOf(world.R_NW() * C.transpose()))), 2e-7);
    EXPECT_LT((a.trans - world.antennaN(r.tau_ka)).norm(), 1e-5);
    EXPECT_NEAR(a.b, world.clockBias(a.t_k - kH0), 1e-5);
  }
}

// 20 noisy runs (white code/Doppler noise whose variance the a-priori model states exactly, a deterministic clock): the
// mean NEES of e = [ψ, t, b, d, ι_E] against the covariance the A51 alignment reports is χ²₇ on average.
TEST(U13Init, LioAidedIsStatisticallyConsistent) {
  const std::string brdc = devSetBrdc();
  if (brdc.empty()) GTEST_SKIP() << "blocked: dev-set fixture with brdc/*.rnx (A24) under " << LIVO_GNSS_TEST_DATA_DIR;
  rtk::NavStore nav;
  std::string why;
  ASSERT_GT(nav.loadRinexNav(brdc, true, &why), 0) << brdc << ": " << why;
  const int runs = 20, dim = 7;
  double nees_sum = 0.0;
  Eigen::VectorXd err_sum = Eigen::VectorXd::Zero(dim);
  double sig_psi = 0.0, sig_t = 0.0;
  for (int run = 0; run < runs; ++run) {
    Params p = fewSatParams();
    p.noise = true;
    p.seed = 200 + run;
    p.psr_bias = 0.0;
    p.var_matches_noise = true;
    World world(p, nav);
    Config cfg;
    cfg.process.clock.S_b = cfg.process.clock.S_d = 1e-12;   // the world's clock is deterministic
    Leveling lev;
    Initializer init(cfg, lev);
    AugBlock aug(cfg);
    const RunOut r = runFacade(world, cfg, lev, init, aug, makeHostP(300 + run), p.duration, true, true);
    ASSERT_TRUE(r.activated) << "run " << run << ": " << (r.align_failures.empty() ? r.activate_why : r.align_failures.back());
    const AlignResult& a = r.align;
    ASSERT_EQ(a.path, "lio_aided");
    ASSERT_EQ(a.cov_noise_formal.rows(), 6) << "t, b, d, ι_E";
    const Eigen::Matrix3d C = lev.C(world.gW());
    Eigen::VectorXd e(dim);
    e(0) = wrap(a.yaw - yawOf(world.R_NW() * C.transpose()));
    e.segment<3>(1) = a.trans - world.antennaN(r.tau_ka);
    e(4) = a.b - world.clockBias(a.t_k - kH0);
    e(5) = a.d - p.d0;
    e(6) = a.isb[1] - p.isb_E;
    const Eigen::MatrixXd S = laJointCov(a, std::sqrt(tiltPriorVar(cfg)));
    nees_sum += e.dot(S.ldlt().solve(e));
    for (int i = 0; i < dim; ++i) err_sum(i) += e(i) / std::sqrt(S(i, i));
    sig_psi += a.sigma_yaw;
    sig_t += a.sigma_trans_max_m;
  }
  const double mean = nees_sum / runs;
  const double lo = chi2Quantile(runs * dim, 0.005) / runs, hi = chi2Quantile(runs * dim, 0.995) / runs;
  std::cout << "[U13] A51 consistency over " << runs << " runs: mean NEES " << mean << " (χ²₇ 99 % band of the mean [" << lo << ", " << hi
            << "]), mean σ_ψ " << sig_psi / runs / kDeg << "°, mean σ_t,max " << sig_t / runs << " m, mean normalised errors "
            << (err_sum / runs).transpose() << "\n";
  EXPECT_GE(mean, lo);
  EXPECT_LE(mean, hi);
  for (int i = 0; i < dim; ++i) EXPECT_LT(std::fabs(err_sum(i) / runs), 3.0 / std::sqrt(static_cast<double>(runs))) << "component " << i;
}

// Data too poor for A51: three distinct satellites in the window (t and b need a fourth line of sight) never activate
// and say why; a few-satellite standstill never even arms the path (no alignment at standstill).
TEST(U13Init, LioAidedInsufficientGeometryKeepsWaiting) {
  const std::string brdc = devSetBrdc();
  if (brdc.empty()) GTEST_SKIP() << "blocked: dev-set fixture with brdc/*.rnx (A24) under " << LIVO_GNSS_TEST_DATA_DIR;
  rtk::NavStore nav;
  std::string why;
  ASSERT_GT(nav.loadRinexNav(brdc, true, &why), 0) << brdc << ": " << why;
  {
    Params p = fewSatParams();
    p.pool_gps = 3;
    p.pool_gal = 0;
    World world(p, nav);
    ASSERT_EQ(world.poolSize(), 3);
    Config cfg;
    Leveling lev;
    Initializer init(cfg, lev);
    AugBlock aug(cfg);
    const RunOut r = runFacade(world, cfg, lev, init, aug, makeHostP(41), p.duration);
    EXPECT_FALSE(r.activated);
    EXPECT_EQ(init.phase(), Phase::COLLECT);
    EXPECT_EQ(aug.dim(), 0);
    ASSERT_FALSE(r.align_failures.empty()) << "the moving window arms the A51 path";
    EXPECT_NE(r.align_failures.back().find("too few distinct code satellites"), std::string::npos) << r.align_failures.back();
    // at most one attempt per kLaRetryS of GPST
    EXPECT_LE(static_cast<double>(r.align_failures.size()), p.duration / Initializer::kLaRetryS + 1.0);
    std::cout << "[U13] A51 three satellites: " << r.align_failures.size() << " attempts, last: " << r.align_failures.back() << "\n";
  }
  {
    Params p = fewSatParams();
    p.t_static = 100.0;   // never moves
    World world(p, nav);
    Config cfg;
    Leveling lev;
    Initializer init(cfg, lev);
    AugBlock aug(cfg);
    const RunOut r = runFacade(world, cfg, lev, init, aug, makeHostP(42), p.duration);
    EXPECT_FALSE(r.activated);
    EXPECT_TRUE(r.align_failures.empty()) << "no alignment at standstill: " << r.align_failures.front();
    EXPECT_FALSE(init.laArmed());
    EXPECT_GE(init.laWindowSize(), cfg.init.min_epochs);
    EXPECT_LE(init.laWindowSpan(), Initializer::kLaMaxSpanS + 1e-9);
  }
}

// A51 model choice (INTERFACE_CHANGES P4-INIT I-11): the receiver clock drift wanders (M3DGR's receiver: 0.1–0.25 m/s per
// minute, S_d ≈ 1e-3 m²/s³). One drift shared by every Doppler row of a 60 s window (A51 as written) turns that wander into
// a ψ error at walking speed; every epoch's own (eliminated) drift makes ψ immune. Noise-free otherwise, 0.8 m/s: the ψ
// error stays far below its σ for wander up to S_d = 1e-3, t and b within their σ.
TEST(U13Init, LioAidedClockDriftWanderDoesNotBiasYaw) {
  const std::string brdc = devSetBrdc();
  if (brdc.empty()) GTEST_SKIP() << "blocked: dev-set fixture with brdc/*.rnx (A24) under " << LIVO_GNSS_TEST_DATA_DIR;
  rtk::NavStore nav;
  std::string why;
  ASSERT_GT(nav.loadRinexNav(brdc, true, &why), 0) << brdc << ": " << why;
  for (double sd : {1e-4, 1e-3}) {
    for (uint32_t ws : {1u, 2u, 3u, 4u}) {
      Params p = fewSatParams();
      p.drift_rw = sd;
      p.wander_seed = ws;
      p.speed = 0.8;
      World world(p, nav);
      Config cfg;
      Leveling lev;
      Initializer init(cfg, lev);
      AugBlock aug(cfg);
      const RunOut r = runFacade(world, cfg, lev, init, aug, makeHostP(50), p.duration, true, true);
      ASSERT_TRUE(r.activated) << "S_d " << sd << " seed " << ws << ": " << (r.align_failures.empty() ? r.activate_why : r.align_failures.back());
      const AlignResult& a = r.align;
      const Eigen::Matrix3d C = lev.C(world.gW());
      const double e = wrap(a.yaw - yawOf(world.R_NW() * C.transpose()));
      const Eigen::Vector3d et = a.trans - world.antennaN(r.tau_ka);
      std::cout << "[U13] A51 clock wander S_d " << sd << " seed " << ws << ": ψ err " << e / kDeg << "° (σ " << a.sigma_yaw / kDeg
                << "°), |t err| " << et.norm() << " m, b err " << a.b - world.clockBias(a.t_k - kH0) << " m, d err "
                << a.d - world.clockDrift(a.t_k - kH0) << " m/s\n";
      EXPECT_LT(std::fabs(e), 0.05 * a.sigma_yaw);
      for (int i = 0; i < 3; ++i) EXPECT_LT(std::fabs(et(i)), std::sqrt(a.cov_noise_formal(i, i)));
      EXPECT_LT(std::fabs(a.b - world.clockBias(a.t_k - kH0)), 3.0 * std::sqrt(a.cov_noise_formal(3, 3)));
      EXPECT_LT(std::fabs(a.d - world.clockDrift(a.t_k - kH0)), 3.0 * std::sqrt(a.cov_noise_formal(4, 4)) + 0.02);
    }
  }
}

// A51 on rows as the facade delivers them without any fix: el = 90°, no ionosphere/troposphere, not models_fixed, the
// sample carrying the Klobuchar and the receiver stds. The path must re-evaluate elevation, both atmospheric models and
// the §4.7 variances at its own antenna positions: noise-free, the recovery is exact although the delivered rows are
// metres off (the troposphere alone is 2.3 m at the zenith).
TEST(U13Init, LioAidedReevaluatesAtmosphereAndVariances) {
  const std::string brdc = devSetBrdc();
  if (brdc.empty()) GTEST_SKIP() << "blocked: dev-set fixture with brdc/*.rnx (A24) under " << LIVO_GNSS_TEST_DATA_DIR;
  rtk::NavStore nav;
  std::string why;
  ASSERT_GT(nav.loadRinexNav(brdc, true, &why), 0) << brdc << ": " << why;
  Params p = fewSatParams();
  p.atmosphere = true;
  World world(p, nav);
  ASSERT_TRUE(world.haveKlobuchar()) << "the fixture's BRDC header carries GPSA/GPSB (A38)";
  Config cfg;
  Leveling lev;
  Initializer init(cfg, lev);
  AugBlock aug(cfg);
  const RunOut r = runFacade(world, cfg, lev, init, aug, makeHostP(61), p.duration, true, true);
  ASSERT_TRUE(r.activated) << (r.align_failures.empty() ? r.activate_why : r.align_failures.back());
  const AlignResult& a = r.align;
  EXPECT_EQ(a.path, "lio_aided");
  const Eigen::Matrix3d C = lev.C(world.gW());
  const double e_psi = wrap(a.yaw - yawOf(world.R_NW() * C.transpose()));
  const Eigen::Vector3d e_t = a.trans - world.antennaN(r.tau_ka);
  std::cout << "[U13] A51 re-evaluated atmosphere: activation at τ = " << r.tau_ka << " s, ψ err " << e_psi << " rad, |t err| " << e_t.norm()
            << " m, b err " << a.b - world.clockBias(a.t_k - kH0) << " m, ι_E err " << a.isb[1] - p.isb_E << " m (σ_ψ " << a.sigma_yaw / kDeg
            << "°, σ_t,max " << a.sigma_trans_max_m << " m)\n";
  EXPECT_LT(std::fabs(e_psi), 2e-7);
  EXPECT_LT(e_t.norm(), 1e-4);
  EXPECT_NEAR(a.b, world.clockBias(a.t_k - kH0), 1e-4);
  EXPECT_NEAR(a.d, p.d0, 1e-5);
  EXPECT_NEAR(a.isb[1], p.isb_E, 1e-4);
}
