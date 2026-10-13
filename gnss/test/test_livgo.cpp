// SPDX-License-Identifier: BSD-3-Clause
// FAST-LIVGO arm.
//   LivgoDtw / LivgoInit   DTW lag of the arm: a delayed GNSS speed sequence must come back as a negative offset correction
//                          of that delay; the III-C2 least squares. Not filter tests.
//   LivgoDefaults          every A75 switch of LivgoConfig, and the shared pieces they touch (NoiseModel::flat, the legacy pin),
//                          sit at their pre-A75 behaviour by default.
//   LivgoFrameExact        AMENDMENTS A75 §6.1: AugBlock::pinRegistrationExact; the fixed-anchor TDCP gain on the host
//                          against the legacy pin; the yaw lever of a fixed anchor and the shared-anchor columns that remove it
//                          (livgo_anchor.hpp), checked by central differences.
//   LivgoNoise             AMENDMENTS A75 §6.6: flat row variance = σ² + the IMU-bridge terms, in NoiseModel and in the rows
//                          RowBuilder builds from it.
//   LivgoTdcpGain          AMENDMENTS A75 revision 1 (prediction P2): the within-epoch counterfactual that measures the TDCP share
//                          of the host position variance (livgo_gain.hpp), on the toy of LivgoFrameExact: exact frame against the
//                          legacy pin.
//   LivgoA80               AMENDMENTS A80 (LIVGO-literal): the DTW grid/window parameters, the travel history of the frame least
//                          squares, the tilt rule, freeze_yaw (the yaw pin), the hard C/N0 and elevation masks, the primary-signal
//                          rule, the defaults of every A80 switch and the construction-time checks of the facade.
//   LivgoA81               AMENDMENTS A81: livgo.tdcp_raim_both_ends (default, the pair-start selection with the switch off and on,
//                          the facade's acceptance of it).
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include "livo_gnss/diagnostics.hpp"
#include "livo_gnss/gnss_fusion_livgo.hpp"
#include "livo_gnss/livgo.hpp"
#include "livo_gnss/livgo_init.hpp"
#include "internal/aug_block.hpp"
#include "internal/bridge.hpp"
#include "internal/leveling.hpp"
#include "internal/livgo_anchor.hpp"
#include "internal/livgo_gain.hpp"
#include "internal/quality.hpp"
#include "internal/rows.hpp"
#include "internal/update.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>

TEST(LivgoDtw, DelayedGnssSpeedComesBackNegative) {
  const double delay = 0.3;
  std::vector<double> th, vh, tg, vg;
  for (int i = 0; i < 200; ++i) {
    const double t = 0.1 * i;
    th.push_back(t);
    vh.push_back(std::sin(2.0 * M_PI * t / 5.0));
    tg.push_back(t);
    vg.push_back(std::sin(2.0 * M_PI * (t - delay) / 5.0));
  }
  double lag = 99.0;
  ASSERT_TRUE(livo_gnss::livgoDtwLag(tg, vg, th, vh, &lag));
  EXPECT_NEAR(lag, -delay, 0.15);
}

TEST(LivgoInit, LongBaselineRecoversTiltAndLever) {
  const double deg = M_PI / 180.0;
  const Eigen::Matrix3d R = Eigen::AngleAxisd(30.0 * deg, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
                            Eigen::AngleAxisd(0.5 * deg, Eigen::Vector3d::UnitY()).toRotationMatrix();
  const Eigen::Vector3d p(10.0, -5.0, 2.0);
  const Eigen::Vector3d lever(0.30, -0.10, 0.40);
  std::vector<livo_gnss::LivgoInitSample> samples;
  for (int i = 0; i < 80; ++i) {
    livo_gnss::LivgoInitSample s;
    const double yaw = 0.015 * i;
    s.p_I = Eigen::Vector3d(5.0 * i, 8.0 * std::sin(0.05 * i), 0.0);
    s.R_I = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const Eigen::Vector3d noise(0.02 * std::sin(i), 0.02 * std::cos(i * 0.7), 0.01 * std::sin(i * 0.3));
    s.p_r = R * (s.p_I + s.R_I * lever) + p + noise;
    samples.push_back(s);
  }
  Eigen::Vector3d p0 = Eigen::Vector3d::Zero();
  for (const auto& s : samples) p0 += s.p_r - (s.p_I + lever);
  p0 /= static_cast<double>(samples.size());
  livo_gnss::LivgoInitEstimate est;
  ASSERT_TRUE(livo_gnss::livgoInitLeastSquares(samples, Eigen::Matrix3d::Identity(), p0, lever, &est));
  EXPECT_LT((est.R - R).norm(), 1e-3);
  EXPECT_LT((est.lever - lever).norm(), 0.02);
  EXPECT_LT(est.rms_m, 0.05);
  const double sx = std::sqrt(std::max(0.0, est.cov(0, 0))) / deg;
  const double sy = std::sqrt(std::max(0.0, est.cov(1, 1))) / deg;
  EXPECT_LT(std::max(sx, sy), 0.2);
}

TEST(LivgoInit, ShortNoisyWindowDoesNotResolveTilt) {
  const double deg = M_PI / 180.0;
  const Eigen::Matrix3d R = Eigen::AngleAxisd(0.5 * deg, Eigen::Vector3d::UnitY()).toRotationMatrix();
  const Eigen::Vector3d lever(0.30, -0.10, 0.40);
  std::vector<livo_gnss::LivgoInitSample> samples;
  for (int i = 0; i < 20; ++i) {
    livo_gnss::LivgoInitSample s;
    s.p_I = Eigen::Vector3d(1.0 * i, 0.0, 0.0);
    s.R_I = Eigen::Matrix3d::Identity();
    const Eigen::Vector3d noise(std::sin(i), std::cos(i), std::sin(0.5 * i));
    s.p_r = R * (s.p_I + lever) + noise;
    samples.push_back(s);
  }
  livo_gnss::LivgoInitEstimate est;
  ASSERT_TRUE(livo_gnss::livgoInitLeastSquares(samples, Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), lever, &est));
  const double sy = std::sqrt(std::max(0.0, est.cov(1, 1))) / deg;
  EXPECT_GT(sy, 1.0);
  EXPECT_GT(est.rms_m, 0.5);
}

TEST(LivgoDtw, AlignedSeriesStayNearZero) {
  std::vector<double> t, v;
  for (int i = 0; i < 200; ++i) {
    const double ti = 0.1 * i;
    t.push_back(ti);
    v.push_back(std::sin(2.0 * M_PI * ti / 5.0) + 0.2 * std::sin(2.0 * M_PI * ti / 1.7));
  }
  double lag = 99.0;
  ASSERT_TRUE(livo_gnss::livgoDtwLag(t, v, t, v, &lag));
  EXPECT_NEAR(lag, 0.0, 0.15);
}

// =============================================================================================================
// AMENDMENTS A75 (LIVGO-v2). The scenes are synthetic and small: a host state with a diagonal covariance, a registration
// block (tilt, yaw, translation) and rows built from the real RowBuilder::antennaJacobians.
// =============================================================================================================
namespace {

using namespace livo_gnss;

constexpr double kDeg = M_PI / 180.0;

SatId satId(int i) {
  SatId s;
  s.sys = Sys::GPS;
  s.prn = i;
  s.rtk_sat = i;
  return s;
}

Eigen::Vector3d geo2ecef(double lat, double lon, double h) {
  const double a = 6378137.0, f = 1.0 / 298.257223563, e2 = f * (2.0 - f);
  const double sl = std::sin(lat), cl = std::cos(lat);
  const double N = a / std::sqrt(1.0 - e2 * sl * sl);
  return Eigen::Vector3d((N + h) * cl * std::cos(lon), (N + h) * cl * std::sin(lon), (N * (1.0 - e2) + h) * sl);
}
Eigen::Matrix3d enuRot(double lat, double lon) {
  const double sl = std::sin(lat), cl = std::cos(lat), so = std::sin(lon), co = std::cos(lon);
  Eigen::Matrix3d R;
  R.col(0) = Eigen::Vector3d(-so, co, 0.0);
  R.col(1) = Eigen::Vector3d(-sl * co, -sl * so, cl);
  R.col(2) = Eigen::Vector3d(cl * co, cl * so, sl);
  return R;
}

struct SceneOptions {
  double lever_m = 0.0;          // |a^W − o^W|: distance of the antenna from the pivot
  double sigma_p_m = 2e-3;       // host position σ
  double sigma_g = 3e-3;         // host gravity σ [m/s²] (what a converged FAST-LIVO2 reports: ~3e-3)
  double sigma_tilt_rad = 1.5e-4;
  double sigma_yaw_rad = 0.5 * kDeg;
  double sigma_trans_m = 3.0;
};

// Host + registration + bridge, all fixed-size or small; non-copyable because the AugBlock refers to cfg.
struct Scene {
  Config cfg;
  CoreState x;
  BridgeResult br;
  Leveling lev;
  AugBlock aug;
  AnchorFrame anchor;
  Eigen::Vector3d lever = Eigen::Vector3d(-0.316, 0.109, 0.401);
  Eigen::Vector3d pivot = Eigen::Vector3d::Zero();
  int off_tilt = -1, off_yaw = -1, off_trans = -1;
  Scene() : aug(cfg) {}
  Scene(const Scene&) = delete;
  Scene& operator=(const Scene&) = delete;
  LinPoint lin() const {
    LinPoint lp;
    lp.x = &x;
    lp.bridge = &br;
    lp.aug = &aug;
    lp.leveling = &lev;
    lp.anchor = &anchor;
    lp.lever_I = lever;
    lp.pivot_W = pivot;
    lp.gravity_est_en = true;
    lp.exposure_est_en = true;
    lp.delta = br.delta;
    return lp;
  }
};

std::unique_ptr<Scene> makeScene(const SceneOptions& o) {
  std::unique_ptr<Scene> sc(new Scene());
  CoreState& x = sc->x;
  x.R = so3Exp(Eigen::Vector3d(0.1, -0.05, 0.4));
  x.g = Eigen::Vector3d(0.02, -0.03, -9.81);
  x.v = Eigen::Vector3d(1.2, 0.3, 0.02);
  x.inv_expo = 1.0;
  const Eigen::Vector3d y_target = sc->pivot + o.lever_m * Eigen::Vector3d(std::cos(0.6), std::sin(0.6), 0.0) +
                                   (o.lever_m > 0.0 ? Eigen::Vector3d(0.0, 0.0, 0.5) : Eigen::Vector3d::Zero());
  x.p = y_target - x.R * sc->lever;   // a^W = p + R l = y_target
  x.P = HostMat::Zero();
  for (int i = 0; i < 3; ++i) {
    x.P(HostIdx::rot + i, HostIdx::rot + i) = 1e-8;
    x.P(HostIdx::pos + i, HostIdx::pos + i) = o.sigma_p_m * o.sigma_p_m;
    x.P(HostIdx::vel + i, HostIdx::vel + i) = 1e-4;
    x.P(HostIdx::bg + i, HostIdx::bg + i) = 1e-8;
    x.P(HostIdx::ba + i, HostIdx::ba + i) = 1e-6;
    x.P(HostIdx::grav + i, HostIdx::grav + i) = o.sigma_g * o.sigma_g;
  }
  x.P(HostIdx::expo, HostIdx::expo) = 1e-6;
  sc->lev.setRef(x.g);
  BridgeResult& br = sc->br;
  br.ok = true;
  br.delta = 0.05;
  br.t_k = 100.0;
  br.t_g = br.t_k - br.delta;
  br.dR = so3Exp(Eigen::Vector3d(0.01, 0.005, -0.02));
  br.D = Eigen::Vector3d(0.001, 0.002, -0.001);
  br.E = Eigen::Vector3d(0.01, -0.02, 0.005);
  br.omega_g = Eigen::Vector3d(0.02, -0.01, 0.3);
  br.bg_snap = x.bg;
  br.ba_snap = x.ba;
  br.g_snap = x.g;
  br.R_k = x.R;
  br.a_bar = Eigen::Vector3d(0.3, -0.2, 9.75);
  br.Sigma_D = 1e-6 * Eigen::Vector3d(1.0, 2.0, 3.0).asDiagonal();
  br.var_v_bridge = sc->cfg.imu_bridge.acc_n * sc->cfg.imu_bridge.acc_n * br.delta;
  br.var_omega = 4e-6;
  // registration: independent slots, tilt(2) yaw(1) trans(3)
  auto add = [&](Slot kind, const char* name, const Eigen::VectorXd& x0, double sigma) {
    const int d = static_cast<int>(x0.size());
    const Eigen::MatrixXd F0 = Eigen::MatrixXd::Zero(d, HostIdx::dim);
    const Eigen::MatrixXd Fa = Eigen::MatrixXd::Zero(d, sc->aug.dim());
    sc->aug.addSlot(kind, 0, name, x0, F0, Fa, Eigen::MatrixXd::Identity(d, d) * sigma * sigma, x.P);
  };
  add(Slot::Tilt, "tilt", Eigen::Vector2d(1e-4, -2e-4), o.sigma_tilt_rad);
  Eigen::VectorXd psi(1);
  psi(0) = 0.7;
  add(Slot::Yaw, "yaw", psi, o.sigma_yaw_rad);
  add(Slot::Trans, "trans", Eigen::Vector3d(5.0, -3.0, 1.0), o.sigma_trans_m);
  add(Slot::Clk, "clk", Eigen::Vector2d(120.0, 0.4), 1.0);   // the Doppler rows need the drift column
  sc->off_tilt = sc->aug.find(Slot::Tilt);
  sc->off_yaw = sc->aug.find(Slot::Yaw);
  sc->off_trans = sc->aug.find(Slot::Trans);
  const double lat = 30.0 * kDeg, lon = 110.0 * kDeg;
  sc->anchor.r0_ecef = geo2ecef(lat, lon, 50.0);
  sc->anchor.R_EN = enuRot(lat, lon);
  sc->anchor.valid = true;
  return sc;
}

// Six lines of sight (anchor ENU).
std::vector<Eigen::Vector3d> losSet() {
  const double az[6] = {20.0, 100.0, 170.0, 250.0, 320.0, 60.0}, el[6] = {30.0, 45.0, 60.0, 40.0, 35.0, 70.0};
  std::vector<Eigen::Vector3d> u;
  for (int i = 0; i < 6; ++i) {
    const double a = az[i] * kDeg, e = el[i] * kDeg;
    u.push_back(Eigen::Vector3d(std::cos(e) * std::sin(a), std::cos(e) * std::cos(a), std::sin(e)));
  }
  return u;
}

// Fixed-anchor TDCP rows of one pair end for a host position error `dp_W` at k1: H = −uᵀJ_a, ν from the physical model
// (the rows see −uᵀ M dp plus a common clock step); R = σ². `Jc` (nullable): the shared-anchor columns are applied.
RowSet tdcpToyRows(const AntennaJacobians& J, const Eigen::Vector3d& dp_W, double sigma_m, const AntennaJacobians* Jc, const AugBlock& aug) {
  RowSet rs;
  rs.cls = MeasClass::Tdcp;
  const std::vector<Eigen::Vector3d> u = losSet();
  const std::vector<int> cols = Jc ? livgoSharedAnchorColumns(aug, true) : std::vector<int>();
  for (size_t i = 0; i < u.size(); ++i) {
    MeasRow r;
    r.cls = MeasClass::Tdcp;
    r.sat = satId(static_cast<int>(i) + 1);
    r.pair_id = 1;
    r.H = -u[i].transpose() * J.J_a;
    if (Jc) livgoShareAnchorColumns(&r.H, u[i], *Jc, cols);
    r.nu = -u[i].dot(J.M * dp_W) + 5.0;   // 5 m: the common Δclk the projection takes out
    r.R = sigma_m * sigma_m;
    rs.rows.push_back(r);
  }
  return rs;
}

struct GainResult {
  double gain = 0.0;        // (Δp · dp_true)/|dp_true|²: the share of the host position error the update takes out
  double yaw_ratio = 0.0;   // posterior σ_ψ / prior σ_ψ
  double yaw_shift = 0.0;   // posterior yaw mean − prior yaw mean [rad] (A80)
  bool committed = false;
};

// One epoch's TDCP update of the toy: `exact` selects the pin of the registration after the slots exist; `share` the shared-anchor
// columns of the rows (needs the pair's k0 state: the host moved `disp_W` since).
GainResult runToyUpdate(const SceneOptions& o, bool exact, bool share, const Eigen::Vector3d& disp_W, const Eigen::Vector3d& dp_W,
                        bool freeze_yaw = false) {
  std::unique_ptr<Scene> sc = makeScene(o);
  const Eigen::VectorXd x_pin = sc->aug.x();
  if (exact) sc->aug.pinRegistrationExact(x_pin, freeze_yaw);
  else sc->aug.pinRegistration(x_pin);
  NoiseModel noise(sc->cfg);
  RowBuilder rb(sc->cfg, noise);
  LivgoAnchor anchor;
  anchor.valid = true;
  anchor.core = sc->x;
  anchor.core.p = sc->x.p - disp_W;
  anchor.br = sc->br;
  JointUpdater updater(sc->cfg);
  JointUpdater::Options opt;
  opt.disable_l4_l5 = true;   // the gain is the object here, not the gates
  CoreState core = sc->x;
  const double psi_var0 = sc->aug.Q()(sc->off_yaw, sc->off_yaw);
  const Eigen::Vector3d p0 = core.p;
  const CoreState core_start = sc->x;
  const Eigen::VectorXd xg_start = sc->aug.x();
  std::vector<double> nu_start;   // the innovations at the start (first call): later iterates see ν − H·(x_i ⊟ x_start)
  const JointUpdater::RowFactory factory = [&](const CoreState& xi, const AugBlock& a, std::vector<RowSet>* sets) {
    LinPoint lp = sc->lin();
    lp.x = &xi;
    lp.aug = &a;
    const AntennaJacobians J = rb.antennaJacobians(lp);
    AntennaJacobians Jc;
    if (share) Jc = livgoAnchorAt(rb, lp, anchor);
    RowSet rs = tdcpToyRows(J, dp_W, 1e-3, share ? &Jc : nullptr, a);
    if (nu_start.empty())
      for (const MeasRow& r : rs.rows) nu_start.push_back(r.nu);
    // a linear measurement: z − h(x_i) = (z − h(x_start)) − H (x_i − x_start); without this the IEKF iterations would apply the
    // same innovation again at every iterate
    const HostVec dxh = xi.boxminus(core_start);
    const Eigen::VectorXd dxa = a.x() - xg_start;
    for (size_t i = 0; i < rs.rows.size(); ++i)
      rs.rows[i].nu = nu_start[i] - rs.rows[i].H.head(HostIdx::dim).dot(dxh) - rs.rows[i].H.tail(dxa.size()).dot(dxa);
    sets->push_back(rs);
  };
  const JointUpdater::Result res = updater.update(core, sc->aug, factory, opt);
  GainResult g;
  g.committed = res.committed;
  g.gain = (core.p - p0).dot(dp_W) / dp_W.squaredNorm();
  g.yaw_ratio = psi_var0 > 0.0 ? std::sqrt(sc->aug.Q()(sc->off_yaw, sc->off_yaw) / psi_var0) : 0.0;   // 0: a pinned yaw (A80)
  g.yaw_shift = sc->aug.x()(sc->off_yaw) - x_pin(sc->off_yaw);
  return g;
}

}  // namespace

// ---- defaults: every A75 switch is the pre-A75 behaviour --------------------------------------------------------------
TEST(LivgoDefaults, ConfigSwitchesSitAtThePreA75Behaviour) {
  const LivgoConfig c;
  EXPECT_FALSE(c.frame_exact);
  EXPECT_EQ(c.lambda_source, LivgoConfig::LambdaSource::Min);
  EXPECT_TRUE(c.dtw_online);
  EXPECT_EQ(c.noise_mode, LivgoConfig::NoiseMode::Model);
  // the fields that existed before
  EXPECT_FALSE(c.enable);
  EXPECT_TRUE(c.paper_init);
  EXPECT_FALSE(c.dual_only);
  EXPECT_EQ(c.hessian_window, 10);
  EXPECT_EQ(c.dtw_min_epochs, 30);
  // A75 revision 1: the P2 observer is off and has no file
  EXPECT_FALSE(c.tdcp_gain_log);
  EXPECT_TRUE(c.tdcp_gain_log_path.empty());
}

TEST(LivgoDefaults, FlatNoiseIsOffAndTheModelIsUntouched) {
  Config cfg;
  NoiseModel nm(cfg);
  EXPECT_FALSE(nm.flat.on);
  // the §4.7 Doppler variance, recomputed here from its terms (as U7 does)
  NoiseModel::DoppTerms dt;
  dt.el_rad = 40.0 * kDeg;
  dt.lambda_m = 0.1903;
  dt.dopp_std_hz = 0.4;
  dt.delta_s = 0.05;
  dt.omega_norm = 0.3;
  dt.ant_acc_norm = 0.5;
  dt.v_norm = 1.4;
  dt.dt_since_last_s = 0.1;
  dt.var_v_bridge = 1e-4;
  dt.var_omega_lever = 2e-5;
  const double sin_el = std::max(std::sin(dt.el_rad), std::sin(5.0 * kDeg));
  const double s0 = std::max(dt.lambda_m * dt.dopp_std_hz, cfg.noise.dopp.sigma0_mps);
  const double kappa = NoiseModel::kappaAR(cfg.noise.dopp.tau_s, dt.dt_since_last_s);
  const double vl = dt.omega_norm * cfg.frames.lever_arm_sigma, vt = dt.ant_acc_norm * cfg.time.sigma_toff_s, vv = 0.01 * dt.v_norm;
  const double dopp_ref = (s0 * s0 * 0.5 * (1.0 + 1.0 / (sin_el * sin_el)) * kappa + dt.var_v_bridge + dt.var_omega_lever +
                           cfg.process.clock.S_d * dt.delta_s + vl * vl + vt * vt + vv * vv) *
                          cfg.noise.scale.dopp;
  EXPECT_NEAR(nm.doppVar(dt), dopp_ref, 1e-15 * dopp_ref);
  NoiseModel::TdcpTerms tt;
  tt.el_k0_rad = 35.0 * kDeg;
  tt.el_k1_rad = 36.0 * kDeg;
  tt.cp_std_k0_m = 0.003;
  tt.cp_std_k1_m = 0.004;
  tt.bridge_term_k1 = 2e-6;
  tt.dt_pair_s = 0.1;
  tt.dv_pair_norm = 0.02;
  const double s_0 = nm.sigmaPhase(tt.cp_std_k0_m, tt.el_k0_rad), s_1 = nm.sigmaPhase(tt.cp_std_k1_m, tt.el_k1_rad);
  const double vr = cfg.noise.tdcp.sigma_rate_mps * tt.dt_pair_s, vtt = tt.dv_pair_norm * cfg.time.sigma_toff_s;
  const double tdcp_ref = (s_0 * s_0 + s_1 * s_1 + tt.bridge_term_k1 + vr * vr + cfg.noise.tdcp.sigma_windup_m * cfg.noise.tdcp.sigma_windup_m +
                           vtt * vtt) *
                          cfg.noise.scale.tdcp;
  EXPECT_NEAR(nm.tdcpVar(tt), tdcp_ref, 1e-15 * tdcp_ref);
}

TEST(LivgoDefaults, LegacyPinKeepsMarginalVarianceAndDropsCrossTerms) {
  SceneOptions o;
  std::unique_ptr<Scene> sc = makeScene(o);
  // correlate the registration with the host and with itself, then pin
  Eigen::MatrixXd& Q = sc->aug.Q();
  Q(sc->off_tilt, sc->off_trans) = Q(sc->off_trans, sc->off_tilt) = 1e-5;
  Q(sc->off_yaw, sc->off_trans) = Q(sc->off_trans, sc->off_yaw) = 2e-4;
  sc->aug.C().col(sc->off_yaw).setConstant(1e-7);
  const Eigen::VectorXd x_pin = sc->aug.x();
  const double var_yaw = Q(sc->off_yaw, sc->off_yaw), var_t = Q(sc->off_trans, sc->off_trans);
  sc->aug.x()(sc->off_yaw) += 0.1;
  sc->aug.pinRegistration(x_pin);
  EXPECT_EQ(sc->aug.x()(sc->off_yaw), x_pin(sc->off_yaw)) << "the legacy pin restores the yaw mean too";
  EXPECT_EQ(sc->aug.Q()(sc->off_yaw, sc->off_yaw), var_yaw);
  EXPECT_EQ(sc->aug.Q()(sc->off_trans, sc->off_trans), var_t) << "the legacy pin keeps the translation's marginal variance (A75 §2)";
  EXPECT_EQ(sc->aug.Q()(sc->off_yaw, sc->off_trans), 0.0);
  EXPECT_EQ(sc->aug.Q()(sc->off_tilt, sc->off_trans), 0.0);
  EXPECT_EQ(sc->aug.C().col(sc->off_yaw).norm(), 0.0);
}

TEST(LivgoDefaults, FlatSigmasMustBePositive) {
  Config cfg;
  cfg.enable = true;
  cfg.ephem.ura_semantics = Config::Ephem::UraSemantics::Metres;
  LivgoConfig lc;
  lc.enable = true;
  lc.noise_mode = LivgoConfig::NoiseMode::Flat;
  lc.flat_sigma_tdcp_m = 0.001;
  lc.flat_sigma_dopp_mps = 0.05;
  EXPECT_NO_THROW({ GnssFusionLivgo ok(cfg, lc); (void)ok; });   // the control: this configuration is accepted
  lc.flat_sigma_tdcp_m = 0.0;
  EXPECT_THROW({ GnssFusionLivgo bad(cfg, lc); (void)bad; }, std::invalid_argument);
  lc.flat_sigma_tdcp_m = 0.001;
  lc.flat_sigma_dopp_mps = -1.0;
  EXPECT_THROW({ GnssFusionLivgo bad(cfg, lc); (void)bad; }, std::invalid_argument);
}

// ---- frame_exact ----------------------------------------------------------------------------------------------------
TEST(LivgoFrameExact, PinMakesTiltAndTranslationExactAndLeavesTheYawAState) {
  SceneOptions o;
  std::unique_ptr<Scene> sc = makeScene(o);
  Eigen::MatrixXd& Q = sc->aug.Q();
  // correlated registration and a host cross-covariance on every slot
  std::mt19937_64 rng(11);
  std::normal_distribution<double> g(0.0, 1.0);
  const int n = sc->aug.dim();
  for (int i = 0; i < n; ++i)
    for (int j = 0; j <= i; ++j)
      if (i != j) Q(i, j) = Q(j, i) = 0.1 * std::sqrt(Q(i, i) * Q(j, j)) * (g(rng) > 0 ? 1.0 : -1.0);
  for (int c = 0; c < n; ++c)
    for (int r = 0; r < HostIdx::dim; ++r) sc->aug.C()(r, c) = 1e-9 * g(rng);
  const Eigen::VectorXd x_pin = sc->aug.x();
  const Eigen::MatrixXd Q0 = Q;
  const Eigen::Matrix<double, HostIdx::dim, Eigen::Dynamic> C0 = sc->aug.C();
  sc->aug.x() += 0.01 * Eigen::VectorXd::Ones(n);   // everything walked away
  sc->aug.pinRegistrationExact(x_pin);
  // means: tilt and translation restored, the yaw is not
  for (int k = 0; k < 2; ++k) EXPECT_EQ(sc->aug.x()(sc->off_tilt + k), x_pin(sc->off_tilt + k));
  for (int k = 0; k < 3; ++k) EXPECT_EQ(sc->aug.x()(sc->off_trans + k), x_pin(sc->off_trans + k));
  EXPECT_EQ(sc->aug.x()(sc->off_yaw), x_pin(sc->off_yaw) + 0.01);
  // tilt and translation: exact zeros in Q (rows and columns, diagonal included) and in the host cross-covariance
  for (int k = 0; k < 2; ++k) {
    EXPECT_EQ(sc->aug.Q().row(sc->off_tilt + k).norm(), 0.0);
    EXPECT_EQ(sc->aug.Q().col(sc->off_tilt + k).norm(), 0.0);
    EXPECT_EQ(sc->aug.C().col(sc->off_tilt + k).norm(), 0.0);
  }
  for (int k = 0; k < 3; ++k) {
    EXPECT_EQ(sc->aug.Q().row(sc->off_trans + k).norm(), 0.0);
    EXPECT_EQ(sc->aug.Q().col(sc->off_trans + k).norm(), 0.0);
    EXPECT_EQ(sc->aug.C().col(sc->off_trans + k).norm(), 0.0);
  }
  // the yaw keeps its variance and its covariance with the host (and with the states that are not tilt/translation)
  EXPECT_EQ(sc->aug.Q()(sc->off_yaw, sc->off_yaw), Q0(sc->off_yaw, sc->off_yaw));
  EXPECT_EQ((sc->aug.C().col(sc->off_yaw) - C0.col(sc->off_yaw)).norm(), 0.0);
  // the joint covariance is still PSD with the exact zero rows (the L5 test of the updater)
  EXPECT_TRUE(JointUpdater::posteriorPsd(sc->aug.jointCov(sc->x.P)));
}

// The dead row of AMENDMENTS A75 §2: a fixed-anchor TDCP row has the translation columns −u₁ᵀ that no anchor cancels, so the
// translation's marginal variance (σ 3 m) sits in S and takes the position error of the host. With the exact frame it does not.
// The antenna sits on the pivot here (no yaw/tilt lever), so the translation is the only difference.
TEST(LivgoFrameExact, FixedAnchorTdcpReachesTheHostOnlyWithTheExactFrame) {
  SceneOptions o;
  o.lever_m = 0.0;
  const Eigen::Vector3d dp(3e-3, -1e-3, 0.5e-3);
  const GainResult legacy = runToyUpdate(o, /*exact=*/false, /*share=*/false, Eigen::Vector3d::Zero(), dp);
  const GainResult exact = runToyUpdate(o, /*exact=*/true, /*share=*/false, Eigen::Vector3d::Zero(), dp);
  ASSERT_TRUE(legacy.committed);
  ASSERT_TRUE(exact.committed);
  std::printf("[ LIVGO-v2 ] host position gain of one TDCP epoch: legacy pin %.3e, exact frame %.3f\n", legacy.gain, exact.gain);
  EXPECT_LT(legacy.gain, 1e-3) << "the translation variance absorbs the rows (A75 §2: ~1e-7)";
  EXPECT_GT(exact.gain, 0.5) << "without it the rows pull the host onto the carrier displacement";
  EXPECT_LT(exact.gain, 1.0001);
}

// The same rows far from the pivot. With a yaw that stays a state, a fixed anchor taken as a number gives the yaw the lever
// arm of the pivot distance: one epoch of millimetre rows would "determine" the yaw to a few thousandths of its prior σ, and
// the Doppler rows could not move it afterwards. The shared-anchor columns (livgo_anchor.hpp) give the yaw the displacement
// over the pair instead.
TEST(LivgoFrameExact, YawLeverOfAFixedAnchorIsRemovedByTheSharedColumns) {
  SceneOptions o;
  o.lever_m = 50.0;
  const Eigen::Vector3d dp(3e-3, -1e-3, 0.5e-3);
  const Eigen::Vector3d disp(0.1, 0.02, 0.0);   // the antenna moved 0.1 m since k0
  const GainResult as_number = runToyUpdate(o, /*exact=*/true, /*share=*/false, disp, dp);
  const GainResult shared = runToyUpdate(o, /*exact=*/true, /*share=*/true, disp, dp);
  ASSERT_TRUE(as_number.committed);
  ASSERT_TRUE(shared.committed);
  std::printf("[ LIVGO-v2 ] 50 m from the pivot, posterior/prior sigma_yaw: anchor as a number %.4f, shared columns %.3f; host gain %.3f / %.3f\n",
              as_number.yaw_ratio, shared.yaw_ratio, as_number.gain, shared.gain);
  EXPECT_LT(as_number.yaw_ratio, 0.05) << "the lever-arm column collapses the yaw variance";
  EXPECT_GT(shared.yaw_ratio, 0.3) << "the displacement over the pair does not";
  EXPECT_LE(shared.yaw_ratio, 1.0 + 1e-9);
  EXPECT_GT(shared.gain, 0.5);
}

// The shared columns of a TDCP row against central differences of the model it is the Jacobian of: h = −u₁·(a^N(x) − ĉ(x)) with the
// anchor ĉ(x) the k0 antenna mapped through the same registration and gravity (frozen line of sight). Columns: tilt, yaw,
// translation, host gravity — the relative ones; the host position column keeps the k1 antenna's.
TEST(LivgoFrameExact, SharedAnchorColumnsMatchCentralDifferences) {
  SceneOptions o;
  o.lever_m = 60.0;
  std::unique_ptr<Scene> sc = makeScene(o);
  NoiseModel noise(sc->cfg);
  RowBuilder rb(sc->cfg, noise);
  LivgoAnchor anchor;
  anchor.valid = true;
  anchor.core = sc->x;
  anchor.core.p = sc->x.p - Eigen::Vector3d(0.12, -0.03, 0.01);
  anchor.core.R = sc->x.R * so3Exp(Eigen::Vector3d(1e-3, -2e-3, 3e-3));   // the host turned a little since k0
  anchor.br = sc->br;
  const Eigen::Vector3d u = losSet()[1];
  const LinPoint lp = sc->lin();
  const AntennaJacobians J = rb.antennaJacobians(lp);
  const AntennaJacobians Jc = livgoAnchorAt(rb, lp, anchor);
  Eigen::RowVectorXd H = -u.transpose() * J.J_a;
  const std::vector<int> cols = livgoSharedAnchorColumns(sc->aug, true);
  ASSERT_EQ(cols.size(), 9u) << "tilt 2 + yaw 1 + translation 3 + gravity 3";
  livgoShareAnchorColumns(&H, u, Jc, cols);
  // h(x) = −u·(a^N(x) − ĉ(x)) as a function of the registration (additive) and the host gravity (additive)
  auto model = [&](const Eigen::VectorXd& xg, const Eigen::Vector3d& g) {
    AugBlock a2 = sc->aug;   // copy-constructed (the Config reference is shared), then only the means are replaced
    a2.x() = xg;
    CoreState xk = sc->x;
    xk.g = g;
    LinPoint l2 = sc->lin();
    l2.x = &xk;
    l2.aug = &a2;
    const AntennaJacobians J2 = rb.antennaJacobians(l2);
    const AntennaJacobians Jc2 = livgoAnchorAt(rb, l2, anchor);
    return -u.dot(J2.a_N - Jc2.a_N);
  };
  const Eigen::VectorXd xg0 = sc->aug.x();
  for (int c : cols) {
    double fd = 0.0;
    if (c >= HostIdx::dim) {
      const double h = 1e-6;
      Eigen::VectorXd xp = xg0, xm = xg0;
      xp(c - HostIdx::dim) += h;
      xm(c - HostIdx::dim) -= h;
      fd = (model(xp, sc->x.g) - model(xm, sc->x.g)) / (2.0 * h);
    } else {
      const double h = 1e-6;
      Eigen::Vector3d gp = sc->x.g, gm = sc->x.g;
      gp(c - HostIdx::grav) += h;
      gm(c - HostIdx::grav) -= h;
      fd = (model(xg0, gp) - model(xg0, gm)) / (2.0 * h);
    }
    const double scale = std::max(std::fabs(H(c)), std::fabs(fd));
    // 5e-8: the roundoff of a difference quotient of a ~60 m quantity at h = 1e-6 (6e-9) with margin; the columns that matter are 0.1–10
    EXPECT_NEAR(H(c), fd, 1e-5 * scale + 5e-8) << "column " << c;
  }
  // the translation columns cancel exactly (both ends carry the identity), and the yaw column is the pair's displacement,
  // not the lever arm
  for (int k = 0; k < 3; ++k) EXPECT_NEAR(H(HostIdx::dim + sc->off_trans + k), 0.0, 1e-12);
  const double lever_col = std::fabs((-u.transpose() * J.J_a)(HostIdx::dim + sc->off_yaw));
  EXPECT_GT(lever_col, 10.0 * std::fabs(H(HostIdx::dim + sc->off_yaw))) << "the unshared yaw column is the lever arm";
  // the host position column is untouched
  const Eigen::RowVectorXd H0 = -u.transpose() * J.J_a;
  for (int k = 0; k < 3; ++k) EXPECT_EQ(H(HostIdx::pos + k), H0(HostIdx::pos + k));
}

// ---- flat noise ---------------------------------------------------------------------------------------------------------
TEST(LivgoNoise, FlatVarianceIsSigmaSquaredPlusTheBridgeTerms) {
  Config cfg;
  NoiseModel nm(cfg);
  nm.flat.on = true;
  nm.flat.sigma_dopp_mps = 0.05;
  nm.flat.sigma_tdcp_m = 0.001;
  // Doppler: nothing depends on elevation, C/N0-like receiver std, κ, scale, σ_toff, the lever-arm σ or the speed
  for (double el : {8.0, 30.0, 89.0}) {
    NoiseModel::DoppTerms t;
    t.el_rad = el * kDeg;
    t.lambda_m = 0.19;
    t.dopp_std_hz = 3.0;                 // a receiver std that would dominate the model
    t.delta_s = 0.07;
    t.omega_norm = 2.0;
    t.ant_acc_norm = 3.0;
    t.v_norm = 5.0;
    t.var_v_bridge = 1.5e-4;
    t.var_omega_lever = 3e-5;
    const double ref = 0.05 * 0.05 + 1.5e-4 + 3e-5 + cfg.process.clock.S_d * 0.07;
    EXPECT_DOUBLE_EQ(nm.doppVar(t), ref) << "el " << el;
  }
  // TDCP: σ² + u₁ᵀMΣ_DMᵀu₁, independent of the phase σ, elevations, pair length, Δv and σ_toff
  for (double cp : {0.002, 0.02}) {
    NoiseModel::TdcpTerms t;
    t.el_k0_rad = 10.0 * kDeg;
    t.el_k1_rad = 20.0 * kDeg;
    t.cp_std_k0_m = cp;
    t.cp_std_k1_m = cp;
    t.bridge_term_k1 = 7e-7;
    t.dt_pair_s = 0.1;
    t.dv_pair_norm = 3.0;
    EXPECT_DOUBLE_EQ(nm.tdcpVar(t), 0.001 * 0.001 + 7e-7) << "cp_std " << cp;
  }
  // the pseudorange variance and the helper functions the slip detector uses are not flat
  NoiseModel model(cfg);
  NoiseModel::PsrTerms pt;
  pt.el_rad = 30.0 * kDeg;
  pt.cn0 = 35.0;
  EXPECT_EQ(nm.psrVar(pt), model.psrVar(pt));
  EXPECT_EQ(nm.sigmaPhase(0.003, 0.5), model.sigmaPhase(0.003, 0.5));
  EXPECT_EQ(nm.doppMarginalVar(0.19, 1.0, 0.5), model.doppMarginalVar(0.19, 1.0, 0.5));
}

// The rows RowBuilder builds from a flat model: R_i = σ² + bridge, and the bridge cross terms between satellites (R_full) carry the same
// unscaled quadratic form (the owner hands the builder a Config with noise.scale 1).
TEST(LivgoNoise, FlatRowsOfTheBuilderCarryOnlySigmaAndTheBridge) {
  SceneOptions o;
  o.lever_m = 20.0;
  std::unique_ptr<Scene> sc = makeScene(o);
  Config cfg_flat = sc->cfg;
  cfg_flat.noise.scale.dopp = 1.0;
  cfg_flat.noise.scale.tdcp = 1.0;
  NoiseModel noise(cfg_flat);
  noise.flat.on = true;
  noise.flat.sigma_dopp_mps = 0.05;
  noise.flat.sigma_tdcp_m = 0.001;
  RowBuilder rb(cfg_flat, noise);
  const LinPoint lp = sc->lin();
  const AntennaJacobians J = rb.antennaJacobians(lp);
  const AnchorFrame& an = sc->anchor;
  const Eigen::Vector3d r_A = an.r0_ecef + an.R_EN * J.a_N;
  const std::vector<Eigen::Vector3d> los = losSet();
  const int m = static_cast<int>(los.size());
  ObsEpoch epoch;
  epoch.t_rx = GTime::fromSec(1.4e9);
  std::vector<SatGeometry> geo;
  std::vector<MaskVerdict> masks;
  for (int j = 0; j < m; ++j) {
    SatObs so;
    so.sat = satId(j + 1);
    SignalObs sig;
    sig.freq_hz = 1575.42e6;
    sig.code = SignalCode::L1C;
    sig.status = kStatusPsrValid | kStatusCpValid | kStatusHalfCycValid;
    sig.psr_m = 2.2e7;
    sig.dopp_hz = 100.0 * (j + 1) - 50.0;   // non-zero: doppValid()
    sig.dopp_std_hz = 0.5 + j;     // large and satellite-dependent: must not matter
    sig.cn0_dbhz = 20.0 + 4.0 * j;   // likewise
    so.sig.push_back(sig);
    epoch.sats.push_back(so);
    SatGeometry g;
    g.sat = so.sat;
    g.state.ok = true;
    g.state.r_ecef = r_A + an.R_EN * los[j] * 2.3e7;
    g.state.v_ecef = Eigen::Vector3d(1000.0 * (j + 1), -2000.0, 500.0 * j);
    g.state.dts = 1e-4;
    g.state.ddts = 1e-11;
    g.range = (g.state.r_ecef - r_A).norm();
    g.e_ecef = (g.state.r_ecef - r_A) / g.range;
    g.u_N = an.R_EN.transpose() * g.e_ecef;
    g.el_rad = std::asin(los[j].z());
    g.az_rad = std::atan2(los[j].x(), los[j].y());
    g.lambda_m = 299792458.0 / sig.freq_hz;
    geo.push_back(g);
    MaskVerdict v;
    v.psr = v.dopp = v.cp = true;
    masks.push_back(v);
  }
  // Doppler
  RowSet dop;
  rb.dopplerRows(epoch, geo, masks, lp, J, {}, &dop);
  ASSERT_EQ(static_cast<int>(dop.rows.size()), m);
  ASSERT_TRUE(dop.has_full_R);
  const Eigen::Matrix3d Rg = sc->x.R * sc->br.dR.transpose();
  const Eigen::Matrix3d MRL = J.M * Rg * skew(sc->lever);
  const Eigen::Matrix3d Sov = sc->br.var_omega * MRL * MRL.transpose();
  std::vector<Eigen::Vector3d> u1;
  for (int j = 0; j < m; ++j) u1.push_back(an.R_EN.transpose() * geo[j].e_ecef);
  for (int i = 0; i < m; ++i) {
    const double ref = 0.05 * 0.05 + sc->br.var_v_bridge + u1[i].dot(Sov * u1[i]) + cfg_flat.process.clock.S_d * sc->br.delta;
    EXPECT_NEAR(dop.rows[i].R, ref, 1e-15) << "Doppler row " << i;
    for (int k = 0; k < m; ++k) {
      if (k == i) EXPECT_NEAR(dop.R_full(i, k), dop.rows[i].R, 1e-15);
      else EXPECT_NEAR(dop.R_full(i, k), u1[i].dot(Sov * u1[k]), 1e-15) << "Doppler cross term " << i << "," << k;
    }
  }
  // TDCP (fixed anchor: no clone column)
  RowBuilder::TdcpPairInput pair;
  pair.pair_id = 1;
  pair.clone_slot = -1;
  pair.c_hat_N = J.a_N - Eigen::Vector3d(0.1, 0.0, 0.0);
  pair.dt_pair_s = 0.1;
  for (int j = 0; j < m; ++j) {
    pair.sats.push_back(geo[j].sat);
    pair.phi_k0_cyc.push_back(1e8);
    pair.phi_k1_cyc.push_back(1e8 + 0.3 * j);
    pair.dts_k0.push_back(geo[j].state.dts);
    pair.dts_k1.push_back(geo[j].state.dts);
    pair.rs_k0.push_back(geo[j].state.r_ecef);
    pair.el_k0_rad.push_back(geo[j].el_rad);
    pair.el_k1_rad.push_back(geo[j].el_rad);
    pair.cp_std_k0_m.push_back(0.01 * (j + 1));   // large: must not matter
    pair.cp_std_k1_m.push_back(0.01 * (j + 1));
  }
  RowSet td;
  rb.tdcpRows(pair, geo, lp, J, &td);
  ASSERT_EQ(static_cast<int>(td.rows.size()), m);
  ASSERT_TRUE(td.has_full_R);
  const Eigen::Matrix3d MSDMt = J.M * sc->br.Sigma_D * J.M.transpose();
  for (int i = 0; i < m; ++i) {
    const double ref = 0.001 * 0.001 + u1[i].dot(MSDMt * u1[i]);
    EXPECT_NEAR(td.rows[i].R, ref, 1e-15) << "TDCP row " << i;
    for (int k = 0; k < m; ++k) {
      if (k == i) EXPECT_NEAR(td.R_full(i, k), td.rows[i].R, 1e-15);
      else EXPECT_NEAR(td.R_full(i, k), u1[i].dot(MSDMt * u1[k]), 1e-15) << "TDCP cross term " << i << "," << k;
    }
  }
  // the R_full of both sets is positive definite
  Eigen::LDLT<Eigen::MatrixXd> ld(td.R_full), ldd(dop.R_full);
  EXPECT_TRUE(ld.isPositive());
  EXPECT_TRUE(ldd.isPositive());
}

// ---- A75 revision 1: the TDCP share of the host position variance (prediction P2) --------------------------------------------
namespace {

// One epoch of the toy: three position-observing rows of class Dopp (σ 5 mm: the "everything but TDCP" part of the epoch, any
// class tag does for the instrument) and, while `tdcp_in`, the six fixed-anchor TDCP rows (σ 1 mm) of tdcpToyRows. The rows
// carry ν = 0: the object is the posterior covariance, which does not depend on the innovations.
struct GainToy {
  double tr_prior = 0.0, tr_all = 0.0, tr_no = 0.0;
  bool committed = false, cf_ok = false;
  double g() const { return 1.0 - tr_all / tr_no; }
};

GainToy runGainToy(bool exact) {
  SceneOptions o;
  o.lever_m = 0.0;
  std::unique_ptr<Scene> sc = makeScene(o);
  const Eigen::VectorXd x_pin = sc->aug.x();
  if (exact) sc->aug.pinRegistrationExact(x_pin);
  else sc->aug.pinRegistration(x_pin);
  NoiseModel noise(sc->cfg);
  RowBuilder rb(sc->cfg, noise);
  JointUpdater updater(sc->cfg);
  JointUpdater::Options opt;
  opt.disable_l4_l5 = true;
  bool tdcp_in = true;
  const double sigma_pos = 5e-3;
  const JointUpdater::RowFactory factory = [&](const CoreState& xi, const AugBlock& a, std::vector<RowSet>* sets) {
    RowSet pos;
    pos.cls = MeasClass::Dopp;
    for (int i = 0; i < 3; ++i) {
      MeasRow r;
      r.cls = MeasClass::Dopp;
      r.sat = satId(20 + i);
      r.H = Eigen::RowVectorXd::Zero(HostIdx::dim + a.dim());
      r.H(HostIdx::pos + i) = 1.0;
      r.R = sigma_pos * sigma_pos;
      pos.rows.push_back(r);
    }
    sets->push_back(pos);
    if (tdcp_in) {
      LinPoint lp = sc->lin();
      lp.x = &xi;
      lp.aug = &a;
      sets->push_back(tdcpToyRows(rb.antennaJacobians(lp), Eigen::Vector3d::Zero(), 1e-3, nullptr, a));
    }
  };
  const CoreState core0 = sc->x;
  const AugBlock aug0(sc->aug);
  GainToy t;
  t.tr_prior = livgoPosTrace(core0.P);
  CoreState core = core0;
  AugBlock aug(aug0);
  const JointUpdater::Result res = updater.update(core, aug, factory, opt);
  t.committed = res.committed;
  t.tr_all = livgoPosTrace(core.P);
  tdcp_in = false;
  t.cf_ok = livgoCounterfactualPosTrace(updater, core0, aug0, factory, opt, &t.tr_no);
  tdcp_in = true;
  return t;
}

}  // namespace

// The counterfactual is the epoch without its TDCP rows: here the three position rows alone, whose posterior has a closed form.
// The TDCP share g = 1 − tr(all)/tr(without) then separates a live row set (exact frame) from the dead one (legacy pin:
// A75 §2, gain ~1e-7) by orders of magnitude — the two registered thresholds of P2 (alive ≥ 1e-2, dead < 1e-3) sit between.
TEST(LivgoTdcpGain, CounterfactualIsTheEpochWithoutTheTdcpRowsAndSeparatesLiveFromDeadRows) {
  const GainToy exact = runGainToy(/*exact=*/true);
  const GainToy legacy = runGainToy(/*exact=*/false);
  ASSERT_TRUE(exact.committed);
  ASSERT_TRUE(legacy.committed);
  ASSERT_TRUE(exact.cf_ok);
  ASSERT_TRUE(legacy.cf_ok);
  const double s_p2 = 2e-3 * 2e-3, s_r2 = 5e-3 * 5e-3;
  const double want_no = 3.0 / (1.0 / s_p2 + 1.0 / s_r2);   // independent axes, one 5 mm row per axis
  std::printf("[ LIVGO-v2 ] TDCP share of the host position variance: exact frame %.3e (tr %.4e -> %.4e), legacy pin %.3e\n", exact.g(),
              exact.tr_no, exact.tr_all, legacy.g());
  EXPECT_NEAR(exact.tr_no / want_no, 1.0, 1e-9) << "the counterfactual is the Doppler-like rows alone";
  EXPECT_NEAR(legacy.tr_no / want_no, 1.0, 1e-9);
  EXPECT_LT(exact.tr_no, exact.tr_prior);
  EXPECT_GT(exact.g(), 1e-2) << "alive threshold of P2";
  EXPECT_LT(legacy.g(), 1e-3) << "dead threshold of P2 (the legacy pin)";
  EXPECT_GE(legacy.g(), -1e-12);
  EXPECT_LT(exact.g(), 1.0);
}

// With no row left the counterfactual commits nothing: the posterior position block is the prior's (the TDCP rows were the whole
// epoch). The inputs are const; the copies the helper makes are what changes.
TEST(LivgoTdcpGain, EpochWithOnlyTdcpRowsHasThePriorAsItsCounterfactual) {
  SceneOptions o;
  o.lever_m = 0.0;
  std::unique_ptr<Scene> sc = makeScene(o);
  sc->aug.pinRegistrationExact(sc->aug.x());
  JointUpdater updater(sc->cfg);
  JointUpdater::Options opt;
  opt.disable_l4_l5 = true;
  const JointUpdater::RowFactory none = [](const CoreState&, const AugBlock&, std::vector<RowSet>*) {};
  const CoreState core0 = sc->x;
  const AugBlock aug0(sc->aug);
  double tr = -1.0;
  ASSERT_TRUE(livgoCounterfactualPosTrace(updater, core0, aug0, none, opt, &tr));
  EXPECT_EQ(tr, livgoPosTrace(core0.P));
}

// =============================================================================================================
// AMENDMENTS A80 (LIVGO-literal)
// =============================================================================================================
namespace {

// The long-baseline least-squares problem of LivgoInit.LongBaselineRecoversTiltAndLever as a reusable list of samples:
// `n` epochs along a 5 m/epoch track, true rotation R (30 deg yaw, 0.5 deg about y), a small deterministic noise.
std::vector<livo_gnss::LivgoInitSample> a80Track(int n, double step_m, const Eigen::Matrix3d& R, const Eigen::Vector3d& p, const Eigen::Vector3d& lever,
                                                 double noise_m) {
  std::vector<livo_gnss::LivgoInitSample> v;
  for (int i = 0; i < n; ++i) {
    livo_gnss::LivgoInitSample s;
    s.p_I = Eigen::Vector3d(step_m * i, 8.0 * std::sin(0.05 * i), 0.0);
    s.R_I = Eigen::AngleAxisd(0.015 * i, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const Eigen::Vector3d noise = noise_m * Eigen::Vector3d(std::sin(i), std::cos(i * 0.7), 0.5 * std::sin(i * 0.3));
    s.p_r = R * (s.p_I + s.R_I * lever) + p + noise;
    v.push_back(s);
  }
  return v;
}

livo_gnss::LivgoInitEstimate a80Solve(const std::vector<livo_gnss::LivgoInitSample>& samples, const Eigen::Vector3d& lever) {
  Eigen::Vector3d p0 = Eigen::Vector3d::Zero();
  for (const auto& s : samples) p0 += s.p_r - (s.p_I + lever);
  p0 /= static_cast<double>(samples.size());
  livo_gnss::LivgoInitEstimate est;
  livo_gnss::livgoInitLeastSquares(samples, Eigen::Matrix3d::Identity(), p0, lever, &est);
  return est;
}

// A GPS L1 satellite that passes every mask at 45 dB-Hz / 45 deg (test_u12_gating's cleanGps).
SatObs a80CleanGps(double cn0) {
  SatObs so;
  so.sat.sys = Sys::GPS;
  so.sat.prn = 7;
  so.sat.rtk_sat = 7;
  so.t_rx = GTime::fromSec(1.4e9);
  SignalObs sig;
  sig.freq_hz = 1575.42e6;
  sig.code = SignalCode::L1C;
  sig.psr_m = 2.2e7;
  sig.cp_cyc = 1.15e8;
  sig.dopp_hz = 1000.0;
  sig.cn0_dbhz = cn0;
  sig.status = kStatusPsrValid | kStatusCpValid | kStatusHalfCycValid | kStatusHalfCycSub;
  so.sig.push_back(sig);
  return so;
}

MaskVerdict a80Verdict(const Config::Masks& masks, const SatObs& so, double el_deg) {
  Config cfg;
  cfg.masks = masks;
  QualityGate gate(cfg);
  MaskInput in;
  in.obs = &so;
  in.el_deg = el_deg;
  in.az_deg = 120.0;
  in.has_eph = true;
  in.svh = 0;
  in.eph_age_s = 600.0;
  in.psr_track_ok = true;
  in.cp_track_ok = true;
  return gate.evaluate(in);
}

struct EventSink : livo_gnss::DiagnosticsSink {
  std::vector<livo_gnss::Event> events;
  void onEvent(const livo_gnss::Event& e) override { events.push_back(e); }
  bool has(const std::string& needle) const {
    for (const auto& e : events)
      if (e.detail.find(needle) != std::string::npos) return true;
    return false;
  }
};

}  // namespace

// ---- defaults: every A80 switch is the pre-A80 behaviour --------------------------------------------------------------
TEST(LivgoA80, SwitchesSitAtThePreA80Behaviour) {
  const LivgoConfig c;
  EXPECT_FALSE(c.freeze_yaw);
  EXPECT_EQ(c.init_travel_m, 0.0);
  EXPECT_EQ(c.init_tilt_sigma_max_deg, 0.0);
  EXPECT_FALSE(c.cn0_hard);
  EXPECT_FALSE(c.primary_only);
  EXPECT_EQ(c.dtw_grid_s, 0.1);
  EXPECT_EQ(c.dtw_window_s, 40.0);
  EXPECT_EQ(c.dtw_min_epochs, 30);
  // the earlier numbers the helper reads: the paper's 35 dB-Hz and 15 deg are the defaults of cn0_min_dbhz and el_min_deg
  EXPECT_EQ(c.cn0_min_dbhz, 35.0);
  EXPECT_EQ(c.el_min_deg, 15.0);
  const LivgoDtwParams d;
  EXPECT_EQ(d.grid_s, 0.1);
  EXPECT_EQ(d.window_s, 40.0);
  EXPECT_EQ(d.band_s, 2.0);
}

// Without cn0_hard the masks are what the arms before A80 ran with: the yaml's C/N0 floors untouched, el_min_deg only raises.
TEST(LivgoA80, MasksWithoutCn0HardAreTheEarlierOnes) {
  LivgoConfig l;
  Config::Masks m;
  m.el_deg = 10.0;
  m.cn0_psr = 20.0;
  m.cn0_dopp = 12.0;
  m.cn0_cp = 25.0;
  livgoApplyMasks(l, &m);
  EXPECT_EQ(m.el_deg, 15.0) << "a looser mask is raised to el_min_deg";
  EXPECT_EQ(m.cn0_psr, 20.0);
  EXPECT_EQ(m.cn0_dopp, 12.0);
  EXPECT_EQ(m.cn0_cp, 25.0);
  m.el_deg = 20.0;
  livgoApplyMasks(l, &m);
  EXPECT_EQ(m.el_deg, 20.0) << "a stricter mask is kept";
}

// cn0_hard: the three C/N0 masks are cn0_min_dbhz and the elevation mask is el_min_deg; the real gate then cuts at those values.
TEST(LivgoA80, Cn0HardMasksCutPseudorangeDopplerAndCarrierAt35DbHzAnd15Deg) {
  LivgoConfig l;
  l.cn0_hard = true;
  Config::Masks m;   // the yaml's looser values, and a stricter elevation to see that it is assigned
  m.el_deg = 20.0;
  m.cn0_psr = 20.0;
  m.cn0_dopp = 12.0;
  m.cn0_cp = 25.0;
  livgoApplyMasks(l, &m);
  EXPECT_EQ(m.cn0_psr, 35.0);
  EXPECT_EQ(m.cn0_dopp, 35.0);
  EXPECT_EQ(m.cn0_cp, 35.0);
  EXPECT_EQ(m.el_deg, 15.0);
  {
    const SatObs so = a80CleanGps(35.0);
    const MaskVerdict v = a80Verdict(m, so, 15.0);
    EXPECT_TRUE(v.psr && v.dopp && v.cp) << "35.0 dB-Hz at 15.0 deg is on the limit and passes";
  }
  {
    const SatObs so = a80CleanGps(34.9);
    const MaskVerdict v = a80Verdict(m, so, 45.0);
    EXPECT_FALSE(v.psr);
    EXPECT_FALSE(v.dopp);
    EXPECT_FALSE(v.cp);
    EXPECT_NE(v.reasons & kMaskCn0Psr, 0u);
    EXPECT_NE(v.reasons & kMaskCn0Dopp, 0u);
    EXPECT_NE(v.reasons & kMaskCn0Cp, 0u);
  }
  {
    const SatObs so = a80CleanGps(45.0);
    const MaskVerdict v = a80Verdict(m, so, 14.9);
    EXPECT_FALSE(v.psr || v.dopp || v.cp);
    EXPECT_NE(v.reasons & kMaskElevation, 0u);
  }
  // the same 34.9 dB-Hz satellite passes the yaml's own floors (what the earlier arms admit)
  Config::Masks yaml;
  yaml.cn0_psr = 20.0;
  yaml.cn0_dopp = 12.0;
  yaml.cn0_cp = 25.0;
  const SatObs so = a80CleanGps(34.9);
  const MaskVerdict v = a80Verdict(yaml, so, 45.0);
  EXPECT_TRUE(v.psr && v.dopp && v.cp);
}

TEST(LivgoA80, PrimarySignalIsTheMainFrequencyOfEachSystem) {
  EXPECT_TRUE(livgoIsPrimarySignal(Sys::GPS, SignalCode::L1C));
  EXPECT_FALSE(livgoIsPrimarySignal(Sys::GPS, SignalCode::L2X));
  EXPECT_FALSE(livgoIsPrimarySignal(Sys::GPS, SignalCode::L5X));
  EXPECT_TRUE(livgoIsPrimarySignal(Sys::QZS, SignalCode::L1C));
  EXPECT_TRUE(livgoIsPrimarySignal(Sys::GLO, SignalCode::L1C));
  EXPECT_FALSE(livgoIsPrimarySignal(Sys::GLO, SignalCode::L2C));
  EXPECT_TRUE(livgoIsPrimarySignal(Sys::GAL, SignalCode::L1X));
  EXPECT_FALSE(livgoIsPrimarySignal(Sys::GAL, SignalCode::L7X)) << "E5b only: a secondary band, which the L0 gate alone would admit";
  EXPECT_FALSE(livgoIsPrimarySignal(Sys::GAL, SignalCode::L5Q));
  EXPECT_TRUE(livgoIsPrimarySignal(Sys::BDS, SignalCode::L2I)) << "B1I (1561.098 MHz) is the BDS main frequency";
  EXPECT_FALSE(livgoIsPrimarySignal(Sys::BDS, SignalCode::L7I));
  EXPECT_FALSE(livgoIsPrimarySignal(Sys::GPS, SignalCode::Unknown));
  // and what the L0 gate does with a secondary-only satellite (the reason for the switch): L2X as sig[0] passes it
  SatObs so = a80CleanGps(45.0);
  so.sig[0].code = SignalCode::L2X;
  so.sig[0].freq_hz = 1227.60e6;
  const MaskVerdict v = a80Verdict(Config::Masks(), so, 45.0);
  EXPECT_TRUE(v.psr && v.dopp && v.cp) << "the core's L0 only refuses the geometry-free-only codes";
}

// ---- DTW parameters ------------------------------------------------------------------------------------------------------
namespace {
// An aperiodic speed series (sum of incommensurate sines); gnss = host delayed by delay(t).
void a80Speeds(double t_end, double dt, const std::function<double(double)>& delay, std::vector<double>* t, std::vector<double>* vh,
               std::vector<double>* vg) {
  auto f = [](double x) { return std::sin(0.9 * x) + 0.7 * std::sin(0.37 * x + 1.0) + 0.5 * std::sin(1.9 * x + 0.3) + 0.3 * std::sin(3.1 * x); };
  for (double ti = 0.0; ti < t_end; ti += dt) {
    t->push_back(ti);
    vh->push_back(f(ti));
    vg->push_back(f(ti - delay(ti)));
  }
}
}  // namespace

TEST(LivgoA80, DtwDefaultParametersEqualThePassedDefaults) {
  std::vector<double> t, vh, vg;
  a80Speeds(60.0, 0.1, [](double) { return 0.3; }, &t, &vh, &vg);
  double a = 99.0, b = 98.0;
  ASSERT_TRUE(livo_gnss::livgoDtwLag(t, vg, t, vh, &a));
  LivgoDtwParams p;
  p.grid_s = 0.1;
  p.window_s = 40.0;
  p.band_s = 2.0;
  ASSERT_TRUE(livo_gnss::livgoDtwLag(t, vg, t, vh, &b, p));
  EXPECT_EQ(a, b);
  EXPECT_NEAR(a, -0.3, 0.15);
}

TEST(LivgoA80, DtwGridParameterTakesEffect) {
  {
    std::vector<double> t, vh, vg;
    a80Speeds(40.0, 0.05, [](double) { return 0.3; }, &t, &vh, &vg);
    LivgoDtwParams p;
    p.grid_s = 0.05;
    double lag = 99.0;
    ASSERT_TRUE(livo_gnss::livgoDtwLag(t, vg, t, vh, &lag, p));
    EXPECT_NEAR(lag, -0.3, 0.08);
  }
  {
    // a 1 s delay on a 0.25 s grid (the band stays 2 s = 8 samples)
    std::vector<double> t, vh, vg;
    a80Speeds(120.0, 0.25, [](double) { return 1.0; }, &t, &vh, &vg);
    LivgoDtwParams p;
    p.grid_s = 0.25;
    p.window_s = 60.0;
    double lag = 99.0;
    ASSERT_TRUE(livo_gnss::livgoDtwLag(t, vg, t, vh, &lag, p));
    EXPECT_NEAR(lag, -1.0, 0.3);
  }
}

// The window selects the tail of the overlap: a delay of 0.3 s for the first 70 s and -0.2 s for the last 20 s. A window of 15 s sees
// only the second regime (the lag comes back as +0.2), the window of 90 s mostly the first (-0.3).
TEST(LivgoA80, DtwWindowParameterSelectsTheTailOfTheOverlap) {
  std::vector<double> t, vh, vg;
  a80Speeds(90.0, 0.1, [](double ti) { return ti < 70.0 ? 0.3 : -0.2; }, &t, &vh, &vg);
  LivgoDtwParams tail;
  tail.window_s = 15.0;
  LivgoDtwParams whole;
  whole.window_s = 90.0;
  double lag_tail = 99.0, lag_whole = 99.0;
  ASSERT_TRUE(livo_gnss::livgoDtwLag(t, vg, t, vh, &lag_tail, tail));
  ASSERT_TRUE(livo_gnss::livgoDtwLag(t, vg, t, vh, &lag_whole, whole));
  EXPECT_NEAR(lag_tail, 0.2, 0.15);
  EXPECT_NEAR(lag_whole, -0.3, 0.15);
}

TEST(LivgoA80, DtwRefusesDegenerateParameters) {
  std::vector<double> t, vh, vg;
  a80Speeds(60.0, 0.1, [](double) { return 0.3; }, &t, &vh, &vg);
  double lag = 99.0;
  LivgoDtwParams p;
  p.grid_s = 0.0;
  EXPECT_FALSE(livo_gnss::livgoDtwLag(t, vg, t, vh, &lag, p));
  p = LivgoDtwParams();
  p.grid_s = -0.1;
  EXPECT_FALSE(livo_gnss::livgoDtwLag(t, vg, t, vh, &lag, p));
  p = LivgoDtwParams();
  p.window_s = 1.0;   // 10 samples < 20
  EXPECT_FALSE(livo_gnss::livgoDtwLag(t, vg, t, vh, &lag, p));
  p = LivgoDtwParams();
  p.band_s = 0.0;
  EXPECT_FALSE(livo_gnss::livgoDtwLag(t, vg, t, vh, &lag, p));
  p = LivgoDtwParams();
  p.window_s = std::nan("");
  EXPECT_FALSE(livo_gnss::livgoDtwLag(t, vg, t, vh, &lag, p));
}

// ---- the travel history of the frame least squares (init_travel_m) -------------------------------------------------------
TEST(LivgoA80, HistoryTravelIsHorizontalAndNeverDecreases) {
  LivgoInitHistory h;
  const Eigen::Vector3d g(0.0, 0.0, -9.81);
  auto at = [](const Eigen::Vector3d& p) {
    LivgoInitEpoch e;
    e.p_I = p;
    e.r_ecef = Eigen::Vector3d(1.0, 2.0, 3.0);
    return e;
  };
  h.add(at(Eigen::Vector3d(0, 0, 0)), g);
  EXPECT_EQ(h.travelM(), 0.0);
  h.add(at(Eigen::Vector3d(0, 0, 100.0)), g);
  EXPECT_EQ(h.travelM(), 0.0) << "a climb is not horizontal travel";
  h.add(at(Eigen::Vector3d(30.0, 40.0, 5.0)), g);
  EXPECT_NEAR(h.travelM(), 50.0, 1e-12);
  h.add(at(Eigen::Vector3d(1.0, 0.0, 0.0)), g);
  EXPECT_NEAR(h.travelM(), 50.0, 1e-12) << "coming back does not undo the travel";
  EXPECT_EQ(h.size(), 4u);
  // the plane follows the host gravity, not the W axes: with gravity along -x the horizontal plane is (y, z)
  LivgoInitHistory t;
  const Eigen::Vector3d gx(-9.81, 0.0, 0.0);
  t.add(at(Eigen::Vector3d(0, 0, 0)), gx);
  t.add(at(Eigen::Vector3d(80.0, 0, 0)), gx);
  EXPECT_EQ(t.travelM(), 0.0);
  t.add(at(Eigen::Vector3d(80.0, 3.0, 4.0)), gx);
  EXPECT_NEAR(t.travelM(), 5.0, 1e-12);
  // non-finite epochs are not added
  LivgoInitEpoch bad = at(Eigen::Vector3d(std::nan(""), 0, 0));
  t.add(bad, gx);
  EXPECT_EQ(t.size(), 3u);
}

TEST(LivgoA80, HistoryCapacityDropsTheOldestAndClearResets) {
  LivgoInitHistory h(5);
  const Eigen::Vector3d g(0.0, 0.0, -9.81);
  for (int i = 0; i < 12; ++i) {
    LivgoInitEpoch e;
    e.p_I = Eigen::Vector3d(i, 0.0, 0.0);
    h.add(e, g);
  }
  EXPECT_EQ(h.size(), 5u);
  EXPECT_EQ(h.epochs().front().p_I.x(), 7.0);
  EXPECT_EQ(h.epochs().back().p_I.x(), 11.0);
  EXPECT_NEAR(h.travelM(), 11.0, 1e-12) << "the travel keeps its reference (the first epoch since the clear) when the oldest epochs go";
  h.clear();
  EXPECT_TRUE(h.empty());
  EXPECT_EQ(h.travelM(), 0.0);
}

TEST(LivgoA80, HistorySamplesAreInTheEnuFrameOfTheAnchor) {
  const double lat = 30.0 * kDeg, lon = 110.0 * kDeg;
  const Eigen::Vector3d r0 = geo2ecef(lat, lon, 50.0);
  const Eigen::Matrix3d R_EN = enuRot(lat, lon);
  LivgoInitHistory h;
  LivgoInitEpoch e;
  e.p_I = Eigen::Vector3d(1.0, 2.0, 3.0);
  e.R_I = Eigen::Matrix3d::Identity();
  const Eigen::Vector3d enu(12.0, -7.0, 0.5);
  e.r_ecef = r0 + R_EN * enu;
  h.add(e, Eigen::Vector3d(0, 0, -9.81));
  const std::vector<LivgoInitSample> v = h.samples(r0, R_EN);
  ASSERT_EQ(v.size(), 1u);
  EXPECT_LT((v[0].p_r - enu).norm(), 1e-6);
  EXPECT_EQ(v[0].p_I, e.p_I);
}

// The trigger and the window of the registered rule: the gate opens at the first epoch whose travel reaches init_travel_m, and the
// least squares then runs on EVERY epoch collected up to there, not on a short sliding window of the latest ones. The all-epochs
// solution is better determined (smaller formal tilt sigma) than the solution of the last 20 epochs of the same track.
TEST(LivgoA80, TravelGateOpensAtTheDistanceAndTheSolveUsesEveryEpoch) {
  const double deg = M_PI / 180.0;
  const double init_travel_m = 50.0;
  const Eigen::Matrix3d R = Eigen::AngleAxisd(30.0 * deg, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
                            Eigen::AngleAxisd(0.5 * deg, Eigen::Vector3d::UnitY()).toRotationMatrix();
  const Eigen::Vector3d p(10.0, -5.0, 2.0), lever(0.30, -0.10, 0.40);
  const std::vector<livo_gnss::LivgoInitSample> track = a80Track(120, 1.0, R, p, lever, 0.5);   // 1 m per epoch, 0.5 m SPP-like noise
  LivgoInitHistory h;
  int open_at = -1;
  for (int i = 0; i < static_cast<int>(track.size()); ++i) {
    LivgoInitEpoch e;
    e.p_I = track[static_cast<size_t>(i)].p_I;
    e.R_I = track[static_cast<size_t>(i)].R_I;
    e.r_ecef = track[static_cast<size_t>(i)].p_r;   // identity "ECEF -> ENU" below: r0 = 0, R_EN = I
    h.add(e, Eigen::Vector3d(0, 0, -9.81));
    if (open_at < 0 && !(h.travelM() < init_travel_m)) open_at = i;
  }
  // p_I = (i, 8 sin(0.05 i), 0): the distance from epoch 0 is 49.27 m at epoch 49 and 50.2 m at epoch 50
  EXPECT_EQ(open_at, 50);
  // the solve at the gate: every epoch collected up to it
  LivgoInitHistory g;
  for (int i = 0; i <= open_at; ++i) {
    LivgoInitEpoch e;
    e.p_I = track[static_cast<size_t>(i)].p_I;
    e.R_I = track[static_cast<size_t>(i)].R_I;
    e.r_ecef = track[static_cast<size_t>(i)].p_r;
    g.add(e, Eigen::Vector3d(0, 0, -9.81));
  }
  const std::vector<livo_gnss::LivgoInitSample> all = g.samples(Eigen::Vector3d::Zero(), Eigen::Matrix3d::Identity());
  ASSERT_EQ(all.size(), static_cast<size_t>(open_at) + 1);
  const std::vector<livo_gnss::LivgoInitSample> window(all.end() - 20, all.end());   // a sliding window of the latest epochs
  const livo_gnss::LivgoInitEstimate est_all = a80Solve(all, lever);
  const livo_gnss::LivgoInitEstimate est_win = a80Solve(window, lever);
  ASSERT_TRUE(est_all.ok);
  ASSERT_TRUE(est_win.ok);
  const LivgoTiltDecision d_all = livgoTiltDecision(est_all, 0.0), d_win = livgoTiltDecision(est_win, 0.0);
  std::printf("[ A80 ] frame least squares at the 50 m gate: %zu epochs, tilt sigma %.3f deg; last 20 epochs: tilt sigma %.3f deg\n", all.size(),
              d_all.sigma_deg, d_win.sigma_deg);
  EXPECT_LT(d_all.sigma_deg, d_win.sigma_deg);
  EXPECT_LT((est_all.R - R).norm(), 0.1);
}

// ---- the registered tilt rule ---------------------------------------------------------------------------------------------
TEST(LivgoA80, TiltRuleKeepsAWellDeterminedTiltAndFallsBackOtherwise) {
  const double deg = M_PI / 180.0;
  const Eigen::Matrix3d R = Eigen::AngleAxisd(30.0 * deg, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
                            Eigen::AngleAxisd(0.5 * deg, Eigen::Vector3d::UnitY()).toRotationMatrix();
  const Eigen::Vector3d p(10.0, -5.0, 2.0), lever(0.30, -0.10, 0.40);
  // a long baseline: the formal tilt sigma is far below 0.2 deg
  const livo_gnss::LivgoInitEstimate good = a80Solve(a80Track(80, 5.0, R, p, lever, 0.05), lever);
  ASSERT_TRUE(good.ok);
  const LivgoTiltDecision keep = livgoTiltDecision(good, 0.2);
  EXPECT_LT(keep.sigma_deg, 0.2);
  EXPECT_TRUE(keep.use_ls_tilt);
  // the same estimate against a threshold below its sigma: the tilt is not taken
  const LivgoTiltDecision tight = livgoTiltDecision(good, 0.5 * keep.sigma_deg);
  EXPECT_FALSE(tight.use_ls_tilt);
  EXPECT_EQ(tight.sigma_deg, keep.sigma_deg) << "the reported sigma does not depend on the threshold";
  // a short, noisy, straight window (A73's failure): sigma above one degree -> fallback at 0.2
  std::vector<livo_gnss::LivgoInitSample> shortw;
  for (int i = 0; i < 20; ++i) {
    livo_gnss::LivgoInitSample s;
    s.p_I = Eigen::Vector3d(1.0 * i, 0.0, 0.0);
    s.R_I = Eigen::Matrix3d::Identity();
    s.p_r = Eigen::AngleAxisd(0.5 * deg, Eigen::Vector3d::UnitY()).toRotationMatrix() * (s.p_I + lever) +
            Eigen::Vector3d(std::sin(i), std::cos(i), std::sin(0.5 * i));
    shortw.push_back(s);
  }
  const livo_gnss::LivgoInitEstimate bad = a80Solve(shortw, lever);
  ASSERT_TRUE(bad.ok);
  const LivgoTiltDecision fall = livgoTiltDecision(bad, 0.2);
  EXPECT_GT(fall.sigma_deg, 1.0);
  EXPECT_FALSE(fall.use_ls_tilt);
  // max_deg = 0: no check, the least-squares tilt is always taken (the earlier behaviour); the sigma is still reported
  const LivgoTiltDecision off = livgoTiltDecision(bad, 0.0);
  EXPECT_TRUE(off.use_ls_tilt);
  EXPECT_EQ(off.sigma_deg, fall.sigma_deg);
  // a non-finite covariance counts as exceeding when the rule is on
  livo_gnss::LivgoInitEstimate nan_est = good;
  nan_est.cov(0, 0) = std::nan("");
  EXPECT_FALSE(livgoTiltDecision(nan_est, 0.2).use_ls_tilt);
  EXPECT_TRUE(livgoTiltDecision(nan_est, 0.0).use_ls_tilt);
}

// ---- freeze_yaw -----------------------------------------------------------------------------------------------------------
TEST(LivgoA80, FreezeYawPinsTheYawLikeTiltAndTranslation) {
  SceneOptions o;
  std::unique_ptr<Scene> sc = makeScene(o);
  Eigen::MatrixXd& Q = sc->aug.Q();
  std::mt19937_64 rng(5);
  std::normal_distribution<double> g(0.0, 1.0);
  const int n = sc->aug.dim();
  for (int i = 0; i < n; ++i)
    for (int j = 0; j < i; ++j) Q(i, j) = Q(j, i) = 0.1 * std::sqrt(Q(i, i) * Q(j, j)) * (g(rng) > 0 ? 1.0 : -1.0);
  for (int c = 0; c < n; ++c)
    for (int r = 0; r < HostIdx::dim; ++r) sc->aug.C()(r, c) = 1e-9 * g(rng);
  const Eigen::VectorXd x_pin = sc->aug.x();
  sc->aug.x() += 0.01 * Eigen::VectorXd::Ones(n);
  sc->aug.pinRegistrationExact(x_pin, /*include_yaw=*/true);
  for (int k = 0; k < 2; ++k) EXPECT_EQ(sc->aug.x()(sc->off_tilt + k), x_pin(sc->off_tilt + k));
  EXPECT_EQ(sc->aug.x()(sc->off_yaw), x_pin(sc->off_yaw)) << "the yaw mean is restored";
  for (int k = 0; k < 3; ++k) EXPECT_EQ(sc->aug.x()(sc->off_trans + k), x_pin(sc->off_trans + k));
  const int offs[3] = {sc->off_tilt, sc->off_yaw, sc->off_trans};
  const int dims[3] = {2, 1, 3};
  for (int b = 0; b < 3; ++b)
    for (int k = 0; k < dims[b]; ++k) {
      EXPECT_EQ(sc->aug.Q().row(offs[b] + k).norm(), 0.0) << "block " << b;
      EXPECT_EQ(sc->aug.Q().col(offs[b] + k).norm(), 0.0) << "block " << b;
      EXPECT_EQ(sc->aug.C().col(offs[b] + k).norm(), 0.0) << "block " << b;
    }
  EXPECT_EQ(sc->aug.Q()(sc->off_yaw, sc->off_yaw), 0.0) << "no yaw variance: no gain can reach it";
  EXPECT_TRUE(JointUpdater::posteriorPsd(sc->aug.jointCov(sc->x.P)));
  // the clock block next to them is untouched
  const int oc = sc->aug.find(Slot::Clk);
  EXPECT_GT(sc->aug.Q()(oc, oc), 0.0);
}

// The update: with the yaw frozen no row moves it (the K row of a zero variance is exactly zero) and the variance stays 0, while
// the A75 frame (yaw a state) lets the same rows change its mean and variance. The host still takes the TDCP displacement.
TEST(LivgoA80, FreezeYawLeavesNoGainOnTheYawAndTheHostStillMoves) {
  SceneOptions o;
  o.lever_m = 50.0;
  const Eigen::Vector3d dp(3e-3, -1e-3, 0.5e-3);
  const Eigen::Vector3d disp(0.1, 0.02, 0.0);
  const GainResult a75 = runToyUpdate(o, /*exact=*/true, /*share=*/true, disp, dp, /*freeze_yaw=*/false);
  const GainResult frozen = runToyUpdate(o, /*exact=*/true, /*share=*/true, disp, dp, /*freeze_yaw=*/true);
  ASSERT_TRUE(a75.committed);
  ASSERT_TRUE(frozen.committed);
  std::printf("[ A80 ] yaw after one TDCP epoch 50 m from the pivot: A75 frame sigma ratio %.3f, mean shift %.3e rad; frozen: ratio %.3f, shift %.3e; host gain %.3f / %.3f\n",
              a75.yaw_ratio, a75.yaw_shift, frozen.yaw_ratio, frozen.yaw_shift, a75.gain, frozen.gain);
  EXPECT_GT(a75.yaw_ratio, 0.0);
  EXPECT_LT(a75.yaw_ratio, 1.0);
  EXPECT_NE(a75.yaw_shift, 0.0);
  // K's yaw row is zero; the 2e-16 is the update's own wrap of the yaw into (-pi, pi], which the facade's pin undoes after every update
  EXPECT_NEAR(frozen.yaw_shift, 0.0, 1e-14) << "K's yaw row is zero";
  EXPECT_EQ(frozen.yaw_ratio, 0.0) << "and the yaw variance stays an exact zero";
  EXPECT_GT(frozen.gain, 0.5) << "the host takes the rows in either case";
}

// pinRegistrationExact without the new argument is the A75 pin (the yaw stays a state).
TEST(LivgoA80, ExactPinDefaultStillLeavesTheYaw) {
  SceneOptions o;
  std::unique_ptr<Scene> sc = makeScene(o);
  const Eigen::VectorXd x_pin = sc->aug.x();
  const double var_yaw = sc->aug.Q()(sc->off_yaw, sc->off_yaw);
  sc->aug.x()(sc->off_yaw) += 0.1;
  sc->aug.pinRegistrationExact(x_pin);
  EXPECT_EQ(sc->aug.x()(sc->off_yaw), x_pin(sc->off_yaw) + 0.1);
  EXPECT_EQ(sc->aug.Q()(sc->off_yaw, sc->off_yaw), var_yaw);
}

// ---- the facade's construction-time checks -------------------------------------------------------------------------------
TEST(LivgoA80, FacadeRefusesUnusableNumbersAndSaysWhenASwitchCannotBeHonoured) {
  Config cfg;
  cfg.enable = true;
  cfg.ephem.ura_semantics = Config::Ephem::UraSemantics::Metres;
  LivgoConfig ok;
  ok.enable = true;
  EXPECT_NO_THROW({ GnssFusionLivgo f(cfg, ok); (void)f; });   // the defaults, and the A80 arms' values below
  {
    LivgoConfig l = ok;
    l.freeze_yaw = true;
    l.frame_exact = true;
    l.init_travel_m = 50.0;
    l.init_tilt_sigma_max_deg = 0.2;
    l.cn0_hard = true;
    l.primary_only = true;
    l.dtw_grid_s = 0.1;
    l.dtw_window_s = 40.0;
    EXPECT_NO_THROW({ GnssFusionLivgo f(cfg, l); (void)f; });
  }
  struct Bad {
    const char* what;
    void (*mutate)(LivgoConfig&);
  };
  const Bad bad[] = {
      {"negative travel", [](LivgoConfig& l) { l.init_travel_m = -1.0; }},
      {"NaN travel", [](LivgoConfig& l) { l.init_travel_m = std::nan(""); }},
      {"negative tilt sigma", [](LivgoConfig& l) { l.init_tilt_sigma_max_deg = -0.2; }},
      {"zero grid", [](LivgoConfig& l) { l.dtw_grid_s = 0.0; }},
      {"window under 20 steps", [](LivgoConfig& l) { l.dtw_window_s = 1.5; }},
      {"NaN window", [](LivgoConfig& l) { l.dtw_window_s = std::nan(""); }},
      {"cn0 above 70", [](LivgoConfig& l) { l.cn0_hard = true; l.cn0_min_dbhz = 80.0; }},
      {"elevation 90", [](LivgoConfig& l) { l.cn0_hard = true; l.el_min_deg = 90.0; }},
  };
  for (const Bad& b : bad) {
    LivgoConfig l = ok;
    b.mutate(l);
    EXPECT_THROW({ GnssFusionLivgo f(cfg, l); (void)f; }, std::invalid_argument) << b.what;
  }
  // a cn0 / elevation value outside the range is only an error when cn0_hard uses it
  {
    LivgoConfig l = ok;
    l.cn0_min_dbhz = 80.0;
    EXPECT_NO_THROW({ GnssFusionLivgo f(cfg, l); (void)f; });
  }
  // switches the configuration cannot honour are reported once at start-up
  {
    LivgoConfig l = ok;
    l.freeze_yaw = true;   // without frame_exact
    l.paper_init = false;
    l.init_travel_m = 50.0;
    l.init_tilt_sigma_max_deg = 0.2;
    GnssFusionLivgo f(cfg, l);
    EventSink sink;
    f.setDiagnosticsSink(&sink);
    EXPECT_TRUE(sink.has("freeze_yaw without frame_exact"));
    EXPECT_TRUE(sink.has("init_travel_m without paper_init"));
    EXPECT_TRUE(sink.has("init_tilt_sigma_max_deg without paper_init"));
  }
  {
    LivgoConfig l = ok;   // the defaults of A80 say nothing
    GnssFusionLivgo f(cfg, l);
    EventSink sink;
    f.setDiagnosticsSink(&sink);
    EXPECT_FALSE(sink.has("ignored"));
  }
  {
    LivgoConfig l = ok;
    l.dual_only = true;
    l.cn0_hard = true;
    GnssFusionLivgo f(cfg, l);
    EventSink sink;
    f.setDiagnosticsSink(&sink);
    EXPECT_TRUE(sink.has("need the full arm"));
  }
}

// ---- AMENDMENTS A81: livgo.tdcp_raim_both_ends --------------------------------------------------------------------------------
// The default is the behaviour before A81; the pair-start selection (livgoTdcpStartMembers, which the facade calls where the pair's
// k0 members are chosen) with the switch off is the usable satellites in order whatever the epoch's RAIM said, with it on it also
// leaves out the satellites that epoch's robust least squares rejected.
TEST(LivgoA81, SwitchDefaultsToTheBehaviourBeforeA81) {
  const LivgoConfig c;
  EXPECT_FALSE(c.tdcp_raim_both_ends);
}

namespace {
struct StartScene {
  // six satellites at an epoch; the third has no usable carrier (no geometry or the carrier mask refused it)
  std::vector<char> usable{1, 1, 0, 1, 1, 1};
  std::vector<int> ids{11, 12, 13, 14, 15, 16};
};
}  // namespace

TEST(LivgoA81, StartMembersWithTheSwitchOffAreTheUsableSatellitesWhateverRaimSaid) {
  const StartScene s;
  const std::vector<int> all = {0, 1, 3, 4, 5};
  EXPECT_EQ(livgoTdcpStartMembers(false, s.usable, s.ids, {}), all);
  EXPECT_EQ(livgoTdcpStartMembers(false, s.usable, s.ids, {12, 14}), all);              // rejected satellites still open a pair
  EXPECT_EQ(livgoTdcpStartMembers(false, s.usable, s.ids, {11, 12, 13, 14, 15, 16}), all);
}

TEST(LivgoA81, StartMembersWithTheSwitchOnDropTheSatellitesTheEpochsRaimRejected) {
  const StartScene s;
  EXPECT_EQ(livgoTdcpStartMembers(true, s.usable, s.ids, {}), (std::vector<int>{0, 1, 3, 4, 5}));   // nothing rejected: as off
  EXPECT_EQ(livgoTdcpStartMembers(true, s.usable, s.ids, {12, 14}), (std::vector<int>{0, 4, 5}));
  // a rejected satellite that was unusable anyway, and one that is not in the epoch, change nothing
  EXPECT_EQ(livgoTdcpStartMembers(true, s.usable, s.ids, {13, 99}), (std::vector<int>{0, 1, 3, 4, 5}));
  // the order is the epoch's, the indices are positions in it (not ids)
  EXPECT_EQ(livgoTdcpStartMembers(true, s.usable, s.ids, {11}), (std::vector<int>{1, 3, 4, 5}));
  EXPECT_TRUE(livgoTdcpStartMembers(true, s.usable, s.ids, {11, 12, 14, 15, 16}).empty());
  EXPECT_TRUE(livgoTdcpStartMembers(true, {}, {}, {1}).empty());
  // the two vectors are read up to the shorter one
  EXPECT_EQ(livgoTdcpStartMembers(true, s.usable, std::vector<int>{11, 12, 13}, {12}), (std::vector<int>{0}));
}

TEST(LivgoA81, FacadeAcceptsTheSwitchAndSaysWhenItCannotBeHonoured) {
  Config cfg;
  cfg.enable = true;
  cfg.ephem.ura_semantics = Config::Ephem::UraSemantics::Metres;
  {
    LivgoConfig l;
    l.enable = true;
    l.tdcp_raim_both_ends = true;
    EXPECT_NO_THROW({ GnssFusionLivgo f(cfg, l); (void)f; });
    GnssFusionLivgo f(cfg, l);
    EventSink sink;
    f.setDiagnosticsSink(&sink);
    EXPECT_FALSE(sink.has("ignored"));   // honoured by the full arm, nothing to say
  }
  {
    LivgoConfig l;   // the T-plus-Hessian-switch arm (dual_only) has no TDCP pairing of its own to change
    l.enable = true;
    l.dual_only = true;
    l.tdcp_raim_both_ends = true;
    GnssFusionLivgo f(cfg, l);
    EventSink sink;
    f.setDiagnosticsSink(&sink);
    EXPECT_TRUE(sink.has("need the full arm"));
  }
}

// =====================================================================================================================================
// AMENDMENTS A82: the initialisation in the paper's order. The DTW at the travel gate with the author's acceptance test (the score functions
// are ports of FAST-LIVO2-RTK's; golden numbers below), the history re-pairing at the compensated times, livgo.single_init.
//   LivgoA82   defaults; the ports against the author's functions; the sign of the lag; the acceptance conditions and their boundaries; the
//              DTW over a speed history; the pose track and the re-pairing; the frame least squares with a delayed GNSS stream, with and
//              without the re-pairing; the re-initialisation gate; the facade's start-up checks and the new event.
// =====================================================================================================================================
namespace {

// The generator of the golden numbers. The same text is in the reference harness (/data2/fl2g/_dev/a82/author_score_ref.cpp on the build
// host), which compiles the author's normalizeVelocitySeries, interpolateLinear and scoreVelocityOffset verbatim (FAST-LIVO2-RTK
// src/optimization.cpp, sha256 8825c2ed...) in the project image (g++ 9.4, -O0 -ffp-contract=off) and printed the numbers. The author's code is
// GPL-2.0 and stays out of this BSD-3 repository; only its output is here.
double a82Motion(double t) {
  const double u = t - 1000.0;
  return 8.0 + 5.0 * std::sin(0.05 * u) + 2.0 * std::sin(0.31 * u) + 0.3 * std::cos(7.1 * u);
}
void a82Series(int n, std::vector<double>* th, std::vector<double>* vh, std::vector<double>* tg, std::vector<double>* vg) {
  for (int i = 0; i < n; ++i) {
    const double t = 1000.0 + 0.1 * i + 0.004 * std::sin(0.37 * i);
    th->push_back(t);
    vh->push_back(a82Motion(t));
  }
  for (int j = 0; j < n; ++j) {
    if (j == 17 || j == 18 || j == 211) continue;   // three GNSS epochs missing
    const double t = 1000.0 + 0.1 * j + 0.003 * std::cos(0.51 * j);
    tg->push_back(t);
    vg->push_back(a82Motion(t - 0.4) + 0.2 * std::sin(13.3 * j));   // GNSS stamps late by 0.4 s: stamp t carries the motion of t - 0.4
  }
}

// A deterministic white sequence (LCG), unit variance, for the uncorrelated cases.
std::vector<double> a82Noise(int n, unsigned seed) {
  std::vector<double> v;
  unsigned s = seed;
  for (int i = 0; i < n; ++i) {
    double acc = 0.0;
    for (int k = 0; k < 12; ++k) {   // sum of 12 uniforms - 6: mean 0, variance 1
      s = s * 1664525u + 1013904223u;
      acc += static_cast<double>(s >> 8) / 16777216.0;
    }
    v.push_back(acc - 6.0);
  }
  return v;
}

}  // namespace

TEST(LivgoA82, SwitchesSitAtThePreA82Behaviour) {
  const LivgoConfig c;
  EXPECT_FALSE(c.dtw_at_init);
  EXPECT_FALSE(c.single_init);
  EXPECT_EQ(std::string(eventKindName(EventKind::REINIT_SUPPRESSED)), "REINIT_SUPPRESSED");
  Event e;
  e.kind = EventKind::REINIT_SUPPRESSED;
  e.magnitude = 3.0;
  e.detail = "single_init: branch=heading_yaw suppressed=3 total=3";
  EXPECT_NE(e.logLine().find(" REINIT_SUPPRESSED magnitude=3 "), std::string::npos) << e.logLine();
  // the old kinds keep their names (the new one was appended before Count)
  EXPECT_EQ(std::string(eventKindName(EventKind::FRAME_JUMP)), "FRAME_JUMP");
  EXPECT_EQ(static_cast<int>(EventKind::REINIT_SUPPRESSED) + 1, static_cast<int>(EventKind::Count));
}

// The three ports against the author's functions: same inputs, the numbers the author's code prints. 1e-12 absolute.
TEST(LivgoA82, ScoreFunctionsMatchTheAuthorsCode) {
  std::vector<double> th, vh, tg, vg;
  a82Series(400, &th, &vh, &tg, &vg);
  ASSERT_EQ(th.size(), 400u);
  ASSERT_EQ(tg.size(), 397u);
  std::vector<double> nh = vh, ng = vg;
  double sd_h = 0.0, sd_g = 0.0;
  ASSERT_TRUE(livgoNormalizeSeries(&nh, &sd_h));
  ASSERT_TRUE(livgoNormalizeSeries(&ng, &sd_g));
  EXPECT_GT(sd_h, 1.0);
  EXPECT_GT(sd_g, 1.0);
  const double tol = 1e-12;
  EXPECT_NEAR(nh[0], -1.8909290580178499, tol);
  EXPECT_NEAR(nh[200], 0.14718421092865025, tol);
  EXPECT_NEAR(nh[399], 0.51514325163210051, tol);
  EXPECT_NEAR(ng[0], -2.3629369252018275, tol);
  EXPECT_NEAR(ng[199], 0.18858238075316811, tol);
  EXPECT_NEAR(ng.back(), 0.086950706044999049, tol);
  struct Gold {
    double offset, score;
    int pairs;
  };
  const double kInf = std::numeric_limits<double>::infinity();
  const Gold gold[] = {{-0.4, 0.99647140863135852, 395},  {-0.35, 0.99611783711402824, 396}, {0.0, 0.96667507142740383, 398},
                       {0.4, 0.97317589569135587, 396},   {-2.0, 0.9005314968775503, 379},   {2.0, 0.79743247076515844, 380},
                       {-10.0, -0.22722515222231457, 300}, {10.0, -0.24872785412122542, 299}, {-50.0, -kInf, 0},
                       {-39.0, -kInf, 9},                 {-35.0, 0.89369027418316294, 50}};
  for (const Gold& g : gold) {
    int pairs = -1;
    const double s = livgoScoreVelocityOffset(th, nh, tg, ng, g.offset, &pairs);
    EXPECT_EQ(pairs, g.pairs) << "offset " << g.offset;
    if (std::isinf(g.score)) {
      EXPECT_TRUE(std::isinf(s) && s < 0.0) << "offset " << g.offset << " (fewer than 30 pairs scores -inf)";
    } else {
      EXPECT_NEAR(s, g.score, tol) << "offset " << g.offset;
    }
  }
  // normalizeVelocitySeries' refusals
  {
    std::vector<double> flat;
    for (int i = 0; i < 40; ++i) flat.push_back(3.0 + 0.01 * std::sin(i));
    EXPECT_FALSE(livgoNormalizeSeries(&flat));
    EXPECT_NEAR(flat[0], 3.0 + 0.01 * std::sin(0), 0.0) << "a refused series is left as it was";
    std::vector<double> s19, s20;
    for (int i = 0; i < 20; ++i) s20.push_back(5.0 + (i % 2 ? 1.0 : -1.0));
    for (int i = 0; i < 19; ++i) s19.push_back(5.0 + (i % 2 ? 1.0 : -1.0));
    EXPECT_FALSE(livgoNormalizeSeries(&s19)) << "19 samples";
    EXPECT_TRUE(livgoNormalizeSeries(&s20)) << "20 samples";
    std::vector<double> lo, hi;
    for (int i = 0; i < 40; ++i) lo.push_back(5.0 + (i % 2 ? 0.0499 : -0.0499));
    for (int i = 0; i < 40; ++i) hi.push_back(5.0 + (i % 2 ? 0.0501 : -0.0501));
    double sd_lo = 0.0, sd_hi = 0.0;
    EXPECT_FALSE(livgoNormalizeSeries(&lo, &sd_lo)) << "std 0.0499 m/s";
    EXPECT_TRUE(livgoNormalizeSeries(&hi, &sd_hi)) << "std 0.0501 m/s";
    EXPECT_NEAR(sd_lo, 0.0499, 1e-12);
    EXPECT_NEAR(sd_hi, 0.0501, 1e-12);
    EXPECT_NEAR(hi[0], -0.99999999999999956, tol);
    std::vector<double> empty;
    double sd = 1.0;
    EXPECT_FALSE(livgoNormalizeSeries(&empty, &sd));
    EXPECT_TRUE(std::isnan(sd));
  }
  // interpolateLinear
  {
    struct Q {
      double t;
      bool ok;
      double v;
    };
    const Q q[] = {{999.0, false, 0.0},
                   {tg[0], true, 7.3707409411309426},
                   {tg[0] - 1e-9, false, 0.0},
                   {0.5 * (tg[16] + tg[17]), true, 9.0263422492778798},
                   {tg[17], true, 9.357850582573974},
                   {1020.0, true, 12.080594519507645},
                   {1001.75, true, 9.030568185512351},
                   {0.5 * (tg[208] + tg[209]), true, 12.27576698330537},
                   {tg.back(), true, 11.657708447864858},
                   {tg.back() + 1e-9, false, 0.0}};
    for (const Q& x : q) {
      double v = -777.0;
      const bool ok = livgoInterpolateLinear(tg, vg, x.t, &v);
      EXPECT_EQ(ok, x.ok) << "t " << x.t;
      if (ok && x.ok) {
        EXPECT_NEAR(v, x.v, tol) << "t " << x.t;
      }
    }
    // equal times at the end: the author's code takes the last value (upper_bound runs to the end before the zero-width test)
    const std::vector<double> two_t = {0.0, 0.0}, two_v = {1.0, 2.0};
    double v = -777.0;
    EXPECT_TRUE(livgoInterpolateLinear(two_t, two_v, 0.0, &v));
    EXPECT_EQ(v, 2.0);
    EXPECT_FALSE(livgoInterpolateLinear(std::vector<double>{1.0}, std::vector<double>{1.0}, 1.0, &v)) << "fewer than two samples";
  }
}

// The sign. The GNSS stamp t carries the motion of t - d (a stream that is late by d); livgoDtwLag returns -d, the facade adds it to the
// GNSS time map (stamp + lag = the host time of the same motion), and the author's fine_offset - the argument of scoreVelocityOffset, applied
// by initialAlign as gps.stamp += fine_offset - is the same number.
TEST(LivgoA82, LagAndAuthorsFineOffsetAreTheSameNumber) {
  const double d = 0.3;
  std::vector<double> t, vh, vg;
  a80Speeds(60.0, 0.1, [d](double) { return d; }, &t, &vh, &vg);   // vg(t) = f(t - d): the GNSS stamps are late by d
  double lag = 99.0;
  ASSERT_TRUE(livgoDtwLag(t, vg, t, vh, &lag));
  EXPECT_LT(lag, 0.0);
  EXPECT_NEAR(lag, -d, 0.15);
  std::vector<double> nh = vh, ng = vg;
  ASSERT_TRUE(livgoNormalizeSeries(&nh));
  ASSERT_TRUE(livgoNormalizeSeries(&ng));
  // the author's search (initialAlign: -10..10 s in 0.05 s steps, the best score) lands on -d too
  double best = 0.0, best_score = -std::numeric_limits<double>::infinity();
  for (int k = -200; k <= 200; ++k) {
    const double off = 0.05 * k;
    const double s = livgoScoreVelocityOffset(t, nh, t, ng, off);
    if (s > best_score) {
      best_score = s;
      best = off;
    }
  }
  EXPECT_NEAR(best, -d, 0.051);
  EXPECT_NEAR(lag, best, 0.15);
  // reading the GNSS series at (host time - offset) is the same as stamping it at (stamp + offset): the facade's addOffset(lag)
  std::vector<double> t_shifted = t;
  for (double& x : t_shifted) x += lag;
  int p1 = 0, p2 = 0;
  const double via_offset = livgoScoreVelocityOffset(t, nh, t, ng, lag, &p1);
  const double via_stamps = livgoScoreVelocityOffset(t, nh, t_shifted, ng, 0.0, &p2);
  EXPECT_NEAR(via_offset, via_stamps, 1e-12);
  EXPECT_EQ(p1, p2);
  // and it helps: the compensated stamps agree with the host better than the raw ones
  const double raw = livgoScoreVelocityOffset(t, nh, t, ng, 0.0);
  EXPECT_GT(via_stamps, raw);
  // the wrong sign does not
  EXPECT_LT(livgoScoreVelocityOffset(t, nh, t, ng, -lag), raw);
}

// The acceptance conditions on their own, at their boundaries (the three numbers the author's initialAlign reads).
TEST(LivgoA82, AcceptanceConditionsAndBoundaries) {
  const double kInf = std::numeric_limits<double>::infinity();
  const char* why = "";
  EXPECT_TRUE(livgoDtwAccept(-0.3, 0.9, 0.7, &why));
  EXPECT_STREQ(why, "adopted");
  // score at the lag: strictly above 0.25 (0.25 itself is refused), finite
  EXPECT_FALSE(livgoDtwAccept(-0.3, 0.25, -kInf, &why));
  EXPECT_STREQ(why, "score_at_lag_not_above_0.25");
  EXPECT_TRUE(livgoDtwAccept(-0.3, 0.2500001, -kInf, &why));
  EXPECT_FALSE(livgoDtwAccept(-0.3, std::nan(""), 0.0, &why));
  EXPECT_FALSE(livgoDtwAccept(-0.3, -kInf, 0.0, &why));
  EXPECT_STREQ(why, "score_at_lag_not_above_0.25");
  // against the zero lag: more than 0.05 better; a zero-lag score that is not finite (-inf: fewer than 30 pairs) always counts as improved
  EXPECT_FALSE(livgoDtwAccept(-0.3, 0.85, 0.81, &why));
  EXPECT_STREQ(why, "no_gain_over_zero_lag");
  EXPECT_TRUE(livgoDtwAccept(-0.3, 0.85, 0.79, &why));
  EXPECT_FALSE(livgoDtwAccept(-0.3, 0.85, 0.85, &why)) << "no gain at all";
  EXPECT_FALSE(livgoDtwAccept(-0.3, 0.85, 0.95, &why)) << "the zero lag is better";
  EXPECT_TRUE(livgoDtwAccept(-0.3, 0.85, -kInf, &why));
  EXPECT_TRUE(livgoDtwAccept(-0.3, 0.85, std::nan(""), &why)) << "!isfinite(zero_score) is improved in the author's code";
  // the lag: finite and at most 10 s in magnitude, both signs
  EXPECT_TRUE(livgoDtwAccept(10.0, 0.9, 0.5));
  EXPECT_TRUE(livgoDtwAccept(-10.0, 0.9, 0.5));
  EXPECT_FALSE(livgoDtwAccept(10.001, 0.9, 0.5, &why));
  EXPECT_STREQ(why, "lag_out_of_range");
  EXPECT_FALSE(livgoDtwAccept(-10.001, 0.9, 0.5));
  EXPECT_FALSE(livgoDtwAccept(std::nan(""), 0.9, 0.5));
  EXPECT_FALSE(livgoDtwAccept(kInf, 0.9, 0.5));
  // without a reason pointer
  EXPECT_TRUE(livgoDtwAccept(0.1, 0.9, 0.5));
}

// The DTW over a speed history, end to end through livgoDtwInit: a clear delay is taken; each refusal says which condition it was.
TEST(LivgoA82, DtwInitTakesAClearDelayAndRefusesTheRest) {
  {
    std::vector<double> t, vh, vg;
    a80Speeds(60.0, 0.1, [](double) { return 0.5; }, &t, &vh, &vg);
    const LivgoDtwInit r = livgoDtwInit(t, vg, vh, 0.1);
    EXPECT_TRUE(r.dtw_ok);
    EXPECT_TRUE(r.adopted) << r.reason;
    EXPECT_STREQ(r.reason, "adopted");
    EXPECT_NEAR(r.lag_s, -0.5, 0.2);
    EXPECT_EQ(r.applied_lag_s, r.lag_s);
    EXPECT_GT(r.score_lag, 0.25);
    EXPECT_GT(r.score_lag - r.score_zero, 0.05);
    EXPECT_GT(r.sd_gnss, 0.05);
    EXPECT_GT(r.sd_host, 0.05);
    EXPECT_EQ(r.n, static_cast<int>(t.size()));
    EXPECT_EQ(r.pairs_zero, r.n) << "host and GNSS series share their time stamps; the zero lag pairs every sample";
    std::printf("[ A82 ] DTW at init, delay 0.5 s over 60 s: lag %.2f s, score %.4f, zero-lag score %.4f, pairs %d / %d\n", r.lag_s, r.score_lag,
                r.score_zero, r.pairs_lag, r.pairs_zero);
  }
  {   // identical series: the DTW finds no delay (lag 0), so there is nothing to gain over the zero lag
    std::vector<double> t, vh, vg;
    a80Speeds(60.0, 0.1, [](double) { return 0.0; }, &t, &vh, &vg);
    const LivgoDtwInit r = livgoDtwInit(t, vg, vh, 0.1);
    EXPECT_TRUE(r.dtw_ok);
    EXPECT_FALSE(r.adopted);
    EXPECT_STREQ(r.reason, "no_gain_over_zero_lag");
    EXPECT_EQ(r.lag_s, 0.0);
    EXPECT_EQ(r.applied_lag_s, 0.0);
  }
  {   // the harness series: a real 0.4 s delay, but the speed is so smooth that the zero lag already scores 0.967 (best 0.996): the
      // author's test needs a gain above 0.05 and does not take it
    std::vector<double> th, vh, tg, vg;
    a82Series(400, &th, &vh, &tg, &vg);
    // the DTW wants one shared time axis: GNSS speeds read at the host stamps
    std::vector<double> vg_at_host(th.size());
    for (size_t i = 0; i < th.size(); ++i) {
      double v = 0.0;
      if (!livgoInterpolateLinear(tg, vg, th[i], &v)) v = vg.front();
      vg_at_host[i] = v;
    }
    const LivgoDtwInit r = livgoDtwInit(th, vg_at_host, vh, 0.1);
    EXPECT_TRUE(r.dtw_ok);
    EXPECT_FALSE(r.adopted);
    EXPECT_STREQ(r.reason, "no_gain_over_zero_lag");
    EXPECT_EQ(r.applied_lag_s, 0.0) << "not taken: the offset stays 0";
    EXPECT_GT(r.score_lag, 0.25);
    EXPECT_LT(r.score_lag - r.score_zero, 0.05);
  }
  {   // a flat history: the standard deviation is far below 0.05 m/s
    std::vector<double> t, vh, vg;
    for (int i = 0; i < 300; ++i) {
      t.push_back(0.1 * i);
      vh.push_back(3.0 + 0.01 * std::sin(0.7 * i));
      vg.push_back(3.0 + 0.01 * std::sin(0.7 * i + 0.2));
    }
    const LivgoDtwInit r = livgoDtwInit(t, vg, vh, 0.1);
    EXPECT_FALSE(r.adopted);
    EXPECT_STREQ(r.reason, "series_short_or_flat");
    EXPECT_LT(r.sd_host, 0.05);
    EXPECT_LT(r.sd_gnss, 0.05);
    EXPECT_TRUE(std::isnan(r.score_lag)) << "no score is computed for a refused series";
    EXPECT_EQ(r.applied_lag_s, 0.0);
  }
  {   // fewer than 20 samples (19 epochs 0.25 s apart: 4.5 s, the DTW itself runs, the series test refuses)
    std::vector<double> t, vh, vg;
    for (int i = 0; i < 19; ++i) {
      t.push_back(0.25 * i);
      vh.push_back(5.0 + (i % 2 ? 1.0 : -1.0) * (1.0 + 0.1 * i));
      vg.push_back(5.0 + (i % 2 ? 1.0 : -1.0) * (1.0 + 0.1 * i));
    }
    const LivgoDtwInit r = livgoDtwInit(t, vg, vh, 0.1);
    EXPECT_FALSE(r.adopted);
    EXPECT_EQ(r.n, 19);
    EXPECT_TRUE(std::string(r.reason) == "series_short_or_flat" || std::string(r.reason) == "dtw_failed") << r.reason;
    EXPECT_EQ(r.applied_lag_s, 0.0);
  }
  {   // two unrelated white sequences: whatever lag the DTW finds, the correlation at a fixed lag is about zero
    std::vector<double> t;
    for (int i = 0; i < 600; ++i) t.push_back(0.1 * i);
    std::vector<double> vh = a82Noise(600, 1u), vg = a82Noise(600, 2u);
    for (double& x : vh) x += 8.0;
    for (double& x : vg) x += 8.0;
    const LivgoDtwInit r = livgoDtwInit(t, vg, vh, 0.1);
    EXPECT_TRUE(r.dtw_ok);
    EXPECT_FALSE(r.adopted);
    EXPECT_STREQ(r.reason, "score_at_lag_not_above_0.25");
    EXPECT_LT(r.score_lag, 0.25);
    EXPECT_EQ(r.applied_lag_s, 0.0);
  }
  {   // malformed input
    const LivgoDtwInit a = livgoDtwInit({}, {}, {}, 0.1);
    EXPECT_FALSE(a.adopted);
    EXPECT_STREQ(a.reason, "no_series");
    const LivgoDtwInit b = livgoDtwInit({0.0, 0.1, 0.2}, {1.0, 2.0}, {1.0, 2.0, 3.0}, 0.1);
    EXPECT_STREQ(b.reason, "no_series");
    std::vector<double> t, v;
    for (int i = 0; i < 100; ++i) {
      t.push_back(0.1 * i);
      v.push_back(std::sin(i));
    }
    const LivgoDtwInit c = livgoDtwInit(t, v, v, 0.0);   // a zero grid: the DTW refuses
    EXPECT_FALSE(c.dtw_ok);
    EXPECT_STREQ(c.reason, "dtw_failed");
  }
}

// ---- the pose track and the re-pairing ------------------------------------------------------------------------------------------------
namespace {

// A host motion whose position is a cubic polynomial of time (so the Hermite interpolation of the track is exact) and whose attitude turns at a
// constant rate about z (so the interpolation of the rotation is exact).
LivgoHostPose a82Pose(double t) {
  LivgoHostPose h;
  h.t = t;
  const double u = t - 100.0;
  h.p = Eigen::Vector3d(10.0 * u + 0.8 * u * u + 0.05 * u * u * u, 3.0 * u - 0.4 * u * u + 0.02 * u * u * u, 0.1 * u * u);
  h.v = Eigen::Vector3d(10.0 + 1.6 * u + 0.15 * u * u, 3.0 - 0.8 * u + 0.06 * u * u, 0.2 * u);
  h.R = Eigen::AngleAxisd(0.3 * u, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return h;
}

LivgoInitHistory a82History(int n_epochs, double t0, double step, bool with_speeds) {
  LivgoInitHistory h;
  for (int i = 0; i < n_epochs; ++i) {
    LivgoInitEpoch e;
    e.t_est = t0 + step * i;
    e.p_I = Eigen::Vector3d(-1.0, -1.0, -1.0);   // a pose that is not the track's, so a missed replacement shows
    e.r_ecef = Eigen::Vector3d(1.0, 2.0, 3.0 + i);
    if (with_speeds) {
      e.v_gnss = 5.0 + i;
      e.v_host = 6.0 + i;
    }
    h.add(e, Eigen::Vector3d(0, 0, -9.81));
  }
  return h;
}

}  // namespace

TEST(LivgoA82, HostPoseInterpolationIsExactForCubicPositionAndConstantRateRotation) {
  const LivgoHostPose a = a82Pose(103.0), b = a82Pose(103.1);
  for (double s : {0.0, 0.1, 0.37, 0.5, 0.93, 1.0}) {
    const double t = 103.0 + 0.1 * s;
    Eigen::Matrix3d R;
    Eigen::Vector3d p;
    livgoInterpolateHostPose(a, b, t, &R, &p);
    const LivgoHostPose truth = a82Pose(t);
    EXPECT_LT((p - truth.p).norm(), 1e-9) << "s " << s;
    EXPECT_LT((R - truth.R).norm(), 1e-12) << "s " << s;
  }
  // the ends are the samples
  Eigen::Matrix3d R;
  Eigen::Vector3d p;
  livgoInterpolateHostPose(a, b, a.t, &R, &p);
  EXPECT_EQ(p, a.p);
  EXPECT_EQ(R, a.R);
  // a snap: p = A sin(w t), h = 0.1 s; the error stays under h^4/384 * A w^4 = 5.3e-7 m
  const double A = 5.0, w = 0.8;
  auto sinPose = [&](double t) {
    LivgoHostPose h;
    h.t = t;
    h.p = Eigen::Vector3d(A * std::sin(w * t), 0.0, 0.0);
    h.v = Eigen::Vector3d(A * w * std::cos(w * t), 0.0, 0.0);
    return h;
  };
  double worst = 0.0;
  for (int k = 0; k < 200; ++k) {
    const double t0 = 7.0 + 0.1 * k;
    for (double s : {0.25, 0.5, 0.75}) {
      Eigen::Vector3d q;
      livgoInterpolateHostPose(sinPose(t0), sinPose(t0 + 0.1), t0 + 0.1 * s, nullptr, &q);
      worst = std::max(worst, std::fabs(q.x() - A * std::sin(w * (t0 + 0.1 * s))));
    }
  }
  EXPECT_LT(worst, 1e-6);
  std::printf("[ A82 ] Hermite pose interpolation of p = 5 sin(0.8 t) at 0.1 s: worst error %.2e m\n", worst);
  // a degenerate interval keeps the first sample
  livgoInterpolateHostPose(a, a, a.t, &R, &p);
  EXPECT_EQ(p, a.p);
}

TEST(LivgoA82, TrackAndSpeedSeriesOfTheHistory) {
  LivgoInitHistory h = a82History(10, 100.0, 0.1, true);
  EXPECT_EQ(h.trackSize(), 0u);
  h.addTrack(a82Pose(100.0));
  h.addTrack(a82Pose(100.1));
  h.addTrack(a82Pose(100.1));                 // not later than the last: ignored
  h.addTrack(a82Pose(100.05));                // earlier: ignored
  LivgoHostPose bad = a82Pose(100.2);
  bad.p.x() = std::nan("");
  h.addTrack(bad);                            // non-finite: ignored
  LivgoHostPose bad_t = a82Pose(100.3);
  bad_t.t = std::nan("");
  h.addTrack(bad_t);
  EXPECT_EQ(h.trackSize(), 2u);
  h.addTrack(a82Pose(100.2));
  EXPECT_EQ(h.trackSize(), 3u);
  // the speed series: every epoch has all three numbers
  LivgoSpeedSeries s = h.speedSeries();
  ASSERT_EQ(s.t.size(), 10u);
  EXPECT_EQ(s.t.front(), 100.0);
  EXPECT_EQ(s.v_gnss[3], 8.0);
  EXPECT_EQ(s.v_host[3], 9.0);
  // an epoch missing a number (no Doppler solution, no host speed, no time) is not in the series, in the history it stays
  LivgoInitHistory g;
  for (int i = 0; i < 4; ++i) {
    LivgoInitEpoch e;
    e.t_est = 1.0 + i;
    e.v_gnss = 2.0;
    e.v_host = 3.0;
    if (i == 1) e.v_gnss = std::nan("");
    if (i == 2) e.v_host = std::nan("");
    if (i == 3) e.t_est = std::nan("");
    g.add(e, Eigen::Vector3d(0, 0, -9.81));
  }
  EXPECT_EQ(g.size(), 4u);
  const LivgoSpeedSeries gs = g.speedSeries();
  ASSERT_EQ(gs.t.size(), 1u);
  EXPECT_EQ(gs.t[0], 1.0);
  // an epoch of the earlier arms carries none of them (default NaN): the series is empty
  LivgoInitHistory old;
  LivgoInitEpoch e;
  old.add(e, Eigen::Vector3d(0, 0, -9.81));
  EXPECT_TRUE(old.speedSeries().t.empty());
  // clear() takes the track with the epochs
  h.clear();
  EXPECT_EQ(h.trackSize(), 0u);
  EXPECT_TRUE(h.empty());
  // capacity: the track keeps the newest samples
  LivgoInitHistory cap(5);
  for (int i = 0; i < 12; ++i) cap.addTrack(a82Pose(100.0 + 0.1 * i));
  EXPECT_EQ(cap.trackSize(), 5u);
  EXPECT_NEAR(cap.track().front().t, 100.7, 1e-12);
}

TEST(LivgoA82, RepairPairsEveryEpochWithTheHostPoseAtItsCompensatedTime) {
  const double lag = -0.37;
  LivgoInitHistory h = a82History(300, 100.013, 0.1, false);   // epoch stamps off the track grid
  for (int k = 0; k <= 400; ++k) h.addTrack(a82Pose(99.0 + 0.1 * k + 0.02 * std::sin(k)));   // 0.08-0.12 s steps
  const LivgoRepairResult r = h.repairPairing(lag);
  // epochs with t_est + lag before the track starts (99.0 + 0.02 sin(0) = 99.0) cannot be paired... the first is at 100.013 - 0.37 = 99.643: inside
  EXPECT_EQ(r.dropped, 0);
  EXPECT_EQ(r.repaired, 300);
  ASSERT_EQ(h.size(), 300u);
  EXPECT_LE(r.max_gap_s, 0.13);
  double worst_p = 0.0, worst_R = 0.0;
  for (size_t i = 0; i < h.size(); ++i) {
    const LivgoInitEpoch& e = h.epochs()[i];
    const LivgoHostPose truth = a82Pose(100.013 + 0.1 * static_cast<double>(i) + lag);
    worst_p = std::max(worst_p, (e.p_I - truth.p).norm());
    worst_R = std::max(worst_R, (e.R_I - truth.R).norm());
    EXPECT_EQ(e.r_ecef.z(), 3.0 + static_cast<double>(i)) << "the GNSS position stays with its epoch";
  }
  EXPECT_LT(worst_p, 1e-9);
  EXPECT_LT(worst_R, 1e-12);
  // a positive lag runs past the end of the track: those epochs are dropped (here the track ends at 139.0 + 0.02 sin(400))
  LivgoInitHistory late = a82History(300, 100.013, 0.1, false);
  for (int k = 0; k <= 400; ++k) late.addTrack(a82Pose(99.0 + 0.1 * k));   // ends at 139.0
  const LivgoRepairResult r2 = late.repairPairing(+0.37);
  // epoch i: 100.013 + 0.1 i + 0.37 <= 139.0  <=>  i <= 385.17: all 300 epochs (last 129.913 + 0.37) are inside; make it bite with a lag of +10 s
  EXPECT_EQ(r2.dropped, 0);
  LivgoInitHistory late10 = a82History(300, 100.013, 0.1, false);
  for (int k = 0; k <= 400; ++k) late10.addTrack(a82Pose(99.0 + 0.1 * k));
  const LivgoRepairResult r3 = late10.repairPairing(+10.0);
  // 100.013 + 0.1 i + 10 <= 139.0  <=>  i <= 289.87: epochs 0..289 stay (290), 10 go
  EXPECT_EQ(r3.repaired, 290);
  EXPECT_EQ(r3.dropped, 10);
  EXPECT_EQ(late10.size(), 290u);
  // and one at the very start: lag -2 s drops the epochs before 101.0 + ... : 100.013 + 0.1 i - 2 < 99.0 <=> i < 9.87 -> 10 epochs
  LivgoInitHistory early = a82History(300, 100.013, 0.1, false);
  for (int k = 0; k <= 400; ++k) early.addTrack(a82Pose(99.0 + 0.1 * k));
  const LivgoRepairResult r4 = early.repairPairing(-2.0);
  EXPECT_EQ(r4.dropped, 10);
  EXPECT_EQ(r4.repaired, 290);
}

TEST(LivgoA82, RepairDropsWhatItCannotPairHonestly) {
  // a hole in the track wider than kLivgoRepairMaxGapS: epochs whose compensated time falls inside it are dropped, the rest are paired
  LivgoInitHistory h = a82History(300, 100.013, 0.1, false);
  const double hole_a = 99.0 + 0.1 * 110, hole_b = 99.0 + 0.1 * 135;   // the samples 111..134 are missing: a 2.5 s hole between these two
  for (int k = 0; k <= 400; ++k) {
    if (k > 110 && k < 135) continue;
    h.addTrack(a82Pose(99.0 + 0.1 * k));
  }
  const LivgoRepairResult r = h.repairPairing(-0.25);
  int in_hole = 0;
  for (int i = 0; i < 300; ++i) {
    const double tau = 100.013 + 0.1 * i - 0.25;
    if (tau > hole_a && tau < hole_b) ++in_hole;
  }
  EXPECT_GT(in_hole, 20);
  EXPECT_EQ(r.dropped, in_hole);
  EXPECT_EQ(r.repaired, 300 - in_hole);
  EXPECT_LE(r.max_gap_s, kLivgoRepairMaxGapS);
  for (const LivgoInitEpoch& e : h.epochs()) {
    const double tau = e.t_est - 0.25;
    const LivgoHostPose truth = a82Pose(tau);
    EXPECT_LT((e.p_I - truth.p).norm(), 1e-9);
  }
  // an epoch with no time stamp, and a track too short to interpolate in: dropped, not mis-paired
  LivgoInitHistory g = a82History(5, 100.0, 0.1, false);
  g.addTrack(a82Pose(100.0));
  const LivgoRepairResult r1 = g.repairPairing(0.0);
  EXPECT_EQ(r1.dropped, 5) << "a one-sample track pairs nothing";
  EXPECT_TRUE(g.empty());
  LivgoInitHistory k = a82History(3, 100.0, 0.1, false);
  for (int i = 0; i < 20; ++i) k.addTrack(a82Pose(99.5 + 0.1 * i));
  LivgoInitEpoch no_time;
  no_time.p_I = Eigen::Vector3d(-1, -1, -1);
  k.add(no_time, Eigen::Vector3d(0, 0, -9.81));
  const LivgoRepairResult r2 = k.repairPairing(0.0);
  EXPECT_EQ(r2.repaired, 3);
  EXPECT_EQ(r2.dropped, 1);
  // a non-finite lag pairs nothing
  LivgoInitHistory m = a82History(3, 100.0, 0.1, false);
  for (int i = 0; i < 20; ++i) m.addTrack(a82Pose(99.5 + 0.1 * i));
  EXPECT_EQ(m.repairPairing(std::nan("")).dropped, 3);
  // an epoch whose compensated time is a track sample's own time gets that sample's pose
  LivgoInitHistory y;
  LivgoInitEpoch e;
  e.t_est = 100.25;
  y.add(e, Eigen::Vector3d(0, 0, -9.81));
  for (int i = 0; i < 20; ++i) y.addTrack(a82Pose(99.5 + 0.25 * i));
  const LivgoHostPose stored = y.track()[2];   // t = 100.0
  ASSERT_EQ(y.repairPairing(-0.25).repaired, 1);
  ASSERT_EQ(y.size(), 1u);
  EXPECT_LT((y.epochs()[0].p_I - stored.p).norm(), 1e-9);
  EXPECT_LT((y.epochs()[0].R_I - stored.R).norm(), 1e-12);
}

// What the re-pairing is for: GNSS positions stamped t carry the host pose of t - d. Paired by stamp (no compensation) the frame least squares
// absorbs the delay as an error in rotation, translation and lever; paired with the host pose at the compensated time (lag = -d) it
// recovers the frame.
TEST(LivgoA82, FrameLeastSquaresNeedsTheRepairedPairingWhenTheGnssStreamIsLate) {
  const double deg = M_PI / 180.0;
  const Eigen::Matrix3d R = Eigen::AngleAxisd(30.0 * deg, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
                            Eigen::AngleAxisd(0.5 * deg, Eigen::Vector3d::UnitY()).toRotationMatrix();
  const Eigen::Vector3d p(10.0, -5.0, 2.0), lever(0.30, -0.10, 0.40);
  const double d = 0.5;
  auto hostAt = [](double t) {   // 10 m/s along a weaving track, the attitude turns with it
    LivgoHostPose h;
    const double u = t - 100.0;
    h.t = t;
    h.p = Eigen::Vector3d(10.0 * u, 8.0 * std::sin(0.5 * u), 0.0);
    h.v = Eigen::Vector3d(10.0, 4.0 * std::cos(0.5 * u), 0.0);
    h.R = Eigen::AngleAxisd(0.15 * u, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    return h;
  };
  LivgoInitHistory hist;
  for (int i = 0; i < 600; ++i) {
    const double stamp = 100.037 + 0.1 * i;
    const LivgoHostPose at_stamp = hostAt(stamp);          // what the facade pairs by stamp
    const LivgoHostPose carried = hostAt(stamp - d);       // the motion the GNSS position actually belongs to
    LivgoInitEpoch e;
    e.t_est = stamp;
    e.p_I = at_stamp.p;
    e.R_I = at_stamp.R;
    e.r_ecef = R * (carried.p + carried.R * lever) + p + 0.01 * Eigen::Vector3d(std::sin(i), std::cos(0.7 * i), 0.5 * std::sin(0.3 * i));
    hist.add(e, Eigen::Vector3d(0, 0, -9.81));
  }
  for (int k = 0; k <= 700; ++k) hist.addTrack(hostAt(99.0 + 0.1 * k));
  const auto solve = [&](const LivgoInitHistory& h) {
    return a80Solve(h.samples(Eigen::Vector3d::Zero(), Eigen::Matrix3d::Identity()), lever);
  };
  const livo_gnss::LivgoInitEstimate by_stamp = solve(hist);
  LivgoInitHistory fixed = hist;
  const LivgoRepairResult rr = fixed.repairPairing(-d);
  EXPECT_EQ(rr.dropped, 0);
  EXPECT_EQ(rr.repaired, 600);
  const livo_gnss::LivgoInitEstimate repaired = solve(fixed);
  ASSERT_TRUE(by_stamp.ok);
  ASSERT_TRUE(repaired.ok);
  std::printf("[ A82 ] frame least squares, GNSS 0.5 s late at 10 m/s: paired by stamp rms %.3f m (rotation error %.3f deg); re-paired rms %.4f m (%.4f deg)\n",
              by_stamp.rms_m, (by_stamp.R - R).norm() / deg, repaired.rms_m, (repaired.R - R).norm() / deg);
  EXPECT_GT(by_stamp.rms_m, 0.5);
  EXPECT_LT(repaired.rms_m, 0.02);
  EXPECT_LT((repaired.R - R).norm(), 2e-3);
  EXPECT_LT((repaired.p - p).norm(), 0.05);
  EXPECT_LT((repaired.lever - lever).norm(), 0.05);
  EXPECT_GT((by_stamp.R - R).norm(), (repaired.R - R).norm());
}

// ---- livgo.single_init: the re-initialisation gate -------------------------------------------------------------------------------------
TEST(LivgoA82, ReinitGateSuppressesAfterTheFirstActivationAndCounts) {
  using B = LivgoReinitBranch;
  {   // the switch off: every branch runs, before and after the first ACTIVATE
    LivgoReinitGate g;
    EXPECT_TRUE(g.allow(B::HeadingYaw));
    g.onActivated();
    EXPECT_TRUE(g.allow(B::HeadingYaw));
    EXPECT_TRUE(g.allow(B::Outage));
    EXPECT_TRUE(g.allow(B::ConditioningFailed));
    EXPECT_EQ(g.total(), 0);
  }
  {   // the switch on: nothing is suppressed before the first ACTIVATE (the first initialisation may be retried), everything afterwards
    LivgoReinitGate g;
    g.setSingleInit(true);
    EXPECT_FALSE(g.activated());
    EXPECT_TRUE(g.allow(B::HeadingYaw));
    EXPECT_TRUE(g.allow(B::Outage));
    EXPECT_TRUE(g.allow(B::ConditioningFailed));
    EXPECT_EQ(g.total(), 0);
    g.onActivated();
    EXPECT_TRUE(g.activated());
    EXPECT_FALSE(g.allow(B::HeadingYaw));
    EXPECT_FALSE(g.allow(B::HeadingYaw));
    EXPECT_FALSE(g.allow(B::Outage));
    EXPECT_FALSE(g.allow(B::ConditioningFailed));
    EXPECT_FALSE(g.allow(B::HeadingYaw));
    EXPECT_EQ(g.suppressed(B::HeadingYaw), 3);
    EXPECT_EQ(g.suppressed(B::Outage), 1);
    EXPECT_EQ(g.suppressed(B::ConditioningFailed), 1);
    EXPECT_EQ(g.total(), 5);
    EXPECT_STREQ(LivgoReinitGate::name(B::HeadingYaw), "heading_yaw");
    EXPECT_STREQ(LivgoReinitGate::name(B::Outage), "outage");
    EXPECT_STREQ(LivgoReinitGate::name(B::ConditioningFailed), "conditioning_failed");
  }
}

TEST(LivgoA82, FacadeAcceptsTheSwitchesAndSaysWhenTheyCannotBeHonoured) {
  Config cfg;
  cfg.enable = true;
  cfg.ephem.ura_semantics = Config::Ephem::UraSemantics::Metres;
  LivgoConfig arm;   // what LIVGO-literal.yaml sets
  arm.enable = true;
  arm.frame_exact = true;
  arm.freeze_yaw = true;
  arm.init_travel_m = 50.0;
  arm.init_tilt_sigma_max_deg = 0.2;
  arm.dtw_online = true;
  arm.dtw_at_init = true;
  arm.dtw_grid_s = 0.1;
  arm.single_init = true;
  {
    EXPECT_NO_THROW({ GnssFusionLivgo f(cfg, arm); (void)f; });
    GnssFusionLivgo f(cfg, arm);
    EventSink sink;
    f.setDiagnosticsSink(&sink);
    EXPECT_FALSE(sink.has("ignored")) << "the arm's own configuration says nothing";
  }
  {   // the DTW at init needs the DTW and the travel gate whose history it runs on
    LivgoConfig l = arm;
    l.dtw_online = false;
    GnssFusionLivgo f(cfg, l);
    EventSink sink;
    f.setDiagnosticsSink(&sink);
    EXPECT_TRUE(sink.has("dtw_at_init without dtw_online"));
  }
  {
    LivgoConfig l = arm;
    l.init_travel_m = 0.0;
    GnssFusionLivgo f(cfg, l);
    EventSink sink;
    f.setDiagnosticsSink(&sink);
    EXPECT_TRUE(sink.has("dtw_at_init without paper_init and init_travel_m > 0"));
  }
  {
    LivgoConfig l = arm;
    l.paper_init = false;
    GnssFusionLivgo f(cfg, l);
    EventSink sink;
    f.setDiagnosticsSink(&sink);
    EXPECT_TRUE(sink.has("dtw_at_init without paper_init and init_travel_m > 0"));
  }
  {   // single_init alone is honoured by the full arm
    LivgoConfig l;
    l.enable = true;
    l.single_init = true;
    GnssFusionLivgo f(cfg, l);
    EventSink sink;
    f.setDiagnosticsSink(&sink);
    EXPECT_FALSE(sink.has("ignored"));
  }
  {   // the T-plus-Hessian-switch arm has neither the DTW at init nor a re-initialisation of its own to hold
    LivgoConfig l;
    l.enable = true;
    l.dual_only = true;
    l.single_init = true;
    GnssFusionLivgo f(cfg, l);
    EventSink sink;
    f.setDiagnosticsSink(&sink);
    EXPECT_TRUE(sink.has("need the full arm"));
  }
  {
    LivgoConfig l;
    l.enable = true;
    l.dual_only = true;
    l.dtw_at_init = true;
    GnssFusionLivgo f(cfg, l);
    EventSink sink;
    f.setDiagnosticsSink(&sink);
    EXPECT_TRUE(sink.has("need the full arm"));
  }
  {   // and the defaults of A82 say nothing
    LivgoConfig l;
    l.enable = true;
    GnssFusionLivgo f(cfg, l);
    EventSink sink;
    f.setDiagnosticsSink(&sink);
    EXPECT_FALSE(sink.has("ignored"));
  }
}
