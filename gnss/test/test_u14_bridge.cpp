// SPDX-License-Identifier: BSD-3-Clause
// test_u14_bridge.cpp — TEST_PLAN §1 U14 (ImuBridge, DESIGN §4.1, AMENDMENTS A10, A33). Owner: WP-b.
//   1. noiseless high-rate truth, δ ≤ 0.1 s → bridged position error < 0.1 mm (plus evaluate() at the snapshot)
//   2. epochs before the current cycle start chain relative motion across cycles whose start states jump; a short
//      bridge_ring_cycles no longer drops cycles inside max_bridge_s (A33)
//   3. the ring keeps at least bridge_ring_cycles cycles and every cycle with t_k ≥ t_latest − (max_bridge_s + 0.1 s),
//      nothing older (A33)
//   4. 15 Hz LIO cycles with the default 4-cycle setting: δ = 0.28 s is bridged, exactly and to < 0.1 mm (A33)
//   5. interpolate() equals the host propagation IMU_Processing.cpp:412-421 term by term
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <random>
#include <stdexcept>
#include <vector>

#include "internal/bridge.hpp"

using namespace livo_gnss;

namespace {

// ---- analytic truth trajectory ----------------------------------------------------------------------------
struct Truth {
  Eigen::Vector3d p(double t) const {
    return Eigen::Vector3d(2.0 * std::sin(0.8 * t), 1.5 * std::cos(0.6 * t) + 0.3 * t, 0.4 * std::sin(1.1 * t + 0.3));
  }
  Eigen::Vector3d v(double t) const {
    return Eigen::Vector3d(1.6 * std::cos(0.8 * t), -0.9 * std::sin(0.6 * t) + 0.3, 0.44 * std::cos(1.1 * t + 0.3));
  }
  Eigen::Vector3d a(double t) const {
    return Eigen::Vector3d(-1.28 * std::sin(0.8 * t), -0.54 * std::cos(0.6 * t), -0.484 * std::sin(1.1 * t + 0.3));
  }
  Eigen::Vector3d theta(double t) const {
    return Eigen::Vector3d(0.25 * std::sin(0.9 * t), 0.2 * std::cos(0.7 * t), 0.6 * t + 0.1 * std::sin(0.5 * t));
  }
  Eigen::Matrix3d R(double t) const { return so3Exp(theta(t)); }
  // body rate ω = vee(Rᵀ Ṙ) by a tight central difference (error O(h²) ≈ 1e-10)
  Eigen::Vector3d omega(double t) const {
    const double h = 1e-5;
    return so3Log(Eigen::Matrix3d(R(t - h).transpose() * R(t + h))) / (2.0 * h);
  }
};

const Eigen::Vector3d kG(0.05, -0.12, -9.80);   // W is not level (DESIGN §1)

BridgeSample truthSample(const Truth& tr, double t) {
  BridgeSample s;
  s.t = t;
  s.R = tr.R(t);
  s.p = tr.p(t);
  s.v = tr.v(t);
  s.a_world = tr.a(t);
  s.omega = tr.omega(t);
  s.f_body = s.R.transpose() * (s.a_world - kG);
  return s;
}

// Sample times of one LIO cycle: start, IMU ticks strictly inside, end (as UndistortPcl records them).
std::vector<double> cycleTimes(double t_prev, double t_k, double imu_dt, double imu_phase) {
  std::vector<double> ts{t_prev};
  const double first = std::ceil((t_prev - imu_phase) / imu_dt) * imu_dt + imu_phase;
  for (double t = first; t < t_k - 1e-12; t += imu_dt)
    if (t > t_prev + 1e-12) ts.push_back(t);
  ts.push_back(t_k);
  return ts;
}

double rotAngle(const Eigen::Matrix3d& A, const Eigen::Matrix3d& B) { return so3Log(Eigen::Matrix3d(A.transpose() * B)).norm(); }

// ---- literal copy of the host propagation --------------------------------------------------------------------
// utils/so3_math.h:24-40 Exp(ang_vel, dt)
template <typename T, typename Ts>
Eigen::Matrix<T, 3, 3> hostExp(const Eigen::Matrix<T, 3, 1>& ang_vel, const Ts& dt) {
  T ang_vel_norm = ang_vel.norm();
  Eigen::Matrix<T, 3, 3> Eye3 = Eigen::Matrix<T, 3, 3>::Identity();
  if (ang_vel_norm > 0.0000001) {
    Eigen::Matrix<T, 3, 1> r_axis = ang_vel / ang_vel_norm;
    Eigen::Matrix<T, 3, 3> K;
    K << 0.0, -r_axis[2], r_axis[1], r_axis[2], 0.0, -r_axis[0], -r_axis[1], r_axis[0], 0.0;
    T r_ang = ang_vel_norm * dt;
    return Eye3 + std::sin(r_ang) * K + (1.0 - std::cos(r_ang)) * K * K;
  } else {
    return Eye3;
  }
}

// IMU_Processing.cpp:412-421 for one interval, in place.
void hostStep(const Eigen::Vector3d& angvel_avr, const Eigen::Vector3d& acc_avr, const Eigen::Vector3d& gravity, double dt,
              Eigen::Matrix3d& R_imu, Eigen::Vector3d& pos_imu, Eigen::Vector3d& vel_imu, Eigen::Vector3d& acc_imu) {
  Eigen::Matrix3d Exp_f = hostExp(angvel_avr, dt);
  /* propogation of IMU attitude */
  R_imu = R_imu * Exp_f;
  /* Specific acceleration (global frame) of IMU */
  acc_imu = R_imu * acc_avr + gravity;
  /* propogation of IMU */
  pos_imu = pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt;
  /* velocity of IMU */
  vel_imu = vel_imu + acc_imu * dt;
}

// Integrate a piecewise-constant (ω, f) sequence from a start state with the host formula; returns the samples the
// host would record (start sample first). `t[i]` are the sample times, (w[i], f[i]) the rates of (t[i-1], t[i]].
std::vector<BridgeSample> integrateHost(const std::vector<double>& t, const std::vector<Eigen::Vector3d>& w,
                                        const std::vector<Eigen::Vector3d>& f, const Eigen::Matrix3d& R0, const Eigen::Vector3d& p0,
                                        const Eigen::Vector3d& v0, const Eigen::Vector3d& w0, const Eigen::Vector3d& a0) {
  std::vector<BridgeSample> out;
  BridgeSample s0;
  s0.t = t[0];
  s0.R = R0;
  s0.p = p0;
  s0.v = v0;
  s0.omega = w0;
  s0.a_world = a0;
  s0.f_body = R0.transpose() * (a0 - kG);
  out.push_back(s0);
  Eigen::Matrix3d R = R0;
  Eigen::Vector3d p = p0, v = v0, acc;
  for (size_t i = 1; i < t.size(); ++i) {
    hostStep(w[i], f[i], kG, t[i] - t[i - 1], R, p, v, acc);
    BridgeSample s;
    s.t = t[i];
    s.R = R;
    s.p = p;
    s.v = v;
    s.omega = w[i];
    s.a_world = acc;
    s.f_body = f[i];
    out.push_back(s);
  }
  return out;
}

// Same integration but with the W-frame acceleration of every interval prescribed (`a[i]` for (t[i-1], t[i]]), so a
// jump of the start attitude does not change the recorded a_world: this isolates the start-state jump that the
// chaining must cancel from the (2 mm)²-per-cycle effect of re-recording a_world with a corrected attitude.
std::vector<BridgeSample> integrateWithAccel(const std::vector<double>& t, const std::vector<Eigen::Vector3d>& w,
                                             const std::vector<Eigen::Vector3d>& a, const Eigen::Matrix3d& R0, const Eigen::Vector3d& p0,
                                             const Eigen::Vector3d& v0, const Eigen::Vector3d& w0) {
  std::vector<BridgeSample> out;
  BridgeSample s0;
  s0.t = t[0];
  s0.R = R0;
  s0.p = p0;
  s0.v = v0;
  s0.omega = w0;
  s0.a_world = a[0];
  s0.f_body = R0.transpose() * (a[0] - kG);
  out.push_back(s0);
  Eigen::Matrix3d R = R0;
  Eigen::Vector3d p = p0, v = v0;
  for (size_t i = 1; i < t.size(); ++i) {
    const double dt = t[i] - t[i - 1];
    R = R * hostExp(w[i], dt);
    p = p + v * dt + 0.5 * a[i] * dt * dt;
    v = v + a[i] * dt;
    BridgeSample s;
    s.t = t[i];
    s.R = R;
    s.p = p;
    s.v = v;
    s.omega = w[i];
    s.a_world = a[i];
    s.f_body = R.transpose() * (a[i] - kG);
    out.push_back(s);
  }
  return out;
}

}  // namespace

// noiseless high-rate truth, δ ≤ 0.1 s → position error < 0.1 mm
TEST(U14Bridge, NoiselessHighRateTruthUnder0p1mm) {
  Config cfg;
  cfg.time.max_bridge_s = 0.3;
  cfg.time.bridge_ring_cycles = 4;   // A33: all 8 cycles (0.05 s each) end within max_bridge_s + 0.1 s of t_k, so the ring
                                     // holds the whole 0.4 s and max_bridge_s is the only limit
  ImuBridge bridge(cfg);
  Truth tr;
  const Eigen::Vector3d bg_snap(1e-3, -2e-3, 5e-4), ba_snap(0.01, -0.02, 0.03);
  // 200 Hz IMU, 20 Hz LIO cycles (0.05 s) so that δ ≤ 0.1 s also crosses cycle boundaries
  const double imu_dt = 0.005, phase = 0.0013;
  const double cyc = 0.05;
  double t_prev = 0.0, last_cycle_start = 0.0;
  std::vector<double> cycle_starts;
  for (int c = 0; c < 8; ++c) {
    const double t_k = t_prev + cyc;
    std::vector<BridgeSample> s;
    for (double t : cycleTimes(t_prev, t_k, imu_dt, phase)) s.push_back(truthSample(tr, t));
    bridge.pushCycle(s, t_prev, t_k, bg_snap, ba_snap, kG);
    last_cycle_start = t_prev;
    cycle_starts.push_back(t_prev);
    t_prev = t_k;
  }
  const double t_k = bridge.latestCycleEnd();
  ASSERT_NEAR(t_k, 0.4, 1e-12);
  const Eigen::Vector3d lever(-0.31635, 0.10926, 0.4006);

  std::mt19937 rng(14);
  std::uniform_real_distribution<double> ud(0.0, 0.1);
  std::vector<double> deltas{0.0, 1e-4, 0.005, 0.0137, 0.05, 0.0500001, 0.0731, 0.1};
  for (int i = 0; i < 40; ++i) deltas.push_back(ud(rng));
  double worst_p = 0.0;
  for (double d : deltas) {
    const double t_g = t_k - d;
    BridgeResult br;
    ASSERT_TRUE(bridge.query(t_g, &br)) << "delta " << d;
    EXPECT_NEAR(br.delta, d, 1e-12);
    EXPECT_EQ(br.cross_cycle, t_g < last_cycle_start);
    // truth at the cycle end is the last sample; reconstruct the epoch state from the relative quantities
    const Eigen::Vector3d p_k = tr.p(t_k), v_k = tr.v(t_k);
    const Eigen::Matrix3d R_k = tr.R(t_k);
    const Eigen::Vector3d p_g = p_k - v_k * d - br.D;
    const Eigen::Vector3d v_g = v_k - br.E;
    const Eigen::Matrix3d R_g = R_k * br.dR.transpose();
    const double ep = (p_g - tr.p(t_g)).norm();
    worst_p = std::max(worst_p, ep);
    EXPECT_LT(ep, 1e-4) << "delta " << d;
    EXPECT_LT((v_g - tr.v(t_g)).norm(), 1e-3) << "delta " << d;
    EXPECT_LT(rotAngle(R_g, tr.R(t_g)), 1e-4) << "delta " << d;
    EXPECT_LT((br.omega_g - tr.omega(t_g)).norm(), 5e-3) << "delta " << d;
    EXPECT_LT((br.p_g_hat - p_g).norm(), 1e-12);
    EXPECT_LT((br.v_g_hat - v_g).norm(), 1e-12);
    EXPECT_LT((br.R_g_hat - R_g).norm(), 1e-12);
    EXPECT_LT((br.R_k - R_k).norm(), 1e-15);
    // ā_b is the time-mean of f_body over (t_g, t_k]: bounded by the truth's specific-force range
    EXPECT_LT((br.a_bar - tr.R(t_k - 0.5 * d).transpose() * (tr.a(t_k - 0.5 * d) - kG)).norm(), 0.05 + 2.0 * d);
    // Σ_D: white-noise term only (A10), plus (2 mm)² per crossed cycle boundary (cycle starts after t_g)
    int crossed = 0;
    for (double cs : cycle_starts)
      if (cs > t_g) ++crossed;
    EXPECT_EQ(br.cross_cycle, crossed > 0);
    const double expect_sd = cfg.imu_bridge.acc_n * cfg.imu_bridge.acc_n * d * d * d / 3.0 + crossed * 4e-6;
    EXPECT_NEAR(br.Sigma_D(0, 0), expect_sd, 1e-15 + 1e-12 * expect_sd);
    EXPECT_NEAR(br.Sigma_D(1, 1), expect_sd, 1e-15 + 1e-12 * expect_sd);
    EXPECT_NEAR(br.Sigma_D(0, 1), 0.0, 0.0);
    EXPECT_NEAR(br.var_v_bridge, cfg.imu_bridge.acc_n * cfg.imu_bridge.acc_n * d, 1e-18);

    // evaluate() at the snapshot (x = propagated state at t_k with the snapshot biases/gravity) reproduces the antenna
    CoreState x;
    x.R = R_k;
    x.p = p_k;
    x.v = v_k;
    x.bg = bg_snap;
    x.ba = ba_snap;
    x.g = kG;
    const AntennaAtEpoch ant = ImuBridge::evaluate(x, br, lever, true);
    EXPECT_LT((ant.y - (tr.p(t_g) + tr.R(t_g) * lever)).norm(), 2e-4) << "delta " << d;
    EXPECT_LT((ant.z - (tr.v(t_g) + tr.R(t_g) * tr.omega(t_g).cross(lever))).norm(), 5e-3) << "delta " << d;
    EXPECT_LT((ant.omega_g - br.omega_g).norm(), 1e-15);
    EXPECT_LT((ant.p_g - p_g).norm(), 1e-12);

    // A10 first-order model: evaluate() is exactly linear in b_a and g with the §4.2 coefficients
    const Eigen::Vector3d dba(0.02, -0.01, 0.03), dg(0.01, 0.02, -0.03);
    CoreState xb = x;
    xb.ba += dba;
    const AntennaAtEpoch ab = ImuBridge::evaluate(xb, br, lever, true);
    // δa = −R̂_k δb_a ⇒ D(x) = D̂ + ½δ²R̂_k δb_a ⇒ Δy = −½δ² R̂_k δb_a (J_a column −½δ²MR̂);
    //                    E(x) = Ê − δR̂_k δb_a  ⇒ Δz = +δ R̂_k δb_a   (J_w column +δMR̂)
    EXPECT_LT((ab.y - ant.y + 0.5 * d * d * (R_k * dba)).norm(), 1e-12);
    EXPECT_LT((ab.z - ant.z - d * (R_k * dba)).norm(), 1e-12);
    CoreState xg = x;
    xg.g += dg;
    const AntennaAtEpoch ag = ImuBridge::evaluate(xg, br, lever, true);
    EXPECT_LT((ag.y - ant.y - 0.5 * d * d * dg).norm(), 1e-12);            // δg column +½δ²M
    EXPECT_LT((ag.z - ant.z + d * dg).norm(), 1e-12);                      // J_w δg column −δM
    const AntennaAtEpoch ag0 = ImuBridge::evaluate(xg, br, lever, false);  // gravity_est_en = false freezes the δg term
    EXPECT_LT((ag0.y - ant.y).norm(), 1e-15);
    EXPECT_LT((ag0.z - ant.z).norm(), 1e-15);
    // b_g enters only through ω_g
    CoreState xw = x;
    xw.bg += Eigen::Vector3d(0.01, 0.0, -0.02);
    const AntennaAtEpoch aw = ImuBridge::evaluate(xw, br, lever, true);
    EXPECT_LT((aw.omega_g - (ant.omega_g - Eigen::Vector3d(0.01, 0.0, -0.02))).norm(), 1e-15);
    EXPECT_LT((aw.y - ant.y).norm(), 1e-15);
  }
  // rejections: next cycle, too old for max_bridge_s (the ring itself holds everything from t = 0)
  BridgeResult br;
  EXPECT_EQ(bridge.cyclesStored(), 8);
  EXPECT_FALSE(bridge.query(t_k + 1e-6, &br));
  EXPECT_FALSE(bridge.query(t_k - 0.3000001, &br));
  EXPECT_TRUE(bridge.query(t_k - 0.2999999, &br));
  EXPECT_TRUE(br.cross_cycle);
  std::printf("[U14] worst bridged position error over %zu epochs: %.3e m\n", deltas.size(), worst_p);
}

// epochs before the cycle start chain relative motion across cycles
TEST(U14Bridge, CrossCycleChaining) {
  // Piecewise-constant (ω, f) inputs at 5 ms over [0, 0.5]; the continuous integration is the reference. Each LIO
  // cycle re-integrates the same inputs from a start state that jumps (the host's LIO/VIO corrections), so only the
  // relative quantities ΔR = ∏Exp(ω dt), E = ∫a, D = ∫∫a − δ∫a are shared with the continuous trajectory.
  std::mt19937 rng(7);
  std::normal_distribution<double> nd(0.0, 1.0);
  const double dt = 0.005, cyc = 0.1, T = 0.5;
  const int n_int = static_cast<int>(std::round(T / dt));
  std::vector<double> t(n_int + 1);
  std::vector<Eigen::Vector3d> w(n_int + 1), f(n_int + 1);
  for (int i = 0; i <= n_int; ++i) {
    t[i] = i * dt;
    w[i] = Eigen::Vector3d(0.3 * nd(rng), 0.3 * nd(rng), 0.8 + 0.2 * nd(rng));
    f[i] = Eigen::Vector3d(0.5 * nd(rng), 0.5 * nd(rng), 9.8 + 0.5 * nd(rng));
  }
  const Eigen::Matrix3d R0 = so3Exp(Eigen::Vector3d(0.1, -0.2, 0.7));
  const Eigen::Vector3d p0(3.0, -1.0, 0.5), v0(1.2, 0.3, -0.1);
  const std::vector<BridgeSample> cont = integrateHost(t, w, f, R0, p0, v0, w[0], R0 * f[0] + kG);
  auto contAt = [&](double tg, Eigen::Matrix3d* R, Eigen::Vector3d* p, Eigen::Vector3d* v) {
    size_t i = 0;
    while (i + 2 < cont.size() && cont[i + 1].t < tg) ++i;
    ImuBridge::interpolate(cont[i], cont[i + 1], tg, R, p, v);
  };

  // the W-frame accelerations the continuous trajectory recorded; every cycle re-integrates exactly these
  std::vector<Eigen::Vector3d> a_cont(cont.size());
  for (size_t i = 0; i < cont.size(); ++i) a_cont[i] = cont[i].a_world;
  auto buildRing = [&](int ring_cycles, ImuBridge* bridge, std::vector<Eigen::Vector3d>* bg_of_cycle) {
    const int per = static_cast<int>(std::round(cyc / dt));
    const int n_cyc = static_cast<int>(std::round(T / cyc));
    for (int c = 0; c < n_cyc; ++c) {
      const int i0 = c * per;
      std::vector<double> tc(t.begin() + i0, t.begin() + i0 + per + 1);
      std::vector<Eigen::Vector3d> wc(w.begin() + i0, w.begin() + i0 + per + 1), ac(a_cont.begin() + i0, a_cont.begin() + i0 + per + 1);
      Eigen::Matrix3d Rs = cont[i0].R;
      Eigen::Vector3d ps = cont[i0].p, vs = cont[i0].v;
      if (c > 0) {   // host update jump at the cycle boundary (attitude, position and velocity)
        Rs = Rs * so3Exp(Eigen::Vector3d(0.01 * nd(rng), 0.01 * nd(rng), 0.01 * nd(rng)));
        ps += Eigen::Vector3d(0.05 * nd(rng), 0.05 * nd(rng), 0.05 * nd(rng));
        vs += Eigen::Vector3d(0.02 * nd(rng), 0.02 * nd(rng), 0.02 * nd(rng));
      }
      const std::vector<BridgeSample> s = integrateWithAccel(tc, wc, ac, Rs, ps, vs, cont[i0].omega);
      const Eigen::Vector3d bg(1e-3 * c, 0.0, 0.0), ba(0.0, 1e-2 * c, 0.0), g = kG + Eigen::Vector3d(0.0, 0.0, 1e-3 * c);
      bg_of_cycle->push_back(bg);
      bridge->pushCycle(s, tc.front(), tc.back(), bg, ba, g);
    }
    (void)ring_cycles;
  };

  Config cfg;
  cfg.time.max_bridge_s = 0.3;
  cfg.time.bridge_ring_cycles = 4;
  ImuBridge bridge(cfg);
  std::vector<Eigen::Vector3d> bgs;
  buildRing(4, &bridge, &bgs);
  // A33: at least 4 cycles and every cycle ending at or after 0.5 − (0.3 + 0.1) = 0.1 s — all five here (the first ends
  // on that horizon and is kept)
  EXPECT_EQ(bridge.cyclesStored(), 5);
  EXPECT_EQ(bridge.oldestTime(), 0.0);
  EXPECT_NEAR(bridge.latestCycleEnd(), 0.5, 1e-12);

  struct Case {
    double t_g;
    int cycle;
    bool cross;
  };
  const std::vector<Case> cases{{0.2, 2, true}, {0.2125, 2, true}, {0.25, 2, true}, {0.2999, 2, true}, {0.3, 3, true},
                                {0.3777, 3, true}, {0.4, 4, false}, {0.41, 4, false}, {0.45, 4, false}, {0.5, 4, false}};
  // relative motion of the continuous trajectory from t_g to t_k = 0.5 (what every ring must reproduce)
  auto expectContinuous = [&](const BridgeResult& br, double t_g) {
    Eigen::Matrix3d Rg, Rk;
    Eigen::Vector3d pg, vg, pk, vk;
    contAt(t_g, &Rg, &pg, &vg);
    contAt(0.5, &Rk, &pk, &vk);
    const Eigen::Matrix3d dR_ref = Rg.transpose() * Rk;
    const Eigen::Vector3d E_ref = vk - vg;
    const Eigen::Vector3d D_ref = pk - pg - vk * (0.5 - t_g);
    EXPECT_LT((br.dR - dR_ref).norm(), 1e-9) << "t_g " << t_g;
    EXPECT_LT((br.E - E_ref).norm(), 1e-9) << "t_g " << t_g;
    EXPECT_LT((br.D - D_ref).norm(), 1e-9) << "t_g " << t_g;
  };
  for (const Case& c : cases) {
    BridgeResult br;
    ASSERT_TRUE(bridge.query(c.t_g, &br)) << "t_g " << c.t_g;
    EXPECT_EQ(br.cross_cycle, c.cross) << "t_g " << c.t_g;
    EXPECT_NEAR(br.delta, 0.5 - c.t_g, 1e-12);
    expectContinuous(br, c.t_g);
    // snapshots: b̂_g of the cycle that holds t_g (ω̂_g was corrected with it), b̂_a / ĝ of the cycle ending at t_k
    EXPECT_LT((br.bg_snap - bgs[c.cycle]).norm(), 1e-15);
    EXPECT_NEAR(br.ba_snap.y(), 1e-2 * 4, 1e-15);
    EXPECT_NEAR(br.g_snap.z(), kG.z() + 4e-3, 1e-15);
    // ω̂_g: the rate of the interval (t_i, t_{i+1}] with t_{i+1} > t_g (the one *starting* at a sample instant), the
    // last interval's rate at t_g = t_k
    size_t i = 0;
    while (i + 2 < cont.size() && !(cont[i + 1].t > c.t_g)) ++i;
    const Eigen::Vector3d w_expect = (c.t_g < cont.back().t) ? cont[i + 1].omega : cont.back().omega;
    EXPECT_LT((br.omega_g - w_expect).norm(), 1e-15) << "t_g " << c.t_g;
    // Σ_D carries (2 mm)² per crossed cycle
    const int crossed = 4 - c.cycle;
    EXPECT_NEAR(br.Sigma_D(2, 2) - cfg.imu_bridge.acc_n * cfg.imu_bridge.acc_n * std::pow(0.5 - c.t_g, 3) / 3.0, crossed * 4e-6, 1e-15);
  }
  // rejected: too old for max_bridge_s (δ = 0.31) although its cycle is still held, next cycle
  BridgeResult br;
  EXPECT_LT(bridge.oldestTime(), 0.19);
  EXPECT_FALSE(bridge.query(0.19, &br));
  EXPECT_FALSE(bridge.query(0.5 + 1e-9, &br));
  // A short ring (A33, replaces the old "a shorter ring drops old cycles inside max_bridge_s"): bridge_ring_cycles is a
  // minimum only, so a 2-cycle setting keeps the same five cycles and bridges every case exactly like the 4-cycle ring —
  // including t_g = 0.25 s (δ = 0.25 s, three cycles back), which the count-only rule refused
  Config cfg2 = cfg;
  cfg2.time.bridge_ring_cycles = 2;
  ImuBridge bridge2(cfg2);
  std::vector<Eigen::Vector3d> bgs2;
  rng.seed(7);   // other start-state jumps than the first ring; the chaining must cancel them all the same
  buildRing(2, &bridge2, &bgs2);
  EXPECT_EQ(bridge2.cyclesStored(), 5);
  EXPECT_EQ(bridge2.oldestTime(), 0.0);
  for (const Case& c : cases) {
    BridgeResult b2;
    ASSERT_TRUE(bridge2.query(c.t_g, &b2)) << "short ring, t_g " << c.t_g;
    EXPECT_EQ(b2.cross_cycle, c.cross) << "short ring, t_g " << c.t_g;
    expectContinuous(b2, c.t_g);
  }
  EXPECT_TRUE(bridge2.query(0.25, &br));
  EXPECT_TRUE(br.cross_cycle);
  EXPECT_FALSE(bridge2.query(0.19, &br));   // max_bridge_s still applies
  // precondition violations throw instead of silently corrupting the ring
  std::vector<BridgeSample> bad(1);
  bad[0].t = 0.6;
  EXPECT_THROW(bridge2.pushCycle(bad, 0.5, 0.6, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), kG), std::invalid_argument);
  EXPECT_THROW(bridge2.pushCycle({}, 0.5, 0.6, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), kG), std::invalid_argument);
  // a gap or an overlap between consecutive cycles is refused too (the host cannot produce one: prop_beg_time =
  // last_prop_end_time, IMU_Processing.cpp:248/:470); the ring is left untouched, so an epoch in the gap is never
  // bridged with a negative segment length
  {
    Truth tr;
    auto cycle = [&](double t0, double t1) {
      std::vector<BridgeSample> s;
      for (double t : cycleTimes(t0, t1, 0.005, 0.0013)) s.push_back(truthSample(tr, t));
      return s;
    };
    BridgeResult ref;
    ASSERT_TRUE(bridge2.query(0.35, &ref));
    EXPECT_THROW(bridge2.pushCycle(cycle(0.55, 0.65), 0.55, 0.65, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), kG), std::invalid_argument);   // gap
    EXPECT_THROW(bridge2.pushCycle(cycle(0.45, 0.55), 0.45, 0.55, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), kG), std::invalid_argument);   // overlap
    EXPECT_EQ(bridge2.cyclesStored(), 5);
    EXPECT_EQ(bridge2.oldestTime(), 0.0);
    EXPECT_NEAR(bridge2.latestCycleEnd(), 0.5, 1e-12);
    BridgeResult again;
    ASSERT_TRUE(bridge2.query(0.35, &again));
    EXPECT_TRUE(again.D == ref.D && again.E == ref.E && again.dR == ref.dR);
    EXPECT_FALSE(bridge2.query(0.6, &again));   // still "next cycle"
    // the contiguous cycle is accepted (1e-9 slack on t_prev, as for samples[0]); the cycle ending at 0.1 s is now
    // outside the horizon 0.6 − 0.4 = 0.2 s and more than two are held, so it goes (the one ending at 0.2 s stays)
    bridge2.pushCycle(cycle(0.5 + 4e-10, 0.6), 0.5 + 4e-10, 0.6, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), kG);
    EXPECT_EQ(bridge2.cyclesStored(), 5);
    EXPECT_NEAR(bridge2.oldestTime(), 0.1, 1e-12);
    EXPECT_NEAR(bridge2.latestCycleEnd(), 0.6, 1e-12);
    ASSERT_TRUE(bridge2.query(0.55, &again));
    EXPECT_FALSE(again.cross_cycle);
  }
}

// A33: after every push the ring holds exactly the last max(min(pushed, bridge_ring_cycles), #{t_k ≥ t_latest −
// (max_bridge_s + 0.1 s)}) cycles — at least the configured count, everything an epoch within max_bridge_s can need,
// and nothing older. Irregular cycle lengths (LIO cycles follow image/LiDAR arrival, skipped frames give long ones) and
// settings where either term binds; then the edge of the rule at ±1 µs.
TEST(U14Bridge, RingRetentionFollowsA33Rule) {
  auto stillCycle = [](double t0, double t1) {   // start and end sample only; motion is irrelevant to retention
    std::vector<BridgeSample> s(2);
    s[0].t = t0;
    s[1].t = t1;
    return s;
  };
  const Eigen::Vector3d z = Eigen::Vector3d::Zero();
  const std::vector<double> lens{0.0671, 0.0329, 0.1013, 0.0659, 0.1497, 0.0211, 0.0663, 0.0672, 0.1981, 0.0337,
                                 0.0668, 0.0665, 0.2512, 0.0669, 0.0664, 0.1203, 0.0409, 0.0667, 0.0333, 0.0337};
  struct Setting {
    double max_bridge_s;
    int ring;
  };
  // (0.3, 4) = the M3DGR defaults; (0.3, 1): the horizon alone; (0.3, 12): the count dominates; (0.05, 4): the count
  // binds at a short horizon; (1.0, 2): a long horizon
  for (const Setting st : {Setting{0.3, 4}, Setting{0.3, 1}, Setting{0.3, 12}, Setting{0.05, 4}, Setting{1.0, 2}}) {
    Config cfg;
    cfg.time.max_bridge_s = st.max_bridge_s;
    cfg.time.bridge_ring_cycles = st.ring;
    ImuBridge bridge(cfg);
    std::vector<double> t_prev_all, t_k_all;
    double t = 5.0;
    int count_bound = 0, horizon_bound = 0;   // pushes after which each term decided the size
    for (size_t m = 0; m < lens.size(); ++m) {
      const double t0 = t, t1 = t + lens[m];
      bridge.pushCycle(stillCycle(t0, t1), t0, t1, z, z, kG);
      t_prev_all.push_back(t0);
      t_k_all.push_back(t1);
      t = t1;
      const double horizon = t1 - (st.max_bridge_s + 0.1);
      int within = 0;
      for (double tk : t_k_all) {
        // no cycle end within 1 µs of the horizon in this sequence, so the expectation does not depend on rounding
        ASSERT_GT(std::abs(tk - horizon), 1e-6) << "sequence puts a cycle end on the horizon (setting " << st.max_bridge_s << "/" << st.ring << ")";
        if (tk >= horizon) ++within;
      }
      const int by_count = std::min(static_cast<int>(m) + 1, st.ring);
      const int expect = std::max(by_count, within);
      if (by_count > within) ++count_bound;
      if (within > by_count) ++horizon_bound;
      ASSERT_EQ(bridge.cyclesStored(), expect) << "setting " << st.max_bridge_s << "/" << st.ring << " after push " << m;
      EXPECT_EQ(bridge.oldestTime(), t_prev_all[m + 1 - expect]);
      EXPECT_EQ(bridge.latestCycleEnd(), t1);
      // the purpose: an epoch just inside max_bridge_s (or at the first cycle start, early on) is bridgeable, one just
      // outside it is LATE
      const double reach = std::min(st.max_bridge_s, t1 - t_prev_all.front()) * (1.0 - 1e-9);
      BridgeResult br;
      EXPECT_TRUE(bridge.query(t1 - reach, &br)) << "setting " << st.max_bridge_s << "/" << st.ring << " after push " << m;
      EXPECT_FALSE(bridge.query(t1 - st.max_bridge_s * (1.0 + 1e-9), &br));
    }
    // each setting exercises the term it is there for
    if (st.ring == 1 || st.max_bridge_s >= 1.0) { EXPECT_GT(horizon_bound, 10) << st.max_bridge_s << "/" << st.ring; }
    if (st.ring == 12 || st.max_bridge_s < 0.1) { EXPECT_GT(count_bound, 10) << st.max_bridge_s << "/" << st.ring; }
    if (st.ring == 4 && st.max_bridge_s == 0.3) { EXPECT_GT(horizon_bound, 5); }   // the defaults need the horizon term
  }
  // The edge of the rule (defaults 0.3 s / 4): cycles end at 1.0, 1.1, 1.2, 1.3; the fifth ends at 1.4 − 1 µs, so the
  // horizon is 1.0 − 1 µs and the cycle ending at 1.0 stays; the sixth ends at 1.4 + 1 µs, the horizon moves to
  // 1.0 + 1 µs and that cycle goes (six held > 4)
  {
    Config cfg;
    ImuBridge b(cfg);
    const std::vector<double> ends{1.0, 1.1, 1.2, 1.3, 1.4 - 1e-6};
    double t0 = 0.9;
    for (double t1 : ends) {
      b.pushCycle(stillCycle(t0, t1), t0, t1, z, z, kG);
      t0 = t1;
    }
    EXPECT_EQ(b.cyclesStored(), 5);
    EXPECT_EQ(b.oldestTime(), 0.9);
    b.pushCycle(stillCycle(t0, 1.4 + 1e-6), t0, 1.4 + 1e-6, z, z, kG);
    EXPECT_EQ(b.cyclesStored(), 5);
    EXPECT_EQ(b.oldestTime(), 1.0);
  }
  // before the ring reaches back far enough (start-up) "older than the ring" is the only refusal inside max_bridge_s
  {
    Config cfg;
    ImuBridge b(cfg);
    BridgeResult br;
    EXPECT_FALSE(b.query(1.0, &br));   // empty
    b.pushCycle(stillCycle(1.0, 1.1), 1.0, 1.1, z, z, kG);
    EXPECT_TRUE(b.query(1.0, &br));
    EXPECT_FALSE(b.query(1.0 - 1e-6, &br));
  }
}

// A33 on the M3DGR configuration (defaults): LIO runs at the 15 Hz camera rate, so δ = 0.28 s < max_bridge_s = 0.3 s
// reaches into the fifth cycle back, beyond the 4/15 s = 0.267 s the default four cycles span. It must be bridged —
// exactly (continuous-integration reference across start-state jumps, as in CrossCycleChaining) and accurately
// (< 0.1 mm against the analytic truth, at t = 0 and at M3DGR time stamps ≈ 1.733e9 s).
TEST(U14Bridge, FifteenHzCyclesBridge280ms) {
  Config cfg;
  ASSERT_EQ(cfg.time.max_bridge_s, 0.3);
  ASSERT_EQ(cfg.time.bridge_ring_cycles, 4);
  const int n_cyc = 16;
  std::vector<double> tb(n_cyc + 1);   // cycle c = (tb[c], tb[c+1]]
  for (int c = 0; c <= n_cyc; ++c) tb[c] = c / 15.0;
  const double t_end = tb[n_cyc];
  const double imu_dt = 0.005;

  // ---- (a) exact chaining ------------------------------------------------------------------------------------------
  // (ω, a) piecewise constant on the 200 Hz grid: input i holds on ((i−1)·dt, i·dt]. Every cycle records the host's
  // samples (start, IMU ticks inside, end) and re-integrates from a start state that jumps; the reference integrates
  // the same inputs over the union of all sample times without jumps.
  std::mt19937 rng(15);
  std::normal_distribution<double> nd(0.0, 1.0);
  const int n_grid = static_cast<int>(std::ceil(t_end / imu_dt)) + 1;
  std::vector<Eigen::Vector3d> w_grid(n_grid + 1), a_grid(n_grid + 1);
  for (int i = 0; i <= n_grid; ++i) {
    w_grid[i] = Eigen::Vector3d(0.3 * nd(rng), 0.3 * nd(rng), 0.8 + 0.2 * nd(rng));
    a_grid[i] = Eigen::Vector3d(0.5 * nd(rng), 0.5 * nd(rng), 0.3 * nd(rng));
  }
  auto inputOf = [&](double t_interval_end) { return static_cast<int>(std::ceil(t_interval_end / imu_dt - 1e-9)); };
  std::vector<std::vector<double>> cyc_t(n_cyc);
  std::vector<double> all_t{0.0};
  for (int c = 0; c < n_cyc; ++c) {
    cyc_t[c] = cycleTimes(tb[c], tb[c + 1], imu_dt, 0.0);
    all_t.insert(all_t.end(), cyc_t[c].begin() + 1, cyc_t[c].end());
  }
  auto inputs = [&](const std::vector<double>& ts, std::vector<Eigen::Vector3d>* w, std::vector<Eigen::Vector3d>* a) {
    w->resize(ts.size());
    a->resize(ts.size());
    for (size_t s = 0; s < ts.size(); ++s) {
      const int i = std::max(inputOf(ts[s]), 0);
      (*w)[s] = w_grid[i];
      (*a)[s] = a_grid[i];
    }
  };
  const Eigen::Matrix3d R0 = so3Exp(Eigen::Vector3d(0.1, -0.2, 0.7));
  const Eigen::Vector3d p0(3.0, -1.0, 0.5), v0(1.2, 0.3, -0.1);
  std::vector<Eigen::Vector3d> w_all, a_all;
  inputs(all_t, &w_all, &a_all);
  const std::vector<BridgeSample> cont = integrateWithAccel(all_t, w_all, a_all, R0, p0, v0, w_all[0]);
  auto contAt = [&](double tg, Eigen::Matrix3d* R, Eigen::Vector3d* p, Eigen::Vector3d* v) {
    size_t i = 0;
    while (i + 2 < cont.size() && cont[i + 1].t < tg) ++i;
    ImuBridge::interpolate(cont[i], cont[i + 1], tg, R, p, v);
  };
  auto contIndex = [&](double tq) {   // index of tq in the merged grid
    size_t i = 0;
    while (i + 1 < cont.size() && cont[i].t < tq) ++i;
    return i;
  };

  ImuBridge bridge(cfg);
  for (int c = 0; c < n_cyc; ++c) {
    std::vector<Eigen::Vector3d> wc, ac;
    inputs(cyc_t[c], &wc, &ac);
    const BridgeSample& s0 = cont[contIndex(tb[c])];
    ASSERT_EQ(s0.t, tb[c]);
    Eigen::Matrix3d Rs = s0.R;
    Eigen::Vector3d ps = s0.p, vs = s0.v;
    if (c > 0) {   // host update jump at the cycle boundary
      Rs = Rs * so3Exp(Eigen::Vector3d(0.01 * nd(rng), 0.01 * nd(rng), 0.01 * nd(rng)));
      ps += Eigen::Vector3d(0.05 * nd(rng), 0.05 * nd(rng), 0.05 * nd(rng));
      vs += Eigen::Vector3d(0.02 * nd(rng), 0.02 * nd(rng), 0.02 * nd(rng));
    }
    bridge.pushCycle(integrateWithAccel(cyc_t[c], wc, ac, Rs, ps, vs, s0.omega), tb[c], tb[c + 1], Eigen::Vector3d::Zero(),
                     Eigen::Vector3d::Zero(), kG);
  }
  // A33 at 15 Hz: 6 × 1/15 s = 0.4 s exactly, so the cycles ending at or after t_end − 0.4 s are c = 9..15 (the one
  // ending on the horizon is kept): seven, three more than bridge_ring_cycles
  EXPECT_EQ(bridge.cyclesStored(), 7);
  EXPECT_EQ(bridge.oldestTime(), tb[n_cyc - 7]);
  EXPECT_EQ(bridge.latestCycleEnd(), t_end);

  // (apart from δ = 0, the cycle end itself, none of these puts t_g on a cycle boundary or an IMU tick:
  // 200·t_g = 213.33… − 200 δ)
  for (double d : {0.0, 0.03, 0.07, 0.13, 0.21, 0.25, 0.27, 0.28, 0.29, 0.2999999}) {
    const double t_g = t_end - d;
    BridgeResult br;
    ASSERT_TRUE(bridge.query(t_g, &br)) << "delta " << d;
    EXPECT_NEAR(br.delta, d, 1e-12);
    int j = 0;   // cycle holding t_g: the last one starting at or before it
    while (j + 1 < n_cyc && tb[j + 1] <= t_g) ++j;
    const int crossed = (n_cyc - 1) - j;
    EXPECT_EQ(br.cross_cycle, crossed > 0) << "delta " << d;
    const double expect_sd = cfg.imu_bridge.acc_n * cfg.imu_bridge.acc_n * d * d * d / 3.0 + crossed * 4e-6;
    EXPECT_NEAR(br.Sigma_D(0, 0), expect_sd, 1e-15 + 1e-12 * expect_sd) << "delta " << d;
    Eigen::Matrix3d Rg, Rk;
    Eigen::Vector3d pg, vg, pk, vk;
    contAt(t_g, &Rg, &pg, &vg);
    contAt(t_end, &Rk, &pk, &vk);
    EXPECT_LT((br.dR - Rg.transpose() * Rk).norm(), 1e-9) << "delta " << d;
    EXPECT_LT((br.E - (vk - vg)).norm(), 1e-9) << "delta " << d;
    EXPECT_LT((br.D - (pk - pg - vk * d)).norm(), 1e-9) << "delta " << d;
    if (d == 0.28) {
      EXPECT_EQ(crossed, 4);
      // the discriminating fact: t_g lies before the start of the 4th-newest cycle, so a ring of bridge_ring_cycles = 4
      // (the rule before A33) could not bridge this epoch
      EXPECT_LT(t_g, tb[n_cyc - 4]);
    }
  }
  // just beyond max_bridge_s: refused as LATE although the ring still holds that time
  {
    const double t_g = t_end - 0.3 * (1.0 + 1e-6);
    EXPECT_LT(bridge.oldestTime(), t_g);
    BridgeResult br;
    EXPECT_FALSE(bridge.query(t_g, &br));
  }

  // ---- (b) accuracy against the analytic truth at δ = 0.28 s, at t = 0 and at M3DGR time stamps ---------------------
  Truth tr;
  for (double t0 : {0.0, 1733040083.0}) {
    ImuBridge b(cfg);
    for (int c = 0; c < n_cyc; ++c) {
      const double tp = t0 + tb[c], tk = t0 + tb[c + 1];
      std::vector<BridgeSample> s;
      for (double t : cycleTimes(tp, tk, imu_dt, t0 + 0.0013)) {
        BridgeSample x = truthSample(tr, t - t0);   // t − t0 is exact (Sterbenz) for the large origin
        x.t = t;
        s.push_back(x);
      }
      b.pushCycle(s, tp, tk, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), kG);
    }
    EXPECT_GE(b.cyclesStored(), 6) << "t0 " << t0;   // at 1.7e9 s the tie at exactly 0.4 s is below the time resolution
    const double t_k = t0 + t_end;
    const double t_g = t_k - 0.28;
    BridgeResult br;
    ASSERT_TRUE(b.query(t_g, &br)) << "t0 " << t0;
    EXPECT_TRUE(br.cross_cycle);
    const double sg = t_g - t0, sk = t_k - t0, d = br.delta;
    EXPECT_NEAR(d, 0.28, 1e-6) << "t0 " << t0;
    const Eigen::Vector3d p_g = tr.p(sk) - tr.v(sk) * d - br.D;
    const Eigen::Vector3d v_g = tr.v(sk) - br.E;
    const Eigen::Matrix3d R_g = tr.R(sk) * br.dR.transpose();
    EXPECT_LT((p_g - tr.p(sg)).norm(), 1e-4) << "t0 " << t0;
    EXPECT_LT((v_g - tr.v(sg)).norm(), 1e-3) << "t0 " << t0;
    EXPECT_LT(rotAngle(R_g, tr.R(sg)), 1e-4) << "t0 " << t0;
    std::printf("[U14] 15 Hz cycles, delta 0.28 s, t0 = %.1f: bridged position error %.3e m, %d cycles held\n", t0,
                (p_g - tr.p(sg)).norm(), b.cyclesStored());
  }
}

// interpolation equals IMU_Processing.cpp:412-421 term by term
TEST(U14Bridge, InterpolationMatchesHostPropagation) {
  std::mt19937 rng(3);
  std::normal_distribution<double> nd(0.0, 1.0);
  const double G_m_s2 = 9.81;
  const double mean_acc_norm = 9.79;                       // acc_avr = acc_avr * G_m_s2 / mean_acc.norm() − bias_a (:353)
  const Eigen::Vector3d bias_g(1e-3, -2e-3, 3e-3), bias_a(0.01, 0.02, -0.03), gravity = kG;
  Eigen::Matrix3d R_imu = so3Exp(Eigen::Vector3d(0.3, -0.1, 0.5));
  Eigen::Vector3d pos_imu(1.0, 2.0, 3.0), vel_imu(0.5, -0.2, 0.1), acc_imu = Eigen::Vector3d::Zero();
  const double prop_beg_time = 100.0;
  // host-style records: start sample, then one per IMU interval
  std::vector<BridgeSample> rec;
  {
    BridgeSample s;
    s.t = prop_beg_time;
    s.R = R_imu;
    s.p = pos_imu;
    s.v = vel_imu;
    rec.push_back(s);
  }
  double t_head = prop_beg_time - 0.002;
  for (int i = 0; i < 25; ++i) {
    const double t_tail = t_head + 0.005;
    const Eigen::Vector3d gyr_head(0.1 * nd(rng), 0.1 * nd(rng), 0.5 + 0.1 * nd(rng));
    const Eigen::Vector3d gyr_tail = gyr_head + Eigen::Vector3d(0.01 * nd(rng), 0.01 * nd(rng), 0.01 * nd(rng));
    const Eigen::Vector3d acc_head(0.3 * nd(rng), 0.3 * nd(rng), 9.8 + 0.3 * nd(rng));
    const Eigen::Vector3d acc_tail = acc_head + Eigen::Vector3d(0.05 * nd(rng), 0.05 * nd(rng), 0.05 * nd(rng));
    Eigen::Vector3d angvel_avr, acc_avr;
    angvel_avr << 0.5 * (gyr_head.x() + gyr_tail.x()), 0.5 * (gyr_head.y() + gyr_tail.y()), 0.5 * (gyr_head.z() + gyr_tail.z());
    acc_avr << 0.5 * (acc_head.x() + acc_tail.x()), 0.5 * (acc_head.y() + acc_tail.y()), 0.5 * (acc_head.z() + acc_tail.z());
    angvel_avr -= bias_g;
    acc_avr = acc_avr * G_m_s2 / mean_acc_norm - bias_a;
    const double dt = (i == 0) ? (t_tail - prop_beg_time) : (t_tail - t_head);   // :357-372 first-interval rule
    const double offs_t = t_tail - prop_beg_time;
    hostStep(angvel_avr, acc_avr, gravity, dt, R_imu, pos_imu, vel_imu, acc_imu);
    BridgeSample s;   // IMUpose.push_back(set_pose6d(offs_t, acc_imu, angvel_avr, vel_imu, pos_imu, R_imu)) (:430)
    s.t = prop_beg_time + offs_t;
    s.a_world = acc_imu;
    s.omega = angvel_avr;
    s.R = R_imu;
    s.p = pos_imu;
    s.v = vel_imu;
    s.f_body = acc_avr;
    rec.push_back(s);
    t_head = t_tail;
  }
  // (a) at the interval end the interpolation reproduces the recorded sample
  for (size_t i = 0; i + 1 < rec.size(); ++i) {
    Eigen::Matrix3d R;
    Eigen::Vector3d p, v;
    ImuBridge::interpolate(rec[i], rec[i + 1], rec[i + 1].t, &R, &p, &v);
    EXPECT_LT((R - rec[i + 1].R).cwiseAbs().maxCoeff(), 1e-14) << "interval " << i;
    EXPECT_LT((p - rec[i + 1].p).cwiseAbs().maxCoeff(), 1e-13) << "interval " << i;
    EXPECT_LT((v - rec[i + 1].v).cwiseAbs().maxCoeff(), 1e-13) << "interval " << i;
  }
  // (b) inside the interval it equals the host statements executed with dt = τ from the interval start
  std::uniform_real_distribution<double> ud(0.0, 1.0);
  for (size_t i = 0; i + 1 < rec.size(); ++i) {
    for (int k = 0; k < 5; ++k) {
      const double tau = ud(rng) * (rec[i + 1].t - rec[i].t);
      Eigen::Matrix3d R = rec[i].R, Rh;
      Eigen::Vector3d p = rec[i].p, v = rec[i].v, acc, ph, vh;
      // The host integrates the interval with one constant acceleration acc_imu = R_{i+1} f + g (computed with the
      // end attitude, :415) and one constant rate; the interpolation keeps exactly those two constants (DESIGN §4.1),
      // so the reference is the host's three statements with dt → τ. acc is recomputed from the end attitude and
      // acc_avr exactly as :415 does, not read back from the recorded a_world (which the bridge consumes).
      Eigen::Matrix3d Exp_f = hostExp(rec[i + 1].omega, tau);
      R = R * Exp_f;
      acc = rec[i + 1].R * rec[i + 1].f_body + gravity;
      p = p + v * tau + 0.5 * acc * tau * tau;
      v = v + acc * tau;
      ImuBridge::interpolate(rec[i], rec[i + 1], rec[i].t + tau, &Rh, &ph, &vh);
      // same terms in the same order; the residual 1e-15..1e-14 is FMA contraction (-ffp-contract=fast by default
      // in GCC 9 for C++17) decided differently in the two translation units, not a formula difference
      EXPECT_LT((R - Rh).cwiseAbs().maxCoeff(), 1e-14) << "interval " << i;
      EXPECT_LT((p - ph).cwiseAbs().maxCoeff(), 1e-13) << "interval " << i;
      EXPECT_LT((v - vh).cwiseAbs().maxCoeff(), 1e-13) << "interval " << i;
    }
  }
  // (c) small-rate branch: ‖ω‖ ≤ 1e-7 gives the identity rotation exactly, as Exp(ang_vel, dt) does
  BridgeSample a = rec[0], b = rec[1];
  b.omega = Eigen::Vector3d(5e-8, 0.0, -5e-8);
  Eigen::Matrix3d R;
  Eigen::Vector3d p, v;
  ImuBridge::interpolate(a, b, b.t, &R, &p, &v);
  EXPECT_TRUE(R == a.R);
  // (d) sigmaD closed form
  const Eigen::Matrix3d S = ImuBridge::sigmaD(0.067, 0.0115, 0.02, 1e-3, 9.81, 1);
  const double expect = 0.0115 * 0.0115 * std::pow(0.067, 3) / 3.0 + std::pow(0.5 * 0.067 * 0.067 * (0.02 + 9.81 * 1e-3), 2) + 4e-6;
  EXPECT_NEAR(S(0, 0), expect, 1e-18);
  EXPECT_NEAR(S(1, 2), 0.0, 0.0);
}
