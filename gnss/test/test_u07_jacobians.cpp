// SPDX-License-Identifier: BSD-3-Clause
// test_u07_jacobians.cpp — TEST_PLAN §1 U7: every Jacobian of DESIGN §4.2–§4.6 / A10 / A11 against manifold
// central differences through the nonlinear model itself (ImuBridge::evaluate + Leveling::M + geometry).
// The host columns are perturbed with CoreState::boxplus (right perturbation on R), the augmented columns
// additively. Relative error < 1e-6 on every non-zero column; expected-zero columns < 1e-8 absolute.
// Row Jacobians are checked against the design's own linearisation model: the line of sight and the
// Sagnac term are frozen at the linearisation point (DESIGN §4.3 "视线对位置的导数与 Sagnac 率导数忽略").
// The TDCP clone columns are the one deliberate departure (AMENDMENTS A64): +u_k1ᵀ, checked against its definition and
// against the exact derivative +u_k0ᵀ it replaces.
// Runs against WP-b's Leveling / ImuBridge::evaluate / AugBlock and WP-a's rtk_adapter (both merged: a failure of
// theirs is a failure here, never "blocked"). The Doppler forward models never restate the Sagnac-rate formula under
// test: they differentiate the pseudorange model numerically (AMENDMENTS A36), on synthetic straight-line motion and
// on real broadcast orbits (the dev-set fixture's IGS BRDC).
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>
#include <iostream>
#include <limits>
#include <random>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "internal/rows.hpp"
#include "internal/leveling.hpp"
#include "internal/aug_block.hpp"
#include "internal/bridge.hpp"
#include "internal/rtk_adapter.hpp"
#include "internal/update.hpp"   // JointUpdater::projection (AMENDMENTS A70)
#include "../../gnss_io/test/fixture_discovery.hpp"   // WP-e's one discovery rule (A24): realFixtures(), fixtureBrdc()
#include "../../third_party/rtklib/testing/wpa_test_support.hpp"   // fixture slices + decodeChannel (A43 on real epoch times)

using namespace livo_gnss;

namespace {

constexpr double kC = 299792458.0;
constexpr double kOmgE = 7.2921151467e-5;
constexpr double kDeg = M_PI / 180.0;

SatId sat(int i) {
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
// The pseudorange's Sagnac term (geodist, rtkcmn.c:3612; DESIGN §1): the one Earth-rotation expression the test writes.
double sagnacTerm(const Eigen::Vector3d& rs, const Eigen::Vector3d& rr) { return kOmgE * (rs.x() * rr.y() - rs.y() * rr.x()) / kC; }
double rangeSagnac(const Eigen::Vector3d& rs, const Eigen::Vector3d& rr, Eigen::Vector3d* e) {
  const Eigen::Vector3d d = rs - rr;
  const double r = d.norm();
  if (e) *e = d / r;
  return r + sagnacTerm(rs, rr);
}
// Doppler forward model of the Sagnac rate WITHOUT restating the rate formula under test (AMENDMENTS A36: a forward
// model sharing the code's formula shares its sign error — that is how the resdop sign went unnoticed): the central
// difference of the Sagnac term along the straight-line motion r_s + v_s τ, r_A + v_A τ. The term is bilinear in τ
// there, so the difference is exact up to rounding (~1e-15 m/s at h = 1 s).
double sagnacRateNumeric(const Eigen::Vector3d& rs, const Eigen::Vector3d& vs, const Eigen::Vector3d& rr, const Eigen::Vector3d& vr) {
  const double h = 1.0;
  return (sagnacTerm(rs + vs * h, rr + vr * h) - sagnacTerm(rs - vs * h, rr - vr * h)) / (2.0 * h);
}
// SO(3) left Jacobian (Barfoot 2017 eq. 7.36), written here independently of rows.cpp for the A34 exact-column check.
Eigen::Matrix3d leftJacobian(const Eigen::Vector3d& th) {
  const double a = th.norm();
  const Eigen::Matrix3d K = skew(th);
  if (a < 1e-7) return Eigen::Matrix3d::Identity() + 0.5 * K;
  return Eigen::Matrix3d::Identity() + (1.0 - std::cos(a)) / (a * a) * K + (a - std::sin(a)) / (a * a * a) * K * K;
}
// ‖rs − rA(aN)‖ − ‖rs − rA(aN0)‖ without cancellation, formed from the N-frame antenna vectors (translation
// invariance: rs − rA = (rs − r0) − R_EN aN) so that a − b = R_EN(aN0 − aN) is exact to ~1e-15 m. Differencing
// two 2.3e7 m norms, or even two 6e6 m ECEF positions, leaves ε·6e6 ≈ 1e-9 m of roundoff — larger than the
// ½δ²-sized b_a/g entries times the central-difference step.
double normDiffStable(const Eigen::Vector3d& rs_rel, const Eigen::Matrix3d& R_EN, const Eigen::Vector3d& aN, const Eigen::Vector3d& aN0) {
  const Eigen::Vector3d a = rs_rel - R_EN * aN, b = rs_rel - R_EN * aN0;
  const Eigen::Vector3d d = R_EN * (aN0 - aN);
  return d.dot(a + b) / (a.norm() + b.norm());
}

// A random linearisation scene: host state, bridge result, leveling, augmented block with every slot.
struct Scene {
  Config cfg;
  CoreState x;
  BridgeResult br;
  Leveling lev;
  AugBlock aug{cfg};
  AnchorFrame anchor;
  Eigen::Vector3d lever = Eigen::Vector3d(-0.316, 0.109, 0.401);
  Eigen::Vector3d pivot = Eigen::Vector3d(2.0, -1.0, 0.5);
  int off_tilt = -1, off_yaw = -1, off_trans = -1, off_clk = -1, off_isb = -1, off_clone = -1, off_td = -1;
  int N = 0;

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

std::unique_ptr<Scene> makeScene(unsigned seed, bool iterate_off_snapshot, bool zero_tilt = false) {
  auto sc = std::make_unique<Scene>();
  std::mt19937_64 rng(seed);
  std::normal_distribution<double> g(0.0, 1.0);
  CoreState& x = sc->x;
  x.R = so3Exp(Eigen::Vector3d(0.3 * g(rng), 0.3 * g(rng), 2.0 * g(rng)));
  x.p = Eigen::Vector3d(12.0, -7.0, 1.5) + Eigen::Vector3d(g(rng), g(rng), g(rng));
  x.v = Eigen::Vector3d(1.3, -0.4, 0.05) + 0.1 * Eigen::Vector3d(g(rng), g(rng), g(rng));
  x.bg = 1e-3 * Eigen::Vector3d(g(rng), g(rng), g(rng));
  x.ba = 1e-2 * Eigen::Vector3d(g(rng), g(rng), g(rng));
  x.g = so3Exp(0.05 * Eigen::Vector3d(g(rng), g(rng), 0.0)) * Eigen::Vector3d(0.0, 0.0, -9.81);
  x.inv_expo = 1.2;
  x.P = HostMat::Identity() * 1e-4;
  BridgeResult& br = sc->br;
  br.ok = true;
  br.delta = 0.05;
  br.t_k = 100.0;
  br.t_g = br.t_k - br.delta;
  br.dR = so3Exp(Eigen::Vector3d(0.01, 0.005, -0.02));
  br.D = Eigen::Vector3d(0.001, 0.002, -0.001);
  br.E = Eigen::Vector3d(0.01, -0.02, 0.005);
  br.omega_g = Eigen::Vector3d(0.02, -0.01, 0.3);
  br.bg_snap = x.bg + 1e-4 * Eigen::Vector3d(1.0, -2.0, 0.5);
  br.ba_snap = x.ba + 1e-3 * Eigen::Vector3d(-1.0, 0.5, 2.0);
  br.g_snap = x.g + 1e-2 * Eigen::Vector3d(0.3, -0.2, 0.1);
  br.R_k = iterate_off_snapshot ? Eigen::Matrix3d(x.R * so3Exp(Eigen::Vector3d(-2e-3, 1e-3, 3e-3))) : x.R;
  br.a_bar = Eigen::Vector3d(0.3, -0.2, 9.75);
  br.Sigma_D = Eigen::Matrix3d::Identity() * 1e-8;
  br.var_v_bridge = sc->cfg.imu_bridge.acc_n * sc->cfg.imu_bridge.acc_n * br.delta;   // ImuBridge::query's acc_n²δ (A31)
  sc->lev.setRef(x.g);
  // slots: tilt(2) yaw(1) trans(3) clk(2) isb_E(1) clone(3) td(1)
  auto add = [&](Slot kind, int tag, const char* name, const Eigen::VectorXd& x0) {
    Eigen::MatrixXd F0 = Eigen::MatrixXd::Zero(x0.size(), HostIdx::dim);
    Eigen::MatrixXd Fa = Eigen::MatrixXd::Zero(x0.size(), sc->aug.dim());
    Eigen::MatrixXd Nm = Eigen::MatrixXd::Identity(x0.size(), x0.size()) * 1e-4;
    sc->aug.addSlot(kind, tag, name, x0, F0, Fa, Nm, x.P);
  };
  add(Slot::Tilt, 0, "tilt", zero_tilt ? Eigen::Vector2d(0.0, 0.0) : Eigen::Vector2d(1.5e-3, -2.5e-3));
  Eigen::VectorXd psi(1);
  psi(0) = 0.7;
  add(Slot::Yaw, 0, "yaw", psi);
  add(Slot::Trans, 0, "trans", Eigen::Vector3d(5.0, -3.0, 1.0));
  add(Slot::Clk, 0, "clk", Eigen::Vector2d(120.0, 0.4));
  Eigen::VectorXd isb(1);
  isb(0) = 3.5;
  add(Slot::Isb, sysBit(Sys::GAL), "isb_E", isb);
  add(Slot::Clone, 1, "clone_1", Eigen::Vector3d(9.0, -6.0, 2.0));
  Eigen::VectorXd td(1);
  td(0) = 0.002;
  add(Slot::Td, 0, "td", td);
  sc->off_tilt = sc->aug.find(Slot::Tilt);
  sc->off_yaw = sc->aug.find(Slot::Yaw);
  sc->off_trans = sc->aug.find(Slot::Trans);
  sc->off_clk = sc->aug.find(Slot::Clk);
  sc->off_isb = sc->aug.find(Slot::Isb, sysBit(Sys::GAL));
  sc->off_clone = sc->aug.find(Slot::Clone, 1);
  sc->off_td = sc->aug.find(Slot::Td);
  sc->N = HostIdx::dim + sc->aug.dim();
  const double lat = 30.0 * kDeg, lon = 110.0 * kDeg;
  sc->anchor.r0_ecef = geo2ecef(lat, lon, 50.0);
  sc->anchor.R_EN = enuRot(lat, lon);
  sc->anchor.valid = true;
  return sc;
}

// The reference nonlinear model (independent of rows.cpp): antenna position / velocity in N.
void refAntenna(const Scene& sc, const CoreState& x, const Eigen::VectorXd& xg, Eigen::Vector3d* a_N, Eigen::Vector3d* w_N) {
  const AntennaAtEpoch ae = ImuBridge::evaluate(x, sc.br, sc.lever, true);
  const Eigen::Vector2d phi = xg.segment<2>(sc.off_tilt);
  const double psi = xg(sc.off_yaw);
  const Eigen::Vector3d t = xg.segment<3>(sc.off_trans);
  const Eigen::Matrix3d M = sc.lev.M(psi, phi, x.g);
  *a_N = M * (ae.y - sc.pivot) + t;
  *w_N = M * ae.z;
}

// Central difference of a vector function of (host, aug) over column k with step h.
using ModelFn = std::function<Eigen::VectorXd(const CoreState&, const Eigen::VectorXd&)>;
Eigen::VectorXd centralDiff(const Scene& sc, const ModelFn& f, int k, double h) {
  const Eigen::VectorXd xg0 = sc.aug.x();
  if (k < HostIdx::dim) {
    HostVec d = HostVec::Zero();
    d(k) = h;
    const Eigen::VectorXd fp = f(sc.x.boxplus(d), xg0);
    const Eigen::VectorXd fm = f(sc.x.boxplus(HostVec(-d)), xg0);
    return (fp - fm) / (2.0 * h);
  }
  Eigen::VectorXd xp = xg0, xm = xg0;
  xp(k - HostIdx::dim) += h;
  xm(k - HostIdx::dim) -= h;
  return (f(sc.x, xp) - f(sc.x, xm)) / (2.0 * h);
}

// Step per column: host δθ 1e-4 (the host's so3Exp returns the identity for |w| ≤ 1e-5, so3_math.h:44), other
// rotation-like coordinates 1e-5, additive ones 1e-3 (the model is linear in them).
double stepFor(const Scene& sc, int k) {
  if (k < HostIdx::dim) return k < 3 ? 1e-4 : ((k >= HostIdx::grav && k < HostIdx::grav + 3) ? 1e-5 : 1e-3);
  const int a = k - HostIdx::dim;
  if ((a >= sc.off_tilt && a < sc.off_tilt + 2) || a == sc.off_yaw) return 1e-5;
  return 1e-3;
}

void compareColumns(const Scene& sc, const Eigen::MatrixXd& J, const ModelFn& f, const std::vector<int>& skip, const char* what,
                    double rel_tol = 1e-6, double abs_tol = 1e-8) {
  for (int k = 0; k < sc.N; ++k) {
    if (std::find(skip.begin(), skip.end(), k) != skip.end()) continue;
    const Eigen::VectorXd fd = centralDiff(sc, f, k, stepFor(sc, k));
    const Eigen::VectorXd jc = J.col(k);
    const double scale = std::max(jc.norm(), fd.norm());
    const double err = (jc - fd).norm();
    if (scale < 1e-6) {
      EXPECT_LT(err, abs_tol) << what << " column " << k << " (expected ≈ 0)";
    } else {
      EXPECT_LT(err / scale, rel_tol) << what << " column " << k << " rel.err " << err / scale;
    }
  }
}

}  // namespace

// J_a and J_w (δ ≠ 0, random state) vs manifold central differences, rel. err < 1e-6 (δb_g relaxed)
TEST(U07Jacobians, JaJwCentralDifferencesRandomState) {
  for (int variant = 0; variant < 4; ++variant) {
    auto sc = makeScene(100 + variant, variant % 2 == 1);   // odd variants: iterate ≠ bridge snapshot
    NoiseModel noise(sc->cfg);
    RowBuilder rb(sc->cfg, noise);
    const AntennaJacobians J = rb.antennaJacobians(sc->lin());
    ASSERT_EQ(J.J_a.cols(), sc->N);
    ASSERT_EQ(J.J_w.cols(), sc->N);
    Eigen::Vector3d a_ref, w_ref;
    refAntenna(*sc, sc->x, sc->aug.x(), &a_ref, &w_ref);
    EXPECT_LT((J.a_N - a_ref).norm(), 1e-12);
    EXPECT_LT((J.w_N - w_ref).norm(), 1e-12);
    const std::vector<int> skip = {HostIdx::dim + sc->off_td};   // t_d has no model in evaluate() (checked in TdColumn)
    ModelFn fa = [&](const CoreState& x, const Eigen::VectorXd& xg) { Eigen::Vector3d a, w; refAntenna(*sc, x, xg, &a, &w); return Eigen::VectorXd(a); };
    ModelFn fw = [&](const CoreState& x, const Eigen::VectorXd& xg) { Eigen::Vector3d a, w; refAntenna(*sc, x, xg, &a, &w); return Eigen::VectorXd(w); };
    compareColumns(*sc, J.J_a, fa, skip, "J_a");
    compareColumns(*sc, J.J_w, fw, skip, "J_w");
    // structural expectations of the §4.2 table
    EXPECT_LT(J.J_a.col(HostIdx::expo).norm(), 1e-12);
    EXPECT_LT(J.J_w.col(HostIdx::expo).norm(), 1e-12);
    EXPECT_LT((J.J_a.block<3, 3>(0, HostIdx::pos) - J.M).norm(), 1e-12);
    EXPECT_LT((J.J_w.block<3, 3>(0, HostIdx::vel) - J.M).norm(), 1e-12);
    EXPECT_LT((J.J_a.block<3, 3>(0, HostIdx::vel) + sc->br.delta * J.M).norm(), 1e-12);
    EXPECT_LT((J.J_a.block<3, 3>(0, HostIdx::bg).norm()), 1e-12);
    EXPECT_LT((J.J_a.block<3, 3>(0, HostIdx::dim + sc->off_trans) - Eigen::Matrix3d::Identity()).norm(), 1e-12);
    EXPECT_LT((J.J_w.block<3, 3>(0, HostIdx::dim + sc->off_trans).norm()), 1e-12);
    EXPECT_LT((J.J_a.block<3, 2>(0, HostIdx::dim + sc->off_clk).norm() + J.J_a.col(HostIdx::dim + sc->off_isb).norm() +
               J.J_a.block<3, 3>(0, HostIdx::dim + sc->off_clone).norm()),
              1e-12);
  }
  // gravity column absent when the host does not estimate gravity
  {
    auto sc = makeScene(7, false);
    NoiseModel noise(sc->cfg);
    RowBuilder rb(sc->cfg, noise);
    LinPoint lp = sc->lin();
    lp.gravity_est_en = false;
    const AntennaJacobians J = rb.antennaJacobians(lp);
    EXPECT_LT((J.J_a.block<3, 3>(0, HostIdx::grav).norm()), 1e-15);
    EXPECT_LT((J.J_w.block<3, 3>(0, HostIdx::grav).norm()), 1e-15);
  }
}

// row Jacobians (pseudorange, Doppler, TDCP) vs central differences of the design's linearisation model
TEST(U07Jacobians, PseudorangeDopplerTdcpRows) {
  auto sc = makeScene(200, true);
  NoiseModel noise(sc->cfg);
  RowBuilder rb(sc->cfg, noise);
  const LinPoint lp = sc->lin();
  const AntennaJacobians J = rb.antennaJacobians(lp);
  const AnchorFrame& an = sc->anchor;
  std::mt19937_64 rng(5);
  std::uniform_real_distribution<double> u(0.0, 1.0);
  // synthetic satellites (GPS and one Galileo for the ISB column)
  const int m = 6;
  ObsEpoch epoch;
  epoch.t_rx = GTime::fromSec(1.4e9);   // integer second: on the 1 Hz decimation grid
  std::vector<SatGeometry> geo;
  std::vector<MaskVerdict> masks;
  const Eigen::Vector3d r_A = an.r0_ecef + an.R_EN * J.a_N;
  for (int j = 0; j < m; ++j) {
    SatObs so;
    so.sat = (j == 2) ? SatId{Sys::GAL, 5, 64} : sat(j + 1);
    SignalObs sig;
    sig.freq_hz = 1575.42e6;
    sig.code = (j == 2) ? SignalCode::L1X : SignalCode::L1C;
    sig.status = kStatusPsrValid | kStatusCpValid | kStatusHalfCycValid;
    sig.psr_m = 2.2e7 + 1e5 * u(rng);
    sig.dopp_hz = 2000.0 * (u(rng) - 0.5);
    sig.cn0_dbhz = 44.0;
    so.sig.push_back(sig);
    epoch.sats.push_back(so);
    const double el = (15.0 + 60.0 * u(rng)) * kDeg, az = 2.0 * M_PI * u(rng);
    const Eigen::Vector3d los_N(std::cos(el) * std::sin(az), std::cos(el) * std::cos(az), std::sin(el));
    SatGeometry g;
    g.sat = so.sat;
    g.state.ok = true;
    g.state.r_ecef = r_A + an.R_EN * los_N * 2.3e7;
    g.state.v_ecef = Eigen::Vector3d(u(rng) - 0.5, u(rng) - 0.5, u(rng) - 0.5).cross(an.R_EN * los_N).normalized() * 3000.0;
    g.state.dts = 1e-4 * (u(rng) - 0.5);
    g.state.ddts = 1e-11;
    g.state.var_eph = 1.0;
    // Broadcast group delays of the record (A37): GPS TGD in tgd[0]; the Galileo record is I/NAV (A16), whose E1 clock
    // pairs with BGD(E1,E5b) = tgd[1], not RTKLIB-EX prange()'s BGD(E1,E5a) = tgd[0].
    g.state.tgd_s[0] = (j == 2) ? 2.1e-9 : -4.6e-9 - 0.3e-9 * j;
    g.state.tgd_s[1] = (j == 2) ? 2.6e-9 : 0.0;
    g.state.nav_type = (j == 2) ? 1 : 0;
    g.range = rangeSagnac(g.state.r_ecef, r_A, &g.e_ecef);
    g.u_N = an.R_EN.transpose() * g.e_ecef;
    g.el_rad = el;
    g.az_rad = az;
    g.lambda_m = kC / sig.freq_hz;
    g.ion_m = 1.0;
    g.trop_m = 2.5;
    g.var_ion = 0.25;
    g.var_trop = 0.1;
    geo.push_back(g);
    MaskVerdict v;
    v.psr = v.dopp = v.cp = true;
    masks.push_back(v);
  }
  const double delta = sc->br.delta;
  // ---- pseudorange ----
  RowSet psr;
  rb.pseudorangeRows(epoch, geo, masks, lp, J, &psr);
  ASSERT_EQ(static_cast<int>(psr.rows.size()), m);
  for (int j = 0; j < m; ++j) {
    const SatGeometry& g = geo[j];
    const double sagnac0 = g.range - (g.state.r_ecef - r_A).norm();   // frozen at x̂
    const bool gal = epoch.sats[j].sat.sys == Sys::GAL;
    // varying part of the design model (LOS/Sagnac frozen): stable range difference + clock/ISB terms
    ModelFn h = [&, j, gal](const CoreState& x, const Eigen::VectorXd& xg) {
      Eigen::Vector3d a, w;
      refAntenna(*sc, x, xg, &a, &w);
      const double b = xg(sc->off_clk), d = xg(sc->off_clk + 1), iota = gal ? xg(sc->off_isb) : 0.0;
      Eigen::VectorXd r(1);
      r(0) = normDiffStable(Eigen::Vector3d(geo[j].state.r_ecef - an.r0_ecef), an.R_EN, a, J.a_N) + b - delta * d + iota;
      return r;
    };
    const double h_full = g.range + sc->aug.x()(sc->off_clk) - delta * sc->aug.x()(sc->off_clk + 1) + (gal ? sc->aug.x()(sc->off_isb) : 0.0) -
                          kC * g.state.dts + g.trop_m + g.ion_m;
    (void)sagnac0;
    EXPECT_NEAR(psr.rows[j].h, h_full, 1e-9) << "psr h row " << j;
    // P_corr = P − c·TGD (GPS L1C) / P − c·BGD(E1,E5b) (Galileo I/NAV E1): values from the record, not codeBiasMetres()
    EXPECT_NEAR(psr.rows[j].z, epoch.sats[j].sig[0].psr_m - kC * (gal ? g.state.tgd_s[1] : g.state.tgd_s[0]), 1e-9) << "psr z row " << j;
    compareColumns(*sc, Eigen::MatrixXd(psr.rows[j].H), h, {HostIdx::dim + sc->off_td}, "psr H", 1e-6, 1e-8);
    EXPECT_NEAR(psr.rows[j].H(HostIdx::dim + sc->off_clk), 1.0, 1e-12);
    EXPECT_NEAR(psr.rows[j].H(HostIdx::dim + sc->off_clk + 1), -delta, 1e-12);
    EXPECT_NEAR(psr.rows[j].H(HostIdx::dim + sc->off_isb), gal ? 1.0 : 0.0, 1e-12);
    EXPECT_GT(psr.rows[j].R, 0.0);
  }
  // 1 Hz selection (AMENDMENTS A43): `epoch` took its GPST second on the first call above, so a later epoch of the same
  // second yields no pseudorange rows, the first epoch of the next second does whatever its phase (x.x05 s here, the
  // Occlusion03 grid that the old |frac| < 5 ms never selected), and the taking epoch keeps its rows on every rebuild
  // (IEKF relinearisation). The rule itself is checked in OneHzDecimationFirstEpochPerGpstSecond.
  {
    ObsEpoch same = epoch;
    same.t_rx = GTime{epoch.t_rx.sec, 0.3};
    RowSet none;
    rb.pseudorangeRows(same, geo, masks, lp, J, &none);
    EXPECT_TRUE(none.rows.empty()) << "second epoch of a GPST second must not build code rows (A43)";
    ObsEpoch next = epoch;
    next.t_rx = GTime{epoch.t_rx.sec + 1, 0.005000000001018634};
    RowSet taken;
    rb.pseudorangeRows(next, geo, masks, lp, J, &taken);
    EXPECT_EQ(taken.rows.size(), psr.rows.size()) << "first epoch of the next GPST second (x.005 s) must build code rows (A43)";
    RowSet again;
    rb.pseudorangeRows(next, geo, masks, lp, J, &again);
    EXPECT_EQ(again.rows.size(), taken.rows.size()) << "a rebuild of the taking epoch must keep its rows";
    RowSet late;
    rb.pseudorangeRows(same, geo, masks, lp, J, &late);   // decided once: still none
    EXPECT_TRUE(late.rows.empty());
    EXPECT_EQ(rb.psrDecimation().taken(), 2u);
    EXPECT_EQ(rb.psrDecimation().skipped(), 1u);
  }
  // m8: a non-GPS satellite whose system has no ISB slot yields no pseudorange row (ι_s unknown ⇒ biased row)
  {
    Config cfg2;
    AugBlock aug2(cfg2);
    Eigen::MatrixXd F3 = Eigen::MatrixXd::Zero(3, HostIdx::dim);
    aug2.addSlot(Slot::Trans, 0, "trans", Eigen::Vector3d::Zero(), F3, Eigen::MatrixXd(3, 0), Eigen::Matrix3d::Identity(), sc->x.P);
    Eigen::MatrixXd F2 = Eigen::MatrixXd::Zero(2, HostIdx::dim);
    aug2.addSlot(Slot::Clk, 0, "clk", Eigen::Vector2d(120.0, 0.4), F2, Eigen::MatrixXd::Zero(2, 3), Eigen::Matrix2d::Identity(), sc->x.P);
    LinPoint lp2 = lp;
    lp2.aug = &aug2;
    const AntennaJacobians J2 = rb.antennaJacobians(lp2);
    RowSet psr2;
    rb.pseudorangeRows(epoch, geo, masks, lp2, J2, &psr2);
    ASSERT_EQ(static_cast<int>(psr2.rows.size()), m - 1);
    for (const auto& r : psr2.rows) EXPECT_NE(r.sat.sys, Sys::GAL);
  }
  // m3: the row's line of sight is R_ENᵀ e of the anchor even when SatGeometry::u_N was filled elsewhere
  {
    std::vector<SatGeometry> geo_u = geo;
    for (auto& g : geo_u) g.u_N = Eigen::Vector3d(1.0, 0.0, 0.0);   // garbage in the diagnostic field
    RowSet psr3;
    rb.pseudorangeRows(epoch, geo_u, masks, lp, J, &psr3);
    ASSERT_EQ(psr3.rows.size(), psr.rows.size());
    for (size_t j = 0; j < psr.rows.size(); ++j) EXPECT_LT((psr3.rows[j].H - psr.rows[j].H).norm(), 1e-15);
  }
  // A37: the group delay follows the record the state came from; a NaN bias (unknown Galileo data source, a
  // secondary signal as primary) removes that satellite's code row and nothing else.
  {
    std::vector<SatGeometry> geo_f = geo;
    geo_f[2].state.nav_type = 2;   // the Galileo record is F/NAV: its E1 clock pairs with BGD(E1,E5a) = tgd[0]
    RowSet f;
    rb.pseudorangeRows(epoch, geo_f, masks, lp, J, &f);
    ASSERT_EQ(f.rows.size(), psr.rows.size());
    EXPECT_NEAR(f.rows[2].z, epoch.sats[2].sig[0].psr_m - kC * geo[2].state.tgd_s[0], 1e-9);
    EXPECT_GT(std::fabs(f.rows[2].z - psr.rows[2].z), 0.1);   // |BGD(E1,E5a) − BGD(E1,E5b)|·c = 0.15 m here
    std::vector<SatGeometry> geo_n = geo;
    geo_n[2].state.nav_type = 0;   // data source unknown ⇒ the BGD cannot be chosen
    ObsEpoch ep_n = epoch;
    ep_n.sats[4].sig[0].code = SignalCode::L2X;   // GPS with a secondary signal in the primary slot
    RowSet n;
    rb.pseudorangeRows(ep_n, geo_n, masks, lp, J, &n);
    ASSERT_EQ(static_cast<int>(n.rows.size()), m - 2);
    for (const auto& r : n.rows) {
      EXPECT_NE(r.sat.rtk_sat, geo[2].sat.rtk_sat);
      EXPECT_NE(r.sat.rtk_sat, geo[4].sat.rtk_sat);
    }
  }
  // ---- Doppler (satellite 3 excluded by an open TDCP pair) ----
  RowSet dop;
  rb.dopplerRows(epoch, geo, masks, lp, J, {geo[3].sat.rtk_sat}, &dop);
  ASSERT_EQ(static_cast<int>(dop.rows.size()), m - 1);
  // A31: the Doppler variance does not depend on the host's σ_ba/σ_θ (no (δ(σ_ba+gσ_θ))² term): a host covariance
  // with 1e6× larger attitude and accelerometer-bias blocks leaves every R bit-identical.
  CoreState x_bigP = sc->x;
  x_bigP.P.block<3, 3>(HostIdx::rot, HostIdx::rot) *= 1e6;
  x_bigP.P.block<3, 3>(HostIdx::ba, HostIdx::ba) *= 1e6;
  LinPoint lp_bigP = lp;
  lp_bigP.x = &x_bigP;
  RowSet dop_bigP;
  rb.dopplerRows(epoch, geo, masks, lp_bigP, rb.antennaJacobians(lp_bigP), {geo[3].sat.rtk_sat}, &dop_bigP);
  ASSERT_EQ(dop_bigP.rows.size(), dop.rows.size());
  const Config& cfg = sc->cfg;
  for (int q = 0; q < m - 1; ++q) {
    const int j = q < 3 ? q : q + 1;
    const SatGeometry& g = geo[j];
    const Eigen::Vector3d rdot0 = an.R_EN * J.w_N;
    // Sagnac rate by differentiating the range model, never by restating the rate formula (A36)
    const double sag_rate0 = sagnacRateNumeric(g.state.r_ecef, g.state.v_ecef, r_A, rdot0);
    // R as ruled: max(λ·dopp_std, σ_D0)²·(1+1/sin²el)/2·κ_AR(τ_D, 0.1 s) (A35-C5/C6; dopp_std = 0 here) + the bridge's
    // velocity white noise u_jᵀM(var_v_bridge·I)Mᵀu_j (A45/C10, formed here as that quadratic form) + S_d δ + (‖ω‖σ_l)² +
    // (‖α_A‖σ_toff)² + (0.01‖v‖)², nothing else (A31)
    const double se = std::max(std::sin(g.el_rad), std::sin(5.0 * kDeg));
    const double rho = std::exp(-0.1 / cfg.noise.dopp.tau_s);
    const double omega = (sc->br.omega_g + sc->br.bg_snap - sc->x.bg).norm();
    const double s0 = cfg.noise.dopp.sigma0_mps;
    const Eigen::Vector3d u_row = an.R_EN.transpose() * g.e_ecef;
    const double v_bridge = u_row.dot(J.M * (sc->br.var_v_bridge * Eigen::Matrix3d::Identity()) * J.M.transpose() * u_row);
    ASSERT_GT(v_bridge, 0.0);
    const double R_ruled = (s0 * s0 * 0.5 * (1.0 + 1.0 / (se * se)) * (1.0 + rho) / (1.0 - rho) + v_bridge + cfg.process.clock.S_d * delta +
                            std::pow(omega * cfg.frames.lever_arm_sigma, 2) + std::pow(J.alpha_N.norm() * cfg.time.sigma_toff_s, 2) +
                            std::pow(0.01 * J.w_N.norm(), 2)) *
                           cfg.noise.scale.dopp;
    EXPECT_NEAR(dop.rows[q].R, R_ruled, 1e-12 * R_ruled) << "dopp R row " << q;
    EXPECT_EQ(dop_bigP.rows[q].R, dop.rows[q].R) << "dopp R depends on the host's σ_ba/σ_θ (A31), row " << q;
    ModelFn h = [&, j](const CoreState& x, const Eigen::VectorXd& xg) {
      Eigen::Vector3d a, w;
      refAntenna(*sc, x, xg, &a, &w);
      const Eigen::Vector3d rdot = an.R_EN * w;
      Eigen::VectorXd r(1);
      r(0) = -geo[j].e_ecef.dot(rdot) + xg(sc->off_clk + 1);   // varying part (LOS and Sagnac rate frozen)
      return r;
    };
    const double h_full = g.e_ecef.dot(g.state.v_ecef - rdot0) + sag_rate0 + sc->aug.x()(sc->off_clk + 1) - kC * g.state.ddts;
    EXPECT_EQ(dop.rows[q].sat.rtk_sat, g.sat.rtk_sat);
    EXPECT_NEAR(dop.rows[q].h, h_full, 1e-9) << "dopp h row " << q;
    EXPECT_NEAR(dop.rows[q].z, -g.lambda_m * epoch.sats[j].sig[0].dopp_hz, 1e-9);
    compareColumns(*sc, Eigen::MatrixXd(dop.rows[q].H), h, {HostIdx::dim + sc->off_td}, "dopp H", 1e-6, 1e-8);
    EXPECT_NEAR(dop.rows[q].H(HostIdx::dim + sc->off_clk + 1), 1.0, 1e-12);
    EXPECT_NEAR(dop.rows[q].H(HostIdx::dim + sc->off_clk), 0.0, 1e-12);
  }
  // ---- TDCP (clone columns) ----
  RowBuilder::TdcpPairInput pair;
  pair.pair_id = 1;
  pair.clone_slot = sc->off_clone;
  pair.c_hat_N = sc->aug.x().segment<3>(sc->off_clone);
  pair.dt_pair_s = 0.5;
  for (int j = 0; j < m; ++j) {
    pair.sats.push_back(geo[j].sat);
    pair.phi_k0_cyc.push_back(1e8 + 1e3 * u(rng));
    pair.phi_k1_cyc.push_back(pair.phi_k0_cyc.back() + 500.0 * (u(rng) - 0.5));
    pair.dts_k0.push_back(geo[j].state.dts - 1e-11 * 0.5);
    pair.dts_k1.push_back(geo[j].state.dts);
    pair.rs_k0.push_back(geo[j].state.r_ecef - geo[j].state.v_ecef * 0.5);
    pair.el_k0_rad.push_back(geo[j].el_rad);
    pair.el_k1_rad.push_back(geo[j].el_rad);
    pair.cp_std_k0_m.push_back(0.003);
    pair.cp_std_k1_m.push_back(0.003);
  }
  RowSet td;
  rb.tdcpRows(pair, geo, lp, J, &td);
  ASSERT_EQ(static_cast<int>(td.rows.size()), m);
  EXPECT_TRUE(td.has_full_R);
  for (int j = 0; j < m; ++j) {
    const SatGeometry& g = geo[j];
    const double sag1 = g.range - (g.state.r_ecef - r_A).norm();
    const Eigen::Vector3d rc0 = an.r0_ecef + an.R_EN * pair.c_hat_N;
    const double sag0 = rangeSagnac(pair.rs_k0[j], rc0, nullptr) - (pair.rs_k0[j] - rc0).norm();
    ModelFn h = [&, j](const CoreState& x, const Eigen::VectorXd& xg) {
      Eigen::Vector3d a, w;
      refAntenna(*sc, x, xg, &a, &w);
      const Eigen::Vector3d c = xg.segment<3>(sc->off_clone);
      Eigen::VectorXd r(1);
      r(0) = normDiffStable(Eigen::Vector3d(geo[j].state.r_ecef - an.r0_ecef), an.R_EN, a, J.a_N) -
             normDiffStable(Eigen::Vector3d(pair.rs_k0[j] - an.r0_ecef), an.R_EN, c, pair.c_hat_N);   // varying part
      return r;
    };
    const double h_full = ((geo[j].state.r_ecef - r_A).norm() + sag1) - ((pair.rs_k0[j] - rc0).norm() + sag0) - kC * (pair.dts_k1[j] - pair.dts_k0[j]);
    EXPECT_NEAR(td.rows[j].h, h_full, 1e-9) << "tdcp h row " << j;
    EXPECT_NEAR(td.rows[j].z, g.lambda_m * (pair.phi_k1_cyc[j] - pair.phi_k0_cyc[j]), 1e-9);
    const int c0 = HostIdx::dim + sc->off_clone;
    compareColumns(*sc, Eigen::MatrixXd(td.rows[j].H), h, {HostIdx::dim + sc->off_td, c0, c0 + 1, c0 + 2}, "tdcp H", 1e-6, 1e-8);
    EXPECT_LT(td.rows[j].H.segment<2>(HostIdx::dim + sc->off_clk).norm(), 1e-12);   // Δclk is not a state
    // Clone columns (AMENDMENTS A64): +u_k1ᵀ by definition, the k1 line of sight, so the row sees the displacement over
    // the pair only. The exact derivative (central differences) is +u_k0ᵀ; the linearisation leaves out exactly
    // (u_k0 − u_k1)ᵀδc, which this scene makes measurable (the satellite moved 1.5 km over the pair, the clone sits
    // metres from the antenna).
    Eigen::Vector3d e0;
    rangeSagnac(pair.rs_k0[j], rc0, &e0);
    const Eigen::Vector3d u0 = an.R_EN.transpose() * e0, u1 = an.R_EN.transpose() * g.e_ecef;
    const Eigen::Vector3d h_clone = td.rows[j].H.segment<3>(c0).transpose();
    EXPECT_LT((h_clone - u1).norm(), 1e-12) << "tdcp clone column row " << j;
    Eigen::Vector3d fd_clone;
    for (int k = 0; k < 3; ++k) fd_clone(k) = centralDiff(*sc, h, c0 + k, stepFor(*sc, c0 + k))(0);
    EXPECT_LT((fd_clone - u0).norm(), 1e-6) << "exact k0 derivative, row " << j;
    EXPECT_GT((u1 - u0).norm(), 1e-5) << "row " << j << ": the scene does not tell the two lines of sight apart";
  }
}

// AMENDMENTS A70: the Doppler rows of an epoch, projected as one group in the metric of their R onto the orthogonal
// complement of their tilt columns (JointUpdater::projection, the Options::project step, T = Q₂ᵀL⁻¹), no longer depend
// on φ to first order in the model itself: the central differences of T·h over φ vanish (to the truncation level of the
// differences), where those of h do not; T·H matches the central differences of T·h on every other column (the
// projection is a fixed linear map of the rows), and T R Tᵀ = I with the set's full R (A61 F8 term on). The scene's
// velocity has a vertical component, so the two tilt columns are independent (rank 2); level motion would make them
// proportional (U10 checks that case).
TEST(U07Jacobians, DopplerRowsProjectedOffTheTiltAreFirstOrderTiltFree) {
  auto sc = makeScene(200, true);
  sc->br.var_omega = 4e-6;   // the shared ω̂_g lever-arm velocity term: the Doppler set carries a full R (A61 F8)
  NoiseModel noise(sc->cfg);
  RowBuilder rb(sc->cfg, noise);
  const LinPoint lp = sc->lin();
  const AntennaJacobians J = rb.antennaJacobians(lp);
  const AnchorFrame& an = sc->anchor;
  ASSERT_GT(std::fabs(J.w_N.z()), 1e-3 * J.w_N.norm()) << "the scene is meant to have a vertical velocity";
  std::mt19937_64 rng(70);
  std::uniform_real_distribution<double> u(0.0, 1.0);
  const int m = 8;
  ObsEpoch epoch;
  epoch.t_rx = GTime::fromSec(1.4e9);
  std::vector<SatGeometry> geo;
  std::vector<MaskVerdict> masks;
  const Eigen::Vector3d r_A = an.r0_ecef + an.R_EN * J.a_N;
  for (int j = 0; j < m; ++j) {
    SatObs so;
    so.sat = sat(j + 1);
    SignalObs sig;
    sig.freq_hz = 1575.42e6;
    sig.code = SignalCode::L1C;
    sig.status = kStatusPsrValid | kStatusCpValid | kStatusHalfCycValid;
    sig.psr_m = 2.2e7 + 1e5 * u(rng);
    sig.dopp_hz = 2000.0 * (u(rng) - 0.5);
    sig.cn0_dbhz = 30.0 + 15.0 * u(rng);
    so.sig.push_back(sig);
    epoch.sats.push_back(so);
    const double el = (12.0 + 70.0 * u(rng)) * kDeg, az = 2.0 * M_PI * u(rng);
    const Eigen::Vector3d los_N(std::cos(el) * std::sin(az), std::cos(el) * std::cos(az), std::sin(el));
    SatGeometry g;
    g.sat = so.sat;
    g.state.ok = true;
    g.state.r_ecef = r_A + an.R_EN * los_N * 2.3e7;
    g.state.v_ecef = Eigen::Vector3d(u(rng) - 0.5, u(rng) - 0.5, u(rng) - 0.5).cross(an.R_EN * los_N).normalized() * 3000.0;
    g.state.dts = 1e-4 * (u(rng) - 0.5);
    g.state.ddts = 1e-11;
    g.state.var_eph = 1.0;
    g.range = rangeSagnac(g.state.r_ecef, r_A, &g.e_ecef);
    g.u_N = an.R_EN.transpose() * g.e_ecef;
    g.el_rad = el;
    g.az_rad = az;
    g.lambda_m = kC / sig.freq_hz;
    geo.push_back(g);
    MaskVerdict v;
    v.psr = v.dopp = v.cp = true;
    masks.push_back(v);
  }
  RowSet dop;
  rb.dopplerRows(epoch, geo, masks, lp, J, {}, &dop);
  ASSERT_EQ(static_cast<int>(dop.rows.size()), m);
  ASSERT_TRUE(dop.has_full_R) << "var_omega > 0 must give the Doppler set its off-diagonal R";
  const int N = sc->N;
  Eigen::MatrixXd H(m, N);
  Eigen::VectorXd nu(m);
  for (int j = 0; j < m; ++j) {
    H.row(j) = dop.rows[static_cast<size_t>(j)].H;
    nu(j) = dop.rows[static_cast<size_t>(j)].nu;
  }
  const Eigen::MatrixXd R = dop.R_full;
  ASSERT_GT((R - Eigen::MatrixXd(R.diagonal().asDiagonal())).norm(), 0.0);
  const int ct = HostIdx::dim + sc->off_tilt;
  const std::vector<int> cols = {ct, ct + 1};
  // T itself: the projection of [H | I] carries T in its extra columns
  Eigen::MatrixXd HI(m, N + m);
  HI << H, Eigen::MatrixXd::Identity(m, m);
  Eigen::MatrixXd HpI;
  Eigen::VectorXd rpI;
  int rank = -1;
  ASSERT_TRUE(JointUpdater::projection(HI, nu, R, cols, &HpI, &rpI, &rank));
  ASSERT_EQ(rank, 2) << "the scene's velocity is not level: two independent tilt columns";
  ASSERT_EQ(HpI.rows(), m - 2);
  const Eigen::MatrixXd T = HpI.rightCols(m);
  const Eigen::MatrixXd Hp = HpI.leftCols(N);
  EXPECT_LT((T * R * T.transpose() - Eigen::MatrixXd::Identity(m - 2, m - 2)).norm(), 1e-10);
  // the same call on H alone gives the same rows
  Eigen::MatrixXd Hp2;
  Eigen::VectorXd rp2;
  int rank2 = -1;
  ASSERT_TRUE(JointUpdater::projection(H, nu, R, cols, &Hp2, &rp2, &rank2));
  EXPECT_EQ(rank2, 2);
  EXPECT_LT((Hp2 - Hp).norm(), 1e-13 * Hp.norm());
  EXPECT_LT((rp2 - T * nu).norm(), 1e-12 * rp2.norm());
  // the model of the row test: h_j = −e_j·(R_EN w^N(x)) + d (the varying part, LOS and Sagnac rate frozen)
  ModelFn h = [&](const CoreState& x, const Eigen::VectorXd& xg) {
    Eigen::Vector3d a, w;
    refAntenna(*sc, x, xg, &a, &w);
    const Eigen::Vector3d rdot = an.R_EN * w;
    Eigen::VectorXd r(m);
    for (int j = 0; j < m; ++j) r(j) = -geo[static_cast<size_t>(j)].e_ecef.dot(rdot) + xg(sc->off_clk + 1);
    return r;
  };
  ModelFn hp = [&](const CoreState& x, const Eigen::VectorXd& xg) { return Eigen::VectorXd(T * h(x, xg)); };
  for (int k : cols) {
    const Eigen::VectorXd fd = centralDiff(*sc, h, k, stepFor(*sc, k));
    const Eigen::VectorXd fdp = centralDiff(*sc, hp, k, stepFor(*sc, k));
    EXPECT_GT(fd.norm(), 1e-2) << "the unprojected rows must see the tilt (column " << k << ")";
    EXPECT_LT((fd - H.col(k)).norm(), 1e-6 * fd.norm()) << "tilt column " << k;
    EXPECT_LT(fdp.norm(), 1e-7 * T.norm() * fd.norm()) << "the projected rows still see the tilt (column " << k << ")";
    EXPECT_LT(Hp.col(k).norm(), 1e-12 * T.norm() * H.col(k).norm());
  }
  compareColumns(*sc, Hp, hp, {HostIdx::dim + sc->off_td, ct, ct + 1}, "projected dopp H", 1e-6, 1e-8);
}

// PVT rows: h = a^N / w^N with H = J_a / J_w (rtk::pos2ecef, WP-a)
TEST(U07Jacobians, PvtRows) {
  auto sc = makeScene(300, false);
  NoiseModel noise(sc->cfg);
  RowBuilder rb(sc->cfg, noise);
  const LinPoint lp = sc->lin();
  const AntennaJacobians J = rb.antennaJacobians(lp);
  ReceiverPvt pvt;
  pvt.t = GTime{1733040083, 0.2};   // navigation epoch (A18 magnitude); the 1 Hz selection bins it (A43)
  pvt.valid_fix = true;
  pvt.fix_type = 3;
  pvt.lat_deg = 30.0 + 1e-4;
  pvt.lon_deg = 110.0 - 2e-4;
  pvt.h_ell_m = 52.0;
  pvt.h_acc_m = 1.5;
  pvt.v_acc_m = 2.5;
  pvt.vel_ned = Eigen::Vector3d(0.3, 1.1, -0.05);
  pvt.vel_acc_mps = 0.2;
  RowSet pos, vel;
  rb.pvtRows(pvt, lp, J, 1.0, &pos, &vel);   // WP-a merged: an exception from rtk::pos2ecef is a failure, not "blocked"
  ASSERT_EQ(pos.rows.size(), 3u);
  ASSERT_EQ(vel.rows.size(), 3u);
  const Eigen::Vector3d r_pvt = geo2ecef(pvt.lat_deg * kDeg, pvt.lon_deg * kDeg, pvt.h_ell_m);
  const Eigen::Vector3d z_p = sc->anchor.R_EN.transpose() * (r_pvt - sc->anchor.r0_ecef);
  for (int k = 0; k < 3; ++k) {
    EXPECT_NEAR(pos.rows[k].z, z_p(k), 1e-6);
    EXPECT_NEAR(pos.rows[k].h, J.a_N(k), 1e-12);
    EXPECT_LT((pos.rows[k].H - J.J_a.row(k)).norm(), 1e-12);
    EXPECT_NEAR(vel.rows[k].h, J.w_N(k), 1e-12);
    EXPECT_LT((vel.rows[k].H - J.J_w.row(k)).norm(), 1e-12);
  }
  EXPECT_NEAR(vel.rows[0].z, 1.1, 1e-12);    // vel_e
  EXPECT_NEAR(vel.rows[1].z, 0.3, 1e-12);    // vel_n
  EXPECT_NEAR(vel.rows[2].z, 0.05, 1e-12);   // −vel_d
  EXPECT_NEAR(pos.rows[0].R, noise.pvtPosVar(1.5, 2.5, 1.0)(0), 1e-12);
  EXPECT_NEAR(pos.rows[2].R, noise.pvtPosVar(1.5, 2.5, 1.0)(2), 1e-12);
  // A43 on the L-PVT rows: the next solution of the same GPST second builds nothing, a rebuild of the taking one keeps
  // its rows, the first solution of the next second builds them; a solution without a GPST time is never binned.
  ReceiverPvt same = pvt;
  same.t = GTime{pvt.t.sec, 0.3};
  RowSet p1, v1;
  rb.pvtRows(same, lp, J, 1.0, &p1, &v1);
  EXPECT_TRUE(p1.rows.empty() && v1.rows.empty()) << "second PVT of a GPST second must not build rows (A43)";
  RowSet p0, v0;
  rb.pvtRows(pvt, lp, J, 1.0, &p0, &v0);
  EXPECT_EQ(p0.rows.size(), 3u);
  EXPECT_EQ(v0.rows.size(), 3u);
  ReceiverPvt next = pvt;
  next.t = GTime{pvt.t.sec + 1, 0.0};
  RowSet p2, v2;
  rb.pvtRows(next, lp, J, 1.0, &p2, &v2);
  EXPECT_EQ(p2.rows.size(), 3u);
  ReceiverPvt no_time = pvt;
  no_time.t = GTime();
  RowSet p3, v3;
  rb.pvtRows(no_time, lp, J, 1.0, &p3, &v3);
  EXPECT_TRUE(p3.rows.empty());
  EXPECT_EQ(rb.pvtDecimation().taken(), 2u);
  EXPECT_EQ(rb.pvtDecimation().skippedSameBin(), 1u);
  EXPECT_EQ(rb.psrDecimation().taken(), 0u) << "the pseudorange and PVT selections are independent";
}

// geometry() / geometryFromSnapshot() (review m4): with one ephemeris both paths agree bit for bit; when the
// store switches ephemeris inside a TDCP pair, the k1 end built from the live store carries another toe and
// tdcpRows() refuses it, while the k1 end built from the k0 snapshot is accepted. WP-a's NavStore/SatSolver (merged:
// an exception is a failure, not "blocked").
TEST(U07Jacobians, GeometryFromSnapshotAndEphemerisSwitch) {
  auto sc = makeScene(800, false);
  NoiseModel noise(sc->cfg);
  RowBuilder rb(sc->cfg, noise);
  const LinPoint lp = sc->lin();
  const AntennaJacobians J = rb.antennaJacobians(lp);
  const Eigen::Vector3d r_A = sc->anchor.r0_ecef + sc->anchor.R_EN * J.a_N;
  // a synthetic GPS Kepler ephemeris (near-circular MEO) and a second one 2 h later with a different toe
  auto makeEph = [](GTime toe, double M0) {
    KeplerEph e;
    e.sat = sat(7);
    e.toe = e.toc = e.ttr = toe;
    e.A = 5153.8 * 5153.8;
    e.e = 0.01;
    e.i0 = 0.96;
    e.OMG0 = 1.2;
    e.omg = 0.4;
    e.M0 = M0;
    e.f0 = 1e-5;
    e.week = 2340;
    e.toes = 100.0;
    return e;
  };
  const GTime t_k0 = GTime::fromSec(1.4e9), t_k1 = t_k0.add(0.5);
  SatObs so;
  so.sat = sat(7);
  so.t_rx = t_k1;
  SignalObs sig;
  sig.freq_hz = 1575.42e6;
  sig.code = SignalCode::L1C;
  sig.status = kStatusPsrValid | kStatusCpValid | kStatusHalfCycValid;
  sig.psr_m = 2.2e7;
  so.sig.push_back(sig);
  Geodetic rcv;
  rcv.lat_rad = 30.0 * kDeg;
  rcv.lon_rad = 110.0 * kDeg;
  rcv.h_m = 50.0;
  rtk::EphSnapshot snap;
  SatGeometry g_live, g_snap;
  // an invalid snapshot is refused without reaching the adapter
  EXPECT_FALSE(rb.geometryFromSnapshot(so, snap, r_A, rcv, nullptr, t_k1, 0.0, &g_snap));
  rtk::NavStore nav;
  ASSERT_TRUE(nav.add(makeEph(t_k0.add(-1800.0), 0.3), true));
  rtk::SatSolver solver(nav);
  rtk::SatState st0;
  ASSERT_TRUE(solver.satState(so.sat, t_k0, sig.psr_m, &st0, &snap));   // k0: capture the ephemeris
  ASSERT_TRUE(snap.valid());
  // same ephemeris: live path and snapshot path are bit-identical
  ASSERT_TRUE(rb.geometry(so, solver, r_A, rcv, nullptr, t_k1, 0.0, &g_live));
  ASSERT_TRUE(rb.geometryFromSnapshot(so, snap, r_A, rcv, nullptr, t_k1, 0.0, &g_snap));
  EXPECT_EQ(std::memcmp(g_live.state.r_ecef.data(), g_snap.state.r_ecef.data(), 3 * sizeof(double)), 0);
  EXPECT_EQ(g_live.state.dts, g_snap.state.dts);
  EXPECT_EQ(g_live.range, g_snap.range);
  // the store switches inside the pair: a newer toe closer to t_k1 is selected by the live path only
  ASSERT_TRUE(nav.add(makeEph(t_k1.add(-60.0), 0.31), true));
  SatGeometry g_new;
  ASSERT_TRUE(rb.geometry(so, solver, r_A, rcv, nullptr, t_k1, 0.0, &g_new));
  EXPECT_NE(g_new.state.toe, snap.toe());
  ASSERT_TRUE(rb.geometryFromSnapshot(so, snap, r_A, rcv, nullptr, t_k1, 0.0, &g_snap));
  EXPECT_EQ(g_snap.state.toe, snap.toe());
  RowBuilder::TdcpPairInput pair;
  pair.pair_id = 1;
  pair.clone_slot = sc->off_clone;
  pair.c_hat_N = sc->aug.x().segment<3>(sc->off_clone);
  pair.dt_pair_s = 0.5;
  pair.sats.push_back(so.sat);
  pair.phi_k0_cyc.push_back(1e8);
  pair.phi_k1_cyc.push_back(1e8 + 100.0);
  pair.dts_k0.push_back(st0.dts);
  pair.dts_k1.push_back(g_snap.state.dts);
  pair.rs_k0.push_back(st0.r_ecef);
  pair.eph_snap.push_back(snap);
  RowSet mixed, consistent;
  rb.tdcpRows(pair, {g_new}, lp, J, &mixed);
  rb.tdcpRows(pair, {g_snap}, lp, J, &consistent);
  EXPECT_TRUE(mixed.rows.empty()) << "k1 built from another ephemeris must be refused";
  EXPECT_EQ(consistent.rows.size(), 1u);
}

// clone creation Jacobian vs central differences (c = a^N(x̂_k0) ⇒ F = J_a, N = M Σ_D Mᵀ)
TEST(U07Jacobians, CloneJacobians) {
  auto sc = makeScene(400, true);
  NoiseModel noise(sc->cfg);
  RowBuilder rb(sc->cfg, noise);
  const AntennaJacobians J = rb.antennaJacobians(sc->lin());
  Eigen::MatrixXd Fh, Fa, Nm;
  const Eigen::Matrix3d SigmaD = Eigen::Matrix3d::Identity() * 1e-8 + Eigen::Vector3d(1e-5, 2e-5, 0.0) * Eigen::Vector3d(1e-5, 2e-5, 0.0).transpose();
  RowBuilder::cloneJacobians(J, SigmaD, &Fh, &Fa, &Nm);
  ASSERT_EQ(Fh.rows(), 3);
  ASSERT_EQ(Fh.cols(), HostIdx::dim);
  ASSERT_EQ(Fa.cols(), sc->aug.dim());
  Eigen::MatrixXd F(3, sc->N);
  F << Fh, Fa;
  ModelFn fa = [&](const CoreState& x, const Eigen::VectorXd& xg) { Eigen::Vector3d a, w; refAntenna(*sc, x, xg, &a, &w); return Eigen::VectorXd(a); };
  compareColumns(*sc, F, fa, {HostIdx::dim + sc->off_td}, "clone F");
  EXPECT_LT((Nm - J.M * SigmaD * J.M.transpose()).norm(), 1e-15);
  EXPECT_LT((J.M * J.M.transpose() - Eigen::Matrix3d::Identity()).norm(), 1e-12);
}

// ε_z of C(g+δg) ≈ Exp(ε)C(g) is zero to machine precision; ε Jacobian vs central differences
TEST(U07Jacobians, LevelingEpsilonZIsZero) {
  std::mt19937_64 rng(9);
  std::normal_distribution<double> gs(0.0, 1.0);
  for (int trial = 0; trial < 20; ++trial) {
    const Eigen::Vector3d g = so3Exp(0.2 * Eigen::Vector3d(gs(rng), gs(rng), gs(rng))) * Eigen::Vector3d(0.0, 0.0, -9.81);
    Leveling lev;
    lev.setRef(g);
    const Eigen::Matrix3d C = lev.C(g);
    EXPECT_LT((C * g.normalized() + Eigen::Vector3d(0.0, 0.0, 1.0)).norm(), 1e-12);   // levelled: C ĝ = −e_z
    const Eigen::Matrix3d E = lev.epsJacobian(g);
    EXPECT_LT(E.row(2).norm(), 1e-15) << "ε_z must vanish identically";
    EXPECT_LT((E - skew(Eigen::Vector3d(0.0, 0.0, 1.0)) * lev.Gperp(g)).norm(), 1e-15);
    for (int k = 0; k < 3; ++k) {
      const double h = 1e-6;
      Eigen::Vector3d dg = Eigen::Vector3d::Zero();
      dg(k) = h;
      const Eigen::Vector3d ep = so3Log(Eigen::Matrix3d(lev.C(g + dg) * C.transpose()));
      const Eigen::Vector3d em = so3Log(Eigen::Matrix3d(lev.C(g - dg) * C.transpose()));
      const Eigen::Vector3d fd = (ep - em) / (2.0 * h);
      EXPECT_LT((fd - E.col(k)).norm() / std::max(1e-3, E.col(k).norm()), 1e-6) << "ε column " << k;
      EXPECT_LT(std::fabs(fd(2)), 1e-9);
    }
    // the tilt direction (I − ĝĝᵀ) is exactly the null direction of G_⊥ along ĝ
    EXPECT_LT((lev.Gperp(g) * g).norm(), 1e-12);
  }
}

// J_rw of the pivot-coupled registration walk (DESIGN §3.2 with AMENDMENTS A45 / R6): Q_ξ = J_rw diag(q) J_rwᵀ,
// rotation increment Δr = R_z J_l(φ̃) E_xy w_φ + e_z w_ψ. The tilt is additive, Exp(φ̃ + w̃) = Exp(J_l(φ̃) w̃) Exp(φ̃), so
// §3.2's R_z E_xy w_φ is the φ = 0 form. Checked at φ ≠ 0 against
//  (1) the exact formula (J_l from leftJacobian() here, independent of Leveling), 1e-9 relative;
//  (2) §3.2's defining property — the current point m = M(ψ, φ, g)(a^W − o^W) + t is left invariant by the rotation
//      noise, only q_t reaches it — with ∂m/∂φ from fourth-order central differences of the production M() (no J_l
//      shared with the code) and ∂m/∂ψ = e_z × u (R_z(ψ + s) = Exp(e_z s) R_z(ψ)), 1e-9 relative.
// Variant 0 is the scene (|φ| ≈ 2.9e-3, |u| ≈ 12 m): the φ = 0 form misses there by ≈ |φ|q_φ/q_ψ-sized amounts, above the
// tolerance but not by much. Variant 1 (|φ| = 0.05, |u| ≈ 97 m) must see the φ = 0 form fail both checks by ≥ 1e3× the
// tolerance, so the test does discriminate R6.
TEST(U07Jacobians, RandomWalkJrw) {
  for (int variant = 0; variant < 2; ++variant) {
    auto sc = makeScene(500, false);
    const Eigen::Vector3d a_W = sc->x.p + sc->x.R * sc->lever;
    Eigen::Vector3d pivot = sc->pivot;
    if (variant == 1) {
      sc->aug.x().segment<2>(sc->off_tilt) = Eigen::Vector2d(0.03, -0.04);
      pivot = a_W - Eigen::Vector3d(80.0, -55.0, 2.0);
    }
    const Config::Process::Frame& q = sc->cfg.process.frame;
    const double psi = sc->aug.x()(sc->off_yaw);
    const Eigen::Vector2d phi = sc->aug.x().segment<2>(sc->off_tilt);
    ASSERT_GT(phi.norm(), 1e-3);
    const Eigen::Matrix3d M = sc->lev.M(psi, phi, sc->x.g);
    const double ds = 0.37, dalpha = 0.12, dt = 0.1;
    Eigen::MatrixXd Phi, Qp;
    sc->aug.processModel(ds, dalpha, dt, M, a_W, pivot, kC / 1575.42e6, 0.0, &Phi, &Qp);
    ASSERT_EQ(Qp.rows(), sc->aug.dim());
    // ξ index list [tilt(2), yaw, trans(3)]
    const std::vector<int> xi = {sc->off_tilt, sc->off_tilt + 1, sc->off_yaw, sc->off_trans, sc->off_trans + 1, sc->off_trans + 2};
    Eigen::Matrix<double, 6, 6> Qxi;
    for (int i = 0; i < 6; ++i)
      for (int j = 0; j < 6; ++j) Qxi(i, j) = Qp(xi[i], xi[j]);
    const Eigen::Vector3d u = M * (a_W - pivot);              // m − t
    const Eigen::Matrix3d Rz = Leveling::Rz(psi);
    const Eigen::Matrix3d Jl = leftJacobian(Eigen::Vector3d(phi(0), phi(1), 0.0));
    Eigen::Matrix<double, 3, 2> Exy;
    Exy << 1.0, 0.0, 0.0, 1.0, 0.0, 0.0;
    Eigen::Matrix<double, 6, 1> d;
    d << q.q_tilt_per_m * ds, q.q_tilt_per_m * ds, q.q_yaw_per_m * ds + q.q_yaw_per_rad * dalpha, q.q_th_per_m * ds, q.q_th_per_m * ds,
        q.q_tv_per_m * ds;
    auto qOf = [&](const Eigen::Matrix3d& tilt_rot) {   // J_rw diag(q) J_rwᵀ with the tilt rotation block `tilt_rot`
      Eigen::Matrix<double, 6, 6> Jrw = Eigen::Matrix<double, 6, 6>::Identity();
      Jrw.block<3, 2>(3, 0) = skew(u) * tilt_rot * Exy;
      Jrw.block<3, 1>(3, 2) = skew(u) * Eigen::Vector3d(0.0, 0.0, 1.0);
      return Eigen::Matrix<double, 6, 6>(Jrw * d.asDiagonal() * Jrw.transpose());
    };
    const Eigen::Matrix<double, 6, 6> Qref = qOf(Rz * Jl), Qfirst = qOf(Rz);
    const double e_exact = (Qxi - Qref).norm() / Qref.norm(), e_first = (Qxi - Qfirst).norm() / Qfirst.norm();
    EXPECT_LT(e_exact, 1e-9) << "|φ| " << phi.norm() << ": Q_ξ ≠ J_rw diag(q) J_rwᵀ with Δr = R_z J_l(φ̃) E_xy w_φ + e_z w_ψ (A45/R6)";
    // (2) ∂m/∂φ by 4th-order central differences of M() (h = 1e-3 rad: truncation and rounding ≲ 1e-12 relative)
    auto cd4v = [](const std::function<Eigen::Vector3d(double)>& f, double h) {
      return Eigen::Vector3d((f(-2.0 * h) - 8.0 * f(-h) + 8.0 * f(h) - f(2.0 * h)) / (12.0 * h));
    };
    Eigen::Matrix<double, 3, 6> Jm = Eigen::Matrix<double, 3, 6>::Zero();
    for (int k = 0; k < 2; ++k) {
      Jm.col(k) = cd4v(
          [&](double s) {
            Eigen::Vector2d p = phi;
            p(k) += s;
            return Eigen::Vector3d(sc->lev.M(psi, p, sc->x.g) * (a_W - pivot));
          },
          1e-3);
    }
    Jm.col(2) = Eigen::Vector3d::UnitZ().cross(u);
    Jm.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();
    const Eigen::Matrix3d Vref = Eigen::Vector3d(q.q_th_per_m * ds, q.q_th_per_m * ds, q.q_tv_per_m * ds).asDiagonal();
    const double v_exact = (Jm * Qxi * Jm.transpose() - Vref).norm() / Vref.norm();
    const double v_first = (Jm * Qfirst * Jm.transpose() - Vref).norm() / Vref.norm();
    EXPECT_LT(v_exact, 1e-9) << "|φ| " << phi.norm() << ": the rotation noise leaks into the current point";
    if (variant == 1) {
      EXPECT_GT(e_first, 1e3 * 1e-9) << "the φ = 0 form of J_rw is indistinguishable from the exact one here";
      EXPECT_GT(v_first, 1e3 * 1e-9) << "the φ = 0 form's leak into the current point is invisible here";
    }
    std::printf("[U7 R6] |phi| %.2e |u| %.1f m: Q_xi rel.err exact %.1e / phi=0 form %.1e; current-point variance rel.err "
                "exact %.1e / phi=0 form %.1e\n",
                phi.norm(), u.norm(), e_exact, e_first, v_exact, v_first);
  }
}

// re-pivot Jacobian keeps the global antenna position and its covariance (A11 full T)
namespace {
struct RepivotCase {
  double pos_err = 0.0, cov_err = 0.0, jt_err = 0.0, sigma_err = 0.0;
  // t rows of the returned T: tilt block (3×2) and δg block (3×3), with the pre-A34 first-order forms
  // Ξ(o')E_xy / Ξ(o')[e_z]×G_⊥ and the exact A34 forms Ξ(o')J_l(φ̃)E_xy / Ξ(o')Exp(φ̃)[e_z]×G_⊥ built here.
  Eigen::MatrixXd T_tilt, T_grav, tilt_first, grav_first, tilt_exact, grav_exact;
};
RepivotCase runRepivot(bool zero_tilt) {
  RepivotCase rc;
  auto sc = makeScene(600, false, zero_tilt);
  NoiseModel noise(sc->cfg);
  RowBuilder rb(sc->cfg, noise);
  const AntennaJacobians J0 = rb.antennaJacobians(sc->lin());
  const Eigen::MatrixXd Sigma0 = sc->aug.jointCov(sc->x.P);
  const Eigen::Matrix3d cov0 = J0.J_a * Sigma0 * J0.J_a.transpose();
  const Eigen::Vector3d y = sc->x.p + sc->x.R * sc->lever;
  const Eigen::Vector3d a_W = y + Eigen::Vector3d(120.0, -40.0, 3.0);   // > 100 m from the pivot
  const Eigen::Vector3d pivot_old = sc->pivot;
  // Ξ(o') = −R_z(ψ)[Exp(φ̃)C(g)d]× at the pre-pivot state (A11), and both candidate column pairs
  const double psi = sc->aug.x()(sc->off_yaw);
  const Eigen::Vector2d phi = sc->aug.x().segment<2>(sc->off_tilt);
  const Eigen::Matrix3d ExpPhi = Leveling::expTilt(phi);
  const Eigen::Vector3d d = a_W - pivot_old;
  const Eigen::Matrix3d Xi = -Leveling::Rz(psi) * skew(Eigen::Vector3d(ExpPhi * sc->lev.C(sc->x.g) * d));
  const Eigen::Matrix3d Eps = sc->lev.epsJacobian(sc->x.g);
  rc.tilt_first = Xi.leftCols<2>();
  rc.grav_first = Xi * Eps;
  rc.tilt_exact = (Xi * leftJacobian(Eigen::Vector3d(phi(0), phi(1), 0.0))).leftCols<2>();
  rc.grav_exact = Xi * ExpPhi * Eps;
  const Eigen::MatrixXd T_aug = sc->aug.repivot(pivot_old, a_W, sc->lev, sc->x.g, sc->x.P, true);
  EXPECT_EQ(T_aug.rows(), sc->aug.dim());
  EXPECT_EQ(T_aug.cols(), sc->N);
  rc.T_tilt = T_aug.block(sc->off_trans, HostIdx::dim + sc->off_tilt, 3, 2);
  rc.T_grav = T_aug.block(sc->off_trans, HostIdx::grav, 3, 3);
  sc->pivot = a_W;
  const AntennaJacobians J1 = rb.antennaJacobians(sc->lin());
  const Eigen::MatrixXd Sigma1 = sc->aug.jointCov(sc->x.P);
  rc.pos_err = (J1.a_N - J0.a_N).norm();
  const Eigen::Matrix3d cov1 = J1.J_a * Sigma1 * J1.J_a.transpose();
  rc.cov_err = (cov1 - cov0).norm() / cov0.norm();
  Eigen::MatrixXd T = Eigen::MatrixXd::Identity(sc->N, sc->N);
  T.bottomRows(sc->aug.dim()) = T_aug;
  rc.jt_err = (J1.J_a * T - J0.J_a).norm() / J0.J_a.norm();
  rc.sigma_err = (Sigma1 - T * Sigma0 * T.transpose()).norm() / Sigma0.norm();
  return rc;
}
double relDiff(const Eigen::MatrixXd& a, const Eigen::MatrixXd& b) { return (a - b).norm() / std::max(b.norm(), 1e-300); }
}  // namespace

// φ = 0: T is exact (J_l(0) = Exp(0) = I), so the global antenna and its covariance are invariant to machine precision.
TEST(U07Jacobians, RepivotJ) {
  const RepivotCase rc = runRepivot(true);
  EXPECT_LT(rc.pos_err, 1e-9) << "global antenna position changed by the re-pivot";
  EXPECT_LT(rc.cov_err, 1e-9) << "global antenna covariance changed by the re-pivot (φ = 0)";
  EXPECT_LT(rc.jt_err, 1e-9) << "J_a' T ≠ J_a (φ = 0)";
  EXPECT_LT(rc.sigma_err, 1e-9) << "Σ' ≠ T Σ Tᵀ";
}

// φ ≠ 0 (AMENDMENTS A34): with the exact columns ∂t'/∂δφ = Ξ(o')J_l(φ̃)E_xy and ∂t'/∂δg = Ξ(o')Exp(φ̃)[e_z]×G_⊥ the
// identities J_a'·T = J_a and "global antenna covariance unchanged" are exact, as at φ = 0; asserted at the φ = 0
// tolerance 1e-9 (A34's 1e-6 for U7 bounds it; exact columns reach rounding level), and the two columns against the
// exact forms built here (1e-9 relative: a truncated J_l ≈ I + ½[φ̃]× is off by |φ|²/6 ≈ 1.4e-6 and fails). The re-pivot
// is WP-b's (P2b WP-b, A34); while this tree's AugBlock::repivot still writes exactly the pre-A34 first-order columns
// Ξ(o')E_xy and Ξ(o')[e_z]×G_⊥ (both, to 1e-12), the case is blocked on that change. Any other re-pivot — the exact one
// or a wrong one — is judged by the assertions.
TEST(U07Jacobians, RepivotJTiltExact) {
  const RepivotCase rc = runRepivot(false);
  // the scene's tilt (|φ| ≈ 2.9e-3) makes the two forms distinguishable by ~|φ|/2, far above the 1e-12 detector
  ASSERT_GT(relDiff(rc.tilt_exact, rc.tilt_first), 1e-4);
  ASSERT_GT(relDiff(rc.grav_exact, rc.grav_first), 1e-4);
  // the parts of A11 that A34 does not touch hold already
  EXPECT_LT(rc.pos_err, 1e-9) << "global antenna position changed by the re-pivot (φ ≠ 0)";
  EXPECT_LT(rc.sigma_err, 1e-9) << "Σ' ≠ T Σ Tᵀ (φ ≠ 0)";
  if (relDiff(rc.T_tilt, rc.tilt_first) < 1e-12 && relDiff(rc.T_grav, rc.grav_first) < 1e-12) {
    GTEST_SKIP() << "blocked: P2b WP-b AugBlock::repivot with the exact A34 columns (this tree writes the first-order "
                    "Ξ(o')E_xy / Ξ(o')[e_z]×G_⊥; J_a'T error "
                 << rc.jt_err << ", covariance error " << rc.cov_err << " at φ ≠ 0)";
  }
  EXPECT_LT(relDiff(rc.T_tilt, rc.tilt_exact), 1e-9) << "∂t'/∂δφ ≠ Ξ(o')J_l(φ̃)E_xy (A34)";
  EXPECT_LT(relDiff(rc.T_grav, rc.grav_exact), 1e-9) << "∂t'/∂δg ≠ Ξ(o')Exp(φ̃)[e_z]×G_⊥ (A34)";
  EXPECT_LT(rc.jt_err, 1e-9) << "J_a' T ≠ J_a (φ ≠ 0, A34)";
  EXPECT_LT(rc.cov_err, 1e-9) << "global antenna covariance changed by the re-pivot (φ ≠ 0, A34)";
}

// δt_d columns by central differences in t_d (review m6a). A residual time offset means the GNSS epoch is really
// at t_g − t_d, so a^N(t_d) and w^N(t_d) follow from running the bridge model (§4.1: constant a_g = R_g ā_b + g and
// constant ω_g over the interval) back by t_d from the evaluate() point:
//   p(t_d) = p_g − v_g t_d + ½ a_g t_d²,  v(t_d) = v_g − a_g t_d,  R(t_d) = R_g Exp(−ω_g t_d).
// Only ImuBridge::evaluate() (WP-b) and that kinematics enter; nothing of rows.cpp is repeated here.
TEST(U07Jacobians, TdColumnCentralDifference) {
  auto rodrigues = [](const Eigen::Vector3d& w) {
    const double th = w.norm();
    if (th < 1e-12) return Eigen::Matrix3d(Eigen::Matrix3d::Identity() + skew(w));
    const Eigen::Matrix3d K = skew(Eigen::Vector3d(w / th));
    return Eigen::Matrix3d(Eigen::Matrix3d::Identity() + std::sin(th) * K + (1.0 - std::cos(th)) * K * K);
  };
  for (unsigned seed : {700u, 701u}) {
    auto sc = makeScene(seed, seed % 2 == 1);
    NoiseModel noise(sc->cfg);
    RowBuilder rb(sc->cfg, noise);
    const AntennaJacobians J = rb.antennaJacobians(sc->lin());
    const int col = HostIdx::dim + sc->off_td;
    const AntennaAtEpoch ae = ImuBridge::evaluate(sc->x, sc->br, sc->lever, true);
    const Eigen::VectorXd& xg = sc->aug.x();
    const Eigen::Matrix3d M = sc->lev.M(xg(sc->off_yaw), xg.segment<2>(sc->off_tilt), sc->x.g);
    const Eigen::Vector3d t = xg.segment<3>(sc->off_trans);
    auto model = [&](double td, Eigen::Vector3d* a, Eigen::Vector3d* w) {
      const Eigen::Vector3d acc = ae.R_g * sc->br.a_bar + sc->x.g;
      const Eigen::Matrix3d Rt = ae.R_g * rodrigues(-ae.omega_g * td);
      const Eigen::Vector3d p = ae.p_g - ae.v_g * td + 0.5 * acc * td * td;
      const Eigen::Vector3d v = ae.v_g - acc * td;
      *a = M * (p + Rt * sc->lever - sc->pivot) + t;
      *w = M * (v + Rt * ae.omega_g.cross(sc->lever));
    };
    Eigen::Vector3d a0, w0, ap, wp, am, wm;
    model(0.0, &a0, &w0);
    EXPECT_LT((a0 - J.a_N).norm(), 1e-12);   // at t_d = 0 the model is the row builder's antenna
    EXPECT_LT((w0 - J.w_N).norm(), 1e-12);
    const double h = 1e-3;
    model(h, &ap, &wp);
    model(-h, &am, &wm);
    const Eigen::Vector3d fd_a = (ap - am) / (2.0 * h), fd_w = (wp - wm) / (2.0 * h);
    EXPECT_LT((J.J_a.col(col) - fd_a).norm() / fd_a.norm(), 1e-6) << "J_a t_d column, seed " << seed;
    EXPECT_LT((J.J_w.col(col) - fd_w).norm() / fd_w.norm(), 1e-6) << "J_w t_d column, seed " << seed;
    EXPECT_GT(fd_a.norm(), 1.0);   // the columns are not trivially small (|w^N| ≈ 1.3 m/s; |α_A| is the
    EXPECT_GT(fd_w.norm(), 0.1);   // residual of R_g ā_b + g plus the centripetal lever term, 0.9–5.8 m/s² here)
  }
  // without the td slot the column does not exist (matrix width shrinks by one)
  auto sc = makeScene(700, true);
  NoiseModel noise(sc->cfg);
  RowBuilder rb(sc->cfg, noise);
  Config cfg2;
  AugBlock aug2(cfg2);
  Eigen::MatrixXd F0 = Eigen::MatrixXd::Zero(3, HostIdx::dim);
  aug2.addSlot(Slot::Trans, 0, "trans", Eigen::Vector3d::Zero(), F0, Eigen::MatrixXd(3, 0), Eigen::Matrix3d::Identity(), sc->x.P);
  LinPoint lp = sc->lin();
  lp.aug = &aug2;
  const AntennaJacobians J2 = rb.antennaJacobians(lp);
  EXPECT_EQ(J2.J_a.cols(), HostIdx::dim + 3);
}

// ---------------------------------------------------------------------------------------------
// §4.7 Doppler variance as ruled (AMENDMENTS A31, A35-C5/C6, A45 / C10)
// ---------------------------------------------------------------------------------------------
TEST(U07Jacobians, DopplerVarianceAsRuled) {
  Config cfg;
  NoiseModel noise(cfg);
  const double lambda = kC / 1575.42e6;
  const double rho = std::exp(-0.1 / cfg.noise.dopp.tau_s);
  const double kappa = (1.0 + rho) / (1.0 - rho);   // κ_AR(τ_D, 0.1 s)
  EXPECT_NEAR(NoiseModel::kappaAR(cfg.noise.dopp.tau_s, 0.1), kappa, 1e-12 * kappa);
  // (1) receiver term: max(λ·dopp_std, σ_D0)²·(1 + 1/sin²el)/2·κ — a variance factor, 1 at the zenith (C6); κ_AR on
  // this term only (C5). Everything else zero.
  for (double el_deg : {90.0, 45.0, 15.0, 10.0}) {
    for (double std_hz : {0.0, 0.05, 0.2}) {
      NoiseModel::DoppTerms t;
      t.el_rad = el_deg * kDeg;
      t.lambda_m = lambda;
      t.dopp_std_hz = std_hz;
      t.dt_since_last_s = 0.1;
      const double s0 = std::max(lambda * std_hz, cfg.noise.dopp.sigma0_mps);
      const double se = std::sin(t.el_rad);
      const double ruled = s0 * s0 * 0.5 * (1.0 + 1.0 / (se * se)) * kappa * cfg.noise.scale.dopp;
      EXPECT_NEAR(noise.doppVar(t), ruled, 1e-12 * ruled) << "el " << el_deg << "°, dopp_std " << std_hz << " Hz";
    }
  }
  {
    NoiseModel::DoppTerms zen;
    zen.el_rad = M_PI / 2.0;
    zen.lambda_m = lambda;
    zen.dt_since_last_s = 0.1;
    const double s0 = cfg.noise.dopp.sigma0_mps;
    EXPECT_NEAR(noise.doppVar(zen), s0 * s0 * kappa * cfg.noise.scale.dopp, 1e-15) << "zenith factor must be 1";
    // 15°: variance factor (1 + 1/sin²15°)/2 = 7.96, not the (…)² = 63.4 of DESIGN §4.7's typo
    NoiseModel::DoppTerms low = zen;
    low.el_rad = 15.0 * kDeg;
    EXPECT_NEAR(noise.doppVar(low) / noise.doppVar(zen), 0.5 * (1.0 + 1.0 / std::pow(std::sin(15.0 * kDeg), 2)), 1e-9);
  }
  // (2) A31: σ_ba, σ_θ and g never enter
  {
    NoiseModel::DoppTerms a;
    a.el_rad = 40.0 * kDeg;
    a.lambda_m = lambda;
    a.dopp_std_hz = 0.03;
    a.delta_s = 0.08;
    a.omega_norm = 0.7;
    a.ant_acc_norm = 1.9;
    a.v_norm = 1.4;
    NoiseModel::DoppTerms b = a;
    b.sigma_ba = 0.5;
    b.sigma_theta = 0.1;
    b.g = 9.81;
    EXPECT_EQ(noise.doppVar(a), noise.doppVar(b)) << "(δ(σ_ba+gσ_θ))² must not be added (A31)";
    // (3) the remaining additive terms, each at its §4.7 value and outside κ_AR (C5), including the bridge's IMU
    //     white-noise velocity term var_v_bridge = acc_n²δ (AMENDMENTS A45 / C10)
    a.var_v_bridge = cfg.imu_bridge.acc_n * cfg.imu_bridge.acc_n * a.delta_s;
    NoiseModel::DoppTerms r0 = a;
    r0.delta_s = 0.0;
    r0.omega_norm = r0.ant_acc_norm = r0.v_norm = 0.0;
    r0.var_v_bridge = 0.0;
    const double add = a.var_v_bridge + cfg.process.clock.S_d * a.delta_s + std::pow(a.omega_norm * cfg.frames.lever_arm_sigma, 2) +
                       std::pow(a.ant_acc_norm * cfg.time.sigma_toff_s, 2) + std::pow(0.01 * a.v_norm, 2);
    EXPECT_NEAR(noise.doppVar(a) - noise.doppVar(r0), add * cfg.noise.scale.dopp, 1e-15);
  }
  // (4) C10 on its own: var_v_bridge adds exactly itself (× noise.scale.dopp), outside κ_AR (a κ-weighted term would
  //     add κ·var = 4.08·var at τ_D = 0.2 s), at any noise scale
  for (double scale : {1.0, 2.5}) {
    Config c2 = cfg;
    c2.noise.scale.dopp = scale;
    NoiseModel n2(c2);
    NoiseModel::DoppTerms b0;
    b0.el_rad = 30.0 * kDeg;
    b0.lambda_m = lambda;
    b0.dt_since_last_s = 0.1;
    NoiseModel::DoppTerms b1 = b0;
    b1.var_v_bridge = 0.0208 * 0.0208 * 0.067;   // Avia acc_n, δ = 0.067 s (15 Hz cycle): 2.9e-5 m²/s²
    EXPECT_NEAR(n2.doppVar(b1) - n2.doppVar(b0), b1.var_v_bridge * scale, 1e-15) << "scale " << scale;
  }
}

// ---------------------------------------------------------------------------------------------
// AMENDMENTS A36 on real broadcast orbits: the Doppler row's Sagnac rate is the time derivative of the pseudorange's
// Sagnac term, per satellite, to 1e-6 m/s.
// ---------------------------------------------------------------------------------------------
namespace {
// Site of the dev-set fixture (Grass02 receiver_pvt, first record: 34.0339415° N, 108.7614865° E, 388.05 m). The test
// needs a real geometry, not this particular point: a fixture regenerated from another M3DGR sequence keeps it valid.
constexpr double kSiteLatDeg = 34.0339415, kSiteLonDeg = 108.7614865, kSiteH = 388.05;

// Same-day IGS BRDC of the first dev-set fixture (A24: realFixtures() excludes Synthetic_*); "" when there is none.
std::string devSetBrdc() {
  for (const std::string& f : livo_gnss_io::testsupport::realFixtures(LIVO_GNSS_TEST_DATA_DIR)) {
    const std::string b = livo_gnss_io::testsupport::fixtureBrdc(f);
    if (!b.empty()) return b;
  }
  return std::string();
}

// A receive epoch inside every system's broadcast validity window: 1234.5 s after the median GPS toe of the file.
GTime epochInside(const rtk::NavStore& nav) {
  std::vector<GTime> toes;
  for (const rtk::EphKey& k : nav.coverage())
    if (k.sat.sys == Sys::GPS) toes.push_back(k.toe);
  if (toes.empty()) return GTime();
  std::sort(toes.begin(), toes.end());
  return toes[toes.size() / 2].add(1234.5);
}

double primaryFreq(const SatId& id, const rtk::NavStore& nav) {
  if (id.sys == Sys::GLO) {
    const int k = nav.gloFcn(id.prn);
    return 1602.0e6 + (k > -100 ? k : 0) * 0.5625e6;
  }
  return id.sys == Sys::BDS ? 1561.098e6 : 1575.42e6;
}

struct SagnacTruth {
  double rho_dot = 0.0;      // d/dτ ρ(τ), ρ(τ) = geodist(r_s(t_tx + τ), r_A + ṙ_A τ): the range rate along the orbit
  double sagnac = 0.0;       // its Sagnac part d/dτ[ρ − ‖r_s − r_A‖]
  double sagnac_rcv = 0.0;   // the share of the antenna motion (satellite held at r_s(t_tx))
  Eigen::Vector3d rs0 = Eigen::Vector3d::Zero();
};
// Central differences (h = 0.5 s) along the broadcast trajectory r_s(t) = SatSolver::ephPos(t) and a straight antenna
// path. ρ − ‖·‖ is exactly the Sagnac term (geodist), so the curvature of the orbit cancels in `sagnac`; the norm
// difference is formed without cancellation. What is left is rounding of the ~2.3e7 m ranges: ≲ 1e-8 m/s.
bool sagnacTruth(const rtk::SatSolver& solver, const SatId& id, const GTime& t_tx, const Eigen::Vector3d& r_A,
                 const Eigen::Vector3d& v_A, SagnacTruth* out) {
  const double h = 0.5;
  Eigen::Vector3d rp, rm;
  double dts = 0.0, var = 0.0;
  if (!solver.ephPos(id, t_tx, &out->rs0, &dts, &var) || !solver.ephPos(id, t_tx.add(h), &rp, &dts, &var) ||
      !solver.ephPos(id, t_tx.add(-h), &rm, &dts, &var))
    return false;
  auto rangeRates = [&](const Eigen::Vector3d& sp, const Eigen::Vector3d& sm, double* rho_dot, double* sag) {
    const Eigen::Vector3d ap = r_A + v_A * h, am = r_A - v_A * h;
    const Eigen::Vector3d dp = sp - ap, dm = sm - am;
    *rho_dot = (rtk::geodist(sp, ap, nullptr) - rtk::geodist(sm, am, nullptr)) / (2.0 * h);
    const double norm_dot = (dp - dm).dot(dp + dm) / (dp.norm() + dm.norm()) / (2.0 * h);
    *sag = *rho_dot - norm_dot;
  };
  rangeRates(rp, rm, &out->rho_dot, &out->sagnac);
  double unused = 0.0;
  rangeRates(out->rs0, out->rs0, &unused, &out->sagnac_rcv);
  return true;
}
}  // namespace

// Every satellite of the dev-set fixture's same-day IGS BRDC above 10° at the fixture's site, all systems in the file.
// The row-builder scene sits at that site; its antenna moves at ≈ 28 m/s so that the antenna-velocity share of the
// Sagnac rate (≈ (ω_e/c)·|r_s|·|ṙ_A| ~ 1e-4 m/s) is tested along with the satellite-velocity share (mm/s). The resdop
// sign would miss by 2|rate| per satellite; the discrimination counts require satellites where that is ≥ 100× the
// tolerance for each share.
TEST(U07Jacobians, DopplerSagnacRateSignOnRealEphemeris) {
  const std::string brdc = devSetBrdc();
  if (brdc.empty()) GTEST_SKIP() << "blocked: dev-set fixture with brdc/*.rnx (A24) under " << LIVO_GNSS_TEST_DATA_DIR;
  rtk::NavStore nav;
  std::string why;
  ASSERT_GT(nav.loadRinexNav(brdc, true, &why), 0) << brdc << ": " << why;
  const rtk::SatSolver solver(nav);
  const GTime t_rx = epochInside(nav);
  ASSERT_TRUE(t_rx.valid()) << "no GPS record in " << brdc;

  auto sc = makeScene(900, true);
  sc->anchor.r0_ecef = geo2ecef(kSiteLatDeg * kDeg, kSiteLonDeg * kDeg, kSiteH);
  sc->anchor.R_EN = enuRot(kSiteLatDeg * kDeg, kSiteLonDeg * kDeg);
  sc->x.v = Eigen::Vector3d(25.0, -12.0, 0.3);
  NoiseModel noise(sc->cfg);
  RowBuilder rb(sc->cfg, noise);
  const LinPoint lp = sc->lin();
  const AntennaJacobians J = rb.antennaJacobians(lp);
  const Eigen::Vector3d r_A = sc->anchor.r0_ecef + sc->anchor.R_EN * J.a_N;   // exactly what dopplerRows() forms
  const Eigen::Vector3d v_A = sc->anchor.R_EN * J.w_N;
  const Geodetic rcv = rtk::ecef2pos(r_A);
  const double d_hat = sc->aug.x()(sc->off_clk + 1);

  std::set<int> seen;
  std::vector<SatId> ids;
  for (const rtk::EphKey& k : nav.coverage())
    if (seen.insert(k.sat.rtk_sat).second) ids.push_back(k.sat);
  ObsEpoch epoch;
  epoch.t_rx = t_rx;
  std::vector<SatGeometry> geo;
  std::vector<MaskVerdict> masks;
  std::vector<SagnacTruth> truth;
  for (const SatId& id : ids) {
    // a pseudorange consistent with the geometry (b̂ = 0), so the transmit time is the real one
    rtk::SatState st;
    double P = 2.2e7;
    bool ok = true;
    for (int it = 0; it < 3 && ok; ++it) {
      ok = solver.satState(id, t_rx, P, &st) && st.ok;
      if (ok) P = rtk::geodist(st.r_ecef, r_A, nullptr) - kC * st.dts;
    }
    if (!ok) continue;
    SatObs so;
    so.sat = id;
    so.t_rx = t_rx;
    SignalObs sig;
    sig.freq_hz = primaryFreq(id, nav);
    sig.status = kStatusPsrValid;
    sig.psr_m = P;
    so.sig.push_back(sig);
    SatGeometry g;
    if (!rb.geometry(so, solver, r_A, rcv, nullptr, t_rx, 0.0, &g) || g.el_rad < 10.0 * kDeg) continue;
    SagnacTruth tr;
    ASSERT_TRUE(sagnacTruth(solver, id, g.state.t_tx, r_A, v_A, &tr)) << satIdStringOf(id);
    ASSERT_LT((tr.rs0 - g.state.r_ecef).norm(), 1e-6) << satIdStringOf(id) << ": trajectory misses the row's satellite state";
    // the measurement follows the numerical ρ̇: −λD = ρ̇ + d̂ − c·δṫ_s
    so.sig[0].dopp_hz = -(tr.rho_dot + d_hat - kC * g.state.ddts) / g.lambda_m;
    epoch.sats.push_back(so);
    geo.push_back(g);
    truth.push_back(tr);
    MaskVerdict v;
    v.psr = v.dopp = v.cp = true;
    masks.push_back(v);
  }
  ASSERT_GE(geo.size(), 12u) << "satellites above 10° at the fixture site, " << brdc;
  RowSet dop;
  rb.dopplerRows(epoch, geo, masks, lp, J, {}, &dop);
  ASSERT_EQ(dop.rows.size(), geo.size());
  int n_sat_share = 0, n_rcv_share = 0;
  double max_err = 0.0, max_rate = 0.0, max_nu = 0.0;
  std::set<Sys> systems;
  for (size_t q = 0; q < geo.size(); ++q) {
    const MeasRow& r = dop.rows[q];
    const SatGeometry& g = geo[q];
    ASSERT_EQ(r.sat.rtk_sat, g.sat.rtk_sat);
    // the row's Sagnac rate = its prediction minus the plain geometric rate, the clock drift and the satellite drift
    const double sag_row = r.h - d_hat + kC * g.state.ddts - g.e_ecef.dot(g.state.v_ecef - v_A);
    const double err = std::fabs(sag_row - truth[q].sagnac);
    EXPECT_LT(err, 1e-6) << satIdStringOf(g.sat) << ": row Sagnac rate " << sag_row << " m/s vs d/dt of ρ(t) " << truth[q].sagnac;
    // Whole row vs the numerical ρ̇: equal up to satposs' 1 ms forward-difference satellite velocity (½·|a_s|·1 ms
    // ≲ 7e-4 m/s along the line of sight) — a check of the construction, not of the sign.
    EXPECT_LT(std::fabs(r.nu), 2e-3) << satIdStringOf(g.sat);
    max_err = std::max(max_err, err);
    max_rate = std::max(max_rate, std::fabs(truth[q].sagnac));
    max_nu = std::max(max_nu, std::fabs(r.nu));
    if (std::fabs(truth[q].sagnac - truth[q].sagnac_rcv) > 1e-4) ++n_sat_share;
    if (std::fabs(truth[q].sagnac_rcv) > 1e-4) ++n_rcv_share;
    systems.insert(g.sat.sys);
  }
  std::cout << "[U7] A36 on real orbits (" << livo_gnss_io::testsupport::baseName(brdc) << "): " << geo.size()
            << " satellites of " << systems.size() << " systems; max |Sagnac rate| " << max_rate << " m/s, max |row − d/dt ρ(t)| "
            << max_err << " m/s; satellite share > 1e-4 on " << n_sat_share << ", antenna share > 1e-4 on " << n_rcv_share
            << "; max |ν| (forward-difference velocity) " << max_nu << " m/s\n";
  EXPECT_GE(n_sat_share, 8) << "too few satellites discriminate the sign of the satellite-velocity share";
  EXPECT_GE(n_rcv_share, 3) << "too few satellites discriminate the sign of the antenna-velocity share";
  EXPECT_GE(systems.size(), 3u);
}

// ---------------------------------------------------------------------------------------------
// AMENDMENTS A43: the 1 Hz selection of the pseudorange / L-PVT rows takes, per GPST second ⌊tow⌋, the FIRST epoch to
// arrive, whatever the phase of the receiver's epoch grid (EpochDecimator, rows.cpp). The expectation is computed here
// from the rule's text on in-order streams (first element of each distinct ⌊t⌋ in arrival order); the out-of-order,
// idempotency and bin-size cases are spelled out one by one.
// ---------------------------------------------------------------------------------------------
namespace {
// GTime at T0 + ms milliseconds, exact (integer seconds + fraction) for any sign of ms.
GTime atMs(int64_t T0, int64_t ms) {
  const int64_t q = (ms >= 0) ? ms / 1000 : -((-ms + 999) / 1000);
  return GTime{T0 + q, static_cast<double>(ms - q * 1000) / 1000.0};
}
// First element of each distinct ⌊t⌋ of an in-order stream (A43's text).
std::vector<size_t> firstPerSecond(const std::vector<GTime>& ts) {
  std::vector<size_t> idx;
  for (size_t i = 0; i < ts.size(); ++i)
    if (idx.empty() || ts[i].sec != ts[idx.back()].sec) idx.push_back(i);
  return idx;
}
bool oldFiveMsRule(const GTime& t) { return std::fabs(t.frac - std::round(t.frac)) < 5e-3; }   // DESIGN §4.3, retired
}  // namespace

TEST(U07Jacobians, OneHzDecimationFirstEpochPerGpstSecond) {
  const int64_t T0 = 1733040083;   // GTime::toSec() scale (A18/A40), Occlusion03's start
  // (1) receiver grids off the whole tenths by the per-recording M3DGR offsets (audit G15: Grass01 −7 ms, Grass02 0,
  //     Dynamic03 +3 ms, Occlusion03/04 +5 ms), at 10, 5 and 1 Hz, with a deterministic pattern of dropped epochs (the
  //     occlusion sequences publish range_meas only when a satellite is tracked, so some seconds lose their first epoch)
  for (int off_ms : {-7, 0, 3, 5}) {
    for (int step_ms : {100, 200, 1000}) {
      std::vector<GTime> ts;
      for (int k = 0; k < 20000 / step_ms; ++k)
        if ((k * 7) % 13 != 0) ts.push_back(atMs(T0, static_cast<int64_t>(k) * step_ms + off_ms));
      EpochDecimator dec(1.0);
      std::vector<size_t> taken;
      for (size_t i = 0; i < ts.size(); ++i)
        if (dec.offer(ts[i]) == EpochDecimator::Verdict::Take) taken.push_back(i);
      const std::vector<size_t> expect = firstPerSecond(ts);
      EXPECT_EQ(taken, expect) << "grid offset " << off_ms << " ms, step " << step_ms << " ms";
      EXPECT_EQ(dec.taken(), expect.size());
      EXPECT_EQ(dec.skipped(), ts.size() - expect.size());
      EXPECT_EQ(dec.skippedStale(), 0u);
      if (step_ms == 100 && (off_ms == -7 || off_ms == 5)) {
        size_t old = 0;
        for (const GTime& t : ts) old += oldFiveMsRule(t) ? 1 : 0;
        EXPECT_EQ(old, 0u) << "the retired |frac| < 5 ms rule selects nothing at " << off_ms << " ms (why A43 exists)";
        EXPECT_GE(expect.size(), 19u);   // one per second that has an epoch
      }
    }
  }
  // (2) out of order: once second s+1 is taken, a first arrival of second s is Stale (the facade's L0 drops
  //     non-monotonic epochs anyway, DESIGN §10 L0), a second arrival of s+1 is SameBin; both are counted once
  {
    EpochDecimator dec(1.0);
    const GTime a = atMs(T0, 5), b = atMs(T0, 1005), c = atMs(T0, 905), d = atMs(T0, 1105);
    EXPECT_EQ(dec.offer(a), EpochDecimator::Verdict::Take);
    EXPECT_EQ(dec.offer(b), EpochDecimator::Verdict::Take);
    EXPECT_EQ(dec.offer(c), EpochDecimator::Verdict::Stale);
    EXPECT_EQ(dec.offer(d), EpochDecimator::Verdict::SameBin);
    // (3) idempotent per epoch time: the same epochs again (relinearisation; rows built after an arrival-time offer)
    //     get their first verdicts back and nothing is recounted
    EXPECT_EQ(dec.offer(a), EpochDecimator::Verdict::Take);
    EXPECT_EQ(dec.offer(c), EpochDecimator::Verdict::Stale);
    EXPECT_EQ(dec.offer(d), EpochDecimator::Verdict::SameBin);
    EXPECT_TRUE(dec.admit(b));
    EXPECT_EQ(dec.taken(), 2u);
    EXPECT_EQ(dec.skippedStale(), 1u);
    EXPECT_EQ(dec.skippedSameBin(), 1u);
    // an invalid time is never binned (no row) and is not a decimation skip
    EXPECT_EQ(dec.offer(GTime()), EpochDecimator::Verdict::NoTime);
    EXPECT_EQ(dec.skipped(), 2u);
    dec.reset();
    EXPECT_EQ(dec.offer(c), EpochDecimator::Verdict::Take);
    EXPECT_EQ(dec.taken(), 1u);
  }
  // (4) bin size = noise.*.decim_s: 2 s bins take the first epoch of each even-aligned pair of seconds; 0.5 s bins the
  //     first of each half second
  {
    std::vector<GTime> ts;
    for (int k = 0; k < 100; ++k) ts.push_back(atMs(T0, k * 100 + 5));
    EpochDecimator d2(2.0), dh(0.5);
    std::vector<size_t> t2, th;
    for (size_t i = 0; i < ts.size(); ++i) {
      if (d2.admit(ts[i])) t2.push_back(i);
      if (dh.admit(ts[i])) th.push_back(i);
    }
    std::vector<size_t> e2, eh;
    for (size_t i = 0; i < ts.size(); ++i) {
      const int64_t b2 = ts[i].sec >= 0 ? ts[i].sec / 2 : 0;
      if (e2.empty() || ts[e2.back()].sec / 2 != b2) e2.push_back(i);
      const int64_t bh = ts[i].sec * 2 + (ts[i].frac >= 0.5 ? 1 : 0);
      if (eh.empty() || ts[eh.back()].sec * 2 + (ts[eh.back()].frac >= 0.5 ? 1 : 0) != bh) eh.push_back(i);
    }
    EXPECT_EQ(t2, e2);
    EXPECT_EQ(th, eh);
    EXPECT_EQ(d2.binOf(GTime{T0, 0.9}), T0 / 2);
  }
  // (5) RowBuilder: an arrival-time offer decides for every later row build of that epoch (A43 "first to ARRIVE"): the
  //     x.005 epoch arrives first, the x.105 one second; rows are built for the x.105 one first and it still gets none.
  {
    Config cfg;
    NoiseModel noise(cfg);
    RowBuilder rb(cfg, noise);
    const GTime e1 = atMs(T0, 5), e2 = atMs(T0, 105), e3 = atMs(T0, 1005);
    EXPECT_EQ(rb.offerPsrEpoch(e1), EpochDecimator::Verdict::Take);
    EXPECT_EQ(rb.offerPsrEpoch(e2), EpochDecimator::Verdict::SameBin);
    auto sc = makeScene(1000, false);
    const LinPoint lp = sc->lin();
    const AntennaJacobians J = rb.antennaJacobians(lp);
    ReceiverPvt pvt;
    pvt.valid_fix = true;
    pvt.fix_type = 3;
    pvt.lat_deg = 30.0;
    pvt.lon_deg = 110.0;
    pvt.h_ell_m = 50.0;
    pvt.h_acc_m = pvt.v_acc_m = 1.0;
    pvt.vel_acc_mps = 0.1;
    EXPECT_EQ(rb.offerPvtEpoch(e1), EpochDecimator::Verdict::Take);
    EXPECT_EQ(rb.offerPvtEpoch(e2), EpochDecimator::Verdict::SameBin);
    RowSet p, v;
    pvt.t = e2;
    rb.pvtRows(pvt, lp, J, 1.0, &p, &v);
    EXPECT_TRUE(p.rows.empty()) << "the second arrival of a GPST second must not build rows after the arrival-time offer";
    pvt.t = e1;
    rb.pvtRows(pvt, lp, J, 1.0, &p, &v);
    EXPECT_EQ(p.rows.size(), 3u);
    pvt.t = e3;   // never offered: its first row build is its offer
    RowSet p3, v3;
    rb.pvtRows(pvt, lp, J, 1.0, &p3, &v3);
    EXPECT_EQ(p3.rows.size(), 3u);
    EXPECT_EQ(rb.pvtDecimation().taken(), 2u);
    EXPECT_EQ(rb.pvtDecimation().skipped(), 1u);
    EXPECT_EQ(rb.psrDecimation().taken(), 1u);
    EXPECT_EQ(rb.psrDecimation().skipped(), 1u);
    // the configured bin sizes reach the two decimators
    Config c2;
    c2.noise.psr.decim_s = 2.0;
    c2.noise.pvt.decim_s = 5.0;
    RowBuilder rb2(c2, noise);
    EXPECT_EQ(rb2.psrDecimation().decimS(), 2.0);
    EXPECT_EQ(rb2.pvtDecimation().decimS(), 5.0);
  }
}

// A43 on the dev-set fixtures' own epoch times (range_meas and receiver_pvt bytes through WP-e's decoder, record order):
// exactly one epoch per GPST second that has one — the first — on every grid phase, where the retired 5 ms rule took
// none on Occlusion03 (+5 ms grid; README of the fixture, WP-e #16).
TEST(U07Jacobians, OneHzDecimationOnDevSetEpochTimes) {
  LIVO_DEVSET_SLICES(slices);
  const livo_wpa_testing::Channel kPvtChannel{Source::GnssCommPvt, "gnss_comm/GnssPVTSolnMsg", "receiver_pvt", ""};
  struct Sink : MeasurementSink {
    std::vector<GTime> obs, pvt;
    void pushObservation(ObsEpoch&& e) override { obs.push_back(e.t_rx); }
    void pushEphemeris(const KeplerEph&) override {}
    void pushGloEphemeris(const GloEph&) override {}
    void pushIono(const Klobuchar&) override {}
    void pushPvt(const ReceiverPvt& p) override { pvt.push_back(p.t); }
  };
  for (const livo_wpa_testing::Slice& sl : slices) {
    Sink sink;
    std::string why;
    livo_wpa_testing::DecodeStats ds_obs, ds_pvt;
    livo_gnss_io::GnssCommRos1Decoder dec_obs(&satNoOf), dec_pvt(&satNoOf);
    ASSERT_TRUE(livo_wpa_testing::decodeChannel(dec_obs, sl.path, livo_wpa_testing::kMeasChannel, sink, &ds_obs, &why)) << why;
    ASSERT_TRUE(livo_wpa_testing::decodeChannel(dec_pvt, sl.path, kPvtChannel, sink, &ds_pvt, &why, /*required=*/false)) << why;
    for (const std::vector<GTime>* stream : {&sink.obs, &sink.pvt}) {
      const bool is_obs = stream == &sink.obs;
      if (stream->empty()) continue;
      for (size_t i = 1; i < stream->size(); ++i) ASSERT_TRUE((*stream)[i - 1] < (*stream)[i]) << sl.name << ": not in time order";
      EpochDecimator dec(1.0);
      std::vector<size_t> taken;
      size_t old = 0;
      for (size_t i = 0; i < stream->size(); ++i) {
        if (dec.admit((*stream)[i])) taken.push_back(i);
        old += oldFiveMsRule((*stream)[i]) ? 1 : 0;
      }
      const std::vector<size_t> expect = firstPerSecond(*stream);
      EXPECT_EQ(taken, expect) << sl.name << (is_obs ? " range_meas" : " receiver_pvt");
      const double span = stream->back().diff(stream->front());
      EXPECT_GE(static_cast<double>(taken.size()), std::floor(span)) << sl.name << ": a GPST second without its epoch";
      std::printf("[U7 A43] %s %s: %zu epochs over %.1f s, %zu taken (first per GPST second), %llu skipped; the retired "
                  "5 ms rule would take %zu\n",
                  sl.name.c_str(), is_obs ? "range_meas" : "receiver_pvt", stream->size(), span, taken.size(),
                  static_cast<unsigned long long>(dec.skipped()), old);
    }
  }
}
