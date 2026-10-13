// SPDX-License-Identifier: BSD-3-Clause
// test_u05_corrections.cpp — TEST_PLAN §1 U5 (AMENDMENTS A9-1: the robust GNSS-only LS cases moved to U16, WP-c;
// A9-4: pntpos golden file). Owner: WP-a.
//
// Two kinds of observations. Synthetic ones: pseudoranges/Dopplers generated from the IGS BRDC snippet (see
// test_u03_satmap.cpp; $LIVO_GNSS_BRDC_DIR, default /r/data/brdc; sha256-pinned in
// third_party/rtklib/testing/wpa_test_support.hpp) for a known receiver state near Shanghai, with the broadcast Klobuchar
// and Saastamoinen delays and per-system time offsets. Fixture ones (A24, found by gnss_io/test/fixture_discovery.hpp,
// A39): the dev-set slice bytes through WP-e's decoder; the gate tests use dev-set slices only, the *SyntheticSlice
// companions WP-e's synthetic slices (PASS(合成), A24). A missing dependency skips with "blocked: dev-set fixture" /
// "blocked: WP-e merge" / "blocked: WP-c merge" only while it is missing.
//
// Golden files (A9-4 / m17): pntpos() results in hexfloat from a LIVO_RTKLIB_APPLY_PATCHES=OFF build, written once with
//   LIVO_GNSS_WRITE_GOLDEN=1 [LIVO_GNSS_GOLDEN_OUT=<dir>] ./test_u05_corrections
// synthetic → third_party/rtklib/golden/pntpos_unpatched_synth.hex (WP-a's tree; sha256 in third_party/rtklib/VENDORED.md)
// fixture   → gnss/test/data/<slice>/golden/pntpos_unpatched.hex (A24; with LIVO_GNSS_GOLDEN_OUT=<dir>:
//             <dir>/<slice>/golden/pntpos_unpatched.hex, to be copied into the slice).
#include <gtest/gtest.h>

#include <dirent.h>
#include <sys/stat.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iterator>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "../../gnss_io/test/fixture_discovery.hpp"   // A24/A39: the fixture discovery rule shared with WP-e
#include "../../third_party/rtklib/testing/wpa_test_support.hpp"
#include "internal/quality.hpp"
#include "internal/rows.hpp"          // EpochDecimator: the 1 Hz rule of AMENDMENTS A43 (fixture golden sampling)
#include "internal/rtk_adapter.hpp"
#include "internal/time_map.hpp"
#include "livo_gnss/gnss_fusion.hpp"
#include "livo_gnss_io/gnss_comm_mapping.hpp"
#include "rtklib.h"   // WP-a test files may include it (OWNERSHIP §1): RTKLIB constants of the reference model

using namespace livo_gnss;

namespace {

const char* kSnippet = livo_wpa_testing::kSnippetName;   // named in the golden header

using livo_wpa_testing::brdcPath;
#define REQUIRE_SNIPPET() LIVO_REQUIRE_BRDC_SNIPPET()
#define REQUIRE_P2() \
  if (!(rtk::patchLevel() & rtk::kPatchP2)) GTEST_SKIP() << "unpatched reference build (LIVO_RTKLIB_APPLY_PATCHES=OFF): no P2 exports"

GTime ep(double y, double mo, double d, double h, double mi, double s) {
  const double e[6] = {y, mo, d, h, mi, s};
  return rtk::epoch2time(e);
}

constexpr double kD2R = 3.1415926535897932 / 180.0;
constexpr double kHumi = 0.7;   // pntpos.c REL_HUMI

// Synthetic receiver truth (DESIGN §1 sign conventions): pseudorange P = ρ + b + ι_sys − c·δt_s + I + T (+ the group
// delay prange() removes again); Doppler −λD = eᵀ(v_s − v_r) + Sagnac rate + d − c·δṫ_s.
struct Truth {
  Geodetic geo{31.0283 * kD2R, 121.4413 * kD2R, 12.0};
  Eigen::Vector3d r = Eigen::Vector3d::Zero();
  Eigen::Vector3d v{1.2, -0.7, 0.3};
  double b = 1234.5;   // receiver clock [m]
  double d = 0.05;     // clock drift [m/s]
  std::map<Sys, double> isb{{Sys::GPS, 0.0}, {Sys::GLO, 11.5}, {Sys::GAL, 3.2}, {Sys::BDS, -7.9}, {Sys::QZS, 0.8}};
  Truth() { r = rtk::pos2ecef(geo); }
};

SignalCode primaryCode(Sys s) {
  switch (s) {
    case Sys::GPS:
    case Sys::QZS:
    case Sys::GLO: return SignalCode::L1C;
    case Sys::GAL: return SignalCode::L1X;
    case Sys::BDS: return SignalCode::L2I;   // B1I (§12.3)
    default: return SignalCode::Unknown;
  }
}

// Group delay that RTKLIB prange() removes on the single-frequency branch (pntpos.c:161-189), added back so the model
// holds for the corrected pseudorange; taken from the selected record (prange uses the first record of the satellite,
// equal within this 5-hour window).
double prangeGroupDelay(Sys s, const rtk::SatState& st) {
  switch (s) {
    case Sys::GPS:
    case Sys::QZS: return rtk::kCLight * st.tgd_s[0];
    case Sys::GAL: return rtk::kCLight * (rtk::galileoNavSelection() != rtk::GalNavSelection::Any ? st.tgd_s[0] : st.tgd_s[1]);
    case Sys::BDS: return rtk::kCLight * st.tgd_s[0];   // TGD_B1I for CODE_L2I
    case Sys::GLO: {
      const double gamma = (FREQ1_GLO / FREQ2_GLO) * (FREQ1_GLO / FREQ2_GLO);
      return (-st.tgd_s[0] * rtk::kCLight) / (gamma - 1.0);   // P1 − b1/(γ−1), b1 = −dtaun·c
    }
    default: return 0.0;
  }
}

struct SatInfo {
  SatId sat;
  double el = 0.0;
};

// One synthetic epoch. `bds_max_prn` limits BeiDou to what an unpatched build can map (golden: 50).
ObsEpoch makeEpoch(const rtk::NavStore& nav, const Klobuchar& ion, GTime t_rx, const Truth& tr, int bds_max_prn,
                   std::vector<SatInfo>* info = nullptr) {
  ObsEpoch ep;
  ep.t_rx = t_rx;
  ep.source = Source::Synthetic;
  const rtk::SatSolver solver(nav);
  std::set<int> sats;
  for (const rtk::EphKey& k : nav.coverage()) sats.insert(k.sat.rtk_sat);
  for (int n : sats) {
    SatId sat;
    sat.rtk_sat = n;
    sat.sys = rtk::satSys(n, &sat.prn);
    if (sat.sys == Sys::BDS && sat.prn > bds_max_prn) continue;
    const SignalCode code = primaryCode(sat.sys);
    if (code == SignalCode::Unknown) continue;
    const double f = rtk::sat2freq(n, code, nav);
    if (f == 0.0) continue;
    double P = 2.2e7, az = 0.0, el = 0.0;
    rtk::SatState st;
    bool ok = true;
    Eigen::Vector3d e = Eigen::Vector3d::Zero();
    for (int it = 0; it < 4 && ok; ++it) {   // fixed point on the transmit time
      if (!solver.satState(sat, t_rx, P, &st)) {
        ok = false;
        break;
      }
      const double rho = rtk::geodist(st.r_ecef, tr.r, &e);
      rtk::satAzel(tr.geo, e, &az, &el);
      const double I = rtk::ionModelGpsL1(t_rx, ion, tr.geo, az, el) * (FREQL1 / f) * (FREQL1 / f);
      const double T = rtk::tropModel(t_rx, tr.geo, az, el, kHumi);
      P = rho + tr.b + tr.isb.at(sat.sys) - rtk::kCLight * st.dts + I + T + prangeGroupDelay(sat.sys, st);
    }
    // Health as in the core (rtk::svhHealthy = RTKLIB satexclude: the IGS BRDC QZSS svh = 1 is healthy).
    if (!ok || !rtk::svhHealthy(sat.sys, st.svh) || el < 15.0 * kD2R) continue;
    const Eigen::Vector3d vs = st.v_ecef, rs = st.r_ecef;
    const double rate = e.dot(vs - tr.v) + rtk::kOmgE / rtk::kCLight * (vs.y() * tr.r.x() + rs.y() * tr.v.x() - vs.x() * tr.r.y() - rs.x() * tr.v.y());
    SatObs so;
    so.sat = sat;
    so.t_rx = t_rx;
    SignalObs sig;
    sig.freq_hz = f;
    sig.code = code;
    sig.psr_m = P;
    sig.dopp_hz = -(rate + tr.d - rtk::kCLight * st.ddts) * f / rtk::kCLight;
    sig.cn0_dbhz = 45.0;
    sig.status = kStatusPsrValid;
    so.sig.push_back(sig);
    ep.sats.push_back(so);
    if (info) info->push_back(SatInfo{sat, el});
  }
  return ep;
}

const SatObs* findSat(const ObsEpoch& e, int rtk_sat) {
  for (const SatObs& s : e.sats)
    if (s.sat.rtk_sat == rtk_sat) return &s;
  return nullptr;
}

int isbIndex(Sys s) {   // pntpos.c rescode: x[4] GLO, x[5] GAL, x[6] BDS, x[7] IRN, x[8] QZS (QZSDT)
  switch (s) {
    case Sys::GLO: return 4;
    case Sys::GAL: return 5;
    case Sys::BDS: return 6;
    case Sys::IRN: return 7;
    case Sys::QZS: return 8;
    default: return -1;
  }
}

std::string hexf(double v) {
  char b[64];
  std::snprintf(b, sizeof(b), "%a", v);
  return b;
}

// Deterministic golden text of a list of pntpos() results: everything an unpatched build can produce.
std::string goldenText(const std::vector<ObsEpoch>& epochs, const std::vector<rtk::SppRefResult>& res, const std::string& header) {
  std::ostringstream o;
  o << header;
  for (size_t i = 0; i < res.size(); ++i) {
    const rtk::SppRefResult& r = res[i];
    o << "epoch " << epochs[i].t_rx.sec << ' ' << hexf(epochs[i].t_rx.frac) << " ok " << (r.ok ? 1 : 0) << " ns " << r.ns << " nsat "
      << epochs[i].sats.size() << " rr";
    for (int k = 0; k < 3; ++k) o << ' ' << hexf(r.r_ecef[k]);
    for (int k = 0; k < 3; ++k) o << ' ' << hexf(r.v_ecef[k]);
    o << " dtr";
    for (int k = 0; k < 6; ++k) o << ' ' << hexf(r.dtr_s[k]);
    o << " gdop " << hexf(r.gdop) << '\n';
    for (size_t j = 0; j < r.resp.size(); ++j)
      o << "  sat " << rtk::satIdString(r.resp[j].first.rtk_sat) << " resp " << hexf(r.resp[j].second) << " az "
        << hexf(r.azel[j].second[0]) << " el " << hexf(r.azel[j].second[1]) << '\n';
  }
  return o.str();
}

// Rounds a synthetic epoch to RINEX observation precision (F14.3: 1 mm, 1 mHz). The generator runs in this test TU, whose
// floating-point contraction follows the build profile (default: FMA allowed; eval profile: -ffp-contract=off), so the
// last bits of P and D differ between the profiles; on the RINEX grid they coincide (a value would have to lie within a
// few ulps of a grid mid-point to split), which makes one golden file valid for both profiles (I1; maintainer P2b ③).
void roundToRinexPrecision(ObsEpoch* e) {
  for (SatObs& so : e->sats)
    for (SignalObs& sg : so.sig) {
      sg.psr_m = std::round(sg.psr_m * 1e3) / 1e3;
      sg.dopp_hz = std::round(sg.dopp_hz * 1e3) / 1e3;
    }
}

// The synthetic golden set: 12 epochs, 5 min apart, GPS/GLO(pseudorange on)/GAL/QZS/BDS ≤ C50, rounded to RINEX
// precision (roundToRinexPrecision).
void syntheticGoldenRun(const rtk::NavStore& nav, std::vector<ObsEpoch>* epochs, std::vector<rtk::SppRefResult>* res) {
  Klobuchar ion;
  nav.iono(&ion);
  rtk::SppRefOptions opt;
  opt.use_glo_psr = true;
  opt.el_mask_deg = 10.0;
  const rtk::SppRef spp;
  for (int k = 0; k < 12; ++k) {
    Truth tr;
    tr.b += 0.1 * k;
    ObsEpoch e = makeEpoch(nav, ion, ep(2025, 1, 3, 6, 3, 21.0).add(300.0 * k), tr, 50);
    roundToRinexPrecision(&e);
    rtk::SppRefResult r;
    spp.solve(e, nav, opt, &r);
    epochs->push_back(e);
    res->push_back(r);
  }
}

std::string repoRoot() { return std::string(LIVO_GNSS_TEST_DATA_DIR) + "/../../.."; }
std::string readFile(const std::string& p, bool* ok) {
  std::ifstream f(p);
  *ok = static_cast<bool>(f);
  return std::string((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
}
bool writeGoldenRequested() {
  const char* w = std::getenv("LIVO_GNSS_WRITE_GOLDEN");
  return w && std::string(w) == "1";
}
std::string goldenOut(const std::string& default_path) {
  const char* d = std::getenv("LIVO_GNSS_GOLDEN_OUT");
  if (!d || !*d) return default_path;
  const size_t s = default_path.find_last_of('/');
  return std::string(d) + "/" + default_path.substr(s + 1);
}

// First differing line, for a readable failure.
std::string firstDiff(const std::string& a, const std::string& b) {
  std::istringstream sa(a), sb(b);
  std::string la, lb;
  int n = 0;
  while (true) {
    const bool ea = !std::getline(sa, la), eb = !std::getline(sb, lb);
    ++n;
    if (ea && eb) return "";
    if (ea != eb || la != lb) return "line " + std::to_string(n) + ":\n  golden: " + (ea ? "<eof>" : la) + "\n  now:    " + (eb ? "<eof>" : lb);
  }
}

gtime_t toG(GTime t) {
  gtime_t g;
  g.time = static_cast<time_t>(t.sec);
  g.sec = t.frac;
  return g;
}
bool sameBits(double a, double b) { return std::memcmp(&a, &b, sizeof(double)) == 0; }
bool sameBits(const Eigen::VectorXd& a, const Eigen::VectorXd& b) {
  if (a.size() != b.size()) return false;
  for (Eigen::Index i = 0; i < a.size(); ++i)
    if (!sameBits(a[i], b[i])) return false;
  return true;
}
bool sameBits(const Eigen::MatrixXd& a, const Eigen::MatrixXd& b) {
  if (a.rows() != b.rows() || a.cols() != b.cols()) return false;
  for (Eigen::Index i = 0; i < a.rows(); ++i)
    for (Eigen::Index j = 0; j < a.cols(); ++j)
      if (!sameBits(a(i, j), b(i, j))) return false;
  return true;
}

// ---- fixture (A24): slices, channels and decoding are shared (wpa_test_support.hpp) ----
struct ObsSink : MeasurementSink {
  std::vector<ObsEpoch> obs;
  void pushObservation(ObsEpoch&& e) override { obs.push_back(std::move(e)); }
  void pushEphemeris(const KeplerEph&) override {}
  void pushGloEphemeris(const GloEph&) override {}
  void pushIono(const Klobuchar&) override {}
  void pushPvt(const ReceiverPvt&) override {}
};
// range_meas bytes of a slice through WP-e's decoder, in record order (a stub throws std::logic_error).
bool decodeFixtureObs(const std::string& slice, std::vector<ObsEpoch>* out, livo_wpa_testing::DecodeStats* st, std::string* why) {
  livo_gnss_io::GnssCommRos1Decoder dec(&satNoOf);
  ObsSink sink;
  if (!livo_wpa_testing::decodeChannel(dec, slice, livo_wpa_testing::kMeasChannel, sink, st, why)) return false;
  *out = std::move(sink.obs);
  return true;
}

// DESIGN §12.6-3 grids, checked here independently of StdUnitJudge: psr 0.01·2ⁿ m, cp 0.004·n cycles, dopp 0.002·2ⁿ Hz
// (u-blox RXM-RAWX 4-bit fields, n = 0..15); Metres for cp means cycles·λ; Index means the raw 4-bit n.
bool nearInt(double x, int lo, int hi) {
  const double r = std::round(x);
  return r >= lo && r <= hi && std::fabs(x - r) < 1e-6 * std::max(1.0, std::fabs(x));
}
bool psrOnGrid(double v, StdUnit u) {
  if (u == StdUnit::Index) return nearInt(v, 0, 15);
  return u == StdUnit::Metres && v > 0.0 && nearInt(std::log2(v / 0.01), 0, 15);
}
bool cpOnGrid(double v, StdUnit u, double lambda) {
  if (u == StdUnit::Index) return nearInt(v, 0, 15);
  if (u == StdUnit::Cycles) return nearInt(v / 0.004, 0, 15);
  return u == StdUnit::Metres && lambda > 0.0 && nearInt(v / (0.004 * lambda), 0, 15);
}
bool doppOnGrid(double v, StdUnit u) {
  if (u == StdUnit::Index) return nearInt(v, 0, 15);
  return u == StdUnit::Hertz && v > 0.0 && nearInt(std::log2(v / 0.002), 0, 15);
}

}  // namespace

// at the pntpos solution the core prediction differs from ssat.resp by < 1e-6 m
TEST(U05Corrections, PseudorangePredictionMatchesSsatResp) {
  REQUIRE_SNIPPET();
  REQUIRE_P2();
  rtk::setGalileoNavSelection(rtk::GalNavSelection::INav);
  rtk::NavStore nav;
  std::string why;
  ASSERT_GT(nav.loadRinexNav(brdcPath(), true, &why), 0) << why;
  Klobuchar ion;
  ASSERT_TRUE(nav.iono(&ion));   // RINEX header GPSA/GPSB: the same parameters pntpos reads from nav.ion_gps
  rtk::SppRefOptions opt;
  opt.use_glo_psr = true;
  const rtk::SppRef spp;
  const rtk::SatSolver solver(nav);
  std::map<Sys, int> n_sys;
  std::set<int> bds_prn;
  for (int k = 0; k < 4; ++k) {
    const Truth tr;
    const GTime t = ep(2025, 1, 3, 5, 3, 21.0).add(1200.0 * k);
    const ObsEpoch e = makeEpoch(nav, ion, t, tr, 63);
    rtk::SppRefResult res;
    ASSERT_TRUE(spp.solve(e, nav, opt, &res)) << res.msg;
    ASSERT_EQ(res.x_pos.size(), 9);
    ASSERT_EQ(res.x_pos_lin.size(), 9);
    ASSERT_EQ(res.Q_pos.rows(), 9);
    ASSERT_EQ(res.resp.size(), res.azel.size());
    // The synthetic truth is recovered: the model and the generator agree.
    EXPECT_LT((res.r_ecef - tr.r).norm(), 1e-3) << "epoch " << k;
    EXPECT_NEAR(res.dtr_s[0] * rtk::kCLight, tr.b, 1e-3);
    EXPECT_NEAR(res.x_pos[5], tr.isb.at(Sys::GAL), 1e-3);
    EXPECT_NEAR(res.x_pos[6], tr.isb.at(Sys::BDS), 1e-3);
    EXPECT_NEAR(res.x_pos[4], tr.isb.at(Sys::GLO), 1e-3);
    // The linearisation point of the last iteration is within the convergence threshold of the solution.
    EXPECT_LT((res.x_pos - res.x_pos_lin).norm(), 1e-4);
    const Eigen::Vector3d rr = res.x_pos_lin.head<3>();
    const Geodetic geo = rtk::ecef2pos(rr);
    for (size_t j = 0; j < res.resp.size(); ++j) {
      const SatId sat = res.resp[j].first;
      const std::string id = rtk::satIdString(sat.rtk_sat);
      const SatObs* so = findSat(e, sat.rtk_sat);
      ASSERT_NE(so, nullptr) << id;
      const rtk::PrangeRef pr = rtk::prangeRef(*so, nav, opt);
      ASSERT_TRUE(pr.ok) << id;
      const double pred_rtklib = pr.P_corr - res.resp[j].second;   // rescode: v = P − (r + dtr − c·dts + I + T) − ι
      // The core's prediction from the adapter's building blocks at the same point.
      rtk::SatState st;
      ASSERT_TRUE(solver.satState(sat, e.t_rx, so->sig[0].psr_m, &st)) << id;
      Eigen::Vector3d los;
      const double rho = rtk::geodist(st.r_ecef, rr, &los);
      double az = 0.0, el = 0.0;
      rtk::satAzel(geo, los, &az, &el);
      const double f = rtk::sat2freq(sat.rtk_sat, so->sig[0].code, nav);
      const double I = rtk::ionModelGpsL1(e.t_rx, ion, geo, az, el) * (FREQL1 / f) * (FREQL1 / f);
      const double T = rtk::tropModel(e.t_rx, geo, az, el, kHumi);
      const int xi = isbIndex(sat.sys);
      const double pred = rho + res.x_pos_lin[3] - rtk::kCLight * st.dts + I + T + (xi >= 0 ? res.x_pos_lin[xi] : 0.0);
      EXPECT_LT(std::fabs(pred - pred_rtklib), 1e-6) << id << " pred " << pred << " rtklib " << pred_rtklib;
      EXPECT_NEAR(az, res.azel[j].second[0], 1e-12) << id;
      EXPECT_NEAR(el, res.azel[j].second[1], 1e-12) << id;
      // prangeRef reports the group delay prange() applied on its branch (GPS/QZS TGD, GAL BGD_E1E5a under the I/NAV
      // selection — see rtk_adapter.cpp — BDS TGD_B1I, GLO −dtaun·c). prange() reads it from the *first* record of the
      // satellite (gettgd), which in the IGS merge differs from the selected one by print precision only (≤ 1e-16 s).
      EXPECT_NEAR(pr.tgd_s * rtk::kCLight, sat.sys == Sys::GLO ? -st.tgd_s[0] * rtk::kCLight : prangeGroupDelay(sat.sys, st), 1e-6) << id;
      ++n_sys[sat.sys];
      if (sat.sys == Sys::BDS) bds_prn.insert(sat.prn);
    }
  }
  EXPECT_GT(n_sys[Sys::GPS], 16);
  EXPECT_GT(n_sys[Sys::GAL], 8);
  EXPECT_GT(n_sys[Sys::BDS], 8);
  EXPECT_GT(n_sys[Sys::GLO], 4);
  EXPECT_GT(n_sys[Sys::QZS], 0);
  bool geo = false, igso = false, meo = false, c59 = false;
  for (int p : bds_prn) {
    geo = geo || p <= 5 || p >= 59;
    c59 = c59 || p >= 59;
    igso = igso || (p >= 6 && p <= 10) || p == 13 || p == 16 || (p >= 38 && p <= 40);
    meo = meo || (p >= 11 && p <= 37 && p != 13 && p != 16) || (p >= 41 && p <= 50);
  }
  EXPECT_TRUE(geo && igso && meo && c59) << "BDS classes in the pntpos solutions";
}

// Doppler residuals at the estvel solution have ≈0 mean
TEST(U05Corrections, DopplerResidualMeanZeroAtEstvel) {
  REQUIRE_SNIPPET();
  REQUIRE_P2();
  rtk::setGalileoNavSelection(rtk::GalNavSelection::INav);
  rtk::NavStore nav;
  std::string why;
  ASSERT_GT(nav.loadRinexNav(brdcPath(), true, &why), 0) << why;
  Klobuchar ion;
  ASSERT_TRUE(nav.iono(&ion));
  rtk::SppRefOptions opt;
  opt.use_glo_psr = true;
  const rtk::SppRef spp;
  const rtk::SatSolver solver(nav);
  const double err_hz = 1.0;   // prcopt_default.err[4]: resdop weights 1/σ², σ = err·c/f
  for (int k = 0; k < 4; ++k) {
    Truth tr;
    tr.v = Eigen::Vector3d(1.2 - 0.3 * k, -0.7 + 0.2 * k, 0.3);
    tr.d = 0.05 - 0.02 * k;
    const ObsEpoch e = makeEpoch(nav, ion, ep(2025, 1, 3, 5, 13, 21.0).add(1200.0 * k), tr, 63);
    rtk::SppRefResult res;
    ASSERT_TRUE(spp.solve(e, nav, opt, &res)) << res.msg;
    ASSERT_EQ(res.x_vel.size(), 4);
    // estvel recovers the synthetic velocity and drift (P2 exports the drift pntpos drops, R2-6).
    EXPECT_LT((res.v_ecef - tr.v).norm(), 1e-3);
    EXPECT_NEAR(res.x_vel[3], tr.d, 1e-3);
    EXPECT_EQ(res.ddtr, res.x_vel[3] / rtk::kCLight);
    EXPECT_EQ(res.v_ecef, Eigen::Vector3d(res.x_vel.head<3>()));
    // The core's Doppler model (DESIGN §1: −λD = eᵀ(v_s − v_r) + (ω_e/c)(v_s,y x_r + y_s v_r,x − v_s,x y_r − x_s v_r,y)
    // + d − c·δṫ_s) evaluated at the estvel solution with the adapter's satellite state.
    double sw = 0.0, swr = 0.0, sr = 0.0;
    int n = 0;
    for (const auto& pr : res.resp) {
      const SatObs* so = findSat(e, pr.first.rtk_sat);
      ASSERT_NE(so, nullptr);
      const SignalObs& s = so->sig[0];
      rtk::SatState st;
      ASSERT_TRUE(solver.satState(pr.first, e.t_rx, s.psr_m, &st));
      const double f = rtk::sat2freq(pr.first.rtk_sat, s.code, nav);
      const double lam = rtk::kCLight / f;
      Eigen::Vector3d los;
      rtk::geodist(st.r_ecef, res.r_ecef, &los);
      const Eigen::Vector3d& vs = st.v_ecef;
      const Eigen::Vector3d& rs = st.r_ecef;
      const Eigen::Vector3d& rr = res.r_ecef;
      const Eigen::Vector3d& vr = res.v_ecef;
      const double rate = los.dot(vs - vr) + rtk::kOmgE / rtk::kCLight * (vs.y() * rr.x() + rs.y() * vr.x() - vs.x() * rr.y() - rs.x() * vr.y());
      const double D = static_cast<double>(static_cast<float>(s.dopp_hz));   // obsd_t.D is float
      const double r = -lam * D - (rate + res.x_vel[3] - rtk::kCLight * st.ddts);
      const double w = 1.0 / ((err_hz * lam) * (err_hz * lam));
      sw += w;
      swr += w * r;
      sr += r;
      ++n;
      // Noise-free except the float rounding of D (≤ 2.4e-4 Hz · λ): a sign or Sagnac-rate error would be cm/s.
      EXPECT_LT(std::fabs(r), 1e-3) << rtk::satIdString(pr.first.rtk_sat);
    }
    ASSERT_GE(n, 10);
    // Least squares with a clock-drift column: the weighted residual sum vanishes at the solution.
    EXPECT_LT(std::fabs(swr / sw), 1e-6) << "weighted mean";
    EXPECT_LT(std::fabs(sr / n), 1e-4) << "mean";
  }
}

// pntpos() of the patched build equals the golden file produced once by an unpatched build
// (-DLIVO_RTKLIB_APPLY_PATCHES=OFF, hexfloat) bit for bit (A9-4 / m17), on synthetic epochs from the BRDC snippet (the
// fixture variant below does the same on the dev-set slices). The inputs are rounded to RINEX precision so that the
// golden holds in both build profiles (roundToRinexPrecision); its sha256 is in third_party/rtklib/VENDORED.md.
TEST(U05Corrections, PatchedPntposEqualsUnpatchedGolden) {
  REQUIRE_SNIPPET();
  rtk::setGalileoNavSelection(rtk::GalNavSelection::INav);
  rtk::NavStore nav;
  std::string why;
  ASSERT_GT(nav.loadRinexNav(brdcPath(), true, &why), 0) << why;
  std::vector<ObsEpoch> epochs;
  std::vector<rtk::SppRefResult> res;
  syntheticGoldenRun(nav, &epochs, &res);
  int ok = 0;
  for (const auto& r : res) ok += r.ok ? 1 : 0;
  ASSERT_EQ(ok, 12) << "every golden epoch must solve";
  const std::string header = std::string("# livo U5 golden (AMENDMENTS A9-4): pntpos() of the LIVO_RTKLIB_APPLY_PATCHES=OFF build, hexfloat\n") +
                             "# synthetic epochs (test_u05_corrections.cpp syntheticGoldenRun) from " + kSnippet +
                             ", P/D rounded to 1 mm / 1 mHz (profile-independent inputs)\n";
  const std::string text = goldenText(epochs, res, header);
  const std::string path = repoRoot() + "/third_party/rtklib/golden/pntpos_unpatched_synth.hex";
  if (rtk::patchLevel() == 0) {
    if (!writeGoldenRequested()) GTEST_SKIP() << "unpatched reference build: set LIVO_GNSS_WRITE_GOLDEN=1 to write " << path;
    const std::string out = goldenOut(path);
    std::ofstream f(out);
    ASSERT_TRUE(static_cast<bool>(f)) << out;
    f << text;
    GTEST_SKIP() << "golden written to " << out;
  }
  bool have = false;
  const std::string golden = readFile(path, &have);
  ASSERT_TRUE(have) << "golden missing: " << path << " (generate it with an unpatched build, see file header)";
  EXPECT_EQ(golden.size(), text.size());
  const std::string d = firstDiff(golden, text);
  EXPECT_TRUE(d.empty()) << "patched pntpos() differs from the unpatched golden at " << d;
}

// The same comparison on the fixture epochs (A24 golden/pntpos_unpatched.hex; review M1): one range_meas epoch per GPST
// second of each slice, the first to arrive in it (fixture bytes → WP-e's decoder, record order; AMENDMENTS A43, the
// pseudorange-row rule of rows.cpp), ephemerides and
// Klobuchar from the slice's own BRDC file only, BDS limited to C01–C50 (C51+ are unmappable in the unpatched build),
// GLONASS pseudoranges and RAIM enabled so every pntpos() branch the fixture reaches is covered. The unpatched build
// writes the file (LIVO_GNSS_WRITE_GOLDEN=1), the default build compares bit for bit. The inputs are decoded bytes and
// RINEX text, so one golden holds for both build profiles. A dev-set slice without its golden is a failure (WP-a's own
// deliverable, not an external block); a synthetic slice is compared when a golden exists there, otherwise only run.
namespace {

// The 1 Hz subset of a slice (AMENDMENTS A43, the one rule for the pseudorange rows, the L-PVT rows and this sampling):
// per GPST second ⌊tow⌋, the first range_meas epoch to arrive in it, in record order, through the core's own
// EpochDecimator. The receiver's 10 Hz grid sits off the whole tenths of a second by a per-recording constant
// (INTERFACE_CHANGES WP-e #16 / WP-g G15: Grass01 −7 ms, Grass02 0, Occlusion03/04 +5 ms, Dynamic03 +3 ms), which the
// rule ignores. A slice starting inside a second contributes that second's first epoch too (Grass02 at x.8 s,
// Occlusion03 at x.205 s) — the previous test-local rule (nearest to the whole second within 50 ms, P2b integration)
// skipped those two partial seconds; the goldens were rewritten with this rule (P3-C, INTERFACE_CHANGES).
std::vector<size_t> onePerGpstSecond(const std::vector<ObsEpoch>& all) {
  EpochDecimator dec(1.0);
  std::vector<size_t> idx;
  for (size_t i = 0; i < all.size(); ++i)
    if (dec.admit(all[i].t_rx)) idx.push_back(i);
  return idx;
}

// One slice: compare (patched build) or write (unpatched build with LIVO_GNSS_WRITE_GOLDEN=1, path added to `written`).
// `solved` = selected (1 Hz) epochs pntpos() solved.
void pntposGoldenSlice(const livo_wpa_testing::Slice& sl, bool golden_required, std::string* written, int* solved) {
  const std::string& slice = sl.path;
  std::vector<ObsEpoch> all;
  livo_wpa_testing::DecodeStats ds;
  std::string why;
  try {
    ASSERT_TRUE(decodeFixtureObs(slice, &all, &ds, &why)) << why;
  } catch (const std::logic_error& e) {
    LIVO_BLOCKED_IF_STUB(e, "WP-e merge (gnss_io GnssCommRos1Decoder)");
  }
  const std::string brdc = livo_gnss_io::testsupport::fixtureBrdc(slice);
  ASSERT_FALSE(brdc.empty()) << slice << ": A24 brdc/ missing";
  rtk::NavStore nav;
  ASSERT_GT(nav.loadRinexNav(brdc, true, &why), 0) << why;
  std::vector<ObsEpoch> epochs;
  for (const size_t i : onePerGpstSecond(all)) {
    ObsEpoch f = all[i];
    f.sats.erase(std::remove_if(f.sats.begin(), f.sats.end(),
                                [](const SatObs& so) { return !so.sat.valid() || (so.sat.sys == Sys::BDS && so.sat.prn > 50); }),
                 f.sats.end());
    epochs.push_back(std::move(f));
  }
  ASSERT_FALSE(epochs.empty()) << slice << ": no range_meas epoch selected by the 1 Hz rule (A43; " << all.size() << " epochs decoded)";
  rtk::SppRefOptions opt;
  opt.use_glo_psr = true;
  opt.raim_fde = true;
  const rtk::SppRef spp;
  std::vector<rtk::SppRefResult> res;
  int ok = 0;
  for (const ObsEpoch& e : epochs) {
    rtk::SppRefResult r;
    spp.solve(e, nav, opt, &r);
    ok += r.ok ? 1 : 0;
    res.push_back(r);
  }
  *solved = ok;
  std::printf("[U5 fixture] %s: %d range_meas messages (%d refused), %zu epochs at 1 Hz, %d solved\n", sl.name.c_str(), ds.messages,
              ds.refused, epochs.size(), ok);
  const std::string header = std::string("# livo U5 golden (AMENDMENTS A9-4): pntpos() of the LIVO_RTKLIB_APPLY_PATCHES=OFF build, hexfloat\n") +
                             "# fixture " + sl.name + ": one range_meas epoch per GPST second (the first to arrive, A43), BDS <= C50; nav/iono " +
                             brdc.substr(brdc.find_last_of('/') + 1) + " only; use_glo_psr 1, raim_fde 1, Galileo I/NAV\n";
  const std::string text = goldenText(epochs, res, header);
  const std::string path = slice + "/golden/pntpos_unpatched.hex";
  if (rtk::patchLevel() == 0) {
    if (!writeGoldenRequested()) GTEST_SKIP() << "unpatched reference build: set LIVO_GNSS_WRITE_GOLDEN=1 to write " << path;
    std::string out = path;
    const char* d = std::getenv("LIVO_GNSS_GOLDEN_OUT");
    if (d && *d) {   // mirror the slice layout so the file can be copied into the fixture as is
      out = std::string(d) + "/" + sl.name;
      ::mkdir(out.c_str(), 0775);
      out += "/golden";
      ::mkdir(out.c_str(), 0775);
      out += "/pntpos_unpatched.hex";
    }
    std::ofstream f(out);
    ASSERT_TRUE(static_cast<bool>(f)) << out;
    f << text;
    *written += " " + out;
    return;
  }
  bool have = false;
  const std::string golden = readFile(path, &have);
  if (!have && !golden_required) {
    std::printf("[U5 fixture] %s: no golden in this slice, run only\n", sl.name.c_str());
    return;
  }
  ASSERT_TRUE(have) << "fixture golden missing: " << path
                    << " (write it once with the LIVO_RTKLIB_APPLY_PATCHES=OFF build, see the file header and VENDORED.md)";
  EXPECT_EQ(golden.size(), text.size()) << slice;
  const std::string diff = firstDiff(golden, text);
  EXPECT_TRUE(diff.empty()) << slice << ": patched pntpos() differs from the unpatched golden at " << diff;
}

}  // namespace

// Gate: every dev-set slice has its golden and matches it bit for bit; together they solve at least one epoch, or the
// comparison would be vacuous (an open-sky slice, A26 Grass01/02, does).
TEST(U05Corrections, PatchedPntposEqualsUnpatchedGoldenFixture) {
  LIVO_DEVSET_SLICES(slices);
  rtk::setGalileoNavSelection(rtk::GalNavSelection::INav);
  std::string written;
  int dev_solved = 0;
  for (const livo_wpa_testing::Slice& sl : slices) {
    int solved = 0;
    pntposGoldenSlice(sl, /*golden_required=*/true, &written, &solved);
    if (HasFatalFailure() || IsSkipped()) return;
    dev_solved += solved;
  }
  if (!written.empty()) GTEST_SKIP() << "golden written:" << written;
  EXPECT_GT(dev_solved, 0) << "no 1 Hz epoch of the dev-set slices solves: the golden comparison is vacuous";
}

// WP-e's synthetic slices: compared where a golden exists, otherwise run only (PASS(合成), A24).
TEST(U05Corrections, PatchedPntposEqualsUnpatchedGoldenSyntheticSlice) {
  LIVO_SYNTHETIC_SLICES(slices);
  rtk::setGalileoNavSelection(rtk::GalNavSelection::INav);
  std::string written;
  for (const livo_wpa_testing::Slice& sl : slices) {
    int solved = 0;
    pntposGoldenSlice(sl, /*golden_required=*/false, &written, &solved);
    if (HasFatalFailure() || IsSkipped()) return;
  }
  if (!written.empty()) GTEST_SKIP() << "golden written:" << written;
}

namespace {
// u-blox RXM-RAWX quantisation: psrStdev 0.01·2ⁿ m, cpStdev 0.004·n cycles, doStdev 0.002·2ⁿ Hz.
ObsEpoch stdGridEpoch() {
  ObsEpoch e;
  e.t_rx = ep(2025, 1, 3, 6, 0, 0);
  for (int i = 0; i < 12; ++i) {
    SatObs so;
    so.sat = rtk::makeSatId(Sys::GPS, i + 1);
    so.t_rx = e.t_rx;
    SignalObs s;
    s.freq_hz = 1575.42e6;
    s.code = SignalCode::L1C;
    s.psr_m = 2.2e7;
    s.cp_cyc = 1.1e8;
    s.dopp_hz = 1000.0;
    s.status = kStatusPsrValid | kStatusCpValid | kStatusHalfCycValid;
    s.psr_std_raw = 0.01 * std::pow(2.0, i % 8);
    s.cp_std_raw = 0.004 * (1 + i % 15);
    s.dopp_std_raw = 0.002 * std::pow(2.0, i % 10);
    so.sig.push_back(s);
    e.sats.push_back(so);
  }
  return e;
}
}  // namespace

// psr/cp/dopp std units decided from the {0.01·2ⁿ}/{0.004n[·λ]}/{0.002·2ⁿ} grids — synthetic u-blox quantisation
// (review m6: this half turns green as soon as WP-c's StdUnitJudge lands)
TEST(U05Corrections, StdUnitJudgementSynthetic) {
  const ObsEpoch e = stdGridEpoch();
  try {
    StdUnitJudge j;
    for (int k = 0; k < 20; ++k) j.observe(e);
    ASSERT_TRUE(j.decided());
    EXPECT_EQ(j.psrUnit(), StdUnit::Metres);
    EXPECT_EQ(j.cpUnit(), StdUnit::Cycles);
    EXPECT_EQ(j.doppUnit(), StdUnit::Hertz);
    ObsEpoch c = e;
    j.apply(&c);
    const double lam = rtk::kCLight / 1575.42e6;
    for (size_t i = 0; i < c.sats.size(); ++i) {
      EXPECT_NEAR(c.sats[i].sig[0].psr_std_m, e.sats[i].sig[0].psr_std_raw, 1e-12);
      EXPECT_NEAR(c.sats[i].sig[0].cp_std_m, e.sats[i].sig[0].cp_std_raw * lam, 1e-12);
      EXPECT_NEAR(c.sats[i].sig[0].dopp_std_hz, e.sats[i].sig[0].dopp_std_raw, 1e-12);
    }
  } catch (const std::logic_error& ex) {
    LIVO_BLOCKED_IF_STUB(ex, "WP-c merge (StdUnitJudge in quality.cpp)");
  }
}

// The same judgement on the real gnss_comm std fields of the fixture (DESIGN §12.6-3): a decision is reached, and every
// raw value lies on the grid of the decided unit (checked here independently of StdUnitJudge); apply() converts the
// physical units as defined (Metres as is, Cycles·λ, Hertz as is). A real-data criterion (A24/A39): the gate runs on
// dev-set slices; the synthetic slices, whose std fields the generator put on the u-blox grids, run separately.
namespace {

void stdUnitsOnSlice(const livo_wpa_testing::Slice& sl) {
  const std::string& slice = sl.path;
  std::vector<ObsEpoch> obs;
  livo_wpa_testing::DecodeStats ds;
  std::string why;
  try {
    ASSERT_TRUE(decodeFixtureObs(slice, &obs, &ds, &why)) << why;
  } catch (const std::logic_error& e) {
    LIVO_BLOCKED_IF_STUB(e, "WP-e merge (gnss_io GnssCommRos1Decoder)");
  }
  ASSERT_FALSE(obs.empty()) << slice;
  StdUnitJudge j;
  std::vector<ObsEpoch> conv = obs;
  try {
    for (const ObsEpoch& e : obs) j.observe(e);
    ASSERT_TRUE(j.decided()) << slice << ": " << j.samples() << " samples, no decision";
    for (ObsEpoch& e : conv) j.apply(&e);
  } catch (const std::logic_error& ex) {
    LIVO_BLOCKED_IF_STUB(ex, "WP-c merge (StdUnitJudge in quality.cpp)");
  }
  ASSERT_NE(j.psrUnit(), StdUnit::Unknown);
  ASSERT_NE(j.cpUnit(), StdUnit::Unknown);
  ASSERT_NE(j.doppUnit(), StdUnit::Unknown);
  int n_psr = 0, n_cp = 0, n_dopp = 0, off_psr = 0, off_cp = 0, off_dopp = 0;
  for (size_t i = 0; i < obs.size(); ++i) {
    for (size_t k = 0; k < obs[i].sats.size(); ++k) {
      for (size_t m = 0; m < obs[i].sats[k].sig.size(); ++m) {
        const SignalObs& s = obs[i].sats[k].sig[m];
        const SignalObs& t = conv[i].sats[k].sig[m];
        const double lam = s.freq_hz > 0.0 ? rtk::kCLight / s.freq_hz : 0.0;
        if (s.psr_std_raw > 0.0) {
          ++n_psr;
          off_psr += psrOnGrid(s.psr_std_raw, j.psrUnit()) ? 0 : 1;
          if (j.psrUnit() == StdUnit::Metres) {
            EXPECT_DOUBLE_EQ(t.psr_std_m, s.psr_std_raw);
          }
        }
        if (s.cp_std_raw > 0.0) {
          ++n_cp;
          off_cp += cpOnGrid(s.cp_std_raw, j.cpUnit(), lam) ? 0 : 1;
          if (j.cpUnit() == StdUnit::Cycles) {
            EXPECT_NEAR(t.cp_std_m, s.cp_std_raw * lam, 1e-12);
          }
          if (j.cpUnit() == StdUnit::Metres) {
            EXPECT_DOUBLE_EQ(t.cp_std_m, s.cp_std_raw);
          }
        }
        if (s.dopp_std_raw > 0.0) {
          ++n_dopp;
          off_dopp += doppOnGrid(s.dopp_std_raw, j.doppUnit()) ? 0 : 1;
          if (j.doppUnit() == StdUnit::Hertz) {
            EXPECT_DOUBLE_EQ(t.dopp_std_hz, s.dopp_std_raw);
          }
        }
      }
    }
  }
  EXPECT_GT(n_psr, 100) << slice;
  EXPECT_GT(n_dopp, 100) << slice;
  EXPECT_EQ(off_psr, 0) << slice << ": psr_std values off the grid of the decided unit";
  EXPECT_EQ(off_cp, 0) << slice << ": cp_std values off the grid of the decided unit (" << n_cp << " values)";
  EXPECT_EQ(off_dopp, 0) << slice << ": dopp_std values off the grid of the decided unit";
  std::printf("[U5 std units] %s: psr %d cp %d dopp %d values (units %d/%d/%d), %d range_meas refused\n", sl.name.c_str(), n_psr, n_cp,
              n_dopp, static_cast<int>(j.psrUnit()), static_cast<int>(j.cpUnit()), static_cast<int>(j.doppUnit()), ds.refused);
}

}  // namespace

TEST(U05Corrections, StdUnitJudgementOnFixture) {
  LIVO_DEVSET_SLICES(slices);
  for (const livo_wpa_testing::Slice& sl : slices) {
    stdUnitsOnSlice(sl);
    if (HasFatalFailure() || IsSkipped()) return;
  }
}

TEST(U05Corrections, StdUnitJudgementSyntheticSlice) {
  LIVO_SYNTHETIC_SLICES(slices);
  for (const livo_wpa_testing::Slice& sl : slices) {
    stdUnitsOnSlice(sl);
    if (HasFatalFailure() || IsSkipped()) return;
  }
  std::printf("[U5 std units] synthetic slices: PASS(合成) only (A24)\n");
}

// Review M3: the core's group delay (rtk::codeBiasMetres) equals what RTKLIB prange() removes for GPS/QZS/BDS/GLONASS
// (GLONASS with a dtaun set, so the (γ−1) scaling and the sign are exercised), and for Galileo E1 follows the clock of
// the selected record — BGD_E1E5b with an I/NAV record (A16), BGD_E1E5a with an F/NAV one — where prange() keys on the
// global selection and removes BGD_E1E5a under I/NAV (E07 2025-01-03: 1.3262 m vs 0.8376 m).
TEST(U05Corrections, CodeBiasFollowsSelectedRecord) {
  REQUIRE_SNIPPET();
  REQUIRE_P2();
  rtk::NavStore nav;
  std::string why;
  ASSERT_GT(nav.loadRinexNav(brdcPath(), true, &why), 0) << why;
  // RINEX 3.04 carries no dtaun: give every GLONASS record one (first record = selected record for prange's gettgd).
  for (const rtk::EphKey& k : nav.coverage()) {
    if (!k.is_glo) continue;
    GloEph g;
    ASSERT_TRUE(nav.get(k.sat, k.toe, &g));
    g.dtaun = -2.5e-9 - 1e-10 * k.sat.prn;
    ASSERT_TRUE(nav.add(g, true));
  }
  Klobuchar ion;
  ASSERT_TRUE(nav.iono(&ion));
  // prange()'s gettgd() reads the satellite's *first* record in nav.eph, not the selected one (VENDORED.md ②); the
  // store orders nav.eph by (sat, toe, nav_type), which is also the order of coverage(). Under the "any" selection
  // that first record is often an I/NAV one while the selected record is F/NAV (BGD_E1E5b = 0 in the IGS F/NAV
  // records), so the reference value must come from it, not from the selected record.
  std::map<int, KeplerEph> first_gal;
  for (const rtk::EphKey& k : nav.coverage()) {
    if (k.sat.sys != Sys::GAL || first_gal.count(k.sat.rtk_sat)) continue;
    ASSERT_TRUE(nav.get(k.sat, k.toe, &first_gal[k.sat.rtk_sat], k.nav_type));
  }
  rtk::SppRefOptions opt;
  opt.use_glo_psr = true;
  const rtk::SatSolver solver(nav);
  const double gamma_glo = (FREQ1_GLO / FREQ2_GLO) * (FREQ1_GLO / FREQ2_GLO);
  const Truth tr;
  const GTime t = ep(2025, 1, 3, 6, 3, 21.0);
  std::map<Sys, int> n;
  bool saw_e07 = false;
  int any_fnav_mismatch = 0;
  for (rtk::GalNavSelection sel : {rtk::GalNavSelection::INav, rtk::GalNavSelection::FNav, rtk::GalNavSelection::Any}) {
    rtk::setGalileoNavSelection(sel);
    const ObsEpoch e = makeEpoch(nav, ion, t, tr, 63);
    ASSERT_GT(e.sats.size(), 20u);
    for (const SatObs& so : e.sats) {
      const std::string id = rtk::satIdString(so.sat.rtk_sat);
      const SignalObs& s = so.sig[0];
      rtk::SatState st;
      ASSERT_TRUE(solver.satState(so.sat, e.t_rx, s.psr_m, &st)) << id;
      const double cb = rtk::codeBiasMetres(so.sat, st, s.code);
      ASSERT_TRUE(std::isfinite(cb)) << id;
      const rtk::PrangeRef pr = rtk::prangeRef(so, nav, opt);
      ASSERT_TRUE(pr.ok) << id;
      const double removed = s.psr_m - pr.P_corr;   // what prange() took off (single-frequency branch)
      if (so.sat.sys == Sys::GAL) {
        ASSERT_NE(st.nav_type, 0) << id;
        ASSERT_EQ(first_gal.count(so.sat.rtk_sat), 1u) << id;
        const KeplerEph& first = first_gal.at(so.sat.rtk_sat);
        // The core: the BGD of the selected record's own clock.
        EXPECT_TRUE(sameBits(cb, rtk::kCLight * (st.nav_type == 1 ? st.tgd_s[1] : st.tgd_s[0]))) << id;
        // prange(): getseleph(SYS_GAL) != 0 ⇒ BGD_E1E5a, else BGD_E1E5b — of the first record, whatever was selected.
        EXPECT_NEAR(removed, rtk::kCLight * (sel != rtk::GalNavSelection::Any ? first.tgd[0] : first.tgd[1]), 1e-6) << id;
        if (sel == rtk::GalNavSelection::INav) {
          EXPECT_EQ(st.nav_type, 1) << id;
        }
        if (sel == rtk::GalNavSelection::FNav) {
          EXPECT_EQ(st.nav_type, 2) << id;
          EXPECT_NEAR(cb, removed, 1e-6) << id;   // F/NAV: prange's convention and the record agree
        }
        // "any" with an F/NAV record selected: prange() pairs the F/NAV clock with BGD_E1E5b — the mirror image of the
        // I/NAV case; the core keeps BGD_E1E5a (asserted above through sameBits).
        if (sel == rtk::GalNavSelection::Any && st.nav_type == 2 && std::fabs(cb - removed) > 1e-3) ++any_fnav_mismatch;
        if (so.sat.prn == 7 && sel == rtk::GalNavSelection::INav) {
          saw_e07 = true;
          EXPECT_NEAR(cb, 1.3262, 1e-4);       // c·BGD_E1E5b = c·4.423782229424e-9 s
          EXPECT_NEAR(removed, 0.8376, 1e-4);  // c·BGD_E1E5a = c·2.793967723846e-9 s (RTKLIB-EX 2.5.1)
        }
      } else {
        EXPECT_NEAR(cb, removed, 1e-6) << id << " cb " << cb << " prange " << removed;
        if (so.sat.sys == Sys::GLO) {
          EXPECT_NEAR(cb, -st.tgd_s[0] * rtk::kCLight / (gamma_glo - 1.0), 1e-9) << id;
          EXPECT_GT(cb, 1.0) << id;   // dtaun ≈ −2.5 ns ⇒ ≈ +1.1 m: the branch is really exercised
        }
      }
      ++n[so.sat.sys];
    }
  }
  rtk::setGalileoNavSelection(rtk::GalNavSelection::INav);
  EXPECT_TRUE(saw_e07);
  std::printf("[U5 BGD] 'any' selection: %d F/NAV-selected Galileo records where prange() removes another BGD\n", any_fnav_mismatch);
  for (Sys sys : {Sys::GPS, Sys::GLO, Sys::GAL, Sys::BDS, Sys::QZS}) EXPECT_GT(n[sys], 0) << sysName(sys);
  // Anything but the primary signal of the §12.3 scheme is refused loudly.
  rtk::SatState st;
  st.nav_type = 1;
  EXPECT_TRUE(std::isnan(rtk::codeBiasMetres(rtk::makeSatId(Sys::GPS, 7), st, SignalCode::L2X)));
  EXPECT_TRUE(std::isnan(rtk::codeBiasMetres(rtk::makeSatId(Sys::GAL, 7), st, SignalCode::L7X)));
  EXPECT_TRUE(std::isnan(rtk::codeBiasMetres(rtk::makeSatId(Sys::BDS, 19), st, SignalCode::L1C)));
  st.nav_type = 0;
  EXPECT_TRUE(std::isnan(rtk::codeBiasMetres(rtk::makeSatId(Sys::GAL, 7), st, SignalCode::L1X)));
}

// Review m4 (DESIGN §8): with no Klobuchar set the core's ionospheric delay is exactly 0 — not RTKLIB ionmodel()'s
// built-in 2004 set, which it substitutes for an all-zero one (metres); with a set it is ionmodel() bit for bit.
TEST(U05Corrections, IonoModelZeroWithoutParameters) {
  const Truth tr;
  const GTime t = ep(2025, 1, 3, 6, 0, 0);   // 14:00 local time in Shanghai: near the Klobuchar daytime peak
  const double pos[3] = {tr.geo.lat_rad, tr.geo.lon_rad, tr.geo.h_m};
  const Klobuchar none;
  ASSERT_FALSE(none.valid());
  Klobuchar k;
  k.alpha = {2.7940e-08, 7.4506e-09, -1.6689e-07, 1.1921e-07};
  k.beta = {1.5155e+05, -1.4746e+05, -2.6214e+05, 1.9661e+05};
  const double p[8] = {k.alpha[0], k.alpha[1], k.alpha[2], k.alpha[3], k.beta[0], k.beta[1], k.beta[2], k.beta[3]};
  const double zero[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  for (double el_deg : {10.0, 30.0, 60.0, 90.0}) {
    const double azel[2] = {1.0, el_deg * kD2R};
    EXPECT_EQ(rtk::ionModelGpsL1(t, none, tr.geo, azel[0], azel[1]), 0.0) << el_deg;
    EXPECT_GT(ionmodel(toG(t), zero, pos, azel), 1.0) << el_deg;   // the fallback kept out of the core
    EXPECT_TRUE(sameBits(rtk::ionModelGpsL1(t, k, tr.geo, azel[0], azel[1]), ionmodel(toG(t), p, pos, azel))) << el_deg;
  }
}

// Review m8: P2's RAIM branch. When estpos() fails on the full set and raim_fde() recovers by excluding one satellite,
// the exported x/xlin/Q (and estvel's) are those of the kept subset — bit-identical to a direct solve of that subset.
// The gross error sits on the epoch's first satellite: raim_fde's first exclusion starts, like pntpos, from rr = 0
// (sol_e is zero-initialised and only carried over between exclusions), so the two estpos() runs are the same.
TEST(U05Corrections, RaimFdeExportsKeptSubsetState) {
  REQUIRE_SNIPPET();
  REQUIRE_P2();
  rtk::setGalileoNavSelection(rtk::GalNavSelection::INav);
  rtk::NavStore nav;
  std::string why;
  ASSERT_GT(nav.loadRinexNav(brdcPath(), true, &why), 0) << why;
  Klobuchar ion;
  ASSERT_TRUE(nav.iono(&ion));
  const Truth tr;
  ObsEpoch e = makeEpoch(nav, ion, ep(2025, 1, 3, 6, 33, 21.0), tr, 63);
  std::stable_sort(e.sats.begin(), e.sats.end(), [](const SatObs& a, const SatObs& b) { return a.sat.rtk_sat < b.sat.rtk_sat; });
  ASSERT_GE(e.sats.size(), 8u);
  const int bad = e.sats.front().sat.rtk_sat;
  ObsEpoch subset = e;
  subset.sats.erase(subset.sats.begin());
  rtk::SppRefOptions plain_opt;
  plain_opt.use_glo_psr = true;
  rtk::SppRefOptions raim_opt = plain_opt;
  raim_opt.raim_fde = true;
  const rtk::SppRef spp;
  rtk::SppRefResult direct;
  ASSERT_TRUE(spp.solve(subset, nav, plain_opt, &direct)) << direct.msg;
  ASSERT_EQ(direct.x_pos.size(), 9);
  ASSERT_EQ(direct.x_vel.size(), 4);
  bool triggered = false;
  // estpos() only fails on divergence, rank or GDOP (the χ² test is disabled upstream, pntpos.c:384): look for the
  // smallest gross error that makes it fail on this epoch.
  for (double gross : {1e5, 1e6, 1e7, 3e7, 1e8}) {
    ObsEpoch f = e;
    f.sats.front().sig[0].psr_m += gross;
    rtk::SppRefResult plain, fde;
    spp.solve(f, nav, plain_opt, &plain);
    if (plain.ok) continue;
    EXPECT_EQ(plain.x_pos.size(), 0) << "a failed estpos() exports no state";
    EXPECT_EQ(plain.x_vel.size(), 0);
    ASSERT_TRUE(spp.solve(f, nav, raim_opt, &fde)) << "gross " << gross << ": " << fde.msg;
    for (const auto& r : fde.resp) EXPECT_NE(r.first.rtk_sat, bad) << "the gross satellite must be the excluded one";
    ASSERT_EQ(fde.resp.size(), direct.resp.size());
    EXPECT_TRUE(sameBits(fde.x_pos, direct.x_pos));
    EXPECT_TRUE(sameBits(fde.x_pos_lin, direct.x_pos_lin));
    EXPECT_TRUE(sameBits(fde.Q_pos, direct.Q_pos));
    EXPECT_TRUE(sameBits(fde.x_vel, direct.x_vel));
    EXPECT_TRUE(sameBits(fde.x_vel_lin, direct.x_vel_lin));
    EXPECT_TRUE(sameBits(fde.Q_vel, direct.Q_vel));
    for (int i = 0; i < 3; ++i) EXPECT_TRUE(sameBits(fde.r_ecef[i], direct.r_ecef[i]));
    for (size_t j = 0; j < fde.resp.size(); ++j) EXPECT_TRUE(sameBits(fde.resp[j].second, direct.resp[j].second));
    EXPECT_LT((fde.r_ecef - tr.r).norm(), 1e-3);   // and it is the right answer
    triggered = true;
    std::printf("[U5 RAIM] gross error %.0e m on %s made estpos() fail (%s); RAIM kept %zu satellites\n", gross,
                rtk::satIdString(bad).c_str(), plain.msg.c_str(), fde.resp.size());
    break;
  }
  ASSERT_TRUE(triggered) << "no gross error made estpos() fail: the RAIM branch was not exercised";
}
