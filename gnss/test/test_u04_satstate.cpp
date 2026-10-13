// SPDX-License-Identifier: BSD-3-Clause
// test_u04_satstate.cpp — TEST_PLAN §1 U4 (two tiers, AMENDMENTS A9-3; DESIGN §1 "发射时刻", §4.3, §4.5). Owner: WP-a.
// Tier 1 (same data, two code paths, bit-identical) runs on the IGS BRDC snippet (see test_u03_satmap.cpp for its
// provenance; $LIVO_GNSS_BRDC_DIR, default /r/data/brdc; sha256-pinned in third_party/rtklib/testing/wpa_test_support.hpp).
// Tier 2 (bag vs BRDC) needs the dev-set fixture (A24; found by gnss_io/test/fixture_discovery.hpp, A39).
#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dirent.h>
#include <fstream>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

#include "../../gnss_io/test/fixture_discovery.hpp"   // A24/A39: the fixture discovery rule shared with WP-e
#include "../../third_party/rtklib/testing/wpa_test_support.hpp"
#include "internal/ephem_store.hpp"
#include "internal/rtk_adapter.hpp"
#include "livo_gnss/gnss_fusion.hpp"
#include "livo_gnss_io/gnss_comm_mapping.hpp"
#include "rtklib.h"   // WP-a test files may include it (OWNERSHIP §1): satposs()/satpos() are the reference

using namespace livo_gnss;

namespace {

using livo_wpa_testing::brdcPath;
#define REQUIRE_SNIPPET() LIVO_REQUIRE_BRDC_SNIPPET()

GTime ep(double y, double mo, double d, double h, double mi, double s) {
  const double e[6] = {y, mo, d, h, mi, s};
  return rtk::epoch2time(e);
}
gtime_t toG(GTime t) {
  gtime_t g;
  g.time = static_cast<time_t>(t.sec);
  g.sec = t.frac;
  return g;
}
bool same(double a, double b) { return std::memcmp(&a, &b, sizeof(double)) == 0; }
bool same(const Eigen::Vector3d& a, const double* b) { return same(a[0], b[0]) && same(a[1], b[1]) && same(a[2], b[2]); }
bool same(const Eigen::Vector3d& a, const Eigen::Vector3d& b) { return same(a[0], b[0]) && same(a[1], b[1]) && same(a[2], b[2]); }
bool same(const GTime& a, const GTime& b) { return a.sec == b.sec && same(a.frac, b.frac); }

// Receiver used to give the satellites realistic pseudoranges (Shanghai; BDS GEO/IGSO are visible there).
constexpr double kD2R = 3.1415926535897932 / 180.0;
const Geodetic kRcv{31.0283 * kD2R, 121.4413 * kD2R, 12.0};

// Evaluation instants inside the snippet (04:00–09:00 GPST), away from ephemeris-selection boundaries.
std::vector<GTime> evalTimes() { return {ep(2025, 1, 3, 4, 47, 12.3), ep(2025, 1, 3, 6, 3, 21.25), ep(2025, 1, 3, 7, 58, 44.8)}; }

std::vector<SatId> storeSats(const rtk::NavStore& nav) {
  std::set<int> s;
  for (const rtk::EphKey& k : nav.coverage()) s.insert(k.sat.rtk_sat);
  std::vector<SatId> v;
  for (int n : s) {
    SatId id;
    id.rtk_sat = n;
    id.sys = rtk::satSys(n, &id.prn);
    v.push_back(id);
  }
  return v;
}

// Pseudorange of a satellite seen from kRcv with a 300 m receiver clock (three fixed-point passes on t_tx).
double rangeFor(const rtk::SatSolver& s, const SatId& sat, GTime t_rx) {
  const Eigen::Vector3d rr = rtk::pos2ecef(kRcv);
  double P = 2.2e7;
  for (int i = 0; i < 3; ++i) {
    rtk::SatState st;
    if (!s.satState(sat, t_rx, P, &st)) return 0.0;
    P = rtk::geodist(st.r_ecef, rr, nullptr) + 300.0;
  }
  return P;
}

// Restores this RTKLIB copy's global Galileo selection on scope exit (also when an ASSERT returns early).
struct GalSelectionGuard {
  rtk::GalNavSelection saved;
  explicit GalSelectionGuard(rtk::GalNavSelection s) : saved(rtk::galileoNavSelection()) { rtk::setGalileoNavSelection(s); }
  ~GalSelectionGuard() { rtk::setGalileoNavSelection(saved); }
  GalSelectionGuard(const GalSelectionGuard&) = delete;
  GalSelectionGuard& operator=(const GalSelectionGuard&) = delete;
};

enum class BdsClass { None, Geo, Igso, Meo };
BdsClass bdsClass(int prn) {
  if (prn <= 5 || prn >= 59) return BdsClass::Geo;   // ephemeris.c:265
  if ((prn >= 6 && prn <= 10) || prn == 13 || prn == 16 || prn == 31 || (prn >= 38 && prn <= 40) || prn == 56) return BdsClass::Igso;
  return BdsClass::Meo;
}

// The reference nav_t: RTKLIB's own readrnxt of the snippet, reduced with the store's documented rules (key =
// sat + toe rounded to the second + Galileo I/NAV|F/NAV; first record in file order wins; sorted by key). RTKLIB
// satposs()/satpos() on it are the independent path the adapter has to reproduce bit for bit.
struct RefNav {
  nav_t nav;
  RefNav() { std::memset(&nav, 0, sizeof(nav)); }
  ~RefNav() { freenav(&nav, 0xFF); }
  bool load(const std::string& path) {
    nav_t raw;
    std::memset(&raw, 0, sizeof(raw));
    gtime_t t0;
    t0.time = 0;
    t0.sec = 0.0;
    if (readrnxt(path.c_str(), 1, t0, t0, 0.0, "-SYS=GRECJ", nullptr, &raw, nullptr) <= 0) return false;
    auto key = [](const eph_t& e) {
      const int nt = satsys(e.sat, nullptr) == SYS_GAL ? ((e.code & (1 << 9)) ? 1 : (e.code & (1 << 8)) ? 2 : 0) : 0;
      return std::make_tuple(e.sat, static_cast<int64_t>(e.toe.time) + (e.toe.sec >= 0.5 ? 1 : 0), nt);
    };
    std::map<std::tuple<int, int64_t, int>, eph_t> ke;
    for (int i = 0; i < raw.n; ++i)
      if (raw.eph[i].sat > 0) ke.emplace(key(raw.eph[i]), raw.eph[i]);   // emplace keeps the first
    std::map<std::pair<int, int64_t>, geph_t> kg;
    for (int i = 0; i < raw.ng; ++i)
      if (raw.geph[i].sat > 0)
        kg.emplace(std::make_pair(raw.geph[i].sat, static_cast<int64_t>(raw.geph[i].toe.time) + (raw.geph[i].toe.sec >= 0.5 ? 1 : 0)), raw.geph[i]);
    nav.eph = static_cast<eph_t*>(std::malloc(sizeof(eph_t) * ke.size()));
    nav.geph = static_cast<geph_t*>(std::malloc(sizeof(geph_t) * kg.size()));
    for (const auto& kv : ke) nav.eph[nav.n++] = kv.second;
    for (const auto& kv : kg) nav.geph[nav.ng++] = kv.second;
    nav.nmax = nav.n;
    nav.ngmax = nav.ng;
    for (int i = 0; i < 8; ++i) nav.ion_gps[i] = raw.ion_gps[i];
    freenav(&raw, 0xFF);
    return true;
  }
};

}  // namespace

// Tier 1 (same data, two code paths, bit-identical): one ephemeris read from the RINEX fixture evaluated via
// SatSolver::ephPos and via a frozen EphSnapshot (satStateFromSnapshot) gives identical rs/dts bits
TEST(U04SatState, SamePathsBitIdenticalDirectVsSnapshot) {
  REQUIRE_SNIPPET();
  rtk::setGalileoNavSelection(rtk::GalNavSelection::INav);
  rtk::NavStore nav;
  std::string why;
  ASSERT_GT(nav.loadRinexNav(brdcPath(), true, &why), 0) << why;
  const rtk::SatSolver solver(nav);
  std::map<Sys, int> n_sys;
  std::vector<std::tuple<rtk::EphSnapshot, rtk::SatState, GTime, double>> kept;
  for (const GTime& t : evalTimes()) {
    for (const SatId& sat : storeSats(nav)) {
      const double P = rangeFor(solver, sat, t);
      if (P == 0.0) continue;
      rtk::SatState st, st2;
      rtk::EphSnapshot snap;
      ASSERT_TRUE(solver.satState(sat, t, P, &st, &snap));
      ASSERT_TRUE(snap.valid());
      EXPECT_EQ(snap.sat().rtk_sat, sat.rtk_sat);
      EXPECT_TRUE(same(snap.toe(), st.toe));
      ASSERT_TRUE(rtk::SatSolver::satStateFromSnapshot(snap, t, P, &st2));
      const std::string id = rtk::satIdString(sat.rtk_sat);
      EXPECT_TRUE(same(st.r_ecef, st2.r_ecef)) << id;
      EXPECT_TRUE(same(st.v_ecef, st2.v_ecef)) << id;
      EXPECT_TRUE(same(st.dts, st2.dts)) << id;
      EXPECT_TRUE(same(st.ddts, st2.ddts)) << id;
      EXPECT_TRUE(same(st.t_tx, st2.t_tx)) << id;
      EXPECT_TRUE(same(st.var_eph, st2.var_eph)) << id;
      EXPECT_TRUE(same(st.tgd_s[0], st2.tgd_s[0]) && same(st.tgd_s[1], st2.tgd_s[1])) << id;
      EXPECT_EQ(st.svh, st2.svh);
      EXPECT_EQ(st.iode, st2.iode);
      EXPECT_EQ(st.sva, st2.sva);
      EXPECT_EQ(st.nav_type, st2.nav_type);
      // Direct eph2pos/geph2pos at the transmit time selects the same record here and gives the same bits.
      Eigen::Vector3d rs;
      double dts = 0.0, var = 0.0;
      ASSERT_TRUE(solver.ephPos(sat, st.t_tx, &rs, &dts, &var)) << id;
      EXPECT_TRUE(same(rs, st.r_ecef)) << id;
      EXPECT_TRUE(same(dts, st.dts)) << id;
      EXPECT_TRUE(same(var, st.var_eph)) << id;
      ++n_sys[sat.sys];
      if (kept.size() < 400) kept.emplace_back(snap, st, t, P);
    }
  }
  for (Sys s : {Sys::GPS, Sys::GLO, Sys::GAL, Sys::BDS, Sys::QZS}) EXPECT_GT(n_sys[s], 0) << sysName(s);
  // TDCP k0/k1 (§4.5, R2-11): the snapshot is a deep copy — clearing the store (or any later insertion) does not
  // change what it evaluates, and a copied snapshot evaluates identically.
  nav.clear();
  EXPECT_EQ(nav.countGeph(), 0);
  for (const auto& k : kept) {
    const rtk::EphSnapshot copy = std::get<0>(k);
    rtk::EphSnapshot assigned;
    assigned = copy;
    rtk::SatState a, b;
    ASSERT_TRUE(rtk::SatSolver::satStateFromSnapshot(copy, std::get<2>(k), std::get<3>(k), &a));
    ASSERT_TRUE(rtk::SatSolver::satStateFromSnapshot(assigned, std::get<2>(k), std::get<3>(k), &b));
    EXPECT_TRUE(same(a.r_ecef, std::get<1>(k).r_ecef));
    EXPECT_TRUE(same(a.dts, std::get<1>(k).dts));
    EXPECT_TRUE(same(b.r_ecef, a.r_ecef));
  }
  rtk::SatState none;
  EXPECT_FALSE(rtk::SatSolver::satStateFromSnapshot(rtk::EphSnapshot(), ep(2025, 1, 3, 6, 0, 0), 2.2e7, &none));
  EXPECT_FALSE(none.ok);
}

// Tier 2 (bag ephemeris vs same-day BRDC for the same (sat, toe), RINEX precision): |Δr| < 1e-3 m, |Δdts| < 1e-11 s.
// Fixture layout (A24): <data>/<seq>_<t0>_<D>s/{msgdef,bytes,golden,brdc}, found by WP-e's fixture_discovery.hpp (A39).
// The bag ephemerides come from the fixture bytes through WP-e's decoder; the BRDC file of the slice is the same-day IGS
// merge. A slice without ephemeris topics (A26: Occlusion03/04, Dynamic03) contributes nothing.
namespace {

struct Tier2Count {
  int keys = 0, matched = 0, compared = 0;
  std::string unmatched;
};

// One slice. `had_eph` = the slice carries ephemeris topics; per-system counts are added to `total`.
void bagVsRinexTier2(const livo_wpa_testing::Slice& sl, bool* had_eph, std::map<Sys, Tier2Count>* total) {
  *had_eph = false;
  struct Sink : MeasurementSink {
    std::vector<KeplerEph> eph;
    std::vector<GloEph> geph;
    void pushObservation(ObsEpoch&&) override {}
    void pushEphemeris(const KeplerEph& e) override { eph.push_back(e); }
    void pushGloEphemeris(const GloEph& g) override { geph.push_back(g); }
    void pushIono(const Klobuchar&) override {}
    void pushPvt(const ReceiverPvt&) override {}
  };
  const std::string& slice = sl.path;
  if (livo_wpa_testing::findFile(slice + "/bytes", "ephem", ".bin", "glo").empty()) {
    std::printf("[U4 tier 2] %s: no ephem topic in the slice\n", sl.name.c_str());
    return;
  }
  Sink sink;
  livo_wpa_testing::DecodeStats ds;
  try {
    livo_gnss_io::GnssCommRos1Decoder dec(&satNoOf);
    std::string why;
    ASSERT_TRUE(livo_wpa_testing::decodeChannel(dec, slice, livo_wpa_testing::kEphemChannel, sink, &ds, &why)) << why;
    ASSERT_TRUE(livo_wpa_testing::decodeChannel(dec, slice, livo_wpa_testing::kGloEphemChannel, sink, &ds, &why, false)) << why;
  } catch (const std::logic_error& e) {
    LIVO_BLOCKED_IF_STUB(e, "WP-e merge (gnss_io GnssCommRos1Decoder)");
  }
  std::printf("[U4 tier 2] %s: %d ephemeris messages, %d refused by the decoder%s%s\n", sl.name.c_str(), ds.messages, ds.refused,
              ds.refused ? ", first: " : "", ds.first_refusal.c_str());
  ASSERT_FALSE(sink.eph.empty()) << slice << ": no GnssEphemMsg decoded";
  *had_eph = true;
  Config cfg;
  cfg.ephem.ura_semantics = Config::Ephem::UraSemantics::Index;   // positions/clocks do not depend on sva
  rtk::NavStore bag_nav, rnx_nav;
  EphemStore bag(cfg, bag_nav);
  for (const KeplerEph& e : sink.eph) bag.addBag(e);
  for (const GloEph& g : sink.geph) bag.addBag(g);
  const std::string brdc = livo_gnss_io::testsupport::fixtureBrdc(slice);
  ASSERT_FALSE(brdc.empty()) << slice << ": A24 brdc/ missing";
  std::string why;
  ASSERT_GT(rnx_nav.loadRinexNav(brdc, true, &why), 0) << why;
  // Every bag key is accounted for (review M2): a wrong toe/week/time-scale convention of the gnss_comm path (BDS 14 s,
  // GST week, GLONASS UTC) moves whole systems out of the BRDC key space, which a "skip what does not match" loop
  // would pass silently. Per system: bag keys, keys found in BRDC, keys compared.
  std::map<Sys, Tier2Count> per;
  std::map<BdsClass, int> bds_keys, bds_compared;
  // Each record is evaluated alone below: the Galileo data-source filter of the global selection (A16; the EphemStore
  // constructor set I/NAV) must not hide a matched F/NAV record on both sides — only the AOD rule stays in force.
  const GalSelectionGuard any_source(rtk::GalNavSelection::Any);
  for (const rtk::EphKey& k : bag_nav.coverage()) {
    const std::string id = rtk::satIdString(k.sat.rtk_sat) + " toe " + rtk::timeStr(k.toe, 0) + " nav_type " + std::to_string(k.nav_type);
    Tier2Count& c = per[k.sat.sys];
    ++c.keys;
    const BdsClass cls = k.sat.sys == Sys::BDS ? bdsClass(k.sat.prn) : BdsClass::None;
    if (cls != BdsClass::None) ++bds_keys[cls];
    if (!(k.is_glo ? rnx_nav.has(k.sat, k.toe) : rnx_nav.has(k.sat, k.toe, k.nav_type))) {
      c.unmatched += "\n  " + id;
      continue;
    }
    ++c.matched;
    // Evaluate each record alone (a one-record store) so the selection cannot pick a different toe on either side.
    rtk::NavStore one_b, one_r;
    if (k.is_glo) {
      GloEph gb, gr;
      ASSERT_TRUE(bag_nav.get(k.sat, k.toe, &gb));
      ASSERT_TRUE(rnx_nav.get(k.sat, k.toe, &gr));
      one_b.add(gb, true);
      one_r.add(gr, true);
    } else {
      KeplerEph eb, er;
      ASSERT_TRUE(bag_nav.get(k.sat, k.toe, &eb, k.nav_type));
      ASSERT_TRUE(rnx_nav.get(k.sat, k.toe, &er, k.nav_type));
      one_b.add(eb, true);
      one_r.add(er, true);
    }
    int evaluated = 0;
    for (double dt : {1.0, 600.0, 1500.0}) {   // after toe (Galileo AOD rule), within every system's age limit
      Eigen::Vector3d rb, rr;
      double db = 0, dr = 0, vb = 0, vr = 0;
      const bool ok_b = rtk::SatSolver(one_b).ephPos(k.sat, k.toe.add(dt), &rb, &db, &vb);
      const bool ok_r = rtk::SatSolver(one_r).ephPos(k.sat, k.toe.add(dt), &rr, &dr, &vr);
      EXPECT_EQ(ok_b, ok_r) << id << " dt " << dt << ": only one side evaluates";
      if (!ok_b || !ok_r) continue;
      EXPECT_LT((rb - rr).norm(), 1e-3) << id << " dt " << dt;
      EXPECT_LT(std::fabs(db - dr), 1e-11) << id << " dt " << dt;
      ++evaluated;
    }
    EXPECT_EQ(evaluated, 3) << id << ": a matched record that cannot be evaluated";
    if (evaluated > 0) {
      ++c.compared;
      if (cls != BdsClass::None) ++bds_compared[cls];
    }
  }
  for (const auto& kv : per) {
    const Tier2Count& c = kv.second;
    EXPECT_GT(c.compared, 0) << slice << ": none of the " << c.keys << " " << sysName(kv.first)
                             << " bag keys matched the same-day BRDC (toe/week/time-scale convention?)" << c.unmatched;
    EXPECT_LE(10 * (c.keys - c.matched), c.keys) << slice << ": more than 10 % of the " << sysName(kv.first)
                                                 << " bag keys are missing from BRDC" << c.unmatched;
    std::printf("[U4 tier 2] %s %s: %d bag keys, %d in BRDC, %d compared%s%s\n", sl.name.c_str(), sysName(kv.first), c.keys, c.matched,
                c.compared, c.unmatched.empty() ? "" : "; not in BRDC:", c.unmatched.c_str());
    Tier2Count& t = (*total)[kv.first];
    t.keys += c.keys;
    t.matched += c.matched;
    t.compared += c.compared;
  }
  for (const auto& kv : bds_keys)
    std::printf("[U4 tier 2] %s BDS %s: %d keys, %d compared\n", sl.name.c_str(),
                kv.first == BdsClass::Geo ? "GEO" : kv.first == BdsClass::Igso ? "IGSO" : "MEO", kv.second, bds_compared[kv.first]);
  // TEST_PLAN U4: BDS MEO/IGSO/GEO each exercised when the bag carries them (tier 1 covers all three on the BRDC snippet).
  for (const auto& kv : bds_keys) EXPECT_GT(bds_compared[kv.first], 0) << slice << ": BDS orbit class " << static_cast<int>(kv.first) << " never compared";
}

// #6c: GPS, Galileo, BeiDou and GLONASS each compared over the slices of a test — a system the gnss_comm path loses as a
// whole (e.g. a BDS toe convention error, or a decoder that drops the system) fails here instead of passing silently.
void expectEverySystemCompared(const std::map<Sys, Tier2Count>& total, const char* what) {
  for (Sys s : {Sys::GPS, Sys::GAL, Sys::BDS, Sys::GLO}) {
    const auto it = total.find(s);
    const int n = it == total.end() ? 0 : it->second.compared;
    EXPECT_GT(n, 0) << what << ": no " << sysName(s) << " bag ephemeris compared with the same-day BRDC";
  }
}

}  // namespace

// Gate (A24/A39): dev-set slices only, one of them with ephemeris topics (A26: Grass01/02).
TEST(U04SatState, BagVsRinexWithinRinexPrecision) {
  LIVO_DEVSET_SLICES(slices);
  bool any_eph = false;
  std::map<Sys, Tier2Count> total;
  for (const livo_wpa_testing::Slice& sl : slices) {
    bool had = false;
    bagVsRinexTier2(sl, &had, &total);
    if (HasFatalFailure() || IsSkipped()) return;
    any_eph = any_eph || had;
  }
  if (!any_eph)
    GTEST_SKIP() << "blocked: dev-set fixture with ephemeris topics (A26: only Grass01/02 carry them; dev-set slices:"
                 << livo_wpa_testing::namesOf(slices, livo_wpa_testing::SliceKind::DevSet) << ")";
  expectEverySystemCompared(total, "dev-set slices");
}

// The same on WP-e's synthetic slices: PASS(合成) only (A24).
TEST(U04SatState, BagVsRinexWithinRinexPrecisionSyntheticSlice) {
  LIVO_SYNTHETIC_SLICES(slices);
  bool any_eph = false;
  std::map<Sys, Tier2Count> total;
  for (const livo_wpa_testing::Slice& sl : slices) {
    bool had = false;
    bagVsRinexTier2(sl, &had, &total);
    if (HasFatalFailure() || IsSkipped()) return;
    any_eph = any_eph || had;
  }
  ASSERT_TRUE(any_eph) << "synthetic slices without ephemeris topics";
  expectEverySystemCompared(total, "synthetic slices");
  std::printf("[U4 tier 2] synthetic slices: PASS(合成) only (A24)\n");
}

// at least one BDS MEO, IGSO and GEO (C59–C62) satellite is exercised (toes = BDT seconds; 27 km if wrong)
TEST(U04SatState, CoversBdsMeoIgsoGeo) {
  REQUIRE_SNIPPET();
  if (!(rtk::patchLevel() & rtk::kPatchP1)) GTEST_SKIP() << "unpatched reference build: C59+ unmappable (P1)";
  rtk::NavStore nav;
  std::string why;
  ASSERT_GT(nav.loadRinexNav(brdcPath(), true, &why), 0) << why;
  const rtk::SatSolver solver(nav);
  std::map<BdsClass, int> n;
  int n_c59 = 0;
  const GTime t = ep(2025, 1, 3, 6, 3, 21.25);
  for (const SatId& sat : storeSats(nav)) {
    if (sat.sys != Sys::BDS) continue;
    const double P = rangeFor(solver, sat, t);
    if (P == 0.0) continue;
    rtk::SatState st;
    ASSERT_TRUE(solver.satState(sat, t, P, &st));
    const std::string id = rtk::satIdString(sat.rtk_sat);
    const double r = st.r_ecef.norm(), v = st.v_ecef.norm();
    const BdsClass c = bdsClass(sat.prn);
    // Geosynchronous orbits (GEO and IGSO) have a ≈ 42164 km (IGSO e up to ~0.01), MEO ≈ 27906 km.
    if (c == BdsClass::Meo) {
      EXPECT_NEAR(r, 27906e3, 300e3) << id;
    } else {
      EXPECT_NEAR(r, 42164e3, 600e3) << id;
    }
    if (c == BdsClass::Geo) {
      // Nearly fixed in ECEF: the residual inclination (≤ ~5°) gives a daily N–S swing of r·ω_e·sin i ≤ 270 m/s,
      // while an IGSO figure-8 runs at 1.3–2.8 km/s.
      EXPECT_LT(v, 400.0) << id;
      if (sat.prn >= 59) ++n_c59;
      // The GEO branch of eph2pos (ephemeris.c:265, prn ≤ 5 || prn ≥ 59: elements in a frame tilted by −5°) is the one
      // taken: the same record evaluated as a non-GEO PRN (standard branch) lands thousands of km away.
      KeplerEph rec;
      ASSERT_TRUE(nav.get(sat, st.toe, &rec));
      rec.sat = rtk::makeSatId(Sys::BDS, 20);
      rtk::NavStore swapped;
      ASSERT_TRUE(swapped.add(rec, true));
      Eigen::Vector3d r_geo, r_std;
      double d1 = 0.0, d2 = 0.0, v1 = 0.0, v2 = 0.0;
      ASSERT_TRUE(solver.ephPos(sat, t, &r_geo, &d1, &v1));
      ASSERT_TRUE(rtk::SatSolver(swapped).ephPos(rec.sat, t, &r_std, &d2, &v2));
      EXPECT_GT((r_geo - r_std).norm(), 1000e3) << id;
      EXPECT_NEAR(r_geo.norm(), r_std.norm(), 1.0) << id;   // a rotation: same radius
      EXPECT_EQ(d1, d2) << id;                              // the clock does not depend on the branch
    }
    if (c == BdsClass::Igso) {
      EXPECT_GT(v, 1000.0) << id;
    }
    // toes in BDT seconds (R2-3): the store's record carries exactly what EphemStore derives from toe.
    KeplerEph rec;
    ASSERT_TRUE(nav.get(sat, st.toe, &rec));
    EXPECT_EQ(rec.toes, EphemStore::toesFor(Sys::BDS, rec.toe)) << id;
    ++n[c];
  }
  EXPECT_GT(n[BdsClass::Meo], 0);
  EXPECT_GT(n[BdsClass::Igso], 0);
  EXPECT_GT(n[BdsClass::Geo], 0);
  EXPECT_GT(n_c59, 0) << "no C59-C63 exercised (the 2025-01-03 BRDC has C59/C60/C62)";
}

// at least one Galileo (I/NAV selected, A16) and one GLONASS satellite is exercised
TEST(U04SatState, CoversGalileoAndGlonass) {
  REQUIRE_SNIPPET();
  rtk::NavStore nav;
  std::string why;
  ASSERT_GT(nav.loadRinexNav(brdcPath(), true, &why), 0) << why;
  const rtk::SatSolver solver(nav);
  rtk::setGalileoNavSelection(rtk::GalNavSelection::INav);
  const GTime t = ep(2025, 1, 3, 6, 3, 21.25);
  int n_gal = 0, n_glo = 0;
  for (const SatId& sat : storeSats(nav)) {
    if (sat.sys != Sys::GAL && sat.sys != Sys::GLO) continue;
    const double P = rangeFor(solver, sat, t);
    if (P == 0.0) continue;
    rtk::SatState st;
    ASSERT_TRUE(solver.satState(sat, t, P, &st));
    const std::string id = rtk::satIdString(sat.rtk_sat);
    if (sat.sys == Sys::GAL) {
      EXPECT_EQ(st.nav_type, 1) << id;                    // I/NAV (code bit 9)
      EXPECT_LT(st.toe.diff(st.t_tx), 0.0) << id;         // AOD > 0: toe strictly before the evaluation time
      if (sat.prn != 14 && sat.prn != 18) {   // E14/E18: eccentric (e = 0.16)
        EXPECT_NEAR(st.r_ecef.norm(), 29600e3, 300e3) << id;
      }
      ++n_gal;
    } else {
      EXPECT_NEAR(st.r_ecef.norm(), 25510e3, 300e3) << id;
      // 3.95 km/s inertial at 64.8° inclination minus ω_e × r: 3.1–3.6 km/s in ECEF.
      EXPECT_GT(st.v_ecef.norm(), 2500.0) << id;
      EXPECT_LT(st.v_ecef.norm(), 4500.0) << id;
      EXPECT_LE(std::fabs(st.toe.diff(t)), 900.0) << id;   // 30-min records, closest one
      EXPECT_EQ(st.var_eph, 25.0) << id;
      ++n_glo;
    }
  }
  EXPECT_GT(n_gal, 3);
  EXPECT_GT(n_glo, 3);

  // Galileo selection rules on E07 (records every 10 min; toe 06:00 exists as I/NAV and F/NAV).
  const SatId e07 = rtk::makeSatId(Sys::GAL, 7);
  const GTime toe = ep(2025, 1, 3, 6, 0, 0);
  ASSERT_TRUE(nav.has(e07, toe, 1));
  ASSERT_TRUE(nav.has(e07, toe, 2));
  rtk::SatState a, b, c;
  // t_rx exactly at a toe (P = 0 keeps t_tx = t_rx − dts ≈ t_rx): that record is not yet usable (AOD ≤ 0).
  ASSERT_TRUE(solver.satState(e07, toe, 0.0, &a));
  EXPECT_EQ(a.toe.diff(toe), -600.0);
  ASSERT_TRUE(solver.satState(e07, toe.add(1.0), 0.0, &b));
  EXPECT_EQ(b.toe.diff(toe), 0.0);
  EXPECT_EQ(b.nav_type, 1);
  rtk::setGalileoNavSelection(rtk::GalNavSelection::FNav);
  ASSERT_TRUE(solver.satState(e07, toe.add(1.0), 0.0, &c));
  EXPECT_EQ(c.nav_type, 2);
  EXPECT_EQ(c.toe.diff(toe), 0.0);
  // I/NAV (E5b/E1) and F/NAV (E5a/E1) clocks differ by the inter-frequency offset, ≈ 1.5 ns for E07 that day.
  EXPECT_GT(std::fabs(c.dts - b.dts), 1e-10);
  EXPECT_LT(std::fabs(c.dts - b.dts), 1e-8);
  // Same orbit parameters in both records; t_tx differs by the clock difference only (1.5 ns · 3.9 km/s).
  EXPECT_LT((b.r_ecef - c.r_ecef).norm(), 1e-4);
  rtk::setGalileoNavSelection(rtk::GalNavSelection::Any);
  ASSERT_TRUE(solver.satState(e07, toe.add(1.0), 0.0, &c));
  EXPECT_EQ(c.nav_type, 2);   // tie on |t − toe|: the later array entry (F/NAV sorts after I/NAV), as seleph does
  rtk::setGalileoNavSelection(rtk::GalNavSelection::INav);
}

// t_tx = t_rx − P/c − δt_s differs from satposs internal time by < 1e-9 s
TEST(U04SatState, TransmitTimeMatchesSatposs) {
  REQUIRE_SNIPPET();
  rtk::setGalileoNavSelection(rtk::GalNavSelection::INav);
  rtk::NavStore nav;
  std::string why;
  ASSERT_GT(nav.loadRinexNav(brdcPath(), true, &why), 0) << why;
  RefNav ref;
  ASSERT_TRUE(ref.load(brdcPath()));
  const rtk::SatSolver solver(nav);
  std::map<Sys, int> n;
  for (const GTime& t : evalTimes()) {
    for (const SatId& sat : storeSats(nav)) {
      const double P = rangeFor(solver, sat, t);
      if (P == 0.0) continue;
      const std::string id = rtk::satIdString(sat.rtk_sat);
      obsd_t o;
      std::memset(&o, 0, sizeof(o));
      o.time = toG(t);
      o.sat = static_cast<uint8_t>(sat.rtk_sat);
      o.P[0] = P;
      double rs[6], dts[2], var[1];
      int svh[1];
      satposs(toG(t), &o, 1, &ref.nav, EPHOPT_BRDC, rs, dts, var, svh);
      rtk::SatState st;
      ASSERT_TRUE(solver.satState(sat, t, P, &st)) << id;
      ASSERT_NE(svh[0], -1) << id << ": RTKLIB found no ephemeris where the adapter did";
      // satposs() and the adapter: same record, same arithmetic → same bits.
      EXPECT_TRUE(same(st.r_ecef, rs)) << id;
      EXPECT_TRUE(same(st.v_ecef, rs + 3)) << id;
      EXPECT_TRUE(same(st.dts, dts[0])) << id;
      EXPECT_TRUE(same(st.ddts, dts[1])) << id;
      EXPECT_EQ(st.svh, svh[0]) << id;
      EXPECT_TRUE(same(st.var_eph, var[0])) << id;   // self-computed σ² equals var_uraeph for valid indices
      // satposs's internal time[i] (ephemeris.c:804/:822) is the time at which satpos() reproduces its output; any
      // difference above ~1e-12 s would move the 3.9 km/s satellite by more than one ulp of its coordinates.
      double rs2[6], dts2[2], var2;
      int svh2;
      ASSERT_TRUE(satpos(toG(st.t_tx), toG(t), sat.rtk_sat, EPHOPT_BRDC, &ref.nav, rs2, dts2, &var2, &svh2)) << id;
      EXPECT_TRUE(same(st.r_ecef, rs2)) << id;
      EXPECT_TRUE(same(st.dts, dts2[0])) << id;
      // And explicitly: t_tx = (t_rx − P/c) − δt_s with δt_s = eph2clk/geph2clk of the selected record (no relativity).
      const gtime_t t1 = timeadd(toG(t), -P / CLIGHT);
      double dt = 0.0;
      bool found = false;
      if (sat.sys == Sys::GLO) {
        for (int i = 0; i < ref.nav.ng && !found; ++i)
          if (ref.nav.geph[i].sat == sat.rtk_sat && timediff(ref.nav.geph[i].toe, toG(st.toe)) == 0.0) {
            dt = geph2clk(t1, &ref.nav.geph[i]);
            found = true;
          }
      } else {
        for (int i = 0; i < ref.nav.n && !found; ++i) {
          const eph_t& e = ref.nav.eph[i];
          const int nt = sat.sys == Sys::GAL ? ((e.code & (1 << 9)) ? 1 : (e.code & (1 << 8)) ? 2 : 0) : 0;
          if (e.sat == sat.rtk_sat && timediff(e.toe, toG(st.toe)) == 0.0 && nt == st.nav_type) {
            dt = eph2clk(t1, &e);
            found = true;
          }
        }
      }
      ASSERT_TRUE(found) << id;
      EXPECT_LT(std::fabs(timediff(toG(st.t_tx), timeadd(t1, -dt))), 1e-9) << id;
      EXPECT_GT(std::fabs(timediff(toG(st.t_tx), t1)), 0.0) << id;   // δt_s is really applied (R2-2)
      ++n[sat.sys];
    }
  }
  for (Sys s : {Sys::GPS, Sys::GLO, Sys::GAL, Sys::BDS, Sys::QZS}) EXPECT_GT(n[s], 0) << sysName(s);
}

// L0 health/age masks of EphemStore (DESIGN §10 L0, A15 per-system keys incl. QZSS 'J') on the real records.
TEST(U04SatState, HealthAndAgeMasks) {
  REQUIRE_SNIPPET();
  Config cfg;
  cfg.ephem.ura_semantics = Config::Ephem::UraSemantics::Metres;
  rtk::NavStore nav;
  EphemStore es(cfg, nav);
  std::string why;
  ASSERT_GT(es.loadRinex(brdcPath(), &why), 0) << why;
  const GTime t = ep(2025, 1, 3, 6, 3, 21.25);
  std::map<Sys, int> healthy;
  for (const SatId& sat : storeSats(nav)) {
    double age = -1.0;
    const bool h = es.healthy(sat, t, &age);
    const double lim = cfg.masks.eph_age_s.at(sysChar(sat.sys));
    if (h) {
      EXPECT_LE(age, lim) << rtk::satIdString(sat.rtk_sat);
      ++healthy[sat.sys];
    }
  }
  for (Sys s : {Sys::GPS, Sys::GLO, Sys::GAL, Sys::BDS}) EXPECT_GT(healthy[s], 4) << sysName(s);
  // QZSS: every record of the day has svh = 1 (bit 0, L1C/B-LEX), masked like RTKLIB satexclude.
  EXPECT_GT(healthy[Sys::QZS], 0);
  const SatId j02 = rtk::makeSatId(Sys::QZS, 194);
  KeplerEph jr;
  bool found = false;
  for (const rtk::EphKey& k : nav.coverage())
    if (!found && k.sat.rtk_sat == j02.rtk_sat) found = nav.get(k.sat, k.toe, &jr);
  ASSERT_TRUE(found);
  EXPECT_EQ(jr.svh, 1);
  EXPECT_GE(es.nSatsHealthy(t), 5);   // WAIT_NAV (§8)
  // Beyond the per-system age limit: GLONASS 1800 s (RTKLIB selgeph has the same limit), BDS 3600 s < RTKLIB's 21601 s.
  const GTime late = ep(2025, 1, 3, 10, 30, 0);   // last records at 09:00
  double age = 0.0;
  const SatId c11 = rtk::makeSatId(Sys::BDS, 11), r01 = rtk::makeSatId(Sys::GLO, 1), g01 = rtk::makeSatId(Sys::GPS, 1);
  EXPECT_FALSE(es.healthy(r01, late, &age));
  const rtk::SatSolver solver(nav);
  EXPECT_TRUE(solver.ephHealthy(c11, late, 21601.0, &age));   // a record exists ...
  EXPECT_GT(age, 3600.0);
  EXPECT_FALSE(es.healthy(c11, late, &age));                   // ... but is older than masks.eph_age_s['C']
  // A satellite flagged unhealthy is refused whatever its age; a GPS svh = 1 is not masked.
  KeplerEph bad;
  found = false;
  for (const rtk::EphKey& k : nav.coverage())
    if (!found && k.sat.rtk_sat == g01.rtk_sat) found = nav.get(k.sat, k.toe, &bad);
  ASSERT_TRUE(found);
  bad.svh = 1;
  rtk::NavStore one;
  ASSERT_TRUE(one.add(bad, true));
  EXPECT_FALSE(rtk::SatSolver(one).ephHealthy(g01, bad.toe.add(60.0), 7200.0, &age));
  bad.svh = 0;
  ASSERT_TRUE(one.add(bad, true));
  EXPECT_TRUE(rtk::SatSolver(one).ephHealthy(g01, bad.toe.add(60.0), 7200.0, &age));
  EXPECT_EQ(age, 60.0);
  // A configuration without a system key is a hole, not "0 s" or "unlimited" (m16).
  Config holes = cfg;
  holes.masks.eph_age_s.erase('J');
  rtk::NavStore nav2;
  EphemStore es2(holes, nav2);
  ASSERT_GT(es2.loadRinex(brdcPath(), &why), 0) << why;
  EXPECT_FALSE(es2.healthy(j02, t, &age));
}

// L0 health rule (review m1/m2) = RTKLIB satexclude(): QZSS ignores bit 0; GLONASS reads svh = Bn | RINEX ≥ 3.05
// health flags << 1 (rinex.c:1356) as (svh & 9) == 0 && (svh & 6) != 4; every other system needs svh == 0.
TEST(U04SatState, SvhHealthRuleMatchesSatexclude) {
  const std::pair<Sys, int> sats[] = {{Sys::GPS, 7}, {Sys::GLO, 5}, {Sys::GAL, 11}, {Sys::QZS, 194}, {Sys::BDS, 19}};
  for (const auto& sp : sats) {
    const int sat = rtk::satNo(sp.first, sp.second);
    ASSERT_GT(sat, 0);
    for (int svh = -1; svh <= 511; ++svh) {
      EXPECT_EQ(rtk::svhHealthy(sp.first, svh), satexclude(sat, 0.0, svh, nullptr) == 0) << sysName(sp.first) << " svh " << svh;
    }
  }
  // The readings that differ from a plain svh == 0, both met in real files (IGS BRDC QZSS svh = 1; a RINEX 3.05
  // GLONASS record with its health-flag word set).
  EXPECT_TRUE(rtk::svhHealthy(Sys::QZS, 1));
  EXPECT_FALSE(rtk::svhHealthy(Sys::QZS, 2));
  EXPECT_FALSE(rtk::svhHealthy(Sys::GPS, 1));
  EXPECT_TRUE(rtk::svhHealthy(Sys::GLO, 2));
  EXPECT_TRUE(rtk::svhHealthy(Sys::GLO, 6));
  EXPECT_FALSE(rtk::svhHealthy(Sys::GLO, 4));
  EXPECT_FALSE(rtk::svhHealthy(Sys::GLO, 1));
  EXPECT_FALSE(rtk::svhHealthy(Sys::GLO, 8));
  EXPECT_FALSE(rtk::svhHealthy(Sys::GAL, -1));
  // End to end: the store's GLONASS record with the extended health word is judged by SatSolver::ephHealthy (and so by
  // EphemStore::healthy) exactly as pntpos' satexclude judges it.
  rtk::NavStore nav;
  GloEph g;
  g.sat = rtk::makeSatId(Sys::GLO, 5);
  g.frq = -2;
  g.toe = ep(2025, 1, 3, 6, 15, 18);
  g.tof = g.toe;
  g.pos = Eigen::Vector3d(-1.2e7, 1.5e7, 1.4e7);
  g.vel = Eigen::Vector3d(1500.0, 1200.0, -2500.0);
  g.source = Source::Rinex;
  const rtk::SatSolver solver(nav);
  for (int svh : {0, 2, 6, 1, 4, 8, 3}) {
    g.svh = svh;
    ASSERT_TRUE(nav.add(g, true));
    double age = -1.0;
    EXPECT_EQ(solver.ephHealthy(g.sat, g.toe.add(60.0), 1800.0, &age), svh == 0 || svh == 2 || svh == 6) << "svh " << svh;
    EXPECT_EQ(age, 60.0);
  }
}

// (The optional IGS SP3/CLK accuracy case was deleted in the P3 integration: AMENDMENTS A45 "U4 的 SP3/CLK 可选项:删除(不做)".)
