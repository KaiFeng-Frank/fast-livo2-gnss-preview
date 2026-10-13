// SPDX-License-Identifier: BSD-3-Clause
// test_u03_satmap.cpp — TEST_PLAN §1 U3 (DESIGN §12.3; AMENDMENTS A1, A16). Owner: WP-a.
// Criterion: gnss_comm numbering → (sys, prn) → satno round trip; eph_t fields (A, per-system toes/week, tgd, svh,
// GLONASS frq/glo_fcn/dtaun) survive bag → RINEX → readrnx within RINEX precision; bag ⊕ RINEX merge rules.
//
// Data: IGS BRDC snippet (RINEX 3.04, G/R/E/J/C, 2025-01-03 04:00–09:00 GPST, trimmed from
// BRDC00IGS_R_20250030000_01D_MN.rnx, 1768516 bytes, sha256 b9132287f7c3952101a9c5fc938fa81f1fb61b9d92e8193c190ca842cefd09c6)
// read from $LIVO_GNSS_BRDC_DIR (default /r/data/brdc = /work/fastlivo2_gnss/data/brdc in the build container) and
// checked against that sha256 before use (third_party/rtklib/testing/wpa_test_support.hpp, review m5).
// The "bag" ephemerides of the synthetic tests are those RINEX records re-expressed the way gnss_comm delivers them
// (GPST toe_tow as toes, GPS week, URA in metres, no sva): status PASS (synthetic). They cannot see the conventions of
// the real gnss_comm path — BagEphemerisVsBrdcFixture does, on the dev-set fixture (A24/A39; review m7).
#include <gtest/gtest.h>

#include <dirent.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iterator>
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
#include "rtklib.h"   // WP-a test files may include it (OWNERSHIP §1): RTKLIB's RINEX writer/reader is the reference

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

// DESIGN §12.3 gnss_comm numbering, written independently of gnss_io.
Sys gnssCommSplit(int id, int* prn) {
  if (id >= 1 && id <= 32) { *prn = id; return Sys::GPS; }
  if (id >= 33 && id <= 59) { *prn = id - 32; return Sys::GLO; }
  if (id >= 60 && id <= 97) { *prn = id - 59; return Sys::GAL; }
  if (id >= 98 && id <= 160) { *prn = id - 97; return Sys::BDS; }
  *prn = 0;
  return Sys::Unknown;
}

constexpr double kD2R = 3.1415926535897932 / 180.0;

Config metresCfg() {
  Config c;
  c.ephem.ura_semantics = Config::Ephem::UraSemantics::Metres;
  return c;
}

// The RINEX record as gnss_comm would hand it over: GPST toe_tow in toes, GPS week (wrong for BDS on purpose — the
// store must derive both per system), URA in metres, sva not derived.
KeplerEph asBagMessage(const KeplerEph& r) {
  KeplerEph b = r;
  b.source = Source::GnssCommEphem;
  b.sva = -1;
  b.toes = rtk::time2gpst(r.toe, &b.week);
  return b;
}

// Test-side writer path: core types → RTKLIB structs for outrnxnavb/outrnxgnavb.
eph_t toEph(const KeplerEph& k) {
  eph_t e;
  std::memset(&e, 0, sizeof(e));
  e.sat = k.sat.rtk_sat; e.iode = k.iode; e.iodc = k.iodc; e.sva = k.sva; e.svh = k.svh; e.week = k.week; e.code = k.code;
  e.flag = k.flag; e.toe = toG(k.toe); e.toc = toG(k.toc); e.ttr = toG(k.ttr);
  e.A = k.A; e.e = k.e; e.i0 = k.i0; e.OMG0 = k.OMG0; e.omg = k.omg; e.M0 = k.M0; e.deln = k.deln; e.OMGd = k.OMGd; e.idot = k.idot;
  e.crc = k.crc; e.crs = k.crs; e.cuc = k.cuc; e.cus = k.cus; e.cic = k.cic; e.cis = k.cis; e.toes = k.toes; e.fit = k.fit;
  e.f0 = k.f0; e.f1 = k.f1; e.f2 = k.f2;
  for (int i = 0; i < 6; ++i) e.tgd[i] = k.tgd[i];
  return e;
}
geph_t toGeph(const GloEph& g) {
  geph_t o;
  std::memset(&o, 0, sizeof(o));
  o.sat = g.sat.rtk_sat; o.iode = g.iode; o.frq = g.frq; o.svh = g.svh; o.flags = g.flags; o.sva = g.sva; o.age = g.age;
  o.toe = toG(g.toe); o.tof = toG(g.tof);
  for (int i = 0; i < 3; ++i) { o.pos[i] = g.pos[i]; o.vel[i] = g.vel[i]; o.acc[i] = g.acc[i]; }
  o.taun = g.taun; o.gamn = g.gamn; o.dtaun = g.dtaun;
  return o;
}

// "Within RINEX precision": the writer prints 12 significant digits (rinex.c outnavf, mantissa 0.1–1), so a value
// survives with relative error < 5e-12; exact zeros stay zero.
::testing::AssertionResult rinexEq(const char* what, double a, double b) {
  if (a == b || std::fabs(a - b) <= 1e-11 * std::fabs(a)) return ::testing::AssertionSuccess();
  char buf[160];
  std::snprintf(buf, sizeof(buf), "%s: %.17g vs %.17g", what, a, b);
  return ::testing::AssertionFailure() << buf;
}

// The same field decoded by the receiver (bag) and by the IGS merge (BRDC): several IGS sources store some values in
// single precision (Grass02 C26 TGD1: -5e-09 broadcast, -4.9999999696e-09 in the BRDC = float(-5e-09)), so the
// comparison allows float32 rounding (eps 6e-8). Every convention error it is meant to catch (units, scale, sign, wrong
// field, wrong week or time scale) is an O(1) relative difference.
::testing::AssertionResult igsEq(const char* what, double a, double b) {
  if (a == b || std::fabs(a - b) <= 1.2e-7 * std::max(std::fabs(a), std::fabs(b))) return ::testing::AssertionSuccess();
  char buf[160];
  std::snprintf(buf, sizeof(buf), "%s: %.17g (bag) vs %.17g (BRDC)", what, a, b);
  return ::testing::AssertionFailure() << buf;
}

struct KeyCmp {
  bool operator()(const rtk::EphKey& a, const rtk::EphKey& b) const {
    return std::make_tuple(a.sat.rtk_sat, a.toe.sec, a.nav_type) < std::make_tuple(b.sat.rtk_sat, b.toe.sec, b.nav_type);
  }
};

// ---- fixture (A24) helpers: slices, channels and decoding are shared (wpa_test_support.hpp) ----
struct EphSink : MeasurementSink {
  std::vector<KeplerEph> eph;
  std::vector<GloEph> geph;
  void pushObservation(ObsEpoch&&) override {}
  void pushEphemeris(const KeplerEph& e) override { eph.push_back(e); }
  void pushGloEphemeris(const GloEph& g) override { geph.push_back(g); }
  void pushIono(const Klobuchar&) override {}
  void pushPvt(const ReceiverPvt&) override {}
};
// The two ephemeris topics of a slice through WP-e's decoder (a stub throws std::logic_error).
bool decodeFixtureEphemerides(const std::string& slice, EphSink* sink, livo_wpa_testing::DecodeStats* st, std::string* why) {
  livo_gnss_io::GnssCommRos1Decoder dec(&satNoOf);
  return livo_wpa_testing::decodeChannel(dec, slice, livo_wpa_testing::kEphemChannel, *sink, st, why) &&
         livo_wpa_testing::decodeChannel(dec, slice, livo_wpa_testing::kGloEphemChannel, *sink, st, why, /*required=*/false);
}

enum class BdsClass { None, Geo, Igso, Meo };
BdsClass bdsClass(int prn) {
  if (prn <= 5 || prn >= 59) return BdsClass::Geo;   // ephemeris.c:265
  if ((prn >= 6 && prn <= 10) || prn == 13 || prn == 16 || prn == 31 || (prn >= 38 && prn <= 40) || prn == 56) return BdsClass::Igso;
  return BdsClass::Meo;
}

}  // namespace

// all 160 gnss_comm ids → (sys, prn) → satno → back (GPS 1-32, GLO 33-59, GAL 60-97, BDS 98-160)
TEST(U03SatMap, GnssComm160NumberingRoundTrip) {
  std::set<int> seen;
  int mapped = 0;
  for (int id = 1; id <= 160; ++id) {
    int prn = 0;
    const Sys sys = gnssCommSplit(id, &prn);
    ASSERT_NE(sys, Sys::Unknown) << id;
    const int n = rtk::satNo(sys, prn);
    if (sys == Sys::GAL && prn > 36) {   // E37/E38: beyond MAXPRNGAL, see GalPrn37And38AreDropped
      EXPECT_EQ(n, 0) << id;
      continue;
    }
    ASSERT_GT(n, 0) << "gnss_comm id " << id << " (" << sysChar(sys) << prn << ") unmapped";
    ++mapped;
    EXPECT_TRUE(seen.insert(n).second) << "satno collision at id " << id;
    int prn2 = 0;
    EXPECT_EQ(rtk::satSys(n, &prn2), sys) << id;
    EXPECT_EQ(prn2, prn) << id;
    char expect_id[8];
    std::snprintf(expect_id, sizeof(expect_id), "%c%02d", sysChar(sys), prn);
    EXPECT_EQ(rtk::satIdString(n), expect_id) << id;
    EXPECT_EQ(rtk::satIdFromString(expect_id), n) << id;
    const SatId s = makeSatIdOf(sys, prn);   // exported wrapper used by gnss_io's SatNoFn
    EXPECT_EQ(s.rtk_sat, n);
    EXPECT_EQ(satNoOf(sys, prn), n);
    EXPECT_EQ(satIdStringOf(s), expect_id);
  }
  EXPECT_EQ(mapped, 158);   // 160 minus E37/E38 (all of C01–C63 need P1, see BdsPrnAbove50RequiresP1)
  EXPECT_EQ(rtk::satNo(Sys::GPS, 0), 0);
  EXPECT_EQ(rtk::satNo(Sys::GPS, 33), 0);
  EXPECT_EQ(rtk::satNo(Sys::GLO, 28), 0);
  EXPECT_EQ(rtk::satSys(0, nullptr), Sys::Unknown);
  EXPECT_EQ(rtk::satSys(rtk::maxSat() + 1, nullptr), Sys::Unknown);
}

// gnss_io's split (WP-e) agrees with DESIGN §12.3 on every id (runs once gnss_io's mapping is merged)
TEST(U03SatMap, GnssCommSplitMatchesGnssIo) {
  try {
    for (int id = 0; id <= 161; ++id) {
      int p_ref = 0, p_io = -1;
      const Sys s_ref = gnssCommSplit(id, &p_ref);
      const Sys s_io = livo_gnss_io::gnssCommSatToSys(static_cast<uint32_t>(id), &p_io);
      EXPECT_EQ(s_io, s_ref) << id;
      if (s_ref != Sys::Unknown) {
        EXPECT_EQ(p_io, p_ref) << id;
      }
    }
  } catch (const std::logic_error& e) {
    LIVO_BLOCKED_IF_STUB(e, "WP-e merge (gnss_io gnssCommSatToSys)");
  }
}

// E37/E38 exceed MAXPRNGAL and must be counted, not mapped
TEST(U03SatMap, GalPrn37And38AreDropped) {
  EXPECT_EQ(rtk::satNo(Sys::GAL, 36) > 0, true);
  EXPECT_EQ(rtk::satNo(Sys::GAL, 37), 0);
  EXPECT_EQ(rtk::satNo(Sys::GAL, 38), 0);
  const Config cfg = metresCfg();
  rtk::NavStore nav;
  EphemStore es(cfg, nav);
  KeplerEph e;
  e.sat = rtk::makeSatId(Sys::GAL, 37);   // rtk_sat = 0
  e.toe = e.toc = e.ttr = ep(2025, 1, 3, 6, 0, 0);
  e.A = 29600e3;
  e.ura_raw = 3.12;
  e.code = (1 << 0) | (1 << 9);
  EXPECT_FALSE(es.addBag(e));
  e.sat = rtk::makeSatId(Sys::GAL, 38);
  EXPECT_FALSE(es.addBag(e));
  EXPECT_EQ(es.rejectedGalPrn(), 2);
  EXPECT_EQ(nav.countEph(Sys::GAL), 0);
  e.sat = rtk::makeSatId(Sys::GAL, 36);
  EXPECT_TRUE(es.addBag(e));
  EXPECT_EQ(es.rejectedGalPrn(), 2);
  EXPECT_EQ(nav.countEph(Sys::GAL), 1);
}

// C51–C63 map only when MAXPRNCMP=63 (patch P1)
TEST(U03SatMap, BdsPrnAbove50RequiresP1) {
  if (!(rtk::patchLevel() & rtk::kPatchP1)) {
    // The LIVO_RTKLIB_APPLY_PATCHES=OFF reference build: exactly the defect P1 fixes (rtkcmn.c:415).
    EXPECT_EQ(rtk::maxPrnBds(), 50);
    EXPECT_EQ(rtk::satNo(Sys::BDS, 59), 0);
    GTEST_SKIP() << "unpatched reference build (LIVO_RTKLIB_APPLY_PATCHES=OFF): C51-C63 unmappable by construction";
  }
  EXPECT_EQ(rtk::maxPrnBds(), 63);
  EXPECT_EQ(rtk::maxSat(), 32 + 27 + 36 + 10 + 63 + 14 + 39);   // GPS GLO GAL QZS BDS IRN SBS = 221 ≤ 255
  EXPECT_LE(rtk::maxSat(), 255);
  for (int prn = 51; prn <= 63; ++prn) {
    const int n = rtk::satNo(Sys::BDS, prn);
    ASSERT_GT(n, 0) << "C" << prn;
    int p = 0;
    EXPECT_EQ(rtk::satSys(n, &p), Sys::BDS);
    EXPECT_EQ(p, prn);
  }
  EXPECT_EQ(rtk::satIdString(rtk::satNo(Sys::BDS, 59)), "C59");
  EXPECT_EQ(rtk::satNo(Sys::BDS, 64), 0);
  // BDS precedes IRN/SBS in the numbering: those shift, GPS..BDS do not (the U5 golden relies on it).
  EXPECT_EQ(rtk::satNo(Sys::BDS, 1), 32 + 27 + 36 + 10 + 1);
  EXPECT_EQ(rtk::satNo(Sys::SBS, 120), 32 + 27 + 36 + 10 + 63 + 14 + 1);
}

// A, toes per system, week, tgd, svh survive bag→RINEX→readrnx within RINEX precision
TEST(U03SatMap, KeplerEphFieldsRoundTripViaRinex) {
  REQUIRE_SNIPPET();
  rtk::NavStore rinex;
  std::string why;
  ASSERT_GT(rinex.loadRinexNav(brdcPath(), true, &why), 1500) << why;   // 1932 unique keys (G/R/E/J/C, 5 h)

  // Bag leg: every RINEX record re-expressed as a gnss_comm message goes through EphemStore::addBag.
  const Config cfg = metresCfg();
  rtk::NavStore bag;
  EphemStore es(cfg, bag);
  std::map<rtk::EphKey, KeplerEph, KeyCmp> original;
  std::map<rtk::EphKey, GloEph, KeyCmp> original_glo;
  std::map<Sys, int> per_sys;
  int glo_i = 0;
  for (const rtk::EphKey& k : rinex.coverage()) {
    if (k.is_glo) {
      // GLONASS through the bag path with a delta_tau_n (RINEX 3.04 has none) that must survive as dtaun.
      GloEph g;
      ASSERT_TRUE(rinex.get(k.sat, k.toe, &g));
      g.source = Source::GnssCommGloEphem;
      g.dtaun = (1.0 + 0.01 * (glo_i++ % 50)) * -1.862645149231e-9;
      ASSERT_TRUE(es.addBag(g)) << rtk::satIdString(k.sat.rtk_sat);
      original_glo[k] = g;
      continue;
    }
    KeplerEph r;
    ASSERT_TRUE(rinex.get(k.sat, k.toe, &r, k.nav_type));
    ASSERT_TRUE(es.addBag(asBagMessage(r))) << rtk::satIdString(k.sat.rtk_sat);
    original[k] = r;
    ++per_sys[k.sat.sys];
  }
  EXPECT_GT(per_sys[Sys::GPS], 0);
  EXPECT_GT(per_sys[Sys::GAL], 0);
  EXPECT_GT(per_sys[Sys::BDS], 0);
  EXPECT_GT(per_sys[Sys::QZS], 0);
  EXPECT_EQ(es.galSourceBitsAssumed(), 0);   // IGS records carry their data-source bits

  // (1) The fields the store derives equal RINEX's convention bit for bit: toes (BDS: BDT seconds), week, sva.
  for (const auto& kv : original) {
    KeplerEph b;
    ASSERT_TRUE(bag.get(kv.first.sat, kv.first.toe, &b, kv.first.nav_type));
    const KeplerEph& r = kv.second;
    const std::string id = rtk::satIdString(r.sat.rtk_sat);
    EXPECT_EQ(b.toes, r.toes) << id;
    EXPECT_EQ(b.week, r.week) << id;
    EXPECT_EQ(b.sva, r.sva) << id;
    EXPECT_EQ(b.source, Source::GnssCommEphem);
  }

  // (2) Round trip through RINEX 3.05 text (RTKLIB writer) and readrnxt.
  const std::string out = ::testing::TempDir() + "/livo_u03_roundtrip.rnx";
  {
    FILE* fp = std::fopen(out.c_str(), "w");
    ASSERT_NE(fp, nullptr);
    static rnxopt_t opt;   // large structs: static storage, zeroed here
    static nav_t hdr;
    std::memset(&opt, 0, sizeof(opt));
    std::memset(&hdr, 0, sizeof(hdr));
    opt.rnxver = 305;      // 3.05 carries GLONASS dtaun
    opt.navsys = SYS_ALL;
    ASSERT_TRUE(outrnxnavh(fp, &opt, &hdr));
    for (const auto& kv : original) {
      KeplerEph b;
      ASSERT_TRUE(bag.get(kv.first.sat, kv.first.toe, &b, kv.first.nav_type));
      const eph_t e = toEph(b);
      ASSERT_TRUE(outrnxnavb(fp, &opt, &e));
    }
    for (const auto& kv : original_glo) {
      GloEph b;
      ASSERT_TRUE(bag.get(kv.first.sat, kv.first.toe, &b));
      const geph_t g = toGeph(b);
      ASSERT_TRUE(outrnxgnavb(fp, &opt, &g));
    }
    std::fclose(fp);
  }
  rtk::NavStore back;
  ASSERT_GT(back.loadRinexNav(out, true, &why), 0) << why;
  int compared = 0;
  for (const auto& kv : original) {
    KeplerEph x;
    const KeplerEph& r = kv.second;
    const std::string id = rtk::satIdString(r.sat.rtk_sat);
    ASSERT_TRUE(back.get(kv.first.sat, kv.first.toe, &x, kv.first.nav_type)) << id << " lost in the round trip";
    // Integers and times exactly; toe would be 14 s off for BDS with GPST toes (bdt2time(week, toes) in decode_eph).
    EXPECT_TRUE(x.toe == r.toe) << id;
    EXPECT_TRUE(x.toc == r.toc) << id;
    EXPECT_TRUE(x.ttr == r.ttr) << id;
    EXPECT_EQ(x.toes, r.toes) << id;
    EXPECT_EQ(x.week, r.week) << id;
    EXPECT_EQ(x.iode, r.iode) << id;
    EXPECT_EQ(x.iodc, r.iodc) << id;
    EXPECT_EQ(x.svh, r.svh) << id;
    EXPECT_EQ(x.sva, r.sva) << id;
    EXPECT_EQ(x.code, r.code) << id;
    EXPECT_EQ(x.flag, r.flag) << id;
    EXPECT_EQ(x.fit, r.fit) << id;
    // Doubles within RINEX precision (A is written as sqrt(A)).
    EXPECT_TRUE(rinexEq("A", x.A, r.A)) << id;
    EXPECT_TRUE(rinexEq("e", x.e, r.e)) << id;
    EXPECT_TRUE(rinexEq("i0", x.i0, r.i0)) << id;
    EXPECT_TRUE(rinexEq("OMG0", x.OMG0, r.OMG0)) << id;
    EXPECT_TRUE(rinexEq("omg", x.omg, r.omg)) << id;
    EXPECT_TRUE(rinexEq("M0", x.M0, r.M0)) << id;
    EXPECT_TRUE(rinexEq("deln", x.deln, r.deln)) << id;
    EXPECT_TRUE(rinexEq("OMGd", x.OMGd, r.OMGd)) << id;
    EXPECT_TRUE(rinexEq("idot", x.idot, r.idot)) << id;
    EXPECT_TRUE(rinexEq("crc", x.crc, r.crc)) << id;
    EXPECT_TRUE(rinexEq("crs", x.crs, r.crs)) << id;
    EXPECT_TRUE(rinexEq("cuc", x.cuc, r.cuc)) << id;
    EXPECT_TRUE(rinexEq("cus", x.cus, r.cus)) << id;
    EXPECT_TRUE(rinexEq("cic", x.cic, r.cic)) << id;
    EXPECT_TRUE(rinexEq("cis", x.cis, r.cis)) << id;
    EXPECT_TRUE(rinexEq("f0", x.f0, r.f0)) << id;
    EXPECT_TRUE(rinexEq("f1", x.f1, r.f1)) << id;
    EXPECT_TRUE(rinexEq("f2", x.f2, r.f2)) << id;
    EXPECT_TRUE(rinexEq("tgd0", x.tgd[0], r.tgd[0])) << id;
    if (r.sat.sys == Sys::GAL || r.sat.sys == Sys::BDS) {
      EXPECT_TRUE(rinexEq("tgd1", x.tgd[1], r.tgd[1])) << id;
    }
    ++compared;
  }
  EXPECT_EQ(compared, static_cast<int>(original.size()));
  ASSERT_GT(original_glo.size(), 100u);
  for (const auto& kv : original_glo) {
    GloEph x;
    const GloEph& r = kv.second;
    const std::string id = rtk::satIdString(r.sat.rtk_sat);
    ASSERT_TRUE(back.get(kv.first.sat, kv.first.toe, &x)) << id << " lost in the round trip";
    EXPECT_TRUE(x.toe == r.toe) << id;
    EXPECT_TRUE(x.tof == r.tof) << id;
    EXPECT_EQ(x.frq, r.frq) << id;
    EXPECT_EQ(x.svh, r.svh) << id;
    EXPECT_EQ(x.age, r.age) << id;
    EXPECT_EQ(x.iode, r.iode) << id;
    EXPECT_TRUE(rinexEq("dtaun", x.dtaun, r.dtaun)) << id;
    EXPECT_TRUE(rinexEq("taun", x.taun, r.taun)) << id;
    EXPECT_TRUE(rinexEq("gamn", x.gamn, r.gamn)) << id;
    for (int i = 0; i < 3; ++i) {
      EXPECT_TRUE(rinexEq("pos", x.pos[i], r.pos[i])) << id;
      EXPECT_TRUE(rinexEq("vel", x.vel[i], r.vel[i])) << id;
      EXPECT_TRUE(rinexEq("acc", x.acc[i], r.acc[i])) << id;
    }
    int prn = 0;
    rtk::satSys(r.sat.rtk_sat, &prn);
    EXPECT_EQ(bag.gloFcn(prn), r.frq) << id;   // glo_fcn = frq + 8 recorded by the bag path
  }
}

// toes = time2bdt(gpst2bdt(toe)) for BDS; GPS/GAL = time2gpst(toe)
TEST(U03SatMap, BdsToesIsBdtSeconds) {
  const GTime toe = rtk::gpst2time(2347, 446414.0);   // a BDS toe: BDT 2025-01-03 04:00:00 = GPST 04:00:14
  EXPECT_EQ(EphemStore::toesFor(Sys::BDS, toe), 446400.0);
  EXPECT_EQ(EphemStore::weekFor(Sys::BDS, toe), 2347 - 1356);
  for (Sys s : {Sys::GPS, Sys::GAL, Sys::QZS}) {
    EXPECT_EQ(EphemStore::toesFor(s, toe), 446414.0);
    EXPECT_EQ(EphemStore::weekFor(s, toe), 2347);   // GAL: RINEX/RTKLIB "GAL week" is continuous with the GPS week
  }
  KeplerEph k;
  k.sat = rtk::makeSatId(Sys::BDS, 11);
  k.toe = toe;
  k.toes = 446414.0;   // GPST toe_tow as delivered
  k.week = 2347;
  EphemStore::completeFields(&k);
  EXPECT_EQ(k.toes, 446400.0);
  EXPECT_EQ(k.week, 991);
  EphemStore::completeFields(&k);   // idempotent
  EXPECT_EQ(k.toes, 446400.0);
  EXPECT_EQ(k.week, 991);

  // The 27 km that a GPST toes costs (R2-3), on a real MEO record: eph2pos uses omge·toes.
  REQUIRE_SNIPPET();
  rtk::NavStore nav;
  std::string why;
  ASSERT_GT(nav.loadRinexNav(brdcPath(), true, &why), 0) << why;
  const SatId c11 = rtk::makeSatId(Sys::BDS, 11);
  const GTime t = ep(2025, 1, 3, 6, 3, 21.25);
  Eigen::Vector3d r_ok, r_bad;
  double dts = 0.0, var = 0.0;
  const rtk::SatSolver solver(nav);
  ASSERT_TRUE(solver.ephPos(c11, t, &r_ok, &dts, &var));
  KeplerEph rec;
  bool found = false;
  for (const rtk::EphKey& key : nav.coverage())
    if (key.sat.rtk_sat == c11.rtk_sat && std::fabs(key.toe.diff(t)) < 1800.0) found = nav.get(key.sat, key.toe, &rec, key.nav_type);
  ASSERT_TRUE(found);
  EXPECT_EQ(rec.toes, EphemStore::toesFor(Sys::BDS, rec.toe));   // RINEX already carries BDT seconds
  rec.toes = rtk::time2gpst(rec.toe, nullptr);                   // the gnss_comm toe_tow mistake
  rtk::NavStore wrong;
  ASSERT_TRUE(wrong.add(rec, true));
  ASSERT_TRUE(rtk::SatSolver(wrong).ephPos(c11, t, &r_bad, &dts, &var));
  // A rotation about the z axis by omge·14 s = 1.02e-3 rad: 16–28.5 km for a 27906 km MEO orbit (distance from the
  // axis r·cos(lat)), z untouched (eph2pos rs[2] = y·sin(i) does not depend on Ω).
  const double err = (r_bad - r_ok).norm();
  EXPECT_GT(err, 15e3);
  EXPECT_LT(err, 30e3);
  EXPECT_EQ(r_bad.z(), r_ok.z());
  EXPECT_NEAR(err, std::hypot(r_ok.x(), r_ok.y()) * 7.292115e-5 * 14.0, 1.0);
}

// frq, glo_fcn = frq+8 and delta_tau_n → dtaun are mapped
TEST(U03SatMap, GloEphFrqGloFcnDtaun) {
  const Config cfg;   // URA semantics Unknown: GLONASS carries no URA-dependent field, so it is not refused
  rtk::NavStore nav;
  EphemStore es(cfg, nav);
  GloEph g;
  g.sat = rtk::makeSatId(Sys::GLO, 5);
  g.frq = -2;
  g.toe = ep(2025, 1, 3, 6, 15, 18);
  g.tof = ep(2025, 1, 3, 6, 10, 18);
  g.pos = Eigen::Vector3d(-1.2e7, 1.5e7, 1.4e7);
  g.vel = Eigen::Vector3d(1500.0, 1200.0, -2500.0);
  g.taun = -1.0e-5;
  g.gamn = 1.0e-12;
  g.dtaun = -2.793967723846e-9;
  g.source = Source::GnssCommGloEphem;
  ASSERT_TRUE(es.addBag(g));
  EXPECT_EQ(nav.gloFcn(5), -2);
  EXPECT_EQ(es.gloFcn(g.sat), -2);
  GloEph s;
  ASSERT_TRUE(nav.get(g.sat, g.toe, &s));
  EXPECT_EQ(s.frq, -2);
  EXPECT_EQ(s.dtaun, g.dtaun);
  EXPECT_EQ(s.taun, g.taun);
  EXPECT_EQ(s.gamn, g.gamn);
  // Carrier frequency from the channel (RTKLIB sat2freq through geph/glo_fcn) and the freqs[] cross-check.
  EXPECT_EQ(rtk::sat2freq(g.sat.rtk_sat, SignalCode::L1C, nav), 1602.0e6 - 2 * 0.5625e6);
  EXPECT_EQ(rtk::sat2freq(g.sat.rtk_sat, SignalCode::L2C, nav), 1246.0e6 - 2 * 0.4375e6);
  EXPECT_TRUE(es.gloFcnConsistent(g.sat, 1602.0e6 - 2 * 0.5625e6));
  EXPECT_TRUE(es.gloFcnConsistent(g.sat, 1246.0e6 - 2 * 0.4375e6));
  EXPECT_FALSE(es.gloFcnConsistent(g.sat, 1602.0e6 - 1 * 0.5625e6));
  EXPECT_FALSE(es.gloFcnConsistent(g.sat, 1602.3e6));
  EXPECT_FALSE(es.gloFcnConsistent(rtk::makeSatId(Sys::GLO, 6), 1602.0e6));   // no channel on record yet
  // A different channel for the same slot, and a channel off the FDMA plan, are dropped and counted.
  GloEph c = g;
  c.frq = 3;
  c.toe = g.toe.add(900.0);
  EXPECT_FALSE(es.addBag(c));
  c.frq = 7;
  c.sat = rtk::makeSatId(Sys::GLO, 6);
  EXPECT_FALSE(es.addBag(c));
  EXPECT_EQ(es.rejectedGloFcn(), 2);
  EXPECT_EQ(nav.countGeph(), 1);
  {
    const EphemStore::Coverage cov = es.coverage();
    EXPECT_EQ(cov.n_geph_bag, 1);
    EXPECT_EQ(cov.n_geph_bag_delivered, 1);
    ASSERT_EQ(cov.entries.size(), 1u);
    EXPECT_TRUE(cov.entries[0].bag_delivered);
  }

  // RINEX wins on the same (sat, toe) (A1), but RINEX 3.04 has no dtaun: the bag's value is kept (R2-5), in both
  // arrival orders.
  REQUIRE_SNIPPET();
  std::string why;
  for (int order = 0; order < 2; ++order) {
    rtk::NavStore n2;
    EphemStore e2(cfg, n2);
    rtk::NavStore ref;
    ASSERT_GT(ref.loadRinexNav(brdcPath(), true, &why), 0) << why;
    GloEph rr;
    rtk::EphKey key;
    bool found = false;
    for (const rtk::EphKey& k : ref.coverage())
      if (k.is_glo && !found) {
        found = ref.get(k.sat, k.toe, &rr);
        key = k;
      }
    ASSERT_TRUE(found);
    EXPECT_EQ(rr.dtaun, 0.0);   // RINEX 3.04
    GloEph b = rr;
    b.source = Source::GnssCommGloEphem;
    b.dtaun = 1.5e-9;
    b.taun += 1e-9;             // distinguishable: RINEX must win on every other field
    if (order == 0) {
      EXPECT_TRUE(e2.addBag(b));
      EXPECT_GT(e2.loadRinex(brdcPath(), &why), 0) << why;
    } else {
      EXPECT_GT(e2.loadRinex(brdcPath(), &why), 0) << why;
      EXPECT_FALSE(e2.addBag(b));
    }
    GloEph m;
    ASSERT_TRUE(n2.get(key.sat, key.toe, &m));
    EXPECT_EQ(m.source, Source::Rinex) << order;
    EXPECT_EQ(m.taun, rr.taun) << order;
    EXPECT_EQ(m.dtaun, 1.5e-9) << order;
    EXPECT_EQ(e2.gloDtaunFilledFromBag(), 1) << order;
    // m3: the key the bag delivered stays "bag delivered" although the stored record is RINEX's, in both orders;
    // RINEX-only keys are not marked.
    const EphemStore::Coverage cov = e2.coverage();
    EXPECT_EQ(cov.n_geph_bag, 0) << order;
    EXPECT_EQ(cov.n_geph_bag_delivered, 1) << order;
    int marked = 0;
    for (const rtk::EphKey& k : cov.entries) {
      if (!k.bag_delivered) continue;
      ++marked;
      EXPECT_EQ(k.sat.rtk_sat, key.sat.rtk_sat) << order;
      EXPECT_TRUE(k.toe == key.toe) << order;
      EXPECT_EQ(k.source, Source::Rinex) << order;
    }
    EXPECT_EQ(marked, 1) << order;
    int prn = 0;
    rtk::satSys(key.sat.rtk_sat, &prn);
    EXPECT_EQ(n2.gloFcn(prn), rr.frq) << order;   // channel table filled from RINEX records
  }
}

// freqs → CODE_* table incl. BDS 1575.42 rejection and GLO channel cross-check
TEST(U03SatMap, FreqToCodeTable) {
  struct Row {
    Sys sys;
    int prn;
    double f;
    SignalCode code;
    int rtk_code;
    int idx;
  };
  const Row rows[] = {
      {Sys::GPS, 7, 1575.42e6, SignalCode::L1C, CODE_L1C, 0},  {Sys::GPS, 7, 1227.60e6, SignalCode::L2X, CODE_L2X, 1},
      {Sys::GPS, 7, 1176.45e6, SignalCode::L5X, CODE_L5X, 2},  {Sys::QZS, 194, 1575.42e6, SignalCode::L1C, CODE_L1C, 0},
      {Sys::GAL, 11, 1575.42e6, SignalCode::L1X, CODE_L1X, 0}, {Sys::GAL, 11, 1207.14e6, SignalCode::L7X, CODE_L7X, 1},
      {Sys::GAL, 11, 1176.45e6, SignalCode::L5Q, CODE_L5Q, 2}, {Sys::BDS, 19, 1561.098e6, SignalCode::L2I, CODE_L2I, 0},
      {Sys::BDS, 19, 1207.14e6, SignalCode::L7I, CODE_L7I, 1}, {Sys::BDS, 60, 1561.098e6, SignalCode::L2I, CODE_L2I, 0},
  };
  rtk::NavStore nav;
  for (const Row& r : rows) {
    int k = 99;
    SignalVerdict v = SignalVerdict::RejectedSystem;
    EXPECT_EQ(signalFromFrequency(r.sys, r.f, &k, &v), r.code) << sysChar(r.sys) << " " << r.f;
    EXPECT_EQ(v, SignalVerdict::Accepted);
    EXPECT_EQ(k, 0);
    EXPECT_EQ(rtk::codeToRtk(r.code), r.rtk_code);
    EXPECT_EQ(rtk::codeFromRtk(r.rtk_code), r.code);
    EXPECT_EQ(rtk::freqIndex(r.sys, r.code), r.idx) << signalCodeName(r.code);
    const int sat = rtk::satNo(r.sys, r.prn);
    ASSERT_GT(sat, 0);
    // RTKLIB's frequency of the code equals the table's nominal frequency (sat2freq==0 would drop the satellite in
    // rescode/resdop, R2-4).
    EXPECT_EQ(rtk::sat2freq(sat, r.code, nav), r.f) << signalCodeName(r.code);
    EXPECT_EQ(nominalFrequencyHz(r.sys, r.code, 0), r.f);
  }
  int k = 0;
  SignalVerdict v = SignalVerdict::Accepted;
  EXPECT_EQ(signalFromFrequency(Sys::BDS, 1575.42e6, &k, &v), SignalCode::Unknown);
  EXPECT_EQ(v, SignalVerdict::RejectedBdsB1C);
  EXPECT_EQ(signalFromFrequency(Sys::GPS, 1600.0e6, &k, &v), SignalCode::Unknown);
  EXPECT_EQ(v, SignalVerdict::RejectedUnknownFreq);
  EXPECT_EQ(signalFromFrequency(Sys::GLO, 1602.0e6 + 0.3e6, &k, &v), SignalCode::Unknown);
  EXPECT_EQ(v, SignalVerdict::RejectedGloChannel);
  EXPECT_EQ(signalFromFrequency(Sys::GLO, 1602.0e6 + 7 * 0.5625e6, &k, &v), SignalCode::Unknown);   // k = 7 off-plan
  EXPECT_EQ(signalFromFrequency(Sys::GLO, 1602.0e6 - 7 * 0.5625e6, &k, &v), SignalCode::L1C);
  EXPECT_EQ(k, -7);
  EXPECT_EQ(signalFromFrequency(Sys::GLO, 1246.0e6 + 6 * 0.4375e6, &k, &v), SignalCode::L2C);
  EXPECT_EQ(k, 6);
  EXPECT_EQ(rtk::freqIndex(Sys::GLO, SignalCode::L1C), 0);
  EXPECT_EQ(rtk::freqIndex(Sys::GLO, SignalCode::L2C), 1);
  EXPECT_EQ(rtk::codeToRtk(SignalCode::Unknown), CODE_NONE);
  EXPECT_EQ(rtk::codeFromRtk(CODE_L1W), SignalCode::Unknown);
  // GLONASS: the channel lives in glo_fcn until a record of the slot is stored.
  const int r3 = rtk::satNo(Sys::GLO, 3);
  EXPECT_EQ(rtk::sat2freq(r3, SignalCode::L1C, nav), 0.0);   // unknown channel
  nav.setGloFcn(3, 4);
  EXPECT_EQ(nav.gloFcn(3), 4);
  EXPECT_EQ(rtk::sat2freq(r3, SignalCode::L1C, nav), 1602.0e6 + 4 * 0.5625e6);
  EXPECT_EQ(nominalFrequencyHz(Sys::GLO, SignalCode::L1C, 4), 1602.0e6 + 4 * 0.5625e6);
}

// A1 / m7 / A16: RINEX wins on (sat, toe, nav_type), bag fills the gaps; Galileo I/NAV and F/NAV of one toe coexist;
// the I/NAV selection returns a record with code bit 9; URA semantics Unknown refuses bag ephemerides.
TEST(U03SatMap, EphemMergeBagAndRinex) {
  REQUIRE_SNIPPET();
  std::string why;
  rtk::NavStore ref;
  ASSERT_GT(ref.loadRinexNav(brdcPath(), true, &why), 0) << why;
  // A GPS record present in RINEX, and a bag-only key (same satellite, toe moved by 16 s).
  KeplerEph r1;
  rtk::EphKey k1;
  bool found = false;
  for (const rtk::EphKey& k : ref.coverage())
    if (!found && k.sat.sys == Sys::GPS) {
      found = ref.get(k.sat, k.toe, &r1, k.nav_type);
      k1 = k;
    }
  ASSERT_TRUE(found);
  KeplerEph b1 = asBagMessage(r1);
  b1.f0 += 1e-7;   // distinguishable
  KeplerEph b2 = asBagMessage(r1);
  b2.toe = b2.toe.add(16.0);
  b2.f0 += 2e-7;

  const Config cfg = metresCfg();
  for (int order = 0; order < 2; ++order) {
    rtk::NavStore nav;
    EphemStore es(cfg, nav);
    if (order == 0) {   // bag first: RINEX replaces the shared key
      EXPECT_TRUE(es.addBag(b1));
      EXPECT_TRUE(es.addBag(b2));
      EXPECT_GT(es.loadRinex(brdcPath(), &why), 0) << why;
    } else {            // RINEX first: the bag cannot overwrite it
      EXPECT_GT(es.loadRinex(brdcPath(), &why), 0) << why;
      EXPECT_FALSE(es.addBag(b1));
      EXPECT_TRUE(es.addBag(b2));
    }
    KeplerEph m1, m2;
    ASSERT_TRUE(nav.get(k1.sat, k1.toe, &m1));
    ASSERT_TRUE(nav.get(k1.sat, b2.toe, &m2));
    EXPECT_EQ(m1.source, Source::Rinex) << order;
    EXPECT_EQ(m1.f0, r1.f0) << order;
    EXPECT_EQ(m2.source, Source::GnssCommEphem) << order;
    EXPECT_EQ(m2.f0, b2.f0) << order;
    const EphemStore::Coverage c = es.coverage();
    EXPECT_EQ(c.n_eph_bag, 1) << order;              // stored bag records: the key RINEX lacks
    EXPECT_EQ(c.n_eph_bag_delivered, 2) << order;    // m3: "bag only" coverage also counts the key RINEX won
    int marked = 0;
    for (const rtk::EphKey& k : c.entries) {
      if (!k.bag_delivered) continue;
      ++marked;
      EXPECT_EQ(k.sat.rtk_sat, k1.sat.rtk_sat) << order;
      const bool shared = k.toe == k1.toe;
      EXPECT_TRUE(shared || k.toe == b2.toe) << order;
      EXPECT_EQ(k.source, shared ? Source::Rinex : Source::GnssCommEphem) << order;
    }
    EXPECT_EQ(marked, 2) << order;
    EXPECT_EQ(c.n_eph_rinex, nav.countEph(Sys::GPS) + nav.countEph(Sys::GAL) + nav.countEph(Sys::QZS) + nav.countEph(Sys::BDS) - 1);
    EXPECT_EQ(c.n_geph_bag, 0);
    EXPECT_EQ(c.n_geph_rinex, nav.countGeph());
    // Klobuchar from the RINEX header GPSA/GPSB (2025-003: alpha0 2.7940e-08, beta0 1.5155e+05); bag iono_params that
    // arrive afterwards are a diagnostic only (A38): iono() and nav.ion_gps keep the RINEX values.
    Klobuchar k;
    ASSERT_TRUE(es.iono(&k));
    EXPECT_EQ(k.source, Source::Rinex);
    EXPECT_EQ(k.alpha[0], 2.7940e-08);
    EXPECT_EQ(k.beta[0], 1.5155e+05);
    Klobuchar kb = k;
    kb.alpha[0] = 1.0e-8;
    kb.source = Source::GnssCommIono;
    es.setIonoFromBag(kb);
    ASSERT_TRUE(es.iono(&k));
    EXPECT_EQ(k.source, Source::Rinex) << order;
    EXPECT_EQ(k.alpha[0], 2.7940e-08) << order;
    Klobuchar kn;
    ASSERT_TRUE(nav.iono(&kn));
    EXPECT_EQ(kn.alpha[0], 2.7940e-08) << order;
    EXPECT_EQ(es.bagIonoCount(), 1) << order;
    Klobuchar kd;
    ASSERT_TRUE(es.bagIono(&kd));
    EXPECT_EQ(kd.alpha[0], 1.0e-8) << order;
    EXPECT_EQ(kd.source, Source::GnssCommIono) << order;
  }

  // Galileo: IGS BRDC carries F/NAV (code 258) and I/NAV (513/516/517) with one toe; both keys are stored, the first
  // I/NAV record in file order is kept, and the I/NAV selection (A16) yields a record with bit 9.
  const SatId e07 = rtk::makeSatId(Sys::GAL, 7);
  const GTime toe = ep(2025, 1, 3, 4, 0, 0);
  KeplerEph inav, fnav;
  ASSERT_TRUE(ref.get(e07, toe, &inav, 1));
  ASSERT_TRUE(ref.get(e07, toe, &fnav, 2));
  EXPECT_EQ(inav.code, 513);
  EXPECT_EQ(fnav.code, 258);
  EXPECT_NE(inav.f0, fnav.f0);   // E5b/E1 vs E5a/E1 clock
  rtk::setGalileoNavSelection(rtk::GalNavSelection::INav);
  const rtk::SatSolver solver(ref);
  rtk::SatState st;
  ASSERT_TRUE(solver.satState(e07, ep(2025, 1, 3, 4, 3, 21.25), 2.4e7, &st));
  EXPECT_EQ(st.nav_type, 1);
  EXPECT_TRUE(rtk::galileoNavSelection() == rtk::GalNavSelection::INav);

  // Galileo bag record without data-source bits: taken as I/NAV E1-B (u-blox), counted.
  {
    rtk::NavStore nav;
    EphemStore es(cfg, nav);
    KeplerEph g = asBagMessage(inav);
    g.code = 0;
    ASSERT_TRUE(es.addBag(g));
    EXPECT_EQ(es.galSourceBitsAssumed(), 1);
    KeplerEph s;
    ASSERT_TRUE(nav.get(e07, toe, &s, 1));
    EXPECT_EQ(s.code, (1 << 0) | (1 << 9));
  }
  // URA semantics not decided (A16): bag ephemerides refused and counted; index semantics round the value.
  {
    Config unknown;
    rtk::NavStore nav;
    EphemStore es(unknown, nav);
    EXPECT_FALSE(es.addBag(b1));
    EXPECT_EQ(es.rejectedUraUnknown(), 1);
    EXPECT_EQ(nav.countEph(Sys::GPS), 0);
    es.setUraSemantics(Config::Ephem::UraSemantics::Index);
    KeplerEph bi = b1;
    bi.ura_raw = 2.0;
    EXPECT_TRUE(es.addBag(bi));
    KeplerEph s;
    ASSERT_TRUE(nav.get(k1.sat, k1.toe, &s));
    EXPECT_EQ(s.sva, 2);
    EXPECT_EQ(s.ura_raw, 2.0);   // the raw value is preserved for the offline re-judgement
    es.setUraSemantics(Config::Ephem::UraSemantics::Metres);
    KeplerEph bm = b2;
    bm.ura_raw = 3.12;   // metres → URA index 1 (3.4 m bound)
    EXPECT_TRUE(es.addBag(bm));
    ASSERT_TRUE(nav.get(k1.sat, bm.toe, &s));
    EXPECT_EQ(s.sva, 1);
  }
  // Metres semantics on Galileo use the SISA table (3.12 m → index 107), not the GPS URA table.
  EXPECT_EQ(rtk::sisaIndexFromMetres(3.12), 107);
  EXPECT_NEAR(rtk::sisaMetresFromIndex(107), 3.12, 1e-12);
  EXPECT_EQ(rtk::uraIndexFromMetres(3.12), 1);
  EXPECT_EQ(rtk::uraMetresFromIndex(1), 3.4);
  EXPECT_EQ(rtk::uraMetresFromIndex(15), 6144.0);
  EXPECT_EQ(rtk::uraMetresFromIndex(-1), 6144.0);
  EXPECT_EQ(rtk::sisaIndexFromMetres(-1.0), 255);   // NAPA
  EXPECT_EQ(rtk::ephVarianceFromIndex(Sys::GAL, 255), 500.0 * 500.0);
  EXPECT_EQ(rtk::ephVarianceFromIndex(Sys::GAL, -1), 500.0 * 500.0);
  EXPECT_EQ(rtk::ephVarianceFromIndex(Sys::GPS, 0), 2.4 * 2.4);
  EXPECT_EQ(rtk::ephVarianceFromIndex(Sys::GLO, 0), 25.0);
}

// A38 (revising A1/§12.3): the GPS Klobuchar is the RINEX BRDC header GPSA/GPSB only; bag iono_params are recorded as a
// diagnostic and never reach iono(), hasIono() (frozen skeleton method, review B1) or nav.ion_gps (what pntpos reads);
// without RINEX GPSA/GPSB there is no Klobuchar and the core's ionospheric delay is exactly 0 (A37, DESIGN §8) — in
// both arrival orders, also across a NAV file without GPSA/GPSB.
TEST(U03SatMap, HasIonoFollowsSources) {
  REQUIRE_SNIPPET();
  std::string why;
  rtk::NavStore ref;
  ASSERT_GT(ref.loadRinexNav(brdcPath(), true, &why), 0) << why;
  KeplerEph r;
  bool found = false;
  for (const rtk::EphKey& k : ref.coverage())
    if (!found && k.sat.sys == Sys::GPS) found = ref.get(k.sat, k.toe, &r, k.nav_type);
  ASSERT_TRUE(found);
  // One GPS record through RTKLIB's writer with outiono = 0: a valid NAV file that carries no Klobuchar set.
  const std::string no_iono = ::testing::TempDir() + "/livo_u03_no_iono.rnx";
  {
    FILE* fp = std::fopen(no_iono.c_str(), "w");
    ASSERT_NE(fp, nullptr);
    static rnxopt_t opt;
    static nav_t hdr;
    std::memset(&opt, 0, sizeof(opt));
    std::memset(&hdr, 0, sizeof(hdr));
    opt.rnxver = 304;
    opt.navsys = SYS_ALL;
    ASSERT_TRUE(outrnxnavh(fp, &opt, &hdr));
    const eph_t e = toEph(r);
    ASSERT_TRUE(outrnxnavb(fp, &opt, &e));
    std::fclose(fp);
  }
  Klobuchar kb;
  kb.alpha = {1.0e-8, 0.0, 0.0, 0.0};
  kb.beta = {1.2e5, 0.0, 0.0, 0.0};
  kb.source = Source::GnssCommIono;
  // A second valid NAV file whose header carries another GPSA/GPSB (4 significant digits, as RTKLIB writes them).
  const std::string other_iono = ::testing::TempDir() + "/livo_u03_other_iono.rnx";
  const double kOtherA0 = 1.118e-08, kOtherB0 = 8.806e+04;
  {
    FILE* fp = std::fopen(other_iono.c_str(), "w");
    ASSERT_NE(fp, nullptr);
    static rnxopt_t opt;
    static nav_t hdr;
    std::memset(&opt, 0, sizeof(opt));
    std::memset(&hdr, 0, sizeof(hdr));
    opt.rnxver = 304;
    opt.navsys = SYS_ALL;
    opt.outiono = 1;
    const double ion[8] = {kOtherA0, 2.235e-08, -5.960e-08, -1.192e-07, kOtherB0, 3.277e+04, -1.966e+05, -6.554e+04};
    for (int i = 0; i < 8; ++i) hdr.ion_gps[i] = ion[i];
    ASSERT_TRUE(outrnxnavh(fp, &opt, &hdr));
    const eph_t e = toEph(r);
    ASSERT_TRUE(outrnxnavb(fp, &opt, &e));
    std::fclose(fp);
  }
  const Config cfg = metresCfg();
  const Geodetic rcv{31.0283 * kD2R, 121.4413 * kD2R, 12.0};
  const GTime t = ep(2025, 1, 3, 6, 0, 0);   // 14:00 local time near Shanghai: a Klobuchar set gives metres here
  for (int order = 0; order < 2; ++order) {
    rtk::NavStore nav;
    EphemStore es(cfg, nav);
    Klobuchar k;
    EXPECT_FALSE(es.hasIono()) << order;
    EXPECT_FALSE(es.iono(&k)) << order;
    EXPECT_FALSE(es.bagIono(&k)) << order;
    ASSERT_EQ(es.loadRinex(no_iono, &why), 1) << why;
    EXPECT_FALSE(es.hasIono()) << "NAV file without GPSA/GPSB, order " << order;
    if (order == 0) {
      ASSERT_GT(es.loadRinex(brdcPath(), &why), 0) << why;
      EXPECT_TRUE(es.hasIono());
      ASSERT_TRUE(es.iono(&k));
      EXPECT_EQ(k.source, Source::Rinex);
      EXPECT_EQ(k.alpha[0], 2.7940e-08);
      es.setIonoFromBag(kb);
      EXPECT_TRUE(es.hasIono());
      ASSERT_TRUE(es.iono(&k));
      EXPECT_EQ(k.source, Source::Rinex);          // A38: the later bag message changes nothing
      EXPECT_EQ(k.alpha[0], 2.7940e-08);
    } else {
      // Bag iono_params alone: no GPS Klobuchar at all (A38), so the core's delay is 0 (A37) — not the bag set, not
      // RTKLIB's built-in 2004 set.
      es.setIonoFromBag(kb);
      EXPECT_FALSE(es.hasIono());
      Klobuchar none;
      EXPECT_FALSE(es.iono(&none));
      EXPECT_FALSE(none.valid());
      EXPECT_FALSE(nav.iono(nullptr));             // nav.ion_gps untouched: pntpos never sees the bag set
      for (double el_deg : {15.0, 45.0, 90.0}) {
        EXPECT_EQ(rtk::ionModelGpsL1(t, none, rcv, 1.0, el_deg * kD2R), 0.0) << el_deg;
        EXPECT_GT(rtk::ionModelGpsL1(t, kb, rcv, 1.0, el_deg * kD2R), 0.5) << el_deg;   // the bag set would not be 0
      }
      ASSERT_GT(es.loadRinex(brdcPath(), &why), 0) << why;
      EXPECT_TRUE(es.hasIono());
      ASSERT_TRUE(es.iono(&k));
      EXPECT_EQ(k.source, Source::Rinex);          // the RINEX header arriving later is the Klobuchar in force
      EXPECT_EQ(k.alpha[0], 2.7940e-08);
    }
    Klobuchar kn;
    ASSERT_TRUE(nav.iono(&kn));
    EXPECT_EQ(kn.alpha[0], 2.7940e-08) << order;   // nav.ion_gps (what pntpos reads) = GPSA/GPSB, never the bag set
    EXPECT_EQ(kn.beta[0], 1.5155e+05) << order;
    EXPECT_EQ(es.bagIonoCount(), 1) << order;
    Klobuchar kd;
    ASSERT_TRUE(es.bagIono(&kd)) << order;          // kept for the diagnostics
    EXPECT_EQ(kd.alpha[0], 1.0e-8) << order;
    // A later NAV file with another GPSA/GPSB does not replace the first one's (first file in force, iono() and
    // nav.ion_gps alike).
    ASSERT_GE(es.loadRinex(other_iono, &why), 0) << why;
    ASSERT_TRUE(es.iono(&k));
    EXPECT_EQ(k.alpha[0], 2.7940e-08) << order;
    ASSERT_TRUE(nav.iono(&kn));
    EXPECT_EQ(kn.alpha[0], 2.7940e-08) << order;
  }
  // ... and the other way round: that file first, the BRDC second.
  rtk::NavStore nav;
  EphemStore es(cfg, nav);
  ASSERT_EQ(es.loadRinex(other_iono, &why), 1) << why;
  ASSERT_GT(es.loadRinex(brdcPath(), &why), 0) << why;
  Klobuchar k;
  ASSERT_TRUE(es.iono(&k));
  EXPECT_EQ(k.source, Source::Rinex);
  EXPECT_EQ(k.alpha[0], kOtherA0);
  EXPECT_EQ(k.beta[0], kOtherB0);
  Klobuchar kn;
  ASSERT_TRUE(nav.iono(&kn));
  EXPECT_EQ(kn.alpha[0], kOtherA0);
}

// A16's URA semantics belong to the gnss_comm messages (WP-e contract, INTERFACE_CHANGES): records RTKLIB decoded
// itself (UBX stream, RINEX through RinexSource) carry RTKLIB's index in sva (ura_raw = sva) and keep it under every
// semantics — also Unknown, which A16 permits for those sources — while a gnss_comm record is still refused under
// Unknown and converted from ura_raw under Index/Metres.
TEST(U03SatMap, UraSemanticsApplyToGnssCommOnly) {
  REQUIRE_SNIPPET();
  rtk::NavStore ref;
  std::string why;
  ASSERT_GT(ref.loadRinexNav(brdcPath(), true, &why), 0) << why;
  KeplerEph gps, gal;
  bool have_gps = false, have_gal = false;
  for (const rtk::EphKey& k : ref.coverage()) {
    if (!have_gps && k.sat.sys == Sys::GPS) have_gps = ref.get(k.sat, k.toe, &gps, k.nav_type);
    if (!have_gal && k.sat.sys == Sys::GAL && k.nav_type == 1) have_gal = ref.get(k.sat, k.toe, &gal, k.nav_type);
  }
  ASSERT_TRUE(have_gps && have_gal);
  gps.sva = 3;   // 6.85 m: every conversion of ura_raw = 3 would change it (Metres → 1, and Unknown refuses)
  ASSERT_GT(gal.sva, 49);   // a SISA index read as metres is off the SISA table (Metres → 255)
  using U = Config::Ephem::UraSemantics;
  for (U sem : {U::Unknown, U::Index, U::Metres}) {
    Config cfg;
    cfg.ephem.ura_semantics = sem;
    for (Source src : {Source::UbxStream, Source::Rinex}) {
      rtk::NavStore nav;
      EphemStore es(cfg, nav);
      for (KeplerEph r : {gps, gal}) {
        r.source = src;
        r.ura_raw = r.sva;
        ASSERT_TRUE(es.addBag(r)) << sourceName(src) << " " << static_cast<int>(sem);
        KeplerEph s;
        ASSERT_TRUE(nav.get(r.sat, r.toe, &s, rtk::galNavTypeOf(r.code)));
        EXPECT_EQ(s.sva, r.sva) << sourceName(src) << " " << sysName(r.sat.sys) << " semantics " << static_cast<int>(sem);
      }
      EXPECT_EQ(es.rejectedUraUnknown(), 0);
    }
    // The gnss_comm path is unchanged: 4.85 m is URA index 2 (bound 4.85 m), as an index it rounds to 5.
    rtk::NavStore nav;
    EphemStore es(cfg, nav);
    KeplerEph b = asBagMessage(gps);
    b.ura_raw = 4.85;
    KeplerEph s;
    if (sem == U::Unknown) {
      EXPECT_FALSE(es.addBag(b));
      EXPECT_EQ(es.rejectedUraUnknown(), 1);
      EXPECT_FALSE(nav.get(b.sat, b.toe, &s));
    } else {
      ASSERT_TRUE(es.addBag(b));
      ASSERT_TRUE(nav.get(b.sat, b.toe, &s));
      EXPECT_EQ(s.sva, sem == U::Metres ? 2 : 5);
    }
  }
}

// U3 on the fixture slices (review m7; A24/A39): the real gnss_comm ephemerides (fixture bytes → WP-e decoder →
// EphemStore::addBag) against the slice's same-day IGS BRDC for every key both carry, then bag → RINEX 3.05 text →
// readrnxt. This is the leg that sees the conventions of the gnss_comm path — BDS toe in BDT (key and toes), GPST
// GLONASS toe, Galileo data-source code (nav_type of the key), tgd/BGD order, GLONASS frq and τn sign, health.
// A slice without ephemeris topics (A26: Occlusion03/04 and Dynamic03 have none) contributes nothing.
namespace {

struct SysCount {
  int keys = 0, matched = 0;
};

// One slice; `had_eph` = the slice carries ephemeris topics. Per-system key/match counts are added to `total` (#6c: every
// system must be matched over the slices of a test, so a system the decoder drops as a whole cannot pass silently).
void bagEphemerisVsBrdc(const livo_wpa_testing::Slice& sl, bool* had_eph, std::map<Sys, SysCount>* total) {
  *had_eph = false;
  const std::string& slice = sl.path;
  if (livo_wpa_testing::findFile(slice + "/bytes", "ephem", ".bin", "glo").empty()) {
    std::printf("[U3 fixture] %s: no ephem topic in the slice\n", sl.name.c_str());
    return;
  }
  EphSink sink;
  livo_wpa_testing::DecodeStats ds;
  std::string why;
  try {
    ASSERT_TRUE(decodeFixtureEphemerides(slice, &sink, &ds, &why)) << why;
  } catch (const std::logic_error& e) {
    LIVO_BLOCKED_IF_STUB(e, "WP-e merge (gnss_io GnssCommRos1Decoder)");
  }
  std::printf("[U3 fixture] %s: %d ephemeris messages, %d refused by the decoder%s%s\n", sl.name.c_str(), ds.messages, ds.refused,
              ds.refused ? ", first: " : "", ds.first_refusal.c_str());
  ASSERT_FALSE(sink.eph.empty()) << slice << ": no GnssEphemMsg decoded";
  *had_eph = true;
  // Index semantics: every compared field is URA-independent; sva only has to survive the round trip below.
  Config cfg;
  cfg.ephem.ura_semantics = Config::Ephem::UraSemantics::Index;
  rtk::NavStore bag_nav, rnx;
  EphemStore bag(cfg, bag_nav);
  for (const KeplerEph& e : sink.eph) bag.addBag(e);
  for (const GloEph& g : sink.geph) bag.addBag(g);
  EXPECT_EQ(bag.rejectedGloFcn(), 0) << slice << ": GLONASS channel conflicts in the bag";
  const std::string brdc = livo_gnss_io::testsupport::fixtureBrdc(slice);
  ASSERT_FALSE(brdc.empty()) << slice << ": A24 brdc/ missing";
  ASSERT_GT(rnx.loadRinexNav(brdc, true, &why), 0) << why;

  struct Count {
    int keys = 0, matched = 0;
    std::string unmatched;
  };
  std::map<Sys, Count> per;
  std::map<BdsClass, int> bds_keys, bds_matched;
  for (const rtk::EphKey& k : bag_nav.coverage()) {
    const std::string id = rtk::satIdString(k.sat.rtk_sat) + " toe " + rtk::timeStr(k.toe, 0) + " nav_type " + std::to_string(k.nav_type);
    Count& c = per[k.sat.sys];
    ++c.keys;
    const BdsClass cls = k.sat.sys == Sys::BDS ? bdsClass(k.sat.prn) : BdsClass::None;
    if (cls != BdsClass::None) ++bds_keys[cls];
    if (k.is_glo) {
      GloEph b, r;
      ASSERT_TRUE(bag_nav.get(k.sat, k.toe, &b));
      if (!rnx.get(k.sat, k.toe, &r)) {
        c.unmatched += "\n  " + id;
        continue;
      }
      ++c.matched;
      EXPECT_EQ(b.frq, r.frq) << id;
      EXPECT_TRUE(igsEq("taun", b.taun, r.taun)) << id;   // sign: RINEX −TauN is negated by readrnx
      EXPECT_TRUE(igsEq("gamn", b.gamn, r.gamn)) << id;
      for (int i = 0; i < 3; ++i) {
        EXPECT_TRUE(igsEq("pos", b.pos[i], r.pos[i])) << id;
        EXPECT_TRUE(igsEq("vel", b.vel[i], r.vel[i])) << id;
        EXPECT_TRUE(igsEq("acc", b.acc[i], r.acc[i])) << id;
      }
      EXPECT_EQ(rtk::svhHealthy(Sys::GLO, b.svh), rtk::svhHealthy(Sys::GLO, r.svh)) << id << " svh " << b.svh << "/" << r.svh;
      EXPECT_EQ(bag.gloFcn(k.sat), r.frq) << id;
      continue;
    }
    KeplerEph b, r;
    ASSERT_TRUE(bag_nav.get(k.sat, k.toe, &b, k.nav_type));
    if (!rnx.get(k.sat, k.toe, &r, k.nav_type)) {
      c.unmatched += "\n  " + id;
      continue;
    }
    ++c.matched;
    if (cls != BdsClass::None) ++bds_matched[cls];
    EXPECT_TRUE(b.toc == r.toc) << id;
    EXPECT_EQ(b.toes, r.toes) << id;   // BDS: BDT seconds (27 km if not, R2-3)
    EXPECT_EQ(b.week, r.week) << id;
    EXPECT_TRUE(igsEq("A", b.A, r.A)) << id;
    EXPECT_TRUE(igsEq("e", b.e, r.e)) << id;
    EXPECT_TRUE(igsEq("i0", b.i0, r.i0)) << id;
    EXPECT_TRUE(igsEq("OMG0", b.OMG0, r.OMG0)) << id;
    EXPECT_TRUE(igsEq("omg", b.omg, r.omg)) << id;
    EXPECT_TRUE(igsEq("M0", b.M0, r.M0)) << id;
    EXPECT_TRUE(igsEq("deln", b.deln, r.deln)) << id;
    EXPECT_TRUE(igsEq("OMGd", b.OMGd, r.OMGd)) << id;
    EXPECT_TRUE(igsEq("idot", b.idot, r.idot)) << id;
    EXPECT_TRUE(igsEq("crc", b.crc, r.crc)) << id;
    EXPECT_TRUE(igsEq("crs", b.crs, r.crs)) << id;
    EXPECT_TRUE(igsEq("cuc", b.cuc, r.cuc)) << id;
    EXPECT_TRUE(igsEq("cus", b.cus, r.cus)) << id;
    EXPECT_TRUE(igsEq("cic", b.cic, r.cic)) << id;
    EXPECT_TRUE(igsEq("cis", b.cis, r.cis)) << id;
    EXPECT_TRUE(igsEq("f0", b.f0, r.f0)) << id;
    EXPECT_TRUE(igsEq("f1", b.f1, r.f1)) << id;
    EXPECT_TRUE(igsEq("f2", b.f2, r.f2)) << id;
    EXPECT_TRUE(igsEq("tgd0", b.tgd[0], r.tgd[0])) << id;
    if (k.sat.sys == Sys::GAL || k.sat.sys == Sys::BDS) {
      EXPECT_TRUE(igsEq("tgd1", b.tgd[1], r.tgd[1])) << id;
    }
    if (k.sat.sys == Sys::GPS || k.sat.sys == Sys::GAL) {
      EXPECT_EQ(b.iode, r.iode) << id;   // IODE / IODnav (BDS AODE is not comparable, §4.5)
    }
    if (k.sat.sys == Sys::GPS) {
      EXPECT_EQ(b.svh, r.svh) << id;
    } else {
      EXPECT_EQ(rtk::svhHealthy(k.sat.sys, b.svh), rtk::svhHealthy(k.sat.sys, r.svh)) << id << " svh " << b.svh << "/" << r.svh;
    }
  }
  // A convention error of one system makes that whole system vanish from the matches (review M2): every system the
  // bag carries must match, and at most 10 % of its keys may be absent from the IGS merge.
  for (const auto& kv : per) {
    const Count& c = kv.second;
    std::printf("[U3 fixture] %s %s: %d bag keys, %d in BRDC\n", sl.name.c_str(), sysName(kv.first), c.keys, c.matched);
    EXPECT_GT(c.matched, 0) << slice << ": no " << sysName(kv.first) << " bag key of " << c.keys
                            << " found in the same-day BRDC (toe/week/time-scale convention of the gnss_comm path?)" << c.unmatched;
    EXPECT_LE(10 * (c.keys - c.matched), c.keys) << slice << ": " << sysName(kv.first) << " bag keys missing from BRDC" << c.unmatched;
  }
  for (const auto& kv : bds_keys) EXPECT_GT(bds_matched[kv.first], 0) << slice << ": a BDS orbit class (GEO/IGSO/MEO) never matched";
  for (const auto& kv : per) {
    (*total)[kv.first].keys += kv.second.keys;
    (*total)[kv.first].matched += kv.second.matched;
  }

  // bag → RINEX 3.05 (carries GLONASS dtaun) → readrnxt: every bag record survives within RINEX precision.
  const std::string out = ::testing::TempDir() + "/livo_u03_fixture_roundtrip.rnx";
  {
    FILE* fp = std::fopen(out.c_str(), "w");
    ASSERT_NE(fp, nullptr);
    static rnxopt_t opt;
    static nav_t hdr;
    std::memset(&opt, 0, sizeof(opt));
    std::memset(&hdr, 0, sizeof(hdr));
    opt.rnxver = 305;
    opt.navsys = SYS_ALL;
    ASSERT_TRUE(outrnxnavh(fp, &opt, &hdr));
    for (const rtk::EphKey& k : bag_nav.coverage()) {
      if (k.is_glo) {
        GloEph g;
        ASSERT_TRUE(bag_nav.get(k.sat, k.toe, &g));
        const geph_t o = toGeph(g);
        ASSERT_TRUE(outrnxgnavb(fp, &opt, &o));
      } else {
        KeplerEph e;
        ASSERT_TRUE(bag_nav.get(k.sat, k.toe, &e, k.nav_type));
        const eph_t o = toEph(e);
        ASSERT_TRUE(outrnxnavb(fp, &opt, &o));
      }
    }
    std::fclose(fp);
  }
  rtk::NavStore back;
  ASSERT_GT(back.loadRinexNav(out, true, &why), 0) << why;
  for (const rtk::EphKey& k : bag_nav.coverage()) {
    const std::string id = rtk::satIdString(k.sat.rtk_sat);
    if (k.is_glo) {
      GloEph b, x;
      ASSERT_TRUE(bag_nav.get(k.sat, k.toe, &b));
      ASSERT_TRUE(back.get(k.sat, k.toe, &x)) << id << " lost in the round trip";
      EXPECT_EQ(x.frq, b.frq) << id;
      EXPECT_EQ(x.svh, b.svh) << id;
      EXPECT_TRUE(rinexEq("dtaun", x.dtaun, b.dtaun)) << id;
      EXPECT_TRUE(rinexEq("taun", x.taun, b.taun)) << id;
      EXPECT_TRUE(rinexEq("gamn", x.gamn, b.gamn)) << id;
      for (int i = 0; i < 3; ++i) EXPECT_TRUE(rinexEq("pos", x.pos[i], b.pos[i])) << id;
      continue;
    }
    KeplerEph b, x;
    ASSERT_TRUE(bag_nav.get(k.sat, k.toe, &b, k.nav_type));
    ASSERT_TRUE(back.get(k.sat, k.toe, &x, k.nav_type)) << id << " lost in the round trip";
    EXPECT_TRUE(x.toc == b.toc) << id;
    EXPECT_EQ(x.toes, b.toes) << id;
    EXPECT_EQ(x.week, b.week) << id;
    EXPECT_EQ(x.svh, b.svh) << id;
    if (k.sat.sys != Sys::BDS) {
      EXPECT_EQ(x.code, b.code) << id;   // RINEX BDS has no code field (rinex.c decode_eph never reads it back)
    }
    EXPECT_TRUE(rinexEq("A", x.A, b.A)) << id;
    EXPECT_TRUE(rinexEq("M0", x.M0, b.M0)) << id;
    EXPECT_TRUE(rinexEq("f0", x.f0, b.f0)) << id;
    EXPECT_TRUE(rinexEq("tgd0", x.tgd[0], b.tgd[0])) << id;
    if (k.sat.sys == Sys::GAL || k.sat.sys == Sys::BDS) {
      EXPECT_TRUE(rinexEq("tgd1", x.tgd[1], b.tgd[1])) << id;
    }
  }
}

// #6c: over the slices of a test every one of GPS, Galileo, BeiDou and GLONASS had bag keys and matched the BRDC.
void expectAllSystemsMatched(const std::map<Sys, SysCount>& total, const char* what) {
  for (Sys s : {Sys::GPS, Sys::GAL, Sys::BDS, Sys::GLO}) {
    const auto it = total.find(s);
    const SysCount c = it == total.end() ? SysCount() : it->second;
    EXPECT_GT(c.keys, 0) << what << ": no " << sysName(s) << " ephemeris decoded from any slice";
    EXPECT_GT(c.matched, 0) << what << ": no " << sysName(s) << " bag key matched the same-day BRDC";
  }
}

}  // namespace

// Gate (A24/A39): dev-set slices only; at least one of them must carry ephemeris topics (A26: Grass01/02).
TEST(U03SatMap, BagEphemerisVsBrdcFixture) {
  LIVO_DEVSET_SLICES(slices);
  bool any_eph = false;
  std::map<Sys, SysCount> total;
  for (const livo_wpa_testing::Slice& sl : slices) {
    bool had = false;
    bagEphemerisVsBrdc(sl, &had, &total);
    if (HasFatalFailure() || IsSkipped()) return;
    any_eph = any_eph || had;
  }
  if (!any_eph)
    GTEST_SKIP() << "blocked: dev-set fixture with ephemeris topics (A26: only Grass01/02 carry them; dev-set slices:"
                 << livo_wpa_testing::namesOf(slices, livo_wpa_testing::SliceKind::DevSet) << ")";
  expectAllSystemsMatched(total, "dev-set slices");
}

// The same on WP-e's synthetic slices: PASS means PASS(合成) (A24) — code and exporter agree, nothing about the data.
TEST(U03SatMap, BagEphemerisVsBrdcSyntheticSlice) {
  LIVO_SYNTHETIC_SLICES(slices);
  bool any_eph = false;
  std::map<Sys, SysCount> total;
  for (const livo_wpa_testing::Slice& sl : slices) {
    bool had = false;
    bagEphemerisVsBrdc(sl, &had, &total);
    if (HasFatalFailure() || IsSkipped()) return;
    any_eph = any_eph || had;
  }
  ASSERT_TRUE(any_eph) << "synthetic slices without ephemeris topics";
  expectAllSystemsMatched(total, "synthetic slices");
  std::printf("[U3 fixture] synthetic slices: PASS(合成) only (A24)\n");
}

// A38 on real data: the dev-set bag iono_params (fixture bytes → WP-e decoder → EphemStore::setIonoFromBag) are recorded
// and never become the GPS Klobuchar — before the slice's BRDC is loaded there is none (hasIono() false, nav.ion_gps
// empty, I = 0 per A37), afterwards it is exactly the BRDC header GPSA/GPSB, whatever order the bag messages and the NAV
// file arrive in. The check discriminates only where a bag set differs from GPSA/GPSB (M3DGR: all eight values differ,
// WP-e #9); at least one dev-set slice has to provide such a set, or the gate would be vacuous.
TEST(U03SatMap, BagIonoParamsAreDiagnosticOnlyFixture) {
  LIVO_DEVSET_SLICES(slices);
  int discriminating = 0, with_iono = 0;
  for (const livo_wpa_testing::Slice& sl : slices) {
    const std::string& slice = sl.path;
    if (livo_wpa_testing::findFile(slice + "/bytes", "iono", ".bin").empty()) {
      std::printf("[U3 iono] %s: no iono_params topic in the slice\n", sl.name.c_str());
      continue;
    }
    struct IonoSink : MeasurementSink {
      std::vector<Klobuchar> iono;
      void pushObservation(ObsEpoch&&) override {}
      void pushEphemeris(const KeplerEph&) override {}
      void pushGloEphemeris(const GloEph&) override {}
      void pushIono(const Klobuchar& k) override { iono.push_back(k); }
      void pushPvt(const ReceiverPvt&) override {}
    } sink;
    livo_wpa_testing::DecodeStats ds;
    std::string why;
    try {
      livo_gnss_io::GnssCommRos1Decoder dec(&satNoOf);
      ASSERT_TRUE(livo_wpa_testing::decodeChannel(dec, slice, livo_wpa_testing::kIonoChannel, sink, &ds, &why)) << why;
    } catch (const std::logic_error& e) {
      LIVO_BLOCKED_IF_STUB(e, "WP-e merge (gnss_io GnssCommRos1Decoder)");
    }
    ASSERT_FALSE(sink.iono.empty()) << slice << ": iono_params topic present but nothing decoded (" << ds.first_refusal << ")";
    ++with_iono;
    const std::string brdc = livo_gnss_io::testsupport::fixtureBrdc(slice);
    ASSERT_FALSE(brdc.empty()) << slice << ": A24 brdc/ missing";
    rtk::NavStore ref;
    ASSERT_GT(ref.loadRinexNav(brdc, true, &why), 0) << why;
    Klobuchar gps;
    ASSERT_TRUE(ref.iono(&gps)) << brdc << ": no GPSA/GPSB in the IGS header";
    int differ = 0;
    for (const Klobuchar& k : sink.iono) {
      bool same = true;
      for (int i = 0; i < 4; ++i) same = same && k.alpha[i] == gps.alpha[i] && k.beta[i] == gps.beta[i];
      differ += same ? 0 : 1;
    }
    discriminating += differ > 0 ? 1 : 0;
    std::printf("[U3 iono] %s: %zu bag iono_params, %d differ from the BRDC GPSA/GPSB (alpha0 bag %.4e / GPSA %.4e)\n",
                sl.name.c_str(), sink.iono.size(), differ, sink.iono.front().alpha[0], gps.alpha[0]);
    const Config cfg = metresCfg();
    for (int order = 0; order < 2; ++order) {
      rtk::NavStore nav;
      EphemStore es(cfg, nav);
      if (order == 0) {
        for (const Klobuchar& k : sink.iono) es.setIonoFromBag(k);
        Klobuchar none;
        EXPECT_FALSE(es.hasIono()) << slice;
        EXPECT_FALSE(es.iono(&none)) << slice;
        EXPECT_FALSE(nav.iono(nullptr)) << slice;
        ASSERT_GT(es.loadRinex(brdc, &why), 0) << why;
      } else {
        ASSERT_GT(es.loadRinex(brdc, &why), 0) << why;
        for (const Klobuchar& k : sink.iono) es.setIonoFromBag(k);
      }
      Klobuchar k, kn, kd;
      ASSERT_TRUE(es.iono(&k)) << slice;
      ASSERT_TRUE(nav.iono(&kn)) << slice;
      EXPECT_EQ(k.source, Source::Rinex) << slice;
      for (int i = 0; i < 4; ++i) {
        EXPECT_EQ(k.alpha[i], gps.alpha[i]) << slice << " alpha" << i << " order " << order;
        EXPECT_EQ(k.beta[i], gps.beta[i]) << slice << " beta" << i << " order " << order;
        EXPECT_EQ(kn.alpha[i], gps.alpha[i]) << slice << " nav.ion_gps alpha" << i << " order " << order;
        EXPECT_EQ(kn.beta[i], gps.beta[i]) << slice << " nav.ion_gps beta" << i << " order " << order;
      }
      EXPECT_EQ(es.bagIonoCount(), static_cast<int>(sink.iono.size())) << slice;
      ASSERT_TRUE(es.bagIono(&kd)) << slice;
      EXPECT_EQ(kd.alpha[0], sink.iono.back().alpha[0]) << slice;   // the diagnostic keeps the last bag message
    }
  }
  if (with_iono == 0)
    GTEST_SKIP() << "blocked: dev-set fixture with an iono_params topic (A26: Grass01/02; dev-set slices:"
                 << livo_wpa_testing::namesOf(slices, livo_wpa_testing::SliceKind::DevSet) << ")";
  EXPECT_GT(discriminating, 0) << "every dev-set bag iono_params set equals the BRDC GPSA/GPSB: the A38 routing check cannot "
                                  "tell the two sources apart on these slices";
}
