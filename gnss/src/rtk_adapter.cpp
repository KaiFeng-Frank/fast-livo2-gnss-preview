// SPDX-License-Identifier: BSD-3-Clause
// rtk_adapter — the ONLY translation unit of the core that includes rtklib.h (DESIGN §12.1).
// Owner: WP-a. Tests: U1, U3, U4, U5, U6.
//
// Every RTKLIB call below is spelled with a leading `::` because most adapter functions carry the RTKLIB name
// (rtk::timeadd wraps ::timeadd); unqualified lookup would find the wrapper and stop there.
#include "internal/rtk_adapter.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <map>
#include <new>
#include <tuple>
#include <vector>

// rtklib.h brings unprefixed macros (PI, CLIGHT, MAXOBS, ...) and C symbols into this TU only.
#include "rtklib.h"

// Layout sanity: the PUBLIC macro set of livo_rtklib must reach this TU unchanged.
static_assert(NFREQ == 3, "livo_rtklib PUBLIC macro NFREQ=3 not visible");
static_assert(NEXOBS == 3, "livo_rtklib PUBLIC macro NEXOBS=3 not visible");
static_assert(MAXSAT <= 255, "obsd_t.sat is uint8_t: MAXSAT must stay <= 255 (P1 assertion)");

// The constants of rtk_adapter.hpp are copies (callers never include rtklib.h); keep them equal.
static_assert(livo_gnss::rtk::kCLight == CLIGHT, "kCLight");
static_assert(livo_gnss::rtk::kOmgE == OMGE, "kOmgE");
static_assert(livo_gnss::rtk::kFreqL1 == FREQL1, "kFreqL1");
static_assert(livo_gnss::rtk::kFreqL2 == FREQL2, "kFreqL2");
static_assert(livo_gnss::rtk::kFreqL5 == FREQL5, "kFreqL5");
static_assert(livo_gnss::rtk::kFreqE5b == FREQE5b, "kFreqE5b");
static_assert(livo_gnss::rtk::kFreqB1I == FREQ1_CMP, "kFreqB1I");
static_assert(livo_gnss::rtk::kFreqG1 == FREQ1_GLO, "kFreqG1");
static_assert(livo_gnss::rtk::kDFreqG1 == DFRQ1_GLO, "kDFreqG1");
static_assert(livo_gnss::rtk::kFreqG2 == FREQ2_GLO, "kFreqG2");
static_assert(livo_gnss::rtk::kDFreqG2 == DFRQ2_GLO, "kDFreqG2");
// Sys mirrors SYS_* bit for bit (types.hpp).
static_assert(static_cast<int>(livo_gnss::Sys::GPS) == SYS_GPS && static_cast<int>(livo_gnss::Sys::SBS) == SYS_SBS &&
                  static_cast<int>(livo_gnss::Sys::GLO) == SYS_GLO && static_cast<int>(livo_gnss::Sys::GAL) == SYS_GAL &&
                  static_cast<int>(livo_gnss::Sys::QZS) == SYS_QZS && static_cast<int>(livo_gnss::Sys::BDS) == SYS_CMP &&
                  static_cast<int>(livo_gnss::Sys::IRN) == SYS_IRN,
              "Sys bit values must equal RTKLIB SYS_*");

namespace livo_gnss {
namespace rtk {
namespace {

inline gtime_t toG(GTime t) {
  gtime_t g;
  g.time = static_cast<time_t>(t.sec);
  g.sec = t.frac;
  return g;
}
inline GTime fromG(gtime_t g) {
  GTime t;
  t.sec = static_cast<int64_t>(g.time);
  t.frac = g.sec;
  return t;
}
inline int bitOf(Sys s) { return static_cast<int>(sysBit(s)); }
Sys sysFromBit(int b) {
  switch (b) {
    case SYS_GPS: return Sys::GPS;
    case SYS_SBS: return Sys::SBS;
    case SYS_GLO: return Sys::GLO;
    case SYS_GAL: return Sys::GAL;
    case SYS_QZS: return Sys::QZS;
    case SYS_CMP: return Sys::BDS;
    case SYS_IRN: return Sys::IRN;
    default: return Sys::Unknown;   // SYS_NONE, SYS_LEO
  }
}
SatId satIdFromNo(int sat) {
  SatId s;
  int prn = 0;
  s.sys = sysFromBit(::satsys(sat, &prn));
  s.prn = prn;
  s.rtk_sat = (s.sys == Sys::Unknown) ? 0 : sat;
  return s;
}
inline void geoToPos(const Geodetic& g, double pos[3]) {
  pos[0] = g.lat_rad;
  pos[1] = g.lon_rad;
  pos[2] = g.h_m;
}

// Ephemeris-store key (m7): toe rounded to the second (every broadcast toe is an integer second; rounding only
// guards against representation noise of a producer), Galileo I/NAV vs F/NAV apart.
inline int64_t toeKey(gtime_t t) { return static_cast<int64_t>(t.time) + (t.sec >= 0.5 ? 1 : 0); }
inline int navTypeOf(const eph_t& e) { return ::satsys(e.sat, nullptr) == SYS_GAL ? galNavTypeOf(e.code) : 0; }

// ---- RTKLIB rinex.c static tables, copied (uraindex/sisa_index/sisa_value; ephemeris.c var_uraeph) ----
const double kUraEph[] = {2.4, 3.4, 4.85, 6.85, 9.65, 13.65, 24.0, 48.0, 96.0, 192.0, 384.0, 768.0, 1536.0, 3072.0, 6144.0, 0.0};
const double kUraValue[] = {2.4, 3.4, 4.85, 6.85, 9.65, 13.65, 24.0, 48.0, 96.0, 192.0, 384.0, 768.0, 1536.0, 3072.0, 6144.0};
const double kUraNominal[] = {2.0, 2.8, 4.0, 5.7, 8.0, 11.3, 16.0, 32.0, 64.0, 128.0, 256.0, 512.0, 1024.0, 2048.0, 4096.0, 8192.0};
constexpr double kStdGalNapa = 500.0;   // ephemeris.c STD_GAL_NAPA
constexpr double kErrEphGlo = 5.0;      // ephemeris.c ERREPH_GLO

// RINEX writes URA as metres (rinex.c uravalue/sisa_value); a store entry read from RINEX reports that value as ura_raw.
double rinexUraMetres(int sys, int sva) {
  if (sys == SYS_GAL) return sisaMetresFromIndex(sva);
  return (0 <= sva && sva < 15) ? kUraNominal[sva] : 8192.0;
}

eph_t toEph(const KeplerEph& k) {
  eph_t e;
  std::memset(&e, 0, sizeof(e));
  e.sat = k.sat.rtk_sat;
  e.iode = k.iode;
  e.iodc = k.iodc;
  e.sva = k.sva;
  e.svh = k.svh;
  e.week = k.week;
  e.code = k.code;
  e.flag = k.flag;
  e.toe = toG(k.toe);
  e.toc = toG(k.toc);
  e.ttr = toG(k.ttr);
  e.A = k.A; e.e = k.e; e.i0 = k.i0; e.OMG0 = k.OMG0; e.omg = k.omg; e.M0 = k.M0; e.deln = k.deln; e.OMGd = k.OMGd; e.idot = k.idot;
  e.crc = k.crc; e.crs = k.crs; e.cuc = k.cuc; e.cus = k.cus; e.cic = k.cic; e.cis = k.cis;
  e.toes = k.toes;
  e.fit = k.fit;
  e.f0 = k.f0; e.f1 = k.f1; e.f2 = k.f2;
  for (int i = 0; i < 6; ++i) e.tgd[i] = k.tgd[i];
  e.Adot = k.Adot;
  e.ndot = k.ndot;
  return e;
}

KeplerEph fromEph(const eph_t& e) {
  KeplerEph k;
  k.sat = satIdFromNo(e.sat);
  k.iode = e.iode; k.iodc = e.iodc; k.sva = e.sva; k.svh = e.svh; k.week = e.week; k.code = e.code; k.flag = e.flag;
  k.toe = fromG(e.toe); k.toc = fromG(e.toc); k.ttr = fromG(e.ttr);
  k.A = e.A; k.e = e.e; k.i0 = e.i0; k.OMG0 = e.OMG0; k.omg = e.omg; k.M0 = e.M0; k.deln = e.deln; k.OMGd = e.OMGd; k.idot = e.idot;
  k.crc = e.crc; k.crs = e.crs; k.cuc = e.cuc; k.cus = e.cus; k.cic = e.cic; k.cis = e.cis;
  k.toes = e.toes; k.fit = e.fit; k.f0 = e.f0; k.f1 = e.f1; k.f2 = e.f2;
  for (int i = 0; i < 6; ++i) k.tgd[i] = e.tgd[i];
  k.Adot = e.Adot; k.ndot = e.ndot;
  return k;
}

geph_t toGeph(const GloEph& g) {
  geph_t o;
  std::memset(&o, 0, sizeof(o));
  o.sat = g.sat.rtk_sat;
  o.iode = g.iode;
  o.frq = g.frq;
  o.svh = g.svh;
  o.flags = g.flags;
  o.sva = g.sva;
  o.age = g.age;
  o.toe = toG(g.toe);
  o.tof = toG(g.tof);
  for (int i = 0; i < 3; ++i) {
    o.pos[i] = g.pos[i];
    o.vel[i] = g.vel[i];
    o.acc[i] = g.acc[i];
  }
  o.taun = g.taun;
  o.gamn = g.gamn;
  o.dtaun = g.dtaun;
  return o;
}

GloEph fromGeph(const geph_t& o) {
  GloEph g;
  g.sat = satIdFromNo(o.sat);
  g.iode = o.iode; g.frq = o.frq; g.svh = o.svh; g.flags = o.flags; g.sva = o.sva; g.age = o.age;
  g.toe = fromG(o.toe); g.tof = fromG(o.tof);
  for (int i = 0; i < 3; ++i) {
    g.pos[i] = o.pos[i];
    g.vel[i] = o.vel[i];
    g.acc[i] = o.acc[i];
  }
  g.taun = o.taun; g.gamn = o.gamn; g.dtaun = o.dtaun;
  return g;
}

}  // namespace

// ---------------------------------------------------------------------------------------------
// Time
// ---------------------------------------------------------------------------------------------
GTime gpst2time(int week, double tow) { return fromG(::gpst2time(week, tow)); }
double time2gpst(GTime t, int* week) { return ::time2gpst(toG(t), week); }
GTime gpst2utc(GTime t) { return fromG(::gpst2utc(toG(t))); }
GTime utc2gpst(GTime t) { return fromG(::utc2gpst(toG(t))); }
GTime gpst2bdt(GTime t) { return fromG(::gpst2bdt(toG(t))); }
GTime bdt2gpst(GTime t) { return fromG(::bdt2gpst(toG(t))); }
GTime bdt2time(int week, double tow) { return fromG(::bdt2time(week, tow)); }
double time2bdt(GTime t, int* week) { return ::time2bdt(toG(t), week); }
GTime gst2time(int week, double tow) { return fromG(::gst2time(week, tow)); }
double time2gst(GTime t, int* week) { return ::time2gst(toG(t), week); }
GTime epoch2time(const double ep[6]) { return fromG(::epoch2time(ep)); }
void time2epoch(GTime t, double ep[6]) { ::time2epoch(toG(t), ep); }
double utc2gmst(GTime utc, double ut1_utc) { return ::utc2gmst(toG(utc), ut1_utc); }
GTime timeadd(GTime t, double sec) { return fromG(::timeadd(toG(t), sec)); }
double timediff(GTime a, GTime b) { return ::timediff(toG(a), toG(b)); }
std::string timeStr(GTime t, int n_frac) {
  char s[64] = {0};
  ::time2str(toG(t), s, n_frac);
  return std::string(s);
}
double leapSecondsAt(GTime gpst) {
  // gpst2utc walks the table from the newest entry; GPST − UTC is the (non-negative) difference.
  const gtime_t g = toG(gpst);
  return ::timediff(g, ::gpst2utc(g));
}

// ---------------------------------------------------------------------------------------------
// Satellite numbering and signals
// ---------------------------------------------------------------------------------------------
int satNo(Sys sys, int prn) { return ::satno(bitOf(sys), prn); }
Sys satSys(int sat, int* prn) {
  int p = 0;
  const Sys s = sysFromBit(::satsys(sat, &p));
  if (prn) *prn = p;
  return s;
}
std::string satIdString(int sat) {
  char id[8] = {0};
  ::satno2id(sat, id);
  return std::string(id);
}
int satIdFromString(const std::string& id) { return ::satid2no(id.c_str()); }
SatId makeSatId(Sys sys, int prn) {
  SatId s;
  s.sys = sys;
  s.prn = prn;
  s.rtk_sat = satNo(sys, prn);
  return s;
}
int maxSat() { return MAXSAT; }
int maxPrnBds() { return MAXPRNCMP; }

int codeToRtk(SignalCode code) {
  switch (code) {
    case SignalCode::L1C: return CODE_L1C;
    case SignalCode::L2X: return CODE_L2X;
    case SignalCode::L2C: return CODE_L2C;
    case SignalCode::L1X: return CODE_L1X;
    case SignalCode::L7X: return CODE_L7X;
    case SignalCode::L2I: return CODE_L2I;
    case SignalCode::L7I: return CODE_L7I;
    case SignalCode::L5X: return CODE_L5X;
    case SignalCode::L5Q: return CODE_L5Q;
    default: return CODE_NONE;
  }
}
SignalCode codeFromRtk(int rtk_code) {
  switch (rtk_code) {
    case CODE_L1C: return SignalCode::L1C;
    case CODE_L2X: return SignalCode::L2X;
    case CODE_L2C: return SignalCode::L2C;
    case CODE_L1X: return SignalCode::L1X;
    case CODE_L7X: return SignalCode::L7X;
    case CODE_L2I: return SignalCode::L2I;
    case CODE_L7I: return SignalCode::L7I;
    case CODE_L5X: return SignalCode::L5X;
    case CODE_L5Q: return SignalCode::L5Q;
    default: return SignalCode::Unknown;
  }
}
int freqIndex(Sys sys, SignalCode code) { return ::code2idx(bitOf(sys), static_cast<uint8_t>(codeToRtk(code))); }
void setGalileoNavSelection(GalNavSelection sel) { ::setseleph(SYS_GAL, static_cast<int>(sel)); }
GalNavSelection galileoNavSelection() { return static_cast<GalNavSelection>(::getseleph(SYS_GAL)); }
int uraIndexFromMetres(double ura_m) {
  int i;
  for (i = 0; i < 15; ++i)   // rinex.c uraindex(): first bound >= value; 15 when above 6144 m
    if (kUraEph[i] >= ura_m) break;
  return i;
}
double uraMetresFromIndex(int sva) { return (sva < 0 || 14 < sva) ? 6144.0 : kUraValue[sva]; }
int galNavTypeOf(int eph_code) { return (eph_code & (1 << 9)) ? 1 : (eph_code & (1 << 8)) ? 2 : 0; }   // ephemeris.c seleph bits

int sisaIndexFromMetres(double v) {
  // rinex.c sisa_index()
  if (v < 0.0 || v > 6.0) return 255;
  if (v <= 0.49) return static_cast<int>(std::round(v / 0.01));
  if (v <= 0.98) return static_cast<int>(std::round((v - 0.5) / 0.02)) + 50;
  if (v <= 1.96) return static_cast<int>(std::round((v - 1.0) / 0.04)) + 75;
  return static_cast<int>(std::round((v - 2.0) / 0.16)) + 100;
}
double sisaMetresFromIndex(int s) {
  // rinex.c sisa_value()
  if (s < 0) return -1.0;
  if (s <= 49) return s * 0.01;
  if (s <= 74) return 0.5 + (s - 50) * 0.02;
  if (s <= 99) return 1.0 + (s - 75) * 0.04;
  if (s <= 125) return 2.0 + (s - 100) * 0.16;
  return -1.0;
}
double ephVarianceFromIndex(Sys sys, int sva) {
  // DESIGN §4.3: σ_eph² from the index EphemStore derived under the judged URA semantics (A16). Same tables as
  // RTKLIB var_uraeph, but written here so a wrong index never hides behind RTKLIB's defaults: a negative index
  // (not derived) is "unknown" (6144 m / NAPA), not var_uraeph's (0.01·ura)² for Galileo.
  if (sys == Sys::GLO) return kErrEphGlo * kErrEphGlo;
  if (sys == Sys::GAL) {
    const double m = sisaMetresFromIndex(sva);
    return m < 0.0 ? kStdGalNapa * kStdGalNapa : m * m;
  }
  const double m = uraMetresFromIndex(sva);
  return m * m;
}
int patchLevel() {
  int level = 0;
  if (MAXPRNCMP >= 63) level |= kPatchP1;
#ifdef RTKX_P2
  level |= kPatchP2;
#endif
  return level;
}

// ---------------------------------------------------------------------------------------------
// Coordinates / models
// ---------------------------------------------------------------------------------------------
Geodetic ecef2pos(const Eigen::Vector3d& r_ecef) {
  const double r[3] = {r_ecef[0], r_ecef[1], r_ecef[2]};
  double pos[3];
  ::ecef2pos(r, pos);
  Geodetic g;
  g.lat_rad = pos[0];
  g.lon_rad = pos[1];
  g.h_m = pos[2];
  return g;
}
Eigen::Vector3d pos2ecef(const Geodetic& geo) {
  double pos[3], r[3];
  geoToPos(geo, pos);
  ::pos2ecef(pos, r);
  return Eigen::Vector3d(r[0], r[1], r[2]);
}
Eigen::Matrix3d enuRotation(const Geodetic& geo) {
  double pos[3], E[9];
  geoToPos(geo, pos);
  ::xyz2enu(pos, E);   // column-major ECEF→ENU (rtkcmn.c xyz2enu); R_{E<-N} is its transpose (DESIGN §1)
  Eigen::Matrix3d R_NE;
  for (int c = 0; c < 3; ++c)
    for (int r = 0; r < 3; ++r) R_NE(r, c) = E[r + 3 * c];
  return R_NE.transpose();
}
double geodist(const Eigen::Vector3d& rs, const Eigen::Vector3d& rr, Eigen::Vector3d* e) {
  const double s[3] = {rs[0], rs[1], rs[2]}, r[3] = {rr[0], rr[1], rr[2]};
  double ee[3] = {0.0, 0.0, 0.0};
  const double d = ::geodist(s, r, ee);
  if (e) *e = Eigen::Vector3d(ee[0], ee[1], ee[2]);
  return d;
}
void satAzel(const Geodetic& rcv, const Eigen::Vector3d& e, double* az, double* el) {
  double pos[3], azel[2];
  const double ev[3] = {e[0], e[1], e[2]};
  geoToPos(rcv, pos);
  ::satazel(pos, ev, azel);
  if (az) *az = azel[0];
  if (el) *el = azel[1];
}
double ionModelGpsL1(GTime t, const Klobuchar& ion, const Geodetic& rcv, double az, double el) {
  // DESIGN §8 WAIT_NAV: without parameters I = 0 (the caller carries σ_ion = 5 m). RTKLIB ionmodel() would silently
  // substitute its built-in 2004 set for an all-zero one (7.6 m at 30° elevation), as pntpos does with an empty
  // nav.ion_gps; that fallback is kept out of the core (review m4).
  if (!ion.valid()) return 0.0;
  const double p[8] = {ion.alpha[0], ion.alpha[1], ion.alpha[2], ion.alpha[3], ion.beta[0], ion.beta[1], ion.beta[2], ion.beta[3]};
  double pos[3];
  geoToPos(rcv, pos);
  const double azel[2] = {az, el};
  return ::ionmodel(toG(t), p, pos, azel);
}
double tropModel(GTime t, const Geodetic& rcv, double az, double el, double humi) {
  double pos[3];
  geoToPos(rcv, pos);
  const double azel[2] = {az, el};
  return ::tropmodel(toG(t), pos, azel, humi);
}

// ---------------------------------------------------------------------------------------------
// NavStore
// The canonical store is two sorted vectors (key → entry + provenance); nav.eph / nav.geph are rebuilt from them
// lazily (sync) before any RTKLIB routine reads nav_t. Sorting by (sat, toe, nav_type) makes the array order — and
// with it RTKLIB's tie-breaks (seleph "t<=tmin" keeps the later index; gettgd/sat2freq take the first entry of a
// satellite) — independent of the arrival order of the messages.
// ---------------------------------------------------------------------------------------------
struct NavStore::Impl {
  struct KEntry {
    eph_t eph;
    int64_t toe_key;
    int nav_type;
    Source src;
    double ura_raw;
    double t_record;
    bool bag_delivered;   // m3: a non-RINEX source delivered this key at least once (survives replacement)
  };
  struct GEntry {
    geph_t geph;
    int64_t toe_key;
    Source src;
    double ura_raw;
    double t_record;
    bool bag_delivered;
  };
  nav_t nav;
  std::vector<KEntry> keph;   // sorted by (sat, toe_key, nav_type), unique
  std::vector<GEntry> geph;   // sorted by (sat, toe_key), unique
  bool dirty = false;

  Impl() { std::memset(&nav, 0, sizeof(nav)); }
  ~Impl() { ::freenav(&nav, 0xFF); }

  static bool kless(const KEntry& a, int sat, int64_t toe, int nt) {
    return std::tie(a.eph.sat, a.toe_key, a.nav_type) < std::tie(sat, toe, nt);
  }
  static bool gless(const GEntry& a, int sat, int64_t toe) { return std::tie(a.geph.sat, a.toe_key) < std::tie(sat, toe); }
  std::vector<KEntry>::iterator kfind(int sat, int64_t toe, int nt) {
    auto it = std::lower_bound(keph.begin(), keph.end(), 0, [&](const KEntry& a, int) { return kless(a, sat, toe, nt); });
    return (it != keph.end() && it->eph.sat == sat && it->toe_key == toe && it->nav_type == nt) ? it : keph.end();
  }
  std::vector<GEntry>::iterator gfind(int sat, int64_t toe) {
    auto it = std::lower_bound(geph.begin(), geph.end(), 0, [&](const GEntry& a, int) { return gless(a, sat, toe); });
    return (it != geph.end() && it->geph.sat == sat && it->toe_key == toe) ? it : geph.end();
  }
  // [first, last) of one satellite in keph / geph (== its index range in nav.eph / nav.geph after sync()).
  std::pair<size_t, size_t> krange(int sat) const {
    auto lo = std::lower_bound(keph.begin(), keph.end(), sat, [](const KEntry& a, int s) { return a.eph.sat < s; });
    auto hi = std::upper_bound(lo, keph.end(), sat, [](int s, const KEntry& a) { return s < a.eph.sat; });
    return {static_cast<size_t>(lo - keph.begin()), static_cast<size_t>(hi - keph.begin())};
  }
  std::pair<size_t, size_t> grange(int sat) const {
    auto lo = std::lower_bound(geph.begin(), geph.end(), sat, [](const GEntry& a, int s) { return a.geph.sat < s; });
    auto hi = std::upper_bound(lo, geph.end(), sat, [](int s, const GEntry& a) { return s < a.geph.sat; });
    return {static_cast<size_t>(lo - geph.begin()), static_cast<size_t>(hi - geph.begin())};
  }
  bool insertK(KEntry ke, bool replace) {
    auto it = std::lower_bound(keph.begin(), keph.end(), 0,
                               [&](const KEntry& a, int) { return kless(a, ke.eph.sat, ke.toe_key, ke.nav_type); });
    if (it != keph.end() && it->eph.sat == ke.eph.sat && it->toe_key == ke.toe_key && it->nav_type == ke.nav_type) {
      // The provenance mark is kept whichever record wins (m3); it is metadata, so "not replaced" stays false.
      if (!replace) {
        it->bag_delivered = it->bag_delivered || ke.bag_delivered;
        return false;
      }
      ke.bag_delivered = ke.bag_delivered || it->bag_delivered;
      *it = ke;
    } else {
      keph.insert(it, ke);
    }
    dirty = true;
    return true;
  }
  bool insertG(GEntry ge, bool replace) {
    auto it = std::lower_bound(geph.begin(), geph.end(), 0,
                               [&](const GEntry& a, int) { return gless(a, ge.geph.sat, ge.toe_key); });
    if (it != geph.end() && it->geph.sat == ge.geph.sat && it->toe_key == ge.toe_key) {
      if (!replace) {
        it->bag_delivered = it->bag_delivered || ge.bag_delivered;
        return false;
      }
      ge.bag_delivered = ge.bag_delivered || it->bag_delivered;
      *it = ge;
    } else {
      geph.insert(it, ge);
    }
    dirty = true;
    return true;
  }
  void sync() {
    if (!dirty) return;
    const size_t n = keph.size(), ng = geph.size();
    std::free(nav.eph);
    std::free(nav.geph);
    nav.eph = nullptr;
    nav.geph = nullptr;
    nav.n = nav.nmax = nav.ng = nav.ngmax = 0;
    if (n) {
      nav.eph = static_cast<eph_t*>(std::malloc(sizeof(eph_t) * n));
      if (!nav.eph) throw std::bad_alloc();
      for (size_t i = 0; i < n; ++i) nav.eph[i] = keph[i].eph;
      nav.n = nav.nmax = static_cast<int>(n);
    }
    if (ng) {
      nav.geph = static_cast<geph_t*>(std::malloc(sizeof(geph_t) * ng));
      if (!nav.geph) throw std::bad_alloc();
      for (size_t i = 0; i < ng; ++i) nav.geph[i] = geph[i].geph;
      nav.ng = nav.ngmax = static_cast<int>(ng);
    }
    dirty = false;
  }
};

NavStore::NavStore() : impl_(new Impl) {}
NavStore::~NavStore() = default;

bool NavStore::add(const KeplerEph& k, bool replace_existing) {
  const int sys = ::satsys(k.sat.rtk_sat, nullptr);
  if (!(sys & (SYS_GPS | SYS_GAL | SYS_QZS | SYS_CMP | SYS_IRN))) return false;   // unmapped or not a Kepler system
  Impl::KEntry ke;
  ke.eph = toEph(k);
  ke.toe_key = toeKey(ke.eph.toe);
  ke.nav_type = navTypeOf(ke.eph);
  ke.src = k.source;
  ke.ura_raw = k.ura_raw;
  ke.t_record = k.t_record;
  ke.bag_delivered = k.source != Source::Rinex;
  return impl_->insertK(ke, replace_existing);
}

bool NavStore::add(const GloEph& g, bool replace_existing) {
  if (::satsys(g.sat.rtk_sat, nullptr) != SYS_GLO) return false;
  Impl::GEntry ge;
  ge.geph = toGeph(g);
  ge.toe_key = toeKey(ge.geph.toe);
  ge.src = g.source;
  ge.ura_raw = g.ura_raw;
  ge.t_record = g.t_record;
  ge.bag_delivered = g.source != Source::Rinex;
  return impl_->insertG(ge, replace_existing);
}

bool NavStore::has(const SatId& sat, GTime toe, int nav_type) const {
  if (sat.sys == Sys::GLO || ::satsys(sat.rtk_sat, nullptr) == SYS_GLO) return impl_->gfind(sat.rtk_sat, toeKey(toG(toe))) != impl_->geph.end();
  return impl_->kfind(sat.rtk_sat, toeKey(toG(toe)), nav_type) != impl_->keph.end();
}

bool NavStore::get(const SatId& sat, GTime toe, KeplerEph* out, int nav_type) const {
  auto it = impl_->kfind(sat.rtk_sat, toeKey(toG(toe)), nav_type);
  if (it == impl_->keph.end()) return false;
  if (out) {
    *out = fromEph(it->eph);
    out->source = it->src;
    out->t_record = it->t_record;
    out->ura_raw = it->ura_raw;   // bag: as delivered; RINEX: the SV-accuracy metres the file carried (nominal of sva)
  }
  return true;
}

bool NavStore::get(const SatId& sat, GTime toe, GloEph* out) const {
  auto it = impl_->gfind(sat.rtk_sat, toeKey(toG(toe)));
  if (it == impl_->geph.end()) return false;
  if (out) {
    *out = fromGeph(it->geph);
    out->source = it->src;
    out->t_record = it->t_record;
    out->ura_raw = it->ura_raw;
  }
  return true;
}

Source NavStore::sourceOf(const SatId& sat, GTime toe, int nav_type) const {
  if (::satsys(sat.rtk_sat, nullptr) == SYS_GLO) {
    auto it = impl_->gfind(sat.rtk_sat, toeKey(toG(toe)));
    return it == impl_->geph.end() ? Source::Synthetic : it->src;
  }
  auto it = impl_->kfind(sat.rtk_sat, toeKey(toG(toe)), nav_type);
  return it == impl_->keph.end() ? Source::Synthetic : it->src;
}

void NavStore::setIono(const Klobuchar& ion) {
  for (int i = 0; i < 4; ++i) {
    impl_->nav.ion_gps[i] = ion.alpha[i];
    impl_->nav.ion_gps[i + 4] = ion.beta[i];
  }
}

bool NavStore::iono(Klobuchar* out) const {
  const double* p = impl_->nav.ion_gps;
  bool any = false;
  for (int i = 0; i < 8; ++i) any = any || p[i] != 0.0;
  if (!any) return false;
  if (out) {
    for (int i = 0; i < 4; ++i) {
      out->alpha[i] = p[i];
      out->beta[i] = p[i + 4];
    }
  }
  return true;
}

void NavStore::setGloFcn(int prn, int frq) {
  if (prn < 1 || prn > 32) return;
  impl_->nav.glo_fcn[prn - 1] = frq + 8;
}

int NavStore::gloFcn(int prn) const {
  if (prn < 1 || prn > 32 || impl_->nav.glo_fcn[prn - 1] <= 0) return -100;
  return impl_->nav.glo_fcn[prn - 1] - 8;
}

int NavStore::loadRinexNav(const std::string& path, bool replace_existing, std::string* why) {
  // readrnxt() returns 0 both for "no such file" (expath) and for "no data": check readability first.
  if (FILE* fp = std::fopen(path.c_str(), "r")) {
    std::fclose(fp);
  } else {
    if (why) *why = "cannot open RINEX NAV file: " + path;
    return -1;
  }
  nav_t tmp;
  std::memset(&tmp, 0, sizeof(tmp));
  gtime_t t0;
  t0.time = 0;
  t0.sec = 0.0;
  // SBAS and NavIC are never used (DESIGN §12.3): skip them at parse time (IGS BRDC carries ~10k SBAS records/day).
  const int stat = ::readrnxt(path.c_str(), 1, t0, t0, 0.0, "-SYS=GRECJ", nullptr, &tmp, nullptr);
  if (stat < 0) {
    ::freenav(&tmp, 0xFF);
    if (why) *why = "readrnxt error: " + path;
    return -1;
  }
  Impl& im = *impl_;
  int changed = 0;
  // Merge in file order. Several records of one key in RINEX (IGS BRDC repeats Galileo I/NAV per data source,
  // codes 513/516/517, with equal parameters up to print precision): the first one read wins, deterministically.
  // A RINEX record meets a bag record of the same key: replaced iff replace_existing (A1: RINEX is authoritative).
  auto handleK = [&](const eph_t& e, std::vector<Impl::KEntry>* pending, std::map<std::tuple<int, int64_t, int>, bool>* seen) {
    Impl::KEntry ke;
    ke.eph = e;
    ke.toe_key = toeKey(e.toe);
    ke.nav_type = navTypeOf(e);
    ke.src = Source::Rinex;
    ke.ura_raw = rinexUraMetres(::satsys(e.sat, nullptr), e.sva);
    ke.t_record = 0.0;
    ke.bag_delivered = false;
    const auto key = std::make_tuple(e.sat, ke.toe_key, ke.nav_type);
    if ((*seen)[key]) return;   // later duplicate within this load
    (*seen)[key] = true;
    auto it = im.kfind(e.sat, ke.toe_key, ke.nav_type);
    if (it != im.keph.end()) {
      if (it->src == Source::Rinex || !replace_existing) return;
      ke.bag_delivered = it->bag_delivered;   // the bag delivered this key (m3)
      *it = ke;
      im.dirty = true;
      ++changed;
      return;
    }
    pending->push_back(ke);
  };
  std::vector<Impl::KEntry> kpend;
  std::map<std::tuple<int, int64_t, int>, bool> kseen;
  for (int i = 0; i < tmp.n; ++i) {
    if (tmp.eph[i].sat <= 0) continue;
    handleK(tmp.eph[i], &kpend, &kseen);
  }
  std::vector<Impl::GEntry> gpend;
  std::map<std::pair<int, int64_t>, bool> gseen;
  for (int i = 0; i < tmp.ng; ++i) {
    const geph_t& g = tmp.geph[i];
    if (g.sat <= 0) continue;
    Impl::GEntry ge;
    ge.geph = g;
    ge.toe_key = toeKey(g.toe);
    ge.src = Source::Rinex;
    ge.ura_raw = 0.0;
    ge.t_record = 0.0;
    ge.bag_delivered = false;
    const auto key = std::make_pair(g.sat, ge.toe_key);
    if (gseen[key]) continue;
    gseen[key] = true;
    auto it = im.gfind(g.sat, ge.toe_key);
    if (it != im.geph.end()) {
      if (it->src == Source::Rinex || !replace_existing) continue;
      ge.bag_delivered = it->bag_delivered;
      *it = ge;
      im.dirty = true;
      ++changed;
      continue;
    }
    gpend.push_back(ge);
  }
  if (!kpend.empty()) {
    im.keph.insert(im.keph.end(), kpend.begin(), kpend.end());
    std::sort(im.keph.begin(), im.keph.end(), [](const Impl::KEntry& a, const Impl::KEntry& b) {
      return std::tie(a.eph.sat, a.toe_key, a.nav_type) < std::tie(b.eph.sat, b.toe_key, b.nav_type);
    });
    im.dirty = true;
    changed += static_cast<int>(kpend.size());
  }
  if (!gpend.empty()) {
    im.geph.insert(im.geph.end(), gpend.begin(), gpend.end());
    std::sort(im.geph.begin(), im.geph.end(),
              [](const Impl::GEntry& a, const Impl::GEntry& b) { return std::tie(a.geph.sat, a.toe_key) < std::tie(b.geph.sat, b.toe_key); });
    im.dirty = true;
    changed += static_cast<int>(gpend.size());
  }
  // Header GPSA/GPSB: only while nothing is stored yet, so the first NAV file that carries them stays in force (A38: the
  // GPS Klobuchar comes from RINEX alone; EphemStore never writes bag iono_params here).
  bool have = false, file_has = false;
  for (int i = 0; i < 8; ++i) {
    have = have || im.nav.ion_gps[i] != 0.0;
    file_has = file_has || tmp.ion_gps[i] != 0.0;
  }
  if (!have && file_has)
    for (int i = 0; i < 8; ++i) im.nav.ion_gps[i] = tmp.ion_gps[i];
  ::freenav(&tmp, 0xFF);
  return changed;
}

int NavStore::countEph(Sys sys) const {
  int n = 0;
  for (const auto& e : impl_->keph) n += (::satsys(e.eph.sat, nullptr) == bitOf(sys)) ? 1 : 0;
  return n;
}
int NavStore::countGeph() const { return static_cast<int>(impl_->geph.size()); }

std::vector<EphKey> NavStore::coverage() const {
  std::vector<EphKey> out;
  out.reserve(impl_->keph.size() + impl_->geph.size());
  for (const auto& e : impl_->keph) {
    EphKey k;
    k.sat = satIdFromNo(e.eph.sat);
    k.toe = fromG(e.eph.toe);
    k.nav_type = e.nav_type;
    k.source = e.src;
    k.is_glo = false;
    k.bag_delivered = e.bag_delivered;
    out.push_back(k);
  }
  for (const auto& g : impl_->geph) {
    EphKey k;
    k.sat = satIdFromNo(g.geph.sat);
    k.toe = fromG(g.geph.toe);
    k.nav_type = 0;
    k.source = g.src;
    k.is_glo = true;
    k.bag_delivered = g.bag_delivered;
    out.push_back(k);
  }
  // one order for both kinds: satellite number, then toe
  std::stable_sort(out.begin(), out.end(), [](const EphKey& a, const EphKey& b) {
    if (a.sat.rtk_sat != b.sat.rtk_sat) return a.sat.rtk_sat < b.sat.rtk_sat;
    return a.toe.diff(b.toe) < 0.0;
  });
  return out;
}

void NavStore::clear() {
  impl_->keph.clear();
  impl_->geph.clear();
  impl_->dirty = true;
  std::memset(impl_->nav.ion_gps, 0, sizeof(impl_->nav.ion_gps));
  std::memset(impl_->nav.glo_fcn, 0, sizeof(impl_->nav.glo_fcn));
  impl_->sync();
}

// RTKLIB sat2freq (GLONASS channel from the store's geph / glo_fcn), defined here because it needs NavStore::Impl.
double sat2freq(int sat, SignalCode code, const NavStore& nav) {
  NavStore::Impl* im = nav.impl();
  im->sync();
  return ::sat2freq(sat, static_cast<uint8_t>(codeToRtk(code)), &im->nav);
}

// ---------------------------------------------------------------------------------------------
// EphSnapshot
// ---------------------------------------------------------------------------------------------
struct EphSnapshot::Impl {
  bool is_glo = false;
  bool valid = false;
  eph_t eph;
  geph_t geph;
  Impl() {
    std::memset(&eph, 0, sizeof(eph));
    std::memset(&geph, 0, sizeof(geph));
  }
};

EphSnapshot::EphSnapshot() : impl_(new Impl) {}
EphSnapshot::~EphSnapshot() = default;
EphSnapshot::EphSnapshot(const EphSnapshot& o) : impl_(new Impl(*o.impl_)) {}
EphSnapshot& EphSnapshot::operator=(const EphSnapshot& o) {
  if (this != &o) *impl_ = *o.impl_;
  return *this;
}
bool EphSnapshot::valid() const { return impl_->valid; }
SatId EphSnapshot::sat() const {
  if (!impl_->valid) return SatId();
  return satIdFromNo(impl_->is_glo ? impl_->geph.sat : impl_->eph.sat);
}
GTime EphSnapshot::toe() const {
  if (!impl_->valid) return GTime();
  return fromG(impl_->is_glo ? impl_->geph.toe : impl_->eph.toe);
}

// ---------------------------------------------------------------------------------------------
// SatSolver
// ---------------------------------------------------------------------------------------------
namespace {

// Mirror of ephemeris.c seleph() (iode = -1) over one satellite's slice of the store: same age limits, same Galileo
// data-source and AOD rules, same "t <= tmin keeps the later index" tie-break. Because nav.eph is exactly this vector
// after sync(), RTKLIB's own satposs()/pntpos() select the same record (U4 checks bit identity against satposs).
const eph_t* selectEph(const NavStore::Impl& im, gtime_t time, int sat) {
  const int sys = ::satsys(sat, nullptr);
  double tmax;
  switch (sys) {
    case SYS_GPS: tmax = MAXDTOE + 1.0; break;
    case SYS_GAL: tmax = MAXDTOE_GAL; break;
    case SYS_QZS: tmax = MAXDTOE_QZS + 1.0; break;
    case SYS_CMP: tmax = MAXDTOE_CMP + 1.0; break;
    case SYS_IRN: tmax = MAXDTOE_IRN + 1.0; break;
    default: tmax = MAXDTOE + 1.0; break;
  }
  double tmin = tmax + 1.0;
  const eph_t* best = nullptr;
  const auto r = im.krange(sat);
  for (size_t i = r.first; i < r.second; ++i) {
    const eph_t& e = im.keph[i].eph;
    if (sys == SYS_GAL) {
      const int sel = ::getseleph(SYS_GAL);
      if (sel == 1 && !(e.code & (1 << 9))) continue;   // I/NAV
      if (sel == 2 && !(e.code & (1 << 8))) continue;   // F/NAV
      if (::timediff(e.toe, time) >= 0.0) continue;     // AOD <= 0
    }
    const double t = std::fabs(::timediff(e.toe, time));
    if (t > tmax) continue;
    if (t <= tmin) {
      best = &e;
      tmin = t;
    }
  }
  return best;
}

// Mirror of ephemeris.c selgeph() (iode = -1).
const geph_t* selectGeph(const NavStore::Impl& im, gtime_t time, int sat) {
  const double tmax = MAXDTOE_GLO;
  double tmin = tmax + 1.0;
  const geph_t* best = nullptr;
  const auto r = im.grange(sat);
  for (size_t i = r.first; i < r.second; ++i) {
    const geph_t& g = im.geph[i].geph;
    const double t = std::fabs(::timediff(g.toe, time));
    if (t > tmax) continue;
    if (t <= tmin) {
      best = &g;
      tmin = t;
    }
  }
  return best;
}

void fillCommon(const eph_t& e, SatState* out) {
  int prn = 0;
  const Sys s = sysFromBit(::satsys(e.sat, &prn));
  out->svh = e.svh;
  out->tgd_s[0] = e.tgd[0];
  out->tgd_s[1] = e.tgd[1];
  out->toe = fromG(e.toe);
  out->iode = e.iode;
  out->sva = e.sva;
  out->nav_type = navTypeOf(e);
  out->var_eph = ephVarianceFromIndex(s, e.sva);
}

// satposs() for one satellite with a known record (ephemeris.c:804 → :822, then ephpos :566-568 and :588-589):
// t_tx = t_rx − P/c − δt_s with δt_s from eph2clk (no relativity), position/clock at t_tx, velocity and clock drift
// by the same 1 ms forward difference. Operation for operation the same arithmetic, so the bits agree.
bool evalKepler(const eph_t& eph, gtime_t t_rx, double pr, SatState* out) {
  if (eph.A <= 0.0 || !std::isfinite(pr)) return false;
  const gtime_t t1 = ::timeadd(t_rx, -pr / CLIGHT);
  const double dt = ::eph2clk(t1, &eph);
  const gtime_t ttx = ::timeadd(t1, -dt);
  const double tt = 1E-3;
  double rs[3], rst[3], dts = 0.0, dtst = 0.0, var = 0.0;
  ::eph2pos(ttx, &eph, rs, &dts, &var);
  ::eph2pos(::timeadd(ttx, tt), &eph, rst, &dtst, &var);
  for (int i = 0; i < 3; ++i) {
    out->r_ecef[i] = rs[i];
    out->v_ecef[i] = (rst[i] - rs[i]) / tt;
  }
  out->dts = dts;
  out->ddts = (dtst - dts) / tt;
  if (dts == 0.0) {   // satposs :833-836 fallback: broadcast clock without relativity, no drift
    out->dts = ::eph2clk(ttx, &eph);
    out->ddts = 0.0;
  }
  out->t_tx = fromG(ttx);
  fillCommon(eph, out);
  out->ok = true;
  return true;
}

bool evalGlo(const geph_t& g, gtime_t t_rx, double pr, SatState* out) {
  if (std::fabs(g.taun) > 1.0 || !std::isfinite(pr)) return false;   // ephclk(): invalid data guard
  const gtime_t t1 = ::timeadd(t_rx, -pr / CLIGHT);
  const double dt = ::geph2clk(t1, &g);
  const gtime_t ttx = ::timeadd(t1, -dt);
  const double tt = 1E-3;
  double rs[3], rst[3], dts = 0.0, dtst = 0.0, var = 0.0;
  ::geph2pos(ttx, &g, rs, &dts, &var);
  ::geph2pos(::timeadd(ttx, tt), &g, rst, &dtst, &var);
  for (int i = 0; i < 3; ++i) {
    out->r_ecef[i] = rs[i];
    out->v_ecef[i] = (rst[i] - rs[i]) / tt;
  }
  out->dts = dts;
  out->ddts = (dtst - dts) / tt;
  if (dts == 0.0) {   // satposs fallback (see evalKepler)
    out->dts = ::geph2clk(ttx, &g);
    out->ddts = 0.0;
  }
  out->t_tx = fromG(ttx);
  out->svh = g.svh;
  out->tgd_s[0] = g.dtaun;   // GLONASS: delay between L1 and L2 (prange G1 branch), see rtk_adapter.hpp
  out->tgd_s[1] = 0.0;
  out->toe = fromG(g.toe);
  out->iode = g.iode;
  out->sva = g.sva;
  out->nav_type = 0;
  out->var_eph = ephVarianceFromIndex(Sys::GLO, g.sva);
  out->ok = true;
  return true;
}

}  // namespace

SatSolver::SatSolver(const NavStore& nav) : nav_(nav) {}

bool SatSolver::satState(const SatId& sat, GTime t_rx, double pseudorange_m, SatState* out, EphSnapshot* snap_out) const {
  *out = SatState();
  NavStore::Impl* im = nav_.impl();
  im->sync();
  const gtime_t trx = toG(t_rx);
  const int sys = ::satsys(sat.rtk_sat, nullptr);
  if (sys == SYS_GLO) {
    const geph_t* g = selectGeph(*im, trx, sat.rtk_sat);
    if (!g) return false;
    const geph_t copy = *g;
    if (!evalGlo(copy, trx, pseudorange_m, out)) return false;
    if (snap_out) {
      EphSnapshot::Impl* s = snap_out->impl();
      s->is_glo = true;
      s->geph = copy;
      std::memset(&s->eph, 0, sizeof(s->eph));
      s->valid = true;
    }
    return true;
  }
  if (!(sys & (SYS_GPS | SYS_GAL | SYS_QZS | SYS_CMP | SYS_IRN))) return false;
  const eph_t* e = selectEph(*im, trx, sat.rtk_sat);
  if (!e) return false;
  const eph_t copy = *e;
  if (!evalKepler(copy, trx, pseudorange_m, out)) return false;
  if (snap_out) {
    EphSnapshot::Impl* s = snap_out->impl();
    s->is_glo = false;
    s->eph = copy;
    std::memset(&s->geph, 0, sizeof(s->geph));
    s->valid = true;
  }
  return true;
}

bool SatSolver::satStateFromSnapshot(const EphSnapshot& snap, GTime t_rx, double pseudorange_m, SatState* out) {
  *out = SatState();
  if (!snap.valid()) return false;
  const EphSnapshot::Impl* s = snap.impl();
  return s->is_glo ? evalGlo(s->geph, toG(t_rx), pseudorange_m, out) : evalKepler(s->eph, toG(t_rx), pseudorange_m, out);
}

bool SatSolver::ephPos(const SatId& sat, GTime t, Eigen::Vector3d* rs, double* dts, double* var) const {
  NavStore::Impl* im = nav_.impl();
  im->sync();
  const gtime_t tg = toG(t);
  double r[3] = {0.0, 0.0, 0.0}, d = 0.0, v = 0.0;
  const int sys = ::satsys(sat.rtk_sat, nullptr);
  if (sys == SYS_GLO) {
    const geph_t* g = selectGeph(*im, tg, sat.rtk_sat);
    if (!g) return false;
    ::geph2pos(tg, g, r, &d, &v);
    v = ephVarianceFromIndex(Sys::GLO, g->sva);
  } else {
    const eph_t* e = selectEph(*im, tg, sat.rtk_sat);
    if (!e || e->A <= 0.0) return false;
    ::eph2pos(tg, e, r, &d, &v);
    v = ephVarianceFromIndex(sysFromBit(sys), e->sva);
  }
  if (rs) *rs = Eigen::Vector3d(r[0], r[1], r[2]);
  if (dts) *dts = d;
  if (var) *var = v;
  return true;
}

bool SatSolver::ephHealthy(const SatId& sat, GTime t, double max_age_s, double* age_s) const {
  NavStore::Impl* im = nav_.impl();
  im->sync();
  const gtime_t tg = toG(t);
  int svh = 0;
  double age = 0.0;
  const int sys = ::satsys(sat.rtk_sat, nullptr);
  if (sys == SYS_GLO) {
    const geph_t* g = selectGeph(*im, tg, sat.rtk_sat);
    if (!g) return false;
    svh = g->svh;
    age = std::fabs(::timediff(tg, g->toe));
  } else {
    const eph_t* e = selectEph(*im, tg, sat.rtk_sat);
    if (!e) return false;
    svh = e->svh;
    age = std::fabs(::timediff(tg, e->toe));
  }
  if (age_s) *age_s = age;
  return svhHealthy(sysFromBit(sys), svh) && age <= max_age_s;
}

// ---------------------------------------------------------------------------------------------
// pntpos reference solver
// ---------------------------------------------------------------------------------------------
namespace {

// One SatObs → obsd_t: primary signal in slot 0 (what pntpos uses), further signals at their code2idx slot.
bool toObsd(const SatObs& so, gtime_t t_epoch, obsd_t* o) {
  std::memset(o, 0, sizeof(*o));
  if (so.sat.rtk_sat <= 0 || so.sig.empty()) return false;
  o->time = so.t_rx.valid() ? toG(so.t_rx) : t_epoch;
  o->sat = static_cast<uint8_t>(so.sat.rtk_sat);
  o->rcv = 1;
  for (size_t k = 0; k < so.sig.size(); ++k) {
    const SignalObs& s = so.sig[k];
    int slot = 0;
    if (k > 0) {
      slot = ::code2idx(::satsys(so.sat.rtk_sat, nullptr), static_cast<uint8_t>(codeToRtk(s.code)));
      if (slot <= 0 || slot >= NFREQ || o->code[slot] != CODE_NONE) continue;
    }
    o->code[slot] = static_cast<uint8_t>(codeToRtk(s.code));
    o->P[slot] = s.psrValid() ? s.psr_m : 0.0;
    o->L[slot] = s.cpValid() ? s.cp_cyc : 0.0;
    o->D[slot] = static_cast<float>(s.dopp_hz);
    o->SNR[slot] = static_cast<float>(s.cn0_dbhz);
    o->LLI[slot] = s.lli;
    o->Pstd[slot] = static_cast<float>(s.psr_std_m);
    // obsd_t.Lstd is in cycles (rtklib.h:598), SignalObs::cp_std_m in metres; pntpos never reads it (m9).
    o->Lstd[slot] = s.freq_hz > 0.0 ? static_cast<float>(s.cp_std_m * s.freq_hz / CLIGHT) : 0.0f;
  }
  return true;
}

prcopt_t makeOpt(const SppRefOptions& o) {
  prcopt_t opt = prcopt_default;
  opt.mode = PMODE_SINGLE;   // otherwise pntpos forces IONOOPT_BRDC/TROPOPT_SAAS (pntpos.c:678-681)
  opt.nf = 1;
  opt.navsys = SYS_GPS | SYS_GAL | SYS_QZS | SYS_CMP | (o.use_glo_psr ? SYS_GLO : 0);
  opt.elmin = o.el_mask_deg * D2R;
  opt.ionoopt = o.ionoopt;
  opt.tropopt = o.tropopt;
  opt.sateph = EPHOPT_BRDC;
  opt.posopt[4] = o.raim_fde ? 1 : 0;
  // o.err_brdci cannot reach pntpos: ERR_BRDCI is a macro (0.5) in pntpos.c; the reference row keeps RTKLIB's value.
  return opt;
}

}  // namespace

struct SppRef::Impl {
  prcopt_t opt;
  Impl() { opt = prcopt_default; }
};
SppRef::SppRef() : impl_(new Impl) {}
SppRef::~SppRef() = default;

bool SppRef::solve(const ObsEpoch& epoch, const NavStore& nav, const SppRefOptions& o, SppRefResult* out) const {
  *out = SppRefResult();
  NavStore::Impl* im = nav.impl();
  im->sync();
  const gtime_t t_epoch = toG(epoch.t_rx);
  // Sorted by satellite, one record per satellite (RTKLIB sortobs() convention; rescode drops *both* members of an
  // adjacent duplicate pair, so duplicates are removed here, keeping the first).
  std::vector<obsd_t> obs;
  obs.reserve(epoch.sats.size());
  for (const SatObs& so : epoch.sats) {
    obsd_t d;
    if (toObsd(so, t_epoch, &d)) obs.push_back(d);
  }
  std::stable_sort(obs.begin(), obs.end(), [](const obsd_t& a, const obsd_t& b) { return a.sat < b.sat; });
  obs.erase(std::unique(obs.begin(), obs.end(), [](const obsd_t& a, const obsd_t& b) { return a.sat == b.sat; }), obs.end());
  if (obs.size() > static_cast<size_t>(MAXOBS)) obs.resize(MAXOBS);
  if (obs.empty()) {
    out->msg = "no observation data";
    return false;
  }
  impl_->opt = makeOpt(o);
  const int n = static_cast<int>(obs.size());
  sol_t sol;
  std::memset(&sol, 0, sizeof(sol));
  std::vector<double> azel(2 * static_cast<size_t>(n), 0.0);
  std::vector<ssat_t> ssat(MAXSAT);
  std::memset(ssat.data(), 0, sizeof(ssat_t) * ssat.size());
  std::vector<char> msg(MAXERRMSG, '\0');
  const int stat = ::pntpos(obs.data(), n, &im->nav, &impl_->opt, &sol, azel.data(), ssat.data(), msg.data());
  out->ok = stat != 0;
  out->msg = msg.data();
  for (int i = 0; i < 3; ++i) {
    out->r_ecef[i] = sol.rr[i];
    out->v_ecef[i] = sol.rr[i + 3];
  }
  for (int i = 0; i < 6; ++i) out->dtr_s[i] = sol.dtr[i];
  out->ns = sol.ns;
  std::vector<double> azels;
  for (int i = 0; i < n; ++i) {
    const ssat_t& s = ssat[obs[i].sat - 1];
    if (!s.vs) continue;
    const SatId id = satIdFromNo(obs[i].sat);
    out->resp.emplace_back(id, s.resp[0]);
    out->azel.emplace_back(id, Eigen::Vector2d(azel[2 * i], azel[2 * i + 1]));
    azels.push_back(azel[2 * i]);
    azels.push_back(azel[2 * i + 1]);
  }
  if (!azels.empty()) {
    double dop[4] = {0, 0, 0, 0};
    ::dops(static_cast<int>(azels.size() / 2), azels.data(), impl_->opt.elmin, dop);
    out->gdop = dop[0];
  }
#ifdef RTKX_P2
  rtkx_sol_t ls;
  ::rtkx_lastsol(&ls);
  auto take = [](const rtkx_est_t& e, Eigen::VectorXd* x, Eigen::VectorXd* xl, Eigen::MatrixXd* Q) {
    if (!e.valid) return;
    x->resize(e.nx);
    xl->resize(e.nx);
    Q->resize(e.nx, e.nx);
    for (int i = 0; i < e.nx; ++i) {
      (*x)[i] = e.x[i];
      (*xl)[i] = e.xlin[i];
      for (int j = 0; j < e.nx; ++j) (*Q)(i, j) = e.Q[i + j * e.nx];   // RTKLIB matrices are column-major
    }
  };
  take(ls.pos, &out->x_pos, &out->x_pos_lin, &out->Q_pos);
  take(ls.vel, &out->x_vel, &out->x_vel_lin, &out->Q_vel);
  if (ls.vel.valid) out->ddtr = ls.vel.x[3] / CLIGHT;
#endif
  return out->ok;
}

PrangeRef prangeRef(const SatObs& so, const NavStore& nav, const SppRefOptions& o) {
  PrangeRef r;
#ifdef RTKX_P2
  NavStore::Impl* im = nav.impl();
  im->sync();
  obsd_t d;
  if (!toObsd(so, toG(so.t_rx), &d)) return r;
  const prcopt_t opt = makeOpt(o);
  double var = 0.0;
  const double P = ::rtkx_prange(&d, &im->nav, &opt, &var);
  if (P == 0.0) return r;
  const int sys = ::satsys(d.sat, nullptr);
  r.ok = true;
  r.P_corr = P;
  // varerr() needs the elevation, which a SatObs does not carry: the zenith value is reported (plus prange's own
  // code-bias term). The core never uses it — its σ² is self-computed (§4.7, RESOLVED R2-12).
  r.var = var + ::rtkx_varerr(&opt, &d, PI / 2.0, sys);
  // The group delay prange() applied on the single-frequency branch (pntpos.c:161-189). Two upstream quirks are
  // reproduced, not fixed (this is the reference row): gettgd() reads the satellite's *first* record in nav.eph, and
  // the Galileo branch still follows the 2.4.3 selection convention — any non-zero getseleph(SYS_GAL) picks
  // BGD_E1E5a (tgd[0]), although in RTKLIB-EX 2.5.1 selection 1 means I/NAV (ephemeris.c:456), whose E1 clock needs
  // BGD_E1E5b (tgd[1]). The core's own rows take rtk::codeBiasMetres() of the selected record (review M3).
  double b = 0.0;
  switch (sys) {
    case SYS_GAL: b = ::getseleph(SYS_GAL) ? ::rtkx_gettgd(d.sat, &im->nav, 0) : ::rtkx_gettgd(d.sat, &im->nav, 1); break;
    case SYS_CMP:
      if (d.code[0] == CODE_L2I) b = ::rtkx_gettgd(d.sat, &im->nav, 0);
      else if (d.code[0] == CODE_L1P) b = ::rtkx_gettgd(d.sat, &im->nav, 2);
      else b = ::rtkx_gettgd(d.sat, &im->nav, 2) + ::rtkx_gettgd(d.sat, &im->nav, 4);
      break;
    default: b = ::rtkx_gettgd(d.sat, &im->nav, 0); break;   // GPS/QZS TGD, GLO −dtaun·c, IRN TGD
  }
  r.tgd_s = b / CLIGHT;
#else
  (void)so;
  (void)nav;
  (void)o;
#endif
  return r;
}

// ---------------------------------------------------------------------------------------------
// Group delay and health rule (review M3 / m1 / m2)
// ---------------------------------------------------------------------------------------------
double codeBiasMetres(const SatId& sat, const SatState& st, SignalCode code) {
  constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
  switch (sat.sys) {
    case Sys::GPS:
    case Sys::QZS:
      return code == SignalCode::L1C ? CLIGHT * st.tgd_s[0] : kNaN;   // prange L1: P1 − TGD
    case Sys::BDS:
      return code == SignalCode::L2I ? CLIGHT * st.tgd_s[0] : kNaN;   // prange B1I: P1 − TGD1
    case Sys::GLO:
      if (code != SignalCode::L1C) return kNaN;
      {
        // prange G1: P1 − b1/(γ−1) with b1 = gettgd() = −dtaun·c and γ from the nominal G1/G2 carriers.
        const double gamma = (FREQ1_GLO / FREQ2_GLO) * (FREQ1_GLO / FREQ2_GLO);
        return (-st.tgd_s[0] * CLIGHT) / (gamma - 1.0);
      }
    case Sys::GAL:
      // The BGD must match the clock the record carries (Galileo OS SIS ICD §5.1.5: E1 single-frequency users apply
      // BGD(E1,E5b) with the I/NAV clock, BGD(E1,E5a) with the F/NAV clock) — keyed on the selected record, not on
      // the global selection as RTKLIB-EX prange() does (see the header).
      if (code != SignalCode::L1X) return kNaN;
      if (st.nav_type == 1) return CLIGHT * st.tgd_s[1];
      if (st.nav_type == 2) return CLIGHT * st.tgd_s[0];
      return kNaN;
    default:
      return kNaN;
  }
}

bool svhHealthy(Sys sys, int svh) {
  // rtkcmn.c satexclude() without its option/URA parts, same bit tests in the same order.
  if (svh < 0) return false;
  if (sys == Sys::QZS) svh &= 0xFE;   // QZSS L1C/B–LEX health bit, set on every IGS BRDC QZSS record (svh = 1)
  if (sys == Sys::GLO) return (svh & 9) == 0 && (svh & 6) != 4;
  return svh == 0;
}

}  // namespace rtk

// Exported wrappers (types.hpp) — pure forwards.
int satNoOf(Sys sys, int prn) { return rtk::satNo(sys, prn); }
SatId makeSatIdOf(Sys sys, int prn) { return rtk::makeSatId(sys, prn); }
std::string satIdStringOf(const SatId& sat) { return rtk::satIdString(sat.rtk_sat); }

}  // namespace livo_gnss
