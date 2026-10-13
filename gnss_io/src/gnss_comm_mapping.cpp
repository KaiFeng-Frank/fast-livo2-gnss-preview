// SPDX-License-Identifier: BSD-3-Clause
// gnss_comm_mapping (DESIGN §12.2–§12.3, AMENDMENTS A16/A19/A23). Owner: WP-e. Tests: U2, T1–T3.
//
// Field-name mapping from decoded gnss_comm field trees to livo_gnss types. The expectation tables below are
// the flattened (type, name) layouts read from the dataset's own connection headers (M3DGR, identical in every
// bag audited) — functional facts of the wire format written as our own constants; no gnss_comm text is
// copied. A topic whose layout differs is rejected as a whole with a readable diff (no partial acceptance).
//
// Conventions fixed here (all checked by T1 against the rosbags golden CSV):
//   * numbering: gnss_comm sat → (sys, prn): GPS 1–32, GLO 33–59 (prn = sat−32), GAL 60–97 (sat−59),
//     BDS 98–160 (sat−97); SatId::rtk_sat through the SatNoFn callback (gnss_io does not link RTKLIB here);
//   * signals: freqs[] → code by the §12.3 table; one signal per code, first listed wins (later ones counted in
//     MappingStats::rejected_repeated_code); primary first (signal_order.hpp, shared with the RINEX/UBX paths);
//   * times: every GnssTimeMsg is GPST (week, tow) — also for BDS (§12.3) — converted with RTKLIB's own
//     gpst2time arithmetic so the GTime is bit-identical to what rtk_adapter would produce;
//   * KeplerEph toes/week are filled per system from toe (GPS/QZS/GAL: time2gpst(toe); BDS:
//     time2bdt(gpst2bdt(toe)) — a GPST toes would put a BDS satellite 27 km off, R2-3); the message's own
//     toe_tow/week are redundant and ignored (EphemStore::completeFields recomputes the same, idempotent);
//   * ura is kept raw with sva = −1: its semantics (metres vs index) are judged by the ingest self-check (A16);
//   * receiver std fields are kept raw (units judged later, §12.6-3);
//   * iono_params: 8 values → alpha[0..3], beta[0..3] (RTKLIB ion_gps order); header.stamp is host time and is
//     not a GPST validity epoch, so Klobuchar::t_valid stays unset. Always pushed with Source::GnssCommIono and
//     nothing else: on M3DGR these are not the GPS broadcast set (A38; BDS D1 inferred, WP-e #9), the GPS
//     Klobuchar comes from the RINEX header GPSA/GPSB (Source::Rinex) and the core keeps GnssCommIono records for
//     diagnostics only — the source tag is how it tells the two apart;
//   * time_pulse_info is only counted (A19).
#include "livo_gnss_io/gnss_comm_mapping.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <utility>

#include "signal_order.hpp"

namespace livo_gnss_io {

using livo_gnss::GTime;
using livo_gnss::SignalCode;
using livo_gnss::SignalVerdict;
using livo_gnss::Source;
using livo_gnss::Sys;

namespace {

// ---- RTKLIB-identical GPST/BDT arithmetic (rtkcmn.c gpst2time, time2gpst, time2bdt, timeadd, gpst2bdt) ----
constexpr int64_t kGpst0 = 315964800;    // epoch2time({1980,1,6,0,0,0})
constexpr int64_t kBdt0 = 1136073600;    // epoch2time({2006,1,1,0,0,0})

GTime gpst2time(int week, double sec) {
  GTime t;
  t.sec = kGpst0;
  if (sec < -1e9 || 1e9 < sec) sec = 0.0;
  t.sec += static_cast<int64_t>(86400) * 7 * week + static_cast<int>(sec);
  t.frac = sec - static_cast<int>(sec);
  return t;
}

double secondsOfWeek(const GTime& t, int64_t t0, int* week) {
  const int64_t sec = t.sec - t0;
  const int w = static_cast<int>(sec / (86400 * 7));
  if (week) *week = w;
  return static_cast<double>(static_cast<double>(sec) - static_cast<double>(w) * 86400 * 7) + t.frac;
}

GTime timeadd(GTime t, double s) {
  t.frac += s;
  const double tt = std::floor(t.frac);
  t.sec += static_cast<int>(tt);
  t.frac -= tt;
  return t;
}

// ---- expectation tables ------------------------------------------------------------------------------
struct Expectation {
  std::string datatype;
  Layout layout;
};

void addTime(Layout* L, const std::string& n) {
  L->emplace_back("gnss_comm/GnssTimeMsg", n);
  L->emplace_back("uint32", n + ".week");
  L->emplace_back("float64", n + ".tow");
}

Expectation makeMeas() {
  Expectation e{"gnss_comm/GnssMeasMsg", {}};
  Layout& L = e.layout;
  L.emplace_back("gnss_comm/GnssObsMsg[]", "meas");
  addTime(&L, "meas.time");
  L.emplace_back("uint32", "meas.sat");
  for (const char* n : {"freqs", "CN0"}) L.emplace_back("float64[]", std::string("meas.") + n);
  for (const char* n : {"LLI", "code"}) L.emplace_back("uint8[]", std::string("meas.") + n);
  for (const char* n : {"psr", "psr_std", "cp", "cp_std", "dopp", "dopp_std"}) L.emplace_back("float64[]", std::string("meas.") + n);
  L.emplace_back("uint8[]", "meas.status");
  return e;
}

Expectation makeEphem() {
  Expectation e{"gnss_comm/GnssEphemMsg", {}};
  Layout& L = e.layout;
  L.emplace_back("uint32", "sat");
  for (const char* t : {"ttr", "toe", "toc"}) addTime(&L, t);
  L.emplace_back("float64", "toe_tow");
  for (const char* n : {"week", "iode", "iodc", "health", "code"}) L.emplace_back("uint32", n);
  for (const char* n : {"ura", "A", "e", "i0", "omg", "OMG0", "M0", "delta_n", "OMG_dot", "i_dot", "cuc", "cus", "crc", "crs", "cic",
                        "cis", "af0", "af1", "af2", "tgd0", "tgd1", "A_dot", "n_dot"})
    L.emplace_back("float64", n);
  return e;
}

Expectation makeGloEphem() {
  Expectation e{"gnss_comm/GnssGloEphemMsg", {}};
  Layout& L = e.layout;
  L.emplace_back("uint32", "sat");
  for (const char* t : {"ttr", "toe"}) addTime(&L, t);
  L.emplace_back("int32", "freqo");
  for (const char* n : {"iode", "health", "age"}) L.emplace_back("uint32", n);
  for (const char* n : {"ura", "pos_x", "pos_y", "pos_z", "vel_x", "vel_y", "vel_z", "acc_x", "acc_y", "acc_z", "tau_n", "gamma",
                        "delta_tau_n"})
    L.emplace_back("float64", n);
  return e;
}

Expectation makeIono() {
  Expectation e{"gnss_comm/StampedFloat64Array", {}};
  Layout& L = e.layout;
  L.emplace_back("std_msgs/Header", "header");
  L.emplace_back("uint32", "header.seq");
  L.emplace_back("time", "header.stamp");
  L.emplace_back("string", "header.frame_id");
  L.emplace_back("float64[]", "data");
  return e;
}

Expectation makePvt() {
  Expectation e{"gnss_comm/GnssPVTSolnMsg", {}};
  Layout& L = e.layout;
  addTime(&L, "time");
  L.emplace_back("uint8", "fix_type");
  L.emplace_back("bool", "valid_fix");
  L.emplace_back("bool", "diff_soln");
  L.emplace_back("uint8", "carr_soln");
  L.emplace_back("uint8", "num_sv");
  for (const char* n : {"latitude", "longitude", "altitude", "height_msl", "h_acc", "v_acc", "p_dop", "vel_n", "vel_e", "vel_d", "vel_acc"})
    L.emplace_back("float64", n);
  return e;
}

const Expectation* expectationFor(Source src) {
  static const Expectation meas = makeMeas(), ephem = makeEphem(), glo = makeGloEphem(), iono = makeIono(), pvt = makePvt();
  switch (src) {
    case Source::GnssCommMeas: return &meas;
    case Source::GnssCommEphem: return &ephem;
    case Source::GnssCommGloEphem: return &glo;
    case Source::GnssCommIono: return &iono;
    case Source::GnssCommPvt: return &pvt;
    default: return nullptr;
  }
}

// Local names: gnss_io uses only the livo_gnss *headers*, never the core library's symbols (DEVELOPING §1),
// so srcName() (types.cpp) is not called from here.
const char* srcName(Source s) {
  switch (s) {
    case Source::GnssCommMeas: return "range_meas";
    case Source::GnssCommEphem: return "ephem";
    case Source::GnssCommGloEphem: return "glo_ephem";
    case Source::GnssCommIono: return "iono_params";
    case Source::GnssCommPvt: return "receiver_pvt";
    case Source::GnssCommTimePulse: return "time_pulse_info";
    case Source::Rinex: return "rinex";
    case Source::UbxStream: return "ubx_stream";
    case Source::Synthetic: return "synthetic";
    default: return "?";
  }
}

bool isGnssCommSource(Source s) {
  return s == Source::GnssCommMeas || s == Source::GnssCommEphem || s == Source::GnssCommGloEphem || s == Source::GnssCommIono ||
         s == Source::GnssCommPvt || s == Source::GnssCommTimePulse;
}

std::string normalizeDatatype(const std::string& t) {   // "pkg/msg/Type" → "pkg/Type"
  const size_t a = t.find('/');
  if (a == std::string::npos) return t;
  const size_t b = t.find('/', a + 1);
  if (b == std::string::npos || t.compare(a + 1, b - a - 1, "msg") != 0) return t;
  return t.substr(0, a) + t.substr(b);
}

// ---- field access (a tree built elsewhere — e.g. from the golden CSV — is checked the same way) ----------
struct Access {
  std::string* why;
  bool fail(const std::string& m) {
    if (why) *why = m;
    return false;
  }
  const FieldNode* scalar(const FieldNode& msg, const char* name) {
    const FieldNode* n = msg.child(name);
    if (!n || n->is_array || n->is_message) {
      fail(std::string("missing scalar field '") + name + "'");
      return nullptr;
    }
    return n;
  }
  bool num(const FieldNode& msg, const char* name, double* out) {
    const FieldNode* n = scalar(msg, name);
    if (!n) return false;
    const double v = n->scalar.toDouble();
    if (n->scalar.type == livo_gnss_io::BuiltinType::String || n->scalar.type == livo_gnss_io::BuiltinType::NotBuiltin)
      return fail(std::string("field '") + name + "' is not numeric");
    *out = v;
    return true;
  }
  bool integer(const FieldNode& msg, const char* name, int64_t* out) {
    const FieldNode* n = scalar(msg, name);
    if (!n) return false;
    switch (n->scalar.type) {
      case BuiltinType::Boolean: *out = n->scalar.b ? 1 : 0; return true;
      case BuiltinType::Int8:
      case BuiltinType::Int16:
      case BuiltinType::Int32:
      case BuiltinType::Int64: *out = n->scalar.i; return true;
      case BuiltinType::UInt8:
      case BuiltinType::UInt16:
      case BuiltinType::UInt32:
      case BuiltinType::UInt64:
        if (n->scalar.u > static_cast<uint64_t>(std::numeric_limits<int64_t>::max())) return fail(std::string("field '") + name + "' overflows");
        *out = static_cast<int64_t>(n->scalar.u);
        return true;
      default: return fail(std::string("field '") + name + "' is not an integer");
    }
  }
  bool boolean(const FieldNode& msg, const char* name, bool* out) {
    int64_t v = 0;
    if (!integer(msg, name, &v)) return false;
    *out = v != 0;
    return true;
  }
  const std::vector<Value>* array(const FieldNode& msg, const char* name) {
    const FieldNode* n = msg.child(name);
    if (!n || !n->is_array || n->is_message) {
      fail(std::string("missing array field '") + name + "'");
      return nullptr;
    }
    return &n->array;
  }
  // gnss_comm/GnssTimeMsg (week, tow) — GPST for every system (§12.3).
  bool gpst(const FieldNode& msg, const char* name, GTime* out) {
    const FieldNode* n = msg.child(name);
    if (!n || !n->is_message || n->is_array) return fail(std::string("missing time field '") + name + "'");
    int64_t week = 0;
    double tow = 0.0;
    if (!integer(*n, "week", &week) || !num(*n, "tow", &tow)) return false;
    *out = gpst2time(static_cast<int>(week), tow);
    return true;
  }
};

double arrNum(const std::vector<Value>& a, size_t k) { return a[k].toDouble(); }
uint8_t arrU8(const std::vector<Value>& a, size_t k) {
  const Value& v = a[k];
  switch (v.type) {
    case BuiltinType::Int8:
    case BuiltinType::Int16:
    case BuiltinType::Int32:
    case BuiltinType::Int64: return static_cast<uint8_t>(v.i);
    case BuiltinType::Boolean: return v.b ? 1 : 0;
    default: return static_cast<uint8_t>(v.u);
  }
}

// Satellite resolution shared by observations and ephemerides; false = unusable (counted).
bool resolveSat(uint64_t gc_sat, SatNoFn satno, livo_gnss::SatId* id, MappingStats* stats) {
  int prn = 0;
  const Sys sys = gc_sat > 0xffffffffull ? Sys::Unknown : gnssCommSatToSys(static_cast<uint32_t>(gc_sat), &prn);
  if (sys == Sys::Unknown) {
    if (stats) ++stats->rejected_sat_range;
    return false;
  }
  const int rtk = satno ? satno(sys, prn) : 0;
  if (rtk <= 0) {
    // E37/E38 exist in gnss_comm numbering but not in RTKLIB (MAXPRNGAL 36): DESIGN §10 L0 "丢弃计数".
    if (stats) {
      if (sys == Sys::GAL && prn > 36) ++stats->rejected_gal_prn;
      else ++stats->rejected_sat_range;
    }
    return false;
  }
  id->sys = sys;
  id->prn = prn;
  id->rtk_sat = rtk;
  return true;
}

}  // namespace

// ---- expectation tables --------------------------------------------------------------------------------
const Layout& expectedLayout(Source src) {
  static const Layout empty;
  const Expectation* e = expectationFor(src);
  return e ? e->layout : empty;
}

bool validateLayout(Source src, const MsgSchema& schema, std::string* diff) {
  if (diff) diff->clear();
  if (src == Source::GnssCommTimePulse) return true;   // diagnostics only, never rejected (A19)
  const Expectation* e = expectationFor(src);
  if (!e) {
    if (diff) *diff = std::string("no expectation table for source ") + srcName(src);
    return false;
  }
  std::string d;
  const std::string got_type = normalizeDatatype(schema.datatype);
  if (got_type != e->datatype) d += "datatype: expected " + e->datatype + ", got " + got_type + "\n";
  const Layout got = schema.flatten();
  const Layout& exp = e->layout;
  const size_t n = std::max(got.size(), exp.size());
  int shown = 0;
  for (size_t i = 0; i < n; ++i) {
    const bool has_e = i < exp.size(), has_g = i < got.size();
    if (has_e && has_g && exp[i] == got[i]) continue;
    if (shown++ < 8) {
      d += "#" + std::to_string(i) + ": expected " + (has_e ? exp[i].first + " " + exp[i].second : std::string("<end>")) + ", got " +
           (has_g ? got[i].first + " " + got[i].second : std::string("<end>")) + "\n";
    }
  }
  if (shown > 8) d += "... " + std::to_string(shown - 8) + " more differing entries\n";
  if (!d.empty()) {
    if (diff)
      *diff = "layout of " + std::string(srcName(src)) + " differs from the expectation table (" + std::to_string(exp.size()) +
              " entries expected, " + std::to_string(got.size()) + " found):\n" + d;
    return false;
  }
  return true;
}

Sys gnssCommSatToSys(uint32_t sat, int* prn) {
  int p = 0;
  Sys s = Sys::Unknown;
  if (sat >= 1 && sat <= 32) {
    s = Sys::GPS;
    p = static_cast<int>(sat);
  } else if (sat >= 33 && sat <= 59) {
    s = Sys::GLO;
    p = static_cast<int>(sat) - 32;
  } else if (sat >= 60 && sat <= 97) {
    s = Sys::GAL;
    p = static_cast<int>(sat) - 59;
  } else if (sat >= 98 && sat <= 160) {
    s = Sys::BDS;
    p = static_cast<int>(sat) - 97;
  }
  if (prn) *prn = p;
  return s;
}

// ---- field tree → core types -----------------------------------------------------------------------------
bool toObsEpoch(const FieldNode& msg, double t_record, SatNoFn satno, livo_gnss::ObsEpoch* out, MappingStats* stats, std::string* why) {
  Access a{why};
  const FieldNode* meas = msg.child("meas");
  if (!meas || !meas->is_array || !meas->is_message) return a.fail("missing message array 'meas'");
  livo_gnss::ObsEpoch ep;
  ep.t_record = t_record;
  ep.source = Source::GnssCommMeas;
  bool have_time = false;
  for (const FieldNode& m : meas->children) {
    int64_t gc_sat = 0;
    GTime t;
    if (!a.integer(m, "sat", &gc_sat) || !a.gpst(m, "time", &t)) return false;
    if (!have_time) {   // the epoch tag is the first satellite's; gnss_comm stamps all with one rcvTow
      ep.t_rx = t;
      have_time = true;
    }
    static const char* kArrays[] = {"freqs", "CN0", "LLI", "code", "psr", "psr_std", "cp", "cp_std", "dopp", "dopp_std", "status"};
    const std::vector<Value>* arr[11];
    for (int k = 0; k < 11; ++k)
      if (!(arr[k] = a.array(m, kArrays[k]))) return false;
    const size_t nf = arr[0]->size();
    for (int k = 1; k < 11; ++k)
      if (arr[k]->size() != nf)
        return a.fail("sat " + std::to_string(gc_sat) + ": array '" + kArrays[k] + "' has " + std::to_string(arr[k]->size()) +
                      " entries, freqs has " + std::to_string(nf));
    livo_gnss::SatObs so;
    if (gc_sat < 0 || !resolveSat(static_cast<uint64_t>(gc_sat), satno, &so.sat, stats)) continue;
    so.t_rx = t;
    for (size_t k = 0; k < nf; ++k) {
      livo_gnss::SignalObs s;
      s.freq_hz = arrNum(*arr[0], k);
      SignalVerdict verdict = SignalVerdict::Accepted;
      s.code = livo_gnss::signalFromFrequency(so.sat.sys, s.freq_hz, nullptr, &verdict);
      if (verdict != SignalVerdict::Accepted) {
        if (stats) {
          if (verdict == SignalVerdict::RejectedBdsB1C) ++stats->rejected_bds_b1c;
          else ++stats->rejected_unknown_freq;   // unknown frequency or GLONASS channel off the FDMA grid
        }
        continue;
      }
      // One signal per code, first listed wins (signal_order.hpp rule 1; same choice as the RINEX and UBX
      // paths); the dropped ones are counted (MappingStats::rejected_repeated_code, WP-e #7, A39).
      if (detail::hasCode(so, s.code)) {
        if (stats) ++stats->rejected_repeated_code;
        continue;
      }
      s.cn0_dbhz = arrNum(*arr[1], k);
      s.lli = arrU8(*arr[2], k);
      s.psr_m = arrNum(*arr[4], k);
      s.psr_std_raw = arrNum(*arr[5], k);
      s.cp_cyc = arrNum(*arr[6], k);
      s.cp_std_raw = arrNum(*arr[7], k);
      s.dopp_hz = arrNum(*arr[8], k);
      s.dopp_std_raw = arrNum(*arr[9], k);
      s.status = arrU8(*arr[10], k);
      so.sig.push_back(s);
    }
    if (so.sig.empty()) continue;   // no usable signal: nothing to carry (the golden CSV keeps the raw record)
    detail::primaryFirst(&so);
    if (stats) {
      ++stats->sats;
      stats->signals += so.sig.size();
    }
    ep.sats.push_back(std::move(so));
  }
  if (out) *out = std::move(ep);
  return true;
}

bool toKeplerEph(const FieldNode& msg, double t_record, SatNoFn satno, livo_gnss::KeplerEph* out, MappingStats* stats, std::string* why) {
  Access a{why};
  livo_gnss::KeplerEph e;
  int64_t gc_sat = 0, iode = 0, iodc = 0, health = 0, code = 0;
  if (!a.integer(msg, "sat", &gc_sat) || !a.gpst(msg, "ttr", &e.ttr) || !a.gpst(msg, "toe", &e.toe) || !a.gpst(msg, "toc", &e.toc) ||
      !a.integer(msg, "iode", &iode) || !a.integer(msg, "iodc", &iodc) || !a.integer(msg, "health", &health) ||
      !a.integer(msg, "code", &code))
    return false;
  struct F {
    const char* name;
    double* dst;
  };
  const F fields[] = {{"ura", &e.ura_raw}, {"A", &e.A},       {"e", &e.e},         {"i0", &e.i0},       {"omg", &e.omg},
                      {"OMG0", &e.OMG0},   {"M0", &e.M0},     {"delta_n", &e.deln}, {"OMG_dot", &e.OMGd}, {"i_dot", &e.idot},
                      {"cuc", &e.cuc},     {"cus", &e.cus},   {"crc", &e.crc},     {"crs", &e.crs},     {"cic", &e.cic},
                      {"cis", &e.cis},     {"af0", &e.f0},    {"af1", &e.f1},      {"af2", &e.f2},      {"tgd0", &e.tgd[0]},
                      {"tgd1", &e.tgd[1]}, {"A_dot", &e.Adot}, {"n_dot", &e.ndot}};
  for (const F& f : fields)
    if (!a.num(msg, f.name, f.dst)) return false;
  e.iode = static_cast<int>(iode);
  e.iodc = static_cast<int>(iodc);
  e.svh = static_cast<int>(health);
  e.code = static_cast<int>(code);
  e.sva = -1;   // derived from ura_raw by EphemStore once the URA semantics are known (A16)
  e.t_record = t_record;
  e.source = Source::GnssCommEphem;
  bool usable = gc_sat >= 0 && resolveSat(static_cast<uint64_t>(gc_sat), satno, &e.sat, stats);
  if (usable && !(e.sat.sys == Sys::GPS || e.sat.sys == Sys::GAL || e.sat.sys == Sys::BDS)) {
    // GnssEphemMsg carries GPS/GAL/BDS Keplerian sets only; a GLONASS number here is a producer error.
    if (stats) ++stats->rejected_sat_range;
    e.sat = livo_gnss::SatId();
    usable = false;
  }
  if (usable) {
    if (e.sat.sys == Sys::BDS) {
      e.toes = secondsOfWeek(timeadd(e.toe, -14.0), kBdt0, &e.week);   // time2bdt(gpst2bdt(toe))
    } else {
      e.toes = secondsOfWeek(e.toe, kGpst0, &e.week);                  // time2gpst(toe); GST seconds == GPST
    }
  }
  if (out) *out = e;   // unusable ⇒ sat.rtk_sat == 0; the caller does not push it
  return true;
}

bool toGloEph(const FieldNode& msg, double t_record, SatNoFn satno, livo_gnss::GloEph* out, MappingStats* stats, std::string* why) {
  Access a{why};
  livo_gnss::GloEph g;
  int64_t gc_sat = 0, freqo = 0, iode = 0, health = 0, age = 0;
  if (!a.integer(msg, "sat", &gc_sat) || !a.gpst(msg, "ttr", &g.tof) || !a.gpst(msg, "toe", &g.toe) || !a.integer(msg, "freqo", &freqo) ||
      !a.integer(msg, "iode", &iode) || !a.integer(msg, "health", &health) || !a.integer(msg, "age", &age) ||
      !a.num(msg, "ura", &g.ura_raw) || !a.num(msg, "pos_x", &g.pos.x()) || !a.num(msg, "pos_y", &g.pos.y()) ||
      !a.num(msg, "pos_z", &g.pos.z()) || !a.num(msg, "vel_x", &g.vel.x()) || !a.num(msg, "vel_y", &g.vel.y()) ||
      !a.num(msg, "vel_z", &g.vel.z()) || !a.num(msg, "acc_x", &g.acc.x()) || !a.num(msg, "acc_y", &g.acc.y()) ||
      !a.num(msg, "acc_z", &g.acc.z()) || !a.num(msg, "tau_n", &g.taun) || !a.num(msg, "gamma", &g.gamn) ||
      !a.num(msg, "delta_tau_n", &g.dtaun))
    return false;
  g.frq = static_cast<int>(freqo);
  g.iode = static_cast<int>(iode);
  g.svh = static_cast<int>(health);
  g.age = static_cast<int>(age);
  g.t_record = t_record;
  g.source = Source::GnssCommGloEphem;
  bool usable = gc_sat >= 0 && resolveSat(static_cast<uint64_t>(gc_sat), satno, &g.sat, stats);
  if (usable && g.sat.sys != Sys::GLO) {
    if (stats) ++stats->rejected_sat_range;
    g.sat = livo_gnss::SatId();
  }
  if (out) *out = g;
  return true;
}

bool toKlobuchar(const FieldNode& msg, double t_record, livo_gnss::Klobuchar* out, std::string* why) {
  Access a{why};
  const std::vector<Value>* d = a.array(msg, "data");
  if (!d) return false;
  if (d->size() != 8) return a.fail("iono_params carries " + std::to_string(d->size()) + " values, expected 8 (alpha0..3, beta0..3)");
  livo_gnss::Klobuchar k;
  for (int i = 0; i < 4; ++i) {
    k.alpha[i] = (*d)[i].toDouble();
    k.beta[i] = (*d)[4 + i].toDouble();
  }
  k.t_record = t_record;
  k.source = Source::GnssCommIono;
  if (out) *out = k;
  return true;
}

bool toReceiverPvt(const FieldNode& msg, double t_record, livo_gnss::ReceiverPvt* out, std::string* why) {
  Access a{why};
  livo_gnss::ReceiverPvt p;
  int64_t fix_type = 0, carr = 0, nsv = 0;
  double vn = 0, ve = 0, vd = 0;
  if (!a.gpst(msg, "time", &p.t) || !a.integer(msg, "fix_type", &fix_type) || !a.boolean(msg, "valid_fix", &p.valid_fix) ||
      !a.boolean(msg, "diff_soln", &p.diff_soln) || !a.integer(msg, "carr_soln", &carr) || !a.integer(msg, "num_sv", &nsv) ||
      !a.num(msg, "latitude", &p.lat_deg) || !a.num(msg, "longitude", &p.lon_deg) || !a.num(msg, "altitude", &p.h_ell_m) ||
      !a.num(msg, "height_msl", &p.h_msl_m) || !a.num(msg, "h_acc", &p.h_acc_m) || !a.num(msg, "v_acc", &p.v_acc_m) ||
      !a.num(msg, "p_dop", &p.p_dop) || !a.num(msg, "vel_n", &vn) || !a.num(msg, "vel_e", &ve) || !a.num(msg, "vel_d", &vd) ||
      !a.num(msg, "vel_acc", &p.vel_acc_mps))
    return false;
  p.fix_type = static_cast<int>(fix_type);
  p.carr_soln = static_cast<int>(carr);
  p.num_sv = static_cast<int>(nsv);
  p.vel_ned = Eigen::Vector3d(vn, ve, vd);
  p.t_record = t_record;
  p.source = Source::GnssCommPvt;
  if (out) *out = p;
  return true;
}

// ---- RawDecoder ------------------------------------------------------------------------------------------
GnssCommRos1Decoder::GnssCommRos1Decoder(SatNoFn satno) : satno_(satno) {}
GnssCommRos1Decoder::~GnssCommRos1Decoder() = default;

bool GnssCommRos1Decoder::registerMessageDefinition(Source src, const std::string& datatype, const std::string& md5,
                                                    const std::string& msg_def, std::string* why) {
  (void)md5;   // diagnostics only: the layout comparison is the acceptance criterion (DESIGN §12.2)
  if (why) why->clear();
  if (!isGnssCommSource(src)) return true;   // RawDecoder contract: non-ROS sources ignore this call
  MsgSchema schema;
  std::string err;
  const bool parsed = MsgDefParser::parse(datatype, msg_def, &schema, &err);
  if (src == Source::GnssCommTimePulse) {   // counted only (A19); keep a decoder when the text parses
    if (parsed) decoders_[src].reset(new Ros1Decoder(schema));
    return true;
  }
  // A failed (re-)registration removes the old decoder: stale schemas must not decode new bytes.
  if (!parsed) {
    decoders_.erase(src);
    if (why) *why = "message_definition of " + std::string(srcName(src)) + " does not parse: " + err;
    return false;
  }
  if (!validateLayout(src, schema, &err)) {
    decoders_.erase(src);
    if (why) *why = err;
    return false;
  }
  decoders_[src].reset(new Ros1Decoder(schema));
  return true;
}

bool GnssCommRos1Decoder::decode(Source src, const uint8_t* data, size_t size, double t_record, livo_gnss::MeasurementSink& sink,
                                 std::string* why) {
  if (why) why->clear();
  if (src == Source::GnssCommTimePulse) {   // A19: counted, never interpreted
    ++stats_.time_pulse;
    return true;
  }
  const auto it = decoders_.find(src);
  if (it == decoders_.end() || !it->second) {
    ++stats_.decode_errors;
    if (why) *why = std::string("no accepted message definition for ") + srcName(src);
    return false;
  }
  FieldNode tree;
  size_t used = 0;
  std::string err;
  if (!it->second->decode(data, size, &tree, &used, &err)) {
    ++stats_.decode_errors;
    if (why) *why = std::string(srcName(src)) + ": " + err;
    return false;
  }
  bool ok = true;
  switch (src) {
    case Source::GnssCommMeas: {
      livo_gnss::ObsEpoch ep;
      ok = toObsEpoch(tree, t_record, satno_, &ep, &stats_, &err);
      if (ok) {
        ++stats_.epochs;
        ep.seq = static_cast<uint32_t>(stats_.epochs);   // monotone per source
        // An empty meas[] carries no time tag at all: counted, not pushed.
        if (ep.t_rx.valid()) sink.pushObservation(std::move(ep));
      }
      break;
    }
    case Source::GnssCommEphem: {
      livo_gnss::KeplerEph e;
      ok = toKeplerEph(tree, t_record, satno_, &e, &stats_, &err);
      if (ok && e.sat.valid()) {
        ++stats_.ephem;
        sink.pushEphemeris(e);
      }
      break;
    }
    case Source::GnssCommGloEphem: {
      livo_gnss::GloEph g;
      ok = toGloEph(tree, t_record, satno_, &g, &stats_, &err);
      if (ok && g.sat.valid()) {
        ++stats_.glo_ephem;
        sink.pushGloEphemeris(g);
      }
      break;
    }
    case Source::GnssCommIono: {
      livo_gnss::Klobuchar k;
      ok = toKlobuchar(tree, t_record, &k, &err);
      if (ok) {
        ++stats_.iono;
        sink.pushIono(k);
      }
      break;
    }
    case Source::GnssCommPvt: {
      livo_gnss::ReceiverPvt p;
      ok = toReceiverPvt(tree, t_record, &p, &err);
      if (ok) {
        ++stats_.pvt;
        sink.pushPvt(p);
      }
      break;
    }
    default: ok = false; err = "unsupported source";
  }
  if (!ok) {
    ++stats_.decode_errors;
    if (why) *why = std::string(srcName(src)) + ": " + err;
  }
  return ok;
}

bool GnssCommRos1Decoder::registered(Source src) const { return decoders_.count(src) > 0; }

}  // namespace livo_gnss_io
