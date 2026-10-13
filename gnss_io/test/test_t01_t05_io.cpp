// SPDX-License-Identifier: BSD-3-Clause
// test_t01_t05_io.cpp — gnss_io tests T1–T5 as defined in AMENDMENTS A20 (ROS-free). The MessageInstance /
// ShapeShifter convergence check needs ROS and is I14.
//   T1 field trees → ObsEpoch/KeplerEph/GloEph/Klobuchar/ReceiverPvt per DESIGN §12.3, against the golden CSV
//   T2 MsgDefParser::md5 == connection-header md5sum; changed layouts rejected
//   T3 C++ decoder CSV == tools/gnss_bag_export.py CSV, byte for byte
//   T4 RinexSource (readrnxt) OBS/NAV: same ObsEpoch layout as the bag path; NAV/iono read back
//   T5 UbxStream (input_ubx): synthetic RXM-RAWX / RXM-SFRBX / NAV-PVT frames; checksum errors counted
// Fixtures (gnss/test/data/README.md), found by fixture_discovery.hpp:
//   * Synthetic_1419919200_10s — hand-made boundary content under the DESIGN §12.3 assumptions: proves that the
//     code and the exporter agree (PASS here is "PASS (synthetic)"), nothing about the data;
//   * the development-set fixture(s) (Grass02_*) — real receiver output: the *DevSet* cases, including the
//     discriminators of the gnss_comm time and iono conventions, run on realFixtures() only.
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cfloat>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <map>
#include <random>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

#include "golden_csv.hpp"
#include "livo_gnss/signal_table.hpp"
#include "livo_gnss_io/gnss_comm_mapping.hpp"
#include "livo_gnss_io/rinex_source.hpp"
#include "livo_gnss_io/ubx_stream.hpp"

using namespace livo_gnss;
using namespace livo_gnss_io;
namespace ts = livo_gnss_io::testsupport;

namespace {

const std::string kRoot = LIVO_GNSS_TEST_DATA_DIR;
const std::string kSynth = kRoot + "/Synthetic_1419919200_10s";
#define REQUIRE_SYNTHETIC_WIRE_FIXTURE() \
  do { \
    if (!ts::isDir(kSynth)) \
      GTEST_SKIP() << "optional synthetic wire fixture is not distributed in the research preview: " << kSynth; \
  } while (0)
constexpr int64_t kGpst0 = 315964800;     // 1980-01-06 as Unix seconds
constexpr int64_t kBdt0 = 1136073600;     // 2006-01-01 as Unix seconds

// RTKLIB numbering (rtklib.h NSATGPS 32, NSATGLO 27, NSATGAL 36, NSATQZS 10, BDS after P1: 1..63). The core
// passes livo_gnss::satNoOf; gnss_io only sees the callback, so the tests use this stand-in.
int testSatNo(Sys sys, int prn) {
  switch (sys) {
    case Sys::GPS: return (prn >= 1 && prn <= 32) ? prn : 0;
    case Sys::GLO: return (prn >= 1 && prn <= 27) ? 32 + prn : 0;
    case Sys::GAL: return (prn >= 1 && prn <= 36) ? 59 + prn : 0;
    case Sys::QZS: return (prn >= 193 && prn <= 202) ? 95 + prn - 192 : 0;
    case Sys::BDS: return (prn >= 1 && prn <= 63) ? 105 + prn : 0;
    default: return 0;
  }
}

struct CaptureSink : MeasurementSink {
  std::vector<ObsEpoch> obs;
  std::vector<KeplerEph> eph;
  std::vector<GloEph> geph;
  std::vector<Klobuchar> ion;
  std::vector<ReceiverPvt> pvt;
  void pushObservation(ObsEpoch&& e) override { obs.push_back(std::move(e)); }
  void pushEphemeris(const KeplerEph& e) override { eph.push_back(e); }
  void pushGloEphemeris(const GloEph& e) override { geph.push_back(e); }
  void pushIono(const Klobuchar& k) override { ion.push_back(k); }
  void pushPvt(const ReceiverPvt& p) override { pvt.push_back(p); }
};

Source sourceOfTopic(const std::string& t) {
  if (t == "range_meas") return Source::GnssCommMeas;
  if (t == "ephem") return Source::GnssCommEphem;
  if (t == "glo_ephem") return Source::GnssCommGloEphem;
  if (t == "iono_params") return Source::GnssCommIono;
  if (t == "receiver_pvt") return Source::GnssCommPvt;
  return Source::GnssCommTimePulse;
}

double recordTime(uint64_t t_ns) { return static_cast<double>(t_ns / 1000000000ull) + 1e-9 * static_cast<double>(t_ns % 1000000000ull); }

// Decodes every record of `topic` through GnssCommRos1Decoder into `sink`.
bool decodeTopic(const std::string& fx, const std::string& topic, GnssCommRos1Decoder* dec, CaptureSink* sink, std::vector<ts::Record>* recs,
                 std::vector<bool>* ok, std::string* why) {
  ts::TopicSchema sch;
  if (!ts::loadTopic(fx, topic, &sch, why)) return false;
  if (!dec->registerMessageDefinition(sourceOfTopic(topic), sch.datatype, sch.md5, sch.msgdef, why)) return false;
  if (!ts::readRecords(fx + "/bytes/" + topic + ".bin", recs, why)) return false;
  ok->clear();
  for (const ts::Record& r : *recs) ok->push_back(dec->decode(sourceOfTopic(topic), r.data.data(), r.data.size(), recordTime(r.t_ns), *sink, why));
  return true;
}

bool loadCsv(const std::string& fx, const std::string& name, ts::Csv* csv) {
  std::string text, why;
  return ts::readFile(fx + "/golden/" + name + ".csv", &text) && ts::parseCsv(text, csv, &why);
}

// Typed cell access by column name; empty cell = absent.
struct Row {
  const ts::Csv& csv;
  size_t r;
  const std::string* cell(const std::string& c) const {
    const auto it = csv.col.find(c);
    return it == csv.col.end() ? nullptr : &csv.rows[r][it->second];
  }
  bool has(const std::string& c) const {
    const std::string* s = cell(c);
    return s && !s->empty();
  }
  double d(const std::string& c) const { return std::strtod(cell(c)->c_str(), nullptr); }
  long long i(const std::string& c) const { return std::strtoll(cell(c)->c_str(), nullptr, 10); }
};

// GPST (week, tow) → GTime, written independently of the mapping (plain integer seconds + fraction).
GTime gpst(long long week, double tow) {
  GTime t;
  const double whole = std::trunc(tow);
  t.sec = kGpst0 + week * 604800 + static_cast<int64_t>(whole);
  t.frac = tow - whole;
  return t;
}

bool sameTime(const GTime& a, const GTime& b) { return a.sec == b.sec && a.frac == b.frac; }

// §12.3 numbering, spelled out.
Sys expectSys(long long sat, int* prn) {
  if (sat >= 1 && sat <= 32) return *prn = static_cast<int>(sat), Sys::GPS;
  if (sat >= 33 && sat <= 59) return *prn = static_cast<int>(sat - 32), Sys::GLO;
  if (sat >= 60 && sat <= 97) return *prn = static_cast<int>(sat - 59), Sys::GAL;
  if (sat >= 98 && sat <= 160) return *prn = static_cast<int>(sat - 97), Sys::BDS;
  *prn = 0;
  return Sys::Unknown;
}

// §12.3 freqs → code, spelled out (0 = rejected; *b1c marks BDS 1575.42).
SignalCode expectCode(Sys sys, double f, bool* b1c) {
  *b1c = false;
  auto near = [&](double f0) { return std::fabs(f - f0) < 1e3; };
  switch (sys) {
    case Sys::GPS: return near(1575.42e6) ? SignalCode::L1C : near(1227.60e6) ? SignalCode::L2X : near(1176.45e6) ? SignalCode::L5X : SignalCode::Unknown;
    case Sys::GAL: return near(1575.42e6) ? SignalCode::L1X : near(1207.14e6) ? SignalCode::L7X : near(1176.45e6) ? SignalCode::L5Q : SignalCode::Unknown;
    case Sys::BDS:
      if (near(1575.42e6)) *b1c = true;
      return near(1561.098e6) ? SignalCode::L2I : near(1207.14e6) ? SignalCode::L7I : SignalCode::Unknown;
    case Sys::GLO:
      for (int k = -7; k <= 6; ++k) {
        if (near(1602e6 + k * 0.5625e6)) return SignalCode::L1C;
        if (near(1246e6 + k * 0.4375e6)) return SignalCode::L2C;
      }
      return SignalCode::Unknown;
    default: return SignalCode::Unknown;
  }
}

bool isPrimary(Sys sys, SignalCode c) {
  return (sys == Sys::GPS || sys == Sys::GLO) ? c == SignalCode::L1C : sys == Sys::GAL ? c == SignalCode::L1X : sys == Sys::BDS ? c == SignalCode::L2I : false;
}

// ---- civil calendar (Howard Hinnant's days_from_civil) → seconds on the calendar's own time scale ----
int64_t daysFromCivil(int64_t y, unsigned m, unsigned d) {
  y -= m <= 2;
  const int64_t era = (y >= 0 ? y : y - 399) / 400;
  const unsigned yoe = static_cast<unsigned>(y - era * 400);
  const unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return era * 146097 + static_cast<int64_t>(doe) - 719468;
}
GTime calendarTime(int y, int mo, int d, int hh, int mi, double ss) {
  GTime t;
  t.sec = daysFromCivil(y, static_cast<unsigned>(mo), static_cast<unsigned>(d)) * 86400 + hh * 3600 + mi * 60 + static_cast<int64_t>(std::floor(ss));
  t.frac = ss - std::floor(ss);
  return t;
}

// ---- tiny RINEX-3 NAV text reader used as the independent oracle for T4 ----
double rnum(const std::string& s, size_t pos, size_t len) {
  if (pos >= s.size()) return 0.0;
  std::string f = s.substr(pos, len);
  for (char& c : f)
    if (c == 'D' || c == 'd') c = 'E';
  return std::strtod(f.c_str(), nullptr);
}
struct NavRec {
  char sys = 0;
  int prn = 0;
  int ep[5] = {0, 0, 0, 0, 0};
  double sec = 0;
  std::vector<double> data;
};
std::vector<NavRec> readNavText(const std::string& path, double ion[8], bool* has_ion) {
  std::vector<NavRec> out;
  std::string text;
  if (!ts::readFile(path, &text)) return out;
  std::vector<std::string> lines;
  std::stringstream ss(text);
  for (std::string l; std::getline(ss, l);) lines.push_back(l);
  size_t i = 0;
  *has_ion = false;
  for (; i < lines.size(); ++i) {
    const std::string& l = lines[i];
    if (l.compare(0, 4, "GPSA") == 0)
      for (int k = 0; k < 4; ++k) ion[k] = rnum(l, 5 + 12 * static_cast<size_t>(k), 12);
    if (l.compare(0, 4, "GPSB") == 0) {
      for (int k = 0; k < 4; ++k) ion[4 + k] = rnum(l, 5 + 12 * static_cast<size_t>(k), 12);
      *has_ion = true;
    }
    if (l.size() > 60 && l.find("END OF HEADER", 60) != std::string::npos) {
      ++i;
      break;
    }
  }
  while (i < lines.size()) {
    const std::string& l = lines[i];
    if (l.empty() || !std::isalpha(static_cast<unsigned char>(l[0]))) {
      ++i;
      continue;
    }
    NavRec r;
    r.sys = l[0];
    r.prn = std::atoi(l.substr(1, 2).c_str());
    r.ep[0] = std::atoi(l.substr(4, 4).c_str());
    for (int k = 1; k < 5; ++k) r.ep[k] = std::atoi(l.substr(9 + 3 * static_cast<size_t>(k - 1), 2).c_str());
    r.sec = std::atof(l.substr(21, 2).c_str());
    for (int k = 0; k < 3; ++k) r.data.push_back(rnum(l, 23 + 19 * static_cast<size_t>(k), 19));
    const int n = (r.sys == 'R' || r.sys == 'S') ? 3 : 7;
    for (int j = 1; j <= n && i + static_cast<size_t>(j) < lines.size(); ++j)
      for (int k = 0; k < 4; ++k) r.data.push_back(rnum(lines[i + static_cast<size_t>(j)], 4 + 19 * static_cast<size_t>(k), 19));
    out.push_back(r);
    i += static_cast<size_t>(n) + 1;
  }
  return out;
}

int uraIndex(double m) {   // IS-GPS-200 URA table (RTKLIB rinex.c uraindex)
  static const double ura[15] = {2.4, 3.4, 4.85, 6.85, 9.65, 13.65, 24.0, 48.0, 96.0, 192.0, 384.0, 768.0, 1536.0, 3072.0, 6144.0};
  int i = 0;
  while (i < 15 && ura[i] < m) ++i;
  return i;
}
int sisaIndex(double v) {   // Galileo SISA (RTKLIB rinex.c sisa_index)
  if (v < 0.0 || v > 6.0) return 255;
  if (v <= 0.49) return static_cast<int>(std::round(v / 0.01));
  if (v <= 0.98) return static_cast<int>(std::round((v - 0.5) / 0.02)) + 50;
  if (v <= 1.96) return static_cast<int>(std::round((v - 1.0) / 0.04)) + 75;
  return static_cast<int>(std::round((v - 2.0) / 0.16)) + 100;
}

void expectRel(double a, double b, double rel, const std::string& what) {
  EXPECT_LE(std::fabs(a - b), rel * std::max(std::fabs(a), std::fabs(b)) + 1e-300) << what << ": " << a << " vs " << b;
}

// Bag-path epochs (non-empty) of a fixture.
std::vector<ObsEpoch> bagEpochs(const std::string& fx, MappingStats* stats) {
  GnssCommRos1Decoder dec(testSatNo);
  CaptureSink sink;
  std::vector<ts::Record> recs;
  std::vector<bool> ok;
  std::string why;
  EXPECT_TRUE(decodeTopic(fx, "range_meas", &dec, &sink, &recs, &ok, &why)) << why;
  if (stats) *stats = dec.stats();
  return sink.obs;
}

// ================================================================================================= T1
struct EdgeTally {
  int empty_epochs = 0, empty_sats = 0, b1c = 0, unknown = 0, gal_prn = 0, sat_range = 0, reordered = 0, glo_kmin = 0, glo_kmax = 0,
      bds_geo = 0, lli = 0, repeated_code = 0;
  std::set<int> status_values;
};

void checkObsMapping(const std::string& fx, EdgeTally* tally) {
  GnssCommRos1Decoder dec(testSatNo);
  CaptureSink sink;
  std::vector<ts::Record> recs;
  std::vector<bool> ok;
  std::string why;
  ASSERT_TRUE(decodeTopic(fx, "range_meas", &dec, &sink, &recs, &ok, &why)) << why;
  ts::Csv csv;
  ASSERT_TRUE(loadCsv(fx, "obs", &csv));
  ASSERT_EQ(csv.rows.size(), recs.size());
  size_t pushed = 0;
  uint64_t n_sats = 0, n_sig = 0;
  EdgeTally t;
  for (size_t r = 0; r < csv.rows.size(); ++r) {
    SCOPED_TRACE("row " + std::to_string(r));
    EXPECT_TRUE(ok[r]);
    const Row row{csv, r};
    int nsat = 0;
    while (row.has("meas_" + std::to_string(nsat) + ".sat")) ++nsat;
    if (nsat == 0) {   // no time tag at all: counted, not pushed
      ++t.empty_epochs;
      continue;
    }
    ASSERT_LT(pushed, sink.obs.size());
    const ObsEpoch& ep = sink.obs[pushed++];
    EXPECT_EQ(ep.source, Source::GnssCommMeas);
    EXPECT_EQ(ep.t_record, recordTime(recs[r].t_ns));
    EXPECT_EQ(ep.seq, static_cast<uint32_t>(r + 1));
    EXPECT_TRUE(sameTime(ep.t_rx, gpst(row.i("meas_0.time.week"), row.d("meas_0.time.tow"))));
    size_t out_k = 0;
    for (int k = 0; k < nsat; ++k) {
      const std::string p = "meas_" + std::to_string(k) + ".";
      int prn = 0;
      const Sys sys = expectSys(row.i(p + "sat"), &prn);
      const int rtk = testSatNo(sys, prn);
      std::vector<SignalObs> exp_sig;
      bool primary_seen_late = false;
      int nf = 0;
      while (row.has(p + "freqs_" + std::to_string(nf))) ++nf;
      if (nf == 0) ++t.empty_sats;
      for (int q = 0; q < nf; ++q) {
        const std::string qq = "_" + std::to_string(q);
        bool b1c = false;
        const double f = row.d(p + "freqs" + qq);
        const SignalCode c = expectCode(sys, f, &b1c);
        if (sys != Sys::Unknown && rtk > 0) {
          if (b1c) ++t.b1c;
          else if (c == SignalCode::Unknown) ++t.unknown;
        }
        if (c == SignalCode::Unknown) continue;
        // one signal per code, the first listed wins (E1-B beside E1-C; gnss_io signal_order.hpp rule 1)
        if (std::any_of(exp_sig.begin(), exp_sig.end(), [&](const SignalObs& x) { return x.code == c; })) {
          if (sys != Sys::Unknown && rtk > 0) ++t.repeated_code;
          continue;
        }
        SignalObs s;
        s.freq_hz = f;
        s.code = c;
        s.cn0_dbhz = row.d(p + "CN0" + qq);
        s.lli = static_cast<uint8_t>(row.i(p + "LLI" + qq));
        s.psr_m = row.d(p + "psr" + qq);
        s.psr_std_raw = row.d(p + "psr_std" + qq);
        s.cp_cyc = row.d(p + "cp" + qq);
        s.cp_std_raw = row.d(p + "cp_std" + qq);
        s.dopp_hz = row.d(p + "dopp" + qq);
        s.dopp_std_raw = row.d(p + "dopp_std" + qq);
        s.status = static_cast<uint8_t>(row.i(p + "status" + qq));
        if (isPrimary(sys, c) && !exp_sig.empty()) primary_seen_late = true;
        exp_sig.push_back(s);
        if (sys == Sys::GLO) {
          if (std::fabs(f - (1602e6 - 7 * 0.5625e6)) < 1.0) ++t.glo_kmin;
          if (std::fabs(f - (1602e6 + 6 * 0.5625e6)) < 1.0) ++t.glo_kmax;
        }
      }
      if (sys == Sys::Unknown) {
        ++t.sat_range;
        continue;
      }
      if (rtk == 0) {
        if (sys == Sys::GAL && prn > 36) ++t.gal_prn;
        else ++t.sat_range;
        continue;
      }
      if (exp_sig.empty()) continue;   // nothing accepted: satellite not carried
      std::stable_partition(exp_sig.begin(), exp_sig.end(), [&](const SignalObs& s) { return isPrimary(sys, s.code); });
      if (primary_seen_late) ++t.reordered;
      ASSERT_LT(out_k, ep.sats.size());
      const SatObs& so = ep.sats[out_k++];
      EXPECT_EQ(so.sat.sys, sys);
      EXPECT_EQ(so.sat.prn, prn);
      EXPECT_EQ(so.sat.rtk_sat, rtk);
      EXPECT_TRUE(sameTime(so.t_rx, gpst(row.i(p + "time.week"), row.d(p + "time.tow"))));
      if (sys == Sys::BDS && (prn <= 5 || prn >= 59)) ++t.bds_geo;
      ASSERT_EQ(so.sig.size(), exp_sig.size()) << sysName(sys) << prn;
      for (size_t q = 0; q < exp_sig.size(); ++q) {
        const SignalObs &a = so.sig[q], &e = exp_sig[q];
        SCOPED_TRACE(std::string(sysName(sys)) + std::to_string(prn) + " sig " + std::to_string(q));
        EXPECT_EQ(a.code, e.code);
        EXPECT_EQ(a.freq_hz, e.freq_hz);
        EXPECT_EQ(a.psr_m, e.psr_m);
        EXPECT_EQ(a.cp_cyc, e.cp_cyc);
        EXPECT_EQ(a.dopp_hz, e.dopp_hz);
        EXPECT_EQ(a.cn0_dbhz, e.cn0_dbhz);
        EXPECT_EQ(a.lli, e.lli);
        EXPECT_EQ(a.status, e.status);
        EXPECT_EQ(a.psr_std_raw, e.psr_std_raw);   // std kept raw; unit judged later (§12.6-3)
        EXPECT_EQ(a.cp_std_raw, e.cp_std_raw);
        EXPECT_EQ(a.dopp_std_raw, e.dopp_std_raw);
        EXPECT_EQ(a.psr_std_unit, StdUnit::Unknown);
        EXPECT_EQ(a.psr_std_m, 0.0);
        t.status_values.insert(a.status);
        if (a.lli & 1) ++t.lli;
      }
      n_sats += 1;
      n_sig += so.sig.size();
    }
    EXPECT_EQ(out_k, ep.sats.size()) << "extra satellites pushed";
  }
  EXPECT_EQ(pushed, sink.obs.size());
  const MappingStats& st = dec.stats();
  EXPECT_EQ(st.epochs, recs.size());
  EXPECT_EQ(st.sats, n_sats);
  EXPECT_EQ(st.signals, n_sig);
  EXPECT_EQ(st.rejected_bds_b1c, static_cast<uint64_t>(t.b1c));
  EXPECT_EQ(st.rejected_unknown_freq, static_cast<uint64_t>(t.unknown));
  EXPECT_EQ(st.rejected_gal_prn, static_cast<uint64_t>(t.gal_prn));
  EXPECT_EQ(st.rejected_sat_range, static_cast<uint64_t>(t.sat_range));
  EXPECT_EQ(st.rejected_repeated_code, static_cast<uint64_t>(t.repeated_code));   // WP-e #7 (A39)
  EXPECT_EQ(st.decode_errors, 0u);
  *tally = t;
}

// The gnss_comm topics of a fixture differ by bag (A26: Occlusion03/04 and Dynamic03 have no ephem, glo_ephem or
// iono_params): per-topic checks run where the topic is present.
bool hasTopic(const std::string& fx, const std::string& topic) { return ts::isFile(fx + "/msgdef/" + topic + ".msgdef"); }

void checkKeplerMapping(const std::string& fx, bool expect_edges) {
  GnssCommRos1Decoder dec(testSatNo);
  CaptureSink sink;
  std::vector<ts::Record> recs;
  std::vector<bool> ok;
  std::string why;
  ASSERT_TRUE(decodeTopic(fx, "ephem", &dec, &sink, &recs, &ok, &why)) << why;
  ts::Csv csv;
  ASSERT_TRUE(loadCsv(fx, "ephem", &csv));
  size_t pushed = 0;
  int gal_rej = 0, bds_checked = 0;
  for (size_t r = 0; r < csv.rows.size(); ++r) {
    SCOPED_TRACE("ephem row " + std::to_string(r));
    const Row row{csv, r};
    int prn = 0;
    const Sys sys = expectSys(row.i("sat"), &prn);
    const int rtk = testSatNo(sys, prn);
    if (rtk == 0 || sys == Sys::GLO || sys == Sys::Unknown) {
      if (sys == Sys::GAL && prn > 36) ++gal_rej;
      continue;
    }
    ASSERT_LT(pushed, sink.eph.size());
    const KeplerEph& e = sink.eph[pushed++];
    EXPECT_EQ(e.sat.sys, sys);
    EXPECT_EQ(e.sat.prn, prn);
    EXPECT_EQ(e.sat.rtk_sat, rtk);
    EXPECT_EQ(e.source, Source::GnssCommEphem);
    EXPECT_EQ(e.t_record, recordTime(recs[r].t_ns));
    const GTime toe = gpst(row.i("toe.week"), row.d("toe.tow"));
    EXPECT_TRUE(sameTime(e.toe, toe));
    EXPECT_TRUE(sameTime(e.toc, gpst(row.i("toc.week"), row.d("toc.tow"))));
    EXPECT_TRUE(sameTime(e.ttr, gpst(row.i("ttr.week"), row.d("ttr.tow"))));
    // toes / week in the satellite system's own scale (§12.3): BDS in BDT (toe − 14 s since 2006-01-01). The
    // message's own toe_tow/week are redundant and ignored by the mapping (in the development data they carry
    // the GPST values of toe, for BDS too), so nothing here depends on them.
    if (sys == Sys::BDS) {
      const int64_t s = toe.sec - 14 - kBdt0;
      EXPECT_EQ(e.week, static_cast<int>(s / 604800));
      EXPECT_NEAR(e.toes, static_cast<double>(s % 604800) + toe.frac, 1e-9);
      EXPECT_NE(e.toes, row.d("toe.tow"));   // the 27 km trap: GPST seconds are not BDT seconds
      ++bds_checked;
    } else {
      const int64_t s = toe.sec - kGpst0;
      EXPECT_EQ(e.week, static_cast<int>(s / 604800));
      EXPECT_NEAR(e.toes, static_cast<double>(s % 604800) + toe.frac, 1e-9);
    }
    EXPECT_EQ(e.iode, row.i("iode"));
    EXPECT_EQ(e.iodc, row.i("iodc"));
    EXPECT_EQ(e.svh, row.i("health"));
    EXPECT_EQ(e.code, row.i("code"));
    EXPECT_EQ(e.sva, -1);                     // URA semantics decided later (A16)
    EXPECT_EQ(e.ura_raw, row.d("ura"));
    const std::vector<std::pair<const char*, double>> f = {
        {"A", e.A},       {"e", e.e},       {"i0", e.i0},     {"omg", e.omg},   {"OMG0", e.OMG0}, {"M0", e.M0},
        {"delta_n", e.deln}, {"OMG_dot", e.OMGd}, {"i_dot", e.idot}, {"cuc", e.cuc}, {"cus", e.cus}, {"crc", e.crc},
        {"crs", e.crs},   {"cic", e.cic},   {"cis", e.cis},   {"af0", e.f0},    {"af1", e.f1},    {"af2", e.f2},
        {"tgd0", e.tgd[0]}, {"tgd1", e.tgd[1]}, {"A_dot", e.Adot}, {"n_dot", e.ndot}};
    for (const auto& kv : f) EXPECT_EQ(kv.second, row.d(kv.first)) << kv.first;
  }
  EXPECT_EQ(pushed, sink.eph.size());
  EXPECT_EQ(dec.stats().ephem, pushed);
  EXPECT_EQ(dec.stats().rejected_gal_prn, static_cast<uint64_t>(gal_rej));
  if (expect_edges) {
    EXPECT_GE(gal_rej, 1);
    EXPECT_GE(bds_checked, 3);
  }
}

void checkGloMapping(const std::string& fx) {
  std::vector<ts::Record> recs;
  std::vector<bool> ok;
  std::string why;
  GnssCommRos1Decoder dg(testSatNo);
  CaptureSink sg;
  ASSERT_TRUE(decodeTopic(fx, "glo_ephem", &dg, &sg, &recs, &ok, &why)) << why;
  ts::Csv cg;
  ASSERT_TRUE(loadCsv(fx, "glo_ephem", &cg));
  ASSERT_EQ(sg.geph.size(), cg.rows.size());
  for (size_t r = 0; r < cg.rows.size(); ++r) {
    const Row row{cg, r};
    const GloEph& g = sg.geph[r];
    int prn = 0;
    EXPECT_EQ(expectSys(row.i("sat"), &prn), Sys::GLO);
    EXPECT_EQ(g.sat.prn, prn);
    EXPECT_EQ(g.sat.rtk_sat, 32 + prn);
    EXPECT_EQ(g.frq, row.i("freqo"));
    EXPECT_EQ(g.iode, row.i("iode"));
    EXPECT_EQ(g.svh, row.i("health"));
    EXPECT_EQ(g.age, row.i("age"));
    EXPECT_EQ(g.ura_raw, row.d("ura"));
    EXPECT_TRUE(sameTime(g.toe, gpst(row.i("toe.week"), row.d("toe.tow"))));
    EXPECT_TRUE(sameTime(g.tof, gpst(row.i("ttr.week"), row.d("ttr.tow"))));
    EXPECT_EQ(g.pos.x(), row.d("pos_x"));
    EXPECT_EQ(g.pos.z(), row.d("pos_z"));
    EXPECT_EQ(g.vel.y(), row.d("vel_y"));
    EXPECT_EQ(g.acc.z(), row.d("acc_z"));
    EXPECT_EQ(g.taun, row.d("tau_n"));        // RTKLIB taun convention (dts = −taun + gamn·t)
    EXPECT_EQ(g.gamn, row.d("gamma"));
    EXPECT_EQ(g.dtaun, row.d("delta_tau_n"));
    EXPECT_EQ(g.source, Source::GnssCommGloEphem);
  }
}

void checkEphMapping(const std::string& fx, bool expect_edges) {
  if (expect_edges) {
    ASSERT_TRUE(hasTopic(fx, "ephem") && hasTopic(fx, "glo_ephem")) << fx;
  }
  if (hasTopic(fx, "ephem")) checkKeplerMapping(fx, expect_edges);
  if (hasTopic(fx, "glo_ephem")) checkGloMapping(fx);
}

void checkIonoPvt(const std::string& fx, bool expect_edges) {
  std::vector<ts::Record> recs;
  std::vector<bool> ok;
  std::string why;
  if (expect_edges) {
    ASSERT_TRUE(hasTopic(fx, "iono_params") && hasTopic(fx, "receiver_pvt")) << fx;
  }
  if (hasTopic(fx, "iono_params")) {
    GnssCommRos1Decoder dec(testSatNo);
    CaptureSink sink;
    ASSERT_TRUE(decodeTopic(fx, "iono_params", &dec, &sink, &recs, &ok, &why)) << why;
    ts::Csv csv;
    ASSERT_TRUE(loadCsv(fx, "iono", &csv));
    size_t pushed = 0, bad = 0;
    for (size_t r = 0; r < csv.rows.size(); ++r) {
      const Row row{csv, r};
      int n = 0;
      while (row.has("data_" + std::to_string(n))) ++n;
      if (n != 8) {
        EXPECT_FALSE(ok[r]) << "iono with " << n << " values must be refused";
        ++bad;
        continue;
      }
      EXPECT_TRUE(ok[r]);
      ASSERT_LT(pushed, sink.ion.size());
      const Klobuchar& k = sink.ion[pushed++];
      for (int i = 0; i < 4; ++i) {
        EXPECT_EQ(k.alpha[i], row.d("data_" + std::to_string(i)));
        EXPECT_EQ(k.beta[i], row.d("data_" + std::to_string(4 + i)));
      }
      EXPECT_FALSE(k.t_valid.valid());   // header.stamp is host time, not a GPST validity epoch
      EXPECT_EQ(k.source, Source::GnssCommIono);
    }
    EXPECT_EQ(pushed, sink.ion.size());
    EXPECT_EQ(dec.stats().decode_errors, bad);
    if (expect_edges) {
      EXPECT_GE(bad, 1u);
    }
  }
  if (hasTopic(fx, "receiver_pvt")) {
    GnssCommRos1Decoder dec(testSatNo);
    CaptureSink sink;
    ASSERT_TRUE(decodeTopic(fx, "receiver_pvt", &dec, &sink, &recs, &ok, &why)) << why;
    ts::Csv csv;
    ASSERT_TRUE(loadCsv(fx, "pvt", &csv));
    ASSERT_EQ(sink.pvt.size(), csv.rows.size());
    int no_fix = 0;
    for (size_t r = 0; r < csv.rows.size(); ++r) {
      const Row row{csv, r};
      const ReceiverPvt& p = sink.pvt[r];
      EXPECT_TRUE(sameTime(p.t, gpst(row.i("time.week"), row.d("time.tow"))));
      EXPECT_EQ(p.fix_type, row.i("fix_type"));
      EXPECT_EQ(p.valid_fix, row.i("valid_fix") != 0);
      EXPECT_EQ(p.diff_soln, row.i("diff_soln") != 0);
      EXPECT_EQ(p.carr_soln, row.i("carr_soln"));
      EXPECT_EQ(p.num_sv, row.i("num_sv"));
      EXPECT_EQ(p.lat_deg, row.d("latitude"));
      EXPECT_EQ(p.lon_deg, row.d("longitude"));
      EXPECT_EQ(p.h_ell_m, row.d("altitude"));   // height above ellipsoid
      EXPECT_EQ(p.h_msl_m, row.d("height_msl"));
      EXPECT_EQ(p.h_acc_m, row.d("h_acc"));
      EXPECT_EQ(p.v_acc_m, row.d("v_acc"));
      EXPECT_EQ(p.p_dop, row.d("p_dop"));
      EXPECT_EQ(p.vel_ned.x(), row.d("vel_n"));
      EXPECT_EQ(p.vel_ned.y(), row.d("vel_e"));
      EXPECT_EQ(p.vel_ned.z(), row.d("vel_d"));
      EXPECT_EQ(p.vel_acc_mps, row.d("vel_acc"));
      EXPECT_EQ(p.t_record, recordTime(recs[r].t_ns));
      no_fix += !p.valid_fix;
    }
    if (expect_edges) {
      EXPECT_GE(no_fix, 1);
    }
  }
}

// ================================================================================================= T4 helpers
struct RinexExpect {
  bool present = false;
  SignalObs s;
};

// What the exporter's RINEX writing rules (tools/gnss_bag_export.py rinex_from_slice) leave of a bag signal,
// as RinexSource must report it (rtklib_conv.hpp status synthesis).
RinexExpect expectAfterRinex(const SignalObs& b) {
  RinexExpect r;
  const bool C = (b.status & 1) && b.psr_m != 0.0;
  const bool L = (b.status & 2) && b.cp_cyc != 0.0;
  const bool D = b.dopp_hz != 0.0, S = b.cn0_dbhz != 0.0;
  r.present = C || L || D || S;
  r.s = b;
  r.s.psr_m = C ? b.psr_m : 0.0;
  r.s.cp_cyc = L ? b.cp_cyc : 0.0;
  r.s.dopp_hz = D ? b.dopp_hz : 0.0;
  r.s.cn0_dbhz = S ? b.cn0_dbhz : 0.0;
  r.s.lli = L ? static_cast<uint8_t>((b.lli & 1) | ((b.status & 4) ? 0 : 2)) : 0;
  r.s.status = static_cast<uint8_t>((C ? 1 : 0) | (L ? 2 : 0) | ((L && (b.status & 4)) ? 4 : 0));
  return r;
}

bool bdsHigh(const SatId& s) { return s.sys == Sys::BDS && s.prn > 50; }

// RINEX epoch F11.7: the text rounds to 1e-7 s (≤ 5e-8 s; the exporter renders it from exact integer ticks), and
// the bag's tow double has a spacing of ≤ 1.2e-10 s below 604800 s.
constexpr double kRinexEpochTol = 0.5e-7 + 1e-9;

void compareRinexEpoch(const ObsEpoch& bag, const ObsEpoch& rnx, bool only_bds_high, int* n_sats, int* n_sigs, int* missing_high) {
  EXPECT_EQ(rnx.source, Source::Rinex);
  EXPECT_LE(std::fabs(rnx.t_rx.diff(bag.t_rx)), kRinexEpochTol);
  std::map<std::pair<int, int>, const SatObs*> rs;
  for (const SatObs& so : rnx.sats) rs[{static_cast<int>(so.sat.sys), so.sat.prn}] = &so;
  std::set<std::pair<int, int>> expected;
  for (const SatObs& b : bag.sats) {
    if (bdsHigh(b.sat) != only_bds_high) continue;
    std::vector<RinexExpect> ex;
    for (const SignalObs& s : b.sig) {
      const RinexExpect e = expectAfterRinex(s);
      if (e.present) ex.push_back(e);
    }
    if (ex.empty()) continue;
    const std::pair<int, int> key{static_cast<int>(b.sat.sys), b.sat.prn};
    expected.insert(key);
    const auto it = rs.find(key);
    if (it == rs.end()) {
      if (only_bds_high) ++*missing_high;
      else ADD_FAILURE() << "RINEX path lost " << sysName(b.sat.sys) << b.sat.prn;
      continue;
    }
    const SatObs& r = *it->second;
    EXPECT_EQ(r.sat.rtk_sat, testSatNo(b.sat.sys, b.sat.prn));   // RTKLIB's own numbering
    EXPECT_LE(std::fabs(r.t_rx.diff(b.t_rx)), kRinexEpochTol);
    ASSERT_EQ(r.sig.size(), ex.size()) << sysName(b.sat.sys) << b.sat.prn;
    EXPECT_EQ(isPrimary(r.sat.sys, r.sig[0].code), isPrimary(b.sat.sys, b.sig[0].code)) << "primary first";
    for (const RinexExpect& e : ex) {
      const auto sit = std::find_if(r.sig.begin(), r.sig.end(), [&](const SignalObs& x) { return x.code == e.s.code; });
      ASSERT_NE(sit, r.sig.end()) << signalCodeName(e.s.code);
      SCOPED_TRACE(std::string(sysName(b.sat.sys)) + std::to_string(b.sat.prn) + " " + signalCodeName(e.s.code));
      EXPECT_EQ(sit->freq_hz, e.s.freq_hz);
      EXPECT_LE(std::fabs(sit->psr_m - e.s.psr_m), 5.001e-4);    // F14.3
      EXPECT_LE(std::fabs(sit->cp_cyc - e.s.cp_cyc), 5.001e-4);
      EXPECT_LE(std::fabs(sit->dopp_hz - e.s.dopp_hz), 1e-3);    // F14.3 + RTKLIB's float Doppler
      EXPECT_LE(std::fabs(sit->cn0_dbhz - e.s.cn0_dbhz), 5.001e-4);
      EXPECT_EQ(sit->lli, e.s.lli);
      EXPECT_EQ(sit->status, e.s.status);
      ++*n_sigs;
    }
    ++*n_sats;
  }
  for (const auto& kv : rs)
    if (bdsHigh(kv.second->sat) == only_bds_high) {
      EXPECT_TRUE(expected.count(kv.first)) << "unexpected RINEX satellite";
    }
}

// ================================================================================================= T5 helpers
std::vector<uint8_t> ubxFrame(uint8_t cls, uint8_t id, const std::vector<uint8_t>& payload) {
  std::vector<uint8_t> f = {0xB5, 0x62, cls, id, static_cast<uint8_t>(payload.size() & 0xFF), static_cast<uint8_t>(payload.size() >> 8)};
  f.insert(f.end(), payload.begin(), payload.end());
  uint8_t a = 0, b = 0;
  for (size_t i = 2; i < f.size(); ++i) {
    a = static_cast<uint8_t>(a + f[i]);
    b = static_cast<uint8_t>(b + a);
  }
  f.push_back(a);
  f.push_back(b);
  return f;
}
template <typename T>
void put(std::vector<uint8_t>* b, size_t at, T v) {
  if (b->size() < at + sizeof(T)) b->resize(at + sizeof(T));
  std::memcpy(b->data() + at, &v, sizeof(T));   // little-endian host
}

struct RawxMeas {
  double pr, cp;
  float dop;
  uint8_t gnss, sv, sig, freq_id;
  uint16_t lock_ms;
  uint8_t cno, pr_std, cp_std, do_std, trk;
};
std::vector<uint8_t> rawxPayload(double tow, uint16_t week, const std::vector<RawxMeas>& m) {
  std::vector<uint8_t> p(16 + 32 * m.size(), 0);
  put(&p, 0, tow);
  put(&p, 8, week);
  put(&p, 10, static_cast<int8_t>(18));
  put(&p, 11, static_cast<uint8_t>(m.size()));
  put(&p, 12, static_cast<uint8_t>(0x01));
  put(&p, 13, static_cast<uint8_t>(1));   // version 1: sigId valid
  for (size_t i = 0; i < m.size(); ++i) {
    const size_t o = 16 + 32 * i;
    put(&p, o, m[i].pr);
    put(&p, o + 8, m[i].cp);
    put(&p, o + 16, m[i].dop);
    p[o + 20] = m[i].gnss;
    p[o + 21] = m[i].sv;
    p[o + 22] = m[i].sig;
    p[o + 23] = m[i].freq_id;
    put(&p, o + 24, m[i].lock_ms);
    p[o + 26] = m[i].cno;
    p[o + 27] = m[i].pr_std;
    p[o + 28] = m[i].cp_std;
    p[o + 29] = m[i].do_std;
    p[o + 30] = m[i].trk;
  }
  return p;
}

// GPS LNAV subframe (10 words × 24 data bits, bit positions as in IS-GPS-200 / RTKLIB decode_frame).
struct Bits {
  uint8_t b[30] = {0};
  void setu(int pos, int len, uint64_t v) {
    for (int i = 0; i < len; ++i) {
      const int bit = pos + i;
      const uint64_t one = (v >> (len - 1 - i)) & 1u;
      if (one) b[bit / 8] |= static_cast<uint8_t>(1u << (7 - bit % 8));
      else b[bit / 8] &= static_cast<uint8_t>(~(1u << (7 - bit % 8)));
    }
  }
  void sets(int pos, int len, int64_t v) { setu(pos, len, static_cast<uint64_t>(v) & ((len == 64) ? ~0ull : ((1ull << len) - 1))); }
  uint32_t word24(int i) const {
    uint32_t v = 0;
    for (int k = 0; k < 24; ++k) {
      const int bit = 24 * i + k;
      v = (v << 1) | ((b[bit / 8] >> (7 - bit % 8)) & 1u);
    }
    return v;
  }
};
std::vector<uint8_t> sfrbxGps(uint8_t prn, const Bits& sub) {
  std::vector<uint8_t> p(8 + 40, 0);
  p[0] = 0;       // gnssId GPS
  p[1] = prn;
  p[2] = 0;       // sigId L1 C/A
  p[4] = 10;      // numWords
  p[6] = 2;       // version
  for (int i = 0; i < 10; ++i) put(&p, 8 + 4 * static_cast<size_t>(i), static_cast<uint32_t>(sub.word24(i) << 6));   // parity bits not checked
  return p;
}
void lnavHeader(Bits* s, int id, uint32_t tow_count) {
  s->setu(0, 8, 0x8B);
  s->setu(24, 17, tow_count);
  s->setu(43, 3, static_cast<uint64_t>(id));
}

// ================================================================================================= T1: conventions
// Discriminators of the gnss_comm time conventions that need no statistics: every broadcast epoch has a fixed
// resolution in its own time system (interface control documents), so the §12.3 convention "every GnssTimeMsg is
// GPST" predicts one residue per record in GPST seconds and each alternative predicts another:
//   GPS  toe, toc  LSB 16 s (IS-GPS-200)                           GPST s ≡ 0 (mod 16)
//   GAL  toe, toc  LSB 60 s (Galileo OS SIS ICD)                   GPST s ≡ 0 (mod 60)    (GST seconds = GPST's)
//   BDS  toe, toc  LSB 8 s in BDT (BDS SIS ICD B1I)                GPST s − 14 ≡ 0 (mod 8); stored BDT gives 6
//   GLO  tb, 15 min in Moscow time = UTC + 3 h (GLONASS ICD)       GPST s − 18 ≡ 0 (mod 900); UTC/GLONASST give 882
//   all  |toe − first epoch of the slice| < 1 week                 BDT/GST week numbers are 1356/1024 weeks off
// (18 s = GPST − UTC for every date since 2017-01-01.) Hourly BDS uploads additionally put toe on whole BDT hours;
// seeing at least one corroborates the 14 s. 604800 is a multiple of every modulus, so residues of seconds since
// the GPST epoch equal residues of the time of week.
struct TimeScaleReport {
  std::vector<std::string> violations;
  std::map<char, int> checked;   // records checked per system letter
  int bds_whole_bdt_hour = 0;
};

TimeScaleReport timeScaleDiscriminators(const std::vector<KeplerEph>& eph, const std::vector<GloEph>& geph, const GTime& epoch) {
  TimeScaleReport r;
  auto residue = [&](const std::string& what, const GTime& t, int64_t offset, int64_t lsb) {
    if (t.frac != 0.0) {
      r.violations.push_back(what + ": not a whole second");
      return;
    }
    const int64_t s = t.sec - kGpst0 - offset;
    const int64_t m = ((s % lsb) + lsb) % lsb;
    if (m != 0)
      r.violations.push_back(what + ": (GPST s - " + std::to_string(offset) + ") mod " + std::to_string(lsb) + " = " + std::to_string(m));
  };
  auto nearSlice = [&](const std::string& what, const GTime& t) {
    if (!(std::fabs(t.diff(epoch)) < 604800.0)) r.violations.push_back(what + ": more than a week from the slice");
  };
  for (const KeplerEph& e : eph) {
    const std::string id = std::string(1, sysChar(e.sat.sys)) + std::to_string(e.sat.prn);
    if (e.sat.sys == Sys::GPS) {
      residue(id + " toe", e.toe, 0, 16);
      residue(id + " toc", e.toc, 0, 16);
    } else if (e.sat.sys == Sys::GAL) {
      residue(id + " toe", e.toe, 0, 60);
      residue(id + " toc", e.toc, 0, 60);
    } else if (e.sat.sys == Sys::BDS) {
      residue(id + " toe", e.toe, 14, 8);
      residue(id + " toc", e.toc, 14, 8);
      if (e.toe.frac == 0.0 && (e.toe.sec - kGpst0 - 14) % 3600 == 0) ++r.bds_whole_bdt_hour;
    } else {
      continue;
    }
    nearSlice(id + " toe", e.toe);
    ++r.checked[sysChar(e.sat.sys)];
  }
  for (const GloEph& g : geph) {
    const std::string id = "R" + std::to_string(g.sat.prn);
    residue(id + " toe", g.toe, 18, 900);
    nearSlice(id + " toe", g.toe);
    ++r.checked['R'];
  }
  return r;
}

// All ephemerides / observation epochs of a fixture through the bag path.
struct FixtureNav {
  std::vector<KeplerEph> eph;
  std::vector<GloEph> geph;
  std::vector<Klobuchar> ion;
  std::vector<double> ion_t_record;
  std::vector<std::pair<double, Sys>> eph_t_record;   // record time and system of every ephemeris message pushed
  GTime first_epoch;
};
FixtureNav fixtureNav(const std::string& fx) {
  FixtureNav out;
  std::string why;
  std::vector<ts::Record> recs;
  std::vector<bool> ok;
  CaptureSink s;
  GnssCommRos1Decoder dec(testSatNo);
  for (const char* topic : {"ephem", "glo_ephem", "iono_params", "range_meas"}) {
    if (!ts::isFile(fx + "/msgdef/" + topic + ".msgdef")) continue;
    const size_t n_eph = s.eph.size(), n_ion = s.ion.size();
    EXPECT_TRUE(decodeTopic(fx, topic, &dec, &s, &recs, &ok, &why)) << fx << " " << topic << ": " << why;
    for (size_t i = n_eph; i < s.eph.size(); ++i) out.eph_t_record.emplace_back(s.eph[i].t_record, s.eph[i].sat.sys);
    for (size_t i = n_ion; i < s.ion.size(); ++i) out.ion_t_record.push_back(s.ion[i].t_record);
  }
  out.eph = s.eph;
  out.geph = s.geph;
  out.ion = s.ion;
  if (!s.obs.empty()) out.first_epoch = s.obs.front().t_rx;
  return out;
}

// IONOSPHERIC CORR records of a RINEX 3 NAV header: type ("GPSA", "GPSB", "BDSA", ...) → the four values of every
// line of that type, in file order.
std::map<std::string, std::vector<std::array<double, 4>>> ionoHeader(const std::string& path) {
  std::map<std::string, std::vector<std::array<double, 4>>> out;
  std::string text;
  if (!ts::readFile(path, &text)) return out;
  std::stringstream ss(text);
  for (std::string l; std::getline(ss, l);) {
    if (l.size() > 60 && l.find("END OF HEADER", 60) != std::string::npos) break;
    if (l.size() < 60 || l.find("IONOSPHERIC CORR", 60) == std::string::npos) continue;
    std::array<double, 4> v{};
    for (size_t k = 0; k < 4; ++k) v[k] = rnum(l, 5 + 12 * k, 12);
    out[l.substr(0, 4)].push_back(v);
  }
  return out;
}

// Half a unit in the last place of x printed with `digits` decimals in scientific notation (RINEX %12.4E: 4,
// D19.12: 12): the rounding the text adds to the exact binary value.
double sciHalfUlp(double x, int digits) {
  if (x == 0.0) return 0.0;
  return 0.5 * std::pow(10.0, std::floor(std::log10(std::fabs(x))) - digits);
}

// Bound for a value that went double → D19.12 text → double: the text's half-ulp plus two conversions of ≤ ½ ulp.
double d19Tol(double x) { return sciHalfUlp(x, 12) + 2.0 * DBL_EPSILON * std::fabs(x); }

// P1 (WP-a's MAXPRNCMP 63) in this build, decided by the dependency itself, not by what a test observes:
// livo_gnss::satNoOf(BDS, 59) is non-zero exactly when RTKLIB numbers C51–C63. Before WP-a's rtk_adapter is merged
// satNoOf is the skeleton stub, which throws.
enum class P1State { NotMerged, Absent, Present };
P1State p1State() {
  try {
    return livo_gnss::satNoOf(Sys::BDS, 59) != 0 ? P1State::Present : P1State::Absent;
  } catch (const std::logic_error&) {
    return P1State::NotMerged;
  }
}

}  // namespace

// ================================================================================================= T1
// T1: gnss_comm field trees → ObsEpoch/KeplerEph/GloEph/Klobuchar/ReceiverPvt per DESIGN §12.3 on the fixture
TEST(T01Io, MappingToObsEpochMatchesDesignTable) {
  REQUIRE_SYNTHETIC_WIRE_FIXTURE();
  ASSERT_TRUE(ts::isDir(kSynth)) << kSynth;
  EdgeTally t;
  checkObsMapping(kSynth, &t);
  // The synthetic fixture exists to exercise the boundaries: each must actually occur.
  EXPECT_GE(t.empty_epochs, 1);
  EXPECT_GE(t.empty_sats, 1);
  EXPECT_GE(t.b1c, 1);
  EXPECT_GE(t.unknown, 2);        // GPS 1381.05 MHz and an off-grid GLONASS frequency
  EXPECT_GE(t.gal_prn, 1);        // E37
  EXPECT_GE(t.reordered, 1);      // secondary listed first in the message
  EXPECT_GE(t.glo_kmin, 1);
  EXPECT_GE(t.glo_kmax, 1);
  EXPECT_GE(t.bds_geo, 1);
  EXPECT_GE(t.lli, 1);
  EXPECT_GE(t.repeated_code, 1);   // second E1 entry, listed after and before the original
  for (int s : {0x00, 0x01, 0x03, 0x07, 0x0F}) EXPECT_TRUE(t.status_values.count(s)) << "status " << s;
  checkEphMapping(kSynth, true);
  checkIonoPvt(kSynth, true);
}

// T1 on field trees read from the rosbags golden CSV: the mapping gives the same result as from the bytes
TEST(T01Io, MappingFromGoldenCsvTreesEqualsBytePath) {
  REQUIRE_SYNTHETIC_WIRE_FIXTURE();
  ts::TopicSchema sch;
  std::string why;
  ASSERT_TRUE(ts::loadTopic(kSynth, "range_meas", &sch, &why)) << why;
  ts::Csv csv;
  ASSERT_TRUE(loadCsv(kSynth, "obs", &csv));
  const std::vector<ObsEpoch> bag = bagEpochs(kSynth, nullptr);
  size_t k = 0;
  for (size_t r = 0; r < csv.rows.size(); ++r) {
    FieldNode tree;
    uint64_t t_ns = 0;
    ASSERT_TRUE(ts::rowToTree(sch.schema, csv, r, &tree, &t_ns, &why)) << why;
    ObsEpoch ep;
    MappingStats st;
    ASSERT_TRUE(toObsEpoch(tree, recordTime(t_ns), testSatNo, &ep, &st, &why)) << why;
    if (!ep.t_rx.valid()) continue;
    ASSERT_LT(k, bag.size());
    const ObsEpoch& b = bag[k++];
    ASSERT_EQ(ep.sats.size(), b.sats.size());
    for (size_t i = 0; i < ep.sats.size(); ++i) {
      ASSERT_EQ(ep.sats[i].sig.size(), b.sats[i].sig.size());
      EXPECT_EQ(ep.sats[i].sat.rtk_sat, b.sats[i].sat.rtk_sat);
      for (size_t q = 0; q < ep.sats[i].sig.size(); ++q) {
        EXPECT_EQ(std::memcmp(&ep.sats[i].sig[q].psr_m, &b.sats[i].sig[q].psr_m, sizeof(double)), 0);
        EXPECT_EQ(std::memcmp(&ep.sats[i].sig[q].cp_cyc, &b.sats[i].sig[q].cp_cyc, sizeof(double)), 0);
        EXPECT_EQ(ep.sats[i].sig[q].code, b.sats[i].sig[q].code);
      }
    }
  }
  EXPECT_EQ(k, bag.size());
  // A tree without the required fields is refused with a reason (not a crash).
  FieldNode empty;
  empty.is_message = true;
  ObsEpoch ep;
  EXPECT_FALSE(toObsEpoch(empty, 0.0, testSatNo, &ep, nullptr, &why));
  EXPECT_FALSE(why.empty());
  KeplerEph e;
  EXPECT_FALSE(toKeplerEph(empty, 0.0, testSatNo, &e, nullptr, &why));
  Klobuchar kl;
  EXPECT_FALSE(toKlobuchar(empty, 0.0, &kl, &why));
  // numbering boundaries (§12.3)
  int prn = -1;
  EXPECT_EQ(gnssCommSatToSys(0, &prn), Sys::Unknown);
  EXPECT_EQ(gnssCommSatToSys(32, &prn), Sys::GPS);
  EXPECT_EQ(prn, 32);
  EXPECT_EQ(gnssCommSatToSys(33, &prn), Sys::GLO);
  EXPECT_EQ(prn, 1);
  EXPECT_EQ(gnssCommSatToSys(59, &prn), Sys::GLO);
  EXPECT_EQ(prn, 27);
  EXPECT_EQ(gnssCommSatToSys(60, &prn), Sys::GAL);
  EXPECT_EQ(prn, 1);
  EXPECT_EQ(gnssCommSatToSys(97, &prn), Sys::GAL);
  EXPECT_EQ(prn, 38);
  EXPECT_EQ(gnssCommSatToSys(98, &prn), Sys::BDS);
  EXPECT_EQ(prn, 1);
  EXPECT_EQ(gnssCommSatToSys(160, &prn), Sys::BDS);
  EXPECT_EQ(prn, 63);
  EXPECT_EQ(gnssCommSatToSys(161, &prn), Sys::Unknown);
  for (uint32_t s = 1; s <= 160; ++s) {   // all 160 numbers round-trip through (sys, prn)
    const Sys sys = gnssCommSatToSys(s, &prn);
    const uint32_t back = sys == Sys::GPS ? prn : sys == Sys::GLO ? 32 + prn : sys == Sys::GAL ? 59 + prn : 97 + prn;
    EXPECT_EQ(back, s);
  }
}

TEST(T01Io, MappingDevSetFixture) {
  int n = 0;
  std::map<std::string, int> with_topic;
  for (const std::string& fx : ts::realFixtures(kRoot)) {
    SCOPED_TRACE(fx);
    ASSERT_TRUE(hasTopic(fx, "range_meas")) << "the exporter makes no fixture without range_meas";
    EdgeTally t;
    checkObsMapping(fx, &t);
    checkEphMapping(fx, false);   // no synthetic conventions asserted (toe_tow etc.); topics the bag lacks are skipped
    checkIonoPvt(fx, false);
    for (const char* topic : {"ephem", "glo_ephem", "iono_params", "receiver_pvt"}) with_topic[topic] += hasTopic(fx, topic) ? 1 : 0;
    ++n;
  }
  if (n == 0) GTEST_SKIP() << "blocked: dev-set fixture (A24: no gnss/test/data/<seq>_<t0>_<D>s from a development bag)";
  // Not vacuous per topic: real receiver output of every message type went through the mapping at least once.
  for (const auto& kv : with_topic) EXPECT_GT(kv.second, 0) << "no development fixture carries " << kv.first;
}

// Signals dropped by the first-listed-wins rule (signal_order.hpp rule 1) are counted in
// MappingStats::rejected_repeated_code (WP-e #7, A39): per repeated signal of a satellite the mapping carries, never for a
// signal rejected for its frequency, never for a satellite rejected before its signals are looked at; the kept signal is
// the first one listed. Hand-built field trees (the gnss_comm layout); the fixtures' counts are checked against the
// golden CSV by checkObsMapping (T1 MappingToObsEpochMatchesDesignTable / MappingDevSetFixture).
namespace {
struct SigIn {
  double f_hz, psr_m;
};
FieldNode scalarNode(const std::string& name, BuiltinType t, double v) {
  FieldNode n;
  n.name = name;
  n.type = builtinName(t);
  n.scalar.type = t;
  if (t == BuiltinType::Float64) n.scalar.f = v;
  else n.scalar.u = static_cast<uint64_t>(v);
  return n;
}
FieldNode arrayNode(const std::string& name, BuiltinType t, const std::vector<double>& v) {
  FieldNode n;
  n.name = name;
  n.type = builtinName(t);
  n.is_array = true;
  for (double x : v) {
    Value val;
    val.type = t;
    if (t == BuiltinType::Float64) val.f = x;
    else val.u = static_cast<uint64_t>(x);
    n.array.push_back(val);
  }
  return n;
}
// gnss_comm/GnssMeasMsg with one GnssObsMsg per (sat, signals); every signal valid (status 0x07), psr as given.
FieldNode measTree(uint32_t week, double tow, const std::vector<std::pair<uint32_t, std::vector<SigIn>>>& sats) {
  FieldNode root;
  root.type = "gnss_comm/GnssMeasMsg";
  root.is_message = true;
  FieldNode meas;
  meas.name = "meas";
  meas.type = "GnssObsMsg";
  meas.is_array = meas.is_message = true;
  for (size_t k = 0; k < sats.size(); ++k) {
    std::vector<double> f, psr, one, zero, status;
    for (const SigIn& s : sats[k].second) {
      f.push_back(s.f_hz);
      psr.push_back(s.psr_m);
      one.push_back(1.0);
      zero.push_back(0.0);
      status.push_back(7.0);
    }
    FieldNode e;
    e.name = std::to_string(k);
    e.type = "GnssObsMsg";
    e.is_message = true;
    FieldNode time;
    time.name = "time";
    time.type = "GnssTimeMsg";
    time.is_message = true;
    time.children = {scalarNode("week", BuiltinType::UInt32, week), scalarNode("tow", BuiltinType::Float64, tow)};
    e.children = {time,
                  scalarNode("sat", BuiltinType::UInt32, sats[k].first),
                  arrayNode("freqs", BuiltinType::Float64, f),
                  arrayNode("CN0", BuiltinType::Float64, std::vector<double>(f.size(), 40.0)),
                  arrayNode("LLI", BuiltinType::UInt8, zero),
                  arrayNode("code", BuiltinType::UInt8, zero),
                  arrayNode("psr", BuiltinType::Float64, psr),
                  arrayNode("psr_std", BuiltinType::Float64, one),
                  arrayNode("cp", BuiltinType::Float64, zero),
                  arrayNode("cp_std", BuiltinType::Float64, one),
                  arrayNode("dopp", BuiltinType::Float64, zero),
                  arrayNode("dopp_std", BuiltinType::Float64, one),
                  arrayNode("status", BuiltinType::UInt8, status)};
    meas.children.push_back(e);
  }
  root.children.push_back(meas);
  return root;
}
}  // namespace

TEST(T01Io, RepeatedCodeIsCounted) {
  REQUIRE_SYNTHETIC_WIRE_FIXTURE();
  const double L1 = 1575.42e6, L2 = 1227.60e6, E5b = 1207.14e6, G1_k1 = 1602e6 + 0.5625e6, UNKNOWN = 1381.05e6;
  // gnss_comm numbering: GPS = prn, GLO = 32 + prn, GAL = 59 + prn, BDS = 97 + prn
  const FieldNode tree = measTree(2350, 205800.0,
                                  {{70, {{L1, 1.0}, {L1, 2.0}, {E5b, 3.0}}},                // E11: E1-B, E1-C (dropped), E5b
                                   {71, {{E5b, 4.0}, {L1, 5.0}, {L1, 6.0}, {L1, 7.0}}},     // E12: E5b first, three E1 (two dropped)
                                   {5, {{L1, 8.0}, {L2, 9.0}}},                           // G05: nothing repeated
                                   {6, {{UNKNOWN, 10.0}, {UNKNOWN, 11.0}}},               // G06: unknown frequency twice
                                   {107, {{L1, 12.0}, {L1, 13.0}}},                       // C10: BDS B1C twice (rejected)
                                   {96, {{L1, 14.0}, {L1, 15.0}}},                        // E37: no RTKLIB number
                                   {0, {{L1, 16.0}, {L1, 17.0}}},                         // outside the numbering
                                   {39, {{G1_k1, 18.0}, {G1_k1, 19.0}}}});                // R07 k=+1: G1 twice
  MappingStats st;
  ObsEpoch ep;
  std::string why;
  ASSERT_TRUE(toObsEpoch(tree, 1.0, testSatNo, &ep, &st, &why)) << why;
  EXPECT_EQ(st.rejected_repeated_code, 4u);   // E11 1 + E12 2 + R07 1
  EXPECT_EQ(st.rejected_unknown_freq, 2u);    // G06: counted as unknown frequencies, not as repeats
  EXPECT_EQ(st.rejected_bds_b1c, 2u);         // C10: likewise
  EXPECT_EQ(st.rejected_gal_prn, 1u);         // E37: signals never examined
  EXPECT_EQ(st.rejected_sat_range, 1u);       // sat 0: likewise
  EXPECT_EQ(st.sats, 4u);                     // E11, E12, G05, R07 carried
  ASSERT_EQ(ep.sats.size(), 4u);
  auto psrs = [](const SatObs& so) {
    std::vector<double> v;
    for (const SignalObs& s : so.sig) v.push_back(s.psr_m);
    return v;
  };
  EXPECT_EQ(psrs(ep.sats[0]), (std::vector<double>{1.0, 3.0}));   // first E1 kept
  EXPECT_EQ(psrs(ep.sats[1]), (std::vector<double>{5.0, 4.0}));   // first E1 kept, primary first
  EXPECT_EQ(psrs(ep.sats[2]), (std::vector<double>{8.0, 9.0}));
  EXPECT_EQ(psrs(ep.sats[3]), (std::vector<double>{18.0}));
  // The counter accumulates per MappingStats (the decoder's lifetime), and a null MappingStats is allowed.
  ASSERT_TRUE(toObsEpoch(tree, 2.0, testSatNo, &ep, &st, &why)) << why;
  EXPECT_EQ(st.rejected_repeated_code, 8u);
  ASSERT_TRUE(toObsEpoch(tree, 3.0, testSatNo, &ep, nullptr, &why)) << why;
  EXPECT_EQ(ep.sats.size(), 4u);

  // Through the RawDecoder front end on the synthetic fixture (E1-C injected after and before the E1-B entry): the
  // counter equals the independent count from the golden CSV and is not zero.
  MappingStats fx_stats;
  bagEpochs(kSynth, &fx_stats);
  EdgeTally t;
  checkObsMapping(kSynth, &t);
  EXPECT_GE(t.repeated_code, 1);
  EXPECT_EQ(fx_stats.rejected_repeated_code, static_cast<uint64_t>(t.repeated_code));
}

// The discriminators themselves: silent on the synthetic fixture (written under the §12.3 conventions) and loud on
// each alternative convention — every record it touches is flagged. Proves the checker, not the data.
TEST(T01Io, TimeScaleDiscriminatorsCatchEachAlternative) {
  REQUIRE_SYNTHETIC_WIRE_FIXTURE();
  const FixtureNav nav = fixtureNav(kSynth);
  ASSERT_TRUE(nav.first_epoch.valid());
  const TimeScaleReport ok = timeScaleDiscriminators(nav.eph, nav.geph, nav.first_epoch);
  EXPECT_TRUE(ok.violations.empty()) << ok.violations.front();
  std::map<char, int> checked = ok.checked;
  for (char c : {'G', 'E', 'C', 'R'}) EXPECT_GT(checked[c], 0) << c;
  EXPECT_GT(ok.bds_whole_bdt_hour, 0);

  struct Alternative {
    const char* what;
    Sys sys;
    double shift_s;   // what the stored GPST label would be off by under the alternative
    int per_record;   // violations per record (toe [+ toc]; the week check adds one for week shifts)
  };
  const std::vector<Alternative> alts = {
      {"BDS epochs stored in BDT", Sys::BDS, -14.0, 2},
      {"BDS week number in BDT (1356 weeks)", Sys::BDS, -1356.0 * 604800.0, 1},
      {"GAL week number in GST (1024 weeks)", Sys::GAL, -1024.0 * 604800.0, 1},
      {"GPS epochs off by the leap seconds", Sys::GPS, 18.0, 2},
      {"GLONASS epochs stored in UTC", Sys::GLO, -18.0, 1},
      {"GLONASS epochs stored in GLONASST", Sys::GLO, 10800.0 - 18.0, 1}};
  for (const Alternative& a : alts) {
    SCOPED_TRACE(a.what);
    std::vector<KeplerEph> eph = nav.eph;
    std::vector<GloEph> geph = nav.geph;
    int touched = 0;
    for (KeplerEph& e : eph)
      if (e.sat.sys == a.sys) {
        e.toe = e.toe.add(a.shift_s);
        e.toc = e.toc.add(a.shift_s);
        ++touched;
      }
    for (GloEph& g : geph)
      if (a.sys == Sys::GLO) {
        g.toe = g.toe.add(a.shift_s);
        ++touched;
      }
    ASSERT_GT(touched, 0);
    const TimeScaleReport bad = timeScaleDiscriminators(eph, geph, nav.first_epoch);
    EXPECT_EQ(static_cast<int>(bad.violations.size()), a.per_record * touched);
  }
}

// DESIGN §12.3 on real data: every GnssTimeMsg of the bag's ephemerides is GPST (BDS: BDT + 14 s, GLONASS:
// UTC + 18 s) — decided per record by the discriminators above, on the development fixtures only.
TEST(T01Io, DevSetEphemerisTimeScales) {
  int n_fx = 0;
  std::map<char, int> checked;
  int bds_hour = 0;
  for (const std::string& fx : ts::realFixtures(kRoot)) {
    SCOPED_TRACE(fx);
    const FixtureNav nav = fixtureNav(fx);
    if (nav.eph.empty() && nav.geph.empty()) continue;   // e.g. Occlusion03/04, Dynamic03 carry no ephemeris topics
    ASSERT_TRUE(nav.first_epoch.valid());
    const TimeScaleReport r = timeScaleDiscriminators(nav.eph, nav.geph, nav.first_epoch);
    for (const std::string& v : r.violations) ADD_FAILURE() << v;
    for (const auto& kv : r.checked) checked[kv.first] += kv.second;
    bds_hour += r.bds_whole_bdt_hour;
    ++n_fx;
  }
  if (n_fx == 0) GTEST_SKIP() << "blocked: dev-set fixture with ephemerides (A24)";
  // The discriminators that matter must actually have run: a fixture slice without BDS or GLONASS ephemerides
  // would leave the 14 s / 18 s question open (choose another slice).
  for (char c : {'G', 'E', 'C', 'R'}) EXPECT_GT(checked[c], 0) << "no " << c << " ephemeris in the development fixtures";
  EXPECT_GT(bds_hour, 0) << "no BDS toe on a whole BDT hour";
}

// The eight iono_params values are a Klobuchar set in the LNAV/D1 order and scale: α0..α3 in units of 2^-30,
// 2^-27, 2^-24, 2^-24 s/semicircle^n and β0..β3 in 2^11, 2^14, 2^16, 2^16 s/semicircle^n, each an 8-bit signed
// integer on the broadcast (the scales are the same for GPS LNAV and BDS D1). A swapped α/β block or a scale error
// breaks the integrality, so this pins the mapping's 8 → alpha[0..3], beta[0..3].
namespace {
std::vector<std::string> klobucharOffGrid(const Klobuchar& k) {
  static const int kAlpha[4] = {-30, -27, -24, -24}, kBeta[4] = {11, 14, 16, 16};
  std::vector<std::string> v;
  for (int i = 0; i < 4; ++i) {
    const double a = std::ldexp(k.alpha[i], -kAlpha[i]), b = std::ldexp(k.beta[i], -kBeta[i]);
    if (a != std::round(a)) v.push_back("alpha" + std::to_string(i) + " = " + std::to_string(k.alpha[i]) + " is not a multiple of its LSB");
    if (b != std::round(b)) v.push_back("beta" + std::to_string(i) + " = " + std::to_string(k.beta[i]) + " is not a multiple of its LSB");
    if (!(a >= -128 && a <= 127 && b >= -128 && b <= 127)) v.push_back("coefficient " + std::to_string(i) + " outside the 8-bit range");
  }
  return v;
}
}  // namespace

TEST(T01Io, DevSetIonoParamsKlobucharQuantisation) {
  int n = 0;
  for (const std::string& fx : ts::realFixtures(kRoot)) {
    SCOPED_TRACE(fx);
    for (const Klobuchar& k : fixtureNav(fx).ion) {
      for (const std::string& v : klobucharOffGrid(k)) ADD_FAILURE() << v;
      ++n;
    }
  }
  if (n == 0) GTEST_SKIP() << "blocked: dev-set fixture with iono_params (A26: only Grass01/02 carry the topic)";
}

// AMENDMENTS A38 (revises A1 and DESIGN §12.3): the GPS Klobuchar set is taken from the RINEX BRDC header GPSA/GPSB only;
// the bag's iono_params are kept for diagnostics — on M3DGR they differ from the day's GPSA/GPSB (WP-e #9; each record
// follows a BDS ephemeris, BDS D1 subframe 1 inferred). Whether a set is used as GPS Klobuchar is decided in the core by
// its source (WP-a EphemStore: RINEX only); what gnss_io guarantees is checked here, on every development fixture that
// carries the topic:
//   * the mapping is value for value the rosbags-decoded golden CSV (bit-exact) and on the LNAV/D1 quantisation grid;
//   * every bag record reaches the sink tagged Source::GnssCommIono, record time kept, no validity epoch — never the
//     tag of the GPS set, which the fixture's BRDC header delivers through RinexSource as Source::Rinex, equal to the
//     header text;
//   * a bag record that differs from GPSA/GPSB beyond the header's %12.4E rounding is reported, not failed.
TEST(T01Io, DevSetIonoParamsDiagnosticOnly) {
  int n = 0, n_fx = 0, differing = 0;
  for (const std::string& fx : ts::realFixtures(kRoot)) {
    if (!hasTopic(fx, "iono_params")) continue;
    SCOPED_TRACE(fx);
    std::string why;
    // The GPS set of the day: BRDC header through RinexSource (RTKLIB readrnxt keeps the last GPSA/GPSB line it reads).
    const std::string brdc = ts::fixtureBrdc(fx);
    ASSERT_FALSE(brdc.empty()) << "fixture has iono_params but no brdc/";
    const auto hdr = ionoHeader(brdc);
    ASSERT_TRUE(hdr.count("GPSA") && hdr.count("GPSB")) << brdc;
    const std::array<double, 4>& ga = hdr.at("GPSA").back();
    const std::array<double, 4>& gb = hdr.at("GPSB").back();
    RinexSource src;
    ASSERT_TRUE(src.open("", brdc, GTime(), GTime(), &why)) << why;
    Klobuchar gps;
    ASSERT_TRUE(src.iono(&gps)) << brdc;
    EXPECT_EQ(gps.source, Source::Rinex);
    for (int i = 0; i < 4; ++i) {   // RTKLIB str2num and rnum() convert the same header text with strtod
      EXPECT_EQ(gps.alpha[i], ga[i]) << "GPSA " << i;
      EXPECT_EQ(gps.beta[i], gb[i]) << "GPSB " << i;
    }
    // The bag records, one by one against the golden CSV.
    GnssCommRos1Decoder dec(testSatNo);
    CaptureSink sink;
    std::vector<ts::Record> recs;
    std::vector<bool> ok;
    ASSERT_TRUE(decodeTopic(fx, "iono_params", &dec, &sink, &recs, &ok, &why)) << why;
    ts::Csv csv;
    ASSERT_TRUE(loadCsv(fx, "iono", &csv));
    ASSERT_EQ(csv.rows.size(), recs.size());
    const FixtureNav nav = fixtureNav(fx);   // record times of the ephemerides, for the diagnostic line
    size_t pushed = 0;
    for (size_t r = 0; r < csv.rows.size(); ++r) {
      SCOPED_TRACE("iono_params record " + std::to_string(r));
      const Row row{csv, r};
      int nv = 0;
      while (row.has("data_" + std::to_string(nv))) ++nv;
      if (nv != 8) {   // not a Klobuchar set at all: refused and counted (checkIonoPvt)
        EXPECT_FALSE(ok[r]);
        continue;
      }
      ASSERT_TRUE(ok[r]);
      ASSERT_LT(pushed, sink.ion.size());
      const Klobuchar& k = sink.ion[pushed++];
      for (int i = 0; i < 4; ++i) {
        EXPECT_EQ(k.alpha[i], row.d("data_" + std::to_string(i)));
        EXPECT_EQ(k.beta[i], row.d("data_" + std::to_string(4 + i)));
      }
      for (const std::string& v : klobucharOffGrid(k)) ADD_FAILURE() << v;
      EXPECT_EQ(k.source, Source::GnssCommIono);
      EXPECT_NE(k.source, gps.source);
      EXPECT_FALSE(k.t_valid.valid());   // header.stamp is host time
      EXPECT_EQ(k.t_record, recordTime(recs[r].t_ns));
      bool same = true;
      for (int i = 0; i < 4; ++i) {
        same = same && std::fabs(k.alpha[i] - ga[i]) <= sciHalfUlp(ga[i], 4) + DBL_EPSILON * std::fabs(ga[i]);
        same = same && std::fabs(k.beta[i] - gb[i]) <= sciHalfUlp(gb[i], 4) + DBL_EPSILON * std::fabs(gb[i]);
      }
      if (!same) {
        ++differing;
        std::string after = "none";   // the system of the ephemeris message recorded just before this record
        double best = -1e300;
        for (const auto& e : nav.eph_t_record)
          if (e.first <= k.t_record && e.first > best) {
            best = e.first;
            after = std::string(sysName(e.second)) + " ephemeris " + std::to_string(k.t_record - e.first) + " s earlier";
          }
        std::printf("[T1 iono diagnostic] %s record %zu: alpha=[%.4e %.4e %.4e %.4e] beta=[%.4e %.4e %.4e %.4e] != GPSA/GPSB "
                    "[%.4e %.4e %.4e %.4e | %.4e %.4e %.4e %.4e]; recorded after: %s\n",
                    fx.substr(fx.find_last_of('/') + 1).c_str(), r, k.alpha[0], k.alpha[1], k.alpha[2], k.alpha[3], k.beta[0], k.beta[1],
                    k.beta[2], k.beta[3], ga[0], ga[1], ga[2], ga[3], gb[0], gb[1], gb[2], gb[3], after.c_str());
      }
      ++n;
    }
    EXPECT_EQ(pushed, sink.ion.size());
    EXPECT_EQ(dec.stats().iono, pushed);
    ++n_fx;
  }
  if (n == 0) GTEST_SKIP() << "blocked: dev-set fixture with iono_params (A26: only Grass01/02 carry the topic)";
  std::printf("[T1 iono] %d iono_params record(s) in %d development fixture(s); %d differ from the day's GPSA/GPSB: diagnostics "
              "only (A38)\n",
              n, n_fx, differing);
  RecordProperty("iono_params_records", n);
  RecordProperty("iono_params_not_gps_klobuchar", differing);
}

// ================================================================================================= T2
// T2: MsgDefParser::md5(schema) equals the bag connection-header md5sum for all fixture message types
// md5sums as recorded in the M3DGR connection headers (development bag Grass02; the first five are byte-identical
// in Dark01/Outdoor01, whose headers were read as a format audit, A2): a second oracle next to the fixtures' .md5.
namespace {
const std::map<std::string, std::string>& recordedMd5() {
  static const std::map<std::string, std::string> m = {{"range_meas", "0eafeb530a3e6637722c4e296734c119"},
                                                       {"ephem", "b4346f99e431c870e392a438684c0fe8"},
                                                       {"glo_ephem", "6893dee0d8b1c9325a6693f3488e0144"},
                                                       {"iono_params", "fb60495edd59d3fcf90e173153ae8a9a"},
                                                       {"receiver_pvt", "d18171357d7a159f76d4d7c0b12fb631"},
                                                       {"time_pulse_info", "de306035e295e3c4b1e0771a216b495a"}};
  return m;
}
}  // namespace

TEST(T02Io, MsgDefMd5MatchesConnectionHeader) {
  if (ts::listFixtures(kRoot).empty()) GTEST_SKIP() << "optional raw wire fixtures are not distributed in the research preview";
  const std::map<std::string, std::string>& recorded = recordedMd5();
  int checked = 0;
  for (const std::string& fx : ts::listFixtures(kRoot)) {
    for (const auto& tc : ts::topicCsv()) {
      if (!ts::isFile(fx + "/msgdef/" + tc.first + ".msgdef")) continue;
      SCOPED_TRACE(fx + " " + tc.first);
      ts::TopicSchema sch;
      std::string why;
      ASSERT_TRUE(ts::loadTopic(fx, tc.first, &sch, &why)) << why;
      EXPECT_EQ(MsgDefParser::md5(sch.schema), sch.md5);
      const auto it = recorded.find(tc.first);
      if (it != recorded.end()) {
        EXPECT_EQ(MsgDefParser::md5(sch.schema), it->second);
      }
      GnssCommRos1Decoder dec(testSatNo);
      EXPECT_TRUE(dec.registerMessageDefinition(sourceOfTopic(tc.first), sch.datatype, sch.md5, sch.msgdef, &why)) << why;
      EXPECT_TRUE(dec.registered(sourceOfTopic(tc.first)));
      // dropping the last field of the main type is a layout change: refused with a diff
      if (tc.first != "time_pulse_info") {
        std::vector<std::string> lines;
        {
          std::stringstream in(sch.msgdef);
          for (std::string l; std::getline(in, l);) lines.push_back(l);
        }
        size_t sep = 0;
        while (sep < lines.size() && lines[sep].compare(0, 3, "===") != 0) ++sep;
        size_t last = sep;
        for (size_t i = 0; i < sep; ++i) {
          const size_t nb = lines[i].find_first_not_of(" \t\r");
          if (nb != std::string::npos && lines[i][nb] != '#') last = i;
        }
        ASSERT_LT(last, sep);
        lines.erase(lines.begin() + static_cast<std::ptrdiff_t>(last));
        std::string cut;
        for (const std::string& l : lines) cut += l + "\n";
        GnssCommRos1Decoder d2(testSatNo);
        EXPECT_FALSE(d2.registerMessageDefinition(sourceOfTopic(tc.first), sch.datatype, sch.md5, cut, &why));
        EXPECT_NE(why.find("differs"), std::string::npos) << why;
      }
      ++checked;
    }
  }
  EXPECT_GE(checked, 5);
  // time_pulse_info is never interpreted (A19): any byte string is counted; its md5 is T2's sixth type below.
  GnssCommRos1Decoder dec(testSatNo);
  CaptureSink sink;
  std::string why;
  const uint8_t any[3] = {1, 2, 3};
  EXPECT_TRUE(dec.decode(Source::GnssCommTimePulse, any, 3, 0.0, sink, &why));
  EXPECT_EQ(dec.stats().time_pulse, 1u);
  // RawDecoder contract: non-ROS sources ignore the call (accepted, nothing registered, nothing decodable)
  EXPECT_TRUE(dec.registerMessageDefinition(Source::Rinex, "x/Y", "", "int8 a\n", &why));
  EXPECT_FALSE(dec.registered(Source::Rinex));
  EXPECT_FALSE(dec.decode(Source::Rinex, any, 3, 0.0, sink, &why));
}

// T2's sixth type, gnss_comm/GnssTimePulseInfoMsg: only bags that carry the topic can supply it (Dark01 and
// Outdoor01 have none; the development bag Grass02 has it). Its layout is parsed and decoded like any other, but
// never checked against an expectation table and never interpreted (A19).
TEST(T02Io, MsgDefMd5TimePulseInfo) {
  int n = 0;
  for (const std::string& fx : ts::listFixtures(kRoot)) {
    if (!ts::isFile(fx + "/msgdef/time_pulse_info.msgdef")) continue;
    SCOPED_TRACE(fx);
    ts::TopicSchema sch;
    std::string why;
    ASSERT_TRUE(ts::loadTopic(fx, "time_pulse_info", &sch, &why)) << why;
    EXPECT_EQ(MsgDefParser::md5(sch.schema), sch.md5);
    EXPECT_EQ(MsgDefParser::md5(sch.schema), recordedMd5().at("time_pulse_info"));
    std::vector<ts::Record> recs;
    ASSERT_TRUE(ts::readRecords(fx + "/bytes/time_pulse_info.bin", &recs, &why)) << why;
    ASSERT_FALSE(recs.empty());
    const Ros1Decoder raw(sch.schema);
    GnssCommRos1Decoder dec(testSatNo);
    ASSERT_TRUE(dec.registerMessageDefinition(Source::GnssCommTimePulse, sch.datatype, sch.md5, sch.msgdef, &why)) << why;
    CaptureSink sink;
    for (const ts::Record& r : recs) {
      FieldNode node;
      size_t used = 0;
      EXPECT_TRUE(raw.decode(r.data.data(), r.data.size(), &node, &used, &why)) << why;   // the layout parses the bytes
      EXPECT_TRUE(dec.decode(Source::GnssCommTimePulse, r.data.data(), r.data.size(), recordTime(r.t_ns), sink, &why)) << why;
    }
    EXPECT_EQ(dec.stats().time_pulse, recs.size());   // counted only (A19)
    EXPECT_TRUE(sink.obs.empty() && sink.eph.empty() && sink.geph.empty() && sink.ion.empty() && sink.pvt.empty());
    ++n;
  }
  if (n == 0) GTEST_SKIP() << "blocked: no fixture carries /ublox_driver/time_pulse_info (dev-set fixture from Grass02, A24)";
}

TEST(T02Io, MsgDefDevSetFixture) {
  int checked = 0;
  for (const std::string& fx : ts::realFixtures(kRoot)) {
    for (const auto& tc : ts::topicCsv()) {
      if (!ts::isFile(fx + "/msgdef/" + tc.first + ".msgdef")) continue;
      SCOPED_TRACE(fx + " " + tc.first);
      ts::TopicSchema sch;
      std::string why;
      ASSERT_TRUE(ts::loadTopic(fx, tc.first, &sch, &why)) << why;
      EXPECT_EQ(MsgDefParser::md5(sch.schema), sch.md5);
      EXPECT_EQ(sch.md5, recordedMd5().at(tc.first));
      if (tc.first != "time_pulse_info") {
        EXPECT_TRUE(validateLayout(sourceOfTopic(tc.first), sch.schema, &why)) << why;
      }
      ++checked;
    }
  }
  if (checked == 0) GTEST_SKIP() << "blocked: dev-set fixture (A24: no gnss/test/data/<seq>_<t0>_<D>s from a development bag)";
}

// ================================================================================================= T3
// T3: C++ decoder and tools/gnss_bag_export.py produce byte-identical CSV on the same slice
namespace {
void checkCsvIdentical(const std::string& fx, int* topics) {
  *topics = 0;
  for (const auto& tc : ts::topicCsv()) {
    if (!ts::isFile(fx + "/msgdef/" + tc.first + ".msgdef")) continue;
    SCOPED_TRACE(fx + " " + tc.first);
    ts::TopicSchema sch;
    std::string why;
    ASSERT_TRUE(ts::loadTopic(fx, tc.first, &sch, &why)) << why;
    std::vector<ts::Record> recs;
    ASSERT_TRUE(ts::readRecords(fx + "/bytes/" + tc.first + ".bin", &recs, &why)) << why;
    const Ros1Decoder dec(sch.schema);
    std::vector<std::pair<uint64_t, FieldNode>> msgs;
    for (const ts::Record& r : recs) {
      FieldNode n;
      size_t used = 0;
      ASSERT_TRUE(dec.decode(r.data.data(), r.data.size(), &n, &used, &why)) << why;
      msgs.emplace_back(r.t_ns, std::move(n));
    }
    const std::string mine = ts::goldenCsv(sch.schema, msgs);
    std::string golden;
    ASSERT_TRUE(ts::readFile(fx + "/golden/" + tc.second + ".csv", &golden));
    if (mine != golden) {
      size_t i = 0;
      while (i < mine.size() && i < golden.size() && mine[i] == golden[i]) ++i;
      const size_t line = static_cast<size_t>(std::count(golden.begin(), golden.begin() + static_cast<std::ptrdiff_t>(std::min(i, golden.size())), '\n'));
      ADD_FAILURE() << "first difference at byte " << i << " (line " << line + 1 << "): C++ '" << mine.substr(i, 40) << "' vs Python '"
                    << golden.substr(i, 40) << "'";
    }
    EXPECT_EQ(mine.size(), golden.size());
    ++*topics;
  }
}
}  // namespace

TEST(T03Io, CppAndPythonCsvByteIdentical) {
  REQUIRE_SYNTHETIC_WIRE_FIXTURE();
  int topics = 0;
  checkCsvIdentical(kSynth, &topics);
  EXPECT_GE(topics, 5);
}

TEST(T03Io, CsvDevSetFixture) {
  int n = 0;
  for (const std::string& fx : ts::realFixtures(kRoot)) {
    int topics = 0;
    checkCsvIdentical(fx, &topics);
    EXPECT_GE(topics, 1) << fx;
    ++n;
  }
  if (n == 0) GTEST_SKIP() << "blocked: dev-set fixture (A24: no gnss/test/data/<seq>_<t0>_<D>s from a development bag)";
}

// The golden CSV format contract other readers rely on (INTERFACE_CHANGES WP-e #4), checked from the decoded bytes
// against the rosbags-written file: for every variable-length array, an element the message has owns at least one
// non-empty cell and an element it lacks (a union column) has only empty cells. Element presence can therefore
// only be read from non-empty cells; the contract must matter (rows shorter than the union must occur).
namespace {
struct ContractTally {
  size_t shorter_than_union = 0;   // (row, array) pairs with fewer elements than the union has columns for
  std::vector<std::string> violations;
};

bool columnWithPrefix(const ts::Csv& csv, const std::string& p) {
  const auto it = csv.col.lower_bound(p);
  return it != csv.col.end() && it->first.compare(0, p.size(), p) == 0;
}

// Cells of element k of array `n`: builtin → "n_k" (or "n_k.sec"/"n_k.nsec"), message → every "n_k.*".
std::vector<const std::string*> elementCells(const ts::Csv& csv, const std::vector<std::string>& row, const FieldSpec& f,
                                             const std::string& n, size_t k) {
  std::vector<const std::string*> out;
  const std::string e = n + "_" + std::to_string(k);
  if (f.builtin != BuiltinType::NotBuiltin) {
    for (const std::string& c : {e, e + ".sec", e + ".nsec"}) {
      const auto it = csv.col.find(c);
      if (it != csv.col.end()) out.push_back(&row[it->second]);
    }
  } else {
    for (auto it = csv.col.lower_bound(e + "."); it != csv.col.end() && it->first.compare(0, e.size() + 1, e + ".") == 0; ++it)
      out.push_back(&row[it->second]);
  }
  return out;
}

void checkContract(const MsgSchema& root, const MsgSchema& s, const FieldNode& node, const std::string& prefix, const ts::Csv& csv,
                   const std::vector<std::string>& row, ContractTally* t) {
  for (const FieldSpec& f : s.fields) {
    const FieldNode* c = node.child(f.name);
    const std::string n = prefix + f.name;
    const MsgSchema* sub = f.builtin == BuiltinType::NotBuiltin ? ts::nestedOf(root, f.resolved_type) : nullptr;
    if (!c || (f.builtin == BuiltinType::NotBuiltin && !sub)) {
      t->violations.push_back(n + ": not in the decoded tree / schema");
      continue;
    }
    if (!f.is_array) {
      if (sub) checkContract(root, *sub, *c, n + ".", csv, row, t);
      continue;
    }
    size_t width = 0;
    while (f.builtin != BuiltinType::NotBuiltin ? !elementCells(csv, row, f, n, width).empty()
                                                : columnWithPrefix(csv, n + "_" + std::to_string(width) + "."))
      ++width;
    const size_t len = sub ? c->children.size() : c->array.size();
    if (len > width) t->violations.push_back(n + ": " + std::to_string(len) + " elements, " + std::to_string(width) + " in the header");
    if (len < width) ++t->shorter_than_union;
    for (size_t k = 0; k < width; ++k) {
      bool any = false;
      for (const std::string* cell : elementCells(csv, row, f, n, k)) any = any || !cell->empty();
      if (k < len && !any) t->violations.push_back(n + "_" + std::to_string(k) + ": present element without a non-empty cell");
      if (k >= len && any) t->violations.push_back(n + "_" + std::to_string(k) + ": absent element with a non-empty cell");
      if (k < len && sub) checkContract(root, *sub, c->children[k], n + "_" + std::to_string(k) + ".", csv, row, t);
    }
  }
}
}  // namespace

TEST(T03Io, GoldenCsvArrayContract) {
  if (ts::listFixtures(kRoot).empty()) GTEST_SKIP() << "optional raw wire fixtures are not distributed in the research preview";
  int fixtures = 0;
  for (const std::string& fx : ts::listFixtures(kRoot)) {
    size_t shorter_meas = 0;
    for (const auto& tc : ts::topicCsv()) {
      if (!ts::isFile(fx + "/msgdef/" + tc.first + ".msgdef")) continue;
      SCOPED_TRACE(fx + " " + tc.first);
      ts::TopicSchema sch;
      std::string why, text;
      ASSERT_TRUE(ts::loadTopic(fx, tc.first, &sch, &why)) << why;
      std::vector<ts::Record> recs;
      ASSERT_TRUE(ts::readRecords(fx + "/bytes/" + tc.first + ".bin", &recs, &why)) << why;
      ASSERT_TRUE(ts::readFile(fx + "/golden/" + tc.second + ".csv", &text));
      ts::Csv csv;
      ASSERT_TRUE(ts::parseCsv(text, &csv, &why)) << why;
      ASSERT_EQ(csv.rows.size(), recs.size());
      const Ros1Decoder dec(sch.schema);
      ContractTally t;
      for (size_t r = 0; r < recs.size(); ++r) {
        FieldNode node;
        size_t used = 0;
        ASSERT_TRUE(dec.decode(recs[r].data.data(), recs[r].data.size(), &node, &used, &why)) << why;
        checkContract(sch.schema, sch.schema, node, "", csv, csv.rows[r], &t);
      }
      for (size_t i = 0; i < t.violations.size() && i < 5; ++i) ADD_FAILURE() << t.violations[i];
      EXPECT_TRUE(t.violations.empty()) << t.violations.size() << " violations";
      if (tc.first == "range_meas") shorter_meas = t.shorter_than_union;
    }
    EXPECT_GT(shorter_meas, 0u) << fx << ": no range_meas row is shorter than the union — the contract would go untested";
    ++fixtures;
  }
  EXPECT_GE(fixtures, 1);
}

// ================================================================================================= T4
namespace {
std::string brdcPath() { return kSynth + "/brdc/BRDC00IGS_R_20250030000_01D_MN.rnx"; }
std::string obsPath() { return kSynth + "/rinex/Synthetic.obs"; }
std::string navPath() { return kSynth + "/rinex/Synthetic.nav"; }
}  // namespace

// T4: RinexSource reads OBS/NAV via readrnxt and produces the same ObsEpoch layout (primary signal first)
TEST(T04Io, RinexSourceReadsObsAndNav) {
  REQUIRE_SYNTHETIC_WIRE_FIXTURE();
  const std::vector<ObsEpoch> bag = bagEpochs(kSynth, nullptr);
  ASSERT_EQ(bag.size(), 99u);
  RinexSource src;
  std::string why;
  ASSERT_TRUE(src.open(obsPath(), brdcPath(), GTime(), GTime(), &why)) << why;
  ASSERT_EQ(src.epochsRemaining(), bag.size());
  int n_sats = 0, n_sigs = 0, missing_high = 0;
  for (size_t k = 0; k < bag.size(); ++k) {
    SCOPED_TRACE("epoch " + std::to_string(k));
    ObsEpoch ep;
    ASSERT_TRUE(src.nextEpoch(&ep));
    EXPECT_EQ(ep.seq, static_cast<uint32_t>(k + 1));
    compareRinexEpoch(bag[k], ep, false, &n_sats, &n_sigs, &missing_high);
  }
  EXPECT_FALSE(src.nextEpoch(nullptr));
  EXPECT_EQ(src.epochsRemaining(), 0u);
  EXPECT_GT(n_sats, 99 * 20);
  EXPECT_GT(n_sigs, n_sats);

  // NAV (IGS BRDC crop): iono header and records against an independent reading of the text
  double ion[8] = {0};
  bool has_ion = false;
  const std::vector<NavRec> recs = readNavText(brdcPath(), ion, &has_ion);
  ASSERT_TRUE(has_ion);
  Klobuchar k;
  ASSERT_TRUE(src.iono(&k));
  for (int i = 0; i < 4; ++i) {
    EXPECT_EQ(k.alpha[i], ion[i]);
    EXPECT_EQ(k.beta[i], ion[4 + i]);
  }
  EXPECT_EQ(k.source, Source::Rinex);
  const std::vector<KeplerEph>& eph = src.ephemerides();
  const std::vector<GloEph>& geph = src.gloEphemerides();
  std::map<char, int> per_sys;
  for (const KeplerEph& e : eph) ++per_sys[sysChar(e.sat.sys)];
  EXPECT_GT(per_sys['G'], 0);
  EXPECT_GT(per_sys['E'], 0);
  EXPECT_GT(per_sys['C'], 0);
  EXPECT_GT(geph.size(), 0u);
  std::map<char, int> checked;
  for (const NavRec& r : recs) {
    if (checked[r.sys] >= 3) continue;
    const auto& d = r.data;
    if (r.sys == 'G' || r.sys == 'E' || (r.sys == 'C' && r.prn <= 50)) {
      const Sys sys = r.sys == 'G' ? Sys::GPS : r.sys == 'E' ? Sys::GAL : Sys::BDS;
      GTime toc = calendarTime(r.ep[0], r.ep[1], r.ep[2], r.ep[3], r.ep[4], r.sec);
      GTime toe;
      if (sys == Sys::BDS) {   // BDT in the file; GPST inside (+14 s)
        toc.sec += 14;
        toe.sec = kBdt0 + 14 + static_cast<int64_t>(d[21]) * 604800 + static_cast<int64_t>(d[11]);
      } else {
        toe.sec = kGpst0 + static_cast<int64_t>(d[21]) * 604800 + static_cast<int64_t>(d[11]);
      }
      toe.frac = d[11] - std::floor(d[11]);
      const auto it = std::find_if(eph.begin(), eph.end(), [&](const KeplerEph& e) {
        return e.sat.sys == sys && e.sat.prn == r.prn && e.toe.sec == toe.sec && e.iode == static_cast<int>(d[3]) &&
               (sys != Sys::GAL || e.code == static_cast<int>(d[20]));
      });
      ASSERT_NE(it, eph.end()) << r.sys << r.prn;
      SCOPED_TRACE(std::string(1, r.sys) + std::to_string(r.prn));
      EXPECT_TRUE(sameTime(it->toc, toc));
      EXPECT_EQ(it->A, d[10] * d[10]);
      EXPECT_EQ(it->e, d[8]);
      EXPECT_EQ(it->M0, d[6]);
      EXPECT_EQ(it->OMGd, d[18]);
      EXPECT_EQ(it->f0, d[0]);
      EXPECT_EQ(it->toes, d[11]);
      EXPECT_EQ(it->week, static_cast<int>(d[21]));
      EXPECT_EQ(it->tgd[0], d[25]);
      EXPECT_EQ(it->svh, static_cast<int>(d[24]));
      EXPECT_EQ(it->sva, sys == Sys::GAL ? sisaIndex(d[23]) : uraIndex(d[23]));
      EXPECT_EQ(it->ura_raw, static_cast<double>(it->sva));   // index semantics for RTKLIB-read entries
      if (sys == Sys::BDS) {
        EXPECT_EQ(it->iodc, static_cast<int>(d[28]));
      }
      EXPECT_EQ(it->source, Source::Rinex);
      ++checked[r.sys];
    } else if (r.sys == 'R') {
      GTime toe_utc = calendarTime(r.ep[0], r.ep[1], r.ep[2], r.ep[3], r.ep[4], r.sec);
      const int64_t tod = toe_utc.sec % 86400;
      toe_utc.sec += ((tod + 450) / 900) * 900 - tod;   // RTKLIB rounds toc to 15 min
      const int64_t toe_gpst = toe_utc.sec + 18;
      const auto it = std::find_if(geph.begin(), geph.end(), [&](const GloEph& g) { return g.sat.prn == r.prn && g.toe.sec == toe_gpst; });
      ASSERT_NE(it, geph.end()) << "R" << r.prn;
      EXPECT_EQ(it->taun, -d[0]);
      EXPECT_EQ(it->gamn, d[1]);
      EXPECT_EQ(it->pos.x(), d[3] * 1e3);
      EXPECT_EQ(it->vel.y(), d[8] * 1e3);
      EXPECT_EQ(it->acc.z(), d[13] * 1e3);
      EXPECT_EQ(it->frq, static_cast<int>(d[10]));
      EXPECT_EQ(it->svh, static_cast<int>(d[6]));
      EXPECT_EQ(it->age, static_cast<int>(d[14]));
      ++checked['R'];
    }
  }
  EXPECT_EQ(checked['G'], 3);
  EXPECT_EQ(checked['E'], 3);
  EXPECT_EQ(checked['C'], 3);
  EXPECT_EQ(checked['R'], 3);
}

// Exporter NAV (D19.12) read back by readrnxt vs the bag-path record, within what the text can hold: each value's
// half-ulp at 13 significant digits plus two binary↔decimal conversions (d19Tol); A goes through √A in the file.
namespace {
void checkKeplerRoundTrip(const KeplerEph& b, const KeplerEph& r) {
  EXPECT_LE(std::fabs(r.toc.diff(b.toc)), 1e-6);
  EXPECT_LE(std::fabs(r.ttr.diff(b.ttr)), 1e-3);
  EXPECT_EQ(r.week, b.week);
  EXPECT_NEAR(r.toes, b.toes, 1e-6);
  EXPECT_EQ(r.iode, b.iode);
  if (b.sat.sys != Sys::GAL) {
    EXPECT_EQ(r.iodc, b.iodc);
  }
  EXPECT_EQ(r.svh, b.svh);
  if (b.sat.sys != Sys::BDS) {
    EXPECT_EQ(r.code, b.code);
  }
  EXPECT_EQ(r.sva, b.sat.sys == Sys::GAL ? sisaIndex(b.ura_raw) : uraIndex(b.ura_raw));   // ura was written in metres
  const double sqrta = std::sqrt(b.A);
  EXPECT_LE(std::fabs(r.A - b.A), 2.0 * sqrta * d19Tol(sqrta) + 4.0 * DBL_EPSILON * b.A) << "A";
  const std::vector<std::tuple<const char*, double, double>> f = {
      {"e", r.e, b.e},         {"i0", r.i0, b.i0},       {"OMG0", r.OMG0, b.OMG0}, {"omg", r.omg, b.omg},       {"M0", r.M0, b.M0},
      {"deln", r.deln, b.deln}, {"OMGd", r.OMGd, b.OMGd}, {"idot", r.idot, b.idot}, {"f0", r.f0, b.f0},         {"f1", r.f1, b.f1},
      {"f2", r.f2, b.f2},      {"tgd0", r.tgd[0], b.tgd[0]}, {"tgd1", r.tgd[1], b.tgd[1]}, {"crc", r.crc, b.crc}, {"crs", r.crs, b.crs},
      {"cuc", r.cuc, b.cuc},   {"cus", r.cus, b.cus},    {"cic", r.cic, b.cic},    {"cis", r.cis, b.cis}};
  for (const auto& x : f) EXPECT_LE(std::fabs(std::get<1>(x) - std::get<2>(x)), d19Tol(std::get<2>(x))) << std::get<0>(x);
}

// GLONASS: pos/vel/acc are written in km (value / 1e3) and read back × 1e3, so the text's half-ulp scales by 1e3.
void checkGloRoundTrip(const GloEph& b, const GloEph& r) {
  EXPECT_EQ(r.frq, b.frq);
  EXPECT_EQ(r.iode, b.iode);
  EXPECT_EQ(r.svh, b.svh);
  EXPECT_EQ(r.age, b.age);
  EXPECT_LE(std::fabs(r.tof.diff(b.tof)), 1e-3);
  EXPECT_LE(std::fabs(r.taun - b.taun), d19Tol(b.taun)) << "taun";
  EXPECT_LE(std::fabs(r.gamn - b.gamn), d19Tol(b.gamn)) << "gamn";
  for (int i = 0; i < 3; ++i) {
    auto tol = [](double v) { return 1e3 * d19Tol(v / 1e3) + 4.0 * DBL_EPSILON * std::fabs(v); };
    EXPECT_LE(std::fabs(r.pos[i] - b.pos[i]), tol(b.pos[i])) << "pos " << i;
    EXPECT_LE(std::fabs(r.vel[i] - b.vel[i]), tol(b.vel[i])) << "vel " << i;
    EXPECT_LE(std::fabs(r.acc[i] - b.acc[i]), tol(b.acc[i])) << "acc " << i;
  }
}

const KeplerEph* findKepler(const std::vector<KeplerEph>& v, const KeplerEph& b) {
  const auto it = std::find_if(v.begin(), v.end(), [&](const KeplerEph& r) {
    return r.sat.sys == b.sat.sys && r.sat.prn == b.sat.prn && std::fabs(r.toe.diff(b.toe)) < 1e-6;
  });
  return it == v.end() ? nullptr : &*it;
}

std::string findWithExt(const std::string& dir, const std::string& ext) {
  std::vector<std::string> found;
  if (DIR* d = ::opendir(dir.c_str())) {
    while (dirent* e = ::readdir(d)) {
      const std::string n = e->d_name;
      if (n.size() > ext.size() && n.compare(n.size() - ext.size(), ext.size(), ext) == 0) found.push_back(dir + "/" + n);
    }
    ::closedir(d);
  }
  std::sort(found.begin(), found.end());
  return found.empty() ? std::string() : found.front();
}
}  // namespace

// T4: the exporter's bag2rinex NAV read back through readrnxt equals the bag-path ephemerides (RINEX precision).
// C51–C63 are excluded here and covered, gated on P1, by RinexSourceBdsC51toC63.
TEST(T04Io, ExporterNavRoundTripsBagEphemerides) {
  REQUIRE_SYNTHETIC_WIRE_FIXTURE();
  GnssCommRos1Decoder dec(testSatNo);
  CaptureSink sink;
  std::vector<ts::Record> recs;
  std::vector<bool> ok;
  std::string why;
  ASSERT_TRUE(decodeTopic(kSynth, "ephem", &dec, &sink, &recs, &ok, &why)) << why;
  ASSERT_TRUE(decodeTopic(kSynth, "glo_ephem", &dec, &sink, &recs, &ok, &why)) << why;
  RinexSource src;
  ASSERT_TRUE(src.open("", navPath(), GTime(), GTime(), &why)) << why;
  EXPECT_EQ(src.epochsRemaining(), 0u);
  int n_eph = 0;
  for (const KeplerEph& b : sink.eph) {
    if (bdsHigh(b.sat)) continue;
    SCOPED_TRACE(std::string(sysName(b.sat.sys)) + std::to_string(b.sat.prn));
    const KeplerEph* r = findKepler(src.ephemerides(), b);
    if (!r) {
      ADD_FAILURE() << "no RINEX entry";
      continue;
    }
    checkKeplerRoundTrip(b, *r);
    ++n_eph;
  }
  EXPECT_GE(n_eph, 20);
  int n_glo = 0;
  for (const GloEph& b : sink.geph) {
    const auto it = std::find_if(src.gloEphemerides().begin(), src.gloEphemerides().end(),
                                 [&](const GloEph& r) { return r.sat.prn == b.sat.prn && std::fabs(r.toe.diff(b.toe)) < 1e-6; });
    ASSERT_NE(it, src.gloEphemerides().end()) << "R" << b.sat.prn;
    SCOPED_TRACE("R" + std::to_string(b.sat.prn));
    checkGloRoundTrip(b, *it);
    ++n_glo;
  }
  EXPECT_GE(n_glo, 5);
}

// T4: BDS PRN 51–63 (C59–C62 GEO) through the RINEX path — OBS and NAV — need WP-a's P1 (MAXPRNCMP 63). Gated on
// the dependency (p1State), not on what the test observes: once P1 is in the build, a C51–C63 record lost on
// either path is a failure.
TEST(T04Io, RinexSourceBdsC51toC63) {
  REQUIRE_SYNTHETIC_WIRE_FIXTURE();
  switch (p1State()) {
    case P1State::NotMerged: GTEST_SKIP() << "blocked: WP-a P1/rtk_adapter not merged (livo_gnss::satNoOf is the skeleton stub)";
    case P1State::Absent: GTEST_SKIP() << "unpatched reference build (LIVO_RTKLIB_APPLY_PATCHES=OFF, A9-4): C51-C63 unmappable by design";
    case P1State::Present: break;
  }
  const std::vector<ObsEpoch> bag = bagEpochs(kSynth, nullptr);
  RinexSource src;
  std::string why;
  ASSERT_TRUE(src.open(obsPath(), brdcPath(), GTime(), GTime(), &why)) << why;
  int n_sats = 0, n_sigs = 0, missing = 0;
  for (size_t k = 0; k < bag.size(); ++k) {
    ObsEpoch ep;
    ASSERT_TRUE(src.nextEpoch(&ep));
    compareRinexEpoch(bag[k], ep, true, &n_sats, &n_sigs, &missing);
  }
  EXPECT_EQ(missing, 0);
  EXPECT_GT(n_sats, 0);
  // NAV: every C51–C63 bag ephemeris written by the exporter comes back
  GnssCommRos1Decoder dec(testSatNo);
  CaptureSink sink;
  std::vector<ts::Record> recs;
  std::vector<bool> ok;
  ASSERT_TRUE(decodeTopic(kSynth, "ephem", &dec, &sink, &recs, &ok, &why)) << why;
  RinexSource nav;
  ASSERT_TRUE(nav.open("", navPath(), GTime(), GTime(), &why)) << why;
  int n_nav = 0;
  for (const KeplerEph& b : sink.eph) {
    if (!bdsHigh(b.sat)) continue;
    SCOPED_TRACE("C" + std::to_string(b.sat.prn));
    const KeplerEph* r = findKepler(nav.ephemerides(), b);
    ASSERT_NE(r, nullptr) << "no RINEX entry";
    checkKeplerRoundTrip(b, *r);
    ++n_nav;
  }
  EXPECT_GT(n_nav, 0);
}

// T4: observation window and error paths
TEST(T04Io, RinexSourceWindowAndErrors) {
  REQUIRE_SYNTHETIC_WIRE_FIXTURE();
  const std::vector<ObsEpoch> bag = bagEpochs(kSynth, nullptr);
  ASSERT_GT(bag.size(), 30u);
  RinexSource src;
  std::string why;
  ASSERT_TRUE(src.open(obsPath(), "", bag[10].t_rx, bag[20].t_rx, &why)) << why;
  EXPECT_EQ(src.epochsRemaining(), 11u);   // RTKLIB screent: both ends inclusive within DTTOL
  ObsEpoch ep;
  ASSERT_TRUE(src.nextEpoch(&ep));
  EXPECT_LE(std::fabs(ep.t_rx.diff(bag[10].t_rx)), kRinexEpochTol);
  EXPECT_FALSE(src.iono(nullptr));          // no NAV given
  EXPECT_TRUE(src.ephemerides().empty());
  RinexSource bad;
  EXPECT_FALSE(bad.open("", "", GTime(), GTime(), &why));
  EXPECT_FALSE(why.empty());
  EXPECT_FALSE(bad.open(kSynth + "/rinex/does_not_exist.obs", "", GTime(), GTime(), &why));
  EXPECT_NE(why.find("cannot open"), std::string::npos) << why;
  EXPECT_FALSE(bad.open(kSynth + "/golden/obs.csv", "", GTime(), GTime(), &why));   // not RINEX
  EXPECT_EQ(bad.epochsRemaining(), 0u);
  EXPECT_FALSE(bad.nextEpoch(&ep));
}

// T4 on real receiver output: the development fixture's bag2rinex OBS read back through readrnxt carries the same
// signals as the bag path (RINEX rules of expectAfterRinex), in the same layout, values within F14.3 and epochs
// within F11.7 — a real rcvTow is not on a 0.1 s grid. GLONASS channels come from the fixture's broadcast records.
TEST(T04Io, RinexObsDevSetFixture) {
  int n_fx = 0, all_sats = 0, all_sigs = 0;
  for (const std::string& fx : ts::realFixtures(kRoot)) {
    const std::string obs = findWithExt(fx + "/rinex", ".obs");
    if (obs.empty()) continue;
    SCOPED_TRACE(fx);
    const std::vector<ObsEpoch> bag = bagEpochs(fx, nullptr);
    RinexSource src;
    std::string why;
    ASSERT_TRUE(src.open(obs, ts::fixtureBrdc(fx), GTime(), GTime(), &why)) << why;
    ASSERT_EQ(src.epochsRemaining(), bag.size());
    int n_sats = 0, n_sigs = 0, missing_high = 0, n_high = 0, n_sigs_high = 0;
    std::vector<ObsEpoch> rnx(bag.size());
    for (size_t k = 0; k < bag.size(); ++k) {
      SCOPED_TRACE("epoch " + std::to_string(k));
      ASSERT_TRUE(src.nextEpoch(&rnx[k]));
      compareRinexEpoch(bag[k], rnx[k], false, &n_sats, &n_sigs, &missing_high);
    }
    EXPECT_GT(n_sats, 0);
    EXPECT_GE(n_sigs, n_sats);   // a single-frequency bag (Occlusion03: GPS L1 + GLONASS G1) has one signal per satellite
    all_sats += n_sats;
    all_sigs += n_sigs;
    if (p1State() == P1State::Present)
      for (size_t k = 0; k < bag.size(); ++k) compareRinexEpoch(bag[k], rnx[k], true, &n_high, &n_sigs_high, &missing_high);
    EXPECT_EQ(missing_high, 0);
    ++n_fx;
  }
  if (n_fx == 0) GTEST_SKIP() << "blocked: dev-set fixture with rinex/ (A24; exporter --rinex)";
  EXPECT_GT(all_sigs, all_sats) << "no development fixture carries a second signal: the multi-signal RINEX layout went untested";
}

// ================================================================================================= T5
// T5: UbxStream decodes synthetic RXM-RAWX/SFRBX frames through input_ubx; checksum errors are counted
TEST(T05Io, UbxStreamDecodesRawx) {
  const uint16_t week = 2347;
  const double tow = 453600.1;
  // Measurements in a deliberately scrambled order (secondary before primary for GPS 5).
  const std::vector<RawxMeas> meas = {
      {20512345.678, 107791234.567, -1234.5f, 0, 5, 3, 0, 5000, 44, 5, 3, 2, 0x07},    // GPS 5 L2CL
      {20512344.321, 107791234.125, -1580.25f, 0, 5, 0, 0, 5000, 48, 4, 1, 2, 0x07},   // GPS 5 L1 C/A
      {21000001.5, 110400000.25, 820.5f, 0, 9, 0, 0, 5000, 40, 6, 2, 3, 0x03},        // GPS 9: half cycle not resolved
      {19100000.25, 101800000.5, 2500.0f, 6, 3, 0, 3, 0, 42, 5, 2, 3, 0x0F},          // GLO slot 3, k = −4, lock 0 → slip, sub-half
      {23500000.5, 123400000.75, -300.0f, 2, 11, 1, 0, 6000, 45, 5, 1, 2, 0x07},      // GAL E11 E1B
      {23500001.5, 94800000.5, -229.5f, 2, 11, 5, 0, 6000, 41, 6, 2, 3, 0x07},        // GAL E11 E5bI
      {37000000.5, 192600000.25, 10.5f, 3, 3, 1, 0, 7000, 39, 6, 2, 2, 0x07},         // BDS C03 GEO B1I D2 (+0.5 cycle)
      {37000001.5, 148900000.5, 8.5f, 3, 3, 3, 0, 7000, 37, 6, 3, 3, 0x07},           // BDS C03 GEO B2I D2
      {22300000.25, 116100000.5, -650.25f, 3, 25, 0, 0, 7000, 46, 5, 2, 2, 0x01},     // BDS C25 MEO B1I: carrier invalid
  };
  const std::vector<uint8_t> rawx = ubxFrame(0x02, 0x15, rawxPayload(tow, week, meas));
  CaptureSink sink;
  UbxStream ubx;
  std::string why;
  std::vector<uint8_t> stream = {0x00, 0xB5, 0x13, 0x62};   // leading garbage incl. a false sync byte
  stream.insert(stream.end(), rawx.begin(), rawx.end());
  EXPECT_TRUE(ubx.decode(Source::UbxStream, stream.data(), stream.size(), 12.5, sink, &why)) << why;
  ASSERT_EQ(sink.obs.size(), 1u);
  EXPECT_EQ(ubx.framesDecoded(), 1u);
  EXPECT_EQ(ubx.checksumErrors(), 0u);
  const ObsEpoch& ep = sink.obs[0];
  EXPECT_EQ(ep.source, Source::UbxStream);
  EXPECT_EQ(ep.t_record, 12.5);
  EXPECT_EQ(ep.seq, 1u);
  EXPECT_TRUE(sameTime(ep.t_rx, gpst(week, tow)));
  ASSERT_EQ(ep.sats.size(), 6u);   // order of first appearance: G05, G09, R03, E11, C03, C25
  struct ExpSig {
    Sys sys;
    int prn;
    SignalCode code;
    double freq, psr, cp, dopp, cn0;
    uint8_t lli, status;
    double pr_std, cp_std;
  };
  auto f = [](double x) { return static_cast<double>(static_cast<float>(x)); };
  const std::vector<std::vector<ExpSig>> exp = {
      {{Sys::GPS, 5, SignalCode::L1C, 1575.42e6, 20512344.321, 107791234.125, -1580.25, 48, 0, 0x07, f(0.01 * 16), f(1 * 0.004)},
       {Sys::GPS, 5, SignalCode::L2X, 1227.60e6, 20512345.678, 107791234.567, -1234.5, 44, 0, 0x07, f(0.01 * 32), f(3 * 0.004)}},
      {{Sys::GPS, 9, SignalCode::L1C, 1575.42e6, 21000001.5, 110400000.25, 820.5, 40, 2, 0x03, f(0.01 * 64), f(2 * 0.004)}},
      {{Sys::GLO, 3, SignalCode::L1C, 1602e6 - 4 * 0.5625e6, 19100000.25, 101800000.5, 2500.0, 42, 1, 0x0F, f(0.01 * 32), f(2 * 0.004)}},
      {{Sys::GAL, 11, SignalCode::L1X, 1575.42e6, 23500000.5, 123400000.75, -300.0, 45, 0, 0x07, f(0.01 * 32), f(1 * 0.004)},
       {Sys::GAL, 11, SignalCode::L7X, 1207.14e6, 23500001.5, 94800000.5, -229.5, 41, 0, 0x07, f(0.01 * 64), f(2 * 0.004)}},
      {{Sys::BDS, 3, SignalCode::L2I, 1561.098e6, 37000000.5, 192600000.75, 10.5, 39, 0, 0x07, f(0.01 * 64), f(2 * 0.004)},
       {Sys::BDS, 3, SignalCode::L7I, 1207.14e6, 37000001.5, 148900001.0, 8.5, 37, 0, 0x07, f(0.01 * 64), f(3 * 0.004)}},
  };
  for (size_t i = 0; i < exp.size(); ++i) {
    const SatObs& so = ep.sats[i];
    SCOPED_TRACE(std::string(sysName(exp[i][0].sys)) + std::to_string(exp[i][0].prn));
    EXPECT_EQ(so.sat.sys, exp[i][0].sys);
    EXPECT_EQ(so.sat.prn, exp[i][0].prn);
    EXPECT_EQ(so.sat.rtk_sat, testSatNo(exp[i][0].sys, exp[i][0].prn));
    ASSERT_EQ(so.sig.size(), exp[i].size());
    for (size_t q = 0; q < exp[i].size(); ++q) {
      const SignalObs& s = so.sig[q];
      const ExpSig& e = exp[i][q];
      EXPECT_EQ(s.code, e.code) << q;
      EXPECT_EQ(s.freq_hz, e.freq) << q;
      EXPECT_EQ(s.psr_m, e.psr) << q;
      EXPECT_EQ(s.cp_cyc, e.cp) << q;
      EXPECT_EQ(s.dopp_hz, e.dopp) << q;
      EXPECT_EQ(s.cn0_dbhz, e.cn0) << q;
      EXPECT_EQ(s.lli, e.lli) << q;
      EXPECT_EQ(s.status, e.status) << q;
      EXPECT_EQ(s.psr_std_raw, e.pr_std) << q;
      EXPECT_EQ(s.cp_std_raw, e.cp_std) << q;
    }
  }
  const SatObs& c25 = ep.sats[5];   // carrier invalid (trkStat bit1 clear): RTKLIB zeroes L, the signal stays
  EXPECT_EQ(c25.sat.sys, Sys::BDS);
  EXPECT_EQ(c25.sat.prn, 25);
  ASSERT_EQ(c25.sig.size(), 1u);
  EXPECT_EQ(c25.sig[0].cp_cyc, 0.0);
  EXPECT_EQ(c25.sig[0].psr_m, 22300000.25);
  EXPECT_EQ(c25.sig[0].status, 0x01);
  EXPECT_EQ(c25.sig[0].lli, 0);
}

TEST(T05Io, UbxStreamBdsMeoInvalidCarrierAndChunking) {
  const std::vector<RawxMeas> meas = {
      {22300000.25, 116100000.5, -650.25f, 3, 25, 0, 0, 7000, 46, 5, 2, 2, 0x01},   // BDS C25 MEO B1I: carrier invalid
      {20512344.321, 107791234.125, -1580.25f, 0, 5, 0, 0, 5000, 48, 4, 1, 2, 0x07}};
  std::vector<uint8_t> a = ubxFrame(0x02, 0x15, rawxPayload(453600.2, 2347, meas));
  std::vector<uint8_t> b = ubxFrame(0x02, 0x15, rawxPayload(453600.3, 2347, meas));
  std::vector<uint8_t> c = ubxFrame(0x02, 0x15, rawxPayload(453600.4, 2347, meas));
  std::vector<uint8_t> bad = b;
  bad[20] ^= 0x5A;   // payload corruption → checksum error, frame dropped
  std::vector<uint8_t> stream;
  for (const auto* fr : {&a, &bad, &c}) {
    stream.insert(stream.end(), fr->begin(), fr->end());
    stream.push_back(0xB5);   // stray sync bytes between frames
  }
  // one call
  CaptureSink s1;
  UbxStream u1;
  std::string why;
  EXPECT_FALSE(u1.decode(Source::UbxStream, stream.data(), stream.size(), 1.0, s1, &why));   // the chunk held a bad frame
  EXPECT_NE(why.find("checksum"), std::string::npos) << why;
  EXPECT_EQ(u1.checksumErrors(), 1u);
  EXPECT_EQ(u1.framesDecoded(), 2u);
  ASSERT_EQ(s1.obs.size(), 2u);
  EXPECT_TRUE(sameTime(s1.obs[0].t_rx, gpst(2347, 453600.2)));
  EXPECT_TRUE(sameTime(s1.obs[1].t_rx, gpst(2347, 453600.4)));   // resynchronised after the corrupted frame
  const SatObs& c25 = s1.obs[0].sats[0];
  EXPECT_EQ(c25.sat.sys, Sys::BDS);
  EXPECT_EQ(c25.sat.prn, 25);
  ASSERT_EQ(c25.sig.size(), 1u);
  EXPECT_EQ(c25.sig[0].cp_cyc, 0.0);          // RTKLIB zeroes the phase when trkStat bit1 is clear
  EXPECT_EQ(c25.sig[0].status, 0x01);
  EXPECT_EQ(c25.sig[0].psr_m, 22300000.25);   // MEO: no GEO half-cycle shift
  // byte-by-byte and random chunks give the same result
  std::mt19937 rng(7);
  for (int mode = 0; mode < 2; ++mode) {
    CaptureSink s2;
    UbxStream u2;
    size_t p = 0;
    while (p < stream.size()) {
      const size_t n = mode == 0 ? 1 : std::min<size_t>(stream.size() - p, 1 + rng() % 97);
      u2.decode(Source::UbxStream, stream.data() + p, n, 1.0, s2, &why);
      p += n;
    }
    EXPECT_EQ(u2.checksumErrors(), 1u) << mode;
    EXPECT_EQ(u2.framesDecoded(), 2u) << mode;
    ASSERT_EQ(s2.obs.size(), 2u) << mode;
    EXPECT_EQ(s2.obs[1].sats[1].sig[0].psr_m, 20512344.321);
  }
  // reset() forgets everything, counters included
  u1.reset();
  EXPECT_EQ(u1.checksumErrors(), 0u);
  EXPECT_EQ(u1.framesDecoded(), 0u);
  // registerMessageDefinition is a no-op for a byte stream
  EXPECT_TRUE(u1.registerMessageDefinition(Source::UbxStream, "", "", "", &why));
}

TEST(T05Io, UbxStreamSfrbxEphemerisIonoAndNavPvt) {
  // A GPS LNAV ephemeris quantised to its broadcast LSBs (values of a real G04 record, 2025-01-03).
  const double SC2RAD = 3.1415926535898;   // IS-GPS-200 semicircle
  const int week = 2347, prn = 4, iode = 40, iodc = 808, sva = 2, svh = 0, code = 1, flag = 0;
  const uint32_t tow1 = 453594;                // HOW TOW (seconds; multiple of 6)
  const int64_t toc_i = 453600 / 16, toes_i = 453600 / 16;
  const int64_t f0_i = 67910, f1_i = 53, f2_i = 0, tgd_i = -10;
  const int64_t crs_i = -4146, deln_i = 12054, m0_i = -332684211, cuc_i = -3679, e_i = 27808264, cus_i = 4821;
  const uint64_t sqrta_i = 2702022416ull;
  const int64_t cic_i = 59, omg0_i = 182941513, cis_i = 7, i0_i = 660612421, crc_i = 6707, w_i = -2027143512, omgd_i = -23056,
                idot_i = -703;
  Bits s1, s2, s3, s4;
  lnavHeader(&s1, 1, tow1 / 6);
  s1.setu(48, 10, week % 1024);
  s1.setu(58, 2, code);
  s1.setu(60, 4, sva);
  s1.setu(64, 6, svh);
  s1.setu(70, 2, iodc >> 8);
  s1.setu(72, 1, flag);
  s1.sets(160, 8, tgd_i);
  s1.setu(168, 8, iodc & 0xFF);
  s1.setu(176, 16, toc_i);
  s1.sets(192, 8, f2_i);
  s1.sets(200, 16, f1_i);
  s1.sets(216, 22, f0_i);
  lnavHeader(&s2, 2, tow1 / 6 + 1);
  s2.setu(48, 8, iode);
  s2.sets(56, 16, crs_i);
  s2.sets(72, 16, deln_i);
  s2.sets(88, 32, m0_i);
  s2.sets(120, 16, cuc_i);
  s2.setu(136, 32, static_cast<uint64_t>(e_i));
  s2.sets(168, 16, cus_i);
  s2.setu(184, 32, sqrta_i);
  s2.setu(216, 16, toes_i);
  s2.setu(232, 1, 0);
  lnavHeader(&s3, 3, tow1 / 6 + 2);
  s3.sets(48, 16, cic_i);
  s3.sets(64, 32, omg0_i);
  s3.sets(96, 16, cis_i);
  s3.sets(112, 32, i0_i);
  s3.sets(144, 16, crc_i);
  s3.sets(160, 32, w_i);
  s3.sets(192, 24, omgd_i);
  s3.setu(216, 8, iode);
  s3.sets(224, 14, idot_i);
  // subframe 4 page 18: Klobuchar + UTC
  const int a_i[4] = {15, 12, -8, 1}, b_i[4] = {74, -96, -1, 5};
  lnavHeader(&s4, 4, tow1 / 6 + 3);
  s4.setu(48, 2, 1);
  s4.setu(50, 6, 56);
  for (int k = 0; k < 4; ++k) {
    s4.sets(56 + 8 * k, 8, a_i[k]);
    s4.sets(88 + 8 * k, 8, b_i[k]);
  }
  s4.sets(120, 24, 5);
  s4.sets(144, 32, -3);
  s4.setu(176, 8, 144);
  s4.setu(184, 8, week % 256);
  s4.sets(192, 8, 18);
  s4.setu(200, 8, 137);
  s4.setu(208, 8, 7);
  s4.sets(216, 8, 18);

  std::vector<uint8_t> stream;
  for (const Bits* s : {&s1, &s2, &s3, &s4}) {
    const auto fr = ubxFrame(0x02, 0x13, sfrbxGps(prn, *s));
    stream.insert(stream.end(), fr.begin(), fr.end());
  }
  CaptureSink sink;
  UbxStream ubx;
  std::string why;
  EXPECT_TRUE(ubx.decode(Source::UbxStream, stream.data(), stream.size(), 3.0, sink, &why)) << why;
  EXPECT_EQ(ubx.framesDecoded(), 4u);
  ASSERT_EQ(sink.eph.size(), 1u);
  const KeplerEph& e = sink.eph[0];
  EXPECT_EQ(e.sat.sys, Sys::GPS);
  EXPECT_EQ(e.sat.prn, prn);
  EXPECT_EQ(e.iode, iode);
  EXPECT_EQ(e.iodc, iodc);
  EXPECT_EQ(e.sva, sva);
  EXPECT_EQ(e.svh, svh);
  EXPECT_EQ(e.code, code);
  EXPECT_EQ(e.week, week);   // 10-bit week resolved by RTKLIB with the host clock (valid until ~2034)
  EXPECT_EQ(e.toes, 453600.0);
  EXPECT_TRUE(sameTime(e.toe, gpst(week, 453600.0)));
  EXPECT_TRUE(sameTime(e.toc, gpst(week, 453600.0)));
  EXPECT_TRUE(sameTime(e.ttr, gpst(week, tow1)));
  EXPECT_EQ(e.fit, 4.0);
  EXPECT_EQ(e.source, Source::UbxStream);
  EXPECT_EQ(e.t_record, 3.0);
  const double rel = 1e-14;
  expectRel(e.f0, std::ldexp(static_cast<double>(f0_i), -31), rel, "af0");
  expectRel(e.f1, std::ldexp(static_cast<double>(f1_i), -43), rel, "af1");
  EXPECT_EQ(e.f2, 0.0);
  expectRel(e.tgd[0], std::ldexp(static_cast<double>(tgd_i), -31), rel, "tgd");
  expectRel(e.crs, std::ldexp(static_cast<double>(crs_i), -5), rel, "crs");
  expectRel(e.deln, std::ldexp(static_cast<double>(deln_i), -43) * SC2RAD, rel, "deln");
  expectRel(e.M0, std::ldexp(static_cast<double>(m0_i), -31) * SC2RAD, rel, "M0");
  expectRel(e.cuc, std::ldexp(static_cast<double>(cuc_i), -29), rel, "cuc");
  expectRel(e.e, std::ldexp(static_cast<double>(e_i), -33), rel, "e");
  expectRel(e.cus, std::ldexp(static_cast<double>(cus_i), -29), rel, "cus");
  const double sqrta = std::ldexp(static_cast<double>(sqrta_i), -19);
  expectRel(e.A, sqrta * sqrta, rel, "A");
  expectRel(e.cic, std::ldexp(static_cast<double>(cic_i), -29), rel, "cic");
  expectRel(e.OMG0, std::ldexp(static_cast<double>(omg0_i), -31) * SC2RAD, rel, "OMG0");
  expectRel(e.cis, std::ldexp(static_cast<double>(cis_i), -29), rel, "cis");
  expectRel(e.i0, std::ldexp(static_cast<double>(i0_i), -31) * SC2RAD, rel, "i0");
  expectRel(e.crc, std::ldexp(static_cast<double>(crc_i), -5), rel, "crc");
  expectRel(e.omg, std::ldexp(static_cast<double>(w_i), -31) * SC2RAD, rel, "omg");
  expectRel(e.OMGd, std::ldexp(static_cast<double>(omgd_i), -43) * SC2RAD, rel, "OMGd");
  expectRel(e.idot, std::ldexp(static_cast<double>(idot_i), -43) * SC2RAD, rel, "idot");
  ASSERT_EQ(sink.ion.size(), 1u);
  const Klobuchar& k = sink.ion[0];
  const int ascale[4] = {-30, -27, -24, -24}, bscale[4] = {11, 14, 16, 16};
  for (int i = 0; i < 4; ++i) {
    expectRel(k.alpha[i], std::ldexp(static_cast<double>(a_i[i]), ascale[i]), rel, "alpha");
    expectRel(k.beta[i], std::ldexp(static_cast<double>(b_i[i]), bscale[i]), rel, "beta");
  }
  EXPECT_EQ(k.source, Source::UbxStream);
  // the same subframes again: RTKLIB suppresses an unchanged ephemeris (no second push)
  EXPECT_TRUE(ubx.decode(Source::UbxStream, stream.data(), stream.size(), 4.0, sink, &why)) << why;
  EXPECT_EQ(sink.eph.size(), 1u);

  // NAV-PVT: iTOW is exact, the week comes from the UTC date (or else the last RXM-RAWX epoch) within half a week.
  // The date fields are whole seconds, so around the week rollover they can sit on the other side of it.
  struct PvtCase {
    const char* what;
    int y, mo, d, hh, mi, ss;
    uint8_t valid;       // NAV-PVT valid flags (0x07: validDate | validTime | fullyResolved)
    uint32_t itow_ms;
    double rawx_tow;     // < 0: no RXM-RAWX before the NAV-PVT, else a week-2347 RAWX epoch at this tow first
    int gps_week;        // expected week; -1: refused (week unknown)
  };
  const std::vector<PvtCase> cases = {
      {"date and iTOW in the same week", 2025, 1, 3, 5, 59, 42, 0x07, 453600000u + 100u, -1.0, 2347},   // GPST Fri 06:00:00.1
      {"UTC Saturday, GPST already Sunday", 2025, 1, 4, 23, 59, 52, 0x07, 10000u, -1.0, 2348},         // GPST Sun 00:00:10
      {"end of week, date rounded into the next one", 2025, 1, 4, 23, 59, 42, 0x07, 604799900u, -1.0, 2347},   // date → 2348/0
      {"start of week, date still in the last one", 2025, 1, 4, 23, 59, 41, 0x07, 500u, -1.0, 2348},        // date → 2347/604799
      {"no valid date: week of the last RAWX", 0, 0, 0, 0, 0, 0, 0x00, 453601000u, 453600.1, 2347},
      {"no valid date, RAWX before the rollover", 0, 0, 0, 0, 0, 0, 0x00, 200u, 604799.5, 2348},
      {"no valid date and no RAWX: refused", 0, 0, 0, 0, 0, 0, 0x00, 453601000u, -1.0, -1}};
  for (const PvtCase& c : cases) {
    SCOPED_TRACE(c.what);
    std::vector<uint8_t> p(92, 0);
    put(&p, 0, c.itow_ms);
    put(&p, 4, static_cast<uint16_t>(c.y));
    p[6] = static_cast<uint8_t>(c.mo);
    p[7] = static_cast<uint8_t>(c.d);
    p[8] = static_cast<uint8_t>(c.hh);
    p[9] = static_cast<uint8_t>(c.mi);
    p[10] = static_cast<uint8_t>(c.ss);
    p[11] = c.valid;
    p[20] = 3;                                    // 3D fix
    p[21] = static_cast<uint8_t>(0x01 | 0x02 | (2 << 6));   // gnssFixOK, diffSoln, carrSoln fixed
    p[23] = 17;
    put(&p, 24, static_cast<int32_t>(1214360000));   // lon 121.436 deg
    put(&p, 28, static_cast<int32_t>(310252000));    // lat 31.0252 deg
    put(&p, 32, static_cast<int32_t>(18123));        // height 18.123 m
    put(&p, 36, static_cast<int32_t>(27600));
    put(&p, 40, static_cast<uint32_t>(1200));
    put(&p, 44, static_cast<uint32_t>(2100));
    put(&p, 48, static_cast<int32_t>(-12));
    put(&p, 52, static_cast<int32_t>(34));
    put(&p, 56, static_cast<int32_t>(-5));
    put(&p, 68, static_cast<uint32_t>(80));
    put(&p, 76, static_cast<uint16_t>(130));
    CaptureSink ps;
    UbxStream u;
    if (c.rawx_tow >= 0.0) {
      const auto rx = ubxFrame(0x02, 0x15, rawxPayload(c.rawx_tow, 2347, {{20512344.321, 107791234.125, -1580.25f, 0, 5, 0, 0, 5000, 48, 4, 1, 2, 0x07}}));
      ASSERT_TRUE(u.decode(Source::UbxStream, rx.data(), rx.size(), 8.0, ps, &why)) << why;
      ASSERT_EQ(ps.obs.size(), 1u);
    }
    const auto fr = ubxFrame(0x01, 0x07, p);
    if (c.gps_week < 0) {
      EXPECT_FALSE(u.decode(Source::UbxStream, fr.data(), fr.size(), 9.0, ps, &why));
      EXPECT_NE(why.find("week unknown"), std::string::npos) << why;
      EXPECT_TRUE(ps.pvt.empty());
      continue;
    }
    EXPECT_TRUE(u.decode(Source::UbxStream, fr.data(), fr.size(), 9.0, ps, &why)) << why;
    ASSERT_EQ(ps.pvt.size(), 1u);
    const ReceiverPvt& v = ps.pvt[0];
    EXPECT_TRUE(sameTime(v.t, gpst(c.gps_week, c.itow_ms * 1e-3))) << v.t.sec << " + " << v.t.frac;
    EXPECT_EQ(v.fix_type, 3);
    EXPECT_TRUE(v.valid_fix);
    EXPECT_TRUE(v.diff_soln);
    EXPECT_EQ(v.carr_soln, 2);
    EXPECT_EQ(v.num_sv, 17);
    EXPECT_DOUBLE_EQ(v.lon_deg, 121.436);
    EXPECT_DOUBLE_EQ(v.lat_deg, 31.0252);
    EXPECT_DOUBLE_EQ(v.h_ell_m, 18.123);
    EXPECT_DOUBLE_EQ(v.h_msl_m, 27.6);
    EXPECT_DOUBLE_EQ(v.h_acc_m, 1.2);
    EXPECT_DOUBLE_EQ(v.v_acc_m, 2.1);
    EXPECT_DOUBLE_EQ(v.vel_ned.x(), -0.012);
    EXPECT_DOUBLE_EQ(v.vel_ned.y(), 0.034);
    EXPECT_DOUBLE_EQ(v.vel_ned.z(), -0.005);
    EXPECT_DOUBLE_EQ(v.vel_acc_mps, 0.08);
    EXPECT_DOUBLE_EQ(v.p_dop, 1.3);
    EXPECT_EQ(v.source, Source::UbxStream);
    EXPECT_EQ(v.t_record, 9.0);
  }
  // a NAV-PVT with a short payload is refused (decode returns false, nothing pushed)
  const auto short_pvt = ubxFrame(0x01, 0x07, std::vector<uint8_t>(40, 0));
  CaptureSink ps;
  UbxStream u;
  EXPECT_FALSE(u.decode(Source::UbxStream, short_pvt.data(), short_pvt.size(), 0.0, ps, &why));
  EXPECT_TRUE(ps.pvt.empty());
}
