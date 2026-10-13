// SPDX-License-Identifier: BSD-3-Clause
// test_u06_paths.cpp — TEST_PLAN §1 U6 (two tiers, AMENDMENTS A9-3). Owner: WP-a.
// Needs the dev-set fixture (A24; found by gnss_io/test/fixture_discovery.hpp, A39) and WP-e's GnssCommRos1Decoder /
// RinexSource (gate order OWNERSHIP §1b): until both exist the tests skip with "blocked: ...". Path equivalence is a
// real-data criterion (A24): the gate tests run on the dev-set slices only; WP-e's synthetic slices run in the
// *SyntheticSlice companions (PASS(合成)). Both paths take the GPS Klobuchar from the slice's BRDC header GPSA/GPSB, as
// the core does since A38 (bag iono_params are diagnostics only).
//
// Fixture layout (A24; WP-e's gnss/test/data/README.md): <data>/<seq>_<t0>_<D>s/{msgdef/<topic>.msgdef|.md5,
// bytes/<topic>.bin, golden/*.csv, brdc/*.rnx}; bytes = [uint64 t_record_ns][uint32 len][len bytes]…; golden CSV =
// rosbags export, first column t_record_ns, message fields flattened in definition order (arrays with a _i suffix,
// nested fields as outer.inner, floats %.17g, time as .sec/.nsec); the column set is the union over all rows and a
// shorter array leaves its cells empty, so an element exists only where one of its cells is filled.
#include <gtest/gtest.h>

#include <dirent.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "../../gnss_io/test/fixture_discovery.hpp"   // A24/A39: the fixture discovery rule shared with WP-e
#include "../../third_party/rtklib/testing/wpa_test_support.hpp"   // LIVO_BLOCKED_IF_STUB, slice macros
#include "internal/ephem_store.hpp"
#include "internal/rtk_adapter.hpp"
#include "livo_gnss/gnss_fusion.hpp"
#include "livo_gnss_io/gnss_comm_mapping.hpp"
#include "livo_gnss_io/rinex_source.hpp"
#include "livo_gnss_io/ros1_decoder.hpp"
#include "livo_gnss_io/ros1_msgdef.hpp"
#include "rtklib.h"   // WP-a test files may include it (OWNERSHIP §1): RTKLIB's RINEX NAV writer

using namespace livo_gnss;
namespace io = livo_gnss_io;

namespace {

using livo_wpa_testing::findFile;
using livo_wpa_testing::readText;
using livo_wpa_testing::trimmed;

struct Collected : MeasurementSink {
  std::vector<ObsEpoch> obs;
  std::vector<KeplerEph> eph;
  std::vector<GloEph> geph;
  std::vector<Klobuchar> iono;
  void pushObservation(ObsEpoch&& e) override { obs.push_back(std::move(e)); }
  void pushEphemeris(const KeplerEph& e) override { eph.push_back(e); }
  void pushGloEphemeris(const GloEph& g) override { geph.push_back(g); }
  void pushIono(const Klobuchar& k) override { iono.push_back(k); }
  void pushPvt(const ReceiverPvt&) override {}
};

// Path A: fixture bytes → C++ decoder (WP-e); a stub throws std::logic_error. Messages the decoder refuses are counted
// (DecodeStats) — path B must then refuse the same rows.
bool decodeBytes(const std::string& slice, Collected* out, livo_wpa_testing::DecodeStats* st, std::string* why) {
  io::GnssCommRos1Decoder dec(&satNoOf);
  namespace t = livo_wpa_testing;
  return t::decodeChannel(dec, slice, t::kMeasChannel, *out, st, why) && t::decodeChannel(dec, slice, t::kEphemChannel, *out, st, why, false) &&
         t::decodeChannel(dec, slice, t::kGloEphemChannel, *out, st, why, false) && t::decodeChannel(dec, slice, t::kIonoChannel, *out, st, why, false);
}

// ---- Path B: golden CSV (%.17g) → field tree → the same gnss_comm mapping (WP-e) ----
struct Csv {
  std::vector<std::string> header;
  std::vector<std::vector<std::string>> rows;
};
std::vector<std::string> splitCsv(const std::string& line) {
  std::vector<std::string> v;
  std::string cur;
  bool q = false;
  for (char ch : line) {
    if (ch == '"') q = !q;
    else if (ch == ',' && !q) {
      v.push_back(cur);
      cur.clear();
    } else if (ch != '\r') cur += ch;
  }
  v.push_back(cur);
  return v;
}
bool readCsv(const std::string& path, Csv* out) {
  std::ifstream f(path);
  if (!f) return false;
  std::string line;
  if (!std::getline(f, line)) return false;
  out->header = splitCsv(line);
  while (std::getline(f, line))
    if (!line.empty()) out->rows.push_back(splitCsv(line));
  return true;
}

bool parseValue(io::BuiltinType t, const std::string& s, io::Value* v) {
  v->type = t;
  if (s.empty()) return false;
  char* end = nullptr;
  switch (t) {
    case io::BuiltinType::Boolean: v->b = (s == "1" || s == "true" || s == "True"); return true;
    case io::BuiltinType::Int8:
    case io::BuiltinType::Int16:
    case io::BuiltinType::Int32:
    case io::BuiltinType::Int64: v->i = std::strtoll(s.c_str(), &end, 10); return *end == '\0';
    case io::BuiltinType::UInt8:
    case io::BuiltinType::UInt16:
    case io::BuiltinType::UInt32:
    case io::BuiltinType::UInt64: v->u = std::strtoull(s.c_str(), &end, 10); return *end == '\0';
    case io::BuiltinType::Float32:
    case io::BuiltinType::Float64: v->f = std::strtod(s.c_str(), &end); return *end == '\0';
    case io::BuiltinType::String: v->s = s; return true;
    default: return false;
  }
}

class NodeBuilder {
 public:
  NodeBuilder(const Csv& csv, size_t row) {
    for (size_t i = 0; i < csv.header.size() && i < csv.rows[row].size(); ++i) cells_[csv.header[i]] = csv.rows[row][i];
  }
  bool build(const io::MsgSchema& schema, const std::string& prefix, io::FieldNode* out, std::string* why) const {
    out->is_message = true;
    out->type = schema.datatype;
    for (const io::FieldSpec& f : schema.fields) {
      io::FieldNode c;
      c.name = f.name;
      c.type = f.type;
      c.is_array = f.is_array;
      const std::string key = prefix + f.name;
      if (f.builtin != io::BuiltinType::NotBuiltin) {
        if (!f.is_array) {
          if (!scalar(f.builtin, key, &c.scalar)) {
            *why = "CSV column(s) for " + key + " missing or unparsable";
            return false;
          }
        } else {
          for (int i = 0;; ++i) {
            io::Value v;
            if (f.fixed_len >= 0 && i >= f.fixed_len) break;
            if (!scalar(f.builtin, key + "_" + std::to_string(i), &v)) break;
            c.array.push_back(v);
          }
          if (f.fixed_len >= 0 && static_cast<int>(c.array.size()) != f.fixed_len) {
            *why = "fixed array " + key + " incomplete in CSV";
            return false;
          }
        }
      } else {
        auto it = schema.nested.find(f.resolved_type);
        if (it == schema.nested.end() || !it->second) {
          *why = "no schema for " + f.resolved_type;
          return false;
        }
        c.is_message = true;
        if (!f.is_array) {
          if (!build(*it->second, key + ".", &c, why)) return false;
        } else {
          for (int i = 0;; ++i) {
            if (f.fixed_len >= 0 && i >= f.fixed_len) break;
            const std::string p = key + "_" + std::to_string(i) + ".";
            if (!hasPrefix(p)) break;
            io::FieldNode el;
            if (!build(*it->second, p, &el, why)) return false;
            c.children.push_back(el);
          }
        }
      }
      out->children.push_back(c);
    }
    return true;
  }

 private:
  bool cell(const std::string& k, std::string* v) const {
    auto it = cells_.find(k);
    if (it == cells_.end() || it->second.empty()) return false;
    *v = it->second;
    return true;
  }
  bool scalar(io::BuiltinType t, const std::string& key, io::Value* v) const {
    std::string s, ns;
    if (t == io::BuiltinType::Time || t == io::BuiltinType::Duration) {
      v->type = t;
      if (!(cell(key + ".sec", &s) || cell(key + ".secs", &s))) return false;
      if (!(cell(key + ".nanosec", &ns) || cell(key + ".nsecs", &ns) || cell(key + ".nsec", &ns))) return false;
      v->sec = static_cast<uint32_t>(std::strtoul(s.c_str(), nullptr, 10));
      v->nsec = static_cast<uint32_t>(std::strtoul(ns.c_str(), nullptr, 10));
      return true;
    }
    if (t == io::BuiltinType::String) {   // an empty string is an empty cell: the column's presence is what counts
      auto it = cells_.find(key);
      if (it == cells_.end()) return false;
      v->type = t;
      v->s = it->second;
      return true;
    }
    return cell(key, &s) && parseValue(t, s, v);
  }
  // An element of a variable-length nested array exists in this row iff one of its cells is filled: the exporter's
  // column set is the union over all rows (the longest array wins) and a shorter row leaves the rest empty (A24 /
  // gnss/test/data/README.md), so a present column alone says nothing.
  bool hasPrefix(const std::string& p) const {
    for (auto it = cells_.lower_bound(p); it != cells_.end() && it->first.compare(0, p.size(), p) == 0; ++it)
      if (!it->second.empty()) return true;
    return false;
  }
  std::map<std::string, std::string> cells_;
};

// Path B. A row the mapping refuses (WP-e's contract: semantically invalid messages, e.g. iono_params not 8 long) is
// counted in `refused` and skipped — the byte path refuses the same messages; a CSV/schema problem is fatal.
bool decodeGoldenCsv(const std::string& slice, Collected* out, int* refused, std::string* why) {
  struct G {
    Source src;
    const char* datatype;
    const char* token;
    const char* not_token;
    const char* csv;
  };
  const G gs[] = {{Source::GnssCommMeas, "gnss_comm/GnssMeasMsg", "range_meas", "", "obs.csv"},
                  {Source::GnssCommEphem, "gnss_comm/GnssEphemMsg", "ephem", "glo", "ephem.csv"},
                  {Source::GnssCommGloEphem, "gnss_comm/GnssGloEphemMsg", "glo_ephem", "", "glo_ephem.csv"},
                  {Source::GnssCommIono, "gnss_comm/StampedFloat64Array", "iono", "", "iono.csv"}};
  io::MappingStats stats;
  for (const G& g : gs) {
    const std::string def = findFile(slice + "/msgdef", g.token, ".msgdef", g.not_token);
    Csv csv;
    if (def.empty() || !readCsv(slice + "/golden/" + g.csv, &csv)) continue;
    io::MsgSchema schema;
    if (!io::MsgDefParser::parse(g.datatype, readText(def), &schema, why)) return false;
    for (size_t r = 0; r < csv.rows.size(); ++r) {
      io::FieldNode node;
      if (!NodeBuilder(csv, r).build(schema, "", &node, why)) return false;
      const double t_record = std::strtod(csv.rows[r][0].c_str(), nullptr) * 1e-9;
      bool ok = true;
      switch (g.src) {
        // What reaches the sink follows GnssCommRos1Decoder::decode (WP-e): an epoch without any satellite has no time
        // tag and is not pushed; an ephemeris of an unmapped satellite (E37/E38, C51+ before P1) is not pushed.
        case Source::GnssCommMeas: {
          ObsEpoch e;
          ok = io::toObsEpoch(node, t_record, &satNoOf, &e, &stats, why);
          if (ok && e.t_rx.valid()) out->obs.push_back(std::move(e));
          break;
        }
        case Source::GnssCommEphem: {
          KeplerEph e;
          ok = io::toKeplerEph(node, t_record, &satNoOf, &e, &stats, why);
          if (ok && e.sat.valid()) out->eph.push_back(e);
          break;
        }
        case Source::GnssCommGloEphem: {
          GloEph e;
          ok = io::toGloEph(node, t_record, &satNoOf, &e, &stats, why);
          if (ok && e.sat.valid()) out->geph.push_back(e);
          break;
        }
        default: {
          Klobuchar k;
          ok = io::toKlobuchar(node, t_record, &k, why);
          if (ok) out->iono.push_back(k);
          break;
        }
      }
      if (!ok) ++*refused;
    }
  }
  return true;
}

std::string hexf(double v) {
  char b[64];
  std::snprintf(b, sizeof(b), "%a", v);
  return b;
}

// A38: the GPS Klobuchar of a slice = its BRDC header GPSA/GPSB (what the core and pntpos use; bag iono_params are not
// GPS Klobuchar on M3DGR).
bool sliceGpsKlobuchar(const std::string& slice, Klobuchar* out, std::string* why) {
  const std::string brdc = livo_gnss_io::testsupport::fixtureBrdc(slice);
  if (brdc.empty()) {
    *why = slice + ": A24 brdc/ missing";
    return false;
  }
  rtk::NavStore nav;
  if (nav.loadRinexNav(brdc, true, why) <= 0) return false;
  if (!nav.iono(out)) {
    *why = brdc + ": no GPSA/GPSB in the header";
    return false;
  }
  return true;
}

// pntpos over every epoch with the ephemerides of the same path and the slice's GPS Klobuchar; the result text is
// compared bit for bit.
std::string solveAll(const Collected& c, const Klobuchar& gps_ion, std::vector<rtk::SppRefResult>* results) {
  Config cfg;
  cfg.ephem.ura_semantics = Config::Ephem::UraSemantics::Index;   // the same on both paths; pntpos only excludes on it
  rtk::NavStore nav;
  EphemStore es(cfg, nav);
  for (const KeplerEph& e : c.eph) es.addBag(e);
  for (const GloEph& g : c.geph) es.addBag(g);
  for (const Klobuchar& k : c.iono) es.setIonoFromBag(k);   // recorded only (A38): nav.ion_gps stays empty ...
  nav.setIono(gps_ion);                                     // ... until the BRDC header set is put there
  const rtk::SppRef spp;
  rtk::SppRefOptions opt;
  std::ostringstream o;
  for (const ObsEpoch& e : c.obs) {
    rtk::SppRefResult r;
    spp.solve(e, nav, opt, &r);
    o << e.t_rx.sec << ' ' << hexf(e.t_rx.frac) << ' ' << r.ok << ' ' << r.ns;
    for (int k = 0; k < 3; ++k) o << ' ' << hexf(r.r_ecef[k]) << ' ' << hexf(r.v_ecef[k]);
    for (int k = 0; k < 6; ++k) o << ' ' << hexf(r.dtr_s[k]);
    for (const auto& p : r.resp) o << ' ' << p.first.rtk_sat << ':' << hexf(p.second);
    o << '\n';
    if (results) results->push_back(r);
  }
  return o.str();
}

// RINEX 3 observation code of a §12.3 signal (RTKLIB obscodes).
const char* rinexCode(SignalCode c) {
  switch (c) {
    case SignalCode::L1C: return "1C";
    case SignalCode::L2X: return "2X";
    case SignalCode::L2C: return "2C";
    case SignalCode::L1X: return "1X";
    case SignalCode::L7X: return "7X";
    case SignalCode::L2I: return "2I";
    case SignalCode::L7I: return "7I";
    case SignalCode::L5X: return "5X";
    case SignalCode::L5Q: return "5Q";
    default: return nullptr;
  }
}

// RINEX 3.04 OBS writer for tier 2 (F14.3 as rnx2rtkp/gnss_bag_export.py would write). Every signal goes to the column
// of its own code: a satellite tracked on its secondary signal only (e.g. BDS B2I without B1I — seen on Grass02) must
// reach the RINEX path as that signal, or the two paths apply different ionosphere scalings and group delays and the
// comparison stops being about rounding. GLONASS channels go to the "GLONASS SLOT / FRQ #" header.
bool writeRinexObs(const std::string& path, const std::vector<ObsEpoch>& epochs) {
  std::map<char, std::vector<SignalCode>> codes;   // per system, first-seen order (sig[0] = primary when tracked)
  std::map<int, int> glo_k;                        // GLONASS slot → channel k
  for (const ObsEpoch& e : epochs)
    for (const SatObs& so : e.sats)
      for (const SignalObs& s : so.sig) {
        if (!rinexCode(s.code)) continue;
        std::vector<SignalCode>& v = codes[rtk::satIdString(so.sat.rtk_sat)[0]];
        if (std::find(v.begin(), v.end(), s.code) == v.end()) v.push_back(s.code);
        if (so.sat.sys == Sys::GLO && s.code == SignalCode::L1C && s.freq_hz > 0.0)
          glo_k[so.sat.prn] = static_cast<int>(std::lround((s.freq_hz - rtk::kFreqG1) / rtk::kDFreqG1));
      }
  FILE* fp = std::fopen(path.c_str(), "w");
  if (!fp) return false;
  auto hdr = [&](const std::string& body, const char* label) { std::fprintf(fp, "%-60.60s%-20s\n", body.c_str(), label); };
  hdr("     3.04           OBSERVATION DATA    M", "RINEX VERSION / TYPE");
  hdr("livo U6", "PGM / RUN BY / DATE");
  hdr("", "MARKER NAME");
  for (const auto& kv : codes) {
    std::vector<std::string> types;
    for (SignalCode c : kv.second)
      for (const char* t : {"C", "L", "D", "S"}) types.push_back(std::string(t) + rinexCode(c));
    for (size_t i = 0; i < types.size(); i += 13) {   // 13 types per line, continuation lines indented by 6
      char b[64];
      if (i == 0) std::snprintf(b, sizeof(b), "%c  %3d", kv.first, static_cast<int>(types.size()));
      else std::snprintf(b, sizeof(b), "      ");
      std::string body = b;
      for (size_t j = i; j < types.size() && j < i + 13; ++j) body += " " + types[j];
      hdr(body, "SYS / # / OBS TYPES");
    }
  }
  if (!glo_k.empty()) {
    std::string body;
    char b[32];
    std::snprintf(b, sizeof(b), "%3d", static_cast<int>(glo_k.size()));
    body = b;
    int n = 0;
    for (const auto& kv : glo_k) {
      if (n > 0 && n % 8 == 0) {
        hdr(body, "GLONASS SLOT / FRQ #");
        body = "   ";
      }
      std::snprintf(b, sizeof(b), " R%02d %2d", kv.first, kv.second);
      body += b;
      ++n;
    }
    hdr(body, "GLONASS SLOT / FRQ #");
  }
  if (!epochs.empty()) {
    double e6[6];
    rtk::time2epoch(epochs.front().t_rx, e6);
    char b[64];
    std::snprintf(b, sizeof(b), "  %04.0f    %02.0f    %02.0f    %02.0f    %02.0f   %10.7f     GPS", e6[0], e6[1], e6[2], e6[3], e6[4], e6[5]);
    hdr(b, "TIME OF FIRST OBS");
  }
  hdr("", "END OF HEADER");
  for (const ObsEpoch& e : epochs) {
    std::vector<const SatObs*> sats;
    for (const SatObs& so : e.sats)
      if (codes.count(rtk::satIdString(so.sat.rtk_sat)[0]) && !so.sig.empty()) sats.push_back(&so);
    double e6[6];
    rtk::time2epoch(e.t_rx, e6);
    std::fprintf(fp, "> %04.0f %02.0f %02.0f %02.0f %02.0f %10.7f  0%3d\n", e6[0], e6[1], e6[2], e6[3], e6[4], e6[5], static_cast<int>(sats.size()));
    for (const SatObs* so : sats) {
      std::fprintf(fp, "%3s", rtk::satIdString(so->sat.rtk_sat).c_str());
      for (SignalCode c : codes.at(rtk::satIdString(so->sat.rtk_sat)[0])) {
        const SignalObs* s = nullptr;
        for (const SignalObs& x : so->sig)
          if (x.code == c) s = &x;
        if (!s) {
          std::fprintf(fp, "%64s", "");
          continue;
        }
        if (s->psrValid()) std::fprintf(fp, "%14.3f  ", s->psr_m);
        else std::fprintf(fp, "%16s", "");
        if (s->cpValid()) std::fprintf(fp, "%14.3f%1d ", s->cp_cyc, s->lli & 3);
        else std::fprintf(fp, "%16s", "");
        std::fprintf(fp, "%14.3f  %14.3f  ", s->dopp_hz, s->cn0_dbhz);
      }
      std::fprintf(fp, "\n");
    }
  }
  std::fclose(fp);
  return true;
}

}  // namespace

// Tier 1 (same source, two code paths, bit-identical): fixture bytes through the C++ decoder vs the same
// epochs read back from the rosbags golden CSV (%.17g round-trips doubles) give bit-identical pntpos solutions
namespace {

void tier1Slice(const livo_wpa_testing::Slice& sl) {
  const std::string& slice = sl.path;
  Collected a, b;
  livo_wpa_testing::DecodeStats ds;
  int refused_csv = 0;
  std::string why;
  try {
    ASSERT_TRUE(decodeBytes(slice, &a, &ds, &why)) << slice << ": " << why;
    ASSERT_TRUE(decodeGoldenCsv(slice, &b, &refused_csv, &why)) << slice << ": " << why;
  } catch (const std::logic_error& e) {
    LIVO_BLOCKED_IF_STUB(e, "WP-e merge (gnss_io GnssCommRos1Decoder / RinexSource / gnss_comm mapping)");
  }
  Klobuchar ion;
  ASSERT_TRUE(sliceGpsKlobuchar(slice, &ion, &why)) << why;
  EXPECT_EQ(ds.refused, refused_csv) << slice << ": both paths must refuse the same messages (bytes: " << ds.first_refusal << ")";
  ASSERT_FALSE(a.obs.empty()) << slice;
  ASSERT_EQ(a.obs.size(), b.obs.size()) << slice;
  ASSERT_EQ(a.eph.size(), b.eph.size()) << slice;
  ASSERT_EQ(a.geph.size(), b.geph.size()) << slice;
  ASSERT_EQ(a.iono.size(), b.iono.size()) << slice;
  // Field-level first (clearer failure), then the solutions.
  for (size_t i = 0; i < a.obs.size(); ++i) {
    ASSERT_EQ(a.obs[i].sats.size(), b.obs[i].sats.size()) << i;
    EXPECT_TRUE(a.obs[i].t_rx == b.obs[i].t_rx) << i;
    for (size_t j = 0; j < a.obs[i].sats.size(); ++j) {
      const SatObs &x = a.obs[i].sats[j], &y = b.obs[i].sats[j];
      EXPECT_EQ(x.sat.rtk_sat, y.sat.rtk_sat);
      ASSERT_EQ(x.sig.size(), y.sig.size());
      for (size_t k = 0; k < x.sig.size(); ++k) {
        EXPECT_EQ(std::memcmp(&x.sig[k].psr_m, &y.sig[k].psr_m, sizeof(double)), 0);
        EXPECT_EQ(std::memcmp(&x.sig[k].cp_cyc, &y.sig[k].cp_cyc, sizeof(double)), 0);
        EXPECT_EQ(std::memcmp(&x.sig[k].dopp_hz, &y.sig[k].dopp_hz, sizeof(double)), 0);
        EXPECT_EQ(x.sig[k].code, y.sig[k].code);
        EXPECT_EQ(x.sig[k].status, y.sig[k].status);
      }
    }
  }
  EXPECT_EQ(solveAll(a, ion, nullptr), solveAll(b, ion, nullptr)) << slice;
  std::printf("[U6 tier 1] %s: %zu epochs, %zu+%zu ephemerides, %zu iono_params, %d messages refused by the decoder%s%s\n",
              sl.name.c_str(), a.obs.size(), a.eph.size(), a.geph.size(), a.iono.size(), ds.refused, ds.refused ? ", first: " : "",
              ds.first_refusal.c_str());
}

}  // namespace

TEST(U06Paths, DecoderVsGoldenCsvBitIdenticalPntpos) {
  LIVO_DEVSET_SLICES(slices);
  for (const livo_wpa_testing::Slice& sl : slices) {
    tier1Slice(sl);
    if (HasFatalFailure() || IsSkipped()) return;
  }
}

TEST(U06Paths, DecoderVsGoldenCsvBitIdenticalPntposSyntheticSlice) {
  LIVO_SYNTHETIC_SLICES(slices);
  for (const livo_wpa_testing::Slice& sl : slices) {
    tier1Slice(sl);
    if (HasFatalFailure() || IsSkipped()) return;
  }
  std::printf("[U6 tier 1] synthetic slices: PASS(合成) only (A24)\n");
}

// Tier 2 (bag vs RINEX, F14.3 / D19.12 rounding): pntpos position differs < 1 cm (3D) and receiver clock < 1 cm
// The RINEX NAV of the second path is written from the slice's bag ephemerides, so a slice without ephemeris topics
// (A26: Occlusion03/04, Dynamic03) has no bag-vs-RINEX comparison and contributes nothing, like U3/U4 (A24; P2b
// integration: before this rule the Occlusion03 slice wrote an empty NAV file and failed loadRinexNav).
namespace {

// `had_eph` = the slice carries ephemeris topics (and was compared).
void tier2Slice(const livo_wpa_testing::Slice& sl, bool* had_eph) {
  *had_eph = false;
  const std::string& slice = sl.path;
  if (livo_wpa_testing::findFile(slice + "/bytes", "ephem", ".bin", "glo").empty()) {
    std::printf("[U6 tier 2] %s: no ephem topic in the slice\n", sl.name.c_str());
    return;
  }
  *had_eph = true;
  Collected bag;
  livo_wpa_testing::DecodeStats ds;
  std::string why;
  try {
    ASSERT_TRUE(decodeBytes(slice, &bag, &ds, &why)) << slice << ": " << why;
  } catch (const std::logic_error& e) {
    LIVO_BLOCKED_IF_STUB(e, "WP-e merge (gnss_io GnssCommRos1Decoder / RinexSource / gnss_comm mapping)");
  }
  // bag → RINEX: OBS with F14.3 (this file's writer), NAV with D19.12-class rounding (RTKLIB outrnxnavb).
  const std::string obs_path = ::testing::TempDir() + "/livo_u06.obs", nav_path = ::testing::TempDir() + "/livo_u06.nav";
  ASSERT_TRUE(writeRinexObs(obs_path, bag.obs));
  {
    Config cfg;
    cfg.ephem.ura_semantics = Config::Ephem::UraSemantics::Index;
    rtk::NavStore nav;
    EphemStore es(cfg, nav);
    for (const KeplerEph& e : bag.eph) es.addBag(e);
    for (const GloEph& g : bag.geph) es.addBag(g);
    FILE* fp = std::fopen(nav_path.c_str(), "w");
    ASSERT_NE(fp, nullptr);
    static rnxopt_t opt;
    static nav_t hdr;
    std::memset(&opt, 0, sizeof(opt));
    std::memset(&hdr, 0, sizeof(hdr));
    opt.rnxver = 305;
    opt.navsys = SYS_ALL;
    outrnxnavh(fp, &opt, &hdr);
    for (const rtk::EphKey& k : nav.coverage()) {
      if (k.is_glo) {
        GloEph g;
        nav.get(k.sat, k.toe, &g);
        geph_t o;
        std::memset(&o, 0, sizeof(o));
        o.sat = g.sat.rtk_sat; o.iode = g.iode; o.frq = g.frq; o.svh = g.svh; o.flags = g.flags; o.sva = g.sva; o.age = g.age;
        o.toe.time = static_cast<time_t>(g.toe.sec); o.toe.sec = g.toe.frac; o.tof.time = static_cast<time_t>(g.tof.sec); o.tof.sec = g.tof.frac;
        for (int i = 0; i < 3; ++i) { o.pos[i] = g.pos[i]; o.vel[i] = g.vel[i]; o.acc[i] = g.acc[i]; }
        o.taun = g.taun; o.gamn = g.gamn; o.dtaun = g.dtaun;
        outrnxgnavb(fp, &opt, &o);
      } else {
        KeplerEph x;
        nav.get(k.sat, k.toe, &x, k.nav_type);
        eph_t e;
        std::memset(&e, 0, sizeof(e));
        e.sat = x.sat.rtk_sat; e.iode = x.iode; e.iodc = x.iodc; e.sva = x.sva; e.svh = x.svh; e.week = x.week; e.code = x.code; e.flag = x.flag;
        e.toe.time = static_cast<time_t>(x.toe.sec); e.toe.sec = x.toe.frac; e.toc.time = static_cast<time_t>(x.toc.sec); e.toc.sec = x.toc.frac;
        e.ttr.time = static_cast<time_t>(x.ttr.sec); e.ttr.sec = x.ttr.frac;
        e.A = x.A; e.e = x.e; e.i0 = x.i0; e.OMG0 = x.OMG0; e.omg = x.omg; e.M0 = x.M0; e.deln = x.deln; e.OMGd = x.OMGd; e.idot = x.idot;
        e.crc = x.crc; e.crs = x.crs; e.cuc = x.cuc; e.cus = x.cus; e.cic = x.cic; e.cis = x.cis; e.toes = x.toes; e.fit = x.fit;
        e.f0 = x.f0; e.f1 = x.f1; e.f2 = x.f2;
        for (int i = 0; i < 6; ++i) e.tgd[i] = x.tgd[i];
        outrnxnavb(fp, &opt, &e);
      }
    }
    std::fclose(fp);
  }
  // RINEX path: WP-e's RinexSource for the observations, the core's NavStore for the navigation file.
  Collected rnx;
  try {
    io::RinexSource src;
    GTime t0;
    ASSERT_TRUE(src.open(obs_path, "", t0, t0, &why)) << why;
    ObsEpoch e;
    while (src.nextEpoch(&e)) rnx.obs.push_back(e);
  } catch (const std::logic_error& e) {
    LIVO_BLOCKED_IF_STUB(e, "WP-e merge (gnss_io GnssCommRos1Decoder / RinexSource / gnss_comm mapping)");
  }
  ASSERT_EQ(rnx.obs.size(), bag.obs.size());
  Klobuchar ion;
  ASSERT_TRUE(sliceGpsKlobuchar(slice, &ion, &why)) << why;
  std::vector<rtk::SppRefResult> ra, rb;
  solveAll(bag, ion, &ra);
  rtk::NavStore nav;
  ASSERT_GT(nav.loadRinexNav(nav_path, true, &why), 0) << why;
  nav.setIono(ion);   // the same GPS Klobuchar on both paths (the RINEX NAV text written above carries none)
  const rtk::SppRef spp;
  const rtk::SppRefOptions opt;
  int compared = 0;
  for (size_t i = 0; i < rnx.obs.size(); ++i) {
    rtk::SppRefResult r;
    spp.solve(rnx.obs[i], nav, opt, &r);
    rb.push_back(r);
    if (!ra[i].ok || !r.ok) {
      EXPECT_EQ(ra[i].ok, r.ok) << i;
      continue;
    }
    EXPECT_LT((ra[i].r_ecef - r.r_ecef).norm(), 0.01) << i;
    EXPECT_LT(std::fabs(ra[i].dtr_s[0] - r.dtr_s[0]) * rtk::kCLight, 0.01) << i;
    ++compared;
  }
  EXPECT_GT(compared, 0) << slice;
  std::printf("[U6 tier 2] %s: %zu epochs, %d solutions compared (bag vs RINEX text)\n", sl.name.c_str(), rnx.obs.size(), compared);
}

}  // namespace

// Gate (A24/A39): dev-set slices only, one of them with ephemeris topics (A26: Grass01/02).
TEST(U06Paths, BagVsRinexWithinRinexPrecision) {
  LIVO_DEVSET_SLICES(slices);
  bool any_eph = false;
  for (const livo_wpa_testing::Slice& sl : slices) {
    bool had = false;
    tier2Slice(sl, &had);
    if (HasFatalFailure() || IsSkipped()) return;
    any_eph = any_eph || had;
  }
  if (!any_eph)
    GTEST_SKIP() << "blocked: dev-set fixture with ephemeris topics (A26: only Grass01/02 carry them; dev-set slices:"
                 << livo_wpa_testing::namesOf(slices, livo_wpa_testing::SliceKind::DevSet) << ")";
}

TEST(U06Paths, BagVsRinexWithinRinexPrecisionSyntheticSlice) {
  LIVO_SYNTHETIC_SLICES(slices);
  bool any_eph = false;
  for (const livo_wpa_testing::Slice& sl : slices) {
    bool had = false;
    tier2Slice(sl, &had);
    if (HasFatalFailure() || IsSkipped()) return;
    any_eph = any_eph || had;
  }
  ASSERT_TRUE(any_eph) << "synthetic slices without ephemeris topics";
  std::printf("[U6 tier 2] synthetic slices: PASS(合成) only (A24)\n");
}
