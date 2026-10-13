// SPDX-License-Identifier: BSD-3-Clause
// test_wpf_serialisation.cpp — WP-f serialisation and configuration checks (gnss/src/diagnostics.cpp,
// gnss/src/config.cpp). Registered in LIVO_GNSS_TESTS since P2b (AMENDMENTS A40; `--core` ctest = 19 executables).
// Why: 68af88c fixed a header/row column misalignment in CovRecord::csvHeader — the header and the row are
// built by one walk now, and this file pins that property so it cannot silently regress. A40 added the two
// validateConfig time-map rules; they are checked against TimeMap itself (both directions), not only against
// hand-picked strings, because the point of the rule is that a validated config never throws later.
#include <gtest/gtest.h>

#include <cmath>
#include <cstdlib>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "internal/time_map.hpp"
#include "livo_gnss/config.hpp"
#include "livo_gnss/diagnostics.hpp"

using namespace livo_gnss;

namespace {

std::vector<std::string> splitCsv(const std::string& s) {
  std::vector<std::string> out;
  std::string cur;
  for (char c : s) {
    if (c == ',') { out.push_back(cur); cur.clear(); } else cur += c;
  }
  out.push_back(cur);
  return out;
}

int indexOf(const std::vector<std::string>& v, const std::string& name) {
  for (size_t i = 0; i < v.size(); ++i)
    if (v[i] == name) return static_cast<int>(i);
  return -1;
}

// validateConfig messages that mention `needle` (the key a rule is about).
int countMentions(const std::vector<std::string>& problems, const std::string& needle) {
  int n = 0;
  for (const auto& p : problems) n += (p.find(needle) != std::string::npos);
  return n;
}

// What TimeMap's constructor (WP-a, time_map.cpp) does with a Config::Time: it rejects configuration errors with
// std::invalid_argument; a skeleton stub throws std::logic_error("not implemented: ...") (→ blocked); anything
// else is a failure of the cross-check.
enum class TimeMapOutcome { Accepted, Rejected, Stub, Other };
TimeMapOutcome constructTimeMap(const Config::Time& t, std::string* what) {
  try {
    const TimeMap m(t);
    (void)m;
    return TimeMapOutcome::Accepted;
  } catch (const std::invalid_argument& e) {   // before logic_error: invalid_argument derives from it
    *what = e.what();
    return TimeMapOutcome::Rejected;
  } catch (const std::logic_error& e) {
    *what = e.what();
    return std::string(e.what()).rfind("not implemented", 0) == 0 ? TimeMapOutcome::Stub : TimeMapOutcome::Other;
  } catch (const std::exception& e) {
    *what = e.what();
    return TimeMapOutcome::Other;
  }
}

// Leap strings of the A40 rule: kLeapAccepted must validate, kLeapRefused must be refused.
const std::vector<std::string> kLeapAccepted = {"auto", "18", "0", "9", "+18", "99", "00"};
const std::vector<std::string> kLeapRefused = {"",   "Auto", "AUTO", "auto ", "18.0", "18.5", "1e1", " 18",  "18 ", "-1",
                                               "-0", "100",  "018",  "0x12",  "+",    "-",    "++1", "eighteen", "18s", "nan",
                                               "inf", "99999999999999999999999999999999999999999999999999"};

}  // namespace

// Header and row of every CSV record have the same number of columns (the 68af88c regression).
TEST(WpfSerialisation, CovHeaderAndRowHaveSameColumnCount) {
  CovRecord r;
  r.t_k = 0.1;
  r.stage = "postLIO";
  r.effct_feat_num = 123;
  const auto h = splitCsv(CovRecord::csvHeader());
  EXPECT_EQ(h.size(), splitCsv(r.csvRow()).size());
  // A variable-length augmented block must stay one column ('|'-separated), never shift the others.
  r.aug_sigma = Eigen::VectorXd::Constant(3, 0.5);
  EXPECT_EQ(h.size(), splitCsv(r.csvRow()).size());
  const auto row = splitCsv(r.csvRow());
  EXPECT_EQ(row[indexOf(h, "aug_sigma")], "0.5|0.5|0.5");
}

TEST(WpfSerialisation, EpochAndSatHeaderAndRowHaveSameColumnCount) {
  EpochRecord e;
  EXPECT_EQ(splitCsv(EpochRecord::csvHeader()).size(), splitCsv(e.csvRow()).size());
  SatRecord s;
  EXPECT_EQ(splitCsv(SatRecord::csvHeader()).size(), splitCsv(s.csvRow()).size());
}

// Named columns land where the header says they do, and %.17g round-trips the double exactly.
TEST(WpfSerialisation, CovColumnsMatchHeaderNamesAndRoundTrip) {
  CovRecord r;
  r.t_k = 1732537008.1234567;
  r.stage = "prior";
  r.effct_feat_num = -1;
  r.host_sigma[4] = 0.1;
  r.cond_prior = 3.0e7;
  const auto h = splitCsv(CovRecord::csvHeader());
  const auto row = splitCsv(r.csvRow());
  ASSERT_EQ(h.size(), row.size());
  EXPECT_EQ(row[indexOf(h, "stage")], "prior");
  EXPECT_EQ(row[indexOf(h, "effct_feat_num")], "-1");
  EXPECT_EQ(std::strtod(row[indexOf(h, "t_k")].c_str(), nullptr), r.t_k);
  EXPECT_EQ(std::strtod(row[indexOf(h, "host_sigma_4")].c_str(), nullptr), 0.1);
  EXPECT_EQ(std::strtod(row[indexOf(h, "cond_prior")].c_str(), nullptr), 3.0e7);
  // Eigenvector columns are written column-major: pos_eigvec_<col>_<row>.
  EXPECT_EQ(std::strtod(row[indexOf(h, "pos_eigvec_2_1")].c_str(), nullptr), 1.0);   // identity default
  EXPECT_EQ(std::strtod(row[indexOf(h, "pos_eigvec_2_0")].c_str(), nullptr), 0.0);
}

// JSON summary: NaN/Inf become null (JSON has neither), and the object parses as key/value lines.
TEST(WpfSerialisation, SummaryJsonHasNoNan) {
  Summary s;
  s.tdcp_coverage = std::nan("");
  s.elapsed_p50_ms = INFINITY;
  const std::string j = s.toJson();
  EXPECT_EQ(j.find("nan"), std::string::npos);
  EXPECT_EQ(j.find("inf"), std::string::npos);
  EXPECT_NE(j.find("\"tdcp_coverage\": null"), std::string::npos);
  EXPECT_NE(j.find("\"elapsed_p50_ms\": null"), std::string::npos);
}

// canonicalDump is a one-to-one function of the Config: equal configs → equal bytes, one field change → a
// change, and user strings cannot forge a line or a list element.
TEST(WpfSerialisation, CanonicalDumpIsInjectiveAndEscaped) {
  Config a, b;
  EXPECT_EQ(canonicalDump(a), canonicalDump(b));
  b.masks.el_deg += 1.0;
  EXPECT_NE(canonicalDump(a), canonicalDump(b));

  Config c;
  c.time.calib_file = "x\ny=1";   // a newline inside a string must not create a second line
  const std::string d0 = canonicalDump(a), d1 = canonicalDump(c);
  const auto lines = [](const std::string& s) { size_t n = 0; for (char ch : s) n += (ch == '\n'); return n; };
  EXPECT_EQ(lines(d0), lines(d1));
  EXPECT_NE(d1.find("\"x\\ny=1\""), std::string::npos);

  Config e1, e2;
  e1.nav_rinex = {"a,b"};
  e2.nav_rinex = {"a", "b"};
  EXPECT_NE(canonicalDump(e1), canonicalDump(e2));
}

// validateConfig: the default (enable=false) config is accepted untouched (zero-channel gate needs it), and
// the A16 rule fires only when the core would actually run.
TEST(WpfSerialisation, ValidateConfigDefaultsAndUraRule) {
  Config c;
  EXPECT_TRUE(validateConfig(c).empty());
  c.enable = true;   // source = gnss_comm_ros1, ura_semantics = unknown → refused
  const auto problems = validateConfig(c);
  bool ura = false;
  for (const auto& p : problems) ura = ura || (p.find("ura_semantics") != std::string::npos);
  EXPECT_TRUE(ura);
  c.ephem.ura_semantics = Config::Ephem::UraSemantics::Index;
  const auto after = validateConfig(c);
  for (const auto& p : after) EXPECT_EQ(p.find("ura_semantics"), std::string::npos) << p;
}

// A40: drift_ppm != 0 needs its reference instant t_ref_gpst_s (GTime::toSec() scale, A18). The default (0, 0)
// stays valid — the zero-channel gate runs the untouched config — and a drift with a real M3DGR-scale reference is
// accepted; drift with t_ref = 0 (the §14 default) or before the GPS epoch is refused with the key named.
TEST(WpfSerialisation, ValidateConfigRejectsDriftWithoutReference) {
  const std::string key = "gnss.time.t_ref_gpst_s";
  Config c;
  EXPECT_TRUE(validateConfig(c).empty());   // drift 0, t_ref 0
  c.time.drift_ppm = 1.5;
  auto p = validateConfig(c);
  ASSERT_EQ(p.size(), 1u) << (p.empty() ? std::string() : p[0]);
  EXPECT_EQ(countMentions(p, key), 1) << p[0];
  EXPECT_NE(p[0].find("got 0"), std::string::npos) << p[0];
  c.time.t_ref_gpst_s = 1733040083.0;   // Occlusion03 start, GPST week 2343 tow 28883 (A18 example)
  EXPECT_TRUE(validateConfig(c).empty());
  c.time.drift_ppm = -3.0;
  EXPECT_TRUE(validateConfig(c).empty());
  c.time.t_ref_gpst_s = 315964800.0 - 1.0;   // one second before the GPS epoch on the toSec() scale
  EXPECT_EQ(countMentions(validateConfig(c), key), 1);
  c.time.t_ref_gpst_s = 315964800.0;
  EXPECT_TRUE(validateConfig(c).empty());
  // Without drift the reference is unused: any non-negative value (including 0) is fine.
  c.time.drift_ppm = 0.0;
  c.time.t_ref_gpst_s = 0.0;
  EXPECT_TRUE(validateConfig(c).empty());
  c.time.t_ref_gpst_s = 12.0;
  EXPECT_TRUE(validateConfig(c).empty());
  // The rule holds with the core disabled too (TimeMap would throw on it whenever the core runs).
  c.enable = false;
  c.time.drift_ppm = 0.5;
  c.time.t_ref_gpst_s = 0.0;
  EXPECT_EQ(countMentions(validateConfig(c), key), 1);
}

// A40: gnss.time.leap is "auto" or an integer string; everything else is refused with the key named.
TEST(WpfSerialisation, ValidateConfigLeapStrings) {
  for (const auto& s : kLeapAccepted) {
    Config c;
    c.time.leap = s;
    EXPECT_TRUE(validateConfig(c).empty()) << "leap '" << s << "' should validate";
  }
  for (const auto& s : kLeapRefused) {
    Config c;
    c.time.leap = s;
    const auto p = validateConfig(c);
    ASSERT_EQ(p.size(), 1u) << "leap '" << s << "' should be the only problem";
    EXPECT_EQ(countMentions(p, "gnss.time.leap"), 1) << p[0];
  }
}

// The point of the A40 rules: a config that validates never makes TimeMap throw, and whatever TimeMap refuses is
// refused by validateConfig first (validateConfig may be stricter, e.g. "18.0" or "-0"). Checked on every leap
// string above and on the drift/reference pairs.
TEST(WpfSerialisation, ValidateConfigAgreesWithTimeMap) {
  std::vector<Config::Time> cases;
  for (const auto* list : {&kLeapAccepted, &kLeapRefused})
    for (const auto& s : *list) {
      Config::Time t;
      t.leap = s;
      cases.push_back(t);
    }
  for (const auto& dr : std::vector<std::pair<double, double>>{
           {0.0, 0.0}, {1.5, 0.0}, {-2.0, 0.0}, {1.5, 1733040083.0}, {-3.0, 1737451258.0}, {0.5, 315964799.0}, {0.0, 12.0}}) {
    Config::Time t;
    t.drift_ppm = dr.first;
    t.t_ref_gpst_s = dr.second;
    cases.push_back(t);
  }
  int accepted_both = 0, refused_both = 0;
  for (const auto& t : cases) {
    Config c;
    c.time = t;
    const bool valid = validateConfig(c).empty();
    std::string what;
    const TimeMapOutcome tm = constructTimeMap(t, &what);
    if (tm == TimeMapOutcome::Stub) GTEST_SKIP() << "blocked: TimeMap is a skeleton stub (" << what << ")";
    ASSERT_NE(tm, TimeMapOutcome::Other) << "TimeMap threw an unexpected exception for leap '" << t.leap << "': " << what;
    if (valid) {
      EXPECT_EQ(tm, TimeMapOutcome::Accepted) << "validated config rejected by TimeMap (leap '" << t.leap << "', drift "
                                              << t.drift_ppm << ", t_ref " << t.t_ref_gpst_s << "): " << what;
    }
    if (tm == TimeMapOutcome::Rejected) {
      EXPECT_FALSE(valid) << "TimeMap refuses (" << what << ") but validateConfig accepts leap '" << t.leap << "', drift "
                          << t.drift_ppm << ", t_ref " << t.t_ref_gpst_s;
    }
    accepted_both += (valid && tm == TimeMapOutcome::Accepted);
    refused_both += (!valid && tm == TimeMapOutcome::Rejected);
  }
  // Both outcomes must actually occur, otherwise the implications above are vacuous.
  EXPECT_GE(accepted_both, 8);
  EXPECT_GE(refused_both, 8);
}
