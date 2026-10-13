// SPDX-License-Identifier: BSD-3-Clause
// TimeMap (DESIGN §1, §7; AMENDMENTS A18, A40). Owner: WP-a. Tests: U1.
//
// Conventions (A18, time origin as ruled in A40):
//   * GTime is GPST counted from the Unix epoch (like RTKLIB gtime_t); host/sensor stamps are Unix UTC seconds.
//   * t_from_gpst_s / t_ref_gpst_s in the config and in the calibration file are on the same scale as GTime::toSec():
//     seconds since 1970-01-01 on the GPST scale, no leap seconds (= 315964800 + week·604800 + tow; M3DGR 2024–2025
//     ≈ 1.73e9; A18's example 1733040083 = week 2343, tow 28883). Not the 1980-01-06 origin: a value on that origin is
//     315964800 s early, which with a 3 ppm drift moves the map by ≈ 948 s — checkTimeOrigin() refuses such files
//     when the caller knows the data span.
//   * t_est = t_gpst − leap(t) + offset + drift_ppm·1e-6·(t_gpst − t_ref)   for base = utc | host
//     t_est = t_gpst          + offset + drift_ppm·1e-6·(t_gpst − t_ref)   for base = gpst (no leap second)
#include "internal/time_map.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>

#include "internal/rtk_adapter.hpp"

namespace livo_gnss {

namespace {

constexpr double kSpanMarginS = 86400.0;       // A40: ± 1 day around the data span (timecalib_yaml.py SPAN_MARGIN_S)

// t − v for a GTime t and a GTime::toSec() value v, keeping the fraction out of the large integer part: t.sec and v are
// both ≈ 1.7e9, so their difference is exact (Sterbenz) and only the fraction is rounded once.
inline double secondsSince(GTime t, double v) { return (static_cast<double>(t.sec) - v) + t.frac; }

// GTime of a GTime::toSec() value (whole seconds and fraction split without passing the fraction through 1.7e9).
inline GTime fromToSec(double v) {
  GTime g;
  const double w = std::floor(v);
  g.sec = static_cast<int64_t>(w);
  g.frac = v - w;
  return g;
}

bool parseDouble(const std::string& s, double* out) {
  if (s.empty()) return false;
  errno = 0;
  char* end = nullptr;
  const double v = std::strtod(s.c_str(), &end);
  if (errno != 0 || end == s.c_str() || *end != '\0' || !std::isfinite(v)) return false;
  *out = v;
  return true;
}

bool parseBool(const std::string& s, bool* out) {
  if (s == "true" || s == "True" || s == "TRUE") {
    *out = true;
    return true;
  }
  if (s == "false" || s == "False" || s == "FALSE") {
    *out = false;
    return true;
  }
  return false;
}

std::string trim(const std::string& s) {
  const size_t a = s.find_first_not_of(" \t\r");
  if (a == std::string::npos) return std::string();
  const size_t b = s.find_last_not_of(" \t\r");
  return s.substr(a, b - a + 1);
}

// Removes a trailing comment: '#' at line start or preceded by whitespace (YAML), outside quotes.
std::string stripComment(const std::string& line) {
  bool sq = false, dq = false;
  for (size_t i = 0; i < line.size(); ++i) {
    const char c = line[i];
    if (c == '\'' && !dq) sq = !sq;
    else if (c == '"' && !sq) dq = !dq;
    else if (c == '#' && !sq && !dq && (i == 0 || line[i - 1] == ' ' || line[i - 1] == '\t')) return line.substr(0, i);
  }
  return line;
}

std::string unquote(const std::string& v) {
  if (v.size() >= 2 && ((v.front() == '"' && v.back() == '"') || (v.front() == '\'' && v.back() == '\'')))
    return v.substr(1, v.size() - 2);
  return v;
}

}  // namespace

TimeMap::TimeMap(const Config::Time& cfg) : cfg_(cfg) {
  Segment s;
  s.offset_s = cfg.offset_s;
  s.drift_ppm = cfg.drift_ppm;
  s.t_ref_gpst_s = cfg.t_ref_gpst_s;
  s.sigma_toff_s = cfg.sigma_toff_s;
  segments_.push_back(s);
  if (cfg.leap != "auto") {
    // A fixed leap second count is a test/future-date override; anything unparsable is a configuration error, never
    // a silent fallback to the table.
    double v = 0.0;
    if (!parseDouble(trim(cfg.leap), &v) || v != std::floor(v))
      throw std::invalid_argument("gnss.time.leap must be \"auto\" or an integer, got \"" + cfg.leap + "\"");
    leap_fixed_ = true;
    leap_fixed_s_ = v;
  }
  // t_ref is a GTime::toSec() value: 0 is 1970-01-01, so a drift with the §14 default t_ref would be multiplied by
  // ~1.7e9 s (1 ppm ⇒ 1700 s). Same policy as the leap string: a configuration error, not a silent map.
  if (cfg.drift_ppm != 0.0 && cfg.t_ref_gpst_s == 0.0)
    throw std::invalid_argument("gnss.time.drift_ppm != 0 needs gnss.time.t_ref_gpst_s (GTime::toSec() seconds: 1970 origin, "
                                "GPST scale), got 0");
}

double TimeMap::leapSeconds(GTime gpst) const { return leap_fixed_ ? leap_fixed_s_ : rtk::leapSecondsAt(gpst); }

double TimeMap::toEstimator(GTime t_gpst_true) const {
  const Segment& s = segmentAt(t_gpst_true);
  const double leap = (cfg_.base == Config::Time::Base::Gpst) ? 0.0 : leapSeconds(t_gpst_true);
  // Small terms first, the 1.7e9 s integer part last: a single rounding at the end (double resolution ~2.4e-7 s).
  const double drift = s.drift_ppm * 1e-6 * secondsSince(t_gpst_true, s.t_ref_gpst_s);
  const double small = t_gpst_true.frac + s.offset_s + drift;
  return (static_cast<double>(t_gpst_true.sec) - leap) + small;
}

GTime TimeMap::toGpst(double t_est) const {
  // Affine per segment: t_est = g − L + o + d·(g − r) ⇒ g − r = (u − r)/(1 + d), u = t_est + L − o (r: GTime::toSec()).
  // Leap second and segment depend on g itself: iterate from the leap/segment of the first guess (converges in
  // one pass except within seconds of a leap/segment boundary; three passes are plenty).
  const bool use_leap = cfg_.base != Config::Time::Base::Gpst;
  GTime g = GTime::fromSec(t_est);
  for (int it = 0; it < 3; ++it) {
    const Segment& s = segmentAt(g);
    const double L = use_leap ? leapSeconds(g) : 0.0;
    const double d = s.drift_ppm * 1e-6;
    const GTime u = GTime::fromSec(t_est).add(L - s.offset_s);
    const double du = secondsSince(u, s.t_ref_gpst_s);
    const GTime next = u.add(-d / (1.0 + d) * du);
    if (next == g) break;
    g = next;
  }
  return g;
}

double TimeMap::receiverToEstimator(GTime t_rcv, double clock_bias_m) const {
  return toEstimator(receiverToTrue(t_rcv, clock_bias_m));
}

GTime TimeMap::receiverToTrue(GTime t_rcv, double clock_bias_m) {
  // DESIGN §1: t_gpst,true = t_rcv − b/c (a receiver clock running ahead, b > 0, tags events late).
  return rtk::timeadd(t_rcv, -clock_bias_m / rtk::kCLight);
}

bool TimeMap::loadCalibFile(const std::string& path, std::string* why) {
  auto fail = [&](const std::string& m) {
    if (why) *why = path + ": " + m;
    return false;
  };
  std::ifstream in(path);
  if (!in) return fail("cannot open");

  // Restricted YAML subset of A18: top-level scalars and one `segments:` list of flat mappings, 2-space indents,
  // no anchors/flow syntax. Unknown keys are ignored (the tool adds method/peak_corr/...).
  std::map<std::string, std::string> top;
  std::vector<std::map<std::string, std::string>> items;
  std::vector<int> item_line;
  bool in_segments = false;
  std::string line;
  int lineno = 0;
  while (std::getline(in, line)) {
    ++lineno;
    const std::string raw = stripComment(line);
    if (trim(raw).empty()) continue;
    if (raw.find('\t') != std::string::npos) return fail("line " + std::to_string(lineno) + ": tab indentation");
    const size_t indent = raw.find_first_not_of(' ');
    std::string body = trim(raw);
    if (body == "---" || body == "...") continue;
    if (body.find('{') != std::string::npos || body.find('[') != std::string::npos || body[0] == '&' || body[0] == '*')
      return fail("line " + std::to_string(lineno) + ": flow/anchor syntax is not part of the A18 subset");
    if (indent == 0) {
      in_segments = false;
      const size_t c = body.find(':');
      if (c == std::string::npos) return fail("line " + std::to_string(lineno) + ": expected `key: value`");
      const std::string key = trim(body.substr(0, c)), val = trim(body.substr(c + 1));
      if (key == "segments") {
        if (!val.empty()) return fail("line " + std::to_string(lineno) + ": `segments:` must open a block list");
        in_segments = true;
        continue;
      }
      top[key] = unquote(val);
      continue;
    }
    if (!in_segments) return fail("line " + std::to_string(lineno) + ": indented line outside `segments:`");
    if (indent == 2 && body.compare(0, 2, "- ") == 0) {
      items.emplace_back();
      item_line.push_back(lineno);
      body = trim(body.substr(2));
    } else if (indent == 2 && body == "-") {
      items.emplace_back();
      item_line.push_back(lineno);
      continue;
    } else if (indent != 4) {
      return fail("line " + std::to_string(lineno) + ": unexpected indentation " + std::to_string(indent));
    }
    if (items.empty()) return fail("line " + std::to_string(lineno) + ": segment key before the first `- `");
    const size_t c = body.find(':');
    if (c == std::string::npos) return fail("line " + std::to_string(lineno) + ": expected `key: value`");
    const std::string key = trim(body.substr(0, c)), val = unquote(trim(body.substr(c + 1)));
    if (items.back().count(key)) return fail("line " + std::to_string(lineno) + ": duplicate key " + key);
    items.back()[key] = val;
  }
  if (items.empty()) return fail("no segments");

  std::vector<Segment> segs;
  for (size_t i = 0; i < items.size(); ++i) {
    const auto& m = items[i];
    const std::string where = "segment at line " + std::to_string(item_line[i]);
    Segment s;
    double* dst[5] = {&s.t_from_gpst_s, &s.offset_s, &s.drift_ppm, &s.t_ref_gpst_s, &s.sigma_toff_s};
    const char* names[5] = {"t_from_gpst_s", "offset_s", "drift_ppm", "t_ref_gpst_s", "sigma_toff_s"};
    for (int k = 0; k < 5; ++k) {
      auto it = m.find(names[k]);
      if (it == m.end()) return fail(where + ": missing " + names[k]);
      if (!parseDouble(it->second, dst[k])) return fail(where + ": " + names[k] + " is not a number: " + it->second);
    }
    auto tp = m.find("tdcp_paused");
    if (tp == m.end()) return fail(where + ": missing tdcp_paused");
    if (!parseBool(tp->second, &s.tdcp_paused)) return fail(where + ": tdcp_paused is not a boolean: " + tp->second);
    if (!(s.sigma_toff_s >= 0.0)) return fail(where + ": sigma_toff_s < 0");
    if (s.drift_ppm != 0.0 && s.t_ref_gpst_s == 0.0) return fail(where + ": drift_ppm != 0 with t_ref_gpst_s = 0");
    if (!segs.empty() && !(s.t_from_gpst_s > segs.back().t_from_gpst_s))
      return fail(where + ": t_from_gpst_s must increase strictly");
    segs.push_back(s);
  }

  // leap_s is the leap second count the tool used for its GPST→UTC conversion: a different table value at the
  // calibrated epochs would shift the whole map by whole seconds, so it must agree when the table is in use.
  auto lp = top.find("leap_s");
  if (lp != top.end()) {
    double leap_file = 0.0;
    if (!parseDouble(lp->second, &leap_file)) return fail("leap_s is not a number: " + lp->second);
    for (const Segment& s : segs) {
      const double t_gps = s.t_ref_gpst_s != 0.0 ? s.t_ref_gpst_s : s.t_from_gpst_s;
      if (t_gps == 0.0) continue;
      const GTime at = fromToSec(t_gps);
      if (cfg_.base != Config::Time::Base::Gpst && leapSeconds(at) != leap_file)
        return fail("leap_s " + lp->second + " disagrees with the leap second table (" + std::to_string(leapSeconds(at)) + ")");
    }
  }
  setSegments(segs);
  return true;
}

// Frozen skeleton definition (df88477), kept verbatim: one segment from −∞; σ_toff and the TDCP pause of the former
// first segment carry over.
void TimeMap::setOffset(double offset_s, double drift_ppm, double t_ref_gpst_s) {
  Segment s = segments_.front();
  s.t_from_gpst_s = 0.0;
  s.offset_s = offset_s;
  s.drift_ppm = drift_ppm;
  s.t_ref_gpst_s = t_ref_gpst_s;
  segments_.assign(1, s);
}

void TimeMap::setSegments(const std::vector<Segment>& segments) {
  if (segments.empty()) throw std::invalid_argument("TimeMap::setSegments: at least one segment is required");
  std::vector<Segment> s = segments;
  std::stable_sort(s.begin(), s.end(), [](const Segment& a, const Segment& b) { return a.t_from_gpst_s < b.t_from_gpst_s; });
  segments_ = s;
}

const TimeMap::Segment& TimeMap::segmentAt(GTime gpst) const {
  // Segment i covers [t_from_i, t_from_{i+1}); the first one extends to −∞ (A18).
  if (segments_.size() == 1) return segments_.front();
  auto it = std::upper_bound(segments_.begin() + 1, segments_.end(), gpst,
                             [](const GTime& g, const Segment& s) { return secondsSince(g, s.t_from_gpst_s) < 0.0; });
  return *(it - 1);
}

bool TimeMap::isWholeSecond(GTime gpst, double tol_s) {
  // tow − round(tow) depends on the fraction only (the week-second integer part is exact).
  const double f = gpst.frac;
  return std::fabs(f - std::round(f)) < tol_s;
}

bool TimeMap::checkTimeOrigin(const std::vector<Segment>& segments, GTime span_begin, GTime span_end, std::string* why) {
  auto fail = [&](const std::string& m) {
    if (why) *why = m;
    return false;
  };
  if (!(span_end.diff(span_begin) >= 0.0)) return fail("time-origin check: data span ends before it begins");
  // Same order as setSegments(): the first segment by t_from is the one from −∞.
  std::vector<Segment> s = segments;
  std::stable_sort(s.begin(), s.end(), [](const Segment& a, const Segment& b) { return a.t_from_gpst_s < b.t_from_gpst_s; });
  // v ∈ [begin − 1 d, end + 1 d], evaluated against the GTime ends so no fraction passes through 1.7e9 (NaN: outside).
  auto minus = [](double v, GTime t) { return -secondsSince(t, v); };   // v − t
  auto inside = [&](double v) { return minus(v, span_begin) >= -kSpanMarginS && minus(v, span_end) <= kSpanMarginS; };
  auto describe = [&](size_t n, const char* key, double v) {
    char b[512];
    std::snprintf(b, sizeof(b),
                  "segment %zu: %s %.3f outside the data span [%.3f, %.3f] +- 1 day (time origin: GTime::toSec(), seconds "
                  "since 1970-01-01 on the GPST scale; a value on the 1980-01-06 origin is 315964800 s early)",
                  n, key, v, span_begin.toSec(), span_end.toSec());
    return std::string(b);
  };
  for (size_t n = 0; n < s.size(); ++n) {
    if (n > 0 && !inside(s[n].t_from_gpst_s)) return fail(describe(n, "t_from_gpst_s", s[n].t_from_gpst_s));
    if (s[n].drift_ppm != 0.0 && !inside(s[n].t_ref_gpst_s)) return fail(describe(n, "t_ref_gpst_s", s[n].t_ref_gpst_s));
  }
  return true;
}

bool TimeMap::loadCalibFile(const std::string& path, std::string* why, GTime span_begin, GTime span_end) {
  TimeMap parsed(*this);
  if (!parsed.loadCalibFile(path, why)) return false;
  std::string w;
  if (!checkTimeOrigin(parsed.segments_, span_begin, span_end, &w)) {
    if (why) *why = path + ": " + w;
    return false;
  }
  segments_ = parsed.segments_;
  return true;
}

}  // namespace livo_gnss
