// SPDX-License-Identifier: GPL-2.0-only
// OutputWriter — file writers of DESIGN §15 / §13 (owner: WP-f). Formats:
//   state_trace.txt   one line per processOnce(), every number %a (hexfloat): bit-exact, so the determinism
//                     gates compare sha256 of this file (DESIGN §13 "四道门");
//   traj_*.tum        TUM "t x y z qx qy qz qw" with %.9f (evo input);
//   *.csv             %.17g doubles (exact round trip), header from the record's csvHeader();
//   run_manifest.json / summary.json / anchor.yaml; the manifest merges ManifestInfo::extra_json (one JSON object,
//                     members appended verbatim after the frozen fields; malformed or colliding → nothing merged
//                     and an "extra_json_error" member instead, so the file is always valid JSON).
// Nothing here reads the estimator; the caller (src/replay_main.cpp in ref-R, GnssRosBridge in P3) passes
// the values in. Files are opened lazily where a run may legitimately never produce them (N-frame outputs
// before ACTIVATE), eagerly where an empty file with a header is the expected outcome.
#include "livo_gnss_ros/outputs.hpp"

#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

namespace livo_gnss_ros {

namespace {

// One output file; open() is idempotent and remembers the first failure.
struct File {
  std::FILE* f = nullptr;
  std::string path;
  bool open(const std::string& p, std::string* why) {
    if (f) return true;
    path = p;
    f = std::fopen(p.c_str(), "w");
    if (!f) {
      if (why) *why = "cannot open " + p + ": " + std::strerror(errno);
      return false;
    }
    return true;
  }
  void flush() { if (f) std::fflush(f); }
  void close() { if (f) { std::fclose(f); f = nullptr; } }
};

void jsonEscapeTo(std::string& o, const std::string& s) {
  o += '"';
  for (unsigned char ch : s) {
    switch (ch) {
      case '"': o += "\\\""; break;
      case '\\': o += "\\\\"; break;
      case '\n': o += "\\n"; break;
      case '\r': o += "\\r"; break;
      case '\t': o += "\\t"; break;
      default:
        if (ch < 0x20) { char b[8]; std::snprintf(b, sizeof b, "\\u%04x", ch); o += b; }
        else o += static_cast<char>(ch);
    }
  }
  o += '"';
}

std::string g17(double v) {
  char b[40];
  std::snprintf(b, sizeof b, "%.17g", v);
  return b;
}

// TUM line: t x y z qx qy qz qw, %.9f (same precision as the upstream evo writer's std::fixed default of 6
// is *not* used: 9 decimals keep sub-nanometre position and the quaternion to 1e-9).
void tumLine(std::FILE* f, double t, const Eigen::Vector3d& p, const Eigen::Quaterniond& q) {
  std::fprintf(f, "%.9f %.9f %.9f %.9f %.9f %.9f %.9f %.9f\n", t, p.x(), p.y(), p.z(), q.x(), q.y(), q.z(), q.w());
}

// ManifestInfo::extra_json (P2b, AMENDMENTS A40): the members of one JSON object are merged verbatim into the top
// level of run_manifest.json. This recogniser (RFC 8259 grammar, nesting ≤ 64) only decides "exactly one well-formed
// object" and returns each top-level member as (raw key text, raw member text `"key": value`), so a malformed extra
// can never make the manifest itself invalid JSON. Keys are compared as raw text between the quotes.
class JsonMembers {
 public:
  explicit JsonMembers(const std::string& s) : s_(s) {}
  bool parse(std::vector<std::pair<std::string, std::string>>* out, std::string* why) {
    ws();
    if (i_ >= s_.size() || s_[i_] != '{') return fail("not a JSON object", why);
    if (!object(out)) return fail(err_.c_str(), why);
    ws();
    if (i_ != s_.size()) return fail("trailing text after the object", why);
    return true;
  }

 private:
  bool fail(const char* m, std::string* why) {
    if (why) *why = std::string(m) + " (offset " + std::to_string(i_) + ")";
    return false;
  }
  bool err(const char* m) {
    if (err_.empty()) err_ = m;
    return false;
  }
  void ws() {
    while (i_ < s_.size() && (s_[i_] == ' ' || s_[i_] == '\t' || s_[i_] == '\n' || s_[i_] == '\r')) ++i_;
  }
  bool digit() const { return i_ < s_.size() && s_[i_] >= '0' && s_[i_] <= '9'; }
  bool literal(const char* w) {
    const size_t n = std::strlen(w);
    if (s_.compare(i_, n, w) != 0) return err("bad literal");
    i_ += n;
    return true;
  }
  bool jstr() {
    if (i_ >= s_.size() || s_[i_] != '"') return err("expected a string");
    ++i_;
    while (i_ < s_.size()) {
      const unsigned char c = static_cast<unsigned char>(s_[i_]);
      if (c == '"') { ++i_; return true; }
      if (c < 0x20) return err("control character inside a string");
      if (c == '\\') {
        if (++i_ >= s_.size()) break;
        if (s_[i_] == 'u') {
          for (int k = 1; k <= 4; ++k)
            if (i_ + k >= s_.size() || !std::isxdigit(static_cast<unsigned char>(s_[i_ + k]))) return err("bad \\u escape");
          i_ += 5;
          continue;
        }
        if (std::strchr("\"\\/bfnrt", s_[i_]) == nullptr) return err("bad escape");
      }
      ++i_;
    }
    return err("unterminated string");
  }
  bool number() {
    if (i_ < s_.size() && s_[i_] == '-') ++i_;
    if (i_ < s_.size() && s_[i_] == '0') ++i_;
    else if (digit()) { while (digit()) ++i_; }
    else return err("bad value");
    if (i_ < s_.size() && s_[i_] == '.') {
      ++i_;
      if (!digit()) return err("bad fraction");
      while (digit()) ++i_;
    }
    if (i_ < s_.size() && (s_[i_] == 'e' || s_[i_] == 'E')) {
      ++i_;
      if (i_ < s_.size() && (s_[i_] == '+' || s_[i_] == '-')) ++i_;
      if (!digit()) return err("bad exponent");
      while (digit()) ++i_;
    }
    return true;
  }
  bool value() {
    ws();
    if (i_ >= s_.size()) return err("expected a value");
    switch (s_[i_]) {
      case '{': return object(nullptr);
      case '[': return array();
      case '"': return jstr();
      case 't': return literal("true");
      case 'f': return literal("false");
      case 'n': return literal("null");
      default: return number();
    }
  }
  bool array() {
    if (++depth_ > 64) return err("nesting deeper than 64");
    ++i_;
    ws();
    if (i_ < s_.size() && s_[i_] == ']') { ++i_; --depth_; return true; }
    for (;;) {
      if (!value()) return false;
      ws();
      if (i_ < s_.size() && s_[i_] == ',') { ++i_; continue; }
      if (i_ < s_.size() && s_[i_] == ']') { ++i_; --depth_; return true; }
      return err("expected ',' or ']'");
    }
  }
  bool object(std::vector<std::pair<std::string, std::string>>* members) {
    if (++depth_ > 64) return err("nesting deeper than 64");
    ++i_;
    ws();
    if (i_ < s_.size() && s_[i_] == '}') { ++i_; --depth_; return true; }
    for (;;) {
      ws();
      const size_t k0 = i_;
      if (!jstr()) return false;
      const size_t k1 = i_;
      ws();
      if (i_ >= s_.size() || s_[i_] != ':') return err("expected ':'");
      ++i_;
      if (!value()) return false;
      if (members) members->emplace_back(s_.substr(k0 + 1, k1 - k0 - 2), s_.substr(k0, i_ - k0));
      ws();
      if (i_ < s_.size() && s_[i_] == ',') { ++i_; continue; }
      if (i_ < s_.size() && s_[i_] == '}') { ++i_; --depth_; return true; }
      return err("expected ',' or '}'");
    }
  }

  const std::string& s_;
  size_t i_ = 0;
  int depth_ = 0;
  std::string err_;
};

}  // namespace

struct OutputWriter::Impl {
  OutputOptions opt;
  bool open = false;
  File state_trace, traj_w, traj_n, traj_ant_n, traj_geo, anchor, epochs, sats, cov, events, timing;
  File traj_n_odom, traj_ant_n_odom;   // A61 F13
  std::string why;

  std::string path(const char* name) const { return opt.dir + "/" + name; }

  // Lazy opener for files that exist only when the core produced something.
  std::FILE* lazy(File& fl, const char* name, const char* header) {
    if (!open) return nullptr;
    if (!fl.f) {
      if (!fl.open(path(name), &why)) return nullptr;
      if (header) std::fputs(header, fl.f);
    }
    return fl.f;
  }
};

OutputWriter::OutputWriter() : impl_(new Impl) {}
OutputWriter::~OutputWriter() { close(); delete impl_; }

bool OutputWriter::open(const OutputOptions& opt, std::string* why) {
  close();
  impl_->opt = opt;
  if (opt.dir.empty()) {   // "" = disabled: every write becomes a no-op, not an error
    impl_->open = false;
    return true;
  }
  std::error_code ec;
  std::filesystem::create_directories(opt.dir, ec);
  if (ec) {
    if (why) *why = "cannot create " + opt.dir + ": " + ec.message();
    return false;
  }
  std::string w;
  auto fail = [&](const std::string& msg) { if (why) *why = msg; close(); return false; };
  if (opt.state_trace) {
    if (!impl_->state_trace.open(impl_->path("state_trace.txt"), &w)) return fail(w);
    // Column key (DESIGN §13); one header line so the file is self-describing, identical across runs.
    std::fputs("# cycle flag t_k R(9,row-major) p(3) v(3) b_g(3) b_a(3) g(3) tau diagP(19) x_G(n) diagQ_G(n)  [%a hexfloat]\n",
               impl_->state_trace.f);
  }
  if (opt.traj_w && !impl_->traj_w.open(impl_->path("traj_W.tum"), &w)) return fail(w);
  if (opt.cov_csv) {
    if (!impl_->cov.open(impl_->path("filter_cov.csv"), &w)) return fail(w);
    std::fprintf(impl_->cov.f, "%s\n", livo_gnss::CovRecord::csvHeader().c_str());
  }
  if (!impl_->events.open(impl_->path("events.log"), &w)) return fail(w);
  if (opt.timing) {
    if (!impl_->timing.open(impl_->path("timing.csv"), &w)) return fail(w);
    std::fputs("cycle,t_k,lio_ms,vio_ms,gnss_ms\n", impl_->timing.f);
  }
  if (!impl_->epochs.open(impl_->path("gnss_epochs.csv"), &w)) return fail(w);
  std::fprintf(impl_->epochs.f, "%s\n", livo_gnss::EpochRecord::csvHeader().c_str());
  impl_->open = true;
  return true;
}

bool OutputWriter::isOpen() const { return impl_->open; }

void OutputWriter::writeStateTrace(int cycle, int flag, double t_k, const livo_gnss::CoreState& core, const Eigen::VectorXd& x_G,
                                   const Eigen::VectorXd& diag_Q_G) {
  std::FILE* f = impl_->state_trace.f;
  if (!impl_->open || !f) return;
  std::fprintf(f, "%d %d %a", cycle, flag, t_k);
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c) std::fprintf(f, " %a", core.R(r, c));
  for (int i = 0; i < 3; ++i) std::fprintf(f, " %a", core.p[i]);
  for (int i = 0; i < 3; ++i) std::fprintf(f, " %a", core.v[i]);
  for (int i = 0; i < 3; ++i) std::fprintf(f, " %a", core.bg[i]);
  for (int i = 0; i < 3; ++i) std::fprintf(f, " %a", core.ba[i]);
  for (int i = 0; i < 3; ++i) std::fprintf(f, " %a", core.g[i]);
  std::fprintf(f, " %a", core.inv_expo);
  for (int i = 0; i < livo_gnss::HostIdx::dim; ++i) std::fprintf(f, " %a", core.P(i, i));
  for (int i = 0; i < x_G.size(); ++i) std::fprintf(f, " %a", x_G[i]);
  for (int i = 0; i < diag_Q_G.size(); ++i) std::fprintf(f, " %a", diag_Q_G[i]);
  std::fputc('\n', f);
}

void OutputWriter::writeTrajW(double t_k, const Eigen::Matrix3d& R_WI, const Eigen::Vector3d& p_W) {
  std::FILE* f = impl_->traj_w.f;
  if (!impl_->open || !f) return;
  tumLine(f, t_k, p_W, Eigen::Quaterniond(R_WI));
}

void OutputWriter::writeTrajN(double t_k, const Eigen::Matrix3d& R_NI, const Eigen::Vector3d& p_N) {
  if (!impl_->opt.traj_n) return;
  std::FILE* f = impl_->lazy(impl_->traj_n, "traj_N.tum", nullptr);
  if (f) tumLine(f, t_k, p_N, Eigen::Quaterniond(R_NI));
}

void OutputWriter::writeTrajAntN(double t_k, const Eigen::Vector3d& a_N) {
  if (!impl_->opt.traj_n) return;
  std::FILE* f = impl_->lazy(impl_->traj_ant_n, "traj_ant_N.tum", nullptr);
  // Antenna position has no attitude: identity quaternion keeps the TUM layout evo expects.
  if (f) tumLine(f, t_k, a_N, Eigen::Quaterniond::Identity());
}

void OutputWriter::writeTrajNOdom(double t_k, const Eigen::Matrix3d& R_NI, const Eigen::Vector3d& p_N) {
  if (!impl_->opt.traj_n) return;
  std::FILE* f = impl_->lazy(impl_->traj_n_odom, "traj_N_odom.tum", nullptr);
  if (f) tumLine(f, t_k, p_N, Eigen::Quaterniond(R_NI));
}

void OutputWriter::writeTrajAntNOdom(double t_k, const Eigen::Vector3d& a_N) {
  if (!impl_->opt.traj_n) return;
  std::FILE* f = impl_->lazy(impl_->traj_ant_n_odom, "traj_ant_N_odom.tum", nullptr);
  if (f) tumLine(f, t_k, a_N, Eigen::Quaterniond::Identity());
}

void OutputWriter::writeTrajGeo(double t_k, const livo_gnss::Geodetic& g) {
  if (!impl_->opt.traj_n) return;
  std::FILE* f = impl_->lazy(impl_->traj_geo, "traj_N_geo.csv", "t_k,lat_deg,lon_deg,h_m\n");
  if (f) std::fprintf(f, "%.9f,%.17g,%.17g,%.17g\n", t_k, g.lat_rad * 180.0 / M_PI, g.lon_rad * 180.0 / M_PI, g.h_m);
}

void OutputWriter::writeAnchor(const livo_gnss::AnchorFrame& a) {
  if (!impl_->open) return;
  // Rewritten on every call (the anchor is fixed once; a re-anchor would be a new file version).
  impl_->anchor.close();
  if (!impl_->anchor.open(impl_->path("anchor.yaml"), &impl_->why)) return;
  std::FILE* f = impl_->anchor.f;
  std::fprintf(f, "valid: %s\n", a.valid ? "true" : "false");
  std::fprintf(f, "r0_ecef: [%s, %s, %s]\n", g17(a.r0_ecef.x()).c_str(), g17(a.r0_ecef.y()).c_str(), g17(a.r0_ecef.z()).c_str());
  std::fprintf(f, "R_EN:  # row-major, columns = east/north/up unit vectors at r0\n");
  for (int r = 0; r < 3; ++r)
    std::fprintf(f, "  - [%s, %s, %s]\n", g17(a.R_EN(r, 0)).c_str(), g17(a.R_EN(r, 1)).c_str(), g17(a.R_EN(r, 2)).c_str());
  std::fprintf(f, "lat_deg: %s\nlon_deg: %s\nh_m: %s\n", g17(a.geo.lat_rad * 180.0 / M_PI).c_str(), g17(a.geo.lon_rad * 180.0 / M_PI).c_str(),
               g17(a.geo.h_m).c_str());
  std::fprintf(f, "t_fixed_gpst_sec: %lld\nt_fixed_gpst_frac: %s\n", static_cast<long long>(a.t_fixed.sec), g17(a.t_fixed.frac).c_str());
  impl_->anchor.flush();
}

void OutputWriter::writeTiming(int cycle, double t_k, double lio_ms, double vio_ms, double gnss_ms) {
  std::FILE* f = impl_->timing.f;
  if (!impl_->open || !f) return;
  std::fprintf(f, "%d,%.9f,%.6f,%.6f,%.6f\n", cycle, t_k, lio_ms, vio_ms, gnss_ms);
}

void OutputWriter::writeManifest(const ManifestInfo& m) {
  if (!impl_->open) return;
  File mf;
  if (!mf.open(impl_->path("run_manifest.json"), &impl_->why)) return;
  // One "  \"key\": value" line per member, joined with ",\n" (the frozen fields first, in their historical order).
  std::vector<std::string> lines;
  std::set<std::string> keys;
  auto add = [&](const char* k, const std::string& value_json) {
    lines.push_back(std::string("  \"") + k + "\": " + value_json);
    keys.insert(k);
  };
  auto str = [&](const char* k, const std::string& v) { std::string j; jsonEscapeTo(j, v); add(k, j); };
  str("git_describe", m.git_describe);
  str("git_dirty", m.git_dirty);
  str("profile", m.profile);
  str("compile_defs", m.compile_defs);
  str("eigen_version", m.eigen_version);
  str("compiler", m.compiler);
  str("image_digest", m.image_digest);
  str("config_sha256", m.config_sha256);
  str("config_dump", m.config_dump);
  str("bag_path", m.bag_path);
  str("bag_sha256", m.bag_sha256);
  str("arm", m.arm);
  str("host", m.host);
  add("seed", std::to_string(m.seed));
  add("start_offset_s", g17(m.start_offset_s));
  add("t_start_wall", g17(m.t_start_wall));
  add("t_end_wall", g17(m.t_end_wall));

  // extra_json: all-or-nothing. A malformed text, or a member whose key repeats a manifest field or another extra
  // member, merges nothing and is reported in "extra_json_error" — the manifest stays valid JSON with unique keys.
  if (m.extra_json.find_first_not_of(" \t\r\n") != std::string::npos) {
    std::vector<std::pair<std::string, std::string>> members;
    std::string why;
    if (JsonMembers(m.extra_json).parse(&members, &why)) {
      std::set<std::string> seen;
      for (const auto& kv : members) {
        if (keys.count(kv.first) != 0 || !seen.insert(kv.first).second) {
          why = "duplicate key \"" + kv.first + "\"";
          break;
        }
      }
    }
    if (why.empty()) {
      for (const auto& kv : members) lines.push_back("  " + kv.second);
    } else {
      std::string j;
      jsonEscapeTo(j, "extra_json not merged: " + why);
      lines.push_back("  \"extra_json_error\": " + j);
    }
  }

  std::string s = "{\n";
  for (size_t i = 0; i < lines.size(); ++i) {
    s += lines[i];
    s += (i + 1 < lines.size()) ? ",\n" : "\n";
  }
  s += "}\n";
  std::fputs(s.c_str(), mf.f);
  mf.close();
}

void OutputWriter::onEpoch(const livo_gnss::EpochRecord& r) {
  std::FILE* f = impl_->epochs.f;
  if (!impl_->open || !f) return;
  std::fprintf(f, "%s\n", r.csvRow().c_str());
}

void OutputWriter::onSat(const livo_gnss::SatRecord& r) {
  if (!impl_->opt.sat_csv) return;
  std::FILE* f = impl_->lazy(impl_->sats, "gnss_sats.csv", nullptr);
  if (!f) return;
  if (std::ftell(f) == 0) std::fprintf(f, "%s\n", livo_gnss::SatRecord::csvHeader().c_str());
  std::fprintf(f, "%s\n", r.csvRow().c_str());
}

void OutputWriter::onCov(const livo_gnss::CovRecord& r) {
  std::FILE* f = impl_->cov.f;
  if (!impl_->open || !f) return;
  std::fprintf(f, "%s\n", r.csvRow().c_str());
}

void OutputWriter::onEvent(const livo_gnss::Event& e) {
  std::FILE* f = impl_->events.f;
  if (!impl_->open || !f) return;
  std::fprintf(f, "%s\n", e.logLine().c_str());
}

void OutputWriter::onSummary(const livo_gnss::Summary& s) {
  if (!impl_->open) return;
  File sf;
  if (!sf.open(impl_->path("summary.json"), &impl_->why)) return;
  std::fputs(s.toJson().c_str(), sf.f);
  sf.close();
}

void OutputWriter::flush() {
  for (File* fl : {&impl_->state_trace, &impl_->traj_w, &impl_->traj_n, &impl_->traj_ant_n, &impl_->traj_n_odom,
                   &impl_->traj_ant_n_odom, &impl_->traj_geo, &impl_->anchor,
                   &impl_->epochs, &impl_->sats, &impl_->cov, &impl_->events, &impl_->timing})
    fl->flush();
}

void OutputWriter::close() {
  for (File* fl : {&impl_->state_trace, &impl_->traj_w, &impl_->traj_n, &impl_->traj_ant_n, &impl_->traj_n_odom,
                   &impl_->traj_ant_n_odom, &impl_->traj_geo, &impl_->anchor,
                   &impl_->epochs, &impl_->sats, &impl_->cov, &impl_->events, &impl_->timing})
    fl->close();
  impl_->open = false;
}

}  // namespace livo_gnss_ros
