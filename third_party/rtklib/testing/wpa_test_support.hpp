// SPDX-License-Identifier: BSD-2-Clause
// wpa_test_support.hpp — test support for WP-a's U3–U6 (gtest only; never compiled into a library).
//
// 1. The IGS BRDC snippet the U3/U4/U5 tests evaluate lives outside the repository (third_party/rtklib/VENDORED.md:
//    provenance, size, sha256; read from $LIVO_GNSS_BRDC_DIR, default /r/data/brdc = /work/fastlivo2_gnss/data/brdc
//    inside the build container). Every numeric expectation of those tests was derived from exactly that file, so its
//    SHA-256 is pinned here (review m5): a missing file is an external block (SKIP "blocked: ..."), a different file is a
//    failure.
// 2. "blocked" is reserved for dependencies that are still skeleton stubs. A stub throws std::logic_error("not
//    implemented: ...") (gnss/src/internal/stub.hpp, gnss_io stub()); std::invalid_argument & co. derive from
//    std::logic_error too, so a bare catch would relabel a real failure of merged code as "blocked".
// 3. Fixture slices (A24/A39; files and records owned by WP-e). Discovery is WP-e's one rule,
//    gnss_io/test/fixture_discovery.hpp: a slice is any sub-directory of gnss/test/data with msgdef/ and bytes/
//    (listFixtures(); '.'-prefixed exporter work dirs are skipped), Synthetic_* slices are isSyntheticFixture(), the
//    rest is realFixtures(). The name <seq>_<t0>_<D>s is informative only. On top of it WP-a keeps the A2 guard (A24
//    "WP-a 请求"): a slice named after an evaluation sequence fails the test and is never read. Gate tests (every
//    real-data criterion: time scales, URA/iono semantics, bag vs BRDC, path equivalence, the pntpos golden) run on the
//    dev-set slices only (realFixtures() without evaluation names) and stay "blocked: dev-set fixture" without one; the
//    synthetic slices run in separate *SyntheticSlice tests whose PASS means "PASS(合成)" — code and exporter agree
//    with each other, nothing about the data (A24).
#pragma once

#include <dirent.h>

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#include "../../../gnss_io/test/fixture_discovery.hpp"   // A24/A39: the one discovery rule (WP-e)
#include "livo_gnss/gnss_fusion.hpp"
#include "livo_gnss_io/gnss_comm_mapping.hpp"

namespace livo_wpa_testing {

constexpr const char* kSnippetName = "BRDC00IGS_R_20250030000_01D_MN_0400-0900_GRECJ.rnx";
constexpr const char* kSnippetSha256 = "b9132287f7c3952101a9c5fc938fa81f1fb61b9d92e8193c190ca842cefd09c6";

inline std::string brdcPath() {
  const char* d = std::getenv("LIVO_GNSS_BRDC_DIR");
  return std::string(d && *d ? d : "/r/data/brdc") + "/" + kSnippetName;
}

// SHA-256 (FIPS 180-4), lower-case hex.
inline std::string sha256Hex(std::vector<unsigned char> m) {
  static const uint32_t K[64] = {
      0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5, 0xd807aa98, 0x12835b01,
      0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174, 0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc,
      0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da, 0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147,
      0x06ca6351, 0x14292967, 0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
      0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070, 0x19a4c116, 0x1e376c08,
      0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3, 0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208,
      0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2};
  uint32_t h[8] = {0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a, 0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19};
  const uint64_t bits = static_cast<uint64_t>(m.size()) * 8u;
  m.push_back(0x80);
  while (m.size() % 64 != 56) m.push_back(0);
  for (int i = 7; i >= 0; --i) m.push_back(static_cast<unsigned char>(bits >> (8 * i)));
  auto rotr = [](uint32_t x, int n) { return (x >> n) | (x << (32 - n)); };
  for (size_t off = 0; off < m.size(); off += 64) {
    uint32_t w[64];
    for (int i = 0; i < 16; ++i)
      w[i] = (static_cast<uint32_t>(m[off + 4 * i]) << 24) | (static_cast<uint32_t>(m[off + 4 * i + 1]) << 16) |
             (static_cast<uint32_t>(m[off + 4 * i + 2]) << 8) | static_cast<uint32_t>(m[off + 4 * i + 3]);
    for (int i = 16; i < 64; ++i) {
      const uint32_t s0 = rotr(w[i - 15], 7) ^ rotr(w[i - 15], 18) ^ (w[i - 15] >> 3);
      const uint32_t s1 = rotr(w[i - 2], 17) ^ rotr(w[i - 2], 19) ^ (w[i - 2] >> 10);
      w[i] = w[i - 16] + s0 + w[i - 7] + s1;
    }
    uint32_t a = h[0], b = h[1], c = h[2], d = h[3], e = h[4], f = h[5], g = h[6], k = h[7];
    for (int i = 0; i < 64; ++i) {
      const uint32_t t1 = k + (rotr(e, 6) ^ rotr(e, 11) ^ rotr(e, 25)) + ((e & f) ^ (~e & g)) + K[i] + w[i];
      const uint32_t t2 = (rotr(a, 2) ^ rotr(a, 13) ^ rotr(a, 22)) + ((a & b) ^ (a & c) ^ (b & c));
      k = g;
      g = f;
      f = e;
      e = d + t1;
      d = c;
      c = b;
      b = a;
      a = t1 + t2;
    }
    h[0] += a; h[1] += b; h[2] += c; h[3] += d; h[4] += e; h[5] += f; h[6] += g; h[7] += k;
  }
  char out[65] = {0};
  for (int i = 0; i < 8; ++i) std::snprintf(out + 8 * i, 9, "%08x", static_cast<unsigned>(h[i]));
  return std::string(out, 64);
}

inline std::string sha256File(const std::string& path) {
  std::ifstream f(path, std::ios::binary);
  if (!f) return std::string();
  return sha256Hex(std::vector<unsigned char>((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>()));
}

enum class SnippetState { Missing, Ok, Mismatch };
// Hashed once per test executable (1.77 MB).
inline SnippetState snippetState(std::string* sha_out = nullptr) {
  static std::string sha;
  static bool done = false;
  if (!done) {
    if (FILE* f = std::fopen(brdcPath().c_str(), "r")) {
      std::fclose(f);
      sha = sha256File(brdcPath());
    }
    done = true;
  }
  if (sha_out) *sha_out = sha;
  if (sha.empty()) return SnippetState::Missing;
  return sha == kSnippetSha256 ? SnippetState::Ok : SnippetState::Mismatch;
}

// True for the exception of a skeleton stub (see 2. above).
inline bool isSkeletonStub(const std::exception& e) { return std::strncmp(e.what(), "not implemented", 15) == 0; }

// ---- 3. fixture slices ----
inline std::string readText(const std::string& p) {
  std::ifstream f(p, std::ios::binary);
  return std::string((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
}
inline std::string trimmed(std::string s) {
  while (!s.empty() && (s.back() == '\n' || s.back() == '\r' || s.back() == ' ')) s.pop_back();
  return s;
}
// The file of `dir` whose name contains `token` (and not `not_token`) and ends in `ext`; "" when none. Names are sorted
// first so that the pick does not depend on readdir order.
inline std::string findFile(const std::string& dir, const std::string& token, const std::string& ext, const std::string& not_token = "") {
  std::vector<std::string> names;
  if (DIR* d = ::opendir(dir.c_str())) {
    while (dirent* e = ::readdir(d)) names.emplace_back(e->d_name);
    ::closedir(d);
  }
  std::sort(names.begin(), names.end());
  for (const std::string& n : names)
    if (n.find(token) != std::string::npos && (not_token.empty() || n.find(not_token) == std::string::npos) && n.size() > ext.size() &&
        n.compare(n.size() - ext.size(), ext.size(), ext) == 0)
      return dir + "/" + n;
  return std::string();
}

enum class SliceKind { DevSet, Synthetic, EvalSet };
struct Slice {
  std::string path, name;
  SliceKind kind = SliceKind::DevSet;
};
// A2: the evaluation sequences (never a fixture), compared case-insensitively with '-' and '_' removed.
inline bool isEvalSetName(const std::string& name) {
  std::string n;
  for (char c : name)
    if (c != '-' && c != '_') n += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  static const char* const kEval[] = {"varyingillu03", "varyingillu04", "varyingillu05", "outdoor01", "outdoor04",
                                      "dark01",        "dark02",        "gnssdenial01",  "gnssdenial02"};
  for (const char* e : kEval)
    if (n.compare(0, std::strlen(e), e) == 0) return true;
  return false;
}
// Every fixture of `data_dir` by WP-e's listFixtures() (sorted by path), classified: evaluation-sequence name (A2) first,
// then Synthetic_* (isSyntheticFixture), else dev set — i.e. DevSet = realFixtures() without evaluation names.
inline std::vector<Slice> fixtureSlices(const std::string& data_dir) {
  namespace ts = livo_gnss_io::testsupport;
  std::vector<Slice> v;
  for (const std::string& p : ts::listFixtures(data_dir)) {
    const std::string n = ts::baseName(p);
    const SliceKind k = isEvalSetName(n) ? SliceKind::EvalSet : ts::isSyntheticFixture(p) ? SliceKind::Synthetic : SliceKind::DevSet;
    v.push_back(Slice{p, n, k});
  }
  return v;
}
inline std::vector<Slice> slicesOf(const std::vector<Slice>& v, SliceKind k) {
  std::vector<Slice> out;
  for (const Slice& x : v)
    if (x.kind == k) out.push_back(x);
  return out;
}
inline std::string namesOf(const std::vector<Slice>& v, SliceKind k) {
  std::string s;
  for (const Slice& x : v)
    if (x.kind == k) s += " " + x.name;
  return s;
}

// The gnss_comm channels of a slice (WP-e's exporter names the files after the topic's last component).
struct Channel {
  livo_gnss::Source src;
  const char* datatype;
  const char* token;
  const char* not_token;
};
const Channel kMeasChannel{livo_gnss::Source::GnssCommMeas, "gnss_comm/GnssMeasMsg", "range_meas", ""};
const Channel kEphemChannel{livo_gnss::Source::GnssCommEphem, "gnss_comm/GnssEphemMsg", "ephem", "glo"};
const Channel kGloEphemChannel{livo_gnss::Source::GnssCommGloEphem, "gnss_comm/GnssGloEphemMsg", "glo_ephem", ""};
const Channel kIonoChannel{livo_gnss::Source::GnssCommIono, "gnss_comm/StampedFloat64Array", "iono", ""};

struct DecodeStats {
  int messages = 0, refused = 0;
  std::string first_refusal;
};
// Feeds every record of bytes/<topic>.bin ([uint64 t_record_ns][uint32 len][len bytes]…, A24) of channel `c` through
// `dec` into `sink`, in record order. The decoder refuses single messages (wire errors, and semantically invalid ones
// such as an iono_params array that is not 8 long — WP-e's contract in INTERFACE_CHANGES), and a fixture may carry
// such messages on purpose: they are counted in `st`, not fatal. False (with `why`) for a missing channel when
// `required`, a rejected msg_def or a truncated file. A decoder that is still a stub throws (see 2.).
inline bool decodeChannel(livo_gnss_io::GnssCommRos1Decoder& dec, const std::string& slice, const Channel& c,
                          livo_gnss::MeasurementSink& sink, DecodeStats* st, std::string* why, bool required = true) {
  const std::string def = findFile(slice + "/msgdef", c.token, ".msgdef", c.not_token);
  const std::string md5 = findFile(slice + "/msgdef", c.token, ".md5", c.not_token);
  const std::string bin = findFile(slice + "/bytes", c.token, ".bin", c.not_token);
  if (def.empty() || md5.empty() || bin.empty()) {
    if (why) *why = slice + ": fixture channel " + c.token + " missing (A24 msgdef/*.msgdef, msgdef/*.md5, bytes/*.bin)";
    return !required;
  }
  if (!dec.registerMessageDefinition(c.src, c.datatype, trimmed(readText(md5)), readText(def), why)) return false;
  std::ifstream f(bin, std::ios::binary);
  for (;;) {
    uint64_t t_ns = 0;
    uint32_t len = 0;
    if (!f.read(reinterpret_cast<char*>(&t_ns), 8)) break;
    if (!f.read(reinterpret_cast<char*>(&len), 4)) {
      if (why) *why = bin + ": truncated record header";
      return false;
    }
    std::vector<uint8_t> buf(len);
    if (len && !f.read(reinterpret_cast<char*>(buf.data()), len)) {
      if (why) *why = bin + ": truncated record";
      return false;
    }
    std::string w;
    ++st->messages;
    if (!dec.decode(c.src, buf.data(), buf.size(), static_cast<double>(t_ns) * 1e-9, sink, &w)) {
      if (st->refused++ == 0) st->first_refusal = std::string(c.token) + ": " + w;
    }
  }
  return true;
}

}  // namespace livo_wpa_testing

// Top of a TEST body: SKIP (external block) when the snippet is absent, FAIL when it is not the pinned file.
#define LIVO_REQUIRE_BRDC_SNIPPET()                                                                                     \
  do {                                                                                                                  \
    std::string sha_now_;                                                                                               \
    const livo_wpa_testing::SnippetState st_ = livo_wpa_testing::snippetState(&sha_now_);                               \
    if (st_ == livo_wpa_testing::SnippetState::Missing)                                                                 \
      GTEST_SKIP() << "blocked: BRDC snippet not found at " << livo_wpa_testing::brdcPath()                             \
                   << " (set LIVO_GNSS_BRDC_DIR; provenance in third_party/rtklib/VENDORED.md)";                        \
    if (st_ == livo_wpa_testing::SnippetState::Mismatch)                                                                \
      FAIL() << "BRDC snippet " << livo_wpa_testing::brdcPath() << " has sha256 " << sha_now_ << ", expected "          \
             << livo_wpa_testing::kSnippetSha256 << " (third_party/rtklib/VENDORED.md)";                                \
  } while (0)

// Body of a catch (const std::logic_error& e) around a call into another work package: SKIP "blocked: <dep>" when that
// code is still a stub, FAIL otherwise.
#define LIVO_BLOCKED_IF_STUB(e, dep)                                                                                    \
  do {                                                                                                                  \
    if (!livo_wpa_testing::isSkeletonStub(e)) FAIL() << "exception from merged code (not a skeleton stub): " << (e).what(); \
    GTEST_SKIP() << "blocked: " << dep << " (" << (e).what() << ")";                                                    \
  } while (0)

// Top of a gate TEST on real data (A24/A39): `var` = the dev-set slices. FAIL when an evaluation-sequence slice is
// present (A2: it is never read); SKIP "blocked: dev-set fixture" when there is no dev-set slice (synthetic slices never
// stand in for one).
#define LIVO_DEVSET_SLICES(var)                                                                                         \
  std::vector<livo_wpa_testing::Slice> var;                                                                             \
  do {                                                                                                                  \
    const std::vector<livo_wpa_testing::Slice> all_ = livo_wpa_testing::fixtureSlices(LIVO_GNSS_TEST_DATA_DIR);        \
    const std::string eval_ = livo_wpa_testing::namesOf(all_, livo_wpa_testing::SliceKind::EvalSet);                    \
    if (!eval_.empty()) FAIL() << "AMENDMENTS A2: evaluation-sequence fixture slice(s) in gnss/test/data, not read:" << eval_; \
    var = livo_wpa_testing::slicesOf(all_, livo_wpa_testing::SliceKind::DevSet);                                       \
    if (var.empty())                                                                                                    \
      GTEST_SKIP() << "blocked: dev-set fixture (A24 gnss/test/data/<seq>_<t0>_<D>s from WP-e's exporter; realFixtures() "  \
                   << "is empty; synthetic slices:" << livo_wpa_testing::namesOf(all_, livo_wpa_testing::SliceKind::Synthetic) \
                   << ")";                                                                                              \
  } while (0)

// Top of a companion TEST on WP-e's synthetic slices (A24 "PASS(合成)"): `var` = the Synthetic_* slices; same A2 guard;
// SKIP when there is none.
#define LIVO_SYNTHETIC_SLICES(var)                                                                                      \
  std::vector<livo_wpa_testing::Slice> var;                                                                             \
  do {                                                                                                                  \
    const std::vector<livo_wpa_testing::Slice> all_ = livo_wpa_testing::fixtureSlices(LIVO_GNSS_TEST_DATA_DIR);        \
    const std::string eval_ = livo_wpa_testing::namesOf(all_, livo_wpa_testing::SliceKind::EvalSet);                    \
    if (!eval_.empty()) FAIL() << "AMENDMENTS A2: evaluation-sequence fixture slice(s) in gnss/test/data, not read:" << eval_; \
    var = livo_wpa_testing::slicesOf(all_, livo_wpa_testing::SliceKind::Synthetic);                                    \
    if (var.empty()) GTEST_SKIP() << "no synthetic slice (Synthetic_*) in gnss/test/data";                              \
  } while (0)
