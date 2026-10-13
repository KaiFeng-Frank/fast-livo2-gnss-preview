// SPDX-License-Identifier: BSD-3-Clause
// fixture_discovery.hpp — the one discovery rule for the A24 unit-test fixtures (header-only, gtest-free, POSIX).
// Owner: WP-e. Shared with WP-a's fixture-based U4–U6 (INTERFACE_CHANGES WP-e #3):
//   #include "../../gnss_io/test/fixture_discovery.hpp"   (quoted include, relative to gnss/test/)
//
// A fixture is a sub-directory of gnss/test/data holding msgdef/ and bytes/. Its name <seq>_<t0>_<D>s is
// informative only: D is the slice's real duration, chosen per slice within the 2 MB per-file cap (DEVELOPING §4;
// 43 s for the 22-satellite open-sky Grass02 slice, 76 s for the few-satellite Occlusion03 one), so nothing may key
// on the name. Directories whose name starts with '.' are the exporter's work-in-progress (tools/gnss_bag_export.py
// writes .tmp-* and renames at the end) and are never fixtures. The gnss_comm topics present differ by bag (A26:
// Occlusion03/04 and Dynamic03 carry no ephem, glo_ephem or iono_params): a check needing a topic tests for its
// msgdef/<topic>.msgdef.
//
// Synthetic_* fixtures are written by gnss/test/data/synthetic/make_synthetic_bag.py under the DESIGN §12.3
// assumptions: they prove that code and exporter agree with each other, nothing about the data. Every criterion
// about real data (time scales, URA/iono semantics, bag-vs-BRDC agreement) must iterate realFixtures().
#pragma once

#include <dirent.h>
#include <sys/stat.h>

#include <algorithm>
#include <string>
#include <vector>

namespace livo_gnss_io {
namespace testsupport {

inline bool isDir(const std::string& p) {
  struct stat st;
  return ::stat(p.c_str(), &st) == 0 && S_ISDIR(st.st_mode);
}

inline bool isFile(const std::string& p) {
  struct stat st;
  return ::stat(p.c_str(), &st) == 0 && S_ISREG(st.st_mode);
}

inline std::string baseName(const std::string& p) {
  const size_t s = p.find_last_of('/');
  return s == std::string::npos ? p : p.substr(s + 1);
}

// Every fixture directory under `root` (A24 layout: msgdef/ and bytes/ present), sorted by path.
inline std::vector<std::string> listFixtures(const std::string& root) {
  std::vector<std::string> out;
  DIR* d = ::opendir(root.c_str());
  if (!d) return out;
  while (dirent* e = ::readdir(d)) {
    const std::string name = e->d_name;
    if (name.empty() || name[0] == '.') continue;   // ".", "..", exporter work dirs
    const std::string p = root + "/" + name;
    if (isDir(p + "/msgdef") && isDir(p + "/bytes")) out.push_back(p);
  }
  ::closedir(d);
  std::sort(out.begin(), out.end());
  return out;
}

inline bool isSyntheticFixture(const std::string& dir) { return baseName(dir).compare(0, 10, "Synthetic_") == 0; }

// Fixtures cut from real (development-set) bags: the only ones a data-semantics criterion may use.
inline std::vector<std::string> realFixtures(const std::string& root) {
  std::vector<std::string> out;
  for (const std::string& f : listFixtures(root))
    if (!isSyntheticFixture(f)) out.push_back(f);
  return out;
}

inline std::vector<std::string> syntheticFixtures(const std::string& root) {
  std::vector<std::string> out;
  for (const std::string& f : listFixtures(root))
    if (isSyntheticFixture(f)) out.push_back(f);
  return out;
}

// The fixture's cropped same-day IGS broadcast file (brdc/*.rnx), "" when absent.
inline std::string fixtureBrdc(const std::string& fixture) {
  std::vector<std::string> found;
  DIR* d = ::opendir((fixture + "/brdc").c_str());
  if (!d) return std::string();
  while (dirent* e = ::readdir(d)) {
    const std::string n = e->d_name;
    if (n.size() > 4 && n.compare(n.size() - 4, 4, ".rnx") == 0) found.push_back(fixture + "/brdc/" + n);
  }
  ::closedir(d);
  std::sort(found.begin(), found.end());
  return found.empty() ? std::string() : found.front();
}

}  // namespace testsupport
}  // namespace livo_gnss_io
