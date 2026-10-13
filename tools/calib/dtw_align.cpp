// SPDX-License-Identifier: BSD-3-Clause
// dtw_align — runs the C++ livgoDtwLag (gnss/include/livo_gnss/livgo.hpp, header-only) on the series that tools/calib/dtw_scan.py
// handed to its Python replica, for the bit-level comparison of A81 2 item 1. Not part of any build target: compile it by hand with
// the flags of the gnss library (see docs/dev/A81_DTW_SCAN.md):
//
//   c++ -O3 -DNDEBUG -march=native -mtune=native -funroll-loops -ffp-contract=off -std=c++17 -DEIGEN_DONT_PARALLELIZE
//       -I<tree>/gnss/include -isystem /usr/include/eigen3 tools/calib/dtw_align.cpp -o dtw_align
//   dtw_align <dump from dtw_scan.py scan --dump> > results.txt
//   dtw_scan.py compare --dump <dump> --cpp results.txt
//
// Input (text, hexfloats): "SERIES <id> <n>" then n lines "t vg vh"; "CALL <series id> <grid> <window> <band> <py ok> <py lag|->".
// Output: one line per CALL, in order: "RES <index> <ok> <lag as %a | ->". The series is passed twice, as the facade does
// (dtw_tg == dtw_th element for element: both are the estimator time of the epoch).
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <string>
#include <vector>

#include "livo_gnss/livgo.hpp"

namespace {
struct Series {
  std::vector<double> t, vg, vh;
};
}  // namespace

int main(int argc, char** argv) {
  if (argc != 2) {
    std::fprintf(stderr, "usage: dtw_align <dump>\n");
    return 2;
  }
  std::FILE* f = std::fopen(argv[1], "r");
  if (!f) {
    std::fprintf(stderr, "cannot open %s\n", argv[1]);
    return 2;
  }
  std::map<long, Series> series;
  char line[512];
  long cur = -1;
  long remaining = 0;
  long index = 0;
  while (std::fgets(line, sizeof line, f)) {
    if (remaining > 0) {
      char *p = line, *e = nullptr;
      const double t = std::strtod(p, &e);
      p = e;
      const double g = std::strtod(p, &e);
      p = e;
      const double h = std::strtod(p, &e);
      Series& s = series[cur];
      s.t.push_back(t);
      s.vg.push_back(g);
      s.vh.push_back(h);
      --remaining;
      continue;
    }
    if (std::strncmp(line, "SERIES ", 7) == 0) {
      long id = 0, n = 0;
      std::sscanf(line + 7, "%ld %ld", &id, &n);
      cur = id;
      remaining = n;
      series[cur] = Series();
      series[cur].t.reserve(static_cast<size_t>(n));
      series[cur].vg.reserve(static_cast<size_t>(n));
      series[cur].vh.reserve(static_cast<size_t>(n));
    } else if (std::strncmp(line, "CALL ", 5) == 0) {
      long id = 0;
      char gs[64], ws[64], bs[64], oks[8], ls[64];
      if (std::sscanf(line + 5, "%ld %63s %63s %63s %7s %63s", &id, gs, ws, bs, oks, ls) != 6) {
        std::fprintf(stderr, "bad CALL line %ld\n", index);
        return 3;
      }
      const Series& s = series.at(id);
      livo_gnss::LivgoDtwParams prm;
      prm.grid_s = std::strtod(gs, nullptr);
      prm.window_s = std::strtod(ws, nullptr);
      prm.band_s = std::strtod(bs, nullptr);
      const std::vector<double> tg = s.t, th = s.t;   // the facade keeps dtw_tg and dtw_th as two vectors with the same values
      double lag = 0.0;
      const bool ok = livo_gnss::livgoDtwLag(tg, s.vg, th, s.vh, &lag, prm);
      if (ok)
        std::printf("RES %ld 1 %a\n", index, lag);
      else
        std::printf("RES %ld 0 -\n", index);
      ++index;
    }
  }
  std::fclose(f);
  std::fprintf(stderr, "dtw_align: %ld calls\n", index);
  return 0;
}
