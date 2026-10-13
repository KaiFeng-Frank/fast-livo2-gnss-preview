// SPDX-License-Identifier: BSD-3-Clause
// RinexSource — RINEX 3.x OBS/NAV input through RTKLIB readrnxt (DESIGN §12.4, U6, T4). A pull source:
// the replay driver asks for the next epoch and injects it at t_est + rinex.latency_s. Also used by
// tests to run the same epochs through the RINEX path and the bag path (U6: bit-identical pntpos).
// rtklib.h is included by rinex_source.cpp only (pimpl); the public types are livo_gnss's.
// Owner: WP-e. Tests: test_t01_t05_io.cpp (T04), gnss/test/test_u06_paths.cpp.
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "livo_gnss/types.hpp"

namespace livo_gnss_io {

class RinexSource {
 public:
  RinexSource();
  ~RinexSource();
  RinexSource(const RinexSource&) = delete;
  RinexSource& operator=(const RinexSource&) = delete;

  // Reads OBS (optional) and NAV (optional) files. `t_start`/`t_end` limit the observation window
  // (0 = unbounded). Returns false with `why` on read errors.
  bool open(const std::string& obs_path, const std::string& nav_path, livo_gnss::GTime t_start, livo_gnss::GTime t_end, std::string* why);
  // Sequential observation epochs (sorted by time; per-satellite primary signal first).
  bool nextEpoch(livo_gnss::ObsEpoch* out);
  size_t epochsRemaining() const;
  // Ephemerides and Klobuchar header parameters read from the NAV file.
  const std::vector<livo_gnss::KeplerEph>& ephemerides() const;
  const std::vector<livo_gnss::GloEph>& gloEphemerides() const;
  bool iono(livo_gnss::Klobuchar* out) const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace livo_gnss_io
