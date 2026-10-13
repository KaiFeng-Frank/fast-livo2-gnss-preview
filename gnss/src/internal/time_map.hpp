// SPDX-License-Identifier: BSD-3-Clause
// TimeMap — GNSS time systems and the GPST ↔ estimator-time mapping (DESIGN §1 "时间", §7).
// Implements: t_est = t_gpst,true − leap(t) + offset_i + drift_i·(t_gpst − t_ref,i), t_gpst,true = t_rcv − b/c,
// piecewise: the offline tool splits the sequence into segments at host-clock steps (§7 "台阶即分段") and
// may mark a segment "TDCP paused" (insufficient excitation); the calibration YAML format is fixed in
// AMENDMENTS A18 (m8). All GNSS-system conversions go through rtk_adapter (RTKLIB functions) so results
// are bit-identical to RTKLIB (U1). Owner: WP-a. Tests: test_u01_time.cpp.
#pragma once

#include <string>
#include <vector>

#include "livo_gnss/config.hpp"
#include "livo_gnss/types.hpp"

namespace livo_gnss {

class TimeMap {
 public:
  explicit TimeMap(const Config::Time& cfg);

  // Leap seconds GPST−UTC at `gpst` from the RTKLIB table (rtkcmn.c:171; 18 s from 2017-01-01).
  // cfg.leap == "auto" uses the table; an integer string overrides it (tests, future dates).
  double leapSeconds(GTime gpst) const;

  // Estimator time of a *true* GPST instant (receiver clock bias already removed). Units: seconds on
  // the host sensor timeline (LiDAR header scale, DESIGN §1).
  double toEstimator(GTime t_gpst_true) const;
  // Inverse mapping (Newton on the drift term is unnecessary: the map is affine in t).
  GTime toGpst(double t_est) const;
  // Convenience: receiver time tag + clock bias [m] → estimator time (t_true = t_rcv − b/c).
  double receiverToEstimator(GTime t_rcv, double clock_bias_m) const;
  static GTime receiverToTrue(GTime t_rcv, double clock_bias_m);

  // One calibrated segment [t_from, next.t_from) (A18). Every *_gpst_s value (here, in the calibration file and in
  // gnss.time.t_ref_gpst_s) is GTime::toSec(): seconds since 1970-01-01 on the GPST scale, no leap seconds
  // (= 315964800 + week·604800 + tow; A18/A40). A single segment with t_from = 0 is the plain offset+drift model of
  // DESIGN §7.
  struct Segment {
    double t_from_gpst_s = 0.0;
    double offset_s = 0.0, drift_ppm = 0.0, t_ref_gpst_s = 0.0;
    double sigma_toff_s = 0.05;
    bool tdcp_paused = false;          // excitation too low in this segment: TDCP off, σ_toff = 50 ms (§7)
  };
  // Per-sequence calibration (tools/time_calib/gnss_time_calib.py output, DESIGN §7, format A18). Replaces
  // the config's offset/drift/sigma with the file's segments. Returns false with `why` on parse error.
  bool loadCalibFile(const std::string& path, std::string* why);
  // Single-segment convenience (config values / tests).
  void setOffset(double offset_s, double drift_ppm, double t_ref_gpst_s);
  // Add a constant to every segment's offset (FAST-LIVGO online DTW residual). Does not replace segments.
  void addOffset(double delta_s);
  void setSegments(const std::vector<Segment>& segments);   // sorted by t_from; first t_from is ignored (−∞)
  const std::vector<Segment>& segments() const { return segments_; }
  const Segment& segmentAt(GTime gpst) const;
  // Values of the active segment (single-segment case: the constants of the config).
  double offset(GTime gpst) const { return segmentAt(gpst).offset_s; }
  double driftPpm(GTime gpst) const { return segmentAt(gpst).drift_ppm; }
  double tRef(GTime gpst) const { return segmentAt(gpst).t_ref_gpst_s; }
  double sigmaToff(GTime gpst) const { return segmentAt(gpst).sigma_toff_s; }
  bool tdcpPaused(GTime gpst) const { return segmentAt(gpst).tdcp_paused; }

  // RETIRED (AMENDMENTS A43): the former §4.3 1 Hz predicate |tow − round(tow)| < 5 ms. The core no longer calls it —
  // the rule is "first epoch to arrive in each GPST second" (rows.hpp EpochDecimator); kept for U1 only.
  static bool isWholeSecond(GTime gpst, double tol_s = 5e-3);

 private:
  Config::Time cfg_;
  std::vector<Segment> segments_;      // never empty after construction (one segment from cfg_)
  bool leap_fixed_ = false;
  double leap_fixed_s_ = 18.0;

 public:
  // ---- appended 2026-09-25, P2b (WP-a; AMENDMENTS A18 time origin, A40; WP-g request in INTERFACE_CHANGES) ----
  // Time-origin sanity check (A40): every segment start except the first one (−∞) and the t_ref of every segment with
  // drift_ppm != 0 must lie in [span_begin − 1 day, span_end + 1 day], span = the GPST extent of the data (e.g. first
  // and last GNSS epoch). A value written on the 1980-01-06 origin is 315964800 s early and fails, so does a relative
  // time; t_ref of a drift-free segment is never used and not checked. Same rule as tools/fl2g/timecalib_yaml.py
  // check_span(). False with `why` naming the segment and the value; also false for span_end before span_begin.
  static bool checkTimeOrigin(const std::vector<Segment>& segments, GTime span_begin, GTime span_end, std::string* why);
  // loadCalibFile() plus checkTimeOrigin() on the file's segments; the map is replaced only when both pass. The
  // two-argument form cannot check the origin (no span) and stays for callers that do not know the data span yet.
  bool loadCalibFile(const std::string& path, std::string* why, GTime span_begin, GTime span_end);
};

}  // namespace livo_gnss
