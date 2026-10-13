// SPDX-License-Identifier: BSD-3-Clause
// signal_table — the ONE copy of the DESIGN §12.3 "freqs → CODE_*" table (M6: it used to exist twice,
// as rtk::codeFromFrequency and livo_gnss_io::signalFromFrequency). Header-only and RTKLIB-free so that
// gnss_io (which must not link the core) and rtk_adapter (which maps SignalCode to RTKLIB CODE_*) share it.
//
//   GPS/QZS  1575.42 → L1C   1227.60 → L2X   1176.45 → L5X (GF checks only)
//   GLO      1602 + 0.5625k → L1C   1246 + 0.4375k → L2C   (k integer in [−7, +6], recovered here)
//   GAL      1575.42 → L1X   1207.14 → L7X   1176.45 → L5Q (GF only)
//   BDS      1561.098 → L2I (RINEX C2I, TGD_B1I in prange)   1207.14 → L7I   1575.42 → REJECTED (B1C, counted)
//   anything else → Unknown (rejected, counted as unknown frequency)
// Frequencies are matched within kFreqTolHz; receivers report them as exact doubles (e.g. 1575420000.0).
// Frozen with the skeleton (data, not algorithm). Tests: U3 FreqToCodeTable (WP-a), T1 (WP-e).
#pragma once

#include <cmath>

#include "livo_gnss/types.hpp"

namespace livo_gnss {

enum class SignalVerdict : uint8_t {
  Accepted = 0,
  RejectedBdsB1C,      // BDS 1575.42 MHz (§12.3 "BDS 1575.42 拒收计数")
  RejectedUnknownFreq, // no table entry for (sys, freq)
  RejectedGloChannel,  // GLONASS frequency not on the FDMA grid or k outside [−7, +6]
  RejectedSystem       // Sys::Unknown / SBS / IRN (never used)
};

constexpr double kFreqTolHz = 1.0e3;

// Returns the SignalCode for (sys, freq_hz). `glo_k` (nullable) receives the GLONASS channel number for
// Sys::GLO (0 otherwise); `verdict` (nullable) says why Unknown was returned.
inline SignalCode signalFromFrequency(Sys sys, double freq_hz, int* glo_k, SignalVerdict* verdict) {
  auto near = [&](double f0) { return std::fabs(freq_hz - f0) < kFreqTolHz; };
  auto done = [&](SignalCode c, SignalVerdict v) {
    if (verdict) *verdict = v;
    return c;
  };
  if (glo_k) *glo_k = 0;
  switch (sys) {
    case Sys::GPS:
    case Sys::QZS:
      if (near(1575.42e6)) return done(SignalCode::L1C, SignalVerdict::Accepted);
      if (near(1227.60e6)) return done(SignalCode::L2X, SignalVerdict::Accepted);
      if (near(1176.45e6)) return done(SignalCode::L5X, SignalVerdict::Accepted);
      return done(SignalCode::Unknown, SignalVerdict::RejectedUnknownFreq);
    case Sys::GAL:
      if (near(1575.42e6)) return done(SignalCode::L1X, SignalVerdict::Accepted);
      if (near(1207.14e6)) return done(SignalCode::L7X, SignalVerdict::Accepted);
      if (near(1176.45e6)) return done(SignalCode::L5Q, SignalVerdict::Accepted);
      return done(SignalCode::Unknown, SignalVerdict::RejectedUnknownFreq);
    case Sys::BDS:
      if (near(1561.098e6)) return done(SignalCode::L2I, SignalVerdict::Accepted);
      if (near(1207.14e6)) return done(SignalCode::L7I, SignalVerdict::Accepted);
      if (near(1575.42e6)) return done(SignalCode::Unknown, SignalVerdict::RejectedBdsB1C);
      return done(SignalCode::Unknown, SignalVerdict::RejectedUnknownFreq);
    case Sys::GLO: {
      // G1: 1602 MHz + k·0.5625 MHz ; G2: 1246 MHz + k·0.4375 MHz (rtklib.h FREQ1_GLO/DFRQ1_GLO, FREQ2_GLO/DFRQ2_GLO)
      const double k1 = (freq_hz - 1602.0e6) / 0.5625e6;
      const double k2 = (freq_hz - 1246.0e6) / 0.4375e6;
      const double r1 = std::round(k1), r2 = std::round(k2);
      if (std::fabs(k1 - r1) * 0.5625e6 < kFreqTolHz && r1 >= -7.0 && r1 <= 6.0) {
        if (glo_k) *glo_k = static_cast<int>(r1);
        return done(SignalCode::L1C, SignalVerdict::Accepted);
      }
      if (std::fabs(k2 - r2) * 0.4375e6 < kFreqTolHz && r2 >= -7.0 && r2 <= 6.0) {
        if (glo_k) *glo_k = static_cast<int>(r2);
        return done(SignalCode::L2C, SignalVerdict::Accepted);
      }
      return done(SignalCode::Unknown, SignalVerdict::RejectedGloChannel);
    }
    default:
      return done(SignalCode::Unknown, SignalVerdict::RejectedSystem);
  }
}

// Nominal carrier frequency [Hz] of a code for a system (GLONASS needs k); 0 when the pair is not in the table.
inline double nominalFrequencyHz(Sys sys, SignalCode code, int glo_k) {
  switch (sys) {
    case Sys::GPS:
    case Sys::QZS:
      return code == SignalCode::L1C ? 1575.42e6 : code == SignalCode::L2X ? 1227.60e6 : code == SignalCode::L5X ? 1176.45e6 : 0.0;
    case Sys::GAL:
      return code == SignalCode::L1X ? 1575.42e6 : code == SignalCode::L7X ? 1207.14e6 : code == SignalCode::L5Q ? 1176.45e6 : 0.0;
    case Sys::BDS:
      return code == SignalCode::L2I ? 1561.098e6 : code == SignalCode::L7I ? 1207.14e6 : 0.0;
    case Sys::GLO:
      return code == SignalCode::L1C ? 1602.0e6 + glo_k * 0.5625e6 : code == SignalCode::L2C ? 1246.0e6 + glo_k * 0.4375e6 : 0.0;
    default:
      return 0.0;
  }
}

// True for the secondary-frequency codes that only feed the geometry-free slip check (§12.3 "第二频点仅 GF 周跳").
inline bool isGeometryFreeOnly(SignalCode code) {
  return code == SignalCode::L5X || code == SignalCode::L5Q;
}

}  // namespace livo_gnss
