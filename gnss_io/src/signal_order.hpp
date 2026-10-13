// SPDX-License-Identifier: BSD-3-Clause
// signal_order — the signal-selection rules shared by the three gnss_io observation paths (gnss_comm bag bytes,
// RINEX, UBX) so that their ObsEpoch layouts agree (T4). Internal, header-only, RTKLIB-free. Owner: WP-e.
//
// 1. One signal per code, first occurrence wins (in the order the source lists them). The code is derived from
//    the carrier frequency (livo_gnss/signal_table.hpp), so two components of one band — Galileo E1-B and E1-C
//    are both 1575.42 MHz, hence both L1X — would otherwise both be carried, and sig[1] would be a copy of the
//    primary band that the core takes for a second frequency (GF slip check, DESIGN §10 L3-D6). The exporter's
//    RINEX writer keeps the first entry per band and RTKLIB's readers keep one code per band slot, so the rule
//    makes the bag path carry the same signal as the other two.
// 2. Primary (L1-class) signal first (DESIGN §12.3 "每星只取主信号(L1 类)作伪距/多普勒/TDCP"):
//      GPS/QZS L1C (1575.42) · GLO L1C (G1) · GAL L1X (E1) · BDS L2I (B1I 1561.098).
//    sig[0] is the primary signal whenever the satellite has one; the others keep their input order. A satellite
//    tracked only on a secondary band keeps that band at sig[0] — the core's L0 must check the code (no policy
//    decision is taken in the I/O layer).
#pragma once

#include <algorithm>

#include "livo_gnss/types.hpp"

namespace livo_gnss_io {
namespace detail {

// Rule 1: true when `so` already carries a signal with this code (the new one is then dropped).
inline bool hasCode(const livo_gnss::SatObs& so, livo_gnss::SignalCode c) {
  for (const livo_gnss::SignalObs& s : so.sig)
    if (s.code == c) return true;
  return false;
}

inline bool isPrimaryClass(livo_gnss::Sys sys, livo_gnss::SignalCode c) {
  using livo_gnss::SignalCode;
  using livo_gnss::Sys;
  switch (sys) {
    case Sys::GPS:
    case Sys::QZS:
    case Sys::GLO: return c == SignalCode::L1C;
    case Sys::GAL: return c == SignalCode::L1X;
    case Sys::BDS: return c == SignalCode::L2I;
    default: return false;
  }
}

// Rule 2 (after rule 1 has been applied while the signals were collected).
inline void primaryFirst(livo_gnss::SatObs* so) {
  std::stable_partition(so->sig.begin(), so->sig.end(),
                        [&](const livo_gnss::SignalObs& s) { return isPrimaryClass(so->sat.sys, s.code); });
}

}  // namespace detail
}  // namespace livo_gnss_io
