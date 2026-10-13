// SPDX-License-Identifier: BSD-3-Clause
// Data helpers of the public types (frozen with the skeleton; no algorithm lives here).
#include "livo_gnss/types.hpp"

#include <cmath>

namespace livo_gnss {

GTime GTime::fromSec(double t) {
  GTime g;
  const double s = std::floor(t);
  g.sec = static_cast<int64_t>(s);
  g.frac = t - s;
  return g;
}

GTime GTime::add(double dt) const {
  // Same normalisation as RTKLIB timeadd(): fold the integer part of (frac + dt) into sec.
  GTime g = *this;
  const double tt = g.frac + dt;
  const double f = std::floor(tt);
  g.sec += static_cast<int64_t>(f);
  g.frac = tt - f;
  return g;
}

const char* sysName(Sys s) {
  switch (s) {
    case Sys::GPS: return "GPS";
    case Sys::SBS: return "SBS";
    case Sys::GLO: return "GLO";
    case Sys::GAL: return "GAL";
    case Sys::QZS: return "QZS";
    case Sys::BDS: return "BDS";
    case Sys::IRN: return "IRN";
    default: return "Unknown";
  }
}

char sysChar(Sys s) {
  switch (s) {
    case Sys::GPS: return 'G';
    case Sys::SBS: return 'S';
    case Sys::GLO: return 'R';
    case Sys::GAL: return 'E';
    case Sys::QZS: return 'J';
    case Sys::BDS: return 'C';
    case Sys::IRN: return 'I';
    default: return '?';
  }
}

const char* signalCodeName(SignalCode c) {
  switch (c) {
    case SignalCode::L1C: return "L1C";
    case SignalCode::L2X: return "L2X";
    case SignalCode::L2C: return "L2C";
    case SignalCode::L1X: return "L1X";
    case SignalCode::L7X: return "L7X";
    case SignalCode::L2I: return "L2I";
    case SignalCode::L7I: return "L7I";
    case SignalCode::L5X: return "L5X";
    case SignalCode::L5Q: return "L5Q";
    default: return "Unknown";
  }
}

const char* sourceName(Source s) {
  switch (s) {
    case Source::GnssCommMeas: return "gnss_comm_meas";
    case Source::GnssCommEphem: return "gnss_comm_ephem";
    case Source::GnssCommGloEphem: return "gnss_comm_glo_ephem";
    case Source::GnssCommIono: return "gnss_comm_iono";
    case Source::GnssCommPvt: return "gnss_comm_pvt";
    case Source::GnssCommTimePulse: return "gnss_comm_time_pulse";
    case Source::Rinex: return "rinex";
    case Source::UbxStream: return "ubx_stream";
    case Source::Synthetic: return "synthetic";
    default: return "?";
  }
}

int ObsEpoch::countSys(Sys s) const {
  int n = 0;
  for (const auto& so : sats) n += (so.sat.sys == s) ? 1 : 0;
  return n;
}

}  // namespace livo_gnss
