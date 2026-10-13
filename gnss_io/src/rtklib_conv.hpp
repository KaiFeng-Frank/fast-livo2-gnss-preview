// SPDX-License-Identifier: BSD-3-Clause
// rtklib_conv — RTKLIB structs → livo_gnss types, shared by the two gnss_io TUs that may include rtklib.h
// (rinex_source.cpp, ubx_stream.cpp; AMENDMENTS A21). Including it anywhere else fails to compile by
// construction (RTKLIB's include path is attached to those two source files only). Owner: WP-e.
//
// The signal identity of an RTKLIB observation slot is taken from its carrier frequency through the single
// §12.3 table (livo_gnss/signal_table.hpp), exactly like the gnss_comm path, so all three paths label a
// signal the same way (T4). Tracking status bits (u-blox semantics, types.hpp StatusBit) are synthesised
// from what RTKLIB keeps:
//   bit0 psr valid       = P ≠ 0 (RTKLIB zeroes P when the receiver flags it invalid)
//   bit1 cp valid        = L ≠ 0 (idem for the phase)
//   bit2 half-cycle ok   = L ≠ 0 and not LLI_HALFC
//   bit3 half-cycle sub. = LLI_HALFA (only the UBX path sets it; RINEX LLI carries bits 0–1 only)
// lli keeps the RINEX bits (0 slip, 1 half-cycle). For RTKLIB-read ephemerides the delivered accuracy is
// RTKLIB's URA/SISA *index* (RINEX metres converted by uraindex/sisa_index), so ura_raw = sva (index semantics).
#pragma once

#include <cstdint>

#include "livo_gnss/signal_table.hpp"
#include "livo_gnss/types.hpp"
#include "signal_order.hpp"
// rtklib.h last: it defines unprefixed macros (PI, CLIGHT, ...) that must not reach other headers.
#include "rtklib.h"

namespace livo_gnss_io {
namespace rtkconv {

inline livo_gnss::GTime toGTime(const gtime_t& t) {
  livo_gnss::GTime g;
  g.sec = static_cast<int64_t>(t.time);
  g.frac = t.sec;
  return g;
}

inline gtime_t toRtk(const livo_gnss::GTime& g) {
  gtime_t t;
  t.time = static_cast<time_t>(g.sec);
  t.sec = g.frac;
  return t;
}

inline livo_gnss::Sys sysOf(int rtk_sys) {
  using livo_gnss::Sys;
  switch (rtk_sys) {
    case SYS_GPS: return Sys::GPS;
    case SYS_SBS: return Sys::SBS;
    case SYS_GLO: return Sys::GLO;
    case SYS_GAL: return Sys::GAL;
    case SYS_QZS: return Sys::QZS;
    case SYS_CMP: return Sys::BDS;
    case SYS_IRN: return Sys::IRN;
    default: return Sys::Unknown;
  }
}

inline livo_gnss::SatId satId(int sat) {
  livo_gnss::SatId id;
  int prn = 0;
  const int s = satsys(sat, &prn);
  id.sys = sysOf(s);
  id.prn = prn;
  id.rtk_sat = s ? sat : 0;
  return id;
}

// RTKLIB's RINEX reader records the code only together with the pseudorange (rinex.c decode_obsdata), so a
// main slot holding only L/D/S has CODE_NONE. The slot index fixes the band (code2idx); this representative
// code is used only to obtain that band's carrier frequency.
inline uint8_t bandCode(int rtk_sys, int slot) {
  static const uint8_t gps[3] = {CODE_L1C, CODE_L2X, CODE_L5X};
  static const uint8_t glo[3] = {CODE_L1C, CODE_L2C, CODE_L3X};
  static const uint8_t gal[3] = {CODE_L1X, CODE_L7X, CODE_L5X};
  static const uint8_t bds[3] = {CODE_L2I, CODE_L7I, CODE_L5X};
  if (slot < 0 || slot > 2) return CODE_NONE;
  switch (rtk_sys) {
    case SYS_GPS:
    case SYS_QZS: return gps[slot];
    case SYS_GLO: return glo[slot];
    case SYS_GAL: return gal[slot];
    case SYS_CMP: return bds[slot];
    default: return CODE_NONE;
  }
}

// One RTKLIB observation record → SatObs (primary signal first). `nslots` = NFREQ for RINEX (main slots: one
// code per band, RTKLIB's priority choice) or NFREQ+NEXOBS for UBX. Returns false when nothing usable remains.
inline bool toSatObs(const obsd_t& o, const nav_t* nav, int nslots, livo_gnss::SatObs* so) {
  using livo_gnss::SignalObs;
  so->sat = satId(o.sat);
  so->t_rx = toGTime(o.time);
  so->sig.clear();
  if (!so->sat.valid()) return false;
  const int sys = satsys(o.sat, nullptr);
  for (int k = 0; k < nslots && k < NFREQ + NEXOBS; ++k) {
    if (o.P[k] == 0.0 && o.L[k] == 0.0 && o.D[k] == 0.0f && o.SNR[k] == 0.0f) continue;
    uint8_t code = o.code[k];
    if (code == CODE_NONE) {
      if (k >= NFREQ) continue;
      code = bandCode(sys, k);
      if (code == CODE_NONE) continue;
    }
    const double f = sat2freq(o.sat, code, nav);
    if (!(f > 0.0)) continue;   // e.g. GLONASS without a known channel
    livo_gnss::SignalVerdict verdict = livo_gnss::SignalVerdict::Accepted;
    SignalObs s;
    s.freq_hz = f;
    s.code = livo_gnss::signalFromFrequency(so->sat.sys, f, nullptr, &verdict);
    if (verdict != livo_gnss::SignalVerdict::Accepted) continue;
    // A second code of the same band (UBX extended slot): the first slot wins (signal_order.hpp rule 1).
    if (detail::hasCode(*so, s.code)) continue;
    s.psr_m = o.P[k];
    s.cp_cyc = o.L[k];
    s.dopp_hz = static_cast<double>(o.D[k]);
    s.cn0_dbhz = static_cast<double>(o.SNR[k]);
    s.lli = static_cast<uint8_t>(o.LLI[k] & (LLI_SLIP | LLI_HALFC));
    uint8_t st = 0;
    if (o.P[k] != 0.0) st |= livo_gnss::kStatusPsrValid;
    if (o.L[k] != 0.0) {
      st |= livo_gnss::kStatusCpValid;
      if (!(o.LLI[k] & LLI_HALFC)) st |= livo_gnss::kStatusHalfCycValid;
    }
    if (o.LLI[k] & LLI_HALFA) st |= livo_gnss::kStatusHalfCycSub;
    s.status = st;
    s.psr_std_raw = static_cast<double>(o.Pstd[k]);
    s.cp_std_raw = static_cast<double>(o.Lstd[k]);
    s.dopp_std_raw = 0.0;   // RTKLIB keeps no Doppler std
    so->sig.push_back(s);
  }
  detail::primaryFirst(so);
  return !so->sig.empty();
}

inline livo_gnss::KeplerEph toKepler(const eph_t& e, livo_gnss::Source src, double t_record) {
  livo_gnss::KeplerEph k;
  k.sat = satId(e.sat);
  k.iode = e.iode;
  k.iodc = e.iodc;
  k.sva = e.sva;
  k.ura_raw = static_cast<double>(e.sva);   // index semantics (see header comment)
  k.svh = e.svh;
  k.week = e.week;
  k.code = e.code;
  k.flag = e.flag;
  k.toe = toGTime(e.toe);
  k.toc = toGTime(e.toc);
  k.ttr = toGTime(e.ttr);
  k.A = e.A;
  k.e = e.e;
  k.i0 = e.i0;
  k.OMG0 = e.OMG0;
  k.omg = e.omg;
  k.M0 = e.M0;
  k.deln = e.deln;
  k.OMGd = e.OMGd;
  k.idot = e.idot;
  k.crc = e.crc;
  k.crs = e.crs;
  k.cuc = e.cuc;
  k.cus = e.cus;
  k.cic = e.cic;
  k.cis = e.cis;
  k.toes = e.toes;
  k.fit = e.fit;
  k.f0 = e.f0;
  k.f1 = e.f1;
  k.f2 = e.f2;
  for (int i = 0; i < 6; ++i) k.tgd[i] = e.tgd[i];
  k.Adot = e.Adot;
  k.ndot = e.ndot;
  k.t_record = t_record;
  k.source = src;
  return k;
}

inline livo_gnss::GloEph toGlo(const geph_t& g, livo_gnss::Source src, double t_record) {
  livo_gnss::GloEph o;
  o.sat = satId(g.sat);
  o.iode = g.iode;
  o.frq = g.frq;
  o.svh = g.svh;
  o.sva = g.sva;
  o.age = g.age;
  o.ura_raw = static_cast<double>(g.sva);
  o.toe = toGTime(g.toe);
  o.tof = toGTime(g.tof);
  for (int i = 0; i < 3; ++i) {
    o.pos[i] = g.pos[i];
    o.vel[i] = g.vel[i];
    o.acc[i] = g.acc[i];
  }
  o.taun = g.taun;
  o.gamn = g.gamn;
  o.dtaun = g.dtaun;
  o.flags = g.flags;
  o.t_record = t_record;
  o.source = src;
  return o;
}

inline livo_gnss::Klobuchar toKlobuchar(const double ion[8], livo_gnss::Source src, double t_record) {
  livo_gnss::Klobuchar k;
  for (int i = 0; i < 4; ++i) {
    k.alpha[i] = ion[i];
    k.beta[i] = ion[4 + i];
  }
  k.t_record = t_record;
  k.source = src;
  return k;
}

}  // namespace rtkconv
}  // namespace livo_gnss_io
