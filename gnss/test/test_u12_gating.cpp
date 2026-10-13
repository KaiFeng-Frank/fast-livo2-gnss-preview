// SPDX-License-Identifier: BSD-3-Clause
// test_u12_gating.cpp — TEST_PLAN §1 U12 (AMENDMENTS A9-2: alarm hysteresis → U17, jump detection → U9).
// WP-c gates and detectors on synthetic signals (no WP-b state machinery; the L0 health rule is WP-a's rtk::svhHealthy(),
// AMENDMENTS A37), plus the full L0 on the dev-set fixtures with the M3DGR C/N0 placeholders (AMENDMENTS A44; slices,
// decoder and IGS BRDC as in U5/U6).
//
// Statistical criteria and what they mean here:
//   * false alarm "α ± 50 %": the per-row false-rejection rate of a detector run at level α on clean
//     Gaussian data lies in [0.5α, 1.5α] (Monte-Carlo, enough rows that the Poisson width is ≪ 50 %);
//   * detection (AMENDMENTS A42, replacing A35-C2's 5.62σ): a fault of non-centrality k (E[w] = k for the faulty row)
//     is detected by the two-sided test at 3.29σ with probability Φ(k − 3.29). Ruled criteria: k ≥ 5.8σ ⇒ ≥ 99 %
//     (theory Φ(2.51) = 0.994; A35-C2's 5.62 = 3.29 + z_0.99 left no room for the iterated FDE, which measured 0.9895
//     there, C9), 5σ ⇒ ≥ 95 % (theory 95.6 %), two faults at 6σ ⇒ both ≥ 90 %, strongly correlated TDCP rows at 6σ ⇒
//     ≥ 98 %. Single-fault rates are estimated with the first-iteration w-test of the faulty row as a control variate
//     (its mean is exact, PsrGateScene::detectSingle). Where the theoretical rate sits ≥ 6σ_MC above the requirement
//     (5σ, 6σ, 8σ) the estimate is asserted directly; at the 5.8σ boundary the estimate must clear 99 % by 3σ̂ of
//     Monte-Carlo error (p̂ − 3σ̂ ≥ 0.99; σ̂ ≈ 6e-5 at n = 2e5), a statement about the gate, not about the seed.
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <map>
#include <random>
#include <set>
#include <string>
#include <vector>

#include <Eigen/Cholesky>
#include <Eigen/Dense>

#include "../../third_party/rtklib/testing/wpa_test_support.hpp"   // dev-set slices + decodeChannel (A44 fixture test)
#include "internal/update.hpp"
#include "internal/slip.hpp"
#include "internal/quality.hpp"
#include "internal/rows.hpp"
#include "internal/rtk_adapter.hpp"
#include "internal/time_offset.hpp"

using namespace livo_gnss;

namespace {

constexpr double kC = 299792458.0;
constexpr double kDeg = M_PI / 180.0;

SatId sat(Sys sys, int prn, int rtk) {
  SatId s;
  s.sys = sys;
  s.prn = prn;
  s.rtk_sat = rtk;
  return s;
}

SatObs cleanGps() {
  SatObs so;
  so.sat = sat(Sys::GPS, 7, 7);
  so.t_rx = GTime::fromSec(1.4e9);
  SignalObs sig;
  sig.freq_hz = 1575.42e6;
  sig.code = SignalCode::L1C;
  sig.psr_m = 2.2e7;
  sig.cp_cyc = 1.15e8;
  sig.dopp_hz = 1000.0;
  sig.cn0_dbhz = 45.0;
  sig.lli = 0;
  sig.status = kStatusPsrValid | kStatusCpValid | kStatusHalfCycValid | kStatusHalfCycSub;
  so.sig.push_back(sig);
  return so;
}

MaskInput cleanInput(const SatObs& so) {
  MaskInput in;
  in.obs = &so;
  in.el_deg = 45.0;
  in.az_deg = 120.0;
  in.has_eph = true;
  in.svh = 0;
  in.eph_age_s = 600.0;
  in.psr_track_ok = true;
  in.cp_track_ok = true;
  return in;
}

// WGS-84 geodetic → ECEF and the ENU rotation R_{E←N} (local, so the synthetic tests do not go through rtk_adapter).
Eigen::Vector3d geo2ecef(double lat, double lon, double h) {
  const double a = 6378137.0, f = 1.0 / 298.257223563, e2 = f * (2.0 - f);
  const double sl = std::sin(lat), cl = std::cos(lat);
  const double N = a / std::sqrt(1.0 - e2 * sl * sl);
  return Eigen::Vector3d((N + h) * cl * std::cos(lon), (N + h) * cl * std::sin(lon), (N * (1.0 - e2) + h) * sl);
}
Eigen::Matrix3d enuRot(double lat, double lon) {
  const double sl = std::sin(lat), cl = std::cos(lat), so = std::sin(lon), co = std::cos(lon);
  Eigen::Matrix3d R;
  R.col(0) = Eigen::Vector3d(-so, co, 0.0);
  R.col(1) = Eigen::Vector3d(-sl * co, -sl * so, cl);
  R.col(2) = Eigen::Vector3d(cl * co, cl * so, sl);
  return R;
}
double rangeSagnac(const Eigen::Vector3d& rs, const Eigen::Vector3d& rr, Eigen::Vector3d* e) {
  const Eigen::Vector3d d = rs - rr;
  const double r = d.norm();
  if (e) *e = d / r;
  return r + 7.2921151467e-5 * (rs.x() * rr.y() - rs.y() * rr.x()) / kC;
}

}  // namespace

// ---------------------------------------------------------------------------------------------
// L0 masks: every MaskBit fires on the matching synthetic signal and nothing else
// ---------------------------------------------------------------------------------------------
TEST(U12Gating, MaskBitsFireIndividually) {
  Config cfg;
  QualityGate gate(cfg);
  {
    SatObs so = cleanGps();
    MaskVerdict v = gate.evaluate(cleanInput(so));
    EXPECT_EQ(v.reasons, kMaskNone);
    EXPECT_TRUE(v.psr && v.dopp && v.cp);
  }
  struct Case {
    const char* name;
    uint32_t expect;
    bool psr, dopp, cp;
    void (*mutate)(SatObs&, MaskInput&, Config&);
  };
  const Case cases[] = {
      {"NaN", kMaskNaN, false, false, false, [](SatObs& s, MaskInput&, Config&) { s.sig[0].psr_m = std::nan(""); }},
      {"PsrRange", kMaskPsrRange, false, true, true, [](SatObs& s, MaskInput&, Config&) { s.sig[0].psr_m = 1.0e6; }},
      {"RangeRate", kMaskRangeRate, true, false, true, [](SatObs& s, MaskInput&, Config&) { s.sig[0].dopp_hz = 20000.0; }},
      {"Elevation", kMaskElevation, false, false, false, [](SatObs&, MaskInput& in, Config&) { in.el_deg = 5.0; }},
      {"Cn0Psr", kMaskCn0Psr, false, true, true, [](SatObs& s, MaskInput&, Config&) { s.sig[0].cn0_dbhz = 29.0; }},
      {"Cn0PsrCp", kMaskCn0Psr | kMaskCn0Cp, false, true, false, [](SatObs& s, MaskInput&, Config&) { s.sig[0].cn0_dbhz = 27.0; }},
      {"Cn0All", kMaskCn0Psr | kMaskCn0Cp | kMaskCn0Dopp, false, false, false, [](SatObs& s, MaskInput&, Config&) { s.sig[0].cn0_dbhz = 24.0; }},
      {"StatusPsr", kMaskStatusPsr, false, true, true, [](SatObs& s, MaskInput&, Config&) { s.sig[0].status &= ~kStatusPsrValid; }},
      {"StatusCp", kMaskStatusCp, true, true, false, [](SatObs& s, MaskInput&, Config&) { s.sig[0].status &= ~kStatusCpValid; }},
      {"HalfCycle", kMaskHalfCycle, true, true, false, [](SatObs& s, MaskInput&, Config&) { s.sig[0].status &= ~kStatusHalfCycValid; }},
      {"Health", kMaskHealth, false, false, false, [](SatObs&, MaskInput& in, Config&) { in.svh = 1; }},
      {"EphAge", kMaskEphAge, false, false, false, [](SatObs&, MaskInput& in, Config&) { in.eph_age_s = 8000.0; }},
      {"TrackPsr", kMaskTrackPsr, false, true, true, [](SatObs&, MaskInput& in, Config&) { in.psr_track_ok = false; }},
      {"TrackCp", kMaskTrackCp, true, true, false, [](SatObs&, MaskInput& in, Config&) { in.cp_track_ok = false; }},
      {"Excluded", kMaskExcluded, false, false, false, [](SatObs&, MaskInput&, Config& c) { c.exclude_sats = {"G07"}; }},
      {"SystemOff", kMaskSystemOff, false, false, false, [](SatObs&, MaskInput&, Config& c) { c.systems.gps = false; }},
      {"GloPsrOff", kMaskSystemOff, false, true, true,
       [](SatObs& s, MaskInput&, Config&) { s.sat = sat(Sys::GLO, 5, 37); s.sig[0].freq_hz = 1602.0e6 + 1.0 * 0.5625e6; }},
      {"BdsB1C", kMaskBdsB1C, false, false, false,
       [](SatObs& s, MaskInput&, Config&) { s.sat = sat(Sys::BDS, 20, 120); s.sig[0].freq_hz = 1575.42e6; }},
      {"GalPrn", kMaskGalPrn, false, false, false,
       [](SatObs& s, MaskInput&, Config&) { s.sat = sat(Sys::GAL, 37, 96); s.sig[0].code = SignalCode::L1X; }},
      {"StdPsr", kMaskStdPsr, false, true, true, [](SatObs& s, MaskInput&, Config&) { s.sig[0].psr_std_m = 6.0; }},
      {"StdDopp", kMaskStdDopp, true, false, true, [](SatObs& s, MaskInput&, Config&) { s.sig[0].dopp_std_hz = 2.0; }},
      {"StdCp", kMaskStdCp, true, true, false, [](SatObs& s, MaskInput&, Config&) { s.sig[0].cp_std_m = 0.03; }},
      {"NoEph", kMaskNoEph, false, false, false, [](SatObs&, MaskInput& in, Config&) { in.has_eph = false; in.svh = 1; in.eph_age_s = 9e9; }},
      {"CodeUnknown", kMaskCodeUnknown, false, false, false, [](SatObs& s, MaskInput&, Config&) { s.sig[0].freq_hz = 1.0e9; }},
  };
  for (const Case& c : cases) {
    Config cc;
    SatObs so = cleanGps();
    MaskInput in = cleanInput(so);
    c.mutate(so, in, cc);
    QualityGate g(cc);
    MaskVerdict v = g.evaluate(in);
    EXPECT_EQ(v.reasons, c.expect) << c.name << " reasons=0x" << std::hex << v.reasons;
    EXPECT_EQ(v.psr, c.psr) << c.name;
    EXPECT_EQ(v.dopp, c.dopp) << c.name;
    EXPECT_EQ(v.cp, c.cp) << c.name;
  }
  // BDS GEO detection and the psr_scale hook
  EXPECT_TRUE(QualityGate::isBdsGeo(sat(Sys::BDS, 3, 103)));
  EXPECT_TRUE(QualityGate::isBdsGeo(sat(Sys::BDS, 60, 160)));
  EXPECT_FALSE(QualityGate::isBdsGeo(sat(Sys::BDS, 21, 121)));
  EXPECT_FALSE(QualityGate::isBdsGeo(sat(Sys::GPS, 3, 3)));

  // L0 health (AMENDMENTS A37): MaskInput carries the raw broadcast svh and the gate reads it as RTKLIB satexclude()
  // does (rtk::svhHealthy) — QZSS ignores bit 0 (set on every QZSS record of the IGS BRDC), GLONASS is healthy iff
  // (svh & 9) == 0 && (svh & 6) != 4 (Bn MSB | RINEX ≥ 3.05 flags << 1), every other system needs svh = 0. A literal
  // svh ≠ 0 would mask all QZSS and the healthy RINEX ≥ 3.05 GLONASS satellites.
  {
    struct HealthCase {
      const char* name;
      Sys sys;
      int prn, rtk;
      double freq_hz;
      SignalCode code;
      int svh;
      bool healthy;
    };
    const double g1 = 1602.0e6 + 1.0 * 0.5625e6;
    const HealthCase hc[] = {
        {"GPS svh 0", Sys::GPS, 7, 7, 1575.42e6, SignalCode::L1C, 0, true},
        {"GPS svh 1", Sys::GPS, 7, 7, 1575.42e6, SignalCode::L1C, 1, false},
        {"GPS svh 32", Sys::GPS, 7, 7, 1575.42e6, SignalCode::L1C, 32, false},
        {"GPS svh -1 (no record)", Sys::GPS, 7, 7, 1575.42e6, SignalCode::L1C, -1, false},
        {"GAL svh 1", Sys::GAL, 11, 70, 1575.42e6, SignalCode::L1X, 1, false},
        {"BDS svh 1", Sys::BDS, 21, 121, 1561.098e6, SignalCode::L2I, 1, false},
        {"QZS svh 1 (L1C/B bit of the IGS BRDC)", Sys::QZS, 193, 200, 1575.42e6, SignalCode::L1C, 1, true},
        {"QZS svh 2", Sys::QZS, 193, 200, 1575.42e6, SignalCode::L1C, 2, false},
        {"QZS svh 3", Sys::QZS, 193, 200, 1575.42e6, SignalCode::L1C, 3, false},
        {"GLO svh 0", Sys::GLO, 5, 37, g1, SignalCode::L1C, 0, true},
        {"GLO svh 2 (RINEX 3.05 health flags, healthy)", Sys::GLO, 5, 37, g1, SignalCode::L1C, 2, true},
        {"GLO svh 6 (RINEX 3.05 health flags, healthy)", Sys::GLO, 5, 37, g1, SignalCode::L1C, 6, true},
        {"GLO svh 1 (Bn MSB)", Sys::GLO, 5, 37, g1, SignalCode::L1C, 1, false},
        {"GLO svh 4", Sys::GLO, 5, 37, g1, SignalCode::L1C, 4, false},
        {"GLO svh 8", Sys::GLO, 5, 37, g1, SignalCode::L1C, 8, false},
    };
    for (const HealthCase& c : hc) {
      SatObs so = cleanGps();
      so.sat = sat(c.sys, c.prn, c.rtk);
      so.sig[0].freq_hz = c.freq_hz;
      so.sig[0].code = c.code;
      MaskInput in = cleanInput(so);
      in.svh = c.svh;
      const MaskVerdict v = gate.evaluate(in);
      EXPECT_EQ((v.reasons & kMaskHealth) != 0, !c.healthy) << c.name << " reasons=0x" << std::hex << v.reasons;
      if (c.healthy) {
        EXPECT_TRUE(v.dopp && v.cp) << c.name;   // GLONASS code stays off (glo_psr), the rest of the satellite is usable
        EXPECT_EQ(v.psr, c.sys != Sys::GLO) << c.name;
      } else {
        EXPECT_FALSE(v.psr || v.dopp || v.cp) << c.name;
      }
    }
  }

  // TrackCounter: consecutive-epoch arcs, LLI restarts the carrier arc, absence resets both.
  TrackCounter tc(cfg.masks);
  ObsEpoch ep;
  ep.sats.push_back(cleanGps());
  for (int k = 1; k <= 12; ++k) {
    tc.tick(ep);
    EXPECT_EQ(tc.psrCount(ep.sats[0].sat), k);
    EXPECT_EQ(tc.cpCount(ep.sats[0].sat), k);
    EXPECT_EQ(tc.psrOk(ep.sats[0].sat), k >= cfg.masks.track_psr);
    EXPECT_EQ(tc.cpOk(ep.sats[0].sat), k >= cfg.masks.track_cp);
  }
  ep.sats[0].sig[0].lli = 1;
  tc.tick(ep);
  EXPECT_EQ(tc.psrCount(ep.sats[0].sat), 13);
  EXPECT_EQ(tc.cpCount(ep.sats[0].sat), 1);
  ObsEpoch empty;
  tc.tick(empty);
  EXPECT_EQ(tc.psrCount(ep.sats[0].sat), 0);
  EXPECT_EQ(tc.cpCount(ep.sats[0].sat), 0);
  // Epoch-stream hole (AMENDMENTS A3: 0.5 s with every topic missing): the arcs restart; a jittered 0.11 s step
  // at the 10 Hz rate does not (review n3).
  {
    TrackCounter tg(cfg.masks);
    ObsEpoch e;
    e.sats.push_back(cleanGps());
    const SatId id = e.sats[0].sat;
    double t = 1.4e9;
    for (int k = 0; k < 12; ++k, t += 0.1) {
      e.t_rx = GTime::fromSec(t);
      tg.tick(e);
    }
    EXPECT_EQ(tg.psrCount(id), 12);
    t += 0.01;   // 0.11 s step
    e.t_rx = GTime::fromSec(t);
    tg.tick(e);
    EXPECT_EQ(tg.psrCount(id), 13);
    EXPECT_EQ(tg.cpCount(id), 13);
    t += 0.5;    // hole
    e.t_rx = GTime::fromSec(t);
    tg.tick(e);
    EXPECT_EQ(tg.psrCount(id), 1);
    EXPECT_EQ(tg.cpCount(id), 1);
    EXPECT_FALSE(tg.psrOk(id));
  }

  // StdUnitJudge on synthetic grids: converted-metre psr std, cycle cp std, Hz Doppler std.
  StdUnitJudge judge;
  ObsEpoch grid;
  const double lambda = kC / 1575.42e6;
  for (int i = 0; i < 80; ++i) {
    SatObs so = cleanGps();
    so.sig[0].psr_std_raw = 0.01 * std::pow(2.0, i % 8);
    so.sig[0].cp_std_raw = 0.004 * (1 + i % 6);
    so.sig[0].dopp_std_raw = 0.002 * std::pow(2.0, i % 5);
    grid.sats.push_back(so);
  }
  judge.observe(grid);
  EXPECT_TRUE(judge.decided());
  EXPECT_EQ(judge.psrUnit(), StdUnit::Metres);
  EXPECT_EQ(judge.cpUnit(), StdUnit::Cycles);
  EXPECT_EQ(judge.doppUnit(), StdUnit::Hertz);
  judge.apply(&grid);
  EXPECT_NEAR(grid.sats[1].sig[0].psr_std_m, 0.02, 1e-12);
  EXPECT_NEAR(grid.sats[1].sig[0].cp_std_m, 0.008 * lambda, 1e-12);
  EXPECT_NEAR(grid.sats[1].sig[0].dopp_std_hz, 0.004, 1e-12);
}

// ---------------------------------------------------------------------------------------------
// AMENDMENTS A44: the C/N0 masks are receiver-dependent configuration (M3DGR placeholders 15/12/18 dB-Hz until P4)
// ---------------------------------------------------------------------------------------------
// Every C/N0 threshold is the configured one, per row class and independently of the other two: a signal exactly at the
// threshold passes, 0.01 dB-Hz below fails — for DESIGN §14's 30/25/28, the A44 placeholders and arbitrary values. A
// threshold built into the gate anywhere would fail one of these.
TEST(U12Gating, Cn0MasksFollowConfiguredThresholds) {
  struct Th {
    double psr, dopp, cp;
  };
  const Th sets[] = {{30.0, 25.0, 28.0}, {15.0, 12.0, 18.0}, {0.0, 0.0, 0.0}, {41.5, 7.25, 33.0}, {22.0, 22.0, 22.0}, {12.0, 18.0, 15.0}};
  for (const Th& th : sets) {
    Config cfg;
    cfg.masks.cn0_psr = th.psr;
    cfg.masks.cn0_dopp = th.dopp;
    cfg.masks.cn0_cp = th.cp;
    ASSERT_TRUE(validateConfig(cfg).empty()) << "C/N0 thresholds " << th.psr << "/" << th.dopp << "/" << th.cp;
    QualityGate gate(cfg);
    std::set<double> probes = {0.0, 5.0, 10.0, 20.0, 29.99, 30.0, 45.0, 60.0};
    for (double v : {th.psr, th.dopp, th.cp}) {
      probes.insert(v);
      if (v >= 0.01) probes.insert(v - 0.01);
      probes.insert(v + 0.01);
    }
    for (double c : probes) {
      SatObs so = cleanGps();
      so.sig[0].cn0_dbhz = c;
      const MaskVerdict v = gate.evaluate(cleanInput(so));
      const std::string at = std::to_string(c) + " dB-Hz vs " + std::to_string(th.psr) + "/" + std::to_string(th.dopp) + "/" + std::to_string(th.cp);
      EXPECT_EQ(v.psr, c >= th.psr) << at;
      EXPECT_EQ(v.dopp, c >= th.dopp) << at;
      EXPECT_EQ(v.cp, c >= th.cp) << at;
      EXPECT_EQ((v.reasons & kMaskCn0Psr) != 0, c < th.psr) << at;
      EXPECT_EQ((v.reasons & kMaskCn0Dopp) != 0, c < th.dopp) << at;
      EXPECT_EQ((v.reasons & kMaskCn0Cp) != 0, c < th.cp) << at;
      EXPECT_EQ(v.reasons & ~static_cast<uint32_t>(kMaskCn0Psr | kMaskCn0Dopp | kMaskCn0Cp), 0u) << at;
    }
  }
}

namespace {
const char* maskBitName(int b) {
  static const char* const kNames[] = {"NaN",       "PsrRange", "RangeRate", "Elevation", "Cn0Psr",   "Cn0Dopp",  "Cn0Cp",    "StatusPsr",
                                       "StatusCp",  "HalfCycle", "Health",   "EphAge",    "TrackPsr", "TrackCp",  "Excluded", "SystemOff",
                                       "BdsB1C",    "GalPrn",   "StdPsr",    "StdDopp",   "StdCp",    "NoEph",    "CodeUnknown"};
  return (b >= 0 && b < 23) ? kNames[b] : "?";
}
constexpr int kMaskBits = 23;

struct L0Tally {
  long sat_epochs = 0, psr = 0, dopp = 0, cp = 0, cp_in_data = 0;
  long reached = 0, cn0_mismatch = 0;                 // signals that reached the row checks / with C/N0 bits ≠ thresholds
  long epochs = 0, epochs_psr5 = 0;                   // epochs with ≥ 5 code-usable satellites (DESIGN §8 COLLECT)
  std::array<long, kMaskBits> bit{};
  std::map<char, long> unhealthy;                     // Health bit per system (RINEX letter)
  std::string first_mismatch;
  std::string table() const {
    std::string out;
    for (int b = 0; b < kMaskBits; ++b)
      if (bit[b] > 0) out += std::string(" ") + maskBitName(b) + " " + std::to_string(bit[b]);
    if (!unhealthy.empty()) {
      out += " (Health by system:";
      for (const auto& kv : unhealthy) out += std::string(" ") + kv.first + " " + std::to_string(kv.second);
      out += ")";
    }
    return out;
  }
};

// The full L0 of DESIGN §10 as the facade will run it (QualityGate::evaluate) over a slice: std units judged over the
// slice and applied (§12.6-3), TrackCounter ticked per epoch, the broadcast state of the slice's IGS BRDC at the
// receiver's own position (receiver_pvt: elevation, raw svh, |t − toe|). `cfg` carries the masks under test.
L0Tally runL0(const std::vector<ObsEpoch>& epochs, const StdUnitJudge& judge, const rtk::NavStore& nav, const Geodetic& site,
              const Config& cfg) {
  L0Tally t;
  const rtk::SatSolver solver(nav);
  const Eigen::Vector3d r_site = rtk::pos2ecef(site);
  QualityGate gate(cfg);
  Config probe_cfg = cfg;   // C/N0 thresholds above any real signal: the gate reached the row checks iff Cn0Psr fires
  probe_cfg.masks.cn0_psr = probe_cfg.masks.cn0_dopp = probe_cfg.masks.cn0_cp = 70.0;
  QualityGate probe(probe_cfg);
  TrackCounter tc(cfg.masks);
  for (ObsEpoch e : epochs) {
    judge.apply(&e);
    tc.tick(e);
    ++t.epochs;
    int n_psr = 0;
    for (const SatObs& so : e.sats) {
      const SignalObs* sig = so.primary();
      if (!sig) continue;
      ++t.sat_epochs;
      MaskInput in;
      in.obs = &so;
      in.svh = -1;
      in.eph_age_s = 9e9;
      rtk::SatState st;
      in.has_eph = so.sat.valid() && solver.satState(so.sat, e.t_rx, sig->psrValid() ? sig->psr_m : 2.2e7, &st) && st.ok;
      if (in.has_eph) {
        Eigen::Vector3d los;
        rtk::geodist(st.r_ecef, r_site, &los);
        double az = 0.0, el = 0.0;
        rtk::satAzel(site, los, &az, &el);
        in.el_deg = el / kDeg;
        in.az_deg = az / kDeg;
        in.svh = st.svh;
        const auto it = cfg.masks.eph_age_s.find(sysChar(so.sat.sys));
        double age = 9e9;
        solver.ephHealthy(so.sat, e.t_rx, it != cfg.masks.eph_age_s.end() ? it->second : 0.0, &age);
        in.eph_age_s = age;
      }
      in.psr_track_ok = tc.psrOk(so.sat);
      in.cp_track_ok = tc.cpOk(so.sat);
      in.is_bds_geo = QualityGate::isBdsGeo(so.sat);
      const MaskVerdict v = gate.evaluate(in);
      for (int b = 0; b < kMaskBits; ++b)
        if (v.reasons & (1u << b)) ++t.bit[b];
      if (v.reasons & kMaskHealth) ++t.unhealthy[sysChar(so.sat.sys)];
      t.psr += v.psr ? 1 : 0;
      t.dopp += v.dopp ? 1 : 0;
      t.cp += v.cp ? 1 : 0;
      t.cp_in_data += sig->cpValid() ? 1 : 0;
      n_psr += v.psr ? 1 : 0;
      const bool reached = (probe.evaluate(in).reasons & kMaskCn0Psr) != 0;
      const bool b_psr = (v.reasons & kMaskCn0Psr) != 0, b_dopp = (v.reasons & kMaskCn0Dopp) != 0, b_cp = (v.reasons & kMaskCn0Cp) != 0;
      const double c = sig->cn0_dbhz;
      const bool ok = reached ? (b_psr == (c < cfg.masks.cn0_psr) && b_dopp == (c < cfg.masks.cn0_dopp) && b_cp == (c < cfg.masks.cn0_cp))
                              : (!b_psr && !b_dopp && !b_cp);
      t.reached += reached ? 1 : 0;
      if (!ok && t.cn0_mismatch++ == 0) t.first_mismatch = satIdStringOf(so.sat) + " at C/N0 " + std::to_string(c);
    }
    t.epochs_psr5 += n_psr >= 5 ? 1 : 0;
  }
  return t;
}
}  // namespace

// A44 on the dev-set fixtures (the task's criterion: with the placeholders, the satellites of Grass02 and Occlusion03
// pass L0). Full L0 per slice (runL0) with the M3DGR placeholders — cn0 15/12/18 dB-Hz (A44) and the receiver-std masks
// 41 m / 16.4 Hz (A47, the P3 integration's ruling on C11) — and every other mask at its Config default (DESIGN §14/A15),
// i.e. what an M3DGR arm runs with. Criteria per slice:
//   (a) the C/N0 bits of every signal follow the configured thresholds (and DESIGN §14's 30/25/28 in the comparison run);
//   (b) the satellites pass L0: code rows and Doppler rows each have usable satellite-epochs, carrier rows too where the
//       slice has carrier phase at all (Occlusion03 has none: status bit1 never set).
// Printed per slice for the report: usable counts with the placeholders and with §14's values, and the per-bit table.
TEST(U12Gating, DevSetSatellitesPassL0WithA44Placeholders) {
  LIVO_DEVSET_SLICES(slices);
  rtk::setGalileoNavSelection(rtk::GalNavSelection::INav);   // what the core sets for ephem.gal_prefer_inav = true (A16)
  const livo_wpa_testing::Channel kPvtChannel{Source::GnssCommPvt, "gnss_comm/GnssPVTSolnMsg", "receiver_pvt", ""};
  struct Sink : MeasurementSink {
    std::vector<ObsEpoch> obs;
    std::vector<ReceiverPvt> pvt;
    void pushObservation(ObsEpoch&& e) override { obs.push_back(std::move(e)); }
    void pushEphemeris(const KeplerEph&) override {}
    void pushGloEphemeris(const GloEph&) override {}
    void pushIono(const Klobuchar&) override {}
    void pushPvt(const ReceiverPvt& p) override { pvt.push_back(p); }
  };
  for (const livo_wpa_testing::Slice& sl : slices) {
    Sink sink;
    std::string why;
    livo_wpa_testing::DecodeStats ds_obs, ds_pvt;
    livo_gnss_io::GnssCommRos1Decoder dec_obs(&satNoOf), dec_pvt(&satNoOf);
    ASSERT_TRUE(livo_wpa_testing::decodeChannel(dec_obs, sl.path, livo_wpa_testing::kMeasChannel, sink, &ds_obs, &why)) << why;
    ASSERT_TRUE(livo_wpa_testing::decodeChannel(dec_pvt, sl.path, kPvtChannel, sink, &ds_pvt, &why)) << why;
    ASSERT_FALSE(sink.obs.empty()) << sl.name;
    // the receiver's own position (first 3D fix) for elevations; the fixtures are static-to-slow ground runs
    Geodetic site;
    bool have_site = false;
    for (const ReceiverPvt& p : sink.pvt) {
      if (p.valid_fix && (p.fix_type == 3 || p.fix_type == 4)) {
        site.lat_rad = p.lat_deg * kDeg;
        site.lon_rad = p.lon_deg * kDeg;
        site.h_m = p.h_ell_m;
        have_site = true;
        break;
      }
    }
    ASSERT_TRUE(have_site) << sl.name << ": no 3D receiver_pvt fix for the elevation mask";
    const std::string brdc = livo_gnss_io::testsupport::fixtureBrdc(sl.path);
    ASSERT_FALSE(brdc.empty()) << sl.name << ": A24 brdc/ missing";
    rtk::NavStore nav;
    ASSERT_GT(nav.loadRinexNav(brdc, true, &why), 0) << why;
    StdUnitJudge judge;
    for (const ObsEpoch& e : sink.obs) judge.observe(e);
    ASSERT_TRUE(judge.decided()) << sl.name << ": std units undecided (§12.6-3)";

    Config ph;   // M3DGR placeholders (A44 C/N0; A47 receiver-std masks, config/m3dgr_avia.yaml), the rest at its default
    ph.masks.cn0_psr = 15.0;
    ph.masks.cn0_dopp = 12.0;
    ph.masks.cn0_cp = 18.0;
    ph.masks.max_psr_std_m = 41.0;   // A47 (INTERFACE_CHANGES P3-C C11): §14's 5 m / 1 Hz starve Occlusion03 completely
    ph.masks.max_dopp_std_hz = 16.4;
    const Config d14;   // DESIGN §14 30/25/28
    Config lifted = ph; // diagnostics only (not asserted): the placeholders with the receiver-std masks switched off
    lifted.masks.max_psr_std_m = lifted.masks.max_dopp_std_hz = lifted.masks.max_cp_std_m = 1e9;
    const L0Tally a = runL0(sink.obs, judge, nav, site, ph);
    const L0Tally b = runL0(sink.obs, judge, nav, site, d14);
    const L0Tally c = runL0(sink.obs, judge, nav, site, lifted);
    std::printf("[U12 A44] %s: %ld epochs, %ld satellite-epochs; usable psr/dopp/cp with 15/12/18: %ld/%ld/%ld (epochs with >= 5 "
                "code satellites %ld), with DESIGN 14's 30/25/28: %ld/%ld/%ld (%ld); 15/12/18 without the std masks (diagnostic): "
                "%ld/%ld/%ld (%ld)\n",
                sl.name.c_str(), a.epochs, a.sat_epochs, a.psr, a.dopp, a.cp, a.epochs_psr5, b.psr, b.dopp, b.cp, b.epochs_psr5, c.psr,
                c.dopp, c.cp, c.epochs_psr5);
    std::printf("[U12 A44] %s L0 bits with 15/12/18:%s\n", sl.name.c_str(), a.table().c_str());
    std::printf("[U12 A44] %s L0 bits with 30/25/28:%s\n", sl.name.c_str(), b.table().c_str());
    // (a) C/N0 semantics on real signals
    EXPECT_GT(a.reached, 0) << sl.name << ": no signal reached the row checks, the C/N0 criterion is vacuous";
    EXPECT_EQ(a.cn0_mismatch, 0) << sl.name << ": C/N0 bits differ from the configured 15/12/18, first at " << a.first_mismatch;
    EXPECT_EQ(b.cn0_mismatch, 0) << sl.name << ": C/N0 bits differ from the configured 30/25/28, first at " << b.first_mismatch;
    // (b) the satellites can pass L0 with the placeholders
    EXPECT_GT(a.psr, 0) << sl.name << ": no satellite passes L0 for a code row with the A44 placeholders; L0 bits:" << a.table();
    EXPECT_GT(a.dopp, 0) << sl.name << ": no satellite passes L0 for a Doppler row with the A44 placeholders; L0 bits:" << a.table();
    if (a.cp_in_data > 0) {
      EXPECT_GT(a.cp, 0) << sl.name << ": no satellite passes L0 for a carrier row with the A44 placeholders; L0 bits:" << a.table();
    }
  }
}

// ---------------------------------------------------------------------------------------------
// Cycle slips: 1/2/10 cycles via D4 (Doppler integration), half cycles via D2 (status) and D5 (LIVO
// prediction), false alarms of the statistical detector D5 at α ± 50 %.
// ---------------------------------------------------------------------------------------------
TEST(U12Gating, CycleSlips1_2_10AndHalf) {
  Config cfg;
  SlipDetector det(cfg.slip);
  const double lambda = kC / 1575.42e6;
  // threshold formula (DESIGN §10 L3 D4 with the j_max term)
  EXPECT_NEAR(det.d4Threshold(1.0, 0.1, lambda), std::max(0.5, 0.4 + 1.0 / (12.0 * lambda)), 1e-12);
  EXPECT_NEAR(det.d4Threshold(0.5, 0.05, lambda), 0.5, 1e-12);

  std::mt19937_64 rng(12345);
  std::normal_distribution<double> gauss(0.0, 1.0);
  std::uniform_real_distribution<double> uni(-1.0, 1.0);
  const double dt = 0.5, sigma_d = 0.05, sigma_phi_m = 0.002;
  const int trials = 20000;
  auto run = [&](double slip_cyc, bool toggle_half, double innov_sigma, int* detected, int* by_d4, int* by_d2, int* by_d5) {
    *detected = *by_d4 = *by_d2 = *by_d5 = 0;
    for (int n = 0; n < trials; ++n) {
      const double v0 = 800.0 * uni(rng), acc = 1.0 * uni(rng);        // LOS range rate and its rate
      const double D0 = -v0 / lambda, D1 = -(v0 + acc * dt) / lambda;   // D = −ρ̇/λ (approaching positive)
      const double dphi_true = (v0 * dt + 0.5 * acc * dt * dt) / lambda;  // λΦ follows the range: dΦ/dt = −D
      SignalObs a, b;
      a.status = b.status = kStatusPsrValid | kStatusCpValid | kStatusHalfCycValid;
      a.dopp_hz = D0 + sigma_d * gauss(rng);
      b.dopp_hz = D1 + sigma_d * gauss(rng);
      a.cp_cyc = 1.0e8 + sigma_phi_m / lambda * gauss(rng);
      b.cp_cyc = a.cp_cyc + dphi_true + slip_cyc + sigma_phi_m / lambda * gauss(rng);
      if (toggle_half) b.status |= kStatusHalfCycSub;
      SlipInput in;
      in.k0 = &a;
      in.k1 = &b;
      in.dt_s = dt;
      in.lambda_m = lambda;
      in.sigma_dopp_hz = sigma_d;
      if (innov_sigma > 0.0) {
        in.innov_valid = true;
        in.innov_sigma_m = innov_sigma;
        in.innov_pred_m = slip_cyc * lambda + innov_sigma * gauss(rng);
      }
      SlipVerdict v = det.check(in);
      if (v.slip) ++*detected;
      if (v.detectors & (1u << 3)) ++*by_d4;
      if (v.detectors & (1u << 1)) ++*by_d2;
      if (v.detectors & (1u << 4)) ++*by_d5;
    }
  };
  int det_, d4, d2, d5;
  for (double cyc : {1.0, 2.0, 10.0}) {
    run(cyc, false, 0.0, &det_, &d4, &d2, &d5);
    const double rate = static_cast<double>(d4) / trials;
    EXPECT_GE(rate, 0.99) << cyc << "-cycle slip D4 detection " << rate;
  }
  // no slip: D4 must stay silent (its floor of 0.5 cycles is ≫ the 0.02-cycle noise)
  run(0.0, false, 0.0, &det_, &d4, &d2, &d5);
  EXPECT_EQ(det_, 0) << "D4 false alarms " << det_;
  // half cycle: D2 (status bit toggled) is deterministic
  run(0.5, true, 0.0, &det_, &d4, &d2, &d5);
  EXPECT_EQ(d2, trials);
  // half cycle without the status flag: D5 with a 1 cm LIVO prediction (9.5 cm = 9.5σ, > 5 cm floor)
  run(0.5, false, 0.01, &det_, &d4, &d2, &d5);
  EXPECT_GE(static_cast<double>(d5) / trials, 0.99) << "half-cycle D5 detection " << static_cast<double>(d5) / trials;
  // D5 false alarms at α = 1e-3 (two-sided 3.29σ) with σ = 3 cm so the 5 cm floor is below 3.29σ
  {
    const int n = 400000;
    int fa = 0;
    SignalObs a, b;
    a.status = b.status = kStatusPsrValid | kStatusCpValid | kStatusHalfCycValid;
    for (int i = 0; i < n; ++i) {
      SlipInput in;
      in.k0 = &a;
      in.k1 = &b;
      in.dt_s = dt;
      in.lambda_m = lambda;
      in.innov_valid = true;
      in.innov_sigma_m = 0.03;
      in.innov_pred_m = 0.03 * gauss(rng);
      if (det.check(in).detectors & (1u << 4)) ++fa;
    }
    const double alpha = 1e-3;
    EXPECT_GE(fa, static_cast<int>(0.5 * alpha * n)) << "D5 false alarms " << fa;
    EXPECT_LE(fa, static_cast<int>(1.5 * alpha * n)) << "D5 false alarms " << fa;
  }
  // D1, D3, D6 single triggers
  {
    SignalObs a, b, a2, b2;
    a.status = b.status = a2.status = b2.status = kStatusPsrValid | kStatusCpValid | kStatusHalfCycValid;
    a.cp_cyc = b.cp_cyc = 1e8;
    a2.cp_cyc = b2.cp_cyc = 7.8e7;
    SlipInput in;
    in.k0 = &a;
    in.k1 = &b;
    in.dt_s = 0.5;
    in.lambda_m = lambda;
    EXPECT_FALSE(det.check(in).slip);
    b.lli = 1;
    EXPECT_EQ(det.check(in).detectors, 1u << 0);
    b.lli = 0;
    in.dt_s = 2.0;
    EXPECT_EQ(det.check(in).detectors, 1u << 2);
    in.dt_s = 0.5;
    in.k0_second = &a2;
    in.k1_second = &b2;
    in.lambda2_m = kC / 1227.60e6;
    EXPECT_FALSE(det.check(in).slip);
    b2.cp_cyc += 0.06 / in.lambda2_m;   // GF change 6 cm > 5 cm
    EXPECT_EQ(det.check(in).detectors, 1u << 5);
  }
  // L2 integer-ms clock jump inside the pair: Φ jumps by c·1 ms/λ (≈ 1.58e6 cycles), D does not. Without the
  // correction D4 fires on every satellite; with SlipInput::clock_jump_m the pair stays usable (review m5).
  {
    const double jump = 299792.458;
    SignalObs a, b;
    a.status = b.status = kStatusPsrValid | kStatusCpValid | kStatusHalfCycValid;
    a.dopp_hz = b.dopp_hz = 1000.0;                    // constant range rate −λD
    a.cp_cyc = 1.0e8;
    b.cp_cyc = a.cp_cyc - 1000.0 * dt + jump / lambda;   // dΦ/dt = −D, plus the receiver's jump
    SlipInput in;
    in.k0 = &a;
    in.k1 = &b;
    in.dt_s = dt;
    in.lambda_m = lambda;
    in.sigma_dopp_hz = sigma_d;
    EXPECT_EQ(det.check(in).detectors, 1u << 3);
    in.clock_jump_m = jump;
    EXPECT_FALSE(det.check(in).slip);
    in.clock_jump_m = -jump;   // wrong sign is still a slip
    EXPECT_EQ(det.check(in).detectors, 1u << 3);
  }
}

// ---------------------------------------------------------------------------------------------
// L4 on TDCP rows: the common Δclk is a nuisance parameter (W = S⁻¹ − S⁻¹1(1ᵀS⁻¹1)⁻¹1ᵀS⁻¹). Clean rows whose
// prediction uncertainty comes from the yaw state (σ_ψ = 0.02° and 2°, pair displacement 1 m) or from the relative
// clone/antenna position (30 mm per axis) must be rejected at α ± 50 % per row. Review M1 measured 6.8e-3 (1°) /
// 2.8e-2 (2°) per row with the robust-Δclk-then-S⁻¹ form. Detection of a 6σ single fault: ≥ 98 % here — when S
// is dominated by a rank-3 state term the rows are strongly correlated, the iterated FDE sometimes removes a
// correlated clean row first and the fault's non-centrality shrinks with the lost redundancy (measured 0.989 for
// the 30 mm case vs 0.997 for R-dominated pseudorange rows); registered with request C2 (INTERFACE_CHANGES).
// ---------------------------------------------------------------------------------------------
TEST(U12Gating, TdcpGateFalseAlarmWithNuisanceClock) {
  Config cfg;
  JointUpdater upd(cfg);
  const double alpha = cfg.rejection.alpha_w;
  std::mt19937_64 rng(2024);
  std::normal_distribution<double> gauss(0.0, 1.0);
  std::uniform_real_distribution<double> uni(0.0, 1.0);
  const int m = 8;
  const double rsd = 0.008;
  struct Case {
    const char* name;
    double sigma_psi_deg, rel_sd_m;
  };
  const Case cases[] = {{"sigma_psi 0.02 deg", 0.02, 0.0}, {"sigma_psi 2 deg", 2.0, 0.0}, {"relative position 30 mm", 0.0, 0.03}};
  for (const Case& c : cases) {
    // columns: yaw (1) and relative position (3): H_ψ = −uᵀ[e_z]×(M Δy), H_rel = −uᵀ (t cancels between clone and current)
    const int N = HostIdx::dim + 4, yaw = HostIdx::dim, rel = HostIdx::dim + 1;
    Eigen::MatrixXd Sigma = Eigen::MatrixXd::Identity(N, N) * 1e-12;
    const double sp = c.sigma_psi_deg * kDeg;
    Sigma(yaw, yaw) = std::max(sp * sp, 1e-12);
    Sigma.block<3, 3>(rel, rel) = Eigen::Matrix3d::Identity() * std::max(c.rel_sd_m * c.rel_sd_m, 1e-12);
    const int epochs = 30000;
    long rej = 0, rows = 0, skipped = 0, clean_epochs = 0, det = 0, det_trials = 0;
    const Eigen::Vector3d ez(0.0, 0.0, 1.0);
    for (int e = 0; e < epochs; ++e) {
      RowSet set;
      set.cls = MeasClass::Tdcp;
      const double th = 2.0 * M_PI * uni(rng);
      const Eigen::Vector3d dy(std::cos(th), std::sin(th), 0.0);
      const double dpsi = sp * gauss(rng);
      const Eigen::Vector3d drel = c.rel_sd_m * Eigen::Vector3d(gauss(rng), gauss(rng), gauss(rng));
      Eigen::MatrixXd H = Eigen::MatrixXd::Zero(m, N);
      for (int j = 0; j < m; ++j) {
        const double el = (15.0 + 65.0 * uni(rng)) * kDeg, az = 2.0 * M_PI * uni(rng);
        const Eigen::Vector3d u(std::cos(el) * std::sin(az), std::cos(el) * std::cos(az), std::sin(el));
        const double hpsi = -u.dot(ez.cross(dy));
        MeasRow r;
        r.cls = MeasClass::Tdcp;
        r.sat = sat(Sys::GPS, j + 1, j + 1);
        r.pair_id = 1;
        r.H = Eigen::RowVectorXd::Zero(N);
        r.H(yaw) = hpsi;
        r.H.segment<3>(rel) = -u.transpose();
        r.R = rsd * rsd;
        r.nu = 0.37 + hpsi * dpsi - u.dot(drel) + rsd * gauss(rng);   // common Δclk + state effect + noise
        H.row(j) = r.H;
        set.rows.push_back(r);
      }
      set.R_full = Eigen::MatrixXd::Identity(m, m) * rsd * rsd;
      set.has_full_R = true;
      // odd epochs: one row carries a fault of non-centrality 6 in the nuisance-parameter statistic
      const bool inject = (e % 2 == 1);
      int bad = -1;
      if (inject) {
        const Eigen::MatrixXd S = H * Sigma * H.transpose() + set.R_full;
        const Eigen::MatrixXd Si = S.inverse();
        const Eigen::VectorXd s1 = Si * Eigen::VectorXd::Ones(m);
        const Eigen::MatrixXd W = Si - s1 * s1.transpose() / s1.sum();
        bad = static_cast<int>(uni(rng) * m) % m;
        set.rows[bad].nu += 6.0 / std::sqrt(W(bad, bad));
      }
      GateResult g = upd.gateClass(set, Sigma, cfg.rejection.min_meas.tdcp, alpha, cfg.rejection.alpha_nis, cfg.rejection.max_reject_frac);
      if (!inject) {
        ++clean_epochs;
        rej += g.n_in - g.n_out;
        rows += m;
        if (g.class_skipped) ++skipped; else EXPECT_EQ(g.dof, g.n_out - 1);
      } else {
        ++det_trials;
        if (set.rows[bad].reason != RejectReason::NotRejected) ++det;
      }
    }
    const double fa = static_cast<double>(rej) / rows, pd = static_cast<double>(det) / det_trials;
    std::cout << "[U12] TDCP L4 (" << c.name << "): per-row false rejection " << fa << " (α = " << alpha << "), class skipped "
              << static_cast<double>(skipped) / clean_epochs << ", 6σ single-fault detection " << pd << "\n";
    EXPECT_GE(fa, 0.5 * alpha) << c.name;
    EXPECT_LE(fa, 1.5 * alpha) << c.name;
    EXPECT_LE(static_cast<double>(skipped) / clean_epochs, 0.01) << c.name;
    EXPECT_GE(pd, 0.98) << c.name;
  }
}

// ---------------------------------------------------------------------------------------------
// TimeOffsetMonitor: an injected ±50 ms step is flagged within one window, never before
// ---------------------------------------------------------------------------------------------
TEST(U12Gating, TimeOffsetStep) {
  for (double step_s : {0.05, -0.05}) {
    Config cfg;
    TimeOffsetMonitor mon(cfg.time);
    std::mt19937_64 rng(777);
    std::normal_distribution<double> gauss(0.0, 1.0);
    std::uniform_real_distribution<double> uni(-1.0, 1.0);
    const double t_step = 100.0, sigma = 0.05;
    double t_first_warn = -1.0;
    bool warned_before = false;
    TimeOffsetEstimate last;
    for (int k = 0; k <= 2000; ++k) {
      const double t = 0.1 * k;
      const double acc = 1.0 * std::sin(0.5 * t);           // vehicle acceleration magnitude profile
      const double dtoff = t >= t_step ? step_s : 0.0;
      std::vector<TimeOffsetSample> rows(8);
      for (auto& r : rows) {
        r.x = uni(rng) * acc;                                 // u_jᵀα_A
        r.nu = -dtoff * r.x + sigma * gauss(rng);             // innovation carries −δt·x
        r.var = sigma * sigma;
      }
      mon.addEpoch(t, rows);
      last = mon.estimate();
      if (last.warn) {
        if (t < t_step) warned_before = true;
        if (t_first_warn < 0.0) t_first_warn = t;
      }
    }
    EXPECT_FALSE(warned_before) << "TOFF_WARN before the step";
    ASSERT_GT(t_first_warn, 0.0) << "step never flagged";
    EXPECT_LE(t_first_warn, t_step + cfg.time.toff_monitor.window_s) << "flagged only after one window";
    EXPECT_TRUE(last.valid);
    EXPECT_NEAR(last.offset_s, step_s, 0.01);
    EXPECT_LT(last.sigma_s, 0.005);
  }
  // no excitation (zero acceleration) ⇒ no estimate, no warning
  {
    Config cfg;
    TimeOffsetMonitor mon(cfg.time);
    for (int k = 0; k < 300; ++k) {
      std::vector<TimeOffsetSample> rows(6);
      for (auto& r : rows) { r.x = 0.0; r.nu = 0.01; r.var = 1e-4; }
      mon.addEpoch(0.1 * k, rows);
    }
    EXPECT_FALSE(mon.estimate().valid);
    EXPECT_FALSE(mon.estimate().warn);
  }
}

// ---------------------------------------------------------------------------------------------
// L4: Baarda w + epoch NIS + iterated FDE on synthetic pseudorange-like rows
// ---------------------------------------------------------------------------------------------
namespace {
// Pseudorange-like L4 scene: m = 8 rows, LIO position σ 1 cm, registration σ 1 m per axis, clock σ 3 m, drift σ 0.1 m/s,
// R = 1 m². S is dominated by the rank-4 state term (≈ 10 m² against 1 m²), so the rows' w statistics are correlated.
struct PsrGateScene {
  static constexpr int m = 8;
  static constexpr int N = HostIdx::dim + 5;   // trans(3) + clk(2)
  Config cfg;
  JointUpdater upd;
  Eigen::MatrixXd Sigma;
  std::mt19937_64 rng;
  std::normal_distribution<double> gauss{0.0, 1.0};
  std::uniform_real_distribution<double> uni{0.0, 1.0};
  explicit PsrGateScene(unsigned seed) : upd(cfg), rng(seed) {
    Sigma = Eigen::MatrixXd::Identity(N, N) * 1e-6;
    Sigma.block<3, 3>(HostIdx::pos, HostIdx::pos) = Eigen::Matrix3d::Identity() * 1e-4;
    Sigma.block<3, 3>(HostIdx::dim, HostIdx::dim) = Eigen::Matrix3d::Identity() * 1.0;
    Sigma(HostIdx::dim + 3, HostIdx::dim + 3) = 9.0;
    Sigma(HostIdx::dim + 4, HostIdx::dim + 4) = 0.01;
  }
  RowSet makeSet(Eigen::MatrixXd* H, Eigen::MatrixXd* S) {
    RowSet set;
    set.cls = MeasClass::Psr;
    H->setZero(m, N);
    for (int i = 0; i < m; ++i) {
      const double el = (15.0 + 65.0 * uni(rng)) * kDeg, az = 2.0 * M_PI * uni(rng);
      const Eigen::Vector3d u(std::cos(el) * std::sin(az), std::cos(el) * std::cos(az), std::sin(el));
      MeasRow r;
      r.cls = MeasClass::Psr;
      r.sat = sat(Sys::GPS, i + 1, i + 1);
      r.H = Eigen::RowVectorXd::Zero(N);
      r.H.segment<3>(HostIdx::pos) = -u.transpose();
      r.H.segment<3>(HostIdx::dim) = -u.transpose();
      r.H(HostIdx::dim + 3) = 1.0;
      r.R = 1.0;
      H->row(i) = r.H;
      set.rows.push_back(r);
    }
    *S = *H * Sigma * H->transpose() + Eigen::MatrixXd::Identity(m, m);
    return set;
  }
  Eigen::VectorXd sample(const Eigen::MatrixXd& S) {
    Eigen::LLT<Eigen::MatrixXd> llt(S);
    Eigen::VectorXd z(m);
    for (int i = 0; i < m; ++i) z(i) = gauss(rng);
    return Eigen::VectorXd(llt.matrixL() * z);
  }
  GateResult gate(RowSet& set, double max_reject_frac = -1.0) {
    return upd.gateClass(set, Sigma, cfg.rejection.min_meas.psr, cfg.rejection.alpha_w, cfg.rejection.alpha_nis,
                         max_reject_frac >= 0.0 ? max_reject_frac : cfg.rejection.max_reject_frac);
  }
  // Detection of one fault of non-centrality k (E[w] = k) on a random row: "detected" = that row rejected by L4.
  // `cv` is the control-variate estimate: the first-iteration w-test of the faulty row alone, D_w = 1[|w_i| > 3.29],
  // has the exact mean Φ(k − 3.29) + Φ(−k − 3.29) (w_i ~ N(k, 1) for any geometry), so p = E[D_w] + E[D_L4 − D_w]
  // and only the rare disagreements of the iterated gate with that single test (NIS assist, swamping by correlated
  // clean rows) carry Monte-Carlo noise — ≈ 2.5× smaller σ than the plain rate at the same n.
  struct Detection {
    double plain = 0.0, cv = 0.0, cv_sigma = 0.0, w_alone = 0.0, swamp = 0.0;
  };
  Detection detectSingle(double k, int trials) {
    const double thr = normalQuantile(1.0 - 0.5 * cfg.rejection.alpha_w);
    Detection d;
    d.w_alone = 0.5 * std::erfc((thr - k) / std::sqrt(2.0)) + 0.5 * std::erfc((thr + k) / std::sqrt(2.0));
    long det = 0, swamped = 0;
    double s1 = 0.0, s2 = 0.0;
    for (int e = 0; e < trials; ++e) {
      Eigen::MatrixXd H, S;
      RowSet set = makeSet(&H, &S);
      const Eigen::MatrixXd Sinv = S.inverse();
      Eigen::VectorXd nu = sample(S);
      const int i = static_cast<int>(uni(rng) * m) % m;
      nu(i) += k / std::sqrt(Sinv(i, i));   // E[w_i] = k
      for (int j = 0; j < m; ++j) set.rows[j].nu = nu(j);
      const double w_i = (Sinv * nu)(i) / std::sqrt(Sinv(i, i));
      gate(set);
      const int d_l4 = set.rows[i].reason != RejectReason::NotRejected ? 1 : 0;
      const double diff = static_cast<double>(d_l4 - (std::fabs(w_i) > thr ? 1 : 0));
      det += d_l4;
      s1 += diff;
      s2 += diff * diff;
      for (int j = 0; j < m; ++j)
        if (j != i && set.rows[j].reason != RejectReason::NotRejected) {
          ++swamped;
          break;
        }
    }
    const double n = static_cast<double>(trials), mean = s1 / n;
    d.plain = static_cast<double>(det) / n;
    d.cv = d.w_alone + mean;
    d.cv_sigma = std::sqrt(std::max(0.0, s2 / n - mean * mean) / n);
    d.swamp = static_cast<double>(swamped) / n;
    return d;
  }
  // Two faults of non-centrality k on two random rows: both rejected.
  double detectDouble(double k, int trials) {
    int both = 0;
    for (int e = 0; e < trials; ++e) {
      Eigen::MatrixXd H, S;
      RowSet set = makeSet(&H, &S);
      const Eigen::MatrixXd Sinv = S.inverse();
      Eigen::VectorXd nu = sample(S);
      const int i = static_cast<int>(uni(rng) * m) % m;
      const int j = (i + 1 + static_cast<int>(uni(rng) * (m - 1))) % m;
      nu(i) += k / std::sqrt(Sinv(i, i));
      nu(j) += k / std::sqrt(Sinv(j, j));
      for (int q = 0; q < m; ++q) set.rows[q].nu = nu(q);
      gate(set);
      if (set.rows[i].reason != RejectReason::NotRejected && set.rows[j].reason != RejectReason::NotRejected) ++both;
    }
    return static_cast<double>(both) / trials;
  }
};
}  // namespace

TEST(U12Gating, SingleAndMultiNlosDetected) {
  PsrGateScene sc(4242);
  const int m = PsrGateScene::m;
  const double alpha = sc.cfg.rejection.alpha_w;
  // (a) false alarms on clean data: per-row rejection rate within α ± 50 %
  {
    const int epochs = 40000;
    long rejected = 0, total = 0;
    for (int e = 0; e < epochs; ++e) {
      Eigen::MatrixXd H, S;
      RowSet set = sc.makeSet(&H, &S);
      const Eigen::VectorXd nu = sc.sample(S);
      for (int i = 0; i < m; ++i) set.rows[i].nu = nu(i);
      GateResult g = sc.gate(set);
      total += m;
      rejected += (g.n_in - g.n_out);
      EXPECT_FALSE(g.class_skipped);
    }
    const double fa = static_cast<double>(rejected) / total;
    std::cout << "[U12] L4 per-row false rejection rate on clean data: " << fa << " (α = " << alpha << ")\n";
    EXPECT_GE(fa, 0.5 * alpha) << "false alarm rate " << fa;
    EXPECT_LE(fa, 1.5 * alpha) << "false alarm rate " << fa;
  }
  // (b) single fault of non-centrality k (A42): 5σ ⇒ ≥ 95 % and, as instances of k ≥ 5.8σ ⇒ ≥ 99 %, 6σ and 8σ.
  //     Theory (w-test alone) 0.9563 / 0.9966 / 1 − 1e-6 is ≥ 6σ_MC above each requirement, so they are asserted
  //     directly on the control-variate estimate; the boundary k = 5.8 has its own test below.
  const PsrGateScene::Detection p5 = sc.detectSingle(5.0, 50000);
  const PsrGateScene::Detection p6 = sc.detectSingle(6.0, 20000);
  const PsrGateScene::Detection p8 = sc.detectSingle(8.0, 5000);
  std::cout << "[U12] single fault detection (L4 / w-test alone): 5σ " << p5.cv << " ± " << p5.cv_sigma << " / " << p5.w_alone
            << ", 6σ " << p6.cv << " ± " << p6.cv_sigma << " / " << p6.w_alone << ", 8σ " << p8.cv << " / " << p8.w_alone
            << " (plain rates " << p5.plain << ", " << p6.plain << ", " << p8.plain << ")\n";
  EXPECT_GE(p5.cv, 0.95) << "5σ single-fault detection (A42: ≥ 95 %)";
  EXPECT_GE(p6.cv, 0.99) << "6σ single-fault detection (A42: k ≥ 5.8σ ⇒ ≥ 99 %)";
  EXPECT_GE(p8.cv, 0.999) << "8σ single-fault detection";
  EXPECT_LE(std::max(p5.swamp, std::max(p6.swamp, p8.swamp)), 0.03) << "clean rows rejected alongside the fault";
  // (c) two simultaneous faults (same sign, masking each other): both rows rejected (A42: 6σ ⇒ ≥ 90 %)
  const double d8 = sc.detectDouble(8.0, 4000);
  const double d6 = sc.detectDouble(6.0, 10000);
  std::cout << "[U12] two simultaneous faults both detected: 6σ " << d6 << ", 8σ " << d8 << "\n";
  EXPECT_GE(d8, 0.99) << "two 8σ faults both detected " << d8;
  EXPECT_GE(d6, 0.90) << "two 6σ faults both detected " << d6 << " (A42: ≥ 90 %)";
  // (d) majority rule: 5 of 8 rows grossly wrong ⇒ class skipped, nothing accepted; every row is either rejected by
  //     the w/NIS test or — if still standing when > max_reject_frac have been rejected — labelled Majority. (Whether
  //     any row is left standing depends on how the five gross errors combine; the labelling itself is pinned down
  //     deterministically in SkippedClassLabelsBelowMinOrMajority.)
  {
    Eigen::MatrixXd H, S;
    RowSet set = sc.makeSet(&H, &S);
    Eigen::VectorXd nu = sc.sample(S);
    for (int i = 0; i < 5; ++i) nu(i) += 50.0 * (i % 2 ? 1.0 : -1.0);
    for (int q = 0; q < m; ++q) set.rows[q].nu = nu(q);
    GateResult g = sc.gate(set);
    EXPECT_TRUE(g.class_skipped);
    EXPECT_EQ(g.n_out, 0);
    int n_majority = 0, n_rejected = 0;
    for (const auto& r : set.rows) {
      EXPECT_FALSE(r.accepted);
      if (r.reason == RejectReason::Majority) ++n_majority;
      else if (r.reason == RejectReason::WTest || r.reason == RejectReason::Nis) ++n_rejected;
      else ADD_FAILURE() << "unexpected reason " << rejectReasonName(r.reason);
    }
    EXPECT_EQ(n_rejected, g.n_rejected_w + g.n_rejected_nis);
    EXPECT_EQ(n_rejected + n_majority, m);
    if (n_majority > 0) {
      EXPECT_GT(n_rejected, static_cast<int>(sc.cfg.rejection.max_reject_frac * m));
    }
  }
}

// A42 at its boundary: a single fault of k = 5.8σ must be detected ≥ 99 % (and so every larger k: the rate increases
// with k). The w-test alone gives Φ(5.8 − 3.29) = Φ(2.51) = 0.994; the iterated FDE loses a little of it to swamping (a
// correlated clean row rejected first, after which the fault's w falls under the threshold; 0.058 points at 5.62σ, C9)
// and gains a little from the NIS assist. Criterion (file header): p̂ − 3σ̂ ≥ 0.99 on the control-variate estimate.
TEST(U12Gating, SingleFaultDetectionAtRuledBoundary) {
  PsrGateScene sc(5800);
  const int n = 200000;
  const double k = 5.8;
  const PsrGateScene::Detection d = sc.detectSingle(k, n);
  std::cout << "[U12] " << k << "σ single-fault detection: L4 " << d.cv << " ± " << d.cv_sigma << " (control variate; plain " << d.plain
            << ", n " << n << "), w-test alone " << d.w_alone << ", L4 − w-test " << d.cv - d.w_alone << ", swamping " << d.swamp << "\n";
  EXPECT_NEAR(d.w_alone, 0.994, 5e-4) << "Φ(5.8 − 3.29) of A42";
  EXPECT_GE(d.cv - 3.0 * d.cv_sigma, 0.99) << k << "σ single-fault detection " << d.cv << " ± " << d.cv_sigma
                                           << " is not ≥ 99 % beyond Monte-Carlo error (A42: k ≥ 5.8σ ⇒ ≥ 99 %)";
  EXPECT_LE(d.swamp, 0.03);
}

// ---------------------------------------------------------------------------------------------
// L4 class skipping (AMENDMENTS A35): fewer than m_min rows left — from the start, or after rejections (a TDCP pair
// reduced to one row) — labels the rows still standing BelowMin; more than max_reject_frac rejected labels them Majority.
// ---------------------------------------------------------------------------------------------
TEST(U12Gating, SkippedClassLabelsBelowMinOrMajority) {
  Config cfg;
  JointUpdater upd(cfg);
  const double alpha = cfg.rejection.alpha_w;
  std::mt19937_64 rng(77);
  std::uniform_real_distribution<double> uni(0.0, 1.0);
  const int N = HostIdx::dim + 3;
  Eigen::MatrixXd Sigma = Eigen::MatrixXd::Identity(N, N) * 1e-6;
  Sigma.block<3, 3>(HostIdx::dim, HostIdx::dim) = Eigen::Matrix3d::Identity() * 1e-4;
  auto makeRows = [&](MeasClass cls, int m, double r_var) {
    RowSet set;
    set.cls = cls;
    for (int j = 0; j < m; ++j) {
      const double el = (20.0 + 60.0 * uni(rng)) * kDeg, az = 2.0 * M_PI * uni(rng);
      const Eigen::Vector3d u(std::cos(el) * std::sin(az), std::cos(el) * std::cos(az), std::sin(el));
      MeasRow r;
      r.cls = cls;
      r.sat = sat(Sys::GPS, j + 1, j + 1);
      r.pair_id = cls == MeasClass::Tdcp ? 1 : -1;
      r.H = Eigen::RowVectorXd::Zero(N);
      r.H.segment<3>(HostIdx::dim) = -u.transpose();
      r.R = r_var;
      r.nu = 0.0;
      set.rows.push_back(r);
    }
    return set;
  };
  // (a) a pseudorange class below its minimum from the start: nothing is tested or rejected, both rows BelowMin
  {
    RowSet set = makeRows(MeasClass::Psr, 2, 1.0);
    GateResult g = upd.gateClass(set, Sigma, 3, alpha, cfg.rejection.alpha_nis, cfg.rejection.max_reject_frac);
    EXPECT_TRUE(g.class_skipped);
    EXPECT_EQ(g.n_in, 2);
    EXPECT_EQ(g.n_out, 0);
    EXPECT_EQ(g.n_rejected_w + g.n_rejected_nis, 0);
    for (const auto& r : set.rows) {
      EXPECT_EQ(r.reason, RejectReason::BelowMin) << rejectReasonName(r.reason);
      EXPECT_FALSE(r.accepted);
    }
  }
  // (b) a two-satellite TDCP pair with one faulty row (min_meas.tdcp = 2): with dof 1 the two |w| are equal, one row
  //     is rejected and the other is left alone below the minimum ⇒ BelowMin (1 of 2 rejected is not a majority)
  {
    ASSERT_EQ(cfg.rejection.min_meas.tdcp, 2);
    RowSet set = makeRows(MeasClass::Tdcp, 2, 0.008 * 0.008);
    set.R_full = Eigen::MatrixXd::Identity(2, 2) * 0.008 * 0.008;
    set.has_full_R = true;
    set.rows[0].nu = 0.37 + 0.5;   // common Δclk + a 0.5 m fault
    set.rows[1].nu = 0.37;
    GateResult g = upd.gateClass(set, Sigma, cfg.rejection.min_meas.tdcp, alpha, cfg.rejection.alpha_nis, cfg.rejection.max_reject_frac);
    EXPECT_TRUE(g.class_skipped);
    EXPECT_EQ(g.n_out, 0);
    EXPECT_EQ(g.n_rejected_w + g.n_rejected_nis, 1);
    int below = 0, tested_out = 0;
    for (const auto& r : set.rows) {
      EXPECT_FALSE(r.accepted);
      if (r.reason == RejectReason::BelowMin) ++below;
      if (r.reason == RejectReason::WTest || r.reason == RejectReason::Nis) ++tested_out;
    }
    EXPECT_EQ(below, 1);
    EXPECT_EQ(tested_out, 1);
  }
  // (c) three TDCP rows, two faulty: the second rejection leaves one row (< 2) ⇒ BelowMin takes precedence over the
  //     majority count (2 of 3) — the class stopped because too few rows were left to test
  {
    RowSet set = makeRows(MeasClass::Tdcp, 3, 0.008 * 0.008);
    set.R_full = Eigen::MatrixXd::Identity(3, 3) * 0.008 * 0.008;
    set.has_full_R = true;
    set.rows[0].nu = 0.37 + 0.8;
    set.rows[1].nu = 0.37 - 0.6;
    set.rows[2].nu = 0.37;
    GateResult g = upd.gateClass(set, Sigma, cfg.rejection.min_meas.tdcp, alpha, cfg.rejection.alpha_nis, cfg.rejection.max_reject_frac);
    EXPECT_TRUE(g.class_skipped);
    EXPECT_EQ(g.n_rejected_w + g.n_rejected_nis, 2);
    int below = 0;
    for (const auto& r : set.rows) below += (r.reason == RejectReason::BelowMin);
    EXPECT_EQ(below, 1);
  }
  // (d) Majority: three consistent pseudorange rows and one gross error with max_reject_frac = 0.2 — the error is
  //     rejected, the three survivors pass the test, and 1 of 4 > 0.2 · 4 makes the class sit out: they are Majority
  {
    RowSet set = makeRows(MeasClass::Psr, 4, 1.0);
    set.rows[2].nu = 50.0;
    GateResult g = upd.gateClass(set, Sigma, cfg.rejection.min_meas.psr, alpha, cfg.rejection.alpha_nis, 0.2);
    EXPECT_TRUE(g.class_skipped);
    EXPECT_EQ(g.n_out, 0);
    EXPECT_EQ(set.rows[2].reason, RejectReason::WTest) << rejectReasonName(set.rows[2].reason);
    for (int j : {0, 1, 3}) {
      EXPECT_EQ(set.rows[j].reason, RejectReason::Majority) << "row " << j << ": " << rejectReasonName(set.rows[j].reason);
      EXPECT_FALSE(set.rows[j].accepted);
    }
    // the same set under the default 50 % rule is simply accepted (the class is updated with the three rows)
    RowSet again = makeRows(MeasClass::Psr, 4, 1.0);
    again.rows[2].nu = 50.0;
    GateResult g2 = upd.gateClass(again, Sigma, cfg.rejection.min_meas.psr, alpha, cfg.rejection.alpha_nis, cfg.rejection.max_reject_frac);
    EXPECT_FALSE(g2.class_skipped);
    EXPECT_EQ(g2.n_out, 3);
  }
  EXPECT_STREQ(rejectReasonName(RejectReason::BelowMin), "below_min");
  EXPECT_STREQ(rejectReasonName(RejectReason::Majority), "majority");
}

// ---------------------------------------------------------------------------------------------
// After an integer-ms clock jump k, the TDCP rows stay usable once k·c·1 ms is removed from λΔΦ.
// Also: the projected rows are invariant to the common jump even without the correction (Δclk is a
// nuisance), and the gate accepts the pair.
// ---------------------------------------------------------------------------------------------
TEST(U12Gating, TdcpUsableAfterClockJumpCorrection) {
  Config cfg;
  NoiseModel noise(cfg);
  RowBuilder rb(cfg, noise);
  const double lat = 30.0 * kDeg, lon = 110.0 * kDeg;
  AnchorFrame anchor;
  anchor.r0_ecef = geo2ecef(lat, lon, 50.0);
  anchor.R_EN = enuRot(lat, lon);
  anchor.valid = true;
  const double lambda = kC / 1575.42e6;
  const int m = 8, n_aug = 3, N = HostIdx::dim + n_aug;
  std::mt19937_64 rng(99);
  std::uniform_real_distribution<double> uni(0.0, 1.0);
  std::normal_distribution<double> gauss(0.0, 1.0);

  const Eigen::Vector3d c_true(10.0, 5.0, 1.0), a_true = c_true + Eigen::Vector3d(0.6, 0.3, 0.02);
  const Eigen::Vector3d c_err(0.01, -0.02, 0.015), a_err(0.02, -0.01, 0.03);
  const Eigen::Vector3d c_hat = c_true + c_err, a_hat = a_true + a_err;
  const double dt_pair = 0.5, dclk_true = 0.3;
  const double jump = 299792.458;   // c·1 ms

  auto build = [&](double phase_jump_m, double clock_jump_m, RowSet* set, std::vector<double>* nu_pred) {
    std::mt19937_64 r2(31);
    std::normal_distribution<double> g2(0.0, 1.0);
    std::uniform_real_distribution<double> u2(0.0, 1.0);
    RowBuilder::TdcpPairInput pair;
    pair.pair_id = 3;
    pair.clone_slot = 0;
    pair.dt_pair_s = dt_pair;
    pair.c_hat_N = c_hat;
    pair.clock_jump_m = clock_jump_m;
    std::vector<SatGeometry> geo;
    nu_pred->clear();
    for (int j = 0; j < m; ++j) {
      const double el = (15.0 + 60.0 * u2(r2)) * kDeg, az = 2.0 * M_PI * u2(r2);
      const Eigen::Vector3d los_N(std::cos(el) * std::sin(az), std::cos(el) * std::cos(az), std::sin(el));
      const Eigen::Vector3d rs0 = anchor.r0_ecef + anchor.R_EN * (los_N * 2.3e7);
      Eigen::Vector3d vs = Eigen::Vector3d(u2(r2) - 0.5, u2(r2) - 0.5, u2(r2) - 0.5).cross(anchor.R_EN * los_N);
      vs = vs.normalized() * 3000.0;
      const Eigen::Vector3d rs1 = rs0 + vs * dt_pair;
      const double dts0 = 1e-4 * (u2(r2) - 0.5), dts1 = dts0 + 1e-9 * dt_pair;   // 0.15 m drift: a missing term is visible
      // true phases (metres → cycles), constant ambiguity per satellite
      const Eigen::Vector3d rA0 = anchor.r0_ecef + anchor.R_EN * c_true, rA1 = anchor.r0_ecef + anchor.R_EN * a_true;
      const double rho0 = rangeSagnac(rs0, rA0, nullptr), rho1 = rangeSagnac(rs1, rA1, nullptr);
      const double amb = std::floor(1e6 * u2(r2));
      const double phi0 = (rho0 + 100.0 - kC * dts0) / lambda + amb + 0.0005 / lambda * g2(r2);
      const double phi1 = (rho1 + 100.0 + dclk_true + phase_jump_m - kC * dts1) / lambda + amb + 0.0005 / lambda * g2(r2);
      pair.sats.push_back(sat(Sys::GPS, j + 1, j + 1));
      pair.phi_k0_cyc.push_back(phi0);
      pair.phi_k1_cyc.push_back(phi1);
      pair.dts_k0.push_back(dts0);
      pair.dts_k1.push_back(dts1);
      pair.rs_k0.push_back(rs0);
      pair.el_k0_rad.push_back(el);
      pair.el_k1_rad.push_back(el);
      pair.cp_std_k0_m.push_back(0.003);
      pair.cp_std_k1_m.push_back(0.003);
      SatGeometry g;
      g.sat = pair.sats.back();
      g.state.ok = true;
      g.state.r_ecef = rs1;
      g.state.v_ecef = vs;
      g.state.dts = dts1;
      const Eigen::Vector3d rA1_hat = anchor.r0_ecef + anchor.R_EN * a_hat;
      g.range = rangeSagnac(rs1, rA1_hat, &g.e_ecef);
      g.u_N = anchor.R_EN.transpose() * g.e_ecef;
      g.el_rad = el;
      g.az_rad = az;
      g.lambda_m = lambda;
      geo.push_back(g);
      // predicted innovation: Δclk + H δ with δ = truth − estimate (−a_err on a^N, −c_err on the clone)
      Eigen::Vector3d e0;
      rangeSagnac(rs0, anchor.r0_ecef + anchor.R_EN * c_hat, &e0);
      const Eigen::Vector3d u0 = anchor.R_EN.transpose() * e0;
      nu_pred->push_back(dclk_true - g.u_N.dot(-a_err) + u0.dot(-c_err));
    }
    AntennaJacobians J;
    J.a_N = a_hat;
    J.w_N = (a_true - c_true) / dt_pair;
    J.M = Eigen::Matrix3d::Identity();
    J.J_a = Eigen::Matrix<double, 3, Eigen::Dynamic>::Zero(3, N);
    J.J_w = Eigen::Matrix<double, 3, Eigen::Dynamic>::Zero(3, N);
    J.J_a.block<3, 3>(0, HostIdx::pos) = Eigen::Matrix3d::Identity();
    J.J_w.block<3, 3>(0, HostIdx::vel) = Eigen::Matrix3d::Identity();
    LinPoint lp;
    lp.anchor = &anchor;
    lp.aug = nullptr;
    lp.bridge = nullptr;
    CoreState x;
    lp.x = &x;
    set->rows.clear();
    rb.tdcpRows(pair, geo, lp, J, set);
  };

  RowSet plain, corrected, uncorrected;
  std::vector<double> pred;
  build(0.0, 0.0, &plain, &pred);
  ASSERT_EQ(static_cast<int>(plain.rows.size()), m);
  EXPECT_TRUE(plain.has_full_R);
  for (int j = 0; j < m; ++j) {
    EXPECT_NEAR(plain.rows[j].nu, pred[j], 3e-3) << "row " << j;   // 0.5 mm phase noise at both ends + ignored Sagnac derivative
  }
  // the position columns carry −u_k1 and the clone columns +u_k1 (AMENDMENTS A64: the k1 line of sight at both ends)
  for (int j = 0; j < m; ++j) {
    const Eigen::Vector3d hpos = plain.rows[j].H.segment<3>(HostIdx::pos).transpose();
    const Eigen::Vector3d hcl = plain.rows[j].H.segment<3>(HostIdx::dim).transpose();
    EXPECT_NEAR(hpos.norm(), 1.0, 1e-9);
    EXPECT_NEAR(hcl.norm(), 1.0, 1e-9);
    EXPECT_LT((hpos + hcl).norm(), 1e-15);
  }
  // (A) jump in the phases, corrected: innovations bit-for-bit-equivalent to the no-jump case
  build(jump, jump, &corrected, &pred);
  for (int j = 0; j < m; ++j) EXPECT_NEAR(corrected.rows[j].nu, plain.rows[j].nu, 1e-6) << "row " << j;
  // (B) jump not corrected: raw innovations carry ~c·1 ms, the robust Δclk sees it, the projected rows do not
  build(jump, 0.0, &uncorrected, &pred);
  double dclk_plain = 0.0, dclk_unc = 0.0;
  RowSet proj_plain, proj_unc, proj_corr;
  ASSERT_TRUE(RowBuilder::projectOutCommonClock(plain, &proj_plain, &dclk_plain));
  ASSERT_TRUE(RowBuilder::projectOutCommonClock(uncorrected, &proj_unc, &dclk_unc));
  ASSERT_TRUE(RowBuilder::projectOutCommonClock(corrected, &proj_corr, nullptr));
  EXPECT_NEAR(dclk_plain, dclk_true, 0.05);
  EXPECT_NEAR(dclk_unc, dclk_true + jump, 0.05);
  ASSERT_EQ(proj_plain.rows.size(), static_cast<size_t>(m - 1));
  for (int j = 0; j < m - 1; ++j) {
    EXPECT_NEAR(proj_unc.rows[j].nu, proj_plain.rows[j].nu, 1e-6) << "projected row " << j;
    EXPECT_NEAR(proj_corr.rows[j].nu, proj_plain.rows[j].nu, 1e-6) << "projected row " << j;
    EXPECT_DOUBLE_EQ(proj_plain.rows[j].R, 1.0);
  }
  // (C) the L4 gate accepts the whole pair in every variant (prior σ 5 cm on antenna and clone)
  Config gcfg;
  JointUpdater upd(gcfg);
  Eigen::MatrixXd Sigma = Eigen::MatrixXd::Identity(N, N) * 1e-6;
  Sigma.block<3, 3>(HostIdx::pos, HostIdx::pos) = Eigen::Matrix3d::Identity() * 0.0025;
  Sigma.block<3, 3>(HostIdx::dim, HostIdx::dim) = Eigen::Matrix3d::Identity() * 0.0025;
  for (RowSet* s : {&plain, &corrected, &uncorrected}) {
    GateResult g = upd.gateClass(*s, Sigma, gcfg.rejection.min_meas.tdcp, gcfg.rejection.alpha_w, gcfg.rejection.alpha_nis,
                                 gcfg.rejection.max_reject_frac);
    EXPECT_EQ(g.n_out, m);
    EXPECT_FALSE(g.class_skipped);
    EXPECT_EQ(g.dof, m - 1);
    EXPECT_LT(g.nis, chi2Quantile(m - 1, 0.999));
  }
}
