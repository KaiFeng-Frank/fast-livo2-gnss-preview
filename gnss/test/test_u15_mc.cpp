// SPDX-License-Identifier: BSD-3-Clause
// test_u15_mc.cpp — TEST_PLAN §1 U15: Monte-Carlo consistency of the joint filter on the ROS-free simulator (gnss/sim).
// Scope, criteria and findings: AMENDMENTS A60. Owner: P3-E.
//
// Profiles. quick (default, what ctest runs): a few runs per scenario on short paths, with wide criteria that catch a
// gross inconsistency (about a factor 2 in variance) in a couple of minutes. full (U15_FULL=1): TEST_PLAN's 50 runs of
// 1 km per scenario with the A60 criteria — 6 to 16 min per scenario on one core (four systems the longest); run one
// scenario per process to use several (--gtest_filter=U15GroupA.OpenSky, …). U15_RUNS / U15_PATH_M override the run count / path of either
// profile. Every scenario prints one report line ("[U15] …").
// The scenes need the development-set fixture's BRDC (Grass02); without it every test SKIPs.
// U15Explore.Scenario is the study harness behind A60 (skipped unless U15_EXPLORE is set; see its comment).
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <string>
#include <vector>

#include "livo_gnss/gnss_fusion.hpp"
#include "../sim/simulator.hpp"
#include "../../gnss_io/test/fixture_discovery.hpp"

using namespace livo_gnss;
using namespace livo_gnss::sim;

namespace {

constexpr double kDeg = M_PI / 180.0;
constexpr int64_t kFixtureT0 = 1421486476 + 315964800;   // Grass02 fixture t0 on the GTime::toSec() scale

std::string grass02Brdc() {
  for (const std::string& f : livo_gnss_io::testsupport::realFixtures(LIVO_GNSS_TEST_DATA_DIR)) {
    if (livo_gnss_io::testsupport::baseName(f).compare(0, 8, "Grass02_") != 0) continue;
    return livo_gnss_io::testsupport::fixtureBrdc(f);
  }
  return std::string();
}

// The fixture's site and day; 40 start times 150 s apart (seed mod 40) give 40 constellation geometries.
SimParams baseParams(Scenario s, uint32_t seed, double path_m) {
  SimParams p;
  p.scenario = s;
  p.seed = seed;
  p.nav_rinex = grass02Brdc();
  p.site.lat_rad = 34.0339415 * kDeg;
  p.site.lon_rad = 108.7614865 * kDeg;
  p.site.h_m = 388.05;
  p.t0.sec = kFixtureT0 + 1000 + static_cast<int64_t>(seed % 40) * 150;
  p.t0.frac = 0.0;
  p.path_m = path_m;
  return p;
}

bool envFlag(const char* name) {
  const char* v = std::getenv(name);
  return v && *v && std::string(v) != "0";
}
bool fullProfile() { return envFlag("U15_FULL"); }

struct Profile {
  int runs = 0;
  double path_m = 0.0;
};

// quick: 4 runs (group A) / 2 runs (B, C) on 250 m — longer where the scenario's event needs room: the outage (120 s from
// 0.4 T) and the few-satellite window (90 s from 0.4 T). full: 50 × 1 km.
Profile profileFor(Scenario s) {
  Profile p;
  const bool full = fullProfile();
  p.runs = full ? 50 : (scenarioGroup(s) == Group::ModelMatched ? 4 : 2);
  p.path_m = full ? 1000.0 : s == Scenario::Outage120s ? 450.0 : s == Scenario::FewSats ? 320.0 : 250.0;
  if (const char* r = std::getenv("U15_RUNS")) p.runs = std::max(1, std::atoi(r));
  if (const char* m = std::getenv("U15_PATH_M")) p.path_m = std::atof(m);
  return p;
}

std::vector<RunResult> runMany(Scenario s, const Profile& pr, bool fixed_anchor, bool* no_fixture) {
  std::vector<RunResult> out;
  *no_fixture = false;
  for (int k = 0; k < pr.runs; ++k) {
    SimParams p = baseParams(s, static_cast<uint32_t>(k), pr.path_m);
    if (p.nav_rinex.empty()) {
      *no_fixture = true;
      return out;
    }
    p.fixed_anchor = fixed_anchor;
    RunResult r = runScenario(p, simConfig(p));
    if (r.no_fixture) {
      *no_fixture = true;
      return out;
    }
    out.push_back(std::move(r));
  }
  return out;
}

void report(const char* tag, Scenario s, const std::vector<RunResult>& runs, const ConsistencyStats& c) {
  double model = 0.0;
  for (const RunResult& r : runs)
    if (std::isfinite(r.model_psr_max_abs_m)) model = std::max(model, r.model_psr_max_abs_m);
  std::printf("[U15] %s%s %s: %s; generator |P - h(truth)| max %.2e m\n", tag, scenarioName(s),
              fullProfile() ? "(full)" : "(quick)", c.describe().c_str(), model);
  for (const RunResult& r : runs)
    if (!r.error.empty()) std::printf("[U15]   run failed: %s\n", r.error.c_str());
}

// Preconditions every scene must meet whatever its group: no run failed, every run activated, and (groups A/C) the
// generated pseudoranges equal the filter's own measurement model at the truth.
void expectScenesSound(const std::vector<RunResult>& runs) {
  for (size_t k = 0; k < runs.size(); ++k) {
    const RunResult& r = runs[k];
    EXPECT_TRUE(r.error.empty()) << "run " << k << ": " << r.error;
    EXPECT_TRUE(r.activated) << "run " << k << " never activated";
    if (std::isfinite(r.model_psr_max_abs_m)) {
      EXPECT_LT(r.model_psr_max_abs_m, 1e-3) << "run " << k;
    }
  }
}

// ---- A60 criteria ----
// Group A (model-matched): every NEES kind (ξ 6 dof, antenna 3, clock b and d 1 each) with its time-averaged ANEES/dof in
// [lo, hi] and its per-time ANEES inside the Bar-Shalom 95 % band of the run count for at least `in_band` of the times;
// every NIS class pooled NIS/dof in [0.9, 1.1] with ≥ 90 % of the epochs inside their own 95 % interval. The NEES of one
// run is strongly correlated in time (the clock and the registration are random walks), so the time average of a
// consistent filter scatters far more than χ²(N·T) would say: the full-profile bounds [0.75, 1.33] / 0.8 are what 50
// runs of a consistent filter meet (A60 table); the quick profile only rejects gross errors.
struct NeesBounds {
  double lo, hi, in_band;
};
NeesBounds groupABounds() { return fullProfile() ? NeesBounds{0.75, 1.33, 0.8} : NeesBounds{0.4, 2.5, 0.5}; }

void checkGroupA(Scenario s) {
  const Profile pr = profileFor(s);
  bool no_fixture = false;
  const std::vector<RunResult> runs = runMany(s, pr, false, &no_fixture);
  if (no_fixture) GTEST_SKIP() << "Grass02 BRDC fixture not present";
  ASSERT_EQ(scenarioGroup(s), Group::ModelMatched);
  const ConsistencyStats c = aggregate(runs);
  report("A ", s, runs, c);
  expectScenesSound(runs);
  const NeesBounds nb = groupABounds();
  std::string why;
  EXPECT_TRUE(c.withinBounds(nb.lo, nb.hi, nb.in_band, 0.9, 1.1, 0.9, &why)) << scenarioName(s) << ": " << why;
}

// Mean of nees/dof over the samples of every run with t in [from(r), to(r)).
template <typename From, typename To>
double windowAneesPerDof(const std::vector<RunResult>& runs, double NeesSample::*field, int dof, From from, To to, int* n_out,
                         bool skip_deceived = false) {
  double sum = 0.0;
  int n = 0;
  for (const RunResult& r : runs) {
    const double a = from(r), b = to(r);
    for (const NeesSample& smp : r.nees) {
      const double v = smp.*field;
      if (smp.t < a || smp.t >= b || !std::isfinite(v) || (skip_deceived && smp.host_deceived)) continue;
      sum += v / dof;
      ++n;
    }
  }
  if (n_out) *n_out = n;
  return n > 0 ? sum / n : kNaN;
}

}  // namespace

// ---------------------------------------------------------------------------------------------
// The simulator itself: reproducible, the host stand-in consistent, the scene on the filter's model.
// ---------------------------------------------------------------------------------------------
TEST(U15Simulator, ReproducibleSelfConsistentScene) {
  SimParams p = baseParams(Scenario::OpenSky, 7, 150.0);
  if (p.nav_rinex.empty()) GTEST_SKIP() << "Grass02 BRDC fixture not present";
  const Config cfg = simConfig(p);
  const RunResult a = runScenario(p, cfg), b = runScenario(p, cfg);
  ASSERT_TRUE(a.error.empty()) << a.error;
  ASSERT_TRUE(a.activated);
  std::printf("[U15] simulator: %s; drift from %.1f s; DOV (%.1f, %.1f) µrad; |P - h(truth)| max %.2e m\n", a.describe().c_str(),
              a.drift_start_s, a.dov(0) * 1e6, a.dov(1) * 1e6, a.model_psr_max_abs_m);
  // bit-for-bit reproducible on one build (every draw from the run's seeded engine)
  ASSERT_EQ(a.nees.size(), b.nees.size());
  for (size_t i = 0; i < a.nees.size(); ++i) {
    EXPECT_EQ(a.nees[i].nees_ant, b.nees[i].nees_ant) << i;
    EXPECT_EQ(a.nees[i].nees_xi, b.nees[i].nees_xi) << i;
  }
  EXPECT_EQ(a.t_activate, b.t_activate);
  // the registration walk runs from the map origin (A61 F1: the activation tilt prior carries it)
  EXPECT_DOUBLE_EQ(a.drift_start_s, 0.0);
  // drift_from_activation (the pre-A61 study setting): the walk starts where a first pass activated, and the second pass
  // activates at the same cycle (else runScenario reports an error)
  {
    SimParams q = p;
    q.drift_from_activation = true;
    const RunResult d = runScenario(q, cfg);
    ASSERT_TRUE(d.error.empty()) << d.error;
    EXPECT_DOUBLE_EQ(d.drift_start_s, d.t_activate);
    EXPECT_EQ(d.t_activate, a.t_activate);
  }
  // the generated pseudoranges are the filter's own model at the truth (sub-mm: satellite transmit-time iteration only)
  EXPECT_LT(a.model_psr_max_abs_m, 1e-3);
  // the synthetic host (IMU propagation + mm-level pose update) is itself consistent: pose NEES/6 near 1
  EXPECT_GT(a.host_nees_mean / 6.0, 0.5);
  EXPECT_LT(a.host_nees_mean / 6.0, 2.0);
  // the antithetic twin differs (the noise is really drawn) and still activates
  SimParams q = p;
  q.noise_sign = -1.0;
  const RunResult c = runScenario(q, cfg);
  ASSERT_TRUE(c.error.empty()) << c.error;
  EXPECT_TRUE(c.activated);
  ASSERT_FALSE(c.nees.empty());
  ASSERT_FALSE(a.nees.empty());
  EXPECT_NE(c.nees.back().ant_err_N(0), a.nees.back().ant_err_N(0));
}

// ---------------------------------------------------------------------------------------------
// Group A: the scene follows the filter's model — NEES and NIS in their χ² bands.
// ---------------------------------------------------------------------------------------------
TEST(U15GroupA, OpenSky) { checkGroupA(Scenario::OpenSky); }
TEST(U15GroupA, Outage120s) { checkGroupA(Scenario::Outage120s); }
TEST(U15GroupA, ClockJumps) { checkGroupA(Scenario::ClockJumps); }
TEST(U15GroupA, CycleSlips) { checkGroupA(Scenario::CycleSlips); }
TEST(U15GroupA, FourSystemsIsb) { checkGroupA(Scenario::FourSystemsIsb); }
TEST(U15GroupA, FewSats) { checkGroupA(Scenario::FewSats); }
TEST(U15GroupA, LongStill) { checkGroupA(Scenario::LongStill); }

// ---------------------------------------------------------------------------------------------
// Group B: realistic time-correlated errors (Gauss–Markov multipath, NLOS) covered by κ_AR — the filter must not be
// over-confident. The per-epoch NIS is not a criterion here: κ_AR inflates R for the time average by design, so a
// consistent κ_AR filter shows per-epoch NIS/dof ≈ 1/κ.
// ---------------------------------------------------------------------------------------------
TEST(U15GroupB, UrbanMultipathNotOverconfident) {
  const Scenario s = Scenario::UrbanMultipath;
  const Profile pr = profileFor(s);
  bool no_fixture = false;
  const std::vector<RunResult> runs = runMany(s, pr, false, &no_fixture);
  if (no_fixture) GTEST_SKIP() << "Grass02 BRDC fixture not present";
  const ConsistencyStats c = aggregate(runs);
  report("B ", s, runs, c);
  expectScenesSound(runs);
  const double hi = fullProfile() ? 1.33 : 2.5;
  EXPECT_LE(c.xi.anees_per_dof, hi);
  EXPECT_LE(c.ant.anees_per_dof, hi);
  // bounded: the global antenna error stays at the pseudorange level
  double sq = 0.0;
  int n = 0;
  for (const RunResult& r : runs)
    for (const NeesSample& smp : r.nees) {
      sq += smp.ant_err_N.squaredNorm();
      ++n;
    }
  ASSERT_GT(n, 0);
  EXPECT_LT(std::sqrt(sq / n), 5.0) << "antenna RMS error";
}

// ---------------------------------------------------------------------------------------------
// Group C: the map violates the model — the filter stays bounded and heads back once the violation is over.
//   bounded: ANEES/dof of ξ and of the antenna averaged from the violation on ≤ 5 (quick 8);
//   recovering (transient violations): antenna ANEES/dof over the last 15 % of each run ≤ 4 and (full profile) either
//   consistent (≤ 1.33) or below its value over the first 15 % after the violation — not "recovered": a relocalisation
//   jump is absorbed only as fast as the path-length random walk lets ξ move (DESIGN §16 limitation 2; A60 F9).
// ---------------------------------------------------------------------------------------------
namespace {

void checkGroupC(Scenario s, bool transient, std::vector<RunResult>* out) {
  const Profile pr = profileFor(s);
  bool no_fixture = false;
  *out = runMany(s, pr, false, &no_fixture);
  if (no_fixture) {
    out->clear();
    return;
  }
  const std::vector<RunResult>& runs = *out;
  const ConsistencyStats c = aggregate(runs, /*exclude_mismatch=*/false);
  report("C ", s, runs, c);
  expectScenesSound(runs);
  const double bounded = fullProfile() ? 5.0 : 8.0;
  auto from = [](const RunResult& r) { return r.t_mismatch; };
  auto end = [](const RunResult& r) { return r.duration_s + 1.0; };
  int n_xi = 0, n_ant = 0;
  // ξ is not compared with the true registration while the LIO pose itself is wrong (NeesSample::host_deceived, the map
  // ghosting windows): there it rightly absorbs the LIO's error; the antenna is judged throughout (A61).
  const double xi_v = windowAneesPerDof(runs, &NeesSample::nees_xi, 6, from, end, &n_xi, /*skip_deceived=*/true);
  const double ant_v = windowAneesPerDof(runs, &NeesSample::nees_ant, 3, from, end, &n_ant);
  std::printf("[U15]   %s: from the violation on ANEES/dof xi %.3f (%d) ant %.3f (%d)\n", scenarioName(s), xi_v, n_xi, ant_v, n_ant);
  EXPECT_GT(n_ant, 0);
  EXPECT_LE(xi_v, bounded);
  EXPECT_LE(ant_v, bounded);
  if (transient) {
    auto after = [](const RunResult& r) { return r.t_mismatch + 0.15 * r.duration_s; };
    auto tail = [](const RunResult& r) { return 0.85 * r.duration_s; };
    int n_first = 0, n_tail = 0;
    const double ant_f = windowAneesPerDof(runs, &NeesSample::nees_ant, 3, from, after, &n_first);
    const double ant_t = windowAneesPerDof(runs, &NeesSample::nees_ant, 3, tail, end, &n_tail);
    std::printf("[U15]   %s: antenna ANEES/dof first 15 %% after the violation %.3f (%d), last 15 %% %.3f (%d)\n", scenarioName(s),
                ant_f, n_first, ant_t, n_tail);
    EXPECT_GT(n_tail, 0);
    EXPECT_LE(ant_t, 4.0);
    // the trend needs the full profile: two 250 m runs end ~35 s after the last ghost window, inside the run-to-run scatter
    if (fullProfile()) {
      EXPECT_TRUE(ant_t <= 1.33 || ant_t < ant_f) << "antenna ANEES/dof " << ant_t << " at the end, " << ant_f << " after the violation";
    }
  }
}

}  // namespace

// The LIO relocalises against the old map at 0.65 T: its pose jumps by the drift accumulated since 0.15 T.
TEST(U15GroupC, RevisitSnapBoundedAndRecovering) {
  std::vector<RunResult> runs;
  checkGroupC(Scenario::RevisitSnap, true, &runs);
  if (runs.empty()) GTEST_SKIP() << "Grass02 BRDC fixture not present";
}

// A deterministic 4e-5 rad/m yaw drift from 0.3 T on (2.3°/km, far outside q_ψ): the GNSS keeps the yaw on the truth.
TEST(U15GroupC, YawDriftMismatchTracked) {
  std::vector<RunResult> runs;
  checkGroupC(Scenario::YawDriftMismatch, false, &runs);
  if (runs.empty()) GTEST_SKIP() << "Grass02 BRDC fixture not present";
  // tracked: the yaw error at the end is less than half the deterministic drift accumulated since 0.3 T (the lag of a
  // random-walk model behind a ramp is fixed, so the ratio shrinks on longer paths: ~0.3 at 250 m)
  const double drift = 4.0e-5 * 0.7 * profileFor(Scenario::YawDriftMismatch).path_m;
  for (const RunResult& r : runs) {
    ASSERT_FALSE(r.nees.empty());
    EXPECT_LT(std::fabs(r.nees.back().yaw_err), 0.5 * drift) << "yaw error vs deterministic drift " << drift << " rad";
  }
}

// The LIO snaps to a ghost copy of the map (0.4 m, −0.2 m) for 20 s at 0.45 T and 0.65 T.
TEST(U15GroupC, MapGhostingBoundedAndRecovering) {
  std::vector<RunResult> runs;
  checkGroupC(Scenario::MapGhosting, true, &runs);
  if (runs.empty()) GTEST_SKIP() << "Grass02 BRDC fixture not present";
  // the integrity monitor sees the LIO jump (LIVO alarm) in most runs: a 0.45 m ghost against metre-level pseudoranges
  // is not a sure detection (full profile: 49 of 50 runs)
  int alarmed = 0;
  for (const RunResult& r : runs) alarmed += r.events[static_cast<size_t>(EventKind::LIVO_ALARM_ON)] > 0 ? 1 : 0;
  std::printf("[U15]   map_ghosting: LIVO_ALARM in %d of %zu runs\n", alarmed, runs.size());
  EXPECT_GE(alarmed, static_cast<int>(0.8 * static_cast<double>(runs.size())));
}

// AMENDMENTS A66/A67: A59's per-satellite Doppler biases. Open sky (OpenSky's scene draw for draw) whose satellites below
// 30° carry a constant Doppler bias of ±0.15–0.55 m/s and are described as the development set's biased satellites were
// (A59: C/N0 16–22 dB-Hz, row σ 0.7–3 m/s, the bias invisible to the NIS): they report 20 dB-Hz (below the carrier mask:
// no TDCP rows) and a receiver Doppler std of 4 Hz (σ 0.56–1.45 m/s from 10° to 30°) — SimParams' defaults; every term is
// drawn at the filter's own variance (group A's noise), the biases are the only violation. Criteria (A66 "倾斜 z² 不失控、
// 里程计输出不劣于宿主"):
//   - the tilt: its per-component z² averaged over the samples after activation, pooled over the runs and in every run,
//     within group C's bound (≤ 5; quick ≤ 8);
//   - the odometry output: in every run its ATE below the host's (U15Ate's criterion);
//   - full profile only: the same scenes with the Doppler rows' full tilt content — noise.dopp.project_tilt = false and
//     consider_tilt = false, the pre-A66 joint gain — push the tilt further (pooled z² of the two components larger) —
//     the regression check of the mechanism (the quick runs are too short for it). Both are printed.
// The mechanism is the configured default: A66's consider entry until A70, the A69/A70 projection of the Doppler rows
// orthogonal to the tilt since (consider an option, default off). The literal group-A σ for the biased rows
// (U15_SIM dbcn0=45,dbstd=0: the biases are 1–5σ, the Doppler NIS shows them) is a study setting, reported in A67/A70.
namespace {
struct TiltOdo {
  double z2x = kNaN, z2y = kNaN, worst_run = 0.0, ratio_med = kNaN, ratio_max = 0.0;
  int runs = 0, odo_worse = 0;
};
TiltOdo tiltAndOdometry(const std::vector<RunResult>& runs) {
  TiltOdo t;
  double sx = 0.0, sy = 0.0;
  long n = 0;
  std::vector<double> ratio;
  for (const RunResult& r : runs) {
    double rx = 0.0, ry = 0.0;
    long rn = 0;
    for (const NeesSample& smp : r.nees) {
      if (!std::isfinite(smp.z2_xi[0]) || !std::isfinite(smp.z2_xi[1])) continue;
      rx += smp.z2_xi[0];
      ry += smp.z2_xi[1];
      ++rn;
    }
    if (rn > 0) t.worst_run = std::max(t.worst_run, std::max(rx, ry) / static_cast<double>(rn));
    sx += rx;
    sy += ry;
    n += rn;
    if (std::isfinite(r.ate_o) && std::isfinite(r.ate_u) && r.ate_u > 0.0) {
      ratio.push_back(r.ate_o / r.ate_u);
      t.ratio_max = std::max(t.ratio_max, r.ate_o / r.ate_u);
      if (!(r.ate_o < r.ate_u)) ++t.odo_worse;
    }
    ++t.runs;
  }
  if (n > 0) {
    t.z2x = sx / static_cast<double>(n);
    t.z2y = sy / static_cast<double>(n);
  }
  if (!ratio.empty()) {
    std::sort(ratio.begin(), ratio.end());
    t.ratio_med = ratio[ratio.size() / 2];
  }
  return t;
}
}  // namespace

TEST(U15GroupC, LowElevationDopplerBiasTiltBoundedOdometryBeatsHost) {
  const Scenario s = Scenario::LowElevDopplerBias;
  const Profile pr = profileFor(s);
  std::vector<RunResult> on, off;
  for (int k = 0; k < pr.runs; ++k) {
    SimParams p = baseParams(s, static_cast<uint32_t>(k), pr.path_m);
    if (p.nav_rinex.empty()) GTEST_SKIP() << "Grass02 BRDC fixture not present";
    Config cfg = simConfig(p);
    ASSERT_TRUE(cfg.noise.dopp.project_tilt) << "A70 default";
    ASSERT_FALSE(cfg.noise.dopp.consider_tilt) << "A70 default (A66's consider is an option)";
    on.push_back(runScenario(p, cfg));
    if (on.back().no_fixture) GTEST_SKIP() << "Grass02 BRDC fixture not present";
    if (fullProfile()) {
      cfg.noise.dopp.project_tilt = false;
      cfg.noise.dopp.consider_tilt = false;
      off.push_back(runScenario(p, cfg));
    }
  }
  const ConsistencyStats c = aggregate(on, /*exclude_mismatch=*/false);
  report("C ", s, on, c);
  expectScenesSound(on);
  const TiltOdo t = tiltAndOdometry(on);
  const double bound = fullProfile() ? 5.0 : 8.0;
  std::printf("[U15]   %s: tilt z2 x %.3f y %.3f (worst run %.2f); odometry/host ATE median %.2f max %.2f, %d of %d runs not below the host\n",
              scenarioName(s), t.z2x, t.z2y, t.worst_run, t.ratio_med, t.ratio_max, t.odo_worse, t.runs);
  EXPECT_LE(t.z2x, bound);
  EXPECT_LE(t.z2y, bound);
  EXPECT_LE(t.worst_run, bound) << "a run's tilt ran away";
  EXPECT_EQ(t.odo_worse, 0) << "the odometry output is not below the host alone in every run";
  if (!off.empty()) {
    const TiltOdo u = tiltAndOdometry(off);
    std::printf("[U15]   %s with the Doppler rows' full tilt content (pre-A66 gain): tilt z2 x %.3f y %.3f (worst run %.2f); "
                "odometry/host ATE median %.2f max %.2f\n",
                scenarioName(s), u.z2x, u.z2y, u.worst_run, u.ratio_med, u.ratio_max);
    EXPECT_GT(u.z2x + u.z2y, t.z2x + t.z2y) << "the Doppler biases no longer push the tilt harder without the default treatment";
  }
}

// ---------------------------------------------------------------------------------------------
// AMENDMENTS A61 ATE proxy: the SE(3)-aligned position ATE of the antenna (evo_ape -a, as M3DGR and FAST-LIVGO report) of
// the host alone (U, traj_W) and of the odometry output (traj_N_odom, F13) at the same instants. The odometry output keeps
// every drift correction since activation and leaves out the later revision of the activation instant — the like-for-like
// of FAST-LIVGO's frame, fixed after its initialisation. The global output (traj_N) is not asserted: on short runs its
// convergence after activation (information about where the whole trajectory lies, which an SE(3) alignment cannot
// absorb once it changes along the run) costs more ATE than the drift it removes (A61: 250 m, T/U 1.5 open sky and 3.9
// realistic, while the odometry output is at 0.20 / 0.25; 1 km: 0.27 / 0.67 against 0.05 / 0.10).
// ---------------------------------------------------------------------------------------------
TEST(U15Ate, OdometryOutputRemovesTheDrift) {
  for (Scenario s : {Scenario::OpenSky, Scenario::OpenSkyRealistic}) {
    Profile pr = profileFor(s);
    if (!std::getenv("U15_RUNS")) pr.runs = fullProfile() ? 20 : 2;
    bool no_fixture = false;
    const std::vector<RunResult> runs = runMany(s, pr, false, &no_fixture);
    if (no_fixture) GTEST_SKIP() << "Grass02 BRDC fixture not present";
    std::vector<double> ratio, ratio_glob;
    for (const RunResult& r : runs) {
      ASSERT_TRUE(r.error.empty()) << r.error;
      ASSERT_TRUE(std::isfinite(r.ate_u) && std::isfinite(r.ate_o) && std::isfinite(r.ate_t)) << r.describe();
      EXPECT_LT(r.ate_o, r.ate_u) << scenarioName(s) << ": the odometry output does not beat the host alone";
      ratio.push_back(r.ate_o / r.ate_u);
      ratio_glob.push_back(r.ate_t / r.ate_u);
    }
    std::sort(ratio.begin(), ratio.end());
    std::sort(ratio_glob.begin(), ratio_glob.end());
    const double med = ratio[ratio.size() / 2], med_g = ratio_glob[ratio_glob.size() / 2];
    std::printf("[U15] ATE %s: %zu runs of %.0f m, odometry/host median %.2f [%.2f, %.2f], global/host median %.2f\n",
                scenarioName(s), runs.size(), pr.path_m, med, ratio.front(), ratio.back(), med_g);
    EXPECT_LT(med, fullProfile() ? 0.3 : 0.5) << scenarioName(s);
  }
}

// ---------------------------------------------------------------------------------------------
// TEST_PLAN U15 "对照固定锚版 NEES 随时间增长" / DESIGN §6: with the fixed-anchor TDCP (A-FA) the posterior collapses
// to the TDCP noise level while the error keeps the pseudorange level — the antenna/ξ NEES is off by orders of magnitude
// from the first TDCP on (so its own ∝ k growth is hidden under e_0²/σ²), and the clock, which the pseudoranges keep
// pulling against a frozen position, has its NEES grow with time (full profile, 20 runs of 1 km: last third ×1.48 the
// first; ×1.8 on 450 m — the trend needs the full profile to stand out of the run-to-run scatter); the clone design on
// the same seeds stays in the group-A bounds.
// ---------------------------------------------------------------------------------------------
TEST(U15FixedAnchor, NeesInconsistentAndGrowingCloneConsistent) {
  Profile pr = profileFor(Scenario::OpenSky);
  if (!std::getenv("U15_RUNS")) pr.runs = fullProfile() ? 20 : 2;
  bool no_fixture = false;
  const std::vector<RunResult> fa = runMany(Scenario::OpenSky, pr, true, &no_fixture);
  if (no_fixture) GTEST_SKIP() << "Grass02 BRDC fixture not present";
  const std::vector<RunResult> cl = runMany(Scenario::OpenSky, pr, false, &no_fixture);
  const ConsistencyStats cf = aggregate(fa), cc = aggregate(cl);
  report("A-FA ", Scenario::OpenSky, fa, cf);
  report("clone ", Scenario::OpenSky, cl, cc);
  expectScenesSound(fa);
  expectScenesSound(cl);
  EXPECT_GT(cf.ant.anees_per_dof, 100.0);
  EXPECT_GT(cf.xi.anees_per_dof, 100.0);
  if (fullProfile()) {
    EXPECT_GT(cf.b.first_third, 10.0);
    EXPECT_GT(cf.b.last_third, 1.2 * cf.b.first_third) << "clock NEES of the fixed anchor does not grow";
  }
  EXPECT_LT(cc.ant.anees_per_dof, groupABounds().hi);
  EXPECT_LT(cc.xi.anees_per_dof, groupABounds().hi);
}

// ---------------------------------------------------------------------------------------------
// Study harness (A60): one scenario, many runs, with the scene/filter knobs of the findings and the diagnostics that
// isolated them — first-sample z² (the initialiser), ANEES per 20 s since activation, per-component z² and signed means
// (bias checks with antithetic pairs), clock error magnitudes, rows accepted/rejected per class.
//   U15_EXPLORE=1 U15_SCENARIO=open_sky U15_RUNS=20 U15_PATH_M=300 U15_SEED0=0 U15_ZBIN=60
//   U15_CFG="q_tilt=0,q_yaw=0,q_th=0,q_tv=0,tdcp=0,dopp=0,psr=0,gal=0,bds=0,glo=0,S_b=…,S_d=…,srate=…,swu=…,dov=…,grav=…"
//   U15_SIM="catmo=1,dfa=0,dov=0,dih=1,sign=-1,fa=1,lever=0,cn0=18,toff=0.02,seph=0,gyr_n=…,acc_n=…,turn=…,lio_rot=…,lio_pos=…"
//   (U15_CFG satcsv=1 also lists the TDCP rows the slip detectors flagged; fj=0/1 turns F9 off/on — on by default only in
//   the revisit and ghosting scenes, A64 —, chain=0 F10, pst=0 F11; dproj=0/1 the A70 projection of the Doppler rows off
//   the tilt (default on), dcons=0/1 A66's consider entry (default off since A70; both 0 = the pre-A66 gain);
//   sphi/toff/slev set σ_Φ0, σ_toff and the lever-arm σ of the filter and — where the scene draws from them — the scene)
//   U15_SETTLE / U15_EVAL_DT / U15_TRACE=1 (per-sample clock trace)
// ---------------------------------------------------------------------------------------------
namespace {

void applyCfgOverrides(const char* ov, Config* cfg) {
  const std::string o(ov);
  size_t pos = 0;
  while (pos < o.size()) {
    const size_t e = o.find(',', pos);
    const std::string kv = o.substr(pos, e == std::string::npos ? std::string::npos : e - pos);
    pos = e == std::string::npos ? o.size() : e + 1;
    const size_t eq = kv.find('=');
    const std::string k = kv.substr(0, eq);
    const double v = eq == std::string::npos ? 0.0 : std::atof(kv.substr(eq + 1).c_str());
    if (k == "q_tilt") cfg->process.frame.q_tilt_per_m = v;
    else if (k == "q_yaw") cfg->process.frame.q_yaw_per_m = v;
    else if (k == "q_th") cfg->process.frame.q_th_per_m = v;
    else if (k == "q_tv") cfg->process.frame.q_tv_per_m = v;
    else if (k == "tdcp") cfg->use.tdcp = v != 0.0;
    else if (k == "dopp") cfg->use.dopp = v != 0.0;
    else if (k == "psr") cfg->use.psr = v != 0.0;
    else if (k == "dov") cfg->frames.sigma_dov_rad = v;
    else if (k == "lev") cfg->frames.sigma_lev_rad = v;
    else if (k == "grav") cfg->frames.gnss_updates_gravity = v != 0.0;
    else if (k == "gal") cfg->systems.gal = v != 0.0;
    else if (k == "bds") cfg->systems.bds = v != 0.0;
    else if (k == "glo") cfg->systems.glo = v != 0.0;
    else if (k == "S_b") cfg->process.clock.S_b = v;
    else if (k == "S_d") cfg->process.clock.S_d = v;
    else if (k == "srate") cfg->noise.tdcp.sigma_rate_mps = v;
    else if (k == "swu") cfg->noise.tdcp.sigma_windup_m = v;
    else if (k == "satcsv") cfg->output.sat_csv = v != 0.0;
    else if (k == "fj") cfg->integrity.frame_jump.enable = v != 0.0;
    else if (k == "chain") cfg->noise.tdcp.chain_pairs = v != 0.0;
    else if (k == "pst") cfg->noise.tdcp.phase_states = v != 0.0;
    else if (k == "sphi") cfg->noise.tdcp.sigma0_m = v;
    else if (k == "toff") cfg->time.sigma_toff_s = v;
    else if (k == "slev") cfg->frames.lever_arm_sigma = v;
    else if (k == "td") cfg->time.estimate_online = v != 0.0;
    else if (k == "odom") cfg->output.odom_output = v != 0.0;
    else if (k == "tint") cfg->noise.tdcp.interval_s = v;
    else if (k == "dcons") cfg->noise.dopp.consider_tilt = v != 0.0;
    else if (k == "dproj") cfg->noise.dopp.project_tilt = v != 0.0;
    else if (!k.empty()) std::printf("unknown U15_CFG key %s\n", k.c_str());
  }
}

void applySimOverrides(const char* ov, SimParams* p) {
  const std::string o(ov);
  size_t pos = 0;
  while (pos < o.size()) {
    const size_t e = o.find(',', pos);
    const std::string kv = o.substr(pos, e == std::string::npos ? std::string::npos : e - pos);
    pos = e == std::string::npos ? o.size() : e + 1;
    const size_t eq = kv.find('=');
    const std::string k = kv.substr(0, eq);
    const double v = eq == std::string::npos ? 0.0 : std::atof(kv.substr(eq + 1).c_str());
    if (k == "lever") p->lever_I *= v;
    else if (k == "gyr_n") p->gyr_n = v;
    else if (k == "acc_n") p->acc_n = v;
    else if (k == "gyr_w") p->gyr_w = v;
    else if (k == "acc_w") p->acc_w = v;
    else if (k == "turn") p->turn_rate_rps = v;
    else if (k == "lio_rot") p->lio_rot_sigma = v;
    else if (k == "lio_pos") p->lio_pos_sigma = v;
    else if (k == "sign") p->noise_sign = v;
    else if (k == "catmo") p->carrier_atmo = static_cast<int>(v);
    else if (k == "fa") p->fixed_anchor = v != 0.0;
    else if (k == "dfa") p->drift_from_activation = v != 0.0;
    else if (k == "dov") p->draw_dov = v != 0.0;
    else if (k == "dih") p->drift_in_host = v != 0.0;
    else if (k == "cn0") p->realistic_cn0_base_db = v;
    else if (k == "toff") p->toff_sigma_s = v;
    else if (k == "seph") p->single_ephemeris = v != 0.0;
    else if (k == "dblo") p->dopp_bias_lo_mps = v;
    else if (k == "dbhi") p->dopp_bias_hi_mps = v;
    else if (k == "dbel") p->dopp_bias_el_deg = v;
    else if (k == "dbstd") p->dopp_bias_std_hz = v;
    else if (k == "dbcn0") p->dopp_bias_cn0_db = v;
    else if (k == "dstart") p->drift_start_s = v;
    else if (!k.empty()) std::printf("unknown U15_SIM key %s\n", k.c_str());
  }
}

}  // namespace

TEST(U15Explore, Scenario) {
  if (!envFlag("U15_EXPLORE")) GTEST_SKIP() << "study harness: set U15_EXPLORE=1 (see the comment above)";
  Scenario s = Scenario::OpenSky;
  if (const char* sc = std::getenv("U15_SCENARIO")) {
    ASSERT_TRUE(scenarioFromName(sc, &s)) << sc;
  }
  const double path = std::getenv("U15_PATH_M") ? std::atof(std::getenv("U15_PATH_M")) : 300.0;
  const int runs = std::getenv("U15_RUNS") ? std::max(1, std::atoi(std::getenv("U15_RUNS"))) : 1;
  const int seed0 = std::getenv("U15_SEED0") ? std::atoi(std::getenv("U15_SEED0")) : 0;
  const double zbin = std::getenv("U15_ZBIN") ? std::atof(std::getenv("U15_ZBIN")) : 60.0;
  std::vector<RunResult> rr;
  for (int k = 0; k < runs; ++k) {
    SimParams p = baseParams(s, static_cast<uint32_t>(seed0 + k), path);
    if (p.nav_rinex.empty()) GTEST_SKIP() << "Grass02 BRDC fixture not present";
    if (const char* st = std::getenv("U15_SETTLE")) p.settle_s = std::atof(st);
    if (const char* ev = std::getenv("U15_EVAL_DT")) p.eval_dt_s = std::atof(ev);
    if (const char* sv = std::getenv("U15_SIM")) applySimOverrides(sv, &p);
    Config cfg = simConfig(p);
    if (const char* ov = std::getenv("U15_CFG")) applyCfgOverrides(ov, &cfg);
    const auto t0 = std::chrono::steady_clock::now();
    RunResult r = runScenario(p, cfg);
    const double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    std::printf("[U15] %s seed %d (%.1f s): %s\n", scenarioName(s), seed0 + k, secs, r.describe().c_str());
    std::printf("   activate: %s\n", r.activate_detail.c_str());
    if (!r.nees.empty()) {
      const NeesSample& f = r.nees.front();
      std::printf("   first sample t %.1f: err E/N/U %+.3f %+.3f %+.3f (sigma %.3f %.3f %.3f) z_b %+.2f\n", f.t, f.ant_err_N(0),
                  f.ant_err_N(1), f.ant_err_N(2), f.sigma_ant(0), f.sigma_ant(1), f.sigma_ant(2), f.zs_b);
    }
    if (envFlag("U15_TRACE"))
      for (const std::string& e : r.event_log) std::printf("   event %s\n", e.c_str());
    if (envFlag("U15_TRACE"))
      for (const NeesSample& n : r.nees) {
        Eigen::Vector3d corr = Eigen::Vector3d::Constant(kNaN);   // odometry output − global output (A61 F13)
        for (const RunResult::TrajSample& q : r.traj)
          if (q.t == n.t) corr = q.est_odo - q.est_N;
        std::printf("   trace t %.1f zs_b %+.2f zs_d %+.2f err_b %+.3f sig_b %.3f | ant err %+.3f %+.3f %+.3f sig %.3f %.3f %.3f yaw err %+.5f sig %.5f"
                    " | odo-glob %+.3f %+.3f %+.3f | xi NEES %.1f zs phi %+.2f %+.2f psi %+.2f t %+.2f %+.2f %+.2f\n",
                    n.t, n.zs_b, n.zs_d, n.err_b, n.sigma_b, n.ant_err_N(0), n.ant_err_N(1), n.ant_err_N(2), n.sigma_ant(0),
                    n.sigma_ant(1), n.sigma_ant(2), n.yaw_err, n.yaw_sigma, corr(0), corr(1), corr(2), n.nees_xi, n.zs_xi[0],
                    n.zs_xi[1], n.zs_xi[2], n.zs_xi[3], n.zs_xi[4], n.zs_xi[5]);
      }
    rr.push_back(std::move(r));
  }
  // rows per class
  {
    std::array<RowCounts, 3> tot{};
    for (const RunResult& r : rr)
      for (size_t c = 0; c < 3; ++c) {
        tot[c].candidates += r.rows[c].candidates;
        tot[c].accepted += r.rows[c].accepted;
        for (size_t q = 0; q < tot[c].rejected.size(); ++q) tot[c].rejected[q] += r.rows[c].rejected[q];
      }
    const char* cn[3] = {"psr", "dopp", "tdcp"};
    for (size_t c = 0; c < 3; ++c) {
      std::printf("   rows %s: candidates %ld accepted %ld;", cn[c], tot[c].candidates, tot[c].accepted);
      for (size_t q = 0; q < tot[c].rejected.size(); ++q)
        if (tot[c].rejected[q] > 0) std::printf(" %s %ld", rejectReasonName(static_cast<RejectReason>(q)), tot[c].rejected[q]);
      std::printf("\n");
    }
  }
  // ATE proxy (A61): SE(3)-aligned position ATE of the host (U) and of the global estimate (T) per run.
  // U15_ATE_SKIP=<s>: recomputed without the first s seconds after activation (the registration's convergence).
  {
    const double skip = std::getenv("U15_ATE_SKIP") ? std::atof(std::getenv("U15_ATE_SKIP")) : 0.0;
    if (skip > 0.0)
      for (RunResult& r : rr) {
        std::vector<Eigen::Vector3d> hw, en, eo, tr;
        for (const RunResult::TrajSample& q : r.traj)
          if (q.t >= r.t_activate + skip) {
            hw.push_back(q.host_W);
            en.push_back(q.est_N);
            eo.push_back(q.est_odo);
            tr.push_back(q.truth);
          }
        r.ate_u = hw.size() >= 3 ? ateSe3(hw, tr) : kNaN;
        r.ate_t = en.size() >= 3 ? ateSe3(en, tr) : kNaN;
        r.ate_o = eo.size() >= 3 ? ateSe3(eo, tr) : kNaN;
      }
    std::vector<double> au, at, ao, ratio, ratio_o;
    for (const RunResult& r : rr)
      if (std::isfinite(r.ate_u) && std::isfinite(r.ate_t)) {
        au.push_back(r.ate_u);
        at.push_back(r.ate_t);
        ratio.push_back(r.ate_t / r.ate_u);
        if (std::isfinite(r.ate_o)) {
          ao.push_back(r.ate_o);
          ratio_o.push_back(r.ate_o / r.ate_u);
        }
      }
    auto med = [](std::vector<double> v) {
      if (v.empty()) return kNaN;
      std::sort(v.begin(), v.end());
      return v.size() % 2 ? v[v.size() / 2] : 0.5 * (v[v.size() / 2 - 1] + v[v.size() / 2]);
    };
    if (!au.empty()) {
      std::printf("   ATE (SE(3), antenna): U median %.3f m [%.3f, %.3f]; T median %.3f m [%.3f, %.3f]; T/U median %.2f (n %zu)\n",
                  med(au), *std::min_element(au.begin(), au.end()), *std::max_element(au.begin(), au.end()), med(at),
                  *std::min_element(at.begin(), at.end()), *std::max_element(at.begin(), at.end()), med(ratio), au.size());
      if (!ao.empty())
        std::printf("   ATE odometry output (traj_N_odom, A61 F13): T median %.3f m [%.3f, %.3f]; T/U median %.2f\n", med(ao),
                    *std::min_element(ao.begin(), ao.end()), *std::max_element(ao.begin(), ao.end()), med(ratio_o));
      for (size_t k = 0; k < rr.size(); ++k)
        std::printf("   ATE run %zu: U %.3f T %.3f T_odo %.3f (time offset %+.1f ms)\n", k, rr[k].ate_u, rr[k].ate_t, rr[k].ate_o,
                    rr[k].toff_true_s * 1e3);
    }
  }
  // slip flags (U15_CFG satcsv=1): detector mix, elevation and time since activation of the flagged TDCP rows
  {
    std::map<int, int> by_det, by_el, by_t;
    int n = 0;
    for (size_t k = 0; k < rr.size(); ++k) {
      const double t0 = static_cast<double>(baseParams(s, static_cast<uint32_t>(seed0 + static_cast<int>(k)), path).t0.sec);
      for (const SlipFlag& f : rr[k].slip_flags) {
        ++n;
        for (int d = 0; d < 6; ++d)
          if (f.detectors & (1u << d)) ++by_det[d + 1];
        ++by_el[static_cast<int>(f.el_deg / 15.0)];
        ++by_t[static_cast<int>((f.t_gpst - t0 - rr[k].t_activate) / 30.0)];
        if (n <= 12)
          std::printf("   slip flag run %zu t %+.1f s after ACTIVATE sat %d el %.1f° detectors 0x%02x innov %+.4f m sigma %.4f m w %.1f\n", k,
                      f.t_gpst - t0 - rr[k].t_activate, f.sat, f.el_deg, f.detectors, f.innov_m, f.sigma_m, f.w);
      }
    }
    if (n > 0) {
      std::printf("   slip flags %d: by detector", n);
      for (const auto& kv : by_det) std::printf(" D%d %d", kv.first, kv.second);
      std::printf(" | by elevation");
      for (const auto& kv : by_el) std::printf(" %d-%d° %d", kv.first * 15, kv.first * 15 + 15, kv.second);
      std::printf(" | by time since ACTIVATE");
      for (const auto& kv : by_t) std::printf(" %d-%d s %d", kv.first * 30, kv.first * 30 + 30, kv.second);
      std::printf("\n");
    }
  }
  // the initialiser: z² of the first sample of every run
  {
    double z[5] = {0, 0, 0, 0, 0};
    int n = 0;
    for (const RunResult& r : rr) {
      if (r.nees.empty()) continue;
      const NeesSample& f = r.nees.front();
      for (size_t i = 0; i < 3; ++i) z[i] += f.z2_ant[i];
      z[3] += f.zs_b * f.zs_b;
      z[4] += f.z2_xi[2];
      ++n;
    }
    if (n > 0)
      std::printf("   first-sample z2: E %.2f N %.2f U %.2f b %.2f psi %.2f (n %d)\n", z[0] / n, z[1] / n, z[2] / n, z[3] / n, z[4] / n, n);
  }
  const ConsistencyStats c = aggregate(rr);
  std::printf("[U15] %s: %s\n", scenarioName(s), c.describe().c_str());
  // ANEES/dof per 20 s since activation
  std::map<int, std::array<double, 8>> bins;
  for (const RunResult& r : rr)
    for (const NeesSample& n : r.nees) {
      auto& b = bins[static_cast<int>((n.t - r.t_activate) / 20.0)];
      const double v[4] = {n.nees_xi / 6.0, n.nees_ant / 3.0, n.nees_b, n.nees_d};
      for (size_t i = 0; i < 4; ++i)
        if (std::isfinite(v[i])) {
          b[i] += v[i];
          b[4 + i] += 1.0;
        }
    }
  for (const auto& kv : bins)
    std::printf("   since-activation %3d-%3d s: xi %.2f ant %.2f b %.2f d %.2f (n %.0f)\n", kv.first * 20, kv.first * 20 + 20,
                kv.second[0] / kv.second[4], kv.second[1] / kv.second[5], kv.second[2] / kv.second[6], kv.second[3] / kv.second[7],
                kv.second[4]);
  // per-component z² and signed means (filter − truth, /σ) per zbin since activation
  std::map<int, std::array<double, 40>> comp;
  for (const RunResult& r : rr)
    for (const NeesSample& n : r.nees) {
      auto& b = comp[static_cast<int>((n.t - r.t_activate) / zbin)];
      for (size_t i = 0; i < 6; ++i)
        if (std::isfinite(n.z2_xi[i])) {
          b[i] += n.z2_xi[i];
          b[20 + i] += 1.0;
        }
      for (size_t i = 0; i < 3; ++i)
        if (std::isfinite(n.z2_ant[i])) {
          b[6 + i] += n.z2_ant[i];
          b[9 + i] += n.zs_ant[i];
          b[26 + i] += 1.0;
        }
      if (std::isfinite(n.zs_b)) {
        b[12] += n.zs_b * n.zs_b;
        b[13] += n.zs_b;
        b[14] += n.err_b * n.err_b;
        b[15] += n.sigma_b * n.sigma_b;
        b[29] += 1.0;
      }
      if (std::isfinite(n.sigma_ant(2))) {
        b[16] += n.ant_err_N(2);
        b[17] += n.sigma_ant(2);
      }
      if (std::isfinite(n.zs_isbE)) {
        b[18] += n.zs_isbE * n.zs_isbE;
        b[30] += 1.0;
      }
    }
  for (const auto& kv : comp) {
    const auto& b = kv.second;
    const double t0 = kv.first * zbin, t1 = t0 + zbin;
    std::printf("   z2 %5.1f-%5.1f s: phi %.2f %.2f psi %.2f t %.2f %.2f %.2f | ant E %.2f N %.2f U %.2f | b %.2f | isbE %.2f\n", t0, t1,
                b[0] / b[20], b[1] / b[21], b[2] / b[22], b[3] / b[23], b[4] / b[24], b[5] / b[25], b[6] / b[26], b[7] / b[27],
                b[8] / b[28], b[12] / b[29], b[18] / b[30]);
    std::printf("   zs %5.1f-%5.1f s: ant E %+.2f N %+.2f U %+.2f | b %+.2f | rms e_b %.3f m, rms sig_b %.3f m | U mean err %+.3f m, "
                "mean sigma %.3f m\n",
                t0, t1, b[9] / b[26], b[10] / b[27], b[11] / b[28], b[13] / b[29], std::sqrt(b[14] / b[29]), std::sqrt(b[15] / b[29]),
                b[16] / b[28], b[17] / b[28]);
  }
}

// A host gravity direction that is wrong by `bias_y_rad` and then held (no extra covariance).
// Returns the gate max |φ(t)−φ(first)|/σ(t), the TDCP NIS per dof, and the final |φ−φ*|/σ
// (z² of φ_y; φ* is the tilt that undoes C(ĝ) relative to the true frame).
struct LevOut {
  double max_ratio = 0, nis_dof = 0, final_z2_y = 0, final_phi_y_deg = 0, final_sig_y_deg = 0;
  bool ok = false;
};
LevOut runLevelBias(double bias_y_rad, double sigma_lev, double path_m) {
  LevOut o;
  SimParams p = baseParams(Scenario::OpenSky, 1, path_m);
  if (p.nav_rinex.empty()) return o;
  p.draw_dov = false;
  p.level_bias = Eigen::Vector2d(0.0, bias_y_rad);
  p.eval_dt_s = 2.0;
  Config cfg = simConfig(p);
  cfg.frames.sigma_lev_rad = sigma_lev;
  cfg.process.frame.q_tilt_per_m = 4.837802525912528e-08;
  const RunResult r = runScenario(p, cfg);
  if (!r.activated || r.nees.empty()) return o;
  o.ok = true;
  o.nis_dof = r.nis[2].meanPerDof();
  const Eigen::Vector2d phi0 = r.nees.front().tilt;
  for (const NeesSample& n : r.nees) {
    for (int a = 0; a < 2; ++a) {
      if (!(n.tilt_sigma(a) > 0.0)) continue;
      o.max_ratio = std::max(o.max_ratio, std::fabs(n.tilt(a) - phi0(a)) / n.tilt_sigma(a));
    }
  }
  const NeesSample& last = r.nees.back();
  o.final_z2_y = last.z2_xi[1];
  o.final_phi_y_deg = last.tilt(1) * 180.0 / M_PI;
  o.final_sig_y_deg = last.tilt_sigma(1) * 180.0 / M_PI;
  std::printf("[U15] level bias %.2f deg  σ_lev %.3f deg  path %.0f m  max|Δφ|/σ %.2f  NIS/dof %.2f  φ_y %.3f°  σ_y %.3f°  z2_y %.2f\n",
              bias_y_rad * 180.0 / M_PI, sigma_lev * 180.0 / M_PI, path_m, o.max_ratio, o.nis_dof, o.final_phi_y_deg, o.final_sig_y_deg,
              o.final_z2_y);
  return o;
}

// Held host-g direction error on OpenSky (120 m, turns, ~3 cm/s vertical).
// The 0.03° prior does not reproduce the ZRR gate failure: φ_y moves ~0.03°, |Δφ|/σ stays below 1,
// TDCP NIS/dof stays ~1, and z² against the host-g leveling stays ~1 (the 0.7° remains in C(g)).
// The BMI088-sized prior (0.021824 rad) lets φ move ~0.3° and then |Δφ|/σ exceeds 1, because σ
// falls back toward the random-walk scale. A 4° error still exceeds that prior.
TEST(U15GroupA, LevelingBiasGateAndPrior) {
  if (grass02Brdc().empty()) GTEST_SKIP() << "Grass02 BRDC fixture not present";
  const double bias = 0.7 * kDeg;
  const LevOut tight = runLevelBias(bias, 0.0, 120.0);
  ASSERT_TRUE(tight.ok);
  EXPECT_LT(tight.max_ratio, 1.0);
  EXPECT_LT(std::fabs(tight.final_phi_y_deg), 0.1);
  EXPECT_NEAR(tight.nis_dof, 1.0, 0.25);
  EXPECT_LT(tight.final_z2_y, 4.0);
  const LevOut wide = runLevelBias(bias, 0.021824, 120.0);
  ASSERT_TRUE(wide.ok);
  EXPECT_GT(wide.max_ratio, 1.5);
  EXPECT_NEAR(wide.nis_dof, 1.0, 0.25);
  const LevOut big = runLevelBias(4.0 * kDeg, 0.021824, 120.0);
  ASSERT_TRUE(big.ok);
  EXPECT_GT(big.max_ratio, 1.0);
  EXPECT_LT(big.nis_dof, 2.0);
}

// A 20 mg body-y bias absorbed into g at init, on the simplified host (IMU propagation + pose update).
// The stand-in's tight prior is σ_g = 5e-3 m/s², not the host's 1e-5, so g is not frozen: the first
// run left 0.444° and z²/3 = 278. The BMI088 prior left 0.044° and z²/3 = 3.77. The bars below lock that.
TEST(U15GroupA, TurnonBiasHostGravity) {
  if (grass02Brdc().empty()) GTEST_SKIP() << "Grass02 BRDC fixture not present";
  auto once = [](bool prior) {
    SimParams p = baseParams(Scenario::OpenSky, 3, 120.0);
    p.draw_dov = false;
    p.turnon_ba = Eigen::Vector3d(0.0, 0.020 * 9.81, 0.0);
    p.host_turnon_prior = prior;
    p.eval_dt_s = 1.0;
    return runScenario(p, simConfig(p));
  };
  const RunResult tight = once(false);
  const RunResult wide = once(true);
  ASSERT_TRUE(tight.error.empty()) << tight.error;
  ASSERT_TRUE(wide.error.empty()) << wide.error;
  ASSERT_TRUE(tight.activated && wide.activated);
  std::printf("[U15] turn-on 20 mg  tight |g| %.3f deg  z2 %.2f   prior |g| %.3f deg  z2 %.2f  block z2 g %.2f / %.2f\n",
              tight.host_g_err_deg, tight.host_g_z2_end, wide.host_g_err_deg, wide.host_g_z2_end,
              tight.host_block_z2[5], wide.host_block_z2[5]);
  EXPECT_GT(tight.host_g_err_deg, 0.40);
  EXPECT_GT(tight.host_g_z2_end, 50.0);
  EXPECT_LT(wide.host_g_err_deg, 0.10);
  EXPECT_LT(wide.host_g_z2_end, 16.0);
  EXPECT_LT(wide.host_g_err_deg, 0.25 * tight.host_g_err_deg);
}

// Real ENU trajectory + the day's broadcast ephemeris at the real site and start time.
// FL2G_REPLAY_TUM, FL2G_REPLAY_NAV, FL2G_REPLAY_LAT, FL2G_REPLAY_LON, FL2G_REPLAY_H,
// FL2G_REPLAY_T0 (GTime seconds, same scale as the other U15 scenes). Skipped in ordinary ctest.
TEST(U15GroupA, ReplayTrajectoryLevelingBias) {
  const char* tum = std::getenv("FL2G_REPLAY_TUM");
  const char* nav = std::getenv("FL2G_REPLAY_NAV");
  const char* lat = std::getenv("FL2G_REPLAY_LAT");
  const char* lon = std::getenv("FL2G_REPLAY_LON");
  const char* hh = std::getenv("FL2G_REPLAY_H");
  const char* t0s = std::getenv("FL2G_REPLAY_T0");
  if (!tum || !nav || !lat || !lon || !hh || !t0s) GTEST_SKIP() << "set FL2G_REPLAY_* to run a trajectory replay";
  SimParams p = baseParams(Scenario::OpenSky, 1, 100.0);
  p.nav_rinex = nav;
  p.replay_enu_tum = tum;
  p.draw_dov = false;
  p.gnss_dt = 0.1;
  p.eval_dt_s = 5.0;
  p.site.lat_rad = std::atof(lat) * kDeg;
  p.site.lon_rad = std::atof(lon) * kDeg;
  p.site.h_m = std::atof(hh);
  p.t0.sec = static_cast<int64_t>(std::atof(t0s));
  p.t0.frac = std::atof(t0s) - static_cast<double>(p.t0.sec);
  p.level_bias = Eigen::Vector2d(0.0, -0.7 * kDeg);
  Config cfg = simConfig(p);
  cfg.frames.sigma_lev_rad = 0.0;
  cfg.process.frame.q_tilt_per_m = 4.837802525912528e-08;
  const RunResult r = runScenario(p, cfg);
  ASSERT_TRUE(r.activated) << r.error;
  ASSERT_FALSE(r.nees.empty());
  const Eigen::Vector2d phi0 = r.nees.front().tilt;
  double max_ratio = 0.0;
  for (const NeesSample& n : r.nees)
    for (int a = 0; a < 2; ++a)
      if (n.tilt_sigma(a) > 0.0)
        max_ratio = std::max(max_ratio, std::fabs(n.tilt(a) - phi0(a)) / n.tilt_sigma(a));
  const NeesSample& last = r.nees.back();
  std::printf("[U15] replay %s  dur %.0f s  max|Δφ|/σ %.2f  NIS/dof %.2f  φ %.3f/%.3f deg  σ %.3f/%.3f deg  z2 %.2f/%.2f\n",
              tum, r.duration_s, max_ratio, r.nis[2].meanPerDof(), last.tilt(0) * 180.0 / M_PI, last.tilt(1) * 180.0 / M_PI,
              last.tilt_sigma(0) * 180.0 / M_PI, last.tilt_sigma(1) * 180.0 / M_PI, last.z2_xi[0], last.z2_xi[1]);
  EXPECT_TRUE(std::isfinite(max_ratio));
}
