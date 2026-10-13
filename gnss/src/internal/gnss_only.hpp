// SPDX-License-Identifier: BSD-3-Clause
// gnss_only — self-written GNSS-only estimators (DESIGN §8 COLLECT, §10 L1, §11 RECOVERY clock reset):
//   * RobustSpp: Gauss–Newton → least-median-of-squares consensus over minimal 4-satellite subsets (with
//     re-admission of consistent satellites) → IRLS-Huber weighted LS on pseudoranges → iterated Baarda
//     w-test, plus the C/N0–elevation NLOS pre-screen; outputs position, clock, ISBs, full covariance, GDOP,
//     used/rejected sets. The LMS stage in front of the design's "IRLS-Huber + iterated w-test" is a registered
//     deviation (INTERFACE_CHANGES WP-c, review m2a): Huber is leverage-blind and a 30 m blunder on a low
//     satellite drags the M-estimate by ~35 m before the w-test can see it (U16).
//   * RobustDopplerLs: velocity + clock drift from Doppler (same robust scheme).
//   * ClockOnlyLs: clock (b, d) with the antenna position fixed to the LIVO prediction (L2 / RECOVERY); b needs
//     a GPS/QZSS code (no ISB), otherwise ok = false.
// IRLS lives ONLY here (never inside the EKF, R1-4). RTKLIB pntpos is not used as a judge (its valsol
// χ² return is commented out, pntpos.c:384) — it is a reference row only (rtk::SppRef).
// Owner: WP-c (moved into this round, AMENDMENTS A9). Tests: test_u16_gnss_only.cpp (+30 m single
// outlier must be rejected; two simultaneous NLOS detected ≥95 %; clock-only LS), test_u13_init.cpp (P3).
#pragma once

#include <vector>

#include <Eigen/Core>

#include "livo_gnss/config.hpp"
#include "livo_gnss/types.hpp"
#include "quality.hpp"
#include "rtk_adapter.hpp"

namespace livo_gnss {

struct SppSolution {
  bool ok = false;
  GTime t;                                   // epoch (receiver time)
  Eigen::Vector3d r_ecef = Eigen::Vector3d::Zero();
  double b_m = 0.0;                          // GPS receiver clock bias [m]
  double isb_m[4] = {0, 0, 0, 0};            // [GLO, GAL, BDS, QZS] inter-system biases [m]; 0 if not estimated
  bool isb_est[4] = {false, false, false, false};
  Eigen::VectorXd x;                         // [r(3), b, ι...] full state
  Eigen::MatrixXd Q;                         // formal covariance × σ̂0² (floored at 3² m² per §8)
  double sigma0 = 0.0;                       // a-posteriori unit weight σ̂0
  double gdop = 0.0, pdop = 0.0;
  int n_used = 0, n_rejected = 0, iterations = 0;
  std::vector<SatId> used, rejected;
  std::vector<double> residuals;             // per used satellite [m]
  Geodetic geo;
};

struct DopplerSolution {
  bool ok = false;
  GTime t;
  Eigen::Vector3d v_ecef = Eigen::Vector3d::Zero();
  double d_mps = 0.0;                        // clock drift [m/s]
  Eigen::VectorXd x;                         // [v(3), d]
  Eigen::MatrixXd Q;
  double sigma0 = 0.0;
  int n_used = 0, n_rejected = 0;
  std::vector<SatId> used, rejected;
};

// Per-satellite prepared input (after L0 masks and satState).
struct SppInput {
  SatId sat;
  rtk::SatState state;
  double P_corr = 0.0;         // P − rtk::codeBiasMetres(sat, state, code) [m] (A37; NaN bias ⇒ caller sets psr_usable = false)
  double dopp_hz = 0.0;
  double lambda_m = 0.0;
  double cn0 = 0.0, el_rad = 0.0, az_rad = 0.0;
  double var_psr = 0.0, var_dopp = 0.0;   // from NoiseModel (a-priori)
  bool psr_usable = false, dopp_usable = false;
  double ion_m = 0.0, trop_m = 0.0;       // corrections applied at the current iterate (updated inside)
  // Appended by WP-c (INTERFACE_CHANGES.md): true keeps el/az/ion_m/trop_m exactly as given instead of
  // re-evaluating them through rtk_adapter at every iterate (synthetic geometry in U16, no WP-a dependency;
  // the production caller leaves it false). The solver treats the input as fixed only when every entry says so.
  bool models_fixed = false;
};

class RobustSpp {
 public:
  RobustSpp(const Config& cfg, const NoiseModel& noise);
  // Precondition: ≥ init.min_sats usable pseudoranges. Starts from `x0` (r_ecef, b) or from a
  // Bancroft/zero start when `x0_valid` is false. Iono/tropo are re-evaluated at every iterate.
  bool solve(GTime t, std::vector<SppInput>& in, const Klobuchar* ion, bool x0_valid, const Eigen::Vector3d& r0,
             double b0, SppSolution* out) const;
  // C/N0–elevation NLOS pre-screen (§8 COLLECT): flags satellites whose C/N0 is below the elevation
  // trend by more than init.nlos_cn0_margin_db.
  static int nlosPrescreen(std::vector<SppInput>& in, double margin_db);

 private:
  const Config& cfg_;
  const NoiseModel& noise_;
};

class RobustDopplerLs {
 public:
  RobustDopplerLs(const Config& cfg, const NoiseModel& noise);
  // Requires a position (r_ecef) for the line-of-sight and Sagnac-rate terms (§1 Doppler equation; the Sagnac rate is
  // the time derivative of the pseudorange's Sagnac term, AMENDMENTS A36 — the opposite sign of RTKLIB resdop).
  bool solve(GTime t, const std::vector<SppInput>& in, const Eigen::Vector3d& r_ecef, DopplerSolution* out) const;

 private:
  const Config& cfg_;
  const NoiseModel& noise_;
};

// Clock-only LS with the antenna position fixed (L2 non-integer jump, RECOVERY with σ_b > 1e3 m).
struct ClockOnlyResult {
  bool ok = false;
  double b_m = 0.0, d_mps = 0.0;
  double var_b = 0.0, var_d = 0.0;    // MAD²-based (§10 L2)
  int n_used = 0;
};
ClockOnlyResult clockOnlyLs(const std::vector<SppInput>& in, const Eigen::Vector3d& r_ecef_pred,
                            const Eigen::Vector3d& v_ecef_pred);

}  // namespace livo_gnss
