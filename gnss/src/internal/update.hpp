// SPDX-License-Identifier: BSD-3-Clause
// JointUpdater — H3 joint (19+n)-dim covariance-form IEKF (DESIGN §4.8) with the L4/L5 gates (§10):
//   1) snapshot (x⁻, x_G⁻, Σ⁻); 2) iterate: rows at x_i → r_i, H_i, R; S = H_i Σ H_iᵀ + R (LDLT, never Σ⁻¹);
//   K = Σ H_iᵀ S⁻¹; x_{i+1} = x⁻ ⊞ K[r_i + H_i(x_i ⊟ x⁻)]; 3) iteration rule: σ_ψ > 2° or first |Δψ| > 0.1°
//   → ≤ 3; fixed 3 for reacq_epochs after ACTIVATE/RECOVERY; else single step; 4) gates on r_0, S_0 only;
//   5) Joseph Σ⁺ = (I−KH)Σ(I−KH)ᵀ + KRKᵀ, symmetrised; 6) L5 post-check → rollback to the snapshot;
//   7) write back host (R,p,τ,v,b_g,b_a,g,cov) and x_G, C, Q_G.
// L4: per-row Baarda w_i = (Wν)_i/√W_ii with W = S⁻¹, α = 1e-3; epoch NIS νᵀWν vs χ²_m(1−α); drop argmax|w_i|
// and recompute until pass or m < m_min; >50 % dropped ⇒ class skipped this epoch. TDCP: the common Δclk is a
// nuisance parameter, W = S⁻¹ − S⁻¹1(1ᵀS⁻¹1)⁻¹1ᵀS⁻¹ (Δclk-invariant, dof m − 1; equals Baarda on the
// Householder-projected rows) — registered deviation from the literal §10 L4 "先减稳健 Δclk" (review M1).
// Schmidt mode (L7): host columns of K zeroed and the host (mean and P) restored bit for bit after the commit;
// only C and Q_G change. Consider columns (AMENDMENTS A66, Options::consider): the same idea for a subset of rows × a
// subset of augmented columns (the Doppler rows do not update the tilt φ). Projected rows (AMENDMENTS A70,
// Options::project): a class's rows made insensitive to a slot by the TDCP Δclk construction (the Doppler rows lose
// their tilt content). No IRLS, no R inflation, no step scaling inside (R1-3/R1-4).
// Owner: WP-c. Tests: test_u10_updater.cpp (linear-Gaussian == batch LS; Joseph PSD; χ² thresholds;
// rollback bit-exact; conditional iteration), test_u12_gating.cpp.
#pragma once

#include <array>
#include <functional>
#include <vector>

#include <Eigen/Core>

#include "livo_gnss/config.hpp"
#include "livo_gnss/core_state.hpp"
#include "livo_gnss/diagnostics.hpp"
#include "aug_block.hpp"
#include "rows.hpp"

namespace livo_gnss {

// χ² helpers (Wilson–Hilferty approximation is NOT acceptable; use an exact inverse regularised gamma).
double chi2Quantile(int dof, double p);           // quantile of χ²_dof at probability p (U10 checks tabulated values)
double normalQuantile(double p);                  // two-sided w threshold: normalQuantile(1 − α/2) = 3.29 at α = 1e-3

struct GateResult {
  int n_in = 0, n_out = 0;
  int n_rejected_w = 0, n_rejected_nis = 0;
  double nis = 0.0;
  int dof = 0;
  bool class_skipped = false;                     // majority rejected or below m_min
  std::vector<double> w;                          // per input row (0 for rejected)
};

class JointUpdater {
 public:
  JointUpdater(const Config& cfg);

  // Callback that rebuilds all row sets at an iterate (the epoch's rows depend on x through h and H).
  using RowFactory = std::function<void(const CoreState& x, const AugBlock& aug, std::vector<RowSet>* sets)>;

  struct Options {
    bool force_three_iterations = false;   // ACTIVATE/RECOVERY reacquisition window
    bool schmidt = false;                  // L7 LIVO_ALARM: do not write host columns
    bool disable_l4_l5 = false;            // T−reject arm
    bool observe_only = false;             // OBSERVE: compute everything, commit nothing
    double sigma_prior_pos_m = 0.0;        // for the L5 step check
    // Appended (AMENDMENTS A66; INTERFACE_CHANGES): consider (Schmidt) columns per measurement class — L7's "host
    // columns of K zeroed" generalised to a subset of rows × a subset of augmented columns. For each entry the rows of
    // class `cls` do not update the columns of slot `slot` (tag 0): their gain onto those columns is zero, so the slot's
    // mean does not move with them, while the slot's uncertainty still enters every S through H (no row is modelled as
    // more precise than it is). The slot's gain from the rows it may use, A, is the constrained optimum
    // Σ_{s,:} H_Aᵀ S_AA⁻¹ (minimum variance of the slot given that its update may depend on those rows only; the same
    // as processing A first with the full gain and the other rows next with the slot considered); every other column
    // keeps the joint optimum ΣHᵀS⁻¹ (the Schmidt–Kalman gain of the estimated states); Σ⁺ is the Joseph form of the gain
    // actually used, exact for any gain. An entry whose slot is absent, or whose class has no accepted row this epoch,
    // leaves the update bit-identical to an empty list. Host columns are L7's (`schmidt`), not addressed here.
    struct Consider {
      MeasClass cls = MeasClass::Dopp;
      Slot slot = Slot::Tilt;
    };
    std::vector<Consider> consider;
    // Appended (AMENDMENTS A69/A70; INTERFACE_CHANGES): the rows of a class projected onto the orthogonal complement of a
    // slot's columns — the Doppler rows made tilt-free the way the TDCP rows are made Δclk-free. For every class named
    // by an entry, the accepted rows of that class (one set per epoch; R has no cross terms to other rows, one block per
    // set) are whitened with their R (R = L Lᵀ) and projected onto the orthogonal complement of the range of the
    // whitened columns of every slot (tag 0) listed with the class: m rows → m − r unit-variance rows, r the numerical
    // rank of those columns (projection() below). The kept rows do not depend on the slot (their columns are zero up to
    // rounding): they carry no information on it and cannot make any other state correlated with it, so no later update
    // (H2 after the LIO) can pass their content on to it either — what a consider entry cannot stop (A67). Their
    // information on every other state is the original rows' with the slot a free nuisance parameter (no prior): the
    // batch least-squares elimination. What the rows said along the slot's directions — for the tilt and level motion
    // mostly the vertical velocity — is dropped. L4 gates the original rows (before this step); the per-row gate output,
    // gnss_sats.csv and the per-class statistics are unchanged; the gain, L5 (dof m − r) and the Joseph form see the
    // projected rows. No row of the class, no such slot, or rank 0 ⇒ bit-identical to an empty list; m ≤ r ⇒ the class
    // contributes nothing this epoch. The projected rows keep their class, so a consider entry for the same pair still
    // zeroes the slot's gain from them (the part the slot's prior correlations would give it).
    struct Project {
      MeasClass cls = MeasClass::Dopp;
      Slot slot = Slot::Tilt;
    };
    std::vector<Project> project;
  };

  struct Result {
    bool committed = false;
    bool rolled_back = false;
    int iterations = 0;
    std::array<GateResult, static_cast<size_t>(MeasClass::Count)> gates{};
    double nis_post = 0.0;
    double f_p = 0.0, f_xi = 0.0, f_clk = 0.0, f_v = 0.0;   // gain shares (§5 D3)
    double dp_host_norm = 0.0, dyaw = 0.0, dt_norm = 0.0;
    HostVec dx_host = HostVec::Zero();
    Eigen::VectorXd dx_aug;
    std::string rollback_reason;
    // The row sets after gating, with MeasRow::reason / accepted / nu / R filled and the per-row w in
    // gates[cls].w (same order as rows). gnss_sats.csv is written from this (m4).
    std::vector<RowSet> gated;
  };

  // Runs one epoch's joint update in place on (core, aug). On rollback both are bit-identical to entry.
  Result update(CoreState& core, AugBlock& aug, const RowFactory& rows, const Options& opt) const;

  // L4 on one class: returns the accepted rows (marks MeasRow::reason / accepted on the input rows).
  GateResult gateClass(RowSet& set, const Eigen::MatrixXd& Sigma, int m_min, double alpha_w, double alpha_nis,
                       double max_reject_frac) const;
  // Joseph form update of Σ with rows (H, R); returns K.
  static Eigen::MatrixXd josephUpdate(Eigen::MatrixXd& Sigma, const Eigen::MatrixXd& H, const Eigen::MatrixXd& R, const Eigen::MatrixXd& K);
  // L5 posterior NIS rᵀ(R − HΣ⁺Hᵀ)⁻¹ r.
  static double posteriorNis(const Eigen::VectorXd& r, const Eigen::MatrixXd& H, const Eigen::MatrixXd& R, const Eigen::MatrixXd& Sigma_post);
  // Appended (AMENDMENTS A64; INTERFACE_CHANGES): the L5 "Σ⁺ positive semi-definite" test. Every diagonal element finite
  // and ≥ 0, and every eigenvalue of the Jacobi-scaled matrix D Σ D (D = diag(1/√Σ_ii), unit diagonal — Σ mixes rad², m²,
  // (m/s)², clock m²) ≥ −tol. A stopped clone made without noise — the Origin slot at ACTIVATE/REANCHOR (A61 F13) — is an
  // exact linear function of other states, so the joint covariance is PSD with exact zero directions, which the Joseph
  // update leaves at the rounding level. The pre-A64 test (Eigen LDLT: info() == Success and isPositive()) failed on such
  // matrices by construction: a rounding-level negative pivot flips isPositive(), and an exact zero pivot followed by a
  // non-zero one fails info() (the LDLT picks pivots on the unreduced diagonal); it rolled back the activation epoch's
  // update (A62 ⑤). Diagonal scaling keeps the inertia, so tol = 0 would test the same property; kPsdEigTol admits only
  // rounding-level negatives (≤ 1e-9 of a variance).
  static constexpr double kPsdEigTol = 1e-9;
  static bool posteriorPsd(const Eigen::MatrixXd& Sigma, double tol = kPsdEigTol);
  // Appended (AMENDMENTS A70; INTERFACE_CHANGES): the projection behind Options::project, exposed for U7/U10. Rows H
  // (m × N), r (m), R (m × m, positive definite) and state columns `cols`: with R = L Lᵀ and F = L⁻¹ H_{:,cols}, the kept
  // rows are *Hp = T H and *rp = T r with T = Q₂ᵀ L⁻¹, Q₂ an orthonormal basis of range(F)^⊥ — so T R Tᵀ = I and
  // T H_{:,cols} = 0 up to rounding; *rank = r = the numerical rank of F (m − r rows kept, none when m ≤ r). Full column
  // rank: Q₂ from the Householder QR of F, the construction of the TDCP Δclk projection (and, for the tilt, bit for bit
  // the A67 diagnostic build's). Rank-deficient — the tilt columns of the Doppler rows are exactly proportional when the
  // velocity is level, and both vanish when it is zero: Q₂ = the left singular vectors of F beyond the rank (a Householder
  // basis of F would also take out a direction that the rounding of the dependent column makes up). A singular value
  // s_k ≤ kProjRankTol·s_1 counts as zero, and so does s_1 ≤ kProjRankTol·‖L⁻¹H‖_F (rank 0: *Hp and *rp are left
  // untouched, the caller keeps the rows as they are). Returns false only when R is not positive definite.
  static constexpr double kProjRankTol = 1e-10;
  static bool projection(const Eigen::MatrixXd& H, const Eigen::VectorXd& r, const Eigen::MatrixXd& R, const std::vector<int>& cols,
                         Eigen::MatrixXd* Hp, Eigen::VectorXd* rp, int* rank);

 private:
  const Config& cfg_;
};

}  // namespace livo_gnss
