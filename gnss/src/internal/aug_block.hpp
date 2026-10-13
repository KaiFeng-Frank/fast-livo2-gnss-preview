// SPDX-License-Identifier: BSD-3-Clause
// AugBlock — the augmented GNSS state x_G with named slots and the joint covariance bookkeeping
// Σ = [[P, C], [Cᵀ, Q_G]] (DESIGN §2.2–§2.4). P belongs to the host; C (19×n) and Q_G (n×n) live here.
//   H1 propagate:   C ← F_tot C Φ_Gᵀ,  Q_G ← Φ_G Q_G Φ_Gᵀ + Q_G(Δs, Δα, Δt),  x_G ← Φ_G x_G
//   H2 condition:   A = Cᵀ(P⁻)⁻¹ (Jacobi scaling + LDLT; eigen pseudo-inverse fallback → event),
//                   x_G ← x_G + A(x⁺ ⊟ x⁻),  C ← P⁺Aᵀ,  Q_G ← Q_G − A(P⁻ − P⁺)Aᵀ, symmetrised
//   slots:          consistent augmentation y = f(x, n): P_yx = F_x P, P_yy = F_x P F_xᵀ + F_n Σ_n F_nᵀ;
//                   removing a slot = deleting its rows/columns (marginalisation in covariance form)
//   re-pivot (§2.3, completed in AMENDMENTS A11, exact columns A34): o' = a^W, d = o' − o, t' = t + M d; the
//                   error-state map is y = T x over [host 19 | aug n] with T = I except the t rows:
//                   ∂t'/∂δψ = [e_z]× M d,  ∂t'/∂δφ = Ξ(o') J_l(φ̃) E_xy,
//                   ∂t'/∂δg = Ξ(o') Exp(φ̃) [e_z]× G_⊥ (host cols, gravity_est_en),
//                   Ξ(o') = −R_z(ψ)[Exp(φ̃) C(g) d]×, Exp = Leveling::expTilt, J_l = Leveling::tiltLeftJacobian;
//                   Σ' = T Σ Tᵀ needs P_host for the δg coupling (M3); the global antenna position and its
//                   covariance are unchanged, φ ≠ 0 included (U9 < 1e-9)
// Process noise of the registration walk (§3.2): Q_ξ = J_rw diag(q_φΔs, q_φΔs, q_ψΔs + q_ψrΔα, q_thΔs,
// q_thΔs, q_tvΔs) J_rwᵀ with Δt = [m − t]× Δr coupling about the pivot, Δr = R_z(ψ) J_l(φ̃) E_xy w_φ + e_z w_ψ (the
// exact rotation increment of an additive w_φ, AMENDMENTS A45 / R6; §3.2 wrote its φ = 0 form R_z E_xy w_φ).
// Owner: WP-b. Tests: test_u08_hooks.cpp (joint filter vs split, < 1e-12), test_u09_augblock.cpp.
#pragma once

#include <string>
#include <vector>

#include <Eigen/Core>

#include "livo_gnss/config.hpp"
#include "livo_gnss/core_state.hpp"
#include "livo_gnss/diagnostics.hpp"
#include "leveling.hpp"

namespace livo_gnss {

// Phase (appended, AMENDMENTS A61 F11; INTERFACE_CHANGES): the carrier-phase noise ε_j(k) of the epoch shared by two
// chained TDCP pairs, one component per satellite (tag = the id of the pair that starts there); constant, no noise.
// Origin (appended, A61 F13): [a^N; ψ] at ACTIVATE — the antenna and the yaw of that instant as later information
// revises them (a stopped clone, no noise); the odometry output removes their revision (GnssFusion::globalPoseOdom).
enum class Slot : uint8_t { Tilt = 0, Yaw, Trans, Clk, Isb, Clone, Td, Phase, Origin };
const char* slotName(Slot s);

struct SlotInfo {
  Slot kind = Slot::Tilt;
  int tag = 0;                // Isb: system bit; Clone, Phase: pair id
  int offset = 0;             // first column inside x_G
  int dim = 0;
  std::string name;           // "tilt", "yaw", "trans", "clk", "isb_C", "clone_17", "td"
};

class AugBlock {
 public:
  explicit AugBlock(const Config& cfg);

  int dim() const { return static_cast<int>(x_.size()); }
  const std::vector<SlotInfo>& slots() const { return slots_; }
  // Column offset of a slot, or -1 when absent.
  int find(Slot kind, int tag = 0) const;
  bool has(Slot kind, int tag = 0) const { return find(kind, tag) >= 0; }

  const Eigen::VectorXd& x() const { return x_; }
  Eigen::VectorXd& x() { return x_; }
  const Eigen::MatrixXd& Q() const { return Q_; }
  Eigen::MatrixXd& Q() { return Q_; }
  const Eigen::Matrix<double, HostIdx::dim, Eigen::Dynamic>& C() const { return C_; }
  Eigen::Matrix<double, HostIdx::dim, Eigen::Dynamic>& C() { return C_; }

  // Consistent augmentation of a new slot (returns its index in slots()). F_host: dim×19 Jacobian of the
  // new state w.r.t. the host error state; F_aug: dim×n w.r.t. the existing block; N: F_n Σ_n F_nᵀ (dim×dim).
  int addSlot(Slot kind, int tag, const std::string& name, const Eigen::VectorXd& x0,
              const Eigen::MatrixXd& F_host, const Eigen::MatrixXd& F_aug, const Eigen::MatrixXd& N, const HostMat& P_host);
  // Delete a slot (rows/columns dropped; everything else bit-unchanged, U9).
  void removeSlot(int slot_index);
  void clear();

  // H1. Φ_G and Q_proc are built by processModel(); F_tot from the host.
  void propagate(const HostMat& F_tot, const Eigen::MatrixXd& Phi_G, const Eigen::MatrixXd& Q_proc);
  // Process model over one LIO step: Δs path length, Δα yaw path, Δt time (§3.1–§3.2). The pivot coupling
  // Δt = [m − t]× Δr uses u = M(a^W − o^W) with o^W = `pivot_W` (M2); `omega_z_abs_dt` = Σ|ω_z|dt of the
  // step for the wind-up lower bound of S_b (R2-13).
  void processModel(double ds, double dalpha, double dt, const Eigen::Matrix3d& M, const Eigen::Vector3d& a_W,
                    const Eigen::Vector3d& pivot_W, double lambda_windup_m, double omega_z_abs_dt,
                    Eigen::MatrixXd* Phi_G, Eigen::MatrixXd* Q_proc) const;

  // H2. `dx = x⁺ ⊟ x⁻` in the host tangent space. Returns false only when even the pseudo-inverse failed.
  bool conditionOnHostUpdate(const HostMat& P_prior, const HostMat& P_post, const HostVec& dx, bool* used_pseudoinverse);

  // §2.3 re-pivot (A11, A34): o' = a_W. `leveling` (with its C_ref) and `g_W` build M and Ξ(o'), `P_host` enters
  // the δg coupling of Σ' = T Σ Tᵀ; `gravity_est_en=false` drops the δg column. Returns the transform T
  // restricted to the aug rows ([n × (19+n)]) so U9 can check the global antenna covariance is invariant. The δg
  // column is the exact derivative when g_W is the gravity C_ref was last set from (Leveling::epsJacobian); the facade
  // meets that by re-pivoting before the step's GNSS updates (onStepEnd has just refreshed C_ref, DESIGN §9).
  Eigen::MatrixXd repivot(const Eigen::Vector3d& pivot_old, const Eigen::Vector3d& a_W, const Leveling& leveling,
                          const Eigen::Vector3d& g_W, const HostMat& P_host, bool gravity_est_en);
  // Wrap the yaw slot into (−π, π].
  void wrapYaw();
  void symmetrize();

  // Joint covariance assembly/disassembly used by the updater (§4.8).
  Eigen::MatrixXd jointCov(const HostMat& P_host) const;
  void setFromJoint(const Eigen::MatrixXd& Sigma, HostMat* P_host);

  // Registration view (M is recomputed by the caller from Leveling).
  Eigen::Vector2d tilt() const;
  double yaw() const;
  Eigen::Vector3d trans() const;
  double clockBias() const;      // b [m]
  double clockDrift() const;     // d [m/s]
  bool isb(Sys sys, double* value, double* sigma) const;
  Eigen::VectorXd sigma() const { return Q_.diagonal().cwiseMax(0.0).cwiseSqrt(); }

  // Δt_epoch of the clock wind-up bound used by processModel() (ClockModel::windup_epoch_dt_s; default 0.1 s, the
  // nominal M3DGR receiver interval). The glue may set the measured epoch interval. Appended by WP-b (INTERFACE_CHANGES).
  void setWindupEpochDt(double seconds) { windup_epoch_dt_s_ = seconds; }
  double windupEpochDt() const { return windup_epoch_dt_s_; }

  // FAST-LIVGO: restore the registration mean and drop its cross-covariance so H2 cannot walk a frozen
  // W→ENU. Process noise is removed by zeroing the frame random-walk intensities on that arm, not here,
  // so this translation unit's off path stays the T arm's.
  void pinRegistration(const Eigen::VectorXd& x_pin);
  // Appended (AMENDMENTS A75, livgo/frame_exact): tilt and translation become exact constants — mean restored to
  // x_pin, variance, covariance with every other state and (through GnssFusionLivgo's zeroed frame process noise) process
  // noise all zero, so neither block appears in any S and no gain can move them. The yaw slot is NOT touched: it stays an
  // ordinary constant state with its own variance and its correlations (it is the one registration parameter the
  // Doppler rows keep estimating). Rows and columns of a zero variance are exact zeros, which the Jacobi-scaled PSD test of
  // L5 (JointUpdater::posteriorPsd keeps scale 1 for a zero diagonal), the Joseph update and H2 all carry through unchanged.
  // AMENDMENTS A80 (livgo/freeze_yaw): with include_yaw the yaw slot is made an exact constant in the same way (mean restored,
  // variance and covariance zero, the zero process noise of the LIVGO arm leaves it so) — the registration is then three exact
  // numbers and no registration state is left to estimate. false = the A75 behaviour above.
  void pinRegistrationExact(const Eigen::VectorXd& x_pin, bool include_yaw = false);

 private:
  const Config& cfg_;
  std::vector<SlotInfo> slots_;
  Eigen::VectorXd x_;
  Eigen::MatrixXd Q_;
  Eigen::Matrix<double, HostIdx::dim, Eigen::Dynamic> C_;
  double windup_epoch_dt_s_ = 0.1;
};

}  // namespace livo_gnss
