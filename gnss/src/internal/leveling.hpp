// SPDX-License-Identifier: BSD-3-Clause
// Leveling — yaw-free leveling C(g) of the W frame from the host gravity state (DESIGN §1 "配准"):
//   C(g) = Δ(C_ref·ĝ → −e_z) · C_ref,  Δ = minimal rotation between two vectors (Eigen FromTwoVectors),
//   C_ref refreshed at every step end with the final g (onStepEnd).
// First-order: C(g + δg) ≈ Exp(ε) C(g),  ε = [e_z]× C (I − ĝĝᵀ) δg / ‖g‖,  ε_z ≡ 0 (D1 §2.2; U7 checks
// ε_z = 0 to machine precision and the Jacobian by central differences).
// Owner: WP-b. Tests: test_u07_jacobians.cpp.
#pragma once

#include <Eigen/Core>

namespace livo_gnss {

class Leveling {
 public:
  Leveling();
  // Set/refresh the reference rotation from the current gravity (called at ACTIVATE and every onStepEnd).
  void setRef(const Eigen::Vector3d& g_W);
  const Eigen::Matrix3d& ref() const { return C_ref_; }
  bool hasRef() const { return has_ref_; }

  // C(g) with the stored C_ref. Precondition: hasRef().
  Eigen::Matrix3d C(const Eigen::Vector3d& g_W) const;
  // G_⊥ = C (I − ĝĝᵀ)/‖g‖ (3×3) and the ε Jacobian [e_z]× G_⊥ (3×3, third row zero). Both at the given g.
  Eigen::Matrix3d Gperp(const Eigen::Vector3d& g_W) const;
  Eigen::Matrix3d epsJacobian(const Eigen::Vector3d& g_W) const;

  // Minimal rotation taking unit vector a onto unit vector b (Eigen::Quaterniond::FromTwoVectors).
  static Eigen::Matrix3d minimalRotation(const Eigen::Vector3d& a, const Eigen::Vector3d& b);
  // Full M = R_z(ψ) Exp([φ_x, φ_y, 0]) C(g) (§1). tilt = (φ_x, φ_y). Exp is expTilt() below.
  Eigen::Matrix3d M(double yaw, const Eigen::Vector2d& tilt, const Eigen::Vector3d& g_W) const;
  static Eigen::Matrix3d Rz(double yaw);
  // Exp([φ_x, φ_y, 0]) without the host's 1e-5 small-angle cut-off (so3Exp): Rodrigues with sin θ/θ and
  // 2 sin²(θ/2)/θ² (no cancellation), Taylor below 1e-4 rad. Used by M() and AugBlock::repivot so the tilt residual
  // φ (prior σ_DOV = 1.5e-4, starts at 0) has no dead zone in which M ignores it while its Jacobian does not, and so
  // that central differences through M() work at any step (U7). φ is an additive state, not a host rotation: the
  // host's ⊞ keeps so3Exp and nothing bit-exact depends on this. Appended by WP-b (INTERFACE_CHANGES).
  static Eigen::Matrix3d expTilt(const Eigen::Vector2d& tilt);
  // Left Jacobian J_l(φ̃) of SO(3) at φ̃ = [φ_x, φ_y, 0]: Exp(φ̃ + δφ̃) = Exp(J_l(φ̃) δφ̃) Exp(φ̃) + O(|δφ̃|²). The tilt is
  // additive, so ∂(M v)/∂δφ = Ξ(v) J_l(φ̃) E_xy exactly (AMENDMENTS A34; the re-pivot T and the J_a/J_w tilt column).
  // J_l = I + (1 − cos θ)/θ² [φ̃]× + (θ − sin θ)/θ³ [φ̃]×², the first coefficient as 2 sin²(θ/2)/θ² like expTilt,
  // both by their Taylor series below 1e-4 rad (same branch point as expTilt). Appended by WP-b (INTERFACE_CHANGES, P2b).
  static Eigen::Matrix3d tiltLeftJacobian(const Eigen::Vector2d& tilt);

 private:
  Eigen::Matrix3d C_ref_;
  bool has_ref_ = false;
};

}  // namespace livo_gnss
