// SPDX-License-Identifier: BSD-3-Clause
// Leveling (DESIGN §1 "配准" C(g), ε Jacobian; derivation D1 §2.2). Owner: WP-b. Tests: U7 (WP-c), U9 sanity.
#include "internal/leveling.hpp"

#include <cmath>

#include <Eigen/Geometry>

#include "livo_gnss/core_state.hpp"

namespace livo_gnss {

namespace {
// C(g)·ĝ = −e_z: gravity points down in the levelled frame (N is ENU).
const Eigen::Vector3d kDown(0.0, 0.0, -1.0);
}  // namespace

Leveling::Leveling() : C_ref_(Eigen::Matrix3d::Identity()) {}

void Leveling::setRef(const Eigen::Vector3d& g_W) {
  // First call (ACTIVATE): any levelling rotation is admissible, yaw is carried by ψ. Later calls
  // (onStepEnd) compose the minimal rotation onto the stored reference; its axis is horizontal because
  // C_ref·ĝ ≈ −e_z already, so the refresh never rotates about e_z and no yaw leaks into C(g) (D1 §2.2).
  if (has_ref_) {
    C_ref_ = C(g_W);
  } else {
    C_ref_ = minimalRotation(g_W.normalized(), kDown);
    has_ref_ = true;
  }
}

Eigen::Matrix3d Leveling::C(const Eigen::Vector3d& g_W) const {
  // C(g) = Δ(C_ref·ĝ → −e_z) · C_ref
  const Eigen::Vector3d u = C_ref_ * g_W.normalized();
  return minimalRotation(u, kDown) * C_ref_;
}

Eigen::Matrix3d Leveling::Gperp(const Eigen::Vector3d& g_W) const {
  // G_⊥ = C (I − ĝĝᵀ)/‖g‖: the change of ĝ per unit change of g, rotated into the levelled frame.
  const double n = g_W.norm();
  const Eigen::Vector3d gh = g_W / n;
  return C(g_W) * (Eigen::Matrix3d::Identity() - gh * gh.transpose()) / n;
}

Eigen::Matrix3d Leveling::epsJacobian(const Eigen::Vector3d& g_W) const {
  // C(g+δg) ≈ Exp(ε) C(g) with ε = [e_z]× C (I − ĝĝᵀ) δg/‖g‖ (D1 §2.2: (I+[ε]×)C(ĝ+δĝ) = −e_z and Cδĝ is
  // horizontal). [e_z]× has a zero third row, so ε_z ≡ 0 exactly (not only to rounding).
  return skew(Eigen::Vector3d::UnitZ()) * Gperp(g_W);
}

Eigen::Matrix3d Leveling::minimalRotation(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
  return Eigen::Quaterniond::FromTwoVectors(a, b).toRotationMatrix();
}

Eigen::Matrix3d Leveling::M(double yaw, const Eigen::Vector2d& tilt, const Eigen::Vector3d& g_W) const {
  // M = R_{N←W} = R_z(ψ) · Exp([φ_x, φ_y, 0]ᵀ) · C(g)   (DESIGN §1)
  return Rz(yaw) * expTilt(tilt) * C(g_W);
}

Eigen::Matrix3d Leveling::expTilt(const Eigen::Vector2d& tilt) {
  // R = I + A [w]× + B [w]×²,  A = sin θ/θ,  B = (1 − cos θ)/θ² = 2 sin²(θ/2)/θ²  (no cancellation in B).
  // Below 1e-4 rad the two-term Taylor forms are exact to double (dropped terms θ⁴/120, θ⁴/720 < 1e-18).
  const Eigen::Vector3d w(tilt.x(), tilt.y(), 0.0);
  const double th2 = w.squaredNorm();
  const double th = std::sqrt(th2);
  double A, B;
  if (th < 1e-4) {
    A = 1.0 - th2 / 6.0;
    B = 0.5 - th2 / 24.0;
  } else {
    const double sh = std::sin(0.5 * th) / th;
    A = std::sin(th) / th;
    B = 2.0 * sh * sh;
  }
  const Eigen::Matrix3d K = skew(w);
  return Eigen::Matrix3d::Identity() + A * K + B * K * K;
}

Eigen::Matrix3d Leveling::tiltLeftJacobian(const Eigen::Vector2d& tilt) {
  // J_l(w) = I + A [w]× + B [w]×²,  A = (1 − cos θ)/θ² = 2 sin²(θ/2)/θ²,  B = (θ − sin θ)/θ³  (Barfoot 2017, 7.36).
  // Below 1e-4 rad: A = ½ − θ²/24, B = 1/6 − θ²/120 (dropped θ⁴/720, θ⁴/5040 < 2e-19). Above, the cancellation in
  // θ − sin θ costs ≈ 6ε/θ² relative on B (≤ 7e-8 at the branch point), i.e. < 2e-16 absolute on B θ².
  const Eigen::Vector3d w(tilt.x(), tilt.y(), 0.0);
  const double th2 = w.squaredNorm();
  const double th = std::sqrt(th2);
  double A, B;
  if (th < 1e-4) {
    A = 0.5 - th2 / 24.0;
    B = 1.0 / 6.0 - th2 / 120.0;
  } else {
    const double sh = std::sin(0.5 * th) / th;
    A = 2.0 * sh * sh;
    B = (th - std::sin(th)) / (th2 * th);
  }
  const Eigen::Matrix3d K = skew(w);
  return Eigen::Matrix3d::Identity() + A * K + B * K * K;
}

Eigen::Matrix3d Leveling::Rz(double yaw) {
  const double c = std::cos(yaw), s = std::sin(yaw);
  Eigen::Matrix3d R;
  R << c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0;
  return R;
}

}  // namespace livo_gnss
