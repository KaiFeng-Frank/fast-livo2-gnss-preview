#pragma once

// Initial covariance of the host accelerometer bias and gravity after FAST-LIVO2's
// specific-force initialisation. Off unless the caller asks.
//
// The init writes g = -mean(f)/|mean(f)|·G and leaves b_a = 0, with both blocks at
// 1e-5 (common_lib.h). A constant turn-on bias is then indistinguishable from a
// tilt of g until the attitude moves. BMI088 (Livox Avia user manual v1.0;
// BST-BMI088-DS001-13 Table 4), not fit to any trajectory:
//   zero-g offset typical 20 mg, package alignment 0.5°.
// b_a ~ N(0, σ_b² I), σ_b = 0.020·9.81. The same bias, projected orthogonal to g,
// is the tangent part of δg, so Cov(δg, δb_a) = −P_⊥ σ_b². The 0.5° alignment is
// an extra tangent variance on g only (it is not a bias). The radial component of
// g stays at the upstream 1e-5, so |g| is not loosened. 9.81 matches G_m_s2.

#include <Eigen/Core>

inline bool applyBmi088TurnonPrior(Eigen::Matrix<double, 19, 19>* cov, const Eigen::Vector3d& g) {
  const double nrm = g.norm();
  if (!(nrm > 1.0) || cov == nullptr) return false;
  const Eigen::Vector3d u = g / nrm;
  constexpr double kG = 9.81;
  constexpr double kZeroG = 0.020;
  constexpr double kAlignRad = 0.5 * 3.14159265358979323846 / 180.0;
  constexpr double kRadial = 1.0e-5;
  const double sb2 = (kZeroG * kG) * (kZeroG * kG);
  const double st2 = sb2 + (kG * kAlignRad) * (kG * kAlignRad);
  const Eigen::Matrix3d uu = u * u.transpose();
  const Eigen::Matrix3d perp = Eigen::Matrix3d::Identity() - uu;
  const Eigen::Matrix3d cross = -perp * sb2;
  cov->block<3, 3>(13, 13) = Eigen::Matrix3d::Identity() * sb2;
  cov->block<3, 3>(16, 16) = perp * st2 + uu * kRadial;
  cov->block<3, 3>(16, 13) = cross;
  cov->block<3, 3>(13, 16) = cross.transpose();
  return true;
}
