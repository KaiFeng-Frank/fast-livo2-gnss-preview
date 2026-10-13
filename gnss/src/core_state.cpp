// SPDX-License-Identifier: BSD-3-Clause
// Manifold helpers with the host's closed forms (utils/so3_math.h Exp(v1,v2,v3)/Log, common_lib.h:167-207).
// Frozen with the skeleton. Not required to be bit-identical to the host, but the formulas and the
// small-angle branches are the same so that the joint update's ⊞ agrees with StatesGroup::operator+.
#include "livo_gnss/core_state.hpp"

#include <cmath>

namespace livo_gnss {

Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
  Eigen::Matrix3d K;
  K << 0.0, -v[2], v[1], v[2], 0.0, -v[0], -v[1], v[0], 0.0;
  return K;
}

Eigen::Matrix3d so3Exp(const Eigen::Vector3d& w) {
  // so3_math.h:44-58 Exp(v1,v2,v3): Rodrigues with the 1e-5 small-angle cut-off.
  const double norm = std::sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]);
  const Eigen::Matrix3d Eye3 = Eigen::Matrix3d::Identity();
  if (norm > 0.00001) {
    const Eigen::Matrix3d K = skew(w / norm);
    return Eye3 + std::sin(norm) * K + (1.0 - std::cos(norm)) * K * K;
  }
  return Eye3;
}

Eigen::Vector3d so3Log(const Eigen::Matrix3d& R) {
  // so3_math.h:61-66 Log(R).
  const double theta = (R.trace() > 3.0 - 1e-6) ? 0.0 : std::acos(0.5 * (R.trace() - 1));
  const Eigen::Vector3d K(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1));
  return (std::abs(theta) < 0.001) ? Eigen::Vector3d(0.5 * K) : Eigen::Vector3d(0.5 * theta / std::sin(theta) * K);
}

CoreState CoreState::boxplus(const HostVec& dx) const {
  CoreState a = *this;
  a.R = R * so3Exp(dx.segment<3>(HostIdx::rot));
  a.p = p + dx.segment<3>(HostIdx::pos);
  a.inv_expo = inv_expo + dx(HostIdx::expo);
  a.v = v + dx.segment<3>(HostIdx::vel);
  a.bg = bg + dx.segment<3>(HostIdx::bg);
  a.ba = ba + dx.segment<3>(HostIdx::ba);
  a.g = g + dx.segment<3>(HostIdx::grav);
  return a;
}

HostVec CoreState::boxminus(const CoreState& b) const {
  HostVec d;
  d.segment<3>(HostIdx::rot) = so3Log(Eigen::Matrix3d(b.R.transpose() * R));
  d.segment<3>(HostIdx::pos) = p - b.p;
  d(HostIdx::expo) = inv_expo - b.inv_expo;
  d.segment<3>(HostIdx::vel) = v - b.v;
  d.segment<3>(HostIdx::bg) = bg - b.bg;
  d.segment<3>(HostIdx::ba) = ba - b.ba;
  d.segment<3>(HostIdx::grav) = g - b.g;
  return d;
}

}  // namespace livo_gnss
