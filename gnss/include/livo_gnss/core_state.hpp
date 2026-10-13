// SPDX-License-Identifier: BSD-3-Clause
// Host (FAST-LIVO2) state as seen by the GNSS core (DESIGN §2.1, §12.5).
// The host's 19-dim ESIKF is never modified; ros/livo_host_view copies StatesGroup <-> CoreState member
// by member (static_assert(DIM_STATE == 19) lives there). Perturbation conventions must match
// common_lib.h: right perturbation on R (R = R̂·Exp(δθ), :170/:184), ⊟ = Log(R_bᵀR_a) (:197-198), all
// other blocks additive.
#pragma once

#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "livo_gnss/export.hpp"

namespace livo_gnss {

// Column indices of the host error state [δθ | δp | δτ | δv | δb_g | δb_a | δg] (common_lib.h:30).
struct HostIdx {
  static constexpr int rot = 0;
  static constexpr int pos = 3;
  static constexpr int expo = 6;
  static constexpr int vel = 7;
  static constexpr int bg = 10;
  static constexpr int ba = 13;
  static constexpr int grav = 16;
  static constexpr int dim = 19;
};

using HostMat = Eigen::Matrix<double, HostIdx::dim, HostIdx::dim>;
using HostVec = Eigen::Matrix<double, HostIdx::dim, 1>;

struct LIVO_GNSS_API CoreState {
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();   // R_{W<-I} (rot_end)
  Eigen::Vector3d p = Eigen::Vector3d::Zero();       // p^W_I (pos_end)
  Eigen::Vector3d v = Eigen::Vector3d::Zero();       // v^W_I (vel_end)
  Eigen::Vector3d bg = Eigen::Vector3d::Zero();      // gyro bias (bias_g)
  Eigen::Vector3d ba = Eigen::Vector3d::Zero();      // accel bias (bias_a)
  Eigen::Vector3d g = Eigen::Vector3d::Zero();       // gravity in W (gravity)
  double inv_expo = 1.0;                             // inverse exposure time (inv_expo_time)
  HostMat P = HostMat::Identity();                   // cov

  // Manifold operations with the host's conventions. Used by the joint update (§4.8) and the
  // conditioning hook (§2.4 H2). Not required to be bit-identical to the host (GNSS-on runs are not
  // bit-compared to R), but the formulas are the same Rodrigues forms as utils/so3_math.h.
  CoreState boxplus(const HostVec& dx) const;
  HostVec boxminus(const CoreState& other) const;    // *this ⊟ other
};

// SO(3) exponential / logarithm, same closed forms as the host's Exp()/Log() (utils/so3_math.h).
LIVO_GNSS_API Eigen::Matrix3d so3Exp(const Eigen::Vector3d& w);
LIVO_GNSS_API Eigen::Vector3d so3Log(const Eigen::Matrix3d& R);
LIVO_GNSS_API Eigen::Matrix3d skew(const Eigen::Vector3d& v);

// One sample of the IMU bridge (DESIGN §4.1), recorded inside UndistortPcl's propagation loop.
// The first sample of a LIO cycle is the cycle start {prop_beg_time, R, p, v, angvel_last, acc_s_last};
// every later sample is the state *after* integrating one IMU interval (IMU_Processing.cpp:412-421).
struct LIVO_GNSS_API BridgeSample {
  double t = 0.0;                                    // estimator time [s]
  Eigen::Vector3d a_world = Eigen::Vector3d::Zero(); // kinematic acceleration in W: a = R·f + g (acc_imu, :415)
  Eigen::Vector3d omega = Eigen::Vector3d::Zero();   // bias-corrected angular rate in I (angvel_avr, :352)
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();   // R_{W<-I} at t
  Eigen::Vector3d p = Eigen::Vector3d::Zero();       // p^W at t
  Eigen::Vector3d v = Eigen::Vector3d::Zero();       // v^W at t
  // Bias-corrected specific force in I (acc_avr after :353 = f̃·G/‖mean_acc‖ − b̂_a); ā_b of the §4.2 δθ/δb_a
  // columns is its mean over (t_g, t_k]. Equals Rᵀ(a_world − g) with the g of the recording (M1).
  Eigen::Vector3d f_body = Eigen::Vector3d::Zero();
};

enum class UpdateSource : uint8_t { LIO = 0, VIO = 1 };
enum class SlamMode : uint8_t { LIVO = 0, ONLY_LIO = 1, ONLY_LO = 2 };  // ONLY_LO is rejected (§9)

}  // namespace livo_gnss
