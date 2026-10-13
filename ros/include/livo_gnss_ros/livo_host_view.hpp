// SPDX-License-Identifier: GPL-2.0-only
// livo_host_view — StatesGroup ↔ livo_gnss::CoreState member-by-member copies and ImuProcess bridge
// access (DESIGN §12.5 "适配层"). This is the only place that knows both the host layout (common_lib.h)
// and the core layout; static_assert(DIM_STATE == 19) pins the host dimension. Copy functions are unit
// tested (ros-side rostest, IMPL_PLAN 7). Owner: P3 (livo_host_view.cpp).
#pragma once

#include <vector>

#include "common_lib.h"
#include "IMU_Processing.h"
#include "livo_gnss/core_state.hpp"

namespace livo_gnss_ros {

static_assert(DIM_STATE == 19, "livo_gnss expects the upstream 19-dim ESIKF (common_lib.h:30)");

// StatesGroup → CoreState (rot_end, pos_end, vel_end, bias_g, bias_a, gravity, inv_expo_time, cov).
livo_gnss::CoreState toCore(const StatesGroup& s);
// CoreState → StatesGroup (all seven members and cov; nothing else touched).
void fromCore(const livo_gnss::CoreState& c, StatesGroup* s);
// Round-trip identity check used by the G-bypass gate (returns true when fromCore(toCore(s)) == s bitwise).
bool roundTripIsBitwiseIdentity(const StatesGroup& s);

// Bridge samples recorded by ImuProcess during UndistortPcl (IMPL_PLAN 7: bridge_, F_accum_, t0/t1
// getters). Returns false when the last cycle recorded no samples (VIO cycle / IMU init).
bool takeBridge(const ImuProcess& imu, std::vector<livo_gnss::BridgeSample>* samples, livo_gnss::HostMat* F_tot,
                double* t_prev, double* t_k);

livo_gnss::SlamMode toSlamMode(SLAM_MODE m);

}  // namespace livo_gnss_ros
