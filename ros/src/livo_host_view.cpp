// SPDX-License-Identifier: GPL-2.0-only
// livo_host_view — StatesGroup <-> livo_gnss::CoreState member-by-member copies and the ImuProcess bridge view
// (DESIGN §12.5 "适配层", IMPL_PLAN 7). The only TU that knows both layouts: the static_asserts below pin the host
// dimension, the member types and the error-state column order the core assumes (common_lib.h:30/:167-205,
// core_state.hpp HostIdx). Every copy is a plain assignment (no arithmetic), so fromCore(toCore(s)) reproduces `s`
// bit for bit; roundTripIsBitwiseIdentity() checks exactly that on raw bytes (-0.0 and NaN payloads included).
// Tested by ros/test/test_p3b_glue.cpp (HostView*, ImuHooks*). Owner: P3-B.
#include "livo_gnss_ros/livo_host_view.hpp"

#include <cstring>
#include <type_traits>

namespace livo_gnss_ros {

namespace {

static_assert(livo_gnss::HostIdx::dim == DIM_STATE, "CoreState::P must have the host's DIM_STATE");
static_assert(livo_gnss::HostIdx::rot == 0 && livo_gnss::HostIdx::pos == 3 && livo_gnss::HostIdx::expo == 6 && livo_gnss::HostIdx::vel == 7 &&
                  livo_gnss::HostIdx::bg == 10 && livo_gnss::HostIdx::ba == 13 && livo_gnss::HostIdx::grav == 16,
              "error-state columns must follow StatesGroup::operator+ (common_lib.h:167-180): θ p τ v b_g b_a g");
static_assert(std::is_same<decltype(StatesGroup::rot_end), Eigen::Matrix3d>::value, "rot_end must be Matrix3d");
static_assert(std::is_same<decltype(StatesGroup::pos_end), Eigen::Vector3d>::value, "pos_end must be Vector3d");
static_assert(std::is_same<decltype(StatesGroup::vel_end), Eigen::Vector3d>::value, "vel_end must be Vector3d");
static_assert(std::is_same<decltype(StatesGroup::bias_g), Eigen::Vector3d>::value, "bias_g must be Vector3d");
static_assert(std::is_same<decltype(StatesGroup::bias_a), Eigen::Vector3d>::value, "bias_a must be Vector3d");
static_assert(std::is_same<decltype(StatesGroup::gravity), Eigen::Vector3d>::value, "gravity must be Vector3d");
static_assert(std::is_same<decltype(StatesGroup::inv_expo_time), double>::value, "inv_expo_time must be double");
static_assert(std::is_same<decltype(StatesGroup::cov), livo_gnss::HostMat>::value, "cov must be the 19x19 HostMat");
static_assert(std::is_same<decltype(ImuProcess::BridgeSample::R), Eigen::Matrix3d>::value &&
                  std::is_same<decltype(ImuProcess::BridgeSample::a_world), Eigen::Vector3d>::value,
              "ImuProcess::BridgeSample layout");

template <class M>
bool sameBits(const M& a, const M& b) {
  return std::memcmp(a.data(), b.data(), sizeof(typename M::Scalar) * static_cast<size_t>(a.size())) == 0;
}

bool sameBits(double a, double b) { return std::memcmp(&a, &b, sizeof(double)) == 0; }

}  // namespace

livo_gnss::CoreState toCore(const StatesGroup& s) {
  livo_gnss::CoreState c;
  c.R = s.rot_end;
  c.p = s.pos_end;
  c.v = s.vel_end;
  c.bg = s.bias_g;
  c.ba = s.bias_a;
  c.g = s.gravity;
  c.inv_expo = s.inv_expo_time;
  c.P = s.cov;
  return c;
}

void fromCore(const livo_gnss::CoreState& c, StatesGroup* s) {
  s->rot_end = c.R;
  s->pos_end = c.p;
  s->vel_end = c.v;
  s->bias_g = c.bg;
  s->bias_a = c.ba;
  s->gravity = c.g;
  s->inv_expo_time = c.inv_expo;
  s->cov = c.P;
}

bool roundTripIsBitwiseIdentity(const StatesGroup& s) {
  StatesGroup t;   // default state: every member differs from almost any `s`, so a skipped member shows up
  fromCore(toCore(s), &t);
  return sameBits(t.rot_end, s.rot_end) && sameBits(t.pos_end, s.pos_end) && sameBits(t.vel_end, s.vel_end) &&
         sameBits(t.bias_g, s.bias_g) && sameBits(t.bias_a, s.bias_a) && sameBits(t.gravity, s.gravity) &&
         sameBits(t.inv_expo_time, s.inv_expo_time) && sameBits(t.cov, s.cov);
}

bool takeBridge(const ImuProcess& imu, std::vector<livo_gnss::BridgeSample>* samples, livo_gnss::HostMat* F_tot, double* t_prev,
                double* t_k) {
  if (!imu.gnss_hooks_enabled() || !imu.gnss_bridge_valid()) return false;   // hooks off, VIO cycle, IMU init
  const std::vector<ImuProcess::BridgeSample>& src = imu.gnss_bridge_samples();
  const double t0 = imu.gnss_bridge_t0(), t1 = imu.gnss_bridge_t1();
  // The first propagated cycle after IMU initialisation starts at last_prop_end_time = 0 (IMU_Processing.h:80): its
  // start sample is not a state at any real time, so that one cycle is not a bridge.
  if (src.empty() || !(t0 > 0.0) || !(t1 > t0)) return false;
  samples->clear();
  samples->reserve(src.size());
  for (const ImuProcess::BridgeSample& h : src) {
    livo_gnss::BridgeSample b;
    b.t = h.t;
    b.a_world = h.a_world;
    b.omega = h.omega;
    b.R = h.R;
    b.p = h.p;
    b.v = h.v;
    b.f_body = h.f_body;
    samples->push_back(b);
  }
  *F_tot = imu.gnss_bridge_F();
  *t_prev = t0;
  *t_k = t1;
  return true;
}

livo_gnss::SlamMode toSlamMode(SLAM_MODE m) {
  switch (m) {
    case LIVO: return livo_gnss::SlamMode::LIVO;
    case ONLY_LIO: return livo_gnss::SlamMode::ONLY_LIO;
    default: return livo_gnss::SlamMode::ONLY_LO;
  }
}

}  // namespace livo_gnss_ros
