// SPDX-License-Identifier: GPL-2.0-only
// P3-B glue tests — rostest ros/test/p3b_glue.test (a roscore is needed by Params.* and I14.*; the rest runs without).
//   HostView.*  StatesGroup <-> CoreState copies are bit exact (DESIGN §12.5 "拷贝函数单测覆盖").
//   ImuHooks.*  the IMU_Processing.* hook lines: disabled = no trace, enabled = the host propagation is unchanged bit for
//               bit, the bridge holds exactly the propagated cycle, F_tot is the product of the step transitions (H1).
//   Params.*    the gnss: parameter table covers config.hpp exactly; config/m3dgr_avia.yaml and the A46 arm overlays
//               load to the intended Config; unknown keys / wrong types / missing nav_rinex are refused.
//   Sat2Freq.*  the A35-C8 one-time sat2freq cross-check.
//   Wgs84.*     geodetic <-> ECEF helpers of the N-frame outputs.
//   I14.*       TEST_PLAN I14: the same bag messages through rosbag::MessageInstance::write() and through a ShapeShifter
//               that crossed the ROS transport give the same bytes and definitions and decode to the same records.
// Fixture: the Grass02 development-set slice of gnss/test/data (A24; LIVO_GNSS_TEST_DATA_DIR). Owner: P3-B.
#include <gtest/gtest.h>

#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <topic_tools/shape_shifter.h>

#include "IMU_Processing.h"
#include "livo_gnss/signal_table.hpp"
#include "livo_gnss_io/gnss_comm_mapping.hpp"
#include "livo_gnss_io/rinex_source.hpp"
#include "livo_gnss_ros/gnss_ros_bridge.hpp"
#include "livo_gnss_ros/livo_host_view.hpp"
#include "../src/gnss_ros_detail.hpp"
#include "../../gnss_io/test/fixture_discovery.hpp"

namespace {

using livo_gnss_ros::detail::configFromParamTree;

// ------------------------------------------------------------------------------------------------------------------
// helpers
// ------------------------------------------------------------------------------------------------------------------
template <class M>
bool sameBits(const M& a, const M& b) {
  return a.size() == b.size() && std::memcmp(a.data(), b.data(), sizeof(typename M::Scalar) * static_cast<size_t>(a.size())) == 0;
}
bool sameBits(double a, double b) { return std::memcmp(&a, &b, sizeof(double)) == 0; }

bool sameState(const StatesGroup& a, const StatesGroup& b) {
  return sameBits(a.rot_end, b.rot_end) && sameBits(a.pos_end, b.pos_end) && sameBits(a.vel_end, b.vel_end) && sameBits(a.bias_g, b.bias_g) &&
         sameBits(a.bias_a, b.bias_a) && sameBits(a.gravity, b.gravity) && sameBits(a.inv_expo_time, b.inv_expo_time) && sameBits(a.cov, b.cov);
}

std::string dataDir() {
#ifdef LIVO_GNSS_TEST_DATA_DIR
  return LIVO_GNSS_TEST_DATA_DIR;
#else
  const char* e = std::getenv("LIVO_GNSS_TEST_DATA_DIR");
  return e ? std::string(e) : std::string();
#endif
}

// The development-set fixture with every gnss_comm topic (Grass02; A24/A26).
std::string grassFixture() {
  for (const std::string& f : livo_gnss_io::testsupport::realFixtures(dataDir()))
    if (livo_gnss_io::testsupport::baseName(f).compare(0, 8, "Grass02_") == 0) return f;
  return std::string();
}

std::string readFile(const std::string& p) {
  std::ifstream f(p, std::ios::binary);
  std::stringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

std::string firstLine(const std::string& s) {
  const size_t n = s.find('\n');
  return n == std::string::npos ? s : s.substr(0, n);
}

struct FixtureTopic {
  std::string name, topic, type, md5, msgdef;
  livo_gnss::Source src;
  std::vector<std::pair<uint64_t, std::vector<uint8_t>>> msgs;   // (t_record_ns, serialised body)
};

// bytes/<topic>.bin = [uint64 t_record_ns][uint32 len][len bytes]... little endian (A24).
std::vector<FixtureTopic> loadFixture(const std::string& dir) {
  const std::vector<std::pair<std::string, livo_gnss::Source>> topics = {
      {"range_meas", livo_gnss::Source::GnssCommMeas},      {"ephem", livo_gnss::Source::GnssCommEphem},
      {"glo_ephem", livo_gnss::Source::GnssCommGloEphem},   {"iono_params", livo_gnss::Source::GnssCommIono},
      {"receiver_pvt", livo_gnss::Source::GnssCommPvt},     {"time_pulse_info", livo_gnss::Source::GnssCommTimePulse}};
  std::vector<FixtureTopic> out;
  for (const auto& t : topics) {
    if (!livo_gnss_io::testsupport::isFile(dir + "/msgdef/" + t.first + ".msgdef")) continue;
    FixtureTopic ft;
    ft.name = t.first;
    ft.topic = "/ublox_driver/" + t.first;
    ft.src = t.second;
    ft.type = firstLine(readFile(dir + "/msgdef/" + t.first + ".type"));
    ft.md5 = firstLine(readFile(dir + "/msgdef/" + t.first + ".md5"));
    ft.msgdef = readFile(dir + "/msgdef/" + t.first + ".msgdef");
    const std::string b = readFile(dir + "/bytes/" + t.first + ".bin");
    size_t i = 0;
    while (i + 12 <= b.size()) {
      uint64_t ns = 0;
      uint32_t len = 0;
      std::memcpy(&ns, b.data() + i, 8);
      std::memcpy(&len, b.data() + i + 8, 4);
      i += 12;
      if (i + len > b.size()) break;
      ft.msgs.emplace_back(ns, std::vector<uint8_t>(b.begin() + static_cast<long>(i), b.begin() + static_cast<long>(i + len)));
      i += len;
    }
    out.push_back(ft);
  }
  return out;
}

std::string g17(double v) {
  char b[40];
  std::snprintf(b, sizeof b, "%.17g", v);
  return b;
}

// Every field of every decoded item, one line each (%.17g: exact), in arrival order per channel.
class RecordingSink final : public livo_gnss::MeasurementSink {
 public:
  std::map<std::string, std::vector<std::string>> lines;
  void pushObservation(livo_gnss::ObsEpoch&& e) override {
    std::string s = "obs " + std::to_string(e.t_rx.sec) + " " + g17(e.t_rx.frac) + " " + g17(e.t_record) + " " +
                    std::to_string(static_cast<int>(e.source)) + " " + std::to_string(e.seq);
    for (const auto& so : e.sats) {
      s += " | " + std::to_string(so.sat.rtk_sat) + " " + std::to_string(static_cast<int>(so.sat.sys)) + " " + std::to_string(so.sat.prn) + " " +
           std::to_string(so.t_rx.sec) + " " + g17(so.t_rx.frac);
      for (const auto& sg : so.sig)
        s += " [" + g17(sg.freq_hz) + " " + std::to_string(static_cast<int>(sg.code)) + " " + g17(sg.psr_m) + " " + g17(sg.cp_cyc) + " " +
             g17(sg.dopp_hz) + " " + g17(sg.cn0_dbhz) + " " + std::to_string(sg.lli) + " " + std::to_string(sg.status) + " " +
             g17(sg.psr_std_raw) + " " + g17(sg.cp_std_raw) + " " + g17(sg.dopp_std_raw) + " " + std::to_string(static_cast<int>(sg.psr_std_unit)) +
             " " + std::to_string(static_cast<int>(sg.cp_std_unit)) + " " + std::to_string(static_cast<int>(sg.dopp_std_unit)) + " " +
             g17(sg.psr_std_m) + " " + g17(sg.cp_std_m) + " " + g17(sg.dopp_std_hz) + "]";
    }
    lines["obs"].push_back(s);
  }
  void pushEphemeris(const livo_gnss::KeplerEph& k) override {
    std::string s = std::to_string(k.sat.rtk_sat) + " " + std::to_string(k.iode) + " " + std::to_string(k.iodc) + " " + std::to_string(k.sva) +
                    " " + g17(k.ura_raw) + " " + std::to_string(k.svh) + " " + std::to_string(k.week) + " " + std::to_string(k.code) + " " +
                    std::to_string(k.flag) + " " + std::to_string(static_cast<int>(k.source)) + " " +
                    std::to_string(k.toe.sec) + " " + g17(k.toe.frac) + " " + std::to_string(k.toc.sec) + " " + g17(k.toc.frac) + " " +
                    std::to_string(k.ttr.sec) + " " + g17(k.ttr.frac);
    for (double v : {k.A, k.e, k.i0, k.OMG0, k.omg, k.M0, k.deln, k.OMGd, k.idot, k.crc, k.crs, k.cuc, k.cus, k.cic, k.cis, k.toes, k.fit, k.f0,
                     k.f1, k.f2, k.Adot, k.ndot, k.t_record})
      s += " " + g17(v);
    for (double v : k.tgd) s += " " + g17(v);
    lines["eph"].push_back(s);
  }
  void pushGloEphemeris(const livo_gnss::GloEph& g) override {
    std::string s = std::to_string(g.sat.rtk_sat) + " " + std::to_string(g.iode) + " " + std::to_string(g.frq) + " " + std::to_string(g.svh) +
                    " " + std::to_string(g.sva) + " " + std::to_string(g.age) + " " + std::to_string(g.toe.sec) + " " + g17(g.toe.frac) + " " +
                    std::to_string(g.tof.sec) + " " + g17(g.tof.frac) + " " + std::to_string(g.flags) + " " + std::to_string(static_cast<int>(g.source));
    for (int i = 0; i < 3; ++i) s += " " + g17(g.pos[i]) + " " + g17(g.vel[i]) + " " + g17(g.acc[i]);
    for (double v : {g.taun, g.gamn, g.dtaun, g.ura_raw, g.t_record}) s += " " + g17(v);
    lines["glo"].push_back(s);
  }
  void pushIono(const livo_gnss::Klobuchar& k) override {
    std::string s = std::to_string(k.t_valid.sec) + " " + g17(k.t_valid.frac) + " " + g17(k.t_record) + " " + std::to_string(static_cast<int>(k.source));
    for (double v : k.alpha) s += " " + g17(v);
    for (double v : k.beta) s += " " + g17(v);
    lines["iono"].push_back(s);
  }
  void pushPvt(const livo_gnss::ReceiverPvt& p) override {
    std::string s = std::to_string(p.t.sec) + " " + g17(p.t.frac) + " " + std::to_string(p.fix_type) + " " + std::to_string(p.valid_fix) + " " +
                    std::to_string(p.diff_soln) + " " + std::to_string(p.carr_soln) + " " + std::to_string(p.num_sv) + " " +
                    std::to_string(static_cast<int>(p.source));
    for (double v : {p.lat_deg, p.lon_deg, p.h_ell_m, p.h_msl_m, p.h_acc_m, p.v_acc_m, p.p_dop, p.vel_ned.x(), p.vel_ned.y(), p.vel_ned.z(),
                     p.vel_acc_mps, p.t_record})
      s += " " + g17(v);
    lines["pvt"].push_back(s);
  }
};

// The glue's decoding stage (CheckedDecoder around the gnss_comm decoder, as configure() builds it) fed with the
// (datatype, md5, definition, bytes) of one message; registration on the first message of each channel.
struct GlueDecodePath {
  livo_gnss_ros::detail::Sat2FreqCheck check;
  livo_gnss_io::GnssCommRos1Decoder comm{&livo_gnss::satNoOf};
  livo_gnss_ros::detail::CheckedDecoder dec{&comm, &check};
  RecordingSink sink;
  std::map<livo_gnss::Source, bool> registered;
  int errors = 0;
  void feed(livo_gnss::Source src, const std::string& type, const std::string& md5, const std::string& def, const std::vector<uint8_t>& b,
            double t_record) {
    std::string why;
    if (!registered[src]) {
      if (!dec.registerMessageDefinition(src, type, md5, def, &why)) ++errors;
      registered[src] = true;
    }
    if (!dec.decode(src, b.data(), b.size(), t_record, sink, &why)) ++errors;
  }
};

// ------------------------------------------------------------------------------------------------------------------
// HostView
// ------------------------------------------------------------------------------------------------------------------
StatesGroup randomState(std::mt19937_64& rng) {
  std::uniform_real_distribution<double> u(-10.0, 10.0);
  StatesGroup s;
  const Eigen::Vector3d w(u(rng), u(rng), u(rng));
  s.rot_end = Eigen::AngleAxisd(w.norm() * 0.1, w.normalized()).toRotationMatrix();
  for (int i = 0; i < 3; ++i) {
    s.pos_end[i] = u(rng);
    s.vel_end[i] = u(rng);
    s.bias_g[i] = u(rng) * 1e-3;
    s.bias_a[i] = u(rng) * 1e-2;
    s.gravity[i] = u(rng);
  }
  s.inv_expo_time = u(rng);
  for (int r = 0; r < DIM_STATE; ++r)
    for (int c = 0; c < DIM_STATE; ++c) s.cov(r, c) = u(rng);
  return s;
}

TEST(HostView, CopiesAreBitExact) {
  std::mt19937_64 rng(20260925);
  for (int k = 0; k < 200; ++k) {
    StatesGroup s = randomState(rng);
    if (k == 0) {   // values an arithmetic copy would lose: -0.0, a NaN payload, a denormal, infinities
      s.pos_end[0] = -0.0;
      s.vel_end[1] = std::numeric_limits<double>::denorm_min();
      s.cov(3, 4) = std::nan("0x5a5a");
      s.cov(18, 18) = -std::numeric_limits<double>::infinity();
      s.inv_expo_time = -0.0;
    }
    const livo_gnss::CoreState c = livo_gnss_ros::toCore(s);
    EXPECT_TRUE(sameBits(c.R, s.rot_end) && sameBits(c.p, s.pos_end) && sameBits(c.v, s.vel_end) && sameBits(c.bg, s.bias_g) &&
                sameBits(c.ba, s.bias_a) && sameBits(c.g, s.gravity) && sameBits(c.inv_expo, s.inv_expo_time) && sameBits(c.P, s.cov))
        << "toCore member mapping, case " << k;
    StatesGroup t;
    livo_gnss_ros::fromCore(c, &t);
    EXPECT_TRUE(sameState(t, s)) << "fromCore(toCore(s)) != s, case " << k;
    EXPECT_TRUE(livo_gnss_ros::roundTripIsBitwiseIdentity(s)) << "case " << k;
  }
}

TEST(HostView, SlamModeMapping) {
  EXPECT_EQ(livo_gnss_ros::toSlamMode(LIVO), livo_gnss::SlamMode::LIVO);
  EXPECT_EQ(livo_gnss_ros::toSlamMode(ONLY_LIO), livo_gnss::SlamMode::ONLY_LIO);
  EXPECT_EQ(livo_gnss_ros::toSlamMode(ONLY_LO), livo_gnss::SlamMode::ONLY_LO);
}

// ------------------------------------------------------------------------------------------------------------------
// ImuHooks: a synthetic LIVO-style sequence (200 Hz IMU, 10 Hz LIO cycles each followed by its zero-IMU VIO cycle)
// ------------------------------------------------------------------------------------------------------------------
struct SynthImu {
  double t0 = 100.0, rate = 200.0;
  sensor_msgs::ImuConstPtr at(int i) const {
    sensor_msgs::ImuPtr m(new sensor_msgs::Imu());
    const double t = t0 + i / rate;
    m->header.stamp = ros::Time().fromSec(t);
    m->angular_velocity.x = 0.3 * std::sin(1.3 * t);
    m->angular_velocity.y = 0.2 * std::cos(0.7 * t);
    m->angular_velocity.z = 0.5 * std::sin(0.4 * t) + 0.05;
    m->linear_acceleration.x = 0.8 * std::sin(0.9 * t);
    m->linear_acceleration.y = 0.5 * std::cos(1.1 * t);
    m->linear_acceleration.z = 9.7 + 0.3 * std::sin(2.0 * t);
    return m;
  }
};

struct HookRig {
  ImuProcessPtr imu{new ImuProcess()};
  StatesGroup state;
  LidarMeasureGroup lm;
  PointCloudXYZI::Ptr out{new PointCloudXYZI()};
  explicit HookRig(bool hooks, bool zero_noise) {
    imu->set_extrinsic(V3D(0.04165, 0.02326, -0.0284), M3D::Identity());
    imu->set_imu_init_frame_num(5);
    if (zero_noise) {
      imu->set_gyr_cov_scale(V3D::Zero());
      imu->set_acc_cov_scale(V3D::Zero());
      imu->set_gyr_bias_cov(V3D::Zero());
      imu->set_acc_bias_cov(V3D::Zero());
      imu->set_inv_expo_cov(0.0);
    } else {
      imu->set_gyr_cov_scale(V3D(0.3, 0.3, 0.3));
      imu->set_acc_cov_scale(V3D(0.5, 0.5, 0.5));
      imu->set_gyr_bias_cov(V3D(1e-4, 1e-4, 1e-4));
      imu->set_acc_bias_cov(V3D(1e-4, 1e-4, 1e-4));
      imu->set_inv_expo_cov(0.1);
    }
    imu->enable_gnss_hooks(hooks);
  }
  // One cycle: LIO with IMU samples first..last (all in (t_prev, t_k], as sync_packages cuts them) and a point cloud
  // over the cycle, or VIO with none.
  void cycle(const SynthImu& s, int first, int last, double t_prev, double t_k, bool lio) {
    MeasureGroup m;
    if (lio) {
      for (int i = first; i <= last; ++i) m.imu.push_back(s.at(i));
      m.lio_time = t_k;
      lm.lio_vio_flg = LIO;
      lm.pcl_proc_cur.reset(new PointCloudXYZI());
      for (int k = 0; k < 64; ++k) {
        PointType p;
        p.x = static_cast<float>(5.0 + std::sin(0.3 * k));
        p.y = static_cast<float>(2.0 * std::cos(0.2 * k));
        p.z = static_cast<float>(0.1 * k - 1.0);
        p.intensity = 1.0f;
        p.curvature = static_cast<float>((t_k - t_prev) * 1000.0 * (k + 0.5) / 64.0);   // offset [ms] from the cycle start
        lm.pcl_proc_cur->push_back(p);
      }
    } else {
      m.vio_time = t_k;
      m.lio_time = t_k;
      lm.lio_vio_flg = VIO;
    }
    lm.measures.clear();
    lm.measures.push_back(m);
    imu->Process2(lm, state, out);
  }
};

// Runs `n` LIVO cycle pairs on the rigs; calls `each(cycle_index, t_prev, t_k, prior)` after every LIO cycle.
template <class F>
void runSequence(std::vector<HookRig*> rigs, int n, F each) {
  SynthImu s;
  const int per = 20;   // 0.1 s cycles; cycle c holds IMU samples c·per .. (c+1)·per − 1 and ends 2.5 ms after the last
  auto t_end = [&s, per](int c) { return s.t0 + (c + 1) * per / s.rate - 0.0025; };
  // IMU initialisation frame (Process2 returns before UndistortPcl).
  for (HookRig* r : rigs) r->cycle(s, 0, per - 1, 0.0, t_end(0), true);
  double t_prev = t_end(0);
  for (int c = 1; c <= n; ++c) {
    const double t_k = t_end(c);
    std::vector<StatesGroup> priors;
    for (HookRig* r : rigs) priors.push_back(r->state);
    for (HookRig* r : rigs) r->cycle(s, c * per, (c + 1) * per - 1, t_prev, t_k, true);
    each(c, t_prev, t_k, priors);
    for (HookRig* r : rigs) r->cycle(s, 0, -1, t_k, t_k, false);   // the zero-IMU VIO cycle of LIVO
    t_prev = t_k;
  }
}

TEST(ImuHooks, EnabledHooksLeaveThePropagationBitIdentical) {
  HookRig off(false, false), on(true, false);
  int compared = 0;
  runSequence({&off, &on}, 40, [&](int c, double, double, const std::vector<StatesGroup>&) {
    ASSERT_TRUE(sameState(off.state, on.state)) << "host state differs after LIO cycle " << c;
    ASSERT_EQ(off.out->size(), on.out->size());
    for (size_t i = 0; i < off.out->size(); ++i)
      ASSERT_TRUE(std::memcmp(&off.out->points[i], &on.out->points[i], sizeof(PointType)) == 0) << "undistorted point " << i << ", cycle " << c;
    ++compared;
  });
  EXPECT_EQ(compared, 40);
  EXPECT_TRUE(sameState(off.state, on.state)) << "after the last VIO cycle";
  EXPECT_FALSE(off.imu->gnss_hooks_enabled());
  EXPECT_TRUE(off.imu->gnss_bridge_samples().empty()) << "disabled hooks recorded samples";
  EXPECT_FALSE(off.imu->gnss_bridge_valid());
  EXPECT_TRUE(sameBits(off.imu->gnss_bridge_F(), livo_gnss::HostMat(livo_gnss::HostMat::Identity()))) << "disabled hooks touched F_accum";
  EXPECT_EQ(on.imu->gnss_bridge_orphan_steps(), 0) << "a VIO cycle propagated the state";
}

TEST(ImuHooks, BridgeHoldsThePropagatedCycle) {
  HookRig on(true, false);
  double last_t1 = 0.0;
  int checked = 0;
  runSequence({&on}, 30, [&](int c, double t_prev, double t_k, const std::vector<StatesGroup>& prior) {
    const ImuProcess& imu = *on.imu;
    ASSERT_TRUE(imu.gnss_bridge_valid()) << "cycle " << c;
    const auto& b = imu.gnss_bridge_samples();
    ASSERT_GE(b.size(), 2u);
    EXPECT_EQ(imu.gnss_bridge_t1(), t_k);
    EXPECT_EQ(on.lm.last_lio_update_time, t_k);
    if (c == 1) {
      EXPECT_EQ(imu.gnss_bridge_t0(), 0.0) << "first propagated cycle starts at last_prop_end_time = 0";
    } else {
      EXPECT_EQ(imu.gnss_bridge_t0(), last_t1) << "cycle start = previous cycle end (contiguous chain)";
      EXPECT_EQ(imu.gnss_bridge_t0(), t_prev);
    }
    last_t1 = imu.gnss_bridge_t1();
    // start sample = the state the cycle started from
    EXPECT_EQ(b.front().t, imu.gnss_bridge_t0());
    EXPECT_TRUE(sameBits(b.front().R, prior[0].rot_end) && sameBits(b.front().p, prior[0].pos_end) && sameBits(b.front().v, prior[0].vel_end));
    EXPECT_NEAR((b.front().R * b.front().f_body + prior[0].gravity - b.front().a_world).norm(), 0.0, 1e-12);
    // last sample = the propagated state handed to the LIO update, bit for bit
    EXPECT_TRUE(sameBits(b.back().R, on.state.rot_end) && sameBits(b.back().p, on.state.pos_end) && sameBits(b.back().v, on.state.vel_end));
    EXPECT_NEAR(b.back().t, t_k, 1e-9);
    for (size_t i = 1; i < b.size(); ++i) {
      EXPECT_LE(b[i - 1].t, b[i].t) << "sample times, cycle " << c;
      EXPECT_NEAR((b[i].R * b[i].f_body + on.state.gravity - b[i].a_world).norm(), 0.0, 1e-12) << "a = R f + g, sample " << i;
    }
    EXPECT_TRUE(sameBits(imu.gnss_bridge_bg_snap(), prior[0].bias_g) && sameBits(imu.gnss_bridge_ba_snap(), prior[0].bias_a) &&
                sameBits(imu.gnss_bridge_g_snap(), prior[0].gravity));
    // livo_host_view::takeBridge: refuses the first cycle, copies every other one exactly
    std::vector<livo_gnss::BridgeSample> s;
    livo_gnss::HostMat F;
    double t0 = -1, t1 = -1;
    const bool ok = livo_gnss_ros::takeBridge(imu, &s, &F, &t0, &t1);
    if (c == 1) {
      EXPECT_FALSE(ok) << "the t0 = 0 cycle is not a bridge";
    } else {
      ASSERT_TRUE(ok);
      EXPECT_EQ(t0, imu.gnss_bridge_t0());
      EXPECT_EQ(t1, imu.gnss_bridge_t1());
      EXPECT_TRUE(sameBits(F, imu.gnss_bridge_F()));
      ASSERT_EQ(s.size(), b.size());
      for (size_t i = 0; i < b.size(); ++i)
        EXPECT_TRUE(s[i].t == b[i].t && sameBits(s[i].R, b[i].R) && sameBits(s[i].p, b[i].p) && sameBits(s[i].v, b[i].v) &&
                    sameBits(s[i].a_world, b[i].a_world) && sameBits(s[i].omega, b[i].omega) && sameBits(s[i].f_body, b[i].f_body))
            << "sample " << i;
      ++checked;
    }
  });
  EXPECT_EQ(checked, 29);
  // after the trailing VIO cycle the bridge is invalid and takeBridge refuses it
  EXPECT_FALSE(on.imu->gnss_bridge_valid());
  std::vector<livo_gnss::BridgeSample> s;
  livo_gnss::HostMat F;
  double t0, t1;
  EXPECT_FALSE(livo_gnss_ros::takeBridge(*on.imu, &s, &F, &t0, &t1));
}

// With every process-noise term zero the host covariance obeys P_k = F_n … F_1 P_{k-1} F_1ᵀ … F_nᵀ exactly, so the
// accumulated F_tot must reproduce it (to rounding): the H1 product order is right and no step is missing.
TEST(ImuHooks, AccumulatedFReproducesTheCovariancePropagation) {
  HookRig on(true, true);
  int checked = 0;
  runSequence({&on}, 25, [&](int c, double, double, const std::vector<StatesGroup>& prior) {
    const livo_gnss::HostMat& F = on.imu->gnss_bridge_F();
    const livo_gnss::HostMat pred = F * prior[0].cov * F.transpose();
    const double scale = std::max(1.0, on.state.cov.cwiseAbs().maxCoeff());
    EXPECT_LT((pred - on.state.cov).cwiseAbs().maxCoeff() / scale, 1e-12) << "cycle " << c;
    EXPECT_FALSE(sameBits(F, livo_gnss::HostMat(livo_gnss::HostMat::Identity()))) << "F_tot = I after a propagated cycle";
    ++checked;
  });
  EXPECT_EQ(checked, 25);
}

// ------------------------------------------------------------------------------------------------------------------
// Params
// ------------------------------------------------------------------------------------------------------------------
TEST(Params, TableCoversConfigHppExactly) {
  EXPECT_TRUE(livo_gnss_ros::detail::loaderCoverageProblems().empty());
  size_t dump_lines = 0;
  std::istringstream in(livo_gnss::canonicalDump(livo_gnss::Config()));
  std::string l;
  while (std::getline(in, l)) dump_lines += (l.find('=') != std::string::npos);
  // The livgo/* keys are not fields of Config (they go into loadedLivgoConfig(), A72+); loaderCoverageProblems() leaves them out and
  // so does the count. (Before A75 this line compared all keys against the dump and had been failing since the first livgo key.)
  size_t table_keys = 0;
  for (const std::string& k : livo_gnss_ros::detail::configKeyPaths()) table_keys += (k.compare(0, 6, "livgo/") != 0);
  EXPECT_EQ(table_keys, dump_lines);
}

// Leaf-wise overlay, as roslaunch <rosparam load> and `rosparam load` apply a file on top of an existing tree.
void overlay(XmlRpc::XmlRpcValue& base, XmlRpc::XmlRpcValue& top) {
  for (auto it = top.begin(); it != top.end(); ++it) {
    if (it->second.getType() == XmlRpc::XmlRpcValue::TypeStruct && base.hasMember(it->first) &&
        base[it->first].getType() == XmlRpc::XmlRpcValue::TypeStruct)
      overlay(base[it->first], it->second);
    else
      base[it->first] = it->second;
  }
}

TEST(Params, M3dgrConfigLoadsAsIntended) {
  XmlRpc::XmlRpcValue tree;
  ASSERT_TRUE(ros::param::get("/gnss", tree)) << "p3b_glue.test loads config/m3dgr_avia.yaml";
  livo_gnss::Config c;
  std::vector<std::string> errors;
  EXPECT_TRUE(configFromParamTree(tree, &c, &errors)) << (errors.empty() ? "" : errors.front());
  EXPECT_TRUE(livo_gnss::validateConfig(c).empty());
  EXPECT_FALSE(c.enable) << "the base configuration is the U arm (zero channel)";
  EXPECT_EQ(c.ephem.ura_semantics, livo_gnss::Config::Ephem::UraSemantics::Metres);   // A45
  EXPECT_EQ(c.masks.cn0_psr, 20.0);                                                  // A58 (P4 calibration; was A44)
  EXPECT_EQ(c.masks.cn0_dopp, 12.0);
  EXPECT_EQ(c.masks.cn0_cp, 25.0);
  EXPECT_EQ(c.masks.max_psr_std_m, 41.0);                                            // A58 (guards; was A47)
  EXPECT_EQ(c.masks.max_dopp_std_hz, 16.4);
  EXPECT_EQ(c.noise.psr.k_std, 0.0);                                                 // A58 noise / clock / frame
  EXPECT_EQ(c.process.frame.q_yaw_per_rad, 0.0);
  EXPECT_EQ(c.imu_bridge.acc_n, 0.0208);                                             // A15 Avia
  EXPECT_EQ(c.imu_bridge.gyr_n, 0.00322);
  EXPECT_EQ(c.frames.lever_arm_frame, livo_gnss::Config::Frames::LeverArmFrame::Lidar);
  EXPECT_EQ(c.frames.lever_arm_xyz[0], -0.358);
  EXPECT_EQ(c.frames.lever_arm_xyz[1], 0.086);
  EXPECT_EQ(c.frames.lever_arm_xyz[2], 0.429);
  EXPECT_EQ(c.masks.eph_age_s.at('J'), 7200.0);
  EXPECT_EQ(c.noise.psr.sys_factor.at('R'), 1.5);
  EXPECT_EQ(c.time.base, livo_gnss::Config::Time::Base::Host);
  EXPECT_EQ(c.time.leap, "auto");
  EXPECT_TRUE(c.nav_rinex.empty()) << "nav_rinex is injected per sequence";
  EXPECT_EQ(c.topics.meas, "/ublox_driver/range_meas");
  // Apart from the M3DGR-specific values (above, and the A58 calibration below), every field equals the config.hpp default.
  livo_gnss::Config d;
  d.masks.cn0_psr = 20.0;
  d.masks.cn0_dopp = 12.0;
  d.masks.cn0_cp = 25.0;
  d.masks.max_psr_std_m = 41.0;
  d.masks.max_dopp_std_hz = 16.4;
  d.ephem.ura_semantics = livo_gnss::Config::Ephem::UraSemantics::Metres;
  d.noise.psr.sigma0_m = 0.773;                    // A58: TEST_PLAN §3 closed-form calibration on the development set
  d.noise.psr.k_std = 0.0;
  d.noise.psr.tau_s = 33.1;
  d.noise.dopp.sigma0_mps = 0.127;
  d.noise.dopp.tau_s = 0.047;
  d.noise.pvt.tau_pos_s = 21.6;
  d.noise.pvt.tau_vel_s = 0.52;
  d.noise.pvt.scale = 0.35;
  d.noise.scale.psr = 0.0178;
  d.noise.scale.dopp = 0.2118;
  d.noise.scale.tdcp = 0.882;
  d.process.clock.S_b = 1.0e-3;
  d.process.clock.S_d = 1.43e-2;
  d.process.frame.q_tilt_per_m = 4.8378025259125282e-08;   // A70 frozen value, in the repository yaml since 9f15078
  d.process.frame.q_yaw_per_m = 2.78e-8;
  d.process.frame.q_yaw_per_rad = 0.0;
  d.process.frame.q_th_per_m = 4.58e-5;
  d.process.frame.q_tv_per_m = 0.00055234912699583371;
  EXPECT_EQ(livo_gnss::canonicalDump(c), livo_gnss::canonicalDump(d));
}

TEST(Params, ArmOverlaysGiveTheA46Arms) {
  struct Arm {
    const char* ns;
    bool enable;
    livo_gnss::Config::Mode mode;
    bool psr, dopp, tdcp, pvt_vel;
  };
  using M = livo_gnss::Config::Mode;
  const Arm arms[] = {{"U", false, M::Tight, true, true, true, true},      {"L_PVT", true, M::LoosePvt, false, false, false, true},
                      {"T", true, M::Tight, true, true, true, true},      {"T_pr", true, M::Tight, false, true, true, true},
                      {"T_dop", true, M::Tight, true, false, true, true}, {"T_tdcp", true, M::Tight, true, true, false, true}};
  for (const Arm& a : arms) {
    XmlRpc::XmlRpcValue base, top;
    ASSERT_TRUE(ros::param::get("/gnss", base));
    ASSERT_TRUE(ros::param::get(std::string("/p3b_test/arms/") + a.ns + "/gnss", top)) << a.ns;
    overlay(base, top);
    livo_gnss::Config c;
    std::vector<std::string> errors;
    EXPECT_TRUE(configFromParamTree(base, &c, &errors)) << a.ns;
    EXPECT_TRUE(livo_gnss::validateConfig(c).empty()) << a.ns;
    EXPECT_EQ(c.enable, a.enable) << a.ns;
    EXPECT_TRUE(c.update_enable) << a.ns;
    EXPECT_EQ(c.mode, a.mode) << a.ns;
    EXPECT_EQ(c.update_slot, livo_gnss::Config::UpdateSlot::PreLio) << a.ns;
    EXPECT_EQ(c.use.psr, a.psr) << a.ns;
    EXPECT_EQ(c.use.dopp, a.dopp) << a.ns;
    EXPECT_EQ(c.use.tdcp, a.tdcp) << a.ns;
    EXPECT_EQ(c.use.pvt_vel, a.pvt_vel) << a.ns;
    EXPECT_EQ(c.masks.cn0_psr, 20.0) << a.ns << ": the overlay must not reset the shared configuration";
  }
}

TEST(Params, UnknownKeysWrongTypesAndBadNamesAreRefused) {
  auto load = [](const std::function<void(XmlRpc::XmlRpcValue&)>& fill, livo_gnss::Config* c) {
    XmlRpc::XmlRpcValue t;
    t["enable"] = false;   // make it a struct
    fill(t);
    std::vector<std::string> e;
    configFromParamTree(t, c, &e);
    return e;
  };
  livo_gnss::Config c;
  auto e = load([](XmlRpc::XmlRpcValue& t) { t["enabel"] = true; }, &c);
  ASSERT_EQ(e.size(), 1u);
  EXPECT_NE(e[0].find("gnss/enabel: unknown key"), std::string::npos) << e[0];
  e = load([](XmlRpc::XmlRpcValue& t) { t["masks"]["cn0_psr"] = std::string("15"); }, &c);
  ASSERT_EQ(e.size(), 1u);
  EXPECT_NE(e[0].find("gnss/masks/cn0_psr: expected a number"), std::string::npos) << e[0];
  e = load([](XmlRpc::XmlRpcValue& t) { t["time"]["bridge_ring_cycles"] = 4.0; }, &c);
  ASSERT_EQ(e.size(), 1u);
  EXPECT_NE(e[0].find("expected an integer"), std::string::npos) << e[0];
  e = load([](XmlRpc::XmlRpcValue& t) { t["mode"] = std::string("tite"); }, &c);
  ASSERT_EQ(e.size(), 1u);
  EXPECT_NE(e[0].find("one of tight|loose_pvt|livgo_probe|livgo_probe_norej"), std::string::npos) << e[0];
  e = load([](XmlRpc::XmlRpcValue& t) { t["masks"]["eph_age_s"]["X"] = 1.0; }, &c);
  EXPECT_EQ(e.size(), 1u);
  e = load([](XmlRpc::XmlRpcValue& t) { t["frames"]["lever_arm_xyz"][0] = 1.0; }, &c);
  EXPECT_EQ(e.size(), 1u) << "a 1-element lever arm";
  e = load([](XmlRpc::XmlRpcValue& t) { t["noise"]["psr"]["nope"] = 1.0; }, &c);
  ASSERT_EQ(e.size(), 1u);
  EXPECT_NE(e[0].find("gnss/noise/psr/nope: unknown key"), std::string::npos) << e[0];
  e = load([](XmlRpc::XmlRpcValue& t) { t["noise"] = 1.0; }, &c);
  ASSERT_EQ(e.size(), 1u);
  EXPECT_NE(e[0].find("expected a map"), std::string::npos) << e[0];
  // accepted forms: leap as an integer, a partial per-system map (merged), a single nav_rinex string
  livo_gnss::Config ok;
  e = load(
      [](XmlRpc::XmlRpcValue& t) {
        t["time"]["leap"] = 18;
        t["masks"]["eph_age_s"]["G"] = 100.0;
        t["nav_rinex"] = std::string("/tmp/x.rnx");
        t["time"]["offset_s"] = 1;   // an integer where a double is expected
      },
      &ok);
  EXPECT_TRUE(e.empty()) << (e.empty() ? "" : e.front());
  EXPECT_EQ(ok.time.leap, "18");
  EXPECT_EQ(ok.masks.eph_age_s.at('G'), 100.0);
  EXPECT_EQ(ok.masks.eph_age_s.at('J'), 7200.0) << "keys absent from a partial map keep their defaults";
  ASSERT_EQ(ok.nav_rinex.size(), 1u);
  EXPECT_EQ(ok.nav_rinex[0], "/tmp/x.rnx");
  EXPECT_EQ(ok.time.offset_s, 1.0);
}

// AMENDMENTS A75: the livgo/* keys of LIVGO-v2. Absent = the pre-A75 behaviour; wrong types and unknown spellings are refused; the
// keys never reach livo_gnss::Config (the T-arm layout), only loadedLivgoConfig().
TEST(Params, LivgoV2SwitchesParseAndDefaultToThePreA75Behaviour) {
  using livo_gnss::LivgoConfig;
  auto load = [](const std::function<void(XmlRpc::XmlRpcValue&)>& fill) {
    XmlRpc::XmlRpcValue t;
    t["enable"] = false;   // make it a struct
    fill(t);
    livo_gnss::Config c;
    std::vector<std::string> e;
    configFromParamTree(t, &c, &e);
    return e;
  };
  // absent
  auto e = load([](XmlRpc::XmlRpcValue& t) { t["livgo"]["enable"] = true; });
  EXPECT_TRUE(e.empty()) << (e.empty() ? "" : e.front());
  {
    const LivgoConfig& l = livo_gnss_ros::detail::loadedLivgoConfig();
    EXPECT_TRUE(l.enable);
    EXPECT_FALSE(l.frame_exact);
    EXPECT_EQ(l.lambda_source, LivgoConfig::LambdaSource::Min);
    EXPECT_TRUE(l.dtw_online);
    EXPECT_EQ(l.noise_mode, LivgoConfig::NoiseMode::Model);
    EXPECT_EQ(l.flat_sigma_tdcp_m, 0.001);
    EXPECT_EQ(l.flat_sigma_dopp_mps, 0.05);
    EXPECT_FALSE(l.tdcp_gain_log);
    EXPECT_TRUE(l.tdcp_gain_log_path.empty());
  }
  // LIVGO-v2-paper
  e = load([](XmlRpc::XmlRpcValue& t) {
    t["livgo"]["enable"] = true;
    t["livgo"]["paper_init"] = false;
    t["livgo"]["frame_exact"] = true;
    t["livgo"]["lambda_source"] = std::string("lio");
    t["livgo"]["dtw_online"] = false;
    t["livgo"]["lambda_th"] = 0.0;
    t["livgo"]["noise_mode"] = std::string("flat");
    t["livgo"]["flat_sigma_tdcp_m"] = 0.002;
    t["livgo"]["flat_sigma_dopp_mps"] = 1;   // an integer where a double is expected
    t["livgo"]["tdcp_gain_log"] = true;
  });
  EXPECT_TRUE(e.empty()) << (e.empty() ? "" : e.front());
  {
    const LivgoConfig& l = livo_gnss_ros::detail::loadedLivgoConfig();
    EXPECT_TRUE(l.frame_exact);
    EXPECT_EQ(l.lambda_source, LivgoConfig::LambdaSource::Lio);
    EXPECT_FALSE(l.dtw_online);
    EXPECT_EQ(l.noise_mode, LivgoConfig::NoiseMode::Flat);
    EXPECT_EQ(l.flat_sigma_tdcp_m, 0.002);
    EXPECT_EQ(l.flat_sigma_dopp_mps, 1.0);
    EXPECT_FALSE(l.paper_init);
    EXPECT_TRUE(l.tdcp_gain_log);
  }
  // each call starts from the defaults again (a key of the previous call does not linger)
  e = load([](XmlRpc::XmlRpcValue& t) { t["livgo"]["enable"] = true; });
  EXPECT_TRUE(e.empty());
  EXPECT_FALSE(livo_gnss_ros::detail::loadedLivgoConfig().frame_exact);
  EXPECT_EQ(livo_gnss_ros::detail::loadedLivgoConfig().noise_mode, LivgoConfig::NoiseMode::Model);
  EXPECT_FALSE(livo_gnss_ros::detail::loadedLivgoConfig().tdcp_gain_log);
  // refusals
  e = load([](XmlRpc::XmlRpcValue& t) { t["livgo"]["lambda_source"] = std::string("vio"); });
  ASSERT_EQ(e.size(), 1u);
  EXPECT_NE(e[0].find("gnss/livgo/lambda_source: expected one of min|lio"), std::string::npos) << e[0];
  e = load([](XmlRpc::XmlRpcValue& t) { t["livgo"]["noise_mode"] = std::string("paper"); });
  ASSERT_EQ(e.size(), 1u);
  EXPECT_NE(e[0].find("gnss/livgo/noise_mode: expected one of model|flat"), std::string::npos) << e[0];
  e = load([](XmlRpc::XmlRpcValue& t) { t["livgo"]["frame_exact"] = 1; });
  ASSERT_EQ(e.size(), 1u);
  EXPECT_NE(e[0].find("gnss/livgo/frame_exact: expected true or false"), std::string::npos) << e[0];
  e = load([](XmlRpc::XmlRpcValue& t) { t["livgo"]["dtw_online"] = std::string("false"); });
  ASSERT_EQ(e.size(), 1u);
  EXPECT_NE(e[0].find("gnss/livgo/dtw_online: expected true or false"), std::string::npos) << e[0];
  e = load([](XmlRpc::XmlRpcValue& t) { t["livgo"]["flat_sigma_tdcp_m"] = std::string("1 mm"); });
  ASSERT_EQ(e.size(), 1u);
  EXPECT_NE(e[0].find("gnss/livgo/flat_sigma_tdcp_m: expected a number"), std::string::npos) << e[0];
  e = load([](XmlRpc::XmlRpcValue& t) { t["livgo"]["tdcp_gain_log"] = std::string("true"); });
  ASSERT_EQ(e.size(), 1u);
  EXPECT_NE(e[0].find("gnss/livgo/tdcp_gain_log: expected true or false"), std::string::npos) << e[0];
  e = load([](XmlRpc::XmlRpcValue& t) { t["livgo"]["frame_exat"] = true; });
  ASSERT_EQ(e.size(), 1u);
  EXPECT_NE(e[0].find("gnss/livgo/frame_exat: unknown key"), std::string::npos) << e[0];
}

// AMENDMENTS A80: the livgo/* keys of LIVGO-literal. Absent = the pre-A80 behaviour; wrong types and unknown spellings are refused.
TEST(Params, LivgoA80SwitchesParseAndDefaultToThePreA80Behaviour) {
  using livo_gnss::LivgoConfig;
  auto load = [](const std::function<void(XmlRpc::XmlRpcValue&)>& fill) {
    XmlRpc::XmlRpcValue t;
    t["enable"] = false;   // make it a struct
    fill(t);
    livo_gnss::Config c;
    std::vector<std::string> e;
    configFromParamTree(t, &c, &e);
    return e;
  };
  auto e = load([](XmlRpc::XmlRpcValue& t) { t["livgo"]["enable"] = true; });
  EXPECT_TRUE(e.empty()) << (e.empty() ? "" : e.front());
  {
    const LivgoConfig& l = livo_gnss_ros::detail::loadedLivgoConfig();
    EXPECT_FALSE(l.freeze_yaw);
    EXPECT_EQ(l.init_travel_m, 0.0);
    EXPECT_EQ(l.init_tilt_sigma_max_deg, 0.0);
    EXPECT_FALSE(l.cn0_hard);
    EXPECT_FALSE(l.primary_only);
    EXPECT_FALSE(l.tdcp_raim_both_ends);   // A81: default = the behaviour before A81
    EXPECT_EQ(l.dtw_grid_s, 0.1);
    EXPECT_EQ(l.dtw_window_s, 40.0);
    EXPECT_EQ(l.dtw_min_epochs, 30);
  }
  e = load([](XmlRpc::XmlRpcValue& t) {
    t["livgo"]["enable"] = true;
    t["livgo"]["freeze_yaw"] = true;
    t["livgo"]["init_travel_m"] = 50;               // an integer where a double is expected
    t["livgo"]["init_tilt_sigma_max_deg"] = 0.2;
    t["livgo"]["cn0_hard"] = true;
    t["livgo"]["cn0_min_dbhz"] = 35;
    t["livgo"]["el_min_deg"] = 15;
    t["livgo"]["primary_only"] = true;
    t["livgo"]["tdcp_raim_both_ends"] = true;
    t["livgo"]["dtw_grid_s"] = 0.05;
    t["livgo"]["dtw_window_s"] = 60;
    t["livgo"]["dtw_min_epochs"] = 100;
  });
  EXPECT_TRUE(e.empty()) << (e.empty() ? "" : e.front());
  {
    const LivgoConfig& l = livo_gnss_ros::detail::loadedLivgoConfig();
    EXPECT_TRUE(l.freeze_yaw);
    EXPECT_EQ(l.init_travel_m, 50.0);
    EXPECT_EQ(l.init_tilt_sigma_max_deg, 0.2);
    EXPECT_TRUE(l.cn0_hard);
    EXPECT_EQ(l.cn0_min_dbhz, 35.0);
    EXPECT_EQ(l.el_min_deg, 15.0);
    EXPECT_TRUE(l.primary_only);
    EXPECT_TRUE(l.tdcp_raim_both_ends);
    EXPECT_EQ(l.dtw_grid_s, 0.05);
    EXPECT_EQ(l.dtw_window_s, 60.0);
    EXPECT_EQ(l.dtw_min_epochs, 100);
  }
  // each call starts from the defaults again
  e = load([](XmlRpc::XmlRpcValue& t) { t["livgo"]["enable"] = true; });
  EXPECT_TRUE(e.empty());
  EXPECT_FALSE(livo_gnss_ros::detail::loadedLivgoConfig().freeze_yaw);
  EXPECT_EQ(livo_gnss_ros::detail::loadedLivgoConfig().init_travel_m, 0.0);
  EXPECT_EQ(livo_gnss_ros::detail::loadedLivgoConfig().dtw_window_s, 40.0);
  // refusals
  e = load([](XmlRpc::XmlRpcValue& t) { t["livgo"]["freeze_yaw"] = 1; });
  ASSERT_EQ(e.size(), 1u);
  EXPECT_NE(e[0].find("gnss/livgo/freeze_yaw: expected true or false"), std::string::npos) << e[0];
  e = load([](XmlRpc::XmlRpcValue& t) { t["livgo"]["cn0_hard"] = std::string("true"); });
  ASSERT_EQ(e.size(), 1u);
  EXPECT_NE(e[0].find("gnss/livgo/cn0_hard: expected true or false"), std::string::npos) << e[0];
  e = load([](XmlRpc::XmlRpcValue& t) { t["livgo"]["primary_only"] = 1.0; });
  ASSERT_EQ(e.size(), 1u);
  EXPECT_NE(e[0].find("gnss/livgo/primary_only: expected true or false"), std::string::npos) << e[0];
  e = load([](XmlRpc::XmlRpcValue& t) { t["livgo"]["init_travel_m"] = std::string("50 m"); });
  ASSERT_EQ(e.size(), 1u);
  EXPECT_NE(e[0].find("gnss/livgo/init_travel_m: expected a number"), std::string::npos) << e[0];
  e = load([](XmlRpc::XmlRpcValue& t) { t["livgo"]["init_tilt_sigma_max_deg"] = true; });
  ASSERT_EQ(e.size(), 1u);
  EXPECT_NE(e[0].find("gnss/livgo/init_tilt_sigma_max_deg: expected a number"), std::string::npos) << e[0];
  e = load([](XmlRpc::XmlRpcValue& t) { t["livgo"]["dtw_grid_s"] = std::string("0.1"); });
  ASSERT_EQ(e.size(), 1u);
  EXPECT_NE(e[0].find("gnss/livgo/dtw_grid_s: expected a number"), std::string::npos) << e[0];
  e = load([](XmlRpc::XmlRpcValue& t) { t["livgo"]["dtw_window_s"] = std::string("40"); });
  ASSERT_EQ(e.size(), 1u);
  EXPECT_NE(e[0].find("gnss/livgo/dtw_window_s: expected a number"), std::string::npos) << e[0];
  e = load([](XmlRpc::XmlRpcValue& t) { t["livgo"]["dtw_min_epochs"] = 30.5; });
  ASSERT_EQ(e.size(), 1u);
  EXPECT_NE(e[0].find("gnss/livgo/dtw_min_epochs: expected an integer"), std::string::npos) << e[0];
  e = load([](XmlRpc::XmlRpcValue& t) { t["livgo"]["tdcp_raim_both_ends"] = 1; });
  ASSERT_EQ(e.size(), 1u);
  EXPECT_NE(e[0].find("gnss/livgo/tdcp_raim_both_ends: expected true or false"), std::string::npos) << e[0];
  e = load([](XmlRpc::XmlRpcValue& t) { t["livgo"]["freez_yaw"] = true; });
  ASSERT_EQ(e.size(), 1u);
  EXPECT_NE(e[0].find("gnss/livgo/freez_yaw: unknown key"), std::string::npos) << e[0];
}

// AMENDMENTS A82: livgo/dtw_at_init and livgo/single_init parse as booleans (a number or a string is refused), and sit at the
// behaviour before A82 (false) when the key is absent.
TEST(Params, LivgoA82SwitchesParseAndDefaultToThePreA82Behaviour) {
  using livo_gnss::LivgoConfig;
  auto load = [](const std::function<void(XmlRpc::XmlRpcValue&)>& fill) {
    XmlRpc::XmlRpcValue t;
    t["enable"] = false;   // make it a struct
    fill(t);
    livo_gnss::Config c;
    std::vector<std::string> e;
    configFromParamTree(t, &c, &e);
    return e;
  };
  auto e = load([](XmlRpc::XmlRpcValue& t) { t["livgo"]["enable"] = true; });
  EXPECT_TRUE(e.empty()) << (e.empty() ? "" : e.front());
  EXPECT_FALSE(livo_gnss_ros::detail::loadedLivgoConfig().dtw_at_init);
  EXPECT_FALSE(livo_gnss_ros::detail::loadedLivgoConfig().single_init);
  e = load([](XmlRpc::XmlRpcValue& t) {
    t["livgo"]["enable"] = true;
    t["livgo"]["dtw_at_init"] = true;
    t["livgo"]["single_init"] = true;
  });
  EXPECT_TRUE(e.empty()) << (e.empty() ? "" : e.front());
  EXPECT_TRUE(livo_gnss_ros::detail::loadedLivgoConfig().dtw_at_init);
  EXPECT_TRUE(livo_gnss_ros::detail::loadedLivgoConfig().single_init);
  e = load([](XmlRpc::XmlRpcValue& t) {
    t["livgo"]["enable"] = true;
    t["livgo"]["single_init"] = true;
  });
  EXPECT_TRUE(e.empty());
  EXPECT_FALSE(livo_gnss_ros::detail::loadedLivgoConfig().dtw_at_init) << "each switch is its own key";
  EXPECT_TRUE(livo_gnss_ros::detail::loadedLivgoConfig().single_init);
  e = load([](XmlRpc::XmlRpcValue& t) { t["livgo"]["enable"] = true; });   // each call starts from the defaults again
  EXPECT_TRUE(e.empty());
  EXPECT_FALSE(livo_gnss_ros::detail::loadedLivgoConfig().single_init);
  e = load([](XmlRpc::XmlRpcValue& t) { t["livgo"]["dtw_at_init"] = 1; });
  ASSERT_EQ(e.size(), 1u);
  EXPECT_NE(e[0].find("gnss/livgo/dtw_at_init: expected true or false"), std::string::npos) << e[0];
  e = load([](XmlRpc::XmlRpcValue& t) { t["livgo"]["single_init"] = std::string("true"); });
  ASSERT_EQ(e.size(), 1u);
  EXPECT_NE(e[0].find("gnss/livgo/single_init: expected true or false"), std::string::npos) << e[0];
  e = load([](XmlRpc::XmlRpcValue& t) { t["livgo"]["single_inti"] = true; });
  ASSERT_EQ(e.size(), 1u);
  EXPECT_NE(e[0].find("gnss/livgo/single_inti: unknown key"), std::string::npos) << e[0];
}

// AMENDMENTS A80: the two arm files of LIVGO-literal, laid over the base configuration exactly as the launch does, give the arm the
// implementation record describes: every livgo/* switch, the 10 Hz consecutive-epoch fixed-anchor TDCP, post_vio, the flat noise
// of the paper variant only. (A misspelt key in a yaml would be refused by the table; a missing one would run another arm.)
TEST(Params, LivgoLiteralArmOverlaysLoadAsRegistered) {
  using livo_gnss::LivgoConfig;
  for (const char* ns : {"LIVGO_literal", "LIVGO_literal_paper"}) {
    const bool paper = std::string(ns) == "LIVGO_literal_paper";
    XmlRpc::XmlRpcValue base, top;
    ASSERT_TRUE(ros::param::get("/gnss", base));
    ASSERT_TRUE(ros::param::get(std::string("/p3b_test/arms/") + ns + "/gnss", top)) << ns;
    overlay(base, top);
    livo_gnss::Config c;
    std::vector<std::string> errors;
    EXPECT_TRUE(configFromParamTree(base, &c, &errors)) << ns << ": " << (errors.empty() ? "" : errors.front());
    EXPECT_TRUE(livo_gnss::validateConfig(c).empty()) << ns;
    EXPECT_TRUE(c.enable) << ns;
    EXPECT_TRUE(c.update_enable) << ns;
    EXPECT_EQ(c.mode, livo_gnss::Config::Mode::Tight) << ns;
    EXPECT_EQ(c.update_slot, livo_gnss::Config::UpdateSlot::PostVio) << ns;
    EXPECT_FALSE(c.use.psr) << ns;
    EXPECT_TRUE(c.use.dopp) << ns;
    EXPECT_TRUE(c.use.tdcp) << ns;
    EXPECT_FALSE(c.use.pvt_vel) << ns;
    EXPECT_EQ(c.masks.el_deg, 15.0) << ns;
    EXPECT_EQ(c.noise.tdcp.interval_s, 0.1) << ns;
    EXPECT_EQ(c.noise.tdcp.max_span_s, 0.15) << ns;
    EXPECT_TRUE(c.noise.tdcp.chain_pairs) << ns;
    EXPECT_FALSE(c.noise.dopp.exclusive_with_tdcp) << ns;
    EXPECT_TRUE(c.ablation.tdcp_fixed_anchor) << ns;
    EXPECT_FALSE(c.ablation.disable_l4_l5) << ns;
    const LivgoConfig& l = livo_gnss_ros::detail::loadedLivgoConfig();
    EXPECT_TRUE(l.enable) << ns;
    EXPECT_TRUE(l.paper_init) << ns;
    EXPECT_FALSE(l.dual_only) << ns;
    EXPECT_EQ(l.init_travel_m, 50.0) << ns;
    EXPECT_EQ(l.init_tilt_sigma_max_deg, 0.2) << ns;
    EXPECT_TRUE(l.frame_exact) << ns;
    EXPECT_TRUE(l.freeze_yaw) << ns;
    EXPECT_TRUE(l.cn0_hard) << ns;
    EXPECT_EQ(l.cn0_min_dbhz, 35.0) << ns;
    EXPECT_EQ(l.el_min_deg, 15.0) << ns;
    EXPECT_TRUE(l.primary_only) << ns;
    EXPECT_TRUE(l.tdcp_raim_both_ends) << ns;       // A81 section 1 item 1
    EXPECT_TRUE(l.dtw_online) << ns;
    EXPECT_EQ(l.dtw_grid_s, 0.1) << ns;
    EXPECT_TRUE(l.dtw_at_init) << ns;                  // A82 section 1: DTW at the travel gate, not after dtw_min_epochs epochs
    EXPECT_TRUE(l.single_init) << ns;                  // A82 section 3
    EXPECT_EQ(l.dtw_window_s, 40.0) << ns;
    EXPECT_EQ(l.dtw_min_epochs, 30) << ns;
    EXPECT_EQ(l.lambda_source, LivgoConfig::LambdaSource::Lio) << ns;
    EXPECT_EQ(l.lambda_th, 0.0) << ns;
    EXPECT_TRUE(l.tdcp_gain_log) << ns;
    EXPECT_EQ(l.noise_mode, paper ? LivgoConfig::NoiseMode::Flat : LivgoConfig::NoiseMode::Model) << ns;
    if (paper) {
      EXPECT_EQ(l.flat_sigma_tdcp_m, 0.001) << ns;
      EXPECT_EQ(l.flat_sigma_dopp_mps, 0.05) << ns;
    }
    // the masks the facade runs with: the paper's, whatever the base yaml says
    livo_gnss::Config::Masks m = c.masks;
    livo_gnss::livgoApplyMasks(l, &m);
    EXPECT_EQ(m.cn0_psr, 35.0) << ns;
    EXPECT_EQ(m.cn0_dopp, 35.0) << ns;
    EXPECT_EQ(m.cn0_cp, 35.0) << ns;
    EXPECT_EQ(m.el_deg, 15.0) << ns;
  }
}

// Every leaf of a parameter tree, keyed by its path, with the XML-RPC text of its value.
void treeLeaves(XmlRpc::XmlRpcValue& v, const std::string& path, std::map<std::string, std::string>* out) {
  if (v.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
    for (auto it = v.begin(); it != v.end(); ++it) treeLeaves(it->second, path + "/" + it->first, out);
  } else {
    (*out)[path] = v.toXml();
  }
}

// AMENDMENTS A81 section 2: the diagnostic twin of each LIVGO-literal arm is the arm plus gnss_sats.csv: its yaml has every leaf of the
// arm's and exactly one more, gnss/output/sat_csv true, and loads to the same configuration with only output.sat_csv differing.
TEST(Params, LivgoLiteralDiagArmsDifferOnlyInTheSatelliteCsv) {
  using livo_gnss::LivgoConfig;
  for (const std::string ns : {"LIVGO_literal", "LIVGO_literal_paper"}) {
    XmlRpc::XmlRpcValue arm, diag;
    ASSERT_TRUE(ros::param::get("/p3b_test/arms/" + ns + "/gnss", arm)) << ns;
    ASSERT_TRUE(ros::param::get("/p3b_test/arms/" + ns + "_diag/gnss", diag)) << ns << "_diag";
    std::map<std::string, std::string> la, ld;
    treeLeaves(arm, "gnss", &la);
    treeLeaves(diag, "gnss", &ld);
    ASSERT_EQ(la.count("gnss/output/sat_csv"), 0u) << ns;
    ASSERT_EQ(ld.count("gnss/output/sat_csv"), 1u) << ns;
    EXPECT_NE(ld["gnss/output/sat_csv"].find("<boolean>1</boolean>"), std::string::npos) << ns;
    ld.erase("gnss/output/sat_csv");
    EXPECT_EQ(la, ld) << ns << ": the diagnostic arm may differ from the arm only in output.sat_csv";
    // and through the loader, laid over the base configuration as the launch does
    livo_gnss::Config c[2];
    LivgoConfig l[2];
    XmlRpc::XmlRpcValue* tops[2] = {&arm, &diag};
    for (int i = 0; i < 2; ++i) {
      XmlRpc::XmlRpcValue base;
      ASSERT_TRUE(ros::param::get("/gnss", base));
      overlay(base, *tops[i]);
      std::vector<std::string> errors;
      EXPECT_TRUE(configFromParamTree(base, &c[i], &errors)) << ns << ": " << (errors.empty() ? "" : errors.front());
      EXPECT_TRUE(livo_gnss::validateConfig(c[i]).empty()) << ns;
      l[i] = livo_gnss_ros::detail::loadedLivgoConfig();
    }
    EXPECT_FALSE(c[0].output.sat_csv) << ns;
    EXPECT_TRUE(c[1].output.sat_csv) << ns;
    c[1].output.sat_csv = false;
    EXPECT_EQ(livo_gnss::canonicalDump(c[0]), livo_gnss::canonicalDump(c[1])) << ns;
    EXPECT_TRUE(l[1].enable && l[1].tdcp_raim_both_ends && l[1].freeze_yaw && l[1].cn0_hard && l[1].primary_only) << ns;
    EXPECT_TRUE(l[1].dtw_at_init && l[1].single_init) << ns;
    EXPECT_EQ(l[0].dtw_at_init, l[1].dtw_at_init) << ns;
    EXPECT_EQ(l[0].single_init, l[1].single_init) << ns;
    EXPECT_EQ(l[0].noise_mode, l[1].noise_mode) << ns;
    EXPECT_EQ(l[0].init_travel_m, l[1].init_travel_m) << ns;
    EXPECT_EQ(l[0].lambda_th, l[1].lambda_th) << ns;
  }
}

// The bridge's own start-up rules on top of validateConfig (A45 nav_rinex; unknown keys) and the zero channel.
TEST(Params, BridgeStartupRules) {
  {
    ros::NodeHandle nh("/p3b_case_navless");
    nh.setParam("gnss/enable", true);
    nh.setParam("gnss/ephem/ura_semantics", std::string("metres"));
    livo_gnss_ros::GnssRosBridge b;
    std::string why;
    EXPECT_FALSE(b.loadParams(nh, &why));
    EXPECT_NE(why.find("gnss.nav_rinex is empty"), std::string::npos) << why;
  }
  {
    ros::NodeHandle nh("/p3b_case_unknown");
    nh.setParam("gnss/enable", false);
    nh.setParam("gnss/masks/cn0_pssr", 12.0);
    livo_gnss_ros::GnssRosBridge b;
    std::string why;
    EXPECT_FALSE(b.loadParams(nh, &why)) << "unknown keys are refused even with gnss.enable false";
    EXPECT_NE(why.find("gnss/masks/cn0_pssr: unknown key"), std::string::npos) << why;
  }
  {
    ros::NodeHandle nh("/p3b_case_missing_file");
    nh.setParam("gnss/enable", true);
    nh.setParam("gnss/ephem/ura_semantics", std::string("metres"));
    nh.setParam("gnss/nav_rinex", std::vector<std::string>{"/nonexistent/BRDC.rnx"});
    livo_gnss_ros::GnssRosBridge b;
    std::string why;
    EXPECT_FALSE(b.loadParams(nh, &why));
    EXPECT_NE(why.find("cannot read '/nonexistent/BRDC.rnx'"), std::string::npos) << why;
  }
  {
    ros::NodeHandle nh("/p3b_case_gravity_align");
    nh.setParam("gnss/enable", true);
    nh.setParam("gnss/ephem/ura_semantics", std::string("metres"));
    nh.setParam("gnss/nav_rinex", std::vector<std::string>{dataDir() + "/Grass02_1421486476_43s/brdc/BRDC00IGS_R_20250210000_01D_MN.rnx"});
    nh.setParam("uav/gravity_align_en", true);
    livo_gnss_ros::GnssRosBridge b;
    std::string why;
    EXPECT_FALSE(b.loadParams(nh, &why));
    EXPECT_NE(why.find("gravity_align_en"), std::string::npos) << why;
  }
}

// gnss.enable = false: after loadParams/configure/setupRos nothing is active, the IMU hooks stay off, every hook is a
// no-op on the host state and nothing is written.
TEST(Params, ZeroChannelBridgeIsInert) {
  ros::NodeHandle nh("/p3b_case_zero");
  nh.setParam("gnss/enable", false);
  livo_gnss_ros::GnssRosBridge b;
  std::string why;
  ASSERT_TRUE(b.loadParams(nh, &why)) << why;
  ImuProcessPtr imu(new ImuProcess());
  ASSERT_TRUE(b.configure(M3D::Identity(), V3D::Zero(), LIVO, imu, &why)) << why;
  b.setupRos(nh, true);
  EXPECT_FALSE(b.enabled());
  EXPECT_EQ(b.fusion(), nullptr);
  EXPECT_FALSE(imu->gnss_hooks_enabled());
  EXPECT_FALSE(b.outputs()->isOpen());
  std::mt19937_64 rng(7);
  StatesGroup s = randomState(rng);
  const StatesGroup s0 = s;
  b.onLioStep(s, *imu, 123.0);
  b.onCoreUpdated(s0, s, livo_gnss::UpdateSource::LIO, 100);
  b.onPostVio(s, 123.0);
  b.onStepEnd(s, 123.0, 1);
  b.publishGlobal(s, 123.0);
  b.finish();
  EXPECT_TRUE(sameState(s, s0));
}

// ------------------------------------------------------------------------------------------------------------------
// Sat2Freq (A35-C8)
// ------------------------------------------------------------------------------------------------------------------
TEST(Sat2Freq, NominalCarriersPendingGlonassAndMismatches) {
  livo_gnss_ros::detail::Sat2FreqCheck chk;
  livo_gnss::ObsEpoch e;
  auto sat = [](livo_gnss::Sys sys, int prn) { return livo_gnss::makeSatIdOf(sys, prn); };
  auto add = [&e](const livo_gnss::SatId& s, livo_gnss::SignalCode c, double f) {
    livo_gnss::SatObs o;
    o.sat = s;
    livo_gnss::SignalObs sg;
    sg.code = c;
    sg.freq_hz = f;
    o.sig.push_back(sg);
    e.sats.push_back(o);
  };
  const livo_gnss::SatId r5 = sat(livo_gnss::Sys::GLO, 5), r7 = sat(livo_gnss::Sys::GLO, 7);
  add(sat(livo_gnss::Sys::GPS, 1), livo_gnss::SignalCode::L1C, 1575.42e6);
  add(sat(livo_gnss::Sys::BDS, 12), livo_gnss::SignalCode::L2I, 1561.098e6);
  add(sat(livo_gnss::Sys::GAL, 3), livo_gnss::SignalCode::L1X, 1575.42e6 + 5.0);   // 5 Hz off: mismatch
  add(r5, livo_gnss::SignalCode::L1C, 1602.0e6 - 3 * 0.5625e6);
  add(r7, livo_gnss::SignalCode::L1C, 1602.0e6 + 2 * 0.5625e6);
  ASSERT_TRUE(r5.valid() && r7.valid());
  chk.onObservation(e);
  chk.onObservation(e);   // one-time per (sat, code)
  auto r = chk.report();
  EXPECT_EQ(r.pairs, 3u);
  EXPECT_EQ(r.consistent, 2u);
  EXPECT_EQ(r.mismatched, 1u);
  EXPECT_EQ(r.pending, 2u) << "GLONASS without an ephemeris channel waits";
  chk.setGloChannel(r5.rtk_sat, -3);
  chk.setGloChannel(r7.rtk_sat, 1);   // wrong channel ⇒ mismatch
  r = chk.report();
  EXPECT_EQ(r.pairs, 5u);
  EXPECT_EQ(r.consistent, 3u);
  EXPECT_EQ(r.mismatched, 2u);
  EXPECT_EQ(r.pending, 0u);
  EXPECT_EQ(r.mismatches.size(), 2u);
}

// The Grass02 slice through the glue's decoding stage, set up as configure() sets it up — GLONASS channels from the
// day's NAV file (A38/A45: nav_rinex is a required input; here the fixture's cropped IGS BRDC), then the bag's own
// glo_ephem — : audit G14 found every slot of the whole bag consistent, the glue must agree on the slice.
TEST(Sat2Freq, Grass02FixtureIsConsistent) {
  const std::string fx = grassFixture();
  if (fx.empty()) GTEST_SKIP() << "blocked: dev-set fixture Grass02 (LIVO_GNSS_TEST_DATA_DIR=" << dataDir() << ")";
  GlueDecodePath p;
  const std::string brdc = livo_gnss_io::testsupport::fixtureBrdc(fx);
  ASSERT_FALSE(brdc.empty());
  livo_gnss_io::RinexSource rs;
  std::string w;
  ASSERT_TRUE(rs.open("", brdc, livo_gnss::GTime(), livo_gnss::GTime(), &w)) << w;
  ASSERT_FALSE(rs.gloEphemerides().empty());
  for (const livo_gnss::GloEph& g : rs.gloEphemerides()) p.check.setGloChannel(g.sat.rtk_sat, g.frq);
  for (const FixtureTopic& t : loadFixture(fx))
    if (t.src == livo_gnss::Source::GnssCommGloEphem)
      for (const auto& m : t.msgs) p.feed(t.src, t.type, t.md5, t.msgdef, m.second, m.first * 1e-9);
  for (const FixtureTopic& t : loadFixture(fx))
    if (t.src == livo_gnss::Source::GnssCommMeas)
      for (const auto& m : t.msgs) p.feed(t.src, t.type, t.md5, t.msgdef, m.second, m.first * 1e-9);
  EXPECT_EQ(p.errors, 0);
  const auto r = p.check.report();
  std::printf("[Sat2Freq] Grass02 slice: %llu (sat,code) pairs, %llu consistent, %llu mismatched, %llu pending, max |diff| %.3f Hz\n",
              static_cast<unsigned long long>(r.pairs), static_cast<unsigned long long>(r.consistent),
              static_cast<unsigned long long>(r.mismatched), static_cast<unsigned long long>(r.pending), r.max_abs_diff_hz);
  EXPECT_GE(r.pairs, 20u) << "the slice carries ~24 satellites on 1-2 accepted codes each";
  EXPECT_EQ(r.mismatched, 0u) << (r.mismatches.empty() ? "" : r.mismatches.front());
  EXPECT_EQ(r.pending, 0u) << "every GLONASS satellite has a channel from the NAV file or the bag";
}

// ------------------------------------------------------------------------------------------------------------------
// WGS-84
// ------------------------------------------------------------------------------------------------------------------
TEST(Wgs84, RoundTripAndEnuFrame) {
  for (double lat : {-89.0, -31.2, 0.0, 31.02, 88.5})
    for (double lon : {-179.0, 0.0, 121.43})
      for (double h : {-50.0, 12.0, 4000.0}) {
        livo_gnss::Geodetic g;
        g.lat_rad = lat * M_PI / 180.0;
        g.lon_rad = lon * M_PI / 180.0;
        g.h_m = h;
        const Eigen::Vector3d r = livo_gnss_ros::detail::geodeticToEcef(g);
        const livo_gnss::Geodetic b = livo_gnss_ros::detail::ecefToGeodetic(r);
        EXPECT_NEAR(b.lat_rad, g.lat_rad, 1e-10);
        EXPECT_NEAR(std::remainder(b.lon_rad - g.lon_rad, 2 * M_PI), 0.0, 1e-12);
        EXPECT_NEAR(b.h_m, g.h_m, 1e-4);
        const Eigen::Matrix3d R = livo_gnss_ros::detail::enuToEcefRotation(g.lat_rad, g.lon_rad);
        EXPECT_LT((R.transpose() * R - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff(), 1e-15);
        EXPECT_NEAR(R.determinant(), 1.0, 1e-15);
        g.h_m += 1.0;   // "up" moves along the third column
        EXPECT_LT((livo_gnss_ros::detail::geodeticToEcef(g) - r - R.col(2)).norm(), 1e-6);
      }
}

// ------------------------------------------------------------------------------------------------------------------
// I14
// ------------------------------------------------------------------------------------------------------------------
// "" when equal, otherwise where the first difference is (the containers are too large to print).
template <class V>
std::string firstDifference(const std::map<std::string, std::vector<V>>& a, const std::map<std::string, std::vector<V>>& b) {
  if (a.size() != b.size()) return "different channel sets (" + std::to_string(a.size()) + " vs " + std::to_string(b.size()) + ")";
  for (const auto& kv : a) {
    const auto it = b.find(kv.first);
    if (it == b.end()) return "channel " + kv.first + " missing";
    if (kv.second.size() != it->second.size())
      return kv.first + ": " + std::to_string(kv.second.size()) + " vs " + std::to_string(it->second.size()) + " items";
    for (size_t i = 0; i < kv.second.size(); ++i)
      if (!(kv.second[i] == it->second[i])) return kv.first + ": item " + std::to_string(i) + " differs";
  }
  return "";
}

std::string writeFixtureBag(const std::vector<FixtureTopic>& topics) {
  char tmpl[] = "/tmp/p3b_i14_XXXXXX";
  const int fd = mkstemp(tmpl);
  if (fd < 0) return std::string();
  close(fd);
  std::remove(tmpl);   // only the unique name is wanted; the bag is <name>.bag
  const std::string path = std::string(tmpl) + ".bag";
  rosbag::Bag bag(path, rosbag::bagmode::Write);
  // record order = time order across topics, as recorded
  std::vector<std::tuple<uint64_t, size_t, size_t>> order;
  for (size_t t = 0; t < topics.size(); ++t)
    for (size_t i = 0; i < topics[t].msgs.size(); ++i) order.emplace_back(topics[t].msgs[i].first, t, i);
  std::stable_sort(order.begin(), order.end());
  for (const auto& o : order) {
    const FixtureTopic& t = topics[std::get<1>(o)];
    const std::vector<uint8_t>& b = t.msgs[std::get<2>(o)].second;
    topic_tools::ShapeShifter ss;
    ss.morph(t.md5, t.type, t.msgdef, "");
    ros::serialization::IStream in(const_cast<uint8_t*>(b.data()), static_cast<uint32_t>(b.size()));
    ss.read(in);
    ros::Time ts;
    ts.fromNSec(std::get<0>(o));
    bag.write(t.topic, ts, ss);
  }
  bag.close();
  return path;
}

// No ROS graph: the bag round trip and the ShapeShifter read/write keep the fixture bytes and definitions exactly.
TEST(I14, BagAndShapeShifterKeepTheBytes) {
  const std::string fx = grassFixture();
  if (fx.empty()) GTEST_SKIP() << "blocked: dev-set fixture Grass02 (LIVO_GNSS_TEST_DATA_DIR=" << dataDir() << ")";
  const std::vector<FixtureTopic> topics = loadFixture(fx);
  ASSERT_EQ(topics.size(), 6u) << "Grass02 carries all six gnss_comm topics";
  const std::string bag_path = writeFixtureBag(topics);
  ASSERT_FALSE(bag_path.empty());
  std::map<std::string, size_t> next;
  std::map<std::string, const FixtureTopic*> by_topic;
  for (const FixtureTopic& t : topics) by_topic[t.topic] = &t;
  rosbag::Bag bag(bag_path, rosbag::bagmode::Read);
  size_t n = 0;
  for (const rosbag::MessageInstance& m : rosbag::View(bag)) {
    const FixtureTopic& t = *by_topic.at(m.getTopic());
    const auto& want = t.msgs.at(next[m.getTopic()]++);
    std::vector<uint8_t> got;
    ASSERT_TRUE(livo_gnss_ros::detail::messageBytes(m, &got));
    EXPECT_EQ(got, want.second) << m.getTopic();
    EXPECT_EQ(m.getDataType(), t.type);
    EXPECT_EQ(m.getMD5Sum(), t.md5);
    EXPECT_EQ(m.getMessageDefinition(), t.msgdef);
    EXPECT_EQ(m.getTime().toNSec(), want.first);
    boost::shared_ptr<topic_tools::ShapeShifter> ss = m.instantiate<topic_tools::ShapeShifter>();
    ASSERT_TRUE(ss != nullptr);
    std::vector<uint8_t> via_ss;
    ASSERT_TRUE(livo_gnss_ros::detail::messageBytes(*ss, &via_ss));
    EXPECT_EQ(via_ss, want.second);
    ++n;
  }
  bag.close();
  std::remove(bag_path.c_str());
  size_t total = 0;
  for (const FixtureTopic& t : topics) total += t.msgs.size();
  EXPECT_EQ(n, total);
}

// The live path for real: every bag message is published by a ShapeShifter publisher and received by a ShapeShifter
// subscriber through roscpp (serialised by the publisher, deserialised by the subscription, type/md5/definition from
// the connection header — as a live /ublox_driver topic arrives), while the replay path reads the same messages with
// rosbag::MessageInstance. Both go through the glue's decoding stage; bytes, definitions and records must agree.
TEST(I14, MessageInstanceAndShapeShifterConverge) {
  const std::string fx = grassFixture();
  if (fx.empty()) GTEST_SKIP() << "blocked: dev-set fixture Grass02 (LIVO_GNSS_TEST_DATA_DIR=" << dataDir() << ")";
  ASSERT_TRUE(ros::master::check()) << "rostest provides the master";
  const std::vector<FixtureTopic> topics = loadFixture(fx);
  const std::string bag_path = writeFixtureBag(topics);
  ASSERT_FALSE(bag_path.empty());
  std::map<std::string, livo_gnss::Source> src_of;
  for (const FixtureTopic& t : topics) src_of["/p3b_i14" + t.topic] = t.src;

  // replay path
  GlueDecodePath replay;
  std::map<std::string, std::vector<std::vector<uint8_t>>> bytes_replay, bytes_live;
  std::map<std::string, std::string> def_replay, def_live;
  rosbag::Bag bag(bag_path, rosbag::bagmode::Read);
  std::vector<rosbag::MessageInstance> msgs;
  for (const rosbag::MessageInstance& m : rosbag::View(bag)) msgs.push_back(m);
  for (const rosbag::MessageInstance& m : msgs) {
    const std::string topic = "/p3b_i14" + m.getTopic();
    std::vector<uint8_t> b;
    ASSERT_TRUE(livo_gnss_ros::detail::messageBytes(m, &b));
    bytes_replay[topic].push_back(b);
    def_replay[topic] = m.getDataType() + "\n" + m.getMD5Sum() + "\n" + m.getMessageDefinition();
    replay.feed(src_of.at(topic), m.getDataType(), m.getMD5Sum(), m.getMessageDefinition(), b, m.getTime().toSec());
  }

  // live path
  ros::NodeHandle nh;
  GlueDecodePath live;
  std::map<std::string, ros::Publisher> pubs;
  std::map<std::string, std::vector<double>> t_rec;   // the bag time of each published message (same t_record as replay)
  for (const rosbag::MessageInstance& m : msgs) t_rec["/p3b_i14" + m.getTopic()].push_back(m.getTime().toSec());
  std::map<std::string, size_t> received;
  std::vector<ros::Subscriber> subs;
  for (const FixtureTopic& t : topics) {
    const std::string topic = "/p3b_i14" + t.topic;
    topic_tools::ShapeShifter proto;
    proto.morph(t.md5, t.type, t.msgdef, "");
    pubs[topic] = proto.advertise(nh, topic, 100000, false);
    ros::SubscribeOptions so;
    so.initByFullCallbackType<const ros::MessageEvent<topic_tools::ShapeShifter const>&>(
        topic, 100000, [&, topic](const ros::MessageEvent<topic_tools::ShapeShifter const>& ev) {
          const topic_tools::ShapeShifter& ss = *ev.getConstMessage();
          std::vector<uint8_t> b;
          ASSERT_TRUE(livo_gnss_ros::detail::messageBytes(ss, &b));
          bytes_live[topic].push_back(b);
          def_live[topic] = ss.getDataType() + "\n" + ss.getMD5Sum() + "\n" + ss.getMessageDefinition();
          const size_t k = received[topic]++;
          live.feed(src_of.at(topic), ss.getDataType(), ss.getMD5Sum(), ss.getMessageDefinition(), b, t_rec.at(topic).at(k));
        });
    subs.push_back(nh.subscribe(so));
  }
  const ros::WallTime deadline = ros::WallTime::now() + ros::WallDuration(20.0);
  for (auto& kv : pubs)
    while (kv.second.getNumSubscribers() == 0 && ros::WallTime::now() < deadline) ros::WallDuration(0.01).sleep();
  for (const rosbag::MessageInstance& m : msgs) {
    boost::shared_ptr<topic_tools::ShapeShifter> ss = m.instantiate<topic_tools::ShapeShifter>();
    ASSERT_TRUE(ss != nullptr);
    pubs.at("/p3b_i14" + m.getTopic()).publish(*ss);   // by reference: serialised, then deserialised by the subscription
    ros::spinOnce();
  }
  size_t total = msgs.size(), got = 0;
  while (ros::WallTime::now() < deadline) {
    ros::spinOnce();
    got = 0;
    for (const auto& kv : received) got += kv.second;
    if (got == total) break;
    ros::WallDuration(0.01).sleep();
  }
  bag.close();
  std::remove(bag_path.c_str());
  ASSERT_EQ(got, total) << "messages received through the transport";
  EXPECT_EQ(firstDifference(bytes_live, bytes_replay), "") << "serialised bodies";
  EXPECT_TRUE(def_live == def_replay) << "datatype / md5 / message_definition";
  EXPECT_EQ(replay.errors, 0);
  EXPECT_EQ(live.errors, 0);
  EXPECT_EQ(firstDifference(live.sink.lines, replay.sink.lines), "") << "decoded records, field by field";
  EXPECT_EQ(replay.sink.lines.size(), 5u) << "obs, eph, glo, iono, pvt decoded (time_pulse_info is only counted, A19)";
  EXPECT_EQ(live.comm.stats().epochs, replay.comm.stats().epochs);
  EXPECT_EQ(live.comm.stats().time_pulse, replay.comm.stats().time_pulse);
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_p3b_glue");
  ros::Time::init();
  return RUN_ALL_TESTS();
}
