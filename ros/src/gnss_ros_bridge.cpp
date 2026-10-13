// SPDX-License-Identifier: GPL-2.0-only
// GnssRosBridge — ROS1 glue between LIVMapper and livo_gnss::GnssFusion (DESIGN §9, §12.2, §14, §15; IMPL_PLAN 7;
// AMENDMENTS A12, A35-C8, A38, A45). Owner: P3-B.
//
//   * loadParams: the `gnss:` parameter tree → livo_gnss::Config through one table that names every config.hpp field
//     (unknown keys, wrong types and unknown enum names are refused, nothing is ignored silently); validateConfig plus
//     the glue's own start-up rules (A45: nav_rinex required for gnss_comm; input files readable; gravity alignment
//     off). The table is checked against canonicalDump() so a field added to config.hpp cannot be missed.
//   * configure: constructs the core (which loads cfg.nav_rinex itself, gnss_fusion.hpp), sets the host-derived
//     settings and registers the gnss_io decoder wrapped in the A35-C8 sat2freq cross-check; enables the ImuProcess
//     bridge hooks. The glue reads the same NAV files once more only for the GLONASS channels of that check.
//   * bytes (I14): replay (rosbag::MessageInstance::write) and live (topic_tools::ShapeShifter::write) produce the same
//     serialised body and the same (datatype, md5, message_definition) triple; both meet in Impl::ingest →
//     GnssFusion::registerMessageDefinition (first contact of a channel) and GnssFusion::feedBytes.
//   * hooks: StatesGroup <-> CoreState through livo_host_view; the host is written back only when the core committed an
//     update and update_enable is true (OBSERVE never writes the host, whatever the core reports: bypass gate). Any
//     exception of the core disables GNSS for the rest of the run — the host goes on exactly as without GNSS — and is
//     reported loudly (ROS_ERROR, an events.log line, enabled() = false so fastlivo_replay exits non-zero).
//   * outputs: N-frame trajectories, anchor.yaml, /gnss/* topics and the TF enu→camera_init; the live node also writes
//     the host files (traj_W, filter_cov) itself. In fastlivo_replay the OutputWriter object is shared with the driver,
//     which opens it and writes every host file (A13), so no file is ever opened twice.
// With gnss.enable = false nothing below runs after loadParams: no core, no hooks, no writer, no topics, no files.
#include "livo_gnss_ros/gnss_ros_bridge.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <utility>

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <nav_msgs/Odometry.h>
#include <ros/serialization.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_datatypes.h>

#include "gnss_ros_detail.hpp"
#include "livo_gnss/gnss_fusion_livgo.hpp"
#include "livo_gnss/signal_table.hpp"
#include "livo_gnss_io/gnss_comm_mapping.hpp"
#include "livo_gnss_io/rinex_source.hpp"
#include "livo_gnss_io/ros1_decoder.hpp"
#include "livo_gnss_io/ros1_msgdef.hpp"
#include "livo_gnss_io/ubx_stream.hpp"
#include "livo_gnss_ros/livo_host_view.hpp"

namespace livo_gnss_ros {

namespace detail {

// =====================================================================================================================
// Parameter table (DESIGN §14 / config.hpp). One entry per Config field; the path is the canonicalDump key with '/'.
// =====================================================================================================================
namespace {

using XV = XmlRpc::XmlRpcValue;
using livo_gnss::Config;

struct Key {
  const char* path;
  // Assigns the value; on a type/value mismatch returns false and names what was expected in *want.
  std::function<bool(XV&, Config&, std::string*)> set;
};

#define FL2G_FIELD(expr) [](Config& c) -> decltype((c.expr)) { return c.expr; }

bool expected(std::string* want, const std::string& what) {
  *want = what;
  return false;
}

bool asNumber(XV& v, double* d) {
  if (v.getType() == XV::TypeDouble) {
    *d = static_cast<double>(v);
    return true;
  }
  if (v.getType() == XV::TypeInt) {
    *d = static_cast<int>(v);
    return true;
  }
  return false;
}

template <class G>
Key keyDouble(const char* p, G g) {
  return {p, [g](XV& v, Config& c, std::string* want) {
            double d;
            if (!asNumber(v, &d)) return expected(want, "a number");
            g(c) = d;
            return true;
          }};
}
template <class G>
Key keyInt(const char* p, G g) {
  return {p, [g](XV& v, Config& c, std::string* want) {
            if (v.getType() != XV::TypeInt) return expected(want, "an integer");
            g(c) = static_cast<int>(v);
            return true;
          }};
}
template <class G>
Key keyBool(const char* p, G g) {
  return {p, [g](XV& v, Config& c, std::string* want) {
            if (v.getType() != XV::TypeBoolean) return expected(want, "true or false");
            g(c) = static_cast<bool>(v);
            return true;
          }};
}
template <class G>
Key keyString(const char* p, G g) {
  return {p, [g](XV& v, Config& c, std::string* want) {
            if (v.getType() != XV::TypeString) return expected(want, "a string");
            g(c) = static_cast<std::string>(v);
            return true;
          }};
}
// List of strings; a single string is accepted as a one-element list (nav_rinex: one file per sequence day).
template <class G>
Key keyStringList(const char* p, G g) {
  return {p, [g](XV& v, Config& c, std::string* want) {
            std::vector<std::string> out;
            if (v.getType() == XV::TypeString) {
              out.push_back(static_cast<std::string>(v));
            } else if (v.getType() == XV::TypeArray) {
              for (int i = 0; i < v.size(); ++i) {
                if (v[i].getType() != XV::TypeString) return expected(want, "a list of strings");
                out.push_back(static_cast<std::string>(v[i]));
              }
            } else {
              return expected(want, "a list of strings");
            }
            g(c) = out;
            return true;
          }};
}
template <class G>
Key keyVec3(const char* p, G g) {
  return {p, [g](XV& v, Config& c, std::string* want) {
            if (v.getType() != XV::TypeArray || v.size() != 3) return expected(want, "a list of 3 numbers");
            double d[3];
            for (int i = 0; i < 3; ++i)
              if (!asNumber(v[i], &d[i])) return expected(want, "a list of 3 numbers");
            for (int i = 0; i < 3; ++i) g(c)[i] = d[i];
            return true;
          }};
}
// Per-system map {G: .., E: .., C: .., R: .., J: ..}: merged key by key into the defaults (a YAML that omits 'J' keeps
// the A15 default instead of failing validateConfig).
template <class G>
Key keySysMap(const char* p, G g) {
  return {p, [g](XV& v, Config& c, std::string* want) {
            if (v.getType() != XV::TypeStruct) return expected(want, "a map {G,E,C,R,J: number}");
            std::map<char, double> m = g(c);
            for (auto it = v.begin(); it != v.end(); ++it) {
              const std::string& k = it->first;
              double d;
              if (k.size() != 1 || std::string("GECRJ").find(k[0]) == std::string::npos || !asNumber(it->second, &d))
                return expected(want, "a map {G,E,C,R,J: number} (bad entry '" + k + "')");
              m[k[0]] = d;
            }
            g(c) = m;
            return true;
          }};
}
template <class E, class G>
Key keyEnum(const char* p, G g, std::vector<std::pair<const char*, E>> names) {
  return {p, [g, names](XV& v, Config& c, std::string* want) {
            std::string list;
            for (const auto& n : names) list += (list.empty() ? "" : "|") + std::string(n.first);
            if (v.getType() != XV::TypeString) return expected(want, "one of " + list);
            const std::string s = static_cast<std::string>(v);
            for (const auto& n : names)
              if (s == n.first) {
                g(c) = n.second;
                return true;
              }
            return expected(want, "one of " + list);
          }};
}
// time.leap: "auto" or an integer (YAML `leap: 18` arrives as an int); validateConfig judges the string (A40).
template <class G>
Key keyLeap(const char* p, G g) {
  return {p, [g](XV& v, Config& c, std::string* want) {
            if (v.getType() == XV::TypeString) {
              g(c) = static_cast<std::string>(v);
              return true;
            }
            if (v.getType() == XV::TypeInt) {
              g(c) = std::to_string(static_cast<int>(v));
              return true;
            }
            return expected(want, "'auto' or an integer");
          }};
}

const std::vector<Key>& table() {
  using M = Config::Mode;
  using U = Config::UpdateSlot;
  using S = Config::SourceKind;
  using LF = Config::Frames::LeverArmFrame;
  using TB = Config::Time::Base;
  using UR = Config::Ephem::UraSemantics;
  static const std::vector<Key> t = {
      keyBool("enable", FL2G_FIELD(enable)),
      keyBool("update_enable", FL2G_FIELD(update_enable)),
      keyEnum<M>("mode", FL2G_FIELD(mode),
                 {{"tight", M::Tight}, {"loose_pvt", M::LoosePvt}, {"livgo_probe", M::LivgoProbe}, {"livgo_probe_norej", M::LivgoProbeNorej}}),
      keyEnum<U>("update_slot", FL2G_FIELD(update_slot), {{"pre_lio", U::PreLio}, {"post_vio", U::PostVio}}),
      keyBool("use/psr", FL2G_FIELD(use.psr)),
      keyBool("use/dopp", FL2G_FIELD(use.dopp)),
      keyBool("use/tdcp", FL2G_FIELD(use.tdcp)),
      keyBool("use/pvt_vel", FL2G_FIELD(use.pvt_vel)),
      keyEnum<S>("source", FL2G_FIELD(source), {{"gnss_comm_ros1", S::GnssCommRos1}, {"rinex", S::Rinex}, {"ubx_stream", S::UbxStream}}),
      keyString("topics/meas", FL2G_FIELD(topics.meas)),
      keyString("topics/ephem", FL2G_FIELD(topics.ephem)),
      keyString("topics/glo_ephem", FL2G_FIELD(topics.glo_ephem)),
      keyString("topics/iono", FL2G_FIELD(topics.iono)),
      keyString("topics/pvt", FL2G_FIELD(topics.pvt)),
      keyString("topics/tp", FL2G_FIELD(topics.tp)),
      keyString("rinex/obs", FL2G_FIELD(rinex.obs)),
      keyString("rinex/nav", FL2G_FIELD(rinex.nav)),
      keyDouble("rinex/latency_s", FL2G_FIELD(rinex.latency_s)),
      keyStringList("nav_rinex", FL2G_FIELD(nav_rinex)),
      keyBool("systems/gps", FL2G_FIELD(systems.gps)),
      keyBool("systems/glo", FL2G_FIELD(systems.glo)),
      keyBool("systems/gal", FL2G_FIELD(systems.gal)),
      keyBool("systems/bds", FL2G_FIELD(systems.bds)),
      keyBool("systems/qzs_as_gps", FL2G_FIELD(systems.qzs_as_gps)),
      keyBool("systems/glo_psr", FL2G_FIELD(systems.glo_psr)),
      keyBool("systems/bds_geo/use", FL2G_FIELD(systems.bds_geo.use)),
      keyDouble("systems/bds_geo/psr_scale", FL2G_FIELD(systems.bds_geo.psr_scale)),
      keyStringList("exclude_sats", FL2G_FIELD(exclude_sats)),
      keyEnum<LF>("frames/lever_arm_frame", FL2G_FIELD(frames.lever_arm_frame), {{"lidar", LF::Lidar}, {"imu", LF::Imu}}),
      keyVec3("frames/lever_arm_xyz", FL2G_FIELD(frames.lever_arm_xyz)),
      keyDouble("frames/lever_arm_sigma", FL2G_FIELD(frames.lever_arm_sigma)),
      keyDouble("frames/pivot_distance_m", FL2G_FIELD(frames.pivot_distance_m)),
      keyDouble("frames/sigma_dov_rad", FL2G_FIELD(frames.sigma_dov_rad)),
      keyDouble("frames/sigma_lev_rad", FL2G_FIELD(frames.sigma_lev_rad)),
      keyBool("frames/gnss_updates_gravity", FL2G_FIELD(frames.gnss_updates_gravity)),
      keyEnum<TB>("time/base", FL2G_FIELD(time.base), {{"gpst", TB::Gpst}, {"utc", TB::Utc}, {"host", TB::Host}}),
      keyLeap("time/leap", FL2G_FIELD(time.leap)),
      keyDouble("time/offset_s", FL2G_FIELD(time.offset_s)),
      keyDouble("time/drift_ppm", FL2G_FIELD(time.drift_ppm)),
      keyDouble("time/t_ref_gpst_s", FL2G_FIELD(time.t_ref_gpst_s)),
      keyString("time/calib_file", FL2G_FIELD(time.calib_file)),
      keyDouble("time/sigma_toff_s", FL2G_FIELD(time.sigma_toff_s)),
      keyBool("time/estimate_online", FL2G_FIELD(time.estimate_online)),
      keyDouble("time/td_rw_s2_per_s", FL2G_FIELD(time.td_rw_s2_per_s)),
      keyDouble("time/max_bridge_s", FL2G_FIELD(time.max_bridge_s)),
      keyInt("time/bridge_ring_cycles", FL2G_FIELD(time.bridge_ring_cycles)),
      keyBool("time/toff_monitor/enable", FL2G_FIELD(time.toff_monitor.enable)),
      keyDouble("time/toff_monitor/window_s", FL2G_FIELD(time.toff_monitor.window_s)),
      keyDouble("time/toff_monitor/min_abs_s", FL2G_FIELD(time.toff_monitor.min_abs_s)),
      keyDouble("time/toff_monitor/n_sigma", FL2G_FIELD(time.toff_monitor.n_sigma)),
      keyDouble("imu_bridge/acc_n", FL2G_FIELD(imu_bridge.acc_n)),
      keyDouble("imu_bridge/gyr_n", FL2G_FIELD(imu_bridge.gyr_n)),
      keyDouble("masks/el_deg", FL2G_FIELD(masks.el_deg)),
      keyDouble("masks/cn0_psr", FL2G_FIELD(masks.cn0_psr)),
      keyDouble("masks/cn0_dopp", FL2G_FIELD(masks.cn0_dopp)),
      keyDouble("masks/cn0_cp", FL2G_FIELD(masks.cn0_cp)),
      keyInt("masks/track_psr", FL2G_FIELD(masks.track_psr)),
      keyInt("masks/track_cp", FL2G_FIELD(masks.track_cp)),
      keySysMap("masks/eph_age_s", FL2G_FIELD(masks.eph_age_s)),
      keyDouble("masks/max_psr_std_m", FL2G_FIELD(masks.max_psr_std_m)),
      keyDouble("masks/max_dopp_std_hz", FL2G_FIELD(masks.max_dopp_std_hz)),
      keyDouble("masks/max_cp_std_m", FL2G_FIELD(masks.max_cp_std_m)),
      keyDouble("masks/psr_min_m", FL2G_FIELD(masks.psr_min_m)),
      keyDouble("masks/psr_max_m", FL2G_FIELD(masks.psr_max_m)),
      keyDouble("masks/max_range_rate_mps", FL2G_FIELD(masks.max_range_rate_mps)),
      keyDouble("noise/psr/sigma0_m", FL2G_FIELD(noise.psr.sigma0_m)),
      keyDouble("noise/psr/cn0_ref", FL2G_FIELD(noise.psr.cn0_ref)),
      keyDouble("noise/psr/k_std", FL2G_FIELD(noise.psr.k_std)),
      keyDouble("noise/psr/tau_s", FL2G_FIELD(noise.psr.tau_s)),
      keyDouble("noise/psr/decim_s", FL2G_FIELD(noise.psr.decim_s)),
      keyDouble("noise/psr/err_brdci", FL2G_FIELD(noise.psr.err_brdci)),
      keySysMap("noise/psr/sys_factor", FL2G_FIELD(noise.psr.sys_factor)),
      keyDouble("noise/dopp/sigma0_mps", FL2G_FIELD(noise.dopp.sigma0_mps)),
      keyDouble("noise/dopp/tau_s", FL2G_FIELD(noise.dopp.tau_s)),
      keyBool("noise/dopp/exclusive_with_tdcp", FL2G_FIELD(noise.dopp.exclusive_with_tdcp)),
      keyBool("noise/dopp/consider_tilt", FL2G_FIELD(noise.dopp.consider_tilt)),                     // A66
      keyBool("noise/dopp/project_tilt", FL2G_FIELD(noise.dopp.project_tilt)),                       // A70
      keyDouble("noise/tdcp/sigma0_m", FL2G_FIELD(noise.tdcp.sigma0_m)),
      keyDouble("noise/tdcp/interval_s", FL2G_FIELD(noise.tdcp.interval_s)),
      keyDouble("noise/tdcp/sigma_rate_mps", FL2G_FIELD(noise.tdcp.sigma_rate_mps)),
      keyDouble("noise/tdcp/sigma_windup_m", FL2G_FIELD(noise.tdcp.sigma_windup_m)),
      keyDouble("noise/tdcp/max_span_s", FL2G_FIELD(noise.tdcp.max_span_s)),
      keyBool("noise/tdcp/chain_pairs", FL2G_FIELD(noise.tdcp.chain_pairs)),
      keyBool("noise/tdcp/phase_states", FL2G_FIELD(noise.tdcp.phase_states)),
      keyDouble("noise/pvt/tau_pos_s", FL2G_FIELD(noise.pvt.tau_pos_s)),
      keyDouble("noise/pvt/tau_vel_s", FL2G_FIELD(noise.pvt.tau_vel_s)),
      keyDouble("noise/pvt/decim_s", FL2G_FIELD(noise.pvt.decim_s)),
      keyDouble("noise/pvt/scale", FL2G_FIELD(noise.pvt.scale)),
      keyDouble("noise/scale/psr", FL2G_FIELD(noise.scale.psr)),
      keyDouble("noise/scale/dopp", FL2G_FIELD(noise.scale.dopp)),
      keyDouble("noise/scale/tdcp", FL2G_FIELD(noise.scale.tdcp)),
      keyDouble("process/clock/S_b", FL2G_FIELD(process.clock.S_b)),
      keyDouble("process/clock/S_d", FL2G_FIELD(process.clock.S_d)),
      keyBool("process/clock/S_b_min_windup", FL2G_FIELD(process.clock.S_b_min_windup)),
      keyDouble("process/isb_q", FL2G_FIELD(process.isb_q)),
      keyDouble("process/frame/q_tilt_per_m", FL2G_FIELD(process.frame.q_tilt_per_m)),
      keyDouble("process/frame/q_yaw_per_m", FL2G_FIELD(process.frame.q_yaw_per_m)),
      keyDouble("process/frame/q_yaw_per_rad", FL2G_FIELD(process.frame.q_yaw_per_rad)),
      keyDouble("process/frame/q_th_per_m", FL2G_FIELD(process.frame.q_th_per_m)),
      keyDouble("process/frame/q_tv_per_m", FL2G_FIELD(process.frame.q_tv_per_m)),
      keyDouble("process/frame/q_time", FL2G_FIELD(process.frame.q_time)),
      keyInt("init/min_livo_frames", FL2G_FIELD(init.min_livo_frames)),
      keyInt("init/min_sats", FL2G_FIELD(init.min_sats)),
      keyDouble("init/max_gdop", FL2G_FIELD(init.max_gdop)),
      keyInt("init/min_epochs", FL2G_FIELD(init.min_epochs)),
      keyDouble("init/min_window_s", FL2G_FIELD(init.min_window_s)),
      keyDouble("init/min_hspeed_mps", FL2G_FIELD(init.min_hspeed_mps)),
      keyInt("init/min_moving_epochs", FL2G_FIELD(init.min_moving_epochs)),
      keyDouble("init/min_path_m", FL2G_FIELD(init.min_path_m)),
      keyDouble("init/yaw_sigma_max_deg", FL2G_FIELD(init.yaw_sigma_max_deg)),
      keyDouble("init/spp_sigma_floor_m", FL2G_FIELD(init.spp_sigma_floor_m)),
      keyDouble("init/nlos_cn0_margin_db", FL2G_FIELD(init.nlos_cn0_margin_db)),
      keyDouble("rejection/alpha_w", FL2G_FIELD(rejection.alpha_w)),
      keyDouble("rejection/alpha_nis", FL2G_FIELD(rejection.alpha_nis)),
      keyDouble("rejection/max_reject_frac", FL2G_FIELD(rejection.max_reject_frac)),
      keyInt("rejection/min_meas/psr", FL2G_FIELD(rejection.min_meas.psr)),
      keyInt("rejection/min_meas/dopp", FL2G_FIELD(rejection.min_meas.dopp)),
      keyInt("rejection/min_meas/tdcp", FL2G_FIELD(rejection.min_meas.tdcp)),
      keyBool("rejection/post_check", FL2G_FIELD(rejection.post_check)),
      keyDouble("rejection/core_step_max_m", FL2G_FIELD(rejection.core_step_max_m)),
      keyDouble("rejection/clock_jump_km", FL2G_FIELD(rejection.clock_jump_km)),
      keyInt("rejection/lockout_epochs", FL2G_FIELD(rejection.lockout_epochs)),
      keyBool("slip/lli", FL2G_FIELD(slip.lli)),
      keyBool("slip/half_cycle", FL2G_FIELD(slip.half_cycle)),
      keyDouble("slip/max_gap_s", FL2G_FIELD(slip.max_gap_s)),
      keyDouble("slip/dopp_tol_cyc", FL2G_FIELD(slip.dopp_tol_cyc)),
      keyDouble("slip/jerk_max", FL2G_FIELD(slip.jerk_max)),
      keyDouble("slip/innov_abs_m", FL2G_FIELD(slip.innov_abs_m)),
      keyDouble("slip/gf_tol_m", FL2G_FIELD(slip.gf_tol_m)),
      keyBool("integrity/velocity_alarm/enable", FL2G_FIELD(integrity.velocity_alarm.enable)),
      keyDouble("integrity/velocity_alarm/alpha", FL2G_FIELD(integrity.velocity_alarm.alpha)),
      keyDouble("integrity/velocity_alarm/min_err_mps", FL2G_FIELD(integrity.velocity_alarm.min_err_mps)),
      keyInt("integrity/velocity_alarm/n_consec", FL2G_FIELD(integrity.velocity_alarm.n_consec)),
      keyInt("integrity/velocity_alarm/n_clear", FL2G_FIELD(integrity.velocity_alarm.n_clear)),
      keyBool("integrity/heading/enable", FL2G_FIELD(integrity.heading.enable)),
      keyDouble("integrity/heading/min_speed", FL2G_FIELD(integrity.heading.min_speed)),
      keyInt("integrity/heading/n_consec", FL2G_FIELD(integrity.heading.n_consec)),
      keyDouble("integrity/heading/min_err_deg", FL2G_FIELD(integrity.heading.min_err_deg)),
      keyDouble("integrity/degeneracy/lambda_enter", FL2G_FIELD(integrity.degeneracy.lambda_enter)),
      keyDouble("integrity/degeneracy/lambda_exit", FL2G_FIELD(integrity.degeneracy.lambda_exit)),
      keyInt("integrity/degeneracy/window", FL2G_FIELD(integrity.degeneracy.window)),
      keyBool("integrity/degeneracy/floor_enable", FL2G_FIELD(integrity.degeneracy.floor_enable)),
      keyDouble("integrity/degeneracy/sigma_floor_m", FL2G_FIELD(integrity.degeneracy.sigma_floor_m)),
      keyBool("integrity/frame_jump/enable", FL2G_FIELD(integrity.frame_jump.enable)),                // A61 F9
      keyDouble("integrity/frame_jump/chi2", FL2G_FIELD(integrity.frame_jump.chi2)),
      keyDouble("integrity/frame_jump/min_translation_m", FL2G_FIELD(integrity.frame_jump.min_translation_m)),
      keyDouble("integrity/frame_jump/min_rotation_rad", FL2G_FIELD(integrity.frame_jump.min_rotation_rad)),
      keyDouble("iterate/yaw_sigma_deg", FL2G_FIELD(iterate.yaw_sigma_deg)),
      keyDouble("iterate/dyaw_deg", FL2G_FIELD(iterate.dyaw_deg)),
      keyInt("iterate/max_iter", FL2G_FIELD(iterate.max_iter)),
      keyInt("iterate/reacq_epochs", FL2G_FIELD(iterate.reacq_epochs)),
      keyDouble("outage/outage_s", FL2G_FIELD(outage.outage_s)),
      keyInt("outage/recover_min_sats", FL2G_FIELD(outage.recover_min_sats)),
      keyDouble("outage/recover_settle_s", FL2G_FIELD(outage.recover_settle_s)),
      keyDouble("outage/reinit_after_s", FL2G_FIELD(outage.reinit_after_s)),
      keyDouble("outage/reinit_sigma_t_m", FL2G_FIELD(outage.reinit_sigma_t_m)),
      keyDouble("outage/reinit_sigma_psi_rad", FL2G_FIELD(outage.reinit_sigma_psi_rad)),
      keyBool("ablation/frame_fixed_after_init", FL2G_FIELD(ablation.frame_fixed_after_init)),
      keyBool("ablation/tdcp_fixed_anchor", FL2G_FIELD(ablation.tdcp_fixed_anchor)),
      keyBool("ablation/disable_l4_l5", FL2G_FIELD(ablation.disable_l4_l5)),
      keyString("output/frame_id", FL2G_FIELD(output.frame_id)),
      keyDouble("output/smooth_trans_mps", FL2G_FIELD(output.smooth_trans_mps)),
      keyDouble("output/smooth_yaw_dps", FL2G_FIELD(output.smooth_yaw_dps)),
      keyBool("output/odom_output", FL2G_FIELD(output.odom_output)),                                  // A61 F13
      keyBool("output/sat_csv", FL2G_FIELD(output.sat_csv)),
      keyEnum<UR>("ephem/ura_semantics", FL2G_FIELD(ephem.ura_semantics), {{"unknown", UR::Unknown}, {"index", UR::Index}, {"metres", UR::Metres}}),
      keyBool("ephem/gal_prefer_inav", FL2G_FIELD(ephem.gal_prefer_inav)),
      // Not Config fields. Written into loadedLivgoConfig() so the T-arm Config layout stays the pre-LIVGO one.
      {"livgo/enable", [](XV& v, Config&, std::string* want) {
         if (v.getType() != XV::TypeBoolean) return expected(want, "true or false");
         loadedLivgoConfig().enable = static_cast<bool>(v);
         return true;
       }},
      {"livgo/el_min_deg", [](XV& v, Config&, std::string* want) {
         double d;
         if (!asNumber(v, &d)) return expected(want, "a number");
         loadedLivgoConfig().el_min_deg = d;
         return true;
       }},
      {"livgo/cn0_min_dbhz", [](XV& v, Config&, std::string* want) {
         double d;
         if (!asNumber(v, &d)) return expected(want, "a number");
         loadedLivgoConfig().cn0_min_dbhz = d;
         return true;
       }},
      {"livgo/dual_only", [](XV& v, Config&, std::string* want) {
         if (v.getType() != XV::TypeBoolean) return expected(want, "true or false");
         loadedLivgoConfig().dual_only = static_cast<bool>(v);
         return true;
       }},
      {"livgo/paper_init", [](XV& v, Config&, std::string* want) {
         if (v.getType() != XV::TypeBoolean) return expected(want, "true or false");
         loadedLivgoConfig().paper_init = static_cast<bool>(v);
         return true;
       }},
      {"livgo/lambda_th", [](XV& v, Config&, std::string* want) {
         double d;
         if (!asNumber(v, &d)) return expected(want, "a number");
         loadedLivgoConfig().lambda_th = d;
         return true;
       }},
      {"livgo/hessian_window", [](XV& v, Config&, std::string* want) {
         if (v.getType() != XV::TypeInt) return expected(want, "an integer");
         loadedLivgoConfig().hessian_window = static_cast<int>(v);
         return true;
       }},
      {"livgo/dtw_min_epochs", [](XV& v, Config&, std::string* want) {
         if (v.getType() != XV::TypeInt) return expected(want, "an integer");
         loadedLivgoConfig().dtw_min_epochs = static_cast<int>(v);
         return true;
       }},
      // AMENDMENTS A75 (LIVGO-v2). Defaults of LivgoConfig are the pre-A75 behaviour.
      {"livgo/frame_exact", [](XV& v, Config&, std::string* want) {
         if (v.getType() != XV::TypeBoolean) return expected(want, "true or false");
         loadedLivgoConfig().frame_exact = static_cast<bool>(v);
         return true;
       }},
      {"livgo/lambda_source", [](XV& v, Config&, std::string* want) {
         if (v.getType() != XV::TypeString) return expected(want, "one of min|lio");
         const std::string t = static_cast<std::string>(v);
         if (t == "min") loadedLivgoConfig().lambda_source = livo_gnss::LivgoConfig::LambdaSource::Min;
         else if (t == "lio") loadedLivgoConfig().lambda_source = livo_gnss::LivgoConfig::LambdaSource::Lio;
         else return expected(want, "one of min|lio");
         return true;
       }},
      {"livgo/dtw_online", [](XV& v, Config&, std::string* want) {
         if (v.getType() != XV::TypeBoolean) return expected(want, "true or false");
         loadedLivgoConfig().dtw_online = static_cast<bool>(v);
         return true;
       }},
      {"livgo/noise_mode", [](XV& v, Config&, std::string* want) {
         if (v.getType() != XV::TypeString) return expected(want, "one of model|flat");
         const std::string t = static_cast<std::string>(v);
         if (t == "model") loadedLivgoConfig().noise_mode = livo_gnss::LivgoConfig::NoiseMode::Model;
         else if (t == "flat") loadedLivgoConfig().noise_mode = livo_gnss::LivgoConfig::NoiseMode::Flat;
         else return expected(want, "one of model|flat");
         return true;
       }},
      {"livgo/flat_sigma_tdcp_m", [](XV& v, Config&, std::string* want) {
         double d;
         if (!asNumber(v, &d)) return expected(want, "a number");
         loadedLivgoConfig().flat_sigma_tdcp_m = d;
         return true;
       }},
      {"livgo/flat_sigma_dopp_mps", [](XV& v, Config&, std::string* want) {
         double d;
         if (!asNumber(v, &d)) return expected(want, "a number");
         loadedLivgoConfig().flat_sigma_dopp_mps = d;
         return true;
       }},
      // AMENDMENTS A75 revision 1 (prediction P2): the observer that writes livgo_tdcp_gain.csv. Default false.
      {"livgo/tdcp_gain_log", [](XV& v, Config&, std::string* want) {
         if (v.getType() != XV::TypeBoolean) return expected(want, "true or false");
         loadedLivgoConfig().tdcp_gain_log = static_cast<bool>(v);
         return true;
       }},
      // AMENDMENTS A80 (LIVGO-literal). Defaults of LivgoConfig are the pre-A80 behaviour.
      {"livgo/freeze_yaw", [](XV& v, Config&, std::string* want) {
         if (v.getType() != XV::TypeBoolean) return expected(want, "true or false");
         loadedLivgoConfig().freeze_yaw = static_cast<bool>(v);
         return true;
       }},
      {"livgo/init_travel_m", [](XV& v, Config&, std::string* want) {
         double d;
         if (!asNumber(v, &d)) return expected(want, "a number");
         loadedLivgoConfig().init_travel_m = d;
         return true;
       }},
      {"livgo/init_tilt_sigma_max_deg", [](XV& v, Config&, std::string* want) {
         double d;
         if (!asNumber(v, &d)) return expected(want, "a number");
         loadedLivgoConfig().init_tilt_sigma_max_deg = d;
         return true;
       }},
      {"livgo/cn0_hard", [](XV& v, Config&, std::string* want) {
         if (v.getType() != XV::TypeBoolean) return expected(want, "true or false");
         loadedLivgoConfig().cn0_hard = static_cast<bool>(v);
         return true;
       }},
      {"livgo/primary_only", [](XV& v, Config&, std::string* want) {
         if (v.getType() != XV::TypeBoolean) return expected(want, "true or false");
         loadedLivgoConfig().primary_only = static_cast<bool>(v);
         return true;
       }},
      {"livgo/dtw_grid_s", [](XV& v, Config&, std::string* want) {
         double d;
         if (!asNumber(v, &d)) return expected(want, "a number");
         loadedLivgoConfig().dtw_grid_s = d;
         return true;
       }},
      {"livgo/dtw_window_s", [](XV& v, Config&, std::string* want) {
         double d;
         if (!asNumber(v, &d)) return expected(want, "a number");
         loadedLivgoConfig().dtw_window_s = d;
         return true;
       }},
      // AMENDMENTS A81. Default false = the behaviour before A81.
      {"livgo/tdcp_raim_both_ends", [](XV& v, Config&, std::string* want) {
         if (v.getType() != XV::TypeBoolean) return expected(want, "true or false");
         loadedLivgoConfig().tdcp_raim_both_ends = static_cast<bool>(v);
         return true;
       }},
      // AMENDMENTS A82. Default false = the behaviour before A82.
      {"livgo/dtw_at_init", [](XV& v, Config&, std::string* want) {
         if (v.getType() != XV::TypeBoolean) return expected(want, "true or false");
         loadedLivgoConfig().dtw_at_init = static_cast<bool>(v);
         return true;
       }},
      {"livgo/single_init", [](XV& v, Config&, std::string* want) {
         if (v.getType() != XV::TypeBoolean) return expected(want, "true or false");
         loadedLivgoConfig().single_init = static_cast<bool>(v);
         return true;
       }},
  };
  return t;
}

#undef FL2G_FIELD

const char* typeName(const XV& v) {
  switch (v.getType()) {
    case XV::TypeBoolean: return "bool";
    case XV::TypeInt: return "int";
    case XV::TypeDouble: return "double";
    case XV::TypeString: return "string";
    case XV::TypeArray: return "list";
    case XV::TypeStruct: return "map";
    default: return "other";
  }
}

void walk(XV& node, const std::string& prefix, const std::map<std::string, const Key*>& keys, const std::set<std::string>& namespaces,
          Config* cfg, std::vector<std::string>* errors) {
  for (auto it = node.begin(); it != node.end(); ++it) {
    const std::string path = prefix.empty() ? it->first : prefix + "/" + it->first;
    const auto k = keys.find(path);
    if (k != keys.end()) {
      std::string want;
      if (!k->second->set(it->second, *cfg, &want))
        errors->push_back("gnss/" + path + ": expected " + want + ", got a " + typeName(it->second));
    } else if (namespaces.count(path) != 0 && it->second.getType() == XV::TypeStruct) {
      walk(it->second, path, keys, namespaces, cfg, errors);
    } else if (namespaces.count(path) != 0) {
      errors->push_back("gnss/" + path + ": expected a map (namespace of config.hpp), got a " + typeName(it->second));
    } else {
      errors->push_back("gnss/" + path + ": unknown key (not a field of livo_gnss::Config, DESIGN §14)");
    }
  }
}

}  // namespace

livo_gnss::LivgoConfig& loadedLivgoConfig() {
  static livo_gnss::LivgoConfig cfg;
  return cfg;
}

bool configFromParamTree(XmlRpc::XmlRpcValue& gnss, livo_gnss::Config* cfg, std::vector<std::string>* errors) {
  loadedLivgoConfig() = livo_gnss::LivgoConfig();
  const size_t n0 = errors->size();
  std::map<std::string, const Key*> keys;
  std::set<std::string> namespaces;
  for (const Key& k : table()) {
    keys[k.path] = &k;
    std::string p = k.path;
    for (size_t s = p.find('/'); s != std::string::npos; s = p.find('/', s + 1)) namespaces.insert(p.substr(0, s));
  }
  if (gnss.getType() != XV::TypeStruct) {
    errors->push_back(std::string("gnss: expected a map, got a ") + typeName(gnss));
    return false;
  }
  walk(gnss, "", keys, namespaces, cfg, errors);
  return errors->size() == n0;
}

std::vector<std::string> configKeyPaths() {
  std::vector<std::string> out;
  for (const Key& k : table()) out.push_back(k.path);
  return out;
}

std::vector<std::string> loaderCoverageProblems() {
  std::set<std::string> dump_keys;
  std::istringstream in(livo_gnss::canonicalDump(livo_gnss::Config()));
  std::string line;
  while (std::getline(in, line)) {
    const size_t eq = line.find('=');
    if (eq == std::string::npos) continue;
    std::string k = line.substr(0, eq);
    std::replace(k.begin(), k.end(), '.', '/');
    dump_keys.insert(k);
  }
  std::set<std::string> table_keys;
  for (const Key& k : table()) table_keys.insert(k.path);
  std::vector<std::string> out;
  for (const auto& k : dump_keys)
    if (!table_keys.count(k)) out.push_back("config.hpp field '" + k + "' has no gnss/ parameter entry in gnss_ros_bridge.cpp");
  for (const auto& k : table_keys)
    if (!dump_keys.count(k) && k.compare(0, 6, "livgo/") != 0)
      out.push_back("parameter entry '" + k + "' names no field of canonicalDump(Config)");
  return out;
}

// =====================================================================================================================
// Bytes (I14)
// =====================================================================================================================
bool messageBytes(const rosbag::MessageInstance& m, std::vector<uint8_t>* out) {
  try {
    out->assign(m.size(), 0);
    ros::serialization::OStream s(out->data(), static_cast<uint32_t>(out->size()));
    m.write(s);
    return true;
  } catch (const std::exception&) {
    out->clear();
    return false;
  }
}

bool messageBytes(const topic_tools::ShapeShifter& m, std::vector<uint8_t>* out) {
  try {
    out->assign(m.size(), 0);
    ros::serialization::OStream s(out->data(), static_cast<uint32_t>(out->size()));
    m.write(s);
    return true;
  } catch (const std::exception&) {
    out->clear();
    return false;
  }
}

// =====================================================================================================================
// A35-C8 sat2freq cross-check
// =====================================================================================================================
void Sat2FreqCheck::setGloChannel(int rtk_sat, int k) {
  if (rtk_sat <= 0) return;
  glo_k_[rtk_sat] = k;
  for (auto it = pending_.begin(); it != pending_.end();) {
    if (it->first.first == rtk_sat) {
      check(it->second.sat, it->second.code, it->second.freq_hz, k);
      it = pending_.erase(it);
    } else {
      ++it;
    }
  }
}

void Sat2FreqCheck::onObservation(const livo_gnss::ObsEpoch& e) {
  for (const livo_gnss::SatObs& s : e.sats) {
    if (!s.sat.valid()) continue;
    for (const livo_gnss::SignalObs& sig : s.sig) {
      if (sig.code == livo_gnss::SignalCode::Unknown) continue;
      const std::pair<int, int> key(s.sat.rtk_sat, static_cast<int>(sig.code));
      if (!seen_.insert(key).second) continue;
      if (s.sat.sys == livo_gnss::Sys::GLO) {
        const auto k = glo_k_.find(s.sat.rtk_sat);
        if (k == glo_k_.end()) {
          pending_[key] = Pending{s.sat, sig.code, sig.freq_hz};
          continue;
        }
        check(s.sat, sig.code, sig.freq_hz, k->second);
      } else {
        check(s.sat, sig.code, sig.freq_hz, 0);
      }
    }
  }
}

void Sat2FreqCheck::check(const livo_gnss::SatId& sat, livo_gnss::SignalCode code, double freq_hz, int k) {
  // RTKLIB sat2freq: QZSS uses the GPS carriers (FREQ1/FREQ2/FREQ5); nominalFrequencyHz holds the same constants.
  const double expected = livo_gnss::nominalFrequencyHz(sat.sys, code, k);
  ++rep_.pairs;
  const double diff = std::fabs(freq_hz - expected);
  if (expected > 0.0) rep_.max_abs_diff_hz = std::max(rep_.max_abs_diff_hz, diff);
  if (expected > 0.0 && diff <= kTolHz) {
    ++rep_.consistent;
    return;
  }
  ++rep_.mismatched;
  if (rep_.mismatches.size() < 50) {
    char b[160];
    std::snprintf(b, sizeof b, "%s %s: message %.4f MHz, sat2freq %.4f MHz%s", livo_gnss::satIdStringOf(sat).c_str(),
                  livo_gnss::signalCodeName(code), freq_hz * 1e-6, expected * 1e-6, sat.sys == livo_gnss::Sys::GLO ? " (channel from ephemeris)" : "");
    rep_.mismatches.push_back(b);
  }
}

Sat2FreqCheck::Report Sat2FreqCheck::report() const {
  Report r = rep_;
  r.pending = pending_.size();
  return r;
}

void CheckingSink::pushObservation(livo_gnss::ObsEpoch&& epoch) {
  if (check_) check_->onObservation(epoch);
  next_.pushObservation(std::move(epoch));
}

void CheckingSink::pushGloEphemeris(const livo_gnss::GloEph& eph) {
  if (check_) check_->setGloChannel(eph.sat.rtk_sat, eph.frq);
  next_.pushGloEphemeris(eph);
}

bool CheckedDecoder::registerMessageDefinition(livo_gnss::Source src, const std::string& datatype, const std::string& md5,
                                               const std::string& msg_def, std::string* why) {
  return inner_->registerMessageDefinition(src, datatype, md5, msg_def, why);
}

bool CheckedDecoder::decode(livo_gnss::Source src, const uint8_t* data, size_t size, double t_record, livo_gnss::MeasurementSink& sink,
                            std::string* why) {
  CheckingSink checked(sink, check_);
  return inner_->decode(src, data, size, t_record, checked, why);
}

// =====================================================================================================================
// WGS-84
// =====================================================================================================================
namespace {
constexpr double kWgsA = 6378137.0;
constexpr double kWgsF = 1.0 / 298.257223563;
}  // namespace

Eigen::Vector3d geodeticToEcef(const livo_gnss::Geodetic& g) {
  const double e2 = kWgsF * (2.0 - kWgsF);
  const double sp = std::sin(g.lat_rad), cp = std::cos(g.lat_rad), sl = std::sin(g.lon_rad), cl = std::cos(g.lon_rad);
  const double v = kWgsA / std::sqrt(1.0 - e2 * sp * sp);
  return Eigen::Vector3d((v + g.h_m) * cp * cl, (v + g.h_m) * cp * sl, (v * (1.0 - e2) + g.h_m) * sp);
}

livo_gnss::Geodetic ecefToGeodetic(const Eigen::Vector3d& r) {
  const double e2 = kWgsF * (2.0 - kWgsF);
  const double r2 = r.x() * r.x() + r.y() * r.y();
  double z = r.z(), zk = 0.0, v = kWgsA;
  for (int it = 0; it < 30 && std::fabs(z - zk) >= 1e-4; ++it) {
    zk = z;
    const double sp = z / std::sqrt(r2 + z * z);
    v = kWgsA / std::sqrt(1.0 - e2 * sp * sp);
    z = r.z() + v * e2 * sp;
  }
  livo_gnss::Geodetic g;
  g.lat_rad = r2 > 1e-12 ? std::atan(z / std::sqrt(r2)) : (r.z() > 0.0 ? M_PI / 2.0 : -M_PI / 2.0);
  g.lon_rad = r2 > 1e-12 ? std::atan2(r.y(), r.x()) : 0.0;
  g.h_m = std::sqrt(r2 + z * z) - v;
  return g;
}

Eigen::Matrix3d enuToEcefRotation(double lat_rad, double lon_rad) {
  const double sp = std::sin(lat_rad), cp = std::cos(lat_rad), sl = std::sin(lon_rad), cl = std::cos(lon_rad);
  Eigen::Matrix3d R;
  R << -sl, -sp * cl, cp * cl,   //
      cl, -sp * sl, cp * sl,     //
      0.0, cp, sp;
  return R;
}

}  // namespace detail

// =====================================================================================================================
// GnssRosBridge
// =====================================================================================================================
namespace {

bool fileReadable(const std::string& p) {
  std::ifstream f(p, std::ios::binary);
  return f.good();
}

std::string join(const std::vector<std::string>& v, const char* sep) {
  std::string s;
  for (size_t i = 0; i < v.size(); ++i) s += (i ? sep : "") + v[i];
  return s;
}

bool sameBits(const Eigen::Vector3d& a, const Eigen::Vector3d& b) { return std::memcmp(a.data(), b.data(), sizeof(double) * 3) == 0; }

// 6x6 covariance [δθ, δp] (globalPose) → ROS pose covariance order (x y z, rot x y z), row-major.
void rosPoseCovariance(const Eigen::Matrix<double, 6, 6>& c_rot_pos, boost::array<double, 36>* out) {
  const int map[6] = {3, 4, 5, 0, 1, 2};   // ROS index i ← core index map[i]
  for (int i = 0; i < 6; ++i)
    for (int j = 0; j < 6; ++j) (*out)[static_cast<size_t>(6 * i + j)] = c_rot_pos(map[i], map[j]);
}

}  // namespace

struct GnssRosBridge::Impl {
  livo_gnss::Config cfg;
  bool loaded = false, configured = false, failed = false, finished = false, replay_mode = false;
  std::string failure;

  // Non-gnss parameters the glue needs (DESIGN §14 output:/debug:, upstream uav:).
  OutputOptions out_opt;
  bool gravity_align_en = false;

  // Outputs, decoders, core — in this order, so the core (which holds non-owning pointers to the writer as its
  // diagnostics sink and to the decoder) is destroyed first.
  OutputWriter outputs;
  bool own_outputs = false;   // live node: this bridge opened the writer (replay: fastlivo_replay opens it)
  ImuProcessPtr imu;
  detail::Sat2FreqCheck sat2freq;
  std::unique_ptr<livo_gnss_io::GnssCommRos1Decoder> comm;
  std::unique_ptr<livo_gnss_io::UbxStream> ubx;
  std::unique_ptr<detail::CheckedDecoder> decoder;
  std::unique_ptr<livo_gnss::GnssFusion> fusion;
  // FAST-LIVGO is a different type in a different translation unit. Exactly one of the two pointers is set,
  // so the T/U GnssFusion object code is never entered when the arm is on, and never shares a type with it.
  livo_gnss::LivgoConfig livgo_cfg;
  std::unique_ptr<livo_gnss::GnssFusionLivgo> fusion_livgo;

  // const: covRecord() is const and already called non-const methods through the pointer.
  template <class F>
  auto use(F&& f) const -> decltype(f(std::declval<livo_gnss::GnssFusion&>())) {
    if (livgo_cfg.enable) return f(*fusion_livgo);
    return f(*fusion);
  }
  void dropCore() {
    fusion.reset();
    fusion_livgo.reset();
  }
  std::map<std::string, livo_gnss::Source> topic_src;
  std::map<livo_gnss::Source, int> channel_state;   // 0 unseen, 1 registered, -1 rejected
  struct ChannelStats {
    uint64_t msgs = 0, bytes = 0, dropped = 0, byte_errors = 0;
  };
  std::map<livo_gnss::Source, ChannelStats> channel_stats;
  // source: ubx_stream — the ROS wrapper message (GnssRawStream{header, format, uint8[] data}) is parsed from its own
  // message_definition to reach the UBX bytes.
  std::unique_ptr<livo_gnss_io::Ros1Decoder> ubx_wrapper;

  // ROS.
  std::vector<ros::Subscriber> subs;
  ros::Publisher pub_odom, pub_odom_smooth, pub_fused_fix, pub_spp_fix, pub_diag;
  std::unique_ptr<tf::TransformBroadcaster> tf_br;
  bool pubs_ready = false;

  // Hook bookkeeping.
  double last_t_k = 0.0, last_diag_t = -1.0e300;
  uint64_t n_lio_steps = 0, n_no_bridge = 0, n_commits = 0, n_post_vio_commits = 0;
  bool snapshot_warned = false;
  StatesGroup last_prior;
  bool have_last_prior = false;
  livo_gnss::GTime anchor_t_written;
  bool anchor_written = false;
  livo_gnss::GTime last_spp_t;

  bool active() const {
    return cfg.enable && configured && !failed && (fusion != nullptr || fusion_livgo != nullptr);
  }

  void failSafe(const char* where, const std::string& what) {
    if (failed) return;
    failed = true;
    failure = std::string(where) + ": " + what;
    ROS_ERROR("[gnss] GNSS disabled for the rest of the run after an error in %s — the host continues without GNSS. %s", where,
              what.c_str());
    livo_gnss::Event ev;
    ev.kind = livo_gnss::EventKind::CONFIG_WARNING;
    ev.t_k = last_t_k;
    ev.detail = "GNSS_FAILSAFE " + failure;
    outputs.onEvent(ev);
    if (imu) imu->enable_gnss_hooks(false);
  }

  // Byte convergence point of both sources (DESIGN §12.2, I14).
  void ingest(livo_gnss::Source src, const std::string& datatype, const std::string& md5, const std::string& msg_def,
              const std::vector<uint8_t>& bytes, double t_record);
  bool ubxPayload(const std::vector<uint8_t>& bytes, std::vector<uint8_t>* ubx_bytes, std::string* why) const;

  livo_gnss::CovRecord covRecord(const StatesGroup& s, const StatesGroup& prior, const char* stage, double t_k, int effct) const;
  void writeStepOutputs(const livo_gnss::CoreState& core, const StatesGroup& state, double t_k);
  void publishDiagnostics(double t_k);
};

void GnssRosBridge::Impl::ingest(livo_gnss::Source src, const std::string& datatype, const std::string& md5, const std::string& msg_def,
                                 const std::vector<uint8_t>& bytes, double t_record) {
  ChannelStats& cs = channel_stats[src];
  cs.msgs++;
  cs.bytes += bytes.size();
  int& state = channel_state[src];
  if (state < 0) {
    cs.dropped++;
    return;
  }
  try {
    if (state == 0) {
      std::string why;
      bool ok = true;
      if (src == livo_gnss::Source::UbxStream) {
        livo_gnss_io::MsgSchema schema;
        ok = livo_gnss_io::MsgDefParser::parse(datatype, msg_def, &schema, &why);
        if (ok) ubx_wrapper.reset(new livo_gnss_io::Ros1Decoder(schema));
      }
      ok = ok && use([&](auto& g){ return g.registerMessageDefinition(src, datatype, md5, msg_def); });   // false → the core logs MSGDEF_REJECT
      state = ok ? 1 : -1;
      if (!ok) {
        ROS_ERROR("[gnss] message definition of %s (%s, md5 %s) rejected%s%s; the channel is ignored", livo_gnss::sourceName(src),
                  datatype.c_str(), md5.c_str(), why.empty() ? "" : ": ", why.c_str());
        cs.dropped++;
        return;
      }
    }
    if (src == livo_gnss::Source::UbxStream) {
      std::vector<uint8_t> ubx_bytes;
      std::string why;
      if (!ubxPayload(bytes, &ubx_bytes, &why)) {
        cs.byte_errors++;
        ROS_WARN_THROTTLE(10.0, "[gnss] UBX wrapper message not decodable: %s", why.c_str());
        return;
      }
      use([&](auto& g){ return g.feedBytes(src, ubx_bytes.data(), ubx_bytes.size(), t_record); });
    } else {
      use([&](auto& g){ return g.feedBytes(src, bytes.data(), bytes.size(), t_record); });
    }
  } catch (const std::exception& e) {
    failSafe("feedBytes", e.what());
  }
}

bool GnssRosBridge::Impl::ubxPayload(const std::vector<uint8_t>& bytes, std::vector<uint8_t>* ubx_bytes, std::string* why) const {
  if (!ubx_wrapper) {
    *why = "no wrapper schema";
    return false;
  }
  livo_gnss_io::FieldNode root;
  size_t used = 0;
  if (!ubx_wrapper->decode(bytes.data(), bytes.size(), &root, &used, why)) return false;
  const livo_gnss_io::FieldNode* data = root.child("data");
  if (!data || !data->is_array) {
    *why = "no uint8[] field 'data'";
    return false;
  }
  ubx_bytes->clear();
  ubx_bytes->reserve(data->array.size());
  for (const livo_gnss_io::Value& v : data->array) ubx_bytes->push_back(static_cast<uint8_t>(v.u));
  return true;
}

livo_gnss::CovRecord GnssRosBridge::Impl::covRecord(const StatesGroup& s, const StatesGroup& prior, const char* stage, double t_k, int effct) const {
  livo_gnss::CovRecord r;
  r.t_k = t_k;
  r.stage = stage;
  r.effct_feat_num = effct;
  for (int i = 0; i < DIM_STATE; ++i) r.host_sigma[i] = std::sqrt(std::max(0.0, s.cov(i, i)));
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> ep(s.cov.block<3, 3>(3, 3));
  r.pos_eig = ep.eigenvalues();
  r.pos_eigvec = ep.eigenvectors();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> er(s.cov.block<3, 3>(0, 0));
  r.rot_eig = er.eigenvalues();
  r.rot_eigvec = er.eigenvectors();
  r.min_diag_prior = prior.cov.diagonal().minCoeff();
  Eigen::SelfAdjointEigenSolver<livo_gnss::HostMat> ea(prior.cov, Eigen::EigenvaluesOnly);
  const double lmin = ea.eigenvalues().minCoeff(), lmax = ea.eigenvalues().maxCoeff();
  r.cond_prior = (lmin > 0.0) ? lmax / lmin : std::numeric_limits<double>::infinity();
  if ((fusion || fusion_livgo) && !failed) {
    Eigen::VectorXd x_G, dq;
    use([&](auto& g){ return g.augmentedState(&x_G, &dq, nullptr); });
    r.aug_sigma = dq.cwiseMax(0.0).cwiseSqrt();
  }
  return r;
}

void GnssRosBridge::Impl::writeStepOutputs(const livo_gnss::CoreState& core, const StatesGroup& state, double t_k) {
  if (!outputs.isOpen()) return;
  if (own_outputs && out_opt.traj_w) outputs.writeTrajW(t_k, state.rot_end, state.pos_end);   // replay: the driver writes traj_W
  Eigen::Matrix3d R_NI;
  Eigen::Vector3d t_NI;
  Eigen::Matrix<double, 6, 6> cov;
  if (!use([&](auto& g){ return g.globalPose(core, &R_NI, &t_NI, &cov); })) return;   // N exists only after ACTIVATE
  outputs.writeTrajN(t_k, R_NI, t_NI);
  Eigen::Vector3d a_N;
  Eigen::Matrix3d a_cov;
  if (use([&](auto& g){ return g.antennaN(core, &a_N, &a_cov); })) outputs.writeTrajAntN(t_k, a_N);
  {   // A61 F13: the odometry output (the activation instant's later revision left out)
    Eigen::Matrix3d R_o;
    Eigen::Vector3d t_o, a_o;
    if (use([&](auto& g){ return g.globalPoseOdom(core, &R_o, &t_o); })) outputs.writeTrajNOdom(t_k, R_o, t_o);
    if (use([&](auto& g){ return g.antennaOdom(core, &a_o); })) outputs.writeTrajAntNOdom(t_k, a_o);
  }
  livo_gnss::AnchorFrame anchor;
  if (use([&](auto& g){ return g.anchor(&anchor); }) && anchor.valid) {
    if (!anchor_written || anchor.t_fixed != anchor_t_written) {
      outputs.writeAnchor(anchor);
      anchor_written = true;
      anchor_t_written = anchor.t_fixed;
    }
    outputs.writeTrajGeo(t_k, detail::ecefToGeodetic(anchor.r0_ecef + anchor.R_EN * t_NI));
  }
}

void GnssRosBridge::Impl::publishDiagnostics(double t_k) {
  if (!pubs_ready || t_k - last_diag_t < 1.0) return;   // 1 Hz in estimator time (DESIGN §15)
  last_diag_t = t_k;
  const livo_gnss::FusionStatus st = use([&](auto& g){ return g.status(); });
  diagnostic_msgs::DiagnosticArray arr;
  arr.header.stamp = ros::Time().fromSec(t_k);
  diagnostic_msgs::DiagnosticStatus ds;
  ds.name = "livo_gnss";
  ds.hardware_id = "gnss";
  ds.level = st.livo_alarm ? diagnostic_msgs::DiagnosticStatus::WARN : diagnostic_msgs::DiagnosticStatus::OK;
  ds.message = livo_gnss::phaseName(st.phase);
  auto kv = [&ds](const char* k, const std::string& v) {
    diagnostic_msgs::KeyValue p;
    p.key = k;
    p.value = v;
    ds.values.push_back(p);
  };
  kv("active", st.active ? "true" : "false");
  kv("degenerate", st.degenerate ? "true" : "false");
  kv("schmidt", st.schmidt ? "true" : "false");
  kv("slots", std::to_string(st.n_slots));
  kv("aug_dim", std::to_string(st.aug_dim));
  kv("sats_last", std::to_string(st.n_sats_last));
  kv("epochs_processed", std::to_string(st.epochs_processed));
  kv("epochs_committed", std::to_string(st.epochs_committed));
  kv("epochs_rolled_back", std::to_string(st.epochs_rolled_back));
  kv("sigma_yaw_deg", std::to_string(st.sigma_yaw_rad * 180.0 / M_PI));
  kv("sigma_t_m", std::to_string(st.sigma_t_m));
  kv("sigma_b_m", std::to_string(st.sigma_b_m));
  kv("tdcp_pairs_open", std::to_string(st.tdcp_pairs_open));
  kv("outage_s", std::to_string(st.outage_s));
  arr.status.push_back(ds);
  pub_diag.publish(arr);
}

GnssRosBridge::GnssRosBridge() : impl_(new Impl) {}

GnssRosBridge::~GnssRosBridge() {
  if (impl_ && impl_->own_outputs) impl_->outputs.close();
}

bool GnssRosBridge::loadParams(ros::NodeHandle& nh, std::string* why) {
  Impl& I = *impl_;
  I.cfg = livo_gnss::Config();
  std::vector<std::string> errors;
  XmlRpc::XmlRpcValue tree;
  if (nh.getParam("gnss", tree)) detail::configFromParamTree(tree, &I.cfg, &errors);
  else detail::loadedLivgoConfig() = livo_gnss::LivgoConfig();
  I.livgo_cfg = detail::loadedLivgoConfig();

  // Keys outside gnss: that the glue needs (DESIGN §14 output:/debug:; upstream uav:).
  nh.param<std::string>("output/dir", I.out_opt.dir, "");
  if (I.livgo_cfg.enable && !I.out_opt.dir.empty()) I.livgo_cfg.hessian_log = I.out_opt.dir + "/livgo_hessian.csv";
  if (I.livgo_cfg.enable && I.livgo_cfg.tdcp_gain_log && !I.out_opt.dir.empty())
    I.livgo_cfg.tdcp_gain_log_path = I.out_opt.dir + "/livgo_tdcp_gain.csv";
  nh.param<bool>("output/traj_w", I.out_opt.traj_w, true);
  nh.param<bool>("output/traj_n", I.out_opt.traj_n, true);
  nh.param<bool>("output/state_trace", I.out_opt.state_trace, true);
  nh.param<bool>("output/global_cloud", I.out_opt.global_cloud, false);
  nh.param<bool>("output/timing", I.out_opt.timing, true);
  nh.param<bool>("debug/cov_csv", I.out_opt.cov_csv, true);
  I.out_opt.sat_csv = I.cfg.output.sat_csv;
  nh.param<bool>("uav/gravity_align_en", I.gravity_align_en, false);

  // A config.hpp field without a parameter entry would silently keep its default: refuse to run GNSS then.
  const std::vector<std::string> coverage = detail::loaderCoverageProblems();
  if (!coverage.empty()) {
    if (I.cfg.enable) errors.insert(errors.end(), coverage.begin(), coverage.end());
    else ROS_WARN("[gnss] parameter table out of date (harmless while gnss.enable is false): %s", join(coverage, "; ").c_str());
  }
  for (const std::string& p : livo_gnss::validateConfig(I.cfg)) errors.push_back(p);
  if (I.cfg.enable) {
    // AMENDMENTS A38/A45: GPS Klobuchar and every ephemeris of the obstructed sequences come from the RINEX NAV files.
    if (I.cfg.source == livo_gnss::Config::SourceKind::GnssCommRos1 && I.cfg.nav_rinex.empty())
      errors.push_back("gnss.nav_rinex is empty with source gnss_comm_ros1 (AMENDMENTS A38/A45: the same-day IGS BRDC file is a required input)");
    for (const std::string& p : I.cfg.nav_rinex)
      if (!p.empty() && !fileReadable(p)) errors.push_back("gnss.nav_rinex: cannot read '" + p + "'");
    if (!I.cfg.time.calib_file.empty() && !fileReadable(I.cfg.time.calib_file))
      errors.push_back("gnss.time.calib_file: cannot read '" + I.cfg.time.calib_file + "'");
    if (I.cfg.source == livo_gnss::Config::SourceKind::Rinex) {
      if (!I.cfg.rinex.obs.empty() && !fileReadable(I.cfg.rinex.obs)) errors.push_back("gnss.rinex.obs: cannot read '" + I.cfg.rinex.obs + "'");
      if (!I.cfg.rinex.nav.empty() && !fileReadable(I.cfg.rinex.nav)) errors.push_back("gnss.rinex.nav: cannot read '" + I.cfg.rinex.nav + "'");
    }
    if (I.gravity_align_en)
      errors.push_back("gnss.enable needs uav/gravity_align_en: false (W must stay the IMU frame at initialisation, DESIGN §1)");
  }
  if (!errors.empty()) {
    if (why) *why = join(errors, "; ");
    return false;
  }
  I.loaded = true;
  ROS_INFO("[gnss] enable=%s update_enable=%s mode=%s source=%s nav_rinex=%zu file(s)", I.cfg.enable ? "true" : "false",
           I.cfg.update_enable ? "true" : "false",
           I.cfg.mode == livo_gnss::Config::Mode::Tight ? "tight" : I.cfg.mode == livo_gnss::Config::Mode::LoosePvt ? "loose_pvt" : "livgo_probe*",
           I.cfg.source == livo_gnss::Config::SourceKind::GnssCommRos1 ? "gnss_comm_ros1" : I.cfg.source == livo_gnss::Config::SourceKind::Rinex ? "rinex" : "ubx_stream",
           I.cfg.nav_rinex.size());
  return true;
}

const livo_gnss::Config& GnssRosBridge::config() const { return impl_->cfg; }

bool GnssRosBridge::enabled() const { return impl_->cfg.enable && !impl_->failed; }

bool GnssRosBridge::configure(const M3D& extR, const V3D& extT, SLAM_MODE mode, const ImuProcessPtr& imu, std::string* why) {
  Impl& I = *impl_;
  I.imu = imu;
  if (!I.loaded) {
    if (why) *why = "configure() before a successful loadParams()";
    return false;
  }
  I.configured = true;
  if (!I.cfg.enable) return true;   // zero channel: no core, the ImuProcess hooks stay off
  if (!imu) {
    if (why) *why = "no ImuProcess";
    return false;
  }
  if (mode == ONLY_LO) {
    if (why) *why = "gnss.enable needs the IMU: ONLY_LO has no bridge (DESIGN §9)";
    return false;
  }
  try {
    if (I.livgo_cfg.enable) I.fusion_livgo.reset(new livo_gnss::GnssFusionLivgo(I.cfg, I.livgo_cfg));
    else I.fusion.reset(new livo_gnss::GnssFusion(I.cfg));   // loads cfg.nav_rinex (gnss_fusion.hpp)
    I.use([&](auto& g){ return g.setExtrinsics(extR, extT); });                // T_{I<-L}: l^I = R_IL l^L + t_IL for lever_arm_frame lidar
    if (!I.use([&](auto& g){ return g.setSlamMode(toSlamMode(mode)); })) {
      if (why) *why = "the core refused the SLAM mode";
      I.dropCore();
      return false;
    }
    I.use([&](auto& g){ return g.setHostFlags(imu->gravity_estimation_enabled(), imu->exposure_estimation_enabled()); });
    I.use([&](auto& g){ return g.setDiagnosticsSink(&I.outputs); });

    // GLONASS FDMA channels for the sat2freq check from the NAV files the core loads (bag glo_ephem adds more).
    for (const std::string& p : I.cfg.nav_rinex) {
      livo_gnss_io::RinexSource rs;
      std::string w;
      if (!rs.open("", p, livo_gnss::GTime(), livo_gnss::GTime(), &w)) {
        if (why) *why = "gnss.nav_rinex '" + p + "': " + w;
        I.dropCore();
        return false;
      }
      for (const livo_gnss::GloEph& g : rs.gloEphemerides()) I.sat2freq.setGloChannel(g.sat.rtk_sat, g.frq);
      ROS_INFO("[gnss] nav_rinex %s: %zu Kepler + %zu GLONASS ephemerides", p.c_str(), rs.ephemerides().size(), rs.gloEphemerides().size());
    }

    using SK = livo_gnss::Config::SourceKind;
    auto topic = [&I](const std::string& t, livo_gnss::Source s) {
      if (!t.empty()) I.topic_src[t] = s;
    };
    switch (I.cfg.source) {
      case SK::GnssCommRos1:
        I.comm.reset(new livo_gnss_io::GnssCommRos1Decoder(&livo_gnss::satNoOf));
        I.decoder.reset(new detail::CheckedDecoder(I.comm.get(), &I.sat2freq));
        topic(I.cfg.topics.meas, livo_gnss::Source::GnssCommMeas);
        topic(I.cfg.topics.ephem, livo_gnss::Source::GnssCommEphem);
        topic(I.cfg.topics.glo_ephem, livo_gnss::Source::GnssCommGloEphem);
        topic(I.cfg.topics.iono, livo_gnss::Source::GnssCommIono);
        topic(I.cfg.topics.pvt, livo_gnss::Source::GnssCommPvt);
        topic(I.cfg.topics.tp, livo_gnss::Source::GnssCommTimePulse);   // counted only (A19)
        break;
      case SK::UbxStream:
        // The raw stream topic (GnssRawStream) is configured as topics.meas; the other gnss_comm topics do not exist.
        I.ubx.reset(new livo_gnss_io::UbxStream());
        I.decoder.reset(new detail::CheckedDecoder(I.ubx.get(), &I.sat2freq));
        topic(I.cfg.topics.meas, livo_gnss::Source::UbxStream);
        break;
      case SK::Rinex: {
        // Pull source (§12.4): every OBS epoch is handed to the core now; the core schedules it at t_est + rinex.latency_s.
        livo_gnss_io::RinexSource rs;
        std::string w;
        if (!rs.open(I.cfg.rinex.obs, I.cfg.rinex.nav, livo_gnss::GTime(), livo_gnss::GTime(), &w)) {
          if (why) *why = "gnss.rinex: " + w;
          I.dropCore();
          return false;
        }
        const bool nav_listed = std::find(I.cfg.nav_rinex.begin(), I.cfg.nav_rinex.end(), I.cfg.rinex.nav) != I.cfg.nav_rinex.end();
        if (!I.cfg.rinex.nav.empty() && !nav_listed) {
          std::string w2;
          if (I.use([&](auto& g){ return g.loadNavRinex(I.cfg.rinex.nav, &w2); }) <= 0) {
            if (why) *why = "gnss.rinex.nav '" + I.cfg.rinex.nav + "': no ephemeris loaded" + (w2.empty() ? "" : ": " + w2);
            I.dropCore();
            return false;
          }
        }
        for (const livo_gnss::GloEph& g : rs.gloEphemerides()) I.sat2freq.setGloChannel(g.sat.rtk_sat, g.frq);
        size_t n = 0;
        livo_gnss::ObsEpoch e;
        while (rs.nextEpoch(&e)) {
          I.sat2freq.onObservation(e);
          I.use([&](auto& g){ return g.pushObservation(std::move(e)); });
          e = livo_gnss::ObsEpoch();
          ++n;
        }
        ROS_INFO("[gnss] source rinex: %zu epochs from %s", n, I.cfg.rinex.obs.c_str());
        break;
      }
    }
    if (I.decoder) I.use([&](auto& g){ return g.setRawDecoder(I.decoder.get()); });
  } catch (const std::exception& e) {
    if (why) *why = std::string("GNSS core: ") + e.what();
    I.dropCore();
    return false;
  }
  imu->enable_gnss_hooks(true);
  return true;
}

void GnssRosBridge::setupRos(ros::NodeHandle& nh, bool replay_mode) {
  Impl& I = *impl_;
  I.replay_mode = replay_mode;
  if (!I.active()) return;
  // Live node: the bridge owns the writer. Replay: fastlivo_replay opens this same writer (outputs()) and writes the
  // host files itself, so opening it here would open the files twice.
  if (!replay_mode && !I.out_opt.dir.empty()) {
    OutputOptions live = I.out_opt;
    live.state_trace = false;   // one line per processOnce() is a replay artefact (DESIGN §13); timing likewise
    live.timing = false;
    std::string w;
    if (I.outputs.open(live, &w)) I.own_outputs = true;
    else ROS_ERROR("[gnss] output/dir: %s", w.c_str());
  }
  I.pub_odom = nh.advertise<nav_msgs::Odometry>("/gnss/global_odometry", 100);
  I.pub_odom_smooth = nh.advertise<nav_msgs::Odometry>("/gnss/global_odometry_smooth", 100);
  I.pub_fused_fix = nh.advertise<sensor_msgs::NavSatFix>("/gnss/fused_fix", 100);
  I.pub_spp_fix = nh.advertise<sensor_msgs::NavSatFix>("/gnss/spp_fix", 100);
  I.pub_diag = nh.advertise<diagnostic_msgs::DiagnosticArray>("/gnss/diagnostics", 10);
  I.tf_br.reset(new tf::TransformBroadcaster());
  I.pubs_ready = true;
  if (replay_mode) return;   // fastlivo_replay hands every bag message to handleReplayMessage()
  for (const auto& kv : I.topic_src) {
    const std::string topic = kv.first;
    ros::SubscribeOptions so;
    so.initByFullCallbackType<const ros::MessageEvent<topic_tools::ShapeShifter const>&>(
        topic, 10000, [this, topic](const ros::MessageEvent<topic_tools::ShapeShifter const>& ev) {
          handleLiveMessage(topic, ev.getConstMessage(), ev.getReceiptTime().toSec());
        });
    I.subs.push_back(nh.subscribe(so));
  }
}

void GnssRosBridge::handleReplayMessage(const rosbag::MessageInstance& m) {
  Impl& I = *impl_;
  if (!I.active()) return;
  const auto it = I.topic_src.find(m.getTopic());
  if (it == I.topic_src.end()) return;
  std::vector<uint8_t> bytes;
  if (!detail::messageBytes(m, &bytes)) {
    I.channel_stats[it->second].byte_errors++;
    return;
  }
  I.ingest(it->second, m.getDataType(), m.getMD5Sum(), m.getMessageDefinition(), bytes, m.getTime().toSec());
}

void GnssRosBridge::handleLiveMessage(const std::string& topic, const topic_tools::ShapeShifter::ConstPtr& msg, double t_record) {
  Impl& I = *impl_;
  if (!I.active() || !msg) return;
  const auto it = I.topic_src.find(topic);
  if (it == I.topic_src.end()) return;
  std::vector<uint8_t> bytes;
  if (!detail::messageBytes(*msg, &bytes)) {
    I.channel_stats[it->second].byte_errors++;
    return;
  }
  I.ingest(it->second, msg->getDataType(), msg->getMD5Sum(), msg->getMessageDefinition(), bytes, t_record);
}

void GnssRosBridge::onLioStep(StatesGroup& state, const ImuProcess& imu, double t_k) {
  Impl& I = *impl_;
  if (!I.active()) return;
  I.last_t_k = t_k;
  if (imu.gnss_bridge_orphan_steps() != 0) {
    I.failSafe("onLioStep", "IMU propagation outside a LIO cycle (a VIO cycle moved the state): the H1 chain F_tot is incomplete");
    return;
  }
  std::vector<livo_gnss::BridgeSample> samples;
  livo_gnss::HostMat F_tot;
  double t_prev = 0.0, t_end = 0.0;
  if (!takeBridge(imu, &samples, &F_tot, &t_prev, &t_end)) {
    I.n_no_bridge++;   // first propagated cycle after IMU initialisation
    return;
  }
  if (std::fabs(t_end - t_k) > 1e-9) {
    I.failSafe("onLioStep", "bridge end differs from the cycle end t_k");
    return;
  }
  livo_gnss::CoreState core = toCore(state);
  // The bridge was propagated with the host's b_g, b_a, g, which UndistortPcl never changes (gravity alignment is refused).
  if (!I.snapshot_warned && !(sameBits(imu.gnss_bridge_bg_snap(), core.bg) && sameBits(imu.gnss_bridge_ba_snap(), core.ba) &&
                              sameBits(imu.gnss_bridge_g_snap(), core.g))) {
    I.snapshot_warned = true;
    ROS_WARN("[gnss] bridge snapshots differ from the host state at the hook (b_g/b_a/g changed during propagation)");
  }
  try {
    const livo_gnss::StepReport r = I.use([&](auto& g){ return g.onLioStep(core, F_tot, samples, t_prev, t_k); });
    I.n_lio_steps++;
    if (r.core_modified) {
      if (!I.cfg.update_enable) {
        I.failSafe("onLioStep", "the core reported a host update with update_enable = false (OBSERVE must never write the host)");
        return;
      }
      fromCore(core, &state);
      I.n_commits++;
    }
  } catch (const std::exception& e) {
    I.failSafe("onLioStep", e.what());
  }
}

bool GnssRosBridge::livgoEnabled() const { return impl_->loaded && impl_->livgo_cfg.enable; }

bool GnssRosBridge::livgoLambdaLioOnly() const {
  return impl_->loaded && impl_->livgo_cfg.enable && impl_->livgo_cfg.lambda_source == livo_gnss::LivgoConfig::LambdaSource::Lio;
}

void GnssRosBridge::setLivgoLambdaMin(double lambda_min) {
  Impl& I = *impl_;
  if (!I.active() || !I.livgo_cfg.enable || !I.fusion_livgo) return;
  try {
    I.fusion_livgo->setLivgoLambdaMin(lambda_min);
  } catch (const std::exception& e) {
    I.failSafe("setLivgoLambdaMin", e.what());
  }
}

void GnssRosBridge::onCoreUpdated(const StatesGroup& prior, const StatesGroup& post, livo_gnss::UpdateSource src, int effct_feat_num) {
  Impl& I = *impl_;
  if (!I.active()) return;
  try {
    I.use([&](auto& g){ return g.onCoreUpdated(toCore(prior), toCore(post), src); });
  } catch (const std::exception& e) {
    I.failSafe("onCoreUpdated", e.what());
    return;
  }
  if (I.own_outputs && I.out_opt.cov_csv) {   // live node; in replay filter_cov.csv is written by the driver (A13)
    if (src == livo_gnss::UpdateSource::LIO) {
      writeCov(prior, "prior", I.last_t_k, effct_feat_num);
    } else {
      I.last_prior = prior;
      I.have_last_prior = true;
    }
    writeCov(post, src == livo_gnss::UpdateSource::LIO ? "postLIO" : "postVIO", I.last_t_k, effct_feat_num);
  }
}

void GnssRosBridge::onPostVio(StatesGroup& state, double t_k) {
  Impl& I = *impl_;
  if (!I.active()) return;
  try {
    livo_gnss::CoreState core = toCore(state);
    const livo_gnss::StepReport r = I.use([&](auto& g){ return g.onPostVio(core, t_k); });   // pre_lio: empty report (A12)
    if (r.core_modified) {
      if (!I.cfg.update_enable) {
        I.failSafe("onPostVio", "the core reported a host update with update_enable = false (OBSERVE must never write the host)");
        return;
      }
      fromCore(core, &state);
      I.n_post_vio_commits++;
    }
  } catch (const std::exception& e) {
    I.failSafe("onPostVio", e.what());
  }
}

void GnssRosBridge::onStepEnd(const StatesGroup& state, double t_k, int cycle_idx) {
  (void)cycle_idx;
  Impl& I = *impl_;
  if (!I.active()) return;
  I.last_t_k = t_k;
  try {
    const livo_gnss::CoreState core = toCore(state);
    I.use([&](auto& g){ return g.onStepEnd(core, t_k); });
    I.writeStepOutputs(core, state, t_k);
    I.publishDiagnostics(t_k);
  } catch (const std::exception& e) {
    I.failSafe("onStepEnd", e.what());
  }
}

void GnssRosBridge::publishGlobal(const StatesGroup& state, double t_k) {
  Impl& I = *impl_;
  if (!I.active() || !I.pubs_ready) return;
  try {
    const livo_gnss::CoreState core = toCore(state);
    const ros::Time stamp = ros::Time().fromSec(t_k);   // topics carry the sensor time t_k (DESIGN §15)
    const std::string& frame = I.cfg.output.frame_id;
    Eigen::Matrix3d R_NI;
    Eigen::Vector3d t_NI;
    Eigen::Matrix<double, 6, 6> cov;
    if (!I.use([&](auto& g){ return g.globalPose(core, &R_NI, &t_NI, &cov); })) return;   // before ACTIVATE there is no N frame
    auto odom = [&](const Eigen::Matrix3d& R, const Eigen::Vector3d& t, const Eigen::Matrix<double, 6, 6>* c) {
      nav_msgs::Odometry o;
      o.header.stamp = stamp;
      o.header.frame_id = frame;
      o.child_frame_id = "aft_mapped";
      const Eigen::Quaterniond q(R);
      o.pose.pose.position.x = t.x();
      o.pose.pose.position.y = t.y();
      o.pose.pose.position.z = t.z();
      o.pose.pose.orientation.w = q.w();
      o.pose.pose.orientation.x = q.x();
      o.pose.pose.orientation.y = q.y();
      o.pose.pose.orientation.z = q.z();
      if (c) rosPoseCovariance(*c, &o.pose.covariance);
      const Eigen::Vector3d v_body = state.rot_end.transpose() * state.vel_end;   // twist in the child (body) frame
      o.twist.twist.linear.x = v_body.x();
      o.twist.twist.linear.y = v_body.y();
      o.twist.twist.linear.z = v_body.z();
      return o;
    };
    I.pub_odom.publish(odom(R_NI, t_NI, &cov));
    Eigen::Matrix3d R_s;
    Eigen::Vector3d t_s;
    if (I.use([&](auto& g){ return g.globalPoseSmooth(&R_s, &t_s); })) I.pub_odom_smooth.publish(odom(R_s, t_s, nullptr));

    livo_gnss::AnchorFrame anchor;
    const bool have_anchor = I.use([&](auto& g){ return g.anchor(&anchor); }) && anchor.valid;
    auto fix = [&](const Eigen::Vector3d& r_ecef, const Eigen::Matrix3d& cov_enu_local, uint8_t cov_type) {
      sensor_msgs::NavSatFix f;
      f.header.stamp = stamp;
      f.header.frame_id = frame;
      f.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
      f.status.service = (I.cfg.systems.gps ? sensor_msgs::NavSatStatus::SERVICE_GPS : 0) |
                         (I.cfg.systems.glo ? sensor_msgs::NavSatStatus::SERVICE_GLONASS : 0) |
                         (I.cfg.systems.bds ? sensor_msgs::NavSatStatus::SERVICE_COMPASS : 0) |
                         (I.cfg.systems.gal ? sensor_msgs::NavSatStatus::SERVICE_GALILEO : 0);
      const livo_gnss::Geodetic g = detail::ecefToGeodetic(r_ecef);
      f.latitude = g.lat_rad * 180.0 / M_PI;
      f.longitude = g.lon_rad * 180.0 / M_PI;
      f.altitude = g.h_m;   // WGS-84 ellipsoidal height
      for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) f.position_covariance[static_cast<size_t>(3 * i + j)] = cov_enu_local(i, j);
      f.position_covariance_type = cov_type;
      return f;
    };
    Eigen::Vector3d a_N;
    Eigen::Matrix3d a_cov;
    if (have_anchor && I.use([&](auto& g){ return g.antennaN(core, &a_N, &a_cov); })) {
      const Eigen::Vector3d a_E = anchor.r0_ecef + anchor.R_EN * a_N;
      const livo_gnss::Geodetic g = detail::ecefToGeodetic(a_E);
      const Eigen::Matrix3d R_loc = detail::enuToEcefRotation(g.lat_rad, g.lon_rad).transpose() * anchor.R_EN;   // N → local ENU
      I.pub_fused_fix.publish(fix(a_E, R_loc * a_cov * R_loc.transpose(), sensor_msgs::NavSatFix::COVARIANCE_TYPE_KNOWN));
    }
    Eigen::Vector3d r_spp;
    Eigen::Matrix3d c_spp;
    livo_gnss::GTime t_spp;
    if (I.use([&](auto& g){ return g.lastSpp(&r_spp, &c_spp, &t_spp); }) && t_spp != I.last_spp_t) {
      I.last_spp_t = t_spp;
      const livo_gnss::Geodetic g = detail::ecefToGeodetic(r_spp);
      const Eigen::Matrix3d R_EL = detail::enuToEcefRotation(g.lat_rad, g.lon_rad);
      I.pub_spp_fix.publish(fix(r_spp, R_EL.transpose() * c_spp * R_EL, sensor_msgs::NavSatFix::COVARIANCE_TYPE_KNOWN));
    }
    // TF enu → camera_init = T_{N<-W}: p^N = M (p^W − o^W) + t.
    livo_gnss::Registration xi;
    if (I.tf_br && I.use([&](auto& g){ return g.registration(&xi); }) && xi.active) {
      const Eigen::Vector3d t_NW = xi.trans - xi.M * xi.pivot_w;
      const Eigen::Quaterniond q(xi.M);
      tf::Transform T;
      T.setOrigin(tf::Vector3(t_NW.x(), t_NW.y(), t_NW.z()));
      T.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
      I.tf_br->sendTransform(tf::StampedTransform(T, stamp, frame, "camera_init"));
    }
  } catch (const std::exception& e) {
    I.failSafe("publishGlobal", e.what());
  }
}

void GnssRosBridge::writeCov(const StatesGroup& state, const char* stage, double t_k, int effct_feat_num) {
  Impl& I = *impl_;
  if (!I.outputs.isOpen()) return;
  const bool is_prior = std::string(stage) == "prior";
  if (is_prior) {
    I.last_prior = state;
    I.have_last_prior = true;
  }
  const StatesGroup& prior = (is_prior || !I.have_last_prior) ? state : I.last_prior;
  try {
    I.outputs.onCov(I.covRecord(state, prior, stage, t_k, effct_feat_num));
  } catch (const std::exception& e) {
    I.failSafe("writeCov", e.what());
  }
}

void GnssRosBridge::finish() {
  Impl& I = *impl_;
  if (I.finished) return;
  I.finished = true;
  if (!I.cfg.enable || !I.configured) return;
  if ((I.fusion || I.fusion_livgo) && !I.failed) {
    try {
      I.use([&](auto& g){ return g.finish(); });   // summary.json through the diagnostics sink
    } catch (const std::exception& e) {
      I.failSafe("finish", e.what());
    }
  }
  for (const auto& kv : I.channel_stats)
    ROS_INFO("[gnss] channel %s: %llu messages, %llu bytes, %llu dropped, %llu byte errors, definition %s", livo_gnss::sourceName(kv.first),
             static_cast<unsigned long long>(kv.second.msgs), static_cast<unsigned long long>(kv.second.bytes),
             static_cast<unsigned long long>(kv.second.dropped), static_cast<unsigned long long>(kv.second.byte_errors),
             I.channel_state[kv.first] > 0 ? "registered" : I.channel_state[kv.first] < 0 ? "REJECTED" : "never seen");
  const detail::Sat2FreqCheck::Report r = I.sat2freq.report();
  ROS_INFO("[gnss] sat2freq check (A35-C8): %llu (sat,code) pairs, %llu consistent, %llu mismatched, %llu GLONASS pending without an "
           "ephemeris channel, max |diff| %.3f Hz",
           static_cast<unsigned long long>(r.pairs), static_cast<unsigned long long>(r.consistent), static_cast<unsigned long long>(r.mismatched),
           static_cast<unsigned long long>(r.pending), r.max_abs_diff_hz);
  for (const std::string& m : r.mismatches) {
    ROS_WARN("[gnss] sat2freq mismatch: %s", m.c_str());
    livo_gnss::Event ev;
    ev.kind = livo_gnss::EventKind::CONFIG_WARNING;
    ev.t_k = I.last_t_k;
    ev.detail = "SAT2FREQ_MISMATCH " + m;
    I.outputs.onEvent(ev);
  }
  ROS_INFO("[gnss] hooks: %llu LIO steps, %llu without bridge, %llu host commits (+%llu post_vio)%s%s",
           static_cast<unsigned long long>(I.n_lio_steps), static_cast<unsigned long long>(I.n_no_bridge),
           static_cast<unsigned long long>(I.n_commits), static_cast<unsigned long long>(I.n_post_vio_commits), I.failed ? "; FAILED: " : "",
           I.failed ? I.failure.c_str() : "");
  if (I.own_outputs) {
    I.outputs.flush();
    I.outputs.close();
    I.own_outputs = false;
  }
}

livo_gnss::GnssFusion* GnssRosBridge::fusion() { return impl_->active() ? impl_->fusion.get() : nullptr; }

OutputWriter* GnssRosBridge::outputs() { return &impl_->outputs; }

}  // namespace livo_gnss_ros
