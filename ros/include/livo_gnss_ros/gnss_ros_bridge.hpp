// SPDX-License-Identifier: GPL-2.0-only
// GnssRosBridge — ROS1 glue between LIVMapper and livo_gnss::GnssFusion (DESIGN §12.2, §9, §15;
// IMPL_PLAN 7): parameter loading (gnss: tree → Config), live subscriptions via topic_tools::ShapeShifter
// and replay dispatch via rosbag::MessageInstance (both end in GnssFusion::feedBytes), the hook calls at
// the five LIVMapper sites, published topics/TF and the output writers. ROS2 port rewrites only this layer.
// Owner: P3 (gnss_ros_bridge.cpp); outputs.cpp belongs to WP-f.
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <rosbag/message_instance.h>
#include <topic_tools/shape_shifter.h>
#include <tf/transform_broadcaster.h>

#include "common_lib.h"
#include "IMU_Processing.h"
#include "livo_gnss/config.hpp"
#include "livo_gnss/gnss_fusion.hpp"
#include "livo_gnss_ros/outputs.hpp"

namespace livo_gnss_io {
class GnssCommRos1Decoder;
class UbxStream;
class RinexSource;
}  // namespace livo_gnss_io

namespace livo_gnss_ros {

class GnssRosBridge {
 public:
  GnssRosBridge();
  ~GnssRosBridge();

  // readParameters (LIVMapper.cpp:116 after): reads `gnss:`, `output:`, `publish:` (DESIGN §14).
  // Returns false when the config is rejected (validateConfig) — the node must not start.
  bool loadParams(ros::NodeHandle& nh, std::string* why);
  const livo_gnss::Config& config() const;
  bool enabled() const;                 // gnss.enable

  // initializeComponents (:162 after): extrinsics T_{I<-L}, slam mode, host flags, IMU noise.
  bool configure(const M3D& extR, const V3D& extT, SLAM_MODE mode, const ImuProcessPtr& imu, std::string* why);

  // initializeSubscribersAndPublishers (:216 after): six ShapeShifter subscriptions (live) or none
  // (replay), publishers for /gnss/* and the TF broadcaster.
  void setupRos(ros::NodeHandle& nh, bool replay_mode);

  // Byte sources (converge on GnssFusion::feedBytes, DESIGN §12.2). Replay: called by fastlivo_replay
  // for every GNSS-topic message; live: ShapeShifter callback. Both register the message definition
  // on first contact (rejection → event MSGDEF_REJECT and the topic is ignored).
  void handleReplayMessage(const rosbag::MessageInstance& m);
  void handleLiveMessage(const std::string& topic, const topic_tools::ShapeShifter::ConstPtr& msg, double t_record);

  // ---- LIVMapper hook sites (DESIGN §9) ----
  // processImu, between :254 and :256, LIO cycles only and only after IMU init. Converts _state ↔ CoreState,
  // pulls the bridge from ImuProcess, calls GnssFusion::onLioStep and writes the core back when modified.
  void onLioStep(StatesGroup& state, const ImuProcess& imu, double t_k);
  // handleLIO :371 after / handleVIO :305 after.
  void onCoreUpdated(const StatesGroup& prior, const StatesGroup& post, livo_gnss::UpdateSource src, int effct_feat_num);
  // FAST-LIVGO. livgoEnabled is true after loadParams when gnss.livgo.enable is set, even before the fusion exists.
  bool livgoEnabled() const;
  // A75 livgo.lambda_source = lio: the host exports only the LiDAR translation block's λ_min (src/LIVMapper.cpp); false = the
  // weaker of the LIO and VIO blocks (the arm as before A75).
  bool livgoLambdaLioOnly() const;
  void setLivgoLambdaMin(double lambda_min);
  // update_slot = post_vio (M4/A12): after H2 of the cycle's last host update and before onStepEnd();
  // forwards to GnssFusion::onPostVio and writes the core back when modified. No-op for pre_lio.
  void onPostVio(StatesGroup& state, double t_k);
  // stateEstimationAndMapping after the switch (LIVO: VIO cycle; ONLY_LIO: LIO cycle).
  void onStepEnd(const StatesGroup& state, double t_k, int cycle_idx);
  // handleLIO :409 after: /gnss/global_odometry(_smooth), /gnss/fused_fix, TF enu→camera_init.
  void publishGlobal(const StatesGroup& state, double t_k);
  // filter_cov.csv rows at prior/postLIO/postVIO/postGNSS. In the ref-R build every writer call is made
  // from src/replay_main.cpp after processOnce() (A13); this entry point serves the live node.
  void writeCov(const StatesGroup& state, const char* stage, double t_k, int effct_feat_num);
  // run :553 before / replay end: flush files, summary.json, run_manifest.json.
  void finish();

  livo_gnss::GnssFusion* fusion();      // nullptr when gnss.enable = false
  OutputWriter* outputs();

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace livo_gnss_ros
