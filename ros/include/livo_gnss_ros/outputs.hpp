// SPDX-License-Identifier: GPL-2.0-only
// OutputWriter — every file of DESIGN §15 / §13: state_trace.txt (hexfloat, one line per processOnce),
// traj_W.tum / traj_N.tum / traj_ant_N.tum (%.9f, t_k), traj_N_geo.csv, anchor.yaml, gnss_epochs.csv,
// gnss_sats.csv (optional), filter_cov.csv, events.log, timing.csv, run_manifest.json, summary.json.
// Implements livo_gnss::DiagnosticsSink so the core never touches files. GNSS-off runs still write
// state_trace/traj_W (zero-channel gate). Who calls it (AMENDMENTS A13, M8): in ref-R and in fastlivo_replay
// every call happens in src/replay_main.cpp after each processOnce() — LIVMapper's members are public, so
// state_propagat (prior), _state (posterior), lio_vio_flg and effct_feat_num are read there; LIVMapper never
// links or includes this library. Owner: WP-f (outputs.cpp).
#pragma once

#include <cstdio>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "livo_gnss/core_state.hpp"
#include "livo_gnss/diagnostics.hpp"
#include "livo_gnss/types.hpp"

namespace livo_gnss_ros {

struct OutputOptions {
  std::string dir;                  // output.dir ("" = disabled)
  bool traj_w = true, traj_n = true, state_trace = true, global_cloud = false;
  bool sat_csv = false;             // gnss.output.sat_csv
  bool cov_csv = true;              // debug.cov_csv
  bool timing = true;
};

struct ManifestInfo {
  std::string git_describe, git_dirty, profile, compile_defs, eigen_version, compiler, image_digest;
  std::string config_sha256, config_dump, bag_path, bag_sha256, arm, host;
  int seed = 0;
  double start_offset_s = 0.0;
  double t_start_wall = 0.0, t_end_wall = 0.0;
  // Empty, or the text of one JSON object whose members writeManifest merges verbatim into the top level of
  // run_manifest.json (replay_stats fields of fastlivo_replay). (Appended P2b, INTERFACE_CHANGES WP-f.)
  std::string extra_json;
};

class OutputWriter final : public livo_gnss::DiagnosticsSink {
 public:
  OutputWriter();
  ~OutputWriter() override;
  bool open(const OutputOptions& opt, std::string* why);
  bool isOpen() const;

  // state_trace.txt: cycle, flag, t_k, R(9), p, v, b_g, b_a, g, τ, diag(P)(19), then x_G and diag(Q_G)
  // (all %a hexfloat; DESIGN §13). `x_G` / `diag_Q_G` may be empty.
  void writeStateTrace(int cycle, int flag, double t_k, const livo_gnss::CoreState& core, const Eigen::VectorXd& x_G,
                       const Eigen::VectorXd& diag_Q_G);
  // Trajectories (TUM: t x y z qx qy qz qw, %.9f).
  void writeTrajW(double t_k, const Eigen::Matrix3d& R_WI, const Eigen::Vector3d& p_W);
  void writeTrajN(double t_k, const Eigen::Matrix3d& R_NI, const Eigen::Vector3d& p_N);
  void writeTrajAntN(double t_k, const Eigen::Vector3d& a_N);
  // Appended (AMENDMENTS A61 F13): traj_N_odom.tum / traj_ant_N_odom.tum, GnssFusion::globalPoseOdom / antennaOdom.
  void writeTrajNOdom(double t_k, const Eigen::Matrix3d& R_NI, const Eigen::Vector3d& p_N);
  void writeTrajAntNOdom(double t_k, const Eigen::Vector3d& a_N);
  void writeTrajGeo(double t_k, const livo_gnss::Geodetic& g);
  void writeAnchor(const livo_gnss::AnchorFrame& a);
  void writeTiming(int cycle, double t_k, double lio_ms, double vio_ms, double gnss_ms);
  void writeManifest(const ManifestInfo& m);

  // DiagnosticsSink
  void onEpoch(const livo_gnss::EpochRecord& r) override;
  void onSat(const livo_gnss::SatRecord& r) override;
  void onCov(const livo_gnss::CovRecord& r) override;
  void onEvent(const livo_gnss::Event& e) override;
  void onSummary(const livo_gnss::Summary& s) override;

  void flush();
  void close();

 private:
  struct Impl;
  Impl* impl_;
};

}  // namespace livo_gnss_ros
