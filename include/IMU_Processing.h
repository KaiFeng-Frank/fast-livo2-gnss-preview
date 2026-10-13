// Modified for FAST-LIVO2-GNSS, September–October 2026; see THIRD_PARTY_NOTICES.md.
/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef IMU_PROCESSING_H
#define IMU_PROCESSING_H

#include <Eigen/Eigen>
#include "common_lib.h"
#include <condition_variable>
#include <nav_msgs/Odometry.h>
#include <utils/so3_math.h>
#include <fstream>
const bool time_list(PointType &x, PointType &y) { return (x.curvature < y.curvature); }

/// *************IMU Process and undistortion
class ImuProcess
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImuProcess();
  ~ImuProcess();

  void Reset();
  void Reset(double start_timestamp, const sensor_msgs::ImuConstPtr &lastimu);
  void set_extrinsic(const V3D &transl, const M3D &rot);
  void set_extrinsic(const V3D &transl);
  void set_extrinsic(const MD(4, 4) & T);
  void set_gyr_cov_scale(const V3D &scaler);
  void set_acc_cov_scale(const V3D &scaler);
  void set_gyr_bias_cov(const V3D &b_g);
  void set_acc_bias_cov(const V3D &b_a);
  void set_inv_expo_cov(const double &inv_expo);
  void set_imu_init_frame_num(const int &num);
  // BMI088 turn-on prior on b_a and g. Default off: the covariance written by
  // StatesGroup is left exactly as upstream.
  void set_turnon_prior(bool en);
  void disable_imu();
  void disable_gravity_est();
  void disable_bias_est();
  void disable_exposure_est();
  void Process2(LidarMeasureGroup &lidar_meas, StatesGroup &stat, PointCloudXYZI::Ptr cur_pcl_un_);
  void UndistortPcl(LidarMeasureGroup &lidar_meas, StatesGroup &state_inout, PointCloudXYZI &pcl_out);

  ofstream fout_imu;
  double IMU_mean_acc_norm;
  V3D unbiased_gyr;

  V3D cov_acc;
  V3D cov_gyr;
  V3D cov_bias_gyr;
  V3D cov_bias_acc;
  double cov_inv_expo;
  double first_lidar_time = 0.0;
  bool imu_time_init = false;
  bool imu_need_init = true;
  M3D Eye3d;
  V3D Zero3d;
  int lidar_type;

private:
  void IMU_init(const MeasureGroup &meas, StatesGroup &state, int &N);
  void Forward_without_imu(LidarMeasureGroup &meas, StatesGroup &state_inout, PointCloudXYZI &pcl_out);
  PointCloudXYZI pcl_wait_proc;
  sensor_msgs::ImuConstPtr last_imu;
  PointCloudXYZI::Ptr cur_pcl_un_;
  vector<Pose6D> IMUpose;
  M3D Lid_rot_to_IMU;
  V3D Lid_offset_to_IMU;
  V3D mean_acc;
  V3D mean_gyr;
  V3D angvel_last;
  V3D acc_s_last;
  double last_prop_end_time = 0.0;
  double time_last_scan = 0.0;
  int init_iter_num = 1, MAX_INI_COUNT = 20;
  bool b_first_frame = true;
  bool imu_en = true;
  bool gravity_est_en = true;
  bool turnon_prior_en_ = false;
  bool ba_bg_est_en = true;
  bool exposure_estimate_en = true;

  // ---- GNSS tight-coupling hooks (P3; IMPL_PLAN 7, DESIGN §4.1/§9, AMENDMENTS A10). Inert until the GNSS glue calls
  // enable_gnss_hooks(true) (gnss.enable): UndistortPcl then also records the IMU bridge samples of every LIO cycle
  // and the product of its per-step transitions F_x (H1). The hook statements only read values the upstream code has
  // already computed and write only the members below; with hooks disabled they are skipped (zero-channel gate).
public:
  struct BridgeSample
  {
    double t;    // estimator time of the sample
    V3D a_world; // a = R f + g over the interval ending at t (acc_imu, :415); start sample: acc_s_last
    V3D omega;   // bias-corrected angular rate over that interval (angvel_avr, :352); start sample: angvel_last
    M3D R;       // R_{W<-I} at t
    V3D p, v;    // p^W, v^W at t
    V3D f_body;  // bias-corrected, gravity-scaled specific force in I (acc_avr, :353); start sample: R^T (a_world - g)
  };
  void enable_gnss_hooks(bool en) { hooks_en_ = en; }
  bool gnss_hooks_enabled() const { return hooks_en_; }
  // The last UndistortPcl call was a LIO cycle (bridge_t0_, bridge_t1_] and recorded it; false after a VIO cycle.
  bool gnss_bridge_valid() const { return bridge_valid_; }
  const std::vector<BridgeSample> &gnss_bridge_samples() const { return bridge_; }   // [0] = cycle start
  const MD(DIM_STATE, DIM_STATE) &gnss_bridge_F() const { return F_accum_; }          // F_n ... F_1 of that cycle
  double gnss_bridge_t0() const { return bridge_t0_; }
  double gnss_bridge_t1() const { return bridge_t1_; }
  const V3D &gnss_bridge_bg_snap() const { return bridge_bg_snap_; }   // b_g, b_a, g the cycle was propagated with
  const V3D &gnss_bridge_ba_snap() const { return bridge_ba_snap_; }
  const V3D &gnss_bridge_g_snap() const { return bridge_g_snap_; }
  // IMU steps propagated outside a recorded LIO cycle (a VIO cycle that moved the state; never happens upstream,
  // sync_packages gives VIO cycles no IMU): the glue treats a non-zero count as a broken H1 chain.
  long gnss_bridge_orphan_steps() const { return bridge_orphan_steps_; }
  bool gravity_estimation_enabled() const { return gravity_est_en; }
  bool exposure_estimation_enabled() const { return exposure_estimate_en; }

private:
  // Out of line and never inlined, so the hook arithmetic cannot be interleaved with UndistortPcl's own code.
  __attribute__((noinline)) void bridgeBegin(double t, const StatesGroup &state);
  __attribute__((noinline)) void accumulateF(const MD(DIM_STATE, DIM_STATE) &F_x);
  __attribute__((noinline)) void bridgeSample(double t, const M3D &R, const V3D &p, const V3D &v, const V3D &omega, const V3D &a_world,
                                              const V3D &f_body);
  __attribute__((noinline)) void bridgeEnd(double t1, bool lio_cycle);
  bool hooks_en_ = false;
  bool bridge_valid_ = false;
  bool bridge_open_ = false;
  std::vector<BridgeSample> bridge_;
  MD(DIM_STATE, DIM_STATE) F_accum_ = MD(DIM_STATE, DIM_STATE)::Identity();
  double bridge_t0_ = 0.0, bridge_t1_ = 0.0;
  V3D bridge_bg_snap_ = V3D::Zero(), bridge_ba_snap_ = V3D::Zero(), bridge_g_snap_ = V3D::Zero();
  long bridge_orphan_steps_ = 0;
};
typedef std::shared_ptr<ImuProcess> ImuProcessPtr;
#endif