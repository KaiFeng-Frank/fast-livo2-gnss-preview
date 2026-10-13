// SPDX-License-Identifier: BSD-3-Clause
// GnssFusionLivgo — the FAST-LIVGO arm as its own type, so the T/U GnssFusion translation unit
// stays the pre-arm source (byte-identical host trajectories when gnss.livgo.enable is false).
#pragma once

#include "livo_gnss/gnss_fusion.hpp"
#include "livo_gnss/livgo.hpp"

namespace livo_gnss {

class LIVO_GNSS_API GnssFusionLivgo final : public MeasurementSink {
 public:
  // Throws std::invalid_argument when validateConfig() is non-empty.
  explicit GnssFusionLivgo(const Config& cfg, const LivgoConfig& livgo);
  ~GnssFusionLivgo() override;
  GnssFusionLivgo(const GnssFusionLivgo&) = delete;
  GnssFusionLivgo& operator=(const GnssFusionLivgo&) = delete;

  // ---- host-derived configuration (called once by the ROS glue before the first onLioStep) ----
  // T_{I<-L} (extR/extT) so lever_arm_frame=lidar can be turned into l^I (§1 "A").
  void setExtrinsics(const Eigen::Matrix3d& R_IL, const Eigen::Vector3d& t_IL);
  // ONLY_LO is refused (no IMU bridge, §9); returns false in that case.
  bool setSlamMode(SlamMode mode);
  // Host flags that change the Jacobian layout: gravity_est_en (δg columns), exposure estimation.
  void setHostFlags(bool gravity_est_en, bool exposure_est_en);
  void setDiagnosticsSink(DiagnosticsSink* sink);   // nullable; not owned
  void setRawDecoder(RawDecoder* decoder);          // nullable; not owned

  // ---- raw bytes (both replay and live converge here, §12.2) ----
  bool registerMessageDefinition(Source src, const std::string& datatype, const std::string& md5,
                                 const std::string& msg_def);   // false → event MSGDEF_REJECT
  void feedBytes(Source src, const uint8_t* data, size_t size, double t_record);

  // ---- decoded measurements (MeasurementSink) ----
  void pushObservation(ObsEpoch&& epoch) override;
  void pushEphemeris(const KeplerEph& eph) override;
  void pushGloEphemeris(const GloEph& eph) override;
  void pushIono(const Klobuchar& iono) override;
  void pushPvt(const ReceiverPvt& pvt) override;
  // AMENDMENTS A1: load broadcast ephemerides from RINEX NAV files (also done at construction for
  // cfg.nav_rinex). Returns the number of ephemerides added.
  int loadNavRinex(const std::string& path, std::string* why);

  // ---- hooks (DESIGN §2.4, §9) ----
  // H1 + state machine + H3. `bridge` holds this cycle's samples (first = cycle start), `F_tot` the
  // accumulated host transition over (t_prev, t_k]. Rewrites `core` (mean and P) when an update commits.
  // With update_slot = post_vio only H1 + bookkeeping happen here; the epochs (t_est ≤ t_k) are held for onPostVio().
  StepReport onLioStep(CoreState& core, const HostMat& F_tot, const std::vector<BridgeSample>& bridge,
                       double t_prev, double t_k);
  // H2: exact conditioning of the augmented block on the host's own update (prior = state_propagat,
  // post = _state). Called after handleLIO (:371) and handleVIO (:305); early-return paths do not call it.
  void onCoreUpdated(const CoreState& prior, const CoreState& post, UpdateSource src);
  // FAST-LIVGO dual mode. lambda_min is the weaker modality's translation-block Hessian eigenvalue.
  void setLivgoLambdaMin(double lambda_min);
  // update_slot = post_vio (I13 equivalence check, AMENDMENTS A12): runs the H3 updates held by the last
  // onLioStep on the VIO posterior. Called after H2(VIO) (LIVO) or after H2(LIO) (ONLY_LIO), before
  // onStepEnd(); `t_k` is that cycle's end. No-op (returns an empty report) when update_slot = pre_lio.
  StepReport onPostVio(CoreState& core, double t_k);
  // End of cycle: refresh C_ref with the final gravity, update the smoothed output state.
  void onStepEnd(const CoreState& core, double t_k);

  // ---- queries ----
  // Global pose T_{N<-I} = (M R, M(p − o^W) + t) with 6x6 covariance [δθ, δp] in N; false before ACTIVATE.
  bool globalPose(const CoreState& core, Eigen::Matrix3d* R_NI, Eigen::Vector3d* t_NI,
                  Eigen::Matrix<double, 6, 6>* cov) const;
  // Antenna position a^N and its covariance (traj_ant_N.tum); false before ACTIVATE.
  bool antennaN(const CoreState& core, Eigen::Vector3d* a_N, Eigen::Matrix3d* cov) const;
  // Rate-limited output pose for /gnss/global_odometry_smooth (§11); same validity as globalPose().
  bool globalPoseSmooth(Eigen::Matrix3d* R_NI, Eigen::Vector3d* t_NI) const;
  bool anchor(Geodetic* geo) const;
  bool anchor(AnchorFrame* frame) const;
  bool registration(Registration* xi) const;
  // Latest GNSS-only solutions (for /gnss/spp_fix and self-checks); false when none yet.
  bool lastSpp(Eigen::Vector3d* r_ecef, Eigen::Matrix3d* cov, GTime* t) const;
  FusionStatus status() const;
  // Augmented block for state_trace.txt: x_G and diag(Q_G) (hexfloat written by the caller) and slot names.
  void augmentedState(Eigen::VectorXd* x_G, Eigen::VectorXd* diag_Q_G, std::vector<std::string>* slots) const;
  const Config& config() const;
  Summary summary() const;
  // Flush end-of-run diagnostics (summary to the sink). Idempotent.
  void finish();

  // ---- appended (AMENDMENTS A61 F13; INTERFACE_CHANGES) ----
  // Odometry output: the global pose / antenna re-expressed as if the registration's estimate at ACTIVATE had been exact,
  // T_odo = [R_z(ψ̂₀(act) − ψ̂₀), â₀(act) − R_z(·)â₀] ∘ T_{N<-I}, where â₀, ψ̂₀ are today's estimates of the antenna and yaw at
  // ACTIVATE (Slot::Origin) and (act) their values then. The later revision of that instant — information about where
  // the whole trajectory lies, not about its shape — is left out; every correction of the drift since then is kept.
  // Same validity as globalPose(); equals it without the slot (output.odom_output = false) and right after ACTIVATE.
  bool globalPoseOdom(const CoreState& core, Eigen::Matrix3d* R_NI, Eigen::Vector3d* t_NI) const;
  bool antennaOdom(const CoreState& core, Eigen::Vector3d* a_N) const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace livo_gnss
