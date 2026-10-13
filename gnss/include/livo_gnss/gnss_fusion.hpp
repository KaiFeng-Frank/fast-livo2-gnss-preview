// SPDX-License-Identifier: BSD-3-Clause
// GnssFusion — the single facade of the ROS-free core (DESIGN §12.5). All calls are synchronous and
// happen on the host's single processing thread (RTKLIB has global mutable state, §12.1).
//
// Call sequence per LIO cycle (DESIGN §9):
//   onLioStep()      H1 propagation → state machine → H3 joint update(s) (update_slot = pre_lio); may rewrite `core`
//   onCoreUpdated()  H2 conditioning after the host's LIO and VIO updates (prior/posterior pairs)
//   onPostVio()      update_slot = post_vio only: the H3 updates deferred from onLioStep run here (M4)
//   onStepEnd()      refresh C_ref, expose global pose for the writers
// Measurements arrive through the MeasurementSink interface (already decoded) or through
// feedBytes() (raw bytes routed to the RawDecoder registered by the ROS glue, see gnss_io).
#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "livo_gnss/config.hpp"
#include "livo_gnss/core_state.hpp"
#include "livo_gnss/diagnostics.hpp"
#include "livo_gnss/export.hpp"
#include "livo_gnss/types.hpp"

namespace livo_gnss {

// Decoded-measurement entry points. GnssFusion implements it; gnss_io decoders write into it.
class LIVO_GNSS_API MeasurementSink {
 public:
  virtual ~MeasurementSink() = default;
  virtual void pushObservation(ObsEpoch&& epoch) = 0;
  virtual void pushEphemeris(const KeplerEph& eph) = 0;
  virtual void pushGloEphemeris(const GloEph& eph) = 0;
  virtual void pushIono(const Klobuchar& iono) = 0;
  virtual void pushPvt(const ReceiverPvt& pvt) = 0;
};

// Raw-byte decoder interface implemented in gnss_io (BSD-3, ROS-free). The core owns no decoder so
// it stays independent of gnss_io; the ROS glue wires one in with setRawDecoder().
class LIVO_GNSS_API RawDecoder {
 public:
  virtual ~RawDecoder() = default;
  // Register the ROS1 message definition of a wire channel before the first feedBytes() on it.
  // Returns false (and fills `why`) when the flattened (type,name) layout differs from the built-in
  // expectation table (DESIGN §12.2 "不一致即拒收"). Non-ROS sources ignore this call.
  virtual bool registerMessageDefinition(Source src, const std::string& datatype, const std::string& md5,
                                         const std::string& msg_def, std::string* why) = 0;
  // Decode one message / chunk of bytes and push the result into `sink`. Returns false on a
  // malformed buffer (never throws, never crashes on garbage; U2).
  virtual bool decode(Source src, const uint8_t* data, size_t size, double t_record, MeasurementSink& sink,
                      std::string* why) = 0;
};

// (`FusionStatus`, not `Status`: X11/Xlib.h `#define Status int` would break any TU that includes both.)
struct LIVO_GNSS_API FusionStatus {
  Phase phase = Phase::OFF;
  bool active = false;           // registration slots exist (ACTIVATE happened)
  bool degenerate = false, livo_alarm = false, schmidt = false;
  int n_slots = 0, aug_dim = 0;
  int n_sats_last = 0;
  GTime last_epoch;
  double last_epoch_t_est = 0.0;
  int epochs_processed = 0, epochs_committed = 0, epochs_rolled_back = 0;
  double sigma_yaw_rad = 0.0, sigma_t_m = 0.0, sigma_b_m = 0.0;
  int tdcp_pairs_open = 0;
  double outage_s = 0.0;         // time since last valid epoch
};

struct LIVO_GNSS_API StepReport {
  int epochs_processed = 0;      // epochs consumed this cycle (t_est ≤ t_k)
  int epochs_committed = 0;
  int epochs_rolled_back = 0;
  bool core_modified = false;    // host mean/cov were rewritten
  bool activated = false;        // ACTIVATE happened in this cycle
  double elapsed_ms = 0.0;
};

class LIVO_GNSS_API GnssFusion final : public MeasurementSink {
 public:
  // Throws std::invalid_argument when validateConfig() is non-empty.
  explicit GnssFusion(const Config& cfg);
  ~GnssFusion() override;
  GnssFusion(const GnssFusion&) = delete;
  GnssFusion& operator=(const GnssFusion&) = delete;

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
