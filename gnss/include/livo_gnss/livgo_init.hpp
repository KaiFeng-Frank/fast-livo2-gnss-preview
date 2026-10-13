// SPDX-License-Identifier: BSD-3-Clause
// FAST-LIVGO initialisation, arXiv:2606.19190v2 eq. (2)–(3): least squares for the full W→ENU
// rotation, the ENU translation and the IMU→antenna lever, on the initialisation window's GNSS
// positions and LIO poses. The paper does not give N or the window; the caller passes the window
// the existing initialiser already accepted.
#pragma once

#include <cstddef>
#include <deque>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Core>

namespace livo_gnss {

struct LivgoInitSample {
  Eigen::Vector3d p_I = Eigen::Vector3d::Zero();   // ^G p_I
  Eigen::Matrix3d R_I = Eigen::Matrix3d::Identity();  // ^G R_I
  Eigen::Vector3d p_r = Eigen::Vector3d::Zero();   // ^E p_r
};

struct LivgoInitEstimate {
  bool ok = false;
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();  // ^E R_G
  Eigen::Vector3d p = Eigen::Vector3d::Zero();      // ^E p_G
  Eigen::Vector3d lever = Eigen::Vector3d::Zero();  // ^I t_r
  double rms_m = 0.0;
  int n = 0;
  int iterations = 0;
  // Formal covariance of the left-perturbation [δφ(3), δp(3), δt(3)] at the solution, σ² (H+μI)⁻¹.
  // μ = 1e-6·trace(H) holds a rank-deficient direction at the start value; the corresponding σ is then large.
  Eigen::Matrix<double, 9, 9> cov = Eigen::Matrix<double, 9, 9>::Zero();
  double lambda_min = 0.0;   // smallest eigenvalue of the undamped normal matrix
  std::string note;
};

// R0 / lever0 are the start (the mechanical lever, so an unobservable lever component stays there).
// p0 is the caller's start for ^E p_G. Needs at least 10 epochs.
bool livgoInitLeastSquares(const std::vector<LivgoInitSample>& samples, const Eigen::Matrix3d& R0,
                           const Eigen::Vector3d& p0, const Eigen::Vector3d& lever0, LivgoInitEstimate* out);

// ---- AMENDMENTS A80 (LIVGO-literal) ----

// One synchronous epoch of the history the frame least squares runs on: the host IMU pose at the epoch's estimator time and the
// epoch's SPP position, both as the facade has them (the SPP in ECEF; the ENU frame is fixed only when the least squares runs).
struct LivgoInitEpoch {
  Eigen::Vector3d p_I = Eigen::Vector3d::Zero();      // ^W p_I
  Eigen::Matrix3d R_I = Eigen::Matrix3d::Identity();  // ^W R_I
  Eigen::Vector3d r_ecef = Eigen::Vector3d::Zero();   // SPP position of the antenna
  // ---- AMENDMENTS A82 (livgo.dtw_at_init). Read only by the DTW at the frame initialisation; NaN = not available. ----
  double t_est = std::numeric_limits<double>::quiet_NaN();     // estimator time the pose above belongs to (before any DTW lag)
  double v_gnss = std::numeric_limits<double>::quiet_NaN();    // GNSS Doppler speed magnitude of the epoch [m/s]
  double v_host = std::numeric_limits<double>::quiet_NaN();    // host antenna speed magnitude at t_est [m/s]
};

// ---- AMENDMENTS A82 ----
// One sample of the host's pose track: the host IMU pose and velocity at an estimator time, W frame (the facade evaluates its IMU bridge
// at every GNSS epoch of the collection and keeps the result here). The track is what a DTW lag re-pairs the history's GNSS positions
// with: the host pose at (epoch time + lag) is read off it.
struct LivgoHostPose {
  double t = 0.0;
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();   // ^W R_I
  Eigen::Vector3d p = Eigen::Vector3d::Zero();       // ^W p_I
  Eigen::Vector3d v = Eigen::Vector3d::Zero();       // ^W v_I
};

// Host pose at time `t` between two samples a (t_a) and b (t_b), t_a <= t <= t_b, t_a < t_b: position by the cubic Hermite polynomial
// through (p, v) at both ends (exact whenever the position is a cubic polynomial of time over the step, i.e. up to a constant jerk; the
// error is h^4/384 times the snap otherwise), rotation by R_a Exp(s Log(R_a^T R_b)), s = (t - t_a)/(t_b - t_a) (constant rate). Errors
// against the continuous motion: position < 0.03 mm at h = t_b - t_a = 0.1 s and a snap of 100 m/s^4, rotation about alpha h^2/8 for an angular acceleration alpha
// (about 1 mrad at 1 rad/s^2, h = 0.1 s, i.e. 0.2 mm at a 0.2 m lever). Both are far below the SPP noise the pose is paired with.
void livgoInterpolateHostPose(const LivgoHostPose& a, const LivgoHostPose& b, double t, Eigen::Matrix3d* R, Eigen::Vector3d* p);

// A track interval wider than this is not interpolated across (the epoch is dropped from the re-paired history): the interpolation
// above is a statement about a smooth motion over a short step, not a promise over a data hole. A numerical-validity guard, not a
// method parameter: the receiver epochs are 0.1-0.2 s apart, ten times that is a hole.
constexpr double kLivgoRepairMaxGapS = 1.0;

struct LivgoRepairResult {
  int repaired = 0;          // history epochs whose pose was replaced by the host pose at (t_est + lag)
  int dropped = 0;           // history epochs left out: compensated time outside the track, a hole in the track, no t_est
  double max_gap_s = 0.0;    // widest track interval interpolated across
};

// The history's time-aligned series for the DTW: the epochs that have t_est, v_gnss and v_host.
struct LivgoSpeedSeries {
  std::vector<double> t, v_gnss, v_host;
};

// Every synchronous epoch collected since the collection started (the last clear()), and the horizontal travel of the host over
// them. The travel is the largest horizontal distance of any added epoch's p_I from the FIRST epoch added since the last clear(),
// "horizontal" meaning orthogonal to the gravity the caller passes with the epoch (host gravity in W; any scale, direction only).
// It never decreases (a trajectory that comes back has still been that far). The capacity keeps a vehicle that never moves from
// growing the list without bound; the default is 100000 epochs (2.8 h at 10 Hz), far above any travel gate's wait; at the capacity
// the oldest epochs are dropped (the travel keeps its reference).
class LivgoInitHistory {
 public:
  explicit LivgoInitHistory(std::size_t capacity = 100000) : capacity_(capacity < 2 ? 2 : capacity) {}
  void add(const LivgoInitEpoch& e, const Eigen::Vector3d& g_W);
  double travelM() const { return travel_m_; }
  std::size_t size() const { return epochs_.size(); }
  bool empty() const { return epochs_.empty(); }
  void clear();
  // The least-squares input in the ENU frame (r0_ecef, R_EN): ^E p_r = R_ENᵀ (r_ecef − r0_ecef).
  std::vector<LivgoInitSample> samples(const Eigen::Vector3d& r0_ecef, const Eigen::Matrix3d& R_EN) const;
  const std::deque<LivgoInitEpoch>& epochs() const { return epochs_; }

  // ---- AMENDMENTS A82 (livgo.dtw_at_init) ----
  // The host pose track (LivgoHostPose), cleared with the history. Appended in time order; a sample whose time is not later than the
  // last one (or not finite) is ignored. Capacity: the history's (the oldest samples go first).
  void addTrack(const LivgoHostPose& p);
  std::size_t trackSize() const { return track_.size(); }
  const std::deque<LivgoHostPose>& track() const { return track_; }
  // The series the DTW runs on: the epochs that carry t_est, v_gnss and v_host (all finite), in history order.
  LivgoSpeedSeries speedSeries() const;
  // The paper's order puts the time compensation before the least squares: with a DTW lag adopted, the GNSS position of an epoch stamped
  // t_est belongs to the host pose at (t_est + lag_s), not at t_est. Every epoch's p_I / R_I is replaced by the track's pose at
  // that time (livgoInterpolateHostPose between the two samples around it). An epoch without a finite t_est, whose compensated time is
  // outside the track, or that falls in a track interval wider than kLivgoRepairMaxGapS is removed from the history (the travel keeps its
  // value: the gate has opened). The track stays.
  LivgoRepairResult repairPairing(double lag_s);

 private:
  std::size_t capacity_;
  std::deque<LivgoInitEpoch> epochs_;
  std::deque<LivgoHostPose> track_;   // A82
  double travel_m_ = 0.0;
  bool have_origin_ = false;
  Eigen::Vector3d origin_ = Eigen::Vector3d::Zero();   // p_I of the first epoch added since the last clear()
};

// The pre-registered tilt rule of A80: with max_deg > 0 the least-squares tilt is used only when its formal sigma is at most
// max_deg. sigma_deg is the larger of the formal sigmas of the two horizontal rotation axes (the E and N components of the left
// perturbation, est.cov(0,0) and est.cov(1,1)); a non-finite value counts as exceeding. max_deg <= 0: no check, the
// least-squares tilt is always used and sigma_deg is still reported.
struct LivgoTiltDecision {
  bool use_ls_tilt = true;
  double sigma_deg = 0.0;
};
LivgoTiltDecision livgoTiltDecision(const LivgoInitEstimate& est, double max_deg);

}  // namespace livo_gnss
