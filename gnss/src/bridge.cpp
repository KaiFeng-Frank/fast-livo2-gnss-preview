// SPDX-License-Identifier: BSD-3-Clause
// ImuBridge (DESIGN §4.1, AMENDMENTS A10). Owner: WP-b. Tests: U14.
#include "internal/bridge.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace livo_gnss {

namespace {

// AMENDMENTS A33: the ring holds every cycle whose end lies within max_bridge_s + kRingMarginS of the latest cycle end
// (and at least bridge_ring_cycles cycles). kTieSlackS keeps a cycle whose end sits exactly on that horizon in exact
// arithmetic (15 Hz cycles: 6 × 1/15 s = 0.4 s) whichever way t_latest − horizon rounds — the same 1e-9 s slack as
// the contiguity check in pushCycle. Keeping such a cycle costs nothing; the 0.1 s margin is already beyond δ_max.
constexpr double kRingMarginS = 0.1;
constexpr double kTieSlackS = 1e-9;

// Rotation over τ at constant rate ω in the host's own form Exp(angvel, dt) (utils/so3_math.h:24-40):
// axis = ω/‖ω‖, angle = ‖ω‖·τ, 1e-7 cut-off on ‖ω‖. Kept separate from so3Exp (cut-off on ‖ω τ‖) so that U14's
// term-by-term comparison with IMU_Processing.cpp:412 holds for the same inputs.
Eigen::Matrix3d expRate(const Eigen::Vector3d& w, double tau) {
  const double n = w.norm();
  const Eigen::Matrix3d Eye3 = Eigen::Matrix3d::Identity();
  if (n > 0.0000001) {
    const Eigen::Vector3d axis = w / n;
    const Eigen::Matrix3d K = skew(axis);
    const double ang = n * tau;
    return Eye3 + std::sin(ang) * K + (1.0 - std::cos(ang)) * K * K;
  }
  return Eye3;
}

// Relative motion of one cycle between t_a (interpolated) and its end t_end, plus the time-weighted specific
// force sum. Everything is a function of the a/ω samples only (ΔR = ∏Exp(ω dt), E = ∫a, D = ∫∫a − δ∫a), which is
// what makes chaining across cycles with jumping start states exact.
struct Segment {
  Eigen::Matrix3d dR = Eigen::Matrix3d::Identity();
  Eigen::Vector3d D = Eigen::Vector3d::Zero(), E = Eigen::Vector3d::Zero();
  double len = 0.0;                                  // t_end − t_a
  Eigen::Vector3d f_sum = Eigen::Vector3d::Zero();   // Σ f_body,i+1 · overlap_i
  double f_w = 0.0;                                  // Σ overlap_i
};

}  // namespace

ImuBridge::ImuBridge(const Config& cfg) : cfg_(cfg) {}

void ImuBridge::pushCycle(const std::vector<BridgeSample>& samples, double t_prev, double t_k, const Eigen::Vector3d& bg_snap,
                          const Eigen::Vector3d& ba_snap, const Eigen::Vector3d& g_snap) {
  if (samples.empty()) throw std::invalid_argument("ImuBridge::pushCycle: a LIO cycle needs at least the start sample");
  if (!(t_k > t_prev)) throw std::invalid_argument("ImuBridge::pushCycle: t_k must be > t_prev");
  if (std::abs(samples.front().t - t_prev) > 1e-9) throw std::invalid_argument("ImuBridge::pushCycle: samples[0] is not the cycle start");
  for (size_t i = 1; i < samples.size(); ++i)
    if (samples[i].t < samples[i - 1].t) throw std::invalid_argument("ImuBridge::pushCycle: sample times must be non-decreasing");
  // Cross-cycle chaining assumes t_prev(m+1) == t_k(m). The host guarantees it (prop_beg_time = last_prop_end_time,
  // IMU_Processing.cpp:248/:470), so a gap or overlap can only be a glue bug: refuse it instead of returning wrong
  // chained motion (and negative segment lengths) for epochs in the gap.
  if (!ring_.empty() && std::abs(t_prev - ring_.back().t_k) > 1e-9)
    throw std::invalid_argument("ImuBridge::pushCycle: cycle is not contiguous with the previous one (t_prev != last t_k)");
  Cycle c;
  c.t_prev = t_prev;
  c.t_k = t_k;
  c.bg_snap = bg_snap;
  c.ba_snap = ba_snap;
  c.g_snap = g_snap;
  c.samples = samples;
  ring_.push_back(std::move(c));
  // Retention by duration (AMENDMENTS A33): keep at least `bridge_ring_cycles` cycles and every cycle with
  // t_k ≥ t_latest − (max_bridge_s + 0.1 s). A count alone is not enough: with LIO driven by the 15 Hz camera
  // (M3DGR) four cycles span 0.267 s < max_bridge_s = 0.3 s, and a late epoch inside max_bridge_s would be refused as
  // "older than the ring". The cycles are contiguous and ordered, so the kept set is a suffix of the ring and dropping
  // from the front while the oldest cycle is outside the horizon is exact.
  const size_t keep = static_cast<size_t>(std::max(1, cfg_.time.bridge_ring_cycles));
  const double horizon = t_k - (cfg_.time.max_bridge_s + kRingMarginS);
  while (ring_.size() > keep && ring_.front().t_k < horizon - kTieSlackS) ring_.pop_front();
}

void ImuBridge::interpolate(const BridgeSample& a, const BridgeSample& b, double t, Eigen::Matrix3d* R, Eigen::Vector3d* p,
                            Eigen::Vector3d* v) {
  // (t_i, t_{i+1}]: constant a_{i+1} (incl. g) and ω_{i+1}, τ = t − t_i. Same three statements as
  // IMU_Processing.cpp:412-421 with dt → τ (U14 checks them term by term).
  const double tau = t - a.t;
  *R = a.R * expRate(b.omega, tau);
  *p = a.p + a.v * tau + 0.5 * b.a_world * tau * tau;
  *v = a.v + b.a_world * tau;
}

namespace {

// Index i of the interval (t_i, t_{i+1}] used for t: the first one with t_{i+1} > t, so that t on a sample instant
// takes the interval that *starts* there (τ = 0 reproduces the sample exactly); the last interval when none.
size_t intervalOf(const std::vector<BridgeSample>& s, double t) {
  size_t i = 0;
  while (i + 2 < s.size() && !(s[i + 1].t > t)) ++i;
  return i;
}

// Segment of one cycle from t_a (∈ [t_prev, t_k]) to the cycle end; (R_a, p_a, v_a, ω_a) is the nominal state at t_a
// in that cycle's own state chain.
Segment segmentFrom(const std::vector<BridgeSample>& s, double t_a, double t_end, Eigen::Matrix3d* R_a, Eigen::Vector3d* p_a,
                    Eigen::Vector3d* v_a, Eigen::Vector3d* omega_a) {
  Segment seg;
  const BridgeSample& last = s.back();
  if (s.size() == 1 || !(t_a < last.t)) {
    // At (or past) the last sample, or a cycle with only its start sample: the state is held, no motion
    // (D = E = 0, ΔR = I); this keeps δ = 0 epochs bit-exact instead of rounding through interpolate().
    *R_a = last.R;
    *p_a = last.p;
    *v_a = last.v;
    *omega_a = last.omega;
    seg.len = std::max(t_end, last.t) - t_a;
    return seg;
  }
  const size_t i = intervalOf(s, t_a);
  const BridgeSample& sa = s[i];
  const BridgeSample& sb = s[i + 1];
  const double t_c = std::max(t_a, sa.t);   // t_a == t_prev (within 1e-9) lands on the start sample
  ImuBridge::interpolate(sa, sb, t_c, R_a, p_a, v_a);
  *omega_a = sb.omega;   // rate of the interval that contains t_a (the host's angvel_avr of that interval)
  seg.len = std::max(t_end, last.t) - t_a;
  seg.dR = R_a->transpose() * last.R;
  seg.E = last.v - *v_a;
  seg.D = last.p - *p_a - last.v * seg.len;
  // time-weighted specific force over (t_a, t_end]: partial first interval, then whole intervals
  {
    const double w0 = std::max(0.0, sb.t - t_c);
    seg.f_sum += sb.f_body * w0;
    seg.f_w += w0;
    for (size_t k = i + 1; k + 1 < s.size(); ++k) {
      const double w = std::max(0.0, s[k + 1].t - s[k].t);
      seg.f_sum += s[k + 1].f_body * w;
      seg.f_w += w;
    }
  }
  return seg;
}

}  // namespace

bool ImuBridge::query(double t_g, BridgeResult* out) const {
  if (!out) return false;
  *out = BridgeResult{};
  if (ring_.empty()) return false;
  const Cycle& last = ring_.back();
  out->t_g = t_g;
  out->t_k = last.t_k;
  out->delta = last.t_k - t_g;
  if (t_g > last.t_k) return false;                        // epoch belongs to the next cycle
  if (out->delta > cfg_.time.max_bridge_s) return false;   // LATE_EPOCH (counted by the caller)
  // Cycles are contiguous (t_prev(m+1) == t_k(m)); take the latest one whose start is not after t_g.
  int j = static_cast<int>(ring_.size()) - 1;
  while (j > 0 && t_g < ring_[j].t_prev) --j;
  if (t_g < ring_[j].t_prev) return false;                 // older than the ring
  const int n = static_cast<int>(ring_.size()) - 1;

  Eigen::Matrix3d R_a;
  Eigen::Vector3d p_a, v_a, omega_a;
  Segment acc = segmentFrom(ring_[j].samples, t_g, ring_[j].t_k, &R_a, &p_a, &v_a, &omega_a);
  // Chain the whole later cycles: ΔR_ac = ΔR_ab ΔR_bc, E_ac = E_ab + E_bc, D_ac = D_ab + D_bc − E_bc·δ_ab
  // (δ_ab = accumulated length so far); all start-state jumps at the cycle boundaries cancel.
  for (int m = j + 1; m <= n; ++m) {
    Eigen::Matrix3d Rb;
    Eigen::Vector3d pb, vb, wb;
    const Cycle& c = ring_[m];
    const Segment s = segmentFrom(c.samples, c.t_prev, c.t_k, &Rb, &pb, &vb, &wb);
    acc.D = acc.D + s.D - s.E * acc.len;
    acc.E += s.E;
    acc.dR = acc.dR * s.dR;
    acc.len += s.len;
    acc.f_sum += s.f_sum;
    acc.f_w += s.f_w;
  }

  out->ok = true;
  out->cross_cycle = (j < n);
  out->dR = acc.dR;
  out->D = acc.D;
  out->E = acc.E;
  out->omega_g = omega_a;
  out->bg_snap = ring_[j].bg_snap;   // ω̂_g was bias-corrected with this b̂_g, so ω_g = ω̂_g + b̂_g^snap − b_g is exact
  out->ba_snap = last.ba_snap;       // δa uses the snapshot of the cycle that ends at t_k (A10)
  out->g_snap = last.g_snap;
  out->R_k = last.samples.back().R;
  out->a_bar = (acc.f_w > 0.0) ? Eigen::Vector3d(acc.f_sum / acc.f_w) : last.samples.back().f_body;
  const int crossed = n - j;
  // Only the IMU white-noise term here: after A10 the (σ_ba, σ_θ, σ_g) sensitivity of D/E is carried by the δθ,
  // δb_a and δg columns of J_a/J_w through Σ, so putting it into Σ_D as well would count it twice. The full
  // formula stays available as sigmaD() for callers that need it.
  out->Sigma_D = sigmaD(out->delta, cfg_.imu_bridge.acc_n, 0.0, 0.0, last.g_snap.norm(), crossed);
  out->var_v_bridge = cfg_.imu_bridge.acc_n * cfg_.imu_bridge.acc_n * std::max(out->delta, 0.0);
  {   // A61 F8: the IMU interval that holds t_g (the one ω̂_g belongs to)
    const std::vector<BridgeSample>& sm = ring_[j].samples;
    double dt_s = 0.0;
    for (size_t i = 1; i < sm.size(); ++i)
      if (sm[i].t >= t_g) {
        dt_s = sm[i].t - sm[i - 1].t;
        break;
      }
    if (!(dt_s > 0.0) && sm.size() >= 2) dt_s = sm.back().t - sm[sm.size() - 2].t;
    out->var_omega = dt_s > 0.0 ? cfg_.imu_bridge.gyr_n * cfg_.imu_bridge.gyr_n / (2.0 * dt_s) : 0.0;
  }
  // Nominal state at t_g expressed through the latest snapshot (identical to the direct interpolation when the
  // epoch lies in the current cycle; consistent with evaluate() at the snapshot when it does not).
  const BridgeSample& sk = last.samples.back();
  out->R_g_hat = sk.R * acc.dR.transpose();
  out->p_g_hat = sk.p - sk.v * out->delta - acc.D;
  out->v_g_hat = sk.v - acc.E;
  return true;
}

double ImuBridge::latestCycleEnd() const { return ring_.empty() ? 0.0 : ring_.back().t_k; }
double ImuBridge::oldestTime() const { return ring_.empty() ? 0.0 : ring_.front().t_prev; }
int ImuBridge::cyclesStored() const { return static_cast<int>(ring_.size()); }

AntennaAtEpoch ImuBridge::evaluate(const CoreState& x, const BridgeResult& br, const Eigen::Vector3d& lever_I, bool gravity_est_en) {
  // AMENDMENTS A10 (the U7 reference model):
  //   δθ = Log(R̂_kᵀ R),  δa = −R̂_k[ā_b]× δθ − R̂_k(b_a − b̂_a) + (g − ĝ)   [last term only with gravity_est_en]
  //   D(x) = D̂ − ½δ² δa,  E(x) = Ê + δ δa
  //   p_g = p − vδ − D(x),  R_g = R ΔR̂ᵀ,  v_g = v − E(x),  ω_g = ω̂_g + b̂_g − b_g
  //   y = p_g + R_g l,  z = v_g + R_g [ω_g]× l
  AntennaAtEpoch a;
  const double d = br.delta;
  const Eigen::Vector3d dtheta = so3Log(Eigen::Matrix3d(br.R_k.transpose() * x.R));
  Eigen::Vector3d da = -br.R_k * (br.a_bar.cross(dtheta)) - br.R_k * (x.ba - br.ba_snap);
  if (gravity_est_en) da += x.g - br.g_snap;
  const Eigen::Vector3d D = br.D - 0.5 * d * d * da;
  const Eigen::Vector3d E = br.E + d * da;
  a.p_g = x.p - x.v * d - D;
  a.R_g = x.R * br.dR.transpose();
  a.v_g = x.v - E;
  a.omega_g = br.omega_g + br.bg_snap - x.bg;
  a.y = a.p_g + a.R_g * lever_I;
  a.z = a.v_g + a.R_g * (a.omega_g.cross(lever_I));
  return a;
}

Eigen::Matrix3d ImuBridge::sigmaD(double delta, double acc_n, double sigma_ba, double sigma_theta, double g, int crossed_cycles) {
  // §4.1: Σ_D ≈ (acc_n² δ³/3 + (½ δ² (σ_ba + g σ_θ))²) I, plus (2 mm)² per crossed cycle (the a_world samples of
  // a later cycle were recorded with a slightly different attitude/bias after the host's update).
  const double d = std::max(delta, 0.0);
  const double white = acc_n * acc_n * d * d * d / 3.0;
  const double bias = 0.5 * d * d * (sigma_ba + g * sigma_theta);
  const double cross = static_cast<double>(std::max(crossed_cycles, 0)) * (2.0e-3 * 2.0e-3);
  return (white + bias * bias + cross) * Eigen::Matrix3d::Identity();
}

}  // namespace livo_gnss
