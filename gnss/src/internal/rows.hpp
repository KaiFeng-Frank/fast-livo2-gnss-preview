// SPDX-License-Identifier: BSD-3-Clause
// rows — measurement rows and Jacobians (DESIGN §4.2–§4.6):
//   J_a = ∂a^N/∂δx, J_w = ∂w^N/∂δx over [host 19 | φ | ψ | t | clk | isb | clone | td] (table §4.2);
//   pseudorange row  z = P_corr − [‖r_s − r_A‖ + Sagnac + b̂ − δ d̂ + ι̂_s − c δt_s + T̂ + Î],  H = −u_jᵀ J_a, +1 δb, −δ δd, +1 δι_s
//                    P_corr = P − rtk::codeBiasMetres(sat, state, code); NaN ⇒ no row (AMENDMENTS A37)
//   Doppler row      z = −λD − [e_jᵀ(v_s − ṙ_A) + Sagnac_rate + d̂ − c δṫ_s],                   H = −u_jᵀ J_w, +1 δd
//                    Sagnac_rate = (ω_e/c)(ẋ_s y_A + x_s ẏ_A − ẏ_s x_A − y_s ẋ_A) = d/dt of the pseudorange's
//                    Sagnac term (AMENDMENTS A36; RTKLIB resdop and the original DESIGN §1 have the opposite sign)
//   TDCP row         z_j = λ(Φ_k1 − Φ_k0) − {[ρ_k1(a^N(x̂)) − ρ_k0(ĉ)] − c(δt_s,k1 − δt_s,k0)},  H: −u_k1ᵀ J_a (current), +u_k1ᵀ (clone)
//                    (AMENDMENTS A64: the clone column takes the k1 line of sight; the exact derivative +u_k0ᵀ would add
//                    (u_k0 − u_k1)ᵀδc, the chain's float-PPP-like absolute information — U7 checks both)
//                    then Δclk removed by Householder projection (m rows → m−1 unit-variance rows)
//   PVT rows         z_p = ENU(lat,lon,h) − a^N, H = J_a position part; z_v = (vel_e, vel_n, −vel_d) − w^N
//   1 Hz selection   pseudorange and PVT rows only for the first epoch to arrive in each GPST second (AMENDMENTS A43,
//                    EpochDecimator below; replaces DESIGN §4.3's |tow − round(tow)| < 5 ms)
// Innovation convention ν = z − ĥ(x̂), δx = Kν (same as LIO's z = −dis, voxel_map.cpp:457).
// Owner: WP-c. Tests: test_u07_jacobians.cpp (manifold central differences, rel. err < 1e-6),
// test_u10_updater.cpp (Householder ≡ single-difference + whitening), test_u11_tdcp.cpp.
#pragma once

#include <cstdint>
#include <deque>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "livo_gnss/config.hpp"
#include "livo_gnss/core_state.hpp"
#include "livo_gnss/diagnostics.hpp"
#include "livo_gnss/types.hpp"
#include "aug_block.hpp"
#include "bridge.hpp"
#include "leveling.hpp"
#include "quality.hpp"
#include "rtk_adapter.hpp"

namespace livo_gnss {

// One scalar measurement row over the joint error state (19 + n columns).
struct MeasRow {
  MeasClass cls = MeasClass::Psr;
  SatId sat;
  int pair_id = -1;                 // TDCP pair (clone tag)
  double z = 0.0;                   // measurement
  double h = 0.0;                   // prediction ĥ(x̂)
  double nu = 0.0;                  // innovation z − h
  Eigen::RowVectorXd H;             // 1 × (19 + n)
  double R = 0.0;                   // variance (diagonal part)
  double el_rad = 0.0, az_rad = 0.0, cn0 = 0.0;
  RejectReason reason = RejectReason::NotRejected;   // set by the gates; NotRejected = candidate
  bool accepted = false;
};

// A set of rows of one class for one epoch. When `R_full` is set it carries the bridge-induced
// off-diagonal correlations (TDCP), else R is diagonal from MeasRow::R.
struct RowSet {
  MeasClass cls = MeasClass::Psr;
  std::vector<MeasRow> rows;
  Eigen::MatrixXd R_full;
  bool has_full_R = false;
  int nCandidates() const { return static_cast<int>(rows.size()); }
};

// Geometry shared by all rows of one satellite at one epoch.
struct SatGeometry {
  SatId sat;
  rtk::SatState state;
  Eigen::Vector3d e_ecef = Eigen::Vector3d::Zero();   // unit vector receiver → satellite (ECEF)
  // e in the local ENU of `rcv_geo` (the receiver): az/el companion and diagnostics only. The row builders form
  // their u_j = R_ENᵀ e from the anchor's R_EN (LinPoint::anchor) so H matches the N frame of J_a (review m3).
  Eigen::Vector3d u_N = Eigen::Vector3d::Zero();
  double range = 0.0;                                 // geodist incl. Sagnac [m]
  double az_rad = 0.0, el_rad = 0.0;
  double ion_m = 0.0, trop_m = 0.0, var_ion = 0.0, var_trop = 0.0;
  double lambda_m = 0.0;
};

// The linearisation point of one epoch: host iterate + bridge + registration + slots.
struct LinPoint {
  const CoreState* x = nullptr;
  const BridgeResult* bridge = nullptr;
  const AugBlock* aug = nullptr;
  const Leveling* leveling = nullptr;
  const AnchorFrame* anchor = nullptr;
  Eigen::Vector3d lever_I = Eigen::Vector3d::Zero();
  Eigen::Vector3d pivot_W = Eigen::Vector3d::Zero();
  bool gravity_est_en = true;
  bool exposure_est_en = true;
  double delta = 0.0;              // δ (bridge)
};

// Antenna position/velocity in N with their Jacobians over the joint error state.
struct AntennaJacobians {
  Eigen::Vector3d a_N = Eigen::Vector3d::Zero();
  Eigen::Vector3d w_N = Eigen::Vector3d::Zero();
  Eigen::Vector3d alpha_N = Eigen::Vector3d::Zero();   // antenna acceleration (td column)
  Eigen::Matrix<double, 3, Eigen::Dynamic> J_a;        // 3 × (19+n)
  Eigen::Matrix<double, 3, Eigen::Dynamic> J_w;
  Eigen::Matrix3d M = Eigen::Matrix3d::Identity();
};

// Appended by P3-C (AMENDMENTS A43; INTERFACE_CHANGES): the 1 Hz decimation rule of the pseudorange and L-PVT rows.
// Every GPST bin ⌊t / decim_s⌋ (decim_s = 1 s: the whole second ⌊tow⌋) is served by the FIRST epoch that arrives in it,
// whatever the phase of the receiver's epoch grid (M3DGR: −7 … +5 ms off the tenths of a second, per recording, so the
// old |tow − round(tow)| < 5 ms took no epoch at all on Grass01/Occlusion03/04). Every other epoch of that bin is
// skipped (SameBin), and so is an epoch of a bin older than the last one taken (Stale: arrived out of order). Skips are
// counted for gnss_epochs.csv. offer() is idempotent per epoch time: offering an epoch again (the relinearisations of
// one epoch, or a caller that offers every arriving epoch and builds rows for some of them later) returns its first
// verdict and counts nothing — within the last kRecent distinct epochs offered.
class EpochDecimator {
 public:
  enum class Verdict : uint8_t { Take = 0, SameBin, Stale, NoTime };
  static constexpr size_t kRecent = 64;   // distinct epoch times remembered for idempotency (6.4 s at 10 Hz)

  explicit EpochDecimator(double decim_s = 1.0);
  Verdict offer(const GTime& t);
  bool admit(const GTime& t) { return offer(t) == Verdict::Take; }
  // ⌊t / decim_s⌋ in GPST seconds (GTime scale); exact integer arithmetic when decim_s is a whole number of seconds.
  int64_t binOf(const GTime& t) const;
  double decimS() const { return decim_s_; }
  uint64_t taken() const { return n_take_; }
  uint64_t skippedSameBin() const { return n_same_; }
  uint64_t skippedStale() const { return n_stale_; }
  uint64_t skipped() const { return n_same_ + n_stale_; }
  void reset();

 private:
  double decim_s_ = 1.0;
  int64_t whole_s_ = 1;                                // decim_s_ as whole seconds, 0 when it is not one
  bool has_bin_ = false;
  int64_t last_bin_ = 0;                               // latest bin taken
  std::deque<std::pair<GTime, Verdict>> recent_;       // first verdict of the last kRecent distinct epochs
  uint64_t n_take_ = 0, n_same_ = 0, n_stale_ = 0;
};

class RowBuilder {
 public:
  RowBuilder(const Config& cfg, const NoiseModel& noise);

  // §4.2 table. Columns for absent slots are omitted (matrix width = 19 + aug->dim()).
  AntennaJacobians antennaJacobians(const LinPoint& lp) const;
  // Geometry for one satellite at the antenna ECEF position `r_A` (from a_N and the anchor). `rcv_geo` is the
  // receiver's own geodetic position (az/el, Saastamoinen height, Klobuchar pierce point) — never the anchor
  // (review m3). `clock_bias_m` (b̂) is needed for carrier-only epochs: the transmit time then uses
  // ρ̂ + b̂ − c·δt_s instead of P (§1, m5).
  bool geometry(const SatObs& obs, const rtk::SatSolver& solver, const Eigen::Vector3d& r_A_ecef,
                const Geodetic& rcv_geo, const Klobuchar* ion, GTime t, double clock_bias_m, SatGeometry* out) const;
  // Same evaluation with a frozen ephemeris copy (rtk::SatSolver::satStateFromSnapshot): the k1 end of a TDCP
  // pair is built through this with the snapshot captured at k0, so both ends use one ephemeris even when the
  // store switched in between (§4.5, R2-11). Appended by WP-c (INTERFACE_CHANGES.md, review m4).
  bool geometryFromSnapshot(const SatObs& obs, const rtk::EphSnapshot& snap, const Eigen::Vector3d& r_A_ecef,
                            const Geodetic& rcv_geo, const Klobuchar* ion, GTime t, double clock_bias_m,
                            SatGeometry* out) const;

  // Row builders. Each appends candidate rows (reason = None) for the usable signals of `epoch`. pseudorangeRows() builds
  // none for an epoch that did not take its GPST second (A43, see offerPsrEpoch() at the end of the class).
  void pseudorangeRows(const ObsEpoch& epoch, const std::vector<SatGeometry>& geo, const std::vector<MaskVerdict>& masks,
                       const LinPoint& lp, const AntennaJacobians& J, RowSet* out) const;
  void dopplerRows(const ObsEpoch& epoch, const std::vector<SatGeometry>& geo, const std::vector<MaskVerdict>& masks,
                   const LinPoint& lp, const AntennaJacobians& J, const std::vector<int>& tdcp_excluded_sats, RowSet* out) const;
  // TDCP: k0 quantities come from the clone slot (ĉ, ρ_k0 with the frozen ephemeris snapshot) and
  // k1 from the current linearisation; rows are *before* projection (one per satellite + Δclk nuisance).
  struct TdcpPairInput {
    int pair_id = -1;
    int clone_slot = -1;
    GTime t_k0, t_k1;
    std::vector<SatId> sats;
    std::vector<double> phi_k0_cyc, phi_k1_cyc;          // carrier phase at both ends
    std::vector<double> rho_k0;                          // ρ_k0(ĉ) at the *previous* iterate (diagnostics only)
    std::vector<double> dts_k0, dts_k1;                  // satellite clock at both ends [s]
    std::vector<Eigen::Vector3d> u_k0;                   // LOS at k0 at the previous iterate (not read: A64 clone column uses u_k1)
    std::vector<double> cp_std_k0_m, el_k0_rad;
    // m3: what tdcpRows() needs to recompute ρ_k0 at every iterate ĉ_i and to fill §4.7:
    // Frozen ephemeris captured at k0 (R2-11). The caller builds `geo_k1` with geometryFromSnapshot(eph_snap[j], …);
    // tdcpRows() refuses a k1 state whose toe differs from the snapshot's (review m4). Empty ⇒ no check (tests).
    std::vector<rtk::EphSnapshot> eph_snap;
    std::vector<double> psr_k0;                          // P at k0 used for the k0 transmit time (caller's record; not read here)
    std::vector<Eigen::Vector3d> rs_k0;                  // satellite ECEF position at the k0 transmit time
    std::vector<double> cp_std_k1_m, el_k1_rad;          // σ_Φ(k1) inputs
    Eigen::Vector3d dv_pair = Eigen::Vector3d::Zero();   // Δv_pair = w^N(k1) − w^N(k0) for the σ_toff term
    double dt_pair_s = 0.0;                              // t_k1 − t_k0
    double clock_bias_k0_m = 0.0;                        // b̂ at k0 (caller's carrier-only transmit time; not read here)
    // Appended by WP-c (INTERFACE_CHANGES.md): the clone value at the *current* iterate, ĉ_i, read by the
    // row factory from aug.x() at the clone offset; tdcpRows() recomputes ρ_k0(ĉ_i) from it (the clone column carries
    // the k1 line of sight since AMENDMENTS A64, so u_k0(ĉ_i) is no longer needed).
    // `clone_slot` above is the column offset of that clone inside x_G (aug.find(Slot::Clone, pair_id));
    // −1 ⇒ the clone columns are omitted (fixed-anchor ablation, A-FA).
    Eigen::Vector3d c_hat_N = Eigen::Vector3d::Zero();
    // Integer-millisecond receiver clock jump k·c·1 ms detected by L2 inside (k0, k1] (DESIGN §10 L2,
    // AMENDMENTS A9-2): removed from λΔΦ so the pair stays usable; 0 when no jump.
    double clock_jump_m = 0.0;
    // Appended (AMENDMENTS A61 F2; INTERFACE_CHANGES): every member's troposphere and ionosphere model delays at k0 — the
    // pseudorange rows' Saastamoinen/Klobuchar at the k0 antenna (SatGeometry::trop_m / ion_m of the k0 epoch).
    // tdcpRows() adds their change over the pair, (T_k1 − T_k0) − (I_k1 − I_k0), to h: the carrier carries +T − I, and
    // over a 0.5 s pair the change reaches millimetres at low elevation, systematically (U15: −0.2 m vertical in 4 min
    // when left out). A NaN ionosphere entry (no Klobuchar at k0) drops only the ionosphere part; empty ⇒ not modelled.
    std::vector<double> trop_k0_m, ion_k0_m;
    // Appended (AMENDMENTS A61 F11; INTERFACE_CHANGES): per member, the column (offset inside x_G) of the phase-noise state
    // of its k0 / k1 epoch (Slot::Phase), −1 when that end's σ²_Φ is in R instead. The row predicts ĥ + η̂_k1 − η̂_k0 and
    // carries +1 / −1 in those columns. Empty ⇒ none.
    std::vector<int> eta_k0_col, eta_k1_col;
  };
  void tdcpRows(const TdcpPairInput& pair, const std::vector<SatGeometry>& geo_k1, const LinPoint& lp,
                const AntennaJacobians& J, RowSet* out) const;
  // Householder projection removing the common Δclk (§4.5): whitened W = R^{−1/2}, o = W·1, Qᵀo = ‖o‖e_1,
  // keep rows 2..m. Returns the (m−1)-row set with unit variances; `robust_dclk` is the weighted median
  // estimate used for the per-satellite gate *before* projection (§10 L4).
  static bool projectOutCommonClock(const RowSet& in, RowSet* out, double* robust_dclk);
  // L-PVT rows (§4.6); none for a solution that did not take its GPST second (A43, offerPvtEpoch()).
  void pvtRows(const ReceiverPvt& pvt, const LinPoint& lp, const AntennaJacobians& J, double dt_since_last_s,
               RowSet* pos_out, RowSet* vel_out) const;

  // Clone creation Jacobian (§2.4 stopped-state augmentation): c = a^N(x̂_k0) ⇒ F_host = J_a host part,
  // F_aug = J_a aug part, N = M Σ_D,k0 Mᵀ (bridge noise at the k0 end, §4.7).
  static void cloneJacobians(const AntennaJacobians& J, const Eigen::Matrix3d& Sigma_D_k0, Eigen::MatrixXd* F_host,
                             Eigen::MatrixXd* F_aug, Eigen::MatrixXd* N);

  // ---- appended by P3-C (AMENDMENTS A43; INTERFACE_CHANGES) ----
  // The 1 Hz selection is applied by pseudorangeRows() (bins of cfg.noise.psr.decim_s over ObsEpoch::t_rx) and pvtRows()
  // (cfg.noise.pvt.decim_s over ReceiverPvt::t) themselves: they build rows only for the epoch that took its bin, and
  // every later call for that same epoch (IEKF relinearisation) keeps the decision. A43 selects the first epoch to
  // ARRIVE in the bin: a caller that builds rows for only some epochs (initialisation window, outage) offers every
  // arriving epoch here once, in arrival order; an epoch never offered is offered by its first row build. The state
  // lives in this object: one RowBuilder per fusion instance (a fresh one has forgotten every bin and takes any epoch).
  EpochDecimator::Verdict offerPsrEpoch(const GTime& t_rx) const;
  EpochDecimator::Verdict offerPvtEpoch(const GTime& t) const;
  const EpochDecimator& psrDecimation() const { return psr_decim_; }
  const EpochDecimator& pvtDecimation() const { return pvt_decim_; }

 private:
  const Config& cfg_;
  const NoiseModel& noise_;
  // Appended by P3-C (A43): decimation state behind the const row builders (single-threaded core, DESIGN §12.1).
  mutable EpochDecimator psr_decim_;
  mutable EpochDecimator pvt_decim_;
};

}  // namespace livo_gnss
