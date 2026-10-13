// SPDX-License-Identifier: BSD-3-Clause
// The fixed anchor of a FAST-LIVGO TDCP pair under livgo/frame_exact (AMENDMENTS A75 §6). Owner: GnssFusionLivgo
// (implementation in livgo_pin.cpp, which is only linked into the LIVGO arm's object code path).
//
// A fixed-anchor TDCP row is ν = Δρ_meas − [ρ_k1(a^N(x)) − ρ_k0(ĉ)] with ĉ a number: the N-frame antenna position at the
// k0 epoch, taken after k0's update. Its Jacobian is −u₁ᵀJ_a, J_a the k1 antenna's. That is right for the host columns (the
// anchor is deliberately not tied to the host: the row measures where the antenna is relative to a point that was just
// estimated). It is wrong for a state that the anchor and the k1 antenna *share*: the registration (ψ, φ, t) and the host
// gravity direction. The anchor was computed with the same ψ̂, φ̂, t̂, ĝ; an error in any of them moves ĉ and a^N(x) alike
// and leaves ρ_k1 − ρ_k0 untouched. J_a nevertheless carries ∂a^N/∂ψ = [e_z]×(a^N − t) — a lever arm of up to
// frames.pivot_distance_m (100 m) times σ_ψ (0.5°: 0.9 m), and ∂a^N/∂δg ~ lever × σ_g/g (~1.5 cm at 50 m) — into S, orders of
// magnitude above a millimetre-level carrier row. For a translation that is the dead row of AMENDMENTS A75 §2 and frame_exact
// removes it with a zero variance; for the yaw (which frame_exact keeps estimating) and the gravity (a host state) the same
// treatment is a change of the row: the anchor is stored as the k0 antenna in W — the host state and bridge of the update
// at k0 — and mapped through the *iterate's* registration and gravity,
//     ĉ(x) = a^N(x_k0 ; ψ(x), φ(x), t(x), g(x)),
// so h is consistent at every iterate, and the row's columns for those states are the *relative* ones
//     −u₁ᵀ(J_a − J_c)|_{ψ, φ, t, δg} ,   e.g.  ∂(a^N_k1 − ĉ)/∂ψ = [e_z]×(a^N_k1 − ĉ)  (the displacement over the pair, mm).
// That is exactly what the clone of arm T gives those columns (the clone is correlated with them), without a clone slot.
#pragma once

#include <vector>

#include <Eigen/Core>

#include "livo_gnss/core_state.hpp"
#include "aug_block.hpp"
#include "bridge.hpp"
#include "rows.hpp"

namespace livo_gnss {

// What a pair keeps of k0: the host posterior of k0's update and the bridge of the k0 epoch (both fixed-size, cheap).
struct LivgoAnchor {
  bool valid = false;
  CoreState core;
  BridgeResult br;
};

// The k0 antenna at the iterate `lp`: lp.aug supplies the registration (ψ, φ, t), lp.x->g the gravity (the iterate's —
// the anchor shares it); the host state, the bridge and everything else are the anchor's. Returns a_N = ĉ(x) and the
// Jacobian J_c over the same columns as the row's J_a (host columns are those of the *k0* state and are not used).
AntennaJacobians livgoAnchorAt(const RowBuilder& rows, const LinPoint& lp, const LivgoAnchor& anchor);

// The columns of x_G that the k1 antenna and its anchor share: tilt, yaw, translation (when present) and, with
// `gravity`, the host gravity columns HostIdx::grav … +2 (rows.cpp adds them under lp.gravity_est_en &&
// frames.gnss_updates_gravity).
std::vector<int> livgoSharedAnchorColumns(const AugBlock& aug, bool gravity);

// H ← H + u₁ᵀ J_c over `cols`: −u₁ᵀJ_a becomes −u₁ᵀ(J_a − J_c) on the shared columns, the others are untouched.
void livgoShareAnchorColumns(Eigen::RowVectorXd* H, const Eigen::Vector3d& u1, const AntennaJacobians& Jc, const std::vector<int>& cols);

}  // namespace livo_gnss
