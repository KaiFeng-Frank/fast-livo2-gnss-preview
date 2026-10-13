// SPDX-License-Identifier: BSD-3-Clause
// The TDCP share of the host position variance of one epoch (AMENDMENTS A75 revision 1, prediction P2). Header only; owner:
// GnssFusionLivgo.
//
// Question: do the TDCP rows of an epoch move the host position uncertainty at all? In the legacy arm they could not (the
// translation variance absorbs the rows, gain ~1e-7), and the registered way to tell "the rows are alive" from "dead" has to
// compare the *same epoch* with and without them: a comparison between epochs with and without TDCP rows is confounded by
// satellite geometry, by the LIO/VIO updates of the same step (filter_cov.csv has no row between the VIO posterior and the
// GNSS update in the post_vio slot), and, with 10 Hz chained pairs, has hardly any epoch without rows to compare with.
//
// The counterfactual is the epoch the filter would have made without the TDCP rows: the same prior (the state before the
// epoch's update), the same options, the same gates and iteration, the row factory without the TDCP set. It runs on copies;
// the facade's state, the filter's output and the order of its calculations do not depend on it. What is compared is the
// trace of the host position block of the posterior covariance (frame-invariant; HostIdx::pos), m².
//
//     g = 1 − tr P⁺_pos(all rows) / tr P⁺_pos(without the TDCP rows)         ∈ [0, 1)
//
// the fraction of the position variance that is left after Doppler (and whatever else the epoch carries) which the TDCP rows
// remove in addition.
#pragma once

#include "livo_gnss/core_state.hpp"
#include "aug_block.hpp"
#include "update.hpp"

namespace livo_gnss {

// tr of the host position block of P, m².
inline double livgoPosTrace(const HostMat& P) { return P.block<3, 3>(HostIdx::pos, HostIdx::pos).trace(); }

// The posterior position trace of the epoch without the TDCP rows. (core0, aug0) are the state before the epoch's update;
// `rows_without_tdcp` builds the epoch's row sets except the TDCP one. false: the counterfactual update rolled back (no number).
// With no row left the update commits nothing and the trace is the prior's — the TDCP rows were the whole epoch.
inline bool livgoCounterfactualPosTrace(const JointUpdater& updater, const CoreState& core0, const AugBlock& aug0,
                                        const JointUpdater::RowFactory& rows_without_tdcp, const JointUpdater::Options& opt,
                                        double* tr_pos) {
  CoreState core = core0;
  AugBlock aug(aug0);   // refers to the same Config; a copy of the slots, mean, Q, C
  const JointUpdater::Result r = updater.update(core, aug, rows_without_tdcp, opt);
  if (r.rolled_back) return false;
  if (tr_pos) *tr_pos = livgoPosTrace(core.P);
  return true;
}

}  // namespace livo_gnss
