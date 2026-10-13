// SPDX-License-Identifier: BSD-3-Clause
// FAST-LIVGO helpers that must not live in the T-arm translation units (AugBlock::processModel, TimeMap).
#include "internal/aug_block.hpp"
#include "internal/livgo_anchor.hpp"
#include "internal/time_map.hpp"

namespace livo_gnss {

void AugBlock::pinRegistration(const Eigen::VectorXd& x_pin) {
  const Slot kinds[3] = {Slot::Tilt, Slot::Yaw, Slot::Trans};
  for (Slot k : kinds) {
    const int off = find(k, 0);
    if (off < 0) continue;
    int dim = 0;
    for (const SlotInfo& s : slots_)
      if (s.kind == k && s.tag == 0) dim = s.dim;
    if (dim <= 0 || off + dim > x_.size() || off + dim > x_pin.size()) continue;
    x_.segment(off, dim) = x_pin.segment(off, dim);
    if (C_.cols() >= off + dim) C_.middleCols(off, dim).setZero();
    if (Q_.rows() >= off + dim) {
      // Marginal variance stays (the activation uncertainty). Cross terms go, so H2 cannot walk the mean.
      const Eigen::VectorXd diag = Q_.diagonal().segment(off, dim);
      Q_.middleRows(off, dim).setZero();
      Q_.middleCols(off, dim).setZero();
      Q_.diagonal().segment(off, dim) = diag;
    }
  }
  symmetrize();
}

void AugBlock::pinRegistrationExact(const Eigen::VectorXd& x_pin, bool include_yaw) {
  const Slot kinds[3] = {Slot::Tilt, Slot::Trans, Slot::Yaw};
  const int n_kinds = include_yaw ? 3 : 2;   // the yaw slot only with include_yaw (A80 freeze_yaw)
  for (int ki = 0; ki < n_kinds; ++ki) {
    const Slot k = kinds[ki];
    const int off = find(k, 0);
    if (off < 0) continue;
    int dim = 0;
    for (const SlotInfo& s : slots_)
      if (s.kind == k && s.tag == 0) dim = s.dim;
    if (dim <= 0 || off + dim > x_.size() || off + dim > x_pin.size()) continue;
    x_.segment(off, dim) = x_pin.segment(off, dim);
    if (C_.cols() >= off + dim) C_.middleCols(off, dim).setZero();
    if (Q_.rows() >= off + dim) {
      // Deleting rows and columns is a marginalisation, so the rest stays positive semidefinite; the block itself is an
      // exact zero (no marginal variance either).
      Q_.middleRows(off, dim).setZero();
      Q_.middleCols(off, dim).setZero();
    }
  }
  symmetrize();
}

void TimeMap::addOffset(double delta_s) {
  for (Segment& s : segments_) s.offset_s += delta_s;
}

// ---- livgo/frame_exact: the fixed anchor shares the registration and the gravity with the k1 antenna (livgo_anchor.hpp) ----
AntennaJacobians livgoAnchorAt(const RowBuilder& rows, const LinPoint& lp, const LivgoAnchor& anchor) {
  CoreState xc = anchor.core;
  if (lp.x) xc.g = lp.x->g;   // the host gravity is one state: the anchor takes the iterate's value
  LinPoint a = lp;
  a.x = &xc;
  a.bridge = &anchor.br;
  a.delta = anchor.br.delta;
  return rows.antennaJacobians(a);
}

std::vector<int> livgoSharedAnchorColumns(const AugBlock& aug, bool gravity) {
  std::vector<int> cols;
  for (const SlotInfo& s : aug.slots()) {
    if (s.tag != 0) continue;
    if (s.kind == Slot::Tilt || s.kind == Slot::Yaw || s.kind == Slot::Trans)
      for (int j = 0; j < s.dim; ++j) cols.push_back(HostIdx::dim + s.offset + j);
  }
  if (gravity)
    for (int j = 0; j < 3; ++j) cols.push_back(HostIdx::grav + j);
  return cols;
}

void livgoShareAnchorColumns(Eigen::RowVectorXd* H, const Eigen::Vector3d& u1, const AntennaJacobians& Jc, const std::vector<int>& cols) {
  for (int c : cols) {
    if (c < 0 || c >= H->size() || c >= Jc.J_a.cols()) continue;
    (*H)(c) += u1.dot(Jc.J_a.col(c));
  }
}

}  // namespace livo_gnss
