// SPDX-License-Identifier: BSD-3-Clause
// AugBlock (DESIGN §2.2–§2.4, §3.2; AMENDMENTS A11). Owner: WP-b. Tests: U8, U9.
#include "internal/aug_block.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>

#include "internal/clock.hpp"

namespace livo_gnss {

const char* slotName(Slot s) {
  switch (s) {
    case Slot::Tilt: return "tilt";
    case Slot::Yaw: return "yaw";
    case Slot::Trans: return "trans";
    case Slot::Clk: return "clk";
    case Slot::Isb: return "isb";
    case Slot::Clone: return "clone";
    case Slot::Td: return "td";
    case Slot::Phase: return "phase";
    case Slot::Origin: return "origin";
    default: return "?";
  }
}

AugBlock::AugBlock(const Config& cfg) : cfg_(cfg), x_(0), Q_(0, 0), C_(HostIdx::dim, 0) {}

int AugBlock::find(Slot kind, int tag) const {
  for (const auto& s : slots_)
    if (s.kind == kind && s.tag == tag) return s.offset;
  return -1;
}

int AugBlock::addSlot(Slot kind, int tag, const std::string& name, const Eigen::VectorXd& x0, const Eigen::MatrixXd& F_host,
                      const Eigen::MatrixXd& F_aug, const Eigen::MatrixXd& N, const HostMat& P_host) {
  const int n = dim();
  const int d = static_cast<int>(x0.size());
  if (d <= 0) throw std::invalid_argument("AugBlock::addSlot: empty slot");
  if (has(kind, tag)) throw std::invalid_argument("AugBlock::addSlot: slot already present: " + name);
  if (F_host.rows() != d || F_host.cols() != HostIdx::dim) throw std::invalid_argument("AugBlock::addSlot: F_host must be dim×19");
  const bool aug_given = F_aug.size() > 0;
  if (aug_given && (F_aug.rows() != d || F_aug.cols() != n)) throw std::invalid_argument("AugBlock::addSlot: F_aug must be dim×n");
  if (N.rows() != d || N.cols() != d) throw std::invalid_argument("AugBlock::addSlot: N must be dim×dim");

  // Consistent augmentation y = f(x_host, x_G, n) (§2.4): with Σ = [[P, C], [Cᵀ, Q]],
  //   Σ_yh = F_host P + F_aug Cᵀ,  Σ_ya = F_host C + F_aug Q,
  //   Σ_yy = F_host P F_hostᵀ + F_host C F_augᵀ + F_aug Cᵀ F_hostᵀ + F_aug Q F_augᵀ + N.
  Eigen::MatrixXd S_yh = F_host * P_host;                                   // d×19
  Eigen::MatrixXd S_ya = F_host * C_;                                       // d×n
  Eigen::MatrixXd S_yy = F_host * P_host * F_host.transpose() + N;          // d×d
  if (aug_given && n > 0) {
    const Eigen::MatrixXd FC = F_host * C_ * F_aug.transpose();
    S_yh += F_aug * C_.transpose();
    S_ya += F_aug * Q_;
    S_yy += FC + FC.transpose() + F_aug * Q_ * F_aug.transpose();
  }
  S_yy = 0.5 * (S_yy + S_yy.transpose()).eval();   // .eval(): no in-place transpose aliasing

  // Existing entries are copied block-wise (bit-unchanged, U9); the new slot is appended at the end.
  Eigen::VectorXd x(n + d);
  x.head(n) = x_;
  x.tail(d) = x0;
  Eigen::Matrix<double, HostIdx::dim, Eigen::Dynamic> C(HostIdx::dim, n + d);
  C.leftCols(n) = C_;
  C.rightCols(d) = S_yh.transpose();
  Eigen::MatrixXd Q(n + d, n + d);
  Q.topLeftCorner(n, n) = Q_;
  Q.topRightCorner(n, d) = S_ya.transpose();
  Q.bottomLeftCorner(d, n) = S_ya;
  Q.bottomRightCorner(d, d) = S_yy;
  x_.swap(x);
  C_.swap(C);
  Q_.swap(Q);

  SlotInfo info;
  info.kind = kind;
  info.tag = tag;
  info.offset = n;
  info.dim = d;
  info.name = name;
  slots_.push_back(info);
  return static_cast<int>(slots_.size()) - 1;
}

void AugBlock::removeSlot(int slot_index) {
  if (slot_index < 0 || slot_index >= static_cast<int>(slots_.size())) throw std::out_of_range("AugBlock::removeSlot: bad slot index");
  const int n = dim();
  const int o = slots_[slot_index].offset;
  const int d = slots_[slot_index].dim;
  const int r = n - o - d;   // columns after the slot
  // Deleting rows/columns is the covariance-form marginalisation (§2.4). Block copies keep every other entry bit-identical.
  Eigen::VectorXd x(n - d);
  x.head(o) = x_.head(o);
  x.tail(r) = x_.tail(r);
  Eigen::Matrix<double, HostIdx::dim, Eigen::Dynamic> C(HostIdx::dim, n - d);
  C.leftCols(o) = C_.leftCols(o);
  C.rightCols(r) = C_.rightCols(r);
  Eigen::MatrixXd Q(n - d, n - d);
  Q.topLeftCorner(o, o) = Q_.topLeftCorner(o, o);
  Q.topRightCorner(o, r) = Q_.topRightCorner(o, r);
  Q.bottomLeftCorner(r, o) = Q_.bottomLeftCorner(r, o);
  Q.bottomRightCorner(r, r) = Q_.bottomRightCorner(r, r);
  x_.swap(x);
  C_.swap(C);
  Q_.swap(Q);
  slots_.erase(slots_.begin() + slot_index);
  for (auto& s : slots_)
    if (s.offset > o) s.offset -= d;
}

void AugBlock::clear() {
  slots_.clear();
  x_.resize(0);
  Q_.resize(0, 0);
  C_.resize(HostIdx::dim, 0);
}

void AugBlock::propagate(const HostMat& F_tot, const Eigen::MatrixXd& Phi_G, const Eigen::MatrixXd& Q_proc) {
  const int n = dim();
  if (n == 0) return;
  if (Phi_G.rows() != n || Phi_G.cols() != n || Q_proc.rows() != n || Q_proc.cols() != n)
    throw std::invalid_argument("AugBlock::propagate: Phi_G/Q_proc must be n×n");
  // H1 (§2.4): C ← F_tot C Φ_Gᵀ, Q_G ← Φ_G Q_G Φ_Gᵀ + Q_G(Δs, Δα, Δt), x_G ← Φ_G x_G. F is block-diagonal over
  // [host | aug] and the noises are independent, so this equals the joint propagation (U8).
  C_ = F_tot * C_ * Phi_G.transpose();
  Q_ = Phi_G * Q_ * Phi_G.transpose() + Q_proc;
  x_ = Phi_G * x_;
  symmetrize();
}

void AugBlock::processModel(double ds, double dalpha, double dt, const Eigen::Matrix3d& M, const Eigen::Vector3d& a_W,
                            const Eigen::Vector3d& pivot_W, double lambda_windup_m, double omega_z_abs_dt, Eigen::MatrixXd* Phi_G,
                            Eigen::MatrixXd* Q_proc) const {
  const int n = dim();
  Phi_G->setIdentity(n, n);
  Q_proc->setZero(n, n);
  if (n == 0) return;
  ds = std::max(ds, 0.0);
  dalpha = std::max(dalpha, 0.0);
  const double dtp = std::max(dt, 0.0);
  const Config::Process& pr = cfg_.process;

  // Registration random walk by path length about the current pivot (§3.2, D1 §3.5):
  //   Δr = R_z(ψ) J_l(φ̃) E_xy w_φ + e_z w_ψ,  Δt = [u]× Δr + w_t0,  u = M(a^W − o^W)
  //   Q_ξ = J_rw diag(q_φΔs, q_φΔs, q_ψΔs + q_ψrΔα, q_thΔs, q_thΔs, q_tvΔs) J_rwᵀ
  // so the current global point m = u + t is left untouched by the rotation noise (U9). The tilt is additive and
  // Exp(φ̃ + w̃_φ) = Exp(J_l(φ̃) w̃_φ) Exp(φ̃), so R_z J_l(φ̃) E_xy w_φ is the rotation increment of M = R_z Exp(φ̃) C that
  // an additive w_φ causes (AMENDMENTS A45 / R6, the same exact column as A34's re-pivot and the J_a/J_w tilt column;
  // §3.2's R_z E_xy is its φ = 0 form, whose residue (½|φ||u|)² q_φΔs would reach the current point). J_l is
  // Leveling::tiltLeftJacobian, the one M()/repivot use. q_time·Δt (default 0) is a per-second term on the
  // translation only.
  const int i_tilt = find(Slot::Tilt), i_yaw = find(Slot::Yaw), i_trans = find(Slot::Trans);
  if (i_tilt >= 0 || i_yaw >= 0 || i_trans >= 0) {
    const double psi = (i_yaw >= 0) ? x_(i_yaw) : 0.0;
    const Eigen::Vector2d tilt = (i_tilt >= 0) ? Eigen::Vector2d(x_.segment<2>(i_tilt)) : Eigen::Vector2d::Zero();
    const Eigen::Matrix3d U = skew(Eigen::Vector3d(M * (a_W - pivot_W)));
    const Eigen::Matrix3d RzJl = Leveling::Rz(psi) * Leveling::tiltLeftJacobian(tilt);
    Eigen::Matrix<double, 6, 6> J = Eigen::Matrix<double, 6, 6>::Identity();   // rows/cols [φ(2) ψ t(3)] / [w_φ w_ψ w_t0]
    J.block<3, 2>(3, 0) = U * RzJl.leftCols<2>();
    J.block<3, 1>(3, 2) = U.col(2);
    Eigen::Matrix<double, 6, 1> q;
    const double qt = pr.frame.q_time * dtp;
    q << pr.frame.q_tilt_per_m * ds, pr.frame.q_tilt_per_m * ds, pr.frame.q_yaw_per_m * ds + pr.frame.q_yaw_per_rad * dalpha,
        pr.frame.q_th_per_m * ds + qt, pr.frame.q_th_per_m * ds + qt, pr.frame.q_tv_per_m * ds + qt;
    // A noise source whose state is not in the filter does not exist: without a tilt/yaw slot the w_φ/w_ψ columns are
    // dropped, otherwise their [u]× coupling would put rotation noise into t alone (the design activates all three
    // slots together, §8; this only guards a partial block).
    if (i_tilt < 0) q(0) = q(1) = 0.0;
    if (i_yaw < 0) q(2) = 0.0;
    const Eigen::Matrix<double, 6, 6> Qxi = J * q.asDiagonal() * J.transpose();
    const int col[6] = {i_tilt < 0 ? -1 : i_tilt, i_tilt < 0 ? -1 : i_tilt + 1, i_yaw, i_trans < 0 ? -1 : i_trans,
                        i_trans < 0 ? -1 : i_trans + 1, i_trans < 0 ? -1 : i_trans + 2};
    for (int a = 0; a < 6; ++a) {
      if (col[a] < 0) continue;
      for (int b = 0; b < 6; ++b)
        if (col[b] >= 0) (*Q_proc)(col[a], col[b]) = Qxi(a, b);
    }
  }
  // Receiver clock (§3.1): exact two-state discretisation with the wind-up lower bound on S_b (bound over Δt_epoch,
  // clock.hpp; the step's mean |ω_z| = Σ|ω_z|dt / Δt).
  const int i_clk = find(Slot::Clk);
  if (i_clk >= 0) {
    ClockModel cm(pr.clock);
    cm.windup_epoch_dt_s = windup_epoch_dt_s_;
    const double wz = (dtp > 0.0) ? omega_z_abs_dt / dtp : 0.0;
    Phi_G->block<2, 2>(i_clk, i_clk) = cm.Phi(dtp);
    Q_proc->block<2, 2>(i_clk, i_clk) = cm.Q(dtp, wz, lambda_windup_m);
  }
  // ISB and td: random walks in time; clones, phase-noise and origin slots (A61 F11/F13): stopped, no noise (§3.3).
  for (const auto& s : slots_) {
    if (s.kind == Slot::Isb)
      for (int k = 0; k < s.dim; ++k) (*Q_proc)(s.offset + k, s.offset + k) += pr.isb_q * dtp;
    else if (s.kind == Slot::Td)
      for (int k = 0; k < s.dim; ++k) (*Q_proc)(s.offset + k, s.offset + k) += cfg_.time.td_rw_s2_per_s * dtp;
  }
}

bool AugBlock::conditionOnHostUpdate(const HostMat& P_prior, const HostMat& P_post, const HostVec& dx, bool* used_pseudoinverse) {
  if (used_pseudoinverse) *used_pseudoinverse = false;
  const int n = dim();
  if (n == 0) return true;
  // A = Cᵀ(P⁻)⁻¹, computed as Aᵀ = (P⁻)⁻¹ C through a Jacobi-scaled LDLT (P⁻ mixes rad², m², (m/s)², bias units;
  // scaling by 1/√diag keeps the pivots O(1)). With D = diag(1/√P⁻_ii): (P⁻)⁻¹ C = D (D P⁻ D)⁻¹ (D C).
  HostVec dinv;
  for (int i = 0; i < HostIdx::dim; ++i) {
    const double p = P_prior(i, i);
    dinv(i) = (std::isfinite(p) && p > 0.0) ? 1.0 / std::sqrt(p) : 1.0;
  }
  const HostMat Ps = dinv.asDiagonal() * P_prior * dinv.asDiagonal();
  const Eigen::MatrixXd B = dinv.asDiagonal() * C_;   // 19×n
  Eigen::MatrixXd Y;                                  // (D P⁻ D)⁻¹ B
  bool ok = false;
  {
    Eigen::LDLT<HostMat> ldlt(0.5 * (Ps + Ps.transpose()));
    if (ldlt.info() == Eigen::Success && ldlt.isPositive()) {
      Y = ldlt.solve(B);
      const double bn = B.cwiseAbs().maxCoeff();
      const double res = (Ps * Y - B).cwiseAbs().maxCoeff();
      ok = Y.allFinite() && (bn <= 0.0 ? res == 0.0 : res <= 1e-8 * bn);
    }
  }
  if (!ok) {
    // Fallback (§2.4): eigen-decomposition pseudo-inverse of the scaled prior. Directions with (numerically) zero
    // prior variance carry no cross-covariance either (range(C) ⊆ range(P⁻) for a PSD joint), so this is the exact
    // conditioning on the singular prior. The caller records event PSEUDOINVERSE.
    Eigen::SelfAdjointEigenSolver<HostMat> es(0.5 * (Ps + Ps.transpose()));
    if (es.info() != Eigen::Success) return false;
    const HostVec lam = es.eigenvalues();
    const double lmax = lam.maxCoeff();
    HostVec inv = HostVec::Zero();
    for (int i = 0; i < HostIdx::dim; ++i) inv(i) = (lam(i) > 1e-12 * lmax && lam(i) > 0.0) ? 1.0 / lam(i) : 0.0;
    Y = es.eigenvectors() * inv.asDiagonal() * es.eigenvectors().transpose() * B;
    if (!Y.allFinite()) return false;
    if (used_pseudoinverse) *used_pseudoinverse = true;
  }
  const Eigen::MatrixXd At = dinv.asDiagonal() * Y;   // Aᵀ, 19×n
  // H2 (§2.4): x_G ← x_G + A(x⁺ ⊟ x⁻), C ← P⁺Aᵀ, Q_G ← Q_G − A(P⁻ − P⁺)Aᵀ. Exact for any host gain K because the
  // joint gain has the structure [K; AK] (K_G = P_GH (P⁻)⁻¹ K_H, independent of H_i); Joseph or (I−G)P⁻ alike (U8).
  // "Exact" refers to the error-state form x⁻ ⊞ Σ_i K_i(...): the upstream x_i ⊞ (Kν + vec − G·vec) writing differs
  // from it by the second-order BCH term of the SO(3) retraction (U8 quantifies it: 7e-6 at an exaggerated 0.09 rad
  // step, negligible at real update sizes).
  x_ += At.transpose() * dx;
  const HostMat dP = P_prior - P_post;
  Q_ -= At.transpose() * dP * At;
  C_ = P_post * At;
  symmetrize();
  wrapYaw();
  return true;
}

Eigen::MatrixXd AugBlock::repivot(const Eigen::Vector3d& pivot_old, const Eigen::Vector3d& a_W, const Leveling& leveling,
                                  const Eigen::Vector3d& g_W, const HostMat& P_host, bool gravity_est_en) {
  const int n = dim();
  const int i_trans = find(Slot::Trans);
  const int i_yaw = find(Slot::Yaw), i_tilt = find(Slot::Tilt);
  // T over [host 19 | aug n]: identity except the t rows (A11). Returned restricted to the aug rows.
  Eigen::MatrixXd T(n, HostIdx::dim + n);
  T.setZero();
  T.rightCols(n).setIdentity();
  if (i_trans < 0) return T;
  const double psi = (i_yaw >= 0) ? x_(i_yaw) : 0.0;
  const Eigen::Vector2d tilt = (i_tilt >= 0) ? Eigen::Vector2d(x_.segment<2>(i_tilt)) : Eigen::Vector2d::Zero();
  const Eigen::Vector3d d = a_W - pivot_old;                                   // o' − o
  const Eigen::Matrix3d Rz = Leveling::Rz(psi);
  const Eigen::Matrix3d ExpPhi = Leveling::expTilt(tilt);   // the Exp of Leveling::M() (no small-angle cut-off)
  const Eigen::Matrix3d Cg = leveling.C(g_W);
  const Eigen::Matrix3d M = Rz * ExpPhi * Cg;
  const Eigen::Vector3d Md = M * d;
  const Eigen::Matrix3d Xi = -Rz * skew(Eigen::Vector3d(ExpPhi * Cg * d));   // Ξ(o') = −R_z(ψ)[Exp(φ̃) C(g) d]×
  // The t rows are the exact derivatives of t' = t + M(ψ, φ, g) d (AMENDMENTS A34; A11 had their φ = 0 forms):
  //   ∂t'/∂δψ = [e_z]× M d                      R_z(ψ + δψ) = Exp(e_z δψ) R_z(ψ)
  //   ∂t'/∂δφ = Ξ(o') J_l(φ̃) E_xy              Exp(φ̃ + δφ̃) = Exp(J_l(φ̃) δφ̃) Exp(φ̃)
  //   ∂t'/∂δg = Ξ(o') Exp(φ̃) [e_z]× G_⊥        C(g + δg) = Exp(ε) C(g) ⇒ M' = R_z Exp(Exp(φ̃) ε) Exp(φ̃) C
  // (host cols 16–18, only gravity_est_en). The δg column is exact where epsJacobian is, i.e. at the g that C_ref was
  // last set from (onStepEnd refreshes it every step; rows.cpp's δg column has the same property).
  if (i_yaw >= 0) T.block<3, 1>(i_trans, HostIdx::dim + i_yaw) = Eigen::Vector3d::UnitZ().cross(Md);
  if (i_tilt >= 0) {
    const Eigen::Matrix3d XiJl = Xi * Leveling::tiltLeftJacobian(tilt);
    T.block<3, 2>(i_trans, HostIdx::dim + i_tilt) = XiJl.leftCols<2>();
  }
  if (gravity_est_en) T.block<3, 3>(i_trans, HostIdx::grav) = Xi * ExpPhi * leveling.epsJacobian(g_W);

  // Σ' = T Σ Tᵀ with T = [[I, 0], [G, A]]: P unchanged, C' = P Gᵀ + C Aᵀ, Q' = G P Gᵀ + G C Aᵀ + (G C Aᵀ)ᵀ + A Q Aᵀ.
  const Eigen::MatrixXd G = T.leftCols(HostIdx::dim);   // n×19
  const Eigen::MatrixXd A = T.rightCols(n);             // n×n
  const Eigen::MatrixXd GCA = G * C_ * A.transpose();
  Eigen::MatrixXd Qn = G * P_host * G.transpose() + GCA + GCA.transpose() + A * Q_ * A.transpose();
  Eigen::Matrix<double, HostIdx::dim, Eigen::Dynamic> Cn = P_host * G.transpose() + C_ * A.transpose();
  Q_.swap(Qn);
  C_.swap(Cn);
  x_.segment<3>(i_trans) += Md;   // t' = t + M(o' − o): the global antenna a^N = M(a^W − o') + t' is unchanged
  symmetrize();
  return T;
}

void AugBlock::wrapYaw() {
  const int i = find(Slot::Yaw);
  if (i < 0) return;
  double& psi = x_(i);
  if (!std::isfinite(psi)) return;
  psi -= 2.0 * M_PI * std::floor((psi + M_PI) / (2.0 * M_PI));   // → [−π, π)
  if (psi <= -M_PI) psi += 2.0 * M_PI;                              // → (−π, π]
}

void AugBlock::symmetrize() {
  if (Q_.size() == 0) return;
  Q_ = 0.5 * (Q_ + Q_.transpose()).eval();
}

Eigen::MatrixXd AugBlock::jointCov(const HostMat& P_host) const {
  const int n = dim();
  Eigen::MatrixXd S(HostIdx::dim + n, HostIdx::dim + n);
  S.topLeftCorner<HostIdx::dim, HostIdx::dim>() = P_host;
  S.topRightCorner(HostIdx::dim, n) = C_;
  S.bottomLeftCorner(n, HostIdx::dim) = C_.transpose();
  S.bottomRightCorner(n, n) = Q_;
  return S;
}

void AugBlock::setFromJoint(const Eigen::MatrixXd& Sigma, HostMat* P_host) {
  const int n = dim();
  if (Sigma.rows() != HostIdx::dim + n || Sigma.cols() != HostIdx::dim + n)
    throw std::invalid_argument("AugBlock::setFromJoint: Sigma must be (19+n)×(19+n)");
  if (P_host) *P_host = Sigma.topLeftCorner<HostIdx::dim, HostIdx::dim>();
  C_ = Sigma.topRightCorner(HostIdx::dim, n);
  Q_ = Sigma.bottomRightCorner(n, n);
}

Eigen::Vector2d AugBlock::tilt() const {
  const int i = find(Slot::Tilt);
  return i < 0 ? Eigen::Vector2d::Zero() : Eigen::Vector2d(x_.segment<2>(i));
}
double AugBlock::yaw() const {
  const int i = find(Slot::Yaw);
  return i < 0 ? 0.0 : x_(i);
}
Eigen::Vector3d AugBlock::trans() const {
  const int i = find(Slot::Trans);
  return i < 0 ? Eigen::Vector3d::Zero() : Eigen::Vector3d(x_.segment<3>(i));
}
double AugBlock::clockBias() const {
  const int i = find(Slot::Clk);
  return i < 0 ? 0.0 : x_(i);
}
double AugBlock::clockDrift() const {
  const int i = find(Slot::Clk);
  return i < 0 ? 0.0 : x_(i + 1);
}
bool AugBlock::isb(Sys sys, double* value, double* sigma) const {
  // Slot tag = sysBit(sys). GPS (and QZSS folded into it) has no slot: ι_G ≡ 0 ⇒ false, value 0.
  const int i = find(Slot::Isb, static_cast<int>(sysBit(sys)));
  if (value) *value = (i < 0) ? 0.0 : x_(i);
  if (sigma) *sigma = (i < 0) ? 0.0 : std::sqrt(std::max(Q_(i, i), 0.0));
  return i >= 0;
}

}  // namespace livo_gnss
