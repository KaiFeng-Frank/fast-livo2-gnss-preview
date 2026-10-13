// SPDX-License-Identifier: BSD-3-Clause
// JointUpdater (DESIGN §4.8, §10 L4/L5). Owner: WP-c. Tests: U10, U12.
//
// Joint (19+n)-dimensional covariance-form IEKF over Σ = [[P, C], [Cᵀ, Q_G]]. Σ is never inverted:
// S = HΣHᵀ + R goes through LDLT, K = ΣHᵀS⁻¹. Gates (L4) use only the first-iterate innovation r_0 and
// S_0; the update is Joseph-form and symmetrised; the posterior check (L5) rolls back to a bit-exact
// snapshot. Nothing in this file scales a residual or inflates R (DESIGN §0-4, R1-3/R1-4); the A70 projection maps rows
// by a fixed linear transform T with their exact covariance T R Tᵀ = I, as the TDCP Δclk projection does.
#include "internal/update.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <Eigen/Cholesky>
#include <Eigen/Dense>
#include <Eigen/QR>
#include <Eigen/SVD>

namespace livo_gnss {

// ---------------------------------------------------------------------------------------------
// χ² / normal quantiles (exact to ~1e-12; the Wilson–Hilferty form is only the Newton start)
// ---------------------------------------------------------------------------------------------
namespace {

// Regularised lower incomplete gamma P(a, x) (Numerical Recipes gser/gcf).
double gammaP(double a, double x) {
  if (!(x > 0.0)) return 0.0;
  const double gln = std::lgamma(a);
  if (x < a + 1.0) {
    double ap = a, sum = 1.0 / a, del = sum;
    for (int n = 0; n < 1000; ++n) {
      ap += 1.0;
      del *= x / ap;
      sum += del;
      if (std::fabs(del) < std::fabs(sum) * 1e-16) break;
    }
    return sum * std::exp(-x + a * std::log(x) - gln);
  }
  // continued fraction (modified Lentz)
  const double fpmin = 1e-300;
  double b = x + 1.0 - a, c = 1.0 / fpmin, d = 1.0 / b, h = d;
  for (int i = 1; i < 1000; ++i) {
    const double an = -i * (i - a);
    b += 2.0;
    d = an * d + b;
    if (std::fabs(d) < fpmin) d = fpmin;
    c = b + an / c;
    if (std::fabs(c) < fpmin) c = fpmin;
    d = 1.0 / d;
    const double del = d * c;
    h *= del;
    if (std::fabs(del - 1.0) < 1e-16) break;
  }
  return 1.0 - std::exp(-x + a * std::log(x) - gln) * h;
}

double chi2Pdf(int k, double x) {
  const double a = 0.5 * k;
  return std::exp((a - 1.0) * std::log(x) - 0.5 * x - a * std::log(2.0) - std::lgamma(a));
}

}  // namespace

double chi2Quantile(int dof, double p) {
  if (dof < 1 || !(p > 0.0) || !(p < 1.0)) return std::numeric_limits<double>::quiet_NaN();
  const double k = static_cast<double>(dof);
  // Wilson–Hilferty start, then Newton on F(x) = P(k/2, x/2) − p with a bisection safeguard.
  const double z = normalQuantile(p);
  const double wh = 2.0 / (9.0 * k);
  double x = k * std::pow(1.0 - wh + z * std::sqrt(wh), 3.0);
  if (!(x > 0.0)) x = 0.5 * k;
  double lo = 0.0, hi = std::max(4.0 * k + 40.0 * std::sqrt(k) + 100.0, 2.0 * x);
  while (gammaP(0.5 * k, 0.5 * hi) < p) hi *= 2.0;
  for (int it = 0; it < 200; ++it) {
    const double f = gammaP(0.5 * k, 0.5 * x) - p;
    if (f > 0.0) hi = std::min(hi, x); else lo = std::max(lo, x);
    const double dfdx = chi2Pdf(dof, x);
    double xn = (dfdx > 0.0) ? x - f / dfdx : 0.5 * (lo + hi);
    if (!(xn > lo) || !(xn < hi)) xn = 0.5 * (lo + hi);
    if (std::fabs(xn - x) < 1e-13 * std::max(1.0, x)) { x = xn; break; }
    x = xn;
  }
  return x;
}

double normalQuantile(double p) {
  if (!(p > 0.0) || !(p < 1.0)) return std::numeric_limits<double>::quiet_NaN();
  // Acklam's rational approximation (|rel. err| < 1.15e-9) followed by one Halley refinement with erfc.
  static const double a[] = {-3.969683028665376e+01, 2.209460984245205e+02, -2.759285104469687e+02,
                             1.383577518672690e+02, -3.066479806614716e+01, 2.506628277459239e+00};
  static const double b[] = {-5.447609879822406e+01, 1.615858368580409e+02, -1.556989798598866e+02,
                             6.680131188771972e+01, -1.328068155288572e+01};
  static const double c[] = {-7.784894002430293e-03, -3.223964580411365e-01, -2.400758277161838e+00,
                             -2.549732539343734e+00, 4.374664141464968e+00, 2.938163982698783e+00};
  static const double d[] = {7.784695709041462e-03, 3.224671290700398e-01, 2.445134137142996e+00,
                             3.754408661907416e+00};
  const double plow = 0.02425, phigh = 1.0 - plow;
  double x;
  if (p < plow) {
    const double q = std::sqrt(-2.0 * std::log(p));
    x = (((((c[0] * q + c[1]) * q + c[2]) * q + c[3]) * q + c[4]) * q + c[5]) /
        ((((d[0] * q + d[1]) * q + d[2]) * q + d[3]) * q + 1.0);
  } else if (p <= phigh) {
    const double q = p - 0.5, r = q * q;
    x = (((((a[0] * r + a[1]) * r + a[2]) * r + a[3]) * r + a[4]) * r + a[5]) * q /
        (((((b[0] * r + b[1]) * r + b[2]) * r + b[3]) * r + b[4]) * r + 1.0);
  } else {
    const double q = std::sqrt(-2.0 * std::log(1.0 - p));
    x = -(((((c[0] * q + c[1]) * q + c[2]) * q + c[3]) * q + c[4]) * q + c[5]) /
        ((((d[0] * q + d[1]) * q + d[2]) * q + d[3]) * q + 1.0);
  }
  for (int it = 0; it < 2; ++it) {
    const double e = 0.5 * std::erfc(-x / std::sqrt(2.0)) - p;
    const double u = e * std::sqrt(2.0 * M_PI) * std::exp(0.5 * x * x);
    x = x - u / (1.0 + 0.5 * x * u);
  }
  return x;
}

// ---------------------------------------------------------------------------------------------
// helpers
// ---------------------------------------------------------------------------------------------
namespace {

double wrapPi(double a) {
  a = std::fmod(a + M_PI, 2.0 * M_PI);
  if (a < 0.0) a += 2.0 * M_PI;
  return a - M_PI;   // (−π, π]: fmod result in [0, 2π) → a − π in [−π, π); fix the −π endpoint below
}
double wrapYawValue(double a) {
  const double w = wrapPi(a);
  return (w <= -M_PI) ? w + 2.0 * M_PI : w;
}

int minMeas(const Config& cfg, MeasClass cls) {
  switch (cls) {
    case MeasClass::Psr: return cfg.rejection.min_meas.psr;
    case MeasClass::Dopp: return cfg.rejection.min_meas.dopp;
    case MeasClass::Tdcp: return cfg.rejection.min_meas.tdcp;
    default: return 1;
  }
}

// Sub-system of the accepted rows of one set (TDCP: projected first).
void appendAccepted(const RowSet& set, std::vector<Eigen::RowVectorXd>* H, std::vector<double>* r,
                    std::vector<Eigen::MatrixXd>* Rblocks, std::vector<MeasClass>* cls) {
  if (set.cls == MeasClass::Tdcp) {
    RowSet only;
    only.cls = MeasClass::Tdcp;
    std::vector<int> idx;
    for (int i = 0; i < static_cast<int>(set.rows.size()); ++i) {
      if (set.rows[i].accepted && set.rows[i].reason == RejectReason::NotRejected) idx.push_back(i);
    }
    if (idx.size() < 2) return;
    only.rows.reserve(idx.size());
    for (int i : idx) only.rows.push_back(set.rows[i]);
    if (set.has_full_R) {
      only.has_full_R = true;
      only.R_full.resize(static_cast<int>(idx.size()), static_cast<int>(idx.size()));
      for (size_t a = 0; a < idx.size(); ++a)
        for (size_t b = 0; b < idx.size(); ++b) only.R_full(static_cast<int>(a), static_cast<int>(b)) = set.R_full(idx[a], idx[b]);
    }
    RowSet proj;
    if (!RowBuilder::projectOutCommonClock(only, &proj, nullptr)) return;
    for (const auto& row : proj.rows) {
      H->push_back(row.H);
      r->push_back(row.nu);
      cls->push_back(MeasClass::Tdcp);
    }
    Rblocks->push_back(Eigen::MatrixXd::Identity(static_cast<int>(proj.rows.size()), static_cast<int>(proj.rows.size())));
    return;
  }
  std::vector<int> idx;
  for (int i = 0; i < static_cast<int>(set.rows.size()); ++i) {
    if (set.rows[i].accepted && set.rows[i].reason == RejectReason::NotRejected) idx.push_back(i);
  }
  if (idx.empty()) return;
  Eigen::MatrixXd Rb(static_cast<int>(idx.size()), static_cast<int>(idx.size()));
  for (size_t a = 0; a < idx.size(); ++a) {
    H->push_back(set.rows[idx[a]].H);
    r->push_back(set.rows[idx[a]].nu);
    cls->push_back(set.cls);
    for (size_t b = 0; b < idx.size(); ++b) {
      Rb(static_cast<int>(a), static_cast<int>(b)) =
          set.has_full_R ? set.R_full(idx[a], idx[b]) : (a == b ? set.rows[idx[a]].R : 0.0);
    }
  }
  Rblocks->push_back(Rb);
}

// `cls` (A66): the class of every assembled row (a TDCP set's projected rows are Tdcp). R is block-diagonal over the sets.
bool assembleSystem(const std::vector<RowSet>& sets, int N, Eigen::VectorXd* r, Eigen::MatrixXd* H, Eigen::MatrixXd* R,
                    std::vector<MeasClass>* cls) {
  std::vector<Eigen::RowVectorXd> Hrows;
  std::vector<double> rv;
  std::vector<Eigen::MatrixXd> Rb;
  cls->clear();
  for (const auto& s : sets) appendAccepted(s, &Hrows, &rv, &Rb, cls);
  const int m = static_cast<int>(rv.size());
  if (m == 0) return false;
  r->resize(m);
  H->resize(m, N);
  R->setZero(m, m);
  int k = 0;
  for (const auto& B : Rb) {
    const int mb = static_cast<int>(B.rows());
    R->block(k, k, mb, mb) = B;
    k += mb;
  }
  for (int i = 0; i < m; ++i) {
    (*r)(i) = rv[i];
    if (Hrows[i].size() != N) return false;
    H->row(i) = Hrows[i];
  }
  return true;
}

// Copies the iteration-0 membership (accepted / reason) onto the rows rebuilt at a later iterate. The
// factory's contract is to emit the same rows in the same order at every iterate; a structural mismatch
// (different count or satellite) aborts the extra iteration rather than mixing decisions.
bool applyMembership(const std::vector<RowSet>& decided, std::vector<RowSet>* fresh) {
  if (decided.size() != fresh->size()) return false;
  for (size_t s = 0; s < decided.size(); ++s) {
    const RowSet& a = decided[s];
    RowSet& b = (*fresh)[s];
    if (a.rows.size() != b.rows.size() || a.cls != b.cls) return false;
    for (size_t i = 0; i < a.rows.size(); ++i) {
      if (a.rows[i].sat.rtk_sat != b.rows[i].sat.rtk_sat || a.rows[i].pair_id != b.rows[i].pair_id) return false;
      b.rows[i].accepted = a.rows[i].accepted;
      b.rows[i].reason = a.rows[i].reason;
    }
  }
  return true;
}

// Consider columns (AMENDMENTS A66, Options::consider). For every augmented slot named by an entry: A = the assembled rows
// whose class is not listed for that slot. When every row is in A the gain is left untouched (bit-identical to no entry).
// Otherwise the slot's rows of K become [Σ_{s,:} H_Aᵀ S_AA⁻¹ on A, 0 elsewhere]: with the gain onto the slot restricted to
// the rows of A, its posterior covariance (E_s − K_s H) Σ (E_s − K_s H)ᵀ + K_s R K_sᵀ depends on K_s alone and, R having no
// cross terms between the classes (one block per set), is smallest there — the minimum-variance slot update given A, i.e.
// the result of processing A first with the full gain and the remaining rows next with the slot considered; the other
// columns' rows of the joint optimum ΣHᵀS⁻¹ are what minimises theirs (the Schmidt–Kalman gain). S_AA is a principal
// submatrix of the positive definite S. Returns false only if its factorisation fails.
bool applyConsider(const std::vector<JointUpdater::Options::Consider>& consider, const AugBlock& aug,
                   const std::vector<MeasClass>& row_cls, const Eigen::MatrixXd& Sigma, const Eigen::MatrixXd& H,
                   const Eigen::MatrixXd& S, Eigen::MatrixXd* K) {
  const int m = static_cast<int>(row_cls.size());
  std::vector<Slot> done;
  for (const JointUpdater::Options::Consider& c : consider) {
    if (std::find(done.begin(), done.end(), c.slot) != done.end()) continue;
    done.push_back(c.slot);
    int off = -1, dim = 0;
    for (const SlotInfo& si : aug.slots())
      if (si.kind == c.slot && si.tag == 0) {
        off = si.offset;
        dim = si.dim;
        break;
      }
    if (off < 0 || dim <= 0) continue;
    std::vector<int> allowed;
    for (int i = 0; i < m; ++i) {
      bool excluded = false;
      for (const JointUpdater::Options::Consider& e : consider)
        if (e.slot == c.slot && e.cls == row_cls[static_cast<size_t>(i)]) excluded = true;
      if (!excluded) allowed.push_back(i);
    }
    if (static_cast<int>(allowed.size()) == m) continue;   // no row of an excluded class: the ordinary gain
    const int col = HostIdx::dim + off;
    K->middleRows(col, dim).setZero();
    const int a = static_cast<int>(allowed.size());
    if (a == 0) continue;
    Eigen::MatrixXd S_aa(a, a), B(a, dim);
    const Eigen::MatrixXd SigmaCols = Sigma.middleCols(col, dim);   // Σ_{:,s}
    for (int p = 0; p < a; ++p) {
      for (int q = 0; q < a; ++q) S_aa(p, q) = S(allowed[static_cast<size_t>(p)], allowed[static_cast<size_t>(q)]);
      B.row(p) = H.row(allowed[static_cast<size_t>(p)]) * SigmaCols;   // (H_A Σ_{:,s})
    }
    Eigen::LDLT<Eigen::MatrixXd> ldlt(S_aa);
    if (ldlt.info() != Eigen::Success) return false;
    const Eigen::MatrixXd X = ldlt.solve(B);   // S_AA⁻¹ H_A Σ_{:,s}
    for (int p = 0; p < a; ++p) K->block(col, allowed[static_cast<size_t>(p)], dim, 1) = X.row(p).transpose();
  }
  return true;
}

// Projected rows (AMENDMENTS A70, Options::project), applied to every assembly of the system (first iterate and each
// relinearisation). For every class named by an entry: the columns of all its slots (tag 0; a slot absent from the block
// contributes none), the class's rows d and the others o. When there are rows and columns and the projection has rank
// r > 0, the system becomes [the o rows as they were, in their order | the m_d − r kept rows of the class, R = I] — the
// order and arithmetic of the A67 diagnostic build (diag_dproj.diff); otherwise it is left untouched (bit-identical).
// Returns false when a projection cannot be formed: R of the class not positive definite, or R coupling the class to
// another row (never by construction — assembleSystem() gives one R block per set, and a set holds one class).
bool applyProjection(const std::vector<JointUpdater::Options::Project>& project, const AugBlock& aug, Eigen::VectorXd* r,
                     Eigen::MatrixXd* H, Eigen::MatrixXd* R, std::vector<MeasClass>* cls) {
  std::vector<MeasClass> done;
  for (const JointUpdater::Options::Project& p : project) {
    if (std::find(done.begin(), done.end(), p.cls) != done.end()) continue;
    done.push_back(p.cls);
    std::vector<int> cols;
    for (const JointUpdater::Options::Project& e : project) {
      if (e.cls != p.cls) continue;
      for (const SlotInfo& si : aug.slots()) {
        if (si.kind != e.slot || si.tag != 0) continue;
        for (int j = 0; j < si.dim; ++j) {
          const int c = HostIdx::dim + si.offset + j;
          if (std::find(cols.begin(), cols.end(), c) == cols.end()) cols.push_back(c);
        }
        break;
      }
    }
    if (cols.empty()) continue;
    std::vector<int> d, o;
    for (int i = 0; i < static_cast<int>(cls->size()); ++i) ((*cls)[static_cast<size_t>(i)] == p.cls ? d : o).push_back(i);
    if (d.empty()) continue;
    const int md = static_cast<int>(d.size()), mo = static_cast<int>(o.size()), N = static_cast<int>(H->cols());
    for (int a : d)
      for (int b : o)
        if ((*R)(a, b) != 0.0 || (*R)(b, a) != 0.0) return false;
    Eigen::MatrixXd Rd(md, md), Hd(md, N);
    Eigen::VectorXd rd(md);
    for (int a = 0; a < md; ++a) {
      Hd.row(a) = H->row(d[static_cast<size_t>(a)]);
      rd(a) = (*r)(d[static_cast<size_t>(a)]);
      for (int b = 0; b < md; ++b) Rd(a, b) = (*R)(d[static_cast<size_t>(a)], d[static_cast<size_t>(b)]);
    }
    Eigen::MatrixXd Hp;
    Eigen::VectorXd rp;
    int rank = 0;
    if (!JointUpdater::projection(Hd, rd, Rd, cols, &Hp, &rp, &rank)) return false;
    if (rank == 0) continue;   // the rows do not see the slot's columns: nothing to take out
    const int kept = std::max(0, md - rank);
    const int m = mo + kept;
    Eigen::VectorXd r2(m);
    Eigen::MatrixXd H2(m, N), R2 = Eigen::MatrixXd::Zero(m, m);
    std::vector<MeasClass> c2;
    c2.reserve(static_cast<size_t>(m));
    for (int a = 0; a < mo; ++a) {
      r2(a) = (*r)(o[static_cast<size_t>(a)]);
      H2.row(a) = H->row(o[static_cast<size_t>(a)]);
      for (int b = 0; b < mo; ++b) R2(a, b) = (*R)(o[static_cast<size_t>(a)], o[static_cast<size_t>(b)]);
      c2.push_back((*cls)[static_cast<size_t>(o[static_cast<size_t>(a)])]);
    }
    if (kept > 0) {
      r2.tail(kept) = rp;
      H2.bottomRows(kept) = Hp;
      R2.bottomRightCorner(kept, kept).setIdentity();
    }
    for (int a = 0; a < kept; ++a) c2.push_back(p.cls);
    *r = r2;
    *H = H2;
    *R = R2;
    *cls = c2;
  }
  return true;
}

}  // namespace

// ---------------------------------------------------------------------------------------------
// JointUpdater
// ---------------------------------------------------------------------------------------------
JointUpdater::JointUpdater(const Config& cfg) : cfg_(cfg) {}

Eigen::MatrixXd JointUpdater::josephUpdate(Eigen::MatrixXd& Sigma, const Eigen::MatrixXd& H, const Eigen::MatrixXd& R,
                                           const Eigen::MatrixXd& K_in) {
  Eigen::MatrixXd K = K_in;
  if (K.size() == 0) {
    const Eigen::MatrixXd S = H * Sigma * H.transpose() + R;
    K = S.ldlt().solve(H * Sigma).transpose();     // K = ΣHᵀS⁻¹ without forming S⁻¹
  }
  const Eigen::MatrixXd IKH = Eigen::MatrixXd::Identity(Sigma.rows(), Sigma.cols()) - K * H;
  Sigma = IKH * Sigma * IKH.transpose() + K * R * K.transpose();
  Sigma = 0.5 * (Sigma + Sigma.transpose()).eval();
  return K;
}

double JointUpdater::posteriorNis(const Eigen::VectorXd& r, const Eigen::MatrixXd& H, const Eigen::MatrixXd& R,
                                  const Eigen::MatrixXd& Sigma_post) {
  Eigen::MatrixXd M = R - H * Sigma_post * H.transpose();
  M = 0.5 * (M + M.transpose()).eval();
  Eigen::LDLT<Eigen::MatrixXd> ldlt(M);
  if (ldlt.info() != Eigen::Success) return std::numeric_limits<double>::infinity();
  const double nis = r.dot(ldlt.solve(r));
  return std::isfinite(nis) ? nis : std::numeric_limits<double>::infinity();
}

bool JointUpdater::posteriorPsd(const Eigen::MatrixXd& Sigma, double tol) {
  const Eigen::Index n = Sigma.rows();
  if (Sigma.cols() != n) return false;
  if (n == 0) return true;
  Eigen::VectorXd d(n);
  for (Eigen::Index i = 0; i < n; ++i) {
    const double v = Sigma(i, i);
    if (!std::isfinite(v) || v < 0.0) return false;
    d(i) = v > 0.0 ? 1.0 / std::sqrt(v) : 1.0;   // a zero variance keeps scale 1 (its row must then be zero)
  }
  Eigen::MatrixXd Ss = d.asDiagonal() * Sigma * d.asDiagonal();
  Ss = 0.5 * (Ss + Ss.transpose()).eval();
  if (!Ss.allFinite()) return false;
  // Eigenvalues, not LDLT: Eigen's LDLT picks its pivots on the unreduced diagonal, so an exactly zero Schur complement
  // followed by a non-zero pivot reports NumericalIssue for a perfectly PSD singular matrix.
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(Ss, Eigen::EigenvaluesOnly);
  if (es.info() != Eigen::Success) return false;
  return es.eigenvalues().minCoeff() >= -tol;
}

bool JointUpdater::projection(const Eigen::MatrixXd& H, const Eigen::VectorXd& r, const Eigen::MatrixXd& R, const std::vector<int>& cols,
                              Eigen::MatrixXd* Hp, Eigen::VectorXd* rp, int* rank) {
  *rank = 0;
  const int m = static_cast<int>(H.rows()), nc = static_cast<int>(cols.size());
  if (r.size() != m || R.rows() != m || R.cols() != m) return false;
  for (int c : cols)
    if (c < 0 || c >= H.cols()) return false;
  if (m == 0 || nc == 0) return true;   // nothing to project
  // whitening: L⁻¹ with R = L Lᵀ (a diagonal R divides each row by its σ)
  Eigen::LLT<Eigen::MatrixXd> llt(R);
  if (llt.info() != Eigen::Success) return false;
  const Eigen::MatrixXd Hw = llt.matrixL().solve(H);
  const Eigen::VectorXd rw = llt.matrixL().solve(r);
  if (!Hw.allFinite() || !rw.allFinite()) return false;
  Eigen::MatrixXd F(m, nc);
  for (int j = 0; j < nc; ++j) F.col(j) = Hw.col(cols[static_cast<size_t>(j)]);
  // numerical rank from the singular values of F (the Householder R's diagonal is no rank test when a column is zero)
  const Eigen::JacobiSVD<Eigen::MatrixXd> sv(F);
  const Eigen::VectorXd s = sv.singularValues();
  if (!(s(0) > kProjRankTol * Hw.norm())) return true;   // rank 0: the rows do not see these columns
  int rk = 1;
  while (rk < s.size() && s(rk) > kProjRankTol * s(0)) ++rk;
  *rank = rk;
  if (rk == nc) {
    // full column rank: Householder QR of F, Q = [Q₁ Q₂] with range(Q₁) = range(F) (as the TDCP Δclk projection)
    Eigen::HouseholderQR<Eigen::MatrixXd> qr(F);
    const Eigen::MatrixXd Q = qr.householderQ() * Eigen::MatrixXd::Identity(m, m);
    const Eigen::MatrixXd Q2 = Q.rightCols(std::max(0, m - rk));
    const Eigen::MatrixXd Hk = Q2.transpose() * Hw;
    const Eigen::VectorXd rk_ = Q2.transpose() * rw;
    *Hp = Hk;
    *rp = rk_;
  } else {
    // rank-deficient: the left singular vectors beyond the rank span range(F)^⊥ up to the dropped s_k
    const Eigen::JacobiSVD<Eigen::MatrixXd> fu(F, Eigen::ComputeFullU);
    const Eigen::MatrixXd Q2 = fu.matrixU().rightCols(std::max(0, m - rk));
    const Eigen::MatrixXd Hk = Q2.transpose() * Hw;
    const Eigen::VectorXd rk_ = Q2.transpose() * rw;
    *Hp = Hk;
    *rp = rk_;
  }
  return true;
}

GateResult JointUpdater::gateClass(RowSet& set, const Eigen::MatrixXd& Sigma, int m_min, double alpha_w, double alpha_nis,
                                   double max_reject_frac) const {
  GateResult gr;
  const int M = static_cast<int>(set.rows.size());
  gr.w.assign(static_cast<size_t>(M), 0.0);
  std::vector<int> act;
  for (int i = 0; i < M; ++i) {
    set.rows[i].accepted = false;
    if (set.rows[i].reason == RejectReason::NotRejected) act.push_back(i);
  }
  gr.n_in = static_cast<int>(act.size());
  const bool tdcp = set.cls == MeasClass::Tdcp;
  if (tdcp) {
    // Diagnostics only (gnss_sats.csv innov_tdcp): the logged ν is made Δclk-free with the weighted-median
    // estimate. The test below is exactly invariant to a common shift of ν, so this changes no decision.
    double dclk = 0.0;
    RowBuilder::projectOutCommonClock(set, nullptr, &dclk);
    for (int i : act) set.rows[i].nu -= dclk;
  }
  const double w_thr = normalQuantile(1.0 - 0.5 * alpha_w);
  const int N = static_cast<int>(Sigma.rows());
  int n_rej = 0;
  // Why the class sits out when it does: fewer than m_min rows left (from the start or after rejections) is
  // BelowMin (AMENDMENTS A35); > max_reject_frac rejected is Majority. A numerically untestable S (not PD) keeps the
  // pre-A35 label Majority — RejectReason has no value of its own for it.
  RejectReason skip_reason = RejectReason::Majority;
  while (true) {
    const int m = static_cast<int>(act.size());
    if (m == 0 || m < m_min) {
      gr.class_skipped = true;
      skip_reason = RejectReason::BelowMin;
      break;
    }
    Eigen::MatrixXd H(m, N), R(m, m);
    Eigen::VectorXd nu(m);
    for (int a = 0; a < m; ++a) {
      const MeasRow& row = set.rows[act[a]];
      nu(a) = row.nu;
      H.row(a) = row.H;
      for (int b = 0; b < m; ++b) R(a, b) = set.has_full_R ? set.R_full(act[a], act[b]) : (a == b ? row.R : 0.0);
    }
    const Eigen::MatrixXd S = H * Sigma * H.transpose() + R;
    Eigen::LDLT<Eigen::MatrixXd> ldlt(S);
    if (ldlt.info() != Eigen::Success || !ldlt.isPositive()) {
      gr.class_skipped = true;   // S not positive definite: nothing here can be tested, the class sits out
      break;
    }
    // Test matrix W: S⁻¹ for an ordinary class (Baarda w_i = (S⁻¹ν)_i/√(S⁻¹)_ii, epoch NIS νᵀS⁻¹ν, dof m).
    // TDCP (§4.5): the common Δclk is a nuisance parameter, so W = S⁻¹ − S⁻¹1(1ᵀS⁻¹1)⁻¹1ᵀS⁻¹, the S-metric
    // projection orthogonal to 1. W·1 = 0 makes every w_i and the NIS invariant to Δclk, dof = m − 1, and
    // Pᵀ(P S Pᵀ)⁻¹P = W for the Householder rows P of projectOutCommonClock(), so this is the same statistic
    // the update sees. (Subtracting a robust Δclk and then using S⁻¹ leaves the Δclk uncertainty out of S:
    // review M1 measured 2.8e-2 false rejections per row at σ_ψ = 2° against α = 1e-3.)
    Eigen::MatrixXd W = ldlt.solve(Eigen::MatrixXd::Identity(m, m));
    int dof = m;
    if (tdcp) {
      const Eigen::VectorXd s1 = W * Eigen::VectorXd::Ones(m);
      const double q = s1.sum();   // 1ᵀS⁻¹1
      if (!(q > 0.0)) {
        gr.class_skipped = true;
        break;
      }
      W -= s1 * s1.transpose() / q;
      dof = m - 1;
    }
    const Eigen::VectorXd W_nu = W * nu;
    Eigen::VectorXd w(m);
    for (int a = 0; a < m; ++a) {
      // W_aa > 0 strictly for m ≥ 2 in the TDCP case (Cauchy–Schwarz); a row with no redundancy left is untestable.
      const double dia = W(a, a);
      w(a) = dia > 0.0 ? W_nu(a) / std::sqrt(dia) : 0.0;
    }
    const double nis = nu.dot(W_nu);
    const double nis_thr = dof > 0 ? chi2Quantile(dof, 1.0 - alpha_nis) : std::numeric_limits<double>::infinity();
    int imax = 0;
    for (int a = 1; a < m; ++a) {
      if (std::fabs(w(a)) > std::fabs(w(imax))) imax = a;
    }
    const bool w_fail = std::fabs(w(imax)) > w_thr;
    const bool nis_fail = !(nis <= nis_thr);
    if (w_fail || nis_fail) {
      MeasRow& bad = set.rows[act[imax]];
      bad.reason = w_fail ? RejectReason::WTest : RejectReason::Nis;
      bad.accepted = false;
      if (w_fail) ++gr.n_rejected_w; else ++gr.n_rejected_nis;
      ++n_rej;
      act.erase(act.begin() + imax);
      continue;
    }
    for (int a = 0; a < m; ++a) gr.w[static_cast<size_t>(act[a])] = w(a);
    gr.nis = nis;
    gr.dof = dof;
    break;
  }
  if (!gr.class_skipped && n_rej > max_reject_frac * gr.n_in) {
    gr.class_skipped = true;
    skip_reason = RejectReason::Majority;
  }
  if (gr.class_skipped) {
    // Too few rows left (BelowMin) or majority rejected (Majority): the whole class sits out this epoch
    // (§10 L4 "m<m_min" / "被剔>50% ⇒ 该类本历元不更新"); the rows still standing carry that reason.
    for (int i : act) {
      set.rows[i].reason = skip_reason;
      set.rows[i].accepted = false;
    }
    gr.n_out = 0;
  } else {
    for (int i : act) set.rows[i].accepted = true;
    gr.n_out = static_cast<int>(act.size());
  }
  return gr;
}

JointUpdater::Result JointUpdater::update(CoreState& core, AugBlock& aug, const RowFactory& rows, const Options& opt) const {
  Result res;
  const int n = aug.dim();
  const int N = HostIdx::dim + n;
  // 1) snapshot (bit-exact restore on rollback / observe-only)
  const CoreState core0 = core;
  const Eigen::VectorXd xg0 = aug.x();
  const Eigen::MatrixXd Q0 = aug.Q();
  const Eigen::Matrix<double, HostIdx::dim, Eigen::Dynamic> C0 = aug.C();
  auto restore = [&]() {
    core = core0;
    aug.x() = xg0;
    aug.Q() = Q0;
    aug.C() = C0;
  };
  const Eigen::MatrixXd Sigma = aug.jointCov(core.P);
  const int yaw_off = aug.find(Slot::Yaw);
  const int clk_off = aug.find(Slot::Clk);
  const int tilt_off = aug.find(Slot::Tilt);
  const int trans_off = aug.find(Slot::Trans);

  // 2) rows at x_0 and L4 gates on (r_0, S_0)
  std::vector<RowSet> sets;
  rows(core, aug, &sets);
  const bool gate = !(opt.disable_l4_l5 || cfg_.ablation.disable_l4_l5);
  for (auto& set : sets) {
    const size_t c = static_cast<size_t>(set.cls);
    if (gate) {
      GateResult gr = gateClass(set, Sigma, minMeas(cfg_, set.cls), cfg_.rejection.alpha_w, cfg_.rejection.alpha_nis,
                                cfg_.rejection.max_reject_frac);
      GateResult& acc = res.gates[c];
      acc.n_in += gr.n_in;
      acc.n_out += gr.n_out;
      acc.n_rejected_w += gr.n_rejected_w;
      acc.n_rejected_nis += gr.n_rejected_nis;
      acc.nis += gr.nis;
      acc.dof += gr.dof;
      acc.class_skipped = acc.class_skipped || gr.class_skipped;
      acc.w.insert(acc.w.end(), gr.w.begin(), gr.w.end());
    } else {
      GateResult& acc = res.gates[c];
      for (auto& row : set.rows) {
        if (row.reason == RejectReason::NotRejected) {
          row.accepted = true;
          ++acc.n_in;
          ++acc.n_out;
        }
        acc.w.push_back(0.0);
      }
    }
  }
  res.gated = sets;

  Eigen::VectorXd r_i;
  Eigen::MatrixXd H_i, R_i;
  std::vector<MeasClass> cls_i;
  if (!assembleSystem(sets, N, &r_i, &H_i, &R_i, &cls_i)) {
    res.committed = false;
    return res;   // nothing accepted: state untouched
  }
  // A70: rows of a class projected orthogonal to a slot's columns (after L4: the gates judged the original rows)
  if (!opt.project.empty()) {
    if (!applyProjection(opt.project, aug, &r_i, &H_i, &R_i, &cls_i)) {
      res.rollback_reason = "proj_failed";   // R of the class not positive definite (never by construction)
      restore();
      res.rolled_back = true;
      return res;
    }
    if (r_i.size() == 0) {
      res.committed = false;
      return res;   // every accepted row was taken out by the projection: state untouched
    }
  }

  // 3) iterations
  Eigen::VectorXd dx_prev = Eigen::VectorXd::Zero(N);   // x_i ⊟ x⁻
  Eigen::VectorXd dx = Eigen::VectorXd::Zero(N);        // x_{i+1} ⊟ x⁻
  CoreState core_i = core0;
  Eigen::VectorXd xg_i = xg0;
  Eigen::MatrixXd K, H_last, R_last;
  Eigen::VectorXd r_last;
  int max_iter = 1;
  const double deg = M_PI / 180.0;
  for (int i = 0;; ++i) {
    const Eigen::MatrixXd S = H_i * Sigma * H_i.transpose() + R_i;
    Eigen::LDLT<Eigen::MatrixXd> ldlt(S);
    if (ldlt.info() != Eigen::Success) {
      res.rollback_reason = "S_ldlt_failed";
      restore();
      res.rolled_back = true;
      return res;
    }
    K = ldlt.solve(H_i * Sigma).transpose();            // ΣHᵀS⁻¹
    if (!opt.consider.empty() && !applyConsider(opt.consider, aug, cls_i, Sigma, H_i, S, &K)) {
      res.rollback_reason = "S_ldlt_failed";              // A66: a principal submatrix of S failed to factorise
      restore();
      res.rolled_back = true;
      return res;
    }
    if (opt.schmidt) K.topRows(HostIdx::dim).setZero();  // L7: never write the host
    const Eigen::VectorXd dx_new = K * (r_i + H_i * dx_prev);
    core_i = core0.boxplus(dx_new.head<HostIdx::dim>());
    xg_i = xg0 + dx_new.tail(n);
    if (yaw_off >= 0) xg_i(yaw_off) = wrapYawValue(xg_i(yaw_off));
    res.iterations = i + 1;
    H_last = H_i;
    R_last = R_i;
    r_last = r_i;
    const Eigen::VectorXd step = dx_new - dx;
    dx = dx_new;
    if (i == 0) {
      // §4.8 (3): iterate only when the yaw leverage is the dominant non-linearity.
      if (opt.force_three_iterations) {
        max_iter = std::max(1, cfg_.iterate.max_iter);
      } else if (yaw_off >= 0) {
        const double sigma_psi = std::sqrt(std::max(0.0, Sigma(HostIdx::dim + yaw_off, HostIdx::dim + yaw_off)));
        const double dpsi = std::fabs(dx_new(HostIdx::dim + yaw_off));
        if (sigma_psi > cfg_.iterate.yaw_sigma_deg * deg || dpsi > cfg_.iterate.dyaw_deg * deg) {
          max_iter = std::max(1, cfg_.iterate.max_iter);
        }
      }
    } else if (!opt.force_three_iterations && step.norm() < 1e-12 * std::max(1.0, dx.norm())) {
      break;   // converged early ("at most 3"); the reacquisition window runs its fixed count (§4.8-3, review n1)
    }
    if (i + 1 >= max_iter) break;
    // relinearise at x_{i+1}: the factory reads x_G from aug (Q/C untouched). A throwing factory must not leave
    // the iterate in aug (review n4): restore, then propagate.
    aug.x() = xg_i;
    std::vector<RowSet> fresh;
    try {
      rows(core_i, aug, &fresh);
    } catch (...) {
      aug.x() = xg0;
      throw;
    }
    aug.x() = xg0;
    if (!applyMembership(sets, &fresh)) break;
    Eigen::VectorXd r_n;
    Eigen::MatrixXd H_n, R_n;
    std::vector<MeasClass> cls_n;
    if (!assembleSystem(fresh, N, &r_n, &H_n, &R_n, &cls_n)) break;
    if (!opt.project.empty() && !applyProjection(opt.project, aug, &r_n, &H_n, &R_n, &cls_n)) break;
    if (H_n.rows() != H_i.rows() || cls_n != cls_i) break;
    r_i = r_n;
    H_i = H_n;
    R_i = R_n;
    dx_prev = dx;
  }

  // 4) Joseph with the last linearisation
  Eigen::MatrixXd Sigma_post = Sigma;
  josephUpdate(Sigma_post, H_last, R_last, K);

  // 5) L5 posterior checks
  bool rollback = false;
  const int m = static_cast<int>(r_last.size());
  if (!Sigma_post.allFinite() || !dx.allFinite()) {
    rollback = true;
    res.rollback_reason = "nan";
  }
  if (!rollback && gate && cfg_.rejection.post_check) {
    // linearised post-fit residual r_post = r_last − H_last (x⁺ ⊟ x_last)
    const Eigen::VectorXd r_post = r_last - H_last * (dx - dx_prev);
    if (opt.schmidt) {
      // R − HΣ⁺Hᵀ is Cov(r_post) only for the optimal gain (then it equals RS⁻¹R). With the host rows of K zeroed
      // it can be indefinite and would force a rollback of every Schmidt update; the general form
      // (I − HK) S (I − HK)ᵀ is the covariance of r_post for any K and reduces to RS⁻¹R for the optimal one.
      const Eigen::MatrixXd IHK = Eigen::MatrixXd::Identity(m, m) - H_last * K;
      const Eigen::MatrixXd S_last = H_last * Sigma * H_last.transpose() + R_last;
      Eigen::MatrixXd cov = IHK * S_last * IHK.transpose();
      cov = 0.5 * (cov + cov.transpose()).eval();
      Eigen::LDLT<Eigen::MatrixXd> cl(cov);
      const double nis = (cl.info() == Eigen::Success) ? r_post.dot(cl.solve(r_post)) : std::numeric_limits<double>::infinity();
      res.nis_post = std::isfinite(nis) ? nis : std::numeric_limits<double>::infinity();
    } else {
      // With the gain K = ΣHᵀS⁻¹, r_post = (I − HK) r̃ = R S⁻¹ r̃ (r̃ = r_last + H_last·dx_prev, the innovation at the last
      // linearisation) and its covariance R − HΣ⁺Hᵀ is R S⁻¹ R, so the posterior NIS is r̃ᵀ S⁻¹ r̃ exactly (U10) — taken
      // that way (A61): for carrier rows R ≪ S, and R S⁻¹ R (~1e-17 m²) lies below the rounding of HΣ⁺Hᵀ, which made
      // posteriorNis() noise and rolled back sound TDCP epochs.
      // Consider columns (A66) change K, not this value: for any gain r_post = G r̃ with G = I − HK and Cov(r_post) = G S Gᵀ,
      // so r_postᵀ(G S Gᵀ)⁻¹r_post = r̃ᵀS⁻¹r̃ whenever G is invertible (U10 checks it against the general form above).
      const Eigen::VectorXd r_tilde = r_last + H_last * dx_prev;
      Eigen::LDLT<Eigen::MatrixXd> sl(Eigen::MatrixXd(H_last * Sigma * H_last.transpose() + R_last));
      const double nis = (sl.info() == Eigen::Success) ? r_tilde.dot(sl.solve(r_tilde)) : std::numeric_limits<double>::infinity();
      res.nis_post = std::isfinite(nis) ? nis : std::numeric_limits<double>::infinity();
    }
    const double thr = chi2Quantile(m, 1.0 - cfg_.rejection.alpha_nis);
    if (!(res.nis_post <= thr)) {
      rollback = true;
      res.rollback_reason = "post_nis";
    }
    const double dp = dx.segment<3>(HostIdx::pos).norm();
    double sigma_prior = opt.sigma_prior_pos_m;
    if (!(sigma_prior > 0.0)) {
      sigma_prior = std::sqrt(std::max(0.0, core0.P.block<3, 3>(HostIdx::pos, HostIdx::pos).diagonal().maxCoeff()));
    }
    const double step_max = std::max(3.0 * sigma_prior, cfg_.rejection.core_step_max_m);
    if (!rollback && sigma_prior < cfg_.rejection.core_step_max_m && dp > step_max) {
      rollback = true;
      res.rollback_reason = "host_step";
    }
    // Σ⁺ PSD up to rounding (A64): the exact zero directions of a noise-free stopped clone (Origin, A61 F13) must not
    // decide the update on the sign of the rounding (A62 ⑤); see posteriorPsd().
    if (!rollback && !posteriorPsd(Sigma_post)) {
      rollback = true;
      res.rollback_reason = "not_psd";
    }
  }

  // 6) bookkeeping: gain shares (§5 D3) and update magnitudes
  const double dxn = dx.norm();
  if (dxn > 0.0) {
    res.f_p = dx.segment<3>(HostIdx::pos).norm() / dxn;
    res.f_v = dx.segment<3>(HostIdx::vel).norm() / dxn;
    double xi2 = 0.0;
    if (tilt_off >= 0) xi2 += dx.segment<2>(HostIdx::dim + tilt_off).squaredNorm();
    if (yaw_off >= 0) xi2 += dx(HostIdx::dim + yaw_off) * dx(HostIdx::dim + yaw_off);
    if (trans_off >= 0) xi2 += dx.segment<3>(HostIdx::dim + trans_off).squaredNorm();
    res.f_xi = std::sqrt(xi2) / dxn;
    if (clk_off >= 0) res.f_clk = dx.segment<2>(HostIdx::dim + clk_off).norm() / dxn;
  }
  res.dp_host_norm = dx.segment<3>(HostIdx::pos).norm();
  res.dyaw = yaw_off >= 0 ? dx(HostIdx::dim + yaw_off) : 0.0;
  res.dt_norm = trans_off >= 0 ? dx.segment<3>(HostIdx::dim + trans_off).norm() : 0.0;
  res.dx_host = dx.head<HostIdx::dim>();
  res.dx_aug = dx.tail(n);

  // 7) commit or roll back
  if (rollback || opt.observe_only) {
    restore();
    res.committed = false;
    res.rolled_back = rollback;
    return res;
  }
  core = core_i;
  aug.x() = xg_i;
  aug.setFromJoint(Sigma_post, &core.P);
  aug.wrapYaw();
  if (opt.schmidt) {
    // L7 "只写帧/钟块": with the host rows of K zeroed the Joseph form leaves P unchanged in exact arithmetic, but the
    // GEMM rounding and the symmetrisation would still rewrite it (the host's (I−G)P is not symmetric, DESIGN
    // §16-11; review m1). Only C and Q_G carry the Schmidt update; the host is restored bit for bit.
    core = core0;
  }
  res.committed = true;
  return res;
}

}  // namespace livo_gnss
