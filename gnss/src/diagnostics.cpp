// SPDX-License-Identifier: BSD-3-Clause
// CSV/JSON serialisation of the diagnostics records (DESIGN §15 column order). Owner: WP-f.
// The enum name tables are frozen with the skeleton (they are data, used by tests and the writers).
// Every double is written with %.17g (exact round trip), so two deterministic runs produce identical files.
#include "livo_gnss/diagnostics.hpp"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>

namespace livo_gnss {

const char* phaseName(Phase p) {
  switch (p) {
    case Phase::OFF: return "OFF";
    case Phase::OBSERVE: return "OBSERVE";
    case Phase::WAIT_CORE: return "WAIT_CORE";
    case Phase::WAIT_NAV: return "WAIT_NAV";
    case Phase::COLLECT: return "COLLECT";
    case Phase::ALIGN: return "ALIGN";
    case Phase::TRACKING: return "TRACKING";
    case Phase::OUTAGE: return "OUTAGE";
    case Phase::RECOVERY: return "RECOVERY";
    default: return "?";
  }
}

const char* measClassName(MeasClass c) {
  switch (c) {
    case MeasClass::Psr: return "psr";
    case MeasClass::Dopp: return "dopp";
    case MeasClass::Tdcp: return "tdcp";
    case MeasClass::PvtPos: return "pvt_pos";
    case MeasClass::PvtVel: return "pvt_vel";
    default: return "?";
  }
}

const char* rejectReasonName(RejectReason r) {
  switch (r) {
    case RejectReason::NotRejected: return "none";
    case RejectReason::Mask: return "mask";
    case RejectReason::L1: return "l1";
    case RejectReason::Clock: return "clock";
    case RejectReason::SlipD1: return "slip_d1";
    case RejectReason::SlipD2: return "slip_d2";
    case RejectReason::SlipD3: return "slip_d3";
    case RejectReason::SlipD4: return "slip_d4";
    case RejectReason::SlipD5: return "slip_d5";
    case RejectReason::SlipD6: return "slip_d6";
    case RejectReason::WTest: return "w";
    case RejectReason::Nis: return "nis";
    case RejectReason::Majority: return "majority";
    case RejectReason::Step: return "step";
    case RejectReason::Exclusive: return "exclusive";
    case RejectReason::NoEph: return "no_eph";
    case RejectReason::Late: return "late";
    case RejectReason::BelowMin: return "below_min";   // appended P2b with the enum value (A35)
    default: return "?";
  }
}

const char* eventKindName(EventKind k) {
  switch (k) {
    case EventKind::ACTIVATE: return "ACTIVATE";
    case EventKind::REANCHOR: return "REANCHOR";
    case EventKind::REINIT: return "REINIT";
    case EventKind::REINIT_YAW: return "REINIT_YAW";
    case EventKind::CLOCK_JUMP: return "CLOCK_JUMP";
    case EventKind::SLIP: return "SLIP";
    case EventKind::LATE_EPOCH: return "LATE_EPOCH";
    case EventKind::TOFF_WARN: return "TOFF_WARN";
    case EventKind::LIVO_ALARM_ON: return "LIVO_ALARM_ON";
    case EventKind::LIVO_ALARM_OFF: return "LIVO_ALARM_OFF";
    case EventKind::DEGENERATE_ON: return "DEGENERATE_ON";
    case EventKind::DEGENERATE_OFF: return "DEGENERATE_OFF";
    case EventKind::REPIVOT: return "REPIVOT";
    case EventKind::ROLLBACK: return "ROLLBACK";
    case EventKind::MSGDEF_REJECT: return "MSGDEF_REJECT";
    case EventKind::OUTAGE_ON: return "OUTAGE_ON";
    case EventKind::OUTAGE_OFF: return "OUTAGE_OFF";
    case EventKind::RECOVERY: return "RECOVERY";
    case EventKind::PSEUDOINVERSE: return "PSEUDOINVERSE";
    case EventKind::TDCP_PAUSED: return "TDCP_PAUSED";
    case EventKind::PHASE_CHANGE: return "PHASE_CHANGE";
    case EventKind::CONFIG_WARNING: return "CONFIG_WARNING";
    case EventKind::FRAME_JUMP: return "FRAME_JUMP";
    case EventKind::REINIT_SUPPRESSED: return "REINIT_SUPPRESSED";
    default: return "?";
  }
}

// ---------------------------------------------------------------------------------------------
// Small CSV helpers: a row is built by appending fields; the header is built with the same walk so
// the two can never drift apart.
// ---------------------------------------------------------------------------------------------
namespace {

class Row {
 public:
  void add(const char* name, double v) { if (key(name)) return; char b[40]; std::snprintf(b, sizeof b, "%.17g", v); s_ += b; }
  void add(const char* name, int v) { if (key(name)) return; s_ += std::to_string(v); }
  void add(const char* name, bool v) { if (key(name)) return; s_ += v ? "1" : "0"; }
  void add(const char* name, uint64_t v) { if (key(name)) return; s_ += std::to_string(v); }
  void add(const char* name, const char* v) { if (key(name)) return; s_ += v; }
  void add(const std::string& name, double v) { add(name.c_str(), v); }
  void add(const std::string& name, int v) { add(name.c_str(), v); }
  void add(const std::string& name, bool v) { add(name.c_str(), v); }
  // Header mode: only the names are collected.
  static Row header() { Row r; r.header_ = true; return r; }
  const std::string& str() const { return s_; }

 private:
  // Appends the separator and, in header mode, the column name; returns true when the value must be skipped.
  bool key(const char* name) {
    if (!s_.empty()) s_ += ',';
    if (header_) s_ += name;
    return header_;
  }
  std::string s_;
  bool header_ = false;
};

// Per-class counters (ClassStats) with a "<class>_" prefix; header and row share this walk.
void classStats(Row& r, const char* cls, const ClassStats& c) {
  const std::string p = std::string(cls) + "_";
  r.add(p + "cand", c.candidates);
  r.add(p + "acc", c.accepted);
  for (size_t i = 1; i < static_cast<size_t>(RejectReason::Count); ++i)   // NotRejected is not a rejection
    r.add(p + "rej_" + rejectReasonName(static_cast<RejectReason>(i)), c.rejected[i]);
  r.add(p + "nis_prior", c.nis_prior);
  r.add(p + "dof", c.dof);
  r.add(p + "nis_post", c.nis_post);
  r.add(p + "skipped", c.skipped);
}

void epochWalk(Row& r, const EpochRecord& e) {
  r.add("week", e.gpst.week);
  r.add("tow", e.gpst.tow);
  r.add("t_gpst", e.t_gpst);
  r.add("t_est", e.t_est);
  r.add("t_k", e.t_k);
  r.add("delta", e.delta);
  r.add("arrival_latency", e.arrival_latency);
  r.add("cross_cycle_bridge", e.cross_cycle_bridge);
  r.add("phase", phaseName(e.phase));
  r.add("degenerate", e.degenerate);
  r.add("livo_alarm", e.livo_alarm);
  r.add("schmidt", e.schmidt);
  r.add("observe_only", e.observe_only);
  r.add("n_gps", e.n_gps);
  r.add("n_glo", e.n_glo);
  r.add("n_gal", e.n_gal);
  r.add("n_bds", e.n_bds);
  r.add("n_qzs", e.n_qzs);
  r.add("n_total_raw", e.n_total_raw);
  for (size_t i = 0; i < static_cast<size_t>(MeasClass::Count); ++i)
    classStats(r, measClassName(static_cast<MeasClass>(i)), e.cls[i]);
  r.add("f_p", e.f_p);
  r.add("f_xi", e.f_xi);
  r.add("f_clk", e.f_clk);
  r.add("f_v", e.f_v);
  r.add("tilt_x", e.tilt[0]);
  r.add("tilt_y", e.tilt[1]);
  r.add("tilt_sigma_x", e.tilt_sigma[0]);
  r.add("tilt_sigma_y", e.tilt_sigma[1]);
  r.add("yaw", e.yaw);
  r.add("yaw_sigma", e.yaw_sigma);
  r.add("trans_x", e.trans[0]);
  r.add("trans_y", e.trans[1]);
  r.add("trans_z", e.trans[2]);
  r.add("trans_sigma_x", e.trans_sigma[0]);
  r.add("trans_sigma_y", e.trans_sigma[1]);
  r.add("trans_sigma_z", e.trans_sigma[2]);
  r.add("clk_b", e.clk_b);
  r.add("clk_b_sigma", e.clk_b_sigma);
  r.add("clk_d", e.clk_d);
  r.add("clk_d_sigma", e.clk_d_sigma);
  r.add("isb_C", e.isb_C);
  r.add("isb_E", e.isb_E);
  r.add("isb_R", e.isb_R);
  r.add("td", e.td);
  r.add("td_sigma", e.td_sigma);
  r.add("spp_minus_filter_x", e.spp_minus_filter[0]);
  r.add("spp_minus_filter_y", e.spp_minus_filter[1]);
  r.add("spp_minus_filter_z", e.spp_minus_filter[2]);
  r.add("spp_valid", e.spp_valid);
  r.add("lambda_1", e.lambda[0]);
  r.add("lambda_2", e.lambda[1]);
  r.add("lambda_3", e.lambda[2]);
  r.add("u1_x", e.u1[0]);
  r.add("u1_y", e.u1[1]);
  r.add("u1_z", e.u1[2]);
  for (int i = 0; i < 19; ++i) r.add("host_sigma_" + std::to_string(i), e.host_sigma[i]);
  r.add("dp_host_norm", e.dp_host_norm);
  r.add("dyaw", e.dyaw);
  r.add("dt_norm", e.dt_norm);
  r.add("iterations", e.iterations);
  r.add("committed", e.committed);
  r.add("rolled_back", e.rolled_back);
  r.add("n_tdcp_pairs_open", e.n_tdcp_pairs_open);
  r.add("elapsed_ms", e.elapsed_ms);
  r.add("psr_decim_skipped", e.psr_decim_skipped);   // A43 skip counts (P3 integration, C12)
  r.add("pvt_decim_skipped", e.pvt_decim_skipped);
}

// "G07"-style id without going through RTKLIB (satIdStringOf lives in the adapter).
std::string satLabel(const SatId& s) {
  char b[16];
  std::snprintf(b, sizeof b, "%c%02d", sysChar(s.sys), s.prn);
  return b;
}

void satWalk(Row& r, const SatRecord& s) {
  r.add("week", s.gpst.week);
  r.add("tow", s.gpst.tow);
  r.add("sat", satLabel(s.sat).c_str());
  r.add("rtk_sat", s.sat.rtk_sat);
  r.add("el_deg", s.el_deg);
  r.add("az_deg", s.az_deg);
  r.add("cn0", s.cn0);
  r.add("psr_usable", s.psr_usable);
  r.add("dopp_usable", s.dopp_usable);
  r.add("cp_usable", s.cp_usable);
  r.add("sigma_psr", s.sigma_psr);
  r.add("sigma_dopp", s.sigma_dopp);
  r.add("sigma_tdcp", s.sigma_tdcp);
  r.add("innov_psr", s.innov_psr);
  r.add("innov_dopp", s.innov_dopp);
  r.add("innov_tdcp", s.innov_tdcp);
  r.add("w_psr", s.w_psr);
  r.add("w_dopp", s.w_dopp);
  r.add("w_tdcp", s.w_tdcp);
  r.add("reason_psr", rejectReasonName(s.reason_psr));
  r.add("reason_dopp", rejectReasonName(s.reason_dopp));
  r.add("reason_tdcp", rejectReasonName(s.reason_tdcp));
  r.add("slip_detectors", static_cast<int>(s.slip_detectors));
  r.add("nlos_suspect", s.nlos_suspect);
}

void covWalk(Row& r, const CovRecord& c) {
  r.add("t_k", c.t_k);
  r.add("stage", c.stage.c_str());
  r.add("effct_feat_num", c.effct_feat_num);
  for (int i = 0; i < 19; ++i) r.add("host_sigma_" + std::to_string(i), c.host_sigma[i]);
  for (int i = 0; i < 3; ++i) r.add("pos_eig_" + std::to_string(i + 1), c.pos_eig[i]);
  for (int i = 0; i < 3; ++i) r.add("rot_eig_" + std::to_string(i + 1), c.rot_eig[i]);
  r.add("min_diag_prior", c.min_diag_prior);
  r.add("cond_prior", c.cond_prior);
  for (int col = 0; col < 3; ++col)
    for (int row = 0; row < 3; ++row) r.add("pos_eigvec_" + std::to_string(col + 1) + "_" + std::to_string(row), c.pos_eigvec(row, col));
  for (int col = 0; col < 3; ++col)
    for (int row = 0; row < 3; ++row) r.add("rot_eigvec_" + std::to_string(col + 1) + "_" + std::to_string(row), c.rot_eigvec(row, col));
  // Variable length: one column, '|'-separated, empty when there are no slots.
  std::string aug;
  for (int i = 0; i < c.aug_sigma.size(); ++i) {
    char b[40];
    std::snprintf(b, sizeof b, "%.17g", c.aug_sigma[i]);
    if (i) aug += '|';
    aug += b;
  }
  r.add("aug_sigma", aug.c_str());
}

std::string jsonString(const std::string& s) {
  std::string o = "\"";
  for (unsigned char ch : s) {
    switch (ch) {
      case '"': o += "\\\""; break;
      case '\\': o += "\\\\"; break;
      case '\n': o += "\\n"; break;
      case '\r': o += "\\r"; break;
      case '\t': o += "\\t"; break;
      default:
        if (ch < 0x20) { char b[8]; std::snprintf(b, sizeof b, "\\u%04x", ch); o += b; }
        else o += static_cast<char>(ch);
    }
  }
  return o + "\"";
}

std::string jsonNumber(double v) {
  if (std::isnan(v) || std::isinf(v)) return "null";   // JSON has no NaN/Inf
  char b[40];
  std::snprintf(b, sizeof b, "%.17g", v);
  return b;
}

}  // namespace

std::string EpochRecord::csvHeader() { Row r = Row::header(); epochWalk(r, EpochRecord()); return r.str(); }
std::string EpochRecord::csvRow() const { Row r; epochWalk(r, *this); return r.str(); }
std::string SatRecord::csvHeader() { Row r = Row::header(); satWalk(r, SatRecord()); return r.str(); }
std::string SatRecord::csvRow() const { Row r; satWalk(r, *this); return r.str(); }
std::string CovRecord::csvHeader() { Row r = Row::header(); covWalk(r, CovRecord()); return r.str(); }
std::string CovRecord::csvRow() const { Row r; covWalk(r, *this); return r.str(); }

std::string Event::logLine() const {
  char b[256];
  std::snprintf(b, sizeof b, "t_k=%.9f t_est=%.9f gpst=%d:%.9f %s magnitude=%.17g detail=", t_k, t_est, gpst.week, gpst.tow,
                eventKindName(kind), magnitude);
  return std::string(b) + jsonString(detail);
}

std::string Summary::toJson() const {
  std::string s = "{\n";
  auto num = [&](const char* k, double v, bool last = false) { s += "  \"" + std::string(k) + "\": " + jsonNumber(v) + (last ? "\n" : ",\n"); };
  auto integer = [&](const char* k, int v) { s += "  \"" + std::string(k) + "\": " + std::to_string(v) + ",\n"; };
  auto perClass = [&](const char* k, const std::array<double, static_cast<size_t>(MeasClass::Count)>& a) {
    s += "  \"" + std::string(k) + "\": {";
    for (size_t i = 0; i < a.size(); ++i) {
      if (i) s += ", ";
      s += "\"" + std::string(measClassName(static_cast<MeasClass>(i))) + "\": " + jsonNumber(a[i]);
    }
    s += "},\n";
  };
  integer("epochs_total", epochs_total);
  integer("epochs_updated", epochs_updated);
  integer("epochs_rolled_back", epochs_rolled_back);
  perClass("mean_nis_per_dof", mean_nis_per_dof);
  perClass("acceptance_rate", acceptance_rate);
  num("tdcp_coverage", tdcp_coverage);
  num("elapsed_p50_ms", elapsed_p50_ms);
  num("elapsed_p95_ms", elapsed_p95_ms);
  num("elapsed_p99_ms", elapsed_p99_ms);
  integer("n_activate", n_activate);
  integer("n_reinit", n_reinit);
  integer("n_reanchor", n_reanchor);
  integer("n_clock_jump", n_clock_jump);
  integer("n_slip", n_slip);
  integer("n_late", n_late);
  integer("n_msgdef_reject", n_msgdef_reject);
  integer("n_bds_b1c_reject", n_bds_b1c_reject);
  integer("n_gal_prn_gt36", n_gal_prn_gt36);
  s += "  \"n_glo_fcn_mismatch\": " + std::to_string(n_glo_fcn_mismatch) + "\n";
  s += "}\n";
  return s;
}

}  // namespace livo_gnss
