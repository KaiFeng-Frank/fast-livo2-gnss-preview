// SPDX-License-Identifier: BSD-3-Clause
// Config validation and canonical dump (preregistration sha256). Owner: WP-f.
//
// validateConfig: structural and range checks only — no value is "tuned" here. The data-dependent
// requirements (AMENDMENTS A16: ura_semantics must be decided by the ingest audit before a gnss_comm run;
// RINEX source needs both files) are enforced only when gnss.enable is true, because with enable=false the
// core is never constructed and the run must stay bit-identical to R with an untouched config (DESIGN §13).
// The time-map checks (A40: leap string, drift without its reference instant) hold for every config: they are
// the configuration errors TimeMap would otherwise throw on, reported here with the key name.
//
// canonicalDump: one "key=value" line per field in header order (config.hpp), doubles as %.17g, enums as
// their YAML names, maps sorted by key. Two builds of the same Config produce the same bytes, so the sha256
// of this dump identifies the configuration (TEST_PLAN §3).
#include "livo_gnss/config.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

namespace livo_gnss {

namespace {

const char* modeName(Config::Mode m) {
  switch (m) {
    case Config::Mode::Tight: return "tight";
    case Config::Mode::LoosePvt: return "loose_pvt";
    case Config::Mode::LivgoProbe: return "livgo_probe";
    case Config::Mode::LivgoProbeNorej: return "livgo_probe_norej";
  }
  return "?";
}
const char* slotName(Config::UpdateSlot s) { return s == Config::UpdateSlot::PreLio ? "pre_lio" : "post_vio"; }
const char* sourceName(Config::SourceKind s) {
  switch (s) {
    case Config::SourceKind::GnssCommRos1: return "gnss_comm_ros1";
    case Config::SourceKind::Rinex: return "rinex";
    case Config::SourceKind::UbxStream: return "ubx_stream";
  }
  return "?";
}
const char* leverFrameName(Config::Frames::LeverArmFrame f) { return f == Config::Frames::LeverArmFrame::Lidar ? "lidar" : "imu"; }
const char* baseName(Config::Time::Base b) {
  switch (b) {
    case Config::Time::Base::Gpst: return "gpst";
    case Config::Time::Base::Utc: return "utc";
    case Config::Time::Base::Host: return "host";
  }
  return "?";
}
const char* uraName(Config::Ephem::UraSemantics u) {
  switch (u) {
    case Config::Ephem::UraSemantics::Unknown: return "unknown";
    case Config::Ephem::UraSemantics::Index: return "index";
    case Config::Ephem::UraSemantics::Metres: return "metres";
  }
  return "?";
}

bool finite(double v) { return std::isfinite(v); }

// GPS epoch 1980-01-06 00:00:00 on the GTime::toSec() scale (Unix-epoch origin, GPST, no leap seconds; A18/A40).
constexpr double kGpsEpochToSec = 315964800.0;

// gnss.time.leap (A40): "auto", or GPST−UTC as a plain decimal integer of one or two digits (optional '+'), i.e.
// [0, 99] s (18 s since 2017; it has never been negative). No whitespace, sign '-', fraction or exponent: the
// config.hpp contract is "an integer string". Every string accepted here also parses in TimeMap (trim + strtod +
// integral, time_map.cpp), so a configuration that validates cannot throw there later; test_wpf_serialisation
// cross-checks both directions.
bool leapValid(const std::string& s) {
  if (s == "auto") return true;
  const size_t i0 = (!s.empty() && s[0] == '+') ? 1 : 0;
  const size_t n = s.size() - i0;
  if (n < 1 || n > 2) return false;
  for (size_t i = i0; i < s.size(); ++i)
    if (s[i] < '0' || s[i] > '9') return false;
  return true;
}

// RINEX satellite id: system letter + two digits ("G07", "C59").
bool satIdValid(const std::string& s) {
  if (s.size() != 3) return false;
  const char c = s[0];
  if (!(c == 'G' || c == 'R' || c == 'E' || c == 'C' || c == 'J' || c == 'I' || c == 'S')) return false;
  return s[1] >= '0' && s[1] <= '9' && s[2] >= '0' && s[2] <= '9' && s != std::string(1, c) + "00";
}

class Checker {
 public:
  explicit Checker(std::vector<std::string>* out) : out_(out) {}
  void require(bool ok, const std::string& msg) { if (!ok) out_->push_back(msg); }
  void positive(const char* key, double v) { require(finite(v) && v > 0.0, std::string(key) + " must be > 0"); }
  void nonneg(const char* key, double v) { require(finite(v) && v >= 0.0, std::string(key) + " must be >= 0"); }
  void prob(const char* key, double v) { require(finite(v) && v > 0.0 && v < 1.0, std::string(key) + " must be in (0,1)"); }
  void atLeast(const char* key, int v, int lo) { require(v >= lo, std::string(key) + " must be >= " + std::to_string(lo)); }
  void sysMap(const char* key, const std::map<char, double>& m) {
    // A15: QZSS carries its own key; a missing key would silently read 0 through operator[].
    for (char c : {'G', 'E', 'C', 'R', 'J'})
      require(m.count(c) != 0, std::string(key) + " lacks system key '" + std::string(1, c) + "'");
    for (const auto& kv : m) require(finite(kv.second) && kv.second > 0.0, std::string(key) + "[" + std::string(1, kv.first) + "] must be > 0");
  }

 private:
  std::vector<std::string>* out_;
};

// ---- canonical dump helpers ----
// Strings are quoted with '"', '\\', newline, CR and tab escaped, so a path containing '\n' or ',' cannot
// forge a line or a list element and the dump stays a one-to-one function of the Config.
std::string quoted(const std::string& v) {
  std::string o = "\"";
  for (unsigned char ch : v) {
    switch (ch) {
      case '"': o += "\\\""; break;
      case '\\': o += "\\\\"; break;
      case '\n': o += "\\n"; break;
      case '\r': o += "\\r"; break;
      case '\t': o += "\\t"; break;
      default: o += static_cast<char>(ch);
    }
  }
  return o + "\"";
}

class Dump {
 public:
  void add(const char* k, double v) { char b[40]; std::snprintf(b, sizeof b, "%.17g", v); line(k, b); }
  void add(const char* k, int v) { line(k, std::to_string(v)); }
  void add(const char* k, bool v) { line(k, v ? "true" : "false"); }
  void add(const char* k, const char* v) { line(k, v); }   // enum names: fixed identifiers, never user text
  void add(const char* k, const std::string& v) { line(k, quoted(v)); }
  void add(const char* k, const std::vector<std::string>& v) {
    std::string s = "[";
    for (size_t i = 0; i < v.size(); ++i) { if (i) s += ","; s += quoted(v[i]); }
    line(k, s + "]");
  }
  void add(const char* k, const std::map<char, double>& m) {   // std::map iterates in key order
    std::string s = "{";
    bool first = true;
    for (const auto& kv : m) {
      char b[48];
      std::snprintf(b, sizeof b, "%s%c:%.17g", first ? "" : ",", kv.first, kv.second);
      s += b;
      first = false;
    }
    line(k, s + "}");
  }
  void add3(const char* k, const double v[3]) {
    char b[128];
    std::snprintf(b, sizeof b, "[%.17g,%.17g,%.17g]", v[0], v[1], v[2]);
    line(k, b);
  }
  std::string str() const { return s_; }

 private:
  void line(const char* k, const std::string& v) { s_ += k; s_ += '='; s_ += v; s_ += '\n'; }
  std::string s_;
};

}  // namespace

std::vector<std::string> validateConfig(const Config& c) {
  std::vector<std::string> out;
  Checker ck(&out);

  // ---- data-dependent requirements: only when the core will actually run ----
  if (c.enable) {
    if (c.source == Config::SourceKind::GnssCommRos1)
      ck.require(c.ephem.ura_semantics != Config::Ephem::UraSemantics::Unknown,
                 "gnss.ephem.ura_semantics is 'unknown' with source gnss_comm_ros1: run the ingest audit (§12.6-4) and set index|metres (A16)");
    if (c.source == Config::SourceKind::Rinex) {
      ck.require(!c.rinex.obs.empty(), "gnss.rinex.obs is empty with source rinex");
      ck.require(!c.rinex.nav.empty(), "gnss.rinex.nav is empty with source rinex");
    }
    for (const auto& p : c.nav_rinex) ck.require(!p.empty(), "gnss.nav_rinex contains an empty path");
    ck.require(c.systems.gps || c.systems.glo || c.systems.gal || c.systems.bds, "gnss.systems: no constellation enabled");
  }
  ck.require(!(c.mode == Config::Mode::LoosePvt && !c.use.pvt_vel && !c.use.psr && !c.use.dopp && !c.use.tdcp),
             "gnss.mode loose_pvt with every measurement class disabled");

  // ---- topics ----
  if (c.source == Config::SourceKind::GnssCommRos1) {
    ck.require(!c.topics.meas.empty(), "gnss.topics.meas is empty");
    ck.require(!c.topics.ephem.empty(), "gnss.topics.ephem is empty");
    ck.require(!c.topics.glo_ephem.empty(), "gnss.topics.glo_ephem is empty");
  }
  ck.nonneg("gnss.rinex.latency_s", c.rinex.latency_s);

  // ---- systems / exclusions ----
  ck.positive("gnss.systems.bds_geo.psr_scale", c.systems.bds_geo.psr_scale);
  for (const auto& s : c.exclude_sats) ck.require(satIdValid(s), "gnss.exclude_sats: bad satellite id '" + s + "' (expected e.g. G07, C59)");

  // ---- frames ----
  for (int i = 0; i < 3; ++i) ck.require(finite(c.frames.lever_arm_xyz[i]), "gnss.frames.lever_arm_xyz has a non-finite entry");
  ck.nonneg("gnss.frames.lever_arm_sigma", c.frames.lever_arm_sigma);
  ck.positive("gnss.frames.pivot_distance_m", c.frames.pivot_distance_m);
  ck.positive("gnss.frames.sigma_dov_rad", c.frames.sigma_dov_rad);
  ck.nonneg("gnss.frames.sigma_lev_rad", c.frames.sigma_lev_rad);

  // ---- time ----
  // Checked whatever gnss.enable says: TimeMap's constructor rejects the same two mistakes (leap string, drift
  // without reference) with std::invalid_argument, so a config that fails here could never run (A40).
  ck.require(leapValid(c.time.leap),
             "gnss.time.leap must be 'auto' or GPST-UTC as a decimal integer in [0, 99] (e.g. 18), got '" + c.time.leap + "'");
  ck.require(finite(c.time.offset_s) && std::fabs(c.time.offset_s) < 3600.0, "gnss.time.offset_s must be finite and |offset| < 3600 s");
  ck.require(finite(c.time.drift_ppm) && std::fabs(c.time.drift_ppm) < 1000.0, "gnss.time.drift_ppm must be finite and |drift| < 1000 ppm");
  ck.require(finite(c.time.t_ref_gpst_s) && c.time.t_ref_gpst_s >= 0.0, "gnss.time.t_ref_gpst_s must be >= 0");
  // A40: the drift term drift·(t_gpst − t_ref) needs its reference instant. t_ref_gpst_s is on the GTime::toSec()
  // scale (M3DGR ≈ 1.73e9); the §14 default 0 would multiply ~1.7e9 s (1 ppm ⇒ ~1700 s), and any value before the
  // GPS epoch is equally meaningless on this scale. With drift = 0 the reference is unused and 0 stays valid.
  if (c.time.drift_ppm != 0.0) {
    char got[40];
    std::snprintf(got, sizeof got, "%.17g", c.time.t_ref_gpst_s);
    ck.require(finite(c.time.t_ref_gpst_s) && c.time.t_ref_gpst_s >= kGpsEpochToSec,
               "gnss.time.drift_ppm != 0 needs gnss.time.t_ref_gpst_s = the drift reference instant in GTime::toSec() seconds "
               "(Unix-epoch origin, GPST scale, no leap seconds; >= 315964800 = GPS epoch, M3DGR ~1.73e9; A18/A40), got " +
                   std::string(got));
  }
  ck.positive("gnss.time.sigma_toff_s", c.time.sigma_toff_s);
  ck.positive("gnss.time.td_rw_s2_per_s", c.time.td_rw_s2_per_s);
  ck.positive("gnss.time.max_bridge_s", c.time.max_bridge_s);
  ck.require(c.time.max_bridge_s <= 5.0, "gnss.time.max_bridge_s must be <= 5 s (§4.1: δ ≤ 0.3 s by design)");
  ck.atLeast("gnss.time.bridge_ring_cycles", c.time.bridge_ring_cycles, 1);
  ck.positive("gnss.time.toff_monitor.window_s", c.time.toff_monitor.window_s);
  ck.nonneg("gnss.time.toff_monitor.min_abs_s", c.time.toff_monitor.min_abs_s);
  ck.positive("gnss.time.toff_monitor.n_sigma", c.time.toff_monitor.n_sigma);

  // ---- IMU bridge ----
  ck.positive("gnss.imu_bridge.acc_n", c.imu_bridge.acc_n);
  ck.positive("gnss.imu_bridge.gyr_n", c.imu_bridge.gyr_n);

  // ---- masks ----
  ck.require(finite(c.masks.el_deg) && c.masks.el_deg >= 0.0 && c.masks.el_deg < 90.0, "gnss.masks.el_deg must be in [0,90)");
  for (auto kv : {std::make_pair("gnss.masks.cn0_psr", c.masks.cn0_psr), std::make_pair("gnss.masks.cn0_dopp", c.masks.cn0_dopp),
                  std::make_pair("gnss.masks.cn0_cp", c.masks.cn0_cp)})
    ck.require(finite(kv.second) && kv.second >= 0.0 && kv.second <= 70.0, std::string(kv.first) + " must be in [0,70] dB-Hz");
  ck.atLeast("gnss.masks.track_psr", c.masks.track_psr, 1);
  ck.atLeast("gnss.masks.track_cp", c.masks.track_cp, 1);
  ck.sysMap("gnss.masks.eph_age_s", c.masks.eph_age_s);
  ck.positive("gnss.masks.max_psr_std_m", c.masks.max_psr_std_m);
  ck.positive("gnss.masks.max_dopp_std_hz", c.masks.max_dopp_std_hz);
  ck.positive("gnss.masks.max_cp_std_m", c.masks.max_cp_std_m);
  ck.positive("gnss.masks.psr_min_m", c.masks.psr_min_m);
  ck.require(finite(c.masks.psr_max_m) && c.masks.psr_max_m > c.masks.psr_min_m, "gnss.masks.psr_max_m must exceed psr_min_m");
  ck.positive("gnss.masks.max_range_rate_mps", c.masks.max_range_rate_mps);

  // ---- noise ----
  ck.positive("gnss.noise.psr.sigma0_m", c.noise.psr.sigma0_m);
  ck.require(finite(c.noise.psr.cn0_ref) && c.noise.psr.cn0_ref > 0.0 && c.noise.psr.cn0_ref <= 70.0, "gnss.noise.psr.cn0_ref must be in (0,70]");
  ck.nonneg("gnss.noise.psr.k_std", c.noise.psr.k_std);
  ck.positive("gnss.noise.psr.tau_s", c.noise.psr.tau_s);
  ck.positive("gnss.noise.psr.decim_s", c.noise.psr.decim_s);
  ck.require(finite(c.noise.psr.err_brdci) && c.noise.psr.err_brdci >= 0.0 && c.noise.psr.err_brdci <= 1.0, "gnss.noise.psr.err_brdci must be in [0,1]");
  ck.sysMap("gnss.noise.psr.sys_factor", c.noise.psr.sys_factor);
  ck.positive("gnss.noise.dopp.sigma0_mps", c.noise.dopp.sigma0_mps);
  ck.positive("gnss.noise.dopp.tau_s", c.noise.dopp.tau_s);
  ck.positive("gnss.noise.tdcp.sigma0_m", c.noise.tdcp.sigma0_m);
  ck.positive("gnss.noise.tdcp.interval_s", c.noise.tdcp.interval_s);
  ck.positive("gnss.noise.tdcp.sigma_rate_mps", c.noise.tdcp.sigma_rate_mps);
  ck.nonneg("gnss.noise.tdcp.sigma_windup_m", c.noise.tdcp.sigma_windup_m);
  ck.positive("gnss.noise.tdcp.max_span_s", c.noise.tdcp.max_span_s);
  ck.require(c.noise.tdcp.max_span_s >= c.noise.tdcp.interval_s, "gnss.noise.tdcp.max_span_s must be >= interval_s");
  ck.positive("gnss.noise.pvt.tau_pos_s", c.noise.pvt.tau_pos_s);
  ck.positive("gnss.noise.pvt.tau_vel_s", c.noise.pvt.tau_vel_s);
  ck.positive("gnss.noise.pvt.decim_s", c.noise.pvt.decim_s);
  ck.positive("gnss.noise.pvt.scale", c.noise.pvt.scale);
  ck.positive("gnss.noise.scale.psr", c.noise.scale.psr);
  ck.positive("gnss.noise.scale.dopp", c.noise.scale.dopp);
  ck.positive("gnss.noise.scale.tdcp", c.noise.scale.tdcp);

  // ---- process ----
  ck.positive("gnss.process.clock.S_b", c.process.clock.S_b);
  ck.positive("gnss.process.clock.S_d", c.process.clock.S_d);
  ck.nonneg("gnss.process.isb_q", c.process.isb_q);
  ck.nonneg("gnss.process.frame.q_tilt_per_m", c.process.frame.q_tilt_per_m);
  ck.nonneg("gnss.process.frame.q_yaw_per_m", c.process.frame.q_yaw_per_m);
  ck.nonneg("gnss.process.frame.q_yaw_per_rad", c.process.frame.q_yaw_per_rad);
  ck.nonneg("gnss.process.frame.q_th_per_m", c.process.frame.q_th_per_m);
  ck.nonneg("gnss.process.frame.q_tv_per_m", c.process.frame.q_tv_per_m);
  ck.nonneg("gnss.process.frame.q_time", c.process.frame.q_time);

  // ---- init ----
  ck.atLeast("gnss.init.min_livo_frames", c.init.min_livo_frames, 1);
  ck.atLeast("gnss.init.min_sats", c.init.min_sats, 4);
  ck.positive("gnss.init.max_gdop", c.init.max_gdop);
  ck.atLeast("gnss.init.min_epochs", c.init.min_epochs, 1);
  ck.positive("gnss.init.min_window_s", c.init.min_window_s);
  ck.positive("gnss.init.min_hspeed_mps", c.init.min_hspeed_mps);
  ck.atLeast("gnss.init.min_moving_epochs", c.init.min_moving_epochs, 1);
  ck.positive("gnss.init.min_path_m", c.init.min_path_m);
  ck.positive("gnss.init.yaw_sigma_max_deg", c.init.yaw_sigma_max_deg);
  ck.positive("gnss.init.spp_sigma_floor_m", c.init.spp_sigma_floor_m);
  ck.nonneg("gnss.init.nlos_cn0_margin_db", c.init.nlos_cn0_margin_db);

  // ---- rejection ----
  ck.prob("gnss.rejection.alpha_w", c.rejection.alpha_w);
  ck.prob("gnss.rejection.alpha_nis", c.rejection.alpha_nis);
  ck.require(finite(c.rejection.max_reject_frac) && c.rejection.max_reject_frac > 0.0 && c.rejection.max_reject_frac <= 1.0,
             "gnss.rejection.max_reject_frac must be in (0,1]");
  ck.atLeast("gnss.rejection.min_meas.psr", c.rejection.min_meas.psr, 1);
  ck.atLeast("gnss.rejection.min_meas.dopp", c.rejection.min_meas.dopp, 1);
  ck.atLeast("gnss.rejection.min_meas.tdcp", c.rejection.min_meas.tdcp, 1);
  ck.positive("gnss.rejection.core_step_max_m", c.rejection.core_step_max_m);
  ck.positive("gnss.rejection.clock_jump_km", c.rejection.clock_jump_km);
  ck.atLeast("gnss.rejection.lockout_epochs", c.rejection.lockout_epochs, 0);

  // ---- slip ----
  ck.positive("gnss.slip.max_gap_s", c.slip.max_gap_s);
  ck.positive("gnss.slip.dopp_tol_cyc", c.slip.dopp_tol_cyc);
  ck.positive("gnss.slip.jerk_max", c.slip.jerk_max);
  ck.positive("gnss.slip.innov_abs_m", c.slip.innov_abs_m);
  ck.positive("gnss.slip.gf_tol_m", c.slip.gf_tol_m);

  // ---- integrity ----
  ck.prob("gnss.integrity.velocity_alarm.alpha", c.integrity.velocity_alarm.alpha);
  ck.positive("gnss.integrity.velocity_alarm.min_err_mps", c.integrity.velocity_alarm.min_err_mps);
  ck.atLeast("gnss.integrity.velocity_alarm.n_consec", c.integrity.velocity_alarm.n_consec, 1);
  ck.atLeast("gnss.integrity.velocity_alarm.n_clear", c.integrity.velocity_alarm.n_clear, 1);
  ck.positive("gnss.integrity.heading.min_speed", c.integrity.heading.min_speed);
  ck.atLeast("gnss.integrity.heading.n_consec", c.integrity.heading.n_consec, 1);
  ck.positive("gnss.integrity.heading.min_err_deg", c.integrity.heading.min_err_deg);
  ck.positive("gnss.integrity.degeneracy.lambda_enter", c.integrity.degeneracy.lambda_enter);
  ck.require(finite(c.integrity.degeneracy.lambda_exit) && c.integrity.degeneracy.lambda_exit >= c.integrity.degeneracy.lambda_enter,
             "gnss.integrity.degeneracy.lambda_exit must be >= lambda_enter (hysteresis)");
  ck.atLeast("gnss.integrity.degeneracy.window", c.integrity.degeneracy.window, 1);
  ck.positive("gnss.integrity.degeneracy.sigma_floor_m", c.integrity.degeneracy.sigma_floor_m);
  ck.positive("gnss.integrity.frame_jump.chi2", c.integrity.frame_jump.chi2);
  ck.positive("gnss.integrity.frame_jump.min_translation_m", c.integrity.frame_jump.min_translation_m);
  ck.positive("gnss.integrity.frame_jump.min_rotation_rad", c.integrity.frame_jump.min_rotation_rad);

  // ---- iterate ----
  ck.positive("gnss.iterate.yaw_sigma_deg", c.iterate.yaw_sigma_deg);
  ck.positive("gnss.iterate.dyaw_deg", c.iterate.dyaw_deg);
  ck.atLeast("gnss.iterate.max_iter", c.iterate.max_iter, 1);
  ck.atLeast("gnss.iterate.reacq_epochs", c.iterate.reacq_epochs, 0);

  // ---- outage ----
  ck.positive("gnss.outage.outage_s", c.outage.outage_s);
  ck.atLeast("gnss.outage.recover_min_sats", c.outage.recover_min_sats, 4);
  ck.nonneg("gnss.outage.recover_settle_s", c.outage.recover_settle_s);
  ck.positive("gnss.outage.reinit_after_s", c.outage.reinit_after_s);
  ck.positive("gnss.outage.reinit_sigma_t_m", c.outage.reinit_sigma_t_m);
  ck.positive("gnss.outage.reinit_sigma_psi_rad", c.outage.reinit_sigma_psi_rad);

  // ---- ablation consistency ----
  ck.require(!(c.ablation.tdcp_fixed_anchor && !c.ablation.frame_fixed_after_init),
             "gnss.ablation.tdcp_fixed_anchor requires frame_fixed_after_init (A-FA arm)");

  // ---- output ----
  ck.require(!c.output.frame_id.empty(), "gnss.output.frame_id is empty");
  ck.positive("gnss.output.smooth_trans_mps", c.output.smooth_trans_mps);
  ck.positive("gnss.output.smooth_yaw_dps", c.output.smooth_yaw_dps);

  return out;
}

std::string canonicalDump(const Config& c) {
  Dump d;
  d.add("enable", c.enable);
  d.add("update_enable", c.update_enable);
  d.add("mode", modeName(c.mode));
  d.add("update_slot", slotName(c.update_slot));
  d.add("use.psr", c.use.psr);
  d.add("use.dopp", c.use.dopp);
  d.add("use.tdcp", c.use.tdcp);
  d.add("use.pvt_vel", c.use.pvt_vel);
  d.add("source", sourceName(c.source));
  d.add("topics.meas", c.topics.meas);
  d.add("topics.ephem", c.topics.ephem);
  d.add("topics.glo_ephem", c.topics.glo_ephem);
  d.add("topics.iono", c.topics.iono);
  d.add("topics.pvt", c.topics.pvt);
  d.add("topics.tp", c.topics.tp);
  d.add("rinex.obs", c.rinex.obs);
  d.add("rinex.nav", c.rinex.nav);
  d.add("rinex.latency_s", c.rinex.latency_s);
  d.add("nav_rinex", c.nav_rinex);
  d.add("systems.gps", c.systems.gps);
  d.add("systems.glo", c.systems.glo);
  d.add("systems.gal", c.systems.gal);
  d.add("systems.bds", c.systems.bds);
  d.add("systems.qzs_as_gps", c.systems.qzs_as_gps);
  d.add("systems.glo_psr", c.systems.glo_psr);
  d.add("systems.bds_geo.use", c.systems.bds_geo.use);
  d.add("systems.bds_geo.psr_scale", c.systems.bds_geo.psr_scale);
  d.add("exclude_sats", c.exclude_sats);
  d.add("frames.lever_arm_frame", leverFrameName(c.frames.lever_arm_frame));
  d.add3("frames.lever_arm_xyz", c.frames.lever_arm_xyz);
  d.add("frames.lever_arm_sigma", c.frames.lever_arm_sigma);
  d.add("frames.pivot_distance_m", c.frames.pivot_distance_m);
  d.add("frames.sigma_dov_rad", c.frames.sigma_dov_rad);
  d.add("frames.sigma_lev_rad", c.frames.sigma_lev_rad);
  d.add("frames.gnss_updates_gravity", c.frames.gnss_updates_gravity);
  d.add("time.base", baseName(c.time.base));
  d.add("time.leap", c.time.leap);
  d.add("time.offset_s", c.time.offset_s);
  d.add("time.drift_ppm", c.time.drift_ppm);
  d.add("time.t_ref_gpst_s", c.time.t_ref_gpst_s);
  d.add("time.calib_file", c.time.calib_file);
  d.add("time.sigma_toff_s", c.time.sigma_toff_s);
  d.add("time.estimate_online", c.time.estimate_online);
  d.add("time.td_rw_s2_per_s", c.time.td_rw_s2_per_s);
  d.add("time.max_bridge_s", c.time.max_bridge_s);
  d.add("time.bridge_ring_cycles", c.time.bridge_ring_cycles);
  d.add("time.toff_monitor.enable", c.time.toff_monitor.enable);
  d.add("time.toff_monitor.window_s", c.time.toff_monitor.window_s);
  d.add("time.toff_monitor.min_abs_s", c.time.toff_monitor.min_abs_s);
  d.add("time.toff_monitor.n_sigma", c.time.toff_monitor.n_sigma);
  d.add("imu_bridge.acc_n", c.imu_bridge.acc_n);
  d.add("imu_bridge.gyr_n", c.imu_bridge.gyr_n);
  d.add("masks.el_deg", c.masks.el_deg);
  d.add("masks.cn0_psr", c.masks.cn0_psr);
  d.add("masks.cn0_dopp", c.masks.cn0_dopp);
  d.add("masks.cn0_cp", c.masks.cn0_cp);
  d.add("masks.track_psr", c.masks.track_psr);
  d.add("masks.track_cp", c.masks.track_cp);
  d.add("masks.eph_age_s", c.masks.eph_age_s);
  d.add("masks.max_psr_std_m", c.masks.max_psr_std_m);
  d.add("masks.max_dopp_std_hz", c.masks.max_dopp_std_hz);
  d.add("masks.max_cp_std_m", c.masks.max_cp_std_m);
  d.add("masks.psr_min_m", c.masks.psr_min_m);
  d.add("masks.psr_max_m", c.masks.psr_max_m);
  d.add("masks.max_range_rate_mps", c.masks.max_range_rate_mps);
  d.add("noise.psr.sigma0_m", c.noise.psr.sigma0_m);
  d.add("noise.psr.cn0_ref", c.noise.psr.cn0_ref);
  d.add("noise.psr.k_std", c.noise.psr.k_std);
  d.add("noise.psr.tau_s", c.noise.psr.tau_s);
  d.add("noise.psr.decim_s", c.noise.psr.decim_s);
  d.add("noise.psr.err_brdci", c.noise.psr.err_brdci);
  d.add("noise.psr.sys_factor", c.noise.psr.sys_factor);
  d.add("noise.dopp.sigma0_mps", c.noise.dopp.sigma0_mps);
  d.add("noise.dopp.tau_s", c.noise.dopp.tau_s);
  d.add("noise.dopp.exclusive_with_tdcp", c.noise.dopp.exclusive_with_tdcp);
  d.add("noise.dopp.consider_tilt", c.noise.dopp.consider_tilt);   // A66
  d.add("noise.dopp.project_tilt", c.noise.dopp.project_tilt);     // A70
  d.add("noise.tdcp.sigma0_m", c.noise.tdcp.sigma0_m);
  d.add("noise.tdcp.interval_s", c.noise.tdcp.interval_s);
  d.add("noise.tdcp.sigma_rate_mps", c.noise.tdcp.sigma_rate_mps);
  d.add("noise.tdcp.sigma_windup_m", c.noise.tdcp.sigma_windup_m);
  d.add("noise.tdcp.max_span_s", c.noise.tdcp.max_span_s);
  d.add("noise.tdcp.chain_pairs", c.noise.tdcp.chain_pairs);
  d.add("noise.tdcp.phase_states", c.noise.tdcp.phase_states);
  d.add("noise.pvt.tau_pos_s", c.noise.pvt.tau_pos_s);
  d.add("noise.pvt.tau_vel_s", c.noise.pvt.tau_vel_s);
  d.add("noise.pvt.decim_s", c.noise.pvt.decim_s);
  d.add("noise.pvt.scale", c.noise.pvt.scale);
  d.add("noise.scale.psr", c.noise.scale.psr);
  d.add("noise.scale.dopp", c.noise.scale.dopp);
  d.add("noise.scale.tdcp", c.noise.scale.tdcp);
  d.add("process.clock.S_b", c.process.clock.S_b);
  d.add("process.clock.S_d", c.process.clock.S_d);
  d.add("process.clock.S_b_min_windup", c.process.clock.S_b_min_windup);
  d.add("process.isb_q", c.process.isb_q);
  d.add("process.frame.q_tilt_per_m", c.process.frame.q_tilt_per_m);
  d.add("process.frame.q_yaw_per_m", c.process.frame.q_yaw_per_m);
  d.add("process.frame.q_yaw_per_rad", c.process.frame.q_yaw_per_rad);
  d.add("process.frame.q_th_per_m", c.process.frame.q_th_per_m);
  d.add("process.frame.q_tv_per_m", c.process.frame.q_tv_per_m);
  d.add("process.frame.q_time", c.process.frame.q_time);
  d.add("init.min_livo_frames", c.init.min_livo_frames);
  d.add("init.min_sats", c.init.min_sats);
  d.add("init.max_gdop", c.init.max_gdop);
  d.add("init.min_epochs", c.init.min_epochs);
  d.add("init.min_window_s", c.init.min_window_s);
  d.add("init.min_hspeed_mps", c.init.min_hspeed_mps);
  d.add("init.min_moving_epochs", c.init.min_moving_epochs);
  d.add("init.min_path_m", c.init.min_path_m);
  d.add("init.yaw_sigma_max_deg", c.init.yaw_sigma_max_deg);
  d.add("init.spp_sigma_floor_m", c.init.spp_sigma_floor_m);
  d.add("init.nlos_cn0_margin_db", c.init.nlos_cn0_margin_db);
  d.add("rejection.alpha_w", c.rejection.alpha_w);
  d.add("rejection.alpha_nis", c.rejection.alpha_nis);
  d.add("rejection.max_reject_frac", c.rejection.max_reject_frac);
  d.add("rejection.min_meas.psr", c.rejection.min_meas.psr);
  d.add("rejection.min_meas.dopp", c.rejection.min_meas.dopp);
  d.add("rejection.min_meas.tdcp", c.rejection.min_meas.tdcp);
  d.add("rejection.post_check", c.rejection.post_check);
  d.add("rejection.core_step_max_m", c.rejection.core_step_max_m);
  d.add("rejection.clock_jump_km", c.rejection.clock_jump_km);
  d.add("rejection.lockout_epochs", c.rejection.lockout_epochs);
  d.add("slip.lli", c.slip.lli);
  d.add("slip.half_cycle", c.slip.half_cycle);
  d.add("slip.max_gap_s", c.slip.max_gap_s);
  d.add("slip.dopp_tol_cyc", c.slip.dopp_tol_cyc);
  d.add("slip.jerk_max", c.slip.jerk_max);
  d.add("slip.innov_abs_m", c.slip.innov_abs_m);
  d.add("slip.gf_tol_m", c.slip.gf_tol_m);
  d.add("integrity.velocity_alarm.enable", c.integrity.velocity_alarm.enable);
  d.add("integrity.velocity_alarm.alpha", c.integrity.velocity_alarm.alpha);
  d.add("integrity.velocity_alarm.min_err_mps", c.integrity.velocity_alarm.min_err_mps);
  d.add("integrity.velocity_alarm.n_consec", c.integrity.velocity_alarm.n_consec);
  d.add("integrity.velocity_alarm.n_clear", c.integrity.velocity_alarm.n_clear);
  d.add("integrity.heading.enable", c.integrity.heading.enable);
  d.add("integrity.heading.min_speed", c.integrity.heading.min_speed);
  d.add("integrity.heading.n_consec", c.integrity.heading.n_consec);
  d.add("integrity.heading.min_err_deg", c.integrity.heading.min_err_deg);
  d.add("integrity.degeneracy.lambda_enter", c.integrity.degeneracy.lambda_enter);
  d.add("integrity.degeneracy.lambda_exit", c.integrity.degeneracy.lambda_exit);
  d.add("integrity.degeneracy.window", c.integrity.degeneracy.window);
  d.add("integrity.degeneracy.floor_enable", c.integrity.degeneracy.floor_enable);
  d.add("integrity.degeneracy.sigma_floor_m", c.integrity.degeneracy.sigma_floor_m);
  d.add("integrity.frame_jump.enable", c.integrity.frame_jump.enable);
  d.add("integrity.frame_jump.chi2", c.integrity.frame_jump.chi2);
  d.add("integrity.frame_jump.min_translation_m", c.integrity.frame_jump.min_translation_m);
  d.add("integrity.frame_jump.min_rotation_rad", c.integrity.frame_jump.min_rotation_rad);
  d.add("iterate.yaw_sigma_deg", c.iterate.yaw_sigma_deg);
  d.add("iterate.dyaw_deg", c.iterate.dyaw_deg);
  d.add("iterate.max_iter", c.iterate.max_iter);
  d.add("iterate.reacq_epochs", c.iterate.reacq_epochs);
  d.add("outage.outage_s", c.outage.outage_s);
  d.add("outage.recover_min_sats", c.outage.recover_min_sats);
  d.add("outage.recover_settle_s", c.outage.recover_settle_s);
  d.add("outage.reinit_after_s", c.outage.reinit_after_s);
  d.add("outage.reinit_sigma_t_m", c.outage.reinit_sigma_t_m);
  d.add("outage.reinit_sigma_psi_rad", c.outage.reinit_sigma_psi_rad);
  d.add("ablation.frame_fixed_after_init", c.ablation.frame_fixed_after_init);
  d.add("ablation.tdcp_fixed_anchor", c.ablation.tdcp_fixed_anchor);
  d.add("ablation.disable_l4_l5", c.ablation.disable_l4_l5);
  d.add("output.frame_id", c.output.frame_id);
  d.add("output.smooth_trans_mps", c.output.smooth_trans_mps);
  d.add("output.smooth_yaw_dps", c.output.smooth_yaw_dps);
  d.add("output.odom_output", c.output.odom_output);
  d.add("output.sat_csv", c.output.sat_csv);
  d.add("ephem.ura_semantics", uraName(c.ephem.ura_semantics));
  d.add("ephem.gal_prefer_inav", c.ephem.gal_prefer_inav);
  return d.str();
}

}  // namespace livo_gnss
