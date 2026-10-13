// SPDX-License-Identifier: GPL-2.0-only
// fastlivo_replay — deterministic offline driver (DESIGN §13, IMPL_PLAN 3, AMENDMENTS A13). Owner: WP-f.
//
//   * parameters: the caller (launch/m3dgr_avia_replay.launch or a script) starts a private roscore and
//     loads the LIVO yaml (global namespace, as the live node reads it) and the camera yaml into
//     /laserMapping (vikit camera_loader); this node is named laserMapping like the live one;
//   * feeding: rosbag::View in *record-time* order (= rosbag play order; header order would put a LiDAR
//     frame before its own IMU samples), every message dispatched to the same LIVMapper callbacks the
//     live node subscribes with — livox_pcl_cbk / standard_pcl_cbk / imu_cbk / img_cbk / img_compressed_cbk;
//     every other message goes to GnssRosBridge::handleReplayMessage, which takes the configured GNSS topics
//     (raw serialised bytes + message_definition, DESIGN §12.2) and ignores the rest (P3-B);
//   * after every message `while (mapper.processOnce()) { write outputs }` — the loop body of run();
//   * perturbation seed k: the view starts at the *record time of the k-th LiDAR message* (record order) at or
//     after bag_begin + start_offset_s. imu_cbk / img_cbk drop everything received before the first LiDAR
//     frame (0d2c034 LIVMapper.cpp :801 / :878), so the only thing a start time can change is which LiDAR
//     frame comes first — indexing LiDAR frames makes seeds distinct by construction, whereas the DESIGN §13
//     "k×0.05 s" offsets fell pairwise onto the same 10 Hz frame (review M1; deviation registered in
//     INTERFACE_CHANGES, AMENDMENTS request). start_offset_s is a fixed trim, not a second seed knob;
//   * outputs after each cycle, read from LIVMapper's public members (A13): state_propagat (prior),
//     _state (posterior), LidarMeasures.lio_vio_flg, voxelmap_manager->effct_feat_num_ →
//     state_trace.txt (hexfloat), traj_W.tum (step end), filter_cov.csv (prior/postLIO/postVIO), timing.csv;
//     run_manifest.json and replay_stats.json at the end. The manifest carries the replay statistics too
//     (ManifestInfo::extra_json, AMENDMENTS A40), so it alone identifies a run; replay_stats.json repeats them and is
//     written last: its "complete": true is the completion criterion, together with the "fastlivo_replay: done"
//     line and the node's exit code (docs/dev/ref_r_gates/replay_run.sh).
// No wall clock reaches the estimator; timing.csv is measured around processOnce() and is never compared.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fstream>
#include <limits>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include <unistd.h>

#include <Eigen/Dense>
#include <XmlRpcValue.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/message_instance.h>
#include <rosbag/query.h>
#include <rosbag/view.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include "LIVMapper.h"
#include "livo_gnss/core_state.hpp"
#include "livo_gnss/diagnostics.hpp"
#include "livo_gnss_ros/gnss_ros_bridge.hpp"
#include "livo_gnss_ros/outputs.hpp"

namespace {

// ---------------------------------------------------------------------------------------------
// SHA-256 (FIPS 180-4) — for the bag and the configuration dump in run_manifest.json. Self-contained so
// the replay driver needs no OpenSSL link line (CMake is frozen, OWNERSHIP §3).
// ---------------------------------------------------------------------------------------------
class Sha256 {
 public:
  Sha256() { reset(); }
  void reset() {
    static const uint32_t h0[8] = {0x6a09e667u, 0xbb67ae85u, 0x3c6ef372u, 0xa54ff53au, 0x510e527fu, 0x9b05688cu, 0x1f83d9abu, 0x5be0cd19u};
    std::memcpy(h_, h0, sizeof h_);
    len_ = 0;
    buf_len_ = 0;
  }
  void update(const uint8_t* p, size_t n) {
    len_ += n;
    while (n > 0) {
      const size_t take = std::min(n, sizeof(buf_) - buf_len_);
      std::memcpy(buf_ + buf_len_, p, take);
      buf_len_ += take;
      p += take;
      n -= take;
      if (buf_len_ == sizeof(buf_)) {
        block(buf_);
        buf_len_ = 0;
      }
    }
  }
  std::string hex() {
    const uint64_t bits = len_ * 8;
    uint8_t pad = 0x80;
    update(&pad, 1);
    const uint8_t zero = 0;
    while (buf_len_ != 56) update(&zero, 1);
    uint8_t lb[8];
    for (int i = 7; i >= 0; --i) lb[7 - i] = static_cast<uint8_t>(bits >> (8 * i));
    update(lb, 8);
    char out[65];
    for (int i = 0; i < 8; ++i) std::snprintf(out + 8 * i, 9, "%08x", h_[i]);
    return std::string(out, 64);
  }

 private:
  static uint32_t rotr(uint32_t x, int n) { return (x >> n) | (x << (32 - n)); }
  void block(const uint8_t* p) {
    static const uint32_t k[64] = {
        0x428a2f98u, 0x71374491u, 0xb5c0fbcfu, 0xe9b5dba5u, 0x3956c25bu, 0x59f111f1u, 0x923f82a4u, 0xab1c5ed5u, 0xd807aa98u, 0x12835b01u,
        0x243185beu, 0x550c7dc3u, 0x72be5d74u, 0x80deb1feu, 0x9bdc06a7u, 0xc19bf174u, 0xe49b69c1u, 0xefbe4786u, 0x0fc19dc6u, 0x240ca1ccu,
        0x2de92c6fu, 0x4a7484aau, 0x5cb0a9dcu, 0x76f988dau, 0x983e5152u, 0xa831c66du, 0xb00327c8u, 0xbf597fc7u, 0xc6e00bf3u, 0xd5a79147u,
        0x06ca6351u, 0x14292967u, 0x27b70a85u, 0x2e1b2138u, 0x4d2c6dfcu, 0x53380d13u, 0x650a7354u, 0x766a0abbu, 0x81c2c92eu, 0x92722c85u,
        0xa2bfe8a1u, 0xa81a664bu, 0xc24b8b70u, 0xc76c51a3u, 0xd192e819u, 0xd6990624u, 0xf40e3585u, 0x106aa070u, 0x19a4c116u, 0x1e376c08u,
        0x2748774cu, 0x34b0bcb5u, 0x391c0cb3u, 0x4ed8aa4au, 0x5b9cca4fu, 0x682e6ff3u, 0x748f82eeu, 0x78a5636fu, 0x84c87814u, 0x8cc70208u,
        0x90befffau, 0xa4506cebu, 0xbef9a3f7u, 0xc67178f2u};
    uint32_t w[64];
    for (int i = 0; i < 16; ++i)
      w[i] = (uint32_t(p[4 * i]) << 24) | (uint32_t(p[4 * i + 1]) << 16) | (uint32_t(p[4 * i + 2]) << 8) | uint32_t(p[4 * i + 3]);
    for (int i = 16; i < 64; ++i) {
      const uint32_t s0 = rotr(w[i - 15], 7) ^ rotr(w[i - 15], 18) ^ (w[i - 15] >> 3);
      const uint32_t s1 = rotr(w[i - 2], 17) ^ rotr(w[i - 2], 19) ^ (w[i - 2] >> 10);
      w[i] = w[i - 16] + s0 + w[i - 7] + s1;
    }
    uint32_t a = h_[0], b = h_[1], c = h_[2], d = h_[3], e = h_[4], f = h_[5], g = h_[6], h = h_[7];
    for (int i = 0; i < 64; ++i) {
      const uint32_t S1 = rotr(e, 6) ^ rotr(e, 11) ^ rotr(e, 25);
      const uint32_t ch = (e & f) ^ (~e & g);
      const uint32_t t1 = h + S1 + ch + k[i] + w[i];
      const uint32_t S0 = rotr(a, 2) ^ rotr(a, 13) ^ rotr(a, 22);
      const uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
      const uint32_t t2 = S0 + maj;
      h = g; g = f; f = e; e = d + t1; d = c; c = b; b = a; a = t1 + t2;
    }
    h_[0] += a; h_[1] += b; h_[2] += c; h_[3] += d; h_[4] += e; h_[5] += f; h_[6] += g; h_[7] += h;
  }
  uint32_t h_[8];
  uint64_t len_;
  uint8_t buf_[64];
  size_t buf_len_;
};

std::string sha256OfString(const std::string& s) {
  Sha256 h;
  h.update(reinterpret_cast<const uint8_t*>(s.data()), s.size());
  return h.hex();
}

std::string sha256OfFile(const std::string& path) {
  std::FILE* f = std::fopen(path.c_str(), "rb");
  if (!f) return "";
  Sha256 h;
  std::vector<uint8_t> buf(1 << 20);
  size_t n;
  while ((n = std::fread(buf.data(), 1, buf.size(), f)) > 0) h.update(buf.data(), n);
  std::fclose(f);
  return h.hex();
}

// ---------------------------------------------------------------------------------------------
// Canonical JSON of the parameter server (sorted keys, %.17g doubles): the configuration that actually
// drove this run, hashed into run_manifest.json.
// ---------------------------------------------------------------------------------------------
void jsonEscape(std::string& o, const std::string& s) {
  o += '"';
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
  o += '"';
}

// Keys that identify a *run*, not a configuration: set per run by launch/m3dgr_avia_replay.launch. They are
// left out of the hashed dump (config_sha256 must be equal for the same configuration across seeds and output
// directories, review m5) but kept in config_dump.
const char* const kRunIdentityKeys[] = {"replay/bag", "replay/seed", "replay/start_offset_s", "replay/duration_s",
                                        "output/dir", "evo/seq_name",
                                        // P3-B: per-sequence GNSS inputs (the day's BRDC file, the sequence's time
                                        // calibration, RINEX source files) identify the data, not the configuration.
                                        "gnss/nav_rinex", "gnss/time/calib_file", "gnss/rinex/obs", "gnss/rinex/nav"};

bool isRunIdentityKey(const std::string& path) {
  for (const char* k : kRunIdentityKeys)
    if (path == k) return true;
  return false;
}

// `path` = slash-joined key path of `v` ("replay/seed"); with strip_run_keys the run-identity keys are skipped.
void canonicalJson(std::string& o, XmlRpc::XmlRpcValue& v, const std::string& path, bool strip_run_keys) {
  switch (v.getType()) {
    case XmlRpc::XmlRpcValue::TypeBoolean: o += static_cast<bool>(v) ? "true" : "false"; break;
    case XmlRpc::XmlRpcValue::TypeInt: o += std::to_string(static_cast<int>(v)); break;
    case XmlRpc::XmlRpcValue::TypeDouble: {
      char b[40];
      std::snprintf(b, sizeof b, "%.17g", static_cast<double>(v));
      o += b;
      break;
    }
    case XmlRpc::XmlRpcValue::TypeString: jsonEscape(o, static_cast<std::string>(v)); break;
    case XmlRpc::XmlRpcValue::TypeArray: {
      o += '[';
      for (int i = 0; i < v.size(); ++i) {
        if (i) o += ',';
        canonicalJson(o, v[i], path, strip_run_keys);
      }
      o += ']';
      break;
    }
    case XmlRpc::XmlRpcValue::TypeStruct: {
      std::map<std::string, XmlRpc::XmlRpcValue> sorted;   // XmlRpcValue's struct is already a std::map; be explicit
      for (auto it = v.begin(); it != v.end(); ++it) sorted[it->first] = it->second;
      o += '{';
      bool first = true;
      for (auto& kv : sorted) {
        const std::string sub = path.empty() ? kv.first : path + "/" + kv.first;
        if (strip_run_keys && isRunIdentityKey(sub)) continue;
        if (!first) o += ',';
        first = false;
        jsonEscape(o, kv.first);
        o += ':';
        canonicalJson(o, kv.second, sub, strip_run_keys);
      }
      o += '}';
      break;
    }
    default: jsonEscape(o, v.toXml()); break;
  }
}

// Parameter tree minus roslaunch's own bookkeeping keys (they change per launch and carry no configuration);
// strip_run_keys additionally drops kRunIdentityKeys (the hashed variant).
std::string paramDump(bool strip_run_keys) {
  XmlRpc::XmlRpcValue root;
  if (!ros::param::get("/", root) || root.getType() != XmlRpc::XmlRpcValue::TypeStruct) return "{}";
  for (const char* k : {"roslaunch", "run_id", "rosdistro", "rosversion"})
    if (root.hasMember(k)) root[k] = XmlRpc::XmlRpcValue();   // XmlRpcValue has no erase: blank the entry
  std::string o;
  // Rebuild without the blanked members.
  std::map<std::string, XmlRpc::XmlRpcValue> kept;
  for (auto it = root.begin(); it != root.end(); ++it)
    if (it->second.getType() != XmlRpc::XmlRpcValue::TypeInvalid) kept[it->first] = it->second;
  o += '{';
  bool first = true;
  for (auto& kv : kept) {
    if (strip_run_keys && isRunIdentityKey(kv.first)) continue;
    if (!first) o += ',';
    first = false;
    jsonEscape(o, kv.first);
    o += ':';
    canonicalJson(o, kv.second, kv.first, strip_run_keys);
  }
  o += '}';
  return o;
}

// ---------------------------------------------------------------------------------------------
// Identity of the binary that produced a run: sha256 of the executable and of every mapped shared object
// under the same devel prefix (<devel>/lib/fast_livo/fastlivo_replay → <devel>/lib/*.so, i.e. liblaser_mapping,
// libvio, liblio, libimu_proc, liblivo_gnss*, vikit). A git label passed by a runner can be wrong (review M2:
// a diag binary labelled 68af88c was built from HEAD); these hashes cannot.
// ---------------------------------------------------------------------------------------------
struct BuildFile {
  std::string path, sha256;
};

std::vector<BuildFile> buildFiles(const char* argv0) {
  std::vector<BuildFile> out;
  // Under valgrind /proc/self/exe is the memcheck tool, so prefer an absolute argv[0] (roslaunch always passes
  // the full path); fall back to /proc/self/exe.
  std::string exe_path = (argv0 && argv0[0] == '/') ? std::string(argv0) : std::string();
  if (exe_path.empty()) {
    char exe[4096];
    const ssize_t n = readlink("/proc/self/exe", exe, sizeof exe - 1);
    if (n <= 0) return out;
    exe_path.assign(exe, static_cast<size_t>(n));
  }
  // <devel>/lib/fast_livo/fastlivo_replay → prefix <devel>/lib
  std::string prefix = exe_path;
  for (int up = 0; up < 2; ++up) {
    const size_t s = prefix.rfind('/');
    if (s == std::string::npos) return out;
    prefix.erase(s);
  }
  std::vector<std::string> paths{exe_path};
  std::ifstream maps("/proc/self/maps");
  std::string line;
  while (std::getline(maps, line)) {
    const size_t slash = line.find('/');
    if (slash == std::string::npos) continue;
    const std::string p = line.substr(slash);
    if (p.compare(0, prefix.size() + 1, prefix + "/") != 0) continue;
    if (p.find(".so") == std::string::npos) continue;
    if (std::find(paths.begin(), paths.end(), p) == paths.end()) paths.push_back(p);
  }
  std::sort(paths.begin() + 1, paths.end());
  for (const auto& p : paths) out.push_back({p, sha256OfFile(p)});
  return out;
}

// One hash over "<path> <sha256>\n" lines (paths relative to the devel prefix so two checkouts of the same
// build compare equal).
std::string buildSha256(const std::vector<BuildFile>& files, std::string* listing) {
  std::string s;
  if (!files.empty()) {
    std::string prefix = files[0].path;
    for (int up = 0; up < 2; ++up) prefix.erase(prefix.rfind('/'));
    for (const auto& f : files) {
      const std::string rel = (f.path.compare(0, prefix.size() + 1, prefix + "/") == 0) ? f.path.substr(prefix.size() + 1) : f.path;
      s += rel + " " + f.sha256 + "\n";
    }
  }
  if (listing) *listing = s;
  return sha256OfString(s);
}

// ---------------------------------------------------------------------------------------------
// Build identity for run_manifest.json.
// ---------------------------------------------------------------------------------------------
#define FL2G_STR2(x) #x
#define FL2G_STR(x) FL2G_STR2(x)

// Runtime probe of floating-point contraction in *this* TU: (1+e)(1-e)-1 with e=2^-27 is exactly -2^-54 when
// fused and rounds to 0 when the product is rounded first. volatile forces the loads, not the contraction.
bool fpContractionProbe() {
  volatile double a = 1.0 + std::ldexp(1.0, -27);
  volatile double b = 1.0 - std::ldexp(1.0, -27);
  volatile double c = -1.0;
  const double r = a * b + c;
  return r != 0.0;
}

std::string compileDefs() {
  std::string s;
#ifdef MP_EN
  s += "MP_EN ";
#endif
#ifdef MP_PROC_NUM
  s += "MP_PROC_NUM=" FL2G_STR(MP_PROC_NUM) " ";
#endif
#ifdef EIGEN_DONT_PARALLELIZE
  s += "EIGEN_DONT_PARALLELIZE ";
#endif
#ifdef _OPENMP
  s += "_OPENMP=" FL2G_STR(_OPENMP) " ";
#endif
#ifdef NDEBUG
  s += "NDEBUG ";
#endif
#ifdef __OPTIMIZE__
  s += "__OPTIMIZE__ ";
#endif
#ifdef __AVX512F__
  s += "__AVX512F__ ";
#endif
#ifdef __AVX2__
  s += "__AVX2__ ";
#endif
#ifdef __FMA__
  s += "__FMA__ ";
#endif
  s += fpContractionProbe() ? "fp_contract_probe=fused" : "fp_contract_probe=unfused";
  return s;
}

std::string profileName() {
#if defined(MP_EN)
  return "default";
#elif defined(EIGEN_DONT_PARALLELIZE)
  return "eval";
#else
  return "omp_off";
#endif
}

double wallNow() {
  return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
}

std::string envOr(const char* name, const std::string& dflt) {
  const char* v = std::getenv(name);
  return v ? std::string(v) : dflt;
}

// Inside docker gethostname() is the container id (review m5); the runner passes the physical host as FL2G_HOST.
std::string hostName() {
  const std::string env = envOr("FL2G_HOST", "");
  if (!env.empty()) return env;
  char b[256] = {0};
  if (gethostname(b, sizeof b - 1) != 0) return "unknown";
  return std::string(b) + " (container)";
}

// ---------------------------------------------------------------------------------------------
// Arguments (CLI overrides the replay/* parameters; the launch file passes only the build identity).
// ---------------------------------------------------------------------------------------------
struct Args {
  std::string bag, output_dir, arm = "U";
  double start_offset_s = 0.0, duration_s = -1.0;
  int seed = 0;
  bool have_bag = false, have_offset = false, have_duration = false, have_seed = false, have_output = false;
  std::string git_describe, git_dirty, image_digest, bag_sha256;
  bool help = false;
  std::vector<std::string> unknown;
  // P3-B: GNSS inputs of the run, applied before LIVMapper reads its parameters.
  std::vector<std::string> param_files;   // --param-file: rosparam overlays (config/arms/*.yaml, gate configs), in order
  std::vector<std::string> nav_rinex;     // --nav-rinex: the sequence day's BRDC file(s) → gnss/nav_rinex (A1/A38)
  bool no_gnss_messages = false;          // --no-gnss-messages: GNSS topics of the bag never reach the bridge (G-无数据)
};

void usage() {
  std::fprintf(stderr,
               "fastlivo_replay [--bag <file.bag>] [--start-offset <s>] [--duration <s>] [--seed <k>] [--output-dir <dir>]\n"
               "                [--arm <name>] [--git-describe <s>] [--git-dirty <0|1>] [--image-digest <s>] [--bag-sha256 <hex>]\n"
               "                [--param-file <yaml>]... [--nav-rinex <rnx>]... [--no-gnss-messages]\n"
               "Defaults come from rosparam: replay/{bag,start_offset_s,duration_s,seed}, output/{dir,traj_w,state_trace,timing},\n"
               "debug/cov_csv. View start = record time of the seed-th LiDAR message (record order) at or after\n"
               "bag begin + start_offset_s; seed is the only perturbation knob (DESIGN §13 as amended).\n"
               "GNSS (P3-B): --param-file loads a rosparam overlay (e.g. config/arms/T.yaml) before the estimator reads its\n"
               "parameters, --nav-rinex sets gnss/nav_rinex (repeatable), --no-gnss-messages keeps the bag's GNSS topics from\n"
               "the bridge (no-data gate).\n");
}

Args parse(int argc, char** argv) {
  Args a;
  for (int i = 1; i < argc; ++i) {
    const std::string s = argv[i];
    auto val = [&]() -> const char* { return (i + 1 < argc) ? argv[++i] : ""; };
    if (s == "--help" || s == "-h") a.help = true;
    else if (s == "--bag") { a.bag = val(); a.have_bag = true; }
    else if (s == "--start-offset") { a.start_offset_s = std::atof(val()); a.have_offset = true; }
    else if (s == "--duration") { a.duration_s = std::atof(val()); a.have_duration = true; }
    else if (s == "--seed") { a.seed = std::atoi(val()); a.have_seed = true; }
    else if (s == "--output-dir") { a.output_dir = val(); a.have_output = true; }
    else if (s == "--arm") a.arm = val();
    else if (s == "--git-describe") a.git_describe = val();
    else if (s == "--git-dirty") a.git_dirty = val();
    else if (s == "--image-digest") a.image_digest = val();
    else if (s == "--bag-sha256") a.bag_sha256 = val();
    else if (s == "--param-file") a.param_files.push_back(val());
    else if (s == "--nav-rinex") a.nav_rinex.push_back(val());
    else if (s == "--no-gnss-messages") a.no_gnss_messages = true;
    else a.unknown.push_back(s);
  }
  return a;
}

// ---------------------------------------------------------------------------------------------
// StatesGroup → CoreState, member by member (the same mapping as livo_host_view::toCore, kept local so the
// ref-R driver depends on nothing P3 still has to implement).
// ---------------------------------------------------------------------------------------------
livo_gnss::CoreState toCoreLocal(const StatesGroup& s) {
  livo_gnss::CoreState c;
  c.R = s.rot_end;
  c.p = s.pos_end;
  c.v = s.vel_end;
  c.bg = s.bias_g;
  c.ba = s.bias_a;
  c.g = s.gravity;
  c.inv_expo = s.inv_expo_time;
  c.P = s.cov;
  return c;
}

// filter_cov.csv row (DESIGN §15): √diag(P) of `s`, eigen-decomposition of its position/rotation blocks,
// min diag and condition number of the cycle prior `prior` (R3-16). No augmented block in ref-R.
livo_gnss::CovRecord covRow(double t_k, const char* stage, int effct_feat_num, const StatesGroup& s, const StatesGroup& prior) {
  livo_gnss::CovRecord r;
  r.t_k = t_k;
  r.stage = stage;
  r.effct_feat_num = effct_feat_num;
  for (int i = 0; i < 19; ++i) r.host_sigma[i] = std::sqrt(std::max(0.0, s.cov(i, i)));
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> ep(s.cov.block<3, 3>(3, 3));
  r.pos_eig = ep.eigenvalues();
  r.pos_eigvec = ep.eigenvectors();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> er(s.cov.block<3, 3>(0, 0));
  r.rot_eig = er.eigenvalues();
  r.rot_eigvec = er.eigenvectors();
  r.min_diag_prior = prior.cov.diagonal().minCoeff();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 19, 19>> ea(prior.cov, Eigen::EigenvaluesOnly);
  const double lmin = ea.eigenvalues().minCoeff(), lmax = ea.eigenvalues().maxCoeff();
  r.cond_prior = (lmin > 0.0) ? lmax / lmin : std::numeric_limits<double>::infinity();
  return r;
}

struct Stats {
  uint64_t n_msgs = 0, n_lidar = 0, n_imu = 0, n_img = 0, n_img_raw = 0, n_type_mismatch = 0;
  std::map<std::string, uint64_t> other;   // topic → count (GNSS topics counted here this round)
  uint64_t cycles = 0, cycles_lio = 0, cycles_vio = 0, cycles_lo = 0;
  double t_first_msg = 0.0, t_last_msg = 0.0;
  uint64_t n_gnss_routed = 0;              // P3-B: messages of the configured GNSS topics handed to the bridge
};

// P3-B: one rosparam overlay (config/arms/*.yaml, gate configurations) loaded into the global namespace with the
// rosparam tool itself — the same YAML loader and the same leaf-wise merge as the launch file's <rosparam load>.
bool loadParamOverlay(const std::string& file) {
  if (file.empty() || file.find('\'') != std::string::npos) {
    std::fprintf(stderr, "fastlivo_replay: --param-file '%s': empty or contains a quote\n", file.c_str());
    return false;
  }
  if (!std::ifstream(file).good()) {
    std::fprintf(stderr, "fastlivo_replay: --param-file '%s': cannot read\n", file.c_str());
    return false;
  }
  const std::string cmd = "rosparam load '" + file + "' /";
  const int rc = std::system(cmd.c_str());
  if (rc != 0) {
    std::fprintf(stderr, "fastlivo_replay: '%s' failed (status %d)\n", cmd.c_str(), rc);
    return false;
  }
  std::fprintf(stderr, "fastlivo_replay: parameter overlay %s loaded\n", file.c_str());
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "laserMapping");   // same node name as the live node (vikit reads /laserMapping)
  Args a = parse(argc, argv);
  if (a.help) {
    usage();
    return 0;
  }
  for (const auto& u : a.unknown) std::fprintf(stderr, "fastlivo_replay: ignoring unknown argument '%s'\n", u.c_str());
  ros::NodeHandle nh;

  // ---- replay parameters (DESIGN §14 `replay:`), CLI overrides ----
  std::string bag_path, order, output_dir;
  double start_offset_s = 0.0, duration_s = -1.0, img_jitter_s = 0.0;
  int seed = 0;
  bool out_traj_w = true, out_state_trace = true, out_timing = true, cov_csv = true;
  nh.param<std::string>("replay/bag", bag_path, "");
  nh.param<double>("replay/start_offset_s", start_offset_s, 0.0);
  nh.param<double>("replay/duration_s", duration_s, -1.0);
  nh.param<std::string>("replay/order", order, "record");
  nh.param<double>("replay/img_jitter_s", img_jitter_s, 0.0);
  nh.param<int>("replay/seed", seed, 0);
  nh.param<std::string>("output/dir", output_dir, "");
  nh.param<bool>("output/traj_w", out_traj_w, true);
  nh.param<bool>("output/state_trace", out_state_trace, true);
  nh.param<bool>("output/timing", out_timing, true);
  nh.param<bool>("debug/cov_csv", cov_csv, true);
  if (a.have_bag) bag_path = a.bag;
  if (a.have_offset) start_offset_s = a.start_offset_s;
  if (a.have_duration) duration_s = a.duration_s;
  if (a.have_seed) seed = a.seed;
  if (a.have_output) output_dir = a.output_dir;
  if (bag_path.empty()) {
    std::fprintf(stderr, "fastlivo_replay: no bag (replay/bag or --bag)\n");
    usage();
    return 2;
  }
  if (order != "record") {
    std::fprintf(stderr, "fastlivo_replay: replay/order='%s' not supported (only 'record', DESIGN §13)\n", order.c_str());
    return 2;
  }
  if (img_jitter_s != 0.0) {
    std::fprintf(stderr, "fastlivo_replay: replay/img_jitter_s != 0 is not implemented in this round (second CRN source, DESIGN §13)\n");
    return 2;
  }
  if (seed < 0) {
    std::fprintf(stderr, "fastlivo_replay: seed must be >= 0\n");
    return 2;
  }
  if (start_offset_s < 0.0) {
    std::fprintf(stderr, "fastlivo_replay: replay/start_offset_s must be >= 0\n");
    return 2;
  }
  if (start_offset_s != 0.0 && seed != 0)
    std::fprintf(stderr,
                 "fastlivo_replay: WARNING both replay/start_offset_s=%.3f and seed=%d are set; the seed selects the %d-th LiDAR "
                 "frame after the offset. The perturbation knob is the seed alone (review m4); keep start_offset_s fixed per arm.\n",
                 start_offset_s, seed, seed);

  const double t_start_wall = wallNow();

  // ---- GNSS inputs of this run (P3-B), before LIVMapper reads the parameters ----
  for (const std::string& f : a.param_files)
    if (!loadParamOverlay(f)) return 2;
  if (!a.nav_rinex.empty()) ros::param::set("/gnss/nav_rinex", a.nav_rinex);

  // ---- estimator (constructor reads the LIVO parameters; replay mode → no subscriptions, no timer) ----
  // Heap-allocated and deliberately never destroyed: ~LIVMapper crashes in free() on the -march=native build
  // (Eigen 64-byte aligned operator new/delete vs. the system PCL's malloc for the same pcl::PointCloud, upstream
  // issue registered in INTERFACE_CHANGES). Every output is flushed and closed before main returns, so skipping
  // the teardown changes no file; it makes the exit status meaningful (0 = complete, review m2) instead of
  // SIGSEGV on every complete run. The OS reclaims the memory.
  LIVMapper* mapper_p = new LIVMapper(nh);
  LIVMapper& mapper = *mapper_p;
  mapper.replay_mode_ = true;
  image_transport::ImageTransport it(nh);
  mapper.initializeSubscribersAndPublishers(nh, it);

  // ---- outputs ----
  // P3-B: one writer for the run, owned by the GNSS bridge: the driver opens it and writes the host files as in ref-R;
  // with gnss.enable the bridge adds the N-frame trajectories and the core's diagnostics (gnss_epochs.csv, events.log,
  // summary.json) to the same files. With gnss.enable = false the bridge never writes, so the files equal ref-R's.
  livo_gnss_ros::OutputWriter& out = *mapper.gnss_->outputs();
  livo_gnss_ros::OutputOptions opt;
  opt.dir = output_dir;
  opt.traj_w = out_traj_w;
  bool out_traj_n = true;
  nh.param<bool>("output/traj_n", out_traj_n, true);
  opt.traj_n = out_traj_n && mapper.gnss_->enabled();   // the N frame exists only with GNSS (none in ref-R)
  opt.sat_csv = mapper.gnss_->config().output.sat_csv;
  opt.state_trace = out_state_trace;
  opt.cov_csv = cov_csv;
  opt.timing = out_timing;
  std::string why;
  if (!out.open(opt, &why)) {
    std::fprintf(stderr, "fastlivo_replay: %s\n", why.c_str());
    return 3;
  }
  if (output_dir.empty()) std::fprintf(stderr, "fastlivo_replay: output/dir empty — no files will be written\n");

  // ---- bag ----
  rosbag::Bag bag;
  try {
    bag.open(bag_path, rosbag::bagmode::Read);
  } catch (const rosbag::BagException& e) {
    std::fprintf(stderr, "fastlivo_replay: cannot open bag %s: %s\n", bag_path.c_str(), e.what());
    return 3;
  }
  rosbag::View whole(bag);
  const ros::Time bag_begin = whole.getBeginTime(), bag_end = whole.getEndTime();
  const std::string lid_topic = mapper.lid_topic, imu_topic = mapper.imu_topic, img_topic = mapper.img_topic;
  const std::string img_topic_compressed = img_topic + "/compressed";
  const bool livox = (mapper.p_pre->lidar_type == AVIA);

  // Seed k → the k-th LiDAR message (record order) at or after bag_begin + start_offset_s opens the view.
  const ros::Time coarse_begin = bag_begin + ros::Duration(start_offset_s);
  ros::Time view_begin;
  uint64_t n_lidar_seen = 0;   // LiDAR messages scanned while locating the seed frame
  {
    rosbag::View lidar_view(bag, rosbag::TopicQuery(lid_topic), coarse_begin, ros::TIME_MAX);
    bool found = false;
    for (const rosbag::MessageInstance& m : lidar_view) {
      if (static_cast<int>(n_lidar_seen) == seed) {
        view_begin = m.getTime();
        found = true;
      }
      n_lidar_seen++;
      if (found) break;
    }
    if (!found) {
      std::fprintf(stderr, "fastlivo_replay: seed %d needs %d LiDAR messages on %s after offset %.3f s, bag has %llu\n", seed, seed + 1,
                   lid_topic.c_str(), start_offset_s, static_cast<unsigned long long>(n_lidar_seen));
      return 3;
    }
  }
  const double offset_eff = (view_begin - bag_begin).toSec();
  const ros::Time view_end = (duration_s > 0.0) ? view_begin + ros::Duration(duration_s) : ros::TIME_MAX;
  rosbag::View view(bag, view_begin, view_end);

  std::fprintf(stderr,
               "fastlivo_replay: bag=%s begin=%.6f end=%.6f view_begin=%.6f (= %d-th LiDAR frame after start_offset_s=%.3f; "
               "effective offset %.6f s) duration=%.1f\n"
               "fastlivo_replay: lidar=%s (%s) imu=%s img=%s%s output=%s\n",
               bag_path.c_str(), bag_begin.toSec(), bag_end.toSec(), view_begin.toSec(), seed, start_offset_s, offset_eff, duration_s,
               lid_topic.c_str(), livox ? "livox CustomMsg" : "PointCloud2", imu_topic.c_str(), img_topic.c_str(),
               mapper.img_compressed_en ? "/compressed" : "", output_dir.c_str());

  Stats st;
  const bool step_end_is_vio = (mapper.slam_mode_ == LIVO);
  // P3-B: GNSS state of the run (gnss.enable as configured; failures surface as a non-zero exit status).
  const bool gnss_enable = mapper.gnss_->config().enable;
  bool gnss_aug_failed = false;
  const livo_gnss::Config::Topics& gt = mapper.gnss_->config().topics;
  auto isGnssTopic = [&gt](const std::string& t) {
    return !t.empty() && (t == gt.meas || t == gt.ephem || t == gt.glo_ephem || t == gt.iono || t == gt.pvt || t == gt.tp);
  };

  // Everything written after one successful processOnce() (A13).
  auto afterCycle = [&](double elapsed_ms) {
    const int flag = static_cast<int>(mapper.LidarMeasures.lio_vio_flg);
    const double t_k = mapper.LidarMeasures.last_lio_update_time;
    const int cycle = mapper.cycle_idx_ - 1;   // 0-based index of this cycle
    st.cycles++;
    if (flag == LIO) st.cycles_lio++;
    else if (flag == VIO) st.cycles_vio++;
    else st.cycles_lo++;
    const livo_gnss::CoreState post = toCoreLocal(mapper._state);
    // P3-B: x_G and diag(Q_G) follow the host columns when the GNSS core runs (DESIGN §13); empty otherwise.
    Eigen::VectorXd x_G, diag_Q_G;
    if (livo_gnss::GnssFusion* f = gnss_aug_failed ? nullptr : mapper.gnss_->fusion()) {
      try {
        f->augmentedState(&x_G, &diag_Q_G, nullptr);
      } catch (const std::exception& e) {
        std::fprintf(stderr, "fastlivo_replay: GNSS augmentedState failed: %s\n", e.what());
        gnss_aug_failed = true;
        x_G.resize(0);
        diag_Q_G.resize(0);
      }
    }
    out.writeStateTrace(cycle, flag, t_k, post, x_G, diag_Q_G);
    if (cov_csv) {
      if (flag == VIO) {
        out.onCov(covRow(t_k, "postVIO", -1, mapper._state, mapper.state_propagat));
      } else {
        // effct_feat_num_ is an upstream member without an initialiser (voxel_map.h:214); it is first written by
        // StateEstimation(), which runs once lidar_map_inited is true. Before that (IMU initialisation) the value
        // is unknown: -1 (valgrind: uninitialised read otherwise). A LIO cycle whose undistorted scan is empty
        // returns before StateEstimation ("[ LIO ]: No point!!!", LIVMapper.cpp handleLIO) and would leave the
        // previous cycle's count in place: also -1 (review m9).
        const bool lio_updated = mapper.lidar_map_inited && mapper.feats_undistort && !mapper.feats_undistort->empty();
        const int effct = lio_updated ? mapper.voxelmap_manager->effct_feat_num_ : -1;
        out.onCov(covRow(t_k, "prior", effct, mapper.state_propagat, mapper.state_propagat));
        out.onCov(covRow(t_k, "postLIO", effct, mapper._state, mapper.state_propagat));
      }
    }
    const bool step_end = step_end_is_vio ? (flag == VIO) : true;
    if (step_end) out.writeTrajW(t_k, post.R, post.p);
    out.writeTiming(cycle, t_k, flag == VIO ? 0.0 : elapsed_ms, flag == VIO ? elapsed_ms : 0.0, 0.0);
  };

  auto drain = [&]() {
    for (;;) {
      const auto t_a = std::chrono::steady_clock::now();
      const bool ok = mapper.processOnce();
      const auto t_b = std::chrono::steady_clock::now();
      if (!ok) break;
      afterCycle(std::chrono::duration<double, std::milli>(t_b - t_a).count());
    }
  };

  for (const rosbag::MessageInstance& m : view) {
    if (!ros::ok()) break;
    const std::string& topic = m.getTopic();
    const std::string& type = m.getDataType();
    st.n_msgs++;
    if (st.n_msgs == 1) st.t_first_msg = m.getTime().toSec();
    st.t_last_msg = m.getTime().toSec();

    if (topic == lid_topic) {
      if (livox) {
        auto p = m.instantiate<livox_ros_driver::CustomMsg>();
        if (p) { mapper.livox_pcl_cbk(p); st.n_lidar++; } else st.n_type_mismatch++;
      } else {
        auto p = m.instantiate<sensor_msgs::PointCloud2>();
        if (p) { mapper.standard_pcl_cbk(p); st.n_lidar++; } else st.n_type_mismatch++;
      }
    } else if (topic == imu_topic) {
      auto p = m.instantiate<sensor_msgs::Imu>();
      if (p) { mapper.imu_cbk(p); st.n_imu++; } else st.n_type_mismatch++;
    } else if (mapper.img_compressed_en && topic == img_topic_compressed) {
      // Exactly one image path, as the live node subscribes exactly one (common/img_compressed; review m6).
      auto p = m.instantiate<sensor_msgs::CompressedImage>();
      if (p) { mapper.img_compressed_cbk(p); st.n_img++; } else st.n_type_mismatch++;
    } else if (!mapper.img_compressed_en && topic == img_topic) {
      auto p = m.instantiate<sensor_msgs::Image>();
      if (p) { mapper.img_cbk(p); st.n_img_raw++; } else st.n_type_mismatch++;
    } else {
      // P3-B: the bridge takes the configured GNSS topics (raw bytes + message_definition) and ignores the rest.
      if (gnss_enable && !a.no_gnss_messages && isGnssTopic(topic)) {
        mapper.gnss_->handleReplayMessage(m);
        st.n_gnss_routed++;
      }
      st.other[topic + " [" + type + "]"]++;   // every non-LIVO topic is still counted per topic
      continue;                                 // a GNSS message alone never completes an estimator cycle
    }
    drain();
  }
  drain();
  mapper.finish();
  out.flush();

  // ---- replay statistics: one member list (key, JSON value text), two outputs ----
  // replay_stats.json = {"complete": true, <all members>}, written last (completion criterion). run_manifest.json
  // merges the same members through ManifestInfo::extra_json (AMENDMENTS A40) except the two whose key is already a
  // manifest field: "seed" (same value) and "start_offset_s" (the manifest field holds the *effective* offset; the
  // configured trim is merged as "start_offset_trim_s"). Doubles as %.9f, the historical replay_stats.json format.
  const std::vector<BuildFile> bfiles = buildFiles(argv[0]);
  const std::string build_sha = buildSha256(bfiles, nullptr);
  std::vector<std::pair<std::string, std::string>> stats;
  {
    auto f9 = [](double v) {
      char b[64];
      std::snprintf(b, sizeof b, "%.9f", v);
      return std::string(b);
    };
    auto u = [](uint64_t v) { return std::to_string(v); };
    auto js = [](const std::string& v) {
      std::string o;
      jsonEscape(o, v);
      return o;
    };
    stats.emplace_back("bag_begin", f9(bag_begin.toSec()));
    stats.emplace_back("bag_end", f9(bag_end.toSec()));
    stats.emplace_back("view_begin", f9(view_begin.toSec()));
    stats.emplace_back("view_end", f9(duration_s > 0.0 ? view_end.toSec() : bag_end.toSec()));
    stats.emplace_back("start_offset_s", f9(start_offset_s));
    stats.emplace_back("seed", std::to_string(seed));
    stats.emplace_back("offset_effective_s", f9(offset_eff));
    stats.emplace_back("seed_semantics", js("view begins at the record time of the seed-th LiDAR message at or after bag_begin + start_offset_s"));
    stats.emplace_back("first_lidar_record_time", f9(view_begin.toSec()));
    stats.emplace_back("duration_s", f9(duration_s));
    std::string ex = "[";
    for (size_t i = 0; i < sizeof(kRunIdentityKeys) / sizeof(kRunIdentityKeys[0]); ++i) ex += (i ? ", " : "") + js(kRunIdentityKeys[i]);
    stats.emplace_back("config_sha256_excludes", ex + "]");
    stats.emplace_back("build_sha256", js(build_sha));
    std::string bf = "{";
    for (size_t i = 0; i < bfiles.size(); ++i) bf += (i ? ", " : "") + js(bfiles[i].path) + ": " + js(bfiles[i].sha256);
    stats.emplace_back("build_files", bf + "}");
    stats.emplace_back("t_first_msg", f9(st.t_first_msg));
    stats.emplace_back("t_last_msg", f9(st.t_last_msg));
    stats.emplace_back("n_msgs", u(st.n_msgs));
    stats.emplace_back("n_lidar", u(st.n_lidar));
    stats.emplace_back("n_imu", u(st.n_imu));
    stats.emplace_back("n_img_compressed", u(st.n_img));
    stats.emplace_back("n_img_raw", u(st.n_img_raw));
    stats.emplace_back("n_type_mismatch", u(st.n_type_mismatch));
    stats.emplace_back("cycles", u(st.cycles));
    stats.emplace_back("cycles_lio", u(st.cycles_lio));
    stats.emplace_back("cycles_vio", u(st.cycles_vio));
    stats.emplace_back("cycles_lo", u(st.cycles_lo));
    std::string ot = "{";
    bool first = true;
    for (const auto& kv : st.other) {
      ot += (first ? "" : ", ") + js(kv.first) + ": " + u(kv.second);
      first = false;
    }
    stats.emplace_back("other_topics", ot + "}");
    // P3-B: GNSS inputs and outcome of the run.
    auto jsa = [&js](const std::vector<std::string>& v) {
      std::string o = "[";
      for (size_t i = 0; i < v.size(); ++i) o += (i ? ", " : "") + js(v[i]);
      return o + "]";
    };
    stats.emplace_back("gnss_enable", gnss_enable ? "true" : "false");
    stats.emplace_back("gnss_update_enable", mapper.gnss_->config().update_enable ? "true" : "false");
    stats.emplace_back("gnss_param_files", jsa(a.param_files));
    stats.emplace_back("gnss_nav_rinex", jsa(mapper.gnss_->config().nav_rinex));
    stats.emplace_back("gnss_no_messages", a.no_gnss_messages ? "true" : "false");
    stats.emplace_back("gnss_msgs_routed", u(st.n_gnss_routed));
    stats.emplace_back("gnss_failed", (gnss_enable && (!mapper.gnss_->enabled() || gnss_aug_failed)) ? "true" : "false");
  }

  // ---- manifest ----
  const std::string dump = paramDump(false);
  const std::string dump_hashed = paramDump(true);
  livo_gnss_ros::ManifestInfo mi;
  mi.git_describe = !a.git_describe.empty() ? a.git_describe : envOr("FL2G_GIT_DESCRIBE", "unknown");
  mi.git_dirty = !a.git_dirty.empty() ? a.git_dirty : envOr("FL2G_GIT_DIRTY", "unknown");
  mi.profile = profileName();
  // build_sha256 rides in compile_defs (ManifestInfo is frozen; the per-file listing is in replay_stats.json).
  mi.compile_defs = compileDefs() + " build_sha256=" + build_sha;
  mi.eigen_version = std::to_string(EIGEN_WORLD_VERSION) + "." + std::to_string(EIGEN_MAJOR_VERSION) + "." + std::to_string(EIGEN_MINOR_VERSION);
  mi.compiler = __VERSION__;
  mi.image_digest = !a.image_digest.empty() ? a.image_digest : envOr("FL2G_IMAGE_DIGEST", "");
  mi.config_sha256 = sha256OfString(dump_hashed);   // run-identity keys excluded, see kRunIdentityKeys
  mi.config_dump = dump;
  mi.bag_path = bag_path;
  mi.bag_sha256 = !a.bag_sha256.empty() ? a.bag_sha256 : sha256OfFile(bag_path);
  mi.arm = a.arm;
  mi.host = hostName();
  mi.seed = seed;
  mi.start_offset_s = offset_eff;
  mi.t_start_wall = t_start_wall;
  mi.t_end_wall = wallNow();
  {
    std::string extra = "{\n";
    bool first = true;
    for (const auto& kv : stats) {
      if (kv.first == "seed") continue;   // == mi.seed
      const std::string key = (kv.first == "start_offset_s") ? "start_offset_trim_s" : kv.first;
      extra += std::string(first ? "" : ",\n") + "  \"" + key + "\": " + kv.second;
      first = false;
    }
    mi.extra_json = extra + "\n}";
  }
  out.writeManifest(mi);

  // ---- replay statistics (message routing, cycle counts, build identity); written last ----
  if (!output_dir.empty()) {
    std::ofstream js(output_dir + "/replay_stats.json");
    js << "{\n  \"complete\": true";
    for (const auto& kv : stats) js << ",\n  \"" << kv.first << "\": " << kv.second;
    js << "\n}\n";
  }
  out.close();
  std::fprintf(stderr, "fastlivo_replay: done — %llu messages, %llu cycles (%llu LIO, %llu VIO), %llu type mismatches\n",
               static_cast<unsigned long long>(st.n_msgs), static_cast<unsigned long long>(st.cycles),
               static_cast<unsigned long long>(st.cycles_lio), static_cast<unsigned long long>(st.cycles_vio),
               static_cast<unsigned long long>(st.n_type_mismatch));
  // P3-B: a GNSS run whose core failed continued as the host alone (fail-safe); its outputs are not a GNSS result.
  if (gnss_enable && (!mapper.gnss_->enabled() || gnss_aug_failed)) {
    std::fprintf(stderr, "fastlivo_replay: GNSS failed during the run (see the [gnss] errors and events.log): exit status 4\n");
    return 4;
  }
  return 0;
}
