// SPDX-License-Identifier: BSD-3-Clause
// Synthetic GNSS activation and output example; never a real-data accuracy claim.
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

#include "simulator.hpp"

namespace {
using livo_gnss::sim::RunResult;

template <typename Pick>
void writeTrajectory(const std::filesystem::path& path, const RunResult& r, Pick pick) {
  std::ofstream out(path);
  out << "# Synthetic antenna positions; seconds since scene start; identity quaternions\n";
  out << std::setprecision(17);
  for (const auto& sample : r.traj) {
    const Eigen::Vector3d p = pick(sample);
    out << sample.t << ' ' << p.x() << ' ' << p.y() << ' ' << p.z() << " 0 0 0 1\n";
  }
  if (!out) throw std::runtime_error("could not write " + path.string());
}
}  // namespace

int main(int argc, char** argv) {
  if (argc != 3) {
    std::cerr << "Usage: preview_demo <synthetic.nav> <output-directory>\n";
    return 2;
  }
  try {
    livo_gnss::sim::SimParams p;
    p.scenario = livo_gnss::sim::Scenario::OpenSky;
    p.seed = 0;
    p.nav_rinex = argv[1];
    p.site.lat_rad = 34.0 * std::acos(-1.0) / 180.0;
    p.site.lon_rad = 108.0 * std::acos(-1.0) / 180.0;
    p.site.h_m = 400.0;
    p.t0.sec = 1737446400;  // 2025-01-21 08:00:00 GPST, GTime's calendar scale
    p.t0.frac = 0.0;
    p.path_m = 150.0;
    const auto cfg = livo_gnss::sim::simConfig(p);
    const RunResult r = livo_gnss::sim::runScenario(p, cfg);
    std::cout << r.describe() << '\n' << "Activation: " << r.activate_detail << '\n';
    if (r.no_fixture || !r.error.empty() || !r.activated || r.traj.size() < 3) {
      std::cerr << "Synthetic demo did not activate and produce enough samples.\n";
      return 1;
    }
    const char* names[3] = {"psr", "dopp", "tdcp"};
    for (std::size_t i = 0; i < 3; ++i) {
      std::cout << names[i] << ": candidates " << r.rows[i].candidates << ", accepted " << r.rows[i].accepted << '\n';
      if (r.rows[i].accepted <= 0) throw std::runtime_error(std::string("no accepted ") + names[i] + " rows");
    }
    if (!std::isfinite(r.ate_u) || !std::isfinite(r.ate_t) || !std::isfinite(r.ate_o))
      throw std::runtime_error("non-finite synthetic ATE");
    const std::filesystem::path output(argv[2]);
    std::filesystem::create_directories(output);
    auto recorded_cfg = cfg;
    recorded_cfg.nav_rinex = {"synthetic.nav"};  // portable input identity; bytes are hashed by score_demo.py
    std::ofstream config_output(output / "config.txt");
    config_output << livo_gnss::canonicalDump(recorded_cfg);
    if (!config_output) throw std::runtime_error("could not write config.txt");
    writeTrajectory(output / "truth.tum", r, [](const auto& s) { return s.truth; });
    writeTrajectory(output / "host.tum", r, [](const auto& s) { return s.host_W; });
    writeTrajectory(output / "global.tum", r, [](const auto& s) { return s.est_N; });
    writeTrajectory(output / "odometry.tum", r, [](const auto& s) { return s.est_odo; });
    std::ofstream out(output / "summary.json");
    out << std::setprecision(17)
        << "{\n  \"schema\": \"fastlivo.gnss.preview.synthetic.v1\",\n"
        << "  \"synthetic_only\": true,\n  \"scenario\": \"open_sky\",\n  \"seed\": 0,\n"
        << "  \"path_m\": " << p.path_m << ",\n  \"activated\": true,\n"
        << "  \"config\": \"config.txt\",\n  \"navigation_input\": \"synthetic.nav\",\n"
        << "  \"activation_s\": " << r.t_activate << ",\n  \"settle_s\": " << p.settle_s << ",\n"
        << "  \"duration_s\": " << r.duration_s << ",\n  \"samples\": " << r.traj.size() << ",\n"
        << "  \"accepted_rows\": {\"psr\": " << r.rows[0].accepted << ", \"dopp\": " << r.rows[1].accepted
        << ", \"tdcp\": " << r.rows[2].accepted << "},\n"
        << "  \"ate_rmse_m\": {\"host\": " << r.ate_u << ", \"global\": " << r.ate_t
        << ", \"odometry\": " << r.ate_o << "}\n}\n";
    if (!out) throw std::runtime_error("could not write summary.json");
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
  return 0;
}
