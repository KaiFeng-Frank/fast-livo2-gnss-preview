// SPDX-License-Identifier: GPL-2.0-only
// manifest_check.cpp — WP-f gate tool for OutputWriter::writeManifest + ManifestInfo::extra_json (AMENDMENTS A40),
// linked against the built liblivo_gnss_ros.so (the ROS glue has no ctest; I14 is P3). For every case below it
// writes <root>/<case>/run_manifest.json (and the raw extra text as extra.json); `manifest_check.py --cases <root>`
// then decides each case: strict JSON with unique keys always; an empty/blank/{} extra gives the pre-P2b layout
// byte for byte; a well-formed extra is merged member by member, verbatim, after the frozen fields; anything
// malformed or colliding merges nothing and leaves exactly one "extra_json_error" member.
// Built and run by `gates_local.sh <tree> <wt> manifest-check` (container, flock, nice 19).
#include <cstdio>
#include <string>
#include <utility>
#include <vector>

#include "livo_gnss_ros/outputs.hpp"

int main(int argc, char** argv) {
  if (argc != 2) {
    std::fprintf(stderr, "usage: manifest_check <output_root>\n");
    return 2;
  }
  const std::string root = argv[1];
  const std::vector<std::pair<std::string, std::string>> cases = {
      {"empty", ""},
      {"blank", "  \n\t "},
      {"empty_object", "{ }"},
      {"merge",
       "{\n  \"alpha\": 1,\n  \"beta\": {\"x\": [1, 2.5e-3, \"a\\\"b\"], \"y\": null},\n  \"gamma\": \"caf\xc3\xa9 \\u00e9\\n\",\n"
       "  \"delta\": [true, false, -0.0, 1E+2]\n}"},
      {"nesting_64", "{\"a\": " + std::string(63, '[') + std::string(63, ']') + "}"},
      {"collide_manifest", "{\"seed\": 3}"},
      {"collide_self", "{\"a\": 1, \"a\": 2}"},
      {"trailing_text", "{\"a\": 1} x"},
      {"trailing_comma", "{\"a\": 1,}"},
      {"unterminated_string", "{\"a\": \"abc}"},
      {"not_an_object", "[1, 2]"},
      {"raw_newline_in_string", "{\"a\": \"x\ny\"}"},
      {"leading_zero", "{\"a\": 01}"},
      {"bad_escape", "{\"a\": \"\\q\"}"},
      {"nan_literal", "{\"a\": NaN}"},
      {"nesting_65", "{\"a\": " + std::string(64, '[') + std::string(64, ']') + "}"},
  };
  int failed = 0;
  for (const auto& c : cases) {
    livo_gnss_ros::OutputOptions o;
    o.dir = root + "/" + c.first;
    o.state_trace = o.traj_w = o.cov_csv = o.timing = false;
    livo_gnss_ros::OutputWriter w;
    std::string why;
    if (!w.open(o, &why)) {
      std::fprintf(stderr, "manifest_check: %s\n", why.c_str());
      ++failed;
      continue;
    }
    livo_gnss_ros::ManifestInfo m;
    m.git_describe = "g-1";
    m.git_dirty = "0";
    m.profile = "eval";
    m.compile_defs = "X=1 build_sha256=00";
    m.eigen_version = "3.3.7";
    m.compiler = "gcc";
    m.image_digest = "sha256:0";
    m.config_sha256 = "abc";
    m.config_dump = "{\"k\":\"v\\n\"}\ttab";
    m.bag_path = "/r/a.bag";
    m.bag_sha256 = "00";
    m.arm = "U";
    m.host = "h";
    m.seed = 7;
    m.start_offset_s = 0.25;
    m.t_start_wall = 1737451258.5;
    m.t_end_wall = 1737451300.125;
    m.extra_json = c.second;
    w.writeManifest(m);
    w.close();
    std::FILE* f = std::fopen((o.dir + "/extra.json").c_str(), "w");
    if (f) {
      std::fputs(c.second.c_str(), f);
      std::fclose(f);
    }
  }
  std::printf("manifest_check: %zu cases written under %s (%d failed to open)\n", cases.size(), root.c_str(), failed);
  return failed == 0 ? 0 : 1;
}
