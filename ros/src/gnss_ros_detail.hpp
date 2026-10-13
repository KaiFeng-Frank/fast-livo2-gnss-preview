// SPDX-License-Identifier: GPL-2.0-only
// Internals of gnss_ros_bridge.cpp shared with its tests (ros/test/test_p3b_glue.cpp). Not a public header: the
// public glue interface is ros/include/livo_gnss_ros/gnss_ros_bridge.hpp (frozen); these are the pieces the tests
// must reach directly — the parameter table, the two byte sources of I14, the A35-C8 sat2freq cross-check wrapped
// around the gnss_io decoder, and the WGS-84 helpers of the N-frame outputs. Owner: P3-B.
#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <XmlRpcValue.h>
#include <rosbag/message_instance.h>
#include <topic_tools/shape_shifter.h>

#include "livo_gnss/config.hpp"
#include "livo_gnss/gnss_fusion.hpp"
#include "livo_gnss/livgo.hpp"
#include "livo_gnss/types.hpp"

namespace livo_gnss_ros {
namespace detail {

// ---- parameters (DESIGN §14) ----
// Fills `cfg` (which holds the defaults) from the `gnss:` parameter tree. Keys are the config.hpp field paths with
// '/' separators ("time/toff_monitor/window_s"), the same set canonicalDump() lists with '.'. Per-system maps
// (masks/eph_age_s, noise/psr/sys_factor) merge key by key into the defaults. An unknown key, a wrong type, an
// unknown enum name or a malformed list appends a message naming the key to `errors` (nothing is ignored silently).
// Returns errors->empty().
bool configFromParamTree(XmlRpc::XmlRpcValue& gnss, livo_gnss::Config* cfg, std::vector<std::string>* errors);
// Every key path of the loader table, in config.hpp order.
std::vector<std::string> configKeyPaths();
// Differences between the loader table and canonicalDump(Config()) (a field of config.hpp without a loader entry, or
// an entry without a field); empty when both describe the same set of keys.
std::vector<std::string> loaderCoverageProblems();
// gnss/livgo/* is not a Config field (that would change sizeof(Config) and the T-arm object code). The loader
// still accepts the keys and parks them here. Reset to defaults at the start of every configFromParamTree.
livo_gnss::LivgoConfig& loadedLivgoConfig();

// ---- bytes (DESIGN §12.2, TEST_PLAN I14): the serialised message body, without the ROS length prefix ----
bool messageBytes(const rosbag::MessageInstance& m, std::vector<uint8_t>* out);   // replay: MessageInstance::write()
bool messageBytes(const topic_tools::ShapeShifter& m, std::vector<uint8_t>* out); // live: ShapeShifter::write()

// ---- A35-C8: one-time sat2freq cross-check (ingest self-check 1, glue part) ----
// Each (satellite, signal code) is checked the first time it is decoded: the carrier frequency the message reports
// must equal the RTKLIB sat2freq() value, i.e. the nominal frequency of that code (signal_table.hpp, the same
// constants as rtklib.h FREQ*) with the GLONASS FDMA channel taken from an ephemeris (nav.glo_fcn, bag glo_ephem or
// RINEX NAV). GLONASS pairs seen before any ephemeris of their satellite stay pending until one arrives. Diagnostic
// only: rows are never removed here (the row builders do not repeat the check either, A35-C8).
class Sat2FreqCheck {
 public:
  struct Report {
    uint64_t pairs = 0, consistent = 0, mismatched = 0, pending = 0;
    double max_abs_diff_hz = 0.0;
    std::vector<std::string> mismatches;   // "R07 L1C: message 1602.5625 MHz, sat2freq 1602.0000 MHz"
  };
  void setGloChannel(int rtk_sat, int k);   // from GloEph::frq (bag or RINEX); resolves pending pairs
  void onObservation(const livo_gnss::ObsEpoch& e);
  Report report() const;
  static constexpr double kTolHz = 1.0;

 private:
  struct Pending {
    livo_gnss::SatId sat;
    livo_gnss::SignalCode code;
    double freq_hz;
  };
  void check(const livo_gnss::SatId& sat, livo_gnss::SignalCode code, double freq_hz, int k);
  std::map<int, int> glo_k_;                              // rtk_sat → channel k
  std::set<std::pair<int, int>> seen_;                    // (rtk_sat, code)
  std::map<std::pair<int, int>, Pending> pending_;
  Report rep_;
};

// Forwards everything to the next sink; observations and GLONASS ephemerides also feed the Sat2FreqCheck.
class CheckingSink final : public livo_gnss::MeasurementSink {
 public:
  CheckingSink(livo_gnss::MeasurementSink& next, Sat2FreqCheck* check) : next_(next), check_(check) {}
  void pushObservation(livo_gnss::ObsEpoch&& epoch) override;
  void pushEphemeris(const livo_gnss::KeplerEph& eph) override { next_.pushEphemeris(eph); }
  void pushGloEphemeris(const livo_gnss::GloEph& eph) override;
  void pushIono(const livo_gnss::Klobuchar& iono) override { next_.pushIono(iono); }
  void pushPvt(const livo_gnss::ReceiverPvt& pvt) override { next_.pushPvt(pvt); }

 private:
  livo_gnss::MeasurementSink& next_;
  Sat2FreqCheck* check_;
};

// The RawDecoder the glue registers with GnssFusion: the gnss_io decoder (gnss_comm ROS1 or UBX) whose output passes
// a CheckingSink on its way into the core's sink.
class CheckedDecoder final : public livo_gnss::RawDecoder {
 public:
  CheckedDecoder(livo_gnss::RawDecoder* inner, Sat2FreqCheck* check) : inner_(inner), check_(check) {}
  bool registerMessageDefinition(livo_gnss::Source src, const std::string& datatype, const std::string& md5, const std::string& msg_def,
                                 std::string* why) override;
  bool decode(livo_gnss::Source src, const uint8_t* data, size_t size, double t_record, livo_gnss::MeasurementSink& sink,
              std::string* why) override;

 private:
  livo_gnss::RawDecoder* inner_;
  Sat2FreqCheck* check_;
};

// ---- WGS-84 (N-frame outputs: traj_N_geo.csv, NavSatFix) ----
Eigen::Vector3d geodeticToEcef(const livo_gnss::Geodetic& g);
livo_gnss::Geodetic ecefToGeodetic(const Eigen::Vector3d& r);    // iterative, |Δz| < 1e-4 m
Eigen::Matrix3d enuToEcefRotation(double lat_rad, double lon_rad); // columns = east, north, up at (lat, lon)

}  // namespace detail
}  // namespace livo_gnss_ros
