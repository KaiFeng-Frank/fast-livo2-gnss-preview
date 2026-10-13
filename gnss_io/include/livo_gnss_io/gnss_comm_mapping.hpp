// SPDX-License-Identifier: BSD-3-Clause
// gnss_comm_mapping — from decoded field trees to livo_gnss types by *field name* (DESIGN §12.2–§12.3):
//   * expectation tables: flattened (type,name) layouts of the six gnss_comm message types as
//     functional facts read from the dataset's own message_definition; a mismatch rejects the topic and
//     reports the diff (no partial acceptance);
//   * numbering: gnss_comm sat → (sys, prn) → RTKLIB satno (GPS 1-32, GLO 33-59, GAL 60-97, BDS 98-160);
//   * observations: freqs[] → SignalCode (table), psr/cp/dopp/CN0/LLI/std/status per slot; primary
//     signal first; BDS 1575.42 rejected and counted; GLO channel cross-checked later by EphemStore;
//   * ephemerides: toe/toc/ttr GPST(week,tow) → GTime; A = semi-major axis; tgd0/1 → tgd[0]/[1];
//     health → svh; ura kept raw; GLONASS freqo→frq, tau_n→taun, gamma→gamn, delta_tau_n→dtaun;
//   * iono_params (8 values) → Klobuchar; receiver_pvt → ReceiverPvt; time_pulse_info → counted only (A19:
//     its checks live in tools/m3dgr_audit.py; nothing of it enters the core).
// This header contains NO gnss_comm text; the tables are built from our own field-name constants.
// Owner: WP-e. Tests: test_u02_wire.cpp (layout rejection), test_t01_t05_io.cpp, test_u03/u06 (core).
#pragma once

#include <array>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "livo_gnss/gnss_fusion.hpp"
#include "livo_gnss/signal_table.hpp"
#include "livo_gnss/types.hpp"
#include "livo_gnss_io/ros1_decoder.hpp"
#include "livo_gnss_io/ros1_msgdef.hpp"

namespace livo_gnss_io {

using Layout = std::vector<std::pair<std::string, std::string>>;

// Expected flattened layouts (functional facts of the wire format). Empty for TimePulse (diagnostics
// only, never rejected).
const Layout& expectedLayout(livo_gnss::Source src);
// Compares `schema.flatten()` with the expectation; on mismatch fills a human-readable diff.
bool validateLayout(livo_gnss::Source src, const MsgSchema& schema, std::string* diff);

// gnss_comm satellite numbering ↔ livo_gnss::SatId (through rtk::satNo inside the core: the mapping
// here only splits the ranges; `rtk_sat` is filled by the core via a callback to keep gnss_io free
// of RTKLIB). Returns Sys::Unknown for ids outside the ranges.
livo_gnss::Sys gnssCommSatToSys(uint32_t sat, int* prn);

struct MappingStats {
  uint64_t epochs = 0, sats = 0, signals = 0;
  uint64_t rejected_bds_b1c = 0, rejected_unknown_freq = 0, rejected_gal_prn = 0, rejected_sat_range = 0;
  uint64_t ephem = 0, glo_ephem = 0, iono = 0, pvt = 0, time_pulse = 0;
  uint64_t decode_errors = 0;
  // Second and later signals with the same SignalCode on one satellite (first listed wins, same rule as the
  // RINEX/UBX paths; gnss_io/src/signal_order.hpp). (Appended P2b, INTERFACE_CHANGES WP-e #7.)
  uint64_t rejected_repeated_code = 0;
};

// Satellite-id resolver so the mapping can fill SatId::rtk_sat without linking RTKLIB.
using SatNoFn = int (*)(livo_gnss::Sys, int prn);

// Field-tree → core types. Each returns false (and `why`) when a required field is missing.
bool toObsEpoch(const FieldNode& msg, double t_record, SatNoFn satno, livo_gnss::ObsEpoch* out, MappingStats* stats, std::string* why);
bool toKeplerEph(const FieldNode& msg, double t_record, SatNoFn satno, livo_gnss::KeplerEph* out, MappingStats* stats, std::string* why);
bool toGloEph(const FieldNode& msg, double t_record, SatNoFn satno, livo_gnss::GloEph* out, MappingStats* stats, std::string* why);
bool toKlobuchar(const FieldNode& msg, double t_record, livo_gnss::Klobuchar* out, std::string* why);
bool toReceiverPvt(const FieldNode& msg, double t_record, livo_gnss::ReceiverPvt* out, std::string* why);
// freqs[] → SignalCode: use livo_gnss::signalFromFrequency() from livo_gnss/signal_table.hpp (the single
// copy of the §12.3 table, M6); toObsEpoch() counts its SignalVerdict rejections in MappingStats.

// RawDecoder implementation for the six gnss_comm ROS1 channels. Both byte sources (rosbag
// MessageInstance::write() and topic_tools::ShapeShifter::write()) feed the same object.
class GnssCommRos1Decoder final : public livo_gnss::RawDecoder {
 public:
  explicit GnssCommRos1Decoder(SatNoFn satno);
  ~GnssCommRos1Decoder() override;
  bool registerMessageDefinition(livo_gnss::Source src, const std::string& datatype, const std::string& md5,
                                 const std::string& msg_def, std::string* why) override;
  bool decode(livo_gnss::Source src, const uint8_t* data, size_t size, double t_record, livo_gnss::MeasurementSink& sink,
              std::string* why) override;
  bool registered(livo_gnss::Source src) const;
  const MappingStats& stats() const { return stats_; }

 private:
  SatNoFn satno_;
  std::map<livo_gnss::Source, std::unique_ptr<Ros1Decoder>> decoders_;
  MappingStats stats_;
};

}  // namespace livo_gnss_io
