// SPDX-License-Identifier: BSD-3-Clause
// UbxStream — in-process UBX decoding via RTKLIB input_ubx (DESIGN §12.4): RXM-RAWX → ObsEpoch,
// RXM-SFRBX → KeplerEph/GloEph/Klobuchar, NAV-PVT → ReceiverPvt. Bytes come from the BSD-licensed
// `GnssRawStream{header, format, uint8[] data}` message (real F9P) — no GPL driver involved.
// rtklib.h is included by ubx_stream.cpp only (pimpl raw_t). Untested on hardware (DESIGN §16-13).
// Owner: WP-e. Tests: test_t01_t05_io.cpp (T05, synthetic RAWX frames).
#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include "livo_gnss/gnss_fusion.hpp"
#include "livo_gnss/types.hpp"

namespace livo_gnss_io {

class UbxStream final : public livo_gnss::RawDecoder {
 public:
  UbxStream();
  ~UbxStream() override;
  // Not applicable to UBX (returns true, ignores the definition).
  bool registerMessageDefinition(livo_gnss::Source src, const std::string& datatype, const std::string& md5,
                                 const std::string& msg_def, std::string* why) override;
  // Feeds a chunk of bytes; every complete UBX frame produces zero or more pushes into `sink`.
  bool decode(livo_gnss::Source src, const uint8_t* data, size_t size, double t_record, livo_gnss::MeasurementSink& sink,
              std::string* why) override;
  void reset();
  uint64_t framesDecoded() const;
  uint64_t checksumErrors() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace livo_gnss_io
