// SPDX-License-Identifier: BSD-3-Clause
// ros1_decoder — ROS1 wire-format decoder driven by a MsgSchema (DESIGN §12.2): little-endian scalars,
// variable-length arrays with a uint32 length prefix, strings likewise, nested messages inlined,
// time/duration = 2 × uint32 (sec, nsec). Output is a field tree; nothing is interpreted here.
// Malformed buffers (truncation, absurd lengths) return false, never throw, never read out of bounds (U2).
// Owner: WP-e. Tests: test_u02_wire.cpp.
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "livo_gnss_io/ros1_msgdef.hpp"

namespace livo_gnss_io {

struct Value {
  BuiltinType type = BuiltinType::NotBuiltin;
  // Scalars (one of these is meaningful per `type`):
  int64_t i = 0;
  uint64_t u = 0;
  double f = 0.0;
  bool b = false;
  std::string s;
  uint32_t sec = 0, nsec = 0;           // Time / Duration
  double toDouble() const;              // numeric view (time → sec + 1e-9·nsec)
};

struct FieldNode {
  std::string name;
  std::string type;                     // as in FieldSpec::type
  bool is_array = false;
  bool is_message = false;
  Value scalar;                         // when !is_array && !is_message
  std::vector<Value> array;             // when is_array && !is_message
  std::vector<FieldNode> children;      // when is_message (fields in order); for message arrays: one
                                        // child per element, each holding its own children
  const FieldNode* child(const std::string& name) const;   // nullptr if absent
};

class Ros1Decoder {
 public:
  explicit Ros1Decoder(const MsgSchema& schema);
  // Decodes `size` bytes of a serialised message (without the ROS length prefix) into `out`.
  // Returns false on any inconsistency; `consumed` reports the bytes read.
  bool decode(const uint8_t* data, size_t size, FieldNode* out, size_t* consumed, std::string* why) const;
  const MsgSchema& schema() const { return schema_; }

  // Little-endian readers with bounds checks (shared with the UBX path).
  struct Cursor {
    const uint8_t* p = nullptr;
    size_t n = 0, pos = 0;
    bool readU8(uint8_t* v);
    bool readU16(uint16_t* v);
    bool readU32(uint32_t* v);
    bool readU64(uint64_t* v);
    bool readI8(int8_t* v);
    bool readI16(int16_t* v);
    bool readI32(int32_t* v);
    bool readI64(int64_t* v);
    bool readF32(float* v);
    bool readF64(double* v);
    bool readString(std::string* v, size_t max_len = (1u << 20));
  };

 private:
  MsgSchema schema_;
};

}  // namespace livo_gnss_io
