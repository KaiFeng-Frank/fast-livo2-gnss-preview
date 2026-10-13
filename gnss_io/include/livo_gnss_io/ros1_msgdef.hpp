// SPDX-License-Identifier: BSD-3-Clause
// ros1_msgdef — parser of a ROS1 connection-header `message_definition` into a schema (DESIGN §12.2).
// Handles: the main type followed by `MSG: <pkg/Type>` dependency sections separated by '=' lines,
// built-in types (bool,int8..int64,uint8..uint64,float32,float64,string,time,duration), Header,
// fixed-length arrays `T[N]`, variable-length arrays `T[]`, constants (`T NAME=value`, ignored for
// layout), comments. The flattened (type,name) sequence is what gnss_comm_mapping compares with its
// expectation table. Nothing here is gnss_comm specific; the schema is data read at run time.
// Owner: WP-e. Tests: test_u02_wire.cpp.
#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace livo_gnss_io {

// (`NotBuiltin` / `Boolean`, not `None` / `Bool`: X11 macros, m19.)
enum class BuiltinType : uint8_t {
  NotBuiltin = 0, Boolean, Int8, UInt8, Int16, UInt16, Int32, UInt32, Int64, UInt64, Float32, Float64, String, Time, Duration
};
const char* builtinName(BuiltinType t);
BuiltinType builtinFromName(const std::string& name);   // NotBuiltin for non-builtin
size_t builtinSize(BuiltinType t);                       // fixed wire size (String/Time/Duration: 0/8/8; string is variable)

struct FieldSpec {
  std::string type;          // as written ("float64", "GnssTimeMsg", "std_msgs/Header")
  std::string name;
  BuiltinType builtin = BuiltinType::NotBuiltin;
  bool is_array = false;
  int fixed_len = -1;        // -1 = variable-length array (uint32 length prefix); ≥0 fixed
  std::string resolved_type; // fully qualified nested type ("gnss_comm/GnssTimeMsg")
};

struct MsgSchema {
  std::string datatype;                                  // "gnss_comm/GnssMeasMsg"
  std::vector<FieldSpec> fields;
  std::map<std::string, std::shared_ptr<MsgSchema>> nested;   // dependency schemas by qualified name
  // Flattened (type, name) sequence with nested fields expanded as "outer.inner" and arrays marked
  // by "[]" / "[N]" on the type; this is the layout compared against the expectation table.
  std::vector<std::pair<std::string, std::string>> flatten() const;
  // Constants of this type, one "T NAME=value" line each in definition order, spelled exactly as genmsg
  // writes them into the md5 text; filled by MsgDefParser::parse, and md5() puts them before the field
  // lines. Not part of the layout (flatten() ignores them). (Appended P2b, INTERFACE_CHANGES WP-e #1.)
  std::vector<std::string> constants;
};

class MsgDefParser {
 public:
  // Parses `msg_def` (the connection header text) for `datatype`. Returns false with `why` on syntax
  // errors or unresolved nested types. Never throws on malformed input.
  static bool parse(const std::string& datatype, const std::string& msg_def, MsgSchema* out, std::string* why);
  // Splits the definition into (qualified type name → body) sections.
  static std::vector<std::pair<std::string, std::string>> splitSections(const std::string& datatype, const std::string& msg_def);
  // Parses one field line ("float64[] freqs  # comment"); returns false for blank/comment/constant lines.
  static bool parseFieldLine(const std::string& line, const std::string& package, FieldSpec* out, std::string* why);
  // ROS1 md5 of a schema (for diagnostics only; the decoder does not require md5 agreement).
  // Exact only for types whose closure has no constants, or whose constants parse() recorded in
  // MsgSchema::constants (WP-e #1). Otherwise a type with constants gets a wrong digest, not an error:
  // do not accept a definition on the strength of md5() until that holds (WP-e #2).
  static std::string md5(const MsgSchema& schema);
};

}  // namespace livo_gnss_io
