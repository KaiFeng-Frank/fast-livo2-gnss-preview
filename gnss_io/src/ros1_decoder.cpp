// SPDX-License-Identifier: BSD-3-Clause
// ros1_decoder (DESIGN §12.2). Owner: WP-e. Tests: U2 (test_u02_wire.cpp), T3.
//
// ROS1 serialisation (roscpp_serialization / genpy): scalars little-endian at their natural size, bool as one
// byte, string = uint32 length + bytes (no terminator), time/duration = 2 × 32 bit (sec, nsec; duration
// signed), variable-length arrays = uint32 count + elements, fixed arrays = elements only, nested messages
// inlined. Nothing is interpreted here. Every read is bounds-checked; array counts are checked against the
// bytes left before anything is allocated, so garbage cannot trigger a huge allocation (U2).
//
// Output-size bound: each wire element becomes a Value (~80 B) or a FieldNode (~200 B), so a uint8[] of n bytes
// would cost ~80·n bytes of memory. A per-message budget of kMaxNodes tree nodes (array elements + fields)
// caps that at a few tens of MB whatever the input; a gnss_comm range_meas with 40 satellites needs ~2·10³.
#include "livo_gnss_io/ros1_decoder.hpp"

#include <cmath>
#include <cstring>
#include <limits>
#include <string>

namespace livo_gnss_io {

namespace {

constexpr int kMaxDepth = 32;
constexpr uint32_t kMaxEmptyElements = 1u << 16;   // bound for arrays of zero-size messages (no bytes to check)
constexpr size_t kMaxNodes = size_t{1} << 18;      // per-message node budget (see header comment); U2 tests it

using SchemaMap = std::map<std::string, std::shared_ptr<MsgSchema>>;

const MsgSchema* lookupNested(const SchemaMap& m, const std::string& name) {
  const auto it = m.find(name);
  return (it == m.end() || !it->second) ? nullptr : it->second.get();
}

// Minimum number of wire bytes one instance of `s` occupies (empty variable arrays / strings = 4 bytes).
// Saturates instead of overflowing; used only for the "count × min size ≤ bytes left" sanity check.
size_t minWireSize(const MsgSchema& s, const SchemaMap& lookup, int depth) {
  if (depth > kMaxDepth) return 0;
  size_t total = 0;
  auto add = [&](size_t v) { total = (total > std::numeric_limits<size_t>::max() - v) ? std::numeric_limits<size_t>::max() : total + v; };
  for (const FieldSpec& f : s.fields) {
    size_t elem = 0;
    if (f.builtin != BuiltinType::NotBuiltin) {
      elem = f.builtin == BuiltinType::String ? 4u : builtinSize(f.builtin);
    } else {
      const MsgSchema* sub = lookupNested(lookup, f.resolved_type);
      elem = sub ? minWireSize(*sub, lookup, depth + 1) : 0u;
    }
    if (!f.is_array) {
      add(elem);
    } else if (f.fixed_len < 0) {
      add(4u);
    } else {
      const size_t n = static_cast<size_t>(f.fixed_len);
      add((elem != 0 && n > std::numeric_limits<size_t>::max() / elem) ? std::numeric_limits<size_t>::max() : n * elem);
    }
  }
  return total;
}

bool readValue(BuiltinType t, Ros1Decoder::Cursor& c, Value* v) {
  v->type = t;
  switch (t) {
    case BuiltinType::Boolean: {
      uint8_t x;
      if (!c.readU8(&x)) return false;
      v->b = x != 0;
      return true;
    }
    case BuiltinType::Int8: {
      int8_t x;
      if (!c.readI8(&x)) return false;
      v->i = x;
      return true;
    }
    case BuiltinType::UInt8: {
      uint8_t x;
      if (!c.readU8(&x)) return false;
      v->u = x;
      return true;
    }
    case BuiltinType::Int16: {
      int16_t x;
      if (!c.readI16(&x)) return false;
      v->i = x;
      return true;
    }
    case BuiltinType::UInt16: {
      uint16_t x;
      if (!c.readU16(&x)) return false;
      v->u = x;
      return true;
    }
    case BuiltinType::Int32: {
      int32_t x;
      if (!c.readI32(&x)) return false;
      v->i = x;
      return true;
    }
    case BuiltinType::UInt32: {
      uint32_t x;
      if (!c.readU32(&x)) return false;
      v->u = x;
      return true;
    }
    case BuiltinType::Int64: {
      int64_t x;
      if (!c.readI64(&x)) return false;
      v->i = x;
      return true;
    }
    case BuiltinType::UInt64: {
      uint64_t x;
      if (!c.readU64(&x)) return false;
      v->u = x;
      return true;
    }
    case BuiltinType::Float32: {
      float x;
      if (!c.readF32(&x)) return false;
      v->f = static_cast<double>(x);
      return true;
    }
    case BuiltinType::Float64: return c.readF64(&v->f);
    case BuiltinType::String: return c.readString(&v->s);
    case BuiltinType::Time:
    case BuiltinType::Duration: return c.readU32(&v->sec) && c.readU32(&v->nsec);
    default: return false;
  }
}

struct Walker {
  const SchemaMap& lookup;
  Ros1Decoder::Cursor& c;
  std::string* why;
  size_t nodes_left = kMaxNodes;

  bool fail(const std::string& m) {
    if (why) *why = m + " at byte " + std::to_string(c.pos) + " of " + std::to_string(c.n);
    return false;
  }

  // Takes `n` nodes from the per-message budget before they are allocated.
  bool spend(size_t n, const std::string& path) {
    if (n > nodes_left)
      return fail("message exceeds the decoder's budget of " + std::to_string(kMaxNodes) + " tree nodes at '" + path + "'");
    nodes_left -= n;
    return true;
  }

  bool count(const FieldSpec& f, size_t elem_min, uint32_t* n, const std::string& path) {
    if (f.fixed_len >= 0) {
      *n = static_cast<uint32_t>(f.fixed_len);
    } else if (!c.readU32(n)) {
      return fail("truncated array length of '" + path + "'");
    }
    const size_t left = c.n - c.pos;
    if (elem_min == 0) {
      if (*n > kMaxEmptyElements) return fail("implausible element count " + std::to_string(*n) + " of '" + path + "'");
    } else if (static_cast<size_t>(*n) > left / elem_min) {
      return fail("array '" + path + "' claims " + std::to_string(*n) + " elements, only " + std::to_string(left) + " bytes left");
    }
    return spend(*n, path);
  }

  bool message(const MsgSchema& s, FieldNode* node, const std::string& path, int depth) {
    if (depth > kMaxDepth) return fail("nesting deeper than " + std::to_string(kMaxDepth));
    if (!spend(s.fields.size(), path.empty() ? std::string("<root>") : path)) return false;
    node->children.reserve(s.fields.size());
    for (const FieldSpec& f : s.fields) {
      const std::string fpath = path.empty() ? f.name : path + "." + f.name;
      FieldNode child;
      child.name = f.name;
      child.type = f.type;
      child.is_array = f.is_array;
      if (f.builtin != BuiltinType::NotBuiltin) {
        if (f.is_array) {
          const size_t elem = f.builtin == BuiltinType::String ? 4u : builtinSize(f.builtin);
          uint32_t n = 0;
          if (!count(f, elem, &n, fpath)) return false;
          child.array.resize(n);
          for (uint32_t i = 0; i < n; ++i)
            if (!readValue(f.builtin, c, &child.array[i])) return fail("truncated element " + std::to_string(i) + " of '" + fpath + "'");
        } else if (!readValue(f.builtin, c, &child.scalar)) {
          return fail("truncated field '" + fpath + "'");
        }
      } else {
        const MsgSchema* sub = lookupNested(lookup, f.resolved_type);
        if (!sub) return fail("unresolved nested type '" + f.resolved_type + "'");
        child.is_message = true;
        if (f.is_array) {
          uint32_t n = 0;
          if (!count(f, minWireSize(*sub, lookup, 0), &n, fpath)) return false;
          child.children.resize(n);
          for (uint32_t i = 0; i < n; ++i) {
            FieldNode& e = child.children[i];
            e.name = std::to_string(i);   // element nodes are addressable with child("<index>")
            e.type = f.type;
            e.is_message = true;
            if (!message(*sub, &e, fpath + "[" + std::to_string(i) + "]", depth + 1)) return false;
          }
        } else if (!message(*sub, &child, fpath, depth + 1)) {
          return false;
        }
      }
      node->children.push_back(std::move(child));
    }
    return true;
  }
};

uint64_t le(const uint8_t* p, int n) {
  uint64_t v = 0;
  for (int i = n - 1; i >= 0; --i) v = (v << 8) | p[i];
  return v;
}

}  // namespace

double Value::toDouble() const {
  switch (type) {
    case BuiltinType::Boolean: return b ? 1.0 : 0.0;
    case BuiltinType::Int8:
    case BuiltinType::Int16:
    case BuiltinType::Int32:
    case BuiltinType::Int64: return static_cast<double>(i);
    case BuiltinType::UInt8:
    case BuiltinType::UInt16:
    case BuiltinType::UInt32:
    case BuiltinType::UInt64: return static_cast<double>(u);
    case BuiltinType::Float32:
    case BuiltinType::Float64: return f;
    case BuiltinType::Time: return static_cast<double>(sec) + 1e-9 * static_cast<double>(nsec);
    case BuiltinType::Duration:   // ROS1 duration fields are signed int32 on the wire
      return static_cast<double>(static_cast<int32_t>(sec)) + 1e-9 * static_cast<double>(static_cast<int32_t>(nsec));
    default: return std::numeric_limits<double>::quiet_NaN();   // string / not a builtin: no numeric view
  }
}

const FieldNode* FieldNode::child(const std::string& name) const {
  for (const FieldNode& c : children)
    if (c.name == name) return &c;
  return nullptr;
}

Ros1Decoder::Ros1Decoder(const MsgSchema& schema) : schema_(schema) {}

bool Ros1Decoder::decode(const uint8_t* data, size_t size, FieldNode* out, size_t* consumed, std::string* why) const {
  if (consumed) *consumed = 0;
  if (!data && size != 0) {
    if (why) *why = "null buffer";
    return false;
  }
  Cursor c;
  c.p = data;
  c.n = size;
  c.pos = 0;
  FieldNode root;
  root.name = "";
  root.type = schema_.datatype;
  root.is_message = true;
  std::string w;
  Walker walker{schema_.nested, c, &w};
  bool ok = walker.message(schema_, &root, "", 0);
  if (consumed) *consumed = c.pos;
  if (ok && c.pos != size) {
    w = "trailing bytes: layout consumed " + std::to_string(c.pos) + " of " + std::to_string(size);
    ok = false;
  }
  if (!ok) {
    if (why) *why = w;
    return false;
  }
  if (out) *out = std::move(root);
  return true;
}

// ---- cursor ---------------------------------------------------------------------------------------
bool Ros1Decoder::Cursor::readU8(uint8_t* v) {
  if (!p || pos > n || n - pos < 1) return false;
  *v = p[pos];
  pos += 1;
  return true;
}
bool Ros1Decoder::Cursor::readU16(uint16_t* v) {
  if (!p || pos > n || n - pos < 2) return false;
  *v = static_cast<uint16_t>(le(p + pos, 2));
  pos += 2;
  return true;
}
bool Ros1Decoder::Cursor::readU32(uint32_t* v) {
  if (!p || pos > n || n - pos < 4) return false;
  *v = static_cast<uint32_t>(le(p + pos, 4));
  pos += 4;
  return true;
}
bool Ros1Decoder::Cursor::readU64(uint64_t* v) {
  if (!p || pos > n || n - pos < 8) return false;
  *v = le(p + pos, 8);
  pos += 8;
  return true;
}
bool Ros1Decoder::Cursor::readI8(int8_t* v) {
  uint8_t u;
  if (!readU8(&u)) return false;
  *v = static_cast<int8_t>(u);
  return true;
}
bool Ros1Decoder::Cursor::readI16(int16_t* v) {
  uint16_t u;
  if (!readU16(&u)) return false;
  *v = static_cast<int16_t>(u);
  return true;
}
bool Ros1Decoder::Cursor::readI32(int32_t* v) {
  uint32_t u;
  if (!readU32(&u)) return false;
  *v = static_cast<int32_t>(u);
  return true;
}
bool Ros1Decoder::Cursor::readI64(int64_t* v) {
  uint64_t u;
  if (!readU64(&u)) return false;
  *v = static_cast<int64_t>(u);
  return true;
}
bool Ros1Decoder::Cursor::readF32(float* v) {
  uint32_t u;
  if (!readU32(&u)) return false;
  std::memcpy(v, &u, sizeof(u));   // IEEE-754 binary32 bit pattern, byte order already fixed by readU32
  return true;
}
bool Ros1Decoder::Cursor::readF64(double* v) {
  uint64_t u;
  if (!readU64(&u)) return false;
  std::memcpy(v, &u, sizeof(u));
  return true;
}
bool Ros1Decoder::Cursor::readString(std::string* v, size_t max_len) {
  const size_t start = pos;
  uint32_t len = 0;
  if (!readU32(&len)) return false;
  if (len > max_len || n - pos < len) {
    pos = start;
    return false;
  }
  v->assign(reinterpret_cast<const char*>(p + pos), len);
  pos += len;
  return true;
}

}  // namespace livo_gnss_io
