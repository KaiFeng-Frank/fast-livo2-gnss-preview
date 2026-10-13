// SPDX-License-Identifier: BSD-3-Clause
// ros1_msgdef (DESIGN §12.2). Owner: WP-e. Tests: U2 (test_u02_wire.cpp), T2 (test_t01_t05_io.cpp).
//
// The parsing and md5 rules reproduce the ROS1 message generator (genmsg msg_loader / gentools
// compute_md5_text) as observed on real connection headers:
//   * a line is stripped of everything from the first '#'; blank lines are ignored;
//   * a comment-stripped line containing '=' is a constant (not part of the wire layout; parseConstantLine
//     below follows genmsg _load_constant_line literally, including its quirks, because the constant's text
//     enters the md5 verbatim);
//   * otherwise "<type> <name>", type optionally suffixed "[]" / "[N]" (tokens split on any blank; genmsg
//     splits on spaces only, so a tab-separated field line is accepted here where genmsg would refuse it);
//   * unqualified message types resolve into the section's package, bare "Header" to std_msgs/Header;
//   * md5 text = constants ("T N=V", MsgSchema::constants, definition order) then fields, builtins as written
//     ("float64[] x"), message fields as "<md5 of the nested type> name" (array suffix dropped), joined by '\n'.
// Checked against md5sums recorded in M3DGR connection headers (U2/T2: the six gnss_comm types present,
// std_msgs/Header 2176deca…, sensor_msgs/NavSatFix 2d3a8cd4… whose closure has constants) and against genmsg
// itself on hand-written definitions (U2 Md5WithConstants).
#include "livo_gnss_io/ros1_msgdef.hpp"

#include <cctype>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <functional>
#include <set>

namespace livo_gnss_io {

namespace {

constexpr int kMaxDepth = 32;              // nesting guard (ROS1 forbids recursion; this bounds garbage input)
constexpr long kMaxFixedLen = 1L << 24;    // fixed-array length guard (keeps size arithmetic far from overflow)

// ---- MD5 (RFC 1321), used only for the ROS1 md5sum of a definition --------------------------------
std::string md5Hex(const std::string& msg) {
  static const uint32_t K[64] = {
      0xd76aa478u, 0xe8c7b756u, 0x242070dbu, 0xc1bdceeeu, 0xf57c0fafu, 0x4787c62au, 0xa8304613u, 0xfd469501u,
      0x698098d8u, 0x8b44f7afu, 0xffff5bb1u, 0x895cd7beu, 0x6b901122u, 0xfd987193u, 0xa679438eu, 0x49b40821u,
      0xf61e2562u, 0xc040b340u, 0x265e5a51u, 0xe9b6c7aau, 0xd62f105du, 0x02441453u, 0xd8a1e681u, 0xe7d3fbc8u,
      0x21e1cde6u, 0xc33707d6u, 0xf4d50d87u, 0x455a14edu, 0xa9e3e905u, 0xfcefa3f8u, 0x676f02d9u, 0x8d2a4c8au,
      0xfffa3942u, 0x8771f681u, 0x6d9d6122u, 0xfde5380cu, 0xa4beea44u, 0x4bdecfa9u, 0xf6bb4b60u, 0xbebfbc70u,
      0x289b7ec6u, 0xeaa127fau, 0xd4ef3085u, 0x04881d05u, 0xd9d4d039u, 0xe6db99e5u, 0x1fa27cf8u, 0xc4ac5665u,
      0xf4292244u, 0x432aff97u, 0xab9423a7u, 0xfc93a039u, 0x655b59c3u, 0x8f0ccc92u, 0xffeff47du, 0x85845dd1u,
      0x6fa87e4fu, 0xfe2ce6e0u, 0xa3014314u, 0x4e0811a1u, 0xf7537e82u, 0xbd3af235u, 0x2ad7d2bbu, 0xeb86d391u};
  static const int S[64] = {7, 12, 17, 22, 7, 12, 17, 22, 7, 12, 17, 22, 7, 12, 17, 22,
                            5, 9,  14, 20, 5, 9,  14, 20, 5, 9,  14, 20, 5, 9,  14, 20,
                            4, 11, 16, 23, 4, 11, 16, 23, 4, 11, 16, 23, 4, 11, 16, 23,
                            6, 10, 15, 21, 6, 10, 15, 21, 6, 10, 15, 21, 6, 10, 15, 21};
  std::vector<uint8_t> buf(msg.begin(), msg.end());
  const uint64_t bit_len = static_cast<uint64_t>(msg.size()) * 8u;
  buf.push_back(0x80u);
  while (buf.size() % 64u != 56u) buf.push_back(0u);
  for (int i = 0; i < 8; ++i) buf.push_back(static_cast<uint8_t>(bit_len >> (8 * i)));

  uint32_t a0 = 0x67452301u, b0 = 0xefcdab89u, c0 = 0x98badcfeu, d0 = 0x10325476u;
  auto rotl = [](uint32_t x, int c) { return (x << c) | (x >> (32 - c)); };
  for (size_t off = 0; off < buf.size(); off += 64) {
    uint32_t M[16];
    for (int i = 0; i < 16; ++i) {
      const uint8_t* p = &buf[off + 4u * static_cast<size_t>(i)];
      M[i] = static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) | (static_cast<uint32_t>(p[2]) << 16) |
             (static_cast<uint32_t>(p[3]) << 24);
    }
    uint32_t A = a0, B = b0, C = c0, D = d0;
    for (int i = 0; i < 64; ++i) {
      uint32_t F;
      int g;
      if (i < 16) {
        F = (B & C) | (~B & D);
        g = i;
      } else if (i < 32) {
        F = (D & B) | (~D & C);
        g = (5 * i + 1) % 16;
      } else if (i < 48) {
        F = B ^ C ^ D;
        g = (3 * i + 5) % 16;
      } else {
        F = C ^ (B | ~D);
        g = (7 * i) % 16;
      }
      F = F + A + K[i] + M[g];
      A = D;
      D = C;
      C = B;
      B = B + rotl(F, S[i]);
    }
    a0 += A;
    b0 += B;
    c0 += C;
    d0 += D;
  }
  static const char* hex = "0123456789abcdef";
  std::string out;
  out.reserve(32);
  for (uint32_t w : {a0, b0, c0, d0}) {
    for (int i = 0; i < 4; ++i) {
      const uint8_t byte = static_cast<uint8_t>(w >> (8 * i));
      out.push_back(hex[byte >> 4]);
      out.push_back(hex[byte & 0x0f]);
    }
  }
  return out;
}

// ---- small string helpers -------------------------------------------------------------------------
bool isSpace(char c) { return c == ' ' || c == '\t' || c == '\r' || c == '\n' || c == '\v' || c == '\f'; }

std::string trim(const std::string& s) {
  size_t b = 0, e = s.size();
  while (b < e && isSpace(s[b])) ++b;
  while (e > b && isSpace(s[e - 1])) --e;
  return s.substr(b, e - b);
}

std::vector<std::string> splitLines(const std::string& text) {
  std::vector<std::string> lines;
  size_t start = 0;
  while (start <= text.size()) {
    size_t nl = text.find('\n', start);
    if (nl == std::string::npos) nl = text.size();
    std::string line = text.substr(start, nl - start);
    if (!line.empty() && line.back() == '\r') line.pop_back();
    lines.push_back(line);
    if (nl == text.size()) break;
    start = nl + 1;
  }
  return lines;
}

// Section separator of a concatenated definition: a line made only of '=' (rosbag/genpy write 80).
bool isSeparator(const std::string& line) {
  const std::string t = trim(line);
  if (t.size() < 3) return false;
  for (char c : t)
    if (c != '=') return false;
  return true;
}

// "pkg/msg/Type" (ROS2-style names, e.g. from rosbags) → "pkg/Type"; surrounding blanks removed.
std::string normalizeType(const std::string& t) {
  std::string s = trim(t);
  const size_t a = s.find('/');
  if (a != std::string::npos) {
    const size_t b = s.find('/', a + 1);
    if (b != std::string::npos && s.compare(a + 1, b - a - 1, "msg") == 0) s = s.substr(0, a) + s.substr(b);
  }
  return s;
}

std::string packageOf(const std::string& qualified) {
  const size_t a = qualified.find('/');
  return a == std::string::npos ? std::string() : qualified.substr(0, a);
}

bool isIdentStart(char c) { return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z'); }
bool isIdentChar(char c) { return isIdentStart(c) || (c >= '0' && c <= '9') || c == '_'; }

// genmsg is_legal_resource_base_name for field names.
bool validFieldName(const std::string& n) {
  if (n.empty() || !isIdentStart(n[0])) return false;
  for (char c : n)
    if (!isIdentChar(c)) return false;
  return true;
}

// Base type token: "Type", "pkg/Type" or "pkg/msg/Type"; each part an identifier.
bool validTypeBase(const std::string& t) {
  if (t.empty()) return false;
  int parts = 0;
  size_t start = 0;
  while (true) {
    const size_t slash = t.find('/', start);
    const std::string part = t.substr(start, slash == std::string::npos ? std::string::npos : slash - start);
    if (part.empty() || !isIdentStart(part[0])) return false;
    for (char c : part)
      if (!isIdentChar(c)) return false;
    ++parts;
    if (slash == std::string::npos) break;
    start = slash + 1;
  }
  return parts <= 3;
}

std::string arraySuffix(const FieldSpec& f) {
  if (!f.is_array) return std::string();
  if (f.fixed_len < 0) return "[]";
  return "[" + std::to_string(f.fixed_len) + "]";
}

// ---- constants (genmsg msg_loader._load_constant_line + convert_constant_value) ----------------------------
// Python's str.strip() on the ASCII range (str.isspace(): space, \t \n \v \f \r and \x1c–\x1f). Definitions are
// ASCII in practice; non-ASCII whitespace (U+00A0 …) is not stripped here.
bool pySpace(char c) { return c == ' ' || (c >= '\t' && c <= '\r') || (c >= '\x1c' && c <= '\x1f'); }

std::string pyStrip(const std::string& s) {
  size_t b = 0, e = s.size();
  while (b < e && pySpace(s[b])) ++b;
  while (e > b && pySpace(s[e - 1])) --e;
  return s.substr(b, e - b);
}

// genmsg _strip_comments(): line.split('#')[0].strip()
std::string stripComment(const std::string& line) { return pyStrip(line.substr(0, line.find('#'))); }

// genmsg PRIMITIVE_TYPES: the only legal constant types (no time/duration, no arrays, no messages).
bool isConstantType(const std::string& t) {
  static const std::set<std::string> kTypes = {"int8",  "uint8",   "int16",   "uint16", "int32", "uint32", "int64",
                                               "uint64", "float32", "float64", "string", "bool",  "char",   "byte"};
  return kTypes.count(t) > 0;
}

// Python int() of a stripped decimal literal: [+-] digit (['_'] digit)*. `digits` = the magnitude without underscores
// and leading zeros ("0" for zero).
bool pyIntLiteral(const std::string& s, bool* negative, std::string* digits) {
  size_t i = 0;
  *negative = false;
  digits->clear();
  if (i < s.size() && (s[i] == '+' || s[i] == '-')) *negative = s[i++] == '-';
  bool digit_last = false;
  for (; i < s.size(); ++i) {
    const char c = s[i];
    if (c >= '0' && c <= '9') {
      if (!(digits->empty() && c == '0')) digits->push_back(c);
      digit_last = true;
    } else if (c == '_' && digit_last) {
      digit_last = false;   // a single underscore between digits
    } else {
      return false;
    }
  }
  if (!digit_last) return false;   // empty, sign only, or a trailing underscore
  if (digits->empty()) *digits = "0";
  return true;
}

// a <= b for decimal digit strings without leading zeros
bool decimalLE(const std::string& a, const std::string& b) { return a.size() != b.size() ? a.size() < b.size() : a <= b; }

// genmsg's integer bounds, [lower, upper] as magnitudes: unsigned (uint*, char) 0 .. int(math.pow(2, b) - 1), signed
// (int*, byte) -upper-1 .. int(math.pow(2, b-1) - 1). math.pow is a double: for b = 64 the "- 1" is lost, so genmsg
// accepts uint64 up to 2^64 and int64 over [-2^63 - 1, 2^63] — reproduced, it is what genmsg let through.
bool genmsgIntInRange(const std::string& type, bool negative, const std::string& digits) {
  struct B {
    const char* type;
    const char* upper;
    const char* lower_mag;   // "0": unsigned
  };
  static const B kBounds[] = {{"int8", "127", "128"},
                              {"byte", "127", "128"},
                              {"uint8", "255", "0"},
                              {"char", "255", "0"},
                              {"int16", "32767", "32768"},
                              {"uint16", "65535", "0"},
                              {"int32", "2147483647", "2147483648"},
                              {"uint32", "4294967295", "0"},
                              {"int64", "9223372036854775808", "9223372036854775809"},
                              {"uint64", "18446744073709551616", "0"}};
  for (const B& b : kBounds)
    if (type == b.type) return decimalLE(digits, negative ? b.lower_mag : b.upper);
  return false;
}

// Python float() of a stripped literal: [+-] (inf | infinity | nan | (digitpart ['.' [digitpart]] | '.' digitpart)
// [(e|E) [+-] digitpart]), digitpart = digit (['_'] digit)*, keywords case-insensitive.
bool pyFloatLiteral(const std::string& s) {
  size_t i = 0;
  if (i < s.size() && (s[i] == '+' || s[i] == '-')) ++i;
  std::string rest = s.substr(i);
  for (char& c : rest) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  if (rest == "inf" || rest == "infinity" || rest == "nan") return true;
  auto digitpart = [&](size_t* p) {   // true when at least one digit was consumed and the part ended on a digit
    bool digit_last = false, any = false;
    while (*p < s.size()) {
      const char c = s[*p];
      if (c >= '0' && c <= '9') {
        digit_last = any = true;
      } else if (c == '_' && digit_last && *p + 1 < s.size() && s[*p + 1] >= '0' && s[*p + 1] <= '9') {
        digit_last = false;
      } else {
        break;
      }
      ++*p;
    }
    return any && digit_last;
  };
  const bool int_part = digitpart(&i);
  bool frac_part = false;
  if (i < s.size() && s[i] == '.') {
    ++i;
    frac_part = digitpart(&i);   // "5." is a float; "1._5" stops at '_' and fails the end check below
  }
  if (!int_part && !frac_part) return false;
  if (i < s.size() && (s[i] == 'e' || s[i] == 'E')) {
    ++i;
    if (i < s.size() && (s[i] == '+' || s[i] == '-')) ++i;
    if (!digitpart(&i)) return false;
  }
  return i == s.size();
}

// One constant line (its comment-stripped text contains '='), reproduced from genmsg msg_loader:
//   clean = strip(line before the first '#'); tokens = [t.strip() for t in clean.split(" ") if t.strip()];
//   type = tokens[0], a primitive type;
//   string:    name = strip(line[line.find(' ')+1 : line.find('=')]), value = strip(line[line.find('=')+1:])
//              — taken from the whole line, so a '#' after the '=' belongs to the value; genmsg keys the name on
//              the first *space* of the line, so an indented string constant gets "string NAME" as its name (kept:
//              that is the text genmsg hashed);
//   otherwise: name, value = strip-each(" ".join(tokens[1:]).split("=")), exactly two parts; the value must
//              convert: int types by Python int() within genmsg's range (genmsgIntInRange), float types by Python
//              float(); a bool value genmsg evaluates as a Python literal (ast.literal_eval) — not reproduced here: any
//              non-empty value is accepted, which only widens acceptance for definitions genmsg itself refuses.
// Constant names are not checked (genmsg does not check them either). `md5_line` = "T NAME=VALUE" as
// compute_md5_text writes it (the value text as written, not the converted value).
bool parseConstantLine(const std::string& line, std::string* md5_line, std::string* why) {
  auto fail = [&](const std::string& m) {
    if (why) *why = m + " in '" + pyStrip(line) + "'";
    return false;
  };
  const std::string clean = stripComment(line);
  std::vector<std::string> tok;
  for (size_t start = 0; start <= clean.size();) {
    size_t sp = clean.find(' ', start);
    if (sp == std::string::npos) sp = clean.size();
    const std::string t = pyStrip(clean.substr(start, sp - start));
    if (!t.empty()) tok.push_back(t);
    start = sp + 1;
  }
  if (tok.empty()) return fail("empty constant declaration");
  const std::string& type = tok[0];
  if (!isConstantType(type)) return fail("'" + type + "' is not a legal constant type");
  std::string name, value;
  if (type == "string") {
    const size_t eq = line.find('=');
    const size_t sp = line.find(' ');
    const size_t from = sp == std::string::npos ? 0 : sp + 1;   // Python find() gives -1 → slice from 0
    name = from <= eq ? pyStrip(line.substr(from, eq - from)) : std::string();
    value = pyStrip(line.substr(eq + 1));
  } else {
    std::string joined;
    for (size_t k = 1; k < tok.size(); ++k) joined += (k > 1 ? " " : "") + tok[k];
    const size_t eq = joined.find('=');
    if (eq == std::string::npos || joined.find('=', eq + 1) != std::string::npos) return fail("invalid constant declaration");
    name = pyStrip(joined.substr(0, eq));
    value = pyStrip(joined.substr(eq + 1));
    if (type == "float32" || type == "float64") {
      if (!pyFloatLiteral(value)) return fail("invalid constant value '" + value + "' for " + type);
    } else if (type == "bool") {
      if (value.empty()) return fail("empty constant value for bool");
    } else {
      bool neg = false;
      std::string digits;
      if (!pyIntLiteral(value, &neg, &digits)) return fail("invalid constant value '" + value + "' for " + type);
      const bool in_range = genmsgIntInRange(type, neg, digits);
      if (!in_range) return fail("constant value '" + value + "' out of range for " + type);
    }
  }
  if (md5_line) *md5_line = type + " " + name + "=" + value;
  return true;
}

using SchemaMap = std::map<std::string, std::shared_ptr<MsgSchema>>;

void flattenRec(const MsgSchema& s, const SchemaMap& lookup, const std::string& prefix,
                std::vector<std::pair<std::string, std::string>>* out, int depth) {
  if (depth > kMaxDepth) return;
  for (const FieldSpec& f : s.fields) {
    const std::string name = prefix.empty() ? f.name : prefix + "." + f.name;
    if (f.builtin != BuiltinType::NotBuiltin) {
      out->emplace_back(std::string(builtinName(f.builtin)) + arraySuffix(f), name);
      continue;
    }
    out->emplace_back(f.resolved_type + arraySuffix(f), name);
    const auto it = lookup.find(f.resolved_type);
    // An unresolved nested type leaves only its own entry; the expectation comparison then fails on the
    // missing children, which is the desired outcome for a hand-built inconsistent schema.
    if (it != lookup.end() && it->second) flattenRec(*it->second, lookup, name, out, depth + 1);
  }
}

// Empty string when a nested type cannot be resolved (only possible for hand-built schemas).
std::string md5Rec(const MsgSchema& s, const SchemaMap& lookup, int depth) {
  if (depth > kMaxDepth) return std::string();
  std::string text;
  for (const std::string& c : s.constants) text += c + "\n";   // genmsg lists the constants first
  for (const FieldSpec& f : s.fields) {
    if (f.builtin != BuiltinType::NotBuiltin) {
      text += f.type + arraySuffix(f) + " " + f.name + "\n";   // type as written (byte/char kept, genmsg)
      continue;
    }
    const auto it = lookup.find(f.resolved_type);
    if (it == lookup.end() || !it->second) return std::string();
    const std::string sub = md5Rec(*it->second, lookup, depth + 1);
    if (sub.empty()) return std::string();
    text += sub + " " + f.name + "\n";                          // array suffix dropped for message fields
  }
  if (!text.empty() && text.back() == '\n') text.pop_back();
  return md5Hex(text);
}

}  // namespace

// ---- builtins --------------------------------------------------------------------------------------
const char* builtinName(BuiltinType t) {
  switch (t) {
    case BuiltinType::Boolean: return "bool";
    case BuiltinType::Int8: return "int8";
    case BuiltinType::UInt8: return "uint8";
    case BuiltinType::Int16: return "int16";
    case BuiltinType::UInt16: return "uint16";
    case BuiltinType::Int32: return "int32";
    case BuiltinType::UInt32: return "uint32";
    case BuiltinType::Int64: return "int64";
    case BuiltinType::UInt64: return "uint64";
    case BuiltinType::Float32: return "float32";
    case BuiltinType::Float64: return "float64";
    case BuiltinType::String: return "string";
    case BuiltinType::Time: return "time";
    case BuiltinType::Duration: return "duration";
    default: return "";
  }
}

BuiltinType builtinFromName(const std::string& name) {
  static const std::map<std::string, BuiltinType> table = {
      {"bool", BuiltinType::Boolean},   {"int8", BuiltinType::Int8},       {"uint8", BuiltinType::UInt8},
      {"int16", BuiltinType::Int16},    {"uint16", BuiltinType::UInt16},   {"int32", BuiltinType::Int32},
      {"uint32", BuiltinType::UInt32},  {"int64", BuiltinType::Int64},     {"uint64", BuiltinType::UInt64},
      {"float32", BuiltinType::Float32}, {"float64", BuiltinType::Float64}, {"string", BuiltinType::String},
      {"time", BuiltinType::Time},      {"duration", BuiltinType::Duration},
      // deprecated ROS1 aliases (genmsg): byte = int8, char = uint8
      {"byte", BuiltinType::Int8},      {"char", BuiltinType::UInt8}};
  const auto it = table.find(name);
  return it == table.end() ? BuiltinType::NotBuiltin : it->second;
}

size_t builtinSize(BuiltinType t) {
  switch (t) {
    case BuiltinType::Boolean:
    case BuiltinType::Int8:
    case BuiltinType::UInt8: return 1;
    case BuiltinType::Int16:
    case BuiltinType::UInt16: return 2;
    case BuiltinType::Int32:
    case BuiltinType::UInt32:
    case BuiltinType::Float32: return 4;
    case BuiltinType::Int64:
    case BuiltinType::UInt64:
    case BuiltinType::Float64:
    case BuiltinType::Time:
    case BuiltinType::Duration: return 8;
    default: return 0;   // String (variable) and NotBuiltin
  }
}

// ---- schema -----------------------------------------------------------------------------------------
std::vector<std::pair<std::string, std::string>> MsgSchema::flatten() const {
  std::vector<std::pair<std::string, std::string>> out;
  flattenRec(*this, nested, "", &out, 0);
  return out;
}

// ---- parser -----------------------------------------------------------------------------------------
std::vector<std::pair<std::string, std::string>> MsgDefParser::splitSections(const std::string& datatype,
                                                                           const std::string& msg_def) {
  std::vector<std::pair<std::string, std::string>> sections;
  std::string name = normalizeType(datatype), body;
  bool expect_msg = false;   // after a separator the next non-blank line must be "MSG: <type>"
  for (const std::string& line : splitLines(msg_def)) {
    if (isSeparator(line)) {
      sections.emplace_back(name, body);
      name.clear();
      body.clear();
      expect_msg = true;
      continue;
    }
    if (expect_msg) {
      const std::string t = trim(line);
      if (t.empty()) continue;
      expect_msg = false;
      if (t.compare(0, 4, "MSG:") == 0) {
        name = normalizeType(t.substr(4));
        continue;
      }
      // Not a MSG: line: keep it in an unnamed section so that parse() reports the malformation.
    }
    body += line;
    body += '\n';
  }
  sections.emplace_back(name, body);
  return sections;
}

bool MsgDefParser::parseFieldLine(const std::string& line, const std::string& package, FieldSpec* out, std::string* why) {
  if (why) why->clear();
  std::string s = line;
  const size_t hash = s.find('#');
  if (hash != std::string::npos) s.erase(hash);
  s = trim(s);
  if (s.empty()) return false;                     // blank / comment-only line
  if (s.find('=') != std::string::npos) return false;   // constant: no wire layout (genmsg CONSTCHAR rule)

  std::vector<std::string> tok;
  size_t i = 0;
  while (i < s.size()) {
    while (i < s.size() && isSpace(s[i])) ++i;
    size_t j = i;
    while (j < s.size() && !isSpace(s[j])) ++j;
    if (j > i) tok.push_back(s.substr(i, j - i));
    i = j;
  }
  auto fail = [&](const std::string& m) {
    if (why) *why = m + " in '" + trim(line) + "'";
    return false;
  };
  if (tok.size() != 2) return fail("expected '<type> <name>'");
  const std::string& type_tok = tok[0];
  const std::string& name = tok[1];
  if (!validFieldName(name)) return fail("illegal field name '" + name + "'");

  FieldSpec f;
  std::string base = type_tok;
  const size_t lb = type_tok.find('[');
  if (lb != std::string::npos) {
    base = type_tok.substr(0, lb);
    if (type_tok.back() != ']') return fail("malformed array suffix");
    const std::string digits = type_tok.substr(lb + 1, type_tok.size() - lb - 2);
    f.is_array = true;
    if (digits.empty()) {
      f.fixed_len = -1;
    } else {
      long n = 0;
      for (char c : digits) {
        if (c < '0' || c > '9') return fail("malformed array length");
        n = n * 10 + (c - '0');
        if (n > kMaxFixedLen) return fail("array length too large");
      }
      f.fixed_len = static_cast<int>(n);
    }
  }
  if (!validTypeBase(base)) return fail("illegal type '" + base + "'");
  f.type = base;
  f.name = name;
  f.builtin = builtinFromName(base);
  if (f.builtin == BuiltinType::NotBuiltin) {
    // genmsg: only the exact token "Header" maps to std_msgs/Header (an array of Header resolves into the package).
    if (type_tok == "Header") {
      f.resolved_type = "std_msgs/Header";
    } else if (base.find('/') != std::string::npos) {
      f.resolved_type = normalizeType(base);
    } else if (!package.empty()) {
      f.resolved_type = package + "/" + base;
    } else {
      f.resolved_type = base;
    }
  }
  if (out) *out = f;
  return true;
}

bool MsgDefParser::parse(const std::string& datatype, const std::string& msg_def, MsgSchema* out, std::string* why) {
  auto fail = [&](const std::string& m) {
    if (why) *why = m;
    return false;
  };
  if (!out) return fail("null output schema");
  const std::string root = normalizeType(datatype);
  if (root.empty() || root.find('/') == std::string::npos || !validTypeBase(root))
    return fail("datatype '" + datatype + "' is not <package>/<Type>");

  SchemaMap by_name;
  for (const auto& sec : splitSections(root, msg_def)) {
    const std::string& name = sec.first;
    if (name.empty() && trim(sec.second).empty()) continue;   // trailing separator with nothing after it
    if (name.empty()) return fail("dependency section without a 'MSG: <package>/<Type>' line");
    if (name.find('/') == std::string::npos || !validTypeBase(name)) return fail("bad section type '" + name + "'");
    auto sch = std::make_shared<MsgSchema>();
    sch->datatype = name;
    const std::string pkg = packageOf(name);
    int line_no = 0;
    std::set<std::string> names;
    for (const std::string& line : splitLines(sec.second)) {
      ++line_no;
      FieldSpec f;
      std::string w;
      if (parseFieldLine(line, pkg, &f, &w)) {
        if (!names.insert(f.name).second) return fail(name + ": duplicate field '" + f.name + "'");
        sch->fields.push_back(f);
      } else if (!w.empty()) {
        return fail(name + " line " + std::to_string(line_no) + ": " + w);
      } else if (stripComment(line).find('=') != std::string::npos) {   // the lines parseFieldLine leaves as constants
        std::string c;
        if (!parseConstantLine(line, &c, &w)) return fail(name + " line " + std::to_string(line_no) + ": " + w);
        sch->constants.push_back(c);
      }
    }
    const auto it = by_name.find(name);
    if (it != by_name.end()) {
      // A repeated section is tolerated only when it is the same definition (field by field, same constants).
      const auto a = it->second->fields;
      bool same = a.size() == sch->fields.size() && it->second->constants == sch->constants;
      for (size_t k = 0; same && k < a.size(); ++k)
        same = a[k].type == sch->fields[k].type && a[k].name == sch->fields[k].name && a[k].is_array == sch->fields[k].is_array &&
               a[k].fixed_len == sch->fields[k].fixed_len;
      if (!same) return fail("conflicting definitions of '" + name + "'");
      continue;
    }
    by_name.emplace(name, sch);
  }

  // Resolve nested types and reject cycles (ROS1 has no recursive messages; md5 would not terminate).
  std::map<std::string, int> state;   // 1 = on the DFS stack, 2 = done
  std::string err;
  std::function<bool(const std::string&, int)> visit = [&](const std::string& n, int depth) -> bool {
    if (depth > kMaxDepth) {
      err = "nesting deeper than " + std::to_string(kMaxDepth);
      return false;
    }
    const int st = state[n];
    if (st == 2) return true;
    if (st == 1) {
      err = "recursive message type '" + n + "'";
      return false;
    }
    const auto it = by_name.find(n);
    if (it == by_name.end()) {
      err = "unresolved nested type '" + n + "' (no 'MSG: " + n + "' section)";
      return false;
    }
    state[n] = 1;
    for (const FieldSpec& f : it->second->fields)
      if (f.builtin == BuiltinType::NotBuiltin && !visit(f.resolved_type, depth + 1)) return false;
    state[n] = 2;
    return true;
  };
  if (!visit(root, 0)) return fail(err);

  // Every reachable schema carries the transitive closure of its dependencies (shared, immutable), so a
  // nested schema can be flattened / hashed / decoded on its own.
  std::function<void(const std::string&, SchemaMap*)> collect = [&](const std::string& n, SchemaMap* acc) {
    for (const FieldSpec& f : by_name.at(n)->fields) {
      if (f.builtin != BuiltinType::NotBuiltin || acc->count(f.resolved_type)) continue;
      acc->emplace(f.resolved_type, by_name.at(f.resolved_type));
      collect(f.resolved_type, acc);
    }
  };
  for (const auto& kv : state) {
    SchemaMap closure;
    collect(kv.first, &closure);
    by_name.at(kv.first)->nested = closure;
  }
  *out = *by_name.at(root);
  return true;
}

// genmsg compute_md5: the constants parse() recorded (MsgSchema::constants, WP-e #1) come first in every type's
// text, nested types included. A hand-built schema that has constants but did not record them still gets a wrong
// value (header comment, WP-e #2); every schema made by parse() carries them.
std::string MsgDefParser::md5(const MsgSchema& schema) { return md5Rec(schema, schema.nested, 0); }

}  // namespace livo_gnss_io
