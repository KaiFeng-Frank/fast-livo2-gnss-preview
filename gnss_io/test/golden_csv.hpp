// SPDX-License-Identifier: BSD-3-Clause
// golden_csv.hpp — test support for the A24 fixture (header-only, gtest-free). Owner: WP-e.
//   * fixture discovery: fixture_discovery.hpp (the single rule, shared with WP-a);
//   * loading msgdef/<topic>.{msgdef,md5,type} and bytes/<topic>.bin records;
//   * RFC-4180 CSV parsing;
//   * goldenCsv(): the C++ twin of tools/gnss_bag_export.py's golden CSV writer (T3 compares both byte for
//     byte; the format is documented in the exporter's docstring and gnss/test/data/README.md);
//   * rowToTree(): golden CSV row → FieldNode (the inverse and the reference reader), so a test can take the
//     rosbags-decoded values through the same mapping code as the C++ decoder output (U2 here; usable by WP-a's
//     U6 tier 1 via #include "../../gnss_io/test/golden_csv.hpp");
//   * compareTrees(): structural + exact value comparison of two field trees.
//
// Variable-length arrays and the union columns (format contract, INTERFACE_CHANGES WP-e #4): one row per
// message, so the column set of an array path is the union over all rows (its longest occurrence). An element a
// row does not have leaves *all* its cells empty; an element it has always owns at least one non-empty cell (the
// exporter refuses to write an empty string as an array element or a message element with no non-empty leaf).
// Element presence is therefore decided by non-empty cells, never by the existence of a column: in a real
// range_meas slice most rows carry fewer satellites than the union has columns for.
#pragma once

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "fixture_discovery.hpp"
#include "livo_gnss_io/ros1_decoder.hpp"
#include "livo_gnss_io/ros1_msgdef.hpp"

namespace livo_gnss_io {
namespace testsupport {

// ------------------------------------------------------------------------------------------- files
inline bool readFile(const std::string& path, std::string* out) {
  std::ifstream f(path, std::ios::binary);
  if (!f) return false;
  std::ostringstream ss;
  ss << f.rdbuf();
  *out = ss.str();
  return true;
}

// Topic short name → golden CSV name (A24).
inline const std::vector<std::pair<std::string, std::string>>& topicCsv() {
  static const std::vector<std::pair<std::string, std::string>> t = {{"range_meas", "obs"}, {"ephem", "ephem"},
                                                                     {"glo_ephem", "glo_ephem"}, {"iono_params", "iono"},
                                                                     {"receiver_pvt", "pvt"}, {"time_pulse_info", "time_pulse"}};
  return t;
}

inline std::string trimmed(std::string s) {
  while (!s.empty() && (s.back() == '\n' || s.back() == '\r' || s.back() == ' ')) s.pop_back();
  return s;
}

struct TopicSchema {
  std::string topic, datatype, md5, msgdef;
  MsgSchema schema;
};

// Loads msgdef/<topic>.{msgdef,md5,type} and parses it. false (+why) when absent or unparsable.
inline bool loadTopic(const std::string& fixture, const std::string& topic, TopicSchema* out, std::string* why) {
  const std::string b = fixture + "/msgdef/" + topic;
  out->topic = topic;
  if (!readFile(b + ".msgdef", &out->msgdef) || !readFile(b + ".md5", &out->md5) || !readFile(b + ".type", &out->datatype)) {
    if (why) *why = "missing " + b + ".{msgdef,md5,type}";
    return false;
  }
  out->md5 = trimmed(out->md5);
  out->datatype = trimmed(out->datatype);
  return MsgDefParser::parse(out->datatype, out->msgdef, &out->schema, why);
}

struct Record {
  uint64_t t_ns = 0;
  std::vector<uint8_t> data;
};

// bytes/<topic>.bin: [uint64 t_record_ns][uint32 len][len bytes] ..., little-endian, record order.
inline bool readRecords(const std::string& path, std::vector<Record>* out, std::string* why) {
  std::string raw;
  if (!readFile(path, &raw)) {
    if (why) *why = "cannot read " + path;
    return false;
  }
  out->clear();
  size_t p = 0;
  auto le = [&](size_t at, int n) {
    uint64_t v = 0;
    for (int i = n - 1; i >= 0; --i) v = (v << 8) | static_cast<uint8_t>(raw[at + static_cast<size_t>(i)]);
    return v;
  };
  while (p < raw.size()) {
    if (raw.size() - p < 12) {
      if (why) *why = path + ": truncated record header";
      return false;
    }
    Record r;
    r.t_ns = le(p, 8);
    const size_t len = static_cast<size_t>(le(p + 8, 4));
    p += 12;
    if (raw.size() - p < len) {
      if (why) *why = path + ": truncated record body";
      return false;
    }
    r.data.assign(raw.begin() + static_cast<std::ptrdiff_t>(p), raw.begin() + static_cast<std::ptrdiff_t>(p + len));
    p += len;
    out->push_back(std::move(r));
  }
  return true;
}

// ------------------------------------------------------------------------------------------- CSV parsing
inline std::vector<std::string> splitCsvLine(const std::string& line) {
  std::vector<std::string> cells;
  std::string cur;
  bool quoted = false;
  for (size_t i = 0; i < line.size(); ++i) {
    const char c = line[i];
    if (quoted) {
      if (c == '"') {
        if (i + 1 < line.size() && line[i + 1] == '"') {
          cur.push_back('"');
          ++i;
        } else {
          quoted = false;
        }
      } else {
        cur.push_back(c);
      }
    } else if (c == '"') {
      quoted = true;
    } else if (c == ',') {
      cells.push_back(cur);
      cur.clear();
    } else {
      cur.push_back(c);
    }
  }
  cells.push_back(cur);
  return cells;
}

struct Csv {
  std::vector<std::string> header;
  std::vector<std::vector<std::string>> rows;
  std::map<std::string, size_t> col;   // column name → index
};

// Lines are split on '\n' outside quotes (quoted cells may contain line breaks).
inline bool parseCsv(const std::string& text, Csv* out, std::string* why) {
  std::vector<std::string> lines;
  std::string cur;
  bool quoted = false;
  for (char c : text) {
    if (c == '"') quoted = !quoted;
    if (c == '\n' && !quoted) {
      lines.push_back(cur);
      cur.clear();
    } else {
      cur.push_back(c);
    }
  }
  if (!cur.empty()) lines.push_back(cur);
  if (lines.empty()) {
    if (why) *why = "empty CSV";
    return false;
  }
  out->header = splitCsvLine(lines[0]);
  out->col.clear();
  for (size_t i = 0; i < out->header.size(); ++i) out->col[out->header[i]] = i;
  out->rows.clear();
  for (size_t i = 1; i < lines.size(); ++i) {
    out->rows.push_back(splitCsvLine(lines[i]));
    if (out->rows.back().size() != out->header.size()) {
      if (why) *why = "row " + std::to_string(i) + " has " + std::to_string(out->rows.back().size()) + " cells, header " +
                      std::to_string(out->header.size());
      return false;
    }
  }
  return true;
}

// ------------------------------------------------------------------------------------------- golden writer
inline const MsgSchema* nestedOf(const MsgSchema& root, const std::string& name) {
  const auto it = root.nested.find(name);
  return (it == root.nested.end() || !it->second) ? nullptr : it->second.get();
}

// Shape of one message on the wire: per field (schema order) the builtin-array length, or the element
// shapes of a message field (one element for a scalar message field).
struct Shape;
struct FieldShape {
  size_t len = 0;
  std::vector<Shape> elems;
};
struct Shape {
  std::vector<FieldShape> f;
};

inline Shape shapeOf(const MsgSchema& root, const MsgSchema& s, const FieldNode& n) {
  Shape sh;
  sh.f.resize(s.fields.size());
  for (size_t i = 0; i < s.fields.size(); ++i) {
    const FieldSpec& fs = s.fields[i];
    const FieldNode* c = i < n.children.size() ? &n.children[i] : nullptr;
    if (!c) continue;
    if (fs.builtin != BuiltinType::NotBuiltin) {
      if (fs.is_array) sh.f[i].len = c->array.size();
    } else {
      const MsgSchema* sub = nestedOf(root, fs.resolved_type);
      if (!sub) continue;
      if (fs.is_array) {
        for (const FieldNode& e : c->children) sh.f[i].elems.push_back(shapeOf(root, *sub, e));
      } else {
        sh.f[i].elems.push_back(shapeOf(root, *sub, *c));
      }
    }
  }
  return sh;
}

inline void mergeShape(const MsgSchema& root, const MsgSchema& s, Shape* a, const Shape& b) {
  for (size_t i = 0; i < s.fields.size(); ++i) {
    const FieldSpec& fs = s.fields[i];
    FieldShape& fa = a->f[i];
    const FieldShape& fb = b.f[i];
    if (fs.builtin != BuiltinType::NotBuiltin) {
      fa.len = std::max(fa.len, fb.len);
      continue;
    }
    const MsgSchema* sub = nestedOf(root, fs.resolved_type);
    for (size_t k = 0; k < fb.elems.size(); ++k) {
      if (k < fa.elems.size()) {
        if (sub) mergeShape(root, *sub, &fa.elems[k], fb.elems[k]);
      } else {
        fa.elems.push_back(fb.elems[k]);
      }
    }
  }
}

inline bool isTimeLike(BuiltinType t) { return t == BuiltinType::Time || t == BuiltinType::Duration; }

inline void columns(const MsgSchema& root, const MsgSchema& s, const Shape& sh, const std::string& prefix, std::vector<std::string>* out) {
  for (size_t i = 0; i < s.fields.size(); ++i) {
    const FieldSpec& fs = s.fields[i];
    const std::string name = prefix + fs.name;
    auto leaf = [&](const std::string& n) {
      if (isTimeLike(fs.builtin)) {
        out->push_back(n + ".sec");
        out->push_back(n + ".nsec");
      } else {
        out->push_back(n);
      }
    };
    if (fs.builtin != BuiltinType::NotBuiltin) {
      if (fs.is_array) {
        for (size_t k = 0; k < sh.f[i].len; ++k) leaf(name + "_" + std::to_string(k));
      } else {
        leaf(name);
      }
      continue;
    }
    const MsgSchema* sub = nestedOf(root, fs.resolved_type);
    if (!sub) continue;
    if (fs.is_array) {
      for (size_t k = 0; k < sh.f[i].elems.size(); ++k) columns(root, *sub, sh.f[i].elems[k], name + "_" + std::to_string(k) + ".", out);
    } else if (!sh.f[i].elems.empty()) {
      columns(root, *sub, sh.f[i].elems[0], name + ".", out);
    }
  }
}

inline std::string fmtFloat(double x) {
  if (std::isnan(x)) return "nan";   // Python prints every NaN as 'nan' (glibc would print '-nan' for a signed one)
  char b[64];
  std::snprintf(b, sizeof(b), "%.17g", x);
  return b;
}

inline std::string fmtString(const std::string& s) {
  if (s.find_first_of(",\"\r\n") == std::string::npos) return s;
  std::string q = "\"";
  for (char c : s) {
    if (c == '"') q += "\"\"";
    else q.push_back(c);
  }
  q += "\"";
  return q;
}

inline void leafCells(BuiltinType t, const Value* v, std::vector<std::string>* out) {
  if (isTimeLike(t)) {
    if (!v) {
      out->push_back("");
      out->push_back("");
    } else if (t == BuiltinType::Time) {
      out->push_back(std::to_string(v->sec));
      out->push_back(std::to_string(v->nsec));
    } else {
      out->push_back(std::to_string(static_cast<int32_t>(v->sec)));
      out->push_back(std::to_string(static_cast<int32_t>(v->nsec)));
    }
    return;
  }
  if (!v) {
    out->push_back("");
    return;
  }
  switch (t) {
    case BuiltinType::Float32:
    case BuiltinType::Float64: out->push_back(fmtFloat(v->f)); break;
    case BuiltinType::Int8:
    case BuiltinType::Int16:
    case BuiltinType::Int32:
    case BuiltinType::Int64: out->push_back(std::to_string(v->i)); break;
    case BuiltinType::UInt8:
    case BuiltinType::UInt16:
    case BuiltinType::UInt32:
    case BuiltinType::UInt64: out->push_back(std::to_string(v->u)); break;
    case BuiltinType::Boolean: out->push_back(v->b ? "1" : "0"); break;
    case BuiltinType::String: out->push_back(fmtString(v->s)); break;
    default: out->push_back("?");
  }
}

inline void cells(const MsgSchema& root, const MsgSchema& s, const Shape& sh, const FieldNode* n, std::vector<std::string>* out) {
  for (size_t i = 0; i < s.fields.size(); ++i) {
    const FieldSpec& fs = s.fields[i];
    const FieldNode* c = (n && i < n->children.size()) ? &n->children[i] : nullptr;
    if (fs.builtin != BuiltinType::NotBuiltin) {
      if (fs.is_array) {
        for (size_t k = 0; k < sh.f[i].len; ++k) leafCells(fs.builtin, (c && k < c->array.size()) ? &c->array[k] : nullptr, out);
      } else {
        leafCells(fs.builtin, c ? &c->scalar : nullptr, out);
      }
      continue;
    }
    const MsgSchema* sub = nestedOf(root, fs.resolved_type);
    if (!sub) continue;
    if (fs.is_array) {
      for (size_t k = 0; k < sh.f[i].elems.size(); ++k)
        cells(root, *sub, sh.f[i].elems[k], (c && k < c->children.size()) ? &c->children[k] : nullptr, out);
    } else if (!sh.f[i].elems.empty()) {
      cells(root, *sub, sh.f[i].elems[0], c, out);
    }
  }
}

inline Shape emptyShape(const MsgSchema& root, const MsgSchema& s) {
  Shape sh;
  sh.f.resize(s.fields.size());
  for (size_t i = 0; i < s.fields.size(); ++i) {
    const FieldSpec& fs = s.fields[i];
    if (fs.builtin != BuiltinType::NotBuiltin) {
      if (fs.is_array && fs.fixed_len >= 0) sh.f[i].len = static_cast<size_t>(fs.fixed_len);
    } else if (!fs.is_array) {
      const MsgSchema* sub = nestedOf(root, fs.resolved_type);
      if (sub) sh.f[i].elems.push_back(emptyShape(root, *sub));
    }
  }
  return sh;
}

// Golden CSV of decoded messages (record order) — must equal the exporter's file byte for byte (T3).
inline std::string goldenCsv(const MsgSchema& root, const std::vector<std::pair<uint64_t, FieldNode>>& msgs) {
  Shape sh;
  bool first = true;
  for (const auto& m : msgs) {
    const Shape s = shapeOf(root, root, m.second);
    if (first) {
      sh = s;
      first = false;
    } else {
      mergeShape(root, root, &sh, s);
    }
  }
  if (first) sh = emptyShape(root, root);
  std::vector<std::string> head{"t_record_ns"};
  columns(root, root, sh, "", &head);
  std::string out;
  for (size_t i = 0; i < head.size(); ++i) out += (i ? "," : "") + head[i];
  out += "\n";
  for (const auto& m : msgs) {
    std::vector<std::string> row{std::to_string(m.first)};
    cells(root, root, sh, &m.second, &row);
    for (size_t i = 0; i < row.size(); ++i) out += (i ? "," : "") + row[i];
    out += "\n";
  }
  return out;
}

// ------------------------------------------------------------------------------------------- CSV → tree
inline bool parseCell(BuiltinType t, const std::string& cell, Value* v, std::string* why) {
  v->type = t;
  const char* s = cell.c_str();
  char* end = nullptr;
  errno = 0;
  switch (t) {
    case BuiltinType::Float32:
    case BuiltinType::Float64:
      if (cell == "nan") {
        v->f = std::nan("");
        return true;
      }
      v->f = std::strtod(s, &end);
      break;
    case BuiltinType::Int8:
    case BuiltinType::Int16:
    case BuiltinType::Int32:
    case BuiltinType::Int64: v->i = std::strtoll(s, &end, 10); break;
    case BuiltinType::UInt8:
    case BuiltinType::UInt16:
    case BuiltinType::UInt32:
    case BuiltinType::UInt64: v->u = std::strtoull(s, &end, 10); break;
    case BuiltinType::Boolean:
      v->b = cell == "1";
      if (cell != "0" && cell != "1") {
        if (why) *why = "bad bool '" + cell + "'";
        return false;
      }
      return true;
    case BuiltinType::String: v->s = cell; return true;
    default:
      if (why) *why = "unsupported cell type";
      return false;
  }
  if (cell.empty() || !end || *end != '\0' || errno == ERANGE) {
    if (why) *why = "unparsable cell '" + cell + "'";
    return false;
  }
  return true;
}

struct RowReader {
  const MsgSchema& root;
  const Csv& csv;
  const std::vector<std::string>& row;
  std::string* why;

  const std::string* cell(const std::string& col) const {
    const auto it = csv.col.find(col);
    return it == csv.col.end() ? nullptr : &row[it->second];
  }
  bool leafPresent(BuiltinType t, const std::string& n) const {
    const std::string* c = cell(isTimeLike(t) ? n + ".sec" : n);
    return c && !c->empty();
  }
  bool readLeaf(BuiltinType t, const std::string& n, Value* v) {
    if (isTimeLike(t)) {
      Value a, b;
      if (!parseCell(BuiltinType::Int64, *cell(n + ".sec"), &a, why) || !parseCell(BuiltinType::Int64, *cell(n + ".nsec"), &b, why)) return false;
      v->type = t;
      v->sec = static_cast<uint32_t>(a.i);
      v->nsec = static_cast<uint32_t>(b.i);
      return true;
    }
    return parseCell(t, *cell(n), v, why);
  }
  // Any non-empty leaf cell below `prefix` (element presence of message arrays).
  bool anyBelow(const std::string& prefix) const {
    for (const auto& kv : csv.col)
      if (kv.first.compare(0, prefix.size(), prefix) == 0 && !row[kv.second].empty()) return true;
    return false;
  }
  bool message(const MsgSchema& s, const std::string& prefix, FieldNode* node) {
    for (const FieldSpec& fs : s.fields) {
      FieldNode c;
      c.name = fs.name;
      c.type = fs.type;
      c.is_array = fs.is_array;
      const std::string n = prefix + fs.name;
      if (fs.builtin != BuiltinType::NotBuiltin) {
        if (fs.is_array) {
          for (size_t k = 0; leafPresent(fs.builtin, n + "_" + std::to_string(k)); ++k) {
            Value v;
            if (!readLeaf(fs.builtin, n + "_" + std::to_string(k), &v)) return false;
            c.array.push_back(v);
          }
        } else {
          if (!cell(isTimeLike(fs.builtin) ? n + ".sec" : n)) {
            if (why) *why = "column '" + n + "' missing";
            return false;
          }
          if (!readLeaf(fs.builtin, n, &c.scalar)) return false;
        }
      } else {
        const MsgSchema* sub = nestedOf(root, fs.resolved_type);
        if (!sub) {
          if (why) *why = "unresolved " + fs.resolved_type;
          return false;
        }
        c.is_message = true;
        if (fs.is_array) {
          for (size_t k = 0; anyBelow(n + "_" + std::to_string(k) + "."); ++k) {
            FieldNode e;
            e.name = std::to_string(k);
            e.type = fs.type;
            e.is_message = true;
            if (!message(*sub, n + "_" + std::to_string(k) + ".", &e)) return false;
            c.children.push_back(std::move(e));
          }
        } else if (!message(*sub, n + ".", &c)) {
          return false;
        }
      }
      node->children.push_back(std::move(c));
    }
    return true;
  }
};

inline bool rowToTree(const MsgSchema& root, const Csv& csv, size_t row_index, FieldNode* out, uint64_t* t_ns, std::string* why) {
  const std::vector<std::string>& row = csv.rows.at(row_index);
  if (t_ns) *t_ns = std::strtoull(row.at(0).c_str(), nullptr, 10);
  FieldNode n;
  n.type = root.datatype;
  n.is_message = true;
  RowReader r{root, csv, row, why};
  if (!r.message(root, "", &n)) return false;
  *out = std::move(n);
  return true;
}

// ------------------------------------------------------------------------------------------- tree compare
inline bool sameValue(const Value& a, const Value& b) {
  if (a.type != b.type) return false;
  switch (a.type) {
    case BuiltinType::Float32:
    case BuiltinType::Float64:
      if (std::isnan(a.f) || std::isnan(b.f)) return std::isnan(a.f) && std::isnan(b.f);
      return std::memcmp(&a.f, &b.f, sizeof(double)) == 0;   // bit-exact (−0 ≠ +0)
    case BuiltinType::Int8:
    case BuiltinType::Int16:
    case BuiltinType::Int32:
    case BuiltinType::Int64: return a.i == b.i;
    case BuiltinType::UInt8:
    case BuiltinType::UInt16:
    case BuiltinType::UInt32:
    case BuiltinType::UInt64: return a.u == b.u;
    case BuiltinType::Boolean: return a.b == b.b;
    case BuiltinType::String: return a.s == b.s;
    case BuiltinType::Time:
    case BuiltinType::Duration: return a.sec == b.sec && a.nsec == b.nsec;
    default: return false;
  }
}

// Exact structural comparison; `diff` names the first difference as a dotted path.
inline bool compareTrees(const FieldNode& a, const FieldNode& b, const std::string& path, std::string* diff) {
  auto fail = [&](const std::string& m) {
    if (diff) *diff = path + ": " + m;
    return false;
  };
  if (a.is_array != b.is_array || a.is_message != b.is_message) return fail("kind differs");
  if (a.is_message) {
    if (a.children.size() != b.children.size())
      return fail("child count " + std::to_string(a.children.size()) + " vs " + std::to_string(b.children.size()));
    for (size_t i = 0; i < a.children.size(); ++i) {
      if (a.children[i].name != b.children[i].name) return fail("child name " + a.children[i].name + " vs " + b.children[i].name);
      if (!compareTrees(a.children[i], b.children[i], path + "." + a.children[i].name, diff)) return false;
    }
    return true;
  }
  if (a.is_array) {
    if (a.array.size() != b.array.size()) return fail("length " + std::to_string(a.array.size()) + " vs " + std::to_string(b.array.size()));
    for (size_t i = 0; i < a.array.size(); ++i)
      if (!sameValue(a.array[i], b.array[i])) return fail("element " + std::to_string(i) + " differs");
    return true;
  }
  return sameValue(a.scalar, b.scalar) ? true : fail("value differs");
}

}  // namespace testsupport
}  // namespace livo_gnss_io
