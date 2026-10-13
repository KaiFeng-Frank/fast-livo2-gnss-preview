// SPDX-License-Identifier: BSD-3-Clause
// test_u02_wire.cpp — U2 (TEST_PLAN §1): ROS1 message_definition parsing / md5 and the wire decoder.
//   * hand-written definition + independently serialised bytes → every value back, md5 against genmsg;
//   * fixture bytes decoded field-for-field equal to the rosbags-decoded golden CSV (exact bits);
//   * malformed buffers and definitions: false + reason, never a crash / huge allocation;
//   * the expectation table rejects a changed (type, name) layout and accepts comment/blank-line edits.
// md5 oracles below were produced by genmsg itself (ROS Noetic, gentools.compute_md5) and Python hashlib,
// i.e. independently of gnss_io/src/ros1_msgdef.cpp.
#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <limits>
#include <random>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

#include "golden_csv.hpp"
#include "livo_gnss_io/gnss_comm_mapping.hpp"
#include "livo_gnss_io/ros1_decoder.hpp"
#include "livo_gnss_io/ros1_msgdef.hpp"

using namespace livo_gnss;
using namespace livo_gnss_io;
namespace ts = livo_gnss_io::testsupport;

namespace {

const std::string kSep = std::string(80, '=') + "\n";

const char* kHeaderDef =
    "# Standard metadata for higher-level stamped data types.\n"
    "uint32 seq\n"
    "time stamp\n"
    "string frame_id\n";

const char* kInnerDef =
    "# hand-written for U2\n"
    "uint32 id      # identifier\n"
    "float32[] w\n"
    "time stamp\n";

const char* kAllTypesDef =
    "Header header\n"
    "bool b\n"
    "int8 i8\n"
    "uint8 u8\n"
    "int16 i16\n"
    "uint16 u16\n"
    "int32 i32\n"
    "uint32 u32\n"
    "int64 i64\n"
    "uint64 u64\n"
    "float32 f32\n"
    "float64 f64\n"
    "string s\n"
    "time t\n"
    "duration d\n"
    "byte by\n"
    "char ch\n"
    "float64[] vf\n"
    "float64[3] ff\n"
    "int32[] vi\n"
    "string[] vs\n"
    "Inner in1\n"
    "Inner[] vin\n"
    "Inner[2] fin\n";

std::string allTypesMsgDef(const std::string& main = kAllTypesDef) {
  return main + kSep + "MSG: std_msgs/Header\n" + kHeaderDef + kSep + "MSG: u2_test/Inner\n" + kInnerDef;
}

// Independent little-endian ROS1 serialiser for the hand-written type.
struct Wire {
  std::vector<uint8_t> b;
  void raw(const void* p, size_t n) {
    const uint8_t* c = static_cast<const uint8_t*>(p);
    for (size_t i = 0; i < n; ++i) b.push_back(c[i]);   // host is little-endian (x86-64/aarch64 LE)
  }
  void u8(uint8_t v) { raw(&v, 1); }
  void i8(int8_t v) { raw(&v, 1); }
  void u16(uint16_t v) { raw(&v, 2); }
  void i16(int16_t v) { raw(&v, 2); }
  void u32(uint32_t v) { raw(&v, 4); }
  void i32(int32_t v) { raw(&v, 4); }
  void u64(uint64_t v) { raw(&v, 8); }
  void i64(int64_t v) { raw(&v, 8); }
  void f32(float v) { raw(&v, 4); }
  void f64(double v) { raw(&v, 8); }
  void str(const std::string& s) {
    u32(static_cast<uint32_t>(s.size()));
    raw(s.data(), s.size());
  }
  void inner(uint32_t id, const std::vector<float>& w, uint32_t sec, uint32_t nsec) {
    u32(id);
    u32(static_cast<uint32_t>(w.size()));
    for (float x : w) f32(x);
    u32(sec);
    u32(nsec);
  }
};

bool bitEqual(double a, double b) { return std::memcmp(&a, &b, sizeof(double)) == 0; }

const FieldNode& at(const FieldNode& n, const char* name) {
  const FieldNode* c = n.child(name);
  if (!c) throw std::runtime_error(std::string("missing child ") + name);
  return *c;
}

std::string fixtureRoot() { return LIVO_GNSS_TEST_DATA_DIR; }

struct NullSink : MeasurementSink {
  int obs = 0, eph = 0, geph = 0, ion = 0, pvt = 0;
  void pushObservation(ObsEpoch&&) override { ++obs; }
  void pushEphemeris(const KeplerEph&) override { ++eph; }
  void pushGloEphemeris(const GloEph&) override { ++geph; }
  void pushIono(const Klobuchar&) override { ++ion; }
  void pushPvt(const ReceiverPvt&) override { ++pvt; }
};

Source sourceOfTopic(const std::string& t) {
  if (t == "range_meas") return Source::GnssCommMeas;
  if (t == "ephem") return Source::GnssCommEphem;
  if (t == "glo_ephem") return Source::GnssCommGloEphem;
  if (t == "iono_params") return Source::GnssCommIono;
  if (t == "receiver_pvt") return Source::GnssCommPvt;
  return Source::GnssCommTimePulse;
}

// Decodes every record of every topic of `fixture` and compares it with the golden CSV row (exact bits).
void checkFixtureAgainstGolden(const std::string& fixture, int* n_topics, size_t* n_records) {
  *n_topics = 0;
  *n_records = 0;
  for (const auto& tc : ts::topicCsv()) {
    const std::string& topic = tc.first;
    if (!ts::isFile(fixture + "/msgdef/" + topic + ".msgdef")) continue;
    SCOPED_TRACE(fixture + " " + topic);
    ts::TopicSchema sch;
    std::string why;
    ASSERT_TRUE(ts::loadTopic(fixture, topic, &sch, &why)) << why;
    std::vector<ts::Record> recs;
    ASSERT_TRUE(ts::readRecords(fixture + "/bytes/" + topic + ".bin", &recs, &why)) << why;
    std::string text;
    ASSERT_TRUE(ts::readFile(fixture + "/golden/" + tc.second + ".csv", &text));
    ts::Csv csv;
    ASSERT_TRUE(ts::parseCsv(text, &csv, &why)) << why;
    ASSERT_EQ(csv.rows.size(), recs.size()) << "golden rows vs byte records";
    const Ros1Decoder dec(sch.schema);
    for (size_t i = 0; i < recs.size(); ++i) {
      FieldNode from_bytes, from_csv;
      size_t used = 0;
      ASSERT_TRUE(dec.decode(recs[i].data.data(), recs[i].data.size(), &from_bytes, &used, &why)) << "record " << i << ": " << why;
      EXPECT_EQ(used, recs[i].data.size());
      uint64_t t_ns = 0;
      ASSERT_TRUE(ts::rowToTree(sch.schema, csv, i, &from_csv, &t_ns, &why)) << "row " << i << ": " << why;
      EXPECT_EQ(t_ns, recs[i].t_ns) << "record " << i;
      std::string diff;
      EXPECT_TRUE(ts::compareTrees(from_bytes, from_csv, topic, &diff)) << "record " << i << ": " << diff;
    }
    ++*n_topics;
    *n_records += recs.size();
  }
}

}  // namespace

// ---------------------------------------------------------------------------------------------------------
// hand-written msg_def + synthetic bytes round trip through the decoder
TEST(U02Wire, HandwrittenMsgDefSyntheticBytesRoundTrip) {
  MsgSchema s;
  std::string why;
  ASSERT_TRUE(MsgDefParser::parse("u2_test/AllTypes", allTypesMsgDef(), &s, &why)) << why;
  ASSERT_EQ(s.fields.size(), 24u);
  EXPECT_EQ(s.datatype, "u2_test/AllTypes");
  EXPECT_EQ(s.fields[0].resolved_type, "std_msgs/Header");
  EXPECT_EQ(s.fields[15].type, "byte");
  EXPECT_EQ(s.fields[15].builtin, BuiltinType::Int8);
  EXPECT_EQ(s.fields[16].builtin, BuiltinType::UInt8);
  EXPECT_TRUE(s.fields[18].is_array);
  EXPECT_EQ(s.fields[18].fixed_len, 3);
  EXPECT_EQ(s.fields[19].fixed_len, -1);
  EXPECT_EQ(s.fields[22].resolved_type, "u2_test/Inner");
  ASSERT_EQ(s.nested.count("u2_test/Inner"), 1u);
  ASSERT_EQ(s.nested.count("std_msgs/Header"), 1u);

  // md5 against genmsg (the ROS1 generator), for the root and both dependencies.
  EXPECT_EQ(MsgDefParser::md5(s), "6fd518f818fa429bf3b92ca0861af607");
  EXPECT_EQ(MsgDefParser::md5(*s.nested.at("u2_test/Inner")), "6182a983d89d2010bb40960acd0f55ec");
  EXPECT_EQ(MsgDefParser::md5(*s.nested.at("std_msgs/Header")), "2176decaecbce78abc3b96ef049fabed");

  // flattened layout (canonical builtin names, resolved message types, array markers)
  const std::vector<std::pair<std::string, std::string>> exp = {
      {"std_msgs/Header", "header"}, {"uint32", "header.seq"}, {"time", "header.stamp"}, {"string", "header.frame_id"},
      {"bool", "b"}, {"int8", "i8"}, {"uint8", "u8"}, {"int16", "i16"}, {"uint16", "u16"}, {"int32", "i32"}, {"uint32", "u32"},
      {"int64", "i64"}, {"uint64", "u64"}, {"float32", "f32"}, {"float64", "f64"}, {"string", "s"}, {"time", "t"},
      {"duration", "d"}, {"int8", "by"}, {"uint8", "ch"}, {"float64[]", "vf"}, {"float64[3]", "ff"}, {"int32[]", "vi"},
      {"string[]", "vs"}, {"u2_test/Inner", "in1"}, {"uint32", "in1.id"}, {"float32[]", "in1.w"}, {"time", "in1.stamp"},
      {"u2_test/Inner[]", "vin"}, {"uint32", "vin.id"}, {"float32[]", "vin.w"}, {"time", "vin.stamp"},
      {"u2_test/Inner[2]", "fin"}, {"uint32", "fin.id"}, {"float32[]", "fin.w"}, {"time", "fin.stamp"}};
  EXPECT_EQ(s.flatten(), exp);

  // independently serialised message with boundary values
  const double nan = std::numeric_limits<double>::quiet_NaN(), inf = std::numeric_limits<double>::infinity();
  Wire w;
  w.u32(7);
  w.u32(1735883982);
  w.u32(999999999);
  w.str("frame,with \"quotes\"");
  w.u8(2);   // bool on the wire is a byte; any non-zero is true
  w.i8(-128);
  w.u8(255);
  w.i16(-32768);
  w.u16(65535);
  w.i32(std::numeric_limits<int32_t>::min());
  w.u32(std::numeric_limits<uint32_t>::max());
  w.i64(std::numeric_limits<int64_t>::min());
  w.u64(std::numeric_limits<uint64_t>::max());
  w.f32(-0.0f);
  w.f64(nan);
  w.str("");
  w.u32(1419919200);
  w.u32(123456789);
  w.i32(-5);
  w.i32(-250000000);
  w.i8(-1);
  w.u8(200);
  w.u32(3);
  w.f64(inf);
  w.f64(-inf);
  w.f64(1.0e-310);   // subnormal
  w.f64(1.5);
  w.f64(2.5);
  w.f64(-3.5);
  w.u32(0);          // vi empty
  w.u32(2);
  w.str("α,β");
  w.str("x");
  w.inner(11, {1.25f, -2.5f}, 10, 20);
  w.u32(2);
  w.inner(21, {}, 30, 40);
  w.inner(22, {3.0f}, 50, 60);
  w.inner(31, {4.0f, 5.0f, 6.0f}, 70, 80);
  w.inner(32, {}, 90, 100);

  const Ros1Decoder dec(s);
  FieldNode n;
  size_t used = 0;
  ASSERT_TRUE(dec.decode(w.b.data(), w.b.size(), &n, &used, &why)) << why;
  EXPECT_EQ(used, w.b.size());
  ASSERT_TRUE(n.is_message);
  ASSERT_EQ(n.children.size(), 24u);
  const FieldNode& h = at(n, "header");
  EXPECT_EQ(at(h, "seq").scalar.u, 7u);
  EXPECT_EQ(at(h, "stamp").scalar.sec, 1735883982u);
  EXPECT_EQ(at(h, "stamp").scalar.nsec, 999999999u);
  EXPECT_DOUBLE_EQ(at(h, "stamp").scalar.toDouble(), 1735883982.0 + 0.999999999);
  EXPECT_EQ(at(h, "frame_id").scalar.s, "frame,with \"quotes\"");
  EXPECT_TRUE(at(n, "b").scalar.b);
  EXPECT_EQ(at(n, "i8").scalar.i, -128);
  EXPECT_EQ(at(n, "u8").scalar.u, 255u);
  EXPECT_EQ(at(n, "i16").scalar.i, -32768);
  EXPECT_EQ(at(n, "u16").scalar.u, 65535u);
  EXPECT_EQ(at(n, "i32").scalar.i, std::numeric_limits<int32_t>::min());
  EXPECT_EQ(at(n, "u32").scalar.u, std::numeric_limits<uint32_t>::max());
  EXPECT_EQ(at(n, "i64").scalar.i, std::numeric_limits<int64_t>::min());
  EXPECT_EQ(at(n, "u64").scalar.u, std::numeric_limits<uint64_t>::max());
  EXPECT_TRUE(bitEqual(at(n, "f32").scalar.f, -0.0));
  EXPECT_EQ(at(n, "f32").scalar.type, BuiltinType::Float32);
  EXPECT_TRUE(std::isnan(at(n, "f64").scalar.f));
  EXPECT_EQ(at(n, "s").scalar.s, "");
  EXPECT_EQ(at(n, "t").scalar.sec, 1419919200u);
  EXPECT_EQ(at(n, "t").scalar.nsec, 123456789u);
  EXPECT_DOUBLE_EQ(at(n, "d").scalar.toDouble(), -5.0 - 0.25);   // duration fields are signed
  EXPECT_EQ(at(n, "by").scalar.i, -1);
  EXPECT_EQ(at(n, "ch").scalar.u, 200u);
  const FieldNode& vf = at(n, "vf");
  ASSERT_EQ(vf.array.size(), 3u);
  EXPECT_TRUE(bitEqual(vf.array[0].f, inf));
  EXPECT_TRUE(bitEqual(vf.array[1].f, -inf));
  EXPECT_TRUE(bitEqual(vf.array[2].f, 1.0e-310));
  const FieldNode& ff = at(n, "ff");
  ASSERT_EQ(ff.array.size(), 3u);   // fixed length: no count on the wire
  EXPECT_EQ(ff.array[2].f, -3.5);
  EXPECT_TRUE(at(n, "vi").array.empty());
  ASSERT_EQ(at(n, "vs").array.size(), 2u);
  EXPECT_EQ(at(n, "vs").array[0].s, "α,β");
  const FieldNode& in1 = at(n, "in1");
  EXPECT_EQ(at(in1, "id").scalar.u, 11u);
  ASSERT_EQ(at(in1, "w").array.size(), 2u);
  EXPECT_EQ(at(in1, "w").array[1].f, -2.5);
  EXPECT_EQ(at(in1, "stamp").scalar.nsec, 20u);
  const FieldNode& vin = at(n, "vin");
  ASSERT_TRUE(vin.is_array && vin.is_message);
  ASSERT_EQ(vin.children.size(), 2u);
  EXPECT_EQ(at(*vin.child("1"), "id").scalar.u, 22u);
  EXPECT_TRUE(at(*vin.child("0"), "w").array.empty());
  const FieldNode& fin = at(n, "fin");
  ASSERT_EQ(fin.children.size(), 2u);
  EXPECT_EQ(at(fin.children[0], "w").array.size(), 3u);
  EXPECT_EQ(at(fin.children[1], "stamp").scalar.nsec, 100u);
  EXPECT_EQ(n.child("nope"), nullptr);
  EXPECT_TRUE(std::isnan(at(n, "s").scalar.toDouble()));   // strings have no numeric view

  // A nested schema is self-contained (carries its own dependency closure).
  const MsgSchema& inner = *s.nested.at("u2_test/Inner");
  Wire wi;
  wi.inner(5, {7.0f}, 1, 2);
  FieldNode ni;
  ASSERT_TRUE(Ros1Decoder(inner).decode(wi.b.data(), wi.b.size(), &ni, &used, &why)) << why;
  EXPECT_EQ(at(ni, "w").array[0].f, 7.0);

  // Comments, blank lines, tabs, CRLF and a trailing separator change neither layout nor md5.
  std::string noisy = allTypesMsgDef("# leading comment\n\nHeader\theader   # trailing\r\n" + std::string(kAllTypesDef).substr(14)) + kSep;
  MsgSchema s2;
  ASSERT_TRUE(MsgDefParser::parse("u2_test/msg/AllTypes", noisy, &s2, &why)) << why;   // ROS2-style name normalised
  EXPECT_EQ(s2.flatten(), s.flatten());
  EXPECT_EQ(MsgDefParser::md5(s2), MsgDefParser::md5(s));

  // Constant lines are not part of the wire layout.
  MsgSchema sc;
  const std::string with_const =
      std::string(kAllTypesDef) + "uint8 MODE_A=1\nstring LABEL=a,b # not a comment\nint8 NEG = -3\n";
  ASSERT_TRUE(MsgDefParser::parse("u2_test/WithConst", allTypesMsgDef(with_const), &sc, &why)) << why;
  EXPECT_EQ(sc.flatten(), s.flatten());
  FieldNode nc;
  ASSERT_TRUE(Ros1Decoder(sc).decode(w.b.data(), w.b.size(), &nc, &used, &why)) << why;
  std::string diff;
  EXPECT_TRUE(ts::compareTrees(n, nc, "root", &diff)) << diff;
}

// MD5 primitive across the RFC-1321 padding boundaries (single-field messages; text length L = 5 + name).
TEST(U02Wire, Md5PaddingBoundaries) {
  const std::vector<std::pair<int, const char*>> vectors = {
      {55, "79d2a0dce3d6180a99576cb1343a4f7d"}, {56, "b34c9851441f40daf473e86d7b229d2c"}, {63, "663e6d2f2f6a237da788f9a28668d2f2"},
      {64, "0d020f0b57bd0470270ce0914f14bc4b"}, {65, "e90863dbb7fc1bd18522acc1ffd8d6c8"}, {119, "84d7125c7962bdf46ff5ca45f441c139"},
      {120, "4df667ad03390700020aacf4fb70dd98"}, {128, "2f844f737543f4615a5880b4a4d4e687"}};
  for (const auto& v : vectors) {
    MsgSchema s;
    std::string why;
    ASSERT_TRUE(MsgDefParser::parse("u2_test/Pad", "int8 " + std::string(static_cast<size_t>(v.first - 5), 'a') + "\n", &s, &why)) << why;
    EXPECT_EQ(MsgDefParser::md5(s), v.second) << "text length " << v.first;
  }
  MsgSchema empty;
  std::string why;
  ASSERT_TRUE(MsgDefParser::parse("u2_test/Empty", "# no fields\n", &empty, &why)) << why;
  EXPECT_EQ(MsgDefParser::md5(empty), "d41d8cd98f00b204e9800998ecf8427e");
  FieldNode n;
  size_t used = 1;
  EXPECT_TRUE(Ros1Decoder(empty).decode(nullptr, 0, &n, &used, &why)) << why;   // zero-size message
  EXPECT_EQ(used, 0u);
}

// md5 of definitions with constants (MsgSchema::constants, WP-e #1 / A39): genmsg hashes "T NAME=VALUE" lines first, in
// definition order, in every type of the closure. Oracles: genmsg itself (ROS Noetic msg_loader + gentools, run on these
// exact texts; constants lists as genmsg's MsgSpec.constants spell them) and the md5sum of the M3DGR connection header
// /ublox_driver/receiver_lla (sensor_msgs/NavSatFix, identical in Grass01/Grass02/Occlusion03), whose closure has
// constants in the root and in a nested type.
namespace {
// /ublox_driver/receiver_lla message_definition without comments and blank lines (tools/gnss_bag_export.py strip_msgdef;
// constant lines keep their spacing, which genmsg normalises).
const char* kNavSatFixDef =
    "Header header\n"
    "NavSatStatus status\n"
    "float64 latitude\n"
    "float64 longitude\n"
    "float64 altitude\n"
    "float64[9] position_covariance\n"
    "uint8 COVARIANCE_TYPE_UNKNOWN = 0\n"
    "uint8 COVARIANCE_TYPE_APPROXIMATED = 1\n"
    "uint8 COVARIANCE_TYPE_DIAGONAL_KNOWN = 2\n"
    "uint8 COVARIANCE_TYPE_KNOWN = 3\n"
    "uint8 position_covariance_type\n"
    "================================================================================\n"
    "MSG: std_msgs/Header\n"
    "uint32 seq\n"
    "time stamp\n"
    "string frame_id\n"
    "================================================================================\n"
    "MSG: sensor_msgs/NavSatStatus\n"
    "int8 STATUS_NO_FIX =  -1\n"
    "int8 STATUS_FIX =      0\n"
    "int8 STATUS_SBAS_FIX = 1\n"
    "int8 STATUS_GBAS_FIX = 2\n"
    "int8 status\n"
    "uint16 SERVICE_GPS =     1\n"
    "uint16 SERVICE_GLONASS = 2\n"
    "uint16 SERVICE_COMPASS = 4\n"
    "uint16 SERVICE_GALILEO = 8\n"
    "uint16 service\n";

// Constants only in a nested type, with genmsg's spelling rules: comments dropped after a numeric value but part of a
// string value, blanks around '=' and around a string value stripped, deprecated byte/char kept as written, the value
// text hashed as written ("+7", "1e-3"), an indented numeric constant, an empty string, the 64-bit extremes.
const char* kConstOuterDef = "ConstInner inner\nint32 x\n";
const char* kConstInnerDef =
    "byte B = -5   # byte keeps its spelling\n"
    "char C=200\n"
    "float64 PI =  3.14159   # comment dropped\n"
    "float32 E=1e-3\n"
    "bool T=True\n"
    "string S =  spaced value  # kept: '#' belongs to a string constant\n"
    "string EMPTY=\n"
    "uint64 BIG=18446744073709551615\n"
    "int64 SMALL=-9223372036854775808\n"
    "int32 PLUS=+7\n"
    "  int16 IND = 3\n"
    "uint8 v\n";
}  // namespace

TEST(U02Wire, Md5WithConstants) {
  std::string why;
  {   // the constants of the round-trip test's WithConst variant, in the root type only
    const std::string with_const = std::string(kAllTypesDef) + "uint8 MODE_A=1\nstring LABEL=a,b # not a comment\nint8 NEG = -3\n";
    MsgSchema s;
    ASSERT_TRUE(MsgDefParser::parse("u2_test/WithConst", allTypesMsgDef(with_const), &s, &why)) << why;
    EXPECT_EQ(s.constants, (std::vector<std::string>{"uint8 MODE_A=1", "string LABEL=a,b # not a comment", "int8 NEG=-3"}));
    EXPECT_TRUE(s.nested.at("u2_test/Inner")->constants.empty());
    EXPECT_TRUE(s.nested.at("std_msgs/Header")->constants.empty());
    EXPECT_EQ(MsgDefParser::md5(s), "eefce4efca9bd11dd1dee598bc61535d");
    // Same fields as AllTypes: the digest differs only through the constants, and without them it is AllTypes'.
    MsgSchema bare = s;
    bare.constants.clear();
    EXPECT_EQ(MsgDefParser::md5(bare), "6fd518f818fa429bf3b92ca0861af607");
  }
  {   // sensor_msgs/NavSatFix as recorded in the M3DGR bags: constants in the root and in the nested NavSatStatus
    MsgSchema s;
    ASSERT_TRUE(MsgDefParser::parse("sensor_msgs/msg/NavSatFix", kNavSatFixDef, &s, &why)) << why;
    EXPECT_EQ(s.constants, (std::vector<std::string>{"uint8 COVARIANCE_TYPE_UNKNOWN=0", "uint8 COVARIANCE_TYPE_APPROXIMATED=1",
                                                     "uint8 COVARIANCE_TYPE_DIAGONAL_KNOWN=2", "uint8 COVARIANCE_TYPE_KNOWN=3"}));
    ASSERT_EQ(s.nested.count("sensor_msgs/NavSatStatus"), 1u);
    const MsgSchema& status = *s.nested.at("sensor_msgs/NavSatStatus");
    EXPECT_EQ(status.constants,
              (std::vector<std::string>{"int8 STATUS_NO_FIX=-1", "int8 STATUS_FIX=0", "int8 STATUS_SBAS_FIX=1", "int8 STATUS_GBAS_FIX=2",
                                        "uint16 SERVICE_GPS=1", "uint16 SERVICE_GLONASS=2", "uint16 SERVICE_COMPASS=4", "uint16 SERVICE_GALILEO=8"}));
    EXPECT_EQ(MsgDefParser::md5(s), "2d3a8cd499b9b4a0249fb98fd05cfa48");        // the connection header's md5sum
    EXPECT_EQ(MsgDefParser::md5(status), "331cdbddfa4bc96ffc3b9ad98900a54c");   // genmsg, sensor_msgs/NavSatStatus
    // constants are no part of the wire layout
    const std::vector<std::pair<std::string, std::string>> exp = {
        {"std_msgs/Header", "header"}, {"uint32", "header.seq"},  {"time", "header.stamp"},     {"string", "header.frame_id"},
        {"sensor_msgs/NavSatStatus", "status"}, {"int8", "status.status"}, {"uint16", "status.service"},
        {"float64", "latitude"}, {"float64", "longitude"}, {"float64", "altitude"}, {"float64[9]", "position_covariance"},
        {"uint8", "position_covariance_type"}};
    EXPECT_EQ(s.flatten(), exp);
  }
  {   // constants only in a nested type (the outer digest changes through the nested md5), genmsg spelling rules
    MsgSchema s;
    ASSERT_TRUE(MsgDefParser::parse("u2_test/ConstOuter", std::string(kConstOuterDef) + kSep + "MSG: u2_test/ConstInner\n" + kConstInnerDef, &s,
                                    &why))
        << why;
    EXPECT_TRUE(s.constants.empty());
    const MsgSchema& in = *s.nested.at("u2_test/ConstInner");
    EXPECT_EQ(in.constants, (std::vector<std::string>{"byte B=-5", "char C=200", "float64 PI=3.14159", "float32 E=1e-3", "bool T=True",
                                                      "string S=spaced value  # kept: '#' belongs to a string constant", "string EMPTY=",
                                                      "uint64 BIG=18446744073709551615", "int64 SMALL=-9223372036854775808",
                                                      "int32 PLUS=+7", "int16 IND=3"}));
    EXPECT_EQ(MsgDefParser::md5(in), "84c598e8c22b00bb413feaf2c459c4b4");
    EXPECT_EQ(MsgDefParser::md5(s), "baad3e402a17fd8d7612abc237808849");
    ASSERT_EQ(in.fields.size(), 1u);   // only "uint8 v" is a field
  }
  {   // genmsg quirk kept on purpose: a string constant's name is cut at the line's first *space*, so an indented one
      // is hashed as "string string LEAD=x" (the connection header's md5sum is whatever genmsg made of the .msg text);
      // a leading tab is stripped before that cut and leaves the name alone
    MsgSchema s;
    ASSERT_TRUE(MsgDefParser::parse("u2_test/Lead", "  string LEAD=x\nuint8 v\n", &s, &why)) << why;
    EXPECT_EQ(s.constants, (std::vector<std::string>{"string string LEAD=x"}));
    EXPECT_EQ(MsgDefParser::md5(s), "3ca0f2578b4c740c69ab622d8d2bddc4");
    ASSERT_TRUE(MsgDefParser::parse("u2_test/Probe", "\tstring TABLEAD=y\nuint8 v\n", &s, &why)) << why;
    EXPECT_EQ(s.constants, (std::vector<std::string>{"string TABLEAD=y"}));
    EXPECT_EQ(MsgDefParser::md5(s), "ed2c6a42fb007798dfd16d1e3d53c940");
  }
  // One constant line followed by "uint8 v", as u2_test/Probe: accepted spellings with genmsg's constant text and md5 ...
  const std::vector<std::tuple<std::string, std::string, std::string>> accepted = {
      {"int32 X=1_000", "int32 X=1_000", "38e929f8af34e63807ec82e793b78b27"},
      {"uint8 X=007", "uint8 X=007", "3611a30a4b4a9786cf28bbafaeb7b86d"},
      {"float64 F=inf", "float64 F=inf", "3ca54569caffa603f44ac5a3e8adc0c5"},
      {"float64 F=-Infinity", "float64 F=-Infinity", "a28b813a0ecfd995fae9a70d38d14f2d"},
      {"float64 F=nan", "float64 F=nan", "afb76e2bf64849f0e046c9aba970eb96"},
      {"float64 F=.5", "float64 F=.5", "bf09632e54e553c14490555e9fad63b6"},
      {"float64 F=5.", "float64 F=5.", "6d335fc82ab752b4264f49e66b6d56e6"},
      {"float64 F=1_0.5e1_0", "float64 F=1_0.5e1_0", "61085f59a53b23ee7ea21cb94e6f73ca"},
      {"float32 F=+1E-3", "float32 F=+1E-3", "72a2c2d8bda6a280c0d994aae22ae70c"},
      {"bool B=1", "bool B=1", "e9be4c03356132a6b10370dde12bd6af"},
      {"bool B=False", "bool B=False", "4233416c1a4bc920d5b6b99dcfca4232"},
      {"int8 1bad=1", "int8 1bad=1", "45b71f3c2181bd95b8d7e5f09a739952"},   // genmsg does not check constant names
      {"string S=", "string S=", "5e29d257e444811cf759b58a35150e75"},
      {"int8 \tX=1", "int8 X=1", "4ab75750d325f2f993be892275bcdaf2"},
      {"int8\t X = 1", "int8 X=1", "4ab75750d325f2f993be892275bcdaf2"},
      {"string\t S=a b", "string S=a b", "1362220aa4b47c267a1365850c46043d"},
      {"int64 X=-9223372036854775808", "int64 X=-9223372036854775808", "77802a51e3937ed51d33aa0fd81422cf"},
      {"uint64 X=18446744073709551615", "uint64 X=18446744073709551615", "807f291404e027ce73d9d84016e4d722"},
      // genmsg's 64-bit bounds come from math.pow in double precision: one past the true range is let through
      {"uint64 X=18446744073709551616", "uint64 X=18446744073709551616", "897ea077cd1b73d7d0904b9ee8c6713d"},
      {"int64 X=9223372036854775808", "int64 X=9223372036854775808", "e944e80144ec70cae51fbe836e6e3043"},
      {"int64 X=-9223372036854775809", "int64 X=-9223372036854775809", "e09f558552f4c11d076383cbe6e1e3fb"},
      {"uint32 X=4294967295", "uint32 X=4294967295", "37ce994147eeed9d192fe2d9e2136e36"},
      {"uint8 X=-0", "uint8 X=-0", "31960b49f36ac0d6ed42e61b0f5fcd59"},
      {"int8 X=+0", "int8 X=+0", "1f4739cd77f9d20fb87c7905fdb849ed"},
      // bool values are Python literals for genmsg (ast.literal_eval); the value text is hashed as written
      {"bool B='x'", "bool B='x'", "a1eefe062a99ba71ddae75e357f805ea"},
      {"bool B=[1]", "bool B=[1]", "26dc053c83db34c81a0d6efaa1cc98be"},
      {"bool B=None", "bool B=None", "81b50adb94cec4b991ac71a39f7991c2"},
      {"bool B=0x1", "bool B=0x1", "04f09b0e7fa15f7cc6b43807a0f2157b"}};
  for (const auto& a : accepted) {
    SCOPED_TRACE(std::get<0>(a));
    MsgSchema s;
    ASSERT_TRUE(MsgDefParser::parse("u2_test/Probe", std::get<0>(a) + "\nuint8 v\n", &s, &why)) << why;
    EXPECT_EQ(s.constants, std::vector<std::string>{std::get<1>(a)});
    EXPECT_EQ(MsgDefParser::md5(s), std::get<2>(a));
  }
  // ... and the constant lines genmsg refuses (InvalidMsgSpec): illegal constant types, values out of the type's range or
  // not convertible, two '=' — parse() refuses the definition with a reason, as for a malformed field line. (genmsg also
  // refuses a bool value that is no Python literal, e.g. "bool B=foo"; that one is accepted here: see ros1_msgdef.cpp.)
  for (const char* bad : {"time T=0", "duration D=0", "uint8[] A=1", "Header H=1", "uint8 X=256", "int8 X=-129", "uint8 X=-1", "char C=256",
                          "byte B=128", "int8 X=1=2", "uint8 X=abc", "float64 F=abc", "int8 X=", "bool B=", "int32 X=0x10", "int32 X=1.5",
                          "float64 F=1__0", "int8\tX=1", "int32 X=1 2", "uint64 X=18446744073709551617", "int64 X=9223372036854775809",
                          "int64 X=-9223372036854775810", "uint32 X=4294967296", "int32 X=-2147483649"}) {
    SCOPED_TRACE(bad);
    MsgSchema s;
    why.clear();
    EXPECT_FALSE(MsgDefParser::parse("u2_test/Probe", std::string(bad) + "\nuint8 v\n", &s, &why));
    EXPECT_NE(why.find("line 1"), std::string::npos) << why;
  }
  {   // a repeated dependency section must repeat its constants too
    MsgSchema s;
    const std::string twice = std::string("ConstInner a\n") + kSep + "MSG: u2_test/ConstInner\nuint8 K=1\nuint8 v\n" + kSep +
                              "MSG: u2_test/ConstInner\nuint8 K=2\nuint8 v\n";
    EXPECT_FALSE(MsgDefParser::parse("u2_test/Twice", twice, &s, &why));
    EXPECT_NE(why.find("conflicting"), std::string::npos) << why;
  }
}

// fixture bytes decode field-for-field equal to the rosbags Python golden file (synthetic fixture: code and
// exporter agree on hand-made boundary content; says nothing about the data)
TEST(U02Wire, FixtureDecodeEqualsRosbagsGolden) {
  if (ts::syntheticFixtures(fixtureRoot()).empty())
    GTEST_SKIP() << "optional synthetic wire fixtures are not distributed in the research preview";
  int fixtures = 0;
  for (const std::string& fx : ts::syntheticFixtures(fixtureRoot())) {
    int topics = 0;
    size_t records = 0;
    checkFixtureAgainstGolden(fx, &topics, &records);
    EXPECT_GE(topics, 5) << fx;           // range_meas, ephem, glo_ephem, iono_params, receiver_pvt
    EXPECT_GE(records, 200u) << fx;       // not vacuous
    ++fixtures;
  }
  ASSERT_GE(fixtures, 1) << "no Synthetic_* fixture under " << fixtureRoot();
}

// the same check on the development-set fixtures (A24; real receiver output)
TEST(U02Wire, FixtureDecodeEqualsRosbagsGoldenDevSet) {
  int fixtures = 0;
  for (const std::string& fx : ts::realFixtures(fixtureRoot())) {
    int topics = 0;
    size_t records = 0;
    checkFixtureAgainstGolden(fx, &topics, &records);
    EXPECT_GE(topics, 1) << fx;
    EXPECT_GE(records, 100u) << fx;       // not vacuous
    ++fixtures;
  }
  if (fixtures == 0) GTEST_SKIP() << "blocked: dev-set fixture (A24: no gnss/test/data/<seq>_<t0>_<D>s from a development bag)";
}

// malformed buffers return false and never crash
TEST(U02Wire, MalformedBufferReportsNoCrash) {
  const std::string fx = fixtureRoot() + "/Synthetic_1419919200_10s";
  if (!ts::isDir(fx)) GTEST_SKIP() << "optional synthetic wire fixture is not distributed in the research preview: " << fx;
  ts::TopicSchema sch;
  std::string why;
  ASSERT_TRUE(ts::loadTopic(fx, "range_meas", &sch, &why)) << why;
  std::vector<ts::Record> recs;
  ASSERT_TRUE(ts::readRecords(fx + "/bytes/range_meas.bin", &recs, &why)) << why;
  ASSERT_GT(recs.size(), 10u);
  const std::vector<uint8_t> good = recs[5].data;
  const Ros1Decoder dec(sch.schema);
  FieldNode n;
  size_t used = 0;
  ASSERT_TRUE(dec.decode(good.data(), good.size(), &n, &used, &why)) << why;

  // every truncation fails cleanly
  for (size_t len = 0; len < good.size(); ++len) {
    why.clear();
    EXPECT_FALSE(dec.decode(good.data(), len, &n, &used, &why)) << "len " << len;
    EXPECT_LE(used, len);
    EXPECT_FALSE(why.empty());
  }
  // trailing bytes = layout mismatch
  std::vector<uint8_t> longer = good;
  longer.push_back(0);
  EXPECT_FALSE(dec.decode(longer.data(), longer.size(), &n, &used, &why));
  EXPECT_NE(why.find("trailing"), std::string::npos) << why;
  EXPECT_EQ(used, good.size());

  // absurd counts are rejected by the count-vs-bytes-left check, i.e. before anything is allocated (would be up
  // to 4e9 elements); the reason names that check and the array
  for (uint32_t bad : {0xFFFFFFFFu, 0x7FFFFFFFu, static_cast<uint32_t>(good.size())}) {
    std::vector<uint8_t> b = good;
    std::memcpy(b.data(), &bad, 4);   // meas[] count
    EXPECT_FALSE(dec.decode(b.data(), b.size(), &n, &used, &why)) << bad;
    EXPECT_NE(why.find("array 'meas' claims " + std::to_string(bad) + " elements"), std::string::npos) << why;
    EXPECT_EQ(used, 4u) << "nothing past the count may be read";
  }
  {   // first satellite's freqs[] count (after meas count 4 + time 12 + sat 4)
    std::vector<uint8_t> b = good;
    const uint32_t bad = 0x40000000u;
    std::memcpy(b.data() + 20, &bad, 4);
    EXPECT_FALSE(dec.decode(b.data(), b.size(), &n, &used, &why));
    EXPECT_NE(why.find("meas[0].freqs"), std::string::npos) << why;
  }
  // random corruption: either a clean failure or a complete decode, never a crash
  std::mt19937 rng(20250925u);
  int failures = 0;
  for (int trial = 0; trial < 3000; ++trial) {
    std::vector<uint8_t> b = good;
    const int nflip = 1 + static_cast<int>(rng() % 4);
    for (int k = 0; k < nflip; ++k) b[rng() % b.size()] = static_cast<uint8_t>(rng());
    if (rng() % 3 == 0) b.resize(rng() % (b.size() + 1));
    if (!dec.decode(b.data(), b.size(), &n, &used, &why)) ++failures;
    else EXPECT_EQ(used, b.size());
  }
  EXPECT_GT(failures, 0);
  // null / empty inputs
  EXPECT_FALSE(dec.decode(nullptr, 10, &n, &used, &why));
  EXPECT_FALSE(dec.decode(good.data(), 0, &n, &used, &why));

  // cursor primitives at the buffer end
  const uint8_t three[3] = {1, 2, 3};
  Ros1Decoder::Cursor c;
  c.p = three;
  c.n = 3;
  uint32_t u32v = 0;
  EXPECT_FALSE(c.readU32(&u32v));
  uint16_t u16v = 0;
  EXPECT_TRUE(c.readU16(&u16v));
  EXPECT_EQ(u16v, 0x0201);
  std::string sv;
  c.pos = 0;
  EXPECT_FALSE(c.readString(&sv));   // length 0x030201 > bytes left
  EXPECT_EQ(c.pos, 0u);              // cursor restored
  const uint8_t str[6] = {2, 0, 0, 0, 'o', 'k'};
  Ros1Decoder::Cursor cs;
  cs.p = str;
  cs.n = 6;
  EXPECT_FALSE(cs.readString(&sv, 1));   // max_len guard
  EXPECT_TRUE(cs.readString(&sv));
  EXPECT_EQ(sv, "ok");

  // the RawDecoder front end: unregistered source and garbage bytes both fail, nothing is pushed
  GnssCommRos1Decoder gd([](Sys, int prn) { return prn; });
  NullSink sink;
  EXPECT_FALSE(gd.decode(Source::GnssCommMeas, good.data(), good.size(), 1.0, sink, &why));
  EXPECT_FALSE(why.empty());
  ASSERT_TRUE(gd.registerMessageDefinition(Source::GnssCommMeas, sch.datatype, sch.md5, sch.msgdef, &why)) << why;
  std::vector<uint8_t> garbage(good.begin(), good.begin() + 37);
  EXPECT_FALSE(gd.decode(Source::GnssCommMeas, garbage.data(), garbage.size(), 1.0, sink, &why));
  EXPECT_EQ(gd.stats().decode_errors, 2u);
  EXPECT_EQ(sink.obs, 0);
  EXPECT_TRUE(gd.decode(Source::GnssCommMeas, good.data(), good.size(), 1.0, sink, &why)) << why;
  EXPECT_EQ(sink.obs, 1);

  // malformed definitions: false + reason
  const std::vector<std::pair<std::string, std::string>> bad_defs = {
      {"p/A", "float64 1bad\n"},                                     // illegal field name
      {"p/A", "float64[x] a\n"},                                     // bad array length
      {"p/A", "float64[3 a\n"},                                      // unterminated suffix
      {"p/A", "float64 a b\n"},                                      // three tokens
      {"p/A", "Foo a\n"},                                            // unresolved nested type
      {"p/A", "B b\n" + kSep + "MSG: p/B\nA a\n"},                   // recursion
      {"p/A", "int8 a\n" + kSep + "int8 b\n"},                       // section without MSG:
      {"p/A", "int8 a\nint8 a\n"},                                   // duplicate field
      {"p/A", "B b\n" + kSep + "MSG: p/B\nint8 x\n" + kSep + "MSG: p/B\nint16 x\n"},   // conflicting sections
      {"A", "int8 a\n"},                                             // datatype without package
      {"", "int8 a\n"}};
  for (const auto& d : bad_defs) {
    MsgSchema s;
    why.clear();
    EXPECT_FALSE(MsgDefParser::parse(d.first, d.second, &s, &why)) << d.second;
    EXPECT_FALSE(why.empty()) << d.second;
  }
  FieldSpec f;
  EXPECT_FALSE(MsgDefParser::parseFieldLine("   # only a comment", "p", &f, &why));
  EXPECT_TRUE(why.empty());   // not an error, just not a field
  EXPECT_FALSE(MsgDefParser::parseFieldLine("int32 X=5", "p", &f, &why));
  EXPECT_TRUE(why.empty());
  EXPECT_TRUE(MsgDefParser::parseFieldLine("Header h", "p", &f, &why));
  EXPECT_EQ(f.resolved_type, "std_msgs/Header");
  EXPECT_TRUE(MsgDefParser::parseFieldLine("Header[] hs", "p", &f, &why));   // genmsg: only the bare token maps
  EXPECT_EQ(f.resolved_type, "p/Header");
  EXPECT_TRUE(MsgDefParser::parseFieldLine("geometry_msgs/msg/Point pt", "p", &f, &why));
  EXPECT_EQ(f.resolved_type, "geometry_msgs/Point");
}

// Decoded output is bounded per message whatever the input: 2^18 tree nodes (ros1_decoder.cpp kMaxNodes; a
// gnss_comm range_meas needs ~2·10³). Without the budget a uint8[] of n bytes becomes n Values (~80 B each).
TEST(U02Wire, DecodedNodesAreBudgetedPerMessage) {
  constexpr uint32_t kBudget = 1u << 18;   // keep equal to kMaxNodes
  MsgSchema one, two;
  std::string why;
  ASSERT_TRUE(MsgDefParser::parse("u2_test/Blob", "uint8[] a\n", &one, &why)) << why;
  ASSERT_TRUE(MsgDefParser::parse("u2_test/Blobs", "uint8[] a\nuint8[] b\n", &two, &why)) << why;
  auto blob = [](const std::vector<uint32_t>& lens) {
    Wire w;
    for (uint32_t n : lens) {
      w.u32(n);
      w.b.insert(w.b.end(), n, static_cast<uint8_t>(0x5A));
    }
    return w.b;
  };
  FieldNode n;
  size_t used = 0;
  // within budget: decodes (1 root field + the elements)
  const std::vector<uint8_t> ok = blob({kBudget - 1});
  ASSERT_TRUE(Ros1Decoder(one).decode(ok.data(), ok.size(), &n, &used, &why)) << why;
  EXPECT_EQ(at(n, "a").array.size(), kBudget - 1);
  // one element more than the budget allows: refused before the allocation, with the reason
  const std::vector<uint8_t> over = blob({kBudget});
  EXPECT_FALSE(Ros1Decoder(one).decode(over.data(), over.size(), &n, &used, &why));
  EXPECT_NE(why.find("budget"), std::string::npos) << why;
  EXPECT_EQ(used, 4u);
  // the budget is per message, not per array: two halves that each fit are refused together
  const std::vector<uint8_t> split = blob({kBudget / 2, kBudget / 2});
  EXPECT_FALSE(Ros1Decoder(two).decode(split.data(), split.size(), &n, &used, &why));
  EXPECT_NE(why.find("'b'"), std::string::npos) << why;
  // ... and a fresh message starts with the full budget again
  ASSERT_TRUE(Ros1Decoder(one).decode(ok.data(), ok.size(), &n, &used, &why)) << why;
}

// Golden CSV of time and duration fields: time = two unsigned 32-bit numbers, duration = two signed ones (ROS1
// wire), in the C++ writer, and back through rowToTree() bit for bit (tools/gnss_bag_export.py prints the same).
TEST(U02Wire, GoldenCsvTimeAndDurationSignedness) {
  MsgSchema s;
  std::string why;
  ASSERT_TRUE(MsgDefParser::parse("u2_test/Times", "time t\nduration d\ntime[] ts\n", &s, &why)) << why;
  Wire w;
  w.u32(0x80000001u);        // t.sec beyond 2^31 (2038+) stays unsigned
  w.u32(999999999u);
  w.i32(-5);                 // d = -5 s - 0.25 s
  w.i32(-250000000);
  w.u32(2);
  w.u32(1u);
  w.u32(2u);
  w.u32(3u);
  w.u32(4u);
  FieldNode n;
  size_t used = 0;
  ASSERT_TRUE(Ros1Decoder(s).decode(w.b.data(), w.b.size(), &n, &used, &why)) << why;
  const std::string csv = ts::goldenCsv(s, {{7u, n}});
  EXPECT_EQ(csv, "t_record_ns,t.sec,t.nsec,d.sec,d.nsec,ts_0.sec,ts_0.nsec,ts_1.sec,ts_1.nsec\n"
                 "7,2147483649,999999999,-5,-250000000,1,2,3,4\n");
  ts::Csv parsed;
  ASSERT_TRUE(ts::parseCsv(csv, &parsed, &why)) << why;
  FieldNode back;
  uint64_t t_ns = 0;
  ASSERT_TRUE(ts::rowToTree(s, parsed, 0, &back, &t_ns, &why)) << why;
  EXPECT_EQ(t_ns, 7u);
  std::string diff;
  EXPECT_TRUE(ts::compareTrees(n, back, "times", &diff)) << diff;
  EXPECT_DOUBLE_EQ(at(back, "d").scalar.toDouble(), -5.25);
}

// Definition text edited by field *line*: the line whose comment-free tokens are exactly `type name` is found
// (comments and spacing do not matter), so the variants below hold whether the fixture stores the definition
// comment-stripped (default, tools/gnss_bag_export.py) or verbatim.
namespace {
std::vector<std::string> lines(const std::string& text) {
  std::vector<std::string> out;
  std::stringstream in(text);
  for (std::string l; std::getline(in, l);) out.push_back(l);
  return out;
}
std::string joined(const std::vector<std::string>& v) {
  std::string t;
  for (const std::string& l : v) t += l + "\n";
  return t;
}
size_t findField(const std::vector<std::string>& v, const std::string& type, const std::string& name) {
  for (size_t i = 0; i < v.size(); ++i) {
    std::stringstream in(v[i].substr(0, v[i].find('#')));
    std::string a, b, c;
    if ((in >> a >> b) && !(in >> c) && a == type && b == name) return i;
  }
  return std::string::npos;
}
// replace (to != ""), delete (to == "") or, with `after`, insert `to` after the field line
std::string editField(const std::string& text, const std::string& type, const std::string& name, const std::string& to, bool after = false) {
  std::vector<std::string> v = lines(text);
  const size_t i = findField(v, type, name);
  EXPECT_NE(i, std::string::npos) << type << " " << name;
  if (i == std::string::npos) return text;
  if (after) v.insert(v.begin() + static_cast<std::ptrdiff_t>(i) + 1, to);
  else if (to.empty()) v.erase(v.begin() + static_cast<std::ptrdiff_t>(i));
  else v[i] = to;
  return joined(v);
}
std::string swapFields(const std::string& text, const std::string& t1, const std::string& n1, const std::string& t2, const std::string& n2) {
  std::vector<std::string> v = lines(text);
  const size_t i = findField(v, t1, n1), j = findField(v, t2, n2);
  EXPECT_TRUE(i != std::string::npos && j != std::string::npos);
  if (i == std::string::npos || j == std::string::npos) return text;
  std::swap(v[i], v[j]);
  return joined(v);
}
}  // namespace

// a modified (type,name) layout is rejected with a version diff
TEST(U02Wire, ExpectationTableRejectsChangedLayout) {
  const std::string fx = fixtureRoot() + "/Synthetic_1419919200_10s";
  if (!ts::isDir(fx)) GTEST_SKIP() << "optional synthetic wire fixture is not distributed in the research preview: " << fx;
  ts::TopicSchema meas;
  std::string why;
  ASSERT_TRUE(ts::loadTopic(fx, "range_meas", &meas, &why)) << why;
  ASSERT_TRUE(validateLayout(Source::GnssCommMeas, meas.schema, &why)) << why;
  const std::string& def = meas.msgdef;
  struct Variant {
    const char* what;
    std::string text;
    const char* diff_mentions;
  };
  const std::vector<Variant> variants = {
      {"swapped fields", swapFields(def, "float64[]", "psr", "float64[]", "psr_std"), "meas.psr"},
      {"float64 -> float32", editField(def, "float64[]", "cp", "float32[] cp"), "float32[] meas.cp"},
      {"renamed field", editField(def, "float64[]", "CN0", "float64[] cn0"), "meas.cn0"},
      {"added field", editField(def, "uint32", "sat", "uint8 extra", true), "meas.extra"},
      {"removed field", editField(def, "float64[]", "dopp_std", ""), "meas.dopp_std"},
      {"nested type changed", editField(def, "float64", "tow", "float32 tow"), "meas.time.tow"},
      {"variable -> fixed array", editField(def, "uint8[]", "LLI", "uint8[2] LLI"), "uint8[2] meas.LLI"}};
  for (const Variant& v : variants) {
    SCOPED_TRACE(v.what);
    MsgSchema s;
    ASSERT_TRUE(MsgDefParser::parse(meas.datatype, v.text, &s, &why)) << why;   // still a valid definition
    std::string diff;
    EXPECT_FALSE(validateLayout(Source::GnssCommMeas, s, &diff));
    EXPECT_NE(diff.find(v.diff_mentions), std::string::npos) << diff;
    GnssCommRos1Decoder gd([](Sys, int prn) { return prn; });
    EXPECT_FALSE(gd.registerMessageDefinition(Source::GnssCommMeas, meas.datatype, meas.md5, v.text, &why));
    EXPECT_FALSE(why.empty());
    EXPECT_FALSE(gd.registered(Source::GnssCommMeas));
  }
  {   // the same layout under another package is a different message (type names are part of the table)
    std::string t = meas.msgdef;
    for (size_t p = t.find("MSG: gnss_comm/"); p != std::string::npos; p = t.find("MSG: gnss_comm/", p)) t.replace(p, 15, "MSG: other_pkg/");
    MsgSchema s;
    ASSERT_TRUE(MsgDefParser::parse("other_pkg/GnssMeasMsg", t, &s, &why)) << why;
    std::string diff;
    EXPECT_FALSE(validateLayout(Source::GnssCommMeas, s, &diff));
    EXPECT_NE(diff.find("datatype"), std::string::npos) << diff;
  }
  {   // comments, blank lines and trailing blanks only: accepted, same md5 (genmsg ignores them)
    std::string noisy = "# rewritten\n\n";
    for (const std::string& l : lines(meas.msgdef))   // separators and MSG: lines never carry comments
      noisy += l + ((l.empty() || l[0] == '=' || l.compare(0, 4, "MSG:") == 0) ? "\n" : "   # note\n\n");
    MsgSchema s;
    ASSERT_TRUE(MsgDefParser::parse(meas.datatype, noisy, &s, &why)) << why;
    EXPECT_TRUE(validateLayout(Source::GnssCommMeas, s, &why)) << why;
    EXPECT_EQ(MsgDefParser::md5(s), meas.md5);
  }
  {   // a failed re-registration drops the previously accepted schema (no stale decoding)
    GnssCommRos1Decoder gd([](Sys, int prn) { return prn; });
    ASSERT_TRUE(gd.registerMessageDefinition(Source::GnssCommMeas, meas.datatype, meas.md5, meas.msgdef, &why)) << why;
    EXPECT_TRUE(gd.registered(Source::GnssCommMeas));
    EXPECT_FALSE(gd.registerMessageDefinition(Source::GnssCommMeas, meas.datatype, meas.md5, variants[1].text, &why));
    EXPECT_FALSE(gd.registered(Source::GnssCommMeas));
  }
  // every expectation table matches its own topic and no other one; time_pulse is never rejected (A19)
  const std::vector<std::string> topics = {"range_meas", "ephem", "glo_ephem", "iono_params", "receiver_pvt"};
  for (const std::string& a : topics) {
    ts::TopicSchema sa;
    ASSERT_TRUE(ts::loadTopic(fx, a, &sa, &why)) << why;
    EXPECT_FALSE(expectedLayout(sourceOfTopic(a)).empty()) << a;
    for (const std::string& b : topics) {
      std::string diff;
      EXPECT_EQ(validateLayout(sourceOfTopic(b), sa.schema, &diff), a == b) << a << " as " << b << ": " << diff;
    }
    EXPECT_TRUE(validateLayout(Source::GnssCommTimePulse, sa.schema, &why));
    EXPECT_FALSE(validateLayout(Source::Rinex, sa.schema, &why));
  }
  EXPECT_TRUE(expectedLayout(Source::GnssCommTimePulse).empty());
}
