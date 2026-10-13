// SPDX-License-Identifier: BSD-3-Clause
// UbxStream via input_ubx (DESIGN §12.4). Owner: WP-e. Tests: T5. Includes rtklib.h (pimpl raw_t, A21).
//
// Framing is done here (sync 0xB5 0x62, length, Fletcher-8 checksum) so that checksum failures can be counted
// separately from RTKLIB's other "-1" outcomes and a corrupted frame is re-scanned for a later sync pair.
// Only checksum-valid frames are handed to input_ubx, byte by byte:
//   RXM-RAWX  → ObsEpoch (RTKLIB rules: invalid pr/cp zeroed, BDS GEO +0.5 cycle, slip/half-cycle → LLI);
//               receiver std-devs kept through RTKLIB's "-RCVSTDS" option (pr 0.01·2^n m, cp n·0.004 cycles);
//   RXM-SFRBX → KeplerEph / GloEph when RTKLIB completes an ephemeris; GPS subframe 4 page 18 → Klobuchar;
//   NAV-PVT   → ReceiverPvt, decoded here (RTKLIB has no NAV-PVT decoder): time of week from iTOW (exact ms),
//               week from the UTC date (RTKLIB's leap-second table) or else the last RAWX epoch, within half a
//               week; refused (decode() false, nothing pushed) when neither exists.
// Known dependency: RTKLIB resolves the 10-bit LNAV week with the host clock (adjgpsweek → timeget); correct
// while the host date is within ±512 weeks of the data (DESIGN §16-13: UBX path untested on hardware).
#include "livo_gnss_io/ubx_stream.hpp"

#include <cstdio>
#include <cstring>
#include <vector>

#include "rtklib_conv.hpp"   // last: includes rtklib.h

namespace livo_gnss_io {

namespace {

constexpr uint8_t kSync1 = 0xB5, kSync2 = 0x62;

uint32_t u4(const uint8_t* p) {
  return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) | (static_cast<uint32_t>(p[2]) << 16) |
         (static_cast<uint32_t>(p[3]) << 24);
}
int32_t i4(const uint8_t* p) { return static_cast<int32_t>(u4(p)); }
uint16_t u2(const uint8_t* p) { return static_cast<uint16_t>(p[0] | (p[1] << 8)); }

// UBX-NAV-PVT (class 0x01 id 0x07, 92-byte payload; u-blox interface description, F9 HPG 1.32 §3.15.13).
// iTOW is exact but carries no week. The week comes from a reference epoch: the UTC date/time when the receiver
// flags both valid (converted with RTKLIB's leap-second table), otherwise the last RXM-RAWX epoch. The reference
// only has to lie within half a week of the solution: the date fields are whole (rounded) seconds, so at the end
// of a week they can already point into the next one; the week that keeps |t − ref| < 302400 s is taken.
// Without any reference the epoch is unknown and the message is refused (nothing pushed).
bool decodeNavPvt(const uint8_t* pl, size_t len, const gtime_t& last_raw_time, livo_gnss::ReceiverPvt* out, std::string* err) {
  if (len < 92) {
    *err = "NAV-PVT payload of " + std::to_string(len) + " bytes (92 expected)";
    return false;
  }
  livo_gnss::ReceiverPvt p;
  const double tow = u4(pl + 0) * 1e-3;   // iTOW [ms] → s (GPS time of week)
  const uint8_t valid = pl[11];
  gtime_t ref = {0, 0.0};
  if ((valid & 0x03) == 0x03) {   // validDate && validTime
    double ep[6] = {static_cast<double>(u2(pl + 4)), static_cast<double>(pl[6]), static_cast<double>(pl[7]),
                    static_cast<double>(pl[8]), static_cast<double>(pl[9]), static_cast<double>(pl[10])};
    const gtime_t utc = epoch2time(ep);
    if (utc.time != 0) ref = utc2gpst(utc);
  }
  if (ref.time == 0) ref = last_raw_time;
  if (ref.time == 0) {
    *err = "NAV-PVT without a valid UTC date before any RXM-RAWX epoch: GPS week unknown";
    return false;
  }
  int week = 0;
  time2gpst(ref, &week);
  gtime_t t = gpst2time(week, tow);
  const double d = timediff(t, ref);
  if (d > 302400.0) t = gpst2time(week - 1, tow);
  else if (d < -302400.0) t = gpst2time(week + 1, tow);
  p.t = rtkconv::toGTime(t);
  p.fix_type = pl[20];
  const uint8_t flags = pl[21];
  p.valid_fix = (flags & 0x01) != 0;        // gnssFixOK
  p.diff_soln = (flags & 0x02) != 0;
  p.carr_soln = (flags >> 6) & 0x03;
  p.num_sv = pl[23];
  p.lon_deg = i4(pl + 24) * 1e-7;
  p.lat_deg = i4(pl + 28) * 1e-7;
  p.h_ell_m = i4(pl + 32) * 1e-3;
  p.h_msl_m = i4(pl + 36) * 1e-3;
  p.h_acc_m = u4(pl + 40) * 1e-3;
  p.v_acc_m = u4(pl + 44) * 1e-3;
  p.vel_ned = Eigen::Vector3d(i4(pl + 48) * 1e-3, i4(pl + 52) * 1e-3, i4(pl + 56) * 1e-3);
  p.vel_acc_mps = u4(pl + 68) * 1e-3;
  p.p_dop = u2(pl + 76) * 0.01;
  *out = p;
  return true;
}

}  // namespace

struct UbxStream::Impl {
  raw_t raw;
  bool inited = false;
  std::vector<uint8_t> buf;   // bytes not yet framed (carried across decode() calls)
  uint64_t frames = 0, cksum_err = 0;
  uint32_t seq = 0;

  Impl() { init(); }
  ~Impl() { fini(); }
  void init() {
    std::memset(&raw, 0, sizeof(raw));   // init_raw leaves most of nav_t (ion_gps, glo_fcn, ...) untouched
    inited = init_raw(&raw, STRFMT_UBX) != 0;
    if (inited) std::snprintf(raw.opt, sizeof(raw.opt), "%s", "-RCVSTDS");
    buf.clear();
    frames = cksum_err = 0;
    seq = 0;
  }
  void fini() {
    if (inited) free_raw(&raw);
    inited = false;
  }

  // One checksum-valid frame. Returns false on an RTKLIB decode error (-1).
  bool frame(const uint8_t* f, size_t total, double t_record, livo_gnss::MeasurementSink& sink, std::string* err) {
    const uint8_t cls = f[2], id = f[3];
    const uint8_t* pl = f + 6;
    const size_t plen = total - 8;
    int ret = 0;
    for (size_t k = 0; k < total; ++k) ret = input_ubx(&raw, f[k]);   // only the last byte can complete a message
    if (ret < 0) {
      *err = "RTKLIB rejected UBX " + std::to_string(cls) + "/" + std::to_string(id);
      return false;
    }
    if (ret == 1) {   // observations (RXM-RAWX / RXM-RAW)
      livo_gnss::ObsEpoch ep;
      ep.t_rx = rtkconv::toGTime(raw.time);
      ep.t_record = t_record;
      ep.source = livo_gnss::Source::UbxStream;
      ep.seq = ++seq;
      for (int i = 0; i < raw.obs.n; ++i) {
        livo_gnss::SatObs so;
        if (rtkconv::toSatObs(raw.obs.data[i], &raw.nav, NFREQ + NEXOBS, &so)) ep.sats.push_back(std::move(so));
      }
      sink.pushObservation(std::move(ep));
    } else if (ret == 2) {   // a complete ephemeris
      int prn = 0;
      const int sat = raw.ephsat;
      const int sys = satsys(sat, &prn);
      if (sys == SYS_GLO) {
        if (prn >= 1 && prn <= MAXPRNGLO) sink.pushGloEphemeris(rtkconv::toGlo(raw.nav.geph[prn - 1], livo_gnss::Source::UbxStream, t_record));
      } else if (sys != SYS_NONE && sat >= 1 && sat <= MAXSAT && (raw.ephset == 0 || raw.ephset == 1)) {
        sink.pushEphemeris(rtkconv::toKepler(raw.nav.eph[sat - 1 + MAXSAT * raw.ephset], livo_gnss::Source::UbxStream, t_record));
      }
    } else if (ret == 9) {   // ion/utc: only GPS LNAV carries the Klobuchar set the core uses (§12.1)
      if (cls == 0x02 && id == 0x13 && plen >= 1 && pl[0] == 0 /* gnssId GPS */)
        sink.pushIono(rtkconv::toKlobuchar(raw.nav.ion_gps, livo_gnss::Source::UbxStream, t_record));
    }
    if (cls == 0x01 && id == 0x07) {   // NAV-PVT (not an RTKLIB message: input_ubx returned 0)
      livo_gnss::ReceiverPvt p;
      if (!decodeNavPvt(pl, plen, raw.time, &p, err)) return false;
      p.t_record = t_record;
      p.source = livo_gnss::Source::UbxStream;
      sink.pushPvt(p);
    }
    return true;
  }
};

UbxStream::UbxStream() : impl_(new Impl) {}
UbxStream::~UbxStream() = default;

bool UbxStream::registerMessageDefinition(livo_gnss::Source src, const std::string& datatype, const std::string& md5, const std::string& msg_def,
                                          std::string* why) {
  (void)src;
  (void)datatype;
  (void)md5;
  (void)msg_def;
  if (why) why->clear();
  return true;   // not applicable to a byte stream
}

bool UbxStream::decode(livo_gnss::Source src, const uint8_t* data, size_t size, double t_record, livo_gnss::MeasurementSink& sink,
                       std::string* why) {
  (void)src;
  Impl& m = *impl_;
  if (why) why->clear();
  if (!m.inited) {
    if (why) *why = "RTKLIB init_raw failed";
    return false;
  }
  if (!data && size) {
    if (why) *why = "null buffer";
    return false;
  }
  m.buf.insert(m.buf.end(), data, data + size);
  const uint64_t cks_before = m.cksum_err;
  std::string err, first_err;
  size_t pos = 0;
  const size_t n = m.buf.size();
  while (true) {
    size_t i = pos;
    while (i + 1 < n && !(m.buf[i] == kSync1 && m.buf[i + 1] == kSync2)) ++i;
    if (i + 1 >= n) {   // no complete sync pair; keep a trailing 0xB5 that may start one
      pos = (i < n && m.buf[i] == kSync1) ? i : n;
      break;
    }
    if (n - i < 6) {   // header incomplete
      pos = i;
      break;
    }
    const size_t total = static_cast<size_t>(u2(&m.buf[i + 4])) + 8;
    if (total > static_cast<size_t>(MAXRAWLEN)) {   // cannot be a frame RTKLIB accepts: resync after this pair
      pos = i + 1;
      continue;
    }
    if (n - i < total) {   // wait for the rest of the frame
      pos = i;
      break;
    }
    uint8_t ck_a = 0, ck_b = 0;
    for (size_t k = i + 2; k < i + total - 2; ++k) {
      ck_a = static_cast<uint8_t>(ck_a + m.buf[k]);
      ck_b = static_cast<uint8_t>(ck_b + ck_a);
    }
    if (ck_a != m.buf[i + total - 2] || ck_b != m.buf[i + total - 1]) {
      ++m.cksum_err;
      pos = i + 1;   // a genuine frame may start inside the corrupted one
      continue;
    }
    ++m.frames;
    if (!m.frame(&m.buf[i], total, t_record, sink, &err) && first_err.empty()) first_err = err;
    pos = i + total;
  }
  m.buf.erase(m.buf.begin(), m.buf.begin() + static_cast<std::ptrdiff_t>(pos));
  if (m.cksum_err != cks_before && first_err.empty())
    first_err = std::to_string(m.cksum_err - cks_before) + " UBX frame(s) with a bad checksum";
  if (!first_err.empty()) {
    if (why) *why = first_err;
    return false;
  }
  return true;
}

void UbxStream::reset() {   // back to a freshly constructed decoder (state, buffers and counters)
  impl_->fini();
  impl_->init();
}

uint64_t UbxStream::framesDecoded() const { return impl_->frames; }
uint64_t UbxStream::checksumErrors() const { return impl_->cksum_err; }

}  // namespace livo_gnss_io
