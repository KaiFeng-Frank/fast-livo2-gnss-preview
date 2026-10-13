// SPDX-License-Identifier: BSD-3-Clause
// RinexSource via readrnxt (DESIGN §12.4). Owner: WP-e. Tests: T4, U6. Includes rtklib.h (pimpl, A21).
//
// Observation epochs are RTKLIB's (readrnxt → sortobs, grouped with RTKLIB's own DTTOL like its nextobsf);
// each record keeps the main slots only (one code per band = RTKLIB's code priority), and signals are named
// through the §12.3 frequency table exactly like the gnss_comm path (rtklib_conv.hpp), primary first.
// NAV entries are de-duplicated with uniqnav. Nothing here touches RTKLIB global state except what
// readrnxt itself reads (the code-priority table is left at its default; setcodepri is never called).
#include "livo_gnss_io/rinex_source.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>

#include "rtklib_conv.hpp"   // last: includes rtklib.h

namespace livo_gnss_io {

struct RinexSource::Impl {
  obs_t obs;
  nav_t nav;
  sta_t sta;
  size_t cursor = 0;                    // next epoch index into starts
  std::vector<size_t> starts;           // first obs.data index of every epoch (+ obs.n sentinel)
  std::vector<livo_gnss::KeplerEph> eph;
  std::vector<livo_gnss::GloEph> geph;
  livo_gnss::Klobuchar ion;
  bool has_ion = false;

  Impl() { zero(); }
  ~Impl() { release(); }
  void zero() {
    std::memset(&obs, 0, sizeof(obs));
    std::memset(&nav, 0, sizeof(nav));
    std::memset(&sta, 0, sizeof(sta));
  }
  void release() {
    freeobs(&obs);
    freenav(&nav, 0xFF);
    zero();
    cursor = 0;
    starts.clear();
    eph.clear();
    geph.clear();
    ion = livo_gnss::Klobuchar();
    has_ion = false;
  }
};

RinexSource::RinexSource() : impl_(new Impl) {}
RinexSource::~RinexSource() = default;

bool RinexSource::open(const std::string& obs_path, const std::string& nav_path, livo_gnss::GTime t_start, livo_gnss::GTime t_end,
                       std::string* why) {
  Impl& m = *impl_;
  m.release();
  auto fail = [&](const std::string& s) {
    m.release();
    if (why) *why = s;
    return false;
  };
  if (why) why->clear();
  if (obs_path.empty() && nav_path.empty()) return fail("neither an OBS nor a NAV path was given");
  // readrnxt reports a missing file as "no data"; say what actually happened.
  for (const std::string* p : {&nav_path, &obs_path}) {
    if (p->empty()) continue;
    std::FILE* f = std::fopen(p->c_str(), "r");
    if (!f) return fail("cannot open '" + *p + "'");
    std::fclose(f);
  }
  const gtime_t none = {0, 0.0};
  // NAV first: GLONASS channels then come from the broadcast records; the OBS header's
  // "GLONASS SLOT / FRQ #" (read into the same nav_t) covers satellites without an ephemeris.
  if (!nav_path.empty()) {
    const int stat = readrnxt(nav_path.c_str(), 1, none, none, 0.0, "", nullptr, &m.nav, nullptr);
    if (stat <= 0 || (m.nav.n <= 0 && m.nav.ng <= 0)) return fail("no navigation data read from '" + nav_path + "'");
  }
  if (!obs_path.empty()) {
    // ts/te = 0 means unbounded (RTKLIB screent).
    const gtime_t ts = t_start.valid() ? rtkconv::toRtk(t_start) : none;
    const gtime_t te = t_end.valid() ? rtkconv::toRtk(t_end) : none;
    const int stat = readrnxt(obs_path.c_str(), 1, ts, te, 0.0, "", &m.obs, &m.nav, &m.sta);
    if (stat < 0) return fail("error reading observations from '" + obs_path + "'");
    if (m.obs.n <= 0) return fail("no observation epochs in '" + obs_path + "' within the requested window");
    sortobs(&m.obs);
    for (int i = 0; i < m.obs.n;) {
      m.starts.push_back(static_cast<size_t>(i));
      int j = i + 1;
      while (j < m.obs.n && timediff(m.obs.data[j].time, m.obs.data[i].time) <= DTTOL) ++j;
      i = j;
    }
    m.starts.push_back(static_cast<size_t>(m.obs.n));
  }
  uniqnav(&m.nav);
  for (int i = 0; i < m.nav.n; ++i) {
    if (!m.nav.eph[i].sat) continue;
    m.eph.push_back(rtkconv::toKepler(m.nav.eph[i], livo_gnss::Source::Rinex, 0.0));
  }
  for (int i = 0; i < m.nav.ng; ++i) {
    if (!m.nav.geph[i].sat) continue;
    m.geph.push_back(rtkconv::toGlo(m.nav.geph[i], livo_gnss::Source::Rinex, 0.0));
  }
  bool any_ion = false;
  for (int i = 0; i < 8; ++i) any_ion = any_ion || m.nav.ion_gps[i] != 0.0;
  if (any_ion) {
    m.ion = rtkconv::toKlobuchar(m.nav.ion_gps, livo_gnss::Source::Rinex, 0.0);
    m.has_ion = true;
  }
  return true;
}

bool RinexSource::nextEpoch(livo_gnss::ObsEpoch* out) {
  Impl& m = *impl_;
  if (m.starts.size() < 2 || m.cursor + 1 >= m.starts.size()) return false;
  const size_t b = m.starts[m.cursor], e = m.starts[m.cursor + 1];
  livo_gnss::ObsEpoch ep;
  ep.t_rx = rtkconv::toGTime(m.obs.data[b].time);
  ep.t_record = 0.0;   // files carry no host record time
  ep.source = livo_gnss::Source::Rinex;
  ep.seq = static_cast<uint32_t>(m.cursor + 1);
  for (size_t i = b; i < e; ++i) {
    livo_gnss::SatObs so;
    if (rtkconv::toSatObs(m.obs.data[i], &m.nav, NFREQ, &so)) ep.sats.push_back(std::move(so));
  }
  ++m.cursor;
  if (out) *out = std::move(ep);
  return true;
}

size_t RinexSource::epochsRemaining() const {
  const Impl& m = *impl_;
  return m.starts.size() < 2 ? 0 : (m.starts.size() - 1 - m.cursor);
}

const std::vector<livo_gnss::KeplerEph>& RinexSource::ephemerides() const { return impl_->eph; }
const std::vector<livo_gnss::GloEph>& RinexSource::gloEphemerides() const { return impl_->geph; }

bool RinexSource::iono(livo_gnss::Klobuchar* out) const {
  if (!impl_->has_ion) return false;
  if (out) *out = impl_->ion;
  return true;
}

}  // namespace livo_gnss_io
