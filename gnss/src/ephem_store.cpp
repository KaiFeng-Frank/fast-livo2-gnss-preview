// SPDX-License-Identifier: BSD-3-Clause
// EphemStore (DESIGN §12.3, AMENDMENTS A1, A16, A38). Owner: WP-a. Tests: U3, U4.
#include "internal/ephem_store.hpp"

#include <cmath>
#include <set>
#include <tuple>

#include "livo_gnss/signal_table.hpp"

namespace livo_gnss {

namespace {

bool isKeplerSys(Sys s) { return s == Sys::GPS || s == Sys::GAL || s == Sys::QZS || s == Sys::BDS || s == Sys::IRN; }

}  // namespace

EphemStore::EphemStore(const Config& cfg, rtk::NavStore& nav) : cfg_(cfg), nav_(nav), ura_(cfg.ephem.ura_semantics) {
  // A16: the Galileo record type is a property of this RTKLIB copy's global selection (seleph and prange read it),
  // decided once by the configuration when the core builds its ephemeris store.
  rtk::setGalileoNavSelection(cfg.ephem.gal_prefer_inav ? rtk::GalNavSelection::INav : rtk::GalNavSelection::Any);
}

double EphemStore::toesFor(Sys sys, GTime toe_gpst) {
  // eph2pos rotates by omge·toes (ephemeris.c:266/:277): toes must be in the system's own week (R2-3).
  if (sys == Sys::BDS) return rtk::time2bdt(rtk::gpst2bdt(toe_gpst), nullptr);
  return rtk::time2gpst(toe_gpst, nullptr);   // GPS/QZS/IRN; GAL: GST seconds of week == GPST seconds of week
}

int EphemStore::weekFor(Sys sys, GTime toe_gpst) {
  int w = 0;
  if (sys == Sys::BDS) {
    rtk::time2bdt(rtk::gpst2bdt(toe_gpst), &w);   // BDT week (GPS week − 1356)
    return w;
  }
  // GPS/QZS/IRN: GPS week. GAL: RINEX 3 "GAL week" and RTKLIB eph_t.week are continuous with the GPS week
  // (rcvraw.c decode_gal_inav: gst-week + 1024; rinex.c outrnxnavb writes ttr relative to it), not the GST week.
  rtk::time2gpst(toe_gpst, &w);
  return w;
}

void EphemStore::completeFields(KeplerEph* eph) {
  eph->toes = toesFor(eph->sat.sys, eph->toe);
  eph->week = weekFor(eph->sat.sys, eph->toe);
}

bool EphemStore::addBag(KeplerEph eph) {
  // gnss_comm GAL 60–97 includes E37/E38, beyond RTKLIB MAXPRNGAL (36): counted, never mapped (§12.3).
  if (eph.sat.sys == Sys::GAL && eph.sat.prn > 36) {
    ++n_gal_prn_reject_;
    return false;
  }
  int prn = 0;
  const Sys sys = rtk::satSys(eph.sat.rtk_sat, &prn);
  if (!isKeplerSys(sys)) return false;
  // A16's URA semantics describe the gnss_comm messages only. A record RTKLIB decoded itself (UBX stream; a RINEX file
  // read through RinexSource) already carries RTKLIB's accuracy index in sva (ura_raw = sva, WP-e contract in
  // INTERFACE_CHANGES), so it is taken as is — also when the semantics are Unknown, which A16 allows for sources other
  // than gnss_comm_ros1.
  const bool rtklib_decoded = eph.source == Source::UbxStream || eph.source == Source::Rinex;
  if (!rtklib_decoded) {
    switch (ura_) {
      case Config::Ephem::UraSemantics::Unknown:
        // A16: the meaning of the delivered URA is decided by the audit (self-check 4), never assumed here.
        ++n_ura_unknown_reject_;
        return false;
      case Config::Ephem::UraSemantics::Index:
        eph.sva = static_cast<int>(std::lround(eph.ura_raw));
        break;
      case Config::Ephem::UraSemantics::Metres:
        // Galileo accuracy is SISA, not URA: RTKLIB reads it with sisa_index (rinex.c) and eph2pos interprets sva as SISA.
        eph.sva = (sys == Sys::GAL) ? rtk::sisaIndexFromMetres(eph.ura_raw) : rtk::uraIndexFromMetres(eph.ura_raw);
        break;
    }
  }
  if (sys == Sys::GAL && !(eph.code & ((1 << 8) | (1 << 9)))) {
    eph.code |= (1 << 0) | (1 << 9);   // see galSourceBitsAssumed()
    ++n_gal_code_assumed_;
  }
  eph.sat = rtk::makeSatId(sys, prn);
  completeFields(&eph);
  // A1: bag entries only fill keys RINEX (or an earlier bag message) does not have.
  return nav_.add(eph, /*replace_existing=*/false);
}

bool EphemStore::addBag(GloEph eph) {
  int prn = 0;
  if (rtk::satSys(eph.sat.rtk_sat, &prn) != Sys::GLO) return false;
  // FDMA plan k ∈ [−7, +6] (RTKLIB code2freq_GLO rejects the rest); a channel that disagrees with the one already
  // on record for this slot is a mapping error, not a new satellite (§12.3 "不一致丢弃计数").
  const int known = nav_.gloFcn(prn);
  if (eph.frq < -7 || eph.frq > 6 || (known != -100 && known != eph.frq)) {
    ++n_glo_fcn_reject_;
    return false;
  }
  eph.sat = rtk::makeSatId(Sys::GLO, prn);
  GloEph existing;
  if (nav_.get(eph.sat, eph.toe, &existing)) {
    // RINEX wins (A1) — but RINEX 3.04 has no dtaun: keep the bag's G1 group delay on the RINEX record (R2-5).
    if (existing.source == Source::Rinex && existing.dtaun == 0.0 && eph.dtaun != 0.0) {
      existing.dtaun = eph.dtaun;
      nav_.add(existing, /*replace_existing=*/true);
      ++n_glo_dtaun_filled_;
    }
    nav_.add(eph, /*replace_existing=*/false);   // stored record untouched; marks the key bag-delivered (m3)
    return false;
  }
  if (!nav_.add(eph, /*replace_existing=*/false)) return false;
  if (known == -100) nav_.setGloFcn(prn, eph.frq);
  return true;
}

int EphemStore::loadRinex(const std::string& path, std::string* why) {
  // dtaun of bag GLONASS records that a RINEX 3.04 record of the same key is about to replace.
  std::vector<std::tuple<SatId, GTime, double>> bag_dtaun;
  for (const rtk::EphKey& k : nav_.coverage()) {
    if (!k.is_glo || k.source == Source::Rinex) continue;
    GloEph g;
    if (nav_.get(k.sat, k.toe, &g) && g.dtaun != 0.0) bag_dtaun.emplace_back(k.sat, k.toe, g.dtaun);
  }
  const bool had_ion = nav_.iono(nullptr);
  const int n = nav_.loadRinexNav(path, /*replace_existing=*/true, why);
  if (n < 0) return n;
  for (const auto& t : bag_dtaun) {
    GloEph g;
    if (nav_.get(std::get<0>(t), std::get<1>(t), &g) && g.source == Source::Rinex && g.dtaun == 0.0) {
      g.dtaun = std::get<2>(t);
      nav_.add(g, /*replace_existing=*/true);
      ++n_glo_dtaun_filled_;
    }
  }
  // GLONASS channel numbers from the RINEX records (sat2freq also finds them through geph; the table serves the
  // freqs[] cross-check before any record of a slot is stored).
  for (const rtk::EphKey& k : nav_.coverage()) {
    if (!k.is_glo) continue;
    int prn = 0;
    rtk::satSys(k.sat.rtk_sat, &prn);
    if (nav_.gloFcn(prn) != -100) continue;
    GloEph g;
    if (nav_.get(k.sat, k.toe, &g) && g.frq >= -7 && g.frq <= 6) nav_.setGloFcn(prn, g.frq);
  }
  // Klobuchar (A38): NavStore::loadRinexNav copies the header GPSA/GPSB into nav.ion_gps only while nothing is stored
  // there, so the first NAV file that carries them is the one in force; it is this store's GPS Klobuchar iff it came
  // from a file loaded here (nav.ion_gps was empty before).
  Klobuchar k;
  if (!has_iono_rinex_ && !had_ion && nav_.iono(&k)) {
    k.source = Source::Rinex;
    iono_rinex_ = k;
    has_iono_rinex_ = true;
  }
  return n;
}

void EphemStore::setIonoFromBag(const Klobuchar& ion) {
  // A38: M3DGR's /ublox_driver/iono_params differ from the day's GPSA/GPSB in all eight values and follow BDS ephemeris
  // messages (WP-e #9, WP-g E6): read as GPS Klobuchar they would put a BDS model into the GPS ionosphere. Recorded for
  // the diagnostics only; nav.ion_gps, iono() and hasIono() (has_iono_bag_ stays false) are left to the RINEX header.
  iono_bag_ = ion;
  ++n_bag_iono_;
}

bool EphemStore::iono(Klobuchar* out) const {
  if (!has_iono_rinex_) return false;   // A37/A38: no RINEX GPSA/GPSB ⇒ no GPS Klobuchar (I = 0 in the core)
  if (out) *out = iono_rinex_;
  return true;
}

bool EphemStore::bagIono(Klobuchar* out) const {
  if (n_bag_iono_ == 0) return false;
  if (out) *out = iono_bag_;
  return true;
}

// Frozen skeleton definition (df88477), kept verbatim.
bool EphemStore::hasIono() const { return has_iono_bag_ || has_iono_rinex_; }

bool EphemStore::gloFcnConsistent(const SatId& sat, double observed_freq_hz) const {
  int k_obs = 0;
  SignalVerdict v = SignalVerdict::Accepted;
  if (signalFromFrequency(Sys::GLO, observed_freq_hz, &k_obs, &v) == SignalCode::Unknown) return false;
  const int k_eph = gloFcn(sat);
  return k_eph != -100 && k_eph == k_obs;
}

int EphemStore::gloFcn(const SatId& sat) const {
  int prn = 0;
  if (rtk::satSys(sat.rtk_sat, &prn) != Sys::GLO) return -100;
  return nav_.gloFcn(prn);
}

bool EphemStore::healthy(const SatId& sat, GTime t, double* age_s) const {
  int prn = 0;
  const Sys sys = rtk::satSys(sat.rtk_sat, &prn);
  // m16: every system has its own key (QZSS 'J'); a missing key is a configuration hole, not "0 s" or "unlimited".
  const auto it = cfg_.masks.eph_age_s.find(sysChar(sys));
  if (it == cfg_.masks.eph_age_s.end()) return false;
  const rtk::SatSolver solver(nav_);
  return solver.ephHealthy(sat, t, it->second, age_s);
}

int EphemStore::nSatsHealthy(GTime t) const {
  std::set<int> sats;
  for (const rtk::EphKey& k : nav_.coverage()) sats.insert(k.sat.rtk_sat);
  int n = 0;
  for (int s : sats) {
    SatId id;
    id.rtk_sat = s;
    id.sys = rtk::satSys(s, &id.prn);
    n += healthy(id, t, nullptr) ? 1 : 0;
  }
  return n;
}

EphemStore::Coverage EphemStore::coverage() const {
  Coverage c;
  c.entries = nav_.coverage();
  for (const rtk::EphKey& k : c.entries) {
    const bool rinex = k.source == Source::Rinex;
    if (k.is_glo) {
      (rinex ? c.n_geph_rinex : c.n_geph_bag) += 1;
      c.n_geph_bag_delivered += k.bag_delivered ? 1 : 0;
    } else {
      (rinex ? c.n_eph_rinex : c.n_eph_bag) += 1;
      c.n_eph_bag_delivered += k.bag_delivered ? 1 : 0;
    }
  }
  return c;
}

}  // namespace livo_gnss
