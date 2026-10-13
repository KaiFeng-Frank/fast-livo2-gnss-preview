// SPDX-License-Identifier: BSD-3-Clause
// EphemStore — ephemeris semantics on top of rtk::NavStore (DESIGN §12.3, AMENDMENTS A1, A38):
//   * per-system toes/week (BDS: toes = time2bdt(gpst2bdt(toe)); GAL: GST seconds == GPST seconds),
//   * bag ⊕ RINEX merge: RINEX broadcast entries win on the same key, bag fills the gaps; the key is
//     (sat, toe, nav_type) where nav_type separates Galileo I/NAV from F/NAV (IGS BRDC carries both with
//     one toe; m7) — every stored entry remembers its Source so self-check 8 can report "bag only" coverage,
//   * URA semantics (A16): sva is derived from ura_raw according to cfg.ephem.ura_semantics (Index: sva =
//     int(ura_raw); Metres: sva = RTKLIB uraindex(ura_raw)); Unknown ⇒ addBag() refuses and counts,
//   * GPS Klobuchar from the RINEX BRDC header GPSA/GPSB only (A38); bag iono_params are a diagnostic,
//   * GLONASS channel bookkeeping (glo_fcn) with the freqs[] cross-check,
//   * health / age masks (§10 L0) and the coverage report of self-check 8 ("仅 bag" vs "bag+RINEX").
// Owner: WP-a. Tests: test_u03_satmap.cpp (field mapping round trip), test_u04_satstate.cpp.
#pragma once

#include <string>
#include <vector>

#include "livo_gnss/config.hpp"
#include "livo_gnss/types.hpp"
#include "rtk_adapter.hpp"

namespace livo_gnss {

class EphemStore {
 public:
  EphemStore(const Config& cfg, rtk::NavStore& nav);

  // System-specific fields (U3). toes is the ephemeris reference time in the system's own week
  // seconds; week the system week. Wrong toes for BDS ⇒ 27 km satellite error (R2-3).
  static double toesFor(Sys sys, GTime toe_gpst);
  static int weekFor(Sys sys, GTime toe_gpst);
  // Fills toes/week of a bag ephemeris in place (idempotent).
  static void completeFields(KeplerEph* eph);

  // Bag-sourced ephemerides (gnss_comm). Returns true when stored (false = RINEX entry already present
  // for this key, or rejected: GAL PRN 37–38, unmapped sat, URA semantics Unknown; the unhealthy flag policy
  // is NOT applied here). The URA semantics (A16) apply to gnss_comm records only: a KeplerEph whose source is
  // Source::UbxStream or Source::Rinex (decoded by RTKLIB, sva already an RTKLIB index) keeps its sva.
  bool addBag(KeplerEph eph);
  bool addBag(GloEph eph);
  // RINEX NAV files (IGS BRDC merged). Entries replace bag entries with the same key.
  int loadRinex(const std::string& path, std::string* why);
  // A16: override the config's URA semantics (audit tool / tests).
  void setUraSemantics(Config::Ephem::UraSemantics s) { ura_ = s; }
  Config::Ephem::UraSemantics uraSemantics() const { return ura_; }
  int rejectedUraUnknown() const { return n_ura_unknown_reject_; }

  // Klobuchar (AMENDMENTS A38, revising A1/§12.3): the GPS Klobuchar of the core is the RINEX BRDC header GPSA/GPSB of
  // the first loaded NAV file that carries it (it also sets nav.ion_gps, what pntpos reads). Bag iono_params are not GPS
  // Klobuchar on M3DGR (they equal the BDS D1 set of the day, WP-e #9 / WP-g E6): setIonoFromBag() only records them
  // (bagIonoCount()/bagIono() below) — never nav.ion_gps, iono() or hasIono(). Without RINEX GPSA/GPSB, iono() and
  // hasIono() are false and the core uses I = 0 with σ_ion = 5 m (A37, DESIGN §8).
  void setIonoFromBag(const Klobuchar& ion);
  bool iono(Klobuchar* out) const;
  bool hasIono() const;

  // GLONASS: k from the ephemeris vs k from the observed frequency must agree (§12.3); mismatch → drop + count.
  bool gloFcnConsistent(const SatId& sat, double observed_freq_hz) const;
  int gloFcn(const SatId& sat) const;

  // L0 mask helpers: healthy (rtk::svhHealthy — svh == 0 with RTKLIB's QZSS/GLONASS readings) and age ≤
  // masks.eph_age_s[system letter].
  bool healthy(const SatId& sat, GTime t, double* age_s) const;
  int nSatsHealthy(GTime t) const;                       // WAIT_NAV: ≥5 required (§8)

  struct Coverage {
    // Stored records by the source of the record kept for the key (a bag record survives only where RINEX has none).
    int n_eph_bag = 0, n_eph_rinex = 0, n_geph_bag = 0, n_geph_rinex = 0;
    std::vector<rtk::EphKey> entries;                    // (sat, toe, nav_type, source) sorted by sat then toe
    // ---- appended 2026-09-25 (WP-a, INTERFACE_CHANGES, review m3) ----
    // Keys the bag delivered at least once, whichever record is stored (EphKey::bag_delivered): self-check 8's
    // "仅 bag" coverage; "bag+RINEX" is all of `entries`.
    int n_eph_bag_delivered = 0, n_geph_bag_delivered = 0;
  };
  Coverage coverage() const;
  int rejectedGalPrn() const { return n_gal_prn_reject_; }
  int rejectedGloFcn() const { return n_glo_fcn_reject_; }

 private:
  const Config& cfg_;
  rtk::NavStore& nav_;
  Config::Ephem::UraSemantics ura_;
  Klobuchar iono_bag_, iono_rinex_;             // iono_bag_: the last bag iono_params (diagnostic only, A38)
  bool has_iono_bag_ = false, has_iono_rinex_ = false;   // has_iono_bag_ stays false since A38 (the bag never supplies
                                                          // the GPS Klobuchar); the frozen hasIono() still reads it
  int n_gal_prn_reject_ = 0, n_glo_fcn_reject_ = 0, n_ura_unknown_reject_ = 0;

 public:
  // ---- appended 2026-09-25 (WP-a, INTERFACE_CHANGES) ----
  // Galileo bag ephemerides that carried neither data-source bit 9 (I/NAV) nor 8 (F/NAV) in eph code: taken as
  // I/NAV E1-B, code |= bits 0 and 9 = 513 — what RTKLIB's own u-blox path produces (rcvraw.c:336 decode_gal_inav
  // sets bit 9 only, ublox.c:979 decode_inav adds bit 0 for E1-B; u-blox tracks E1/E5b, never F/NAV). Without it
  // setseleph(SYS_GAL, 1) (A16) would never select them. Counted for the ingest report.
  int galSourceBitsAssumed() const { return n_gal_code_assumed_; }
  // GLONASS bag ephemerides whose delta_tau_n was copied into a RINEX record of the same (sat, toe): RINEX 3.04
  // has no dtaun field, and "RINEX wins" (A1) must not silently drop the G1 group delay (R2-5).
  int gloDtaunFilledFromBag() const { return n_glo_dtaun_filled_; }

 private:
  int n_gal_code_assumed_ = 0, n_glo_dtaun_filled_ = 0;

 public:
  // ---- appended 2026-09-25, P2b (WP-a; AMENDMENTS A38) ----
  // Bag iono_params delivered so far and the last one, for the ingest report / events (diagnostics only: they never
  // reach nav.ion_gps, iono() or hasIono()). bagIono() is false before the first message.
  int bagIonoCount() const { return n_bag_iono_; }
  bool bagIono(Klobuchar* out) const;

 private:
  int n_bag_iono_ = 0;
};

}  // namespace livo_gnss
