// SPDX-License-Identifier: BSD-3-Clause
// rtk_adapter — the ONLY translation unit that includes rtklib.h is gnss/src/rtk_adapter.cpp
// (DESIGN §12.1 隔离; pimpl). Everything RTKLIB provides to the core passes through this header:
// time conversions, satellite numbering, ephemeris store (nav_t), satellite state (satposs), model
// corrections (ionmodel/tropmodel/geodist/satazel), coordinate transforms, and the pntpos reference
// solver (patched P2 exports). No RTKLIB macro or type leaks out (no PI/CLIGHT/MAXOBS here).
// Owner: WP-a. Tests: test_u01_time.cpp (time), test_u03_satmap.cpp (numbering/eph mapping),
// test_u04_satstate.cpp (eph2pos/geph2pos), test_u05_corrections.cpp (prange/resdop/pntpos), test_u06_paths.cpp.
#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "livo_gnss/config.hpp"
#include "livo_gnss/signal_table.hpp"
#include "livo_gnss/types.hpp"

namespace livo_gnss {
namespace rtk {

// ---------------------------------------------------------------------------------------------
// Constants (values copied from rtklib.h so callers never include it; keep in sync in U1/U3 tests).
// ---------------------------------------------------------------------------------------------
constexpr double kCLight = 299792458.0;        // [m/s]
constexpr double kOmgE = 7.2921151467e-5;      // WGS-84 earth rotation rate [rad/s]
constexpr double kFreqL1 = 1.57542e9;          // GPS/GAL/QZS L1/E1 [Hz]
constexpr double kFreqL2 = 1.22760e9;
constexpr double kFreqL5 = 1.17645e9;
constexpr double kFreqE5b = 1.20714e9;         // GAL E5b / BDS B2I
constexpr double kFreqB1I = 1.561098e9;        // BDS B1I
constexpr double kFreqG1 = 1.60200e9;          // GLO G1 base
constexpr double kDFreqG1 = 0.56250e6;         // GLO G1 channel step
constexpr double kFreqG2 = 1.24600e9;
constexpr double kDFreqG2 = 0.43750e6;

// ---------------------------------------------------------------------------------------------
// Time (rtkcmn.c). All bit-identical to the RTKLIB functions of the same name (U1).
// ---------------------------------------------------------------------------------------------
GTime gpst2time(int week, double tow);
double time2gpst(GTime t, int* week);
GTime gpst2utc(GTime t);
GTime utc2gpst(GTime t);
GTime gpst2bdt(GTime t);                // GPST − 14 s
GTime bdt2gpst(GTime t);
GTime bdt2time(int week, double tow);   // BDT week (GPS week − 1356)
double time2bdt(GTime t, int* week);
GTime gst2time(int week, double tow);   // GST week (GPS week − 1024)
double time2gst(GTime t, int* week);
GTime epoch2time(const double ep[6]);
void time2epoch(GTime t, double ep[6]);
double utc2gmst(GTime utc, double ut1_utc);
GTime timeadd(GTime t, double sec);
double timediff(GTime a, GTime b);
std::string timeStr(GTime t, int n_frac);   // time2str
double leapSecondsAt(GTime gpst);           // GPST−UTC via gpst2utc (table lookup by date)

// ---------------------------------------------------------------------------------------------
// Satellite numbering and signals (rtkcmn.c satno/satsys/satid2no/sat2freq/code2idx).
// ---------------------------------------------------------------------------------------------
int satNo(Sys sys, int prn);                       // 0 if out of range (e.g. C51–C63 before P1, E37–E38)
Sys satSys(int sat, int* prn);                     // Sys::Unknown if invalid
std::string satIdString(int sat);                  // "G07", "C59"
int satIdFromString(const std::string& id);
SatId makeSatId(Sys sys, int prn);                 // fills rtk_sat via satNo
int maxSat();                                      // MAXSAT (255 cap asserted by P1)
int maxPrnBds();                                   // MAXPRNCMP as compiled (50 before P1, 63 after)

// SignalCode ↔ RTKLIB CODE_*. The freqs → SignalCode table itself lives in livo_gnss/signal_table.hpp
// (single copy, M6): use livo_gnss::signalFromFrequency(); it also yields the GLONASS channel k.
int codeToRtk(SignalCode code);
SignalCode codeFromRtk(int rtk_code);
double sat2freq(int sat, SignalCode code, const class NavStore& nav);   // RTKLIB sat2freq (needs glo_fcn); 0 if unknown
int freqIndex(Sys sys, SignalCode code);                    // code2idx (0 = L1 slot, 1 = L2/E5b/B2 slot)

// Galileo ephemeris selection of this RTKLIB copy (ephemeris.c seleph: 0 = closest toe of either type,
// 1 = I/NAV only (code bit 9), 2 = F/NAV only (bit 8)). Global state — set once by the core (A16 gal_prefer_inav).
enum class GalNavSelection : int { Any = 0, INav = 1, FNav = 2 };
void setGalileoNavSelection(GalNavSelection sel);
GalNavSelection galileoNavSelection();
// URA index from a URA value in metres (RTKLIB uraindex()) — used when cfg.ephem.ura_semantics = Metres.
int uraIndexFromMetres(double ura_m);
double uraMetresFromIndex(int sva);                         // RTKLIB ephemeris.c ura_value[] (the bound var_uraeph squares; not
                                                            // rinex.c ura_nominal), 6144 m when out of range

// ---------------------------------------------------------------------------------------------
// Coordinates (rtkcmn.c ecef2pos/pos2ecef/xyz2enu).
// ---------------------------------------------------------------------------------------------
Geodetic ecef2pos(const Eigen::Vector3d& r_ecef);
Eigen::Vector3d pos2ecef(const Geodetic& geo);
Eigen::Matrix3d enuRotation(const Geodetic& geo);          // R_{E<-N} (transpose of xyz2enu), DESIGN §1
double geodist(const Eigen::Vector3d& rs, const Eigen::Vector3d& rr, Eigen::Vector3d* e);   // incl. Sagnac
void satAzel(const Geodetic& rcv, const Eigen::Vector3d& e, double* az, double* el);
// [m] on L1. Exactly 0 when !ion.valid() (DESIGN §8: no Klobuchar set ⇒ I = 0, σ_ion from the caller) — never
// RTKLIB ionmodel()'s built-in 2004 parameters, which it substitutes for an all-zero set.
double ionModelGpsL1(GTime t, const Klobuchar& ion, const Geodetic& rcv, double az, double el);
double tropModel(GTime t, const Geodetic& rcv, double az, double el, double humi);               // [m]

// ---------------------------------------------------------------------------------------------
// NavStore — owns a nav_t (pimpl). Merge policy is decided by ephem_store; this class only stores.
// Key of an entry (m7): (sat, toe, nav_type). nav_type = 0 for every system except Galileo, where it
// separates I/NAV (eph_t.code bit 9 → 1) from F/NAV (bit 8 → 2) so both BRDC records with one toe fit.
// Each entry keeps the Source it came from (bag vs RINEX) for the self-check-8 "bag only" coverage.
// ---------------------------------------------------------------------------------------------
struct EphKey {
  SatId sat;
  GTime toe;
  int nav_type = 0;                     // 0 = n/a, 1 = GAL I/NAV, 2 = GAL F/NAV
  Source source = Source::Synthetic;    // source of the record that is stored for this key
  bool is_glo = false;
  // ---- appended 2026-09-25 (WP-a, INTERFACE_CHANGES, review m3) ----
  // A non-RINEX source (bag / UBX / synthetic) delivered this key at least once, whichever record is stored now:
  // RINEX replacing a bag record, or a bag record meeting a RINEX one, keeps it. Self-check 8's "bag only" coverage.
  bool bag_delivered = false;
};
int galNavTypeOf(int eph_code);         // from eph_t.code data-source bits (0 for non-Galileo codes)

class NavStore {
 public:
  NavStore();
  ~NavStore();
  NavStore(const NavStore&) = delete;
  NavStore& operator=(const NavStore&) = delete;

  // Insert into nav.eph / nav.geph. When an entry with the same key exists: replace if
  // `replace_existing`, else keep the old one. Returns true when the store changed. Fills
  // eph_t.toes/week from the KeplerEph fields (caller sets them per system, see EphemStore); eph_t.sva
  // is taken from KeplerEph::sva as given (EphemStore derives it from ura_raw per A16).
  // A record whose source is not Source::Rinex marks its key EphKey::bag_delivered even when the stored record
  // is kept (m3); the mark survives any later replacement.
  bool add(const KeplerEph& eph, bool replace_existing);
  bool add(const GloEph& eph, bool replace_existing);
  bool has(const SatId& sat, GTime toe, int nav_type = 0) const;
  bool get(const SatId& sat, GTime toe, KeplerEph* out, int nav_type = 0) const;
  bool get(const SatId& sat, GTime toe, GloEph* out) const;
  Source sourceOf(const SatId& sat, GTime toe, int nav_type = 0) const;   // Synthetic when absent
  void setIono(const Klobuchar& ion);           // nav.ion_gps
  bool iono(Klobuchar* out) const;
  void setGloFcn(int prn, int frq);              // nav.glo_fcn[prn-1] = frq + 8
  int gloFcn(int prn) const;                     // frq, or -100 if unknown
  // readrnxt(nav only). Adds every ephemeris (replace_existing policy applies; Source::Rinex) and, if the
  // header carries GPSA/GPSB and no iono is stored yet, the Klobuchar parameters. Returns count added.
  int loadRinexNav(const std::string& path, bool replace_existing, std::string* why);
  int countEph(Sys sys) const;
  int countGeph() const;
  std::vector<EphKey> coverage() const;          // every stored entry with its source (self-check 8)
  void clear();

  struct Impl;
  Impl* impl() const { return impl_.get(); }     // used only inside rtk_adapter.cpp / gnss_io rinex

 private:
  std::unique_ptr<Impl> impl_;
};

// A frozen copy of one eph_t/geph_t so both ends of a TDCP pair use the same ephemeris (§4.5, R2-11).
class EphSnapshot {
 public:
  EphSnapshot();
  ~EphSnapshot();
  EphSnapshot(const EphSnapshot&);
  EphSnapshot& operator=(const EphSnapshot&);
  bool valid() const;
  SatId sat() const;
  GTime toe() const;
  struct Impl;
  Impl* impl() const { return impl_.get(); }

 private:
  std::unique_ptr<Impl> impl_;
};

// Satellite state at transmit time.
struct SatState {
  bool ok = false;
  Eigen::Vector3d r_ecef = Eigen::Vector3d::Zero();   // [m] at t_tx (ECEF at t_tx; Sagnac handled in geodist)
  Eigen::Vector3d v_ecef = Eigen::Vector3d::Zero();   // [m/s]
  double dts = 0.0;                                   // satellite clock bias [s] incl. relativistic term
  double ddts = 0.0;                                  // satellite clock drift [s/s]
  double var_eph = 0.0;                               // σ_eph² [m²] (self-computed from sva / URA semantics, §4.3)
  int svh = 0;
  GTime t_tx;                                         // t_rx − P/c − δt_s (two-step, ephemeris.c:804→:822)
  // m5: what the row builder needs besides position/clock:
  double tgd_s[2] = {0.0, 0.0};                       // eph_t.tgd[0..1] (GPS TGD; GAL BGD E5a/E1, E5b/E1; BDS TGD1/TGD2); GLO: dtaun in [0]
                                                      // (E1 with an I/NAV clock, A16, pairs with BGD E5b/E1 = [1]; RTKLIB-EX
                                                      // prange() uses [0] there). Row builders take the group delay from
                                                      // rtk::codeBiasMetres(), never from these raw fields (review M3).
  GTime toe;                                          // toe of the ephemeris that was selected
  int iode = -1;                                      // its IODE (BDS: AODE, not unique — never used as a key)
  int sva = -1;                                       // its URA index
  int nav_type = 0;                                   // GAL I/NAV = 1, F/NAV = 2
};

class SatSolver {
 public:
  explicit SatSolver(const NavStore& nav);
  // Selects the ephemeris RTKLIB-style (minimum |t − toe|) and evaluates eph2pos/geph2pos at t_tx.
  // `pseudorange_m` may be the predicted range ρ̂ + b̂ − c·δt_s for carrier-only epochs (§1). Optionally
  // captures the ephemeris used into `snap_out`.
  bool satState(const SatId& sat, GTime t_rx, double pseudorange_m, SatState* out, EphSnapshot* snap_out = nullptr) const;
  // Same evaluation with a frozen ephemeris copy (TDCP k0/k1 consistency).
  static bool satStateFromSnapshot(const EphSnapshot& snap, GTime t_rx, double pseudorange_m, SatState* out);
  // Direct eph2pos/geph2pos at time t (U4: bit-compare bag path vs RINEX path).
  bool ephPos(const SatId& sat, GTime t, Eigen::Vector3d* rs, double* dts, double* var) const;
  // Health per DESIGN §10 L0 as rtk::svhHealthy() (RTKLIB satexclude reading of svh) and age within limit (age
  // computed against toe).
  bool ephHealthy(const SatId& sat, GTime t, double max_age_s, double* age_s) const;

 private:
  const NavStore& nav_;
};

// ---------------------------------------------------------------------------------------------
// pntpos reference solver (DESIGN §10 L1 "参照行", U5/U6). Uses the P2 exports for the full x/Q.
// ---------------------------------------------------------------------------------------------
struct SppRefOptions {
  bool use_glo_psr = false;
  bool raim_fde = false;
  double el_mask_deg = 10.0;
  int ionoopt = 1;          // RTKLIB IONOOPT_BRDC
  int tropopt = 1;          // TROPOPT_SAAS
  double err_brdci = 0.5;
};

struct SppRefResult {
  bool ok = false;
  std::string msg;
  Eigen::Vector3d r_ecef = Eigen::Vector3d::Zero();
  double dtr_s[6] = {0, 0, 0, 0, 0, 0};       // receiver clock/ISBs [s] (RTKLIB sol.dtr)
  Eigen::Vector3d v_ecef = Eigen::Vector3d::Zero();
  double ddtr = 0.0;                          // clock drift [s/s] (P2 export of estvel x[3])
  Eigen::VectorXd x_pos;                      // full estpos state (P2)
  Eigen::MatrixXd Q_pos;                      // full estpos covariance (P2)
  Eigen::VectorXd x_vel;                      // full estvel state (P2)
  Eigen::MatrixXd Q_vel;
  int ns = 0;
  double gdop = 0.0;
  std::vector<std::pair<SatId, double>> resp;   // per-sat pseudorange residuals (ssat.resp) at the solution
  std::vector<std::pair<SatId, double>> resc;   // per-sat carrier residuals
  // ---- appended 2026-09-25 (WP-a, INTERFACE_CHANGES) ----
  // estpos/estvel evaluate their residuals before the last update: ssat.resp and the elevations belong to the
  // linearisation point of the last iteration (x_pos_lin), not to x_pos (they differ by the last step, < 1e-4 m).
  // Empty when P2 is not compiled in (rtk::patchLevel()).
  Eigen::VectorXd x_pos_lin;
  Eigen::VectorXd x_vel_lin;
  std::vector<std::pair<SatId, Eigen::Vector2d>> azel;   // {az, el} [rad] of the satellites in resp, at x_pos_lin
};

class SppRef {
 public:
  SppRef();
  ~SppRef();
  bool solve(const ObsEpoch& epoch, const NavStore& nav, const SppRefOptions& opt, SppRefResult* out) const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

// P2 export wrappers (pntpos.c static functions made reachable). Used by U5 to check the core's own
// pseudorange prediction against RTKLIB's at the pntpos solution (< 1e-6 m).
struct PrangeRef {
  bool ok = false;
  double P_corr = 0.0;      // rtkx_prange: TGD/BGD-corrected pseudorange [m]
  double var = 0.0;         // rtkx_varerr [m²]
  double tgd_s = 0.0;       // rtkx_gettgd [s]
};
PrangeRef prangeRef(const SatObs& obs, const NavStore& nav, const SppRefOptions& opt);

// ---------------------------------------------------------------------------------------------
// Appended 2026-09-25 (WP-a, INTERFACE_CHANGES).
// ---------------------------------------------------------------------------------------------
// Patches compiled into this RTKLIB copy (bit mask). The LIVO_RTKLIB_APPLY_PATCHES=OFF build that writes the U5
// golden file reports 0 (AMENDMENTS A9-4); the default build reports kPatchP1 | kPatchP2.
constexpr int kPatchP1 = 1;   // MAXPRNCMP 63 (C51–C63 mappable) + MAXSAT <= 255 assertion
constexpr int kPatchP2 = 2;   // rtkx_prange/rtkx_varerr/rtkx_gettgd + full estpos/estvel x/Q
int patchLevel();
// Galileo SISA index ↔ metres (RTKLIB rinex.c sisa_index/sisa_value). uraIndexFromMetres() is the GPS URA table;
// Galileo needs this one (A16 "metres" semantics). sisaMetresFromIndex returns −1 for NAPA/unknown (> 125).
int sisaIndexFromMetres(double sisa_m);
double sisaMetresFromIndex(int sisa);
// σ_eph² [m²] from the ephemeris accuracy index, self-computed (DESIGN §4.3 "不经 var_uraeph"): GPS/QZS/BDS/IRN
// URA table (6144 m when out of range), Galileo SISA (NAPA → 500 m like RTKLIB STD_GAL_NAPA), GLONASS (5 m)².
double ephVarianceFromIndex(Sys sys, int sva);

// ---- appended 2026-09-25, review round (WP-a, INTERFACE_CHANGES) ----
// Broadcast group delay [m] to subtract from the pseudorange of the primary signal `code` of `sat` (DESIGN §4.3
// P_corr), taken from the record `st` was evaluated with (SatSolver::satState / satStateFromSnapshot):
//   GPS/QZS L1C: c·TGD;  BDS L2I (B1I): c·TGD1;  GLONASS L1C (G1): c·(−dtaun)/(γ−1), γ = (f_G1/f_G2)²  — these are the
//   single-frequency branch of RTKLIB prange() (pntpos.c:161-189);
//   Galileo L1X (E1): the BGD that belongs to the clock of the selected record — I/NAV (E5b/E1 clock) c·BGD_E1E5b =
//   c·tgd[1], F/NAV (E5a/E1 clock) c·BGD_E1E5a = c·tgd[0]. RTKLIB-EX 2.5.1 prange() instead keys on getseleph(SYS_GAL)
//   (≠0 ⇒ tgd[0]) — a 2.4.3 leftover that under A16's I/NAV selection pairs the I/NAV clock with the F/NAV BGD
//   (0.49 m on E07 on 2025-01-03; review M3). Deliberately not reproduced; the reference row keeps RTKLIB's value.
// NaN for any other (system, code) — secondary signals and unknown Galileo data source — so a misuse is loud.
double codeBiasMetres(const SatId& sat, const SatState& st, SignalCode code);
// Broadcast health of `svh` as RTKLIB satexclude() reads it (rtkcmn.c:531-552; DESIGN §10 L0 "svh = 0" with the two
// system rules RTKLIB applies): QZSS ignores bit 0 (L1C/B–LEX health, set on every QZSS record in the IGS BRDC);
// GLONASS svh = Bn MSB (bit 0) | RINEX ≥ 3.05 health flags << 1 (rinex.c:1356) is healthy iff (svh & 9) == 0 and
// (svh & 6) != 4; every other system needs svh == 0; svh < 0 (no ephemeris) is unhealthy. SatState::svh stays raw.
bool svhHealthy(Sys sys, int svh);

}  // namespace rtk
}  // namespace livo_gnss
