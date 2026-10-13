// SPDX-License-Identifier: BSD-3-Clause
// Public measurement / ephemeris / time types of the livo_gnss core (DESIGN §1, §12.3, §12.5).
// ROS-free, RTKLIB-free: rtklib.h is only included by gnss/src/rtk_adapter.cpp. Every value here is
// in SI units unless the member name says otherwise; times are GPST unless the member name says otherwise.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "livo_gnss/export.hpp"

namespace livo_gnss {

// ---------------------------------------------------------------------------------------------
// Time (DESIGN §1 "时间"): GPST as integer seconds since the Unix epoch plus a fraction — the same
// representation as RTKLIB gtime_t so conversions through rtk_adapter are exact (U1).
// ---------------------------------------------------------------------------------------------
struct LIVO_GNSS_API GTime {
  int64_t sec = 0;    // integer seconds (Unix epoch, GPST scale)
  double frac = 0.0;  // fractional seconds, [0,1)

  static GTime fromSec(double t);   // split; loses precision beyond ~1e-7 s for 1.7e9 s — use for coarse times only
  double toSec() const { return static_cast<double>(sec) + frac; }
  bool valid() const { return sec != 0 || frac != 0.0; }
  GTime add(double dt) const;       // == RTKLIB timeadd (normalises frac)
  // == RTKLIB timediff bit for bit: difftime(t1.time,t2.time)+t1.sec-t2.sec evaluates left to right,
  // i.e. (Δsec + frac) − b.frac; the other association differs by 1 ulp in ~20 % of random cases (m2).
  double diff(const GTime& b) const { return (static_cast<double>(sec - b.sec) + frac) - b.frac; }
  bool operator<(const GTime& b) const { return diff(b) < 0.0; }
  bool operator==(const GTime& b) const { return sec == b.sec && frac == b.frac; }
  bool operator!=(const GTime& b) const { return !(*this == b); }
};

// Week/time-of-week pair as delivered by receivers (GPST unless the producer says otherwise).
struct WeekTow {
  int week = 0;
  double tow = 0.0;
};

// ---------------------------------------------------------------------------------------------
// Constellations. Bit values mirror RTKLIB SYS_* exactly so masks can be passed through the adapter
// without translation (rtklib.h: SYS_GPS 0x01, SYS_SBS 0x02, SYS_GLO 0x04, SYS_GAL 0x08, SYS_QZS 0x10,
// SYS_CMP 0x20, SYS_IRN 0x40). Never rely on the numeric values outside rtk_adapter.
// (`Unknown`, not `None`: X11 headers define None/Bool/Status as macros and the ROS glue may pull them in.)
// ---------------------------------------------------------------------------------------------
enum class Sys : uint8_t { Unknown = 0x00, GPS = 0x01, SBS = 0x02, GLO = 0x04, GAL = 0x08, QZS = 0x10, BDS = 0x20, IRN = 0x40 };

inline constexpr uint8_t sysBit(Sys s) { return static_cast<uint8_t>(s); }
LIVO_GNSS_API const char* sysName(Sys s);   // "GPS","GLO","GAL","QZS","BDS","IRN","SBS","Unknown"
LIVO_GNSS_API char sysChar(Sys s);          // RINEX letter: G R E J C I S

// Satellite identity. `rtk_sat` is the RTKLIB satno() index (1..MAXSAT, 0 = unmapped) and is the only
// key used inside the core; (sys, prn) are kept for logging. gnss_comm numbering (GPS 1-32, GLO 33-59,
// GAL 60-97, BDS 98-160) is translated by gnss_io before reaching this struct (DESIGN §12.3).
struct SatId {
  Sys sys = Sys::Unknown;
  int prn = 0;
  int rtk_sat = 0;
  bool valid() const { return rtk_sat > 0; }
  bool operator==(const SatId& b) const { return rtk_sat == b.rtk_sat; }
  bool operator<(const SatId& b) const { return rtk_sat < b.rtk_sat; }
};

// RTKLIB satno()/satsys() through the adapter, exported so gnss_io decoders and the ROS glue can fill
// SatId::rtk_sat without linking RTKLIB themselves (gnss_io takes it as a SatNoFn callback). 0 = unmapped.
LIVO_GNSS_API int satNoOf(Sys sys, int prn);
LIVO_GNSS_API SatId makeSatIdOf(Sys sys, int prn);
LIVO_GNSS_API std::string satIdStringOf(const SatId& sat);   // "G07", "C59"

// Signal codes we accept (DESIGN §12.3 freqs -> CODE_* table; the single copy of that table is
// livo_gnss/signal_table.hpp, used by gnss_io and by rtk_adapter). rtk_adapter maps these to RTKLIB CODE_*.
enum class SignalCode : uint8_t {
  Unknown = 0,
  L1C,   // GPS/QZS L1 C/A 1575.42, GLO G1 1602+0.5625k
  L2X,   // GPS/QZS L2C 1227.60
  L2C,   // GLO G2 1246+0.4375k
  L1X,   // GAL E1 1575.42
  L7X,   // GAL E5b 1207.14
  L2I,   // BDS B1I 1561.098 (RINEX C2I; selects TGD_B1I in prange)
  L7I,   // BDS B2I 1207.14
  L5X,   // GPS/QZS L5 1176.45 (accepted for GF checks only)
  L5Q,   // GAL E5a 1176.45 (GF only)
};
LIVO_GNSS_API const char* signalCodeName(SignalCode c);

// Tracking-status bits as delivered by u-blox/gnss_comm (DESIGN §12.3).
enum StatusBit : uint8_t {
  kStatusPsrValid = 1u << 0,
  kStatusCpValid = 1u << 1,
  kStatusHalfCycValid = 1u << 2,
  kStatusHalfCycSub = 1u << 3,
};

// Units of the receiver-reported std fields are judged at ingest (DESIGN §12.6-3). The raw values are
// preserved so the judgement can be re-done offline; the *_m/_hz members are the converted ones.
enum class StdUnit : uint8_t { Unknown = 0, Metres, Cycles, Hertz, Index };

// One signal (frequency slot) of one satellite in one epoch.
struct LIVO_GNSS_API SignalObs {
  double freq_hz = 0.0;               // carrier frequency as delivered (message freqs[]); cross-checked with sat2freq
  SignalCode code = SignalCode::Unknown;
  double psr_m = 0.0;                 // pseudorange [m]; 0 when absent
  double cp_cyc = 0.0;                // carrier phase [cycle], same sign convention as pseudorange (§1)
  double dopp_hz = 0.0;               // Doppler [Hz], approaching positive (range-rate = -lambda*D, §1)
  double cn0_dbhz = 0.0;              // C/N0 [dB-Hz]
  uint8_t lli = 0;                    // loss-of-lock indicator (RINEX semantics: bit0 lost, bit1 half-cycle)
  uint8_t status = 0;                 // StatusBit mask
  double psr_std_raw = 0.0;           // as delivered (unit judged later)
  double cp_std_raw = 0.0;
  double dopp_std_raw = 0.0;
  StdUnit psr_std_unit = StdUnit::Unknown;
  StdUnit cp_std_unit = StdUnit::Unknown;
  StdUnit dopp_std_unit = StdUnit::Unknown;
  double psr_std_m = 0.0;             // converted to metres (0 = unknown -> noise model floor applies)
  double cp_std_m = 0.0;              // converted to metres
  double dopp_std_hz = 0.0;           // converted to Hz

  bool psrValid() const { return (status & kStatusPsrValid) && psr_m > 0.0; }
  bool cpValid() const { return (status & kStatusCpValid) && cp_cyc != 0.0; }
  bool doppValid() const { return dopp_hz != 0.0; }
};

// One satellite in one epoch. sig[0] is the primary (L1-class) signal used for psr/dopp/TDCP; the rest
// are only used for geometry-free slip checks (DESIGN §12.3 "每星只取主信号").
struct LIVO_GNSS_API SatObs {
  SatId sat;
  GTime t_rx;                          // receiver time tag of this satellite's measurement (GPST)
  std::vector<SignalObs> sig;
  const SignalObs* primary() const { return sig.empty() ? nullptr : &sig[0]; }
};

// Where a measurement came from (which decoder path). Also used as the feedBytes() channel key.
enum class Source : uint8_t {
  GnssCommMeas = 0,      // /ublox_driver/range_meas   (gnss_comm/GnssMeasMsg, ROS1 wire bytes)
  GnssCommEphem,         // /ublox_driver/ephem        (gnss_comm/GnssEphemMsg)
  GnssCommGloEphem,      // /ublox_driver/glo_ephem    (gnss_comm/GnssGloEphemMsg)
  GnssCommIono,          // /ublox_driver/iono_params  (gnss_comm/StampedFloat64Array)
  GnssCommPvt,           // /ublox_driver/receiver_pvt (gnss_comm/GnssPVTSolnMsg)
  GnssCommTimePulse,     // /ublox_driver/time_pulse_info (diagnostics only)
  Rinex,                 // RINEX 3.x OBS/NAV files (pull source, DESIGN §12.4)
  UbxStream,             // raw UBX bytes (GnssRawStream), decoded in-process by input_ubx
  Synthetic,             // sim/ and unit tests
  Count
};
LIVO_GNSS_API const char* sourceName(Source s);

// One measurement epoch (all satellites sharing the same receiver time tag).
struct LIVO_GNSS_API ObsEpoch {
  GTime t_rx;                          // epoch receiver time (GPST, uncorrected by receiver clock bias)
  double t_record = 0.0;               // host/bag record time [s, host clock]; diagnostics only (§7)
  Source source = Source::Synthetic;
  uint32_t seq = 0;                    // decoder sequence number (monotone per source)
  std::vector<SatObs> sats;
  int countSys(Sys s) const;
};

// ---------------------------------------------------------------------------------------------
// Ephemerides (DESIGN §12.3). Field names follow RTKLIB eph_t/geph_t; times are GTime (GPST).
// toes / week are filled *per system* by ephem_store (BDS: toes = time2bdt(gpst2bdt(toe))).
// ---------------------------------------------------------------------------------------------
struct LIVO_GNSS_API KeplerEph {
  SatId sat;
  int iode = 0, iodc = 0;
  int sva = -1;                        // URA/SISA index; -1 = not derived (ura_raw carries the message value)
  double ura_raw = 0.0;                // as delivered; semantics (metres or index) judged by ingest self-check §12.6-4
  int svh = 0;                         // health (0 = healthy)
  int week = 0;                        // system week (GPS/GAL: GPS week; BDS: BDT week)
  int code = 0;                        // GPS: codes on L2; GAL: data source; BDS: data source
  int flag = 0;
  GTime toe, toc, ttr;                 // GPST
  double A = 0.0, e = 0.0, i0 = 0.0, OMG0 = 0.0, omg = 0.0, M0 = 0.0, deln = 0.0, OMGd = 0.0, idot = 0.0;
  double crc = 0.0, crs = 0.0, cuc = 0.0, cus = 0.0, cic = 0.0, cis = 0.0;
  double toes = 0.0;                   // toe in system time-of-week [s] (see ephem_store; 27 km if wrong for BDS)
  double fit = 0.0;                    // fit interval [h] (0 = unknown)
  double f0 = 0.0, f1 = 0.0, f2 = 0.0; // clock [s, s/s, s/s^2]
  std::array<double, 6> tgd{{0, 0, 0, 0, 0, 0}};  // [0]=TGD/BGD_E5aE1/TGD1, [1]=BGD_E5bE1/TGD2
  double Adot = 0.0, ndot = 0.0;
  double t_record = 0.0;               // host record time (bag) or file time; 0 = unknown
  Source source = Source::Synthetic;   // bag vs RINEX (AMENDMENTS A1 merge policy)
};

struct LIVO_GNSS_API GloEph {
  SatId sat;
  int iode = 0;
  int frq = 0;                         // frequency channel number k (-7..+6)
  int svh = 0, sva = 0, age = 0;
  double ura_raw = 0.0;
  GTime toe, tof;                      // GPST (toe = epoch of ephemerides; tof = message frame time / ttr)
  Eigen::Vector3d pos = Eigen::Vector3d::Zero();   // PZ-90 ECEF [m]
  Eigen::Vector3d vel = Eigen::Vector3d::Zero();   // [m/s]
  Eigen::Vector3d acc = Eigen::Vector3d::Zero();   // [m/s^2]
  double taun = 0.0;                   // clock bias [s]
  double gamn = 0.0;                   // relative frequency bias
  double dtaun = 0.0;                  // delay between L1 and L2 [s] (prange G1 branch, R2-5)
  double t_record = 0.0;
  Source source = Source::Synthetic;
  int flags = 0;                       // geph_t.flags (RTKLIB-EX rtklib.h:684: P/P1..P4/M status bits); U3 round trip (m6)
};

// Klobuchar broadcast parameters (GPS). BDS/GAL reuse them scaled by (f_L1/f)^2 (known gap, DESIGN §12.1).
struct LIVO_GNSS_API Klobuchar {
  std::array<double, 4> alpha{{0, 0, 0, 0}};
  std::array<double, 4> beta{{0, 0, 0, 0}};
  GTime t_valid;                       // when received / file epoch; 0 = unknown
  double t_record = 0.0;
  Source source = Source::Synthetic;
  bool valid() const { return alpha[0] != 0.0 || beta[0] != 0.0; }
};

// Receiver's own PVT solution (u-blox NAV-PVT via gnss_comm). Used by the L-PVT arm (§4.6) and self-checks.
struct LIVO_GNSS_API ReceiverPvt {
  GTime t;                             // navigation epoch (GPST)
  int fix_type = 0;                    // 0 none, 1 DR, 2 2D, 3 3D, 4 GNSS+DR, 5 time-only
  bool valid_fix = false;
  bool diff_soln = false;
  int carr_soln = 0;                   // 0 none, 1 float, 2 fixed
  int num_sv = 0;
  double lat_deg = 0.0, lon_deg = 0.0;
  double h_ell_m = 0.0, h_msl_m = 0.0; // ellipsoidal / MSL height
  double h_acc_m = 0.0, v_acc_m = 0.0; // horizontal / vertical accuracy estimates
  double p_dop = 0.0;
  Eigen::Vector3d vel_ned = Eigen::Vector3d::Zero();  // [m/s]
  double vel_acc_mps = 0.0;
  double t_record = 0.0;
  Source source = Source::Synthetic;
};

// ---------------------------------------------------------------------------------------------
// Frames (DESIGN §1): E = ECEF WGS-84; N = fixed ENU at r_0; W = FAST-LIVO2 world.
// ---------------------------------------------------------------------------------------------
struct LIVO_GNSS_API Geodetic {
  double lat_rad = 0.0;
  double lon_rad = 0.0;
  double h_m = 0.0;                    // ellipsoidal height
};

// The N frame anchor, fixed at initialisation and never changed (§1, §8 ALIGN). R_EN columns are the
// east/north/up unit vectors at r_0 (transpose of RTKLIB xyz2enu).
struct LIVO_GNSS_API AnchorFrame {
  Eigen::Vector3d r0_ecef = Eigen::Vector3d::Zero();
  Eigen::Matrix3d R_EN = Eigen::Matrix3d::Identity();
  Geodetic geo;
  GTime t_fixed;                       // GPST of the epoch that fixed the anchor
  bool valid = false;
};

// ---------------------------------------------------------------------------------------------
// Registration state ξ = (φ, ψ, t) (DESIGN §1 "配准", §2.2) as exposed to the outside.
// ---------------------------------------------------------------------------------------------
struct LIVO_GNSS_API Registration {
  Eigen::Vector2d tilt = Eigen::Vector2d::Zero();  // φ = (φ_x, φ_y) [rad]
  double yaw = 0.0;                                // ψ [rad], (-π, π]
  Eigen::Vector3d trans = Eigen::Vector3d::Zero(); // t = t_{N<-W} relative to pivot o^W [m]
  Eigen::Vector3d pivot_w = Eigen::Vector3d::Zero();  // o^W [m]
  Eigen::Matrix3d M = Eigen::Matrix3d::Identity(); // R_{N<-W} = R_z(ψ)·Exp([φ,0])·C(g)
  Eigen::Matrix<double, 6, 6> cov = Eigen::Matrix<double, 6, 6>::Zero();  // [φ(2), ψ(1), t(3)]
  bool active = false;
};

}  // namespace livo_gnss
