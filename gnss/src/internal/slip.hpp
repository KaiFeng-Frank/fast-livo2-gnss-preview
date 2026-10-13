// SPDX-License-Identifier: BSD-3-Clause
// SlipDetector — cycle-slip detectors D1–D6 for TDCP pairs (DESIGN §10 L3):
//   D1 LLI set;  D2 status bit2/bit3 changed between the two epochs;  D3 Δt_pair > max_gap_s;
//   D4 Doppler integration |ΔΦ + (D_k1 + D_k0)Δt/2| > max(0.5 cyc, 4σ_DΔt + j_max Δt³/(12λ));
//   D5 LIVO-predicted innovation w-test + 5 cm absolute;  D6 dual-frequency geometry-free > 5 cm.
// Any detector firing ⇒ satellite dropped from the pair (0/1), event SLIP, counter per detector.
// Owner: WP-c. Tests: test_u12_gating.cpp (1/2/10 cycles and half cycles ≥5σ detected ≥99 %, false alarm α±50 %).
#pragma once

#include <cstdint>

#include "livo_gnss/config.hpp"
#include "livo_gnss/types.hpp"

namespace livo_gnss {

struct SlipInput {
  const SignalObs* k0 = nullptr;      // primary signal at pair start
  const SignalObs* k1 = nullptr;      // primary signal at pair end
  const SignalObs* k0_second = nullptr;   // second frequency (D6), may be null
  const SignalObs* k1_second = nullptr;
  double dt_s = 0.0;                  // t_k1 − t_k0
  double lambda_m = 0.0;              // primary wavelength
  double lambda2_m = 0.0;
  double sigma_dopp_hz = 0.0;         // σ_D for D4
  bool lli_between = false;           // any LLI ≠ 0 on intermediate epochs (tracked by the caller)
  double innov_pred_m = 0.0;          // D5: TDCP innovation against the LIVO prediction
  double innov_sigma_m = 0.0;         // D5: its predicted σ (from S)
  bool innov_valid = false;
  // Appended by WP-c (INTERFACE_CHANGES.md, review m5): integer-millisecond receiver clock jump k·c·1 ms
  // detected by L2 inside (k0, k1] [m]. The receiver applies it to the carrier phase as well, so ΔΦ carries
  // ≈ 1.58e6 cycles per ms; D4 removes it before integrating the Doppler (DESIGN §10 L2 "λΔΦ 扣 k·c·1 ms 后
  // TDCP 照用"). 0 when no jump.
  double clock_jump_m = 0.0;
};

struct SlipVerdict {
  bool slip = false;
  uint8_t detectors = 0;              // bit (i−1) set ⇔ Di fired
  double d4_value_cyc = 0.0, d4_threshold_cyc = 0.0;
  double d6_value_m = 0.0;
};

class SlipDetector {
 public:
  explicit SlipDetector(const Config::Slip& cfg);
  SlipVerdict check(const SlipInput& in) const;
  // D4 threshold alone (testable): max(dopp_tol_cyc, 4 σ_D Δt + j_max Δt³ /(12 λ)).
  double d4Threshold(double dt_s, double sigma_dopp_hz, double lambda_m) const;

 private:
  Config::Slip cfg_;
};

}  // namespace livo_gnss
