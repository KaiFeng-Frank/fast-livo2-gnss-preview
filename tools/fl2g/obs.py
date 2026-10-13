# SPDX-License-Identifier: BSD-3-Clause
"""Observation helpers shared by the audit and the time calibration: primary-signal selection (§12.3 "每星只取
主信号": the L1-class signal of the freqs->code table) and the flat per-satellite records the LS solvers take."""
import math

from . import gnsstime, sats

PSR_MIN_M = 1.9e7          # DESIGN §10 L0 range gate
PSR_MAX_M = 4.5e7


def primary_slot(sat):
    """(slot index, slot dict, code, glo_k) of the first L1-class slot of a satellite, or None."""
    if not sat.get("sys"):
        return None
    for k, s in enumerate(sat["slots"]):
        code, verdict, kglo = sats.signal_from_frequency(sat["sys"], s["freq"])
        if verdict == "Accepted" and sats.is_primary(sat["sys"], code):
            return k, s, code, kglo
    return None


def ls_obs(ep):
    """Primary-signal records of one epoch: sat, sys, prn, freq, psr, psr_ok (status bit0 + L0 range), dopp, cn0,
    cp, lli, status, glo_k, dopp_std (the receiver's Doppler std in Hz, 0 when absent)."""
    out = []
    for s in ep["sats"]:
        ps = primary_slot(s)
        if ps is None:
            continue
        _k, sl, _code, kglo = ps
        psr = sl["psr"] or 0.0
        st = sl["status"] or 0
        out.append({"sat": s["sat"], "sys": s["sys"], "prn": s["prn"], "freq": sl["freq"], "psr": psr,
                    "psr_ok": bool(st & 1) and PSR_MIN_M <= psr <= PSR_MAX_M, "dopp": sl["dopp"] or 0.0,
                    "cn0": sl["cn0"] or 0.0, "cp": sl["cp"] or 0.0, "lli": sl["lli"], "status": st, "glo_k": kglo,
                    "dopp_std": sl.get("dopp_std") or 0.0})
    return out


def is_whole_second(t_gpst, tol=5e-3):
    """RETIRED rule (AMENDMENTS A43): the former §4.3 / TimeMap::isWholeSecond predicate |tow - round(tow)| < 5 ms.
    The 1 Hz rule is now the first epoch of every GPST second (one_per_second below, rows.cpp EpochDecimator)."""
    _, tow = gnsstime.week_tow(t_gpst)
    return abs(tow - round(tow)) < tol


def one_per_second(t_gpst):
    """Indices of the first epoch of every GPST second, in order: the tools' 1 Hz subsample. Unlike is_whole_second it
    does not assume that the receiver's 10 Hz grid is aligned to whole seconds (Grass01's epochs sit at x.x93 s, 7 ms
    before the second, where the 5 ms rule selects nothing)."""
    out, last = [], None
    for i, t in enumerate(t_gpst):
        _, tow = gnsstime.week_tow(t)
        sec = math.floor(tow + 1e-9)
        if sec != last:
            out.append(i)
            last = sec
    return out


def grid_offset_s(t_gpst, period=0.1):
    """Median signed offset [s] of the epochs from the nearest multiple of `period` in GPST (0 = epochs on whole tenths
    of the second; Grass01: -0.007)."""
    ph = sorted(((gnsstime.week_tow(t)[1] + 0.5 * period) % period - 0.5 * period) for t in t_gpst)
    return ph[len(ph) // 2] if ph else float("nan")

