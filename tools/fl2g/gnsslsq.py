# SPDX-License-Identifier: BSD-3-Clause
"""GNSS-only least squares for the offline tools: single-epoch SPP and Doppler velocity LS.

Models follow DESIGN §1 (sign conventions of RTKLIB pntpos.c rescode and the core rows), with ONE deliberate
difference in the Doppler Sagnac-rate term:
  pseudorange  P_corr = rho + b_sys - c*dts + I*(f_L1/f)^2 + T,  rho with the Sagnac term (geodist:
               |r_s - r_r| + (omega_e/c)(x_s y_r - y_s x_r)),
               P_corr = P - c*TGD_sys (GPS/QZS TGD, GAL BGD of the clock's second frequency, BDS TGD1; §4.3)
  Doppler      -lambda*D = e.(v_s - v_r) + (omega_e/c)(v_sx y_r + x_s v_ry - v_sy x_r - y_s v_rx) + d - c*dts_dot
               (range rate positive receding; `doppler_sign=-1` evaluates the flipped hypothesis for §12.6-2).
               The Sagnac rate is the time derivative of geodist's Sagnac term (same convention as gnss_comm
               dopp_res). RTKLIB-EX 2.5.1 resdop (pntpos.c:580-581) and the formula copied into DESIGN §1 carry the
               opposite sign: against the exactly computed rate of the rotated-frame range (Earth rotation over the
               flight time, satellite at a fixed emission offset — the design's own convention) the derivative sign
               agrees to 1e-8 m/s, the resdop sign is off by up to ~5 mm/s per satellite (tools/tests/test_gnss.py,
               an independent forward model; reported to the maintainer / WP-c in INTERFACE_CHANGES).
Robustness: iterative rejection of the largest normalised residual (0/1 membership only, as §10 L1).
These are diagnostics (audit, time calibration), not the estimator: the core has its own C++ RobustSpp.
"""
import math

import numpy as np

from . import geodesy, orbits
from .sats import CLIGHT, FREQ_L1

EL_MASK_DEG = 10.0
RE_BIG = 1e7          # placeholder "height" before the first position fix: disables el/iono/trop terms
PI_HALF = math.pi / 2.0


def _tgd_correction_m(eph, sys):
    """c*TGD to subtract from the primary-signal pseudorange (single-frequency user, DESIGN §4.3)."""
    if eph is None:
        return 0.0
    if sys in ("G", "J"):
        return CLIGHT * eph.get("tgd0", 0.0)
    if sys == "E":
        # E1 single-frequency user: I/NAV clock (E5b,E1) -> BGD(E1,E5b) = tgd1; F/NAV clock (E5a,E1) -> BGD(E1,E5a) = tgd0
        return CLIGHT * (eph.get("tgd1", 0.0) if orbits.is_gal_inav(eph.get("code", 0)) else eph.get("tgd0", 0.0))
    if sys == "C":
        return CLIGHT * eph.get("tgd0", 0.0)
    return 0.0


def sat_states(obs, t_rx, selector, t_record=None, need_vel=False):
    """For each observation: selected ephemeris, transmission time, satellite position (and velocity), clock.
    Returns list of dicts (only satellites with an ephemeris)."""
    out = []
    for o in obs:
        eph = selector.select(o["sat"], t_rx, t_record)
        if eph is None:
            continue
        rho0 = o["psr"] if o.get("psr_ok") else 0.075 * CLIGHT
        t_tx = t_rx - rho0 / CLIGHT
        try:
            _, dt0 = orbits.pos_clk(eph, t_tx)
            t_tx -= dt0
            if need_vel:
                rs, vs, dts, ddts = orbits.pos_vel_clk(eph, t_tx)
            else:
                rs, dts = orbits.pos_clk(eph, t_tx)
                vs, ddts = None, None
        except (ValueError, OverflowError, ZeroDivisionError):
            continue
        out.append({"obs": o, "eph": eph, "t_tx": t_tx, "rs": rs, "vs": vs, "dts": dts, "ddts": ddts})
    return out


def _wls(H, v, w):
    W = np.sqrt(w)
    A = H * W[:, None]
    b = v * W
    N = A.T @ A
    dx = np.linalg.solve(N, A.T @ b)
    Q = np.linalg.inv(N)
    return dx, Q


def spp(obs, t_rx, selector, iono=None, x0=None, t_record=None, use_glo=False, el_mask_deg=EL_MASK_DEG,
        max_reject=6, reject_sigma=4.0, cn0_min=30.0):
    """Single-epoch SPP. obs: primary-signal dicts (sat, sys, freq, psr, psr_ok, cn0). Returns dict or None."""
    cand = [o for o in obs if o.get("psr_ok") and o.get("cn0", 0.0) >= cn0_min and (use_glo or o["sys"] != "R")]
    st = sat_states(cand, t_rx, selector, t_record)
    if len(st) < 4:
        return None
    systems = sorted({s["obs"]["sys"] if s["obs"]["sys"] != "J" else "G" for s in st})
    ref = "G" if "G" in systems else systems[0]
    isb_sys = [s for s in systems if s != ref]
    nx = 4 + len(isb_sys)
    x = np.zeros(nx)
    if x0 is not None:
        x[0:3] = x0
    excluded = set()
    result = None
    for _round in range(max_reject + 1):
        used = [s for s in st if s["obs"]["sat"] not in excluded]
        ok = False
        for it in range(12):
            rr = x[0:3]
            have_pos = np.linalg.norm(rr) > 1e6
            pos = geodesy.ecef2pos(rr) if have_pos else (0.0, 0.0, -RE_BIG)
            rows, res, wts, sats_used = [], [], [], []
            for s in used:
                o = s["obs"]
                r, e = geodesy.geodist(s["rs"], rr)
                if r <= 0.0:
                    continue
                el = PI_HALF
                ion = trop = 0.0
                if have_pos:
                    az, el = geodesy.satazel(pos, e)
                    if el < math.radians(el_mask_deg):
                        continue
                    ion = geodesy.ionmodel(t_rx, iono, pos, az, el) * (FREQ_L1 / o["freq"]) ** 2
                    trop = geodesy.tropmodel(pos, el)
                sysk = o["sys"] if o["sys"] != "J" else "G"
                pc = o["psr"] - _tgd_correction_m(s["eph"], o["sys"])
                clk = x[3] + (x[4 + isb_sys.index(sysk)] if sysk in isb_sys else 0.0)
                v = pc - (r + clk - CLIGHT * s["dts"] + ion + trop)
                h = np.zeros(nx)
                h[0:3] = -e
                h[3] = 1.0
                if sysk in isb_sys:
                    h[4 + isb_sys.index(sysk)] = 1.0
                sinel = max(math.sin(el), 0.1)
                var = 0.3 ** 2 + (0.3 / sinel) ** 2 + (0.5 * ion) ** 2 + (0.3 / (sinel + 0.1)) ** 2
                rows.append(h)
                res.append(v)
                wts.append(1.0 / var)
                sats_used.append(o["sat"])
            if len(rows) < nx:
                break
            H = np.array(rows)
            # drop ISB columns of systems with no rows this iteration
            live = [0, 1, 2, 3] + [4 + k for k in range(len(isb_sys)) if np.any(H[:, 4 + k] != 0.0)]
            try:
                dx, Q = _wls(H[:, live], np.array(res), np.array(wts))
            except np.linalg.LinAlgError:
                break
            full = np.zeros(nx)
            full[live] = dx
            x = x + full
            if np.linalg.norm(dx) < 1e-4:
                ok = True
                break
        if not ok:
            return result
        vres = np.array(res) - H[:, live] @ dx
        sig = 1.0 / np.sqrt(np.array(wts))
        wn = vres / sig
        dof = len(res) - len(live)
        rms = float(np.sqrt(np.mean(vres ** 2)))
        try:
            gdop = float(math.sqrt(np.trace(np.linalg.inv(H[:, live].T @ H[:, live]))))
        except np.linalg.LinAlgError:
            gdop = float("nan")
        result = {"rr": x[0:3].copy(), "clk_m": float(x[3]), "isb": {s: float(x[4 + k]) for k, s in enumerate(isb_sys)},
                  "n": len(res), "dof": dof, "rms_m": rms, "gdop": gdop, "sats": sats_used,
                  "excluded": sorted(excluded), "Q": Q}
        k = int(np.argmax(np.abs(wn)))
        if abs(wn[k]) > reject_sigma and dof >= 2:
            excluded.add(sats_used[k])
            continue
        return result
    return result




def doppler_rows(obs, t_rx, selector, rr, t_record=None, doppler_sign=1.0, el_mask_deg=EL_MASK_DEG, cn0_min=25.0):
    """Measurement rows (sat, e, rs, vs, ddts, z = sign*(-lambda*D), el) of one epoch for solve_doppler()."""
    cand = [o for o in obs if o.get("dopp", 0.0) != 0.0 and o.get("cn0", 0.0) >= cn0_min and o.get("freq", 0.0) > 0.0]
    st = sat_states(cand, t_rx, selector, t_record, need_vel=True)
    rr = np.asarray(rr, dtype=float)
    pos = geodesy.ecef2pos(rr)
    rows = []
    for s in st:
        o = s["obs"]
        _, e = geodesy.geodist(s["rs"], rr)
        az, el = geodesy.satazel(pos, e)
        if el < math.radians(el_mask_deg):
            continue
        lam = CLIGHT / o["freq"]
        rows.append((o["sat"], e, s["rs"], s["vs"], s["ddts"], doppler_sign * (-lam * o["dopp"]), el))
    return rows


def doppler_model_rate(e, rs, vs, rr, v_r):
    """Model range rate [m/s] of the Doppler rows (module docstring): e.(v_s - v_r) + the Sagnac rate, i.e. the time
    derivative of geodist's range at a fixed emission offset. e: unit LOS receiver->satellite (ECEF)."""
    return float(e @ (vs - v_r)) + geodesy.OMGE / CLIGHT * (vs[0] * rr[1] + rs[0] * v_r[1] - vs[1] * rr[0] - rs[1] * v_r[0])


def doppler_model_jacobian(e, rs):
    """d(model rate)/d(v_r) (the row of H for the receiver velocity; the clock drift column is +1)."""
    return [-e[0] - geodesy.OMGE / CLIGHT * rs[1], -e[1] + geodesy.OMGE / CLIGHT * rs[0], -e[2]]


def solve_doppler(rows, rr, max_reject=6, reject_sigma=4.0, sigma0=0.05, min_rows=5):
    """Velocity + common clock drift from Doppler rows (module docstring: Sagnac rate = d/dt of geodist's term):
    z = e.(v_s - v_r) + (omega_e/c)(v_sx y_r + x_s v_ry - v_sy x_r - y_s v_rx) + d - c*dts_dot."""
    rr = np.asarray(rr, dtype=float)
    if len(rows) < min_rows:
        return None
    pos = geodesy.ecef2pos(rr)
    excluded = set()
    result = None
    for _round in range(max_reject + 1):
        use = [r for r in rows if r[0] not in excluded]
        if len(use) < min_rows:
            return result
        x = np.zeros(4)
        for _it in range(3):
            H, res, wts = [], [], []
            for sat, e, rs, vs, ddts, z, el in use:
                rate = doppler_model_rate(e, rs, vs, rr, x[0:3])
                res.append(z - (rate + x[3] - CLIGHT * ddts))
                H.append(doppler_model_jacobian(e, rs) + [1.0])
                sinel = max(math.sin(el), 0.1)
                wts.append(1.0 / (sigma0 * (1.0 + 1.0 / sinel) / 2.0) ** 2)
            H = np.array(H)
            try:
                dx, Q = _wls(H, np.array(res), np.array(wts))
            except np.linalg.LinAlgError:
                return result
            x = x + dx
            if np.linalg.norm(dx) < 1e-9:
                break
        vres = np.array(res) - H @ dx
        sig = 1.0 / np.sqrt(np.array(wts))
        wn = vres / sig
        dof = len(res) - 4
        enu = geodesy.enu_matrix(pos[0], pos[1]) @ x[0:3]
        result = {"v_ecef": x[0:3].copy(), "v_enu": enu, "speed": float(np.linalg.norm(x[0:3])), "drift_mps": float(x[3]),
                  "n": len(res), "dof": dof, "rms_mps": float(np.sqrt(np.mean(vres ** 2))), "Q": Q,
                  "sats": [u[0] for u in use], "excluded": sorted(excluded)}
        k = int(np.argmax(np.abs(wn)))
        if abs(wn[k]) > reject_sigma and dof >= 2:
            excluded.add(use[k][0])
            continue
        return result
    return result


def solve_doppler_weighted(rows, sigmas, rr, drift_mps=None, max_reject=6, reject_sigma=4.0, min_rows=None):
    """Weighted Doppler LS with a sigma per row [m/s] (e.g. lambda x the receiver's dopp_std): unknowns v_r and the
    clock drift d, or v_r alone when drift_mps is given (the drift then enters the rows as a known term — how the
    time calibration removes the vertical-velocity / clock-drift collinearity of a zenith-heavy geometry). Same model
    and signs as solve_doppler (linear in v_r and d: one weighted solve per round); 0/1 rejection of the largest
    |residual/sigma| > reject_sigma while at least two redundant rows remain. Returns dict(v_ecef, v_enu, drift_mps,
    n, dof, chi2_dof, Q, sats, excluded) or None (fewer than min_rows rows: 5 with the drift, 4 without)."""
    rr = np.asarray(rr, dtype=float)
    n_unk = 3 if drift_mps is not None else 4
    min_rows = (n_unk + 1) if min_rows is None else min_rows
    if len(rows) < min_rows or len(sigmas) != len(rows):
        return None
    H, y = [], []
    for sat, e, rs, vs, ddts, z, el in rows:
        # z - (rate(v=0) + d - c*dts_dot) = J v (+ d); rate is linear in v_r (doppler_model_rate)
        y.append(z - (doppler_model_rate(e, rs, vs, rr, np.zeros(3)) - CLIGHT * ddts) - (drift_mps or 0.0))
        H.append(doppler_model_jacobian(e, rs) + ([1.0] if drift_mps is None else []))
    H, y, sig = np.array(H), np.array(y), np.asarray(sigmas, dtype=float)
    use = np.ones(len(rows), dtype=bool)
    pos = geodesy.ecef2pos(rr)
    for _round in range(max_reject + 1):
        if use.sum() < min_rows:
            return None
        w = 1.0 / sig[use] ** 2
        A = H[use] * np.sqrt(w)[:, None]
        try:
            Q = np.linalg.inv(A.T @ A)
        except np.linalg.LinAlgError:
            return None
        x = Q @ (A.T @ (y[use] * np.sqrt(w)))
        wn = (y - H @ x) / sig
        dof = int(use.sum()) - n_unk
        k = int(np.argmax(np.where(use, np.abs(wn), -1.0)))
        if abs(wn[k]) > reject_sigma and dof >= 2 and _round < max_reject:
            use[k] = False
            continue
        v = x[0:3]
        return {"v_ecef": v.copy(), "v_enu": geodesy.enu_matrix(pos[0], pos[1]) @ v,
                "drift_mps": float(x[3]) if drift_mps is None else float(drift_mps), "n": int(use.sum()), "dof": dof,
                "chi2_dof": float(np.sum(wn[use] ** 2) / dof) if dof > 0 else float("nan"), "Q": Q,
                "sats": [r[0] for r, u in zip(rows, use) if u], "excluded": [r[0] for r, u in zip(rows, use) if not u]}
    return None


def doppler_ls(obs, t_rx, selector, rr, t_record=None, doppler_sign=1.0, el_mask_deg=EL_MASK_DEG, cn0_min=25.0,
               max_reject=6, reject_sigma=4.0, sigma0=0.05):
    """Single-epoch velocity + clock drift from Doppler. rr: receiver ECEF [m] (from SPP or the receiver PVT;
    ~10 m errors change the LOS by 5e-7, i.e. < 2 mm/s). Returns dict or None."""
    rows = doppler_rows(obs, t_rx, selector, rr, t_record, doppler_sign, el_mask_deg, cn0_min)
    return solve_doppler(rows, rr, max_reject, reject_sigma, sigma0)
