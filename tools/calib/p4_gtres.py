# SPDX-License-Identifier: BSD-3-Clause
"""P4 calibration (TEST_PLAN §3, AMENDMENTS A58): pseudorange and Doppler residuals of a development-set bag at the RTK
truth (A2: development sequences only), independent of the filter — usable on the occluded sequences where the filter
never activates (1–3 satellites per epoch).

  python3 tools/calib/p4_gtres.py <seq> <bag> <out_dir> --root <project root> --nav <BRDC.rnx> [--nav …]

For every /ublox_driver/range_meas epoch (pseudorange: the first epoch of each GPST second, A43) and every satellite's
primary signal: the residual of the pseudorange at the truth antenna position, v_P = P − TGD − (ρ + Sagnac − c·dts + I + T)
(receiver clock NOT removed), and of the Doppler, v_D = −λD − (e·(v_s − v_A) + Sagnac rate − c·ddts) (clock drift NOT
removed), with the same broadcast-ephemeris selection (EphSelector, RTKLIB seleph + L0 health/age), Klobuchar from the nav
file's GPS header (A38), Saastamoinen and elevation as the offline tools use (tools/fl2g). Truth: the audited GT
(tools/eval/gtbase.py: A54 time base and ATE mask; the C-RTK2HP antenna — the 0.2 m to the 9Ps antenna is ignored at
pseudorange/Doppler noise levels), GPST → estimator clock through the sequence's A18 time calibration; truth velocity =
central difference over ±0.5 s. Output <out_dir>/gt_residuals.csv. Needs the image's ROS Python (rosbag).
"""
import argparse
import csv
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "eval"))

from fl2g import geodesy, gnss_msgs, gnsslsq, obs as obs_mod, orbits, rinexnav, rosio, timecalib_yaml  # noqa: E402
from fl2g.sats import CLIGHT  # noqa: E402
from fl2g.sequences import seq_class  # noqa: E402

MAX_EPH_AGE = {"G": 7200.0, "E": 7200.0, "C": 3600.0, "R": 1800.0, "J": 7200.0}   # DESIGN §14 masks.eph_age_s
EL_MASK_DEG = 10.0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("seq")
    ap.add_argument("bag")
    ap.add_argument("out")
    ap.add_argument("--root", required=True)
    ap.add_argument("--nav", action="append", required=True)
    ap.add_argument("--calib", help="A18 time calibration (default audit/<seq>/time_calib/<seq>.yaml)")
    a = ap.parse_args(argv)
    if seq_class(a.seq) != "development":
        print("refusing %s: not a development-set sequence (AMENDMENTS A2)" % a.seq)
        return 3
    import evalcommon as ec
    import gtbase
    _, reg = ec.load_sequences()
    gt, prov, mask = gtbase.load_gt(a.root, a.seq, reg[a.seq])
    o = json.load(open(os.path.join(a.root, "audit", a.seq, "gt", "gt_audit.json")))["provenance"]["origin"]
    Mg = geodesy.enu_matrix(math.radians(o["lat_deg"]), math.radians(o["lon_deg"]))
    r0g = geodesy.pos2ecef(math.radians(o["lat_deg"]), math.radians(o["lon_deg"]), o["h_m"])
    gt_ecef = r0g + gt[:, 1:4] @ Mg
    tg = gt[:, 0]
    calib_path = a.calib or os.path.join(a.root, "audit", a.seq, "time_calib", "%s.yaml" % a.seq)
    with open(calib_path) as f:
        calib = timecalib_yaml.loads(f.read())

    def truth(t_est):
        j = int(np.clip(np.searchsorted(tg, t_est), 1, len(tg) - 1))
        if not (tg[0] <= t_est <= tg[-1]) or tg[j] - tg[j - 1] > 0.5 or mask.inside(t_est):
            return None
        f = (t_est - tg[j - 1]) / max(tg[j] - tg[j - 1], 1e-9)
        return gt_ecef[j - 1] * (1 - f) + gt_ecef[j] * f

    rnx, ion = [], None
    for p in a.nav:
        hdr, recs = rinexnav.read_nav(p)
        rnx += recs
        if ion is None and hdr.get("iono_gps_alpha") and hdr.get("iono_gps_beta"):
            ion = list(hdr["iono_gps_alpha"]) + list(hdr["iono_gps_beta"])
    rb = rosio.RawBag(a.bag)
    eps, eph, geph = [], [], []
    for topic, _dt, raw, t_rec, pytype in rb.iter_raw(topics=[gnss_msgs.TOPIC_MEAS, gnss_msgs.TOPIC_EPHEM, gnss_msgs.TOPIC_GLO_EPHEM]):
        m = rb.deserialize(pytype, raw)
        if topic == gnss_msgs.TOPIC_MEAS:
            eps.append(gnss_msgs.meas_epoch(m, t_rec))
        elif topic == gnss_msgs.TOPIC_EPHEM:
            eph.append(gnss_msgs.kepler_eph(m, t_rec))
        else:
            geph.append(gnss_msgs.glo_eph(m, t_rec))
    rb.close()
    keys = {(r["sat"], round(r["toe"])) for r in rnx}
    sel = orbits.EphSelector(list(rnx) + [r for r in eph + geph if (r["sat"], round(r["toe"])) not in keys], MAX_EPH_AGE)
    os.makedirs(a.out, exist_ok=True)
    n_rows = n_ep = 0
    last_sec = None
    with open(os.path.join(a.out, "gt_residuals.csv"), "w", newline="") as fo:
        w = csv.writer(fo)
        w.writerow(["t_gpst", "t_est", "psr_epoch", "sat", "sys", "freq_hz", "cn0", "el_deg", "az_deg", "psr_std_m", "dopp_std_hz",
                    "v_psr", "v_dopp", "var_ion", "var_trop", "var_eph", "status"])
        for e in eps:
            t = e["t_gpst"]
            if t is None:
                continue
            sec = math.floor(t)
            psr_epoch = sec != last_sec          # A43: the first epoch of each GPST second carries the pseudorange rows
            last_sec = sec
            t_est = timecalib_yaml.evaluate(calib, t)
            ra = truth(t_est)
            r1, r2 = truth(t_est + 0.5), truth(t_est - 0.5)
            if ra is None or r1 is None or r2 is None:
                continue
            va = (r1 - r2) / 1.0
            pos = geodesy.ecef2pos(ra)
            obs = obs_mod.ls_obs(e)
            st = gnsslsq.sat_states(obs, t, sel, None, need_vel=True)
            n_ep += 1
            for s in st:
                ob = s["obs"]
                r, ev = geodesy.geodist(s["rs"], ra)
                if r <= 0:
                    continue
                az, el = geodesy.satazel(pos, ev)
                if el < math.radians(EL_MASK_DEG):
                    continue
                i_l1 = geodesy.ionmodel(t, ion, pos, az, el)
                iono = i_l1 * (gnsslsq.FREQ_L1 / ob["freq"]) ** 2 if ob["freq"] > 0 else 0.0
                trop = geodesy.tropmodel(pos, el)
                v_psr = float("nan")
                if ob.get("psr_ok") and ob["sys"] != "R":
                    pc = ob["psr"] - gnsslsq._tgd_correction_m(s["eph"], ob["sys"])
                    v_psr = pc - (r - CLIGHT * s["dts"] + iono + trop)
                v_dopp = float("nan")
                if ob.get("dopp", 0.0) != 0.0 and ob["freq"] > 0 and s["vs"] is not None:
                    lam = CLIGHT / ob["freq"]
                    v_dopp = -lam * ob["dopp"] - (gnsslsq.doppler_model_rate(ev, s["rs"], s["vs"], ra, va) - CLIGHT * s["ddts"])
                sin_el = math.sin(el)
                ura = s["eph"].get("sva_m", s["eph"].get("ura_raw", 0.0)) or 0.0
                w.writerow(["%.6f" % t, "%.6f" % t_est, int(psr_epoch), ob["sat"], ob["sys"], ob["freq"], ob["cn0"], "%.4f" % math.degrees(el),
                            "%.4f" % math.degrees(az), obs_std(e, ob["sat"], "psr_std"), ob.get("dopp_std", 0.0), "%.5f" % v_psr,
                            "%.6f" % v_dopp, "%.5f" % ((0.5 * iono) ** 2), "%.5f" % ((0.3 / (sin_el + 0.1)) ** 2), "%.5f" % (ura * ura),
                            ob.get("status", 0)])
                n_rows += 1
    print("%s: %d epochs at truth, %d rows -> %s (Klobuchar %s)" % (a.seq, n_ep, n_rows, a.out, "nav header" if ion else "default"))
    return 0


def obs_std(ep, sat, key):
    for s in ep["sats"]:
        if s["sat"] == sat:
            ps = obs_mod.primary_slot(s)
            if ps:
                return ps[1].get(key) or 0.0
    return 0.0


if __name__ == "__main__":
    sys.exit(main())
