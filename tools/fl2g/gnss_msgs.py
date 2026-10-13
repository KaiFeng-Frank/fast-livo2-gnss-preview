# SPDX-License-Identifier: BSD-3-Clause
"""gnss_comm messages (deserialised by rosbag from the bag's own message_definition) -> plain dicts.

Fields are addressed by name (functional facts of the data format, DESIGN §12.2); nothing of gnss_comm is
copied here. Times: gnss_comm GnssTimeMsg is (week, tow) in GPST for every system (ublox_driver converts BDS/GAL
time tags to GPST; the I2 audit re-checks this against RINEX). Ephemeris dicts use RTKLIB eph_t names so
orbits.py can evaluate bag and RINEX records alike; `toes`/`week` are derived per system as the core does
(ephem_store: GPS/GAL toes = GPST tow, BDS toes = BDT tow), the raw `toe_tow`/`week` fields are kept for the audit.
"""
from . import gnsstime
from .sats import from_gnss_comm, satid

TOPIC_MEAS = "/ublox_driver/range_meas"
TOPIC_EPHEM = "/ublox_driver/ephem"
TOPIC_GLO_EPHEM = "/ublox_driver/glo_ephem"
TOPIC_IONO = "/ublox_driver/iono_params"
TOPIC_PVT = "/ublox_driver/receiver_pvt"
TOPIC_LLA = "/ublox_driver/receiver_lla"
TOPIC_TP = "/ublox_driver/time_pulse_info"
GNSS_TOPICS = (TOPIC_MEAS, TOPIC_EPHEM, TOPIC_GLO_EPHEM, TOPIC_IONO, TOPIC_PVT, TOPIC_LLA, TOPIC_TP)


def _t(tm):
    return gnsstime.gpst(tm.week, tm.tow)


def _u8(x):
    return list(bytearray(x)) if isinstance(x, (bytes, bytearray)) else [int(v) for v in x]


def meas_epoch(msg, t_record):
    """GnssMeasMsg -> {t_gpst, t_record, sats: [{sat_raw, sys, prn, sat, t_gpst, slots: [...]}], tags}."""
    sats = []
    tags = set()
    for m in msg.meas:
        sys, prn = from_gnss_comm(m.sat)
        t = _t(m.time)
        tags.add(t)
        n = len(m.freqs)
        lli, code, status = _u8(m.LLI), _u8(m.code), _u8(m.status)
        slots = []
        for k in range(n):
            g = lambda arr, conv=float: conv(arr[k]) if k < len(arr) else None
            slots.append({"freq": float(m.freqs[k]), "cn0": g(m.CN0), "lli": lli[k] if k < len(lli) else None,
                          "code": code[k] if k < len(code) else None, "psr": g(m.psr), "psr_std": g(m.psr_std),
                          "cp": g(m.cp), "cp_std": g(m.cp_std), "dopp": g(m.dopp), "dopp_std": g(m.dopp_std),
                          "status": status[k] if k < len(status) else None})
        sats.append({"sat_raw": int(m.sat), "sys": sys, "prn": prn, "sat": satid(sys, prn) if sys else "X%d" % m.sat,
                     "t_gpst": t, "week": int(m.time.week), "tow": float(m.time.tow), "slots": slots,
                     "array_lengths": (n, len(m.CN0), len(lli), len(code), len(m.psr), len(m.psr_std), len(m.cp),
                                       len(m.cp_std), len(m.dopp), len(m.dopp_std), len(status))})
    t_ep = min(tags) if tags else None
    return {"t_gpst": t_ep, "t_record": t_record, "sats": sats, "n_time_tags": len(tags)}


def kepler_eph(msg, t_record):
    sys, prn = from_gnss_comm(msg.sat)
    toe = _t(msg.toe)
    toc = _t(msg.toc)
    ttr = _t(msg.ttr)
    if sys == "C":
        _, toes = gnsstime.gpst_to_bdt_week_tow(toe)
        week_sys, _ = gnsstime.gpst_to_bdt_week_tow(toe)
    else:
        week_sys, toes = gnsstime.week_tow(toe)
    return {
        "sys": sys, "prn": prn, "sat": satid(sys, prn) if sys else "X%d" % msg.sat, "sat_raw": int(msg.sat),
        "source": "bag", "t_record": t_record,
        "toe": toe, "toc": toc, "ttr": ttr, "toes": toes, "week": week_sys,
        "raw_toe_week": int(msg.toe.week), "raw_toe_tow": float(msg.toe.tow),
        "raw_toe_tow_field": float(msg.toe_tow), "raw_week_field": int(msg.week),
        "iode": int(msg.iode), "iodc": int(msg.iodc), "svh": int(msg.health), "code": int(msg.code),
        "ura_raw": float(msg.ura),
        "A": float(msg.A), "e": float(msg.e), "i0": float(msg.i0), "omg": float(msg.omg), "OMG0": float(msg.OMG0),
        "M0": float(msg.M0), "deln": float(msg.delta_n), "OMGd": float(msg.OMG_dot), "idot": float(msg.i_dot),
        "cuc": float(msg.cuc), "cus": float(msg.cus), "crc": float(msg.crc), "crs": float(msg.crs),
        "cic": float(msg.cic), "cis": float(msg.cis),
        "af0": float(msg.af0), "af1": float(msg.af1), "af2": float(msg.af2),
        "tgd0": float(msg.tgd0), "tgd1": float(msg.tgd1),
        "Adot": float(getattr(msg, "A_dot", 0.0)), "ndot": float(getattr(msg, "n_dot", 0.0)),
    }


def glo_eph(msg, t_record):
    sys, prn = from_gnss_comm(msg.sat)
    return {
        "sys": sys, "prn": prn, "sat": satid(sys, prn) if sys else "X%d" % msg.sat, "sat_raw": int(msg.sat),
        "source": "bag", "t_record": t_record,
        "toe": _t(msg.toe), "ttr": _t(msg.ttr), "frq": int(msg.freqo), "iode": int(msg.iode),
        "svh": int(msg.health), "age": int(msg.age), "ura_raw": float(msg.ura),
        "pos": [float(msg.pos_x), float(msg.pos_y), float(msg.pos_z)],
        "vel": [float(msg.vel_x), float(msg.vel_y), float(msg.vel_z)],
        "acc": [float(msg.acc_x), float(msg.acc_y), float(msg.acc_z)],
        "taun": float(msg.tau_n), "gamn": float(msg.gamma), "dtaun": float(msg.delta_tau_n),
    }


def pvt(msg, t_record):
    return {"t_gpst": _t(msg.time), "t_record": t_record, "fix_type": int(msg.fix_type), "valid_fix": bool(msg.valid_fix),
            "diff_soln": bool(msg.diff_soln), "carr_soln": int(msg.carr_soln), "num_sv": int(msg.num_sv),
            "lat": float(msg.latitude), "lon": float(msg.longitude), "h_ell": float(msg.altitude),
            "h_msl": float(msg.height_msl), "h_acc": float(msg.h_acc), "v_acc": float(msg.v_acc), "p_dop": float(msg.p_dop),
            "vel_n": float(msg.vel_n), "vel_e": float(msg.vel_e), "vel_d": float(msg.vel_d), "vel_acc": float(msg.vel_acc)}


def iono(msg, t_record):
    st = msg.header.stamp.to_sec() if hasattr(msg, "header") else None
    return {"t_record": t_record, "stamp": st, "data": [float(v) for v in msg.data]}


def navsatfix(msg, t_record):
    return {"t_record": t_record, "stamp": msg.header.stamp.to_sec(), "seq": int(msg.header.seq),
            "frame_id": msg.header.frame_id, "status": int(msg.status.status), "service": int(msg.status.service),
            "lat": float(msg.latitude), "lon": float(msg.longitude), "alt": float(msg.altitude),
            "cov": [float(v) for v in msg.position_covariance], "cov_type": int(msg.position_covariance_type)}


def generic(msg):
    """Any genpy message -> nested dict by __slots__ (used for time_pulse_info, whose layout we only introspect)."""
    if hasattr(msg, "__slots__"):
        return {k: generic(getattr(msg, k)) for k in msg.__slots__}
    if isinstance(msg, (list, tuple)):
        return [generic(v) for v in msg]
    if isinstance(msg, (bytes, bytearray)):
        return list(bytearray(msg))
    if hasattr(msg, "to_sec"):
        return msg.to_sec()
    return msg
