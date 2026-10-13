#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Write the synthetic gnss_comm bag behind the unit-test fixture gnss/test/data/Synthetic_* (WP-e, A24).

The synthetic fixture complements the development-set one (Grass02_*): its *schema* is the dataset's own
(connection-header definitions + md5sums, comment-stripped, as dumped by `tools/gnss_bag_export.py msgdefs` from
the development bag Grass02; the five gnss_comm types are byte-identical in every M3DGR bag audited), its
*content* is made here to reach boundaries real data rarely shows. It follows the DESIGN §12.3 assumptions by
construction, so it proves self-consistency of code and exporter only — never a fact about the data
(gnss_io/test/fixture_discovery.hpp: real-data criteria exclude Synthetic_*). Nothing of gnss_comm is imported or
copied: rosbags builds the message classes from that text.

Content (deterministic, seed fixed):
  * ephemerides: the real broadcast records of the same-day IGS BRDC file valid at t0 (GPS, Galileo I/NAV,
    BeiDou incl. GEO C01-C05/C59-C62, GLONASS), re-expressed in the gnss_comm field conventions assumed by
    DESIGN §12.3: every GnssTimeMsg is GPST (BDS toe/toc/ttr = BDT + 14 s), A = semi-major axis,
    toe_tow/week = GPST tow/week of toe for every system (what the producer writes, BDS included: development
    bag Grass02; gnss_io ignores both), ura in metres, GLONASS tau_n = RTKLIB taun (-RINEX
    field), pos/vel/acc in m;
  * observations at 10 Hz from those orbits for a static antenna in Minhang, Shanghai (no iono/tropo, a
    receiver clock offset and drift, 0.3 m code noise), multi-frequency, u-blox-like std lattices;
  * injected boundaries: a satellite with an empty signal list, an epoch with no satellites, BDS B1C
    (1575.42 MHz, must be rejected and counted), Galileo E37 (outside RTKLIB numbering), an unknown GPS
    frequency, an off-grid GLONASS frequency, status 0/0x0F/cp-invalid combinations, a cycle slip with
    LLI=1, GLONASS channels -7/+6 in the edge list, an iono_params message with 7 values (invalid), a PVT
    without a fix, a duplicate ephemeris, an ephemeris for E37, a second E1 entry of one Galileo satellite
    (E1-C beside E1-B: same code L1X; appended after the original, or inserted before it, so "first listed
    wins" is observable).
Run inside the fastlivo2-gnss:noetic image (rosbags 0.9):
  python3 make_synthetic_bag.py --msgdef-dir DIR --brdc BRDC00IGS_R_20250030000_01D_MN.rnx --out synthetic.bag
"""
import argparse
import math
import os
import random
import sys

import numpy as np

C = 299792458.0
WEEK = 604800
LEAP = 18
MU = {'G': 3.9860050e14, 'E': 3.986004418e14, 'C': 3.986004418e14}
OMGE = {'G': 7.2921151467e-5, 'E': 7.2921151467e-5, 'C': 7.292115e-5}
MU_GLO, J2_GLO, RE_GLO, OMGE_GLO = 3.9860044e14, 1.0826257e-3, 6378136.0, 7.292115e-5
BDT0 = 820108814                     # BDT epoch in GPST seconds since 1980-01-06
FREQ = {'L1': 1575.42e6, 'L2': 1227.60e6, 'L5': 1176.45e6, 'E5b': 1207.14e6, 'B1I': 1561.098e6, 'B2I': 1207.14e6}
GC_OFFSET = {'G': 0, 'R': 32, 'E': 59, 'C': 97}


# ------------------------------------------------------------------------------------------------ RINEX NAV
def num(s):
    s = s.strip().replace('D', 'E').replace('d', 'E')
    return float(s) if s else 0.0


def gps_sec(y, mo, d, hh, mi, ss):
    import datetime
    dt = datetime.datetime(y, mo, d, hh, mi, 0)
    return (dt - datetime.datetime(1980, 1, 6)).total_seconds() + ss


def read_brdc(path):
    recs, ion = [], None
    ga = gb = None
    with open(path) as f:
        lines = f.readlines()
    i = 0
    while i < len(lines):
        ln = lines[i]
        i += 1
        if ln.startswith('GPSA'):
            ga = [num(ln[5 + 12 * k:17 + 12 * k]) for k in range(4)]
        if ln.startswith('GPSB'):
            gb = [num(ln[5 + 12 * k:17 + 12 * k]) for k in range(4)]
        if 'END OF HEADER' in ln[60:]:
            break
    if ga and gb:
        ion = ga + gb
    while i < len(lines):
        ln = lines[i]
        if not ln[:1].isalpha():
            i += 1
            continue
        letter, prn = ln[0], int(ln[1:3])
        nlines = {'G': 7, 'E': 7, 'C': 7, 'J': 7, 'I': 7, 'R': 3, 'S': 3}[letter]
        y, mo, d, hh, mi, ss = int(ln[4:8]), int(ln[9:11]), int(ln[12:14]), int(ln[15:17]), int(ln[18:20]), int(ln[21:23])
        data = [num(ln[23 + 19 * k:42 + 19 * k]) for k in range(3)]
        for j in range(nlines):
            c = lines[i + 1 + j]
            data += [num(c[4 + 19 * k:23 + 19 * k]) for k in range(4)]
        recs.append({'sys': letter, 'prn': prn, 'epoch': (y, mo, d, hh, mi, ss), 'data': data})
        i += 1 + nlines
    return recs, ion


def kepler(rec):
    """Normalised Keplerian record; all times as GPST seconds since 1980-01-06."""
    d, s = rec['data'], rec['sys']
    ep = rec['epoch']
    if s == 'C':
        week = int(d[21])
        toe = BDT0 + week * WEEK + d[11]
        toc = gps_sec(*ep) + 14.0
        ttr = BDT0 + week * WEEK + d[27]
        iodc = int(d[28])
        tgd = (d[25], d[26])
        code = 0
    else:
        week = int(d[21])
        toe = week * WEEK + d[11]
        toc = gps_sec(*ep)
        ttr = week * WEEK + d[27]
        if ttr - toe > WEEK / 2:
            ttr -= WEEK
        elif toe - ttr > WEEK / 2:
            ttr += WEEK
        iodc = int(d[26]) if s == 'G' else 0
        tgd = (d[25], 0.0) if s == 'G' else (d[25], d[26])
        code = int(d[20])
    return dict(sys=s, prn=rec['prn'], toe=toe, toc=toc, ttr=ttr, week=week, toes=d[11], af0=d[0], af1=d[1], af2=d[2],
                iode=int(d[3]), crs=d[4], deln=d[5], M0=d[6], cuc=d[7], e=d[8], cus=d[9], A=d[10] ** 2, cic=d[12],
                OMG0=d[13], cis=d[14], i0=d[15], crc=d[16], omg=d[17], OMGd=d[18], idot=d[19], code=code,
                ura=d[23], svh=int(d[24]), tgd=tgd, iodc=iodc)


def glonass(rec):
    d = rec['data']
    toe_utc = gps_sec(*rec['epoch'])
    tow_utc = toe_utc % WEEK
    return dict(sys='R', prn=rec['prn'], toe=toe_utc + LEAP, tof=(toe_utc - toe_utc % 86400) + math.fmod(d[2], 86400.0) + LEAP,
                taun=-d[0], gamn=d[1], pos=np.array([d[3], d[7], d[11]]) * 1e3, vel=np.array([d[4], d[8], d[12]]) * 1e3,
                acc=np.array([d[5], d[9], d[13]]) * 1e3, svh=int(d[6]), frq=int(d[10]), age=int(d[14]),
                iode=int(math.fmod(tow_utc + 10800.0, 86400.0) / 900.0 + 0.5))


# ------------------------------------------------------------------------------------------------ orbits
def kepler_pos(k, t):
    s = k['sys']
    mu, omge = MU[s], OMGE[s]
    tk = t - k['toe']
    n = math.sqrt(mu / k['A'] ** 3) + k['deln']
    M = k['M0'] + n * tk
    E = M
    for _ in range(30):
        E = M + k['e'] * math.sin(E)
    sinE, cosE = math.sin(E), math.cos(E)
    u = math.atan2(math.sqrt(1 - k['e'] ** 2) * sinE, cosE - k['e']) + k['omg']
    r = k['A'] * (1 - k['e'] * cosE)
    i = k['i0'] + k['idot'] * tk
    s2, c2 = math.sin(2 * u), math.cos(2 * u)
    u += k['cus'] * s2 + k['cuc'] * c2
    r += k['crs'] * s2 + k['crc'] * c2
    i += k['cis'] * s2 + k['cic'] * c2
    x, y = r * math.cos(u), r * math.sin(u)
    cosi = math.cos(i)
    if s == 'C' and (k['prn'] <= 5 or k['prn'] >= 59):   # BDS GEO (RTKLIB eph2pos)
        O = k['OMG0'] + k['OMGd'] * tk - omge * k['toes']
        sinO, cosO = math.sin(O), math.cos(O)
        xg, yg, zg = x * cosO - y * cosi * sinO, x * sinO + y * cosi * cosO, y * math.sin(i)
        sino, coso = math.sin(omge * tk), math.cos(omge * tk)
        c5, s5 = math.cos(math.radians(-5.0)), math.sin(math.radians(-5.0))
        rs = np.array([xg * coso + yg * sino * c5 + zg * sino * s5, -xg * sino + yg * coso * c5 + zg * coso * s5, -yg * s5 + zg * c5])
    else:
        O = k['OMG0'] + (k['OMGd'] - omge) * tk - omge * k['toes']
        sinO, cosO = math.sin(O), math.cos(O)
        rs = np.array([x * cosO - y * cosi * sinO, x * sinO + y * cosi * cosO, y * math.sin(i)])
    tc = t - k['toc']
    dts = k['af0'] + k['af1'] * tc + k['af2'] * tc * tc - 2.0 * math.sqrt(mu * k['A']) * k['e'] * sinE / C ** 2
    return rs, dts


def glo_deq(x, acc):
    r2 = x[0] ** 2 + x[1] ** 2 + x[2] ** 2
    r3 = r2 * math.sqrt(r2)
    omg2 = OMGE_GLO ** 2
    a = 1.5 * J2_GLO * MU_GLO * RE_GLO ** 2 / r2 / r3
    b = 5.0 * x[2] ** 2 / r2
    c = -MU_GLO / r3 - a * (1.0 - b)
    return np.array([x[3], x[4], x[5], (c + omg2) * x[0] + 2.0 * OMGE_GLO * x[4] + acc[0],
                     (c + omg2) * x[1] - 2.0 * OMGE_GLO * x[3] + acc[1], (c - 2.0 * a) * x[2] + acc[2]])


def glo_pos(g, t):
    tk = t - g['toe']
    dts = -g['taun'] + g['gamn'] * tk
    x = np.concatenate([g['pos'], g['vel']])
    while abs(tk) > 1e-9:
        h = math.copysign(min(60.0, abs(tk)), tk)
        k1 = glo_deq(x, g['acc'])
        k2 = glo_deq(x + k1 * h / 2, g['acc'])
        k3 = glo_deq(x + k2 * h / 2, g['acc'])
        k4 = glo_deq(x + k3 * h, g['acc'])
        x = x + (k1 + 2 * k2 + 2 * k3 + k4) * h / 6
        tk -= h
    return x[:3], dts


def geodetic_to_ecef(lat, lon, h):
    a, f = 6378137.0, 1 / 298.257223563
    e2 = f * (2 - f)
    sl, cl = math.sin(lat), math.cos(lat)
    N = a / math.sqrt(1 - e2 * sl * sl)
    return np.array([(N + h) * cl * math.cos(lon), (N + h) * cl * math.sin(lon), (N * (1 - e2) + h) * sl])


def elevation(rr, rs, lat, lon):
    e = rs - rr
    e /= np.linalg.norm(e)
    up = np.array([math.cos(lat) * math.cos(lon), math.cos(lat) * math.sin(lon), math.sin(lat)])
    return math.degrees(math.asin(float(e @ up)))


# ------------------------------------------------------------------------------------------------ main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--msgdef-dir', required=True)
    ap.add_argument('--brdc', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--t0', default='2025-01-03T06:00:00', help='GPST start')
    ap.add_argument('--duration', type=float, default=10.0)
    ap.add_argument('--rate', type=float, default=10.0)
    args = ap.parse_args()

    from rosbags.rosbag1 import Writer
    from rosbags.typesys import Stores, get_types_from_msg, get_typestore

    rnd = random.Random(20250103)
    y, mo, d = (int(v) for v in args.t0[:10].split('-'))
    hh, mi, ss = (int(v) for v in args.t0[11:].split(':'))
    t0 = gps_sec(y, mo, d, hh, mi, float(ss))
    lat, lon, hgt = math.radians(31.0252), math.radians(121.4360), 18.0
    rr = geodetic_to_ecef(lat, lon, hgt)
    dtr0, dtr_rate = 2.5e-6, 1.2e-8                       # receiver clock [s], [s/s]

    recs, ion = read_brdc(args.brdc)
    best = {}
    for r in recs:
        s = r['sys']
        if s in ('G', 'E', 'C'):
            k = kepler(r)
            if s == 'E' and not (k['code'] & 0x200 and k['code'] & 0x001):   # I/NAV E1-B, clock for E5b/E1
                continue
            if not (k['toe'] - 600 <= t0 + args.duration and t0 - k['toe'] <= 7200):
                continue
            key = (s, r['prn'])
            if key not in best or abs(k['toe'] - t0) < abs(best[key]['toe'] - t0):
                best[key] = k
        elif s == 'R':
            g = glonass(r)
            if abs(g['toe'] - t0) > 900:
                continue
            key = ('R', r['prn'])
            if key not in best or abs(g['toe'] - t0) < abs(best[key]['toe'] - t0):
                best[key] = g

    def state(key, t):
        e = best[key]
        return glo_pos(e, t) if key[0] == 'R' else kepler_pos(e, t)

    vis = []
    for key in sorted(best):
        rs, _ = state(key, t0)
        el = elevation(rr, rs, lat, lon)
        if el > 12.0 and best[key]['svh'] == 0:
            vis.append((key, el))
    pick = {'G': 8, 'R': 5, 'E': 6}
    chosen = []
    for s in ('G', 'R', 'E'):
        cand = sorted([v for v in vis if v[0][0] == s], key=lambda v: -v[1])
        chosen += cand[:pick[s]]
    # BeiDou: every orbit class (U4 asks for MEO, IGSO and GEO incl. C59-C62): C59+ GEOs first, then C01-C05
    # GEOs, IGSO, MEO — each by elevation.
    bds_geo_hi = lambda p: p >= 59
    bds_geo_lo = lambda p: p <= 5
    bds_igso = lambda p: p in (6, 7, 8, 9, 10, 13, 16, 31, 38, 39, 40, 56)
    bds_meo = lambda p: not (bds_geo_hi(p) or bds_geo_lo(p) or bds_igso(p))
    cand_c = [v for v in vis if v[0][0] == 'C']
    for cls, n in ((bds_geo_hi, 2), (bds_geo_lo, 1), (bds_igso, 2), (bds_meo, 4)):
        chosen += sorted([v for v in cand_c if cls(v[0][1])], key=lambda v: -v[1])[:n]
    print('visible', len(vis), 'chosen', [('%s%02d' % k, round(el, 1)) for k, el in chosen], file=sys.stderr)

    # signals per satellite (index 0 primary); ambiguities fixed per signal
    plan = {}
    for n, (key, el) in enumerate(chosen):
        s, prn = key
        if s == 'G':
            fs = [FREQ['L1']] + ([FREQ['L2']] if n % 2 == 0 else [])
        elif s == 'E':
            fs = [FREQ['L1'], FREQ['E5b']]
        elif s == 'C':
            fs = [FREQ['B1I'], FREQ['B2I']] if n % 3 else [FREQ['B1I']]
        else:
            k = best[key]['frq']
            fs = [1602e6 + k * 0.5625e6] + ([1246e6 + k * 0.4375e6] if n % 2 else [])
        plan[key] = dict(el=el, freqs=fs, amb=[rnd.randint(-2000000, 2000000) for _ in fs],
                         cn0=[round(min(52.0, 30.0 + el / 3.0) - 3.0 * j + rnd.uniform(-1, 1), 0) for j in range(len(fs))])

    # message classes from the dataset's own schema text
    store = get_typestore(Stores.ROS1_NOETIC)
    conns = {}
    for short in ('range_meas', 'ephem', 'glo_ephem', 'iono_params', 'receiver_pvt'):
        base = os.path.join(args.msgdef_dir, short)
        text = open(base + '.msgdef').read()
        mtype = open(base + '.type').read().strip()
        md5 = open(base + '.md5').read().strip()
        rt = mtype.replace('/', '/msg/', 1)
        store.register(get_types_from_msg(text, rt))
        conns[short] = (rt, text, md5)
    T = store.types
    TimeMsg = T['gnss_comm/msg/GnssTimeMsg']
    Obs, Meas = T['gnss_comm/msg/GnssObsMsg'], T['gnss_comm/msg/GnssMeasMsg']
    Eph, Glo = T['gnss_comm/msg/GnssEphemMsg'], T['gnss_comm/msg/GnssGloEphemMsg']
    Arr, Pvt = T['gnss_comm/msg/StampedFloat64Array'], T['gnss_comm/msg/GnssPVTSolnMsg']
    Header, Time = T['std_msgs/msg/Header'], T['builtin_interfaces/msg/Time']

    def gt(gps):
        w = int(gps // WEEK)
        return TimeMsg(week=w, tow=gps - w * WEEK)

    def host_ns(gps, lat_s):
        return int(round((gps - LEAP + GPST0_UNIX + lat_s) * 1e9))

    def f64(v):
        return np.array(v, dtype=np.float64)

    def u8(v):
        return np.array(v, dtype=np.uint8)

    out = []                                               # (topic, t_ns, msg)

    # ---- ephemerides at the start (+ a duplicate and an E37 one later) ----
    for key, _ in chosen:
        e = best[key]
        s, prn = key
        if s == 'R':
            m = Glo(sat=32 + prn, ttr=gt(e['tof']), toe=gt(e['toe']), freqo=e['frq'], iode=e['iode'], health=e['svh'], age=e['age'],
                    ura=0.0, pos_x=float(e['pos'][0]), pos_y=float(e['pos'][1]), pos_z=float(e['pos'][2]), vel_x=float(e['vel'][0]),
                    vel_y=float(e['vel'][1]), vel_z=float(e['vel'][2]), acc_x=float(e['acc'][0]), acc_y=float(e['acc'][1]),
                    acc_z=float(e['acc'][2]), tau_n=e['taun'], gamma=e['gamn'], delta_tau_n=0.0)
            out.append(('glo_ephem', host_ns(t0, 0.020 + 0.001 * prn), m))
        else:
            toe_gps = gt(e['toe'])                                     # toe_tow/week: GPST, as in the real bags
            m = Eph(sat=GC_OFFSET[s] + prn, ttr=gt(e['ttr']), toe=toe_gps, toc=gt(e['toc']), toe_tow=toe_gps.tow, week=toe_gps.week,
                    iode=e['iode'], iodc=e['iodc'], health=e['svh'], code=e['code'], ura=e['ura'], A=e['A'], e=e['e'], i0=e['i0'],
                    omg=e['omg'], OMG0=e['OMG0'], M0=e['M0'], delta_n=e['deln'], OMG_dot=e['OMGd'], i_dot=e['idot'], cuc=e['cuc'],
                    cus=e['cus'], crc=e['crc'], crs=e['crs'], cic=e['cic'], cis=e['cis'], af0=e['af0'], af1=e['af1'], af2=e['af2'],
                    tgd0=e['tgd'][0], tgd1=e['tgd'][1], A_dot=0.0, n_dot=0.0)
            out.append(('ephem', host_ns(t0, 0.020 + 0.001 * (GC_OFFSET[s] + prn)), m))
            if s == 'G' and len([x for x in out if x[0] == 'ephem']) == 1:
                out.append(('ephem', host_ns(t0 + 5.0, 0.031), m))                  # duplicate, later
            if s == 'E' and not any(x[0] == 'ephem' and x[2].sat == 59 + 37 for x in out):
                bad = Eph(**{f: getattr(m, f) for f in m.__dataclass_fields__ if f != '__msgtype__'})
                bad.sat = 59 + 37                                                    # E37: no RTKLIB number
                out.append(('ephem', host_ns(t0 + 2.0, 0.033), bad))
    # ---- iono: one valid, one malformed (7 values) ----
    if ion:
        out.append(('iono_params', host_ns(t0, 0.019), Arr(header=Header(seq=0, stamp=Time(sec=int(t0 - LEAP + GPST0_UNIX), nanosec=19000000),
                                                                       frame_id=''), data=f64(ion))))
        out.append(('iono_params', host_ns(t0 + 6.0, 0.019), Arr(header=Header(seq=1, stamp=Time(sec=int(t0 + 6 - LEAP + GPST0_UNIX),
                                                                                                   nanosec=0), frame_id='gnss,edge'),
                                                                  data=f64(ion[:7]))))

    # ---- observations ----
    used = {k for k, _ in chosen}
    free_g = [p for p in range(32, 0, -1) if ('G', p) not in used]      # PRNs for the non-physical edge satellites
    g_unknown, g_empty = free_g[0], free_g[1]
    # GLONASS channel edge cases (k = -7, +6) on slots that have no broadcast record that day, so a RINEX
    # reader takes their channel from the OBS header (GLONASS SLOT / FRQ #) exactly as written from these
    # observations (RTKLIB sat2freq prefers an ephemeris' channel). The off-grid frequency (never written to
    # RINEX) goes to any other unused slot.
    brdc_glo = {r['prn'] for r in recs if r['sys'] == 'R'}
    free_r = [p for p in range(1, 28) if p not in brdc_glo and ('R', p) not in used]
    if len(free_r) < 2:
        raise SystemExit('need two GLONASS slots without a broadcast record for the channel edge cases')
    r_kmin, r_kmax = free_r[0], free_r[1]
    r_offgrid = next(p for p in range(27, 0, -1) if ('R', p) not in used and p not in (r_kmin, r_kmax))
    n_ep = int(round(args.duration * args.rate))
    slip_key = next(k for k, _ in chosen if k[0] == 'G')
    w0 = int(t0 // WEEK)
    tow0 = t0 - w0 * WEEK                                   # integral: t0 is a whole second
    for j in range(n_ep):
        t = t0 + j / args.rate
        tm = TimeMsg(week=w0, tow=tow0 + j / args.rate)     # receiver tag as u-blox reports it (no 1.4e9 rounding)
        dtr = dtr0 + dtr_rate * (t - t0)
        lat_s = 0.040 + 0.004 * math.sin(j)
        if j == 37:                                         # epoch with no satellites at all
            out.append(('range_meas', host_ns(t, lat_s), Meas(meas=[])))
            continue
        meas = []
        for n, (key, _) in enumerate(chosen):
            s, prn = key
            p = plan[key]
            rs, dts = state(key, t - 0.075)
            for _ in range(3):
                rho = float(np.linalg.norm(rs - rr))
                rs, dts = state(key, t - dtr - rho / C)
            rho = float(np.linalg.norm(rs - rr)) + OMGE['G'] * (rs[0] * rr[1] - rs[1] * rr[0]) / C
            rs2, dts2 = state(key, t - dtr - rho / C + 0.5)
            rs1, dts1 = state(key, t - dtr - rho / C - 0.5)
            rate = float(np.linalg.norm(rs2 - rr) - np.linalg.norm(rs1 - rr)) + C * (dtr_rate - (dts2 - dts1))
            freqs, psr, cp, dopp, cn0, lli, st, ps, cs, ds, code = [], [], [], [], [], [], [], [], [], [], []
            for q, f in enumerate(p['freqs']):
                lam = C / f
                P = rho + C * (dtr - dts) + rnd.gauss(0.0, 0.3)
                amb = p['amb'][q]
                slip = key == slip_key and q == 0 and j == 55
                if key == slip_key and q == 0 and j >= 55:
                    amb += 7                                  # cycle slip at epoch 55, flagged by LLI
                L = (rho + C * (dtr - dts)) / lam + amb + rnd.gauss(0.0, 0.01)
                D = -rate / lam + rnd.gauss(0.0, 0.05)
                status = 0x07
                if s in ('R', 'C') and q == 0 and n % 2 == 0:
                    status = 0x0F                             # half cycle subtracted
                if n == 3 and q == len(p['freqs']) - 1:
                    status, L = 0x01, 0.0                     # carrier invalid on one signal
                if n == 5 and j % 20 == 7:
                    status = 0x03                             # half cycle not resolved
                if n == 6 and j == 12:
                    status, P, L = 0x00, 0.0, 0.0             # nothing valid but Doppler/CN0
                freqs.append(f)
                psr.append(P)
                cp.append(L)
                dopp.append(D)
                cn0.append(p['cn0'][q])
                lli.append(1 if slip else 0)
                st.append(status)
                ps.append(0.01 * 2 ** rnd.randint(2, 6))
                cs.append(0.004 * rnd.randint(1, 5))
                ds.append(0.002 * 2 ** rnd.randint(4, 8))
                code.append(q)
            meas.append(Obs(time=tm, sat=GC_OFFSET[s] + prn, freqs=f64(freqs), CN0=f64(cn0), LLI=u8(lli), code=u8(code), psr=f64(psr),
                            psr_std=f64(ps), cp=f64(cp), cp_std=f64(cs), dopp=f64(dopp), dopp_std=f64(ds), status=u8(st)))
        # --- boundary satellites (not physical; exercise the mapping rules) ---
        edge = []
        if j % 10 == 3:
            first_c = next(m for m in meas if m.sat > 97)          # BDS: add a B1C slot (rejected + counted)
            k = len(first_c.freqs)
            first_c.freqs = f64(list(first_c.freqs) + [1575.42e6])
            for fld, val in (('CN0', 38.0), ('psr', first_c.psr[0] + 0.3), ('psr_std', 0.32), ('cp', first_c.cp[0] * 1.0),
                             ('cp_std', 0.008), ('dopp', first_c.dopp[0]), ('dopp_std', 0.064)):
                setattr(first_c, fld, f64(list(getattr(first_c, fld)) + [val]))
            first_c.LLI = u8(list(first_c.LLI) + [0])
            first_c.code = u8(list(first_c.code) + [k])
            first_c.status = u8(list(first_c.status) + [0x07])
        if j % 10 == 4:
            edge.append(Obs(time=tm, sat=59 + 37, freqs=f64([1575.42e6]), CN0=f64([35.0]), LLI=u8([0]), code=u8([0]), psr=f64([2.3e7]),
                            psr_std=f64([0.64]), cp=f64([1.2e8]), cp_std=f64([0.012]), dopp=f64([-123.0]), dopp_std=f64([0.128]),
                            status=u8([7])))
            edge.append(Obs(time=tm, sat=g_unknown, freqs=f64([1381.05e6]), CN0=f64([33.0]), LLI=u8([0]), code=u8([9]), psr=f64([2.2e7]),
                            psr_std=f64([1.28]), cp=f64([0.0]), cp_std=f64([0.02]), dopp=f64([10.0]), dopp_std=f64([0.256]),
                            status=u8([1])))
        if j % 10 == 5:
            edge.append(Obs(time=tm, sat=32 + r_offgrid, freqs=f64([1602e6 + 7.5 * 0.5625e6]), CN0=f64([30.0]), LLI=u8([0]), code=u8([0]),
                            psr=f64([2.1e7]), psr_std=f64([0.64]), cp=f64([0.0]), cp_std=f64([0.02]), dopp=f64([5.0]),
                            dopp_std=f64([0.128]), status=u8([1])))
        if j % 10 == 6:
            edge.append(Obs(time=tm, sat=g_empty, freqs=f64([]), CN0=f64([]), LLI=u8([]), code=u8([]), psr=f64([]), psr_std=f64([]),
                            cp=f64([]), cp_std=f64([]), dopp=f64([]), dopp_std=f64([]), status=u8([])))
        if j == 8:   # GLONASS channel boundaries -7 and +6 on two otherwise unused slots
            for slot, k in ((r_kmin, -7), (r_kmax, 6)):
                edge.append(Obs(time=tm, sat=32 + slot, freqs=f64([1602e6 + k * 0.5625e6, 1246e6 + k * 0.4375e6]), CN0=f64([36.0, 31.0]),
                                LLI=u8([0, 0]), code=u8([0, 1]), psr=f64([2.05e7, 2.05e7 + 1.1]), psr_std=f64([0.32, 0.64]),
                                cp=f64([1.1e8, 8.5e7]), cp_std=f64([0.004, 0.008]), dopp=f64([1234.5, 961.2]), dopp_std=f64([0.064, 0.128]),
                                status=u8([7, 15])))
        if j == 9:   # secondary band listed first: the mapping must put the primary signal at sig[0]
            m0 = next(m for m in meas if 60 <= m.sat <= 97)
            order = list(range(len(m0.freqs)))[::-1]
            for fld in ('freqs', 'CN0', 'psr', 'psr_std', 'cp', 'cp_std', 'dopp', 'dopp_std'):
                setattr(m0, fld, f64([getattr(m0, fld)[i] for i in order]))
            for fld in ('LLI', 'code', 'status'):
                setattr(m0, fld, u8([getattr(m0, fld)[i] for i in order]))
        if j % 10 in (2, 8):   # a second E1 entry (E1-C beside E1-B): same code; first listed wins on every path
            m0 = next(m for m in meas if 60 <= m.sat <= 97)
            dup = {'freqs': float(m0.freqs[0]), 'CN0': float(m0.CN0[0]) - 2.0, 'psr': float(m0.psr[0]) + 0.25,
                   'psr_std': float(m0.psr_std[0]), 'cp': float(m0.cp[0]) + 0.5, 'cp_std': float(m0.cp_std[0]),
                   'dopp': float(m0.dopp[0]) + 0.125, 'dopp_std': float(m0.dopp_std[0])}
            at = len(m0.freqs) if j % 10 == 2 else 0          # after the original (kept) / before it (the copy wins)
            for fld, val in dup.items():
                v = list(getattr(m0, fld))
                v.insert(at, val)
                setattr(m0, fld, f64(v))
            for fld, val in (('LLI', 0), ('code', len(m0.code)), ('status', 0x07)):
                v = list(getattr(m0, fld))
                v.insert(at, val)
                setattr(m0, fld, u8(v))
        out.append(('range_meas', host_ns(t, lat_s), Meas(meas=meas + edge)))

        # PVT at the same rate
        lla_lat, lla_lon = math.degrees(lat) + rnd.gauss(0, 1e-6), math.degrees(lon) + rnd.gauss(0, 1e-6)
        fix = j != 21
        out.append(('receiver_pvt', host_ns(t, lat_s + 0.002),
                    Pvt(time=tm, fix_type=3 if fix else 0, valid_fix=fix, diff_soln=False, carr_soln=0,
                        num_sv=len(meas) if fix else 0, latitude=lla_lat, longitude=lla_lon, altitude=hgt + rnd.gauss(0, 0.5),
                        height_msl=hgt + 9.6, h_acc=1.2 + 0.01 * j, v_acc=2.1, p_dop=1.3, vel_n=rnd.gauss(0, 0.02), vel_e=rnd.gauss(0, 0.02),
                        vel_d=rnd.gauss(0, 0.03), vel_acc=0.08)))

    out.sort(key=lambda x: x[1])
    if os.path.exists(args.out):
        os.remove(args.out)
    with Writer(args.out) as w:
        cs = {}
        for short, (rt, text, md5) in conns.items():
            cs[short] = w.add_connection('/ublox_driver/' + short, rt, typestore=store, msgdef=text, md5sum=md5, callerid='/ublox_driver')
        for short, t_ns, msg in out:
            w.write(cs[short], t_ns, store.serialize_ros1(msg, conns[short][0]))
    print('wrote %s: %d messages' % (args.out, len(out)), file=sys.stderr)
    return 0


GPST0_UNIX = 315964800

if __name__ == '__main__':
    sys.exit(main())
