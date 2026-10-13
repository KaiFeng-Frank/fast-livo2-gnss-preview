#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""I2 data audit of one M3DGR bag (TEST_PLAN §2 I2, DESIGN §12.6 ingest self-checks 1-8 where they can be done
offline): one report per bag (markdown + JSON + CSV + plots), archived before any next step.

Checks (ids in the report): topics/types/md5/msg_def; rates and time ranges (header vs record time); (sys, freq,
code) histogram and the freqs->code table; satellites per epoch; C/N0; psr/cp/dopp availability; LLI/status;
std units (§12.6-3); phase-rate vs Doppler sign, overall and per satellite (§12.6-2); GLONASS channel vs ephemeris
and RTKLIB sat2freq vs freqs (§12.6-1, A35-C8); bag ephemerides vs same-day RINEX (field semantics, orbit/clock
agreement); URA semantics (§12.6-4, A16); ephemeris coverage bag-only vs bag+RINEX incl. C59-C62 (§12.6-8, A1); the
Klobuchar source (A38: RINEX GPSA/GPSB; bag iono_params classified as a diagnostic); time_pulse_info (A19: leap
second / time system / gross errors / host clock steps); GNSS arrival latency; Doppler-LS sign and SPP/Doppler vs
receiver PVT (§12.6-5); the lever-arm direction (§12.6-7, G13: with the ref-R replay's state_trace.txt and its A18
time calibration, --replay-dir/--time-calib; SKIP "blocked: ref-R" without them); Livox type/md5; CompressedImage
format and grey-image path equality; GT file format.

Data discipline (AMENDMENTS A2): on evaluation sequences this tool is a *format/decoding* audit. It never reads
an estimated trajectory; self-checks 5 (GNSS-only SPP / Doppler LS against the receiver's PVT, G12a/b) and 7 (the
lever arm against a replay trajectory, G13) are computed on development sequences only (SKIP otherwise, also for a
sequence name of unknown class; --replay-dir is then never opened). Owner: WP-g.
"""
import argparse
import collections
import csv
import datetime
import math
import os
import re
import struct
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402

from fl2g import TOOL_VERSION, geodesy, gnss_msgs, gnsslsq, gnsstime, orbits, report, rinexnav, robust, rosio, sats  # noqa: E402
from fl2g import obs as obs_mod  # noqa: E402
from fl2g import timecalib_yaml  # noqa: E402
from fl2g.replay import load_state_trace  # noqa: E402,F401  (G13; re-exported for the tests)
from fl2g.sequences import DEV_SEQUENCES, EVAL_SEQUENCES, seq_class  # noqa: E402,F401  (A2 classes)

# Topics the replay / GNSS path cannot do without (a missing one is a FAIL) and topics whose absence only removes an
# option (WARN, or nothing): since A38 every sequence takes its ephemerides and the GPS Klobuchar set from the RINEX
# NAV file (A1's merge is a requirement, not a supplement), so the bag's ephem/glo_ephem only add data, its iono_params
# are diagnostics, receiver_pvt feeds the L-PVT arm and self-check 5, time_pulse_info only the A19 checks.
CORE_TOPICS = ("/livox/avia/lidar", "/livox/avia/imu", "/camera/color/image_raw/compressed", gnss_msgs.TOPIC_MEAS)
OPTIONAL_TOPIC_EFFECT = {
    gnss_msgs.TOPIC_EPHEM: ("WARN", "Kepler ephemerides only from the RINEX NAV file (A1/A26)"),
    gnss_msgs.TOPIC_GLO_EPHEM: ("WARN", "GLONASS ephemerides only from the RINEX NAV file (A1/A26)"),
    gnss_msgs.TOPIC_PVT: ("WARN", "no L-PVT arm (DESIGN §4.6) and no self-check 5"),
    gnss_msgs.TOPIC_IONO: ("PASS", "no effect: the GPS Klobuchar set comes from the RINEX header (A38)"),
    gnss_msgs.TOPIC_TP: ("PASS", "the A19 time_pulse_info checks have no object"),
}
REQUIRED_TOPICS = CORE_TOPICS + tuple(OPTIONAL_TOPIC_EFFECT)
IMU_TOPIC = "/livox/avia/imu"
DEFAULT_LEVER_AVIA = (-0.31635, 0.10926, 0.4006)      # l^I, Avia IMU frame (DESIGN §14, A15; tools/m3dgr_calib.py)
RTK_TOPIC = "/mavros/global_position/raw/fix"
MAX_EPH_AGE = {"G": 7200.0, "E": 7200.0, "C": 3600.0, "R": 1800.0, "J": 7200.0}   # Config::Masks::eph_age_s (L0)
# RTKLIB 2.4.3 CODE_* numbers seen in gnss_comm data -> (signal, carrier MHz), for explaining freqs/code mismatches
RTKLIB_CODE_BAND_MHZ = {1: ("L1C/E1C", 1575.42), 12: ("L1X/E1BC", 1575.42), 14: ("L2C (GLO G2 C/A)", 0.0), 17: ("L2L", 1227.60),
                        18: ("L2X", 1227.60), 24: ("L5I/E5aI", 1176.45), 25: ("L5Q/E5aQ", 1176.45), 26: ("L5X/E5aX", 1176.45),
                        27: ("L7I (E5bI/B2I)", 1207.14), 28: ("L7Q (E5bQ)", 1207.14), 29: ("L7X (E5bX)", 1207.14),
                        47: ("L1I (B1I)", 1561.098)}
EPH_FIELDS = ("af0", "af1", "af2", "A", "e", "i0", "OMG0", "omg", "M0", "deln", "OMGd", "idot", "cuc", "cus", "crc", "crs",
              "cic", "cis", "tgd0", "tgd1", "svh", "toc")
URA_EPH = (2.4, 3.4, 4.85, 6.85, 9.65, 13.65, 24.0, 48.0, 96.0, 192.0, 384.0, 768.0, 1536.0, 3072.0, 6144.0)
URA_NOMINAL = (2.0, 2.8, 4.0, 5.7, 8.0, 11.3, 16.0, 32.0, 64.0, 128.0, 256.0, 512.0, 1024.0, 2048.0, 4096.0, 8192.0)


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0], formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("bag", help="input bag")
    p.add_argument("out_dir", help="report directory")
    p.add_argument("--nav-rinex", action="append", default=[], help="broadcast RINEX nav file(s) for coverage/URA checks")
    p.add_argument("--gt", default="", help="ground-truth file for the format check")
    p.add_argument("--max-epochs", type=int, default=0, help="limit epochs (0 = all)")
    p.add_argument("--seq", default="", help="sequence name (default: bag basename)")
    p.add_argument("--repo", default=os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                   help="repository root (for include/livox_ros_driver/CustomMsg.h)")
    p.add_argument("--image-topic", default="/camera/color/image_raw/compressed")
    p.add_argument("--lidar-topic", default="/livox/avia/lidar")
    p.add_argument("--imu-topic", default=IMU_TOPIC, help="the estimator's IMU (gyro for the lever-arm check G13)")
    p.add_argument("--replay-dir", default="",
                   help="GNSS-free ref-R replay of this bag (its state_trace.txt): enables the lever-arm check G13 "
                        "(§12.6-7) on development sequences; never read on other sequences (A2)")
    p.add_argument("--time-calib", default="",
                   help="A18 time calibration (method xcorr) of that replay, maps GNSS epochs to estimator time (G13)")
    p.add_argument("--lever-arm", type=float, nargs=3, default=DEFAULT_LEVER_AVIA,
                   help="antenna phase centre in the IMU frame [m] (DESIGN §14 / A15, Avia)")
    p.add_argument("--imu-time-offset", type=float, default=0.0,
                   help="time_offset/imu_time_offset of the replay config (config/m3dgr_avia.yaml: 0.0)")
    p.add_argument("--image-samples", type=int, default=150, help="images decoded for the pixel-path check")
    p.add_argument("--ls-rate", type=float, default=1.0, help="epochs per second evaluated by SPP / Doppler LS")
    p.add_argument("--no-plots", action="store_true")
    p.add_argument("--archive-label", default="", help="archive path printed in the report (host path of out_dir)")
    p.add_argument("--duration", type=float, default=0.0, help="only the first N seconds of the bag (debug runs; 0 = all)")
    return p


# ------------------------------------------------------------------------------------------------------------
# collection pass
# ------------------------------------------------------------------------------------------------------------
class Collected:
    def __init__(self):
        self.topic = collections.defaultdict(lambda: {"rec": [], "hdr": [], "size": [], "seq": []})
        self.epochs = []
        self.eph = []
        self.geph = []
        self.iono = []
        self.pvt = []
        self.lla = []
        self.tp = []
        self.tp_raw = []
        self.lidar_timebase = []
        self.gyro = []                      # (header stamp, wx, wy, wz) of the estimator's IMU (G13)
        self.image_formats = collections.Counter()
        self.image_samples = []
        self.decode_errors = collections.Counter()
        self.pass_seconds = 0.0


def collect(bag_path, args, conns):
    rb = rosio.RawBag(bag_path)
    hdr_topics = {}
    for c in conns:
        try:
            hdr_topics[c["topic"]] = rosio.first_field_is_header(c["datatype"], c["msg_def"])
        except (ValueError, KeyError):
            hdr_topics[c["topic"]] = False
    n_img = sum(c["count"] or 0 for c in conns if c["topic"] == args.image_topic)
    img_every = max(1, n_img // max(1, args.image_samples))
    img_k = 0
    col = Collected()
    t0 = time.time()
    n_ep = 0
    for topic, datatype, raw, t_rec, pytype in rb.iter_raw(duration=args.duration or None):
        d = col.topic[topic]
        d["rec"].append(t_rec)
        d["size"].append(len(raw))
        if hdr_topics.get(topic):
            try:
                seq, st, _frame, used = rosio.parse_header(raw)
                d["hdr"].append(st)
                d["seq"].append(seq)
            except struct.error:
                col.decode_errors[topic + ":header"] += 1
                used = None
        else:
            used = None
        try:
            if topic == gnss_msgs.TOPIC_MEAS:
                if args.max_epochs and n_ep >= args.max_epochs:
                    continue
                col.epochs.append(gnss_msgs.meas_epoch(rb.deserialize(pytype, raw), t_rec))
                n_ep += 1
            elif topic == gnss_msgs.TOPIC_EPHEM:
                col.eph.append(gnss_msgs.kepler_eph(rb.deserialize(pytype, raw), t_rec))
            elif topic == gnss_msgs.TOPIC_GLO_EPHEM:
                col.geph.append(gnss_msgs.glo_eph(rb.deserialize(pytype, raw), t_rec))
            elif topic == gnss_msgs.TOPIC_IONO:
                col.iono.append(gnss_msgs.iono(rb.deserialize(pytype, raw), t_rec))
            elif topic == gnss_msgs.TOPIC_PVT:
                col.pvt.append(gnss_msgs.pvt(rb.deserialize(pytype, raw), t_rec))
            elif topic == gnss_msgs.TOPIC_LLA:
                col.lla.append(gnss_msgs.navsatfix(rb.deserialize(pytype, raw), t_rec))
            elif topic == gnss_msgs.TOPIC_TP:
                m = rb.deserialize(pytype, raw)
                col.tp.append({"t_record": t_rec, "msg": gnss_msgs.generic(m)})
            elif topic == args.lidar_topic and used is not None:
                timebase, point_num = struct.unpack_from("<QI", raw, used)
                col.lidar_timebase.append((d["hdr"][-1], timebase * 1e-9, point_num))
            elif topic == args.imu_topic and used is not None:
                # sensor_msgs/Imu after the header: orientation (4 d) + its covariance (9 d), then angular_velocity
                col.gyro.append((d["hdr"][-1],) + struct.unpack_from("<3d", raw, used + 104))
            elif topic == args.image_topic and used is not None:
                (flen,) = struct.unpack_from("<I", raw, used)
                fmt = raw[used + 4:used + 4 + flen].decode("utf-8", errors="replace")
                col.image_formats[fmt] += 1
                if img_k % img_every == 0 and len(col.image_samples) < args.image_samples:
                    off = used + 4 + flen
                    (dlen,) = struct.unpack_from("<I", raw, off)
                    col.image_samples.append((t_rec, fmt, bytes(raw[off + 4:off + 4 + dlen])))
                img_k += 1
        except Exception as e:  # a decode failure is itself an audit finding, never a crash
            col.decode_errors["%s:%s" % (topic, type(e).__name__)] += 1
    rb.close()
    col.pass_seconds = time.time() - t0
    return col


# ------------------------------------------------------------------------------------------------------------
# helpers
# ------------------------------------------------------------------------------------------------------------
primary_slot = obs_mod.primary_slot
ls_obs = obs_mod.ls_obs
is_whole_second = obs_mod.is_whole_second
one_per_second = obs_mod.one_per_second


def on_grid(values, grid, rtol=1e-6):
    grid = np.asarray(sorted(grid), dtype=float)
    v = np.asarray(values, dtype=float)
    if v.size == 0:
        return np.zeros(0, dtype=bool)
    idx = np.clip(np.searchsorted(grid, v), 1, grid.size - 1)
    near = np.minimum(np.abs(v - grid[idx - 1]), np.abs(v - grid[idx]))
    return near <= rtol * np.maximum(np.abs(v), 1e-12) + 1e-12


def uraindex(value):
    for i, u in enumerate(URA_EPH):
        if u >= value:
            return i
    return 15


def sisa_value(idx):
    """Galileo SISA index -> metres (OS SIS ICD 5.1.12)."""
    if idx <= 49:
        return idx * 0.01
    if idx <= 74:
        return 0.5 + (idx - 50) * 0.02
    if idx <= 99:
        return 1.0 + (idx - 75) * 0.04
    if idx <= 125:
        return 2.0 + (idx - 100) * 0.16
    return float("nan")


def sisa_index(value):
    """RTKLIB-EX sisa_index (metres -> index)."""
    if value < 0.0 or value > 6.0:
        return 255
    if value <= 0.5:
        return int(value / 0.01)
    if value <= 1.0:
        return int((value - 0.5) / 0.02) + 50
    if value <= 2.0:
        return int((value - 1.0) / 0.04) + 75
    return int((value - 2.0) / 0.16) + 100


def ura_formula(n):
    """IS-GPS-200 URA index -> metres by the ICD formula (2^(1+n/2) for n <= 6, 2^(n-2) above)."""
    if n <= 6:
        return 2.0 ** (1.0 + n / 2.0)
    return 2.0 ** (n - 2.0)


def livox_repo_md5(repo):
    path = os.path.join(repo, "include", "livox_ros_driver", "CustomMsg.h")
    try:
        with open(path) as f:
            text = f.read()
    except OSError:
        return None, None, path
    m = re.search(r"struct MD5Sum< ::livox_ros_driver::CustomMsg_<ContainerAllocator> >\s*\{\s*static const char\* value\(\)\s*\{\s*return \"([0-9a-f]{32})\"", text)
    d = re.search(r"struct DataType< ::livox_ros_driver::CustomMsg_<ContainerAllocator> >\s*\{\s*static const char\* value\(\)\s*\{\s*return \"([^\"]+)\"", text)
    return (m.group(1) if m else None), (d.group(1) if d else None), path


# ------------------------------------------------------------------------------------------------------------
# analyses
# ------------------------------------------------------------------------------------------------------------
def analyse_connections(conns, checks, rep):
    rows = []
    bad = []
    known = {}
    try:  # installed ROS message packages give an independent md5 oracle for standard types
        import roslib.message
        for c in conns:
            cls = roslib.message.get_message_class(c["datatype"])
            if cls is not None:
                known[c["datatype"]] = cls._md5sum
    except Exception:
        pass
    for c in conns:
        try:
            mine = rosio.ros_md5(c["datatype"], c["msg_def"])
        except Exception as e:
            mine = "error:%s" % e
        ok = mine == c["md5sum"]
        ext = known.get(c["datatype"])
        if not ok or (ext is not None and ext != c["md5sum"]):
            bad.append(c["topic"])
        rows.append({"topic": c["topic"], "datatype": c["datatype"], "md5sum": c["md5sum"], "md5_recomputed": mine,
                     "md5_installed": ext, "callerid": c["callerid"], "latching": c["latching"], "count": c["count"]})
    rep["connections"] = rows
    n_conn = collections.Counter(r["topic"] for r in rows)
    multi = {t: n for t, n in n_conn.items() if n > 1}
    checks.add("C1", "msg_def md5 recomputed = connection md5 (and = installed ROS type md5 where installed)",
               "PASS" if not bad else "FAIL",
               "%d connections, %d mismatches%s; %d standard types also matched the installed ROS packages" %
               (len(rows), len(bad), (": " + ", ".join(bad)) if bad else "", sum(1 for r in rows if r["md5_installed"])),
               mismatches=bad, multi_connection_topics=multi)
    present = {r["topic"] for r in rows}
    missing = [t for t in REQUIRED_TOPICS if t not in present]
    rep["missing_required_topics"] = missing
    core_missing = [t for t in CORE_TOPICS if t in missing]
    opt_missing = [t for t in missing if t in OPTIONAL_TOPIC_EFFECT]
    if core_missing:
        verdict = "FAIL"
    elif any(OPTIONAL_TOPIC_EFFECT[t][0] == "WARN" for t in opt_missing):
        verdict = "WARN"
    else:
        verdict = "PASS"
    checks.add("C2", "topics of the replay/GNSS path present (DESIGN §14; since A38 the RINEX NAV file is an input of every sequence)",
               verdict, "missing: %s" % ("; ".join(
                   ["%s (required: the replay/GNSS path cannot run)" % t for t in core_missing] +
                   ["%s (%s)" % (t, OPTIONAL_TOPIC_EFFECT[t][1]) for t in opt_missing]) if missing else "none"),
               missing=missing)


def analyse_topics(col, conns, checks, rep, bag_start, bag_end):
    out = {}
    for c in conns:
        t = c["topic"]
        if t in out:
            continue
        d = col.topic.get(t)
        if not d or not d["rec"]:
            out[t] = {"datatype": c["datatype"], "n": 0}
            continue
        rec = np.array(d["rec"])
        e = {"datatype": c["datatype"], "n": int(rec.size), "rec_first": float(rec[0]), "rec_last": float(rec[-1]),
             "bytes_total": int(sum(d["size"]))}
        drec = np.diff(rec)
        if drec.size:
            span = float(rec[-1] - rec[0])
            e["rate_avg_hz"] = (rec.size - 1) / span if span > 0 else None      # bursty arrivals make medians useless
            per = (span / (rec.size - 1)) if rec.size > 1 else 0.0
            gaps = np.where(drec > max(3.0 * per, 0.25))[0]
            e["rec_gaps"] = [(float(rec[i] - bag_start), float(drec[i])) for i in gaps[:20]]
            e["rec_n_gaps"] = int(gaps.size)
        if d["hdr"]:
            hdr = np.array(d["hdr"])
            dh = np.diff(hdr)
            per_h = float(np.median(dh)) if dh.size else float("nan")
            e.update({"hdr_first": float(hdr[0]), "hdr_last": float(hdr[-1]),
                      "hdr_rate_hz": (1.0 / per_h) if per_h and per_h > 0 else None,
                      "hdr_n_gaps": int(np.sum(dh > max(3.0 * per_h, 0.25))) if dh.size else 0,
                      "hdr_nonincreasing": int(np.sum(dh <= 0)), "hdr_backwards": int(np.sum(dh < 0)),
                      "rec_minus_hdr_ms": robust.summary((rec - hdr) * 1e3)})
            s, i0 = robust.theil_sen(rec - rec[0], (rec - hdr) * 1e3, max_pairs=50000)
            e["rec_minus_hdr_drift_ms_per_s"] = s
            e["hdr_gaps"] = [(float(hdr[i] - hdr[0]), float(dh[i])) for i in np.where(dh > max(3.0 * per_h, 0.25))[0][:20]] if dh.size else []
            seqs = np.array(d["seq"], dtype=np.int64)
            e["seq_jumps"] = int(np.sum(np.diff(seqs) != 1)) if seqs.size > 1 else 0
        out[t] = e
    rep["topics"] = out
    req = [t for t in REQUIRED_TOPICS if t in out and out[t].get("n")]
    # gaps judged on header stamps where the topic has a header (arrival bursts are not gaps), else on record time
    gap_topics = {}
    for t in req:
        if (out[t].get("rate_avg_hz") or 0.0) < 1.0:
            continue                      # sparse topics (ephemerides, iono) have no nominal period
        g = out[t].get("hdr_n_gaps") if "hdr_n_gaps" in out[t] else out[t].get("rec_n_gaps")
        if g:
            gap_topics[t] = g
    def rate_txt(t):
        o = out[t]
        name = t.split("/")[-1] if t != gnss_msgs.TOPIC_MEAS else "range_meas"
        if o.get("hdr_rate_hz"):
            return "%s %.1f Hz (header), avg %.2f Hz" % (name, o["hdr_rate_hz"], o.get("rate_avg_hz") or 0.0)
        return "%s avg %.2f Hz (%d msgs)" % (name, o.get("rate_avg_hz") or 0.0, o["n"])
    gap_detail = {t: (out[t].get("hdr_gaps") or out[t].get("rec_gaps") or [])[:3] for t in gap_topics}
    checks.add("C3", "rates and time ranges (header-stamp period; record-time average)", "INFO" if not gap_topics else "WARN",
               "; ".join(rate_txt(t) for t in req) + ("; gaps (>max(3x period, 0.25 s)) as (t since start or first header [s], length [s]): %s" % {
                   t.split("/")[-1]: [(round(a, 1), round(b, 3)) for a, b in v] for t, v in gap_detail.items()} if gap_topics else "; no gaps"),
               gap_topics=gap_topics)
    hdr_rows = {t: (out[t]["rec_minus_hdr_ms"]["median"], out[t]["rec_minus_hdr_ms"]["p1"], out[t]["rec_minus_hdr_ms"]["p99"],
                    out[t]["rec_minus_hdr_drift_ms_per_s"] * 1e3)
                for t in out if out[t].get("rec_minus_hdr_ms", {}).get("n")}
    checks.add("C4", "header stamp vs record time per topic (median/p1/p99 ms, drift ppm)", "INFO",
               "; ".join("%s %.1f/%.1f/%.1f ms, %.0f ppm" % ((t,) + v) for t, v in sorted(hdr_rows.items())
                         if t in REQUIRED_TOPICS or t == RTK_TOPIC))


def analyse_epochs(col, checks, rep):
    eps = col.epochs
    if not eps:
        checks.add("G1", "GNSS epochs", "FAIL", "no range_meas epochs")
        return
    t = np.array([e["t_gpst"] for e in eps])
    dt = np.diff(t)
    multi_tag = sum(1 for e in eps if e["n_time_tags"] > 1)
    n10 = int(np.sum(np.abs(dt - 0.1) < 0.005)) if dt.size else 0
    rep["epochs"] = {"n": len(eps), "t_first_gpst": float(t[0]), "t_last_gpst": float(t[-1]),
                     "dt": robust.summary(dt), "frac_100ms": n10 / max(1, dt.size), "nonincreasing": int(np.sum(dt <= 0)),
                     "epochs_with_multiple_time_tags": multi_tag, "gaps_gt_0p15s": [(float(t[i] - t[0]), float(dt[i])) for i in np.where(dt > 0.15)[0][:20]]}
    ok = dt.size and rep["epochs"]["frac_100ms"] > 0.98 and rep["epochs"]["nonincreasing"] == 0 and multi_tag == 0
    checks.add("G1", "epoch interval 10 Hz, monotone, one time tag per epoch (§12.6-1)", "PASS" if ok else "WARN",
               "%d epochs; %.2f %% at 0.1 s; %d non-increasing; %d epochs with >1 time tag; %d gaps >0.15 s" %
               (len(eps), 100.0 * rep["epochs"]["frac_100ms"], rep["epochs"]["nonincreasing"], multi_tag,
                len(rep["epochs"]["gaps_gt_0p15s"])))
    # the 1 Hz decimation of the pseudorange/PVT rows. AMENDMENTS A43 (P3) replaced DESIGN §4.3's |tow - round(tow)| < 5 ms
    # (TimeMap::isWholeSecond, retired) with "the first epoch of every GPST second" (rows.cpp EpochDecimator), which
    # selects one epoch in every second that has data whatever the grid phase. G15 checks that rule and keeps the retired
    # rule's selection beside it (it is what made Grass01 (-7 ms) and Occlusion03/04 (+5 ms) lose every second before A43)
    secs = {}
    sec_first = set()
    for tt in t:
        tow = gnsstime.week_tow(tt)[1]
        secs.setdefault(int(round(tow)), False)
        if abs(tow - round(tow)) < 5e-3:
            secs[int(round(tow))] = True
        sec_first.add(int(math.floor(tow + 1e-9)))
    frac_rule = float(np.mean(list(secs.values()))) if secs else 0.0
    n_floor = len(sec_first)
    n_first = len(obs_mod.one_per_second(t))
    grid = obs_mod.grid_offset_s(t)
    rep["epochs"].update({"grid_offset_s": grid, "frac_seconds_with_whole_second_epoch": frac_rule,
                          "a43_seconds_with_data": n_floor, "a43_epochs_selected": n_first})
    checks.add("G15", "1 Hz decimation rule (AMENDMENTS A43: the first epoch of every GPST second; the retired DESIGN §4.3 5 ms rule reported beside it)",
               "PASS" if n_first == n_floor and n_floor > 0 else "FAIL",
               "A43 rule: %d epochs for %d GPST seconds with data; 10 Hz epoch grid offset %+.1f ms from the GPST tenths; the retired "
               "|tow - round(tow)| < 5 ms rule would select an epoch in %.1f%% of the %d seconds" % (
                   n_first, n_floor, 1e3 * grid, 100.0 * frac_rule, len(secs)))
    # array-length consistency inside each GnssObsMsg
    bad_len = sum(1 for e in eps for s in e["sats"] if len(set(s["array_lengths"])) != 1)
    unknown_sat = collections.Counter(s["sat_raw"] for e in eps for s in e["sats"] if not s["sys"])
    checks.add("G2", "per-satellite array lengths consistent; sat numbers inside the gnss_comm ranges", "PASS" if not bad_len and not unknown_sat else "FAIL",
               "%d inconsistent satellites; unknown sat numbers: %s" % (bad_len, dict(unknown_sat) or "none"))


SIGNAL_CHECKS = (("G3", "(sys, freq, code) histogram and freqs->code table verdicts (signal_table.hpp)"),
                 ("G4", "satellites per epoch (primary signal present)"),
                 ("G5", "C/N0 distribution (dB-Hz, primary signal)"),
                 ("G6", "psr / cp / dopp availability (status bits + value; with the L0 C/N0 masks)"),
                 ("G7", "LLI / status statistics"),
                 ("G8", "std unit judgement on the u-blox grids (§12.6-3)"))


def analyse_signals(col, checks, rep):
    if not col.epochs:
        # a GNSS-denial bag or a bag_filter --drop-gnss output: nothing to judge, not a crash (C2/G1 report why)
        for cid, title in SIGNAL_CHECKS:
            checks.add(cid, title, "SKIP", "no range_meas epochs in this bag")
        return
    hist = collections.Counter()
    verdicts = collections.Counter()
    primary_first = collections.Counter()
    per_sys_epoch = collections.defaultdict(list)
    cn0 = collections.defaultdict(list)
    avail = collections.defaultdict(collections.Counter)
    lli = collections.defaultdict(collections.Counter)
    status = collections.defaultdict(collections.Counter)
    std_vals = {"psr": [], "cp": [], "dopp": []}
    std_lams = []
    per_sat = collections.defaultdict(lambda: collections.Counter())
    for e in col.epochs:
        cnt = collections.Counter()
        for s in e["sats"]:
            if not s["sys"]:
                continue
            ps = primary_slot(s)
            for k, sl in enumerate(s["slots"]):
                code, verdict, kglo = sats.signal_from_frequency(s["sys"], sl["freq"])
                hist[(s["sys"], "%.4f" % (sl["freq"] / 1e6), sl["code"], code)] += 1
                verdicts[verdict] += 1
                if sl["psr_std"] is not None:
                    std_vals["psr"].append(sl["psr_std"])
                if sl["cp_std"] is not None:
                    std_vals["cp"].append(sl["cp_std"])
                    std_lams.append(sats.wavelength(sl["freq"]))
                if sl["dopp_std"] is not None:
                    std_vals["dopp"].append(sl["dopp_std"])
            primary_first["slot0" if ps is not None and ps[0] == 0 else ("other_slot" if ps else "none")] += 1
            if ps is None:
                continue
            _k, sl, code, _ = ps
            cnt[s["sys"]] += 1
            per_sat[s["sat"]]["epochs"] += 1
            st = sl["status"] or 0
            psr_ok = bool(st & 1) and sl["psr"] and 1.9e7 <= sl["psr"] <= 4.5e7
            cp_ok = bool(st & 2) and bool(sl["cp"])
            dp_ok = bool(sl["dopp"])
            c = sl["cn0"] or 0.0
            cn0[s["sys"]].append(c)
            a = avail[s["sys"]]
            a["n"] += 1
            a["psr"] += psr_ok
            a["cp"] += cp_ok
            a["dopp"] += dp_ok
            a["psr_cn0_30"] += psr_ok and c >= 30.0
            a["dopp_cn0_25"] += dp_ok and c >= 25.0
            a["cp_cn0_28"] += cp_ok and c >= 28.0
            a["half_cyc_valid"] += bool(st & 4)
            a["half_cyc_sub"] += bool(st & 8)
            lli[s["sys"]][sl["lli"]] += 1
            status[s["sys"]][st] += 1
            per_sat[s["sat"]]["psr"] += psr_ok
            per_sat[s["sat"]]["cp"] += cp_ok
            per_sat[s["sat"]]["dopp"] += dp_ok
            per_sat[s["sat"]]["cn0_sum"] += c
        for sy in "GRECJ":
            per_sys_epoch[sy].append(cnt.get(sy, 0))
    rep["signal_histogram"] = [{"sys": k[0], "freq_mhz": k[1], "code_field": k[2], "table_code": k[3], "count": v}
                               for k, v in sorted(hist.items(), key=lambda kv: (kv[0][0], kv[0][1]))]
    # the `code` field follows RTKLIB 2.4.3 CODE_* numbering; its band says which carrier the slot really is
    incons = []
    for r in rep["signal_histogram"]:
        band = RTKLIB_CODE_BAND_MHZ.get(r["code_field"])
        if band and band[1] > 0.0 and r["table_code"] == "Unknown" and abs(band[1] - float(r["freq_mhz"])) > 0.01:
            incons.append("%s code %s (%s, %.2f MHz) reported at %s MHz: %d slots" % (r["sys"], r["code_field"], band[0], band[1], r["freq_mhz"], r["count"]))
    rep["freq_code_inconsistent"] = incons
    rep["signal_verdicts"] = dict(verdicts)
    rep["primary_slot"] = dict(primary_first)
    checks.add("G3", "(sys, freq, code) histogram and freqs->code table verdicts (signal_table.hpp)",
               "PASS" if set(verdicts) <= {"Accepted", "RejectedBdsB1C"} else "WARN",
               "; ".join("%s %s MHz code=%s->%s: %d" % (r["sys"], r["freq_mhz"], r["code_field"], r["table_code"], r["count"])
                         for r in rep["signal_histogram"]) + "; verdicts %s; primary slot first: %s%s" % (
                   dict(verdicts), dict(primary_first),
                   ("; freqs field inconsistent with the code field: " + "; ".join(incons) +
                    " (the table rejects these slots; only the second-frequency GF slip check loses them)") if incons else ""))
    spe = {}
    for sy, v in per_sys_epoch.items():
        v = np.array(v)
        if v.size and v.max() > 0:
            spe[sy] = {"min": int(v.min()), "mean": float(v.mean()), "max": int(v.max())}
    tot = np.sum([np.array(per_sys_epoch[s]) for s in per_sys_epoch], axis=0) if per_sys_epoch else np.zeros(0)
    rep["sats_per_epoch"] = {"per_system": spe, "total": robust.summary(tot)}
    checks.add("G4", "satellites per epoch (primary signal present)", "INFO",
               "total min/mean/max %d/%.1f/%d; " % (int(tot.min()), float(tot.mean()), int(tot.max())) +
               ", ".join("%s %d/%.1f/%d" % (sats.SYS_NAMES[s], d["min"], d["mean"], d["max"]) for s, d in sorted(spe.items())))
    rep["cn0"] = {sats.SYS_NAMES[s]: robust.summary(v) for s, v in cn0.items()}
    checks.add("G5", "C/N0 distribution (dB-Hz, primary signal)", "INFO",
               "; ".join("%s median %.0f p5 %.0f p95 %.0f" % (s, d["median"], d["p5"], d["p95"]) for s, d in sorted(rep["cn0"].items())))
    av = {}
    for s, a in avail.items():
        n = max(1, a["n"])
        av[sats.SYS_NAMES[s]] = {k: a[k] / n for k in ("psr", "cp", "dopp", "psr_cn0_30", "dopp_cn0_25", "cp_cn0_28", "half_cyc_valid", "half_cyc_sub")}
        av[sats.SYS_NAMES[s]]["n"] = a["n"]
    rep["availability"] = av
    checks.add("G6", "psr / cp / dopp availability (status bits + value; with the L0 C/N0 masks)", "INFO",
               "; ".join("%s psr %.0f%% cp %.0f%% dopp %.0f%% (masked %.0f/%.0f/%.0f%%)" %
                         (s, 100 * d["psr"], 100 * d["cp"], 100 * d["dopp"], 100 * d["psr_cn0_30"], 100 * d["cp_cn0_28"], 100 * d["dopp_cn0_25"])
                         for s, d in sorted(av.items())))
    rep["lli_hist"] = {sats.SYS_NAMES[s]: {str(k): v for k, v in c.items()} for s, c in lli.items()}
    rep["status_hist"] = {sats.SYS_NAMES[s]: {str(k): v for k, v in c.items()} for s, c in status.items()}
    checks.add("G7", "LLI / status statistics", "INFO",
               "LLI %s; status %s" % (rep["lli_hist"], rep["status_hist"]))
    rep["per_sat"] = {k: dict(v) for k, v in per_sat.items()}
    # --- std units (§12.6-3) ---
    judge = judge_std_units(std_vals, std_lams)
    rep["std_units"] = judge
    decided = all(judge[k]["unit"] not in ("ambiguous", "none") for k in ("psr", "cp", "dopp"))
    checks.add("G8", "std unit judgement on the u-blox grids (§12.6-3)", "PASS" if decided else "FAIL",
               "psr %s (%.1f%% on 0.01*2^n m); cp %s (%.1f%% on 0.004n cyc, %.1f%% on 0.004n*lambda m, %.1f%% on 0.004n/lambda); dopp %s (%.1f%% on 0.002*2^n Hz, %.1f%% as m/s); cp_std=0.06 (cpStdev=15, u-blox 'invalid') on %.1f%% of slots" %
               (judge["psr"]["unit"], 100 * judge["psr"]["hit_metres"], judge["cp"]["unit"], 100 * judge["cp"]["hit_cycles"],
                100 * judge["cp"]["hit_metres"], 100 * judge["cp"]["hit_per_lambda"], judge["dopp"]["unit"], 100 * judge["dopp"]["hit_hz"],
                100 * judge["dopp"]["hit_mps"], 100 * judge["cp"]["frac_invalid_marker"]))


def judge_std_units(vals, lams):
    out = {}
    psr = np.array([v for v in vals["psr"] if v > 0])
    g_psr = [0.01 * 2 ** n for n in range(16)]
    hp = float(np.mean(on_grid(psr, g_psr))) if psr.size else 0.0
    out["psr"] = {"n": int(psr.size), "hit_metres": hp, "unit": "metres" if hp > 0.99 else ("none" if not psr.size else "ambiguous"),
                  "values": sorted({round(v, 6) for v in psr.tolist()})[:40]}
    cp = np.array(vals["cp"], dtype=float)
    lam = np.array(lams, dtype=float)
    ok = cp > 0
    g_cp = [0.004 * n for n in range(1, 16)]
    h_cyc = float(np.mean(on_grid(cp[ok], g_cp))) if ok.any() else 0.0
    h_m = float(np.mean(on_grid(cp[ok] / lam[ok], g_cp))) if ok.any() else 0.0
    h_pl = float(np.mean(on_grid(cp[ok] * lam[ok], g_cp))) if ok.any() else 0.0
    unit = "cycles" if h_cyc > 0.99 and h_m < 0.5 else ("metres" if h_m > 0.99 and h_cyc < 0.5 else ("none" if not ok.any() else "ambiguous"))
    out["cp"] = {"n": int(ok.sum()), "hit_cycles": h_cyc, "hit_metres": h_m, "hit_per_lambda": h_pl, "unit": unit,
                 "frac_invalid_marker": float(np.mean(np.abs(cp - 0.06) < 1e-9)) if cp.size else 0.0,
                 "values": sorted({round(v, 6) for v in cp.tolist()})[:40]}
    dp = np.array(vals["dopp"], dtype=float)
    okd = dp > 0
    g_d = [0.002 * 2 ** n for n in range(16)]
    h_hz = float(np.mean(on_grid(dp[okd], g_d))) if okd.any() else 0.0
    lam_d = np.full(dp.shape, np.nan)
    if len(lams) == dp.size:
        lam_d = np.array(lams)
    h_mps = float(np.mean(on_grid(dp[okd] / lam_d[okd], g_d))) if okd.any() and np.all(np.isfinite(lam_d[okd])) else 0.0
    unit = "hertz" if h_hz > 0.99 and h_mps < 0.5 else ("metres_per_second" if h_mps > 0.99 and h_hz < 0.5 else ("none" if not okd.any() else "ambiguous"))
    out["dopp"] = {"n": int(okd.sum()), "hit_hz": h_hz, "hit_mps": h_mps, "unit": unit,
                   "values": sorted({round(v, 6) for v in dp.tolist()})[:40]}
    return out


SIGN_MIN_PAIRS = 50          # clean 0.1 s pairs a satellite needs for its own §12.6-2 verdict
SIGN_MIN_STD = 2.0           # m/s: range-rate spread that makes corr > 0.99 decisive against ~0.1 m/s pair noise


def per_satellite_sign(by_sat):
    """§12.6-2 per satellite ('每星'): corr and slope of lambda*dPhi/dt against -lambda*D over the satellite's clean
    pairs. The correlation criterion only means something when the satellite's range rate varies well above the
    pair noise (~0.1 m/s: phase differences over 0.1 s): with a spread >= SIGN_MIN_STD, corr ~ 1 - (0.1/2)^2/2 = 0.9988
    for a correct sign and scale, and the slope is known to ~0.1 / (2 sqrt(50)) = 0.007. Satellites below that (GEO,
    short arcs) are reported, not judged."""
    out = {}
    for k, v in by_sat.items():
        a = np.asarray(v, dtype=float)
        if a.shape[0] < 20:
            continue
        x, y = a[:, 0], a[:, 1]
        slope = float(np.polyfit(x, y, 1)[0]) if np.ptp(x) > 0 else float("nan")
        corr = robust.pearson(x, y)
        spread = float(np.std(y))
        decisive = a.shape[0] >= SIGN_MIN_PAIRS and spread >= SIGN_MIN_STD
        out[k] = {"n": int(a.shape[0]), "corr": corr, "slope": slope, "range_rate_std_mps": spread, "decisive": bool(decisive),
                  "ok": bool(np.isfinite(corr) and corr > 0.99 and abs(slope - 1.0) < 0.02)}
    return out


def analyse_sign(col, checks, rep):
    """§12.6-2: lambda*dPhi/dt vs -lambda*D (corr > 0.99, slope ~ 1), overall and per satellite; sign of lambda*dPhi
    vs dP and -lambda*D*dt."""
    prev = {}
    xs, ys, clean = [], [], []
    by_sat = collections.defaultdict(list)
    sec_prev = {}
    sgn = collections.Counter()
    firsts = set(one_per_second([e["t_gpst"] for e in col.epochs]))
    for n_e, e in enumerate(col.epochs):
        whole = n_e in firsts
        for s in e["sats"]:
            if not s["sys"]:
                continue
            ps = primary_slot(s)
            if ps is None:
                continue
            _k, sl, _c, _ = ps
            st = sl["status"] or 0
            cp_ok = bool(st & 2) and bool(sl["cp"])
            psr_ok = bool(st & 1) and bool(sl["psr"])
            lam = sats.wavelength(sl["freq"])
            rec = (e["t_gpst"], sl["cp"], sl["dopp"], sl["psr"], cp_ok, psr_ok, sl["lli"] or 0, lam, sl["freq"])
            p = prev.get(s["sat"])
            if p and cp_ok and p[4] and 0 < rec[0] - p[0] <= 0.15 and p[8] == rec[8] and rec[2] and p[2]:
                dtt = rec[0] - p[0]
                x = lam * (rec[1] - p[1]) / dtt
                y = -lam * 0.5 * (rec[2] + p[2])
                xs.append(x)
                ys.append(y)
                clean.append((rec[6] & 1) == 0)
                if (rec[6] & 1) == 0:
                    by_sat[s["sat"]].append((x, y))
            prev[s["sat"]] = rec
            if whole:
                q = sec_prev.get(s["sat"])
                if q and 0.9 < rec[0] - q[0] < 1.1 and cp_ok and q[4] and psr_ok and q[5] and rec[2] and q[2]:
                    dphi = lam * (rec[1] - q[1])
                    dp = rec[3] - q[3]
                    dd = -lam * 0.5 * (rec[2] + q[2]) * (rec[0] - q[0])
                    if abs(dphi) > 5.0:
                        sgn["phi_vs_psr_same"] += (dphi > 0) == (dp > 0)
                        sgn["phi_vs_dopp_same"] += (dphi > 0) == (dd > 0)
                        sgn["n"] += 1
                sec_prev[s["sat"]] = rec
    xs, ys, clean = np.array(xs), np.array(ys), np.array(clean, dtype=bool)
    res = {"n_pairs": int(xs.size), "n_clean": int(clean.sum()), "sign_1s": dict(sgn)}
    use = clean if clean.sum() >= 100 else np.ones(xs.size, dtype=bool)
    if use.sum() >= 20:
        x, y = xs[use], ys[use]
        A = np.vstack([x, np.ones_like(x)]).T
        (slope, icpt), *_ = np.linalg.lstsq(A, y, rcond=None)
        d = x - y
        per_sat = per_satellite_sign(by_sat)
        res.update({"used": "clean (LLI bit0 = 0)" if use is clean else "all cp-valid pairs", "corr": robust.pearson(x, y),
                    "slope": float(slope), "intercept_mps": float(icpt), "median_abs_diff_mps": float(np.median(np.abs(d))),
                    "p99_abs_diff_mps": float(np.percentile(np.abs(d), 99)),
                    "frac_same_sign": float(np.mean(np.sign(x) == np.sign(y))), "per_sat": per_sat,
                    "per_sat_slope": {k: v["slope"] for k, v in per_sat.items()}})
        ok = res["corr"] > 0.99 and abs(res["slope"] - 1.0) < 0.02
        flipped = res["corr"] < -0.99 and abs(res["slope"] + 1.0) < 0.02
        sign_ok = sgn["n"] == 0 or (sgn["phi_vs_psr_same"] / sgn["n"] > 0.95 and sgn["phi_vs_dopp_same"] / sgn["n"] > 0.95)
        decisive = {k: v for k, v in per_sat.items() if v["decisive"]}
        bad_sats = sorted(k for k, v in decisive.items() if not v["ok"])
        verdict = "PASS" if ok and sign_ok and not bad_sats else "FAIL"
        ev = "%d pairs (%s): corr %.5f, slope %.4f, median |diff| %.3f m/s; per satellite (§12.6-2 '每星', decisive = >= %d clean pairs and range-rate std >= %.0f m/s): %d/%d decisive satellites with corr > 0.99 and |slope - 1| < 0.02%s (slopes %.4f..%.4f), %d not decisive; 1 s sign agreement lambda*dPhi vs dP %s, vs -lambda*D*dt %s (n=%d)%s" % (
            int(use.sum()), res["used"], res["corr"], res["slope"], res["median_abs_diff_mps"], SIGN_MIN_PAIRS, SIGN_MIN_STD,
            len(decisive) - len(bad_sats), len(decisive), (" — FAILING: %s" % ", ".join(
                "%s (corr %.4f, slope %.4f)" % (k, decisive[k]["corr"], decisive[k]["slope"]) for k in bad_sats)) if bad_sats else "",
            min([v["slope"] for v in decisive.values()] or [float("nan")]), max([v["slope"] for v in decisive.values()] or [float("nan")]),
            len(per_sat) - len(decisive),
            "%.1f%%" % (100.0 * sgn["phi_vs_psr_same"] / sgn["n"]) if sgn["n"] else "—",
            "%.1f%%" % (100.0 * sgn["phi_vs_dopp_same"] / sgn["n"]) if sgn["n"] else "—", sgn["n"],
            "; SIGN FLIPPED" if flipped else "")
    else:
        verdict, ev = "SKIP", "fewer than 20 consecutive carrier-valid pairs"
    rep["sign_check"] = res
    checks.add("G9", "carrier-phase rate vs Doppler sign/scale (§12.6-2: corr > 0.99, slope ~ 1)", verdict, ev)
    return xs, ys


def load_rinex(paths):
    header, recs = None, []
    for p in paths:
        h, r = rinexnav.read_nav(p)
        header = header or h
        recs.extend(r)
    return header, recs


def match_rinex(bag_eph, rnx_by_sat, tol=0.5):
    """RINEX record with the same satellite and toe (GPST), preferring Galileo I/NAV; also reports the toe offset
    that matched if not 0 (a +-14 s match would reveal a BDT/GPST label error)."""
    cands = rnx_by_sat.get(bag_eph["sat"], [])
    for shift in (0.0, 14.0, -14.0):
        m = [r for r in cands if abs(r["toe"] - (bag_eph["toe"] + shift)) < tol]
        if bag_eph["sys"] == "E":
            # the same navigation message type as the bag record (I/NAV vs F/NAV share a toe in IGS BRDC), else I/NAV
            same_type = [r for r in m if orbits.is_gal_inav(r["code"]) == orbits.is_gal_inav(bag_eph.get("code", 0))]
            m = same_type or [r for r in m if orbits.is_gal_inav(r["code"])] or m
        if bag_eph["sys"] != "R" and "iode" in bag_eph:
            same = [r for r in m if r.get("iode") == bag_eph["iode"]]
            m = same or m
        if m:
            return m[0], shift
    return None, None


def analyse_ephemerides(col, rnx_header, rnx, checks, rep, t_first, t_last):
    rnx_by_sat = collections.defaultdict(list)
    for r in rnx:
        rnx_by_sat[r["sat"]].append(r)
    rows = []
    shifts = collections.Counter()
    worst = collections.defaultdict(lambda: [0.0, 0.0])
    sem = collections.defaultdict(collections.Counter)
    for b in col.eph:
        r, shift = match_rinex(b, rnx_by_sat)
        if r is None:
            rows.append({"sat": b["sat"], "toe_gpst": b["toe"], "matched": False})
            continue
        shifts[(b["sys"], shift)] += 1
        dr_max, dt_max = 0.0, 0.0
        for dt in (-1800.0, 0.0, 1800.0, 3600.0):
            tt = b["toe"] + dt
            try:
                pb, cb = orbits.kepler_pos_clk(b, tt)
                pr, cr = orbits.kepler_pos_clk(r, tt)
            except (ValueError, KeyError):
                continue
            dr_max = max(dr_max, float(np.linalg.norm(pb - pr)))
            dt_max = max(dt_max, abs(cb - cr))
        w = worst[b["sys"]]
        w[0] = max(w[0], dr_max)
        w[1] = max(w[1], dt_max)
        s = sem[b["sys"]]
        s["n"] += 1
        s["toe_tow_is_system_tow"] += abs(b["raw_toe_tow_field"] - r["toes"]) < 1e-3
        s["week_is_system_week"] += b["raw_week_field"] == r["week"]
        s["week_is_gps_week"] += b["raw_week_field"] == gnsstime.week_tow(r["toe"])[0]
        s["toe_msg_is_gpst"] += abs(b["toe"] - r["toe"]) < 1e-3
        s["tgd0_eq"] += abs(b["tgd0"] - r["tgd0"]) < 1e-12
        s["tgd1_eq"] += abs(b["tgd1"] - r["tgd1"]) < 1e-12
        s["tgd0_eq_rnx_tgd1"] += abs(b["tgd0"] - r["tgd1"]) < 1e-12
        s["iode_eq"] += b["iode"] == r["iode"]
        s["svh_eq"] += b["svh"] == r["svh"]
        s["code_eq"] += b["code"] == r["code"]
        rows.append({"sat": b["sat"], "toe_gpst": b["toe"], "matched": True, "toe_shift_s": shift, "dr_max_m": dr_max,
                     "ddts_max_s": dt_max, "ura_raw": b["ura_raw"], "rnx_sva_m": r["sva_m"], "rnx_code": r["code"],
                     "bag_code": b["code"], "bag_iode": b["iode"], "rnx_iode": r["iode"], "bag_svh": b["svh"], "rnx_svh": r["svh"]})
    rep["eph_compare"] = {"rows": rows, "toe_shift_counts": {"%s:%s" % k: v for k, v in shifts.items()},
                          "worst_by_system": {k: {"dr_max_m": v[0], "ddts_max_s": v[1]} for k, v in worst.items()},
                          "semantics": {k: dict(v) for k, v in sem.items()}}
    n_match = sum(1 for r in rows if r["matched"])
    nonzero_shift = {k: v for k, v in shifts.items() if k[1] != 0.0}
    # intra-bag consistency: every copy of one (sat, toe, IODE) must carry the same parameters (E7)
    groups = collections.defaultdict(list)
    for b in col.eph:
        groups[(b["sat"], round(b["toe"]), b["iode"])].append(b)
    conflicts = []
    for key, lst in groups.items():
        sets = collections.Counter(tuple(b[f] for f in EPH_FIELDS) for b in lst)
        if len(sets) > 1:
            major = sets.most_common(1)[0][0]
            odd = [b for b in lst if tuple(b[f] for f in EPH_FIELDS) != major]
            diff = sorted({f for b in odd for i, f in enumerate(EPH_FIELDS) if b[f] != major[i]})
            conflicts.append({"sat": key[0], "toe_gpst": key[1], "iode": key[2], "copies": len(lst), "odd_copies": len(odd),
                              "odd_t_record": [b["t_record"] for b in odd], "fields": diff,
                              "odd_af1": [b["af1"] for b in odd]})
    conflict_keys = {(c["sat"], c["toe_gpst"]) for c in conflicts}
    bad_rows = [r for r in rows if r["matched"] and (r["dr_max_m"] >= 1e-3 or r["ddts_max_s"] >= 1e-11)]
    unexplained = [r for r in bad_rows if (r["sat"], round(r["toe_gpst"])) not in conflict_keys]
    # a conflicting key is "explained" when its majority copy agrees with RINEX (the odd copies are data corruption)
    good_rows = [r for r in rows if r["matched"] and r["dr_max_m"] < 1e-3 and r["ddts_max_s"] < 1e-11]
    good_keys = {(r["sat"], round(r["toe_gpst"])) for r in good_rows}
    unexplained += [r for r in bad_rows if (r["sat"], round(r["toe_gpst"])) in conflict_keys and (r["sat"], round(r["toe_gpst"])) not in good_keys]
    rep["eph_compare"]["conflicts"] = conflicts
    rep["eph_compare"]["n_outside_tier2"] = len(bad_rows)
    if not col.eph:
        verdict = "SKIP"                     # nothing to compare (Occlusion03/04, Dynamic03 have no ephem topic, A26)
    elif n_match == 0:
        verdict = "SKIP" if not rnx else "FAIL"
    else:
        verdict = "PASS" if not unexplained and not nonzero_shift else "FAIL"
    checks.add("E1", "bag Kepler ephemerides vs same-day RINEX (same sat,toe): |dr| < 1e-3 m, |ddts| < 1e-11 s (A9-3 tier 2)",
               verdict, "%d/%d matched, %d outside tier 2 (%d not explained by a corrupted copy, see E7); worst per system %s; toe label shifts %s; field semantics %s" % (
                   n_match, len(col.eph), len(bad_rows), len(unexplained), {k: "%.2e m / %.2e s" % tuple(v) for k, v in worst.items()},
                   {("%s:%+g" % k): v for k, v in shifts.items()}, {k: dict(v) for k, v in sem.items()}))
    checks.add("E7", "bag ephemeris stream consistency: all copies of one (sat, toe, IODE) identical", "PASS" if not conflicts else "WARN",
               "%d keys, %d with conflicting copies%s" % (len(groups), len(conflicts), (": " + "; ".join(
                   "%s toe %s IODE %d: %d/%d copies differ in %s (af1 %s)" % (c["sat"], gnsstime.utc_iso(gnsstime.gpst_to_utc(c["toe_gpst"])), c["iode"],
                                                                        c["odd_copies"], c["copies"], ",".join(c["fields"]), ["%.3g" % v for v in c["odd_af1"]])
                   for c in conflicts[:5]) + " — a store keyed by (sat, toe, IODE) that lets later copies overwrite would take a corrupted clock (A1: the RINEX entry wins on the same key)") if conflicts else ""))
    # GLONASS
    grow = []
    gsem = collections.Counter()
    for b in col.geph:
        cands = [r for r in rnx_by_sat.get(b["sat"], []) if abs(r["toe"] - b["toe"]) < 1.0]
        if not cands:
            grow.append({"sat": b["sat"], "matched": False})
            continue
        r = cands[0]
        dpos = float(np.max(np.abs(np.array(b["pos"]) - np.array(r["pos"]))))
        dvel = float(np.max(np.abs(np.array(b["vel"]) - np.array(r["vel"]))))
        dacc = float(np.max(np.abs(np.array(b["acc"]) - np.array(r["acc"]))))
        gsem["n"] += 1
        gsem["taun_same_sign_as_rtklib"] += abs(b["taun"] - r["taun"]) < 1e-12
        gsem["taun_opposite_sign"] += abs(b["taun"] + r["taun"]) < 1e-12 and abs(r["taun"]) > 0
        gsem["gamn_eq"] += abs(b["gamn"] - r["gamn"]) < 1e-15
        gsem["frq_eq"] += b["frq"] == r["frq"]
        gsem["pos_eq_1mm"] += dpos < 1e-3
        try:
            pb, cb = orbits.glo_pos_clk(b, b["toe"] + 600.0)
            pr, cr = orbits.glo_pos_clk(r, r["toe"] + 600.0)
            dr = float(np.linalg.norm(pb - pr))
            ddt = abs(cb - cr)
        except Exception:
            dr, ddt = float("nan"), float("nan")
        grow.append({"sat": b["sat"], "matched": True, "dpos_m": dpos, "dvel_mps": dvel, "dacc": dacc, "dr_600s_m": dr, "ddts_600s_s": ddt,
                     "bag_frq": b["frq"], "rnx_frq": r["frq"], "bag_taun": b["taun"], "rnx_taun": r["taun"], "ura_raw": b["ura_raw"]})
    rep["glo_eph_compare"] = {"rows": grow, "semantics": dict(gsem)}
    gm = [g for g in grow if g["matched"]]
    if gm:
        worst_g = max(g["dr_600s_m"] for g in gm)
        ok = worst_g < 1e-3 and gsem["taun_same_sign_as_rtklib"] == gsem["n"] and gsem["frq_eq"] == gsem["n"]
        checks.add("E2", "bag GLONASS ephemerides vs RINEX (pos/vel/acc units, tau_n sign, channel)", "PASS" if ok else "FAIL",
                   "%d/%d matched; worst |dr| at toe+600 s %.2e m; semantics %s" % (len(gm), len(col.geph), worst_g, dict(gsem)))
    else:
        checks.add("E2", "bag GLONASS ephemerides vs RINEX", "SKIP" if not col.geph or not rnx else "FAIL",
                   "%d bag GLONASS ephemerides, none matched a RINEX record" % len(col.geph))
    # --- URA semantics (§12.6-4, A16) ---
    ura_semantics(rows, col, checks, rep)
    # --- adjacent RINEX records continuity: validates the orbit code on independent data ---
    cont = []
    for sat, lst in rnx_by_sat.items():
        if sat[0] not in "GECJ":
            continue
        lst = sorted([r for r in lst if r["svh"] == 0 and (sat[0] != "E" or orbits.is_gal_inav(r["code"]))], key=lambda r: r["toe"])
        for a, b in zip(lst, lst[1:]):
            if not (t_first - 3 * 3600 <= b["toe"] <= t_last + 3 * 3600) or not (0 < b["toe"] - a["toe"] <= 7200.0):
                continue
            tm = 0.5 * (a["toe"] + b["toe"])
            pa, _ = orbits.kepler_pos_clk(a, tm)
            pb, _ = orbits.kepler_pos_clk(b, tm)
            cont.append((sat, float(np.linalg.norm(pa - pb))))
    rep["rinex_continuity"] = {"n": len(cont), "by_system": {s: robust.summary([d for k, d in cont if k[0] == s]) for s in "GECJ" if any(k[0] == s for k, _ in cont)}}
    if cont:
        bysys = rep["rinex_continuity"]["by_system"]
        ok = all(v["p95"] < 10.0 for v in bysys.values())
        checks.add("E3", "orbit code self-check: adjacent RINEX records agree at the midpoint (p95 < 10 m per system)",
                   "PASS" if ok else "FAIL", "; ".join("%s median %.2f m p95 %.2f m (n=%d)" % (s, v["median"], v["p95"], v["n"]) for s, v in sorted(bysys.items())))


def ura_semantics(rows, col, checks, rep):
    per = collections.defaultdict(collections.Counter)
    samples = collections.defaultdict(list)
    for r in rows:
        if not r.get("matched") or r.get("rnx_sva_m") is None:
            continue
        sysc = r["sat"][0]
        u, m = r["ura_raw"], r["rnx_sva_m"]
        c = per[sysc]
        c["n"] += 1
        c["metres_exact"] += abs(u - m) <= 1e-6 * max(1.0, abs(m))
        c["metres_approx"] += abs(u - m) <= 0.03 * max(1.0, abs(m))
        isint = abs(u - round(u)) < 1e-9 and u >= 0
        if sysc == "E":
            c["index"] += isint and (abs(sisa_value(int(round(u))) - m) < 0.005 or sisa_index(m) == int(round(u)))
        else:
            c["index"] += isint and 0 <= int(round(u)) <= 15 and (uraindex(m) == int(round(u)) or abs(URA_NOMINAL[int(round(u))] - m) < 1e-6)
            c["icd_formula_metres"] += isint is False and any(abs(u - ura_formula(n)) < 1e-6 for n in range(16))
        if len(samples[sysc]) < 8:
            samples[sysc].append((u, m))
    decision = {}
    for sysc, c in per.items():
        n = c["n"]
        if n == 0:
            continue
        # one hypothesis must explain >= 95 % of the matched records while the other explains <= 50 %
        fm, fa, fi = c["metres_exact"] / n, c["metres_approx"] / n, c["index"] / n
        if fm >= 0.95 and fi <= 0.5:
            decision[sysc] = "metres"
        elif fi >= 0.95 and fa <= 0.5:
            decision[sysc] = "index"
        elif fa >= 0.95 and fi <= 0.5:
            decision[sysc] = "metres"
        else:
            decision[sysc] = "ambiguous"
    overall = set(decision.values())
    rep["ura_semantics"] = {"per_system_counts": {k: dict(v) for k, v in per.items()}, "decision": decision,
                            "samples_bag_vs_rinex": {k: v for k, v in samples.items()},
                            "config_value": overall.pop() if len(overall) == 1 and "ambiguous" not in decision.values() else None,
                            "glo_ura_raw_values": sorted({g["ura_raw"] for g in col.geph})}
    if not per:
        checks.add("E4", "URA semantics: bag ura vs same-day RINEX SV accuracy (§12.6-4, A16)", "SKIP",
                   "no bag ephemeris matched a RINEX record")
        return
    cv = rep["ura_semantics"]["config_value"]
    checks.add("E4", "URA semantics: bag ura vs same-day RINEX SV accuracy (§12.6-4, A16)",
               "PASS" if cv else "FAIL",
               "decision per system %s -> gnss.ephem.ura_semantics: %s; counts %s; samples (bag, rinex m) %s" %
               (decision, cv or "UNDECIDED (per-system semantics differ or ambiguous)",
                {k: dict(v) for k, v in per.items()}, dict(samples)))


def analyse_coverage(col, rnx, checks, rep, out_dir, plots):
    if not col.epochs:
        checks.add("E5", "ephemeris coverage timeline: bag-only (causal by record time) vs bag+RINEX (A1, §12.6-8)", "SKIP",
                   "no range_meas epochs in this bag")
        return
    bag_recs = [dict(r) for r in col.eph] + [dict(r) for r in col.geph]
    sel_bag = orbits.EphSelector(bag_recs, MAX_EPH_AGE, causal=True)
    rnx_keys = {(r["sat"], round(r["toe"])) for r in rnx}
    merged = list(rnx) + [r for r in bag_recs if (r["sat"], round(r["toe"])) not in rnx_keys]
    sel_all = orbits.EphSelector(merged, MAX_EPH_AGE, causal=False)
    sat_rows = collections.defaultdict(lambda: collections.Counter())
    series = []
    first5 = {"bag_only": None, "bag_rinex": None}
    geo = collections.Counter()
    firsts = set(one_per_second([e["t_gpst"] for e in col.epochs]))
    for n_e, e in enumerate(col.epochs):
        if n_e not in firsts:
            continue
        nb = na = nobs = 0
        per_sys = collections.Counter()
        for o in ls_obs(e):
            if not (o["psr_ok"] or o["dopp"]):
                continue
            nobs += 1
            b = sel_bag.select(o["sat"], e["t_gpst"], e["t_record"]) is not None
            a = sel_all.select(o["sat"], e["t_gpst"]) is not None
            nb += b
            na += a
            sr = sat_rows[o["sat"]]
            sr["obs"] += 1
            sr["bag"] += b
            sr["all"] += a
            per_sys[(o["sys"], "obs")] += 1
            per_sys[(o["sys"], "bag")] += b
            per_sys[(o["sys"], "all")] += a
            if o["sys"] == "C" and sats.is_bds_geo(o["prn"]):
                geo[o["sat"]] += 1
        series.append((e["t_gpst"], nobs, nb, na))
        if first5["bag_only"] is None and nb >= 5:
            first5["bag_only"] = e["t_gpst"]
        if first5["bag_rinex"] is None and na >= 5:
            first5["bag_rinex"] = e["t_gpst"]
    t0 = col.epochs[0]["t_gpst"] if col.epochs else 0.0
    sysc = collections.defaultdict(dict)
    tot = collections.Counter()
    for sat, c in sat_rows.items():
        s = sat[0]
        for k in ("obs", "bag", "all"):
            sysc[s][k] = sysc[s].get(k, 0) + c[k]
            tot[k] += c[k]
    cov = {"sat_epoch_fraction": {"bag_only": tot["bag"] / max(1, tot["obs"]), "bag_rinex": tot["all"] / max(1, tot["obs"])},
           "per_system": {sats.SYS_NAMES[s]: {"bag_only": v["bag"] / max(1, v["obs"]), "bag_rinex": v["all"] / max(1, v["obs"]), "sat_epochs": v["obs"]} for s, v in sysc.items()},
           "first_epoch_ge5_healthy_eph_s": {k: (v - t0 if v is not None else None) for k, v in first5.items()},
           "frac_epochs_ge5": {"bag_only": float(np.mean([s[2] >= 5 for s in series])) if series else 0.0,
                               "bag_rinex": float(np.mean([s[3] >= 5 for s in series])) if series else 0.0},
           "bds_geo_observed": dict(geo),
           "bds_geo_with_eph": {s: sat_rows[s]["all"] for s in geo},
           "sats_never_covered_bag_only": sorted(s for s, c in sat_rows.items() if c["bag"] == 0),
           "sats_never_covered_bag_rinex": sorted(s for s, c in sat_rows.items() if c["all"] == 0)}
    rep["eph_coverage"] = cov
    with open(os.path.join(out_dir, "eph_coverage.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["sat", "obs_epochs_1hz", "with_eph_bag_only", "with_eph_bag_rinex"])
        for sat in sorted(sat_rows):
            c = sat_rows[sat]
            w.writerow([sat, c["obs"], c["bag"], c["all"]])
    with open(os.path.join(out_dir, "eph_coverage_timeline.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_gpst", "t_rel_s", "sats_observed", "with_eph_bag_only", "with_eph_bag_rinex"])
        for s in series:
            w.writerow(["%.3f" % s[0], "%.1f" % (s[0] - t0), s[1], s[2], s[3]])
    checks.add("E5", "ephemeris coverage timeline: bag-only (causal by record time) vs bag+RINEX (A1, §12.6-8)",
               "INFO" if cov["frac_epochs_ge5"]["bag_rinex"] > 0.9 else "WARN",
               "sat-epochs with a healthy ephemeris: bag-only %.1f%%, bag+RINEX %.1f%%; epochs with >=5: %.1f%% / %.1f%%; first >=5 at %s / %s s; BDS GEO observed %s; never covered bag-only: %s" % (
                   100 * cov["sat_epoch_fraction"]["bag_only"], 100 * cov["sat_epoch_fraction"]["bag_rinex"],
                   100 * cov["frac_epochs_ge5"]["bag_only"], 100 * cov["frac_epochs_ge5"]["bag_rinex"],
                   report.fmt(cov["first_epoch_ge5_healthy_eph_s"]["bag_only"], 1), report.fmt(cov["first_epoch_ge5_healthy_eph_s"]["bag_rinex"], 1),
                   dict(geo) or "none", ", ".join(cov["sats_never_covered_bag_only"]) or "none"))
    if plots and series:
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            arr = np.array(series)
            fig, ax = plt.subplots(figsize=(9, 3.2))
            ax.plot(arr[:, 0] - t0, arr[:, 1], label="observed (psr or Doppler)", color="#444444")
            ax.plot(arr[:, 0] - t0, arr[:, 3], label="with ephemeris: bag+RINEX", color="#1f77b4")
            ax.plot(arr[:, 0] - t0, arr[:, 2], label="with ephemeris: bag only (causal)", color="#d62728")
            ax.axhline(5, color="#999999", ls=":", lw=1)
            ax.set_xlabel("t - t0 [s]")
            ax.set_ylabel("satellites")
            ax.legend(fontsize=8, loc="best")
            ax.set_title("Ephemeris coverage")
            fig.tight_layout()
            fig.savefig(os.path.join(out_dir, "plot_eph_coverage.png"), dpi=110)
            plt.close(fig)
        except Exception:
            pass


def analyse_glo_channel(col, rnx, checks, rep):
    k_eph = {}
    for r in list(rnx) + list(col.geph):
        if r["sys"] == "R":
            k_eph.setdefault(r["sat"], set()).add(r["frq"])
    mism = collections.Counter()
    n = 0
    k_obs = {}
    for e in col.epochs:
        for s in e["sats"]:
            if s["sys"] != "R":
                continue
            for sl in s["slots"]:
                code, verdict, k = sats.signal_from_frequency("R", sl["freq"])
                if verdict != "Accepted":
                    continue
                n += 1
                k_obs.setdefault(s["sat"], set()).add(k)
                if s["sat"] in k_eph and k not in k_eph[s["sat"]]:
                    mism[s["sat"]] += 1
    rep["glo_channel"] = {"k_obs": {k: sorted(v) for k, v in k_obs.items()}, "k_eph": {k: sorted(v) for k, v in k_eph.items() if k in k_obs},
                          "mismatch_slots": dict(mism), "slots": n}
    if n == 0:
        checks.add("G10", "GLONASS channel k from freqs vs ephemeris frq (§12.3, §12.6-1)", "SKIP", "no GLONASS slots")
        return
    no_eph = sorted(s for s in k_obs if s not in k_eph)
    checks.add("G10", "GLONASS channel k from freqs vs ephemeris frq (§12.3, §12.6-1)", "PASS" if not mism else "FAIL",
               "%d slots; mismatching satellites %s; observed without any ephemeris: %s" % (n, dict(mism) or "none", no_eph or "none"))


def analyse_sat2freq(col, rnx, checks, rep):
    """Self-check 1, sat2freq part (AMENDMENTS A35-C8: the cross-check belongs here, the row builders do not repeat
    it): every accepted slot's code (freqs -> signal_table) taken back through RTKLIB-EX sat2freq(sat, code, nav) must
    give the slot's freqs within the table tolerance. For GLONASS, sat2freq takes the channel of the satellite's
    first ephemeris in nav (RINEX records first, then the bag's, as the store merges them, A1); a satellite without
    any ephemeris gets RTKLIB's error value 0 and is counted as unverifiable, not as a mismatch."""
    fcn = {}
    for r in list(rnx) + list(col.geph):
        if r.get("sys") == "R" and r["sat"] not in fcn:
            fcn[r["sat"]] = int(r["frq"])
    per = collections.defaultdict(collections.Counter)
    bad = collections.Counter()
    unver = collections.Counter()
    for e in col.epochs:
        for s in e["sats"]:
            if not s["sys"]:
                continue
            for sl in s["slots"]:
                code, verdict, _k = sats.signal_from_frequency(s["sys"], sl["freq"])
                c = per[s["sys"]]
                if verdict != "Accepted":
                    c["not_accepted"] += 1
                    continue
                f = sats.rtk_sat2freq(s["sys"], code, fcn.get(s["sat"]))
                if f == 0.0:
                    c["unverifiable"] += 1
                    unver[s["sat"]] += 1
                elif abs(f - sl["freq"]) > sats.FREQ_TOL_HZ:
                    c["mismatch"] += 1
                    bad["%s %s %.4f MHz -> %.4f MHz" % (s["sat"], code, sl["freq"] / 1e6, f / 1e6)] += 1
                else:
                    c["consistent"] += 1
    rep["sat2freq"] = {"per_system": {sats.SYS_NAMES.get(k, k): dict(v) for k, v in per.items()}, "mismatches": dict(bad),
                       "unverifiable_sats": dict(unver), "glo_channels_from_eph": fcn}
    title = "self-check 1: RTKLIB sat2freq(sat, code, nav) of every accepted slot = its freqs (A35-C8)"
    if not per:
        checks.add("G14", title, "SKIP", "no range_meas epochs in this bag")
        return
    n_ok = sum(v["consistent"] for v in per.values())
    checks.add("G14", title, "FAIL" if bad else "PASS",
               "%d slots consistent, %d mismatching%s, %d unverifiable (GLONASS without an ephemeris channel: %s), %d slots not accepted by the table (G3); per system %s" % (
                   n_ok, sum(bad.values()), (": " + "; ".join("%s (%d)" % kv for kv in list(bad.items())[:6])) if bad else "",
                   sum(unver.values()), ", ".join(sorted(unver)) or "none", sum(v["not_accepted"] for v in per.values()),
                   {sats.SYS_NAMES.get(k, k): dict(v) for k, v in sorted(per.items())}))


KLOB_LSB = (2.0 ** -30, 2.0 ** -27, 2.0 ** -24, 2.0 ** -24, 2.0 ** 11, 2.0 ** 14, 2.0 ** 16, 2.0 ** 16)   # GPS & BDS D1 alike


def klob_distance(a, b):
    """RMS difference of two Klobuchar sets in broadcast LSB units (GPS ICD Table 20-X; BDS D1 uses the same scales)."""
    return float(np.sqrt(np.mean([((x - y) / l) ** 2 for x, y, l in zip(a, b, KLOB_LSB)])))


def iono_reference_sets(rnx_header):
    """(label, 8 values) Klobuchar sets from the RINEX header: GPS (GPSA+GPSB), BDS per SV/time mark, QZS."""
    out = []
    if not rnx_header:
        return out
    by = collections.defaultdict(dict)
    for e in rnx_header.get("iono", []):
        k = e["kind"]
        if k in ("GPSA", "GPSB", "QZSA", "QZSB", "BDSA", "BDSB", "IRNA", "IRNB"):
            by[(k[:3], e["mark"], e["sv"])][k[3]] = e["vals"]
    for (sysk, mark, sv), ab in by.items():
        if "A" in ab and "B" in ab:
            out.append(("%s%s" % (sysk, (" %s %s" % (mark, sv)) if mark or sv else ""), ab["A"] + ab["B"]))
    return out


E6_TITLE = "Klobuchar source (A38): GPS set = RINEX header GPSA/GPSB; bag iono_params only a diagnostic (family classified)"


def analyse_iono(col, rnx_header, checks, rep):
    """A38: the core takes the GPS Klobuchar set from the RINEX header only (the bag's iono_params were found to be
    BDS D1 sets on Grass02/Outdoor01 and never enter nav.ion_gps). E6 therefore passes on the RINEX set being there
    (without it the core runs with I = 0, sigma_ion = 5 m, DESIGN §8) and classifies the bag's sets as a diagnostic."""
    bag_sets = collections.Counter(tuple(round(v, 15) for v in i["data"]) for i in col.iono)
    rn = None
    if rnx_header and rnx_header.get("iono_gps_alpha") and rnx_header.get("iono_gps_beta"):
        rn = tuple(rnx_header["iono_gps_alpha"] + rnx_header["iono_gps_beta"])
    refs = iono_reference_sets(rnx_header)
    rep["iono"] = {"bag_messages": len(col.iono), "bag_distinct_sets": [list(k) for k in bag_sets], "rinex_gpsa_gpsb": list(rn) if rn else None,
                   "gps_klobuchar_source": "rinex_header" if rn else "none (I = 0, sigma_ion = 5 m)"}
    head = ("GPS Klobuchar from the RINEX header GPSA/GPSB: %s" % (["%.4g" % v for v in rn],) if rn else
            "RINEX header GPSA/GPSB NOT available: the core runs without ionosphere model (I = 0, sigma_ion = 5 m, DESIGN §8)")
    verdict = "PASS" if rn else "WARN"
    if not col.iono:
        checks.add("E6", E6_TITLE, verdict, head + "; no /ublox_driver/iono_params in the bag")
        return
    lens = {len(k) for k in bag_sets}
    if lens != {8}:
        checks.add("E6", E6_TITLE, "WARN", head + "; bag iono_params value counts %s (8 expected): gnss_io rejects them (decode error), "
                   "no effect on the core (A38)" % sorted(lens))
        return
    classes = []
    for k in bag_sets:
        d = sorted(((klob_distance(k, v), lab) for lab, v in refs), key=lambda x: x[0])
        d_gps = min([x for x in d if x[1].startswith("GPS")] or [(float("inf"), "GPS")])[0]
        d_bds = min([x for x in d if x[1].startswith("BDS")] or [(float("inf"), "BDS")])[0]
        # nearest broadcast family decides; a GPS set from another upload of the day is still nearer to GPSA/GPSB
        # (same climatology) than to any BDS D1 set, whose alpha2/alpha3/beta are typically an order larger
        if not refs:
            cls = "unknown"
        elif d_gps < d_bds:
            cls = "GPS"
        else:
            cls = "BDS-like"
        classes.append({"set": list(k), "nearest": d[:3], "dist_gps_lsb": d_gps, "dist_bds_lsb": d_bds, "class": cls})
    t = np.array([i["t_record"] for i in col.iono])
    cadence = float((t[-1] - t[0]) / (t.size - 1)) if t.size > 1 else float("nan")     # messages arrive in bursts
    rep["iono"].update({"classification": classes, "publish_interval_median_s": cadence})
    checks.add("E6", E6_TITLE, verdict,
               head + "; bag iono_params (diagnostic, not used, A38): %d messages, %d distinct sets, one every %.1f s on average; %s" % (
                   len(col.iono), len(bag_sets), cadence,
                   "; ".join("set %s: %s (distance to RINEX GPSA/GPSB %.0f LSB, to the nearest RINEX BDSA/BDSB %.0f LSB)" % (
                       ["%.4g" % v for v in c["set"]], c["class"], c["dist_gps_lsb"], c["dist_bds_lsb"]) for c in classes)))


TP_SYS_BITS = {0x01: "GPS", 0x04: "GLO", 0x08: "GAL", 0x10: "QZS", 0x20: "BDS"}   # RTKLIB/gnss_comm SYS_* bit values
TP_AMBIGUOUS_FRAC = 0.1


def _field(msg, word):
    """First top-level field whose name contains `word` (layout known only from the bag's message_definition)."""
    for k, v in (msg.items() if isinstance(msg, dict) else []):
        if word in k.lower():
            return v
    return None


def tp_time_to_gpst(week, tow, utc_based, time_sys):
    """GPST seconds (Unix origin) of a time_pulse_info time (week, tow) read in its declared time base, and the name
    of the interpretation. u-blox TIM-TP gives week/tow "according to the time base": UTC when utc_based, else the
    GNSS time of time_sys (BDT: GPST - 14 s, week - 1356; GST: GPST seconds, week - 1024; GLONASST: UTC + 3 h on the
    GPS week count; QZSST = GPST). An absent or zero time_sys is read as GPST."""
    t = gnsstime.gpst(week, tow)
    if utc_based:
        return gnsstime.utc_to_gpst(t), "UTC"
    name = TP_SYS_BITS.get(int(time_sys)) if time_sys not in (None, 0) else "GPS"
    if name in ("GPS", "QZS"):
        return t, "GPST"
    if name == "BDS":
        return gnsstime.bdt_to_gpst(week, tow), "BDT"
    if name == "GAL":
        return gnsstime.gpst(week + gnsstime.GST_WEEK_OFFSET, tow), "GST"
    if name == "GLO":
        return gnsstime.utc_to_gpst(t - gnsstime.GLOT_MINUS_UTC_S), "GLONASST"
    return t, "unknown time_sys %r read as GPST" % (time_sys,)


def analyse_time_pulse(col, checks, rep, lat_offsets):
    """A19: leap second / time system / gross errors / host-clock steps of time_pulse_info.

    Each message announces the NEXT pulse: its host arrival, corrected by the range_meas transport latency (median
    record - GPST->UTC), precedes the pulse's UTC by a lead in [0, P) with P the pulse period (1 s, or 5 s as on
    Grass01/02 where 1 Hz messages announce a 5 s pulse). Split lead = n + phi: the fractional part phi is fixed by
    the host-vs-GNSS phase of the announcements (its circular mean phi0), the integer parts n tell where each message
    sits in the window. A consistent time base has every n in [0, P-1]; an integer-second error k shifts all n by k
    (+-1: a leap second, +-18: GPST/UTC mix-up, else a wrong time system). An error is only detectable if it pushes
    some message out of the window: +1 s needs a message in the last second before a pulse (max n = P-1), -1 s one in
    the first (min n = 0) — always true for P = 1 s, true for P = 5 s when the announcements cover the window; if
    neither holds the verdict is WARN (not decisive). phi0 within the jitter of an integer makes n unstable (WARN,
    ambiguous). Host-clock steps are steps of the unwrapped phi series (the integer sawtooth of P > 1 s is not a step).
    Repeated announcements of one pulse are not interval errors; intervals must be whole multiples of the period."""
    if not col.tp:
        rep["time_pulse"] = {"present": False}
        checks.add("T1", "time_pulse_info checks (A19: leap second, time system, gross errors, host clock steps)", "SKIP",
                   "topic /ublox_driver/time_pulse_info absent from this bag (README §4.2 lists it); nothing to check. "
                   "Host-clock steps are covered by the range_meas latency series instead (L1).")
        return
    ts, recs, interp = [], [], collections.Counter()
    sysv, utcv = collections.Counter(), collections.Counter()
    for tp in col.tp:
        m = tp["msg"]
        tt = m.get("time") if isinstance(m, dict) else None
        ub, tsys = _field(m, "utc"), _field(m, "sys")
        sysv[str(tsys)] += 1
        utcv[str(ub)] += 1
        if isinstance(tt, dict) and "week" in tt and "tow" in tt:
            t, how = tp_time_to_gpst(tt["week"], tt["tow"], bool(ub), tsys)
            ts.append(t)
            recs.append(tp["t_record"])
            interp[how] += 1
    res = {"present": True, "n": len(col.tp), "time_system_values": dict(sysv), "utc_based_values": dict(utcv),
           "interpretation": dict(interp), "fields": sorted(col.tp[0]["msg"].keys()) if isinstance(col.tp[0]["msg"], dict) else None}
    verdict, ev = "INFO", ""
    if ts:
        ts, recs = np.array(ts), np.array(recs)
        pulses = np.unique(np.round(ts, 6))
        d = np.diff(pulses)
        period = float(np.median(d)) if d.size else 1.0
        res["pulse_interval_s"] = robust.summary(d)
        res["repeated_announcements"] = int(ts.size - pulses.size)
        res["gross_interval_errors"] = int(np.sum(np.abs(d / period - np.round(d / period)) > 1e-3)) if d.size else 0
        med_lat = float(np.median(lat_offsets)) / 1e3 if len(lat_offsets) else float("nan")
        res["meas_latency_used_s"] = med_lat if np.isfinite(med_lat) else None
        tu = np.array([gnsstime.gpst_to_utc(t) for t in ts])
        res["record_minus_utc_ms"] = robust.summary((recs - tu) * 1e3)
        head = "%d messages (%s), pulse period %.3f s, %d repeated announcements, %d gross interval errors" % (
            ts.size, dict(interp), period, res["repeated_announcements"], res["gross_interval_errors"])
        if not np.isfinite(med_lat):
            verdict = "FAIL" if res["gross_interval_errors"] else "WARN"
            ev = head + "; no range_meas latency: the leap-second test needs it"
        else:
            lead = tu - (recs - med_lat)
            res["lead_s"] = robust.summary(lead)
            ang = 2.0 * np.pi * (lead - np.floor(lead))
            c_, s_ = float(np.mean(np.cos(ang))), float(np.mean(np.sin(ang)))
            phi0 = (math.atan2(s_, c_) / (2.0 * np.pi)) % 1.0
            dphi = (lead - phi0 + 0.5) % 1.0 - 0.5                       # each message's phase relative to phi0
            jitter = float(robust.robust_std(dphi))
            n = np.round(lead - phi0 - dphi).astype(int)                  # integer part consistent with phi0
            naive = np.floor(lead).astype(int)
            mode = collections.Counter(naive.tolist()).most_common(1)[0][0]
            frac_off = float(np.mean(naive != mode)) if period <= 1.0 + 1e-3 else float(np.mean((naive < 0) | (naive > round(period) - 1)))
            # ambiguous when the spread of the phases (jitter, or several announcements of one pulse) reaches a whole second
            ph_lo, ph_hi = phi0 + float(np.percentile(dphi, 1)), phi0 + float(np.percentile(dphi, 99))
            ambiguous = ph_lo < 0.01 or ph_hi > 0.99
            n_lo, n_hi, P = int(n.min()), int(n.max()), int(round(period))
            k = 0
            if n_lo < 0:
                k = n_lo
            elif n_hi > P - 1:
                k = n_hi - (P - 1)
            decisive_late = n_hi == P - 1          # a +1 s error would push these messages out of the window
            decisive_early = n_lo == 0             # a -1 s error would push these out
            res.update({"integer_second_offset_k": k, "frac_messages_other_k": frac_off, "lead_phase_s": phi0, "lead_phase_jitter_s": jitter,
                        "lead_phase_range_s": [ph_lo, ph_hi],
                        "lead_integer_parts": sorted(set(n.tolist())), "ambiguous": bool(ambiguous),
                        "decisive": {"late_errors": bool(decisive_late), "early_errors": bool(decisive_early)}})
            # host-clock steps move phi; the integer sawtooth of a P > 1 s pulse train does not
            res["host_steps"] = robust.detect_steps(recs - recs[0], 1e3 * (np.unwrap(ang) / (2.0 * np.pi)), 50.0, 20.0)
            # k > 0: the announced pulse time is k s too large (late); k < 0: too small (early)
            what = {0: "consistent", 1: "one leap second (announced time 1 s late)", -1: "one leap second (announced time 1 s early)",
                    -18: "GPST/UTC mix-up (UTC numbers read as GPST)", 18: "GPST/UTC mix-up (GPST numbers flagged UTC)"}.get(
                        k, "wrong time system (%+d s)" % k)
            if res["gross_interval_errors"] or (k != 0 and not ambiguous):
                verdict = "FAIL"
            elif ambiguous or not (decisive_late and decisive_early) or abs(period - round(period)) > 1e-3:
                verdict = "WARN"
            else:
                verdict = "PASS"
            ev = head + "; lead of the announcement before its pulse: median %.3f s, window [0, %.0f) s, integer parts %s with phase %.3f s (jitter %.1f ms) => integer-second offset %+d s: %s%s%s; record-UTC median %.1f ms; host steps %s" % (
                float(np.median(lead)), period, res["lead_integer_parts"], phi0, 1e3 * jitter, k, what,
                ("; the phases (%.3f..%.3f s) reach a whole second: integer parts unstable, ambiguous (%.0f%% of the messages give another k)" % (ph_lo, ph_hi, 100 * frac_off)) if ambiguous else "",
                "" if (decisive_late and decisive_early) else "; not decisive for %s errors (no announcement in the %s second of the window)" % (
                    " and ".join(x for x, ok in (("+1 s", decisive_late), ("-1 s", decisive_early)) if not ok),
                    " / ".join(x for x, ok in (("last", decisive_late), ("first", decisive_early)) if not ok)),
                res["record_minus_utc_ms"]["median"], res["host_steps"] or "none")
    rep["time_pulse"] = res
    checks.add("T1", "time_pulse_info checks (A19: leap second, time system, gross errors, host clock steps)", verdict,
               ev or "present but no (week, tow) time field found: fields %s" % res["fields"])


def analyse_latency(col, checks, rep, out_dir, plots):
    eps = col.epochs
    if not eps:
        checks.add("L1", "GNSS arrival latency: range_meas record time - GPST->UTC", "SKIP", "no range_meas epochs in this bag")
        checks.add("L3", "self-check 6 (time, record-time part)", "SKIP", "no range_meas epochs in this bag")
        return np.zeros(0)
    tg = np.array([e["t_gpst"] for e in eps])
    tr = np.array([e["t_record"] for e in eps])
    tu = np.array([gnsstime.gpst_to_utc(t) for t in tg])
    lat = (tr - tu) * 1e3
    s, i = robust.theil_sen(tu - tu[0], lat, max_pairs=100000)
    steps = robust.detect_steps(tu - tu[0], lat, 50.0, 20.0)
    res = {"range_meas_record_minus_gpst_utc_ms": robust.summary(lat), "drift_ms_per_s": s, "drift_ppm": s * 1e3,
           "steps": steps, "leap_s_used": gnsstime.leap_at_gpst(tg[0])}
    if col.pvt:
        pg = np.array([p["t_gpst"] for p in col.pvt])
        pr = np.array([p["t_record"] for p in col.pvt])
        res["receiver_pvt_record_minus_gpst_utc_ms"] = robust.summary((pr - np.array([gnsstime.gpst_to_utc(t) for t in pg])) * 1e3)
    if col.lla:
        res["receiver_lla_stamp_minus_gpst_s"] = robust.summary(
            [l["stamp"] - p["t_gpst"] for l, p in zip(col.lla, col.pvt)]) if len(col.lla) == len(col.pvt) else None
    rep["latency"] = res
    with open(os.path.join(out_dir, "latency.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_gpst", "t_utc", "t_record", "record_minus_utc_ms"])
        for a, b, c, d in zip(tg, tu, tr, lat):
            w.writerow(["%.3f" % a, "%.6f" % b, "%.6f" % c, "%.3f" % d])
    checks.add("L1", "GNSS arrival latency: range_meas record time - GPST->UTC (leap %d s)" % res["leap_s_used"], "INFO" if not steps else "WARN",
               "median %.1f ms, MAD %.1f, p1 %.1f, p99 %.1f, min %.1f, max %.1f; drift %.1f ppm; persistent steps >50 ms: %s%s" % (
                   res["range_meas_record_minus_gpst_utc_ms"]["median"], res["range_meas_record_minus_gpst_utc_ms"]["mad"],
                   res["range_meas_record_minus_gpst_utc_ms"]["p1"], res["range_meas_record_minus_gpst_utc_ms"]["p99"],
                   res["range_meas_record_minus_gpst_utc_ms"]["min"], res["range_meas_record_minus_gpst_utc_ms"]["max"],
                   res["drift_ppm"], steps or "none",
                   ("; receiver_pvt median %.1f ms" % res["receiver_pvt_record_minus_gpst_utc_ms"]["median"]) if "receiver_pvt_record_minus_gpst_utc_ms" in res else ""))
    med_s = res["range_meas_record_minus_gpst_utc_ms"]["median"] / 1e3
    checks.add("L3", "self-check 6 (time, record-time part): |record - (GPST - leap)| < 0.3 s (leap seconds and time system consistent)",
               "PASS" if abs(med_s) < 0.3 else "FAIL",
               "median %+.4f s with leap %d s (a missing/extra leap second would show as ~±1 s, a GPST/UTC mix-up as ±18 s); "
               "the cross-correlation part of self-check 6 is method 2 of tools/time_calib/gnss_time_calib.py on the ref-R replay" % (med_s, res["leap_s_used"]))
    if res.get("receiver_lla_stamp_minus_gpst_s"):
        r = res["receiver_lla_stamp_minus_gpst_s"]
        checks.add("L2", "receiver_lla header stamp convention", "INFO",
                   "header.stamp - GPST(seconds, Unix origin) median %.3f s (min %.3f, max %.3f): receiver_lla stamps are %s" % (
                       r["median"], r["min"], r["max"], "GPST without leap seconds (not ROS/UTC time)" if abs(r["median"]) < 0.05 else "not GPST"))
    if plots:
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            fig, ax = plt.subplots(figsize=(9, 3.2))
            ax.plot(tu - tu[0], lat, ".", ms=1.5, color="#1f77b4")
            ax.set_xlabel("t - t0 [s]")
            ax.set_ylabel("record - GPST->UTC [ms]")
            ax.set_title("range_meas arrival (host clock offset + latency)")
            fig.tight_layout()
            fig.savefig(os.path.join(out_dir, "plot_latency.png"), dpi=110)
            plt.close(fig)
        except Exception:
            pass
    return lat


def analyse_ls(col, rnx, rnx_header, checks, rep, rate_hz, evaluation=False):
    """Doppler-LS sign test (G11) and self-check 5 (G12a/b: SPP / Doppler LS vs the receiver PVT).

    G11 is a decoding property (which Doppler sign fits the geometry at all: post-fit residuals of the two sign
    hypotheses differ by orders of magnitude), not an accuracy. G12a/b compare GNSS-only solutions with the receiver's
    PVT — a calibration-grade judgement that TEST_PLAN §3 puts on the development set: with evaluation=True (every
    non-development sequence, AMENDMENTS A2) they are SKIP and nothing is computed; the Doppler LS of G11 then
    linearises at the receiver's own PVT position (a few metres change the line of sight by < 1e-6, irrelevant for
    the sign test) instead of an SPP."""
    recs = [dict(r) for r in rnx]
    keys = {(r["sat"], round(r["toe"])) for r in recs}
    recs += [dict(r) for r in col.eph + col.geph if (r["sat"], round(r["toe"])) not in keys]
    sel = orbits.EphSelector(recs, MAX_EPH_AGE, causal=False)
    # GPS Klobuchar as the core takes it (A38): the RINEX header GPSA/GPSB, never the bag's iono_params (without a
    # RINEX set the core uses I = 0; this diagnostic SPP then falls back to RTKLIB's default Klobuchar set)
    iono, iono_src = None, "none (RTKLIB default set in this diagnostic; the core would use I = 0)"
    if rnx_header and rnx_header.get("iono_gps_alpha") and rnx_header.get("iono_gps_beta"):
        iono, iono_src = rnx_header["iono_gps_alpha"] + rnx_header["iono_gps_beta"], "rinex_header"
    pvt_t = np.array([p["t_gpst"] for p in col.pvt]) if col.pvt else np.zeros(0)
    step = max(1, int(round(10.0 / rate_hz)))
    rows = []
    last_rr = None
    for n, e in enumerate(col.epochs):
        if n % step:
            continue
        obs = ls_obs(e)
        p = None
        if pvt_t.size:
            k = int(np.argmin(np.abs(pvt_t - e["t_gpst"])))
            if abs(pvt_t[k] - e["t_gpst"]) < 0.05:
                p = col.pvt[k]
        p_rr = geodesy.pos2ecef(math.radians(p["lat"]), math.radians(p["lon"]), p["h_ell"]) if p and p["fix_type"] >= 2 else None
        row = {"t_gpst": e["t_gpst"], "spp": None, "spp_relaxed": None, "dop": None, "dop_relaxed": None, "dop_flip": None, "pvt": p}
        if evaluation:
            rr = p_rr
        else:
            x0 = last_rr if last_rr is not None else p_rr
            # L0-masked SPP (C/N0 >= 30) for self-check 5; an unmasked SPP only as a decode check when the mask leaves < 4
            sp = gnsslsq.spp(obs, e["t_gpst"], sel, iono=iono, x0=x0)
            sp_relaxed = sp if sp else gnsslsq.spp(obs, e["t_gpst"], sel, iono=iono, x0=x0, cn0_min=0.0)
            best = sp if sp and sp["rms_m"] < 30.0 else (sp_relaxed if sp_relaxed and sp_relaxed["rms_m"] < 30.0 else None)
            rr = best["rr"] if best else p_rr
            if best:
                last_rr = best["rr"]
            if sp:
                row["spp"] = {"rr": sp["rr"], "rms_m": sp["rms_m"], "n": sp["n"], "gdop": sp["gdop"], "isb": sp["isb"], "excluded": sp["excluded"]}
            if sp_relaxed:
                row["spp_relaxed"] = {"rr": sp_relaxed["rr"], "rms_m": sp_relaxed["rms_m"], "n": sp_relaxed["n"]}
        if rr is not None:
            # sign test on every Doppler (no C/N0 mask, el >= 5 deg): a decoding property, not an accuracy one
            d1r = gnsslsq.doppler_ls(obs, e["t_gpst"], sel, rr, doppler_sign=1.0, cn0_min=0.0, el_mask_deg=5.0)
            d2 = gnsslsq.doppler_ls(obs, e["t_gpst"], sel, rr, doppler_sign=-1.0, max_reject=0, cn0_min=0.0, el_mask_deg=5.0)
            if d1r:
                row["dop_relaxed"] = {"rms_mps": d1r["rms_mps"], "n": d1r["n"], "excluded": d1r["excluded"]}
            if d2:
                row["dop_flip"] = {"rms_mps": d2["rms_mps"], "n": d2["n"]}
            if not evaluation:
                d1 = gnsslsq.doppler_ls(obs, e["t_gpst"], sel, rr, doppler_sign=1.0)
                if d1:
                    row["dop"] = {"v_enu": d1["v_enu"], "rms_mps": d1["rms_mps"], "n": d1["n"], "drift_mps": d1["drift_mps"], "excluded": d1["excluded"]}
        rows.append(row)
    res = {"epochs": len(rows), "iono_source": iono_src,
           "linearisation_point": "receiver PVT (evaluation sequence, A2)" if evaluation else "SPP, receiver PVT when no SPP"}
    pairs = [(r["dop_relaxed"]["rms_mps"], r["dop_flip"]["rms_mps"]) for r in rows if r["dop_relaxed"] and r["dop_flip"]]
    dres = [a for a, _ in pairs]
    fres = [b for _, b in pairs]
    res["doppler_rms_mps_unmasked"] = robust.summary(dres)
    res["doppler_flipped_rms_mps_unmasked"] = robust.summary(fres)
    if pairs:
        ok = np.median(dres) < 0.5 and np.median(fres) > 20.0 * np.median(dres)
        checks.add("G11", "Doppler sign by LS fit: -lambda*D = range rate (§1) vs flipped sign (all Dopplers, el >= 5°)", "PASS" if ok else "FAIL",
                   "post-fit RMS median %.3f m/s with the design sign vs %.1f m/s flipped (%d epochs at %.1f Hz; linearised at the %s)" % (
                       np.median(dres), np.median(fres), len(dres), rate_hz, res["linearisation_point"]))
    else:
        checks.add("G11", "Doppler sign by LS fit", "SKIP", "no epoch with >= 5 Doppler satellites and an ephemeris")
    if evaluation:
        why = ("A2: not a development sequence — self-check 5 compares GNSS-only solutions with the receiver PVT (a "
               "calibration-grade judgement, TEST_PLAN §3: development set only); not computed")
        checks.add("G12a", "self-check 5 (position): SPP vs receiver PVT", "SKIP", why)
        checks.add("G12b", "self-check 5 (velocity): Doppler LS vs receiver PVT", "SKIP", why)
        rep["ls"] = res
        return rows
    res["doppler_rms_mps_masked"] = robust.summary([r["dop"]["rms_mps"] for r in rows if r["dop"]])
    res["epochs_with_masked_doppler_ls"] = sum(1 for r in rows if r["dop"])
    res["epochs_with_masked_spp"] = sum(1 for r in rows if r["spp"])
    res["epochs_with_unmasked_spp"] = sum(1 for r in rows if r["spp_relaxed"])
    # self-check 5
    hd, vd, spd, spd_h, spd_v = [], [], [], [], []
    for r in rows:
        p = r["pvt"]
        if not p or p["fix_type"] < 3:
            continue
        lat, lon = math.radians(p["lat"]), math.radians(p["lon"])
        rp = geodesy.pos2ecef(lat, lon, p["h_ell"])
        E = geodesy.enu_matrix(lat, lon)
        open_sky = p["h_acc"] < 5.0 and p["num_sv"] >= 8
        if r["spp"] and r["spp"]["rms_m"] < 30.0:
            d = E @ (r["spp"]["rr"] - rp)
            hd.append((math.hypot(d[0], d[1]), open_sky))
            vd.append((d[2], open_sky))
        if r["dop"]:
            v = np.array(r["dop"]["v_enu"])
            vp = np.array([p["vel_e"], p["vel_n"], -p["vel_d"]])
            spd.append((float(np.linalg.norm(v - vp)), open_sky))
            spd_h.append((float(np.hypot(v[0] - vp[0], v[1] - vp[1])), open_sky))
            spd_v.append((float(abs(v[2] - vp[2])), open_sky))
    def med(lst, open_only):
        v = [a for a, o in lst if o or not open_only]
        return (float(np.median(v)) if v else float("nan")), len(v)
    h_med, h_n = med(hd, True)
    v_med, v_n = med(spd, True)
    h_all, _ = med(hd, False)
    v_all, _ = med(spd, False)
    vh_med, _ = med(spd_h, True)
    vv_med, _ = med(spd_v, True)
    res["spp_vs_pvt"] = {"horizontal_diff_m_open_median": h_med, "n_open": h_n, "horizontal_diff_m_all_median": h_all,
                         "velocity_diff_mps_open_median": v_med, "n_vel_open": v_n, "velocity_diff_mps_all_median": v_all,
                         "velocity_diff_horizontal_mps_open_median": vh_med, "velocity_diff_vertical_mps_open_median": vv_med,
                         "vertical_diff_m": robust.summary([a for a, _ in vd])}
    res["spp_rms_m"] = robust.summary([r["spp"]["rms_m"] for r in rows if r["spp"]])
    hr = []
    for r in rows:
        p = r["pvt"]
        if p and p["fix_type"] >= 3 and r["spp_relaxed"] and r["spp_relaxed"]["rms_m"] < 30.0:
            lat, lon = math.radians(p["lat"]), math.radians(p["lon"])
            d = geodesy.enu_matrix(lat, lon) @ (r["spp_relaxed"]["rr"] - geodesy.pos2ecef(lat, lon, p["h_ell"]))
            hr.append(math.hypot(d[0], d[1]))
    res["unmasked_spp_vs_pvt_horizontal_m"] = robust.summary(hr)
    if h_n >= 10:
        checks.add("G12a", "self-check 5 (position): L0-masked SPP vs receiver PVT on open-sky epochs (h_acc < 5 m, >= 8 SV): horizontal median < 3 m",
                   "PASS" if h_med < 3.0 else "FAIL", "horizontal median %.2f m (n=%d; all epochs %.2f m); iono from %s" % (h_med, h_n, h_all, res["iono_source"]))
    else:
        checks.add("G12a", "self-check 5 (position): SPP vs receiver PVT", "SKIP",
                   "not evaluable: %d open-sky epochs have an L0-masked SPP (C/N0 >= 30 dB-Hz leaves < 4 pseudoranges; masked SPP epochs %d of %d); "
                   "decode-only unmasked SPP vs PVT horizontal median %s m (n=%d)" % (
                       h_n, res["epochs_with_masked_spp"], len(rows), report.fmt(float(np.median(hr)) if hr else None, 1), len(hr)))
    if v_n >= 10:
        checks.add("G12b", "self-check 5 (velocity): L0-masked Doppler LS vs receiver PVT velocity on open-sky epochs: median |dv| < 0.2 m/s",
                   "PASS" if v_med < 0.2 else "FAIL", "median %.3f m/s (n=%d; all epochs %.3f m/s; horizontal %.3f, vertical %.3f m/s); post-fit "
                   "RMS of those Doppler LS median %.3f m/s. A large vertical part with a small post-fit RMS is the geometry of the "
                   "C/N0-masked rows, not a motion: when only near-zenith satellites pass the mask, vertical velocity and clock drift "
                   "are nearly collinear (INTERFACE_CHANGES WP-g P4-TIME: the Doppler rows agree with the replay's antenna velocity to "
                   "~0.1 m/s on Grass01/02)" % (v_med, v_n, v_all, vh_med, vv_med, res["doppler_rms_mps_masked"].get("median", float("nan"))))
    else:
        checks.add("G12b", "self-check 5 (velocity): Doppler LS vs receiver PVT", "SKIP",
                   "not evaluable: %d open-sky epochs with an L0-masked Doppler LS (masked epochs %d of %d)" % (v_n, res["epochs_with_masked_doppler_ls"], len(rows)))
    rep["ls"] = res
    return rows


# ------------------------------------------------------------------------------------------------------------
# §12.6-7 lever-arm direction (G13)
# ------------------------------------------------------------------------------------------------------------
LEVER_EPOCH_STEP = 2          # every 2nd 10 Hz epoch (5 Hz): Doppler noise is white per epoch, multipath is not
LEVER_MIN_TURN = 0.15         # rad/s: |yaw rate| (level frame) of a turning epoch
LEVER_MIN_ROWS = 200          # turning Doppler rows needed for a verdict
LEVER_MIN_TERM = 0.02         # m/s: RMS of the lever term on the LOS (after the per-epoch drift) needed for a verdict
LEVER_K_TOL = 0.5             # |k| <= 0.5 -> l confirmed; |k + 2| <= 0.5 -> sign flipped (DESIGN §12.6-7: ~0 / ~-2)
LEVER_BLOCK_S = 10.0          # block length of the bootstrap (correlated multipath)
G13_TITLE = "lever-arm direction (§12.6-7): Doppler innovations with l vs -l on turns; regression on the omega x l term (~0; ~-2 = sign flipped)"


def level_rotation(g):
    """Minimal rotation C with C g/|g| = -e_z (a yaw-free levelling; the yaw is fitted separately)."""
    a = np.asarray(g, dtype=float) / np.linalg.norm(g)
    b = np.array([0.0, 0.0, -1.0])
    v = np.cross(a, b)
    s, c = float(np.linalg.norm(v)), float(a @ b)
    if s < 1e-12:
        return np.eye(3) if c > 0 else np.diag([1.0, -1.0, -1.0])
    vx = np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])
    return np.eye(3) + vx + vx @ vx * ((1.0 - c) / (s * s))


def _epoch_demean(x, ep, w, n_ep):
    num = np.bincount(ep, weights=w * x, minlength=n_ep)
    den = np.bincount(ep, weights=w, minlength=n_ep)
    return x - (num / np.where(den > 0, den, 1.0))[ep]


def lever_arm_test(samples, traj, gyro, t_est_of, lever, imu_time_offset=0.0, epoch_step=LEVER_EPOCH_STEP, seed=0):
    """DESIGN §12.6-7 on prepared inputs (the audit builds them from the bag, the replay and the time calibration).

    samples  : GNSS epochs [{"t_gpst", "rr" (ECEF), "rows": [(sat, e, rs, vs, ddts, z, el), ...]}], z = -lambda*D
               (gnsslsq.doppler_rows); traj: load_state_trace() arrays (W frame, estimator time); gyro: (n, 4) header
               stamp + angular rate of the estimator's IMU; t_est_of: GPST -> estimator time (A18 file).
    Model: the antenna velocity is w^W = v + R (omega - b_g) x h, with h = l (design), -l or 0. It is levelled with the
    replay's gravity (C g = -|g| e_z), rotated by a yaw psi into ENU and into ECEF at the receiver, and predicts every
    Doppler row through the solver's model rate (gnsslsq: e.(v_s - w) + Sagnac rate + d - c dts_dot); the clock drift
    d of each epoch is the weighted mean of its rows (a nuisance, as in the core's Doppler innovations). psi is fitted
    per hypothesis by least squares (closed form on a 0.01 deg grid). Innovations nu(h) are compared on turning epochs
    (|yaw rate| >= LEVER_MIN_TURN), and nu(l) is regressed on the lever term m (its contribution to the prediction,
    per-epoch demeaned): with the true lever s*l, nu(l) = (s - 1) m + noise, so k ~ 0 confirms l, k ~ -2 means the
    antenna is at -l, k ~ -1 a lever that is not seen at all. Rows are weighted with the A35-C6 variance-factor form
    sigma_D0^2 (1 + 1/sin^2 el)/2, sigma_D0 = 0.05 m/s; the standard error of k comes from a block bootstrap."""
    l = np.asarray(lever, dtype=float)
    C = level_rotation(np.median(traj["g"], axis=0))
    tt = traj["t"]
    tg, g3 = gyro[:, 0] - imu_time_offset, gyro[:, 1:4]
    P0, Q0, Z0, PL, QL, ZL, B, W, EP, TURN, T_EP = [], [], [], [], [], [], [], [], [], [], []
    n_ep = 0
    used = 0
    for k, s in enumerate(samples):
        if k % epoch_step or not s["rows"]:
            continue
        t = t_est_of(s["t_gpst"])
        if not (tt[0] <= t <= tt[-1]) or not (tg[0] <= t <= tg[-1]):
            continue
        i = int(np.clip(np.searchsorted(tt, t), 1, tt.size - 1))
        j = i if abs(tt[i] - t) < abs(t - tt[i - 1]) else i - 1
        f_ = (t - tt[i - 1]) / (tt[i] - tt[i - 1]) if tt[i] > tt[i - 1] else 0.0
        v = traj["v"][i - 1] + f_ * (traj["v"][i] - traj["v"][i - 1])
        R = traj["R"][j]
        om = np.array([np.interp(t, tg, g3[:, c]) for c in range(3)]) - traj["bg"][j]
        a0 = C @ v                                          # levelled IMU velocity
        aq = C @ (R @ np.cross(om, l))                      # levelled lever term R (omega x l)
        wz = float((C @ (R @ om))[2])
        pos = geodesy.ecef2pos(np.asarray(s["rr"], dtype=float))
        E = geodesy.enu_matrix(pos[0], pos[1])              # ECEF -> ENU
        for sat, e, rs, vs, ddts, z, el in s["rows"]:
            J = np.array(gnsslsq.doppler_model_jacobian(e, rs))       # d rate / d v_receiver (ECEF)
            rate_sat = gnsslsq.doppler_model_rate(e, rs, vs, np.asarray(s["rr"], dtype=float), np.zeros(3))
            fN = E @ J                                      # J . w_E = fN . w_N
            P0.append(fN[0] * a0[0] + fN[1] * a0[1]); Q0.append(fN[1] * a0[0] - fN[0] * a0[1]); Z0.append(fN[2] * a0[2])
            PL.append(fN[0] * aq[0] + fN[1] * aq[1]); QL.append(fN[1] * aq[0] - fN[0] * aq[1]); ZL.append(fN[2] * aq[2])
            B.append(z - rate_sat + sats.CLIGHT * ddts)
            sinel = max(math.sin(el), 0.1)
            W.append(1.0 / (0.05 ** 2 * (1.0 + 1.0 / sinel ** 2) / 2.0))
            EP.append(n_ep)
            TURN.append(abs(wz) >= LEVER_MIN_TURN)
        T_EP.append(s["t_gpst"])
        n_ep += 1
        used += 1
    if n_ep < 10:
        return {"n_epochs": n_ep, "n_rows": len(B), "reason": "fewer than 10 GNSS epochs inside the replay's time span"}
    P0, Q0, Z0, PL, QL, ZL, B, W = (np.array(x) for x in (P0, Q0, Z0, PL, QL, ZL, B, W))
    EP, TURN = np.array(EP), np.array(TURN, dtype=bool)
    grid = np.radians(np.arange(0.0, 360.0, 0.01))

    def fit(hs):
        # nu(psi) = beta - cos(psi) P - sin(psi) Q with beta = B - Z (all per-epoch demeaned, the drift eliminated)
        P, Q, Z = P0 + hs * PL, Q0 + hs * QL, Z0 + hs * ZL
        b = _epoch_demean(B - Z, EP, W, n_ep)
        Pt, Qt = _epoch_demean(P, EP, W, n_ep), _epoch_demean(Q, EP, W, n_ep)
        s_bb, s_bp, s_bq = np.sum(W * b * b), np.sum(W * b * Pt), np.sum(W * b * Qt)
        s_pp, s_pq, s_qq = np.sum(W * Pt * Pt), np.sum(W * Pt * Qt), np.sum(W * Qt * Qt)
        c, s_ = np.cos(grid), np.sin(grid)
        ssr = s_bb - 2 * c * s_bp - 2 * s_ * s_bq + c * c * s_pp + 2 * c * s_ * s_pq + s_ * s_ * s_qq
        psi = float(grid[int(np.argmin(ssr))])
        nu = b - math.cos(psi) * Pt - math.sin(psi) * Qt
        m = _epoch_demean(math.cos(psi) * PL + math.sin(psi) * QL + ZL, EP, W, n_ep)   # lever term of the prediction
        return psi, nu, m

    out = {"n_epochs": n_ep, "n_rows": int(B.size), "n_turn_rows": int(TURN.sum()), "hypotheses": {}}
    for name, hs in (("l", 1.0), ("-l", -1.0), ("0", 0.0)):
        psi, nu, m = fit(hs)
        out["hypotheses"][name] = {"psi_deg": math.degrees(psi) % 360.0,
                                   "rms_turn_mps": float(np.sqrt(np.mean(nu[TURN] ** 2))) if TURN.any() else float("nan"),
                                   "rms_all_mps": float(np.sqrt(np.mean(nu ** 2)))}
        if name == "l":
            nu_l, m_l = nu, m
    den = float(np.sum(W * m_l * m_l))
    k = float(np.sum(W * nu_l * m_l) / den) if den > 0 else float("nan")
    rng = np.random.RandomState(seed)
    blk = np.floor((np.array(T_EP) - T_EP[0]) / LEVER_BLOCK_S).astype(int)[EP]
    ub = np.unique(blk)
    ks = []
    for _ in range(200):
        pick = rng.choice(ub, ub.size, replace=True)
        idx = np.concatenate([np.where(blk == b_)[0] for b_ in pick])
        d_ = float(np.sum(W[idx] * m_l[idx] ** 2))
        if d_ > 0:
            ks.append(float(np.sum(W[idx] * nu_l[idx] * m_l[idx]) / d_))
    out.update({"k": k, "k_se_block_bootstrap": float(np.std(ks)) if ks else float("nan"), "blocks": int(ub.size),
                "lever_term_rms_turn_mps": float(np.sqrt(np.mean(m_l[TURN] ** 2))) if TURN.any() else float("nan")})
    return out


def lever_verdict(res):
    """(verdict, text) of lever_arm_test's result (thresholds above)."""
    if "k" not in res:
        return "SKIP", "not evaluable: %s" % res.get("reason", "no data")
    h = res["hypotheses"]
    txt = ("k = %+.2f (block-bootstrap s.e. %.2f, %d blocks); innovation RMS on %d turning rows (|yaw rate| >= %.2f rad/s): "
           "l %.3f / -l %.3f / 0 %.3f m/s, lever term RMS %.3f m/s (%d epochs, %d rows; fitted yaw %.2f / %.2f / %.2f deg)" % (
               res["k"], res["k_se_block_bootstrap"], res["blocks"], res["n_turn_rows"], LEVER_MIN_TURN, h["l"]["rms_turn_mps"],
               h["-l"]["rms_turn_mps"], h["0"]["rms_turn_mps"], res["lever_term_rms_turn_mps"], res["n_epochs"], res["n_rows"],
               h["l"]["psi_deg"], h["-l"]["psi_deg"], h["0"]["psi_deg"]))
    if res["n_turn_rows"] < LEVER_MIN_ROWS or not res["lever_term_rms_turn_mps"] >= LEVER_MIN_TERM:
        return "SKIP", "insufficient excitation (needs >= %d turning rows and a lever term RMS >= %.2f m/s): %s" % (
            LEVER_MIN_ROWS, LEVER_MIN_TERM, txt)
    if abs(res["k"]) <= LEVER_K_TOL and h["l"]["rms_turn_mps"] < h["-l"]["rms_turn_mps"]:
        return "PASS", "l confirmed: " + txt
    if abs(res["k"] + 2.0) <= LEVER_K_TOL and h["-l"]["rms_turn_mps"] < h["l"]["rms_turn_mps"]:
        return "FAIL", "SIGN FLIPPED (the antenna is at -l): " + txt
    return "WARN", "inconclusive (neither l nor -l explains the turns; k ~ -1 = no lever seen): " + txt


def analyse_lever_arm(col, rnx, args, checks, rep, evaluation):
    """G13 = DESIGN §12.6-7 on the development set: needs the GNSS-free replay trajectory (state_trace.txt of ref-R)
    and its method-2 time calibration; never read on other sequences (A2: a calibration-grade judgement)."""
    if evaluation:
        checks.add("G13", G13_TITLE, "SKIP", "A2: not a development sequence — the lever-arm judgement uses the replay trajectory "
                   "and belongs to the development set; not computed")
        return
    st = os.path.join(args.replay_dir, "state_trace.txt") if args.replay_dir else ""
    missing = []
    if not st or not os.path.exists(st):
        missing.append("the GNSS-free ref-R replay of this bag (state_trace.txt%s)" % ((" not in " + args.replay_dir) if args.replay_dir else ""))
    if not args.time_calib or not os.path.exists(args.time_calib):
        missing.append("its method-2 time calibration (--time-calib, A18 file of tools/time_calib/gnss_time_calib.py)")
    if missing:
        checks.add("G13", G13_TITLE, "SKIP", "blocked: ref-R — needs %s" % " and ".join(missing))
        return
    with open(args.time_calib) as f:
        calib = timecalib_yaml.loads(f.read())
    traj = load_state_trace(st)
    gyro = np.array(col.gyro, dtype=float) if col.gyro else np.zeros((0, 4))
    if gyro.shape[0] < 100:
        checks.add("G13", G13_TITLE, "SKIP", "no %s samples in the bag" % args.imu_topic)
        return
    recs = [dict(r) for r in rnx]
    keys = {(r["sat"], round(r["toe"])) for r in recs}
    recs += [dict(r) for r in col.eph + col.geph if (r["sat"], round(r["toe"])) not in keys]
    sel = orbits.EphSelector(recs, MAX_EPH_AGE, causal=False)
    pvt_t = np.array([p["t_gpst"] for p in col.pvt]) if col.pvt else np.zeros(0)
    samples, rr_last = [], None
    for e in col.epochs:
        rr = None
        if pvt_t.size:
            kk = int(np.argmin(np.abs(pvt_t - e["t_gpst"])))
            p = col.pvt[kk]
            if abs(pvt_t[kk] - e["t_gpst"]) < 1.0 and p["fix_type"] >= 2:
                rr = geodesy.pos2ecef(math.radians(p["lat"]), math.radians(p["lon"]), p["h_ell"])
        rr = rr if rr is not None else rr_last
        if rr is None:
            continue
        rr_last = rr
        rows = gnsslsq.doppler_rows(ls_obs(e), e["t_gpst"], sel, rr)
        samples.append({"t_gpst": e["t_gpst"], "rr": rr, "rows": rows})
    res = lever_arm_test(samples, traj, gyro, lambda t: timecalib_yaml.evaluate(calib, t), args.lever_arm, args.imu_time_offset)
    res.update({"replay_dir": args.replay_dir, "time_calib": args.time_calib, "lever_arm": list(args.lever_arm),
                "time_calib_methods": sorted({s.get("method", "?") for s in calib["segments"]}),
                "time_calib_sigma_s": max(s["sigma_toff_s"] for s in calib["segments"])})
    rep["lever_arm"] = res
    verdict, txt = lever_verdict(res)
    checks.add("G13", G13_TITLE, verdict, txt + "; time map %s (sigma %.0f ms), replay %s" % (
        "/".join(res["time_calib_methods"]), 1e3 * res["time_calib_sigma_s"], args.replay_dir))


def analyse_livox(col, conns, args, checks, rep):
    md5, dtype, path = livox_repo_md5(args.repo)
    rows = [c for c in conns if c["topic"] in (args.lidar_topic, "/livox/mid360/lidar")]
    res = {"repo_header": path, "repo_md5": md5, "repo_datatype": dtype,
           "bag": [{"topic": c["topic"], "datatype": c["datatype"], "md5sum": c["md5sum"]} for c in rows]}
    avia = [c for c in rows if c["topic"] == args.lidar_topic]
    if col.lidar_timebase:
        h = np.array([a for a, _b, _n in col.lidar_timebase])
        tb = np.array([b for _a, b, _n in col.lidar_timebase])
        d = h - tb
        res["timebase_first_s"] = float(tb[0])
        res["header_minus_timebase_s"] = robust.summary(d)
        slope, _ = robust.theil_sen(h - h[0], d - d[0], max_pairs=50000)
        res["header_minus_timebase_drift_ppm"] = slope * 1e6
        res["timebase_semantics"] = ("timebase == header stamp (first-point time on the header clock)" if abs(np.median(d)) < 1e-3 else
                                     ("timebase on another clock (offset %.6g s, drift %.1f ppm vs header)" % (float(np.median(d)), slope * 1e6)))
        res["points_per_scan"] = robust.summary([n for _h, _t, n in col.lidar_timebase])
    rep["livox"] = res
    if md5 is None or not avia:
        checks.add("S1", "Livox CustomMsg type/md5 vs the repository's livox_ros_driver headers", "SKIP" if not avia else "FAIL",
                   "repo header %s md5 %s; bag %s" % (path, md5, res["bag"]))
        return
    ok = all(c["md5sum"] == md5 for c in avia) and all(c["datatype"] == dtype for c in avia)
    checks.add("S1", "Livox CustomMsg type/md5 vs the repository's livox_ros_driver headers", "PASS" if ok else "FAIL",
               "%s: %s [%s] vs repo %s [%s]; %s; points/scan median %s" % (
                   args.lidar_topic, avia[0]["datatype"], avia[0]["md5sum"], dtype, md5,
                   res.get("timebase_semantics", "no scans read"), report.fmt(res.get("points_per_scan", {}).get("median"), 0)))


def analyse_images(col, args, checks, rep):
    res = {"formats": dict(col.image_formats), "samples": len(col.image_samples)}
    try:
        import cv2
    except ImportError:
        rep["image"] = res
        checks.add("S2", "CompressedImage format and grey-image path equality", "SKIP", "python cv2 not available")
        return
    n_ok = n_grey_ok = n_unch = 0
    shapes = collections.Counter()
    bad = []
    for t_rec, fmt, data in col.image_samples:
        buf = np.frombuffer(data, dtype=np.uint8)
        # path A = image_transport republish (compressed_image_transport decoder, mode "unchanged") + cv_bridge bgr8
        a = cv2.imdecode(buf, cv2.IMREAD_UNCHANGED)
        enc = fmt.split(";")[0].strip() if ";" in fmt else ("bgr8" if a is not None and a.ndim == 3 else "mono8")
        if a is not None and a.ndim == 3 and ";" in fmt and enc in ("rgb8", "bgr8"):
            comp = fmt[fmt.find(";"):]
            if "compressed bgr" in comp:
                if enc == "rgb8":
                    a = cv2.cvtColor(a, cv2.COLOR_BGR2RGB)
            else:
                if enc == "bgr8":
                    a = cv2.cvtColor(a, cv2.COLOR_RGB2BGR)
        if a is not None and enc == "rgb8":
            a = cv2.cvtColor(a, cv2.COLOR_RGB2BGR)       # cv_bridge toCvCopy(msg, "bgr8")
        elif a is not None and a.ndim == 2:
            a = cv2.cvtColor(a, cv2.COLOR_GRAY2BGR)
        # path B = replay direct read (DESIGN §13): IMREAD_COLOR, channel order decided by the format field
        b = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        comp = fmt[fmt.find(";"):] if ";" in fmt else ""
        if b is not None and comp and "compressed bgr" not in comp and "compressed" in comp:
            b = cv2.cvtColor(b, cv2.COLOR_RGB2BGR)
        u = cv2.imdecode(buf, cv2.IMREAD_UNCHANGED)
        c = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        n_unch += u is not None and c is not None and u.shape == c.shape and bool(np.array_equal(u, c))
        if a is None or b is None:
            bad.append((t_rec, "decode failed"))
            continue
        shapes[str(a.shape)] += 1
        same = a.shape == b.shape and bool(np.array_equal(a, b))
        n_ok += same
        ga = cv2.cvtColor(a, cv2.COLOR_BGR2GRAY)
        gb = cv2.cvtColor(b, cv2.COLOR_BGR2GRAY)
        n_grey_ok += bool(np.array_equal(ga, gb))
        if not same:
            bad.append((t_rec, "bgr differs"))
    res.update({"bgr_equal": n_ok, "grey_equal": n_grey_ok, "unchanged_equals_color": n_unch, "shapes": dict(shapes), "bad": bad[:10]})
    rep["image"] = res
    n = len(col.image_samples)
    ok = n > 0 and n_ok == n and n_grey_ok == n
    checks.add("S2", "CompressedImage format; replay direct decode vs republish path, bgr8 and grey pixel-exact (I2)",
               "PASS" if ok else ("SKIP" if n == 0 else "FAIL"),
               "formats %s; %d sampled: bgr equal %d, grey equal %d, IMREAD_UNCHANGED==IMREAD_COLOR %d; shapes %s (Python emulation of both decode paths with OpenCV; the C++ replay path itself is to be measured by WP-f)" % (
                   dict(col.image_formats), n, n_ok, n_grey_ok, n_unch, dict(shapes)))


def analyse_gt(path, col, checks, rep):
    if not path:
        checks.add("S3", "GT file format", "SKIP", "no --gt given")
        return
    rows, bad_lines, ncols = [], 0, collections.Counter()
    with open(path) as f:
        for ln in f:
            s = ln.strip()
            if not s or s.startswith("#"):
                continue
            parts = s.replace(",", " ").split()
            ncols[len(parts)] += 1
            try:
                rows.append([float(x) for x in parts[:8]])
            except ValueError:
                bad_lines += 1
    if not rows:
        checks.add("S3", "GT file format", "FAIL", "no numeric rows in %s" % path)
        return
    a = np.array([r for r in rows if len(r) == 8])
    t = a[:, 0]
    dt = np.diff(t)
    q = a[:, 4:8]
    qn = np.linalg.norm(q, axis=1)
    ident = np.all(np.abs(q - np.array([0, 0, 0, 1.0])) < 1e-9, axis=1)
    pos = a[:, 1:4]
    same_pos = np.all(np.abs(np.diff(pos, axis=0)) < 1e-12, axis=1)
    res = {"path": path, "rows": int(a.shape[0]), "column_counts": dict(ncols), "bad_lines": bad_lines,
           "t_first": float(t[0]), "t_last": float(t[-1]), "dt": robust.summary(dt), "nonincreasing": int(np.sum(dt <= 0)),
           "backwards": int(np.sum(dt < 0)), "exact_duplicate_stamps": int(np.sum(dt == 0)), "max_backward_s": float(-dt.min()) if dt.size and dt.min() < 0 else 0.0,
           "quat_norm": robust.summary(qn), "quat_identity_frac": float(np.mean(ident)),
           "repeated_position_frac": float(np.mean(same_pos)) if same_pos.size else 0.0,
           "first_position": pos[0].tolist(), "extent_m": (pos.max(axis=0) - pos.min(axis=0)).tolist()}
    rtk = col.topic.get(RTK_TOPIC)
    if rtk and rtk["hdr"]:
        h = np.sort(np.array(rtk["hdr"]))          # header stamps are not monotonic (3 copies per fix, some 16 us earlier)
        idx = np.clip(np.searchsorted(h, t), 1, h.size - 1)
        near = np.minimum(np.abs(t - h[idx - 1]), np.abs(t - h[idx]))
        res["gt_stamps_equal_rtk_header_1us"] = float(np.mean(near < 1e-6))
        res["rtk_messages"] = int(h.size)
    res["bag_record_range"] = None
    rep["gt_format"] = res
    ok = set(ncols) == {8} and bad_lines == 0
    checks.add("S3", "GT file format (TUM: t x y z qx qy qz qw)", "PASS" if ok and res["nonincreasing"] == 0 else ("WARN" if ok else "FAIL"),
               "%d rows, columns %s; %d non-increasing stamps (%d backwards, max %.1f us, %d exact duplicates); quaternion identity on %.1f%% rows (orientation not provided); repeated positions %.1f%%; GT stamps = %s header stamps on %s of rows" % (
                   res["rows"], dict(ncols), res["nonincreasing"], res["backwards"], 1e6 * res["max_backward_s"], res["exact_duplicate_stamps"],
                   100 * res["quat_identity_frac"], 100 * res["repeated_position_frac"], RTK_TOPIC,
                   ("%.1f%%" % (100 * res["gt_stamps_equal_rtk_header_1us"])) if "gt_stamps_equal_rtk_header_1us" in res else "n/a"))


# ------------------------------------------------------------------------------------------------------------
# report
# ------------------------------------------------------------------------------------------------------------
def write_markdown(path, seq, rep, checks):
    L = []
    L.append("# I2 数据审计 — %s" % seq)
    L.append("")
    L.append("生成:`tools/m3dgr_audit.py`(tool_version %d),%s;bag `%s`(%s 字节,%s;sha256 %s)。序列类别:**%s**(AMENDMENTS A2)。" % (
        TOOL_VERSION, rep["generated_utc"], rep["bag"]["path"], rep["bag"]["size"], rep["bag"]["mtime_utc"], rep["bag"].get("sha256") or "未计算", rep["sequence_class"]))
    if rep["sequence_class"] == "evaluation":
        L.append("")
        L.append("> 评测集:本报告只做格式/解码审计。没有读取任何估计轨迹;不做 GNSS-only 解对接收机 PVT 的比较(§12.6-5 自检 G12a/b)与杆臂方向(§12.6-7 自检 G13),两者在评测集上 SKIP(A2);G11 只比较多普勒两种符号假设的拟合残差(解码性质)。")
    L.append("")
    L.append("总评:**%s**(%s)。远端归档:`%s`。" % (checks.worst(), ", ".join("%s %d" % kv for kv in sorted(checks.counts().items())), rep["archive_dir"]))
    L.append("")
    L.append("## 供 WP-a / 核心使用的结论")
    L.append("")
    for line in rep["conclusions"]:
        L.append("- " + line)
    L.append("")
    L.append("## 检查表")
    L.append("")
    L.append(checks.markdown())
    L.append("")
    L.append("## 话题清单")
    L.append("")
    trs = []
    for c in rep["connections"]:
        tp = rep["topics"].get(c["topic"], {})
        trs.append((c["topic"], c["datatype"], c["md5sum"], "yes" if c["md5_recomputed"] == c["md5sum"] else "NO",
                    tp.get("n", 0), report.fmt(tp.get("hdr_rate_hz") or tp.get("rate_avg_hz"), 2),
                    report.fmt(tp.get("rec_minus_hdr_ms", {}).get("median"), 1) if tp.get("rec_minus_hdr_ms") else "—"))
    L.append(report.table(("topic", "type", "md5", "md5 recomputed", "msgs", "rate [Hz] (header, else avg)", "record-header median [ms]"), trs))
    L.append("")
    L.append("## 信号直方图 (sys, freq, code 字段 → signal_table)")
    L.append("")
    L.append(report.table(("sys", "freq [MHz]", "code field", "table code", "count"),
                          [(r["sys"], r["freq_mhz"], r["code_field"], r["table_code"], r["count"]) for r in rep.get("signal_histogram", [])]))
    L.append("")
    if rep.get("eph_compare", {}).get("semantics"):
        L.append("## 星历字段语义(bag vs 同日 RINEX,按 (sat,toe) 配对的计数)")
        L.append("")
        sem = rep["eph_compare"]["semantics"]
        keys = sorted({k for v in sem.values() for k in v})
        L.append(report.table(["sys"] + keys, [[s] + [v.get(k, 0) for k in keys] for s, v in sorted(sem.items())]))
        L.append("")
    L.append("## 产物")
    L.append("")
    L.append("`audit.json`(全部数字)、`latency.csv`、`eph_coverage.csv`、`eph_coverage_timeline.csv`、`plot_*.png`,位于远端归档目录。")
    L.append("")
    with open(path, "w") as f:
        f.write("\n".join(L))


def conclusions(rep, checks):
    c = []
    us = rep.get("ura_semantics", {})
    if us.get("config_value"):
        c.append("`gnss.ephem.ura_semantics: %s`(A16;判定依据见 E4)。" % us["config_value"])
    elif rep.get("n_bag_kepler_eph", 0) == 0:
        c.append("bag 里没有 Kepler 星历(%s):URA 语义(A16)在本序列没有作用对象——星历全部来自 RINEX,其 sva 由 RTKLIB 自解(索引语义,WP-a 契约);"
                 "E4 无从判定。validateConfig 仍要求 `source: gnss_comm_ros1` 时给出取值:同一接收机与 ublox_driver 的开发集/评测集序列(Grass01/02、Dark01、Outdoor01)判定一致为 metres,"
                 "本序列配置沿用 `metres`(推断:同一驱动的字段语义不随序列变;对本序列数值无影响)。" % (
                     "无 /ublox_driver/ephem 话题" if gnss_msgs.TOPIC_EPHEM in rep.get("missing_required_topics", []) else "话题存在但无消息"))
    elif us:
        c.append("URA 语义未能判定(E4):%s —— 本序列 YAML 不得写 ura_semantics,核心将拒绝启动(validateConfig)。" % us.get("decision"))
    su = rep.get("std_units")
    if su:
        c.append("std 单位(§12.6-3):psr_std=%s,cp_std=%s,dopp_std=%s;cp_std=0.06 是 u-blox cpStdev=15 的『无效』标记,占 %.1f%%。" % (
            su["psr"]["unit"], su["cp"]["unit"], su["dopp"]["unit"], 100 * su["cp"]["frac_invalid_marker"]))
    cov = rep.get("eph_coverage")
    if cov:
        c.append("星历覆盖(A1):卫星-历元覆盖率 仅 bag %.1f%% / bag+RINEX %.1f%%;≥5 星历元比例 %.1f%% / %.1f%% ⇒ %s。" % (
            100 * cov["sat_epoch_fraction"]["bag_only"], 100 * cov["sat_epoch_fraction"]["bag_rinex"],
            100 * cov["frac_epochs_ge5"]["bag_only"], 100 * cov["frac_epochs_ge5"]["bag_rinex"],
            "必须配置 `gnss.nav_rinex`" if cov["sat_epoch_fraction"]["bag_rinex"] - cov["sat_epoch_fraction"]["bag_only"] > 0.05 else "bag 星历基本够用,仍建议配 nav_rinex"))
    io = rep.get("iono")
    if io is not None:
        cls = [x["class"] for x in io.get("classification", [])]
        src = "RINEX 头 GPSA/GPSB(本日可用)" if io["rinex_gpsa_gpsb"] else "**本日 RINEX 头无 GPSA/GPSB ⇒ 核心 I=0、σ_ion=5 m**"
        if io["bag_messages"] and cls:
            c.append("Klobuchar(A38):GPS 参数取 %s;bag 的 iono_params 只记诊断,E6 判为 %s(发布节奏 %.1f s%s)。" % (
                src, "/".join(sorted(set(cls))), io.get("publish_interval_median_s") or float("nan"),
                ",与 BDS D1 子帧 1 在各北斗星上轮播相符" if "BDS-like" in cls else ""))
        elif io["bag_messages"]:
            c.append("Klobuchar(A38):GPS 参数取 %s;bag 的 iono_params 值个数异常(见 E6),只记诊断。" % src)
        else:
            c.append("Klobuchar(A38):GPS 参数取 %s;bag 无 iono_params。" % src)
    sem = rep.get("eph_compare", {}).get("semantics", {})
    if "C" in sem and sem["C"].get("n"):
        sc = sem["C"]
        c.append("BDS 星历字段(E1 对拍 %d 条):toe(GnssTimeMsg)是 GPST(%d/%d);`toe_tow` 字段是 BDT 周内秒的只有 %d 条、`week` 字段是 GPS 周的 %d 条 ⇒ 核心须按 §12.3 由 GPST toe 重算 toes(BDT)与 BDT 周,不得直接用消息的 toe_tow/week。" % (
            sc["n"], sc.get("toe_msg_is_gpst", 0), sc["n"], sc.get("toe_tow_is_system_tow", 0), sc.get("week_is_gps_week", 0)))
    if rep.get("freq_code_inconsistent"):
        c.append("观测频点字段与信号码不一致(G3):%s ⇒ ublox_driver 把这些槽的 freqs 写错;按 §12.3 频点表它们被拒收,主信号不受影响,只失去该系统的第二频点 GF 周跳检测(如需使用,可在 gnss_io 按 code 映射,需维护者决定)。" % "; ".join(rep["freq_code_inconsistent"]))
    conf = rep.get("eph_compare", {}).get("conflicts", [])
    if conf:
        c.append("星历流一致性(E7):%d 个 (sat,toe,IODE) 键出现内容不同的副本(%s)⇒ EphemStore 对同键新副本应比对内容,不一致即计数并保留先到/RINEX 版本,不能后到覆盖。" % (
            len(conf), ", ".join("%s:%s" % (x["sat"], "/".join(x["fields"])) for x in conf[:4])))
    av = rep.get("availability", {})
    cn = rep.get("cn0", {})
    if av and cn:
        c.append("C/N0 中位 %s dB-Hz;按 DESIGN §10 L0 掩码(伪距 30 / 多普勒 25 / 载波 28 dB-Hz)可用率 伪距 %s、多普勒 %s ⇒ 默认掩码会饿死滤波与初始化(COLLECT 需 ≥5 星 SPP);掩码必须在开发集按 TEST_PLAN §3 重新标定,不得在本序列上调。" % (
            ", ".join("%s %.0f" % (k, v["median"]) for k, v in sorted(cn.items())),
            ", ".join("%s %.0f%%" % (k, 100 * v["psr_cn0_30"]) for k, v in sorted(av.items())),
            ", ".join("%s %.0f%%" % (k, 100 * v["dopp_cn0_25"]) for k, v in sorted(av.items()))))
    tp = rep.get("time_pulse", {})
    if not tp.get("present", False):
        c.append("time_pulse_info 话题不存在 ⇒ A19 的检查无对象;主机钟台阶只能从 range_meas 记录时间序列看(L1)。")
    lat = rep.get("latency", {})
    if lat:
        r = lat["range_meas_record_minus_gpst_utc_ms"]
        c.append("range_meas 记录时间 − GPST→UTC:中位 %.1f ms、MAD %.1f ms、漂移 %.1f ppm、台阶 %s(= 主机钟偏差+到达延迟;作 §7 方法①先验,不是最终时偏)。" % (
            r["median"], r["mad"], lat["drift_ppm"], lat["steps"] or "无"))
    tps = rep.get("topics", {})
    def rh(t):
        o = tps.get(t, {})
        return o.get("rec_minus_hdr_ms", {}).get("median"), (o.get("rec_minus_hdr_drift_ms_per_s") or 0.0) * 1e3
    li, im, ca = rh("/livox/avia/lidar"), rh("/livox/avia/imu"), rh("/camera/color/image_raw/compressed")
    if li[0] is not None and im[0] is not None:
        tb = rep.get("livox", {}).get("timebase_semantics", "")
        c.append("传感器时基(C4,记录时间 − 头戳中位,漂移):Avia LiDAR %.1f ms / %.0f ppm、Avia IMU %.1f ms / %.0f ppm、相机 %s ms。Livox 头戳 = 设备时间戳 + 常数(S1:%s),与主机时钟的关系逐序列不同%s ⇒ 估计器时基(LiDAR 头戳)对 GPST 的偏差与漂移只能由 §7 方法②逐序列标定;range_meas 记录时间只作先验。" % (
            li[0], li[1], im[0], im[1], report.fmt(ca[0], 1), tb or "未测",
            "(本序列 LiDAR 头戳领先主机收包时刻,不可能是主机收包戳)" if li[0] < 0 else "(本序列与『扫描起点主机时刻 + 约 100 ms 扫描』相容)"))
    sg = rep.get("sign_check", {})
    if sg.get("slope") is not None:
        c.append("载波/多普勒符号(§12.6-2):斜率 %.4f、相关 %.5f ⇒ `λ·dΦ/dt = −λD` 成立,与 DESIGN §1 一致。" % (sg["slope"], sg["corr"]))
    lv = rep.get("livox", {})
    if lv.get("bag"):
        c.append("Livox:%s 与仓库 include/livox_ros_driver 头 md5 %s。" % (
            ", ".join("%s %s [%s]" % (b["topic"], b["datatype"], b["md5sum"][:8]) for b in lv["bag"]), "一致" if all(b["md5sum"] == lv.get("repo_md5") for b in lv["bag"]) else "不一致"))
    im = rep.get("image", {})
    if im.get("formats"):
        c.append("CompressedImage format:%s;回放直读应 `IMREAD_COLOR` 并按 format 决定通道序(S2)。S2 的 PASS 是 Python 对两条解码路径的仿真,C++ 回放路径本身待 WP-f 实测。" % im["formats"])
    g13 = next((x for x in checks.items if x["id"] == "G13"), None)
    if g13 is not None:
        if rep.get("sequence_class") != "development":
            c.append("§12.6-7 杆臂方向:评测集不做(A2,标定级判断只在开发集);calibration.md 方向推导见 docs/dev/audit/m3dgr_calib.md。")
        elif g13["verdict"] == "SKIP":
            c.append("§12.6-7 杆臂方向:%s;calibration.md 方向推导见 docs/dev/audit/m3dgr_calib.md。" % g13["evidence"])
        else:
            c.append("§12.6-7 杆臂方向(G13 %s):%s。" % (g13["verdict"], g13["evidence"]))
    g14 = next((x for x in checks.items if x["id"] == "G14"), None)
    if g14 is not None and g14["verdict"] != "SKIP":
        c.append("自检 1 的 sat2freq 交叉核对(A35-C8,G14):%s。" % ("全部一致" if g14["verdict"] == "PASS" else "**有不一致,见 G14**"))
    return c


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    os.makedirs(args.out_dir, exist_ok=True)
    seq = args.seq or os.path.splitext(os.path.basename(args.bag))[0]
    st = os.stat(args.bag)
    rep = {"tool": "m3dgr_audit.py", "tool_version": TOOL_VERSION, "sequence": seq, "sequence_class": seq_class(seq),
           "generated_utc": datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"), "archive_dir": args.archive_label or os.path.abspath(args.out_dir),
           "bag": {"path": os.path.abspath(args.bag), "size": st.st_size,
                   "mtime_utc": datetime.datetime.fromtimestamp(st.st_mtime, datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")},
           "nav_rinex": args.nav_rinex}
    shaf = args.bag + ".sha256"
    if os.path.exists(shaf):
        with open(shaf) as f:
            rep["bag"]["sha256"] = f.read().split()[0]
    checks = report.Checks()
    rb = rosio.RawBag(args.bag)
    conns = rb.connections()
    bag_start, bag_end = rb.start_end()
    rb.close()
    rep["bag"].update({"start": bag_start, "end": bag_end, "duration": bag_end - bag_start})
    analyse_connections(conns, checks, rep)
    col = collect(args.bag, args, conns)
    rep["collect_seconds"] = col.pass_seconds
    rep["decode_errors"] = dict(col.decode_errors)
    rep["n_bag_kepler_eph"], rep["n_bag_glo_eph"] = len(col.eph), len(col.geph)
    if col.decode_errors:
        checks.add("C0", "Python decode of every message", "FAIL", "decode errors: %s" % dict(col.decode_errors))
    analyse_topics(col, conns, checks, rep, bag_start, bag_end)
    analyse_epochs(col, checks, rep)
    analyse_signals(col, checks, rep)
    analyse_sign(col, checks, rep)
    rnx_header, rnx = load_rinex(args.nav_rinex) if args.nav_rinex else (None, [])
    rep["rinex_records"] = len(rnx)
    t_first = col.epochs[0]["t_gpst"] if col.epochs else 0.0
    t_last = col.epochs[-1]["t_gpst"] if col.epochs else 0.0
    if rnx:
        rnx = [r for r in rnx if t_first - 6 * 3600 <= r["toe"] <= t_last + 6 * 3600]
    analyse_glo_channel(col, rnx, checks, rep)
    analyse_sat2freq(col, rnx, checks, rep)
    analyse_ephemerides(col, rnx_header, rnx, checks, rep, t_first, t_last)
    analyse_coverage(col, rnx, checks, rep, args.out_dir, not args.no_plots)
    analyse_iono(col, rnx_header, checks, rep)
    lat = analyse_latency(col, checks, rep, args.out_dir, not args.no_plots)
    analyse_time_pulse(col, checks, rep, lat)
    # self-checks 5 and 7 only on the development set (A2); an unknown class is treated like an evaluation sequence
    evaluation = rep["sequence_class"] != "development"
    analyse_ls(col, rnx, rnx_header, checks, rep, args.ls_rate, evaluation=evaluation)
    analyse_lever_arm(col, rnx, args, checks, rep, evaluation=evaluation)
    analyse_livox(col, conns, args, checks, rep)
    analyse_images(col, args, checks, rep)
    analyse_gt(args.gt, col, checks, rep)
    rep["conclusions"] = conclusions(rep, checks)
    rep["checks"] = checks.items
    rep["verdict"] = checks.worst()
    report.write_json(os.path.join(args.out_dir, "audit.json"), rep)
    write_markdown(os.path.join(args.out_dir, "audit.md"), seq, rep, checks)
    print("audit %s: %s %s -> %s" % (seq, checks.worst(), checks.counts(), args.out_dir))
    return 0


if __name__ == "__main__":
    sys.exit(main())
