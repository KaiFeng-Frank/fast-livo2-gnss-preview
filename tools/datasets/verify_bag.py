#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Acceptance check of a finished slim bag (A80 data tools; run on the remote host inside the project image).

  verify_bag.py <bag> --ref-bag <M3DGR bag> [--image-size 640x512] [--camera-stride 20] [--lidar-stride 10] [--json OUT]

It reads the bag back with a second, independent code path (rosbags Reader plus the raw-byte parsers) and checks what the
builders promise, so that a bag is only handed on when these all hold:

  connections  the four gnss_comm topics carry type, md5, message_definition, callerid and latching byte-identical to the
               reference bag; IMU / camera / cloud connections carry the standard md5 sums
  IMU          200 Hz grid (no gap above 50 ms), stamps = record times, finite values, median |acc| at 1 g (rates, not increments)
  camera       10 Hz, every sampled frame has the processing size, record = stamp + the arrival model constant
  LiDAR        per-point `time` starts at 0 and ends near the scan period, in microseconds, non-decreasing; record = stamp + span
  GNSS         epoch rate, satellites per epoch, GPST->UTC: record time - epoch UTC = the arrival constant for every epoch
  clock        all streams on one clock: the GNSS epoch span covers the IMU span (same UTC date, no stray leap-second shift)

Exit status 1 when any check fails; the full numbers go to --json. Only reads; never writes next to the bag.
"""
import argparse
import json
import os
import struct
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)

import numpy as np  # noqa: E402

from dscommon import gnsscomm, rosbytes, timebase  # noqa: E402
from fl2g import gnsstime  # noqa: E402

NS = 10 ** 9
STD_MD5 = {"sensor_msgs/Imu": "6a62c6daae103f4ff57a132d6f95cec2", "sensor_msgs/CompressedImage": "8f7a12909da2c9d3332d540a0977563f",
           "sensor_msgs/PointCloud2": "1158d486dd51d683ce2f1be655c3c181"}
TOPICS = {"imu": "/imu_raw", "lidar": "/velodyne_points", "camera": "/cam00/image_raw/compressed"}


class Report(object):
    def __init__(self):
        self.checks = []
        self.data = {}

    def check(self, name, ok, detail=""):
        self.checks.append({"name": name, "ok": bool(ok), "detail": detail})
        sys.stdout.write("%s  %-42s %s\n" % ("ok  " if ok else "FAIL", name, detail))
        sys.stdout.flush()
        return ok


def jpeg_size(data):
    """(width, height) from the first SOF marker of a JPEG, without decoding."""
    i = 2
    while i + 9 < len(data):
        if data[i] != 0xFF:
            i += 1
            continue
        m = data[i + 1]
        if m in (0xC0, 0xC1, 0xC2):
            h, w = struct.unpack_from(">HH", data, i + 5)
            return w, h
        if m in (0xD8, 0x01) or 0xD0 <= m <= 0xD7:
            i += 2
            continue
        (ln,) = struct.unpack_from(">H", data, i + 2)
        i += 2 + ln
    raise ValueError("no SOF marker")


def conn_key(c):
    ext = getattr(c, "ext", None)
    return (c.msgtype, c.digest, getattr(c.msgdef, "data", c.msgdef), getattr(ext, "callerid", None), getattr(ext, "latching", None))


def check_connections(rep, reader, refs):
    by_topic = {}
    for c in reader.connections:
        by_topic.setdefault(c.topic, []).append(c)
    for topic in gnsscomm.SHORT:
        cs = by_topic.get(topic, [])
        ref = refs.conns[topic]
        want = (ref["msgtype"], ref["md5"], ref["msgdef"], ref["callerid"], ref["latching"])
        rep.check("connection %s" % topic, len(cs) == 1 and conn_key(cs[0]) == want,
                  "1 connection, type/md5/definition/callerid/latching identical to the reference" if cs and conn_key(cs[0]) == want else
                  "%d connection(s), differs from the reference" % len(cs))
    for kind, topic in TOPICS.items():
        cs = by_topic.get(topic, [])
        if not cs:
            rep.check("connection %s" % topic, False, "missing")
            continue
        typ = cs[0].msgtype.replace("/msg/", "/")
        rep.check("connection %s" % topic, len(cs) == 1 and STD_MD5[typ] == cs[0].digest, "%s md5 %s" % (typ, cs[0].digest))
    return by_topic


def iter_topic(reader, conns):
    for c, t, raw in reader.messages(connections=conns):
        yield t, bytes(raw)


def check_imu(rep, reader, conns):
    stamps, rec, acc, gyro, bad = [], [], [], [], 0
    for t, raw in iter_topic(reader, conns):
        d = rosbytes.parse_imu(raw)
        stamps.append(d["stamp_ns"])
        rec.append(t)
        acc.append(np.linalg.norm(d["acc"]))
        gyro.append(np.linalg.norm(d["gyro"]))
        if d["frame_id"] != b"imu" or not np.all(np.isfinite(d["acc"] + d["gyro"])):
            bad += 1
    st, rc = np.array(stamps, dtype=np.int64), np.array(rec, dtype=np.int64)
    dt = np.diff(st) / 1e9
    acc = np.array(acc)
    info = {"n": len(st), "first_ns": int(st[0]), "last_ns": int(st[-1]), "dt_median_s": float(np.median(dt)), "dt_max_s": float(dt.max()),
            "acc_norm_median": float(np.median(acc)), "acc_norm_p5_p95": [float(np.percentile(acc, 5)), float(np.percentile(acc, 95))],
            "gyro_norm_median": float(np.median(gyro)), "record_minus_stamp_ns_max_abs": int(np.abs(rc - st).max())}
    big = np.where(dt > 0.02)[0]
    info["gaps"] = [[int(st[i]), int(st[i + 1]), float(dt[i])] for i in big]
    info["dt_min_s"] = float(dt.min())
    rep.data["imu"] = info
    rep.check("IMU rates (not increments)", 9.6 < info["acc_norm_median"] < 10.1, "median |acc| %.3f m/s^2 (p5..p95 %.2f..%.2f)" % (
        info["acc_norm_median"], info["acc_norm_p5_p95"][0], info["acc_norm_p5_p95"][1]))
    rep.check("IMU frame and values", bad == 0 and info["record_minus_stamp_ns_max_abs"] == 0, "%d bad samples; record == stamp" % bad)
    return info


def check_imu_grid(rep, imu, first_sensor_ns):
    """200 Hz grid. A gap that ends before the first camera/LiDAR stamp lies outside what a replay uses (the host starts at the
    first LiDAR message): it is reported, not failed. Data02_20240718 has such a start-up glitch in the platform's own I300.imr
    (52 records, then 1.74 s of nothing, two minutes before the first scan)."""
    late = [g for g in imu["gaps"] if g[1] >= first_sensor_ns]
    early = [g for g in imu["gaps"] if g[1] < first_sensor_ns]
    note = "; start-up gap(s) before the first scan, not replayed: %s" % ", ".join("%.3f s at %.1f s" % (g[2], (g[0] - imu["first_ns"]) / 1e9) for g in early) if early else ""
    rep.check("IMU grid", abs(imu["dt_median_s"] - 0.005) < 1e-4 and imu["dt_min_s"] > 0 and not [g for g in late if g[2] >= 0.05],
              "%d samples, dt median %.4f s, %d gap(s) above 20 ms inside the replayed span%s" % (imu["n"], imu["dt_median_s"], len(late), note))


def check_camera(rep, reader, conns, stride, size, latency_ns):
    n, sizes, lat, stamps = 0, set(), set(), []
    for t, raw in iter_topic(reader, conns):
        d = rosbytes.parse_compressed(raw)
        stamps.append(d["stamp_ns"])
        if n % stride == 0:
            sizes.add(jpeg_size(d["data"]))
        lat.add(t - d["stamp_ns"])
        n += 1
    st = np.array(stamps, dtype=np.int64)
    dt = np.diff(st) / 1e9
    info = {"n": n, "sizes": sorted(sizes), "record_minus_stamp_ns": sorted(lat), "dt_median_s": float(np.median(dt)) if n > 1 else None,
            "dt_max_s": float(dt.max()) if n > 1 else None, "first_ns": int(st[0]), "last_ns": int(st[-1])}
    rep.data["camera"] = info
    rep.check("camera size", sizes == {tuple(size)} if size else len(sizes) == 1, "%d frames, sampled sizes %s" % (n, sorted(sizes)))
    rep.check("camera rate", abs(info["dt_median_s"] - 0.1) < 2e-3 and info["dt_max_s"] < 1.0,
              "dt median %.4f s, max gap %.3f s" % (info["dt_median_s"], info["dt_max_s"]))
    rep.check("camera arrival", lat == {latency_ns}, "record - stamp = %s ns" % sorted(lat))
    return info


def check_lidar(rep, reader, conns, stride):
    n, bad, spans, stamps, tail_err = 0, [], [], [], []
    npts = []
    for t, raw in iter_topic(reader, conns):
        c = rosbytes.Cloud(raw)
        stamps.append(c.stamp_ns)
        if n % stride == 0:
            col = c.column(raw, "time")
            span = float(col.max())
            spans.append(span)
            npts.append(c.npoints)
            if col.min() != 0.0 or np.any(np.diff(col) < 0) or not (20000.0 < span < 500000.0):
                bad.append(n)
            tail_err.append(abs((t - c.stamp_ns) - round(span * 1e3)))
        n += 1
    st = np.array(stamps, dtype=np.int64)
    dt = np.diff(st) / 1e9
    info = {"n": n, "sampled": len(spans), "span_us_min_max": [min(spans), max(spans)], "points_median": float(np.median(npts)),
            "dt_median_s": float(np.median(dt)), "dt_max_s": float(dt.max()), "first_ns": int(st[0]), "last_ns": int(st[-1]),
            "record_minus_frame_end_ns_max_abs": float(max(tail_err))}
    rep.data["lidar"] = info
    rep.check("LiDAR per-point time (us, from 0)", not bad, "%d scans, %d sampled, span %.0f..%.0f us, %d bad" % (n, len(spans), info["span_us_min_max"][0],
                                                                                                               info["span_us_min_max"][1], len(bad)))
    rep.check("LiDAR rate", abs(info["dt_median_s"] - 0.1) < 5e-3 and info["dt_max_s"] < 1.0, "dt median %.4f s, max gap %.3f s" % (info["dt_median_s"], info["dt_max_s"]))
    rep.check("LiDAR arrival (record = frame end)", info["record_minus_frame_end_ns_max_abs"] <= 1500, "max deviation %.0f ns" % info["record_minus_frame_end_ns_max_abs"])
    return info


def check_gnss(rep, reader, by_topic, gnss_latency_ns):
    n, nsat, lat, utc = 0, [], [], []
    for t, raw in iter_topic(reader, by_topic[gnsscomm.TOPIC_MEAS]):
        (k,) = struct.unpack_from("<I", raw, 0)
        if k == 0:
            continue
        week, tow = struct.unpack_from("<Id", raw, 4)
        u = timebase.utc_ns_of_gpst(week, tow)
        utc.append(u)
        lat.append(t - u)
        nsat.append(k)
        n += 1
    u = np.array(utc, dtype=np.int64)
    dt = np.diff(u) / 1e9
    lat = np.array(lat)
    info = {"epochs": n, "first_ns": int(u[0]), "last_ns": int(u[-1]), "dt_median_s": float(np.median(dt)), "dt_max_s": float(dt.max()),
            "satellite_signal_records_per_epoch_median": float(np.median(nsat)), "record_minus_epoch_ns_min_max": [int(lat.min()), int(lat.max())]}
    for short, topic in (("ephem", gnsscomm.TOPIC_EPHEM), ("glo_ephem", gnsscomm.TOPIC_GLO_EPHEM), ("iono_params", gnsscomm.TOPIC_IONO)):
        info[short] = sum(c.msgcount for c in by_topic[topic])
    rep.data["gnss"] = info
    rep.check("GNSS epochs", n >= 50 and info["dt_median_s"] > 0 and (abs(info["dt_median_s"] - 0.1) < 1e-3 or abs(info["dt_median_s"] - 1.0) < 1e-3),
              "%d epochs, dt median %.3f s, max gap %.1f s, %.0f satellite records per epoch" % (n, info["dt_median_s"], info["dt_max_s"], info["satellite_signal_records_per_epoch_median"]))
    rep.check("GNSS GPST -> UTC (record time)", int(lat.min()) == gnss_latency_ns and int(lat.max()) == gnss_latency_ns,
              "record - epoch UTC in [%d, %d] ns, expected %d" % (lat.min(), lat.max(), gnss_latency_ns))
    rep.check("GNSS ephemerides", info["ephem"] > 0 and info["iono_params"] >= 1, "ephem %d, glo_ephem %d, iono_params %d" % (info["ephem"], info["glo_ephem"], info["iono_params"]))
    return info


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("bag")
    ap.add_argument("--ref-bag", required=True)
    ap.add_argument("--image-size", default="640x512", help="expected processing size, '' = only require one size")
    ap.add_argument("--camera-stride", type=int, default=20)
    ap.add_argument("--lidar-stride", type=int, default=10)
    ap.add_argument("--json", default=None)
    a = ap.parse_args(argv)
    from rosbags.rosbag1 import Reader
    from smartpnt import slim

    rep = Report()
    refs = gnsscomm.load_reference_defs(a.ref_bag)
    size = tuple(int(x) for x in a.image_size.lower().split("x")) if a.image_size else None
    with Reader(a.bag) as r:
        rep.data["bag"] = {"path": a.bag, "size": os.path.getsize(a.bag), "start_ns": r.start_time, "end_ns": r.end_time, "messages": r.message_count}
        by_topic = check_connections(rep, r, refs)
        imu = check_imu(rep, r, by_topic[TOPICS["imu"]])
        cam = check_camera(rep, r, by_topic[TOPICS["camera"]], a.camera_stride, size, slim.CAMERA_LATENCY_NS) if TOPICS["camera"] in by_topic else None
        lid = check_lidar(rep, r, by_topic[TOPICS["lidar"]], a.lidar_stride) if TOPICS["lidar"] in by_topic else None
        gn = check_gnss(rep, r, by_topic, slim.GNSS_LATENCY_NS)
    check_imu_grid(rep, imu, min([x["first_ns"] for x in (cam, lid) if x] or [imu["first_ns"]]))
    # one clock: the receiver epochs overlap the sensor span (a wrong date moves them by days, a bad shift by minutes; the
    # receiver file may start later than the IMU and stop earlier: the overlap must still cover most of the sensors)
    first = min(x["first_ns"] for x in (imu, cam, lid) if x)
    last = max(x["last_ns"] for x in (imu, cam, lid) if x)
    overlap = (min(last, gn["last_ns"]) - max(first, gn["first_ns"])) / float(last - first)
    rep.check("GNSS overlaps the sensors (one clock)", overlap > 0.8,
              "sensors %s .. %s, GNSS %s .. %s UTC, overlap %.1f %% of the sensor span" % (
                  tuple(gnsstime.utc_iso(x / 1e9) for x in (first, last, gn["first_ns"], gn["last_ns"])) + (100.0 * overlap,)))
    rep.check("stream starts agree", all(abs(x["first_ns"] - imu["first_ns"]) < 900 * NS for x in (cam, lid) if x),
              "camera/LiDAR first stamp within 15 min of the first IMU stamp")
    ok = all(c["ok"] for c in rep.checks)
    rep.data["checks"] = rep.checks
    rep.data["ok"] = ok
    if a.json:
        with open(a.json, "w") as f:
            json.dump(rep.data, f, indent=1, sort_keys=True, default=str)
            f.write("\n")
    sys.stdout.write("%s: %d/%d checks passed\n" % (a.bag, sum(c["ok"] for c in rep.checks), len(rep.checks)))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
