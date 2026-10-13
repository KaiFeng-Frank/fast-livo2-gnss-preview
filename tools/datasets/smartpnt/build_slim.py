#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Build the slim replay bag of one SmartPNT-MSF sequence (A80), run on the remote host inside the project image.

  build_slim.py <seq> --ref-bag <M3DGR bag with the gnss_comm topics> [--root R] [--image-size 640x512] [--only-bags N]
                      [--max-seconds S] [--with-base] [--keep-raw]

What it does (streaming: LiDAR and camera bags are downloaded on demand, one ahead, and deleted behind):
  1. fetch the small files (mini receiver RINEX, brdm, IMU bag, I300.imr, GT .ref, description, [base]) and the shared
     calibration files; sizes and sha256 go to <root>/<seq>/build.json
  2. time rule (dscommon/timebase.py): the stamps' time of day is GPST, the date is wrong; one constant shift per
     sequence maps them to UTC Unix, the date taken from the sequence's own GPS data
  3. IMU increments -> rates (x200), camera JPEG -> processing size, VLP-16 `time` -> microseconds from the first point,
     RINEX observations/ephemerides -> gnss_comm messages (dscommon/gnsscomm.py, definitions copied from --ref-bag)
  4. merge by record time (arrival model in slim.py), write <root>/<seq>/<seq>.bag, verify, hash
  5. convert the ground truth (gt_convert.py) into <root>/<seq>/gt/
The record of everything (decisions, counts, anomalies, hashes) is <root>/<seq>/build.json.
"""
import argparse
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import numpy as np  # noqa: E402

from dscommon import bagmerge, gnsscomm, httpfetch, layout, rinex, rosbytes, timebase  # noqa: E402
from fl2g import gnsstime, rinexnav  # noqa: E402
from smartpnt import fetch_smartpnt, gt_convert, portal, slim  # noqa: E402

DEFAULT_ROOT = "/data2/fl2g/smartpnt"
TOPICS = {"imu": "/imu_raw", "lidar": "/velodyne_points", "camera": "/cam00/image_raw/compressed"}
SRC_TOPICS = {"imu": "/imu_raw", "lidar": "/velodyne_points", "camera": "/cam00/image_raw/compressed"}
NS = 1000000000


def log(msg):
    sys.stderr.write("[build_slim %s] %s\n" % (time.strftime("%H:%M:%S"), msg))
    sys.stderr.flush()


class Prefetcher(object):
    """Downloads entry i+1 in a thread while entry i is being converted."""

    def __init__(self, store, entries):
        from concurrent.futures import ThreadPoolExecutor
        self.store, self.entries = store, entries
        self.ex = ThreadPoolExecutor(max_workers=1)
        self.fut = {}

    def start(self, i):
        if i < len(self.entries) and i not in self.fut:
            self.fut[i] = self.ex.submit(self.store.ensure, self.entries[i])

    def open(self, i):
        self.start(i)
        return self.fut[i].result()

    def close(self, i, keep=False):
        if not keep:
            self.store.release(self.entries[i])


def first_message(path, topic):
    from rosbags.rosbag1 import Reader
    with Reader(path) as r:
        conns = [c for c in r.connections if c.topic == topic]
        if not conns:
            raise ValueError("%s has no topic %s" % (path, topic))
        for c, t, raw in r.messages(connections=conns):
            return t, raw, c
    raise ValueError("%s: topic %s is empty" % (path, topic))


def acquire_build_lock(seqdir):
    """One build per sequence directory at a time (flock on <seqdir>/.build.lock, also across containers sharing the disk):
    two queues that both reach a sequence would otherwise truncate each other's bag and raw files (happened once with two
    queues running the last sequences of the list). Returns the open file; the lock ends with the process."""
    import fcntl
    os.makedirs(seqdir, exist_ok=True)
    f = open(os.path.join(seqdir, ".build.lock"), "w")
    try:
        fcntl.flock(f, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except (IOError, OSError):
        f.close()
        raise SystemExit("another build_slim holds %s/.build.lock: one build per sequence at a time" % seqdir)
    f.write("%d\n" % os.getpid())
    f.flush()
    return f


def finished_build(seqdir, out_bag):
    """True when <seqdir>/build.json records a finished build of exactly this bag file (size and seconds present):
    a re-run of a queue then skips the sequence instead of overwriting a verified bag. build.json is written last."""
    p = os.path.join(seqdir, "build.json")
    if not (os.path.isfile(p) and os.path.isfile(out_bag)):
        return False
    try:
        with open(p) as f:
            rec = json.load(f)
        return rec.get("bag", {}).get("size") == os.path.getsize(out_bag) and "seconds" in rec and rec.get("gt") is not None
    except (ValueError, OSError):
        return False


def parse_size(s):
    w, h = s.lower().split("x")
    return int(w), int(h)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("seq")
    ap.add_argument("--root", default=DEFAULT_ROOT)
    ap.add_argument("--ref-bag", required=True)
    ap.add_argument("--image-size", default="640x512")
    ap.add_argument("--jpeg-quality", type=int, default=95)
    ap.add_argument("--bags", default=None, help="A:B slice of the LiDAR/camera bag lists (probe builds), e.g. 2:12")
    ap.add_argument("--no-camera", action="store_true", help="LiDAR + IMU + GNSS only (analysis / LIO-only bags)")
    ap.add_argument("--no-lidar", action="store_true", help="camera + IMU + GNSS only")
    ap.add_argument("--max-seconds", type=float, default=None, help="stop all streams this many seconds after the first image")
    ap.add_argument("--with-base", action="store_true")
    ap.add_argument("--keep-raw", action="store_true")
    ap.add_argument("--workers", type=int, default=3)
    ap.add_argument("--connections", type=int, default=4)
    ap.add_argument("--max-sigma-3d", type=float, default=0.5)
    ap.add_argument("--max-q", type=int, default=3)
    ap.add_argument("--lidar-time-unit", choices=["s", "ms", "us", "ns"], default=None)
    ap.add_argument("--out", default=None)
    ap.add_argument("--project-root", default="/data2/fl2g",
                    help="second project root for run_matrix/eval_ate: links bags/, fetch record and audit/<seq>/gt (dscommon/layout.py)")
    ap.add_argument("--refresh-tree", action="store_true")
    ap.add_argument("--rebuild", action="store_true", help="build again although a finished build (build.json with the bag's size) exists")
    a = ap.parse_args(argv)

    t_start = time.time()
    seqdir = os.path.join(a.root, a.seq)
    out_bag = a.out or os.path.join(seqdir, a.seq + ".bag")
    lock = acquire_build_lock(seqdir)           # held until the process ends
    if not a.rebuild and finished_build(seqdir, out_bag):
        log("%s is already built (build.json matches the bag); --rebuild to build again" % a.seq)
        return 0
    files, lay = fetch_smartpnt.load_layout(a.root, a.seq, a.refresh_tree)
    store = portal.RawStore(seqdir, a.connections)
    cfg_store = portal.RawStore(os.path.join(a.root, "_config"), a.connections)
    record = {"tool": "tools/datasets/smartpnt/build_slim.py", "sequence": a.seq, "platform": portal.ROOT_PREFIX + fetch_smartpnt.PLATFORM,
              "args": {k: v for k, v in vars(a).items()}, "warnings": []}

    # 1. small files + calibration
    log("fetching small files")
    small = [lay.obs_mini, lay.nav, lay.imu_bag] + lay.gt + lay.desc + ([lay.imr] if lay.imr else [])
    if a.with_base and lay.base:
        small.append(lay.base)
    for e in small:
        store.ensure(e)
    for r in portal.CONFIG_FILES:
        if r in files:
            cfg_store.ensure(files[r])
    p_obs, p_nav, p_imu = store.path(lay.obs_mini), store.path(lay.nav), store.path(lay.imu_bag)

    # 2. reference times and the shift
    imu_t0, imu_raw0, imu_conn = first_message(p_imu, SRC_TOPICS["imu"])
    imu_first = rosbytes.parse_imu(imu_raw0)["stamp_ns"]
    f_obs, hdr, sel = rinex.open_obs(p_obs)
    wk, tow = rinex.gpst_from_calendar(*hdr.first_obs, time_system=hdr.time_system, leap_s=hdr.leap_s)
    ref_gpst = gnsstime.gpst(wk, tow)
    shift, cand, leap = timebase.resolve_shift_ns(imu_first, ref_gpst)
    record["time_rule"] = {
        "imu_first_stamp_ns": imu_first, "imu_first_tod_s": timebase.tod_ns(imu_first) / 1e9,
        "reference": "first epoch of the mini receiver RINEX (GPST week %d tow %.3f, header time system %s)" % (wk, tow, hdr.time_system),
        "gpst_first_imu_s": cand, "leap_s": leap, "shift_ns": shift,
        "rule": "stamp_new = stamp_old + shift_ns; stamp_old time of day == GPST time of day (IMU stamps equal the I300 IMR "
                "record times); stamp_new is UTC Unix = GPST - leap"}
    log("time shift %+d ns (%.1f days %+.0f s), leap %d, IMU first GPST tod %.3f" % (
        shift, shift / 86400e9, (shift % (86400 * NS)) / 1e9, leap, timebase.tod_ns(imu_first) / 1e9))

    # connections of the output bag
    refs = gnsscomm.load_reference_defs(a.ref_bag)
    conns = {TOPICS["imu"]: bagmerge.conn_from_source(imu_conn, TOPICS["imu"])}
    sl = slice(*[int(x) if x else None for x in a.bags.split(":")]) if a.bags else slice(None)
    lid_pf = Prefetcher(store, [] if a.no_lidar else lay.lidar[sl])
    cam_pf = Prefetcher(store, [] if a.no_camera else lay.cam[sl])
    first_sensor = []
    if lid_pf.entries:
        lid_pf.start(0)
        _, lid_raw0, lid_conn = first_message(lid_pf.open(0), SRC_TOPICS["lidar"])
        conns[TOPICS["lidar"]] = bagmerge.conn_from_source(lid_conn, TOPICS["lidar"])
        first_sensor.append((("LiDAR"), rosbytes.Cloud(lid_raw0).stamp_ns + shift))
    if cam_pf.entries:
        cam_pf.start(0)
        cam_t0, cam_raw0, cam_conn = first_message(cam_pf.open(0), SRC_TOPICS["camera"])
        conns[TOPICS["camera"]] = bagmerge.conn_from_source(cam_conn, TOPICS["camera"])
        first_sensor.append(("camera", rosbytes.parse_compressed(cam_raw0)["stamp_ns"] + shift))
    for topic in (gnsscomm.TOPIC_MEAS, gnsscomm.TOPIC_EPHEM, gnsscomm.TOPIC_GLO_EPHEM, gnsscomm.TOPIC_IONO):
        c = refs.get(topic)
        conns[topic] = bagmerge.Conn(topic, c["msgtype"], c["md5"], c["msgdef"], c["callerid"], c["latching"])
    imu_first_new = imu_first + shift
    for name, t in first_sensor:
        if abs(t - imu_first_new) > 15 * 60 * NS:           # the streams must share one clock (one bogus date); minutes apart is normal
            raise SystemExit("first %s stamp is %.0f s from the first IMU stamp after the time rule: the sensors are not on one clock" %
                             (name, (t - imu_first_new) / 1e9))
    sensors_first = max(t for _, t in first_sensor) if first_sensor else imu_first_new
    # a full build starts at the first IMU sample (the host initialises on it); a bag slice at its first sensor message
    win_lo = imu_first_new if a.bags is None else sensors_first - NS
    win_hi = (sensors_first + int(a.max_seconds * NS)) if a.max_seconds else None
    record["window_utc_ns"] = [win_lo, win_hi]

    # 3. streams
    stats = {"imu": {}, "lidar": {}, "camera": {}, "gnss": {}}
    rule = slim.LidarTimeRule(a.lidar_time_unit)
    slimmer = slim.ImageSlimmer(parse_size(a.image_size), a.jpeg_quality)

    def upto(stream):
        for ev in stream:
            if win_hi is not None and ev[0] > win_hi:
                return
            yield ev

    def imu_stream():
        wrap = timebase.Unwrapper()
        for t, raw, c in bagmerge.iter_bag(p_imu, SRC_TOPICS["imu"]):
            stamp, payload = slim.imu_to_rates(raw, shift, 0, wrap)
            yield stamp, TOPICS["imu"], payload
        stats["imu"]["wraps"] = wrap.wraps

    notes = []

    def lidar_stream():
        wrap = timebase.Unwrapper()
        lb = bagmerge.LazyBags(len(lid_pf.entries), lid_pf.open, lid_pf.close, SRC_TOPICS["lidar"], lid_pf.start)
        lb.notes = notes
        n = 0
        for t, raw in lb:
            r = slim.lidar_to_slim(raw, shift, rule, wrap)
            if r is None:
                continue
            record_ns, stamp, payload, info = r
            n += 1
            yield record_ns, TOPICS["lidar"], payload
        stats["lidar"]["wraps"] = wrap.wraps

    def cam_stream():
        wrap = timebase.Unwrapper()
        cb = bagmerge.LazyBags(len(cam_pf.entries), cam_pf.open, cam_pf.close, SRC_TOPICS["camera"], cam_pf.start)
        cb.notes = notes

        # the unwrapper is stateful: stamps are unwrapped in source order before the (parallel) JPEG work
        def pre():
            for t, raw in cb:
                d = rosbytes.parse_header(raw)
                st = wrap(d[1])
                yield st, raw

        def work(item):
            st, raw = item
            d = rosbytes.parse_compressed(raw)
            payload = rosbytes.build_compressed(0, st + shift, d["frame_id"], d["format"], slimmer(d["data"]))
            return st + shift + slim.CAMERA_LATENCY_NS, TOPICS["camera"], payload

        for ev in bagmerge.pmap(work, pre(), a.workers, 4 * a.workers):
            yield ev
        stats["camera"]["wraps"] = wrap.wraps

    gstats = {}

    def gnss_obs_stream():
        n = 0
        for ep in rinex.iter_epochs(f_obs, hdr, sel.wanted()):
            t_utc = timebase.utc_ns_of_gpst(ep.week, ep.tow)
            if t_utc < win_lo - 2 * NS:
                continue
            obs = gnsscomm.epoch_to_obs(ep, sel, gstats)
            if not obs:
                continue
            n += 1
            yield t_utc + slim.GNSS_LATENCY_NS, gnsscomm.TOPIC_MEAS, gnsscomm.pack_meas(obs)
        gstats["epochs"] = n

    nav_hdr, nav_recs = rinexnav.read_nav(p_nav)
    t_lo_g = (win_lo - imu_first_new) / 1e9 + cand
    t_hi_g = cand + (a.max_seconds + 30 if a.max_seconds else 3 * 3600.0 + 7200.0)
    # span of the full sequence: from the cam bags' count is unknown here; the bag's IMU span is exact
    from rosbags.rosbag1 import Reader
    with Reader(p_imu) as r:
        imu_span_s = (r.end_time - r.start_time) / 1e9
    if not a.max_seconds:
        t_hi_g = cand + imu_span_s + 60.0
    nav_msgs = gnsscomm.nav_messages(nav_recs, t_lo_g, t_hi_g)
    data_end_ns = (imu_first_new + int(round(imu_span_s * NS))) if not a.max_seconds else win_hi
    iono = gnsscomm.iono_values(nav_hdr)

    def gnss_nav_stream():
        evs = []
        if iono:
            evs.append((win_lo, gnsscomm.TOPIC_IONO, gnsscomm.pack_iono(iono, win_lo)))
        for t_tx, topic, fields in nav_msgs:
            t_rec = max(timebase.utc_ns_of_gpst(*gnsstime.week_tow(t_tx)), win_lo + 1)
            raw = gnsscomm.pack_ephem(fields) if topic == gnsscomm.TOPIC_EPHEM else gnsscomm.pack_glo_ephem(fields)
            evs.append((t_rec, topic, raw))
        evs.sort(key=lambda e: e[0])
        for ev in evs:
            if ev[0] > data_end_ns:                 # an ephemeris transmitted after the last sensor sample says nothing
                continue
            yield ev

    def window(stream):
        for ev in stream:
            if ev[0] < win_lo:
                continue
            yield ev

    cand_streams = [("imu", imu_stream(), "imu"), ("gnss_obs", gnss_obs_stream(), "gnss"), ("gnss_nav", gnss_nav_stream(), "gnss")]
    if lid_pf.entries:
        cand_streams.append(("lidar", lidar_stream(), "lidar"))
    if cam_pf.entries:
        cand_streams.append(("camera", cam_stream(), "camera"))
    mons = [bagmerge.Monotonic(upto(window(st)), name) for name, st, _ in cand_streams]
    ranks = [bagmerge.RANK[kind] for _, _, kind in cand_streams]
    merged = bagmerge.merge(list(zip(ranks, mons)))

    def progress(n, t):
        log("%d messages, record time %s UTC" % (n, gnsstime.utc_iso(t / 1e9)))

    os.makedirs(os.path.dirname(out_bag), exist_ok=True)
    log("writing %s" % out_bag)
    wstats = bagmerge.write_bag(out_bag, conns, merged, progress)
    for m in mons:
        if m.clamped:
            record["warnings"].append("%s stream: %d messages had a record time before their predecessor (clamped)" % (m.name, m.clamped))

    # 4. verify and hash
    log("verifying")
    from rosbags.rosbag1 import Reader
    with Reader(out_bag) as r:
        counts = {}
        for c in r.connections:
            counts[c.topic] = counts.get(c.topic, 0) + c.msgcount
        record["bag"] = {"start_ns": r.start_time, "end_ns": r.end_time, "duration_s": (r.end_time - r.start_time) / 1e9,
                         "messages": r.message_count}
    for topic, s in wstats.items():
        if counts.get(topic, 0) != s["n"]:
            raise RuntimeError("topic %s: wrote %d, bag index says %d" % (topic, s["n"], counts.get(topic, 0)))
    record["topics"] = {t: dict(s, first_utc=gnsstime.utc_iso(s["first_ns"] / 1e9) if s["first_ns"] else None,
                                last_utc=gnsstime.utc_iso(s["last_ns"] / 1e9) if s["last_ns"] else None)
                        for t, s in wstats.items()}
    record["bag"]["sha256"] = httpfetch.sha256_file(out_bag)
    record["bag"]["size"] = os.path.getsize(out_bag)
    record["lidar_time"] = {"unit": rule.unit, "source_time_reference_scans": rule.reference, "scans": rule.scans, "unit_flips": rule.flips, "reordered_scans": rule.nonmonotonic,
                            "span_s_min": rule.span_min, "span_s_max": rule.span_max,
                            "rule": "absolute time of a point = header stamp + time; slim stamp = first point, time in us from it"}
    record["gnss"] = {"obs": gstats, "ephemeris_messages": len(nav_msgs), "iono": iono, "rinex_header": {
        "version": hdr.version, "time_system": hdr.time_system, "interval": hdr.interval, "glo_fcn": hdr.glo_fcn,
        "receiver": hdr.receiver, "program": hdr.program, "obs_types": hdr.obs_types,
        "slots": {s: {str(k): v for k, v in d.items()} for s, d in sel.slots.items()}}}
    record["arrival_model_ns"] = {"imu": 0, "lidar": "frame end", "camera": slim.CAMERA_LATENCY_NS, "gnss": slim.GNSS_LATENCY_NS}
    record["camera"] = {"size": list(slimmer.size), "jpeg_quality": a.jpeg_quality, "interpolation": "INTER_LINEAR (the host's own)"}
    f_obs.close()

    # 5. ground truth
    log("converting ground truth")
    names = {e.rel.split("/")[-1]: e for e in lay.gt}
    readme = [e for e in lay.desc if e.rel.endswith("README.xml")]
    gt_audit = gt_convert.convert(store.path(lay.gt_primary), store.path(names["I300_GroundTruth.ref"]), os.path.join(seqdir, "gt"),
                                  a.seq, a.max_sigma_3d, a.max_q, store.path(readme[0]) if readme else None, lay.mini_id)
    if gt_audit["readme_check"] and not gt_audit["readme_check"].get("ok"):
        record["warnings"].append("README.xml and the .ref header disagree about the mini antenna: %s" % gt_audit["readme_check"])
    record["gt"] = {k: gt_audit[k] for k in ("readme_check", "primary", "i300", "epochs", "t_first_utc", "t_last_utc", "lever_i300_to_antenna_i300_frame",
                                             "i300_transfer_residual_m", "quality", "mask")}
    # replay inputs that stay on disk next to the bag: the broadcast ephemeris (gnss.nav_rinex), the I300 IMR (time check),
    # the ground-truth .ref files and the description; the rest of the small files (RINEX, IMU bag) is released
    keep = [lay.nav] + ([lay.imr] if lay.imr else []) + list(lay.gt) + list(lay.desc)
    record["replay_inputs"] = {"nav_rinex": store.path(lay.nav), "mini_receiver_rinex_portal_path": lay.obs_mini.rel,
                               "imr_i300": store.path(lay.imr) if lay.imr else None, "gt_dir": os.path.join(seqdir, "gt")}
    if not a.keep_raw:
        for e in small:
            if e not in keep:
                store.release(e)
    if a.project_root and not a.out:
        record["layout"] = layout.publish(a.project_root, "smartpnt", a.seq, out_bag, os.path.join(seqdir, "gt"), [store.path(lay.nav)],
                                          record["bag"]["sha256"], {"duration_s": record["bag"]["duration_s"]})
    if notes:
        record["reindexed_source_bags"] = notes
        record["warnings"].append("%d source bag(s) were unindexed (recording cut off) and reindexed: the incomplete tail chunk is lost" % len(notes))
    record["raw_files"] = dict(store.manifest)
    record["config_files"] = dict(cfg_store.manifest)
    record["seconds"] = round(time.time() - t_start, 1)
    with open(os.path.join(seqdir, "build.json"), "w") as f:
        json.dump(record, f, indent=1, sort_keys=True, default=str)
        f.write("\n")
    log("done in %.0f s: %s" % (record["seconds"], out_bag))
    return 0


if __name__ == "__main__":
    sys.exit(main())
