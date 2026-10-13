#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Slim replay bag of MARS-LVIG HKisland_GNSS03 (A80 pipeline-debug sequence), remote host, project image.

  build_slim.py [--bag B] [--out O] [--image-size 612x512]

One ordered pass over the source bag. Kept topics (everything else is dropped): /livox/lidar (Avia CustomMsg) and /livox/imu
byte for byte, every /ublox_driver/* topic byte for byte (the F9P stream the repository's gnss_comm path reads), the DJI RTK
topics (position ground truth, velocity, yaw, status and the time-sync records), and the left camera, whose JPEG is shrunk
to the size the host processes (config/camera_MARS_LVIG.yaml: 2448x2048, scale 0.25 -> 612x512; the host's own
cv::resize INTER_LINEAR, so it does not resize again). Record times and the order of messages are the source's. The
ephemeris gap of the stream (the bag's own ephemerides start about 11 s into the bag and five satellites never get one)
is not patched here: the IGS BRDC file fetch_mars.py downloads is injected as gnss.nav_rinex (A1/A38/A45).
Ground truth: gt_convert.py.
"""
import argparse
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, os.path.dirname(os.path.dirname(HERE)))

from dscommon import bagmerge, httpfetch, layout, rosbytes  # noqa: E402
from smartpnt import slim  # noqa: E402
import gt_convert  # noqa: E402

KEEP_PREFIXES = ("/ublox_driver/",)
KEEP_TOPICS = ("/livox/lidar", "/livox/imu", "/left_camera/image/compressed",
               "/dji_osdk_ros/rtk_position", "/dji_osdk_ros/rtk_velocity", "/dji_osdk_ros/rtk_yaw", "/dji_osdk_ros/rtk_info_position",
               "/dji_osdk_ros/rtk_info_yaw", "/dji_osdk_ros/rtk_connection_status", "/dji_osdk_ros/time_sync_gps_utc",
               "/dji_osdk_ros/time_sync_fc_time_utc", "/dji_osdk_ros/time_sync_pps_source", "/dji_osdk_ros/time_sync_nmea_msg")
CAMERA = "/left_camera/image/compressed"


def keep(topic):
    return topic in KEEP_TOPICS or topic.startswith(KEEP_PREFIXES)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--bag", default="/data2/fl2g/mars/HKisland_GNSS03.bag")
    ap.add_argument("--out", default="/data2/fl2g/mars/HKisland_GNSS03_slim.bag")
    ap.add_argument("--image-size", default="612x512")
    ap.add_argument("--jpeg-quality", type=int, default=95)
    ap.add_argument("--workers", type=int, default=3)
    ap.add_argument("--seq", default="HKisland_GNSS03")
    ap.add_argument("--project-root", default="/data2/fl2g", help="second project root (dscommon/layout.py); '' = no links")
    ap.add_argument("--gt-shift-s", type=float, default=0.0, help="GT time shift measured on the development run (README); default: not measured")
    ap.add_argument("--gt-method", default=None, help="how --gt-shift-s was found (xcorr); default none for 0, else xcorr")
    a = ap.parse_args(argv)
    from rosbags.rosbag1 import Reader
    t0 = time.time()
    w, h = [int(x) for x in a.image_size.lower().split("x")]
    slimmer = slim.ImageSlimmer((w, h), a.jpeg_quality)
    counts = {"in": {}, "out": {}}
    with Reader(a.bag) as r:
        conns = [c for c in r.connections if keep(c.topic)]
        kept_topics = sorted({c.topic for c in conns})
        missing = [t for t in KEEP_TOPICS if t not in kept_topics]
        wconns = {}
        for c in conns:
            if c.topic in wconns:
                continue
            wconns[c.topic] = bagmerge.conn_from_source(c)
        sys.stderr.write("[mars build] keeping %d topics, missing from the bag: %s\n" % (len(kept_topics), missing))

        def source():
            for c, t, raw in r.messages(connections=conns):
                counts["in"][c.topic] = counts["in"].get(c.topic, 0) + 1
                yield t, c.topic, raw

        def convert(ev):
            t, topic, raw = ev
            if topic == CAMERA:
                d = rosbytes.parse_compressed(raw)
                raw = rosbytes.build_compressed(d.get("seq", 0), d["stamp_ns"], d["frame_id"], d["format"], slimmer(d["data"]))
            return t, topic, raw

        mons = bagmerge.Monotonic(bagmerge.pmap(convert, source(), a.workers, 4 * a.workers), "mars")
        os.makedirs(os.path.dirname(a.out), exist_ok=True)

        def progress(n, t):
            sys.stderr.write("[mars build] %d messages\n" % n)

        stats = bagmerge.write_bag(a.out, wconns, iter(mons), progress)
        if mons.clamped:
            sys.stderr.write("[mars build] WARNING %d messages clamped to keep record order\n" % mons.clamped)
    record = {"tool": "tools/datasets/mars/build_slim.py", "source": a.bag, "source_sha256": None, "out": a.out,
              "image_size": [w, h], "jpeg_quality": a.jpeg_quality, "kept_topics": kept_topics, "missing_topics": missing,
              "topics": stats, "clamped": mons.clamped, "seconds": round(time.time() - t0, 1)}
    fetch = os.path.join(os.path.dirname(a.bag), a.seq + ".fetch.json")
    fetch_rec = None
    if os.path.isfile(fetch):
        with open(fetch) as f:
            fetch_rec = json.load(f)
        record["source_sha256"] = fetch_rec.get("sha256")
    record["out_sha256"] = httpfetch.sha256_file(a.out)
    record["out_size"] = os.path.getsize(a.out)
    gdir = os.path.join(os.path.dirname(a.out), "gt")
    record["gt"] = gt_convert.convert(a.out, gdir, a.seq, a.gt_shift_s, a.gt_method or ("xcorr" if a.gt_shift_s else "none"))
    if fetch_rec and a.project_root:
        record["layout"] = layout.publish(a.project_root, "mars", a.seq, a.out, gdir, [x["path"] for x in fetch_rec.get("brdc", [])],
                                          record["out_sha256"])
    with open(os.path.join(os.path.dirname(a.out), a.seq + ".build.json"), "w") as f:
        json.dump(record, f, indent=1, default=str)
        f.write("\n")
    sys.stderr.write("[mars build] done in %.0f s: %s\n" % (record["seconds"], a.out))
    return 0


if __name__ == "__main__":
    sys.exit(main())
