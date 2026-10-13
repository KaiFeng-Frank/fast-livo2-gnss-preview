#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Keep only the topics the replay needs (IMPL_PLAN §2 data chain) and drop the rest, bytes untouched.

Keeps /livox/avia/{lidar,imu}, /camera/color/image_raw/compressed and /ublox_driver/* (raw bytes and record
times unchanged); optional time window; writes sha256 of the output. Owner: WP-g.
`--lidar mid360` (AMENDMENTS A76) keeps /livox/mid360/{lidar,imu} instead of the Avia pair; the default is the Avia
pair, so every existing call behaves as before. The bag is a copy of the topics only: write it to a directory of its own
(data/m3dgr/filtered_mid360/), never over a bag of the other lidar.

Implementation (AMENDMENTS A23: Python + rosbags, no C++ target): every kept message is copied as the raw
serialised bytes with its original record timestamp (ns), through a connection re-created from the input
connection's own message_definition, md5sum, callerid and latching (one output connection per input connection,
so duplicate publishers stay distinguishable). After writing, `--verify` (default on) re-reads both bags and
checks that the kept stream is identical message by message (topic, record time ns, sha256 of the bytes) and that
every connection header field (type, md5sum, message_definition, callerid, latching) is preserved.
Run tools/m3dgr_audit.py and tools/gt_audit.py on the ORIGINAL bag (they also read /mavros/global_position/raw/fix,
/odom and /livox/avia/imu), or keep those topics with --keep, before deleting the original (IMPL_PLAN §2 "删原包").
A --start/--duration window drops everything before the window, including the sparse GNSS state messages
(ephemerides every ~30 s, iono): a windowed bag may start without ephemerides (use nav_rinex, A1).
"""
import argparse
import hashlib
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

DEFAULT_KEEP = ("/livox/avia/lidar", "/livox/avia/imu", "/camera/color/image_raw/compressed")
LIDAR_KEEP = {"avia": DEFAULT_KEEP,
              "mid360": ("/livox/mid360/lidar", "/livox/mid360/imu", "/camera/color/image_raw/compressed")}
GNSS_PREFIX = "/ublox_driver/"


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0], formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("bag_in")
    p.add_argument("bag_out")
    p.add_argument("--keep", action="append", default=[], help="extra topic to keep (repeatable)")
    p.add_argument("--lidar", choices=sorted(LIDAR_KEEP), default="avia",
                   help="which LiDAR/IMU pair to keep besides the colour image and /ublox_driver/* (default avia, A76)")
    p.add_argument("--drop-gnss", action="store_true", help="drop /ublox_driver/* (G-no-data gate input)")
    p.add_argument("--start", type=float, default=0.0, help="window start, seconds after the first message")
    p.add_argument("--duration", type=float, default=-1.0, help="window length in seconds (-1 = to the end)")
    p.add_argument("--no-verify", action="store_true", help="skip the read-back verification")
    p.add_argument("--force", action="store_true", help="overwrite bag_out")
    return p


def keep_topic(topic, extra, drop_gnss, base=DEFAULT_KEEP):
    if topic in base or topic in extra:
        return True
    return topic.startswith(GNSS_PREFIX) and not drop_gnss


def window_ns(reader, start, duration):
    t0 = reader.start_time
    lo = t0 + int(round(start * 1e9)) if start > 0 else None
    hi = (t0 + int(round((start + duration) * 1e9))) if duration >= 0 else None
    return lo, hi


def _msgdef_text(msgdef):
    # rosbags >= 0.10 wraps the text, while Writer.add_connection still takes a string.
    return getattr(msgdef, "data", msgdef)


def _conn_key(c):
    ext = c.ext
    return (c.topic, c.msgtype, c.digest, _msgdef_text(c.msgdef), getattr(ext, "callerid", None), getattr(ext, "latching", None))


def filter_bag(bag_in, bag_out, extra=(), drop_gnss=False, start=0.0, duration=-1.0, lidar="avia"):
    from rosbags.rosbag1 import Reader, Writer
    kept = {}
    stats = {"messages": 0, "bytes": 0, "per_topic": {}}
    with Reader(bag_in) as reader:
        base = LIDAR_KEEP[lidar]
        conns = [c for c in reader.connections if keep_topic(c.topic, extra, drop_gnss, base)]
        lo, hi = window_ns(reader, start, duration)
        with Writer(bag_out) as writer:
            for c in conns:
                ext = c.ext
                kept[c.id] = writer.add_connection(c.topic, c.msgtype, msgdef=_msgdef_text(c.msgdef), md5sum=c.digest,
                                                   callerid=getattr(ext, "callerid", None), latching=getattr(ext, "latching", None))
            for c, t_ns, raw in reader.messages(connections=conns, start=lo, stop=hi):
                writer.write(kept[c.id], t_ns, raw)
                stats["messages"] += 1
                stats["bytes"] += len(raw)
                stats["per_topic"][c.topic] = stats["per_topic"].get(c.topic, 0) + 1
        stats["window_ns"] = (lo, hi)
        stats["dropped_topics"] = sorted({c.topic for c in reader.connections} - {c.topic for c in conns})
    return stats


def stream_digest(bag, topics, lo=None, hi=None):
    """sha256 over (topic, record time ns, sha256(bytes)) in record order + per-connection header fields."""
    from rosbags.rosbag1 import Reader
    h = hashlib.sha256()
    n = 0
    with Reader(bag) as reader:
        conns = [c for c in reader.connections if c.topic in topics]
        headers = sorted(_conn_key(c) for c in conns)
        for c, t_ns, raw in reader.messages(connections=conns, start=lo, stop=hi):
            h.update(c.topic.encode())
            h.update(t_ns.to_bytes(8, "little"))
            h.update(hashlib.sha256(raw).digest())
            n += 1
    return h.hexdigest(), n, headers


def verify(bag_in, bag_out, stats):
    from rosbags.rosbag1 import Reader
    with Reader(bag_out) as r:
        topics = {c.topic for c in r.connections}
    lo, hi = stats["window_ns"]
    din, nin, hin = stream_digest(bag_in, topics, lo, hi)
    dout, nout, hout = stream_digest(bag_out, topics)
    problems = []
    if nin != nout:
        problems.append("message count %d in window vs %d written" % (nin, nout))
    if din != dout:
        problems.append("message stream digest differs")
    if hin != hout:
        problems.append("connection header fields differ")
    return problems, {"stream_sha256": dout, "messages": nout}


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    if os.path.exists(args.bag_out) and not args.force:
        print("bag_filter: %s exists (use --force)" % args.bag_out, file=sys.stderr)
        return 2
    if os.path.exists(args.bag_out):
        os.remove(args.bag_out)
    stats = filter_bag(args.bag_in, args.bag_out, set(args.keep), args.drop_gnss, args.start, args.duration, args.lidar)
    from fl2g.rosio import sha256_file
    sha = sha256_file(args.bag_out)
    with open(args.bag_out + ".sha256", "w") as f:
        f.write("%s  %s\n" % (sha, os.path.basename(args.bag_out)))
    print("bag_filter: wrote %s: %d messages, %d payload bytes; sha256 %s" % (args.bag_out, stats["messages"], stats["bytes"], sha))
    for t, n in sorted(stats["per_topic"].items()):
        print("  keep %-45s %8d" % (t, n))
    print("  dropped topics: %s" % (", ".join(stats["dropped_topics"]) or "none"))
    if not args.no_verify:
        problems, info = verify(args.bag_in, args.bag_out, stats)
        if problems:
            print("bag_filter: VERIFY FAILED: %s" % "; ".join(problems), file=sys.stderr)
            return 1
        print("bag_filter: verified: %d messages byte-identical with identical record times; connection headers preserved (stream sha256 %s)" % (
            info["messages"], info["stream_sha256"]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
