#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Inspect gnss_comm topics in an M3DGR bag: epochs, sats/epoch per system, obs types, ephemeris coverage."""
import sys, collections, rosbag
SYS = {1: "GPS", 2: "GLO", 4: "GAL", 8: "BDS", 16: "QZS", 32: "SBS"}
def sat_sys(sat):
    # gnss_comm sat numbering (same as RTKLIB): GPS 1-32, GLO 33-59, GAL 60-95, BDS 96-..; confirm by message fields
    return sat
b = rosbag.Bag(sys.argv[1])
first = {}
n_ep = 0; per_ep = []; sats_seen = collections.Counter(); ephem_sats = set(); glo_sats = set()
obs_stats = collections.Counter(); hdr_minus_gpst = []
for topic, msg, t in b.read_messages(topics=["/ublox_driver/range_meas", "/ublox_driver/ephem", "/ublox_driver/glo_ephem", "/ublox_driver/receiver_pvt", "/ublox_driver/iono_params", "/ublox_driver/time_pulse_info"]):
    if topic not in first:
        first[topic] = msg
    if topic.endswith("range_meas"):
        n_ep += 1; per_ep.append(len(msg.meas))
        for m in msg.meas:
            sats_seen[m.sat] += 1
            obs_stats["psr>0"] += any(p > 0 for p in m.psr)
            obs_stats["cp>0"] += any(c != 0 for c in m.cp)
            obs_stats["dopp!=0"] += any(d != 0 for d in m.dopp)
            obs_stats["n"] += 1
        if msg.meas:
            tt = msg.meas[0].time
            gpst = tt.week * 604800 + tt.tow
            hdr_minus_gpst.append(t.to_sec() - (gpst + 315964800 - 18))
    elif topic.endswith("/ephem"):
        ephem_sats.add(msg.sat)
    elif topic.endswith("glo_ephem"):
        glo_sats.add(msg.sat)
print("epochs", n_ep, "sats/epoch min/mean/max", min(per_ep), sum(per_ep) / len(per_ep), max(per_ep))
print("distinct sats observed", len(sats_seen), sorted(sats_seen))
print("ephem sats", sorted(ephem_sats), "glo ephem sats", sorted(glo_sats))
print("obs stats", dict(obs_stats))
import statistics
if hdr_minus_gpst:
    print("bag_time - (GPST->UTC) [s]: median %.4f  min %.4f max %.4f" % (statistics.median(hdr_minus_gpst), min(hdr_minus_gpst), max(hdr_minus_gpst)))
for k, v in first.items():
    s = str(v)
    print("=====", k); print(s[:2500])
