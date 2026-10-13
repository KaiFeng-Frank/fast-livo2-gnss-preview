#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Fetch the modalities of one SmartPNT-MSF sequence that A80 needs (run on the remote host only).

  fetch_smartpnt.py plan  <seq> [--root R] [--check]    list the files per role with portal sizes (--check: HEAD every file
                                                         and compare Content-Length with the portal's rounded size)
  fetch_smartpnt.py fetch <seq> [--roles small,config,lidar,cam] [--only-bags N] [--root R]
                                                         download into <root>/<seq>/raw/, write <root>/<seq>/fetch.json

Roles: small = mini receiver RINEX 3, broadcast ephemeris (brdm), IMU rosbag (+ I300.imr), 04_GroundTruth/*.ref,
06_Description, the BASE RINEX; config = <root>/_config/ (03_config calibration files and 02_pytools, shared by all
sequences); lidar = 03_LiDAR/*.bag (VLP-16); cam = 02_CAM/cam00/*.bag. The big roles are normally NOT fetched by hand:
build_slim.py streams them (download, convert, delete behind). Every file is checked against the server's Content-Length
(and the nginx ETag size) and against the portal's rounded size; sha256 of the received bytes goes to the manifest.
"""
import argparse
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from dscommon import httpfetch  # noqa: E402
from smartpnt import portal  # noqa: E402

DEFAULT_ROOT = "/data2/fl2g/smartpnt"
PLATFORM = "00_MINI"


def load_layout(root, seq, refresh=False):
    tree = portal.fetch_tree(os.path.join(root, "_portal", "tree.json"), refresh)
    files = portal.index_files(tree)
    return files, portal.Layout(files, PLATFORM, seq)


def role_entries(files, lay, roles, only_bags=None):
    out = []
    if "small" in roles:
        out += lay.small_files()
    if "config" in roles:
        out += [files[r] for r in portal.CONFIG_FILES if r in files]
    if "lidar" in roles:
        out += lay.lidar[:only_bags]
    if "cam" in roles:
        out += lay.cam[:only_bags]
    return out


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("cmd", choices=["plan", "fetch"])
    p.add_argument("seq")
    p.add_argument("--root", default=DEFAULT_ROOT)
    p.add_argument("--roles", default="small,config")
    p.add_argument("--only-bags", type=int, default=None)
    p.add_argument("--check", action="store_true", help="plan: HEAD every file")
    p.add_argument("--connections", type=int, default=4)
    p.add_argument("--refresh-tree", action="store_true")
    a = p.parse_args(argv)
    files, lay = load_layout(a.root, a.seq, a.refresh_tree)
    roles = set(a.roles.split(","))
    ents = role_entries(files, lay, roles, a.only_bags)
    if a.cmd == "plan":
        total = 0.0
        for e in ents:
            line = "%-12s %s" % (e.size_text, e.rel)
            if a.check:
                h = httpfetch.head(e.url())
                ok = abs(h["size"] - e.approx[0]) <= e.approx[1] + 1
                line += "   server %d B etag-size %s %s" % (h["size"], httpfetch.etag_size(h["etag"]), "ok" if ok else "SIZE MISMATCH")
            print(line)
            total += e.approx[0]
        print("total %.1f MiB in %d files; mini receiver %s, primary GT %s, mini id %s" % (
            total / (1 << 20), len(ents), lay.obs_mini.rel, lay.gt_primary.rel, lay.mini_id))
        return 0
    seqdir = os.path.join(a.root, a.seq)
    store = portal.RawStore(seqdir, a.connections)
    cfg_store = portal.RawStore(os.path.join(a.root, "_config"), a.connections)
    for e in ents:
        s = cfg_store if e.rel.startswith(("03_config", "02_pytools")) else store
        path = s.ensure(e)
        m = s.manifest[e.rel]
        print("%-90s %12d B  sha256 %s  %.1fs" % (e.rel, m["size"], m["sha256"][:16], m["fetch_seconds"]))
    store.save_manifest(os.path.join(seqdir, "fetch.json"))
    cfg_store.save_manifest(os.path.join(a.root, "_config", "fetch.json"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
