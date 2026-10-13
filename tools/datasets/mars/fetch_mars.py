#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Fetch MARS-LVIG HKisland_GNSS03 (hku-mars; third-party copy on hf-mirror, run on the remote host only).

  fetch_mars.py fetch [--root /data2/fl2g/mars]      bag (10.04 GB), sha256 against the hub's LFS oid, IGS BRDC of the bag's UTC days
  fetch_mars.py info                                  what the hub says (size, sha256, commit) without downloading

Facts of the source (probed 2026-10-02): https://hf-mirror.com/datasets/swd123456/HKisland_GNSS03_Dataset_from_MARS-LVIG/resolve/
main/HKisland_GNSS03.bag answers 302 with `x-linked-etag` = the file's sha256 (LFS oid 8ce44d1d...c317) and `x-linked-size`
10040751440; the Location is a signed CDN link valid for about an hour (the downloader re-resolves it on a 4xx), which
serves ranges. The original MARS-LVIG data are on the project's own site; this copy is the one the A80 plan names.
Writes <root>/HKisland_GNSS03.bag and <root>/HKisland_GNSS03.fetch.json (hashes, official sha256, BRDC list).
"""
import argparse
import http.client
import json
import os
import sys
import time
import urllib.parse

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from dscommon import brdc, httpfetch  # noqa: E402

DEFAULT_ROOT = "/data2/fl2g/mars"
HUB = "https://hf-mirror.com/datasets/swd123456/HKisland_GNSS03_Dataset_from_MARS-LVIG/resolve/main/HKisland_GNSS03.bag"
NAME = "HKisland_GNSS03"


def resolve(url=HUB, timeout=60):
    """-> dict(url, size, sha256, commit): the signed CDN URL and the hub's claims, from the first hop's headers."""
    u = urllib.parse.urlsplit(url)
    c = http.client.HTTPSConnection(u.hostname, u.port, timeout=timeout)
    c.request("HEAD", u.path)
    r = c.getresponse()
    r.read()
    if r.status not in (301, 302, 303, 307, 308):
        raise httpfetch.FetchError("%s -> %d (expected a redirect to the CDN)" % (url, r.status))
    loc = r.getheader("Location")
    out = {"url": loc, "size": int(r.getheader("x-linked-size", "-1")), "sha256": (r.getheader("x-linked-etag") or "").strip('"').lower(),
           "commit": r.getheader("x-repo-commit")}
    c.close()
    if not loc or len(out["sha256"]) != 64:
        raise httpfetch.FetchError("hub answer without Location or x-linked-etag")
    return out


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("cmd", choices=["fetch", "info"])
    p.add_argument("--root", default=DEFAULT_ROOT)
    p.add_argument("--connections", type=int, default=4)
    a = p.parse_args(argv)
    info = resolve()
    if a.cmd == "info":
        print(json.dumps({k: v for k, v in info.items() if k != "url"}, indent=1))
        return 0
    os.makedirs(a.root, exist_ok=True)
    dest = os.path.join(a.root, NAME + ".bag")
    holder = httpfetch.Url(info["url"], refresh=lambda: resolve()["url"])
    t0 = time.time()
    rec = httpfetch.download(holder, dest, expect_size=info["size"], connections=a.connections)
    if rec["sha256"] != info["sha256"]:
        raise SystemExit("sha256 %s of the received bytes != the hub's LFS oid %s" % (rec["sha256"], info["sha256"]))
    from rosbags.rosbag1 import Reader
    with Reader(dest) as r:
        t_start, t_end = r.start_time / 1e9, r.end_time / 1e9
    nav = brdc.ensure(os.path.join(a.root, "brdc"), t_start, t_end)
    record = {"sequence": NAME, "source": HUB, "size": rec["size"], "sha256": rec["sha256"], "sha256_official_lfs_oid": info["sha256"],
              "sha256_matches_official": True, "repo_commit": info["commit"], "bag_start_utc": t_start, "bag_end_utc": t_end,
              "fetch_seconds": round(time.time() - t0, 1), "brdc": nav}
    with open(os.path.join(a.root, NAME + ".fetch.json"), "w") as f:
        json.dump(record, f, indent=1)
        f.write("\n")
    print(json.dumps(record, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
