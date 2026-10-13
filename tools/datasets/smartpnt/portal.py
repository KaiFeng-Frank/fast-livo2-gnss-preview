# SPDX-License-Identifier: BSD-3-Clause
"""SmartPNT-MSF data portal (Wuhan University, http://smartpnt-datasets.com): directory tree, file selection per
sequence, download URLs, and the raw-file store with sha256 bookkeeping.

Portal facts (probed 2026-10-02): no login; GET <api>/web/file/list returns the whole tree (type 0 directory, 1 file; the
size is a human string rounded to 0.01 of its unit, e.g. "78.17 MB", NOT a byte count); the data are plain HTTP files on
http://oss.smartpnt-datasets.com:8009/<path under SmartPNT/SmartPNT/> (what POST <api>/web/file/getDownloadUrl returns),
nginx with Range support, Content-Length / ETag / Last-Modified, no checksums are published. So the authoritative size
is the server's Content-Length (checked against the portal's rounded value), and the sha256 recorded here is that of the
bytes we received; the file's identity is (path, size, ETag, sha256).
"""
import json
import os
import re
import sys
import threading
import time
import urllib.parse
import urllib.request

from dscommon import httpfetch

API = "http://smartpnt-datasets.com/smart-pnt-dataset-api"
OSS = "http://oss.smartpnt-datasets.com:8009"
ROOT_PREFIX = "/data/SmartPNT/SmartPNT/"
UNITS = {"B": 1, "KB": 1 << 10, "MB": 1 << 20, "GB": 1 << 30, "TB": 1 << 40}


class PortalError(RuntimeError):
    pass


def human_to_bytes(text):
    """'78.17 MB' -> (approximate bytes, half-unit rounding bound in bytes)."""
    m = re.match(r"^\s*([\d.]+)\s*([KMGT]?B)\s*$", text)
    if not m:
        raise PortalError("cannot read the portal size %r" % text)
    unit = UNITS[m.group(2)]
    return float(m.group(1)) * unit, 0.005 * unit + 0.5


def fetch_tree(cache_path, refresh=False, timeout=120):
    """The portal tree (dict as served), cached in `cache_path`."""
    if os.path.isfile(cache_path) and not refresh:
        with open(cache_path, encoding="utf-8") as f:
            return json.load(f)
    req = urllib.request.Request(API + "/web/file/list", headers={"User-Agent": "fl2g-dataset-tools"})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        tree = json.loads(r.read().decode("utf-8"))
    if tree.get("code") != 0:
        raise PortalError("portal list: %s" % tree.get("msg"))
    d = os.path.dirname(os.path.abspath(cache_path))
    os.makedirs(d, exist_ok=True)
    with open(cache_path, "w", encoding="utf-8") as f:
        json.dump(tree, f, ensure_ascii=False)
    return tree


class Entry(object):
    def __init__(self, node):
        self.id = node["id"]
        self.path = node["path"]
        if not self.path.startswith(ROOT_PREFIX):
            raise PortalError("unexpected portal path %r" % self.path)
        self.rel = self.path[len(ROOT_PREFIX):]
        self.is_file = node.get("type") == 1
        self.size_text = node.get("fileSize", "")
        self.approx = human_to_bytes(self.size_text) if self.size_text else (0.0, 0.0)

    def url(self):
        return OSS + "/" + urllib.parse.quote(self.rel)

    def __repr__(self):
        return "Entry(%s, %s)" % (self.rel, self.size_text)


def index_files(tree):
    """{relative path: Entry} of every file of the tree."""
    out = {}

    def walk(n):
        if n.get("type") == 1:
            e = Entry(n)
            out[e.rel] = e
        for c in n.get("children", []) or []:
            walk(c)

    for root in tree["data"]:
        walk(root)
    return out


def sequence_prefix(platform, seq):
    return "%s/%s/" % (platform, seq)


def _num_suffix(rel, pat):
    m = re.search(pat, rel)
    return int(m.group(1)) if m else -1


class Layout(object):
    """The files of one sequence by role. Raises PortalError when a required role is missing or ambiguous."""

    def __init__(self, files, platform, seq):
        self.platform, self.seq = platform, seq
        pre = sequence_prefix(platform, seq)
        mine = {rel[len(pre):]: e for rel, e in files.items() if rel.startswith(pre)}
        if not mine:
            raise PortalError("sequence %s/%s is not in the portal tree" % (platform, seq))
        self.sub = mine
        self.obs_mini = self._one(r"^00_GNSS/01_ROVE/\d\d_mini[^/]*/ROVE_\d+\.24[oO]$", "mini receiver RINEX")
        self.nav = self._one(r"^00_GNSS/02_Product/brdm\d+\.24p$", "broadcast ephemeris (brdm)")
        self.imu_bag = self._one(r"^01_SINS/rosbag/imu_\d{4}_[\d_]+\.bag$", "IMU bag")
        self.imr = self._opt(r"^01_SINS/I300\.imr$")
        self.lidar = self._many(r"^03_LiDAR/[^/]+_(\d+)\.bag$", r"_(\d+)\.bag$")
        self.cam = self._many(r"^02_CAM/cam00/[^/]+_(\d+)_00\.bag$", r"_(\d+)_00\.bag$")
        self.gt = self._many(r"^04_GroundTruth/[^/]+\.ref$", None)
        self.desc = self._many(r"^06_Description/(Data_Description\.pdf|README\.xml)$", None)
        self.base = self._base()
        if not self.lidar or not self.cam:
            raise PortalError("%s: no LiDAR or cam00 bags" % seq)
        mini_id = re.search(r"ROVE_(\d+)", self.obs_mini.rel).group(1)
        self.mini_id = mini_id
        self.gt_primary = self._gt_primary()

    def _rels(self, pat):
        rx = re.compile(pat)
        return [r for r in self.sub if rx.search(r)]

    def _one(self, pat, what):
        r = self._rels(pat)
        if len(r) != 1:
            raise PortalError("%s: expected exactly one %s (%s), found %s" % (self.seq, what, pat, r))
        return self.sub[r[0]]

    def _opt(self, pat):
        r = self._rels(pat)
        return self.sub[r[0]] if len(r) == 1 else None

    def _many(self, pat, sort_pat):
        r = self._rels(pat)
        if sort_pat:
            r.sort(key=lambda x: _num_suffix(x, sort_pat))
        else:
            r.sort()
        return [self.sub[x] for x in r]

    def _base(self):
        for pat in (r"BASE_01\.24[oO]$", r"BASE_SGG\.24o$", r"/BASE\.24o$", r"WUH2"):
            r = self._rels(r"^00_GNSS/00_BASE/.*" + pat)
            if r:
                return self.sub[sorted(r)[0]]
        return None

    def _gt_primary(self):
        """GT file of the mini's GNSS antenna: Mini_GroundTruth.ref, else ROVE_<mini id>_GroundTruth.ref."""
        names = {e.rel.split("/")[-1]: e for e in self.gt}
        for n in ("Mini_GroundTruth.ref", "ROVE_%s_GroundTruth.ref" % self.mini_id):
            if n in names:
                return names[n]
        raise PortalError("%s: no GT file for the mini antenna among %s" % (self.seq, sorted(names)))

    def small_files(self):
        out = [self.obs_mini, self.nav, self.imu_bag] + self.gt + self.desc
        if self.imr:
            out.append(self.imr)
        if self.base:
            out.append(self.base)
        return out


CONFIG_FILES = ("03_config/mini_LiDAR.yaml", "03_config/mini_cam00.yaml",
                "03_config/cam_calibration/extrinsic_parameters/camchain-imucam-2024-06-26-02-19-12_0.yaml",
                "03_config/cam_calibration/extrinsic_parameters/imu-2024-06-26-02-19-12_0.yaml",
                "03_config/cam_calibration/extrinsic_parameters/results-imucam-2024-06-26-02-19-12_0.txt",
                "03_config/cam_calibration/intrinsic_parameters/cam00/cam00.mat",
                "03_config/cam_calibration/circumferential_calibration/camsRelation_undistort.yaml",
                "03_config/cam_calibration/circumferential_calibration/camsSpace_undistort.yaml",
                "02_pytools/cam_compress2raw.py", "02_pytools/cam_raw2compress.py")


class RawStore(object):
    """Downloads portal files into <root>/raw/<rel>, verifies size (Content-Length vs the portal's rounded value) and
    records (size, ETag, Last-Modified, sha256) in `manifest` (thread-safe). `release` deletes a file after use (the
    originals are fetched on demand and removed; the manifest keeps the record)."""

    def __init__(self, root, connections=4, log=None, attempts=4):
        self.root = os.path.join(root, "raw")
        self.connections = connections
        self.log = log or (lambda m: sys.stderr.write(m + "\n"))
        self.attempts = attempts
        self.manifest = {}
        self.lock = threading.Lock()

    def path(self, entry):
        return os.path.join(self.root, entry.rel)

    def ensure(self, entry):
        """Local path of `entry`, downloading it when needed."""
        dest = self.path(entry)
        approx, bound = entry.approx
        rec = None
        for attempt in range(1, self.attempts + 1):             # a transient server or network failure must not end a multi-hour build
            try:
                rec = httpfetch.download(entry.url(), dest, None, self.connections)
                break
            except (httpfetch.FetchError, OSError) as e:
                self.log("download of %s failed (attempt %d of %d): %s" % (entry.rel, attempt, self.attempts, e))
                if attempt == self.attempts:
                    raise
                time.sleep(30.0 * attempt)
        if abs(rec["size"] - approx) > bound + 1:
            raise PortalError("%s: server size %d differs from the portal's %s by more than its rounding" %
                              (entry.rel, rec["size"], entry.size_text))
        with self.lock:
            self.manifest[entry.rel] = {
                "url": entry.url(), "portal_id": entry.id, "portal_size": entry.size_text, "size": rec["size"],
                "etag": rec["etag"], "sha256": rec["sha256"], "fetch_seconds": round(rec["seconds"], 2),
                "fetched_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()), "kept": True}
        return dest

    def release(self, entry):
        dest = self.path(entry)
        if os.path.isfile(dest):
            os.remove(dest)
        with self.lock:
            if entry.rel in self.manifest:
                self.manifest[entry.rel]["kept"] = False

    def save_manifest(self, path):
        tmp = path + ".tmp"
        with self.lock:
            data = dict(self.manifest)
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=1, sort_keys=True, ensure_ascii=False)
            f.write("\n")
        os.replace(tmp, path)
