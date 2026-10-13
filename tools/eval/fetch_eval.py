#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Rolling fetch of one M3DGR sequence on the remote host (P3-D; entry point tools/eval/fetch_eval.sh).

    fetch_eval.sh plan   <seq>...          cloud listing, sizes and the disk plan; downloads nothing
    fetch_eval.sh fetch  <seq> [options]    the pipeline below, one sequence
    fetch_eval.sh drop   <seq>              delete the filtered bag of a sequence (its runs are done)
    fetch_eval.sh status                    filtered bags on disk, staging leftovers, free space
    --lidar mid360 (before the subcommand; A76) selects the MID-360 host profile of tools/eval/lidars.json: the filtered bag keeps
    /livox/mid360/{lidar,imu} (tools/bag_filter.py --lidar mid360) and goes to data/m3dgr/filtered_mid360/ (record, logs and
    sha256 there too), unpacked through data/m3dgr/fetch_mid360/; the Avia directories are never touched. Default: avia.

Pipeline of `fetch` (every step is recorded in <bag dir>/<seq>.fetch.json, <bag dir> = data/m3dgr/filtered for Avia; logs in <seq>.fetch/):
  1  source  a pre-existing data/m3dgr/bags/<seq>.bag is used as is (never deleted unless --delete-local-original);
             otherwise the share is listed with aliyunpan and every file is downloaded by its own aliyunpan process
             (one process per file, ~13 MB/s each) into data/m3dgr/fetch/<seq>/;
  2  unpack  split sets <seq>.partNNN.exe: 7-Zip reads the byte concatenation of the pieces (the share's own
             merge_script.ps1 concatenates them in name order into one SFX <seq>.exe) through numbered volume links
             <seq>.exe.001..N, so the merged copy is never written (saves the archive size on a 94 %-full disk);
             --merge cat instead concatenates into <seq>.exe deleting each piece as soon as it is appended;
             `7z x`; the archive must hold exactly one .bag; pieces are deleted afterwards;
  3  verify  sha256 against the official checksums.txt (and the registry copy); `rosbag info` for the index;
             unindexed -> `rosbag reindex --output-dir`; a reindex that returns success but keeps under 98% of the
             registry duration_s (rosbag reindex stops at a corrupt chunk) is a failure, and with --allow-repair
             follows the same A3 route as a failed reindex: tools/repair_bag.py + reindex;
  4  inputs  GT file copied from the downloaded GT tree to data/m3dgr/gt/<seq>.txt (never overwritten: compared);
             IGS BRDC of every UTC day the bag spans (BKG mirror) into data/brdc/ (A1/A38: nav_rinex is required);
  5  audits  (on the unfiltered bag, before anything is deleted) tools/gt_audit.py (trajectory GT only; P4 gives the
             GT time base eval_ate.py applies) and, with --i2, tools/m3dgr_audit.py; outputs go to audit/<seq>/... when
             that archive does not exist yet, otherwise into <seq>.fetch/audit/ (an existing WP-g archive is never
             overwritten; the new gt_audited.tum is compared with the archived one);
  6  filter  tools/bag_filter.py keeps /livox/avia/{lidar,imu}, the compressed colour image and /ublox_driver/*
             plus /mavros/global_position/raw/fix and /odom (so gt_audit can be re-run on the filtered bag); the
             tool verifies the kept stream byte for byte;
  7  cleanup the unfiltered bag is deleted when this run created it.
Disk: before downloading, unpacking, reindexing and filtering the free space is checked against the need plus a
margin; not enough -> exit 75 (EX_TEMPFAIL: run_matrix.py retries after it has dropped a bag).

A2: evaluation-set sequences are fetched only with --evaluation-phase (this round rehearses on development sequences).
Runs on the remote host (Python >= 3.8, stdlib only); ROS steps and the Python tools run in the project image
(`docker run -u 1000:1000`, nice 19 / ionice idle inside the container). One fetch at a time per host
(data/m3dgr/.fetch.lock).
"""
import argparse
import datetime
import fcntl
import glob
import os
import re
import shlex
import shutil
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import evalcommon as ec  # noqa: E402

EX_TEMPFAIL = 75
EX_BLOCKED = 3
GIB = 1024 ** 3
MARGIN_BYTES = 3 * GIB              # never plan the disk closer than this
BAG_PER_README_GB = 1.08 * 1e9      # README sizes are decimal-ish GB of the bag; plan with 8 % slack
KEEP_EXTRA_TOPICS = ("/mavros/global_position/raw/fix", "/odom")   # gt_audit inputs (bag_filter drops them by default)
BKG_BRDC = "https://igs.bkg.bund.de/root_ftp/IGS/BRDC/{y}/{doy}/BRDC00IGS_R_{y}{doy}0000_01D_MN.rnx.gz"
PART_RE = re.compile(r"^(?P<stem>.+)\.part(?P<n>\d+)\.exe$")
LS_ROW = re.compile(r"^\s*(\d+)\s+(\S+)\s+(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2})\s+(.+?)\s*$")
UNIT = {"B": 1, "KB": 1024, "MB": 1024 ** 2, "GB": 1024 ** 3, "TB": 1024 ** 4}


# ---------------------------------------------------------------------------------------------------------------
# pure helpers (unit-tested)
def parse_size(text):
    """aliyunpan sizes '2.00GB' / '287.61MB' / '824B' / '-' (directory) -> bytes (approximate, binary units) or None."""
    m = re.fullmatch(r"([0-9.]+)\s*([KMGT]?B)", text.strip())
    if not m:
        return None
    return int(float(m.group(1)) * UNIT[m.group(2)])


def parse_ls(text):
    """`aliyunpan ls <dir>` output -> [{"name", "size", "is_dir"}] (header/footer lines ignored)."""
    out = []
    for ln in text.splitlines():
        m = LS_ROW.match(ln)
        if not m:
            continue
        name = m.group(4)
        is_dir = name.endswith("/")
        out.append({"name": name.rstrip("/"), "size": None if is_dir else parse_size(m.group(2)), "is_dir": is_dir})
    return out


def select_parts(entries, seq):
    """Pieces <seq>.partNNN.exe of a split set in number order; numbering must be 1..N without gaps."""
    parts = []
    for e in entries:
        m = PART_RE.match(e["name"])
        if m and not e["is_dir"] and m.group("stem") == seq:
            parts.append((int(m.group("n")), e))
    parts.sort(key=lambda x: x[0])
    nums = [n for n, _ in parts]
    if not parts:
        raise ValueError("no %s.partNNN.exe pieces in the listing" % seq)
    if nums != list(range(1, len(nums) + 1)):
        raise ValueError("piece numbers of %s are not 1..%d: %s" % (seq, len(nums), nums))
    return [e for _, e in parts]


def checksum_lookup(text, key):
    """Official M3DGR checksums.txt ('<HEX>  <Category\\sub\\Seq.bag>' per line, BOM tolerated) -> uppercase hex or None."""
    for ln in text.splitlines():
        s = ln.strip().lstrip("\ufeff")
        if not s:
            continue
        parts = s.split(None, 1)
        if len(parts) == 2 and parts[1].strip() == key:
            return parts[0].upper()
    return None


# sequences.json duration_s is the README table's integer seconds. A rosbag whose measured
# duration is under this fraction of that value is a truncated index, not a short sequence.
# 98% still accepts README rounding (Outdoor01 lists 411 s; the A3 repaired bag is 411.6 s)
# and rejects the 2026-09-30 reindex prefix (262.6 / 411 = 63.8%), where rosbag reindex
# returned success after keeping only the bytes before the corrupt chunk.
REINDEX_DURATION_FLOOR = 0.98


def bag_duration_s(info):
    """rosbag info duration field -> seconds (a trailing unit, if any, is ignored)."""
    return float(str(info.get("duration", "0")).split()[0] or 0)


def reindex_decision(measured_s, registry_s, sha_official, allow_repair, floor=REINDEX_DURATION_FLOOR):
    """What to do after rosbag reindex returns a readable bag.

    'accept' — duration covers the registry (or the registry has no duration).
    'repair' — too short, checksum is not official, and --allow-repair is set: run repair_bag.py.
    'fail'   — too short and repair is not appropriate (official checksum, or repair not requested).
    """
    if registry_s is None or float(registry_s) <= 0:
        return "accept"
    if float(measured_s) >= floor * float(registry_s):
        return "accept"
    if sha_official or not allow_repair:
        return "fail"
    return "repair"


def brdc_names(t_start, t_end):
    """UTC days touched by [t_start, t_end + 20 s] (GPST leads UTC by 18 s) -> [(year, doy, file name)]."""
    out = []
    d = datetime.datetime.fromtimestamp(t_start, datetime.timezone.utc).date()
    last = datetime.datetime.fromtimestamp(t_end + 20.0, datetime.timezone.utc).date()
    while d <= last:
        y, doy = d.year, d.timetuple().tm_yday
        out.append((y, "%03d" % doy, "BRDC00IGS_R_%d%03d0000_01D_MN.rnx" % (y, doy)))
        d += datetime.timedelta(days=1)
    return out


def parse_rosbag_info_yaml(text):
    """The few keys we need from `rosbag info --yaml` (flat 'key: value' lines at column 0)."""
    out = {}
    for ln in text.splitlines():
        m = re.match(r"^(path|version|duration|start|end|size|messages|indexed|compression):\s*(.*)$", ln)
        if m:
            out[m.group(1)] = m.group(2).strip()
    return out


def seven_zip_listing(text):
    """`7z l -slt` output -> [{"path", "size"}] of the files of the (innermost) archive."""
    files, cur = [], {}
    started = False
    for ln in text.splitlines():
        if ln.startswith("----------"):
            started = True
            continue
        if not started:
            continue
        if not ln.strip():
            if cur.get("Path") and cur.get("Folder", "-") != "+" and "Size" in cur:
                files.append({"path": cur["Path"], "size": int(cur["Size"])})
            cur = {}
            continue
        if " = " in ln:
            k, v = ln.split(" = ", 1)
            cur[k.strip()] = v.strip()
    if cur.get("Path") and cur.get("Folder", "-") != "+" and "Size" in cur:
        files.append({"path": cur["Path"], "size": int(cur["Size"])})
    return files


def volume_links(dest_dir, seq, piece_paths):
    """Numbered volume names 7-Zip joins transparently: <seq>.exe.001 ... -> the pieces (relative symlinks)."""
    links = []
    for i, p in enumerate(piece_paths, 1):
        link = os.path.join(dest_dir, "%s.exe.%03d" % (seq, i))
        if os.path.lexists(link):
            os.remove(link)
        os.symlink(os.path.relpath(p, dest_dir), link)
        links.append(link)
    return links


def cat_merge(piece_paths, out_path, delete_pieces=True, bufsize=64 << 20):
    """Concatenate the pieces in order into out_path; each piece is deleted right after it has been appended."""
    total = 0
    with open(out_path, "wb") as out:
        for p in piece_paths:
            with open(p, "rb") as f:
                shutil.copyfileobj(f, out, bufsize)
            total += os.path.getsize(p)
            if delete_pieces:
                os.remove(p)
    if os.path.getsize(out_path) != total:
        raise RuntimeError("merged size %d != sum of pieces %d" % (os.path.getsize(out_path), total))
    return total


# ---------------------------------------------------------------------------------------------------------------
class Fetch:
    def __init__(self, args):
        self.args = args
        self.root = ec.project_root(args.root)
        self.reg, self.seqs = ec.load_sequences()
        self.image = args.image
        self.repo_rel = os.path.relpath(ec.REPO, self.root)       # the synced tree, e.g. wt/p3-d/ws/src/FAST-LIVO2
        self.aliyunpan = args.aliyunpan or os.path.join(self.root, "tools", "aliyunpan")
        self.lidar = ec.lidar_profile(getattr(args, "lidar", None))              # A76 host LiDAR profile
        self.bag_dir = os.path.join(self.root, *self.lidar["bag_dir"].split("/"))
        self.staging_root = os.path.join(self.root, *self.lidar["staging_dir"].split("/"))
        os.environ.setdefault("ALIYUNPAN_CONFIG_DIR", os.path.join(self.root, "tools", "apcfg"))

    # -- paths
    def p(self, *rel):
        return os.path.join(self.root, *rel)

    def rel(self, path):
        return os.path.relpath(path, self.root)

    def in_container(self, path):
        return "/r/" + self.rel(path)

    # -- small utilities
    def free_bytes(self, path=None):
        st = os.statvfs(path or self.p("data"))
        return st.f_bavail * st.f_frsize

    def need_disk(self, need, what):
        free = self.free_bytes()
        self.log("disk: %s needs %.1f GiB (+%.0f GiB margin), free %.1f GiB" % (what, need / GIB, MARGIN_BYTES / GIB, free / GIB))
        if free < need + MARGIN_BYTES:
            raise TempFail("insufficient disk for %s: need %.1f GiB + %.0f GiB margin, free %.1f GiB" % (
                what, need / GIB, MARGIN_BYTES / GIB, free / GIB))

    def log(self, msg):
        line = "[fetch_eval %s] %s" % (time.strftime("%F %T"), msg)
        print(line, flush=True)
        if getattr(self, "logf", None):
            self.logf.write(line + "\n")
            self.logf.flush()

    def run(self, cmd, log_name=None, check=True, capture=False, timeout=None):
        """Run a host command (nice 19 / ionice idle); output to a log file in the record dir, or captured."""
        full = ["nice", "-n", "19", "ionice", "-c3"] + list(cmd)
        self.log("$ " + " ".join(shlex.quote(c) for c in cmd))
        if capture:
            r = subprocess.run(full, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True, timeout=timeout)
            if check and r.returncode != 0:
                raise StepError("%s failed (rc %d): %s" % (cmd[0], r.returncode, r.stdout[-2000:]))
            return r.returncode, r.stdout
        with open(os.path.join(self.rec_dir, log_name or "cmd.log"), "a") as lf:
            r = subprocess.run(full, stdout=lf, stderr=subprocess.STDOUT, timeout=timeout)
        if check and r.returncode != 0:
            raise StepError("%s failed (rc %d), see %s" % (cmd[0], r.returncode, log_name))
        return r.returncode, None

    def docker(self, script, log_name, check=True, capture=False):
        """bash `script` in the project image: /r = project root, cwd = the synced repository tree."""
        home = self.in_container(self.p("wt", self.args.wt_home, "home")) if self.args.wt_home else "/tmp"
        cmd = shlex.split(self.args.docker_cmd) + ["run", "--rm", "--init", "-u", self.args.docker_user,
               "-v", "%s:/r" % self.root,
               "-e", "HOME=%s" % home,
               "-e", "OMP_NUM_THREADS=1", "-e", "OPENBLAS_NUM_THREADS=1", "-e", "PYTHONDONTWRITEBYTECODE=1",
               "-w", "/r/" + self.repo_rel, self.image,
               "nice", "-n", "19", "ionice", "-c3", "bash", "-c", "source /opt/ros/noetic/setup.bash; " + script]
        self.log("$ docker: " + script)
        if capture:
            r = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True)
            if log_name:
                with open(os.path.join(self.rec_dir, log_name), "a") as lf:
                    lf.write(r.stdout)
            if check and r.returncode != 0:
                raise StepError("container step failed (rc %d): %s" % (r.returncode, r.stdout[-2000:]))
            return r.returncode, r.stdout
        with open(os.path.join(self.rec_dir, log_name), "a") as lf:
            r = subprocess.run(cmd, stdout=lf, stderr=subprocess.STDOUT)
        if check and r.returncode != 0:
            raise StepError("container step failed (rc %d), see %s" % (r.returncode, log_name))
        return r.returncode, None

    def step(self, name, **kw):
        kw["t_utc"] = ec.utc_now()
        self.record["steps"].append(dict(step=name, **kw))
        self.save()

    def save(self):
        ec.atomic_write_json(self.rec_json, self.record)

    # -- cloud
    def aliyunpan_ls(self, cloud_dir):
        r = subprocess.run([self.aliyunpan, "ls", cloud_dir], stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                           universal_newlines=True, timeout=180)
        if r.returncode != 0 or "当前目录" not in r.stdout:
            raise StepError("aliyunpan ls %s failed (rc %d): %s" % (cloud_dir, r.returncode, r.stdout[-800:]))
        return parse_ls(r.stdout)

    def cloud_files(self, info, seq):
        """-> [(cloud path, approx size)] of the pieces (or the single archive) of a sequence."""
        cloud = info.get("cloud")
        if not cloud:
            raise ec.Blocked("%s: %s" % (seq, info.get("cloud_note", "no cloud path in tools/eval/sequences.json")))
        if info["cloud_layout"] == "parts":
            ents = self.aliyunpan_ls(cloud)
            return [(cloud.rstrip("/") + "/" + e["name"], e["size"]) for e in select_parts(ents, seq)]
        parent, base = cloud.rsplit("/", 1)
        ents = [e for e in self.aliyunpan_ls(parent) if e["name"] == base and not e["is_dir"]]
        if len(ents) != 1:
            raise StepError("%s not found in the listing of %s" % (base, parent))
        return [(cloud, ents[0]["size"])]

    def aliyunpan_logged_in(self):
        r = subprocess.run([self.aliyunpan, "who"], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True, timeout=60)
        return r.returncode == 0 and ("uid" in r.stdout.lower() or "用户名" in r.stdout or "当前帐号" in r.stdout), r.stdout.strip()

    def download(self, files, staging):
        """One aliyunpan process per file, all in parallel; aliyunpan checks each file's hash itself."""
        procs = []
        for i, (cp, _) in enumerate(files, 1):
            lf = open(os.path.join(self.rec_dir, "download_%02d.log" % i), "w")
            cmd = ["nice", "-n", "19", self.aliyunpan, "download", "-p", "1", "--sp", "3", "--retry", "10",
                   "--saveto", staging, cp]
            self.log("$ " + " ".join(shlex.quote(c) for c in cmd[3:]))
            procs.append((cp, subprocess.Popen(cmd, stdout=lf, stderr=subprocess.STDOUT), lf, i))
        failed = []
        for cp, pr, lf, i in procs:
            rc = pr.wait()
            lf.close()
            with open(os.path.join(self.rec_dir, "download_%02d.log" % i), errors="replace") as f:
                text = f.read()
            local = os.path.join(staging, cp.lstrip("/"))
            size = os.path.getsize(local) if os.path.isfile(local) else -1
            want = dict(files).get(cp)
            # aliyunpan verifies each file's content hash itself ("检验文件有效性成功"); the listing size is rounded
            # to 2 decimals of its unit, so it is only a coarse cross-check (1 % + 1 MiB)
            size_ok = want is None or abs(size - want) <= 0.01 * want + (1 << 20)
            ok = rc == 0 and size >= 0 and "检验文件有效性成功" in text and size_ok
            if not ok:
                failed.append("%s (rc %d, size %d vs listed ~%s, hash check %s)" % (
                    cp, rc, size, want, "ok" if "检验文件有效性成功" in text else "not reported"))
        if failed:
            raise StepError("download failed: " + "; ".join(failed))
        return [os.path.join(staging, cp.lstrip("/")) for cp, _ in files]

    # -- main pipeline
    def fetch(self, seq):
        a = self.args
        info = self.seqs.get(seq)
        if info is None:
            raise ec.Blocked("%s is not in tools/eval/sequences.json" % seq)
        cls = ec.check_phase(seq, a.evaluation_phase, "fetching")
        if cls != info["class"]:
            raise StepError("registry class %s != tools/fl2g/sequences.py class %s" % (info["class"], cls))
        filt_dir = self.bag_dir
        os.makedirs(filt_dir, exist_ok=True)
        out_bag = os.path.join(filt_dir, seq + ".bag")
        self.rec_dir = os.path.join(filt_dir, seq + ".fetch")
        self.rec_json = os.path.join(filt_dir, seq + ".fetch.json")
        if os.path.exists(out_bag) and not a.force:
            self.log("%s already filtered: %s (use --force to redo)" % (seq, out_bag))
            return 0
        os.makedirs(self.rec_dir, exist_ok=True)
        self.logf = open(os.path.join(self.rec_dir, "fetch.log"), "a")
        staging = os.path.join(self.staging_root, seq)
        self.record = {"tool": "tools/eval/fetch_eval.py", "sequence": seq, "class": cls, "host": ec.host_name(),
                       "started_utc": ec.utc_now(), "argv": sys.argv, "source_id": source_id(), "steps": [],
                       "status": "running"}
        if self.lidar["name"] != ec.load_lidars()[0]:
            self.record["lidar"] = self.lidar["name"]
        self.save()
        created = []          # files this run created and must remove when done (unfiltered bag, pieces)
        try:
            # 1 source ------------------------------------------------------------------------------------------
            local = self.p("data", "m3dgr", "bags", seq + ".bag")
            if os.path.isfile(local) and not a.no_local:
                src = local
                self.step("source", kind="local", path=self.rel(src), size=os.path.getsize(src))
            else:
                if os.path.isdir(staging) and os.listdir(staging):
                    if not a.resume:
                        self.log("removing leftovers of an earlier attempt in %s" % staging)
                        shutil.rmtree(staging)
                os.makedirs(staging, exist_ok=True)
                ok, who = self.aliyunpan_logged_in()
                if not ok:
                    raise ec.Blocked("aliyunpan is not logged in (ALIYUNPAN_CONFIG_DIR=%s): %s" % (os.environ["ALIYUNPAN_CONFIG_DIR"], who[-300:]))
                files = self.cloud_files(info, seq)
                comp = sum(s or 0 for _, s in files)
                bag_est = int(info.get("size_gb", 0) * BAG_PER_README_GB)
                self.step("listing", files=[{"cloud": c, "size_approx": s} for c, s in files], compressed_approx=comp,
                          bag_estimate=bag_est)
                self.need_disk(comp + bag_est, "download + unpack of %s" % seq)
                t0 = time.time()
                pieces = self.download(files, staging)
                created += pieces
                self.step("download", files=[self.rel(x) for x in pieces], sizes=[os.path.getsize(x) for x in pieces],
                          seconds=round(time.time() - t0, 1))
                # 2 unpack ------------------------------------------------------------------------------------------
                xdir = os.path.join(staging, "x")
                os.makedirs(xdir, exist_ok=True)
                if len(pieces) > 1 and a.merge == "cat":
                    arch = os.path.join(staging, seq + ".exe")
                    total = cat_merge(pieces, arch)
                    created.append(arch)
                    self.step("merge", method="cat (pieces deleted as appended)", size=total)
                elif len(pieces) > 1:
                    links = volume_links(staging, seq, pieces)
                    created += links
                    arch = links[0]
                    self.step("merge", method="7-Zip volumes %s.exe.001..%03d (no merged copy)" % (seq, len(links)),
                              size=sum(os.path.getsize(x) for x in pieces))
                else:
                    arch = pieces[0]
                _, lst = self.run(["7z", "l", "-slt", arch], capture=True)
                bags = [f for f in seven_zip_listing(lst) if f["path"].endswith(".bag")]
                if len(bags) != 1:
                    raise StepError("archive must hold exactly one .bag, it lists %s" % [f["path"] for f in seven_zip_listing(lst)])
                self.need_disk(bags[0]["size"], "extracting %s" % bags[0]["path"])
                t0 = time.time()
                self.run(["7z", "x", "-y", "-o" + xdir, arch], log_name="7z.log")
                src = os.path.join(xdir, bags[0]["path"])
                if not os.path.isfile(src) or os.path.getsize(src) != bags[0]["size"]:
                    raise StepError("extracted bag missing or of the wrong size: %s" % src)
                created.append(src)
                for x in list(created):
                    if x != src and os.path.lexists(x):
                        os.remove(x)
                        created.remove(x)
                self.step("unpack", bag=self.rel(src), size=bags[0]["size"], seconds=round(time.time() - t0, 1))
            # 3 verify --------------------------------------------------------------------------------------------
            t0 = time.time()
            _, out = self.run(["sha256sum", src], capture=True)
            got = out.split()[0].upper()
            want = info.get("sha256", "").upper()
            ck = self.p(self.reg["checksums_file"])
            want_file = None
            if os.path.isfile(ck):
                with open(ck, encoding="utf-8-sig", errors="replace") as f:
                    want_file = checksum_lookup(f.read(), info["checksum_key"])
            if want_file and want and want_file != want:
                raise StepError("registry sha256 of %s disagrees with %s" % (seq, ck))
            official = got == (want_file or want)
            alt = {k: v for k, v in info.get("sha256_alt", {}).items() if v.upper() == got}
            self.step("sha256", sha256=got.lower(), official=official, official_alt=sorted(alt), seconds=round(time.time() - t0, 1),
                      note=None if official else ("known defect: " + info["known_defect"] if info.get("known_defect") else "MISMATCH"))
            if not official and not alt and src != self.p("data", "m3dgr", "bags", seq + ".bag") and not info.get("known_defect") and not a.accept_mismatch:
                raise StepError("sha256 %s of %s matches no official checksum (use --accept-mismatch to continue to the repair route)" % (got.lower(), seq))
            info_yaml = self.rosbag_info(src)
            if info_yaml.get("indexed", "True") != "True":
                original = src
                src = self.reindex(src, staging, created, sha_official=official, keep_input=True)
                info_yaml = self.rosbag_info(src)
                measured = bag_duration_s(info_yaml)
                decision = reindex_decision(measured, info.get("duration_s"), official, a.allow_repair)
                self.step("reindex_duration", measured_s=measured, registry_s=info.get("duration_s"),
                          floor=REINDEX_DURATION_FLOOR, decision=decision)
                if decision == "repair":
                    if src in created and src != original:
                        os.remove(src)
                        created.remove(src)
                    src = self.repair_and_reindex(original, staging, created)
                    info_yaml = self.rosbag_info(src)
                    measured = bag_duration_s(info_yaml)
                    if reindex_decision(measured, info.get("duration_s"), True, False) != "accept":
                        raise StepError("repaired %s is still short: %.3f s vs registry duration_s %s" % (
                            seq, measured, info.get("duration_s")))
                elif decision == "fail":
                    raise StepError("reindex of %s kept %.3f s, under %.0f%% of registry duration_s %s "
                                    "(corrupt tail is not recovered by rosbag reindex; --allow-repair runs repair_bag.py)" % (
                                        seq, measured, 100 * REINDEX_DURATION_FLOOR, info.get("duration_s")))
                elif original in created and original != src:
                    os.remove(original)
                    created.remove(original)
            t_start, t_end = float(info_yaml["start"]), float(info_yaml["end"])
            self.step("bag_info", start=t_start, end=t_end, duration=bag_duration_s(info_yaml),
                      messages=info_yaml.get("messages"), indexed=info_yaml.get("indexed"))
            # 4 inputs --------------------------------------------------------------------------------------------
            gt_dst = self.ensure_gt(seq, info)
            brdc = self.ensure_brdc(t_start, t_end)
            # 5 audits --------------------------------------------------------------------------------------------
            if not a.no_gt_audit and info["gt_kind"] == "rtk" and gt_dst:
                self.gt_audit(seq, src, gt_dst)
            if a.i2:
                self.i2_audit(seq, src, gt_dst, brdc)
            # 6 filter --------------------------------------------------------------------------------------------
            self.need_disk(os.path.getsize(src), "filtering %s" % seq)
            os.makedirs(staging, exist_ok=True)
            tmp_bag = os.path.join(staging, seq + ".bag")
            for x in (tmp_bag, tmp_bag + ".sha256"):
                if os.path.exists(x):
                    os.remove(x)
            keep = " ".join("--keep %s" % shlex.quote(t) for t in KEEP_EXTRA_TOPICS)
            t0 = time.time()
            if self.lidar["name"] != ec.load_lidars()[0]:      # the Avia command line stays as it was
                keep += " --lidar %s" % shlex.quote(self.lidar["name"])
            self.docker("python3 tools/bag_filter.py %s %s %s" % (shlex.quote(self.in_container(src)),
                                                                   shlex.quote(self.in_container(tmp_bag)), keep), "bag_filter.log")
            os.replace(tmp_bag, out_bag)
            os.replace(tmp_bag + ".sha256", out_bag + ".sha256")
            with open(out_bag + ".sha256") as f:
                fsha = f.read().split()[0]
            with open(os.path.join(self.rec_dir, "bag_filter.log"), errors="replace") as f:
                flog = f.read()
            verified = "bag_filter: verified" in flog
            self.step("filter", bag=self.rel(out_bag), size=os.path.getsize(out_bag), sha256=fsha, verified=verified,
                      kept_extra=list(KEEP_EXTRA_TOPICS), seconds=round(time.time() - t0, 1),
                      source_size=os.path.getsize(src))
            if not verified:
                raise StepError("bag_filter did not report a verified copy (see bag_filter.log)")
            # 7 cleanup -------------------------------------------------------------------------------------------
            if src in created or (a.delete_local_original and src == self.p("data", "m3dgr", "bags", seq + ".bag")):
                os.remove(src)
                if src in created:
                    created.remove(src)
                self.step("delete_unfiltered", path=self.rel(src))
            self.record.update(status="done", finished_utc=ec.utc_now(), filtered_bag=self.rel(out_bag),
                               filtered_sha256=fsha, brdc=brdc, gt=self.rel(gt_dst) if gt_dst else None)
            self.save()
            if os.path.isdir(staging) and not os.listdir(staging):
                os.rmdir(staging)
            elif os.path.isdir(staging) and not a.keep_staging:
                shutil.rmtree(staging)
            self.log("%s: DONE -> %s (%.2f GiB)" % (seq, out_bag, os.path.getsize(out_bag) / GIB))
            return 0
        except TempFail as e:
            self.record.update(status="tempfail", error=str(e), finished_utc=ec.utc_now())
            self.save()
            self.log("TEMPFAIL: %s" % e)
            return EX_TEMPFAIL
        except (ec.Blocked, ec.PhaseError) as e:
            self.record.update(status="blocked", error=str(e), finished_utc=ec.utc_now())
            self.save()
            self.log("BLOCKED: %s" % e)
            return EX_BLOCKED
        except Exception as e:     # noqa: BLE001 — every failure is recorded before it propagates
            self.record.update(status="failed", error="%s: %s" % (type(e).__name__, e), finished_utc=ec.utc_now())
            self.save()
            self.log("FAILED: %s: %s" % (type(e).__name__, e))
            return 1
        finally:
            if not a.keep_staging:
                for x in created:
                    if os.path.lexists(x):
                        os.remove(x)

    def rosbag_info(self, bag):
        rc, out = self.docker("rosbag info --yaml %s" % shlex.quote(self.in_container(bag)), "rosbag_info.log", check=False, capture=True)
        info = parse_rosbag_info_yaml(out)
        if "unindexed" in out.lower() or info.get("indexed") == "False":
            info["indexed"] = "False"
        elif rc != 0:
            raise StepError("rosbag info failed: %s" % out[-800:])
        return info

    def reindex(self, bag, staging, created, sha_official, keep_input=False):
        self.need_disk(os.path.getsize(bag), "reindexing")
        rdir = os.path.join(staging, "reindexed")
        os.makedirs(rdir, exist_ok=True)
        rc, _ = self.docker("rosbag reindex --output-dir=%s %s" % (shlex.quote(self.in_container(rdir)), shlex.quote(self.in_container(bag))),
                            "reindex.log", check=False)
        out = os.path.join(rdir, os.path.basename(bag))
        if rc == 0 and os.path.isfile(out):
            self.step("reindex", output=self.rel(out), size=os.path.getsize(out))
            if not keep_input and bag in created:
                os.remove(bag)
                created.remove(bag)
            created.append(out)
            return out
        if sha_official or not self.args.allow_repair:
            raise StepError("rosbag reindex failed (rc %d); %s" % (
                rc, "an official-checksum bag should reindex" if sha_official else "--allow-repair enables the A3 route"))
        return self.repair_and_reindex(bag, staging, created)

    def repair_and_reindex(self, bag, staging, created):
        """A3: drop corrupt chunks, then reindex the repaired bag (sha_official forced so a second failure stops)."""
        rep = os.path.join(staging, "repaired.bag")
        self.need_disk(os.path.getsize(bag), "repair_bag")
        self.run(["python3", os.path.join(ec.TOOLS, "repair_bag.py"), bag, rep], log_name="repair.log")
        created.append(rep)
        dropped = os.path.getsize(bag) - os.path.getsize(rep)
        self.step("repair", output=self.rel(rep), dropped_bytes=dropped, note="A3 route: corrupt chunks dropped; the gap must be footnoted")
        if bag in created:
            os.remove(bag)
            created.remove(bag)
        return self.reindex(rep, staging, created, sha_official=True)

    def ensure_gt(self, seq, info):
        src = self.p(self.reg["gt_raw_root"], info["gt"])
        dst = self.p("data", "m3dgr", "gt", seq + ".txt") if info["gt_kind"] == "rtk" else self.p("data", "m3dgr", "gt", os.path.basename(info["gt"]))
        if not os.path.isfile(src):
            if os.path.isfile(dst):
                self.step("gt", path=self.rel(dst), note="raw GT tree missing; existing copy kept")
                return dst
            self.step("gt", path=None, note="GT missing: %s" % self.rel(src))
            return None
        if os.path.isfile(dst):
            same = ec.sha256_file(src) == ec.sha256_file(dst)
            self.step("gt", path=self.rel(dst), kind=info["gt_kind"], identical_to_raw=same,
                      note=None if same else "existing GT differs from the raw download; kept (never overwritten)")
        else:
            os.makedirs(os.path.dirname(dst), exist_ok=True)
            shutil.copy2(src, dst)
            self.step("gt", path=self.rel(dst), kind=info["gt_kind"], copied_from=self.rel(src))
        return dst

    def ensure_brdc(self, t_start, t_end):
        got = []
        for y, doy, name in brdc_names(t_start, t_end):
            dst = self.p("data", "brdc", name)
            if not os.path.isfile(dst):
                if self.args.no_brdc_download:
                    raise ec.Blocked("BRDC %s missing and --no-brdc-download" % name)
                url = BKG_BRDC.format(y=y, doy=doy)
                gz = dst + ".gz.part"
                self.run(["curl", "-fsSL", "--retry", "3", "-o", gz, url], log_name="brdc.log", timeout=600)
                self.run(["gunzip", "-f", "-S", ".gz.part", gz], log_name="brdc.log")
                with open(dst, errors="replace") as f:
                    head = f.read(400)
                if "RINEX VERSION / TYPE" not in head:
                    os.remove(dst)
                    raise StepError("%s is not a RINEX file" % name)
                self.step("brdc_download", file=self.rel(dst), url=url)
            got.append(self.rel(dst))
        self.step("brdc", files=got)
        return got

    def gt_audit(self, seq, bag, gt):
        arch = self.p("audit", seq, "gt")
        out = arch if not os.path.exists(os.path.join(arch, "gt_audit.json")) else os.path.join(self.rec_dir, "audit", "gt")
        os.makedirs(out, exist_ok=True)
        self.docker("python3 tools/gt_audit.py %s %s %s --seq %s --archive-label %s --no-plots" % (
            shlex.quote(self.in_container(gt)), shlex.quote(self.in_container(bag)), shlex.quote(self.in_container(out)),
            shlex.quote(seq), shlex.quote(out)), "gt_audit.log")
        res = ec.load_json(os.path.join(out, "gt_audit.json"))
        o = res.get("outputs", {})
        cmp_archive = None
        if out != arch and os.path.isfile(os.path.join(arch, "gt_audited.tum")) and o.get("gt_audited"):
            cmp_archive = ec.sha256_file(os.path.join(arch, "gt_audited.tum")) == ec.sha256_file(os.path.join(out, "gt_audited.tum"))
        self.step("gt_audit", out=self.rel(out), verdict=res.get("verdict"), gt_audited=o.get("gt_audited"),
                  time_shift_applied_s=o.get("time_shift_applied_s"), withheld=o.get("gt_audited_withheld"),
                  archive=self.rel(arch), identical_to_archive=cmp_archive)

    def i2_audit(self, seq, bag, gt, brdc):
        arch = self.p("audit", seq)
        out = arch if not os.path.exists(os.path.join(arch, "audit.json")) else os.path.join(self.rec_dir, "audit")
        os.makedirs(out, exist_ok=True)
        nav = " ".join("--nav-rinex %s" % shlex.quote("/r/" + b) for b in brdc)
        gta = ("--gt %s" % shlex.quote(self.in_container(gt))) if gt and self.seqs[seq]["gt_kind"] == "rtk" else ""
        self.docker("python3 tools/m3dgr_audit.py %s %s %s %s --seq %s --archive-label %s --no-plots" % (
            shlex.quote(self.in_container(bag)), shlex.quote(self.in_container(out)), nav, gta, shlex.quote(seq), shlex.quote(out)),
            "i2_audit.log")
        res = ec.load_json(os.path.join(out, "audit.json"))
        counts = {}
        for c in res.get("checks", []):
            v = c.get("verdict") if isinstance(c, dict) else None
            if v:
                counts[v] = counts.get(v, 0) + 1
        arch_counts = None
        if out != arch:
            old = ec.load_json(os.path.join(arch, "audit.json"))
            arch_counts = {}
            for c in old.get("checks", []):
                v = c.get("verdict") if isinstance(c, dict) else None
                if v:
                    arch_counts[v] = arch_counts.get(v, 0) + 1
        # the I2 report is archived before anything is deleted (TEST_PLAN I2); its verdict is recorded, not a gate here
        self.step("i2_audit", out=self.rel(out), verdict=res.get("verdict"), counts=counts, archive=self.rel(arch),
                  archive_counts=arch_counts)


class TempFail(Exception):
    pass


class StepError(Exception):
    pass


def source_id():
    p = os.path.join(ec.HERE, ".source_id")
    if os.path.isfile(p):
        with open(p) as f:
            return f.read().strip()
    try:
        r = subprocess.run(["git", "-C", ec.REPO, "describe", "--always", "--dirty"], stdout=subprocess.PIPE,
                           stderr=subprocess.DEVNULL, universal_newlines=True, timeout=10)
        return r.stdout.strip() or "unknown"
    except Exception:  # noqa: BLE001
        return "unknown"


# ---------------------------------------------------------------------------------------------------------------
def cmd_plan(args):
    f = Fetch(args)
    rc = 0
    free = f.free_bytes()
    print("free on %s: %.1f GiB (margin %.0f GiB)" % (f.p("data"), free / GIB, MARGIN_BYTES / GIB))
    for seq in args.seqs:
        info = f.seqs.get(seq)
        if info is None:
            print("%s: not in the registry" % seq)
            rc = 1
            continue
        cls = ec.seq_class(seq)
        local = f.p("data", "m3dgr", "bags", seq + ".bag")
        filt = os.path.join(f.bag_dir, seq + ".bag")
        line = "%-15s %-11s " % (seq, cls)
        if os.path.isfile(filt):
            print(line + "filtered bag present (%.2f GiB)" % (os.path.getsize(filt) / GIB))
            continue
        if os.path.isfile(local):
            print(line + "local unfiltered copy %.2f GiB -> filter only (needs <= %.1f GiB)" % (os.path.getsize(local) / GIB, os.path.getsize(local) / GIB))
            continue
        if not info.get("cloud"):
            print(line + "BLOCKED: " + info.get("cloud_note", "no cloud path"))
            rc = max(rc, EX_BLOCKED)
            continue
        try:
            files = f.cloud_files(info, seq)
        except Exception as e:  # noqa: BLE001
            print(line + "listing failed: %s" % e)
            rc = 1
            continue
        comp = sum(s or 0 for _, s in files)
        bag = info.get("size_gb", 0) * BAG_PER_README_GB
        need = comp + bag + MARGIN_BYTES
        print(line + "%d file(s) %.2f GiB compressed, bag ~%.1f GiB: peak %.1f GiB incl. margin -> %s" % (
            len(files), comp / GIB, bag / GIB, need / GIB, "fits" if need <= free else "DOES NOT FIT now"))
    return rc


def _bag_dir(root, args):
    """The filtered-bag directory of the selected host LiDAR (A76; data/m3dgr/filtered for Avia)."""
    return os.path.join(root, *ec.lidar_profile(getattr(args, "lidar", None))["bag_dir"].split("/"))


def cmd_drop(args):
    root = ec.project_root(args.root)
    rc = 0
    for seq in args.seqs:
        b = os.path.join(_bag_dir(root, args), seq + ".bag")
        if not os.path.isfile(b):
            print("%s: no filtered bag" % seq)
            continue
        size = os.path.getsize(b)
        os.remove(b)
        if os.path.isfile(b + ".sha256"):
            os.remove(b + ".sha256")
        rj = os.path.join(_bag_dir(root, args), seq + ".fetch.json")
        if os.path.isfile(rj):
            r = ec.load_json(rj)
            r.setdefault("steps", []).append({"step": "drop", "t_utc": ec.utc_now(), "freed": size})
            r["status"] = "dropped"
            ec.atomic_write_json(rj, r)
        print("%s: dropped filtered bag (%.2f GiB freed)" % (seq, size / GIB))
    return rc


def cmd_status(args):
    root = ec.project_root(args.root)
    fd = _bag_dir(root, args)
    st = os.statvfs(os.path.join(root, "data"))
    print("free: %.1f GiB" % (st.f_bavail * st.f_frsize / GIB))
    for b in sorted(glob.glob(os.path.join(fd, "*.bag"))):
        print("filtered %-15s %.2f GiB" % (os.path.basename(b)[:-4], os.path.getsize(b) / GIB))
    for j in sorted(glob.glob(os.path.join(fd, "*.fetch.json"))):
        r = ec.load_json(j)
        print("record   %-15s %s %s" % (r.get("sequence"), r.get("status"), r.get("error", "") or ""))
    sd = os.path.join(root, *ec.lidar_profile(getattr(args, "lidar", None))["staging_dir"].split("/"))
    if os.path.isdir(sd):
        for d in sorted(os.listdir(sd)):
            print("staging  %s (leftover)" % d)
    return 0


def build_parser():
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0], formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--root", default=None, help="project root (default FL2G_ROOT or /work/fastlivo2_gnss)")
    p.add_argument("--image", default=ec.IMAGE_DEFAULT)
    p.add_argument("--aliyunpan", default="", help="aliyunpan binary (default <root>/tools/aliyunpan)")
    p.add_argument("--wt-home", default="", help="remote worktree whose home/ is the container HOME (default /tmp)")
    p.add_argument("--lidar", default=None, choices=sorted(ec.load_lidars()[1]),
                   help="host LiDAR profile of tools/eval/lidars.json (A76): which topics the filtered bag keeps and which "
                        "directory it goes to (default: %s)" % ec.load_lidars()[0])
    sub = p.add_subparsers(dest="cmd")
    sp = sub.add_parser("plan")
    sp.add_argument("seqs", nargs="+")
    sf = sub.add_parser("fetch")
    sf.add_argument("seqs", nargs=1)
    sf.add_argument("--docker-cmd", default="sudo -n docker", help=argparse.SUPPRESS)
    sf.add_argument("--docker-user", default="1000:1000", help=argparse.SUPPRESS)
    sf.add_argument("--evaluation-phase", action="store_true", help="A2: required for evaluation-set sequences")
    sf.add_argument("--no-local", action="store_true", help="download even when data/m3dgr/bags/<seq>.bag exists")
    sf.add_argument("--delete-local-original", action="store_true",
                    help="also delete a pre-existing data/m3dgr/bags/<seq>.bag after filtering (default: keep)")
    sf.add_argument("--merge", choices=("volumes", "cat"), default="volumes")
    sf.add_argument("--i2", action="store_true", help="run tools/m3dgr_audit.py on the unfiltered bag (I2)")
    sf.add_argument("--no-gt-audit", action="store_true")
    sf.add_argument("--allow-repair", action="store_true", help="A3 route for a corrupt bag (repair_bag.py + reindex)")
    sf.add_argument("--accept-mismatch", action="store_true", help="continue although the sha256 matches no official value")
    sf.add_argument("--no-brdc-download", action="store_true")
    sf.add_argument("--resume", action="store_true", help="keep staging leftovers of an earlier attempt")
    sf.add_argument("--keep-staging", action="store_true", help="debug: keep pieces/unfiltered bag")
    sf.add_argument("--force", action="store_true", help="redo although the filtered bag exists")
    sd = sub.add_parser("drop")
    sd.add_argument("seqs", nargs="+")
    sub.add_parser("status")
    return p


def main(argv=None):
    args = build_parser().parse_args(argv)
    if args.cmd == "plan":
        return cmd_plan(args)
    if args.cmd == "drop":
        return cmd_drop(args)
    if args.cmd == "status":
        return cmd_status(args)
    if args.cmd != "fetch":
        build_parser().print_help()
        return 2
    root = ec.project_root(args.root)
    seq = args.seqs[0]
    try:                      # A2 and registry refusals before anything touches the disk or the network
        _, seqs = ec.load_sequences()
        if seq not in seqs:
            raise ec.Blocked("%s is not in tools/eval/sequences.json" % seq)
        ec.check_phase(seq, args.evaluation_phase, "fetching")
    except (ec.Blocked, ec.PhaseError) as e:
        print("fetch_eval: BLOCKED: %s" % e, file=sys.stderr)
        return EX_BLOCKED
    lock_path = os.path.join(root, "data", "m3dgr", ".fetch.lock")
    with open(lock_path, "a") as lk:
        try:
            fcntl.flock(lk, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            print("fetch_eval: another fetch holds %s; waiting" % lock_path, flush=True)
            fcntl.flock(lk, fcntl.LOCK_EX)
        return Fetch(args).fetch(seq)


if __name__ == "__main__":
    sys.exit(main())
