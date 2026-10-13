# SPDX-License-Identifier: BSD-3-Clause
"""IGS broadcast ephemeris (BRDC00IGS, multi-GNSS merged, RINEX 3 NAV) from the BKG mirror, like tools/eval/fetch_eval.py but
with the repository's own downloader (remote host only)."""
import gzip
import os
import shutil

from . import httpfetch
from fl2g import gnsstime

BKG = "https://igs.bkg.bund.de/root_ftp/IGS/BRDC/{y}/{doy}/BRDC00IGS_R_{y}{doy}0000_01D_MN.rnx.gz"


def day_of_year(t_utc):
    import datetime
    d = datetime.datetime.fromtimestamp(t_utc, datetime.timezone.utc)
    return d.year, d.timetuple().tm_yday


def brdc_days(t_start_utc, t_end_utc):
    """[(year, 'DDD', file name)] of every UTC day of [t_start, t_end] (a bag crossing midnight needs two files)."""
    out, t = [], t_start_utc
    while True:
        y, doy = day_of_year(t)
        out.append((y, "%03d" % doy, "BRDC00IGS_R_%d%03d0000_01D_MN.rnx" % (y, doy)))
        nxt = (int(t // 86400) + 1) * 86400
        if nxt > t_end_utc:
            break
        t = nxt
    seen, res = set(), []
    for x in out:
        if x[2] not in seen:
            seen.add(x[2])
            res.append(x)
    return res


def ensure(dest_dir, t_start_utc, t_end_utc, connections=2):
    """Download and gunzip the BRDC file(s) of the span into dest_dir; returns [dict(path, sha256, sha256_gz, url, size)]."""
    os.makedirs(dest_dir, exist_ok=True)
    out = []
    for y, doy, name in brdc_days(t_start_utc, t_end_utc):
        dst = os.path.join(dest_dir, name)
        url = BKG.format(y=y, doy=doy)
        gz = dst + ".gz"
        rec = httpfetch.download(url, gz, connections=connections)
        if not os.path.isfile(dst):
            with gzip.open(gz, "rb") as fi, open(dst + ".part", "wb") as fo:
                shutil.copyfileobj(fi, fo)
            os.replace(dst + ".part", dst)
        out.append({"path": dst, "name": name, "url": url, "size_gz": rec["size"], "sha256_gz": rec["sha256"],
                    "sha256": httpfetch.sha256_file(dst), "size": os.path.getsize(dst)})
    return out
