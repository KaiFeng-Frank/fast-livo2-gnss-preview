# SPDX-License-Identifier: BSD-3-Clause
"""Resumable multi-connection HTTP range downloader with size and sha256 bookkeeping (stdlib only, Python 3.8).

Runs on the remote host only (the repository's data rules: nothing is downloaded on a development machine). The server
of SmartPNT-MSF (nginx) answers HEAD with Content-Length / ETag (`"<hex mtime>-<hex size>"`) and ranges with 206; the
portal's own size column is a rounded human string, so the authoritative size is Content-Length, cross-checked against
the portal value to its rounding and, for nginx ETags, against the size encoded in the ETag.
"""
import hashlib
import http.client
import os
import re
import threading
import time
import urllib.parse

BLOCK = 1 << 20


class FetchError(RuntimeError):
    pass


class Url(object):
    """A download URL that can be re-resolved (signed CDN links expire): `refresh` is a callable returning a new URL."""

    def __init__(self, url, refresh=None):
        self._url, self._refresh = url, refresh
        self._lock = threading.Lock()

    def get(self):
        with self._lock:
            return self._url

    def refresh(self, stale):
        """Re-resolve unless another thread already replaced `stale`."""
        with self._lock:
            if self._refresh and self._url == stale:
                self._url = self._refresh()
            return self._url


def _conn(url, timeout):
    u = urllib.parse.urlsplit(url)
    cls = http.client.HTTPSConnection if u.scheme == "https" else http.client.HTTPConnection
    return cls(u.hostname, u.port, timeout=timeout), (u.path or "/") + (("?" + u.query) if u.query else "")


def head(url, timeout=60, retries=3, headers=None):
    """-> dict(size, etag, last_modified, accept_ranges). Follows no redirects (the SmartPNT OSS has none)."""
    last = None
    for k in range(retries):
        try:
            c, path = _conn(url, timeout)
            c.request("HEAD", path, headers=headers or {})
            r = c.getresponse()
            r.read()
            if r.status != 200:
                raise FetchError("HEAD %s -> %d" % (url, r.status))
            out = {"size": int(r.getheader("Content-Length", "-1")), "etag": (r.getheader("ETag") or "").strip('"'),
                   "last_modified": r.getheader("Last-Modified"), "accept_ranges": r.getheader("Accept-Ranges"),
                   "server": r.getheader("Server") or ""}
            c.close()
            return out
        except (OSError, http.client.HTTPException, FetchError) as e:
            last = e
            time.sleep(1.0 + k)
    raise FetchError("HEAD %s failed: %s" % (url, last))


def etag_size(etag, server="nginx"):
    """Size encoded in an nginx static-file ETag `<8 hex digits mtime>-<hex size>`, or None (any other server's ETag, or
    another format, says nothing about the size: Apache's is `<inode>-<size>-<mtime>`, hub CDNs use content hashes)."""
    if not str(server).lower().startswith("nginx"):
        return None
    m = re.match(r"^[0-9a-f]{8}-([0-9a-f]+)$", etag or "")
    return int(m.group(1), 16) if m else None


def sha256_file(path, block=BLOCK):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        while True:
            b = f.read(block)
            if not b:
                break
            h.update(b)
    return h.hexdigest()


def _segment(holder, path, a, b, timeout, retries, errors, progress):
    """Fetch bytes [a, b] into `path` at offset a (pwrite). Retries resume where the failed attempt stopped; a 4xx answer
    re-resolves the URL (expired signature)."""
    pos = a
    fd = os.open(path, os.O_WRONLY)
    try:
        attempt = 0
        while pos <= b:
            url = holder.get()
            try:
                c, p = _conn(url, timeout)
                c.request("GET", p, headers={"Range": "bytes=%d-%d" % (pos, b)})
                r = c.getresponse()
                if r.status in (400, 401, 403):
                    r.read()
                    holder.refresh(url)
                    raise FetchError("GET range %d-%d -> %d (URL refreshed)" % (pos, b, r.status))
                if r.status != 206:
                    raise FetchError("GET range %d-%d -> %d" % (pos, b, r.status))
                while pos <= b:
                    chunk = r.read(min(BLOCK, b - pos + 1))
                    if not chunk:
                        break
                    os.pwrite(fd, chunk, pos)
                    pos += len(chunk)
                    progress[0] += len(chunk)
                c.close()
                if pos <= b:
                    raise FetchError("short read at %d of %d" % (pos, b))
            except (OSError, http.client.HTTPException, FetchError) as e:
                attempt += 1
                if attempt > retries:
                    errors.append("%s [%d,%d]: %s" % (url.split("?")[0], a, b, e))
                    return
                time.sleep(min(30.0, 2.0 * attempt))
    finally:
        os.close(fd)


def download(url, dest, expect_size=None, connections=4, segment_bytes=64 << 20, timeout=60, retries=6, sha256=True):
    """Download `url` (a string or a Url) to `dest` (written as dest.part, renamed when complete and of the expected size).

    Returns dict(size, sha256, etag, seconds). An existing `dest` of the right size is kept (and hashed) instead of
    downloaded again. Raises FetchError on any mismatch."""
    holder = url if isinstance(url, Url) else Url(url)
    url = holder.get()
    t0 = time.time()
    info = head(url, timeout)
    size = info["size"]
    if size < 0:
        raise FetchError("%s: no Content-Length" % url)
    es = etag_size(info["etag"], info.get("server", ""))
    if es is not None and es != size:
        raise FetchError("%s: ETag size %d != Content-Length %d" % (url, es, size))
    if expect_size is not None and size != expect_size:
        raise FetchError("%s: Content-Length %d != expected %d" % (url, size, expect_size))
    os.makedirs(os.path.dirname(os.path.abspath(dest)), exist_ok=True)
    if os.path.isfile(dest) and os.path.getsize(dest) == size:
        return {"size": size, "sha256": sha256_file(dest) if sha256 else None, "etag": info["etag"],
                "seconds": time.time() - t0, "reused": True}
    part = dest + ".part"
    with open(part, "wb") as f:
        f.truncate(size)
    segs = []
    a = 0
    while a < size:
        b = min(size - 1, a + segment_bytes - 1)
        segs.append((a, b))
        a = b + 1
    errors, progress = [], [0]
    queue = list(segs)
    lock = threading.Lock()

    def worker():
        while True:
            with lock:
                if not queue or errors:
                    return
                a, b = queue.pop(0)
            _segment(holder, part, a, b, timeout, retries, errors, progress)

    ths = [threading.Thread(target=worker, daemon=True) for _ in range(max(1, min(connections, len(segs))))]
    for t in ths:
        t.start()
    for t in ths:
        t.join()
    if errors:
        raise FetchError("download of %s failed: %s" % (url, errors[0]))
    if os.path.getsize(part) != size or progress[0] != size:
        raise FetchError("%s: wrote %d of %d bytes" % (url, progress[0], size))
    os.replace(part, dest)
    return {"size": size, "sha256": sha256_file(dest) if sha256 else None, "etag": info["etag"],
            "seconds": time.time() - t0, "reused": False}
