# SPDX-License-Identifier: BSD-3-Clause
"""Time-ordered slim-bag writer: several already time-ordered message streams are merged by record time and written
through rosbags (Apache-2.0). Plus an order-preserving thread-pool map (JPEG work releases the GIL) and the lazy raw-bag
reader the streams are made of (a bag file is opened when its first message is needed and closed after its last).
"""
import heapq
import os
from collections import deque
from concurrent.futures import ThreadPoolExecutor

# rank of a message kind at equal record time: IMU first so that a frame never precedes the samples it needs
RANK = {"imu": 0, "gnss": 1, "camera": 2, "lidar": 3}


class Conn(object):
    """Connection description for Writer.add_connection (the source bag's own header fields, copied verbatim)."""

    def __init__(self, topic, msgtype, md5, msgdef, callerid=None, latching=None):
        self.topic, self.msgtype, self.md5, self.msgdef = topic, msgtype, md5, msgdef
        self.callerid, self.latching = callerid, latching


def msgdef_text(msgdef):
    """The definition text of a rosbags Connection (a str in rosbags < 0.10, a MessageDefinition(format, data) in later ones)."""
    return getattr(msgdef, "data", msgdef)


def conn_from_source(reader_conn, topic=None):
    ext = reader_conn.ext
    return Conn(topic or reader_conn.topic, reader_conn.msgtype, reader_conn.digest, msgdef_text(reader_conn.msgdef),
                getattr(ext, "callerid", None), getattr(ext, "latching", None))


class Monotonic(object):
    """Wraps a stream of (t_ns, topic, payload): non-decreasing record times are enforced (a violation is counted and
    the time is clamped), so heapq.merge's precondition holds by construction."""

    def __init__(self, stream, name):
        self.stream, self.name = stream, name
        self.last = None
        self.clamped = 0

    def __iter__(self):
        for t, topic, payload in self.stream:
            if self.last is not None and t < self.last:
                self.clamped += 1
                t = self.last
            self.last = t
            yield t, topic, payload


def merge(streams):
    """streams: [(rank int, iterable of (t_ns, topic, payload))]; yields (t_ns, topic, payload) in record order."""
    def keyed(rank, it, idx):
        for t, topic, payload in it:
            yield (t, rank, idx), topic, payload

    iters = [keyed(r, s, i) for i, (r, s) in enumerate(streams)]
    for key, topic, payload in heapq.merge(*iters, key=lambda e: e[0]):
        yield key[0], topic, payload


def write_bag(path, conns, merged, progress=None, chunk_threshold=None, log=None):
    """Write `merged` (iterable of (t_ns, topic, payload)) to a new bag; `conns`: {topic: Conn}. Returns per-topic stats
    {topic: dict(n, first_ns, last_ns, bytes)}."""
    from rosbags.rosbag1 import Writer
    if os.path.exists(path):
        os.remove(path)
    stats = {}
    with Writer(path) as w:
        if chunk_threshold:
            w.chunk_threshold = chunk_threshold
        wc = {}
        for topic, c in conns.items():
            wc[topic] = w.add_connection(topic, c.msgtype, msgdef=c.msgdef, md5sum=c.md5, callerid=c.callerid,
                                         latching=c.latching)
            stats[topic] = {"n": 0, "first_ns": None, "last_ns": None, "bytes": 0}
        n = 0
        for t, topic, payload in merged:
            w.write(wc[topic], int(t), payload)
            s = stats[topic]
            s["n"] += 1
            s["bytes"] += len(payload)
            if s["first_ns"] is None:
                s["first_ns"] = int(t)
            s["last_ns"] = int(t)
            n += 1
            if progress and n % 20000 == 0:
                progress(n, t)
    return stats


def pmap(func, iterable, workers=3, window=24):
    """Ordered map over `iterable` with a thread pool and at most `window` items in flight."""
    if workers <= 1:
        for x in iterable:
            yield func(x)
        return
    q = deque()
    with ThreadPoolExecutor(max_workers=workers) as ex:
        it = iter(iterable)
        done = False
        while True:
            while not done and len(q) < window:
                try:
                    x = next(it)
                except StopIteration:
                    done = True
                    break
                q.append(ex.submit(func, x))
            if not q:
                return
            yield q.popleft().result()


def reindex_in_place(path):
    """Rebuild the index of an unindexed bag (a recording that was cut off: the connection and chunk-info records at the end
    are missing) with the ROS1 python API, in place: the incomplete tail chunk is dropped, everything before it is kept. The
    caller works on a file it owns (a downloaded copy); returns the number of chunks seen."""
    import rosbag
    n = 0
    b = rosbag.Bag(path, "a", allow_unindexed=True)
    try:
        for _ in b.reindex():
            n += 1
    finally:
        b.close()
    return n


def open_reader(path, notes=None):
    """An opened rosbags Reader; an unindexed bag is reindexed first (and `notes`, a list, gets a line about it)."""
    from rosbags.rosbag1 import Reader, ReaderError
    r = Reader(path)
    try:
        r.open()
        return r
    except ReaderError as e:
        if "not indexed" not in str(e):
            raise
    size0 = os.path.getsize(path)
    chunks = reindex_in_place(path)
    if notes is not None:
        notes.append({"file": os.path.basename(path), "reindexed": True, "chunks": chunks, "size_before": size0,
                      "size_after": os.path.getsize(path)})
    r = Reader(path)
    r.open()
    return r


def iter_bag(path, topic, notes=None):
    """(t_ns, raw bytes, connection) of one topic of a bag in record order."""
    r = open_reader(path, notes)
    try:
        conns = [c for c in r.connections if c.topic == topic]
        for c, t, raw in r.messages(connections=conns):
            yield t, raw, c
    finally:
        r.close()


class LazyBags(object):
    """Messages of one topic across several bag files in the given order. `open_file(i)` must return a local path (it
    may download), `close_file(i)` is called after the file's last message (it may delete). Prefetch of the next file is
    the caller's business (`prefetch(i)` hook, called when file i is opened)."""

    def __init__(self, n, open_file, close_file, topic, prefetch=None):
        self.n, self.open_file, self.close_file, self.topic, self.prefetch = n, open_file, close_file, topic, prefetch
        self.conn = None
        self.notes = []                      # reindexed files (open_reader)

    def __iter__(self):
        for i in range(self.n):
            path = self.open_file(i)
            if self.prefetch and i + 1 < self.n:
                self.prefetch(i + 1)
            try:
                for t, raw, c in iter_bag(path, self.topic, self.notes):
                    if self.conn is None:
                        self.conn = c
                    yield t, raw
            finally:
                self.close_file(i)
