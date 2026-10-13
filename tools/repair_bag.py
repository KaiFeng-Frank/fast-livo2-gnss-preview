#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Repair an unindexed ROS1 (v2.0) bag that contains a zeroed/corrupt region.

Records are parsed sequentially (header_len, header, data_len, data). A chunk record (op=0x05) is
accepted only if it is uncompressed, its 'size' equals data_len, and its inner records
(op=0x02 message data / op=0x07 connection) tile the chunk body exactly. At the first record that
fails, the script scans forward for the next acceptable chunk and resumes there; index-data records
(op=0x04) that trail a dropped chunk are dropped with it. The output is the concatenation of all
accepted records; run `rosbag reindex` on it afterwards (the stale bag-header index fields are
rebuilt by reindex).

usage: repair_bag.py <in.bag> <out.bag>
"""
import os
import struct
import sys

SCAN_WINDOW = 64 << 20


def parse_header(buf):
    fields, i = {}, 0
    while i < len(buf):
        if i + 4 > len(buf):
            return None
        (n,) = struct.unpack_from("<I", buf, i)
        i += 4
        if n == 0 or i + n > len(buf):
            return None
        field = buf[i:i + n]
        i += n
        if b"=" not in field:
            return None
        k, v = field.split(b"=", 1)
        fields[k] = v
    return fields


def chunk_body_ok(body):
    i, n = 0, len(body)
    while i < n:
        if i + 4 > n:
            return False
        (hl,) = struct.unpack_from("<I", body, i)
        i += 4
        if hl == 0 or i + hl + 4 > n:
            return False
        h = parse_header(body[i:i + hl])
        i += hl
        if not h or h.get(b"op") not in (b"\x02", b"\x07"):
            return False
        (dl,) = struct.unpack_from("<I", body, i)
        i += 4
        if i + dl > n:
            return False
        i += dl
    return i == n


def read_record(f, pos, size):
    """Return (op, end) for a well-formed record at pos, else None."""
    if pos + 8 > size:
        return None
    f.seek(pos)
    (hl,) = struct.unpack("<I", f.read(4))
    if hl == 0 or hl > (1 << 20) or pos + 4 + hl + 4 > size:
        return None
    h = parse_header(f.read(hl))
    if not h or len(h.get(b"op", b"")) != 1:
        return None
    (dl,) = struct.unpack("<I", f.read(4))
    end = pos + 4 + hl + 4 + dl
    if end > size:
        return None
    op = h[b"op"][0]
    if op == 5:
        if b"size" not in h or h.get(b"compression", b"none") != b"none":
            return None
        if struct.unpack("<I", h[b"size"])[0] != dl:
            return None
        if not chunk_body_ok(f.read(dl)):
            return None
    return op, end


def next_chunk(f, start, size):
    """Scan forward from start for the next acceptable chunk record."""
    pos = start
    while pos < size:
        f.seek(pos)
        window = f.read(min(SCAN_WINDOW, size - pos))
        j = 0
        while True:
            k = window.find(b"op=\x05", j)
            if k < 0:
                break
            # the op field sits somewhere inside the record header; try plausible record starts
            for back in range(8, 1024):
                cand = pos + k - back
                if cand < start:
                    break
                r = read_record(f, cand, size)
                if r is not None and r[0] == 5:
                    return cand
            j = k + 1
        pos += max(1, len(window) - 1024)
    return None


def main():
    src, dst = sys.argv[1], sys.argv[2]
    size = os.path.getsize(src)
    f = open(src, "rb")
    if f.read(13) != b"#ROSBAG V2.0\n":
        sys.exit("not a v2.0 bag")
    keep = [(0, 13)]
    pos = 13
    skipped = []
    while pos < size:
        r = read_record(f, pos, size)
        if r is not None:
            keep.append((pos, r[1]))
            pos = r[1]
            continue
        bad = pos
        nxt = next_chunk(f, pos + 1, size)
        if nxt is None:
            print("no acceptable chunk after", bad, "- dropping the tail")
            skipped.append((bad, size))
            break
        # the rejected chunk and its trailing index records lie in [bad, nxt) and are dropped
        skipped.append((bad, nxt))
        pos = nxt
    print("skipped regions:", skipped, "bytes:", sum(b - a for a, b in skipped))
    with open(dst, "wb") as o:
        for a, b in keep:
            f.seek(a)
            left = b - a
            while left:
                c = f.read(min(left, 64 << 20))
                o.write(c)
                left -= len(c)
    print("wrote", dst, os.path.getsize(dst))


if __name__ == "__main__":
    main()
