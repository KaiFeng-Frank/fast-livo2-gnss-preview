# SPDX-License-Identifier: BSD-3-Clause
"""ROS1 bag access for the offline tools.

* `ros_md5`: the ROS1 message md5sum recomputed from a connection's full `message_definition` (genmsg
  compute_md5_text rules: constants first as "type name=value", builtin fields "type name", message fields
  "<md5 of the sub-type> name" with the array suffix dropped, Header -> std_msgs/Header, unqualified types
  resolved in the parent's package). Used by the I2 audit to prove each connection's msg_def matches its md5.
* raw header parsing (seq, stamp, frame_id) without deserialising the rest (LiDAR/image payloads);
* a raw iterator over a bag in record-time order that drops the bag's pages from the page cache as it goes
  (posix_fadvise DONTNEED) so a multi-GB audit does not evict the page cache of other jobs on the host.
"""
import hashlib
import os
import struct

BUILTINS = {"int8", "uint8", "int16", "uint16", "int32", "uint32", "int64", "uint64", "float32", "float64",
            "string", "bool", "char", "byte", "time", "duration"}
SEPARATOR = "=" * 80


def split_msgdef(full_text, main_type):
    """Full connection message_definition -> {type: text} (main type first, then every `MSG: pkg/Type` section)."""
    out = {}
    cur_type, cur = main_type, []
    for line in full_text.split("\n"):
        if line.strip() == SEPARATOR:
            out[cur_type] = "\n".join(cur)
            cur_type, cur = None, []
            continue
        if cur_type is None:
            if line.startswith("MSG: "):
                cur_type = line[5:].strip()
            continue
        cur.append(line)
    if cur_type is not None:
        out[cur_type] = "\n".join(cur)
    return out


def _strip_comment(line):
    return line.split("#")[0].strip()


def _bare(t):
    return t[:t.find("[")] if "[" in t else t


def _resolve(field_type, package):
    bare = _bare(field_type)
    if bare in BUILTINS:
        return field_type
    if bare == "Header":
        return "std_msgs/Header" + field_type[len(bare):]
    if "/" in bare:
        return field_type
    return package + "/" + field_type


def parse_spec(text, package):
    """-> (constants [(type, name, value_text)], fields [(resolved type, name)])."""
    consts, fields = [], []
    for orig in text.split("\n"):
        clean = _strip_comment(orig)
        if not clean:
            continue
        if "=" in clean:
            parts = clean.split()
            ctype = parts[0]
            if ctype == "string":
                idx = orig.find("=")
                name = orig[orig.find(" ") + 1:idx].strip()
                val = orig[idx + 1:]
            else:
                name, val = [x.strip() for x in " ".join(parts[1:]).split("=")]
            consts.append((ctype, name, val.strip()))
        else:
            parts = clean.split()
            if len(parts) != 2:
                raise ValueError("invalid field line: %r" % orig)
            fields.append((_resolve(parts[0], package), parts[1]))
    return consts, fields


def md5_text(type_name, defs, cache):
    package = type_name.split("/")[0]
    if type_name not in defs:
        raise KeyError("definition of %s missing from message_definition" % type_name)
    consts, fields = parse_spec(defs[type_name], package)
    buf = []
    for ctype, name, val in consts:
        buf.append("%s %s=%s" % (ctype, name, val))
    for ftype, name in fields:
        bare = _bare(ftype)
        if bare in BUILTINS:
            buf.append("%s %s" % (ftype, name))
        else:
            buf.append("%s %s" % (md5_of(bare, defs, cache), name))
    return "\n".join(buf).strip()


def md5_of(type_name, defs, cache=None):
    if cache is None:
        cache = {}
    if type_name not in cache:
        cache[type_name] = hashlib.md5(md5_text(type_name, defs, cache).encode("utf-8")).hexdigest()
    return cache[type_name]


def ros_md5(datatype, full_msgdef):
    """md5sum of `datatype` recomputed from the connection's full message_definition."""
    return md5_of(datatype, split_msgdef(full_msgdef, datatype))


def main_fields(datatype, full_msgdef):
    defs = split_msgdef(full_msgdef, datatype)
    return parse_spec(defs.get(datatype, ""), datatype.split("/")[0])[1]


def first_field_is_header(datatype, full_msgdef):
    f = main_fields(datatype, full_msgdef)
    return bool(f) and _bare(f[0][0]) == "std_msgs/Header"


def parse_header(raw):
    """std_msgs/Header at the start of a serialised message -> (seq, stamp [s], frame_id, bytes consumed)."""
    seq, secs, nsecs, n = struct.unpack_from("<IIII", raw, 0)
    frame = raw[16:16 + n].decode("utf-8", errors="replace")
    return seq, secs + nsecs * 1e-9, frame, 16 + n


def stamp_ns(secs, nsecs):
    return int(secs) * 1000000000 + int(nsecs)


class RawBag:
    """Thin wrapper over rosbag.Bag: connection inventory and a page-cache friendly raw iterator."""

    def __init__(self, path, drop_cache_every=256 << 20):
        import rosbag  # ROS1 Python API (in the project image)
        self.path = path
        self.bag = rosbag.Bag(path, "r")
        self.drop_every = drop_cache_every
        self._bytes = 0
        self._last_drop = 0

    def close(self):
        self._drop_cache()
        self.bag.close()

    def _drop_cache(self):
        try:
            os.posix_fadvise(self.bag._file.fileno(), 0, 0, os.POSIX_FADV_DONTNEED)
        except (AttributeError, OSError, ValueError):
            pass

    def connections(self):
        out = []
        counts = {}
        try:
            for c in self.bag._get_connections():
                counts[c.id] = 0
            for c in self.bag._get_connections():
                idx = self.bag._connection_indexes.get(c.id, [])
                counts[c.id] = len(idx)
        except AttributeError:
            pass
        for c in self.bag._get_connections():
            h = c.header or {}
            dec = lambda k: h.get(k, b"").decode("utf-8", errors="replace") if isinstance(h.get(k), bytes) else h.get(k)
            out.append({"id": c.id, "topic": c.topic, "datatype": c.datatype, "md5sum": c.md5sum, "msg_def": c.msg_def,
                        "callerid": dec("callerid"), "latching": dec("latching"), "count": counts.get(c.id)})
        return out

    def start_end(self):
        return self.bag.get_start_time(), self.bag.get_end_time()

    def iter_raw(self, topics=None, duration=None):
        """Yield (topic, datatype, raw bytes, t_record [s], pytype) in record-time order (optionally only the first
        `duration` seconds of the bag, for quick runs)."""
        end = None
        if duration is not None and duration > 0:
            import rospy
            end = rospy.Time.from_sec(self.bag.get_start_time() + duration)
        for topic, msg, t in self.bag.read_messages(topics=topics, raw=True, end_time=end):
            datatype, data, _md5, _pos, pytype = msg
            self._bytes += len(data)
            if self._bytes - self._last_drop > self.drop_every:
                self._drop_cache()
                self._last_drop = self._bytes
            yield topic, datatype, data, t.to_sec(), pytype

    @staticmethod
    def deserialize(pytype, data):
        m = pytype()
        m.deserialize(data)
        return m


def sha256_file(path, block=16 << 20):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        while True:
            b = f.read(block)
            if not b:
                break
            h.update(b)
    try:
        fd = os.open(path, os.O_RDONLY)
        os.posix_fadvise(fd, 0, 0, os.POSIX_FADV_DONTNEED)
        os.close(fd)
    except (AttributeError, OSError):
        pass
    return h.hexdigest()
