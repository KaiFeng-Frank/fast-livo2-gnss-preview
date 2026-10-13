# SPDX-License-Identifier: BSD-3-Clause
"""Raw ROS1 message editing for the slim-bag writers: sensor_msgs/Imu, CompressedImage and PointCloud2 are parsed and
rebuilt from their serialised bytes (no ROS installation, no typestore), time stamps are integer nanoseconds.

Everything here follows the ROS1 wire format: little endian, strings and arrays with a uint32 length prefix,
`time` = (uint32 sec, uint32 nsec). The standard definitions of the three types are fixed by their md5 sums
(IMU 6a62c6daae103f4ff57a132d6f95cec2, CompressedImage 8f7a12909da2c9d3332d540a0977563f, PointCloud2
1158d486dd51d683ce2f1be655c3c181), which the writers copy from the source bags' connection headers.
"""
import struct

import numpy as np

NS = 1000000000


def split_ns(t_ns):
    sec, nsec = divmod(int(t_ns), NS)
    return sec, nsec


def parse_header(raw, off=0):
    """-> (seq, stamp_ns, frame_id bytes, offset after the header)."""
    seq, sec, nsec, fl = struct.unpack_from("<IIII", raw, off)
    fid = bytes(raw[off + 16:off + 16 + fl])
    return seq, sec * NS + nsec, fid, off + 16 + fl


def pack_header(seq, stamp_ns, frame_id):
    sec, nsec = split_ns(stamp_ns)
    fid = frame_id if isinstance(frame_id, bytes) else frame_id.encode()
    return struct.pack("<IIII", seq & 0xFFFFFFFF, sec, nsec, len(fid)) + fid


# --- sensor_msgs/Imu -------------------------------------------------------------------------------------------
IMU_BODY = struct.Struct("<4d9d3d9d3d9d")           # orientation, its covariance, angular velocity, cov, acceleration, cov


def parse_imu(raw):
    """-> dict(seq, stamp_ns, frame_id, gyro (3,), acc (3,)); the covariances and the orientation are not kept."""
    seq, stamp, fid, off = parse_header(raw)
    v = IMU_BODY.unpack_from(raw, off)
    if off + IMU_BODY.size != len(raw):
        raise ValueError("Imu message of %d bytes does not match the standard layout" % len(raw))
    return {"seq": seq, "stamp_ns": stamp, "frame_id": fid, "gyro": v[13:16], "acc": v[25:28], "orientation": v[0:4]}


def build_imu(seq, stamp_ns, frame_id, gyro, acc):
    """Imu with zero orientation (orientation_covariance[0] = -1: no orientation estimate, as sensor_msgs says) and
    unknown (all-zero) covariances."""
    ocov = [0.0] * 9
    ocov[0] = -1.0
    v = [0.0, 0.0, 0.0, 0.0] + ocov + list(gyro) + [0.0] * 9 + list(acc) + [0.0] * 9
    return pack_header(seq, stamp_ns, frame_id) + IMU_BODY.pack(*v)


# --- sensor_msgs/CompressedImage ---------------------------------------------------------------------------------
def parse_compressed(raw):
    seq, stamp, fid, off = parse_header(raw)
    (fl,) = struct.unpack_from("<I", raw, off)
    fmt = bytes(raw[off + 4:off + 4 + fl])
    off += 4 + fl
    (dl,) = struct.unpack_from("<I", raw, off)
    data = bytes(raw[off + 4:off + 4 + dl])
    if off + 4 + dl != len(raw):
        raise ValueError("CompressedImage with %d trailing bytes" % (len(raw) - off - 4 - dl))
    return {"seq": seq, "stamp_ns": stamp, "frame_id": fid, "format": fmt, "data": data}


def build_compressed(seq, stamp_ns, frame_id, fmt, data):
    f = fmt if isinstance(fmt, bytes) else fmt.encode()
    return pack_header(seq, stamp_ns, frame_id) + struct.pack("<I", len(f)) + f + struct.pack("<I", len(data)) + data


# --- sensor_msgs/PointCloud2 -----------------------------------------------------------------------------------
class Cloud(object):
    """Offsets of a PointCloud2 message inside its serialised bytes."""

    def __init__(self, raw):
        self.seq, self.stamp_ns, self.frame_id, off = parse_header(raw)
        self.height, self.width, nf = struct.unpack_from("<III", raw, off)
        off += 12
        self.fields = []
        for _ in range(nf):
            (nl,) = struct.unpack_from("<I", raw, off)
            name = bytes(raw[off + 4:off + 4 + nl]).decode()
            off += 4 + nl
            foff, dtype, count = struct.unpack_from("<IBI", raw, off)
            off += 9
            self.fields.append((name, foff, dtype, count))
        self.big_endian, self.point_step, self.row_step = struct.unpack_from("<BII", raw, off)
        off += 9
        (self.data_len,) = struct.unpack_from("<I", raw, off)
        self.data_off = off + 4
        (self.is_dense,) = struct.unpack_from("<B", raw, self.data_off + self.data_len)
        if self.data_off + self.data_len + 1 != len(raw):
            raise ValueError("PointCloud2: %d trailing bytes" % (len(raw) - self.data_off - self.data_len - 1))
        self.npoints = self.data_len // self.point_step if self.point_step else 0

    def field(self, name):
        for f in self.fields:
            if f[0] == name:
                return f
        raise KeyError(name)

    def column(self, raw, name, np_dtype="<f4"):
        """Copy of one float32/uint16 column as an array."""
        _, foff, _, _ = self.field(name)
        a = np.frombuffer(raw, dtype=np.uint8, count=self.data_len, offset=self.data_off).reshape(-1, self.point_step)
        sz = np.dtype(np_dtype).itemsize
        return np.ascontiguousarray(a[:, foff:foff + sz]).view(np_dtype)[:, 0]


def rewrite_cloud(raw, cloud, stamp_ns, time_values=None, time_field="time"):
    """Copy of `raw` with a new header stamp and, when `time_values` (float32 array of npoints) is given, a new
    per-point `time` column. Everything else is byte-identical."""
    buf = bytearray(raw)
    sec, nsec = split_ns(stamp_ns)
    struct.pack_into("<II", buf, 4, sec, nsec)
    if time_values is not None:
        _, foff, dtype, _ = cloud.field(time_field)
        if dtype != 7:
            raise ValueError("per-point time field is not FLOAT32 (datatype %d)" % dtype)
        a = np.frombuffer(buf, dtype=np.uint8, count=cloud.data_len, offset=cloud.data_off).reshape(-1, cloud.point_step)
        a[:, foff:foff + 4] = np.ascontiguousarray(np.asarray(time_values, dtype="<f4")).view(np.uint8).reshape(-1, 4)
    return bytes(buf)


def detect_time_unit(max_abs):
    """Unit of a per-point time column from its largest magnitude: seconds (<= 0.5), milliseconds (<= 500),
    microseconds (<= 5e5), else nanoseconds. A scan lasts about 0.1 s, so the ranges do not overlap in practice."""
    if max_abs <= 0.5:
        return "s"
    if max_abs <= 500.0:
        return "ms"
    if max_abs <= 5.0e5:
        return "us"
    return "ns"


UNIT_TO_S = {"s": 1.0, "ms": 1e-3, "us": 1e-6, "ns": 1e-9}
