# SPDX-License-Identifier: BSD-3-Clause
"""Inertial Explorer `.ref` ground-truth files (SmartPNT-MSF 04_GroundTruth) and their conversion to the TUM files the
evaluation tools read (tools/eval/gtbase.py contract: gt_clean.tum, gt_audited.tum, gt_timebase.yaml, trust_mask.csv).

File facts (read from the files themselves, dev sequence Data01_20240707 and the 0718/0719 ones):
  * `Source: GNSS/INS Epochs (Smoothed TC Combined)`, 1 Hz, WGS84 geodetic position, ECEF, velocity, attitude
    (Heading clockwise from north, Pitch nose up, Roll right side down) of the carrier frame (x right, y forward,
    z up; the 100C IMU sits in it with `Body to IMU Rotations`), position covariance, quality Q and AmbStatus,
    forward/backward separation columns. Time = GPST (Week, GPSTime [s of week]).
  * Every file of a sequence is the same smoothed trajectory transferred to another point of the platform:
    `IMU to GNSS Antenna Lever Arms` is the 100C-IMU -> rover-antenna vector the processing used (x right, y forward,
    z up) and `IMU->Secondary Sensor Lever Arms` the vector (from the 100C IMU, same axes) of the point this file's position
    refers to (absent in 100C_GroundTruth.ref: the 100C IMU itself). tests / the data: `Mini_GroundTruth.ref` and
    `ROVE_<mini>_GroundTruth.ref` carry the mini's GNSS antenna (Ant00); `I300_GroundTruth.ref` the mini's IMU.
    The difference of two files' positions, rotated by the attitude into the carrier frame, is constant to 0.4 mm
    and equals the difference of their lever arms (checked in `check_lever`), which also fixes the sign conventions
    R_nb = Rz(-heading) Rx(pitch) Ry(roll) used here.
"""
import math
import re

import numpy as np

from fl2g import geodesy, gnsstime

COLUMNS_MIN = 57
# I300 body axes (x down, y right, z back) expressed in the carrier frame (x right, y forward, z up): columns are the
# I300 axes. Placement angles (0, 90, -90) of Data_Description Table 2-3; verified against the gyro in verify_mount().
R_BODY_FROM_I300 = np.array([[0.0, 1.0, 0.0],
                             [0.0, 0.0, -1.0],
                             [-1.0, 0.0, 0.0]])


class RefFile(object):
    def __init__(self, path):
        self.path = path
        self.header = {}
        self.names = []
        self.rows = 0
        self.skipped = 0
        self.col = {}


def _float(v):
    """A numeric cell; IE marks cells it could not compute with an error code such as `E228!` (-> NaN)."""
    try:
        return float(v)
    except ValueError:
        return float("nan")


_LEVER = re.compile(r"x=\s*(-?[\d.]+)\s*,\s*y=\s*(-?[\d.]+)\s*,\s*z=\s*(-?[\d.]+)")


def _lever_after(lines, i):
    for k in range(i + 1, min(i + 3, len(lines))):
        m = _LEVER.search(lines[k])
        if m:
            return tuple(float(x) for x in m.groups())
    return None


def read_ref(path):
    """Parse a .ref file: header facts and the data columns by name (numpy arrays; Q int, AmbStatus/Date/HMS lists)."""
    with open(path, "r", errors="replace") as f:
        lines = f.read().splitlines()
    r = RefFile(path)
    hdr = r.header
    names = None
    start = None
    for i, ln in enumerate(lines):
        s = ln.strip()
        if s.startswith("Week") and names is None:
            names = s.split()
            start = i + 2                         # one units line below the names
            break
        if s.startswith("IMU to GNSS Antenna Lever Arms"):
            hdr["imu_to_antenna"] = _lever_after(lines, i)
        elif s.startswith("IMU->Secondary Sensor Lever Arms"):
            hdr["secondary_sensor"] = _lever_after(lines, i)
        elif s.startswith("Body to IMU Rotations"):
            m = re.findall(r"=\s*(-?[\d.]+)", " ".join(lines[i + 1:i + 2]))
            hdr["body_to_imu_deg"] = tuple(float(x) for x in m[:3])
        elif s.startswith("Source:"):
            hdr["source"] = s[7:].strip()
        elif s.startswith("Program:"):
            hdr["program"] = s[8:].strip()
        elif s.startswith("Datum:"):
            hdr["datum"] = s[6:].strip()
        elif s.startswith("SolFile:"):
            hdr["solfile"] = s[8:].strip()
        elif s.startswith("ProcessInfo:"):
            hdr["process_info"] = s[12:].strip()
        elif s.startswith("Lat, Lon, El Hgt"):
            hdr["master"] = s
    if names is None:
        raise ValueError("%s: no table header line (Week ...)" % path)
    names = list(names)
    seen = {}
    for i, n in enumerate(names):
        if n in seen:
            names[i] = n + "_" + ("hms" if n == "GPSTime" else str(seen[n]))
        seen[n] = seen.get(n, 0) + 1
    r.names = names
    raw = []
    for ln in lines[start:]:
        p = ln.split()
        if len(p) != len(names):
            r.skipped += 1
            continue
        try:
            float(p[1])
            int(p[0])
        except ValueError:
            r.skipped += 1
            continue
        raw.append(p)
    r.rows = len(raw)
    for j, n in enumerate(names):
        vals = [p[j] for p in raw]
        if n in ("AmbStatus", "Date", "GPSTime_hms", "iFlag"):
            r.col[n] = vals
        elif n in ("Week", "Q", "NS", "GP", "GL", "BD", "GA", "QZ"):
            r.col[n] = np.array([int(float(v)) for v in vals])
        else:
            r.col[n] = np.array([_float(v) for v in vals])
    return r


def times_gpst(r):
    """(week int array, tow float array)."""
    return r.col["Week"], r.col["GPSTime"]


def gpst_seconds(r):
    return np.array([gnsstime.gpst(w, t) for w, t in zip(*times_gpst(r))])


def utc_seconds(r):
    g = gpst_seconds(r)
    return np.array([x - gnsstime.leap_at_gpst(x) for x in g])


def attitude_matrices(heading_deg, pitch_deg, roll_deg):
    """R_nb (n,3,3): carrier frame (x right, y forward, z up) -> ENU, R = Rz(-heading) Rx(pitch) Ry(roll)."""
    h = np.radians(np.asarray(heading_deg, dtype=float))
    p = np.radians(np.asarray(pitch_deg, dtype=float))
    r = np.radians(np.asarray(roll_deg, dtype=float))
    n = h.size
    cz, sz = np.cos(-h), np.sin(-h)
    cx, sx = np.cos(p), np.sin(p)
    cy, sy = np.cos(r), np.sin(r)
    Rz = np.zeros((n, 3, 3))
    Rz[:, 0, 0], Rz[:, 0, 1], Rz[:, 1, 0], Rz[:, 1, 1], Rz[:, 2, 2] = cz, -sz, sz, cz, 1.0
    Rx = np.zeros((n, 3, 3))
    Rx[:, 0, 0], Rx[:, 1, 1], Rx[:, 1, 2], Rx[:, 2, 1], Rx[:, 2, 2] = 1.0, cx, -sx, sx, cx
    Ry = np.zeros((n, 3, 3))
    Ry[:, 0, 0], Ry[:, 0, 2], Ry[:, 1, 1], Ry[:, 2, 0], Ry[:, 2, 2] = cy, sy, 1.0, -sy, cy
    return np.einsum("nij,njk,nkl->nil", Rz, Rx, Ry)


def quat_from_matrices(R):
    """(n,3,3) -> (n,4) [qx qy qz qw], w >= 0 (Shepperd's method)."""
    n = R.shape[0]
    q = np.zeros((n, 4))
    tr = R[:, 0, 0] + R[:, 1, 1] + R[:, 2, 2]
    for i in range(n):
        m = R[i]
        t = tr[i]
        if t > 0.0:
            s = math.sqrt(t + 1.0) * 2.0
            w, x, y, z = 0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s
        elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
            s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
            w, x, y, z = (m[2, 1] - m[1, 2]) / s, 0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s
        elif m[1, 1] > m[2, 2]:
            s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
            w, x, y, z = (m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s
        else:
            s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
            w, x, y, z = (m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s
        if w < 0.0:
            w, x, y, z = -w, -x, -y, -z
        q[i] = (x, y, z, w)
    return q


def ecef_positions(r):
    lat, lon, h = np.radians(r.col["Latitude"]), np.radians(r.col["Longitude"]), r.col["H-Ell"]
    return np.array([geodesy.pos2ecef(a, b, c) for a, b, c in zip(lat, lon, h)])


def enu_positions(r, origin=None):
    """ENU (n,3) relative to `origin` = (lat, lon, h) in rad/rad/m (default: the first epoch) and the origin used."""
    lat, lon, h = np.radians(r.col["Latitude"]), np.radians(r.col["Longitude"]), r.col["H-Ell"]
    if origin is None:
        origin = (lat[0], lon[0], h[0])
    return geodesy.geodetic_to_enu(lat, lon, h, origin[0], origin[1], origin[2]), origin


def sigma_3d(r):
    """Frame-invariant 1-sigma position uncertainty sqrt(Cx11 + Cx22 + Cx33) [m] (the covariance's trace)."""
    return np.sqrt(np.maximum(r.col["Cx11"] + r.col["Cx22"] + r.col["Cx33"], 0.0))


def check_lever(a, b, lever_a_minus_b_rfu):
    """Position of file `a` minus file `b`, rotated into the carrier frame by a's attitude: (mean, std) of the 3-vector.
    Files must have the same epochs. The mean must equal the difference of the files' lever arms (header)."""
    if a.rows != b.rows or np.max(np.abs(a.col["GPSTime"] - b.col["GPSTime"])) > 1e-6:
        raise ValueError("files do not share their epochs")
    d = ecef_positions(a) - ecef_positions(b)
    lat, lon = np.radians(a.col["Latitude"]), np.radians(a.col["Longitude"])
    enu = np.array([geodesy.enu_matrix(la, lo) @ v for la, lo, v in zip(lat, lon, d)])
    R = attitude_matrices(a.col["Heading"], a.col["Pitch"], a.col["Roll"])
    v = np.einsum("nji,nj->ni", R, enu)                    # R^T enu
    return v.mean(axis=0), v.std(axis=0)


def point_lever_rfu(r):
    """Vector from the 100C IMU to the point this file's position refers to, carrier frame (x right, y forward, z up)."""
    sec = r.header.get("secondary_sensor")
    return np.zeros(3) if sec is None else np.array(sec)


def build_poses(r, lever_in_i300=None, origin=None, shift_s=0.0):
    """Poses of the reference point of `r` on the evaluation clock: rows (t_utc, x, y, z, qx, qy, qz, qw) with the I300
    orientation (carrier attitude times R_BODY_FROM_I300), ENU positions of the file's point. Returns (rows, origin)."""
    t = utc_seconds(r) + shift_s
    enu, origin = enu_positions(r, origin)
    Rn_b = attitude_matrices(r.col["Heading"], r.col["Pitch"], r.col["Roll"])
    Rn_i = np.einsum("nij,jk->nik", Rn_b, R_BODY_FROM_I300)
    q = quat_from_matrices(Rn_i)
    rows = np.column_stack([t, enu, q])
    return rows, origin, Rn_i


def transfer_point(rows, Rn_i, lever_i300):
    """Rows of the same trajectory at another rigid point: p' = p + R_n_i * lever (lever in the I300 frame)."""
    out = rows.copy()
    out[:, 1:4] += np.einsum("nij,j->ni", Rn_i, np.asarray(lever_i300, dtype=float))
    return out


def write_tum(path, rows, fmt="%.6f"):
    with open(path, "w") as f:
        for r in rows:
            f.write(" ".join([fmt % r[0]] + ["%.6f" % v for v in r[1:4]] + ["%.9f" % v for v in r[4:8]]) + "\n")
