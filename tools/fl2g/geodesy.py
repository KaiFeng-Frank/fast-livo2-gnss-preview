# SPDX-License-Identifier: BSD-3-Clause
"""Geodesy for the offline tools: WGS-84 ECEF <-> geodetic, ENU, azimuth/elevation, geometric range with the
Sagnac term, Klobuchar and Saastamoinen models (same formulas and constants as RTKLIB rtkcmn.c so tool numbers
are comparable with the C++ core), Transverse Mercator (UTM) and the GCJ-02 datum shift (only used as
*hypotheses* by gt_audit.py to identify how a ground-truth file was produced).
"""
import math

import numpy as np

from . import gnsstime

RE_WGS84 = 6378137.0
FE_WGS84 = 1.0 / 298.257223563
E2_WGS84 = FE_WGS84 * (2.0 - FE_WGS84)
OMGE = 7.2921151467e-5
CLIGHT = 299792458.0
PI = math.pi


def ecef2pos(r):
    """ECEF [m] -> (lat [rad], lon [rad], h [m]) (RTKLIB ecef2pos iteration)."""
    x, y, z = float(r[0]), float(r[1]), float(r[2])
    r2 = x * x + y * y
    zk = 0.0
    v = RE_WGS84
    zz = z
    while abs(zz - zk) >= 1e-4:
        zk = zz
        sinp = zz / math.sqrt(r2 + zz * zz)
        v = RE_WGS84 / math.sqrt(1.0 - E2_WGS84 * sinp * sinp)
        zz = z + v * E2_WGS84 * sinp
    if r2 > 1e-12:
        lat = math.atan(zz / math.sqrt(r2))
        lon = math.atan2(y, x)
    else:
        lat = PI / 2.0 if z > 0.0 else -PI / 2.0
        lon = 0.0
    h = math.sqrt(r2 + zz * zz) - v
    return lat, lon, h


def pos2ecef(lat, lon, h):
    sinp, cosp = math.sin(lat), math.cos(lat)
    sinl, cosl = math.sin(lon), math.cos(lon)
    v = RE_WGS84 / math.sqrt(1.0 - E2_WGS84 * sinp * sinp)
    return np.array([(v + h) * cosp * cosl, (v + h) * cosp * sinl, (v * (1.0 - E2_WGS84) + h) * sinp])


def enu_matrix(lat, lon):
    """Rows = east, north, up unit vectors in ECEF (RTKLIB xyz2enu): enu = E @ d_ecef; R_{E<-N} = E.T."""
    sinp, cosp = math.sin(lat), math.cos(lat)
    sinl, cosl = math.sin(lon), math.cos(lon)
    return np.array([[-sinl, cosl, 0.0],
                     [-sinp * cosl, -sinp * sinl, cosp],
                     [cosp * cosl, cosp * sinl, sinp]])


def geodetic_to_enu(lat, lon, h, lat0, lon0, h0):
    """Arrays of geodetic coordinates [rad, rad, m] -> ENU [m] at the origin (lat0, lon0, h0) (exact, via ECEF)."""
    lat = np.atleast_1d(np.asarray(lat, dtype=float))
    lon = np.atleast_1d(np.asarray(lon, dtype=float))
    h = np.atleast_1d(np.asarray(h, dtype=float))
    sinp, cosp = np.sin(lat), np.cos(lat)
    v = RE_WGS84 / np.sqrt(1.0 - E2_WGS84 * sinp * sinp)
    xyz = np.stack([(v + h) * cosp * np.cos(lon), (v + h) * cosp * np.sin(lon), (v * (1.0 - E2_WGS84) + h) * sinp], axis=1)
    r0 = pos2ecef(lat0, lon0, h0)
    return (enu_matrix(lat0, lon0) @ (xyz - r0).T).T


def satazel(pos, e):
    """Azimuth/elevation [rad] of the unit LOS vector e (ECEF) at geodetic pos (RTKLIB satazel)."""
    lat, lon, h = pos
    if h <= -RE_WGS84:
        return 0.0, PI / 2.0
    enu = enu_matrix(lat, lon) @ e
    az = math.atan2(enu[0], enu[1]) if (enu[0] * enu[0] + enu[1] * enu[1]) > 1e-12 else 0.0
    if az < 0.0:
        az += 2.0 * PI
    el = math.asin(max(-1.0, min(1.0, enu[2])))
    return az, el


def geodist(rs, rr):
    """Geometric range with the Sagnac correction and the unit LOS receiver->satellite (RTKLIB geodist)."""
    d = rs - rr
    r = float(np.linalg.norm(d))
    if r <= 0.0:
        return -1.0, d
    e = d / r
    return r + OMGE * (rs[0] * rr[1] - rs[1] * rr[0]) / CLIGHT, e


def ionmodel(t_gpst, ion, pos, az, el):
    """Klobuchar L1 delay [m] (RTKLIB ionmodel; ion = 4 alpha + 4 beta; t in GPST seconds)."""
    default = (0.1118e-07, -0.7451e-08, -0.5961e-07, 0.1192e-06, 0.1167e+06, -0.2294e+06, -0.1311e+06, 0.1049e+07)
    if pos[2] < -1e3 or el <= 0.0:
        return 0.0
    if ion is None or sum(abs(v) for v in ion) <= 0.0:
        ion = default
    psi = 0.0137 / (el / PI + 0.11) - 0.022
    phi = pos[0] / PI + psi * math.cos(az)
    phi = max(-0.416, min(0.416, phi))
    lam = pos[1] / PI + psi * math.sin(az) / math.cos(phi * PI)
    phi += 0.064 * math.cos((lam - 1.617) * PI)
    _, tow = gnsstime.week_tow(t_gpst)
    tt = 43200.0 * lam + tow
    tt -= math.floor(tt / 86400.0) * 86400.0
    f = 1.0 + 16.0 * (0.53 - el / PI) ** 3
    amp = ion[0] + phi * (ion[1] + phi * (ion[2] + phi * ion[3]))
    per = ion[4] + phi * (ion[5] + phi * (ion[6] + phi * ion[7]))
    amp = max(0.0, amp)
    per = max(72000.0, per)
    x = 2.0 * PI * (tt - 50400.0) / per
    return CLIGHT * f * ((5e-9 + amp * (1.0 + x * x * (-0.5 + x * x / 24.0))) if abs(x) < 1.57 else 5e-9)


def tropmodel(pos, el, humi=0.7):
    """Saastamoinen with the standard atmosphere [m] (RTKLIB tropmodel)."""
    if pos[2] < -100.0 or pos[2] > 1e4 or el <= 0.0:
        return 0.0
    hgt = max(0.0, pos[2])
    pres = 1013.25 * (1.0 - 2.2557e-5 * hgt) ** 5.2568
    temp = 15.0 - 6.5e-3 * hgt + 273.16
    e = 6.108 * humi * math.exp((17.15 * temp - 4684.0) / (temp - 38.45))
    z = PI / 2.0 - el
    trph = 0.0022768 * pres / (1.0 - 0.00266 * math.cos(2.0 * pos[0]) - 0.00028 * hgt / 1e3) / math.cos(z)
    trpw = 0.002277 * (1255.0 / temp + 0.05) * e / math.cos(z)
    return trph + trpw


# ---------------------------------------------------------------------------------------------------------
# Transverse Mercator (Krueger n-series to n^3, Karney 2011), used for the "GT is UTM" hypothesis.
# ---------------------------------------------------------------------------------------------------------
def utm_zone(lon_deg):
    return int(math.floor((lon_deg + 180.0) / 6.0)) + 1


def tm_forward(lat_deg, lon_deg, lon0_deg, k0=0.9996, false_e=500000.0, false_n=0.0):
    lat = np.radians(np.asarray(lat_deg, dtype=float))
    dlon = np.radians(np.asarray(lon_deg, dtype=float) - lon0_deg)
    f = FE_WGS84
    n = f / (2.0 - f)
    A = RE_WGS84 / (1.0 + n) * (1.0 + n * n / 4.0 + n ** 4 / 64.0)
    alpha = (n / 2.0 - 2.0 * n * n / 3.0 + 5.0 * n ** 3 / 16.0,
             13.0 * n * n / 48.0 - 3.0 * n ** 3 / 5.0,
             61.0 * n ** 3 / 240.0)
    c = 2.0 * math.sqrt(n) / (1.0 + n)
    s = np.sin(lat)
    t = np.sinh(np.arctanh(s) - c * np.arctanh(c * s))
    xi = np.arctan2(t, np.cos(dlon))
    eta = np.arctanh(np.sin(dlon) / np.sqrt(1.0 + t * t))
    xs, es = xi.copy(), eta.copy()
    for j, a in enumerate(alpha, start=1):
        xs = xs + a * np.sin(2 * j * xi) * np.cosh(2 * j * eta)
        es = es + a * np.cos(2 * j * xi) * np.sinh(2 * j * eta)
    return false_e + k0 * A * es, false_n + k0 * A * xs


# ---------------------------------------------------------------------------------------------------------
# GCJ-02 (the Chinese map datum obfuscation). Only the forward shift is modelled; the inverse is a fixed-point
# iteration. Used by gt_audit.py to test whether a GT file was built from GCJ-02 coordinates.
# ---------------------------------------------------------------------------------------------------------
_GCJ_A = 6378245.0
_GCJ_EE = 0.00669342162296594323


def _gcj_dlat(x, y):
    r = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y + 0.1 * x * y + 0.2 * np.sqrt(np.abs(x))
    r += (20.0 * np.sin(6.0 * x * PI) + 20.0 * np.sin(2.0 * x * PI)) * 2.0 / 3.0
    r += (20.0 * np.sin(y * PI) + 40.0 * np.sin(y / 3.0 * PI)) * 2.0 / 3.0
    r += (160.0 * np.sin(y / 12.0 * PI) + 320.0 * np.sin(y * PI / 30.0)) * 2.0 / 3.0
    return r


def _gcj_dlon(x, y):
    r = 300.0 + x + 2.0 * y + 0.1 * x * x + 0.1 * x * y + 0.1 * np.sqrt(np.abs(x))
    r += (20.0 * np.sin(6.0 * x * PI) + 20.0 * np.sin(2.0 * x * PI)) * 2.0 / 3.0
    r += (20.0 * np.sin(x * PI) + 40.0 * np.sin(x / 3.0 * PI)) * 2.0 / 3.0
    r += (150.0 * np.sin(x / 12.0 * PI) + 300.0 * np.sin(x / 30.0 * PI)) * 2.0 / 3.0
    return r


def wgs84_to_gcj02(lat_deg, lon_deg):
    lat = np.asarray(lat_deg, dtype=float)
    lon = np.asarray(lon_deg, dtype=float)
    dlat = _gcj_dlat(lon - 105.0, lat - 35.0)
    dlon = _gcj_dlon(lon - 105.0, lat - 35.0)
    rad = np.radians(lat)
    magic = 1.0 - _GCJ_EE * np.sin(rad) ** 2
    sm = np.sqrt(magic)
    dlat = dlat * 180.0 / ((_GCJ_A * (1.0 - _GCJ_EE)) / (magic * sm) * PI)
    dlon = dlon * 180.0 / (_GCJ_A / sm * np.cos(rad) * PI)
    return lat + dlat, lon + dlon


def gcj02_to_wgs84(lat_deg, lon_deg, iters=10):
    lat_g = np.asarray(lat_deg, dtype=float)
    lon_g = np.asarray(lon_deg, dtype=float)
    lat, lon = lat_g.copy(), lon_g.copy()
    for _ in range(iters):
        la, lo = wgs84_to_gcj02(lat, lon)
        lat = lat - (la - lat_g)
        lon = lon - (lo - lon_g)
    return lat, lon
