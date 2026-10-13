# SPDX-License-Identifier: BSD-3-Clause
"""GNSS time scales used by the offline tools (DESIGN §1 "时间").

Absolute times are float seconds on Unix-like axes:
  * GPST seconds : 315964800 + week*604800 + tow — GPS time counted from the Unix epoch *without* leap seconds.
                   gnss_comm's ublox_driver stamps receiver_lla headers on this axis (audit finding).
  * UTC seconds  : GPST seconds - leap(t) — Unix time, i.e. the host clock scale when the host runs NTP.
float64 resolves ~2.4e-7 s at 1.7e9 s, three orders below the millisecond effects the tools measure.
The leap-second table is RTKLIB's (rtkcmn.c leaps[]) so tools and the C++ TimeMap (U1) agree; BDT = GPST - 14 s
(week offset 1356), GST seconds == GPST seconds (week offset 1024), GLONASST = UTC + 3 h.
"""
import calendar
import math

GPST0_UNIX = 315964800.0          # 1980-01-06T00:00:00Z as Unix seconds
WEEK_S = 604800.0
BDT_MINUS_GPST_S = -14.0
BDS_WEEK_OFFSET = 1356
GST_WEEK_OFFSET = 1024
GLOT_MINUS_UTC_S = 3 * 3600.0

# (UTC date from which the value applies, GPST - UTC [s]), newest first (RTKLIB rtkcmn.c leaps[]).
_LEAP_TABLE = (
    ((2017, 1, 1), 18), ((2015, 7, 1), 17), ((2012, 7, 1), 16), ((2009, 1, 1), 15), ((2006, 1, 1), 14),
    ((1999, 1, 1), 13), ((1997, 7, 1), 12), ((1996, 1, 1), 11), ((1994, 7, 1), 10), ((1993, 7, 1), 9),
    ((1992, 7, 1), 8), ((1991, 1, 1), 7), ((1990, 1, 1), 6), ((1988, 1, 1), 5), ((1985, 7, 1), 4),
    ((1983, 7, 1), 3), ((1982, 7, 1), 2), ((1981, 7, 1), 1),
)
_LEAPS = tuple((float(calendar.timegm((y, m, d, 0, 0, 0))), n) for (y, m, d), n in _LEAP_TABLE)


def leap_at_utc(t_utc):
    """GPST - UTC [s] at a UTC instant (Unix seconds)."""
    for t0, n in _LEAPS:
        if t_utc >= t0:
            return n
    return 0


def leap_at_gpst(t_gpst):
    """GPST - UTC [s] at a GPST instant (GPST seconds): the UTC date of that instant selects the entry."""
    for t0, n in _LEAPS:
        if t_gpst - n >= t0:
            return n
    return 0


def gpst(week, tow):
    """GPST seconds of a GPS (week, tow)."""
    return GPST0_UNIX + float(week) * WEEK_S + float(tow)


def gpst_to_utc(t_gpst):
    return t_gpst - leap_at_gpst(t_gpst)


def utc_to_gpst(t_utc):
    return t_utc + leap_at_utc(t_utc)


def week_tow(t_gpst):
    """(GPS week, tow) of GPST seconds."""
    s = t_gpst - GPST0_UNIX
    w = int(math.floor(s / WEEK_S))
    return w, s - w * WEEK_S


def bdt_to_gpst(bdt_week, bdt_tow):
    """GPST seconds of a BDT (week, tow)."""
    return gpst(bdt_week + BDS_WEEK_OFFSET, bdt_tow) - BDT_MINUS_GPST_S


def gpst_to_bdt_week_tow(t_gpst):
    w, tow = week_tow(t_gpst + BDT_MINUS_GPST_S)
    return w - BDS_WEEK_OFFSET, tow


def calendar_to_seconds(year, month, day, hour, minute, sec):
    """Calendar fields read as if they were UTC -> seconds; used for RINEX epochs of any time system
    (the caller then shifts by the system's offset)."""
    isec = int(math.floor(sec))
    return float(calendar.timegm((int(year), int(month), int(day), int(hour), int(minute), isec))) + (sec - isec)


def utc_iso(t_utc):
    """ISO-8601 text of a UTC instant (millisecond resolution) for reports."""
    import datetime
    d = datetime.datetime.fromtimestamp(t_utc, datetime.timezone.utc)
    return d.strftime("%Y-%m-%dT%H:%M:%S.") + "%03dZ" % int(round(d.microsecond / 1000.0) % 1000)
