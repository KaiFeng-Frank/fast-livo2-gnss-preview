# SPDX-License-Identifier: BSD-3-Clause
"""Time rules of the slim bags (A80).

The SmartPNT-MSF platform stamps its sensors with GPS time of day on a wrong calendar date (bag headers read 2014-07-xx
for 2024 data): the IMU stamps equal the I300 IMR record times (GPST seconds of week) to the microsecond, with the
date replaced. The slim bag follows the convention of the rest of the repository (DESIGN §1, time_map.cpp: sensor
stamps are Unix UTC seconds, GPST = UTC + leap): every stamp of a sequence is shifted by one constant integer number of
nanoseconds so that
        stamp_new = stamp_old + shift,   shift = (true GPST second of the first IMU stamp) - leap - stamp_old_first,
the true date being fixed by the sequence's own GPS data (first RINEX epoch / first ground-truth epoch), the leap
count being that of the GPST date (18 s for every 2017-2026 date). Streams that wrap at the GPST midnight (a drop of
about a day in the stamps) are unwrapped.
"""
from fl2g import gnsstime

NS = 1000000000
DAY_NS = 86400 * NS


def tod_ns(stamp_ns):
    return int(stamp_ns) % DAY_NS


def resolve_shift_ns(first_stamp_ns, ref_gpst_s, max_gap_s=3600.0):
    """Integer ns shift mapping the bogus-date stamps (time of day = GPST time of day) to UTC Unix stamps.

    `ref_gpst_s`: a GPST instant (seconds on the Unix-origin GPST scale, gnsstime.gpst) known to be near the stream's
    start: the first RINEX epoch of the sequence. The GPST day is the one that puts the first stamp nearest to it; a
    gap above `max_gap_s` (one hour: the receiver and the IMU start minutes apart) is an error, not a guess. Returns
    (shift_ns, gpst_first_s, leap_s)."""
    tod = tod_ns(first_stamp_ns) / float(NS)
    base_day = int(ref_gpst_s // 86400)
    best = None
    for k in (-1, 0, 1):
        cand = (base_day + k) * 86400 + tod
        d = abs(cand - ref_gpst_s)
        if best is None or d < best[0]:
            best = (d, cand)
    if best[0] > max_gap_s:
        raise ValueError("the first stamp (time of day %.3f s) is %.0f s from the reference instant: wrong sequence or clock?"
                         % (tod, best[0]))
    cand = best[1]
    day_start_ns = int(round((cand - tod))) * NS
    gpst_first_ns = day_start_ns + tod_ns(first_stamp_ns)          # exact integer arithmetic
    leap = gnsstime.leap_at_gpst(cand)
    shift = gpst_first_ns - leap * NS - int(first_stamp_ns)
    return shift, cand, leap


class Unwrapper(object):
    """Adds one day to the stamps after each backward jump of more than half a day (time-of-day wrap at midnight)."""

    def __init__(self):
        self.last = None
        self.days = 0
        self.wraps = 0

    def __call__(self, stamp_ns):
        s = int(stamp_ns)
        if self.last is not None and s + self.days * DAY_NS < self.last - DAY_NS // 2:
            self.days += 1
            self.wraps += 1
        out = s + self.days * DAY_NS
        self.last = out
        return out


def to_utc_ns(stamp_ns, shift_ns):
    return int(stamp_ns) + int(shift_ns)


def utc_ns_of_gpst(week, tow):
    """UTC Unix nanoseconds of a GPST (week, tow) (the same leap rule as the sensors)."""
    t = gnsstime.gpst(week, 0.0)
    whole = int(t) + int(tow // 1)
    frac = tow - (tow // 1)
    leap = gnsstime.leap_at_gpst(whole)
    return (whole - leap) * NS + int(round(frac * NS))
