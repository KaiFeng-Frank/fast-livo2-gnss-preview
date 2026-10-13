# SPDX-License-Identifier: BSD-3-Clause
"""The per-sequence time-calibration file (AMENDMENTS A18): writer and a strict reader of the restricted subset.

Contract with TimeMap::loadCalibFile (WP-a, hand-written parser, no yaml-cpp):
  * top level: exactly `sequence`, `tool_version`, `leap_s`, `segments` in this order, scalars only;
  * `segments:` is a block list; each item is `  - key: value` followed by `    key: value` lines (2-space indent);
  * the first six keys of every segment are, in order, t_from_gpst_s, offset_s, drift_ppm, t_ref_gpst_s,
    sigma_toff_s, tdcp_paused; further keys (method, peak_corr, window_scatter_ms, ...) are informational and
    ignored by the core;
  * no comments, anchors, flow syntax, quotes or tabs; numbers are finite and written with Python repr (shortest
    round-trip text, parseable by strtod); booleans are `true`/`false`; segments sorted by t_from; the first
    segment's t_from is written as 0 (it is treated as -inf by the core).
Meaning (DESIGN §1): t_est = t_gpst_true - leap + offset_s + drift_ppm*1e-6*(t_gpst - t_ref_gpst_s).
Time origin of every *_gpst_s value: GTime::toSec() (gnss/include/livo_gnss/types.hpp), i.e. seconds since the
Unix epoch 1970-01-01 on the GPST scale, without leap seconds: 315964800 + week*604800 + tow (fl2g.gnsstime.gpst).
(The withdrawn A18 example value 1416000000 read as seconds since 1980-01-06; a parser that assumed that origin
would be 315964800 s off, and with a drift of 3 ppm the drift term alone would move the mapping by 3e-6 * 315964800 s
= about 948 s — AMENDMENTS A18 as corrected in P2b.) `dumps(..., span=...)` refuses a t_ref (when drift != 0) or a
segment start outside the data span ± 1 day, which catches that mistake.
"""
import math
import re

REQUIRED_SEGMENT_KEYS = ("t_from_gpst_s", "offset_s", "drift_ppm", "t_ref_gpst_s", "sigma_toff_s", "tdcp_paused")
TOP_KEYS = ("sequence", "tool_version", "leap_s", "segments")
_SAFE = re.compile(r"^[A-Za-z0-9_.\-]+$")


def _scalar(v):
    if isinstance(v, bool):
        return "true" if v else "false"
    if isinstance(v, int):
        return str(v)
    if isinstance(v, float):
        if not math.isfinite(v):
            raise ValueError("non-finite value in time calibration file")
        return repr(float(v))
    s = str(v)
    if not _SAFE.match(s):
        raise ValueError("unsafe scalar %r (restricted YAML subset: [A-Za-z0-9_.-]+)" % s)
    return s


def sanitize_name(name):
    return re.sub(r"[^A-Za-z0-9_.\-]", "_", name) or "unnamed"


SPAN_MARGIN_S = 86400.0


def check_span(segments, span):
    """Every non-first t_from, and every t_ref of a segment with drift, must lie in [span0 - 1 day, span1 + 1 day]
    (GPST seconds, Unix origin). A value on another origin (1980: -315964800 s) or a relative time fails."""
    lo, hi = float(span[0]) - SPAN_MARGIN_S, float(span[1]) + SPAN_MARGIN_S
    segs = sorted(segments, key=lambda s: s["t_from_gpst_s"])
    for n, seg in enumerate(segs):
        if n > 0 and not lo <= float(seg["t_from_gpst_s"]) <= hi:
            raise ValueError("segment %d: t_from_gpst_s %r outside the data span [%r, %r] +- 1 day (time origin: GTime::toSec, "
                             "seconds since 1970 on the GPST scale)" % (n, seg["t_from_gpst_s"], span[0], span[1]))
        if float(seg["drift_ppm"]) != 0.0 and not lo <= float(seg["t_ref_gpst_s"]) <= hi:
            raise ValueError("segment %d: t_ref_gpst_s %r outside the data span [%r, %r] +- 1 day (time origin: GTime::toSec, "
                             "seconds since 1970 on the GPST scale)" % (n, seg["t_ref_gpst_s"], span[0], span[1]))


def dumps(sequence, leap_s, segments, tool_version=1, span=None):
    if not segments:
        raise ValueError("at least one segment is required")
    if span is not None:
        check_span(segments, span)
    segs = sorted(segments, key=lambda s: s["t_from_gpst_s"])
    lines = ["sequence: %s" % _scalar(sanitize_name(sequence)), "tool_version: %d" % int(tool_version),
             "leap_s: %d" % int(leap_s), "segments:"]
    for n, seg in enumerate(segs):
        missing = [k for k in REQUIRED_SEGMENT_KEYS if k not in seg]
        if missing:
            raise ValueError("segment %d misses %s" % (n, missing))
        keys = list(REQUIRED_SEGMENT_KEYS) + [k for k in seg if k not in REQUIRED_SEGMENT_KEYS]
        for i, k in enumerate(keys):
            v = seg[k]
            if k == "t_from_gpst_s" and n == 0:
                v = 0
            elif k in ("t_from_gpst_s", "offset_s", "drift_ppm", "t_ref_gpst_s", "sigma_toff_s"):
                v = float(v)
            elif k == "tdcp_paused":
                v = bool(v)
            if not re.match(r"^[a-z_][a-z0-9_]*$", k):
                raise ValueError("bad key %r" % k)
            lines.append(("  - " if i == 0 else "    ") + "%s: %s" % (k, _scalar(v)))
    return "\n".join(lines) + "\n"


def write(path, sequence, leap_s, segments, tool_version=1, span=None):
    text = dumps(sequence, leap_s, segments, tool_version, span)
    loads(text)  # self-check: what we write must pass the strict reader
    with open(path, "w") as f:
        f.write(text)
    return text


def _value(text):
    if text in ("true", "false"):
        return text == "true"
    if re.match(r"^[+-]?(\d+)$", text):
        return int(text)
    try:
        v = float(text)
    except ValueError:
        if not _SAFE.match(text):
            raise ValueError("unsupported scalar %r" % text)
        return text
    if not math.isfinite(v):
        raise ValueError("non-finite number %r" % text)
    return v


def loads(text):
    """Strict reader of the A18 subset; raises ValueError on anything outside it."""
    if "\t" in text:
        raise ValueError("tabs are not allowed")
    top = {}
    segments = []
    in_segments = False
    cur = None
    for ln_no, line in enumerate(text.split("\n"), start=1):
        if not line.strip():
            continue
        if "#" in line or "&" in line or "*" in line or "{" in line or "[" in line or '"' in line or "'" in line:
            raise ValueError("line %d: comments/anchors/flow/quotes are outside the subset" % ln_no)
        if not in_segments:
            m = re.match(r"^([a-z_]+): ?(.*)$", line)
            if not m:
                raise ValueError("line %d: expected a top-level key" % ln_no)
            k, v = m.group(1), m.group(2)
            if k == "segments":
                if v != "":
                    raise ValueError("line %d: segments must be a block list" % ln_no)
                in_segments = True
                continue
            top[k] = _value(v)
            continue
        m = re.match(r"^  - ([a-z_][a-z0-9_]*): (.+)$", line)
        if m:
            cur = {m.group(1): _value(m.group(2))}
            cur_keys = [m.group(1)]
            segments.append((cur, cur_keys))
            continue
        m = re.match(r"^    ([a-z_][a-z0-9_]*): (.+)$", line)
        if m and cur is not None:
            if m.group(1) in cur:
                raise ValueError("line %d: duplicate key" % ln_no)
            cur[m.group(1)] = _value(m.group(2))
            cur_keys.append(m.group(1))
            continue
        raise ValueError("line %d: outside the restricted subset: %r" % (ln_no, line))
    if list(top.keys()) != list(TOP_KEYS[:3]):
        raise ValueError("top-level keys must be exactly %s in order, got %s" % (TOP_KEYS, list(top.keys())))
    if not segments:
        raise ValueError("no segments")
    out = []
    prev_from = -math.inf
    for n, (seg, keys) in enumerate(segments):
        if tuple(keys[:6]) != REQUIRED_SEGMENT_KEYS:
            raise ValueError("segment %d: first six keys must be %s, got %s" % (n, REQUIRED_SEGMENT_KEYS, keys[:6]))
        if not isinstance(seg["tdcp_paused"], bool):
            raise ValueError("segment %d: tdcp_paused must be true/false" % n)
        for k in REQUIRED_SEGMENT_KEYS[:5]:
            if isinstance(seg[k], bool) or not isinstance(seg[k], (int, float)):
                raise ValueError("segment %d: %s must be a number" % (n, k))
        if n > 0 and seg["t_from_gpst_s"] <= prev_from:
            raise ValueError("segments must be sorted by t_from_gpst_s")
        prev_from = seg["t_from_gpst_s"] if n > 0 else -math.inf
        out.append(seg)
    top["segments"] = out
    return top


def evaluate(calib, t_gpst, leap_s=None):
    """t_est for a true GPST instant under a loaded calibration (first segment covers -inf)."""
    segs = calib["segments"]
    seg = segs[0]
    for s in segs[1:]:
        if t_gpst >= s["t_from_gpst_s"]:
            seg = s
    leap = calib["leap_s"] if leap_s is None else leap_s
    return t_gpst - leap + seg["offset_s"] + seg["drift_ppm"] * 1e-6 * (t_gpst - seg["t_ref_gpst_s"])
