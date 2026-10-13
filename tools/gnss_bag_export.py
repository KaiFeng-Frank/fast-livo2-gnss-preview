#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Export the gnss_comm topics of a ROS1 bag (DESIGN §12.2 Python exporter; AMENDMENTS A24 fixture layout).

Uses rosbags (Apache-2.0) as a separate program: the bag's own connection headers are the schema, nothing of
gnss_comm is imported or copied. Sub-commands:

  msgdefs <bag> <out_dir> [--verbatim]
      connection headers only (message_definition / md5sum / type) of the gnss_comm topics: a format audit,
      no message content is read (allowed on evaluation bags, AMENDMENTS A2).
  fixture <bag> <out_root> --seq NAME [--start S] [--duration D|auto] [--brdc FILE] [--rinex [--ura-semantics U]]
      A24 unit-test fixture <out_root>/<NAME>_<t0>_<D>s/ with
        msgdef/<topic>.{msgdef,md5,type}   connection header definition, md5sum, ROS1 type
        bytes/<topic>.bin                  [uint64 t_record_ns][uint32 len][len bytes] ... in record order
        golden/<name>.csv                  rosbags-decoded messages (format below)
        brdc/<BRDC file>                   same-day IGS broadcast NAV cropped to toc in [t0-2 h, t0+3 h], G/R/E/C
        rinex/<NAME>.obs [, .nav]          RINEX 3.04 written from the slice (--rinex; bag2rinex path of U3/T4);
                                           the NAV needs --ura-semantics (A16: decided by the ingest self-check)
        SHA256SUMS
      t0 = integer GPST seconds since 1980-01-06 of the first range_meas epoch of the slice; D = the slice's real
      duration in whole seconds. --duration auto takes the longest D <= --max-duration (A24: 60 s) for which every
      file stays within --max-file-bytes (DEVELOPING §4: 2 MB). The directory is written as .tmp-<name> and renamed
      only when complete and within the cap; an existing fixture directory is never overwritten.
      Fixtures are discovered by structure (msgdef/ + bytes/), never by name: gnss_io/test/fixture_discovery.hpp.
  rinex <bag> <out_dir> --name NAME [--ura-semantics U] [--start S] [--duration D]
      RINEX 3.04 OBS (and NAV when --ura-semantics is given) only.

msgdef files hold the connection-header definition without comments and blank lines (strip_msgdef): exactly the
(type, name) lines that the wire layout and the ROS1 md5 depend on — the same functional facts as gnss_io's
expectation tables — and none of the upstream .msg prose (DESIGN §12.2; the user decides before any release
whether the verbatim text may be stored, --verbatim / --msgdef-verbatim restore it).

Golden CSV (T3: the C++ decoder + gnss_io/test/golden_csv.hpp must reproduce it byte for byte):
  * header "t_record_ns,<columns>", then one row per message in record order; '\n' line ends;
  * columns = message fields in definition order; nested fields "outer.inner"; array elements "name_i"
    (i from 0); time/duration fields expand to "<name>.sec" and "<name>.nsec" (time: unsigned 32-bit decimal,
    duration: signed 32-bit decimal, as on the ROS1 wire);
  * variable-length arrays: the column set is the union over all rows (per array path, the longest
    occurrence); a row without that element leaves all its cells empty, a present element always owns at least
    one non-empty cell (an empty string inside an array, or a message element without any non-empty leaf, is
    refused) — readers decide element presence by non-empty cells, never by column existence;
  * float32/float64 '%.17g' (exact double round trip; NaN -> 'nan'), integers decimal, bool 0/1, strings
    verbatim unless they contain , " CR LF (then RFC 4180 quoting).
"""
import argparse
import datetime
import hashlib
import math
import os
import re
import shutil
import struct
import sys

TOPICS = ('range_meas', 'ephem', 'glo_ephem', 'iono_params', 'receiver_pvt', 'time_pulse_info')
CSV_NAME = {'range_meas': 'obs', 'ephem': 'ephem', 'glo_ephem': 'glo_ephem', 'iono_params': 'iono',
            'receiver_pvt': 'pvt', 'time_pulse_info': 'time_pulse'}
BDT0_GPST = 820108800 + 14      # BDT 2006-01-01 00:00:00 expressed as GPST seconds since 1980-01-06
LEAP_S = 18                     # GPST - UTC for every date the dataset covers (2024-2025)
GPST_EPOCH = datetime.datetime(1980, 1, 6)

# ---------------------------------------------------------------------------------------------------------
# ROS1 message_definition (same rules as gnss_io/src/ros1_msgdef.cpp)
# ---------------------------------------------------------------------------------------------------------
BUILTINS = {'bool', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64', 'float32',
            'float64', 'string', 'time', 'duration', 'byte', 'char'}
CANONICAL = {'byte': 'int8', 'char': 'uint8'}
FLOATS = {'float32', 'float64'}
INTS = {'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64'}
STRING_CONST = re.compile(r'\s*string\s+[A-Za-z][A-Za-z0-9_]*\s*=')


class Field:
    __slots__ = ('base', 'name', 'is_array', 'fixed_len', 'builtin', 'resolved')

    def __init__(self, base, name, is_array, fixed_len, builtin, resolved):
        self.base, self.name, self.is_array, self.fixed_len = base, name, is_array, fixed_len
        self.builtin, self.resolved = builtin, resolved


def norm_type(t):
    """'pkg/msg/Type' -> 'pkg/Type'."""
    parts = t.strip().split('/')
    if len(parts) == 3 and parts[1] == 'msg':
        return parts[0] + '/' + parts[2]
    return t.strip()


def strip_msgdef(text):
    """Definition without comments and blank lines; layout and ROS1 md5 unchanged (genmsg drops both before
    hashing). Kept: '=' section separators, 'MSG: <type>' lines, field lines, constant lines (a string constant
    verbatim: in ROS1 a '#' after 'string NAME=' belongs to the value). Idempotent."""
    out = []
    for line in text.split('\n'):
        line = line.rstrip('\r')
        t = line.strip()
        if not t:
            continue
        if len(t) >= 3 and set(t) == {'='}:
            out.append(t)
        elif t.startswith('MSG:'):
            out.append(t)
        elif STRING_CONST.match(line):
            out.append(line.rstrip())
        else:
            s = line.split('#', 1)[0].rstrip()
            if s.strip():
                out.append(s)
    return '\n'.join(out) + '\n'


def parse_msgdef(datatype, text):
    """Returns (root_name, {qualified type: [Field]}) for a connection-header message_definition."""
    root = norm_type(datatype)
    sections, name, body, expect_msg = [], root, [], False
    for line in text.split('\n'):
        line = line.rstrip('\r')
        t = line.strip()
        if len(t) >= 3 and set(t) == {'='}:
            sections.append((name, body))
            name, body, expect_msg = '', [], True
            continue
        if expect_msg:
            if not t:
                continue
            expect_msg = False
            if t.startswith('MSG:'):
                name = norm_type(t[4:])
                continue
        body.append(line)
    sections.append((name, body))
    defs = {}
    for name, body in sections:
        if not name and not ''.join(body).strip():
            continue
        if not name:
            raise ValueError('dependency section without MSG: line')
        pkg = name.split('/')[0]
        fields = []
        for line in body:
            s = line.split('#', 1)[0].strip()
            if not s or '=' in s:
                continue                     # blank / comment / constant (no wire layout)
            tok = s.split()
            if len(tok) != 2:
                raise ValueError('bad field line %r in %s' % (line, name))
            type_tok, fname = tok
            m = re.fullmatch(r'([A-Za-z][A-Za-z0-9_/]*)(\[(\d*)\])?', type_tok)
            if not m:
                raise ValueError('bad type %r in %s' % (type_tok, name))
            base, is_array = m.group(1), m.group(2) is not None
            fixed = int(m.group(3)) if (is_array and m.group(3)) else -1
            builtin = CANONICAL.get(base, base) if base in BUILTINS else None
            resolved = None
            if builtin is None:
                if type_tok == 'Header':
                    resolved = 'std_msgs/Header'
                elif '/' in base:
                    resolved = norm_type(base)
                else:
                    resolved = pkg + '/' + base
            fields.append(Field(base, fname, is_array, fixed, builtin, resolved))
        defs.setdefault(name, fields)
    return root, defs


# ---------------------------------------------------------------------------------------------------------
# Golden CSV
# ---------------------------------------------------------------------------------------------------------
def fmt_float(x):
    x = float(x)
    if math.isnan(x):
        return 'nan'
    return '%.17g' % x


def fmt_str(s):
    if any(c in s for c in ',"\r\n'):
        return '"' + s.replace('"', '""') + '"'
    return s


def attr(msg, name):
    if hasattr(msg, name):
        return getattr(msg, name)
    return getattr(msg, name + '_')      # rosbags suffixes Python keywords


def u32(v):
    return int(v) % (1 << 32)


def i32(v):
    v = int(v) % (1 << 32)
    return v - (1 << 32) if v >= (1 << 31) else v


def leaf_cells(builtin, v, out):
    # ROS1 wire: time = 2 x uint32, duration = 2 x int32. rosbags' ROS2-style Time/Duration types may hold either
    # signedness, so both parts are normalised to the wire's (the C++ twin prints the same).
    if builtin == 'time':
        out.append(str(u32(v.sec)))
        out.append(str(u32(v.nanosec)))
    elif builtin == 'duration':
        out.append(str(i32(v.sec)))
        out.append(str(i32(v.nanosec)))
    elif builtin in FLOATS:
        out.append(fmt_float(v))
    elif builtin in INTS:
        out.append(str(int(v)))
    elif builtin == 'bool':
        out.append('1' if bool(v) else '0')
    elif builtin == 'string':
        out.append(fmt_str(v))
    else:
        raise ValueError(builtin)


def leaf_names(builtin, name):
    return [name + '.sec', name + '.nsec'] if builtin in ('time', 'duration') else [name]


def shape_of(defs, tname, msg):
    """Per-field shape: builtin array -> length; message -> sub-shape; message array -> list of sub-shapes."""
    shp = {}
    for f in defs[tname]:
        v = attr(msg, f.name)
        if f.builtin is not None:
            if f.is_array:
                shp[f.name] = len(v)
        elif f.is_array:
            shp[f.name] = [shape_of(defs, f.resolved, e) for e in v]
        else:
            shp[f.name] = shape_of(defs, f.resolved, v)
    return shp


def merge_shape(defs, tname, a, b):
    out = {}
    for f in defs[tname]:
        if f.builtin is not None:
            if f.is_array:
                out[f.name] = max(a[f.name], b[f.name])
        elif f.is_array:
            la, lb = a[f.name], b[f.name]
            merged = []
            for i in range(max(len(la), len(lb))):
                if i < len(la) and i < len(lb):
                    merged.append(merge_shape(defs, f.resolved, la[i], lb[i]))
                else:
                    merged.append(la[i] if i < len(la) else lb[i])
            out[f.name] = merged
        else:
            out[f.name] = merge_shape(defs, f.resolved, a[f.name], b[f.name])
    return out


def columns(defs, tname, shp, prefix, out):
    for f in defs[tname]:
        name = prefix + f.name
        if f.builtin is not None:
            if f.is_array:
                for i in range(shp[f.name]):
                    out.extend(leaf_names(f.builtin, '%s_%d' % (name, i)))
            else:
                out.extend(leaf_names(f.builtin, name))
        elif f.is_array:
            for i, sub in enumerate(shp[f.name]):
                columns(defs, f.resolved, sub, '%s_%d.' % (name, i), out)
        else:
            columns(defs, f.resolved, shp[f.name], name + '.', out)


def cells(defs, tname, shp, msg, out, path=''):
    """Cells of one message laid out on the union shape `shp`; msg None = absent element (empty cells).
    Refuses values the union layout could not tell from an absent element (module docstring)."""
    for f in defs[tname]:
        v = attr(msg, f.name) if msg is not None else None
        where = path + f.name
        if f.builtin is not None:
            width = 2 if f.builtin in ('time', 'duration') else 1
            if f.is_array:
                n = len(v) if v is not None else 0
                for i in range(shp[f.name]):
                    if i < n:
                        before = len(out)
                        leaf_cells(f.builtin, v[i], out)
                        if all(c == '' for c in out[before:]):
                            raise ValueError('%s_%d: an empty string inside an array cannot be told from an absent '
                                             'element in the union-column CSV' % (where, i))
                    else:
                        out.extend([''] * width)
            elif v is None:
                out.extend([''] * width)
            else:
                leaf_cells(f.builtin, v, out)
        elif f.is_array:
            n = len(v) if v is not None else 0
            for i, sub in enumerate(shp[f.name]):
                before = len(out)
                cells(defs, f.resolved, sub, v[i] if i < n else None, out, '%s_%d.' % (where, i))
                if i < n and all(c == '' for c in out[before:]):
                    raise ValueError('%s_%d: a message element without any non-empty cell cannot be told from an '
                                     'absent element in the union-column CSV' % (where, i))
        else:
            cells(defs, f.resolved, shp[f.name], v, out, where + '.')


def empty_shape(defs, tname):
    shp = {}
    for f in defs[tname]:
        if f.builtin is not None:
            if f.is_array:
                shp[f.name] = 0 if f.fixed_len < 0 else f.fixed_len
        elif f.is_array:
            shp[f.name] = []
        else:
            shp[f.name] = empty_shape(defs, f.resolved)
    return shp


def golden_csv(defs, root, records):
    """records: [(t_ns, msg)] in record order -> CSV text."""
    shp = None
    for _, msg in records:
        s = shape_of(defs, root, msg)
        shp = s if shp is None else merge_shape(defs, root, shp, s)
    if shp is None:          # no messages: header from an empty shape
        shp = empty_shape(defs, root)
    head = ['t_record_ns']
    columns(defs, root, shp, '', head)
    lines = [','.join(head)]
    for t_ns, msg in records:
        row = [str(int(t_ns))]
        cells(defs, root, shp, msg, row)
        assert len(row) == len(head)
        lines.append(','.join(row))
    return '\n'.join(lines) + '\n'


# ---------------------------------------------------------------------------------------------------------
# bag access
# ---------------------------------------------------------------------------------------------------------
def open_bag(path):
    from rosbags.rosbag1 import Reader
    return Reader(path)


def gnss_connections(reader, prefix):
    out = {}
    for c in reader.connections:
        if not c.topic.startswith(prefix.rstrip('/') + '/'):
            continue
        short = c.topic.rstrip('/').split('/')[-1]
        if short in TOPICS:
            if short in out:
                raise SystemExit('two connections for %s: the fixture needs one schema per topic' % c.topic)
            out[short] = c
    return out


def ros1_type(msgtype):
    return norm_type(msgtype)


def typestore_for(conns):
    from rosbags.typesys import Stores, get_types_from_msg, get_typestore
    store = get_typestore(Stores.ROS1_NOETIC)
    for c in conns.values():
        store.register(get_types_from_msg(c.msgdef, c.msgtype))
    return store


def msgdef_files(short, c, verbatim):
    """{relative name: bytes} of the three msgdef files of one connection."""
    text = c.msgdef if verbatim else strip_msgdef(c.msgdef)
    return {short + '.msgdef': text.encode(), short + '.md5': (c.digest + '\n').encode(),
            short + '.type': (ros1_type(c.msgtype) + '\n').encode()}


def cmd_msgdefs(args):
    os.makedirs(args.out_dir, exist_ok=True)
    with open_bag(args.bag) as r:
        conns = gnss_connections(r, args.topics_prefix)
        for short, c in sorted(conns.items()):
            for fn, data in msgdef_files(short, c, args.verbatim).items():
                with open(os.path.join(args.out_dir, fn), 'wb') as f:
                    f.write(data)
            print('%-16s %-34s %s %d msgs' % (short, ros1_type(c.msgtype), c.digest, c.msgcount))
    return 0


def read_slice(reader, conns, start, duration):
    """{short: [(t_ns, raw bytes)]} for [bag_start + start, + duration) in record order (duration < 0: to the
    end). rosbags' start/stop use the bag index, so only the chunks of the window are read."""
    t_lo = reader.start_time + int(round(start * 1e9))
    t_hi = None if duration < 0 else t_lo + int(round(duration * 1e9))
    by_id = {c.id: s for s, c in conns.items()}
    out = {s: [] for s in conns}
    for c, t, raw in reader.messages(connections=list(conns.values()), start=t_lo, stop=t_hi):
        if t < t_lo or (t_hi is not None and t >= t_hi):
            continue
        out[by_id[c.id]].append((t, bytes(raw)))
    return t_lo, out


# ---------------------------------------------------------------------------------------------------------
# GPST handling for RINEX. A (week, tow) pair is never folded into one double: at 1.4e9 s the spacing of doubles
# is 2.4e-7 s, coarser than the 1e-7 s of RINEX epochs; seconds are split into whole + fraction instead.
# ---------------------------------------------------------------------------------------------------------
def gps_seconds(week, tow):
    """Whole-second quantities only (toe/toc/ttr of broadcast records)."""
    return week * 604800 + tow


def split_gpst(week, tow):
    tow = float(tow)
    whole = math.floor(tow)
    return int(week) * 604800 + int(whole), tow - whole


def calendar_int(whole):
    """Integer GPST seconds since 1980-01-06 -> (y, m, d, hh, mm, ss) on the GPST calendar."""
    dt = GPST_EPOCH + datetime.timedelta(seconds=int(whole))
    return dt.year, dt.month, dt.day, dt.hour, dt.minute, dt.second


def rinex_epoch(week, tow, sec_width):
    """(y, m, d, hh, mm, seconds text) for RINEX epoch fields: seconds with 7 decimals rendered from integer
    ticks of 1e-7 s, so the only error is the format's own rounding (<= 5e-8 s); a fraction rounding up to a
    full second carries into the calendar (never '60.0000000')."""
    whole, frac = split_gpst(week, tow)
    ticks = int(math.floor(frac * 1e7 + 0.5))
    if ticks >= 10 ** 7:
        whole += 1
        ticks -= 10 ** 7
    y, mo, d, hh, mi, ss = calendar_int(whole)
    return y, mo, d, hh, mi, '%*d.%07d' % (sec_width - 8, ss, ticks)


# ---------------------------------------------------------------------------------------------------------
# RINEX 3.04 writers (bag2rinex; T4, U3)
# ---------------------------------------------------------------------------------------------------------
SYS_OF_GC = [(1, 32, 'G', 0), (33, 59, 'R', 32), (60, 97, 'E', 59), (98, 160, 'C', 97)]


def gc_sat(sat):
    for lo, hi, letter, off in SYS_OF_GC:
        if lo <= sat <= hi:
            return letter, sat - off
    return None, 0


def rinex_band(letter, freq):
    """gnss_comm frequency -> RINEX band+attribute (same frequency table as signal_table.hpp); None = skip."""
    near = lambda f0: abs(freq - f0) < 1e3
    if letter == 'G':
        return '1C' if near(1575.42e6) else '2X' if near(1227.60e6) else '5X' if near(1176.45e6) else None
    if letter == 'E':
        return '1X' if near(1575.42e6) else '7X' if near(1207.14e6) else '5X' if near(1176.45e6) else None
    if letter == 'C':
        return ('2I' if near(1561.098e6) else '7I' if near(1207.14e6) else '1X' if near(1575.42e6)
                else '5X' if near(1176.45e6) else None)
    if letter == 'R':
        k1, k2 = (freq - 1602e6) / 0.5625e6, (freq - 1246e6) / 0.4375e6
        if abs(k1 - round(k1)) * 0.5625e6 < 1e3 and -7 <= round(k1) <= 6:
            return '1C'
        if abs(k2 - round(k2)) * 0.4375e6 < 1e3 and -7 <= round(k2) <= 6:
            return '2C'
    return None


def glo_channel(freq):
    for base, step in ((1602e6, 0.5625e6), (1246e6, 0.4375e6)):
        k = (freq - base) / step
        if abs(k - round(k)) * step < 1e3 and -7 <= round(k) <= 6:
            return int(round(k))
    return None


def rnx_line(content, label):
    return '%-60s%-20s\n' % (content[:60], label)


def rinex_obs_text(name, epochs, marker_type):
    """epochs: [((week, tow), [(letter, prn, {band: dict(C,L,lli,D,S,k)})])] -> RINEX 3.04 OBS text."""
    types = {}
    glo = {}
    for _, sats in epochs:
        for letter, prn, sig in sats:
            for band, o in sig.items():
                types.setdefault(letter, set()).add(band)
                if letter == 'R' and o.get('k') is not None:
                    glo[prn] = o['k']
    order = {'G': 0, 'R': 1, 'E': 2, 'C': 3}
    obs_types = {}
    for letter in sorted(types, key=lambda x: order[x]):
        codes = []
        for band in sorted(types[letter]):
            codes += ['C' + band, 'L' + band, 'D' + band, 'S' + band]
        obs_types[letter] = codes
    # Header records required by RINEX 3.04 (table A2) even where the bag has nothing to say ('unknown').
    lines = [rnx_line('%9.2f%11s%-20s%-20s' % (3.04, '', 'OBSERVATION DATA', 'M'), 'RINEX VERSION / TYPE'),
             rnx_line('%-20s%-20s%-20s' % ('gnss_bag_export', 'livo_gnss', ''), 'PGM / RUN BY / DATE'),
             rnx_line(name, 'MARKER NAME'),
             rnx_line('%-20s' % marker_type, 'MARKER TYPE'),
             rnx_line('%-20s%-40s' % ('unknown', 'unknown'), 'OBSERVER / AGENCY'),
             rnx_line('%-20s%-20s%-20s' % ('unknown', 'unknown', 'unknown'), 'REC # / TYPE / VERS'),
             rnx_line('%-20s%-20s' % ('unknown', 'unknown'), 'ANT # / TYPE'),
             rnx_line('%14.4f%14.4f%14.4f' % (0.0, 0.0, 0.0), 'APPROX POSITION XYZ'),
             rnx_line('%14.4f%14.4f%14.4f' % (0.0, 0.0, 0.0), 'ANTENNA: DELTA H/E/N')]
    for letter, codes in obs_types.items():
        for i in range(0, len(codes), 13):
            chunk = ''.join(' %3s' % c for c in codes[i:i + 13])
            head = '%s  %3d' % (letter, len(codes)) if i == 0 else ' ' * 6
            lines.append(rnx_line(head + chunk, 'SYS / # / OBS TYPES'))
    lines.append(rnx_line('%-20s' % 'DBHZ', 'SIGNAL STRENGTH UNIT'))
    if epochs:
        for (week, tow), label in ((epochs[0][0], 'TIME OF FIRST OBS'), (epochs[-1][0], 'TIME OF LAST OBS')):
            y, mo, d, hh, mi, ss = rinex_epoch(week, tow, 13)
            lines.append(rnx_line('%6d%6d%6d%6d%6d%s%5s%3s' % (y, mo, d, hh, mi, ss, '', 'GPS'), label))
    # No phase-shift correction was applied: observation code without a value (RTKLIB outrnx_phase_shift style).
    for letter, codes in obs_types.items():
        for c in codes:
            if c[0] == 'L':
                lines.append(rnx_line('%s %3s' % (letter, c), 'SYS / PHASE SHIFT'))
    if glo:
        items = sorted(glo.items())
        for i in range(0, len(items), 8):
            head = '%3d ' % len(items) if i == 0 else ' ' * 4
            lines.append(rnx_line(head + ''.join('R%02d %2d ' % (p, k) for p, k in items[i:i + 8]), 'GLONASS SLOT / FRQ #'))
    if 'R' in obs_types:   # code-phase biases unknown: codes with blank values
        lines.append(rnx_line(''.join(' %s %8s' % (c, '') for c in ('C1C', 'C1P', 'C2C', 'C2P')), 'GLONASS COD/PHS/BIS'))
    lines.append(rnx_line('', 'END OF HEADER'))
    for (week, tow), sats in epochs:
        y, mo, d, hh, mi, ss = rinex_epoch(week, tow, 11)
        rows = []
        for letter, prn, sig in sats:
            fields = []
            for code in obs_types[letter]:
                o = sig.get(code[1:])
                v = None if o is None else o.get(code[0])
                if v is None:
                    fields.append(' ' * 16)
                else:
                    lli = o.get('lli', 0) if code[0] == 'L' else 0
                    fields.append('%14.3f%s ' % (v, str(lli) if lli else ' '))
            rows.append(('%s%02d%s' % (letter, prn, ''.join(fields))).rstrip() + '\n')
        lines.append('> %04d %02d %02d %02d %02d%s  %d%3d\n' % (y, mo, d, hh, mi, ss, 0, len(rows)))
        lines.extend(rows)
    return ''.join(lines)


def d19(x):
    return '%19.12E' % float(x)


URA_GPS = [2.4, 3.4, 4.85, 6.85, 9.65, 13.65, 24.0, 48.0, 96.0, 192.0, 384.0, 768.0, 1536.0, 3072.0, 6144.0]


def ura_metres(value, letter, semantics):
    if semantics == 'metres':
        return float(value)
    i = int(value)                                   # index semantics
    if letter == 'E':
        if i <= 49: return i * 0.01
        if i <= 74: return 0.5 + (i - 50) * 0.02
        if i <= 99: return 1.0 + (i - 75) * 0.04
        if i <= 125: return 2.0 + (i - 100) * 0.16
        return -1.0
    return URA_GPS[i] if 0 <= i < 15 else 6144.0


def rinex_nav_text(ephs, gephs, iono, ura_semantics):
    lines = [rnx_line('%9.2f%11s%-20s%-20s' % (3.04, '', 'N: GNSS NAV DATA', 'M: MIXED'), 'RINEX VERSION / TYPE'),
             rnx_line('%-20s%-20s%-20s' % ('gnss_bag_export', 'livo_gnss', ''), 'PGM / RUN BY / DATE')]
    if iono is not None:
        a, b = iono[:4], iono[4:8]
        lines.append(rnx_line('GPSA ' + ''.join('%12.4E' % v for v in a), 'IONOSPHERIC CORR'))
        lines.append(rnx_line('GPSB ' + ''.join('%12.4E' % v for v in b), 'IONOSPHERIC CORR'))
    lines.append(rnx_line('%6d' % LEAP_S, 'LEAP SECONDS'))
    lines.append(rnx_line('', 'END OF HEADER'))

    def rec(first, rows):
        out = [first]
        for r in rows:
            out.append('    ' + ''.join(d19(v) for v in r) + '\n')
        return out

    for e in ephs:
        letter, prn = gc_sat(e.sat)
        if letter not in ('G', 'E', 'C'):
            continue
        toe = gps_seconds(e.toe.week, e.toe.tow)
        toc = gps_seconds(e.toc.week, e.toc.tow)
        ttr = gps_seconds(e.ttr.week, e.ttr.tow)
        if letter == 'C':                                    # BDT scale in the file
            toe_s, toc_s, ttr_s = toe - 14, toc - 14, ttr - 14
            week = int((toe_s - (BDT0_GPST - 14)) // 604800)
            base = BDT0_GPST - 14 + week * 604800
        else:
            toe_s, toc_s, ttr_s = toe, toc, ttr
            week = int(toe // 604800)
            base = week * 604800
        y, mo, d, hh, mi, ss = calendar_int(round(toc_s))
        first = '%s%02d %04d %02d %02d %02d %02d %02d%s%s%s\n' % (letter, prn, y, mo, d, hh, mi, ss, d19(e.af0), d19(e.af1), d19(e.af2))
        sqrta = math.sqrt(e.A)
        ura = ura_metres(e.ura, letter, ura_semantics)
        if letter == 'G':
            rows = [[e.iode, e.crs, e.delta_n, e.M0], [e.cuc, e.e, e.cus, sqrta], [toe_s - base, e.cic, e.OMG0, e.cis],
                    [e.i0, e.crc, e.omg, e.OMG_dot], [e.i_dot, e.code, week, 0.0], [ura, e.health, e.tgd0, e.iodc],
                    [ttr_s - base, 0.0, 0.0, 0.0]]
        elif letter == 'E':
            rows = [[e.iode, e.crs, e.delta_n, e.M0], [e.cuc, e.e, e.cus, sqrta], [toe_s - base, e.cic, e.OMG0, e.cis],
                    [e.i0, e.crc, e.omg, e.OMG_dot], [e.i_dot, e.code, week, 0.0], [ura, e.health, e.tgd0, e.tgd1],
                    [ttr_s - base, 0.0, 0.0, 0.0]]
        else:
            rows = [[e.iode, e.crs, e.delta_n, e.M0], [e.cuc, e.e, e.cus, sqrta], [toe_s - base, e.cic, e.OMG0, e.cis],
                    [e.i0, e.crc, e.omg, e.OMG_dot], [e.i_dot, 0.0, week, 0.0], [ura, e.health, e.tgd0, e.tgd1],
                    [ttr_s - base, e.iodc, 0.0, 0.0]]
        lines.extend(rec(first, rows))
    for g in gephs:
        letter, prn = gc_sat(g.sat)
        if letter != 'R':
            continue
        toe_utc = gps_seconds(g.toe.week, g.toe.tow) - LEAP_S
        tof_utc = gps_seconds(g.ttr.week, g.ttr.tow) - LEAP_S
        y, mo, d, hh, mi, ss = calendar_int(round(toe_utc))
        first = 'R%02d %04d %02d %02d %02d %02d %02d%s%s%s\n' % (prn, y, mo, d, hh, mi, ss, d19(-g.tau_n), d19(g.gamma),
                                                                d19(tof_utc % 604800))
        rows = [[g.pos_x / 1e3, g.vel_x / 1e3, g.acc_x / 1e3, g.health], [g.pos_y / 1e3, g.vel_y / 1e3, g.acc_y / 1e3, g.freqo],
                [g.pos_z / 1e3, g.vel_z / 1e3, g.acc_z / 1e3, g.age]]
        lines.extend(rec(first, rows))
    return ''.join(lines)


def rinex_from_slice(msgs, name, ura_semantics, marker_type):
    """msgs: {short: [(t_ns, msg)]}. Returns (obs text or None, nav text or None, number of epochs). The NAV is
    only written with a URA semantics decision (A16)."""
    epochs, obs = [], None
    if 'range_meas' in msgs:
        for _, m in msgs['range_meas']:
            if not len(m.meas):
                continue
            sats = []
            for o in m.meas:
                letter, prn = gc_sat(int(o.sat))
                if letter is None:
                    continue
                sig = {}
                taken = set()   # first entry per band wins even when it carries nothing (gnss_io signal_order.hpp rule 1)
                for k in range(len(o.freqs)):
                    f = float(o.freqs[k])
                    band = rinex_band(letter, f)
                    if band is None or band in taken:
                        continue
                    taken.add(band)
                    st = int(o.status[k])
                    entry = {'k': glo_channel(f) if letter == 'R' else None}
                    psr, cp, dopp, cn0 = float(o.psr[k]), float(o.cp[k]), float(o.dopp[k]), float(o.CN0[k])
                    if (st & 1) and psr != 0.0:
                        entry['C'] = psr
                    if (st & 2) and cp != 0.0:
                        entry['L'] = cp
                        entry['lli'] = (int(o.LLI[k]) & 1) | (0 if (st & 4) else 2)
                    if dopp != 0.0:
                        entry['D'] = dopp
                    if cn0 != 0.0:
                        entry['S'] = cn0
                    if any(x in entry for x in 'CLDS'):
                        sig[band] = entry
                if sig:
                    sats.append((letter, prn, sig))
            if sats:
                epochs.append(((m.meas[0].time.week, m.meas[0].time.tow), sats))
        obs = rinex_obs_text(name, epochs, marker_type)
    nav = None
    if ura_semantics:
        ephs = [m for _, m in msgs.get('ephem', [])]
        gephs = [m for _, m in msgs.get('glo_ephem', [])]
        iono = None
        for _, m in msgs.get('iono_params', []):
            d = list(m.data)
            if len(d) == 8:
                iono = d
                break
        nav = rinex_nav_text(ephs, gephs, iono, ura_semantics)
    return obs, nav, len(epochs)


# ---------------------------------------------------------------------------------------------------------
# BRDC crop
# ---------------------------------------------------------------------------------------------------------
REC_START = re.compile(r'^([GRECJIS])(\d\d) (\d{4}) (\d\d) (\d\d) (\d\d) (\d\d) (\d\d)')


def crop_brdc(src, t_lo_gps, t_hi_gps, keep='GREC'):
    """Header + the records of the systems in `keep` whose epoch (read as GPST; the BDT/UTC offsets of < 20 s do
    not matter here) lies in [t_lo, t_hi]. Default G/R/E/C: the systems gnss_comm can number (DESIGN §12.3), the
    only ones a bag-vs-BRDC comparison can use."""
    out, keep_rec, n_keep = [], False, 0
    with open(src) as f:
        lines = f.readlines()
    i = 0
    while i < len(lines):
        out.append(lines[i])
        if 'END OF HEADER' in lines[i][60:]:
            out.insert(len(out) - 1, rnx_line('cropped by gnss_bag_export.py: %d..%d GPST s, %s' % (t_lo_gps, t_hi_gps, keep),
                                              'COMMENT'))
            i += 1
            break
        i += 1
    for line in lines[i:]:
        m = REC_START.match(line)
        if m:
            letter = m.group(1)
            dt = datetime.datetime(*[int(m.group(k)) for k in range(3, 9)])
            t = (dt - GPST_EPOCH).total_seconds()
            keep_rec = letter in keep and t_lo_gps <= t <= t_hi_gps
            n_keep += keep_rec
        if keep_rec:
            out.append(line)
    return ''.join(out), n_keep


# ---------------------------------------------------------------------------------------------------------
# fixture
# ---------------------------------------------------------------------------------------------------------
def sha256_bytes(data):
    return hashlib.sha256(data).hexdigest()


def fixture_files(conns, recs, t_hi, args, name):
    """{relative path: bytes} of every exporter-made file except the BRDC crop, for the records before t_hi."""
    files = {}
    msgs = {}
    for short, c in sorted(conns.items()):
        sub = [(t, raw, m) for (t, raw, m) in recs[short] if t_hi is None or t < t_hi]
        for fn, data in msgdef_files(short, c, args.msgdef_verbatim).items():
            files['msgdef/' + fn] = data
        files['bytes/%s.bin' % short] = b''.join(struct.pack('<QI', t, len(raw)) + raw for t, raw, _ in sub)
        root, defs = parse_msgdef(c.msgtype, c.msgdef)
        files['golden/%s.csv' % CSV_NAME[short]] = golden_csv(defs, root, [(t, m) for t, _, m in sub]).encode()
        msgs[short] = [(t, m) for t, _, m in sub]
    if args.rinex:
        obs, nav, _ = rinex_from_slice(msgs, name, args.ura_semantics, args.marker_type)
        if obs is not None:
            files['rinex/%s.obs' % name] = obs.encode()
        if nav is not None:
            files['rinex/%s.nav' % name] = nav.encode()
    return files


def cmd_fixture(args):
    max_d = args.max_duration if args.duration == 'auto' else float(args.duration)
    with open_bag(args.bag) as r:
        conns = gnss_connections(r, args.topics_prefix)
        if 'range_meas' not in conns:
            raise SystemExit('no %s/range_meas connection' % args.topics_prefix)
        store = typestore_for(conns)
        t_lo, sl = read_slice(r, conns, args.start, max_d)
    recs = {s: [(t, raw, store.deserialize_ros1(raw, conns[s].msgtype)) for t, raw in v] for s, v in sl.items()}
    t0 = None
    for _, _, m in recs['range_meas']:
        if len(m.meas):
            t0 = split_gpst(m.meas[0].time.week, m.meas[0].time.tow)[0]
            break
    if t0 is None:
        raise SystemExit('no range_meas epoch with satellites in the slice')

    def too_big(files):
        return [(p, len(b)) for p, b in sorted(files.items()) if len(b) > args.max_file_bytes]

    brdc = None
    if args.brdc:
        text, n = crop_brdc(args.brdc, t0 - 7200, t0 + 10800, args.brdc_systems)
        brdc = ('brdc/' + os.path.basename(args.brdc), text.encode())
        print('brdc: %d records kept, %d bytes' % (n, len(brdc[1])))
        if len(brdc[1]) > args.max_file_bytes:
            raise SystemExit('BRDC crop is %d bytes > %d' % (len(brdc[1]), args.max_file_bytes))
    if args.duration == 'auto':
        # Every file grows monotonically with the window, so the longest admissible whole-second D is a bisection.
        lo, hi = 0, int(math.floor(max_d))
        while lo < hi:
            mid = (lo + hi + 1) // 2
            if too_big(fixture_files(conns, recs, t_lo + mid * 10 ** 9, args, args.seq)):
                hi = mid - 1
            else:
                lo = mid
        if lo == 0:
            raise SystemExit('even a 1 s slice exceeds %d bytes per file' % args.max_file_bytes)
        dur = lo
    else:
        dur = int(round(float(args.duration)))
    t_hi = t_lo + int(round((dur if args.duration == 'auto' else float(args.duration)) * 1e9))
    files = fixture_files(conns, recs, t_hi, args, args.seq)
    if brdc:
        files[brdc[0]] = brdc[1]
    big = too_big(files)
    if big:
        for rel, sz in big:
            print('TOO BIG: %s %d bytes > %d (shorten --duration or use --duration auto)' % (rel, sz, args.max_file_bytes),
                  file=sys.stderr)
        return 3
    name = '%s_%d_%ds' % (args.seq, t0, dur)
    final = os.path.join(args.out_root, name)
    if os.path.exists(final):
        raise SystemExit('%s exists: write to another root and copy (a fixture may hold files of other work packages, '
                         'e.g. golden/pntpos_unpatched.hex)' % final)
    tmp = os.path.join(args.out_root, '.tmp-' + name)   # dot-dirs are never fixtures (fixture_discovery.hpp)
    if os.path.exists(tmp):
        shutil.rmtree(tmp)
    for rel, data in files.items():
        p = os.path.join(tmp, rel)
        os.makedirs(os.path.dirname(p), exist_ok=True)
        with open(p, 'wb') as f:
            f.write(data)
    sums = sorted((rel, sha256_bytes(data), len(data)) for rel, data in files.items())
    with open(os.path.join(tmp, 'SHA256SUMS'), 'w') as f:
        for rel, h, _ in sums:
            f.write('%s  %s\n' % (h, rel))
    os.rename(tmp, final)
    print('fixture %s (%d s, %d range_meas records)' % (final, dur, sum(1 for t, _, _ in recs['range_meas'] if t < t_hi)))
    for rel, h, sz in sums:
        print('| `%s` | %d | `%s` |' % (rel, sz, h))
    return 0


def cmd_rinex(args):
    with open_bag(args.bag) as r:
        conns = gnss_connections(r, args.topics_prefix)
        store = typestore_for(conns)
        _, sl = read_slice(r, conns, args.start, args.duration)
    msgs = {s: [(t, store.deserialize_ros1(raw, conns[s].msgtype)) for t, raw in v] for s, v in sl.items()}
    os.makedirs(args.out_dir, exist_ok=True)
    obs, nav, n = rinex_from_slice(msgs, args.name, args.ura_semantics, args.marker_type)
    for text, ext in ((obs, '.obs'), (nav, '.nav')):
        if text is not None:
            with open(os.path.join(args.out_dir, args.name + ext), 'w') as f:
                f.write(text)
    print('rinex: %d epochs%s' % (n, '' if nav is not None else ' (no NAV: --ura-semantics not given, A16)'))
    return 0


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__.split('\n\n')[0], formatter_class=argparse.RawDescriptionHelpFormatter,
                                epilog=__doc__.split('\n\n', 1)[1])
    p.add_argument('--topics-prefix', default='/ublox_driver', help='gnss_comm topic prefix')
    sub = p.add_subparsers(dest='cmd', required=True)
    a = sub.add_parser('msgdefs', help='dump connection headers (format audit only)')
    a.add_argument('bag')
    a.add_argument('out_dir')
    a.add_argument('--verbatim', action='store_true', help='keep comments (see module docstring)')
    b = sub.add_parser('fixture', help='A24 unit-test fixture')
    b.add_argument('bag')
    b.add_argument('out_root')
    b.add_argument('--seq', required=True, help='sequence name used in the fixture directory')
    b.add_argument('--start', type=float, default=0.0, help='start offset from bag begin [s]')
    b.add_argument('--duration', default='auto', help="window [s], or 'auto' (longest whole-second window within the cap)")
    b.add_argument('--max-duration', type=float, default=60.0, help='upper bound for --duration auto (A24 nominal 60 s)')
    b.add_argument('--brdc', default='', help='same-day IGS BRDC RINEX NAV to crop into the fixture')
    b.add_argument('--brdc-systems', default='GREC', help='systems kept in the BRDC crop')
    b.add_argument('--rinex', action='store_true', help='also write RINEX 3.04 OBS (and NAV with --ura-semantics) of the slice')
    b.add_argument('--ura-semantics', choices=('metres', 'index'), default=None, help='URA meaning of the bag ephemerides (A16)')
    b.add_argument('--marker-type', default='GROUND_CRAFT', help='RINEX MARKER TYPE (M3DGR: ground robot)')
    b.add_argument('--msgdef-verbatim', action='store_true', help='store the connection-header text with comments')
    b.add_argument('--max-file-bytes', type=int, default=2 * 1024 * 1024, help='per-file size cap (DEVELOPING §4)')
    c = sub.add_parser('rinex', help='RINEX 3.04 OBS (+ NAV with --ura-semantics) only')
    c.add_argument('bag')
    c.add_argument('out_dir')
    c.add_argument('--name', required=True)
    c.add_argument('--ura-semantics', choices=('metres', 'index'), default=None)
    c.add_argument('--marker-type', default='GROUND_CRAFT')
    c.add_argument('--start', type=float, default=0.0)
    c.add_argument('--duration', type=float, default=-1.0)
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    if args.cmd == 'fixture' and args.duration != 'auto':
        try:
            float(args.duration)
        except ValueError:
            raise SystemExit("--duration must be a number of seconds or 'auto'")
    return {'msgdefs': cmd_msgdefs, 'fixture': cmd_fixture, 'rinex': cmd_rinex}[args.cmd](args)


if __name__ == '__main__':
    sys.exit(main())
