#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
# vg_origin.py — group valgrind memcheck error contexts by ORIGIN (--track-origins): the allocation whose
# uninitialised bytes were consumed, or, for Invalid read/write, the block the address refers to. Used for
# REF_R_GATES.md §5/§5b (review M3: per-origin attribution, not a single "all from one source" claim).
#   vg_origin.py <valgrind.log>
import re, sys, collections
path = sys.argv[1]
kind_re = re.compile(r'^==\d+== (Invalid read of size \d+|Invalid write of size \d+|Conditional jump or move depends on uninitialised value\(s\)|Use of uninitialised value of size \d+|Invalid free\(\)[^\n]*|Mismatched free\(\)[^\n]*|Syscall param [^\n]*|Jump to the invalid address[^\n]*|Process terminating[^\n]*)')
frame_re = re.compile(r'^==\d+==\s+(?:at|by) 0x[0-9A-F]+: (.*?) \((?:in )?(.*?)\)$')
origin_re = re.compile(r'^==\d+==\s+Uninitialised value was created by a (heap|stack|client request|\S+) allocation')
addr_re = re.compile(r'^==\d+==\s+Address 0x[0-9a-f]+ is (.*)$')
blocks = []
cur = None
for line in open(path, errors='replace'):
    line = line.rstrip('\n')
    m = kind_re.match(line)
    if m:
        cur = {'kind': m.group(1), 'frames': [], 'origin': None, 'ofr': [], 'addr': None, 'afr': [], 'mode': 'err'}
        blocks.append(cur); continue
    if cur is None: continue
    if origin_re.match(line):
        cur['origin'] = origin_re.match(line).group(1); cur['mode'] = 'orig'; continue
    if addr_re.match(line):
        cur['addr'] = addr_re.match(line).group(1)[:90]; cur['mode'] = 'addr'; continue
    fm = frame_re.match(line)
    if fm:
        ({'err': cur['frames'], 'orig': cur['ofr'], 'addr': cur['afr']}[cur['mode']]).append((fm.group(1), fm.group(2)))
        continue
    if line.endswith('== '):
        cur = None
def short(frs, n=4):
    out = []
    for f, lib in frs[:n]:
        f = re.sub(r'\(.*', '', f)[:48]
        lib = lib.rsplit('/', 1)[-1][:28]
        out.append(f"{f}@{lib}")
    return ' <- '.join(out)
def interesting(frs):
    skip = ('memmove','memcpy','__ieee754','sincos','do_sin','do_cos','__printf','hack_digit','__mpn','atan2','vgpreload','printf_fp','Eigen::','std::','libstdc++','libc.so','libm.so','libopencv','libpcl','libSophus','libvikit','???')
    for f, lib in frs:
        if not any(s in f or s in lib for s in skip): return f"{re.sub(r'\(.*', '', f)[:60]}@{lib.rsplit('/',1)[-1][:24]}"
    return short(frs, 1)
c = collections.Counter(); ex = {}; first = {}
for idx, b in enumerate(blocks):
    if b['kind'].startswith(('Invalid read', 'Invalid write', 'Invalid free', 'Mismatched')):
        key = (b['kind'][:14], interesting(b['frames']), (b['addr'] or '?')[:60], short(b['afr'], 3))
    else:
        key = ('uninit', b['origin'] or 'no-origin', short(b['ofr'], 4), '')
    c[key] += 1
    ex.setdefault(key, interesting(b['frames']))
    first.setdefault(key, idx + 1)
print("total error contexts:", len(blocks))
for k, n in sorted(c.items(), key=lambda kv: -kv[1]):
    print(f"{n:6d}  first#{first[k]:<5d} {k[0]:8s} {k[1]:60s} | {k[2]} | {k[3]}   e.g. consumer: {ex[k]}")
# Consumer stacks (P2b): for every context the error kind and the top of its own stack, grouped, so a reader can
# check that each remaining context ends in a print path (printf / ROS_INFO) and never in estimator state.
cons = collections.Counter()
for b in blocks:
    frs = [re.sub(r'\(.*', '', f)[:44] for f, lib in b['frames'][:8]]
    cons[(b['kind'][:48], ' <- '.join(frs))] += 1
print("consumer stacks (contexts | kind | top 8 frames):")
for (kind, st), n in sorted(cons.items(), key=lambda kv: -kv[1]):
    print(f"{n:6d}  {kind:48s} | {st}")
