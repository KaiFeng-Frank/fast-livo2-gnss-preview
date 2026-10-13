#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Compile-command gate (IMPL_PLAN 2): compare the compile commands of the upstream targets between the
reference workspace (ws_upstream) and a HEAD workspace.

For every source file of the upstream package (src/*.cpp of fast_livo, and optionally vikit) the two commands
are tokenised; `-I<dir>`, `-isystem <dir>` and the `-o`/source paths are removed and compared separately.
Anything else (flags, -D definitions, -march, -std, -f*) must be identical. Include directories may only be
*added* on the HEAD side (new subdirectories), never removed or reordered relative to each other.

usage: compare_compile_commands.py <upstream compile_commands.json> <head compile_commands.json>
                                   [--upstream-root R1] [--head-root R2] [--include-vikit]
exit code 0 = identical (modulo added include dirs), 1 = differences found.
"""
import argparse
import json
import os
import shlex
import sys


def norm_tokens(cmd, roots):
    toks = shlex.split(cmd)
    flags, incs, out = [], [], None
    i = 0
    while i < len(toks):
        t = toks[i]
        if t.startswith("-I"):
            incs.append(("-I", t[2:]))
        elif t == "-isystem":
            incs.append(("-isystem", toks[i + 1]))
            i += 1
        elif t in ("-o", "-c"):
            if t == "-o":
                out = toks[i + 1]
            i += 1
        else:
            flags.append(t)
        i += 1
    # path-bearing definitions (ROOT_DIR) are compared after root substitution
    def sub(s):
        for r in roots:
            s = s.replace(r, "<ROOT>")
        return s
    flags = [sub(f) for f in flags]
    incs = [(k, sub(v)) for k, v in incs]
    return flags, incs


def index(cc, roots):
    m = {}
    for e in cc:
        f = e["file"]
        key = f
        for r in roots:
            if f.startswith(r):
                key = os.path.relpath(f, r)
        m[key] = norm_tokens(e["command"], roots)
    return m


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("upstream")
    ap.add_argument("head")
    ap.add_argument("--upstream-root", default="/r/ws_upstream/src")
    ap.add_argument("--head-root", required=True, help="e.g. /r/wt/skel/ws/src")
    ap.add_argument("--include-vikit", action="store_true", help="also compare rpg_vikit TUs")
    a = ap.parse_args()
    up = index(json.load(open(a.upstream)), [a.upstream_root])
    hd = index(json.load(open(a.head)), [a.head_root])
    bad = 0
    for key in sorted(up):
        if not a.include_vikit and not key.startswith("FAST-LIVO2/"):
            continue
        if key not in hd:
            print(f"MISSING in head: {key}")
            bad += 1
            continue
        uf, ui = up[key]
        hf, hi = hd[key]
        if uf != hf:
            bad += 1
            print(f"FLAGS DIFFER: {key}")
            print("  only upstream:", [t for t in uf if t not in hf])
            print("  only head    :", [t for t in hf if t not in uf])
            if sorted(uf) == sorted(hf):
                print("  (same multiset, different order)")
        ui_s = [v for _, v in ui]
        hi_s = [v for _, v in hi]
        removed = [v for v in ui_s if v not in hi_s]
        added = [v for v in hi_s if v not in ui_s]
        # relative order of the common includes must be preserved
        common_u = [v for v in ui_s if v in hi_s]
        common_h = [v for v in hi_s if v in ui_s]
        if removed or common_u != common_h:
            bad += 1
            print(f"INCLUDES DIFFER (removed/reordered): {key}")
            print("  removed:", removed)
            print("  order upstream:", common_u)
            print("  order head    :", common_h)
        elif added:
            print(f"includes added (allowed): {key}: {added}")
    print("RESULT:", "IDENTICAL (modulo added include dirs)" if bad == 0 else f"{bad} DIFFERENCES")
    return 0 if bad == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
