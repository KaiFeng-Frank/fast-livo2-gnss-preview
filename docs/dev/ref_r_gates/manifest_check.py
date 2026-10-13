#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
# manifest_check.py — run_manifest.json checks for ManifestInfo::extra_json (AMENDMENTS A40; WP-f, ref-R gates).
#
#   manifest_check.py --run <run_dir>      one replay run: run_manifest.json is strict JSON with unique keys, carries
#                                          no "extra_json_error", and holds every replay_stats.json member with the
#                                          same value (except "complete"; "seed" is the frozen manifest field, the
#                                          configured trim "start_offset_s" is merged as "start_offset_trim_s", and the
#                                          frozen "start_offset_s" equals "offset_effective_s"). Prints counts only.
#   manifest_check.py --cases <root>       the writeManifest cases written by manifest_check.cpp (see there).
# Exit status 0 = all checks hold. Nothing here prints a trajectory value (A2).
import json
import math
import sys

FROZEN = ["git_describe", "git_dirty", "profile", "compile_defs", "eigen_version", "compiler", "image_digest",
          "config_sha256", "config_dump", "bag_path", "bag_sha256", "arm", "host", "seed", "start_offset_s",
          "t_start_wall", "t_end_wall"]


class DuplicateKey(ValueError):
    pass


def pairs_no_dup(pairs):
    seen = {}
    for k, v in pairs:
        if k in seen:
            raise DuplicateKey("duplicate key %r" % k)
        seen[k] = v
    return seen


def strict_constant(name):   # json accepts NaN/Infinity by default; RFC 8259 does not
    raise ValueError("non-standard JSON constant %s" % name)


def load_strict(path):
    with open(path, encoding="utf-8") as f:
        text = f.read()
    return text, json.loads(text, object_pairs_hook=pairs_no_dup, parse_constant=strict_constant)


def keys_in_order(text):
    return [k for k, _ in json.loads(text, object_pairs_hook=lambda p: p)]


def same(a, b):
    if isinstance(a, float) and isinstance(b, float):
        return a == b or (math.isnan(a) and math.isnan(b))
    return a == b


def check_run(run):
    problems = []
    try:
        _, stats = load_strict(run + "/replay_stats.json")
        mtext, man = load_strict(run + "/run_manifest.json")
    except (OSError, ValueError) as e:
        print("manifest_check: FAIL %s: %s" % (run, e))
        return 1
    order = keys_in_order(mtext)
    if order[:len(FROZEN)] != FROZEN:
        problems.append("frozen manifest fields out of order or missing")
    if "extra_json_error" in man:
        problems.append("extra_json not merged: %s" % man["extra_json_error"])
    merged = 0
    for k, v in stats.items():
        if k in ("complete", "seed"):
            continue
        mk = "start_offset_trim_s" if k == "start_offset_s" else k
        if mk not in man:
            problems.append("replay_stats member %s missing from the manifest" % k)
        elif not same(man[mk], v):
            problems.append("member %s differs between manifest and replay_stats.json" % k)
        else:
            merged += 1
    if man.get("seed") != stats.get("seed"):
        problems.append("seed differs")
    # frozen field = %.17g of the effective offset, replay statistic = the same double as %.9f (±0.5 ns)
    a, b = man.get("start_offset_s"), stats.get("offset_effective_s")
    if not (isinstance(a, (int, float)) and isinstance(b, (int, float)) and abs(a - b) <= 1e-9):
        problems.append("manifest start_offset_s != offset_effective_s")
    if "build_sha256=" + str(stats.get("build_sha256")) not in str(man.get("compile_defs")):
        problems.append("compile_defs does not carry the self-reported build_sha256")
    extra = [k for k in order if k not in FROZEN]
    print("manifest_check: %s: %d frozen + %d merged members, %s" % (run, len(FROZEN), len(extra),
          "OK" if not problems else "FAIL: " + "; ".join(problems)))
    return 0 if not problems else 1


# Expected outcome of each manifest_check.cpp case: "legacy" (no extra member), "merged" (members listed), "error".
CASES = {
    "empty": ("legacy", None),
    "blank": ("legacy", None),
    "empty_object": ("legacy", None),
    "merge": ("merged", ["alpha", "beta", "gamma", "delta"]),
    "nesting_64": ("merged", ["a"]),
    "collide_manifest": ("error", None),
    "collide_self": ("error", None),
    "trailing_text": ("error", None),
    "trailing_comma": ("error", None),
    "unterminated_string": ("error", None),
    "not_an_object": ("error", None),
    "raw_newline_in_string": ("error", None),
    "leading_zero": ("error", None),
    "bad_escape": ("error", None),
    "nan_literal": ("error", None),
    "nesting_65": ("error", None),
}


def legacy_text(man):
    # The pre-P2b writeManifest layout, rebuilt from the parsed values the case program set (all plain ASCII).
    out = "{\n"
    for i, k in enumerate(FROZEN):
        v = man[k]
        if isinstance(v, str):
            val = json.dumps(v, ensure_ascii=False)
        elif isinstance(v, bool) or v is None:
            raise ValueError("unexpected type for %s" % k)
        elif isinstance(v, int):
            val = str(v)
        else:
            val = "%.17g" % v
        out += '  "%s": %s%s\n' % (k, val, "," if i + 1 < len(FROZEN) else "")
    return out + "}\n"


def check_cases(root):
    bad = 0
    for name, (kind, members) in CASES.items():
        path = "%s/%s/run_manifest.json" % (root, name)
        try:
            text, man = load_strict(path)
        except (OSError, ValueError) as e:
            print("case %-22s FAIL (not strict JSON: %s)" % (name, e))
            bad += 1
            continue
        order = keys_in_order(text)
        extra = [k for k in order if k not in FROZEN]
        ok = order[:len(FROZEN)] == FROZEN
        if kind == "legacy":
            ok = ok and extra == [] and text == legacy_text(man)
        elif kind == "merged":
            ok = ok and extra == members
            src = open("%s/%s/extra.json" % (root, name), encoding="utf-8").read()
            want = json.loads(src)
            ok = ok and all(same(man[k], want[k]) for k in members)
            # verbatim: each input member (one per line in these cases) is a manifest line "  <member>[,]"
            lines = text.split("\n")
            for member in (m.strip() for m in src.strip()[1:-1].split(",\n")):
                ok = ok and ("  " + member in lines or "  " + member + "," in lines)
        else:
            ok = ok and extra == ["extra_json_error"] and man["extra_json_error"].startswith("extra_json not merged: ")
        print("case %-22s %-6s %s%s" % (name, kind, "OK" if ok else "FAIL",
                                        ("  (" + man["extra_json_error"] + ")") if "extra_json_error" in man else ""))
        bad += (not ok)
    print("RESULT: %d/%d cases OK" % (len(CASES) - bad, len(CASES)))
    return 0 if bad == 0 else 1


if __name__ == "__main__":
    if len(sys.argv) == 3 and sys.argv[1] == "--run":
        sys.exit(check_run(sys.argv[2]))
    if len(sys.argv) == 3 and sys.argv[1] == "--cases":
        sys.exit(check_cases(sys.argv[2]))
    print(__doc__ if __doc__ else "usage: manifest_check.py --run <dir> | --cases <root>", file=sys.stderr)
    sys.exit(2)
