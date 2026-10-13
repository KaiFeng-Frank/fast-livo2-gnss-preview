# SPDX-License-Identifier: BSD-3-Clause
"""Report helpers: JSON with numpy values, markdown tables, check records (id, verdict, evidence).

Verdicts used by every tool: PASS, FAIL, WARN (outside the expectation but not a gate), INFO (a measured fact
without a pass criterion), SKIP (cannot be evaluated; `reason` says why, e.g. "blocked: dev-set fixture").
"""
import json
import math

import numpy as np


def to_jsonable(x):
    if isinstance(x, dict):
        return {str(k): to_jsonable(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [to_jsonable(v) for v in x]
    if isinstance(x, np.ndarray):
        return to_jsonable(x.tolist())
    if isinstance(x, (np.integer,)):
        return int(x)
    if isinstance(x, (np.floating,)):
        x = float(x)
    if isinstance(x, float):
        if math.isnan(x) or math.isinf(x):
            return None
        return x
    if isinstance(x, (np.bool_,)):
        return bool(x)
    if isinstance(x, bytes):
        return x.decode("utf-8", errors="replace")
    return x


def write_json(path, obj):
    with open(path, "w") as f:
        json.dump(to_jsonable(obj), f, indent=1, sort_keys=False, ensure_ascii=False)
        f.write("\n")


def fmt(v, nd=3):
    if v is None:
        return "—"
    if isinstance(v, bool):
        return "yes" if v else "no"
    if isinstance(v, (int, np.integer)):
        return str(int(v))
    if isinstance(v, (float, np.floating)):
        v = float(v)
        if math.isnan(v):
            return "—"
        if v != 0.0 and (abs(v) >= 1e6 or abs(v) < 10 ** (-nd)):
            return "%.*e" % (max(1, nd - 1), v)
        return "%.*f" % (nd, v)
    return str(v)


def table(headers, rows):
    out = ["| " + " | ".join(headers) + " |", "|" + "|".join(["---"] * len(headers)) + "|"]
    for r in rows:
        out.append("| " + " | ".join(fmt(c) if not isinstance(c, str) else c.replace("|", "\\|") for c in r) + " |")
    return "\n".join(out)


class Checks:
    """Ordered list of checks with verdicts; `worst()` gives the report headline."""

    ORDER = {"FAIL": 0, "WARN": 1, "SKIP": 2, "INFO": 3, "PASS": 4}

    def __init__(self):
        self.items = []

    def add(self, cid, title, verdict, evidence, **data):
        assert verdict in self.ORDER, verdict
        self.items.append({"id": cid, "title": title, "verdict": verdict, "evidence": evidence, "data": data})
        return verdict

    def worst(self):
        if not self.items:
            return "INFO"
        return min((c["verdict"] for c in self.items), key=lambda v: self.ORDER[v])

    def counts(self):
        d = {}
        for c in self.items:
            d[c["verdict"]] = d.get(c["verdict"], 0) + 1
        return d

    def markdown(self):
        rows = [(c["id"], c["title"], "**%s**" % c["verdict"], c["evidence"]) for c in self.items]
        return table(("#", "check", "verdict", "evidence"), rows)
