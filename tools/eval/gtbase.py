# SPDX-License-Identifier: BSD-3-Clause
"""Evaluation ground truth (AMENDMENTS A54; P4-EVAL): time base and ATE mask from tools/gt_audit.py's outputs.

Contract with tools/gt_audit.py (WP-g; A54 implemented by P4-TIME: decide_timebase / write_timebase_yaml / trust_mask),
directory audit/<seq>/gt/:
  gt_clean.tum         cleaned GT (sorted, stamp-unique), GT stamps (the RTK receiver clock as recorded);
  gt_timebase.yaml     the A54 decision, flat `key: value` lines (strings JSON-quoted, null for missing numbers):
      sequence         must be the sequence being evaluated
      method           chain | record_time | none: chain = P4 passed (the pre-registered odometry chain); record_time =
                       P4 failed, the record-time value is used; none = neither (0 s, flagged in the table)
      applied_shift_s  seconds added to the gt_clean.tum stamps (the 20 ms rule applied; alias time_shift_applied_s)
      offset_s, uncertainty_s, confidence, flag, p4_verdict   carried into the report
      gt_file          optional; else gt_audited.tum when present: must equal gt_clean.tum + applied_shift_s (checked,
                       a stale file blocks); when absent the shift is applied here
      ate_mask_file / ate_mask_column   optional, default trust_mask.csv / ate_exclude
  trust_mask.csv       one row per gt_clean.tum row (same order): time column `t` (gt_clean stamps) and the ATE-mask
                       column, header `ate_exclude` optionally followed by an explanation in parentheses (P4-TIME writes
                       `ate_exclude(bits 8|32|64)`: gaps/stalls, no sensor data, RTK jumps/plateaus); 1 = the GT row lies
                       in a stretch excluded from the ATE (A54: 掩码内的时段不计入 ATE), 0 = scored.
Masked stretches = maximal runs of consecutive excluded GT rows, [t_first, t_last] on the evaluation clock. The ATE
drops the GT rows inside them and the estimate rows whose stamps fall inside them (so no pair is formed inside a masked
stretch from either side); the masked time inside each scoring span is reported (A54: 主表注明各序列被掩掉的时长),
counted as gt_audit counts its masked_s: every excluded GT row stands for min(dt to the next row, 0.5 s).
Audits written before A54 (no gt_timebase.yaml): gt_audit.json outputs.gt_audited when P4 passed (method "p4",
source "legacy", no mask — flagged in the table), otherwise blocked (re-run gt_audit).
"""
import csv
import json
import math
import os

import numpy as np

import evalcommon as ec

TIMEBASE_FILE = "gt_timebase.yaml"
MASK_FILE_DEFAULT = "trust_mask.csv"
MASK_COLUMN_DEFAULT = "ate_exclude"
METHODS = {"chain": "p4", "p4": "p4", "p4_chain": "p4", "record_time": "record_time", "record-time": "record_time",
           "none": "none",
           # A80 data (tools/datasets): "gpst" = the GT epochs and the sensor stamps are the same GPST instants by construction
           # (SmartPNT-MSF .ref files; shift must be 0), "xcorr" = shift measured by cross-correlating speeds (the receiver's Doppler
           # speed or the estimator's own, GT 5 Hz or faster; MARS-LVIG)
           "gpst": "gpst", "xcorr": "xcorr"}
SHIFT_KEYS = ("applied_shift_s", "time_shift_applied_s")
METHOD_LABEL = {"p4": "P4", "record_time": "记录时间旁证", "none": "未定(0)", "gpst": "GPST 同钟", "xcorr": "速度互相关"}


def _scalar(text):
    """One flat YAML value: a JSON-quoted string (as tools/gt_audit.py writes them), a single-quoted string, or a plain
    scalar with an optional trailing comment."""
    s = text.strip()
    if s.startswith('"'):
        return json.JSONDecoder().raw_decode(s)[0]
    if s.startswith("'"):
        end = s.find("'", 1)
        if end < 0:
            raise ValueError("unterminated quote in %r" % text)
        return s[1:end]
    s = s.split(" #", 1)[0].strip()
    low = s.lower()
    if low in ("true", "false"):
        return low == "true"
    if low in ("null", "~", ""):
        return None
    try:
        return int(s)
    except ValueError:
        pass
    try:
        return float(s)
    except ValueError:
        return s


def read_flat_yaml(path):
    """`key: value` lines (comment lines and blank lines ignored). PyYAML is used when present so a nested value is
    read correctly; the fallback refuses nested structures instead of guessing."""
    with open(path, encoding="utf-8") as f:
        text = f.read()
    try:
        import yaml
        d = yaml.safe_load(text) or {}
        if not isinstance(d, dict):
            raise ValueError("%s is not a mapping" % path)
        return d
    except ImportError:
        pass
    d = {}
    for i, ln in enumerate(text.splitlines(), 1):
        if not ln.strip() or ln.lstrip().startswith("#"):
            continue
        if ln[0].isspace() or ":" not in ln:
            raise ValueError("%s:%d: only flat `key: value` lines are supported without PyYAML" % (path, i))
        k, v = ln.split(":", 1)
        if not v.strip() or v.strip().startswith("#"):
            raise ValueError("%s:%d: key %r has no inline value (nested structure?)" % (path, i, k.strip()))
        d[k.strip()] = _scalar(v)
    return d


ROW_DT_CAP_S = 0.5            # a GT row stands for the time to the next row, at most 0.5 s (tools/gt_audit.py durations)


class Mask:
    """Masked stretches [a, b] (evaluation clock, closed, sorted, disjoint) that the ATE removes, and the excluded GT
    rows with the time each stands for (min(dt to the next row, 0.5 s), the accounting of gt_audit's masked_s) for the
    reported masked seconds."""

    def __init__(self, intervals=None, status="absent", rows_t=None, rows_s=None, **prov):
        self.iv = np.asarray(intervals if intervals is not None else [], dtype=float).reshape(-1, 2)
        self.rows_t = np.asarray(rows_t if rows_t is not None else [], dtype=float)
        self.rows_s = np.asarray(rows_s if rows_s is not None else [], dtype=float)
        self.status = status
        self.prov = prov

    def inside(self, t):
        """Boolean array: stamp t inside any masked stretch."""
        t = np.asarray(t, dtype=float)
        if self.iv.shape[0] == 0:
            return np.zeros(t.shape, dtype=bool)
        k = np.searchsorted(self.iv[:, 0], t, side="right") - 1
        ok = k >= 0
        out = np.zeros(t.shape, dtype=bool)
        out[ok] = t[ok] <= self.iv[k[ok], 1]
        return out

    def seconds_in(self, t0, t1):
        """Masked GT time inside [t0, t1]: the excluded rows there, each counted min(dt to the next row, 0.5 s)."""
        if self.rows_t.size == 0 or t1 <= t0:
            return 0.0
        m = (self.rows_t >= t0) & (self.rows_t <= t1)
        return float(np.sum(self.rows_s[m]))

    def stretches_in(self, t0, t1):
        return int(np.sum((self.iv[:, 1] >= t0) & (self.iv[:, 0] <= t1))) if self.iv.shape[0] else 0

    def record(self):
        return dict(self.prov, status=self.status, n_stretches=int(self.iv.shape[0]), total_s=float(np.sum(self.rows_s)),
                    stretches=[[float(a), float(b)] for a, b in self.iv[:50]])


def stretches_from_rows(t, excluded):
    """Maximal runs of consecutive excluded rows -> (k, 2) [t_first, t_last]."""
    ex = np.asarray(excluded, dtype=bool)
    if not ex.any():
        return np.zeros((0, 2))
    pos = np.where(ex)[0]
    breaks = np.where(np.diff(pos) > 1)[0]
    starts = pos[np.concatenate([[0], breaks + 1])]
    ends = pos[np.concatenate([breaks, [pos.size - 1]])]
    return np.stack([t[starts], t[ends]], axis=1)


def _check_equal_shifted(path, clean, shift, what):
    ga = ec.read_tum(path)
    if ga.shape != clean.shape or np.max(np.abs(ga[:, 0] - (clean[:, 0] + shift))) > 2e-6 or np.max(np.abs(ga[:, 1:] - clean[:, 1:])) > 0:
        raise ec.Blocked("%s is not gt_clean.tum shifted by %.6f s (%s; stale or edited)" % (os.path.basename(path), shift, what))


def read_mask_rows(path, column, clean_t):
    """trust_mask.csv -> boolean excluded flags aligned with gt_clean rows. The mask column is `column` or
    `column(<explanation>)`."""
    with open(path, newline="") as f:
        rd = csv.reader(f)
        hdr = [h.strip() for h in next(rd)]
        if "t" not in hdr:
            raise ec.Blocked("%s has no `t` column" % path)
        cols = [i for i, h in enumerate(hdr) if h == column or h.startswith(column + "(")]
        if len(cols) != 1:
            raise ec.Blocked("%s has %s `%s` column (A54 ATE mask; tools/gt_audit.py)" % (path, "no" if not cols else "more than one", column))
        it, ic = hdr.index("t"), cols[0]
        t, ex = [], []
        for row in rd:
            if not row:
                continue
            t.append(float(row[it]))
            v = row[ic].strip()
            if v not in ("0", "1"):
                raise ec.Blocked("%s: `%s` must be 0 or 1, got %r" % (path, column, v))
            ex.append(v == "1")
    t = np.asarray(t)
    if t.size != clean_t.size or np.max(np.abs(t - clean_t)) > 5e-6:
        raise ec.Blocked("%s rows do not match gt_clean.tum (%d vs %d rows)" % (path, t.size, clean_t.size))
    return np.asarray(ex, dtype=bool)


def load_gt(root, seq, info, override=None):
    """-> (gt on the evaluation clock (n, 8), provenance dict, Mask) or raise ec.Blocked."""
    if info["gt_kind"] != "rtk":
        raise ec.Blocked("%s: GT is %s (start/end markers), not a trajectory: metric = tools/eval/eval_aruco.py "
                         "(official M3DGR ArUco_evaluate.py, TEST_PLAN E5, A56 separate table), not an ATE"
                         % (seq, info["gt_kind"]))
    d = override or os.path.join(root, "audit", seq, "gt")
    clean_p = os.path.join(d, "gt_clean.tum")
    if not os.path.isfile(clean_p):
        raise ec.Blocked("%s: %s missing — run tools/gt_audit.py first (TEST_PLAN §4 pre-evaluation audit)" % (seq, clean_p))
    clean = ec.read_tum(clean_p)
    cs, n_re, n_dup = ec.sanitize_tum(clean)
    if n_re or n_dup:
        raise ec.Blocked("%s: gt_clean.tum is not sorted/stamp-unique (%d reordered, %d duplicate stamps)" % (seq, n_re, n_dup))
    tb_p = os.path.join(d, TIMEBASE_FILE)
    prov = {"dir": os.path.relpath(d, root), "gt_clean_sha256": ec.sha256_file(clean_p), "rows": int(clean.shape[0])}
    if os.path.isfile(tb_p):
        try:
            tb = read_flat_yaml(tb_p)
        except (ValueError, OSError) as e:
            raise ec.Blocked("%s: %s unreadable: %s" % (seq, TIMEBASE_FILE, e))
        if tb.get("sequence") not in (None, seq):
            raise ec.Blocked("%s: %s is for sequence %r" % (seq, TIMEBASE_FILE, tb.get("sequence")))
        raw_m = str(tb.get("method", "")).strip()
        method = METHODS.get(raw_m.lower())
        if method is None:
            raise ec.Blocked("%s: %s method %r is not one of chain | record_time | none | gpst | xcorr (A54, A80)" % (seq, TIMEBASE_FILE, raw_m))
        given = [(k, tb[k]) for k in SHIFT_KEYS if k in tb]
        if not given or any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(float(v)) for _, v in given):
            raise ec.Blocked("%s: %s needs a numeric applied_shift_s (A54)" % (seq, TIMEBASE_FILE))
        if len({float(v) for _, v in given}) > 1:
            raise ec.Blocked("%s: %s has two different shifts %s" % (seq, TIMEBASE_FILE, given))
        shift = float(given[0][1])
        if method in ("none", "gpst") and shift != 0.0:
            raise ec.Blocked("%s: %s method %s with a non-zero shift %.6f s" % (seq, TIMEBASE_FILE, raw_m, shift))
        gt_file = tb.get("gt_file") or ("gt_audited.tum" if os.path.isfile(os.path.join(d, "gt_audited.tum")) else None)
        if gt_file:
            _check_equal_shifted(os.path.join(d, str(gt_file)), clean, shift, TIMEBASE_FILE)
        prov.update(source=TIMEBASE_FILE, method=method, method_raw=raw_m, time_shift_applied_s=shift,
                    estimate_s=tb.get("offset_s"), uncertainty_s=tb.get("uncertainty_s", tb.get("sigma_s")),
                    confidence=tb.get("confidence"), flag=tb.get("flag"), p4_verdict=tb.get("p4_verdict"),
                    gt_file=gt_file, timebase_sha256=ec.sha256_file(tb_p))
        mfile = str(tb.get("ate_mask_file") or MASK_FILE_DEFAULT)
        mcol = str(tb.get("ate_mask_column") or MASK_COLUMN_DEFAULT)
        mp = os.path.join(d, mfile)
        if not os.path.isfile(mp):
            raise ec.Blocked("%s: A54 ATE mask %s missing" % (seq, mp))
        ex = read_mask_rows(mp, mcol, clean[:, 0])
        te = clean[:, 0] + shift
        dt = np.minimum(np.diff(te, append=te[-1]), ROW_DT_CAP_S)
        mask = Mask(stretches_from_rows(te, ex), status="applied", rows_t=te[ex], rows_s=dt[ex], file=mfile, column=mcol,
                    rows_excluded=int(ex.sum()), rows=int(ex.size), sha256=ec.sha256_file(mp))
    else:
        j = os.path.join(d, "gt_audit.json")
        if not os.path.isfile(j):
            raise ec.Blocked("%s: neither %s nor gt_audit.json in %s — run tools/gt_audit.py (A54)" % (seq, TIMEBASE_FILE, d))
        rep = ec.load_json(j)
        out = rep.get("outputs", {})
        if not out.get("gt_audited"):
            raise ec.Blocked("%s: audit predates A54 (no %s) and P4 failed (%s): re-run tools/gt_audit.py so that it writes "
                             "the A54 time base" % (seq, TIMEBASE_FILE, out.get("gt_audited_withheld", "gt_audited.tum withheld")))
        shift = float(out.get("time_shift_applied_s") or 0.0)
        _check_equal_shifted(os.path.join(d, out["gt_audited"]), clean, shift, "gt_audit.json")
        p4 = [c for c in rep.get("checks", []) if c.get("id") == "P4"]
        prov.update(source="legacy gt_audit.json (pre-A54: no time-base record, no ATE mask)", method="p4", method_raw="P4 " + (
            p4[0].get("verdict", "?") if p4 else "?"), time_shift_applied_s=shift, gt_file=out["gt_audited"],
            tool_version=rep.get("tool_version"))
        mask = Mask(status="absent (audit predates A54)")
    gt = cs.copy()
    gt[:, 0] += shift
    prov.update(method_label=METHOD_LABEL[prov["method"]], span=[float(gt[0, 0]), float(gt[-1, 0])])
    return gt, prov, mask
