#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Main deliverable table (AMENDMENTS A46, A52, A54): sequence x {FAST-LIVO2 (paper), FAST-LIVO2 (ours), FAST-LIVGO
(paper), our tight coupling}, ATE RMSE in metres, plus the supplementary tables (A52): "30 s after activation" and E4.

    make_table.py --ate <eval dir>/ate.json [--ate ...] [--mode main|dev] [--out-md F] [--out-csv F] [--out-csv-supp F]

  * paper columns: FAST-LIVGO Table II (hard-coded below with its source; tests/test_make_table.py re-parses the
    verbatim row text of the paper, so a transcription error fails a test);
  * our columns: eval_ate.py cells — median [min, max] over the seeds (A46: five seeds, A30 semantics); a cell with
    fewer valid seeds than runs says n/N. Main table = E1, the whole sequence (A52: GNSS arms completed before
    ACTIVATE with the activation-time T_{N<-W}); U = HEAD with gnss.enable false (A55), T = the full tight coupling;
  * one column per sequence says the GT time base (A54: P4 chain, record-time corroboration, or none = 0, flagged)
    and how long the A54 mask removed inside the E1 span (A54: 主表注明各序列被掩掉的时长);
  * a second column of ours (AMENDMENTS A61 F13): the same arm's odometry output traj_N_odom (eval_ate.py e1o) — the
    registration's estimate at activation held, every drift correction since kept: FAST-LIVGO's frame, fixed after its
    initialisation, is the like-for-like; traj_N (the global output) keeps revising where the whole trajectory lies,
    which an SE(3)-aligned ATE counts as error;
  * supplementary tables: every arm present on the span from the latest activation + 30 s (U on the same span), and
    E4 = every arm's traj_W on the E1 span;
  * --mode main: the seven Table II sequences in the paper's order, GNSS-denial01/02 listed separately (ArUco
    start/end GT, E5 — never mixed into the ATE columns, A56). Without --aruco the separate list stays "待评测";
    --aruco aruco.json (eval_aruco.py) fills that table. --mode dev: the development sequences in the input
    (paper columns "—", labelled as a rehearsal, never an evaluation result);
  * footnotes: paper provenance (unaudited single runs), metric and E1 span, A55 + A27 + 986b4b1/border difference
    (A40), Outdoor01 gap (A3), GNSS-denial, GT time base and mask.
Exact values only in the tables (and the CSVs); no difference or ratio columns.
"""
import argparse
import csv
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import eval_aruco  # noqa: E402
import evalcommon as ec  # noqa: E402

# FAST-LIVGO: A Degeneracy-Robust LiDAR-Inertial-Visual-GNSS Fusion Odometry — Zhiyu Chen, Chunran Zheng, Jiayu Wen,
# et al., arXiv:2606.19190v2 [cs.RO], 24 Jun 2026. Table II "Quantitative Comparison of ATE RMSE (m) on the M3DGR
# Dataset Across Different Evaluated Algorithms"; header groups "Open Sky Scenarios" (Varying-illu03/04/05, Outdoor01,
# Outdoor04) and "GNSS Denied Scenarios" (Dark01, Dark02). Rows used here: "FAST-LIVO2" and "Ours" (= FAST-LIVGO).
# Transcribed from the arXiv HTML (local copy /workspace/fastlivo2_gnss_scout_20260924/livgo.txt).
LIVGO_SOURCE = "FAST-LIVGO (Chen et al., arXiv:2606.19190v2, 2026-06-24), Table II"
TABLE2_COLUMNS = ("Varying-illu03", "Varying-illu04", "Varying-illu05", "Outdoor01", "Outdoor04", "Dark01", "Dark02")
TABLE2_GROUP = {"Varying-illu03": "open_sky", "Varying-illu04": "open_sky", "Varying-illu05": "open_sky",
                "Outdoor01": "open_sky", "Outdoor04": "open_sky", "Dark01": "gnss_denied", "Dark02": "gnss_denied"}
TABLE2 = {
    "FAST-LIVO2": (1.44, 0.35, 0.16, 0.27, 0.25, 0.31, 0.18),
    "FAST-LIVGO": (0.23, 0.19, 0.09, 0.11, 0.13, 0.34, 0.27),     # the paper's row "Ours"
}
# verbatim text of the table in the source (header + the two rows), for the transcription test
TABLE2_VERBATIM = ("TABLE II: Quantitative Comparison of ATE RMSE (m) on the M3DGR Dataset Across Different Evaluated "
                   "Algorithms. Algorithm Open Sky Scenarios GNSS Denied Scenarios Varying-illu03 Varying-illu04 "
                   "Varying-illu05 Outdoor01 Outdoor04 Dark01 Dark02 RTKLIB 7.50 7.97 5.62 2.08 3.45 16.39 40.88 "
                   "LIO-SAM-GPS 1.93 1.01 4.83 0.45 0.58 1.85 3.62 FAST-LIVO2 1.44 0.35 0.16 0.27 0.25 0.31 0.18 "
                   "LIGO 0.98 0.26 0.28 0.33 0.36 2.45 4.12 Ours w/o Rejection 1.05 0.28 0.13 0.18 0.20 - - "
                   "Ours 0.23 0.19 0.09 0.11 0.13 0.34 0.27")
ARUCO_SEQUENCES = ("GNSS-denial01", "GNSS-denial02")
GROUP_LABEL = {"open_sky": "Open Sky", "gnss_denied": "GNSS Denied"}     # the paper's own header words (Table II)
TIME_BASE_LABEL = {"p4": "P4", "record_time": "记录时间旁证", "none": "未定,按 0ᶠ", "gpst": "GPST 同钟", "xcorr": "速度互相关"}

FOOTNOTES = [
    ("a", "论文值:%s,ATE RMSE(m)。未审计、单次运行,LiDAR/配置/对齐方式未说明(TEST_PLAN §4)。表 II 把 Dark01/02 列在"
          " \"GNSS Denied Scenarios\",M3DGR README 把它们标为夜间序列(非 GNSS 拒止)。" % LIVGO_SOURCE),
    ("b", "我们跑:每格 5 个种子(A30:种子 k = 记录序第 k 帧 LiDAR 开视窗,k = 0..4;A46),中位数 [最小, 最大],"
          "有效种子不足时注 n/N。指标 = `evo_ape tum <GT> <est> -a --t_max_diff 0.05`(SE(3) Umeyama 对齐、不含尺度,"
          "位置 ATE RMSE,与 M3DGR/FAST-LIVGO 一致),主表为整条序列(E1,A52):U 用 traj_W;GNSS 臂激活后用 traj_N,"
          "激活前的宿主位姿用激活时刻的 T_{N←W} 映射到 N 系补齐(等价于激活前就是纯 LIVO,只用激活时刻的一次估计),"
          "激活后无全局输出的步(REINIT 重新收集)用此前最近一次的 T_{N←W},从未激活的运行即其宿主轨迹;时段 = 真值与"
          "该序列全部运行(各臂各种子)的交集。确定性离线回放(eval profile),不是实时运行。"),
    ("c", "U = HEAD 同一二进制、gnss.enable: false(A55;G-零通道门已证逐位等于 R = 0d2c034 + A27 越界读修复,"
          "`VIOManager::updateState` 金字塔粗层越界读的边界保护;未保护的上游同一输入两次运行不逐位相同)。"
          "M3DGR/FAST-LIVGO 的 FAST-LIVO2 数字对应发布版 986b4b1(VIO border 48 px),本仓库 U 为 0d2c034(border 80 px)"
          "+ A27 保护(A40)。两列 FAST-LIVO2 并列只为看出两边设置是否对得上(A46)。"),
    ("d", "Outdoor01:本地副本在 t − t0 ≈ 262.5 s 处所有话题缺约 0.5 s(阿里云盘包损坏,tools/repair_bag.py 丢弃坏块后"
          " reindex;A3),与论文值的对比受此影响。"),
    ("e", "GNSS-denial01/02 不在表 II;真值是 ArUco 起终点(官方 ArUco_evaluate.py,TEST_PLAN E5),不是 ATE,另表报告(A56),"
          "不混入本表的 ATE 列。GNSS-denial01 来自 M3DGR 的另一个阿里云盘分享(README:399),2026-09-26 导入(A68)。"
          "另表数字来自 tools/eval/eval_aruco.py(--aruco;算法与官方脚本一致,W = traj_W,N = traj_N)。"),
    ("f", "真值(A54):时基取 tools/gt_audit.py 的 gt_timebase.yaml —— P4(预注册互相关链)通过用其值;P4 失败用记录时间"
          "旁证值;两者都没有按 0 s 并标「未定」。掩码 = gt_audit 可信掩码中 A54 规定不计入 ATE 的时段(RTK 跳变/毛刺等),"
          "真值与估计两侧都去掉;「掩掉」为 E1 时段内被掩掉的时长。「无掩码(旧审计)」= 审计早于 A54,只在 P4 通过时使用。"),
    ("g", "里程计输出(A61 F13):同一次运行的 traj_N_odom.tum(GnssFusion::globalPoseOdom)——配准在激活时刻的估计固定不动,"
          "之后的信息对「激活时刻天线位置与航向」的修订不进入输出,自激活以来的漂移修正全部保留;E1 构造同 b(eval_ate.py e1o)。"
          "FAST-LIVGO 初始化后固定 W→ENU(其式 2-3),与本列同口径;前一列 traj_N 是全局输出,持续修订整条轨迹的绝对位置,"
          "SE(3) 对齐的 ATE 把这一修订计为误差。早于 A61 的运行没有此文件,记「—」。"),
]


def load_cells(paths):
    cells, blocked, meta, seqs = {}, {}, [], {}
    for p in paths:
        r = ec.load_json(p)
        meta.append({"ate": p, "generated_utc": r.get("generated_utc"), "metric": r.get("metric", {}).get("cmd"),
                     "matrices": [(m.get("name"), m.get("source_id")) for m in r.get("matrices", [])]})
        for c in r.get("cells", []):
            k = (c["seq"], c["arm"])
            if k in cells:
                raise ValueError("cell %s/%s appears in two inputs" % k)
            cells[k] = c
        for seq, why in r.get("blocked", {}).items():
            blocked.setdefault(seq, why)
        for seq, s in r.get("sequences", {}).items():
            if seq in seqs and seqs[seq].get("gt", {}).get("gt_clean_sha256") != s.get("gt", {}).get("gt_clean_sha256"):
                raise ValueError("sequence %s was scored against two different GTs" % seq)
            seqs.setdefault(seq, s)
    return cells, blocked, meta, seqs


def sub_cell(c, name):
    """The post30 / e4 block of a cell, with the parent's flags (for fmt_cell)."""
    if c is None:
        return None
    s = dict(c.get(name) or {})
    s.setdefault("n_runs", c.get("n_runs"))
    for k in ("numpy_only", "identical_seed_groups", "n_distinct", "a55_flag"):
        s[k] = c.get(k)
    return s


def fmt_cell(c, digits, pending_text):
    if c is None:
        return pending_text
    if not c.get("n_ok"):
        return "无有效运行(0/%d)" % (c.get("n_runs") or 0)
    s = "%s [%s, %s]" % (ec.fmt_m(c["median_m"], digits), ec.fmt_m(c["min_m"], digits), ec.fmt_m(c["max_m"], digits))
    if c["n_ok"] < (c.get("n_runs") or c["n_ok"]):
        s += " (n=%d/%d)" % (c["n_ok"], c["n_runs"])
    if c.get("identical_seed_groups"):
        s += " ‡%d" % (c.get("n_distinct") or c["n_ok"])
    if c.get("numpy_only"):
        s += " *"
    if c.get("a55_flag"):
        s += " †"
    return s


def post30_text(c, seq_rec, digits):
    """A supplementary-table cell "30 s after activation": no GNSS run of the sequence activated => no such span."""
    if c is None:
        return "—"
    if seq_rec and not seq_rec.get("post30_spans"):
        return "无 GNSS 激活"
    p = c.get("post30") or {}
    if not p.get("n_ok") and c.get("gnss_arm") and p.get("n_never_activated") == c.get("n_runs"):
        return "未激活(0/%d)" % c.get("n_runs", 0)
    return fmt_cell(sub_cell(c, "post30"), digits, "—")


def odo_text(c, digits, pending_text):
    """Our odometry-output cell (A61 F13, eval_ate.py e1o): pending like the T cell; a cell scored before A61 has no e1o."""
    if c is None:
        return pending_text
    if not c.get("e1o"):
        return "—"
    return fmt_cell(sub_cell(c, "e1o"), digits, "—")


def gt_text(s):
    """Time base and masked seconds of a sequence record (eval_ate.py `sequences`)."""
    if not s:
        return "—"
    g, m = s.get("gt", {}), s.get("mask", {})
    tb = TIME_BASE_LABEL.get(g.get("method"), str(g.get("method")))
    if g.get("method") in ("p4", "record_time"):
        applied = float(g.get("time_shift_applied_s") or 0.0)
        est = g.get("estimate_s")
        tb += " %+.1f ms" % (1e3 * (float(est) if est is not None else applied))
        if est is not None and applied == 0.0 and float(est) != 0.0:
            tb += "(≤ 20 ms,不补偿)"
    if str(m.get("status", "")).startswith("applied"):
        masked = [x.get("masked_s", 0.0) for x in s.get("e1_spans", [])]
        mk = "掩掉 %s s" % "/".join("%.1f" % x for x in masked) if masked else "掩掉 —"
    else:
        mk = "无掩码(旧审计)"
    return "%s;%s" % (tb, mk)


def build_rows(mode, cells, blocked, seqs, u_arm, t_arm, digits):
    _, reg = ec.load_sequences()
    rows = []
    if mode == "main":
        order = list(TABLE2_COLUMNS)
    else:
        present = {s for s, _ in cells} | set(blocked) | set(seqs)
        order = [s for s in reg if reg[s]["class"] == "development" and s in present]
    for seq in order:
        paper = seq in TABLE2_COLUMNS
        i = TABLE2_COLUMNS.index(seq) if paper else None
        why = blocked.get(seq)
        if why and why.startswith(seq + ": "):
            why = why[len(seq) + 2:]
        pend_u = ("阻塞:%s" % why) if why else "待跑"
        pend_t = ("阻塞:%s" % why) if why else "待跑"
        notes = []
        if seq == "Outdoor01":
            notes.append("d")
        s = seqs.get(seq)
        rows.append({
            "seq": seq, "group": GROUP_LABEL.get(TABLE2_GROUP.get(seq, ""), "开发集" if mode == "dev" else ""),
            "livo2_paper": ec.fmt_m(TABLE2["FAST-LIVO2"][i], 2) if paper else "—", "livo2_paper_v": TABLE2["FAST-LIVO2"][i] if paper else None,
            "u": fmt_cell(cells.get((seq, u_arm)), digits, pend_u), "u_cell": cells.get((seq, u_arm)),
            "livgo_paper": ec.fmt_m(TABLE2["FAST-LIVGO"][i], 2) if paper else "—", "livgo_paper_v": TABLE2["FAST-LIVGO"][i] if paper else None,
            "t": fmt_cell(cells.get((seq, t_arm)), digits, pend_t), "t_cell": cells.get((seq, t_arm)),
            "t_odo": odo_text(cells.get((seq, t_arm)), digits, pend_t),
            "gt": gt_text(s), "seq_rec": s, "notes": notes})
    return rows


def arms_present(cells, seqs_in_table):
    order = list(ec.load_arms())
    present = {a for (s, a) in cells if s in seqs_in_table}
    return sorted(present, key=lambda a: order.index(a) if a in order else 99)


def load_aruco(paths):
    """aruco.json files from eval_aruco.py. None when the caller did not pass any (the separate table stays pending)."""
    if not paths:
        return None
    merged = {"cells": [], "blocked": {}, "sequences": {}, "sources": []}
    seen = set()
    for p in paths:
        r = ec.load_json(p)
        merged["sources"].append({"aruco": p, "generated_utc": r.get("generated_utc")})
        for c in r.get("cells") or []:
            k = (c["seq"], c["arm"], c["output"])
            if k in seen:
                raise ValueError("E5 cell %s/%s/%s appears in two inputs" % k)
            seen.add(k)
            merged["cells"].append(c)
        for seq, why in (r.get("blocked") or {}).items():
            merged["blocked"].setdefault(seq, why)
        for seq, s in (r.get("sequences") or {}).items():
            prev = merged["sequences"].get(seq)
            if prev and prev.get("sha256") != s.get("sha256"):
                raise ValueError("sequence %s was scored against two different ArUco GTs" % seq)
            merged["sequences"].setdefault(seq, s)
    return merged


def write_csv_aruco(path, aruco):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["seq", "arm", "output", "n_ok", "n_runs", "note", "translation_median_m", "translation_min_m",
                    "translation_max_m", "rotation_median", "rotation_min", "rotation_max", "rmse_median", "rmse_min",
                    "rmse_max", "tracking_median_pct", "tracking_min_pct", "tracking_max_pct"])
        for c in aruco.get("cells") or []:
            def g(block, name, cell=c):
                v = (cell.get(block) or {}).get(name)
                return "" if v is None else v
            w.writerow([c["seq"], c["arm"], c["output"], c["n_ok"], c["n_runs"], c.get("note") or "",
                        g("translation_m", "median"), g("translation_m", "min"), g("translation_m", "max"),
                        g("rotation_frobenius", "median"), g("rotation_frobenius", "min"), g("rotation_frobenius", "max"),
                        g("rmse", "median"), g("rmse", "min"), g("rmse", "max"),
                        g("tracking_rate_pct", "median"), g("tracking_rate_pct", "min"), g("tracking_rate_pct", "max")])


def render_md(mode, rows, cells, blocked, meta, u_arm, t_arm, digits, aruco=None):
    title = "# 主交付表:M3DGR ATE RMSE(m)" if mode == "main" else "# 开发集演练版(不是评测结果):ATE RMSE(m)"
    L = [title, ""]
    if mode == "dev":
        L += ["本表只证明评测流水线(取数 → 回放 → ATE → 出表)在开发集上端到端可用;开发集不在 FAST-LIVGO 表 II 中,论文列为 —。", ""]
    L += ["| 序列 | 论文分组ᵃ | FAST-LIVO2(论文)ᵃ | FAST-LIVO2(我们跑,%s)ᵇᶜ | FAST-LIVGO(论文)ᵃ | 我们的紧耦合(%s,traj_N)ᵇ "
          "| 我们的紧耦合·里程计输出(%s,traj_N_odom)ᵍ | 真值:时基;掩掉ᶠ |" % (u_arm, t_arm, t_arm),
          "|---|---|---|---|---|---|---|---|"]
    for r in rows:
        name = r["seq"] + ("".join("ᵈ" if n == "d" else n for n in r["notes"]))
        L.append("| %s | %s | %s | %s | %s | %s | %s | %s |" % (name, r["group"], r["livo2_paper"], r["u"], r["livgo_paper"], r["t"],
                                                           r["t_odo"], r["gt"]))
    if mode == "main":
        if aruco is None:
            L += ["", "GNSS-denial01/02(ArUco 起终点真值,另一指标,另表,见脚注 e):"]
            for s in ARUCO_SEQUENCES:
                why = blocked.get(s, "待评测(ArUco_evaluate.py)")
                L.append("- %s:%s" % (s, why[len(s) + 2:] if why.startswith(s + ": ") else why))
        else:
            L += ["", "## GNSS-denial 另表:E5 ArUco 起终点误差(A56)", ""]
            L.extend(eval_aruco.markdown_lines(aruco, digits))
            L.append("")
            for src in aruco.get("sources") or []:
                L.append("E5 来源:`%s`(%s)。" % (src["aruco"], src.get("generated_utc")))
    L += ["", "单元格:中位数 [最小, 最大];* = 只有 numpy 复算(evo 不可用时),不作最终数字;‡m = 有种子在时段内给出逐位"
               "相同的轨迹,m 个种子里只有这么多条不同轨迹(A30 的种子塌缩,见 ate.json 的 identical_seed_groups);"
               "† = A55/I11 违例:该序列 U 与 GNSS 臂不是同一二进制,或有种子的激活前宿主轨迹与同种子 U 不逐位相同"
               "(成对比较无效,见 ate.json)。", ""]
    # supplementary tables (A52)
    in_table = {r["seq"] for r in rows}
    arms = arms_present(cells, in_table)
    if arms:
        L += ["## 附表 1:激活 30 s 后(A52)", "",
              "时段 = [该序列各 GNSS 运行中最晚的激活 + 30 s, E1 时段末](各臂共有,U 也在此时段上评);从未激活的运行没有此值。", "",
              "| 序列 | 时段 s(掩掉 s) | " + " | ".join(arms) + " |", "|---|---|" + "---|" * len(arms)]
        for r in rows:
            s = r["seq_rec"] or {}
            sp = "; ".join("%.1f (%.1f)" % (x["length_s"], x["masked_s"]) for x in s.get("post30_spans", [])) or "—"
            L.append("| %s | %s | %s |" % (r["seq"], sp, " | ".join(post30_text(cells.get((r["seq"], a)), s, digits) for a in arms)))
        L += ["", "## 附表 2:E4 — 各臂宿主轨迹 traj_W(E1 时段)", "",
              "检验 GNSS 是否改动宿主(TEST_PLAN E4);U 的 E4 即其 E1。", "",
              "| 序列 | " + " | ".join(arms) + " |", "|---|" + "---|" * len(arms)]
        for r in rows:
            L.append("| %s | %s |" % (r["seq"], " | ".join(fmt_cell(sub_cell(cells.get((r["seq"], a)), "e4"), digits, "—") for a in arms)))
        L.append("")
    for key, text in FOOTNOTES:
        if mode == "dev" and key in ("d", "e"):
            continue
        L.append("%s. %s" % (key, text))
    L += ["", "来源:"]
    for m in meta:
        L.append("- `%s`(%s;%s;矩阵 %s)" % (m["ate"], m["generated_utc"], m["metric"],
                                         ", ".join("%s@%s" % (n, s) for n, s in m["matrices"])))
    return "\n".join(L) + "\n"


def write_csv(path, rows):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["seq", "group", "fast_livo2_paper", "u_median_m", "u_min_m", "u_max_m", "u_n_ok", "u_n_runs",
                    "fast_livgo_paper", "t_median_m", "t_min_m", "t_max_m", "t_n_ok", "t_n_runs", "gt_time_base",
                    "gt_estimate_s", "gt_shift_s", "gt_source", "mask_status", "masked_s_e1", "notes", "t_odo_median_m",
                    "t_odo_min_m", "t_odo_max_m", "t_odo_n_ok"])
        for r in rows:
            u, t = r["u_cell"] or {}, r["t_cell"] or {}
            o = t.get("e1o") or {}
            s = r["seq_rec"] or {}
            g, m = s.get("gt", {}), s.get("mask", {})
            w.writerow([r["seq"], r["group"], r["livo2_paper_v"] if r["livo2_paper_v"] is not None else "",
                        u.get("median_m", ""), u.get("min_m", ""), u.get("max_m", ""), u.get("n_ok", ""), u.get("n_runs", ""),
                        r["livgo_paper_v"] if r["livgo_paper_v"] is not None else "",
                        t.get("median_m", ""), t.get("min_m", ""), t.get("max_m", ""), t.get("n_ok", ""), t.get("n_runs", ""),
                        g.get("method", ""), "" if g.get("estimate_s") is None else g["estimate_s"], g.get("time_shift_applied_s", ""),
                        g.get("source", ""), m.get("status", ""),
                        " ".join("%.3f" % x["masked_s"] for x in s.get("e1_spans", [])), " ".join(r["notes"]),
                        o.get("median_m", ""), o.get("min_m", ""), o.get("max_m", ""), o.get("n_ok", "")])


def write_csv_supp(path, rows, cells):
    in_table = {r["seq"] for r in rows}
    arms = arms_present(cells, in_table)
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["seq", "metric", "arm", "median_m", "min_m", "max_m", "n_ok", "n_runs", "spans"])
        for r in rows:
            for name in ("post30", "e4", "e1o"):
                for a in arms:
                    c = sub_cell(cells.get((r["seq"], a)), name)
                    if c is None or (name == "e1o" and not cells[(r["seq"], a)].get("e1o")):
                        continue
                    w.writerow([r["seq"], name, a, c.get("median_m", ""), c.get("min_m", ""), c.get("max_m", ""), c.get("n_ok", ""),
                                c.get("n_runs", ""), " ".join("%.6f-%.6f" % tuple(sp) for sp in c.get("spans", []))])


def build_parser():
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0], formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--ate", action="append", default=[], help="ate.json of eval_ate.py (repeatable)")
    p.add_argument("--mode", choices=("main", "dev"), default="main")
    p.add_argument("--u-arm", default="U")
    p.add_argument("--t-arm", default="T")
    p.add_argument("--digits", type=int, default=2)
    p.add_argument("--out-md", default="")
    p.add_argument("--out-csv", default="")
    p.add_argument("--out-csv-supp", default="", help="supplementary tables (post30, e4, e1o) in long format")
    p.add_argument("--aruco", action="append", default=[], help="aruco.json of eval_aruco.py (E5, A56 separate table)")
    p.add_argument("--out-csv-aruco", default="", help="E5 separate table in long format (not mixed into the ATE csv)")
    return p


def main(argv=None):
    a = build_parser().parse_args(argv)
    cells, blocked, meta, seqs = load_cells(a.ate)
    aruco = load_aruco(a.aruco)
    rows = build_rows(a.mode, cells, blocked, seqs, a.u_arm, a.t_arm, a.digits)
    md = render_md(a.mode, rows, cells, blocked, meta, a.u_arm, a.t_arm, a.digits, aruco)
    if a.out_md:
        with open(a.out_md, "w") as f:
            f.write(md)
    else:
        sys.stdout.write(md)
    if a.out_csv:
        write_csv(a.out_csv, rows)
    if a.out_csv_supp:
        write_csv_supp(a.out_csv_supp, rows, cells)
    if a.out_csv_aruco:
        if aruco is None:
            raise ValueError("--out-csv-aruco needs --aruco")
        write_csv_aruco(a.out_csv_aruco, aruco)
    return 0


if __name__ == "__main__":
    sys.exit(main())
