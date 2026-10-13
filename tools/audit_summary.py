#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Compose the committed audit summary docs/dev/audit/<seq>.md from one remote archive directory (WP-g).

Inputs (all written by the tools themselves, nothing is recomputed here):
  <archive>/audit.md                          tools/m3dgr_audit.py (I2 + §12.6 self-checks)
  <archive>/gt/gt_audit.md                    tools/gt_audit.py (TEST_PLAN §4 pre-audit of the GT)
  <archive>/time_calib/<seq>_record_time.json tools/time_calib/gnss_time_calib.py without a replay (method 1 prior)
  <archive>/time_calib/<seq>.json             the same tool with a replay (method 2 / 3), when it ran
  --notes FILE                                hand-written provenance/blocked notes, placed right after the title block
Output: markdown on stdout (or --out). The GT report's headings are demoted one level so the summary has one title.
"""
import argparse
import json
import os
import sys


def read(path):
    with open(path) as f:
        return f.read()


def demote(md, drop_title=True):
    out = []
    for i, line in enumerate(md.split("\n")):
        if drop_title and i == 0 and line.startswith("# "):
            continue
        out.append("#" + line if line.startswith("#") else line)
    return "\n".join(out).strip("\n")


def ms(v, nd=1):
    return "—" if v is None else "%.*f" % (nd, 1e3 * v)


def method1_text(rep):
    m1 = rep["method1_record_time"]
    L = ["range_meas 记录时间 − GPST→UTC(leap %d s,主机钟视角 = 主机钟偏差 + GNSS 传输延迟):中位 %s ms、MAD %s ms、p5/p95 %s/%s ms、漂移 %.1f ppm、持续台阶(>50 ms)%s。" % (
        m1["leap_s"], ms(m1["median_s"]), ms(m1["mad_s"]), ms(m1["p5_s"]), ms(m1["p95_s"]), m1["drift_ppm"],
        "、".join("%+.0f ms @ %.1f s" % (1e3 * s[1], s[0]) for s in m1["steps"]) or "无")]
    hm = m1.get("header_minus_record_ms") or {}
    if hm:
        L.append("")
        L.append("头戳 − 记录时间(判定 LiDAR 时基):" + ";".join(
            "%s 中位 %.1f ms(p1 %.1f / p99 %.1f,漂移 %.1f ppm)" % (t, v["median_ms"], v["p1_ms"], v["p99_ms"], v["drift_ppm"])
            for t, v in sorted(hm.items())) + "。")
    pr = rep.get("method1_estimator_clock_prior")
    L.append("")
    if pr:
        L.append("估计器时钟(Livox 头戳)先验:GNSS「记录 − UTC」下包络 %s ms 减 Avia IMU「记录 − 头戳」下包络 %s ms ⇒ 上界 %s ms"
                 "(主机钟台阶在两者中抵消;漂移 %.1f ppm;估计器时钟台阶 %s);GNSS 传输延迟不可观,按 [0, %.0f] ms 均匀(推断)⇒ 先验 %s ms、σ %.0f ms。" % (
                     ms(pr["gnss_record_low_median_s"]), ms(pr["imu_record_minus_header_low_median_s"]), ms(pr["upper_bound_s"]),
                     pr["drift_ppm"], "、".join("%+.0f ms @ %.1f" % (1e3 * s[1], s[0]) for s in pr["steps"]) or "无",
                     1e3 * pr["latency_g_assumed_range_s"][1], ms(pr["offset_s"]), 1e3 * pr["sigma_s"]))
    else:
        L.append("估计器时钟先验:无(bag 缺 IMU 头戳)。")
    return "\n".join(L)


SEG_KEYS = ("t_from_gpst_s", "offset_s", "drift_ppm", "t_ref_gpst_s", "sigma_toff_s", "tdcp_paused", "method", "peak_corr",
            "window_scatter_ms", "n_windows", "n_windows_rejected", "n_windows_isolated", "bootstrap_sd_ms", "peak_width_sigma_ms",
            "dtw_offset_s", "dtw_consistent")


def fmt_cell(k, v):
    if v is None:
        return "—"
    if isinstance(v, bool):
        return "true" if v else "false"
    if k in ("offset_s", "sigma_toff_s", "dtw_offset_s"):
        return "%.2f ms" % (1e3 * v)
    if k in ("t_from_gpst_s", "t_ref_gpst_s"):
        return "%.3f" % v if v else "0"
    if isinstance(v, float):
        return "%.4g" % v
    return str(v)


def method2_text(rep):
    L = []
    inp = rep.get("method2_inputs", {})
    wins = rep.get("method2_windows") or []
    used = [w for w in wins if w.get("used")]
    L.append("输入:回放轨迹 `%s`(%s,%s 个速度样本,估计器速度 std %s m/s);多普勒 LS 速度 %s 个历元。窗口 %d 个,可用 %d 个(窗长/步长见 JSON)。状态:%s。" % (
        inp.get("trajectory"), inp.get("kind"), inp.get("traj_samples"),
        "%.2f" % inp["est_speed_std"] if inp.get("est_speed_std") is not None else "—", inp.get("gnss_epochs"),
        len(wins), len(used), rep.get("method2_status")))
    if inp.get("signal"):                             # tool_version 3
        dl = inp.get("doppler_ls") or {}
        L.append("")
        L.append("信号(tool_version 3):%s;W 的竖直方向:%s。多普勒 LS:C/N0 ≥ %s dB-Hz、仰角 ≥ %s°,按接收机 dopp_std 加权,"
                 "两遍(第二遍固定平滑后的钟漂):%s 个历元有 ≥ 4 行,第二遍解出 %s 个(每历元用行中位 %s)。" % (
                     inp["signal"], inp.get("up_source"), dl.get("cn0_min_dbhz", "—"), dl.get("el_min_deg", "—"),
                     dl.get("epochs_with_4_rows", "—"), dl.get("pass2_epochs", "—"), dl.get("rows_used_median", "—")))
    if wins and all("t_from" in w for w in wins):
        t0 = wins[0]["t_from"]
        L.append("")
        L.append("| 窗起点 [s] | 用 | 峰值相关 | 偏移 [ms] | 峰宽精度 [ms] | GNSS 覆盖 | 原因 |")
        L.append("|---|---|---|---|---|---|---|")
        for w in wins:
            L.append("| %.0f | %s | %s | %s | %s | %s | %s |" % (
                w["t_from"] - t0, "是" if w.get("used") else "否", "%.3f" % w["peak"] if w.get("peak") is not None else "—",
                ms(w.get("offset")), ms(w.get("sigma_pw")), "%.0f%%" % (100 * w["gnss_coverage"]) if "gnss_coverage" in w else "—",
                (w.get("reason") or ("峰在搜索边界" if w.get("at_edge") else "")).replace("|", "/")))
    segs = rep.get("method2_segments")
    if segs is None:                                  # reports of tool_version 1 carried only the chosen segments
        segs = rep.get("segments") or []
    if segs:
        L.append("")
        L.append("互相关给出的段(写入文件的以状态行为准):")
        L.append("")
        L.append("| " + " | ".join(SEG_KEYS) + " |")
        L.append("|" + "|".join(["---"] * len(SEG_KEYS)) + "|")
        for s in segs:
            L.append("| " + " | ".join(fmt_cell(k, s.get(k)) for k in SEG_KEYS) + " |")
    dtw = rep.get("method3_dtw")
    if dtw:
        L.append("")
        L.append("方法③ DTW 交叉核对:DTW 偏移 %s ms(路径偏移 MAD %s ms)对 xcorr %s ms,%s(>3σ 才告警)。" % (
            ms(dtw["offset_s"], 2), ms(dtw["mad_s"], 1), ms(dtw["xcorr_offset_s"], 2), "一致" if dtw["consistent"] else "**不一致**"))
    if rep.get("warnings"):
        L.append("")
        L.append("警告:" + ";".join(rep["warnings"]) + "。")
    return "\n".join(L)


def compose(archive, seq, notes="", host_root="/work/fastlivo2_gnss"):
    parts = []
    audit_md = os.path.join(archive, "audit.md")
    parts.append(read(audit_md).rstrip("\n") if os.path.exists(audit_md) else "# I2 数据审计 — %s\n\n(缺 audit.md)" % seq)
    if notes:
        # the notes go right after the title block (title + generation line + class note + verdict line)
        head, sep, rest = parts[0].partition("\n## ")
        parts[0] = head.rstrip("\n") + "\n\n## 本轮说明\n\n" + notes.strip("\n") + "\n\n## " + rest if sep else parts[0] + "\n\n## 本轮说明\n\n" + notes.strip("\n")
    gt_md = os.path.join(archive, "gt", "gt_audit.md")
    if os.path.exists(gt_md):
        parts.append("## GT 审计(tools/gt_audit.py)\n\n" + demote(read(gt_md)))
    tc = os.path.join(archive, "time_calib")
    m1 = os.path.join(tc, "%s_record_time.json" % seq)
    m2 = os.path.join(tc, "%s.json" % seq)
    sec = ["## 时间标定(tools/time_calib/gnss_time_calib.py,DESIGN §7)"]
    if os.path.exists(m1):
        r1 = json.loads(read(m1))
        sec += ["", "### 方法①(记录时间统计与估计器时钟先验)", "", method1_text(r1), "",
                "方法①文件 `%s_record_time.yaml`(不读回放;method record_time,σ 75 ms;tool_version ≥ 3 按 AMENDMENTS A53 TDCP 不暂停,"
                "tool_version ≤ 2 的文件仍写着暂停):" % seq,
                "", "```yaml", r1["yaml"].rstrip("\n") if r1.get("yaml") else "(未写出:%s)" % r1.get("status"), "```"]
    if os.path.exists(m2):
        r2 = json.loads(read(m2))
        sec += ["", "### 方法②(水平天线速度互相关)与方法③(DTW)", "", method2_text(r2), ""]
        if r2.get("yaml"):
            methods = sorted({sg.get("method") for sg in r2.get("segments") or []})
            what = ("标定文件(方法②)" if methods == ["xcorr"] else
                    "标定文件(方法②无可用段或不如方法①准,逐段取方法① method %s;A53:σ ≤ 0.1 s 即 TDCP 不暂停)" % "/".join(methods)
                    if "config" not in methods else "写出的文件(method config:两法都不可用,TDCP 暂停)")
            sec += ["%s `time_calib/%s.yaml`(AMENDMENTS A18 格式;所有 `*_gpst_s` 为 GTime::toSec(),1970 原点、GPST 尺度):" % (what, seq),
                    "", "```yaml", r2["yaml"].rstrip("\n"), "```"]
        else:
            sec += ["未写出 A18 文件:%s。" % r2.get("status")]
    parts.append("\n".join(sec))
    text = "\n\n".join(p.rstrip("\n") for p in parts) + "\n"
    # the tools ran in the container, where the project root is mounted at /r: show host paths
    return text.replace("`/r/", "`%s/" % host_root.rstrip("/")) if host_root else text


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("archive", help="remote archive directory audit/<seq>")
    p.add_argument("--seq", required=True)
    p.add_argument("--notes", default="", help="markdown file with hand-written notes (provenance, blocked items, commands)")
    p.add_argument("--out", default="")
    p.add_argument("--host-root", default="/work/fastlivo2_gnss", help="host path of the container's /r ('' keeps /r)")
    a = p.parse_args(argv)
    text = compose(a.archive, a.seq, read(a.notes) if a.notes else "", a.host_root)
    if a.out:
        with open(a.out, "w") as f:
            f.write(text)
    else:
        sys.stdout.write(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
