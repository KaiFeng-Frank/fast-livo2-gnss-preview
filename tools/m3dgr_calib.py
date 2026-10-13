#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Derive the M3DGR sensor chain for the config (DESIGN §14): lever arm l^I from calibration.md, extrinsics, camera.

Antenna lever arm in the configured frame → IMU frame through extR/extT (Avia: T=[0.04165,0.02326,−0.0284], R=I);
Rcl/Pcl avia→camera; D435i PINHOLE intrinsics; MID-360 chain (≈[−0.284,0.001,−0.211], must be recomputed and
verified by §12.6-7). Writes config/m3dgr_*.yaml fragments. Owner: WP-g.

Convention of calibration.md (its §1 header): a row "source → target" with T, R means p_target = R·p_source + T.
FAST-LIVO2 convention: extrinsic_T/R = T_{I←L} (p^I = R p^L + T, LIVMapper.cpp:104-105), Rcl/Pcl = T_{C←L}.
Hence
    l^{I_avia}   = R_{avia_imu←avia} · p_ant^{avia} + T_{avia_imu←avia}            (A15: DESIGN §14 value)
    l^{I_mid360} = T_{mid360_imu←mid360} ∘ T_{mid360←avia} (p_ant^{avia})
with p_ant^{avia} = T of the row "GNSS antenna → avia" (the antenna *point* mapped into the avia frame).
The tool also checks every pair of rows that are each other's inverse and three closed chains
(camera→avia, avia→mid360, camera_imu→mid360) so that the direction convention is proven on the rotations it can
be proven on; the antenna row has no rotation and no inverse row, so its direction is argued physically (the
reverse reading would put the antenna 0.43 m *below* the front LiDAR, under the ground plane of README Fig. 1)
and is to be confirmed on data by §12.6-7 (development set, needs the ref-R replay: blocked this round).
The YAML output (argument out_yaml) is a fragment for WP-f's config/m3dgr_*.yaml; this tool never writes config/.

--gt-lever X Y Z (A76): the ground-truth antenna (C-RTK 2HP) is a different antenna from the 9Ps one the filter uses, and
its lever arm is not in calibration.md. A58 fitted its horizontal components on the development set in the AVIA IMU frame
(l_z held at the 9Ps value; A63/A76 lever-ref -0.176 -0.016 0.401). This option carries that rigid-body point to the
selected LiDAR's IMU frame through avia_imu -> avia -> <lidar> -> <lidar>_imu (point transform p' = R p + T per row, the
inverse of the avia -> avia_imu row first) and checks it three ways (round trip, distances to the 9Ps antenna preserved,
the same point through the camera_imu and camera rows). The point keeps the provenance of its input: the x, y values are
a development-set fit, not a calibration; only their re-expression in the other frame is calibrated.
A77: for the MID-360 host the repository no longer uses this chain value (its avia -> mid360 TRANSLATION disagrees with the
data by 0.15-0.17 m along the heading, A76 record section six); the MID-360 GT lever and 9Ps lever of tools/eval/lidars.json
and config/m3dgr_mid360.yaml come from tools/calib/host_gt_lever.py, which keeps this chain's ROTATION and vertical only.
"""
import argparse
import datetime
import math
import re
import sys

import numpy as np

DESIGN_L_AVIA = np.array([-0.31635, 0.10926, 0.4006])          # DESIGN §14 / A15
DESIGN_L_MID360_APPROX = np.array([-0.284, 0.001, -0.211])      # DESIGN §14 "推断"


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0], formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("calibration_md", help="M3DGR calibration.md")
    p.add_argument("out_yaml", help="output YAML fragment")
    p.add_argument("--lidar", choices=["avia", "mid360"], default="avia")
    p.add_argument("--md", default="", help="also write the derivation as markdown (e.g. docs/dev/audit/m3dgr_calib.md)")
    p.add_argument("--source-label", default="", help="how the markdown cites the input (default: its path)")
    p.add_argument("--gt-lever", type=float, nargs=3, default=None, metavar=("X", "Y", "Z"),
                   help="GT antenna (C-RTK 2HP) lever arm in the AVIA IMU frame (A58/A63 lever-ref: -0.176 -0.016 0.401); "
                        "adds its image in the selected LiDAR's IMU frame, with self-checks (A76)")
    return p


_ROW = re.compile(r"^\|\s*(?P<src>[^|]+?)\s*(?:→|->)\s*(?P<dst>[^|]+?)\s*\|\s*(?P<T>[^|]+?)\s*\|\s*(?P<R>[^|]*?)\s*\|\s*$")


def _clean(name):
    return name.replace("`", "").strip()


def _floats(text):
    return [float(x) for x in re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", text)]


def parse_calibration(text):
    """-> dict {(src, dst): (R 3x3, T 3)}; rows with R '—' get R = None (translation-only point offset)."""
    out = {}
    for line in text.splitlines():
        m = _ROW.match(line.strip())
        if not m:
            continue
        src, dst = _clean(m.group("src")), _clean(m.group("dst"))
        if src.lower() == "direction":
            continue
        T = _floats(m.group("T"))
        if len(T) != 3:
            continue
        rtext = m.group("R").replace("`", "").strip()
        if rtext.lower().startswith("identity"):
            R = np.eye(3)
        elif rtext in ("—", "-", "", "–"):
            R = None
        else:
            v = _floats(rtext)
            if len(v) != 9:
                continue
            R = np.array(v).reshape(3, 3)
        out[(src, dst)] = (R, np.array(T))
    intr = {}
    for key in ("fx", "fy", "cx", "cy", "k1", "k2", "p1", "p2", "image_width", "image_height"):
        m = re.search(r"^\s*%s:\s*([-+0-9.eE]+)" % key, text, re.M)
        if m:
            intr[key] = float(m.group(1))
    m = re.search(r"model_type:\s*(\S+)", text)
    if m:
        intr["model"] = m.group(1)
    imu = {}
    for block, name in (("avia_livox", "avia"), ("mid360_livox", "mid360")):
        bm = re.search(r"name:\s*%s(.*?)(?:```)" % block, text, re.S)
        if bm:
            b = bm.group(1)
            avg = {}
            for k in ("gyr_n", "gyr_w", "acc_n", "acc_w"):
                mm = re.search(r"avg-axis:.*?%s:\s*([-+0-9.eE]+)" % k, b, re.S)
                if mm:
                    avg[k] = float(mm.group(1))
            imu[name] = avg
    return out, intr, imu


def inv(R, T):
    return R.T, -R.T @ T


def compose(a, b):
    """a ∘ b for rows p_mid = R_b p_src + T_b, p_dst = R_a p_mid + T_a."""
    Ra, Ta = a
    Rb, Tb = b
    return Ra @ Rb, Ra @ Tb + Ta


def rot_angle_deg(R):
    c = (np.trace(R) - 1.0) / 2.0
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def orthonormality(R):
    return float(np.max(np.abs(R @ R.T - np.eye(3)))), float(np.linalg.det(R))


def derive(rows, gt_lever_avia_imu=None):
    res = {"checks": [], "lever": {}}
    get = lambda s, d: rows.get((s, d))
    # 1) rotation matrices are rotations
    for (s, d), (R, T) in rows.items():
        if R is not None:
            e, det = orthonormality(R)
            res["checks"].append(("orthonormal %s→%s" % (s, d), e < 5e-4 and abs(det - 1) < 5e-4, "max abs(RRᵀ−I) %.1e, det %.6f" % (e, det)))
    # 2) listed inverse pairs
    for (s, d), (R, T) in rows.items():
        if R is None or (d, s) not in rows or s > d:
            continue
        R2, T2 = rows[(d, s)]
        Ri, Ti = inv(R, T)
        dR = rot_angle_deg(Ri.T @ R2)
        dT = float(np.linalg.norm(Ti - T2))
        res["checks"].append(("inverse pair %s↔%s" % (s, d), dR < 0.05 and dT < 1e-3, "Δrot %.4f°, ΔT %.2e m" % (dR, dT)))
    # 3) closed chains
    chains = (("camera", "camera_imu", "avia"), ("avia", "camera", "mid360"), ("camera_imu", "camera", "mid360"),
              ("camera", "camera_imu", "mid360"))
    for a, b, c in chains:
        ab, bc, ac = get(a, b), get(b, c), get(a, c)
        if ab is None and get(b, a) is not None:
            ab = inv(*get(b, a))
        if bc is None and get(c, b) is not None:
            bc = inv(*get(c, b))
        if ac is None and get(c, a) is not None:
            ac = inv(*get(c, a))
        if not (ab and bc and ac) or ab[0] is None or bc[0] is None or ac[0] is None:
            continue
        R, T = compose(bc, ab)
        dR = rot_angle_deg(R.T @ ac[0])
        dT = float(np.linalg.norm(T - ac[1]))
        res["checks"].append(("chain %s→%s→%s = %s→%s" % (a, b, c, a, c), dR < 0.5 and dT < 0.01, "Δrot %.3f°, ΔT %.4f m" % (dR, dT)))
    # 4) lever arms
    ant = get("GNSS antenna", "avia")
    if ant is None:
        for (s, d), v in rows.items():
            if "antenna" in s.lower() and d == "avia":
                ant = v
    if ant is None:
        raise ValueError("calibration.md has no 'GNSS antenna → avia' row")
    p_ant = ant[1]
    R_ai, T_ai = get("avia", "avia_imu")
    l_avia = R_ai @ p_ant + T_ai
    res["lever"]["p_ant_avia"] = p_ant
    res["lever"]["l_avia_imu"] = l_avia
    res["lever"]["l_avia_imu_reverse_hypothesis"] = R_ai @ (-p_ant) + T_ai
    am = get("avia", "mid360")
    mm = get("mid360", "mid360_imu")
    if am and mm:
        p_mid = am[0] @ p_ant + am[1]
        res["lever"]["p_ant_mid360"] = p_mid
        res["lever"]["l_mid360_imu"] = mm[0] @ p_mid + mm[1]
        # MID-360 origin in the avia frame: antenna height relative to it (plausibility)
        Rm, Tm = inv(*am)
        res["lever"]["mid360_origin_in_avia"] = Tm
        res["R_mid360_avia"] = am[0]
    res["checks"].append(("A15: l^I (Avia) = DESIGN §14 [-0.31635, 0.10926, 0.4006]",
                          float(np.max(np.abs(l_avia - DESIGN_L_AVIA))) < 1e-9,
                          "derived %s, max |Δ| %.1e m" % (np.array2string(l_avia, precision=6), float(np.max(np.abs(l_avia - DESIGN_L_AVIA))))))
    if "l_mid360_imu" in res["lever"]:
        lm = res["lever"]["l_mid360_imu"]
        res["checks"].append(("MID-360 chain vs DESIGN §14 estimate [-0.284, 0.001, -0.211] (3 decimals)",
                              float(np.max(np.abs(lm - DESIGN_L_MID360_APPROX))) < 1e-3,
                              "derived %s" % np.array2string(lm, precision=6)))
    res["checks"].append(("antenna above the front LiDAR (plausibility of the row direction, README Fig. 1: antennas on the top plate)",
                          p_ant[2] > 0.2, "antenna z in avia frame %+.3f m (reverse reading would be %+.3f m)" % (p_ant[2], -p_ant[2])))
    res["extrinsic_avia"] = get("avia", "avia_imu")
    res["rcl_pcl_avia"] = get("avia", "camera")
    res["extrinsic_mid360"] = get("mid360", "mid360_imu")
    if gt_lever_avia_imu is not None:
        _gt_lever(res, rows, np.asarray(gt_lever_avia_imu, dtype=float), p_ant)
    return res


def _chain_point(rows, steps, p):
    """Apply the point transforms p' = R p + T of `steps` (list of (src, dst, invert)) in order."""
    for src, dst, invert in steps:
        R, T = rows[(src, dst)]
        p = (R.T @ (p - T)) if invert else (R @ p + T)
    return p


def _gt_lever(res, rows, l_gt_avia_imu, p_ant_avia):
    """GT antenna point (Avia IMU frame) -> mid360 IMU frame, with self-checks (see the module docstring)."""
    out = res["lever"]
    out["gt_avia_imu"] = l_gt_avia_imu
    to_avia = [("avia", "avia_imu", True)]
    p_avia = _chain_point(rows, to_avia, l_gt_avia_imu)
    out["gt_avia"] = p_avia
    if ("avia", "mid360") not in rows or ("mid360", "mid360_imu") not in rows:
        return
    fwd = to_avia + [("avia", "mid360", False), ("mid360", "mid360_imu", False)]
    l_mid = _chain_point(rows, fwd, l_gt_avia_imu)
    out["gt_mid360_imu"] = l_mid
    out["gt_mid360"] = _chain_point(rows, to_avia + [("avia", "mid360", False)], l_gt_avia_imu)
    chk = res["checks"]
    # (1) round trip through the transposed rotations (a row that is not a rotation breaks it)
    back = _chain_point(rows, [("mid360", "mid360_imu", True), ("avia", "mid360", True), ("avia", "avia_imu", False)], l_mid)
    e_rt = float(np.max(np.abs(back - l_gt_avia_imu)))
    chk.append(("GT lever: round trip avia_imu → mid360_imu → avia_imu", e_rt < 5e-6,
                "max |Δ| %.1e m (the calibration.md rotations carry 6 digits)" % e_rt))
    # (2) rigid motion: the vector between the 9Ps and the GT antenna keeps its length
    l9_avia = out["l_avia_imu"]
    l9_mid = out["l_mid360_imu"]
    d_avia, d_mid = float(np.linalg.norm(l_gt_avia_imu - l9_avia)), float(np.linalg.norm(l_mid - l9_mid))
    chk.append(("GT lever: |GT antenna − 9Ps antenna| unchanged by the frame change", abs(d_avia - d_mid) < 5e-6,
                "Avia IMU frame %.7f m, mid360 IMU frame %.7f m" % (d_avia, d_mid)))
    # (3) the same point through the camera_imu rows and through the camera rows (other calibration rows)
    for via in ("camera_imu", "camera"):
        keys = [("avia", via), (via, "mid360")]
        if all(k in rows or (k[1], k[0]) in rows for k in keys):
            steps = to_avia
            for src, dst in keys:
                if (src, dst) in rows:
                    steps = steps + [(src, dst, False)]
                else:
                    steps = steps + [(dst, src, True)]
            alt = _chain_point(rows, steps + [("mid360", "mid360_imu", False)], l_gt_avia_imu)
            e = float(np.max(np.abs(alt - l_mid)))
            chk.append(("GT lever: same point via avia → %s → mid360 rows" % via, e < 5e-5,
                        "max |Δ| %.1e m vs the direct avia → mid360 row" % e))
    # (4) the 9Ps antenna through this very function reproduces the lever the config uses
    l9_chain = _chain_point(rows, [("avia", "avia_imu", True)] + fwd[1:], l9_avia)
    e9 = float(np.max(np.abs(l9_chain - l9_mid)))
    chk.append(("GT lever: the 9Ps antenna through the same chain = l^I(mid360) of the config", e9 < 1e-9, "max |Δ| %.1e m" % e9))
    # rotation Avia IMU <- mid360 IMU (free vectors), for the vehicle-frame view of the lever
    R_am, _ = rows[("avia", "mid360")]
    out["R_avia_imu_from_mid360_imu"] = R_am.T
    out["gt_lever_in_avia_axes"] = R_am.T @ l_mid          # vector mid360 IMU origin -> GT antenna, Avia-IMU axes
    out["mid360_imu_origin_in_avia_imu"] = l_gt_avia_imu - R_am.T @ l_mid


def _v(a, nd=10):
    """Compact list text with nd significant digits (enough to carry calibration.md's values unchanged)."""
    return "[" + ", ".join("%.*g" % (nd, float(x)) for x in np.asarray(a).ravel()) + "]"


def yaml_fragment(res, intr, imu, lidar):
    L = ["# generated by tools/m3dgr_calib.py from M3DGR calibration.md (%s); fragment for config/m3dgr_*.yaml (WP-f)" %
         datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%d"),
         "# convention: calibration.md 'src -> dst' rows mean p_dst = R p_src + T; FAST-LIVO2 extrinsic_T/R = T_{I<-L}, Rcl/Pcl = T_{C<-L}"]
    if lidar == "avia":
        R, T = res["extrinsic_avia"]
        L += ["extrin_calib:", "  extrinsic_T: %s" % _v(T), "  extrinsic_R: %s" % _v(R)]
        Rc, Pc = res["rcl_pcl_avia"]
        L += ["  Rcl: %s" % _v(Rc, 7), "  Pcl: %s" % _v(Pc, 7)]
        L += ["gnss:", "  frames:", "    lever_arm_frame: lidar",
              "    lever_arm_xyz: %s   # antenna in the avia frame; l^I = %s (A15)" % (_v(res["lever"]["p_ant_avia"]), _v(res["lever"]["l_avia_imu"]))]
        n = imu.get("avia", {})
    else:
        R, T = res["extrinsic_mid360"]
        L += ["extrin_calib:", "  extrinsic_T: %s" % _v(T), "  extrinsic_R: %s" % _v(R)]
        L += ["gnss:", "  frames:", "    lever_arm_frame: lidar",
              "    lever_arm_xyz: %s   # antenna in the mid360 frame; l^I = %s" % (_v(res["lever"]["p_ant_mid360"]), _v(res["lever"]["l_mid360_imu"]))]
        n = imu.get("mid360", {})
    if n:
        L += ["  imu_bridge:", "    acc_n: %.6g" % n.get("acc_n", float("nan")), "    gyr_n: %.6g" % n.get("gyr_n", float("nan"))]
    gl = res["lever"]
    if "gt_avia_imu" in gl:
        # not a config key: the evaluation lever-ref (tools/eval/hosts.json), in the IMU frame the trajectory is in
        key = "gt_mid360_imu" if (lidar == "mid360" and "gt_mid360_imu" in gl) else "gt_avia_imu"
        L += ["# GT antenna (C-RTK 2HP) in the %s IMU frame (evaluation lever-ref, not a config key; A76): %s" % (
            "mid360" if key == "gt_mid360_imu" else "avia", _v(gl[key], 10))]
    if intr:
        L += ["camera:   # D435i color, %s %dx%d" % (intr.get("model", "?"), int(intr.get("image_width", 0)), int(intr.get("image_height", 0))),
              "  cam_fx: %.8g" % intr.get("fx", float("nan")), "  cam_fy: %.8g" % intr.get("fy", float("nan")),
              "  cam_cx: %.8g" % intr.get("cx", float("nan")), "  cam_cy: %.8g" % intr.get("cy", float("nan")),
              "  cam_d0: %g" % intr.get("k1", 0.0), "  cam_d1: %g" % intr.get("k2", 0.0), "  cam_d2: %g" % intr.get("p1", 0.0), "  cam_d3: %g" % intr.get("p2", 0.0)]
    return "\n".join(L) + "\n"


def markdown(res, src_path, intr, imu):
    L = ["# M3DGR 标定链推导(tools/m3dgr_calib.py)", "",
         "来源:`%s`(M3DGR calibration.md)。生成日期 %s。" % (src_path, datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%d")), "",
         "## 约定", "",
         "- calibration.md 每行「source → target」给 T、R,含义 `p_target = R·p_source + T`(文件 §1 开头声明)。",
         "- FAST-LIVO2:`extrinsic_T/R` = T_{I←L}(`p^I = R p^L + T`,LIVMapper.cpp:104-105),`Rcl/Pcl` = T_{C←L}(vio.cpp:35-39)。",
         "- 杆臂 `l^I` = 天线相位中心在 IMU 系的坐标(DESIGN §1 A 行)。", "",
         "## 推导", "",
         "1. 「GNSS antenna → avia」行只有 T 没有 R:把天线这个**点**(source 原点)映到 avia 系,`p_ant^avia = T = %s` m。" % _v(res["lever"]["p_ant_avia"], 4),
         "2. 「avia → avia_imu」:R = I,T = %s ⇒ `l^I(Avia) = p_ant^avia + T = %s` m,与 DESIGN §14 / A15 的 `[-0.31635, 0.10926, 0.4006]` **逐位相同**(差 %.1e m)。" % (
             _v(res["extrinsic_avia"][1], 5), _v(res["lever"]["l_avia_imu"], 5), float(np.max(np.abs(res["lever"]["l_avia_imu"] - DESIGN_L_AVIA))))]
    if "l_mid360_imu" in res["lever"]:
        L.append("3. MID-360 链:`p_ant^mid360 = R_{mid360←avia}·p_ant^avia + T_{mid360←avia} = %s`,再经「mid360 → mid360_imu」(R = I,T = %s)得 `l^I(MID-360) = %s` m(DESIGN §14 推断 ≈[−0.284, 0.001, −0.211],3 位小数内一致)。" % (
            _v(res["lever"]["p_ant_mid360"], 5), _v(res["extrinsic_mid360"][1], 5), _v(res["lever"]["l_mid360_imu"], 5)))
        L.append("4. 旁证:MID-360 原点在 avia 系 `%s` m(比 Avia 高约 %.2f m),天线比 Avia 高 %.3f m;MID-360 相对 avia 转角约 %.1f°,所以 MID-360 系里天线 z 为负不代表在下方。" % (
            _v(res["lever"]["mid360_origin_in_avia"], 3), res["lever"]["mid360_origin_in_avia"][2], res["lever"]["p_ant_avia"][2],
            rot_angle_deg(res["R_mid360_avia"])))
    L += ["", "## 方向论证", "",
          "- 反向读法(把 T 当作 avia 原点在天线系)会给 `p_ant^avia = %s`,即天线在前视雷达**下方 0.43 m**。README 图 1:前视雷达(1)装在底板上、离地约 0.3 m,三根天线(5)在顶板上方 15 cm,顶板比底板高 45 cm ⇒ 正向读法(天线高 0.43 m)与结构图一致,反向读法会把天线放到地面以下,排除。" % _v(-res["lever"]["p_ant_avia"], 3),
          "- 横向 +0.086 m(avia 系 y 向左):README 图 1(b) 顶板三根天线中两根在一侧(C-RTK2HP 双天线)、一根在另一侧(推断为 9Ps);用 y>0 的那根与 0.086 m 对应。**RTK 真值用的 2HP 天线不是这一根**,其杆臂未知(TEST_PLAN §4「2HP 杆臂未知记为地板」)。",
          "- 数据证据(§12.6-7:转弯段 l 与 −l 两假设的多普勒残差 RMS 差 ≈ ω·|l_h|)需要 ref-R 回放轨迹,且属于开发集上的判定(A2):本轮 **SKIP(blocked: ref-R merge + dev-set fixture)**。", "",
          "## 自洽检查", "",
          "| 检查 | 结果 | 数值 |", "|---|---|---|"]
    for name, ok, ev in res["checks"]:
        L.append("| %s | %s | %s |" % (name, "PASS" if ok else "**FAIL**", ev))
    gl = res["lever"]
    if "gt_mid360_imu" in gl:
        L += ["", "## 真值天线(C-RTK 2HP)杆臂的换系(A76)", "",
              "- 输入:Avia IMU 系 `%s`(A58 在开发集上拟合 x、y,z 取 9Ps 天线的 0.4006;A63/A76 的 lever-ref)。**这是拟合量不是标定量**;换系本身由标定行给出。" % _v(gl["gt_avia_imu"], 6),
              "- 链:avia_imu →(avia→avia_imu 的逆)avia →(avia→mid360)mid360 →(mid360→mid360_imu)mid360_imu,点变换 `p' = R p + T`。",
              "- Avia 系 `%s`;mid360 系 `%s`;mid360 IMU 系 `%s` m。" % (_v(gl["gt_avia"], 8), _v(gl["gt_mid360"], 10), _v(gl["gt_mid360_imu"], 10)),
              "- mid360 IMU 原点在 Avia IMU 系 `%s`;从它指向真值天线的向量在 Avia IMU 轴上为 `%s`(其水平范数 %.4f m,Avia IMU 点为 %.4f m)。" % (
                  _v(gl["mid360_imu_origin_in_avia_imu"], 8), _v(gl["gt_lever_in_avia_axes"], 8),
                  float(np.hypot(*gl["gt_lever_in_avia_axes"][:2])), float(np.hypot(*gl["gt_avia_imu"][:2]))), ""]
    L += ["", "## 其它输出", "",
          "- Avia:`extrinsic_T = %s`、`extrinsic_R = I`(= calibration §1.6,与上游 avia.yaml 相同);`Rcl/Pcl` 取「avia → camera」行:Pcl = %s。" % (
              _v(res["extrinsic_avia"][1], 5), _v(res["rcl_pcl_avia"][1], 6)),
          "- D435i:%s %dx%d,fx %.5f fy %.5f cx %.5f cy %.5f,畸变全 0。" % (intr.get("model", "?"), int(intr.get("image_width", 0)), int(intr.get("image_height", 0)),
                                                                     intr.get("fx", float("nan")), intr.get("fy", float("nan")), intr.get("cx", float("nan")), intr.get("cy", float("nan"))),
          "- IMU 噪声(Kalibr avg-axis):Avia acc_n %.4g gyr_n %.4g;MID-360 acc_n %.4g gyr_n %.4g(A15 默认值取 Avia)。" % (
              imu.get("avia", {}).get("acc_n", float("nan")), imu.get("avia", {}).get("gyr_n", float("nan")),
              imu.get("mid360", {}).get("acc_n", float("nan")), imu.get("mid360", {}).get("gyr_n", float("nan"))), ""]
    return "\n".join(L)


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    with open(args.calibration_md, encoding="utf-8") as f:
        text = f.read()
    rows, intr, imu = parse_calibration(text)
    res = derive(rows, args.gt_lever)
    frag = yaml_fragment(res, intr, imu, args.lidar)
    with open(args.out_yaml, "w") as f:
        f.write(frag)
    if args.md:
        with open(args.md, "w") as f:
            f.write(markdown(res, args.source_label or args.calibration_md, intr, imu))
    bad = [c for c in res["checks"] if not c[1]]
    for name, ok, ev in res["checks"]:
        print("%-5s %s: %s" % ("PASS" if ok else "FAIL", name, ev))
    print("l^I avia   = %s" % _v(res["lever"]["l_avia_imu"], 6))
    if "l_mid360_imu" in res["lever"]:
        print("l^I mid360 = %s" % _v(res["lever"]["l_mid360_imu"], 6))
    if args.gt_lever is not None:
        print("GT antenna (C-RTK 2HP) in the Avia IMU frame    = %s (input)" % _v(res["lever"]["gt_avia_imu"], 6))
        if "gt_mid360_imu" in res["lever"]:
            print("GT antenna (C-RTK 2HP) in the mid360 IMU frame  = %s" % _v(res["lever"]["gt_mid360_imu"], 10))
            print("GT antenna in the mid360 frame (LiDAR origin)   = %s" % _v(res["lever"]["gt_mid360"], 10))
            print("mid360 IMU origin in the Avia IMU frame         = %s" % _v(res["lever"]["mid360_imu_origin_in_avia_imu"], 10))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
