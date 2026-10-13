> Historical research record (through 2026-10-03). Paths are illustrative; archived runs are not bundled. For current status and executable entry points see [the repository README](../../../README.md). Current release docs/LICENSING.md supersedes historical license notes. This record does not authorize new experiments or assert current release validation.

# 数据审计档案(WP-g)

本目录只放**摘要**(markdown,时间标定的 A18 文本内嵌其中);完整产物(`audit.json`、CSV、图、`gt_clean.tum`、
`gt_audited.tum`、`gt_timebase.yaml`、`trust_mask.csv`、`gt_mask_intervals.csv`、时间标定 YAML/JSON/图)归档在远端
`/work/fastlivo2_gnss/audit/<seq>/`;P2b 的旧版(tool_version 2)备份在远端 `wt/p4-time/home/audit_backup_p2b/<seq>/`。
归属:`docs/dev/audit/**` 归 WP-g(AMENDMENTS A40,OWNERSHIP §1c;WP-e 只可在此新建自己的文件)。

| 文件 | 内容 | 生成 |
|---|---|---|
| `<seq>.md` | I2 数据审计(`tools/m3dgr_audit.py`)+ GT 审计(`tools/gt_audit.py`)+ §7 时间标定(`tools/time_calib/gnss_time_calib.py`,方法①先验;有回放时方法②/③),由 `tools/audit_summary.py` 从远端归档拼接,「本轮说明」一节手写 | 远端容器 |
| `m3dgr_calib.md` | calibration.md → 杆臂 / 外参推导与自洽检查(`tools/m3dgr_calib.py --md`) | 远端容器 |

| 序列 | 类别(A2) | 工具版本 | 摘要 |
|---|---|---|---|
| Grass01、Grass02、Occlusion03、Occlusion04、Dynamic03 | 开发集 | 全部 tool_version 3(P4-TIME):时间标定、GT 审计、I2 审计(G13 用上 HEAD U 臂回放 `runs/eval/p4_time_u` 与本轮时间标定;G12b 分水平/竖直;G15 按 A43) | P4 重拼 |
| Varying-illu03/04/05、Outdoor01/04、Dark01/02、GNSS-denial01/02 | 评测集 | 全部 tool_version 3(AMENDMENTS A68,2026-09-26):I2 只做格式/解码审计;GT 审计无回放路线(GNSS-denial 为 ArUco,只做格式核对);时间标定方法①② 用 U 臂回放 `runs/eval/preproc_<seq>`(A64 评测构建,种子 0)。Outdoor01、Dark01 的 P2b 版(I2/GT tool_version 2、时间标定 tool_version 1)在 `audit/<seq>/.previous/<UTC>/` | A68 生成 |

## 数据纪律(AMENDMENTS A2)

Outdoor01、Dark01 属评测集:只有格式/解码审计。工具不读任何估计轨迹;自检 5(GNSS-only SPP/多普勒 LS 对接收机 PVT,
G12a/b)与自检 7(杆臂方向,G13)只在开发集上计算,评测集与类别未知的序列一律 SKIP、不产生数字(`--replay-dir` 在评测集上
根本不打开)。G11 只比较多普勒两种符号假设的拟合残差(解码性质)。GT 审计只用 GT 文件与原始传感器话题;P4c(回放路线)只在开发集
读回放。开发集可以随便看(A26)。

## 各产物的语义要点

- **C2/E6(A1/A38)**:RINEX NAV 是每条序列的必需输入;bag 缺 ephem/glo_ephem/receiver_pvt 为 WARN,缺 iono_params、
  time_pulse_info 不影响任何功能;GPS Klobuchar 只取 RINEX 头 GPSA/GPSB,bag 的 iono_params 只分类(GPS/BDS-like)作诊断。
- **G13 杆臂方向(§12.6-7)**:需要该序列 GNSS-free 回放的 `state_trace.txt`(R、v、b_g、g,估计器时钟)与方法②的 A18 文件。
  天线速度 = v + R(ω−b_g)×h,h ∈ {l, −l, 0},用回放重力调平、按假设各自拟合一个航向,预测每条多普勒;每历元钟漂作冗余参数。
  判定:新息对杆臂项的回归系数 k ≈ 0 且转弯历元新息 RMS(l) < RMS(−l) ⇒ PASS;k ≈ −2 ⇒ FAIL(天线在 −l);其余 WARN;
  转弯行 < 200 或杆臂项 RMS < 0.02 m/s ⇒ SKIP(激励不足)。只判方向(竖直分量主要不可观:地面车以航向角速率为主)。
- **GT 时基(P4 判定 + A54 取值规则)**:P4 判定规则不变(A40):轮速计链(GT→轮速计速度 + 轮速计→IMU 航向角速率,两条峰值都
  ≥ 0.8)为主,记录时间旁证须 ≤ 50 ms。**评测用的值按 A54 固定规则**(`gt_audit.decide_timebase`,tool_version 3):P4 通过 ⇒
  链;P4 失败 ⇒ 记录时间旁证;两者都没有 ⇒ 0 并标记;补偿仍按 |值| > 20 ms。于是 `gt_audited.tum` 总写出(= gt_clean + 补偿),
  `gt_timebase.yaml` 记方法、值、补偿、置信(chain 高;record_time 在亚阈值链 50 ms 内相符时中、否则低;none 无)与不确定度
  (chain:两链路 Carter 界合成,封顶 50 ms;record_time:50 ms,即 P4 对旁证与链一致性的要求),另列亚阈值链与 P4c(仅开发集,
  不参与取值)。每条链路另报延迟精度(峰高 + 峰宽 + 时长)。不设经 9Ps PVT 的 GNSS 路线:接收机速度有滤波延迟(Grass02 上比
  回放速度晚 0.14–0.19 s),「GT 速度对 PVT 速度」量到的是这个滤波器而不是时钟。
- **可信掩码(P5)**:TEST_PLAN §4 的「前后 2 s」对全部判据生效;不加宽的接收机判据比例另列;bag 记录区间外的 GT 行
  (bit 32)不可信、比例只在区间内统计。**A54 新增 bit 64「RTK 跳变/平台」(检查 P7)**:按位置变化的行,逐定位残差
  r = Δp − v_local·Δt(v_local = 前后各 3 个区间的中位速度),水平或垂直 > max(0.5 m, 10σ) 为跳变(0.5 m/0.2 s = 2.5 m/s 的
  速度突变,地面平台做不到;开发集残差 σ 1–3 cm);一对相互抵消的跳变 = 错位平台,连同前后 2 s 掩掉;未配对的跳变把较短一侧当错位
  并 WARN。被掩时长按判据以秒报告(bag 区间内);`gt_mask_intervals.csv` 给出区间(GT 戳与审计后时基两列),`ate_exclude`
  = bit 8|32|64(GT 自身缺陷:间断/冻结、无传感器数据、RTK 平台,A54「掩码内的时段不计入 ATE」);h_acc/num_sv/速度尖峰
  (bit 1/2/4)照旧报告但不标 ATE 排除(开发集上分别掩 100% 与 54–59%,判据未裁定)。
- **时间标定 YAML(tool_version 3,A53)**:`<seq>_record_time.yaml` = 只用方法①(不读回放);`<seq>.yaml` = 方法① + ②:
  逐时段取两者中更准的(方法②段 σ 大于先验 75 ms 的时段用方法①),方法②没有可用窗(无回放、GNSS 覆盖不足、激励不足)⇒ 方法①。
  **TDCP 只在两法都失败时暂停**:method `xcorr`/`record_time` 且 σ_toff ≤ 0.1 s ⇒ `tdcp_paused: false`(TDCP 的 R 已含
  (‖Δv_pair‖·σ_toff)²);`config`(无方法①先验时保留配置值)⇒ 暂停。方法②信号:两侧速度**矢量**经同一零相位高斯(σ 0.25 s)
  平滑后取**水平**分量之模;GNSS 速度 = 两遍加权多普勒 LS(C/N0 ≥ 15 dB-Hz、仰角 ≥ 10°、σ = λ·dopp_std,第二遍固定平滑后的
  钟漂);段的漂移取方法①先验,窗口只拟合水平;窗口须 90% GNSS 覆盖。所有 `*_gpst_s` 为 `GTime::toSec()`(1970 原点、GPST
  尺度、不含闰秒;A18)。tool_version ≤ 2 的文件(评测集 Dark01/Outdoor01 的旧先验)写着 TDCP 暂停,A68 已重做并移到 `.previous/`。

## 评测阶段一键命令(逐序列预处理;A68 已对九条评测集跑完,记录与结果见 AMENDMENTS A68)

评测集的时间标定与 GT 审计是逐序列预处理(不看精度:时间标定只比回放速度与 GNSS 多普勒速度,GT 审计只比 GT 与原始传感器话题,
P4c 回放路线只在开发集)。`tools/time_calib/preprocess_seq.py` 一次做完:HEAD 的 U 臂回放(A55,`gnss.enable: false`,种子 0;
经 `tools/eval/run_matrix.py`,矩阵名 `preproc_<seq>`,已完成则复用)→ 方法① 与 方法①+② 时间标定 → GT 审计(A54),写进
`audit/<seq>/`(旧文件先移到 `audit/<seq>/.previous/<UTC>/`),摘要 `audit/<seq>/preprocess.json`。评测集必须带
`--evaluation-phase`(A2)。输入:过滤后的包(`fetch_eval.sh` 已保留 RTK 定位与 `/odom` 话题)、`data/m3dgr/gt/<seq>.txt`、同日
BRDC(取数记录或按日期)。

```bash
# 本机:同步到一棵长任务用的远端树并以 systemd 用户单元运行(<HEAD eval 树> = 当前 HEAD 的 eval profile 构建,例如 p3i-strict)
L=tools/eval/launch_remote.sh; W=/workspace/fastlivo2_gnss_wt/<本机树>
for S in Varying-illu03 Varying-illu04 Varying-illu05 Outdoor01 Outdoor04 Dark01 Dark02; do
  $L $W <远端名> start preproc-$S -- python3 tools/time_calib/preprocess_seq.py $S --evaluation-phase --devel <HEAD eval 树>:ws/devel
done
$L - - logs preproc-Outdoor01 20          # 末行一句摘要:时间标定段(方法、偏移、σ、TDCP)与 GT 时基(方法、值、补偿、置信)、ATE 排除时长
# 开发集同一条命令去掉 --evaluation-phase;已有完整回放时加 --replay-dir runs/eval/<矩阵>/<seq>/U/s0;--dry-run 只打印计划
```
GNSS-denial01/02 的 GT 是 ArUco 起终点(不是轨迹):只做时间标定,`gt_kind` 非 rtk 时 GT 审计自动跳过。

## 重新生成(全部在远端容器里,经 `tools/dev/remote_tool.sh`:先拿主机锁,容器内 nice 19 / ionice idle)

```bash
T=tools/dev/remote_tool.sh; W=/workspace/fastlivo2_gnss_wt/p4-time; N=p4-time  # 本地工作树与远端名
S=Grass02; D=BRDC00IGS_R_20250210000_01D_MN.rnx                             # 序列与同日 IGS BRDC(见下表)
R=/r/runs/eval/p4_time_u/$S/U/s0                                             # 该序列的 HEAD U 臂回放(A55;没有就省略相关参数)
$T $W $N -- python3 tools/time_calib/gnss_time_calib.py /r/data/m3dgr/bags/$S.bag - /r/audit/$S/time_calib/${S}_record_time.yaml --seq $S --plot
$T $W $N -- python3 tools/time_calib/gnss_time_calib.py /r/data/m3dgr/bags/$S.bag $R /r/audit/$S/time_calib/$S.yaml --seq $S --nav-rinex /r/data/brdc/$D --plot
$T $W $N -- python3 tools/gt_audit.py /r/data/m3dgr/gt/$S.txt /r/data/m3dgr/bags/$S.bag /r/audit/$S/gt --seq $S \
      --archive-label /work/fastlivo2_gnss/audit/$S/gt --replay-dir $R
$T $W $N -- python3 tools/m3dgr_audit.py /r/data/m3dgr/bags/$S.bag /r/audit/$S --nav-rinex /r/data/brdc/$D \
      --gt /r/data/m3dgr/gt/$S.txt --archive-label /work/fastlivo2_gnss/audit/$S --replay-dir $R --time-calib /r/audit/$S/time_calib/$S.yaml
# 摘要:把远端 audit/$S 的 *.md/*.json/*.yaml 拉到本地 <dir>,再
python3 tools/audit_summary.py <dir>/$S --seq $S --notes <本轮说明.md> --out docs/dev/audit/$S.md
tools/dev/run_tool_tests.sh $W $N            # tools/tests(unittest)
```

| 序列 | 日期(UTC) | IGS BRDC |
|---|---|---|
| Grass01 | 2025-01-19 | `BRDC00IGS_R_20250190000_01D_MN.rnx` |
| Grass02 | 2025-01-21 | `BRDC00IGS_R_20250210000_01D_MN.rnx` |
| Occlusion03、Occlusion04 | 2024-12-01 | `BRDC00IGS_R_20243360000_01D_MN.rnx` |
| Dynamic03 | 2024-12-06 | `BRDC00IGS_R_20243410000_01D_MN.rnx` |
| Dark01 | 2024-11-25 | `BRDC00IGS_R_20243300000_01D_MN.rnx` |
| Outdoor01、Outdoor04 | 2025-01-03 | `BRDC00IGS_R_20250030000_01D_MN.rnx` |
| Dark02 | 2024-11-25 | `BRDC00IGS_R_20243300000_01D_MN.rnx` |
| Varying-illu03/04/05 | 2025-01-13 | `BRDC00IGS_R_20250130000_01D_MN.rnx` |
| GNSS-denial01 | 2025-01-19 | `BRDC00IGS_R_20250190000_01D_MN.rnx` |
| GNSS-denial02 | 2025-01-21 | `BRDC00IGS_R_20250210000_01D_MN.rnx` |

`bag_filter.py` 会删掉 `/mavros/global_position/raw/fix`、`/odom`、`/livox/mid360/*` 等话题:**先对原包跑完
m3dgr_audit / gt_audit**(或用 `--keep` 保留这些话题)再删原包(IMPL_PLAN §2「删原包」)。
