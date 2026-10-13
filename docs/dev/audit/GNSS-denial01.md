> Historical research record (through 2026-10-03). Paths are illustrative; archived runs are not bundled. For current status and executable entry points see [the repository README](../../../README.md). Current release docs/LICENSING.md supersedes historical license notes. This record does not authorize new experiments or assert current release validation.

# I2 数据审计 — GNSS-denial01

生成:`tools/m3dgr_audit.py`(tool_version 3),2026-09-26T13:28:13Z;bag `/work/fastlivo2_gnss/data/m3dgr/fetch/GNSS-denial01/x/GNSS-denial01.bag`(11228253212 字节,2025-01-30T04:50:09Z;sha256 未计算)。序列类别:**evaluation**(AMENDMENTS A2)。

> 评测集:本报告只做格式/解码审计。没有读取任何估计轨迹;不做 GNSS-only 解对接收机 PVT 的比较(§12.6-5 自检 G12a/b)与杆臂方向(§12.6-7 自检 G13),两者在评测集上 SKIP(A2);G11 只比较多普勒两种符号假设的拟合残差(解码性质)。

总评:**WARN**(INFO 9, PASS 19, SKIP 4, WARN 2)。远端归档:`/work/fastlivo2_gnss/audit/GNSS-denial01`。

## 本轮说明

评测集(A2),AMENDMENTS A68(2026-09-26,tool_version 3)。原包 = 阿里云盘 `/来自分享/GNSS-denial01`(4 段分卷,cat 拼接后 7z 解 SFX),sha256 与官方一致,已索引;审计后删除。

过滤后 `data/m3dgr/filtered/GNSS-denial01.bag`(3.38 GiB,sha256 `97c200e22a18b9e8…`;保留 Avia LiDAR/IMU、压缩彩色图、`/ublox_driver/*`、RTK 定位与 `/odom`,逐消息核对通过)。BRDC `BRDC00IGS_R_20250190000_01D_MN.rnx`。时长 609.7 s。

**只做格式/解码审计、GT 自身审计与时间标定**:没有读取任何估计轨迹对真值的比较;自检 5(G12a/b)、自检 7(G13)与 GT 审计的回放路线 P4c 在评测集上不计算。方法②只比较 U 臂 GNSS-free 回放(A55,HEAD 零通道,A64 评测构建 `wt/a64-strict`,种子 0,`runs/eval/preproc_GNSS_denial01/GNSS-denial01/U/s0`)的天线水平速度与多普勒 LS 水平速度。

**时间标定**:method xcorr +20.2 ms、σ_toff 70.7 ms、漂移 -4.48 ppm(2 窗,峰值相关 0.996);TDCP 不暂停(A53)。

**来源**:M3DGR README:399 的单独阿里云盘分享 `https://www.alipan.com/s/KafsJdSFrYz`,用户 2026-09-26 导入,落在资源库 `/来自分享/GNSS-denial01/`(不在 `/来自分享/Dataset/` 下;登记表路径见 `67cc329`)。

**GT**:ArUco 起终点(不是轨迹):11 行文本 = 3×3 旋转、平移 3 个数与 `bag_time: 609s`(与包时长 609.7 s 一致);旋转正交到 1.6e-8、行列式 1,起终点相对转角 1.96°、|t| 0.046 m;两张 640×480 JPEG。无时间戳,GT 时基与可信掩码不适用;指标为官方 `ArUco_evaluate.py` 的起终点误差(TEST_PLAN E5,A56),仓库里尚未实现。

## 供 WP-a / 核心使用的结论

- `gnss.ephem.ura_semantics: metres`(A16;判定依据见 E4)。
- std 单位(§12.6-3):psr_std=metres,cp_std=cycles,dopp_std=hertz;cp_std=0.06 是 u-blox cpStdev=15 的『无效』标记,占 40.0%。
- 星历覆盖(A1):卫星-历元覆盖率 仅 bag 77.0% / bag+RINEX 90.8%;≥5 星历元比例 94.6% / 97.4% ⇒ 必须配置 `gnss.nav_rinex`。
- Klobuchar(A38):GPS 参数取 RINEX 头 GPSA/GPSB(本日可用);bag 的 iono_params 只记诊断,E6 判为 BDS-like(发布节奏 11.4 s,与 BDS D1 子帧 1 在各北斗星上轮播相符)。
- BDS 星历字段(E1 对拍 51 条):toe(GnssTimeMsg)是 GPST(51/51);`toe_tow` 字段是 BDT 周内秒的只有 0 条、`week` 字段是 GPS 周的 51 条 ⇒ 核心须按 §12.3 由 GPST toe 重算 toes(BDT)与 BDT 周,不得直接用消息的 toe_tow/week。
- 观测频点字段与信号码不一致(G3):E code 28 (L7Q (E5bQ), 1207.14 MHz) reported at 1227.6000 MHz: 17468 slots ⇒ ublox_driver 把这些槽的 freqs 写错;按 §12.3 频点表它们被拒收,主信号不受影响,只失去该系统的第二频点 GF 周跳检测(如需使用,可在 gnss_io 按 code 映射,需维护者决定)。
- C/N0 中位 BDS 27, GAL 27, GLO 28, GPS 27 dB-Hz;按 DESIGN §10 L0 掩码(伪距 30 / 多普勒 25 / 载波 28 dB-Hz)可用率 伪距 BDS 30%, GAL 4%, GLO 16%, GPS 6%、多普勒 BDS 73%, GAL 82%, GLO 83%, GPS 76% ⇒ 默认掩码会饿死滤波与初始化(COLLECT 需 ≥5 星 SPP);掩码必须在开发集按 TEST_PLAN §3 重新标定,不得在本序列上调。
- range_meas 记录时间 − GPST→UTC:中位 -66.6 ms、MAD 13.8 ms、漂移 -61.9 ppm、台阶 无(= 主机钟偏差+到达延迟;作 §7 方法①先验,不是最终时偏)。
- 传感器时基(C4,记录时间 − 头戳中位,漂移):Avia LiDAR -21.2 ms / -52 ppm、Avia IMU -76.0 ms / -19 ppm、相机 44.1 ms。Livox 头戳 = 设备时间戳 + 常数(S1:timebase on another clock (offset 1.73728e+09 s, drift 0.0 ppm vs header)),与主机时钟的关系逐序列不同(本序列 LiDAR 头戳领先主机收包时刻,不可能是主机收包戳) ⇒ 估计器时基(LiDAR 头戳)对 GPST 的偏差与漂移只能由 §7 方法②逐序列标定;range_meas 记录时间只作先验。
- 载波/多普勒符号(§12.6-2):斜率 1.0000、相关 1.00000 ⇒ `λ·dΦ/dt = −λD` 成立,与 DESIGN §1 一致。
- Livox:/livox/mid360/lidar livox_ros_driver2/CustomMsg [e4d6829b], /livox/avia/lidar livox_ros_driver/CustomMsg [e4d6829b] 与仓库 include/livox_ros_driver 头 md5 一致。
- CompressedImage format:{'rgb8; jpeg compressed bgr8': 18276};回放直读应 `IMREAD_COLOR` 并按 format 决定通道序(S2)。S2 的 PASS 是 Python 对两条解码路径的仿真,C++ 回放路径本身待 WP-f 实测。
- §12.6-7 杆臂方向:评测集不做(A2,标定级判断只在开发集);calibration.md 方向推导见 docs/dev/audit/m3dgr_calib.md。
- 自检 1 的 sat2freq 交叉核对(A35-C8,G14):全部一致。

## 检查表

| # | check | verdict | evidence |
|---|---|---|---|
| C1 | msg_def md5 recomputed = connection md5 (and = installed ROS type md5 where installed) | **PASS** | 17 connections, 0 mismatches; 9 standard types also matched the installed ROS packages |
| C2 | topics of the replay/GNSS path present (DESIGN §14; since A38 the RINEX NAV file is an input of every sequence) | **PASS** | missing: none |
| C3 | rates and time ranges (header-stamp period; record-time average) | **INFO** | lidar 10.0 Hz (header), avg 10.00 Hz; imu 200.1 Hz (header), avg 203.45 Hz; compressed 30.0 Hz (header), avg 29.98 Hz; range_meas avg 10.00 Hz (6096 msgs); ephem avg 0.21 Hz (128 msgs); glo_ephem avg 0.08 Hz (44 msgs); receiver_pvt avg 10.00 Hz (6096 msgs); iono_params avg 0.09 Hz (51 msgs); time_pulse_info avg 1.00 Hz (610 msgs); no gaps |
| C4 | header stamp vs record time per topic (median/p1/p99 ms, drift ppm) | **INFO** | /camera/color/image_raw/compressed 44.1/25.7/56.8 ms, 4 ppm; /livox/avia/imu -76.0/-152.7/-9.2 ms, -19 ppm; /livox/avia/lidar -21.2/-65.9/-1.7 ms, -52 ppm; /mavros/global_position/raw/fix 19.7/-7.5/105.4 ms, -19 ppm; /ublox_driver/iono_params 219.0/168.7/271.1 ms, -90 ppm |
| G1 | epoch interval 10 Hz, monotone, one time tag per epoch (§12.6-1) | **PASS** | 6096 epochs; 100.00 % at 0.1 s; 0 non-increasing; 0 epochs with >1 time tag; 0 gaps >0.15 s |
| G15 | 1 Hz decimation rule (AMENDMENTS A43: the first epoch of every GPST second; the retired DESIGN §4.3 5 ms rule reported beside it) | **PASS** | A43 rule: 610 epochs for 610 GPST seconds with data; 10 Hz epoch grid offset +2.0 ms from the GPST tenths; the retired \|tow - round(tow)\| < 5 ms rule would select an epoch in 99.8% of the 611 seconds |
| G2 | per-satellite array lengths consistent; sat numbers inside the gnss_comm ranges | **PASS** | 0 inconsistent satellites; unknown sat numbers: none |
| G3 | (sys, freq, code) histogram and freqs->code table verdicts (signal_table.hpp) | **WARN** | C 1207.1400 MHz code=27->L7I: 16586; C 1561.0980 MHz code=47->L2I: 30684; E 1227.6000 MHz code=28->Unknown: 17468; E 1575.4200 MHz code=1->L1X: 19071; G 1227.6000 MHz code=17->L2X: 9151; G 1575.4200 MHz code=1->L1C: 29121; R 1244.6875 MHz code=14->L2C: 2267; R 1245.5625 MHz code=14->L2C: 4857; R 1246.8750 MHz code=14->L2C: 50; R 1600.3125 MHz code=1->L1C: 2810; R 1600.8750 MHz code=1->L1C: 1278; R 1601.4375 MHz code=1->L1C: 5261; R 1603.1250 MHz code=1->L1C: 503; R 1603.6875 MHz code=1->L1C: 5477; verdicts {'Accepted': 127116, 'RejectedUnknownFreq': 17468}; primary slot first: {'slot0': 94205, 'none': 3207}; freqs field inconsistent with the code field: E code 28 (L7Q (E5bQ), 1207.14 MHz) reported at 1227.6000 MHz: 17468 slots (the table rejects these slots; only the second-frequency GF slip check loses them) |
| G4 | satellites per epoch (primary signal present) | **INFO** | total min/mean/max 1/15.5/23; BDS 0/5.0/8, GAL 0/3.1/5, GPS 1/4.8/8, GLO 0/2.5/4 |
| G5 | C/N0 distribution (dB-Hz, primary signal) | **INFO** | BDS median 27 p5 16 p95 31; GAL median 27 p5 15 p95 29; GLO median 28 p5 21 p95 30; GPS median 27 p5 15 p95 30 |
| G6 | psr / cp / dopp availability (status bits + value; with the L0 C/N0 masks) | **INFO** | BDS psr 100% cp 57% dopp 100% (masked 30/45/73%); GAL psr 100% cp 72% dopp 100% (masked 4/42/82%); GLO psr 100% cp 72% dopp 100% (masked 16/56/83%); GPS psr 100% cp 57% dopp 100% (masked 6/38/76%) |
| G7 | LLI / status statistics | **INFO** | LLI {'GPS': {'3': 11014, '0': 14032, '2': 4036, '1': 39}, 'BDS': {'3': 12300, '0': 15740, '2': 2622, '1': 22}, 'GLO': {'0': 9774, '3': 4008, '2': 1529, '1': 18}, 'GAL': {'0': 14251, '3': 4810, '1': 10}}; status {'GPS': {'1': 11063, '7': 3701, '15': 10370, '3': 3987}, 'BDS': {'1': 12331, '15': 7825, '7': 7937, '3': 2568, '11': 23}, 'GLO': {'15': 4910, '7': 4882, '1': 4039, '3': 1470, '11': 28}, 'GAL': {'7': 14234, '1': 4810, '5': 27}} |
| G8 | std unit judgement on the u-blox grids (§12.6-3) | **PASS** | psr metres (100.0% on 0.01*2^n m); cp cycles (100.0% on 0.004n cyc, 0.0% on 0.004n*lambda m, 0.0% on 0.004n/lambda); dopp hertz (100.0% on 0.002*2^n Hz, 0.0% as m/s); cp_std=0.06 (cpStdev=15, u-blox 'invalid') on 40.0% of slots |
| G9 | carrier-phase rate vs Doppler sign/scale (§12.6-2: corr > 0.99, slope ~ 1) | **PASS** | 58014 pairs (clean (LLI bit0 = 0)): corr 1.00000, slope 1.0000, median \|diff\| 0.054 m/s; per satellite (§12.6-2 '每星', decisive = >= 50 clean pairs and range-rate std >= 2 m/s): 21/21 decisive satellites with corr > 0.99 and \|slope - 1\| < 0.02 (slopes 0.9980..1.0024), 1 not decisive; 1 s sign agreement lambda*dPhi vs dP 100.0%, vs -lambda*D*dt 100.0% (n=5461) |
| G10 | GLONASS channel k from freqs vs ephemeris frq (§12.3, §12.6-1) | **PASS** | 22503 slots; mismatching satellites none; observed without any ephemeris: none |
| G14 | self-check 1: RTKLIB sat2freq(sat, code, nav) of every accepted slot = its freqs (A35-C8) | **PASS** | 127116 slots consistent, 0 mismatching, 0 unverifiable (GLONASS without an ephemeris channel: none), 17468 slots not accepted by the table (G3); per system {'BDS': {'consistent': 47270}, 'GAL': {'consistent': 19071, 'not_accepted': 17468}, 'GPS': {'consistent': 38272}, 'GLO': {'consistent': 22503}} |
| E1 | bag Kepler ephemerides vs same-day RINEX (same sat,toe): \|dr\| < 1e-3 m, \|ddts\| < 1e-11 s (A9-3 tier 2) | **PASS** | 128/128 matched, 0 outside tier 2 (0 not explained by a corrupted copy, see E7); worst per system {'G': '2.03e-04 m / 3.66e-16 s', 'E': '3.40e-04 m / 4.50e-15 s', 'C': '2.22e-04 m / 3.97e-16 s'}; toe label shifts {'G:+0': 43, 'E:+0': 34, 'C:+0': 51}; field semantics {'G': {'n': 43, 'toe_tow_is_system_tow': 43, 'week_is_system_week': 43, 'week_is_gps_week': 43, 'toe_msg_is_gpst': 43, 'tgd0_eq': 43, 'tgd1_eq': 43, 'tgd0_eq_rnx_tgd1': 0, 'iode_eq': 43, 'svh_eq': 43, 'code_eq': 0}, 'E': {'n': 34, 'toe_tow_is_system_tow': 34, 'week_is_system_week': 34, 'week_is_gps_week': 34, 'toe_msg_is_gpst': 34, 'tgd0_eq': 34, 'tgd1_eq': 34, 'tgd0_eq_rnx_tgd1': 5, 'iode_eq': 34, 'svh_eq': 34, 'code_eq': 34}, 'C': {'n': 51, 'toe_tow_is_system_tow': 0, 'week_is_system_week': 0, 'week_is_gps_week': 51, 'toe_msg_is_gpst': 51, 'tgd0_eq': 51, 'tgd1_eq': 51, 'tgd0_eq_rnx_tgd1': 29, 'iode_eq': 51, 'svh_eq': 51, 'code_eq': 51}} |
| E7 | bag ephemeris stream consistency: all copies of one (sat, toe, IODE) identical | **PASS** | 19 keys, 0 with conflicting copies |
| E2 | bag GLONASS ephemerides vs RINEX (pos/vel/acc units, tau_n sign, channel) | **PASS** | 44/44 matched; worst \|dr\| at toe+600 s 4.30e-05 m; semantics {'n': 44, 'taun_same_sign_as_rtklib': 44, 'taun_opposite_sign': 0, 'gamn_eq': 44, 'frq_eq': 44, 'pos_eq_1mm': 44} |
| E4 | URA semantics: bag ura vs same-day RINEX SV accuracy (§12.6-4, A16) | **PASS** | decision per system {'G': 'metres', 'E': 'metres', 'C': 'metres'} -> gnss.ephem.ura_semantics: metres; counts {'G': {'n': 43, 'metres_exact': 43, 'metres_approx': 43, 'index': 0, 'icd_formula_metres': 0}, 'E': {'n': 34, 'metres_exact': 34, 'metres_approx': 34, 'index': 0}, 'C': {'n': 51, 'metres_exact': 51, 'metres_approx': 51, 'index': 0, 'icd_formula_metres': 0}}; samples (bag, rinex m) {'G': [(2.0, 2.0), (2.0, 2.0), (2.0, 2.0), (2.0, 2.0), (2.0, 2.0), (2.0, 2.0), (2.0, 2.0), (2.0, 2.0)], 'E': [(3.12, 3.12), (3.12, 3.12), (3.12, 3.12), (3.12, 3.12), (3.12, 3.12), (3.12, 3.12), (3.12, 3.12), (3.12, 3.12)], 'C': [(2.0, 2.0), (2.0, 2.0), (2.0, 2.0), (2.0, 2.0), (2.0, 2.0), (2.0, 2.0), (2.0, 2.0), (2.0, 2.0)]} |
| E3 | orbit code self-check: adjacent RINEX records agree at the midpoint (p95 < 10 m per system) | **PASS** | C median 0.11 m p95 0.41 m (n=260); E median 0.08 m p95 0.33 m (n=658); G median 0.39 m p95 2.28 m (n=104) |
| E5 | ephemeris coverage timeline: bag-only (causal by record time) vs bag+RINEX (A1, §12.6-8) | **INFO** | sat-epochs with a healthy ephemeris: bag-only 77.0%, bag+RINEX 90.8%; epochs with >=5: 94.6% / 97.4%; first >=5 at 7.0 / 0.0 s; BDS GEO observed none; never covered bag-only: C06, C09, C16, E18, E27, G01, G07, G08, G19, R13, R24 |
| E6 | Klobuchar source (A38): GPS set = RINEX header GPSA/GPSB; bag iono_params only a diagnostic (family classified) | **PASS** | GPS Klobuchar from the RINEX header GPSA/GPSB: ['2.98e-08', '2.98e-08', '-1.192e-07', '0', '1.516e+05', '-1.638e+05', '0', '6.554e+04']; bag iono_params (diagnostic, not used, A38): 51 messages, 1 distinct sets, one every 11.4 s on average; set ['3.446e-08', '1.714e-07', '-1.49e-06', '2.146e-06', '1.331e+05', '-3.768e+05', '1.049e+06', '0']: BDS-like (distance to RINEX GPSA/GPSB 18 LSB, to the nearest RINEX BDSA/BDSB 12 LSB) |
| L1 | GNSS arrival latency: range_meas record time - GPST->UTC (leap 18 s) | **INFO** | median -66.6 ms, MAD 13.8, p1 -110.3, p99 -47.1, min -111.9, max -46.2; drift -61.9 ppm; persistent steps >50 ms: none; receiver_pvt median -42.7 ms |
| L3 | self-check 6 (time, record-time part): \|record - (GPST - leap)\| < 0.3 s (leap seconds and time system consistent) | **PASS** | median -0.0666 s with leap 18 s (a missing/extra leap second would show as ~±1 s, a GPST/UTC mix-up as ±18 s); the cross-correlation part of self-check 6 is method 2 of tools/time_calib/gnss_time_calib.py on the ref-R replay |
| L2 | receiver_lla header stamp convention | **INFO** | header.stamp - GPST(seconds, Unix origin) median 0.000 s (min 0.000, max 0.000): receiver_lla stamps are GPST without leap seconds (not ROS/UTC time) |
| T1 | time_pulse_info checks (A19: leap second, time system, gross errors, host clock steps) | **WARN** | 610 messages ({'UTC': 610}), pulse period 5.000 s, 487 repeated announcements, 0 gross interval errors; lead of the announcement before its pulse: median 2.953 s, window [0, 5) s, integer parts [0, 1, 2, 3, 4] with phase 0.957 s (jitter 19.5 ms) => integer-second offset +0 s: consistent; the phases (0.927..0.995 s) reach a whole second: integer parts unstable, ambiguous (0% of the messages give another k); record-UTC median -3019.8 ms; host steps none |
| G11 | Doppler sign by LS fit: -lambda*D = range rate (§1) vs flipped sign (all Dopplers, el >= 5°) | **PASS** | post-fit RMS median 0.080 m/s with the design sign vs 387.6 m/s flipped (594 epochs at 1.0 Hz; linearised at the receiver PVT (evaluation sequence, A2)) |
| G12a | self-check 5 (position): SPP vs receiver PVT | **SKIP** | A2: not a development sequence — self-check 5 compares GNSS-only solutions with the receiver PVT (a calibration-grade judgement, TEST_PLAN §3: development set only); not computed |
| G12b | self-check 5 (velocity): Doppler LS vs receiver PVT | **SKIP** | A2: not a development sequence — self-check 5 compares GNSS-only solutions with the receiver PVT (a calibration-grade judgement, TEST_PLAN §3: development set only); not computed |
| G13 | lever-arm direction (§12.6-7): Doppler innovations with l vs -l on turns; regression on the omega x l term (~0; ~-2 = sign flipped) | **SKIP** | A2: not a development sequence — the lever-arm judgement uses the replay trajectory and belongs to the development set; not computed |
| S1 | Livox CustomMsg type/md5 vs the repository's livox_ros_driver headers | **PASS** | /livox/avia/lidar: livox_ros_driver/CustomMsg [e4d6829bdfe657cb6c21a746c86b21a6] vs repo livox_ros_driver/CustomMsg [e4d6829bdfe657cb6c21a746c86b21a6]; timebase on another clock (offset 1.73728e+09 s, drift 0.0 ppm vs header); points/scan median 24000 |
| S2 | CompressedImage format; replay direct decode vs republish path, bgr8 and grey pixel-exact (I2) | **PASS** | formats {'rgb8; jpeg compressed bgr8': 18276}; 150 sampled: bgr equal 150, grey equal 150, IMREAD_UNCHANGED==IMREAD_COLOR 150; shapes {'(480, 640, 3)': 150} (Python emulation of both decode paths with OpenCV; the C++ replay path itself is to be measured by WP-f) |
| S3 | GT file format | **SKIP** | no --gt given |

## 话题清单

| topic | type | md5 | md5 recomputed | msgs | rate [Hz] (header, else avg) | record-header median [ms] |
|---|---|---|---|---|---|---|
| /camera/imu | sensor_msgs/Imu | 6a62c6daae103f4ff57a132d6f95cec2 | yes | 121927 | 200.18 | -1.5 |
| /livox/mid360/imu | sensor_msgs/Imu | 6a62c6daae103f4ff57a132d6f95cec2 | yes | 121928 | 202.18 | 7.7e-02 |
| /camera/aligned_depth_to_color/image_raw/compressedDepth | sensor_msgs/CompressedImage | 8f7a12909da2c9d3332d540a0977563f | yes | 18277 | 30.01 | 42.2 |
| /camera/color/image_raw/compressed | sensor_msgs/CompressedImage | 8f7a12909da2c9d3332d540a0977563f | yes | 18276 | 30.01 | 44.1 |
| /odom | nav_msgs/Odometry | cd5e73d190d741a2f92e81eda573aca7 | yes | 12193 | 20.00 | 8.4e-02 |
| /cv_camera/image_raw/compressed | sensor_msgs/CompressedImage | 8f7a12909da2c9d3332d540a0977563f | yes | 9145 | 15.00 | 18.4 |
| /ublox_driver/range_meas | gnss_comm/GnssMeasMsg | 0eafeb530a3e6637722c4e296734c119 | yes | 6096 | 10.00 | — |
| /livox/mid360/lidar | livox_ros_driver2/CustomMsg | e4d6829bdfe657cb6c21a746c86b21a6 | yes | 6096 | 10.00 | 100.8 |
| /livox/avia/imu | sensor_msgs/Imu | 6a62c6daae103f4ff57a132d6f95cec2 | yes | 124010 | 200.15 | -76.0 |
| /livox/avia/lidar | livox_ros_driver/CustomMsg | e4d6829bdfe657cb6c21a746c86b21a6 | yes | 6096 | 10.02 | -21.2 |
| /mavros/global_position/raw/fix | sensor_msgs/NavSatFix | 2d3a8cd499b9b4a0249fb98fd05cfa48 | yes | 9144 | 15.00 | 19.7 |
| /ublox_driver/receiver_pvt | gnss_comm/GnssPVTSolnMsg | d18171357d7a159f76d4d7c0b12fb631 | yes | 6096 | 10.00 | — |
| /ublox_driver/receiver_lla | sensor_msgs/NavSatFix | 2d3a8cd499b9b4a0249fb98fd05cfa48 | yes | 6096 | 10.00 | -18042.7 |
| /ublox_driver/time_pulse_info | gnss_comm/GnssTimePulseInfoMsg | de306035e295e3c4b1e0771a216b495a | yes | 610 | 1.00 | — |
| /ublox_driver/ephem | gnss_comm/GnssEphemMsg | b4346f99e431c870e392a438684c0fe8 | yes | 128 | 0.21 | — |
| /ublox_driver/glo_ephem | gnss_comm/GnssGloEphemMsg | 6893dee0d8b1c9325a6693f3488e0144 | yes | 44 | 0.08 | — |
| /ublox_driver/iono_params | gnss_comm/StampedFloat64Array | fb60495edd59d3fcf90e173153ae8a9a | yes | 51 | 0.09 | 219.0 |

## 信号直方图 (sys, freq, code 字段 → signal_table)

| sys | freq [MHz] | code field | table code | count |
|---|---|---|---|---|
| C | 1207.1400 | 27 | L7I | 16586 |
| C | 1561.0980 | 47 | L2I | 30684 |
| E | 1227.6000 | 28 | Unknown | 17468 |
| E | 1575.4200 | 1 | L1X | 19071 |
| G | 1227.6000 | 17 | L2X | 9151 |
| G | 1575.4200 | 1 | L1C | 29121 |
| R | 1244.6875 | 14 | L2C | 2267 |
| R | 1245.5625 | 14 | L2C | 4857 |
| R | 1246.8750 | 14 | L2C | 50 |
| R | 1600.3125 | 1 | L1C | 2810 |
| R | 1600.8750 | 1 | L1C | 1278 |
| R | 1601.4375 | 1 | L1C | 5261 |
| R | 1603.1250 | 1 | L1C | 503 |
| R | 1603.6875 | 1 | L1C | 5477 |

## 星历字段语义(bag vs 同日 RINEX,按 (sat,toe) 配对的计数)

| sys | code_eq | iode_eq | n | svh_eq | tgd0_eq | tgd0_eq_rnx_tgd1 | tgd1_eq | toe_msg_is_gpst | toe_tow_is_system_tow | week_is_gps_week | week_is_system_week |
|---|---|---|---|---|---|---|---|---|---|---|---|
| C | 51 | 51 | 51 | 51 | 51 | 29 | 51 | 51 | 0 | 51 | 0 |
| E | 34 | 34 | 34 | 34 | 34 | 5 | 34 | 34 | 34 | 34 | 34 |
| G | 0 | 43 | 43 | 43 | 43 | 0 | 43 | 43 | 43 | 43 | 43 |

## 产物

`audit.json`(全部数字)、`latency.csv`、`eph_coverage.csv`、`eph_coverage_timeline.csv`、`plot_*.png`,位于远端归档目录。

## 时间标定(tools/time_calib/gnss_time_calib.py,DESIGN §7)

### 方法①(记录时间统计与估计器时钟先验)

range_meas 记录时间 − GPST→UTC(leap 18 s,主机钟视角 = 主机钟偏差 + GNSS 传输延迟):中位 -66.6 ms、MAD 13.8 ms、p5/p95 -106.3/-48.1 ms、漂移 -61.9 ppm、持续台阶(>50 ms)无。

头戳 − 记录时间(判定 LiDAR 时基):/camera/color/image_raw/compressed 中位 -44.1 ms(p1 -56.8 / p99 -25.7,漂移 -4.3 ppm);/livox/avia/imu 中位 76.0 ms(p1 9.2 / p99 152.7,漂移 18.8 ppm);/livox/avia/lidar 中位 21.2 ms(p1 1.7 / p99 65.9,漂移 51.8 ppm)。

估计器时钟(Livox 头戳)先验:GNSS「记录 − UTC」下包络 -67.9 ms 减 Avia IMU「记录 − 头戳」下包络 -116.7 ms ⇒ 上界 48.4 ms(主机钟台阶在两者中抵消;漂移 -4.5 ppm;估计器时钟台阶 无);GNSS 传输延迟不可观,按 [0, 150] ms 均匀(推断)⇒ 先验 -26.6 ms、σ 75 ms。

方法①文件 `GNSS-denial01_record_time.yaml`(不读回放;method record_time,σ 75 ms;tool_version ≥ 3 按 AMENDMENTS A53 TDCP 不暂停,tool_version ≤ 2 的文件仍写着暂停):

```yaml
sequence: GNSS-denial01
tool_version: 3
leap_s: 18
segments:
  - t_from_gpst_s: 0
    offset_s: -0.02657439738101955
    drift_ppm: -4.47837132333502
    t_ref_gpst_s: 1737276886.7519999
    sigma_toff_s: 0.075
    tdcp_paused: false
    method: record_time
```

### 方法②(水平天线速度互相关)与方法③(DTW)

输入:回放轨迹 `/work/fastlivo2_gnss/runs/eval/preproc_GNSS_denial01/GNSS-denial01/U/s0/traj_W.tum`(tum,18265 个速度样本,估计器速度 std 0.19 m/s);多普勒 LS 速度 5859 个历元。窗口 19 个,可用 2 个(窗长/步长见 JSON)。状态:ran。

信号(tool_version 3):horizontal antenna speed = |horizontal part of the zero-phase Gaussian-smoothed (0.25 s) velocity vector| on both sides, 100 Hz grid;W 的竖直方向:state_trace gravity (median g = [-0.3666, -0.0474, -9.8029])。多普勒 LS:C/N0 ≥ 15.0 dB-Hz、仰角 ≥ 10.0°,按接收机 dopp_std 加权,两遍(第二遍固定平滑后的钟漂):5911 个历元有 ≥ 4 行,第二遍解出 5859 个(每历元用行中位 14.0)。

| 窗起点 [s] | 用 | 峰值相关 | 偏移 [ms] | 峰宽精度 [ms] | GNSS 覆盖 | 原因 |
|---|---|---|---|---|---|---|
| 0 | 是 | 0.999 | 33.5 | 31.8 | 100% |  |
| 30 | 否 | — | — | — | 98% | insufficient excitation (speed std < 0.1 m/s) or data |
| 60 | 否 | — | — | — | 94% | insufficient excitation (speed std < 0.1 m/s) or data |
| 90 | 否 | — | — | — | 95% | insufficient excitation (speed std < 0.1 m/s) or data |
| 120 | 否 | — | — | — | 95% | insufficient excitation (speed std < 0.1 m/s) or data |
| 150 | 否 | — | — | — | 89% | GNSS velocity covers 89% of the window (< 90%) |
| 180 | 否 | 0.113 | — | 1123.5 | 93% | 峰在搜索边界 |
| 210 | 否 | — | — | — | 100% | insufficient excitation (speed std < 0.1 m/s) or data |
| 240 | 否 | — | — | — | 100% | insufficient excitation (speed std < 0.1 m/s) or data |
| 270 | 否 | — | — | — | 100% | insufficient excitation (speed std < 0.1 m/s) or data |
| 300 | 否 | — | — | — | 100% | insufficient excitation (speed std < 0.1 m/s) or data |
| 330 | 否 | — | — | — | 100% | insufficient excitation (speed std < 0.1 m/s) or data |
| 360 | 否 | — | — | — | 100% | insufficient excitation (speed std < 0.1 m/s) or data |
| 390 | 否 | — | — | — | 100% | insufficient excitation (speed std < 0.1 m/s) or data |
| 420 | 否 | — | — | — | 100% | insufficient excitation (speed std < 0.1 m/s) or data |
| 450 | 否 | — | — | — | 100% | insufficient excitation (speed std < 0.1 m/s) or data |
| 480 | 否 | — | — | — | 83% | GNSS velocity covers 83% of the window (< 90%) |
| 510 | 否 | — | — | — | 83% | GNSS velocity covers 83% of the window (< 90%) |
| 540 | 是 | 0.994 | 6.8 | 109.6 | 100% |  |

互相关给出的段(写入文件的以状态行为准):

| t_from_gpst_s | offset_s | drift_ppm | t_ref_gpst_s | sigma_toff_s | tdcp_paused | method | peak_corr | window_scatter_ms | n_windows | n_windows_rejected | n_windows_isolated | bootstrap_sd_ms | peak_width_sigma_ms | dtw_offset_s | dtw_consistent |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 0 | 20.18 ms | -4.478 | 1737276883.176 | 70.67 ms | false | xcorr | 0.9963 | 17.2 | 2 | 0 | 0 | -1 | 70.67 | 120.18 ms | true |

方法③ DTW 交叉核对:DTW 偏移 120.18 ms(路径偏移 MAD 800.0 ms)对 xcorr 20.18 ms,一致(>3σ 才告警)。

警告:window scatter 17.2 ms > 5 ms in segment from 0.0;segment from 0.0 has 2 window(s): sigma 71 ms。

标定文件(方法②) `time_calib/GNSS-denial01.yaml`(AMENDMENTS A18 格式;所有 `*_gpst_s` 为 GTime::toSec(),1970 原点、GPST 尺度):

```yaml
sequence: GNSS-denial01
tool_version: 3
leap_s: 18
segments:
  - t_from_gpst_s: 0
    offset_s: 0.020175571138389803
    drift_ppm: -4.47837132333502
    t_ref_gpst_s: 1737276883.1764746
    sigma_toff_s: 0.07067414322468003
    tdcp_paused: false
    method: xcorr
    peak_corr: 0.9963373488283407
    window_scatter_ms: 17.199223873215924
    n_windows: 2
    n_windows_rejected: 0
    n_windows_isolated: 0
    bootstrap_sd_ms: -1.0
    peak_width_sigma_ms: 70.67414322468004
    short_segment: true
    drift_source: method1_prior
    dtw_offset_s: 0.12017557113838981
    dtw_consistent: true
```
