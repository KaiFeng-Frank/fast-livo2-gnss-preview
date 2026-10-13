> Historical research record (through 2026-10-03). Paths are illustrative; archived runs are not bundled. For current status and executable entry points see [the repository README](../../../README.md). Current release docs/LICENSING.md supersedes historical license notes. This record does not authorize new experiments or assert current release validation.

# I2 数据审计 — GNSS-denial02

生成:`tools/m3dgr_audit.py`(tool_version 3),2026-09-26T12:12:10Z;bag `/work/fastlivo2_gnss/data/m3dgr/fetch/GNSS-denial02/x/GNSS-denial02.bag`(13683052916 字节,2025-01-30T07:19:03Z;sha256 未计算)。序列类别:**evaluation**(AMENDMENTS A2)。

> 评测集:本报告只做格式/解码审计。没有读取任何估计轨迹;不做 GNSS-only 解对接收机 PVT 的比较(§12.6-5 自检 G12a/b)与杆臂方向(§12.6-7 自检 G13),两者在评测集上 SKIP(A2);G11 只比较多普勒两种符号假设的拟合残差(解码性质)。

总评:**WARN**(INFO 7, PASS 20, SKIP 4, WARN 3)。远端归档:`/work/fastlivo2_gnss/audit/GNSS-denial02`。

## 本轮说明

评测集(A2),AMENDMENTS A68(2026-09-26,tool_version 3)。原包 = 阿里云盘 `/来自分享/Dataset/GNSS_Denied/GNSS-denial02`(5 段分卷,cat 拼接后 7z 解 SFX),sha256 与官方一致,已索引;审计后删除。

过滤后 `data/m3dgr/filtered/GNSS-denial02.bag`(4.17 GiB,sha256 `7d3f4e7a55d85ffb…`;保留 Avia LiDAR/IMU、压缩彩色图、`/ublox_driver/*`、RTK 定位与 `/odom`,逐消息核对通过)。BRDC `BRDC00IGS_R_20250210000_01D_MN.rnx`。时长 750.4 s。

**只做格式/解码审计、GT 自身审计与时间标定**:没有读取任何估计轨迹对真值的比较;自检 5(G12a/b)、自检 7(G13)与 GT 审计的回放路线 P4c 在评测集上不计算。方法②只比较 U 臂 GNSS-free 回放(A55,HEAD 零通道,A64 评测构建 `wt/a64-strict`,种子 0,`runs/eval/preproc_GNSS_denial02/GNSS-denial02/U/s0`)的天线水平速度与多普勒 LS 水平速度。

**时间标定**:method xcorr +52.0 ms、σ_toff 70.9 ms、漂移 -7.21 ppm(2 窗,峰值相关 0.994);TDCP 不暂停(A53)。

**GT**:ArUco 起终点(不是轨迹):11 行文本 = 3×3 旋转、平移 3 个数与 `bag_time: 750s`(与包时长 750.4 s 一致);旋转正交到 1.4e-8、行列式 1,起终点相对转角 13.2°、|t| 0.239 m;两张 640×480 JPEG。无时间戳,GT 时基与可信掩码不适用;指标为官方 `ArUco_evaluate.py` 的起终点误差(TEST_PLAN E5,A56),仓库里尚未实现。

## 供 WP-a / 核心使用的结论

- `gnss.ephem.ura_semantics: metres`(A16;判定依据见 E4)。
- std 单位(§12.6-3):psr_std=metres,cp_std=cycles,dopp_std=hertz;cp_std=0.06 是 u-blox cpStdev=15 的『无效』标记,占 44.3%。
- 星历覆盖(A1):卫星-历元覆盖率 仅 bag 73.6% / bag+RINEX 90.4%;≥5 星历元比例 81.0% / 86.2% ⇒ 必须配置 `gnss.nav_rinex`。
- Klobuchar(A38):GPS 参数取 RINEX 头 GPSA/GPSB(本日可用);bag 的 iono_params 只记诊断,E6 判为 BDS-like(发布节奏 16.4 s,与 BDS D1 子帧 1 在各北斗星上轮播相符)。
- BDS 星历字段(E1 对拍 45 条):toe(GnssTimeMsg)是 GPST(45/45);`toe_tow` 字段是 BDT 周内秒的只有 0 条、`week` 字段是 GPS 周的 45 条 ⇒ 核心须按 §12.3 由 GPST toe 重算 toes(BDT)与 BDT 周,不得直接用消息的 toe_tow/week。
- 观测频点字段与信号码不一致(G3):E code 28 (L7Q (E5bQ), 1207.14 MHz) reported at 1227.6000 MHz: 21518 slots ⇒ ublox_driver 把这些槽的 freqs 写错;按 §12.3 频点表它们被拒收,主信号不受影响,只失去该系统的第二频点 GF 周跳检测(如需使用,可在 gnss_io 按 code 映射,需维护者决定)。
- C/N0 中位 BDS 26, GAL 27, GLO 28, GPS 26 dB-Hz;按 DESIGN §10 L0 掩码(伪距 30 / 多普勒 25 / 载波 28 dB-Hz)可用率 伪距 BDS 14%, GAL 1%, GLO 28%, GPS 4%、多普勒 BDS 67%, GAL 78%, GLO 89%, GPS 67% ⇒ 默认掩码会饿死滤波与初始化(COLLECT 需 ≥5 星 SPP);掩码必须在开发集按 TEST_PLAN §3 重新标定,不得在本序列上调。
- range_meas 记录时间 − GPST→UTC:中位 -58.5 ms、MAD 20.9 ms、漂移 95.9 ppm、台阶 无(= 主机钟偏差+到达延迟;作 §7 方法①先验,不是最终时偏)。
- 传感器时基(C4,记录时间 − 头戳中位,漂移):Avia LiDAR -62.9 ms / 89 ppm、Avia IMU -108.1 ms / 88 ppm、相机 44.8 ms。Livox 头戳 = 设备时间戳 + 常数(S1:timebase on another clock (offset 1.73745e+09 s, drift 0.0 ppm vs header)),与主机时钟的关系逐序列不同(本序列 LiDAR 头戳领先主机收包时刻,不可能是主机收包戳) ⇒ 估计器时基(LiDAR 头戳)对 GPST 的偏差与漂移只能由 §7 方法②逐序列标定;range_meas 记录时间只作先验。
- 载波/多普勒符号(§12.6-2):斜率 1.0000、相关 1.00000 ⇒ `λ·dΦ/dt = −λD` 成立,与 DESIGN §1 一致。
- Livox:/livox/mid360/lidar livox_ros_driver2/CustomMsg [e4d6829b], /livox/avia/lidar livox_ros_driver/CustomMsg [e4d6829b] 与仓库 include/livox_ros_driver 头 md5 一致。
- CompressedImage format:{'rgb8; jpeg compressed bgr8': 22494};回放直读应 `IMREAD_COLOR` 并按 format 决定通道序(S2)。S2 的 PASS 是 Python 对两条解码路径的仿真,C++ 回放路径本身待 WP-f 实测。
- §12.6-7 杆臂方向:评测集不做(A2,标定级判断只在开发集);calibration.md 方向推导见 docs/dev/audit/m3dgr_calib.md。
- 自检 1 的 sat2freq 交叉核对(A35-C8,G14):全部一致。

## 检查表

| # | check | verdict | evidence |
|---|---|---|---|
| C1 | msg_def md5 recomputed = connection md5 (and = installed ROS type md5 where installed) | **PASS** | 17 connections, 0 mismatches; 9 standard types also matched the installed ROS packages |
| C2 | topics of the replay/GNSS path present (DESIGN §14; since A38 the RINEX NAV file is an input of every sequence) | **PASS** | missing: none |
| C3 | rates and time ranges (header-stamp period; record-time average) | **WARN** | lidar 10.0 Hz (header), avg 10.00 Hz; imu 200.1 Hz (header), avg 203.21 Hz; compressed 30.0 Hz (header), avg 29.98 Hz; range_meas avg 9.95 Hz (7467 msgs); ephem avg 0.14 Hz (105 msgs); glo_ephem avg 0.08 Hz (61 msgs); receiver_pvt avg 10.00 Hz (7503 msgs); iono_params avg 0.06 Hz (45 msgs); time_pulse_info avg 0.99 Hz (743 msgs); gaps (>max(3x period, 0.25 s)) as (t since start or first header [s], length [s]): {'range_meas': [(129.8, 1.699), (134.1, 1.1), (147.5, 0.9)]} |
| C4 | header stamp vs record time per topic (median/p1/p99 ms, drift ppm) | **INFO** | /camera/color/image_raw/compressed 44.8/2.2/57.4 ms, 1 ppm; /livox/avia/imu -108.1/-181.9/-33.9 ms, 88 ppm; /livox/avia/lidar -62.9/-92.3/-26.4 ms, 89 ppm; /mavros/global_position/raw/fix 10.7/-0.4/113.3 ms, -5 ppm; /ublox_driver/iono_params 247.7/193.8/303.7 ms, 25 ppm |
| G1 | epoch interval 10 Hz, monotone, one time tag per epoch (§12.6-1) | **PASS** | 7467 epochs; 99.93 % at 0.1 s; 0 non-increasing; 0 epochs with >1 time tag; 5 gaps >0.15 s |
| G15 | 1 Hz decimation rule (AMENDMENTS A43: the first epoch of every GPST second; the retired DESIGN §4.3 5 ms rule reported beside it) | **PASS** | A43 rule: 751 epochs for 751 GPST seconds with data; 10 Hz epoch grid offset +4.0 ms from the GPST tenths; the retired \|tow - round(tow)\| < 5 ms rule would select an epoch in 99.6% of the 750 seconds |
| G2 | per-satellite array lengths consistent; sat numbers inside the gnss_comm ranges | **PASS** | 0 inconsistent satellites; unknown sat numbers: none |
| G3 | (sys, freq, code) histogram and freqs->code table verdicts (signal_table.hpp) | **WARN** | C 1207.1400 MHz code=27->L7I: 22026; C 1561.0980 MHz code=47->L2I: 28364; E 1227.6000 MHz code=28->Unknown: 21518; E 1575.4200 MHz code=1->L1X: 20146; G 1227.6000 MHz code=17->L2X: 9163; G 1575.4200 MHz code=1->L1C: 34157; R 1242.9375 MHz code=14->L2C: 3269; R 1244.6875 MHz code=14->L2C: 292; R 1246.0000 MHz code=14->L2C: 1528; R 1246.8750 MHz code=14->L2C: 715; R 1247.7500 MHz code=14->L2C: 4729; R 1598.0625 MHz code=1->L1C: 5035; R 1600.3125 MHz code=1->L1C: 367; R 1600.8750 MHz code=1->L1C: 60; R 1602.0000 MHz code=1->L1C: 3721; R 1603.1250 MHz code=1->L1C: 2486; R 1604.2500 MHz code=1->L1C: 5699; verdicts {'Accepted': 141757, 'RejectedUnknownFreq': 21518}; primary slot first: {'slot0': 100035, 'none': 14063}; freqs field inconsistent with the code field: E code 28 (L7Q (E5bQ), 1207.14 MHz) reported at 1227.6000 MHz: 21518 slots (the table rejects these slots; only the second-frequency GF slip check loses them) |
| G4 | satellites per epoch (primary signal present) | **INFO** | total min/mean/max 0/13.4/26; BDS 0/3.8/8, GAL 0/2.7/6, GPS 0/4.6/9, GLO 0/2.3/5 |
| G5 | C/N0 distribution (dB-Hz, primary signal) | **INFO** | BDS median 26 p5 14 p95 31; GAL median 27 p5 19 p95 29; GLO median 28 p5 19 p95 31; GPS median 26 p5 14 p95 29 |
| G6 | psr / cp / dopp availability (status bits + value; with the L0 C/N0 masks) | **INFO** | BDS psr 100% cp 46% dopp 100% (masked 14/28/67%); GAL psr 100% cp 65% dopp 100% (masked 1/31/78%); GLO psr 100% cp 78% dopp 100% (masked 28/65/89%); GPS psr 100% cp 45% dopp 100% (masked 4/29/67%) |
| G7 | LLI / status statistics | **INFO** | LLI {'GPS': {'3': 16808, '0': 10971, '2': 6340, '1': 38}, 'BDS': {'0': 11340, '3': 14025, '2': 2966, '1': 33}, 'GLO': {'0': 11673, '3': 3041, '2': 2609, '1': 45}, 'GAL': {'0': 13486, '3': 6642, '1': 18}}; status {'GPS': {'1': 16899, '7': 6585, '3': 6189, '15': 4424, '11': 60}, 'BDS': {'7': 6598, '15': 4775, '1': 14090, '3': 2900, '11': 1}, 'GLO': {'7': 5587, '1': 3076, '3': 2493, '15': 6131, '11': 81}, 'GAL': {'7': 13456, '1': 6642, '5': 48}} |
| G8 | std unit judgement on the u-blox grids (§12.6-3) | **PASS** | psr metres (100.0% on 0.01*2^n m); cp cycles (100.0% on 0.004n cyc, 0.0% on 0.004n*lambda m, 0.0% on 0.004n/lambda); dopp hertz (100.0% on 0.002*2^n Hz, 0.0% as m/s); cp_std=0.06 (cpStdev=15, u-blox 'invalid') on 44.3% of slots |
| G9 | carrier-phase rate vs Doppler sign/scale (§12.6-2: corr > 0.99, slope ~ 1) | **PASS** | 54088 pairs (clean (LLI bit0 = 0)): corr 1.00000, slope 1.0000, median \|diff\| 0.054 m/s; per satellite (§12.6-2 '每星', decisive = >= 50 clean pairs and range-rate std >= 2 m/s): 18/18 decisive satellites with corr > 0.99 and \|slope - 1\| < 0.02 (slopes 0.9984..1.0001), 5 not decisive; 1 s sign agreement lambda*dPhi vs dP 100.0%, vs -lambda*D*dt 100.0% (n=5029) |
| G10 | GLONASS channel k from freqs vs ephemeris frq (§12.3, §12.6-1) | **PASS** | 27901 slots; mismatching satellites none; observed without any ephemeris: none |
| G14 | self-check 1: RTKLIB sat2freq(sat, code, nav) of every accepted slot = its freqs (A35-C8) | **PASS** | 141757 slots consistent, 0 mismatching, 0 unverifiable (GLONASS without an ephemeris channel: none), 21518 slots not accepted by the table (G3); per system {'BDS': {'consistent': 50390}, 'GAL': {'consistent': 20146, 'not_accepted': 21518}, 'GPS': {'consistent': 43320}, 'GLO': {'consistent': 27901}} |
| E1 | bag Kepler ephemerides vs same-day RINEX (same sat,toe): \|dr\| < 1e-3 m, \|ddts\| < 1e-11 s (A9-3 tier 2) | **PASS** | 105/105 matched, 0 outside tier 2 (0 not explained by a corrupted copy, see E7); worst per system {'G': '8.71e-05 m / 3.76e-16 s', 'E': '2.29e-04 m / 4.84e-15 s', 'C': '2.31e-04 m / 1.25e-16 s'}; toe label shifts {'G:+0': 28, 'E:+0': 32, 'C:+0': 45}; field semantics {'G': {'n': 28, 'toe_tow_is_system_tow': 28, 'week_is_system_week': 28, 'week_is_gps_week': 28, 'toe_msg_is_gpst': 28, 'tgd0_eq': 28, 'tgd1_eq': 28, 'tgd0_eq_rnx_tgd1': 0, 'iode_eq': 28, 'svh_eq': 28, 'code_eq': 0}, 'E': {'n': 32, 'toe_tow_is_system_tow': 32, 'week_is_system_week': 32, 'week_is_gps_week': 32, 'toe_msg_is_gpst': 32, 'tgd0_eq': 32, 'tgd1_eq': 32, 'tgd0_eq_rnx_tgd1': 0, 'iode_eq': 32, 'svh_eq': 32, 'code_eq': 32}, 'C': {'n': 45, 'toe_tow_is_system_tow': 0, 'week_is_system_week': 0, 'week_is_gps_week': 45, 'toe_msg_is_gpst': 45, 'tgd0_eq': 45, 'tgd1_eq': 45, 'tgd0_eq_rnx_tgd1': 20, 'iode_eq': 45, 'svh_eq': 45, 'code_eq': 45}} |
| E7 | bag ephemeris stream consistency: all copies of one (sat, toe, IODE) identical | **PASS** | 20 keys, 0 with conflicting copies |
| E2 | bag GLONASS ephemerides vs RINEX (pos/vel/acc units, tau_n sign, channel) | **PASS** | 61/61 matched; worst \|dr\| at toe+600 s 4.23e-05 m; semantics {'n': 61, 'taun_same_sign_as_rtklib': 61, 'taun_opposite_sign': 0, 'gamn_eq': 61, 'frq_eq': 61, 'pos_eq_1mm': 61} |
| E4 | URA semantics: bag ura vs same-day RINEX SV accuracy (§12.6-4, A16) | **PASS** | decision per system {'G': 'metres', 'E': 'metres', 'C': 'metres'} -> gnss.ephem.ura_semantics: metres; counts {'G': {'n': 28, 'metres_exact': 28, 'metres_approx': 28, 'index': 0, 'icd_formula_metres': 0}, 'E': {'n': 32, 'metres_exact': 32, 'metres_approx': 32, 'index': 0}, 'C': {'n': 45, 'metres_exact': 45, 'metres_approx': 45, 'index': 0, 'icd_formula_metres': 0}}; samples (bag, rinex m) {'G': [(2.0, 2.0), (2.0, 2.0), (2.0, 2.0), (2.0, 2.0), (2.0, 2.0), (2.0, 2.0), (2.0, 2.0), (2.0, 2.0)], 'E': [(3.12, 3.12), (3.12, 3.12), (3.12, 3.12), (3.12, 3.12), (3.12, 3.12), (3.12, 3.12), (3.12, 3.12), (3.12, 3.12)], 'C': [(2.0, 2.0), (2.0, 2.0), (2.0, 2.0), (2.0, 2.0), (2.0, 2.0), (2.0, 2.0), (2.0, 2.0), (2.0, 2.0)]} |
| E3 | orbit code self-check: adjacent RINEX records agree at the midpoint (p95 < 10 m per system) | **PASS** | C median 0.11 m p95 0.45 m (n=258); E median 0.09 m p95 0.38 m (n=681); G median 0.36 m p95 2.89 m (n=96) |
| E5 | ephemeris coverage timeline: bag-only (causal by record time) vs bag+RINEX (A1, §12.6-8) | **WARN** | sat-epochs with a healthy ephemeris: bag-only 73.6%, bag+RINEX 90.4%; epochs with >=5: 81.0% / 86.2%; first >=5 at 14.8 / 0.0 s; BDS GEO observed none; never covered bag-only: C06, C29, E11, E12, E16, G01, G03, G08, G13, G19, G21, R13, R18 |
| E6 | Klobuchar source (A38): GPS set = RINEX header GPSA/GPSB; bag iono_params only a diagnostic (family classified) | **PASS** | GPS Klobuchar from the RINEX header GPSA/GPSB: ['2.98e-08', '2.98e-08', '-1.192e-07', '0', '1.516e+05', '-1.638e+05', '0', '6.554e+04']; bag iono_params (diagnostic, not used, A38): 45 messages, 1 distinct sets, one every 16.4 s on average; set ['3.353e-08', '1.118e-07', '-1.073e-06', '1.55e-06', '1.229e+05', '9.83e+04', '-1.245e+06', '1.507e+06']: BDS-like (distance to RINEX GPSA/GPSB 17 LSB, to the nearest RINEX BDSA/BDSB 4 LSB) |
| L1 | GNSS arrival latency: range_meas record time - GPST->UTC (leap 18 s) | **INFO** | median -58.5 ms, MAD 20.9, p1 -88.0, p99 -23.3, min -89.4, max -22.1; drift 95.9 ppm; persistent steps >50 ms: none; receiver_pvt median -25.1 ms |
| L3 | self-check 6 (time, record-time part): \|record - (GPST - leap)\| < 0.3 s (leap seconds and time system consistent) | **PASS** | median -0.0585 s with leap 18 s (a missing/extra leap second would show as ~±1 s, a GPST/UTC mix-up as ±18 s); the cross-correlation part of self-check 6 is method 2 of tools/time_calib/gnss_time_calib.py on the ref-R replay |
| L2 | receiver_lla header stamp convention | **INFO** | header.stamp - GPST(seconds, Unix origin) median 0.000 s (min 0.000, max 0.000): receiver_lla stamps are GPST without leap seconds (not ROS/UTC time) |
| T1 | time_pulse_info checks (A19: leap second, time system, gross errors, host clock steps) | **PASS** | 743 messages ({'UTC': 743}), pulse period 5.000 s, 593 repeated announcements, 0 gross interval errors; lead of the announcement before its pulse: median 2.944 s, window [0, 5) s, integer parts [0, 1, 2, 3, 4] with phase 0.944 s (jitter 23.9 ms) => integer-second offset +0 s: consistent; record-UTC median -3002.9 ms; host steps none |
| G11 | Doppler sign by LS fit: -lambda*D = range rate (§1) vs flipped sign (all Dopplers, el >= 5°) | **PASS** | post-fit RMS median 0.090 m/s with the design sign vs 353.9 m/s flipped (646 epochs at 1.0 Hz; linearised at the receiver PVT (evaluation sequence, A2)) |
| G12a | self-check 5 (position): SPP vs receiver PVT | **SKIP** | A2: not a development sequence — self-check 5 compares GNSS-only solutions with the receiver PVT (a calibration-grade judgement, TEST_PLAN §3: development set only); not computed |
| G12b | self-check 5 (velocity): Doppler LS vs receiver PVT | **SKIP** | A2: not a development sequence — self-check 5 compares GNSS-only solutions with the receiver PVT (a calibration-grade judgement, TEST_PLAN §3: development set only); not computed |
| G13 | lever-arm direction (§12.6-7): Doppler innovations with l vs -l on turns; regression on the omega x l term (~0; ~-2 = sign flipped) | **SKIP** | A2: not a development sequence — the lever-arm judgement uses the replay trajectory and belongs to the development set; not computed |
| S1 | Livox CustomMsg type/md5 vs the repository's livox_ros_driver headers | **PASS** | /livox/avia/lidar: livox_ros_driver/CustomMsg [e4d6829bdfe657cb6c21a746c86b21a6] vs repo livox_ros_driver/CustomMsg [e4d6829bdfe657cb6c21a746c86b21a6]; timebase on another clock (offset 1.73745e+09 s, drift 0.0 ppm vs header); points/scan median 24000 |
| S2 | CompressedImage format; replay direct decode vs republish path, bgr8 and grey pixel-exact (I2) | **PASS** | formats {'rgb8; jpeg compressed bgr8': 22494}; 150 sampled: bgr equal 150, grey equal 150, IMREAD_UNCHANGED==IMREAD_COLOR 150; shapes {'(480, 640, 3)': 150} (Python emulation of both decode paths with OpenCV; the C++ replay path itself is to be measured by WP-f) |
| S3 | GT file format | **SKIP** | no --gt given |

## 话题清单

| topic | type | md5 | md5 recomputed | msgs | rate [Hz] (header, else avg) | record-header median [ms] |
|---|---|---|---|---|---|---|
| /camera/imu | sensor_msgs/Imu | 6a62c6daae103f4ff57a132d6f95cec2 | yes | 150057 | 200.11 | 0.3 |
| /odom | nav_msgs/Odometry | cd5e73d190d741a2f92e81eda573aca7 | yes | 15008 | 20.00 | 8.2e-02 |
| /livox/mid360/imu | sensor_msgs/Imu | 6a62c6daae103f4ff57a132d6f95cec2 | yes | 150067 | 202.44 | 7.4e-02 |
| /livox/mid360/lidar | livox_ros_driver2/CustomMsg | e4d6829bdfe657cb6c21a746c86b21a6 | yes | 7504 | 10.00 | 100.9 |
| /camera/aligned_depth_to_color/image_raw/compressedDepth | sensor_msgs/CompressedImage | 8f7a12909da2c9d3332d540a0977563f | yes | 22494 | 30.00 | 43.2 |
| /camera/color/image_raw/compressed | sensor_msgs/CompressedImage | 8f7a12909da2c9d3332d540a0977563f | yes | 22494 | 30.00 | 44.8 |
| /ublox_driver/range_meas | gnss_comm/GnssMeasMsg | 0eafeb530a3e6637722c4e296734c119 | yes | 7467 | 9.95 | — |
| /cv_camera/image_raw/compressed | sensor_msgs/CompressedImage | 8f7a12909da2c9d3332d540a0977563f | yes | 11255 | 15.00 | 18.5 |
| /livox/avia/imu | sensor_msgs/Imu | 6a62c6daae103f4ff57a132d6f95cec2 | yes | 152458 | 200.15 | -108.1 |
| /livox/avia/lidar | livox_ros_driver/CustomMsg | e4d6829bdfe657cb6c21a746c86b21a6 | yes | 7503 | 10.02 | -62.9 |
| /mavros/global_position/raw/fix | sensor_msgs/NavSatFix | 2d3a8cd499b9b4a0249fb98fd05cfa48 | yes | 11256 | 699050.67 | 10.7 |
| /ublox_driver/receiver_pvt | gnss_comm/GnssPVTSolnMsg | d18171357d7a159f76d4d7c0b12fb631 | yes | 7503 | 10.00 | — |
| /ublox_driver/receiver_lla | sensor_msgs/NavSatFix | 2d3a8cd499b9b4a0249fb98fd05cfa48 | yes | 7503 | 10.00 | -18025.1 |
| /ublox_driver/time_pulse_info | gnss_comm/GnssTimePulseInfoMsg | de306035e295e3c4b1e0771a216b495a | yes | 743 | 0.99 | — |
| /ublox_driver/ephem | gnss_comm/GnssEphemMsg | b4346f99e431c870e392a438684c0fe8 | yes | 105 | 0.14 | — |
| /ublox_driver/glo_ephem | gnss_comm/GnssGloEphemMsg | 6893dee0d8b1c9325a6693f3488e0144 | yes | 61 | 0.08 | — |
| /ublox_driver/iono_params | gnss_comm/StampedFloat64Array | fb60495edd59d3fcf90e173153ae8a9a | yes | 45 | 0.06 | 247.7 |

## 信号直方图 (sys, freq, code 字段 → signal_table)

| sys | freq [MHz] | code field | table code | count |
|---|---|---|---|---|
| C | 1207.1400 | 27 | L7I | 22026 |
| C | 1561.0980 | 47 | L2I | 28364 |
| E | 1227.6000 | 28 | Unknown | 21518 |
| E | 1575.4200 | 1 | L1X | 20146 |
| G | 1227.6000 | 17 | L2X | 9163 |
| G | 1575.4200 | 1 | L1C | 34157 |
| R | 1242.9375 | 14 | L2C | 3269 |
| R | 1244.6875 | 14 | L2C | 292 |
| R | 1246.0000 | 14 | L2C | 1528 |
| R | 1246.8750 | 14 | L2C | 715 |
| R | 1247.7500 | 14 | L2C | 4729 |
| R | 1598.0625 | 1 | L1C | 5035 |
| R | 1600.3125 | 1 | L1C | 367 |
| R | 1600.8750 | 1 | L1C | 60 |
| R | 1602.0000 | 1 | L1C | 3721 |
| R | 1603.1250 | 1 | L1C | 2486 |
| R | 1604.2500 | 1 | L1C | 5699 |

## 星历字段语义(bag vs 同日 RINEX,按 (sat,toe) 配对的计数)

| sys | code_eq | iode_eq | n | svh_eq | tgd0_eq | tgd0_eq_rnx_tgd1 | tgd1_eq | toe_msg_is_gpst | toe_tow_is_system_tow | week_is_gps_week | week_is_system_week |
|---|---|---|---|---|---|---|---|---|---|---|---|
| C | 45 | 45 | 45 | 45 | 45 | 20 | 45 | 45 | 0 | 45 | 0 |
| E | 32 | 32 | 32 | 32 | 32 | 0 | 32 | 32 | 32 | 32 | 32 |
| G | 0 | 28 | 28 | 28 | 28 | 0 | 28 | 28 | 28 | 28 | 28 |

## 产物

`audit.json`(全部数字)、`latency.csv`、`eph_coverage.csv`、`eph_coverage_timeline.csv`、`plot_*.png`,位于远端归档目录。

## 时间标定(tools/time_calib/gnss_time_calib.py,DESIGN §7)

### 方法①(记录时间统计与估计器时钟先验)

range_meas 记录时间 − GPST→UTC(leap 18 s,主机钟视角 = 主机钟偏差 + GNSS 传输延迟):中位 -58.5 ms、MAD 20.9 ms、p5/p95 -85.3/-23.9 ms、漂移 95.9 ppm、持续台阶(>50 ms)无。

头戳 − 记录时间(判定 LiDAR 时基):/camera/color/image_raw/compressed 中位 -44.8 ms(p1 -57.4 / p99 -2.2,漂移 -1.1 ppm);/livox/avia/imu 中位 108.1 ms(p1 33.9 / p99 181.9,漂移 -87.9 ppm);/livox/avia/lidar 中位 62.9 ms(p1 26.4 / p99 92.3,漂移 -89.2 ppm)。

估计器时钟(Livox 头戳)先验:GNSS「记录 − UTC」下包络 -59.4 ms 减 Avia IMU「记录 − 头戳」下包络 -158.4 ms ⇒ 上界 96.5 ms(主机钟台阶在两者中抵消;漂移 -7.2 ppm;估计器时钟台阶 无);GNSS 传输延迟不可观,按 [0, 150] ms 均匀(推断)⇒ 先验 21.5 ms、σ 75 ms。

方法①文件 `GNSS-denial02_record_time.yaml`(不读回放;method record_time,σ 75 ms;tool_version ≥ 3 按 AMENDMENTS A53 TDCP 不暂停,tool_version ≤ 2 的文件仍写着暂停):

```yaml
sequence: GNSS-denial02
tool_version: 3
leap_s: 18
segments:
  - t_from_gpst_s: 0
    offset_s: 0.021509753710545212
    drift_ppm: -7.211130753839346
    t_ref_gpst_s: 1737449129.104
    sigma_toff_s: 0.075
    tdcp_paused: false
    method: record_time
```

### 方法②(水平天线速度互相关)与方法③(DTW)

输入:回放轨迹 `/work/fastlivo2_gnss/runs/eval/preproc_GNSS_denial02/GNSS-denial02/U/s0/traj_W.tum`(tum,22482 个速度样本,估计器速度 std 0.20 m/s);多普勒 LS 速度 6402 个历元。窗口 23 个,可用 2 个(窗长/步长见 JSON)。状态:ran。

信号(tool_version 3):horizontal antenna speed = |horizontal part of the zero-phase Gaussian-smoothed (0.25 s) velocity vector| on both sides, 100 Hz grid;W 的竖直方向:state_trace gravity (median g = [-0.1029, -0.3291, -9.8021])。多普勒 LS:C/N0 ≥ 15.0 dB-Hz、仰角 ≥ 10.0°,按接收机 dopp_std 加权,两遍(第二遍固定平滑后的钟漂):6526 个历元有 ≥ 4 行,第二遍解出 6402 个(每历元用行中位 13.0)。

| 窗起点 [s] | 用 | 峰值相关 | 偏移 [ms] | 峰宽精度 [ms] | GNSS 覆盖 | 原因 |
|---|---|---|---|---|---|---|
| 0 | 是 | 0.999 | 81.7 | 15.9 | 100% |  |
| 30 | 否 | — | — | — | 100% | insufficient excitation (speed std < 0.1 m/s) or data |
| 60 | 否 | — | — | — | 98% | insufficient excitation (speed std < 0.1 m/s) or data |
| 90 | 否 | — | — | — | 48% | GNSS velocity covers 48% of the window (< 90%) |
| 120 | 否 | — | — | — | 30% | GNSS velocity covers 30% of the window (< 90%) |
| 150 | 否 | — | — | — | 80% | GNSS velocity covers 80% of the window (< 90%) |
| 180 | 否 | — | — | — | 100% | insufficient excitation (speed std < 0.1 m/s) or data |
| 210 | 否 | — | — | — | 100% | insufficient excitation (speed std < 0.1 m/s) or data |
| 240 | 否 | — | — | — | 100% | insufficient excitation (speed std < 0.1 m/s) or data |
| 270 | 否 | — | — | — | 100% | insufficient excitation (speed std < 0.1 m/s) or data |
| 300 | 否 | — | — | — | 100% | insufficient excitation (speed std < 0.1 m/s) or data |
| 330 | 否 | — | — | — | 100% | insufficient excitation (speed std < 0.1 m/s) or data |
| 360 | 否 | — | — | — | 100% | insufficient excitation (speed std < 0.1 m/s) or data |
| 390 | 否 | — | — | — | 96% | insufficient excitation (speed std < 0.1 m/s) or data |
| 420 | 否 | — | — | — | 80% | GNSS velocity covers 80% of the window (< 90%) |
| 450 | 否 | — | — | — | 83% | GNSS velocity covers 83% of the window (< 90%) |
| 480 | 否 | — | — | — | 87% | GNSS velocity covers 87% of the window (< 90%) |
| 510 | 否 | — | — | — | 88% | GNSS velocity covers 88% of the window (< 90%) |
| 540 | 否 | — | — | — | 64% | GNSS velocity covers 64% of the window (< 90%) |
| 570 | 否 | — | — | — | 28% | GNSS velocity covers 28% of the window (< 90%) |
| 600 | 否 | — | — | — | 64% | GNSS velocity covers 64% of the window (< 90%) |
| 630 | 否 | — | — | — | 100% | insufficient excitation (speed std < 0.1 m/s) or data |
| 660 | 是 | 0.989 | 22.3 | 125.9 | 100% |  |

互相关给出的段(写入文件的以状态行为准):

| t_from_gpst_s | offset_s | drift_ppm | t_ref_gpst_s | sigma_toff_s | tdcp_paused | method | peak_corr | window_scatter_ms | n_windows | n_windows_rejected | n_windows_isolated | bootstrap_sd_ms | peak_width_sigma_ms | dtw_offset_s | dtw_consistent |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 0 | 51.98 ms | -7.211 | 1737449113.437 | 70.91 ms | false | xcorr | 0.9937 | 38.64 | 2 | 0 | 0 | -1 | 70.91 | 151.98 ms | true |

方法③ DTW 交叉核对:DTW 偏移 151.98 ms(路径偏移 MAD 1900.0 ms)对 xcorr 51.98 ms,一致(>3σ 才告警)。

警告:window scatter 38.6 ms > 5 ms in segment from 0.0;segment from 0.0 has 2 window(s): sigma 71 ms。

标定文件(方法②) `time_calib/GNSS-denial02.yaml`(AMENDMENTS A18 格式;所有 `*_gpst_s` 为 GTime::toSec(),1970 原点、GPST 尺度):

```yaml
sequence: GNSS-denial02
tool_version: 3
leap_s: 18
segments:
  - t_from_gpst_s: 0
    offset_s: 0.05197548355810311
    drift_ppm: -7.211130753839345
    t_ref_gpst_s: 1737449113.4374838
    sigma_toff_s: 0.07090758885468573
    tdcp_paused: false
    method: xcorr
    peak_corr: 0.9936606828966068
    window_scatter_ms: 38.638351806398134
    n_windows: 2
    n_windows_rejected: 0
    n_windows_isolated: 0
    bootstrap_sd_ms: -1.0
    peak_width_sigma_ms: 70.90758885468573
    short_segment: true
    drift_source: method1_prior
    dtw_offset_s: 0.1519754835581031
    dtw_consistent: true
```
