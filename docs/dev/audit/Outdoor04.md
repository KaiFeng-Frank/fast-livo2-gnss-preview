> Historical research record (through 2026-10-03). Paths are illustrative; archived runs are not bundled. For current status and executable entry points see [the repository README](../../../README.md). Current release docs/LICENSING.md supersedes historical license notes. This record does not authorize new experiments or assert current release validation.

# I2 数据审计 — Outdoor04

生成:`tools/m3dgr_audit.py`(tool_version 3),2026-09-26T11:47:28Z;bag `/work/fastlivo2_gnss/data/m3dgr/fetch/Outdoor04/x/Outdoor04.bag`(14349022307 字节,2025-07-31T16:31:18Z;sha256 未计算)。序列类别:**evaluation**(AMENDMENTS A2)。

> 评测集:本报告只做格式/解码审计。没有读取任何估计轨迹;不做 GNSS-only 解对接收机 PVT 的比较(§12.6-5 自检 G12a/b)与杆臂方向(§12.6-7 自检 G13),两者在评测集上 SKIP(A2);G11 只比较多普勒两种符号假设的拟合残差(解码性质)。

总评:**WARN**(INFO 9, PASS 18, SKIP 4, WARN 3)。远端归档:`/work/fastlivo2_gnss/audit/Outdoor04`。

## 本轮说明

评测集(A2),AMENDMENTS A68(2026-09-26,tool_version 3)。原包 = 阿里云盘 `/来自分享/Dataset/Standard/Outdoor04`(5 段分卷,cat 拼接后 7z 解 SFX),sha256 与官方一致,已索引;审计后删除。

过滤后 `data/m3dgr/filtered/Outdoor04.bag`(4.42 GiB,sha256 `bf0b1881de4b269a…`;保留 Avia LiDAR/IMU、压缩彩色图、`/ublox_driver/*`、RTK 定位与 `/odom`,逐消息核对通过)。BRDC `BRDC00IGS_R_20250030000_01D_MN.rnx`。时长 782.8 s。

**只做格式/解码审计、GT 自身审计与时间标定**:没有读取任何估计轨迹对真值的比较;自检 5(G12a/b)、自检 7(G13)与 GT 审计的回放路线 P4c 在评测集上不计算。方法②只比较 U 臂 GNSS-free 回放(A55,HEAD 零通道,A64 评测构建 `wt/a64-strict`,种子 0,`runs/eval/preproc_Outdoor04/Outdoor04/U/s0`)的天线水平速度与多普勒 LS 水平速度。GT 审计在过滤后的包上重跑,产物与在原包上的逐字节相同。

**时间标定**:method xcorr +26.9 ms、σ_toff 26.5 ms、漂移 -7.20 ppm(3 窗,峰值相关 0.990);TDCP 不暂停(A53)。

**GT 时基(A54)**:P4 WARN ⇒ method chain +97.0 ms,补偿 +97.0 ms,置信 high;`ate_exclude` 0.0 s,RTK 跳变平台 0 个。

## 供 WP-a / 核心使用的结论

- `gnss.ephem.ura_semantics: metres`(A16;判定依据见 E4)。
- std 单位(§12.6-3):psr_std=metres,cp_std=cycles,dopp_std=hertz;cp_std=0.06 是 u-blox cpStdev=15 的『无效』标记,占 57.4%。
- 星历覆盖(A1):卫星-历元覆盖率 仅 bag 72.1% / bag+RINEX 92.5%;≥5 星历元比例 99.4% / 100.0% ⇒ 必须配置 `gnss.nav_rinex`。
- Klobuchar(A38):GPS 参数取 RINEX 头 GPSA/GPSB(本日可用);bag 的 iono_params 只记诊断,E6 判为 BDS-like(发布节奏 4.1 s,与 BDS D1 子帧 1 在各北斗星上轮播相符)。
- BDS 星历字段(E1 对拍 184 条):toe(GnssTimeMsg)是 GPST(184/184);`toe_tow` 字段是 BDT 周内秒的只有 0 条、`week` 字段是 GPS 周的 184 条 ⇒ 核心须按 §12.3 由 GPST toe 重算 toes(BDT)与 BDT 周,不得直接用消息的 toe_tow/week。
- 观测频点字段与信号码不一致(G3):E code 28 (L7Q (E5bQ), 1207.14 MHz) reported at 1227.6000 MHz: 50965 slots ⇒ ublox_driver 把这些槽的 freqs 写错;按 §12.3 频点表它们被拒收,主信号不受影响,只失去该系统的第二频点 GF 周跳检测(如需使用,可在 gnss_io 按 code 映射,需维护者决定)。
- 星历流一致性(E7):1 个 (sat,toe,IODE) 键出现内容不同的副本(C06:OMGd)⇒ EphemStore 对同键新副本应比对内容,不一致即计数并保留先到/RINEX 版本,不能后到覆盖。
- C/N0 中位 BDS 27, GAL 24, GLO 29, GPS 27 dB-Hz;按 DESIGN §10 L0 掩码(伪距 30 / 多普勒 25 / 载波 28 dB-Hz)可用率 伪距 BDS 10%, GAL 2%, GLO 0%, GPS 7%、多普勒 BDS 77%, GAL 47%, GLO 84%, GPS 74% ⇒ 默认掩码会饿死滤波与初始化(COLLECT 需 ≥5 星 SPP);掩码必须在开发集按 TEST_PLAN §3 重新标定,不得在本序列上调。
- time_pulse_info 话题不存在 ⇒ A19 的检查无对象;主机钟台阶只能从 range_meas 记录时间序列看(L1)。
- range_meas 记录时间 − GPST→UTC:中位 -42.8 ms、MAD 17.8 ms、漂移 -127.5 ppm、台阶 无(= 主机钟偏差+到达延迟;作 §7 方法①先验,不是最终时偏)。
- 传感器时基(C4,记录时间 − 头戳中位,漂移):Avia LiDAR 9.5 ms / -122 ppm、Avia IMU -48.3 ms / -114 ppm、相机 45.4 ms。Livox 头戳 = 设备时间戳 + 常数(S1:timebase on another clock (offset 1.73589e+09 s, drift 0.0 ppm vs header)),与主机时钟的关系逐序列不同(本序列与『扫描起点主机时刻 + 约 100 ms 扫描』相容) ⇒ 估计器时基(LiDAR 头戳)对 GPST 的偏差与漂移只能由 §7 方法②逐序列标定;range_meas 记录时间只作先验。
- 载波/多普勒符号(§12.6-2):斜率 1.0000、相关 1.00000 ⇒ `λ·dΦ/dt = −λD` 成立,与 DESIGN §1 一致。
- Livox:/livox/avia/lidar livox_ros_driver/CustomMsg [e4d6829b], /livox/mid360/lidar livox_ros_driver2/CustomMsg [e4d6829b] 与仓库 include/livox_ros_driver 头 md5 一致。
- CompressedImage format:{'rgb8; jpeg compressed bgr8': 23470};回放直读应 `IMREAD_COLOR` 并按 format 决定通道序(S2)。S2 的 PASS 是 Python 对两条解码路径的仿真,C++ 回放路径本身待 WP-f 实测。
- §12.6-7 杆臂方向:评测集不做(A2,标定级判断只在开发集);calibration.md 方向推导见 docs/dev/audit/m3dgr_calib.md。
- 自检 1 的 sat2freq 交叉核对(A35-C8,G14):全部一致。

## 检查表

| # | check | verdict | evidence |
|---|---|---|---|
| C1 | msg_def md5 recomputed = connection md5 (and = installed ROS type md5 where installed) | **PASS** | 16 connections, 0 mismatches; 9 standard types also matched the installed ROS packages |
| C2 | topics of the replay/GNSS path present (DESIGN §14; since A38 the RINEX NAV file is an input of every sequence) | **PASS** | missing: /ublox_driver/time_pulse_info (the A19 time_pulse_info checks have no object) |
| C3 | rates and time ranges (header-stamp period; record-time average) | **INFO** | lidar 10.0 Hz (header), avg 10.00 Hz; imu 200.2 Hz (header), avg 203.38 Hz; compressed 30.0 Hz (header), avg 29.98 Hz; range_meas avg 10.00 Hz (7830 msgs); ephem avg 0.44 Hz (342 msgs); glo_ephem avg 0.16 Hz (120 msgs); receiver_pvt avg 10.00 Hz (7829 msgs); iono_params avg 0.24 Hz (184 msgs); no gaps |
| C4 | header stamp vs record time per topic (median/p1/p99 ms, drift ppm) | **INFO** | /camera/color/image_raw/compressed 45.4/27.9/56.2 ms, -3 ppm; /livox/avia/imu -48.3/-137.3/25.8 ms, -114 ppm; /livox/avia/lidar 9.5/-52.2/41.2 ms, -122 ppm; /mavros/global_position/raw/fix 14.0/-1.7/124.7 ms, 3 ppm; /ublox_driver/iono_params 273.4/174.0/305.5 ms, -118 ppm |
| G1 | epoch interval 10 Hz, monotone, one time tag per epoch (§12.6-1) | **PASS** | 7830 epochs; 100.00 % at 0.1 s; 0 non-increasing; 0 epochs with >1 time tag; 0 gaps >0.15 s |
| G15 | 1 Hz decimation rule (AMENDMENTS A43: the first epoch of every GPST second; the retired DESIGN §4.3 5 ms rule reported beside it) | **PASS** | A43 rule: 784 epochs for 784 GPST seconds with data; 10 Hz epoch grid offset -7.0 ms from the GPST tenths; the retired \|tow - round(tow)\| < 5 ms rule would select an epoch in 0.0% of the 784 seconds |
| G2 | per-satellite array lengths consistent; sat numbers inside the gnss_comm ranges | **PASS** | 0 inconsistent satellites; unknown sat numbers: none |
| G3 | (sys, freq, code) histogram and freqs->code table verdicts (signal_table.hpp) | **WARN** | C 1207.1400 MHz code=27->L7I: 46958; C 1561.0980 MHz code=47->L2I: 58504; E 1227.6000 MHz code=28->Unknown: 50965; E 1575.4200 MHz code=1->L1X: 57601; G 1227.6000 MHz code=17->L2X: 24318; G 1575.4200 MHz code=1->L1C: 55973; R 1244.2500 MHz code=14->L2C: 78; R 1244.6875 MHz code=14->L2C: 7830; R 1245.5625 MHz code=14->L2C: 7819; R 1246.0000 MHz code=14->L2C: 7805; R 1247.7500 MHz code=14->L2C: 7830; R 1599.7500 MHz code=1->L1C: 202; R 1600.3125 MHz code=1->L1C: 7830; R 1601.4375 MHz code=1->L1C: 7830; R 1602.0000 MHz code=1->L1C: 7830; R 1603.6875 MHz code=1->L1C: 5577; R 1604.2500 MHz code=1->L1C: 7830; verdicts {'Accepted': 311815, 'RejectedUnknownFreq': 50965}; primary slot first: {'slot0': 209177, 'none': 5220}; freqs field inconsistent with the code field: E code 28 (L7Q (E5bQ), 1207.14 MHz) reported at 1227.6000 MHz: 50965 slots (the table rejects these slots; only the second-frequency GF slip check loses them) |
| G4 | satellites per epoch (primary signal present) | **INFO** | total min/mean/max 22/26.7/35; BDS 6/7.5/8, GAL 6/7.4/10, GPS 6/7.1/11, GLO 4/4.7/6 |
| G5 | C/N0 distribution (dB-Hz, primary signal) | **INFO** | BDS median 27 p5 20 p95 31; GAL median 24 p5 19 p95 29; GLO median 29 p5 18 p95 31; GPS median 27 p5 19 p95 31 |
| G6 | psr / cp / dopp availability (status bits + value; with the L0 C/N0 masks) | **INFO** | BDS psr 100% cp 64% dopp 100% (masked 10/39/77%); GAL psr 100% cp 34% dopp 100% (masked 2/19/47%); GLO psr 21% cp 76% dopp 100% (masked 0/65/84%); GPS psr 59% cp 63% dopp 100% (masked 7/49/74%) |
| G7 | LLI / status statistics | **INFO** | LLI {'GPS': {'0': 32299, '3': 19573, '2': 4062, '1': 39}, 'BDS': {'3': 20144, '0': 35263, '2': 3051, '1': 46}, 'GLO': {'3': 7535, '0': 27005, '2': 2502, '1': 57}, 'GAL': {'0': 20919, '3': 36659, '1': 23}}; status {'GPS': {'7': 19571, '1': 19630, '3': 4005, '15': 12767}, 'BDS': {'1': 20162, '7': 16790, '15': 18519, '3': 3033}, 'GLO': {'1': 7567, '15': 17729, '7': 9333, '3': 2370, '11': 100}, 'GAL': {'7': 20884, '1': 36659, '5': 58}} |
| G8 | std unit judgement on the u-blox grids (§12.6-3) | **PASS** | psr metres (100.0% on 0.01*2^n m); cp cycles (100.0% on 0.004n cyc, 0.0% on 0.004n*lambda m, 0.0% on 0.004n/lambda); dopp hertz (100.0% on 0.002*2^n Hz, 0.0% as m/s); cp_std=0.06 (cpStdev=15, u-blox 'invalid') on 57.4% of slots |
| G9 | carrier-phase rate vs Doppler sign/scale (§12.6-2: corr > 0.99, slope ~ 1) | **PASS** | 119301 pairs (clean (LLI bit0 = 0)): corr 1.00000, slope 1.0000, median \|diff\| 0.054 m/s; per satellite (§12.6-2 '每星', decisive = >= 50 clean pairs and range-rate std >= 2 m/s): 19/19 decisive satellites with corr > 0.99 and \|slope - 1\| < 0.02 (slopes 0.9992..1.0003), 5 not decisive; 1 s sign agreement lambda*dPhi vs dP 100.0%, vs -lambda*D*dt 100.0% (n=11477) |
| G10 | GLONASS channel k from freqs vs ephemeris frq (§12.3, §12.6-1) | **PASS** | 68461 slots; mismatching satellites none; observed without any ephemeris: none |
| G14 | self-check 1: RTKLIB sat2freq(sat, code, nav) of every accepted slot = its freqs (A35-C8) | **PASS** | 311815 slots consistent, 0 mismatching, 0 unverifiable (GLONASS without an ephemeris channel: none), 50965 slots not accepted by the table (G3); per system {'BDS': {'consistent': 105462}, 'GAL': {'consistent': 57601, 'not_accepted': 50965}, 'GPS': {'consistent': 80291}, 'GLO': {'consistent': 68461}} |
| E1 | bag Kepler ephemerides vs same-day RINEX (same sat,toe): \|dr\| < 1e-3 m, \|ddts\| < 1e-11 s (A9-3 tier 2) | **PASS** | 342/342 matched, 1 outside tier 2 (0 not explained by a corrupted copy, see E7); worst per system {'E': '2.05e-04 m / 4.66e-16 s', 'C': '3.40e+05 m / 4.42e-16 s', 'G': '2.34e-04 m / 3.16e-16 s'}; toe label shifts {'E:+0': 57, 'C:+0': 184, 'G:+0': 101}; field semantics {'E': {'n': 57, 'toe_tow_is_system_tow': 57, 'week_is_system_week': 57, 'week_is_gps_week': 57, 'toe_msg_is_gpst': 57, 'tgd0_eq': 57, 'tgd1_eq': 57, 'tgd0_eq_rnx_tgd1': 16, 'iode_eq': 57, 'svh_eq': 57, 'code_eq': 57}, 'C': {'n': 184, 'toe_tow_is_system_tow': 0, 'week_is_system_week': 0, 'week_is_gps_week': 184, 'toe_msg_is_gpst': 184, 'tgd0_eq': 184, 'tgd1_eq': 184, 'tgd0_eq_rnx_tgd1': 25, 'iode_eq': 184, 'svh_eq': 184, 'code_eq': 184}, 'G': {'n': 101, 'toe_tow_is_system_tow': 101, 'week_is_system_week': 101, 'week_is_gps_week': 101, 'toe_msg_is_gpst': 101, 'tgd0_eq': 101, 'tgd1_eq': 101, 'tgd0_eq_rnx_tgd1': 0, 'iode_eq': 101, 'svh_eq': 101, 'code_eq': 0}} |
| E7 | bag ephemeris stream consistency: all copies of one (sat, toe, IODE) identical | **WARN** | 22 keys, 1 with conflicting copies: C06 toe 2025-01-03T07:59:56.000Z IODE 1: 1/18 copies differ in OMGd (af1 ['4.47e-11']) — a store keyed by (sat, toe, IODE) that lets later copies overwrite would take a corrupted clock (A1: the RINEX entry wins on the same key) |
| E2 | bag GLONASS ephemerides vs RINEX (pos/vel/acc units, tau_n sign, channel) | **PASS** | 120/120 matched; worst \|dr\| at toe+600 s 4.95e-05 m; semantics {'n': 120, 'taun_same_sign_as_rtklib': 120, 'taun_opposite_sign': 0, 'gamn_eq': 120, 'frq_eq': 120, 'pos_eq_1mm': 120} |
| E4 | URA semantics: bag ura vs same-day RINEX SV accuracy (§12.6-4, A16) | **PASS** | decision per system {'E': 'metres', 'C': 'metres', 'G': 'metres'} -> gnss.ephem.ura_semantics: metres; counts {'E': {'n': 57, 'metres_exact': 57, 'metres_approx': 57, 'index': 0}, 'C': {'n': 184, 'metres_exact': 184, 'metres_approx': 184, 'index': 0, 'icd_formula_metres': 0}, 'G': {'n': 101, 'metres_exact': 69, 'metres_approx': 101, 'index': 0, 'icd_formula_metres': 32}}; samples (bag, rinex m) {'E': [(3.12, 3.12), (3.12, 3.12), (3.12, 3.12), (3.12, 3.12), (3.12, 3.12), (3.12, 3.12), (3.12, 3.12), (3.12, 3.12)], 'C': [(2.0, 2.0), (2.0, 2.0), (2.0, 2.0), (2.0, 2.0), (2.0, 2.0), (2.0, 2.0), (2.0, 2.0), (2.0, 2.0)], 'G': [(2.0, 2.0), (2.8284271247461903, 2.8), (2.0, 2.0), (2.0, 2.0), (2.0, 2.0), (2.0, 2.0), (2.8284271247461903, 2.8), (2.0, 2.0)]} |
| E3 | orbit code self-check: adjacent RINEX records agree at the midpoint (p95 < 10 m per system) | **PASS** | C median 0.11 m p95 0.39 m (n=260); E median 0.11 m p95 0.53 m (n=650); G median 0.37 m p95 1.49 m (n=95) |
| E5 | ephemeris coverage timeline: bag-only (causal by record time) vs bag+RINEX (A1, §12.6-8) | **INFO** | sat-epochs with a healthy ephemeris: bag-only 72.1%, bag+RINEX 92.5%; epochs with >=5: 99.4% / 100.0%; first >=5 at 4.3 / 0.0 s; BDS GEO observed none; never covered bag-only: C25, E08, E14, E27, E29, E36, G01, G09, G14, G16, G17, R02, R23 |
| E6 | Klobuchar source (A38): GPS set = RINEX header GPSA/GPSB; bag iono_params only a diagnostic (family classified) | **PASS** | GPS Klobuchar from the RINEX header GPSA/GPSB: ['2.794e-08', '2.235e-08', '-1.192e-07', '5.96e-08', '1.516e+05', '-1.966e+05', '-6.554e+04', '3.277e+05']; bag iono_params (diagnostic, not used, A38): 184 messages, 1 distinct sets, one every 4.1 s on average; set ['2.235e-08', '3.353e-07', '-2.027e-06', '2.682e-06', '1.946e+05', '-8.52e+05', '2.294e+06', '-5.898e+05']: BDS-like (distance to RINEX GPSA/GPSB 32 LSB, to the nearest RINEX BDSA/BDSB 7 LSB) |
| L1 | GNSS arrival latency: range_meas record time - GPST->UTC (leap 18 s) | **INFO** | median -42.8 ms, MAD 17.8, p1 -106.6, p99 -6.1, min -108.6, max -4.5; drift -127.5 ppm; persistent steps >50 ms: none; receiver_pvt median -15.8 ms |
| L3 | self-check 6 (time, record-time part): \|record - (GPST - leap)\| < 0.3 s (leap seconds and time system consistent) | **PASS** | median -0.0428 s with leap 18 s (a missing/extra leap second would show as ~±1 s, a GPST/UTC mix-up as ±18 s); the cross-correlation part of self-check 6 is method 2 of tools/time_calib/gnss_time_calib.py on the ref-R replay |
| L2 | receiver_lla header stamp convention | **INFO** | header.stamp - GPST(seconds, Unix origin) median 0.000 s (min 0.000, max 0.000): receiver_lla stamps are GPST without leap seconds (not ROS/UTC time) |
| T1 | time_pulse_info checks (A19: leap second, time system, gross errors, host clock steps) | **SKIP** | topic /ublox_driver/time_pulse_info absent from this bag (README §4.2 lists it); nothing to check. Host-clock steps are covered by the range_meas latency series instead (L1). |
| G11 | Doppler sign by LS fit: -lambda*D = range rate (§1) vs flipped sign (all Dopplers, el >= 5°) | **PASS** | post-fit RMS median 0.114 m/s with the design sign vs 580.2 m/s flipped (783 epochs at 1.0 Hz; linearised at the receiver PVT (evaluation sequence, A2)) |
| G12a | self-check 5 (position): SPP vs receiver PVT | **SKIP** | A2: not a development sequence — self-check 5 compares GNSS-only solutions with the receiver PVT (a calibration-grade judgement, TEST_PLAN §3: development set only); not computed |
| G12b | self-check 5 (velocity): Doppler LS vs receiver PVT | **SKIP** | A2: not a development sequence — self-check 5 compares GNSS-only solutions with the receiver PVT (a calibration-grade judgement, TEST_PLAN §3: development set only); not computed |
| G13 | lever-arm direction (§12.6-7): Doppler innovations with l vs -l on turns; regression on the omega x l term (~0; ~-2 = sign flipped) | **SKIP** | A2: not a development sequence — the lever-arm judgement uses the replay trajectory and belongs to the development set; not computed |
| S1 | Livox CustomMsg type/md5 vs the repository's livox_ros_driver headers | **PASS** | /livox/avia/lidar: livox_ros_driver/CustomMsg [e4d6829bdfe657cb6c21a746c86b21a6] vs repo livox_ros_driver/CustomMsg [e4d6829bdfe657cb6c21a746c86b21a6]; timebase on another clock (offset 1.73589e+09 s, drift 0.0 ppm vs header); points/scan median 24000 |
| S2 | CompressedImage format; replay direct decode vs republish path, bgr8 and grey pixel-exact (I2) | **PASS** | formats {'rgb8; jpeg compressed bgr8': 23470}; 150 sampled: bgr equal 150, grey equal 150, IMREAD_UNCHANGED==IMREAD_COLOR 150; shapes {'(480, 640, 3)': 150} (Python emulation of both decode paths with OpenCV; the C++ replay path itself is to be measured by WP-f) |
| S3 | GT file format (TUM: t x y z qx qy qz qw) | **WARN** | 11745 rows, columns {8: 11745}; 6610 non-increasing stamps (1663 backwards, max 57.0 us, 4947 exact duplicates); quaternion identity on 100.0% rows (orientation not provided); repeated positions 66.7%; GT stamps = /mavros/global_position/raw/fix header stamps on 100.0% of rows |

## 话题清单

| topic | type | md5 | md5 recomputed | msgs | rate [Hz] (header, else avg) | record-header median [ms] |
|---|---|---|---|---|---|---|
| /livox/mid360/imu | sensor_msgs/Imu | 6a62c6daae103f4ff57a132d6f95cec2 | yes | 156583 | 202.80 | 7.4e-02 |
| /camera/imu | sensor_msgs/Imu | 6a62c6daae103f4ff57a132d6f95cec2 | yes | 156593 | 200.27 | 4.3e-03 |
| /ublox_driver/range_meas | gnss_comm/GnssMeasMsg | 0eafeb530a3e6637722c4e296734c119 | yes | 7830 | 10.00 | — |
| /livox/avia/imu | sensor_msgs/Imu | 6a62c6daae103f4ff57a132d6f95cec2 | yes | 159187 | 200.16 | -48.3 |
| /livox/avia/lidar | livox_ros_driver/CustomMsg | e4d6829bdfe657cb6c21a746c86b21a6 | yes | 7829 | 10.02 | 9.5 |
| /camera/aligned_depth_to_color/image_raw/compressedDepth | sensor_msgs/CompressedImage | 8f7a12909da2c9d3332d540a0977563f | yes | 23462 | 30.02 | 43.7 |
| /mavros/global_position/raw/fix | sensor_msgs/NavSatFix | 2d3a8cd499b9b4a0249fb98fd05cfa48 | yes | 11745 | 15.00 | 14.0 |
| /camera/color/image_raw/compressed | sensor_msgs/CompressedImage | 8f7a12909da2c9d3332d540a0977563f | yes | 23470 | 30.02 | 45.4 |
| /cv_camera/image_raw/compressed | sensor_msgs/CompressedImage | 8f7a12909da2c9d3332d540a0977563f | yes | 11742 | 15.01 | 24.1 |
| /odom | nav_msgs/Odometry | cd5e73d190d741a2f92e81eda573aca7 | yes | 15658 | 20.00 | 8.0e-02 |
| /ublox_driver/receiver_pvt | gnss_comm/GnssPVTSolnMsg | d18171357d7a159f76d4d7c0b12fb631 | yes | 7829 | 10.00 | — |
| /ublox_driver/receiver_lla | sensor_msgs/NavSatFix | 2d3a8cd499b9b4a0249fb98fd05cfa48 | yes | 7829 | 10.00 | -18015.8 |
| /livox/mid360/lidar | livox_ros_driver2/CustomMsg | e4d6829bdfe657cb6c21a746c86b21a6 | yes | 7828 | 10.00 | 100.8 |
| /ublox_driver/glo_ephem | gnss_comm/GnssGloEphemMsg | 6893dee0d8b1c9325a6693f3488e0144 | yes | 120 | 0.16 | — |
| /ublox_driver/ephem | gnss_comm/GnssEphemMsg | b4346f99e431c870e392a438684c0fe8 | yes | 342 | 0.44 | — |
| /ublox_driver/iono_params | gnss_comm/StampedFloat64Array | fb60495edd59d3fcf90e173153ae8a9a | yes | 184 | 0.24 | 273.4 |

## 信号直方图 (sys, freq, code 字段 → signal_table)

| sys | freq [MHz] | code field | table code | count |
|---|---|---|---|---|
| C | 1207.1400 | 27 | L7I | 46958 |
| C | 1561.0980 | 47 | L2I | 58504 |
| E | 1227.6000 | 28 | Unknown | 50965 |
| E | 1575.4200 | 1 | L1X | 57601 |
| G | 1227.6000 | 17 | L2X | 24318 |
| G | 1575.4200 | 1 | L1C | 55973 |
| R | 1244.2500 | 14 | L2C | 78 |
| R | 1244.6875 | 14 | L2C | 7830 |
| R | 1245.5625 | 14 | L2C | 7819 |
| R | 1246.0000 | 14 | L2C | 7805 |
| R | 1247.7500 | 14 | L2C | 7830 |
| R | 1599.7500 | 1 | L1C | 202 |
| R | 1600.3125 | 1 | L1C | 7830 |
| R | 1601.4375 | 1 | L1C | 7830 |
| R | 1602.0000 | 1 | L1C | 7830 |
| R | 1603.6875 | 1 | L1C | 5577 |
| R | 1604.2500 | 1 | L1C | 7830 |

## 星历字段语义(bag vs 同日 RINEX,按 (sat,toe) 配对的计数)

| sys | code_eq | iode_eq | n | svh_eq | tgd0_eq | tgd0_eq_rnx_tgd1 | tgd1_eq | toe_msg_is_gpst | toe_tow_is_system_tow | week_is_gps_week | week_is_system_week |
|---|---|---|---|---|---|---|---|---|---|---|---|
| C | 184 | 184 | 184 | 184 | 184 | 25 | 184 | 184 | 0 | 184 | 0 |
| E | 57 | 57 | 57 | 57 | 57 | 16 | 57 | 57 | 57 | 57 | 57 |
| G | 0 | 101 | 101 | 101 | 101 | 0 | 101 | 101 | 101 | 101 | 101 |

## 产物

`audit.json`(全部数字)、`latency.csv`、`eph_coverage.csv`、`eph_coverage_timeline.csv`、`plot_*.png`,位于远端归档目录。

## GT 审计(tools/gt_audit.py)

生成:`tools/gt_audit.py`(tool_version 3),2026-09-26T12:16:51Z。输入:GT 文件与原始 bag 的 RTK 定位话题、9Ps 接收机 PVT、Avia IMU、轮速计(A2:不读任何估计轨迹)。

总评:**WARN**(INFO 4, PASS 3, WARN 2)。远端归档:`/work/fastlivo2_gnss/audit/Outdoor04/gt`。

### 检查表

| # | check | verdict | evidence |
|---|---|---|---|
| P0 | GT file format and cleaning | **INFO** | 11745 rows; 1663 backward stamp steps; 4307 rows reordered and 4947 duplicate-stamp rows dropped in gt_clean.tum; orientation identity on 100.0% rows (position-only GT) |
| P1 | GT provenance: stamps and projection reproduced from the RTK fix (C-RTK2HP) | **PASS** | 100.0% of GT stamps equal /mavros/global_position/raw/fix header stamps (1 us); best hypothesis 'ENU at the first GT fix (exact, WGS-84)' reproduces GT to 1.79e-03 m RMS (max 4.13e-03 m); other hypotheses: UTM zone 49 (E, N, alt) minus first 2.62 m; equirectangular (R=6378137 m) minus first 0.366 m; ENU of GCJ-02-shifted lat/lon 0.0255 m |
| P4 | GT time base vs the estimator clock (Avia IMU header) by cross-correlation; > 20 ms compensated | **WARN** | GT stamp -> Avia IMU header clock: +97.0 ms by chain GT->odom (speed) + odom->IMU (yaw rate); > 20 ms: gt_audited.tum shifted by it; record-time plausibility +62.3 ms (assumes equal transport delays; agreement within 50 ms required); antenna-lead fit (no odometry) L +65.7 ms, r_x +0.33 m (median speed 0.91 m/s, n=3672; r_x = 0 gives +299.9 ms); raw direct course-rate link +288.0 ms (peak 0.77): +191.0 ms against the chain = antenna lead r_x/v, not a clock term; links: gt_course_rate__imu_yaw_rate +288.0 ms (peak 0.772, delay precision 10.7 ms); gt_speed__odom_speed +11.0 ms (peak 0.987, delay precision 54.6 ms); odom_yaw_rate__imu_yaw_rate +86.0 ms (peak 0.856, delay precision 5.9 ms); gt_speed__pvt_speed_utc +175.5 ms (peak 0.982, delay precision 34.7 ms) |
| P2 | datum: RTK fix (GT source) lat/lon vs 9Ps WGS-84 PVT | **PASS** | median horizontal difference 9.3 m vs a GCJ-02 shift of 466 m at this location ⇒ WGS-84 (no GCJ-02 offset) |
| P3 | GT (local XYZ) vs 9Ps PVT in ENU at the GT origin: horizontal SE(2)/Sim(2) structure (ENU, no rotation/scale expected) | **PASS** | SE(2) yaw +0.991° (block-bootstrap σ 0.339°), Sim(2) scale 0.99615 (σ 0.00481); residual trend dE/dE -38.0 dN/dN +3.5 m/km; n=7827 (residual magnitudes are the 9Ps PVT noise, not a result, A2) |
| P5 | pre-registered trust mask (9Ps h_acc > 1 m or num_sv < 8; speed spikes > 3σ; gaps/stalls > 0.5 s; each ±2 s) | **WARN** | trusted 5.4% (GT-derived criteria alone: 94.8%); masked by h_acc>1 m 94.2%, num_sv<8 0.0% (receiver criteria before the ±2 s widening 93.3%), speed spikes 5.2% (15; σ 0.018 m/s, floor 0.03), gaps/stalls 0.0% (0/0); the h_acc criterion masks most rows because the 9Ps is a standalone (non-RTK) receiver: median h_acc 1.10 m — the pre-registered threshold needs a decision |
| P7 | RTK jumps / spikes (A54): fix-to-fix residual against the local velocity > max(0.5 m, 10σ); a jump and its return = displaced plateau, masked ±2 s | **INFO** | 0 jump(s) (thresholds h 0.50 / v 0.50 m; residual σ h 0.010 / v 0.027 m over 3913 fix intervals); masked 0.0 s of 782.7 s (0.00%); ATE-excluded (gaps/stalls + RTK) 0.0 s |
| P4r | GT time base used by the evaluation (AMENDMENTS A54 rule) | **INFO** | method chain: +97.0 ms (applied shift +97.0 ms, > 20 ms rule), confidence high, uncertainty 50.0 ms (peak-width precisions of the two chain links combined (Carter bound, 54.9 ms), capped by the 50 ms agreement with the record-time value that P4 PASS requires) |
| P6 | 2HP antenna lever arm | **INFO** | unknown (the C-RTK2HP antenna is not in calibration.md): an ATE floor of the order of the antenna separation on the top plate (~0.1-0.3 m, README Fig. 1); TEST_PLAN E1' estimates it jointly |

### GT 生成方式假设(对 RTK 定位逐点复算)

| hypothesis | direct RMS [m] | direct max [m] | Sim3 scale | Sim3 yaw [°] | Sim3 RMS [m] |
|---|---|---|---|---|---|
| ENU at the first GT fix (exact, WGS-84) | 0.002 | 0.004 | 1.000 | -4.08e-04 | 3.10e-04 |
| UTM zone 49 (E, N, alt) minus first | 2.616 | 4.795 | 1.000 | 1.252 | 1.20e-04 |
| equirectangular (R=6378137 m) minus first | 0.366 | 0.753 | 0.997 | -0.005 | 0.116 |
| ENU of GCJ-02-shifted lat/lon | 0.026 | 0.053 | 1.000 | 0.007 | 0.014 |

GCJ-02 在本地的绝对平移 466 m,轨迹范围内局部畸变 0.040 m。

### 时基互相关(GT 戳 t 对应其它时钟 t + lag)

| reference | lag [ms] | peak corr | peak width [ms] | delay precision [ms] |
|---|---|---|---|---|
| gt_course_rate__imu_yaw_rate | 287.996 | 0.772 | 800.000 | 10.677 |
| gt_speed__odom_speed | 10.986 | 0.987 | 1390.000 | 54.565 |
| odom_yaw_rate__imu_yaw_rate | 86.006 | 0.856 | 800.000 | 5.875 |
| gt_speed__pvt_speed_utc | 175.467 | 0.982 | 1540.000 | 34.680 |

### 天线超前模型(P4 诊断,不经轮速计)

航向角速率 = ω(t+L) + r_x·d/dt(ω/v):L = +65.7 ms,r_x = +0.33 m(车速中位 0.91 m/s,n = 3672);r_x 置 0 时 L = +299.9 ms。天线在无侧滑点前方 r_x 处时 GT 航向**领先**车头 r_x/v,原始直接链路相对链式结果为正即此效应,不是钟差。

### 评测用 GT 时基(AMENDMENTS A54,`gt_timebase.yaml`)

规则:P4 通过 ⇒ 轮速计链;P4 失败 ⇒ 记录时间旁证;两者都没有 ⇒ 0 并标记;补偿按 TEST_PLAN §4(|值| > 20 ms 才补偿)。

| method | value [ms] | applied shift [ms] | confidence | uncertainty [ms] | P4 | chain [ms] | record time [ms] | sub-threshold chain [ms] | P4c replay (dev only, not used) [ms] |
|---|---|---|---|---|---|---|---|---|---|
| chain | +97.0 | +97.0 | high | 50.0 | WARN | +97.0 | +62.3 | — | — |

### RTK 跳变/毛刺与被掩时长(AMENDMENTS A54)

逐定位差分对局部速度(前后各 3 个定位区间的中位速度)的残差;跳变 = 水平 > 0.50 m 或垂直 > 0.50 m(max(0.5 m, 10σ),σ 水平 0.010 / 垂直 0.027 m);一对相互抵消的跳变之间为错位平台,连同前后 2 s 掩掉(bit 64)。

无跳变。

| criterion | masked [s] | of bag span [%] |
|---|---|---|
| h_acc>1m | 729.298 | 93.179 |
| num_sv<8 | 0.000 | 0.000 |
| speed_spike | 42.006 | 5.367 |
| gap_or_stall | 0.000 | 0.000 |
| no_pvt_within_0.2s | 0.000 | 0.000 |
| rtk_jump_plateau | 0.000 | 0.000 |
| ate_exclude | 0.000 | 0.000 |
| pre_registered_all | 733.298 | 93.690 |

bag 区间内 GT 时长 782.7 s;`ate_exclude` = 间断/冻结 + RTK 跳变平台(bit 8|64;bag 区间外 bit 32 本就不参评),区间见 `gt_mask_intervals.csv`(GT 戳与审计后时基两列)。h_acc/num_sv/速度尖峰(bit 1/2/4)只报告,不标 ATE 排除(未裁定)。

### 产物

`gt_clean.tum`、`gt_audited.tum`(= gt_clean + 0.097 s,方法 chain)、`gt_timebase.yaml`、`trust_mask.csv`、`gt_mask_intervals.csv`、`gt_audit.json`。

## 时间标定(tools/time_calib/gnss_time_calib.py,DESIGN §7)

### 方法①(记录时间统计与估计器时钟先验)

range_meas 记录时间 − GPST→UTC(leap 18 s,主机钟视角 = 主机钟偏差 + GNSS 传输延迟):中位 -42.8 ms、MAD 17.8 ms、p5/p95 -101.1/-11.3 ms、漂移 -127.5 ppm、持续台阶(>50 ms)无。

头戳 − 记录时间(判定 LiDAR 时基):/camera/color/image_raw/compressed 中位 -45.4 ms(p1 -56.2 / p99 -27.9,漂移 2.7 ppm);/livox/avia/imu 中位 48.3 ms(p1 -25.8 / p99 137.3,漂移 114.2 ppm);/livox/avia/lidar 中位 -9.5 ms(p1 -41.2 / p99 52.2,漂移 121.5 ppm)。

估计器时钟(Livox 头戳)先验:GNSS「记录 − UTC」下包络 -43.9 ms 减 Avia IMU「记录 − 头戳」下包络 -85.3 ms ⇒ 上界 41.9 ms(主机钟台阶在两者中抵消;漂移 -7.2 ppm;估计器时钟台阶 无);GNSS 传输延迟不可观,按 [0, 150] ms 均匀(推断)⇒ 先验 -33.1 ms、σ 75 ms。

方法①文件 `Outdoor04_record_time.yaml`(不读回放;method record_time,σ 75 ms;tool_version ≥ 3 按 AMENDMENTS A53 TDCP 不暂停,tool_version ≤ 2 的文件仍写着暂停):

```yaml
sequence: Outdoor04
tool_version: 3
leap_s: 18
segments:
  - t_from_gpst_s: 0
    offset_s: -0.0330690588778584
    drift_ppm: -7.195461699171444
    t_ref_gpst_s: 1735891854.243
    sigma_toff_s: 0.075
    tdcp_paused: false
    method: record_time
```

### 方法②(水平天线速度互相关)与方法③(DTW)

输入:回放轨迹 `/work/fastlivo2_gnss/runs/eval/preproc_Outdoor04/Outdoor04/U/s0/traj_W.tum`(tum,23463 个速度样本,估计器速度 std 0.21 m/s);多普勒 LS 速度 7830 个历元。窗口 25 个,可用 5 个(窗长/步长见 JSON)。状态:ran。

信号(tool_version 3):horizontal antenna speed = |horizontal part of the zero-phase Gaussian-smoothed (0.25 s) velocity vector| on both sides, 100 Hz grid;W 的竖直方向:state_trace gravity (median g = [-0.3514, -0.1793, -9.7991])。多普勒 LS:C/N0 ≥ 15.0 dB-Hz、仰角 ≥ 10.0°,按接收机 dopp_std 加权,两遍(第二遍固定平滑后的钟漂):7830 个历元有 ≥ 4 行,第二遍解出 7830 个(每历元用行中位 24.0)。

| 窗起点 [s] | 用 | 峰值相关 | 偏移 [ms] | 峰宽精度 [ms] | GNSS 覆盖 | 原因 |
|---|---|---|---|---|---|---|
| 0 | 是 | 0.999 | 16.4 | 61.9 | 100% |  |
| 30 | 否 | — | — | — | 100% | insufficient excitation (speed std < 0.1 m/s) or data |
| 60 | 是 | 0.991 | 12.9 | 95.6 | 100% |  |
| 90 | 是 | 0.918 | 52.2 | 130.0 | 100% |  |
| 120 | 否 | — | — | — | 100% | insufficient excitation (speed std < 0.1 m/s) or data |
| 150 | 否 | — | — | — | 100% | insufficient excitation (speed std < 0.1 m/s) or data |
| 180 | 否 | — | — | — | 100% | insufficient excitation (speed std < 0.1 m/s) or data |
| 210 | 否 | — | — | — | 100% | insufficient excitation (speed std < 0.1 m/s) or data |
| 240 | 否 | — | — | — | 100% | insufficient excitation (speed std < 0.1 m/s) or data |
| 270 | 否 | — | — | — | 100% | insufficient excitation (speed std < 0.1 m/s) or data |
| 300 | 否 | — | — | — | 100% | insufficient excitation (speed std < 0.1 m/s) or data |
| 330 | 否 | — | — | — | 100% | insufficient excitation (speed std < 0.1 m/s) or data |
| 360 | 否 | — | — | — | 100% | insufficient excitation (speed std < 0.1 m/s) or data |
| 390 | 否 | — | — | — | 100% | insufficient excitation (speed std < 0.1 m/s) or data |
| 420 | 否 | — | — | — | 100% | insufficient excitation (speed std < 0.1 m/s) or data |
| 450 | 否 | — | — | — | 100% | insufficient excitation (speed std < 0.1 m/s) or data |
| 480 | 否 | — | — | — | 100% | insufficient excitation (speed std < 0.1 m/s) or data |
| 510 | 否 | — | — | — | 100% | insufficient excitation (speed std < 0.1 m/s) or data |
| 540 | 否 | — | — | — | 100% | insufficient excitation (speed std < 0.1 m/s) or data |
| 570 | 否 | — | — | — | 100% | insufficient excitation (speed std < 0.1 m/s) or data |
| 600 | 否 | — | — | — | 100% | insufficient excitation (speed std < 0.1 m/s) or data |
| 630 | 否 | — | — | — | 100% | insufficient excitation (speed std < 0.1 m/s) or data |
| 660 | 否 | — | — | — | 100% | insufficient excitation (speed std < 0.1 m/s) or data |
| 690 | 是 | 0.948 | -254.4 | 103.7 | 100% |  |
| 720 | 是 | 0.990 | -227.9 | 140.6 | 100% |  |

互相关给出的段(写入文件的以状态行为准):

| t_from_gpst_s | offset_s | drift_ppm | t_ref_gpst_s | sigma_toff_s | tdcp_paused | method | peak_corr | window_scatter_ms | n_windows | n_windows_rejected | n_windows_isolated | bootstrap_sd_ms | peak_width_sigma_ms | dtw_offset_s | dtw_consistent |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 0 | 26.88 ms | -7.195 | 1735891583.941 | 26.52 ms | false | xcorr | 0.9897 | 21.97 | 3 | 2 | 0 | 9.85 | 26.52 | 26.88 ms | true |

方法③ DTW 交叉核对:DTW 偏移 26.88 ms(路径偏移 MAD 300.0 ms)对 xcorr 26.88 ms,一致(>3σ 才告警)。

警告:window scatter 22.0 ms > 5 ms in segment from 0.0。

标定文件(方法②) `time_calib/Outdoor04.yaml`(AMENDMENTS A18 格式;所有 `*_gpst_s` 为 GTime::toSec(),1970 原点、GPST 尺度):

```yaml
sequence: Outdoor04
tool_version: 3
leap_s: 18
segments:
  - t_from_gpst_s: 0
    offset_s: 0.026880329627115396
    drift_ppm: -7.195461699171444
    t_ref_gpst_s: 1735891583.9412622
    sigma_toff_s: 0.02652113795222992
    tdcp_paused: false
    method: xcorr
    peak_corr: 0.9897451943712128
    window_scatter_ms: 21.97214520731783
    n_windows: 3
    n_windows_rejected: 2
    n_windows_isolated: 0
    bootstrap_sd_ms: 9.84960729806143
    peak_width_sigma_ms: 26.52113795222992
    short_segment: false
    drift_source: method1_prior
    dtw_offset_s: 0.026880329627115396
    dtw_consistent: true
```
