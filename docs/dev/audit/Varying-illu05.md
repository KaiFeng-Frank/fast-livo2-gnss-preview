> Historical research record (through 2026-10-03). Paths are illustrative; archived runs are not bundled. For current status and executable entry points see [the repository README](../../../README.md). Current release docs/LICENSING.md supersedes historical license notes. This record does not authorize new experiments or assert current release validation.

# I2 数据审计 — Varying-illu05

生成:`tools/m3dgr_audit.py`(tool_version 3),2026-09-26T12:05:47Z;bag `/work/fastlivo2_gnss/data/m3dgr/fetch/Varying-illu05/x/Varying-illu05.bag`(7634964347 字节,2025-08-04T15:56:05Z;sha256 未计算)。序列类别:**evaluation**(AMENDMENTS A2)。

> 评测集:本报告只做格式/解码审计。没有读取任何估计轨迹;不做 GNSS-only 解对接收机 PVT 的比较(§12.6-5 自检 G12a/b)与杆臂方向(§12.6-7 自检 G13),两者在评测集上 SKIP(A2);G11 只比较多普勒两种符号假设的拟合残差(解码性质)。

总评:**WARN**(INFO 9, PASS 19, SKIP 3, WARN 3)。远端归档:`/work/fastlivo2_gnss/audit/Varying-illu05`。

## 本轮说明

评测集(A2),AMENDMENTS A68(2026-09-26,tool_version 3)。原包 = 阿里云盘 `/来自分享/Dataset/Visual_Challenge/Outdoor/Varying-illu05`(3 段分卷,cat 拼接后 7z 解 SFX),sha256 与官方一致,已索引;审计后删除。

过滤后 `data/m3dgr/filtered/Varying-illu05.bag`(2.89 GiB,sha256 `106aa0a5896d5c66…`;保留 Avia LiDAR/IMU、压缩彩色图、`/ublox_driver/*`、RTK 定位与 `/odom`,逐消息核对通过)。BRDC `BRDC00IGS_R_20250130000_01D_MN.rnx`。时长 491.4 s。

**只做格式/解码审计、GT 自身审计与时间标定**:没有读取任何估计轨迹对真值的比较;自检 5(G12a/b)、自检 7(G13)与 GT 审计的回放路线 P4c 在评测集上不计算。方法②只比较 U 臂 GNSS-free 回放(A55,HEAD 零通道,A64 评测构建 `wt/a64-strict`,种子 0,`runs/eval/preproc_Varying_illu05/Varying-illu05/U/s0`)的天线水平速度与多普勒 LS 水平速度。GT 审计在过滤后的包上重跑,产物与在原包上的逐字节相同。

**时间标定**:method xcorr +51.0 ms、σ_toff 11.1 ms、漂移 0.20 ppm(4 窗,峰值相关 0.997);TDCP 不暂停(A53)。

**GT 时基(A54)**:P4 FAIL ⇒ method record_time +25.7 ms,补偿 +25.7 ms,置信 medium;`ate_exclude` 0.0 s,RTK 跳变平台 0 个。

## 供 WP-a / 核心使用的结论

- `gnss.ephem.ura_semantics: metres`(A16;判定依据见 E4)。
- std 单位(§12.6-3):psr_std=metres,cp_std=cycles,dopp_std=hertz;cp_std=0.06 是 u-blox cpStdev=15 的『无效』标记,占 60.1%。
- 星历覆盖(A1):卫星-历元覆盖率 仅 bag 61.0% / bag+RINEX 100.0%;≥5 星历元比例 95.3% / 100.0% ⇒ 必须配置 `gnss.nav_rinex`。
- Klobuchar(A38):GPS 参数取 RINEX 头 GPSA/GPSB(本日可用);bag 的 iono_params 只记诊断,E6 判为 BDS-like(发布节奏 6.4 s,与 BDS D1 子帧 1 在各北斗星上轮播相符)。
- BDS 星历字段(E1 对拍 71 条):toe(GnssTimeMsg)是 GPST(71/71);`toe_tow` 字段是 BDT 周内秒的只有 0 条、`week` 字段是 GPS 周的 71 条 ⇒ 核心须按 §12.3 由 GPST toe 重算 toes(BDT)与 BDT 周,不得直接用消息的 toe_tow/week。
- 观测频点字段与信号码不一致(G3):E code 28 (L7Q (E5bQ), 1207.14 MHz) reported at 1227.6000 MHz: 15381 slots ⇒ ublox_driver 把这些槽的 freqs 写错;按 §12.3 频点表它们被拒收,主信号不受影响,只失去该系统的第二频点 GF 周跳检测(如需使用,可在 gnss_io 按 code 映射,需维护者决定)。
- 星历流一致性(E7):1 个 (sat,toe,IODE) 键出现内容不同的副本(C07:af0/af1)⇒ EphemStore 对同键新副本应比对内容,不一致即计数并保留先到/RINEX 版本,不能后到覆盖。
- C/N0 中位 BDS 25, GAL 25, GLO 26, GPS 26 dB-Hz;按 DESIGN §10 L0 掩码(伪距 30 / 多普勒 25 / 载波 28 dB-Hz)可用率 伪距 BDS 0%, GAL 0%, GLO 21%, GPS 1%、多普勒 BDS 62%, GAL 51%, GLO 65%, GPS 67% ⇒ 默认掩码会饿死滤波与初始化(COLLECT 需 ≥5 星 SPP);掩码必须在开发集按 TEST_PLAN §3 重新标定,不得在本序列上调。
- range_meas 记录时间 − GPST→UTC:中位 8.3 ms、MAD 1.2 ms、漂移 13.0 ppm、台阶 无(= 主机钟偏差+到达延迟;作 §7 方法①先验,不是最终时偏)。
- 传感器时基(C4,记录时间 − 头戳中位,漂移):Avia LiDAR 37.3 ms / 13 ppm、Avia IMU -11.7 ms / 13 ppm、相机 45.4 ms。Livox 头戳 = 设备时间戳 + 常数(S1:timebase on another clock (offset 1.73678e+09 s, drift 0.0 ppm vs header)),与主机时钟的关系逐序列不同(本序列与『扫描起点主机时刻 + 约 100 ms 扫描』相容) ⇒ 估计器时基(LiDAR 头戳)对 GPST 的偏差与漂移只能由 §7 方法②逐序列标定;range_meas 记录时间只作先验。
- 载波/多普勒符号(§12.6-2):斜率 1.0000、相关 1.00000 ⇒ `λ·dΦ/dt = −λD` 成立,与 DESIGN §1 一致。
- Livox:/livox/avia/lidar livox_ros_driver/CustomMsg [e4d6829b], /livox/mid360/lidar livox_ros_driver2/CustomMsg [e4d6829b] 与仓库 include/livox_ros_driver 头 md5 一致。
- CompressedImage format:{'rgb8; jpeg compressed bgr8': 14732};回放直读应 `IMREAD_COLOR` 并按 format 决定通道序(S2)。S2 的 PASS 是 Python 对两条解码路径的仿真,C++ 回放路径本身待 WP-f 实测。
- §12.6-7 杆臂方向:评测集不做(A2,标定级判断只在开发集);calibration.md 方向推导见 docs/dev/audit/m3dgr_calib.md。
- 自检 1 的 sat2freq 交叉核对(A35-C8,G14):全部一致。

## 检查表

| # | check | verdict | evidence |
|---|---|---|---|
| C1 | msg_def md5 recomputed = connection md5 (and = installed ROS type md5 where installed) | **PASS** | 17 connections, 0 mismatches; 9 standard types also matched the installed ROS packages |
| C2 | topics of the replay/GNSS path present (DESIGN §14; since A38 the RINEX NAV file is an input of every sequence) | **PASS** | missing: none |
| C3 | rates and time ranges (header-stamp period; record-time average) | **INFO** | lidar 10.0 Hz (header), avg 10.00 Hz; imu 200.2 Hz (header), avg 203.37 Hz; compressed 30.0 Hz (header), avg 29.98 Hz; range_meas avg 10.00 Hz (4915 msgs); ephem avg 0.21 Hz (104 msgs); glo_ephem avg 0.06 Hz (29 msgs); receiver_pvt avg 10.00 Hz (4914 msgs); iono_params avg 0.16 Hz (71 msgs); time_pulse_info avg 1.00 Hz (492 msgs); no gaps |
| C4 | header stamp vs record time per topic (median/p1/p99 ms, drift ppm) | **INFO** | /camera/color/image_raw/compressed 45.4/30.0/55.9 ms, -0 ppm; /livox/avia/imu -11.7/-61.6/37.7 ms, 13 ppm; /livox/avia/lidar 37.3/36.3/42.6 ms, 13 ppm; /mavros/global_position/raw/fix 14.0/1.2/121.0 ms, -8 ppm; /ublox_driver/iono_params 293.4/287.9/340.9 ms, 14 ppm |
| G1 | epoch interval 10 Hz, monotone, one time tag per epoch (§12.6-1) | **PASS** | 4915 epochs; 100.00 % at 0.1 s; 0 non-increasing; 0 epochs with >1 time tag; 0 gaps >0.15 s |
| G15 | 1 Hz decimation rule (AMENDMENTS A43: the first epoch of every GPST second; the retired DESIGN §4.3 5 ms rule reported beside it) | **PASS** | A43 rule: 493 epochs for 493 GPST seconds with data; 10 Hz epoch grid offset +4.0 ms from the GPST tenths; the retired \|tow - round(tow)\| < 5 ms rule would select an epoch in 100.0% of the 492 seconds |
| G2 | per-satellite array lengths consistent; sat numbers inside the gnss_comm ranges | **PASS** | 0 inconsistent satellites; unknown sat numbers: none |
| G3 | (sys, freq, code) histogram and freqs->code table verdicts (signal_table.hpp) | **WARN** | C 1207.1400 MHz code=27->L7I: 24575; C 1561.0980 MHz code=47->L2I: 32694; E 1227.6000 MHz code=28->Unknown: 15381; E 1575.4200 MHz code=1->L1X: 10509; G 1227.6000 MHz code=17->L2X: 9830; G 1575.4200 MHz code=1->L1C: 28656; R 1244.6875 MHz code=14->L2C: 651; R 1245.1250 MHz code=14->L2C: 16; R 1246.4375 MHz code=14->L2C: 562; R 1246.8750 MHz code=14->L2C: 845; R 1247.3125 MHz code=14->L2C: 1125; R 1248.1875 MHz code=14->L2C: 30; R 1248.6250 MHz code=14->L2C: 4915; R 1598.0625 MHz code=1->L1C: 201; R 1600.3125 MHz code=1->L1C: 60; R 1600.8750 MHz code=1->L1C: 79; R 1602.5625 MHz code=1->L1C: 4915; R 1604.8125 MHz code=1->L1C: 2207; R 1605.3750 MHz code=1->L1C: 4915; verdicts {'Accepted': 126785, 'RejectedUnknownFreq': 15381}; primary slot first: {'slot0': 84236, 'none': 9824}; freqs field inconsistent with the code field: E code 28 (L7Q (E5bQ), 1207.14 MHz) reported at 1227.6000 MHz: 15381 slots (the table rejects these slots; only the second-frequency GF slip check loses them) |
| G4 | satellites per epoch (primary signal present) | **INFO** | total min/mean/max 15/17.1/20; BDS 6/6.7/8, GAL 2/2.1/4, GPS 3/5.8/8, GLO 2/2.5/4 |
| G5 | C/N0 distribution (dB-Hz, primary signal) | **INFO** | BDS median 25 p5 19 p95 28; GAL median 25 p5 19 p95 27; GLO median 26 p5 18 p95 30; GPS median 26 p5 19 p95 28 |
| G6 | psr / cp / dopp availability (status bits + value; with the L0 C/N0 masks) | **INFO** | BDS psr 100% cp 30% dopp 100% (masked 0/15/62%); GAL psr 100% cp 27% dopp 100% (masked 0/2/51%); GLO psr 100% cp 47% dopp 100% (masked 21/35/65%); GPS psr 100% cp 42% dopp 100% (masked 1/16/67%) |
| G7 | LLI / status statistics | **INFO** | LLI {'GPS': {'3': 15716, '2': 3489, '0': 9427, '1': 24}, 'BDS': {'3': 22062, '0': 9263, '2': 1344, '1': 25}, 'GLO': {'3': 6158, '0': 5709, '2': 505, '1': 5}, 'GAL': {'3': 7230, '0': 3272, '1': 7}}; status {'GPS': {'1': 15751, '3': 3454, '7': 2661, '15': 6790}, 'BDS': {'1': 22085, '15': 5626, '3': 1321, '7': 3662}, 'GLO': {'1': 6174, '7': 5191, '3': 489, '15': 523}, 'GAL': {'1': 7230, '5': 16, '7': 3263}} |
| G8 | std unit judgement on the u-blox grids (§12.6-3) | **PASS** | psr metres (100.0% on 0.01*2^n m); cp cycles (100.0% on 0.004n cyc, 0.0% on 0.004n*lambda m, 0.0% on 0.004n/lambda); dopp hertz (100.0% on 0.002*2^n Hz, 0.0% as m/s); cp_std=0.06 (cpStdev=15, u-blox 'invalid') on 60.1% of slots |
| G9 | carrier-phase rate vs Doppler sign/scale (§12.6-2: corr > 0.99, slope ~ 1) | **PASS** | 29898 pairs (clean (LLI bit0 = 0)): corr 1.00000, slope 1.0000, median \|diff\| 0.052 m/s; per satellite (§12.6-2 '每星', decisive = >= 50 clean pairs and range-rate std >= 2 m/s): 11/11 decisive satellites with corr > 0.99 and \|slope - 1\| < 0.02 (slopes 0.9994..1.0001), 4 not decisive; 1 s sign agreement lambda*dPhi vs dP 100.0%, vs -lambda*D*dt 100.0% (n=2827) |
| G10 | GLONASS channel k from freqs vs ephemeris frq (§12.3, §12.6-1) | **PASS** | 20521 slots; mismatching satellites none; observed without any ephemeris: none |
| G14 | self-check 1: RTKLIB sat2freq(sat, code, nav) of every accepted slot = its freqs (A35-C8) | **PASS** | 126785 slots consistent, 0 mismatching, 0 unverifiable (GLONASS without an ephemeris channel: none), 15381 slots not accepted by the table (G3); per system {'BDS': {'consistent': 57269}, 'GAL': {'consistent': 10509, 'not_accepted': 15381}, 'GPS': {'consistent': 38486}, 'GLO': {'consistent': 20521}} |
| E1 | bag Kepler ephemerides vs same-day RINEX (same sat,toe): \|dr\| < 1e-3 m, \|ddts\| < 1e-11 s (A9-3 tier 2) | **PASS** | 104/104 matched, 1 outside tier 2 (0 not explained by a corrupted copy, see E7); worst per system {'G': '2.15e-04 m / 4.38e-16 s', 'C': '3.41e-04 m / 3.80e-06 s', 'E': '1.12e-04 m / 2.03e-17 s'}; toe label shifts {'G:+0': 25, 'C:+0': 71, 'E:+0': 8}; field semantics {'G': {'n': 25, 'toe_tow_is_system_tow': 25, 'week_is_system_week': 25, 'week_is_gps_week': 25, 'toe_msg_is_gpst': 25, 'tgd0_eq': 25, 'tgd1_eq': 25, 'tgd0_eq_rnx_tgd1': 0, 'iode_eq': 25, 'svh_eq': 25, 'code_eq': 0}, 'C': {'n': 71, 'toe_tow_is_system_tow': 0, 'week_is_system_week': 0, 'week_is_gps_week': 71, 'toe_msg_is_gpst': 71, 'tgd0_eq': 71, 'tgd1_eq': 71, 'tgd0_eq_rnx_tgd1': 10, 'iode_eq': 71, 'svh_eq': 71, 'code_eq': 71}, 'E': {'n': 8, 'toe_tow_is_system_tow': 8, 'week_is_system_week': 8, 'week_is_gps_week': 8, 'toe_msg_is_gpst': 8, 'tgd0_eq': 8, 'tgd1_eq': 8, 'tgd0_eq_rnx_tgd1': 0, 'iode_eq': 8, 'svh_eq': 8, 'code_eq': 8}} |
| E7 | bag ephemeris stream consistency: all copies of one (sat, toe, IODE) identical | **WARN** | 13 keys, 1 with conflicting copies: C07 toe 2025-01-13T13:59:56.000Z IODE 1: 1/7 copies differ in af0,af1 (af1 ['1.08e-09']) — a store keyed by (sat, toe, IODE) that lets later copies overwrite would take a corrupted clock (A1: the RINEX entry wins on the same key) |
| E2 | bag GLONASS ephemerides vs RINEX (pos/vel/acc units, tau_n sign, channel) | **PASS** | 29/29 matched; worst \|dr\| at toe+600 s 7.45e-06 m; semantics {'n': 29, 'taun_same_sign_as_rtklib': 29, 'taun_opposite_sign': 0, 'gamn_eq': 29, 'frq_eq': 29, 'pos_eq_1mm': 29} |
| E4 | URA semantics: bag ura vs same-day RINEX SV accuracy (§12.6-4, A16) | **PASS** | decision per system {'G': 'metres', 'C': 'metres', 'E': 'metres'} -> gnss.ephem.ura_semantics: metres; counts {'G': {'n': 25, 'metres_exact': 21, 'metres_approx': 25, 'index': 0, 'icd_formula_metres': 4}, 'C': {'n': 71, 'metres_exact': 71, 'metres_approx': 71, 'index': 0, 'icd_formula_metres': 0}, 'E': {'n': 8, 'metres_exact': 8, 'metres_approx': 8, 'index': 0}}; samples (bag, rinex m) {'G': [(2.0, 2.0), (2.0, 2.0), (2.0, 2.0), (2.0, 2.0), (2.0, 2.0), (2.0, 2.0), (2.0, 2.0), (2.0, 2.0)], 'C': [(2.0, 2.0), (2.0, 2.0), (2.0, 2.0), (2.0, 2.0), (2.0, 2.0), (2.0, 2.0), (2.0, 2.0), (2.0, 2.0)], 'E': [(3.12, 3.12), (3.12, 3.12), (3.12, 3.12), (3.12, 3.12), (3.12, 3.12), (3.12, 3.12), (3.12, 3.12), (3.12, 3.12)]} |
| E3 | orbit code self-check: adjacent RINEX records agree at the midpoint (p95 < 10 m per system) | **PASS** | C median 0.11 m p95 0.56 m (n=263); E median 0.10 m p95 0.46 m (n=658); G median 0.33 m p95 1.84 m (n=82) |
| E5 | ephemeris coverage timeline: bag-only (causal by record time) vs bag+RINEX (A1, §12.6-8) | **INFO** | sat-epochs with a healthy ephemeris: bag-only 61.0%, bag+RINEX 100.0%; epochs with >=5: 95.3% / 100.0%; first >=5 at 22.1 / 0.0 s; BDS GEO observed none; never covered bag-only: C28, E08, E21, G05, G09, G12, G17, G25, R09, R10, R18 |
| E6 | Klobuchar source (A38): GPS set = RINEX header GPSA/GPSB; bag iono_params only a diagnostic (family classified) | **PASS** | GPS Klobuchar from the RINEX header GPSA/GPSB: ['1.956e-08', '-7.451e-09', '-5.96e-08', '1.192e-07', '1.372e+05', '-1.638e+05', '6.554e+04', '-6.554e+04']; bag iono_params (diagnostic, not used, A38): 71 messages, 2 distinct sets, one every 6.4 s on average; set ['2.794e-08', '1.341e-07', '-1.192e-06', '1.669e-06', '1.475e+05', '-1.475e+05', '-6.554e+05', '1.18e+06']: BDS-like (distance to RINEX GPSA/GPSB 16 LSB, to the nearest RINEX BDSA/BDSB 8 LSB); set ['2.794e-08', '1.341e-07', '-1.192e-06', '1.729e-06', '2.458e+05', '-1.475e+05', '-6.554e+05', '1.18e+06']: BDS-like (distance to RINEX GPSA/GPSB 25 LSB, to the nearest RINEX BDSA/BDSB 22 LSB) |
| L1 | GNSS arrival latency: range_meas record time - GPST->UTC (leap 18 s) | **INFO** | median 8.3 ms, MAD 1.2, p1 6.3, p99 13.8, min 5.8, max 14.8; drift 13.0 ppm; persistent steps >50 ms: none; receiver_pvt median 47.1 ms |
| L3 | self-check 6 (time, record-time part): \|record - (GPST - leap)\| < 0.3 s (leap seconds and time system consistent) | **PASS** | median +0.0083 s with leap 18 s (a missing/extra leap second would show as ~±1 s, a GPST/UTC mix-up as ±18 s); the cross-correlation part of self-check 6 is method 2 of tools/time_calib/gnss_time_calib.py on the ref-R replay |
| L2 | receiver_lla header stamp convention | **INFO** | header.stamp - GPST(seconds, Unix origin) median 0.000 s (min 0.000, max 0.000): receiver_lla stamps are GPST without leap seconds (not ROS/UTC time) |
| T1 | time_pulse_info checks (A19: leap second, time system, gross errors, host clock steps) | **PASS** | 492 messages ({'UTC': 492}), pulse period 5.000 s, 393 repeated announcements, 0 gross interval errors; lead of the announcement before its pulse: median 2.938 s, window [0, 5) s, integer parts [0, 1, 2, 3, 4] with phase 0.938 s (jitter 6.0 ms) => integer-second offset +0 s: consistent; record-UTC median -2929.5 ms; host steps none |
| G11 | Doppler sign by LS fit: -lambda*D = range rate (§1) vs flipped sign (all Dopplers, el >= 5°) | **PASS** | post-fit RMS median 0.100 m/s with the design sign vs 431.1 m/s flipped (492 epochs at 1.0 Hz; linearised at the receiver PVT (evaluation sequence, A2)) |
| G12a | self-check 5 (position): SPP vs receiver PVT | **SKIP** | A2: not a development sequence — self-check 5 compares GNSS-only solutions with the receiver PVT (a calibration-grade judgement, TEST_PLAN §3: development set only); not computed |
| G12b | self-check 5 (velocity): Doppler LS vs receiver PVT | **SKIP** | A2: not a development sequence — self-check 5 compares GNSS-only solutions with the receiver PVT (a calibration-grade judgement, TEST_PLAN §3: development set only); not computed |
| G13 | lever-arm direction (§12.6-7): Doppler innovations with l vs -l on turns; regression on the omega x l term (~0; ~-2 = sign flipped) | **SKIP** | A2: not a development sequence — the lever-arm judgement uses the replay trajectory and belongs to the development set; not computed |
| S1 | Livox CustomMsg type/md5 vs the repository's livox_ros_driver headers | **PASS** | /livox/avia/lidar: livox_ros_driver/CustomMsg [e4d6829bdfe657cb6c21a746c86b21a6] vs repo livox_ros_driver/CustomMsg [e4d6829bdfe657cb6c21a746c86b21a6]; timebase on another clock (offset 1.73678e+09 s, drift 0.0 ppm vs header); points/scan median 24000 |
| S2 | CompressedImage format; replay direct decode vs republish path, bgr8 and grey pixel-exact (I2) | **PASS** | formats {'rgb8; jpeg compressed bgr8': 14732}; 150 sampled: bgr equal 150, grey equal 150, IMREAD_UNCHANGED==IMREAD_COLOR 150; shapes {'(480, 640, 3)': 150} (Python emulation of both decode paths with OpenCV; the C++ replay path itself is to be measured by WP-f) |
| S3 | GT file format (TUM: t x y z qx qy qz qw) | **WARN** | 7371 rows, columns {8: 7371}; 4351 non-increasing stamps (363 backwards, max 11.0 us, 3988 exact duplicates); quaternion identity on 100.0% rows (orientation not provided); repeated positions 66.7%; GT stamps = /mavros/global_position/raw/fix header stamps on 100.0% of rows |

## 话题清单

| topic | type | md5 | md5 recomputed | msgs | rate [Hz] (header, else avg) | record-header median [ms] |
|---|---|---|---|---|---|---|
| /camera/imu | sensor_msgs/Imu | 6a62c6daae103f4ff57a132d6f95cec2 | yes | 98292 | 200.25 | -1.1 |
| /livox/mid360/imu | sensor_msgs/Imu | 6a62c6daae103f4ff57a132d6f95cec2 | yes | 98286 | 201.98 | 7.6e-02 |
| /ublox_driver/range_meas | gnss_comm/GnssMeasMsg | 0eafeb530a3e6637722c4e296734c119 | yes | 4915 | 10.00 | — |
| /odom | nav_msgs/Odometry | cd5e73d190d741a2f92e81eda573aca7 | yes | 9829 | 20.00 | 8.1e-02 |
| /camera/aligned_depth_to_color/image_raw/compressedDepth | sensor_msgs/CompressedImage | 8f7a12909da2c9d3332d540a0977563f | yes | 14732 | 30.01 | 43.4 |
| /livox/avia/imu | sensor_msgs/Imu | 6a62c6daae103f4ff57a132d6f95cec2 | yes | 99920 | 200.16 | -11.7 |
| /livox/avia/lidar | livox_ros_driver/CustomMsg | e4d6829bdfe657cb6c21a746c86b21a6 | yes | 4914 | 10.02 | 37.3 |
| /camera/color/image_raw/compressed | sensor_msgs/CompressedImage | 8f7a12909da2c9d3332d540a0977563f | yes | 14732 | 30.01 | 45.4 |
| /ublox_driver/receiver_pvt | gnss_comm/GnssPVTSolnMsg | d18171357d7a159f76d4d7c0b12fb631 | yes | 4914 | 10.00 | — |
| /ublox_driver/receiver_lla | sensor_msgs/NavSatFix | 2d3a8cd499b9b4a0249fb98fd05cfa48 | yes | 4914 | 10.00 | -17952.9 |
| /cv_camera/image_raw/compressed | sensor_msgs/CompressedImage | 8f7a12909da2c9d3332d540a0977563f | yes | 7371 | 15.00 | 16.8 |
| /livox/mid360/lidar | livox_ros_driver2/CustomMsg | e4d6829bdfe657cb6c21a746c86b21a6 | yes | 4914 | 10.00 | 100.7 |
| /mavros/global_position/raw/fix | sensor_msgs/NavSatFix | 2d3a8cd499b9b4a0249fb98fd05cfa48 | yes | 7371 | 15.00 | 14.0 |
| /ublox_driver/time_pulse_info | gnss_comm/GnssTimePulseInfoMsg | de306035e295e3c4b1e0771a216b495a | yes | 492 | 1.00 | — |
| /ublox_driver/ephem | gnss_comm/GnssEphemMsg | b4346f99e431c870e392a438684c0fe8 | yes | 104 | 0.21 | — |
| /ublox_driver/glo_ephem | gnss_comm/GnssGloEphemMsg | 6893dee0d8b1c9325a6693f3488e0144 | yes | 29 | 0.06 | — |
| /ublox_driver/iono_params | gnss_comm/StampedFloat64Array | fb60495edd59d3fcf90e173153ae8a9a | yes | 71 | 0.16 | 293.4 |

## 信号直方图 (sys, freq, code 字段 → signal_table)

| sys | freq [MHz] | code field | table code | count |
|---|---|---|---|---|
| C | 1207.1400 | 27 | L7I | 24575 |
| C | 1561.0980 | 47 | L2I | 32694 |
| E | 1227.6000 | 28 | Unknown | 15381 |
| E | 1575.4200 | 1 | L1X | 10509 |
| G | 1227.6000 | 17 | L2X | 9830 |
| G | 1575.4200 | 1 | L1C | 28656 |
| R | 1244.6875 | 14 | L2C | 651 |
| R | 1245.1250 | 14 | L2C | 16 |
| R | 1246.4375 | 14 | L2C | 562 |
| R | 1246.8750 | 14 | L2C | 845 |
| R | 1247.3125 | 14 | L2C | 1125 |
| R | 1248.1875 | 14 | L2C | 30 |
| R | 1248.6250 | 14 | L2C | 4915 |
| R | 1598.0625 | 1 | L1C | 201 |
| R | 1600.3125 | 1 | L1C | 60 |
| R | 1600.8750 | 1 | L1C | 79 |
| R | 1602.5625 | 1 | L1C | 4915 |
| R | 1604.8125 | 1 | L1C | 2207 |
| R | 1605.3750 | 1 | L1C | 4915 |

## 星历字段语义(bag vs 同日 RINEX,按 (sat,toe) 配对的计数)

| sys | code_eq | iode_eq | n | svh_eq | tgd0_eq | tgd0_eq_rnx_tgd1 | tgd1_eq | toe_msg_is_gpst | toe_tow_is_system_tow | week_is_gps_week | week_is_system_week |
|---|---|---|---|---|---|---|---|---|---|---|---|
| C | 71 | 71 | 71 | 71 | 71 | 10 | 71 | 71 | 0 | 71 | 0 |
| E | 8 | 8 | 8 | 8 | 8 | 0 | 8 | 8 | 8 | 8 | 8 |
| G | 0 | 25 | 25 | 25 | 25 | 0 | 25 | 25 | 25 | 25 | 25 |

## 产物

`audit.json`(全部数字)、`latency.csv`、`eph_coverage.csv`、`eph_coverage_timeline.csv`、`plot_*.png`,位于远端归档目录。

## GT 审计(tools/gt_audit.py)

生成:`tools/gt_audit.py`(tool_version 3),2026-09-26T12:34:14Z。输入:GT 文件与原始 bag 的 RTK 定位话题、9Ps 接收机 PVT、Avia IMU、轮速计(A2:不读任何估计轨迹)。

总评:**FAIL**(FAIL 1, INFO 3, PASS 2, WARN 3)。远端归档:`/work/fastlivo2_gnss/audit/Varying-illu05/gt`。

### 检查表

| # | check | verdict | evidence |
|---|---|---|---|
| P0 | GT file format and cleaning | **INFO** | 7371 rows; 363 backward stamp steps; 726 rows reordered and 3988 duplicate-stamp rows dropped in gt_clean.tum; orientation identity on 100.0% rows (position-only GT) |
| P1 | GT provenance: stamps and projection reproduced from the RTK fix (C-RTK2HP) | **PASS** | 100.0% of GT stamps equal /mavros/global_position/raw/fix header stamps (1 us); best hypothesis 'ENU at the first GT fix (exact, WGS-84)' reproduces GT to 3.95e-03 m RMS (max 6.80e-03 m); other hypotheses: UTM zone 49 (E, N, alt) minus first 4.26 m; equirectangular (R=6378137 m) minus first 0.594 m; ENU of GCJ-02-shifted lat/lon 0.0821 m |
| P4 | GT time base vs the estimator clock (Avia IMU header) by cross-correlation; > 20 ms compensated | **FAIL** | no valid correlation chain (peaks: {'gt_course_rate__imu_yaw_rate': 0.63, 'gt_speed__odom_speed': 1.0, 'odom_yaw_rate__imu_yaw_rate': 0.72, 'gt_speed__pvt_speed_utc': 0.99}): GT time base undetermined by the pre-registered rule; AMENDMENTS A54: the record-time value is used instead (gt_timebase.yaml); for the record, the chain of the sub-threshold links would give +33.0 ms (delay precision 81.9 ms, peaks 1.00/0.72); record-time plausibility +25.7 ms; antenna-lead fit (no odometry) L +41.6 ms, r_x +0.31 m (median speed 0.91 m/s, n=2023; r_x = 0 gives +260.4 ms); links: gt_course_rate__imu_yaw_rate +249.7 ms (peak 0.627, delay precision 13.4 ms); gt_speed__odom_speed -9.6 ms (peak 0.996, delay precision 81.7 ms); odom_yaw_rate__imu_yaw_rate +42.6 ms (peak 0.719, delay precision 5.0 ms); gt_speed__pvt_speed_utc +115.3 ms (peak 0.992, delay precision 58.0 ms) |
| P2 | datum: RTK fix (GT source) lat/lon vs 9Ps WGS-84 PVT | **PASS** | median horizontal difference 1.8 m vs a GCJ-02 shift of 466 m at this location ⇒ WGS-84 (no GCJ-02 offset) |
| P3 | GT (local XYZ) vs 9Ps PVT in ENU at the GT origin: horizontal SE(2)/Sim(2) structure (ENU, no rotation/scale expected) | **WARN** | SE(2) yaw +0.315° (block-bootstrap σ 0.062°), Sim(2) scale 1.01073 (σ 0.00164); residual trend dE/dE +0.8 dN/dN +15.4 m/km; n=4911 (residual magnitudes are the 9Ps PVT noise, not a result, A2) |
| P5 | pre-registered trust mask (9Ps h_acc > 1 m or num_sv < 8; speed spikes > 3σ; gaps/stalls > 0.5 s; each ±2 s) | **WARN** | trusted 0.0% (GT-derived criteria alone: 94.4%); masked by h_acc>1 m 100.0%, num_sv<8 0.0% (receiver criteria before the ±2 s widening 100.0%), speed spikes 5.6% (9; σ 0.019 m/s, floor 0.03), gaps/stalls 0.0% (0/0); the h_acc criterion masks most rows because the 9Ps is a standalone (non-RTK) receiver: median h_acc 1.20 m — the pre-registered threshold needs a decision |
| P7 | RTK jumps / spikes (A54): fix-to-fix residual against the local velocity > max(0.5 m, 10σ); a jump and its return = displaced plateau, masked ±2 s | **INFO** | 0 jump(s) (thresholds h 0.50 / v 0.50 m; residual σ h 0.011 / v 0.028 m over 2451 fix intervals); masked 0.0 s of 491.2 s (0.00%); ATE-excluded (gaps/stalls + RTK) 0.0 s |
| P4r | GT time base used by the evaluation (AMENDMENTS A54 rule) | **WARN** | method record_time: +25.7 ms (applied shift +25.7 ms, > 20 ms rule), confidence medium, uncertainty 50.0 ms (the agreement P4 requires between the record-time value and a chain (A40)); P4 FAIL: record-time value (A54) |
| P6 | 2HP antenna lever arm | **INFO** | unknown (the C-RTK2HP antenna is not in calibration.md): an ATE floor of the order of the antenna separation on the top plate (~0.1-0.3 m, README Fig. 1); TEST_PLAN E1' estimates it jointly |

### GT 生成方式假设(对 RTK 定位逐点复算)

| hypothesis | direct RMS [m] | direct max [m] | Sim3 scale | Sim3 yaw [°] | Sim3 RMS [m] |
|---|---|---|---|---|---|
| ENU at the first GT fix (exact, WGS-84) | 0.004 | 0.007 | 1.000 | -3.20e-04 | 4.24e-04 |
| UTM zone 49 (E, N, alt) minus first | 4.256 | 6.300 | 1.000 | 1.254 | 4.03e-04 |
| equirectangular (R=6378137 m) minus first | 0.594 | 0.822 | 0.999 | 0.112 | 0.133 |
| ENU of GCJ-02-shifted lat/lon | 0.082 | 0.138 | 1.000 | 0.019 | 0.013 |

GCJ-02 在本地的绝对平移 466 m,轨迹范围内局部畸变 0.074 m。

### 时基互相关(GT 戳 t 对应其它时钟 t + lag)

| reference | lag [ms] | peak corr | peak width [ms] | delay precision [ms] |
|---|---|---|---|---|
| gt_course_rate__imu_yaw_rate | 249.749 | 0.627 | 720.000 | 13.409 |
| gt_speed__odom_speed | -9.617 | 0.996 | 1390.000 | 81.725 |
| odom_yaw_rate__imu_yaw_rate | 42.585 | 0.719 | 690.000 | 4.955 |
| gt_speed__pvt_speed_utc | 115.311 | 0.992 | 1490.000 | 58.031 |

### 天线超前模型(P4 诊断,不经轮速计)

航向角速率 = ω(t+L) + r_x·d/dt(ω/v):L = +41.6 ms,r_x = +0.31 m(车速中位 0.91 m/s,n = 2023);r_x 置 0 时 L = +260.4 ms。天线在无侧滑点前方 r_x 处时 GT 航向**领先**车头 r_x/v,原始直接链路相对链式结果为正即此效应,不是钟差。

### 评测用 GT 时基(AMENDMENTS A54,`gt_timebase.yaml`)

规则:P4 通过 ⇒ 轮速计链;P4 失败 ⇒ 记录时间旁证;两者都没有 ⇒ 0 并标记;补偿按 TEST_PLAN §4(|值| > 20 ms 才补偿)。

| method | value [ms] | applied shift [ms] | confidence | uncertainty [ms] | P4 | chain [ms] | record time [ms] | sub-threshold chain [ms] | P4c replay (dev only, not used) [ms] |
|---|---|---|---|---|---|---|---|---|---|
| record_time | +25.7 | +25.7 | medium | 50.0 | FAIL | — | +25.7 | +33.0 | — |

### RTK 跳变/毛刺与被掩时长(AMENDMENTS A54)

逐定位差分对局部速度(前后各 3 个定位区间的中位速度)的残差;跳变 = 水平 > 0.50 m 或垂直 > 0.50 m(max(0.5 m, 10σ),σ 水平 0.011 / 垂直 0.028 m);一对相互抵消的跳变之间为错位平台,连同前后 2 s 掩掉(bit 64)。

无跳变。

| criterion | masked [s] | of bag span [%] |
|---|---|---|
| h_acc>1m | 491.212 | 100.000 |
| num_sv<8 | 0.000 | 0.000 |
| speed_spike | 28.199 | 5.741 |
| gap_or_stall | 0.000 | 0.000 |
| no_pvt_within_0.2s | 0.000 | 0.000 |
| rtk_jump_plateau | 0.000 | 0.000 |
| ate_exclude | 0.000 | 0.000 |
| pre_registered_all | 491.212 | 100.000 |

bag 区间内 GT 时长 491.2 s;`ate_exclude` = 间断/冻结 + RTK 跳变平台(bit 8|64;bag 区间外 bit 32 本就不参评),区间见 `gt_mask_intervals.csv`(GT 戳与审计后时基两列)。h_acc/num_sv/速度尖峰(bit 1/2/4)只报告,不标 ATE 排除(未裁定)。

### 产物

`gt_clean.tum`、`gt_audited.tum`(= gt_clean + 0.026 s,方法 record_time)、`gt_timebase.yaml`、`trust_mask.csv`、`gt_mask_intervals.csv`、`gt_audit.json`。

## 时间标定(tools/time_calib/gnss_time_calib.py,DESIGN §7)

### 方法①(记录时间统计与估计器时钟先验)

range_meas 记录时间 − GPST→UTC(leap 18 s,主机钟视角 = 主机钟偏差 + GNSS 传输延迟):中位 8.3 ms、MAD 1.2 ms、p5/p95 6.7/13.3 ms、漂移 13.0 ppm、持续台阶(>50 ms)无。

头戳 − 记录时间(判定 LiDAR 时基):/camera/color/image_raw/compressed 中位 -45.4 ms(p1 -55.9 / p99 -30.0,漂移 0.3 ppm);/livox/avia/imu 中位 11.7 ms(p1 -37.7 / p99 61.6,漂移 -12.7 ppm);/livox/avia/lidar 中位 -37.3 ms(p1 -42.6 / p99 -36.3,漂移 -13.1 ppm)。

估计器时钟(Livox 头戳)先验:GNSS「记录 − UTC」下包络 7.2 ms 减 Avia IMU「记录 − 头戳」下包络 -58.0 ms ⇒ 上界 64.9 ms(主机钟台阶在两者中抵消;漂移 0.2 ppm;估计器时钟台阶 无);GNSS 传输延迟不可观,按 [0, 150] ms 均匀(推断)⇒ 先验 -10.1 ms、σ 75 ms。

方法①文件 `Varying-illu05_record_time.yaml`(不读回放;method record_time,σ 75 ms;tool_version ≥ 3 按 AMENDMENTS A53 TDCP 不暂停,tool_version ≤ 2 的文件仍写着暂停):

```yaml
sequence: Varying-illu05
tool_version: 3
leap_s: 18
segments:
  - t_from_gpst_s: 0
    offset_s: -0.01008194520636338
    drift_ppm: 0.20182128843645827
    t_ref_gpst_s: 1736778226.604
    sigma_toff_s: 0.075
    tdcp_paused: false
    method: record_time
```

### 方法②(水平天线速度互相关)与方法③(DTW)

输入:回放轨迹 `/work/fastlivo2_gnss/runs/eval/preproc_Varying_illu05/Varying-illu05/U/s0/traj_W.tum`(tum,14721 个速度样本,估计器速度 std 0.34 m/s);多普勒 LS 速度 4915 个历元。窗口 15 个,可用 5 个(窗长/步长见 JSON)。状态:ran。

信号(tool_version 3):horizontal antenna speed = |horizontal part of the zero-phase Gaussian-smoothed (0.25 s) velocity vector| on both sides, 100 Hz grid;W 的竖直方向:state_trace gravity (median g = [-0.2567, -0.1364, -9.8062])。多普勒 LS:C/N0 ≥ 15.0 dB-Hz、仰角 ≥ 10.0°,按接收机 dopp_std 加权,两遍(第二遍固定平滑后的钟漂):4915 个历元有 ≥ 4 行,第二遍解出 4915 个(每历元用行中位 17.0)。

| 窗起点 [s] | 用 | 峰值相关 | 偏移 [ms] | 峰宽精度 [ms] | GNSS 覆盖 | 原因 |
|---|---|---|---|---|---|---|
| 0 | 是 | 0.997 | 79.8 | 228.3 | 100% |  |
| 30 | 否 | — | — | — | 100% | insufficient excitation (speed std < 0.1 m/s) or data |
| 60 | 否 | — | — | — | 100% | insufficient excitation (speed std < 0.1 m/s) or data |
| 90 | 否 | — | — | — | 100% | insufficient excitation (speed std < 0.1 m/s) or data |
| 120 | 否 | — | — | — | 100% | insufficient excitation (speed std < 0.1 m/s) or data |
| 150 | 否 | — | — | — | 100% | insufficient excitation (speed std < 0.1 m/s) or data |
| 180 | 否 | — | — | — | 100% | insufficient excitation (speed std < 0.1 m/s) or data |
| 210 | 否 | — | — | — | 100% | insufficient excitation (speed std < 0.1 m/s) or data |
| 240 | 否 | — | — | — | 100% | insufficient excitation (speed std < 0.1 m/s) or data |
| 270 | 是 | 0.991 | 53.2 | 26.9 | 100% |  |
| 300 | 是 | 0.991 | 52.0 | 25.4 | 100% |  |
| 330 | 否 | — | — | — | 100% | insufficient excitation (speed std < 0.1 m/s) or data |
| 360 | 否 | — | — | — | 100% | insufficient excitation (speed std < 0.1 m/s) or data |
| 390 | 是 | 0.999 | 49.2 | 39.7 | 100% |  |
| 420 | 是 | 0.999 | 49.6 | 36.0 | 100% |  |

互相关给出的段(写入文件的以状态行为准):

| t_from_gpst_s | offset_s | drift_ppm | t_ref_gpst_s | sigma_toff_s | tdcp_paused | method | peak_corr | window_scatter_ms | n_windows | n_windows_rejected | n_windows_isolated | bootstrap_sd_ms | peak_width_sigma_ms | dtw_offset_s | dtw_consistent |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 0 | 50.97 ms | 0.2018 | 1736778312.098 | 11.11 ms | false | xcorr | 0.9972 | 1.912 | 4 | 1 | 0 | 0.8365 | 11.11 | 50.97 ms | true |

方法③ DTW 交叉核对:DTW 偏移 50.97 ms(路径偏移 MAD 400.0 ms)对 xcorr 50.97 ms,一致(>3σ 才告警)。

标定文件(方法②) `time_calib/Varying-illu05.yaml`(AMENDMENTS A18 格式;所有 `*_gpst_s` 为 GTime::toSec(),1970 原点、GPST 尺度):

```yaml
sequence: Varying-illu05
tool_version: 3
leap_s: 18
segments:
  - t_from_gpst_s: 0
    offset_s: 0.050969522229847956
    drift_ppm: 0.20182128843645827
    t_ref_gpst_s: 1736778312.098174
    sigma_toff_s: 0.011113019720543323
    tdcp_paused: false
    method: xcorr
    peak_corr: 0.9972006097686472
    window_scatter_ms: 1.9120052546306194
    n_windows: 4
    n_windows_rejected: 1
    n_windows_isolated: 0
    bootstrap_sd_ms: 0.8364742799194915
    peak_width_sigma_ms: 11.113019720543322
    short_segment: false
    drift_source: method1_prior
    dtw_offset_s: 0.050969522229847956
    dtw_consistent: true
```
