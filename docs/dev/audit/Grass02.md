> Historical research record (through 2026-10-03). Paths are illustrative; archived runs are not bundled. For current status and executable entry points see [the repository README](../../../README.md). Current release docs/LICENSING.md supersedes historical license notes. This record does not authorize new experiments or assert current release validation.

# I2 数据审计 — Grass02

生成:`tools/m3dgr_audit.py`(tool_version 3),2026-09-25T08:22:01Z;bag `/work/fastlivo2_gnss/data/m3dgr/bags/Grass02.bag`(3847442037 字节,2025-07-28T15:20:49Z;sha256 未计算)。序列类别:**development**(AMENDMENTS A2)。

总评:**FAIL**(FAIL 1, INFO 9, PASS 21, SKIP 1, WARN 2)。远端归档:`/work/fastlivo2_gnss/audit/Grass02`。

## 本轮说明

P4-TIME(2026-09-25),tool_version 3;开发集(A2/A26),全部结果可看。**回放** = HEAD 的 U 臂(AMENDMENTS A55:`gnss.enable: false`,种子 0,eval 构建 `wt/p3i-strict`,build_sha256 `36a541331a5c4210…`),`runs/eval/p4_time_u/<seq>/U/s0`;其 state_trace 与已有的 ref-R 回放逐字节相同(Grass02 `2affc187…`、Grass01 `6e262080…`、Occlusion03 `96654a26…`)。P2b 版(tool_version 2)的归档备份在远端 `wt/p4-time/home/audit_backup_p2b/<seq>/`。方法、诊断与五条序列的对照见 INTERFACE_CHANGES「WP-g → P4-TIME」。BRDC `BRDC00IGS_R_20250210000_01D_MN.rnx`。

**时间标定(§7,A53)**:P2b 时方法②峰值 0.50/0.58/0.32/0.27、σ 533 ms 不可用;**原因已查清**:不是激励不足(回放水平速度 std 0.35 m/s,与 Grass01 的 0.38 相近;只有第 3 个窗 std < 0.1 m/s),与 GT 无关,也**不是**草地颠簸的高频天线运动(P2b 的推断被推翻:C/N0 ≥ 25 dB-Hz 的多普勒行对回放天线速度的残差稳健 σ 0.10–0.11 m/s),而是 GNSS 侧速度解:C/N0 ≥ 25 只剩天顶附近的星,竖直速度与钟漂近共线,逐历元竖直噪声 1.6 m/s(真实竖直运动 0.06 m/s),3D 模被噪声抬高(均值 1.36 对 0.75 m/s)。新信号(两遍加权多普勒 LS 固定平滑钟漂 + 水平速度矢量平滑后取模)后峰值 0.979 / 0.988 / 0.865(第 3 窗激励不足),窗偏移 72.6 / 36.6 / −13.9 ms ⇒ `Grass02.yaml` = method xcorr、+31.7 ms、σ_toff 43.5 ms(只有 3 窗,取窗间离散)、漂移 0.6 ppm(方法①)、**TDCP 不暂停**。方法①先验 −13.8 ms,差 +45.6 ms(GNSS 传输延迟下限约 29 ms)。

**GT 时基(A54)**:P4 FAIL(航向角速率链路峰值 0.71 < 0.8)⇒ method record_time −5.9 ms、不补偿(≤ 20 ms)、置信低(亚阈值链 +78.9 ms 与之差 85 ms;P4c +15.4 ms)。与 P3 集成时的手取(不补偿)一致。

**RTK 跳变(A54,P7)**:+109.21…+115.41 s 一个错位平台(出跳 [−0.79, +0.27, −1.86] m,回跳 [+0.29, −0.14, +2.03] m,回跳后残差 0.54 m)——此前未被发现;掩 10.2 s(172.8 s 的 5.9%),`ate_exclude` 10.2 s。

**I2 审计(本轮重跑)**:G12b 3D 中位 0.63 m/s(水平 0.30 / 竖直 0.46,见检查表证据);G15 按 A43;G13 用 method xcorr(σ 43 ms)的时间映射重跑,仍 PASS(k = −0.31 ± 0.13;转弯行新息 RMS l 0.203 / −l 0.220 m/s)。

## 供 WP-a / 核心使用的结论

- `gnss.ephem.ura_semantics: metres`(A16;判定依据见 E4)。
- std 单位(§12.6-3):psr_std=metres,cp_std=cycles,dopp_std=hertz;cp_std=0.06 是 u-blox cpStdev=15 的『无效』标记,占 56.7%。
- 星历覆盖(A1):卫星-历元覆盖率 仅 bag 55.0% / bag+RINEX 85.8%;≥5 星历元比例 93.7% / 100.0% ⇒ 必须配置 `gnss.nav_rinex`。
- Klobuchar(A38):GPS 参数取 RINEX 头 GPSA/GPSB(本日可用);bag 的 iono_params 只记诊断,E6 判为 BDS-like(发布节奏 10.0 s,与 BDS D1 子帧 1 在各北斗星上轮播相符)。
- BDS 星历字段(E1 对拍 16 条):toe(GnssTimeMsg)是 GPST(16/16);`toe_tow` 字段是 BDT 周内秒的只有 0 条、`week` 字段是 GPS 周的 16 条 ⇒ 核心须按 §12.3 由 GPST toe 重算 toes(BDT)与 BDT 周,不得直接用消息的 toe_tow/week。
- 观测频点字段与信号码不一致(G3):E code 28 (L7Q (E5bQ), 1207.14 MHz) reported at 1227.6000 MHz: 10374 slots ⇒ ublox_driver 把这些槽的 freqs 写错;按 §12.3 频点表它们被拒收,主信号不受影响,只失去该系统的第二频点 GF 周跳检测(如需使用,可在 gnss_io 按 code 映射,需维护者决定)。
- C/N0 中位 BDS 26, GAL 26, GLO 26, GPS 25 dB-Hz;按 DESIGN §10 L0 掩码(伪距 30 / 多普勒 25 / 载波 28 dB-Hz)可用率 伪距 BDS 18%, GAL 0%, GLO 17%, GPS 4%、多普勒 BDS 64%, GAL 63%, GLO 70%, GPS 52% ⇒ 默认掩码会饿死滤波与初始化(COLLECT 需 ≥5 星 SPP);掩码必须在开发集按 TEST_PLAN §3 重新标定,不得在本序列上调。
- range_meas 记录时间 − GPST→UTC:中位 38.1 ms、MAD 2.7 ms、漂移 61.0 ppm、台阶 无(= 主机钟偏差+到达延迟;作 §7 方法①先验,不是最终时偏)。
- 传感器时基(C4,记录时间 − 头戳中位,漂移):Avia LiDAR 70.9 ms / 63 ppm、Avia IMU 21.6 ms / 56 ppm、相机 45.1 ms。Livox 头戳 = 设备时间戳 + 常数(S1:timebase on another clock (offset 1.73745e+09 s, drift 0.0 ppm vs header)),与主机时钟的关系逐序列不同(本序列与『扫描起点主机时刻 + 约 100 ms 扫描』相容) ⇒ 估计器时基(LiDAR 头戳)对 GPST 的偏差与漂移只能由 §7 方法②逐序列标定;range_meas 记录时间只作先验。
- 载波/多普勒符号(§12.6-2):斜率 1.0000、相关 1.00000 ⇒ `λ·dΦ/dt = −λD` 成立,与 DESIGN §1 一致。
- Livox:/livox/avia/lidar livox_ros_driver/CustomMsg [e4d6829b], /livox/mid360/lidar livox_ros_driver2/CustomMsg [e4d6829b] 与仓库 include/livox_ros_driver 头 md5 一致。
- CompressedImage format:{'rgb8; jpeg compressed bgr8': 5184};回放直读应 `IMREAD_COLOR` 并按 format 决定通道序(S2)。S2 的 PASS 是 Python 对两条解码路径的仿真,C++ 回放路径本身待 WP-f 实测。
- §12.6-7 杆臂方向(G13 PASS):l confirmed: k = -0.31 (block-bootstrap s.e. 0.13, 18 blocks); innovation RMS on 3565 turning rows (|yaw rate| >= 0.15 rad/s): l 0.203 / -l 0.220 / 0 0.206 m/s, lever term RMS 0.046 m/s (864 epochs, 9221 rows; fitted yaw 235.49 / 236.26 / 235.83 deg); time map xcorr (sigma 43 ms), replay /r/runs/eval/p4_time_u/Grass02/U/s0。
- 自检 1 的 sat2freq 交叉核对(A35-C8,G14):全部一致。

## 检查表

| # | check | verdict | evidence |
|---|---|---|---|
| C1 | msg_def md5 recomputed = connection md5 (and = installed ROS type md5 where installed) | **PASS** | 17 connections, 0 mismatches; 9 standard types also matched the installed ROS packages |
| C2 | topics of the replay/GNSS path present (DESIGN §14; since A38 the RINEX NAV file is an input of every sequence) | **PASS** | missing: none |
| C3 | rates and time ranges (header-stamp period; record-time average) | **INFO** | lidar 10.0 Hz (header), avg 10.00 Hz; imu 200.1 Hz (header), avg 203.37 Hz; compressed 30.0 Hz (header), avg 29.98 Hz; range_meas avg 10.00 Hz (1729 msgs); ephem avg 0.18 Hz (31 msgs); glo_ephem avg 0.13 Hz (21 msgs); receiver_pvt avg 10.00 Hz (1729 msgs); iono_params avg 0.10 Hz (16 msgs); time_pulse_info avg 1.00 Hz (173 msgs); no gaps |
| C4 | header stamp vs record time per topic (median/p1/p99 ms, drift ppm) | **INFO** | /camera/color/image_raw/compressed 45.1/29.9/54.2 ms, -21 ppm; /livox/avia/imu 21.6/-29.6/71.7 ms, 56 ppm; /livox/avia/lidar 70.9/65.9/76.6 ms, 63 ppm; /mavros/global_position/raw/fix 15.7/-0.5/113.9 ms, 1 ppm; /ublox_driver/iono_params 357.1/309.1/368.3 ms, 75 ppm |
| G1 | epoch interval 10 Hz, monotone, one time tag per epoch (§12.6-1) | **PASS** | 1729 epochs; 100.00 % at 0.1 s; 0 non-increasing; 0 epochs with >1 time tag; 0 gaps >0.15 s |
| G15 | 1 Hz decimation rule (AMENDMENTS A43: the first epoch of every GPST second; the retired DESIGN §4.3 5 ms rule reported beside it) | **PASS** | A43 rule: 174 epochs for 174 GPST seconds with data; 10 Hz epoch grid offset -0.0 ms from the GPST tenths; the retired \|tow - round(tow)\| < 5 ms rule would select an epoch in 99.4% of the 174 seconds |
| G2 | per-satellite array lengths consistent; sat numbers inside the gnss_comm ranges | **PASS** | 0 inconsistent satellites; unknown sat numbers: none |
| G3 | (sys, freq, code) histogram and freqs->code table verdicts (signal_table.hpp) | **WARN** | C 1207.1400 MHz code=27->L7I: 5357; C 1561.0980 MHz code=47->L2I: 8222; E 1227.6000 MHz code=28->Unknown: 10374; E 1575.4200 MHz code=1->L1X: 8672; G 1227.6000 MHz code=17->L2X: 4098; G 1575.4200 MHz code=1->L1C: 12565; R 1242.9375 MHz code=14->L2C: 1704; R 1244.6875 MHz code=14->L2C: 1729; R 1245.5625 MHz code=14->L2C: 145; R 1246.0000 MHz code=14->L2C: 1729; R 1247.7500 MHz code=14->L2C: 1729; R 1598.0625 MHz code=1->L1C: 1729; R 1600.3125 MHz code=1->L1C: 1729; R 1601.4375 MHz code=1->L1C: 63; R 1602.0000 MHz code=1->L1C: 1729; R 1603.1250 MHz code=1->L1C: 146; R 1604.2500 MHz code=1->L1C: 1729; verdicts {'Accepted': 53075, 'RejectedUnknownFreq': 10374}; primary slot first: {'slot0': 36584, 'none': 2484}; freqs field inconsistent with the code field: E code 28 (L7Q (E5bQ), 1207.14 MHz) reported at 1227.6000 MHz: 10374 slots (the table rejects these slots; only the second-frequency GF slip check loses them) |
| G4 | satellites per epoch (primary signal present) | **INFO** | total min/mean/max 19/21.2/25; BDS 3/4.8/6, GAL 5/5.0/6, GPS 6/7.3/9, GLO 4/4.1/5 |
| G5 | C/N0 distribution (dB-Hz, primary signal) | **INFO** | BDS median 26 p5 17 p95 31; GAL median 26 p5 20 p95 29; GLO median 26 p5 19 p95 30; GPS median 25 p5 16 p95 29 |
| G6 | psr / cp / dopp availability (status bits + value; with the L0 C/N0 masks) | **INFO** | BDS psr 100% cp 48% dopp 100% (masked 18/34/64%); GAL psr 100% cp 45% dopp 100% (masked 0/24/63%); GLO psr 100% cp 51% dopp 100% (masked 17/40/70%); GPS psr 100% cp 30% dopp 100% (masked 4/18/52%) |
| G7 | LLI / status statistics | **INFO** | LLI {'GPS': {'3': 8037, '0': 2146, '2': 2374, '1': 8}, 'GAL': {'3': 4205, '0': 4457, '1': 10}, 'BDS': {'0': 3317, '3': 3728, '2': 1166, '1': 11}, 'GLO': {'0': 3178, '3': 3081, '2': 848, '1': 18}}; status {'GPS': {'1': 8072, '7': 1317, '15': 837, '3': 2339}, 'GAL': {'1': 4205, '7': 4439, '5': 28}, 'BDS': {'7': 1478, '1': 3739, '15': 1850, '3': 1155}, 'GLO': {'15': 2189, '1': 3096, '3': 774, '11': 59, '7': 1007}} |
| G8 | std unit judgement on the u-blox grids (§12.6-3) | **PASS** | psr metres (100.0% on 0.01*2^n m); cp cycles (100.0% on 0.004n cyc, 0.0% on 0.004n*lambda m, 0.0% on 0.004n/lambda); dopp hertz (100.0% on 0.002*2^n Hz, 0.0% as m/s); cp_std=0.06 (cpStdev=15, u-blox 'invalid') on 56.7% of slots |
| G9 | carrier-phase rate vs Doppler sign/scale (§12.6-2: corr > 0.99, slope ~ 1) | **PASS** | 14695 pairs (clean (LLI bit0 = 0)): corr 1.00000, slope 1.0000, median \|diff\| 0.059 m/s; per satellite (§12.6-2 '每星', decisive = >= 50 clean pairs and range-rate std >= 2 m/s): 11/11 decisive satellites with corr > 0.99 and \|slope - 1\| < 0.02 (slopes 0.9984..0.9996), 8 not decisive; 1 s sign agreement lambda*dPhi vs dP 100.0%, vs -lambda*D*dt 100.0% (n=1187) |
| G10 | GLONASS channel k from freqs vs ephemeris frq (§12.3, §12.6-1) | **PASS** | 14161 slots; mismatching satellites none; observed without any ephemeris: none |
| G14 | self-check 1: RTKLIB sat2freq(sat, code, nav) of every accepted slot = its freqs (A35-C8) | **PASS** | 53075 slots consistent, 0 mismatching, 0 unverifiable (GLONASS without an ephemeris channel: none), 10374 slots not accepted by the table (G3); per system {'BDS': {'consistent': 13579}, 'GAL': {'consistent': 8672, 'not_accepted': 10374}, 'GPS': {'consistent': 16663}, 'GLO': {'consistent': 14161}} |
| E1 | bag Kepler ephemerides vs same-day RINEX (same sat,toe): \|dr\| < 1e-3 m, \|ddts\| < 1e-11 s (A9-3 tier 2) | **PASS** | 31/31 matched, 0 outside tier 2 (0 not explained by a corrupted copy, see E7); worst per system {'G': '1.42e-04 m / 4.15e-16 s', 'E': '1.88e-04 m / 3.31e-15 s', 'C': '2.17e-04 m / 3.76e-16 s'}; toe label shifts {'G:+0': 6, 'E:+0': 9, 'C:+0': 16}; field semantics {'G': {'n': 6, 'toe_tow_is_system_tow': 6, 'week_is_system_week': 6, 'week_is_gps_week': 6, 'toe_msg_is_gpst': 6, 'tgd0_eq': 6, 'tgd1_eq': 6, 'tgd0_eq_rnx_tgd1': 0, 'iode_eq': 6, 'svh_eq': 6, 'code_eq': 0}, 'E': {'n': 9, 'toe_tow_is_system_tow': 9, 'week_is_system_week': 9, 'week_is_gps_week': 9, 'toe_msg_is_gpst': 9, 'tgd0_eq': 9, 'tgd1_eq': 9, 'tgd0_eq_rnx_tgd1': 0, 'iode_eq': 9, 'svh_eq': 9, 'code_eq': 9}, 'C': {'n': 16, 'toe_tow_is_system_tow': 0, 'week_is_system_week': 0, 'week_is_gps_week': 16, 'toe_msg_is_gpst': 16, 'tgd0_eq': 16, 'tgd1_eq': 16, 'tgd0_eq_rnx_tgd1': 7, 'iode_eq': 16, 'svh_eq': 16, 'code_eq': 16}} |
| E7 | bag ephemeris stream consistency: all copies of one (sat, toe, IODE) identical | **PASS** | 12 keys, 0 with conflicting copies |
| E2 | bag GLONASS ephemerides vs RINEX (pos/vel/acc units, tau_n sign, channel) | **PASS** | 21/21 matched; worst \|dr\| at toe+600 s 5.11e-05 m; semantics {'n': 21, 'taun_same_sign_as_rtklib': 21, 'taun_opposite_sign': 0, 'gamn_eq': 21, 'frq_eq': 21, 'pos_eq_1mm': 21} |
| E4 | URA semantics: bag ura vs same-day RINEX SV accuracy (§12.6-4, A16) | **PASS** | decision per system {'G': 'metres', 'E': 'metres', 'C': 'metres'} -> gnss.ephem.ura_semantics: metres; counts {'G': {'n': 6, 'metres_exact': 6, 'metres_approx': 6, 'index': 0, 'icd_formula_metres': 0}, 'E': {'n': 9, 'metres_exact': 9, 'metres_approx': 9, 'index': 0}, 'C': {'n': 16, 'metres_exact': 16, 'metres_approx': 16, 'index': 0, 'icd_formula_metres': 0}}; samples (bag, rinex m) {'G': [(2.0, 2.0), (2.0, 2.0), (2.0, 2.0), (2.0, 2.0), (2.0, 2.0), (2.0, 2.0)], 'E': [(3.12, 3.12), (3.12, 3.12), (3.12, 3.12), (3.12, 3.12), (3.12, 3.12), (3.12, 3.12), (3.12, 3.12), (3.12, 3.12)], 'C': [(2.0, 2.0), (2.0, 2.0), (2.0, 2.0), (2.0, 2.0), (2.0, 2.0), (2.0, 2.0), (2.0, 2.0), (2.0, 2.0)]} |
| E3 | orbit code self-check: adjacent RINEX records agree at the midpoint (p95 < 10 m per system) | **PASS** | C median 0.11 m p95 0.43 m (n=258); E median 0.09 m p95 0.35 m (n=643); G median 0.33 m p95 1.99 m (n=98) |
| E5 | ephemeris coverage timeline: bag-only (causal by record time) vs bag+RINEX (A1, §12.6-8) | **INFO** | sat-epochs with a healthy ephemeris: bag-only 55.0%, bag+RINEX 85.8%; epochs with >=5: 93.7% / 100.0%; first >=5 at 10.2 / 0.0 s; BDS GEO observed none; never covered bag-only: C06, C09, C16, E11, E12, E16, G01, G02, G03, G06, G21, G22, G30, R16, R18, R24 |
| E6 | Klobuchar source (A38): GPS set = RINEX header GPSA/GPSB; bag iono_params only a diagnostic (family classified) | **PASS** | GPS Klobuchar from the RINEX header GPSA/GPSB: ['2.98e-08', '2.98e-08', '-1.192e-07', '0', '1.516e+05', '-1.638e+05', '0', '6.554e+04']; bag iono_params (diagnostic, not used, A38): 16 messages, 1 distinct sets, one every 10.0 s on average; set ['3.353e-08', '1.118e-07', '-1.073e-06', '1.55e-06', '1.229e+05', '9.83e+04', '-1.245e+06', '1.507e+06']: BDS-like (distance to RINEX GPSA/GPSB 17 LSB, to the nearest RINEX BDSA/BDSB 4 LSB) |
| L1 | GNSS arrival latency: range_meas record time - GPST->UTC (leap 18 s) | **INFO** | median 38.1 ms, MAD 2.7, p1 33.0, p99 43.5, min 32.3, max 43.9; drift 61.0 ppm; persistent steps >50 ms: none; receiver_pvt median 69.2 ms |
| L3 | self-check 6 (time, record-time part): \|record - (GPST - leap)\| < 0.3 s (leap seconds and time system consistent) | **PASS** | median +0.0381 s with leap 18 s (a missing/extra leap second would show as ~±1 s, a GPST/UTC mix-up as ±18 s); the cross-correlation part of self-check 6 is method 2 of tools/time_calib/gnss_time_calib.py on the ref-R replay |
| L2 | receiver_lla header stamp convention | **INFO** | header.stamp - GPST(seconds, Unix origin) median 0.000 s (min 0.000, max 0.000): receiver_lla stamps are GPST without leap seconds (not ROS/UTC time) |
| T1 | time_pulse_info checks (A19: leap second, time system, gross errors, host clock steps) | **PASS** | 173 messages ({'UTC': 173}), pulse period 5.000 s, 137 repeated announcements, 0 gross interval errors; lead of the announcement before its pulse: median 2.943 s, window [0, 5) s, integer parts [0, 1, 2, 3, 4] with phase 0.945 s (jitter 4.4 ms) => integer-second offset +0 s: consistent; record-UTC median -2905.4 ms; host steps none |
| G11 | Doppler sign by LS fit: -lambda*D = range rate (§1) vs flipped sign (all Dopplers, el >= 5°) | **PASS** | post-fit RMS median 0.109 m/s with the design sign vs 375.4 m/s flipped (173 epochs at 1.0 Hz; linearised at the SPP, receiver PVT when no SPP) |
| G12a | self-check 5 (position): SPP vs receiver PVT | **SKIP** | not evaluable: 0 open-sky epochs have an L0-masked SPP (C/N0 >= 30 dB-Hz leaves < 4 pseudoranges; masked SPP epochs 0 of 173); decode-only unmasked SPP vs PVT horizontal median 3.1 m (n=173) |
| G12b | self-check 5 (velocity): L0-masked Doppler LS vs receiver PVT velocity on open-sky epochs: median \|dv\| < 0.2 m/s | **FAIL** | median 0.633 m/s (n=171; all epochs 0.633 m/s; horizontal 0.304, vertical 0.460 m/s); post-fit RMS of those Doppler LS median 0.077 m/s. A large vertical part with a small post-fit RMS is the geometry of the C/N0-masked rows, not a motion: when only near-zenith satellites pass the mask, vertical velocity and clock drift are nearly collinear (INTERFACE_CHANGES WP-g P4-TIME: the Doppler rows agree with the replay's antenna velocity to ~0.1 m/s on Grass01/02) |
| G13 | lever-arm direction (§12.6-7): Doppler innovations with l vs -l on turns; regression on the omega x l term (~0; ~-2 = sign flipped) | **PASS** | l confirmed: k = -0.31 (block-bootstrap s.e. 0.13, 18 blocks); innovation RMS on 3565 turning rows (\|yaw rate\| >= 0.15 rad/s): l 0.203 / -l 0.220 / 0 0.206 m/s, lever term RMS 0.046 m/s (864 epochs, 9221 rows; fitted yaw 235.49 / 236.26 / 235.83 deg); time map xcorr (sigma 43 ms), replay /r/runs/eval/p4_time_u/Grass02/U/s0 |
| S1 | Livox CustomMsg type/md5 vs the repository's livox_ros_driver headers | **PASS** | /livox/avia/lidar: livox_ros_driver/CustomMsg [e4d6829bdfe657cb6c21a746c86b21a6] vs repo livox_ros_driver/CustomMsg [e4d6829bdfe657cb6c21a746c86b21a6]; timebase on another clock (offset 1.73745e+09 s, drift 0.0 ppm vs header); points/scan median 24000 |
| S2 | CompressedImage format; replay direct decode vs republish path, bgr8 and grey pixel-exact (I2) | **PASS** | formats {'rgb8; jpeg compressed bgr8': 5184}; 150 sampled: bgr equal 150, grey equal 150, IMREAD_UNCHANGED==IMREAD_COLOR 150; shapes {'(480, 640, 3)': 150} (Python emulation of both decode paths with OpenCV; the C++ replay path itself is to be measured by WP-f) |
| S3 | GT file format (TUM: t x y z qx qy qz qw) | **WARN** | 2595 rows, columns {8: 2595}; 853 non-increasing stamps (95 backwards, max 4.1 us, 758 exact duplicates); quaternion identity on 100.0% rows (orientation not provided); repeated positions 66.8%; GT stamps = /mavros/global_position/raw/fix header stamps on 100.0% of rows |

## 话题清单

| topic | type | md5 | md5 recomputed | msgs | rate [Hz] (header, else avg) | record-header median [ms] |
|---|---|---|---|---|---|---|
| /livox/mid360/imu | sensor_msgs/Imu | 6a62c6daae103f4ff57a132d6f95cec2 | yes | 34586 | 202.41 | 6.7e-02 |
| /camera/imu | sensor_msgs/Imu | 6a62c6daae103f4ff57a132d6f95cec2 | yes | 34587 | 200.21 | 0.3 |
| /odom | nav_msgs/Odometry | cd5e73d190d741a2f92e81eda573aca7 | yes | 3459 | 20.00 | 7.9e-02 |
| /camera/aligned_depth_to_color/image_raw/compressedDepth | sensor_msgs/CompressedImage | 8f7a12909da2c9d3332d540a0977563f | yes | 5184 | 30.01 | 43.3 |
| /camera/color/image_raw/compressed | sensor_msgs/CompressedImage | 8f7a12909da2c9d3332d540a0977563f | yes | 5184 | 30.01 | 45.1 |
| /ublox_driver/range_meas | gnss_comm/GnssMeasMsg | 0eafeb530a3e6637722c4e296734c119 | yes | 1729 | 10.00 | — |
| /livox/avia/imu | sensor_msgs/Imu | 6a62c6daae103f4ff57a132d6f95cec2 | yes | 35145 | 200.15 | 21.6 |
| /livox/avia/lidar | livox_ros_driver/CustomMsg | e4d6829bdfe657cb6c21a746c86b21a6 | yes | 1729 | 10.02 | 70.9 |
| /cv_camera/image_raw/compressed | sensor_msgs/CompressedImage | 8f7a12909da2c9d3332d540a0977563f | yes | 2594 | 15.00 | 19.5 |
| /mavros/global_position/raw/fix | sensor_msgs/NavSatFix | 2d3a8cd499b9b4a0249fb98fd05cfa48 | yes | 2595 | 167772.16 | 15.7 |
| /ublox_driver/receiver_pvt | gnss_comm/GnssPVTSolnMsg | d18171357d7a159f76d4d7c0b12fb631 | yes | 1729 | 10.00 | — |
| /ublox_driver/receiver_lla | sensor_msgs/NavSatFix | 2d3a8cd499b9b4a0249fb98fd05cfa48 | yes | 1729 | 10.00 | -17930.8 |
| /livox/mid360/lidar | livox_ros_driver2/CustomMsg | e4d6829bdfe657cb6c21a746c86b21a6 | yes | 1729 | 10.00 | 100.5 |
| /ublox_driver/time_pulse_info | gnss_comm/GnssTimePulseInfoMsg | de306035e295e3c4b1e0771a216b495a | yes | 173 | 1.00 | — |
| /ublox_driver/ephem | gnss_comm/GnssEphemMsg | b4346f99e431c870e392a438684c0fe8 | yes | 31 | 0.18 | — |
| /ublox_driver/glo_ephem | gnss_comm/GnssGloEphemMsg | 6893dee0d8b1c9325a6693f3488e0144 | yes | 21 | 0.13 | — |
| /ublox_driver/iono_params | gnss_comm/StampedFloat64Array | fb60495edd59d3fcf90e173153ae8a9a | yes | 16 | 0.10 | 357.1 |

## 信号直方图 (sys, freq, code 字段 → signal_table)

| sys | freq [MHz] | code field | table code | count |
|---|---|---|---|---|
| C | 1207.1400 | 27 | L7I | 5357 |
| C | 1561.0980 | 47 | L2I | 8222 |
| E | 1227.6000 | 28 | Unknown | 10374 |
| E | 1575.4200 | 1 | L1X | 8672 |
| G | 1227.6000 | 17 | L2X | 4098 |
| G | 1575.4200 | 1 | L1C | 12565 |
| R | 1242.9375 | 14 | L2C | 1704 |
| R | 1244.6875 | 14 | L2C | 1729 |
| R | 1245.5625 | 14 | L2C | 145 |
| R | 1246.0000 | 14 | L2C | 1729 |
| R | 1247.7500 | 14 | L2C | 1729 |
| R | 1598.0625 | 1 | L1C | 1729 |
| R | 1600.3125 | 1 | L1C | 1729 |
| R | 1601.4375 | 1 | L1C | 63 |
| R | 1602.0000 | 1 | L1C | 1729 |
| R | 1603.1250 | 1 | L1C | 146 |
| R | 1604.2500 | 1 | L1C | 1729 |

## 星历字段语义(bag vs 同日 RINEX,按 (sat,toe) 配对的计数)

| sys | code_eq | iode_eq | n | svh_eq | tgd0_eq | tgd0_eq_rnx_tgd1 | tgd1_eq | toe_msg_is_gpst | toe_tow_is_system_tow | week_is_gps_week | week_is_system_week |
|---|---|---|---|---|---|---|---|---|---|---|---|
| C | 16 | 16 | 16 | 16 | 16 | 7 | 16 | 16 | 0 | 16 | 0 |
| E | 9 | 9 | 9 | 9 | 9 | 0 | 9 | 9 | 9 | 9 | 9 |
| G | 0 | 6 | 6 | 6 | 6 | 0 | 6 | 6 | 6 | 6 | 6 |

## 产物

`audit.json`(全部数字)、`latency.csv`、`eph_coverage.csv`、`eph_coverage_timeline.csv`、`plot_*.png`,位于远端归档目录。

## GT 审计(tools/gt_audit.py)

生成:`tools/gt_audit.py`(tool_version 3),2026-09-25T08:27:46Z。输入:GT 文件与原始 bag 的 RTK 定位话题、9Ps 接收机 PVT、Avia IMU、轮速计;诊断 P4c 另读 GNSS-free 回放 `/work/fastlivo2_gnss/runs/eval/p4_time_u/Grass02/U/s0`(仅开发集,A2)。

总评:**FAIL**(FAIL 1, INFO 4, PASS 2, WARN 3)。远端归档:`/work/fastlivo2_gnss/audit/Grass02/gt`。

### 检查表

| # | check | verdict | evidence |
|---|---|---|---|
| P0 | GT file format and cleaning | **INFO** | 2595 rows; 95 backward stamp steps; 263 rows reordered and 758 duplicate-stamp rows dropped in gt_clean.tum; orientation identity on 100.0% rows (position-only GT) |
| P1 | GT provenance: stamps and projection reproduced from the RTK fix (C-RTK2HP) | **PASS** | 100.0% of GT stamps equal /mavros/global_position/raw/fix header stamps (1 us); best hypothesis 'ENU at the first GT fix (exact, WGS-84)' reproduces GT to 5.17e-05 m RMS (max 7.59e-05 m); other hypotheses: UTM zone 49 (E, N, alt) minus first 0.405 m; equirectangular (R=6378137 m) minus first 0.0516 m; ENU of GCJ-02-shifted lat/lon 0.0057 m |
| P4 | GT time base vs the estimator clock (Avia IMU header) by cross-correlation; > 20 ms compensated | **FAIL** | no valid correlation chain (peaks: {'gt_course_rate__imu_yaw_rate': 0.47, 'gt_speed__odom_speed': 0.95, 'odom_yaw_rate__imu_yaw_rate': 0.71, 'gt_speed__pvt_speed_utc': 0.96}): GT time base undetermined by the pre-registered rule; AMENDMENTS A54: the record-time value is used instead (gt_timebase.yaml); for the record, the chain of the sub-threshold links would give +78.9 ms (delay precision 62.2 ms, peaks 0.95/0.71); record-time plausibility -5.9 ms; antenna-lead fit (no odometry) L +65.7 ms, r_x +0.22 m (median speed 0.92 m/s, n=676; r_x = 0 gives +226.9 ms); links: gt_course_rate__imu_yaw_rate +227.3 ms (peak 0.474, delay precision 60.4 ms); gt_speed__odom_speed +37.0 ms (peak 0.948, delay precision 61.9 ms); odom_yaw_rate__imu_yaw_rate +41.9 ms (peak 0.709, delay precision 6.7 ms); gt_speed__pvt_speed_utc +155.5 ms (peak 0.956, delay precision 43.0 ms) |
| P4c | diagnostic: GT time base via the replay = GT speed vs \|v\| of the GNSS-free replay (estimator clock; development set only) | **INFO** | GT stamp -> estimator clock +15.4 ms (peak 0.971, delay precision 35.1 ms); -63.5 ms against the sub-threshold odometry chain; +21.2 ms against the record-time plausibility |
| P2 | datum: RTK fix (GT source) lat/lon vs 9Ps WGS-84 PVT | **PASS** | median horizontal difference 0.8 m vs a GCJ-02 shift of 466 m at this location ⇒ WGS-84 (no GCJ-02 offset) |
| P3 | GT (local XYZ) vs 9Ps PVT in ENU at the GT origin: horizontal SE(2)/Sim(2) structure (ENU, no rotation/scale expected) | **WARN** | SE(2) yaw +2.001° (block-bootstrap σ 0.530°), Sim(2) scale 1.00497 (σ 0.01003); residual trend dE/dE -14.2 dN/dN +16.9 m/km; n=1726 (residual magnitudes are the 9Ps PVT noise, not a result, A2) |
| P5 | pre-registered trust mask (9Ps h_acc > 1 m or num_sv < 8; speed spikes > 3σ; gaps/stalls > 0.5 s; each ±2 s) | **WARN** | trusted 0.0% (GT-derived criteria alone: 45.6%); masked by h_acc>1 m 100.0%, num_sv<8 0.0% (receiver criteria before the ±2 s widening 99.9%), speed spikes 54.4% (95; σ 0.040 m/s, floor 0.03), gaps/stalls 0.0% (0/0); the h_acc criterion masks most rows because the 9Ps is a standalone (non-RTK) receiver: median h_acc 1.66 m — the pre-registered threshold needs a decision |
| P7 | RTK jumps / spikes (A54): fix-to-fix residual against the local velocity > max(0.5 m, 10σ); a jump and its return = displaced plateau, masked ±2 s | **INFO** | 2 jump(s) (thresholds h 0.50 / v 0.50 m; residual σ h 0.021 / v 0.031 m over 861 fix intervals): plateau +109.2..+115.4 s (6.2 s; jumps 2.04/2.06 m, return residual 0.54 m); masked 10.2 s of 172.8 s (5.90%); ATE-excluded (gaps/stalls + RTK) 10.2 s |
| P4r | GT time base used by the evaluation (AMENDMENTS A54 rule) | **WARN** | method record_time: -5.9 ms (applied shift +0.0 ms, > 20 ms rule), confidence low, uncertainty 50.0 ms (the agreement P4 requires between the record-time value and a chain (A40)); P4 FAIL: record-time value (A54) |
| P6 | 2HP antenna lever arm | **INFO** | unknown (the C-RTK2HP antenna is not in calibration.md): an ATE floor of the order of the antenna separation on the top plate (~0.1-0.3 m, README Fig. 1); TEST_PLAN E1' estimates it jointly |

### GT 生成方式假设(对 RTK 定位逐点复算)

| hypothesis | direct RMS [m] | direct max [m] | Sim3 scale | Sim3 yaw [°] | Sim3 RMS [m] |
|---|---|---|---|---|---|
| ENU at the first GT fix (exact, WGS-84) | 5.17e-05 | 7.59e-05 | 1.000 | 1.00e-04 | 1.81e-05 |
| UTM zone 49 (E, N, alt) minus first | 0.405 | 0.635 | 1.000 | 1.253 | 2.75e-05 |
| equirectangular (R=6378137 m) minus first | 0.052 | 0.100 | 0.998 | 0.030 | 0.027 |
| ENU of GCJ-02-shifted lat/lon | 0.006 | 0.012 | 1.000 | 0.012 | 0.003 |

GCJ-02 在本地的绝对平移 466 m,轨迹范围内局部畸变 0.010 m。

### 时基互相关(GT 戳 t 对应其它时钟 t + lag)

| reference | lag [ms] | peak corr | peak width [ms] | delay precision [ms] |
|---|---|---|---|---|
| gt_course_rate__imu_yaw_rate | 227.282 | 0.474 | 1120.000 | 60.380 |
| gt_speed__odom_speed | 36.969 | 0.948 | 1200.000 | 61.870 |
| odom_yaw_rate__imu_yaw_rate | 41.907 | 0.709 | 870.000 | 6.723 |
| gt_speed__pvt_speed_utc | 155.489 | 0.956 | 1330.000 | 42.956 |

### 天线超前模型(P4 诊断,不经轮速计)

航向角速率 = ω(t+L) + r_x·d/dt(ω/v):L = +65.7 ms,r_x = +0.22 m(车速中位 0.92 m/s,n = 676);r_x 置 0 时 L = +226.9 ms。天线在无侧滑点前方 r_x 处时 GT 航向**领先**车头 r_x/v,原始直接链路相对链式结果为正即此效应,不是钟差。

### 评测用 GT 时基(AMENDMENTS A54,`gt_timebase.yaml`)

规则:P4 通过 ⇒ 轮速计链;P4 失败 ⇒ 记录时间旁证;两者都没有 ⇒ 0 并标记;补偿按 TEST_PLAN §4(|值| > 20 ms 才补偿)。

| method | value [ms] | applied shift [ms] | confidence | uncertainty [ms] | P4 | chain [ms] | record time [ms] | sub-threshold chain [ms] | P4c replay (dev only, not used) [ms] |
|---|---|---|---|---|---|---|---|---|---|
| record_time | -5.9 | +0.0 | low | 50.0 | FAIL | — | -5.9 | +78.9 | +15.4 |

### RTK 跳变/毛刺与被掩时长(AMENDMENTS A54)

逐定位差分对局部速度(前后各 3 个定位区间的中位速度)的残差;跳变 = 水平 > 0.50 m 或垂直 > 0.50 m(max(0.5 m, 10σ),σ 水平 0.021 / 垂直 0.031 m);一对相互抵消的跳变之间为错位平台,连同前后 2 s 掩掉(bit 64)。

| t − GT 起点 [s] | ΔE [m] | ΔN [m] | ΔU [m] | ‖Δ‖ [m] |
|---|---|---|---|---|
| 109.411 | -0.785 | 0.265 | -1.864 | 2.040 |
| 115.410 | 0.292 | -0.139 | 2.030 | 2.056 |

| criterion | masked [s] | of bag span [%] |
|---|---|---|
| h_acc>1m | 172.811 | 100.000 |
| num_sv<8 | 0.000 | 0.000 |
| speed_spike | 93.398 | 54.047 |
| gap_or_stall | 0.000 | 0.000 |
| no_pvt_within_0.2s | 1.00e-05 | 5.79e-06 |
| rtk_jump_plateau | 10.201 | 5.903 |
| ate_exclude | 10.201 | 5.903 |
| pre_registered_all | 172.811 | 100.000 |

bag 区间内 GT 时长 172.8 s;`ate_exclude` = 间断/冻结 + RTK 跳变平台(bit 8|64;bag 区间外 bit 32 本就不参评),区间见 `gt_mask_intervals.csv`(GT 戳与审计后时基两列)。h_acc/num_sv/速度尖峰(bit 1/2/4)只报告,不标 ATE 排除(未裁定)。

### 产物

`gt_clean.tum`、`gt_audited.tum`(= gt_clean + 0.000 s,方法 record_time)、`gt_timebase.yaml`、`trust_mask.csv`、`gt_mask_intervals.csv`、`gt_audit.json`。

## 时间标定(tools/time_calib/gnss_time_calib.py,DESIGN §7)

### 方法①(记录时间统计与估计器时钟先验)

range_meas 记录时间 − GPST→UTC(leap 18 s,主机钟视角 = 主机钟偏差 + GNSS 传输延迟):中位 38.1 ms、MAD 2.7 ms、p5/p95 33.5/42.9 ms、漂移 61.0 ppm、持续台阶(>50 ms)无。

头戳 − 记录时间(判定 LiDAR 时基):/camera/color/image_raw/compressed 中位 -45.1 ms(p1 -54.2 / p99 -29.9,漂移 21.3 ppm);/livox/avia/imu 中位 -21.6 ms(p1 -71.7 / p99 29.6,漂移 -56.4 ppm);/livox/avia/lidar 中位 -70.9 ms(p1 -76.6 / p99 -65.9,漂移 -62.9 ppm)。

估计器时钟(Livox 头戳)先验:GNSS「记录 − UTC」下包络 37.0 ms 减 Avia IMU「记录 − 头戳」下包络 -24.4 ms ⇒ 上界 61.2 ms(主机钟台阶在两者中抵消;漂移 0.6 ppm;估计器时钟台阶 无);GNSS 传输延迟不可观,按 [0, 150] ms 均匀(推断)⇒ 先验 -13.8 ms、σ 75 ms。

方法①文件 `Grass02_record_time.yaml`(不读回放;method record_time,σ 75 ms;tool_version ≥ 3 按 AMENDMENTS A53 TDCP 不暂停,tool_version ≤ 2 的文件仍写着暂停):

```yaml
sequence: Grass02
tool_version: 3
leap_s: 18
segments:
  - t_from_gpst_s: 0
    offset_s: -0.013837563208117631
    drift_ppm: 0.6232660754400827
    t_ref_gpst_s: 1737451363.2
    sigma_toff_s: 0.075
    tdcp_paused: false
    method: record_time
```

### 方法②(水平天线速度互相关)与方法③(DTW)

输入:回放轨迹 `/work/fastlivo2_gnss/runs/eval/p4_time_u/Grass02/U/s0/traj_W.tum`(tum,5173 个速度样本,估计器速度 std 0.35 m/s);多普勒 LS 速度 1729 个历元。窗口 4 个,可用 3 个(窗长/步长见 JSON)。状态:ran。

信号(tool_version 3):horizontal antenna speed = |horizontal part of the zero-phase Gaussian-smoothed (0.25 s) velocity vector| on both sides, 100 Hz grid;W 的竖直方向:state_trace gravity (median g = [-0.61, 0.0068, -9.7881])。多普勒 LS:C/N0 ≥ 15.0 dB-Hz、仰角 ≥ 10.0°,按接收机 dopp_std 加权,两遍(第二遍固定平滑后的钟漂):1729 个历元有 ≥ 4 行,第二遍解出 1729 个(每历元用行中位 18.0)。

| 窗起点 [s] | 用 | 峰值相关 | 偏移 [ms] | 峰宽精度 [ms] | GNSS 覆盖 | 原因 |
|---|---|---|---|---|---|---|
| 0 | 是 | 0.979 | 72.6 | 48.3 | 100% |  |
| 30 | 是 | 0.988 | 36.6 | 37.9 | 100% |  |
| 60 | 否 | — | — | — | 100% | insufficient excitation (speed std < 0.1 m/s) or data |
| 90 | 是 | 0.865 | -13.9 | 68.0 | 100% |  |

互相关给出的段(写入文件的以状态行为准):

| t_from_gpst_s | offset_s | drift_ppm | t_ref_gpst_s | sigma_toff_s | tdcp_paused | method | peak_corr | window_scatter_ms | n_windows | n_windows_rejected | n_windows_isolated | bootstrap_sd_ms | peak_width_sigma_ms | dtw_offset_s | dtw_consistent |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 0 | 31.73 ms | 0.6233 | 1737451337.992 | 43.45 ms | false | xcorr | 0.9787 | 43.45 | 3 | 0 | 0 | 20.06 | 30.54 | 31.73 ms | true |

方法③ DTW 交叉核对:DTW 偏移 31.73 ms(路径偏移 MAD 300.0 ms)对 xcorr 31.73 ms,一致(>3σ 才告警)。

警告:window scatter 43.5 ms > 5 ms in segment from 0.0。

标定文件(方法②) `time_calib/Grass02.yaml`(AMENDMENTS A18 格式;所有 `*_gpst_s` 为 GTime::toSec(),1970 原点、GPST 尺度):

```yaml
sequence: Grass02
tool_version: 3
leap_s: 18
segments:
  - t_from_gpst_s: 0
    offset_s: 0.0317343970017492
    drift_ppm: 0.6232660754400827
    t_ref_gpst_s: 1737451337.9915934
    sigma_toff_s: 0.04345490720163389
    tdcp_paused: false
    method: xcorr
    peak_corr: 0.9786976189511629
    window_scatter_ms: 43.454907201633894
    n_windows: 3
    n_windows_rejected: 0
    n_windows_isolated: 0
    bootstrap_sd_ms: 20.059054401650403
    peak_width_sigma_ms: 30.54417046990602
    short_segment: false
    drift_source: method1_prior
    dtw_offset_s: 0.0317343970017492
    dtw_consistent: true
```
