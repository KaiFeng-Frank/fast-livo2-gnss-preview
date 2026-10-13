> Historical research record (through 2026-10-03). Paths are illustrative; archived runs are not bundled. For current status and executable entry points see [the repository README](../../../README.md). Current release docs/LICENSING.md supersedes historical license notes. This record does not authorize new experiments or assert current release validation.

# I2 数据审计 — Grass01

生成:`tools/m3dgr_audit.py`(tool_version 3),2026-09-25T08:21:07Z;bag `/work/fastlivo2_gnss/data/m3dgr/bags/Grass01.bag`(6566991382 字节,2025-01-29T18:54:42Z;sha256 未计算)。序列类别:**development**(AMENDMENTS A2)。

总评:**FAIL**(FAIL 1, INFO 9, PASS 21, SKIP 1, WARN 2)。远端归档:`/work/fastlivo2_gnss/audit/Grass01`。

## 本轮说明

P4-TIME(2026-09-25),tool_version 3;开发集(A2/A26),全部结果可看。**回放** = HEAD 的 U 臂(AMENDMENTS A55:`gnss.enable: false`,种子 0,eval 构建 `wt/p3i-strict`,build_sha256 `36a541331a5c4210…`),`runs/eval/p4_time_u/<seq>/U/s0`;其 state_trace 与已有的 ref-R 回放逐字节相同(Grass02 `2affc187…`、Grass01 `6e262080…`、Occlusion03 `96654a26…`)。P2b 版(tool_version 2)的归档备份在远端 `wt/p4-time/home/audit_backup_p2b/<seq>/`。方法、诊断与五条序列的对照见 INTERFACE_CHANGES「WP-g → P4-TIME」。BRDC `BRDC00IGS_R_20250190000_01D_MN.rnx`。

**时间标定(§7,A53)**:方法②可用(P2b 时 blocked: ref-R):8 个 60 s 窗全部可用,峰值 0.975–0.998,窗偏移 80–105 ms(峰宽精度 15–48 ms),窗间离散 8.8 ms ⇒ `Grass01.yaml` = method xcorr、+92.6 ms、σ_toff 13.1 ms、漂移 −5.1 ppm(取方法①先验)、**TDCP 不暂停**。方法①先验 +38.5 ms(σ 75 ms);两者差 +54.1 ms ⇒ GNSS 传输延迟下限约 21 ms(先验按 [0, 150] ms 均匀取中点 75 ms)。

**GT 时基(A54)**:P4 PASS,轮速计链 +14.6 ms ⇒ method chain、不补偿(≤ 20 ms)、置信高;记录时间旁证 +4.6 ms,P4c(回放路线)+29.7 ms。

**RTK 跳变(A54,P7)**:GT 首行后 +202.98…+216.78 s 一个错位平台:出跳 [−1.25, +2.67, −6.10] m(ENU),回跳 [+0.62, −2.34, +6.11] m,回跳后残差 0.71 m(平台内浮动解在水平上漂了一段)——即 A54 所说 P3 图上 183–197 s(以 T 激活为原点)的 3 m 尖峰。连同前后 2 s 掩 17.8 s(bag 区间内 GT 287.8 s 的 6.2%),`ate_exclude` 共 17.8 s(`gt_mask_intervals.csv`)。

**I2 审计(本轮随回放重跑)**:G13 杆臂方向由 blocked 变 PASS(k = −0.32 ± 0.07;转弯行新息 RMS l 0.267 / −l 0.283 / 0 0.269 m/s;时间映射 = 上面的 method xcorr);G12b 仍 FAIL(3D 中位 0.38 m/s,水平 0.20 / 竖直 0.28——掩码后的多普勒 LS 几何问题,见检查表证据);G15 改为检查 A43 的规则(PASS;退役的 5 ms 规则在本序列 −7 ms 网格上一秒都选不中)。

## 供 WP-a / 核心使用的结论

- `gnss.ephem.ura_semantics: metres`(A16;判定依据见 E4)。
- std 单位(§12.6-3):psr_std=metres,cp_std=cycles,dopp_std=hertz;cp_std=0.06 是 u-blox cpStdev=15 的『无效』标记,占 58.1%。
- 星历覆盖(A1):卫星-历元覆盖率 仅 bag 54.3% / bag+RINEX 82.5%;≥5 星历元比例 98.6% / 100.0% ⇒ 必须配置 `gnss.nav_rinex`。
- Klobuchar(A38):GPS 参数取 RINEX 头 GPSA/GPSB(本日可用);bag 的 iono_params 只记诊断,E6 判为 BDS-like(发布节奏 7.9 s,与 BDS D1 子帧 1 在各北斗星上轮播相符)。
- BDS 星历字段(E1 对拍 35 条):toe(GnssTimeMsg)是 GPST(35/35);`toe_tow` 字段是 BDT 周内秒的只有 0 条、`week` 字段是 GPS 周的 35 条 ⇒ 核心须按 §12.3 由 GPST toe 重算 toes(BDT)与 BDT 周,不得直接用消息的 toe_tow/week。
- 观测频点字段与信号码不一致(G3):E code 28 (L7Q (E5bQ), 1207.14 MHz) reported at 1227.6000 MHz: 14227 slots ⇒ ublox_driver 把这些槽的 freqs 写错;按 §12.3 频点表它们被拒收,主信号不受影响,只失去该系统的第二频点 GF 周跳检测(如需使用,可在 gnss_io 按 code 映射,需维护者决定)。
- C/N0 中位 BDS 26, GAL 25, GLO 26, GPS 25 dB-Hz;按 DESIGN §10 L0 掩码(伪距 30 / 多普勒 25 / 载波 28 dB-Hz)可用率 伪距 BDS 12%, GAL 0%, GLO 0%, GPS 2%、多普勒 BDS 66%, GAL 63%, GLO 65%, GPS 58% ⇒ 默认掩码会饿死滤波与初始化(COLLECT 需 ≥5 星 SPP);掩码必须在开发集按 TEST_PLAN §3 重新标定,不得在本序列上调。
- range_meas 记录时间 − GPST→UTC:中位 79.3 ms、MAD 1.8 ms、漂移 14.5 ppm、台阶 无(= 主机钟偏差+到达延迟;作 §7 方法①先验,不是最终时偏)。
- 传感器时基(C4,记录时间 − 头戳中位,漂移):Avia LiDAR 60.2 ms / 21 ppm、Avia IMU 10.1 ms / 16 ppm、相机 45.8 ms。Livox 头戳 = 设备时间戳 + 常数(S1:timebase on another clock (offset 1.73728e+09 s, drift 0.0 ppm vs header)),与主机时钟的关系逐序列不同(本序列与『扫描起点主机时刻 + 约 100 ms 扫描』相容) ⇒ 估计器时基(LiDAR 头戳)对 GPST 的偏差与漂移只能由 §7 方法②逐序列标定;range_meas 记录时间只作先验。
- 载波/多普勒符号(§12.6-2):斜率 1.0000、相关 1.00000 ⇒ `λ·dΦ/dt = −λD` 成立,与 DESIGN §1 一致。
- Livox:/livox/mid360/lidar livox_ros_driver2/CustomMsg [e4d6829b], /livox/avia/lidar livox_ros_driver/CustomMsg [e4d6829b] 与仓库 include/livox_ros_driver 头 md5 一致。
- CompressedImage format:{'rgb8; jpeg compressed bgr8': 8629};回放直读应 `IMREAD_COLOR` 并按 format 决定通道序(S2)。S2 的 PASS 是 Python 对两条解码路径的仿真,C++ 回放路径本身待 WP-f 实测。
- §12.6-7 杆臂方向(G13 PASS):l confirmed: k = -0.32 (block-bootstrap s.e. 0.07, 29 blocks); innovation RMS on 4666 turning rows (|yaw rate| >= 0.15 rad/s): l 0.267 / -l 0.283 / 0 0.269 m/s, lever term RMS 0.056 m/s (1439 epochs, 17353 rows; fitted yaw 210.33 / 210.84 / 210.59 deg); time map xcorr (sigma 13 ms), replay /r/runs/eval/p4_time_u/Grass01/U/s0。
- 自检 1 的 sat2freq 交叉核对(A35-C8,G14):全部一致。

## 检查表

| # | check | verdict | evidence |
|---|---|---|---|
| C1 | msg_def md5 recomputed = connection md5 (and = installed ROS type md5 where installed) | **PASS** | 17 connections, 0 mismatches; 9 standard types also matched the installed ROS packages |
| C2 | topics of the replay/GNSS path present (DESIGN §14; since A38 the RINEX NAV file is an input of every sequence) | **PASS** | missing: none |
| C3 | rates and time ranges (header-stamp period; record-time average) | **INFO** | lidar 10.0 Hz (header), avg 10.00 Hz; imu 200.1 Hz (header), avg 203.39 Hz; compressed 30.0 Hz (header), avg 29.98 Hz; range_meas avg 10.00 Hz (2878 msgs); ephem avg 0.19 Hz (53 msgs); glo_ephem avg 0.06 Hz (18 msgs); receiver_pvt avg 10.00 Hz (2878 msgs); iono_params avg 0.13 Hz (35 msgs); time_pulse_info avg 1.00 Hz (288 msgs); no gaps |
| C4 | header stamp vs record time per topic (median/p1/p99 ms, drift ppm) | **INFO** | /camera/color/image_raw/compressed 45.8/26.1/56.6 ms, -8 ppm; /livox/avia/imu 10.1/-40.8/60.5 ms, 16 ppm; /livox/avia/lidar 60.2/49.8/66.8 ms, 21 ppm; /mavros/global_position/raw/fix 14.7/-1.8/117.2 ms, 1 ppm; /ublox_driver/iono_params 348.8/339.3/398.0 ms, 31 ppm |
| G1 | epoch interval 10 Hz, monotone, one time tag per epoch (§12.6-1) | **PASS** | 2878 epochs; 100.00 % at 0.1 s; 0 non-increasing; 0 epochs with >1 time tag; 0 gaps >0.15 s |
| G15 | 1 Hz decimation rule (AMENDMENTS A43: the first epoch of every GPST second; the retired DESIGN §4.3 5 ms rule reported beside it) | **PASS** | A43 rule: 289 epochs for 289 GPST seconds with data; 10 Hz epoch grid offset -7.0 ms from the GPST tenths; the retired \|tow - round(tow)\| < 5 ms rule would select an epoch in 0.0% of the 289 seconds |
| G2 | per-satellite array lengths consistent; sat numbers inside the gnss_comm ranges | **PASS** | 0 inconsistent satellites; unknown sat numbers: none |
| G3 | (sys, freq, code) histogram and freqs->code table verdicts (signal_table.hpp) | **WARN** | C 1207.1400 MHz code=27->L7I: 15213; C 1561.0980 MHz code=47->L2I: 19175; E 1227.6000 MHz code=28->Unknown: 14227; E 1575.4200 MHz code=1->L1X: 14390; G 1227.6000 MHz code=17->L2X: 9034; G 1575.4200 MHz code=1->L1C: 20514; R 1242.9375 MHz code=14->L2C: 50; R 1244.6875 MHz code=14->L2C: 544; R 1245.5625 MHz code=14->L2C: 2832; R 1246.8750 MHz code=14->L2C: 151; R 1598.0625 MHz code=1->L1C: 15; R 1600.3125 MHz code=1->L1C: 170; R 1600.8750 MHz code=1->L1C: 2878; R 1601.4375 MHz code=1->L1C: 2878; R 1603.1250 MHz code=1->L1C: 2878; R 1603.6875 MHz code=1->L1C: 2878; verdicts {'Accepted': 93600, 'RejectedUnknownFreq': 14227}; primary slot first: {'slot0': 65776, 'none': 4678}; freqs field inconsistent with the code field: E code 28 (L7Q (E5bQ), 1207.14 MHz) reported at 1227.6000 MHz: 14227 slots (the table rejects these slots; only the second-frequency GF slip check loses them) |
| G4 | satellites per epoch (primary signal present) | **INFO** | total min/mean/max 21/22.9/25; BDS 6/6.7/8, GAL 5/5.0/5, GPS 5/7.1/10, GLO 4/4.1/5 |
| G5 | C/N0 distribution (dB-Hz, primary signal) | **INFO** | BDS median 26 p5 20 p95 30; GAL median 25 p5 20 p95 28; GLO median 26 p5 21 p95 29; GPS median 25 p5 16 p95 29 |
| G6 | psr / cp / dopp availability (status bits + value; with the L0 C/N0 masks) | **INFO** | BDS psr 100% cp 46% dopp 100% (masked 12/27/66%); GAL psr 80% cp 35% dopp 100% (masked 0/12/63%); GLO psr 26% cp 37% dopp 100% (masked 0/18/65%); GPS psr 86% cp 32% dopp 100% (masked 2/18/58%) |
| G7 | LLI / status statistics | **INFO** | LLI {'GPS': {'0': 3654, '3': 12526, '2': 4307, '1': 27}, 'BDS': {'3': 9072, '0': 8137, '2': 1949, '1': 17}, 'GLO': {'3': 6061, '0': 3819, '2': 1786, '1': 31}, 'GAL': {'3': 8158, '0': 6212, '1': 20}}; status {'GPS': {'7': 1659, '1': 12599, '15': 2022, '3': 4234}, 'BDS': {'1': 9099, '15': 2283, '7': 5871, '3': 1922}, 'GLO': {'1': 6096, '15': 2718, '3': 1691, '11': 60, '7': 1132}, 'GAL': {'1': 8158, '7': 6176, '5': 56}} |
| G8 | std unit judgement on the u-blox grids (§12.6-3) | **PASS** | psr metres (100.0% on 0.01*2^n m); cp cycles (100.0% on 0.004n cyc, 0.0% on 0.004n*lambda m, 0.0% on 0.004n/lambda); dopp hertz (100.0% on 0.002*2^n Hz, 0.0% as m/s); cp_std=0.06 (cpStdev=15, u-blox 'invalid') on 58.1% of slots |
| G9 | carrier-phase rate vs Doppler sign/scale (§12.6-2: corr > 0.99, slope ~ 1) | **PASS** | 23593 pairs (clean (LLI bit0 = 0)): corr 1.00000, slope 1.0000, median \|diff\| 0.061 m/s; per satellite (§12.6-2 '每星', decisive = >= 50 clean pairs and range-rate std >= 2 m/s): 15/15 decisive satellites with corr > 0.99 and \|slope - 1\| < 0.02 (slopes 0.9980..1.0001), 6 not decisive; 1 s sign agreement lambda*dPhi vs dP 100.0%, vs -lambda*D*dt 100.0% (n=1940) |
| G10 | GLONASS channel k from freqs vs ephemeris frq (§12.3, §12.6-1) | **PASS** | 15274 slots; mismatching satellites none; observed without any ephemeris: none |
| G14 | self-check 1: RTKLIB sat2freq(sat, code, nav) of every accepted slot = its freqs (A35-C8) | **PASS** | 93600 slots consistent, 0 mismatching, 0 unverifiable (GLONASS without an ephemeris channel: none), 14227 slots not accepted by the table (G3); per system {'BDS': {'consistent': 34388}, 'GAL': {'consistent': 14390, 'not_accepted': 14227}, 'GPS': {'consistent': 29548}, 'GLO': {'consistent': 15274}} |
| E1 | bag Kepler ephemerides vs same-day RINEX (same sat,toe): \|dr\| < 1e-3 m, \|ddts\| < 1e-11 s (A9-3 tier 2) | **PASS** | 53/53 matched, 0 outside tier 2 (0 not explained by a corrupted copy, see E7); worst per system {'E': '2.37e-04 m / 4.50e-15 s', 'C': '1.09e-04 m / 3.61e-16 s', 'G': '2.03e-04 m / 3.66e-16 s'}; toe label shifts {'E:+0': 12, 'C:+0': 35, 'G:+0': 6}; field semantics {'E': {'n': 12, 'toe_tow_is_system_tow': 12, 'week_is_system_week': 12, 'week_is_gps_week': 12, 'toe_msg_is_gpst': 12, 'tgd0_eq': 12, 'tgd1_eq': 12, 'tgd0_eq_rnx_tgd1': 4, 'iode_eq': 12, 'svh_eq': 12, 'code_eq': 12}, 'C': {'n': 35, 'toe_tow_is_system_tow': 0, 'week_is_system_week': 0, 'week_is_gps_week': 35, 'toe_msg_is_gpst': 35, 'tgd0_eq': 35, 'tgd1_eq': 35, 'tgd0_eq_rnx_tgd1': 11, 'iode_eq': 35, 'svh_eq': 35, 'code_eq': 35}, 'G': {'n': 6, 'toe_tow_is_system_tow': 6, 'week_is_system_week': 6, 'week_is_gps_week': 6, 'toe_msg_is_gpst': 6, 'tgd0_eq': 6, 'tgd1_eq': 6, 'tgd0_eq_rnx_tgd1': 0, 'iode_eq': 6, 'svh_eq': 6, 'code_eq': 0}} |
| E7 | bag ephemeris stream consistency: all copies of one (sat, toe, IODE) identical | **PASS** | 13 keys, 0 with conflicting copies |
| E2 | bag GLONASS ephemerides vs RINEX (pos/vel/acc units, tau_n sign, channel) | **PASS** | 18/18 matched; worst \|dr\| at toe+600 s 4.35e-05 m; semantics {'n': 18, 'taun_same_sign_as_rtklib': 18, 'taun_opposite_sign': 0, 'gamn_eq': 18, 'frq_eq': 18, 'pos_eq_1mm': 18} |
| E4 | URA semantics: bag ura vs same-day RINEX SV accuracy (§12.6-4, A16) | **PASS** | decision per system {'E': 'metres', 'C': 'metres', 'G': 'metres'} -> gnss.ephem.ura_semantics: metres; counts {'E': {'n': 12, 'metres_exact': 12, 'metres_approx': 12, 'index': 0}, 'C': {'n': 35, 'metres_exact': 35, 'metres_approx': 35, 'index': 0, 'icd_formula_metres': 0}, 'G': {'n': 6, 'metres_exact': 6, 'metres_approx': 6, 'index': 0, 'icd_formula_metres': 0}}; samples (bag, rinex m) {'E': [(3.12, 3.12), (3.12, 3.12), (3.12, 3.12), (3.12, 3.12), (3.12, 3.12), (3.12, 3.12), (3.12, 3.12), (3.12, 3.12)], 'C': [(2.0, 2.0), (2.0, 2.0), (2.0, 2.0), (2.0, 2.0), (2.0, 2.0), (2.0, 2.0), (2.0, 2.0), (2.0, 2.0)], 'G': [(2.0, 2.0), (2.0, 2.0), (2.0, 2.0), (2.0, 2.0), (2.0, 2.0), (2.0, 2.0)]} |
| E3 | orbit code self-check: adjacent RINEX records agree at the midpoint (p95 < 10 m per system) | **PASS** | C median 0.11 m p95 0.36 m (n=261); E median 0.08 m p95 0.33 m (n=634); G median 0.39 m p95 2.33 m (n=95) |
| E5 | ephemeris coverage timeline: bag-only (causal by record time) vs bag+RINEX (A1, §12.6-8) | **INFO** | sat-epochs with a healthy ephemeris: bag-only 54.3%, bag+RINEX 82.5%; epochs with >=5: 98.6% / 100.0%; first >=5 at 3.6 / 0.0 s; BDS GEO observed none; never covered bag-only: C06, C09, C24, E18, E23, G01, G02, G03, G07, G19, G21, G22, G30, R13, R14, R22 |
| E6 | Klobuchar source (A38): GPS set = RINEX header GPSA/GPSB; bag iono_params only a diagnostic (family classified) | **PASS** | GPS Klobuchar from the RINEX header GPSA/GPSB: ['2.98e-08', '2.98e-08', '-1.192e-07', '0', '1.516e+05', '-1.638e+05', '0', '6.554e+04']; bag iono_params (diagnostic, not used, A38): 35 messages, 1 distinct sets, one every 7.9 s on average; set ['3.446e-08', '1.714e-07', '-1.49e-06', '2.146e-06', '1.331e+05', '-3.768e+05', '1.049e+06', '0']: BDS-like (distance to RINEX GPSA/GPSB 18 LSB, to the nearest RINEX BDSA/BDSB 12 LSB) |
| L1 | GNSS arrival latency: range_meas record time - GPST->UTC (leap 18 s) | **INFO** | median 79.3 ms, MAD 1.8, p1 70.2, p99 86.7, min 68.5, max 88.1; drift 14.5 ppm; persistent steps >50 ms: none; receiver_pvt median 102.6 ms |
| L3 | self-check 6 (time, record-time part): \|record - (GPST - leap)\| < 0.3 s (leap seconds and time system consistent) | **PASS** | median +0.0793 s with leap 18 s (a missing/extra leap second would show as ~±1 s, a GPST/UTC mix-up as ±18 s); the cross-correlation part of self-check 6 is method 2 of tools/time_calib/gnss_time_calib.py on the ref-R replay |
| L2 | receiver_lla header stamp convention | **INFO** | header.stamp - GPST(seconds, Unix origin) median 0.000 s (min 0.000, max 0.000): receiver_lla stamps are GPST without leap seconds (not ROS/UTC time) |
| T1 | time_pulse_info checks (A19: leap second, time system, gross errors, host clock steps) | **PASS** | 288 messages ({'UTC': 288}), pulse period 5.000 s, 230 repeated announcements, 0 gross interval errors; lead of the announcement before its pulse: median 2.953 s, window [0, 5) s, integer parts [0, 1, 2, 3, 4] with phase 0.953 s (jitter 5.5 ms) => integer-second offset +0 s: consistent; record-UTC median -2873.3 ms; host steps none |
| G11 | Doppler sign by LS fit: -lambda*D = range rate (§1) vs flipped sign (all Dopplers, el >= 5°) | **PASS** | post-fit RMS median 0.103 m/s with the design sign vs 390.2 m/s flipped (288 epochs at 1.0 Hz; linearised at the SPP, receiver PVT when no SPP) |
| G12a | self-check 5 (position): SPP vs receiver PVT | **SKIP** | not evaluable: 0 open-sky epochs have an L0-masked SPP (C/N0 >= 30 dB-Hz leaves < 4 pseudoranges; masked SPP epochs 0 of 288); decode-only unmasked SPP vs PVT horizontal median 3.3 m (n=288) |
| G12b | self-check 5 (velocity): L0-masked Doppler LS vs receiver PVT velocity on open-sky epochs: median \|dv\| < 0.2 m/s | **FAIL** | median 0.382 m/s (n=288; all epochs 0.382 m/s; horizontal 0.200, vertical 0.280 m/s); post-fit RMS of those Doppler LS median 0.079 m/s. A large vertical part with a small post-fit RMS is the geometry of the C/N0-masked rows, not a motion: when only near-zenith satellites pass the mask, vertical velocity and clock drift are nearly collinear (INTERFACE_CHANGES WP-g P4-TIME: the Doppler rows agree with the replay's antenna velocity to ~0.1 m/s on Grass01/02) |
| G13 | lever-arm direction (§12.6-7): Doppler innovations with l vs -l on turns; regression on the omega x l term (~0; ~-2 = sign flipped) | **PASS** | l confirmed: k = -0.32 (block-bootstrap s.e. 0.07, 29 blocks); innovation RMS on 4666 turning rows (\|yaw rate\| >= 0.15 rad/s): l 0.267 / -l 0.283 / 0 0.269 m/s, lever term RMS 0.056 m/s (1439 epochs, 17353 rows; fitted yaw 210.33 / 210.84 / 210.59 deg); time map xcorr (sigma 13 ms), replay /r/runs/eval/p4_time_u/Grass01/U/s0 |
| S1 | Livox CustomMsg type/md5 vs the repository's livox_ros_driver headers | **PASS** | /livox/avia/lidar: livox_ros_driver/CustomMsg [e4d6829bdfe657cb6c21a746c86b21a6] vs repo livox_ros_driver/CustomMsg [e4d6829bdfe657cb6c21a746c86b21a6]; timebase on another clock (offset 1.73728e+09 s, drift 0.0 ppm vs header); points/scan median 24000 |
| S2 | CompressedImage format; replay direct decode vs republish path, bgr8 and grey pixel-exact (I2) | **PASS** | formats {'rgb8; jpeg compressed bgr8': 8629}; 150 sampled: bgr equal 150, grey equal 150, IMREAD_UNCHANGED==IMREAD_COLOR 150; shapes {'(480, 640, 3)': 150} (Python emulation of both decode paths with OpenCV; the C++ replay path itself is to be measured by WP-f) |
| S3 | GT file format (TUM: t x y z qx qy qz qw) | **WARN** | 4318 rows, columns {8: 4318}; 2382 non-increasing stamps (120 backwards, max 26.2 us, 2262 exact duplicates); quaternion identity on 100.0% rows (orientation not provided); repeated positions 66.7%; GT stamps = /mavros/global_position/raw/fix header stamps on 100.0% of rows |

## 话题清单

| topic | type | md5 | md5 recomputed | msgs | rate [Hz] (header, else avg) | record-header median [ms] |
|---|---|---|---|---|---|---|
| /camera/imu | sensor_msgs/Imu | 6a62c6daae103f4ff57a132d6f95cec2 | yes | 57569 | 200.12 | 0.4 |
| /livox/mid360/imu | sensor_msgs/Imu | 6a62c6daae103f4ff57a132d6f95cec2 | yes | 57571 | 201.87 | 7.6e-02 |
| /mavros/global_position/raw/fix | sensor_msgs/NavSatFix | 2d3a8cd499b9b4a0249fb98fd05cfa48 | yes | 4318 | 15.00 | 14.7 |
| /camera/aligned_depth_to_color/image_raw/compressedDepth | sensor_msgs/CompressedImage | 8f7a12909da2c9d3332d540a0977563f | yes | 8629 | 30.00 | 44.0 |
| /camera/color/image_raw/compressed | sensor_msgs/CompressedImage | 8f7a12909da2c9d3332d540a0977563f | yes | 8629 | 30.00 | 45.8 |
| /odom | nav_msgs/Odometry | cd5e73d190d741a2f92e81eda573aca7 | yes | 5757 | 20.00 | 8.5e-02 |
| /livox/mid360/lidar | livox_ros_driver2/CustomMsg | e4d6829bdfe657cb6c21a746c86b21a6 | yes | 2879 | 10.00 | 100.8 |
| /cv_camera/image_raw/compressed | sensor_msgs/CompressedImage | 8f7a12909da2c9d3332d540a0977563f | yes | 4318 | 15.00 | 26.0 |
| /ublox_driver/range_meas | gnss_comm/GnssMeasMsg | 0eafeb530a3e6637722c4e296734c119 | yes | 2878 | 10.00 | — |
| /ublox_driver/receiver_pvt | gnss_comm/GnssPVTSolnMsg | d18171357d7a159f76d4d7c0b12fb631 | yes | 2878 | 10.00 | — |
| /ublox_driver/receiver_lla | sensor_msgs/NavSatFix | 2d3a8cd499b9b4a0249fb98fd05cfa48 | yes | 2878 | 10.00 | -17897.4 |
| /livox/avia/imu | sensor_msgs/Imu | 6a62c6daae103f4ff57a132d6f95cec2 | yes | 58521 | 200.15 | 10.1 |
| /livox/avia/lidar | livox_ros_driver/CustomMsg | e4d6829bdfe657cb6c21a746c86b21a6 | yes | 2878 | 10.02 | 60.2 |
| /ublox_driver/time_pulse_info | gnss_comm/GnssTimePulseInfoMsg | de306035e295e3c4b1e0771a216b495a | yes | 288 | 1.00 | — |
| /ublox_driver/glo_ephem | gnss_comm/GnssGloEphemMsg | 6893dee0d8b1c9325a6693f3488e0144 | yes | 18 | 0.06 | — |
| /ublox_driver/ephem | gnss_comm/GnssEphemMsg | b4346f99e431c870e392a438684c0fe8 | yes | 53 | 0.19 | — |
| /ublox_driver/iono_params | gnss_comm/StampedFloat64Array | fb60495edd59d3fcf90e173153ae8a9a | yes | 35 | 0.13 | 348.8 |

## 信号直方图 (sys, freq, code 字段 → signal_table)

| sys | freq [MHz] | code field | table code | count |
|---|---|---|---|---|
| C | 1207.1400 | 27 | L7I | 15213 |
| C | 1561.0980 | 47 | L2I | 19175 |
| E | 1227.6000 | 28 | Unknown | 14227 |
| E | 1575.4200 | 1 | L1X | 14390 |
| G | 1227.6000 | 17 | L2X | 9034 |
| G | 1575.4200 | 1 | L1C | 20514 |
| R | 1242.9375 | 14 | L2C | 50 |
| R | 1244.6875 | 14 | L2C | 544 |
| R | 1245.5625 | 14 | L2C | 2832 |
| R | 1246.8750 | 14 | L2C | 151 |
| R | 1598.0625 | 1 | L1C | 15 |
| R | 1600.3125 | 1 | L1C | 170 |
| R | 1600.8750 | 1 | L1C | 2878 |
| R | 1601.4375 | 1 | L1C | 2878 |
| R | 1603.1250 | 1 | L1C | 2878 |
| R | 1603.6875 | 1 | L1C | 2878 |

## 星历字段语义(bag vs 同日 RINEX,按 (sat,toe) 配对的计数)

| sys | code_eq | iode_eq | n | svh_eq | tgd0_eq | tgd0_eq_rnx_tgd1 | tgd1_eq | toe_msg_is_gpst | toe_tow_is_system_tow | week_is_gps_week | week_is_system_week |
|---|---|---|---|---|---|---|---|---|---|---|---|
| C | 35 | 35 | 35 | 35 | 35 | 11 | 35 | 35 | 0 | 35 | 0 |
| E | 12 | 12 | 12 | 12 | 12 | 4 | 12 | 12 | 12 | 12 | 12 |
| G | 0 | 6 | 6 | 6 | 6 | 0 | 6 | 6 | 6 | 6 | 6 |

## 产物

`audit.json`(全部数字)、`latency.csv`、`eph_coverage.csv`、`eph_coverage_timeline.csv`、`plot_*.png`,位于远端归档目录。

## GT 审计(tools/gt_audit.py)

生成:`tools/gt_audit.py`(tool_version 3),2026-09-25T08:27:27Z。输入:GT 文件与原始 bag 的 RTK 定位话题、9Ps 接收机 PVT、Avia IMU、轮速计;诊断 P4c 另读 GNSS-free 回放 `/work/fastlivo2_gnss/runs/eval/p4_time_u/Grass01/U/s0`(仅开发集,A2)。

总评:**WARN**(INFO 5, PASS 4, WARN 1)。远端归档:`/work/fastlivo2_gnss/audit/Grass01/gt`。

### 检查表

| # | check | verdict | evidence |
|---|---|---|---|
| P0 | GT file format and cleaning | **INFO** | 4318 rows; 120 backward stamp steps; 246 rows reordered and 2262 duplicate-stamp rows dropped in gt_clean.tum; orientation identity on 100.0% rows (position-only GT) |
| P1 | GT provenance: stamps and projection reproduced from the RTK fix (C-RTK2HP) | **PASS** | 100.0% of GT stamps equal /mavros/global_position/raw/fix header stamps (1 us); best hypothesis 'ENU at the first GT fix (exact, WGS-84)' reproduces GT to 4.26e-04 m RMS (max 8.48e-04 m); other hypotheses: UTM zone 49 (E, N, alt) minus first 1.43 m; equirectangular (R=6378137 m) minus first 0.222 m; ENU of GCJ-02-shifted lat/lon 0.0211 m |
| P4 | GT time base vs the estimator clock (Avia IMU header) by cross-correlation; > 20 ms compensated | **PASS** | GT stamp -> Avia IMU header clock: +14.6 ms by chain GT->odom (speed) + odom->IMU (yaw rate); <= 20 ms: no compensation; record-time plausibility +4.6 ms (assumes equal transport delays; agreement within 50 ms required); antenna-lead fit (no odometry) L -121.0 ms, r_x +0.48 m (median speed 0.89 m/s, n=966; r_x = 0 gives +185.0 ms); raw direct course-rate link +218.8 ms (peak 0.36): +204.2 ms against the chain = antenna lead r_x/v, not a clock term; links: gt_course_rate__imu_yaw_rate +218.8 ms (peak 0.364, delay precision 83.3 ms); gt_speed__odom_speed -9.0 ms (peak 0.818, delay precision 113.7 ms); odom_yaw_rate__imu_yaw_rate +23.6 ms (peak 0.846, delay precision 3.7 ms); gt_speed__pvt_speed_utc +94.9 ms (peak 0.835, delay precision 64.2 ms) |
| P4c | diagnostic: GT time base via the replay = GT speed vs \|v\| of the GNSS-free replay (estimator clock; development set only) | **INFO** | GT stamp -> estimator clock +29.7 ms (peak 0.841, delay precision 63.2 ms); +15.1 ms against the odometry chain; +25.1 ms against the record-time plausibility |
| P2 | datum: RTK fix (GT source) lat/lon vs 9Ps WGS-84 PVT | **PASS** | median horizontal difference 2.9 m vs a GCJ-02 shift of 466 m at this location ⇒ WGS-84 (no GCJ-02 offset) |
| P3 | GT (local XYZ) vs 9Ps PVT in ENU at the GT origin: horizontal SE(2)/Sim(2) structure (ENU, no rotation/scale expected) | **PASS** | SE(2) yaw +0.177° (block-bootstrap σ 0.140°), Sim(2) scale 1.00671 (σ 0.00325); residual trend dE/dE +12.2 dN/dN +14.6 m/km; n=2876 (residual magnitudes are the 9Ps PVT noise, not a result, A2) |
| P5 | pre-registered trust mask (9Ps h_acc > 1 m or num_sv < 8; speed spikes > 3σ; gaps/stalls > 0.5 s; each ±2 s) | **WARN** | trusted 0.0% (GT-derived criteria alone: 40.9%); masked by h_acc>1 m 100.0%, num_sv<8 0.0% (receiver criteria before the ±2 s widening 100.0%), speed spikes 59.1% (125; σ 0.036 m/s, floor 0.03), gaps/stalls 0.0% (0/0); the h_acc criterion masks most rows because the 9Ps is a standalone (non-RTK) receiver: median h_acc 1.46 m — the pre-registered threshold needs a decision |
| P7 | RTK jumps / spikes (A54): fix-to-fix residual against the local velocity > max(0.5 m, 10σ); a jump and its return = displaced plateau, masked ±2 s | **INFO** | 2 jump(s) (thresholds h 0.50 / v 0.50 m; residual σ h 0.021 / v 0.030 m over 1437 fix intervals): plateau +203.0..+216.8 s (13.8 s; jumps 6.78/6.57 m, return residual 0.71 m); masked 17.8 s of 287.8 s (6.18%); ATE-excluded (gaps/stalls + RTK) 17.8 s |
| P4r | GT time base used by the evaluation (AMENDMENTS A54 rule) | **INFO** | method chain: +14.6 ms (applied shift +0.0 ms, > 20 ms rule), confidence high, uncertainty 50.0 ms (peak-width precisions of the two chain links combined (Carter bound, 113.7 ms), capped by the 50 ms agreement with the record-time value that P4 PASS requires) |
| P6 | 2HP antenna lever arm | **INFO** | unknown (the C-RTK2HP antenna is not in calibration.md): an ATE floor of the order of the antenna separation on the top plate (~0.1-0.3 m, README Fig. 1); TEST_PLAN E1' estimates it jointly |

### GT 生成方式假设(对 RTK 定位逐点复算)

| hypothesis | direct RMS [m] | direct max [m] | Sim3 scale | Sim3 yaw [°] | Sim3 RMS [m] |
|---|---|---|---|---|---|
| ENU at the first GT fix (exact, WGS-84) | 4.26e-04 | 8.48e-04 | 1.000 | 2.36e-05 | 6.26e-05 |
| UTM zone 49 (E, N, alt) minus first | 1.427 | 2.270 | 1.000 | 1.254 | 8.04e-05 |
| equirectangular (R=6378137 m) minus first | 0.222 | 0.354 | 0.998 | 0.083 | 0.044 |
| ENU of GCJ-02-shifted lat/lon | 0.021 | 0.037 | 1.000 | 0.013 | 0.005 |

GCJ-02 在本地的绝对平移 466 m,轨迹范围内局部畸变 0.021 m。

### 时基互相关(GT 戳 t 对应其它时钟 t + lag)

| reference | lag [ms] | peak corr | peak width [ms] | delay precision [ms] |
|---|---|---|---|---|
| gt_course_rate__imu_yaw_rate | 218.847 | 0.364 | 960.000 | 83.288 |
| gt_speed__odom_speed | -8.952 | 0.818 | 1310.000 | 113.655 |
| odom_yaw_rate__imu_yaw_rate | 23.565 | 0.846 | 860.000 | 3.678 |
| gt_speed__pvt_speed_utc | 94.926 | 0.835 | 1400.000 | 64.210 |

### 天线超前模型(P4 诊断,不经轮速计)

航向角速率 = ω(t+L) + r_x·d/dt(ω/v):L = -121.0 ms,r_x = +0.48 m(车速中位 0.89 m/s,n = 966);r_x 置 0 时 L = +185.0 ms。天线在无侧滑点前方 r_x 处时 GT 航向**领先**车头 r_x/v,原始直接链路相对链式结果为正即此效应,不是钟差。

### 评测用 GT 时基(AMENDMENTS A54,`gt_timebase.yaml`)

规则:P4 通过 ⇒ 轮速计链;P4 失败 ⇒ 记录时间旁证;两者都没有 ⇒ 0 并标记;补偿按 TEST_PLAN §4(|值| > 20 ms 才补偿)。

| method | value [ms] | applied shift [ms] | confidence | uncertainty [ms] | P4 | chain [ms] | record time [ms] | sub-threshold chain [ms] | P4c replay (dev only, not used) [ms] |
|---|---|---|---|---|---|---|---|---|---|
| chain | +14.6 | +0.0 | high | 50.0 | PASS | +14.6 | +4.6 | — | +29.7 |

### RTK 跳变/毛刺与被掩时长(AMENDMENTS A54)

逐定位差分对局部速度(前后各 3 个定位区间的中位速度)的残差;跳变 = 水平 > 0.50 m 或垂直 > 0.50 m(max(0.5 m, 10σ),σ 水平 0.021 / 垂直 0.030 m);一对相互抵消的跳变之间为错位平台,连同前后 2 s 掩掉(bit 64)。

| t − GT 起点 [s] | ΔE [m] | ΔN [m] | ΔU [m] | ‖Δ‖ [m] |
|---|---|---|---|---|
| 203.187 | -1.254 | 2.671 | -6.100 | 6.776 |
| 216.783 | 0.623 | -2.335 | 6.110 | 6.571 |

| criterion | masked [s] | of bag span [%] |
|---|---|---|
| h_acc>1m | 287.787 | 100.000 |
| num_sv<8 | 0.000 | 0.000 |
| speed_spike | 171.404 | 59.559 |
| gap_or_stall | 0.000 | 0.000 |
| no_pvt_within_0.2s | 0.000 | 0.000 |
| rtk_jump_plateau | 17.798 | 6.184 |
| ate_exclude | 17.798 | 6.184 |
| pre_registered_all | 287.787 | 100.000 |

bag 区间内 GT 时长 287.8 s;`ate_exclude` = 间断/冻结 + RTK 跳变平台(bit 8|64;bag 区间外 bit 32 本就不参评),区间见 `gt_mask_intervals.csv`(GT 戳与审计后时基两列)。h_acc/num_sv/速度尖峰(bit 1/2/4)只报告,不标 ATE 排除(未裁定)。

### 产物

`gt_clean.tum`、`gt_audited.tum`(= gt_clean + 0.000 s,方法 chain)、`gt_timebase.yaml`、`trust_mask.csv`、`gt_mask_intervals.csv`、`gt_audit.json`。

## 时间标定(tools/time_calib/gnss_time_calib.py,DESIGN §7)

### 方法①(记录时间统计与估计器时钟先验)

range_meas 记录时间 − GPST→UTC(leap 18 s,主机钟视角 = 主机钟偏差 + GNSS 传输延迟):中位 79.3 ms、MAD 1.8 ms、p5/p95 72.1/85.5 ms、漂移 14.5 ppm、持续台阶(>50 ms)无。

头戳 − 记录时间(判定 LiDAR 时基):/camera/color/image_raw/compressed 中位 -45.8 ms(p1 -56.6 / p99 -26.1,漂移 7.6 ppm);/livox/avia/imu 中位 -10.1 ms(p1 -60.5 / p99 40.8,漂移 -15.7 ppm);/livox/avia/lidar 中位 -60.2 ms(p1 -66.8 / p99 -49.8,漂移 -20.5 ppm)。

估计器时钟(Livox 头戳)先验:GNSS「记录 − UTC」下包络 78.4 ms 减 Avia IMU「记录 − 头戳」下包络 -34.8 ms ⇒ 上界 113.5 ms(主机钟台阶在两者中抵消;漂移 -5.1 ppm;估计器时钟台阶 无);GNSS 传输延迟不可观,按 [0, 150] ms 均匀(推断)⇒ 先验 38.5 ms、σ 75 ms。

方法①文件 `Grass01_record_time.yaml`(不读回放;method record_time,σ 75 ms;tool_version ≥ 3 按 AMENDMENTS A53 TDCP 不暂停,tool_version ≤ 2 的文件仍写着暂停):

```yaml
sequence: Grass01
tool_version: 3
leap_s: 18
segments:
  - t_from_gpst_s: 0
    offset_s: 0.038507148313280073
    drift_ppm: -5.106769620705324
    t_ref_gpst_s: 1737278747.343
    sigma_toff_s: 0.075
    tdcp_paused: false
    method: record_time
```

### 方法②(水平天线速度互相关)与方法③(DTW)

输入:回放轨迹 `/work/fastlivo2_gnss/runs/eval/p4_time_u/Grass01/U/s0/traj_W.tum`(tum,8617 个速度样本,估计器速度 std 0.38 m/s);多普勒 LS 速度 2878 个历元。窗口 8 个,可用 8 个(窗长/步长见 JSON)。状态:ran。

信号(tool_version 3):horizontal antenna speed = |horizontal part of the zero-phase Gaussian-smoothed (0.25 s) velocity vector| on both sides, 100 Hz grid;W 的竖直方向:state_trace gravity (median g = [-0.3535, -0.3676, -9.7951])。多普勒 LS:C/N0 ≥ 15.0 dB-Hz、仰角 ≥ 10.0°,按接收机 dopp_std 加权,两遍(第二遍固定平滑后的钟漂):2878 个历元有 ≥ 4 行,第二遍解出 2878 个(每历元用行中位 19.0)。

| 窗起点 [s] | 用 | 峰值相关 | 偏移 [ms] | 峰宽精度 [ms] | GNSS 覆盖 | 原因 |
|---|---|---|---|---|---|---|
| 0 | 是 | 0.996 | 103.0 | 25.1 | 100% |  |
| 30 | 是 | 0.995 | 94.7 | 30.2 | 100% |  |
| 60 | 是 | 0.991 | 84.3 | 48.0 | 100% |  |
| 90 | 是 | 0.992 | 86.7 | 42.8 | 100% |  |
| 120 | 是 | 0.998 | 98.1 | 25.4 | 100% |  |
| 150 | 是 | 0.995 | 104.8 | 33.9 | 100% |  |
| 180 | 是 | 0.975 | 88.9 | 22.6 | 100% |  |
| 210 | 是 | 0.994 | 80.3 | 15.3 | 100% |  |

互相关给出的段(写入文件的以状态行为准):

| t_from_gpst_s | offset_s | drift_ppm | t_ref_gpst_s | sigma_toff_s | tdcp_paused | method | peak_corr | window_scatter_ms | n_windows | n_windows_rejected | n_windows_isolated | bootstrap_sd_ms | peak_width_sigma_ms | dtw_offset_s | dtw_consistent |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 0 | 92.60 ms | -5.107 | 1737278739.722 | 13.10 ms | false | xcorr | 0.9947 | 8.804 | 8 | 0 | 0 | 3.047 | 13.1 | 92.60 ms | true |

方法③ DTW 交叉核对:DTW 偏移 92.60 ms(路径偏移 MAD 200.0 ms)对 xcorr 92.60 ms,一致(>3σ 才告警)。

警告:window scatter 8.8 ms > 5 ms in segment from 0.0。

标定文件(方法②) `time_calib/Grass01.yaml`(AMENDMENTS A18 格式;所有 `*_gpst_s` 为 GTime::toSec(),1970 原点、GPST 尺度):

```yaml
sequence: Grass01
tool_version: 3
leap_s: 18
segments:
  - t_from_gpst_s: 0
    offset_s: 0.09259826067721576
    drift_ppm: -5.1067696207053235
    t_ref_gpst_s: 1737278739.7219672
    sigma_toff_s: 0.013101325094101791
    tdcp_paused: false
    method: xcorr
    peak_corr: 0.9946704446093354
    window_scatter_ms: 8.803671630364208
    n_windows: 8
    n_windows_rejected: 0
    n_windows_isolated: 0
    bootstrap_sd_ms: 3.047190168889514
    peak_width_sigma_ms: 13.10132509410179
    short_segment: false
    drift_source: method1_prior
    dtw_offset_s: 0.09259826067721576
    dtw_consistent: true
```
