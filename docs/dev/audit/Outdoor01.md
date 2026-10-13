> Historical research record (through 2026-10-03). Paths are illustrative; archived runs are not bundled. For current status and executable entry points see [the repository README](../../../README.md). Current release docs/LICENSING.md supersedes historical license notes. This record does not authorize new experiments or assert current release validation.

# I2 数据审计 — Outdoor01

生成:`tools/m3dgr_audit.py`(tool_version 3),2026-09-26T13:16:53Z;bag `/work/fastlivo2_gnss/data/m3dgr/bags/Outdoor01.bag`(6547554633 字节,2026-09-24T13:15:09Z;sha256 4cf704d2358580e3ab0246e16e196d84dc3c7565842c070a19057dadcedf3801)。序列类别:**evaluation**(AMENDMENTS A2)。

> 评测集:本报告只做格式/解码审计。没有读取任何估计轨迹;不做 GNSS-only 解对接收机 PVT 的比较(§12.6-5 自检 G12a/b)与杆臂方向(§12.6-7 自检 G13),两者在评测集上 SKIP(A2);G11 只比较多普勒两种符号假设的拟合残差(解码性质)。

总评:**WARN**(INFO 8, PASS 18, SKIP 4, WARN 4)。远端归档:`/work/fastlivo2_gnss/audit/Outdoor01`。

## 本轮说明

评测集(A2),AMENDMENTS A68(2026-09-26,tool_version 3)。原包 = 远端既有 `data/m3dgr/bags/Outdoor01.bag`(未重新下载;A3 修复版本地副本,sha256 `4cf704d2358580e3ab0246e16e196d84dc3c7565842c070a19057dadcedf3801` = 修复时记下的值(官方值两者都不符,已知缺陷);t−t0 ≈ 262.5 s 处所有话题缺约 0.5 s(C3))。

过滤后 `data/m3dgr/filtered/Outdoor01.bag`(2.15 GiB,sha256 `f87005b1005bd496…`;保留 Avia LiDAR/IMU、压缩彩色图、`/ublox_driver/*`、RTK 定位与 `/odom`,逐消息核对通过)。BRDC `BRDC00IGS_R_20250030000_01D_MN.rnx`。时长 411.6 s。

**只做格式/解码审计、GT 自身审计与时间标定**:没有读取任何估计轨迹对真值的比较;自检 5(G12a/b)、自检 7(G13)与 GT 审计的回放路线 P4c 在评测集上不计算。方法②只比较 U 臂 GNSS-free 回放(A55,HEAD 零通道,A64 评测构建 `wt/a64-strict`,种子 0,`runs/eval/preproc_Outdoor01/Outdoor01/U/s0`)的天线水平速度与多普勒 LS 水平速度。GT 审计在过滤后的包上重跑,产物与在原包上的逐字节相同。

**时间标定**:method xcorr +26.1 ms、σ_toff 50.0 ms、漂移 -8.27 ppm(2 窗,峰值相关 0.999);TDCP 不暂停(A53)。

**GT 时基(A54)**:P4 WARN ⇒ method chain -31.6 ms,补偿 -31.6 ms,置信 high;`ate_exclude` 0.0 s,RTK 跳变平台 0 个。

**I2 档案**:原 P2b 的 tool_version 2 报告移到 `audit/Outdoor01/.previous/<UTC>/i2/`,在原包上用 tool_version 3 重跑进档案(`runs/a68/i2_archive_v3.sh`);各检查的判定与 v2 逐项相同,也与取数时写在 `data/m3dgr/filtered/Outdoor01.fetch/audit/` 的 v3 报告逐项相同。时间标定与 GT 审计由 `preprocess_seq.py` 重做(旧的 tool_version 1/2 文件在 `.previous/<UTC>/`),取代了 P2b 档案里写着 TDCP 暂停的方法①临时先验。

## 供 WP-a / 核心使用的结论

- `gnss.ephem.ura_semantics: metres`(A16;判定依据见 E4)。
- std 单位(§12.6-3):psr_std=metres,cp_std=cycles,dopp_std=hertz;cp_std=0.06 是 u-blox cpStdev=15 的『无效』标记,占 57.9%。
- 星历覆盖(A1):卫星-历元覆盖率 仅 bag 67.3% / bag+RINEX 94.8%;≥5 星历元比例 97.1% / 100.0% ⇒ 必须配置 `gnss.nav_rinex`。
- Klobuchar(A38):GPS 参数取 RINEX 头 GPSA/GPSB(本日可用);bag 的 iono_params 只记诊断,E6 判为 BDS-like(发布节奏 2.6 s,与 BDS D1 子帧 1 在各北斗星上轮播相符)。
- BDS 星历字段(E1 对拍 139 条):toe(GnssTimeMsg)是 GPST(139/139);`toe_tow` 字段是 BDT 周内秒的只有 0 条、`week` 字段是 GPS 周的 139 条 ⇒ 核心须按 §12.3 由 GPST toe 重算 toes(BDT)与 BDT 周,不得直接用消息的 toe_tow/week。
- 观测频点字段与信号码不一致(G3):E code 28 (L7Q (E5bQ), 1207.14 MHz) reported at 1227.6000 MHz: 30888 slots ⇒ ublox_driver 把这些槽的 freqs 写错;按 §12.3 频点表它们被拒收,主信号不受影响,只失去该系统的第二频点 GF 周跳检测(如需使用,可在 gnss_io 按 code 映射,需维护者决定)。
- 星历流一致性(E7):1 个 (sat,toe,IODE) 键出现内容不同的副本(C16:af0/af1)⇒ EphemStore 对同键新副本应比对内容,不一致即计数并保留先到/RINEX 版本,不能后到覆盖。
- C/N0 中位 BDS 28, GAL 24, GLO 28, GPS 25 dB-Hz;按 DESIGN §10 L0 掩码(伪距 30 / 多普勒 25 / 载波 28 dB-Hz)可用率 伪距 BDS 10%, GAL 3%, GLO 29%, GPS 15%、多普勒 BDS 90%, GAL 43%, GLO 84%, GPS 53% ⇒ 默认掩码会饿死滤波与初始化(COLLECT 需 ≥5 星 SPP);掩码必须在开发集按 TEST_PLAN §3 重新标定,不得在本序列上调。
- time_pulse_info 话题不存在 ⇒ A19 的检查无对象;主机钟台阶只能从 range_meas 记录时间序列看(L1)。
- range_meas 记录时间 − GPST→UTC:中位 46.9 ms、MAD 2.5 ms、漂移 3.2 ppm、台阶 无(= 主机钟偏差+到达延迟;作 §7 方法①先验,不是最终时偏)。
- 传感器时基(C4,记录时间 − 头戳中位,漂移):Avia LiDAR 107.5 ms / 11 ppm、Avia IMU 57.8 ms / 10 ppm、相机 45.2 ms。Livox 头戳 = 设备时间戳 + 常数(S1:timebase on another clock (offset 1.73589e+09 s, drift 0.0 ppm vs header)),与主机时钟的关系逐序列不同(本序列与『扫描起点主机时刻 + 约 100 ms 扫描』相容) ⇒ 估计器时基(LiDAR 头戳)对 GPST 的偏差与漂移只能由 §7 方法②逐序列标定;range_meas 记录时间只作先验。
- 载波/多普勒符号(§12.6-2):斜率 1.0000、相关 1.00000 ⇒ `λ·dΦ/dt = −λD` 成立,与 DESIGN §1 一致。
- Livox:/livox/mid360/lidar livox_ros_driver2/CustomMsg [e4d6829b], /livox/avia/lidar livox_ros_driver/CustomMsg [e4d6829b] 与仓库 include/livox_ros_driver 头 md5 一致。
- CompressedImage format:{'rgb8; jpeg compressed bgr8': 12323};回放直读应 `IMREAD_COLOR` 并按 format 决定通道序(S2)。S2 的 PASS 是 Python 对两条解码路径的仿真,C++ 回放路径本身待 WP-f 实测。
- §12.6-7 杆臂方向:评测集不做(A2,标定级判断只在开发集);calibration.md 方向推导见 docs/dev/audit/m3dgr_calib.md。
- 自检 1 的 sat2freq 交叉核对(A35-C8,G14):全部一致。

## 检查表

| # | check | verdict | evidence |
|---|---|---|---|
| C1 | msg_def md5 recomputed = connection md5 (and = installed ROS type md5 where installed) | **PASS** | 16 connections, 0 mismatches; 9 standard types also matched the installed ROS packages |
| C2 | topics of the replay/GNSS path present (DESIGN §14; since A38 the RINEX NAV file is an input of every sequence) | **PASS** | missing: /ublox_driver/time_pulse_info (the A19 time_pulse_info checks have no object) |
| C3 | rates and time ranges (header-stamp period; record-time average) | **WARN** | lidar 10.0 Hz (header), avg 9.99 Hz; imu 200.2 Hz (header), avg 203.20 Hz; compressed 30.0 Hz (header), avg 29.94 Hz; range_meas avg 9.99 Hz (4110 msgs); ephem avg 0.54 Hz (215 msgs); glo_ephem avg 0.16 Hz (62 msgs); receiver_pvt avg 9.99 Hz (4111 msgs); iono_params avg 0.38 Hz (139 msgs); gaps (>max(3x period, 0.25 s)) as (t since start or first header [s], length [s]): {'lidar': [(262.5, 0.5)], 'imu': [(262.5, 0.485)], 'compressed': [(262.5, 0.534)], 'range_meas': [(262.5, 0.6)], 'receiver_pvt': [(262.5, 0.603)]} |
| C4 | header stamp vs record time per topic (median/p1/p99 ms, drift ppm) | **INFO** | /camera/color/image_raw/compressed 45.2/29.9/61.4 ms, 1 ppm; /livox/avia/imu 57.8/7.4/107.0 ms, 10 ppm; /livox/avia/lidar 107.5/100.9/112.6 ms, 11 ppm; /mavros/global_position/raw/fix 15.6/-0.7/126.7 ms, -1 ppm; /ublox_driver/iono_params 365.6/317.3/372.4 ms, 5 ppm |
| G1 | epoch interval 10 Hz, monotone, one time tag per epoch (§12.6-1) | **PASS** | 4110 epochs; 99.98 % at 0.1 s; 0 non-increasing; 0 epochs with >1 time tag; 1 gaps >0.15 s |
| G15 | 1 Hz decimation rule (AMENDMENTS A43: the first epoch of every GPST second; the retired DESIGN §4.3 5 ms rule reported beside it) | **PASS** | A43 rule: 412 epochs for 412 GPST seconds with data; 10 Hz epoch grid offset -1.0 ms from the GPST tenths; the retired \|tow - round(tow)\| < 5 ms rule would select an epoch in 99.5% of the 412 seconds |
| G2 | per-satellite array lengths consistent; sat numbers inside the gnss_comm ranges | **PASS** | 0 inconsistent satellites; unknown sat numbers: none |
| G3 | (sys, freq, code) histogram and freqs->code table verdicts (signal_table.hpp) | **WARN** | C 1207.1400 MHz code=27->L7I: 24660; C 1561.0980 MHz code=47->L2I: 32869; E 1227.6000 MHz code=28->Unknown: 30888; E 1575.4200 MHz code=1->L1X: 32823; G 1227.6000 MHz code=17->L2X: 13867; G 1575.4200 MHz code=1->L1C: 36226; R 1244.2500 MHz code=14->L2C: 2613; R 1244.6875 MHz code=14->L2C: 4091; R 1245.5625 MHz code=14->L2C: 928; R 1246.0000 MHz code=14->L2C: 4110; R 1246.4375 MHz code=14->L2C: 40; R 1246.8750 MHz code=14->L2C: 1969; R 1247.7500 MHz code=14->L2C: 4110; R 1598.0625 MHz code=1->L1C: 5; R 1599.7500 MHz code=1->L1C: 1991; R 1600.3125 MHz code=1->L1C: 4110; R 1601.4375 MHz code=1->L1C: 3431; R 1602.0000 MHz code=1->L1C: 4110; R 1604.2500 MHz code=1->L1C: 4110; verdicts {'Accepted': 176063, 'RejectedUnknownFreq': 30888}; primary slot first: {'slot0': 119675, 'none': 2642}; freqs field inconsistent with the code field: E code 28 (L7Q (E5bQ), 1207.14 MHz) reported at 1227.6000 MHz: 30888 slots (the table rejects these slots; only the second-frequency GF slip check loses them) |
| G4 | satellites per epoch (primary signal present) | **INFO** | total min/mean/max 25/29.1/32; BDS 7/8.0/8, GAL 7/8.0/8, GPS 6/8.8/10, GLO 3/4.3/6 |
| G5 | C/N0 distribution (dB-Hz, primary signal) | **INFO** | BDS median 28 p5 23 p95 31; GAL median 24 p5 17 p95 29; GLO median 28 p5 21 p95 32; GPS median 25 p5 18 p95 33 |
| G6 | psr / cp / dopp availability (status bits + value; with the L0 C/N0 masks) | **INFO** | BDS psr 100% cp 76% dopp 100% (masked 10/50/90%); GAL psr 100% cp 25% dopp 100% (masked 3/17/43%); GLO psr 100% cp 72% dopp 100% (masked 29/51/84%); GPS psr 100% cp 40% dopp 100% (masked 15/31/53%) |
| G7 | LLI / status statistics | **INFO** | LLI {'GPS': {'2': 1423, '0': 13536, '3': 21257, '1': 10}, 'BDS': {'0': 24286, '2': 1206, '3': 7364, '1': 13}, 'GLO': {'0': 12822, '3': 4526, '2': 397, '1': 12}, 'GAL': {'0': 8607, '3': 24212, '1': 4}}; status {'GPS': {'3': 1411, '15': 9902, '7': 3644, '1': 21269}, 'BDS': {'7': 15890, '15': 8409, '3': 1192, '1': 7378}, 'GLO': {'15': 10813, '1': 4539, '3': 384, '7': 2021}, 'GAL': {'7': 8601, '1': 24212, '5': 10}} |
| G8 | std unit judgement on the u-blox grids (§12.6-3) | **PASS** | psr metres (100.0% on 0.01*2^n m); cp cycles (100.0% on 0.004n cyc, 0.0% on 0.004n*lambda m, 0.0% on 0.004n/lambda); dopp hertz (100.0% on 0.002*2^n Hz, 0.0% as m/s); cp_std=0.06 (cpStdev=15, u-blox 'invalid') on 57.9% of slots |
| G9 | carrier-phase rate vs Doppler sign/scale (§12.6-2: corr > 0.99, slope ~ 1) | **PASS** | 59898 pairs (clean (LLI bit0 = 0)): corr 1.00000, slope 1.0000, median \|diff\| 0.047 m/s; per satellite (§12.6-2 '每星', decisive = >= 50 clean pairs and range-rate std >= 2 m/s): 18/18 decisive satellites with corr > 0.99 and \|slope - 1\| < 0.02 (slopes 0.9971..1.0004), 8 not decisive; 1 s sign agreement lambda*dPhi vs dP 100.0%, vs -lambda*D*dt 100.0% (n=5360) |
| G10 | GLONASS channel k from freqs vs ephemeris frq (§12.3, §12.6-1) | **PASS** | 35618 slots; mismatching satellites none; observed without any ephemeris: none |
| G14 | self-check 1: RTKLIB sat2freq(sat, code, nav) of every accepted slot = its freqs (A35-C8) | **PASS** | 176063 slots consistent, 0 mismatching, 0 unverifiable (GLONASS without an ephemeris channel: none), 30888 slots not accepted by the table (G3); per system {'BDS': {'consistent': 57529}, 'GAL': {'consistent': 32823, 'not_accepted': 30888}, 'GPS': {'consistent': 50093}, 'GLO': {'consistent': 35618}} |
| E1 | bag Kepler ephemerides vs same-day RINEX (same sat,toe): \|dr\| < 1e-3 m, \|ddts\| < 1e-11 s (A9-3 tier 2) | **PASS** | 215/215 matched, 2 outside tier 2 (0 not explained by a corrupted copy, see E7); worst per system {'G': '1.44e-04 m / 4.86e-16 s', 'E': '1.69e-04 m / 4.28e-16 s', 'C': '2.55e-04 m / 4.84e-06 s'}; toe label shifts {'G:+0': 50, 'E:+0': 26, 'C:+0': 139}; field semantics {'G': {'n': 50, 'toe_tow_is_system_tow': 50, 'week_is_system_week': 50, 'week_is_gps_week': 50, 'toe_msg_is_gpst': 50, 'tgd0_eq': 50, 'tgd1_eq': 50, 'tgd0_eq_rnx_tgd1': 0, 'iode_eq': 50, 'svh_eq': 50, 'code_eq': 0}, 'E': {'n': 26, 'toe_tow_is_system_tow': 26, 'week_is_system_week': 26, 'week_is_gps_week': 26, 'toe_msg_is_gpst': 26, 'tgd0_eq': 26, 'tgd1_eq': 26, 'tgd0_eq_rnx_tgd1': 7, 'iode_eq': 26, 'svh_eq': 26, 'code_eq': 26}, 'C': {'n': 139, 'toe_tow_is_system_tow': 0, 'week_is_system_week': 0, 'week_is_gps_week': 139, 'toe_msg_is_gpst': 139, 'tgd0_eq': 139, 'tgd1_eq': 139, 'tgd0_eq_rnx_tgd1': 17, 'iode_eq': 139, 'svh_eq': 139, 'code_eq': 139}} |
| E7 | bag ephemeris stream consistency: all copies of one (sat, toe, IODE) identical | **WARN** | 20 keys, 1 with conflicting copies: C16 toe 2025-01-03T06:59:56.000Z IODE 1: 2/19 copies differ in af0,af1 (af1 ['-1.43e-09', '-1.43e-09']) — a store keyed by (sat, toe, IODE) that lets later copies overwrite would take a corrupted clock (A1: the RINEX entry wins on the same key) |
| E2 | bag GLONASS ephemerides vs RINEX (pos/vel/acc units, tau_n sign, channel) | **PASS** | 62/62 matched; worst \|dr\| at toe+600 s 6.62e-05 m; semantics {'n': 62, 'taun_same_sign_as_rtklib': 62, 'taun_opposite_sign': 0, 'gamn_eq': 62, 'frq_eq': 62, 'pos_eq_1mm': 62} |
| E4 | URA semantics: bag ura vs same-day RINEX SV accuracy (§12.6-4, A16) | **PASS** | decision per system {'G': 'metres', 'E': 'metres', 'C': 'metres'} -> gnss.ephem.ura_semantics: metres; counts {'G': {'n': 50, 'metres_exact': 50, 'metres_approx': 50, 'index': 0, 'icd_formula_metres': 0}, 'E': {'n': 26, 'metres_exact': 26, 'metres_approx': 26, 'index': 0}, 'C': {'n': 139, 'metres_exact': 139, 'metres_approx': 139, 'index': 0, 'icd_formula_metres': 0}}; samples (bag, rinex m) {'G': [(2.0, 2.0), (2.0, 2.0), (2.0, 2.0), (2.0, 2.0), (2.0, 2.0), (2.0, 2.0), (2.0, 2.0), (2.0, 2.0)], 'E': [(3.12, 3.12), (3.12, 3.12), (3.12, 3.12), (3.12, 3.12), (3.12, 3.12), (3.12, 3.12), (3.12, 3.12), (3.12, 3.12)], 'C': [(2.0, 2.0), (2.0, 2.0), (2.0, 2.0), (2.0, 2.0), (2.0, 2.0), (2.0, 2.0), (2.0, 2.0), (2.0, 2.0)]} |
| E3 | orbit code self-check: adjacent RINEX records agree at the midpoint (p95 < 10 m per system) | **PASS** | C median 0.12 m p95 0.44 m (n=260); E median 0.11 m p95 0.47 m (n=647); G median 0.38 m p95 1.48 m (n=97) |
| E5 | ephemeris coverage timeline: bag-only (causal by record time) vs bag+RINEX (A1, §12.6-8) | **INFO** | sat-epochs with a healthy ephemeris: bag-only 67.3%, bag+RINEX 94.8%; epochs with >=5: 97.1% / 100.0%; first >=5 at 12.0 / 0.0 s; BDS GEO observed none; never covered bag-only: E08, E14, E27, G01, G02, G21, G30 |
| E6 | Klobuchar source (A38): GPS set = RINEX header GPSA/GPSB; bag iono_params only a diagnostic (family classified) | **PASS** | GPS Klobuchar from the RINEX header GPSA/GPSB: ['2.794e-08', '2.235e-08', '-1.192e-07', '5.96e-08', '1.516e+05', '-1.966e+05', '-6.554e+04', '3.277e+05']; bag iono_params (diagnostic, not used, A38): 139 messages, 1 distinct sets, one every 2.6 s on average; set ['2.235e-08', '3.502e-07', '-2.086e-06', '2.742e-06', '1.946e+05', '-8.684e+05', '2.359e+06', '-6.554e+05']: BDS-like (distance to RINEX GPSA/GPSB 33 LSB, to the nearest RINEX BDSA/BDSB 7 LSB) |
| L1 | GNSS arrival latency: range_meas record time - GPST->UTC (leap 18 s) | **INFO** | median 46.9 ms, MAD 2.5, p1 42.5, p99 54.4, min 42.0, max 55.2; drift 3.2 ppm; persistent steps >50 ms: none; receiver_pvt median 79.6 ms |
| L3 | self-check 6 (time, record-time part): \|record - (GPST - leap)\| < 0.3 s (leap seconds and time system consistent) | **PASS** | median +0.0469 s with leap 18 s (a missing/extra leap second would show as ~±1 s, a GPST/UTC mix-up as ±18 s); the cross-correlation part of self-check 6 is method 2 of tools/time_calib/gnss_time_calib.py on the ref-R replay |
| L2 | receiver_lla header stamp convention | **INFO** | header.stamp - GPST(seconds, Unix origin) median 0.000 s (min 0.000, max 0.000): receiver_lla stamps are GPST without leap seconds (not ROS/UTC time) |
| T1 | time_pulse_info checks (A19: leap second, time system, gross errors, host clock steps) | **SKIP** | topic /ublox_driver/time_pulse_info absent from this bag (README §4.2 lists it); nothing to check. Host-clock steps are covered by the range_meas latency series instead (L1). |
| G11 | Doppler sign by LS fit: -lambda*D = range rate (§1) vs flipped sign (all Dopplers, el >= 5°) | **PASS** | post-fit RMS median 0.112 m/s with the design sign vs 720.2 m/s flipped (411 epochs at 1.0 Hz; linearised at the receiver PVT (evaluation sequence, A2)) |
| G12a | self-check 5 (position): SPP vs receiver PVT | **SKIP** | A2: not a development sequence — self-check 5 compares GNSS-only solutions with the receiver PVT (a calibration-grade judgement, TEST_PLAN §3: development set only); not computed |
| G12b | self-check 5 (velocity): Doppler LS vs receiver PVT | **SKIP** | A2: not a development sequence — self-check 5 compares GNSS-only solutions with the receiver PVT (a calibration-grade judgement, TEST_PLAN §3: development set only); not computed |
| G13 | lever-arm direction (§12.6-7): Doppler innovations with l vs -l on turns; regression on the omega x l term (~0; ~-2 = sign flipped) | **SKIP** | A2: not a development sequence — the lever-arm judgement uses the replay trajectory and belongs to the development set; not computed |
| S1 | Livox CustomMsg type/md5 vs the repository's livox_ros_driver headers | **PASS** | /livox/avia/lidar: livox_ros_driver/CustomMsg [e4d6829bdfe657cb6c21a746c86b21a6] vs repo livox_ros_driver/CustomMsg [e4d6829bdfe657cb6c21a746c86b21a6]; timebase on another clock (offset 1.73589e+09 s, drift 0.0 ppm vs header); points/scan median 24000 |
| S2 | CompressedImage format; replay direct decode vs republish path, bgr8 and grey pixel-exact (I2) | **PASS** | formats {'rgb8; jpeg compressed bgr8': 12323}; 150 sampled: bgr equal 150, grey equal 150, IMREAD_UNCHANGED==IMREAD_COLOR 150; shapes {'(480, 640, 3)': 150} (Python emulation of both decode paths with OpenCV; the C++ replay path itself is to be measured by WP-f) |
| S3 | GT file format (TUM: t x y z qx qy qz qw) | **WARN** | 6174 rows, columns {8: 6174}; 3448 non-increasing stamps (135 backwards, max 19.1 us, 3313 exact duplicates); quaternion identity on 100.0% rows (orientation not provided); repeated positions 66.7%; GT stamps = /mavros/global_position/raw/fix header stamps on 99.9% of rows |

## 话题清单

| topic | type | md5 | md5 recomputed | msgs | rate [Hz] (header, else avg) | record-header median [ms] |
|---|---|---|---|---|---|---|
| /livox/mid360/imu | sensor_msgs/Imu | 6a62c6daae103f4ff57a132d6f95cec2 | yes | 82213 | 201.22 | 7.6e-02 |
| /camera/imu | sensor_msgs/Imu | 6a62c6daae103f4ff57a132d6f95cec2 | yes | 82223 | 200.04 | 0.4 |
| /mavros/global_position/raw/fix | sensor_msgs/NavSatFix | 2d3a8cd499b9b4a0249fb98fd05cfa48 | yes | 6166 | 14.98 | 15.6 |
| /camera/aligned_depth_to_color/image_raw/compressedDepth | sensor_msgs/CompressedImage | 8f7a12909da2c9d3332d540a0977563f | yes | 12323 | 29.98 | 43.6 |
| /ublox_driver/receiver_pvt | gnss_comm/GnssPVTSolnMsg | d18171357d7a159f76d4d7c0b12fb631 | yes | 4111 | 9.99 | — |
| /ublox_driver/receiver_lla | sensor_msgs/NavSatFix | 2d3a8cd499b9b4a0249fb98fd05cfa48 | yes | 4111 | 10.00 | -17920.4 |
| /camera/color/image_raw/compressed | sensor_msgs/CompressedImage | 8f7a12909da2c9d3332d540a0977563f | yes | 12323 | 29.98 | 45.2 |
| /odom | nav_msgs/Odometry | cd5e73d190d741a2f92e81eda573aca7 | yes | 8221 | 20.00 | 8.3e-02 |
| /livox/mid360/lidar | livox_ros_driver2/CustomMsg | e4d6829bdfe657cb6c21a746c86b21a6 | yes | 4111 | 10.00 | 100.5 |
| /livox/avia/imu | sensor_msgs/Imu | 6a62c6daae103f4ff57a132d6f95cec2 | yes | 83599 | 200.19 | 57.8 |
| /livox/avia/lidar | livox_ros_driver/CustomMsg | e4d6829bdfe657cb6c21a746c86b21a6 | yes | 4111 | 10.02 | 107.5 |
| /cv_camera/image_raw/compressed | sensor_msgs/CompressedImage | 8f7a12909da2c9d3332d540a0977563f | yes | 6166 | 15.01 | 22.9 |
| /ublox_driver/range_meas | gnss_comm/GnssMeasMsg | 0eafeb530a3e6637722c4e296734c119 | yes | 4110 | 9.99 | — |
| /ublox_driver/ephem | gnss_comm/GnssEphemMsg | b4346f99e431c870e392a438684c0fe8 | yes | 215 | 0.54 | — |
| /ublox_driver/glo_ephem | gnss_comm/GnssGloEphemMsg | 6893dee0d8b1c9325a6693f3488e0144 | yes | 62 | 0.16 | — |
| /ublox_driver/iono_params | gnss_comm/StampedFloat64Array | fb60495edd59d3fcf90e173153ae8a9a | yes | 139 | 0.38 | 365.6 |

## 信号直方图 (sys, freq, code 字段 → signal_table)

| sys | freq [MHz] | code field | table code | count |
|---|---|---|---|---|
| C | 1207.1400 | 27 | L7I | 24660 |
| C | 1561.0980 | 47 | L2I | 32869 |
| E | 1227.6000 | 28 | Unknown | 30888 |
| E | 1575.4200 | 1 | L1X | 32823 |
| G | 1227.6000 | 17 | L2X | 13867 |
| G | 1575.4200 | 1 | L1C | 36226 |
| R | 1244.2500 | 14 | L2C | 2613 |
| R | 1244.6875 | 14 | L2C | 4091 |
| R | 1245.5625 | 14 | L2C | 928 |
| R | 1246.0000 | 14 | L2C | 4110 |
| R | 1246.4375 | 14 | L2C | 40 |
| R | 1246.8750 | 14 | L2C | 1969 |
| R | 1247.7500 | 14 | L2C | 4110 |
| R | 1598.0625 | 1 | L1C | 5 |
| R | 1599.7500 | 1 | L1C | 1991 |
| R | 1600.3125 | 1 | L1C | 4110 |
| R | 1601.4375 | 1 | L1C | 3431 |
| R | 1602.0000 | 1 | L1C | 4110 |
| R | 1604.2500 | 1 | L1C | 4110 |

## 星历字段语义(bag vs 同日 RINEX,按 (sat,toe) 配对的计数)

| sys | code_eq | iode_eq | n | svh_eq | tgd0_eq | tgd0_eq_rnx_tgd1 | tgd1_eq | toe_msg_is_gpst | toe_tow_is_system_tow | week_is_gps_week | week_is_system_week |
|---|---|---|---|---|---|---|---|---|---|---|---|
| C | 139 | 139 | 139 | 139 | 139 | 17 | 139 | 139 | 0 | 139 | 0 |
| E | 26 | 26 | 26 | 26 | 26 | 7 | 26 | 26 | 26 | 26 | 26 |
| G | 0 | 50 | 50 | 50 | 50 | 0 | 50 | 50 | 50 | 50 | 50 |

## 产物

`audit.json`(全部数字)、`latency.csv`、`eph_coverage.csv`、`eph_coverage_timeline.csv`、`plot_*.png`,位于远端归档目录。

## GT 审计(tools/gt_audit.py)

生成:`tools/gt_audit.py`(tool_version 3),2026-09-26T13:11:56Z。输入:GT 文件与原始 bag 的 RTK 定位话题、9Ps 接收机 PVT、Avia IMU、轮速计(A2:不读任何估计轨迹)。

总评:**WARN**(INFO 5, PASS 3, WARN 1)。远端归档:`/work/fastlivo2_gnss/audit/Outdoor01/gt`。

### 检查表

| # | check | verdict | evidence |
|---|---|---|---|
| P0 | GT file format and cleaning | **INFO** | 6174 rows; 135 backward stamp steps; 270 rows reordered and 3313 duplicate-stamp rows dropped in gt_clean.tum; orientation identity on 100.0% rows (position-only GT) |
| P1 | GT provenance: stamps and projection reproduced from the RTK fix (C-RTK2HP) | **PASS** | 99.9% of GT stamps equal /mavros/global_position/raw/fix header stamps (1 us); best hypothesis 'ENU at the first GT fix (exact, WGS-84)' reproduces GT to 8.31e-04 m RMS (max 1.91e-03 m); other hypotheses: UTM zone 49 (E, N, alt) minus first 1.71 m; equirectangular (R=6378137 m) minus first 0.0997 m; ENU of GCJ-02-shifted lat/lon 0.0405 m |
| P4 | GT time base vs the estimator clock (Avia IMU header) by cross-correlation; > 20 ms compensated | **WARN** | GT stamp -> Avia IMU header clock: -31.6 ms by chain GT->odom (speed) + odom->IMU (yaw rate); > 20 ms: gt_audited.tum shifted by it; record-time plausibility -42.2 ms (assumes equal transport delays; agreement within 50 ms required); antenna-lead fit (no odometry) L -53.6 ms, r_x +0.33 m (median speed 0.91 m/s, n=1873; r_x = 0 gives +176.7 ms); raw direct course-rate link +171.2 ms (peak 0.84): +202.8 ms against the chain = antenna lead r_x/v, not a clock term; links: gt_course_rate__imu_yaw_rate +171.2 ms (peak 0.842, delay precision 9.3 ms); gt_speed__odom_speed -5.0 ms (peak 0.993, delay precision 72.0 ms); odom_yaw_rate__imu_yaw_rate -26.6 ms (peak 0.866, delay precision 4.3 ms); gt_speed__pvt_speed_utc +65.2 ms (peak 0.990, delay precision 45.5 ms) |
| P2 | datum: RTK fix (GT source) lat/lon vs 9Ps WGS-84 PVT | **PASS** | median horizontal difference 1.1 m vs a GCJ-02 shift of 466 m at this location ⇒ WGS-84 (no GCJ-02 offset) |
| P3 | GT (local XYZ) vs 9Ps PVT in ENU at the GT origin: horizontal SE(2)/Sim(2) structure (ENU, no rotation/scale expected) | **PASS** | SE(2) yaw -0.171° (block-bootstrap σ 0.241°), Sim(2) scale 0.99635 (σ 0.00121); residual trend dE/dE -4.1 dN/dN -16.9 m/km; n=4109 (residual magnitudes are the 9Ps PVT noise, not a result, A2) |
| P5 | pre-registered trust mask (9Ps h_acc > 1 m or num_sv < 8; speed spikes > 3σ; gaps/stalls > 0.5 s; each ±2 s) | **INFO** | trusted 56.3% (GT-derived criteria alone: 98.8%); masked by h_acc>1 m 43.7%, num_sv<8 0.0% (receiver criteria before the ±2 s widening 42.7%), speed spikes 1.2% (5; σ 0.015 m/s, floor 0.03), gaps/stalls 0.0% (0/0) |
| P7 | RTK jumps / spikes (A54): fix-to-fix residual against the local velocity > max(0.5 m, 10σ); a jump and its return = displaced plateau, masked ±2 s | **INFO** | 0 jump(s) (thresholds h 0.50 / v 0.50 m; residual σ h 0.009 / v 0.015 m over 2056 fix intervals); masked 0.0 s of 411.4 s (0.00%); ATE-excluded (gaps/stalls + RTK) 0.0 s |
| P4r | GT time base used by the evaluation (AMENDMENTS A54 rule) | **INFO** | method chain: -31.6 ms (applied shift -31.6 ms, > 20 ms rule), confidence high, uncertainty 50.0 ms (peak-width precisions of the two chain links combined (Carter bound, 72.1 ms), capped by the 50 ms agreement with the record-time value that P4 PASS requires) |
| P6 | 2HP antenna lever arm | **INFO** | unknown (the C-RTK2HP antenna is not in calibration.md): an ATE floor of the order of the antenna separation on the top plate (~0.1-0.3 m, README Fig. 1); TEST_PLAN E1' estimates it jointly |

### GT 生成方式假设(对 RTK 定位逐点复算)

| hypothesis | direct RMS [m] | direct max [m] | Sim3 scale | Sim3 yaw [°] | Sim3 RMS [m] |
|---|---|---|---|---|---|
| ENU at the first GT fix (exact, WGS-84) | 8.31e-04 | 0.002 | 1.000 | 4.17e-04 | 1.74e-04 |
| UTM zone 49 (E, N, alt) minus first | 1.707 | 3.102 | 1.000 | 1.253 | 1.15e-04 |
| equirectangular (R=6378137 m) minus first | 0.100 | 0.178 | 1.001 | 0.041 | 0.050 |
| ENU of GCJ-02-shifted lat/lon | 0.041 | 0.075 | 1.000 | 0.028 | 0.005 |

GCJ-02 在本地的绝对平移 466 m,轨迹范围内局部畸变 0.040 m。

### 时基互相关(GT 戳 t 对应其它时钟 t + lag)

| reference | lag [ms] | peak corr | peak width [ms] | delay precision [ms] |
|---|---|---|---|---|
| gt_course_rate__imu_yaw_rate | 171.168 | 0.842 | 760.000 | 9.295 |
| gt_speed__odom_speed | -4.989 | 0.993 | 1360.000 | 71.984 |
| odom_yaw_rate__imu_yaw_rate | -26.634 | 0.866 | 730.000 | 4.277 |
| gt_speed__pvt_speed_utc | 65.241 | 0.990 | 1330.000 | 45.454 |

### 天线超前模型(P4 诊断,不经轮速计)

航向角速率 = ω(t+L) + r_x·d/dt(ω/v):L = -53.6 ms,r_x = +0.33 m(车速中位 0.91 m/s,n = 1873);r_x 置 0 时 L = +176.7 ms。天线在无侧滑点前方 r_x 处时 GT 航向**领先**车头 r_x/v,原始直接链路相对链式结果为正即此效应,不是钟差。

### 评测用 GT 时基(AMENDMENTS A54,`gt_timebase.yaml`)

规则:P4 通过 ⇒ 轮速计链;P4 失败 ⇒ 记录时间旁证;两者都没有 ⇒ 0 并标记;补偿按 TEST_PLAN §4(|值| > 20 ms 才补偿)。

| method | value [ms] | applied shift [ms] | confidence | uncertainty [ms] | P4 | chain [ms] | record time [ms] | sub-threshold chain [ms] | P4c replay (dev only, not used) [ms] |
|---|---|---|---|---|---|---|---|---|---|
| chain | -31.6 | -31.6 | high | 50.0 | WARN | -31.6 | -42.2 | — | — |

### RTK 跳变/毛刺与被掩时长(AMENDMENTS A54)

逐定位差分对局部速度(前后各 3 个定位区间的中位速度)的残差;跳变 = 水平 > 0.50 m 或垂直 > 0.50 m(max(0.5 m, 10σ),σ 水平 0.009 / 垂直 0.015 m);一对相互抵消的跳变之间为错位平台,连同前后 2 s 掩掉(bit 64)。

无跳变。

| criterion | masked [s] | of bag span [%] |
|---|---|---|
| h_acc>1m | 180.800 | 43.946 |
| num_sv<8 | 0.000 | 0.000 |
| speed_spike | 4.999 | 1.215 |
| gap_or_stall | 0.000 | 0.000 |
| no_pvt_within_0.2s | 0.199 | 0.048 |
| rtk_jump_plateau | 0.000 | 0.000 |
| ate_exclude | 0.000 | 0.000 |
| pre_registered_all | 180.800 | 43.946 |

bag 区间内 GT 时长 411.4 s;`ate_exclude` = 间断/冻结 + RTK 跳变平台(bit 8|64;bag 区间外 bit 32 本就不参评),区间见 `gt_mask_intervals.csv`(GT 戳与审计后时基两列)。h_acc/num_sv/速度尖峰(bit 1/2/4)只报告,不标 ATE 排除(未裁定)。

### 产物

`gt_clean.tum`、`gt_audited.tum`(= gt_clean + -0.032 s,方法 chain)、`gt_timebase.yaml`、`trust_mask.csv`、`gt_mask_intervals.csv`、`gt_audit.json`。

## 时间标定(tools/time_calib/gnss_time_calib.py,DESIGN §7)

### 方法①(记录时间统计与估计器时钟先验)

range_meas 记录时间 − GPST→UTC(leap 18 s,主机钟视角 = 主机钟偏差 + GNSS 传输延迟):中位 46.9 ms、MAD 2.5 ms、p5/p95 43.1/52.6 ms、漂移 3.2 ppm、持续台阶(>50 ms)无。

头戳 − 记录时间(判定 LiDAR 时基):/camera/color/image_raw/compressed 中位 -45.2 ms(p1 -61.4 / p99 -29.9,漂移 -0.8 ppm);/livox/avia/imu 中位 -57.8 ms(p1 -107.0 / p99 -7.4,漂移 -9.6 ppm);/livox/avia/lidar 中位 -107.5 ms(p1 -112.6 / p99 -100.9,漂移 -10.5 ppm)。

估计器时钟(Livox 头戳)先验:GNSS「记录 − UTC」下包络 45.4 ms 减 Avia IMU「记录 − 头戳」下包络 12.1 ms ⇒ 上界 33.9 ms(主机钟台阶在两者中抵消;漂移 -8.3 ppm;估计器时钟台阶 无);GNSS 传输延迟不可观,按 [0, 150] ms 均匀(推断)⇒ 先验 -41.1 ms、σ 75 ms。

方法①文件 `Outdoor01_record_time.yaml`(不读回放;method record_time,σ 75 ms;tool_version ≥ 3 按 AMENDMENTS A53 TDCP 不暂停,tool_version ≤ 2 的文件仍写着暂停):

```yaml
sequence: Outdoor01
tool_version: 3
leap_s: 18
segments:
  - t_from_gpst_s: 0
    offset_s: -0.041084478952223516
    drift_ppm: -8.265746503679896
    t_ref_gpst_s: 1735888231.549
    sigma_toff_s: 0.075
    tdcp_paused: false
    method: record_time
```

### 方法②(水平天线速度互相关)与方法③(DTW)

输入:回放轨迹 `/work/fastlivo2_gnss/runs/eval/preproc_Outdoor01/Outdoor01/U/s0/traj_W.tum`(tum,12311 个速度样本,估计器速度 std 0.25 m/s);多普勒 LS 速度 4110 个历元。窗口 12 个,可用 2 个(窗长/步长见 JSON)。状态:ran。

信号(tool_version 3):horizontal antenna speed = |horizontal part of the zero-phase Gaussian-smoothed (0.25 s) velocity vector| on both sides, 100 Hz grid;W 的竖直方向:state_trace gravity (median g = [-0.4158, 0.2197, -9.7992])。多普勒 LS:C/N0 ≥ 15.0 dB-Hz、仰角 ≥ 10.0°,按接收机 dopp_std 加权,两遍(第二遍固定平滑后的钟漂):4110 个历元有 ≥ 4 行,第二遍解出 4110 个(每历元用行中位 27.0)。

| 窗起点 [s] | 用 | 峰值相关 | 偏移 [ms] | 峰宽精度 [ms] | GNSS 覆盖 | 原因 |
|---|---|---|---|---|---|---|
| 0 | 是 | 0.999 | 31.7 | 57.2 | 100% |  |
| 30 | 否 | — | — | — | 100% | insufficient excitation (speed std < 0.1 m/s) or data |
| 60 | 否 | — | — | — | 100% | insufficient excitation (speed std < 0.1 m/s) or data |
| 90 | 否 | — | — | — | 100% | insufficient excitation (speed std < 0.1 m/s) or data |
| 120 | 否 | — | — | — | 100% | insufficient excitation (speed std < 0.1 m/s) or data |
| 150 | 否 | — | — | — | 100% | insufficient excitation (speed std < 0.1 m/s) or data |
| 180 | 否 | — | — | — | 100% | insufficient excitation (speed std < 0.1 m/s) or data |
| 210 | 否 | — | — | — | 100% | insufficient excitation (speed std < 0.1 m/s) or data |
| 240 | 否 | — | — | — | 100% | insufficient excitation (speed std < 0.1 m/s) or data |
| 270 | 否 | — | — | — | 100% | insufficient excitation (speed std < 0.1 m/s) or data |
| 300 | 否 | — | — | — | 100% | insufficient excitation (speed std < 0.1 m/s) or data |
| 330 | 是 | 0.999 | 20.5 | 27.9 | 100% |  |

互相关给出的段(写入文件的以状态行为准):

| t_from_gpst_s | offset_s | drift_ppm | t_ref_gpst_s | sigma_toff_s | tdcp_paused | method | peak_corr | window_scatter_ms | n_windows | n_windows_rejected | n_windows_isolated | bootstrap_sd_ms | peak_width_sigma_ms | dtw_offset_s | dtw_consistent |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 0 | 26.12 ms | -8.266 | 1735888222.242 | 50.00 ms | false | xcorr | 0.9991 | 5.986 | 2 | 0 | 0 | -1 | 42.54 | 26.12 ms | true |

方法③ DTW 交叉核对:DTW 偏移 26.12 ms(路径偏移 MAD 300.0 ms)对 xcorr 26.12 ms,一致(>3σ 才告警)。

警告:window scatter 6.0 ms > 5 ms in segment from 0.0;segment from 0.0 has 2 window(s): sigma 50 ms。

标定文件(方法②) `time_calib/Outdoor01.yaml`(AMENDMENTS A18 格式;所有 `*_gpst_s` 为 GTime::toSec(),1970 原点、GPST 尺度):

```yaml
sequence: Outdoor01
tool_version: 3
leap_s: 18
segments:
  - t_from_gpst_s: 0
    offset_s: 0.026124575792843843
    drift_ppm: -8.265746503679896
    t_ref_gpst_s: 1735888222.2423997
    sigma_toff_s: 0.05
    tdcp_paused: false
    method: xcorr
    peak_corr: 0.9991206495365035
    window_scatter_ms: 5.985740388531038
    n_windows: 2
    n_windows_rejected: 0
    n_windows_isolated: 0
    bootstrap_sd_ms: -1.0
    peak_width_sigma_ms: 42.543335059583214
    short_segment: true
    drift_source: method1_prior
    dtw_offset_s: 0.026124575792843843
    dtw_consistent: true
```
