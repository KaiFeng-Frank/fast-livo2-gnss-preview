> Historical research record (through 2026-10-03). Paths are illustrative; archived runs are not bundled. For current status and executable entry points see [the repository README](../../../README.md). Current release docs/LICENSING.md supersedes historical license notes. This record does not authorize new experiments or assert current release validation.

# I2 数据审计 — Dark02

生成:`tools/m3dgr_audit.py`(tool_version 3),2026-09-26T11:52:42Z;bag `/work/fastlivo2_gnss/data/m3dgr/fetch/Dark02/x/Dark02.bag`(8130714988 字节,2024-12-24T17:02:48Z;sha256 未计算)。序列类别:**evaluation**(AMENDMENTS A2)。

> 评测集:本报告只做格式/解码审计。没有读取任何估计轨迹;不做 GNSS-only 解对接收机 PVT 的比较(§12.6-5 自检 G12a/b)与杆臂方向(§12.6-7 自检 G13),两者在评测集上 SKIP(A2);G11 只比较多普勒两种符号假设的拟合残差(解码性质)。

总评:**WARN**(INFO 7, PASS 20, SKIP 4, WARN 3)。远端归档:`/work/fastlivo2_gnss/audit/Dark02`。

## 本轮说明

评测集(A2),AMENDMENTS A68(2026-09-26,tool_version 3)。原包 = 阿里云盘 `/来自分享/Dataset/Visual_Challenge/Outdoor/Dark02`(3 段分卷,cat 拼接后 7z 解 SFX),sha256 与官方一致,已索引;审计后删除。

过滤后 `data/m3dgr/filtered/Dark02.bag`(3.76 GiB,sha256 `96bec070f8a038fe…`;保留 Avia LiDAR/IMU、压缩彩色图、`/ublox_driver/*`、RTK 定位与 `/odom`,逐消息核对通过)。BRDC `BRDC00IGS_R_20243300000_01D_MN.rnx`。时长 710.3 s。

**只做格式/解码审计、GT 自身审计与时间标定**:没有读取任何估计轨迹对真值的比较;自检 5(G12a/b)、自检 7(G13)与 GT 审计的回放路线 P4c 在评测集上不计算。方法②只比较 U 臂 GNSS-free 回放(A55,HEAD 零通道,A64 评测构建 `wt/a64-strict`,种子 0,`runs/eval/preproc_Dark02/Dark02/U/s0`)的天线水平速度与多普勒 LS 水平速度。GT 审计在过滤后的包上重跑,产物与在原包上的逐字节相同。

**时间标定**:method record_time +26.3 ms、σ_toff 75.0 ms、漂移 -4.21 ppm(方法②无可用窗,取方法①先验);TDCP 不暂停(A53)。

**GT 时基(A54)**:P4 FAIL ⇒ method record_time +126.4 ms,补偿 +126.4 ms,置信 medium;`ate_exclude` 0.0 s,RTK 跳变平台 0 个。

## 供 WP-a / 核心使用的结论

- `gnss.ephem.ura_semantics: metres`(A16;判定依据见 E4)。
- std 单位(§12.6-3):psr_std=metres,cp_std=cycles,dopp_std=hertz;cp_std=0.06 是 u-blox cpStdev=15 的『无效』标记,占 84.5%。
- 星历覆盖(A1):卫星-历元覆盖率 仅 bag 39.7% / bag+RINEX 100.0%;≥5 星历元比例 0.0% / 34.2% ⇒ 必须配置 `gnss.nav_rinex`。
- Klobuchar(A38):GPS 参数取 RINEX 头 GPSA/GPSB(本日可用);bag 无 iono_params。
- C/N0 中位 GLO 18, GPS 20 dB-Hz;按 DESIGN §10 L0 掩码(伪距 30 / 多普勒 25 / 载波 28 dB-Hz)可用率 伪距 GLO 0%, GPS 0%、多普勒 GLO 45%, GPS 16% ⇒ 默认掩码会饿死滤波与初始化(COLLECT 需 ≥5 星 SPP);掩码必须在开发集按 TEST_PLAN §3 重新标定,不得在本序列上调。
- time_pulse_info 话题不存在 ⇒ A19 的检查无对象;主机钟台阶只能从 range_meas 记录时间序列看(L1)。
- range_meas 记录时间 − GPST→UTC:中位 -70.3 ms、MAD 16.1 ms、漂移 105.0 ppm、台阶 无(= 主机钟偏差+到达延迟;作 §7 方法①先验,不是最终时偏)。
- 传感器时基(C4,记录时间 − 头戳中位,漂移):Avia LiDAR -76.0 ms / 109 ppm、Avia IMU -117.0 ms / 107 ppm、相机 44.3 ms。Livox 头戳 = 设备时间戳 + 常数(S1:timebase on another clock (offset 1.73254e+09 s, drift 0.0 ppm vs header)),与主机时钟的关系逐序列不同(本序列 LiDAR 头戳领先主机收包时刻,不可能是主机收包戳) ⇒ 估计器时基(LiDAR 头戳)对 GPST 的偏差与漂移只能由 §7 方法②逐序列标定;range_meas 记录时间只作先验。
- 载波/多普勒符号(§12.6-2):斜率 1.0000、相关 1.00000 ⇒ `λ·dΦ/dt = −λD` 成立,与 DESIGN §1 一致。
- Livox:/livox/avia/lidar livox_ros_driver/CustomMsg [e4d6829b], /livox/mid360/lidar livox_ros_driver2/CustomMsg [e4d6829b] 与仓库 include/livox_ros_driver 头 md5 一致。
- CompressedImage format:{'rgb8; jpeg compressed bgr8': 21292};回放直读应 `IMREAD_COLOR` 并按 format 决定通道序(S2)。S2 的 PASS 是 Python 对两条解码路径的仿真,C++ 回放路径本身待 WP-f 实测。
- §12.6-7 杆臂方向:评测集不做(A2,标定级判断只在开发集);calibration.md 方向推导见 docs/dev/audit/m3dgr_calib.md。
- 自检 1 的 sat2freq 交叉核对(A35-C8,G14):全部一致。

## 检查表

| # | check | verdict | evidence |
|---|---|---|---|
| C1 | msg_def md5 recomputed = connection md5 (and = installed ROS type md5 where installed) | **PASS** | 14 connections, 0 mismatches; 8 standard types also matched the installed ROS packages |
| C2 | topics of the replay/GNSS path present (DESIGN §14; since A38 the RINEX NAV file is an input of every sequence) | **PASS** | missing: /ublox_driver/iono_params (no effect: the GPS Klobuchar set comes from the RINEX header (A38)); /ublox_driver/time_pulse_info (the A19 time_pulse_info checks have no object) |
| C3 | rates and time ranges (header-stamp period; record-time average) | **WARN** | lidar 10.0 Hz (header), avg 10.00 Hz; imu 200.2 Hz (header), avg 203.28 Hz; compressed 30.0 Hz (header), avg 29.98 Hz; range_meas avg 9.90 Hz (7033 msgs); ephem avg 0.01 Hz (4 msgs); glo_ephem avg 0.01 Hz (6 msgs); receiver_pvt avg 10.00 Hz (7102 msgs); gaps (>max(3x period, 0.25 s)) as (t since start or first header [s], length [s]): {'range_meas': [(379.9, 1.6), (394.4, 1.0), (403.1, 1.8)]} |
| C4 | header stamp vs record time per topic (median/p1/p99 ms, drift ppm) | **INFO** | /camera/color/image_raw/compressed 44.3/20.7/57.9 ms, 1 ppm; /livox/avia/imu -117.0/-188.5/-37.7 ms, 107 ppm; /livox/avia/lidar -76.0/-96.0/-27.6 ms, 109 ppm; /mavros/global_position/raw/fix 9.3/-1.1/110.2 ms, -1 ppm |
| G1 | epoch interval 10 Hz, monotone, one time tag per epoch (§12.6-1) | **PASS** | 7033 epochs; 99.93 % at 0.1 s; 0 non-increasing; 0 epochs with >1 time tag; 5 gaps >0.15 s |
| G15 | 1 Hz decimation rule (AMENDMENTS A43: the first epoch of every GPST second; the retired DESIGN §4.3 5 ms rule reported beside it) | **PASS** | A43 rule: 708 epochs for 708 GPST seconds with data; 10 Hz epoch grid offset +1.0 ms from the GPST tenths; the retired \|tow - round(tow)\| < 5 ms rule would select an epoch in 99.2% of the 709 seconds |
| G2 | per-satellite array lengths consistent; sat numbers inside the gnss_comm ranges | **PASS** | 0 inconsistent satellites; unknown sat numbers: none |
| G3 | (sys, freq, code) histogram and freqs->code table verdicts (signal_table.hpp) | **PASS** | G 1575.4200 MHz code=1->L1C: 19813; R 1242.9375 MHz code=14->L2C: 46; R 1598.0625 MHz code=1->L1C: 321; R 1601.4375 MHz code=1->L1C: 26; R 1603.1250 MHz code=1->L1C: 3707; R 1603.6875 MHz code=1->L1C: 37; R 1604.2500 MHz code=1->L1C: 121; verdicts {'Accepted': 24071}; primary slot first: {'slot0': 24025, 'none': 46} |
| G4 | satellites per epoch (primary signal present) | **INFO** | total min/mean/max 1/3.4/8; GPS 0/2.8/7, GLO 0/0.6/2 |
| G5 | C/N0 distribution (dB-Hz, primary signal) | **INFO** | GLO median 18 p5 15 p95 28; GPS median 20 p5 14 p95 27 |
| G6 | psr / cp / dopp availability (status bits + value; with the L0 C/N0 masks) | **INFO** | GLO psr 100% cp 38% dopp 100% (masked 0/6/45%); GPS psr 100% cp 9% dopp 100% (masked 0/2/16%) |
| G7 | LLI / status statistics | **INFO** | LLI {'GPS': {'3': 17742, '0': 1369, '2': 695, '1': 7}, 'GLO': {'0': 1517, '1': 1, '3': 2564, '2': 130}}; status {'GPS': {'1': 17756, '7': 1129, '3': 681, '15': 247}, 'GLO': {'7': 1518, '1': 2567, '3': 127}} |
| G8 | std unit judgement on the u-blox grids (§12.6-3) | **PASS** | psr metres (100.0% on 0.01*2^n m); cp cycles (100.0% on 0.004n cyc, 0.0% on 0.004n*lambda m, 0.0% on 0.004n/lambda); dopp hertz (100.0% on 0.002*2^n Hz, 0.0% as m/s); cp_std=0.06 (cpStdev=15, u-blox 'invalid') on 84.5% of slots |
| G9 | carrier-phase rate vs Doppler sign/scale (§12.6-2: corr > 0.99, slope ~ 1) | **PASS** | 3346 pairs (clean (LLI bit0 = 0)): corr 1.00000, slope 1.0000, median \|diff\| 0.048 m/s; per satellite (§12.6-2 '每星', decisive = >= 50 clean pairs and range-rate std >= 2 m/s): 4/4 decisive satellites with corr > 0.99 and \|slope - 1\| < 0.02 (slopes 1.0000..1.0023), 3 not decisive; 1 s sign agreement lambda*dPhi vs dP 100.0%, vs -lambda*D*dt 100.0% (n=279) |
| G10 | GLONASS channel k from freqs vs ephemeris frq (§12.3, §12.6-1) | **PASS** | 4258 slots; mismatching satellites none; observed without any ephemeris: none |
| G14 | self-check 1: RTKLIB sat2freq(sat, code, nav) of every accepted slot = its freqs (A35-C8) | **PASS** | 24071 slots consistent, 0 mismatching, 0 unverifiable (GLONASS without an ephemeris channel: none), 0 slots not accepted by the table (G3); per system {'GPS': {'consistent': 19813}, 'GLO': {'consistent': 4258}} |
| E1 | bag Kepler ephemerides vs same-day RINEX (same sat,toe): \|dr\| < 1e-3 m, \|ddts\| < 1e-11 s (A9-3 tier 2) | **PASS** | 4/4 matched, 0 outside tier 2 (0 not explained by a corrupted copy, see E7); worst per system {'G': '7.92e-06 m / 4.37e-17 s'}; toe label shifts {'G:+0': 4}; field semantics {'G': {'n': 4, 'toe_tow_is_system_tow': 4, 'week_is_system_week': 4, 'week_is_gps_week': 4, 'toe_msg_is_gpst': 4, 'tgd0_eq': 4, 'tgd1_eq': 4, 'tgd0_eq_rnx_tgd1': 0, 'iode_eq': 4, 'svh_eq': 4, 'code_eq': 0}} |
| E7 | bag ephemeris stream consistency: all copies of one (sat, toe, IODE) identical | **PASS** | 3 keys, 0 with conflicting copies |
| E2 | bag GLONASS ephemerides vs RINEX (pos/vel/acc units, tau_n sign, channel) | **PASS** | 6/6 matched; worst \|dr\| at toe+600 s 3.70e-06 m; semantics {'n': 6, 'taun_same_sign_as_rtklib': 6, 'taun_opposite_sign': 0, 'gamn_eq': 6, 'frq_eq': 6, 'pos_eq_1mm': 6} |
| E4 | URA semantics: bag ura vs same-day RINEX SV accuracy (§12.6-4, A16) | **PASS** | decision per system {'G': 'metres'} -> gnss.ephem.ura_semantics: metres; counts {'G': {'n': 4, 'metres_exact': 4, 'metres_approx': 4, 'index': 0, 'icd_formula_metres': 0}}; samples (bag, rinex m) {'G': [(2.0, 2.0), (2.0, 2.0), (2.0, 2.0), (2.0, 2.0)]} |
| E3 | orbit code self-check: adjacent RINEX records agree at the midpoint (p95 < 10 m per system) | **PASS** | C median 0.11 m p95 0.42 m (n=254); E median 0.09 m p95 0.46 m (n=646); G median 0.23 m p95 1.14 m (n=92) |
| E5 | ephemeris coverage timeline: bag-only (causal by record time) vs bag+RINEX (A1, §12.6-8) | **WARN** | sat-epochs with a healthy ephemeris: bag-only 39.7%, bag+RINEX 100.0%; epochs with >=5: 0.0% / 34.2%; first >=5 at — / 0.0 s; BDS GEO observed none; never covered bag-only: G03, G08, G13, G14, G19, G21, G30, R12, R14, R17, R23 |
| E6 | Klobuchar source (A38): GPS set = RINEX header GPSA/GPSB; bag iono_params only a diagnostic (family classified) | **PASS** | GPS Klobuchar from the RINEX header GPSA/GPSB: ['3.353e-08', '1.49e-08', '-1.192e-07', '1.192e-07', '1.516e+05', '-1.475e+05', '0', '0']; no /ublox_driver/iono_params in the bag |
| L1 | GNSS arrival latency: range_meas record time - GPST->UTC (leap 18 s) | **INFO** | median -70.3 ms, MAD 16.1, p1 -89.0, p99 -22.2, min -89.7, max -21.6; drift 105.0 ppm; persistent steps >50 ms: none; receiver_pvt median -58.0 ms |
| L3 | self-check 6 (time, record-time part): \|record - (GPST - leap)\| < 0.3 s (leap seconds and time system consistent) | **PASS** | median -0.0703 s with leap 18 s (a missing/extra leap second would show as ~±1 s, a GPST/UTC mix-up as ±18 s); the cross-correlation part of self-check 6 is method 2 of tools/time_calib/gnss_time_calib.py on the ref-R replay |
| L2 | receiver_lla header stamp convention | **INFO** | header.stamp - GPST(seconds, Unix origin) median 0.000 s (min 0.000, max 0.000): receiver_lla stamps are GPST without leap seconds (not ROS/UTC time) |
| T1 | time_pulse_info checks (A19: leap second, time system, gross errors, host clock steps) | **SKIP** | topic /ublox_driver/time_pulse_info absent from this bag (README §4.2 lists it); nothing to check. Host-clock steps are covered by the range_meas latency series instead (L1). |
| G11 | Doppler sign by LS fit: -lambda*D = range rate (§1) vs flipped sign (all Dopplers, el >= 5°) | **PASS** | post-fit RMS median 0.162 m/s with the design sign vs 188.0 m/s flipped (246 epochs at 1.0 Hz; linearised at the receiver PVT (evaluation sequence, A2)) |
| G12a | self-check 5 (position): SPP vs receiver PVT | **SKIP** | A2: not a development sequence — self-check 5 compares GNSS-only solutions with the receiver PVT (a calibration-grade judgement, TEST_PLAN §3: development set only); not computed |
| G12b | self-check 5 (velocity): Doppler LS vs receiver PVT | **SKIP** | A2: not a development sequence — self-check 5 compares GNSS-only solutions with the receiver PVT (a calibration-grade judgement, TEST_PLAN §3: development set only); not computed |
| G13 | lever-arm direction (§12.6-7): Doppler innovations with l vs -l on turns; regression on the omega x l term (~0; ~-2 = sign flipped) | **SKIP** | A2: not a development sequence — the lever-arm judgement uses the replay trajectory and belongs to the development set; not computed |
| S1 | Livox CustomMsg type/md5 vs the repository's livox_ros_driver headers | **PASS** | /livox/avia/lidar: livox_ros_driver/CustomMsg [e4d6829bdfe657cb6c21a746c86b21a6] vs repo livox_ros_driver/CustomMsg [e4d6829bdfe657cb6c21a746c86b21a6]; timebase on another clock (offset 1.73254e+09 s, drift 0.0 ppm vs header); points/scan median 24000 |
| S2 | CompressedImage format; replay direct decode vs republish path, bgr8 and grey pixel-exact (I2) | **PASS** | formats {'rgb8; jpeg compressed bgr8': 21292}; 150 sampled: bgr equal 150, grey equal 150, IMREAD_UNCHANGED==IMREAD_COLOR 150; shapes {'(480, 640, 3)': 150} (Python emulation of both decode paths with OpenCV; the C++ replay path itself is to be measured by WP-f) |
| S3 | GT file format (TUM: t x y z qx qy qz qw) | **WARN** | 10655 rows, columns {8: 10655}; 5198 non-increasing stamps (291 backwards, max 9.8 us, 4907 exact duplicates); quaternion identity on 100.0% rows (orientation not provided); repeated positions 66.8%; GT stamps = /mavros/global_position/raw/fix header stamps on 100.0% of rows |

## 话题清单

| topic | type | md5 | md5 recomputed | msgs | rate [Hz] (header, else avg) | record-header median [ms] |
|---|---|---|---|---|---|---|
| /camera/imu | sensor_msgs/Imu | 6a62c6daae103f4ff57a132d6f95cec2 | yes | 142060 | 200.23 | -1.0 |
| /livox/mid360/imu | sensor_msgs/Imu | 6a62c6daae103f4ff57a132d6f95cec2 | yes | 142045 | 202.10 | 6.8e-02 |
| /camera/aligned_depth_to_color/image_raw/compressedDepth | sensor_msgs/CompressedImage | 8f7a12909da2c9d3332d540a0977563f | yes | 21292 | 30.01 | 42.5 |
| /camera/color/image_raw/compressed | sensor_msgs/CompressedImage | 8f7a12909da2c9d3332d540a0977563f | yes | 21292 | 30.01 | 44.3 |
| /livox/avia/imu | sensor_msgs/Imu | 6a62c6daae103f4ff57a132d6f95cec2 | yes | 144385 | 200.16 | -117.0 |
| /livox/avia/lidar | livox_ros_driver/CustomMsg | e4d6829bdfe657cb6c21a746c86b21a6 | yes | 7103 | 10.02 | -76.0 |
| /mavros/global_position/raw/fix | sensor_msgs/NavSatFix | 2d3a8cd499b9b4a0249fb98fd05cfa48 | yes | 10655 | 104857.60 | 9.3 |
| /odom | nav_msgs/Odometry | cd5e73d190d741a2f92e81eda573aca7 | yes | 14205 | 20.00 | 7.2e-02 |
| /livox/mid360/lidar | livox_ros_driver2/CustomMsg | e4d6829bdfe657cb6c21a746c86b21a6 | yes | 7103 | 10.00 | 100.7 |
| /ublox_driver/range_meas | gnss_comm/GnssMeasMsg | 0eafeb530a3e6637722c4e296734c119 | yes | 7033 | 9.90 | — |
| /ublox_driver/receiver_pvt | gnss_comm/GnssPVTSolnMsg | d18171357d7a159f76d4d7c0b12fb631 | yes | 7102 | 10.00 | — |
| /ublox_driver/receiver_lla | sensor_msgs/NavSatFix | 2d3a8cd499b9b4a0249fb98fd05cfa48 | yes | 7102 | 10.00 | -18058.0 |
| /ublox_driver/glo_ephem | gnss_comm/GnssGloEphemMsg | 6893dee0d8b1c9325a6693f3488e0144 | yes | 6 | 7.2e-03 | — |
| /ublox_driver/ephem | gnss_comm/GnssEphemMsg | b4346f99e431c870e392a438684c0fe8 | yes | 4 | 7.1e-03 | — |

## 信号直方图 (sys, freq, code 字段 → signal_table)

| sys | freq [MHz] | code field | table code | count |
|---|---|---|---|---|
| G | 1575.4200 | 1 | L1C | 19813 |
| R | 1242.9375 | 14 | L2C | 46 |
| R | 1598.0625 | 1 | L1C | 321 |
| R | 1601.4375 | 1 | L1C | 26 |
| R | 1603.1250 | 1 | L1C | 3707 |
| R | 1603.6875 | 1 | L1C | 37 |
| R | 1604.2500 | 1 | L1C | 121 |

## 星历字段语义(bag vs 同日 RINEX,按 (sat,toe) 配对的计数)

| sys | code_eq | iode_eq | n | svh_eq | tgd0_eq | tgd0_eq_rnx_tgd1 | tgd1_eq | toe_msg_is_gpst | toe_tow_is_system_tow | week_is_gps_week | week_is_system_week |
|---|---|---|---|---|---|---|---|---|---|---|---|
| G | 0 | 4 | 4 | 4 | 4 | 0 | 4 | 4 | 4 | 4 | 4 |

## 产物

`audit.json`(全部数字)、`latency.csv`、`eph_coverage.csv`、`eph_coverage_timeline.csv`、`plot_*.png`,位于远端归档目录。

## GT 审计(tools/gt_audit.py)

生成:`tools/gt_audit.py`(tool_version 3),2026-09-26T12:20:18Z。输入:GT 文件与原始 bag 的 RTK 定位话题、9Ps 接收机 PVT、Avia IMU、轮速计(A2:不读任何估计轨迹)。

总评:**FAIL**(FAIL 1, INFO 3, PASS 2, WARN 3)。远端归档:`/work/fastlivo2_gnss/audit/Dark02/gt`。

### 检查表

| # | check | verdict | evidence |
|---|---|---|---|
| P0 | GT file format and cleaning | **INFO** | 10655 rows; 291 backward stamp steps; 582 rows reordered and 4907 duplicate-stamp rows dropped in gt_clean.tum; orientation identity on 100.0% rows (position-only GT) |
| P1 | GT provenance: stamps and projection reproduced from the RTK fix (C-RTK2HP) | **PASS** | 100.0% of GT stamps equal /mavros/global_position/raw/fix header stamps (1 us); best hypothesis 'ENU at the first GT fix (exact, WGS-84)' reproduces GT to 1.67e-03 m RMS (max 2.85e-03 m); other hypotheses: UTM zone 49 (E, N, alt) minus first 2.28 m; equirectangular (R=6378137 m) minus first 0.172 m; ENU of GCJ-02-shifted lat/lon 0.0625 m |
| P4 | GT time base vs the estimator clock (Avia IMU header) by cross-correlation; > 20 ms compensated | **FAIL** | no valid correlation chain (peaks: {'gt_course_rate__imu_yaw_rate': 0.67, 'gt_speed__odom_speed': 1.0, 'odom_yaw_rate__imu_yaw_rate': 0.78, 'gt_speed__pvt_speed_utc': 0.51}): GT time base undetermined by the pre-registered rule; AMENDMENTS A54: the record-time value is used instead (gt_timebase.yaml); for the record, the chain of the sub-threshold links would give +105.9 ms (delay precision 32.6 ms, peaks 1.00/0.78); record-time plausibility +126.4 ms; antenna-lead fit (no odometry) L +138.6 ms, r_x +0.32 m (median speed 0.90 m/s, n=2668; r_x = 0 gives +367.2 ms); links: gt_course_rate__imu_yaw_rate +359.2 ms (peak 0.671, delay precision 10.7 ms); gt_speed__odom_speed -38.6 ms (peak 0.996, delay precision 32.3 ms); odom_yaw_rate__imu_yaw_rate +144.5 ms (peak 0.778, delay precision 4.8 ms); gt_speed__pvt_speed_utc +995.7 ms (peak 0.507, delay precision 363.0 ms) |
| P2 | datum: RTK fix (GT source) lat/lon vs 9Ps WGS-84 PVT | **PASS** | median horizontal difference 6.5 m vs a GCJ-02 shift of 466 m at this location ⇒ WGS-84 (no GCJ-02 offset) |
| P3 | GT (local XYZ) vs 9Ps PVT in ENU at the GT origin: horizontal SE(2)/Sim(2) structure (ENU, no rotation/scale expected) | **WARN** | SE(2) yaw +2.664° (block-bootstrap σ 0.655°), Sim(2) scale 0.95055 (σ 0.02813); residual trend dE/dE -27.6 dN/dN -83.3 m/km; n=7102 (residual magnitudes are the 9Ps PVT noise, not a result, A2) |
| P5 | pre-registered trust mask (9Ps h_acc > 1 m or num_sv < 8; speed spikes > 3σ; gaps/stalls > 0.5 s; each ±2 s) | **WARN** | trusted 0.0% (GT-derived criteria alone: 87.9%); masked by h_acc>1 m 100.0%, num_sv<8 73.9% (receiver criteria before the ±2 s widening 100.0%), speed spikes 12.1% (27; σ 0.016 m/s, floor 0.03), gaps/stalls 0.0% (0/0); the h_acc criterion masks most rows because the 9Ps is a standalone (non-RTK) receiver: median h_acc 6.44 m — the pre-registered threshold needs a decision |
| P7 | RTK jumps / spikes (A54): fix-to-fix residual against the local velocity > max(0.5 m, 10σ); a jump and its return = displaced plateau, masked ±2 s | **INFO** | 0 jump(s) (thresholds h 0.50 / v 0.50 m; residual σ h 0.010 / v 0.016 m over 3535 fix intervals); masked 0.0 s of 710.3 s (0.00%); ATE-excluded (gaps/stalls + RTK) 0.0 s |
| P4r | GT time base used by the evaluation (AMENDMENTS A54 rule) | **WARN** | method record_time: +126.4 ms (applied shift +126.4 ms, > 20 ms rule), confidence medium, uncertainty 50.0 ms (the agreement P4 requires between the record-time value and a chain (A40)); P4 FAIL: record-time value (A54) |
| P6 | 2HP antenna lever arm | **INFO** | unknown (the C-RTK2HP antenna is not in calibration.md): an ATE floor of the order of the antenna separation on the top plate (~0.1-0.3 m, README Fig. 1); TEST_PLAN E1' estimates it jointly |

### GT 生成方式假设(对 RTK 定位逐点复算)

| hypothesis | direct RMS [m] | direct max [m] | Sim3 scale | Sim3 yaw [°] | Sim3 RMS [m] |
|---|---|---|---|---|---|
| ENU at the first GT fix (exact, WGS-84) | 0.002 | 0.003 | 1.000 | 5.93e-04 | 4.70e-04 |
| UTM zone 49 (E, N, alt) minus first | 2.278 | 3.749 | 1.000 | 1.252 | 4.34e-04 |
| equirectangular (R=6378137 m) minus first | 0.172 | 0.264 | 1.000 | 0.014 | 0.139 |
| ENU of GCJ-02-shifted lat/lon | 0.063 | 0.108 | 1.000 | 0.028 | 0.018 |

GCJ-02 在本地的绝对平移 466 m,轨迹范围内局部畸变 0.058 m。

### 时基互相关(GT 戳 t 对应其它时钟 t + lag)

| reference | lag [ms] | peak corr | peak width [ms] | delay precision [ms] |
|---|---|---|---|---|
| gt_course_rate__imu_yaw_rate | 359.187 | 0.671 | 780.000 | 10.730 |
| gt_speed__odom_speed | -38.604 | 0.996 | 1360.000 | 32.286 |
| odom_yaw_rate__imu_yaw_rate | 144.479 | 0.778 | 780.000 | 4.843 |
| gt_speed__pvt_speed_utc | 995.668 | 0.507 | 1410.000 | 363.046 |

### 天线超前模型(P4 诊断,不经轮速计)

航向角速率 = ω(t+L) + r_x·d/dt(ω/v):L = +138.6 ms,r_x = +0.32 m(车速中位 0.90 m/s,n = 2668);r_x 置 0 时 L = +367.2 ms。天线在无侧滑点前方 r_x 处时 GT 航向**领先**车头 r_x/v,原始直接链路相对链式结果为正即此效应,不是钟差。

### 评测用 GT 时基(AMENDMENTS A54,`gt_timebase.yaml`)

规则:P4 通过 ⇒ 轮速计链;P4 失败 ⇒ 记录时间旁证;两者都没有 ⇒ 0 并标记;补偿按 TEST_PLAN §4(|值| > 20 ms 才补偿)。

| method | value [ms] | applied shift [ms] | confidence | uncertainty [ms] | P4 | chain [ms] | record time [ms] | sub-threshold chain [ms] | P4c replay (dev only, not used) [ms] |
|---|---|---|---|---|---|---|---|---|---|
| record_time | +126.4 | +126.4 | medium | 50.0 | FAIL | — | +126.4 | +105.9 | — |

### RTK 跳变/毛刺与被掩时长(AMENDMENTS A54)

逐定位差分对局部速度(前后各 3 个定位区间的中位速度)的残差;跳变 = 水平 > 0.50 m 或垂直 > 0.50 m(max(0.5 m, 10σ),σ 水平 0.010 / 垂直 0.016 m);一对相互抵消的跳变之间为错位平台,连同前后 2 s 掩掉(bit 64)。

无跳变。

| criterion | masked [s] | of bag span [%] |
|---|---|---|
| h_acc>1m | 710.251 | 100.000 |
| num_sv<8 | 502.045 | 70.686 |
| speed_spike | 86.024 | 12.112 |
| gap_or_stall | 0.000 | 0.000 |
| no_pvt_within_0.2s | 0.000 | 0.000 |
| rtk_jump_plateau | 0.000 | 0.000 |
| ate_exclude | 0.000 | 0.000 |
| pre_registered_all | 710.251 | 100.000 |

bag 区间内 GT 时长 710.3 s;`ate_exclude` = 间断/冻结 + RTK 跳变平台(bit 8|64;bag 区间外 bit 32 本就不参评),区间见 `gt_mask_intervals.csv`(GT 戳与审计后时基两列)。h_acc/num_sv/速度尖峰(bit 1/2/4)只报告,不标 ATE 排除(未裁定)。

### 产物

`gt_clean.tum`、`gt_audited.tum`(= gt_clean + 0.126 s,方法 record_time)、`gt_timebase.yaml`、`trust_mask.csv`、`gt_mask_intervals.csv`、`gt_audit.json`。

## 时间标定(tools/time_calib/gnss_time_calib.py,DESIGN §7)

### 方法①(记录时间统计与估计器时钟先验)

range_meas 记录时间 − GPST→UTC(leap 18 s,主机钟视角 = 主机钟偏差 + GNSS 传输延迟):中位 -70.3 ms、MAD 16.1 ms、p5/p95 -87.7/-23.7 ms、漂移 105.0 ppm、持续台阶(>50 ms)无。

头戳 − 记录时间(判定 LiDAR 时基):/camera/color/image_raw/compressed 中位 -44.3 ms(p1 -57.9 / p99 -20.7,漂移 -1.1 ppm);/livox/avia/imu 中位 117.0 ms(p1 37.7 / p99 188.5,漂移 -107.1 ppm);/livox/avia/lidar 中位 76.0 ms(p1 27.6 / p99 96.0,漂移 -109.4 ppm)。

估计器时钟(Livox 头戳)先验:GNSS「记录 − UTC」下包络 -70.5 ms 减 Avia IMU「记录 − 头戳」下包络 -171.4 ms ⇒ 上界 101.3 ms(主机钟台阶在两者中抵消;漂移 -4.2 ppm;估计器时钟台阶 无);GNSS 传输延迟不可观,按 [0, 150] ms 均匀(推断)⇒ 先验 26.3 ms、σ 75 ms。

方法①文件 `Dark02_record_time.yaml`(不读回放;method record_time,σ 75 ms;tool_version ≥ 3 按 AMENDMENTS A53 TDCP 不暂停,tool_version ≤ 2 的文件仍写着暂停):

```yaml
sequence: Dark02
tool_version: 3
leap_s: 18
segments:
  - t_from_gpst_s: 0
    offset_s: 0.02631737911297624
    drift_ppm: -4.213852087024375
    t_ref_gpst_s: 1732538473.301
    sigma_toff_s: 0.075
    tdcp_paused: false
    method: record_time
```

### 方法②(水平天线速度互相关)与方法③(DTW)

输入:回放轨迹 `/work/fastlivo2_gnss/runs/eval/preproc_Dark02/Dark02/U/s0/traj_W.tum`(tum,21281 个速度样本,估计器速度 std 0.35 m/s);多普勒 LS 速度 2432 个历元。窗口 21 个,可用 0 个(窗长/步长见 JSON)。状态:ran: no usable window (GNSS velocity covers 0% of the window (< 90%); GNSS velocity covers 21% of the window (< 90%); GNSS velocity covers 30% of the window (< 90%); GNSS velocity covers 67% of the window (< 90%); insufficient excitation (speed std < 0.1 m/s) or data; peak 0.04 at the search edge; peak 0.31 at the search edge; peak 0.74 at the search edge) -> method-1 prior (record_time)。

信号(tool_version 3):horizontal antenna speed = |horizontal part of the zero-phase Gaussian-smoothed (0.25 s) velocity vector| on both sides, 100 Hz grid;W 的竖直方向:state_trace gravity (median g = [-0.1221, -0.0264, -9.8087])。多普勒 LS:C/N0 ≥ 15.0 dB-Hz、仰角 ≥ 10.0°,按接收机 dopp_std 加权,两遍(第二遍固定平滑后的钟漂):2642 个历元有 ≥ 4 行,第二遍解出 2432 个(每历元用行中位 6.0)。

| 窗起点 [s] | 用 | 峰值相关 | 偏移 [ms] | 峰宽精度 [ms] | GNSS 覆盖 | 原因 |
|---|---|---|---|---|---|---|
| 0 | 否 | 0.736 | — | 1472.8 | 100% | 峰在搜索边界 |
| 30 | 否 | 0.311 | — | 4845.8 | 100% | 峰在搜索边界 |
| 60 | 否 | — | — | — | 100% | insufficient excitation (speed std < 0.1 m/s) or data |
| 90 | 否 | — | — | — | 100% | insufficient excitation (speed std < 0.1 m/s) or data |
| 120 | 否 | — | — | — | 100% | insufficient excitation (speed std < 0.1 m/s) or data |
| 150 | 否 | 0.042 | — | 912.7 | 96% | 峰在搜索边界 |
| 180 | 否 | — | — | — | 67% | GNSS velocity covers 67% of the window (< 90%) |
| 210 | 否 | — | — | — | 21% | GNSS velocity covers 21% of the window (< 90%) |
| 240 | 否 | — | — | — | 0% | GNSS velocity covers 0% of the window (< 90%) |
| 270 | 否 | — | — | — | 0% | GNSS velocity covers 0% of the window (< 90%) |
| 300 | 否 | — | — | — | 0% | GNSS velocity covers 0% of the window (< 90%) |
| 330 | 否 | — | — | — | 0% | GNSS velocity covers 0% of the window (< 90%) |
| 360 | 否 | — | — | — | 0% | GNSS velocity covers 0% of the window (< 90%) |
| 390 | 否 | — | — | — | 0% | GNSS velocity covers 0% of the window (< 90%) |
| 420 | 否 | — | — | — | 0% | GNSS velocity covers 0% of the window (< 90%) |
| 450 | 否 | — | — | — | 0% | GNSS velocity covers 0% of the window (< 90%) |
| 480 | 否 | — | — | — | 0% | GNSS velocity covers 0% of the window (< 90%) |
| 510 | 否 | — | — | — | 0% | GNSS velocity covers 0% of the window (< 90%) |
| 540 | 否 | — | — | — | 0% | GNSS velocity covers 0% of the window (< 90%) |
| 570 | 否 | — | — | — | 0% | GNSS velocity covers 0% of the window (< 90%) |
| 600 | 否 | — | — | — | 30% | GNSS velocity covers 30% of the window (< 90%) |

标定文件(方法②无可用段或不如方法①准,逐段取方法① method record_time;A53:σ ≤ 0.1 s 即 TDCP 不暂停) `time_calib/Dark02.yaml`(AMENDMENTS A18 格式;所有 `*_gpst_s` 为 GTime::toSec(),1970 原点、GPST 尺度):

```yaml
sequence: Dark02
tool_version: 3
leap_s: 18
segments:
  - t_from_gpst_s: 0
    offset_s: 0.02631737911297624
    drift_ppm: -4.213852087024375
    t_ref_gpst_s: 1732538473.301
    sigma_toff_s: 0.075
    tdcp_paused: false
    method: record_time
```
