> Historical research record (through 2026-10-03). Paths are illustrative; archived runs are not bundled. For current status and executable entry points see [the repository README](../../../README.md). Current release docs/LICENSING.md supersedes historical license notes. This record does not authorize new experiments or assert current release validation.

# I2 数据审计 — Dark01

生成:`tools/m3dgr_audit.py`(tool_version 3),2026-09-26T13:16:40Z;bag `/work/fastlivo2_gnss/data/m3dgr/bags/Dark01.bag`(2378281084 字节,2024-12-23T16:54:58Z;sha256 9f43a8a56df39b338728199646f2e5a6a4e972109fb925abd1a3b42c46ad19db)。序列类别:**evaluation**(AMENDMENTS A2)。

> 评测集:本报告只做格式/解码审计。没有读取任何估计轨迹;不做 GNSS-only 解对接收机 PVT 的比较(§12.6-5 自检 G12a/b)与杆臂方向(§12.6-7 自检 G13),两者在评测集上 SKIP(A2);G11 只比较多普勒两种符号假设的拟合残差(解码性质)。

总评:**WARN**(INFO 9, PASS 20, SKIP 4, WARN 1)。远端归档:`/work/fastlivo2_gnss/audit/Dark01`。

## 本轮说明

评测集(A2),AMENDMENTS A68(2026-09-26,tool_version 3)。原包 = 远端既有 `data/m3dgr/bags/Dark01.bag`(未重新下载;sha256 与官方一致)。

过滤后 `data/m3dgr/filtered/Dark01.bag`(1.11 GiB,sha256 `2f3c2d15edb6ed72…`;保留 Avia LiDAR/IMU、压缩彩色图、`/ublox_driver/*`、RTK 定位与 `/odom`,逐消息核对通过)。BRDC `BRDC00IGS_R_20243300000_01D_MN.rnx`。时长 206.2 s。

**只做格式/解码审计、GT 自身审计与时间标定**:没有读取任何估计轨迹对真值的比较;自检 5(G12a/b)、自检 7(G13)与 GT 审计的回放路线 P4c 在评测集上不计算。方法②只比较 U 臂 GNSS-free 回放(A55,HEAD 零通道,A64 评测构建 `wt/a64-strict`,种子 0,`runs/eval/preproc_Dark01/Dark01/U/s0`)的天线水平速度与多普勒 LS 水平速度。GT 审计在过滤后的包上重跑,产物与在原包上的逐字节相同。

**时间标定**:method record_time +30.9 ms、σ_toff 75.0 ms、漂移 -1.50 ppm(方法②无可用窗,取方法①先验);TDCP 不暂停(A53)。

**GT 时基(A54)**:P4 WARN ⇒ method chain +107.1 ms,补偿 +107.1 ms,置信 high;`ate_exclude` 0.0 s,RTK 跳变平台 0 个。

**I2 档案**:原 P2b 的 tool_version 2 报告移到 `audit/Dark01/.previous/<UTC>/i2/`,在原包上用 tool_version 3 重跑进档案(`runs/a68/i2_archive_v3.sh`);各检查的判定与 v2 逐项相同,也与取数时写在 `data/m3dgr/filtered/Dark01.fetch/audit/` 的 v3 报告逐项相同。时间标定与 GT 审计由 `preprocess_seq.py` 重做(旧的 tool_version 1/2 文件在 `.previous/<UTC>/`),取代了 P2b 档案里写着 TDCP 暂停的方法①临时先验。

## 供 WP-a / 核心使用的结论

- `gnss.ephem.ura_semantics: metres`(A16;判定依据见 E4)。
- std 单位(§12.6-3):psr_std=metres,cp_std=cycles,dopp_std=hertz;cp_std=0.06 是 u-blox cpStdev=15 的『无效』标记,占 92.6%。
- 星历覆盖(A1):卫星-历元覆盖率 仅 bag 51.7% / bag+RINEX 100.0%;≥5 星历元比例 0.0% / 96.6% ⇒ 必须配置 `gnss.nav_rinex`。
- Klobuchar(A38):GPS 参数取 RINEX 头 GPSA/GPSB(本日可用);bag 无 iono_params。
- C/N0 中位 GLO 22, GPS 19 dB-Hz;按 DESIGN §10 L0 掩码(伪距 30 / 多普勒 25 / 载波 28 dB-Hz)可用率 伪距 GLO 0%, GPS 0%、多普勒 GLO 25%, GPS 23% ⇒ 默认掩码会饿死滤波与初始化(COLLECT 需 ≥5 星 SPP);掩码必须在开发集按 TEST_PLAN §3 重新标定,不得在本序列上调。
- time_pulse_info 话题不存在 ⇒ A19 的检查无对象;主机钟台阶只能从 range_meas 记录时间序列看(L1)。
- range_meas 记录时间 − GPST→UTC:中位 -38.7 ms、MAD 2.0 ms、漂移 -43.3 ppm、台阶 无(= 主机钟偏差+到达延迟;作 §7 方法①先验,不是最终时偏)。
- 传感器时基(C4,记录时间 − 头戳中位,漂移):Avia LiDAR -50.1 ms / -44 ppm、Avia IMU -100.6 ms / -45 ppm、相机 43.5 ms。Livox 头戳 = 设备时间戳 + 常数(S1:timebase on another clock (offset 1.73254e+09 s, drift 0.0 ppm vs header)),与主机时钟的关系逐序列不同(本序列 LiDAR 头戳领先主机收包时刻,不可能是主机收包戳) ⇒ 估计器时基(LiDAR 头戳)对 GPST 的偏差与漂移只能由 §7 方法②逐序列标定;range_meas 记录时间只作先验。
- 载波/多普勒符号(§12.6-2):斜率 1.0000、相关 1.00000 ⇒ `λ·dΦ/dt = −λD` 成立,与 DESIGN §1 一致。
- Livox:/livox/avia/lidar livox_ros_driver/CustomMsg [e4d6829b], /livox/mid360/lidar livox_ros_driver2/CustomMsg [e4d6829b] 与仓库 include/livox_ros_driver 头 md5 一致。
- CompressedImage format:{'rgb8; jpeg compressed bgr8': 6181};回放直读应 `IMREAD_COLOR` 并按 format 决定通道序(S2)。S2 的 PASS 是 Python 对两条解码路径的仿真,C++ 回放路径本身待 WP-f 实测。
- §12.6-7 杆臂方向:评测集不做(A2,标定级判断只在开发集);calibration.md 方向推导见 docs/dev/audit/m3dgr_calib.md。
- 自检 1 的 sat2freq 交叉核对(A35-C8,G14):全部一致。

## 检查表

| # | check | verdict | evidence |
|---|---|---|---|
| C1 | msg_def md5 recomputed = connection md5 (and = installed ROS type md5 where installed) | **PASS** | 14 connections, 0 mismatches; 8 standard types also matched the installed ROS packages |
| C2 | topics of the replay/GNSS path present (DESIGN §14; since A38 the RINEX NAV file is an input of every sequence) | **PASS** | missing: /ublox_driver/iono_params (no effect: the GPS Klobuchar set comes from the RINEX header (A38)); /ublox_driver/time_pulse_info (the A19 time_pulse_info checks have no object) |
| C3 | rates and time ranges (header-stamp period; record-time average) | **INFO** | lidar 10.0 Hz (header), avg 10.00 Hz; imu 200.2 Hz (header), avg 203.38 Hz; compressed 30.0 Hz (header), avg 29.98 Hz; range_meas avg 10.00 Hz (2061 msgs); ephem avg 0.02 Hz (3 msgs); glo_ephem avg 0.01 Hz (2 msgs); receiver_pvt avg 10.00 Hz (2062 msgs); no gaps |
| C4 | header stamp vs record time per topic (median/p1/p99 ms, drift ppm) | **INFO** | /camera/color/image_raw/compressed 43.5/25.2/55.5 ms, -2 ppm; /livox/avia/imu -100.6/-151.1/-50.7 ms, -45 ppm; /livox/avia/lidar -50.1/-56.1/-47.7 ms, -44 ppm; /mavros/global_position/raw/fix 9.2/-8.2/110.6 ms, 30 ppm |
| G1 | epoch interval 10 Hz, monotone, one time tag per epoch (§12.6-1) | **PASS** | 2061 epochs; 100.00 % at 0.1 s; 0 non-increasing; 0 epochs with >1 time tag; 0 gaps >0.15 s |
| G15 | 1 Hz decimation rule (AMENDMENTS A43: the first epoch of every GPST second; the retired DESIGN §4.3 5 ms rule reported beside it) | **PASS** | A43 rule: 207 epochs for 207 GPST seconds with data; 10 Hz epoch grid offset +1.0 ms from the GPST tenths; the retired \|tow - round(tow)\| < 5 ms rule would select an epoch in 99.5% of the 207 seconds |
| G2 | per-satellite array lengths consistent; sat numbers inside the gnss_comm ranges | **PASS** | 0 inconsistent satellites; unknown sat numbers: none |
| G3 | (sys, freq, code) histogram and freqs->code table verdicts (signal_table.hpp) | **PASS** | G 1575.4200 MHz code=1->L1C: 9576; R 1245.5625 MHz code=14->L2C: 68; R 1598.0625 MHz code=1->L1C: 22; R 1601.4375 MHz code=1->L1C: 29; R 1603.1250 MHz code=1->L1C: 1832; R 1603.6875 MHz code=1->L1C: 381; verdicts {'Accepted': 11908}; primary slot first: {'slot0': 11840, 'none': 68} |
| G4 | satellites per epoch (primary signal present) | **INFO** | total min/mean/max 4/5.7/8; GPS 3/4.6/6, GLO 0/1.1/2 |
| G5 | C/N0 distribution (dB-Hz, primary signal) | **INFO** | GLO median 22 p5 14 p95 26; GPS median 19 p5 15 p95 26 |
| G6 | psr / cp / dopp availability (status bits + value; with the L0 C/N0 masks) | **INFO** | GLO psr 100% cp 10% dopp 100% (masked 0/1/25%); GPS psr 100% cp 5% dopp 100% (masked 0/0/23%) |
| G7 | LLI / status statistics | **INFO** | LLI {'GPS': {'3': 9033, '0': 425, '2': 116, '1': 2}, 'GLO': {'3': 1922, '2': 22, '0': 320}}; status {'GPS': {'1': 9034, '7': 223, '3': 115, '15': 204}, 'GLO': {'1': 1922, '3': 22, '7': 320}} |
| G8 | std unit judgement on the u-blox grids (§12.6-3) | **PASS** | psr metres (100.0% on 0.01*2^n m); cp cycles (100.0% on 0.004n cyc, 0.0% on 0.004n*lambda m, 0.0% on 0.004n/lambda); dopp hertz (100.0% on 0.002*2^n Hz, 0.0% as m/s); cp_std=0.06 (cpStdev=15, u-blox 'invalid') on 92.6% of slots |
| G9 | carrier-phase rate vs Doppler sign/scale (§12.6-2: corr > 0.99, slope ~ 1) | **PASS** | 636 pairs (clean (LLI bit0 = 0)): corr 1.00000, slope 1.0000, median \|diff\| 0.053 m/s; per satellite (§12.6-2 '每星', decisive = >= 50 clean pairs and range-rate std >= 2 m/s): 1/1 decisive satellites with corr > 0.99 and \|slope - 1\| < 0.02 (slopes 1.0000..1.0000), 2 not decisive; 1 s sign agreement lambda*dPhi vs dP 100.0%, vs -lambda*D*dt 100.0% (n=50) |
| G10 | GLONASS channel k from freqs vs ephemeris frq (§12.3, §12.6-1) | **PASS** | 2332 slots; mismatching satellites none; observed without any ephemeris: none |
| G14 | self-check 1: RTKLIB sat2freq(sat, code, nav) of every accepted slot = its freqs (A35-C8) | **PASS** | 11908 slots consistent, 0 mismatching, 0 unverifiable (GLONASS without an ephemeris channel: none), 0 slots not accepted by the table (G3); per system {'GPS': {'consistent': 9576}, 'GLO': {'consistent': 2332}} |
| E1 | bag Kepler ephemerides vs same-day RINEX (same sat,toe): \|dr\| < 1e-3 m, \|ddts\| < 1e-11 s (A9-3 tier 2) | **PASS** | 3/3 matched, 0 outside tier 2 (0 not explained by a corrupted copy, see E7); worst per system {'G': '1.25e-05 m / 4.37e-17 s'}; toe label shifts {'G:+0': 3}; field semantics {'G': {'n': 3, 'toe_tow_is_system_tow': 3, 'week_is_system_week': 3, 'week_is_gps_week': 3, 'toe_msg_is_gpst': 3, 'tgd0_eq': 3, 'tgd1_eq': 3, 'tgd0_eq_rnx_tgd1': 0, 'iode_eq': 3, 'svh_eq': 3, 'code_eq': 0}} |
| E7 | bag ephemeris stream consistency: all copies of one (sat, toe, IODE) identical | **PASS** | 3 keys, 0 with conflicting copies |
| E2 | bag GLONASS ephemerides vs RINEX (pos/vel/acc units, tau_n sign, channel) | **PASS** | 2/2 matched; worst \|dr\| at toe+600 s 3.48e-06 m; semantics {'n': 2, 'taun_same_sign_as_rtklib': 2, 'taun_opposite_sign': 0, 'gamn_eq': 2, 'frq_eq': 2, 'pos_eq_1mm': 2} |
| E4 | URA semantics: bag ura vs same-day RINEX SV accuracy (§12.6-4, A16) | **PASS** | decision per system {'G': 'metres'} -> gnss.ephem.ura_semantics: metres; counts {'G': {'n': 3, 'metres_exact': 3, 'metres_approx': 3, 'index': 0, 'icd_formula_metres': 0}}; samples (bag, rinex m) {'G': [(2.0, 2.0), (2.0, 2.0), (2.0, 2.0)]} |
| E3 | orbit code self-check: adjacent RINEX records agree at the midpoint (p95 < 10 m per system) | **PASS** | C median 0.11 m p95 0.42 m (n=254); E median 0.09 m p95 0.39 m (n=627); G median 0.23 m p95 1.14 m (n=92) |
| E5 | ephemeris coverage timeline: bag-only (causal by record time) vs bag+RINEX (A1, §12.6-8) | **INFO** | sat-epochs with a healthy ephemeris: bag-only 51.7%, bag+RINEX 100.0%; epochs with >=5: 0.0% / 96.6%; first >=5 at — / 0.0 s; BDS GEO observed none; never covered bag-only: G03, G07, G08, G13, G14, G21, R12, R14, R23 |
| E6 | Klobuchar source (A38): GPS set = RINEX header GPSA/GPSB; bag iono_params only a diagnostic (family classified) | **PASS** | GPS Klobuchar from the RINEX header GPSA/GPSB: ['3.353e-08', '1.49e-08', '-1.192e-07', '1.192e-07', '1.516e+05', '-1.475e+05', '0', '0']; no /ublox_driver/iono_params in the bag |
| L1 | GNSS arrival latency: range_meas record time - GPST->UTC (leap 18 s) | **INFO** | median -38.7 ms, MAD 2.0, p1 -44.7, p99 -36.0, min -45.4, max -35.8; drift -43.3 ppm; persistent steps >50 ms: none; receiver_pvt median -27.6 ms |
| L3 | self-check 6 (time, record-time part): \|record - (GPST - leap)\| < 0.3 s (leap seconds and time system consistent) | **PASS** | median -0.0387 s with leap 18 s (a missing/extra leap second would show as ~±1 s, a GPST/UTC mix-up as ±18 s); the cross-correlation part of self-check 6 is method 2 of tools/time_calib/gnss_time_calib.py on the ref-R replay |
| L2 | receiver_lla header stamp convention | **INFO** | header.stamp - GPST(seconds, Unix origin) median 0.000 s (min 0.000, max 0.000): receiver_lla stamps are GPST without leap seconds (not ROS/UTC time) |
| T1 | time_pulse_info checks (A19: leap second, time system, gross errors, host clock steps) | **SKIP** | topic /ublox_driver/time_pulse_info absent from this bag (README §4.2 lists it); nothing to check. Host-clock steps are covered by the range_meas latency series instead (L1). |
| G11 | Doppler sign by LS fit: -lambda*D = range rate (§1) vs flipped sign (all Dopplers, el >= 5°) | **PASS** | post-fit RMS median 0.130 m/s with the design sign vs 293.0 m/s flipped (200 epochs at 1.0 Hz; linearised at the receiver PVT (evaluation sequence, A2)) |
| G12a | self-check 5 (position): SPP vs receiver PVT | **SKIP** | A2: not a development sequence — self-check 5 compares GNSS-only solutions with the receiver PVT (a calibration-grade judgement, TEST_PLAN §3: development set only); not computed |
| G12b | self-check 5 (velocity): Doppler LS vs receiver PVT | **SKIP** | A2: not a development sequence — self-check 5 compares GNSS-only solutions with the receiver PVT (a calibration-grade judgement, TEST_PLAN §3: development set only); not computed |
| G13 | lever-arm direction (§12.6-7): Doppler innovations with l vs -l on turns; regression on the omega x l term (~0; ~-2 = sign flipped) | **SKIP** | A2: not a development sequence — the lever-arm judgement uses the replay trajectory and belongs to the development set; not computed |
| S1 | Livox CustomMsg type/md5 vs the repository's livox_ros_driver headers | **PASS** | /livox/avia/lidar: livox_ros_driver/CustomMsg [e4d6829bdfe657cb6c21a746c86b21a6] vs repo livox_ros_driver/CustomMsg [e4d6829bdfe657cb6c21a746c86b21a6]; timebase on another clock (offset 1.73254e+09 s, drift 0.0 ppm vs header); points/scan median 24000 |
| S2 | CompressedImage format; replay direct decode vs republish path, bgr8 and grey pixel-exact (I2) | **PASS** | formats {'rgb8; jpeg compressed bgr8': 6181}; 150 sampled: bgr equal 150, grey equal 150, IMREAD_UNCHANGED==IMREAD_COLOR 150; shapes {'(480, 640, 3)': 150} (Python emulation of both decode paths with OpenCV; the C++ replay path itself is to be measured by WP-f) |
| S3 | GT file format (TUM: t x y z qx qy qz qw) | **WARN** | 3094 rows, columns {8: 3094}; 1857 non-increasing stamps (364 backwards, max 20.0 us, 1493 exact duplicates); quaternion identity on 100.0% rows (orientation not provided); repeated positions 66.8%; GT stamps = /mavros/global_position/raw/fix header stamps on 100.0% of rows |

## 话题清单

| topic | type | md5 | md5 recomputed | msgs | rate [Hz] (header, else avg) | record-header median [ms] |
|---|---|---|---|---|---|---|
| /camera/imu | sensor_msgs/Imu | 6a62c6daae103f4ff57a132d6f95cec2 | yes | 41246 | 200.27 | -1.2 |
| /livox/mid360/imu | sensor_msgs/Imu | 6a62c6daae103f4ff57a132d6f95cec2 | yes | 41238 | 201.65 | 6.7e-02 |
| /ublox_driver/receiver_pvt | gnss_comm/GnssPVTSolnMsg | d18171357d7a159f76d4d7c0b12fb631 | yes | 2062 | 10.00 | — |
| /ublox_driver/receiver_lla | sensor_msgs/NavSatFix | 2d3a8cd499b9b4a0249fb98fd05cfa48 | yes | 2062 | 10.00 | -18027.6 |
| /livox/avia/imu | sensor_msgs/Imu | 6a62c6daae103f4ff57a132d6f95cec2 | yes | 41916 | 200.18 | -100.6 |
| /livox/avia/lidar | livox_ros_driver/CustomMsg | e4d6829bdfe657cb6c21a746c86b21a6 | yes | 2062 | 10.02 | -50.1 |
| /odom | nav_msgs/Odometry | cd5e73d190d741a2f92e81eda573aca7 | yes | 4124 | 20.00 | 7.5e-02 |
| /camera/aligned_depth_to_color/image_raw/compressedDepth | sensor_msgs/CompressedImage | 8f7a12909da2c9d3332d540a0977563f | yes | 6181 | 30.01 | 41.8 |
| /camera/color/image_raw/compressed | sensor_msgs/CompressedImage | 8f7a12909da2c9d3332d540a0977563f | yes | 6181 | 30.01 | 43.5 |
| /mavros/global_position/raw/fix | sensor_msgs/NavSatFix | 2d3a8cd499b9b4a0249fb98fd05cfa48 | yes | 3094 | 15.01 | 9.2 |
| /livox/mid360/lidar | livox_ros_driver2/CustomMsg | e4d6829bdfe657cb6c21a746c86b21a6 | yes | 2062 | 10.00 | 100.7 |
| /ublox_driver/range_meas | gnss_comm/GnssMeasMsg | 0eafeb530a3e6637722c4e296734c119 | yes | 2061 | 10.00 | — |
| /ublox_driver/ephem | gnss_comm/GnssEphemMsg | b4346f99e431c870e392a438684c0fe8 | yes | 3 | 0.02 | — |
| /ublox_driver/glo_ephem | gnss_comm/GnssGloEphemMsg | 6893dee0d8b1c9325a6693f3488e0144 | yes | 2 | 5.6e-03 | — |

## 信号直方图 (sys, freq, code 字段 → signal_table)

| sys | freq [MHz] | code field | table code | count |
|---|---|---|---|---|
| G | 1575.4200 | 1 | L1C | 9576 |
| R | 1245.5625 | 14 | L2C | 68 |
| R | 1598.0625 | 1 | L1C | 22 |
| R | 1601.4375 | 1 | L1C | 29 |
| R | 1603.1250 | 1 | L1C | 1832 |
| R | 1603.6875 | 1 | L1C | 381 |

## 星历字段语义(bag vs 同日 RINEX,按 (sat,toe) 配对的计数)

| sys | code_eq | iode_eq | n | svh_eq | tgd0_eq | tgd0_eq_rnx_tgd1 | tgd1_eq | toe_msg_is_gpst | toe_tow_is_system_tow | week_is_gps_week | week_is_system_week |
|---|---|---|---|---|---|---|---|---|---|---|---|
| G | 0 | 3 | 3 | 3 | 3 | 0 | 3 | 3 | 3 | 3 | 3 |

## 产物

`audit.json`(全部数字)、`latency.csv`、`eph_coverage.csv`、`eph_coverage_timeline.csv`、`plot_*.png`,位于远端归档目录。

## GT 审计(tools/gt_audit.py)

生成:`tools/gt_audit.py`(tool_version 3),2026-09-26T13:01:34Z。输入:GT 文件与原始 bag 的 RTK 定位话题、9Ps 接收机 PVT、Avia IMU、轮速计(A2:不读任何估计轨迹)。

总评:**WARN**(INFO 4, PASS 2, SKIP 1, WARN 2)。远端归档:`/work/fastlivo2_gnss/audit/Dark01/gt`。

### 检查表

| # | check | verdict | evidence |
|---|---|---|---|
| P0 | GT file format and cleaning | **INFO** | 3094 rows; 364 backward stamp steps; 887 rows reordered and 1493 duplicate-stamp rows dropped in gt_clean.tum; orientation identity on 100.0% rows (position-only GT) |
| P1 | GT provenance: stamps and projection reproduced from the RTK fix (C-RTK2HP) | **PASS** | 100.0% of GT stamps equal /mavros/global_position/raw/fix header stamps (1 us); best hypothesis 'ENU at the first GT fix (exact, WGS-84)' reproduces GT to 7.19e-05 m RMS (max 1.08e-04 m); other hypotheses: UTM zone 49 (E, N, alt) minus first 0.476 m; equirectangular (R=6378137 m) minus first 0.0644 m; ENU of GCJ-02-shifted lat/lon 0.00656 m |
| P4 | GT time base vs the estimator clock (Avia IMU header) by cross-correlation; > 20 ms compensated | **WARN** | GT stamp -> Avia IMU header clock: +107.1 ms by chain GT->odom (speed) + odom->IMU (yaw rate); > 20 ms: gt_audited.tum shifted by it; record-time plausibility +109.7 ms (assumes equal transport delays; agreement within 50 ms required); antenna-lead fit (no odometry) L +139.7 ms, r_x +0.31 m (median speed 0.65 m/s, n=823; r_x = 0 gives +425.0 ms); raw direct course-rate link +415.6 ms (peak 0.75): +308.5 ms against the chain = antenna lead r_x/v, not a clock term; links: gt_course_rate__imu_yaw_rate +415.6 ms (peak 0.754, delay precision 32.7 ms); gt_speed__odom_speed -26.4 ms (peak 0.995, delay precision 85.0 ms); odom_yaw_rate__imu_yaw_rate +133.5 ms (peak 0.893, delay precision 7.4 ms); gt_speed__pvt_speed_utc +685.2 ms (peak 0.468, delay precision 834.5 ms) |
| P2 | datum: RTK fix (GT source) lat/lon vs 9Ps WGS-84 PVT | **PASS** | median horizontal difference 5.0 m vs a GCJ-02 shift of 466 m at this location ⇒ WGS-84 (no GCJ-02 offset) |
| P3 | GT (local XYZ) vs 9Ps PVT in ENU at the GT origin: horizontal SE(2)/Sim(2) structure (ENU, no rotation/scale expected) | **SKIP** | not decisive: 9Ps PVT noise vs trajectory extent (horizontal spread 16 m, 7 x 30 s blocks) gives yaw σ 3.06° > 1°; the frame is decided by P1; residual trend dE/dE -132.1 dN/dN +96.2 m/km; n=2061 (residual magnitudes are the 9Ps PVT noise, not a result, A2) |
| P5 | pre-registered trust mask (9Ps h_acc > 1 m or num_sv < 8; speed spikes > 3σ; gaps/stalls > 0.5 s; each ±2 s) | **WARN** | trusted 0.0% (GT-derived criteria alone: 100.0%); masked by h_acc>1 m 100.0%, num_sv<8 72.8% (receiver criteria before the ±2 s widening 100.0%), speed spikes 0.0% (0; σ 0.010 m/s, floor 0.03), gaps/stalls 0.0% (0/0); the h_acc criterion masks most rows because the 9Ps is a standalone (non-RTK) receiver: median h_acc 5.67 m — the pre-registered threshold needs a decision |
| P7 | RTK jumps / spikes (A54): fix-to-fix residual against the local velocity > max(0.5 m, 10σ); a jump and its return = displaced plateau, masked ±2 s | **INFO** | 0 jump(s) (thresholds h 0.50 / v 0.50 m; residual σ h 0.008 / v 0.013 m over 1028 fix intervals); masked 0.0 s of 206.2 s (0.00%); ATE-excluded (gaps/stalls + RTK) 0.0 s |
| P4r | GT time base used by the evaluation (AMENDMENTS A54 rule) | **INFO** | method chain: +107.1 ms (applied shift +107.1 ms, > 20 ms rule), confidence high, uncertainty 50.0 ms (peak-width precisions of the two chain links combined (Carter bound, 85.3 ms), capped by the 50 ms agreement with the record-time value that P4 PASS requires) |
| P6 | 2HP antenna lever arm | **INFO** | unknown (the C-RTK2HP antenna is not in calibration.md): an ATE floor of the order of the antenna separation on the top plate (~0.1-0.3 m, README Fig. 1); TEST_PLAN E1' estimates it jointly |

### GT 生成方式假设(对 RTK 定位逐点复算)

| hypothesis | direct RMS [m] | direct max [m] | Sim3 scale | Sim3 yaw [°] | Sim3 RMS [m] |
|---|---|---|---|---|---|
| ENU at the first GT fix (exact, WGS-84) | 7.19e-05 | 1.08e-04 | 1.000 | -1.03e-04 | 2.55e-05 |
| UTM zone 49 (E, N, alt) minus first | 0.476 | 0.804 | 1.000 | 1.252 | 2.48e-05 |
| equirectangular (R=6378137 m) minus first | 0.064 | 0.115 | 0.998 | -0.023 | 0.033 |
| ENU of GCJ-02-shifted lat/lon | 0.007 | 0.014 | 1.000 | 0.011 | 0.005 |

GCJ-02 在本地的绝对平移 466 m,轨迹范围内局部畸变 0.011 m。

### 时基互相关(GT 戳 t 对应其它时钟 t + lag)

| reference | lag [ms] | peak corr | peak width [ms] | delay precision [ms] |
|---|---|---|---|---|
| gt_course_rate__imu_yaw_rate | 415.611 | 0.754 | 940.000 | 32.715 |
| gt_speed__odom_speed | -26.436 | 0.995 | 1330.000 | 84.964 |
| odom_yaw_rate__imu_yaw_rate | 133.518 | 0.893 | 950.000 | 7.449 |
| gt_speed__pvt_speed_utc | 685.249 | 0.468 | 1000.000 | 834.520 |

### 天线超前模型(P4 诊断,不经轮速计)

航向角速率 = ω(t+L) + r_x·d/dt(ω/v):L = +139.7 ms,r_x = +0.31 m(车速中位 0.65 m/s,n = 823);r_x 置 0 时 L = +425.0 ms。天线在无侧滑点前方 r_x 处时 GT 航向**领先**车头 r_x/v,原始直接链路相对链式结果为正即此效应,不是钟差。

### 评测用 GT 时基(AMENDMENTS A54,`gt_timebase.yaml`)

规则:P4 通过 ⇒ 轮速计链;P4 失败 ⇒ 记录时间旁证;两者都没有 ⇒ 0 并标记;补偿按 TEST_PLAN §4(|值| > 20 ms 才补偿)。

| method | value [ms] | applied shift [ms] | confidence | uncertainty [ms] | P4 | chain [ms] | record time [ms] | sub-threshold chain [ms] | P4c replay (dev only, not used) [ms] |
|---|---|---|---|---|---|---|---|---|---|
| chain | +107.1 | +107.1 | high | 50.0 | WARN | +107.1 | +109.7 | — | — |

### RTK 跳变/毛刺与被掩时长(AMENDMENTS A54)

逐定位差分对局部速度(前后各 3 个定位区间的中位速度)的残差;跳变 = 水平 > 0.50 m 或垂直 > 0.50 m(max(0.5 m, 10σ),σ 水平 0.008 / 垂直 0.013 m);一对相互抵消的跳变之间为错位平台,连同前后 2 s 掩掉(bit 64)。

无跳变。

| criterion | masked [s] | of bag span [%] |
|---|---|---|
| h_acc>1m | 206.181 | 100.000 |
| num_sv<8 | 148.182 | 71.870 |
| speed_spike | 0.000 | 0.000 |
| gap_or_stall | 0.000 | 0.000 |
| no_pvt_within_0.2s | 0.000 | 0.000 |
| rtk_jump_plateau | 0.000 | 0.000 |
| ate_exclude | 0.000 | 0.000 |
| pre_registered_all | 206.181 | 100.000 |

bag 区间内 GT 时长 206.2 s;`ate_exclude` = 间断/冻结 + RTK 跳变平台(bit 8|64;bag 区间外 bit 32 本就不参评),区间见 `gt_mask_intervals.csv`(GT 戳与审计后时基两列)。h_acc/num_sv/速度尖峰(bit 1/2/4)只报告,不标 ATE 排除(未裁定)。

### 产物

`gt_clean.tum`、`gt_audited.tum`(= gt_clean + 0.107 s,方法 chain)、`gt_timebase.yaml`、`trust_mask.csv`、`gt_mask_intervals.csv`、`gt_audit.json`。

## 时间标定(tools/time_calib/gnss_time_calib.py,DESIGN §7)

### 方法①(记录时间统计与估计器时钟先验)

range_meas 记录时间 − GPST→UTC(leap 18 s,主机钟视角 = 主机钟偏差 + GNSS 传输延迟):中位 -38.7 ms、MAD 2.0 ms、p5/p95 -44.1/-36.3 ms、漂移 -43.3 ppm、持续台阶(>50 ms)无。

头戳 − 记录时间(判定 LiDAR 时基):/camera/color/image_raw/compressed 中位 -43.5 ms(p1 -55.5 / p99 -25.2,漂移 1.5 ppm);/livox/avia/imu 中位 100.6 ms(p1 50.7 / p99 151.1,漂移 45.4 ppm);/livox/avia/lidar 中位 50.1 ms(p1 47.7 / p99 56.1,漂移 43.9 ppm)。

估计器时钟(Livox 头戳)先验:GNSS「记录 − UTC」下包络 -39.1 ms 减 Avia IMU「记录 − 头戳」下包络 -145.1 ms ⇒ 上界 105.9 ms(主机钟台阶在两者中抵消;漂移 -1.5 ppm;估计器时钟台阶 无);GNSS 传输延迟不可观,按 [0, 150] ms 均匀(推断)⇒ 先验 30.9 ms、σ 75 ms。

方法①文件 `Dark01_record_time.yaml`(不读回放;method record_time,σ 75 ms;tool_version ≥ 3 按 AMENDMENTS A53 TDCP 不暂停,tool_version ≤ 2 的文件仍写着暂停):

```yaml
sequence: Dark01
tool_version: 3
leap_s: 18
segments:
  - t_from_gpst_s: 0
    offset_s: 0.030890596427934028
    drift_ppm: -1.49717860494643
    t_ref_gpst_s: 1732537129.201
    sigma_toff_s: 0.075
    tdcp_paused: false
    method: record_time
```

### 方法②(水平天线速度互相关)与方法③(DTW)

输入:回放轨迹 `/work/fastlivo2_gnss/runs/eval/preproc_Dark01/Dark01/U/s0/traj_W.tum`(tum,6171 个速度样本,估计器速度 std 0.22 m/s);多普勒 LS 速度 1879 个历元。窗口 5 个,可用 0 个(窗长/步长见 JSON)。状态:ran: no usable window (GNSS velocity covers 78% of the window (< 90%); insufficient excitation (speed std < 0.1 m/s) or data; peak 0.28) -> method-1 prior (record_time)。

信号(tool_version 3):horizontal antenna speed = |horizontal part of the zero-phase Gaussian-smoothed (0.25 s) velocity vector| on both sides, 100 Hz grid;W 的竖直方向:state_trace gravity (median g = [-0.178, 0.0128, -9.8074])。多普勒 LS:C/N0 ≥ 15.0 dB-Hz、仰角 ≥ 10.0°,按接收机 dopp_std 加权,两遍(第二遍固定平滑后的钟漂):1950 个历元有 ≥ 4 行,第二遍解出 1879 个(每历元用行中位 5.0)。

| 窗起点 [s] | 用 | 峰值相关 | 偏移 [ms] | 峰宽精度 [ms] | GNSS 覆盖 | 原因 |
|---|---|---|---|---|---|---|
| 0 | 否 | 0.277 | — | 724.5 | 100% |  |
| 30 | 否 | — | — | — | 78% | GNSS velocity covers 78% of the window (< 90%) |
| 60 | 否 | — | — | — | 78% | GNSS velocity covers 78% of the window (< 90%) |
| 90 | 否 | — | — | — | 96% | insufficient excitation (speed std < 0.1 m/s) or data |
| 120 | 否 | — | — | — | 96% | insufficient excitation (speed std < 0.1 m/s) or data |

标定文件(方法②无可用段或不如方法①准,逐段取方法① method record_time;A53:σ ≤ 0.1 s 即 TDCP 不暂停) `time_calib/Dark01.yaml`(AMENDMENTS A18 格式;所有 `*_gpst_s` 为 GTime::toSec(),1970 原点、GPST 尺度):

```yaml
sequence: Dark01
tool_version: 3
leap_s: 18
segments:
  - t_from_gpst_s: 0
    offset_s: 0.030890596427934028
    drift_ppm: -1.49717860494643
    t_ref_gpst_s: 1732537129.201
    sigma_toff_s: 0.075
    tdcp_paused: false
    method: record_time
```
