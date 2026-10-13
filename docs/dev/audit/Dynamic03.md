> Historical research record (through 2026-10-03). Paths are illustrative; archived runs are not bundled. For current status and executable entry points see [the repository README](../../../README.md). Current release docs/LICENSING.md supersedes historical license notes. This record does not authorize new experiments or assert current release validation.

# I2 数据审计 — Dynamic03

生成:`tools/m3dgr_audit.py`(tool_version 3),2026-09-25T08:23:48Z;bag `/work/fastlivo2_gnss/data/m3dgr/bags/Dynamic03.bag`(3434689371 字节,2025-08-02T15:47:04Z;sha256 未计算)。序列类别:**development**(AMENDMENTS A2)。

总评:**WARN**(INFO 7, PASS 13, SKIP 9, WARN 5)。远端归档:`/work/fastlivo2_gnss/audit/Dynamic03`。

## 本轮说明

P4-TIME(2026-09-25),tool_version 3;开发集(A2/A26),全部结果可看。**回放** = HEAD 的 U 臂(AMENDMENTS A55:`gnss.enable: false`,种子 0,eval 构建 `wt/p3i-strict`,build_sha256 `36a541331a5c4210…`),`runs/eval/p4_time_u/<seq>/U/s0`;其 state_trace 与已有的 ref-R 回放逐字节相同(Grass02 `2affc187…`、Grass01 `6e262080…`、Occlusion03 `96654a26…`)。P2b 版(tool_version 2)的归档备份在远端 `wt/p4-time/home/audit_backup_p2b/<seq>/`。方法、诊断与五条序列的对照见 INTERFACE_CHANGES「WP-g → P4-TIME」。BRDC `BRDC00IGS_R_20243410000_01D_MN.rnx`。

**时间标定(§7,A53)**:方法②无输入(每历元 0–3 颗星,0 个多普勒速度历元)⇒ `Dynamic03.yaml` = 方法① method record_time、−85.7 ms、σ_toff 75 ms、漂移 −8.1 ppm、**TDCP 不暂停**(A53;载波为 0)。

**GT 时基(A54)**:P4 PASS,轮速计链 +18.9 ms ⇒ method chain、不补偿(≤ 20 ms)、置信高(记录时间旁证 +37.4 ms;P4c +28.3 ms)。

**RTK 跳变(A54,P7)**:无(最大的竖直小跳 0.37 m 在阈值 0.5 m 以下);`ate_exclude` 0 s。

**I2 审计(本轮随回放重跑)**:G13 SKIP(无可用多普勒历元);G15 按 A43。

## 供 WP-a / 核心使用的结论

- bag 里没有 Kepler 星历(无 /ublox_driver/ephem 话题):URA 语义(A16)在本序列没有作用对象——星历全部来自 RINEX,其 sva 由 RTKLIB 自解(索引语义,WP-a 契约);E4 无从判定。validateConfig 仍要求 `source: gnss_comm_ros1` 时给出取值:同一接收机与 ublox_driver 的开发集/评测集序列(Grass01/02、Dark01、Outdoor01)判定一致为 metres,本序列配置沿用 `metres`(推断:同一驱动的字段语义不随序列变;对本序列数值无影响)。
- std 单位(§12.6-3):psr_std=metres,cp_std=cycles,dopp_std=hertz;cp_std=0.06 是 u-blox cpStdev=15 的『无效』标记,占 100.0%。
- 星历覆盖(A1):卫星-历元覆盖率 仅 bag 0.0% / bag+RINEX 100.0%;≥5 星历元比例 0.0% / 0.0% ⇒ 必须配置 `gnss.nav_rinex`。
- Klobuchar(A38):GPS 参数取 RINEX 头 GPSA/GPSB(本日可用);bag 无 iono_params。
- C/N0 中位 BDS 12, GLO 17, GPS 14 dB-Hz;按 DESIGN §10 L0 掩码(伪距 30 / 多普勒 25 / 载波 28 dB-Hz)可用率 伪距 BDS 0%, GLO 0%, GPS 0%、多普勒 BDS 0%, GLO 0%, GPS 1% ⇒ 默认掩码会饿死滤波与初始化(COLLECT 需 ≥5 星 SPP);掩码必须在开发集按 TEST_PLAN §3 重新标定,不得在本序列上调。
- time_pulse_info 话题不存在 ⇒ A19 的检查无对象;主机钟台阶只能从 range_meas 记录时间序列看(L1)。
- range_meas 记录时间 − GPST→UTC:中位 -69.9 ms、MAD 12.4 ms、漂移 14.6 ppm、台阶 无(= 主机钟偏差+到达延迟;作 §7 方法①先验,不是最终时偏)。
- 传感器时基(C4,记录时间 − 头戳中位,漂移):Avia LiDAR 27.1 ms / -70 ppm、Avia IMU -18.1 ms / 68 ppm、相机 42.5 ms。Livox 头戳 = 设备时间戳 + 常数(S1:timebase on another clock (offset 1.73346e+09 s, drift 0.0 ppm vs header)),与主机时钟的关系逐序列不同(本序列与『扫描起点主机时刻 + 约 100 ms 扫描』相容) ⇒ 估计器时基(LiDAR 头戳)对 GPST 的偏差与漂移只能由 §7 方法②逐序列标定;range_meas 记录时间只作先验。
- Livox:/livox/mid360/lidar livox_ros_driver2/CustomMsg [e4d6829b], /livox/avia/lidar livox_ros_driver/CustomMsg [e4d6829b] 与仓库 include/livox_ros_driver 头 md5 一致。
- CompressedImage format:{'rgb8; jpeg compressed bgr8': 8529};回放直读应 `IMREAD_COLOR` 并按 format 决定通道序(S2)。S2 的 PASS 是 Python 对两条解码路径的仿真,C++ 回放路径本身待 WP-f 实测。
- §12.6-7 杆臂方向:not evaluable: fewer than 10 GNSS epochs inside the replay's time span; time map record_time (sigma 75 ms), replay /r/runs/eval/p4_time_u/Dynamic03/U/s0;calibration.md 方向推导见 docs/dev/audit/m3dgr_calib.md。
- 自检 1 的 sat2freq 交叉核对(A35-C8,G14):全部一致。

## 检查表

| # | check | verdict | evidence |
|---|---|---|---|
| C1 | msg_def md5 recomputed = connection md5 (and = installed ROS type md5 where installed) | **PASS** | 12 connections, 0 mismatches; 8 standard types also matched the installed ROS packages |
| C2 | topics of the replay/GNSS path present (DESIGN §14; since A38 the RINEX NAV file is an input of every sequence) | **WARN** | missing: /ublox_driver/ephem (Kepler ephemerides only from the RINEX NAV file (A1/A26)); /ublox_driver/glo_ephem (GLONASS ephemerides only from the RINEX NAV file (A1/A26)); /ublox_driver/iono_params (no effect: the GPS Klobuchar set comes from the RINEX header (A38)); /ublox_driver/time_pulse_info (the A19 time_pulse_info checks have no object) |
| C3 | rates and time ranges (header-stamp period; record-time average) | **WARN** | lidar 10.0 Hz (header), avg 10.00 Hz; imu 200.2 Hz (header), avg 203.46 Hz; compressed 30.0 Hz (header), avg 29.98 Hz; range_meas avg 3.48 Hz (879 msgs); receiver_pvt avg 10.00 Hz (2845 msgs); gaps (>max(3x period, 0.25 s)) as (t since start or first header [s], length [s]): {'range_meas': [(5.3, 3.198), (9.0, 3.898), (18.7, 9.097)]} |
| C4 | header stamp vs record time per topic (median/p1/p99 ms, drift ppm) | **INFO** | /camera/color/image_raw/compressed 42.5/23.8/53.3 ms, 6 ppm; /livox/avia/imu -18.1/-86.3/61.8 ms, 68 ppm; /livox/avia/lidar 27.1/5.4/77.7 ms, -70 ppm; /mavros/global_position/raw/fix 19.3/-1.1/133.5 ms, 41 ppm |
| G1 | epoch interval 10 Hz, monotone, one time tag per epoch (§12.6-1) | **WARN** | 879 epochs; 96.58 % at 0.1 s; 0 non-increasing; 0 epochs with >1 time tag; 20 gaps >0.15 s |
| G15 | 1 Hz decimation rule (AMENDMENTS A43: the first epoch of every GPST second; the retired DESIGN §4.3 5 ms rule reported beside it) | **PASS** | A43 rule: 111 epochs for 111 GPST seconds with data; 10 Hz epoch grid offset +3.0 ms from the GPST tenths; the retired \|tow - round(tow)\| < 5 ms rule would select an epoch in 86.9% of the 107 seconds |
| G2 | per-satellite array lengths consistent; sat numbers inside the gnss_comm ranges | **PASS** | 0 inconsistent satellites; unknown sat numbers: none |
| G3 | (sys, freq, code) histogram and freqs->code table verdicts (signal_table.hpp) | **PASS** | C 1207.1400 MHz code=27->L7I: 122; C 1561.0980 MHz code=47->L2I: 366; G 1575.4200 MHz code=1->L1C: 163; R 1244.2500 MHz code=14->L2C: 19; R 1248.1875 MHz code=14->L2C: 59; R 1599.7500 MHz code=1->L1C: 240; R 1601.4375 MHz code=1->L1C: 85; verdicts {'Accepted': 1054}; primary slot first: {'slot0': 854, 'none': 197} |
| G4 | satellites per epoch (primary signal present) | **INFO** | total min/mean/max 0/1.0/3; BDS 0/0.4/2, GPS 0/0.2/1, GLO 0/0.4/1 |
| G5 | C/N0 distribution (dB-Hz, primary signal) | **INFO** | BDS median 12 p5 10 p95 14; GLO median 17 p5 12 p95 20; GPS median 14 p5 10 p95 24 |
| G6 | psr / cp / dopp availability (status bits + value; with the L0 C/N0 masks) | **INFO** | BDS psr 100% cp 0% dopp 100% (masked 0/0/0%); GLO psr 100% cp 0% dopp 100% (masked 0/0/0%); GPS psr 100% cp 0% dopp 100% (masked 0/0/1%) |
| G7 | LLI / status statistics | **INFO** | LLI {'GPS': {'3': 163}, 'BDS': {'3': 366}, 'GLO': {'3': 325}}; status {'GPS': {'1': 163}, 'BDS': {'1': 366}, 'GLO': {'1': 325}} |
| G8 | std unit judgement on the u-blox grids (§12.6-3) | **PASS** | psr metres (100.0% on 0.01*2^n m); cp cycles (100.0% on 0.004n cyc, 0.0% on 0.004n*lambda m, 0.0% on 0.004n/lambda); dopp hertz (100.0% on 0.002*2^n Hz, 0.0% as m/s); cp_std=0.06 (cpStdev=15, u-blox 'invalid') on 100.0% of slots |
| G9 | carrier-phase rate vs Doppler sign/scale (§12.6-2: corr > 0.99, slope ~ 1) | **SKIP** | fewer than 20 consecutive carrier-valid pairs |
| G10 | GLONASS channel k from freqs vs ephemeris frq (§12.3, §12.6-1) | **PASS** | 403 slots; mismatching satellites none; observed without any ephemeris: none |
| G14 | self-check 1: RTKLIB sat2freq(sat, code, nav) of every accepted slot = its freqs (A35-C8) | **PASS** | 1054 slots consistent, 0 mismatching, 0 unverifiable (GLONASS without an ephemeris channel: none), 0 slots not accepted by the table (G3); per system {'BDS': {'consistent': 488}, 'GPS': {'consistent': 163}, 'GLO': {'consistent': 403}} |
| E1 | bag Kepler ephemerides vs same-day RINEX (same sat,toe): \|dr\| < 1e-3 m, \|ddts\| < 1e-11 s (A9-3 tier 2) | **SKIP** | 0/0 matched, 0 outside tier 2 (0 not explained by a corrupted copy, see E7); worst per system {}; toe label shifts {}; field semantics {} |
| E7 | bag ephemeris stream consistency: all copies of one (sat, toe, IODE) identical | **PASS** | 0 keys, 0 with conflicting copies |
| E2 | bag GLONASS ephemerides vs RINEX | **SKIP** | 0 bag GLONASS ephemerides, none matched a RINEX record |
| E4 | URA semantics: bag ura vs same-day RINEX SV accuracy (§12.6-4, A16) | **SKIP** | no bag ephemeris matched a RINEX record |
| E3 | orbit code self-check: adjacent RINEX records agree at the midpoint (p95 < 10 m per system) | **PASS** | C median 0.14 m p95 0.50 m (n=258); E median 0.11 m p95 0.41 m (n=657); G median 0.41 m p95 1.94 m (n=96) |
| E5 | ephemeris coverage timeline: bag-only (causal by record time) vs bag+RINEX (A1, §12.6-8) | **WARN** | sat-epochs with a healthy ephemeris: bag-only 0.0%, bag+RINEX 100.0%; epochs with >=5: 0.0% / 0.0%; first >=5 at — / — s; BDS GEO observed none; never covered bag-only: C09, C14, C24, G26, G29, R02, R12 |
| E6 | Klobuchar source (A38): GPS set = RINEX header GPSA/GPSB; bag iono_params only a diagnostic (family classified) | **PASS** | GPS Klobuchar from the RINEX header GPSA/GPSB: ['2.794e-08', '7.451e-09', '-1.192e-07', '1.192e-07', '1.454e+05', '-1.802e+05', '0', '1.311e+05']; no /ublox_driver/iono_params in the bag |
| L1 | GNSS arrival latency: range_meas record time - GPST->UTC (leap 18 s) | **INFO** | median -69.9 ms, MAD 12.4, p1 -98.4, p99 -42.4, min -98.9, max -41.8; drift 14.6 ppm; persistent steps >50 ms: none; receiver_pvt median -65.6 ms |
| L3 | self-check 6 (time, record-time part): \|record - (GPST - leap)\| < 0.3 s (leap seconds and time system consistent) | **PASS** | median -0.0699 s with leap 18 s (a missing/extra leap second would show as ~±1 s, a GPST/UTC mix-up as ±18 s); the cross-correlation part of self-check 6 is method 2 of tools/time_calib/gnss_time_calib.py on the ref-R replay |
| L2 | receiver_lla header stamp convention | **INFO** | header.stamp - GPST(seconds, Unix origin) median 0.000 s (min 0.000, max 0.000): receiver_lla stamps are GPST without leap seconds (not ROS/UTC time) |
| T1 | time_pulse_info checks (A19: leap second, time system, gross errors, host clock steps) | **SKIP** | topic /ublox_driver/time_pulse_info absent from this bag (README §4.2 lists it); nothing to check. Host-clock steps are covered by the range_meas latency series instead (L1). |
| G11 | Doppler sign by LS fit | **SKIP** | no epoch with >= 5 Doppler satellites and an ephemeris |
| G12a | self-check 5 (position): SPP vs receiver PVT | **SKIP** | not evaluable: 0 open-sky epochs have an L0-masked SPP (C/N0 >= 30 dB-Hz leaves < 4 pseudoranges; masked SPP epochs 0 of 88); decode-only unmasked SPP vs PVT horizontal median — m (n=0) |
| G12b | self-check 5 (velocity): Doppler LS vs receiver PVT | **SKIP** | not evaluable: 0 open-sky epochs with an L0-masked Doppler LS (masked epochs 0 of 88) |
| G13 | lever-arm direction (§12.6-7): Doppler innovations with l vs -l on turns; regression on the omega x l term (~0; ~-2 = sign flipped) | **SKIP** | not evaluable: fewer than 10 GNSS epochs inside the replay's time span; time map record_time (sigma 75 ms), replay /r/runs/eval/p4_time_u/Dynamic03/U/s0 |
| S1 | Livox CustomMsg type/md5 vs the repository's livox_ros_driver headers | **PASS** | /livox/avia/lidar: livox_ros_driver/CustomMsg [e4d6829bdfe657cb6c21a746c86b21a6] vs repo livox_ros_driver/CustomMsg [e4d6829bdfe657cb6c21a746c86b21a6]; timebase on another clock (offset 1.73346e+09 s, drift 0.0 ppm vs header); points/scan median 24000 |
| S2 | CompressedImage format; replay direct decode vs republish path, bgr8 and grey pixel-exact (I2) | **PASS** | formats {'rgb8; jpeg compressed bgr8': 8529}; 150 sampled: bgr equal 150, grey equal 150, IMREAD_UNCHANGED==IMREAD_COLOR 150; shapes {'(480, 640, 3)': 150} (Python emulation of both decode paths with OpenCV; the C++ replay path itself is to be measured by WP-f) |
| S3 | GT file format (TUM: t x y z qx qy qz qw) | **WARN** | 4269 rows, columns {8: 4269}; 2403 non-increasing stamps (728 backwards, max 91.1 us, 1675 exact duplicates); quaternion identity on 100.0% rows (orientation not provided); repeated positions 66.7%; GT stamps = /mavros/global_position/raw/fix header stamps on 100.0% of rows |

## 话题清单

| topic | type | md5 | md5 recomputed | msgs | rate [Hz] (header, else avg) | record-header median [ms] |
|---|---|---|---|---|---|---|
| /livox/mid360/imu | sensor_msgs/Imu | 6a62c6daae103f4ff57a132d6f95cec2 | yes | 56903 | 202.04 | 6.0e-02 |
| /camera/imu | sensor_msgs/Imu | 6a62c6daae103f4ff57a132d6f95cec2 | yes | 56905 | 200.28 | -3.0 |
| /livox/mid360/lidar | livox_ros_driver2/CustomMsg | e4d6829bdfe657cb6c21a746c86b21a6 | yes | 2846 | 10.00 | 100.7 |
| /odom | nav_msgs/Odometry | cd5e73d190d741a2f92e81eda573aca7 | yes | 5690 | 20.00 | 6.7e-02 |
| /camera/aligned_depth_to_color/image_raw/compressedDepth | sensor_msgs/CompressedImage | 8f7a12909da2c9d3332d540a0977563f | yes | 8529 | 30.02 | 40.9 |
| /camera/color/image_raw/compressed | sensor_msgs/CompressedImage | 8f7a12909da2c9d3332d540a0977563f | yes | 8529 | 30.02 | 42.5 |
| /ublox_driver/range_meas | gnss_comm/GnssMeasMsg | 0eafeb530a3e6637722c4e296734c119 | yes | 879 | 3.48 | — |
| /ublox_driver/receiver_pvt | gnss_comm/GnssPVTSolnMsg | d18171357d7a159f76d4d7c0b12fb631 | yes | 2845 | 10.00 | — |
| /ublox_driver/receiver_lla | sensor_msgs/NavSatFix | 2d3a8cd499b9b4a0249fb98fd05cfa48 | yes | 2845 | 10.00 | -18065.6 |
| /mavros/global_position/raw/fix | sensor_msgs/NavSatFix | 2d3a8cd499b9b4a0249fb98fd05cfa48 | yes | 4269 | 15.00 | 19.3 |
| /livox/avia/imu | sensor_msgs/Imu | 6a62c6daae103f4ff57a132d6f95cec2 | yes | 57871 | 200.16 | -18.1 |
| /livox/avia/lidar | livox_ros_driver/CustomMsg | e4d6829bdfe657cb6c21a746c86b21a6 | yes | 2845 | 10.02 | 27.1 |

## 信号直方图 (sys, freq, code 字段 → signal_table)

| sys | freq [MHz] | code field | table code | count |
|---|---|---|---|---|
| C | 1207.1400 | 27 | L7I | 122 |
| C | 1561.0980 | 47 | L2I | 366 |
| G | 1575.4200 | 1 | L1C | 163 |
| R | 1244.2500 | 14 | L2C | 19 |
| R | 1248.1875 | 14 | L2C | 59 |
| R | 1599.7500 | 1 | L1C | 240 |
| R | 1601.4375 | 1 | L1C | 85 |

## 产物

`audit.json`(全部数字)、`latency.csv`、`eph_coverage.csv`、`eph_coverage_timeline.csv`、`plot_*.png`,位于远端归档目录。

## GT 审计(tools/gt_audit.py)

生成:`tools/gt_audit.py`(tool_version 3),2026-09-25T08:28:32Z。输入:GT 文件与原始 bag 的 RTK 定位话题、9Ps 接收机 PVT、Avia IMU、轮速计;诊断 P4c 另读 GNSS-free 回放 `/work/fastlivo2_gnss/runs/eval/p4_time_u/Dynamic03/U/s0`(仅开发集,A2)。

总评:**WARN**(INFO 5, PASS 3, SKIP 1, WARN 1)。远端归档:`/work/fastlivo2_gnss/audit/Dynamic03/gt`。

### 检查表

| # | check | verdict | evidence |
|---|---|---|---|
| P0 | GT file format and cleaning | **INFO** | 4269 rows; 728 backward stamp steps; 1990 rows reordered and 1675 duplicate-stamp rows dropped in gt_clean.tum; orientation identity on 100.0% rows (position-only GT) |
| P1 | GT provenance: stamps and projection reproduced from the RTK fix (C-RTK2HP) | **PASS** | 100.0% of GT stamps equal /mavros/global_position/raw/fix header stamps (1 us); best hypothesis 'ENU at the first GT fix (exact, WGS-84)' reproduces GT to 3.46e-04 m RMS (max 7.64e-04 m); other hypotheses: UTM zone 49 (E, N, alt) minus first 1.21 m; equirectangular (R=6378137 m) minus first 0.186 m; ENU of GCJ-02-shifted lat/lon 0.00996 m |
| P4 | GT time base vs the estimator clock (Avia IMU header) by cross-correlation; > 20 ms compensated | **PASS** | GT stamp -> Avia IMU header clock: +18.9 ms by chain GT->odom (speed) + odom->IMU (yaw rate); <= 20 ms: no compensation; record-time plausibility +37.4 ms (assumes equal transport delays; agreement within 50 ms required); antenna-lead fit (no odometry) L +44.7 ms, r_x +0.29 m (median speed 0.90 m/s, n=1197; r_x = 0 gives +266.5 ms); raw direct course-rate link +257.2 ms (peak 0.79): +238.3 ms against the chain = antenna lead r_x/v, not a clock term; links: gt_course_rate__imu_yaw_rate +257.2 ms (peak 0.788, delay precision 13.3 ms); gt_speed__odom_speed -33.3 ms (peak 0.992, delay precision 26.8 ms); odom_yaw_rate__imu_yaw_rate +52.3 ms (peak 0.912, delay precision 6.7 ms); gt_speed__pvt_speed_utc -1000.0 ms (peak 0.210, delay precision 2372.7 ms) |
| P4c | diagnostic: GT time base via the replay = GT speed vs \|v\| of the GNSS-free replay (estimator clock; development set only) | **INFO** | GT stamp -> estimator clock +28.3 ms (peak 0.964, delay precision 49.1 ms); +9.4 ms against the odometry chain; -9.1 ms against the record-time plausibility |
| P2 | datum: RTK fix (GT source) lat/lon vs 9Ps WGS-84 PVT | **PASS** | median horizontal difference 29.8 m vs a GCJ-02 shift of 466 m at this location ⇒ WGS-84 (no GCJ-02 offset) |
| P3 | GT (local XYZ) vs 9Ps PVT in ENU at the GT origin: horizontal SE(2)/Sim(2) structure (ENU, no rotation/scale expected) | **SKIP** | not decisive: 9Ps PVT noise vs trajectory extent (horizontal spread 82 m, 9 x 30 s blocks) gives yaw σ 17.88° > 1°; the frame is decided by P1; residual trend dE/dE -587.8 dN/dN -641.9 m/km; n=1350 (residual magnitudes are the 9Ps PVT noise, not a result, A2) |
| P5 | pre-registered trust mask (9Ps h_acc > 1 m or num_sv < 8; speed spikes > 3σ; gaps/stalls > 0.5 s; each ±2 s) | **WARN** | trusted 0.0% (GT-derived criteria alone: 89.7%); masked by h_acc>1 m 100.0%, num_sv<8 100.0% (receiver criteria before the ±2 s widening 100.0%), speed spikes 10.3% (9; σ 0.012 m/s, floor 0.03), gaps/stalls 0.0% (0/0); the h_acc criterion masks most rows because the 9Ps is a standalone (non-RTK) receiver: median h_acc 31.38 m — the pre-registered threshold needs a decision |
| P7 | RTK jumps / spikes (A54): fix-to-fix residual against the local velocity > max(0.5 m, 10σ); a jump and its return = displaced plateau, masked ±2 s | **INFO** | 0 jump(s) (thresholds h 0.50 / v 0.50 m; residual σ h 0.012 / v 0.021 m over 1421 fix intervals); masked 0.0 s of 284.4 s (0.00%); ATE-excluded (gaps/stalls + RTK) 0.0 s |
| P4r | GT time base used by the evaluation (AMENDMENTS A54 rule) | **INFO** | method chain: +18.9 ms (applied shift +0.0 ms, > 20 ms rule), confidence high, uncertainty 27.6 ms (peak-width precisions of the two chain links combined (Carter bound, 27.6 ms), capped by the 50 ms agreement with the record-time value that P4 PASS requires) |
| P6 | 2HP antenna lever arm | **INFO** | unknown (the C-RTK2HP antenna is not in calibration.md): an ATE floor of the order of the antenna separation on the top plate (~0.1-0.3 m, README Fig. 1); TEST_PLAN E1' estimates it jointly |

### GT 生成方式假设(对 RTK 定位逐点复算)

| hypothesis | direct RMS [m] | direct max [m] | Sim3 scale | Sim3 yaw [°] | Sim3 RMS [m] |
|---|---|---|---|---|---|
| ENU at the first GT fix (exact, WGS-84) | 3.46e-04 | 7.64e-04 | 1.000 | 1.09e-04 | 4.51e-05 |
| UTM zone 49 (E, N, alt) minus first | 1.210 | 2.128 | 1.000 | 1.255 | 1.46e-05 |
| equirectangular (R=6378137 m) minus first | 0.186 | 0.334 | 0.997 | -0.047 | 0.022 |
| ENU of GCJ-02-shifted lat/lon | 0.010 | 0.018 | 1.000 | 0.004 | 0.001 |

GCJ-02 在本地的绝对平移 466 m,轨迹范围内局部畸变 0.011 m。

### 时基互相关(GT 戳 t 对应其它时钟 t + lag)

| reference | lag [ms] | peak corr | peak width [ms] | delay precision [ms] |
|---|---|---|---|---|
| gt_course_rate__imu_yaw_rate | 257.210 | 0.788 | 740.000 | 13.331 |
| gt_speed__odom_speed | -33.317 | 0.992 | 1320.000 | 26.751 |
| odom_yaw_rate__imu_yaw_rate | 52.256 | 0.912 | 710.000 | 6.654 |
| gt_speed__pvt_speed_utc | -1000.000 | 0.210 | 1670.000 | 2372.699 |

### 天线超前模型(P4 诊断,不经轮速计)

航向角速率 = ω(t+L) + r_x·d/dt(ω/v):L = +44.7 ms,r_x = +0.29 m(车速中位 0.90 m/s,n = 1197);r_x 置 0 时 L = +266.5 ms。天线在无侧滑点前方 r_x 处时 GT 航向**领先**车头 r_x/v,原始直接链路相对链式结果为正即此效应,不是钟差。

### 评测用 GT 时基(AMENDMENTS A54,`gt_timebase.yaml`)

规则:P4 通过 ⇒ 轮速计链;P4 失败 ⇒ 记录时间旁证;两者都没有 ⇒ 0 并标记;补偿按 TEST_PLAN §4(|值| > 20 ms 才补偿)。

| method | value [ms] | applied shift [ms] | confidence | uncertainty [ms] | P4 | chain [ms] | record time [ms] | sub-threshold chain [ms] | P4c replay (dev only, not used) [ms] |
|---|---|---|---|---|---|---|---|---|---|
| chain | +18.9 | +0.0 | high | 27.6 | PASS | +18.9 | +37.4 | — | +28.3 |

### RTK 跳变/毛刺与被掩时长(AMENDMENTS A54)

逐定位差分对局部速度(前后各 3 个定位区间的中位速度)的残差;跳变 = 水平 > 0.50 m 或垂直 > 0.50 m(max(0.5 m, 10σ),σ 水平 0.012 / 垂直 0.021 m);一对相互抵消的跳变之间为错位平台,连同前后 2 s 掩掉(bit 64)。

无跳变。

| criterion | masked [s] | of bag span [%] |
|---|---|---|
| h_acc>1m | 284.420 | 100.000 |
| num_sv<8 | 284.420 | 100.000 |
| speed_spike | 29.383 | 10.331 |
| gap_or_stall | 0.000 | 0.000 |
| no_pvt_within_0.2s | 0.000 | 0.000 |
| rtk_jump_plateau | 0.000 | 0.000 |
| ate_exclude | 0.000 | 0.000 |
| pre_registered_all | 284.420 | 100.000 |

bag 区间内 GT 时长 284.4 s;`ate_exclude` = 间断/冻结 + RTK 跳变平台(bit 8|64;bag 区间外 bit 32 本就不参评),区间见 `gt_mask_intervals.csv`(GT 戳与审计后时基两列)。h_acc/num_sv/速度尖峰(bit 1/2/4)只报告,不标 ATE 排除(未裁定)。

### 产物

`gt_clean.tum`、`gt_audited.tum`(= gt_clean + 0.000 s,方法 chain)、`gt_timebase.yaml`、`trust_mask.csv`、`gt_mask_intervals.csv`、`gt_audit.json`。

## 时间标定(tools/time_calib/gnss_time_calib.py,DESIGN §7)

### 方法①(记录时间统计与估计器时钟先验)

range_meas 记录时间 − GPST→UTC(leap 18 s,主机钟视角 = 主机钟偏差 + GNSS 传输延迟):中位 -69.9 ms、MAD 12.4 ms、p5/p95 -97.2/-44.7 ms、漂移 14.6 ppm、持续台阶(>50 ms)无。

头戳 − 记录时间(判定 LiDAR 时基):/camera/color/image_raw/compressed 中位 -42.5 ms(p1 -53.3 / p99 -23.8,漂移 -5.7 ppm);/livox/avia/imu 中位 18.1 ms(p1 -61.8 / p99 86.3,漂移 -67.9 ppm);/livox/avia/lidar 中位 -27.1 ms(p1 -77.7 / p99 -5.4,漂移 70.2 ppm)。

估计器时钟(Livox 头戳)先验:GNSS「记录 − UTC」下包络 -74.6 ms 减 Avia IMU「记录 − 头戳」下包络 -70.6 ms ⇒ 上界 -10.7 ms(主机钟台阶在两者中抵消;漂移 -8.1 ppm;估计器时钟台阶 无);GNSS 传输延迟不可观,按 [0, 150] ms 均匀(推断)⇒ 先验 -85.7 ms、σ 75 ms。

方法①文件 `Dynamic03_record_time.yaml`(不读回放;method record_time,σ 75 ms;tool_version ≥ 3 按 AMENDMENTS A53 TDCP 不暂停,tool_version ≤ 2 的文件仍写着暂停):

```yaml
sequence: Dynamic03
tool_version: 3
leap_s: 18
segments:
  - t_from_gpst_s: 0
    offset_s: -0.08565593844761815
    drift_ppm: -8.109135863224578
    t_ref_gpst_s: 1733458960.403
    sigma_toff_s: 0.075
    tdcp_paused: false
    method: record_time
```

### 方法②(水平天线速度互相关)与方法③(DTW)

输入:回放轨迹 `/work/fastlivo2_gnss/runs/eval/p4_time_u/Dynamic03/U/s0/traj_W.tum`(tum,8517 个速度样本,估计器速度 std — m/s);多普勒 LS 速度 0 个历元。窗口 0 个,可用 0 个(窗长/步长见 JSON)。状态:ran: no usable window (too few Doppler-velocity epochs (0; a solution needs >= 5 rows with C/N0 >= 15 dB-Hz and el >= 10 deg) or trajectory samples) -> method-1 prior (record_time)。

信号(tool_version 3):horizontal antenna speed = |horizontal part of the zero-phase Gaussian-smoothed (0.25 s) velocity vector| on both sides, 100 Hz grid;W 的竖直方向:state_trace gravity (median g = [-0.3215, 0.0801, -9.8042])。多普勒 LS:C/N0 ≥ 15.0 dB-Hz、仰角 ≥ 10.0°,按接收机 dopp_std 加权,两遍(第二遍固定平滑后的钟漂):0 个历元有 ≥ 4 行,第二遍解出 — 个(每历元用行中位 —)。

标定文件(方法②无可用段或不如方法①准,逐段取方法① method record_time;A53:σ ≤ 0.1 s 即 TDCP 不暂停) `time_calib/Dynamic03.yaml`(AMENDMENTS A18 格式;所有 `*_gpst_s` 为 GTime::toSec(),1970 原点、GPST 尺度):

```yaml
sequence: Dynamic03
tool_version: 3
leap_s: 18
segments:
  - t_from_gpst_s: 0
    offset_s: -0.08565593844761815
    drift_ppm: -8.109135863224578
    t_ref_gpst_s: 1733458960.403
    sigma_toff_s: 0.075
    tdcp_paused: false
    method: record_time
```
