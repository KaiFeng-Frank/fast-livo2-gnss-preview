> Historical research record (through 2026-10-03). Paths are illustrative; archived runs are not bundled. For current status and executable entry points see [the repository README](../../../README.md). Current release docs/LICENSING.md supersedes historical license notes. This record does not authorize new experiments or assert current release validation.

# I2 数据审计 — Occlusion03

生成:`tools/m3dgr_audit.py`(tool_version 3),2026-09-25T08:22:35Z;bag `/work/fastlivo2_gnss/data/m3dgr/bags/Occlusion03.bag`(2653911393 字节,2026-09-24T18:31:46Z;sha256 未计算)。序列类别:**development**(AMENDMENTS A2)。

总评:**WARN**(INFO 8, PASS 16, SKIP 7, WARN 3)。远端归档:`/work/fastlivo2_gnss/audit/Occlusion03`。

## 本轮说明

P4-TIME(2026-09-25),tool_version 3;开发集(A2/A26),全部结果可看。**回放** = HEAD 的 U 臂(AMENDMENTS A55:`gnss.enable: false`,种子 0,eval 构建 `wt/p3i-strict`,build_sha256 `36a541331a5c4210…`),`runs/eval/p4_time_u/<seq>/U/s0`;其 state_trace 与已有的 ref-R 回放逐字节相同(Grass02 `2affc187…`、Grass01 `6e262080…`、Occlusion03 `96654a26…`)。P2b 版(tool_version 2)的归档备份在远端 `wt/p4-time/home/audit_backup_p2b/<seq>/`。方法、诊断与五条序列的对照见 INTERFACE_CHANGES「WP-g → P4-TIME」。BRDC `BRDC00IGS_R_20243360000_01D_MN.rnx`。

**时间标定(§7,A53)**:方法②跑了但没有可用窗:两遍多普勒 LS 能解的历元(≥ 5 行,C/N0 ≥ 15 dB-Hz)只有 264 个,散成 ≤ 6.3 s 的短段,3 个 60 s 窗的 GNSS 覆盖 52% / 27% / 9% < 90%(工具不再跨缺口相关)⇒ `Occlusion03.yaml` = 方法① method record_time、−114.1 ms、σ_toff 75 ms、漂移 −7.1 ppm、**TDCP 不暂停**(A53;本序列载波约 0,TDCP 本来也几乎没有)。

**GT 时基(A54)**:P4 FAIL(航向角速率链路峰值 0.72)⇒ method record_time −111.7 ms(补偿)、置信中(亚阈值链 −122.3 ms 在 50 ms 内相符;P4c −103.1 ms)。P3 集成时手取的是亚阈值链 −122.3 ms,按 A54 规则改为记录时间值(相差 10.6 ms)。

**RTK 跳变(A54,P7)**:无;`ate_exclude` 0 s(bag 截断后区间外的 969 行 GT 照旧 bit 32,不参评)。

**I2 审计(本轮随回放重跑)**:G13 SKIP(转弯行只有 3 条);G15 按 A43(退役规则在 +5 ms 网格上一秒都选不中)。

## 供 WP-a / 核心使用的结论

- bag 里没有 Kepler 星历(无 /ublox_driver/ephem 话题):URA 语义(A16)在本序列没有作用对象——星历全部来自 RINEX,其 sva 由 RTKLIB 自解(索引语义,WP-a 契约);E4 无从判定。validateConfig 仍要求 `source: gnss_comm_ros1` 时给出取值:同一接收机与 ublox_driver 的开发集/评测集序列(Grass01/02、Dark01、Outdoor01)判定一致为 metres,本序列配置沿用 `metres`(推断:同一驱动的字段语义不随序列变;对本序列数值无影响)。
- std 单位(§12.6-3):psr_std=metres,cp_std=cycles,dopp_std=hertz;cp_std=0.06 是 u-blox cpStdev=15 的『无效』标记,占 99.5%。
- 星历覆盖(A1):卫星-历元覆盖率 仅 bag 0.0% / bag+RINEX 100.0%;≥5 星历元比例 0.0% / 12.3% ⇒ 必须配置 `gnss.nav_rinex`。
- Klobuchar(A38):GPS 参数取 RINEX 头 GPSA/GPSB(本日可用);bag 无 iono_params。
- C/N0 中位 GLO 18, GPS 19 dB-Hz;按 DESIGN §10 L0 掩码(伪距 30 / 多普勒 25 / 载波 28 dB-Hz)可用率 伪距 GLO 0%, GPS 0%、多普勒 GLO 1%, GPS 3% ⇒ 默认掩码会饿死滤波与初始化(COLLECT 需 ≥5 星 SPP);掩码必须在开发集按 TEST_PLAN §3 重新标定,不得在本序列上调。
- time_pulse_info 话题不存在 ⇒ A19 的检查无对象;主机钟台阶只能从 range_meas 记录时间序列看(L1)。
- range_meas 记录时间 − GPST→UTC:中位 35.6 ms、MAD 0.8 ms、漂移 13.3 ppm、台阶 无(= 主机钟偏差+到达延迟;作 §7 方法①先验,不是最终时偏)。
- 传感器时基(C4,记录时间 − 头戳中位,漂移):Avia LiDAR 169.4 ms / 20 ppm、Avia IMU 119.4 ms / 26 ppm、相机 42.2 ms。Livox 头戳 = 设备时间戳 + 常数(S1:timebase on another clock (offset 1.73304e+09 s, drift 0.0 ppm vs header)),与主机时钟的关系逐序列不同(本序列与『扫描起点主机时刻 + 约 100 ms 扫描』相容) ⇒ 估计器时基(LiDAR 头戳)对 GPST 的偏差与漂移只能由 §7 方法②逐序列标定;range_meas 记录时间只作先验。
- 载波/多普勒符号(§12.6-2):斜率 0.9999、相关 1.00000 ⇒ `λ·dΦ/dt = −λD` 成立,与 DESIGN §1 一致。
- Livox:/livox/avia/lidar livox_ros_driver/CustomMsg [e4d6829b], /livox/mid360/lidar livox_ros_driver2/CustomMsg [e4d6829b] 与仓库 include/livox_ros_driver 头 md5 一致。
- CompressedImage format:{'rgb8; jpeg compressed bgr8': 7273};回放直读应 `IMREAD_COLOR` 并按 format 决定通道序(S2)。S2 的 PASS 是 Python 对两条解码路径的仿真,C++ 回放路径本身待 WP-f 实测。
- §12.6-7 杆臂方向:insufficient excitation (needs >= 200 turning rows and a lever term RMS >= 0.02 m/s): k = +139.50 (block-bootstrap s.e. 514.03, 7 blocks); innovation RMS on 3 turning rows (|yaw rate| >= 0.15 rad/s): l 0.000 / -l 0.000 / 0 0.000 m/s, lever term RMS 0.000 m/s (88 epochs, 96 rows; fitted yaw 98.13 / 97.84 / 97.98 deg); time map record_time (sigma 75 ms), replay /r/runs/eval/p4_time_u/Occlusion03/U/s0;calibration.md 方向推导见 docs/dev/audit/m3dgr_calib.md。
- 自检 1 的 sat2freq 交叉核对(A35-C8,G14):全部一致。

## 检查表

| # | check | verdict | evidence |
|---|---|---|---|
| C1 | msg_def md5 recomputed = connection md5 (and = installed ROS type md5 where installed) | **PASS** | 12 connections, 0 mismatches; 8 standard types also matched the installed ROS packages |
| C2 | topics of the replay/GNSS path present (DESIGN §14; since A38 the RINEX NAV file is an input of every sequence) | **WARN** | missing: /ublox_driver/ephem (Kepler ephemerides only from the RINEX NAV file (A1/A26)); /ublox_driver/glo_ephem (GLONASS ephemerides only from the RINEX NAV file (A1/A26)); /ublox_driver/iono_params (no effect: the GPS Klobuchar set comes from the RINEX header (A38)); /ublox_driver/time_pulse_info (the A19 time_pulse_info checks have no object) |
| C3 | rates and time ranges (header-stamp period; record-time average) | **INFO** | lidar 10.0 Hz (header), avg 10.00 Hz; imu 200.1 Hz (header), avg 203.46 Hz; compressed 30.0 Hz (header), avg 29.98 Hz; range_meas avg 10.00 Hz (2426 msgs); receiver_pvt avg 10.00 Hz (2426 msgs); no gaps |
| C4 | header stamp vs record time per topic (median/p1/p99 ms, drift ppm) | **INFO** | /camera/color/image_raw/compressed 42.2/21.8/55.6 ms, 10 ppm; /livox/avia/imu 119.4/69.9/168.2 ms, 26 ppm; /livox/avia/lidar 169.4/166.9/171.9 ms, 20 ppm; /mavros/global_position/raw/fix 7.7/0.4/107.3 ms, -1 ppm |
| G1 | epoch interval 10 Hz, monotone, one time tag per epoch (§12.6-1) | **PASS** | 2426 epochs; 100.00 % at 0.1 s; 0 non-increasing; 0 epochs with >1 time tag; 0 gaps >0.15 s |
| G15 | 1 Hz decimation rule (AMENDMENTS A43: the first epoch of every GPST second; the retired DESIGN §4.3 5 ms rule reported beside it) | **PASS** | A43 rule: 243 epochs for 243 GPST seconds with data; 10 Hz epoch grid offset +5.0 ms from the GPST tenths; the retired \|tow - round(tow)\| < 5 ms rule would select an epoch in 0.0% of the 244 seconds |
| G2 | per-satellite array lengths consistent; sat numbers inside the gnss_comm ranges | **PASS** | 0 inconsistent satellites; unknown sat numbers: none |
| G3 | (sys, freq, code) histogram and freqs->code table verdicts (signal_table.hpp) | **PASS** | G 1575.4200 MHz code=1->L1C: 4427; R 1245.1250 MHz code=14->L2C: 23; R 1598.0625 MHz code=1->L1C: 178; R 1599.7500 MHz code=1->L1C: 235; R 1600.3125 MHz code=1->L1C: 61; R 1600.8750 MHz code=1->L1C: 225; R 1604.8125 MHz code=1->L1C: 2186; R 1605.3750 MHz code=1->L1C: 47; verdicts {'Accepted': 7382}; primary slot first: {'slot0': 7359, 'none': 23} |
| G4 | satellites per epoch (primary signal present) | **INFO** | total min/mean/max 1/3.0/7; GPS 0/1.8/6, GLO 0/1.2/3 |
| G5 | C/N0 distribution (dB-Hz, primary signal) | **INFO** | GLO median 18 p5 12 p95 22; GPS median 19 p5 14 p95 24 |
| G6 | psr / cp / dopp availability (status bits + value; with the L0 C/N0 masks) | **INFO** | GLO psr 100% cp 0% dopp 100% (masked 0/0/1%); GPS psr 100% cp 1% dopp 100% (masked 0/0/3%) |
| G7 | LLI / status statistics | **INFO** | LLI {'GPS': {'3': 4398, '2': 29}, 'GLO': {'3': 2926, '2': 6}}; status {'GPS': {'1': 4400, '3': 27}, 'GLO': {'1': 2927, '3': 5}} |
| G8 | std unit judgement on the u-blox grids (§12.6-3) | **PASS** | psr metres (100.0% on 0.01*2^n m); cp cycles (100.0% on 0.004n cyc, 0.0% on 0.004n*lambda m, 0.0% on 0.004n/lambda); dopp hertz (100.0% on 0.002*2^n Hz, 0.0% as m/s); cp_std=0.06 (cpStdev=15, u-blox 'invalid') on 99.5% of slots |
| G9 | carrier-phase rate vs Doppler sign/scale (§12.6-2: corr > 0.99, slope ~ 1) | **PASS** | 24 pairs (all cp-valid pairs): corr 1.00000, slope 0.9999, median \|diff\| 0.086 m/s; per satellite (§12.6-2 '每星', decisive = >= 50 clean pairs and range-rate std >= 2 m/s): 0/0 decisive satellites with corr > 0.99 and \|slope - 1\| < 0.02 (slopes nan..nan), 0 not decisive; 1 s sign agreement lambda*dPhi vs dP —, vs -lambda*D*dt — (n=0) |
| G10 | GLONASS channel k from freqs vs ephemeris frq (§12.3, §12.6-1) | **PASS** | 2955 slots; mismatching satellites none; observed without any ephemeris: none |
| G14 | self-check 1: RTKLIB sat2freq(sat, code, nav) of every accepted slot = its freqs (A35-C8) | **PASS** | 7382 slots consistent, 0 mismatching, 0 unverifiable (GLONASS without an ephemeris channel: none), 0 slots not accepted by the table (G3); per system {'GPS': {'consistent': 4427}, 'GLO': {'consistent': 2955}} |
| E1 | bag Kepler ephemerides vs same-day RINEX (same sat,toe): \|dr\| < 1e-3 m, \|ddts\| < 1e-11 s (A9-3 tier 2) | **SKIP** | 0/0 matched, 0 outside tier 2 (0 not explained by a corrupted copy, see E7); worst per system {}; toe label shifts {}; field semantics {} |
| E7 | bag ephemeris stream consistency: all copies of one (sat, toe, IODE) identical | **PASS** | 0 keys, 0 with conflicting copies |
| E2 | bag GLONASS ephemerides vs RINEX | **SKIP** | 0 bag GLONASS ephemerides, none matched a RINEX record |
| E4 | URA semantics: bag ura vs same-day RINEX SV accuracy (§12.6-4, A16) | **SKIP** | no bag ephemeris matched a RINEX record |
| E3 | orbit code self-check: adjacent RINEX records agree at the midpoint (p95 < 10 m per system) | **PASS** | C median 0.13 m p95 0.63 m (n=254); E median 0.09 m p95 0.34 m (n=674); G median 0.30 m p95 1.67 m (n=98) |
| E5 | ephemeris coverage timeline: bag-only (causal by record time) vs bag+RINEX (A1, §12.6-8) | **WARN** | sat-epochs with a healthy ephemeris: bag-only 0.0%, bag+RINEX 100.0%; epochs with >=5: 0.0% / 12.3%; first >=5 at — / 7.0 s; BDS GEO observed none; never covered bag-only: G04, G07, G08, G09, G16, G26, G27, G31, R06, R07, R08, R09, R10, R18 |
| E6 | Klobuchar source (A38): GPS set = RINEX header GPSA/GPSB; bag iono_params only a diagnostic (family classified) | **PASS** | GPS Klobuchar from the RINEX header GPSA/GPSB: ['3.26e-08', '2.235e-08', '-1.192e-07', '5.96e-08', '1.536e+05', '-1.802e+05', '0', '1.966e+05']; no /ublox_driver/iono_params in the bag |
| L1 | GNSS arrival latency: range_meas record time - GPST->UTC (leap 18 s) | **INFO** | median 35.6 ms, MAD 0.8, p1 33.7, p99 37.4, min 33.5, max 38.3; drift 13.3 ppm; persistent steps >50 ms: none; receiver_pvt median 52.2 ms |
| L3 | self-check 6 (time, record-time part): \|record - (GPST - leap)\| < 0.3 s (leap seconds and time system consistent) | **PASS** | median +0.0356 s with leap 18 s (a missing/extra leap second would show as ~±1 s, a GPST/UTC mix-up as ±18 s); the cross-correlation part of self-check 6 is method 2 of tools/time_calib/gnss_time_calib.py on the ref-R replay |
| L2 | receiver_lla header stamp convention | **INFO** | header.stamp - GPST(seconds, Unix origin) median 0.000 s (min 0.000, max 0.000): receiver_lla stamps are GPST without leap seconds (not ROS/UTC time) |
| T1 | time_pulse_info checks (A19: leap second, time system, gross errors, host clock steps) | **SKIP** | topic /ublox_driver/time_pulse_info absent from this bag (README §4.2 lists it); nothing to check. Host-clock steps are covered by the range_meas latency series instead (L1). |
| G11 | Doppler sign by LS fit: -lambda*D = range rate (§1) vs flipped sign (all Dopplers, el >= 5°) | **PASS** | post-fit RMS median 0.181 m/s with the design sign vs 542.2 m/s flipped (27 epochs at 1.0 Hz; linearised at the SPP, receiver PVT when no SPP) |
| G12a | self-check 5 (position): SPP vs receiver PVT | **SKIP** | not evaluable: 0 open-sky epochs have an L0-masked SPP (C/N0 >= 30 dB-Hz leaves < 4 pseudoranges; masked SPP epochs 0 of 243); decode-only unmasked SPP vs PVT horizontal median 46.3 m (n=21) |
| G12b | self-check 5 (velocity): Doppler LS vs receiver PVT | **SKIP** | not evaluable: 0 open-sky epochs with an L0-masked Doppler LS (masked epochs 0 of 243) |
| G13 | lever-arm direction (§12.6-7): Doppler innovations with l vs -l on turns; regression on the omega x l term (~0; ~-2 = sign flipped) | **SKIP** | insufficient excitation (needs >= 200 turning rows and a lever term RMS >= 0.02 m/s): k = +139.50 (block-bootstrap s.e. 514.03, 7 blocks); innovation RMS on 3 turning rows (\|yaw rate\| >= 0.15 rad/s): l 0.000 / -l 0.000 / 0 0.000 m/s, lever term RMS 0.000 m/s (88 epochs, 96 rows; fitted yaw 98.13 / 97.84 / 97.98 deg); time map record_time (sigma 75 ms), replay /r/runs/eval/p4_time_u/Occlusion03/U/s0 |
| S1 | Livox CustomMsg type/md5 vs the repository's livox_ros_driver headers | **PASS** | /livox/avia/lidar: livox_ros_driver/CustomMsg [e4d6829bdfe657cb6c21a746c86b21a6] vs repo livox_ros_driver/CustomMsg [e4d6829bdfe657cb6c21a746c86b21a6]; timebase on another clock (offset 1.73304e+09 s, drift 0.0 ppm vs header); points/scan median 24000 |
| S2 | CompressedImage format; replay direct decode vs republish path, bgr8 and grey pixel-exact (I2) | **PASS** | formats {'rgb8; jpeg compressed bgr8': 7273}; 150 sampled: bgr equal 150, grey equal 150, IMREAD_UNCHANGED==IMREAD_COLOR 150; shapes {'(480, 640, 3)': 150} (Python emulation of both decode paths with OpenCV; the C++ replay path itself is to be measured by WP-f) |
| S3 | GT file format (TUM: t x y z qx qy qz qw) | **WARN** | 5952 rows, columns {8: 5952}; 3568 non-increasing stamps (115 backwards, max 3.8 us, 3453 exact duplicates); quaternion identity on 100.0% rows (orientation not provided); repeated positions 66.7%; GT stamps = /mavros/global_position/raw/fix header stamps on 61.1% of rows |

## 话题清单

| topic | type | md5 | md5 recomputed | msgs | rate [Hz] (header, else avg) | record-header median [ms] |
|---|---|---|---|---|---|---|
| /livox/mid360/imu | sensor_msgs/Imu | 6a62c6daae103f4ff57a132d6f95cec2 | yes | 48527 | 202.57 | 6.7e-02 |
| /camera/imu | sensor_msgs/Imu | 6a62c6daae103f4ff57a132d6f95cec2 | yes | 48520 | 200.19 | 0.4 |
| /odom | nav_msgs/Odometry | cd5e73d190d741a2f92e81eda573aca7 | yes | 4853 | 20.00 | 7.0e-02 |
| /camera/aligned_depth_to_color/image_raw/compressedDepth | sensor_msgs/CompressedImage | 8f7a12909da2c9d3332d540a0977563f | yes | 7273 | 30.01 | 40.4 |
| /camera/color/image_raw/compressed | sensor_msgs/CompressedImage | 8f7a12909da2c9d3332d540a0977563f | yes | 7273 | 30.01 | 42.2 |
| /livox/avia/imu | sensor_msgs/Imu | 6a62c6daae103f4ff57a132d6f95cec2 | yes | 49340 | 200.15 | 119.4 |
| /livox/avia/lidar | livox_ros_driver/CustomMsg | e4d6829bdfe657cb6c21a746c86b21a6 | yes | 2427 | 10.02 | 169.4 |
| /ublox_driver/range_meas | gnss_comm/GnssMeasMsg | 0eafeb530a3e6637722c4e296734c119 | yes | 2426 | 10.00 | — |
| /ublox_driver/receiver_pvt | gnss_comm/GnssPVTSolnMsg | d18171357d7a159f76d4d7c0b12fb631 | yes | 2426 | 10.00 | — |
| /ublox_driver/receiver_lla | sensor_msgs/NavSatFix | 2d3a8cd499b9b4a0249fb98fd05cfa48 | yes | 2426 | 10.00 | -17947.8 |
| /mavros/global_position/raw/fix | sensor_msgs/NavSatFix | 2d3a8cd499b9b4a0249fb98fd05cfa48 | yes | 3639 | 15.01 | 7.7 |
| /livox/mid360/lidar | livox_ros_driver2/CustomMsg | e4d6829bdfe657cb6c21a746c86b21a6 | yes | 2426 | 10.00 | 100.7 |

## 信号直方图 (sys, freq, code 字段 → signal_table)

| sys | freq [MHz] | code field | table code | count |
|---|---|---|---|---|
| G | 1575.4200 | 1 | L1C | 4427 |
| R | 1245.1250 | 14 | L2C | 23 |
| R | 1598.0625 | 1 | L1C | 178 |
| R | 1599.7500 | 1 | L1C | 235 |
| R | 1600.3125 | 1 | L1C | 61 |
| R | 1600.8750 | 1 | L1C | 225 |
| R | 1604.8125 | 1 | L1C | 2186 |
| R | 1605.3750 | 1 | L1C | 47 |

## 产物

`audit.json`(全部数字)、`latency.csv`、`eph_coverage.csv`、`eph_coverage_timeline.csv`、`plot_*.png`,位于远端归档目录。

## GT 审计(tools/gt_audit.py)

生成:`tools/gt_audit.py`(tool_version 3),2026-09-25T08:27:56Z。输入:GT 文件与原始 bag 的 RTK 定位话题、9Ps 接收机 PVT、Avia IMU、轮速计;诊断 P4c 另读 GNSS-free 回放 `/work/fastlivo2_gnss/runs/eval/p4_time_u/Occlusion03/U/s0`(仅开发集,A2)。

总评:**FAIL**(FAIL 1, INFO 4, PASS 2, SKIP 1, WARN 2)。远端归档:`/work/fastlivo2_gnss/audit/Occlusion03/gt`。

### 检查表

| # | check | verdict | evidence |
|---|---|---|---|
| P0 | GT file format and cleaning | **INFO** | 5952 rows; 115 backward stamp steps; 230 rows reordered and 3453 duplicate-stamp rows dropped in gt_clean.tum; orientation identity on 100.0% rows (position-only GT) |
| P1 | GT provenance: stamps and projection reproduced from the RTK fix (C-RTK2HP) | **PASS** | 61.0% of GT stamps equal /mavros/global_position/raw/fix header stamps (1 us); best hypothesis 'ENU at the first GT fix (exact, WGS-84)' reproduces GT to 1.21e-03 m RMS (max 2.28e-03 m); other hypotheses: UTM zone 49 (E, N, alt) minus first 2.35 m; equirectangular (R=6378137 m) minus first 0.365 m; ENU of GCJ-02-shifted lat/lon 0.00999 m |
| P4 | GT time base vs the estimator clock (Avia IMU header) by cross-correlation; > 20 ms compensated | **FAIL** | no valid correlation chain (peaks: {'gt_course_rate__imu_yaw_rate': 0.64, 'gt_speed__odom_speed': 0.99, 'odom_yaw_rate__imu_yaw_rate': 0.72, 'gt_speed__pvt_speed_utc': 0.12}): GT time base undetermined by the pre-registered rule; AMENDMENTS A54: the record-time value is used instead (gt_timebase.yaml); for the record, the chain of the sub-threshold links would give -122.3 ms (delay precision 35.2 ms, peaks 0.99/0.72); record-time plausibility -111.7 ms; antenna-lead fit (no odometry) L -88.9 ms, r_x +0.34 m (median speed 0.91 m/s, n=1066; r_x = 0 gives +147.1 ms); links: gt_course_rate__imu_yaw_rate +132.2 ms (peak 0.641, delay precision 22.1 ms); gt_speed__odom_speed -36.0 ms (peak 0.991, delay precision 34.3 ms); odom_yaw_rate__imu_yaw_rate -86.3 ms (peak 0.716, delay precision 8.0 ms); gt_speed__pvt_speed_utc +693.8 ms (peak 0.125, delay precision 1536.7 ms) |
| P4c | diagnostic: GT time base via the replay = GT speed vs \|v\| of the GNSS-free replay (estimator clock; development set only) | **INFO** | GT stamp -> estimator clock -103.1 ms (peak 0.992, delay precision 26.9 ms); +19.2 ms against the sub-threshold odometry chain; +8.6 ms against the record-time plausibility |
| P2 | datum: RTK fix (GT source) lat/lon vs 9Ps WGS-84 PVT | **PASS** | median horizontal difference 14.2 m vs a GCJ-02 shift of 466 m at this location ⇒ WGS-84 (no GCJ-02 offset) |
| P3 | GT (local XYZ) vs 9Ps PVT in ENU at the GT origin: horizontal SE(2)/Sim(2) structure (ENU, no rotation/scale expected) | **SKIP** | not decisive: 9Ps PVT noise vs trajectory extent (horizontal spread 59 m, 9 x 30 s blocks) gives yaw σ 3.62° > 1°; the frame is decided by P1; residual trend dE/dE -697.5 dN/dN -211.7 m/km; n=2339 (residual magnitudes are the 9Ps PVT noise, not a result, A2) |
| P5 | pre-registered trust mask (9Ps h_acc > 1 m or num_sv < 8; speed spikes > 3σ; gaps/stalls > 0.5 s; each ±2 s) | **WARN** | trusted 0.0% (GT-derived criteria alone: 92.3%); masked by h_acc>1 m 100.0%, num_sv<8 93.4% (receiver criteria before the ±2 s widening 99.7%), speed spikes 7.7% (21; σ 0.013 m/s, floor 0.03), gaps/stalls 0.0% (0/0); fractions over the 1530 GT rows inside the bag's record span (969 rows after/before it: no sensor data, never trusted); the h_acc criterion masks most rows because the 9Ps is a standalone (non-RTK) receiver: median h_acc 9.70 m — the pre-registered threshold needs a decision |
| P7 | RTK jumps / spikes (A54): fix-to-fix residual against the local velocity > max(0.5 m, 10σ); a jump and its return = displaced plateau, masked ±2 s | **INFO** | 0 jump(s) (thresholds h 0.50 / v 0.50 m; residual σ h 0.011 / v 0.024 m over 1981 fix intervals); masked 0.0 s of 243.6 s (0.00%); ATE-excluded (gaps/stalls + RTK) 0.0 s |
| P4r | GT time base used by the evaluation (AMENDMENTS A54 rule) | **WARN** | method record_time: -111.7 ms (applied shift -111.7 ms, > 20 ms rule), confidence medium, uncertainty 50.0 ms (the agreement P4 requires between the record-time value and a chain (A40)); P4 FAIL: record-time value (A54) |
| P6 | 2HP antenna lever arm | **INFO** | unknown (the C-RTK2HP antenna is not in calibration.md): an ATE floor of the order of the antenna separation on the top plate (~0.1-0.3 m, README Fig. 1); TEST_PLAN E1' estimates it jointly |

### GT 生成方式假设(对 RTK 定位逐点复算)

| hypothesis | direct RMS [m] | direct max [m] | Sim3 scale | Sim3 yaw [°] | Sim3 RMS [m] |
|---|---|---|---|---|---|
| ENU at the first GT fix (exact, WGS-84) | 0.001 | 0.002 | 1.000 | -1.22e-04 | 1.76e-04 |
| UTM zone 49 (E, N, alt) minus first | 2.351 | 3.695 | 1.000 | 1.252 | 2.67e-05 |
| equirectangular (R=6378137 m) minus first | 0.365 | 0.550 | 0.997 | -0.067 | 0.028 |
| ENU of GCJ-02-shifted lat/lon | 0.010 | 0.019 | 1.000 | 0.002 | 0.004 |

GCJ-02 在本地的绝对平移 466 m,轨迹范围内局部畸变 0.011 m。

### 时基互相关(GT 戳 t 对应其它时钟 t + lag)

| reference | lag [ms] | peak corr | peak width [ms] | delay precision [ms] |
|---|---|---|---|---|
| gt_course_rate__imu_yaw_rate | 132.213 | 0.641 | 780.000 | 22.124 |
| gt_speed__odom_speed | -35.994 | 0.991 | 1430.000 | 34.311 |
| odom_yaw_rate__imu_yaw_rate | -86.321 | 0.716 | 770.000 | 7.963 |
| gt_speed__pvt_speed_utc | 693.769 | 0.125 | 1360.000 | 1536.711 |

### 天线超前模型(P4 诊断,不经轮速计)

航向角速率 = ω(t+L) + r_x·d/dt(ω/v):L = -88.9 ms,r_x = +0.34 m(车速中位 0.91 m/s,n = 1066);r_x 置 0 时 L = +147.1 ms。天线在无侧滑点前方 r_x 处时 GT 航向**领先**车头 r_x/v,原始直接链路相对链式结果为正即此效应,不是钟差。

### 评测用 GT 时基(AMENDMENTS A54,`gt_timebase.yaml`)

规则:P4 通过 ⇒ 轮速计链;P4 失败 ⇒ 记录时间旁证;两者都没有 ⇒ 0 并标记;补偿按 TEST_PLAN §4(|值| > 20 ms 才补偿)。

| method | value [ms] | applied shift [ms] | confidence | uncertainty [ms] | P4 | chain [ms] | record time [ms] | sub-threshold chain [ms] | P4c replay (dev only, not used) [ms] |
|---|---|---|---|---|---|---|---|---|---|
| record_time | -111.7 | -111.7 | medium | 50.0 | FAIL | — | -111.7 | -122.3 | -103.1 |

### RTK 跳变/毛刺与被掩时长(AMENDMENTS A54)

逐定位差分对局部速度(前后各 3 个定位区间的中位速度)的残差;跳变 = 水平 > 0.50 m 或垂直 > 0.50 m(max(0.5 m, 10σ),σ 水平 0.011 / 垂直 0.024 m);一对相互抵消的跳变之间为错位平台,连同前后 2 s 掩掉(bit 64)。

无跳变。

| criterion | masked [s] | of bag span [%] |
|---|---|---|
| h_acc>1m | 243.600 | 100.000 |
| num_sv<8 | 228.391 | 93.757 |
| speed_spike | 18.603 | 7.637 |
| gap_or_stall | 0.000 | 0.000 |
| no_pvt_within_0.2s | 0.800 | 0.329 |
| rtk_jump_plateau | 0.000 | 0.000 |
| ate_exclude | 0.000 | 0.000 |
| pre_registered_all | 243.600 | 100.000 |

bag 区间内 GT 时长 243.6 s;`ate_exclude` = 间断/冻结 + RTK 跳变平台(bit 8|64;bag 区间外 bit 32 本就不参评),区间见 `gt_mask_intervals.csv`(GT 戳与审计后时基两列)。h_acc/num_sv/速度尖峰(bit 1/2/4)只报告,不标 ATE 排除(未裁定)。

### 产物

`gt_clean.tum`、`gt_audited.tum`(= gt_clean + -0.112 s,方法 record_time)、`gt_timebase.yaml`、`trust_mask.csv`、`gt_mask_intervals.csv`、`gt_audit.json`。

## 时间标定(tools/time_calib/gnss_time_calib.py,DESIGN §7)

### 方法①(记录时间统计与估计器时钟先验)

range_meas 记录时间 − GPST→UTC(leap 18 s,主机钟视角 = 主机钟偏差 + GNSS 传输延迟):中位 35.6 ms、MAD 0.8 ms、p5/p95 34.0/37.1 ms、漂移 13.3 ppm、持续台阶(>50 ms)无。

头戳 − 记录时间(判定 LiDAR 时基):/camera/color/image_raw/compressed 中位 -42.2 ms(p1 -55.6 / p99 -21.8,漂移 -10.0 ppm);/livox/avia/imu 中位 -119.4 ms(p1 -168.2 / p99 -69.9,漂移 -26.4 ppm);/livox/avia/lidar 中位 -169.4 ms(p1 -171.9 / p99 -166.9,漂移 -20.1 ppm)。

估计器时钟(Livox 头戳)先验:GNSS「记录 − UTC」下包络 35.2 ms 减 Avia IMU「记录 − 头戳」下包络 74.0 ms ⇒ 上界 -39.1 ms(主机钟台阶在两者中抵消;漂移 -7.1 ppm;估计器时钟台阶 无);GNSS 传输延迟不可观,按 [0, 150] ms 均匀(推断)⇒ 先验 -114.1 ms、σ 75 ms。

方法①文件 `Occlusion03_record_time.yaml`(不读回放;method record_time,σ 75 ms;tool_version ≥ 3 按 AMENDMENTS A53 TDCP 不暂停,tool_version ≤ 2 的文件仍写着暂停):

```yaml
sequence: Occlusion03
tool_version: 3
leap_s: 18
segments:
  - t_from_gpst_s: 0
    offset_s: -0.1140783316419057
    drift_ppm: -7.148349869474038
    t_ref_gpst_s: 1733040204.255
    sigma_toff_s: 0.075
    tdcp_paused: false
    method: record_time
```

### 方法②(水平天线速度互相关)与方法③(DTW)

输入:回放轨迹 `/work/fastlivo2_gnss/runs/eval/p4_time_u/Occlusion03/U/s0/traj_W.tum`(tum,7265 个速度样本,估计器速度 std 0.26 m/s);多普勒 LS 速度 384 个历元。窗口 3 个,可用 0 个(窗长/步长见 JSON)。状态:ran: no usable window (GNSS velocity covers 27% of the window (< 90%); GNSS velocity covers 52% of the window (< 90%); GNSS velocity covers 9% of the window (< 90%)) -> method-1 prior (record_time)。

信号(tool_version 3):horizontal antenna speed = |horizontal part of the zero-phase Gaussian-smoothed (0.25 s) velocity vector| on both sides, 100 Hz grid;W 的竖直方向:state_trace gravity (median g = [-0.2549, 0.0367, -9.8077])。多普勒 LS:C/N0 ≥ 15.0 dB-Hz、仰角 ≥ 10.0°,按接收机 dopp_std 加权,两遍(第二遍固定平滑后的钟漂):522 个历元有 ≥ 4 行,第二遍解出 384 个(每历元用行中位 5.0)。

| 窗起点 [s] | 用 | 峰值相关 | 偏移 [ms] | 峰宽精度 [ms] | GNSS 覆盖 | 原因 |
|---|---|---|---|---|---|---|
| 0 | 否 | — | — | — | 52% | GNSS velocity covers 52% of the window (< 90%) |
| 30 | 否 | — | — | — | 27% | GNSS velocity covers 27% of the window (< 90%) |
| 60 | 否 | — | — | — | 9% | GNSS velocity covers 9% of the window (< 90%) |

标定文件(方法②无可用段或不如方法①准,逐段取方法① method record_time;A53:σ ≤ 0.1 s 即 TDCP 不暂停) `time_calib/Occlusion03.yaml`(AMENDMENTS A18 格式;所有 `*_gpst_s` 为 GTime::toSec(),1970 原点、GPST 尺度):

```yaml
sequence: Occlusion03
tool_version: 3
leap_s: 18
segments:
  - t_from_gpst_s: 0
    offset_s: -0.1140783316419057
    drift_ppm: -7.148349869474038
    t_ref_gpst_s: 1733040204.255
    sigma_toff_s: 0.075
    tdcp_paused: false
    method: record_time
```
