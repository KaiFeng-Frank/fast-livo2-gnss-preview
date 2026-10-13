> Advanced research tooling. For the supported public build/example path start with [REPRODUCING.md](../../docs/REPRODUCING.md). Historical run paths below are illustrative; acquire external data from its provider and supply your own workspace settings.

# tools/datasets — A80 的新数据:取数、瘦身、转换(SmartPNT-MSF mini 平台、MARS-LVIG HKisland_GNSS03)

归属:本目录是 AMENDMENTS A80 的数据准备工具。SPDX:BSD-3-Clause,Python 3.8(项目镜像 `fastlivo2-gnss:noetic`)+ numpy + rosbags(Apache-2.0,独立程序)+ cv2。
**一切联网、下载、解包、转换都在远端做**(本机只编辑与提交):数据放 `/data2/fl2g/`(远端新盘,200 GiB),`/data2` 任何时候保留 ≥ 20 GB。
**数据纪律(A2)**:评测集序列只做取数、格式层面的统计、时基与真值格式转换;**不回放、不打分、不做 GNSS-only 解与真值比较**
(`smartpnt/gnss_check.py dev` 对非开发集序列拒绝执行)。

## 数据划分(A80 §二,写死)

| 类 | 序列 | 来源 |
|---|---|---|
| 开发集 | `Data01_20240707_100C_CAR_complex`、`Data02_20240718_100C_CAR_complex` | SmartPNT-MSF `00_MINI/` |
| 开发集(管线打通) | `HKisland_GNSS03` | MARS-LVIG(hf-mirror 转存) |
| 评测集 | `Data01_20240719_100C_CAR_opensky`、`Data00_20240703_100C_CAR_complex`、`Data00_20240706_100C_CAR_complex` | SmartPNT-MSF `00_MINI/` |

类别的唯一来源是 `tools/fl2g/sequences.py`(`tools/eval/sequences.json` 与之逐项核对,单测 `tools/eval/tests/test_registry.py`)。

## 工具清单

```
tools/datasets/
  dscommon/                    公共件
    rinex.py                   RINEX 3 观测流式读取;RTKLIB-EX 的频点槽与编码优先级(rtkcmn.c codepris / code2freq_*)
    gnsscomm.py                RINEX → gnss_comm 消息字节(GnssMeasMsg / GnssEphemMsg / GnssGloEphemMsg / iono_params);
                               定义与 md5 从参考 bag(M3DGR 的 Grass01.bag)的连接头逐字节取,布局逐字段核对
    rosbytes.py                Imu / CompressedImage / PointCloud2 的原始字节解析与改写(整数纳秒戳)
    bagmerge.py                按记录时间归并多路有序流、rosbags 写包、保序线程池、懒读多文件
    timebase.py                戳规则:一个序列一个整数纳秒平移,午夜回绕展开
    httpfetch.py               断点、多连接 Range 下载;Content-Length / ETag 尺寸 / 门户尺寸核对;sha256;可重解析的签名 URL
    brdc.py                    IGS BRDC(BKG 镜像)下载与解压
    ie_ref.py                  Inertial Explorer `.ref` 读取、姿态约定、杆臂识别
    layout.py                  第二个工程根 /data2/fl2g 的目录布局(run_matrix / eval_ate 所需的链接与取数记录)
  smartpnt/
    portal.py                  门户树、每序列文件角色识别、原包存取(sha256 记录)
    fetch_smartpnt.py          plan / fetch:列出与下载一条序列需要的模态
    build_slim.py              一条序列的瘦身 bag:流式下载 → 转换 → 删除,写 build.json
    slim.py                    逐消息转换:IMU 增量→速率、相机缩图、VLP-16 时间列、到达时间模型
    gt_convert.py              `.ref` → gt_clean.tum / gt_audited.tum / gt_i300.tum / gt_timebase.yaml / trust_mask.csv / ...
    extrinsics.py              Kalibr 相机链 + LiDAR→IMU 取值 → FAST-LIVO2 的 extrin_calib 块
    calib_check.py             `gravity`:静止段的加速度计向上方向与扫描地面法向(LiDAR→IMU 倾角核对)
    gnss_check.py              `stats`(格式层面,各类序列:卫星数/C-N0/载波/星历覆盖、分系统卫星数、RAIM 口径颗数的时间分布与 <4 颗的断档)与 `dev`(仅开发集:SPP 对真值、速度互相关)
    gt_stats.py                真值自身的统计(各类序列):对 bag 的覆盖、Q/AmbStatus/σ3d、信任掩码、路程、速度与范围——只读转换后的真值目录与 build.json
    imu_gt_check.py            IMU 对真值姿态(各类序列):陀螺沿真值"上"轴的积分对真值航向变化(IMU 标度、真值是否属于这条录制)、1 s 间隔滞后扫描(两钟)、起始静止窗的 |加计|/重力与陀螺偏置
    time_check.py              时基核对(仅开发集):陀螺偏航率对多普勒航向角速率、加计前向力对多普勒速度导数,±150 s 粗扫 + ±3 s 细扫 + 120 s 分窗
    overlay_check.py           相机↔LiDAR 外参核对:静止段累积扫描投影到图像,边缘/互信息得分随旋转偏移的曲线,叠加图 PNG
    probe_ate.py / plot_run.py 探针/回放对真值的评分(杆臂 ATE、体坐标系里的行驶方向)与轨迹+误差随时间图(仅开发集)
  probe_replay.sh              诊断回放:自由叠加 yaml、起点偏移、时长(LIO-only、外参候选、LiDAR 时间偏移);容器参数同 run_matrix
  probe_overlays/              上述探针的叠加文件(lio_only、extrin_A–D、tl_*)
  mars/
    fetch_mars.py              hf-mirror 取包(sha256 对 hub 的 LFS oid)、BRDC
    build_slim.py              单遍过滤:保留话题、相机缩图、其余字节不动;转换真值
    gt_convert.py              DJI RTK 位置 → TUM(+ mask、gt_timebase.yaml)
  verify_bag.py                成品 bag 的验收(独立读回路径):gnss_comm 四个连接与参考 bag 逐字节一致、IMU 网格与量级、相机尺寸与到达常数、
                               LiDAR 逐点 time(µs、从 0 起)、GNSS 历元的 GPST→UTC(记录时间 − 历元 = 80 ms 逐历元)、各流同钟;任一项不过退出码 1
  tests/                       单测(unittest;只在远端镜像里跑,见末节)
```
配置与登记(本目录之外):`config/smartpnt_mini.yaml`、`config/smartpnt_mini_camera.yaml`、`config/mars_lvig_gnss.yaml`、
`launch/{smartpnt_mini,mars_lvig}_replay.launch`、`tools/eval/{sequences,lidars}.json`、`tools/fl2g/sequences.py`;
`tools/eval/gtbase.py` 增加 GT 时基方法 `gpst`(同钟,平移必为 0)与 `xcorr`(速度互相关测得)。

## 命令(远端,镜像内;`tools/` 同步到远端后)

```bash
# 列出一条序列要取的文件(--check:对每个文件 HEAD,核对服务器 Content-Length 与门户的四舍五入尺寸)
python3 tools/datasets/smartpnt/fetch_smartpnt.py plan Data01_20240707_100C_CAR_complex --roles small,config --check
# 一条序列的瘦身 bag(流式取 LiDAR 与 cam00,一次预取一个,用完即删;约 25 GB 下载 / 序列,成品 10–22 GB)
python3 tools/datasets/smartpnt/build_slim.py <seq> --ref-bag /r/data/m3dgr/filtered/Grass01.bag
#   探针:--bags 2:14 --no-camera / --max-seconds 90
#   一个序列同一时刻只能有一个构建(<seq>/.build.lock,flock;第二个直接退出——曾有两个队列同时排到同一条序列,互相截断了对方的 bag);
#   已建成的序列(build.json 与 bag 尺寸相符)再次排到时跳过,--rebuild 强制重建。构建完整一条约 35–55 分钟,受下载速度限制,并行两个队列不会更快。
# MARS
python3 tools/datasets/mars/fetch_mars.py fetch && python3 tools/datasets/mars/build_slim.py
# 成品 bag 验收(每个序列 bag 建成后必跑;写 --json 与 bag 放一起)
python3 tools/datasets/verify_bag.py <bag> --ref-bag /r/data/m3dgr/filtered/Grass01.bag --json <seq>/verify.json   # MARS 无 /imu_raw 等:只用于 SmartPNT
# GNSS 内容统计(格式层面,各类序列;近似位置默认取真值目录的 ENU 原点,只是个地点)与开发集的对真值检查(dev 对非开发集拒绝)
python3 tools/datasets/smartpnt/gnss_check.py stats <bag> --nav <brdm> --gt-dir <seq>/gt --seq <seq> --out <seq>/gnss_check.json
python3 tools/datasets/smartpnt/gnss_check.py dev <bag> --nav <brdm> --gt-dir <seq>/gt --seq <seq>
# 数据自身的核对(不用估计器、不用真值,各类序列都可):LiDAR 与 IMU 的倾角(窗口 = 距 bag 起点的秒数,取首帧扫描之后的静止段),IMU 与平台 IMR
python3 tools/datasets/smartpnt/calib_check.py gravity <bag> --t-from 15 --t-to 40
python3 tools/datasets/smartpnt/imr_check.py <bag> <seq>/raw/.../01_SINS/I300.imr --out <seq>/imr_check.json
# 真值自身的统计与 IMU 对真值姿态(不读估计器与 GNSS 解,各类序列都可;读 <root>/smartpnt/<seq>/)
python3 tools/datasets/smartpnt/gt_stats.py <seq> [<seq> ...] --root /data2/fl2g --out gt_stats.json
python3 tools/datasets/smartpnt/imu_gt_check.py <seq> --root /data2/fl2g --out imu_gt_check.json
# 时基核对与相机↔LiDAR 核对(开发集;时间标定用 U 臂回放,杆臂与 IMU 话题照 lidars.json)
python3 tools/datasets/smartpnt/time_check.py <bag> --nav <brdm> --gt-dir <seq>/gt --seq <seq> --out <seq>/time_check.json   # 步行速度序列加 --min-speed 0.4 --smooth-s 1.0
python3 tools/time_calib/gnss_time_calib.py <bag> <U 臂回放目录> audit/<seq>/time_calib/<seq>.yaml --seq <seq> --nav-rinex <brdm> \
    --lever-arm -0.118 0 0.120 --imu-topic /imu_raw --approx-from-gt <seq>/gt --plot        # 瘦身包没有 receiver_pvt,位置只取地点
python3 tools/datasets/smartpnt/overlay_check.py <bag> --out-dir <seq>/overlay --seq <seq>
# 诊断回放(LIO-only 等)与评分
FL2G_ROOT=/data2/fl2g FL2G_BUILD_ROOT=${FL2G_ROOT:?set FL2G_ROOT to the project directory} SEQ=<seq> tools/datasets/probe_replay.sh <名> smartpnt/bags/<seq>.bag smartpnt_mini_replay.launch <起点 s> <时长 s> <叠加 yaml>...
python3 tools/datasets/smartpnt/probe_ate.py --run runs/probe/<名> --seq <seq>
python3 tools/datasets/smartpnt/plot_run.py --run runs/eval/<矩阵>/<seq>/U/s0 --seq <seq> --out <png>
```
长任务用 `systemd-run --user`(`-p Nice=19 -p IOSchedulingClass=idle`)起,容器内 `nice -n 19`;回放最多 3 个并发,启动前 MemAvailable ≥ 16 GiB(run_matrix 的闸)。

## 第二个工程根与 run_matrix

新数据在 `/data2/fl2g`,当作第二个工程根使用(**不改 run_matrix**):

```bash
FL2G_BUILD_ROOT=${FL2G_ROOT:?set FL2G_ROOT to the project directory} python3 tools/eval/run_matrix.py run --root /data2/fl2g \
    --lidar smartpnt_mini --devel head=<wt>:ws/devel --arms U --seqs <seq> ...   # 构建树仍在主根的 wt/ 下
```
工程根下(`dscommon/layout.py`):`smartpnt/bags/<seq>.bag`(相对符号链接)、`<seq>.bag.sha256`、`<seq>.fetch.json`(`brdc` = gnss.nav_rinex)、
`audit/<seq>/gt/`(gt_clean.tum、gt_audited.tum、gt_timebase.yaml、trust_mask.csv)。链接都是相对的,容器只挂工程根,因此在容器里照常解析。
`audit/<seq>/time_calib/<seq>.yaml`(A18)还没有:run_matrix 找不到时按"无标定"跑,GNSS 臂的时间标定在开发集定参时做。

## SmartPNT-MSF 的转换规则

门户:`http://smartpnt-datasets.com/smart-pnt-dataset-api`(无需登录);文件在 `http://oss.smartpnt-datasets.com:8009/<SmartPNT/SmartPNT 下的路径>`
(nginx,支持 Range)。门户的尺寸是四舍五入到 0.01 单位的字符串(`"78.17 MB"`),不是字节数,也没有公布校验和;所以权威尺寸是服务器的
Content-Length(并与 nginx ETag 里的尺寸、门户的舍入值核对),我们记录的 sha256 是**收到的字节**的哈希,文件身份 = (路径, 尺寸, ETag, sha256),
写进 `<root>/<seq>/build.json` 的 `raw_files` 与 `fetch.json`。

**取哪些文件**(`portal.Layout` 按角色识别,序列之间目录布局不同:0702–0707 为 `ROVE_02_mini` + `ROVE_02_GroundTruth.ref`,0718/0719 为
`03_mini/ROVE_03` + `Mini_GroundTruth.ref`;基站优先 `BASE_01`,否则 `BASE_SGG`/`BASE.24o`):mini 内置接收机 RINEX、广播星历 `brdm*.24p`、
IMU rosbag 与 `I300.imr`、`03_LiDAR/*.bag`(VLP-16)、`02_CAM/cam00/*.bag`、`04_GroundTruth/*.ref`、`06_Description`、基站 RINEX(`--with-base`);
共享的 `03_config`(Kalibr 链、`mini_LiDAR.yaml` 等)与 `02_pytools`。`cam01–03`、`allcam`、IMR 之外的惯导原始日志、精密星历、图件不取。

### 时间(核实过的 GPST/UTC 规则)

* **事实**:bag 头的日期是错的(2014-07-xx,真实 2024-07-xx),**时刻 = GPST 的时刻**。IMU 戳与 `I300.imr`(IMR 头:`iUtcOrGpsTime=2` GPS、`iRcvTimeOrCorrTime=2`
  已校正)的记录时间逐位一致(0707:首尾 `28490.7257 / 30580.7678`),只是日期被换掉。bag 文件名里的时刻(PC 钟)比戳早/晚 1–2 分钟,不可用。
* **规则**(`dscommon/timebase.py`):整条序列所有话题加同一个整数纳秒平移 `shift = (首个 IMU 戳所在 GPST 日的 GPST 秒) − 闰秒(GPST 日期对应的 18 s) − 首个 IMU 戳`,
  GPST 日取"离本序列自己的 GPS 数据(mini 接收机 RINEX 首历元)最近的那一天",超过 1 h 报错;戳后退约一天(过 GPST 午夜)则展开。结果是 UTC Unix 秒,
  与仓库其余部分的约定一致(`time_map.cpp`:传感器戳 = UTC,GPST = UTC + 闰秒),`time.base: host`、`leap: auto` 照旧。
* **同一个常数(五条序列都如此)**:五条 SmartPNT 序列算出的 `shift_ns` 完全相同,= 315 964 782 s = (GPS 历元 1980-01-06 的 Unix 秒 315 964 800) − 18:
  源戳就是"自 1980-01-06 起的 GPST 秒数,被当作 Unix 秒写进头",所以 2024 年读成 2014 年。戳因此是连续的(0706 跨 GPST 午夜,戳不回绕,`Unwrapper` 没有触发),
  按 RINEX 首历元选 GPST 日只是冗余的一致性核对(选出的日子与常数吻合)。
* **端到端核对**(开发集 0707,12 分钟分析包):U 臂(LIO)水平速度与真值水平速度的互相关最佳滞后 −0.027 s(相关 0.992;滞后的分辨率受 1 Hz 真值限制);
  SPP 对真值的误差随历元时标平移的扫描,3D RMS 最小在 0…+5 ms(−20 ms 与 +20 ms 处升到 1.5×)——GNSS 历元戳、18 s 闰秒、真值时钟三者一致到几个 ms。
* **到达时间模型**(记录时间;源 bag 的记录时间就是头戳,没有到达信息;回放按记录时间送入,DESIGN §13):IMU = 戳;LiDAR = 首点戳 + 扫描时长(帧尾);
  相机 = 戳 + 50 ms(Grass01 D435i 的 45.8 ms);GNSS 历元 = 历元 UTC + 80 ms(Grass01 F9P 的 79.3 ms)。常数在 `slim.py`,写进 build.json。
  它们只决定送入顺序与时间标定方法①的先验,不改任何测量值;LIVO 模式由图像驱动(`sync_packages`),相机晚到不会丢图。

### IMU

`/imu_raw` 是**增量**(每 5 ms 样本的角增量 rad、速度增量 m/s),`frame_id` 恒为 `imu`,头 `seq` 恒为 0。静止样本 |Δv| = 0.04908 = g·0.005(×200 = 9.816 m/s²),
IMR 头写 200 Hz,所以瘦身包存速率 = 值 × 200(`slim.IMU_RATE_HZ`);姿态协方差首项 −1(无姿态),其余协方差 0。I300 轴:x 下、y 右、z 后
(Data_Description 表 2-3;静止时 f_x = −9.82,GT 对行驶方向给出 −z;Kalibr 链里相机光轴 = −z)。陀螺零偏大(静止均值 (0, 6.5e-4, −1.2e-3) rad/s),宿主在线估计。
实测白噪声密度(静止段 5–15 s、30–45 s;样本差分标准差 / √2):陀螺 3.4–4.4e-5 rad/s/√Hz,加速度计 0.5–1.1e-3 m/s²/√Hz。

### 相机

cam00,MER-131 全局快门,1280×1024,3 通道 JPEG(~850 KB/帧),10 Hz,戳落在 GPST 的 0.1 s 格点上(硬件触发)。瘦身:`cv2.imdecode` → `cv2.resize(INTER_LINEAR)` 到 640×512 →
JPEG 质量 95 → `CompressedImage`,`format` 原样(`bgr8; jpeg compressed bgr8`)。**为何是 640×512**:相机 yaml 里写原始分辨率的 Kalibr 内参、`scale: 0.5`,宿主处理尺寸
= 640×512,与瘦身图尺寸相同,宿主不再缩放;对精确 2 倍,`INTER_LINEAR` 的双线性取样等于 2×2 均值,也就是宿主自己 `processFrame` 里的 `cv::resize` 会做的事(只多一次 JPEG)。
标定:Kalibr 相机-IMU 链(`camchain-imucam-2024-06-26-02-19-12_0.yaml`,话题 `/cam00` 与 `/imu_raw`;`timeshift_cam_imu = 0.0310331 s`,t_imu = t_cam + shift,
宿主 `img_time_offset = +0.0310331`)。**不用** `03_config/mini_cam00.yaml`:它是 `/cam1` 的 VINS 文件,内参与外参和 Kalibr 链都对不上。

### LiDAR

VLP-16 的 `PointCloud2`(x y z intensity float32、ring uint16、time float32,point_step 22,单回波每帧约 14.8k 点,`dense`)。**点不动,只改 `time` 列与头戳**:
点的绝对时刻 = 头戳 + `time`(无论驱动把 `time` 参考到帧首还是帧尾,逐扫描按此定义统一),瘦身云的头戳 = 本帧最早点的时刻,`time` = 距它的**微秒**数
(宿主 `velodyne_handler` 的 `curvature = time/1000` 是毫秒,即输入须为 µs,否则整帧被当成一个时刻;`given_offset_time` 还要求最后一点 `time > 0`)。
本数据集(0707)的源 `time` 是秒、帧首参考、单调,扫描周期 0.10086 s(9.914 Hz,头戳等差);单位在第一帧上由量级定,逐帧核对(`unit_flips`),非单调帧按时间重排(`reordered_scans`)。
`blind: 1.0`:近场(< 1 m 仅 0.1 %,1.5 m 内 0.7 %)没有静态的平台结构(最拥挤的格子在 61 帧里只出现 16 次)。LiDAR 离地约 1.0 m,开发序列速度 0.6–1.5 m/s。

### LiDAR → IMU 外参(数据决定)

`mini_LiDAR.yaml` 是 LIO-SAM 配置,有多组被注释的候选,没写属于哪颗 IMU。核对(`config/smartpnt_mini.yaml` 头里有完整数字):
* **倾角**:静止段加速度计"向上"与扫描主平面法向(RANSAC)——24 个轴向旋转里,倾角一致(< 10°)的恰是 x_imu = −z_lidar 的四个(0.9–1.8°),其中包括被启用的那组 `[0 0 −1; 0 −1 0; −1 0 0]`(1.6°)。
* **偏航**:对这四个候选各跑一次前 200 s 的 LIO(只用 LiDAR + IMU):ATE 几乎相同(0.52–0.54 m,平台太慢,IMU 水平加速度几乎不起作用),但**估计的机体系里的行驶方向**
  只有这一组给出 (0.001, −0.038, −0.999)(= I300 的前向 −z),其余三组分别落在 +z、+y、−y;真值给出同期的行驶方向 (−0.020, −0.022, −0.9996),偏航一致到约 1°。外参取精确的轴置换,没有拟合角度。
* **平移** `extrinsic_T = [0.03411 0 0]` = −R·extTrans(extTrans = IMU 原点在 LiDAR 轴下的位置),3.4 cm,数据里测不出。
* **LiDAR-IMU 时钟**:`lidar_time_offset` −0.06 / −0.03 / 0 / +0.03 s 的 LIO 平均有效特征数 4716 / 4727 / 4748 / 4736,0 附近最大;灵敏度低(0.6 m/s),只能说明没有大偏移。
* LiDAR→相机 `Rcl, Pcl` = Kalibr 的 T_cam_imu 与上面的 LiDAR→IMU 复合(`extrinsics.py`)。

**第二轮核对(2026-10-03,两条开发序列;诊断回放 `probe_replay.sh`,评分 `probe_ate.py`,叠加文件 `probe_overlays/`;远端 `/data2/fl2g/runs/probe/`)**
* **偏航候选(LIO-only,行驶方向在 IMU 系里对真值)**:四个候选的 ATE 互相不可分(Data02 前 300 s:A 3.02、B 2.73、C 2.88、D 2.85 m;0707 的 600–900 s,真值可信段,路程 231 m:A 0.136、B 0.230、C 0.124、D 0.239 m),
  **行驶方向**分开它们:A 与真值差 1.6–1.7°(Data02 300 s:1.65°;0707 600–900 s:1.67°),B 178°,C 88°,D 92°(两序列同)。次要判据:0707 的 600–900 s 里 A 的平均有效特征数最大、残差最小
  (A 4721 / 0.0321,B 4690 / 0.0353,C 4690 / 0.0341,D 4669 / 0.0338)。IMU 自身的前向轴由真值姿态给出:行驶方向在 I300 系为 (0.003, −0.03, −1.00),即 −z;加计前向力与多普勒速度导数的相关也与这个轴符号一致。
* **倾角**:静止段 0707(69–150 s)、Data02(20–100 s)的加速度计向上方向与扫描地面法向,对被启用的 A:1.59° / 1.72°,方向相同(都沿 I300 的 y 轴,即绕车辆纵轴的横滚),
  两天一致,所以是安装的系统倾角、不是路面坡度;`extrinsics.py --gravity-check` 取一个 1.656° 的纵轴旋转后两窗残差都是 0.09°。**没有采用**:在 Data02 前 600 s 的 LIO-only 上它没有改善(ATE 159 → 174 m),在 900 s 里也不影响 LIVO 在高架段的冻结(见下)。作为已知残差记录。
* **相机↔LiDAR**(`overlay_check.py`;静止段累积 80 帧扫描,边缘得分与互信息随相机系旋转偏移的曲线,叠加图在 `<seq>/overlay/*.png`,每对图的第二张是故意加 3° 偏航误差的同一投影):
  标定位置的边缘得分是扫描最大值的 75–98 %;最大值偏移 0707:偏航 +1.1°、俯仰 +1.0°、横滚 +2.6°;Data02:偏航 +0.5°、俯仰 −2.5°、横滚 +1.2°。偏航与横滚两序列同号,俯仰反号:度量的复现性约 ±2.5°,
  没有 ≥ 3° 的错误或轴混淆;横滚的正偏移与上面 1.66° 的纵轴倾角同号同量级。互信息曲线太平(最大值落在扫描范围端点),不作判据。我没有看这些图(本机不拷数据),结论只来自得分。
* **LiDAR 时钟**:Data02(16 m/s,300 s,LIO-only)的 `lidar_time_offset` −0.06 / −0.03 / 0 / +0.03 / +0.06 s:ATE 3.07 / 3.07 / 3.13 / 3.24 / 3.35 m,平均有效特征数 4126 / 4111 / 4125 / 4117 / 4123,残差 0.0251 / 0.0252 / 0.0249 / 0.0252 / 0.0250:
  平坦,LIO 的地图自洽使它看不见常数时间偏移;没有偏移的证据,也没有它的测量。
* **IMU 标度与偏置**:静止 |加计| 在两序列都是 9.817 m/s²,当地重力(Somigliana + 自由空气)9.7936,比 1.0024;Data02 全序列的陀螺积分与真值航向变化:斜率 0.9975、10 s 区间残差均值 0.0003°,全序列累计 642.8° 对 642.8°。IMU 没有毛病。

### 宿主(FAST-LIVO2)在 SmartPNT 上的三件事(开发集,2026-10-03;上面的偏航/倾角/时钟诊断回放用的是当时的配置,体素 0.5 m、滑动关)

**1. 内存**:上游默认的 `map_sliding_en: false` 让 LIO 体素地图只增不减。Data02(车,16 m/s)的整段地图 600 s 后 148 万体素、9.9 GiB,880 s 后 242 万体素、15.9 GiB,撞上 run_matrix 的 16 GiB 容器限额被杀(`node_exit_code -9`);
2000–4000 s 的序列要 40–80 GiB,超过整机。`map_sliding_en: true`(上游键、上游值:`half_map_size 100`、`sliding_thresh 8`)后 600 s 是 2.9 GiB、1.2 万体素,前 300 s 的 ATE 3.3 m(不滑动 3.0 m,LIO-only 3.0 m)。
视觉地图不是主因:`vio/grid_size` 60 / 100 对 600 s 峰值的影响 < 8 %(9.5 / 9.2 对 9.9 GiB)。地图滑动后整段的峰值(终版配置)是 Data02 10.1 GiB(2214 s)、0707 10.3 GiB(2090 s)、MARS 5.7 GiB(391 s),
剩余增长是视觉地图(随帧数线性,约 5 MB/s):3994 s 的 0719 评测序列约 19 GiB,**要给那条序列的 run_matrix 调高 `--mem-limit-gb`(≥ 24)**,默认 16 不够。

**2. 相机时间偏移**:Kalibr 的 `timeshift_cam_imu` +0.0310331 s 是 2024-06-26 标定录制的值,对这些录制不成立。数据自己的测量(`cam_imu_toff.py`:上半幅图像的帧间平移对陀螺绕相机水平轴的旋转积分,1 ms 步长):
0707(450–1000 s)俯仰通道峰在 +3.9 ms(自助法 3.7–4.0 ms,ρ 0.94;在 +31 ms 处 0.85),偏航通道 +8.4 ms;Data02(130–700 s)俯仰通道峰在 +0.5 ms(自助法 −6…+4 ms),偏航通道在 ±120 ms 内平坦(转弯太慢)。
用 +0.031 s 时 Data02 的 LIVO 在 600–800 s 的高架路段(16 m/s)位姿冻结(估计路程 / 真值路程 0.01),31 ms 在那个速度是 0.5 m;偏移取 0、−0.031 s、或 VIO 协方差 ×10 都让同一窗口正常通过(≥ 0.95),且与 LIO-only 同分。
`config/smartpnt_mini.yaml` 取 `img_time_offset: 0.004`(测量值,效应 < 0.1 m)。

**3. LIO 体素大小**:Data02 在 1000–1250 s 的隧道段(GNSS 星数从 19 掉到 6,高度降到 −8 m),体素 0.5 m 时 LIO 位姿冻结:100 s 窗口的估计/真值路程比 0.59、0.017、0.62(LIO-only 与 LIVO 相同,与视觉无关);
16 线 VLP-16 在 13–17 m/s 下 0.5 m 的体素里点太少、成不了平面。`lio.voxel_size: 1.0`(上游自己在大场景用的范围:`config/MARS_LVIG.yaml` 是 2.0、注释里写 1.0)后同三个窗口是 0.999、0.990、0.999(ATE 7 m / 窗),前 600 s 134 m 对 159 m。
同样的 LIO-only 1300 s 回放里,点滤波全开(`point_filter_num 1`)与倾角校正的外参**仍然冻结**(0.33 / 0.016 与 0.28 / 29),只有体素 1 m 通过;IMU 过程噪声取 0.1 倍、0.01 倍发散(600 s:ATE 740 m、3.2 万 m),取 4 倍在 900 s 内的高架段冻结。
终版配置(体素 1.0、滑动图、`img_time_offset 0.004`)下三条开发序列的整段结果见 AMENDMENTS 的"A80 数据记录(开发集)"五;留下的误差是 LIO 的累积漂移(Data02 每 100 s 窗口 0.4–1.4 % 路程,整段 3.6 %),不是冻结。

### GNSS(RINEX → gnss_comm,写进同一个 bag)

**两条路的比较与选择**:仓库有 `source: rinex`(RTKLIB `readrnxt` 经 `gnss_io/rinex_source.cpp` 直接读 RINEX)与 gnss_comm 消息两条路。两条路的信号选择规则相同(槽位、码优先级、频点表、一码一信号、主频点在前),
差别在输入载体:RINEX 路没有记录时间(`t_record = 0`,到达延迟与桥接按 `rinex.latency_s` 人为注入)、没有接收机 std 与 status 位(由 RTKLIB 的 P/L≠0 与 LLI 合成)、整文件进内存(10 Hz × 66 min ≈ 40 万历元的 `obsd_t`);
gnss_comm 路与 M3DGR 的 F9P 输入同构(记录时间、status、LLI、std 字段齐全),进核心前的代码路径完全相同。
**在同一份数据上并排比过**(开发集 0707 的 12 分钟分析包,T 臂,前 240 s,`gnss_epochs.csv` 逐历元对比;RINEX 路 = `gnss.source: rinex` + 原始 `ROVE_02.24o`、`latency_s: 0.08`):
2273 个共同历元里 GPS / GLO / GAL / BDS 的卫星数 **100 % 相同**;COLLECT → TRACKING → ACTIVATE 发生在同一历元(GPST 28668.5,`epochs_used=423`,yaw σ 0.5–0.6°),相位序列相同;
唯一的结构差别是 **QZSS**:gnss_comm 编号没有 QZSS,bag 路丢掉,RINEX 路按 `qzs_as_gps` 当 GPS 用(每历元多 1 颗 J,`n_total_raw` 13.2 对 12.2,故 `psr_cand` 94 %、`dopp_cand` 39 % 的历元相同);
另外 RINEX 路从接收机文件的首历元起送(早于第一帧 LiDAR 的 1411 个历元落在 WAIT_CORE)。
**选 gnss_comm 路**:卫星集合相同;它与 M3DGR 在核心里走同一条解码与时间映射路径(U/T 与 M3DGR 的比较不引入第二条输入路径),保留记录时间与因果送入,不依赖 RTKLIB 对每种 RINEX 变体的解析
(五条序列的 mini 文件头都是 RTKCONV 2.4.3 b34 转的 RINEX 3.03),也不需要 gnss.rinex 的整文件内存;代价是 QZSS 的 0–2 颗卫星(0707 的 12 分钟包:每历元约 1.3 颗 J、1.8 颗 S;SBAS 两条路都不用)。
开发集端到端验证:T 臂回放(`config/arms/T.yaml` + brdm 作 `gnss.nav_rinex`)里 "WAIT_CORE→WAIT_NAV→COLLECT 15 satellites with healthy ephemeris→TRACKING→ACTIVATE"(yaw σ 0.6°),
2386 条 GNSS 消息(2274 条 range_meas + 星历)全部路由(`gnss_msgs_routed`),无解码拒收。

**映射**(`gnsscomm.py`;与 `tools/gnss_bag_export.py`(消息→RINEX)互为逆,单测里做往返):

| 字段 | 取值 |
|---|---|
| 信号选择 | RTKLIB-EX 的前 NFREQ=3 个频点槽(GPS L1/L2/L5、GLO G1/G2/G3、GAL E1/E5b/E5a/E6/E5ab、BDS B1I/B2/B2a/B3/B1C/B2ab),槽内按 `codepris` 每个观测量(C/L/D/S)独立取最高优先级的码;再按 `signal_table.hpp` 过滤(GPS L1/L2/L5、GAL E1/E5b/E5a、BDS B1I/B2、GLO G1/G2),一码一信号,主频点(L1 类)在前 |
| 卫星 | gnss_comm 编号 GPS 1–32、GLO 33–59、GAL 60–97、BDS 98–160;**QZSS 与 SBAS 无编号,丢弃**(0707 的 12 分钟包:每历元约 1.3 颗 J、1.8 颗 S);GLONASS 频道号取自头的 `GLONASS SLOT / FRQ #` |
| freqs | 槽的标称频率 [Hz];GLO = 1602e6 + k·0.5625e6(G1)、1246e6 + k·0.4375e6(G2) |
| CN0 / psr / cp / dopp | S [dB-Hz] / C [m] / L [周] / D [Hz],RINEX 符号即 gnss_comm 符号(D 对接近的星为正:L 在增、D 为负的行里互相印证) |
| status | bit0 伪距有效(C≠0)、bit1 载波有效(L≠0)、bit2 半周已解(L≠0 且 RINEX LLI 无 bit1)、bit3 恒 0 |
| LLI | RINEX LLI 的 bit0(失锁)| 半周标志置 2 的 bit1;无相位时 0 |
| code | RTKLIB-EX 的观测码编号(所选伪距码);BDS B1I 写 47(ublox_driver 对 B1I 的编号,M3DGR bag 里如此) |
| std 字段 | 0(RINEX 没有接收机 std;0 = "无",`quality.cpp` 的 `StdUnitJudge` 保持 Unknown,噪声模型的底噪生效) |
| 历元时间 | RINEX 历元(GPS 时间系统;BDT/GLO/UTC 头会换算)→ (week, tow);每颗卫星同一 `time` |
| 星历 | brdm(多 GNSS 合并)→ `GnssEphemMsg`(G/E/C)与 `GnssGloEphemMsg`(R);`toe/toc/ttr` = GPST,`week` = toe 的 GPS 周,`toe_tow` = toe 的 GPST 周内秒,`ura` = RINEX SV 精度 [m]("metres" 语义,A16),`tgd0/tgd1` 照 RINEX(GPS TGD;GAL BGD E5a/E1、E5b/E1;BDS TGD1、TGD2),BDS 的 `iodc` = AODC;GLO:`tau_n` = −(RINEX 钟差),`iode` = 莫斯科日 15 分钟槽号,`delta_tau_n = 0`、`ura = 0`(RINEX 3.0x 没有)|
| 星历投送 | toe 在 [bag 起点 − 2 h, 终点 + 2 h] 内的记录按 (卫星, toe, iode) 去重;记录时间 = max(ttr 对应的 UTC, bag 起点 + 1 ns),即序列开始时已有的星历在起点一并给出(接收机在录制前已在运行)|
| iono_params | brdm 头的 GPSA/GPSB(8 个数),`StampedFloat64Array`,起点一条 |
| 不写的话题 | `receiver_pvt`、`receiver_lla`、`time_pulse_info`(RINEX 里没有;L-PVT 臂因此不可用)|

**不同序列的 RINEX 来源**:五条序列的 mini 接收机文件头都写 RTKCONV 2.4.3 b34、RINEX 3.03、10 Hz,没有 3.04(0707 的文件头另写 `format: RTCM 3`);`ROVE_03` 布局的 0718/0719 比 `ROVE_02` 布局的 0702–0707 星多(每历元主频点中位 26 / 31 对 15 / 22 / 18);
LLI 只在首历元与失锁处出现(0707 的 LLI 比例 5e-4)。

### 真值(`gt_convert.py`)

`.ref` 是 NovAtel Inertial Explorer 紧耦合 GNSS/INS **平滑**解,1 Hz,GPST(Week + GPSTime),WGS84 位置、姿态(航向自北顺时针、俯仰抬头为正、横滚右侧向下为正)、位置协方差、Q、AmbStatus、前后向分离量。
* **参考点**:同一条轨迹被"搬"到平台的不同点上。`Mini_GroundTruth.ref`(0718/0719)与 `ROVE_<mini 编号>_GroundTruth.ref`(0702–0707)是 **mini 的 GNSS 天线 Ant00 的相位中心**(头 `IMU->Secondary Sensor Lever Arms`:
  相对 100C 的 IMU,(0.000, −0.112, 0.528) 或 (0.124, −0.128, 0.518),x 右 y 前 z 上);`I300_GroundTruth.ref` 是 mini 的 IMU(HGuide I300);`100C_GroundTruth.ref` 是 100C IMU。
* **杆臂**:Ant00 − I300 = (0, −0.120, +0.118) m(载体系 x 右 y 前 z 上)= I300 系 (−0.118, 0, +0.120)。数据核对:两个文件的位置差经 GT 姿态旋转到载体系,均值与头里的杆臂差一致,标准差 **0.4 mm**
  (翻转横滚或俯仰符号后标准差升到 4.4 mm / 5.7 mm,所以姿态约定同时被确认:R_nb = Rz(−航向)·Rx(俯仰)·Ry(横滚))。
* **姿态输出**:`gt_clean.tum` 的四元数是 **I300 机体系**(x 下、y 右、z 后)到 ENU 的旋转,位置是**天线点**;`gt_i300.tum` 是同一轨迹在 I300 点上(= 天线点 − R·l_I300,与 `I300_GroundTruth.ref` 的位置在 1 mm 内)。
  ENU 原点 = 第一个真值历元;时间 = GPST − 18 s(瘦身包的戳约定)。
* **时基**:`.ref` 的历元是 GPST,瘦身包的戳是同一时刻的 UTC,所以 `gt_timebase.yaml` 写 `method: "gpst"`、平移 0。端到端核对见上节(滞后 −0.027 s)。
* **覆盖与质量**:真值只覆盖序列的一段(0707:传感器 07:54:50–08:29:40 UTC+18 s,真值 07:57:21–08:26:22 GPST,晚开始 2.5 分钟);Q = 1(固定)、2、3(浮点)、4、5(伪距差分/单点,σ ~1 m);
  0707 的 3D σ 中位 0.24 m、p95 1.9 m,约 76 % 是浮点解。`trust_mask.csv` 的默认规则(`--max-sigma-3d 0.5 --max-q 3`,前后各加宽 2 s,gt_audit 的做法)在 0707 上剔 38 % 的历元——
  这个阈值是**待主循环裁定的提议**,不是冻结值;逐历元质量在 `gt_quality.csv`,可用别的阈值重写。
* `.ref` 的陷阱:表头有两个 `GPSTime` 列(第二个是 HMS 字符串);`iFlag` 是字符串(`ZUPT` 等);数值格可为 `E228!`(无前一位置,首行的前后向分离量);文件末尾有错误说明行。

## MARS-LVIG HKisland_GNSS03

`https://hf-mirror.com/datasets/swd123456/HKisland_GNSS03_Dataset_from_MARS-LVIG/resolve/main/HKisland_GNSS03.bag`:302 到签名的 CDN 链接(约 1 h 有效,下载器遇 4xx 自动重解析),首跳响应头的
`x-linked-etag` 就是文件的 **sha256**(LFS oid `8ce44d1d…c317`)、`x-linked-size` = 10 040 751 440 B;收到的字节的 sha256 与之**一致**(记录在 `HKisland_GNSS03.fetch.json`);原包在瘦身包验收后已删(`mars/RAW_REMOVED.txt`;`fetch_mars.py fetch` 重取约 40 s,哈希可再核对)。
bag 跨 391 s(UTC 2023-10-24 07:35:49–07:42:20),所以 BRDC = `BRDC00IGS_R_20232970000_01D_MN.rnx`(BKG,`brdc/`,gz 与解压后的 sha256 都记录)。
瘦身(单遍,记录时间与顺序不变):保留 `/livox/lidar`(CustomMsg)、`/livox/imu`、全部 `/ublox_driver/*`、DJI RTK 话题(`rtk_position|velocity|yaw`、`rtk_info_position|yaw`、`rtk_connection_status`、
`time_sync_*`)按**字节原样**;`/left_camera/image/compressed` 缩到 612×512(`camera_MARS_LVIG.yaml`:2448×2048、`scale: 0.25`,同样让宿主不再缩放)。
真值:`/dji_osdk_ros/rtk_position`(NavSatFix,5 Hz,只有位置,时间戳在主机钟上,与 GNSS 时间只对齐到约 0.1 s)→ TUM;`rtk_info_position` 非最佳状态的历元 ±2 s 进掩码;时基 `none`(0,标记"未测")
直到用开发集的多普勒速度互相关测出后写入(`gt_convert.py --shift-s S --method xcorr`,A54 的 20 ms 规则)。
MARS 的已知陷阱(调研):bag 内星历晚约 11 s 才到、5 颗星始终没有星历(→ `gnss.nav_rinex` 注入 BRDC,A1/A38/A45);约 2.1 % 的载波有效位置位但 cp = 0(调研值;瘦身包里实测主频点 1.46 %;核心要求 cp ≠ 0 才算有效相位,`SignalObs::cpValid`)。
成品 `mars/HKisland_GNSS03_slim.bag`:6.24 GB,sha256 `9b9bfe0fd4d3…`(`HKisland_GNSS03.build.json`),3911 个 GNSS 历元(10 Hz,无断档)、主频点 22–32 颗(中位 28)、C/N0 中位 32 dB-Hz、
论文 RAIM 口径(C/N0 ≥ 35、仰角 ≥ 15°)中位 7 颗、只用 bag 内星历的健康星历占比 87.4 %(加 BRDC 96.3 %)、真值 1955 个历元(5 Hz,RTK 状态全为 50)。
整条回放过(U 与 T 两臂,约 391 s):T 臂 COLLECT → TRACKING → ACTIVATE 正常(21 颗健康星历、yaw σ 0.42°,BRDC 经 `gnss.nav_rinex` 注入)。

## 成品包一览与验收(SmartPNT-MSF,2026-10-03,远端 `/data2/fl2g/smartpnt/<seq>/`)

| 序列 | 类 | 时长 s | bag | 扫描 / 相机帧 / IMU / GNSS 历元 | 主频卫星/历元 | C/N0 中位 | RAIM 口径 | 真值历元 / σ3d 中位 / 掩码剔除 | sha256 前 12 位 |
|---|---|---|---|---|---|---|---|---|---|
| `Data01_20240707_100C_CAR_complex` | 开发 | 2090.1 | 12.74 GB | 20601 / 20422 / 418036 / 20304 | 15 | 38 | 12 | 1742 / 0.244 m / 664 | `5ba2c8934628` |
| `Data02_20240718_100C_CAR_complex` | 开发 | 2213.9 | 10.47 GB | 21804 / 21626 / 442461 / 21845 | 26 | 37 | 20 | 1895 / 0.023 m / 0 | `993b7b5d354a` |
| `Data01_20240719_100C_CAR_opensky` | 评测 | 3993.6 | 17.32 GB | 39474 / 39652 / 798778 / 39936 | 31 | 38 | 25 | 3665 / 0.012 m / 0 | `2ba08664c140` |
| `Data00_20240703_100C_CAR_complex` | 评测 | 2363.7 | 15.44 GB | 23316 / 23426 / 472773 / 23636 | 22 | 36 | 14 | 1519 / 0.412 m / 593 | `bb1922998ce3` |
| `Data00_20240706_100C_CAR_complex` | 评测 | 3271.2 | 21.89 GB | 32291 / 32439 / 653942 / 32712 | 18 | 37 | 13 | 2559 / 0.140 m / 431 | `46b9b94a3259` |

(RAIM 口径 = 1 Hz 子样本里 C/N0 ≥ 35 dB-Hz 且仰角 ≥ 15° 的主频点个数的中位;掩码 = `trust_mask.csv` 默认规则 σ3d ≤ 0.5 m 且 Q ≤ 3 之外的 1 Hz 历元数,仍是待裁定的提议。
评测集三条只做了取数、格式层面统计与真值格式转换,没有回放、没有打分。)

验收(每条序列 bag 建成后都做;结果与 bag 放在一起):
* `verify_bag.py`:五条全部 21/21 通过(gnss_comm 四个连接与 Grass01 的逐字节一致;IMU 200 Hz 网格;相机 640×512、10 Hz、到达 +50 ms;LiDAR 逐点 time 为 µs 且从 0 起;
  GNSS 记录时间 − 历元 UTC = 80 ms 逐历元不差;各流同钟)。
* `imr_check.py`(IMU 对平台自己的 `I300.imr`):五条全部 bag 样本都在 0.36 µs 内与 IMR 的记录时间对上(两边样本数相差 1–13 个),陀螺 bag = IMR × π/180 到 8e-12,加速度计增量相同;
  0706 跨 GPST 午夜,同样逐位对上。
* `calib_check.py gravity`(LiDAR→IMU 倾角,静止段,用同一个 `extrinsic_R`):五条 1.4–2.0°(0703、0706、0707 是 `ROVE_02` 文件布局,0718、0719 是 `ROVE_03` 布局;每条两个静止窗)。
* 开发集 Data02(`ROVE_03` 布局)的 LIO 行驶方向:U 臂回放 300 s,估计机体系里的行驶方向 (−0.002, −0.030, −1.000),真值给出 (−0.003, −0.005, −1.000),差 1.4°——`extrinsic_R` 的偏航对这条序列同样成立。
* 回放冒烟(经 `run_matrix --root /data2/fl2g --lidar smartpnt_mini`,即登记表与工程根布局的官方路径):0707(240 s)与 Data02(300 s)的 U、T 两臂都 `complete=1`;T 臂走到 ACTIVATE
  (0707:COLLECT 开始时 10 颗健康星历,ACTIVATE 时 15 颗、yaw σ 0.60°;Data02:31 颗,25 颗、yaw σ 0.15°、`path=spp_window`)。
* GNSS 时基:Data02 的 Doppler 水平速度对真值水平速度互相关 0.986,最佳滞后 0.0 s(分辨率 0.1 s;0707 的接收机文件质量差,相关只有 0.40,不能用来定时基);GPST→UTC 的 18 s 与五条序列同一个常数 `shift_ns` 互相印证。

**一个观察,原因未查清(不影响评测口径)**:开发集上单点定位(SPP,Klobuchar 电离层)对真值天线位置有一个**近似常量的绝对偏差**:0707 为 ENU (0.2, 7.4, 2.8) m,Data02 为 (0.9, 10.9, 9.9) m,
Data02 的散布很小(水平 p95 12.8 m,中位 11.2 m)。0707 上按系统拆开(只用 GPS、只用 BDS、只用 GAL、G+E)与改 C/N0、仰角门限,偏差都在 7–8.1 m 北向、东向 ≤ 0.5 m,拟合残差 RMS ≤ 0.7 m;
关掉电离层模型只改变天向(12.5 m → 3.2 m),北向不变。偏差对各星座一致,所以不是转换、时标或某个系统的星历问题;候选解释:下午(当地 16 点)太阳活动高年的电离层梯度残差加街道遮挡造成的南北向几何弱,
或真值相对基站坐标的绝对偏差。A80 的帧求解(W→ENU 的平移也在最小二乘里)与 Umeyama 对齐都不依赖绝对位置,所以先不处理;要定性可用 IGS 的 GIM 电离层图重算(需取 IONEX),或用 BASE_01 做伪距差分(`--with-base`)。

## 配置

* `config/smartpnt_mini.yaml`:宿主 = FAST-LIVO2 上游对 16 线旋转 LiDAR 的取值(`NTU_VIRAL.yaml`:`point_filter_num 3`、`blind 1.0`、`beam_err 0.01`)+ 上游 IMU/相机条目(未调);
  `lidar_type: 2`(velodyne)、`scan_line: 16`、外参与时间偏移见上;`gnss:` 树沿用 `m3dgr_avia.yaml` 的结构与 A58 的标定值,只改 `frames`(杆臂 [−0.118, 0, 0.120] 在 IMU 系,σ 0.03)与 `imu_bridge`(I300 实测)。
  **未做**:GNSS 噪声/掩码在 SmartPNT 上的重标定(A80 §四:开发集定参,之后冻结)、`gnss.time.calib_file`(A18)。
* `config/mars_lvig_gnss.yaml`:宿主 = 上游 `MARS_LVIG.yaml` 原样 + `camera_MARS_LVIG.yaml`;`gnss:` 树同上,杆臂取 0(数据集不公布天线杆臂,σ 0.3 m,只作管线打通)。
* 内存:`map_sliding_en: false`(宿主上游默认);单个回放的内存随时长单调增长(README "内存"一节),66 分钟的评测序列的峰值要在开发集的 35 分钟序列上先量,必要时再议。

## 发现的数据陷阱与处理(汇总)

| 陷阱 | 处理 |
|---|---|
| bag 头日期错(2014-07-xx),文件名时刻(PC 钟)与戳差 1–2 分钟 | 戳规则(见"时间"):时刻 = GPST 时刻,日期取自本序列自己的 GPS 数据;文件名不用。IMU 与 `I300.imr` 逐样本核对(`imr_check.py`):0707 的 392 174 个样本全部在 0.36 µs 内对上 |
| `/imu_raw` 是增量,不是速率;IMR 里陀螺单位是度,bag 里是弧度 | 存速率(×200);核对 bag = IMR × π/180 到 1e-11,加速度计两者相同 |
| 录制被截断:最后一个 LiDAR 文件(0707 的 `…_34.bag`)没有索引,rosbags 拒读 | `bagmerge.open_reader` 用 ROS python 的 `reindex`(在下载副本上就地)重建索引,丢不完整的尾块(371 帧 / 37 s 保留);`build.json` 记 `reindexed_source_bags`,并发 warning |
| LiDAR `time` 为秒(宿主 velodyne handler 要微秒),且可以参考到帧首或帧尾;扫描周期 0.10086 s 而非 0.1 s | 按点绝对时刻 = 头戳 + `time` 统一,头戳改成首点、`time` 改成 µs;单位逐帧核对,参考方式计数写进 `build.json`(`source_time_reference_scans`) |
| 各传感器起止不同:IMU 比相机早 10 s 以上;相机比 LiDAR 少一个文件;0718 的最后一个相机文件只有 4 KB | 各流自行结束;宿主在 LIVO 模式下由图像驱动,最后一张图之后的 LiDAR/IMU 不被使用(记在 build.json 的各话题首尾时刻) |
| 序列之间目录与文件名不同(`02_mini/ROVE_02` 对 `03_mini/ROVE_03`;`ROVE_02_GroundTruth.ref` 对 `Mini_GroundTruth.ref`;基站文件名与个数) | `portal.Layout` 按角色识别,多于一个候选就报错不猜 |
| mini 接收机文件是 RTKCONV 转的 RINEX 3.03(含 SBAS、QZSS;文件布局在序列之间不同,星数也不同) | 一个读取器读全部;SBAS/QZSS 无 gnss_comm 编号,丢弃(与 F9P 的 ublox_driver 输入同构);GLONASS 频道取自头 |
| brdm 里 GLONASS 记录的 "message frame time" 的周内秒不遵守 RINEX 的定义(周日的记录给出第 4 天) | 只取日内秒,取离 toe 最近的那个时刻作 `ttr` |
| 未来的星历:toe 在数据末之后的记录,其发送时刻晚于数据末 | 只投送记录时间 ≤ 数据末的星历(曾让 12 分钟的探针 bag 的时间跨度变成 2.9 小时) |
| 真值只覆盖序列的一段(0707 晚开始 2.5 分钟),1 Hz,浮点解居多;`.ref` 表头重名列、`iFlag` 为字符串、`E228!` 数值格、尾部说明行 | `gt_convert.py` 按列名读;输出同时给 `gt_quality.csv`(Q、σ、前后向分离、速度、卫星数);掩码规则是可调的提议 |
| 真值的参考点随文件不同,杆臂在文件头里 | 天线点 Ant00 与 I300 点的位置差经姿态旋转,与头里杆臂差一致到 0.4 mm;并与 `06_Description/README.xml` 的 `to ROVE_NN: … (Septentrio, Ant00)` 交叉核对,记进 build.json |
| 两份相机标定:Kalibr 相机-IMU 链(`/cam00` + `/imu_raw`)与 `mini_cam00.yaml`(VINS,话题 `/cam1`,内参与外参对不上) | 只用 Kalibr 链 |
| `mini_LiDAR.yaml` 的 LiDAR-IMU 外参有多组候选,没写属于哪颗 IMU | 数据决定:倾角(重力对地面法向)、偏航(行驶方向,LIO 机体系对真值)、时钟(有效特征数),见"LiDAR → IMU 外参" |
| 门户的尺寸是四舍五入的字符串,无校验和;ETag 里的尺寸只对 nginx 成立(Apache、Hub 的 ETag 不是尺寸) | 权威尺寸 = 服务器 Content-Length,与门户舍入值、nginx ETag 尺寸核对;sha256 记收到的字节 |
| 镜像里的 scipy 与 numpy 不兼容(`np.typeDict`),`evo` 因此也导入不了 | 本目录的工具只用 numpy;评测里的 evo 仍走 `tools/eval/setup_evo_env.sh` |
| MARS:Hub 链接是 1 小时有效的签名 CDN 链接;bag 内星历晚到、五颗星缺星历;约 1.5 % 的载波有效位置位但 cp = 0;DJI RTK 只有位置、戳在主机钟上 | 下载器遇 4xx 重解析;Hub 的 LFS oid 是官方 sha256,收到的字节与之一致;BRDC 作 `gnss.nav_rinex`(只用 bag 内星历:87.4 % 的卫星历元有健康星历,加 BRDC:96.3 %);`cpValid` 要求 cp ≠ 0;真值时基待测 |
| 源戳是"自 1980-01-06 的 GPST 秒数"被当 Unix 秒写进头(所以 2024 年读成 2014 年),不是一天内的时刻 | 五条序列算出同一个 `shift_ns` = 315 964 782 s(= 315 964 800 − 18);戳连续,0706 跨 GPST 午夜也不回绕;按 RINEX 首历元选 GPST 日只作一致性核对 |
| Data02 的 IMU 开头有一个 1.745 s 的断档(52 个样本之后),平台自己的 `I300.imr` 里同样存在,在首帧扫描前约 2 分钟 | 不动源数据;`verify_bag.py` 对"在首个相机/LiDAR 戳之前结束的断档"只报告不判失败;回放从首帧 LiDAR 起,不受影响 |
| Data02 的 mini 接收机 RINEX 开头只有 22 个历元,随后 641 s 的空洞(接着才是连续数据,中间另有几处 0.2–5.8 s 的小断档) | 空洞在 IMU 起点(08:05:00 UTC)之前;GNSS 历元流按记录时间随回放送入,断档只表现为这一段没有历元;`gnss_check stats` 记录了断档 |
| 真值相对单点定位有近似常量的绝对偏差(0707:北 7.4 m、天 2.8 m;Data02:北 10.9 m、天 9.9 m),各星座一致 | 见"成品包一览与验收"的观察:不影响帧求解与 Umeyama 对齐;原因未查清(候选:电离层梯度残差与街道遮挡的南北向弱几何、或真值相对基站坐标的绝对偏差) |
| 同一序列被两个构建队列同时处理,互相截断对方的 bag 与原包(一次 0706) | 两个进程都杀掉、删掉该序列目录后重建;现在 `build_slim` 用 `<seq>/.build.lock`(flock)拒绝第二个构建,已建成的序列再次排到会跳过 |
| `/data2` 与别的任务共用,成品 bag 合计约 78 GB(一条 10–22 GB) | 先建先验收;MARS 原包(10 GB,可 40 s 重取)与已被全长包取代的 0707 探针包(合计约 6.5 GB)已删,删除记录在 `mars/RAW_REMOVED.txt`、`smartpnt/_probe/REMOVED.txt` |

## 单测

**只在远端镜像里跑**(本机不跑:端到端单测会重编码图像、拖满 CPU;远端 `remote_tool.sh` 持构建锁、容器内 nice 19):

```bash
FL2G_REMOTE=user@<host> FL2G_PORT=<端口> tools/dev/remote_tool.sh <树> <名> -- python3 -m unittest discover -s tools/datasets/tests -v
```
(`tools/eval/tests` 里的 `test_eval_aruco` 两条用例在镜像里因 scipy/numpy 不兼容失败,与本目录无关,主线同样如此。)
覆盖:RINEX 头与历元、时间系统换算、槽位与码优先级、信号映射(含与 `gnss_bag_export.py` 的往返)、定义布局核对(合成定义的 md5 = 已知 gnss_comm md5)、
消息打包尺寸、星历字段、Imu/CompressedImage/PointCloud2 的字节往返、LiDAR 帧首/帧尾参考的等价性、单位检测与重排、图像缩小、戳规则(含午夜与错日)、归并顺序与写读回、
门户布局识别(两种序列风格,含歧义拒绝)、范围下载(本地 Range 服务器:分段、sha256、尺寸/ETag 不符、签名重解析)、`.ref` 解析与姿态约定、杆臂识别与 GT 转换、掩码加宽、MARS 真值、工程根链接、成品 bag 验收(端到端单测里对合成序列的 bag 跑 `verify_bag.py`,并核对错误预期会被拒)、倾角校正(`test_extrinsics_tilt`)、时钟核对的纯函数(`test_clock_checks`:`time_check` 的滞后搜索/分窗/掩码与 `cam_imu_toff` 的偏移扫描,合成信号、已知答案)、`gnss_check.availability`(`test_gnss_availability`)、`gt_stats`(`test_gt_stats`:合成真值目录的路程/覆盖/掩码/质量计数)、`imu_gt_check`(`test_imu_gt_check`:合成偏航率的斜率/标度误差/滞后符号与大小/静止窗/重力)。
`gnss_time_calib` 的 A80 选项在 `tools/tests/test_time_calib.py::A80Options`。
依赖真实数据的单测(参考 bag 的定义核对)在没有 `/r/data/m3dgr/filtered/Grass01.bag` 的机器上跳过。
