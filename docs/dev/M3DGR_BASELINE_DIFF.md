> Historical research record (through 2026-10-03). Paths are illustrative; archived runs are not bundled. For current status and executable entry points see [the repository README](../../README.md). Current release docs/LICENSING.md supersedes historical license notes. This record does not authorize new experiments or assert current release validation.

# M3DGR 官方 FAST-LIVO2 基线 与 0d2c034 的比对(AMENDMENTS A6;WP-f,2026-09-25;P2b 按 A27/A40 补 U 臂定义与脚注)

**目的**:A6 要求实现前核对 M3DGR 官方基线 `baseline_systems/Fast_LIVO2_M3DGR` 自带的 fast_livo2 源码是否改过估计器;若改过,U 臂以哪一份为准要在报告里写明。

**结论先行**:
1. 官方基线的 fast_livo2 源码 **没有 M3DGR 作者的任何修改**——`src/*.cpp`、`include/*.h`、`include/utils/*.h` 共 20 个文件的 git blob id 与 hku-mars/FAST-LIVO2 **首个公开发布提交 986b4b1(2025-01-23,"[Release] code is now available!")逐字节相同**(方法见 §3)。基线只是「v1.0 发布版 + M3DGR 自己的 config/launch」。
2. 因此基线与 0d2c034 的全部差异 = 上游自己在 986b4b1→0d2c034 之间的 18 个提交(bug 修复、LO 模式、更多雷达支持、去 livox_ros_driver 依赖、输出格式)。其中**影响 M3DGR Avia+IMU+相机(LIVO 模式)估计结果的只有一处**:VIO 图像边界 `border`(8ad8f62,"[Fix] fix bugs in issues #65, #69, #74, and #77");另有两处只在数据异常时起作用(IMU 0.2 s 跳变丢弃、空点云保护),其余全部是 LO 模式、非 Avia 雷达、输出/保存路径。
3. **U 臂以 0d2c034 为准**(A6 默认),**加 A27 的越界读保护**(P2b 起;U = R,报告写「U 为 0d2c034 + 越界读修复」)。理由:基线 = 上游旧版,不是「M3DGR 调优版」;0d2c034 的差异是上游自己的修复。评测表里 FAST-LIVGO/M3DGR 论文数字对应的是 986b4b1 代码,与我们 U 臂的 border 不同,这一点在报告里注明即可;不另跑 986b4b1(它不含 `include/livox_ros_driver/*.h`,要装 livox_ros_driver 包才能编)。

## 1 材料

| 项 | 位置 / 值 |
|---|---|
| 官方基线源码 | `https://github.com/sjtuyinjie/M3DGR/tree/main/baseline_systems/Fast_LIVO2_M3DGR`(该目录最后一次改动:提交 2bc4d82,2025-07-24,"Update README.md") |
| 远端副本 | `/work/fastlivo2_gnss/ref/m3dgr_fl2/`(README.md、`src/fast_livo2/{src,include,include/utils,config,launch,CMakeLists.txt,package.xml,README.md}`,`src/CMakeLists.txt`;每文件 sha256 在 `FILES.sha256`) |
| 参考 | `/work/fastlivo2_gnss/ws_upstream/src/FAST-LIVO2` = hku-mars/FAST-LIVO2 @0d2c034 |
| 比对命令 | `diff -ru -w <ws_upstream>/{src,include,CMakeLists.txt,package.xml} <ref>/src/fast_livo2/...`;blob 匹配见 §3 |

M3DGR 基线 README(远端 `ref/m3dgr_fl2/README.md`):`git sparse-checkout set --no-cone baseline_systems/Fast_LIVO2_M3DGR && catkin_make`,`roslaunch fast_livo mapping_m3dgr_avia.launch`(mid360 另有 launch),`rosbag play Dynamic01.bag`。

## 2 逐文件差异(基线 相对 0d2c034)

| 文件 | 差异(行数 `diff -u` 计) | 对 M3DGR Avia LIVO 模式的影响 |
|---|---|---|
| `src/vio.cpp` | 11 | **有**:`border = (patch_size_half + 2) * 8`(基线)vs `(patch_size_half + 1) * (1 << patch_pyrimid_level)`(0d2c034,8ad8f62 修复)。M3DGR 配置 patch_size 8、patch_pyrimid_level 4 ⇒ 基线 48 px、0d2c034 80 px。视觉点选取的图像边界不同 ⇒ 两份代码的 VIO 结果不逐位相同。 |
| `src/LIVMapper.cpp` | 489 | 估计器数学零差异。输入侧两处只在异常数据时起作用:① 基线启用 `if (timestamp > last_timestamp_imu + 0.2) { warn; return; }`(丢掉跳变后的第一条 IMU;0d2c034 注释掉)——只在 IMU 断流 >0.2 s 时触发(Outdoor01 本地副本 262.5 s 处的 0.5 s 缺口会触发一次,A3);② 0d2c034 在 `livox_pcl_cbk` 加了空点云保护(7951a2f),基线会把空帧压进缓冲。其余:去掉 `lidar_time_offset`(仅 standard_pcl_cbk,Avia 不走)、`hilti_en`、LO 模式分支、`pcd_save/type`、`image_save/*`、`RGBpointBodyLidarToIMU`、`voxel_map_pub_`、`is_pub_plane_map_`;`publish_frame_world` 改为只在 VIO 周期(img_en)调用且 `pcl_w_wait_pub` 无条件清空——在 LIVO 模式下与 0d2c034 的「LIO 周期调用但不清空、VIO 周期清空」等价(VIO 门 `pcl_w_wait_pub->empty()` 的取值序列相同);PCD/图像保存路径与文件名不同(输出侧)。 |
| `src/IMU_Processing.cpp` | 87 | **无**:只改 `Forward_without_imu`(仅 imu_en=false 的 LO 模式:0d2c034 加了去畸变(6289d03)并修正 F_x 块下标 (0,9)/(3,6)→(0,10)/(3,7));`time_list` 定义从头文件挪到 .cpp。`UndistortPcl`(LIVO 实际走的路径)逐字节相同。 |
| `src/preprocess.cpp` | 121 | **无**(Avia):`avia_handler` 相同。差异在 oust64(排序)、velodyne(`t` uint32 vs `time` float,af39d8e)、Pandar128 点结构、Robosense(7e88460)。 |
| `src/voxel_map.cpp` / `include/voxel_map.h` | 26 / 11 | 无:`mapSliding()` 返回 void→bool;`map_sliding_en: false`。 |
| `include/common_lib.h` | 22 | 无:枚举少 `ROBOSENSE`、`LO`。 |
| `include/IMU_Processing.h` | 22 | 无:`time_list` 声明式、少 `lidar_type`(仅 LO 用)、少 `<fstream>`。 |
| `include/LIVMapper.h` | 41 | 无:成员增删随上述 .cpp。 |
| `include/preprocess.h` | 60 | 无(Avia):velodyne/Pandar128/robosense 点结构。 |
| `include/livox_ros_driver/*.h` | 基线无 | 基线依赖外部 `livox_ros_driver` 包(package.xml/CMakeLists 加依赖);0d2c034 自带头文件(1666b09)。消息定义与 md5 相同(bag 中 `livox_ros_driver/CustomMsg [e4d6829b…]`)。 |
| `CMakeLists.txt` / `package.xml` | 29 / 18 | 构建:`livox_ros_driver` 依赖、`Sophus::Sophus` 目标 vs `libSophus.so`。 |
| `src/frame.cpp`、`visual_point.cpp`、`main.cpp`、`include/{feature,frame,vio,visual_point}.h`、`utils/*` | 0 | 相同。 |

官方 M3DGR 配置(`config/m3dgr_avia.yaml`、`config/camera_pinhole_m3dgr.yaml`、`launch/mapping_m3dgr_avia.launch`)已按 A6 原样搬入本仓库 `config/m3dgr_avia.yaml`、`config/m3dgr_camera_pinhole.yaml`(只追加 ref-R 的 `common/img_compressed`、`publish/use_sensor_time`、`replay`、`output`、`debug` 键,并把 `evo/pose_output_en` 从 false 改为 true 以写出上游 `Log/result/<seq>.txt`——输出侧开关,不是估计器参数;LIVO 参数一个不改)。官方 launch 用 `image_transport republish compressed→raw`;本仓库的节点直接订阅 `/camera/color/image_raw/compressed` 并在进程内解码(DESIGN §13),其像素级等价由 I2 审计核对。

## 3 blob 匹配(基线 = 986b4b1 的证明)

远端对 20 个文件 `git hash-object`,与 GitHub API `git/trees/<ref>?recursive=1` 的 blob id 比对:

| 上游提交 | 与基线逐字节相同的文件数 | 不同的文件 |
|---|---|---|
| **986b4b1**(2025-01-23 Release) | **20 / 20** | — |
| 8ad8f62(2025-02-09) | 16 / 20 | LIVMapper.cpp、vio.cpp、voxel_map.cpp、voxel_map.h |
| 7951a2f(2025-02-15) | 16 / 20 | 同上 |
| 0d2c034 | 10 / 20 | 表 §2 |

986b4b1→0d2c034 共 18 个上游提交(`compare/986b4b1...0d2c034`):8ad8f62 修 #65/#69/#74/#77(含 border)、7951a2f 空点云、0b79037 LO 模式、6be251b PCD 保存、162f843 MARS-LVIG、6289d03 LO 去畸变、998eada/137b660 NTU-VIRAL、1666b09 去 livox_ros_driver 依赖、c96f4a0 FAST-Calib、af39d8e Ouster/Velodyne 时间戳、46ad5e7 HILTI22、7e88460 Robosense/Pandar128、8aac9d5/9819553/0d2c034 文档、6fb5651 LiDAR-visual BA 输出、b145cbd 四元数分量顺序(仅输出)。

## 4 对评测的含义

- U 臂(及全部 GNSS 臂的宿主)= 0d2c034 + A27 越界读保护(tag `ref-R`),与 A6 默认一致;报告脚注(A40):「M3DGR/FAST-LIVGO 论文的 FAST-LIVO2 数字对应发布版 986b4b1,其 VIO 图像边界为 48 px(本文 80 px);本文 U 另含 `VIOManager::updateState` 的越界读修复,其余估计器代码相同」。
- (P2b 核对)986b4b1 的 `VIOManager::updateState` 与 0d2c034 **逐字节相同**(远端 `ref/m3dgr_fl2` 与 `ws_upstream` 按函数体 diff 无差异),即同样有 A27 的越界读;它的 border 48 px 比 80 px 更靠边,放进来的边缘点更多(模板在最粗层伸出 5·32 = 160 px),越界更容易触发。推断(未在 986b4b1 上实测):已发表的该基线数字来自一份结果随堆内容变化的代码,与 0d2c034 无保护时同种子两次运行在第 649 周期分岔(REF_R_GATES §9)同一机理。
- 不需要「官方源码另跑一次作参照」:官方源码就是上游旧版,参照意义等于「0d2c034 回退一个 VIO 边界修复」。若审稿要求,可在 0d2c034 上把 `border` 改回 `(patch_size_half + 2) * 8` 单独跑 U′ 作脚注,这不属本轮。
- 基线的 IMU 0.2 s 跳变丢弃对 Outdoor01 本地副本(A3 的 0.5 s 缺口)会丢一条 IMU;0d2c034 不丢。两者在该缺口后的行为都不可与官方数字对齐,报告已按 A3 要求注明缺口。

## 5 MID-360 配置与源码(A76,2026-10-02)

官方基线给了第二套配置 `config/m3dgr_mid360.yaml`(sjtuyinjie/M3DGR,文件 blob `347476bc`,sha256 `2b45dbee…`,最后一次改动是提交 `e87e273` 2025-07-24)和 `launch/mapping_m3dgr_mid360.launch`。
对 `m3dgr_avia.yaml` 的逐键差别只有六个键:话题 `/livox/mid360/{lidar,imu}`、`extrinsic_T` `[-0.011, -0.02329, 0.04412]`(`extrinsic_R` 仍是单位阵)、`Rcl/Pcl`(标定表 1.4「mid360 → camera」)、
`scan_line` 4(Avia 6)。`lidar_type` 仍是 1(注释写着 Avia,即 Livox CustomMsg 处理器),`blind` 0.8、`point_filter_num` 1、噪声与迭代参数全同。launch 与 Avia 的只差载入哪份 yaml。
**源码没有为 MID-360 改过**:官方目录里只有一份 fast_livo2 源码(§3 已证与 986b4b1 逐字节相同),`avia_handler` 就是 MID-360 的处理路径(`line < scan_line` 取点,`offset_time` 按毫秒换算);
基线自带的 `livox_ros_driver` 包也只有 v1 的 `CustomMsg`。包里 `/livox/mid360/lidar` 的类型名是 `livox_ros_driver2/CustomMsg`,md5 `e4d6829b…` 与 v1 相同(I2 审计 S1 对 Avia 话题记过;A76 在 Outdoor01 原包的连接记录里对 MID-360 话题复核),
rosbag 的 `MessageInstance::instantiate<T>()` 只比 md5(`isType`),所以 `fastlivo_replay` 里 `m.instantiate<livox_ros_driver::CustomMsg>()` 不用改。本仓库 `config/m3dgr_mid360.yaml` 对官方文件的取舍见该文件头。
