> Historical research record (through 2026-10-03). Paths are illustrative; archived runs are not bundled. For current status and executable entry points see [the repository README](../../../README.md). Current release docs/LICENSING.md supersedes historical license notes. This record does not authorize new experiments or assert current release validation.

## 集成正确性审查判决(基底 `/workspace/fast_livo2_ref` @0d2c034,行号已逐条亲核)

### 核对结论(事实)
- 三个设计引用的 file:line 基本属实:DIM_STATE(common_lib.h:30)、StatesGroup 六个手写函数(:128-222)、IMU 循环 F/Q(IMU_Processing.cpp:382-401,σ²·dt²)、`last_prop_end_time` UB 读(:248,头文件 :80)、`IMUpose` 在 :281 压起点、:490 早退、:538 清空、`last_lio_update_time=prop_end_time`(:435)与 `last_prop_end_time=prop_end_time`(:470)构成不变量;LIO 信息形式全维求逆(voxel_map.cpp:468)只用 `K_1.block<DIM,6>`;VIO 只用 `block<DIM,7>`(vio.cpp:1661-1667)、回退(:1648-1681)、`cov -= G*cov`(:800);VIO 步 `m.imu` 从不填充(LIVMapper.cpp:1058-1066 注释)故传播循环 0 次;main.cpp 单线程 `spinOnce`;仅两处 `#pragma omp`(voxel_map.cpp:660 按下标写、vio.cpp:1554 float 归约);容器无指针键、无指针排序;`publish_frame_world` 无订阅者门控,:1289 的 swap 是 VIO 的门(:288)。
- 在 `ros:noetic-ros-base-focal` 镜像内实测:GCC 9.4.0;`-std=c++17` 下 `-ffp-contract` 默认 **fast**;`-march=x86-64-v3` **不支持**;rosbag `MessageInstance::isType` 接受 md5 `"*"`;`topic_tools/shape_shifter.h` 随 ros-base 提供。
- 本机复跑 D3 的 pad.cpp:扩维后左上 19×19 块在 `-march=native` 下 27/27 DIFF、通用 -O3 25/27 DIFF。**扩 DIM_STATE 与「GNSS 关闭逐字节等于上游」不可兼得**;增广块可以,条件是:所有 TU(含 vikit)`-ffp-contract=off`、同一 -march、去 MP_EN、无 LTO、UB 修复、钩子只读宿主。
- 一个三方都未写明的事实:`sync_packages` 消费的数据集合对到达时序不变(:973 只决定是否等待;:1010-1033/:988-997 只吃 ≤ t_img 的帧与样本)。实时 0.888±0.115 m 的方差只能来自启动期跨话题次序(:773/:850)、vio.cpp:1554 归约与 :248 的 UB。这判定了扰动模型的有效性:D3 start_offset 有效且贴合真实来源,D1 图像抖动有效,D2 单一虚拟时钟无效(推断:种子塌缩为同一轨迹)。

### 按设计
- **D3(集成优先)**:在本镜头下最扎实。零改动宿主 + 三钩子(公式经核对精确)、四道门 + hexfloat 轨迹 + valgrind、eval/rt 双 profile、docker 细节与本机已验证的 Dockerfile 一致。缺陷都是 minor:周期末挂点的理由不成立(GNSS 后验下一周期照样成为 VIO 先验,真实代价是地图/视觉点插入滞后一周期)、压缩图直读的通道序、承重旗标的表述。
- **D2(鲁棒性优先)**:估计器钩子与 D3 等价且桥接样本采集最干净(不依赖 IMUpose),零通道门组合最全。但三处 major:`-march=x86-64-v3` 在 GCC 9.4 上编不过且与 vikit 硬编码 `-march=native` 冲突;虚拟时钟种子模型在 sync 时序不变的事实下不会产生任何扰动;为保 A0 等价而保留 UB 读,把确定性建在未定义行为上。三处都可修(换 -march 并补 vikit、改用 start_offset/图像抖动、零初始化进 R)。
- **D1(一致性优先)**:方法层最完整,桥接语义、push-through、StatesGroup 清单、自检清单都正确;但结构上选了唯一一条不能逐位零通道的路(扩维 + 改 voxel_map/vio),且并行评测没有 Log 隔离(ROOT_DIR 烧死、evo 文件 app 模式追加)。若采纳 D1 的估计器数学,应把状态装进增广块而非 DIM_STATE,则 push-through 与 vio.cpp 改动都不再需要。

### 正确做法(合成)
1. 架构取 D2/D3:宿主 19 维不动,增广块 + 传播/条件化/联合更新三钩子;GNSS 挂点取 :254-256(D1/D2),使地图插入含 GNSS 后验、LIO 无点时 GNSS 照常。
2. 桥接取 D2 在传播循环内自记样本(或 D1 的 IMUpose 插值但截到最后一个 offset=0 标记);F_total 在 :401 后累积。
3. 确定性与门禁取 D3:R 中零初始化 :60/:80/:81、去 MP_EN、`-ffp-contract=off` 全 TU、三包同 -march(patch vikit CMake:10/:28)、hexfloat state_trace、G-确定性/零通道/旁路/无数据四门、valgrind;扰动种子用 D3 start_offset 或 D1 图像抖动(禁用 D2 虚拟时钟),并加「种子间轨迹两两不同」硬门。
4. I/O 与许可证取 D1/D2 的运行时 schema 解码(D3 的 msg_def 校验拒收策略并入),RTKLIB-EX 按 D3 编成 hidden-visibility 的 SHARED 并 `--exclude-libs`,不定义 TRACE,自写 CMake,补 showmsg/settspan/settime 桩。
5. 构建:复用本机 `fastlivo2-mid360:noetic`(docker save/load)或按 D3 Dockerfile 在远端重建;每次运行独立容器并绑定挂载 Log;远端 docker data-root 迁到 /data;全部安排在 v0.6 链结束之后。
6. 零通道判据:增广块方案要求逐字节(state_trace sha256);若任何原因回到扩维,则以「前 N 周期相对差 ≤1e-12 且增长包络不超过 U 自身编译变体的舍入分叉」+「H 空行的 GNSS 开启态与 F0 逐位相同」+ 集成等价三层替代。

[
 {
  "severity": "major",
  "designs": "D2",
  "claim": "§11.5 确定性构建旗标 `-O3 -march=x86-64-v3 -ffp-contract=off(替代 -march=native)`,原版臂与本系统同一组选项",
  "problem": "在目标镜像 `ros:noetic-ros-base-focal` 内实测:`gcc (Ubuntu 9.4.0)` 不认识 `-march=x86-64-v3`(该级别 GCC 11 才加入),构建直接失败。即便换成可用的 -march,vikit 的 `vikit_common/CMakeLists.txt:10` 硬编码 `-march=native`(:28 还带 `-fno-signed-zeros`);fast_livo 与 vikit 的 -march 不一致会让跨包传递的 Eigen 固定尺寸对象对齐要求不同(AVX2 32 B vs AVX-512 64 B),这正是本机 MID360 README 踩过的段错误。D2 只说「原版臂与本系统同一组构建选项」,没有把 vikit 纳入。",
  "fix": "三包(fast_livo、vikit_common、vikit_ros)统一 -march:要么都 `-march=native`(逐位结论只在同机同镜像成立,D3 的口径),要么都改成 GCC 9 认识的具体架构名(如 `-march=haswell`),并给 vikit CMake 打补丁去掉硬编码;把「三包 compile_commands 的 -march/-ffp-contract 一致」写进构建门禁。"
 },
 {
  "severity": "major",
  "designs": "D2",
  "claim": "§13 扰动种子=虚拟时钟延迟仿真:每周期虚拟耗时 c_k~Gamma,「消息记录时间≤虚拟时钟才可用」,以复现「实时下缓冲内容改变 sync 决策」这一不确定性来源",
  "problem": "亲自核对 `sync_packages`(LIVMapper.cpp:940-1075):LIO 分支只在 `img_capture_time > lid_newest_time || > imu_newest_time`(:973)时推迟,一旦放行,消费的数据集合与缓冲里多出多少消息无关——点云只吃 header ≤ t_img 的帧(:1010-1012),IMU 只吃 ≤ lio_time 的样本(:988-997),VIO 分支只弹一张图(:1045-1075),丢图判据只看 `last_lio_update_time`(:965)。因此在按记录时间、各话题保序的前提下,sync 层对到达时刻完全不变。真正随时序变化的数据路径只有启动期跨话题次序(imu_cbk:773 / img_cbk:850 在首帧雷达前丢样)和 OpenMP 归约(vio.cpp:1554),而单一虚拟时钟按记录时间放行不会改变跨话题次序。推断(高置信):D2 的 8 个种子会产生逐位相同的轨迹,成对统计退化为 8 份同一数据;「A0 在 Outdoor01 做 5 次实时校准」会暴露这一点但设计没把它设为硬门。",
  "fix": "改用会真正改变数据的扰动:D3 的 start_offset(直接命中启动期竞态,这恰是实时方差的主要数据来源)或 D1 的图像时间戳抖动;若坚持时序仿真,必须对各话题分别加延迟并允许启动期跨话题乱序。把「种子间 state_trace 两两不同」设为集成自检硬门。"
 },
 {
  "severity": "major",
  "designs": "D2",
  "claim": "§18(8) 为维持 A0 逐位等价,保留 `last_prop_end_time` 未初始化(IMU_Processing.h:80)这一上游缺陷",
  "problem": "IMU_Processing.cpp:248 `prop_beg_time = last_prop_end_time` 在首次 UndistortPcl 时读未初始化成员,是未定义行为;T20/T21 的 sha256 门只在「新分配堆内存恰好为 0」时碰巧通过,valgrind 必报。把确定性建立在 UB 上,与 D2 自己「静默自检未过不产出数字」的纪律矛盾。",
  "fix": "在 A0(harness-pristine)与 fork 里同时对 IMU_Processing.h:60/:80/:81 做 `= 0.0` 类内初始化(D3 的做法;零值即事实上的上游行为),并把 valgrind memcheck 审计纳入 A0 的门禁。"
 },
 {
  "severity": "major",
  "designs": "D1",
  "claim": "§2.1 DIM_STATE 19→33 追加新状态;§13 F0 与 U 只能做统计等价,§19(11) 承认不能逐比特一致",
  "problem": "亲自重跑 D3 的 pad.cpp(本机 Eigen 3.4/GCC 13):`-march=native` 下 27/27 DIFF,通用 `-O3` 下 25/27 DIFF——把 19×19 块嵌进块对角 33×33,即使 GNSS 块零相关,`F P Fᵀ+Q`、`(HᵀR⁻¹H+P⁻¹)⁻¹`、`(I−G)P` 三种核心运算的左上块都不再逐位相同(GEMM 分块/包路径与标量尾部路径的 FMA 差异)。加上 D1 对 voxel_map.cpp:466-472 / vio.cpp:1494-1501,1660-1667 的 push-through 改写与对称化,F0 与 U 从第一步起就在 1e-16 量级分叉,再经数据关联/`error<=last_error` 等离散判决放大;实现错误与舍入混沌在统计门里不可区分,失去「仪器自检」。用户既往纪律明确偏好零通道逐字节。",
  "fix": "改为 D2/D3 的增广块(宿主 19 维一字不改,克隆奇异性只出现在 S=HΣHᵀ+R,不再需要 push-through)。若坚持扩维,可行的零通道判据须分三层:(a) hexfloat state_trace 前 N(≈200)个周期 F0 与 U 相对差 ≤1e-12 且随周期按舍入速率增长,增长包络用「U 自身两种无害编译变体(-ffp-contract=off/fast)」的分叉曲线标定;(b) 现有 F0≈U 集成等价;(c) 管道门:fork 开启 GNSS 但量测行全部剔空(H 空)时必须与 F0 逐位相同。"
 },
 {
  "severity": "major",
  "designs": "D1",
  "claim": "§14(6) 精度运行与时序无关,可以在批内并行;§13 只加 D1/D2/processOnce/回放器",
  "problem": "上游把 `ROOT_DIR` 在编译期烧成源码路径(CMakeLists.txt:40),evo 文件 `Log/result/<seq>.txt` 以 app 模式追加(LIVMapper.cpp:391-397),`fout_pre/fout_out`(:188-189)、`imu.txt`(IMU_Processing.cpp:580)都是固定路径。D1 没有任何 Log 隔离方案,批内并行的多个种子/臂会把同一 evo 文件交错写坏,统计直接失效。",
  "fix": "照 D3:每次运行独立容器并把 `<src>/Log` 绑定挂载到运行目录;或在回放驱动里把 Log 根目录参数化并在 R 中同步修改(注意这属于对 R 的改动,须进 R 的门禁)。"
 },
 {
  "severity": "major",
  "designs": "D1/D2",
  "claim": "D1 §12.1、D2 §11.1:RTKLIB-EX 编成静态库直接链进 fast_livo 可执行文件",
  "problem": "rtklib.h 以无前缀导出大量 C 符号(`norm`、`dot`、`mat`、`trace`、`tracet`、`sortobs`、`showmsg`…,以及 PI/CLIGHT/MAXOBS 等宏);静态链接进主程序后这些符号进入全局命名空间,与其它共享库同名 C 符号发生插桩时是静默错误。D1/D2 只做了头文件 pimpl 隔离,没有做符号可见性隔离。",
  "fix": "照 D3:GNSS 核心编成 SHARED,`-fvisibility=hidden` 并 `-Wl,--exclude-libs,librtklib.a`,RTKLIB 符号不外泄;同时不定义 TRACE。"
 },
 {
  "severity": "minor",
  "designs": "D1",
  "claim": "§13 D2 修复:首个 LIO 步若 `last_prop_end_time` 未初始化则取 `lidar_meas.last_lio_update_time`,「恢复上游不变量」",
  "problem": "不变量 `last_prop_end_time == last_lio_update_time` 确实在每次 UndistortPcl 后成立(:435/:470),但首次调用时 `last_lio_update_time` 是 sync 在首次同步写入的首帧雷达 header(:957),IMU 初始化期间从不推进(:1037 被注释、UndistortPcl 未被调用),已陈旧数秒。取它做 prop_beg_time 使首帧 IMUpose 偏移与 `pcl_proc_cur` 的陈旧时间基准对上,首帧会被真实 IMU 轨迹去畸变;而事实上的上游(堆内存为 0)偏移约 1.7e9,所有点只经 IMUpose[0] 处理。两者都消除 UB,但 D1 的版本改变了首帧数值,说「恢复」而非「改变」不准确。",
  "fix": "要么改成 D3 式零初始化(与事实上游一致),要么在文档里明确 D2 是首帧行为改变,且 U 臂与 fork 必须共用同一版本。"
 },
 {
  "severity": "minor",
  "designs": "D1",
  "claim": "§7.2 快照在 :444 之后取 IMUpose;上游无点时残留未清空的 IMUpose「属上游缺陷,快照不受其影响」",
  "problem": "IMU_Processing.cpp:490 `if (pcl_wait_proc.points.size() < 1) return;` 早于 :538 的 `IMUpose.clear()`,下一 LIO 步 :281 再压入 offset=0 的起点后,向量前部残留上一步条目,其 offset 与本步同量级。D1 若整段拷贝 IMUpose,插值会撞上陈旧条目,并非「不受影响」。",
  "fix": "快照只取最后一个 `offset_time==0.0` 标记(:281 压入)之后的条目;或照 D2 在传播循环里自行记录桥接样本,不依赖 IMUpose。"
 },
 {
  "severity": "minor",
  "designs": "D1",
  "claim": "§14(3) 消除所有非确定源:VIO 归约定序、未初始化读、关闭 imu_prop 定时器",
  "problem": "CMakeLists.txt:62-65 对全部 TU 加 OpenMP 旗标,Eigen 在 `EIGEN_HAS_OPENMP` 下会对足够大的 GEMM 自动多线程(结果矩阵 ≤33 列时目前走串行,但这是实现细节);D1 没有 `EIGEN_DONT_PARALLELIZE` 或 `OMP_NUM_THREADS=1` 兜底,也没有像 D3 那样用 hexfloat 轨迹做逐位门。",
  "fix": "回放构建加 `-DEIGEN_DONT_PARALLELIZE`,进程设 `OMP_NUM_THREADS=1`;确定性门用 `%a` 输出的 state_trace 而非有限精度 .tum 文本。"
 },
 {
  "severity": "minor",
  "designs": "D1",
  "claim": "§13 `docker/` 目录与 §17.3「docker,ROS1」集成测试",
  "problem": "D1 对 Noetic 容器构建没有任何细节:Sophus 必须锁 a621ff2 并以 `-std=gnu++98` 编(本机 `~/fastlivo2_mid360_ws/docker/Dockerfile` 已验证的做法),vikit 须用 `xuankuzcr/rpg_vikit@6c886c8` 且其 CMake 硬编码 `-march=native`,远端需 dockerproxy/gh-proxy/tuna 镜像、docker data-root 迁出 8.1 GB 的根分区、ROOT_DIR 烧死后的 Log 隔离。这些都是构建能否成功的前提。",
  "fix": "采用 D3 §13 的 Dockerfile 要点,或直接 `docker save` 本机 `fastlivo2-mid360:noetic`(4.57 GB,已含 Sophus)到远端 `docker load`,再补 valgrind/rosbags。"
 },
 {
  "severity": "minor",
  "designs": "D2",
  "claim": "T20/T21/T22/T23 以输出文件 sha256 判定逐位一致",
  "problem": "D2 的输出是 `.tum`(十进制有限精度),亚 ulp 的分叉在文本里被四舍五入掩盖,门可能假阳性通过。",
  "fix": "照 D3 每个周期以 `%a`(hexfloat)写全状态与 diag(P) 的 state_trace,再比对 sha256。"
 },
 {
  "severity": "minor",
  "designs": "D2",
  "claim": "§5.6 L3 状态组完好性与 §5.6.4 退化下限直接给宿主 P_pp/P_vv 加项;§5.7 步长约束",
  "problem": "宿主 P 被抬高后立刻进入 LIO 的点方差 `pv.var`(voxel_map.cpp:387,由 `state_.cov` 的 rot/t 块构成)与点到面门限(:735-737),以及地图插入权重(LIVMapper.cpp:420-421):ALARM 误触发会放宽 LIO 关联门并把高方差点写进地图,形成反馈回路。这是对宿主的运行时干预,与「宿主一行不改」的零改动承诺不在同一层面,且没有 D2 自己的 T 类测试覆盖误触发率。",
  "fix": "把 ALARM/退化下限的触发率与触发前后局部 ATE 作为 D1 诊断的预注册指标;评测主臂默认关闭 L3 抬升,另设开启臂做消融。"
 },
 {
  "severity": "minor",
  "designs": "D3",
  "claim": "§5.3 GNSS 放周期末(VIO 之后)的理由 2:光度 VIO 收敛域小,不让 GNSS 外点或大新息改动 VIO 的先验",
  "problem": "GNSS 后验在下一周期 `processImu` 的 :256 `state_propagat = _state` 就成为 LIO 与 VIO 的先验(IMU 传播不稀释修正),所以「大新息」只是晚一个周期进入 VIO,并未被隔离;真实代价是本周期地图插入(:413-424)与视觉点生成(vio.cpp `generateVisualMapPoints`,在 EKF 之后)用的是不含本历元 GNSS 修正的位姿。",
  "fix": "要么如实把取舍写成「地图/视觉点插入滞后一周期」,要么把更新挪到 :254-256(D1/D2 的挂点):H1 与 GNSS 更新同一挂点,省掉 endOfCycle 特判,LIO 无点提前返回(:343-347)时 GNSS 照常。"
 },
 {
  "severity": "minor",
  "designs": "D3",
  "claim": "§10.3 新增 `img_compressed_cbk`,用 `cv::imdecode(IMREAD_COLOR)` 解成 BGR 后进入 `pushImage()`,与 republish 语义相同",
  "problem": "`imdecode(IMREAD_COLOR)` 一律返回 BGR 通道序,而 image_transport 的 compressed 解码会按 `format` 字段(如 \"rgb8; jpeg compressed bgr8\")做通道换序;若 M3DGR 的 JPEG 以 rgb8 记录,D3 直读得到的通道序与 republish 路径相反,vio.cpp:1797 `CV_BGR2GRAY` 的加权(0.114/0.587/0.299)使灰度图不同,所有臂的 VIO 数据与上游发布链不一致。",
  "fix": "在 I2 数据审计里核对 CompressedImage 的 `format` 字符串,必要时按其源编码换序;把「直读灰度与 republish 灰度逐像素一致」加入 I2。"
 },
 {
  "severity": "minor",
  "designs": "D3",
  "claim": "§12.1 `-ffp-contract=off` 用于保证新增代码不改变既有表达式的 FMA 收缩;`accumulateF` 加 noinline",
  "problem": "这一条实际上是必要且正确的:在 `ros:noetic-ros-base-focal` 内实测 GCC 9.4 对 `-std=c++17` 的默认仍是 `-ffp-contract=fast`(上游 CMake 的 `CMAKE_CXX_EXTENSIONS OFF` 并不把它关掉),`-march=native` 有 FMA 时任何 TU 改动都可能改变收缩决策。但 D3 把它做成可选 profile 并让 `rt` profile 保持默认,而 noinline 与 fp-contract=off 同时存在,文档没说清哪一个才是承重项。",
  "fix": "文档明确:承重项是 `-ffp-contract=off` 且必须加在 A0/R 与 HEAD 两个二进制的所有 TU(含 vikit);noinline 只是保险。任何逐位门都只在 eval profile 下定义(已是如此)。"
 },
 {
  "severity": "minor",
  "designs": "D2/D3",
  "claim": "D2 §2.3、D3 §3.3:LIO/VIO 更新后用 `A = P_GH (P⁻)⁻¹`(D3 为 `M=(P⁻)⁻¹C`)对增广块做精确条件化",
  "problem": "公式经我逐项核对为精确(K_G = P_GH(P⁻)⁻¹K_H;P⁺_GH = A P⁺;P⁺_GG = P_GG − A(P⁻−P⁺)Aᵀ;VIO 的 G 来自最后一次被接受的迭代、状态可能已回退,这一怪癖在联合滤波里同样出现,故仍一致)。唯一的工程条件是 P⁻ 可逆:cov_w 对重力块(16-18)没有过程噪声(IMU_Processing.cpp:395-399),其方差只减不增,长序列后可到 1e-10 量级;曝光块在 `exposure_estimate_en=false` 时恒为 1e-5。都不为零,LDLT 在 double 下可行,但条件数会到 1e8-1e10。",
  "fix": "保留 D3 的 Jacobi 缩放与 D2 的伪逆回退;在 D0 诊断里记录 diag(P⁻) 的最小值与条件数,作为长时协方差健康监测的一列。"
 },
 {
  "severity": "minor",
  "designs": "全部",
  "claim": "确定性回放:按记录时间直读 bag,每条消息后同步排空处理循环",
  "problem": "三者做法正确,但都没有写明一个可直接利用的事实:main.cpp 只用 `ros::spinOnce`(单线程),且 sync 层对到达时序不变(见 D2 的种子问题),因此实时 0.888±0.115 m 的方差来源只剩启动期跨话题次序、vio.cpp:1554 的 float 归约和 :248 的 UB 读。这决定了哪些扰动模型是有意义的(D3 的 start_offset 命中真实来源;D1 的图像抖动是另一种有效 CRN;D2 的虚拟时钟无效)。另有上游缺陷::490 早退跳过 :538 清空,下一帧陈旧 IMUpose 条目与新条目 offset 同量级,极少数点会被二次去畸变——三者为保等价都保留,应在局限里列出。",
  "fix": "在评测协议里明确列出这三个实时非确定源,并把「种子间轨迹两两不同」和「valgrind 无未初始化读」作为回放仪器的静默自检项。"
 },
 {
  "severity": "minor",
  "designs": "D1/D2",
  "claim": "以 `topic_tools::ShapeShifter`(在线)与 `rosbag::MessageInstance::getMessageDefinition()+write()`(回放)取原始字节,运行时按内嵌 message_definition 建 schema 解码",
  "problem": "方案成立(已在 focal 镜像里确认 `topic_tools/shape_shifter.h` 随 ros-base 提供,rosbag `isType` 接受 md5 \"*\"),许可证姿态也最干净。但 D1 的 §12.2「回放」与「在线」两条路径都要各写一遍字节提取,而解析器只有一份;D1/D2 都没有说明 msg_def 版本变化(gnss_comm 字段增删)时的拒收策略是否与 D3 一样明确报错。",
  "fix": "把「扁平化 (type,name) 序列与内置期望表逐项比对,不一致即拒收并报版本差异」(D3 的做法)加进 D1/D2 的解码器;两条字节来源汇聚到同一个 `feedBytes()` 入口并做同片段逐字节一致测试(D2 的 T3)。"
 },
 {
  "severity": "minor",
  "designs": "D1",
  "claim": "§13 vio.cpp:1552-1555 并行循环改为写 `errors[i]` 与逐点计数,循环外串行求和",
  "problem": "改动本身正确且保留了并行(比 D2/D3 关掉 MP_EN 更快),`visual_submap->errors[i]` 本来就在 :1632 逐点写;但这属于对估计器核心文件的修改,与 D1 自己「push-through 是唯一数学改动」的表述叠加后,vio.cpp 已有两处改动,零通道门更难定义。",
  "fix": "若转向增广块方案,则用 D3 的 eval profile(去 MP_EN)替代改 vio.cpp;实时 rt profile 保持上游原样。"
 }
]