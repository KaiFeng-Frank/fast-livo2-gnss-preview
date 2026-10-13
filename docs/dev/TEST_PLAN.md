> Historical research record (through 2026-10-03). Paths are illustrative; archived runs are not bundled. For current status and executable entry points see [the repository README](../../README.md). Current release docs/LICENSING.md supersedes historical license notes. This record does not authorize new experiments or assert current release validation.

# 测试计划(静默自检;任一未过不产出评测数字)。落盘副本:`/workspace/fastlivo2_gnss_scout_20260924/test.md`

2026-09-25 按骨架核验修订(AMENDMENTS A9/A17/A20):U4/U6 两档、U5 拆出 U16、U12 拆出 U17 与钟跳检测入 U9、新增 T1–T5 表与 I14。

2026-09-25 P2b 按 A27–A40 同步:§1 夹具说明(A39)、U7/U9 在 φ≠0 的容差(A34)、U12 检出率(A35-C2)、I11 种子语义(A30)、§4 U 臂含越界读保护(A27)与 FAST-LIVGO 脚注、§4 前置审计(A40)与运行种子(A30)。

2026-09-25 P2b 集成补记(AMENDMENTS「P2b 集成记录」):U14 环按时长保留(A33,WP-b R7);U5 夹具金标准的历元取法(每 GPST 秒一个)、U6 第二档只在带星历话题的片段上(A24 同理)。

2026-09-25 U15 实现(AMENDMENTS A60):仿真分三组——A 模型匹配(噪声按滤波器自己的 §4.7 方差抽取,判 NEES/NIS 的 χ² 界)、B 真实时间相关误差(下表 U15 行的 GM 多径等,判「不过度自信」)、C 模型失配(判有界与恢复);场景按设计的假设构造(配准游走从激活起、偏垂线按 σ_DOV 抽取、A/C 组载波不含大气项),偏离处即 A60 的发现;快档进 ctest,全档 50 次 × 1 km 用 `U15_FULL=1`。
2026-09-25 A61:U15 加 ATE 代理(宿主 traj_W、全局 traj_N、里程计输出 traj_N_odom 的 SE(3) 对齐 ATE)与 `OpenSkyRealistic` 场景(A58 误差图景、M3DGR 式 C/N0);新用例 `U15Ate.OdometryOutputRemovesTheDrift`;F1/F2 之后场景默认按物理真值构造(配准游走从地图原点起、载波含大气项)。§4 指标加 e1o(里程计输出的 E1,主表单列,与 FAST-LIVGO 同口径,A61 裁定 1)。
2026-09-26 A61 仪器更正:生成器与滤波器共用「每颗卫星一组广播星历」(`SimParams::single_ephemeris`,默认开;整份文件时星历切换处真值卫星瞬移,跨切换的 TDCP 对带厘米级的场景自造误差);ξ 的倾斜真值以宿主的重力估计 ĝ 为参考(与滤波器 φ 的定义同,宿主重力误差不再算到 ξ 头上);A-FA 对照臂按 DESIGN §4.5 原样不相接(A60 量的就是它);ROLLBACK 事件记入 `RunResult::event_log`;C 组「违例后有界」的 ξ 判据不计 LIO 位姿本身出错的窗口(地图重影,`NeesSample::host_deceived`:ξ 映射宿主坐标,在那里吸收了 LIO 的错误,与真配准无从比较),天线照判。

2026-09-25 P3 集成(AMENDMENTS「P3 集成记录」,INTERFACE_CHANGES P3-C C14):U5/U7 的 1 Hz 规则按 A43(每个 GPST 秒第一个到达的历元);U7 补 C10(多普勒方差含 var_v_bridge)与 R6;U9 补 R6;U12 单故障边界按 A42(5.8σ);新增门面单测(`test_p3_facade`,§1 表末行)。

2026-09-26 A64/A65:U7 的 TDCP 克隆列按 A64 的定义核对(= +u_k1ᵀ,克隆端用 k1 视线;唯一有意偏离精确导数的列),同时核对精确导数 +u_k0ᵀ 与两者之差在用例里可测;U10 补 L5 的 Σ⁺ 半正定判据两例(Jacobi 缩放后的特征值 ≥ −1e-9;无噪声停止克隆的强更新须提交);U12 相接对 `hpos + hcl` 收紧到 < 1e-15。U15 仿真里 F9 只在回访/重影场景打开(代码默认关);判据不变,A64 核心下全档的长静止场景天线带内比例 0.79 不过(A65)。

2026-09-26 A66/A67:U10 补 consider 列用例(`ConsiderColumnsKeepTiltAndAreTheConstrainedOptimum`:多普勒行对 φ 的增益为零、φ 从其余行得受约束最优增益,与「伪距全增益 → 多普勒 φ 置零」的顺序滤波相同,φ 方差最小,L5 一般形式后验 NIS = r̃ᵀS⁻¹r̃,L4 统计量不变,无多普勒行或无 tilt 槽时逐位不变)。U15 加 C 组场景「低仰角多普勒逐星偏差」(`LowElevDopplerBias`:OpenSky 同一场景,仰角 < 30° 的星带 ±0.15–0.55 m/s 常值多普勒偏差,并按 A59 描述为 20 dB-Hz、接收机多普勒 std 4 Hz;各项仍按滤波器方差抽取)与用例 `U15GroupC.LowElevationDopplerBiasTiltBoundedOdometryBeatsHost`(倾斜两分量 z² 的合并均值与每次运行均值 ≤ 5,快档 ≤ 8;每次运行里程计 ATE < 宿主;全档另跑同场景 consider 关的对照,倾斜 z² 须更大)。数值见 AMENDMENTS A67。

2026-09-27 A70(A69 第 1 条):U10 补投影用例(`ProjectedDopplerRowsAreTiltFreeAndKeepTheRest`:多普勒行在其 R 度量下投影到倾斜列的正交补后,保留行单位方差、对 φ 零敏感;对其余状态的信息 = 把 φ 当作无先验扰动参数的批量 LS 消元;提交的更新与基的取法无关、与信息形式批量解相同;L4 统计量不变、L5 后验 NIS = 投影系统的 r̃ᵀS⁻¹r̃;A67 的 H2 通道在投影后消失(consider 下仍在);投影 + consider 时多普勒历元 φ 逐位不动;无多普勒行、无 tilt 槽、倾斜列为零时逐位不变;倾斜列成比例时只去一维;m ≤ r 时该类不贡献;R 非正定回滚)。U7 补 `DopplerRowsProjectedOffTheTiltAreFirstOrderTiltFree`(真实多普勒行投影后,模型对 φ 的中心差分为零,其余列 T·H = 中心差分)。U15 低仰角多普勒偏差用例的默认机制改为投影(consider 默认关),全档对照为两者皆关(A66 之前的增益),判据不变。数值见 AMENDMENTS A70。

## 1 单元测试(gtest,ROS 无关;小夹具 `gnss/test/data`:开发集片段 `<seq>_<t0>_<D>s`(D 为实际时长,单文件 ≤ 2 MB)的 M3DGR GNSS 原始消息字节 + 去注释 msg_def + 同日 IGS BRDC 片段,M3DGR 数据 MIT、IGS 数据公开可用,注明来源;布局与发现规则 A24/A39)
| 编号 | 内容 | 判据 |
|---|---|---|
| U1 时间 | GPST↔UTC(按日期查闰秒表,2026 断言 18 s)、BDT(−14 s,周差 1356)、GST(周差 1024)、GLONASST(UTC+3 h)、周翻转;t_est 映射三种 base、offset/drift 符号;t_rcv−b/c 修正 | 与 RTKLIB 函数及已知历元逐位相等 |
| U2 线格式 | 手工 msg_def + 合成字节往返;夹具字节解码与 rosbags Python 黄金文件逐字段一致;畸形缓冲报错不崩;期望表校验拒绝改动过的布局 | 全部通过 |
| U3 编号/星历映射 | 160 个 gnss_comm 编号→(sys,prn)→satno 往返;eph_t 字段(A、toes 按系统、week、tgd、svh、GLO frq/glo_fcn/dtaun)与 bag2rinex→readrnx 逐字段往返 | RINEX 精度内相等 |
| U4 卫星状态(两档,A9-3) | 第一档:同一条 RINEX 星历经 ephPos 与经 EphSnapshot 两路 rs/dts **逐位相同**;第二档:同 (sat,toe) 的 bag 星历对同日 BRDC:|Δr|<1e-3 m、|Δdts|<1e-11 s;**显式覆盖 BDS MEO/IGSO/GEO(C59–C62)、Galileo(I/NAV,A16)、GLONASS 各≥1 颗**;(对 IGS SP3/CLK 的可选项已按 A45 删除);t_tx 与 satposs 内部 time[i] 差<1e-9 s | 全部通过 |
| U5 改正/SPP | 在 pntpos 解处本模块伪距预测与 ssat.resp 差<1e-6 m;多普勒在 estvel 解处残差均值≈0;补丁构建的 pntpos() 与未打补丁构建(`LIVO_RTKLIB_APPLY_PATCHES=OFF`)一次生成的 hexfloat 金标准逐位一致(A9-4);std 单位格点判定(夹具版取样:每个开发集片段每 GPST 秒**第一个到达**的历元,A43,与 `rows.cpp` 的 `EpochDecimator` 同一规则;P2b 时的「离整秒最近且在 50 ms 内」作废) | 全部通过 |
| U6 路径等价(两档,A9-3) | 第一档:夹具字节经 C++ 解码器 与 经黄金 CSV(%.17g)回读,同批历元进 pntpos **逐位相同**;第二档:bag 对 RINEX(F14.3/D19.12 舍入)pntpos 位置 3D<1 cm、钟<1 cm(RINEX NAV 由 bag 星历写出,无星历话题的片段不参与,同 A24 对 U3/U4 的规定) | 全部通过 |
| U7 雅可比 | J_a、J_w(δ≠0、随机状态)、伪距/多普勒/TDCP/PVT 行、克隆生成雅可比、C(g) 的 ε(ε_z=0 达机器精度)、J_rw(含 J_l(φ̃),A45/R6;φ=0 旧式须检出)、重锚 J、td 列;对宿主 19 维经 ⊞ 与全部槽位做流形中心差分;多普勒方差含 var_v_bridge=acc_n²δ(A45/C10,κ_AR 之外);伪距/PVT 行的 1 Hz 规则 = 每个 GPST 秒第一个到达的历元(A43,合成网格与夹具历元时刻);投影掉倾斜的多普勒行对 φ 一阶零敏感(A70) | 相对误差<1e-6(δb_g 桥接项放宽;φ≠0 时重锚与 δg 列同此容差,A34) |
| U8 钩子 | 联合 (19+n) 维滤波 vs 拆分:传播、类 LIO(6 列、(I−G)P⁻)、类 VIO(7 列、多层、最终 G 于非不动点)、P⁺≠(I−G)P⁻ 的情形 | 相对差<1e-12;Σ 半正定 |
| U9 增广块 | 枢轴噪声下当前点均值/方差不变;钟 Q 与 Van Loan 矩阵指数一致;增删槽位后剩余协方差逐位不变;重锚(A11 完整 T,含 φ、δg 耦合;φ≠0 用精确列 Ξ·J_l(φ̃)·E_xy,A34)后全局天线位置与协方差变化<1e-9(φ≠0 亦然);φ≠0 下枢轴噪声使当前点不变为精确(A45/R6);停放/激活一致增广;**钟跳检测 k=±1,±2 ms、非整毫秒>1 km 判 reset**(自 U12 移入,A9-2) | 全部通过 |
| U10 更新器 | 线性高斯情形等于批量 LS 闭式解;Joseph 与标准形式一致且 PSD;Householder 投影与「单差+白化」解一致;χ² 阈值;回滚后逐位不变;条件迭代收敛;consider 列(A66);多普勒行投影掉倾斜(A70:零敏感、信息 = 无先验扰动参数消元、H2 通道消失、逐位不变的情形、秩亏) | 全部通过 |
| U11 克隆 TDCP | 两历元线性玩具与批量平滑一致;克隆版 vs 完整延迟状态滤波 1000 次 MC NEES 均值在维数±10% | 通过 |
| U12 门控/检测 | 注入单/多 NLOS、1/2/10 周与半周跳 ⇒ 单故障 k≥5.8σ 检出≥99%(A42;原 5.62σ 与理论 0.9901 贴线,C9)、5σ ≥95%,双故障 6σ ≥0.90,强相关 TDCP 行 6σ ≥0.98(A35-C2),虚警在 α±50%;钟跳历元 TDCP 扣 k·c·1 ms 后仍可用(rows.cpp);±50 ms 时偏台阶经 TimeOffsetMonitor(A17)一个窗内 TOFF_WARN;L0 各 MaskBit 逐位触发 | 通过 |
| U13 初始化 | 合成轨迹+真实星历:已知 ψ、r_0、t、b、d、ι 可恢复;静止/激励不足保持等待;初始化历元不重复计数 | 容差内 |
| U14 桥接 | 无噪声高频真轨迹:δ≤0.1 s 位置误差<0.1 mm;跨周期串接;插值与 :412-421 传播逐项一致;环按时长保留(A33):至少 `bridge_ring_cycles` 个周期,且 t_k ≥ t_latest−(max_bridge_s+0.1 s) 的周期全保留,15 Hz 周期下 δ=0.28 s 可桥接 | 通过 |
| U16 GNSS-only 估计器(WP-c,A9-1) | 自写稳健 LS 对注入 +30 m 单星粗差必剔除;2 星同时 NLOS 检出≥95%;无噪声合成历元位置/钟/ISB 恢复<1e-6 m;多普勒 LS 恢复速度与钟漂(含 Sagnac 率与杆臂项);只解钟 LS 的 b、d 与 MAD² 方差;C/N0–仰角 NLOS 预筛只标低 C/N0 星 | 通过 |
| U17 完好性(P3,A9-2) | LIVO_ALARM 连续 5 历元进入、10 历元一致解除;航向连续 20 历元偏差>max(5°,3σ) ⇒ REINIT_YAW;退化指数 λ₁<400 连续 10 帧置位、>800 解除 | 通过 |
| 门面(P3-A2,`test_p3_facade`) | 合成宿主(随机游走地图 + mm 级 LIO + Grass02 真实星历几何)上走 DESIGN §9 调度:enable=false 零触碰;原始字节经解码器;非法配置抛出;激活→跟踪→TDCP 配对与克隆;OBSERVE 与无数据时宿主逐位不变;A43 每秒第一个历元(相位 +5/−7/0 ms);迟到历元桥接或丢弃;post_vio 与 pre_lio 消费同一组历元(A12);中断停放与恢复;整毫秒钟跳由 TDCP 桥接;L-PVT 臂;消融臂只去掉各自那一类 | 13 例全过 |
| U15 Monte-Carlo 一致性(核心验证,50 次) | ROS 无关仿真:1 km 地面轨迹 1–2 m/s 含转弯;IMU 按 M3DGR Allan;真实星座几何;伪距含 GM 多径(τ=30 s,σ=1.5 m)+电离层残余+白噪声;载波含模糊度/周跳/多径;LIO 用「相对漂移地图的位姿量测」(地图系按 q_true 随机游走,mm 级噪声,6 列 H);情景:开阔、城市多径、中断 120 s、钟跳、周跳、4 系统+ISB、少星(2–3 星)、长静止、回访吸附 Δ、模型失配(确定性偏航漂移、地图重影、低仰角多普勒逐星偏差,A66) | ξ/钟/全局天线位置平均 NEES 与各类 NIS 落在 χ² 95% 双侧界;对照固定锚版 NEES 随时间增长(分组、口径与数值界见 AMENDMENTS A60);里程计输出的 ATE 低于宿主(A61) |

### 1b gnss_io 测试 T1–T5(ROS 无关,与核心同一 ctest;定义见 AMENDMENTS A20)
| 编号 | 内容 | 判据 |
|---|---|---|
| T1 映射 | 夹具字段树 → ObsEpoch/KeplerEph/GloEph/Klobuchar/ReceiverPvt 按 §12.3 | 与黄金 CSV 逐字段一致 |
| T2 msg_def | `MsgDefParser::md5` 与夹具连接头 md5sum 相等(6 类型);改动布局被期望表拒收 | 通过 |
| T3 导出一致 | C++ 解码器 CSV 与 `tools/gnss_bag_export.py` 同片段 | 逐字节相同 |
| T4 RINEX 源 | `RinexSource` 经 readrnxt 读 OBS/NAV,布局与 bag 路径一致 | 通过 |
| T5 UBX | 合成 RXM-RAWX/SFRBX 帧经 input_ubx 解出;校验和错误计数 | 通过 |

## 2 集成测试(docker ROS1,需数据)
| 编号 | 内容 |
|---|---|
| I1 | eval/rt 两 profile 均可构建;gnss_core 无 ROS 环境构建;compile_commands 三包 -march/-ffp-contract 一致 |
| I2 数据审计 `tools/m3dgr_audit.py`(每 bag 一份报告,归档后才进下一步) | 话题/类型/md5/msg_def;频率与时间范围(header vs 记录时间);按 (sys,freq,code) 直方图;每历元星数;C/N0 分布;psr/cp/dopp 可用率;LLI/status 统计;std 单位判定;URA 语义;星历时间线与 C59+;iono/time_pulse 内容;GNSS 到达延迟分布;Livox 消息类型/md5;CompressedImage format 与直读灰度逐像素一致;GT 文件格式;杆臂方向(设计 §12.6-7) |
| I3 | G-确定性(R 与 HEAD 各两次) |
| I4 | G-零通道、G-旁路(`update_enable=false`,钩子与 F 累积真实执行) |
| I5 | G-无数据(剔除 GNSS 话题的 bag) |
| I6 | 实时冒烟 roslaunch+rosbag play 1×:不丢消息;GNSS 每历元 p50<5 ms、p99<10 ms;缓冲不增长;输出齐全 |
| I7 | valgrind memcheck(eval 构建,60 s 片段,`--track-origins=yes`)无未初始化读 |
| I8 | Outdoor01 前 120 s:运动后 30 s 内 ACTIVATE、各类 NIS 在带内、σ_ψ<0.2° |
| I9 | Outdoor04 人为删 30/60/120 s GNSS ⇒ 再捕获修正 ≤3×预测 σ;非退化段 W 轨迹与 GNSS 关闭差<1 cm;无 LIO 失效 |
| I10 | 注入周跳与方位扇区 30% 伪距 +5–40 m ⇒ 周跳检出≥99%、NLOS 剔除≥90%;注入 ±50 ms 时偏 ⇒ TOFF_WARN 触发且工具恢复<2 ms |
| I11 | 种子 k=0..9(k = 记录序第 k 帧 LiDAR 开视窗,A30)的 state_trace 两两不同;同种子各臂 ACTIVATE 前逐位相同 |
| I12 | L-PVT/T−pr/T−dop/T−tdcp/T−reject/A-FA/A-FA−norej 端到端可跑 |
| I13 | `update_slot: pre_lio` vs `post_vio`(挂点 `onPostVio`,A12):F0 与 T 各跑 2 条开发序列 10 种子,集成 ATE 无显著差 ⇒ 冻结 pre_lio |
| I14 | rostest:同一 bag 消息经 `rosbag::MessageInstance::write()` 与经 `topic_tools::ShapeShifter::write()` 送入同一 `feedBytes()`,解码记录逐字段相同(原 gnss_io T2,A20) |

## 3 入库自检与标定(开发集,与评测集不相交)
- 开发集:Occlusion03/04、Dynamic03、Grass02(约 17 GB;可扩 Dynamic04)。
- 自检:设计 §12.6 全部 10 项。
- 2026-09-25 执行记录与裁定见 AMENDMENTS A58(工具 `tools/calib/`):NIS/dof=1 的标量取在边际方差上,κ_AR>1 的类(伪距、PVT、多普勒)复核用 κ 归一化(白等效)NIS,TDCP 用自报值;配置 sha256 在 A58。
- 闭式标定(不扫参):D1 协方差诊断;q(RPE-路程 NNLS,上分位);τ_ρ、τ_D(新息自相关);各类 σ 尺度(平均 NIS/dof=1 的标量);S_b、S_d(多普勒 LS 钟漂 Allan);掩码阈值(新息尾部);TDCP 接受率与 NIS 静默自检(q 偏小=锁死判据:TDCP 接受率<70% 或 NIS/dof>1.5 ⇒ 回到 q 标定)。完成后配置冻结、sha256 写入预注册文档。

## 4 评测协议(预注册)
- **序列**:Varying-illu03/04/05、Outdoor01/04、Dark01/02(RTK 真值;Dark 为夜间非拒止,README:349-350)+ GNSS-denial01/02(ArUco 起终点);原包约 82.3 GB,过滤后推断约 30 GB。
- **前置审计 `tools/gt_audit.py`**:真值坐标系(经纬度对 9Ps WGS-84 PVT:GCJ-02 偏百米级;局部 XYZ 对 PVT-ENU 做 SE(3)/Sim(3) 对齐看残差结构,GCJ-02 在 1 km 尺度米级畸变,推断)并纠偏;GT 时基互相关(>20 ms 补偿);可信掩码:9Ps h_acc>1 m 或 num_sv<8、GT 速度尖峰>3σ 或间隔>0.5 s 前后 2 s;2HP 杆臂未知记为地板。**GT 时基语义(A40)**:GT = RTK(mavros NavSatFix)在首个定位处的 WGS-84 ENU,戳 = mavros 头戳,行序非单调且同戳重复;评测只用 `gt_audit.py` 写出的 `gt_audited.tum`(稳定排序 + 同戳去重 + 时基补偿),不用原始 GT 文件;链式时基估计失效或与记录时间旁证相差 >50 ms ⇒ P4 FAIL、不写 `gt_audited.tum`、评测阻断。**掩码加宽(A40)**:「前后 2 s」对全部判据(含 h_acc、num_sv)生效,不加宽的接收机判据比例另列;h_acc>1 m 判据本身未裁定(INTERFACE_CHANGES WP-g 请求 (2))。
- **臂**:U(R = 0d2c034 + A27 越界读保护 + 回放;报告写明)、F0(GNSS 关,须逐位=U,只报一次)、L-PVT、T、T−pr、T−dop、T−tdcp、T−reject、A-FA、A-FA−norej;可选 T+floor、L-SPP;参照行 rnx2rtkp SPP、接收机 PVT;FAST-LIVGO 表 II 单列注明「未审计、单次、LiDAR/配置/对齐未说明」;脚注:M3DGR/FAST-LIVGO 的 FAST-LIVO2 数字对应发布版 986b4b1(VIO border 48 px),本仓库 U 为 0d2c034(80 px)+ A27 保护(A40)。
- **运行**:每臂×每序列×10 种子(`replay.seed` k=0..9:记录序第 k 帧 LiDAR 开视窗;`start_offset_s` 只作固定裁头,A30),eval profile,可并行(独立容器);实时复核:U/T/T−reject 在 Outdoor01、Dark01 各 3 次(rt profile,独占,阶段末争用筛查)。约 10 臂×9 序列×10 种子≈110 CPU·h(推断 0.8–1× 实时)。
- **指标**:E1 主指标 ATE RMSE 仅位置,`evo_ape tum -a`(SE(3) Umeyama 不含尺度,与 M3DGR/FAST-LIVGO 一致),`--t_max_diff 0.05`,GNSS 臂用 traj_N、U/F0 用 traj_W,取各臂共有时段(激活后),全段与可信掩码两版;e1o(A61):GNSS 臂的里程计输出 traj_N_odom 按同一 E1 构造,主表单列,与 FAST-LIVGO 论文列同口径(其 W→ENU 初始化后固定);E1' 联合估 GT 天线杆臂(3 参数,各臂同程序);E2 全局指标:只平移对齐的 ATE 与 SE(3) 对齐旋转角(检验 ψ 与真值坐标系);E3 RPE 10/50/100 m;E4 所有臂 traj_W ATE(检验核心是否被改动);E5 GNSS-denial 官方 ArUco_evaluate.py 起终点误差(W 与 N 两套);E6 一致性:各类 NIS/dof、接受率、f_p 分布、t 对 RTK 的 NEES 序列;E7 发散计数(局部 ATE>5×U 同种子、LIO 有效点<10% 持续 5 s、NaN);E8 耗时 p50/p99。
- **统计**:每序列内各臂对 U(及 T 对 L-PVT、T 对各消融、A-FA 对 T)10 种子成对 Wilcoxon 符号秩精确检验,Holm 校正;「优于」=校正后 p<0.05 且 ≥8/10 种子胜且成对差中位<0;「劣于」对称;其余平局。跨序列只报优/平/劣计数,不混合均值。不伤害等效:Dark01/02、GNSS-denial 上配对差 95% 自助 CI 上界<δ=max(0.05 m,0.5×U 种子 IQR)。试点规则:2 条开发序列上确定性集成 IQR<实时(U 5 次)IQR 的 25% ⇒ 加图像 ±1 ms 抖动 CRN;仍不足 ⇒ 实时重复(非配对)。
- **预注册预测**(设计 §5):(a)–(f) 逐条对照判定;M1 非退化段 f_p 中位<1e-3;M2 运动后 30 s 内 σ_ψ<0.2°;任一失败如实报告为证伪,不在评测序列改参重跑。
- **报告**:精确值只进表(中位与 IQR),正文只报胜率/计数;仪器自检未过不呈报。