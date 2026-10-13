> Historical research record (through 2026-10-03). Paths are illustrative; archived runs are not bundled. For current status and executable entry points see [the repository README](../../../README.md). Current release docs/LICENSING.md supersedes historical license notes. This record does not authorize new experiments or assert current release validation.

# 审查意见逐条处理(落盘副本 `/workspace/fastlivo2_gnss_scout_20260924/resolved.md`)

## R1 估计理论
| # | 意见 | 处理 |
|---|---|---|
| R1-1 major D1 §5.6 回访「正确」 | **采纳**。删除「正确」判定,改写为共享局限(design §5、§16-2):P_ξp≈3e-3·P_pp,吸附跳变 ≈Δ 出现在全局输出,跨越跳变的 TDCP 被 L4 拒,t 以分钟级重收敛;GNSS-denial 预测限定「中断始于起点区」;事件日志记幅值。 |
| R1-2 major D2 LIVO_ALARM 协方差手术 | **采纳**。L7 只置标志:GNSS 更新改 Schmidt(宿主列 K 置零)直至解除,不改宿主 P。其建议之(a)「本周期不插入地图」**驳回**:那是对宿主地图行为的运行时干预,与「宿主一字不改」门禁同层冲突,误报时在地图留洞;Schmidt 已隔离 GNSS 对宿主的影响,地图插入正确性由 LIO 自身负责(voxel_map.cpp:413-424 用 LIO 后验);ALARM 期间局部 ATE 作诊断记录。 |
| R1-3 major D2 κR 步长约束 | **采纳**。删除;宿主步长超限=L5 整体回滚(0/1)。 |
| R1-4 major D3 Huber IRLS 进 EKF | **采纳**。IRLS 只留在 GNSS-only 估计器(L1、初始化);EKF 内用 Baarda w + NIS + FDE。 |
| R1-5 major D3 按时间的帧游走 | **采纳**。按路程 Δs,q_time=0;标定用 RPE-路程 NNLS。 |
| R1-6 major D1 TDCP 缺 σ_toff 项 | **采纳**。TDCP R 加 (‖Δv_pair‖σ_toff)²;σ_toff 取互相关峰宽;激励不足时缩短 interval 或暂停 TDCP。 |
| R1-7 major D2 MA(1) 忽略 | **采纳**。用 D1 不重叠配对,严格独立。 |
| R1-8 minor D3 重叠克隆 | 同上采纳。 |
| R1-9 minor 边缘 χ² → Baarda w | **采纳**。L4 逐星用 w_i=(S⁻¹ν)_i/√(S⁻¹)_ii;验后 NIS 用 rᵀ(R−HΣ⁺Hᵀ)⁻¹r。 |
| R1-10 minor D2 单步无迭代 | **采纳**。条件迭代(σ_ψ>2° 或 |Δψ|>0.1°,≤3 次)+ 激活/再捕获前 5 历元固定 3 次。 |
| R1-11 minor FromTwoVectors 调平的 z 分量 | **采纳**。用 D1 C_ref 无偏航调平,ε_z≡0,δg 列进 H(gravity_est_en 时)。 |
| R1-12 minor 4-DoF 无倾斜漂移通道/曲率说法 | **采纳**。加 2 维 φ(σ_DOV 先验,路程游走);删曲率说法;q_tv 取上分位。 |
| R1-13 minor 无枢轴重锚 | **采纳**。t 相对 o^W,100 m 重锚 J。 |
| R1-14 minor 多普勒/TDCP 相关 | **采纳** D2 互斥(配对端点历元);对内其余历元相关写入局限 7;T−dop 量化。 |
| R1-15 minor D3 H2 前提表述 | **采纳**。改为「G 在终态 x_i 线性化但 x_i 非不动点」(design §2.4)。 |
| R1-16 minor κ_AR 覆盖不到大气偏差 | **采纳**。局限 4;E6 加 t 对 RTK 的 NEES 序列。 |
| R1-17 minor D2 q_th 低估 | **采纳**。标定取上分位;TDCP 接受率/NIS 静默自检作「q 偏小=锁死」判据。 |
| R1-18 minor D3 C=0 激活 + NIS 触发放大 | **采纳**。一致增广(含互协方差),去掉数据驱动放大,σ_ψ post-fit 缩放。 |
| R1-19 minor 挂点位置 | **采纳**。pre_lio 为默认,`update_slot` 配置项,I13 等价性核对后冻结。 |

## R2 GNSS 正确性
| # | 意见 | 处理 |
|---|---|---|
| R2-1 major RAIM 层空操作(valsol return 0 已注释 pntpos.c:384;raim_fde 只在 estpos 失败进入 :689) | **采纳**(本轮亲核 pntpos.c:384/:689 属实)。L1 改为自写 GNSS-only 稳健 LS + w 检验;pntpos 只作参照行;U5 加 +30 m 单星粗差用例。不打 valsol 补丁(不再依赖它)。 |
| R2-2 major D2 发射时刻少减 δt_s | **采纳**。t_tx=t_rx−P/c−δt_s(satposs :804→:822);U4 断言 t_tx 差<1e-9 s。 |
| R2-3 major BDS toes 语义 | **采纳**。toes 按系统(BDS=time2bdt(gpst2bdt(toe)));U4 显式覆盖 BDS 三类卫星。 |
| R2-4 major D1 未回填 obsd_t.code | **采纳**。§12.3 显式 freqs→CODE_* 表;自检 1 硬判据 sat2freq 与 freqs 一致。 |
| R2-5 major D1 GLO 伪距/IFB/dtaun | **采纳**。GLO 伪距默认关;多普勒/TDCP 照用;delta_tau_n→dtaun 补入映射。不建 IFB 状态(默认关即不需要)。 |
| R2-6 minor pntpos 不导出 d/钟协方差 | **采纳**。P2 扩展导出 estpos/estvel 全量 x/Q;且初始化用自写 LS 输出 d 与协方差。 |
| R2-7 minor URA 语义 | **采纳**。入库自检 4 对拍 RINEX 定语义;不经 var_uraeph,自算 σ_eph²。 |
| R2-8 minor std 单位 | **采纳**。自检 3 格点判定;阈值按米给出。 |
| R2-9 minor BDS 1575.42 判 B1C | **采纳**。拒收计数。 |
| R2-10 minor D1 钟跳层在周跳层后 | **采纳**。L2 钟跳提前到 L3 周跳前,λΔΦ 扣 k·c·1 ms。 |
| R2-11 minor P3 按 IODE 回查 | **采纳**。克隆缓存 eph_t 副本,删 P3。 |
| R2-12 minor varerr 引用/默认 prcopt | **采纳**。σ² 自算(§4.7),rtkx_varerr 只供回归。 |
| R2-13 minor 相位缠绕 | **采纳**。由 Δclk 投影吸收;S_b 下限含缠绕项;急转弯 TDCP NIS 分箱检查。 |
| R2-14 minor D4 长间隙误报 | **采纳**。阈值加 j_maxΔt³/(12λ)。 |
| R2-15 minor Klobuchar 共用 | **采纳**。ERR_BRDCI 可配,开发集按系统分箱看 NIS;局限 6。 |

## R3 集成正确性
| # | 意见 | 处理 |
|---|---|---|
| R3-1 major D2 `-march=x86-64-v3` | **采纳**。三包统一 `-march=native`(逐位只在同机同镜像),patch vikit CMake,compile_commands 一致门。 |
| R3-2 major D2 虚拟时钟种子无效 | **采纳**(sync 层对到达时序不变,LIVMapper.cpp:973/:1010-1033/:988-997)。种子改 start_offset;硬门「种子两两不同」;图像抖动作第二 CRN 源。 |
| R3-3 major D2 保留 UB 读 | **采纳**。R 与 fork 同做 IMU_Processing.h:60/:80/:81 零初始化;valgrind 入 R 门禁。 |
| R3-4 major D1 扩维不能逐位 | **采纳**。改增广块;push-through 不再需要(记为扩维退路文档)。 |
| R3-5 major D1 无 Log 隔离 | **采纳**。独立容器绑定挂载 Log。 |
| R3-6 major D1/D2 RTKLIB 静态直链符号外泄 | **采纳**。SHARED + hidden + --exclude-libs,不定义 TRACE。 |
| R3-7 minor D1 D2 修复改变首帧 | **采纳**。用零初始化(与事实上游一致)。 |
| R3-8 minor IMUpose 残留 | **采纳**。桥接样本在循环内自记,不用 IMUpose。 |
| R3-9 minor 缺 EIGEN_DONT_PARALLELIZE/hexfloat | **采纳**。 |
| R3-10 minor D1 缺 docker 细节 | **采纳** D3 §13 要点 + 本机镜像 save/load 备选。 |
| R3-11 minor .tum 判逐位 | **采纳**。hexfloat state_trace。 |
| R3-12 minor L3/退化下限进宿主 P | **采纳**。主臂 floor_enable=false,T+floor 为可选探针;触发率入诊断。 |
| R3-13 minor D3 周期末理由 | **采纳**。挂点改 pre_lio,理由改写为「地图插入含本历元修正、LIO 无点照常」。 |
| R3-14 minor 压缩图通道序 | **采纳**。按 format 字段换序;I2 逐像素一致。 |
| R3-15 minor 承重旗标表述 | **采纳**。明确 `-ffp-contract=off` 全 TU 为承重项,noinline 为保险。 |
| R3-16 minor P⁻ 条件数 | **采纳**。Jacobi 缩放+伪逆回退;filter_cov.csv 记 min diag/条件数。 |
| R3-17 minor 三个非确定源与 :490 缺陷 | **采纳**。写入 §13 与局限 11。 |
| R3-18 minor 解码器拒收策略/统一入口 | **采纳**。期望表比对拒收;两路汇聚 feedBytes;T3 逐字节一致。 |
| R3-19 minor D1 改 vio.cpp 归约 | **采纳**。用 eval profile 去 MP_EN 替代,不改 vio.cpp;rt profile 保持上游。 |

## 驳回或合并(给依据)
- D2 §5.6.3「帧平移 SPP 重锚」苛刻条件版:**不设独立层**,合并进 L6 锁死恢复(同一机制,条件用「L1 一致但 L4 拒>50%」),避免两套重锚规则。
- D3「GNSS 放周期末(VIO 后)」:**驳回**为默认(R1/R3 一致判定其理由不成立:GNSS 后验下一周期 :256 照样成为 VIO 先验);保留为配置项供等价性核对。
- D1 push-through 改写 voxel_map/vio:**驳回**(R3-4 扩维不可逐位);保留为扩维退路文档。
- D1「主系统=卫星最多者、ISB 槽位去主系统后预留」:**驳回**,主系统固定 GPS(ι_G≡0)。理由:ISB 语义依赖主系统,固定才能跨序列/跨中断保持 ISB 近常量先验;GPS 中国境内常年 ≥6 星,可观性不受影响。
- D2 R 斜坡恢复(κ 从 9 降到 1)与 settle:**驳回** R 斜坡(数据无关但仍是人为降权,信息就是信息,L4 门宽已随 P_ξξ 增大);保留 settle 1 s/≥4 星作为进入 RECOVERY 的 0/1 条件。
- D1 星间单差与 D3「Schmidt 考虑」:均不采用(单差丢钟连续性;Schmidt 只作 L7 告警期间的临时模式而非常态)。