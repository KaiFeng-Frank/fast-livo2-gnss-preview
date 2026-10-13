> Historical research record (through 2026-10-03). Paths are illustrative; archived runs are not bundled. For current status and executable entry points see [the repository README](../../../README.md). Current release docs/LICENSING.md supersedes historical license notes. This record does not authorize new experiments or assert current release validation.

# FAST-LIVO2 × 原始 GNSS 紧耦合：完整设计（城市峡谷/遮挡鲁棒性优先）

行号约定：FAST-LIVO2 指 `/workspace/fast_livo2_ref`@0d2c034；RTKLIB 指本地参考副本 rtklibexplorer main@06e8644（实现锁 tag v2.5.1，行号需复核）。「推断」=未实测；「预测」=预注册、可证伪。

## 0 结论先行
1. **宿主 19 维 ESIKF 一行不改，GNSS 全部进「增广块」x_G。** DIM_STATE 仍为 19（common_lib.h:30）；voxel_map.cpp、vio.cpp、common_lib.h 零改动。x_G＝地理参考帧 ψ/tᴺ、接收机钟、杆臂、时偏 td、TDCP 随机克隆。三个钩子与宿主组成一个联合高斯：①IMU 传播后按上游每步 F_x 的累积积 F_total 传播交叉协方差；②LIO/VIO 更新后用宿主先验/后验协方差对 x_G 做**精确**条件化；③GNSS 更新用宿主+增广联合协方差形式+Joseph。GNSS 关闭/未初始化时宿主算术与上游逐位一致（可做 sha256 回归）。
2. **「地图当真值」的回答：GNSS 信息一致地流向三处。** (a) 帧状态——LIVO 漂移住在这里，按行驶距离注入过程噪声、航向噪声以当前天线为枢轴（全局位置不跳）；(b) LIVO 速度/陀螺偏置——多普勒/TDCP 与 LIVO 速度信息同量级；(c) LIVO 位姿——只在 LIVO 自身信息弱的（退化）方向，由协方差自动分配，辅以自算退化指数的协方差下限使分配诚实。健康时 GNSS 对 pᵂ 的增益 ~1e-4 以下，不撕图。
3. **鲁棒性＝三层完好性＋双向外部裁判＋0/1 成员资格。** L1 信号层（掩码＋σ 模型）；L2 量测层：LIVO 预测新息作外部裁判，innovation 空间 Baarda w 检验（全 S，天然边缘化公共钟）＋联合 NIS＋迭代 FDE，1 颗星也能判；L3 状态组层：GNSS-only 解作外部裁判，分别审计 LIVO 速率（与坐标系无关）、帧航向、帧平移。EKF 更新内不用鲁棒核。
4. **TDCP 用 4 维随机克隆（天线 ENU 位置＋参考钟），与多普勒逐星互斥；伪距 1 Hz 抽取并按相关时间膨胀方差。**
5. **时间**：离线仿射标定（记录时间统计＋联合 yaw-时延速度互相关）为先验＋在线 td 随机游走状态；DTW 只作诊断；time_pulse_info 在无主机侧 PPS 捕获时不能做 <10 ms 同步。
6. **FAST-LIVGO 表 II 三现象**统一归因（推断）：GNSS 直接作用于位姿＋固定锚 TDCP 过度自信。本设计预测：开阔改善、夜间等效不劣、去剔除后全局精度降但**不发散**；并设可证伪探针臂 A8（按其论文描述在同一代码复刻）。
7. **I/O 与许可证**：gnss_comm 代码与 .msg 均不入库；M3DGR 消息用「bag 内嵌消息定义驱动」的自写 ROS1 反序列化器读取（实时/回放共用），另有 BSD-3 Python 导出器交叉验证；轨道/钟差/改正只取 RTKLIB-EX v2.5.1（BSD-2）。
8. **可复现评测**：确定性回放（记录时间序直读 bag、同步调用处理循环、单线程、固定浮点语义）＋虚拟时钟延迟仿真种子（8 个）做成对统计；原版臂＝上游估计器原样＋同一回放外壳。

## 1 帧、符号与约定
变换统一写 T_{A←B}：p^A = R_{A←B}p^B + t_{A←B}。
| 符号 | 定义 |
|---|---|
| E | ECEF WGS-84 |
| N | 固定参考点 r₀ 处 ENU；R_{N←E}=RTKLIB xyz2enu(pos₀)（rtkcmn.c:2160）；C≡R_{E←N}=R_{N←E}ᵀ |
| W | FAST-LIVO2 世界系＝初始化时 IMU 系，不水平、yaw 任意（IMU_Processing.cpp:144-145；gravity_align_en 默认 false，LIVMapper.cpp:77，保持关闭以维持 A0 逐位等价） |
| I | 宿主状态 R≡R_{W←I}、p≡pᵂ_I、v≡vᵂ_I（common_lib.h:215-221） |
| L/C | extR/extT＝T_{I←L}（LIVMapper.cpp:104-105,645）；Rcl/Pcl＝T_{C←L}（vio.cpp:35-39） |
| A | 天线相位中心，杆臂 lᴵ（增广状态） |
| Λ(gᵂ) | 把 u=−gᵂ/‖gᵂ‖ 转到 e_z 的最小旋转（同 LIVMapper.cpp:236 的 FromTwoVectors 构造） |
地理参考：M≡R_{N←W}=R_z(ψ)Λ(gᵂ)；aᵂ=p+Rl；aᴺ=M(aᵂ−oᵂ)+tᴺ；aᴱ=r₀+C aᴺ；oᵂ 为枢轴常量（§2.4）。全局输出 T_{N←I}：R^N_I=MR，p^N_I=M(p−oᵂ)+tᴺ。
扰动：右扰动 R=R̂Exp(δθ)，与上游一致（common_lib.h:170,184,197-198；IMU_Processing.cpp:385；voxel_map.cpp:453）；增广块欧氏；新息 ν=z−ĥ，修正 δx=Kν。
观测符号（与 RTKLIB 一致，已核 pntpos.c）：
- 伪距：P = ρ + b + ISB_sys − c·δt_s + I + T + ε，ρ=geodist(r_s,aᴱ)=‖r_s−aᴱ‖+ω_E(x_s y_a−y_s x_a)/c（Sagnac，rtkcmn.c:3612/3621）；TGD/BGD/ISC 由 prange 施加在观测侧（pntpos.c:110）；残差式 v=P−(r+dtr−c·dts+dion+dtrp)（pntpos.c:335）。
- 载波（周）：λΦ = ρ + b + ISB − c·δt_s − I + T + λN + ε。
- 多普勒：距离率 ρ̇=−λD；−λD = e_sᵀ(v_s−v_aᴱ) + ω_E/c·(v_{s,y}x_a+y_s v_{a,x}−v_{s,x}y_a−x_s v_{a,y}) + d − c·δṫ_s + ε（resdop pntpos.c:580,587；与 gnss_comm dopp_res、FAST-LIVGO 式12 同号）。
- TDCP：z=λ(Φ_j−Φ_{j−1})，h=ρ_j−ρ_{j−1}+(b_j−b_{j−1})−c(δt_{s,j}−δt_{s,j−1})；ΔI、ΔT 在 Δt≤1.5 s 忽略（FAST-LIVGO 式10），ISB 抵消。
- e_s=(r_s−aᴱ)/‖·‖（接收机→卫星）；ẽ_s=Cᵀe_s；λ=c/f（GLONASS 按频道）；b=c·δt_r（参考系统 GPS，m），d=c·δṫ_r（m/s），ISB_GPS≡0。
时间：GPST（观测标签）；闰秒 18 s（rtkcmn.c:171，末条 2017-01-01）；滤波时基 τ＝IMU 头戳−imu_time_offset（LIVMapper.cpp:776），Livox 头戳不加 lidar_time_offset（:744），图像加 img_time_offset 与 exposure_time_init（:847,:953）；映射 τ(t_GPST)=α+β(t−t_ref)+td（td>0 表示 GNSS 历元在主机轴上更晚）。

## 2 状态
### 2.1 宿主
上游 19 维 [δθ0-2|δp3-5|δτ6|δv7-9|δb_g10-12|δb_a13-15|δg16-18] 原样，不改初值、噪声、更新。
### 2.2 增广块 x_G（Eigen 动态矩阵，典型 12-17 维）
| 组 | 分量 | 维 | 先验 | 过程 |
|---|---|---|---|---|
| F 帧 | ψ, tᴺ | 4 | 初始化器解×κ_init=4 | §2.4 |
| K 钟 | b, d, ISB_C, ISB_E, [ISB_R] | 4(+1) | 初始化 LS | b←b+dΔt |
| Λ 杆臂 | lᴵ | 3 | 标定值，σ=5 cm/轴 | 常量 |
| T 时偏 | td | 1 | 离线标定，σ=10 ms | 随机游走 1e-8 s²/s |
| C 克隆 | c_a=aᴺ(τ_j), c_b=b(τ_j) | 4 | 上一历元后验线性化克隆 | 常量 |
钟 Q 用连续 PSD 离散（Groves TC_KF 约定，**不用**上游 σ²·dt²，IMU_Processing.cpp:395-399）：Q_bd=[[S_bΔt+S_dΔt³/3, S_dΔt²/2],[S_dΔt²/2, S_dΔt]]，ISB：S_isbΔt·I；默认 S_b=0.1 m²/s、S_d=0.1 m²/s³、S_isb=1e-4 m²/s。GLONASS 伪距默认不用（FDMA 频间偏差），多普勒/TDCP 照用。
### 2.3 为何不扩 DIM_STATE，及精确钩子
不扩的理由：(a) 上游 LIO/VIO 信息形式对全维 P 求逆（voxel_map.cpp:468；vio.cpp:1497,1661），克隆插入瞬间联合协方差奇异，零方差即 NaN（pushthrough_check 已证）；(b) 维数固定 vs 星座进出/克隆增删；(c) 改固定尺寸会改变 Eigen 规约顺序，GNSS 关闭也不再逐位等价；(d) StatesGroup 逐成员手写拷贝与 VIO 回滚（common_lib.h:142-165；vio.cpp:1401,1510,1679）易漏新成员。
钩子（宿主量测只作用于 x_H）：
- 传播：UndistortPcl 每步 F_total←F_x·F_total（IMU_Processing.cpp:401 后）；周期末 P_GH←Φ_G P_GH F_totalᵀ，P_GG←Φ_G P_GG Φ_Gᵀ+Q_G，x_G←Φ_G x_G。宿主过程噪声与 x_G 独立 ⇒ 精确。
- LIVO 更新后（LIO、VIO 各一次）：先验 (x⁻,P⁻)=state_propagat，后验 (x⁺,P⁺)=_state：A=P_GH(P⁻)⁻¹（LDLT 解）；x_G←x_G+A(x⁺⊟x⁻)；P_GH←A P⁺；P_GG←P_GG−A(P⁻−P⁺)Aᵀ（对称化）。证明：K_G=P_GH HᵀS⁻¹，而 x⁺⊟x⁻=P⁻HᵀS⁻¹ν、P⁻−P⁺=P⁻HᵀS⁻¹HP⁻。上游 IEKF 终止迭代满足 x⁺⊟x⁻=K(·)、P⁺=(I−G)P⁻（voxel_map.cpp:469-472,489-490）；VIO 各金字塔层共用同一先验，终值同式（vio.cpp:1664-1667,800）⇒ 对最终线性化点精确（⊟ 雅可比取 I 同上游）。只读 state_propagat 与 _state，不需暴露局部变量 G。
### 2.4 帧漂移模型
4 自由度（yaw＋平移）恰是 LIVO 不可观方向（MINS Lemma 5.1；GVINS 式13-14；OKVIS2-X 4-DoF T_GW）；横滚/俯仰由 IMU 可观，帧倾斜取 Λ(gᵂ)，不另立状态（默认不给 GNSS 改 gᵂ 的通道）。
Q_F = G_ψ(q_ψΔs+q_ψtΔt)G_ψᵀ + diag(q_th,q_th,q_tv)Δs + q_ttΔt·I，G_ψ=[1; −[e_z]×m_k]，m_k=M(aᵂ_k−oᵂ)，Δs＝本周期桥接轨迹折线长度。G_ψ 使航向噪声以当前天线为枢轴（δaᴺ=[e_z]×mδψ+δtᴺ=0）：航向漂移只改未来位移方向；静止不增噪（无需 GVINS 低速锁 yaw 规则，gvins_estimator.cpp:898-908）。
默认（开发序列标定后冻结，§14）：q_ψ=1.2e-7 rad²/m（0.2°/√100m）、q_th=2.5e-5 m²/m、q_tv=1e-4 m²/m（Z 漂移更大）、q_ψt=1e-10、q_tt=1e-8。量纲：一个历元（~0.15 m）tᴺ 噪声 σ≈1.9 mm，与 TDCP 同量级 ⇒ TDCP 能以载波精度驱动 tᴺ 跟踪平移漂移；1 km σ≈16 cm ⇒ 伪距缓慢锚定。
枢轴重锚：‖aᵂ−oᵂ‖>100 m 时 o'=aᵂ、t'=t+M(o'−o)，对 (ψ,tᴺ) 行列施 J=[[1,0],[[e_z]×M(o'−o), I]]（含交叉协方差）；全局天线位置及其协方差不变；作用是限制 ψ-tᴺ 相关杠杆、保条件数。
### 2.5 信息账本与零成本诊断
推算（code-update §6，待实测）：LIO 后 σ_p≈1-4 mm；σ_v 稳态≈3 cm/s（推断：cov_acc=0.5 按 σ²dt²/步 ⇒ PSD≈2.5e-3 m²/s³，15 Hz、σ_p=3 mm ⇒ σ_v²≈√(2qσ_p²/T)≈8e-4）。伪距（1 Hz 膨胀后 3-10 m）对 pᵂ 增益 ~1e-6 ⇒ 只进 b/ISB、tᴺ；多普勒（5-17 cm/s/星）对 vᵂ 增益 0.1-0.3 ⇒ 改速度与 b_g，定 ψ、d；TDCP（LS 位移 2-3 mm）⇒ 健康时主定 ψ 与 tᴺ 漂移，对 pᵂ 的 mm 级拉扯被下一帧 LIO 以地图重锚（无害）；退化方向 P_pp 经 IMU 诚实增长（σ_v 约 0.05 m/s/√s），修正自动分给 pᵂ 并在同周期进图。
- D0-a（今天可做）：临时副本在 LIVMapper.cpp:371 后打印 _state.cov.diagonal()（:263 有同类注释行），实时跑一条 bag。
- D0-b（正式、估计器零改动）：原版臂＋回放记录器，每个 LIO/VIO 阶段读 state_propagat.cov 与 _state.cov，输出 √diag 与信息增益 (P⁺)⁻¹−(P⁻)⁻¹ 的平移 Schur 特征谱；再用 observe 模式（GNSS 只算不写）的真实卫星几何，计算假想更新下修正在 pᵂ 与 (ψ,tᴺ) 间的分配。
- 预注册判读：R1 非退化帧 σ_p 中位≤1 cm 占≥80% ⇒ 地图当真值成立，预测健康段 share_host 中位<1%；R2 λ₁<λ_deg 的帧＝预测发生位姿级修正的时段；R3 w_D=σ_v²/(σ_v²+σ_D²/m)>0.3 ⇒ 多普勒实质修正 LIVO 速度；R4 若 D0 与假设相反，结构不变（分配由协方差自动完成），只在评测前修订预测并存档。
- D1（实现后）：同记录＋每次更新 share_host=‖J_Hδx_H‖²/(‖J_Hδx_H‖²+‖J_Fδx_F‖²)（J 为全局天线位置对各块的雅可比）、NIS、克隆一致性。

## 3 观测模型与雅可比
### 3.1 运动链（历元 τ_g≤t_k，Δ=t_k−τ_g）
桥接快照给出 τ_g 处 IMU 均值 (R̂_g,p̂_g,v̂_g,ω̂_g) 与 ΔR=R̂_gᵀR̂_k^prop。以 t_k 宿主为自变量：R_g=R_kΔRᵀ，p_g=p_k−Δp̂_gk−(v_k−v̂_k^prop)Δ，v_g=v_k−Δv̂_gk，ω_g=ω_m,g−b_g。误差：δθ_g=ΔRδθ_k，δp_g=δp_k−Δδv_k，δv_g=δv_k，δω_g=−δb_g。aᵂ_g=p_g+R_g l，v_aᵂ=v_g+R_g[ω_g]×l，m=M(aᵂ_g−oᵂ)，v_aᴺ=Mv_aᵂ；钟 b(τ_g)=b_k−d_kΔ。
### 3.2 雅可比（空＝0）
| 列 | 伪距 | 多普勒 ρ̇ | TDCP 当前历元 |
|---|---|---|---|
| δθ | ẽᵀMR̂_g[l]×ΔR | ẽᵀMR̂_g[ω_g×l]×ΔR | 同伪距 |
| δp | −ẽᵀM | | −ẽᵀM |
| δv | +Δ·ẽᵀM | −ẽᵀM | +Δ·ẽᵀM |
| δb_g | | −ẽᵀMR̂_g[l]× | |
| δψ | −ẽᵀ[e_z]×m | −ẽᵀ[e_z]×Mv_aᵂ | −ẽᵀ[e_z]×m |
| δtᴺ | −ẽᵀ | | −ẽᵀ |
| δb / δd | 1 / −Δ | 0 / 1 | 1 / −Δ |
| δISB_s | 1（s≠GPS） | | |
| δl | −ẽᵀMR̂_g | −ẽᵀMR̂_g[ω_g]× | −ẽᵀMR̂_g |
| δtd | −ẽᵀMv_aᵂ | −ẽᵀMα_aᵂ | ≈0 |
| δc_a / δc_b | | | +ẽ_{j−1}ᵀ / −1 |
非零宿主列 {0-5,7-12}，跳过 6、越过前 7 列 ⇒ 不能套 LIO/VIO 的 leading-block 写法（voxel_map.cpp:466-472；vio.cpp:1660-1667），GNSS 用稠密 H 的协方差形式（m≲80，状态≤36）。Δ=0、ΔR=I 时与 code-update §4、lit-filter §3.2、LIGO gnss_psr_dopp_factor_nor.hpp:179-201 同号。α_aᵂ＝天线加速度。
### 3.3 伪距
z=prange(obs)（单频加 ERR_CBIAS²）；卫星 satpos(t_tx) 广播星历（ephemeris.c:737），t_tx=τ_rx−P/c（P 无效用 ρ̂+b̂，同 satposs 思路）；电离层 Klobuchar（ionmodel rtkcmn.c:3688，GPS 参数来自 /ublox_driver/iono_params），×(f_L1/f)²，方差 (0.5I)²(f_L1/f)⁴；对流层 Saastamoinen（tropmodel :3791），方差 (0.3/(sin el+0.1))²（tropcorr pntpos.c:251）；轨道方差取 satpos var。R=σ²_model·κ_corr+var_ion+var_trop+var_eph+var_cbias+桥接噪声。节拍：GPST 整秒历元（1 Hz，确定性）；κ_corr=max(1,τ_c/T)=5（τ_c=5 s，城市多路径时间相关，10 Hz 白噪声假设会把 tᴺ 锁在有偏值）。
### 3.4 多普勒
z=−λD；R=σ²_D·2＋桥接＋(σ_ω‖l‖)²。与 TDCP 逐星互斥：同星同历元有有效 TDCP 时不入更新（u-blox 多普勒出自载波环，同源；FAST-LIVGO 两者同用 ⇒ 速度信息重复计数，推断）；仍用于周跳检测与 L3。
### 3.5 TDCP（随机克隆）
历元 j 更新后插入 y_j=[aᴺ(τ_j); b(τ_j)]：P←[[P,PJᵀ],[JP,JPJᵀ]]，旧克隆删行列（协方差形式下即边缘化）。奇异联合协方差只出现在协方差形式更新与只求 P_HH⁻ 逆的钩子里，安全。历元 j+1：两历元载波有效、无周跳、Δt≤1.5 s；两端卫星用**同一条**（较新）星历，避免 IODE 切换跳变。R=σ²_Φ,j+σ²_Φ,j−1；相邻 TDCP 共享历元噪声的 MA(1) 负相关忽略（对长程位移偏保守）。与 FAST-LIVGO 固定锚（式11，上一后验当常数）不同：克隆带自身不确定度与相关，一致；固定锚让每个 TDCP 成为相对错误锚点的 mm 级准绝对约束。
### 3.6 松耦合臂 A2
NAV-PVT（/ublox_driver/receiver_pvt）→ aᴺ 位置（H 为伪距行位置部分以 −I 代 −ẽᵀ）＋NED 速度；R=diag(h_acc²,h_acc²,v_acc²)·κ_corr（1 Hz），速度 vel_acc²·2。与 TC 共用帧、初始化（PVT 代 SPP）、门限（3 自由度）、中断/恢复——只换量测，差异可归因。
### 3.7 近似清单
桥接忽略 Δ²|a|δθ（<5e-5 m）、δb_aΔ²/2；过程噪声相关用 SRI-GVINS 式22-29 的保守加法 R+=HQ_wHᵀ（精确式 S=H(P−Q_w)Hᵀ+R、K=(P−Q_w)HᵀS⁻¹，量级<1 mm²，不采用）；LOS 对位置导数（~2e-4 (m/s)/m）、Sagnac 率对速度导数忽略；Λ(gᵂ) 视为已知；1 km 内地球曲率 Z 效应（d²/2R≈8 cm）由 q_tv 吸收。

## 4 时间对齐
- 时基链：GPST→Unix（gpst2utc rtkcmn.c:1914）→ τ=α+β(t−t_ref)+td。M3DGR 软件同步（README Known Issues），LiDAR 头戳时基未知（推断），主机钟可能在 bag 内被 NTP 步进 ⇒ α、β、分段均需标定。
- 离线工具 tools/timebase_calib.py：①记录时间统计：range_meas 的 bag 记录时间−GPST→Unix ⇒ 偏差＋输出延迟分布、鲁棒线性拟合、>50 ms 持续台阶即分段；同样统计 LiDAR/IMU/图像头戳−记录时间以判定 LiDAR 时基。②主方法：observe 模式回放得 LIVO 天线速度与 GNSS-only 多普勒 LS 速度，30 s 窗对 (ψ,lag) 二维搜索最大化水平速度内积（转弯提供计时信息），加垂向速度通道；抛物线亚采样；窗间 Huber 线性拟合 ⇒ (α,β,σ)。③DTW 对速度模交叉核对，只报偏移中位与离散度，与②差>3σ 告警。输出 gnss/time 覆盖层。同工具核对 GT 时基。
- 在线 td：随机游走，先验＝离线 σ（默认 10 ms）；慢速地面平台弱可观（伪距 v·δtd≈1.5 cm/10 ms），主要作用是把残余时偏诚实计入 S；检测到台阶则方差重置为 (50 ms)²。
| 方案 | 物理约束 | 不确定度 | 失效模式 | 用途 |
|---|---|---|---|---|
| DTW（FAST-LIVGO §III-C1） | 无，任意翘曲 | 无 | 匀速段歧义、非物理翘曲、O(N²) | 仅诊断 |
| 互相关＋仿射拟合 | 偏移＋钟漂 | 有 | 无加减速/转弯的窗跳过 | 离线主方法 |
| 滤波状态 td | 随机游走 | 有 | 弱可观时吸收他误差 | 在线细化（紧先验） |
- time_pulse_info：只含「下一脉冲 GPST＋utc_based＋time_sys」，无主机侧 PPS 沿捕获；记录时间含串口/驱动延迟与 TIM-TP 不定提前量；GVINS 需 /external_trigger 配合，M3DGR 无此话题（README:145-152）⇒ 不能做<10 ms 同步；仅用于闰秒/时间系统检查、>0.5 s 粗错检查、主机钟台阶检测。真机应把 PPS+GPRMC 接 MID-360。
- 桥接：UndistortPcl 前向传播时记录桥接轨迹（首样本＝起点状态与 angvel_last，其后与 IMUpose 同步写，IMU_Processing.cpp:430）与 F_total；不依赖 IMUpose（:538 被清，:490 提前返回时不清）。环形缓冲保留 4 个 LIO 周期；τ_g∈(t_{k−1},t_k] 用本周期；迟到历元串接多周期 IMU 相对运动，Δ≤0.3 s，更老丢弃计数。桥接噪声：Σcov_acc·dt_i²（速度）、Σcov_gyr·dt_i²（姿态）、位置≈Q_vΔ²/3，跨周期另加 (2 mm)²/周期（推断）。推断：RAWX 到达延迟 50-100 ms、相机周期 67 ms ⇒ 多周期桥接是常态路径。

## 5 观测质量与鲁棒性
### 5.1 框架
| 层 | 裁判 | 决定 | 对象 |
|---|---|---|---|
| L1 | 接收机自报（C/N0、LLI、半周、std） | 掩码与 σ | 单信号 |
| L2 | LIVO+帧+钟预测（外部于 GNSS） | 逐星逐类 0/1 成员资格 | 每个量测 |
| L3 | GNSS-only 解（外部于 LIVO） | 协方差诚实化与模式 | LIVO 速度、帧航向/平移、退化方向 |
EKF 内不用 IGG-III/Huber（有效 R 非线性、协方差不一致）；鲁棒核只用于 GNSS-only 估计器（初始化、L3、回退）。
### 5.2 L1
σ²_T=s_sys²[a_T²+b_T²/sin²el+c_T²·10^((45−C/N0)/10)]，取 max(σ²_T,σ²_rcv)（psr_std/cp_std/dopp_std 作下限），乘 κ_T（与 RTKLIB varerr 同族，pntpos.c:51,65）。默认：伪距 a=b=0.3、c=1.0 m；多普勒 a=b=0.02、c=0.05 m/s；载波 a=b=0.002、c=0.003 m；s_sys：G/E/C=1、R=1.5、BDS GEO=2；κ：伪距 5、多普勒 2、TDCP 1。示例：60°/45 dB-Hz ⇒ 1.1 m、0.06 m/s、4 mm；20°/35 ⇒ 3.3 m、0.17 m/s、11 mm。硬掩码：el≥10°；C/N0≥30（伪距）/25（多普勒）/28（载波）；伪距连续跟踪≥10 历元（GVINS 思路 gvins_estimator.cpp:298-311），载波≥3；svh=0、URA≤10 m、星历年龄≤2 h(G/E)/1 h(C)/30 min(R)；status bit0/bit1/bit2。比 FAST-LIVGO 的 35 dB-Hz/15° 宽：城市峡谷里硬掩码砍掉多数卫星，本设计有强先验逐星检验兜底。
### 5.3 L2 逐星检验＋联合检验＋迭代 FDE（伪距/多普勒/TDCP 分开）
1) ν=z−ĥ，S=HPHᵀ+R（全矩阵，含公共钟与帧造成的星间相关）；2) w_i=(S⁻¹ν)_i/√(S⁻¹)_ii~N(0,1)，因 S 含公共钟方差，w_i 天然是边缘化公共钟后的单星检验；3) NIS=νᵀS⁻¹ν 对 χ²_m(1−α)；4) 超限则剔 argmax|w_i|（α=1e-3 ⇒ 3.29）重算，至通过或 m<m_min（伪距 1、多普勒 1、TDCP 2）；5) 被剔>50% ⇒ 该类本历元不更新（可能是先验错），计入 L3；6) 验后 NIS 劣于验前 ⇒ 回滚（照 vio.cpp:1677-1681 语义）。
不以 SPP 残差为主判据：验后残差有故障涂抹、冗余仅 m−3−n_sys，多颗 NLOS 同时存在时 RTKLIB raim_fde 单故障留一法（pntpos.c:472，要求 nvsat≥5）会误剔/漏剔；LIVO 预测新息每颗星对外部预测单独检验、无涂抹、1 颗也能判。先验不可信时段退回 GNSS 内部稳健估计（§5.6.5）。
### 5.4 周跳（任一触发即断链）
D1 LLI；D2 载波无效、半周未解、bit2/bit3 两历元变化（0.5 周跳）；D3 Δt>1.5 s；D4 多普勒-相位：|ΔΦ+(D_j+D_{j−1})Δt/2|>max(0.5 周,4σ_DΔt)（dΦ/dt=−D，接收机钟漂两者一致抵消）；D5 LIVO 预测新息 w 检验＋5 cm 绝对阈（L1 一周 19 cm 在 mm 级预测下>10σ，LLI 未置位时最强）；D6 双频时 GF 历元差>5 cm。LIGO 的码减载波 6σ（GNSS_Assignment.cpp:368,473）在 m 级伪距下对 1 周不敏感，仅可选。上线前在真实数据核对 λΔΦ 与 ΔP、−λDΔt 同号（相关>0.99）。
### 5.5 钟跳/重置
伪距新息加权中位 |m|>1000 m 且≥70% 卫星一致 ⇒ u-blox 钟跳：b←b+m，方差 (10 m)²，本历元 TDCP 断链。钟方差>(1e4 m)² 或长中断 ⇒ 以预测天线位置做只解钟稳健 LS 重置 b、d。
### 5.6 L3 状态组完好性
- 5.6.1 LIVO 速度：GNSS-only 多普勒 LS（≥5 星、IRLS＋w 检验、GDOP<6）速率模与 LIVO 预测天线速率比较（无需 ψ），加垂向速度；连续 5 历元 d²>χ²₁(0.999) 且差>0.2 m/s ⇒ LIVO_ALARM：P_vv+=(0.5 m/s)²I、P_pp+=(0.3 m)²I，保持 5 s，步长上限放宽到 0.2 m；连续 10 历元一致且退化正常则退出。替代 FAST-LIVGO Mode B：触发来自独立速度核验而非 Hessian 特征值，且不「完全信任 GNSS」。
- 5.6.2 帧航向：‖v‖>0.5 m/s 时水平速度方向连续 20 历元偏差>max(5°,3σ) ⇒ σ_ψ²+=(5°)²。
- 5.6.3 帧平移重锚（保守）：SPP 同时满足≥8 星、≥2 系统、PDOP<3、验后 σ̂<3 m、平均 C/N0>40，且与预测偏差>5σ、持续 10 s、方向稳定 ⇒ 沿偏差方向膨胀 P_tt。苛刻是因为「全体 NLOS 同侧偏置＋RAIM 自洽」正是城市峡谷陷阱。
- 5.6.4 自算退化指数：I=(P⁺)⁻¹−(P⁻)⁻¹（线性下恰为 HᵀR⁻¹H，因 P⁺=K_1，voxel_map.cpp:468,489）；取 [θ,p] 块，平移 Schur 补 I_t=I_pp−I_pθI_θθ⁻¹I_θp，特征分解得 λ₁≤λ₂≤λ₃、u₁（1/√λ₁＝单帧该向 σ）；LIVO 合并信息取 LIO 先验到 VIO 后验。不用 lio/min_eigen_value（读入 planner_threshold_，平面拟合阈值，voxel_map.cpp:42,86；上游 6×6 特征分解被注释 :467）。10 帧中位＋滞回：λ₁<400 m⁻²（σ>5 cm）进 DEGENERATE，>800 退出。动作（仅本周期有 GNSS 更新时）：u₁ᵀP_pp u₁<σ_floor²=(5 cm)² ⇒ P_pp+=(σ_floor²−u₁ᵀP_pp u₁)u₁u₁ᵀ（加半正定项）。
- 5.6.5 先验不可信时的回退（初始化、REINIT、钟重置、长中断后）：候选筛选改用 SPP/多普勒 LS 的 IRLS-Huber＋迭代 w 检验，加 C/N0-高度角 NLOS 预筛：在线用高仰角且 RAIM 通过卫星拟合 C/N0_exp(el)（缺省 48−12(1−sin el)），低于期望 8 dB 标嫌疑、初始化期剔除。
### 5.7 步长约束（协方差一致）
单次更新对宿主 ‖δp‖、‖δθ‖ 超上限（常态 5 cm/0.2°，ALARM 20 cm/1°）时，二分求最小 κ≥1 使 R←κR 下不超限，用 κR 更新（等价信息更少的量测，协方差一致），剩余信息留给后续历元。帧不限幅。目的：保护 LIO 点到面关联门（3σ 与 3×radius，voxel_map.cpp:721-737）。
### 5.8 少于 4 颗星
更新对 m 无下限（伪距/多普勒≥1、TDCP≥2）；信息进入钟、帧沿 LOS 分量、LIVO 速度沿 LOS 分量；ISB 近常量，开阔期估好后单星系统仍有几何贡献。前提是 NOMINAL 态先验紧；长中断再捕获要求≥4 星稳定 1 s 才进 RECOVERY。

## 6 初始化（宿主不受影响）
前置：!imu_need_init、lidar_map_inited、IMU 初始化后≥50 个 LIO 帧、≥6 颗有星历卫星、时间映射可用。
I1 收集窗：逐历元稳健 SPP 与多普勒 LS（§5.6.5 预筛），记录桥接后 LIVO 天线 aᵂ、v_aᵂ。I2 航向/钟漂：水平速度≥0.3 m/s 的历元≥30、水平位移≥5 m；2D Procrustes 初值 ψ₀=atan2(Σ(v_D×Λv_a)_z, Σ(v_D·Λv_a)_h)，再以原始多普勒 GN（ψ 与每历元 d_j，Huber），要求 σ_ψ≤2°（GVINS 式25-26 顺序；InGVIO 25 历元/0.4 m/s）。I3 平移/钟/ISB：固定 ψ，窗口原始伪距批量 LS（共享 3 个平移未知数，比逐历元 SPP 稳健），迭代 w 剔除，验后 σ̂≤5 m。I4 联合 GN 精化 (ψ,tᴺ,b_j,d_j,ISB)。I5 插入：r₀＝窗内稳健均值（或配置），oᵂ＝当前 aᵂ，P_G=4Σ（补偿「LIVO 轨迹当已知」与交叉协方差置零），钟外推到 t_k，杆臂/td 用先验，进 WARMUP（同 RECOVERY 斜坡）。失败则滑窗继续并记原因。REINIT：删帧与钟状态重走 I1-I5，全局轨迹在事件点允许不连续。

## 7 更新调度
```
sync(LIO) → processImu():
  Process2（传播、F_total、桥接快照、去畸变）→ gravityAlignment()（关）
  ★ gnss.onPropagated(LidarMeasures,_state,*p_imu)      // LIVMapper.cpp:254 与 :256 之间
      增广块传播 → 模式机/初始化 → 已到达且 τ_g≤t_k 的历元按 τ_g 升序：
      预处理 → L1 → 周跳 → L3 → 退化下限 → L2 FDE → 联合更新(Joseph+对称化+步长+验后回滚) → 克隆 → 日志
      → 宿主均值/协方差写回 _state
  state_propagat=_state; voxelmap_manager->state_=_state  // 上游 :256-257，LIO 先验自动含 GNSS
handleLIO: StateEstimation(:370) → _state=...(:371) → ★onLivoUpdated(LIO) → evo/odom(:384-409) → ★publishGlobal(LIO) → 地图插入(:413-424，已含 GNSS 修正)
sync(VIO) → processImu（零传播）→ handleVIO: processFrame(:305) → ★onLivoUpdated(VIO) → ★publishGlobal(VIO)
```
放在 LIO 前（code-timeline B1）的理由：退化方向修正在同周期进 LIO 先验与地图（FAST-LIVGO 未讨论的地图一致性）；不改 sync 簿记；LIO 无点提前返回（:343-347）时 GNSS 照常。code-update §8 担心的「大新息带偏关联」：健康时对 pᵂ 增益极小，退化/告警有步长上限，再捕获大新息进帧而非位姿。同周期多历元串行（克隆链自然衔接）；τ_g>t_k 留待下周期。ONLY_LIO 模式同样适用。

## 8 异常处理
| 情形 | 处理 |
|---|---|
| 星历缺失/过期/不健康 | 该星不用，计数 |
| IODE 切换 | TDCP 两端用同一较新星历 |
| BDS-3 GEO C59-C62 | RTKLIB-EX MAXPRNCMP=50 静默丢星（rtklib.h:211）⇒ 补丁改 63 |
| 钟跳 | §5.5 |
| 主机时间台阶 | 离线分段；在线 td 方差重置 |
| 周跳/半周 | 断链重建克隆 |
| NaN/非正定 | 回滚本次；特征值截断修复；事件 |
| P⁻ 的 LDLT 失败 | 特征分解伪逆（下限 1e-12），事件 |
| 历元倒退/重复/过旧 | 丢弃计数 |
| 多数被剔 | 本类本历元不更新 |
| σ_ψ>30° 或 σ_t>50 m | REINIT |
| LIO/VIO 提前返回 | 对应钩子不调用 |

## 9 中断与恢复
模式机：OFF/OBSERVE（只算不写）/UNINIT→COLLECT→WARMUP→NOMINAL⇄OUTAGE→RECOVERY→NOMINAL，叠加标志 DEGENERATE、LIVO_ALARM，REINIT 可从任意态进入，全部写 events.log。
- GNSS-denial 全程：初始化条件永不满足 ⇒ 增广块为空 ⇒ 宿主与原版逐位一致；只输出局部轨迹并给出未初始化原因。
- 隧道：进入时口部 NLOS 由 L2 剔除，NOMINAL 下 1-3 星照用；全无 ⇒ OUTAGE，帧按距离、钟按时间传播（100 m 隧道水平 σ_t≈5 cm＋航向项）。
- 夜间：GNSS 与光照无关；退化指数用 LIO+VIO 合并信息，VIO 近零不会误入退化；GNSS 质量只由环境决定。
- 恢复（不让跳变拉坏地图）：①settle 1 s 只收集（跟踪环锁定、C/N0 回升），需≥4 星；②ramp 5 s：R×κ(t)，κ 从 9 线性降到 1；伪距需连续跟踪≥10 历元，TDCP 需该星连续 2 个干净历元；③L2 全程生效，帧不确定度已按距离增长，大新息被接受但进帧状态；④健康 LIVO 下修正进 ψ/tᴺ/钟，地图不动；退化方向受步长上限逐历元缓释；⑤呈现层 /gnss/global_odometry_smooth 限速（平移≤0.2 m/s、航向≤2°/s），评测用未平滑估计；⑥中断>10 s 或不确定度超阈 ⇒ REINIT。

## 10 FAST-LIVGO 表 II 三现象
原值为未审计引用（单次运行、未说明 LiDAR/相机/对齐；Dark01/02 被其归为 GNSS Denied，而 M3DGR 归为 Night，README:349-350；未评 GNSS-denial01/02）。
机制（推断）：其帧 ^E R_G、^E p_G 由 LS 一次解出不进状态（式2-3），TDCP 固定锚（式11），伪距只做 RAIM，多普勒与 TDCP 同用 ⇒ GNSS 直接、且过度自信地作用于位姿。
- (1) 开阔全降：干净载波下固定锚 TDCP＋多普勒逐历元压住航向/尺度/Z 漂移；过度自信在干净数据下无害。
- (2) 夜间高于 FAST-LIVO2（0.34/0.27 vs 0.31/0.18）：LIVO 本身漂移极小；遮挡下通过 χ² 的残余多路径载波率/NLOS 多普勒偏差以 mm 级精度作用在位姿并被固定锚继承，把好位姿拉偏。附加推断：夜间 VIO 光度信息近零，若 λ_min 取 LIVO Hessian 则更易进 Mode B 全收——可用 D0 的昼夜 LIO 与 LIO+VIO 信息谱对比检验。
- (3) 去剔除即发散：周跳（19 cm/周）与 NLOS 多普勒以 mm/cm 精度直接进位姿，位姿一步偏 dm-m，点到面关联失败，IEKF 回不到地图；与论文 V-A2 自述「ESIKF 比 LIO-SAM/LIGO 更脆弱」一致。
本设计在三情形下（预测，§17.6）：(1) ψ 由多普勒/TDCP 定到<0.2°，tᴺ 由 TDCP 跟踪平移漂移、伪距锚定 ⇒ 全局 ATE 低于原版（≥4/5 条），幅度可能小于其未审计数字；(2) 健康 LIVO 下 GNSS 只动帧，帧变化速率受漂移预算 q 约束 ⇒ 伤害有界，局部轨迹等效；(3) A7 去剔除：离群进帧/钟，pᵂ 增益极小 ⇒ 全局变差但**不发散**，局部 ATE 与全量等效——结构上把地图与 GNSS 离群隔离，只有退化方向需要剔除保护地图。
探针 A8（按论文描述复刻，非其代码）：帧初始化后固定；TDCP 固定锚；多普勒同用；伪距仅 RAIM 预筛（C/N0<35、el<15° 硬掩码＋SPP 残差）；双模：自算 LIVO 信息 λ̄_min≥λ_th 时逐星 χ²（式14），否则全收；A8-norej 始终全收。预测：A8 开阔改善；Dark 至少一条劣于原版；A8-norej 在 Dark01/02 至少一条发散。若 A8-norej 不发散 ⇒ 解释(3)被证伪，如实报告。

## 11 底座与 I/O
### 11.1 RTKLIB-EX vendoring（third_party/rtklib，BSD-2，保留 license.txt）
锁 v2.5.1；只编 src/*.c 与 rcv/ublox.c 等依赖，静态库；不用 lib/（IERS/SOFA/BLAS 另有许可）；不 add_subdirectory 上游 CMake。宏 PUBLIC 导出 -DENAGLO -DENAQZS -DENACMP -DENAGAL -DENAIRN -DNFREQ=3 -DNEXOBS=3（obsd_t/nav_t 布局依赖，rtklib.h:159,164）。补丁（PATCHES.md）：P1 MAXPRNCMP 50→63；P2 pntpos.c 导出包装 rtkx_prange()（包 static prange :110），与 pntpos() 逐位回归。rtklib.h 只在一个 pimpl 适配 .cpp 中包含（无前缀宏 rtklib.h:86-96,253）。全局可变状态（eph_sel ephemeris.c:109、leaps rtkcmn.c:171、timeoffset_ :1783）⇒ GNSS 单线程。用到：satpos/satposs（ephemeris.c:737/781）、geodist/satazel（rtkcmn.c:3612/3631）、ionmodel/tropmodel（:3688/3791）、ionocorr/tropcorr（pntpos.c:204/251）、sat2freq/satno（:766/398）、gpst2time/time2gpst/gpst2utc/bdt2gpst、xyz2enu/ecef2pos/pos2ecef、pntpos（仅回归与参照行）、input_ubx（rcv/ublox.c:1437，实时 UBX）。自写：量测模型、增广块、钩子、FDE、完好性、初始化、GNSS-only 稳健 SPP/多普勒 LS（RTKLIB estpos/resdop 为 static 且 RAIM 仅单故障）。
### 11.2 读取 gnss_comm 消息而不链接 gnss_comm
(A) C++ 定义驱动反序列化器（gnss_io，BSD-3）：ROS1 bag 每个 connection 头带完整 message_definition 与 md5（ShapeShifter 实时订阅同样可取）。运行时解析定义文本（主类型＋MSG: 分隔依赖），按 ROS1 线格式（小端、变长数组/字符串 uint32 长度前缀、嵌套内联、time=2×uint32）解码为通用字段树，再按字段名抽取到 gnss_core 结构；缺字段/类型不符明确报错。回放用 rosbag::MessageInstance 原始字节，实时用 topic_tools::ShapeShifter。
(B) Python 导出器 tools/gnss_bag_export.py（BSD-3，rosbags[Apache-2.0]，同用内嵌定义）：出逐星逐信号 CSV（含记录时间）、星历 CSV、iono/PVT/time_pulse JSON、RINEX 3.04 OBS/NAV（供 rnx2rtkp 独立参照行）。(A)(B) 在同片段 CSV 逐字节一致（T3）。
数据语义（来自定义文本与字段含义）：sat 编号 GPS 1-32、GLO 33-59、GAL 60-97、BDS 98-160 → (sys,prn) → RTKLIB satno；A 为半长轴；toe/toc/ttr 为 GPST；tgd0/1：GPS TGD、GAL BGD、BDS TGD1/TGD2；status：bit0 伪距有效、bit1 载波有效、bit2 半周有效、bit3 半周已扣；信号按「频率定频段＋code 字段」映射 RTKLIB CODE_*（BDS B1I→CODE_L2I，决定 prange 取 TGD_B1I，pntpos.c:189-193），未知组合拒收。
许可证理由（工程判断，非法律意见）：GPL-3.0 约束 gnss_comm 的代码（及可争议的 .msg 文本）；我们不复制、不包含、不链接其任何源/头/.msg，字段布局在运行时从数据文件自身读取，是对数据格式的互操作读取，不构成衍生作品；转换工具是独立宽松许可程序 ⇒ GPLv2-only 与 GPLv3 不兼容（FSF FAQ #v2v3Compatibility）的问题不出现。
### 11.3 实时与 ROS2
ROS1：ShapeShifter 订阅 /ublox_driver/*（rosbag play 复核）；真机：原始字节流（串口直读或 GnssRawStream{header,format,uint8[] data}）＋RTKLIB input_ubx 进程内解码，不依赖 GPL 驱动。gnss_core、gnss_io、宿主适配器均无 ROS 头 ⇒ ROS2 只重写参数/订阅/发布胶水（UBX 或 CSV；gnss_comm 的 ROS2 .msg 有 7 个字段名违反 rosidl，不走）。
### 11.4 仓库与许可证
fork 根 GPL-2.0-only（LICENSE 原样；package.xml:15 的 BSD 改 GPL-2.0-only）；src/replay、src/gnss_adapter：GPL-2.0-only；gnss_core、gnss_io、tools：BSD-3；third_party/rtklib：BSD-2；LICENSES/ 放全文，源文件 SPDX 头。只发源码，不发二进制/镜像（上游已依赖 GPLv3 的 rpg_vikit）；库内不出现 GPL-3.0/LGPL-3.0/Apache-2.0 源码或 .msg；evo 不导入，仅测试中作外部命令交叉校验。
### 11.5 构建
docker/Dockerfile.noetic：ros:noetic（dockerproxy.net）＋pcl_ros/cv_bridge/image_transport/tf/eigen_conversions/rosbag/topic_tools/diagnostic_msgs＋Sophus a621ff2＋xuankuzcr/rpg_vikit@6c886c8＋gtest；livox 消息头上游自带（include/livox_ros_driver）。CMake 选项：FASTLIVO_WITH_GNSS；FASTLIVO_DETERMINISTIC：不定义 MP_EN（去 omp_set_num_threads(4) 与并行 for，voxel_map.cpp:658-660、vio.cpp:1552-1555，后者 float 规约是不确定性来源）、-DEIGEN_DONT_PARALLELIZE、-O3 -march=x86-64-v3 -ffp-contract=off（替代 -march=native）。原版臂与本系统用同一组构建选项。

## 12 代码改动清单（上游 @0d2c034，只加不改）
| 文件 | 位置 | 改动 | 对上游算术 |
|---|---|---|---|
| include/IMU_Processing.h | :53 后 | struct BridgeSample{t,R,p,v,w}；bridge_traj_、F_total_(19×19)、bridge_t0_/t1_、bridge_valid_ | 无 |
| src/IMU_Processing.cpp UndistortPcl | :317 后、:322 前 | 清空快照、F_total_=I、压入起点样本 {prop_beg_time,R_imu,pos_imu,vel_imu,angvel_last} | 无 |
| 同上 | :401 后 | F_total_=F_x*F_total_ | 无 |
| 同上 | :430 后 | 压入 {prop_beg_time+offs_t,R_imu,pos_imu,vel_imu,angvel_avr} | 无 |
| 同上 | :435 后 | bridge_t1_=prop_end_time; bridge_valid_=true | 无 |
| include/LIVMapper.h | :22 后、:160 后 | 包含 gnss_adapter/livo_gnss_glue.h；成员 unique_ptr<LivoGnssGlue> gnss_ | 无 |
| src/LIVMapper.cpp readParameters | :116 后 | 构造 glue，loadParams(nh) | 无 |
| initializeComponents | :162 后 | configure(extR,extT,slam_mode_,imu_en,p_imu) | 无 |
| initializeSubscribersAndPublishers | :216 后 | initRos(nh)（ShapeShifter 订阅、发布器） | 无 |
| processImu | :254 与 :256 间 | onPropagated(LidarMeasures,_state,*p_imu) | 关/未初始化为空操作 |
| handleVIO | :305 后；:333 前 | onLivoUpdated(state_propagat,_state,VIO)；publishGlobal(VIO) | 只读 |
| handleLIO | :371 后；:409 后 | onLivoUpdated(...,LIO)；publishGlobal(LIO) | 只读 |
| run | :553 前 | finalize()（刷日志） | 无 |
| CMakeLists.txt | 构建段 | 选项、确定性旗标、子目录、回放可执行、测试 | 构建 |
| package.xml | :15 等 | GPL-2.0-only；依赖 rosbag/topic_tools/diagnostic_msgs | 无 |
不改：common_lib.h（DIM_STATE、StatesGroup）、voxel_map.{h,cpp}、vio.{h,cpp}、preprocess、frame、visual_point。
新增：src/gnss_adapter/livo_host_view（StatesGroup↔HostStateView 索引 θ0 p3 v7 bg10、读快照、写回）、livo_gnss_glue（参数、订阅→gnss_io→GnssManager、钩子转发、话题/TF）；gnss_core（time/TimeMap、ephemeris_store、sat_compute[唯一含 rtklib.h]、quality、slip_detector、gnss_only、aug_block[propagate/conditionOnHostUpdate/insertClone/removeClone/repivot]、models、update[fde/jointUpdate]、integrity、initializer、gnss_manager、diagnostics）；gnss_io（ros1_msgdef、ros1_decoder、gnss_comm_mapping、csv_io、ubx_stream）；src/replay（replay_main、bag_feeder、latency_emulator、recorder）；tools（m3dgr_slim_bag、gnss_bag_export、timebase_calib、gt_audit、m3dgr_calib、eval_ate、run_matrix、report）；test。

## 13 确定性离线回放
- fastlivo_replay --bag --config --out --arm A0..A8 --seed k|none --order record|header。每作业私有 roscore（独立端口与 ROS_LOG_DIR）、rosparam load 到 /laserMapping（vikit camera_loader 从此读，LIVMapper.cpp:128），节点名 laserMapping；调用 initializeSubscribersAndPublishers 但永不 spin（回调与 imu_prop_timer :215 均不被触发）；每作业独立容器/挂载 Log/（上游写 ROOT_DIR/Log，CMakeLists.txt:40，并行会互相覆盖）。
- 喂数：rosbag::View 按记录时间（＝rosbag play 顺序）；CustomMsg→livox_pcl_cbk；Imu→imu_cbk；CompressedImage→cv::imdecode 为 bgr8 Image（保留头）→img_cbk；GNSS→gnss_io→glue（到达时间＝记录时间）。每条后执行 run()（:534-554）主体去掉 spinOnce/sleep：while(sync_packages){handleFirstFrame(); processImu(); stateEstimationAndMapping(); recorder.onStep();}。
- 确定性条件：§11.5 构建；单线程；不依赖墙钟（omp_get_wtime 只打印）；容器以 VOXEL_LOCATION/int 为键（已核，无指针键）。验收 T20。
- 扰动种子：虚拟时钟延迟仿真——每周期虚拟耗时 c_k~Gamma(均值＝实时剖析周期耗时中位，CV=0.3，seed)，消息记录时间≤虚拟时钟才可用（复现实时下缓冲内容改变 sync 决策这一不确定性来源）；seed=none 为无限快；备选 start_offset（跳过 k×0.1 s）。
- 输出：traj_local_{lio,vio}.tum（W，时间＝last_lio_update_time）、traj_global_{lio,vio}.tum（N）、global_llh.csv、livo_info.csv、GNSS 日志、run_manifest.json（git 哈希、构建旗标、配置/bag sha256、种子、臂）。
- A0＝标签 harness-pristine（0d2c034＋src/replay＋确定性选项，估计器与 LIVMapper 零改动；LIVMapper 全员 public，LIVMapper.h:26-185，记录器只读）。

## 14 配置
M3DGR 主配置：/livox/avia/{lidar,imu}（lidar_type 1）；extrinsic_T=[0.04165,0.02326,−0.0284]、R=I（calibration §1.6，与上游 avia.yaml 一致）；Rcl/Pcl＝avia→camera（§1.3）；D435i PINHOLE 640×480，fx 607.80 fy 607.84 cx 328.80 cy 245.53，无畸变；其余 LIVO 参数取 M3DGR baseline_systems/Fast_LIVO2_M3DGR 原样，所有臂共用（同一数据集一套参数）。杆臂：GNSS antenna→avia 的 T＝天线在 Avia 系坐标 [−0.358,0.086,0.429]（calibration §1.7）⇒ lᴵ=[−0.31635,0.10926,0.4006]；MID-360 链由 m3dgr_calib.py 推导（推断约 [−0.284,0.001,−0.211]，须复算）。
```yaml
gnss:
  enable: true
  mode: tc   # off|observe|lc|tc|livgo_probe|livgo_probe_norej
  input: {source: ros1, topic_meas: /ublox_driver/range_meas, topic_ephem: /ublox_driver/ephem, topic_glo_ephem: /ublox_driver/glo_ephem, topic_iono: /ublox_driver/iono_params, topic_pvt: /ublox_driver/receiver_pvt, topic_time_pulse: /ublox_driver/time_pulse_info, nav_rinex_supplement: ''}
  systems: {gps: true, bds: true, gal: true, glo: true, glo_pseudorange: false}
  frames: {lever_arm_imu: [-0.31635, 0.10926, 0.4006], lever_arm_sigma: [0.05, 0.05, 0.05], estimate_lever_arm: true, origin_ecef: [], pivot_distance_m: 100.0, gnss_updates_gravity: false}
  time: {leap_seconds: 18, offset_s: 0.0, skew: 0.0, ref_gpst_s: 0.0, segments: [], estimate_td: true, td_sigma0_s: 0.010, td_rw_s2_per_s: 1.0e-8, max_bridge_s: 0.30, snapshot_ring: 4}
  clock: {S_b: 0.1, S_d: 0.1, S_isb: 1.0e-4, jump_threshold_m: 1000.0}
  frame_process: {q_yaw_per_m: 1.2e-7, q_th_per_m: 2.5e-5, q_tv_per_m: 1.0e-4, q_yaw_per_s: 1.0e-10, q_t_per_s: 1.0e-8}
  noise: {cn0_ref: 45.0, psr: {a: 0.3, b: 0.3, c: 1.0, period_s: 1.0, corr_time_s: 5.0}, dopp: {a: 0.02, b: 0.02, c: 0.05, corr_time_s: 0.2}, cp: {a: 0.002, b: 0.002, c: 0.003}, sys_factor: {G: 1.0, E: 1.0, C: 1.0, C_GEO: 2.0, R: 1.5}, receiver_std_floor: true, tdcp_with_doppler: false}
  masks: {el_deg: 10.0, cn0_psr: 30.0, cn0_dopp: 25.0, cn0_cp: 28.0, track_psr: 10, track_cp: 3, ura_max_m: 10.0, eph_age_s: {G: 7200, E: 7200, C: 3600, R: 1800}}
  rejection: {enable: true, alpha_single: 1.0e-3, alpha_joint: 1.0e-3, max_reject_frac: 0.5, min_meas: {psr: 1, dopp: 1, tdcp: 2}, posterior_check: true}
  slip: {lli: true, need_half_cycle: true, max_gap_s: 1.5, dopp_tol_cyc: 0.5, innov_abs_m: 0.05, gf_tol_m: 0.05, cmc: false}
  integrity:
    velocity: {enable: true, alpha: 1.0e-3, min_err_mps: 0.2, n_consec: 5, inflate_v: 0.5, inflate_p: 0.3, hold_s: 5.0}
    heading: {enable: true, min_speed: 0.5, n_consec: 20, min_err_deg: 5.0, inflate_deg: 5.0}
    position: {enable: true, min_sats: 8, min_sys: 2, pdop_max: 3.0, cn0_mean_min: 40.0, persist_s: 10.0, k_sigma: 5.0}
    degeneracy: {enable: true, lambda_enter: 400.0, lambda_exit: 800.0, window: 10, sigma_floor_m: 0.05}
    step_limit: {pos_m: 0.05, rot_deg: 0.2, pos_alarm_m: 0.20, rot_alarm_deg: 1.0}
    gating_host_sigma_floor: {pos_m: 0.0, vel_mps: 0.0}
  init: {min_livo_frames: 50, min_sats: 6, pdop_max: 4.0, min_speed: 0.3, min_epochs: 30, min_disp_m: 5.0, yaw_sigma_max_deg: 2.0, cov_inflation: 4.0, nlos_cn0_margin_db: 8.0}
  outage: {outage_s: 1.0, long_outage_s: 10.0, settle_s: 1.0, ramp_s: 5.0, ramp_var_factor0: 9.0, min_sats_recover: 4, reinit_yaw_sigma_deg: 30.0, reinit_t_sigma_m: 50.0}
  lc: {use_velocity: true, pos_sigma_scale: 1.0, vel_var_factor: 2.0}
  livgo_probe: {cn0_min: 35.0, el_min_deg: 15.0, lambda_th: 400.0, window: 10}
  output: {frame_id: enu, smooth_trans_mps: 0.2, smooth_yaw_dps: 2.0}
  diag: {dir: Log/gnss, epoch_csv: true, sat_csv: true, livo_info_csv: true}
```
标定协议（一次性、统计量驱动，禁止在评测序列调参）：开发序列 Occlusion03/04、Dynamic03/04；q 由原版臂 W 轨迹对 GT 在距离滞后 10/50/100/200 m 的相对误差方差增长斜率定；各类噪声一个标量缩放使 NIS/自由度∈[0.8,1.2]；若 GT 差分速度给出 LIVO 速度 NEES>3×自由度，才设 gating_host_sigma_floor.vel；配置 sha256 写入预注册文档后冻结。

## 15 日志与诊断
Log/gnss/&lt;seq&gt;/：epochs.csv（t_gpst、τ、t_k、Δ、到达延迟、模式/标志、每系统星数、每类候选/接受/剔除按原因 mask/track/eph/slip_D1-D6/w/nis/majority/l3/step、各类 NIS 与自由度、κ_step、ψ、tᴺ、b、d、ISB、td、杆臂及 σ、λ₁₋₃ 与 u₁、宿主 √diag、share_host、‖δp_H‖、δψ、‖δtᴺ‖、GNSS-only 解与检验量、耗时）；sats.csv（每星：el、az、C/N0、可用性、σ、新息、w、决策与原因、周跳检测器、NLOS 嫌疑）；events.log（模式转移、钟跳、时间台阶、REINIT、重锚、回滚、迟到丢弃、ALARM 起止）；livo_info.csv（D0/D1）。话题：/gnss/global_odometry、/gnss/global_odometry_smooth、/gnss/fused_fix、/gnss/spp_fix、/gnss/diagnostics、TF enu→camera_init。tools/report.py：决策着色天空图、NIS 时间线与 χ² 带、模式/退化时间线、帧状态、信息分配、轨迹叠加。

## 16 测试（静默自检，未全过不产出评测数字）
单元：T1 时间（往返、闰秒 18、BDT−GPST=−14 s、周翻转）；T2 反序列化（手工定义＋字节逐位一致、畸形缓冲报错不崩）；T3 C++/Python 导出 CSV 逐字节一致；T4 星历直填 eph_t 与「RINEX NAV→readrnx」卫星位置/钟逐位一致（可选对 IGS SP3/CLK：G/E<3 m、C MEO/IGSO<5 m、C GEO/R<10 m）；T5 包装 SPP 与 pntpos()<1 mm，开阔历元与接收机 PVT 水平差中位<3 m；T6 全部量测对全部块（含桥接、枢轴、克隆、td）有限差分相对误差<1e-6；T7 钩子 vs 显式联合 KF 差<1e-10；T8 F_total 连乘一致；T9 克隆版 vs 完整延迟状态滤波，1000 次 MC NEES 均值在维数±10%；T10 注入单/多 NLOS、1 ms 钟跳、1/2/10 周、半周 ⇒ ≥5σ 故障检出≥99%，虚警在 α±50%；T11 一致模拟 NIS∈[0.9,1.1]×自由度；T12 1e5 次随机更新后对称误差<1e-12、最小特征值≥−1e-12、无 NaN；T13 步长约束；T14 重锚全局位置变化<1e-9 m；T15 时基工具恢复<2 ms/1 ppm；T16 真实载波注入 k 周 ⇒ 1 周检出 100%、虚警<0.1%。
集成：T20 两次运行 sha256 相同；T21 gnss.enable=false 与原版臂 sha256 相同；T22 剔除 GNSS 话题后开启 GNSS 与 T21 相同；T23 observe 与 T21 相同；T24 Outdoor01 前 120 s：运动后 30 s 内初始化、NIS 在带内、每历元耗时中位<5 ms/p99<10 ms；T25 Outdoor04 删 30/60/120 s GNSS ⇒ 再捕获修正≤3×预测 σ、非退化段局部轨迹与 GNSS 关闭差<1 cm、无 LIO 失效；T26 注入周跳与方位扇区 30% 伪距 +5-40 m ⇒ 周跳检出≥99%、NLOS 剔除≥90%；T27 ±50 ms 时偏注入可告警并由工具恢复；T28 A2/A7/A8/A8-norej 端到端可跑。
数据自检（每序列评测前）：载波/多普勒符号与单位；杆臂符号（多普勒新息对 ω×l 项回归系数≈0，≈−2 即符号反）；星历覆盖时间线；到达延迟分布；GT 审计。

## 17 评测协议
- 17.1 数据：Varying-illu03/04/05、Outdoor01/04、Dark01/02＋GNSS-denial01/02，原包约 82.3 GB；开发序列约 16.8 GB；下载需同意，落本机外置 SSD（根分区仅剩 1.6 GB）。对照 checksums.txt；生成 slim 包（只留 /livox/avia/{lidar,imu}、/camera/color/image_raw/compressed、/ublox_driver/*，原始字节与记录时间原样），推断约原包 25%，实测为准，传远端 /data（约 60 GB 可用）。
- 17.2 GT 审计（算法无关、所有臂同用）：度/米与 Sim(3) 尺度≈1；若为起点 ENU，全局轨迹对 GT 的对齐旋转应近似纯平移＋小航向（诊断 issue #29/#38 歧义；GCJ-02 局部畸变对齐后可忽略，推断）；GT 时基互相关，>20 ms 补偿；可信掩码：GT 跳变（加速度>3 m/s²）或间隔>0.5 s 前后 2 s、本车 PVT h_acc>2 m 时段±2 s；GNSS-denial 用 M3DGR ArUco_evaluate.py 原样＋首尾误差。
- 17.3 臂：A0 原版；A2 松耦合 PVT；A3 紧耦合全量；A4 去伪距；A5 去多普勒（仅用于周跳）；A6 去 TDCP（多普勒不再互斥）；A7 去剔除（关 L2/L3 与 D4-D6，保留 L1 掩码与 LLI，对应其「融合全部预筛观测」）；A8/A8-norej 探针；参照行：rnx2rtkp SPP、接收机 PVT；FAST-LIVGO 表 II 单列注明未审计。
- 17.4 指标：主指标 LIO 阶段输出（与上游 evo 口径同，LIVMapper.cpp:384-405）；GNSS 臂用 N 系全局轨迹，A0 用 W 轨迹；仅位置、时间关联≤10 ms、可信 GT 上 SE(3) Umeyama ATE RMSE；杆臂补偿 ATE（对齐中联合估 GT 天线体坐标杆臂，9 自由度 GN，各臂各估，差异大告警）；次指标：VIO 阶段 ATE、局部 W 轨迹 ATE、10/50 m RPE、Sim(3) 尺度、全局位置 NEES、发散计数（局部 ATE>5×A0 同种子、LIO 有效点<10% 持续 5 s 或 NaN）、实时耗时。自实现 ATE 与 evo_ape（外部命令）差<1e-6。
- 17.5 运行与统计：9 臂×9 序列×8 种子；A0 在 Outdoor01 另做 5 次实时运行以校准扰动模型。同种子成对差，逐序列报 8 种子胜数、中位与自助 95% CI（B=10000）；跨序列只报计数（n=5 Wilcoxon 最小 p=0.0625，不作显著性声明）；不伤害等效：Dark01/02、GNSS-denial01/02 上配对差 95% 上界<δ=max(0.05 m,0.5×A0 种子 IQR)；实时复核：Outdoor01、Dark01 上 A0/A3/A7 各 3 次（独占、先做争用筛查）。文字只报胜率/计数，精确值只进表。
- 17.6 预注册预测：P1 开阔 A3 优于 A0 ≥4/5（每条≥7/8 种子）；P2 Dark 全局与局部等效；P3 GNSS-denial 等效，未初始化则逐位相同；P4 A7 全不发散、局部与 A3 等效、Dark 全局劣于 A3；P5 A8 开阔改善≥4/5、Dark 至少一条劣于 A0、A8-norej 至少一条 Dark 发散；P6 A6 开阔劣于 A3；A4 在最长 Varying-illu03 劣于 A3、短序列等效；A5 周跳多的序列劣于 A3；P7 A3 不劣于 A2，差距最大处在 PVT 退化时段；M1 非退化段 share_host 中位<1%、运动后 30 s 内 σ_ψ<0.2°。任一失败如实报告为该机制证伪，不在评测序列改参重跑。
- 17.7 资源：单线程回放约 1× 实时（推断），全矩阵约 100 CPU·h，远端 12 核/62 GB 约 10 并行 ⇒ 约 11 h；远端 v0.6 链约 09-26 13:00-16:30 结束前不上负载；确定性回放不受争用影响，实时复核必须独占。

## 18 已知局限
1) 软件同步：残余时偏在加减速时影响多普勒/TDCP，td 弱可观；LiDAR 时基未知。2) GT 仅位置、遮挡段不可信、坐标系歧义、GT 天线杆臂未知、GNSS-denial 只有端点 ⇒ 小改进可能落在等效界内。3) 帧级设计不重锚地图：局部地图自洽但不与 ENU 一致；回访时帧需重新收敛；按插入时帧变换导出的全局点云回访处可能双层；不做 LIGO 式样条重插（约束 3）。4) 再捕获/REINIT 时全局输出可跳变（有平滑话题）。5) 伪距时间相关用方差膨胀近似；TDCP MA(1) 相关、桥接相关、LOS 位置导数、Λ(gᵂ) 不确定度被忽略。6) 单频量测，L2 只做周跳；无 PPP/RTK（M3DGR 无基准站）。7) 城市峡谷只能在 M3DGR 上注入验证。8) 上游缺陷保留以维持 A0 等价：(I−G)P 不对称、点协方差缺 R（LIVMapper.cpp:420-421；voxel_map.cpp:387）、last_prop_end_time 未初始化（IMU_Processing.h:80）、偏置噪声硬编码（LIVMapper.cpp:153-154）、Livox 不用 lidar_time_offset（:744）、evo 写在 VIO 前。9) 确定性构建较慢，实时性能单测。10) 许可证结论为工程判断；上游依赖 GPLv3 的 vikit ⇒ 只发源码。11) 方法首创已被 FAST-LIVGO 占，本仓库定位是许可证干净的完整实现与可复现评测。

## 19 实施顺序（一次成型，每步以静默自检收口）
①回放外壳＋记录器（harness-pristine）→T20，跑 D0-b（分析先行）；②gnss_io、导出器、时间库、星历库、卫星计算→T1-T5 与数据自检；③量测模型、增广块、钩子、上游四处只加代码→T6-T8、T12-T14、T21-T23；④初始化、FDE、周跳、钟、L3、退化、步长、模式机、克隆→T9-T11、T16、T24-T27；⑤A2、A8、输出/话题/报告→T28；⑥时基标定、GT 审计、开发序列标定、冻结配置与预注册→评测一轮。
