> Historical research record (through 2026-10-03). Paths are illustrative; archived runs are not bundled. For current status and executable entry points see [the repository README](../../README.md). Current release docs/LICENSING.md supersedes historical license notes. This record does not authorize new experiments or assert current release validation.

# FAST-LIVO2 × 原始 GNSS 紧耦合 — 冻结设计(DESIGN.md)

基底 hku-mars/FAST-LIVO2 @0d2c034(ROS1 Noetic),行号指 `/workspace/fast_livo2_ref`。RTKLIB-EX 锁 tag v2.5.1(行号按 main@06e8644 复核,实现时重核)。gnss_comm @a2035fe 只读理解格式。凡未实测者标「推断」。本文是三份设计(D1 一致性/D2 鲁棒性/D3 集成)与三份审查(R1/R2/R3)的综合定稿,取舍理由见 resolved_issues。落盘副本:`/workspace/fastlivo2_gnss_scout_20260924/DESIGN.md`。

## 0 结论先行
1. **架构:宿主 19 维 ESIKF 一字不改,GNSS 全部状态放增广块 x_G,经三个只读钩子与宿主构成联合高斯**(D2/D3;R1 手推与 D3 hook.cpp 1e-15 实验确认与联合 IEKF 逐迭代精确等价;D3 pad.cpp/R3 复跑证明扩 DIM_STATE 不可能逐位等于上游)。common_lib.h、voxel_map.*、vio.* 零改动;GNSS 关闭时逐字节等于参考提交 R。
2. **「地图当真值」的回答:LIVO 漂移住在配准状态 ξ=(φ,ψ,t) 里,按行驶路程随机游走、旋转增量绕当前枢轴施加。** 一致的增益自动把伪距/多普勒/TDCP 的修正主要分给 ξ、钟、速度/偏置;p^W 只在 LIO 退化方向分到可观份额(份额 ~1e-4,三方一致)。倾斜主体取自已有状态 g^W 经无偏航调平 C_ref(g),另加 2 维倾斜残差 φ 承载地图弯曲(D1)。
3. **TDCP 用 3 维随机克隆 + 接收机钟差增量 Householder 投影消去 + 不重叠配对**(D1),相邻 TDCP 噪声严格独立;不用固定锚(§6 标量推导:固定锚 NEES∝历元数)。
4. **剔除全部是 0/1 成员资格**:L0 掩码 → L1 自写 GNSS-only 稳健 LS(RTKLIB-EX valsol 的 χ² `return 0` 已被注释,pntpos.c:384,raim_fde 只在 estpos 失败时进入 :689,故不能当裁判)→ L2 钟跳整毫秒判定 → L3 周跳 → L4 新息空间 Baarda w 检验 + 历元 NIS + 迭代 FDE(D2)→ L5 验后回滚 → L6 锁死重锚。EKF 内不做任何连续调权、不做数据依赖的协方差膨胀。
5. **挂点**:`processImu()` 中 Process2 与 gravityAlignment 之后、`state_propagat=_state` 之前(LIVMapper.cpp:254→:256),仅 LIO 周期;IMU 桥接样本在 UndistortPcl 传播循环内自记(D2),F_total 在 IMU_Processing.cpp:401 后累积。
6. **底座/I-O**:RTKLIB-EX v2.5.1 BSD-2 vendoring(补丁 P1 MAXPRNCMP→63、P2 导出 prange/varerr/gettgd 与 estpos/estvel 全量 x/Q);GNSS 核心编成 hidden-visibility 的 SHARED 并 `--exclude-libs`;M3DGR 的 gnss_comm 消息用「bag 内嵌 message_definition 驱动的运行时 ROS1 线格式解码器」读取(回放经 rosbag::MessageInstance,在线经 topic_tools::ShapeShifter),布局与内置期望表逐项比对、不一致即拒收;仓库不含 gnss_comm 任何代码或 .msg。
7. **确定性回放与评测**:记录时间序直读 bag、每条消息后同步 `while(processOnce()){}`;eval profile 去 MP_EN、全 TU `-ffp-contract=off`、三包同 -march、EIGEN_DONT_PARALLELIZE;参考提交 R 零初始化 IMU_Processing.h:60/:80/:81;四道逐位门(确定性/零通道/旁路/无数据,hexfloat state_trace);扰动种子=起点偏移 start_offset(命中实时方差的真实来源:启动期跨话题次序);每臂每序列 10 种子成对 Wilcoxon + Holm。

## 1 帧、符号与约定(全文唯一约定)
变换写 T_{A←B}:p^A = R_{A←B} p^B + t_{A←B}。
| 记号 | 定义 |
|---|---|
| E | ECEF WGS-84(RTKLIB 内部;CGCS2000/PZ-90.11 差 cm 级忽略) |
| N | 固定 ENU,原点 r_0(ECEF)由初始化 SPP 定,之后永不改;R_{E←N} 三列=r_0 处东/北/天单位向量(RTKLIB xyz2enu 的转置) |
| W | FAST-LIVO2 世界系=初始化时 IMU 系,不水平、yaw 任意(IMU_Processing.cpp:144-145);`gravity_align_en` 保持 false(LIVMapper.cpp:77) |
| I / L / C | 宿主 R≡R_{W←I}(rot_end),p≡p^W_I,v≡v^W_I;extR/extT=T_{I←L}(LIVMapper.cpp:104-105,:645);Rcl/Pcl=T_{C←L}(vio.cpp:35-39) |
| A | 天线相位中心,杆臂 l^I(IMU 系,常量,σ_l 只进噪声) |
| o^W | 配准枢轴(W 系常量,§2.3 重锚) |
**配准**:R_{N←W} ≡ M = R_z(ψ)·Exp([φ_x,φ_y,0]ᵀ)·C(g);C(g) 为无偏航调平:`C(g)=Δ(C_ref·ĝ→−e_z)·C_ref`,Δ=两向量最小旋转(Eigen FromTwoVectors),C_ref 每步末用最终 g 刷新;一阶 `C(g+δg)≈Exp(ε)C(g)`,`ε=[e_z]×C(I−ĝĝᵀ)δg/‖g‖`,ε_z≡0(D1 §2.2 推导)。天线:a^W=p+R l;a^N=M(a^W−o^W)+t;a^E=r_0+R_{E←N}a^N;天线速度 w^N=M(v+R[ω]×l),ω=ω̃−b_g。全局位姿 R_{N←I}=MR,p^N_I=M(p−o^W)+t。
**扰动**:宿主右扰动 R=R̂Exp(δθ)(common_lib.h:170,:184),⊟=Log(R_bᵀR_a)(:197-198),其余加性;ψ 左扰动 R_z(ψ+δψ)=Exp(e_zδψ)R_z(ψ),取模 (−π,π];增广量加性。新息 ν=z−ĥ(x̂),H=∂h/∂δx,修正 δx=Kν(与 LIO `z=−dis` 同约定,voxel_map.cpp:457;不用 VIO 的「预测减观测」)。
**观测**(e_j=(r_s−r_A)/‖·‖ 接收机指向卫星,u_j=R_{E←N}ᵀe_j):
- 伪距:`P_corr = ρ + b + ι_s − c·δt_s + I + T + ε`,ρ=‖r_s(t_tx)−r_A‖+ω_e(x_s y_A−y_s x_A)/c(geodist rtkcmn.c:3612);TGD/BGD/dtaun 在观测端扣(prange pntpos.c:110-190);与 rescode `v=P−(r+dtr−c·dts+dion+dtrp)`(pntpos.c:335)一致。
- 多普勒:距离率 ρ̇=−λD(D 单位 Hz,接近为正;resdop pntpos.c:580-587、gnss_comm dopp_res、LIVGO 式12 三处同号):`−λD = e_jᵀ(v_s−ṙ_A) + (ω_e/c)(v_{s,y}x_A+y_s ṙ_{A,x}−v_{s,x}y_A−x_s ṙ_{A,y}) + d − c·δṫ_s + ε`。
- 载波:λΦ 与伪距同号(RINEX/u-blox cpMes,ublox.c:446-447/:535-539 不改号),`λ·dΦ/dt=−λD`;TDCP:`λ(Φ_{k1}−Φ_{k0}) = [ρ_{k1}−ρ_{k0}] + Δclk − c(δt_{s,k1}−δt_{s,k0}) + ε`,ΔI/ΔT/相位缠绕公共部分并入 Δclk 或噪声。
**发射时刻**:`t_tx = t_rx − P/c − δt_s`(δt_s 先由 ephclk 取,satposs ephemeris.c:804→:822 两步);载波-only 历元 P 无效时用 ρ̂+b̂−cδt_s 代 P。
**时间**:GNSS 内部 GPST(gtime_t,int64 秒+小数)。估计器时间轴=上游传感器时间:LiDAR header(Livox 不加 lidar_time_offset,LIVMapper.cpp:744)、IMU header−imu_time_offset(:776)、图像 header+img_time_offset(:847)+exposure_time_init(:953)。映射 `t_est = t_gpst,true − leap(t) + offset + drift·(t_gpst−t_ref)`,`t_gpst,true=t_rcv−b/c`;leap 由 RTKLIB 闰秒表按日期取(rtkcmn.c:171,现 18 s),offset/drift 每序列离线标定冻结(§7)。BDT=GPST−14 s(周差 1356);GST 秒同 GPST(周差 1024);GLONASST=UTC+3 h;一律经 RTKLIB 换算函数。

## 2 状态
### 2.1 宿主(不改)
[δθ 0-2 | δp 3-5 | δτ 6 | δv 7-9 | δb_g 10-12 | δb_a 13-15 | δg 16-18],DIM_STATE=19(common_lib.h:30),P=`_state.cov` 由上游维护。
### 2.2 增广块 x_G(Eigen 动态,命名槽位可增删;典型 13-16 维)
| 槽位 | 符号 | 维 | 单位 | 过程模型 | 初值/初协方差 | 存在条件 |
|---|---|---|---|---|---|---|
| tilt | φ=(φ_x,φ_y) | 2 | rad | 路程游走 q_φ·Δs | 0 / σ_DOV²=(1.5e-4)² | 激活后 |
| yaw | ψ | 1 | rad | 路程游走 q_ψΔs + q_ψr·Δα | 初始化 GN-LS,post-fit 缩放 | 激活后 |
| trans | t=t_{N←W} 相对 o^W | 3 | m | 路程游走 diag(q_th,q_th,q_tv)Δs(绕枢轴耦合 §3.2) | 一致增广(§8) | 激活后 |
| clk | b, d | 2 | m, m/s | 两状态 PSD 精确离散(§3.1) | 初始化 LS 全量 x/Q | tight |
| isb | ι_C, ι_E, [ι_R] | 2(+1) | m | 随机游走 q_isb·Δt | 初始化 LS;未观测系统 σ=30 m | tight 且系统启用;GLO 伪距默认关 ⇒ ι_R 不建 |
| clone | c=a^N(t_{k0}) | 3 | m | 静止 Φ=I,Q=0 | 一致增广 Σ_cc=JΣJᵀ | TDCP 对存活期 |
| td | t_d | 1 | s | 随机游走 | 0 / σ_toff² | `time.estimate_online`(默认 off) |
- 钟 Q(Groves 约定,不用上游 σ²·dt²,IMU_Processing.cpp:395-399):`Φ=[[1,Δt],[0,1]]`,`Q=[[S_bΔt+S_dΔt³/3, S_dΔt²/2],[S_dΔt²/2, S_dΔt]]`;S_b 下限使 √(S_bΔt_epoch) ≥ |ω_z|Δt·λ/2π(相位缠绕由钟差增量吸收,R2)。
- 主系统固定 GPS(ι_G≡0),QZSS 并入 GPS;GLONASS 只用多普勒/TDCP(λ 按频道)。
### 2.3 枢轴与重锚(D2)
过程噪声与 ψ 杠杆都相对 o^W;‖a^W−o^W‖>100 m 时 o'=a^W,t'=t+M(o'−o),对 (ψ,t) 行列施 J=[[1,0],[[e_z]×M(o'−o), I]](含交叉协方差);全局天线位置与协方差不变。
### 2.4 联合协方差与三个钩子(精确,R1/R3 逐项核对)
Σ=[[P, C],[Cᵀ, Q_G]],P 归上游,C(19×n)、Q_G 归 GNSS 模块。
- **H1 传播**(UndistortPcl 每 IMU 步 :401 后 `F_tot←F_x·F_tot`;LIO 周期挂点处):`C←F_tot·C·Φ_Gᵀ`,`Q_G←Φ_G Q_G Φ_Gᵀ+Q_G(Δs,Δα,Δt)`,`x_G←Φ_G x_G`。VIO 周期 v_imu 只有 last_imu、循环 0 次(:327)⇒F_tot=I、空操作。
- **H2 条件化**(handleLIO :371 后、handleVIO :305 后;早退路径 :343-347/:288-292 不调用):先验 (x⁻,P⁻)=`state_propagat`(LIO 迭代不改 cov,common_lib.h:182-192),后验 (x⁺,P⁺)=`_state`。`A=Cᵀ(P⁻)⁻¹`(Jacobi 缩放+LDLT,失败则特征分解伪逆并记事件);`x_G←x_G+A(x⁺⊟x⁻)`;`C←P⁺Aᵀ`;`Q_G←Q_G−A(P⁻−P⁺)Aᵀ`,对称化。依据:K_G=P_GH(P⁻)⁻¹K_H 与 H_i 无关,对 LIO 的 (I−G)P⁻(voxel_map.cpp:489-490)与 VIO 最后被接受迭代的 G(vio.cpp:800,G 在终态 x_i 线性化但 x_i 非不动点)均精确;VIO 回退分支只恢复 old_state 不改 G(vio.cpp:1677-1681)。
- **H3 GNSS 联合更新**(§4.8):(19+n) 维协方差形式 IEKF,S=HΣHᵀ+R 用 LDLT,不对 Σ 求逆,克隆奇异只出现在 S 中;写回宿主 x、P 与 x_G、C、Q_G。
- 停放态语义:未激活槽位不存在;激活一律用一致增广 y=f(x,n):P_yx=F_xP,P_yy=F_xPF_xᵀ+F_nΣ_nF_nᵀ;删槽=直接删行列(协方差形式下即边缘化)。

## 3 过程模型
### 3.1 钟、ISB、td
见 §2.2;每 LIO 步按 Δt=t_k−t_{k−1} 一次性离散(GNSS 块与 IMU 块在 F 中无耦合,与逐样本等价)。默认 S_b=0.1 m²/s、S_d=0.1 m²/s³(TCXO 保守值,由开发集多普勒 LS 钟漂 Allan 分析标定后冻结),q_isb=1e-4 m²/s,td 随机游走 1e-8 s²/s。
### 3.2 配准随机游走(按路程,绕当前枢轴)
每步从桥接轨迹算 Δs=Σ‖p̂_{i+1}−p̂_i‖、Δα=Σ|ω_z|dt。噪声源 w_φ~N(0,q_φΔs I_2)、w_ψ~N(0,q_ψΔs+q_ψrΔα)、w_t0~N(0,diag(q_th,q_th,q_tv)Δs);旋转增量 Δr=R_z(ψ)E_xy w_φ+e_z w_ψ;要求当前点 m=M(a^W−o^W)+t 不变 ⇒ Δt=[m−t]×Δr+w_t0(推导 t'=t+[u]×Δr,u=M(a^W−o^W))。`Q_ξ=J_rw diag(q_φΔs,q_φΔs,q_ψΔs+q_ψrΔα,q_thΔs,q_thΔs,q_tvΔs)J_rwᵀ`。时间项 q_time 默认 0(静止不漂移)。
**标定(闭式,不扫参,开发集)**:上游 U 臂 W 轨迹对 GT,段长 s∈{10,20,50,100,200} m,段起点用前 10% 段长对齐平移+yaw,终点误差分沿迹/横向/垂向,NNLS 拟合 `Var_along=q_th s`、`Var_lat=q_th s+q_ψ s³/3`、`Var_vert=q_tv s+q_φ s³/3`;q_ψr 由转弯段残差拟合;**取上分位(75%)斜率而非中位**(q 偏小=锁死,偏大只是信息多流向帧;R1)。默认(标定前占位):q_φ=1e-8 rad²/m、q_ψ=1e-7 rad²/m、q_ψr=1e-5 rad²/rad、q_th=1e-4 m²/m、q_tv=1e-4 m²/m。
### 3.3 克隆
静止、无噪声;与宿主的互协方差经 H1 的 F_tot 自动传递。

## 4 观测模型与雅可比
### 4.1 IMU 桥接(t_g∈(t_{k−1},t_k],δ=t_k−t_g≥0)
桥接样本 {t, a_world, ω_unbiased, R, p, v} 在 UndistortPcl 传播循环内记录(起点=:281 压入的 {prop_beg_time,R,p,v,angvel_last};每样本 :430 后;结束 :435),环形缓冲保留 4 个 LIO 周期;不依赖会被 :538 清空、:490 早退时不清的 IMUpose。区间 (t_i,t_{i+1}] 内常加速度 a_{i+1}(含 g)、常角速度 ω_{i+1},τ=t_g−t_i:p=p_i+v_iτ+½a_{i+1}τ²,v=v_i+a_{i+1}τ,R=R_iExp(ω_{i+1}τ)(与 :412-421 逐项重合)。由此 ΔR̂=R̂_gᵀR̂_k、D̂=p̂_k−p̂_g−v̂_kδ、Ê=v̂_k−v̂_g、ω̂_g。对任意迭代点 x:`p_g=p−vδ−D̂`,`R_g=RΔR̂ᵀ`,`v_g=v−Ê`,`ω_g=ω̂_g+b̂_g^snap−b_g`;`y=p_g+R_g l`,`z=v_g+R_g[ω_g]×l`;a^N=M(y−o^W)+t,w^N=Mz。迟到历元跨周期串接相对运动,δ≤0.3 s,更老丢弃计数。桥接噪声(推算,MID-360 acc_n=0.0115):Σ_D≈(acc_n²δ³/3+(½δ²(σ_ba+gσ_θ))²)I,δ=0.067 s 时≈1e-4 m,伪距/多普勒忽略,TDCP 计入 R;跨周期另加 (2 mm)²/周期。
### 4.2 公共雅可比(J_a=∂a^N/∂δx,J_w=∂w^N/∂δx;ā_b=桥接区间平均比力,G_⊥=C(I−ĝĝᵀ)/‖g‖,Ξ(y)=−R_z(ψ)[Exp(φ̃)C(y−o^W)]×)
| 列 | J_a | J_w |
|---|---|---|
| δθ 0-2 | −M R̂[ΔR̂ᵀl]× − ½δ²M R̂[ā_b]× | −M R̂[ΔR̂ᵀ([ω_g]×l)]× + δ M R̂[ā_b]× |
| δp 3-5 | M | 0 |
| δτ 6 | 0 | 0 |
| δv 7-9 | −δM | M |
| δb_g 10-12 | ≈0(<1e-4 m 忽略;单测放宽容差) | M R̂_g[l]× |
| δb_a 13-15 | −½δ²M R̂ | δ M R̂ |
| δg 16-18 | Ξ(y)[e_z]×G_⊥ + ½δ²M(仅 gravity_est_en) | Ξ(z)[e_z]×G_⊥ − δM |
| δφ | Ξ(y)E_xy | Ξ(z)E_xy |
| δψ | [e_z]×(a^N−t) | [e_z]×w^N |
| δt | I_3 | 0 |
| δt_d(可选) | −w^N | −α_A^N(天线加速度) |
δ=0 时 δθ 列退化为 −M R̂[l]×,δp 列 M,与 code-update §4 表同号。全部列由流形中心差分核验(U7)。
### 4.3 伪距行(1 Hz 抽取:|tow−round(tow)|<5 ms 的历元)
z=P_corr−[‖r_s−r_A(x̂)‖+Sagnac+b̂−δ d̂+ι̂_s−cδt_s+T̂+Î];H=−u_jᵀJ_a,+1 于 δb,−δ 于 δd,+1 于 δι_s(s≠GPS)。P_corr=prange 单频分支;卫星 rs/dts=satposs(含相对论、BDS GEO 分支 ephemeris.c:265);电离层 ionmodel(Klobuchar,/ublox_driver/iono_params)×(f_L1/f)²,方差 (0.5I)²(f_L1/f)⁴(ERR_BRDCI 可配);对流层 tropmodel(Saastamoinen),方差 (0.3/(sin el+0.1))²;轨道方差自算 σ_eph²(按 §12.3 判定的 URA 语义,不经 var_uraeph)。视线对位置的导数(~5e-8/m)与 Sagnac 率导数忽略。
### 4.4 多普勒行(全速率;与 TDCP 逐星互斥:同星在 TDCP 对两端历元不入多普勒行)
z=−λD−[e_jᵀ(v_s−ṙ_A(x̂))+Sagnac_rate+d̂−cδṫ_s];H=−u_jᵀJ_w,+1 于 δd。λ 取消息 freqs 并与 sat2freq 核对;δṫ_s 取 satposs 的 dts[1]。杆臂项 R[ω]×l 必须保留(ω=1 rad/s、l=0.4 m 即 0.4 m/s)。
### 4.5 TDCP 行(3 维克隆 + Δclk 投影 + 不重叠配对)
配对:起点历元 k0 更新后创建克隆 c=a^N(x̂_{k0}),终点 k1=k0+m(默认 0.5 s),下一对从 k1+1 起算(约 1/(m+1) 时间无 TDCP,由多普勒覆盖)。逐星条件:两端 cp 有效(status bit1)、半周已定(bit2)、bit3 相同、区间 LLI 全 0、§10 L3 通过、两端用同一 eph_t 副本(k0 时缓存副本,k1 时用它重算两端;不按 IODE 回查——BDS AODE 不唯一,R2)。
`z_j = λ(Φ_{k1}−Φ_{k0}) − {[ρ_{k1}(a^N(x̂))−ρ_{k0}(ĉ)] − c(δt_{s,k1}−δt_{s,k0})} = Δclk + H_jδx + ε_j`;H_j:当前列 −u_{j,k1}ᵀJ_a,克隆列 +u_{j,k0}ᵀ。Δclk(含钟差增量、ms 钟跳、缠绕公共项)为冗余参数:白化 W=R^{−1/2},o=W·1,Householder Q 使 Qᵀo=‖o‖e_1,保留第 2..m 行 ⇒ m−1 行独立单位方差量测(与星间单差+白化精确等价,无参考星、R 保持对角)。逐星门控在投影前做(以加权中位数稳健估 Δclk,再逐星 w 检验)。
### 4.6 松耦合行(L-PVT 臂)
receiver_pvt 的 lat/lon/alt(椭球高)→N 系 z_p,h=a^N,H=J_a 的位置部分;速度 z_v=(vel_e,vel_n,−vel_d),h=w^N。R=diag(h_acc²,h_acc²,v_acc²)·κ_AR(τ_pvt),1 Hz 抽取;速度 (vel_acc)²·2。无钟槽;帧模型/桥接/门控与 T 相同,只隔离「原始观测 vs 接收机解」。
### 4.7 噪声模型(默认;§17 一次性定标后冻结)
| 量测 | 方差 |
|---|---|
| 伪距 | s_sys²·{σ_P0²(1+1/sin²el)·10^{max(0,45−CN0)/10} + (k_std·psr_std)²}·κ_AR(τ_ρ,1 s) + σ_eph² + var_ion + var_trop + 桥接项 u_jᵀMΣ_DMᵀu_j + 钟桥接 S_bδ+S_dδ³/3 + (u_jᵀw^N·σ_toff)²;σ_P0=0.3 m,s_sys:G/E/C=1、C_GEO=2、R=1.5 |
| 多普勒 | (max(λ·dopp_std,σ_D0)·(1+1/sin²el)/2)² + 桥接 (δ(σ_ba+gσ_θ))² + S_dδ + (‖ω‖σ_l)² + (‖α_A‖σ_toff)² + (0.01‖v‖)²;σ_D0=0.05 m/s;再乘 κ_AR(τ_D,0.1 s) |
| TDCP | λ²(σ²_Φ(k0)+σ²_Φ(k1)) + u_jᵀMΣ_{D,k1}Mᵀu_j + (σ_rate·Δt_pair)² + σ_wu² + **(‖Δv_pair‖·σ_toff)²**(R1);σ_Φ=max(cp_std_m,σ_Φ0/sin el),σ_Φ0=0.004 m,σ_rate=2 mm/s,σ_wu=2 mm;k0 端桥接噪声已在 Σ_cc |
κ_AR(τ,Δt)=(1+ρ)/(1−ρ),ρ=exp(−Δt/τ):AR(1) 样本均值方差放大倍数;τ_ρ、τ_D 由开发集新息自相关拟合(占位 τ_ρ=10 s、τ_D=0.2 s)。psr_std/cp_std/dopp_std 单位由入库自检判定(§12.6)后换算成米。
### 4.8 联合更新(H3)
1) 快照 (x⁻,x_G⁻,Σ⁻);2) 迭代 i=0..:在 x_i 建行得 r_i、H_i、R(对角+桥接满阵);S=H_iΣH_iᵀ+R,LDLT;K=ΣH_iᵀS⁻¹;x_{i+1}=x⁻⊞K[r_i+H_i(x_i⊟x⁻)](宿主 ⊞/⊟ 同上游,ψ 取模);3) **迭代条件**:σ_ψ>2° 或首轮 |Δψ|>0.1° 时最多 3 次;激活后与再捕获后前 5 个历元固定 3 次;其余单步(R_z(ψ) 作用于长杠杆是唯一显著非线性;R1);4) 门控只用 r_0、S_0(§10 L4);5) Joseph:Σ⁺=(I−KH)Σ(I−KH)ᵀ+KRKᵀ,对称化;6) 验后检查(L5)失败整体回滚到快照;7) 写回 `_state`(R,p,τ,v,b_g,b_a,g,cov)与 x_G、C、Q_G。

## 5 「地图当真值」一致性论证与诊断
- 信息量(推算,code-update §6):LIO 后 σ_p≈1–4 mm(每点 R≥1e-3 m²,voxel_map.cpp:449,N_eff 300–3000);伪距合计 ≤30 m⁻²;TDCP 每对 3–5 mm 与 LIO 相对位移信息同量级;多普勒合计 2–4 cm/s。
- 分配:伪距 H_t=−u_jᵀ、H_p=−u_jᵀM 同方向,修正按 P_pp:P_tt^eff 分,p^W 份额≈6e-5;多普勒主进 d、ψ、v^W;TDCP 主进 ξ 增量,p^W/v^W 约一成(推算)。退化方向 P_pp 随 IMU 传播长大(0.5 加计参数下 1 s≈3 cm,10 s≈1 m,推算)⇒ 增益自然转向 p^W,同周期进图(:413-424)。本段曾据此不采用 FAST-LIVGO 式 Hessian 双模;该定位不是用户要求,已作废,见 A73。门宽 S 由 P_ξξ(按真实漂移率标定)主导,LIO 过度自信不会错杀 GNSS。
- **回访旧地图(共享局限,R1 判定)**:GNSS 建立的 P_ξp 仅 ~3e-3·P_pp,LIO 吸附旧地图的跳变 Δ(≤门宽 ~10 cm)几乎全部出现在全局输出;跨越跳变的 TDCP 对被 L4 整历元拒绝;t 以伪距时间常数(分钟级)重收敛。单一 ξ 不能表示分区配准;A→B→中断→回 A 时全局差 (ξ_B−ξ_A)。记录事件(LIO 修正量/预测 σ)与幅值;GNSS-denial 预测只对「中断始于起点区」成立。
- **零成本诊断**:D1(实现前,参考提交 R 的 `debug/cov_csv`):在 :256 前(prior)、:371 后(postLIO)、:305 后(postVIO)只读写出 diag(P)、位置块特征值/向量、effct_feat_num;开发集 2 条序列。预注册:postLIO 最坏轴 σ_p 中位 ≤1 cm ⇒ 前提成立(预期非退化段核心份额 <1%);>20% 帧 σ_p≥10 cm ⇒ 明显退化段,设计不变,单独报告。D2:§3.2 的 RPE-路程标定。D3(实现后):每次 GNSS 更新记 f_p=‖K_pz‖/‖Kz‖、f_ξ、f_clk、f_v;预注册非退化段 f_p 中位 <1e-3、退化段上升;GNSS 开/关下 σ_p 分布无显著差异。D4:各类平均 NIS/dof∈[0.8,1.25],95% 历元在 χ² 界内;t 相对 RTK 真值的 NEES 时间序列作诊断(暴露 κ_AR 覆盖不到的小时级大气偏差,R1)。
- **FAST-LIVGO 表 II 三符号解释(推断)与预注册预测**:①开阔全降=固定锚 TDCP+多普勒逐历元压住漂移,干净数据下过度自信无害;②Dark01/02 上升=残余多径/NLOS 以 mm 级权重压进位姿并被固定锚继承;③去剔除发散=周跳 19 cm 当 mm 级位移进位姿,LIO 关联崩溃。预测:(a) T 相对 U 在开阔序列多数胜,T−tdcp 收益明显变小;(b) Dark01/02 T 不劣于 U(等效界内);(c) T−reject 退化但不发散;(d) A-FA 复现夜间上升、A-FA−norej 至少一条 Dark 发散;(e) L-PVT 对齐 ATE 与 U 无显著差;(f) GNSS-denial 若从未初始化则逐位等于 U。任一失败如实报告为该机制证伪。

## 6 固定锚为何不一致(A-FA 臂的预注册依据)
随机游走真值 x_{k+1}=x_k+w_k,TDCP Δ_k=w_k+n_k(方差 σ²)。固定锚把 x̂_k 当常数 ⇒ 后验 P_{k+1}≈σ²,而真实误差 e_{k+1}=e_k−n_k 方差 kσ² ⇒ NEES≈k 线性发散:门宽压到 mm 级错杀好观测;无门时周跳/多径以 mm 级权重灌入;伪距在 P≈σ² 的滤波器里权重≈0(解释其伪距只做 RAIM)。克隆版 P_{k+1}=P_k+σ² 正确。代价:+3 维、增广簿记、不重叠配对损失 1/(m+1) 覆盖。

## 7 时间对齐
- 映射见 §1;`time.base`∈{gpst,utc,host},M3DGR=host。GnssMeasMsg 无 header,bag 记录时间(主机收包)只作诊断(到达延迟分布,推断 50–100 ms ⇒ 跨周期桥接是常态)。
- **离线标定工具 `tools/time_calib/gnss_time_calib.py`**(每序列一次,冻结;输入为 R 的 GNSS-free 确定性回放,无循环依赖):①记录时间统计:range_meas 记录时间−(GPST→Unix)⇒偏差+延迟分布、鲁棒线性拟合,>50 ms 持续台阶即分段;同样统计 LiDAR/IMU/图像头戳−记录时间判定 LiDAR 时基;②主方法:|v_W(t)| 与自写多普勒 LS 的 |v_GNSS| 重采样 100 Hz,offset₀±1 s 归一化互相关,抛物线细化;60 s 滑窗重复→线性拟合得 offset、drift_ppm;σ_toff 取峰宽/子窗 bootstrap;③DTW 只作交叉核对(报偏移中位与离散度,与②差>3σ 告警);④质检:峰值相关系数、窗间离散 >5 ms 报警。输出 `time_calib/<seq>.yaml`。激励不足(速度 std<0.1 m/s)则保留配置值、σ_toff=50 ms 并自动缩短 TDCP interval 或暂停 TDCP。
- 在线:td 状态可选(默认关,慢速地面弱可观);多普勒残差对 u_jᵀα_A 的滑窗回归斜率作残余时偏诊断,>3σ 告警不自动改值。time_pulse_info 无主机侧 PPS 捕获、无 external_trigger(README:145-152)⇒ 只做闰秒/时间系统/粗错/主机钟台阶检查。
- 同步误差量级(推算,v≈1–2 m/s):10 ms ⇒ 伪距 2 cm(忽略)、多普勒 ~|a|·10 ms(轻微)、TDCP a·Δt_pair·δ ≈ mm–cm(必须计入 R)。

## 8 初始化状态机(宿主不受影响)
`OFF → OBSERVE(只算不写) → WAIT_CORE → WAIT_NAV → COLLECT → ALIGN → TRACKING ⇄ OUTAGE → RECOVERY`,叠加标志 DEGENERATE、LIVO_ALARM;REINIT 可从任意态进入;全部写 events.log。
| 阶段 | 条件与动作 |
|---|---|
| WAIT_CORE | `!p_imu->imu_need_init && lidar_map_inited` 且 ≥50 个 LIO 帧;gravityAlignment 断言未激活 |
| WAIT_NAV | ≥5 颗星有健康星历;缺 iono 时 I=0、σ_ion=5 m,参数到达后切换 |
| COLLECT | 窗口 ≥30 历元且 ≥10 s:逐历元自写稳健 SPP(IRLS-Huber+迭代 w 检验+C/N0-仰角 NLOS 预筛,≥5 星、GDOP<5)与多普勒 LS(输出速度、d 与协方差);同时桥接取宿主天线 a^W_j、w^W_j。窗末历元 k_a |
| ALIGN 时间 | §7 互相关估 offset(在线阶段仅核对离线值,差>3σ 告警) |
| ALIGN 偏航 | 水平速度≥0.5 m/s 的历元≥30、水平位移≥5 m:2D Procrustes 初值 ψ₀=atan2(Σ(v_D×Λw)_z,Σ(v_D·Λw)_h),再以原始多普勒 GN-LS 解 (ψ,每历元 d_j)(d_j 投影掉),要求 post-fit 缩放后 σ_ψ≤3°。静止时不对准 |
| ALIGN 平移/钟/ISB | 固定 ψ,窗口原始伪距批量 LS(共享 t 与 ι,每历元 b_j),迭代 w 剔除,σ̂≤5 m;r_0=窗内稳健均值(N 系由此固定);o^W=当前 a^W;SPP 位置方差取 max(formal·σ̂0²,3² m²) |
| ALIGN 激活 | 一致增广:φ=0+n_φ(σ_DOV);ψ 取 GN 解;t=a^N_SPP−M(ψ,φ,g)(y−o^W),含 t 对 x/ψ/φ 的互协方差 −J_xP、−J_ψσ_ψ²;b、ι 取 LS 联合协方差块;d 取 k_a 多普勒 LS;初始化用过的历元不再作更新(避免重复计数);TDCP 第一对从 k_a+1 起;进 TRACKING(前 5 历元固定 3 次迭代)。失败则滑窗继续并记原因 |
| 消融 | 去伪距/去多普勒/去 TDCP 只影响滤波更新,初始化始终用 SPP 与多普勒 LS |
可观性:ψ 由多普勒(σ_ψ≈σ_v/v_h/√N)与 TDCP(对内位移≥数 cm);t 由伪距(≥1 星即可,有钟状态);b 与垂向 t 由几何分开;d 由多普勒;φ 弱可观由先验主导;ι_s 需该系统与 GPS 同时可见。激活即放开,不设 GVINS 式低速固定 yaw(EKF 不可观时增益为 0,静止时按路程的过程噪声为 0)。

## 9 更新调度
```
sync(LIO) → processImu():
  Process2 (:252): UndistortPcl 内记录桥接样本、F_tot 累积、去畸变
  gravityAlignment (:254, 默认关)
  ★ gnss_->onLioStep(_state, bridge, F_tot, t_prev, t_k)      // 新增,:254 与 :256 之间
      H1 传播 → 状态机/初始化 → 已到达且 t_est≤t_k 的历元按 GPST 升序:
      L0→L1→L2→L3→L4→H3 联合更新(Joseph+对称化)→L5→克隆管理→诊断
      → 宿主均值/协方差写回 _state
  state_propagat=_state; voxelmap_manager->state_=_state       // :256-257 不变,LIO 先验含 GNSS 后验
handleLIO: StateEstimation(:370) → _state=…(:371) → ★H2(LIO) → evo/odom(:384-409 不动) → 地图插入(:413-424,含 GNSS 修正) → ★publishGlobal
sync(VIO) → processImu(零传播;钩子空操作) → handleVIO: processFrame(:305) → ★H2(VIO) → ★onStepEnd(刷新 C_ref、写 traj/TF)
```
- ONLY_LIO 模式:同一挂点;onStepEnd 挂 handleLIO 末尾(:481)。ONLY_LO(无 IMU)不支持,启动报错。
- 放 LIO 前而非 VIO 后(R1/R3 判定两者一致,差别仅在地图插入是否含本历元修正):退化方向修正同周期进 LIO 先验与地图;LIO 无点提前返回(:343-347)时 GNSS 照常;不改 sync 簿记。配置项 `update_slot: pre_lio|post_vio` 保留,冻结前用 F0/T 两臂各跑一次确认统计等价后定 pre_lio。
- 同周期多历元串行(克隆链自然衔接);t_est>t_k 留待下周期。

## 10 异常剔除(全部 0/1 成员资格;EKF 内无鲁棒核、无数据依赖膨胀)
| 层 | 判据 | 动作 |
|---|---|---|
| L0 信号 | 历元时间单调/去重;NaN;伪距∈[1.9e7,4.5e7] m;|λD|<2 km/s;el≥10°;C/N0≥30(伪距)/25(多普勒)/28(载波);status bit0/1/2;svh=0;星历龄 G/E 2 h、C 1 h、R 30 min;伪距连续跟踪≥10 历元、载波≥3(无 locktime 字段,用计数);排除列表;GLO 伪距关;BDS 1575.42 拒收;GAL PRN 37–38 丢弃计数 | 该行不参与 |
| L1 GNSS 内部 | 自写 GNSS-only 稳健 SPP/多普勒 LS(IRLS-Huber 只在此 GNSS-only 估计器内,不进 EKF)+ Baarda w 检验;RTKLIB pntpos 只作参照行(其 valsol χ² 已注释,pntpos.c:384) | 被剔星本历元伪距不用;SPP 与预测水平差>30 m 转 L2 |
| L2 钟跳 | 伪距新息中位数 m:|m−k·c·1 ms|<1 km(k≠0)⇒ b+=k·c·1 ms,λΔΦ 扣 k·c·1 ms 后 TDCP 照用;|m|>1 km 非整毫秒 ⇒ 以预测天线位置做只解钟 LS 重置 b、d(方差 MAD²,C 钟行清零) | 事件 CLOCK_JUMP |
| L3 周跳(TDCP) | D1 LLI;D2 bit2/bit3 两历元变化;D3 Δt_pair>1.5 s;D4 多普勒积分 |ΔΦ+(D_{k1}+D_{k0})Δt/2|>max(0.5 周,4σ_DΔt+j_maxΔt³/(12λ));D5 LIVO 预测新息 w 检验+5 cm 绝对阈;D6 双频 GF>5 cm | 断链、克隆停放、事件 SLIP |
| L4 新息一致性 | 逐星 Baarda `w_i=(S⁻¹ν)_i/√(S⁻¹)_ii`(全 S,天然边缘化公共钟/帧,1 星可判),α=1e-3(3.29);历元 NIS 对 χ²_m(1−α);超限剔 argmax|w_i| 重算,直至通过或 m<m_min(伪距 1/多普勒 1/TDCP 2);被剔>50% ⇒ 该类本历元不更新;TDCP 先减稳健 Δclk 再逐星检验 | 分类分别统计 |
| L5 验后 | 后验 NIS rᵀ(R−HΣ⁺Hᵀ)⁻¹r 超 χ² 阈;或宿主 ‖δp‖>max(3σ_prior,5 cm) 且 σ_prior<5 cm;或 Σ⁺ 非正定/NaN | 整体回滚到快照,状态逐位不变,计数 |
| L6 锁死恢复 | 连续 20 历元 L1 一致(≥5 星)但 L4 拒绝>50% ⇒ 以 SPP 重新一致增广 t 与钟;多普勒也系统性被拒 ⇒ 重做 ψ;停放克隆 | 事件 REANCHOR;T−pr 臂禁用 |
| L7 完好性(外部裁判,只改标志不改宿主 P) | GNSS-only 多普勒速率与 LIVO 预测速率连续 5 历元 d²>χ²₁(0.999) 且差>0.2 m/s ⇒ LIVO_ALARM:GNSS 更新改为 Schmidt(宿主列 K 置零,只写帧/钟块)直至连续 10 历元一致;水平速度方向连续 20 历元偏差>max(5°,3σ) ⇒ 重做 ψ(REINIT_YAW) | 事件;不做 P_vv/P_pp 手术(R1 判 D2 §5.6.1 不一致) |
| 退化下限(默认关) | 信息增益 (P⁺)⁻¹−(P⁻)⁻¹ 的 [θ,p] 块平移 Schur 补 λ₁<400 m⁻² 连续 10 帧(滞回 800)⇒ DEGENERATE 标志;`degeneracy.floor_enable=true` 时沿 u₁ 加半正定下限 (5 cm)² | 主臂只记标志;T+floor 为可选探针臂(它会进 LIO 门与地图点方差,R3) |
否决:IGG-III/Huber 进 EKF;κR 步长缩放(等价权/调权);LIVO_ALARM 的 P 膨胀。设计稿曾否决按 Hessian 最小特征值切双模(上游 6×6 特征分解被注释 voxel_map.cpp:467;lio/min_eigen_value 是平面拟合阈值 :42);该定位不是用户要求,已作废,见 A73。

## 11 中断与恢复
- OUTAGE:连续 1 s 无 L0 有效历元;克隆停放;ξ 按路程、钟按时间传播(60 s 后 σ_b≈85 m,推算;ISB 只慢增,保留)。
- RECOVERY:≥4 星稳定 1 s 后进入;σ_b>1e3 m 先以预测天线位置做只解钟 LS 一致重置;前 5 历元固定 3 次迭代;L4 全程生效(门宽已随 P_ξξ 增大,大新息进帧而非位姿);不做人为限幅、不做 R 斜坡(信息就是信息);1–3 星也可用伪距/多普勒(钟连续);TDCP 需两端干净锁定后重开对。
- REINIT:OUTAGE>60 s 且(σ_t>10 m 或 σ_ψ>0.1 rad),或 L6/L7 触发:删帧与钟槽位重走 COLLECT→ALIGN;全局输出在事件点允许不连续(呈现层 `/gnss/global_odometry_smooth` 限速 0.2 m/s、2°/s,评测用未平滑)。
- GNSS-denial 全程无星:初始化永不满足 ⇒ 增广块为空 ⇒ 宿主逐位等于 R;GNSS-denial01/02 中途拒止、回到起点区:ξ 均值不变、不确定度增长;回起点若 LIO 吸附旧地图则全局输出跳 ≈Δ(§5 局限),否则由 GNSS 修正。
- 隧道口 NLOS 由 L4 剔除;夜间:GNSS 与光照无关,退化指数用 LIO+VIO 合并信息。

## 12 底座与 I/O
### 12.1 RTKLIB-EX v2.5.1 vendoring(`third_party/rtklib/`,BSD-2,保留 license.txt、readme.txt;VENDORED.md 记 tag、各文件 sha256、补丁)
- 编 `src/*.c` + `src/rcv/ublox.c`(input_ubx 真机用)整体成静态库 `livo_rtklib`(不用 lib/:IERS/SOFA/BLAS 各有许可;不 add_subdirectory 上游 CMake);宏 PUBLIC 导出 `ENAGLO ENAQZS ENACMP ENAGAL ENAIRN NFREQ=3 NEXOBS=3 MAXPRNCMP=63`(obsd_t/nav_t 布局依赖,rtklib.h:158-165);`-O2 -fno-fast-math -ffp-contract=off -w`;不定义 TRACE;应用桩 showmsg/settspan/settime。
- 补丁(patches/*.patch,CI 校验 vendored 树=tag+补丁):P1 rtklib.h:211 `MAXPRNCMP` 加 `#ifndef` 并设 63(否则 satno 对 C51–C63 返回 0,rtkcmn.c:415;GEO 分支已支持 prn≥59),编译期断言 MAXSAT≤255(obsd_t.sat 为 uint8);P2 pntpos.c 末尾导出包装 `rtkx_prange/rtkx_varerr/rtkx_gettgd`,并导出 estpos/estvel 的完整 x 与 Q(上游只存位置块 sol->qr,pntpos.c:449-453/:621)。无需 IODE 回查补丁(克隆缓存 eph_t 副本)。
- 隔离:`rtklib.h` 只在 `gnss/src/rtk_adapter.cpp` 一个 TU 包含(pimpl;无前缀宏 PI/CLIGHT/MAXOBS…);`livo_gnss` 编成 SHARED,`-fvisibility=hidden`,`-Wl,--exclude-libs,liblivo_rtklib.a`(norm/dot/mat/trace 等 C 符号不外泄)。全局可变状态 eph_sel/leaps/timeoffset_ ⇒ GNSS 只在主线程串行调用(上游 main.cpp 单线程 spinOnce)。
- 用到:satposs/satpos/eph2pos/geph2pos/ephclk、geodist/satazel、ionmodel/tropmodel、satno/satsys/sat2freq/code2idx、gpst2time/time2gpst/gpst2utc/gpst2bdt/bdt2gpst/time2bdt/gst2time、ecef2pos/pos2ecef/xyz2enu、pntpos(仅参照行/回归)、readrnxt、input_ubx。自写:量测模型、增广块、钩子、FDE、GNSS-only 稳健 SPP/多普勒 LS(RTKLIB estpos/resdop 为 static 且 RAIM 单故障)。
- 已知缺口:BDS/Galileo 电离层共用 GPS Klobuchar×(f_L1/f)²(ionocorr 只用 ion_gps,pntpos.c:204-250),ERR_BRDCI 可配 0.5→0.7 起,开发集按系统分箱看伪距 NIS。
### 12.2 读取 gnss_comm 消息(不链接 GPL-3.0)
- `gnss_io`(BSD-3):`ros1_msgdef.cpp` 解析 bag 连接头 `message_definition`(主类型+`MSG:` 依赖段;内建类型、time、string、定长/变长数组、Header)生成 schema;`ros1_decoder.cpp` 按 ROS1 线格式(小端、变长数组 uint32 长度前缀、嵌套内联、time=2×uint32)解成字段树;`gnss_comm_mapping.cpp` 以字段名为键映射到 `ObsEpoch/KeplerEph/GloEph/Klobuchar/ReceiverPvt`。**扁平化 (type,name) 序列与内置期望表逐项比对,不一致即拒收并报版本差异**(D3)。字节来源两条汇聚到同一 `feedBytes()`:回放 `rosbag::MessageInstance::getMessageDefinition()+write()`;在线 `topic_tools::ShapeShifter`(focal ros-base 自带);rosbag isType 接受 md5 "*"(R3 实测)。
- Python 导出器 `tools/gnss_bag_export.py`(BSD-3,rosbags Apache-2.0 作独立程序):逐星 CSV(含记录时间)、星历 CSV、iono/PVT/time_pulse JSON、RINEX 3.04 OBS/NAV(供 rnx2rtkp 参照行与 GREAT/UrbanNav 输入路径 `source: rinex`,readrnxt)。C++ 与 Python 在同片段 CSV 逐字节一致(T3)。
- 许可证理由(工程判断,非法律意见,写 docs/LICENSING.md):仓库不含 gnss_comm 任何源/头/.msg,不编译不链接;消息定义随数据集(MIT)提供,属数据;按公开序列化规则实现读取器属互操作,字段名/顺序为功能性事实且运行时从数据读取;实时与 GPL-3 的 ublox_driver 只经话题通信(FSF FAQ 分离程序);编号换算用 RTKLIB satno。否决:链接 gnss_comm(GPL-3 与 GPL-2.0-only 不兼容,FSF FAQ #v2v3Compatibility);GPL-3 预处理进程转发(灰区无收益);复刻 .msg 兼容包(复制 GPL-3 文本,且 7 个字段名违反 rosidl 规则);RINEX 作唯一通路(丢精确值与延迟)。
### 12.3 字段语义映射(gnss_comm → RTKLIB;以 §12.6 入库自检为准)
- 卫星号:GPS 1–32、GLO 33–59(prn=sat−32)、GAL 60–97(prn=sat−59)、BDS 98–160(prn=sat−97)→ `satno(sys,prn)`。
- 星历(GnssEphemMsg→eph_t):toe/toc/ttr 为 GPST(week,tow)(gnss_ros.cpp:76-78 用 gpst2time,含 BDS)→gtime_t;**toes 按系统**:GPS/QZS=time2gpst(toe),GAL=time2gpst(toe)(与 GST 秒同),**BDS=time2bdt(gpst2bdt(toe))**(eph2pos 用 omge·toes,ephemeris.c:266/:277;填错 27 km;gnss_comm 自己也对 BDS 重算,gnss_utility.cpp:499);week 按系统口径;A=半长轴;tgd0/tgd1→tgd[0]/tgd[1](GPS TGD;GAL BGD E5a/E1、E5b/E1;BDS TGD1/TGD2);health→svh;ura 语义(米或索引)由自检判定,只用于健康检查与自算 σ_eph²,不经 var_uraeph。
- GLONASS(GnssGloEphemMsg→geph_t):freqo→frq(并写 nav.glo_fcn[prn−1]=frq+8)、tau_n→taun、gamma→gamn、**delta_tau_n→dtaun**、pos/vel/acc 逐项;与 freqs 反推的 k=(f−1602 MHz)/0.5625 MHz 交叉核对,不一致丢弃计数。
- 观测(GnssObsMsg):freqs→CODE_*:GPS/QZS 1575.42→CODE_L1C、1227.60→L2X;GLO 1602+0.5625k→L1C、1246+0.4375k→L2C;GAL 1575.42→L1X、1207.14→L7X;BDS 1561.098→**CODE_L2I**(prange 取 TGD_B1I,pntpos.c:189-193)、1207.14→L7I;**BDS 1575.42 拒收计数**;其他组合拒收。psr→P、cp→L、dopp→D、CN0→SNR、LLI、psr_std/cp_std/dopp_std(单位判定后换米);status bit0 伪距有效、bit1 载波有效、bit2 半周有效、bit3 半周已扣。每星只取主信号(L1 类)作伪距/多普勒/TDCP,第二频点仅 GF 周跳。iono_params 8 值→nav.ion_gps;receiver_pvt→L-PVT 与自检;time_pulse_info 仅诊断。
### 12.4 其它输入
RINEX(`source: rinex`,按 t_est+配置延迟并入事件流);UBX 字节流 `GnssRawStream{header,format,uint8[] data}`(自定义 BSD,真机 F9P,input_ubx 进程内解码,不依赖 GPL 驱动)。
### 12.5 ROS 无关核心接口(`gnss/include/livo_gnss/`,仅依赖 Eigen+livo_rtklib,CI 无 ROS 环境单独编译)
```cpp
namespace livo_gnss {
struct HostIdx { static constexpr int rot=0,pos=3,expo=6,vel=7,bg=10,ba=13,grav=16,dim=19; };
struct CoreState { Eigen::Matrix3d R; Eigen::Vector3d p,v,bg,ba,g; double inv_expo; Eigen::Matrix<double,19,19> P; };
struct BridgeSample { double t; Eigen::Vector3d a_world, omega; Eigen::Matrix3d R; Eigen::Vector3d p,v; };
class GnssFusion {
 public:
  explicit GnssFusion(const Config&);
  void feedBytes(Source, const uint8_t*, size_t, double t_record);   // gnss_io 解码后入队
  void pushObservation(ObsEpoch&&); void pushEphemeris(const KeplerEph&); void pushGloEphemeris(const GloEph&);
  void pushIono(const Klobuchar&); void pushPvt(const ReceiverPvt&);
  StepReport onLioStep(CoreState& core, const Eigen::Matrix<double,19,19>& F_tot,
                       const std::vector<BridgeSample>& bridge, double t_prev, double t_k);  // H1+状态机+H3,可改写 core
  void onCoreUpdated(const CoreState& prior, const CoreState& post, UpdateSource);            // H2
  void onStepEnd(const CoreState&);                                                            // 刷新 C_ref
  bool globalPose(const CoreState&, Eigen::Matrix3d* R_NI, Eigen::Vector3d* t_NI, Eigen::Matrix<double,6,6>* cov) const;
  bool anchor(Geodetic*) const; Status status() const; void setDiagnosticsSink(DiagnosticsSink*);
};}
```
适配层 `livo_host_view` 负责 StatesGroup↔CoreState 逐成员拷贝(`static_assert(DIM_STATE==19)`,拷贝函数单测覆盖);ROS2 移植只重写胶水(rclcpp 订阅+CDR/自有消息)。
### 12.6 入库静默自检(任一未过不产出评测数字)
1 解码计数:每历元星数、频点分布、历元间隔 10 Hz;每颗星 `sat2freq(sat,code,nav)` 与 freqs 一致;msg_def 校验通过。2 符号:每星 λ·dΦ/dt 与 −λD 相关>0.99、斜率≈1;λΔΦ 与 ΔP、−λDΔt 同号。3 std 单位:cp_std 取值集合落在 {0.004n/λ} 或 {0.004n} 格点之一,dopp_std 落在 {0.002·2ⁿ},psr_std 落在 {0.01·2ⁿ},据此定单位。4 URA 语义:与同日 RINEX nav 的 SV accuracy 对拍。5 SPP 对 PVT:开阔段水平差中位<3 m、速度差<0.2 m/s。6 时间:|offset_est+18 s|<0.3 s、互相关峰清晰;到达延迟分布;主机钟台阶。7 杆臂方向:转弯段 l 与 −l 两假设下多普勒残差 RMS 差≈ω·|l_h|(推算 0.18 m/s);多普勒新息对 ω×l 项回归系数≈0(≈−2 即符号反)。8 星历覆盖时间线、C59–C62 出现。9 确定性:同配置两次 sha256 相同。10 F0 与 R 逐位相同。
### 12.7 许可证布局
仓库 GPL-2.0-only(package.xml:15 BSD→GPL-2.0-only,另列 BSD-3/BSD-2);`gnss/`、`gnss_io/`、`tools/` BSD-3(待用户确认,见 decisions);`third_party/rtklib` BSD-2;`ros/`胶水与上游改动 GPL-2.0-only;LICENSES/ 全文,源文件 SPDX 头;只发源码,不发镜像/deb(上游依赖 GPLv3 的 rpg_vikit)。

## 13 确定性离线回放(`fastlivo_replay`)
- 参考提交 R(tag `ref-R`):0d2c034 + processOnce 抽取 + replay_main + 压缩图直读 + 输出写出器 + IMU_Processing.h:60/:80/:81 类内 `=0.0`(零值即事实上游行为;valgrind 审计)。估计器数学零改动。
- 启动:每作业私有 roscore(独立端口、ROS_LOG_DIR),rosparam load 到 `/laserMapping`(vikit camera_loader 从此读,LIVMapper.cpp:128),节点名 laserMapping;replay 模式不建订阅,发布者照建(`publish_frame_world` 清空 `pcl_w_wait_pub` :1289 是 VIO 的门 :288,不得跳过)。
- 喂数:`rosbag::View` 按**记录时间**(=rosbag play 顺序;header 序会让 LiDAR 排在其帧内 IMU 之前,非因果);CustomMsg→livox_pcl_cbk(md5 匹配,livox_ros_driver/driver2 定义相同);Imu→imu_cbk;CompressedImage→`img_compressed_cbk`:按 `format` 字段决定通道序后转 bgr8 再 `pushImage()`(I2 核对直读灰度与 republish 灰度逐像素一致);GNSS 原始字节→feedBytes(t_record)。每条后 `while(mapper.processOnce()){}`(实时 run() :534-554 循环体抽成 processOnce,语义不变)。
- 非确定源(代码审计,R3 核实):sync 层对到达时序不变(:973 只决定是否等待;:1010-1033/:988-997 只吃 ≤t_img 的数据),故实时方差只来自①启动期跨话题次序(:773/:850 首帧雷达前丢样)②vio.cpp:1554 float 归约③:248 的 UB 读。eval profile:去 MP_EN(voxel_map.cpp:660 按下标写本就确定);`-ffp-contract=off` 全 TU(focal GCC 9.4 对 C++17 默认 fast,R3 实测,含 vikit:patch vikit_common/CMakeLists.txt:10/:28 去硬编码);三包同 `-march=native`(逐位结论只在同机同镜像成立;`-march=x86-64-v3` GCC 9.4 不认);`-DEIGEN_DONT_PARALLELIZE`、`OMP_NUM_THREADS=1`;无 LTO;不读墙钟(omp_get_wtime 只打印);容器键为 VOXEL_LOCATION/int;关 imu_prop 定时器。
- 扰动种子:`replay.start_offset_s = k×0.05 s`,k=0..9(改变首帧 LiDAR、IMU 初始化窗口与图像丢弃,命中真实来源);各臂共用同一组种子(CRN);同种子下 GNSS 初始化前各臂逐位相同。硬门:种子间 state_trace 两两不同。否决 D2 虚拟时钟(sync 时序不变 ⇒ 种子塌缩)。预注册试点规则:2 条开发序列上若确定性集成 ATE IQR 不足实时(U 跑 5 次)IQR 的 25%,改用图像时间戳 ±1 ms 抖动种子(D1)作第二 CRN 源,再不足则实时重复。
- 输出:`state_trace.txt`(每 processOnce 一行,`%a` hexfloat:cycle、flag、t_k、R(9)、p、v、b_g、b_a、g、τ、diag(P)、x_G、diag(Q_G));`traj_W.tum`/`traj_N.tum`/`traj_ant_N.tum`(%.9f,步末状态,t_k);上游 evo 文件;GNSS 日志;`timing.csv`(不比对);`run_manifest.json`(git describe/dirty、profile 与编译定义、Eigen/GCC 版本、镜像 digest、配置 sha256+全量转储、bag sha256、seed、臂、主机、起止时间)。
- 四道门(eval profile,同机同镜像):G-确定性(同 bag 两次 sha256 相同;R 与 HEAD 各一次);G-零通道(HEAD `gnss.enable=false` 对 R:state_trace 与上游 evo 逐字节相同;`git diff` 不触及 common_lib.h/voxel_map.*/vio.*/frame.*/visual_point.*/preprocess.*,IMU_Processing.* 只含清单行);G-旁路(`enable=true, update_enable=false`,钩子与 F 累积真实执行,宿主逐位等于 R);G-无数据(GNSS 开、bag 无 GNSS 话题 ⇒ 逐位等于 R)。失败排查:逐 TU 反汇编比对 → accumulateF noinline → 确认 -ffp-contract → valgrind。
- Log 隔离:ROOT_DIR 编译期烧死(CMakeLists.txt:40),evo app 模式追加(:391-397)⇒ 每次运行独立容器并绑定挂载 `<src>/Log` 到运行目录。

## 14 配置(`gnss:` 命名空间;星号项由开发集标定后冻结并计算 sha256;未写的键取上游行为;同一数据集一套 LIVO 参数)
```yaml
common: {img_compressed: true}                # M3DGR: /camera/color/image_raw/compressed
replay: {enable: false, bag: "", start_offset_s: 0.0, duration_s: -1.0, order: record, img_jitter_s: 0.0, seed: 0}
publish: {use_sensor_time: true}
output: {dir: "", traj_w: true, traj_n: true, state_trace: true, global_cloud: false}
debug: {cov_csv: true}
gnss:
  enable: false                # false 时逐位等于 R
  update_enable: true          # false=OBSERVE(只算不写,旁路门)
  mode: tight                  # tight | loose_pvt | livgo_probe | livgo_probe_norej
  update_slot: pre_lio         # pre_lio | post_vio(冻结前等价性核对用)
  use: {psr: true, dopp: true, tdcp: true, pvt_vel: true}
  source: gnss_comm_ros1       # gnss_comm_ros1 | rinex | ubx_stream
  topics: {meas: /ublox_driver/range_meas, ephem: /ublox_driver/ephem, glo_ephem: /ublox_driver/glo_ephem,
           iono: /ublox_driver/iono_params, pvt: /ublox_driver/receiver_pvt, tp: /ublox_driver/time_pulse_info}
  rinex: {obs: "", nav: "", latency_s: 0.0}
  systems: {gps: true, glo: true, gal: true, bds: true, qzs_as_gps: true, glo_psr: false, bds_geo: {use: true, psr_scale: 2.0}}
  exclude_sats: []
  frames: {lever_arm_frame: lidar, lever_arm_xyz: [-0.358, 0.086, 0.429],   # calibration.md §1.7,经 extR/extT 转 IMU 系 ⇒ l=[-0.31635,0.10926,0.4006]
           lever_arm_sigma: 0.03, pivot_distance_m: 100.0, sigma_dov_rad: 1.5e-4, gnss_updates_gravity: true}
  time: {base: host, leap: auto, offset_s: 0.0, drift_ppm: 0.0, t_ref_gpst_s: 0.0, calib_file: "",
         sigma_toff_s: 0.05, estimate_online: false, td_rw_s2_per_s: 1.0e-8, max_bridge_s: 0.3, bridge_ring_cycles: 4}
  imu_bridge: {acc_n: 0.0115, gyr_n: 0.00133}     # M3DGR Kalibr(MID-360);Avia 0.0208/0.00322
  masks: {el_deg: 10.0, cn0_psr: 30.0, cn0_dopp: 25.0, cn0_cp: 28.0, track_psr: 10, track_cp: 3,
          eph_age_s: {G: 7200, E: 7200, C: 3600, R: 1800}, max_psr_std_m: 5.0, max_dopp_std_hz: 1.0, max_cp_std_m: 0.02}
  noise:
    psr:  {sigma0_m: 0.3, cn0_ref: 45.0, k_std: 1.0, tau_s: 10.0*, decim_s: 1.0, err_brdci: 0.5, sys_factor: {G: 1.0, E: 1.0, C: 1.0, R: 1.5}}
    dopp: {sigma0_mps: 0.05*, tau_s: 0.2*, exclusive_with_tdcp: true}
    tdcp: {sigma0_m: 0.004*, interval_s: 0.5, sigma_rate_mps: 0.002*, sigma_windup_m: 0.002, max_span_s: 1.5}
    pvt:  {tau_pos_s: 30.0*, tau_vel_s: 1.0*, decim_s: 1.0, scale: 1.5}
    scale: {psr: 1.0*, dopp: 1.0*, tdcp: 1.0*}      # NIS≈1 标量定标
  process:
    clock: {S_b: 0.1*, S_d: 0.1*, S_b_min_windup: true}
    isb_q: 1.0e-4
    frame: {q_tilt_per_m: 1.0e-8*, q_yaw_per_m: 1.0e-7*, q_yaw_per_rad: 1.0e-5*, q_th_per_m: 1.0e-4*, q_tv_per_m: 1.0e-4*, q_time: 0.0}
  init: {min_livo_frames: 50, min_sats: 5, max_gdop: 5.0, min_epochs: 30, min_window_s: 10.0, min_hspeed_mps: 0.5,
         min_moving_epochs: 30, min_path_m: 5.0, yaw_sigma_max_deg: 3.0, spp_sigma_floor_m: 3.0, nlos_cn0_margin_db: 8.0}
  rejection: {alpha_w: 1.0e-3, alpha_nis: 1.0e-3, max_reject_frac: 0.5, min_meas: {psr: 1, dopp: 1, tdcp: 2},
              post_check: true, core_step_max_m: 0.05, clock_jump_km: 1.0, lockout_epochs: 20}
  slip: {lli: true, half_cycle: true, max_gap_s: 1.5, dopp_tol_cyc: 0.5, jerk_max: 1.0, innov_abs_m: 0.05, gf_tol_m: 0.05}
  integrity: {velocity_alarm: {enable: true, alpha: 1.0e-3, min_err_mps: 0.2, n_consec: 5, n_clear: 10},
              heading: {enable: true, min_speed: 0.5, n_consec: 20, min_err_deg: 5.0},
              degeneracy: {lambda_enter: 400.0, lambda_exit: 800.0, window: 10, floor_enable: false, sigma_floor_m: 0.05}}
  iterate: {yaw_sigma_deg: 2.0, dyaw_deg: 0.1, max_iter: 3, reacq_epochs: 5}
  outage: {outage_s: 1.0, recover_min_sats: 4, recover_settle_s: 1.0, reinit_after_s: 60.0, reinit_sigma_t_m: 10.0, reinit_sigma_psi_rad: 0.1}
  ablation: {frame_fixed_after_init: false, tdcp_fixed_anchor: false, disable_l4_l5: false}
  output: {frame_id: enu, smooth_trans_mps: 0.2, smooth_yaw_dps: 2.0, sat_csv: false}
```
M3DGR 主配置:LIVO 参数取官方 `baseline_systems/Fast_LIVO2_M3DGR`(锁 commit),所有臂共用;Avia 时 extrinsic_T=[0.04165,0.02326,−0.0284]、R=I(calibration §1.6,与上游 avia.yaml 一致);Rcl/Pcl=avia→camera(§1.3);D435i PINHOLE 640×480 fx 607.80 fy 607.84 cx 328.80 cy 245.53;MID-360 杆臂链由 `tools/m3dgr_calib.py` 推导(推断 ≈[−0.284,0.001,−0.211],须复算并经 §12.6-7 检验)。

## 15 输出与诊断
| 产物 | 内容 | 时间戳 |
|---|---|---|
| `traj_W.tum` / `traj_N.tum` / `traj_ant_N.tum` | 步末(VIO 与 GNSS 之后)T_{W←I}、T_{N←I}、天线 N 系位置;N 系仅激活后 | t_k(传感器时间) |
| `traj_N_geo.csv`、`anchor.yaml` | WGS-84 lat/lon/h;r_0、R_{E←N} | — |
| 上游 evo `Log/result/<seq>.txt` | LIO 后验,原样(零通道门用) | t_k |
| `gnss_epochs.csv` | 每历元:GPST/t_est/t_k/δ/到达延迟;状态机与标志;各系统星数;伪距/多普勒/TDCP 各类候选/接受/剔除按原因(mask/L1/clock/slip D1–D6/w/nis/majority/step);各类先验 NIS 与自由度、后验 NIS;增益份额 f_p/f_ξ/f_clk/f_v;ξ 各分量与 σ、b、d、ι、td;SPP 与滤波天线位置差;λ₁₋₃ 与 u₁;宿主 √diag;‖δp_H‖、δψ、‖δt‖;迭代次数、提交/回滚;耗时 | — |
| `gnss_sats.csv`(可选) | 每历元每星:el、az、C/N0、可用性、σ、新息、w、决策与原因、周跳检测器、NLOS 嫌疑 | — |
| `filter_cov.csv`(D1/D3) | t_k、stage(prior/postLIO/postVIO/postGNSS)、effct_feat_num、√diag(P)、位置/姿态块特征值、min diag(P⁻)与条件数、√diag(Q_G) | — |
| `events.log` | ACTIVATE、REANCHOR、REINIT、CLOCK_JUMP、SLIP、LATE_EPOCH、TOFF_WARN、LIVO_ALARM 起止、DEGENERATE 起止、REPIVOT、ROLLBACK、msg_def 拒收 | — |
| `state_trace.txt`、`timing.csv`、`run_manifest.json`、`summary.json`(各类平均 NIS/覆盖率/接受率/耗时 p50/p95/p99) | §13 | — |
| 话题 | `/gnss/global_odometry`(frame enu)、`/gnss/global_odometry_smooth`、`/gnss/fused_fix`、`/gnss/spp_fix`(NavSatFix)、`/gnss/diagnostics`(DiagnosticArray 1 Hz)、TF `enu→camera_init`(=T_{N←W}) | t_k |
`tools/report.py`:决策着色天空图、NIS 时间线与 χ² 带、模式/退化时间线、帧状态、增益分配、轨迹叠加。RViz 同时显示 W 与 enu。

## 16 已知局限
1 地图本身不做全局修正:W 地图保留 LIVO 漂移,全局点云只能按 T(ξ(t)) 逐扫描拼接(硬约束 3 的代价)。2 单一配准假设:回访/分区配准失配,全局输出在回环处不自洽,差值=期间吸收的漂移;靠 L6 兜底并记事件。3 上游 LIO 协方差过度自信不修改,退化轴接管可能不足(退化下限默认关)。4 单频伪距米级;时间相关只覆盖多径尺度(κ_AR),分钟–小时级大气/轨道偏差进入 t 且不平均,P_tt 只对多径尺度一致(以 t 对 RTK 的 NEES 诊断暴露)。5 不估模糊度,无 RTK/PPP(M3DGR 无基准站)。6 电离层:BDS/GAL 共用 GPS Klobuchar;TDCP 电离层速率按白噪声。7 多普勒与 TDCP 只在配对端点互斥,对内其余历元的相关未建模。8 时偏按常数+漂移处理,主机 NTP 台阶只能分段;M3DGR 软同步、LiDAR 时基未知。9 杆臂固定(σ_l 进噪声);calibration.md 天线条目方向靠自检;真值 2HP 天线杆臂未知,构成 ATE 地板。10 TDCP 覆盖损失 1/(m+1)。11 上游缺陷保留以维持 R 等价:(I−G)P 不对称、点协方差缺 R_WI(voxel_map.cpp:387/:551-552、LIVMapper.cpp:420-421)、偏置噪声硬编码(:153-154)、Livox 不用 lidar_time_offset(:744)、evo 写在 VIO 前、:490 早退不清 IMUpose(下一帧陈旧条目二次去畸变)、gravityAlignment 不旋转协方差。12 逐位确定性只在同机同镜像(vikit -march=native)。13 UBX 真机通路无实测(用户无接收机)。14 「A-FA 只复刻更新方案、不含 DTW/双模」这一条定位已作废,见 A73。15 许可证结论为工程判断;上游 vikit GPLv3 冲突本已存在。宿主仍是 GPL-2.0-only,本仓库新代码仍是 BSD-3-Clause。16 「方法首创被占、故定位为许可证干净的差异化实现」已作废,见 A73。