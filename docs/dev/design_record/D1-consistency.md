> Historical research record (through 2026-10-03). Paths are illustrative; archived runs are not bundled. For current status and executable entry points see [the repository README](../../../README.md). Current release docs/LICENSING.md supersedes historical license notes. This record does not authorize new experiments or assert current release validation.

# FAST-LIVO2 × 原始 GNSS 紧耦合(ESIKF)完整设计 —— 估计一致性优先版

依据代码:`/workspace/fast_livo2_ref` @0d2c034(行号均指此副本);RTKLIB-EX main@06e8644(= v2.5.1 线,行号指 scratchpad `rtklibex/src`);gnss_comm @a2035fe(只读,仅用于理解数据格式);FAST-LIVGO arXiv 2606.19190v2 全文。凡本人推出而未实测的结论标「推断」。

---

## 0 结论先行

1. **"地图当真值"的正面回答:把漂移放进配准状态,而不是放大位姿协方差。** LIO 后验 `P_pp` 描述的是"相对当前局部地图"的误差(毫米级,code-update §6 推算),不是相对地球的误差。全局漂移必须由某个状态承载,本设计用 **W→N 配准 ξ=(φ_x, φ_y, ψ, t)**,并给它**按行驶路程计、绕当前位置旋转的随机游走**。游走速率由 LIVO 在开发序列上"相对误差-路程"曲线的闭式拟合给出。
   - 于是一致的卡尔曼增益会自动把伪距、多普勒、TDCP 的修正主要分给 ξ。非退化方向上 `p^W` 只分到约 `P_pp/(P_pp+P_tt) ≈ 1e-4` 的份额,局部体素地图和视觉地图不被撕裂。
   - LIO 退化方向上,`P_pp` 会随 IMU 传播长大,GNSS 自动接管该方向。FAST-LIVGO 用启发式双模切换想做的事,在一致的联合协方差里是自然发生的。
2. **倾斜由 g^W 决定,不估自由 SO(3)。**
   - W 相对水平面的倾斜由已有状态 `g^W` 通过无偏航调平 `C(g)` 给出。在此之外只追加 1 维偏航 ψ、3 维平移 t,再加 2 维"倾斜残差" φ:φ 的先验取垂线偏差量级,并随路程游走,用来承载地图弯曲和 Z 向漂移。
   - `gravity_align_en` 保持上游默认 false,无需先把 W 水平化。
3. **钟采用"主钟差 b + 钟漂 d + 3 个系统间偏差(ISB)"。** 两状态钟模型按连续 PSD 精确离散。不用"每星座独立钟差":接收机只有一个振荡器,独立随机游走会虚增独立性。
4. **TDCP 用随机克隆 + 钟差增量投影消去 + 不重叠配对。**
   - 克隆的是 3 维"起点历元天线在 N 系的位置"。接收机钟差增量作为冗余参数,被投影消去。
   - 采用不重叠配对:起点克隆 → 终点量测 → 下一对从下一历元重新起算,因此相邻 TDCP 的噪声严格独立。
   - FAST-LIVGO 式(11)的固定锚把上一时刻后验当常数。§6 的标量推导显示,这会使 NEES 随历元数线性增长。
5. **push-through 增益是本设计对上游估计器唯一的数学改动,而且是必须的。** 克隆在创建时刻使 P 精确奇异,上游 LIO/VIO 的信息形式 `(HᵀR⁻¹H+P⁻¹)⁻¹`(voxel_map.cpp:468,vio.cpp:1497/1661)会直接失效。因此改为数学等价的 `K₁[:, :k] = P[:, :k](I_k + A·P_kk)⁻¹`,只需解一个 k×k 方程。
6. **挂点与顺序。**
   - 挂点在 `processImu()` 中,Process2 和 gravityAlignment 之后、`state_propagat=_state` 之前(LIVMapper.cpp:254→256)。
   - GNSS 用 IMU 前向快照桥接到 t_k,做独立的协方差形式 IEKF,用 Joseph 形式更新协方差。GNSS 后验就是 LIO 的先验,与 LIO/VIO 顺序更新,不联合迭代。
7. **剔除全部是 0/1 成员资格判定,不做连续调权。** 层次为:信号掩码 → RAIM(GNSS 内部一致性,作为外部判决信号)→ 先验 χ² 门(α 固定)→ 历元级逐星排除 → 提交后检查与回滚。
   - 门宽由 `P_ξξ` 主导,所以对 LIO 协方差的过度自信不敏感。
8. **许可证。**
   - fork 保持 GPL-2.0-only。
   - 卫星轨道/钟差、各项改正、SPP/RAIM、UBX/RINEX 解码全部取自 vendored 的 RTKLIB-EX v2.5.1(BSD-2),只打三处小补丁。
   - gnss_comm 的代码和 `.msg` 一行都不进仓库。M3DGR 的消息由"按 bag 内嵌 `message_definition` 在运行时建 schema 的 ROS1 线格式解码器"读取。
9. **评测。**
   - 确定性离线回放:按 bag 记录时间顺序直读,每条消息之后同步调用处理循环,并对 VIO 浮点归约定序,做到逐比特可复现。
   - 统计比较:用"种子化 ±1 ms 图像时间戳抖动"构造公共随机数集成,每臂每序列 10 个种子,做成对 Wilcoxon 检验并用 Holm 校正。
   - FAST-LIVGO 表 II 只作为未审计的引用。

---

## 1 坐标系、符号与约定

| 记号 | 定义 |
|---|---|
| E | ECEF(WGS84;RTKLIB 内部统一使用。BDS 的 CGCS2000、GLONASS 的 PZ-90.11 与 WGS84 的差在厘米级,忽略) |
| N | 固定 ENU 系,原点 `r_0`(ECEF),在 GNSS 激活时由 SPP 确定后**永不再改**。`R_EN = R_{E←N}`,三列依次为 r_0 处东、北、天的单位向量 |
| W | FAST-LIVO2 世界系 = 地图系(首帧 IMU 系,不水平,yaw 任意) |
| I, L | IMU 系、LiDAR 系;`extR/extT = T_{I←L}`(LIVMapper.cpp:122-123) |
| A | 天线相位中心,杆臂 `l^I`(在 I 系中) |
| L0 | W 的调平系:`R_{L0←W} = C(g)` |

- **变换方向。** `R_{A←B}` 把 B 系坐标变成 A 系坐标,即 `p^A = R_{A←B} p^B + t^A_B`。状态中 `R = R_{W←I}`(rot_end),`p = p^W_I`,`v = v^W_I`。
- **配准。** `R_{N←W} = M := R_z(ψ)·Exp(φ̃)·C(g)`,其中 `φ̃=(φ_x,φ_y,0)ᵀ`;`t := t^N_W`,即 W 原点在 N 中的坐标。于是 `p^N = M p^W + t`。
- **扰动。** 旋转用右扰动 `R = R̂·Exp(δθ)`(common_lib.h:170/:184),与上游一致;其余状态加性;ψ 更新后取模到 (−π, π]。
- **残差约定。** `z = y_meas − h(x̂)`,`H = ∂h/∂δx`,与 LIO 的 `z=−dis`(voxel_map.cpp:457)同一约定。VIO 用的是"预测减观测"(vio.cpp:1621),GNSS 不采用那一套。
- **GNSS 观测符号(显式)。** 定义 `e_j = (r_s − r_A)/‖r_s − r_A‖`(接收机指向卫星),`u_j = R_ENᵀ e_j`。
  - **伪距:**
    `ρ_j = ‖r_s,j(t_tx) − r_A‖ + ω_e(x_s·y_A − y_s·x_A)/c + b + ι_{s(j)} − c·δt_s,j + T_j + I_j + ε`
    TGD/BGD 施加在观测端(RTKLIB `prange`,pntpos.c:110)。RTKLIB `rescode` 的 `v = P − (r + dtr − c·dts + dion + dtrp)`(pntpos.c:335,r 已含 Sagnac,见 `geodist` rtkcmn.c:3612)与此式一致。
  - **多普勒:** 距离率 `ρ̇_j = −λ_j D_j`(D 单位为 Hz,卫星接近时为正)。
    `ρ̇_j = e_jᵀ(v_s − ṙ_A) + (ω_e/c)(v_sy·x_A + y_s·ṙ_Ax − v_sx·y_A − x_s·ṙ_Ay) + d − c·δṫ_s + ε`
    与 RTKLIB `resdop`(pntpos.c:587)、gnss_comm `dopp_res`(gnss_spp.cpp:272-278)、FAST-LIVGO 式(12)三处同号。
  - **载波:** `λΦ` 与伪距同号(RINEX 约定),因此 `λ·dΦ/dt = −λD`。RTKLIB-EX 对 RAWX 的 cpMes/doMes 不改号(rcv/ublox.c:446-447,:535-539)。数据入库时用自检验证(§12.6)。
- **时间。**
  - GNSS 内部一律用 GPST(RTKLIB `gtime_t`)。估计器时间轴沿用上游的传感器时间:LiDAR 取 header(Livox 回调不加 `lidar_time_offset`,LIVMapper.cpp:744);IMU 取 header−imu_time_offset(:776);图像取 header+img_time_offset(:847),切分时再加 exposure_time_init(:953)。
  - GNSS 历元映射为 `t_est = t_gpst,true + gnss_time_offset (+ rate·(t_gpst − t0))`,采用"加"约定。其中 `t_gpst,true = t_rcv − b/c`。
  - offset 默认值为 `−(GPST−UTC)`,由 RTKLIB 闰秒表(rtkcmn.c:171,目前为 18 s)按日期自动取,不硬编码。

---

## 2 状态向量与增广

### 2.1 布局(DIM_STATE 19 → 33,新状态全部追加在 19 之后)

| 索引 | 误差状态 | 单位 | 含义 |
|---|---|---|---|
| 0-2 | δθ | rad | 上游,不动 |
| 3-5 | δp | m | 上游 |
| 6 | δτ | – | 上游(逆曝光) |
| 7-9 | δv | m/s | 上游 |
| 10-12 / 13-15 | δb_g / δb_a | – | 上游 |
| 16-18 | δg | m/s² | 上游(R³ 自由) |
| **19-20** | δφ | rad | 配准倾斜残差(调平系内绕 x/y) |
| **21** | δψ | rad | 配准偏航 |
| **22-24** | δt | m | 配准平移 `t^N_W`(E,N,U) |
| **25** | δb | m | 主系统接收机钟差(c·δt_r) |
| **26** | δd | m/s | 钟漂(c·δṫ_r) |
| **27-29** | δι | m | 另外 3 个系统相对主系统的 ISB(槽位按 {G,R,E,C} 去掉主系统后的顺序) |
| **30-32** | δc | m | TDCP 克隆:克隆历元天线在 N 系的位置 |

- **为什么只能追加:** LIO 的 H 只占前 6 列(voxel_map.cpp:466-472),VIO 只占前 7 列(vio.cpp:1660-1667),而且 0-18 的索引在多处写死(code-timeline §1.3)。
- **为什么是固定 33 维而不做动态块:**
  - 星座集合固定为 4 个,ISB 按最大值预留;TDCP 只需一个 3 维克隆;不引入模糊度状态。
  - 固定尺寸让上游所有 `DIM_STATE` 宏位点(common_lib.h:137/167/182/194、IMU_Processing.cpp:161/301、voxel_map.cpp:366/468-490、vio.h:122、vio.cpp:1497-1501/1661-1667)自动适配。

### 2.2 配准参数化:4-DoF + 2 维倾斜残差

- **问题。** W 是地图系,不水平。它相对真实水平面的倾斜只能通过 `g^W`(W 中的重力方向)知道。
- **做法。** 定义调平旋转 `C(g)`,满足 `C·ĝ = −e_z`。再取 `M = R_z(ψ)·Exp(φ̃)·C(g)`。
  - 倾斜主体来自 `g^W`。IMU 与 LIO 对 g 方向的约束远强于 GNSS:GNSS 要靠长水平基线上的高程差才能看出倾斜,而 GNSS 高程误差是强相关的偏差(推断)。
  - φ 只承载两类残余:垂线偏差(先验 σ_φ0 = 1.5e-4 rad,可配置);地图随路程的弯曲(随机游走 q_φ,见 §3.5)。
- **否决的方案:**
  - 自由 6-DoF(FAST-LIVGO 式(2)-(3)、LIGO 的 Rot3):倾斜会被表示两次(g^W 与 SO(3) 各一份),两者之间的约束不被执行,结果不一致。
  - 纯 4-DoF 并假设 W 水平:W 实际不水平,1 mrad 的倾斜在 500 m 处就造成 0.5 m 高程误差(推算)。
  - 先 gravity_align 再固定 g:地图系的倾斜会被强行塞进姿态和加计偏置。
- **C(g) 的维护(无偏航调平)。**
  - 保存参考 `C_ref`。对任意 g,令 `C(g) = Δ(C_ref·ĝ → −e_z)·C_ref`,其中 Δ 是两向量间的最小旋转(Eigen `Quaterniond::FromTwoVectors`)。由于 `C_ref·ĝ ≈ −e_z`,Δ 的转轴水平,调平不改变偏航(一阶)。
  - 每个滤波步结束时用最终的 g 刷新 `C_ref`。
  - 一阶关系:`C(g+δg) ≈ Exp(ε)·C(g)`,其中 `ε = [e_z]×·C·(I − ĝĝᵀ)·δg/‖g‖`,且 ε_z 恒为 0。
  - 推导:要求 `(I+[ε]×)C(ĝ+δĝ) = −e_z`,展开得 `Cδĝ = −[e_z]×ε`;又因 `Cδĝ` 为水平向量,解为 `ε = [e_z]×Cδĝ`。
- **偏航与平移的耦合。** ψ 绕 W 原点旋转,离原点越远,ψ 与 t 的相关越强。EKF 通过协方差正确处理这一点。过程噪声则按"绕当前位置旋转"施加(§3.5),避免偏航噪声把当前位置"甩走"。

### 2.3 钟:主钟差 + ISB

- **主系统。** 激活窗口内卫星最多的系统(中国境内通常是 BDS)。QZSS 并入 GPS;SBAS 和 IRNSS 不用。
- **伪距模型。** 伪距用 `b + ι_s`;多普勒只含公共的 d,系统间钟漂差约为 0。与 RTKLIB `rescode` 的 `x[3]+x[4..7]` 参数化一致(pntpos.c:344-352)。
- **否决"每星座独立钟差 + 独立随机游走":** 物理上只有一个振荡器,各系统之间只差近常数的硬件延迟和系统时差;独立游走会让滤波器误以为各系统的钟噪声互相独立。
- **否决"星间单差消钟"(code-update §4d/§6.3):** 会丢掉钟的时间连续性。有钟状态时,1 到 3 颗星也能用(GVINS 摘要、InGVIO 表 III 的 2/3 星实验)。

### 2.4 TDCP 克隆

- **定义。** 克隆 `c` = 起点历元 t0 时天线在 N 系的位置,是当时状态的确定性函数 `c = a(x; bridge_0)`(§4.1)。
- **为什么只克隆 3 维天线位置,不克隆 10 维 (θ, p, ξ):** TDCP 只依赖两端天线位置和钟差增量。钟差增量作为冗余参数被投影消去,于是 3 维足够。这与 MSCKF 式的"状态函数克隆"等价,而且是一致的。
- **增广公式。** 设 `J_c = ∂a/∂δx`,则
  - `P_cc = J_c P J_cᵀ + Σ_bridge0`
  - `P_cx = J_c P`
  - 克隆在两次量测之间静止:F 取单位阵,无噪声。它与主状态的互协方差由 IMU 传播中稠密的 `F P Fᵀ` 自动传递(新行的 F 默认就是单位阵)。
- 在克隆存活期间,LIO/VIO 的更新会经由互协方差修正克隆,这是正确的联合高斯行为。

### 2.5 停放态语义(激活/停放)

- **停放。** 未激活的 GNSS 状态(GNSS 初始化前、臂内关闭、系统未见、克隆空闲):均值为 0,方差为 1,与所有状态无互协方差;不施加过程噪声,不进入任何 H。
  - 由于 push-through 增益中这些行满足 `P[parked,:k]=0`,LIO/VIO 对它们的增益恰为 0,不会被改动。
  - 停放一个已激活的状态,等价于对它做高斯边缘化:清零所在行和列,并把对角元置 1。
- **激活**一律用一致增广 `y = f(x, n)`,`n` 与 x 独立:
  - `P_yy = F_x P F_xᵀ + F_n Σ_n F_nᵀ`
  - `P_yx = F_x P`
  - 状态激活标志保存在 GNSS 核心内部,不属于 StatesGroup 的高斯部分。

### 2.6 StatesGroup 改动(common_lib.h)

新增成员:`V2D gnss_tilt; double gnss_yaw; V3D gnss_trans; double clk_b, clk_d; V3D isb; V3D clone_pos;`。以下 6 个手写函数必须逐一补齐,否则 VIO 回退(vio.cpp:1401/1490/1510/1523/1650/1679)会静默丢失这些成员:

- 默认构造 :128-140:新块的协方差取停放值 1.0,不能继承 `INIT_COV`。
- 拷贝构造 :142-152 与 `operator=` :154-165。
- `operator+` :167-180 与 `operator+=` :182-192:ψ 更新后取模。
- `operator−` :194-206:ψ 差值取模。
- 成员声明 :215-222。

新增 `include/state_index.h` 定义命名索引,并用 `static_assert(DIM_STATE==33)` 防漂移。

---

## 3 过程模型与离散化

### 3.1 上游 IMU 块不动

`IMU_Processing.cpp:382-399` 在每个 IMU 样本上执行 `F_x.setIdentity()` 后填入上游各块,新状态的 F=I、Q=0 自动成立。IMU 循环内一行都不改。

**离散化约定的区别:**
- 上游的 `cov_w = σ²·dt²`(:395-399)是"每个 IMU 步的离散方差",会随 IMU 频率变化,单位也不是 PSD。
- GNSS 状态一律用连续 PSD 做精确离散,绝不照搬 σ²dt²。

### 3.2 GNSS 块每个 LIO 步一次性传播

GNSS 块与 IMU 块在 F 中没有耦合,所以在 GNSS 挂点对整个步长 Δt = t_k − t_{k−1} 一次性施加 `Φ_G, Q_G`,与逐样本施加等价:

- `P_GG ← Φ_G P_GG Φ_Gᵀ + Q_G`
- `P_xG ← P_xG Φ_Gᵀ`(IMU 循环已完成 `F_total·P_xG`)

**证明:** 联合转移矩阵为 `diag(F_total, Φ_G)`,由此即得上式。

### 3.3 钟(两状态模型,连续 PSD 精确离散)

`ḃ = d + w_b`,`ḋ = w_d`,PSD 为 `S_b [m²/s]`、`S_d [m²/s³]`。

```
Φ = [[1, Δt],[0, 1]]
Q = [[S_b·Δt + S_d·Δt³/3,  S_d·Δt²/2],
     [S_d·Δt²/2,           S_d·Δt   ]]
```

- 与 Allan 方差的关系:`S_b ≈ c²h_0/2`,`S_d ≈ 2π²c²h_{−2}`(Brown & Hwang)。
- 初值取 TCXO 保守量级 `S_b = 0.1`、`S_d = 0.1`。最终值由开发序列上"逐历元多普勒 LS 钟漂序列"的 Allan 分析给出(§17.4)。
- 钟跳见 §10。

### 3.4 ISB

随机游走 `Q = q_isb·Δt`,初值 `q_isb = 1e-6 m²/s`。

### 3.5 配准随机游走(按路程,绕当前位置)

每步从快照计算两个量:路程 `Δs = Σ‖p̂_{i+1} − p̂_i‖`,转角 `Δα = Σ|ω_z|dt`。噪声源为:
- 旋转噪声 `w_φ ~ N(0, q_φ·Δs·I_2)`,`w_ψ ~ N(0, q_ψ·Δs + q_ψr·Δα)`;
- 当前点的平移噪声 `w_t0 ~ N(0, diag(q_th·Δs, q_th·Δs, q_tv·Δs))`。

在 N 系中,配准旋转增量为 `Δr = R_z(ψ)·E_xy·w_φ + e_z·w_ψ`(`E_xy = [e_x e_y]`)。

**绕当前点旋转的推导:** 要求当前全局点 `g = M p + t` 在增量前后不变,即 `M'p + t' = Mp + t`,其中 `M' = Exp(Δr)M`。解得 `t' = t + [u]×Δr`,`u = M·p`。所以:

```
Δφ = w_φ,  Δψ = w_ψ,  Δt = [u]×(R_z(ψ)E_xy w_φ + e_z w_ψ) + w_t0
Q_ξ = J_rw·diag(q_φΔs, q_φΔs, q_ψΔs+q_ψrΔα, q_thΔs, q_thΔs, q_tvΔs)·J_rwᵀ
```

**否决以时间为自变量的游走:** 静止时 LIVO 不漂移,若按时间游走,会让 ξ 在静止期无端变松。时间项 `q_time` 默认取 0。

**标定(闭式,不扫参):** 在开发序列上用上游 LIVO 轨迹(W)对真值,取段长 s ∈ {10, 20, 50, 100, 200} m。每段在起点用前 10% 段长(至少 2 m)拟合平移和 yaw,然后测终点误差,分解为沿迹、横向、垂向三个分量。按下列模型做非负加权最小二乘:

```
Var_along(s) = q_th·s
Var_lat(s)   = q_th·s + q_ψ·s³/3        (偏航随机游走 → 横向误差 ∝ s^1.5)
Var_vert(s)  = q_tv·s + q_φ·s³/3
```

`q_ψr` 由转弯段与直线段的残差差异拟合得到。
**依据:** LIVO 相对地图的误差很小,段终点误差 ≈ 该段内配准的变化量,所以这组 RPE 统计直接度量 ξ 的游走。

### 3.6 其余

- 克隆:静止,无噪声。
- 停放态:不传播。
- 倾斜残差、偏航、平移的初始不确定度见 §8。

---

## 4 观测模型与雅可比

### 4.1 天线位置与速度(含 IMU 桥接)

GNSS 历元 t_g 落在 (t_{k−1}, t_k] 内,更新时刻为 t_k,记 `δ = t_k − t_g ≥ 0`。桥接量取自 IMU 前向快照(§7.2),与速度无关:
- `ΔR̂ = R̂_gᵀ R̂_k`
- `D̂ = p̂_k − p̂_g − v̂_k·δ`(即 `−∫∫a`)
- `Ê = v̂_k − v̂_g`
- `ω̂_g`:快照中的无偏角速度

对任意状态估计 x(先验或迭代点):

```
p_g = p − v·δ − D̂,   R_g = R·ΔR̂ᵀ,   v_g = v − Ê,   ω_g = ω̂_g + b̂_g^snap − b_g
y := p_g + R_g·l,     a(x) = M·y + t                          (天线在 N 中的位置)
z := v_g + R_g·[ω_g]×·l,  w(x) = M·z                         (天线在 N 中的速度;E、N 均为地固系,无牵连项)
r_A = r_0 + R_EN·a,  ṙ_A = R_EN·w
```

### 4.2 雅可比 J_a = ∂a/∂δx 与 J_w = ∂w/∂δx

记 `ā_b` 为桥接区间内的平均比力,`G_⊥ = C(I − ĝĝᵀ)/‖g‖`,`Ξ(y) = −R_z(ψ)·[Exp(φ̃)Cy]×`。

| 列 | J_a | J_w |
|---|---|---|
| δθ | `−M R̂[ΔR̂ᵀl]× − ½δ²M R̂[ā_b]×` | `−M R̂[ΔR̂ᵀ([ω_g]×l)]× + δ·M R̂[ā_b]×` |
| δp | `M` | 0 |
| δv | `−δ·M` | `M` |
| δb_g | ≈0(<1e-4 m,忽略) | `M R̂_g[l]×` |
| δb_a | `−½δ²M R̂` | `δ·M R̂` |
| δg | `Ξ(y)[e_z]×G_⊥ + ½δ²M` | `Ξ(z)[e_z]×G_⊥ − δ·M` |
| δφ | `Ξ(y)·E_xy` | `Ξ(z)·E_xy` |
| δψ | `[e_z]×(a − t)` | `[e_z]×w` |
| δt | `I_3` | 0 |

- 与 code-update 表的一致性:δθ 列在 δ=0 时退化为 `∂ρ/∂δθ = u_jᵀM R̂[l]×`,与该表的 `eᵀR_EW R_WI[l]×` 同号。
- δτ(索引 6)列恒为 0。
- 所有列都由单元测试做流形上的中心差分核对(§17.1)。

### 4.3 伪距行

```
z_ρ = P_j^corr − [ ‖r_s − r_A(x̂)‖ + Sagnac + b̂ − δ·d̂ + ι̂_s − c·δt_s + T̂_j + Î_j ]
H_ρ = −u_jᵀ·J_a  (+1 于 δb;−δ 于 δd;+1 于 δι_s,仅非主系统)
```

各项来源:
- `P^corr` = RTKLIB `prange` 单频分支(GPS 扣 TGD;GLONASS 扣 −dtaun 的频率缩放;Galileo 扣 BGD;BDS 按 B1I/B1C 选 TGD,pntpos.c:163-190);
- 卫星位置与钟差来自 `satposs`(ephemeris.c:781),已含相对论和 BDS GEO 分支(:265);
- 电离层:`ionocorr`(IONOOPT_BRDC,Klobuchar)再乘 (f_L1/f)²,方差按四次方缩放(同 pntpos.c:323-324);
- 对流层:`tropcorr`(Saastamoinen,pntpos.c:251)。

`−δ·d` 一项把 t_k 的钟差回推到 t_g。

### 4.4 多普勒行

```
z_D = −λ_j D_j − [ e_jᵀ(v_s − ṙ_A(x̂)) + Sagnac_rate + d̂ − c·δṫ_s ]
H_D = −u_jᵀ·J_w  (+1 于 δd)
```

- λ_j 取消息的 `freqs` 字段,并用 RTKLIB `sat2freq` 交叉核对(GLONASS FDMA 需要星历中的频道号)。
- `δṫ_s` 取 `satposs` 输出的 dts[1]。
- e_j 对位置的导数(约 5e-8 rad/m)与 Sagnac 率项的导数都忽略。

### 4.5 TDCP 行(克隆 + 钟差增量投影 + 不重叠配对)

**配对规则。** 起点历元 k0 更新后创建克隆;终点历元 k1 = k0 + m 处构造量测;下一对从 k1+1 起算。默认 m 对应 0.5 s,可配置。
- 这样相邻两对不共享任何相位样本,噪声严格独立。
- 代价是约 1/(m+1) 的时间段不被 TDCP 覆盖,这段由多普勒覆盖。

对每颗满足"起止两端连续锁定,且两端使用同一 IODE 星历"的卫星 j:

```
z_j = λ_j(Φ_j,k1 − Φ_j,k0) − { [ρ_geo,j(k1; a(x̂)) − ρ_geo,j(k0; ĉ)] + ΔT̂_j − ΔÎ_j − c(δt_s,j,k1 − δt_s,j,k0) }
      = Δclk + H_j δx + ε_j
H_j = −u_j,k1ᵀ·J_a(x; δ_1)  于当前状态列;  +u_j,k0ᵀ 于 δc(30-32)
```

- ρ_geo 含 Sagnac。
- 两端卫星状态必须用同一条星历计算:k0 时缓存卫星和 IODE;k1 时用补丁 P3(§12.1)按同一 IODE 重算两端。
- 相位缠绕的公共部分和接收机钟差增量一起被 `Δclk` 吸收。
- `Δclk` 是所有卫星共享的冗余参数,处理方法:
  1. 对残差白化:`W = R^{-1/2}`,`o = W·1`;
  2. 取 Householder 矩阵 Q 使 `Qᵀo = ‖o‖e_1`;
  3. 保留第 2..m 行,得到 m−1 行相互独立、方差为 1 的量测。

  这与"星间单差 + 白化"精确等价,但不需要选参考星,也不会引入非对角 R。

**逐星门控。** 投影之后量测与卫星不再一一对应,所以门控在投影之前做:先以残差的加权中位数稳健估计 `Δclk`,再对每颗星做归一化残差检验,最后只对内点做投影更新(§10)。

### 4.6 噪声模型(全部显式,可追溯)

| 量测 | 方差 |
|---|---|
| 伪距 | `σ²_ρ,j = EFACT_sys²·[a² + b²/sin(el) + snr项] + (k·psr_std)² + var_eph(URA) + var_ion + var_trop + 0.3²`(RTKLIB `varerr` pntpos.c:51 + rescode 中的 vion/vtrp/vare);再乘 κ_AR(τ_ρ, Δt_ρ);再加桥接项 `u_jᵀM Σ_D Mᵀu_j`、钟桥接项 `S_bδ + S_dδ³/3`、时间偏移项 `(u_jᵀw·σ_toff)²` |
| 多普勒 | `σ²_D,j = (λ·max(dopp_std, σ_D0)·k_el(el))²`;再乘 κ_AR(τ_D, Δt_D);再加桥接项 `(δ·(σ_ba + g·σ_θ))²`、钟项 `S_dδ`、杆臂项 `(‖ω_h‖σ_l)²`、时间偏移项 `(‖a_A‖σ_toff)²`、配准漂移率项 `(0.01‖v‖)²` |
| TDCP | `λ²(σ²_Φ(k0) + σ²_Φ(k1)) + u_jᵀMΣ_D1Mᵀu_j + (σ_rate·Δt_pair)² + σ_wu²`,其中 σ_Φ = max(cp_std, σ_Φ0/sin el),σ_rate 是电离层与轨道残余变化率(mm/s 级),k0 端的桥接噪声已计入 P_cc |

**AR(1) 等效方差因子**:`κ_AR(τ, Δt) = (1+ρ)/(1−ρ)`,`ρ = exp(−Δt/τ)`。它是 AR(1) 过程样本均值方差相对白噪声的放大倍数,用来抵消"把时间相关误差当白噪声反复平均"造成的过度自信。
- τ_ρ、τ_D 由开发序列上新息的自相关拟合得到(§17.4)。
- 同时配套抽取速率:伪距默认 1 Hz,多普勒 5 Hz。抽取历元按 GPST 取整规则选,保证确定性。

### 4.7 GNSS 更新是否迭代

GNSS 单独做一个 IEKF,先验固定为挂点入口时的状态 x_prior。第 i 次迭代:

```
δ_i = (x_prior ⊟ x_i) + K_i(z_i − H_i(x_prior ⊟ x_i))
K_i = P H_iᵀ(H_i P H_iᵀ + R)⁻¹
```

形式与上游 LIO 的 `vec + K(z − H·vec)`(voxel_map.cpp:470-472)相同,但增益用协方差形式、稠密 H。

- **何时迭代:** 仅当 σ_ψ > 2°,或第一轮 |Δψ| > 0.1° 时迭代,最多 3 次。R_z(ψ) 作用在长达数百米的 p^W 上,是唯一显著的非线性。
- **其余情况单步:** 视线方向对位置的非线性约 5e-8 rad/m,可忽略。
- **协方差:** 收敛后一次性做 Joseph 更新 `(I−KH)P(I−KH)ᵀ + KRKᵀ`,再对称化。
- **不与 LIO 联合迭代:** GNSS 近线性,顺序更新与联合更新在线性高斯情形下等价。联合迭代需要把 voxel_map.cpp:409-490 泛化成全维 H,还会在 LIO 无点提前返回(LIVMapper.cpp:343-347)时连带丢掉 GNSS。

---

## 5 "地图当真值"的一致性分析

### 5.1 信息量对比(推算,依据 code-update §6)

| 来源 | 信息量 |
|---|---|
| LIO | 每点方差 ≥ 1e-3 m²(voxel_map.cpp:449);N_eff = 300–3000 时,每轴信息 ≈ N_eff/(3R) ≈ 7e4–7e5 m⁻²,对应 σ_p ≈ 1–4 mm |
| 伪距 | 单星 σ 为 1–5 m;乘上 κ_AR 后等效 σ 为数米到十余米;30 颗星合计 ≤ 30 m⁻² |
| TDCP | 单对 σ 约 3–5 mm;十几颗星合计,量级与 LIO 的相对位移信息相当 |
| 多普勒 | 合计 σ 约 2–4 cm/s,与 LIO 经动力学间接给出的速度信息同量级 |

### 5.2 增益如何分配(给出公式与量级)

伪距的位置类列满足 `H_t = −u_jᵀ`、`H_p = −u_jᵀM`:两者方向相同,量测只能区分它们的先验。因此沿视线方向的修正按协方差比例分配:

```
Δp^W : Δt ≈ P_pp : P_tt^eff,   P_tt^eff = P_tt + |u×(Mp)|²·σ_ψ² + 钟/ISB 相关项
```

取 `P_pp = (4 mm)²`、`P_tt = (0.5 m)²`,得 `p^W` 份额约 6e-5,与 code-update 的"新息的 1e-4 到 1e-6"一致。各类量测的主要去向:

| 量测 | 主要进入 | 次要进入 |
|---|---|---|
| 伪距 | t、ψ(经杠杆 u×Mp)、b、ι | LIO 退化方向上的 p^W;φ/g 极小 |
| 多普勒 | d、ψ(速度方向)、v^W(量级相当,份额明显) | b_g、θ(杆臂项) |
| TDCP | ξ 的增量(克隆与当前位置之差)、ψ | 相对位移上的 v^W、p^W:两端 LIO 的相对误差约为 mm 级,而一对内 ξ 的游走约 `√(q_th·0.5 m)` ≈ 7 mm,所以 p^W/v^W 约分到一成(推算);退化方向上更多 |

### 5.3 为什么 P1(配准承载漂移)一致,P2(直接拉位姿)不一致

- **真实生成模型。** `g_true(t) = T(ξ_true(t))·p^W_true(t)`。其中 `p^W` 相对地图精确;`ξ_true` 是"当前所观测地图区域"的配准,会随进入新建地图区而缓慢变化。
- **P2 为什么行不通。** 地图不在状态里,`P_pp` 恒为毫米级。在一致的滤波器里,GNSS 拉不动位姿。要拉动它,只剩两条路:
  - 人为放大 P:新插入的点会与旧地图错位;
  - 接受不一致的约束:比如固定锚 TDCP(§6)。
- **P1 与"把地图放进状态"的关系。** P1 是其一致的近似:地图的全局误差由"当前区域配准"这个低维随机游走代表。

### 5.4 退化方向:自动接管

以长廊或空旷广场为例,LIO 在某个方向上约束很弱,`P_pp` 该方向分量随 IMU 传播长大。以加计参数 0.5 为例:1 s 后约 3 cm,10 s 后约 1 m(推算)。

当 `P_pp,axis ≳ P_tt` 时,增益自然转向 p^W,修正后的位姿随即进入地图插入(LIVMapper.cpp:413-424)。不需要 FAST-LIVGO 式(14)那种基于 Hessian 最小特征值的模式切换。

**风险:** LIO 协方差本身过度自信,把点间相关和地图误差都忽略了,可能导致接管不足。表现为修正仍流向 ξ:全局输出照样被修正,只是局部地图沿该轴的漂移与上游相同。§5.7 的增益分配日志会直接显示这种情况。

### 5.5 门宽由 P_ξξ 主导

伪距和多普勒的 `S = HPHᵀ + R` 中,`P_ξξ`(由真实漂移率标定)和 R 远大于 `P_pp`。所以即使 LIO 过度自信,也不会让 χ² 门错杀好的 GNSS。

这是 P1 相对 P2 的关键鲁棒性优势。FAST-LIVGO 的 Mode A 用的是 LIVO 先验协方差,因此需要在退化时关掉 χ² 检验。

### 5.6 回访旧地图

- **LIO 吸附到旧地图(p^W 跳变 Δ):**
  - 若 GNSS 在线:此前的 GNSS 更新建立了 `P_ξp ≈ −P_pp`(负相关),LIO 更新会使 ξ 同步变化 −Δ,全局位置 g 保持连续。正确。
  - 若 GNSS 中断:LIO 反复重测会使 `P_ξp` 衰减,ξ 的均值停留在中断开始时(随机游走增量零均值),而旧地图区域的真实配准正是它。吸附后 g 恰好被修正回来。正确。
  - 吸附幅度被 LIO 门控限制在厘米级(3σ 门,σ 为数厘米),落在 GNSS 门内。
- **不吸附、形成重影层**(漂移超过门宽,这是常见情况):当前层的配准 = ξ_old + 漂移,随机游走模型覆盖此情形,GNSS 恢复后修正。
- **失效情形:** 在旧区与新区之间反复穿梭,单一 ξ 无法同时代表两层配准。表现为新息偏大、被拒;兜底靠 §10 的"锁死恢复"。已列入局限;日志记录"LIO 修正量 / 预测 σ",让这类事件可见。

### 5.7 零成本诊断在实现前后的用法

**实现前(上游,确定性回放,2 条开发序列):**
- **D1**:在 LIVMapper.cpp:371 之后打印 `_state.cov.diagonal()`(可仿照 :263 被注释掉的同类打印),记录 σ_θ、σ_p、σ_v 的中位数、p95,以及退化段内的时间序列。
  - 预注册判据:σ_p 中位数落在 [1, 10] mm 即确认 P1 的前提。若 > 5 cm,则回到方法层重新评估路由(可能性低)。
- **D2**:按 §3.5 计算 RPE-路程曲线,得到 q 值。

**实现后:**
- **D3**:同样的打印,另加每次 GNSS 更新的增益分配日志:`f_p = ‖K_p z‖/‖Kz‖`,同理计算 f_ξ、f_clk、f_v。
  - 预注册:非退化段 f_p 中位数 < 1e-3;退化段 f_p 上升;GNSS 开与关两种情况下 σ_p 的分布无显著差异。
- **D4**:各量测类型的平均 NIS/dof 应在 [0.8, 1.25] 之内,95% 历元落在 χ² 界内。

### 5.8 用本模型解释 FAST-LIVGO 表 II 的三个符号,以及预注册预测

先解释已有的三个符号(推断):
1. **开阔天空全部下降。** TDCP 与多普勒提供全局系下毫米级的相对约束,修正了 yaw 和平移漂移,轨迹形状变好。
2. **两条夜间序列上升**(0.31→0.34、0.18→0.27)。GNSS 质量下降时,固定锚 TDCP 的过度自信加上直接拉位姿(P2),会把残余的坏观测以毫米级权重压进位姿和地图。
3. **去掉剔除后,夜间序列发散。** 一次周跳(约 19 cm)被当作毫米精度的位移,位姿跳变 19 cm,而 LIO 的门宽只有厘米级,数据关联随即崩溃。

本设计的预注册预测(可证伪):
- **(a)** T 相对 U:开阔序列多数胜出。T−tdcp 的收益明显变小。T−pr 在对齐后的 ATE 上与 T 相当,但在不做旋转对齐的全局指标上更差。
- **(b)** Dark01/02:T 不劣于 U,差距在集成离散之内。
- **(c)** T−reject 退化但不发散。
- **(d)** A-FA(本代码里复刻 FAST-LIVGO 更新方案)在夜间序列上复现"上升",其无剔除版发散。
- **(e)** L-PVT 在对齐 ATE 上与 U 无显著差异:米级 PVT 经 κ_AR 降权后,基本不影响轨迹形状。

---

## 6 固定锚 TDCP 为何不一致,以及克隆版的代价与收益

**标量推导。** 设真值为随机游走 `x_{k+1} = x_k + w_k`,TDCP 量测 `Δ_k = w_k + n_k`(噪声方差 σ²)。

- **固定锚**(式(11)):残差 `r = Δ_k − (x_{k+1} − x̂_k)`,x̂_k 被当作常数。
  - 滤波器因此把"x_{k+1} = x̂_k + Δ_k ± σ"当成一次**绝对**量测。当先验 P 远大于 σ² 时,后验 `P_{k+1} ≈ σ²`,与 k 无关。
  - 真实误差却是 `e_{k+1} = e_k − n_k`,即 `Var = k·σ²`。
  - 所以 NEES ≈ k,随历元数线性发散。
  - 后果一:门宽被压到 mm 级,好观测在累积漂移之后被错杀。
  - 后果二:没有门时,周跳和多径以 mm 级权重直接灌进位姿。
  - 后果三:伪距(m 级)在 P≈σ² 的滤波器里权重约为 0,等于无法一致地引入绝对信息——这恰好解释了 FAST-LIVGO 为何只把伪距用于 RAIM。
- **克隆版:** 量测模型为 `Δ_k = x_{k+1} − x_k(克隆) + n`,互协方差完整保留,`P_{k+1} = P_k + σ²` 正确增长。

**代价:**
1. 多 3 维状态,每对做一次增广雅可比计算,并维护少量簿记;
2. 克隆使 P 奇异,因此必须把 LIO/VIO 改成 push-through;
3. 为了噪声独立采用不重叠配对,损失约 1/(m+1) 的覆盖时间;
4. 若允许相邻配对共享相位,噪声是 MA(1)(相关系数 −0.5)。独立假设在"交替组合"方向上低估方差 1.5 倍,要严格保守需把方差乘 2。本设计选不重叠配对,不需要这个因子。

**收益:**
- 协方差与真实误差同步,χ² 门宽度正确;
- 伪距、TDCP、多普勒按各自精度一致地融合;
- 周跳和异常被门控隔离,即使漏过,也主要作用在 ξ 上,不打坏局部地图。

---

## 7 时间对齐

### 7.1 时间映射

```
t_gpst,true = t_rcv − b̂/c                    (b 可达 ms 级;激活前用 SPP 钟差)
t_est = t_gpst,true + gnss_time_offset + rate·(t_gpst − t_0)
```

- `gnss_time_offset` 默认 = −(GPST−UTC)(RTKLIB 闰秒表),再加上下面估计出的残余。
- `rate` 默认 0。离线工具从"GNSS 消息的 bag 记录时间 − GPST"的下包络做稳健线性拟合,若斜率 > 1e-5(主机钟相对 GPST 漂移),写入配置。

### 7.2 IMU 前向快照与桥接

- **快照位置。** 在 `UndistortPcl` 前向循环完成、状态写回之后(IMU_Processing.cpp:444 之后)保存 `IMUpose`、`prop_beg_time`、`prop_end_time`。
  - 快照必须早于 :490 的无点提前返回和 :538 的 `IMUpose.clear()`。
  - 否则无点帧的快照会丢失;上游在这种情况下还会残留一个未清空的 IMUpose,属上游缺陷,快照不受其影响。
- **插值与传播完全一致。** `Pose6D` 条目 i+1 保存的是区间 (t_i, t_{i+1}] 内的常加速度 `a_{i+1}`(世界系,含 g)和角速度 `ω_{i+1}`,与前向积分 :412-421 一一对应。在 τ = t_g − t_i 处:
  - `p = p_i + v_i τ + ½ a_{i+1} τ²`
  - `v = v_i + a_{i+1} τ`
  - `R = R_i Exp(ω_{i+1} τ)`

  由此算出 ΔR̂、D̂、Ê、ω̂_g。
- **迟到历元。** 维护覆盖最近 0.6 s 的环形缓冲(逐样本存 t、a_world、ω、R),可以跨步反向积分出 D̂、Ê、ΔR̂。δ 最大 0.5 s,超出则丢弃并记日志。
- **跨步的近似。** 跨步时姿态已被更新修正,约 mrad 级,对 D 的影响约 1e-4 m,计入桥接噪声。
- **桥接噪声(推算)。** `Σ_D ≈ (acc_n²δ³/3 + (½δ²(σ_ba + gσ_θ))²)·I`。以 MID-360 的 acc_n = 0.0115(M3DGR calibration.md)代入,δ = 0.067 s 时约 1e-4 m。伪距和多普勒可忽略,TDCP 需要计入。

### 7.3 偏移估计与漂移诊断

- **激活前估计(阶段 2)。** 在 ≥ 30 s、速度标准差 > 0.1 m/s 的窗口内,取多普勒 LS 天线速度模 |w_N| 与 LIVO 天线速度模 |v + Rω×l| 的归一化互相关:
  - 搜索范围为配置偏移 ±0.5 s;
  - 用抛物线插值做亚采样精化;
  - σ_toff 取自峰宽,或子窗口 bootstrap。

  本质上与 FAST-LIVGO 的 DTW 同义,但更简单。
- **激励不足时。** 保留配置偏移,σ_toff 取配置值(默认 50 ms),该不确定度自动进入噪声。
- **在线诊断(不自动改值)。** 对多普勒残差关于 `u_jᵀ·a_A` 做滑窗回归,斜率即残余时间偏移。超过 3σ 就告警。

### 7.4 同步质量对各类观测的影响(推算)

地面机器人速度在 1–2 m/s 量级:
- 10 ms 偏移造成伪距误差约 2 cm,可忽略;
- 多普勒误差约为 |a|·10 ms ≈ 1 cm/s,略有影响;
- TDCP 误差约为 a·Δt·δ ≈ 1 mm。

伪距和多普勒能容忍 ms 级误差,载波类观测需要把 σ_toff 计入噪声。

---

## 8 初始化与可观性

| 阶段 | 条件与动作 |
|---|---|
| 0 LIVO | 上游 IMU 静止初始化(IMU_Processing.cpp:569)。GNSS 状态停放;只解码、缓存,不参与更新 |
| 1 SPP | RTKLIB `pntpos`(pntpos.c:646,开启 RAIM-FDE,`estvel` 测速)。需要连续 ≥ 30 个历元满足:≥ 3+N_sys 颗星、GDOP < 5、有效解。取中位数定 `r_0` 与 `R_EN`,之后固定。选定主系统 |
| 2 时间与偏航 | 先按 §7.3 估计时间偏移;再在 ≥ 3 s、水平速度 ≥ 0.5 m/s 的窗口内,以多普勒观测对 (ψ, 每历元 d) 做 GN-LS(每历元的 d 作为冗余量投影掉),模型为 `−λD − 卫星项 = −u_jᵀ R_z(ψ)C(g)(v+Rω×l)^W + d`。只需速度方向,一段直线即可,不要求转向。条件:σ_ψ < 3°(post-fit 缩放后) |
| 3 激活(一致增广) | 以窗口末历元 k_a 的 SPP 为基础:`φ = 0 + n_φ`(σ_DOV);ψ 取阶段 2 的解;`t = a_SPP^N − M(ψ,φ,g)·y(x)`(y 含桥接);b、ι 取自 SPP 解的联合协方差块;d 取自 k_a 历元的多普勒 LS。按 §2.5 的增广公式写入 P,包括 t 对 x、ψ、φ 的互协方差 `−J_x P`、`−J_ψ σ_ψ²` 等。SPP 位置方差取 `max(formal·σ̂0², 3² m²)`,以覆盖电离层和多径偏差 |
| 4 运行 | k_a 及之前的历元全部用于初始化,不再作为更新量测,避免重复计数。从 k_a+1 起正常更新。TDCP 的第一对从 k_a+1 起算 |

**近似。** ψ 的 LS 用到了窗口内过去的 LIVO 速度,它与当前状态的互协方差被忽略,量级约 cm/s 除以速度再除以 √N(推断)。作为补偿,σ_ψ 采用 post-fit 残差缩放后的值。

**何时"放开"帧状态。**
- 激活即放开:ψ、t、φ 从激活时刻起就带着一致先验参与估计,过程噪声也同时生效。没有 GVINS 那种"低速固定 yaw"阶段。
- GVINS 固定 yaw 是为优化器的数值条件服务。EKF 在不可观时增益自然为 0,静止时配准噪声(按路程)也为 0,不会凭空变松。

**可观性条件:**

| 量 | 可观条件 |
|---|---|
| ψ | 多普勒:水平速度 v_h 越大越好,`σ_ψ ≈ σ_v/v_h/√N`。TDCP:一对内位移 ≥ 数厘米。伪距:需要远大于伪距噪声的位移(数十米) |
| t | 伪距,≥ 1 颗星即可沿视线方向起作用,有钟状态时成立 |
| b | 伪距;与垂向 t 强相关,由几何分开 |
| d | 多普勒 |
| φ | 高程随水平位移的变化(TDCP 垂向 + 伪距),可观性弱,由先验主导 |
| ι_s | 该系统与主系统同时可见 |
| MINS 引理 5.1 的 4 个不可观方向 | 本系统不存在。W 的规范被地图固定(地图即真值),不像纯 VIO 那样有局部 4-DoF 规范自由 |

---

## 9 更新调度

**挂点**(LIVMapper.cpp:248-265):

```
processImu():
  p_imu->Process2(LidarMeasures, _state, feats_undistort);        // :252(内部保存前向快照)
  if (gravity_align_en) gravityAlignment();                        // :254
  if (gnss_ && LidarMeasures.lio_vio_flg == LIO && !p_imu->imu_need_init)  // 新增
      gnss_->onLioStep(_state, p_imu->fwdSnapshot(), LidarMeasures.last_lio_update_time);
  state_propagat = _state;                                         // :256(LIO 先验 = GNSS 后验)
  voxelmap_manager->state_ = _state;                               // :257
```

`onLioStep` 的流程:
1. 快照入环形缓冲,计算 Δt、Δs、Δα。首个快照的起点不合法时跳过(见 §13 的 D2 修复)。
2. 解码待处理消息;星历和电离层参数进入 NavStore;观测历元和 PVT 按 GPST 入队。
3. 已激活时,传播 GNSS 块:钟(§3.3)、ISB、ξ(§3.5,J_rw 取当前点 `u = M p_k`)。
4. 对所有 `t_est ≤ t_k` 的待处理历元,按 GPST 顺序:
   1. L0 掩码,L1 RAIM;
   2. 按启用的观测类型和抽取表构造各行;
   3. 逐星 χ² 检验与历元级排除(§10);
   4. IEKF 加 Joseph 更新;
   5. L4 提交检查,失败则回滚;
   6. 克隆管理:终点历元更新后停放克隆,起点历元更新后创建克隆;
   7. 写诊断。

   尚未激活时,把历元喂给初始化器,满足条件即执行增广。

**步末挂点**(LIVO 模式放在 handleVIO 末尾 :333;纯 LIO 模式放在 handleLIO 末尾 :481):
- 用最终 g 刷新 `C_ref`;
- 写 `traj_W` / `traj_N` / `traj_ant_N`,均为步末也就是 VIO 之后的状态,用传感器时间 t_k;
- 发布 TF `enu→camera_init` 与全局里程计;
- 写 P 对角线日志。

上游的 evo 文件(LIO 后、VIO 前,:384-405)保持不变。

**顺序取舍。** 按上面的挂点,每个历元的处理顺序是 GNSS → LIO → VIO,而不是把 GNSS 放到 VIO 之后。理由:
- 在 P1 下,GNSS 对 p^W 的改动在非退化方向可以忽略,不会干扰 LIO 数据关联或 VIO 的收敛域;
- 在退化方向上,GNSS 先行正好帮助 LIO 关联;
- 地图插入(:413-424)使用的是含 GNSS 信息的后验。

**push-through(必改)。**
- LIO(voxel_map.cpp:468-472):令 A = `H_T_H[:6,:6]`,
  - `K16 = P[:, :6]·(I_6 + A·P[:6,:6])⁻¹`,用 LDLT/LU 解一个 6×6 方程;
  - `G[:, :6] = K16·A`;
  - `solution = K16·HTz + vec − G[:, :6]·vec[:6]`。
- VIO(vio.cpp:1661-1667 与 1497-1501):用 `P' = P/img_point_cov` 代换,形式同上(7 列或 6 列)。
- 协方差仍为 `(I−G)P`(:489-490,:800),之后加对称化 `P = ½(P+Pᵀ)`。
- 数学等价性(Woodbury):`(P⁻¹ + EAEᵀ)⁻¹E = PE(I + A·P_kk)⁻¹`。由于 `A·P_kk` 的特征值 ≥ 0,`I + A·P_kk` 的特征值 ≥ 1,条件数良好。
- code-update §4(c) 的数值试验:相关病态下误差 ≤ 9e-15,存在零方差态时仍正常;信息形式在这两种情况下分别出现 2.4e-3 的误差和 NaN。

---

## 10 异常剔除(全部为 0/1 成员资格)

| 层 | 判据 | 说明 |
|---|---|---|
| L0 信号 | 仰角 ≥ 15°;C/N0 ≥ 30 dB-Hz;`status` 位的 psr/cp 有效;psr_std、dopp_std、cp_std 不超上限;卫星健康、URA 正常;星历龄期正常;可选排除 GLONASS 或 BDS GEO | 阈值来自开发集,冻结后进入评测 |
| L1 GNSS 内部 | `pntpos` 的 χ² 失败且 n ≥ 6 时执行 `raim_fde`(pntpos.c:472),被排除的卫星本历元不用;GDOP 检查 | 这是**外部判决信号**,不依赖 LIVO |
| L2 先验一致性 | 逐星 `ν_j²/S_jj < χ²_1(1−α)`(α=1e-3,阈值 10.83);随后做历元 NIS 检验;失败则逐个剔除归一化新息最大的卫星,最多剔除 30% 且至少保留 4 颗;仍失败则整历元拒绝。TDCP 先减去稳健 Δclk 再逐星检验 | S 由 P_ξξ 主导,见 §5.5 |
| L3 周跳(TDCP) | 对内任一历元出现 LLI 滑动或半周位;cp_std 超限;多普勒积分相位检验 `|ΔΦ_meas − (−∫D dt)| > max(0.5 周, 4σ)`;双频时加 GF 检验;最后由 L2 的 χ² 把关(LIVO 预测位移精确到 mm,19 cm 的周跳必被拒) | gnss_comm 消息没有锁定时间字段,只能依赖 LLI、status 与检验量 |
| L4 提交后 | 后验 NIS(`S⁺ = R − HP⁺Hᵀ`)超过 χ² 阈值,或 `|Δp^W|` 同时超过 3σ_prior,p 和 5 cm,则回滚、不提交 | 仿照 VIO 的回退(vio.cpp:1677-1681) |
| L5 钟跳 | 伪距新息中位数超过 1e4 m,或 TDCP 的 Δclk 跳变超过 1e4 m:用中位数重新初始化 b,方差取 MAD²,清零其互协方差 | 针对 u-blox 1 ms 钟跳;TDCP 已天然免疫 |
| L6 锁死恢复 | 连续 20 个历元都有 RAIM 一致的 SPP(≥ 5 星),但 L2 拒绝超过 50%:用 SPP 重新增广平移 t 和钟;若多普勒也被系统性拒绝,则按阶段 2 重做 ψ;同时停放克隆,记录 REANCHOR 事件 | 仅在含伪距的臂启用;T−pr 臂禁用 |

**否决的做法:**
- IGG-III/Huber 连续调权:会把"成员资格"问题变成调权问题,不透明,也难以审计一致性;
- FAST-LIVGO 按 Hessian 最小特征值切换双模:由 §5.4–5.5 的自动机制取代。

---

## 11 GNSS 中断与恢复

- **中断判定。** 超过 1 s 没有 L0 有效历元即进入 OUTAGE。TDCP 连续性一断,克隆就停放。
- **协方差演化:**
  - `P_ξξ(s) = P_ξξ(s_0) + ∫J_rw Q J_rwᵀ ds`。当前点全局位置的方差按 `q_th·s + q_ψ·s³/3`(横向)增长。
  - `P_bb(Δt) = P_bb + 2P_bdΔt + P_ddΔt² + S_bΔt + S_dΔt³/3`。取 S_d = 0.1 时,60 s 后 σ_b 约 85 m(推算),接收机钟差基本"遗忘"。
  - ISB 只以 q_isb 缓慢增长,得以保留。
- **重新捕获:**
  - σ_b > 1e3 m 时,先以预测的天线位置做单历元钟差 LS 重置(按一致重置处理)。
  - 此后按正常门控更新,门宽已随 `P_ξξ` 增大。
  - 即便只有 1 到 3 颗星,也能用伪距和多普勒:钟有连续性,ISB 保留。
  - TDCP 需要连续两端锁定后才重新开对。
- **GNSS-denial01/02。** 按 M3DGR 描述,这两条序列开头有星、中途拒止、最后回到起点区域(uf.txt 第 527-528 行附近)。
  - 拒止期间 ξ 的均值不变,不确定度增长。
  - 回到起点时,若 LIO 吸附到旧地图,全局位置即刻正确(§5.6);若不吸附,由 GNSS 修正。

---

## 12 底座与 I/O

### 12.1 RTKLIB-EX vendoring

- **版本与位置。** 锁定 tag v2.5.1,放在 `third_party/rtklib_ex/`,保留 `license.txt`(BSD-2)。自写 CMake,把 `src/*.c` 和 `src/rcv/*.c` 整体编成静态库。不使用上游 CMake:它会编共享库和可执行程序,并写入源码树(substrate §3.1)。
- **编译宏。** `target_compile_definitions(rtklib_ex PUBLIC ENAGLO ENAQZS ENACMP ENAGAL ENAIRN NFREQ=3 NEXOBS=3)`。`obsd_t`/`nav_t` 的内存布局依赖这些宏,不一致会造成静默内存错误(rtklib.h:159/164)。
- **头文件隔离。** 只允许 `gnss/src/rtklib_adapter.cpp` 这一个翻译单元包含 `rtklib.h`(pimpl),防止 PI、CLIGHT、MAXOBS 等无前缀宏泄漏到其他代码。
- **应用侧桩。** 提供 `showmsg`、`settspan`、`settime` 等应用侧桩函数。
- **单线程约束。** RTKLIB 有全局可变量 `eph_sel`、`leaps`、`timeoffset_`,所以 GNSS 处理限定在主线程。FAST-LIVO2 的 run 循环本就是单线程。
- **补丁**(写入 `PATCHES.md`,保留 BSD 声明;并对 `pntpos()` 做逐位回归):
  - **P1** rtklib.h:211 `MAXPRNCMP 50→63`,以支持 BDS-3 GEO C59–C63(GEO 分支 ephemeris.c:265 已支持)。编译期断言 `MAXSAT ≤ 255`,因为 `obsd_t.sat` 是 uint8。
  - **P2** pntpos.c:导出包装 `rtkx_prange()` 与 `rtkx_varerr()`,原函数是 static(:110/:51)。
  - **P3** ephemeris.c:导出 `rtkx_satpos_iode()`,可按指定 IODE 计算卫星状态(原 `ephpos`/`seleph` 为 static,:432),保证 TDCP 两端星历一致。
- **使用的函数:**
  - 卫星:`satposs`/`satpos`(:781/:737)、`eph2pos`/`geph2pos`;
  - 几何:`geodist`(Sagnac)、`satazel`;
  - 改正:`ionocorr`/`ionmodel`、`tropcorr`/`tropmodel`;
  - SPP:`pntpos`/`estvel`/`raim_fde`;
  - 编号与频率:`satno`/`satsys`/`sat2freq`/`code2idx`;
  - 时间:`gpst2time`/`gpst2utc`/`timeadd`/`timediff`;
  - 数据读入:`readrnxt`、`input_ubx`/`input_ubxf`。
- **已知缺口:** BDS 没有专用电离层模型,Galileo 没有 NeQuick,统一用 GPS Klobuchar 按频率缩放(见局限)。

### 12.2 不链接 gnss_comm 读取 M3DGR 消息

- **做法:schema 驱动的 ROS1 线格式解码。**
  - ROS1 bag v2.0 的连接记录内嵌 `type`、`md5sum`、`message_definition`(含嵌套的 `MSG:` 段)。
  - `gnss/io/msgdef_parser.cpp` 依据 ROS1 序列化规范解析定义文本,支持内建类型、`time`、`string`、定长与变长数组、`Header`,生成 schema。
  - `ros1_wire_decoder.cpp` 按 schema 把原始字节解成值树。
  - `gnss_comm_adapter.cpp` 以字段名为查找键,映射到自有的 `ObsEpoch`、`eph_t`、`geph_t` 等结构。
- **两条数据来源:**
  - 回放:`rosbag::MessageInstance::getMessageDefinition()` 加 `write()` 取原始字节;
  - 在线:`topic_tools::ShapeShifter`(BSD,ros_comm)订阅任意类型,用 `getMessageDefinition()` 和 `write()` 取字节。
  - 两者都不需要在编译期依赖 gnss_comm。
- **许可证理由(工程判断,非法律意见):**
  1. 仓库中没有 gnss_comm 的任何代码或 `.msg` 文本,构建时既不编译也不链接它;
  2. 消息定义随数据集一起提供,属于数据(M3DGR 为 MIT);
  3. 依据公开的序列化规则实现一个数据格式读取器属于互操作;字段名和字段顺序是功能性事实,而且本方案在运行时从数据中读取它们,不在代码里转录;
  4. 编号换算用 RTKLIB 的 `satno`,不复用 gnss_comm 的 `sat_no`/`satsys` 实现。
- **否决的方案:**
  - 链接 gnss_comm:GPL-3.0 与 GPL-2.0-only 不兼容(FSF FAQ #v2v3Compatibility);
  - 另起 GPL-3 进程预处理再经话题转发:属于灰区,且没有任何技术收益;
  - 复刻 `.msg` 的兼容包:等于复制 GPL-3 文本,而且其中 7 个字段名违反 rosidl 命名规则;
  - 以离线 RINEX 为唯一通路:会丢掉精确数值和到达延迟。RINEX 导出只保留为验证工具:`tools/gnss_bag_to_rinex.py`,MIT,运行时调用 Apache-2.0 的 rosbags,属于独立程序。

### 12.3 编号、频率与星历映射

- **卫星编号。** gnss_comm 的 `sat` 分段为:1–32 为 GPS;33–59 为 GLONASS(prn = sat−32);60–97 为 Galileo(prn = sat−59);98–160 为 BDS(prn = sat−97)。依据是该格式的常量 G32/R27/E38/C63。换算后再用 RTKLIB `satno(sys,prn)`。
- **信号分类。** 不依赖 `code` 的枚举,因为 RTKLIB 各版本的编号可能不同;改为按 `freqs`(Hz)分类:
  - 1575.42 MHz:GPS L1C、Galileo E1、BDS B1C;
  - 1561.098 MHz:BDS B1I,对应 TGD_B1I;
  - G1 频段:GLONASS,频道号由频率反算。

  每颗星只取主信号(L1 类)作伪距、多普勒、TDCP;第二频点仅用于 GF 周跳检验。
- **GPS/GAL/BDS 星历(`GnssEphemMsg`→`eph_t`):**
  - `toe`/`toc`/`ttr` 在消息中以 GPST 周和周内秒存储,包括 BDS(gnss_ros.cpp 的 `msg2ephem` 用 `gpst2time`),直接转成 `gtime_t`;
  - `toe_tow`→`toes`;
  - `tgd0`/`tgd1`→`tgd[0]`/`tgd[1]`;
  - `ura` 是米还是索引未核实:运行时判定,若为整数且 ≤ 15 视为索引,否则按米转成索引。本设计不用 URA 定权,只用于健康检查。
- **GLONASS 星历**(`GnssGloEphemMsg`→`geph_t`):`freqo`→`frq`,`tau_n`→`taun`,`gamma`→`gamn`,位置、速度、加速度逐项对应。
- **其他话题:**
  - `iono_params`(8 个值)→`nav.ion_gps`;
  - `receiver_pvt`→松耦合臂与自检;
  - `time_pulse_info` 仅用于诊断。

### 12.4 其它输入适配器

- **RINEX**(`readrnxt`,rinex.c:1780):用于单元测试和其它数据集(UrbanNav、GREAT)。回放时按 `t_est + 配置延迟` 并入事件流。
- **UBX 字节流**(`input_ubx`,rcv/ublox.c:1437):为将来的 F9P 真机准备。自定义 BSD 许可的消息 `RawStream{header, uint8 format, uint8[] data}`,核心接口为 `feedBytes(format, bytes, t_host)`。

### 12.5 与 ROS 无关的核心接口(`gnss/include/livo_gnss/`)

```cpp
namespace livo_gnss {
struct StateLayout { int rot=0,pos=3,vel=7,bg=10,ba=13,grav=16,tilt=19,yaw=21,trans=22,clk=25,drift=26,isb=27,clone=30,dim=33; };
struct NominalState { Eigen::Matrix3d R; Eigen::Vector3d p,v,bg,ba,g; Eigen::Vector2d tilt; double yaw;
                      Eigen::Vector3d t; double clk,drift; Eigen::Vector3d isb,clone; };
struct ImuSample { double t; Eigen::Vector3d a_world, omega; Eigen::Matrix3d R; Eigen::Vector3d p,v; };
struct ImuTrajectory { double t_beg,t_end; std::vector<ImuSample> s; };
class Estimator {
 public:
  Estimator(const Config&, const StateLayout&);
  void onRawMessage(const RawMsg&);                 // 解码后入队:ObsEpoch / Eph / GloEph / Iono / Pvt
  StepReport onLioStep(double t_k, const ImuTrajectory&, NominalState&, Eigen::Ref<Eigen::MatrixXd> P);
  GlobalPose globalPose(const NominalState&, const Eigen::MatrixXd& P) const;  // T_{N←I}、协方差、LLA
  void onStepEnd(const NominalState&);              // 刷新 C_ref
  const Diagnostics& diag() const;
};
}
```

- 核心在内部完成 ⊞(右扰动,ψ 取模),适配层负责 `StatesGroup ↔ NominalState` 的拷贝。
- 日志通过回调输出,核心不依赖 glog 或 ROS。
- ROS2 移植时只需重写胶水层:rclcpp 订阅加 CDR schema 解码,或自有消息。

### 12.6 入库自检(静默通过后才算有效运行)

1. **解码计数:** 每个历元的卫星数、频点分布、历元间隔(10 Hz);`freqs` 与 `sat2freq` 一致。
2. **符号:** 对每颗星检验 `λ·dΦ/dt` 与 `−λD` 的相关系数 > 0.99,斜率接近 1,确认载波和多普勒约定。
3. **SPP 与接收机 PVT:** 开阔段水平差的中位数 < 3 m;速度差 < 0.2 m/s。
4. **时间偏移:** `|offset_est + 18 s| < 0.3 s`,且互相关峰清晰。
5. **杆臂方向:** 在转弯段比较 l 与 −l 两种假设下的多普勒残差 RMS,预期差异约为 ω·|l_h| ≈ 0.18 m/s(推算),据此检验 calibration.md 中天线条目的方向约定。
6. **确定性:** 同一配置跑两次,输出的 sha256 相同。
7. **GNSS 关闭时的等价性:** F0 与 U 的集成对比无显著差异(§18)。

任一项未过,则不产出评测数字。

---

## 13 代码改动清单(相对上游 @0d2c034)

| 文件 | 位置 | 改动 | 性质 |
|---|---|---|---|
| include/common_lib.h | :30 | `DIM_STATE 19→33` | 扩维 |
| 同上 | :128-140、:142-152、:154-165、:167-180、:182-192、:194-206、:215-222 | StatesGroup 新成员、6 个手写函数、新块停放协方差、ψ 取模 | 扩维 |
| include/state_index.h(新) | – | 命名索引与 `static_assert` | 新增 |
| include/IMU_Processing.h | :53 附近 | public `ImuFwdSnapshot fwd_snapshot_` 及 getter | 挂点 |
| 同上 | :80 | `last_prop_end_time` 增加"未初始化"标志 | **D2**:确定性修复 |
| src/IMU_Processing.cpp | :444 之后 | 保存前向快照(IMUpose、beg、end、LIO 标志) | 挂点 |
| 同上 | :248 | 首个 LIO 步时,若 `last_prop_end_time` 未初始化则取 `lidar_meas.last_lio_update_time`,恢复上游不变量 `last_prop_end_time == last_lio_update_time` | **D2** |
| src/voxel_map.cpp | :466-472 | push-through 增益 | 数学等价改写 |
| 同上 | :489-490 | 协方差更新后对称化 | 数值 |
| src/vio.cpp | :1552-1555、:1632-1636 | 并行循环写 `errors[i]` 与逐点计数,循环外按下标串行求和,替代 `reduction(+:error,n_meas)` | **D1**:确定性 |
| 同上 | :1660-1667、:1494-1501 | push-through(7 列 / 6 列) | 数学等价改写 |
| 同上 | :800 | 对称化 | 数值 |
| include/LIVMapper.h | 成员区 | `std::unique_ptr<GnssBridge> gnss_`;`bool processOnce()`;GNSS 订阅和发布者 | 胶水 |
| src/LIVMapper.cpp | readParameters :50-117 末尾 | `gnss_->loadConfig(nh)` | 胶水 |
| 同上 | initializeComponents :119-167 | 构造 GnssBridge(传入 extR/extT、杆臂) | 胶水 |
| 同上 | initializeSubscribersAndPublishers :192-217 | ShapeShifter 订阅 GNSS 话题;发布 `/gnss/global_odom`、`/gnss/global_path`、`/gnss/diagnostics`、TF `enu→camera_init`、按步配准的全局点云 | 胶水 |
| 同上 | processImu :254→:256 之间 | GNSS 挂点 | 核心挂接 |
| 同上 | handleVIO :333 末尾、handleLIO :481 末尾(仅纯 LIO 模式) | `gnss_->onStepEnd()`:步末轨迹与 TF | 输出 |
| 同上 | run :534-554 | 循环体抽成 `processOnce()`,行为不变 | 回放复用 |
| include/gnss_bridge.h、src/gnss_bridge.cpp(新) | – | StatesGroup↔核心;快照→ImuTrajectory;ROS 参数、订阅、发布 | 胶水 |
| src/replay_main.cpp(新) | – | 确定性回放可执行程序(§14) | 新增 |
| gnss/(新,ROS 无关库 `livo_gnss`) | – | estimator、process_model、meas_{pseudorange,doppler,tdcp,pvt}、update_engine、initializer、integrity、time_sync、leveling、bridge、clone_manager、nav_store、rtklib_adapter、io/{msgdef_parser,ros1_wire_decoder,gnss_comm_adapter,rinex_source,ubx_stream}、test/、sim/ | 新增 |
| third_party/rtklib_ex/(新) | – | v2.5.1 源码、license.txt、PATCHES.md(P1–P3)、自写 CMakeLists | vendoring |
| CMakeLists.txt | :108-133 | 新增 `livo_gnss`、`rtklib_ex`、`fastlivo_replay`、gtest 目标;依赖 rosbag、topic_tools | 构建 |
| package.xml | :15 | `<license>GPL-2.0-only</license>`;新增 rosbag、topic_tools 依赖 | 许可证修正 |
| config/m3dgr_{avia,mid360}_gnss.yaml、launch/*.launch、tools/(Python,MIT)、docker/ | – | 配置、启动、评测与标定脚本 | 新增 |

**上游基线臂 U 的构建:** 上游 0d2c034,只加 D1、D2、`processOnce` 和回放器,估计器数学保持原状。

---

## 14 确定性离线回放(`fastlivo_replay`)

1. **初始化。** 用 roscore 加 rosparam 装载同一份 YAML。vikit 的 `camera_loader` 依赖参数服务器,无法绕开。
2. **读取。** 用 `rosbag::View` 按**记录时间**遍历(可选 `replay.order: header`)。逐条分发:
   - Livox `CustomMsg`:经 `instantiate` 按 md5 匹配,livox_ros_driver 和 livox_ros_driver2 的定义相同,可以兼容;
   - IMU;
   - `CompressedImage`:用 `cv::imdecode` 转 BGR8 后进入 `img_cbk`;
   - GNSS 原始字节。

   每条消息分发之后执行 `while (mapper.processOnce()) {}`,也就是"数据一到齐就立即处理"。这是实时模式的理想极限,且与时序无关。
3. **消除所有非确定源:**
   - VIO 浮点归约定序(D1);
   - 修复未初始化读(D2);
   - `BuildResidualListOMP` 按下标写槽位,本来就是确定的(voxel_map.cpp:658-701);
   - 关闭 `imu_prop` 定时器;
   - 不读墙钟,`omp_get_wtime` 只用于计时打印;
   - 发布函数有副作用(`publish_frame_world` 会清空 `pcl_w_wait_pub`,LIVMapper.cpp:1289),因此照常执行,只是没有订阅者。
4. **种子化扰动(用于集成)。**
   - `replay.seed = s ≥ 1` 时,第 i 帧图像的时间戳加 `J·(2U_i − 1)`,其中 `U_i = splitmix64(s ⊕ i)/2⁶⁴`,J = 1 ms,低于 D435i 软同步的精度;
   - 同一个 s 对所有臂施加完全相同的扰动,即公共随机数;
   - s = 0 为无扰动的规范运行。
5. **输出与清单。**
   - 输出:`traj_W.tum`、`traj_N.tum`、`traj_ant_N.tum`、GNSS 日志、P 对角线、上游 evo 文件;
   - `manifest.json` 记录 git 哈希、配置哈希、bag 的 sha256、seed、CPU 型号、二进制哈希。
6. **验收与调度。**
   - 验收:同一 seed 重复运行两次,所有输出(计时列除外)的 sha256 相同;另用 valgrind memcheck 对回放二进制检查未初始化读。
   - 精度运行与时序无关,可以在批内并行;计时运行必须独占机器。

---

## 15 配置(`gnss:` 命名空间;带星号的项由开发集标定,冻结后计算哈希)

```yaml
gnss:
  enable: true
  mode: tight              # off | loose_pvt | loose_spp | tight | livgo_like(A-FA)
  use: {pseudorange: true, doppler: true, tdcp: true}
  input:
    source: gnss_comm_ros1  # gnss_comm_ros1 | rinex | ubx_stream
    topics: {meas: /ublox_driver/range_meas, ephem: /ublox_driver/ephem, glo_ephem: /ublox_driver/glo_ephem,
             iono: /ublox_driver/iono_params, pvt: /ublox_driver/receiver_pvt, tp: /ublox_driver/time_pulse_info}
    rinex: {obs: "", nav: "", latency_s: 0.0}
    carrier_phase_sign: +1  # 由自检确认
  time: {leap: auto, offset_extra_s: 0.0, rate: 0.0, auto_estimate: true, search_s: 0.5, sigma_s: 0.05}
  frames:
    lever_arm_I: [-0.316, 0.109, 0.401]   # Avia 内置 IMU 系;MID-360 IMU 系为 [-0.285, 0.001, -0.211](按 calibration.md 链式推算,方向待 §12.6-5 检验)
    lever_arm_sigma: 0.03
    enu_ref_ecef: auto
    sigma_dov_rad: 1.5e-4
  systems: {gps: true, glo: true, gal: true, bds: true, qzs_as_gps: true, bds_geo: true, master: auto}
  masks: {elev_deg: 15*, cn0_dbhz: 30*, max_psr_std: 5.0, max_dop_std_hz: 1.0, max_cp_std_cyc: 0.1}
  noise:
    pr:   {a: 0.3, b: 0.3, efact: {G: 1.0, R: 1.5, E: 1.0, C: 1.0}, k_std: 1.0, tau_s: 10.0*, rate_hz: 1.0}
    dop:  {sigma0_hz: 0.05*, tau_s: 0.2*, rate_hz: 5.0}
    tdcp: {sigma0_cyc: 0.02*, interval_s: 0.5, sigma_rate_mps: 0.002*, sigma_windup_m: 0.002}
    pvt:  {tau_pos_s: 30.0*, tau_vel_s: 1.0*, pos_rate_hz: 1.0, vel_rate_hz: 5.0}
  process:
    clock: {S_b: 0.1*, S_d: 0.1*}
    isb_q: 1.0e-6
    frame: {q_tilt_per_m: 1.0e-8*, q_yaw_per_m: 1.0e-7*, q_yaw_per_rad: 1.0e-5*,
            q_trans_h_per_m: 1.0e-4*, q_trans_v_per_m: 1.0e-4*, q_time: 0.0}
  init: {min_epochs: 30, max_gdop: 5.0, min_hspeed: 0.5, min_window_s: 3.0, yaw_sigma_max_deg: 3.0, spp_sigma_floor_m: 3.0}
  gating: {alpha: 1.0e-3, max_excl_frac: 0.3, min_keep: 4, post_check: true, lockout_epochs: 20}
  iterate: {max_iter: 3, yaw_sigma_deg: 2.0, dyaw_deg: 0.1}
  ablation: {frame_fixed_after_init: false, tdcp_fixed_anchor: false, disable_l2_l4: false}
  output: {traj: true, diag_csv: true, sat_csv: false, cov_diag: true, global_cloud: true}
replay: {seed: 0, jitter_img_s: 0.001, order: record, publish: true}
```

---

## 16 日志与诊断

- **`gnss_epoch.csv`**(每个历元一行):
  - 时间:t_gpst、t_est、δ;
  - 卫星:各系统卫星数,伪距/多普勒/TDCP 各自的使用数与拒绝数及原因码;
  - 一致性:先验 NIS 与自由度、后验 NIS;
  - 增益分配:f_p、f_ξ、f_clk、f_v;
  - 状态:ξ 各分量及其 σ、b、d、ι;
  - 对照:SPP 与滤波天线位置之差;
  - 事件:RAIM 排除的卫星、迭代次数、是否回滚。
- **`gnss_step.csv`**(每步一行):Δs、Δα、GNSS 块传播量、快照长度、处理耗时。
- **`cov_diag.csv`**:LIO 后、VIO 后、GNSS 后三处的 σ_θ、σ_p、σ_v、σ_ξ(即 D1/D3)。
- **`gnss_events.log`**:ACTIVATE、REANCHOR、CLOCK_JUMP、LOCKOUT、SLIP、LATE_EPOCH、TOFF_WARN。
- **`summary.json`**:各量测类型的平均 NIS 与覆盖率、接受率、耗时分位数(p50/p95/p99)。
- **ROS 可视化**:`/gnss/diagnostics`(diagnostic_msgs);RViz 配置同时显示 W 与 enu 两个系,以及各自的轨迹和全局配准点云。

---

## 17 测试

### 17.1 单元测试(gtest,不依赖 ROS)

1. **时间:** GPST/UTC/BDT 互换;按日期查闰秒;`t_est` 映射的符号。
2. **msgdef 解析与线格式解码:** 用合成字节测试;另用一条真实 M3DGR 消息,与 rosbags Python 的解码结果逐字段比对(黄金文件)。
3. **编号往返:** 160 个 gnss_comm 编号 → (sys, prn) → `satno` → 再换算回来。
4. **星历映射:** 由映射后的 `eph_t` 算出的卫星位置,与 RTKLIB 读 RINEX 导航文件的结果差 < 1e-6 m。
5. **SPP 包装:** 与 `pntpos()` 逐位一致(验证补丁 P2 的回归)。
6. **雅可比:** J_a、J_w、TDCP、增广、C(g)、桥接、J_rw,全部与流形上的中心差分比对,相对误差 < 1e-6。δb_g 在桥接中被忽略的项需放宽容差。
7. **调平:** `C·ĝ = −e_z`;调平增量的 z 分量为 0,达到机器精度。
8. **钟离散:** 与 Van Loan 矩阵指数结果一致。
9. **J_rw:** 施加噪声后,当前全局点的均值与方差不变。
10. **push-through:** 与信息形式等价,覆盖随机 SPD 矩阵、相关病态(条件数 1e9–1e14)和零方差态。
11. **Joseph 更新:** 更新后 P 保持正半定。
12. **钟差增量投影:** 与"单差 + 白化"的解一致。
13. **克隆 TDCP:** 在两历元线性玩具问题上,与批量平滑解一致。
14. **门控与排除:** 注入粗差、钟跳、周跳,检查判定结果。

### 17.2 仿真 Monte-Carlo 一致性(核心验证,50 次)

- **仿真器(ROS 无关):**
  - 真实地面轨迹约 1 km,速度 1–2 m/s,含转弯;
  - IMU 按 M3DGR 的 Allan 参数生成;
  - GNSS 用真实星座几何,伪距含一阶高斯-马尔可夫多径(τ = 30 s,σ = 1.5 m)、电离层残余、白噪声;载波含模糊度、周跳和多径;另模拟中断、1 ms 钟跳、只有 2 到 3 颗星的时段;
  - LIO 用"相对漂移地图的位姿量测"模拟:地图系按 q_true 做随机游走,量测噪声为 mm 级。LIO 更新直接调用 fork 的 push-through 代码,H 取前 6 列。
- **判据:**
  - ξ、钟、全局天线位置的平均 NEES 落在 χ² 95% 双侧界内;
  - 各类 NIS 同样落在界内。
- **情景:**
  - 开阔天空;城市多径;中断 120 s 后恢复;钟跳;周跳;4 系统加 ISB;少星;
  - 长时间静止:检验相关误差造成的过度自信;
  - 回访吸附:LIO 跳变 Δ;
  - 模型失配:确定性偏航漂移率、地图重影。
- **对照:** 同一仿真跑固定锚版,预期其 NEES 随时间增长(§6)。

### 17.3 集成测试(docker,ROS1)

1. 确定性回放逐比特复现;
2. F0 与 U 在 2 条开发序列上的集成等价性;
3. §12.6 的全部入库自检;
4. 实时 1× `rosbag play` 时,GNSS 每历元 p99 耗时 < 5 ms,缓冲不增长。

### 17.4 开发集上的闭式标定(不扫参)

1. D1/D2(§5.7);
2. 配准游走率 q(§3.5);
3. τ_ρ、τ_D:对 GNSS 激活后相对 LIVO 预测几何的新息做自相关拟合;
4. σ 比例:使开发集上的平均 NIS/dof ≈ 1,这是单个标量的闭式计算;
5. S_b、S_d:逐历元多普勒 LS 钟漂序列的 Allan 分析;
6. 掩码阈值:由新息分布的尾部决定。

完成后配置冻结、计算哈希,之后不再触碰评测序列来调参。

---

## 18 评测协议

### 数据

- **评测序列:** FAST-LIVGO 用过的 7 条(Varying-illu03/04/05、Outdoor01/04、Dark01/02,均有 RTK 真值;注意 Dark 在 M3DGR 中归为夜间户外,不是 GNSS 拒止),加上 GNSS-denial01/02(ArUco 起终点真值),合计约 82 GB。
- **开发/标定集(与评测集不相交):** 推荐 Occlusion03/04、Dynamic03、Grass02,约 17 GB;可扩展到 Dynamic04、Grass01、Z-Rough-Road01。
- 数据放在外置 SSD;docker 的 data-root 需要迁出根分区。

### 对比臂

| 臂 | 定义 |
|---|---|
| U | 上游 FAST-LIVO2,同一回放器,只加 D1/D2 |
| F0 | fork,GNSS 关闭,检验噪声地板 |
| L-PVT | 松耦合:接收机 PVT 的位置与速度,同样使用配准状态、κ_AR 和门控 |
| T | 紧耦合全量 |
| T−pr | 去掉伪距更新行。SPP 仍用于初始化和 RAIM;不启用重锚;b/ι 停放 |
| T−dop | 去掉多普勒 |
| T−tdcp | 去掉 TDCP(克隆停放) |
| A-FA | 本代码中的 FAST-LIVGO 式更新:激活后配准固定;固定锚相邻 TDCP;伪距只用于 RAIM |
| T−reject | 去掉 L2 与 L4,只保留 L0 和 RAIM,对应 FAST-LIVGO 的"w/o Rejection" |

可选:L-SPP(RTKLIB SPP 当位置量测)、A-FA−reject。

### 重复与配对

- 每臂每序列 10 个种子(s = 1..10),各臂共用同一组种子。
- 预注册的试点规则:在 2 条开发序列上,若确定性集成的 ATE 四分位距不到实时集成(U 跑 5 次)四分位距的 25%,则改用实时运行作为重复单元,并相应改为非配对检验。

### 指标

1. **ATE RMSE**,仅位置:`evo_ape tum -a`(SE(3) Umeyama,不含尺度,与 M3DGR/FAST-LIVGO 一致),`--t_max_diff 0.05`。
   - GNSS 臂评测 `traj_N`,U/F0 评测 `traj_W`,都取 IMU 位置。
   - 同时报告全部历元和"真值可信"子集。可信判据预注册为:同一时刻 9Ps PVT 满足 fix_type = 3、num_sv ≥ 12、h_acc ≤ 1.0 m。
2. **全局指标**(仅 GNSS 臂,次要):只做平移对齐、不做旋转对齐的 ATE,用来检验 ψ。同时记录 SE(3) 对齐求出的旋转角,用于判别真值坐标系(GCJ-02 与起点 ENU 两种说法都应接近零旋转,推断)。
3. **RPE**:50 m 与 100 m 段的平移误差。
4. **GNSS-denial01/02**:M3DGR 的 `ArUco_evaluate.py` 起终点误差,W 与 N 两套输出分别报告。
5. **一致性**:各类量测的平均 NIS/dof、接受率。
6. **鲁棒性**:发散次数,判据为 ATE > 5 m 或跟踪中断。
7. **实时性**(独占机器,实时 1× 回放):各模块耗时分位数。

### 统计

- 每条序列内,各臂相对 U(以及 T 相对 L-PVT、T 相对各消融臂)做 10 个种子的成对 Wilcoxon 符号秩检验,多重比较用 Holm 校正。
- "胜出"的预注册判定:成对差的中位数 < 0,且校正后 p < 0.05。
- 跨序列只报告胜出序列的计数,不把不同量级序列的 ATE 混合求均值。

### 报告

- 精确值只进表格(中位数与四分位距);正文只给胜率和计数,不背诵差值。
- FAST-LIVGO 表 II 的数值单独成表,标注"未审计、单次运行、未说明 LiDAR/配置/对齐方式"。
- §5.8 的预注册预测逐条对照实测结果给出判定。

---

## 19 已知局限

1. **地图本身不做全局修正。** W 地图保留 LIVO 漂移。全局一致的点云只能靠按扫描配准 `T(ξ(t))` 拼接,精度受 ξ 的全局精度限制(伪距级)。不引入后端是硬约束 3 的直接结果。
2. **单一配准假设。** 在旧区与新区之间反复穿梭时模型失配,只能靠锁死恢复兜底。
3. **LIO 协方差过度自信属上游问题。** 本设计不修改它,因此退化方向的接管可能不足。
4. **单频伪距的绝对精度只有米级。** 相关误差用 AR(1) 等效方差近似,没有逐星建模偏差状态。
5. **载波侧不估模糊度**,因此没有 RTK/PPP;M3DGR 本身也没有基准站。
6. **电离层模型。** BDS 与 Galileo 共用 GPS Klobuchar;TDCP 中电离层变化率的残差按白噪声预算处理,实际是系统性的,可能导致长时间积分偏差。
7. **忽略的小项。** 相位缠绕的差分部分、垂线偏差的空间变化、地球曲率与地球自转(上游机械编排本来就忽略)。
8. **时间偏移按常数处理。** 只用估计值加噪声膨胀,不在线估计;M3DGR 是软同步。
9. **杆臂固定。** 只以噪声形式计入;calibration.md 中天线条目的方向需要自检确认。真值天线(2HP)的杆臂未知,构成 ATE 的误差地板。
10. **TDCP 覆盖不完整。** 不重叠配对损失约 1/(m+1) 的覆盖时间。
11. **与上游非逐比特一致。** push-through 与对称化使 F0 不能与 U 逐比特一致,只能做统计等价检验。
12. **UBX 真机通路缺少实测。** 用户当前没有 GNSS 接收机。
13. **FAST-LIVGO 的对比不对等。** 其结果不可复现,只能引用;A-FA 仅复刻其更新方案,不含其 DTW 与双模切换。
