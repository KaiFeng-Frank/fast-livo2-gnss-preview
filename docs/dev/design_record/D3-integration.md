> Historical research record (through 2026-10-03). Paths are illustrative; archived runs are not bundled. For current status and executable entry points see [the repository README](../../../README.md). Current release docs/LICENSING.md supersedes historical license notes. This record does not authorize new experiments or assert current release validation.

# FAST-LIVO2-GNSS：在 ESIKF 内紧耦合原始 GNSS 的可交付设计（最小侵入 / 工程视角）

> 设计轮产物：只读上游代码与摸底资料，未改任何项目代码、未下载数据。
> 上游 = hku-mars/FAST-LIVO2 @0d2c034（ROS1）。下文「上游 file:line」均指该版本（`/workspace/fast_livo2_ref`）。
> 标「推断」者未经实测。本轮在 scratchpad 做了两个数值实验（非项目代码），作为两项关键决策的证据：
> `/tmp/claude-1000/-home-frank---/01015a13-6a43-4590-8171-9ed56812dd97/scratchpad/eigen_pad/pad.cpp`、`hook.cpp`。

---

## 0 结论先行

1. **核心 19 维状态与 LIO/VIO 更新代码一行不改。** GNSS 相关状态放在主状态之外的「增广块 e」：钟差/钟漂/系统间偏差、W→ENU 的偏航与平移、TDCP 随机克隆。
   - e 与主状态的互协方差 C（19×n）由三个只读钩子维护：
     - 传播钩子：累积 IMU 传播的 F_tot；
     - 核心更新条件化钩子：LIO/VIO 之后，用先验/后验 (P⁻, P⁺, δx) 做精确的条件高斯更新；
     - GNSS 联合更新：稠密、协方差形式的 IEKF。
   - 数值实验表明，这套钩子与「把 DIM_STATE 扩到 19+n 的联合滤波」在 1e-15 相对误差内一致。
   - 反过来，扩 DIM_STATE 后即使 GNSS 块完全解耦，19×19 主块在传播、信息形式求逆、(I−G)P 三种核心运算上都**不再与上游逐位一致**（实验见 §3.4）。
   - 所以增广块是唯一能同时满足「紧耦合」和「GNSS 关闭时逐字节等于上游」的形态。
2. **方法层：承认「地图当真值」，GNSS 信息主要由帧级状态吸收。**
   - 帧级状态 T_NW = (ψ, t_NW)，带「当前点不动」的随机游走，表示 LIVO 漂移。
   - 核心状态只在 LIO/VIO 退化方向、速度、偏置和钟差上分到可观份额。
   - GNSS 增益的主出口是全局 ENU 轨迹 T_NI = T_NW∘T_WI；局部地图保持自洽。
   - TDCP 用随机克隆（一致），不用固定锚。
3. **时间与调度。**
   - GNSS 历元在「周期末」处理：LIVO 模式在 VIO 之后，ONLY_LIO 模式在 LIO 之后。用自有 IMU 环形缓冲把 t_g 桥接到 t_k。
   - 时间映射：GPST → 估计器时间 = GPST − 闰秒 + 偏移 + 漂移。M3DGR 的偏移和漂移每序列离线标定，然后冻结。
4. **底座与 I/O。**
   - RTKLIB-EX v2.5.1（BSD-2）只取 8 个 .c 文件编成静态库，打 2 个小补丁：MAXPRNCMP=63；导出 prange/varerr/gettgd。
   - gnss_comm 消息用「线格式兼容的自写类型」读取：实时订阅与 rosbag 直读用同一类型、同一反序列化代码。
   - 不链接、不复制 gnss_comm 的任何代码或 .msg；另提供 RINEX 输入路径做交叉验证。
5. **确定性离线回放。**
   - 用 rosbag::View 按记录时间顺序直读，每条消息之后同步排空处理循环。与实时模式共用同一套回调和 `processOnce()`。
   - 构建 profile `eval`：关 OpenMP 并行，加 `-ffp-contract=off`；另做 3 处成员零初始化。
   - 门禁：两次回放逐位一致；GNSS 关闭时逐位等于参考提交 R；GNSS 开但不更新时核心也逐位等于 R。
6. **评测。**
   - 主体：确定性回放 × 起点扰动 10 种子，做成对集合。每臂每序列 10 次，完全可复现。
   - 辅助：上游基线与紧耦合全量两臂各做 5 次实时回放。
   - 主指标是 ENU 轨迹的 ATE。评测前先审计真值：坐标系（GCJ-02 还是 ENU）、时间基准、遮挡段、真值天线杆臂。

---

## 1 约定

### 1.1 帧与变换方向

| 记号 | 定义 | 依据 |
|---|---|---|
| {I} | IMU 体坐标 | 上游状态表示 T_WI |
| {L} | LiDAR | `extrinsic_R/T` = T_IL：p_I = extR·p_L + extT（LIVMapper.cpp:104-105,122-123,645） |
| {C} | 相机 | `Rcl/Pcl` = T_CL（vio.cpp:35-39） |
| {A} | GNSS 天线相位中心（一个点） | 杆臂 l = t_IA，在 I 系中给出 |
| {W} | FAST-LIVO2 世界系，即 IMU 初始化结束时的 IMU 系；不水平，yaw 任意 | IMU_Processing.cpp:144-145；gravity_align_en 默认 false（LIVMapper.cpp:77） |
| {N} | 本地 ENU，原点为 GNSS 初始化时选定的锚点 a（φ_a, λ_a, h_a）；x 东、y 北、z 天；选定后整次运行不变 | 本设计 |
| {E} | ECEF（WGS-84） | — |

**记号**
- x_A = R_AB·x_B + t_AB：T_AB 把 B 系坐标变到 A 系，t_AB 是 B 的原点在 A 中的坐标。
- 上游状态：R ≡ R_WI（rot_end），p ≡ t_WI（pos_end），v ≡ v^W（vel_end），g ≡ g^W（gravity，自由 R³）。
- 锚点：r_a^E 为锚点的 ECEF 坐标；R_EN 为 ENU→ECEF 的旋转，列为 e_E、e_N、e_U 在 ECEF 中的坐标。两者都是常量。

**帧状态**
- T_NW = (R_NW, t_NW)，其中 **R_NW = R_z(ψ)·R_lev(g)**。
- R_lev(g) 是把 W 系上方向 u = −g/‖g‖ 转到 e_z 的最小旋转，用当前重力状态计算。u 接近 −e_z 时退化为绕 x 轴转 π。
- ψ 是绕 N 系 z 轴的偏航。

**天线与全局输出**
- 天线位置：r^N_A = R_NW(p + R·l) + t_NW；r^E_A = r_a^E + R_EN·r^N_A。
- 天线速度：v^N_A = R_NW(v + R·[ω]×·l)，ω = ω̃ − b_g。
- 全局 IMU 位姿（新输出）：R_NI = R_NW·R，t_NI = R_NW·p + t_NW。

**扰动约定**
- 核心：R ← R·Exp(δθ)，右扰动（common_lib.h:170,184），其余加性；⊟ 为 δθ = Log(R_bᵀR_a)（:197-198）。与上游完全相同。
- 偏航：R_z(ψ+δψ) = Exp(e_z·δψ)·R_z(ψ)，左扰动，δψ 在 N 系；ψ 取值 (−π, π]。
- 其余增广量加性。

### 1.2 观测方程符号约定

**几何量**
- 视线：u^E = (r^E_s − r^E_A)/‖r^E_s − r^E_A‖，由接收机指向卫星；u^N = R_ENᵀ·u^E。
- 几何距离含 Sagnac 项，等同 RTKLIB `geodist`（rtkcmn.c:3612）：
  ρ = ‖r^E_s − r^E_A‖ + (ω_ie/c)(x_s·y_A − y_s·x_A)
  - r^E_s、v^E_s、δt_s、δṫ_s 由 RTKLIB `satposs` 在发射时刻算出，δt_s 含相对论项。

**伪距（m）**
- 模型：P − TGD_s = ρ + b + β_sys − c·δt_s + I_s + T_s + ε_P
  - b = c·δt_r，以 GPS 时为基准，单位 m；β_sys 为系统间偏差，β_GPS ≡ 0。
  - I_s：GPS Klobuchar 模型乘 (f_L1/f)²（pntpos.c:323-324 的做法）。
  - T_s：Saastamoinen 模型（`tropmodel`）。
- TGD 规则照搬 RTKLIB `prange` 单频分支（pntpos.c）：
  - GPS 用 TGD；
  - GAL 按 `getseleph` 选 BGD_E1E5b 或 BGD_E1E5a；
  - BDS B1I 用 TGD_B1I；
  - GLO 用 −dtaun·c/(γ−1)。

**多普勒（Hz）**
- 约定：**距离率 = −λD**，与 RTKLIB `resdop`（pntpos.c:587）、gnss_comm `dopp_res`、FAST-LIVGO 式 12 三处一致。
- 模型：−λ·D = ρ̇ + d − c·δṫ_s + ε_D，其中 d = c·δṫ_r（m/s）。
- 距离率含 Sagnac 变化率：
  ρ̇ = u^Eᵀ(v^E_s − v^E_A) + (ω_ie/c)(v_s,y·x_A + y_s·v_A,x − v_s,x·y_A − x_s·v_A,y)

**载波（周）**
- 模型：λΦ = ρ + b + β_sys − c·δt_s − I_s + T_s + λN + ε_Φ，与距离同号（u-blox cpMes 约定）。
- TDCP：λ(Φ_j − Φ_c) = ρ_j − ρ_c + (b_j − b_c) − c(δt_s,j − δt_s,c) + ε
  - 下标 c 为克隆历元。
  - 无周跳且 Δt ≤ 1 s 时，ΔI、ΔT、ΔN 视为 0，并入噪声。

所有残差一律 r = z − ĥ。

### 1.3 时间约定

- **GNSS 绝对时间**：统一用 GPST，内部表示为 (int64 秒, double 小数)，与 gtime_t 同构。这样可避免在 1.4e9 s 量级上用单个 double 造成约 0.2 µs 的截断。
- **估计器时间**：即上游传感器时间轴，IMU 时间戳为 header − imu_time_offset（LIVMapper.cpp:776）。映射公式：
  t_est = t_GPST − [time_base≠gpst]·leap(t_GPST) + offset + drift·(t_GPST − t_ref)
  - `gpst`：传感器已硬同步到 GPS 时，如 GREAT；
  - `utc`：传感器用 PPS+GPRMC 对到 UTC；
  - `host`：软件同步，如 M3DGR，offset/drift 离线标定（§5.2）。
- **接收机时标修正**：真实采样时刻 = 接收机时标 − b/c。u-blox 保证 |δt_r| < 1 ms，先做这步修正再映射。
- **闰秒**：取 RTKLIB 闰秒表（rtkcmn.c:171），最新一条为 2017-01-01，GPST−UTC = 18 s。2026 年仍为 18 s 是推断，单元测试里固化成断言。
- **其它时间系统**，一律经 RTKLIB 的 gpst2bdt/bdt2gpst/gst2time/gpst2utc 转换：
  - BDT = GPST − 14 s，周数差 1356；
  - GST 与 GPST 秒相同，周数差 1024；
  - GLONASST = UTC + 3 h。

---

## 2 方法层：GNSS 在「地图当真值」的滤波器里怎样一致地起作用

### 2.1 事实与量级（推断，待 D0 诊断实测）

**LIO 对位置的约束极强**
- 每点量测方差 R_i ≥ 0.001 m²（voxel_map.cpp:449），有效点 N_eff 为 300–3000。
- 按参数推算，LIO 后每轴 σ_p 约 1–4 mm（code-update §6 的推算）。
- 地图只以平面方差的形式进入 R，不在状态里。

**各类 GNSS 观测的量级对比**
- 伪距：每星 σ 为 1–5 m，≤30 颗星的信息量合计 ≤30 m⁻²。LIO 的量级是 1e5–1e6 m⁻²，所以单历元对 p_W 的修正只有新息的 1e-4 到 1e-6。
- TDCP：每星 σ 约 5–10 mm，约束的是 0.1 s 内的位移增量，信息量与 LIO 同量级。但它约束的是**增量**，不是 W 系的绝对位置。
- 多普勒：每星 σ 为 0.05–0.2 m/s，与 LIO 间接给出的核心速度精度同量级（推断）。

**结论（推断）**
- 在一致的滤波器里，GNSS 拉不动 W 系位姿。
- 能承载 GNSS 信息的只有四类：帧变换 T_NW、钟差、速度与偏置、LIO/VIO 的退化方向。

### 2.2 选择：帧级状态 + 「当前点不动」的随机游走

**建模思路**
- LIVO 的里程计漂移近似为位姿随机游走。地图 W 由正在漂移的里程计建成，所以「W 与真实 ENU 在当前位置处的关系」随时间漂移。这个关系由 T_NW 表示，并给它过程噪声。
- 偏航噪声应当让**之后的运动**转向，而不是让当前位置瞬间跳动。

**枢轴耦合的过程噪声**
- 设枢轴 q = R_NW·p（当前 IMU 相对 W 原点的向量，在 N 系表达）。约束：一个偏航增量 dψ 不改变当前点的全局位置，即 d(R_NW p + t_NW) = 0，由此 dt_NW = −[e_z]×q·dψ。
- 于是 [ψ; t_NW] 的离散过程噪声为
  Q_f(Δt) = J·diag(σ_ψ²Δt, σ_th²Δt, σ_th²Δt, σ_tv²Δt)·Jᵀ，J = [[1, 0₁ₓ₃], [−[e_z]×q, I₃]]
  - σ_th、σ_tv 分别是水平、垂直平移的随机游走强度。
  - q 取传播钩子时刻 t_k 的核心 p。

**被否的替代**
- 放大核心 P，让 GNSS 直接拉位姿：新旧地图错位；回访时 LIO 与 GNSS 互相拉扯；不一致。
- 固定锚 TDCP（把上一时刻后验当常数）：过度自信。
- 用 LS 一次解出 W→ENU 后固定（FAST-LIVGO 式 2-3）：无法吸收漂移，帧误差被冻住。

### 2.3 三类观测各自落在哪里（设计预期）

| 观测 | 主要进入 | 次要进入 |
|---|---|---|
| 伪距 | t_NW（绝对位置）、b、β | 核心退化方向 |
| TDCP（克隆） | T_NW 的增量（漂移率）、b 的差分 | 核心速度、退化方向 |
| 多普勒 | ψ（运动时）、d | 核心速度与加计偏置、经杆臂进入 b_g（弱） |
| 松耦合 PVT | t_NW、ψ（运动时） | 核心退化方向 |

后果：
- 非退化段，W 系轨迹和地图 ≈ 纯 LIVO；
- ENU 轨迹承载 GNSS 增益；
- 退化段（开阔草地、夜间视觉失效叠加 LiDAR 几何退化），核心会被 GNSS 实质性修正。

### 2.4 用本分析解释 FAST-LIVGO 表 II 的三个符号，并给出本系统的可证伪预测

**FAST-LIVGO 的机制**
- 固定锚 TDCP（式 11）把上一时刻后验当常数，忽略锚点与当前状态的互相关。
- 帧由 LS 一次解出后固定（式 2-3）。
- 合起来，每次更新都是对「上一后验 → 当前状态」位移的毫米级、过度自信约束，会压过 LIO 以地图为锚的估计，把**核心与地图**一起拉到 GNSS 位移轨迹上。

**三个符号的解释**

| 符号 | 现象 | 解释 |
|---|---|---|
| ① | 开阔天空 5 条序列 ATE 全部下降 | 载波好时，被拉到的轨迹接近真值。LIVO 漂移最大的 Varying-illu03（1.44 m）获益也最大 |
| ② | Dark01/02 高于纯 FAST-LIVO2 | 这两条序列 GNSS 受遮挡或多径（论文把它们放在 GNSS Denied 表头下；M3DGR 自己的分类是 Night，`m3dgr_README.md:349-350`）。过度自信的错误约束把核心拉偏 |
| ③ | 去掉剔除后 Dark 序列发散 | 未建模的外点以过度自信的权重注入核心，经地图插入固化，不可逆 |

**本系统的预注册预测**（评测时逐条判定）
- **P1**：非退化段，GNSS 各臂的 W 系轨迹与上游几乎一致。机制检验：诊断里核心所占的修正份额 < 1%。
- **P2**：ENU 轨迹是否优于上游 W 轨迹，取决于整条序列的 LIVO 漂移是否大于帧估计误差。
  - 长序列、漂移大的（Varying-illu03、Outdoor04）预期胜；
  - 上游 ATE ≤ 0.15 m 的短序列可能平局，甚至为负。
- **P3**：关闭门控的诊断运行中，核心与地图在 Dark01/02 上不发散，损伤被限制在 ENU 输出里。
- **P4**：固定锚 TDCP 诊断臂能在 W 系复现 FAST-LIVGO 的定性模式：开阔天空下降，退化时上升或不稳定。这是机制复现，不是主方法。
- **P5**：GNSS-denial01/02 上，若 GNSS 从未完成初始化，则回放轨迹与上游逐位一致；否则不出现显著劣化。

### 2.5 零成本诊断 D0：实现前后怎么用

**实现**
- 在参考提交 R 里就加上 `debug/cov_csv`：在四个时刻只读地写出 `_state.cov` 的对角线和位姿块特征值，另附 `voxelmap_manager->effct_feat_num_`。
  - prior：LIVMapper.cpp:256 之前，对应 :263 那行被注释掉的打印；
  - postLIO：:371 之后；
  - postVIO：:305 之后；
  - postGNSS：HEAD 才有。
- 不改变任何估计值。

**实现前**
- 在 M3DGR 的 3 条开阔序列上跑 R，统计 postLIO 的 σ_p 与 σ_θ 分布，以及位置 3×3 块最大特征值对应的方向。
- 预注册判据：

| 实测结果 | 解读 | 对设计的影响 |
|---|---|---|
| postLIO 最坏轴 σ_p 的中位数 ≤ 1 cm | 「地图当真值」成立 | 帧级设计必要。按 κ = σ_p²/(σ_p² + σ²/m) 预估：伪距 κ ≤ 1e-4，TDCP κ ≈ 0.1–0.5 |
| > 20% 的帧 σ_p ≥ 10 cm | 存在明显的 LIO 退化段 | 核心会分到可观 GNSS 信息，设计不变。对这些段单独报告 W 系改进 |

**实现后**
- `filter_cov.csv` 里 postVIO → postGNSS 两阶段迹的减少量，按「核心 / 帧 / 钟」拆分。
- `gnss_epochs.csv` 里的修正拆分：‖δp_core‖ 对比 ‖δt_NW‖ + ‖q‖·|δψ|。
- 按 effct_feat_num 分箱检验 P1：高特征数箱中，核心份额中位数应 < 1%。

---

## 3 状态、增广块与三个钩子

### 3.1 核心 19 维（上游，不改）

布局：[δθ 0-2 | δp 3-5 | δτ 6 | δv 7-9 | δbg 10-12 | δba 13-15 | δg 16-18]

- `DIM_STATE (19)`：common_lib.h:30
- `StatesGroup`：:126-223
- 传播：IMU_Processing.cpp:382-401
- LIO 更新：voxel_map.cpp:338-511
- VIO 更新：vio.cpp:784-802、:1520-1688

### 3.2 增广块 e 的槽位

| 槽位 | 符号 | 维数 | 单位 | 过程模型（Δt = t_k − t_{k-1}） | 初值与初协方差 | 存在条件 |
|---|---|---|---|---|---|---|
| clk_b | b | 1 | m | b += d·Δt + w | SPP；σ = max(σ_SPP, 3 m)·k_init | tight |
| clk_d | d | 1 | m/s | 见下，Q 与 b 联合 | estvel；σ = 0.2 m/s·k_init | tight |
| isb | β_R, β_E, β_C | 3 | m | 随机游走，σ_β = 1e-3 m/√s | SPP 各系统钟差 − b；未观测的系统 σ = 30 m | tight 且对应系统启用 |
| yaw | ψ | 1 | rad | 随机游走，按 §2.2 与 t_NW 枢轴耦合 | 速度对准（§6）；σ = 拟合残差·k_init | tight / loose |
| tNW | t_NW | 3 | m | 随机游走，水平 σ_th、垂直 σ_tv，加枢轴项 | SPP/PVT 均值；水平/垂直 σ 分开给 | tight / loose |
| clone | c_p（N 系）、c_b | 3+1 | m | 静止：Φ = I，Q = 0 | 由上一历元后验解析生成：Σ_cc = JΣJᵀ | TDCP 启用且上一历元载波有效 |
| td（可选） | t_d | 1 | s | 随机游走 | 0；σ = 5 ms | `time/estimate_online` |

**说明**
- 钟模型的 Q 按连续时间 PSD 离散（Groves 约定），**不照搬上游的 σ²·dt²**（IMU_Processing.cpp:395-399）：
  Q_bd = [[S_b·Δt + S_d·Δt³/3, S_d·Δt²/2], [S_d·Δt²/2, S_d·Δt]]
- 维数 n = 5 + 4 + 4 = 13，开启 t_d 时为 14。联合维数 32 或 33。
- 槽位用名字索引、可增删：克隆随历元替换；长期不可见的系统可以把 isb 边缘化。这正是固定尺寸的 DIM_STATE 做不到的。

### 3.3 联合协方差与三个钩子

**联合协方差**
Σ = [[P, C], [Cᵀ, Q_e]]
- P（19×19）就是 `_state.cov`，仍由上游维护；
- C（19×n）与 Q_e（n×n）归 GNSS 模块所有。

**H1 传播钩子**（`processImu`：Process2 与 gravityAlignment 之后，`state_propagat = _state` 之前，即 LIVMapper.cpp:254 与 :256 之间）
- 在 UndistortPcl 每个 IMU 步里累积 F_tot = F_K⋯F_1：F_x 在 IMU_Processing.cpp:382-392 构造，在 :401 使用，累积放在 :401 之后。
- 然后更新增广块：
  C ← F_tot·C·Φ_eᵀ，Q_e ← Φ_e·Q_e·Φ_eᵀ + Q_e(Δt)，e ← Φ_e·e
- 精确性：F 是块对角的，IMU 噪声与增广块噪声独立，所以这等价于对联合系统做传播。
- LIVO 的 VIO 步不做传播：v_imu 里只有 last_imu，循环 0 次（IMU_Processing.cpp:327）。此时 F_tot = I、Δt = 0，钩子为空操作。

**H2 核心更新条件化钩子**（handleLIO :371 之后；handleVIO :305 之后）
- 输入：先验 (x⁻, P⁻) = `state_propagat`，后验 (x⁺, P⁺) = `_state`。
  - LIO 的先验协方差就是 `state_propagat.cov`：`voxelmap_manager->state_` 在 :257 由 `_state` 赋值，StateEstimation 迭代中不改 cov（common_lib.h:182-192）。
  - VIO 的先验是 :256 的 `state_propagat`，即 LIO 后验。
- 计算：
  - M = (P⁻)⁻¹·C：先 Jacobi 缩放再用 LDLT 求解，只用于本模块，不写回核心；
  - δx = x⁺ ⊟ x⁻，用上游的 ⊟；
  - e ← e + Mᵀ·δx；
  - C ← P⁺·M；
  - Q_e ← Q_e − Cᵀ_旧·M + Mᵀ·P⁺·M。
- 依据：这两次更新的量测只依赖核心 x，所以 p(e | x) 不变，上述结果就是条件高斯的精确解。
- 与扩维联合 IEKF 逐项相同：当 P⁺ = (I−G)P⁻ 时，G_full = [[G, 0], [Cᵀ(P⁻)⁻¹G, 0]]，扩维滤波的 (I − G_full)Σ 与上式一致。
- 对 VIO 的上游怪癖也保持联合协方差半正定。怪癖是：vio.cpp:800 做 `cov −= G·cov` 时，G 可能来自已被回退的那一步（:1677-1681）。此时仍有
  Σ⁺ = [I; Mᵀ]·P⁺·[I, M] + blkdiag(0, Q_e − Cᵀ(P⁻)⁻¹C)
- 早退路径不调用钩子：LIO 无点（:343-347）、VIO 无点（:288-292）时状态不变。

**H3 GNSS 联合更新**（周期末，§4.9）
- 在 (19+n) 维上做协方差形式的 IEKF，写回核心 (x, P) 以及 e、C、Q_e。
- 不对任何矩阵求逆：S = HΣHᵀ + R 为 m×m，用 LDLT 分解。所以任何零方差的增广量（例如固定 ψ）都不会产生 NaN。
- 核心 LIO/VIO 的信息形式对全维 P 求逆（voxel_map.cpp:468；vio.cpp:1497,1661），只涉及 19 维，不受增广块影响。

### 3.4 数值验证（本轮实测，本机 Eigen 3.4.0 + GCC 13.3）

**实验 1：`pad.cpp`，扩维后是否仍逐位一致**
- 做法：把上游三种核心运算嵌到 24/28/34 维的块对角矩阵里，GNSS 块与核心零相关，模拟 GNSS 关闭；比较左上 19×19 块是否与 19 维结果逐位相等。三种运算是：FPFᵀ+Q、(HᵀR⁻¹H + P⁻¹)⁻¹（前 6 列技巧）、(I−G)P。
- 结果：
  - `-O3 -march=native`（上游 x86 编译选项，CMakeLists.txt:34）：3 个种子 × 3 个维数 × 3 种运算，**全部 DIFF**；
  - 通用 `-O3`：9 次里只有 1 个种子的 FPFᵀ 在 3 个维数上相同，其余全部 DIFF。
- 原因：19 维乘积 19+19+19 = 57 ≥ 20，已走 GEMM 路径（GeneralMatrixMatrix.h:440）。维数变了，分块和尾部处理也跟着变。
- 结论：扩 DIM_STATE 无法通过「GNSS 关闭时逐字节一致」门。Noetic 镜像（Eigen 3.3.7 / GCC 9.4）待复验，结论方向不变（推断）。

**实验 2：`hook.cpp`，钩子与联合滤波是否等价**
- 做法：取 19 维核心与 9 维增广块，先做 20 个 IMU 步的传播，再做一次类 LIO（6 列）和一次类 VIO（7 列）的仅核心更新，比较「核心 + 钩子」与「28 维联合信息形式滤波」。
- 结果（最大绝对差 / 量级）：

  | 步骤 | 协方差 | 状态增量 |
  |---|---|---|
  | 传播 | 2.5e-16 / 0.44 | — |
  | 类 LIO 更新 | 6.8e-15 / 0.42 | 4.5e-17 |
  | 类 VIO 更新 | 2.4e-15 / 0.36 | 3.0e-14 |

- 结论：钩子与联合滤波数学等价。

### 3.5 被否方案：扩 DIM_STATE 的完整改动位点（留作对照与退路）

**A. 改宏后自动变尺寸的位点**
- common_lib.h:30、:137、:167、:182、:194、:196、:222
- IMU_Processing.cpp:161、:301
- voxel_map.cpp:366、:462、:468-472、:489-490
- vio.h:122
- vio.cpp:1497、:1500-1501、:1661、:1665-1667

**B. 必须手改的位点**

| 位置 | 改动 | 不改的后果 |
|---|---|---|
| `StatesGroup` 构造（common_lib.h:128-140） | 新成员初值；新协方差块显式赋初值 | 继承 INIT_COV = 0.01，钟差 σ 只有 0.1 m，过度自信 |
| 拷贝构造与赋值（:142-152, :154-165） | 加新成员 | 新成员在这些地方被静默丢值：VIO 回退（vio.cpp:1401,1490,1510,1523,1650,1679）、`state_propagat`（LIVMapper.cpp:256）、`voxelmap_manager->state_`（:257,:371）、`latest_ekf_state`（:310,:379） |
| ⊞（:167-180, :182-192） | 加 `state_add.block<n,1>(19,0)` | 新状态不随更新变化 |
| ⊟（:194-206）、`resetpose`（:208-213）、成员（:215-222） | 同步加 | — |
| IMU_Processing.cpp:382-399 | 加 F(b,d) = Δt，新块 Q 按 PSD·Δt | — |
| IMU_Processing.cpp:185-196（Forward_without_imu） | 同上 | — |

**C. 只读前 6 块、不受影响的位点**：voxel_map.cpp:377-378、:552；LIVMapper.cpp:420-421

**D. 额外代价**
- 固定维数：星座集合与 TDCP 克隆只能按最大值预留。
- 任一新状态方差为零，LIO/VIO 的 P⁻¹ 就会给出 NaN。
- 每次 LIO 迭代要做两次 (19+n)³ 求逆。

**退路**：如果 §3.3 的钩子实现中出现无法定位的问题，可以退到本方案。但此时「零通道门」只能改成统计等价，需要用户同意。

---

## 4 观测模型与雅可比

### 4.1 IMU 桥接（t_g → t_k，t_g ≤ t_k）

**为什么不用 IMUpose 快照**
- IMUpose 只在 LIO 步生成，并在 IMU_Processing.cpp:538 被清空。
- 它的积分起点是 LIO 之前的先验，精度不如 LIO/VIO 后验。
- 迟到的历元可能跨多个周期，快照无法覆盖。

**做法**
- GNSS 模块自带 IMU 环形缓冲，保留 ≥2 s。数据从 `imu_cbk` 在样本被接受后（:808 之后）推入，与核心看到的时间戳完全相同。
- 加计按上游同样方式缩放：acc·G_m_s2/‖mean_acc‖（IMU_Processing.cpp:353），系数取 `p_imu->IMU_mean_acc_norm`，在 IMU 初始化完成后设置一次。
- 取周期末后验 (R, p, v, b_g, b_a, g)，用当前偏置对 [t_g, t_k] 的 IMU 做中值积分；t_g 处的 IMU 值线性插值。得到以 t_g 体系为参考、不含重力的预积分量 ΔR、Δv、Δp。

**桥接关系**，记 Δt = t_k − t_g，w := Δv·Δt − Δp：
- R_g = R·ΔRᵀ
- v_g = v − g·Δt − R_g·Δv
- p_g = p − v·Δt + ½g·Δt² + R_g·w
- x := p^W_A(t_g) = p_g + R_g·l
- y := v^W_A(t_g) = v_g + R_g·[ω_g]×·l，ω_g = ω̃(t_g) − b_g

**对核心误差的雅可比**（3×19，按上游列序）

| 列 | ∂x/∂· | ∂y/∂· |
|---|---|---|
| δθ（0-2） | −R_g·[w + l]×·ΔR | R_g([Δv]× − [[ω_g]×l]×)·ΔR |
| δp（3-5） | I | 0 |
| δτ（6） | 0 | 0 |
| δv（7-9） | −Δt·I | I |
| δb_g（10-12） | 桥接项用中心差分 | R_g·[l]× + 桥接项用中心差分 |
| δb_a（13-15） | 中心差分 | 中心差分 |
| δg（16-18） | ½Δt²·I，另加 §4.6 倾斜项 | −Δt·I，另加 §4.6 倾斜项 |

- 偏置列里的桥接项是 O(Δt) 小量。做法是对积分器做中心差分：6 个偏置分量，每个正负各积分一次，共 12 次，每次不到 30 个 IMU 样本，耗时约微秒级。这样避免手推预积分偏置雅可比出错。其余各列都是解析式，全部经数值差分验收（U7）。

**桥接噪声**
- 预积分噪声协方差 Σ_pre（9×9）由配置的 IMU 噪声密度给出，默认取 M3DGR calibration.md 的 Kalibr 值乘 10。
- 按各行对预积分量的敏感度 G 映射成 G·Σ_pre·Gᵀ，加到本历元的 R 上。这是 m×m 满阵，因为同一历元所有行共享同一段桥接误差。

### 4.2 帧与倾斜的公共导数

在 t_g 时刻：
- ∂r^N_A/∂δx_core = R_NW·X_c − 𝟙_g·T(x)
- ∂r^N_A/∂δψ = [e_z]×·R_NW·x
- ∂r^N_A/∂δt_NW = I
- ∂v^N_A/∂δx_core = R_NW·Y_c − 𝟙_g·T(y)
- ∂v^N_A/∂δψ = [e_z]×·R_NW·y

其中：
- X_c、Y_c 为 §4.1 表中的两列雅可比；
- 倾斜项 T(z) := [R_NW·z]×·R_z(ψ)·M_g，M_g = [e_z]×·R_lev·(I − ĝĝᵀ)/‖g‖。推导：R_lev(g+δg) ≈ Exp(M_g·δg)·R_lev(g)，最小旋转的 z 分量归 ψ 吸收；
- 𝟙_g 表示该项只加在 g 列（16-18），且仅当 `gravity_est_en` 为真（LIVMapper.cpp:85）。否则 g 在滤波中并非随机量（F(7,16) 不耦合，IMU_Processing.cpp:391），不应被 GNSS 更新。

### 4.3 伪距行

- 残差：r = (P − TGD) − [ρ(r^E_A) + b + β_sys − c·δt_s + I + T]
- 雅可比：
  - H = −u^Nᵀ·∂r^N_A/∂(core, ψ, t_NW)
  - H_b = +1
  - H_β,sys = +1（sys ≠ GPS）
  - H_d = −Δt，因为 b(t_g) = b_k − d·Δt
  - 可选时偏：H_td = −u^Nᵀ·v^N_A
- 视线 u 对位置的依赖、Sagnac 项对接收机位置的导数（约 6e-6）都忽略，与 RTKLIB `rescode` 的设计矩阵一致。
- Δt = 0 时退化为：H_θ = u^Nᵀ·R_NW·R·[l]×，H_p = −u^Nᵀ·R_NW。

### 4.4 多普勒行（−λD 约定）

- 残差：r = −λ·D − [ρ̇(v^E_A) + d − c·δṫ_s]
- 雅可比：
  - H = −u^Nᵀ·∂v^N_A/∂(core, ψ)
  - H_d = +1
- 杆臂项 R·[ω]×·l 必须保留：ω = 1 rad/s、l = 0.4 m 时就有 0.4 m/s。ω 取 t_g 处插值陀螺值减 b_g。
- 视线对位置的导数约为 |v_s − v_A|/ρ ≈ 2e-4 每米，忽略。

### 4.5 TDCP 行与随机克隆

**克隆**
- 每个载波有效的历元 j 提交后，生成克隆：
  - c_p = r^N_A(t_j) = R_NW·x + t_NW
  - c_b = b_k − d·Δt
- 雅可比 J_c 与伪距行的几何部分相同，外加 c_b 对 b 的系数 1、对 d 的系数 −Δt。
- 先删除旧克隆槽，行列直接删去即为边缘化；再按 Σ_cc = JΣJᵀ、Σ_zc = ΣJᵀ 追加新槽。
- 同时逐星保存：Φ_c、克隆历元的发射时刻、状态位（status bit1-3）、LLI。

**残差**
r = λ(Φ_j − Φ_c) − [ρ_j(r^E_A,j) − ρ_c(c_p) + (b(t_g) − c_b) − c(δt_s,j − δt_s,c)]

**雅可比**
- 当前历元部分同伪距行：H_b = +1，H_d = −Δt；H_β = 0，因为 ISB 在两历元间差分后为 0，其随机游走方差并入噪声。
- H_{c_p} = +u^N_cᵀ，其中 u_c 用克隆位置和克隆历元的卫星位置计算。
- H_{c_b} = −1。

**防星历切换伪跳**：克隆历元的卫星位置和钟差，用**当前历元所用的同一份星历**在克隆发射时刻重算，避免 IODE 切换带来的伪跳。

**有效性**，逐星同时满足：
- 两个历元的 cp 都有效（status bit1）、半周已定（bit2），且 bit3（半周已减）相同；
- 两个历元之间 LLI 均为 0；
- 多普勒辅助周跳检查：|λ(Φ_j − Φ_c) + λ·D̄·Δt_jc| < min(k_slip·σ, 0.4λ)，其中 D̄ = (D_j + D_c)/2；
- 有双频时再加几何无关组合检查；
- 跨度 Δt_jc ≤ `tdcp_max_span_s`（1 s）。

### 4.6 松耦合 PVT 行（松耦合对比臂）

- 数据：接收机 PVT（GnssPVTSolnMsg）的 lat/lon/altitude（WGS-84 椭球高）→ ECEF → N 系，得到 z_p。
  - 位置：h = R_NW·x + t_NW，H = ∂r^N_A/∂(·)；
  - 速度（可选，默认开）：z_v = (vel_e, vel_n, −vel_d)，h = R_NW·y。
- 噪声：R = diag(h_acc², h_acc², v_acc²)·k_pvt²，速度噪声 σ = vel_acc·k_pvt。
- 与伪距一样抽稀到 1 Hz。
- 不分配钟差槽位；帧模型、桥接、门控与紧耦合臂完全相同，所以这组比较只隔离「原始观测 vs 接收机解」这一个变量。

### 4.7 可选时偏状态 t_d

- 观测时刻为 t_g + t_d，雅可比取预测量对时间的导数。
- 伪距用 −u^Nᵀ·v^N_A；TDCP 只取当前项；多普勒忽略（加速度量级）。
- 默认关闭。主路径是离线标定并冻结（§5.2）。

### 4.8 噪声模型（初值；按 §17.3 的原理一次性定标后冻结，不做网格搜索）

**伪距**
σ² = s_sys²·{σ_P0²·(1 + 1/sin²el)·10^{max(0, CN0_ref − CN0)/10} + (k_std·psr_std)²}·k_corr² + σ_eph² + (0.5·I)² + (0.3/(sin el + 0.1))²
- 系统系数 s_sys 照 RTKLIB EFACT，BDS GEO 另 ×2；
- σ_eph² 由 `var_uraeph` 从 URA 索引算出；
- 伪距误差的时间相关（多径，数十秒量级）用两个手段处理：只在 GPST 整秒历元出伪距行（抽稀到 1 Hz），再把 σ 乘 k_corr = 3。10 Hz 白噪声假设会高估信息量约两个数量级（推断）。

**多普勒**
σ_D² = σ_D0²·(1 + 1/sin²el)/2 + (λ·dopp_std)²，全速率。

**TDCP**
σ² = 2·{σ_Φ0²·(1 + 1/sin²el)/2 + (λ·cp_std)²} + 桥接项。

psr_std、cp_std、dopp_std 为接收机自报值，来自 u-blox 的 prStdev、cpStdev、doStdev。

### 4.9 联合更新算法（H3）

1. **组装先验**：Σ = [[P, C], [Cᵀ, Q_e]]；保存 (x⁻, e⁻, Σ⁻) 的快照，供回滚用。
2. **迭代**，i = 0…max_iter−1（默认 2；初始化或重捕获后前 5 个历元用 5）：
   1. 在 x_i 处建行，得到 r_i、H_i，以及 m×m 的 R（对角项加桥接满阵）。
   2. 计算 S = H_i·Σ·H_iᵀ + R，做 LDLT。
   3. 计算 K = Σ·H_iᵀ·S⁻¹。
   4. 更新：x_{i+1} = x⁻ ⊞ K·[r_i + H_i·(x_i ⊟ x⁻)]。核心的 ⊞/⊟ 与上游同义；ψ 取模。
   5. 若 ‖δθ‖ < 1e-6 rad 且 ‖δp‖ < 1e-4 m，则收敛。
3. **门控**：只用 i=0 的 r_0 和 S_0（§8 L4）。伪距行在迭代中按 Huber 重加权（IRLS）。
4. **协方差**：Joseph 形式 Σ⁺ = (I−KH)·Σ·(I−KH)ᵀ + K·R·Kᵀ，然后对称化 Σ⁺ ← ½(Σ⁺ + Σ⁺ᵀ)。
5. **验后检查**（§8 L6），不通过则整体回滚到快照；回滚后的状态逐位不变。
6. **写回**：核心 R/p/τ/v/b_g/b_a/g 与 P 写入 `_state`；e、C、Q_e 写入增广块。

---

## 5 时间对齐与历元调度

### 5.1 映射
见 §1.3。M3DGR 使用 `time_base: host`。GnssMeasMsg 没有 header，其 bag 记录时间（主机收包时刻）**只用于诊断**，即输出延迟分布，不参与估计。

### 5.2 离线时偏标定工具 `tools/time_calib/gnss_time_calib.py`

每序列跑一次，结果冻结。输入是参考提交 R 的确定性回放，不含 GNSS，因此不存在循环依赖。

1. **LIVO 速度序列**：|v_W(t)|，取 `traj_W` 的差分或状态里的速度。
2. **GNSS 速度序列**：|v_GNSS(t_GPST)|，由 RTKLIB `pntpos` 中 estvel 的多普勒测速逐历元得到。
3. **粗搜索**：offset₀ = median(t_rec − (t_GPST − leap)) − 典型延迟。
4. **精搜索**：两序列重采样到 100 Hz，在 offset₀ ± 1 s 内做归一化互相关，峰值再做抛物线细化。
5. **漂移**：按 60 s 窗口滑动重复上一步，对「偏移–时间」做线性拟合，得到 offset 和 drift_ppm。
6. **质检**：峰值相关系数、各窗口偏移的离散度（> 5 ms 报警）、与记录时间延迟分布是否自洽。
7. **输出**：`time_calib/<seq>.yaml`。

**被否的替代：DTW**（FAST-LIVGO §III-C1、FAST-LIVO2-RTK 的做法）。DTW 是非参数弯折，可能过拟合；这里需要的是物理上的「偏移 + 漂移」两参数模型。

### 5.3 历元调度

**缓冲**
- 历元按 t_est 排序入队。实时模式由订阅回调入队，回放模式由派发器入队。

**处理规则**
- 每个周期末（时刻 t_k），把所有 t_est ≤ t_k 的历元按升序逐个处理，每个历元做一次联合更新。
- t_est > t_k 的历元留到下个周期。

**迟到历元**
- t_est < t_{k−1} 的历元照样处理，桥接 Δt = t_k − t_est。
- Δt 超过 `max_bridge_s`（0.3 s）则丢弃，并计数。
- 伪距抽稀：只有 |tow − round(tow)| < 5 ms 的历元带伪距行。

**为什么放在周期末，不放在 processImu 的 :252→:256**（code-timeline §7.2 的推荐点）
1. GNSS 在 LIO、VIO 之后，线性化点是最优的核心估计。
2. 光度 VIO 的收敛域小，回退判据 `error <= last_error`（vio.cpp:1648）是短视的。不让 GNSS 的外点或大新息改动 VIO 的先验。
3. LIVO 与 ONLY_LIO 共用同一个挂点。
4. 本周期地图插入时（:413-424）还不含本历元的 GNSS 修正，但下个周期传播后就含了。稳态下每历元修正量是毫米到厘米级，滞后一个周期（≤67 ms）可以接受。

F_tot 仍在 processImu 里采集（H1），只是更新放在周期末。

---

## 6 初始化

状态机：`DISABLED → WAIT_CORE → WAIT_NAV → COLLECT → ALIGN → TRACKING`，另有 `OUTAGE / DEGRADED / REINIT`，见 §9。

| 阶段 | 条件与动作 |
|---|---|
| WAIT_CORE | 满足 `!p_imu->imu_need_init && lidar_map_inited`，并已过 `warmup_s`（默认 2 s）。gravityAlignment 只在 IMU 初始化完成那一帧执行一次（LIVMapper.cpp:230-246，:254），先于本阶段结束；代码里断言增广块未激活时才允许对齐 |
| WAIT_NAV | 至少 5 颗星有有效星历。缺电离层参数时 I 取 0，σ_ion 取 5 m（同 RTKLIB `ERR_ION`），参数到达后自动切换 |
| COLLECT | 窗口 `window_s`（10 s，且 ≥20 个历元）。逐历元跑 RTKLIB `pntpos`（SPP + RAIM FDE + estvel），记录天线 ECEF 位置 r_j、ENU 速度 v_j、各系统钟差、SPP 协方差；同时用桥接取核心的 x_j = p^W_A(t_j)、y_j = v^W_A(t_j) |
| ALIGN（激励判据） | 水平速度 ≥ `min_speed_mps`（0.5）的历元不少于 5 s，且水平路径 ≥ `min_path_m`（5 m）。GVINS 用平均水平速度 0.3 m/s 作门槛（estimator.cpp:577）。静止时不对准，一直等待 |
| ALIGN（偏航） | 调平 ỹ_j = R_lev·y_j。2D Procrustes：ψ = atan2(Σ(ỹ_x·v_y − ỹ_y·v_x), Σ(ỹ_x·v_x + ỹ_y·v_y))；σ_ψ² ≈ σ_v²/Σ‖ỹ_h‖²，再乘 k_init²（3²） |
| ALIGN（锚点与平移） | 锚点取窗口内 SPP 的 Huber 均值，定下 N 系。t_NW = Huber 均值_j(r^N_SPP,j − R_NW·x_j)。σ 按 SPP 散布 / √N_eff，N_eff = window/τ_corr，τ_corr 取 10 s，所以 N_eff ≈ 1，结果是保守的 |
| ALIGN（钟） | b、β 取 SPP 各系统钟差；d 取 estvel 的钟漂 |
| ALIGN（激活） | 增广块 C 置 0：初值几乎完全来自 GNSS，与核心的相关可以忽略。随后前 10 个历元检查 NIS：若平均 NIS/dof > 3，把初协方差放大 10 倍重做 |
| 松耦合模式 | 用 PVT 的位置和速度替代 SPP 与 estvel，其余相同 |

消融开关（去伪距 / 去多普勒 / 去 TDCP）**只影响滤波更新**，初始化始终使用 SPP 与多普勒测速。否则帧无法建立。

---

## 7 更新调度（每周期时间线，LIVO 模式）

```
[sync→LIO]  processImu():
              t_prev = LidarMeasures.last_lio_update_time            (新增，:252 前)
              Process2 → UndistortPcl: 传播 x,P 到 t_k；F_tot 累积 (:401 后)
              gravityAlignment (仅一次)                                 (:254)
              H1: gnss.onPropagated(F_tot, t_prev, t_k, _state)       (新增，只读核心)
              D0: cov_csv(prior)                                       (新增，只读)
              state_propagat = _state; voxelmap_manager->state_ = _state (:256-257，不变)
            handleLIO(): StateEstimation (:370) → _state = ... (:371)
              H2: gnss.onCoreUpdated(state_propagat, _state, LIO)      (新增，只读核心)
              D0: cov_csv(postLIO)
              上游 evo 写出 (:384-405，不动) → publish → 地图插入 (:413-424)
[sync→VIO]  processImu(): 零传播；H1 为空操作；state_propagat = _state
            handleVIO(): processFrame (:305)
              H2: gnss.onCoreUpdated(state_propagat, _state, VIO) ; D0: cov_csv(postVIO)
            endOfCycle() (新增，stateEstimationAndMapping 的 switch 之后)：
              traj_W(t_k) 写出 → [GNSS] 取 t_est ≤ t_k 的历元，逐个：
                 预处理/掩码 → SPP-RAIM → 钟跳检查 → 桥接建行 → 门控 → 联合 IEKF
                 → 验后检查/回滚 → 写回 _state 与 e → 克隆刷新 → 诊断行
              imu_prop_enable 时刷新 latest_ekf_state → traj_ENU(t_k)、话题发布
              → state_trace(t_k)（确定性门用）
```

- **ONLY_LIO 模式**：endOfCycle 挂在 LIO 步之后。
- **ONLY_LO 模式**：没有 IMU，不支持 GNSS，启动时报错。
- **周期末判定**：`(slam_mode_==LIVO && lio_vio_flg==VIO) || (slam_mode_==ONLY_LIO && lio_vio_flg==LIO)`。

---

## 8 异常处理（分层）

| 层 | 检查 | 动作 |
|---|---|---|
| L0 数据健全 | 历元时间单调、去重；NaN/inf；伪距在 [1.9e7, 4.5e7] m；\|D\|·λ < 2 km/s；周翻转 | 丢弃该星或该历元，计数 |
| L1 信号掩码 | 仰角 ≥ 15°；C/N0 ≥ 30 dB-Hz；status 有效位；健康 svh=0；星历龄；连续跟踪 ≥5 历元（gnss_comm 没有 locktime，用计数代替）；排除列表；系统开关；GLONASS 伪距默认关闭 | 该星对应的行不参与 |
| L2 SPP-RAIM | RTKLIB `pntpos` 开启 RAIM FDE（posopt[4]），被剔的星不参与伪距行；SPP 与滤波预测水平差 > 30 m 时，本历元伪距不用，转 L3 | — |
| L3 钟跳 / 载波 | 伪距新息中位数 m：若 \|m − k·c·1 ms\| < 1 km（k ≠ 0），判为 ms 钟跳：b += k·c·1 ms，本历元跳过 TDCP 并刷新克隆，记事件。若 \|m\| > 1 km 且不是 ms 跳，从 SPP 重置钟（b、β，C 的钟槽清零）。半周与周跳检查见 §4.5 | — |
| L4 新息门控 | 逐行 χ²(1)，阈值 99.9%；迭代剔除最坏行，剔除总数 ≤ 50%；整历元 χ²(m)，阈值 99.9%。各观测类型分开统计 | 超限整历元拒绝 |
| L5 鲁棒 | 伪距行 Huber（k=2）IRLS | — |
| L6 验后 | 验后 NIS 劣于验前；或核心位置一步修正 > `core_step_max_m`（0.5 m），而验前核心 σ < 5 cm 且 SPP 不同意；或 Σ⁺ 非正定 | 整体回滚，状态逐位不变，计数 |
| L7 健康 | 连续 20 个历元整历元拒绝，且 SPP 健康、与滤波差 > 3σ | 进入 DEGRADED，再转 REINIT |

S = HΣHᵀ + R 已经包含了帧状态和核心的真实不确定度。所以 FAST-LIVGO 那种按 Hessian 最小特征值硬切两种模式（式 14）在这里不需要：退化时 S 自然变大，门自然变松。上游唯一的 6×6 Hessian 特征分解本来也被注释掉了（voxel_map.cpp:467）。

---

## 9 中断与恢复

| 事件 | 判据 | 处理 |
|---|---|---|
| OUTAGE | 连续 `outage_s`（5 s）没有被接受的行 | 增广块照常传播，随机游走方差自然增长；克隆超过 `tdcp_max_span_s` 即删除；钟按 b += d·Δt 继续外推 |
| 恢复 | 出现第一个被接受的历元 | 回到 TRACKING。前 5 个历元 max_iter=5，按 §4.9 做数值检查。不做人为限幅：大修正本身就是信息，而且主要落在帧状态 |
| REINIT | OUTAGE 超过 60 s 且（σ_t > 10 m 或 σ_ψ > 0.1 rad，线性化失效）；或 L7 触发 | 旧状态继续运行，后台按 §6 重新采集窗口。成功后替换 ψ、t_NW、b、d、β：值取新估计，Q_e 取初协方差，相关的 C 行清零。ENU 输出会出现一次跳变，写入事件日志 |
| 从未初始化 | 如 GNSS-denial 序列整段无 GNSS | 增广块从不激活，钩子全为空，核心逐位等于上游（回放模式） |

---

## 10 底座与 I/O

### 10.1 RTKLIB-EX vendoring

**来源与存放**
- rtklibexplorer/RTKLIB tag v2.5.1，版本宏见 rtklib.h:79-81：`VER_RTKLIB "EX"`、`PATCH_LEVEL "2.5.1"`。
- 放在 `third_party/rtklib/`，保留 `license.txt` 和 `readme.txt`。新增 `VENDORED.md`，记录上游 commit、各文件 sha256 和补丁清单。

**最小文件集**（候选，由 CI 的「链接闭包」作业最终确认）
- `rtkcmn.c ephemeris.c pntpos.c preceph.c sbas.c ionex.c geoid.c rinex.c`，另加 `rtklib_stubs.c`（链接需要时提供 showmsg/settspan/settime 空实现）。
- 不编：`rcv/*`、`rtkpos/ppp/lambda/stream*`、`lib/`。`lib/` 里的 IERS Fortran、SOFA、openblas 各有独立许可证。

**补丁**（BSD-2 允许修改，保留原声明；以 `patches/*.patch` 形式存放，CI 校验「vendored 树 = tag + 补丁」）
- P1：rtklib.h:211 的 `#define MAXPRNCMP 50` 外包一层 `#ifndef`，由 CMake 定义为 63。
  - 不改的话，`satno` 对 C51–C63 返回 0（rtkcmn.c:415），BDS-3 的 C56–C62 全部丢失，包括上海可见的 GEO C59–C62。GEO 轨道分支本身已支持 prn≥59（ephemeris.c:264-265）。
- P2：在 pntpos.c 末尾追加导出包装 `rtkx_prange()`、`rtkx_varerr()`、`rtkx_gettgd()`。上游这三个是 static（pntpos.c:51、:79、:110 附近）。`var_uraeph` 如果也是 static，同样处理。

**编译**
```cmake
add_library(livo_rtklib STATIC ${RTKLIB_SRCS})
set_target_properties(livo_rtklib PROPERTIES C_STANDARD 99 POSITION_INDEPENDENT_CODE ON)
target_compile_definitions(livo_rtklib PUBLIC ENAGLO ENAGAL ENACMP ENAQZS NFREQ=3 NEXOBS=3 MAXPRNCMP=63)
target_compile_options(livo_rtklib PRIVATE -O2 -fno-fast-math -ffp-contract=off -w)
target_link_libraries(livo_rtklib PUBLIC m pthread)
```
- 宏用 PUBLIC 导出，保证 adapter 看到的 obsd_t/nav_t 布局与库内一致。上游 CMake 用 `-DNFREQ=3 -DNEXOBS=3`（CMakeLists.txt:22），而 rtklib.h:158-165 的默认值是 NEXOBS=0，这两个宏有 #ifndef 保护。
- 不定义 TRACE。
- 顶层 `project(fast_livo C CXX)` 显式声明 C 语言。

**隔离**
- rtklib.h 定义了一批无前缀宏（PI、D2R、CLIGHT、MAXOBS…，rtklib.h:86-96、253）。只允许 `gnss/src/rtk_adapter.cpp` 这一个翻译单元包含它（pimpl）。rtklib.h 自带 `extern "C"`。
- livo_gnss 编成 SHARED 库，`-fvisibility=hidden`，并加 `-Wl,--exclude-libs,liblivo_rtklib.a`。这样 RTKLIB 的全局 C 符号（norm、dot、mat、trace…）不会外泄，也不会被其它共享库的同名调用插桩。

**全局可变状态**
- `eph_sel`（ephemeris.c:109）、闰秒表（rtkcmn.c:171）、`timeoffset_`。
- GNSS 处理只在主处理线程里串行调用；实时与回放两种模式都是如此。

**用到的函数**
- `satposs/satpos`（ephemeris.c:781/737）
- `geodist satazel ionmodel tropmodel ecef2pos pos2ecef xyz2enu`
- `gpst2time time2gpst gpst2utc utc2gpst gpst2bdt bdt2gpst time2bdt gst2time`
- `satno satsys sat2freq code2idx`
- `pntpos`（pntpos.c:646，含 estpos/raim_fde/estvel）
- `readrnxt`（RINEX 路径）

**GLONASS 频道**
- 加入 geph 时同步写 `nav.glo_fcn[prn-1] = frq + 8`，供 `sat2freq` 使用。
- 与消息 `freqs[]` 反推的 k = (f − 1602 MHz)/0.5625 MHz 交叉核对，不一致的星丢弃并记诊断。
- GLONASS 伪距默认关闭，因为低成本接收机的频间偏差（IFB）可达米级。多普勒和 TDCP 照用，λ_k 按频道计算。

**BDS**
- B1I 映射到 `CODE_L2I`（RINEX 3.02 的 C2I），这样 `prange` 会选 TGD_B1I。
- GEO 走 RTKLIB 的 GEO 分支；伪距 σ ×2，可配置。

### 10.2 读取 gnss_comm 消息（不链接 GPL-3.0）

**M3DGR 的 GNSS 话题**（依据 README:145-152；最终以数据审计 §16 I2 为准）

| 话题 | 类型 | 说明 |
|---|---|---|
| range_meas | GnssMeasMsg | 没有 header；时间只在逐星的 GnssTimeMsg(week, tow) 里，为 GPST |
| ephem | GnssEphemMsg | — |
| glo_ephem | GnssGloEphemMsg | — |
| iono_params | StampedFloat64Array | — |
| receiver_pvt | GnssPVTSolnMsg | — |
| time_pulse_info | GnssTimePulseInfoMsg | 没有 header，无法精确映射；只作诊断 |
| receiver_lla | 推断为 NavSatFix | — |

**主方案：线格式兼容的自写类型**
- 命名空间 `livo_gnss_ros::wire`，手写 `ros::serialization::Serializer`。
- ROS1 序列化规则：小端；定长字段按顺序排；变长数组先写 uint32 长度；嵌套消息内联。
- 类型标识：
  - DataType 字串写 `"gnss_comm/GnssMeasMsg"` 等。这是事实性标识。
  - MD5Sum 取 `"*"`：roscpp 发布端校验和 rosbag `MessageInstance::isType` 都接受通配。
- 真正的布局核对放在运行时：
  - 回放模式读 `rosbag::ConnectionInfo::msg_def`；实时模式读 `ros::MessageEvent` 连接头里的 `message_definition`。
  - 解析出扁平化的 (type, name) 序列，与编进二进制的期望表逐项比对。
  - 不一致即拒绝该话题，并报出版本差异。
- 同一类型既用于 `ros::Subscriber`（实时），也用于 `rosbag::MessageInstance::instantiate<T>()`（回放）。两种模式只有一份反序列化代码。

**字段语义映射**（gnss_comm → RTKLIB）

- **卫星号**：gnss_comm 编号 GPS 1–32、GLO 33–59、GAL 60–97、BDS 98–160（`sat_no`；各系统 MIN_PRN=1，MAX_PRN 分别为 32/27/38/63）→ (sys, prn) → `satno()`。GAL PRN 37–38 超出 RTKLIB 的 MAXPRNGAL=36，会被丢弃，并记入诊断。
- **GPS/GAL/BDS 星历**：
  - `A` 是半长轴（gnss_comm eph2pos 用 A³，gnss_utility.cpp:472），与 `eph_t.A` 同义。
  - `toe/toc/ttr` 为 GPST(week, tow)，转成 gtime_t。
  - `toes`：GPS 取 GPST 周内秒；GAL 取 GST 周内秒；BDS 取 time2bdt(gpst2bdt(toe))，因为 BDS 星历的 toe 在消息里已按 GPST 存储（gnss_comm eph2pos 用 toe−14 s 求 BDT，gnss_utility.cpp:496）。
  - `week` 按各系统口径换算。
  - `tgd0/tgd1` → `tgd[0]/tgd[1]`：GPS TGD；GAL BGD E5a/E1、E5b/E1；BDS TGD1/TGD2。与 rtklib.h eph_t 的注释逐项一致。
  - `ura`（m）查表得 URA 索引 `sva`；`health` → `svh`。
- **GLONASS 星历**：`freqo`→`frq`，`tau_n`→`taun`，`gamma`→`gamn`，`delta_tau_n`→`dtaun`，`toe`（GPST）→`geph.toe`。
- **观测**：
  - `freqs[i]` 决定频段：1575.42 MHz→L1/E1；1561.098→B1I；1602+0.5625k→G1；1227.60→L2；1207.14→E5b/B2I；1246+0.4375k→G2。由频段定 code 和 slot。
  - `psr`→P，`cp`→L，`dopp`→D，`CN0`→SNR，`LLI`，`psr_std`→Pstd，`cp_std`→Lstd。
  - `status` 各位：bit0 伪距有效，bit1 载波有效，bit2 半周已定，bit3 半周已减。
- **电离层**：`data` 的 8 个 Klobuchar 系数 → `nav.ion_gps`。

**许可证理由**（工程判断，不是法律意见；写入 `docs/LICENSING.md`）
1. 不链接 gnss_comm 库，不包含其生成的头文件，不复制其 .msg。
2. 自写类型依据的是线格式规格，即字段顺序与基本类型这类功能性接口事实。规格文档 `docs/wire_format_gnss_comm.md` 从数据集 bag 自带的 message_definition 导出后改写成表格。
3. 实时场景下，与 GPL-3.0 的 ublox_driver 只通过 ROS 话题（套接字）交换简单的测量记录。按 FSF FAQ，这属于分离程序之间的通信。
4. 读取数据集 bag（MIT 许可）属于数据处理。

**辅方案：RINEX 路径**
- 工具 `tools/rinex/bag2rinex.py` 基于 rosbags（Apache-2.0 库；工具本身 BSD-3），把 GNSS 话题导出为 RINEX 3.04 obs/nav。
- 配置 `gnss/source: rinex` 时，由 RTKLIB `readrnxt` 读入。
- 用途：
  - 与 RTKLIB rnx2rtkp/pntpos 逐位交叉验证（U6）；
  - 直接支持 GREAT、UrbanNav 这类以 RINEX 发布的数据集。

### 10.3 实时 / 确定性离线：同一处理函数

**公共部分**
- `LIVMapper::run()` 的循环体（:539-551）抽成 `bool processOnce()`：
  `if (!sync_packages(LidarMeasures)) return false; handleFirstFrame(); processImu(); stateEstimationAndMapping(); return true;`
- 实时的 `run()` 变成：`while(ros::ok()){ ros::spinOnce(); if(!processOnce()){ rate.sleep(); continue; } }`，与上游语义完全相同。

**离线驱动 `fastlivo_replay`**（`src/replay_main.cpp`）
- 由 roslaunch 以同名节点 `laserMapping` 启动。同名是因为 `vk::camera_loader::loadFromRosNs("laserMapping", …)` 在 LIVMapper.cpp:128 按这个命名空间读参数。
- 参数加载与实时模式用同一份 yaml。`replay/enable=true` 时不建订阅，发布者照常创建。
- 执行流程：
  1. 用 `rosbag::View(bag, TopicQuery(topics), start + offset, end)` 按 **bag 记录时间** 顺序迭代，这是 rosbag 的原生归并顺序，同时间戳按索引顺序。
  2. 每条消息按话题分派给**同一个回调**：`livox_pcl_cbk`、`standard_pcl_cbk`、`imu_cbk`、`img_cbk`、`img_compressed_cbk`、GNSS 各 handler。
  3. 每分派完一条，就 `while (mapper.processOnce()) {}`。
  4. 结束时调用 `finish()`：savePCD、flush、写 manifest。

**为什么选记录时间序**
- 它与 `rosbag play` 的发布顺序一致。
- 若改用 header 序，LiDAR 消息会排在它那一帧的 IMU 之前（扫描起始时刻 < 帧内 IMU 时刻），这是非因果的。

**确定性来源**（代码审计结论）
- 启动期丢弃逻辑取决于跨话题到达顺序：IMU 早于首帧 LiDAR 时被丢（:773），图像同理（:850）。回放里到达顺序由 bag 固定。
- 只有两处 OpenMP：
  - voxel_map.cpp:660 按下标写结果，本身是确定的；
  - vio.cpp:1554 的 `reduction(+:error, n_meas)` 是 float 归约，多线程时合并顺序不定，会改变 `error <= last_error` 的接受判定。
- 成员 `last_prop_end_time` 没有初始化（IMU_Processing.h:80），却在第一次 UndistortPcl 时被读取（:248）。
- 容器以 VOXEL_LOCATION 或 int 为键，没有指针键容器；没有随机数。
- `omp_get_wtime`、`ros::Time::now()` 只用于计时和输出。

**回放陷阱**：`publish_frame_world` 会清空 `pcl_w_wait_pub`（:1288-1289），而 `handleVIO` 以它是否为空作为门（:288）。所以回放里**禁止跳过任何 publish 调用**。没有订阅者时 roscpp 不会序列化，开销很小。

**压缩图像**：新增 `img_compressed_cbk`，用 `cv::imdecode(IMREAD_COLOR)` 解成 BGR，再调用从 `img_cbk` 抽出的 `pushImage()`（原 :860-876 那段）。两种模式都走这条路径，实时模式不再需要 republish 节点。上游 launch 用 republish（mapping_avia.launch），语义相同。

**扰动种子**：`replay/start_offset_s` 取 k×0.05 s，k=0…9。它改变首帧 LiDAR、IMU 初始化窗口和图像丢弃，用来可复现地模拟实时运行的抽签效应。

### 10.4 许可证布局

| 部分 | 许可证 | 说明 |
|---|---|---|
| 仓库整体 | GPL-2.0-only | 沿用上游 LICENSE |
| package.xml:15 | `BSD` → `GPL-2.0-only` | 另列 BSD-3-Clause、BSD-2-Clause |
| `gnss/`（ROS 无关核心，新写代码） | 建议 BSD-3-Clause | 可复用于 ROS2 移植和其它项目；与 GPL-2.0 兼容；需用户确认 |
| `third_party/rtklib` | BSD-2-Clause | — |
| `tools/` | BSD-3-Clause | — |
| `ros/` 胶水与上游改动 | GPL-2.0-only | — |

- 只发源码，不发布含 vikit（GPLv3，上游已有的形式冲突）的二进制或 docker 镜像。
- 用 `docs/LICENSING.md` 说明全部理由。

---

## 11 代码改动清单

### 11.1 上游文件逐处改动（HEAD 相对 0d2c034）

| 文件 | 上游位置 | 改动 | 所在提交 | 对数值的影响 |
|---|---|---|---|---|
| package.xml | :15 | license 改为 GPL-2.0-only，另列 BSD-3/BSD-2；加 depend：rosbag、diagnostic_msgs | 仓库初始化 | 无 |
| CMakeLists.txt | :2 | `project(fast_livo C CXX)` | 构建 | 无 |
|  | :34 后 | `option(FASTLIVO_STRICT_FP OFF)`，开启时给本包、livo_gnss、rtklib 加 `-ffp-contract=off` | 构建 | 仅 eval profile |
|  | :48-59 | `option(FASTLIVO_OMP ON)`；只有 ON 时才加 `-DMP_EN` | 构建 | 仅 eval profile |
|  | :79-94 | find_package 加 rosbag、diagnostic_msgs | 构建 | 无 |
|  | :120-142 | add_subdirectory(third_party/rtklib, gnss)；`add_library(livo_gnss_ros ros/src/*.cpp)`；`add_executable(fastlivo_replay src/replay_main.cpp)`；增加链接 | 构建 / I/O | 无 |
| include/IMU_Processing.h | :60 `first_lidar_time`、:80 `last_prop_end_time`、:81 `time_last_scan` | 类内初值 `= 0.0` | 回放（R） | 消除未定义行为；R 与 HEAD 相同 |
|  | 类末 | 新增 `MD(DIM_STATE,DIM_STATE) F_accum_ = Identity; bool F_accum_en_ = false;`，以及 `enableFAccum()`、`FAccum()`、`__attribute__((noinline)) accumulateF()` | 集成 | 无（只读 F_x） |
| src/IMU_Processing.cpp | Process2 入口（:546 后） | `if (F_accum_en_) F_accum_.setIdentity();` | 集成 | 无 |
|  | :401 后 | `if (F_accum_en_) accumulateF(F_x);` | 集成 | 无 |
| include/LIVMapper.h | 类 | 新增 `processOnce() finish() endOfCycle() isEndOfCycle() img_compressed_cbk() pushImage()`；成员 `std::unique_ptr<livo_gnss_ros::GnssRosBridge> gnss_`、输出写出器、`replay_mode_`、`cycle_idx_` | 回放 / 集成 | 无 |
| src/LIVMapper.cpp | readParameters :50-117 | 读取 §15 的新参数；未写的参数一律取上游行为 | 回放 / 集成 | 无 |
|  | initializeSubscribersAndPublishers :192-217 | replay 模式不建订阅；`common/img_compressed` 为真时订阅 CompressedImage；GNSS 订阅与发布（`/gnss/odom_enu`、`/gnss/fix`、TF enu→camera_init、`/gnss/status`） | 回放 / 集成 | 无 |
|  | processImu :248-265 | :252 前记 t_prev；:254 后调用 H1 与 D0(prior) | 回放 / 集成 | 只读 |
|  | stateEstimationAndMapping :267-279 | switch 之后 `if (isEndOfCycle()) endOfCycle();` | 回放 / 集成 | GNSS 关闭时只写文件 |
|  | handleVIO :305 后 | H2(VIO)、D0(postVIO) | 回放 / 集成 | 只读 |
|  | handleLIO :371 后 | H2(LIO)、D0(postLIO)；evo（:384-405）**不动** | 回放 / 集成 | 只读 |
|  | run :534-554 | 循环体抽成 processOnce() | 回放 | 等价 |
|  | imu_cbk :808 后 | `if (gnss_) gnss_->pushImu(timestamp, …)`，时间戳与核心一致 | 集成 | 无 |
|  | img_cbk :829-882 | :860-876 抽成 `pushImage(double, const cv::Mat&)`；新增 `img_compressed_cbk` | 回放 | 等价 |
|  | publish_odometry :1340、publish_path :1367、publish_mavros :1358 | `publish/use_sensor_time`（默认 false，即上游 `now()`）为真时改用 t_k | 回放 | 只影响输出 |
| src/replay_main.cpp | 新文件 | 离线确定性驱动 | 回放 | — |

**不改**：common_lib.h（含 `StatesGroup` 与 DEBUG_FILE_DIR）、voxel_map.*、vio.*、frame.*、visual_point.*、preprocess.*、feature.h、utils/*、src/main.cpp。

### 11.2 新增目录、类与接口

```
FAST-LIVO2-GNSS/            (catkin 包名仍为 fast_livo)
├── gnss/                   ROS 无关核心 livo_gnss（仅依赖 Eigen + livo_rtklib；CI 在无 ROS 环境单独编译）
│   ├── include/livo_gnss/
│   │   ├── types.hpp        GTime、SignalObs、SatObs、ObsEpoch、KeplerEph、GloEph、Klobuchar、ReceiverPvt、Geodetic
│   │   ├── core_state.hpp   CoreState（R,p,v,bg,ba,g,inv_expo,P19）、索引常量 kRot=0,kPos=3,kExpo=6,kVel=7,kBg=10,kBa=13,kGrav=16、boxplus/boxminus（与 StatesGroup 同义）
│   │   ├── config.hpp       Config 结构（§15 全部字段与默认值）
│   │   ├── gnss_fusion.hpp  门面类 GnssFusion
│   │   └── diagnostics.hpp  DiagnosticsSink 接口 + EpochReport/CycleReport 结构
│   ├── src/
│   │   ├── rtk_adapter.cpp  唯一包含 rtklib.h：NavStore(pimpl nav_t)、SatSolver(satposs/改正/仰角/λ)、Spp(pntpos 包装)、时间换算
│   │   ├── imu_bridge.cpp   IMU 环形缓冲、桥接积分、偏置列中心差分
│   │   ├── ext_block.cpp    ExtBlock：命名槽位、propagate(F_tot,Δt,q)、conditionOnCore(prior,post)、addSlot/removeSlot
│   │   ├── rows.cpp         PseudorangeRow/DopplerRow/TdcpRow/PvtRow/CloneJacobian、倾斜项 T(z)
│   │   ├── updater.cpp      JointUpdater：组装 Σ、IEKF、Joseph、门控/IRLS、回滚
│   │   ├── initializer.cpp  窗口采集、速度对准、锚点与平移 LS、钟初值
│   │   ├── health.cpp       状态机、钟跳、周跳、断星/重初始化
│   │   └── gnss_fusion.cpp
│   └── test/                gtest（U1–U13，§16）
├── ros/                    ROS1 胶水 livo_gnss_ros（GPL-2.0-only）
│   ├── include/livo_gnss_ros/{wire_msgs.h, wire_validate.h, gnss_ros_bridge.h, param_loader.h, bag_replayer.h, outputs.h}
│   └── src/{wire_validate.cpp, gnss_ros_bridge.cpp, param_loader.cpp, bag_replayer.cpp, outputs.cpp, bag_filter_main.cpp}
├── third_party/rtklib/     RTKLIB-EX v2.5.1 子集 + patches/ + CMakeLists.txt + VENDORED.md
├── src/replay_main.cpp
├── config/m3dgr/{avia_d435i.yaml, camera_d435i.yaml, gnss_m3dgr.yaml, time_calib/<seq>.yaml}
├── launch/{m3dgr_avia_rt.launch, m3dgr_avia_replay.launch}
├── tools/{data/, audit/, time_calib/, rinex/, gt/, eval/}   Python（rosbags、evo、numpy、scipy、pymap3d），BSD-3
├── docker/{Dockerfile, build.sh, run_replay.sh, run_rt.sh}
├── docs/{LICENSING.md, wire_format_gnss_comm.md, GNSS_DESIGN.md, EVAL_PROTOCOL.md}
└── test/gates/{gate_determinism.sh, gate_zero_channel.sh, gate_bypass.sh, gate_nodata.sh, expected/}
```

**门面接口**（`livo_gnss::GnssFusion`，全部同步调用、不含线程）

```cpp
explicit GnssFusion(const Config&);
void setImuAccScale(double s);                                  // G_m_s2 / IMU_mean_acc_norm
void pushImu(double t_est, const Eigen::Vector3d& acc_raw, const Eigen::Vector3d& gyr);
void pushEphemeris(const KeplerEph&);  void pushGloEphemeris(const GloEph&);
void pushIono(const Klobuchar&);       void pushObservation(ObsEpoch&&);  void pushPvt(const ReceiverPvt&);
void onPropagated(const Eigen::Matrix<double,19,19>& F_tot, double t_prev, double t_now, const CoreState& core_now);
void onCoreUpdated(const CoreState& prior, const CoreState& posterior, UpdateSource src);  // LIO / VIO
UpdateResult update(double t_now, CoreState& core, bool core_ready);  // 周期末；committed 时改写 core(x,P)
bool globalPose(const CoreState& core, Eigen::Matrix3d* R_NI, Eigen::Vector3d* t_NI) const;
bool anchor(Geodetic*) const;  Status status() const;  ExtSnapshot extSnapshot() const;
void setDiagnosticsSink(DiagnosticsSink*);
```

**胶水**（`livo_gnss_ros::GnssRosBridge`）
- 负责参数加载、订阅（实时）与回放 handler、wire → core 类型转换、StatesGroup ↔ CoreState 转换、输出文件与话题。
- 在胶水里写 `static_assert(DIM_STATE == 19)`，并对 StatesGroup 的 7 个成员逐一拷贝，拷贝函数单测覆盖。
- ROS2 移植只需重写这一层；核心不变。

### 11.3 提交序列与门禁

每个提交都要通过它之前所有提交的门禁。

1. **仓库初始化**：从 0d2c034 开分支；改许可证元数据；README 增加 GNSS 与许可证说明。
2. **构建**：CMake 选项（默认值等于上游）、docker、CI 骨架。门：默认 profile 构建产物的编译命令与上游一致，按 compile_commands 逐条比对。
3. **回放（打 tag `ref-R`）**：processOnce、replay_main、压缩图、traj_W、state_trace、cov_csv、use_sensor_time、3 处零初始化。门：G-确定性（R 两次回放逐位一致）；valgrind 审计（§12.4）。
4. **RTKLIB**：vendoring、补丁、链接闭包、U1–U6。
5. **GNSS 核心**：U7–U13 全绿；核心库在无 ROS 环境单独编译通过。
6. **I/O**：wire 类型与校验、审计工具、时偏标定工具、bag2rinex。门：I2 审计报告归档；U6（bag 路径 ≡ RINEX 路径）。
7. **集成**：F 累积、5 个挂点、周期末更新、输出与话题。门：G-零通道、G-旁路、G-无数据、G-确定性（GNSS 开）、I6 实时冒烟。
8. **松耦合**：PVT 行、初始化分支。门：同上。
9. **评测**：manifest、编排、统计、报告模板。门：§17.1 的前置审计全部完成后才允许产出评测数字（仪器自检先于判决）。

---

## 12 确定性回放与回归门

### 12.1 构建 profile

| profile | 选项 | 用途 |
|---|---|---|
| `eval` | `-DFASTLIVO_OMP=OFF -DFASTLIVO_STRICT_FP=ON`，Release，`-march=native` | 全部门禁与全部确定性评测 |
| `rt` | 上游默认（OMP ON、native、默认浮点收缩） | 实时回放评测 |

- `FASTLIVO_OMP=OFF` 去掉 `MP_EN`，vio.cpp:1554 的 float 归约改为按下标顺序串行执行。
- `-ffp-contract=off` 保证新增代码引起的内联或代码生成变化不会改变既有表达式的 FMA 收缩。Eigen 显式的 pmadd 内建不受影响，两边一致。
- 回放进程设 `OMP_NUM_THREADS=1`，固定 Eigen 内部的分块。
- vikit 在自己的 CMake 里硬编码 `-march=native`（vikit_common/CMakeLists.txt:10），且三个包的 -march 必须一致，否则 Eigen 对齐会让链接产物段错误（MID360 README 已踩过）。因此**不做跨机器可移植架构**：逐位结论只在同一台机器、同一个镜像内成立，跨机器只做统计比较。

### 12.2 可比对的确定性产物

- `state_trace.txt`：每次 processOnce 写一行，全部用 `%a`（hexfloat），内容为 cycle_idx、flag、t_k、R(9)、p、v、b_g、b_a、g、τ、diag(P)(19)，GNSS 开启时再加 e、diag(Q_e)。
- `traj_W.tum`、`traj_ENU.tum`（%.9f）。
- 上游 evo 文件 `Log/result/<seq>.txt`。
- `gnss_epochs.csv` 的确定性列（%.17g）。
- 计时信息只写 `timing.csv`，不参与比对。

### 12.3 门禁定义（全部在 eval profile、同一镜像、同一台机器上执行）

| 门 | 内容 | 判据 |
|---|---|---|
| G-确定性 | 同一 bag 回放两次 | 上述产物 sha256 相同；R 与 HEAD（GNSS 开）各做一次 |
| G-零通道 | HEAD（`gnss/enable:false`）对比 R | state_trace 与上游 evo 文件逐字节相同。另要求 `git diff 0d2c034..HEAD` 不触及估计器核心文件（common_lib.h、voxel_map.*、vio.*、frame.*、visual_point.*、preprocess.*）；IMU_Processing.* 的 diff 只含 §11.1 所列行 |
| G-旁路 | HEAD（`enable:true, update_enable:false`）对比 R | 核心 state_trace 逐位相同。证明数据摄入、SPP、诊断、钩子、F 累积（这时真的在执行）对核心零副作用 |
| G-无数据 | HEAD（GNSS 开）在去掉 GNSS 话题的 bag 上对比 R | 逐位相同 |
| R 与上游实时 | 不做逐位比较 | R 的集合分布与上游实时 10 次的分布做统计等价检查，作为健全性核查 |

**如果 G-零通道失败**，按以下顺序排查：
1. 逐个翻译单元比对 .o 的反汇编，定位代码生成差异；
2. 确认 `accumulateF` 为 noinline；
3. 检查是否漏了 `-ffp-contract=off`；
4. 用 valgrind 查未初始化读。

### 12.4 未初始化内存审计

- 在 eval 构建上，用 valgrind memcheck（`--track-origins=yes`）跑 60 s 回放片段。
- 估计器核心里报出的未初始化读，一律在 R 里用成员零初始化修复，并列入 `docs/GNSS_DESIGN.md` 的「确定性修复清单」。已知的是 D1–D3：IMU_Processing.h:60、:80、:81。
- 这些修复改变的只是上游未定义行为的一次具体取值。在新分配内存恰为 0 的常见情况下，行为与上游相同（推断）。

---

## 13 构建与部署（远端 Noetic docker）

### 13.1 Dockerfile 要点

以本机 `~/fastlivo2_mid360_ws/docker/Dockerfile` 为蓝本，去掉 realsense 和 Livox-SDK2。

```dockerfile
ARG BASE=dockerproxy.net/library/ros:noetic-ros-base-focal
FROM ${BASE}
ARG APT_MIRROR=mirrors.tuna.tsinghua.edu.cn
ARG GH_PROXY=https://gh-proxy.com/
# apt 源（Ubuntu + packages.ros.org）换镜像；安装：
#  build-essential cmake git libeigen3-dev libpcl-dev libopencv-dev libboost-all-dev libgoogle-glog-dev
#  ros-noetic-{cv-bridge,pcl-ros,pcl-conversions,tf,eigen-conversions,image-transport,
#              compressed-image-transport,rosbag,roslz4,visualization-msgs,diagnostic-msgs,cmake-modules}
#  python3-catkin-tools python3-pip valgrind
# pip（tuna 源）：rosbags evo numpy scipy pymap3d
# Sophus a621ff2（非模板版）：用 -std=gnu++98 编译，绕开 so2.cpp 里 `unit_complex_.real() = 1.;` 在 C++11 以上报错
RUN git clone ${GH_PROXY}https://github.com/strasdat/Sophus.git /opt/Sophus && \
    git -C /opt/Sophus checkout --detach a621ff2e56c56c839a6c40418d42c3c254424b5c && \
    cmake -S /opt/Sophus -B /opt/Sophus/build -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-std=gnu++98 && \
    cmake --build /opt/Sophus/build -j && cmake --build /opt/Sophus/build --target install && ldconfig
```

- FAST-LIVO2 本身用 C++17 编译，链接 libSophus.so（上游 CMakeLists.txt:102）；只有 Sophus 自己用 gnu++98 编。
- vikit：`xuankuzcr/rpg_vikit@6c886c8`，**不是** uzh-rpg 官方版。以子模块或锁定 commit 的方式放在 `ws/src/rpg_vikit`。
- Livox 消息：上游已自带 `include/livox_ros_driver/CustomMsg.h`（v1 生成头），编译时不需要驱动包。M3DGR 的 Livox 消息类型名和 md5 以审计 I2 为准。

### 13.2 目录布局（远端 /data 为 ext4；本机外置 SSD 是 exFAT，不能在上面编译）

```
/work/fl2gnss/
  ws/src/{FAST-LIVO2-GNSS, rpg_vikit}      只读挂载进容器
  build/{eval, rt}                          docker volume，每个 profile 一套
  data/m3dgr/{raw, filtered, manifest.json}
  runs/<arm>/<seq>/<seed>/                  每次运行一个目录
```

- 上游把 `ROOT_DIR` 在编译期烧成源码路径（CMakeLists.txt:40），`Log/*.txt` 写在它下面。并行运行时，每个容器把 `…/FAST-LIVO2-GNSS/Log` 绑定到本次运行自己的目录，这样零代码改动就实现了隔离。

### 13.3 数据预算

- 7 条 FAST-LIVGO 序列原始 59.1 GB，加 2 条 GNSS-denial 共 82.3 GB；远端 /data 只剩约 60 GB。
- 策略：逐条「下载 → 按官方 checksums.txt 校验 sha256 → 过滤 → 删原包」。
- 过滤工具 `ros/src/bag_filter_main.cpp`（rosbag C++，比 `rosbag filter` 的 Python 表达式快得多），只保留：
  `/livox/avia/lidar /livox/avia/imu /camera/color/image_raw/compressed /ublox_driver/*`
- 保存时间不变。估计过滤后约为原来的 37%（推断：Avia 点云约 4.6 MB/s，压缩 RGB 约 0.75 MB/s，其余大头是 MID-360、Insta360 X4、深度图），9 条合计约 30 GB。
- 过滤后 bag 的 sha256 写入 manifest。

### 13.4 时序约束

- 远端 livo_v06 训练和评分链预计 2026-09-26 13:00–16:30 结束。
- 按「任务跑着时禁任何并行重负载」的规则，docker 构建、回放和下载都排在它之后。
- 远端根分区只剩 8.1 GB，镜像约 3–4 GB（推断）。docker data-root 是否迁到 /data 需要用户决定（要 sudo 并重启 dockerd）。

---

## 14 输出

| 文件或话题 | 内容 | 时间戳 |
|---|---|---|
| `traj_W.tum` | 周期末（VIO 与 GNSS 之后）的 T_WI | t_k（`last_lio_update_time`，传感器时间），%.9f |
| `traj_ENU.tum` | T_NI = T_NW∘T_WI；GNSS 初始化之后才写 | 同上 |
| `traj_ENU_geo.csv` | IMU 的 WGS-84 纬度/经度/高程 | 同上 |
| `anchor.yaml` | 锚点大地坐标与 ECEF、R_EN | — |
| 上游 evo `Log/result/<seq>.txt` | LIO 后验（上游原样，零通道门用） | t_k |
| `gnss_epochs.csv` | 每历元一行，字段见下表 | — |
| `gnss_sats.csv`（可选） | 每历元每星：sat、el、az、CN0、各类型验前/验后残差、S_ii、权重、标志 | — |
| `filter_cov.csv`（D0） | t_k、stage（prior/postLIO/postVIO/postGNSS）、effct_feat_num、√diag(P)(19)、位置与姿态 3×3 块特征值、√diag(Q_e) | — |
| `events.log` | 初始化、重初始化、钟跳、断星/恢复、回滚、消息布局校验失败 | — |
| `state_trace.txt`、`timing.csv`、`run_manifest.json` | 见 §12.2；manifest 含 git describe/dirty、profile 与编译定义、Eigen/GCC 版本、镜像 digest、配置 sha256 与参数全量转储、bag sha256、种子、臂名、主机、起止墙钟时间 | — |
| 话题 | `/gnss/odom_enu`（frame "enu"）、`/gnss/fix`（NavSatFix）、TF `enu→camera_init`（即 T_NW）、`/gnss/status`（DiagnosticArray，1 Hz） | 传感器时间 t_k |

**`gnss_epochs.csv` 的字段**
- 时间：GPST week/tow、t_est、所在周期 t_k、桥接 Δt、到达延迟（仅诊断）；
- 状态机状态；
- 卫星数：总星数，以及伪距、多普勒、TDCP 各类型按系统分列的使用数；
- 剔除数：按掩码、RAIM、门控、鲁棒四类分列；
- 钟跳标志；
- SPP：是否成功、星数、PDOP、ENU 位置、与滤波预测之差；
- 新息：各类型 NIS 与自由度、新息 RMS；
- 协方差：核心 σ_p（3）与 σ_yaw 的验前/验后值；σ_ψ 与 σ_t（3）的验前/验后值；b、d 及其 σ；
- 修正拆分：‖δp_core‖、‖δt_NW‖、‖q‖·|δψ|；
- 迭代次数、是否提交或回滚。

---

## 15 配置（新增键；未写的键一律取上游行为；同一数据集只用一套参数）

```yaml
common:
  img_compressed: false          # M3DGR: true；img_topic: /camera/color/image_raw/compressed
replay:
  enable: false
  bag: ""
  start_offset_s: 0.0            # 扰动种子
  duration_s: -1.0
publish:
  use_sensor_time: false         # 评测配置设 true
output:
  dir: ""                        # 空 => ROOT_DIR/Log/out
  traj_w: false
  state_trace: false
debug:
  cov_csv: false                 # D0
gnss:
  enable: false                  # 总开关；false 时逐位等于 R
  update_enable: true            # false：只摄入/SPP/诊断（旁路门）
  mode: tight                    # tight | loose
  source: topics                 # topics | rinex
  rinex: {obs: "", nav: ""}
  topics:
    meas: /ublox_driver/range_meas
    ephem: /ublox_driver/ephem
    glo_ephem: /ublox_driver/glo_ephem
    iono: /ublox_driver/iono_params
    pvt: /ublox_driver/receiver_pvt
    time_pulse: /ublox_driver/time_pulse_info
  use: {psr: true, dopp: true, tdcp: true, pvt_vel: true}
  systems: {gps: true, glo: true, gal: true, bds: true}
  glo_psr: false
  bds_geo: {use: true, psr_sigma_scale: 2.0}
  exclude_sats: []
  lever_arm: {frame: lidar, xyz: [-0.358, 0.086, 0.429]}   # M3DGR calibration.md「GNSS antenna → avia」；
                                                           # 经 extrinsic_R/T 转到 IMU 系：l = extR·xyz + extT
  time:
    base: host                   # gpst | utc | host
    offset_s: 0.0
    drift_ppm: 0.0
    t_ref_gpst_s: 0.0
    calib_file: ""               # tools/time_calib 的输出；非空则覆盖上面三项
    max_bridge_s: 0.3
    estimate_online: false
  imu_bridge: {acc_noise: 0.21, gyr_noise: 0.032}          # 连续时间密度；取 M3DGR Kalibr 值 ×10
  mask:
    elev_deg: 15.0
    cn0_dbhz: 30.0
    min_track_epochs: 5
    max_eph_age_s: {gps: 7200, gal: 14400, bds: 21600, glo: 1800}
  noise:
    psr_sigma0_m: 1.0
    psr_cn0_ref_dbhz: 45.0
    psr_std_weight: 1.0
    psr_decimation_s: 1.0
    psr_corr_inflation: 3.0
    dopp_sigma0_mps: 0.1
    tdcp_sigma0_m: 0.01
    pvt_sigma_scale: 1.5
    pvt_decimation_s: 1.0
    scale: {psr: 1.0, dopp: 1.0, tdcp: 1.0}               # §17.3 的 NIS 定标因子
  process:
    clk_bias_psd: 0.1            # m^2/s
    clk_drift_psd: 0.1           # m^2/s^3
    isb_rw: 1.0e-3               # m/sqrt(s)
    yaw_rw: 1.0e-4               # rad/sqrt(s)
    trans_rw_h: 0.02             # m/sqrt(s)
    trans_rw_v: 0.03             # m/sqrt(s)
    td_rw: 1.0e-4                # s/sqrt(s)
  gate:
    chi2_row_p: 0.999
    chi2_epoch_p: 0.999
    max_reject_frac: 0.5
    huber_k: 2.0
    raim: true
    core_step_max_m: 0.5
    diag_disable_gating: false   # 仅诊断运行（P3）
  update: {max_iter: 2, max_iter_reacq: 5, tdcp_max_span_s: 1.0, slip_k: 4.0}
  tdcp_fixed_anchor_diag: false  # FAST-LIVGO 式固定锚 TDCP，仅诊断臂
  init:
    warmup_s: 2.0
    window_s: 10.0
    min_epochs: 20
    min_sats: 6
    min_speed_mps: 0.5
    min_path_m: 5.0
    cov_inflation: 3.0
    outage_s: 5.0
    reinit_after_s: 60.0
    reinit_sigma_t_m: 10.0
    reinit_sigma_psi_rad: 0.1
  output: {traj_enu: true, sats_csv: false}
```

**M3DGR 配置**
- LIVO 部分取 M3DGR 官方 `baseline_systems/Fast_LIVO2_M3DGR` 的 Avia 配置（取回时记录其 commit），所有臂共用。
- 相机内参取 calibration.md §2。
- Rcl/Pcl 取「avia → camera」。
- extrinsic_T = [0.04165, 0.02326, −0.0284]，与上游 avia.yaml:10 相同，也与 M3DGR「avia → avia_imu」一致。
- 由此杆臂 l = [−0.31635, 0.10926, 0.40060]（在 IMU 系）。这取决于对 calibration.md 方向约定的解读，由审计工具用转弯时的多普勒残差最小二乘做一次核对（§16 I2）。

---

## 16 测试

### 16.1 单元测试（gtest，ROS 无关，CI 无需大数据；小夹具放 `gnss/test/data`：60 s 的 M3DGR GNSS 消息，MIT，注明来源）

| 编号 | 内容 | 判据 |
|---|---|---|
| U1 时间 | GPST↔UTC（闰秒 18 s）、BDT（−14 s，周差 1356）、GST（周差 1024）、GLONASST（UTC+3 h）；周翻转；t_est 映射（offset/drift/base 三种）；接收机时标修正 | 与 RTKLIB 函数结果及已知历元逐位相等 |
| U2 线格式 | 各 wire 类型序列化往返；夹具原始字节解析出的字段值；message_definition 校验器接受原布局、拒绝改动过的布局 | 全部通过 |
| U3 星历适配 | A、toes（GPS/GAL/BDS）、week、tgd、URA 索引、svh、GLONASS frq 与 glo_fcn；与 bag2rinex→readrnx 的 eph_t 逐字段往返 | RINEX 精度内相等 |
| U4 卫星状态 | satposs 与 IGS 最终 SP3/CLK 对比（夹具日期；小文件，取回需用户同意） | GPS/GAL 轨道 3D < 3 m、钟差 < 5 ns；BDS MEO/IGSO < 5 m；BDS GEO < 10 m；GLO < 5 m；C59–C62 必须出现 |
| U5 改正与残差 | 在 RTKLIB pntpos 的解处，本模块的伪距预测与 `ssat.resp` 比较；多普勒在 estvel 解处的残差均值 | 伪距 \|Δ\| < 1e-6 m；多普勒均值 ≈ 0 |
| U6 路径等价 | 同一批历元分别走 bag 路径与 RINEX 路径，进 pntpos | 解逐位相同 |
| U7 雅可比 | 伪距、多普勒、TDCP、PVT、克隆生成、倾斜项，分别对 19 维核心（经 ⊞）与全部增广槽做中心差分；随机状态且 Δt ≠ 0 | 相对误差 < 1e-6 |
| U8 钩子 | 联合对拆分：传播、类 LIO、类 VIO，以及 P⁺ ≠ (I−G)P⁻ 的回退怪癖 | 相对误差 < 1e-12；Σ 半正定 |
| U9 增广块 | 枢轴噪声下当前点不动；钟模型 Q 与矩阵指数积分一致；增删槽位后剩余协方差逐位不变 | 全部通过 |
| U10 更新器 | 线性高斯情形等于批量最小二乘闭式解；Joseph 与标准形式一致；χ² 阈值；回滚后逐位不变 | 全部通过 |
| U11 初始化 | 合成轨迹加合成 GNSS（用真实星历），已知 ψ 与锚点可恢复；激励不足时保持等待 | 在容差内恢复 |
| U12 一致性蒙特卡洛 | 50 次：带已知漂移模型的仿真核心 + 伪距/多普勒/TDCP；注入断星与重捕获、ms 钟跳、周跳、NLOS（k 颗星 +20 m） | 帧状态 NEES 与各观测类型 NIS 落在 95% χ² 区间；NLOS 剔除率 ≥ 95% |
| U13 桥接 | 无噪声高频真轨迹 | Δt ≤ 0.1 s 时位置误差 < 0.1 mm；雅可比数值核验 |

### 16.2 集成测试（ROS1 docker，需要数据）

| 编号 | 内容 |
|---|---|
| I1 | 两个 profile 都能构建；核心库在无 ROS 环境单独构建通过 |
| I2 | 数据审计 `tools/audit/m3dgr_audit.py`，每个 bag 出报告，以下各项查完才进入下一步：话题、类型、md5、msg_def；频率与时间范围（header 对比记录时间）；按 (sys, freq, code) 的直方图；每历元星数；C/N0 分布；psr/cp/dopp 可用率；LLI 与 status 统计；各星星历时间线；iono 与 time_pulse 内容；GNSS 延迟分布；Livox 消息类型；GT 文件格式；杆臂的多普勒核对 |
| I3 | G-确定性 |
| I4 | G-零通道、G-旁路 |
| I5 | G-无数据 |
| I6 | 实时冒烟：roslaunch 加 `rosbag play` 1×，跑完不丢消息；GNSS 更新 p99 < 5 ms；输出齐全 |
| I7 | valgrind 审计（§12.4） |

---

## 17 评测协议

### 17.1 前置审计（全部完成并归档后才允许产出任何比较数字）

1. **数据完整性**：sha256 与官方 checksums.txt 对齐；过滤后的包写入 manifest。
2. **真值审计 `tools/gt/gt_audit.py`**
   - 坐标系：GT 若给经纬度，与 9Ps 接收机的 WGS-84 PVT 比较，GCJ-02 会在绝对坐标上偏出百米级；GT 若给局部 XYZ，看它与 PVT-ENU 做 SE(3)/Sim(3) 对齐后的残差结构，GCJ-02 在 1 km 尺度上会带来米级的平滑畸变（推断）。据此判定 GCJ-02 还是起点 ENU（issue #29 与 #38 说法矛盾），能纠偏则纠偏。
   - 时间基准：用速度互相关对齐 GT 与 PVT，确认 GT 时间戳的时基。
   - 可信掩码（预注册）：9Ps 自报 h_acc > 1 m、num_sv < 8，或 GT 速度出现 > 3σ 的尖峰的时段，标为「GT 可疑」。
   - 真值天线杆臂：2HP 天线到 IMU 的外参没有提供。
3. **时偏标定**：每条序列跑 §5.2，冻结。
4. **参数一次性定标**：只用调参集（§17.3），完成后冻结。

### 17.2 序列与对比臂

**序列**
- RTK 真值：Varying-illu03/04/05、Outdoor01/04、Dark01/02。
- ArUco 起终点真值：GNSS-denial01/02。

**传感器配置**：Avia + Avia IMU + D435i RGB 为主；MID-360 配置作为副线，需用户决定。

**对比臂**（全部用同一份 LIVO 参数）

| 臂 | 内容 |
|---|---|
| 上游基线 | 参考提交 R，即上游估计器加回放 |
| GNSS 关闭 | 必须与上游基线逐位相同（门禁），只报一次 |
| 松耦合 PVT | 接收机 PVT 当天线位置/速度量测，帧模型相同 |
| 紧耦合全量 | 伪距 + 多普勒 + TDCP |
| 去伪距 / 去多普勒 / 去 TDCP | 消融，只影响更新，不影响初始化 |
| 诊断臂（不参与主结论） | 固定锚 TDCP（检验 P4）；关闭门控（检验 P3） |

FAST-LIVGO 表 II 的数字只作**未审计引用**：单次运行；LiDAR/相机选型、对齐方式、统计口径都未说明；Dark01/02 的分类与数据集不符。

### 17.3 参数定标（原理驱动，不做网格扫描）

- **调参集**：Dynamic03、Occlusion03，约 7.2 GB，需用户同意下载。评测集不参与定标。
- **相对权重**：由接收机自报 std 加仰角与 C/N0 模型给出。
- **每类观测的绝对尺度因子**：取调参集上平均 NIS/dof，令其等于 1。
- **帧随机游走强度**：上游在调参集上对齐 GT 后，误差增量的方差随时间线性增长，按斜率估出 σ_th、σ_tv、σ_ψ。
- 定标完成后冻结，写进 `gnss_m3dgr.yaml`，并记录 sha256。

### 17.4 运行矩阵

**确定性集合（主体）**
- 每臂 × 每序列 × 10 个起点扰动种子（0…0.45 s），eval profile。每次运行完全可复现。
- 各臂之间按种子配对：同一种子下，GNSS 初始化之前的部分与上游基线逐位相同。
- 工作量估算：9 条序列共 5653 s；单线程约 0.8× 实时；8 臂 × 10 种子约 100 CPU·h，10 路并行约 10 h。
- 确定性回放不受资源争用影响，可以并行。

**实时（辅助）**
- 上游基线与紧耦合全量各 5 次 × 9 条序列，rt profile，交替执行。
- 期间不跑任何其它负载；阶段结束做争用筛查。约 16 h。

### 17.5 指标

| 编号 | 指标 |
|---|---|
| E1（主指标） | 全局 ATE。GNSS 各臂取 `traj_ENU`，上游基线取 `traj_W`。`evo_ape tum --align`（SE(3) Umeyama，只比位置；GT 姿态置单位阵）。在各臂共有的时间段上计算，因为 ENU 输出从 GNSS 初始化后才有，上游基线也截到同一时段 |
| E1'（副指标） | 在同一对齐基础上多估计一个 GT 天线杆臂（3 个参数），对所有运行按同一程序执行 |
| E2 | RPE，平移，路程段长 10 m 和 50 m |
| E3 | 所有臂的 `traj_W` ATE（检验 P1，看核心是否被改动） |
| E4 | GNSS-denial01/02 用官方 `ArUco_evaluate.py` 算端点误差，W 与 ENU 两种输出都算 |
| E5 | 一致性：各观测类型的 NIS 分布、剔除率、修正拆分（P1） |
| E6 | 耗时：GNSS 部分的 p50/p99，以及每周期总耗时 |

- E1、E2 同时报全段和「GT 可信掩码」两个版本。
- 对齐方式、时段截取、掩码都在评测前预注册。

### 17.6 统计与判定（预注册）

- **单序列**：10 个种子的成对差做 Wilcoxon 符号秩精确检验，同时统计胜负计数。
  - 判定「优于」：p < 0.05 且 ≥ 8/10 个种子胜。
  - 「劣于」对称判定；其余为平局。
- **跨序列**：报告优、平、劣的序列数。
- **表述**：报告正文只用胜率、计数和检验结论表述，精确值只进表格，正文不写差值数字。
- 实时 5 次的结果只用来检验确定性结论的方向能否迁移到实时。

---

## 18 已知局限

1. **单一帧变换无法修复被漂移扭曲的地图。** 回访旧区域时，LIO 贴回旧地图，而 T_NW 已随漂移游走，全局 ENU 输出在回环处会不自洽，差值等于期间被吸收的漂移。这是「不引入位姿图」这一约束的直接代价，FAST-LIVGO 同样存在。
2. **非退化段 W 系轨迹与地图基本等于纯 LIVO。** GNSS 的收益体现在 ENU 输出上，地图不会被「纠正」。这是一致性的代价，不是缺陷，但与「GNSS 消除地图漂移」的期待不同。
3. **伪距的时间相关用抽稀加放大因子近似处理，未建模一阶马尔可夫；多普勒与 TDCP 的误差相关性也未建模。** 两者同源于跟踪环，同时使用会有少量重复计权。
4. **M3DGR 是软件同步。** 时偏按「常数 + 线性漂移」处理；主机 NTP 在序列中途阶跃这类情况无法建模（可选的在线 t_d 只能部分跟踪）。
5. **上游缺陷保持原样，不在本分支修复，以免破坏零通道：**
   - `lidar_time_offset` 对 Livox 不生效（LIVMapper.cpp:744）；
   - `gravityAlignment` 不旋转协方差（:239-242）；
   - 偏置噪声硬编码为 1e-4（:153-154）；
   - 点协方差少乘 R_WI（voxel_map.cpp:387、:551-552）；
   - evo 写在 VIO 之前（:384-405）。
6. **9Ps 接收机特性未知**：单频还是双频、半周标志语义、ms 钟跳行为、是否输出 C59+，都要看数据审计。TDCP 的可用率可能低于预期。
7. **逐位确定性只在同一台机器、同一镜像内成立**（vikit 硬编码 `-march=native`）。
8. **GLONASS 伪距默认不用**（IFB）；BDS 也用 GPS Klobuchar 模型，未实现 BDS 自己的 Klobuchar 和 NeQuick。
9. **只处理单频主信号**；第二频点只用于周跳检查。不做 RTK/PPP，数据也没有基准站。
10. **许可证结论是工程判断，不是法律意见**；vikit 的 GPLv3 与上游 GPL-2.0-only 的冲突是上游本来就有的。

---

## 19 工作量与顺序（估计）

| 模块 | 代码量 | 工作量 |
|---|---|---|
| 仓库初始化、构建、docker | 约 0.3k 行 | 1.5 天 |
| 回放、确定性修复、四道门的脚本 | 约 0.6k 行 | 2 天 |
| RTKLIB vendoring、适配层、U1–U6 | 约 1.2k 行，另测试 0.8k | 3 天 |
| GNSS 核心（桥接、增广块、观测行、更新器、初始化、健康） | 约 3k 行，另测试 2k | 6–8 天 |
| 线格式 I/O、胶水、输出 | 约 1.2k 行 | 2.5 天 |
| 工具：审计、时偏、RINEX、GT、评测 | 约 1.5k 行 Python | 3 天 |
| 松耦合 | 约 0.3k 行 | 1 天 |

总计约 19–21 人日，不含数据下载和评测机时。
