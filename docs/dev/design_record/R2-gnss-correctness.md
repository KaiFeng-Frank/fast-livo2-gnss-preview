> Historical research record (through 2026-10-03). Paths are illustrative; archived runs are not bundled. For current status and executable entry points see [the repository README](../../../README.md). Current release docs/LICENSING.md supersedes historical license notes. This record does not authorize new experiments or assert current release validation.

## GNSS 解算正确性镜头:三设计对抗审查结论

**核对基线**:RTKLIB-EX main@06e8644(scratchpad `rtklibex/src`,与 v2.5.1 同线)、gnss_comm @a2035fe(`gnss_comm/`)、RTKLIB 2.4.3(`rtklib243/`)、FAST-LIVGO 全文、M3DGR README/calibration。三份设计在**符号约定**上全部正确且互相一致:距离率 = −λD(RTKLIB resdop pntpos.c:580-587、gnss_comm dopp_res gnss_spp.cpp:278、LIVGO 式12 三处同号)、Sagnac ω(x_s y_r − y_s x_r)/c(geodist rtkcmn.c:3612)及其速率项、载波 λΦ 与伪距同号、TDCP 中 −ΔI/+ΔT、卫星钟含相对论项(eph2pos ephemeris.c:287)、TGD/BGD 施加在观测端(prange pntpos.c:110-190)、Klobuchar×(f_L1/f)² 与方差四次方缩放(pntpos.c:322-324)、Saastamoinen+1/cos z(tropmodel rtkcmn.c:3791)、GPST/BDT −14 s、GST 周差 1024、闰秒 18 s、BDS GEO 分支(prn≤5||prn≥59,ephemeris.c:265)与 MAXPRNCMP 50→63 补丁——这些都无错。

**没有 fatal**;major 5 条,全部是「设计文本 vs. 被 vendored 的代码/数据的真实行为」之间的缺口,每条都有 file:line 依据:

| # | 设计 | 问题 | 后果 |
|---|---|---|---|
| 1 | D1、D3 | RTKLIB-EX 的 valsol χ² 失败被注释成不返回 0(pntpos.c:384),raim_fde 只在 estpos 失败时才调用(:689) | 两设计的 RAIM 层是空操作,D1 的「外部判决信号」实际不存在 |
| 2 | D2 | 发射时刻 t_tx = τ_rx − P/c 少减 δt_s(satposs 是 :804 再 :822 两步) | 逐星分米级、随 af0 慢变的系统偏差 |
| 3 | D1、D2 | BDS 的 eph_t.toes 必须是 BDT 周内秒(eph2pos :266/277 用 omge·toes);D1 写 toe_tow→toes,D2 未定义;gnss_comm 自己都对 BDS 重算(gnss_utility.cpp:499) | 若填成 GPST tow,BDS 卫星位置错约 27 km,静默 |
| 4 | D1 | 未说明如何回填 obsd_t.code;sat2freq==0 ⇒ rescode/resdop 整颗丢(:321/:565),prange BDS TGD 走错分支(:181) | SPP 静默无星或 B1I 零 TGD |
| 5 | D1 | GLONASS 伪距开启但只有单 ISB,IFB 未建模;GLO 星历映射漏 delta_tau_n→dtaun(prange G1 分支 :168-172) | 米级逐频道偏差进 t/ISB,门控反复 |

minor 10 条:pntpos() 不导出钟漂/钟协方差(D1、D3 初始化拿不到)、URA 字段语义与 Galileo SISA 表、psr_std/cp_std 单位、D1 把 1575.42 判为 BDS B1C、D1 钟跳层排在周跳层之后使「TDCP 免疫」在流程上不成立、D1 P3 按 IODE 回查对 BDS AODE 不唯一、D2/D1 对 varerr 的引用与默认 prcopt 不符、D2/D3 未提相位缠绕、D2 D4 阈值在长间隙误报、三家共用 GPS Klobuchar 的系统性余量。

**分设计评价**
- **D1(一致性优先)**:符号纪律与自检最强(§1 表、§12.6、U4/U5),但 I/O 语义层最薄:toes、code、dtaun、GLO 伪距四处漏洞集中在 §12.3;好在 U4 若覆盖 BDS 三类卫星能兜住前两条。RAIM 层需改写。
- **D2(鲁棒性优先)**:GNSS 内部裁判做法最对(自写 IRLS/w-test,不依赖被去牙的 raim_fde),GLO 伪距关闭、钟跳/半周处理明确;但 §3.3 发射时刻漏卫星钟差是其唯一的公式级错误,toes 未定义需补。
- **D3(集成优先)**:I/O 与时间系统语义最完整(toes 分系统、freqs→CODE_*、glo_fcn、GAL PRN 37–38、k·c·1 ms 钟跳、克隆用星历副本),但把 RAIM 寄托在 pntpos posopt[4] 上同样落空,d 取 estvel 钟漂拿不到。

**建议合并方式**:以 D3 §10.1–10.2 的底座/I/O 语义为骨架;GNSS 内部完好性层采用 D2 §5.3/§5.6.5 的自写 IRLS+w 检验(或补丁恢复 valsol 的 return 0 并逐位回归);符号约定表、Δclk 投影与入库自检采用 D1 §1/§4.5/§12.6;GLO 伪距默认关。所有设计都要把 U4 扩成「BDS MEO/IGSO/GEO + Galileo + GLONASS 各至少一颗,与同日 RINEX nav 逐位比对」并加一条 std 单位判定,否则上述 major 里有三条会在评测里静默存活。

[
 {
  "severity": "major",
  "designs": "D1/D3",
  "claim": "D1 §10 L1:「pntpos 的 χ² 失败且 n≥6 时执行 raim_fde(pntpos.c:472)」作为外部判决信号;D3 §8 L2:「RTKLIB pntpos 开启 RAIM FDE(posopt[4]),被剔的星不参与伪距行」。",
  "problem": "在 vendored 的 RTKLIB-EX(main@06e8644,scratchpad rtklibex/src)里 valsol 的 χ² 失败已被改成只写 msg 不返回 0:pntpos.c:384 `/* return 0; */ /* threshold too strict ... continue on */`;而 pntpos.c:689 `if (!stat&&n>=6&&opt->posopt[4]) raim_fde(...)` 只在 estpos 失败时进入。因此 FDE 只会在 GDOP>30(硬编码 MAX_GDOP)、星数不足或迭代发散时触发,χ² 永远不触发。对比原版 2.4.3(scratchpad rtklib243/pntpos.c valsol)仍 `return 0`。两个设计把这一层当成有效的 GNSS 内部一致性裁判,实际是空操作;D1 更把它列为「外部判决信号」的唯一来源。v2.5.1 tag 需复核,但该改动在 demo5 线已存在多年,方向不会变。",
  "fix": "三选一并写进 PATCHES.md:①补丁 P4 在 valsol 恢复 `return 0`(或加 prcopt 开关),与 pntpos() 做逐位回归;②不用 pntpos 的 FDE,自写 GNSS-only 加权 LS + Baarda w 检验/IRLS(D2 §5.6.5 的做法),1 颗故障星以上也能处理;③至少在设计文本中把 L1 层改写为「自算 χ²(用 rescode 残差 ssat.resp)+ 直接调 raim_fde」。任一方案都要在 T5/U5 中加一个「注入 +30 m 单星粗差,FDE 必须剔除」的用例。"
 },
 {
  "severity": "major",
  "designs": "D2",
  "claim": "§3.3:「卫星 satpos(t_tx) 广播星历(ephemeris.c:737),t_tx=τ_rx−P/c(P 无效用 ρ̂+b̂,同 satposs 思路)」。",
  "problem": "发射时刻少减了卫星钟差。RTKLIB satposs 是两步:ephemeris.c:804 `time=obs.time−pr/c`,再 :822 `time=time−dt`(dt 来自 ephclk,不含相对论);gnss_comm 同样 gnss_spp.cpp:76/84 `sv_tx=time_add(sv_tx,−svdt)`。广播 af0 量级 1e-4~1e-3 s,卫星速度 3.9 km/s ⇒ 沿轨位置差 0.4–4 m,投影到视线上为分米级、逐星不同、随 af0 慢变的系统偏差,不能被 b/ISB 吸收,直接进伪距和 TDCP(两端同偏差近似抵消,但 Doppler 中卫星位置用于视线方向影响小)。D2 自写量测模型、只把 pntpos 当参照行,这一步不会被 T5 抓住。",
  "fix": "t_tx = t_rx − P/c − δt_s,其中 δt_s 先用 ephclk(或 eph2clk/geph2clk)算一次;或直接调 satposs(obs 数组)拿 rs/dts。P 无效(仅载波历元)时用 ρ̂ + b̂ − c·δt_s 代替 P。在 T4/T5 加「本模块 t_tx 与 satposs 内部 time[i] 差 < 1e-9 s」的断言。"
 },
 {
  "severity": "major",
  "designs": "D1/D2",
  "claim": "D1 §12.3:「toe_tow→toes」;D2 §11.2:「toe/toc/ttr 为 GPST」但未定义 eph_t.toes 取什么。",
  "problem": "RTKLIB eph2pos 的升交点经度用 `omge*eph->toes`(ephemeris.c:266 GEO 分支、:277 一般分支),toes 必须是该系统自己的周内秒:GPS/QZS 为 GPST tow,Galileo 为 GST tow(与 GPST 秒相同),BDS 必须是 BDT tow。若 BDS 填成 GPST tow(差 14 s),Ω 偏 ω_e·14 s = 1.02e-3 rad,MEO/IGSO 位置错约 27 km、GEO 同错,整个 BDS(中国境内主系统)全部报废且静默。gnss_comm 自己都不信任 toe_tow 字段:gnss_utility.cpp:499/595 对 BDS 重算 `time2bdt(time_add(toe,−14))`,说明 ublox_driver 写入的 toe_tow 语义不可靠。D1 显式写错;D2 未定义等于留给实现者猜。",
  "fix": "按 D3 §10.2 的做法:GPS/QZS toes=time2gpst(toe);GAL toes=time2gpst(toe)(或 time2gst,数值相同);BDS toes=time2bdt(gpst2bdt(toe));GLONASS 不用 toes。单元测试(D1 U4 / D2 T4)必须显式包含 BDS MEO、IGSO、GEO 三类卫星,并与同日 RINEX nav 经 readrnx→eph2pos 逐位比对。"
 },
 {
  "severity": "major",
  "designs": "D1",
  "claim": "§12.3:「信号分类:不依赖 code 的枚举……改为按 freqs(Hz)分类」,未说明如何填 RTKLIB obsd_t.code。",
  "problem": "RTKLIB 的多处逻辑以 obs->code[0] 为键:rescode pntpos.c:321 `sat2freq(sat,obs.code[0],nav)==0 ⇒ continue`(整颗星丢弃,电离层缩放也失效);prange pntpos.c:178-181 按 code 选 BDS TGD 分支,code 不是 CODE_L2I 时落到 `tgd[2]+tgd[4]`(gnss_comm 无此字段 ⇒ 0 修正,B1I TGD 约 1–4 m);resdop pntpos.c:565 同样 freq==0 则跳过。D1 若只做频段分类而不回填 CODE_*,SPP 会静默退化到无星;其 U5(与 pntpos 逐位一致)用同一份 obsd 不会发现。D2 §11.2、D3 §10.2 都显式写了 freqs→CODE_* 映射。",
  "fix": "显式表:GPS/QZS 1575.42→CODE_L1C、1227.60→L2X;GLO 1602+0.5625k→L1C、1246+0.4375k→L2C(并写 nav.glo_fcn[prn-1]=k+8);GAL 1575.42→L1X、1207.14→L7X;BDS 1561.098→L2I、1207.14→L7I;其他组合拒收。把「每颗星 sat2freq(sat,code,nav) 与消息 freqs 一致」作为入库自检 1 的硬判据。"
 },
 {
  "severity": "major",
  "designs": "D1",
  "claim": "§2.3/§4.6/§15:GLONASS 伪距参与更新,只用一个 ISB 槽位 + EFACT 1.5;§12.3 GLO 星历映射列出 freqo/tau_n/gamma 与位置速度加速度,未列 delta_tau_n。",
  "problem": "u-blox 单机上 GLONASS 码观测存在随频道号 k 线性变化的频间偏差(IFB),量级可达米级,单一 ISB 只能吸收其均值,残余按 k 分布、在 χ²_1 门(10.83)附近反复进出,直接偏置 t 与 ISB;EFACT 只放大方差不去偏。另外 prange 的 G1 单频分支 pntpos.c:168-172 用 `b1=−dtaun·c`、`P1−b1/(γ−1)` 做 GLONASS 群延迟,若 dtaun 未映射则该项恒 0(典型 dtaun≈1e-9 s ⇒ 约 0.4–0.5 m 逐星偏差)。D2 §2.2 与 D3 §10.1 都默认关闭 GLO 伪距。",
  "fix": "默认 glo 伪距关闭(多普勒/TDCP 照用,λ 按频道);若确需开启,增加 1 维「IFB 斜率×k」状态(RTKLIB RTK 的 glomodear 线性模型)并把 delta_tau_n→dtaun 补进映射表;T−pr 之外加一个 T+glo_pr 探针臂验证。"
 },
 {
  "severity": "minor",
  "designs": "D1/D3",
  "claim": "D1 §8 阶段 3:「b、ι 取自 SPP 解的联合协方差块;d 取自 k_a 历元的多普勒 LS」;D3 §6 ALIGN:「d 取 estvel 的钟漂」。",
  "problem": "pntpos() 不导出这些量:estvel 只回写速度 `matcpy(sol->rr+3,x,3,1)`(pntpos.c:621),钟漂 x[3] 与 Q 丢弃;estpos 只存位置块 sol->qr[0..5](pntpos.c:449-453),钟差/ISB 的方差与互协方差不输出。D1 的「一致增广」若拿不到钟块协方差,只能拍一个先验,初始化一致性主张打折;D3 的 d 初值无处取。",
  "fix": "补丁 P2 扩展为导出 estpos/estvel 的完整 x 与 Q(BSD-2 允许),或初始化阶段自写多普勒/伪距 LS(D1 阶段 2 已有 GN-LS,可顺带输出 d 与钟协方差)。"
 },
 {
  "severity": "minor",
  "designs": "全部",
  "claim": "D3 §10.2:「ura(m)查表得 URA 索引 sva」;D1 §12.3:「若为整数且 ≤15 视为索引,否则按米转成索引」;D2 §5.2 掩码「URA≤10 m」。",
  "problem": "gnss_comm Ephem.ura 是 double「satellite signal accuracy」(gnss_constant.hpp:309),gnss_spp.cpp:218/220 用 `ura−1`/`ura−2` 当米用,大概率是米;但 RTKLIB var_uraeph(ephemeris.c:114-131)对 GPS/BDS/QZS 用 URA 索引表 {2.4,3.4,4.85,…},对 Galileo 用 SISA 索引(0–125 分段,>125 ⇒ 500 m 并触发 satexclude 的 MAX_VAR_EPH=300²)。D3 若用 GPS 表反查 Galileo 3.12 m ⇒ 索引 1 ⇒ var_uraeph(GAL,1)=0.01 m,Galileo 伪距被高估精度约 300 倍;D1 的整数启发式对 Galileo SISA 索引(可达 255)失效;D2 的「≤10 m」若字段实为索引会剔光 Galileo。",
  "fix": "先用一条 bag 的 ephem 与同日 RINEX nav 的 SV accuracy 字段对拍确定语义;GPS/BDS/QZS 用 ura_value 表反查最近索引,Galileo 用 SISA 表反查(0–0.49 m 步 0.01,0.5–0.98 步 0.02,1–1.96 步 0.04,2–6 步 0.16);或绕开 var_uraeph,直接以米为单位自算 σ_eph²。"
 },
 {
  "severity": "minor",
  "designs": "全部",
  "claim": "psr_std/cp_std/dopp_std 直接按 gnss_comm 注释的单位(m/cycle/Hz)使用:D1 掩码 max_cp_std_cyc 0.1;D3 TDCP 噪声 (λ·cp_std)²;D2 receiver_std_floor。",
  "problem": "u-blox RAWX 原始字段是 prStdev=0.01·2ⁿ m、cpStdev=0.004·n m、doStdev=0.002·2ⁿ Hz,n≤15(RTKLIB ublox.c:454-455,:537-538 直接按米存 Lstd,与 rtklib.h:598 注释的 cycles 自相矛盾)。ublox_driver 写入 gnss_comm 时是否把 0.004·n 除以 λ 换成周未知;若未换,λ·cp_std 会把 TDCP 噪声低估约 5 倍(0.19×0.06 m 对 0.06 m),D1 的 0.1 周阈值则相当于 0.019 m,把 F9P 正常的 n≥5 全部剔除(RTKLIB-EX 认为 F9P n≤14 有效,ublox.c:116)。三个设计的入库自检都没有单位判定。",
  "fix": "入库自检加一条:cp_std 取值集合应落在 {0.004·n/λ} 或 {0.004·n} 两组离散格点之一,据此判定单位;dopp_std 同理 {0.002·2ⁿ}。阈值改按米给出并按 λ 换算。"
 },
 {
  "severity": "minor",
  "designs": "D1",
  "claim": "§12.3:「1575.42 MHz:GPS L1C、Galileo E1、BDS B1C」。",
  "problem": "把 BDS 1575.42 MHz 判为 B1C 后,RTKLIB prange 走 pntpos.c:181 `tgd[2]+tgd[4]`(TGD_B1Cp+ISC_B1Cd),gnss_comm 只有 tgd0/tgd1 ⇒ 零 TGD;且 B1I 的 D1/D2 星历钟基准与 B-CNAV1 不同,直接混用有米级偏差。9Ps(F9P 内核)只跟踪 B1I/B2I,现实数据不会触发,但设计把它写成合法分支是静默陷阱。",
  "fix": "BDS 只接受 1561.098(B1I→CODE_L2I)与 1207.14(B2I→CODE_L7I);1575.42 的 BDS 信号按「未知组合拒收并计数」处理(D2 §11.2 做法),待有 B-CNAV1 星历支持再开。"
 },
 {
  "severity": "minor",
  "designs": "D1",
  "claim": "§10 L5:「针对 u-blox 1 ms 钟跳;TDCP 已天然免疫」,而 L3 周跳层排在 L5 之前并含「多普勒积分相位检验 |ΔΦ_meas−(−∫D dt)|>max(0.5 周,4σ)」。",
  "problem": "u-blox 钟跳时 rcvTow、prMes 与 cpMes 同时跳:Φ 跳 f·1 ms≈1.58e6 周,λΔΦ 与伪距一样跳 299.79 km。Householder 投影确实能吃掉这个公共项,但 L3 先于 L5 执行,积分相位检验会把当历元全部卫星判为周跳、断链、停放克隆,「免疫」在流程上不成立;D2 §5.5 直接断链(保守但明确),D3 §8 L3 用 |m−k·c·1ms|<1 km 显式判 k 并跳过 TDCP。",
  "fix": "L5 提前到 L3 之前:用伪距新息中位数判 k(D3 判据),同时对 λΔΦ 扣除 k·c·1 ms 后再做 L3;这样 TDCP 在钟跳历元仍可用,才配得上「免疫」。"
 },
 {
  "severity": "minor",
  "designs": "D1",
  "claim": "§4.5/§12.1 P3:「k0 时缓存卫星和 IODE;k1 时用补丁 P3 按同一 IODE 重算两端」,导出 rtkx_satpos_iode()。",
  "problem": "seleph 的 iode 过滤是精确匹配 `nav->eph[i].iode!=iode ⇒ continue`(ephemeris.c:453),但 BDS 的 iode 是 AODE(0–31,表示龄期,不唯一)、Galileo IODnav 也会回绕,同一颗星多条星历可能同 iode,匹配到的可能不是 k0 那条;ephpos 内部对 iode≥0 的分支本就存在,补丁本身不必要。",
  "fix": "克隆时缓存 eph_t/geph_t 副本(或 (toe,iode) 对),两端直接调 extern 的 eph2pos/geph2pos(D3 §4.5「用当前历元所用的同一份星历重算」),不走 seleph;P3 可删。"
 },
 {
  "severity": "minor",
  "designs": "D2",
  "claim": "§5.2:σ²_T=s_sys²[a_T²+b_T²/sin²el+c_T²·10^((45−C/N0)/10)]……「与 RTKLIB varerr 同族,pntpos.c:51,65」;D1 §4.6 引用 varerr 公式并用 P2 导出 rtkx_varerr。",
  "problem": "varerr(pntpos.c:51-79)是 fact²·[eratio²·(a²+b²/sin(el)+c²·10^{0.1(snr_max−snr)})+(d·Pstd)²],方差对 1/sin(el) 而非 1/sin²(el);D2 引用不准(不算错,是自家模型)。更实际的是 D1:若经 P2 直接调 rtkx_varerr 而用 prcopt_default(rtkcmn.c:204-215:err[1..2]=0.003、eratio=300、err[6]=err[7]=0),得到的是 0.9 m 且 snr 项与 Pstd 项都不生效,与其表中 a=b=0.3、含 psr_std 项的公式不符。",
  "fix": "D2 改引用措辞;D1 显式构造 prcopt(err[1]=err[2]=0.001,eratio=300,err[5..7] 与 k_std 按配置填)或干脆自算 σ²,不经 rtkx_varerr。"
 },
 {
  "severity": "minor",
  "designs": "D2/D3",
  "claim": "TDCP 模型只含几何/钟/卫星钟项(D2 §1「ΔI、ΔT 在 Δt≤1.5 s 忽略」;D3 §1.2「ΔI、ΔT、ΔN 视为 0」),未提相位缠绕。",
  "problem": "天线绕竖轴转 θ 时所有卫星载波共同多出 θ/2π 周(转 90° ⇒ 4.8 cm),一阶为公共项,只能靠钟差增量(D2 的 b_j−b_{j−1}、D3 的 b(t_g)−c_b)吸收;D1 §4.5 明写「相位缠绕的公共部分和接收机钟差增量一起被 Δclk 吸收」。D3 的 S_b=0.1 m²/s 在 0.1 s 内 σ≈0.1 m 能容纳,但一旦按 Allan 标定把 S_b 收紧到 1e-2 量级,急转弯的 4.8 cm 会被判成一致新息压进 tᴺ/pᵂ。",
  "fix": "文档中注明缠绕由钟差增量吸收;钟 PSD 标定时对 S_b 设下限使 √(S_b·Δt)≥|ω_z|Δt·λ/2π;在急转弯段做 TDCP NIS 分箱检查。"
 },
 {
  "severity": "minor",
  "designs": "D2",
  "claim": "§5.4 D4:「|ΔΦ+(D_j+D_{j−1})Δt/2|>max(0.5 周,4σ_DΔt)」,Δt 允许到 1.5 s。",
  "problem": "梯形积分误差 ≈ Δt³·D̈/12,D̈=jerk/λ;地面平台 jerk≈1 m/s³ 时 Δt=1.5 s ⇒ 约 1.5 周,远超 0.5 周阈值,长间隙历元会被系统性误判周跳而白丢 TDCP(D5 兜底但没必要)。10 Hz 正常节拍(Δt=0.1 s)无此问题。",
  "fix": "阈值加机动项 max(0.5 周,4σ_DΔt + j_max·Δt³/(12λ)),或 Δt>0.3 s 时只依赖 LLI/半周位与 D5。"
 },
 {
  "severity": "minor",
  "designs": "全部",
  "claim": "BDS/Galileo 电离层统一用 GPS Klobuchar×(f_L1/f)²(D1 §12.1、D2 §3.3、D3 §18.8),视为已知局限。",
  "problem": "核实无误:RTKLIB-EX ublox.c 解码 ion_cmp 但 pntpos.c ionocorr 只用 ion_gps(pntpos.c:204-250),Galileo NeQuick 未实现;gnss_comm 更没有频率缩放。中国境内 BDS 为主系统(D1 主钟挂 BDS)时,BDS Klobuchar(地理纬度、375 km 层高、BDT 地方时)与 GPS 版本差异在低仰角可达分米;三设计都只按 (0.5·I)² 计方差,但 I 本身系统性偏差不是白噪声,伪距 1 Hz 抽稀+κ 膨胀能部分覆盖,TDCP 中的 ΔI 速率项(D1 σ_rate 2 mm/s)在日出日落段可能偏小。",
  "fix": "保持现状但把 ERR_BRDCI 因子作为可配置项(0.5→0.7 起),并在开发集按系统分箱看伪距 NIS;若 BDS 箱 NIS 明显 >1,再考虑按 BDS ICD 自写 BDSK8。"
 }
]