> Advanced research tooling. For the supported public build/example path start with [REPRODUCING.md](../../docs/REPRODUCING.md). Historical run paths below are illustrative; acquire external data from its provider and supply your own workspace settings.

# tools/eval — 评测工具(P3-D 建,P4-EVAL 按 A52/A54/A55 改;AMENDMENTS A46 的简化评测)

归属:`tools/eval/**` 归 P3-D(OWNERSHIP §2c),P4 由 P4-EVAL 维护。SPDX:BSD-3-Clause。默认在远端 `/work/fastlivo2_gnss` 上跑
(本机编辑与单测;整机回放见 `docs/dev/LOCAL_HOST.md`:`--docker-cmd docker --profile local-dev|local-eval`)。**数据纪律(A2)**:评测集序列在评测阶段之前不下载、不回放、不打分——取数、回放、ATE、E5 都在碰任何文件之前
检查序列类别(`tools/fl2g/sequences.py`),评测集必须显式加 `--evaluation-phase`;开发集随便用。

## 流程

```
fetch_eval.sh  ──(data/m3dgr/filtered/<seq>.bag)──▶  run_matrix.py  ──(runs/eval/<名>/…)──▶  eval_ate.py  ──(ate.json)──▶  make_table.py
  取数/校验/过滤                                        臂 × 序列 × 种子                         evo ATE + 自检        │          主交付表
                                                                                              eval_aruco.py             │
                                                                                              (aruco.json, E5) ─────────┤
                                                                                                                  └──▶  dev_report.py
                                                                                                                        开发集报告(A59)
```

| 文件 | 作用 |
|---|---|
| `sequences.json` | 序列登记:云盘路径(分卷/单文件)、官方 sha256(与 `checksums.txt` 由单测逐项核对)、GT 路径与类型(rtk/aruco)、FAST-LIVGO 表 II 列 |
| `lidars.json` | host LiDAR profiles (A76): replay config/launch, bag topics, bag directory, the GT antenna lever in the host IMU frame (scoring) and the level-frame rotation (floor); `avia` (default, the pre-A76 layout) and `mid360` |
| `arms.json` | A46 的六个臂,**全部跑同一个 HEAD eval 构建**(A55):U(traj_W,`config/arms/U.yaml` 即 `gnss.enable: false`)与 L-PVT/T/T-pr/T-dop/T-tdcp(traj_N,`config/arms/<臂>.yaml` 覆盖 + 每序列 GNSS 输入) |
| `fetch_eval.sh` / `fetch_eval.py` | 单条序列的取数流水线(下文);`plan` 只列云盘与磁盘预算;`drop` 删过滤后的包;`status` |
| `run_matrix.py` | 批量确定性离线回放;`plan`/`run`/`status`;可 `--fetch` 滚动取数、`--drop-after` 跑完即删 |
| `eval_ate.py` | `evo_ape tum <GT> <est> -a --t_max_diff 0.05 --t_start T0 --t_end T1`,每运行四个值(E1 整条序列、激活 30 s 后、E4、e1o = 里程计输出的 E1,A61),另有 E1′ 诊断列 e1p/e1op(真值天线杆臂与对齐联合估计,TEST_PLAN §4、A64),每格中位数/范围;numpy 复算与 T_{N←W} 航向核对作仪器自检。ArUco 序列在这里阻断 |
| `eval_aruco.py` | E5:GNSS-denial01/02 的 ArUco 起终点误差,算法与 M3DGR `ArUco_evaluate.py` 一致(相对位姿,不是 ATE)。矩阵模式对每个运行评 `traj_W.tum` 与 `traj_N.tum`;输出 `aruco.json` / `aruco_runs.csv` / `aruco_cells.csv`,由 `make_table.py --aruco` 另表、`dev_report.py --aruco` 附在报告末 |
| `lever_floor.py` | A75/A76: the e1o floor of a host IMU point from the GT track alone (perfect IMU trajectory, heading from the GT motion, `--lidar avia|mid360`); needs no run |
| `e1traj.py` | A52 的 E1 轨迹:由同一步末的 traj_W/traj_N 精确导出 T_{N←W},激活前用激活时刻的 T_{N←W} 补齐;航向与 ACTIVATE 自检 |
| `gtbase.py` | A54 真值:`gt_timebase.yaml`(时基方法与平移)与 `trust_mask.csv` 的 `ate_exclude` 列(掩掉的时段) |
| `make_table.py` | 主交付表(`--mode main`)或开发集演练版(`--mode dev`),Markdown + CSV;论文值硬编码并注明出处 |
| `dev_report.py` | 开发集报告(NEXT_STEPS 第 1 步、A59 的素材):仪器自检、E1 与对 U 的成对比较、每格 GNSS 诊断(激活、接受率、NIS、TDCP 覆盖、f_p、事件)、各类观测的贡献、一致性标志;轨迹图与热图(见下) |
| `livgo_p2.py` | A75 修订 1:预测 P2(LIVGO-v2 的 TDCP 行是否作用于宿主)。读 `<矩阵>/<序列>/<臂>/s<种子>/livgo_tdcp_gain.csv`(LIVGO 门面在 `livgo/tdcp_gain_log` 为真时写出:同一历元带与不带 TDCP 行的后验位置迹),按登记的门槛判 P2a/P2b/P2c 与仪器自检;末行 `P2: MET / NOT MET / UNDECIDED`。单测 `tests/test_livgo_p2.py` |
| `setup_evo_env.sh` | 修镜像里 evo 不能用的问题(见下) |
| `launch_remote.sh` | 本机:同步到远端工作树并以 `systemd-run --user` 起长任务(自动带 `--setenv=ALIYUNPAN_CONFIG_DIR`) |
| `tests/` | 单测(`python3 -m unittest discover -s tools/eval/tests`;远端镜像里跑:`tools/dev/remote_tool.sh <树> <名> -- python3 -m unittest discover -s tools/eval/tests -v`) |

### 宿主雷达(A76:`--lidar avia|mid360`)
`tools/eval/lidars.json` 把「宿主」(GNSS 臂所在的 FAST-LIVO2)的雷达/IMU 对应的文件与数字放在一处:回放配置与 launch(`config/m3dgr_<lidar>.yaml`、
`launch/m3dgr_<lidar>_replay.launch`)、bag 话题与目录、真值天线(C-RTK 2HP)在宿主 IMU 系的杆臂 `gt_lever_imu`、宿主 IMU 系到水平系的旋转。
默认 `avia` 与 A76 之前的目录与命令行逐字相同;`mid360` 另有目录,不与 Avia 的过滤包混放:

| 工具 | `--lidar mid360` 的作用 |
|---|---|
| `bag_filter.py --lidar mid360` | 保留 `/livox/mid360/{lidar,imu}`、压缩彩图、`/ublox_driver/*`(其余 `--keep` 照旧) |
| `fetch_eval.sh --lidar mid360 fetch <seq>` | 过滤包、`.sha256`、`<seq>.fetch.json`、`<seq>.fetch/` 都写进 `data/m3dgr/filtered_mid360/`,解包暂存在 `data/m3dgr/fetch_mid360/`;原始包 `data/m3dgr/bags/` 与 Avia 的目录一概不动。同一把 `data/m3dgr/.fetch.lock`,一次一条。MID-360 的 GT 审计与 BRDC 与 Avia 相同,用 `--no-gt-audit` 跳过重复审计 |
| `run_matrix.py --lidar mid360` | launch 用 `m3dgr_mid360_replay.launch`(先载 `config/m3dgr_mid360.yaml`,臂的覆盖文件照旧在其后),默认 bag 取 `data/m3dgr/filtered_mid360/<seq>.bag`,`--fetch`/`--drop-after` 把 `--lidar` 交给 fetch_eval;manifest 记 `lidar` |
| `eval_ate.py --lidar auto` | 读 manifest 的 `lidar`(没有 = Avia),默认杆臂取该雷达的 `gt_lever_imu`;`--lever-ref x y z` 仍可覆盖;不同雷达的矩阵不能混评 |
| `lever_floor.py --lidar mid360` | 完美 IMU 轨迹的 e1o 地板(见下) |

**e1c(A76 判据 2)**:`eval_ate.py` 的第七个指标。e1o 轨迹(U:traj_W)的 IMU 点按**标定**杆臂 l(不拟合)移到真值天线 x = p + R l,再做与 e1o 相同的关联、时段、掩码和
SE(3) Umeyama 无尺度。等于 e1op 记录里的 `rmse_at_lever_ref`,单列成指标与 csv 列(`ate_e1c_m`、`e1c_*`);E1/e1o 照报。
**杆臂的来源(A77)**:`lidars.json` 的 `gt_lever_imu` 对两种宿主按同一程序取——水平两分量只在开发集上、用该宿主自己的 U 轨迹与全局 SE(3) 联合拟合(A58 的程序,`tools/calib/host_gt_lever.py fit`;MID-360 在水平系里拟合,因为它的 IMU 系相对竖直倾斜 31.8°),竖直分量取机械链值;评测集上不拟合任何杆臂。MID-360 的值不再取标定链的平移。

**e1o 地板**(`lever_floor.py`,A75 第三节的口径):只用 GT。完美 IMU 轨迹 p_IMU = p_ant − Rz(yaw) v,yaw 取 GT 水平速度方向(±2 行中心差分,速度低于 0.2 m/s 保持上一个航向),
v 是从宿主 IMU 原点指向真值天线的向量在水平系里的分量(Avia:A58 杆臂;MID-360:A77 的开发集拟合值转回水平系);再对 p_ant 做 e1o 的 SE(3) Umeyama,取 RMSE。
默认规则复现 A75 的 Avia 表(十条序列差不超过 0.0015 m);输出另带 0.5/1/2 s 窗口规则下的范围。

```bash
# MID-360 过滤包(一次一条;评测集加 --evaluation-phase;systemd 单元存活于 ssh 会话)
FL2G_REMOTE=... FL2G_PORT=... tools/eval/launch_remote.sh $T a76-run start fetch-m360-<seq> -- \
    bash tools/eval/fetch_eval.sh --lidar mid360 fetch <seq> --no-gt-audit [--evaluation-phase]
# 宿主 U 的矩阵(MID-360;U 不需要 GNSS 输入)
python3 tools/eval/run_matrix.py run --name a76-mid-<...> --lidar mid360 --arms U --seqs <seq,...> --seeds 0 \
    --devel head=<wt>:ws/devel --jobs 2 [--evaluation-phase]
python3 tools/eval/eval_ate.py --matrix runs/eval/a76-mid-<...> [--evaluation-phase]        # --lidar 由 manifest 带出
```

### fetch_eval(一条序列)
1. **来源**:`data/m3dgr/bags/<seq>.bag` 已在则直接用(默认不删;`--delete-local-original` 才删);否则 `aliyunpan ls` 列出云盘,
   每个文件一个 `aliyunpan download` 进程并行下载(每进程约 13 MB/s;aliyunpan 自己校验文件哈希)。
2. **解包**:分卷 `<seq>.partNNN.exe` 是一个 7-Zip 自解压 exe 按文件名顺序切开的字节段(云盘里的 `merge_script.ps1` 就是按序拼接);
   默认用 7-Zip 的分卷读取(`<seq>.exe.001…` 链接到各段,不写合并副本,省一个压缩包大小的磁盘),`--merge cat` 则真的拼接
   (每段追加完立刻删);两种方式由单测证明给出同一个 bag。归档里必须恰好一个 `.bag`。
3. **校验**:sha256 对官方 `checksums.txt`(并与登记表一致);`rosbag info` 查索引,未索引 → `rosbag reindex --output-dir`;
   reindex 失败且 sha256 不符 → `--allow-repair` 时走 A3 路线(`tools/repair_bag.py` + reindex,记为缺陷)。
4. **输入**:GT 从已下载的 GT 树拷到 `data/m3dgr/gt/<seq>.txt`(已存在则只比对,不覆盖);bag 覆盖的每个 UTC 日的 IGS BRDC
   (BKG 镜像)到 `data/brdc/`(A38:nav_rinex 是必需输入)。
5. **审计**(在未过滤的包上、删除之前):`tools/gt_audit.py`(轨迹真值;P4 给出 eval_ate 用的 GT 时基),`--i2` 时再跑
   `tools/m3dgr_audit.py`。`audit/<seq>/` 还不存在时写进去;已存在(WP-g 的归档)则写到 `<seq>.fetch/audit/` 并与归档的
   `gt_audited.tum` 逐字节比对。
6. **过滤**:`tools/bag_filter.py`,保留 `/livox/avia/{lidar,imu}`、压缩彩色图、`/ublox_driver/*`,另保留
   `/mavros/global_position/raw/fix`、`/odom`(gt_audit 的输入,体量小,使 GT 审计可在过滤包上复跑);工具自带逐消息核对。
7. **清理**:本次创建的未过滤包删除。每一步写进 `data/m3dgr/filtered/<seq>.fetch.json`,日志在 `<seq>.fetch/`。

磁盘:下载、解包、reindex、过滤之前都按「需要 + 3 GiB 余量」检查可用空间,不够 → 退出码 75(run_matrix 在下一次删包后重试)。
同一主机同一时刻只有一个取数(`data/m3dgr/.fetch.lock`)。

### run_matrix(沿用 `docs/dev/ref_r_gates/replay_run.sh` 的约定)
每个运行一个容器(`-u 1000:1000`,容器内 `nice -n 19`,`OMP_NUM_THREADS=1`,私有 roscore,ROS 日志与上游 `Log/` 绑定到运行目录,
BUILD_ID/镜像 digest/bag sha256 进 manifest),运行目录从不复用;完成判据与 replay_run.sh 相同的四条(`replay_stats.json` 的
`"complete": true`、控制台 `fastlivo_replay: done`、节点退出码 0、`manifest_check.py --run` 通过),`run_status.json` 同格式。

- **并发**(A41;默认值按下文「内存」改为 3):本工具最多 `--jobs` 个(默认 **3**,上限 8);每个运行在整个生命期持有 8 个主机级
  槽锁之一——槽 0 就是 replay_run.sh 的 `$ROOT/.replay.lock`,槽 1–7 是 `$ROOT/.replay.slot<i>.lock`——所以本工具的多个实例与
  replay_run.sh 加起来也不超过 8。锁是 docker 客户端继承的打开文件描述,容器客户端退出才释放。**并发数受内存约束,不受核数约束**,
  见下节;`--jobs` 大于 3 之前先按下节的实测峰值重算。
- **内存**(硬约束,2026-09-25 事故之后):见下文「内存」一节——启动前 MemAvailable ≥ `--min-avail-gb`(16 GiB)才启动、
  每容器 `--memory`/`--memory-swap` = `--mem-limit-gb`(16 GiB)与 `--oom-score-adj 1000`、运行中低于 `--brake-avail-gb`(6 GiB)
  杀最年轻的回放并重新排队(不占 `--retries`)、每次尝试记峰值 RSS。
- **失败重试一次**(`--retries 1`),新目录 `s<k>.retry1`;每次尝试都保留并记录。运行超时(`--timeout-factor` × 序列时长 + 900 s)即 `docker kill`。
- **可续跑**:同名矩阵再跑一次,已完成的运行不会重跑;中断的尝试记为失败尝试。
- **臂的配置**:有覆盖文件或需要 GNSS 输入的臂在运行目录里生成包装 launch:`<include … m3dgr_avia_replay.launch pass_all_args="true"/>`
  之后再 `rosparam load` 覆盖文件(roslaunch 同名参数后设者生效、yaml 字典展开成叶子参数),故不改动任何冻结文件;覆盖文件不得含
  `replay`/`output` 顶层键。每序列 GNSS 输入 `gnss.nav_rinex`(取数记录里的 BRDC)与 `gnss.time.calib_file`(`audit/<seq>/time_calib/<seq>.yaml`,
  无则 `_record_time.yaml` 先验并在 manifest 注明)。U 臂(A55)同样经包装 launch 装 `config/arms/U.yaml`(`gnss.enable: false`),不要 GNSS 输入。
- **一个矩阵一个二进制(A55)**:各臂解析到的 `--devel` 构建必须相同(U = HEAD 零通道,G-零通道门已证逐位 = ref-R),否则拒绝;
  `--allow-mixed-builds` 只给诊断用(记入 manifest,eval_ate 会把这种成对比较标 †)。
- **滚动取数**:`--fetch --max-bags N --drop-after`:缺包的序列一次取一条,磁盘上同时最多 N 条过滤包(含正在取的),序列的全部运行结束即删包。
- manifest:`runs/eval/<名>/matrix_manifest.json`(每个事件后原子重写),事件日志 `matrix_events.log`。

### 内存(2026-09-25 事故之后;硬约束)
**事故**:2026-09-25 20:20 矩阵 `p4_dev` 以 `--jobs 8` 同时起了 8 个 Occlusion04 回放(U 种子 0–4、L-PVT 种子 0–2)。约 20 分钟后
各回放都到了包尾附近(最后一行 traj_W 距包尾 13–52 s),远端 62 GB 内存耗尽;内核 OOM(21:03 起)反复选中的是 oom_score_adj
= 200 的 docker **客户端**(用户 systemd 单元里的进程,RSS 只有几 MB;容器里的回放不在这个 cgroup,杀客户端不释放内存),整机卡死,
21:53 被重启,打断了同机另一个项目的训练。8 个运行无一完成(已删,见 AMENDMENTS A59)。根因:FAST-LIVO2 的体素地图不滑窗
(`map_sliding_en: false`),视觉地图点也只增不减,**单个回放的内存随回放时长单调增长、峰值在包尾**;8 个同序列同时到包尾 ≈ 8 × 峰值。

**规则**(`run_matrix.py`,默认值即本轮硬约束):
- `--jobs` 默认 **3**;
- **启动闸**:每启动一个回放之前重新读 `/proc/meminfo` 的 MemAvailable(即 `free -g` 的 available 列,GiB),< `--min-avail-gb`
  (默认 16)就不启动、等下一轮轮询(`matrix_events.log` 记「memory gate closed/open」);
- **单容器上限**:`docker run --memory = --memory-swap = --mem-limit-gb`(默认 16 GiB,无 swap):失控的回放(发散后地图爆涨)在自己的
  cgroup 里被杀(节点退出码 −9,运行不完整、照常重试一次),不拖垮主机;`--mem-limit-gb 0` 取消;
- **OOM 优先级**:`--oom-score-adj 1000`(默认):万一仍发生全局 OOM,内核先杀回放,而不是 sshd、docker 客户端或别的项目;
- **刹车**:运行中 MemAvailable < `--brake-avail-gb`(默认 6)⇒ `docker kill` 最年轻的回放(损失最少)并重新排队;这次尝试在
  run_status.json 记 `"memory_brake": 1`,不占 `--retries`(续跑时读回),同一运行至多刹 5 次;两次刹车至少间隔 30 s;
- **记录**:每次尝试在 manifest 里记 `mem_available_at_start_gib`、运行期间最低 `min_mem_available_gib`、`peak_rss_gib`(该运行
  fastlivo_replay 进程的 VmHWM = 内核记的峰值常驻内存,每 `--mem-sample-s` 2 s 采样;按节点的 `__log:=/r/<运行目录>/roslog/…`
  参数认运行),完成时 `peak_rss_gib` 也写进 run_status.json。

- **磁盘**:远端 `/data` 与别的项目共用(2026-09-25 只剩约 23 GB),一个运行 95–150 MB(大头是 console.log、filter_cov.csv、
  state_trace.txt)。`--min-free-disk-gb`(默认 8)是启动闸的第二个条件;`--gzip-after console.log,filter_cov.csv,state_trace.txt,timing.csv`
  在运行判定完成(完成判据已读过 console.log 与 state_trace.txt)之后把这些文件原地压成 `.gz`(约 4 倍),manifest 记 `gzipped`。
  eval_ate/dev_report 不读这几个文件;需要 state_trace.txt 的工具(gt_audit、m3dgr_audit、gnss_time_calib)先 `gunzip -k`。

**实测单回放峰值**:见 AMENDMENTS A59(`p4_dev` 的探针运行与全矩阵各运行的 `peak_rss_gib`)。

### eval_ate(A52/A54/A55)
- **E1(主表)= 整条序列**。U 用 `traj_W.tum`;GNSS 臂激活后用 `traj_N.tum`,激活前的宿主位姿用**激活时刻**的 T_{N←W} 映射成 N 系补齐
  (等价于「激活前就是纯 LIVO」,只用激活时刻的一次估计);激活后没有 traj_N 行的步(REINIT 重新收集)用此前最近一次的 T_{N←W}(因果);
  从未激活的运行就是其 traj_W(与同种子 U 逐位相同)。T_{N←W} 不需要新输出:traj_W 与 traj_N 在同一步末以同一 t_k、同一宿主状态写出,
  DESIGN §1 ⇒ T_{N←W} = T_{N←I} ∘ T_{W←I}⁻¹ = (M, t − M o^W),精确到文本 1e-9 舍入(`e1traj.py`)。
  时段 = 真值与该序列全部运行(各臂各种子)的交集;`--span seed` 改为逐种子。按时间排序,同戳行只留最后一行(IMU 初始化期间 traj_W 重复首戳)。
- **附表**:`post30` = 同一批轨迹在 [该时段组里最晚的激活 + 30 s, E1 时段末] 上(U 也在此时段上评;从未激活的运行没有此值);
  `e4` = 各臂 traj_W 在 E1 时段上(检验 GNSS 是否改动宿主;U 的 e4 即其 e1)。
- **E1′(TEST_PLAN §4,A64;诊断列,主表仍是 E1/e1o)**:`e1p` 用 E1 的同一文件(`est_e1.tum`)、同一时段与关联,`e1op` 用 e1o 的
  文件(`est_e1o.tum`;U 的 e1op 即其 e1p);真值天线在 IMU 系的杆臂 l(3 参数)与 SE(3) 对齐联合估计:min Σ|R(p_i + R_i l) + t − y_i|²,
  Levenberg–Marquardt 从 l = 0 起,法方程在 Jacobi 缩放后的特征基里解,缩放特征值低于最大值 1e-10 的方向不动(横滚/俯仰恒定时竖直杆臂
  与 t_z 精确共线;只是弱的方向照常估计)。各臂各运行同一程序。协议 E1 直接拿 IMU 轨迹比 RTK 天线(与 M3DGR/FAST-LIVGO 一致),
  杆臂的水平部分随航向转动、对齐去不掉,是 E1 的地板(A63:开发集上 U 的 E1 大半是它)。只有 numpy(evo 没有这个指标);自检不过即
  INSTRUMENT:l = 0 的起点复现 numpy 的 E1(1e-6)、E1′ 不大于它、第二个起点 (0.3, −0.3, 0.3) m 到同一极小(相对 1e-6)、LM 收敛。
  ate.json 记拟合的杆臂、形式 σ(竖直分量只由横滚/俯仰的变化观测)与秩;`--lever-ref x y z` 另记固定杆臂处的 ATE(A63 用 A58 的
  −0.176 −0.016 0.401)。csv 末尾追加 `ate_e1p_m`、`ate_e1op_m` 与两组杆臂列(运行)、`e1p_*`/`e1op_*`(格)。
- **e1o(A61 F13)**:GNSS 臂的里程计输出 `traj_N_odom.tum`(配准在激活时刻的估计固定,之后对该时刻的修订不进入输出,漂移修正全部
  保留)按与 E1 相同的构造(激活前用激活时刻的 T_{N←W}),同一 E1 时段;U 的 e1o 即其 e1;早于 A61 的运行没有该文件,记「无」。
  主表(`make_table.py`)单列「我们的紧耦合·里程计输出」,与 FAST-LIVGO 论文列同口径(其 W→ENU 初始化后固定)。
- **真值(A54,`gtbase.py`)**:`audit/<seq>/gt/gt_timebase.yaml` 给出时基方法(`p4` | `record_time` | `none`)与加到 `gt_clean.tum` 戳上的
  `time_shift_applied_s`(可选 `gt_file` 指向已平移文件,逐行核对);ATE 掩码 = `trust_mask.csv` 的 `ate_exclude` 列(可由
  `ate_mask_file`/`ate_mask_column` 改),连续被掩的真值行构成时段 [首, 末],真值与估计两侧都去掉该时段内的行,各时段内被掩掉的秒数记入
  ate.json 与主表。早于 A54 的审计(无 gt_timebase.yaml):P4 通过的照用 `gt_audited.tum`(无掩码,表中注明),P4 失败的阻断(重跑 gt_audit)。
  ArUco 序列(GNSS-denial01/02)不是轨迹真值 ⇒ 阻断。它们的指标是 E5,由 `eval_aruco.py` 另算(见下),不进 ATE 列(A56)。
- **仪器自检**(不过即该运行 INSTRUMENT、退出码 4):每个 ATE 用 numpy 重算(evo 1.31.1 的关联规则、只裁参考轨迹、Umeyama 无尺度),
  相对差 > 1e-6;GNSS 臂激活时刻 T_{N←W} 的航向(M 第一列方位角 = ψ + O(倾角²))对 `gnss_epochs.csv` 同步的 ψ,容差 1° + 倾角²/2;
  `events.log` 第一个 ACTIVATE 与第一行 traj_N 同一步(≤ 1 s)。
- **协议核对**(报告,表中 †):GNSS 运行激活前的 traj_W 与同种子 U 的逐位相同(TEST_PLAN I11 后半句、A55;从未激活则全段);
  同一序列 U 与 GNSS 臂的 `build_sha256` 相同(A55)。种子塌缩:每个运行在 E1 时段内的轨迹行取 sha256,同格哈希相同的种子记为
  `identical_seed_groups`(格内 `n_distinct` 条独立轨迹),表中标 ‡。
- 输出:`ate.json`(全部 + 来源)、`ate_runs.csv`、`ate_cells.csv`;每运行 `est_e1.tum`/`est_e4.tum`(已去掩码时段,两种仪器读的就是它)、
  `ape_<指标>.zip`、`evo_<指标>.log`;每序列 `gt_eval.tum`(评测时钟上、已去掩码时段的真值)。控制台只打印计数与标记,不打印 ATE 值。

### eval_aruco(E5;TEST_PLAN,A56)
GNSS-denial01/02 的真值是起点/终点 ArUco 的**相对位姿**(3 行空白 + 3×3 旋转 + 空白 + 3 行平移 + 空白 + `bag_time`,无文件尾换行;官方脚本用 0 起下标 3:6 与 7:10,最后一行是时长),不是轨迹。官方 `ArUco_evaluate.py` 不算 APE:估计的相对位姿是 `inv(T_first) @ T_end`(终点取末尾连续相同位姿段的第一帧,轨迹停更则跟踪率下降),平移误差是两段平移之差的欧氏范数(米;README 说论文用这个更严的平移误差),旋转误差是两段旋转矩阵之差的 Frobenius 范数(官方表头写 Rotation error(m),量本身无量纲),RMSE 把这两个数平方平均后再开方,跟踪率是 `(t_end − t_start) / bag_time`,超过 100% 截断。不对齐,也不做时间关联。

本工具复现上述定义。矩阵模式读每个完成运行的原始 `traj_W.tum` 与 `traj_N.tum`(不经 E1 的激活补齐;U 没有 N,记「无」,不因此丢掉 W)。真值默认 `data/m3dgr/gt/GTGNSS-denial<nn>.txt`(fetch_eval 的拷贝),否则 `sequences.json` 的 `gt_raw_root` + `gt`。评测集必须 `--evaluation-phase`(A2),检查通过之前不读轨迹。`--reference` 加 `--traj` / `--traj-dir` 是官方那种单文件/目录算法器,`--seq` 时同样受 A2 约束。

与官方脚本的差别写在 `eval_aruco.py` 模块说明里:不依赖 evo(位姿矩阵按 evo 1.31.1 的 TUM→SE(3) 来建,单测在本机装了 evo 时对一下);官方预检查拒绝含注释或空行的 TUM,这里同样拒绝;JSON/CSV 保留全精度,官方打印两位小数再按四舍五入后的字符串排序。

```bash
python3 tools/eval/eval_aruco.py --matrix runs/eval/<名> --seqs GNSS-denial01,GNSS-denial02 --evaluation-phase
python3 tools/eval/make_table.py --ate <eval>/ate.json --aruco <eval>/aruco.json --mode main \
      --out-md <eval>/main_table.md --out-csv-aruco <eval>/aruco_table.csv
```

### dev_report(开发集报告;NEXT_STEPS 第 1 步)
输入 = eval_ate 的 `ate.json`(E1/post30/E4、时段、自检状态、`<eval>/<seq>/gt_eval.tum` 与各运行的 `est_e1.tum`,即 ATE 读的同一批数)
加上每个完成运行目录里的 `gnss_epochs.csv`、`summary.json`、`events.log`、`run_manifest.json`(`config_dump` 给出各类的 κ_AR)。
不回放、不重算 ATE。输出(默认 `<eval>/dev_report/`):
- `dev_report.md`:A59 草稿素材。§1 仪器自检(ate.json 的 INSTRUMENT、诊断链 = summary.json 与 gnss_epochs.csv 逐类重算一致、
  A55 同一二进制、I11 激活前宿主 = 同种子 U、完整性、种子塌缩);§2 E1 与对 U 的成对比较(同种子成对差、胜数、比值中位、精确
  Wilcoxon 符号秩 + 同序列 Holm,判定按 TEST_PLAN §4;5 个种子时最小可达 p = 0.0625 ≥ 0.05,判定只能是平局,表中注明,数照报);
  §3 每格 GNSS 诊断(激活数与时刻、激活时 σ_ψ 与路径、各类接受率、合并 NIS/dof 自报值、κ×自报 = 白等效 NIS 上界(A58 裁定 1)、
  多普勒/TDCP 的逐历元 χ² 双侧 95% 带内比例、TDCP 覆盖、非退化段 f_p 中位、事件计数);§4 T 对 T−pr/T−dop/T−tdcp/L-PVT 的成对比较
  (各类观测的贡献);§5 一致性标志(DESIGN §5 D4 的 [0.8, 1.25]、TEST_PLAN §3 锁死判据、REANCHOR/REINIT、未激活、f_p);
- `dev_report.json`(全部 + 来源)、`dev_report_runs.csv`、`dev_report_cells.csv`;
- `fig/traj_<seq>.png`(真值与 U/T/L-PVT 的 E1 轨迹俯视图,选 T 的 E1 为中位数的那个种子,各自用 ATE 同款 SE(3) Umeyama 对齐;
  GNSS 臂的全局输出实线、里程计输出(traj_N_odom)虚线;右侧逐位姿误差随时间,标 T 的激活时刻,A54 掩掉的真值时段画灰带,图例带
  各条的 E1 与 E1′)、`fig/heat_global.png` 与 `fig/heat_odom.png`(全局/里程计两版:序列 × 臂,左 E1、右 E1′,对 U 的成对比值中位,
  log2 色标,格内为中位数、比值与胜数,行标带 U 自己的中位数)、`fig/heat_nis.png`(序列·臂 × 类:NIS/dof × κ 与接受率)。
  matplotlib 不可用时只出表并在报告里写明。
- 表按四个口径(全局 E1、全局 E1′、里程计 E1、里程计 E1′)各做一族对 U 的成对比较与 T 对消融臂/L-PVT 的贡献比较;附表给 post30、E4、
  E1′ 拟合的杆臂中位数、激活路径与时刻。`--white-review <json>`(可重复;`tools/calib/p4_calib.py review` 对 T-diag 运行的输出)
  另出精确白等效 NIS 表(伪距、多普勒逐历元消钟,TDCP 取自报值,按 dof 合并并给逐种子范围)。
- 诊断链自检(summary.json 对 gnss_epochs.csv)按门面的汇总规则核对 TDCP 接受率:`gnss_fusion.cpp` classStats 在 `if (!s) return;`
  之后才加总,整对成员都在建行前丢失的收尾历元(无 TDCP 行集)在 csv 里记为 mask 候选、不进 summary.json;差别恰好是这些历元时判
  「PASS (facade totals rule)」并列出,其余差别仍是 MISMATCH。报告里的接受率按 csv(含这些候选),与 A58/A62/A65 相同。

退出码:0;4 = 仪器自检未全过(报告照写);3 = 有序列被 A2 拒绝或 eval_ate 阻断。一致性标志只报告,不改退出码
(是否「能用 NIS、接受率、TDCP 贡献解释 T 对 U 的表现」是判断,本工具只给判断所需的数)。评测集序列同样要 `--evaluation-phase`。

### evo 在镜像里不能用(镜像缺陷,已登记 INTERFACE_CHANGES)
Dockerfile 先装 apt `python3-scipy`(1.3.3),再 pip 装 `evo>=1.20,<2`,后者拉来 numpy 1.24.4;scipy 1.3.3 仍调 `np.typeDict`
(numpy 1.24 已删),`evo_ape` 在 import 时崩溃。`setup_evo_env.sh` 用镜像自己的 pip 把 scipy 1.10.1 的 cp38 wheel(sha256 固定)
装到 `<root>/tools/eval_pylib/site`(仓库外),`eval_ate.py` 只在调用 evo_ape 时把它放到 PYTHONPATH 前面;镜像本身不动。

## 命令

```bash
T=/workspace/fastlivo2_gnss_wt/p3-d            # 本机树
L=tools/eval/launch_remote.sh
# 一次性:evo 修复(远端主机)
ssh -p 22 user@build-host.example.org bash /work/fastlivo2_gnss/wt/<名>/ws/src/FAST-LIVO2/tools/eval/setup_evo_env.sh
# 取数(远端长任务;评测阶段对评测集加 --evaluation-phase)
$L $T p3-d-run start fetch-<seq> -- bash tools/eval/fetch_eval.sh --wt-home p3-d-run fetch <seq> [--i2]
$L - - logs fetch-<seq> 40
# 批量回放(A55:全部臂同一个 HEAD eval 构建;U = gnss.enable false)
$L $T p3-d-run start mx-<名> -- python3 tools/eval/run_matrix.py run --name <名> --arms U,L-PVT,T,T-pr,T-dop,T-tdcp --seqs <seq,...> \
      --seeds 0-4 --devel head=<HEAD eval 树>:ws/devel --jobs 3 [--fetch --max-bags 2 --drop-after --evaluation-phase]
# ATE 与表(镜像内,经 remote_tool.sh)
tools/dev/remote_tool.sh $T p3-d -- python3 tools/eval/eval_ate.py --matrix runs/eval/<名> [--evaluation-phase]
tools/dev/remote_tool.sh $T p3-d -- python3 tools/eval/make_table.py --ate /r/runs/eval/<名>/eval/ate.json --mode main \
      --out-md /r/runs/eval/<名>/eval/main_table.md --out-csv /r/runs/eval/<名>/eval/main_table.csv \
      --out-csv-supp /r/runs/eval/<名>/eval/supp_tables.csv
# 开发集报告(A59 素材;默认写到 runs/eval/<名>/eval/dev_report/)
tools/dev/remote_tool.sh $T p3-d -- python3 tools/eval/dev_report.py --ate /r/runs/eval/<名>/eval/ate.json
```
