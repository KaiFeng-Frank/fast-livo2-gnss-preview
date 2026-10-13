> Historical research record (through 2026-10-03). Paths are illustrative; archived runs are not bundled. For current status and executable entry points see [the repository README](../../README.md). Current release docs/LICENSING.md supersedes historical license notes. This record does not authorize new experiments or assert current release validation.

# ref-R 门禁记录(WP-f;P2b 在 HEAD 上重跑,2026-09-25)

**被门禁的源码 = 分支 `p2b-f` 的提交 `35ee6c5`**(= 主线 `65b6752` + WP-f P2b:A40 的 validateConfig 两条规则、manifest 并入回放统计、门禁脚本)。tag `ref-R` 打在其后只改文档的提交上(§8)。R 的内容(AMENDMENTS A27、TEST_PLAN §4):0d2c034 + processOnce 抽取 + `replay_main` + 压缩图直读 + 输出写出器 + `IMU_Processing.h` 类内 `=0.0` + **`VIOManager::updateState` 越界读保护**(维护者 65b6752;越界点清零其 z/H 行后跳过,A27)。报告写「U 为 0d2c034 + 越界读修复」。

远端 `/work/fastlivo2_gnss`,主机 `archived-build-host`(Xeon Gold 6248R,AVX-512),镜像 `fastlivo2-gnss:noetic`(`sha256:d3d3f715ee32b52dd56e192b3162d20576a89f2591e060475d87c9a1388f291b`)。两个远端工作树:`wt/p2b-f`(默认 profile)与 `wt/p2b-f-strict`(eval profile;回放、valgrind 都在这里)。所有数字都由 `docs/dev/ref_r_gates/` 的脚本产生(命令见各节与该目录 README)。

**数据纪律(A2)**:门禁序列是开发集 **Grass02**(A26,172 s,可以看任何输出);Dark01 属评测集,本文件对它只记录**哈希、行数、计数**,没有查看任何轨迹或状态数值。bag sha256 由 runner 计算并与官方一致:Grass02 `00734d46…daba43f5aa`(= M3DGR `checksums.txt`),Dark01 取旁文件 `Dark01.bag.sha256`。

## 0 结论

| 门 | 内容 | 结果 | 证据 |
|---|---|---|---|
| ① | `remote_build.sh --ros` 默认 profile;`--ros --strict` eval profile | **PASS**(两者零 error/warning) | §1 |
| ② | 编译命令门(默认 profile 对 `ws_upstream`,含 vikit) | **PASS** `RESULT: IDENTICAL (modulo added include dirs)` | §2 |
| ③ | G-确定性(eval):Grass02 全程 seed 0 两次;Dark01 前 60 s seed 0 两次(只比哈希) | **PASS**:两对各 7 个产物逐字节相同,build/config 身份相同 | §3 |
| ④ | I11(A30 语义):Grass02 seed 0..9 各前 60 s | **PASS**:45/45 对 state_trace 不同,10 个 IMU 初始化窗口两两不同 | §4 |
| ⑤ | valgrind memcheck(`-mno-avx` eval 构建)Grass02 前 60 s | **PASS**:12 个上下文全部是已登记的只进打印的上游项(计时 printf 与其 stdout 写出);Invalid 读写 0 | §5 |
| A40 | `writeManifest` 并入 `extra_json`;replay 写入全部回放统计 | **PASS**:16/16 用例;全部 15 次回放(含 valgrind)的 manifest 检查通过 | §6 |
| ctest | `--core` 19 项,两个 profile | `test_wpf_serialisation` 9/9 两个 profile 都过;其余 FAIL 均为维护者记录的既有项 | §7 |

## 1 构建(①)

| 构建 | 命令 | 结果 |
|---|---|---|
| 默认 profile | `gates_local.sh <tree> p2b-f build-default` = `remote_build.sh <tree> p2b-f --ros --clean` | **通过**:rsync 07:45:36,等锁后目标文件 07:48:50→07:52:02,19 个目标(含 `fastlivo_mapping`、`fastlivo_replay`、`livo_gnss_ros`),零 error/warning;`CMakeCache`:`FASTLIVO_OMP=ON`、`FASTLIVO_STRICT_FP=OFF`、`CMAKE_CXX_FLAGS` 空;`BUILD_ID = 35ee6c5 0` |
| eval profile | `gates_local.sh <tree> p2b-f-strict build-eval` = `remote_build.sh <tree> p2b-f-strict --ros --strict --clean` | **通过**(A28 之前此步编不过):目标文件 07:52:17→07:55:30,零 error/warning;`FASTLIVO_STRICT_FP=ON`、`FASTLIVO_OMP=OFF`、`CMAKE_CXX_FLAGS=-ffp-contract=off`;`compile_commands.json` 83 个 TU 全带 `-ffp-contract=off`、无 `MP_EN`、无 `SHELL:` 残留,带 `-include omp.h` 的恰为 8 个上游 TU(IMU_Processing、LIVMapper、frame、main、preprocess、vio、visual_point、voxel_map),上游 TU 全带 `EIGEN_DONT_PARALLELIZE`;二进制自报 `build_sha256 = f453a0fb6631bef74548afdb15de0be0277d9e8ee008c81c7bc7af8baaf1738f`,`compile_defs` 探针 `fp_contract_probe=unfused` |
| valgrind 变体(A29) | `gates_local.sh <tree> p2b-f-strict vg-build` = `build_eval.sh p2b-f-strict ws build_vg devel_vg "35ee6c5 0" -mno-avx`(WP-g 的 `--vg` 尚未合并;旗标 = `--strict` 的 catkin 步 + `-mno-avx`,不再额外加 `-include omp.h`) | **通过**:08:06:44→08:15:02 rc=0,旗标 `-ffp-contract=off -mno-avx` |

两个 profile 在两个远端工作树里分开构建(同维护者的 maint / maint-strict),主机 flock 串行:默认构建的最后一个目标文件 07:52:02,eval 构建的第一个 07:52:17。

## 2 编译命令门(②)

默认 profile 的 clean 构建、vikit 补丁已打:
```
gates_local.sh <tree> p2b-f compile-gate
= python3 tools/dev/compare_compile_commands.py ws_upstream/build/compile_commands.json wt/p2b-f/ws/build/compile_commands.json --head-root /r/wt/p2b-f/ws/src --include-vikit
RESULT: IDENTICAL (modulo added include dirs)          (exit 0)
```
上游 24 个 TU(fast_livo `src/` + rpg_vikit)在 HEAD 侧全部同名存在;默认 profile 下没有任何 TU 带 `-include omp.h`(A28 只在 STRICT 下生效)。**通过**。

## 3 G-确定性(③)

eval 构建(`wt/p2b-f-strict/ws/devel`,`build_sha256 f453a0fb…`),`seed_gate.sh`,每次运行独立容器、主机级回放锁(同一时刻只一个回放)、容器内 nice 19、`OMP_NUM_THREADS=1`。完成判据四条(`replay_stats.json` complete、`done` 行、节点退出码 0、manifest 检查)全部成立的运行才计入;`hash_runs.sh` 比较 7 个产物并核对两次运行的 build/config 身份。

### 3a Grass02 全程(开发集)
`gates_local.sh <tree> p2b-f-strict seed-gate ws/devel p2b-f/grass02`(07:56:51→08:07:26)

| 产物 | grass02_s0_a = grass02_s0_b(sha256) | 行数 |
|---|---|---|
| state_trace.txt | `2affc187b225df978dcdd32f3c4a41b8d5613994a147fcd3084bcdafc430e2a8` | 10356(1 行列名 + 10355 周期) |
| traj_W.tum | `35b515f0fd8cdee6264c2e8904331827edfe2a62e127f1beef6c3cad3f81e4d7` | 5177(= VIO 周期) |
| filter_cov.csv | `7cc694ea626b486fd063e8d7e6cefcc88facdf9bd92e0e967ff04622a7f19385` | 15534 = 1 + 5178×2 + 5177 |
| Log/imu.txt | `0b4f7d4b6adc817df29177dc5b8d6067b0b5d5aa5292a5189b8685b52efd9fbc` | 35079 |
| Log/mat_pre.txt | `be8085dd37463720ab1977d262140a850fa4093b9f3fe0755094a95eb4eacac5` | 10355 |
| Log/mat_out.txt | `ca44f0d81c8e9b230a2fea2e99953f8e7a28e99501e37f27b254888b7c71ae83` | 6887 |
| Log/result/<seq>.txt | `86dfe9620852488241e09af3c2e77d7cade9a94481812ac2a380db76cf24a2c6` | 3444 |

`hash_runs.sh`:7 个产物全部 `identical`,`build_sha256 same, config_sha256 same`(`config_sha256 = 214f2abac4a04841…`,与第一轮同一份 M3DGR 配置)。两次都是 10355 周期(5178 LIO + 5177 VIO),视窗起点 = 记录序第 0 帧 LiDAR = bag_begin + 0.053854 s,132174 条消息(1729 LiDAR / 35125 IMU / 5183 压缩图,`n_img_raw 0`,0 类型不匹配),`run_status.json` 均为 `complete=1, node_exit_code=0, manifest_ok=1`。**通过**。

格式审计(开发集,顺带):state_trace 每行 47 列(周期、标志、t_k、R(9)、p、v、b_g、b_a、g、τ、diag P(19)),`filter_cov.csv` 的 prior/postLIO/postVIO 行数 5178/5178/5177,`events.log` 空、`gnss_epochs.csv` 只有表头(ref-R 无 GNSS)。

### 3b Dark01 前 60 s(评测集,只比哈希)
`gates_local.sh <tree> p2b-f-strict seed-gate ws/devel p2b-f/dark01_60s data/m3dgr/bags/Dark01.bag 0 60 60`(08:37:25→08:39:33)

| 产物 | dark01_60s_s0_a = _s0_b(sha256) | 行数 |
|---|---|---|
| state_trace.txt | `b29a1d51a3f60198ef353e075b8aed3f61318a8711770249749e642353aa2842` | 3594 |
| traj_W.tum | `f6095b4755276113c35e3169045988b4adb43f35d7fb319dc59d3f06460374a6` | 1796 |
| filter_cov.csv | `40f127262c429f27bead67800812dce35563908c123c34f64fa9d81b9014d844` | 5391 |
| Log/imu.txt | `140754a1a3df366b1d4e9e321c10de8a87054e89b27601ee4b2f068ae82b54d3` | 12139 |
| Log/mat_pre.txt | `90bd841d035ae38d4eb3643b821cdb1a8b7936f1ebd895046806d8a169b13334` | 3593 |
| Log/mat_out.txt | `79275a1026fb9c400b6cb7dc74c9f44ea921de1b4b83c6e2fa0551d100130402` | 2343 |
| Log/result/<seq>.txt | `70a4ad9b5206d3c8fc162d1d35ae32218f2ca36da32ec7fb9184e689f2b36ca1` | 1172 |

7 个产物全部 `identical`,身份相同;3593 周期(1797 LIO + 1796 VIO)。这一对是**判别性**的:没有保护的上游在同一窗口里第 649 周期就分岔(§9),而 60 s 覆盖到第 3592 周期。`Log/imu.txt` 与第一轮 `diag2_sd0` 的哈希前缀 `140754a1a3df` 相同(同一 seed 0 的 IMU 初始化窗口)。**通过**。

## 4 I11 种子两两不同(④,A30:seed k = 记录序第 k 帧 LiDAR)

同一条 seed_gate 链,Grass02,seed 0..9 各前 60 s(08:07:26→08:24:42),同一二进制(全部 `build_sha256 f453a0fb…`)、同一配置(全部 `config_sha256 214f2aba…`):

| seed | 有效偏移(s) | 周期(LIO/VIO) | n_msgs | Log/imu.txt sha256 前 12 位 | state_trace sha256 前 12 位 |
|---|---|---|---|---|---|
| 0 | 0.053854 | 3585 (1793/1792) | 45843 | d39c37a2df64 | 509f4c63a841 |
| 1 | 0.153452 | 3585 (1793/1792) | 45856 | 4a5e86fbb17b | 433c432d78aa |
| 2 | 0.253834 | 3585 (1793/1792) | 45861 | 1c9b79969f69 | 532264854caf |
| 3 | 0.354326 | 3585 (1793/1792) | 45866 | cf723e0a7bf8 | 9903aaab091a |
| 4 | 0.453714 | 3585 (1793/1792) | 45865 | 9f68512381b7 | add2dfa1fe94 |
| 5 | 0.553485 | 3585 (1793/1792) | 45866 | 56fee4d131e8 | 324adebd6e7e |
| 6 | 0.653528 | 3585 (1793/1792) | 45866 | 6620380a5069 | e45d5a10b3dd |
| 7 | 0.753661 | 3585 (1793/1792) | 45863 | 620eace9b337 | f3fe71fedefc |
| 8 | 0.853421 | 3585 (1793/1792) | 45860 | fec08ed622ae | adca5a26126f |
| 9 | 0.953587 | 3585 (1793/1792) | 45860 | 50e08811993f | 68df78450ba7 |

`pairwise_distinct.sh`:`RESULT: all 45 pairs distinct`,首个差异行都是第 2 行(= 周期 0:首帧 LiDAR 不同 ⇒ t_k 不同),10 个 `Log/imu.txt` 两两不同(10 个不同的 IMU 初始化窗口)。全部 10 次运行完成判据四条成立。**通过**。I11 的后半句「同种子各臂 ACTIVATE 前逐位相同」要等 P3 的臂,不在本门。

## 5 valgrind memcheck(⑤)

`gates_local.sh <tree> p2b-f-strict valgrind ws/devel_vg runs/p2b-f/vg_grass02_60s`:`valgrind --tool=memcheck --track-origins=yes --leak-check=no --num-callers=40 --error-limit=no`,`-mno-avx` 的 eval 构建(`build_sha256 = efadb7da9d6c2d5c0efeca98090985d1ba5db731a270bc294dcbfd85a5050c84`),Grass02 seed 0 前 60 s = 3585 周期(1793 LIO + 1792 VIO,与 eval 构建的 `grass02_sd0` 同窗同周期数);08:39:51→09:52:59(73 min,占着回放锁,期间没有别的回放)。`run_status.json`:`complete=1, node_exit_code=0, manifest_ok=1`;控制台 `process has finished cleanly`,无 `Process terminating`、无 Invalid free。

`ERROR SUMMARY: 90 errors from 12 contexts`。`vg_origin.py` 按 origin:**12 个上下文全部 = `operator new ← LIVMapper::LIVMapper`**(构造函数里 new 出的对象);按消费方栈(本轮给 `vg_origin.py` 加的第二张表):

| 上下文 | 类型 | 消费方栈顶 |
|---|---|---|
| 8 | Conditional jump … uninitialised | `__printf_fp_l ← printf_positional ← __vfprintf_internal ← __printf_chk ← VIOManager::processFrame ← LIVMapper::handleVIO ← processOnce` |
| 2 | Conditional jump … uninitialised | `__mpn_extract_double ← __printf_fp_l ← … ← VIOManager::processFrame ← handleVIO` |
| 1 | Conditional jump … uninitialised | `round_away ← __printf_fp_l ← … ← VIOManager::processFrame ← handleVIO` |
| 1 | Syscall param write(buf) | `write ← _IO_file_sync ← fflush ← std::ostream::flush ← VoxelMapManager::StateEstimation ← handleLIO`;被写的是 stdout 的 4096 字节缓冲(由第一次 `ROS_INFO`——vikit `camera_loader → vk::getParam`——分配),里面是上面那些 printf 排版出的字符 |

全部是**只进打印**的上游项:`VIOManager::processFrame` 的计时表 `printf("%lf", compute_jacobian_time / update_ekf_time)` 读 vio.h:114 未初始化的两个成员(`computeJacobianAndUpdateEKF` 在 `total_points == 0` 时提前返回,不会先清零它们),以及这些字符随后被 `write(2)` 到 stdout。都不进状态,登记不改(上游文件零改动规则)。`Invalid read/write/free` = 0;第一轮无保护时的 `cv::fastMalloc`(1202)、no-origin(233)、`UpdateOctoTree`(57)、`Invalid read`(13)四类 = 0;WP-f 自己的代码(`replay_main`、`outputs`、`diagnostics`、`config`、LIVMapper 清单行)在任何栈里都没有出现。**通过**。

更正第一轮的归因:第一轮把第 12 个上下文写成「vikit `getParam` 打印一个未初始化的默认值」;按本轮的消费方栈(同一方法重读第一轮 `vg_diag2_60s` 日志,结构完全相同),vikit `getParam` 只是 stdout 缓冲的**分配者**,未初始化字节的来源同样是 `LIVMapper` 构造时 new 出的对象(即上面的计时成员),结论(只进打印)不变。

附注:valgrind 构建的 `state_trace` 与 eval 构建的 `grass02_sd0` 从第 12 行(周期 10)起不同(`Log/imu.txt` 相同)——`-mno-avx` 改变向量化后的求和次序。逐位结论只在同一构建内成立,这正是 A29 规定该构建「只用于 memcheck,不用于评测数字」的原因。

## 6 manifest 并入回放统计(A40)

- 用例:`gates_local.sh <tree> p2b-f-strict manifest-check ws/devel`——`manifest_check.cpp` 链接已构建的 `liblivo_gnss_ros.so`(主机 flock、容器内 nice 19),写 16 个用例,`manifest_check.py --cases` 判定:`RESULT: 16/16 cases OK`。空、纯空白、`{ }` ⇒ 与 P2b 前的 manifest 逐字节相同;合法对象(含嵌套、转义、`\u` 与 UTF-8、`-0.0`/`1E+2`)与 63 层嵌套 ⇒ 逐成员原样并入冻结字段之后;与冻结字段同名(`seed`)、自身重名、尾随文本、尾逗号、未闭合字符串、非对象、字符串内裸换行、前导零、非法转义、`NaN`、65 层嵌套 ⇒ 一个都不并入,只多一个 `extra_json_error`(给出原因与偏移);每个输出都是键唯一的严格 JSON。
- 真实运行:`replay_run.sh` 的完成判据第 4 条对本文件的 14 次回放(§3、§4)与 valgrind 运行逐次执行 `manifest_check.py --run`:manifest 严格 JSON、17 个冻结字段在前且顺序不变、无 `extra_json_error`、`replay_stats.json` 的 25 个成员(除 `complete`、`seed`)值逐一相同(配置裁头以 `start_offset_trim_s` 出现),冻结字段 `start_offset_s` = `offset_effective_s`(±0.5 ns,前者 %.17g、后者 %.9f),`compile_defs` 带同一 `build_sha256`。全部 `manifest_ok = 1`。

## 7 ctest(`--core`,19 项)

| profile | 命令 | 结果 |
|---|---|---|
| 默认 | `remote_build.sh <tree> p2b-f --core`;`remote_test.sh p2b-f -V` | 17/19;用例 140:OK 127、SKIPPED 11、FAILED 2 |
| eval | `remote_build.sh <tree> p2b-f-strict --core --strict --no-sync`;`remote_test.sh p2b-f-strict -V` | 17/19;用例 140:OK 126、SKIPPED 11、FAILED 3 |

`test_wpf_serialisation` 两个 profile 都是 9/9(原 6 例 + A40 的 `ValidateConfigRejectsDriftWithoutReference`、`ValidateConfigLeapStrings`、`ValidateConfigAgreesWithTimeMap`)。FAIL 与维护者在 65b6752 记录的集合逐项相同(INTERFACE_CHANGES「维护者」小节):① U5 `PatchedPntposEqualsUnpatchedGoldenFixture`(WP-a 金标准文件)、② T1 `DevSetIonoParamsEqualDayGpsKlobuchar`(WP-e,A38)、③ 仅 eval:U5 `PatchedPntposEqualsUnpatchedGolden`(WP-a,合成金标准随 profile);SKIPPED 11 例同其记录(U4 SP3、U13×3、U15×2、U17×3、U2 `Md5WithConstants`、T1 `RepeatedCodeIsCounted`)。用例总数 137→140 只多了本包 3 例。

## 8 tag `ref-R`

`ref-R` 是 annotated tag(本地,未 push),打在分支 `p2b-f` 的 HEAD = **本文件所在的提交**。它与被门禁的源码提交 `35ee6c5` 只差文档:`git diff --stat 35ee6c5 ref-R` 只列 `docs/dev/REF_R_GATES.md`、`docs/dev/M3DGR_BASELINE_DIFF.md`、`docs/dev/ref_r_gates/{README.md, vg_origin.py}`,没有一个参与构建,所以 §1–§7 的二进制(eval `build_sha256 f453a0fb…`,valgrind 变体 `efadb7da…`)就是 ref-R 的二进制。

附带核对(G-零通道的 `git diff` 前提,DESIGN §13、OWNERSHIP §1):相对 0d2c034,`common_lib.h`、`voxel_map.*`、`vio.*`、`frame.*`、`visual_point.*`、`preprocess.*`、`main.cpp` 之中只有 `src/vio.cpp` 有改动,且恰为 A27 的 5 行(2 行注释 + 判据块);`include/IMU_Processing.h` 只有 :60/:80/:81 三处类内 `= 0.0`,`src/IMU_Processing.cpp` 零改动;`LIVMapper.*` 只含 WP-f 清单行。tag 之后 `vio.*` 零改动(OWNERSHIP §1)。

复现:`git checkout ref-R`,按 `docs/dev/ref_r_gates/README.md` 的顺序跑;同机同镜像下 §3/§4 的哈希应逐字节重现(逐位结论不跨主机、不跨镜像,DESIGN §13)。

## 9 历史(第一轮,摘要;全文见提交 ebc12dd 的本文件)

- **根因**:0d2c034 `VIOManager::updateState`(vio.cpp:1520–1640)在金字塔粗层以原始指针读 level-0 像素,模板覆盖 `[v−(half+1)·scale, v+(P−half+1)·scale]`(P=8、scale 最大 32 ⇒ ±160 px),而点只按 80 px border 检查过 ⇒ 边缘点越界读堆。同一二进制、同一 bag、同一种子的两次 Dark01 整包回放在第 651 行(周期 649,VIO)分岔,之后全部不同;分岔位置固定、数值随机。valgrind(Dark01 60 s)在无保护源码上:`cv::fastMalloc` origin 1202 个上下文、no-origin 233、`UpdateOctoTree` 57、`Invalid read` 13。
- **诊断树**(HEAD 源码 + 只 `continue` 的保护,`make_diag_tree.sh`,现已退役):Dark01 整包两次 12357 周期逐位相同;valgrind 上述四类全部归零,剩 12 个上下文全是只进打印的上游项(11 个 = `VIOManager` 计时 `printf` 读 vio.h:114 未初始化的 `compute_jacobian_time/update_ekf_time`,1 个 = 这些字符写出 stdout 的 `write(2)`;第一轮把后者误记为 vikit `getParam` 打印未初始化默认值,见 §5 的更正)。
- **种子**:旧语义 0.05 s 步长在 Dark01 上成对塌缩到同一首帧 LiDAR(审查 M1)⇒ A30 改为「记录序第 k 帧 LiDAR」。
- 维护者据此裁定 A27(保护入 R、U=R)、A28(STRICT 下 `-include omp.h`)、A29(`-mno-avx` 的 valgrind 构建)、A30;本文件 §1–§8 是在含这些裁定的 HEAD 上的重跑。维护者版本的保护与诊断版的唯一差别是越界点先把 z/H_sub 的对应行清零(否则上一迭代写过的行会留在本迭代的 HᵀH、Hᵀz 里);因此 ③④⑤ 以本次在 HEAD 上的结果为准。
- 上游 `~LIVMapper` 在 `-march=native` 构建上退出时 `free()` 崩溃(Eigen 64 字节对齐 vs 系统 PCL 的 malloc):回放驱动不析构 LIVMapper,退出码可信;已登记,不属 R。
