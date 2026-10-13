> Historical research record (through 2026-10-03). Paths are illustrative; archived runs are not bundled. For current status and executable entry points see [the repository README](../../../README.md). Current release docs/LICENSING.md supersedes historical license notes. This record does not authorize new experiments or assert current release validation.

# ref-R 门禁脚本(WP-f)

`docs/dev/REF_R_GATES.md` 记录的每一个数字都由这里的脚本产生;放进仓库是为了让门禁**可复现**(审查 M2)。远端侧脚本经 `tools/dev/remote_build.sh` 的 rsync 到达 `wt/<名字>/ws/src/FAST-LIVO2/docs/dev/ref_r_gates/`,在远端主机上以 `bash` 直接执行;本机侧只有 `gates_local.sh`。

归属:本目录、`REF_R_GATES.md`、`M3DGR_BASELINE_DIFF.md` 归 WP-f(AMENDMENTS A40,OWNERSHIP §1c)。

| 脚本 | 运行处 | 作用 |
|---|---|---|
| `gates_local.sh` | 本机 | 各门禁的一键入口(build-default / compile-gate / build-eval / vg-build / manifest-check / seed-gate / valgrind / vg-report);把本机 `git rev-parse --short HEAD` 与 dirty 标志写成远端 `BUILD_ID` |
| `build_eval.sh` | 远端 | `remote_build.sh --strict` 的 catkin 步(A28 之后 `-include omp.h` 由顶层 CMake 给上游 TU,本脚本不再加)+ 额外旗标,产物进独立的 build/devel 目录;现在只用于 valgrind 的 `-mno-avx` 变体(A29,WP-g 的 `remote_build.sh --vg` 合并后退役)。纪律同 remote_build.sh:主机 flock、容器内 nice 19、-j2、-u 1000:1000 |
| `replay_run.sh` | 远端 | 一次回放:独立容器、主机级回放锁 `flock $ROOT/.replay.lock`(同一时刻只有一个回放,A7)、私有 roscore 端口、`OMP_NUM_THREADS=1`、Log/ 与 ROS_LOG_DIR 绑定到运行目录、`FL2G_HOST`、BUILD_ID → manifest;bag 的 sha256 取 `<bag>.sha256`,没有则 `sha256sum` 一次并缓存到 `runs/.bag_sha256/`;**完成判据** = `replay_stats.json` 的 `"complete": true` + 控制台 `fastlivo_replay: done` + 节点退出码 0 + `manifest_check.py --run` 通过(manifest 是严格 JSON、键唯一、并入了全部回放统计,A40),写 `run_status.json` |
| `seed_gate.sh` | 远端 | ③ 与 ④ 链:seed 0 两次(整包或给定时长,逐位)+ seed 0..n−1 各前 N s(两两不同),顺序执行;n=0 只跑逐位对(评测集只比哈希时用) |
| `hash_runs.sh` / `pairwise_distinct.sh` | 远端 | 只输出 sha256、行数、首个差异行号(A2:评测集不显示任何轨迹数值);逐位对比较 7 个产物(state_trace、traj_W、filter_cov、Log/{mat_pre,mat_out,imu}、Log/result)并核对两次运行的 build_sha256 / config_sha256 相同 |
| `manifest_check.cpp` + `manifest_check.py` | 远端 | `OutputWriter::writeManifest` 的 `extra_json` 合并(A40):C++ 程序链接已构建的 `liblivo_gnss_ros.so` 写 16 个用例,Python 判定(严格 JSON、键唯一;空/空白/`{}` ⇒ 与 P2b 前逐字节相同;合法对象逐成员原样并入;畸形、与 manifest 键冲突、自身重复键、嵌套 >64 ⇒ 不并入且只有一个 `extra_json_error` 成员);`--run` 模式供 replay_run.sh 逐次核对真实 manifest |
| `vg_origin.py` | 远端 | valgrind 日志按 origin 归类(审查 M3),给出每类的计数与首次出现序号;再按「错误类型 + 自身栈顶 8 帧」分组列出消费方(P2b),逐一核对剩余上下文是否都止于打印路径 |

**已退役**(P2b,A27):`make_diag_tree.sh` 与 `diag_guard.py`——它们给 `vio.cpp` 的临时副本打越界读保护来证明 ③④ 的根因;维护者已把保护(清零 z/H 行的版本)提交进 `src/vio.cpp`(65b6752),再套会插入第二份。第一轮的诊断证据留在 git 历史(ebc12dd)与 `REF_R_GATES.md` 的历史节。

## 复现顺序(HEAD;开发集 Grass02 全程可看;Dark01 属评测集,只比哈希,A2)

```bash
G=docs/dev/ref_r_gates/gates_local.sh; L=/workspace/fastlivo2_gnss_wt/p2b-f
$G $L p2b-f        build-default                     # ① 默认 profile(clean)
$G $L p2b-f        compile-gate                      # ② 编译命令门(含 vikit 补丁)
$G $L p2b-f-strict build-eval                        # ① eval profile(remote_build.sh --strict,clean)
$G $L p2b-f-strict vg-build                          # valgrind 用 -mno-avx 变体 → ws/devel_vg
$G $L p2b-f-strict manifest-check ws/devel           # extra_json 合并用例(A40)
$G $L p2b-f-strict seed-gate ws/devel p2b-f/grass02                                   # ③ Grass02 全程两次 + ④ 10 种子各 60 s
$G $L p2b-f-strict seed-gate ws/devel p2b-f/dark01_60s data/m3dgr/bags/Dark01.bag 0 60 60   # ③ Dark01 前 60 s 两次(只比哈希)
$G $L p2b-f-strict valgrind ws/devel_vg runs/p2b-f/vg_grass02_60s                     # ⑤(后台)
$G $L p2b-f-strict vg-report runs/p2b-f/vg_grass02_60s
```
