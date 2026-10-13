> Historical research record (through 2026-10-03). Paths are illustrative; archived runs are not bundled. For current status and executable entry points see [the repository README](../../README.md). Current release docs/LICENSING.md supersedes historical license notes. This record does not authorize new experiments or assert current release validation.

# A81/A82 开发冻结入口

该工具生成与核对冻结文件,不启动实验。真实尺度、变体和阈值只能来自完整开发运行;本说明不填写实验数值。
2026-10-03 本轮唯一完整 recheck 的 SmartPNT Doppler `1.2985784051776035 > 1.25`,D4 失败。
所以本轮没有调用下面的 prepare/promote,也没有 final 评测臂;终态证据见
[CLOSEOUT](../RESULTS.md)。以下仅记录已验证的工具契约,不是续跑授权。
Python 3 + PyYAML 即可;`a80_freeze.py` 与 `a80_noise_readout.py` 放在同一目录。
在保留数据的服务器上执行,`--root /data2/fl2g`,`--repo` 指向该次实验的源码快照。
大型 CSV、轨迹、二进制和实验包留在服务器,仅小代码、JSON 与日志摘要可传本机。

## 输入

先以 `a80_noise_readout.py` 收集两个 SmartPNT 开发序列与 MARS 完整种子 0 的模型诊断运行。
输出 proposal 后仅加其两个方差尺度,完整复核一次,用 `--stage recheck --previous proposal.json` 出复核 JSON。
只能使用 `LIVGO-literal-diag` 或对应数据集的 `LIVGO-literal-recheck-{smartpnt,mars}-diag`;
`time_calib` 不引入文件或在线估计,A82 一次性初始化与诊断开关必须保留。
三组都是真实完整开发、两种统计的原 D4 `[0.8,1.25]` 全过,才能 prepare;固定论文噪声不绕过这个先后顺序。
manifest 的 `evo.seq_name` 是 runner 用于上游输出文件的 `<sequence>_<arm>_s0`,不是裸数据集名;
工具按当前 manifest 的 arm/seed 精准匹配并解出开发序列,不会模糊匹配其他臂或评测序列。

选型 `selection.json` 包含以下字段:

- `ate_files`:官方 `tools/eval/eval_ate.py` 的 masked `ate.json` 引用列表,每项 `{path, sha256}`。
  SmartPNT 使用 `smartpnt_mini`,MARS 使用 `mars_avia` LiDAR profile;按 sequence 共有区间评分,U 臂为 U。
  e1c 的登记杆臂、0.05 s 关联、SE(3) 无尺度对齐及 truth mask/timebase/GT-clean 文件 SHA 都须一致。
- `candidate_arms`:以 `smartpnt_mini`、`mars_lvig` 为键,每项 `{model: 模型复核臂名, flat: 固定论文诊断臂名}`。
  U 与两候选须同构建、同宿主配置、完整种子 0;模型用复核后的尺度,两候选收集时阈值均为 0。
- `binary_files`:覆盖每个 run manifest 的全部 `build_files`,每项 `{path, sha256, build_path}`。
  `path` 是服务器上实际可读文件,`build_path` 是 manifest 内容器记录的路径;两者相同时可省略 `build_path`。
  例如 manifest `/r/wt/a80c/...` 可对应服务器 `/work/fastlivo2_gnss/wt/a80c/...`。
  不仅固定 executable 和 GNSS 库,全部链接库均计算 SHA256,按 C++ 原清单规则复核组合 `build_sha256`。

所有引用是该服务器上仍存在的原始文件;工具先核 SHA256,拒绝变化的证据。
GT 入口固定为 `<root>/audit/<seq>/gt/{gt_clean.tum,gt_timebase.yaml,trust_mask.csv}`。
选择按三条开发序列 `ATE_variant / ATE_U` 的中位数,未激活序列计比例 1;不按数据集分别挑噪声模型。

## 两个阶段

```sh
python3 tools/calib/a80_freeze.py prepare \
  --repo "$SOURCE_SNAPSHOT" --root /data2/fl2g \
  --proposal "$EVIDENCE_DIR/proposal.json" --recheck "$EVIDENCE_DIR/recheck.json" \
  --selection "$EVIDENCE_DIR/selection.json" --out-dir "$EVIDENCE_DIR/frozen-dev"
```

默认只写一个新的输出目录,含 overlay、`arms_append.json` 与 `freeze.json`,不改源码注册表。
核对结果后同命令加 `--install`（使用新的输出目录）才追加两条冻结开发臂。
遇已有异内容同名配置或注册项会拒绝;原四候选不解禁、不覆写。
模型仅添加两个尺度及 `lambda_th`,固定论文噪声仅添加 `lambda_th`。
阈值来自已选择的完整开发 `livgo_hessian.csv[.gz]`,按数据集汇总满窗且单次 ACTIVATE 事件 `t_k+30s` 之后的
有限非负 `lambda_bar`,计算 Hyndman-Fan type 7 第 5 百分位。
与首 `traj_N` 的延迟仍核原仪器区间 `[-1e-6,1] s`,不会改写时钟或调大门。

用 `LIVGO-literal-frozen-smartpnt` 与 `LIVGO-literal-frozen-mars` 运行三条完整开发各一次种子 0。
`development.json` 只含这次官方 masked 报告的 `ate_files:[{path,sha256}]`,随后:

```sh
python3 tools/calib/a80_freeze.py promote \
  --repo "$SOURCE_SNAPSHOT" --root /data2/fl2g \
  --prepared "$EVIDENCE_DIR/frozen-dev/freeze.json" \
  --dev-evidence "$EVIDENCE_DIR/development.json" --out-dir "$EVIDENCE_DIR/final-eval"
```

三条种子 0 必须整段 complete,路程比 `[0.95,1.05]`,全部配置、二进制与冻结来源 hash 不变。
通过才生成独立 `LIVGO-literal-final`,SmartPNT overlay 与已验证的冻结开发文件逐字节相同;
该新臂 `calibration:false,time_calib:false`,原四候选仍 `calibration:true`。
需要注册时显式 `--install`,之后由实验负责人按原计划启动一次最终评测。
工具不会执行评测或基于评测结果再调参。

## 验证与归档

`tools/calib/tests/test_a80_freeze.py` 用独立临时合成夹具验证阶段转换和拒绝边界。
这些夹具不是实验数据或参数提案,不写入正式证据目录。
真实运行的报告、原始 CSV/轨迹、参数/二进制来源与 `freeze.json` 一同在服务器封存,直接上传对应 private GitHub Release。
最终源码另绑定交付 publication SHA;回放组合 build SHA 与运行源码 `1b8aa38` 标签分别登记。
