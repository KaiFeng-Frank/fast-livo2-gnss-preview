> Historical research record (through 2026-10-03). Paths are illustrative; archived runs are not bundled. For current status and executable entry points see [the repository README](../../../README.md). Current release docs/LICENSING.md supersedes historical license notes. This record does not authorize new experiments or assert current release validation.

# M3DGR 标定链推导(tools/m3dgr_calib.py)

来源:`M3DGR calibration.md (github.com/sjtuyinjie/M3DGR main; archived copy /work/fastlivo2_gnss/audit/calib/calibration.md, sha256 39720b5c71a65b4c879769bd793d9c2f299deb61dbe4c0d182807773d601c7a0)`(M3DGR calibration.md)。生成日期 2026-09-24。

## 约定

- calibration.md 每行「source → target」给 T、R,含义 `p_target = R·p_source + T`(文件 §1 开头声明)。
- FAST-LIVO2:`extrinsic_T/R` = T_{I←L}(`p^I = R p^L + T`,LIVMapper.cpp:104-105),`Rcl/Pcl` = T_{C←L}(vio.cpp:35-39)。
- 杆臂 `l^I` = 天线相位中心在 IMU 系的坐标(DESIGN §1 A 行)。

## 推导

1. 「GNSS antenna → avia」行只有 T 没有 R:把天线这个**点**(source 原点)映到 avia 系,`p_ant^avia = T = [-0.358, 0.086, 0.429]` m。
2. 「avia → avia_imu」:R = I,T = [0.04165, 0.02326, -0.0284] ⇒ `l^I(Avia) = p_ant^avia + T = [-0.31635, 0.10926, 0.4006]` m,与 DESIGN §14 / A15 的 `[-0.31635, 0.10926, 0.4006]` **逐位相同**(差 5.6e-17 m)。
3. MID-360 链:`p_ant^mid360 = R_{mid360←avia}·p_ant^avia + T_{mid360←avia} = [-0.2735, 0.024087, -0.25525]`,再经「mid360 → mid360_imu」(R = I,T = [-0.011, -0.02329, 0.04412])得 `l^I(MID-360) = [-0.2845, 0.00079668, -0.21113]` m(DESIGN §14 推断 ≈[−0.284, 0.001, −0.211],3 位小数内一致)。
4. 旁证:MID-360 原点在 avia 系 `[0.00764, 0.0484, 0.503]` m(比 Avia 高约 0.50 m),天线比 Avia 高 0.429 m;MID-360 相对 avia 转角约 31.8°,所以 MID-360 系里天线 z 为负不代表在下方。

## 方向论证

- 反向读法(把 T 当作 avia 原点在天线系)会给 `p_ant^avia = [0.358, -0.086, -0.429]`,即天线在前视雷达**下方 0.43 m**。README 图 1:前视雷达(1)装在底板上、离地约 0.3 m,三根天线(5)在顶板上方 15 cm,顶板比底板高 45 cm ⇒ 正向读法(天线高 0.43 m)与结构图一致,反向读法会把天线放到地面以下,排除。
- 横向 +0.086 m(avia 系 y 向左):README 图 1(b) 顶板三根天线中两根在一侧(C-RTK2HP 双天线)、一根在另一侧(推断为 9Ps);用 y>0 的那根与 0.086 m 对应。**RTK 真值用的 2HP 天线不是这一根**,其杆臂未知(TEST_PLAN §4「2HP 杆臂未知记为地板」)。
- 数据证据(§12.6-7:转弯段 l 与 −l 两假设的多普勒残差 RMS 差 ≈ ω·|l_h|)需要 ref-R 回放轨迹,且属于开发集上的判定(A2):本轮 **SKIP(blocked: ref-R merge + dev-set fixture)**。

## 自洽检查

| 检查 | 结果 | 数值 |
|---|---|---|
| orthonormal camera→camera_imu | PASS | max abs(RRᵀ−I) 9.5e-09, det 1.000000 |
| orthonormal camera_imu→avia | PASS | max abs(RRᵀ−I) 7.1e-07, det 1.000000 |
| orthonormal avia→camera_imu | PASS | max abs(RRᵀ−I) 7.1e-07, det 1.000000 |
| orthonormal camera→avia | PASS | max abs(RRᵀ−I) 1.3e-07, det 1.000000 |
| orthonormal avia→camera | PASS | max abs(RRᵀ−I) 7.3e-07, det 1.000000 |
| orthonormal camera→mid360 | PASS | max abs(RRᵀ−I) 7.6e-08, det 1.000000 |
| orthonormal mid360→camera | PASS | max abs(RRᵀ−I) 8.8e-07, det 1.000000 |
| orthonormal camera_imu→mid360 | PASS | max abs(RRᵀ−I) 5.8e-07, det 1.000000 |
| orthonormal mid360→camera_imu | PASS | max abs(RRᵀ−I) 4.7e-07, det 1.000000 |
| orthonormal avia→avia_imu | PASS | max abs(RRᵀ−I) 0.0e+00, det 1.000000 |
| orthonormal mid360→mid360_imu | PASS | max abs(RRᵀ−I) 0.0e+00, det 1.000000 |
| orthonormal avia→mid360 | PASS | max abs(RRᵀ−I) 1.2e-06, det 0.999999 |
| inverse pair avia↔camera_imu | PASS | Δrot 0.0370°, ΔT 5.03e-07 m |
| inverse pair avia↔camera | PASS | Δrot 0.0209°, ΔT 6.76e-08 m |
| inverse pair camera↔mid360 | PASS | Δrot 0.0000°, ΔT 2.27e-07 m |
| inverse pair camera_imu↔mid360 | PASS | Δrot 0.0000°, ΔT 1.38e-07 m |
| chain camera→camera_imu→avia = camera→avia | PASS | Δrot 0.033°, ΔT 0.0000 m |
| chain avia→camera→mid360 = avia→mid360 | PASS | Δrot 0.041°, ΔT 0.0000 m |
| chain camera_imu→camera→mid360 = camera_imu→mid360 | PASS | Δrot 0.000°, ΔT 0.0000 m |
| chain camera→camera_imu→mid360 = camera→mid360 | PASS | Δrot 0.000°, ΔT 0.0000 m |
| A15: l^I (Avia) = DESIGN §14 [-0.31635, 0.10926, 0.4006] | PASS | derived [-0.31635  0.10926  0.4006 ], max |Δ| 5.6e-17 m |
| MID-360 chain vs DESIGN §14 estimate [-0.284, 0.001, -0.211] (3 decimals) | PASS | derived [-0.284502  0.000797 -0.211127] |
| antenna above the front LiDAR (plausibility of the row direction, README Fig. 1: antennas on the top plate) | PASS | antenna z in avia frame +0.429 m (reverse reading would be -0.429 m) |

## 其它输出

- Avia:`extrinsic_T = [0.04165, 0.02326, -0.0284]`、`extrinsic_R = I`(= calibration §1.6,与上游 avia.yaml 相同);`Rcl/Pcl` 取「avia → camera」行:Pcl = [0.0178448, 0.0894589, 0.161556]。
- D435i:PINHOLE 640x480,fx 607.79773 fy 607.83527 cx 328.79773 cy 245.53322,畸变全 0。
- IMU 噪声(Kalibr avg-axis):Avia acc_n 0.02085 gyr_n 0.003219;MID-360 acc_n 0.01147 gyr_n 0.001325(A15 默认值取 Avia)。
