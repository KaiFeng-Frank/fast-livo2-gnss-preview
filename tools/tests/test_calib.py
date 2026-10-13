# SPDX-License-Identifier: BSD-3-Clause
"""tools/m3dgr_calib.py: parsing of the calibration.md table format, the A15 lever arm (DESIGN §14 value to the
last digit), the MID-360 chain, and that the consistency checks catch a wrong row (they are not tautological).
The table rows below are the numeric facts of M3DGR calibration.md §1 (MIT-licensed dataset documentation)."""
import contextlib
import io
import os
import sys
import tempfile
import unittest

import numpy as np

import _path  # noqa: F401
import m3dgr_calib as mc

sys.path.insert(0, os.path.join(_path.TOOLS, "calib"))
import host_gt_lever as hgl  # noqa: E402

TABLES = """
| Direction | T (m) | R |
|-----------|-------|---|
| `camera` → `camera_imu` | `[0.03668114, -0.00477653, 0.0316039]` | `[0.99957087, 0.00215313, 0.02921355, -0.00192891, 0.99996848, -0.00770122, -0.02922921, 0.00764156, 0.99954353]` |
| `camera_imu` → `avia` | `[-0.19501, 0.0471106, 0.0870882]` | `[0.0502661, -0.0116117, 0.998668, -0.99829, 0.0292847, 0.0505876, -0.0298331, -0.999504, -0.0101199]` |
| `avia` → `camera_imu` | `[0.0594305, 0.083401, 0.193248]` | `[0.0502661, -0.99829, -0.0298331, -0.0116117, 0.0292847, -0.999504, 0.998668, 0.0505876, -0.0101199]` |
| `camera` → `avia` | `[-0.16154853, 0.01195102, 0.09044823]` | `[0.0210767, -0.0038717, 0.9997703, -0.9993969, 0.0275209, 0.0211754, -0.0275965, -0.9996137, -0.0032894]` |
| `avia` → `camera` | `[0.0178448, 0.0894589, 0.161556]` | `[0.0210767, -0.999397, -0.0275965, -0.0038717, 0.0275209, -0.999614, 0.99977, 0.0211754, -0.0032894]` |
| `camera` → `mid360` | `[0.07433267, -0.03806468, -0.43920869]` | `[0.0722207, 0.5210047, 0.8504930, -0.9973866, 0.0394820, 0.0605080, -0.0020542, -0.8526402, 0.5224945]` |
| `mid360` → `camera` | `[-0.0442358, -0.411712, 0.168568]` | `[0.0722207, -0.997387, -0.0020542, 0.521005, 0.039482, -0.85264, 0.850493, 0.060508, 0.522494]` |
| `avia` → `avia_imu` | `[0.04165, 0.02326, -0.0284]` | Identity |
| `mid360` → `mid360_imu` | `[-0.011, -0.02329, 0.04412]` | Identity |
| GNSS antenna → `avia` | `[-0.358, 0.086, 0.429]` | — |
| `avia` → `mid360` | `[0.259632, -0.0425554, -0.431109]` | `[0.849802, -0.0398291, -0.525594, 0.0393196, 0.999153, -0.0121414, 0.525632, -0.0103484, 0.850649]` |

projection_parameters:
  fx: 607.79772949218
  fy: 607.83526613281
  cx: 328.79772949218
  cy: 245.53321838378

```yaml
type: IMU
name: avia_livox
Gyr:
   avg-axis:
      gyr_n: 3.2194626705221046e-03
      gyr_w: 7.1383392426109448e-05
Acc:
   avg-axis:
      acc_n: 2.0846225165441278e-02
      acc_w: 3.4099978362929511e-04
```

```yaml
type: IMU
name: mid360_livox
Gyr:
   avg-axis:
      gyr_n: 1.3254540721802960e-03
      gyr_w: 5.2026292534341382e-05
Acc:
   avg-axis:
      acc_n: 1.1472382386578356e-02
      acc_w: 3.3397564584716999e-04
```
"""


class Calib(unittest.TestCase):
    def test_a15_lever_arm_and_mid360_chain(self):
        rows, intr, imu = mc.parse_calibration(TABLES)
        self.assertIsNone(rows[("GNSS antenna", "avia")][0])
        res = mc.derive(rows)
        np.testing.assert_allclose(res["lever"]["l_avia_imu"], [-0.31635, 0.10926, 0.4006], atol=1e-12)
        np.testing.assert_allclose(res["lever"]["l_mid360_imu"], [-0.284502, 0.000797, -0.211127], atol=5e-6)
        failed = [c for c in res["checks"] if not c[1]]
        self.assertEqual(failed, [], failed)
        self.assertAlmostEqual(intr["fx"], 607.79772949218)
        np.testing.assert_allclose(res["rcl_pcl_avia"][1], [0.0178448, 0.0894589, 0.161556])

    def test_checks_catch_a_wrong_row(self):
        bad = TABLES.replace("`[0.0178448, 0.0894589, 0.161556]`", "`[-0.0178448, 0.0894589, 0.161556]`")
        res = mc.derive(mc.parse_calibration(bad)[0])
        names = [c[0] for c in res["checks"] if not c[1]]
        self.assertTrue(any("inverse pair avia↔camera" in n for n in names), names)
        bad2 = TABLES.replace("0.849802, -0.0398291", "0.849802, 0.0398291")      # corrupt avia->mid360 rotation
        res2 = mc.derive(mc.parse_calibration(bad2)[0])
        self.assertTrue(any(not c[1] for c in res2["checks"] if c[0].startswith("orthonormal avia→mid360") or "chain avia" in c[0]))

    def test_cli_writes_fragment_and_markdown(self):
        with tempfile.TemporaryDirectory() as d:
            src = os.path.join(d, "calibration.md")
            with open(src, "w") as f:
                f.write(TABLES)
            y, md = os.path.join(d, "frag.yaml"), os.path.join(d, "calib.md")
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(mc.main([src, y, "--md", md]), 0)
            with open(y) as f:
                text = f.read()
            with open(md) as f:
                mdt = f.read()
            self.assertIn("lever_arm_xyz: [-0.358, 0.086, 0.429]", text)
            self.assertIn("extrinsic_T: [0.04165, 0.02326, -0.0284]", text)
            self.assertIn("[-0.31635, 0.10926, 0.4006]", mdt)


LEVER_REF = [-0.176, -0.016, 0.401]          # A58/A63: GT antenna (C-RTK 2HP) in the Avia IMU frame
# A77: the same antenna seen from the MID-360 IMU, level-frame horizontal components fitted on the MID-360 U runs of Grass01 and
# Grass02 (tools/calib/host_gt_lever.py fit; seed median, then median over the two sequences, rounded to 1 mm); the vertical
# component is the calibration chain's (not a fit)
A77_LEVEL_XY = (-0.049, -0.152)
A77_GT_LEVER_MID360_IMU = [-0.01999096558, -0.1534376595, -0.04942328067]
A77_9PS_LEVER_MID360_LIDAR = [-0.1332496693, -0.01051226054, -0.1686119725]
# A78 (implementation record, part 1): the registration random walk of the MID-360 host = the 75 % quantile of the A58/A70 `frame`
# program over the 15 MID-360 U runs a76-mid-dev-{g1,g2,zrr}u0..4 (--lever-fixed = lidars.json mid360 gt_lever_imu, --alpha vertical);
# result file runs/a78-score/mid360/frame_mid360_dev3x5_vertical.json on the remote host
A78_FRAME_Q_MID360 = {"q_tilt_per_m": 0.0, "q_yaw_per_m": 1.4836455387643271e-07, "q_yaw_per_rad": 5.0682795400455866e-07,
                      "q_th_per_m": 9.1154744128326817e-05, "q_tv_per_m": 0.00019090891586508499, "q_time": 0.0}
REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


class Mid360Host(unittest.TestCase):
    """A76: the MID-360 host configuration, its GT-antenna lever and the profile the scoring reads, all re-derived from the
    calibration table rows (the numbers are reproducible, not typed twice)."""

    def derive(self):
        rows, intr, imu = mc.parse_calibration(TABLES)
        return rows, intr, imu, mc.derive(rows, LEVER_REF)

    def test_gt_lever_chain_and_its_self_checks(self):
        rows, _, _, res = self.derive()
        failed = [c for c in res["checks"] if not c[1]]
        self.assertEqual(failed, [], failed)
        self.assertGreaterEqual(sum(1 for c in res["checks"] if c[0].startswith("GT lever")), 4)   # + camera_imu path when its row is listed
        lv = res["lever"]
        np.testing.assert_allclose(lv["gt_avia_imu"], LEVER_REF)
        np.testing.assert_allclose(lv["gt_avia"], [-0.21765, -0.03926, 0.4294], atol=1e-12)           # avia_imu -> avia: p - T
        np.testing.assert_allclose(lv["gt_mid360"], [-0.1494537784, -0.0955535749, -0.179837846], atol=1e-9)
        np.testing.assert_allclose(lv["gt_mid360_imu"], [-0.1604537784, -0.1188435749, -0.135717846], atol=1e-9)
        # the rigid body: the vector from the MID-360 IMU to the antenna, in the Avia axes, plus that IMU's position = the antenna
        R = lv["R_avia_imu_from_mid360_imu"]
        np.testing.assert_allclose(lv["mid360_imu_origin_in_avia_imu"] + R @ lv["gt_mid360_imu"], LEVER_REF, atol=1e-12)
        np.testing.assert_allclose(lv["mid360_imu_origin_in_avia_imu"], [0.0363644665, 0.0949477222, 0.4306717794], atol=1e-9)
        self.assertAlmostEqual(float(np.hypot(*lv["gt_lever_in_avia_axes"][:2])), 0.2396, places=4)
        # physical check of the rotation's direction (independent of the table's closure): the static accelerometer of
        # Outdoor01 (bag start, g units for the MID-360, m/s^2 for the Avia) seen through avia -> mid360 (p' = R p) lands
        # on the MID-360 reading within the mounting scatter, the transposed reading is 60 degrees away
        g_avia = np.array([0.3877, -0.1972, 9.7489])
        g_mid = np.array([-0.4539, -0.0301, 0.8882])
        R_am = rows[("avia", "mid360")][0]
        u = lambda x: x / np.linalg.norm(x)          # noqa: E731
        ang = lambda a, b: np.degrees(np.arccos(np.clip(u(a) @ u(b), -1, 1)))      # noqa: E731
        self.assertLess(ang(R_am @ g_avia, g_mid), 3.0)
        self.assertGreater(ang(R_am.T @ g_avia, g_mid), 50.0)

    def test_without_gt_lever_nothing_changes(self):
        rows, _ = mc.parse_calibration(TABLES)[0], None
        res = mc.derive(rows)
        self.assertNotIn("gt_avia_imu", res["lever"])
        with_gt = mc.derive(rows, LEVER_REF)["checks"]
        self.assertEqual([c for c in with_gt if not c[0].startswith("GT lever")], res["checks"])

    def test_cli_prints_and_writes_the_gt_lever(self):
        with tempfile.TemporaryDirectory() as d:
            src = os.path.join(d, "calibration.md")
            with open(src, "w") as f:
                f.write(TABLES)
            y, md = os.path.join(d, "m.yaml"), os.path.join(d, "m.md")
            buf = io.StringIO()
            with contextlib.redirect_stdout(buf):
                self.assertEqual(mc.main([src, y, "--lidar", "mid360", "--gt-lever", "-0.176", "-0.016", "0.401", "--md", md]), 0)
            self.assertIn("GT antenna (C-RTK 2HP) in the mid360 IMU frame  = [-0.1604537784, -0.1188435749, -0.135717846]", buf.getvalue())
            with open(y) as f:
                text = f.read()
            self.assertIn("lever_arm_xyz: [-0.2735022446, 0.0240866806, -0.2552467974]", text)
            self.assertIn("# GT antenna (C-RTK 2HP) in the mid360 IMU frame", text)
            self.assertIn("acc_n: 0.0114724", text)
            with open(md) as f:
                self.assertIn("真值天线(C-RTK 2HP)杆臂的换系", f.read())

    def test_lidars_json_is_the_a77_derivation(self):
        """A77: the MID-360 GT lever of lidars.json is the development-set level-frame fit seen through the calibration chain's
        ROTATION (and its vertical), not the chain's translation. The rotation and the Avia entry are still the chain's."""
        rows, _, _, res = self.derive()
        lid = ec_lidars()
        mid = lid["mid360"]
        a77 = hgl.derive_a77(rows, LEVER_REF, A77_LEVEL_XY)
        np.testing.assert_allclose(mid["gt_lever_imu"], a77["gt_lever_mid360_imu"], atol=1e-9)
        np.testing.assert_allclose(mid["gt_lever_imu"], A77_GT_LEVER_MID360_IMU, atol=1e-9)
        R_am = rows[("avia", "mid360")][0]
        np.testing.assert_allclose(np.asarray(mid["R_level_from_imu"]).reshape(3, 3), R_am.T, atol=1e-12)
        # in the level frame: the fitted components, the chain's vertical (antenna 0.401 over the Avia IMU minus the MID-360
        # IMU's 0.4307 over it)
        v = np.asarray(mid["R_level_from_imu"]).reshape(3, 3) @ np.asarray(mid["gt_lever_imu"])
        origin_z = float(res["lever"]["mid360_imu_origin_in_avia_imu"][2])
        np.testing.assert_allclose(v, [A77_LEVEL_XY[0], A77_LEVEL_XY[1], LEVER_REF[2] - origin_z], atol=2e-6)
        # not the chain's translation: the horizontal vector differs from the chain's by the fit's correction (0.168 m)
        chain_v = R_am.T @ res["lever"]["gt_mid360_imu"]
        self.assertAlmostEqual(float(np.hypot(*(chain_v - v)[:2])), 0.1682, places=3)
        self.assertEqual(lid["avia"]["gt_lever_imu"], LEVER_REF)
        self.assertEqual(lid["avia"]["R_level_from_imu"], [1, 0, 0, 0, 1, 0, 0, 0, 1])
        self.assertIn("A77", mid["gt_lever_source"])
        for name in lid:
            self.assertTrue(os.path.isfile(os.path.join(REPO, lid[name]["config"])))
            self.assertTrue(os.path.isfile(os.path.join(REPO, "launch", lid[name]["replay_launch"])))

    def test_a77_with_the_chains_own_level_components_is_the_chain(self):
        """The construction is not a second chain: fed the chain's own horizontal components it returns the chain's GT lever and
        its 9Ps lever, up to the vertical offset between the two antennas that A58's fit holds at zero (0.4 mm)."""
        rows, _, _, res = self.derive()
        lv = res["lever"]
        R_am = rows[("avia", "mid360")][0]
        chain_xy = (R_am.T @ lv["gt_mid360_imu"])[:2]
        a77 = hgl.derive_a77(rows, LEVER_REF, chain_xy)
        np.testing.assert_allclose(a77["gt_lever_mid360_imu"], lv["gt_mid360_imu"], atol=5e-7)
        self.assertAlmostEqual(float(np.linalg.norm(np.asarray(a77["l9ps_mid360_lidar"]) - lv["p_ant_mid360"])), 0.0004, places=6)
        self.assertEqual(all(c["ok"] for c in a77["checks"]), True)
        self.assertEqual(len(a77["checks"]), 6)

    def test_a77_9ps_lever_is_the_gt_lever_plus_the_vehicle_axes_offset_through_the_chain_rotation(self):
        rows, _, _, res = self.derive()
        a77 = hgl.derive_a77(rows, LEVER_REF, A77_LEVEL_XY)
        R_am, _ = rows[("avia", "mid360")]
        R_mi, t_mi = rows[("mid360", "mid360_imu")]
        off = np.array([-0.31635, 0.10926, 0.4006]) - np.array(LEVER_REF)
        np.testing.assert_allclose(a77["offset_9ps_minus_gt_avia_axes"], [off[0], off[1], 0.0], atol=1e-12)
        np.testing.assert_allclose(a77["offset_9ps_minus_gt_avia_axes"], [-0.14035, 0.12526, 0.0], atol=1e-12)
        np.testing.assert_allclose(np.asarray(a77["l9ps_mid360_imu"]) - np.asarray(a77["gt_lever_mid360_imu"]),
                                   R_am @ np.array([off[0], off[1], 0.0]), atol=1e-12)
        # lever_arm_frame lidar: the filter takes l^I = R_IL l^L + t_IL (extrinsic_R = I): the config value is l^I - t_IL
        np.testing.assert_allclose(R_mi @ np.asarray(a77["l9ps_mid360_lidar"]) + t_mi, a77["l9ps_mid360_imu"], atol=1e-12)
        np.testing.assert_allclose(a77["l9ps_mid360_lidar"], A77_9PS_LEVER_MID360_LIDAR, atol=1e-9)
        np.testing.assert_allclose(a77["l9ps_mid360_lidar"], np.asarray(a77["l9ps_mid360_imu"]) - np.array([-0.011, -0.02329, 0.04412]), atol=1e-12)
        # the separation of the two antennas is the offset's length whatever the fit says
        self.assertAlmostEqual(float(np.linalg.norm(np.asarray(a77["l9ps_mid360_imu"]) - np.asarray(a77["gt_lever_mid360_imu"]))),
                               float(np.hypot(off[0], off[1])), places=6)

    def test_a77_refuses_a_calibration_that_fails_its_own_checks(self):
        rows, _, _, _ = self.derive()
        bad = dict(rows)
        R, T = bad[("avia", "avia_imu")]
        bad[("avia", "avia_imu")] = (R, T + np.array([0.01, 0.0, 0.0]))        # breaks the A15 value
        with self.assertRaises(ValueError):
            hgl.derive_a77(bad, LEVER_REF, A77_LEVEL_XY)

    def test_mid360_yaml_is_the_avia_yaml_with_the_host_keys_of_the_official_file(self):
        import yaml

        def flat(d, pre=""):
            out = {}
            for k, v in d.items():
                out.update(flat(v, pre + k + "/") if isinstance(v, dict) else {pre + k: v})
            return out
        with open(os.path.join(REPO, "config", "m3dgr_avia.yaml")) as f:
            a = flat(yaml.safe_load(f))
        with open(os.path.join(REPO, "config", "m3dgr_mid360.yaml")) as f:
            m = flat(yaml.safe_load(f))
        self.assertEqual(set(a), set(m))
        diff = sorted(k for k in a if a[k] != m[k])
        self.assertEqual(diff, ["common/imu_topic", "common/lid_topic", "extrin_calib/Pcl", "extrin_calib/Rcl",
                                "extrin_calib/extrinsic_T", "gnss/frames/lever_arm_xyz", "gnss/imu_bridge/acc_n",
                                "gnss/imu_bridge/gyr_n", "gnss/process/frame/q_th_per_m", "gnss/process/frame/q_tilt_per_m",
                                "gnss/process/frame/q_tv_per_m", "gnss/process/frame/q_yaw_per_m",
                                "gnss/process/frame/q_yaw_per_rad", "preprocess/scan_line"])
        # A78: the registration random walk is this host's own (the Avia's A58 horizontal / A70 vertical values describe the Avia's drift);
        # q_time (default 0, no drift at rest) is the one key of the frame block both files share
        for k, v in A78_FRAME_Q_MID360.items():
            self.assertEqual(m["gnss/process/frame/" + k], v, k)
        self.assertEqual(a["gnss/process/frame/q_time"], m["gnss/process/frame/q_time"])
        # the official m3dgr_mid360.yaml (sjtuyinjie/M3DGR, blob 347476bc): topics, extrinsics, camera-LiDAR, scan_line
        self.assertEqual((m["common/lid_topic"], m["common/imu_topic"]), ("/livox/mid360/lidar", "/livox/mid360/imu"))
        self.assertEqual(m["extrin_calib/extrinsic_T"], [-0.011, -0.02329, 0.04412])
        self.assertEqual(m["extrin_calib/extrinsic_R"], [1, 0, 0, 0, 1, 0, 0, 0, 1])
        self.assertEqual(m["extrin_calib/Rcl"], [0.0722207, -0.997387, -0.0020542, 0.521005, 0.039482, -0.85264,
                                                0.850493, 0.060508, 0.522494])
        self.assertEqual(m["extrin_calib/Pcl"], [-0.0442358, -0.411712, 0.168568])
        self.assertEqual((m["preprocess/scan_line"], m["preprocess/lidar_type"], m["preprocess/blind"],
                          m["preprocess/point_filter_num"]), (4, 1, 0.8, 1))
        # ... which are the calibration rows: mid360 -> mid360_imu, mid360 -> camera
        rows, intr, imu, res = self.derive()
        np.testing.assert_allclose(m["extrin_calib/extrinsic_T"], rows[("mid360", "mid360_imu")][1], atol=1e-12)
        np.testing.assert_allclose(m["extrin_calib/Rcl"], rows[("mid360", "camera")][0].ravel(), atol=1e-6)
        np.testing.assert_allclose(m["extrin_calib/Pcl"], rows[("mid360", "camera")][1], atol=1e-6)
        # the derived GNSS side: the 9Ps antenna in the MID-360 frame (A77: fitted GT lever + vehicle-axes offset, not the chain's
        # translation), the IMU noise rounded to three digits like the Avia's
        np.testing.assert_allclose(m["gnss/frames/lever_arm_xyz"], hgl.derive_a77(rows, LEVER_REF, A77_LEVEL_XY)["l9ps_mid360_lidar"], atol=1e-9)
        np.testing.assert_allclose(m["gnss/frames/lever_arm_xyz"], A77_9PS_LEVER_MID360_LIDAR, atol=1e-9)
        self.assertEqual(m["gnss/frames/lever_arm_frame"], "lidar")
        self.assertEqual(m["gnss/imu_bridge/acc_n"], float("%.3g" % imu["mid360"]["acc_n"]))
        self.assertEqual(m["gnss/imu_bridge/gyr_n"], float("%.3g" % imu["mid360"]["gyr_n"]))
        self.assertEqual((a["gnss/imu_bridge/acc_n"], a["gnss/imu_bridge/gyr_n"]),
                         (float("%.3g" % imu["avia"]["acc_n"]), float("%.3g" % imu["avia"]["gyr_n"])))
        # the BMI088 turn-on prior belongs to the Avia IMU: off in both
        self.assertFalse(m["imu/turnon_prior_en"])


def ec_lidars():
    import json
    with open(os.path.join(REPO, "tools", "eval", "lidars.json")) as f:
        return json.load(f)["lidars"]


if __name__ == "__main__":
    unittest.main()
