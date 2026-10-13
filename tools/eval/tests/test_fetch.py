# SPDX-License-Identifier: BSD-3-Clause
"""fetch_eval.py: listing/checksum/BRDC parsing, the unpacking of split SFX sets (7-Zip volumes vs byte
concatenation must give the same bag), the A2 refusal before anything is touched, drop/status."""
import datetime
import json
import os
import shutil
import subprocess
import tempfile
import unittest
from unittest import mock

import _path  # noqa: F401
import fetch_eval as fe

LS_PARTS = """
当前目录: /来自分享/Dataset/Visual_Challenge/Outdoor/Varying-illu03
----
  #   文件大小        修改日期                文件(目录)          
  1      2.00GB  2026-09-24 20:45:51  Varying-illu03.part004.exe  
  2      2.00GB  2026-09-24 20:45:51  Varying-illu03.part003.exe  
  3      2.00GB  2026-09-24 20:45:51  Varying-illu03.part002.exe  
  4      2.00GB  2026-09-24 20:45:51  Varying-illu03.part001.exe  
  5    287.61MB  2026-09-24 20:45:51  Varying-illu03.part005.exe  
  6      3.80KB  2026-09-24 20:45:51  merge_script.ps1            
  7        824B  2026-09-24 20:45:51  双击合并.bat                
     总: 8.28GB                       文件总数: 7, 目录总数: 0    
----
"""
LS_DIRS = """
当前目录: /来自分享/Dataset/Visual_Challenge/Outdoor
----
  #   文件大小        修改日期               文件(目录)         
  1           -  2026-09-24 20:45:52  Varying-illu05/           
  5      2.68GB  2026-09-24 20:45:52  Occlusion04.exe           
     总: 9.55GB                       文件总数: 5, 目录总数: 4  
----
"""
CHECKSUMS = ("﻿942E560ED7DEE3FB3F01317D92AA906790771C16C91B2C943DD800C4F77030FB  GNSS_Denied\\GNSS-denial01.bag\n"
             "00A270807BCA336BC5191320B9D453CEFEF76507850286E91063BC726C3775C8  Standard\\Outdoor01.bag\n"
             "9D9F17BD40D3A2721A9780D0861147A19ED21FDEB3CE1686A8A059EA70CB7C0E  Standard\\Outdoor01.orig.bag\n")


class Parsing(unittest.TestCase):
    def test_sizes(self):
        self.assertEqual(fe.parse_size("2.00GB"), 2 * 1024 ** 3)
        self.assertEqual(fe.parse_size("824B"), 824)
        self.assertEqual(fe.parse_size("287.61MB"), int(287.61 * 1024 ** 2))
        self.assertIsNone(fe.parse_size("-"))

    def test_listing_and_piece_order(self):
        ents = fe.parse_ls(LS_PARTS)
        self.assertEqual(len(ents), 7)
        parts = fe.select_parts(ents, "Varying-illu03")
        self.assertEqual([p["name"] for p in parts], ["Varying-illu03.part%03d.exe" % i for i in range(1, 6)])
        d = fe.parse_ls(LS_DIRS)
        self.assertEqual([(e["name"], e["is_dir"]) for e in d], [("Varying-illu05", True), ("Occlusion04.exe", False)])

    def test_piece_gaps_and_foreign_pieces(self):
        ents = [e for e in fe.parse_ls(LS_PARTS) if "part003" not in e["name"]]
        with self.assertRaises(ValueError):
            fe.select_parts(ents, "Varying-illu03")
        with self.assertRaises(ValueError):
            fe.select_parts(fe.parse_ls(LS_PARTS), "Varying-illu04")

    def test_checksum_lookup(self):
        self.assertEqual(fe.checksum_lookup(CHECKSUMS, "GNSS_Denied\\GNSS-denial01.bag")[:8], "942E560E")
        self.assertEqual(fe.checksum_lookup(CHECKSUMS, "Standard\\Outdoor01.bag")[:8], "00A27080")
        self.assertEqual(fe.checksum_lookup(CHECKSUMS, "Standard\\Outdoor01.orig.bag")[:8], "9D9F17BD")
        self.assertIsNone(fe.checksum_lookup(CHECKSUMS, "Standard\\Outdoor04.bag"))

    def test_brdc_days(self):
        t = datetime.datetime(2025, 1, 19, 10, 0, tzinfo=datetime.timezone.utc).timestamp()
        self.assertEqual(fe.brdc_names(t, t + 300), [(2025, "019", "BRDC00IGS_R_20250190000_01D_MN.rnx")])
        late = datetime.datetime(2024, 12, 31, 23, 59, 50, tzinfo=datetime.timezone.utc).timestamp()
        # GPST leads UTC by 18 s: a bag ending 10 s before UTC midnight already has observations of the next GPST day
        self.assertEqual([n[2] for n in fe.brdc_names(late - 100, late)],
                         ["BRDC00IGS_R_20243660000_01D_MN.rnx", "BRDC00IGS_R_20250010000_01D_MN.rnx"])

    def test_rosbag_info_yaml(self):
        y = "path: /r/x.bag\nversion: 2.0\nduration: 288.12\nstart: 1737278585.50\nend: 1737278873.62\nindexed: True\ntopics:\n  - topic: /a\n"
        d = fe.parse_rosbag_info_yaml(y)
        self.assertEqual((d["start"], d["end"], d["indexed"]), ("1737278585.50", "1737278873.62", "True"))


@unittest.skipUnless(shutil.which("7z"), "7z not installed")
class Unpack(unittest.TestCase):
    """A synthetic 7-Zip SFX split like the share's pieces: both unpacking routes give the original bag."""

    def setUp(self):
        self.d = tempfile.mkdtemp()
        self.payload = os.urandom(3 * 1000 * 1000)
        with open(os.path.join(self.d, "Seq01.bag"), "wb") as f:
            f.write(self.payload)
        r = subprocess.run(["7z", "a", "-sfx", "Seq01.exe", "Seq01.bag"], cwd=self.d, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        if r.returncode != 0 or not os.path.isfile(os.path.join(self.d, "Seq01.exe")):
            self.skipTest("7z cannot build an SFX here: %s" % r.stdout[-200:])
        os.remove(os.path.join(self.d, "Seq01.bag"))
        with open(os.path.join(self.d, "Seq01.exe"), "rb") as f:
            blob = f.read()
        os.remove(os.path.join(self.d, "Seq01.exe"))
        self.pieces = []
        for i in range(0, len(blob), 1000 * 1000):
            p = os.path.join(self.d, "Seq01.part%03d.exe" % (len(self.pieces) + 1))
            with open(p, "wb") as f:
                f.write(blob[i:i + 1000 * 1000])
            self.pieces.append(p)
        self.assertGreater(len(self.pieces), 2)

    def tearDown(self):
        shutil.rmtree(self.d)

    def extract(self, arch, out):
        lst = subprocess.run(["7z", "l", "-slt", arch], stdout=subprocess.PIPE, universal_newlines=True).stdout
        files = fe.seven_zip_listing(lst)
        self.assertEqual([f["path"] for f in files], ["Seq01.bag"])
        self.assertEqual(files[0]["size"], len(self.payload))
        subprocess.run(["7z", "x", "-y", "-o" + out, arch], stdout=subprocess.DEVNULL, check=True)
        with open(os.path.join(out, "Seq01.bag"), "rb") as f:
            return f.read()

    def test_volumes_and_cat_give_the_same_bag(self):
        links = fe.volume_links(self.d, "Seq01", self.pieces)
        self.assertEqual(os.path.basename(links[0]), "Seq01.exe.001")
        self.assertEqual(self.extract(links[0], os.path.join(self.d, "v")), self.payload)
        merged = os.path.join(self.d, "Seq01.exe")
        n = fe.cat_merge(self.pieces, merged)
        self.assertEqual(n, os.path.getsize(merged))
        self.assertFalse(any(os.path.exists(p) for p in self.pieces))    # pieces deleted as they are appended
        self.assertEqual(self.extract(merged, os.path.join(self.d, "c")), self.payload)


class ReindexDuration(unittest.TestCase):
    """rosbag reindex can exit 0 after indexing only the prefix before a corrupt chunk (Outdoor01, 2026-09-30)."""

    def test_registry_rounding_is_accepted(self):
        # README duration_s 411; the A3 repaired bag measures 411.6 s.
        self.assertEqual(fe.reindex_decision(411.567, 411, sha_official=False, allow_repair=True), "accept")
        self.assertEqual(fe.reindex_decision(411.567, 411, sha_official=False, allow_repair=False), "accept")

    def test_truncated_reindex_requests_repair(self):
        # 262.6 / 411 = 63.8%, the prefix rosbag reindex kept on the Alipan Outdoor01 bag.
        self.assertEqual(fe.reindex_decision(262.564, 411, sha_official=False, allow_repair=True), "repair")

    def test_truncated_reindex_fails_without_repair_or_on_an_official_bag(self):
        self.assertEqual(fe.reindex_decision(262.564, 411, sha_official=False, allow_repair=False), "fail")
        self.assertEqual(fe.reindex_decision(262.564, 411, sha_official=True, allow_repair=True), "fail")

    def test_floor_boundary(self):
        self.assertEqual(fe.reindex_decision(0.98 * 411, 411, False, True), "accept")
        self.assertEqual(fe.reindex_decision(0.98 * 411 - 0.01, 411, False, True), "repair")

    def test_missing_registry_duration_does_not_force_repair(self):
        self.assertEqual(fe.reindex_decision(10.0, None, False, True), "accept")
        self.assertEqual(fe.reindex_decision(10.0, 0, False, True), "accept")

    def test_bag_duration_ignores_a_trailing_unit(self):
        self.assertEqual(fe.bag_duration_s({"duration": "262.563796"}), 262.563796)
        self.assertEqual(fe.bag_duration_s({"duration": "411.566921 s"}), 411.566921)
        self.assertEqual(fe.bag_duration_s({}), 0.0)


class Guard(unittest.TestCase):
    def test_evaluation_sequence_refused_before_anything_is_touched(self):
        with tempfile.TemporaryDirectory() as root:
            with mock.patch.dict(os.environ, {"FL2G_ROOT": root}), mock.patch.object(fe, "Fetch", side_effect=AssertionError("touched")):
                self.assertEqual(fe.main(["fetch", "Dark01"]), fe.EX_BLOCKED)
                self.assertEqual(fe.main(["fetch", "NotASequence"]), fe.EX_BLOCKED)
            self.assertEqual(os.listdir(root), [])

    def test_drop_and_status(self):
        with tempfile.TemporaryDirectory() as root:
            fd = os.path.join(root, "data", "m3dgr", "filtered")
            os.makedirs(fd)
            for n in ("Grass01.bag", "Grass01.bag.sha256"):
                with open(os.path.join(fd, n), "w") as f:
                    f.write("x" * 10)
            with open(os.path.join(fd, "Grass01.fetch.json"), "w") as f:
                json.dump({"sequence": "Grass01", "status": "done", "steps": []}, f)
            self.assertEqual(fe.main(["--root", root, "drop", "Grass01"]), 0)
            self.assertEqual(sorted(os.listdir(fd)), ["Grass01.fetch.json"])
            with open(os.path.join(fd, "Grass01.fetch.json")) as f:
                self.assertEqual(json.load(f)["status"], "dropped")
            self.assertEqual(fe.main(["--root", root, "status"]), 0)


class HostLidar(unittest.TestCase):
    """A76: `--lidar mid360` sends the filtered bag (MID-360 topics) to data/m3dgr/filtered_mid360 and stages in
    fetch_mid360; the Avia products and the raw bag are never touched; the Avia command line has no --lidar."""

    SHA = "67B3E299920696D041E8937636F64515CDB62F8E93D827D9B8431A5572D9109A"          # Grass01's official sha256

    def run_fetch(self, root, *global_args):
        calls = []

        def fake_docker(inst, script, log_name, check=True, capture=False):
            calls.append(script)
            if script.startswith("rosbag info"):
                return 0, "start: 1737278585.5\nend: 1737278873.4\nduration: 287.9\nmessages: 220097\nindexed: True\n"
            if "tools/bag_filter.py" in script:
                import shlex
                out = shlex.split(script)[3]
                host = os.path.join(root, out[len("/r/"):])
                with open(host, "w") as f:
                    f.write("filtered bytes")
                with open(host + ".sha256", "w") as f:
                    f.write("%s  %s\n" % ("c" * 64, os.path.basename(host)))
                with open(os.path.join(inst.rec_dir, log_name), "a") as f:
                    f.write("bag_filter: verified: 1 messages\n")
                return 0, None
            raise AssertionError(script)

        def fake_run(inst, cmd, log_name=None, check=True, capture=False, timeout=None):
            if cmd[0] == "sha256sum":
                return 0, "%s  %s\n" % (self.SHA.lower(), cmd[1])
            raise AssertionError(cmd)
        with mock.patch.dict(os.environ, {"FL2G_ROOT": root}), \
                mock.patch.object(fe.Fetch, "docker", fake_docker), mock.patch.object(fe.Fetch, "run", fake_run), \
                mock.patch.object(fe.Fetch, "ensure_gt", lambda inst, seq, info: None), \
                mock.patch.object(fe.Fetch, "ensure_brdc", lambda inst, a, b: ["data/brdc/x.rnx"]):
            rc = fe.main(["--root", root] + list(global_args) + ["fetch", "Grass01", "--no-gt-audit"])
        return rc, calls

    def make_root(self, root):
        os.makedirs(os.path.join(root, "data", "m3dgr", "bags"))
        with open(os.path.join(root, "data", "m3dgr", "bags", "Grass01.bag"), "wb") as f:
            f.write(b"raw")
        fa = os.path.join(root, "data", "m3dgr", "filtered")
        os.makedirs(fa)
        with open(os.path.join(fa, "Grass01.bag"), "w") as f:
            f.write("AVIA")
        return fa

    def test_mid360_goes_to_its_own_directory_with_its_own_filter_topics(self):
        with tempfile.TemporaryDirectory() as root:
            fa = self.make_root(root)
            rc, calls = self.run_fetch(root, "--lidar", "mid360")
            self.assertEqual(rc, 0)
            fm = os.path.join(root, "data", "m3dgr", "filtered_mid360")
            with open(os.path.join(fm, "Grass01.bag")) as f:
                self.assertEqual(f.read(), "filtered bytes")
            self.assertTrue(os.path.isfile(os.path.join(fm, "Grass01.bag.sha256")))
            with open(os.path.join(fm, "Grass01.fetch.json")) as f:
                rec = json.load(f)
            self.assertEqual((rec["status"], rec["lidar"], rec["filtered_bag"]), ("done", "mid360", "data/m3dgr/filtered_mid360/Grass01.bag"))
            filt = [c for c in calls if "bag_filter.py" in c][0]
            self.assertIn("--lidar mid360", filt)
            self.assertIn("/r/data/m3dgr/fetch_mid360/Grass01/", filt)
            self.assertEqual(os.listdir(fa), ["Grass01.bag"])                           # the Avia directory is untouched
            with open(os.path.join(fa, "Grass01.bag")) as f:
                self.assertEqual(f.read(), "AVIA")
            self.assertTrue(os.path.isfile(os.path.join(root, "data", "m3dgr", "bags", "Grass01.bag")))   # raw bag kept
            self.assertFalse(os.path.isdir(os.path.join(root, "data", "m3dgr", "fetch")))
            # drop and status act on the MID-360 directory only
            self.assertEqual(fe.main(["--root", root, "--lidar", "mid360", "drop", "Grass01"]), 0)
            self.assertFalse(os.path.exists(os.path.join(fm, "Grass01.bag")))
            self.assertTrue(os.path.exists(os.path.join(fa, "Grass01.bag")))

    def test_the_avia_command_line_is_unchanged(self):
        with tempfile.TemporaryDirectory() as root:
            self.make_root(root)
            os.remove(os.path.join(root, "data", "m3dgr", "filtered", "Grass01.bag"))
            rc, calls = self.run_fetch(root)
            self.assertEqual(rc, 0)
            filt = [c for c in calls if "bag_filter.py" in c][0]
            self.assertNotIn("--lidar", filt)
            self.assertIn("/r/data/m3dgr/fetch/Grass01/", filt)
            with open(os.path.join(root, "data", "m3dgr", "filtered", "Grass01.fetch.json")) as f:
                self.assertNotIn("lidar", json.load(f))
            self.assertFalse(os.path.exists(os.path.join(root, "data", "m3dgr", "filtered_mid360")))


if __name__ == "__main__":
    unittest.main()
