#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Stand-in for `sudo -n docker run ... bash -c <script>` in test_preprocess_seq.py: records the script (FAKE_TOOL_LOG)
and writes the reports the real tools would (time_calib/<seq>.json, gt/gt_audit.json) into the -v <root>:/r tree."""
import json
import os
import shlex
import sys

argv = sys.argv[1:]
assert argv[0] == "run", argv
root = [argv[i + 1].split(":/r")[0] for i, a in enumerate(argv) if a == "-v" and argv[i + 1].endswith(":/r")][0]
script = argv[-1].split("; ", 1)[1]
with open(os.environ["FAKE_TOOL_LOG"], "a") as f:
    f.write(json.dumps({"workdir": argv[argv.index("-w") + 1], "script": script}) + "\n")
tok = shlex.split(script)
host = lambda p: root + p[2:] if p.startswith("/r/") else p
if tok[1].endswith("gnss_time_calib.py"):
    out = host(tok[4])
    os.makedirs(os.path.dirname(out), exist_ok=True)
    rep = {"method2_status": "ran" if tok[3] != "-" else "not run", "method1_estimator_clock_prior": {"offset_s": -0.0138},
           "segments": [{"t_from_gpst_s": 0.0, "offset_s": 0.0412, "drift_ppm": 0.6, "sigma_toff_s": 0.0305, "tdcp_paused": False,
                         "method": "xcorr" if tok[3] != "-" else "record_time", "peak_corr": 0.98, "n_windows": 3}]}
    with open(os.path.splitext(out)[0] + ".json", "w") as f:
        json.dump(rep, f)
    with open(out, "w") as f:
        f.write("sequence: X\n")
elif tok[1].endswith("gt_audit.py"):
    out = host(tok[4])
    os.makedirs(out, exist_ok=True)
    with open(os.path.join(out, "gt_audit.json"), "w") as f:
        json.dump({"verdict": "FAIL", "timebase": {"method": "record_time", "offset_s": -0.0059, "applied_shift_s": 0.0, "confidence": "low"},
                   "trust_mask": {"masked_s": {"ate_exclude": 10.2}, "rtk": {"plateaus": [{}], "unpaired": []}}}, f)
sys.exit(0)
