#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Stand-in for docs/dev/ref_r_gates/manifest_check.py --run in test_run_matrix.py (the real one is WP-f's)."""
import os
import sys

run = sys.argv[sys.argv.index("--run") + 1]
ok = os.path.isfile(os.path.join(run, "run_manifest.json"))
print("fake manifest_check: %s" % ("OK" if ok else "FAIL"))
sys.exit(0 if ok else 1)
