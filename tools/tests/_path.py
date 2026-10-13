# SPDX-License-Identifier: BSD-3-Clause
"""Put tools/ and tools/time_calib/ on sys.path for the tool tests (run from anywhere)."""
import os
import sys

TOOLS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO = os.path.dirname(TOOLS)
for p in (TOOLS, os.path.join(TOOLS, "time_calib")):
    if p not in sys.path:
        sys.path.insert(0, p)
