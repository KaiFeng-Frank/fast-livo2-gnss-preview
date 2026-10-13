# SPDX-License-Identifier: BSD-3-Clause
"""Put tools/eval and tools/ on sys.path for the P3-D tool tests (run from anywhere)."""
import os
import sys

EVAL = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TOOLS = os.path.dirname(EVAL)
REPO = os.path.dirname(TOOLS)
for p in (EVAL, TOOLS):
    if p not in sys.path:
        sys.path.insert(0, p)
