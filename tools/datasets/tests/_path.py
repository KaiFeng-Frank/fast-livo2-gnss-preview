# SPDX-License-Identifier: BSD-3-Clause
"""Put tools/ and tools/datasets/ on sys.path for the dataset-tool tests (run from anywhere)."""
import os
import sys

DATASETS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TOOLS = os.path.dirname(DATASETS)
REPO = os.path.dirname(TOOLS)
EVAL = os.path.join(TOOLS, "eval")
for p in (DATASETS, TOOLS, EVAL):
    if p not in sys.path:
        sys.path.insert(0, p)

# the M3DGR bag whose connection headers carry the gnss_comm definitions (remote host only)
REFERENCE_BAG = os.environ.get("FL2G_GNSS_REF_BAG", "/r/data/m3dgr/filtered/Grass01.bag")
