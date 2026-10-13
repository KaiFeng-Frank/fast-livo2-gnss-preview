# SPDX-License-Identifier: BSD-3-Clause
"""Shared building blocks of the A80 dataset tools (tools/datasets): RINEX 3 reading, RINEX -> gnss_comm messages,
raw ROS1 message editing, the time-ordered slim-bag writer, HTTP range fetching, Inertial Explorer .ref reading and
the ground-truth conversion.

Python 3.8 (the project image fastlivo2-gnss:noetic), numpy, rosbags (Apache-2.0, a separate program like in
tools/gnss_bag_export.py), cv2 for image slimming. Nothing of gnss_comm is copied here: the message definitions are
read from a reference bag's own connection headers at run time (DESIGN §12.2, docs/LICENSING.md).
"""
import os
import sys

TOOL_VERSION = 1

# tools/ (the fl2g package: time scales, geodesy, satellite tables, RINEX NAV reader) next to tools/datasets/
_TOOLS = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _TOOLS not in sys.path:
    sys.path.insert(0, _TOOLS)
