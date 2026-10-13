# SPDX-License-Identifier: BSD-3-Clause
"""Shared helpers of the WP-g offline tools (tools/*.py): time scales, satellite numbering, RINEX NAV,
broadcast orbits, GNSS-only least squares, ROS1 bag access, robust statistics, A18 calibration files.

Pure numpy (+ the ROS1 `rosbag` Python API where a bag is read); runs in the project image
fastlivo2-gnss:noetic. No gnss_comm code or message text is included: gnss_comm messages are read
through the message_definition carried by the bag itself, fields are addressed by name (DESIGN §12.2).
"""

# 1: round 1 (WP-g); 2: P2b (A38 Klobuchar source, A1/A38 topic requirements, per-satellite §12.6-2, A35-C8 sat2freq,
#    §12.6-7 lever arm, GT time-base diagnostics and bag-span mask); 3: P4 (A53 TDCP rule and per-span choice of the
#    more precise time-calibration method, method-2 signal = horizontal speed of vector-smoothed velocities with a
#    two-pass weighted Doppler LS and the method-1 drift; A54 GT time-base rule / gt_timebase.yaml, RTK jump mask).
#    The A18 files carry it as tool_version.
TOOL_VERSION = 3
