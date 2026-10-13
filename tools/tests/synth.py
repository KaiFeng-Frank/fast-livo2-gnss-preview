# SPDX-License-Identifier: BSD-3-Clause
"""Synthetic ground-robot motion shared by the tool tests: planar trajectory with a rich speed / yaw-rate spectrum,
integrated at 1 ms, with an antenna lever arm. Everything deterministic (fixed seeds)."""
import math

import numpy as np

LEVER = np.array([-0.31635, 0.10926, 0.4006])


def speed_profile(t):
    return 1.2 + 0.35 * np.sin(2 * np.pi * t / 13.0) + 0.25 * np.sin(2 * np.pi * t / 4.7 + 0.7) + 0.15 * np.sin(2 * np.pi * t / 29.0 + 2.0)


def yaw_rate_profile(t):
    return 0.3 * np.sin(2 * np.pi * t / 21.0) + 0.2 * np.sin(2 * np.pi * t / 7.3 + 1.1)


class Motion:
    """Continuous planar motion on [t0, t1] (estimator time). IMU frame: x forward, z up; R = Rz(yaw)."""

    def __init__(self, t0, t1, dt=0.001, speed=speed_profile, yaw_rate=yaw_rate_profile):
        self.t = np.arange(t0, t1 + dt, dt)
        tt = self.t - t0
        self.v = speed(tt)
        self.w = yaw_rate(tt)
        self.yaw = np.concatenate([[0.0], np.cumsum(0.5 * (self.w[1:] + self.w[:-1]) * dt)])
        vx = self.v * np.cos(self.yaw)
        vy = self.v * np.sin(self.yaw)
        self.x = np.concatenate([[0.0], np.cumsum(0.5 * (vx[1:] + vx[:-1]) * dt)])
        self.y = np.concatenate([[0.0], np.cumsum(0.5 * (vy[1:] + vy[:-1]) * dt)])

    def pose(self, t):
        yaw = np.interp(t, self.t, self.yaw)
        return np.interp(t, self.t, self.x), np.interp(t, self.t, self.y), yaw

    def antenna_speed(self, t, lever=LEVER):
        """|v + omega x R l| exactly (analytic in the continuous model)."""
        return np.linalg.norm(self.antenna_velocity(t, lever)[:, :2], axis=1)

    def antenna_velocity(self, t, lever=LEVER):
        """v + omega x R l in the (level) world frame, (n, 3) with zero vertical (analytic in the continuous model)."""
        v = np.interp(t, self.t, self.v)
        w = np.interp(t, self.t, self.w)
        yaw = np.interp(t, self.t, self.yaw)
        c, s = np.cos(yaw), np.sin(yaw)
        lx = c * lever[0] - s * lever[1]
        ly = s * lever[0] + c * lever[1]
        return np.column_stack([v * c - w * ly, v * s + w * lx, np.zeros_like(v)])

    def tum_rows(self, times):
        rows = []
        for t in times:
            x, y, yaw = self.pose(t)
            rows.append((t, x, y, 0.0, 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)))
        return rows


def estimator_times(t0, t1, seed=1):
    """LIO 10 Hz + VIO 15 Hz step times with 2 ms jitter (like traj_W.tum)."""
    rng = np.random.RandomState(seed)
    a = np.arange(t0, t1, 0.1) + rng.uniform(-0.002, 0.002, int(math.ceil((t1 - t0) / 0.1)))[: len(np.arange(t0, t1, 0.1))]
    b = np.arange(t0 + 0.033, t1, 1.0 / 15.0)
    t = np.sort(np.concatenate([a, b]))
    return t[(t > t0 + 0.01) & (t < t1 - 0.01)]
