# SPDX-License-Identifier: BSD-3-Clause
"""Reading the state trace of a deterministic replay (fastlivo_replay, DESIGN §13).

Users: the lever-arm check (m3dgr_audit G13) and the GT time-base diagnostic (gt_audit P4c), both on development
sequences only (AMENDMENTS A2: no estimated trajectory is read on an evaluation sequence). gnss_time_calib's
method 2 has its own reader for traj_W.tum / state_trace.txt (it needs only the antenna track)."""
import numpy as np


def load_state_trace(path):
    """state_trace.txt (ros/src/outputs.cpp: a '#' header line, then per processOnce `cycle flag t_k R(9, row-major) p v
    b_g b_a g tau diag(P) ...`, reals as %a hexfloat) -> arrays t, R (n,3,3), p, v, bg, ba, g in W on the estimator
    clock. Rows of equal t_k (the VIO update after the LIO update of one image time) collapse to the last one."""
    rows = []
    with open(path) as f:
        for ln in f:
            s = ln.strip()
            if not s or s.startswith("#"):
                continue
            tok = s.split()
            if len(tok) < 27:
                continue
            vals = []
            for x in tok[:27]:
                try:
                    vals.append(float(x))          # the integer columns; float.fromhex("10") would read 16
                except ValueError:
                    vals.append(float.fromhex(x))
            rows.append(vals)
    if not rows:
        raise ValueError("no state rows in %s" % path)
    a = np.array(rows)
    a = a[np.argsort(a[:, 2], kind="stable")]
    keep = np.ones(a.shape[0], dtype=bool)
    keep[:-1] = a[1:, 2] != a[:-1, 2]
    a = a[keep]
    return {"t": a[:, 2], "R": a[:, 3:12].reshape(-1, 3, 3), "p": a[:, 12:15], "v": a[:, 15:18], "bg": a[:, 18:21],
            "ba": a[:, 21:24], "g": a[:, 24:27]}
