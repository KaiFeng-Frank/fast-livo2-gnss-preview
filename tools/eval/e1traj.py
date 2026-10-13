# SPDX-License-Identifier: BSD-3-Clause
"""E1 trajectories of the GNSS arms (AMENDMENTS A52) and the self-checks around them (P4-EVAL).

A GNSS arm writes, at every step end, traj_W.tum (T_{W<-I}; src/replay_main.cpp afterCycle) and, from ACTIVATE on,
traj_N.tum (T_{N<-I}; ros/src/gnss_ros_bridge.cpp writeStepOutputs): the same t_k (LidarMeasures.last_lio_update_time,
printed %.9f by both) and the same host state (after onPostVio). With DESIGN §1 (R_{N<-I} = M R, p^N_I = M(p - o^W) + t)
the registration of one step end is therefore exactly
    T_{N<-W} = T_{N<-I} o T_{W<-I}^-1 = (M, t - M o^W),
so no further output is needed (up to the 1e-9 rounding of the text files). E1 trajectory (A52, whole sequence):
  * a step end with a traj_N row: that row (the global output);
  * a step end before activation: T_{N<-W}(activation) o T_{W<-I}(t) — one estimate, taken at the activation step,
    i.e. "pure LIVO until activation", no information from later than the activation;
  * a step end after activation without a traj_N row (REINIT re-collection: no registration, no global output): the
    latest T_{N<-W} before it (causal);
  * a run that never activated: traj_W itself (a fixed rigid transform does not change an SE(3)-aligned ATE; the host
    of such a run is the U host, DESIGN §5 (f)).
Self-checks (instrument, not results): the heading of M — the azimuth of its first column, = psi + O(tilt^2) because
C(g) is a yaw-free levelling and Exp([phi, 0]) is a 1e-4 rad tilt — against the filter's own psi in gnss_epochs.csv at
the activation step; the first ACTIVATE of events.log at the first traj_N stamp.
"""
import csv
import math
import os
import re

import numpy as np

HEADING_TOL_DEG = 1.0          # + tilt^2/2: |azimuth(M) - psi| <= tilt^2/4 in exact arithmetic (dev set: <= 0.18 deg)
ACTIVATE_MAX_LAG_S = 1.0       # first traj_N stamp within one second of the first ACTIVATE (same step in practice)
_EVENT = re.compile(r"^t_k=(\S+) t_est=\S+ gpst=\S+ (\S+) magnitude=")


# ---------------------------------------------------------------------------------------------------------------
# rotations (TUM order: qx qy qz qw)
def quat_to_rot(q):
    q = np.asarray(q, dtype=float).reshape(-1, 4)
    q = q / np.linalg.norm(q, axis=1, keepdims=True)
    x, y, z, w = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    r = np.empty((q.shape[0], 3, 3))
    r[:, 0, 0] = 1 - 2 * (y * y + z * z)
    r[:, 0, 1] = 2 * (x * y - z * w)
    r[:, 0, 2] = 2 * (x * z + y * w)
    r[:, 1, 0] = 2 * (x * y + z * w)
    r[:, 1, 1] = 1 - 2 * (x * x + z * z)
    r[:, 1, 2] = 2 * (y * z - x * w)
    r[:, 2, 0] = 2 * (x * z - y * w)
    r[:, 2, 1] = 2 * (y * z + x * w)
    r[:, 2, 2] = 1 - 2 * (x * x + y * y)
    return r


def rot_to_quat(r):
    """(n, 3, 3) -> (n, 4) unit quaternions (qx qy qz qw, qw >= 0), Shepperd's branch per row."""
    r = np.asarray(r, dtype=float).reshape(-1, 3, 3)
    tr = r[:, 0, 0] + r[:, 1, 1] + r[:, 2, 2]
    k = np.argmax(np.stack([tr, r[:, 0, 0], r[:, 1, 1], r[:, 2, 2]], axis=1), axis=1)
    q = np.empty((r.shape[0], 4))
    m = k == 0
    s = np.sqrt(np.maximum(tr[m] + 1.0, 0.0)) * 2
    q[m] = np.stack([(r[m, 2, 1] - r[m, 1, 2]) / s, (r[m, 0, 2] - r[m, 2, 0]) / s, (r[m, 1, 0] - r[m, 0, 1]) / s, s / 4], axis=1)
    m = k == 1
    s = np.sqrt(np.maximum(1.0 + r[m, 0, 0] - r[m, 1, 1] - r[m, 2, 2], 0.0)) * 2
    q[m] = np.stack([s / 4, (r[m, 0, 1] + r[m, 1, 0]) / s, (r[m, 0, 2] + r[m, 2, 0]) / s, (r[m, 2, 1] - r[m, 1, 2]) / s], axis=1)
    m = k == 2
    s = np.sqrt(np.maximum(1.0 + r[m, 1, 1] - r[m, 0, 0] - r[m, 2, 2], 0.0)) * 2
    q[m] = np.stack([(r[m, 0, 1] + r[m, 1, 0]) / s, s / 4, (r[m, 1, 2] + r[m, 2, 1]) / s, (r[m, 0, 2] - r[m, 2, 0]) / s], axis=1)
    m = k == 3
    s = np.sqrt(np.maximum(1.0 + r[m, 2, 2] - r[m, 0, 0] - r[m, 1, 1], 0.0)) * 2
    q[m] = np.stack([(r[m, 0, 2] + r[m, 2, 0]) / s, (r[m, 1, 2] + r[m, 2, 1]) / s, s / 4, (r[m, 1, 0] - r[m, 0, 1]) / s], axis=1)
    q /= np.linalg.norm(q, axis=1, keepdims=True)
    q[q[:, 3] < 0] *= -1.0
    return q


def azimuth_deg(r):
    """Heading of a rotation into an ENU frame: the azimuth (from east, counter-clockwise) of its first column."""
    return math.degrees(math.atan2(r[1, 0], r[0, 0]))


def tilt_deg(r):
    """Angle between R e_z and e_z."""
    return math.degrees(math.acos(max(-1.0, min(1.0, float(r[2, 2])))))


def wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


# ---------------------------------------------------------------------------------------------------------------
def derive_tnw(w, n):
    """T_{N<-W} at every stamp present in both traj_W (w) and traj_N (n) (sanitised TUM arrays, sorted, unique stamps).
    -> (stamps (k,), R_NW (k,3,3), t_NW (k,3)); k = 0 when they share no stamp."""
    common, iw, in_ = np.intersect1d(w[:, 0], n[:, 0], assume_unique=True, return_indices=True)
    r_w = quat_to_rot(w[iw, 4:8])
    r_n = quat_to_rot(n[in_, 4:8])
    r_nw = r_n @ np.transpose(r_w, (0, 2, 1))
    t_nw = n[in_, 1:4] - np.einsum("kij,kj->ki", r_nw, w[iw, 1:4])
    return common, r_nw, t_nw


def assemble_e1(w, n):
    """A52 E1 trajectory of a GNSS run from its sanitised traj_W (w) and traj_N (n; may be empty). -> (e1, info).
    Raises ValueError when traj_N exists but shares no step-end stamp with traj_W (T_{N<-W} not derivable)."""
    info = {"activated": False, "t_act": None, "t_ref_act": None, "rows": int(w.shape[0]), "rows_traj_n": 0,
            "rows_fill_pre": 0, "rows_fill_gap": 0, "rows_traj_n_only": 0, "gaps": [], "gap_s": 0.0, "tnw_act": None,
            "tnw_orthonormality_err": None}
    if n.shape[0] == 0:
        info.update(rows_fill_pre=int(w.shape[0]))
        return w.copy(), info
    stamps, r_nw, t_nw = derive_tnw(w, n)
    if stamps.size == 0:
        raise ValueError("traj_N and traj_W share no step-end stamp: T_{N<-W} cannot be derived (A52)")
    t_act = float(n[0, 0])
    has_n = np.isin(w[:, 0], n[:, 0])
    k = np.searchsorted(stamps, w[:, 0], side="right") - 1          # latest derivable T_{N<-W} at or before t
    pre = (~has_n) & (w[:, 0] < t_act)
    gap = (~has_n) & (w[:, 0] >= t_act)
    k = np.where(k < 0, 0, k)                                       # before the first one: the activation estimate
    e1 = np.empty_like(w)
    e1[:, 0] = w[:, 0]
    fill = ~has_n
    if fill.any():
        r_w = quat_to_rot(w[fill, 4:8])
        rn = r_nw[k[fill]]
        e1[fill, 1:4] = np.einsum("kij,kj->ki", rn, w[fill, 1:4]) + t_nw[k[fill]]
        e1[fill, 4:8] = rot_to_quat(rn @ r_w)
    idx_n = np.searchsorted(n[:, 0], w[has_n, 0])
    e1[has_n, 1:8] = n[idx_n, 1:8]
    n_only = n[~np.isin(n[:, 0], w[:, 0])]
    if n_only.shape[0]:
        e1 = np.vstack([e1, n_only])
        e1 = e1[np.argsort(e1[:, 0], kind="stable")]
    # contiguous post-activation stretches without a global output (listed, first 20)
    gaps, g_s = [], 0.0
    tg = w[gap, 0]
    if tg.size:
        pos = np.where(gap)[0]
        breaks = np.where(np.diff(pos) > 1)[0]
        starts = np.concatenate([[0], breaks + 1])
        ends = np.concatenate([breaks, [pos.size - 1]])
        for a, b in zip(starts, ends):
            t0, t1 = float(w[pos[a], 0]), float(w[pos[b], 0])
            g_s += t1 - t0
            if len(gaps) < 20:
                gaps.append([t0, t1, int(b - a + 1)])
    orth = float(np.max(np.abs(r_nw @ np.transpose(r_nw, (0, 2, 1)) - np.eye(3))))
    r0 = r_nw[0]
    info.update(activated=True, t_act=t_act, t_ref_act=float(stamps[0]), rows=int(e1.shape[0]), rows_traj_n=int(has_n.sum()),
                rows_fill_pre=int(pre.sum()), rows_fill_gap=int(gap.sum()), rows_traj_n_only=int(n_only.shape[0]), gaps=gaps,
                gap_s=g_s, tnw_orthonormality_err=orth,
                tnw_act={"azimuth_deg": azimuth_deg(r0), "tilt_deg": tilt_deg(r0), "t_m": [float(x) for x in t_nw[0]],
                         "R": [[float(x) for x in row] for row in r0]})
    return e1, info


# ---------------------------------------------------------------------------------------------------------------
def heading_check(epochs_csv, t_ref, r_nw, tol_deg=HEADING_TOL_DEG):
    """|azimuth(M) - psi| at the activation step: psi = gnss_epochs.csv `yaw` of the last epoch row processed at or
    before t_ref with a registration (yaw_sigma > 0; the activation row itself still says phase COLLECT). Tolerance
    tol_deg + tilt^2/2 (the exact bound of the yaw of a tilt about a horizontal axis is tilt^2/4)."""
    if not os.path.isfile(epochs_csv):
        return {"status": "skipped: no gnss_epochs.csv"}
    best = None
    with open(epochs_csv, newline="") as f:
        rd = csv.reader(f)
        try:
            hdr = next(rd)
            i_tk, i_y, i_ys = hdr.index("t_k"), hdr.index("yaw"), hdr.index("yaw_sigma")
        except (StopIteration, ValueError):
            return {"status": "skipped: gnss_epochs.csv has no t_k/yaw/yaw_sigma columns"}
        for row in rd:
            try:
                tk, ys = float(row[i_tk]), float(row[i_ys])
            except (ValueError, IndexError):
                continue
            if tk <= t_ref + 1e-6 and math.isfinite(ys) and ys > 0.0:
                best = (tk, float(row[i_y]), ys)
    if best is None:
        return {"status": "skipped: no gnss_epochs.csv row with a registration at or before the activation step"}
    az, tl = azimuth_deg(r_nw), tilt_deg(r_nw)
    tol = tol_deg + math.degrees(0.5 * math.radians(tl) ** 2)
    d = wrap_deg(az - math.degrees(best[1]))
    return {"status": "ok" if abs(d) <= tol else "MISMATCH", "epoch_t_k": best[0], "psi_deg": math.degrees(best[1]),
            "psi_sigma_deg": math.degrees(best[2]), "azimuth_deg": az, "tilt_deg": tl, "diff_deg": d, "tol_deg": tol}


def read_events(path):
    """events.log -> [(kind, t_k)] (DESIGN §15; `t_k=... t_est=... gpst=W:TOW KIND magnitude=... detail=...`)."""
    out = []
    if not os.path.isfile(path):
        return None
    with open(path, errors="replace") as f:
        for ln in f:
            m = _EVENT.match(ln)
            if m:
                out.append((m.group(2), float(m.group(1))))
    return out


def activation_event_check(events, t_first_n):
    """The first ACTIVATE of events.log against the first traj_N stamp (None = never activated)."""
    if events is None:
        return {"status": "skipped: no events.log"}
    acts = [t for k, t in events if k == "ACTIVATE"]
    res = {"n_activate": len(acts), "n_reinit": sum(1 for k, _ in events if k == "REINIT"),
           "first_activate_t_k": acts[0] if acts else None, "first_traj_n": t_first_n}
    if t_first_n is None:
        res["status"] = "ok" if not acts else "MISMATCH: ACTIVATE in events.log but no traj_N row"
    elif not acts:
        res["status"] = "MISMATCH: traj_N rows without an ACTIVATE event"
    else:
        lag = t_first_n - acts[0]
        res["lag_s"] = lag
        res["status"] = "ok" if -1e-6 <= lag <= ACTIVATE_MAX_LAG_S else "MISMATCH: first traj_N %.3f s after the first ACTIVATE" % lag
    return res
