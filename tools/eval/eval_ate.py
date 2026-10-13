#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""ATE of the replay matrices (TEST_PLAN §4 E1/E4; AMENDMENTS A46, A52, A54, A55; P3-D, P4-EVAL).

    eval_ate.py --matrix runs/eval/<name> [--matrix ...] [--out DIR] [--span sequence|seed] [--evaluation-phase]

Metric: `evo_ape tum <GT> <est> -a --t_max_diff 0.05 --t_start T0 --t_end T1` = position ATE RMSE after an SE(3)
Umeyama alignment without scale (as M3DGR and FAST-LIVGO report); GT/estimate pairs associated within 50 ms.
Per run, three values (A52):
  * e1 (main table) — the whole sequence. U: traj_W. GNSS arms: the E1 trajectory of e1traj.py = traj_N from ACTIVATE
    on and, before it, the host poses mapped into N with the activation-time T_{N<-W} (one estimate, taken at
    activation = pure LIVO until activation); a GNSS run that never activated = its traj_W. Span [T0, T1] = GT ∩ every
    run of the sequence (all arms and seeds; --span seed: per seed).
  * post30 (supplementary) — the same trajectories on [latest activation of the span group + 30 s, T1] (the
    registration's convergence transient left out, A50-4); U is scored on the same span; GNSS runs that never activated
    have no value.
  * e4 (supplementary) — traj_W of every arm on the e1 span (the host: does GNSS change it?).
  * e1o (AMENDMENTS A61 F13) — e1 built from the odometry output traj_N_odom.tum instead of traj_N.tum (the
    registration's later revision of the activation instant left out: a frame fixed at activation, as FAST-LIVGO's is,
    with every drift correction since); same span as e1; U: its e1; a run older than A61 (no traj_N_odom.tum) has none.
  * e1p / e1op (TEST_PLAN §4 E1', AMENDMENTS A64; diagnostic columns, the main table stays e1/e1o) — the same file,
    span and association as e1 / e1o, but the GT antenna's lever arm l (3 parameters, IMU frame) is estimated jointly
    with the SE(3) alignment: min over (R, t, l) of Σ |R (p_i + R_i l) + t − y_i|² (p_i, R_i the estimate's IMU pose,
    y_i the RTK antenna), Levenberg–Marquardt from l = 0; every run and arm by the same procedure. The protocol E1
    compares the IMU trajectory with the antenna directly (as M3DGR/FAST-LIVGO do), so the unknown lever arm's
    horizontal part, turning with the heading, is a floor no alignment removes (A63: most of U's E1 on the development
    set). Self-checks (INSTRUMENT on failure): the fit's l = 0 start reproduces the numpy e1 (1e-6); E1' ≤ that value;
    a second start (LEVER_RESTART_M) reaches the same minimum (relative 1e-6); LM converged. Reported with l, its
    formal σ (s² (JᵀJ)⁻¹; the vertical part is only weakly observable, from roll/pitch) and the ATE at a fixed
    reference lever arm (A63's A58 value; since A76 always, see below).
  * e1c (AMENDMENTS A76 decision 2; main criterion of the final comparison, e1o stays reported) — the ATE of the e1o
    trajectory (U: traj_W) with its IMU point moved to the ground-truth antenna by the calibrated lever arm
    l (IMU frame): x_i = p_i + R_i l, then the same association, span, mask and SE(3) Umeyama without scale. l is not fitted:
    it is the host LiDAR's `gt_lever_imu` of tools/eval/lidars.json (Avia: A58's -0.176 -0.016 0.401; MID-360: A77's
    development-set fit on that host's own runs, tools/calib/host_gt_lever.py) unless --lever-ref gives one. It is the
    `rmse_at_lever_ref` of the e1op record, surfaced as a metric of its own.
--no-mask (A81): the ATE mask is not applied: GT and estimates are scored over the whole sequence; ate.json says so (metric.no_mask,
the sequences' mask record `not_applied` = what the registered mask would have removed) and, without --out, the output goes to
<first matrix>/eval_nomask. path_ratio (A81 section 3 item 3, every run and cell): the estimated path length over the
ground-truth path length at the GT times (rows thinned to <= 1 Hz, associated within --t-max-diff, only segments both sides have;
no alignment involved), of the e1 trajectory on the e1 span; the cell lists the seeds outside [0.95, 1.05] (the divergence rule's
second condition; `complete` != 1 is the first, run_matrix's).
--lidar (default auto): which host LiDAR the matrices ran on. auto = the `lidar` key of their manifests (run_matrix.py
--lidar; absent = Avia, the pre-A76 matrices); two matrices of different lidars in one call, or a --lidar that contradicts a
manifest, are refused. It only selects the default lever arm of the lever-ref / e1c scoring and is recorded in ate.json.
GT (A54, gtbase.py): gt_clean.tum shifted by gt_timebase.yaml's time_shift_applied_s (method p4 | record_time | none);
the ATE mask (trust_mask.csv column ate_exclude) removes masked stretches from GT and estimates; the masked seconds inside
every span are reported. Pre-A54 audits: a P4-passed one is used without a mask (flagged), a P4-failed one is blocked.
Instrument self-checks (a failure marks the run INSTRUMENT and the tool exits 4):
  * every ATE is recomputed with numpy (evo's association and Umeyama re-implemented), relative tolerance 1e-6;
  * GNSS runs: the heading of the activation-time T_{N<-W} against the filter's psi (gnss_epochs.csv), and the first
    ACTIVATE of events.log at the first traj_N stamp (e1traj.py).
Protocol checks (reported, † in the table): the traj_W of a GNSS run before its activation (all of it when it never
activated) must be bit-identical to U's of the same seed (TEST_PLAN I11 second half; A55: one binary for every arm),
and U and the GNSS arms of a sequence must share build_sha256 (A55). Seeds whose e1 trajectories are identical inside
the span are grouped (A30 seed collapse, ‡).
Output (--out, default <first matrix>/eval): ate.json (everything + provenance), ate_runs.csv, ate_cells.csv, ate_path_runs.csv, ate_path_cells.csv (A81 path ratio); per run
est_e1.tum / est_e4.tum (masked: exactly what both instruments read), ape_<metric>.zip and evo_<metric>.log; per
sequence gt_eval.tum (masked GT on the evaluation clock).
evo: the image's evo 1.31.1 cannot import scipy (apt scipy 1.3.3 vs pip numpy 1.24); tools/eval/setup_evo_env.sh
installs scipy 1.10.1 for Python 3.8 into <root>/tools/eval_pylib/site, prepended to PYTHONPATH for evo only.
A2: evaluation-set sequences are scored only with --evaluation-phase (nothing of theirs is read before the guard).
Runs inside the project image (tools/dev/remote_tool.sh ... -- python3 tools/eval/eval_ate.py ...).
"""
import argparse
import csv
import io
import json
import math
import os
import shlex
import subprocess
import sys
import zipfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402

import e1traj  # noqa: E402
import evalcommon as ec  # noqa: E402
import gtbase  # noqa: E402

REL_TOL = 1e-6
HOST_TRAJ = "traj_W.tum"
POST_ACT_S = 30.0
METRICS = ("e1", "post30", "e4", "e1o", "e1p", "e1op", "e1c")
LEVER_METRICS = {"e1p": "e1", "e1op": "e1o"}       # E1' (TEST_PLAN §4) of the e1 / e1o trajectory files
ODOM_TRAJ = {"traj_N.tum": "traj_N_odom.tum"}   # A61 F13: the odometry output next to a GNSS arm's scored trajectory
OK = ("ok", "ok (numpy only)")
LEVER_MAX_ITER = 200
LEVER_RESTART_M = (0.3, -0.3, 0.3)              # second start of the E1' fit: the minimum must not depend on the start
LEVER_RESTART_TOL = 1e-6                        # relative agreement of the two starts' RMSE
LEVER_EIG_REL = 1e-10                           # E1' fit: Jacobi-scaled normal-matrix eigenvalues below this x the largest = null
# A81 section 3 item 3 (divergence): the estimated path over the ground-truth path at the GT times (<= 1 Hz) must stay inside this band
PATH_RATIO_BAND = (0.95, 1.05)
PATH_GT_STEP_S = 1.0                            # GT rows used at most this often (a 5 Hz GT is thinned to its 1 Hz rows)
PATH_STEP_TOL_S = 1e-3                          # a row 1 s - 1 ms after the last kept one is the next 1 Hz row
PATH_MAX_GAP_S = 1.5                            # two consecutive kept GT rows further apart than this (a masked or missing stretch) form no segment


# ---------------------------------------------------------------------------------------------------------------
# evo re-implementation (evo 1.31.1: core/sync.py matching_time_indices + associate_trajectories, core/geometry.py
# umeyama_alignment, main_ape.py: only the reference is cut to [t_start, t_end]; metrics.APE trans_part)
def evo_associate(ref_t, est_t, max_diff):
    """-> (ref indices, est indices). The shorter trajectory is scanned; ties/duplicates take the first minimum."""
    est_longer = len(est_t) > len(ref_t)
    short, long_ = (ref_t, est_t) if est_longer else (est_t, ref_t)
    i_s, i_l = [], []
    for i, t in enumerate(short):
        d = np.abs(long_ - t)
        k = int(np.argmin(d))
        if d[k] <= max_diff:
            i_s.append(i)
            i_l.append(k)
    return (i_s, i_l) if est_longer else (i_l, i_s)


def umeyama_se3(x, y):
    """R, t minimising sum |R x_i + t - y_i|^2 (x, y: n x 3), evo's formulation without scale."""
    mx, my = x.mean(axis=0), y.mean(axis=0)
    cov = (y - my).T @ (x - mx) / x.shape[0]
    u, d, vt = np.linalg.svd(cov)
    if np.count_nonzero(d > np.finfo(d.dtype).eps) < 2:
        raise ValueError("degenerate covariance rank, Umeyama alignment is not possible")
    s = np.eye(3)
    if np.linalg.det(u) * np.linalg.det(vt) < 0.0:
        s[2, 2] = -1.0
    r = u @ s @ vt
    return r, my - r @ mx


def ate_numpy(gt, est, t0, t1, max_diff=ec.T_MAX_DIFF_S):
    ref = gt[(gt[:, 0] >= t0) & (gt[:, 0] <= t1)]
    if ref.shape[0] == 0 or est.shape[0] == 0:
        raise ValueError("empty trajectory in the span")
    ir, ie = evo_associate(ref[:, 0], est[:, 0], max_diff)
    if not ir:
        raise ValueError("no GT/estimate pairs within %.3f s" % max_diff)
    y, x = ref[ir, 1:4], est[ie, 1:4]
    r, t = umeyama_se3(x, y)
    e = np.linalg.norm(x @ r.T + t - y, axis=1)
    return {"rmse": float(np.sqrt(np.mean(e ** 2))), "mean": float(np.mean(e)), "median": float(np.median(e)),
            "std": float(np.std(e)), "min": float(np.min(e)), "max": float(np.max(e)), "n_pairs": int(len(ir)),
            "n_ref_in_span": int(ref.shape[0])}


def path_ratio(gt, est, t0, t1, max_diff=ec.T_MAX_DIFF_S):
    """A81 section 3 item 3: estimated path length over ground-truth path length at the GT times (rigid-motion invariant, so
    no alignment is involved). The GT rows of [t0, t1] are thinned to <= 1 Hz; each takes the nearest estimate stamp within
    max_diff (the ATE's association); a segment between two consecutive kept rows counts for both paths only when both rows
    have an estimate and the rows are at most PATH_MAX_GAP_S apart (a masked or missing stretch is no segment on either side).
    -> dict(ratio, est_len_m, gt_len_m, n_segments, n_rows, n_segments_skipped); ratio None without a ground-truth length."""
    ref = gt[(gt[:, 0] >= t0) & (gt[:, 0] <= t1)]
    if ref.shape[0] == 0 or est.shape[0] == 0:
        raise ValueError("empty trajectory in the span")
    keep = [0]
    for i in range(1, ref.shape[0]):
        if ref[i, 0] - ref[keep[-1], 0] >= PATH_GT_STEP_S - PATH_STEP_TOL_S:
            keep.append(i)
    ref = ref[keep]
    ir, ie = evo_associate(ref[:, 0], est[:, 0], max_diff)
    pe = np.full((ref.shape[0], 3), np.nan)
    pe[ir] = est[ie, 1:4]
    seg = (np.diff(ref[:, 0]) <= PATH_MAX_GAP_S) & np.all(np.isfinite(pe[:-1]), axis=1) & np.all(np.isfinite(pe[1:]), axis=1)
    gt_len = float(np.sum(np.linalg.norm(np.diff(ref[:, 1:4], axis=0)[seg], axis=1)))
    est_len = float(np.sum(np.linalg.norm(np.diff(pe, axis=0)[seg], axis=1)))
    return {"ratio": est_len / gt_len if gt_len > 0.0 else None, "est_len_m": est_len, "gt_len_m": gt_len,
            "n_segments": int(np.sum(seg)), "n_rows": int(ref.shape[0]), "n_segments_skipped": int(seg.size - np.sum(seg))}


def path_band_flag(ratio):
    """True when the ratio is outside A81's [0.95, 1.05] (or absent): the divergence rule's second condition."""
    return ratio is None or not (PATH_RATIO_BAND[0] <= ratio <= PATH_RATIO_BAND[1])


def disable_mask(mask):
    """--no-mask: a Mask that removes nothing; the record keeps what the registered mask would have removed."""
    rec = mask.record()
    prov = {k: v for k, v in mask.prov.items() if k in ("file", "column", "sha256")}     # which file; what it removes is under not_applied
    return gtbase.Mask(status="not applied (--no-mask: the whole sequence is scored; registered mask: %s)" % mask.status,
                       not_applied=rec, **prov)


# ---------------------------------------------------------------------------------------------------------------
# E1' (TEST_PLAN §4): the GT antenna lever arm estimated jointly with the SE(3) alignment
def _skew(v):
    """[v]x of every row of v (..., 3) -> (..., 3, 3)."""
    v = np.asarray(v, dtype=float)
    z = np.zeros(v.shape[:-1])
    return np.stack([np.stack([z, -v[..., 2], v[..., 1]], -1),
                     np.stack([v[..., 2], z, -v[..., 0]], -1),
                     np.stack([-v[..., 1], v[..., 0], z], -1)], -2)


def _exp_so3(w):
    th = float(np.linalg.norm(w))
    k = _skew(w)
    if th < 1e-8:
        return np.eye(3) + k + 0.5 * (k @ k)
    return np.eye(3) + math.sin(th) / th * k + (1.0 - math.cos(th)) / th ** 2 * (k @ k)


def lever_fit(p, rb, y, l0, max_iter=LEVER_MAX_ITER):
    """min over (R, t, l) of Σ_i |R (p_i + Rb_i l − c) + t − y_i|² by Levenberg–Marquardt (c = centroid of p, which only
    re-parametrises t; right perturbation R Exp(δθ): ∂r/∂δθ = −R [x]x, ∂r/∂t = I, ∂r/∂l = R Rb_i), started from l0 with
    the Umeyama alignment at l0. The damped normal equations are solved in the Jacobi-scaled eigenbasis, and directions
    whose scaled eigenvalue is below LEVER_EIG_REL of the largest are left where they are: with constant roll/pitch the
    vertical lever is t's z exactly (a null direction a plain solve walks along on rounding noise); a merely weak
    direction (roll/pitch that varies) is kept and estimated.
    -> dict(R, t, l, cost, iterations, converged, normal (at the solution), rank, null (its null directions))."""
    c = p.mean(axis=0)
    l = np.asarray(l0, dtype=float).copy()

    def resid(rr, tt, ll):
        x = p - c + rb @ ll
        return x @ rr.T + tt - y, x

    def normal(rr, x):
        J[:, :, 0:3] = -(rr @ _skew(x))
        J[:, :, 3:6] = np.eye(3)
        J[:, :, 6:9] = rr @ rb
        return np.einsum("nki,nkj->ij", J, J)

    def eig_scaled(A):
        ds = np.sqrt(np.diag(A))
        ds[ds <= 0.0] = 1.0
        w, v = np.linalg.eigh(A / np.outer(ds, ds))
        return ds, w, v, w > LEVER_EIG_REL * max(w[-1], 1e-300)

    x = p - c + rb @ l
    R, t = umeyama_se3(x, y)
    r, x = resid(R, t, l)
    cost = float(np.einsum("ij,ij->", r, r))
    n = p.shape[0]
    J = np.empty((n, 3, 9))
    lam, converged, it = 1e-4, False, 0
    for it in range(1, max_iter + 1):
        A = normal(R, x)
        g = np.einsum("nki,nk->i", J, r)
        ds, w, v, keep = eig_scaled(A)
        gk = v[:, keep].T @ (g / ds)
        step, dc = None, 0.0
        while lam < 1e16:
            d = -(v[:, keep] @ (gk / (w[keep] + lam))) / ds
            R2, t2, l2 = R @ _exp_so3(d[0:3]), t + d[3:6], l + d[6:9]
            r2, x2 = resid(R2, t2, l2)
            c2 = float(np.einsum("ij,ij->", r2, r2))
            if c2 <= cost:
                step, dc = d, cost - c2
                R, t, l, r, x, cost = R2, t2, l2, r2, x2, c2
                lam = max(lam * 0.1, 1e-12)
                break
            lam *= 10.0
        if step is None or dc <= 1e-13 * cost or float(np.linalg.norm(step)) < 1e-12:
            converged = True                   # no descent left (stationary to rounding), or a negligible last step
            break
    A = normal(R, x)
    ds, w, v, keep = eig_scaled(A)
    return {"R": R, "t": t - R @ c, "l": l, "cost": cost, "iterations": it, "converged": converged, "normal": A,
            "rank": int(np.count_nonzero(keep)), "scaled_eig": (ds, w, v, keep)}


def ate_lever_numpy(gt, est, t0, t1, max_diff=ec.T_MAX_DIFF_S, lever_ref=None):
    """E1' of one trajectory on [t0, t1]: evo's association (as ate_numpy), then lever_fit from l = 0 and from
    LEVER_RESTART_M. -> dict(rmse, lever_m, lever_sigma_m, rmse_lever0 (= the plain ATE), rmse_restart, ...)."""
    ref = gt[(gt[:, 0] >= t0) & (gt[:, 0] <= t1)]
    if ref.shape[0] == 0 or est.shape[0] == 0:
        raise ValueError("empty trajectory in the span")
    ir, ie = evo_associate(ref[:, 0], est[:, 0], max_diff)
    if len(ir) < 4:
        raise ValueError("fewer than 4 GT/estimate pairs within %.3f s" % max_diff)
    y, p = ref[ir, 1:4], est[ie, 1:4]
    rb = e1traj.quat_to_rot(est[ie, 4:8])
    n = len(ir)
    r0, tt0 = umeyama_se3(p, y)                      # l = 0: exactly ate_numpy's alignment
    rmse0 = float(np.sqrt(np.mean(np.sum((p @ r0.T + tt0 - y) ** 2, axis=1))))
    fit = lever_fit(p, rb, y, np.zeros(3))
    alt = lever_fit(p, rb, y, np.array(LEVER_RESTART_M))
    rmse = float(np.sqrt(fit["cost"] / n))
    sig = [None, None, None]
    ds, w, v, keep = fit["scaled_eig"]
    dof = 3 * n - fit["rank"]
    if dof > 0:
        # formal σ = sqrt(diag(s² A⁺)) in the observable subspace; a lever component with a real share in a null
        # direction (> 0.1 of the unit eigenvector) has none
        cov = ((v[:, keep] / w[keep]) @ v[:, keep].T) / np.outer(ds, ds) * (fit["cost"] / dof)
        nul = v[:, ~keep]
        for i in range(3):
            if nul.shape[1] == 0 or float(np.linalg.norm(nul[6 + i, :])) <= 0.1:
                sig[i] = float(math.sqrt(max(cov[6 + i, 6 + i], 0.0)))
    e = np.linalg.norm((p + rb @ fit["l"]) @ fit["R"].T + fit["t"] - y, axis=1)
    out = {"rmse": rmse, "median": float(np.median(e)), "max": float(np.max(e)), "n_pairs": int(n),
           "n_ref_in_span": int(ref.shape[0]), "lever_m": [float(v) for v in fit["l"]], "lever_sigma_m": sig,
           "iterations": fit["iterations"], "converged": bool(fit["converged"] and alt["converged"]), "rank": fit["rank"],
           "rmse_lever0": rmse0, "rmse_restart": float(np.sqrt(alt["cost"] / n)),
           "lever_restart_m": [float(v) for v in alt["l"]]}
    if lever_ref is not None:
        x = p + rb @ np.asarray(lever_ref, dtype=float)
        rr, tr = umeyama_se3(x, y)
        out["lever_ref_m"] = [float(v) for v in lever_ref]
        out["rmse_at_lever_ref"] = float(np.sqrt(np.mean(np.sum((x @ rr.T + tr - y) ** 2, axis=1))))
    return out


def lever_problems(res, e1_numpy_rmse):
    """The E1' self-checks -> list of failures (empty = pass)."""
    bad = []
    if e1_numpy_rmse is not None and abs(res["rmse_lever0"] - e1_numpy_rmse) > REL_TOL * max(1.0, e1_numpy_rmse):
        bad.append("l = 0 start %.9f differs from the numpy ATE %.9f" % (res["rmse_lever0"], e1_numpy_rmse))
    if res["rmse"] > res["rmse_lever0"] * (1.0 + 1e-12) + 1e-12:
        bad.append("E1' %.9f above its l = 0 value %.9f" % (res["rmse"], res["rmse_lever0"]))
    if abs(res["rmse_restart"] - res["rmse"]) > LEVER_RESTART_TOL * max(res["rmse"], 1e-3):
        bad.append("second start reaches %.9f, not %.9f" % (res["rmse_restart"], res["rmse"]))
    if not res["converged"]:
        bad.append("Levenberg-Marquardt did not converge in %d iterations" % LEVER_MAX_ITER)
    return bad


# ---------------------------------------------------------------------------------------------------------------
def load_gt(root, seq, info, override=None):
    """-> (GT on the evaluation clock, provenance, gtbase.Mask) or raise ec.Blocked (A54; see gtbase.py)."""
    return gtbase.load_gt(root, seq, info, override)


def evo_available(pylib):
    return pylib and os.path.isdir(pylib)


def run_evo(evo_bin, pylib, gt_path, est_path, t0, t1, zip_path, log_path, max_diff):
    cmd = [evo_bin, "tum", gt_path, est_path, "-a", "--t_max_diff", repr(float(max_diff)), "--t_start", repr(float(t0)),
           "--t_end", repr(float(t1)),
           "--save_results", zip_path, "--no_warnings"]
    env = dict(os.environ)
    env["PYTHONPATH"] = pylib + ((":" + env["PYTHONPATH"]) if env.get("PYTHONPATH") else "")
    env.setdefault("MPLBACKEND", "Agg")
    if os.path.exists(zip_path):
        os.remove(zip_path)
    r = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True, env=env)
    with open(log_path, "w") as f:
        f.write("$ " + " ".join(shlex.quote(c) for c in cmd) + "\n" + r.stdout)
    if r.returncode != 0 or not os.path.isfile(zip_path):
        raise RuntimeError("evo_ape failed (rc %d): %s" % (r.returncode, r.stdout[-600:]))
    with zipfile.ZipFile(zip_path) as z:
        names = z.namelist()
        stats = json.loads(z.read([n for n in names if n.endswith("stats.json")][0]))
        n_pairs = None
        err = [n for n in names if n.endswith("error_array.npy")]
        if err:
            n_pairs = int(np.load(io.BytesIO(z.read(err[0]))).shape[0])
    return {k: float(v) for k, v in stats.items()}, n_pairs, cmd


def evo_version():
    try:
        from importlib import metadata
        return metadata.version("evo")
    except Exception:  # noqa: BLE001
        return None


# ---------------------------------------------------------------------------------------------------------------
def collect_runs(root, matrices, arms_filter, seqs_filter):
    """Complete runs of the matrices -> list of dicts; incomplete/failed jobs are kept as rows without a value."""
    runs, arm_traj, provenance, seen = [], {}, [], {}
    for m in matrices:
        mdir = m if os.path.isabs(m) else os.path.join(root, m)
        man = ec.load_json(os.path.join(mdir, "matrix_manifest.json"))
        provenance.append({"matrix": os.path.relpath(mdir, root), "name": man["name"], "lidar": man.get("lidar"),
                           "source_id": man.get("source_id"),
                           "image_digest": man.get("image_digest"), "arms": man.get("arms"), "seeds": man.get("seeds"),
                           "counts": man.get("counts"), "updated_utc": man.get("updated_utc")})
        for arm, ai in man["arms"].items():
            if arm in arm_traj and arm_traj[arm] != ai["traj"]:
                raise ValueError("arm %s has two trajectory files across matrices" % arm)
            arm_traj[arm] = ai["traj"]
        for r in man["runs"]:
            if (arms_filter and r["arm"] not in arms_filter) or (seqs_filter and r["seq"] not in seqs_filter):
                continue
            key = (r["seq"], r["arm"], r["seed"])
            if key in seen:
                raise ValueError("%s/%s/s%d is in two matrices (%s, %s): evaluate them separately" % (key + (seen[key], man["name"])))
            seen[key] = man["name"]
            ok = [a for a in r["attempts"] if a.get("complete")]
            row = {"matrix": man["name"], "seq": r["seq"], "arm": r["arm"], "seed": r["seed"], "state": r["state"],
                   "attempts": len([a for a in r["attempts"] if a.get("attempt", 0) > 0]),
                   "run_dir": ok[-1]["run_dir"] if ok else None, "traj": man["arms"][r["arm"]]["traj"],
                   "build_sha256": ok[-1].get("build_sha256") if ok else None,
                   "config_sha256": ok[-1].get("config_sha256") if ok else None}
            runs.append(row)
    return runs, arm_traj, provenance


def resolve_lidar(provenance, requested):
    """-> (lidar profile, lever arm used, source of the lever arm) from the matrices' manifests and the command line.
    A matrix without a `lidar` key was made before A76 and ran on the Avia."""
    default = ec.load_lidars()[0]
    seen = sorted({p.get("lidar") or default for p in provenance})
    if requested and requested != "auto":
        if any(x != requested for x in seen):
            raise ec.Blocked("--lidar %s contradicts the manifest(s): %s (matrix %s)" % (
                requested, ", ".join(seen), ", ".join(p["name"] for p in provenance)))
        name = requested
    else:
        if len(seen) > 1:
            raise ec.Blocked("the matrices ran on different host LiDARs (%s): evaluate them separately" % ", ".join(seen))
        name = seen[0] if seen else default
    return ec.lidar_profile(name)


def stat_block(vals, extra=None):
    d = {"n_ok": len(vals), "median_m": float(np.median(vals)) if vals else None, "min_m": float(np.min(vals)) if vals else None,
         "max_m": float(np.max(vals)) if vals else None, "values_m": vals}
    d.update(extra or {})
    return d


def e1c_of(e1op):
    """The calibrated-lever ATE (A76) as a metric of its own, from the e1op record (whose lever result carries the ATE at
    the fixed reference lever arm); a failed or missing e1op gives no value."""
    lv = e1op.get("lever") or {}
    if e1op.get("status") not in OK or lv.get("rmse_at_lever_ref") is None:
        return {"value": None, "status": "no e1c (%s)" % e1op.get("status"), "span": e1op.get("span")}
    out = {"value": lv["rmse_at_lever_ref"], "status": e1op["status"], "lever_ref_m": lv.get("lever_ref_m"),
           "n_pairs": lv.get("n_pairs"), "span": e1op.get("span"), "from": "e1op.lever.rmse_at_lever_ref"}
    if e1op.get("same_as"):
        out["same_as"] = "e1c of " + e1op["same_as"]
    return out


class Scorer:
    """One ATE on a span, from files: numpy always, evo_ape unless --no-evo; -> dict (value, status, numpy, evo, cmd)."""

    def __init__(self, args, pylib, use_evo):
        self.a, self.pylib, self.use_evo = args, pylib, use_evo

    def __call__(self, gt, gt_path, est_path, t0, t1, rd, tag):
        est = ec.read_tum(est_path)                 # both instruments see exactly the numbers in the file
        try:
            npres = ate_numpy(gt, est, t0, t1, self.a.t_max_diff)
        except ValueError as e:
            return {"value": None, "status": "ATE failed: %s" % e, "span": [float(t0), float(t1)]}
        out = {"value": npres["rmse"], "status": "ok (numpy only)", "numpy": npres, "n_pairs": npres["n_pairs"],
               "span": [float(t0), float(t1)]}
        if not self.use_evo:
            return out
        try:
            stats, n_pairs, cmd = run_evo(self.a.evo, self.pylib, gt_path, est_path, t0, t1,
                                          os.path.join(rd, "ape_%s.zip" % tag), os.path.join(rd, "evo_%s.log" % tag), self.a.t_max_diff)
        except RuntimeError as e:
            out.update(value=None, status="INSTRUMENT: %s" % e)
            return out
        agree = abs(stats["rmse"] - npres["rmse"]) <= REL_TOL * max(1.0, npres["rmse"]) and (n_pairs is None or n_pairs == npres["n_pairs"])
        out.update(value=stats["rmse"], evo=stats, evo_cmd=" ".join(shlex.quote(c) for c in cmd),
                   status="ok" if agree else "INSTRUMENT_MISMATCH evo %.9f vs numpy %.9f (pairs %s vs %d)" % (
                       stats["rmse"], npres["rmse"], n_pairs, npres["n_pairs"]))
        return out

    def lever(self, gt, est_path, t0, t1, base):
        """E1' of the file that `base` (the e1 or e1o result of the same file and span) scored; numpy only (evo has no
        such metric); the fit's l = 0 start must reproduce base's numpy value."""
        if base.get("value") is None and "numpy" not in base:
            return {"value": None, "status": "no E1' (%s)" % base.get("status"), "span": [float(t0), float(t1)]}
        est = ec.read_tum(est_path)
        try:
            res = ate_lever_numpy(gt, est, t0, t1, self.a.t_max_diff, self.a.lever_ref)
        except ValueError as e:
            return {"value": None, "status": "E1' failed: %s" % e, "span": [float(t0), float(t1)]}
        bad = lever_problems(res, (base.get("numpy") or {}).get("rmse"))
        return {"value": None if bad else res["rmse"], "status": ("INSTRUMENT: E1' " + "; ".join(bad)) if bad else "ok (numpy only)",
                "lever": res, "n_pairs": res["n_pairs"], "span": [float(t0), float(t1)]}


def span_groups(keys, mode):
    """-> list of lists of (arm, seed): one group per sequence, or one per seed."""
    if mode == "sequence":
        return [sorted(keys)] if keys else []
    return [sorted(k for k in keys if k[1] == s) for s in sorted({k[1] for k in keys})]


def load_run(root, r):
    """Trajectories of one complete run -> dict(w, e1, act, checks) or a status string. GNSS arm = its scored
    trajectory is not the host one (arms.json traj_N.tum)."""
    rd = os.path.join(root, r["run_dir"])
    wp = os.path.join(rd, HOST_TRAJ)
    if not os.path.isfile(wp) or os.path.getsize(wp) == 0:
        return "no %s" % HOST_TRAJ
    w, _, n_dup = ec.sanitize_tum(ec.read_tum(wp))
    if w.shape[0] < 2:
        return "%s has < 2 rows" % HOST_TRAJ
    r["est_rows_dup"] = n_dup
    if r["traj"] == HOST_TRAJ:
        return {"w": w, "e1": w, "act": None, "checks": {}}
    npth = os.path.join(rd, r["traj"])
    n = np.zeros((0, 8))
    if os.path.isfile(npth) and os.path.getsize(npth) > 0:
        n, _, _ = ec.sanitize_tum(ec.read_tum(npth))
    try:
        e1, act = e1traj.assemble_e1(w, n)
    except ValueError as e:
        return "INSTRUMENT: %s" % e
    # A61 F13: the same construction from the odometry output; a run that never activated is its host either way
    e1o = None
    op = os.path.join(rd, ODOM_TRAJ.get(r["traj"], ""))
    if r["traj"] in ODOM_TRAJ and os.path.isfile(op) and os.path.getsize(op) > 0:
        o, _, _ = ec.sanitize_tum(ec.read_tum(op))
        try:
            e1o, _ = e1traj.assemble_e1(w, o)
        except ValueError as e:
            return "INSTRUMENT: %s (%s)" % (e, ODOM_TRAJ[r["traj"]])
    elif n.shape[0] == 0:
        e1o = e1
    checks = {"activate_event": e1traj.activation_event_check(e1traj.read_events(os.path.join(rd, "events.log")),
                                                               act["t_act"])}
    if act["activated"]:
        checks["heading"] = e1traj.heading_check(os.path.join(rd, "gnss_epochs.csv"), act["t_ref_act"],
                                                 np.array(act["tnw_act"]["R"]))
        if act["tnw_orthonormality_err"] > 1e-6:
            checks["tnw_orthonormality"] = {"status": "MISMATCH: |R R^T - I| = %.2e" % act["tnw_orthonormality_err"]}
    return {"w": w, "e1": e1, "e1o": e1o, "act": act, "checks": checks}


def pre_activation_vs_u(v, u):
    """traj_W rows of a GNSS run before its activation (all rows if it never activated) vs U's of the same seed."""
    if u is None:
        return {"status": "no U run"}
    t_act = v["act"]["t_act"] if v["act"]["activated"] else None
    a = v["w"] if t_act is None else v["w"][v["w"][:, 0] < t_act]
    b = u["w"] if t_act is None else u["w"][u["w"][:, 0] < t_act]
    if a.shape == b.shape and np.array_equal(a, b):
        return {"status": "identical", "rows": int(a.shape[0])}
    n = min(a.shape[0], b.shape[0])
    diff = np.where(np.any(a[:n] != b[:n], axis=1))[0]
    at = float(a[diff[0], 0]) if diff.size else (float(a[n, 0]) if a.shape[0] > n else float(b[n, 0]))
    return {"status": "DIFFERS", "rows": [int(a.shape[0]), int(b.shape[0])], "first_difference_t": at}


def evaluate(args):
    root = ec.project_root(args.root)
    _, reg = ec.load_sequences()
    runs, arm_traj, provenance = collect_runs(root, args.matrix, set(ec.split_list(args.arms)) if args.arms else None,
                                              set(ec.split_list(args.seqs)) if args.seqs else None)
    lidar = resolve_lidar(provenance, args.lidar)
    if args.lever_ref:
        lever_ref, lever_src = list(args.lever_ref), "--lever-ref"
    else:
        lever_ref, lever_src = [float(x) for x in lidar["gt_lever_imu"]], "tools/eval/lidars.json: %s gt_lever_imu" % lidar["name"]
    args.lever_ref = lever_ref                  # Scorer.lever reads it
    # A81: the unmasked scoring of the same matrix goes to its own default directory, never over the masked one
    out_dir = args.out or os.path.join(root if not os.path.isabs(args.matrix[0]) else "", args.matrix[0],
                                       "eval_nomask" if args.no_mask else "eval")
    os.makedirs(out_dir, exist_ok=True)
    pylib = args.pylib or os.path.join(root, "tools", "eval_pylib", "site")
    use_evo = not args.no_evo
    if use_evo and not evo_available(pylib):
        raise ec.Blocked("evo needs scipy >= 1.5 next to the image's numpy 1.24: run tools/eval/setup_evo_env.sh "
                         "(expects %s) or pass --no-evo" % pylib)
    score = Scorer(args, pylib, use_evo)
    overrides = dict(x.split("=", 1) for x in args.gt_dir)
    seqs = sorted({r["seq"] for r in runs}, key=lambda s: list(reg).index(s) if s in reg else 999)
    result = {"tool": "tools/eval/eval_ate.py", "generated_utc": ec.utc_now(), "argv": sys.argv, "root": root,
              "metric": {"cmd": "evo_ape tum <gt_eval.tum> <est> -a --t_max_diff %g --t_start T0 --t_end T1" % args.t_max_diff,
                         "alignment": "SE(3) Umeyama, no scale", "pose_relation": "translation part", "t_max_diff_s": args.t_max_diff,
                         "span": args.span, "post_activation_s": POST_ACT_S, "u_arm": args.u_arm,
                         "values": {"e1": "whole sequence; GNSS arms: traj_N from ACTIVATE, before it traj_W mapped with the "
                                          "activation-time T_{N<-W} (A52); U: traj_W",
                                    "post30": "same trajectories on [latest activation + %g s, end] (A52 supplementary)" % POST_ACT_S,
                                    "e4": "traj_W of every arm on the e1 span (TEST_PLAN E4)",
                                    "e1o": "e1 from the odometry output traj_N_odom (AMENDMENTS A61 F13); U: its e1",
                                    "e1p": "E1' of the e1 trajectory: GT antenna lever arm (3 parameters, IMU frame) estimated "
                                           "jointly with the SE(3) alignment (TEST_PLAN §4, A64; diagnostic)",
                                    "e1op": "E1' of the e1o trajectory; U: its e1p"},
                         "lidar": lidar["name"], "lever_ref_m": list(lever_ref), "lever_ref_source": lever_src,
                         "e1c": "ATE of the e1o trajectory (U: traj_W) at the calibrated lever-ref point (A76 decision 2): "
                                "the rmse_at_lever_ref of e1op",
                         "gt": ("gt_clean.tum + gt_timebase.yaml shift, trust_mask.csv ate_exclude stretches removed (A54)"
                                if not args.no_mask else
                                "gt_clean.tum + gt_timebase.yaml shift; --no-mask: no trust_mask.csv stretch removed (A81), the "
                                "whole sequence is scored"),
                         "no_mask": bool(args.no_mask),
                         "path_ratio": {"definition": "estimated path / ground-truth path over the same segments between GT rows "
                                                      "thinned to <= %g Hz, rows associated within t_max_diff (A81 section 3 item 3); "
                                                      "of the e1 trajectory, on the e1 span" % (1.0 / PATH_GT_STEP_S),
                                        "band": list(PATH_RATIO_BAND), "max_gap_s": PATH_MAX_GAP_S},
                         "evo_version": evo_version() if use_evo else None, "evo_pylib": pylib if use_evo else None,
                         "self_check": "numpy re-implementation, relative tolerance %g; T_{N<-W} heading vs psi; ACTIVATE vs "
                                       "first traj_N" % REL_TOL},
              "matrices": provenance, "sequences": {}, "runs": [], "cells": [], "blocked": {}}
    bad_instrument = 0
    refused = []
    arm_order = list(ec.load_arms())
    for seq in seqs:
        info = reg.get(seq)
        try:
            ec.check_phase(seq, args.evaluation_phase, "scoring")
        except ec.PhaseError as e:
            refused.append(str(e))
            result["blocked"][seq] = str(e)
            continue
        try:
            gt, gprov, mask = load_gt(root, seq, info, overrides.get(seq))
        except ec.Blocked as e:
            result["blocked"][seq] = str(e)
            continue
        if args.no_mask:
            mask = disable_mask(mask)
        seq_runs = [r for r in runs if r["seq"] == seq]
        loaded = {}
        for r in seq_runs:
            r["gnss_arm"] = r["traj"] != HOST_TRAJ
            if not r["run_dir"]:
                r["status"] = "no complete run (%s)" % r["state"]
                continue
            v = load_run(root, r)
            if isinstance(v, str):
                r["status"] = v
                if v.startswith("INSTRUMENT"):
                    bad_instrument += 1
                continue
            if v["act"] is not None:
                r["activation"], r["checks"] = v["act"], v["checks"]
                r["activated"] = v["act"]["activated"]
            loaded[(r["arm"], r["seed"])] = v
        # protocol: pre-activation host of every GNSS run == U's of the same seed (A55, I11)
        for r in seq_runs:
            v = loaded.get((r["arm"], r["seed"]))
            if v is not None and v["act"] is not None:
                r["checks"]["pre_activation_vs_U"] = pre_activation_vs_u(v, loaded.get((args.u_arm, r["seed"])))
        # spans: e1 over each group, post30 from the latest activation in the group
        spans, post = {}, {}
        seq_spans, seq_post = [], []
        for g in span_groups(list(loaded), args.span):
            t0 = float(max([gt[0, 0]] + [loaded[k]["e1"][0, 0] for k in g]))
            t1 = float(min([gt[-1, 0]] + [loaded[k]["e1"][-1, 0] for k in g]))
            acts = [loaded[k]["act"]["t_act"] for k in g if loaded[k]["act"] is not None and loaded[k]["act"]["activated"]]
            p = (float(max(t0, max(acts) + POST_ACT_S)), t1) if acts else None
            for k in g:
                spans[k], post[k] = (t0, t1), p
            seq_spans.append({"span": [t0, t1], "length_s": t1 - t0, "masked_s": mask.seconds_in(t0, t1),
                              "masked_stretches": mask.stretches_in(t0, t1), "keys": [list(k) for k in g]})
            if p:
                seq_post.append({"span": list(p), "length_s": p[1] - p[0], "masked_s": mask.seconds_in(*p),
                                 "latest_activation": float(max(acts)), "keys": [list(k) for k in g]})
        sdir = os.path.join(out_dir, seq)
        os.makedirs(sdir, exist_ok=True)
        gt_eval = gt[~mask.inside(gt[:, 0])]
        gt_path = os.path.join(sdir, "gt_eval.tum")
        ec.write_tum(gt_path, gt_eval)
        gt_eval = ec.read_tum(gt_path)
        for r in seq_runs:
            k = (r["arm"], r["seed"])
            v = loaded.get(k)
            if v is None:
                continue
            t0, t1 = spans[k]
            r["e1_span"] = [t0, t1]
            if t1 - t0 < 1.0:
                r["status"] = "e1 span shorter than 1 s"
                continue
            rd = os.path.join(sdir, r["arm"], "s%d" % r["seed"])
            os.makedirs(rd, exist_ok=True)
            e1 = v["e1"][~mask.inside(v["e1"][:, 0])]
            p_e1 = os.path.join(rd, "est_e1.tum")
            ec.write_tum(p_e1, e1)
            e1f = ec.read_tum(p_e1)
            # seed-collapse check (hash only, A2-safe): the rows this run contributes inside the e1 span
            inside = e1f[(e1f[:, 0] >= t0 - args.t_max_diff) & (e1f[:, 0] <= t1 + args.t_max_diff)]
            r["span_rows_sha256"] = ec.sha256_text("".join("%.9f %.9f %.9f %.9f %.9f %.9f %.9f %.9f\n" % tuple(x) for x in inside))
            m = {"e1": score(gt_eval, gt_path, p_e1, t0, t1, rd, "e1")}
            try:                                                                    # A81: the divergence rule's path ratio
                r["path"] = path_ratio(gt_eval, e1f, t0, t1, args.t_max_diff)
            except ValueError as e:
                r["path"] = {"ratio": None, "error": str(e)}
            if r["gnss_arm"] and not v["act"]["activated"]:
                m["post30"] = {"value": None, "status": "never activated"}
            elif post[k] is None:
                m["post30"] = {"value": None, "status": "no activated GNSS run in the span group"}
            elif post[k][1] - post[k][0] < 1.0:
                m["post30"] = {"value": None, "status": "post-activation span shorter than 1 s", "span": list(post[k])}
            else:
                m["post30"] = score(gt_eval, gt_path, p_e1, post[k][0], post[k][1], rd, "post30")
            m["e1p"] = score.lever(gt_eval, p_e1, t0, t1, m["e1"])                  # E1' (TEST_PLAN §4, A64)
            if r["gnss_arm"]:
                p_e4 = os.path.join(rd, "est_e4.tum")
                ec.write_tum(p_e4, v["w"][~mask.inside(v["w"][:, 0])])
                m["e4"] = score(gt_eval, gt_path, p_e4, t0, t1, rd, "e4")
                if v.get("e1o") is None:
                    m["e1o"] = {"value": None, "status": "no %s (a run older than AMENDMENTS A61)" % ODOM_TRAJ.get(r["traj"], "odometry output")}
                    m["e1op"] = {"value": None, "status": "no e1o"}
                else:
                    p_e1o = os.path.join(rd, "est_e1o.tum")
                    ec.write_tum(p_e1o, v["e1o"][~mask.inside(v["e1o"][:, 0])])
                    m["e1o"] = score(gt_eval, gt_path, p_e1o, t0, t1, rd, "e1o")
                    m["e1op"] = score.lever(gt_eval, p_e1o, t0, t1, m["e1o"])
            else:
                m["e4"] = dict(m["e1"], same_as="e1")
                m["e1o"] = dict(m["e1"], same_as="e1")
                m["e1op"] = dict(m["e1p"], same_as="e1p")
            m["e1c"] = e1c_of(m["e1op"])
            r["metrics"] = m
            r["ate_rmse_m"] = m["e1"]["value"]
            r["n_pairs"] = m["e1"].get("n_pairs")
            status = m["e1"]["status"]
            inst = [(name, x["status"]) for name, x in m.items()
                    if name != "e1" and "same_as" not in x and x["status"].startswith("INSTRUMENT")]
            if m["e1"]["status"].startswith("INSTRUMENT"):
                bad_instrument += 1
            for name, st in inst:
                bad_instrument += 1
                status += "; %s: %s" % (name, st)
            for name, c in (r.get("checks") or {}).items():
                if name != "pre_activation_vs_U" and str(c.get("status", "")).startswith("MISMATCH"):
                    bad_instrument += 1
                    status = "INSTRUMENT (%s: %s); e1 %s" % (name, c["status"], status)
            r["status"] = status
        # A55: one binary for U and the GNSS arms
        builds = {}
        for r in seq_runs:
            if r["run_dir"] and r.get("build_sha256"):
                builds.setdefault(r["arm"], set()).add(r["build_sha256"])
        u_b = builds.get(args.u_arm, set())
        g_b = set().union(*[b for a, b in builds.items() if arm_traj.get(a) != HOST_TRAJ]) if builds else set()
        same = None if not (u_b and g_b) else (len(u_b | g_b) == 1)
        result["sequences"][seq] = {"gt": gprov, "mask": mask.record(), "class": ec.seq_class(seq), "span_mode": args.span,
                                    "e1_spans": seq_spans, "post30_spans": seq_post,
                                    "gt_span_length_s": gprov["span"][1] - gprov["span"][0],
                                    "builds": {a: sorted(b) for a, b in builds.items()}, "a55_same_build": same}
        for arm in sorted({r["arm"] for r in seq_runs}, key=lambda a: arm_order.index(a) if a in arm_order else 99):
            rs = [r for r in seq_runs if r["arm"] == arm]
            ok = [r for r in rs if r.get("metrics") and r["metrics"]["e1"]["status"] in OK and not r["status"].startswith("INSTRUMENT")]
            vals = [r["ate_rmse_m"] for r in ok]
            groups = {}
            for r in ok:
                groups.setdefault(r.get("span_rows_sha256"), []).append(r["seed"])
            dup_groups = sorted(g for h, g in groups.items() if h and len(g) > 1)
            gnss = any(r["gnss_arm"] for r in rs)
            sub = {}
            for name in ("post30", "e4", "e1o", "e1p", "e1op", "e1c"):
                good = [r for r in ok if r["metrics"][name]["status"] in OK]
                sub[name] = stat_block([r["metrics"][name]["value"] for r in good],
                                       {"seeds_ok": [r["seed"] for r in good], "n_runs": len(rs),
                                        "spans": sorted({tuple(r["metrics"][name]["span"]) for r in good})})
                if name in LEVER_METRICS:          # E1': the fitted lever arms of the cell (per seed, IMU frame, m)
                    sub[name]["lever_m"] = {str(r["seed"]): r["metrics"][name]["lever"]["lever_m"] for r in good}
            sub["post30"]["n_never_activated"] = sum(1 for r in rs if r.get("activated") is False)
            pav = [r.get("checks", {}).get("pre_activation_vs_U", {}).get("status") for r in rs if r["gnss_arm"]]
            cell = {"seq": seq, "arm": arm, "gnss_arm": gnss, "n_runs": len(rs),
                    "numpy_only": any(r["metrics"]["e1"]["status"] != "ok" for r in ok),
                    "seeds_ok": [r["seed"] for r in ok], "seeds_missing": [r["seed"] for r in rs if r not in ok],
                    "n_distinct": len(groups), "identical_seed_groups": dup_groups,
                    "n_activated": sum(1 for r in rs if r.get("activated")),
                    "activation_after_span_start_s": [r["activation"]["t_act"] - r["e1_span"][0] for r in ok
                                                      if r.get("activated") and r.get("e1_span")],
                    "post30": sub["post30"], "e4": sub["e4"], "e1o": sub["e1o"], "e1p": sub["e1p"], "e1op": sub["e1op"],
                    "e1c": sub["e1c"],
                    "pre_activation_vs_U": {"identical": pav.count("identical"), "differs": pav.count("DIFFERS"),
                                            "no_U": pav.count("no U run")},
                    "a55_flag": bool(pav.count("DIFFERS")) or (gnss and same is False) or (arm == args.u_arm and same is False)}
            pr = {str(r["seed"]): r["path"]["ratio"] for r in ok if (r.get("path") or {}).get("ratio") is not None}
            pv = sorted(pr.values())
            cell["path_ratio"] = {"values": pr, "median": float(np.median(pv)) if pv else None,
                                  "min": pv[0] if pv else None, "max": pv[-1] if pv else None, "band": list(PATH_RATIO_BAND),
                                  "seeds_outside_band": sorted(r["seed"] for r in ok if path_band_flag((r.get("path") or {}).get("ratio")))}
            cell.update(stat_block(vals))
            result["cells"].append(cell)
    for r in runs:
        r.setdefault("status", "not scored: %s" % result["blocked"].get(r["seq"], "?"))
    result["runs"] = runs
    ec.atomic_write_json(os.path.join(out_dir, "ate.json"), result)
    write_csvs(out_dir, result)
    report(result, out_dir)
    if bad_instrument:
        print("eval_ate: %d instrument failure(s) (evo/numpy disagreement, evo failure, T_{N<-W} or ACTIVATE check)" % bad_instrument,
              file=sys.stderr)
        return 4
    return 3 if refused else 0


def _f(x, fmt="%.6f"):
    return fmt % x if x is not None else ""


def lever_cols(m):
    """The fitted lever arm of an E1' result as three csv cells (empty when there is none)."""
    lv = ((m or {}).get("lever") or {}).get("lever_m") if (m or {}).get("value") is not None else None
    return [_f(x) for x in lv] if lv else ["", "", ""]


def write_csvs(out_dir, result):
    with open(os.path.join(out_dir, "ate_runs.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["seq", "arm", "seed", "matrix", "run_dir", "traj", "activated", "t_act", "e1_t0", "e1_t1", "e1_n_pairs",
                    "ate_e1_m", "post30_t0", "ate_post30_m", "ate_e4_m", "pre_activation_vs_u", "status", "ate_e1o_m",
                    "ate_e1p_m", "ate_e1op_m", "e1p_lever_x_m", "e1p_lever_y_m", "e1p_lever_z_m", "e1op_lever_x_m",
                    "e1op_lever_y_m", "e1op_lever_z_m", "ate_e1c_m"])
        for r in result["runs"]:
            m = r.get("metrics") or {}
            sp = r.get("e1_span") or [None, None]
            p30 = m.get("post30", {})
            w.writerow([r["seq"], r["arm"], r["seed"], r["matrix"], r["run_dir"], r["traj"],
                        "" if r.get("activated") is None else int(r["activated"]),
                        _f((r.get("activation") or {}).get("t_act")), _f(sp[0]), _f(sp[1]), r.get("n_pairs", "") or "",
                        _f(m.get("e1", {}).get("value")), _f((p30.get("span") or [None])[0]), _f(p30.get("value")),
                        _f(m.get("e4", {}).get("value")), (r.get("checks") or {}).get("pre_activation_vs_U", {}).get("status", ""),
                        r["status"], _f(m.get("e1o", {}).get("value")), _f(m.get("e1p", {}).get("value")),
                        _f(m.get("e1op", {}).get("value"))] + lever_cols(m.get("e1p")) + lever_cols(m.get("e1op"))
                       + [_f(m.get("e1c", {}).get("value"))])
    with open(os.path.join(out_dir, "ate_cells.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["seq", "arm", "n_ok", "n_runs", "e1_median_m", "e1_min_m", "e1_max_m", "post30_n_ok", "post30_median_m",
                    "post30_min_m", "post30_max_m", "e4_n_ok", "e4_median_m", "e4_min_m", "e4_max_m", "n_activated",
                    "seeds_missing", "n_distinct", "identical_seed_groups", "a55_flag", "e1o_n_ok", "e1o_median_m", "e1o_min_m",
                    "e1o_max_m", "e1p_n_ok", "e1p_median_m", "e1p_min_m", "e1p_max_m", "e1op_n_ok", "e1op_median_m",
                    "e1op_min_m", "e1op_max_m", "e1c_n_ok", "e1c_median_m", "e1c_min_m", "e1c_max_m"])
        for c in result["cells"]:
            w.writerow([c["seq"], c["arm"], c["n_ok"], c["n_runs"], _f(c["median_m"]), _f(c["min_m"]), _f(c["max_m"]),
                        c["post30"]["n_ok"], _f(c["post30"]["median_m"]), _f(c["post30"]["min_m"]), _f(c["post30"]["max_m"]),
                        c["e4"]["n_ok"], _f(c["e4"]["median_m"]), _f(c["e4"]["min_m"]), _f(c["e4"]["max_m"]), c["n_activated"],
                        " ".join(map(str, c["seeds_missing"])), c["n_distinct"],
                        " ".join("=".join(map(str, g)) for g in c["identical_seed_groups"]), int(c["a55_flag"]),
                        c["e1o"]["n_ok"], _f(c["e1o"]["median_m"]), _f(c["e1o"]["min_m"]), _f(c["e1o"]["max_m"]),
                        c["e1p"]["n_ok"], _f(c["e1p"]["median_m"]), _f(c["e1p"]["min_m"]), _f(c["e1p"]["max_m"]),
                        c["e1op"]["n_ok"], _f(c["e1op"]["median_m"]), _f(c["e1op"]["min_m"]), _f(c["e1op"]["max_m"]),
                        c["e1c"]["n_ok"], _f(c["e1c"]["median_m"]), _f(c["e1c"]["min_m"]), _f(c["e1c"]["max_m"])])
    # A81: the path ratio in files of its own (the two csv files above keep the columns the other tools and tests read)
    with open(os.path.join(out_dir, "ate_path_runs.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["seq", "arm", "seed", "e1_t0", "e1_t1", "path_ratio", "est_len_m", "gt_len_m", "segments", "segments_skipped",
                    "gt_rows", "outside_band"])
        for r in result["runs"]:
            p = r.get("path")
            if p is None:
                continue
            sp = r.get("e1_span") or [None, None]
            w.writerow([r["seq"], r["arm"], r["seed"], _f(sp[0]), _f(sp[1]), _f(p.get("ratio")), _f(p.get("est_len_m"), "%.3f"),
                        _f(p.get("gt_len_m"), "%.3f"), p.get("n_segments", ""), p.get("n_segments_skipped", ""), p.get("n_rows", ""),
                        int(path_band_flag(p.get("ratio")))])
    with open(os.path.join(out_dir, "ate_path_cells.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["seq", "arm", "n_ok", "path_ratio_median", "path_ratio_min", "path_ratio_max", "seeds_outside_band"])
        for c in result["cells"]:
            p = c["path_ratio"]
            w.writerow([c["seq"], c["arm"], c["n_ok"], _f(p["median"]), _f(p["min"]), _f(p["max"]), " ".join(map(str, p["seeds_outside_band"]))])


def report(result, out_dir):
    """Console summary: counts and flags only (no ATE values: the numbers live in the files)."""
    print("eval_ate: %d runs, %d cells -> %s" % (len(result["runs"]), len(result["cells"]), out_dir))
    for seq, s in result["sequences"].items():
        print("eval_ate: %s: GT time base %s (%+.4f s, %s), ATE mask %s, masked %s s in the e1 span" % (
            seq, s["gt"]["method"], s["gt"]["time_shift_applied_s"], s["gt"]["source"].split(" ")[0], s["mask"]["status"],
            "/".join("%.1f" % x["masked_s"] for x in s["e1_spans"]) or "—"))
        if s["a55_same_build"] is False:
            print("eval_ate: %s: A55 VIOLATION: U and the GNSS arms ran different binaries %s" % (seq, s["builds"]))
    for c in result["cells"]:
        if c.get("identical_seed_groups"):
            print("eval_ate: %s/%s: seeds %s give identical trajectories in the span (%d distinct of %d)" % (
                c["seq"], c["arm"], c["identical_seed_groups"], c["n_distinct"], c["n_ok"]))
        if c["path_ratio"]["seeds_outside_band"]:
            print("eval_ate: %s/%s: seeds %s have a path ratio outside [%g, %g] (A81 divergence rule)" % (
                c["seq"], c["arm"], c["path_ratio"]["seeds_outside_band"], PATH_RATIO_BAND[0], PATH_RATIO_BAND[1]))
        if c["pre_activation_vs_U"]["differs"]:
            print("eval_ate: %s/%s: %d run(s) whose pre-activation host differs from U of the same seed (A55/I11)" % (
                c["seq"], c["arm"], c["pre_activation_vs_U"]["differs"]))
    for seq, why in result["blocked"].items():
        print("eval_ate: %s not scored: %s" % (seq, why))


def build_parser():
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0], formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--matrix", action="append", required=True, help="runs/eval/<name> (repeatable: e.g. the U and the GNSS matrices)")
    p.add_argument("--out", default="", help="output directory (default <first matrix>/eval)")
    p.add_argument("--arms", default="", help="restrict to these arms (also narrows the common spans)")
    p.add_argument("--seqs", default="", help="restrict to these sequences")
    p.add_argument("--u-arm", default="U", help="the host reference arm (A55 checks)")
    p.add_argument("--gt-dir", action="append", default=[], help="<seq>=<gt_audit output dir> (default <root>/audit/<seq>/gt)")
    p.add_argument("--span", choices=("sequence", "seed"), default="sequence")
    p.add_argument("--t-max-diff", type=float, default=ec.T_MAX_DIFF_S)
    p.add_argument("--evo", default="evo_ape")
    p.add_argument("--pylib", default="", help="scipy override dir for evo (default <root>/tools/eval_pylib/site)")
    p.add_argument("--no-evo", action="store_true", help="numpy re-implementation only (the result says so)")
    p.add_argument("--lever-ref", type=float, nargs=3, default=None, metavar=("X", "Y", "Z"),
                   help="the fixed GT antenna lever arm (IMU frame, m) of the E1' records' rmse_at_lever_ref and of e1c; default "
                        "(A76): the host LiDAR's gt_lever_imu of tools/eval/lidars.json (Avia: A58's -0.176 -0.016 0.401)")
    p.add_argument("--lidar", default="auto", choices=["auto"] + sorted(ec.load_lidars()[1]),
                   help="host LiDAR the matrices ran on: auto = their manifests' `lidar` key (absent = %s); selects the "
                        "default lever arm (A76)" % ec.load_lidars()[0])
    p.add_argument("--no-mask", action="store_true",
                   help="A81: do not remove the trust_mask.csv (A54) stretches: GT and estimates are scored over the whole sequence; "
                        "ate.json records it (metric.no_mask, the mask record's not_applied) and, without --out, the output goes to "
                        "<first matrix>/eval_nomask instead of /eval")
    p.add_argument("--evaluation-phase", action="store_true", help="A2: required to score evaluation-set sequences")
    p.add_argument("--root", default=None)
    return p


def main(argv=None):
    args = build_parser().parse_args(argv)
    try:
        return evaluate(args)
    except (ec.Blocked, ValueError) as e:
        print("eval_ate: BLOCKED: %s" % e, file=sys.stderr)
        return 3


if __name__ == "__main__":
    sys.exit(main())
