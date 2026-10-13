# SPDX-License-Identifier: BSD-3-Clause
"""Robust statistics, step detection, cross-correlation and DTW used by the audit and the time calibration.

Design notes:
* step detection (A18 "台阶(>50 ms 持续)= 新段"): at every sample the medians of the preceding and following
  windows (length = the persistence requirement) are compared; local maxima of the jump above the threshold
  that are at least one window apart are steps. Medians make isolated latency spikes invisible;
* cross-correlation (DESIGN §7 method 2): Pearson correlation of two resampled signals over a lag grid,
  parabolic refinement of the peak; the peak half-width at 0.5 of the peak is reported as a sharpness measure,
  and the peak curvature (a quadratic fit of the fine scan) turns peak height + width into a delay precision
  (tde_sigma, "σ_toff 取峰宽", DESIGN §7);
* DTW (method 3): Sakoe-Chiba banded dynamic time warping on z-scored signals; the per-step index offsets along
  the optimal path give an offset distribution (median, MAD) that is compared with the xcorr offset.
"""
import math

import numpy as np


def mad(x):
    x = np.asarray(x, dtype=float)
    if x.size == 0:
        return float("nan")
    return float(np.median(np.abs(x - np.median(x))))


def robust_std(x):
    return 1.4826 * mad(x)


def percentiles(x, ps=(0, 1, 5, 25, 50, 75, 95, 99, 100)):
    x = np.asarray(x, dtype=float)
    x = x[np.isfinite(x)]
    if x.size == 0:
        return {}
    v = np.percentile(x, ps)
    return {("p%g" % p): float(q) for p, q in zip(ps, v)}


def summary(x):
    x = np.asarray(x, dtype=float)
    x = x[np.isfinite(x)]
    if x.size == 0:
        return {"n": 0}
    d = {"n": int(x.size), "mean": float(np.mean(x)), "std": float(np.std(x)), "median": float(np.median(x)),
         "mad": mad(x), "min": float(np.min(x)), "max": float(np.max(x))}
    d.update(percentiles(x, (1, 5, 25, 75, 95, 99)))
    return d


def theil_sen(t, y, max_pairs=200000, seed=0):
    """Median of pairwise slopes (random subset when large); intercept = median(y - s t). Robust to 29 % outliers."""
    t = np.asarray(t, dtype=float)
    y = np.asarray(y, dtype=float)
    n = t.size
    if n < 2:
        return 0.0, float(np.median(y)) if n else float("nan")
    rng = np.random.RandomState(seed)
    if n * (n - 1) // 2 <= max_pairs:
        i, j = np.triu_indices(n, 1)
    else:
        i = rng.randint(0, n, max_pairs)
        j = rng.randint(0, n, max_pairs)
        keep = i != j
        i, j = i[keep], j[keep]
    dt = t[j] - t[i]
    ok = np.abs(dt) > 1e-9
    s = float(np.median((y[j][ok] - y[i][ok]) / dt[ok]))
    return s, float(np.median(y - s * t))


def running_percentile(t, y, t_eval, half_window, q):
    """q-th percentile of y over t in [t_eval - half_window, t_eval + half_window] (NaN where empty). Used for the
    lower envelope of one-sided latency jitter (bursty sensor delivery only ever adds delay)."""
    t = np.asarray(t, dtype=float)
    y = np.asarray(y, dtype=float)
    t_eval = np.atleast_1d(np.asarray(t_eval, dtype=float))
    order = np.argsort(t, kind="stable")
    t, y = t[order], y[order]
    lo = np.searchsorted(t, t_eval - half_window, side="left")
    hi = np.searchsorted(t, t_eval + half_window, side="right")
    out = np.full(t_eval.size, np.nan)
    for k in range(t_eval.size):
        if hi[k] > lo[k]:
            out[k] = np.percentile(y[lo[k]:hi[k]], q)
    return out


def detect_steps(t, y, min_step, window_s, min_points=10):
    """Persistent level shifts of |size| > min_step: list of (t_step, size, before_median, after_median)."""
    t = np.asarray(t, dtype=float)
    y = np.asarray(y, dtype=float)
    n = t.size
    if n < 2 * min_points:
        return []
    jump = np.zeros(n)
    lo = np.searchsorted(t, t - window_s, side="left")
    hi = np.searchsorted(t, t + window_s, side="right")
    for k in range(n):
        a = y[lo[k]:k]
        b = y[k:hi[k]]
        if a.size >= min_points and b.size >= min_points:
            jump[k] = np.median(b) - np.median(a)
    cand = [k for k in range(n) if abs(jump[k]) > min_step]
    steps = []
    last_t = -np.inf
    # non-maximum suppression within one window
    cand.sort(key=lambda k: -abs(jump[k]))
    chosen = []
    for k in cand:
        if all(abs(t[k] - t[c]) >= window_s for c in chosen):
            chosen.append(k)
    for k in sorted(chosen):
        if t[k] - last_t < window_s:
            continue
        a = y[lo[k]:k]
        b = y[k:hi[k]]
        steps.append((float(t[k]), float(jump[k]), float(np.median(a)), float(np.median(b))))
        last_t = t[k]
    return steps


def pearson(a, b):
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    a = a - a.mean()
    b = b - b.mean()
    den = math.sqrt(float(a @ a) * float(b @ b))
    return float(a @ b) / den if den > 0 else float("nan")


def smooth_uniform(t, x, fs, sigma_s):
    """Linear resampling at fs over [t0, t1] followed by a zero-phase Gaussian (sigma_s) with edge renormalisation.
    Zero phase matters: the same kernel on both sides of a cross-correlation cannot shift its peak."""
    t = np.asarray(t, dtype=float)
    x = np.asarray(x, dtype=float)
    g = np.arange(t[0], t[-1], 1.0 / fs)
    y = np.interp(g, t, x)
    if sigma_s > 0.0:
        half = int(math.ceil(4.0 * sigma_s * fs))
        k = np.exp(-0.5 * (np.arange(-half, half + 1) / (sigma_s * fs)) ** 2)
        num = np.convolve(y, k, mode="same")
        den = np.convolve(np.ones_like(y), k, mode="same")
        y = num / den
    return g, y


def _corr_curve(grid, ag, t_b, b, lags):
    return np.array([pearson(ag, np.interp(grid + L, t_b, b)) for L in lags])


def xcorr_offset(t_a, a, t_b, b, lag_min, lag_max, lag_step=0.01, fs=100.0, t_from=None, t_to=None, fine_step=0.0005,
                 smooth_s=0.0):
    """Find lag L maximising corr(a(t), b(t + L)) over the overlap. Returns dict(lag, peak, lags, curve, width,
    curv_s, duration_s, n, at_edge).

    Both signals are linearly resampled at fs on `a`'s span (optionally cut to [t_from, t_to]); a coarse scan with
    `lag_step` is followed by a fine scan (`fine_step`) of +-2 coarse steps around the coarse peak and a parabolic
    refinement of the fine peak (b is re-interpolated for every lag, so sub-sample lags are meaningful).
    Sign convention: if b is a copy of a delayed by d (b(t) = a(t - d)), the result is L = +d; equivalently a
    feature of `a` at time t appears in `b` at t + L. curv_s is the width of the peak from its curvature
    (rho(L0 + x) ~ rho0 (1 - x^2 / (2 curv_s^2)), quadratic fit of the fine scan), NaN if not concave."""
    t_a = np.asarray(t_a, dtype=float)
    a = np.asarray(a, dtype=float)
    t_b = np.asarray(t_b, dtype=float)
    b = np.asarray(b, dtype=float)
    if smooth_s > 0.0:
        # smooth once on each side's own uniform grid; per-lag re-interpolation of the smoothed b stays exact to
        # well below a millisecond at fs = 100 Hz for band-limited (smoothed) signals
        t_a, a = smooth_uniform(t_a, a, fs, smooth_s)
        t_b, b = smooth_uniform(t_b, b, fs, smooth_s)
    lo = max(t_a[0], t_b[0] - lag_min) if t_from is None else max(t_from, t_a[0], t_b[0] - lag_min)
    hi = min(t_a[-1], t_b[-1] - lag_max) if t_to is None else min(t_to, t_a[-1], t_b[-1] - lag_max)
    if hi - lo < 5.0:
        return None
    grid = np.arange(lo, hi, 1.0 / fs)
    ag = np.interp(grid, t_a, a)
    lags = np.arange(lag_min, lag_max + 0.5 * lag_step, lag_step)
    curve = _corr_curve(grid, ag, t_b, b, lags)
    if not np.any(np.isfinite(curve)):
        return None
    k = int(np.nanargmax(curve))
    edge = k == 0 or k == lags.size - 1
    fl = np.arange(lags[k] - 2 * lag_step, lags[k] + 2 * lag_step + 0.5 * fine_step, fine_step)
    fl = fl[(fl >= lag_min - 1e-12) & (fl <= lag_max + 1e-12)]
    fc = _corr_curve(grid, ag, t_b, b, fl)
    j = int(np.nanargmax(fc))
    lag = float(fl[j])
    if 0 < j < fl.size - 1:
        y0, y1, y2 = fc[j - 1], fc[j], fc[j + 1]
        den = y0 - 2.0 * y1 + y2
        if den < 0:
            lag += 0.5 * (y0 - y2) / den * fine_step
    peak = float(fc[j])
    half = peak - 0.5 * (peak - float(np.nanmin(curve)))
    above = np.where(curve >= half)[0]
    width = float((above.max() - above.min()) * lag_step) if above.size else float("nan")
    curv_s = float("nan")
    okf = np.isfinite(fc)
    if okf.sum() >= 5:
        c2 = np.polyfit(fl[okf] - lag, fc[okf], 2)[0]
        if c2 < 0.0 and peak > 0.0:
            curv_s = math.sqrt(peak / (-2.0 * c2))
    return {"lag": lag, "peak": peak, "lags": lags, "curve": curve, "width": width, "curv_s": curv_s,
            "duration_s": float(grid.size) / fs, "n": int(grid.size), "at_edge": edge}


def tde_sigma(peak, curv_s, duration_s):
    """Standard deviation [s] of a cross-correlation delay estimate from the peak alone (Carter, "Coherence and
    time delay estimation", Proc. IEEE 1987, high-SNR bound): sigma^2 = (1 - rho^2) / (rho^2 N beta^2), with the
    peak correlation rho as the coherence, beta = 1 / curv_s the RMS bandwidth read from the peak curvature, and
    N = duration / (sqrt(2 pi) curv_s) independent samples (correlation time of a Gaussian-shaped peak of width
    curv_s). NaN when the peak is not usable."""
    if not (np.isfinite(peak) and np.isfinite(curv_s) and 0.0 < peak and curv_s > 0.0 and duration_s > 0.0):
        return float("nan")
    rho2 = min(peak * peak, 1.0)
    n_ind = max(1.0, duration_s / (math.sqrt(2.0 * math.pi) * curv_s))
    return float(curv_s * math.sqrt((1.0 - rho2) / (rho2 * n_ind)))


def dtw_offsets(a, b, band):
    """Banded DTW of equally sampled a, b (z-scored inside). Returns list of (i, j) on the optimal path."""
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    a = (a - a.mean()) / (a.std() + 1e-12)
    b = (b - b.mean()) / (b.std() + 1e-12)
    n, m = a.size, b.size
    INF = np.inf
    D = np.full((n + 1, m + 1), INF)
    D[0, 0] = 0.0
    for i in range(1, n + 1):
        j0 = max(1, i - band)
        j1 = min(m, i + band)
        if j0 > j1:
            continue
        cost = (a[i - 1] - b[j0 - 1:j1]) ** 2
        prev = D[i - 1, j0 - 1:j1 + 1]
        # D[i, j] = cost + min(D[i-1, j-1], D[i-1, j], D[i, j-1]); the last term needs a running scan
        best_diag_up = np.minimum(prev[:-1], prev[1:])
        row = np.empty(j1 - j0 + 1)
        left = D[i, j0 - 1]
        for k in range(row.size):
            v = cost[k] + min(best_diag_up[k], left)
            row[k] = v
            left = v
        D[i, j0:j1 + 1] = row
    i, j = n, m
    if not np.isfinite(D[n, m]):
        return []
    path = [(i - 1, j - 1)]
    while i > 1 or j > 1:
        choices = ((D[i - 1, j - 1], i - 1, j - 1), (D[i - 1, j], i - 1, j), (D[i, j - 1], i, j - 1))
        _, i, j = min(choices, key=lambda c: c[0])
        if i < 1 or j < 1:
            break
        path.append((i - 1, j - 1))
    path.reverse()
    return path
