#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Time base of a slim bag from the data alone: the IMU clock against the GNSS receiver's clock (no estimator, no ground truth).

  time_check.py <slim bag> --nav <brdm*.24p> (--gt-dir <dir> | --approx-xyz X Y Z) [--seq S] [--out report.json]

Why: the bag stamps are the platform's GPST seconds written as Unix seconds (dscommon/timebase.py), converted with a constant
shift of 18 s leap seconds, and the GNSS epochs are the receiver's own GPST. Whether the two clocks agree (leap seconds, a
constant offset, a drift, the 101 s by which a recorded file name differs from the stamps) is measured here with two signals
that exist on both sides and do not need an estimator:

  yaw rate    the gyroscope's rotation about the vertical (level frame of tools/eval/lidars.json) against the course rate of the
              Doppler-LS horizontal velocity (a road vehicle drives where it points: chi_dot = -psi_dot), valid above 2 m/s;
  forward     the accelerometer's forward specific force against the time derivative of the Doppler horizontal speed
  acceleration (both high-passed, the slope's gravity leak is slow).

Each is cross-correlated over lags of +-150 s (coarse, 20 Hz: a wrong leap second shows at 18 s, the file-name error at ~100 s)
and +-3 s (fine, 100 Hz, parabolic peak), and windowed (120 s) for the stability of the lag and its drift. The Doppler LS needs
a position only for the local frame (a few km are enough: --approx-xyz, or the ENU origin of the ground-truth directory, which
is a place, not a trajectory). The only thing read from the estimator side is nothing: the LiDAR clock is checked by the
replay's own method-2 time calibration (tools/time_calib/gnss_time_calib.py).

A2: no trajectory ground truth is read, but the report is a GNSS-versus-sensor comparison, so only development sequences run.
numpy only (the image's scipy is broken against its numpy).
"""
import argparse
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, TOOLS)
sys.path.insert(0, os.path.join(TOOLS, "time_calib"))

import numpy as np  # noqa: E402

from dscommon import rosbytes  # noqa: E402
from fl2g import geodesy, rinexnav, gnsstime  # noqa: E402
from fl2g.sequences import seq_class  # noqa: E402

LEAP_S = 18
# SmartPNT mini: I300 frame (x down, y right, z back) -> level frame (x forward, y left, z up): tools/eval/lidars.json R_level_from_imu
R_LEVEL_FROM_IMU = np.array([[0, 0, -1], [0, -1, 0], [-1, 0, 0]], dtype=float)
MIN_SPEED = 2.0               # default; a walking-speed sequence needs --min-speed 0.4 and a longer --smooth-s


def read_imu(bag, topic="/imu_raw"):
    from rosbags.rosbag1 import Reader
    t, g, a = [], [], []
    with Reader(bag) as r:
        conns = [c for c in r.connections if c.topic == topic]
        for c, ts, raw in r.messages(connections=conns):
            d = rosbytes.parse_imu(raw)
            t.append(d["stamp_ns"])
            g.append(d["gyro"])
            a.append(d["acc"])
    return np.array(t, dtype=np.int64) / 1e9, np.array(g), np.array(a)


def gauss(x, sigma_samples):
    """Zero-phase Gaussian smoothing of the rows of x (n or n x k), edge-normalised (no zero padding bias)."""
    r = int(math.ceil(4 * sigma_samples))
    k = np.exp(-0.5 * (np.arange(-r, r + 1) / sigma_samples) ** 2)
    x = np.asarray(x, dtype=float)
    one = np.ones(x.shape[0])
    den = np.convolve(one, k, mode="same")
    if x.ndim == 1:
        return np.convolve(x, k, mode="same") / den
    return np.stack([np.convolve(x[:, j], k, mode="same") / den for j in range(x.shape[1])], axis=1)


def regular(t, x, fs, max_gap_s=0.25):
    """Linear resampling onto a regular grid; (grid, values, valid) where valid = a real sample within max_gap_s."""
    t = np.asarray(t, dtype=float)
    grid = np.arange(t[0], t[-1], 1.0 / fs)
    v = np.interp(grid, t, x) if x.ndim == 1 else np.stack([np.interp(grid, t, x[:, j]) for j in range(x.shape[1])], axis=1)
    j = np.clip(np.searchsorted(t, grid), 1, len(t) - 1)
    gap = np.minimum(np.abs(grid - t[j - 1]), np.abs(t[j] - grid))
    return grid, v, gap <= max_gap_s


def masked_xcorr(a, ma, b, mb, lags):
    """Normalised correlation of a(t) and b(t + lag) [lag in samples] over the samples where both are valid."""
    out = np.full(len(lags), np.nan)
    n = len(a)
    for i, L in enumerate(lags):
        if L >= 0:
            x, y, mx, my = a[:n - L], b[L:], ma[:n - L], mb[L:]
        else:
            x, y, mx, my = a[-L:], b[:n + L], ma[-L:], mb[:n + L]
        m = mx & my
        if m.sum() < 200:
            continue
        xx, yy = x[m] - x[m].mean(), y[m] - y[m].mean()
        d = math.sqrt(float(xx @ xx) * float(yy @ yy))
        if d > 0:
            out[i] = float(xx @ yy) / d
    return out


def peak(lags_s, c):
    """Lag of the maximum with a parabolic refinement; (lag_s, corr, second-best corr outside +-2 s of the peak)."""
    k = int(np.nanargmax(c))
    lag = lags_s[k]
    if 0 < k < len(c) - 1 and np.isfinite(c[k - 1]) and np.isfinite(c[k + 1]):
        d = c[k - 1] - 2 * c[k] + c[k + 1]
        if d < 0:
            lag = lags_s[k] + 0.5 * (c[k - 1] - c[k + 1]) / d * (lags_s[1] - lags_s[0])
    far = np.abs(lags_s - lags_s[k]) > 2.0
    second = float(np.nanmax(c[far])) if far.any() and np.isfinite(c[far]).any() else float("nan")
    return float(lag), float(c[k]), second


def lag_search(ta, a, ma, b, mb, fs_coarse, fs_fine, coarse_s=150.0, fine_s=3.0):
    """a, b on one regular grid at fs_fine (a(t), b(t)); coarse scan on a decimated copy, then a fine scan around the peak.
    Convention: the lag L maximises corr(a(t), b(t + L)); L > 0: b's events come LATER in the stamps than a's."""
    step = int(round(fs_fine / fs_coarse))
    ca, cb = a[::step], b[::step]
    cma, cmb = ma[::step], mb[::step]
    lags = np.arange(-int(coarse_s * fs_coarse), int(coarse_s * fs_coarse) + 1)
    coarse_curve = masked_xcorr(ca, cma, cb, cmb, lags)
    cc = peak(lags / fs_coarse, coarse_curve)
    c0 = cc[0]
    fl = np.arange(int((c0 - fine_s) * fs_fine), int((c0 + fine_s) * fs_fine) + 1)
    # keep the fine window centred on the coarse peak but never beyond +-fine_s of zero unless the coarse peak is elsewhere
    fc = masked_xcorr(a, ma, b, mb, fl)
    lag, corr, second = peak(fl / fs_fine, fc)
    return {"coarse_peak_lag_s": cc[0], "coarse_peak_corr": cc[1], "coarse_second_peak_corr_beyond_2s": cc[2],
            "corr_at_lag_0": float(masked_xcorr(a, ma, b, mb, [0])[0]),
            "corr_at_lag_plus18": float(np.interp(18.0, lags / fs_coarse, np.nan_to_num(coarse_curve, nan=0.0))),
            "corr_at_lag_minus18": float(np.interp(-18.0, lags / fs_coarse, np.nan_to_num(coarse_curve, nan=0.0))),
            "fine_peak_lag_s": lag, "fine_peak_corr": corr, "fine_second_peak_corr_beyond_2s": second}


def windowed(t, a, ma, b, mb, fs, centre, win_s=120.0, step_s=60.0, half_s=1.0):
    """Per-window fine lag (+-half_s around `centre`), with the window's correlation; a linear fit lag(t) for the drift."""
    n = len(t)
    w, st = int(win_s * fs), int(step_s * fs)
    rows = []
    lags = np.arange(int((centre - half_s) * fs), int((centre + half_s) * fs) + 1)
    for i0 in range(0, n - w, st):
        sl = slice(i0, i0 + w)
        if min(ma[sl].mean(), mb[sl].mean()) < 0.6:
            continue
        c = masked_xcorr(a[sl], ma[sl], b[sl], mb[sl], lags)
        if not np.isfinite(c).any():
            continue
        lag, corr, _ = peak(lags / fs, c)
        if corr >= 0.5 and abs(lag - lags[0] / fs) > 1.5 / fs and abs(lag - lags[-1] / fs) > 1.5 / fs:
            rows.append((float(t[i0 + w // 2]), lag, corr))
    if not rows:
        return {"windows": 0}
    r = np.array(rows)
    out = {"windows": len(rows), "lag_median_s": float(np.median(r[:, 1])),
           "lag_mad_s": float(np.median(np.abs(r[:, 1] - np.median(r[:, 1])))), "lag_min_s": float(r[:, 1].min()),
           "lag_max_s": float(r[:, 1].max()), "corr_median": float(np.median(r[:, 2]))}
    if len(rows) >= 4:
        p = np.polyfit(r[:, 0] - r[0, 0], r[:, 1], 1)
        out["drift_ppm"] = float(p[0] * 1e6)
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("bag")
    ap.add_argument("--nav", required=True)
    ap.add_argument("--gt-dir", default="")
    ap.add_argument("--approx-xyz", type=float, nargs=3, default=None)
    ap.add_argument("--seq", default="")
    ap.add_argument("--imu-topic", default="/imu_raw")
    ap.add_argument("--out", default="")
    ap.add_argument("--min-speed", type=float, default=MIN_SPEED, help="horizontal speed [m/s] above which the course is used")
    ap.add_argument("--smooth-s", type=float, default=0.5, help="Gaussian sigma [s] of the zero-phase smoothing of both sides")
    a = ap.parse_args(argv)
    seq = a.seq or os.path.basename(a.bag).replace(".bag", "")
    if seq_class(seq) != "development":
        raise SystemExit("A2: %s is not a development sequence; time_check compares GNSS and sensor signals" % seq)
    xyz = a.approx_xyz
    if xyz is None:
        if not a.gt_dir:
            raise SystemExit("give --approx-xyz or --gt-dir")
        with open(os.path.join(a.gt_dir, "gt_audit.json")) as f:
            xyz = [float(x) for x in geodesy.pos2ecef(*json.load(f)["enu_origin_lat_lon_h_rad_rad_m"])]

    import gnss_time_calib as gtc
    from fl2g import gnss_msgs, rosio
    rb = rosio.RawBag(a.bag)
    eps, eph, geph = [], [], []
    for topic, datatype, raw, t_rec, pytype in rb.iter_raw(topics=list(gnss_msgs.GNSS_TOPICS)):
        if topic == gnss_msgs.TOPIC_MEAS:
            eps.append(gnss_msgs.meas_epoch(rb.deserialize(pytype, raw), t_rec))
        elif topic == gnss_msgs.TOPIC_EPHEM:
            eph.append(gnss_msgs.kepler_eph(rb.deserialize(pytype, raw), t_rec))
        elif topic == gnss_msgs.TOPIC_GLO_EPHEM:
            geph.append(gnss_msgs.glo_eph(rb.deserialize(pytype, raw), t_rec))
    rb.close()
    rnx = rinexnav.read_nav(a.nav)[1]
    tg, vg, dinfo = gtc.doppler_velocity_series(eps, eph, geph, gtc.fixed_position_pvt(eps, xyz), rnx)
    ti, gyro, acc = read_imu(a.bag, a.imu_topic)
    # IMU stamps on the GPST scale (the slim bag is UTC Unix): the GNSS epochs are GPST
    leap = gnsstime.leap_at_gpst(ti[0] + LEAP_S)
    ti_g = ti + leap
    w_level = gyro @ R_LEVEL_FROM_IMU.T
    f_level = acc @ R_LEVEL_FROM_IMU.T
    psi_dot = w_level[:, 2]                    # yaw rate about up, CCW positive
    f_fwd = f_level[:, 0]

    fs = 100.0
    t0, t1 = max(ti_g[0], tg[0]) + 5.0, min(ti_g[-1], tg[-1]) - 5.0
    grid = np.arange(t0, t1, 1.0 / fs)
    # Doppler side: smooth the velocity VECTOR (E, N) with 0.5 s, then course rate and speed derivative
    gg, vv, vvalid = regular(tg, vg[:, :2], fs)
    vs = gauss(vv, a.smooth_s * fs)
    sp = np.hypot(vs[:, 0], vs[:, 1])
    chi_dot = (vs[:, 1] * np.gradient(vs[:, 0], 1 / fs) - vs[:, 0] * np.gradient(vs[:, 1], 1 / fs)) / np.maximum(sp, 0.5) ** 2
    dsp = np.gradient(sp, 1 / fs)
    ok_g = vvalid & (sp > a.min_speed)
    # IMU side on the same time axis, smoothed like the Doppler side (0.5 s) so that both see the same bandwidth
    ii = lambda x: np.interp(gg, ti_g, x)
    psi_s = gauss(ii(psi_dot), a.smooth_s * fs)
    f_s = gauss(ii(f_fwd), a.smooth_s * fs)
    in_span = (gg >= t0) & (gg <= t1)
    ok_a = ok_g & in_span
    # forward acceleration: high-pass both (subtract a 4 s Gaussian low-pass) so that the slope's gravity leak and the bias drop
    f_hp = f_s - gauss(f_s, 4.0 * fs)
    d_hp = dsp - gauss(dsp, 4.0 * fs)
    out = {"sequence": seq, "bag": os.path.abspath(a.bag), "leap_s_applied_to_stamps": leap,
           "doppler": dict(dinfo, epochs=int(len(tg)), approx_xyz=[round(float(x), 1) for x in xyz]),
           "signal": {"min_speed_mps": a.min_speed, "smooth_s": a.smooth_s, "grid_hz": fs,
                      "moving_fraction": float(np.mean(ok_a[in_span])) if in_span.any() else 0.0},
           "convention": "lag L maximises corr(imu(t), gnss(t + L)): L > 0 means the GNSS epochs are stamped later than the same "
                         "motion in the IMU stamps (the GNSS clock is ahead); stamps compared on the GPST scale after the 18 s shift"}
    # yaw rate: gyro psi_dot against -chi_dot (course rate is clockwise)
    for key, x, y, sd in (("yaw_rate", psi_s, -chi_dot, "signal_std_rad_s"), ("forward_acceleration", f_hp, d_hp, "signal_std_mps2")):
        try:
            out[key] = lag_search(gg, x, ok_a, y, ok_a, 20.0, fs)
            out[key]["windows_120s"] = windowed(gg, x, ok_a, y, ok_a, fs, out[key]["fine_peak_lag_s"])
            out[key][sd] = {"imu": float(np.std(x[ok_a])), "gnss": float(np.std(y[ok_a]))}
        except ValueError as e:
            out[key] = {"error": "%s (%d valid samples at %g Hz)" % (e, int(ok_a.sum()), fs)}
    txt = json.dumps(out, indent=1)
    if a.out:
        with open(a.out, "w") as f:
            f.write(txt + "\n")
    print(txt)
    return 0


if __name__ == "__main__":
    sys.exit(main())
