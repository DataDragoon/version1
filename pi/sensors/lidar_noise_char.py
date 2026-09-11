#!/usr/bin/env python3
"""Characterise TF-LC02 noise as a function of averaging window.

Why this exists: background subtraction (groundstation/frontend/src/lib/
bgModelInterp.js) evaluates the model at a lidar-derived standoff, and two-way
phase runs at 4*pi*f/c = 12.0 deg/mm at 5 GHz. So the lidar's *short-window*
noise -- what one SFCW sweep sees, not what a long capture averages down to --
sets a ceiling on how well the background can cancel. This tool measures that
ceiling directly.

Run it with the sensor aimed at a static target at the distance you actually
operate at. Distance matters: noise measured at 164 mm does not describe
behaviour at 400 mm.

    python3 lidar_noise_char.py --seconds 40 --label "400mm wall"

Outputs poll rate, validity, run-length structure (the sensor's true internal
update rate, which is far below the poll rate), raw std, drift, autocorrelation
half-life, and std-of-block-means vs averaging window -- each window converted
to the background suppression it would permit.
"""

import argparse
import json
import math
import time

import numpy as np

from tflc02 import TFLC02

# Two-way phase sensitivity: 4*pi*f/c in rad/mm at the top of the sweep band.
SPEED_OF_LIGHT = 299792458.0
TOP_FREQ_HZ = 5e9
RAD_PER_MM = 4 * math.pi * TOP_FREQ_HZ / SPEED_OF_LIGHT / 1000.0  # 0.2096
# Path multiplier of the wall-face echo, measured on the bench set (CLAUDE.md).
ECHO_ALPHA = 0.93

# Averaging windows to report, in ms. 250 ms is one SFCW sweep (the number that
# matters for live operation); 10 s is roughly one BG-model capture.
TAUS_MS = [2, 5, 10, 20, 50, 100, 167, 250, 500, 1000, 2000, 5000]


def suppression_db(sigma_mm):
    """Background suppression permitted by a standoff error of sigma_mm.

    A standoff error eps rotates the wall echo by theta = RAD_PER_MM*alpha*eps,
    leaving a residual of 2*sin(theta/2) relative to the echo, so suppression is
    -20*log10(2*sin(theta/2)). Past ~5 mm this goes negative: the compensator
    adds more energy than it removes, which is the false-target mechanism.
    """
    theta = RAD_PER_MM * ECHO_ALPHA * sigma_mm
    resid = 2 * abs(math.sin(theta / 2))
    if resid <= 0:
        return float('inf')
    return -20 * math.log10(resid)


def collect(lidar, seconds):
    """Poll as fast as the driver allows, recording (t, dist, error_code)."""
    t, dist, err = [], [], []
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        r = lidar.read_distance_with_error()
        now = time.monotonic() - t0
        if r is None:
            t.append(now)
            dist.append(np.nan)
            err.append(-1)  # -1 = no/!malformed frame, distinct from a real code
        else:
            d, e = r
            t.append(now)
            dist.append(float(d))
            err.append(int(e))
    return np.array(t), np.array(dist), np.array(err)


def run_lengths(valid_d):
    """Lengths of runs of identical consecutive readings.

    The sensor holds its last measurement between internal updates, so the
    median run length is poll_rate / true_update_rate.
    """
    if len(valid_d) == 0:
        return np.array([])
    runs, cur = [], 1
    for i in range(1, len(valid_d)):
        if valid_d[i] == valid_d[i - 1]:
            cur += 1
        else:
            runs.append(cur)
            cur = 1
    runs.append(cur)
    return np.array(runs)


def autocorr_half_life(t, d):
    """Lag (in seconds) at which the autocorrelation of the detrended series
    first falls to 0.5. Independent-sample rate is roughly 1/(2*half_life)."""
    if len(d) < 10:
        return None
    x = d - np.polyval(np.polyfit(t, d, 1), t)
    x = x - x.mean()
    denom = np.dot(x, x)
    if denom <= 0:
        return None
    dt = float(np.median(np.diff(t)))
    max_lag = min(len(x) // 2, 5000)
    for lag in range(1, max_lag):
        r = np.dot(x[:-lag], x[lag:]) / denom
        if r < 0.5:
            if lag == 1:
                return dt
            # linear interpolation between the bracketing lags
            rp = np.dot(x[:-(lag - 1)], x[lag - 1:]) / denom if lag > 1 else 1.0
            frac = (rp - 0.5) / (rp - r) if rp != r else 0.0
            return (lag - 1 + frac) * dt
    return None


def block_mean_std(t, d, tau_s):
    """Std of non-overlapping block means over windows of tau_s seconds.

    This is the quantity that matters: a sweep averages the lidar over its own
    duration, so the error it carries is the scatter of *block means*, not of
    raw samples.
    """
    if len(t) == 0:
        return None, 0
    edges = np.arange(t[0], t[-1] + tau_s, tau_s)
    idx = np.digitize(t, edges) - 1
    means, counts = [], []
    for b in range(len(edges)):
        sel = d[idx == b]
        sel = sel[np.isfinite(sel)]
        if len(sel) > 0:
            means.append(sel.mean())
            counts.append(len(sel))
    if len(means) < 3:
        return None, len(means)
    return float(np.std(means, ddof=1)), len(means)


def characterise(t, d, err, label, seconds):
    out = {'label': label, 'requested_seconds': seconds}
    n = len(t)
    span = t[-1] - t[0] if n > 1 else 0.0
    poll_rate = n / span if span > 0 else 0.0

    finite = np.isfinite(d)
    valid = (err == 0) & finite
    dv, tv = d[valid], t[valid]

    print(f"\n{'='*66}")
    print(f"  {label}")
    print(f"{'='*66}")
    print(f"samples            {n}  over {span:.2f} s")
    print(f"poll rate          {poll_rate:.1f} Hz")
    print(f"valid (err==0)     {valid.sum()}/{n} = {100*valid.sum()/max(n,1):.2f}%")

    codes, counts = np.unique(err, return_counts=True)
    hist = {int(c): int(k) for c, k in zip(codes, counts)}
    print(f"error-code hist    {hist}   (-1 = no/malformed frame)")
    out.update(samples=n, span_s=span, poll_rate_hz=poll_rate,
               valid_frac=float(valid.sum() / max(n, 1)), error_hist=hist)

    if len(dv) < 20:
        print("!! too few valid readings to characterise")
        return out

    print(f"\nmean distance      {dv.mean():.3f} mm")
    print(f"raw std            {dv.std(ddof=1):.4f} mm")
    print(f"min / max          {dv.min():.0f} / {dv.max():.0f} mm")
    print(f"quantisation       {np.median(np.diff(np.unique(dv))):.0f} mm "
          f"(median gap between distinct values)")
    out.update(mean_mm=float(dv.mean()), raw_std_mm=float(dv.std(ddof=1)),
               min_mm=float(dv.min()), max_mm=float(dv.max()))

    runs = run_lengths(dv)
    if len(runs):
        med_run = float(np.median(runs))
        internal = poll_rate / med_run if med_run > 0 else 0.0
        print(f"\nrun length         median {med_run:.0f}, mean {runs.mean():.1f}, "
              f"max {runs.max()} identical consecutive polls")
        print(f"internal update    ~{internal:.1f} Hz  "
              f"(poll rate / median run) -- polling faster than this buys nothing")
        out.update(median_run=med_run, internal_update_hz=float(internal))

    slope, intercept = np.polyfit(tv, dv, 1)
    print(f"\nlinear drift       {slope:+.4f} mm/s  "
          f"({slope*span:+.3f} mm over the run)")
    out['drift_mm_per_s'] = float(slope)

    hl = autocorr_half_life(tv, dv)
    if hl:
        print(f"autocorr half-life {hl*1000:.1f} ms  "
              f"-> independent samples at ~{1/(2*hl):.1f} Hz")
        out['autocorr_half_life_s'] = float(hl)

    print(f"\n{'tau (ms)':>9} {'blocks':>7} {'sigma (mm)':>11} {'suppression (dB)':>17}")
    print(f"{'-'*9} {'-'*7} {'-'*11} {'-'*17}")
    taus = {}
    for tau_ms in TAUS_MS:
        if tau_ms / 1000.0 > span / 3:
            continue
        s, nb = block_mean_std(tv, dv, tau_ms / 1000.0)
        if s is None:
            continue
        note = ''
        if tau_ms == 250:
            note = '  <- one SFCW sweep'
        elif tau_ms == 10000:
            note = '  <- one BG capture'
        print(f"{tau_ms:>9} {nb:>7} {s:>11.4f} {suppression_db(s):>17.1f}{note}")
        taus[tau_ms] = {'sigma_mm': s, 'blocks': nb, 'suppression_db': suppression_db(s)}
    out['tau'] = taus

    # tau^-p scaling exponent, fitted in log-log over the measured windows
    ks = sorted(taus.keys())
    if len(ks) >= 3:
        lx = np.log([k for k in ks])
        ly = np.log([taus[k]['sigma_mm'] for k in ks])
        p = np.polyfit(lx, ly, 1)[0]
        print(f"\nscaling            sigma ~ tau^{p:.3f} "
              f"(white noise would be tau^-0.5)")
        out['tau_exponent'] = float(p)

    if 250 in taus:
        s250 = taus[250]['sigma_mm']
        print(f"\nHEADLINE: sigma at tau=250 ms (one sweep) = {s250:.3f} mm "
              f"-> {suppression_db(s250):.1f} dB suppression ceiling")
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--seconds', type=float, default=40.0, help='run length (default 40)')
    ap.add_argument('--label', default='lidar', help='label for this run')
    ap.add_argument('--port', default='/dev/ttyAMA3', help='serial port')
    ap.add_argument('--json', default=None, help='write results + raw series to this JSON')
    args = ap.parse_args()

    lidar = TFLC02(port=args.port)
    print(f"TF-LC02 on {lidar.ser.port}; collecting {args.seconds:.0f} s "
          f"for '{args.label}'...")
    try:
        t, d, e = collect(lidar, args.seconds)
    finally:
        lidar.close()

    res = characterise(t, d, e, args.label, args.seconds)

    if args.json:
        with open(args.json, 'w') as f:
            json.dump({'summary': res,
                       't': t.tolist(),
                       'dist': [None if not np.isfinite(x) else x for x in d],
                       'err': e.tolist()}, f)
        print(f"\nwrote {args.json}")


if __name__ == '__main__':
    main()
