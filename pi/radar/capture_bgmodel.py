#!/usr/bin/env python3
"""Capture a BG-model training set from the Pi, without the browser.

Writes the same `bgmodel_training_data` v2 JSON the groundstation's BG Model
panel exports, so the file drops straight into that panel's Import, and into
any offline analysis that already reads v2.

Why a Pi-side tool: the export is the only artifact that carries *per-sweep*
standoffs. The saved models keep capture means only, so questions about the
difference between the two regimes -- benchmark scores against a 40-sweep
averaged standoff, live operation runs on one sweep's -- cannot be answered
from a model file. This produces the data that can answer them.

Protocol (matches CLAUDE.md's "BG Model -- Capture Protocol"): one capture is
N sweeps at a *static* standoff. Positions are hand-placed and deliberately
irregular, because uniform undersampling folds alias energy coherently onto one
wrong spatial frequency while irregular spacing scatters it.

The tool is non-interactive on purpose: it alternates a move window (during
which you reposition the rig) with a capture window (hold still). It counts
both down on stdout.

    python3 capture_bgmodel.py --positions 24 --sweeps 40 --move-seconds 5

Requires stream.py (port 9001) and sdr_server.py (port 9003) already running.
"""

import argparse
import asyncio
import json
import math
import os
import random
import statistics
import sys
import time

import websockets

SENSOR_URL = 'ws://127.0.0.1:9001'
SDR_URL = 'ws://127.0.0.1:9003'

# Captures land in the repo's data/ directory, which is gitignored -- these are
# multi-MB measurement artifacts, not source. Timestamped so a second run never
# silently overwrites the first.
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
DATA_DIR = os.path.join(REPO_ROOT, 'data')

# Must match App.jsx's default. A model is indexed by lidar_reading - offset,
# so training and inference have to agree on this or every standoff is biased.
DEFAULT_OFFSET_MM = 132.0

# Mirrors App.jsx sfcwParams. Pushed before the sweep starts, exactly like
# sendSfcwParams() does -- the engine carries its own defaults otherwise and
# the file would record params the hardware never used.
SFCW_PARAMS = {
    'startFreq': 2000, 'stopFreq': 5000, 'stepSize': 60,
    'numBuffers': 1, 'settleCount': 3,
    # tx2/rx2 MUST match what the SFCW panel uses. The reference gain is not just a
    # level -- it re-calibrates h_cal frequency-by-frequency (measured 2026-08-29:
    # 30/20 vs 50/25 gives complex coherence 0.88 and only 6.5 dB of suppression on an
    # unchanged static scene, with per-step |h_cal| ratios spanning 0.007-1.840). A model
    # captured at one reference gain is invalid at another, silently. These are now
    # pushed by App.jsx too, so both sides carry the same numbers.
    'tx1Gain': 50, 'rx1Gain': 25, 'tx2Gain': 45, 'rx2Gain': 5,
    'rangeOffset': 0.378,
}


def position_schedule(n, lo, hi, max_gap=7.0, seed=None):
    """Irregular standoff targets spanning [lo, hi] mm.

    Deliberately irregular rather than uniform: uniform undersampling folds alias
    energy coherently onto one wrong spatial frequency, while irregular spacing
    scatters it. Spacings are drawn from +/-30% of nominal and renormalised onto
    the span, which keeps the largest gap under the ~7.5 mm where an alpha=3
    triple bounce starts to alias (see CLAUDE.md's spacing limits).

    Endpoints are pinned to lo and hi: the span edges are the whole point, because
    querying outside the captured span costs ~20 dB and eventually goes negative.
    """
    rng = random.Random(seed)
    gaps = None
    for _ in range(200):
        g = [rng.uniform(0.7, 1.3) for _ in range(n - 1)]
        scale = (hi - lo) / sum(g)
        g = [x * scale for x in g]
        if max(g) <= max_gap:
            gaps = g
            break
    if gaps is None:  # span too wide for n positions at this max_gap
        gaps = [(hi - lo) / (n - 1)] * (n - 1)
    pos, x = [lo], lo
    for step in gaps:
        x += step
        pos.append(x)
    return pos


class LidarTracker:
    """Mirrors App.jsx: dedupe by lidar_seq, average the distinct readings that
    arrive during one sweep, subtract the antenna offset."""

    def __init__(self, offset_mm):
        self.offset_mm = offset_mm
        self.accum = []
        self.last_seq = None
        self.pose = []
        self.latest = None

    def feed(self, msg):
        d = msg.get('lidar')
        if d is not None:
            seq = msg.get('lidar_seq')
            if seq is None or seq != self.last_seq:
                self.last_seq = seq
                self.accum.append(float(d))
            self.latest = float(d)
        a = msg.get('accel')
        if isinstance(a, list) and len(a) == 3 and all(isinstance(v, (int, float)) for v in a):
            fwd, left, up = a
            self.pose.append((math.degrees(math.atan2(left, up)),
                              math.degrees(math.atan2(-fwd, math.hypot(left, up)))))

    def take(self):
        """Consume the window and return this sweep's provenance."""
        acc, pose = self.accum, self.pose
        self.accum, self.pose = [], []
        n = len(acc)
        mean = sum(acc) / n if n else None
        std = statistics.stdev(acc) if n > 1 else None
        return {
            'lidar_standoff_mm': (mean - self.offset_mm) if mean is not None else None,
            'lidar_raw_mm': mean,
            'lidar_n': n,
            'lidar_std': std,
            'lidar_offset_mm': self.offset_mm,
            'roll_deg': sum(p[0] for p in pose) / len(pose) if pose else None,
            'pitch_deg': sum(p[1] for p in pose) / len(pose) if pose else None,
        }


def capture_stats(samples):
    """Port of groundstation/frontend/src/lib/bgCaptureStats.js computeCaptureStats.

    radarRangeM is omitted: it is documented as diagnostic-only, nothing
    consumes it, and it was measured to be unusable (correlation 0.36 with the
    lidar, 39.9 mm scatter) -- so reimplementing its FFT here would add a
    second copy of code whose output is known not to be trustworthy.
    """
    valid = [s for s in samples if s.get('h_cal_real') and s.get('h_cal_imag')]
    if not valid:
        return None
    n = len(valid)
    S = len(valid[0]['h_cal_real'])

    mRe = [0.0] * S
    mIm = [0.0] * S
    for s in valid:
        for i in range(S):
            mRe[i] += s['h_cal_real'][i]
            mIm[i] += s['h_cal_imag'][i]
    mRe = [v / n for v in mRe]
    mIm = [v / n for v in mIm]

    varF = [0.0] * S
    for s in valid:
        for i in range(S):
            dr = s['h_cal_real'][i] - mRe[i]
            di = s['h_cal_imag'][i] - mIm[i]
            varF[i] += dr * dr + di * di
    dof = max(1, n - 1)
    varF = [v / dof for v in varF]

    sig = sum(mRe[i] ** 2 + mIm[i] ** 2 for i in range(S))
    noise = sum(varF)
    snr_db = 10 * math.log10(sig / (noise or 1e-30))

    corr_sum, corr_n = 0.0, 0
    for k in range(n - 1):
        A, B = valid[k], valid[k + 1]
        dot_re = dot_im = magA = magB = 0.0
        for i in range(S):
            aR, aI = A['h_cal_real'][i], A['h_cal_imag'][i]
            bR, bI = B['h_cal_real'][i], B['h_cal_imag'][i]
            dot_re += aR * bR + aI * bI
            dot_im += aI * bR - aR * bI
            magA += aR * aR + aI * aI
            magB += bR * bR + bI * bI
        corr_sum += math.hypot(dot_re, dot_im) / (math.sqrt(magA) * math.sqrt(magB) + 1e-30)
        corr_n += 1

    dists = [s['lidar_standoff_mm'] for s in samples if s.get('lidar_standoff_mm') is not None]
    d_mean = sum(dists) / len(dists) if dists else None
    d_std = statistics.stdev(dists) if len(dists) > 1 else 0.0

    return {
        'sweepCount': n,
        'numSteps': S,
        'h_mean_real': mRe,
        'h_mean_imag': mIm,
        'noise_var': varF,
        'snrDbPerSweep': snr_db,
        'snrDbAveraged': snr_db + 10 * math.log10(n),
        'coherence': sig / (sig + noise or 1e-30),
        'sweepCorrelation': corr_sum / corr_n if corr_n else None,
        'standoffMm': d_mean,
        'standoffStdMm': d_std,
        'standoffN': len(dists),
        'radarRangeM': None,
    }


async def sensor_task(tracker, stop):
    async with websockets.connect(SENSOR_URL) as ws:
        while not stop.is_set():
            try:
                msg = json.loads(await asyncio.wait_for(ws.recv(), timeout=1.0))
            except asyncio.TimeoutError:
                continue
            tracker.feed(msg)


async def run(args):
    tracker = LidarTracker(args.offset)
    stop = asyncio.Event()
    st = asyncio.create_task(sensor_task(tracker, stop))
    await asyncio.sleep(1.0)
    if tracker.latest is None:
        print("!! no lidar readings on port 9001 -- is stream.py running?")
        stop.set(); await st
        return 1

    targets = None
    if args.span_lo is not None and args.span_hi is not None:
        targets = position_schedule(args.positions, args.span_lo, args.span_hi)
        gaps = [b - a for a, b in zip(targets, targets[1:])]
        print(f"schedule: {args.positions} positions, standoff {targets[0]:.0f}..{targets[-1]:.0f} mm, "
              f"gaps {min(gaps):.1f}-{max(gaps):.1f} mm (irregular)")
        print(f"          lidar should read {targets[0] + args.offset:.0f}..{targets[-1] + args.offset:.0f} mm\n")

    captures = []
    async with websockets.connect(SDR_URL, max_size=None) as ws:
        await ws.send(json.dumps({
            'cmd': 'sfcw_set_params',
            'start_freq_mhz': SFCW_PARAMS['startFreq'],
            'stop_freq_mhz': SFCW_PARAMS['stopFreq'],
            'step_size_mhz': SFCW_PARAMS['stepSize'],
            'num_buffers': SFCW_PARAMS['numBuffers'],
            'settle_count': SFCW_PARAMS['settleCount'],
            'tx1_gain': SFCW_PARAMS['tx1Gain'], 'rx1_gain': SFCW_PARAMS['rx1Gain'],
            'tx2_gain': SFCW_PARAMS['tx2Gain'], 'rx2_gain': SFCW_PARAMS['rx2Gain'],
            'range_offset': SFCW_PARAMS['rangeOffset'],
        }))
        await ws.send(json.dumps({'cmd': 'sfcw_start'}))
        print(f"sweeping {SFCW_PARAMS['startFreq']}-{SFCW_PARAMS['stopFreq']} MHz "
              f"@ {SFCW_PARAMS['stepSize']} MHz\n")

        # The SDR socket must be drained CONTINUOUSLY, not only while capturing.
        # sfcw_start free-runs, so anything not read piles up in the socket
        # buffer -- and a capture that then drains the backlog gets several
        # sweeps in the same instant, each paired with an empty lidar window.
        # (That is exactly what the first version of this tool did: every
        # standoff after the first came back None.) The reader below pairs each
        # sweep with the lidar samples that arrived since the previous one, at
        # the moment it arrives, and hands the pair on.
        sweeps = asyncio.Queue()
        reader_err = {}

        async def sdr_reader():
            try:
                while not stop.is_set():
                    try:
                        m = json.loads(await asyncio.wait_for(ws.recv(), timeout=1.0))
                    except asyncio.TimeoutError:
                        continue
                    if m.get('type') == 'sfcw_result':
                        await sweeps.put((m, tracker.take()))
                    elif m.get('type') == 'sfcw_error':
                        reader_err['e'] = RuntimeError(f"sfcw_error: {m}")
                        return
            except Exception as e:  # noqa: BLE001 - surfaced below
                reader_err['e'] = e

        rt = asyncio.create_task(sdr_reader())

        async def next_sweep(timeout=30.0):
            if 'e' in reader_err:
                raise reader_err['e']
            return await asyncio.wait_for(sweeps.get(), timeout=timeout)

        def drain():
            """Discard sweeps taken while the rig was being repositioned."""
            n = 0
            while not sweeps.empty():
                sweeps.get_nowait()
                n += 1
            return n

        # Discard the first sweep: it straddles the start command, so its lidar
        # window is not a clean one-sweep average.
        await next_sweep()

        try:
            for p in range(args.positions):
                for rem in range(int(args.move_seconds), 0, -1):
                    if targets is not None:
                        tgt = targets[p] + args.offset
                        err = tgt - (tracker.latest or 0)
                        print(f"\r  position {p+1}/{args.positions}: MOVE TO {tgt:.0f} mm "
                              f"(now {tracker.latest or 0:.0f}, {err:+.0f})  {rem}s     ",
                              end='', flush=True)
                    else:
                        print(f"\r  position {p+1}/{args.positions}: MOVE THE RIG "
                              f"({rem}s, lidar {tracker.latest:.0f} mm)   ", end='', flush=True)
                    await asyncio.sleep(1)
                print(f"\r  position {p+1}/{args.positions}: HOLD STILL, capturing "
                      f"{args.sweeps} sweeps...                    ", end='', flush=True)
                # One extra sweep is dropped after the drain: the sweep in flight
                # when the move window ended may have started while the rig was
                # still moving, so its h_cal is not a static-position measurement.
                await next_sweep()

                drain()
                samples = []
                for k in range(args.sweeps):
                    m, prov = await next_sweep()
                    samples.append({
                        'h_cal_real': m.get('h_cal_real'),
                        'h_cal_imag': m.get('h_cal_imag'),
                        'num_steps': m.get('num_steps'),
                        'step_size': m.get('step_size'),
                        'range_offset': m.get('range_offset'),
                        'start_freq': m.get('start_freq'),
                        'stop_freq': m.get('stop_freq'),
                        'timestamp': m.get('timestamp'),
                        'phase_coherence': m.get('phase_coherence'),
                        **prov,
                    })
                stats = capture_stats(samples)
                captures.append({'samples': samples, 'stats': stats})
                sd = stats['standoffMm'] if stats else float('nan')
                print(f"\r  position {p+1}/{args.positions}: standoff {sd:7.2f} mm "
                      f"(sweep-to-sweep std {stats['standoffStdMm']:.2f} mm, "
                      f"SNR {stats['snrDbPerSweep']:.1f} dB/sweep)            ")
                # Save after every position. The bladeRF has been seen to drop its
                # USB stream mid-session, and a 24-position run is ~7 minutes of
                # somebody standing at a bench -- losing all of it to a failure at
                # position 23 is not acceptable. Costs a few ms per position.
                write_out(captures, args, quiet=True)
        except (RuntimeError, asyncio.TimeoutError) as e:
            print(f"\n!! capture aborted after {len(captures)} positions: {e}")
            print(f"   {args.out} holds what was captured -- rerun to add more, "
                  f"or restart start.py if the USB stream died.")

        try:
            await ws.send(json.dumps({'cmd': 'sfcw_stop'}))
        except Exception:
            pass
        stop.set()
        rt.cancel()
        try:
            await rt
        except asyncio.CancelledError:
            pass

    await st

    if not captures:
        print("!! no positions captured")
        return 1
    write_out(captures, args)
    return 0


def write_out(captures, args, quiet=False):
    ref = captures[0]['samples'][0]
    out = {
        'version': 2,
        'type': 'bgmodel_training_data',
        'timestamp': time.strftime('%Y-%m-%dT%H:%M:%S'),
        'session': args.session,
        'sfcwParams': SFCW_PARAMS,
        'lidarAntennaOffsetMm': args.offset,
        'sweepsPerCapture': args.sweeps,
        'common': {
            'num_steps': ref.get('num_steps'),
            'step_size': ref.get('step_size'),
            'range_offset': ref.get('range_offset'),
        },
        'captures': [{
            'stats': c['stats'],
            'standoffs': [s['lidar_standoff_mm'] for s in c['samples']],
            'real': [s['h_cal_real'] for s in c['samples']],
            'imag': [s['h_cal_imag'] for s in c['samples']],
            # Extra columns beyond the browser's v2 export. Consumers key off
            # names, so these are additive -- the panel's import ignores them.
            'lidar_n': [s['lidar_n'] for s in c['samples']],
            'lidar_std': [s['lidar_std'] for s in c['samples']],
            'roll_deg': [s['roll_deg'] for s in c['samples']],
            'pitch_deg': [s['pitch_deg'] for s in c['samples']],
            'sweep_ts': [s['timestamp'] for s in c['samples']],
        } for c in captures],
    }
    with open(args.out, 'w') as f:
        json.dump(out, f)
    if quiet:
        return

    ds = sorted(c['stats']['standoffMm'] for c in captures)
    if len(ds) < 2:
        print(f"\nwrote {args.out} ({len(ds)} position)")
        return
    gaps = [b - a for a, b in zip(ds, ds[1:])]
    print(f"\nwrote {args.out}")
    print(f"  {len(captures)} positions, span {ds[-1]-ds[0]:.1f} mm, "
          f"median gap {statistics.median(gaps):.1f} mm, max gap {max(gaps):.1f} mm")
    print(f"  standoffs: {', '.join(f'{d:.1f}' for d in ds)}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--positions', type=int, default=12)
    ap.add_argument('--sweeps', type=int, default=40, help='sweeps per position')
    ap.add_argument('--move-seconds', type=float, default=8, help='reposition window')
    ap.add_argument('--offset', type=float, default=DEFAULT_OFFSET_MM,
                    help=f'lidar->antenna offset in mm (default {DEFAULT_OFFSET_MM})')
    ap.add_argument('--span-lo', type=float, default=None,
                    help='lowest standoff target in mm; enables per-position targeting')
    ap.add_argument('--span-hi', type=float, default=None,
                    help='highest standoff target in mm')
    ap.add_argument('--session', default='pass1', help='session label (for cross-session drift)')
    ap.add_argument('--out', default=None,
                    help='output path (default: <repo>/data/bgmodel_<session>_<ts>.json)')
    args = ap.parse_args()
    if args.out is None:
        os.makedirs(DATA_DIR, exist_ok=True)
        stamp = time.strftime('%Y%m%d-%H%M%S')
        args.out = os.path.join(DATA_DIR, f'bgmodel_{args.session}_{stamp}.json')
    else:
        d = os.path.dirname(os.path.abspath(args.out))
        os.makedirs(d, exist_ok=True)
    sys.exit(asyncio.run(run(args)))


if __name__ == '__main__':
    main()
