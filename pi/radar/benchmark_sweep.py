#!/usr/bin/env python3
"""Benchmark the SFCW sweep THROUGH the running sdr_server, over the websocket.

Rewritten 2026-09-05. The previous version drove SFCWEngine directly and had
rotted (it referenced `_sweep_core_fast`/`sweep_mode`/`_qt_profiles_rx`, all
removed, and unpacked `_sweep_core` as a 2-tuple when it returns 3).

Why the websocket and not the engine: `timestamp` is stamped in
`_process_h_cal`, and `Viewport.jsx` `useSweepRate` shows the median of adjacent
differences of it -- so the GUI reports the FULL loop, including `_sfcw_callback`
and the asyncio task doing `json.dumps` and the broadcast, all competing for CPU
with the sweep thread and the RX thread. Driving the engine directly excludes
that and reads ~7 ms fast. Run the FULL stack (`pi/start.py`) while measuring:
load is a variable here, not a constant, and extra idle subscriber clients are a
cheap and realistic way to add some.

Metrics, and why each one:
  ms/sweep    median of adjacent `timestamp` deltas -- what the GUI shows.
  vis         sweeps whose adjacent-sweep complex correlation of h_cal fell
              below 0.999. This is "visibly corrupted" to the operator: one bad
              step wrecks the whole sweep, because the range profile is a single
              IFFT across all steps.
  >8 sigma    per-(sweep, step) cells more than 8 robust sigmas off that step's
              own median. 51x more samples per sweep than `vis`, so it detects a
              rate change far sooner.
  S_repeat    signal energy over adjacent-sweep difference energy, /2. Immune to
              slow drift during a capture, unlike deviation-from-the-mean.

**400 sweeps is NOT enough to qualify a settle or timing change.** The failure
rate being chased is ~0.17% of sweeps, so a 400-sweep block reads 0 most of the
time -- the identical configuration has given 2/399 in one block and 0/1499 in
another. That is the trap that let the 2026-08-23 `settle_count` regression ship.
Use >=1200, prefer the per-step robust-z, and bracket every A/B with repeated
controls (ours agree to 0.07-0.2 ms).

Usage:
    python benchmark_sweep.py --label niosII_f --settle 0 --sweeps 1200
    python benchmark_sweep.py --label control --blocks 2      # bracketed
"""
import argparse
import asyncio
import json
import sys

import numpy as np

try:
    import websockets
except ImportError:
    sys.exit("needs `websockets` (pip install websockets)")

DEFAULT_URL = "ws://localhost:9003"
CORRUPT_BAR = 0.999   # adjacent-sweep complex correlation below this = visibly corrupted
Z_BAR = 8.0           # robust sigmas off a step's own median


async def run_block(ws, settle, n, warm, timeout, mode='standard',
                    dwell=4096, nios_settle=1024, pipeline=True):
    """One measurement block: set params, sweep, collect n results after `warm`."""
    await ws.send(json.dumps({'cmd': 'sfcw_set_params', 'settle_count': settle,
                              'sweep_mode': mode, 'nios_dwell': dwell,
                              'nios_settle': nios_settle,
                              'nios_pipeline': pipeline}))
    await asyncio.sleep(0.3)
    await ws.send(json.dumps({'cmd': 'sfcw_start'}))
    ts, H, got = [], [], 0
    while got < n + warm:
        msg = json.loads(await asyncio.wait_for(ws.recv(), timeout=timeout))
        if msg.get('type') != 'sfcw_result':
            continue
        got += 1
        if got <= warm:
            continue          # discard warm-up; the first sweeps follow a cold start
        ts.append(msg['timestamp'])
        H.append(np.asarray(msg['h_cal_real']) + 1j * np.asarray(msg['h_cal_imag']))
    await ws.send(json.dumps({'cmd': 'sfcw_stop'}))
    await asyncio.sleep(0.5)
    try:                       # drain anything still in flight
        while True:
            await asyncio.wait_for(ws.recv(), timeout=0.25)
    except Exception:
        pass
    return np.array(ts), np.array(H)


def score(H):
    """Per-step robust-z outliers, adjacent-sweep correlation, S_repeat."""
    med = np.median(H.real, 0) + 1j * np.median(H.imag, 0)
    dev = np.abs(H - med)
    mad = np.median(dev, 0)
    z = dev / np.where(mad > 0, mad * 1.4826, np.inf)
    diff = np.diff(H, axis=0)
    s_repeat = 10 * np.log10(
        (np.abs(H[1:]) ** 2).sum() / ((np.abs(diff) ** 2).sum() / 2))
    corr = np.array([
        abs(np.vdot(H[i - 1], H[i])) / (np.linalg.norm(H[i - 1]) * np.linalg.norm(H[i]))
        for i in range(1, len(H))])
    return z, corr, float(s_repeat)


async def main(args):
    async with websockets.connect(args.url, max_size=None) as ws:
        print(f"\n{'block':>16} {'ms':>8} {'Hz':>6} {'p10':>7} {'p90':>7} "
              f"{'S_rep':>7} {'corr_min':>9} {'>8sig':>13} {'worst z':>8} "
              f"{'vis corrupt':>13}", flush=True)
        blocks = []
        for b in range(args.blocks):
            ts, H = await run_block(ws, args.settle, args.sweeps, args.warm,
                                    args.timeout, mode=args.mode,
                                    dwell=args.dwell,
                                    nios_settle=args.nios_settle,
                                    pipeline=not args.no_pipeline)
            d = np.diff(ts) * 1000.0
            med = float(np.median(d))
            z, corr, s_repeat = score(H)
            vis = int((corr < CORRUPT_BAR).sum())
            bad = int((z > Z_BAR).sum())
            print(f"{args.label + '#' + str(b + 1):>16} {med:>8.1f} {1000 / med:>6.2f} "
                  f"{np.percentile(d, 10):>7.1f} {np.percentile(d, 90):>7.1f} "
                  f"{s_repeat:>7.2f} {corr.min():>9.5f} {bad:>5}/{H.size:<7} "
                  f"{z.max():>8.1f} {vis:>6}/{len(corr):<6}", flush=True)
            blocks.append(dict(ms=med, vis=vis, n=len(corr), bad=bad, cells=H.size))

        if args.blocks > 1:
            spread = max(b['ms'] for b in blocks) - min(b['ms'] for b in blocks)
            print(f"  control spread {spread:.2f} ms over {args.blocks} blocks "
                  f"-- this is the error bar on any comparison", flush=True)
        tv, tn = sum(b['vis'] for b in blocks), sum(b['n'] for b in blocks)
        tb, tc = sum(b['bad'] for b in blocks), sum(b['cells'] for b in blocks)
        print(f"  {args.label} TOTAL: {tv}/{tn} visibly corrupted, "
              f"{tb}/{tc} cells >{Z_BAR:g} sigma", flush=True)
        if tn < 1200:
            print("  WARNING: <1200 sweeps -- too few to qualify a settle or timing "
                  "change; a clean block at this size proves little.", flush=True)


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--label', default='run', help='name for the output rows')
    p.add_argument('--settle', type=int, default=0, help='settle_count to push (default 0)')
    p.add_argument('--sweeps', type=int, default=1200, help='sweeps per block (default 1200)')
    p.add_argument('--blocks', type=int, default=1, help='repeat blocks; >1 gives an error bar')
    p.add_argument('--warm', type=int, default=10, help='sweeps discarded at block start')
    p.add_argument('--url', default=DEFAULT_URL)
    p.add_argument('--timeout', type=float, default=25.0, help='per-message recv timeout (s)')
    p.add_argument('--mode', choices=('standard', 'nios'), default='standard',
                   help='sweep core: standard (USB retune per step) or nios (FPGA autonomous)')
    p.add_argument('--dwell', type=int, default=4096, help='nios dwell, samples (default 4096)')
    p.add_argument('--nios-settle', type=int, default=1024, help='nios per-step settle, samples')
    p.add_argument('--no-pipeline', action='store_true', help='disable capture pipelining in nios mode')
    asyncio.run(main(p.parse_args()))
