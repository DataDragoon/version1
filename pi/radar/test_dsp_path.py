#!/usr/bin/env python3
"""End-to-end test of the FPGA DSP result path, driven through sdr_server.

    python3 radar/test_dsp_path.py              # switch to dsp, sweep, check
    python3 radar/test_dsp_path.py --compare    # also capture nios first and
                                                # compare |h| between the two
    python3 radar/test_dsp_path.py --seconds 30 # collect for longer

Run on the Pi with the services up (start.py). It talks to sdr_server on
ws://127.0.0.1:9003 and does, in order:

    1. sfcw_stop            so set_params cannot block on the sweep lock
    2. sweep_mode = dsp     and reads it back
    3. sfcw_start           the engine rebuilds the stream: TX timestamped,
                            RX opened lazily by start_rx_dsp() on sweep 1
    4. collect results      for --seconds, counting sweep_core per result
    5. judge                see PASS / FAIL criteria below

PASS means every one of these held:
    - results arrived at all
    - every result carries sweep_core (absent = pre-aafe1c5 engine running)
    - the last LAST_N results all have sweep_core == 'dsp'
    - each h_cal has num_steps finite values and is not all zero

FAIL prints exactly what was seen, and which of the three explanations the
server console will carry (it is the only place the reason is printed):

    'fallback' every sweep     the DSP read failed; console says one of
                                 "buffer is raw samples ... bit 6 is not in
                                  effect"      -> FPGA/Nios GPIO path
                                 "read failed ... code -6"
                                              -> no burst: retunes not running
    'nios' with mode = dsp     the running engine predates 9525b54
    sweep_core absent          the running engine predates aafe1c5

The script changes the sweep mode and leaves it in 'dsp' on exit (pass
--leave nios to put it back). It never touches the FPGA image.
"""
import argparse
import asyncio
import json
import math
import sys
import time

try:
    import websockets
except ImportError:
    sys.exit("pip install websockets")

URI = "ws://127.0.0.1:9003"
LAST_N = 5          # the tail of the run must be all-'dsp' to pass


async def _drain_until(ws, want_type, pred=lambda m: True, timeout=10.0):
    """Return the first message of want_type satisfying pred, or None."""
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            m = json.loads(await asyncio.wait_for(ws.recv(), timeout=timeout))
        except asyncio.TimeoutError:
            return None
        if m.get('type') == want_type and pred(m):
            return m
    return None


async def _flush(ws, quiet=0.15):
    """Discard everything already queued on the socket.

    The server pushes an sfcw_status on connect, after sfcw_stop and after
    sfcw_set_params, on top of answering sfcw_get_status -- so without a
    flush a status request can be answered by a stale broadcast from before
    the command it is meant to check.
    """
    while True:
        try:
            await asyncio.wait_for(ws.recv(), timeout=quiet)
        except asyncio.TimeoutError:
            return


async def _status(ws, timeout=10.0):
    await _flush(ws)
    await ws.send(json.dumps({'cmd': 'sfcw_get_status'}))
    return await _drain_until(ws, 'sfcw_status', timeout=timeout)


async def _set_mode(ws, mode, timeout=5.0):
    """Request the mode, then wait for a status that actually reports it."""
    await _flush(ws)
    await ws.send(json.dumps({'cmd': 'sfcw_set_params', 'sweep_mode': mode}))
    st = await _drain_until(ws, 'sfcw_status',
                            pred=lambda m: m.get('sweep_mode') == mode,
                            timeout=timeout)
    if st is not None:
        return mode
    st = await _status(ws)
    return st.get('sweep_mode') if st else None


async def _stop(ws):
    await ws.send(json.dumps({'cmd': 'sfcw_stop'}))
    t0 = time.time()
    while time.time() - t0 < 15:
        st = await _status(ws, timeout=5)
        if st is not None and not st.get('running'):
            return True
        await asyncio.sleep(0.5)
    return False


async def _collect(ws, seconds):
    """Gather sfcw_result / sfcw_error messages for `seconds`."""
    results, errors = [], []
    t0 = time.time()
    while time.time() - t0 < seconds:
        try:
            m = json.loads(await asyncio.wait_for(ws.recv(), timeout=5))
        except asyncio.TimeoutError:
            continue
        t = m.get('type')
        if t == 'sfcw_result':
            results.append(m)
        elif t in ('sfcw_error', 'error'):
            errors.append(m.get('message'))
    return results, errors


def _h(m):
    re, im = m.get('h_cal_real') or [], m.get('h_cal_imag') or []
    return [complex(a, b) for a, b in zip(re, im)]


def _mag_stats(hs):
    mags = [abs(z) for h in hs for z in h]
    if not mags:
        return None
    mags.sort()
    return {'n': len(mags), 'median': mags[len(mags) // 2],
            'min': mags[0], 'max': mags[-1]}


async def run_mode(ws, mode, seconds):
    print(f"--- {mode}: stopping sweep")
    if not await _stop(ws):
        print("FAIL  could not stop the sweep (status never reported running=False)")
        return None
    got = await _set_mode(ws, mode)
    print(f"--- {mode}: sweep_mode reads back as {got!r}")
    if got != mode:
        print(f"FAIL  requested {mode!r} but engine reports {got!r} "
              f"(no status carried {mode!r} within 5 s of sfcw_set_params)")
        return None
    await ws.send(json.dumps({'cmd': 'sfcw_start'}))
    print(f"--- {mode}: started, collecting for {seconds:.0f} s")
    results, errors = await _collect(ws, seconds)
    return results, errors


def judge_dsp(results, errors):
    ok = True
    n = len(results)
    cores = {}
    for m in results:
        c = m.get('sweep_core', '<absent>')
        cores[c] = cores.get(c, 0) + 1
    print(f"      {n} result(s); sweep_core counts: {cores}")
    for e in errors:
        print(f"      sfcw_error: {e}")

    if n == 0:
        print("FAIL  no results at all -- sweep never produced anything")
        print("      (console: is NIOS primed? did sfcw_start error?)")
        return False
    if '<absent>' in cores:
        print("FAIL  results carry no sweep_core: the RUNNING engine predates "
              "aafe1c5. Restart the services after the pull.")
        return False

    tail = [m.get('sweep_core') for m in results[-LAST_N:]]
    if any(c != 'dsp' for c in tail):
        ok = False
        print(f"FAIL  last {len(tail)} sweep_core values: {tail}")
        if all(c == 'fallback' for c in tail):
            print("      Every sweep fell back: the DSP read is failing. The "
                  "reason is on the server console, one of:")
            print("        'buffer is raw samples ... bit 6 is not in effect' "
                  "-> bit 6 not reaching the FPGA mux (FPGA/Nios GPIO path)")
            print("        'read failed ... code -6'                          "
                  "-> no burst at all: retunes not running")
        elif 'nios' in tail:
            print("      Engine ran the nios core in dsp mode: the RUNNING engine "
                  "predates 9525b54. Restart the services after the pull.")
    else:
        print(f"PASS  last {len(tail)} results all sweep_core='dsp'")

    # value sanity on the dsp results
    dsp = [_h(m) for m in results if m.get('sweep_core') == 'dsp']
    if dsp:
        num_steps = results[-1].get('num_steps')
        bad = 0
        for h in dsp:
            if num_steps and len(h) != num_steps:
                bad += 1
                continue
            if not all(math.isfinite(z.real) and math.isfinite(z.imag) for z in h):
                bad += 1
                continue
            if all(z == 0 for z in h):
                bad += 1
        st = _mag_stats(dsp)
        print(f"      dsp |h|: n={st['n']} median={st['median']:.4f} "
              f"min={st['min']:.4f} max={st['max']:.4f}")
        if bad:
            ok = False
            print(f"FAIL  {bad}/{len(dsp)} dsp sweeps had wrong length, "
                  f"non-finite, or all-zero h_cal")
        else:
            print(f"PASS  {len(dsp)} dsp sweep(s): {num_steps} finite, "
                  f"non-zero values each")
    return ok


async def main(args):
    async with websockets.connect(URI, max_size=None) as ws:
        st = await _status(ws)
        if st is None:
            print("FAIL  no sfcw_status from the server")
            return 1
        print(f"server: sweep_mode={st.get('sweep_mode')} running={st.get('running')} "
              f"num_steps={st.get('num_steps')} nios_primed={st.get('nios_primed')}")

        ref = None
        if args.compare:
            r = await run_mode(ws, 'nios', args.seconds)
            if r is None:
                return 1
            ref = [_h(m) for m in r[0] if m.get('sweep_core') == 'nios']
            print(f"      nios: {len(r[0])} result(s), "
                  f"{len(ref)} with sweep_core='nios'")
            rs = _mag_stats(ref)
            if rs:
                print(f"      nios |h|: median={rs['median']:.4f} "
                      f"min={rs['min']:.4f} max={rs['max']:.4f}")

        r = await run_mode(ws, 'dsp', args.seconds)
        if r is None:
            return 1
        results, errors = r
        ok = judge_dsp(results, errors)

        if ok and ref:
            dsp = [_h(m) for m in results if m.get('sweep_core') == 'dsp']
            rs, ds = _mag_stats(ref), _mag_stats(dsp)
            if rs and ds and rs['median'] > 0:
                ratio = ds['median'] / rs['median']
                print(f"      |h| median dsp/nios = {ratio:.3f}  "
                      f"(same physics, both demodulated: expect ~1; a factor "
                      f"of ~2 or 1/2 means a Q14 / scaling mismatch)")

        if args.leave and args.leave != 'dsp':
            await _stop(ws)
            got = await _set_mode(ws, args.leave)
            await ws.send(json.dumps({'cmd': 'sfcw_start'}))
            print(f"--- left engine in sweep_mode={got!r}, sweep restarted")

        print("RESULT:", "PASS" if ok else "FAIL")
        return 0 if ok else 1


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--seconds', type=float, default=15.0,
                    help='collect results for this long per mode (default 15)')
    ap.add_argument('--compare', action='store_true',
                    help='capture a nios-mode run first and compare |h|')
    ap.add_argument('--leave', default='dsp', choices=['dsp', 'nios', 'standard'],
                    help="sweep mode to leave the engine in (default dsp)")
    a = ap.parse_args()
    try:
        sys.exit(asyncio.run(main(a)))
    except ConnectionRefusedError:
        sys.exit("cannot reach sdr_server on 127.0.0.1:9003 -- are the "
                 "services running (start.py)? Run this ON THE PI.")
