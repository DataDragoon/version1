#!/usr/bin/env python3
"""Measure the real post-retune settling time, and show the frequency stepping.

    python3 radar/measure_settle.py          # 51 steps, default grid
    python3 radar/measure_settle.py 200      # override the dwell

Runs a genuine NIOS sweep, keeps the raw continuous capture instead of throwing
the settle region away, and answers two questions from the data itself:

  1. How long does the receiver ACTUALLY take to settle after a retune?
     The design assumes nios_settle = 1024 samples and the FPGA DSP chain
     discards DSP_FLUSH_N = 1088. If the true figure is much smaller, the dwell
     could be shortened and the sweep rate raised.

  2. Is the synthesiser really stepping?
     TX and RX retune together, so the baseband tone stays at cw_offset and its
     FREQUENCY does not change from step to step. What changes is PHASE -- the
     propagation delay through the target path is different at each RF
     frequency. A clean phase ramp across the 51 steps is the visible evidence
     that tuning happened.

Nothing here needs SignalTap or a rebuild: the transient samples are already
delivered to the host on the raw path. They are simply discarded by the normal
sweep code, which is why they are invisible on the display.

Run with sdr_server stopped -- it holds the device.
"""

import sys
import numpy as np

sys.path.insert(0, '/home/sfr/vikram_fpga_dump/version1/pi')
sys.path.insert(0, '/home/sfr/vikram_fpga_dump/version1/pi/radar')

from bladerf_driver import BladeRFDriver           # noqa: E402
import sfcw_engine as SE                           # noqa: E402


def bar(value, peak, width=44):
    n = 0 if peak <= 0 else int(round(width * min(value / peak, 1.0)))
    return '#' * n


def main():
    drv = BladeRFDriver()
    drv.open()
    eng = SE.SFCWEngine(drv)
    eng.sweep_mode = 'nios'

    if len(sys.argv) > 1:
        eng.nios_dwell = int(sys.argv[1])

    eng._configure_hardware()
    eng._start_tx_rx()

    freqs, qt_rx, qt_tx = eng._build_sweep_grid(
        eng.start_freq, eng.stop_freq, eng.step_size)
    n_steps = len(freqs)
    print("steps=%d  dwell=%d  settle=%d  rate=%.0f MS/s"
          % (n_steps, eng.nios_dwell, eng.nios_settle,
             drv.sample_rate / 1e6), flush=True)

    if not eng._nios_prime(freqs, qt_rx, qt_tx):
        sys.exit("prime failed")

    dwell = int(eng.nios_dwell)
    units = dwell // SE.NIOS_INTERVAL_UNIT
    inflight = eng._nios_launch(units, n_steps, dwell)
    if inflight is None:
        sys.exit("launch failed -- is the sweep firmware loaded?")
    harvested = eng._nios_harvest(inflight)
    if harvested is None:
        sys.exit("harvest timed out")
    rx1, rx2, _start = harvested

    a = np.concatenate(rx1).astype(np.float32)
    b = np.concatenate(rx2).astype(np.float32)
    sig = a[0::2] + 1j * a[1::2]          # antenna
    ref = b[0::2] + 1j * b[1::2]          # reference cable
    print("captured %d samples/channel (%.1f steps' worth)"
          % (len(sig), len(sig) / dwell), flush=True)

    if len(sig) < 4 * dwell:
        sys.exit("capture too short")

    # ---------------------------------------------------------------- 1
    # Locate the retune boundary by folding |d|x||/dn modulo the dwell.
    # A retune is the only periodic excursion in the stream.
    mag = np.abs(sig)
    d = np.abs(np.diff(mag))
    rows = len(d) // dwell
    fold = d[:rows * dwell].reshape(rows, dwell).mean(axis=0)
    boundary = int(np.argmax(fold))
    contrast = fold[boundary] / np.median(fold)
    print("\nretune boundary at offset %d in the dwell "
          "(folded over %d steps, %.1fx above median)"
          % (boundary, rows, contrast), flush=True)
    if contrast < 2.0:
        print("  NOTE: weak periodic feature -- the boundary estimate is "
              "unreliable, so the settling figure below may be too.", flush=True)

    # ---------------------------------------------------------------- 2
    # Settling profile: mean |x| per sample after the retune.
    rolled = np.roll(mag, -boundary)
    rows = len(rolled) // dwell
    prof = rolled[:rows * dwell].reshape(rows, dwell).mean(axis=0)

    tail_lo = min(2000, dwell - 600)
    steady = prof[tail_lo:tail_lo + 500].mean()

    settled = None
    for k in range(dwell - 200):
        if np.all(np.abs(prof[k:k + 200] - steady) < 0.05 * steady):
            settled = k
            break

    print("\n--- SETTLING ---", flush=True)
    print("steady-state |x| = %.1f ADC counts" % steady, flush=True)
    if settled is None:
        print("never settles within 5%% inside one dwell", flush=True)
    else:
        print("settles to within 5%% after %d samples (%.1f us)"
              % (settled, settled / drv.sample_rate * 1e6), flush=True)
    print("design discards  nios_settle = %d, DSP_FLUSH_N = 1088"
          % eng.nios_settle, flush=True)
    if settled is not None and settled < eng.nios_settle * 0.6:
        head = dwell - eng.nios_settle
        newd = settled + 64 + 2900
        print("\n  The design is conservative by %dx. A dwell of ~%d would "
              "still cover it,\n  which would raise the sweep rate from "
              "%.1f Hz to %.1f Hz."
              % (eng.nios_settle // max(settled, 1), newd,
                 drv.sample_rate / (n_steps * dwell),
                 drv.sample_rate / (n_steps * newd)), flush=True)

    peak = prof[:3000].max()
    print("\nmean |x| per sample after the retune:", flush=True)
    for k in (0, 20, 50, 100, 200, 400, 600, 800, 1000, 1088,
              1500, 2000, 3000):
        if k >= dwell:
            break
        print("  n=%-5d %8.1f  %+7.1f%%  %s"
              % (k, prof[k], 100 * (prof[k] - steady) / steady,
                 bar(prof[k], peak)), flush=True)

    # ---------------------------------------------------------------- 3
    # Frequency tuning, made visible: one complex ratio per step.
    print("\n--- FREQUENCY STEPPING ---", flush=True)
    win = dwell - eng.nios_settle - 128
    per_cycle = int(round(drv.sample_rate / drv.cw_offset))
    win -= win % per_cycle
    t = np.arange(win) / drv.sample_rate
    tone = np.exp(-1j * 2 * np.pi * drv.cw_offset * t)

    usable = min(len(sig), len(ref))
    nst = (usable - boundary) // dwell
    nst = min(nst, n_steps)
    phases = []
    for k in range(nst):
        s0 = boundary + k * dwell + eng.nios_settle + 64
        s = sig[s0:s0 + win]
        r = ref[s0:s0 + win]
        if len(s) < win or len(r) < win:
            break
        num = np.dot(s, tone)
        den = np.dot(r, tone)
        phases.append(np.angle(num / den) if abs(den) > 1e-9 else np.nan)

    ph = np.unwrap(np.array(phases))
    print("recovered %d steps; arg(RX1/RX2) unwrapped:" % len(ph), flush=True)
    for k in range(0, len(ph), max(1, len(ph) // 12)):
        print("  step %-3d  f = %8.3f GHz   phase = %+8.2f rad"
              % (k, freqs[k] / 1e9, ph[k]), flush=True)

    if len(ph) > 4:
        sl, _ = np.polyfit(np.arange(len(ph)), ph, 1)
        resid = ph - np.polyval([sl, ph[0]], np.arange(len(ph)))
        print("\n  phase slope   %+.4f rad/step" % sl, flush=True)
        print("  residual std   %.4f rad" % np.std(resid), flush=True)
        print("  A clean ramp means the synthesiser stepped as commanded.\n"
              "  A flat or noisy phase means it did not.", flush=True)

    drv.stop_rx_dual()
    drv.stop_tx_dual()
    drv.device.close()


if __name__ == '__main__':
    main()
