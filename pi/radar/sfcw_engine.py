"""Stepped-Frequency Continuous Wave (SFCW) radar engine.

Orchestrates the bladeRF to sweep through discrete frequency steps,
capture IQ at each, and compute range profiles via IFFT.

Uses dual-channel reference: TX1+RX1 for antenna signal, TX2+RX2 as
phase reference (short cable loopback). Dividing signal by reference
eliminates random PLL phase offsets between TX and RX synthesizers.
"""

import math
import threading
import time
import numpy as np

from bladerf_driver import BladeRFDriver
from bladerf._bladerf import ffi, libbladeRF
import bladerf

SPEED_OF_LIGHT = 299_792_458

# Master quick-tune table: covers the whole usable band at a fixed grid, generated
# once per device connection. Any sweep's start/stop/step is snapped onto this grid
# (see _snap_freq/_snap_step), so retuning never needs the table to be regenerated —
# start/stop/step can change freely at runtime with no device reset. See CLAUDE.md
# "Quick-tune master table" for the history of why this replaced per-grid caching.
#
# bladerf_get_quick_tune() isn't a stateless read — every call WRITES a new fastlock
# profile into a fixed-size on-device table (bladerf2.c: board_data->quick_tune_tx/
# rx_profile, capped at NUM_BBP_FASTLOCK_PROFILES). That counter only resets on a
# full device close+reopen. Past the cap, bladerf_get_quick_tune() returns an error
# and leaves the profile struct unpopulated — MAX_QUICK_TUNE_PROFILES here must stay
# under that hardware ceiling or the table silently contains garbage profiles for
# every frequency past it (this happened: a prior 1-6 GHz/10 MHz table needed 501
# profiles against a 256 cap).
MAX_QUICK_TUNE_PROFILES = 256  # NUM_BBP_FASTLOCK_PROFILES, fpga_common/bladerf2_common.h

# SC16_Q11 is 12-bit signed: +-2047 (the negative rail reaches -2048).
ADC_FULL_SCALE = 2047.0
# Fraction of full scale above which the AD9361 RX path compresses enough to matter.
# The two receivers need DIFFERENT thresholds even though it is the same front end,
# because adc_peak is a max over the sweep and the two channels' peaks mean different
# things. RX2 carries a flat CW reference (|h_reference| spans only 3.5 dB across
# 2-5 GHz), so its per-sweep max is representative of every step: 385 counts is clean,
# 896 already costs 6 dB of h_cal stability, 1780 costs 11 dB. RX1 carries the scene,
# whose level spans ~43 dB across the band, so its max is one strong step and says
# nothing about the other fifty -- measured 2026-08-29, RX1 peaking at 886 (43% FS)
# costs nothing detectable (h_cal 45.2 dB, and rx1_gain 20 vs 25 vs 30 all give
# |h_signal| cv ~1.1%). Warning on RX1 at 40% cried wolf on the normal configuration.
ADC_HOT_FRACTION_RX2 = 0.40   # reference: flat CW, max == typical
ADC_HOT_FRACTION_RX1 = 0.75   # signal: max is a single step, only real clipping matters
# The per-sweep peak sits right on the threshold in normal operation (RX1 measured
# flipping between 78% and 100% FS from one sweep to the next), so a bare
# threshold test flaps and prints a warning every second sweep. Warn only after a
# run of hot sweeps, and clear only after a longer run of clean ones.
ADC_HOT_SWEEPS_TO_WARN = 8
ADC_CLEAN_SWEEPS_TO_CLEAR = 30
QT_MASTER_START_FREQ = 2_000_000_000
QT_MASTER_STOP_FREQ = 5_000_000_000
# Base grids the master table covers. The table is the UNION of these over
# [QT_MASTER_START_FREQ, QT_MASTER_STOP_FREQ], so it is deliberately NOT uniformly
# spaced -- 2000, 2020, 2040, 2050, 2060, 2080, 2100, ...
#
# 20 alone could not represent a 50 MHz step: set_params snapped 50 -> 40 and the
# panel then described a sweep that was not the one running (61 steps and 1.0 m of
# range against the 76 steps and 1.37 m actually swept). Adding the 50 MHz family
# makes 50 exact.
#
# Cost, against the MAX_QUICK_TUNE_PROFILES = 256 hardware ceiling:
#   20 MHz -> 151 points, 50 MHz -> 61, overlap (multiples of 100) -> 31
#   union  -> 151 + 61 - 31 = 181 profiles, 75 under the cap.
# Adding a third family is NOT free -- check the union size against the cap first,
# _ensure_master_quick_tune_table() raises rather than silently storing garbage.
QT_MASTER_STEPS = (20_000_000, 50_000_000)
# The finest family. A sweep picks ONE family (see _snap_sweep) and every frequency
# it visits is a multiple of that family's base, which is what guarantees each one
# is in the union table.
QT_MASTER_STEP = min(QT_MASTER_STEPS)


def _round_half_up(x):
    """Round halves AWAY from zero, unlike Python's round(), which rounds to even.

    Both sides of the wire must agree on this: the groundstation mirrors the
    snapping in lib/sfcwGrid.js so its step count, sweep time and R max describe
    the sweep that will actually run, and JavaScript's Math.round is half-up. With
    Python's banker's rounding the two disagreed exactly on the .5 cases -- 50/20
    rounded to 2 here and 3 there.
    """
    return int(math.floor(float(x) + 0.5))


def master_grid_freqs():
    """The union grid, sorted. Pure function so it can be checked without hardware."""
    pts = set()
    for base in QT_MASTER_STEPS:
        f = QT_MASTER_START_FREQ
        while f <= QT_MASTER_STOP_FREQ:
            pts.add(f)
            f += base
    return sorted(pts)


class SFCWEngine:
    def __init__(self, driver: BladeRFDriver):
        self.driver = driver
        self.start_freq = 2_000_000_000
        self.stop_freq = 5_000_000_000
        self.step_size = 60_000_000
        # 1, not 4. num_buffers averages that many post-settle captures per step, which
        # only helps against noise that changes WITHIN a step -- and measured 2026-08-29
        # that noise is 0.029% (70.8 dB), while the system limit is the per-retune wobble
        # at 38.6 dB. Averaging 4 buffers buys 6 dB on a term already 32 dB below what
        # binds, i.e. nothing, and costs 3 buffer-times per step. NOTE this reverses the
        # 2026-08-23 restoration of 4 documented above: that was correct at the time,
        # when the reference was compressed and the within-step term was much closer to
        # the limit. If the RF chain regresses, this needs re-checking, not assuming.
        self.num_buffers = 1
        # 3, not 10. Validated 2026-08-29 PER STEP, which is what CLAUDE.md's settle_count
        # regression note says an aggregate metric failed to catch: 400 sweeps x 51 steps
        # = 20,400 step-captures at settle=3 produced ZERO cells more than 8 robust sigmas
        # off that step's median, worst excursion 2.9 sigma (Gaussian expectation for that
        # many samples is ~4.1). settle=1 is faster still (3.90 vs 3.35 Hz) and equally
        # clean on the aggregate, but threw a 7.6-sigma excursion in the same test -- the
        # exact intermittent tail that produced the earlier regression -- so 3 is chosen
        # for margin, not for speed. Do not drop below it without repeating the per-step
        # check; an aggregate correlation will not see this.
        self.settle_count = 3
        self.tx1_gain = 50
        self.rx1_gain = 25
        # Reference-channel (TX2 -> loopback cable -> RX2) gains. These set the level
        # the reference lands at on RX2's ADC, and that level is the single largest
        # driver of sweep-to-sweep variability in the whole system: h_cal = h_signal /
        # h_reference, so the reference's own instability is MULTIPLICATIVE and shows up
        # identically at every frequency step regardless of that step's signal level.
        # Measured 2026-08-29 (see CLAUDE.md "Sweep-to-sweep variability is set by the
        # REFERENCE channel's level"), 40 sweeps per point, peak RX2 ADC count over the
        # run vs h_cal sweep-to-sweep scatter -- all with the sync_rx fix in place:
        #   50/25 -> peak 1769 (86% FS) -> 33.6 dB   <- what the capture tools used to set
        #   45/20 -> peak 1616          -> 38.3 dB
        #   40/20 -> peak 1313          -> 43.8 dB
        #   35/20 -> peak  887          -> 45.5 dB
        #   30/20 -> peak  888          -> 46.2 dB   <- shipped, also 47.1 dB on a rerun
        #   25/20 -> peak  245          -> 45.8 dB
        #   15/30 -> peak 2048          -> 43.8 dB
        # Everything with a peak under ~900 counts sits within ~1.5 dB of optimal, which
        # is about the run-to-run spread; above ~1300 it degrades fast. So this is a broad
        # plateau with a cliff on the hot side, not a sharp optimum -- aim for a few
        # hundred counts and do not chase the last decibel.
        #
        # SUPERSEDED 2026-08-29 (later the same day) -- 45/5, not 30/20. Both earlier
        # picks came from scans scored by deviation-from-the-run-mean, which is inflated
        # by any bench drift during the capture and has no control bracket. Re-measured
        # with S_repeat (adjacent-sweep difference, drift-immune) and controls repeated at
        # the start AND end of every run, agreeing to 0.2 dB:
        #     tx2/rx2   S_repeat   range-profile floor   dB std (median)
        #     20/30      19.7 dB
        #     30/20      28.1 dB        -45.8 dBr             0.196
        #     40/10      36.7 dB
        #     45/10      38.6 dB        -52.1 dBr             0.067
        #     45/5       38.6 dB        -53.2 dBr             0.065
        # Monotonic in TX2 gain across a 19 dB span, and NOT a level effect: 45/5 sits at
        # 342 RX2 counts and 45/10 at 585, both 10.5 dB better than 30/20 at 391 counts in
        # between. The mechanism is NOT simply "match the two chains" -- tx1=45 with
        # tx2=45 (perfectly matched) measured 34.8 dB, worse than tx1=50/tx2=45's 38.7 --
        # so treat this as an empirical property of the AD9361 TX gain table at this
        # frequency plan, and RE-MEASURE it after any RF hardware change rather than
        # assuming it transfers.
        #
        # Do NOT raise these to "get more reference signal" -- more is strictly worse
        # once RX2 is compressing. adc_peak in every sfcw_result reports where it is.
        self.tx2_gain = 45
        self.rx2_gain = 5
        self.rx_gain_min = 5
        self.rx_gain_max = 38
        self.range_offset = 0.5
        self.bscan_avg_count = 1
        self.bscan_primer = False
        self.running = False
        self._stop_event = threading.Event()
        self._thread = None
        self._callback = None
        self._lock = threading.Lock()
        self._fpga_tuning = False
        self._gains_dirty = False
        self._warm = False
        self._sweep_lock = threading.Lock()
        # What the groundstation last asked for, before snapping. See
        # _apply_freq_grid for why the raw request has to survive.
        self._req_start = float(self.start_freq)
        self._req_stop = float(self.stop_freq)
        self._req_step = float(self.step_size)
        self._qt_master_freqs = None
        self._qt_master_rx = None
        self._qt_master_tx = None
        self._use_quick_tune = True
        self._last_adc_peak = None
        self._adc_hot_state = ()
        self._adc_hot_run = {'rx1': 0, 'rx2': 0}
        self._adc_clean_run = 0

    @property
    def num_steps(self):
        return int((self.stop_freq - self.start_freq) / self.step_size) + 1

    @property
    def bandwidth(self):
        return self.stop_freq - self.start_freq

    @property
    def range_resolution(self):
        if self.bandwidth == 0:
            return float('inf')
        return SPEED_OF_LIGHT / (2 * self.bandwidth)

    @property
    def max_range(self):
        if self.step_size == 0:
            return float('inf')
        return SPEED_OF_LIGHT / (2 * self.step_size)

    @staticmethod
    def _snap_to_base(value, base):
        snapped = _round_half_up(float(value) / base) * base
        return int(min(max(snapped, QT_MASTER_START_FREQ), QT_MASTER_STOP_FREQ))

    @staticmethod
    def _snap_sweep(start, stop, step):
        """Snap a requested sweep onto ONE of the master table's base grids.

        The table is the union of several bases, but a single sweep must stay
        inside one of them: mixing is not safe. Starting at 2020 (on the 20 grid)
        and stepping 50 visits 2070, which is on NEITHER family and so is not in
        the table at all. Picking one base and snapping start, stop AND step to
        multiples of it makes every visited frequency a multiple of that base,
        hence present by construction.

        The base chosen is whichever one can represent the requested STEP most
        closely; ties go to the finest, which gives the finer start/stop grid.
        Returns (start, stop, step), all snapped.
        """
        best = None
        for base in QT_MASTER_STEPS:
            snapped = max(base, _round_half_up(float(step) / base) * base)
            cand = (abs(snapped - float(step)), base, snapped)
            if best is None or cand[:2] < best[:2]:
                best = cand
        _, base, snapped_step = best
        return (SFCWEngine._snap_to_base(start, base),
                SFCWEngine._snap_to_base(stop, base),
                int(snapped_step))

    def _apply_freq_grid(self):
        """Re-snap all three from the values that were REQUESTED, not from the
        previously snapped ones.

        The base grid depends on the step, so changing the step can change which
        grid start/stop belong to -- and re-snapping an already-snapped value
        loses a little more each time. Keeping the raw request means the snap is
        idempotent no matter what order the panel sets things in.
        """
        self.start_freq, self.stop_freq, self.step_size = self._snap_sweep(
            self._req_start, self._req_stop, self._req_step)

    def set_params(self, **kwargs):
        with self._lock:
            grid_changed = False
            if 'start_freq' in kwargs:
                self._req_start = float(kwargs['start_freq'])
                grid_changed = True
            if 'stop_freq' in kwargs:
                self._req_stop = float(kwargs['stop_freq'])
                grid_changed = True
            if 'step_size' in kwargs:
                self._req_step = float(kwargs['step_size'])
                grid_changed = True
            if grid_changed:
                self._apply_freq_grid()
            if 'num_buffers' in kwargs:
                self.num_buffers = max(1, int(kwargs['num_buffers']))
            if 'settle_count' in kwargs:
                self.settle_count = max(1, int(kwargs['settle_count']))
            if 'tx1_gain' in kwargs:
                self.tx1_gain = int(kwargs['tx1_gain'])
                self._gains_dirty = True
            if 'rx1_gain' in kwargs:
                self.rx1_gain = int(kwargs['rx1_gain'])
                self._gains_dirty = True
            if 'tx2_gain' in kwargs:
                self.tx2_gain = int(kwargs['tx2_gain'])
                self._gains_dirty = True
            if 'rx2_gain' in kwargs:
                self.rx2_gain = int(kwargs['rx2_gain'])
                self._gains_dirty = True
            if 'rx_gain_min' in kwargs:
                self.rx_gain_min = int(kwargs['rx_gain_min'])
            if 'rx_gain_max' in kwargs:
                self.rx_gain_max = int(kwargs['rx_gain_max'])
            if 'range_offset' in kwargs:
                self.range_offset = float(kwargs['range_offset'])
            if 'bscan_avg_count' in kwargs:
                self.bscan_avg_count = max(1, int(kwargs['bscan_avg_count']))
            if 'bscan_primer' in kwargs:
                self.bscan_primer = bool(kwargs['bscan_primer'])

    def get_params(self):
        return {
            'start_freq': self.start_freq,
            'stop_freq': self.stop_freq,
            'step_size': self.step_size,
            'num_buffers': self.num_buffers,
            'settle_count': self.settle_count,
            'tx1_gain': self.tx1_gain,
            'rx1_gain': self.rx1_gain,
            'tx2_gain': self.tx2_gain,
            'rx2_gain': self.rx2_gain,
            'rx_gain_min': self.rx_gain_min,
            'rx_gain_max': self.rx_gain_max,
            'range_offset': self.range_offset,
            'num_steps': self.num_steps,
            'bandwidth': self.bandwidth,
            'range_resolution': self.range_resolution,
            'max_range': self.max_range,
            'bscan_avg_count': self.bscan_avg_count,
            'bscan_primer': self.bscan_primer,
        }

    def run_coherence_test(self, callback=None):
        """Run 3 consecutive sweeps and compute repeatability + correlation metrics.

        Runs in a new thread. Results sent via callback as a dict with type='coherence_result'.
        """
        if self.running:
            return
        self.running = True
        self._stop_event.clear()
        t = threading.Thread(target=self._coherence_test_worker, args=(callback,), daemon=True)
        t.start()

    def _coherence_test_worker(self, callback):
        try:
            self._configure_hardware()
            self._start_tx_rx()
            time.sleep(0.1)

            sweeps = []
            for i in range(3):
                if self._stop_event.is_set():
                    return
                if callback:
                    callback({'type': 'progress', 'step': i, 'total': 3, 'freq_mhz': 0})
                result = self._perform_sweep()
                if result and result.get('type') == 'range_profile':
                    h_cal = np.array(result['h_cal_real']) + 1j * np.array(result['h_cal_imag'])
                    sweeps.append(h_cal)

            if len(sweeps) < 2:
                if callback:
                    callback({'error': 'Not enough sweeps completed'})
                return

            reps = []
            corrs = []
            for i in range(len(sweeps) - 1):
                a_raw = sweeps[i]
                b_raw = sweeps[i + 1]
                residual = b_raw - a_raw
                rep = 1.0 - (np.std(residual) / np.std(a_raw))
                reps.append(float(rep))
                a = a_raw - np.mean(a_raw)
                b = b_raw - np.mean(b_raw)
                corr = np.abs(np.sum(a * np.conj(b))) / (
                    np.sqrt(np.sum(np.abs(a) ** 2)) * np.sqrt(np.sum(np.abs(b) ** 2))
                )
                corrs.append(float(corr))

            if callback:
                callback({
                    'type': 'coherence_result',
                    'repeatability': reps,
                    'correlation': corrs,
                    'avg_repeatability': float(np.mean(reps)),
                    'avg_correlation': float(np.mean(corrs)),
                    'num_sweeps': len(sweeps),
                })
        except Exception as e:
            if callback:
                callback({'error': str(e)})
        finally:
            self._stop_tx_rx()
            self.running = False

    def run_single(self, callback):
        """Run a single sweep and stop. Used for B-scan position captures."""
        if self._warm:
            self._callback = callback
            t = threading.Thread(target=self._warm_sweep_worker, args=(callback,), daemon=True)
            t.start()
            return
        if self.running:
            return
        self._callback = callback
        self._stop_event.clear()
        self.running = True
        self._thread = threading.Thread(target=self._single_sweep_worker, daemon=True)
        self._thread.start()

    def _warm_sweep_worker(self, callback):
        """Perform averaged sweeps with hardware already running (warm B-scan mode)."""
        with self._sweep_lock:
            try:
                if self.bscan_primer:
                    self._perform_sweep_raw()

                avg_count = self.bscan_avg_count
                if avg_count <= 1:
                    result = self._perform_sweep()
                else:
                    h_cal_accum = None
                    completed = 0
                    for i in range(avg_count):
                        raw = self._perform_sweep_raw()
                        if raw is None:
                            continue
                        if h_cal_accum is None:
                            h_cal_accum = raw.copy()
                        else:
                            h_cal_accum += raw
                        completed += 1
                    if completed == 0:
                        result = None
                    else:
                        h_cal_avg = h_cal_accum / completed
                        result = self._process_h_cal(h_cal_avg, self._last_adc_peak)
                if result is not None and callback:
                    callback(result)
            except Exception as e:
                print(f"[sfcw] Warm sweep error: {e}")
                if callback:
                    callback({'error': str(e)})

    def _single_sweep_worker(self):
        try:
            self._configure_hardware()
            self._start_tx_rx()
            time.sleep(0.1)
            result = self._perform_sweep()
            if result is not None and self._callback:
                self._callback(result)
        except Exception as e:
            print(f"[sfcw] Single sweep error: {e}")
            if self._callback:
                self._callback({'error': str(e)})
        finally:
            self._stop_tx_rx()
            self.running = False

    def warm_up(self):
        """Start hardware and keep it running for multiple on-demand sweeps (B-scan mode)."""
        if self._warm or self.running:
            return
        self._stop_event.clear()
        self._configure_hardware()
        self._start_tx_rx()
        time.sleep(0.1)
        self._perform_sweep_raw()
        self._warm = True
        self.running = True

    def cool_down(self):
        """Stop hardware after warm B-scan session."""
        if not self._warm:
            return
        self._stop_tx_rx()
        self._warm = False
        self.running = False

    def start(self, callback):
        if self.running:
            return
        self._callback = callback
        self._stop_event.clear()
        self.running = True
        self._thread = threading.Thread(target=self._sweep_loop, daemon=True)
        self._thread.start()

    def stop(self):
        if not self.running:
            return
        if self._warm:
            self.cool_down()
            return
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=5)
            self._thread = None
        self.running = False

    def _sweep_loop(self):
        try:
            self._configure_hardware()
            self._start_tx_rx()

            while not self._stop_event.is_set():
                if not self.driver.tx_running or not self.driver.rx_running:
                    print("[sfcw] ERROR: TX/RX stream died unexpectedly")
                    if self._callback:
                        self._callback({'error': 'USB stream died — restart sweep'})
                    break
                if self._gains_dirty:
                    self._apply_gains()
                range_profile = self._perform_sweep()
                if range_profile is not None and self._callback:
                    self._callback(range_profile)

        except Exception as e:
            print(f"[sfcw] Sweep error: {e}")
            if self._callback:
                self._callback({'error': str(e)})
        finally:
            self._stop_tx_rx()
            self.running = False

    def _ensure_master_quick_tune_table(self):
        """Generate the full-band quick_tune table once, covering QT_MASTER_START_FREQ..
        QT_MASTER_STOP_FREQ at QT_MASTER_STEP spacing.

        This is the one place that pays the full-VCO-cal cost (one bladerf_set_frequency
        per master grid point) and consumes the device's fixed BBP fastlock profile
        budget (MAX_QUICK_TUNE_PROFILES, see the module comment). It's independent of
        start_freq/stop_freq/step_size, so it only needs to happen once per device
        connection: after this, changing sweep params never requires a device reset,
        since every sweep's frequencies are just slices of this table (see
        _build_sweep_grid). Must be called before streaming starts and before switching
        to FPGA tuning mode (set_frequency needs normal tuning mode to calibrate).
        """
        if self._qt_master_freqs is not None:
            return

        freqs = np.array(master_grid_freqs(), dtype=np.int64)
        if len(freqs) > MAX_QUICK_TUNE_PROFILES:
            raise RuntimeError(
                f"Master quick-tune table needs {len(freqs)} profiles but the bladeRF2 "
                f"firmware caps BBP fastlock profiles at {MAX_QUICK_TUNE_PROFILES} per "
                f"direction. Narrow QT_MASTER_STOP_FREQ - QT_MASTER_START_FREQ, drop a "
                f"family from QT_MASTER_STEPS, or widen one, in sfcw_engine.py."
            )

        dev_ptr = self.driver.device.dev[0]

        qt_rx = []
        qt_tx = []
        for f in freqs:
            f_int = int(f)
            libbladeRF.bladerf_set_frequency(dev_ptr, bladerf.CHANNEL_RX(0), f_int)
            libbladeRF.bladerf_set_frequency(dev_ptr, bladerf.CHANNEL_TX(0), f_int)
            qr = ffi.new('struct bladerf_quick_tune *')
            qt_val = ffi.new('struct bladerf_quick_tune *')
            rc_rx = libbladeRF.bladerf_get_quick_tune(dev_ptr, bladerf.CHANNEL_RX(0), qr)
            rc_tx = libbladeRF.bladerf_get_quick_tune(dev_ptr, bladerf.CHANNEL_TX(0), qt_val)
            if rc_rx != 0 or rc_tx != 0:
                raise RuntimeError(
                    f"bladerf_get_quick_tune failed at {f_int/1e6:.0f} MHz "
                    f"(rx_rc={rc_rx}, tx_rc={rc_tx}) after {len(qt_rx)} profiles built — "
                    f"likely exhausted the device's {MAX_QUICK_TUNE_PROFILES}-profile "
                    f"fastlock table. A device reset reclaims the budget (fresh "
                    f"bladerf_open() resets the on-device counter to 0)."
                )
            qt_rx.append(qr)
            qt_tx.append(qt_val)

        self._qt_master_freqs = freqs
        self._qt_master_rx = qt_rx
        self._qt_master_tx = qt_tx
        bases = "/".join(f"{b/1e6:.0f}" for b in QT_MASTER_STEPS)
        print(f"[sfcw] Generated master quick_tune table: {len(freqs)} profiles "
              f"({QT_MASTER_START_FREQ/1e9:.2f}-{QT_MASTER_STOP_FREQ/1e9:.2f} GHz, "
              f"union of {bases} MHz grids, cap {MAX_QUICK_TUNE_PROFILES})")

    def invalidate_quick_tune_table(self):
        """Drop the cached master table so it regenerates on next use.

        Call after a device.reset() — a fresh device open can leave the AD9361 in a
        state where previously-captured quick_tune profiles no longer apply.
        """
        self._qt_master_freqs = None
        self._qt_master_rx = None
        self._qt_master_tx = None

    def _build_sweep_grid(self, start, stop, step):
        """This sweep's frequencies and, if available, their quick_tune profiles,
        looked up in the master table — no regeneration needed regardless of what
        start/stop/step are, as long as every frequency is ON the table
        (set_params guarantees this via _snap_sweep).

        Looked up BY FREQUENCY. The master table used to be a uniform 20 MHz grid,
        so an index could be computed arithmetically as
        `start_idx + i * (step / QT_MASTER_STEP)`. It is now the union of several
        base grids and is deliberately NOT uniformly spaced, so that arithmetic
        would silently address the wrong profiles — retuning each step to some
        other frequency while reporting the one that was asked for, which is
        exactly the failure mode the MAX_QUICK_TUNE_PROFILES check exists to
        prevent. searchsorted plus an exact-match assertion instead: if a
        frequency is not in the table, fail loudly rather than retune to its
        neighbour.
        """
        num_steps = int((stop - start) / step) + 1
        freqs = (start + np.arange(num_steps) * step).astype(np.int64)

        if self._use_quick_tune and self._qt_master_freqs is not None:
            master = self._qt_master_freqs
            idxs = np.clip(np.searchsorted(master, freqs), 0, len(master) - 1)
            if not np.array_equal(master[idxs], freqs):
                bad = freqs[master[idxs] != freqs]
                raise RuntimeError(
                    f"Sweep frequencies are not on the master quick-tune grid: "
                    f"{[int(b) for b in bad[:5]]} Hz (of {len(bad)}). start={start} "
                    f"stop={stop} step={step}. set_params()/_snap_sweep should make "
                    f"this impossible — the sweep was not snapped, or QT_MASTER_STEPS "
                    f"changed without the table being invalidated."
                )
            qt_rx = [self._qt_master_rx[k] for k in idxs]
            qt_tx = [self._qt_master_tx[k] for k in idxs]
            return master[idxs], qt_rx, qt_tx

        return freqs, None, None

    def _configure_hardware(self):
        self.driver.tx_gain = self.tx1_gain
        self.driver.rx_gain = self.rx1_gain
        self.driver.tx2_gain = self.tx2_gain
        self.driver.rx2_gain = self.rx2_gain
        self.driver.sample_rate = 10_000_000
        self.driver.bandwidth = 8_000_000
        self.driver.set_waveform('cw', offset=100_000, amplitude=0.9)
        if self._use_quick_tune:
            self._ensure_master_quick_tune_table()
        self.driver._configure_channels_dual()
        # NOTE: do NOT call driver.set_tuning_mode_fpga() here. On the bladeRF 2.0
        # micro, BLADERF_TUNING_MODE_FPGA accepts the call (rc=0) but then kills the
        # RX_X2 data path: sync_rx() starts timing out ~8 buffers later with
        # "Transfer timed out for RX buffer", so the sweep gets no data at all.
        # Bisected 2026-08-28 against libbladeRF 2.6.1 / FPGA 0.16.0 (reproduced with
        # both the flashed image and Nuand's official v0.16.0 loaded into RAM, so it
        # is not an FPGA-image problem). libbladeRF's own bladerf2 default_tuning_mode()
        # hardcodes mode = BLADERF_TUNING_MODE_HOST and only reaches FPGA mode via the
        # BLADERF_DEFAULT_TUNING_MODE=fpga env var, citing "errata related to
        # FPGA-based tuning" -- FPGA tuning is simply not a supported default here.
        # Host tuning costs nothing measurable: quick-tune bladerf_schedule_retune()
        # still works (rc=0) and a 51-step sweep runs in 230 ms (4.35 Hz).
        self._fpga_tuning = False

    def _start_tx_rx(self):
        self._rx_cond = threading.Condition()
        self._rx_latest = None
        self._rx_seq = 0
        n = 4096
        t = np.arange(n, dtype=np.float64) / self.driver.sample_rate
        self._ref_tone = np.exp(-1j * 2 * np.pi * self.driver.cw_offset * t)
        self._ref_tone_scaled = self._ref_tone / 2047.0
        self.driver.start_tx_dual()
        self.driver.start_rx_dual(self._rx_capture, num_samples=n)
        time.sleep(0.05)

        # enable_module() resets gain state, so re-push after modules are enabled.
        # driver.tx_gain/rx_gain/tx2_gain/rx2_gain were already synced from
        # self.tx1_gain/rx1_gain/tx2_gain/rx2_gain in _configure_hardware().
        self.driver.reapply_dual_gains()

    def _apply_gains(self):
        dev_ptr = self.driver.device.dev[0]
        libbladeRF.bladerf_set_gain(dev_ptr, bladerf.CHANNEL_TX(0), int(self.tx1_gain))
        libbladeRF.bladerf_set_gain(dev_ptr, bladerf.CHANNEL_TX(1), int(self.tx2_gain))
        libbladeRF.bladerf_set_gain(dev_ptr, bladerf.CHANNEL_RX(0), int(self.rx1_gain))
        libbladeRF.bladerf_set_gain(dev_ptr, bladerf.CHANNEL_RX(1), int(self.rx2_gain))
        self._gains_dirty = False

    def _stop_tx_rx(self):
        self.driver.stop_rx_dual()
        self.driver.stop_tx_dual()
        # Restore single-channel config so calib panel works after SFCW
        self.driver._configure_channels()



    def _rx_capture(self, rx1_iq, rx2_iq):
        with self._rx_cond:
            self._rx_latest = (rx1_iq, rx2_iq)
            self._rx_seq += 1
            self._rx_cond.notify_all()

    def _perform_sweep(self):
        with self._lock:
            start = self.start_freq
            stop = self.stop_freq
            step = self.step_size
            num_buffers = self.num_buffers
            settle_count = self.settle_count

        freqs, qt_rx, qt_tx = self._build_sweep_grid(start, stop, step)
        num_steps = len(freqs)

        def progress(i):
            if self._callback and i % 10 == 0:
                self._callback({
                    'type': 'progress',
                    'step': i,
                    'total': num_steps,
                    'freq_mhz': freqs[i] / 1e6,
                })

        h_cal, dropped_steps, adc_peak = self._sweep_core(
            freqs, qt_rx, qt_tx, num_buffers, settle_count, progress)
        if h_cal is None:
            return None

        if dropped_steps > 0:
            print(f"[sfcw] WARNING: {dropped_steps}/{num_steps} steps had incomplete captures")

        self._warn_if_adc_hot(adc_peak)
        return self._process_h_cal(h_cal, adc_peak)

    def _perform_sweep_raw(self):
        """Like _perform_sweep but returns raw h_cal array for averaging."""
        with self._lock:
            start = self.start_freq
            stop = self.stop_freq
            step = self.step_size
            num_buffers = self.num_buffers
            settle_count = self.settle_count

        freqs, qt_rx, qt_tx = self._build_sweep_grid(start, stop, step)

        h_cal, _, adc_peak = self._sweep_core(freqs, qt_rx, qt_tx, num_buffers, settle_count)
        self._last_adc_peak = adc_peak
        self._warn_if_adc_hot(adc_peak)
        return h_cal

    def _sweep_core(self, freqs, qt_rx, qt_tx, num_buffers, settle_count, progress_cb=None):
        """Sweep loop: retune, settle, capture num_buffers buffers and average them
        (noise averaging — 10*log10(num_buffers) dB of SNR for free), reference-divide.

        settle_count is the number of RX buffer arrivals to wait, after issuing a
        retune, before trusting the data — see CLAUDE.md's Sweep Timing / quick-tune
        regression note for why this matters and shouldn't be dropped carelessly.

        Returns (h_cal, dropped_steps) or (None, 0) if stopped.
        """
        num_steps = len(freqs)
        h_signal = np.zeros(num_steps, dtype=np.complex128)
        h_reference = np.zeros(num_steps, dtype=np.complex128)

        dev_ptr = self.driver.device.dev[0]
        tx_ch = bladerf.CHANNEL_TX(0)
        rx_ch = bladerf.CHANNEL_RX(0)

        use_qt = qt_rx is not None
        ref_tone_scaled = self._ref_tone_scaled
        rx_cond = self._rx_cond
        stop_event = self._stop_event

        dropped_steps = 0
        # Peak |I|,|Q| seen on each RX, in ADC counts of 2047 full scale. Nothing in
        # this repo checked ADC headroom before 2026-08-29, and a too-hot reference was
        # the entire cause of the variability investigated then -- it is cheap to
        # measure and it is the first thing to look at when sweeps get noisy.
        adc_peak_rx1 = 0.0
        adc_peak_rx2 = 0.0

        for i in range(num_steps):
            if stop_event.is_set():
                return None, 0

            f = int(freqs[i])
            if use_qt:
                libbladeRF.bladerf_schedule_retune(dev_ptr, rx_ch, 0, f, qt_rx[i])
                libbladeRF.bladerf_schedule_retune(dev_ptr, tx_ch, 0, f, qt_tx[i])
            else:
                libbladeRF.bladerf_set_frequency(dev_ptr, tx_ch, f)
                libbladeRF.bladerf_set_frequency(dev_ptr, rx_ch, f)

            with rx_cond:
                target_seq = self._rx_seq + settle_count
                while self._rx_seq < target_seq:
                    if not rx_cond.wait(timeout=1.0):
                        break

                sig_bufs = []
                ref_bufs = []
                last_seq = self._rx_seq
                for _ in range(num_buffers):
                    while self._rx_seq <= last_seq:
                        if not rx_cond.wait(timeout=1.0):
                            break
                    if self._rx_seq <= last_seq:
                        break
                    last_seq = self._rx_seq
                    sig_bufs.append(self._rx_latest[0])
                    ref_bufs.append(self._rx_latest[1])

            if sig_bufs:
                sig_arr = np.asarray(sig_bufs, dtype=np.float64)
                ref_arr = np.asarray(ref_bufs, dtype=np.float64)
                sig_cplx = (sig_arr[:, 0::2] + 1j * sig_arr[:, 1::2]) * ref_tone_scaled
                ref_cplx = (ref_arr[:, 0::2] + 1j * ref_arr[:, 1::2]) * ref_tone_scaled
                h_signal[i] = sig_cplx.mean()
                h_reference[i] = ref_cplx.mean()
                # max(|min|, max) rather than np.abs(...).max() -- two reductions with
                # no temporary allocation, which matters at ~3 sweeps/s on the Pi.
                p1 = max(-sig_arr.min(), sig_arr.max())
                if p1 > adc_peak_rx1:
                    adc_peak_rx1 = p1
                p2 = max(-ref_arr.min(), ref_arr.max())
                if p2 > adc_peak_rx2:
                    adc_peak_rx2 = p2
            else:
                dropped_steps += 1

            if progress_cb and i % 10 == 0:
                progress_cb(i)

        ref_mag = np.abs(h_reference)
        valid = ref_mag > 1e-10
        h_cal = np.zeros(num_steps, dtype=np.complex128)
        h_cal[valid] = h_signal[valid] / h_reference[valid]

        adc_peak = {
            'rx1': float(adc_peak_rx1),
            'rx2': float(adc_peak_rx2),
            'full_scale': float(ADC_FULL_SCALE),
        }
        return h_cal, dropped_steps, adc_peak

    def _warn_if_adc_hot(self, adc_peak):
        """Warn when an RX has been close enough to full scale to compress, sustained.

        Deliberately hysteretic. Sweeps free-run at 3-6 Hz and the per-sweep peak sits
        right on the threshold in ordinary operation, so a plain threshold test prints a
        warning and a recovery every couple of sweeps and buries everything else on
        stdout. A warning needs ADC_HOT_SWEEPS_TO_WARN consecutive hot sweeps and clears
        only after ADC_CLEAN_SWEEPS_TO_CLEAR consecutive clean ones.
        """
        if not adc_peak:
            return
        limits = {'rx1': ADC_HOT_FRACTION_RX1 * ADC_FULL_SCALE,
                  'rx2': ADC_HOT_FRACTION_RX2 * ADC_FULL_SCALE}
        hot = []
        for n in ('rx1', 'rx2'):
            if adc_peak.get(n, 0.0) > limits[n]:
                self._adc_hot_run[n] += 1
            else:
                self._adc_hot_run[n] = 0
            if self._adc_hot_run[n] >= ADC_HOT_SWEEPS_TO_WARN:
                hot.append(n)

        if hot:
            self._adc_clean_run = 0
            key = tuple(hot)
            if key != self._adc_hot_state:
                self._adc_hot_state = key
                detail = ', '.join(
                    f"{n.upper()}={adc_peak[n]:.0f}/{ADC_FULL_SCALE:.0f} "
                    f"({100 * adc_peak[n] / ADC_FULL_SCALE:.0f}% FS)" for n in hot)
                print(f"[sfcw] WARNING: RX ADC running hot -- {detail}. The front end is "
                      f"compressing; on RX2 (the reference) that raises the range-profile "
                      f"noise floor by up to 13 dB. Turn the corresponding gain down.")
        elif self._adc_hot_state:
            self._adc_clean_run += 1
            if self._adc_clean_run >= ADC_CLEAN_SWEEPS_TO_CLEAR:
                self._adc_hot_state = ()
                self._adc_clean_run = 0
                print("[sfcw] RX ADC levels back within headroom.")

    def _process_h_cal(self, h_cal, adc_peak=None):
        num_steps = len(h_cal)
        start = self.start_freq
        stop = self.stop_freq
        step = self.step_size

        phase_raw = np.angle(h_cal)
        phase_unwrapped = np.unwrap(phase_raw)
        coeffs = np.polyfit(np.arange(num_steps), phase_unwrapped, 1)
        residuals = phase_unwrapped - np.polyval(coeffs, np.arange(num_steps))
        phase_std = float(np.std(residuals))

        window = np.hanning(num_steps)
        h_windowed = h_cal * window
        nfft = num_steps * 4
        range_profile = np.fft.ifft(h_windowed, n=nfft)
        magnitude_db = 20 * np.log10(np.abs(range_profile) + 1e-12)

        max_range = SPEED_OF_LIGHT / (2 * step)
        distances = np.arange(nfft) / nfft * max_range - self.range_offset

        half = nfft // 2
        magnitude_db = magnitude_db[:half]
        distances = distances[:half]

        valid = distances >= 0
        distances = distances[valid]
        magnitude_db = magnitude_db[valid]

        h_cal_real = h_cal.real.tolist()
        h_cal_imag = h_cal.imag.tolist()

        return {
            'type': 'range_profile',
            'distances': distances.tolist(),
            'magnitudes': magnitude_db.tolist(),
            'h_cal_real': [round(v, 8) for v in h_cal_real],
            'h_cal_imag': [round(v, 8) for v in h_cal_imag],
            'range_resolution': SPEED_OF_LIGHT / (2 * (stop - start)),
            'unambiguous_range': max_range,
            'displayed_range_max': max_range / 2 - self.range_offset,
            'num_steps': num_steps,
            'step_size': step,
            # The true swept frequency axis, so the groundstation never has to
            # guess it from step_size alone (the Imaging Bench's dispersion and
            # raw-S21 views need the actual RF frequencies). stop_freq is the
            # last frequency actually visited, which equals self.stop_freq only
            # when the step divides the span evenly.
            'start_freq': int(start),
            'stop_freq': int(start + (num_steps - 1) * step),
            'range_offset': self.range_offset,
            'timestamp': time.time(),
            # Peak |I|,|Q| in ADC counts on each RX over the whole sweep, so the panel
            # can show headroom. RX2 (the reference) above ~40% of full scale means the
            # reference path is compressing, which raises the range-profile noise floor
            # by up to 16 dB -- see the tx2/rx2 defaults above.
            'adc_peak': adc_peak,
            'gains': {
                'tx1': self.tx1_gain, 'rx1': self.rx1_gain,
                'tx2': self.tx2_gain, 'rx2': self.rx2_gain,
            },
            'phase_coherence': {
                'phase_std_rad': phase_std,
                'phase_std_deg': float(np.degrees(phase_std)),
                'coherent': phase_std < 0.3,
                'slope_rad_per_step': float(coeffs[0]),
            },
        }
