"""Stepped-Frequency Continuous Wave (SFCW) radar engine.

Orchestrates the bladeRF to sweep through discrete frequency steps,
capture IQ at each, and compute range profiles via IFFT.

Uses dual-channel reference: TX1+RX1 for antenna signal, TX2+RX2 as
phase reference (short cable loopback). Dividing signal by reference
eliminates random PLL phase offsets between TX and RX synthesizers.
"""

import math
import os
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

# Settle is gated on wall time since the retune, not on a count of delivered RX
# buffers -- see the long comment in _sweep_core. After the time gate the loop
# drains until a buffer arrives in LOCKSTEP with the hardware, which is what
# distinguishes "the hardware just produced this" from "this was sitting in the
# ring". MAX_BACKLOG_DRAIN bounds that at more than the 16-deep ring so a
# pathological stall cannot hang a sweep.
#
# The lockstep test is TWO-SIDED and that is the whole point. Let lag(n) be a
# buffer's queueing delay, T(n) its arrival: gap(n) = BP + lag(n) - lag(n-1). So
# the gap measures the CHANGE in staleness, and BOTH directions mean stale:
#   gap >> BP  the RX thread was descheduled and has just delivered the OLDEST
#              buffer in the ring -- lag jumped up by (gap - BP). This is the
#              stalest buffer available, and a one-sided `gap >= 0.5*BP` test
#              accepted it as proof of freshness. Inverted, and it was the bug.
#   gap << BP  the ring is non-empty, so sync_rx returns immediately and the gap
#              is just deinterleave time -- backlog being drained, also stale.
# Only gap ~= BP means lag did not change, and lag can only sit at its floor
# there, because a non-empty ring cannot produce a full-period gap.
# Measured 2026-09-05 over 172,901 buffers under full-stack load: 96.4% of gaps
# land in [0.9, 1.1]*BP and a >1.5*BP gap is followed by a <0.5*BP one 80.7% of
# the time -- the stall/drain signature, textbook. [0.8, 1.2] keeps 97.5%, so
# rejecting the rest costs one extra buffer wait on ~2.5% of steps.
#
# Do NOT replace this with a cumulative lag estimate (lag += gap - BP, clamped at
# 0). Tried: mean gap measures 0.4097 ms against a nominal 0.4096, and that 24 ppm
# accumulates to a 19 ms phantom lag within seconds. The gap band is drift-free
# because it never integrates.
# MAX_BACKLOG_DRAIN is DELIBERATELY NOT a cap on the drain loop any more -- see
# the long comment at the drain in _sweep_core. Capping the drain by a COUNT and
# then capturing anyway on exhaustion was the residual-corruption path (fixed
# 2026-09-06). It is kept only as the reference "how deep can the backlog
# legitimately be" figure: libbladeRF's ring is 16 buffers, so anything past this
# is a starvation burst rather than ordinary backlog.
MAX_BACKLOG_DRAIN = 24
# Wall-clock budget for reaching lockstep after the settle deadline. The drain is
# bounded by this instead of by a buffer count, because a late buffer is still at
# the correct frequency (the retune already happened) while an unproven one may
# not be -- so waiting is always the safe direction. 5x the worst RX-thread
# deschedule measured on this rig (50.9 ms).
STALL_GIVEUP_S = 0.25
LOCKSTEP_LO = 0.8
LOCKSTEP_HI = 1.2

# RX buffer geometry. These are TWO different numbers on purpose -- see below.
#
# RX_BUFFER_SAMPLES is how many samples per channel one RX buffer delivers, and
# it sets the clock the whole settle gate is expressed in: at settle_count = 0 a
# step waits one buffer period before the capture and then captures one, so the
# per-step cost is 2 * RX_BUFFER_SAMPLES / sample_rate. Halving it from 4096 to
# 2048 is worth 0.41 ms/step = 21 ms on a 51-step sweep (85.6 -> ~65 ms).
#
# It MUST stay an exact divisor-multiple of the DMA buffer libbladeRF is
# configured with in bladerf_driver.py's sync_config (buffer_size=4096), and
# that is the whole reason the demod length is a separate constant. _rx_loop_dual
# asks sync_rx for RX_BUFFER_SAMPLES * 2 (RX_X2 counts the request as the total
# across both channels), so the request is 4096 -- exactly one DMA buffer, the
# same clean alignment 4096/8192 had.
#
# What happens without that alignment is not subtle and it is what killed an
# earlier attempt to set this to 2000 directly. sync_rx serves a request out of
# whole DMA buffers and carries the remainder forward, so a request that does not
# divide the DMA buffer leaves a leftover that GROWS by (DMA - request) every
# call: at 4000 against 4096 the leftover walks 96 samples per call until, 43
# calls later, a request is served entirely from leftover and returns instantly.
# The data stays contiguous, so this is invisible to any correctness check -- but
# the returned samples end up to a full buffer period BEHIND the arrival that
# delivered them, and the arrival time is the only thing the settle gate can see.
# The gate reasons "this buffer arrived at T >= t_retune + buf_period, so its
# contents cover [T - buf_period, T], which starts at or after the retune". Under
# a walking leftover that reasoning is false by up to a whole period, so the
# capture straddles or entirely precedes the retune and holds the PREVIOUS
# frequency's IQ. Measured: S_repeat 33.8/36.6 dB -> 21.3/17.5 dB and 4/7
# corrupted sweeps per 1199 -> 183/233. That is far too large to be the ~3 dB of
# processing gain lost by shortening the correlation, and it is why the fix is
# alignment, not a longer settle.
#
# All of this was confirmed by direct measurement (SFCW_DIAG, 2026-09-06) before
# this pair of constants was chosen:
#   - DMA quantum: at n=2000 the dominant RX gap sits at 0.2046-0.2048 ms, NOT
#     the 0.2000 ms buf_period computes -- 0.2048 ms is exactly 2048 samples per
#     channel, so sync_config's buffer_size=4096 counts TOTAL samples in RX_X2
#     and the quantum is 2048/channel. RX_BUFFER_SAMPLES must therefore be a
#     multiple of 2048. Only 28.2% of gaps fell within 2% of the computed
#     buf_period, against 79.3% at n=4096.
#   - The gate was BLIND to the failure, exactly as predicted: locked = 100.00%,
#     capture margin never negative (min +8.1 us, same as n=4096's +9.2 us), and
#     the corrupted cells' margins were indistinguishable from the clean ones
#     (median 185.4 us both). More settle_count could never have fixed this --
#     the gate gates on arrival, and arrival had stopped tracking content.
#   - Reproduced collapse: 65.9 ms/sweep (timing exactly as intended) with
#     S_repeat 16.5 dB and 42/499 visibly corrupted, against a same-day n=4096
#     bracket of 28.8-35.1 dB and ~35/1199.
#
# DEMOD_SAMPLES is how many of those samples _sweep_core correlates against the
# reference tone, and it is 2000 rather than 2048 for an unrelated reason. The
# demod is mean(iq * exp(-j2pi*cw_offset*t)) over N samples -- a rectangular
# window -- so its rejection of anything that is not the tone is a sinc with
# nulls every sample_rate/N. LO leakage sits at DC, cw_offset away from the tone,
# so the rejection of DC is set by how many whole tone cycles fit the window:
#   N = 4096 -> 40.96 cycles, DC lands near a null by luck   -> about -60 dB
#   N = 2048 -> 20.48 cycles, DC lands almost exactly BETWEEN -> about -36 dB
#   N = 2000 -> 20.00 cycles, DC lands ON a null (as does every odd harmonic)
# 2048 would therefore have been 24 dB worse at rejecting LO leakage, which is
# the worst case available. The condition is that cw_offset * N / sample_rate be
# an integer -- at 10 Msps and 100 kHz that is any multiple of 100. Verified at
# runtime before making the change: the server reports cw_offset exactly 100000
# and sample_rate exactly 10000000, both plain ints feeding both the TX waveform
# and this tone, so 2000 gives 20.000 cycles and not 20.000-ish.
#
# Dropping the 48 unused samples costs 10*log10(2048/2000) = 0.10 dB of
# processing gain, against a within-step noise term already 70.8 dB down on a
# system limited at ~40 dB. It is free.
RX_BUFFER_SAMPLES = 2048
DEMOD_SAMPLES = 2000

# ---------------------------------------------------------------------------
# NIOS autonomous sweep (sweep_mode='nios') -- ported from fpga_branch
# (docs/nios_sweep.md has the protocol and the firmware side). Commands ride
# as sentinel values in the upper 32 bits of a retune2 packet's timestamp.
# 0x5357xxxx ("SW..") is ~1.8 million years out at 10 MSPS, so it can never be
# a real sample count and the normal retune path is unaffected. Requires the
# Nios II/f image with the rx.vhd tamer fix (hosted_niosII_f_sweep_ts.rbf);
# on stock firmware every path falls back to the standard sweep.
NIOS_CMD_PRIME = 0x53575052   # "SWPR"
NIOS_CMD_EXEC  = 0x53574550   # "SWEP"
NIOS_CMD_STOP  = 0x53575354   # "SWST"
NIOS_CMD_QUERY = 0x53575147   # "SWQG"
# fpga-stepper branch: the sweep run by the FPGA's sweep_stepper block.
# "SWLD" copies the primed profile table into the FPGA (sent once per prime);
# "SWEF" starts a sweep with arg = dwell in 80 MHz system-clock ticks. The
# Nios only sets port/SPDT and hands the AD9361 SPI to the FPGA, which
# preloads the next step's fast-lock profiles during the dwell (~90 us) and
# recalls them at an exact tick, so the step needs no jitter guard and is not
# bounded by the Nios's ~300 us retune rail.
NIOS_CMD_LOAD      = 0x53574C44   # "SWLD"
NIOS_CMD_EXEC_FPGA = 0x53574546   # "SWEF"
DSP_STEPPER        = True         # False: the Nios-timed EXEC of v12
SYS_CLOCK_HZ       = 80_000_000   # sweep_stepper's clock (U_system_pll)
# Shortest dwell the stepper can preload inside: 36 SPI frames at 20 MHz,
# ~90 us = ~920 samples at 10.24 MS/s; 1024 keeps a margin.
DSP_STEPPER_DWELL_FLOOR = 1024
NIOS_INTERVAL_UNIT = 64       # the dwell is sent divided by this
# Samples left unused at BOTH ends of each capture window. Alignment lands
# within a few tens of samples of the step boundary, and a window that starts
# even slightly early pulls the tail of the settling transient into the
# average -- degraded h_cal rather than a failure, so it is worth paying for.
NIOS_WINDOW_GUARD = 64
# Shortest dwell that holds on the II/f image, in samples at 10 Msps.
# Measured 2026-09-07 (Gate 2, dwell-vs-actual): the firmware rails at ~3030
# samples (~303 us/step) -- asked 2048 or 1024, it steps at ~3030 either way.
# 3456 is above the rail but UNSTABLE in practice: per-step period wobbles
# +/-14 samples and the slicer thrashes (S_repeat 13-19 dB, realignment on
# most sweeps). 4096 holds its period to +/-7, slices cleanly (S_repeat
# 36-37 dB, zero realignments over 80-sweep blocks), and with the pipeline the
# sweep is processing-bound anyway, so a shorter dwell buys nothing.
# (The Nios II/e floor was 20544 -- 2.003 ms/step, 42 SPI transactions at
# ~47.7 us of CPU each; that constant does not apply to this image.)
NIOS_MIN_DWELL = 4096
# Shortest dwell the on-FPGA DSP chain can work in, in samples.
#
# rx.vhd fixes DSP_FLUSH_N = 1088 (settle discard, matching the host's
# nios_settle 1024 + NIOS_WINDOW_GUARD 64) and DSP_ACCUM_N = 2900 (29 whole CW
# cycles) at COMPILE time. Both must fit inside one step or seq_adder's restart
# throws away a part-accumulated window and that step returns garbage. Changing
# this needs a new FPGA image, not a set_params call.
# rx.vhd DSP_FLUSH_N + DSP_ACCUM_N of the image in the board. v9/v10 were
# 1088 + 2900 = 3988; v11 is 1088 + 2400 = 3488. This is the floor the FPGA
# can physically meet, not a safe dwell: the Nios restart jitters by tens of
# microseconds, so the dwell must leave hundreds of samples above it.
DSP_MIN_DWELL = 1088 + 2400
# v12 defaults for dsp mode (this branch). FLUSH 512 + ACCUM 1600 = 2112
# samples/step (table indices 2, 2), dwell 3456: the shortest dwell the Nios
# II/f held stably in the 2026-09-07 measurement (it rails at ~3030 samples
# per retune; 3456 = 54 units of 64). Guard = 3456 - 2112 = 1344 samples.
# Sweep 51 x 3456 / 10.24 MS/s = 17.2 ms, ~58 Hz. 512 of settle is the value
# UNDER TEST: benchmark_sweep.py --mode dsp --flush N gives S_repeat vs settle.
DSP_DEFAULT_FLUSH_SEL = 2
DSP_DEFAULT_ACCUM_SEL = 2
# fpga-stepper: the step boundary is exact, so the dwell is need + a small
# guard: 512 + 1600 + 128 = 2240 (35 units of 64). 51 x 2240 / 10.24 MS/s =
# 11.2 ms, ~89 Hz acquisition. (Nios-timed v12: 3456.)
DSP_DEFAULT_DWELL     = 2240 if DSP_STEPPER else 3456
# dsp mode has no host-side slicer, so the NIOS_MIN_DWELL 4096 stability
# argument does not apply; the floor is the Nios's own retune rail.
DSP_DWELL_FLOOR       = 3072
# Consecutive missed bursts before the RX stream is torn down and rebuilt
# instead of just resynced (see _sweep_core_dsp).
DSP_MISSES_BEFORE_REBUILD = 5
# Hard cap on a bulk capture, in RX buffers (~0.25 s at 2048-sample buffers).
# A pipelined inflight capture between on-demand sweeps (warm B-scan mode)
# would otherwise grow without bound at ~78 MB/s. The sweep itself completes
# within ~35 ms of EXEC, so a capped capture still contains the whole train;
# _rx_capture simply stops appending at the cap and harvest proceeds normally.
NIOS_BULK_MAX_BUFFERS = 1220
# How often a run of fallbacks is summarised on stdout (seconds).
NIOS_FALLBACK_LOG_PERIOD_S = 30.0

BLADERF_RX = libbladeRF.BLADERF_RX


def rx_ring_depth():
    # Depth of the driver's RX sync ring, in buffers (lazy import so the
    # pure-math parts of this module stay importable without the driver).
    try:
        from bladerf_driver import RX_RING_DEPTH
        return RX_RING_DEPTH
    except Exception:
        return 16


# Diagnostics for the settle gate, off unless SFCW_DIAG names a path prefix.
# Costs one array store per RX buffer and one per step when on, nothing when off.
# Writes <prefix>_gaps.npy (every RX inter-arrival gap, seconds) and
# <prefix>_steps.npy (per accepted step: step index, drains, accepted gap,
# seconds from retune to acceptance, buffers judged backlog, locked flag).
SFCW_DIAG = os.environ.get('SFCW_DIAG')
DIAG_MAX_GAPS = 4000000
DIAG_MAX_STEPS = 1000000

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
        # 0, and 0 does NOT mean "no settling" -- read the deadline comment in
        # _sweep_core first. The gate always waits one whole buffer period beyond
        # settle_count so a capture cannot straddle the retune; settle_count is
        # EXTRA settling on top of that, and extra settling was measured to buy
        # nothing while costing 0.41 ms per step (21 ms/sweep per unit).
        #
        # This supersedes the 2026-08-29 choice of 3. That value was margin against
        # an intermittent corrupted step which has since been traced to two real
        # bugs in the gate (an inverted one-sided lockstep test, and gating on a
        # buffer's arrival rather than its contents) rather than to RF settling --
        # raising settle_count only ever bought margin by accident, which is why
        # 163,200 step-captures over settle 1..10 had shown no trend. With both
        # fixed, validated PER STEP as CLAUDE.md requires: 5,100 consecutive sweeps
        # at settle_count=0 through the running server -- unloaded, and with four
        # concurrent websocket clients -- gave ZERO visibly corrupted sweeps and
        # one cell beyond 8 robust sigma in 261,142, against the ~1-in-40,000
        # settle-independent background this repo already measured. 168.3 ms/sweep
        # against 231.1 at settle_count=3.
        #
        # Do not raise it to chase a corrupted sweep without first checking the
        # per-step diagnostics (SFCW_DIAG): if the gate is working, the fault is
        # not settling and more settling will not fix it.
        self.settle_count = 0
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
        self.range_offset = 0.378
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
        # --- NIOS autonomous sweep state (see the constants block) ---
        # 'standard' is the validated USB-per-step sweep and stays the default.
        # 'nios' hands the whole sweep to the FPGA firmware and slices one
        # continuous capture at the step boundaries; every failure path in
        # _sweep_core_nios falls back to 'standard' and says why.
        # 'nios' was the default from 2026-09-07: bracketed 2x1200-sweep blocks
        # through the full stack measured 28.6/28.8 ms (34.8 Hz) at S_repeat
        # 35.2/34.9 dB against the standard sweep's 55.4/55.2 ms -- a 1.93x
        # win at equal-or-better quality, with every failure falling back to
        # one standard sweep. 'standard' remains the fully-validated
        # host-driven core; one sfcw_set_params reverts to either.
        #
        # 'dsp' by default as of 2026-09-09, with FPGA image v7 or later. It
        # keeps the NIOS stepping the synthesizers and additionally takes the
        # RESULT from the FPGA: rx.vhd mixes, accumulates and divides on chip
        # and hands back one 64-bit ratio per step, so a sweep is 408 bytes
        # instead of 835 KB and the Pi computes no demodulation and no
        # division. It also removes the ring-overflow fallback by
        # construction -- there is no continuous capture to overflow.
        #
        # NOT the default. 'dsp' has never completed a sweep on hardware, and
        # defaulting to it made every startup depend on it -- the raw path is
        # the one that is known to work and it stays in charge until 'dsp' has
        # proven itself.
        #
        # Select it explicitly, and only when FPGA v7+ is flashed:
        #
        #     {"sweep_mode": "dsp"}      over the sdr_server WebSocket
        #
        # then stop and restart the sweep -- the sample format is fixed when
        # the stream is configured, so the mode cannot cross on a live stream.
        # Switch back with {"sweep_mode": "nios"}.
        #
        # v5/v6 cannot run it at all: without the Nios toggling RFFE GPO bit 24
        # on every retune the accumulators never restart and the DSP FIFO never
        # fills.
        self.sweep_mode = 'nios'
        self.nios_dwell = 4096        # samples per step, rounded to 64 -- see NIOS_MIN_DWELL
        # dsp mode has its own dwell and chain counts (v12); see DSP_DEFAULT_*.
        self.dsp_dwell = DSP_DEFAULT_DWELL
        self.dsp_flush_sel = DSP_DEFAULT_FLUSH_SEL
        self.dsp_accum_sel = DSP_DEFAULT_ACCUM_SEL
        self._dsp_chain_dirty = True
        self.nios_settle = 1024       # samples dropped at the start of a step
        # Overlap the next sweep's EXEC+capture with this sweep's processing.
        # This is where most of the speed lives (48 -> 28.5 ms engine-direct);
        # the cost is that a failure surfaces one sweep late, as a fallback.
        self.nios_pipeline = True
        self._nios_primed = False
        self._nios_primed_steps = 0
        self._nios_primed_key = None
        self._nios_step_offset = 0
        self._nios_last_offset = 0
        self._nios_last_drift = 0
        self._nios_last_score = 0.0
        self._nios_fail = None
        self._nios_realigned = 0
        self._nios_period = 0.0
        self._nios_last_phase = 0
        self._nios_capture_loss = 0.0
        self._nios_capture_lag = 0
        self._nios_period_hist = []
        self._nios_inflight = None
        self._nios_tone = None
        self._nios_tone_len = None
        self._nios_cmd_profile = None
        # Latched when the loaded FPGA image cannot run the autonomous
        # sweep (stock firmware enqueues the sentinels and the sample
        # counter reads 0) -- e.g. after a power cycle reverts to the SPI
        # image. One clear line instead of a prime-and-fail per sweep.
        self._nios_unavailable = False
        self._last_sweep_core = 'standard'
        self._nios_fallbacks = 0
        self._nios_fb_window_start = None
        self._nios_fb_window_count = 0
        self._nios_sweeps_since = 0
        self._nios_last_span_ok = None
        self._bulk_capture = False
        self._bulk_rx1 = []
        self._bulk_rx2 = []
        self._bulk_start_sample = 0
        self._bulk_start_seq = 0
        self._bulk_max_buffers = NIOS_BULK_MAX_BUFFERS

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
                # The NIOS holds a recorded copy of the old grid.
                self._nios_primed = False
                if self._nios_inflight is not None:
                    self._nios_discard_inflight()
            if 'num_buffers' in kwargs:
                self.num_buffers = max(1, int(kwargs['num_buffers']))
            if 'settle_count' in kwargs:
                # 0 is legal and is the default: the gate ALWAYS waits one buffer
                # period beyond this so the capture cannot straddle the retune,
                # and settle_count is settling asked for on top of that. See
                # _sweep_core.
                self.settle_count = max(0, int(kwargs['settle_count']))
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
            if 'sweep_mode' in kwargs:
                mode = str(kwargs['sweep_mode'])
                if mode in ('standard', 'nios', 'dsp') and mode != self.sweep_mode:
                    # 'dsp' streams PACKET_META and the others stream SC16_Q11,
                    # and the format is fixed when the stream is configured --
                    # so crossing into or out of 'dsp' needs the RX stream torn
                    # down and rebuilt, which only happens in _start_tx_rx.
                    # Flag it rather than silently running the new mode over a
                    # stream configured for the old one.
                    if ('dsp' in (mode, self.sweep_mode)
                            and self.driver.rx_running):
                        print(f"[sfcw] sweep_mode {self.sweep_mode} -> {mode} "
                              f"needs the RX stream restarted (different "
                              f"sample format); stop and start the sweep.")
                    self.sweep_mode = mode
                    if self._nios_inflight is not None:
                        self._nios_discard_inflight()
            if 'nios_dwell' in kwargs:
                new_val = max(NIOS_INTERVAL_UNIT, int(kwargs['nios_dwell']))
                if new_val < NIOS_MIN_DWELL:
                    print(f"[sfcw] nios_dwell {new_val} is below the "
                          f"{NIOS_MIN_DWELL}-sample floor the FPGA can hold; "
                          f"clamping")
                    new_val = NIOS_MIN_DWELL
                if new_val != self.nios_dwell:
                    self.nios_dwell = new_val
                    # The steady-median override in _nios_refine_offset would
                    # otherwise carry the OLD dwell's period into the new grid
                    # and slice every sweep at the wrong stride.
                    self._nios_period_hist = []
                    if self._nios_inflight is not None:
                        self._nios_discard_inflight()
            if 'nios_settle' in kwargs:
                self.nios_settle = max(0, int(kwargs['nios_settle']))
            # v12 DSP chain counts (table indices, see BladeRFDriver.DSP_*_TABLE).
            # Applied before the next sweep's EXEC, when the chain is idle.
            if 'dsp_flush_sel' in kwargs:
                self.dsp_flush_sel = max(0, min(7, int(kwargs['dsp_flush_sel'])))
                self._dsp_chain_dirty = True
            if 'dsp_accum_sel' in kwargs:
                self.dsp_accum_sel = max(0, min(7, int(kwargs['dsp_accum_sel'])))
                self._dsp_chain_dirty = True
            if 'dsp_dwell' in kwargs:
                units = max(1, int(kwargs['dsp_dwell']) // NIOS_INTERVAL_UNIT)
                new_val = units * NIOS_INTERVAL_UNIT
                floor = DSP_STEPPER_DWELL_FLOOR if DSP_STEPPER else DSP_DWELL_FLOOR
                if new_val < floor:
                    print(f"[sfcw] dsp_dwell {new_val} is below the floor "
                          f"({floor}: {'stepper preload' if DSP_STEPPER else 'Nios retune rail'}); clamping")
                    new_val = floor
                self.dsp_dwell = new_val
            if 'nios_pipeline' in kwargs:
                self.nios_pipeline = bool(kwargs['nios_pipeline'])
                if not self.nios_pipeline and self._nios_inflight is not None:
                    self._nios_discard_inflight()

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
            'sweep_mode': self.sweep_mode,
            'nios_dwell': self.nios_dwell,
            'nios_settle': self.nios_settle,
            'nios_primed': self._nios_primed,
            'dsp_dwell': getattr(self, 'dsp_dwell', DSP_DEFAULT_DWELL),
            'dsp_flush_sel': getattr(self, 'dsp_flush_sel', 0),
            'dsp_accum_sel': getattr(self, 'dsp_accum_sel', 0),
            # what the FPGA is actually running (after the last apply)
            'dsp_flush_n': self._dsp_chain_counts()[0],
            'dsp_accum_n': self._dsp_chain_counts()[1],
        }

    # ---- v12 DSP chain counts -------------------------------------------
    def _dsp_chain_counts(self):
        """(flush_n, accum_n) the FPGA is running: the last applied pair, or
        what the current selection would give on a v12 image."""
        applied = getattr(self, '_dsp_chain_applied', None)
        if applied is not None:
            return applied
        d = self.driver
        return (d.DSP_FLUSH_TABLE[getattr(self, 'dsp_flush_sel', 0)],
                d.DSP_ACCUM_TABLE[getattr(self, 'dsp_accum_sel', 0)])

    def _dsp_apply_chain(self):
        """Push the selection to the FPGA (between sweeps) and remember what
        it reports back. On v11 and earlier this records the compile-time
        counts, so the dwell check below stays truthful."""
        fs = getattr(self, 'dsp_flush_sel', 0)
        ac = getattr(self, 'dsp_accum_sel', 0)
        flush_n, accum_n, supported = self.driver.dsp_set_chain(fs, ac)
        self._dsp_chain_applied = (flush_n, accum_n)
        self._dsp_chain_dirty = False
        print(f"[sfcw] DSP chain: FLUSH {flush_n} + ACCUM {accum_n} = "
              f"{flush_n + accum_n} samples/step"
              f"{'' if supported else ' (image has no runtime select)'}; "
              f"dwell {self.dsp_dwell} leaves {self.dsp_dwell - flush_n - accum_n} "
              f"of guard")

    # Sweeps in a coherence test. Was 3 -- two adjacent pairs, which says
    # nothing about a 1-in-20 miss or slow drift. 100 sweeps is ~2 s in dsp
    # mode (48 Hz) and ~3 s in nios mode.
    COHERENCE_SWEEPS = 100

    def run_coherence_test(self, callback=None, num_sweeps=None):
        """Run consecutive sweeps and compute repeatability + correlation metrics.

        Runs in a new thread. Results sent via callback as a dict with
        type='coherence_result'. num_sweeps defaults to COHERENCE_SWEEPS.
        """
        if self.running:
            return
        self.running = True
        self._stop_event.clear()
        n = int(num_sweeps) if num_sweeps else self.COHERENCE_SWEEPS
        n = max(2, n)
        t = threading.Thread(target=self._coherence_test_worker,
                             args=(callback, n), daemon=True)
        t.start()

    def _coherence_test_worker(self, callback, num_sweeps):
        try:
            self._configure_hardware()
            self._start_tx_rx()
            time.sleep(0.1)

            sweeps = []
            cores = {}
            for i in range(num_sweeps):
                if self._stop_event.is_set():
                    return
                if callback and (i % 10 == 0 or i == num_sweeps - 1):
                    callback({'type': 'progress', 'step': i, 'total': num_sweeps, 'freq_mhz': 0})
                result = self._perform_sweep()
                if result and result.get('type') == 'range_profile':
                    core = result.get('sweep_core', '?')
                    cores[core] = cores.get(core, 0) + 1
                    # A fallback sweep in dsp mode is all zeros by design; it
                    # must not be scored as a decorrelation.
                    if core == 'fallback':
                        continue
                    h_cal = np.array(result['h_cal_real']) + 1j * np.array(result['h_cal_imag'])
                    sweeps.append(h_cal)

            if len(sweeps) < 2:
                if callback:
                    callback({'error': 'Not enough sweeps completed '
                                       f'({len(sweeps)} of {num_sweeps}; cores {cores})'})
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

            # S_repeat: signal energy over adjacent-sweep difference energy, /2
            # (the same figure benchmark_sweep.py reports). Drift-immune.
            arr = np.array(sweeps)
            diff = np.diff(arr, axis=0)
            e_sig = float(np.mean(np.abs(arr) ** 2))
            e_dif = float(np.mean(np.abs(diff) ** 2))
            s_repeat_db = 10.0 * np.log10(e_sig / (e_dif / 2.0)) if e_dif > 0 else float('inf')

            if callback:
                callback({
                    'type': 'coherence_result',
                    'repeatability': reps,
                    'correlation': corrs,
                    'avg_repeatability': float(np.mean(reps)),
                    'avg_correlation': float(np.mean(corrs)),
                    'min_correlation': float(np.min(corrs)),
                    's_repeat_db': float(s_repeat_db),
                    'num_sweeps': len(sweeps),
                    'requested_sweeps': num_sweeps,
                    'sweep_cores': cores,
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
                # In 'dsp' the RX stream is opened per sweep, immediately
                # before EXEC, and torn down again if a read fails -- see the
                # note in _start_tx_rx. rx_running is therefore legitimately
                # False here on the first pass and between sweeps, so requiring
                # it would abort the loop before a single sweep had run. TX is
                # continuous in every mode and is still checked.
                rx_required = (self.sweep_mode != 'dsp')
                if not self.driver.tx_running or (
                        rx_required and not self.driver.rx_running):
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
        # The hardware sample counter restarts with the stream, and the NIOS
        # forgets its recorded sweep across a re-prime anyway. A fresh stream
        # also re-probes firmware capability (the image may have been
        # reloaded since the last session).
        self._nios_primed = False
        self._nios_unavailable = False
        self._nios_inflight = None
        self._bulk_capture = False
        self._bulk_rx1 = []
        self._bulk_rx2 = []
        self._rx_t = None
        self._rx_gap = 0.0
        self._diag_gaps = None
        if SFCW_DIAG:
            self._diag_gaps = np.zeros(DIAG_MAX_GAPS, dtype=np.float64)
            self._diag_gaps_n = 0
            self._diag_steps = np.zeros((DIAG_MAX_STEPS, 8), dtype=np.float64)
            self._diag_steps_n = 0
        n = RX_BUFFER_SAMPLES
        self._rx_buffer_samples = n
        # The tone is DEMOD_SAMPLES long, not n -- see the constants above. The
        # demod slices each buffer down to this length, so the two must agree.
        self._demod_int16 = DEMOD_SAMPLES * 2
        t = np.arange(DEMOD_SAMPLES, dtype=np.float64) / self.driver.sample_rate
        self._ref_tone = np.exp(-1j * 2 * np.pi * self.driver.cw_offset * t)
        self._ref_tone_scaled = self._ref_tone / 2047.0
        # complex64 copy for the hot demod path in _sweep_core. float32 pairs view
        # directly as complex64, so the int16 -> complex conversion there is one
        # astype plus a free view; the dot then runs in complex64 too. Checked over
        # 200 randomised trials (1/2/4 buffers, 20-2000 ADC counts) against the
        # float64 expression this replaced: worst relative error 1.4e-5 (-97.2 dB),
        # against a system limited at ~42 dB S_repeat. 55 dB of margin.
        self._ref_tone_c64 = self._ref_tone_scaled.astype(np.complex64)
        # TX and RX must agree on timestamps: the enable is one global GPIO
        # bit, and perform_format_config() (bladerf2/common.c) returns
        # BLADERF_ERR_INVAL if one direction's format needs them and the
        # other's does not. Every mode now streams plain SC16_Q11 on RX --
        # dsp mode included, since start_rx_dsp() moved to sample mode in
        # aafe1c5 -- so TX is plain SC16_Q11 everywhere. (Until 2026-09-11 dsp
        # mode still started TX as SC16_Q11_META from its PACKET_META days;
        # start_rx_dsp() then failed sync_config with ERR_INVAL before any
        # read, and every sweep fell back in ~3 ms: 5299 'fallback' results
        # in 15 s, nothing ever asked of the FPGA.)
        self.driver.start_tx_dual(timestamped=False)
        if self.sweep_mode == 'dsp':
            # DELIBERATELY NOT STARTED HERE.
            #
            # In DSP mode the FPGA emits nothing at all until a whole sweep has
            # landed in the DSP FIFO -- the gate holds the read side empty by
            # design. An RX stream opened now would then sit through the ~51
            # USB retunes of NIOS priming with no data to deliver, every queued
            # transfer would time out, the stream would error, and
            # sync_worker.c would move the worker to STOPPED. The first real
            # read then fails in sync_prime_stream, which returns
            # BLADERF_ERR_UNEXPECTED (-1) for any state that is not RUNNING or
            # IDLE. That is the "unexpected error" the DSP path was hitting,
            # and the transfer timeouts logged alongside it were its cause, not
            # its symptom.
            #
            # _sweep_core_dsp opens the stream immediately before EXEC instead,
            # so it is never idle for long. Between sweeps the gap is one sweep
            # period (~21 ms), which is well inside the transfer timeout.
            pass
        else:
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

    def _diag_dump(self):
        if not SFCW_DIAG or getattr(self, '_diag_gaps', None) is None:
            return
        try:
            self._diag_seq = getattr(self, '_diag_seq', 0) + 1
            tag = f"{SFCW_DIAG}{self._diag_seq:02d}"
            np.save(tag + '_gaps.npy', self._diag_gaps[:self._diag_gaps_n])
            np.save(tag + '_steps.npy', self._diag_steps[:self._diag_steps_n])
            print(f"[sfcw] diag: {self._diag_gaps_n} gaps, {self._diag_steps_n} steps "
                  f"-> {tag}_*.npy")
        except Exception as e:
            print(f"[sfcw] diag dump failed: {e}")

    def _stop_tx_rx(self):
        if self._nios_inflight is not None or self._nios_primed:
            self._nios_discard_inflight()
        self._nios_primed = False
        self._nios_period_hist = []
        self._diag_dump()
        if self.sweep_mode == 'dsp':
            # Also drops the DSP mux back to the raw path, so the next stream
            # (calib panel, standard sweep) sees ordinary samples.
            self.driver.stop_rx_dsp()
        else:
            self.driver.stop_rx_dual()
        self.driver.stop_tx_dual()
        # Restore single-channel config so calib panel works after SFCW
        self.driver._configure_channels()



    def _rx_capture(self, rx1_iq, rx2_iq):
        # Stamp the arrival gap HERE, in the RX thread. The sweep thread cannot
        # measure this for itself: timing its own wait conflates "the hardware
        # took a buffer period to produce this" with "I was descheduled", and
        # under the contention that causes the corruption in the first place the
        # second is exactly what happens. Measured from the producer, the gap is
        # a property of the data, not of the consumer's scheduling luck.
        now = time.perf_counter()
        with self._rx_cond:
            self._rx_gap = (now - self._rx_t) if self._rx_t is not None else 0.0
            self._rx_t = now
            self._rx_latest = (rx1_iq, rx2_iq)
            self._rx_seq += 1
            if self._bulk_capture:
                if len(self._bulk_rx1) < self._bulk_max_buffers:
                    # _rx_loop_dual builds fresh arrays per callback, so
                    # holding the reference is enough -- no copy needed.
                    self._bulk_rx1.append(rx1_iq)
                    self._bulk_rx2.append(rx2_iq)
                else:
                    # Cap hit: the sweep is long since over; stop accumulating
                    # so an idle pipelined capture cannot eat memory.
                    self._bulk_capture = False
            if SFCW_DIAG and self._diag_gaps_n < DIAG_MAX_GAPS:
                self._diag_gaps[self._diag_gaps_n] = self._rx_gap
                self._diag_gaps_n += 1
            self._rx_cond.notify_all()

    # ------------------------------------------------------------------
    # NIOS autonomous sweep
    #
    # The FPGA steps the synthesizers through the frequency list on its own,
    # so a sweep costs one USB round-trip instead of two per step. See
    # docs/nios_sweep.md for the protocol and the firmware side.
    # ------------------------------------------------------------------

    def _log_nios_fallback(self, reason):
        """Report fallbacks without drowning stdout.

        A fallback is EXPECTED at a low rate -- it is the span gate refusing an
        ambiguous sweep and taking a correct slower one instead, by design. At
        36 Hz even a 1% rate is a line every three seconds, which buries the
        one message a real failure would print (exactly the trap the
        `_sweep_core` unpack bug fell into: an operator trained to ignore a
        recurring line). So the first of a run is printed in full, and the rest
        are summarised on a timer with a rate.
        """
        now = time.time()
        first = self._nios_fb_window_start is None
        if first:
            self._nios_fb_window_start = now
            self._nios_fb_window_count = 0
        self._nios_fb_window_count += 1
        elapsed = now - self._nios_fb_window_start
        if first or elapsed >= NIOS_FALLBACK_LOG_PERIOD_S:
            if first:
                print(f"[sfcw] NIOS sweep: {reason} — one standard sweep "
                      f"used instead")
            else:
                pct = 100.0 * self._nios_fb_window_count / max(1, self._nios_sweeps_since)
                print(f"[sfcw] NIOS sweep: {self._nios_fb_window_count} fallbacks "
                      f"in the last {elapsed:.0f}s ({pct:.1f}% of sweeps); "
                      f"most recent: {reason}")
            self._nios_fb_window_start = now
            self._nios_fb_window_count = 0
            self._nios_sweeps_since = 0

    def _nios_command(self, cmd, arg=0):
        """Send a sweep command as a sentinel timestamp in a retune2 packet.

        Returns the libbladeRF status. The NIOS answers with a value in the
        response's duration field, but libbladeRF unpacks that straight into a
        log_verbose and drops it (nios_access.c), so nothing comes back here.
        That is why T0 has to be anchored from timestamps below rather than
        simply read.
        """
        prof = self._nios_cmd_profile
        if prof is None and self._qt_master_rx:
            prof = self._qt_master_rx[0]
        if prof is None:
            return -1
        dev_ptr = self.driver.device.dev[0]
        timestamp = ((cmd & 0xFFFFFFFF) << 32) | (arg & 0xFFFFFFFF)
        return libbladeRF.bladerf_schedule_retune(
            dev_ptr, bladerf.CHANNEL_RX(0), timestamp, 0, prof)

    def _nios_prime(self, freqs, qt_rx, qt_tx):
        """Teach the NIOS the sweep by walking it once over USB.

        Costs one full sweep's worth of retunes (~124 ms at 51 steps) and holds
        until the frequency grid changes. The NIOS records the profile indices
        of the RETUNE_NOW packets it sees while primed, so it can only ever
        replay frequencies that were actually tuned here.
        """
        num_steps = len(freqs)
        self._nios_cmd_profile = qt_rx[0]
        if self._nios_command(NIOS_CMD_PRIME, num_steps) != 0:
            print("[sfcw] NIOS prime rejected — is the sweep firmware loaded?")
            return False

        dev_ptr = self.driver.device.dev[0]
        tx_ch = bladerf.CHANNEL_TX(0)
        rx_ch = bladerf.CHANNEL_RX(0)
        for i in range(num_steps):
            f = int(freqs[i])
            libbladeRF.bladerf_schedule_retune(dev_ptr, rx_ch, 0, f, qt_rx[i])
            libbladeRF.bladerf_schedule_retune(dev_ptr, tx_ch, 0, f, qt_tx[i])

        if DSP_STEPPER and self.sweep_mode == 'dsp':
            # Copy the primed table into the FPGA stepper. One USB command;
            # the Nios pushes 34 words per step through the expansion PIO.
            if self._nios_command(NIOS_CMD_LOAD, 0) != 0:
                print("[sfcw] stepper table load (SWLD) rejected -- firmware without "
                      "the fpga-stepper commands?")
                return False
            print(f"[sfcw] FPGA stepper table loaded: {num_steps} steps")

        self._nios_primed = True
        self._nios_primed_steps = num_steps
        self._nios_primed_key = (int(freqs[0]), int(freqs[-1]), num_steps)
        print(f"[sfcw] NIOS primed with {num_steps} steps")
        return True

    def _nios_clear_queue(self):
        """Drop anything sitting in the FPGA's scheduled-retune queues."""
        try:
            dev_ptr = self.driver.device.dev[0]
            libbladeRF.bladerf_cancel_scheduled_retunes(dev_ptr, bladerf.CHANNEL_RX(0))
            libbladeRF.bladerf_cancel_scheduled_retunes(dev_ptr, bladerf.CHANNEL_TX(0))
        except Exception as e:
            print(f"[sfcw] cancel_scheduled_retunes: {e}")

    def _nios_stop(self):
        """Abort any sweep the FPGA still thinks it is running."""
        self._nios_command(NIOS_CMD_STOP)

    def _nios_timestamp(self):
        try:
            return int(self.driver.get_timestamp(BLADERF_RX))
        except Exception:
            return 0

    def _bulk_start(self, max_buffers=None):
        """Open a bulk capture, bounded to max_buffers.

        The bound is not just a memory guard, it is a STABILITY requirement
        under pipelining. The capture for sweep N+1 is opened before sweep N is
        processed, so it keeps accumulating for as long as processing takes --
        and processing cost is dominated by np.concatenate over the captured
        buffer list. That is a positive feedback loop: slower processing -> more
        buffers -> slower concatenate -> slower still. Measured 2026-09-07
        through the full stack, uncapped in practice (a 1220-buffer / 0.25 s
        cap, ~10x what a sweep needs): the sweep drifted 28 ms -> 44 ms over a
        1200-sweep run, and rotated slices appeared with it. Capping at what
        the sweep actually needs holds it flat.
        """
        with self._rx_cond:
            self._bulk_rx1 = []
            self._bulk_rx2 = []
            self._bulk_max_buffers = (NIOS_BULK_MAX_BUFFERS if max_buffers is None
                                      else min(int(max_buffers),
                                               NIOS_BULK_MAX_BUFFERS))
            # Buffer number N (1-based, as _rx_seq counts them) covers samples
            # [(N-1)*S, N*S). The next buffer to land is _rx_seq + 1, so the
            # first sample we collect sits at _rx_seq * S.
            self._bulk_start_sample = self._rx_seq * self._rx_buffer_samples
            self._bulk_start_seq = self._rx_seq
            self._bulk_capture = True

    def _bulk_stop(self):
        with self._rx_cond:
            self._bulk_capture = False
            rx1 = self._bulk_rx1
            rx2 = self._bulk_rx2
            self._bulk_rx1 = []
            self._bulk_rx2 = []
            return rx1, rx2, self._bulk_start_sample

    def _bulk_wait(self, needed_samples, timeout_s):
        """Block until `needed_samples` more samples have been delivered.

        Counted as buffers since the bulk capture started rather than against
        an absolute sample index, so this does not depend on how the buffer
        sequence lines up with the hardware sample counter.
        """
        samples_per_buffer = self._rx_buffer_samples
        with self._rx_cond:
            needed_seq = self._bulk_start_seq + \
                int(np.ceil(needed_samples / samples_per_buffer)) + 2
        deadline = time.perf_counter() + timeout_s
        with self._rx_cond:
            while self._rx_seq < needed_seq:
                remaining = deadline - time.perf_counter()
                if remaining <= 0:
                    return False
                if self._stop_event.is_set():
                    return False
                self._rx_cond.wait(timeout=min(remaining, 0.05))
        return True

    # Dwells folded together to lock the grid phase. The transients average
    # down fast, so a slice of the capture is as good as all of it -- and the
    # full-capture version cost more than the sweep it was measuring.
    _NIOS_FOLD_DWELLS = 16

    @staticmethod
    def _nios_phase_dev(i0, i1, q0, q1, ref):
        """|sin(deviation from the dominant phase advance)| for each sample pair.

        d = c[n+1] * conj(c[n]) is the phase advance as a phasor. Projecting it
        onto the mean advance and taking the perpendicular component gives the
        deviation without ever forming an angle -- atan2 per sample dominated
        the whole sweep on the Pi.
        """
        dr = i1 * i0 + q1 * q0
        di = q1 * i0 - i1 * q0
        if ref is None:
            mr = float(dr.mean())
            mi = float(di.mean())
            nrm = np.sqrt(mr * mr + mi * mi)
            if nrm <= 0.0:
                return None, None
            ref = (mr / nrm, mi / nrm)
        mr, mi = ref
        perp = np.abs(di * mr - dr * mi)
        mag = np.sqrt(dr * dr + di * di)
        np.maximum(mag, 1e-6, out=mag)
        return perp / mag, ref

    def _nios_transient_profile(self, ref_all, ref=None):
        """Per-sample "the phase is not advancing smoothly" indicator.

        Inside a dwell the synthesizer is locked and the sample-to-sample phase
        advance is constant. While it is settling the advance is scrambled.
        That is what marks a step boundary. Result is aligned so element k
        describes the transition into sample k.
        """
        i = np.ascontiguousarray(ref_all[0::2], dtype=np.float32)
        q = np.ascontiguousarray(ref_all[1::2], dtype=np.float32)
        dev, ref = self._nios_phase_dev(i[:-1], i[1:], q[:-1], q[1:], ref)
        if dev is None:
            return np.zeros(len(i), dtype=np.float32)
        return np.concatenate((np.zeros(1, dtype=np.float32), dev))

    def _nios_grid_phase(self, dev, dwell, settle):
        """Where in the dwell the step boundary sits, modulo the dwell.

        Folds the transient indicator over the dwell so every boundary stacks
        up, then finds the longest stretch that is never transient -- that is
        the capture window, and the boundary is `settle` before it.

        Scoring candidate offsets by coherence instead does NOT work: a window
        shorter than the settle region can sit entirely inside a neighbouring
        step's clean tail and score exactly as well as the aligned one.

        Returns (phase, contrast); contrast <= 0 means no grid was found.
        """
        win = dwell - settle
        usable = (len(dev) // dwell) * dwell
        if win <= 0 or usable < dwell * 2:
            return 0, -1.0

        fold = dev[:usable].reshape(-1, dwell).sum(axis=0)
        ext = np.concatenate((fold, fold[:win]))
        csum = np.concatenate(((0.0,), np.cumsum(ext)))
        wsum = csum[win:win + dwell] - csum[:dwell]

        mean = float(wsum.mean())
        if mean <= 0:
            return 0, -1.0
        contrast = float(wsum.max() - wsum.min()) / mean
        quiet_start = int(np.argmin(wsum))

        # quiet_start is where the clean capture window begins, so the step
        # boundary sits `settle` earlier. Verified against
        # test_nios_slicing.py, which models the firmware's real transient
        # pattern: this recovers T0 to the sample.
        return (quiet_start - settle) % dwell, contrast

    def _nios_launch(self, units, num_steps, dwell):
        """Begin a bulk capture and start a sweep inside it.

        The capture is opened before the command goes out, so the sweep is
        guaranteed to fall inside it: EXEC costs a USB round-trip and the
        firmware only starts stepping a dwell after that.
        """
        # Pre-size from what the sweep needs; refined once round_trip is known.
        self._bulk_start(max_buffers=int(np.ceil(
            ((num_steps + 4) * dwell + 32 * self._rx_buffer_samples)
            / self._rx_buffer_samples)) + 8)
        ts0 = self._nios_timestamp()
        rc = self._nios_command(NIOS_CMD_EXEC, (units << 16) | (num_steps & 0xFFFF))
        ts1 = self._nios_timestamp()
        if rc != 0 or ts1 <= ts0:
            self._bulk_stop()
            return None
        round_trip = ts1 - ts0
        # The driver holds up to num_buffers of already-captured samples, so a
        # capture opened after a gap begins with stale ones and can run out
        # before the sweep's last transient -- which is what alignment keys on.
        # Sweeps run back to back in a benchmark and there is no gap; the
        # server serialises a result between them and there is.
        backlog = 16 * self._rx_buffer_samples
        need = round_trip + backlog + (num_steps + 3) * dwell
        # Re-cap now that `need` is known: _bulk_start was called before the
        # round trip could be measured. +8 buffers of slack so the wait is
        # never starved by its own cap.
        with self._rx_cond:
            self._bulk_max_buffers = min(
                NIOS_BULK_MAX_BUFFERS,
                int(np.ceil(need / self._rx_buffer_samples)) + 8)
        return {'round_trip': round_trip, 'ts_start': ts0, 'need': need}

    def _nios_harvest(self, inflight):
        """Wait out an in-flight capture and take the buffers.

        Also measures whether the capture is gapless. The FPGA's sample counter
        advances in real time regardless of what the host does, so comparing it
        against the samples actually delivered says exactly how many were lost
        inside this capture -- and lost samples are a discontinuity in the
        timeline that no uniform period can model.
        """
        need = inflight['need']
        ts_start = inflight.get('ts_start', 0)
        ok = self._bulk_wait(need,
                             timeout_s=need / float(self.driver.sample_rate) + 1.0)
        ts_end = self._nios_timestamp()
        rx1, rx2, start = self._bulk_stop()
        got = sum(len(b) // 2 for b in rx2)
        elapsed = max(1, ts_end - ts_start)
        self._nios_capture_loss = 1.0 - got / float(elapsed)
        # Delivery lag in per-channel samples: how far behind real time the
        # delivered stream was when the harvest closed. Ordinary backlog under
        # load; only a lag near the ring capacity implies actual loss.
        self._nios_capture_lag = max(0, elapsed - got)
        return (rx1, rx2, start) if ok else None

    def _nios_discard_inflight(self):
        """Abandon any capture or sweep still running.

        Unconditional: a harvest that timed out has already cleared the
        in-flight record but may well have left the FPGA mid-sweep.
        """
        self._nios_inflight = None
        if getattr(self, '_rx_cond', None) is not None:
            self._bulk_stop()
        self._nios_stop()

    def _nios_diagnose(self, ref_all, dwell, settle, num_steps, hot, starts):
        """Explain an alignment failure in terms of what the FPGA actually did.

        Finds the step transients directly, without assuming a period, so a
        real period that differs from the dwell the host is slicing at shows up
        as a number instead of as cells that mysteriously stop being hot.
        """
        try:
            dev = self._nios_transient_profile(ref_all)
            k = 64
            sm = np.convolve(dev, np.ones(k, dtype=np.float32) / k, mode='same')
            med = float(np.median(sm))
            mad = float(np.median(np.abs(sm - med))) + 1e-12
            edges = np.flatnonzero((sm[:-1] <= med + 10 * mad) &
                                   (sm[1:] > med + 10 * mad)) + 1
            keep = []
            for x in edges:
                if not keep or x - keep[-1] > dwell // 4:
                    keep.append(int(x))
            gaps = np.diff(np.array(keep)).astype(np.float64) if len(keep) > 1 else np.array([])
            if len(gaps):
                best, best_n = gaps[0], 0
                for g in gaps:
                    c = int(np.sum(np.abs(gaps - g) < 0.1 * g))
                    if c > best_n:
                        best, best_n = g, c
                period = float(np.median(gaps[np.abs(gaps - best) < 0.1 * best]))
            else:
                period = float('nan')
            pattern = ''.join('X' if h else '.' for h in hot)
            print(f"[sfcw]   grid used period {self._nios_period:.1f}, "
                  f"phase {self._nios_last_phase}")
            print(f"[sfcw]   {len(keep)} transients found, actual period "
                  f"{period:.0f} samples vs dwell {dwell} "
                  f"({period - dwell:+.0f}, drifts a probe width after "
                  f"{abs(settle / (period - dwell)) if abs(period - dwell) > 1 else float('inf'):.0f} steps)")
            print(f"[sfcw]   hot cells: {pattern}")
        except Exception as ex:
            print(f"[sfcw]   diagnose failed: {ex}")

    def _nios_detect_transients(self, dev, period_hint):
        """Every step boundary in the capture, as sample positions.

        Thresholds a smoothed transient indicator over the whole capture rather
        than probing where boundaries are predicted to be. Probing is cheaper
        but circular: it needs the period to know where to look, and the period
        is what we are trying to measure. Detecting first has found every
        transient on hardware, including on captures where the probe-based
        version gave up around step ten.

        k=64, not the k=256 the Nios II/e was tuned with. The II/f image's
        fastlock retunes are far cleaner: measured 2026-09-07, a boundary is
        hot for a median of ~16-33 SAMPLES (1.6-3.3 us) with some boundaries
        peaking at only 0.125 deviation -- a 256-sample boxcar dilutes that
        below the 8-MAD threshold and only ~20 of 51 steps were detected
        (every sweep fell back). At k=64, 57 edges are found on the same
        capture; a long II/e-style transient is still fully hot at k=64, so
        this is strictly more sensitive, and the MAD threshold + period fit +
        min-gap merge absorb the extra noise sensitivity.
        """
        k = 64
        if len(dev) <= k * 2:
            return np.empty(0, dtype=np.int64)
        c = np.cumsum(dev, dtype=np.float64)
        sm = (c[k:] - c[:-k]) / k

        # Bulk statistics from a subsample: the threshold describes the quiet
        # floor of a slowly-varying field, and a full np.median (a sort) over
        # the whole capture was most of this function's cost.
        sub = sm[::8]
        med = float(np.median(sub))
        mad = float(np.median(np.abs(sub - med))) + 1e-12
        hot = sm > med + 8.0 * mad
        if not hot.any():
            return np.empty(0, dtype=np.int64)

        edges = np.flatnonzero((~hot[:-1]) & hot[1:]) + 1
        keep = []
        for x in edges:
            if not keep or x - keep[-1] > period_hint // 4:
                keep.append(int(x))
        return np.array(keep, dtype=np.int64)

    def _nios_refine_offset(self, ref_all, t0_guess, search, num_steps):
        # INVARIANT: alignment is derived from the REFERENCE channel only.
        # RX2 is a cable loopback, so nothing in front of the antenna can
        # change it -- which is what makes slicing immune to the scene. Do not
        # feed the signal channel, h_cal, or anything derived from a previous
        # sweep into this decision: a target entering the beam then looks
        # exactly like a mis-slice, and the "correction" rotates good sweeps.
        # That regression is documented at the span gate below.
        """Find T0 in the capture, to the sample.

        Three separate things have to come out of this, and each needs a
        different tool:

        - the period, which is NOT the dwell. The FPGA schedules on its sample
          counter but the host stream loses a fraction of a percent of samples,
          so the timeline is compressed, by 16 to 24 samples per step and not
          by the same amount every sweep. A least-squares fit through the
          detected transients measures it.

        - which step is T0. The firmware activates step 0 at T0-dwell and step
          1 at T0+dwell, so T0's own boundary carries no transient: the train
          has a double-width gap and T0 sits in the middle of it.

        - the exact boundary, to the sample. Detection is biased by the width
          of its own smoothing window, so the position comes instead from
          folding at the measured period, where the longest never-transient
          stretch is the capture window and its leading edge is the boundary.

        Returns (offset, score); score <= 0 means it could not be found.
        """
        dwell = int(self.nios_dwell)
        settle = int(self.nios_settle)
        n_pairs = len(ref_all) // 2
        self._nios_fail = None
        self._nios_period = float(dwell)
        if n_pairs < dwell * 3:
            self._nios_fail = f"capture {n_pairs} < 3 dwells"
            return t0_guess, -1.0

        i = np.ascontiguousarray(ref_all[0::2], dtype=np.float32)
        q = np.ascontiguousarray(ref_all[1::2], dtype=np.float32)

        dev = self._nios_transient_profile(ref_all)
        pos = self._nios_detect_transients(dev, dwell)
        if len(pos) < num_steps // 2:
            self._nios_fail = (f"only {len(pos)} transients in the capture, "
                               f"need at least {num_steps // 2}")
            return t0_guess, -1.0

        gaps = np.diff(pos).astype(np.float64)
        period = float(np.median(gaps))          # robust to the gap at T0
        # Real drift is tens of samples per step, not hundreds. On the II/e the
        # only term was stream compression (period a fraction of a percent
        # SHORTER than the dwell); the II/f image also steps slightly LONG --
        # measured 2026-09-07: dwell 4096 -> period 4107 (+0.27%), 4928 -> 4960
        # (+0.65%). 1.5% accepts both directions with margin; the failure this
        # gate exists to catch (the median gap latching onto something other
        # than the step train) is off by tens of percent, not one.
        if abs(period - dwell) > dwell * 0.015:
            self._nios_fail = (f"step period {period:.0f} is {period - dwell:+.0f} "
                               f"off the {dwell}-sample dwell -- too far to be "
                               f"stream compression")
            return t0_guess, -1.0

        # --- pin the step lattice on the regular tail train ----------------
        #
        # The old missing-tooth search indexed EVERY transient against a grid
        # anchored at pos[0] and looked for the empty cell. That worked on the
        # Nios II/e and mis-anchors on the II/f, because the EXEC front matter
        # is not ON the T0 grid there: measured 2026-09-07, EXEC produces a
        # transient pair ~1940 samples apart (step 0's activation) and the
        # first REGULAR boundary lands 2.7-2.9 dwells after it -- the firmware
        # reads "now" for T0 only after ~300 us of activation-plus-response
        # handling, so the front transients sit a latency-dependent FRACTION
        # of a dwell off the grid. On the II/e the dwell was 6x longer and the
        # same latency rounded away. So: find the longest run of period-spaced
        # transients (that is steps 1..n-1, the only strictly periodic thing
        # in the capture), extend it across weak steps, and anchor T0 at its
        # END -- the firmware stops after step n-1, so the last lattice member
        # is the step n-1 boundary. A missed LAST step shifts the anchor one
        # period. That case is caught structurally by the span gate further
        # down, which refuses the sweep rather than guessing at it.
        good_gap = np.abs(gaps - period) < 0.05 * period
        runs, start = [], None
        for j, g in enumerate(good_gap):
            if g and start is None:
                start = j
            elif not g and start is not None:
                runs.append((start, j))
                start = None
        if start is not None:
            runs.append((start, len(good_gap)))
        if not runs:
            self._nios_fail = "no run of period-spaced transients"
            return t0_guess, -1.0
        lo, hi = max(runs, key=lambda r: r[1] - r[0])   # pos[lo..hi] inclusive

        # A weak step mid-train splits the run in two; its neighbours are
        # still on the lattice. Take every transient within 0.15 period of the
        # main run's lattice, which re-joins the halves and skips the hole.
        k_all = np.round((pos - pos[lo]) / period).astype(np.int64)
        on_lat = np.abs(pos - (pos[lo] + k_all * period)) < 0.15 * period
        sel = np.flatnonzero(on_lat)
        k_sel = k_all[sel].astype(np.float64)
        p_sel = pos[sel].astype(np.float64)

        # A back-to-back capture can begin with STALE buffers holding the tail
        # of the PREVIOUS sweep (see the backlog note in _nios_launch) -- also
        # period-spaced, also on-lattice if its phase happens to align, so the
        # lattice can span more periods than one sweep. Our own train is the
        # trailing one (the capture always ends after this sweep's last step),
        # and steps 1..n-1 span exactly num_steps-2 periods: keep only the
        # trailing window of that many periods. The >2-period EXEC-latency gap
        # between the stale tail and our step 1 keeps stale members out of it.
        keep_lat = k_sel >= k_sel[-1] - (num_steps - 2)
        sel, k_sel, p_sel = sel[keep_lat], k_sel[keep_lat], p_sel[keep_lat]

        covered = len(sel)
        k_span = int(k_sel[-1] - k_sel[0])
        # Interior holes are harmless -- a missed boundary between two present
        # ones does not move either end, and the span (checked below, after
        # the fit) is what pins T0. Only refuse here if so little of the train
        # survived that the fit itself would be unreliable.
        if covered < int(0.75 * (num_steps - 1)):
            self._nios_fail = (f"lattice holds only {covered} of "
                               f"{num_steps - 1} step boundaries")
            return t0_guess, -1.0

        # Refit the period through the lattice members, with outlier
        # rejection: a single spurious detection drags the slope enough to
        # matter (measured +/-3500 samples of accumulated drift across 51
        # steps without this).
        intercept = None
        if covered >= 6:
            slope, intercept = np.polyfit(k_sel, p_sel, 1)
            for _ in range(2):
                resid = p_sel - (slope * k_sel + intercept)
                spread = float(np.median(np.abs(resid))) + 1.0
                keep = np.abs(resid) < 4.0 * spread
                if keep.sum() < 6:
                    break
                slope, intercept = np.polyfit(k_sel[keep], p_sel[keep], 1)
            if abs(slope - period) < period * 0.01:
                period = float(slope)
            else:
                intercept = None

        # The period error is a property of the firmware and the host keeping
        # up, not of this particular sweep, so it barely moves between sweeps.
        # Hold a running median and refuse an estimate that jumps away from it
        # -- one bad fit then costs nothing instead of rotating a whole sweep.
        hist = getattr(self, '_nios_period_hist', None)
        if hist is None:
            hist = self._nios_period_hist = []
        hist.append(period)
        if len(hist) > 9:
            hist.pop(0)
        if len(hist) >= 5:
            steady = float(np.median(hist))
            if abs(period - steady) > 24.0:
                period = steady
        self._nios_period = period

        # --- anchor T0, and REFUSE to guess when the structure is ambiguous --
        #
        # The firmware fires steps 1..n-1 on the dwell grid (step 0 is
        # activated by EXEC itself, off-grid), so a complete lattice holds
        # n-1 boundaries spanning exactly n-2 periods. When that holds, the
        # first member IS step 1 and the last IS step n-1 -- the front anchor
        # (first - period) and the end anchor (last - (n-1)*period) are then
        # algebraically THE SAME NUMBER, and T0 is certain.
        #
        # When a boundary at one END is missed the span comes up short, and
        # the two anchors differ by exactly one period: that is the one-step
        # rotation. Nothing inside the sweep can say which end lost it --
        # both channels shift together, so every capture window is still
        # clean CW, just of the neighbouring frequency, and h_cal comes out a
        # perfectly valid measurement that is simply rotated by one step.
        #
        # So this does NOT try to repair it. It falls back to the standard
        # sweep for this one sweep, which is correct and merely slower.
        # Measured 2026-09-07 over 199 sweeps: span was n-2 on 197, short by
        # one period on 1 (the rotation), and wildly short on 1 broken
        # capture -- so the gate costs ~1% of sweeps and removes the rotation
        # entirely.
        #
        # An earlier version anchored on the END and repaired rotations by
        # correlating against the previous sweep / a running template. That
        # is unfixable by construction: a genuine SCENE CHANGE (a target
        # entering the beam) drops agreement exactly like a mis-slice does,
        # so the resolver rotated correctly-aligned sweeps, and re-seeding
        # the template on one of those latched the error until the sweep was
        # restarted. Alignment must not depend on scene stability.
        self._nios_last_span_ok = (k_span == num_steps - 2)
        if k_span != num_steps - 2:
            self._nios_fail = (
                f"step lattice spans {k_span} periods, expected "
                f"{num_steps - 2} -- a boundary at one end was missed, so "
                f"T0 is ambiguous by a whole step")
            return t0_guess, -1.0

        # Front anchor: first lattice member = step 1, so T0 is one period
        # earlier. Taken off the fitted line rather than the raw detection,
        # which sits a smoothing window off the true edge.
        if intercept is not None:
            p_first = float(np.polyval((slope, intercept), k_sel[0]))
        else:
            p_first = float(p_sel[0])
        approx_t0 = p_first - period

        # Detection sits a smoothing window off the true edge, so take the
        # exact boundary from a fold at the measured period instead.
        per_i = max(1, int(round(period)))
        span = int(min(self._NIOS_FOLD_DWELLS * per_i, n_pairs - 2))
        r0 = int(max(1, min(int(approx_t0), n_pairs - span - 1)))
        dev_r, _ = self._nios_phase_dev(i[r0:r0 + span - 1], i[r0 + 1:r0 + span],
                                        q[r0:r0 + span - 1], q[r0 + 1:r0 + span],
                                        None)
        offset = int(round(approx_t0))
        contrast = 1.0
        if dev_r is not None:
            dev_r = np.concatenate((np.zeros(1, dtype=np.float32), dev_r))
            local, contrast = self._nios_grid_phase(dev_r, per_i, settle)
            if contrast >= 0.05:
                phase = (r0 + local) % per_i
                delta = (phase - offset) % per_i
                if delta > per_i // 2:
                    delta -= per_i
                offset += int(delta)
        self._nios_last_phase = offset

        if offset < 1 or offset + (num_steps - 1) * per_i + per_i > n_pairs:
            self._nios_fail = (f"T0 at {offset} leaves no room for "
                               f"{num_steps} steps in {n_pairs} samples")
            hot = np.zeros(int(k_sel[-1] - k_sel[0]) + 1, dtype=bool)
            hot[(k_sel - k_sel[0]).astype(np.int64)] = True
            self._nios_diagnose(ref_all, dwell, settle, num_steps, hot, pos)
            return t0_guess, -1.0

        return offset, max(contrast, 0.05)

    def _sweep_core_nios(self, freqs, qt_rx, qt_tx, num_buffers, settle_count,
                         progress_cb=None):
        """One command, one continuous capture, sliced at the step boundaries.

        Falls back to the standard sweep on any failure, so selecting this mode
        without the sweep firmware loaded degrades rather than breaks.
        Returns (h_cal, dropped_steps, adc_peak) like _sweep_core; num_buffers
        and settle_count only matter on the fallback path -- the NIOS capture
        window is set by nios_dwell/nios_settle in SAMPLES, not buffers.
        """
        num_steps = len(freqs)
        self._nios_sweeps_since += 1
        if self._stop_event.is_set():
            # THREE values, like _sweep_core's own early return -- see the
            # comment there for the unpack bug this shape prevents.
            return None, 0, None

        def fallback(reason, reprime=False):
            """Take one standard sweep instead of this one, and say why.

            `reprime` separates the two very different reasons we get here:

            - A TRANSIENT failure (the step lattice came up short, the capture
              was cut off, a harvest timed out). The NIOS still holds a
              perfectly good copy of the frequency grid, so tearing that down
              is pure waste -- and worse than waste: re-priming walks 51
              retunes over USB, which disturbs the timing of the very next
              capture and can turn one fallback into a run of them. Leave the
              priming alone; the next sweep almost always aligns.
            - A CAPABILITY failure (EXEC rejected, sample counter dead, prime
              refused). The firmware may not be the sweep firmware at all, in
              which case it treated our sentinels as ordinary scheduled
              retunes and queued them; those never fire and would fill the
              16-deep queue, so the queue is cleared and the grid re-primed.
            """
            self._nios_last_score = -1.0
            self._last_sweep_core = 'fallback'
            self._nios_fallbacks += 1
            self._nios_discard_inflight()
            self._log_nios_fallback(reason)
            if reprime:
                self._nios_primed = False
                self._nios_clear_queue()
            return self._sweep_core(freqs, qt_rx, qt_tx, num_buffers,
                                    settle_count, progress_cb)

        if qt_rx is None or qt_tx is None:
            return fallback("no quick-tune profiles", reprime=True)

        key = (int(freqs[0]), int(freqs[-1]), num_steps)
        if not self._nios_primed or self._nios_primed_key != key:
            if not self._nios_prime(freqs, qt_rx, qt_tx):
                return fallback("priming failed", reprime=True)

        dwell = int(self.nios_dwell)
        settle = int(self.nios_settle)
        if settle >= dwell:
            return fallback(f"settle {settle} >= dwell {dwell}")

        units = dwell // NIOS_INTERVAL_UNIT
        if units < 1 or units > 0xFFFF:
            return fallback(f"dwell {dwell} out of range")
        dwell = units * NIOS_INTERVAL_UNIT      # what the NIOS will actually use

        # Cache the mixing tone at capture-window length.
        if dwell - settle - 2 * NIOS_WINDOW_GUARD < 64:
            return fallback(f"dwell {dwell} leaves no capture window after "
                            f"{settle} settle")

        try:
            inflight = self._nios_inflight
            self._nios_inflight = None
            if inflight is None:
                inflight = self._nios_launch(units, num_steps, dwell)
                if inflight is None:
                    # Stock firmware ACCEPTS the sentinels (it enqueues them
                    # as scheduled retunes) but its sample counter reads 0,
                    # which is what this detects. Latch it off for the session
                    # rather than paying a prime-and-fail on every sweep.
                    self._nios_unavailable = True
                    print("[sfcw] NIOS autonomous sweep unavailable on this "
                          "FPGA image (sample counter not running) -- using "
                          "the standard sweep for this session. Load "
                          "hosted_niosII_f_sweep_ts.rbf and restart the sweep "
                          "to re-enable.")
                    return fallback("EXEC rejected, or the sample counter is "
                                    "not running — is the rx.vhd tamer change "
                                    "in this FPGA image?", reprime=True)

            harvested = self._nios_harvest(inflight)
            if harvested is None:
                return fallback("timed out waiting for the capture")

            # See nios_pipeline in __init__ for why this is off by default.
            if self.nios_pipeline:
                self._nios_inflight = self._nios_launch(units, num_steps, dwell)
        except Exception as e:
            return fallback(f"exec failed: {e}", reprime=True)

        rx1_bufs, rx2_bufs, bulk_start = harvested
        round_trip = inflight['round_trip']
        if not rx1_bufs:
            return fallback("no buffers captured")

        # A capture with a GAP in it (RX thread stalled past the ring depth,
        # libbladeRF dropped buffers) cannot be sliced: every position after
        # the gap is shifted, the step lattice splits in two, and the
        # realignment search will happily return the least-bad of three wrong
        # answers. _nios_harvest measures exactly this -- delivered samples
        # against hardware-clock elapsed -- so gate on it BEFORE slicing.
        # The no-loss baseline reads ~-0.5% (stale head bias); one lost buffer
        # in a ~27 ms capture reads ~+0.8%.
        # What that ratio measures is delivery LAG at harvest, which only
        # means loss once it could have exceeded the sync ring -- with the
        # 256-buffer ring a few ms of lag is ordinary backlog under load
        # (measured 2-7 ms at 30 Hz full stack; an earlier 0.5%-"loss" gate
        # here rejected every sweep). Gate at 75% of the ring: past that,
        # samples may genuinely have been dropped.
        lag_samples = self._nios_capture_lag
        ring_samples = rx_ring_depth() * self._rx_buffer_samples
        if lag_samples > 0.75 * ring_samples:
            return fallback(
                f"RX delivery lagged "
                f"{lag_samples / self.driver.sample_rate * 1e3:.0f} ms at "
                f"harvest -- possible ring overflow, capture untrusted")

        # float32 throughout: this is a million-plus samples and the Pi is
        # memory-bandwidth bound here, not precision bound.
        sig_all = np.concatenate(rx1_bufs).astype(np.float32)
        ref_all = np.concatenate(rx2_bufs).astype(np.float32)

        # Same {'rx1','rx2'} shape _sweep_core produces. Strided by 4: this is
        # a headroom monitor against a 40%/75% threshold, not a measurement,
        # and a full abs().max() over two million-sample arrays costs real ms.
        adc_peak = {'rx1': float(np.abs(sig_all[::4]).max()) if len(sig_all) else 0.0,
                    'rx2': float(np.abs(ref_all[::4]).max()) if len(ref_all) else 0.0}

        if len(ref_all) // 2 < (num_steps + 2) * dwell:
            return fallback("capture does not span the sweep")

        # The anchor is a hint only; _nios_refine_offset validates the grid
        # structurally and ignores it if the data disagrees. Do NOT trim the
        # capture to a trailing window before aligning: where the train sits
        # depends on how much STALE ring content the capture opened with,
        # which is unknowable -- with an empty ring the train is at the FRONT
        # and a (num_steps+4)*dwell tail window cuts its head off (tried
        # 2026-09-07, broke every sweep in exactly that way).
        t0_rel = max(0, round_trip // 2 + dwell)
        offset, score = self._nios_refine_offset(ref_all, t0_rel, 0, num_steps)
        if score <= 0:
            return fallback(f"could not locate the step grid: "
                            f"{self._nios_fail or 'unknown'} "
                            f"(capture loss {self._nios_capture_loss * 100:.1f}%)")
        self._nios_last_offset = offset
        self._nios_last_drift = offset - t0_rel
        self._nios_last_score = score

        # Reduce each step to one complex number.
        #
        # Fancy-indexing a (num_steps, win) window out of the capture built two
        # 16 MB complex arrays and cost more than the sweep itself. Instead the
        # mixing tone is zero-padded out to a full dwell, which lets the steps
        # be addressed as a plain contiguous reshape -- no copy -- and turns
        # the whole reduction into four real matrix-vector products.
        # Steps land every `period` samples, which is a little under the dwell
        # (see _nios_refine_offset). Rounding it to an integer keeps the
        # reshape below a view rather than a 4 MB gather, and costs at most
        # half a sample per step.
        # Everything below is sized from the MEASURED period, not the dwell.
        # Deriving the window from one and the pad from the other is how a
        # 18368-into-18304 broadcast error happens.
        stride = int(round(self._nios_period)) or dwell
        # The measured period can sit slightly ABOVE the dwell on the II/f
        # image (see _nios_refine_offset), so do not clamp it to the dwell --
        # slicing 4107-sample steps at a 4096 stride walks the window 11
        # samples per step, ~550 by step 50. 2% cap = sanity only.
        stride = max(settle + 2 * NIOS_WINDOW_GUARD + 64,
                     min(stride, int(dwell * 1.02)))
        win = stride - settle - 2 * NIOS_WINDOW_GUARD
        # Keep the correlation window an INTEGER number of tone cycles so LO
        # leakage (and every odd harmonic) lands on a null of the rectangular
        # window's sinc -- the same rule DEMOD_SAMPLES follows, see the
        # constants comment at the top of this file. At 100 kHz / 10 Msps that
        # is any multiple of 100 samples. The trimmed samples just extend the
        # trailing guard; cost is 10*log10(win/(win-99)) of processing gain at
        # worst, well under 0.25 dB for any usable dwell.
        per_cycle = int(round(self.driver.sample_rate / self.driver.cw_offset))
        if per_cycle > 1:
            win -= win % per_cycle
        if win < max(64, per_cycle):
            return fallback(f"dwell {dwell} leaves no capture window after "
                            f"{settle} settle + guards")
        if getattr(self, '_nios_tone_len', None) != win:
            t = np.arange(win, dtype=np.float64) / self.driver.sample_rate
            self._nios_tone = np.exp(-1j * 2 * np.pi * self.driver.cw_offset * t) / 2047.0
            self._nios_tone_len = win
        span = num_steps * stride
        if offset + span > len(sig_all) // 2:
            return fallback("capture does not span the sweep")

        if getattr(self, '_nios_pad_key', None) != (stride, settle, win):
            pad_r = np.zeros(stride, dtype=np.float32)
            pad_i = np.zeros(stride, dtype=np.float32)
            lead = settle + NIOS_WINDOW_GUARD
            pad_r[lead:lead + win] = self._nios_tone.real
            pad_i[lead:lead + win] = self._nios_tone.imag
            self._nios_pad_r, self._nios_pad_i = pad_r, pad_i
            self._nios_pad_key = (stride, settle, win)
        pad_r, pad_i = self._nios_pad_r, self._nios_pad_i

        def reduce_steps(flat, off):
            i_v = flat[0::2][off:off + span].reshape(num_steps, stride)
            q_v = flat[1::2][off:off + span].reshape(num_steps, stride)
            return ((i_v @ pad_r - q_v @ pad_i)
                    + 1j * (i_v @ pad_i + q_v @ pad_r)) / win

        def h_cal_at(off):
            ref = reduce_steps(ref_all, off)
            ok = np.abs(ref) > 1e-10
            out = np.zeros(num_steps, dtype=np.complex128)
            out[ok] = reduce_steps(sig_all, off)[ok] / ref[ok]
            return out, int(num_steps - np.count_nonzero(ok))

        # T0 is settled structurally (see the span gate in
        # _nios_refine_offset), so there is nothing left to resolve here and
        # NOTHING in this path looks at the previous sweep. That is
        # deliberate: any scene-similarity check confuses a target entering
        # the beam with a mis-slice, and will happily rotate a correctly
        # aligned sweep to make the scene look more like it used to. See the
        # long note at the span gate for the failure that produced.
        h_cal, dropped = h_cal_at(offset)

        if dropped > num_steps // 5:
            return fallback(f"{dropped}/{num_steps} steps had no reference signal")

        if progress_cb:
            progress_cb(num_steps - 1)

        self._last_sweep_core = 'nios'
        return h_cal, dropped, adc_peak

    def _sweep_core_dsp(self, freqs, qt_rx, qt_tx, num_buffers, settle_count,
                        progress_cb=None):
        """One command, one 408-byte read. The FPGA has already done the work.

        The NIOS still steps the synthesizers exactly as in 'nios' mode, but
        nothing raw is captured: rx.vhd mixes each channel down by the CW
        offset, accumulates DSP_ACCUM_N samples per step, divides the two sums
        and writes one 64-bit ratio per step. The host reads DSP_FIFO_WORDS of
        those and has h_cal directly.

        That deletes the whole slice-and-reduce apparatus the 'nios' core needs
        -- no capture offset to refine, no transient detection, no span gate --
        because there is no continuous stream to align. The FPGA gate releases
        the FIFO only when a WHOLE sweep is in it, so a read either returns a
        complete sweep or nothing.

        Returns (h_cal, dropped, adc_peak) like the other cores.

        adc_peak is None here: it is measured from raw samples, and there are
        none. _warn_if_adc_hot therefore cannot fire in this mode -- see the
        note in _process_h_cal. Watch gain in 'nios' or 'standard' mode first.
        """
        num_steps = len(freqs)
        self._nios_sweeps_since += 1
        if self._stop_event.is_set():
            # THREE values, like _sweep_core's own early return.
            return None, 0, None

        def fallback(reason, reprime=False):
            self._nios_last_score = -1.0
            self._last_sweep_core = 'fallback'
            self._nios_fallbacks += 1
            self._dsp_last_fallback_reason = reason
            self._log_nios_fallback(reason)
            if reprime:
                self._nios_primed = False
                self._nios_clear_queue()
            # NO raw-capture fallback in DSP mode -- there is nothing to fall
            # back TO. _start_tx_rx deliberately opens no RX stream in this
            # mode (the DSP stream is opened here, lazily, by start_rx_dsp),
            # and TX is already timestamped, so a plain SC16_Q11 RX stream
            # cannot even be configured alongside it (the ERR_INVAL noted in
            # _start_tx_rx). Calling _sweep_core here, as 'nios' mode's
            # fallback legitimately does, therefore waits on rx_cond for
            # buffers that never come: settle deadline + STALL_GIVEUP_S +
            # a 1 s capture wait, about 1.3 s per step, all 51 steps dropped,
            # over a minute per sweep with nothing published -- and it
            # repeats every sweep. That is what "stuck after switching to
            # dsp" was.
            #
            # Return an empty sweep at normal cadence instead. The reason is
            # already on the console via _log_nios_fallback, and
            # _perform_sweep reports it as N/N steps incomplete. The next
            # sweep retries the DSP path from scratch (stop_rx_dsp has run),
            # so a transient failure recovers on its own and a persistent one
            # is visible every sweep rather than as a frozen GUI.
            return (np.zeros(num_steps, dtype=np.complex128), num_steps, None)

        # DSP_FIFO_WORDS is a COMPILE-TIME generic in rx.vhd, and the metadata
        # header hard-codes 2*DSP_FIFO_WORDS as the transfer length. A sweep of
        # any other length would have the FPGA declare a burst that does not
        # match what the host wants, so refuse rather than mis-read.
        if num_steps != self.driver.DSP_SWEEP_WORDS:
            return fallback(f"DSP path is built for "
                            f"{self.driver.DSP_SWEEP_WORDS} steps, this sweep "
                            f"has {num_steps}")

        if qt_rx is None or qt_tx is None:
            return fallback("no quick-tune profiles", reprime=True)

        key = (int(freqs[0]), int(freqs[-1]), num_steps)
        if not self._nios_primed or self._nios_primed_key != key:
            if not self._nios_prime(freqs, qt_rx, qt_tx):
                return fallback("priming failed", reprime=True)

        dwell = int(getattr(self, 'dsp_dwell', DSP_DEFAULT_DWELL))   # v12: dsp_dwell, not nios_dwell
        units = dwell // NIOS_INTERVAL_UNIT
        if units < 1 or units > 0xFFFF:
            return fallback(f"dwell {dwell} out of range")
        dwell = units * NIOS_INTERVAL_UNIT

        # The FPGA's FLUSH_N + ACCUM_N must fit inside one step, or
        # seq_adder's restart discards a part-accumulated window and the step
        # returns nothing. v12 makes both runtime-selectable; the check uses
        # what the FPGA last reported (compile-time values on older images).
        need = sum(self._dsp_chain_counts())
        if dwell < need:
            return fallback(f"dwell {dwell} is below the {need} the FPGA DSP "
                            f"chain needs (FLUSH_N + ACCUM_N)")

        try:
            # Open the RX stream as late as possible -- see the note in
            # _start_tx_rx. Priming is done by now, so the stream goes from
            # start to first packet in one sweep period rather than sitting
            # idle through 51 USB retunes and being torn down by transfer
            # timeouts.
            if not self.driver.rx_running:
                self.driver.start_rx_dsp()
                self._dsp_chain_dirty = True
            # Chain counts go in here, between sweeps, when the chain is idle.
            if getattr(self, '_dsp_chain_dirty', True):
                self._dsp_apply_chain()
                need = sum(self._dsp_chain_counts())
                if dwell < need:
                    return fallback(f"dwell {dwell} is below the {need} the "
                                    f"FPGA DSP chain needs (FLUSH_N + ACCUM_N)")

            t_before_exec = time.monotonic()
            ts0 = self._nios_timestamp()
            if DSP_STEPPER:
                # FPGA-timed sweep: dwell in 80 MHz ticks. The step boundary
                # is exact, the Nios is idle throughout.
                ticks = int(round(dwell * SYS_CLOCK_HZ / float(self.driver.sample_rate)))
                rc = self._nios_command(NIOS_CMD_EXEC_FPGA, ticks & 0x0FFFFFFF)
            else:
                rc = self._nios_command(NIOS_CMD_EXEC,
                                        (units << 16) | (num_steps & 0xFFFF))
            if rc != 0 or self._nios_timestamp() <= ts0:
                self._nios_unavailable = True
                print("[sfcw] NIOS autonomous sweep unavailable on this FPGA "
                      "image (sample counter not running) -- using the "
                      "standard sweep for this session.")
                return fallback("EXEC rejected, or the sample counter is not "
                                "running", reprime=True)
            t_after_exec = time.monotonic()

            # One sweep of acquisition, plus slack for the USB round trip.
            budget = (num_steps * dwell) / float(self.driver.sample_rate) + 1.0
            h_cal = self.driver.dsp_read_sweep(num_steps, timeout_s=budget)
            if h_cal is not None:
                self._dsp_timing(t_before_exec, t_after_exec, time.monotonic())
            if h_cal is None:
                # No burst: the FIFO ended the sweep short of num_steps (a
                # step's accumulation was cut by the next restart), and the
                # leftover would make the NEXT burst span two sweeps. Toggle
                # bit 6 -- v11 clears the FIFO on that -- and keep the stream:
                # two GPIO writes instead of the ~1 s teardown/rebuild.
                # Only a run of misses gets the full rebuild, in case the
                # sync worker itself has died.
                self._dsp_misses = getattr(self, '_dsp_misses', 0) + 1
                if self._dsp_misses >= DSP_MISSES_BEFORE_REBUILD:
                    self._dsp_misses = 0
                    try:
                        self.driver.stop_rx_dsp()
                    except Exception:
                        pass
                    return fallback("DSP FIFO read returned no complete sweep "
                                    f"{DSP_MISSES_BEFORE_REBUILD} times running; "
                                    "rebuilding the RX stream")
                try:
                    self.driver.dsp_resync()
                except Exception:
                    pass
                return fallback("DSP FIFO read returned no complete sweep; "
                                "FIFO resynced for the next one")
            self._dsp_misses = 0
        except Exception as e:
            return fallback(f"DSP sweep raised {e!r}")

        h_cal = np.asarray(h_cal, dtype=np.complex128)

        # complex_div flags a zero denominator by returning zero, so a step with
        # no reference signal arrives as exactly 0 rather than as a NaN.
        dropped = int(np.count_nonzero(h_cal == 0))
        if dropped > num_steps // 5:
            return fallback(f"{dropped}/{num_steps} steps had no reference "
                            f"signal")

        if progress_cb:
            progress_cb(num_steps - 1)

        self._last_sweep_core = 'dsp'
        return h_cal, dropped, None

    def _dsp_timing(self, t_before_exec, t_after_exec, t_burst):
        """Where one DSP sweep period goes, summarised every 30 s.

            period    = this EXEC to the previous one (1/rate)
            exec_cmd  = the EXEC command's USB round trip (2 timestamp reads
                        + the command)
            acquire   = EXEC returned -> burst in hand (the FPGA/Nios sweep
                        + USB delivery)
            host      = previous burst in hand -> this EXEC issued (decode,
                        IFFT, broadcast, loop)

        Measured because hiding the host term behind the acquisition
        (EXEC pipelining, 80e218c, reverted) did not change the rate: the
        period must then be set elsewhere, and this says where.
        """
        st = getattr(self, '_dsp_tm', None)
        if st is None:
            st = {'t0': t_before_exec, 'n': 0, 'period': 0.0, 'exec': 0.0,
                  'acq': 0.0, 'host': 0.0, 'prev_exec': None, 'prev_burst': None}
            self._dsp_tm = st
        if st['prev_exec'] is not None:
            st['n'] += 1
            st['period'] += t_before_exec - st['prev_exec']
            st['exec'] += t_after_exec - t_before_exec
            st['acq'] += t_burst - t_after_exec
            st['host'] += t_before_exec - st['prev_burst']
        st['prev_exec'] = t_before_exec
        st['prev_burst'] = t_burst
        if t_burst - st['t0'] >= 30.0 and st['n']:
            n = float(st['n'])
            print("[sfcw] DSP timing over {} sweeps: period {:.2f} ms ({:.1f}/s) "
                  "= exec_cmd {:.2f} + acquire {:.2f} + host {:.2f} ms".format(
                      st['n'], 1e3 * st['period'] / n, n / st['period'],
                      1e3 * st['exec'] / n, 1e3 * st['acq'] / n,
                      1e3 * st['host'] / n))
            st.update({'t0': t_burst, 'n': 0, 'period': 0.0, 'exec': 0.0,
                       'acq': 0.0, 'host': 0.0})

    def _sweep_dispatch(self, freqs, qt_rx, qt_tx, num_buffers, settle_count,
                        progress_cb=None):
        """Route one sweep to the selected core. 'standard' is the validated
        USB-retune-per-step path and stays the default; 'nios' is the FPGA
        autonomous sweep, and every failure inside it falls back to standard,
        so a regression is one set_params away from being undone."""
        if self.sweep_mode == 'dsp' and not self._nios_unavailable:
            return self._sweep_core_dsp(freqs, qt_rx, qt_tx, num_buffers,
                                        settle_count, progress_cb)
        if self.sweep_mode == 'nios' and not self._nios_unavailable:
            return self._sweep_core_nios(freqs, qt_rx, qt_tx, num_buffers,
                                         settle_count, progress_cb)
        self._last_sweep_core = 'standard'
        return self._sweep_core(freqs, qt_rx, qt_tx, num_buffers,
                                settle_count, progress_cb)

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

        h_cal, dropped_steps, adc_peak = self._sweep_dispatch(
            freqs, qt_rx, qt_tx, num_buffers, settle_count, progress)
        if h_cal is None:
            return None

        if dropped_steps > 0:
            if self.sweep_mode == 'dsp' and self._last_sweep_core == 'fallback':
                # The DSP core dropped the whole sweep on purpose (see
                # fallback() in _sweep_core_dsp). Say WHY, on a timer -- the
                # bare "incomplete captures" line at sweep rate buried the
                # reason on 2026-09-11.
                now = time.time()
                last = getattr(self, '_dsp_warn_last', 0.0)
                if now - last >= NIOS_FALLBACK_LOG_PERIOD_S:
                    self._dsp_warn_last = now
                    reason = getattr(self, '_dsp_last_fallback_reason', '?')
                    print(f"[sfcw] WARNING: DSP sweep fell back -- {reason} "
                          f"-- {dropped_steps}/{num_steps} steps empty; the "
                          f"[bladerf] lines above say what the read saw")
            else:
                print(f"[sfcw] WARNING: {dropped_steps}/{num_steps} steps had incomplete captures")

        self._warn_if_adc_hot(adc_peak)
        result = self._process_h_cal(h_cal, adc_peak)
        # Which core actually produced this sweep, plus autonomous-sweep diag.
        # 'fallback' means the nios path failed THIS sweep and the data is a
        # (correct, slower) standard sweep -- a block mixing flavors scores
        # terribly on mixture-blind metrics, so consumers need to know.
        result['sweep_core'] = self._last_sweep_core
        if self.sweep_mode == 'nios':
            result['nios_diag'] = {
                'score': round(self._nios_last_score, 3),
                'period': round(self._nios_period, 2),
                'realigned': self._nios_realigned,
                'fallbacks': self._nios_fallbacks,
                'lag_ms': round(self._nios_capture_lag /
                                float(self.driver.sample_rate) * 1e3, 2),
                'span_ok': self._nios_last_span_ok,
            }
        return result

    def _perform_sweep_raw(self):
        """Like _perform_sweep but returns raw h_cal array for averaging."""
        with self._lock:
            start = self.start_freq
            stop = self.stop_freq
            step = self.step_size
            num_buffers = self.num_buffers
            settle_count = self.settle_count

        freqs, qt_rx, qt_tx = self._build_sweep_grid(start, stop, step)

        h_cal, _, adc_peak = self._sweep_dispatch(freqs, qt_rx, qt_tx, num_buffers, settle_count)
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
        ref_tone_c64 = self._ref_tone_c64
        demod_int16 = self._demod_int16
        rx_cond = self._rx_cond
        stop_event = self._stop_event

        dropped_steps = 0
        backlog_drained = 0
        lockstep_misses = 0
        # Derived from the buffer actually in use, never a second literal: the
        # settle gate is expressed entirely in buffer periods, so a stale
        # constant here silently mis-scales the deadline AND the lockstep band.
        buf_period = float(self._rx_buffer_samples) / float(self.driver.sample_rate)
        settle_s = settle_count * buf_period
        lo_gap = LOCKSTEP_LO * buf_period
        hi_gap = LOCKSTEP_HI * buf_period
        # Peak |I|,|Q| seen on each RX, in ADC counts of 2047 full scale. Nothing in
        # this repo checked ADC headroom before 2026-08-29, and a too-hot reference was
        # the entire cause of the variability investigated then -- it is cheap to
        # measure and it is the first thing to look at when sweeps get noisy.
        adc_peak_rx1 = 0.0
        adc_peak_rx2 = 0.0

        for i in range(num_steps):
            if stop_event.is_set():
                # THREE values, like the success path. This returned `None, 0`
                # until 2026-08-31 -- missed when adc_peak was added on 2026-08-29 --
                # so every stop mid-sweep raised
                #   ValueError: not enough values to unpack (expected 3, got 2)
                # in _perform_sweep / _perform_sweep_raw. It looked harmless because
                # _sweep_loop's except falls straight into a finally that stops TX/RX
                # anyway, which is what stopping was about to do -- but it took the
                # exception path to get there, printed "[sfcw] Sweep error" on every
                # single stop, and pushed a bogus {'error': ...} to the groundstation.
                # Worst of all it buried real sweep errors in noise the operator had
                # learned to ignore. adc_peak is None here rather than a dict; every
                # consumer already guards it (_warn_if_adc_hot's `if not adc_peak`).
                return None, 0, None

            f = int(freqs[i])
            if use_qt:
                libbladeRF.bladerf_schedule_retune(dev_ptr, rx_ch, 0, f, qt_rx[i])
                libbladeRF.bladerf_schedule_retune(dev_ptr, tx_ch, 0, f, qt_tx[i])
            else:
                libbladeRF.bladerf_set_frequency(dev_ptr, tx_ch, f)
                libbladeRF.bladerf_set_frequency(dev_ptr, rx_ch, f)

            t_retune = time.perf_counter()
            with rx_cond:
                # Settle in WALL TIME, then prove we are back in lockstep with the
                # hardware before believing a buffer.
                #
                # The old gate counted buffer DELIVERIES (`_rx_seq + settle_count`),
                # which is not the same as waiting. Under CPU contention -- the
                # asyncio JSON/websocket task in sdr_server is enough -- the RX
                # thread gets starved and buffers pile up in libbladeRF's 16-deep
                # ring. The sweep then consumes settle_count of them in ~zero wall
                # time, so every one, and the capture after them, still holds
                # PRE-RETUNE IQ at the previous frequency: a fully corrupted step.
                # Measured 2026-09-05: 0/25500 bad cells with the engine running
                # alone, 1/6120 through the full server. Raising settle_count could
                # never fix it (163,200 cells over settle 1..10 showed no trend) --
                # skipping N backlogged buffers costs no time and skips no history.
                #
                # (1) real time since the retune, so the hardware has actually
                #     produced settled samples, and (2) drain until a buffer takes
                #     real time to arrive, which is what proves the backlog is gone
                #     and `_rx_latest` is genuinely current.
                # + buf_period, and that term is structural, not slack. A buffer
                # arriving at T holds the samples captured in [T - BP, T], so
                # gating on its ARRIVAL says nothing about its CONTENTS: without
                # this term a buffer arriving one period after the retune starts
                # its capture AT the retune, and anything earlier straddles it.
                # Measured before it existed: implied capture start ran down to
                # 0.106*BP (43 us) after the retune, 11% of steps got under 0.2 ms,
                # and 2 sweeps in 1199 were still corrupted after the lockstep bug
                # above was fixed. With it, settle_count = N means N buffer periods
                # of genuinely settled signal inside the capture window.
                #
                # settle_count therefore DEFAULTS TO 0, and that is not "settling
                # off" -- it is "capture the first buffer that lies entirely after
                # the retune". Extra settling was measured to buy nothing: at
                # settle_count=0 over 77,542 steps the tightest capture began just
                # 9.7 us after the retune, and the 1803 steps with under 41 us of
                # margin had a worst robust-z of 2.81 against 3.7 for the run as a
                # whole -- i.e. the least-settled captures were the cleanest. That
                # is what quick-tune fastlock is supposed to do; the AD9361 is
                # long since settled by the time a whole buffer has elapsed. Raise
                # settle_count only with a per-step check (robust-z per (sweep,
                # step) cell), never on an aggregate correlation -- see CLAUDE.md.
                deadline = t_retune + buf_period + settle_s
                while True:
                    rem = deadline - time.perf_counter()
                    if rem <= 0:
                        break
                    rx_cond.wait(timeout=rem)

                # Drain to lockstep, bounded by WALL TIME rather than by a count
                # of drained buffers. The count budget (MAX_BACKLOG_DRAIN = 24)
                # was the residual-corruption path, found 2026-09-06: on
                # exhausting it the loop fell straight through and captured
                # `_rx_latest` ANYWAY -- a buffer the gate had just failed to
                # prove current, i.e. exactly the stale pre-retune IQ the whole
                # gate exists to reject. Measured on this rig: the RX thread is
                # descheduled for up to 50.9 ms at a time (confirmed by the
                # stall-then-drain signature -- a stall of N buffer periods is
                # followed by exactly N near-zero-gap buffers emptying
                # libbladeRF's 16-deep ring), and a sustained starvation burst
                # can burn 24 drains across several stall/drain cycles without
                # ever catching a clean full-period gap.
                #
                # Waiting longer is ALWAYS safe for correctness here: the retune
                # has already happened, so a late buffer is still at the right
                # frequency -- only an EARLY one is wrong. So the right trade is
                # to convert a rare corrupt step into a rare slow one, which is
                # what an operator asked for ("corrupted values still randomly
                # come in"). STALL_GIVEUP_S is 5x the worst stall observed;
                # reaching it means something far outside normal contention, and
                # only then is an unproven buffer used rather than hanging.
                drains = 0
                locked = False
                stall_deadline = deadline + STALL_GIVEUP_S
                while True:
                    # Test what is already in hand before waiting for more. The
                    # wall-clock sleep above usually overshoots into a buffer that
                    # already qualifies, and unconditionally waiting for the NEXT
                    # one cost a further period per step for nothing.
                    if (self._rx_t is not None and self._rx_t >= deadline
                            and lo_gap <= self._rx_gap <= hi_gap):
                        locked = True
                        break
                    last_seq = self._rx_seq
                    while self._rx_seq <= last_seq:
                        rem = stall_deadline - time.perf_counter()
                        if rem <= 0:
                            break
                        if not rx_cond.wait(timeout=rem):
                            break
                    if self._rx_seq <= last_seq:
                        break          # gave up waiting: nothing new before the deadline
                    drains += 1
                    backlog_drained += 1
                if not locked:
                    # Only reachable now by blowing the wall-clock budget, not by
                    # a mere burst of backlog. A nonzero count here means the RX
                    # thread was starved for longer than STALL_GIVEUP_S, which is
                    # a system problem (contention), not a radar one -- and this
                    # step's data should be treated as suspect.
                    lockstep_misses += 1

                sig_bufs = []
                ref_bufs = []
                # The buffer the gate accepted is already proven current and
                # post-settle -- use it rather than waiting for another, which
                # cost a whole buffer period per step (21 ms/sweep at 51 steps)
                # for no extra safety.
                if self._rx_latest is not None:
                    sig_bufs.append(self._rx_latest[0])
                    ref_bufs.append(self._rx_latest[1])
                last_seq = self._rx_seq
                while len(sig_bufs) < num_buffers:
                    while self._rx_seq <= last_seq:
                        if not rx_cond.wait(timeout=1.0):
                            break
                    if self._rx_seq <= last_seq:
                        break
                    last_seq = self._rx_seq
                    sig_bufs.append(self._rx_latest[0])
                    ref_bufs.append(self._rx_latest[1])

                t_accept = time.perf_counter()

            if sig_bufs:
                # mean(iq * tone) IS a dot product, so it is one BLAS call and no
                # intermediates. The float64 expression this replaced allocated five
                # temporaries per channel per step -- two strided float64 slices for I
                # and Q, a complex128 from the 1j*Q, another from the addition, another
                # from the tone multiply -- about 320 kB of traffic per channel to
                # produce ONE complex number. int16 -> float32 -> view(complex64) is a
                # single pass and the view is free, because float32 I,Q pairs already
                # have exactly the complex64 layout. See _ref_tone_c64 for the accuracy
                # check. adc_peak is taken on the int16 directly (via Python ints, so
                # negating a hypothetical -32768 cannot overflow) rather than on a
                # float64 copy that no longer exists.
                nb = len(sig_bufs)
                acc_s = 0j
                acc_r = 0j
                for sb, rb in zip(sig_bufs, ref_bufs):
                    # Slice to the tone's length before the view. The buffer is
                    # RX_BUFFER_SAMPLES long because that is what keeps the RX
                    # arrivals aligned to libbladeRF's DMA buffer; the correlation
                    # is DEMOD_SAMPLES long because that is what puts LO leakage on
                    # an exact null of the rectangular window's sinc. The slice is
                    # of a contiguous int16 array from the front, so the astype is
                    # one pass and the complex64 view is still free. adc_peak is
                    # taken on the same slice, so it describes exactly the samples
                    # h_cal was computed from.
                    sbu = sb[:demod_int16]
                    rbu = rb[:demod_int16]
                    acc_s += np.dot(sbu.astype(np.float32).view(np.complex64), ref_tone_c64)
                    acc_r += np.dot(rbu.astype(np.float32).view(np.complex64), ref_tone_c64)
                    p1 = max(int(sbu.max()), -int(sbu.min()))
                    if p1 > adc_peak_rx1:
                        adc_peak_rx1 = p1
                    p2 = max(int(rbu.max()), -int(rbu.min()))
                    if p2 > adc_peak_rx2:
                        adc_peak_rx2 = p2
                nsamp = len(ref_tone_c64)
                h_signal[i] = acc_s / (nb * nsamp)
                h_reference[i] = acc_r / (nb * nsamp)
            else:
                dropped_steps += 1

            if SFCW_DIAG and self._diag_steps_n < DIAG_MAX_STEPS:
                # h_cal for THIS step is recorded alongside its own timing, so an
                # outlier step can be tied to the margin it was captured with
                # without aligning against the websocket stream.
                hr = h_reference[i]
                hc = (h_signal[i] / hr) if abs(hr) > 1e-12 else 0j
                self._diag_steps[self._diag_steps_n] = (
                    i, drains, self._rx_gap, t_accept - t_retune,
                    backlog_drained, 1.0 if locked else 0.0,
                    hc.real, hc.imag)
                self._diag_steps_n += 1

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
            # Vectorised for the same reason as sdr_server's broadcast rounding:
            # a Python comprehension over 102 elements holds the GIL for ~0.55 ms
            # on the SWEEP thread, which is ~1.3 RX buffer periods of backlog
            # handed straight to the next step. np.round matches round()'s
            # half-to-even, so the output is identical.
            'h_cal_real': np.round(h_cal_real, 8).tolist(),
            'h_cal_imag': np.round(h_cal_imag, 8).tolist(),
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
