# NIOS Autonomous SFCW Sweep

Goal: cut the 51-step sweep from ~171 ms by moving the per-step retunes off USB
and onto the FPGA, without losing phase coherence.

Status as of 2026-08-22: **flashed and measured.** The firmware works and the
data is correct, but on the Nios II/e core it does not beat a properly tuned
host-driven sweep. The sweep did get 2.1x faster -- from two latent bugs found
along the way, not from the FPGA. See "Results" below before doing more work
here.

## Results (100 sweeps each, bar: every consecutive pair >= 0.999)

| config | sweep | corr mean | corr min | pairs under bar | verdict |
|---|---|---|---|---|---|
| standard, settle 10 (was shipping) | 355.9 ms | 0.99986 | 0.99976 | 0 | pass |
| standard, settle 2 (**new default**) | 188.4 ms | 0.99986 | 0.99977 | 0 | pass |
| standard, settle 1 | 167.4 ms | 0.99986 | 0.99975 | 0 | pass |
| standard, settle 0 | 146.6 ms | 0.99891 | 0.99620 | 39 | **fail** |
| nios, dwell 20480 | 176.0 ms | 0.99985 | 0.99971 | 0 | pass |

Every row was run twice. The NIOS sweep agrees with the standard sweep to
0.99982, so the firmware, the alignment and the slicing are all correct -- it
is simply not faster.

**Why the FPGA does not win.** Per step the firmware issues 42 AD9361 SPI
transactions (2 x 18 to load the next profile, plus recall and port select).
Measured on hardware, the Nios II/e cannot step faster than **2.003 ms**,
whatever dwell it is asked for:

| asked | actual |
|---|---|
| 819 us | 2003 us |
| 1638 us | 2003 us |
| 2458 us | 2452 us |
| 3277 us | 3281 us |

That is 47.7 us per SPI transaction, about 3800 cycles at 80 MHz -- almost all
of it CPU, since the transaction itself is 0.6 us on a 40 MHz bus. 51 steps
gives a 102 ms floor, and roughly 70 ms of host-side capture handling sits on
top.

**What would make it win:** the `Fast` Nios II/f core (`build_bladerf.sh -n
Fast`, needs Quartus Standard or Pro rather than Lite). At roughly one cycle
per instruction instead of six, the step floor should fall to ~350-400 us and
the sweep to ~20 ms of stepping. That is the single change worth trying.

## Why this is needed

Measured on the bench (`pi/radar/benchmark_nios.py`,
`pi/radar/test_settle_sensitivity.py`):

| | ms/step | ms/sweep |
|---|---|---|
| production sweep, settle_count=7 | 3.36 | 171 |
| same, settle_count=0 (incoherent, floor probe) | 2.75 | 140 |
| bare USB retune pair, no capture at all | 2.44 | 124 |

The retune and the buffer wait overlap, so removing USB retunes does not save
2.44 ms per step directly. What it does is remove the **124 ms floor**. Below
about six buffers of settle the sweep is USB-bound and nothing done in software
can help. Coherence collapses below settle_count=4, so the software-only best
case is ~157 ms — a 9% win. Everything beyond that needs the FPGA.

## The blocker, and the HDL change

The plan was to schedule steps against the RX sample counter via
`time_tamer_read()`, and have the host slice its capture at the T0 the NIOS
reports. Both halves depend on that counter running.

It does not run. `rx.vhd` and `tx.vhd` both contain:

```vhdl
set_timestamp_reset : process(rx_clock, rx_reset)
begin
    if( rx_reset = '1' ) then
        timestamp_reset <= '1';
    elsif( rising_edge(rx_clock) ) then
        if( meta_en = '1' ) then      -- only in metadata mode
            timestamp_reset <= '0';
        else
            timestamp_reset <= '1';
        end if;
```

so in plain `SC16_Q11` the counter is pegged at zero. Confirmed on hardware:
`bladerf_get_timestamp()` returns rc=0 and value 0 for both directions while
streaming.

Three ways out were considered:

1. **Use metadata mode.** Rejected: `SC16_Q11_META` on dual RX overruns on
   essentially every buffer on this xA9 (2989 of 3000, immediately, with
   non-monotonic timestamps) — see `pi/radar/test_meta_mode.py`.
2. **Use a different NIOS clock.** There isn't one.
   `hdl/fpga/platforms/bladerf-micro/build/nios_system.tcl` instantiates no
   timer peripheral; the only time sources are `rx_tamer` and `tx_tamer`.
3. **Let the counter run whenever the receiver runs.** Chosen.

The change is one line in
`hdl/fpga/platforms/bladerf-micro/vhdl/rx.vhd`:

```vhdl
            if( meta_en = '1' or rx_enable = '1' ) then
```

It additionally keeps the counter running for `SC16_Q11` streams, which is the
point. It is *close to* behaviour-preserving for metadata users but not exactly:
`meta_en_rx` comes from a Nios GPIO bit (`nios_gpio.o.meta_sync`) while
`rx_enable` comes from the FX3 side, so their ordering is not guaranteed. If
`rx_enable` rises first, a metadata stream's first timestamp is non-zero rather
than ~0. Nothing here is exposed to that -- dual-RX metadata is unusable on this
board anyway, and RF Calib is single-channel `SC16_Q11` -- but it is a real
caveat, not a no-op.

`timestamp_reset` has exactly one producer (this process) and one consumer
(`rx_tamer`'s `ts_reset`), so the blast radius is that one signal. Of the four
things that read the counter, `retune2`'s `duration` field is log-only, and
scheduled retunes could never fire against a frozen counter, so the only
behavioural change that reaches anything is the metadata case above.

This does mean the build is no longer NIOS-only. Since the NIOS firmware is
embedded in the FPGA's on-chip memory initialisation, a full synthesis was
required regardless, so the marginal cost is zero.

## Protocol

Sweep commands ride inside the existing `pkt_retune2` ('U') packet, encoded as
sentinel values in the upper 32 bits of the timestamp field. `0x5357xxxx` is
about 1.8 million years out at 10 MSPS, so it can never collide with a real
sample count, and the normal retune path is untouched.

| sentinel | command | low 32 bits | response |
|---|---|---|---|
| `0x53575052` "SWPR" | prime | step count | 0 |
| `0x53574550` "SWEP" | exec | `[31:16]` dwell÷64, `[15:0]` steps | T0 |
| `0x53575354` "SWST" | stop | — | 0 |
| `0x53575147` "SWQG" | query | — | progress + worst lateness |

`bladerf2_schedule_retune()` passes the timestamp through without validating it,
so no host library change is needed.

**Priming** replays the sweep as ordinary `RETUNE_NOW` packets while the NIOS
records their profile indices. The sweep can therefore only ever replay
frequencies the host actually walked through. RX and TX are counted separately,
so the handler does not depend on their interleaving.

**Execution** activates step 0 immediately, returns
`T0 = now + step_interval`, and steps the rest from the idle path.

## Why the dwell can be short

`adi_fastlock_load()` is 18 SPI writes; `adi_fastlock_recall()` is 1. Loading a
profile at the step boundary would dominate the dwell. Because libbladeRF
assigns `rffe_profile = nios_profile % 8`, consecutive steps always land in
different RFFE slots, so step N+1 can be loaded while step N is still dwelling
without disturbing the live slot.

Simulation measures the critical path at **6 SPI transactions** per step
(recall + port select, per module) against 42 unpipelined — the 36-write load is
off the critical path.

## Finding T0 in the capture

The host has to know which sample the sweep starts on. Three things make that
harder than it looks.

**T0 cannot be read back.** The NIOS returns it in the retune2 response's
64-bit `duration` field, but libbladeRF unpacks that straight into a
`log_verbose` and drops it (`nios_access.c`), and `bladerf_schedule_retune()`
returns only a status. So the host never sees it.

**The timestamp anchor is too coarse.** Reading the sample counter either side
of the EXEC command brackets T0 to within the USB round-trip, about 2.4 ms.
That is several dwells wide, so it cannot pick the right step.

**Scoring candidate offsets by coherence does not work.** The obvious metric --
slide the capture window and maximise the reference channel's coherent
magnitude -- has many equal maxima. A window shorter than the settle region can
sit entirely inside a *neighbouring* step's clean tail and score exactly as
well as the aligned one. Measured: it missed by 2936 samples even when handed a
perfect anchor.

What works is reading the boundaries out of the data:

1. **Grid phase.** Inside a dwell the synthesizer is locked and the
   sample-to-sample phase advance is constant; while settling it is scrambled.
   Folding that indicator modulo the dwell stacks all 51 transients on top of
   each other, and the longest never-transient stretch is the capture window.
   This gives the boundary position to the sample -- but only modulo the dwell,
   since every step looks alike.

2. **Which grid cell.** The firmware activates step 0 at `T0 - dwell` and step 1
   at `T0 + dwell`, then stops. So the transient train is

   ```
   T0-dwell   (nothing at T0)   T0+dwell ... T0+(n-1)*dwell   (silence)
   ```

   with a **missing tooth exactly at T0**. The last transient is therefore
   `T0 + (n-1)*dwell`, which pins T0 absolutely. No template and no firmware
   sync mark needed -- it falls out of the existing timing.

The result is validated structurally rather than against the anchor: steps
1..n-1 must all be transient and T0's own cell must not be. That makes the
alignment independent of how buffer sequence numbers map onto sample counts,
which is the one assumption that cannot be checked until the firmware is
flashed.

## Validation without hardware

Two harnesses, both runnable on the Pi.

**`pi/radar/nios_sim/`** compiles the *real* `pkt_retune2.c` against a mock
AD9361 and drives the state machine.

```
make -C pi/radar/nios_sim run
```

30 checks: priming order, step sequencing, dwell timing, that no step fires
early, pipelining effectiveness, that no preload targets the live slot, query
reporting, stop, degenerate inputs, and that the normal `RETUNE_NOW` /
`CLEAR_QUEUE` / scheduled-retune paths still behave.

This caught a real off-by-one: the first version preloaded `current_step + 1`
after already advancing `current_step`, so it staged step N+2 and never N+1 --
every boundary paid for two loads plus a wasted prefetch, 78 SPI transactions,
worse than not pipelining at all.

**`pi/radar/test_nios_slicing.py`** builds a synthetic capture with the same
transient pattern the firmware produces and runs the engine's real alignment
and slicing over it. Results:

- T0 recovered **exactly** for anchor errors from -30000 to +30000 samples
  (±3 ms, comfortably wider than the round-trip)
- `h_cal` error **4e-16 in float** -- the slicing is mathematically exact
- with int16 samples the error is ~7e-4, scaling as 1/amplitude, i.e. pure
  quantisation, which the hardware has too
- a capture with no sweep in it, and one too short to hold the sweep, are both
  rejected rather than silently mis-sliced

**`pi/radar/test_nios_fallback.py`** exercises the host path against firmware
that has no sweep support: the sample counter reads zero, the engine notices
and hands back to the standard sweep. Not yet run -- it needs the bladeRF, which
was busy.

## Building

`~/bladerf-src/build_sweep_firmware.sh`, on an x86-64 machine with Quartus
Prime Lite. `nios2-elf-gcc` and `quartus_asm` have no ARM builds, so this cannot
run on the Pi. Expect 20–45 minutes.

Load with `bladeRF-cli -l hosted.rbf` (RAM, lost on power cycle) while testing.
Only write it to flash (`-L`) once validated.

## Files

| file | role |
|---|---|
| `~/bladerf-src/.../src/pkt_retune2.c` | sweep state machine (only firmware file changed) |
| `~/bladerf-src/.../vhdl/rx.vhd` | one-line tamer gating change |
| `~/bladerf-src/build_sweep_firmware.sh` | build driver |
| `pi/radar/nios_sim/` | host-side state machine simulation |
| `pi/radar/benchmark_nios.py` | baseline timing + coherence |
| `pi/radar/test_settle_sensitivity.py` | settle vs coherence vs USB floor |
| `pi/radar/test_meta_mode.py` | metadata mode viability |
| `pi/radar/test_rx_alignment.py`, `probe_rx_rate.py` | buffer ↔ sample-count mapping |

## Host side

`sweep_mode` selects the core: `'standard'` (default, unchanged) or `'nios'`.
`nios_dwell` and `nios_settle` are in samples; the dwell is rounded down to a
multiple of 64 because that is the granularity the protocol sends.

Every failure path falls back to the standard sweep and says why, so selecting
`'nios'` on firmware that does not support it degrades instead of breaking.
The fallback also issues `bladerf_cancel_scheduled_retunes()`: firmware without
sweep support does not recognise the sentinels, so it treats them as ordinary
scheduled retunes and enqueues them, and those entries would otherwise fill the
16-deep queue.

Priming is invalidated whenever the frequency grid changes or the stream
restarts, since the NIOS holds a recorded copy of the old grid and the sample
counter restarts with the stream.

## Still to do

- Run `test_nios_fallback.py` on hardware (needs the bladeRF free).
- Flash and validate: confirm the sample counter runs, then 50-sweep coherence
  ≥ 0.999, cross-check against the standard sweep, and walk the dwell down to
  the shortest that holds. `SWQG` reports the worst scheduling lateness, so a
  dwell that outruns the RFFE shows up as a number rather than as bad phase.
- Host processing is ~8 ms per sweep and is currently a fixed cost; at a 30 ms
  sweep that is worth vectorising further.

---

# Addendum 2026-09-07: Nios II/f image — measured, ported, and what changed

Everything above describes the Nios II/e image. A new image (Quartus Prime
**Standard** 18.1, `NIOS_REV=Fast`, same rx.vhd patch and sweep firmware) was
built 2026-09-06 and validated 2026-09-07. It is committed to the repo at
`fpga/images/hostedxA9_niosIIf_sweep_ts_v1.rbf` -- see `fpga/images/README.md`
for its sha256, build provenance, and how to load it. The Pi-side driver was ported from
`fpga_branch` onto the current `sfcw_engine.py` (`sweep_mode='nios'`,
`_sweep_core_nios` and friends) — the branch itself stays unmerged.

## Gate results

- **Timestamp counter (Gate 1): runs.** `bladerf_get_timestamp` advances at
  9.9929 Msps in plain SC16_Q11. The rx.vhd patch is in the image.
- **Step floor (Gate 2): ~303 us (~3030 samples), 6.6x the II/e's 2.003 ms.**
  Asked 1024 or 2048 samples of dwell, the firmware steps at ~3030 either way;
  asked 4096 it holds 4096-4107. The load/activate pipelining works — the
  floor is well under the 42-transaction unpipelined estimate.
- The II/f also services *host-driven* retunes ~6x faster: the standard sweep
  dropped from 65.5 ms to ~55.5 ms through the server on this image with no
  host change at all.

## What the II/f broke in the host-side alignment, and the fixes

1. **Transients are 16-33 SAMPLES wide** (1.6-3.3 us), vs hundreds-plus on the
   II/e. The k=256 boxcar in `_nios_detect_transients` diluted them below
   threshold (~20 of 51 steps found). Now k=64.
2. **The EXEC front matter is not on the T0 grid.** EXEC produces a transient
   pair ~1940 samples apart (step 0's activation) and the first regular
   boundary lands ~3.190 periods later (std 0.0055, measured over 199 sweeps)
   — the firmware reads "now" for T0 only after ~300 us of activation+response
   handling. On the II/e the dwell was 6x longer and this latency rounded
   away; on the II/f it is a large fraction of a dwell and the missing-tooth
   search mis-anchored. `_nios_refine_offset` now finds the run of
   period-spaced transients (steps 1..n-1), **REQUIRES it to span exactly n-2
   periods**, and anchors T0 at its FRONT (first member = step 1). When the
   span holds, the front and end anchors are algebraically identical and T0 is
   certain; when it does not, one end boundary was missed, nothing in the
   sweep can say which, and the sweep is REFUSED rather than guessed at
   (~1% of sweeps). Do NOT add a scene-similarity check to repair it: that was
   tried and it confuses a target entering the beam with a mis-slice, rotating
   good sweeps and latching until the sweep is restarted. See CLAUDE.md,
   "the resolver that had to go".
3. **The step period can run slightly LONG** (dwell 4096 -> 4107 measured, up
   to +0.65% at 4928), where the II/e only ran short (stream compression).
   The period gate is 1.5% now, and the stride is no longer clamped to the
   dwell.
4. **Dwell 3456 is above the rail but unstable** (period wobble +/-14 samples,
   slicer thrash). `NIOS_MIN_DWELL = 4096`; with the capture pipelined the
   sweep is processing-bound anyway, so a shorter dwell buys nothing.

## Sample loss is the failure mode that matters under load

A continuous capture is only sliceable if it is gapless. Under full-stack load
the RX thread stalls (50.9 ms worst measured); libbladeRF's 16-buffer ring
gave 3.3 ms of tolerance, and overflows DROP samples with no flag in
SC16_Q11 — measured 10-26% capture loss per failing sweep, which either fails
alignment loudly or (worse) slices at shifted positions that the +/-1-stride
realignment cheerfully accepts. Three fixes:

- `start_rx_dual` ring 16 -> **256 buffers** (52 ms of stall tolerance, 4 MB).
- `_sweep_core_nios` gates on `_nios_capture_lag` (hardware-clock elapsed
  minus delivered samples) against 75% of the ring before slicing. That
  quantity is delivery LAG, not loss -- 2-7 ms of it is ordinary backlog at
  30 Hz under full-stack load, and a first version that gated it as "loss" at
  a 0.5% threshold rejected every sweep.
- The lattice coverage bar is 0.75 (clean captures detect 78-100% of
  boundaries; a gap-split capture ~60%).

Every failure path still falls back to the standard sweep, so `sweep_mode`
remains one `sfcw_set_params` away from the fully-validated host-driven core.
