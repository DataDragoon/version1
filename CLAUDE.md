# CLAUDE.md — Project Instructions for Claude Code

## Project Overview

SFCW radar for within-wall imaging (rebar, pipes, voids, studs — not beyond the wall).
See CONTEXT.md for full system description.

## Key Facts

- This repo runs on TWO machines: Raspberry Pi (pi/) and PC (groundstation/)
- Clone the repo on both; run the appropriate code on each
- Pi never receives direct user input — all control via groundstation over LAN
- Heavy compute (SAR reconstruction, ML) belongs on the PC side
- Every subsystem must have a corresponding debug tool on groundstation

## Code Conventions

- Python for all Pi code (sensor drivers, radar control, networking)
- Python or TypeScript for groundstation (TBD based on UI framework choice)
- Shared protocol definitions in shared/protocols/
- Keep sensor interfaces minimal and async-friendly
- Prefer ZeroMQ or similar for IPC once protocol is chosen

## Hardware Context

- Raspberry Pi with AI HAT+ (Hailo-8L accelerator)
- LiDAR: TF-LC02 (UART, 115200 baud default) — **wired to `/dev/serial0`, not `/dev/ttyAMA0`.**
  On this Pi 5, `dtoverlay=uart0-pi5` (the GPIO14/15 header UART, `/boot/firmware/config.txt`)
  enumerates as `ttyAMA10`, which `/dev/serial0` symlinks to. `/dev/ttyAMA0` is a *different,
  always-present* PL011 UART used internally for Bluetooth (`hci_uart_bcm`) — it opens
  successfully with no error, so a driver defaulting to it doesn't crash, it just silently
  reads nothing forever. `pi/sensors/tflc02.py` `TFLC02.__init__` now defaults to
  `/dev/serial0`; don't change it back to `/dev/ttyAMA0`. The `config.txt` comment above the
  overlay line still says "creates /dev/ttyAMA0", which is wrong for the Pi 5 — go by
  `/dev/serial0` in code, not that comment.
- IMU: MPU-6500 (I2C, address 0x68)
- SDR: bladeRF (USB, use libbladeRF / pybladeRF)
- Antennas: 2x Vivaldi (wideband, one TX one RX)

**IMU failure must not take LiDAR streaming down with it (fixed 2026-08-24).**
`pi/sensors/stream.py` `sensor_loop()` used to construct `MPU6500()` *before*
`TFLC02()`. When the IMU isn't responding on the I2C bus (`OSError: [Errno 121]
Remote I/O error` — confirm with `i2cdetect -y 1`, address `0x68` absent), that
constructor throws and kills `sensor_loop` before the LiDAR is ever initialized —
so a dead/disconnected IMU presented as "the lidar isn't working" even though the
LiDAR wiring and driver were completely fine. `sensor_loop` now builds the LiDAR
first and wraps IMU init in try/except: on failure it logs a warning and streams
`accel`/`gyro`/`temp` as `null` while LiDAR keeps working normally. Keep this
independence — don't let either sensor's failure gate the other.

**Extended 2026-08-24: the per-iteration reads are guarded too.** Init-time protection
was not enough — an IMU that enumerates fine at startup can drop off the bus later, and
`MPU6500._read_raw` -> `read_i2c_block_data` then raises `OSError 121` on *every* loop
iteration. That exception escaped `sensor_loop`, hit `log_task_exception`, and called
`request_stop()` — killing the whole `stream.py` process, so port 9001 went dead and the
groundstation reconnect-looped. Symptom: LiDAR/standoff reads `—` in the SFCW, C-scan and
BG Model panels (they all share `App.jsx`'s `lidarMm`) and the sidebar's IMU Hz tile also
reads `—`, while the SDR panel on port 9003 keeps working normally — which looks like a
LiDAR bug but is the IMU killing the shared stream. `sensor_loop` now wraps the IMU read
in try/except (streaming nulls) and disables the IMU after `IMU_FAIL_LIMIT = 20`
consecutive failures, because each failing read costs an I2C timeout that would otherwise
throttle the LiDAR rate. The LiDAR read is guarded the same way.

**Diagnosing a missing standoff readout:** the sidebar's IMU Hz tile is on every panel and
tells the two cases apart. Hz blank -> the sensor stream (port 9001) is down, check
`stream.py`'s stdout on the Pi. Hz live but Standoff `—` -> the stream is up and
`read_distance()` is returning `None`, so it's the TF-LC02 serial path (`/dev/serial0`).

**LiDAR silent-serial investigation (2026-08-24), unresolved — needs a bench check, not
more code.** `read_distance()` returns `None` because the TF-LC02 gives back literally zero
bytes on `/dev/serial0` — confirmed both actively (sending the `55 AA 81 00 FA` query) and
passively (just listening for 3s with nothing sent), before *and* after a physical power
cycle of the LiDAR board. A powered TF-LC02 replies to something; total silence on both
fronts across a power cycle rules out a code/protocol bug and points at wiring/connector,
not firmware or timing. Two things already ruled out, don't re-check them: (1) the driver
protocol itself — byte-for-byte identical to a completely separate fork on this same machine
(`~/version-venom`, different GitHub remote), so it isn't a broken deviation from something
that used to work; (2) no lost/alternate C-level implementation exists anywhere — full git
history and a filesystem-wide search turned up nothing, the driver has only ever been this
Python file. Per CONTEXT.md, TF-LC02 VCC shares Pin 2 (5V) with the IMU — worth checking that
connection specifically since it's exactly the rail that would've been disturbed while
rewiring the IMU. Next step is physically checking the LiDAR's VCC/GND/TX/RX leads at the
Pi header, not another round of software changes.

**Voltage ruled out (2026-08-24).** Checked the bench wiring against the table above: VCC is
3.3V and *not* shared with the IMU, contrary to what the table previously claimed (fixed in
CONTEXT.md too) — but the TF-LC02 runs on 3.3V by design, and this exact connection was
already confirmed working before, so this is not a mis-wiring and not the cause of the
current silence. Back to square one on root cause: wiring (VCC/GND/TX/RX) is correct and
was previously functional, yet the module now gives zero bytes both actively and passively
across a power cycle. Worth checking next: whether the connector is fully seated (could have
been jarred loose during the IMU rework even though it's on different pins), continuity
along the full length of each lead (not just voltage presence) rather than just at the
header, physical damage to the module or leads from handling during the IMU swap, and
whether anything in `/boot/firmware/config.txt` (the `dtoverlay=uart0-pi5` line) regressed
since it last worked. Still a bench/hardware problem, not a code problem.

**Pi-side UART stack fully checked (2026-08-24) and ruled out.** `dtoverlay=uart0-pi5` intact
in `config.txt`, `/dev/serial0` -> `ttyAMA10` as expected, `dmesg` shows clean PL011 init with
no errors, nothing else has the port open (`lsof`), user is in `dialout`, and `pinctrl get
14,15` shows both correctly muxed to UART0 TXD0/RXD0 (`a4`, idle-high — normal resting state).
A live passive-listen + active-query test against `/dev/serial0` still returned 0 bytes both
ways. Everything software/OS-side on the Pi is healthy — this is now isolated to the module
itself or its TX/RX leads specifically (not VCC/GND, both already confirmed fine). Next step:
unplug the LiDAR and jumper the Pi's TX (pin 8) straight to RX (pin 10) for a bare loopback
test — if that echoes back what's sent, the Pi's UART is fully exonerated and the module/its
TX-RX leads are the remaining suspect (dead unit, or a lead nicked during the IMU rework).

**RESOLVED (2026-08-24): root cause was a dead UART0 receiver on this Pi's RP1 chip — not
the LiDAR module, not any wiring.** With a bare TX(pin 8)-RX(pin 10) short confirmed solid by
continuity meter, a raw GPIO bit-bang test (toggle GPIO14, read GPIO15 with both pins pulled
out of UART mode) passed perfectly — proving the pins and the physical short were both fine.
But a live UART0 loopback still returned 0 bytes, and `TIOCGICOUNT` (via `fcntl.ioctl`,
`0x545D`) showed why: `tx` incremented into the hundreds of thousands while streaming (real
transmit activity, confirmed independently by `/proc/interrupts` counting real IRQs on the
`uart-pl011` line), but `rx` stayed at exactly 0 with zero frame/overrun/parity errors —
not even noise, total silence on the receive side specifically. Cross-check: live-applying
a second UART (`sudo dtoverlay uart3-pi5`, no reboot needed, brings up `/dev/ttyAMA3` on
GPIO8/9 = physical pins 24/21) and shorting *those* pins instead gave a clean, byte-perfect
loopback (icount rx=5/tx=5 for 5 bytes sent) — so this is not a board-wide or software issue,
it's UART0's receive path specifically. **Fix shipped:** `uart0-pi5` disabled (commented, not
removed) and `uart3-pi5` made persistent in `config.txt`; `tflc02.py` `TFLC02.__init__`
now defaults to `/dev/ttyAMA3` (was `/dev/serial0`) — **the earlier instruction above to
default to `/dev/serial0` no longer applies now that UART0 is dead; don't move it back.**
LiDAR now wired to physical pins 24 (TX) / 21 (RX) instead of 8/10; VCC (3.3V, not shared
with the IMU — see above) and GND unchanged. Live-tested end to end afterward with clean,
stable readings.

**Also found and fixed while re-testing (2026-08-24): `read_distance()` ignored the
protocol's own error code.** The TF-LC02 response includes an `error_code` byte (offset 6)
that `_read_response()` computed but never checked — an invalid/no-return measurement comes
back as literal distance `8888` with `error_code=4`, and the old code returned `8888` as if
it were a real reading. Confirmed via `read_distance_with_error()` that ~30-40% of reads at
the test bench alternated between valid (`error_code=0`) and this invalid sentinel — normal
behavior for this class of sensor (weak/no return depending on target angle/reflectivity),
not a hardware fault. `_read_response()` now returns `None` when `error_code != 0`, so
callers see the same "no reading yet" signal they'd get from any other transient failure,
instead of a spurious 8.888 m jump in the standoff display.

**IMU was a red herring the whole time — it simply wasn't physically connected.** After the
LiDAR fix, `i2cdetect -y 1` showed nothing at any address, matching CONTEXT.md's existing
note that the BNO085's VCC/GND pins were never confirmed after the chip swap. User checked
the bench and confirmed the BNO085 breakout was not plugged in at all. Once connected, it
enumerates at `0x4A` as documented and streams real accel/gyro data (verified live: resting
pose read `up ≈ 1.0g`, matching the axis-remap calibration check above). The original theory
that started this whole investigation — IMU and LiDAR sharing a 5V rail, one dragging the
other down — was wrong on every count: LiDAR VCC turned out to be 3.3V and not shared with
the IMU at all, and the two failures were completely unrelated (one a dead UART peripheral,
the other a bench cable never plugged back in).

**IMU hardware swap (2026-08-24): MPU-6500 -> BNO085. Driver shipped, axis calibration
still pending.** `mpu6500.py` is deleted (confirmed nothing else imported it); `pi/sensors/
bno085.py` is the new driver, wired into `stream.py` in place of it. BNO085 lives at I2C
`0x4A` (was `0x68` for the MPU-6500) and speaks SHTP over I2C, not simple register
read/write — a hand-rolled raw driver on `smbus2`/`i2c_msg`, matching this repo's other
sensor drivers (no framework dependency). `adafruit-circuitpython-bno08x` +
`adafruit-blinka` are installed on the Pi (`pip3 install --break-system-packages`, not in
`requirements.txt`) from cross-checking the protocol during bring-up; not used by the
shipped driver, safe to leave installed or remove.

**Bring-up history, useful if this ever needs debugging again:** initially every
`SET_FEATURE_COMMAND` (enable accelerometer/gyroscope/rotation vector) got zero response
while Product ID queries worked fine — the classic signature of the SH-2 application
firmware not running (stuck in bootloader/reduced mode). Ruled out corrected per-channel TX
sequence numbers (adafruit's library conflates host-TX/device-RX sequence counters into one
list — a real bug, see its `__init__.py` `_sequence_number` TODO — but not the cause here),
long settle times, full channel drains. **Fixed by power-cycling the board** — confirms it
really was a firmware-not-running state, not a protocol bug. If this regresses, power-cycle
before re-chasing protocol theories.

Packet shape (`accel`/`gyro`/`temp` fields, same consumers) is unchanged from the MPU-6500
era rather than switching to the chip's onboard sensor fusion (rotation vector) — smaller
blast radius, `imu_calibration.py`'s axis remap and the groundstation's Madgwick filter in
`ImuDisplay.jsx` keep working unmodified. `bno085.py` converts the calibrated accelerometer
report (m/s^2, Q8) to g (÷9.80665) and gyroscope report (rad/s, Q9) to deg/s (×180/π), so
nothing downstream needed to change for units. BNO085's SH-2 report set has no plain
temperature report — `temp` streams `null` for this driver, which every consumer already
handles gracefully (see the null-accel `ImuDisplay.jsx` crash fixed the same day).

**Fixed (2026-08-24) after TWO rounds of live testing: `imu_calibration.py`'s R_ACCEL/R_GYRO
axis remap for the BNO085.** Round 1's fix (a simple X<->Y swap, reasoned from the gravity
measurement + "pitch reads as yaw") was WRONG on its own terms — round 2's live feedback
was "pitch and roll are now swapped," meaning the true axis identity is a full relabeling of
all three raw axes, not the two-axis swap round 1 assumed:
  raw X -> roll axis (forward), raw Z -> pitch axis (left), raw Y -> yaw axis (up)
This resolves self-consistently: yaw was never reported wrong across either round, so raw Y
= up = yaw axis stands throughout (also matches the direct gravity measurement). Current
`R_GYRO`: `roll = -gyro_x, pitch = +gyro_z, yaw = +gyro_y`. **`R_ACCEL`'s forward/left rows
are inferred, not independently measured** — set equal to `R_GYRO`'s rows on the reasoning
that BNO085's SH-2 reports are documented to share one common sensor frame across
accel/gyro/mag (true for this chip, was NOT true for the MPU-6500 — its original R_ACCEL and
R_GYRO were genuinely different matrices, so don't assume this equivalence generalizes to
other hardware without checking). Verified: fresh calibration + resting pose gives
`up ≈ 1.0g, forward ≈ 0, left ≈ 0`.

**If a third round of "X is now swapped with Y" feedback comes in, stop patching
incrementally and ask for a full walk-through instead** (rotate slowly through each of
roll/pitch/yaw individually, one at a time, reporting the sign and which body-frame value
actually moves for each) — two rounds of partial feedback already produced one wrong
intermediate fix (round 1's swap), and a third partial round risks the same. A single
complete pass pins all three axes and their signs at once instead of iterating on which pair
is currently swapped.

**Testing trap: don't use "does the resting pose look level" as a check that the axis
remap is fine — it's a false negative for detecting a *remaining* problem,** even though it
correctly reproduces `up ≈ 1g` for whichever mapping is currently in place (round 1's WRONG
mapping also passed this exact check). `auto_calibrate` reruns on every `stream.py` startup
and its accel-bias step subtracts whatever the raw reading was *at that moment* from the
expected-gravity vector — calibrating and then immediately reading the *same* static pose
cancels out any remaining wrong-axis error, "up" reads ~1g regardless of whether R_ACCEL is
actually correct. The only real test is dynamic: tilt the board through a known roll/pitch/
yaw and see if the *reported* axis and *sign* match the *physical* motion — same as the
original MPU-6500 discovery procedure, and the only thing that's actually caught a problem
so far in this whole BNO085 remap effort.

**Fixed: BNO085's report backlog could hang `calibrate_gyro_bias` indefinitely.** Initial
`bno085.py` used a 128-byte read buffer and 10ms/100Hz report intervals for both
accelerometer and gyroscope. Each I2C read transaction has real cost regardless of whether
data is pending -- measured 12ms for a 128-byte read on this Pi's I2C bus, vs. 3.2ms at 32
bytes -- and the BNO085 pushes reports on its own schedule whether or not the host is
draining them. At 100Hz+100Hz combined against ~12ms/read, `read_all()`'s drain loop could
never catch up: every call hit its 32-read cap, batched packets grew past the 128-byte
buffer and got silently truncated, and `calibrate_gyro_bias`'s 200Hz sampling loop (400
samples, meant to take 2s) didn't finish within 20s in testing. Fixed by cutting the read
buffer to 48 bytes and both report intervals to 20ms/50Hz (matching `stream.py`'s default
loop rate) -- confirmed steady-state `read_all()` now does 3-5 reads per call, never near
the cap, and calibration completes in ~2s as intended. If report intervals ever need to
drop for a smoother display, drop the per-call read cap first and re-measure steady-state
read count before assuming it's fine -- don't just lower the interval and move on.

**Fixed (2026-08-24): the whole sensor stream was capped at ~7-10Hz by the dead LiDAR,
not by the BNO085 swap.** `TFLC02.read_distance()` blocks on a 100ms UART timeout when the
sensor doesn't answer (measured directly: every call took exactly ~100ms while the LiDAR
issue above was unresolved), and `sensor_loop` used to `await` it inline in the same loop
that reads the IMU and broadcasts -- so a silent LiDAR throttled the *entire* stream to
~1/0.1s = 10Hz regardless of the `--rate` flag, IMU included. This was reported as "the IMU
feels choppy, worse than the MPU" -- plausible red herring, since the MPU-6500 setup was
presumably running while the LiDAR still answered quickly, so the same blocking-inline
pattern never showed up as a problem. `sensor_loop` now runs `imu_poll_loop` and
`lidar_poll_loop` as separate background tasks, each polling its sensor in its own uncapped
loop via `run_in_executor` and publishing the latest reading into a shared dict; the
broadcast loop just reads those dicts and never awaits either sensor directly. Verified: a
still-unresponsive LiDAR (100ms/read, unchanged) no longer affects IMU rate at all --
measured 48-48.7Hz broadcast throughput at `--rate 50`, up from 7.67Hz before this fix. This
also means the stream will keep running at full rate once the LiDAR hardware issue above is
eventually fixed, not just work around it today.

**bladeRF total-sample-throughput warnings explained (2026-08-24).** The
`check_total_sample_rate` warning in libbladeRF sums each active channel's *actual* achieved
sample rate (`bladerf2.c` reads it back via `get_sample_rate`), not the rate Python
requested. `bladerf_driver.py` `_configure_channels_dual()` was calling
`bladerf_set_sample_rate(..., ffi.NULL)` for the `actual` out-param — silently discarding
whether the RFIC's clock/decimation chain rounded the requested rate to something else.
SFCW's `_configure_hardware()` requests 10 Msps; the observed warning math (92.16 / 3
channels, 122.88 / 4 channels) both divide out to exactly 30.72 Msps per channel, so that's
almost certainly what the hardware actually snapped to. Fixed by capturing `actual` (both in
the raw-ffi dual-channel path and by reading back `Channel.sample_rate` after the
Python-wrapped single-channel sets) and logging a `[bladerf] NOTE: <ch> sample rate snapped
to X Msps (requested Y Msps)` line when it differs — visibility only, no behavior change, so
it's safe without live hardware to test against. **The 30.72 Msps hypothesis is now
FALSIFIED (measured on hardware 2026-08-28).** Calling `bladerf_set_sample_rate(..., actual)`
for 10 Msps on all four channels (RX0/RX1/TX0/TX1) returns `rc=0` with
`actual = 10.000000 Msps` on every one — the RFIC hits the requested rate exactly and does
not snap. So whatever produced the 92.16 / 122.88 warning totals, it was not SFCW's 10 Msps
request being silently rounded to 30.72. Dual-channel RX at 10 Msps also streams cleanly
(60/60 buffers, no timeouts), so sweep-time and RX-timeout problems should not be blamed on
sample-rate snapping — see the FPGA tuning-mode section below for what actually caused the
RX timeouts. Actually eliminating the warning (picking a request rate
the RFIC can hit exactly, e.g. a rate near the well-known 30.72 Msps LTE-grid family) needs
live-hardware validation before changing — RF gain/timing code in this repo has a history of
regressions from unverified changes (see the `settle_count`/`num_buffers` regressions
above), so don't just guess a lower rate to silence it without testing on the bench.
The `[INFO @ .../version.c]` firmware/FPGA-newer-than-compatibility-table lines are harmless
and expected — libbladeRF's bundled compatibility table just lags the flashed firmware/FPGA
versions; ignore them, don't chase a libbladeRF upgrade just to silence an INFO line.

## Living Documentation Rule

CLAUDE.md and CONTEXT.md are living documents. Whenever you learn key information
worth persisting — new design decisions, hardware findings, protocol choices,
calibration values, architectural changes, or anything a future session would
need to know — update CLAUDE.md and/or CONTEXT.md immediately. Don't wait to be
asked. These files are how context survives across sessions and collaborators.

## Current Phase

IMU, LiDAR, and bladeRF SDR integrated. All stream to groundstation debug panels.
RF Calib panel provides signal generator + oscilloscope for bladeRF calibration — always
runs both channels (antenna TX1/RX1 + reference TX2/RX2 loopback) simultaneously, viewport
split left (antenna) / right (reference) for both TX and RX.
SFCW panel performs stepped-frequency sweeps (2–5 GHz, hard-bounded by the quick-tune master
table's 256-profile hardware ceiling — see Quick-tune master table below) with range profile
+ waterfall display.
Both RF panels share port 9003 — starting an SFCW sweep auto-stops any active TX/RX in RF Calib.
C-scan panel rasters a 2D grid of positions over the target and shares the SFCW panel's
background model machinery (see below). It rasters either by hand or automatically via the
rover gantry — see "Rover-driven C-scan raster".
Rover Scan panel drives the 2-axis stepper gantry (continuous jog, nudge, calibration,
live position from the controller's own step counter) — see below. The automated grid
raster on top of it lives in the C-Scan panel's Rover scan mode, not here.
Imaging Bench panel replays an exported waterfall snapshot through 11 selectable imaging
effects for offline A/B of processing chains — see below.

## SFCW Amplitude Scaling (Dynamic / Manual)

The SFCW panel carries an `sfcwScaleRange = { dynamic, min, max, isDb }` (App.jsx), the
same shape the C-scan panel uses for its colour scale. Dynamic (the default) is the old
behaviour: the range profile's Y axis tracks session-wide extremes and the waterfall's
colour range tracks its visible history. Manual pins **both** panes to one pair of limits.

Seeding matters: the live limits are computed inside `SfcwDisplay`, not the panel, so the
display publishes them every frame through `onDynamicScale` into an App-level **ref**
(`sfcwDynamicScale`) — a ref, not state, so a 3–6 Hz sweep does not re-render the sidebar.
The panel reads it via `getDynamicScale()` at the moment the toggle is clicked, so switching
to manual never makes the colours or the axis jump.

`isDb` records which units the pinned numbers are in. Flipping the display's dB/LIN button
(or "Reset Scale") hands the scale back to dynamic, because dB limits are meaningless on a
linear trace. Panes flag a pinned scale with an amber `MANUAL` next to their title, and the
waterfall's colour-bar numbers turn amber too.

The other two `SfcwDisplay` instances (C-scan and BG Model live sweep) pass no `scaleRange`
and stay dynamic — `manual` is false whenever the prop is absent.

## Background Subtraction — Groundstation Only (SFCW + B-scan)

All background subtraction happens on the groundstation. The Pi ships raw `h_cal`,
holds no background state, and has no notion of a B-scan at all. These commands no
longer exist: `sfcw_capture_bg`, `sfcw_clear_bg`, `sfcw_bg_mode`, `bscan_clear_bg`,
`bscan_capture`, `bscan_bg_capture`, `bgmodel_capture`. Every "capture" now works by
tagging the next `sfcw_result` to arrive, groundstation-side, with a ref flag.

Both panels offer the same two mutually exclusive sources:
- **Captured reference** — "Capture BG" tags the next sweep as `sfcwBgRef` / `bscanBgRef`.
- **ML model** — "Load Model" infers a background from lidar standoff (`bgModelInfer.js`).

Selecting either clears the other; "Clear BG" clears both. Subtraction is always complex
(vector) — the old complex/magnitude toggle is gone, complex was the default and is now
the only mode.

- SFCW live display: `App.jsx` `processedSfcwResult`.
- C-scan: `lib/bscanBg.js` `applyBscanBg()`, shared by `processedBscanData` (C-scan +
  2D Map), `sarProcessedData` (SAR), and `alignedSvdData` (Aligned).

**The model path is strictly better for B-scans.** A captured reference is only valid
near the standoff it was taken at, so B-scan positions are corrected by phase-aligning
it with the lidar standoff difference — a fudge that degrades as the hand-held standoff
drifts. A model is evaluated at *each position's own* standoff, so no alignment is
needed and it stays valid across the whole captured span. Outside that span the Akima
interpolator clamps, so the panel flags standoffs beyond the model's `d` range.

The Aligned panel subtracts first and rotates the residual to the common reference
position. That is identical to rotating both and subtracting (the alignment ramp is a
common factor) and it lets the model see each position's true standoff.

**Why groundstation-side:** Pi-side subtraction ran before transmission, so it silently
contaminated B-scan captures, SAR, and BG-model *training* data, which all read
`msg.h_cal_*`. Keeping the wire raw means only the live display is affected.

Note `SfcwDisplay` recomputes its own range profile from `h_cal_real/imag` for
windowing/range-comp, so any subtraction must write back into those fields — replacing
only `magnitudes`/`distances` gets silently discarded. `applyBscanBg` does this.

**Removed from the panel:** the SVD filter (the Aligned, SAR and 2D Map panels
keep their own; `lib/svd.js` stays) and the Wall section. Wall standoff / thickness /
permittivity were never doing refraction work in practice — εr defaulted to 1, so the
distance correction was the identity and the only live effect was capping display depth
at the wall thickness. That became a single `maxDepth` field under Display, used by both
`BscanDisplay` and `sar.worker.js` — and was split apart again on 2026-08-31, because those
two uses had nothing to do with each other: the display clipping is gone and the
reconstruction depth moved to the SAR panel. See the C-scan imaging section below. Export is
v6 (see the C-scan section); import still reads v3 and maps the old `wallThickness` onto
SAR's depth.
Pi-side architecture: bladerf_driver.py (HAL) → sfcw_engine.py (sweep logic) → sdr_server.py (WebSocket).

**SFCW params are pushed groundstation → Pi, never read back.** The engine carries its
own defaults, and `sfcw_set_params` used to be sent only from a panel field's `onChange`,
so a fresh page load left the Pi sweeping at its defaults while the panel displayed and
derived everything (step count, sweep time, max range) from different ones. `App.jsx`
`sendSfcwParams()` now pushes the full set on SDR connect and again before every
`sfcw_start` (all three start paths: both `App.jsx` handlers and the panel's own toggle).
The panel is the source of truth; keep new SFCW params in that payload or they will not
reach the Pi.
Next steps: SAR reconstruction integration.

## Rover Scan Panel + firmware (rewritten 2026-08-29)

Panel id `rover` (`RoverPanel.jsx` + `RoverDisplay.jsx`), between `cscan` and `sar`.
Pi side `pi/rover/rover_server.py` (port 9002), launched by `pi/start.py`. Firmware in
`rover/`. Two axes only -- **X = left/right, Y = up/down** -- there is no standoff axis.

    groundstation --ws:9002--> rover_server --ws:8765--> Arduino UNO R4 WiFi

### The architectural change, and why most of the Pi side got deleted

The first firmware ran its steppers from `loop()` and, so WiFi would not disturb them,
refused to call `webSocket.loop()` while moving. **The board was deaf for the duration of
every move**: it could not be stopped, could not be queried, and silently discarded
anything that arrived mid-move.

Everything the Pi used to do was a workaround for that, and is now gone:
`enforce_move_time`, `ack_grace_s`, `move_overhead_s`, `ACK_GIVEUP`, the jog move-train,
the commanded-vs-confirmed split, `invert_x`/`invert_y`. **Do not reintroduce any of it.**

The firmware now generates steps in a 20 kHz `FspTimer` ISR and the main loop only talks.
Consequences: a held jog key is a real continuous jog, E-stop works during motion, and
position is reported at 20 Hz. `rover/motion_core.h` replaces AccelStepper -- that library
is built around "move to a target", and a jog with no target that must decelerate onto a
soft limit means writing the ramp anyway.

### Position: measured, not dead reckoned

The board reports its own step counter; the Pi converts to mm. One frame end to end --
steps, positive = up/right, origin where the operator last declared it -- so there are no
offsets to keep in sync. Direction sense lives in the firmware (`config.h`
`V_DIR_INVERT`/`H_DIR_INVERT`), not on the Pi.

Remaining error sources, all honest:
- **Quantisation**, bounded at <= half a step (65 um on X, 2.5 um on Y) *however many
  moves have been made*. Verified against the simulator: 400 x 1 mm tracks to +0.35 steps,
  the same as 40 x 1 mm. The old firmware's error grew linearly -- 400 x 1 mm would have
  been +14.7 mm.
- **Calibration**, now the dominant X term. See below.
- **Slip / missed steps**, unobservable. Reported as `travel_mm`, the odometer since the
  last declared position, which is the exposure.

**`ideal_mm` is what makes quantisation bounded, and it is subtle.** It is the commanded
trajectory in mm, kept unrounded; every step target is `round(ideal * steps_per_mm)`.
Computing a relative move as `current_mm + delta` instead does NOT work and reproduces the
original bug exactly: the current position is a whole number of steps, so the rounding
falls the same way every time and compounds. Measured that way: 40 x 1 mm came out +1.469
mm (+3.67%). `ideal_mm` is resynced to reality only on explicit events -- a `done` whose
reason is not `completed`, an E-stop, a jog ending, `set_position`.

**Do not resync it by comparing ideal against actual position.** An earlier attempt did,
on the reasoning that only a cut-short move could put them more than a step apart. It
cannot work: the board acknowledges a move -- advancing the sequence its status stream
reports -- *before* dispatching it from its queue, so every move passes through a window
reporting "new sequence, idle, old position", indistinguishable from a move cut short. The
ideal was destroyed on every move and the 3.67% drift came back untouched.

### The 3.67% rounding bug, and why it was invisible in testing

Old firmware: `lround(mm * 7.7166)` per relative move, remainder discarded. At 1 mm that is
8 steps = 1.0367 mm. The error scales inversely with move size -- 500 mm rounds to within
0.008% -- and the rig had been jogged at a 500 mm step, so **it was correctly reported as
"barely any drift" while being 3.67% wrong at the step size a raster actually uses.** Y is
exactly 200 steps/mm and never had this.

### Bugs found by the simulator, worth not reintroducing

- **`cfg` must never be answered with `hello`.** The Pi pushes its configuration in
  response to a hello, so replying to `cfg` with one is an unbounded cfg -> hello -> cfg
  loop. It saturated the link at **60,727 config pushes in one short test**, and because
  each hello resyncs `ideal_mm` it silently reinstated the quantisation drift. Fixed in
  firmware (cfg and clear_estop reply with `status`, which now carries `pos_valid`) and
  guarded structurally on the Pi by `_link_configured`, so no firmware can induce it.
- **The board's move queue is 4 deep and REJECTS what does not fit.** A rejected move is
  the worst failure available here: `ideal_mm` has already advanced past a move that never
  happened. Measured before flow control existed: 400 rapid 1 mm moves tracked 34 mm short.
  `rover_server` now holds moves in its own `_outbox` and releases them only while the
  board has room, refusing loudly past `OUTBOX_MAX`.
- **`dict.setdefault` evaluates its default eagerly**, so `cmd.setdefault('seq',
  self.next_seq())` burned a sequence number on every jog heartbeat.

### Safety -- there are no endstops on this rig

Soft limits are the only thing between a jog and the end of the rail, so they are enforced
**on the board as well as the Pi**: the Pi can crash or lose its link, the board cannot. A
jog caps its speed at `sqrt(2*a*room)` and coasts onto the limit rather than hitting it.

**Stopping distance is `v^2/2a` and it bit the original tuning.** Both axes shared one
`SPEED = 2000` steps/s, which is 10 mm/s vertically but **259 mm/s horizontally** -- a
129.6 mm stopping distance on a rig whose scans span ~100 mm. Speeds and accelerations are
now per-axis in mm and runtime-configurable; the panel shows the resulting stop distance.
Defaults (vertical 1 m of travel, horizontal 4 m):

| | steps/mm | max speed | jog speed | accel | stop dist max / jog | soft limits |
|---|---|---|---|---|---|---|
| Y vertical, leadscrew | 200.0 exact | 25 mm/s | 15 mm/s | 100 mm/s^2 | 3.1 / 1.1 mm | 150-850 mm |
| X horizontal, 66 mm wheels | 7.7166 | 150 mm/s | 60 mm/s | 500 mm/s^2 | 22.5 / 3.6 mm | 0-3900 mm |

Jog speeds were 5 and 20 mm/s until 2026-08-29 and felt sluggish next to a nudge, which
runs at the much higher *max* speed (a 100 mm nudge reaches it; a 1 mm one does not). Use a
nudge for fine placement and the jog for getting somewhere.

E-stop is software, latched, and works during motion; nothing but `clear_estop` runs while
it is latched. Clearing it marks the position invalid, because cutting the step train at
speed is exactly where a stepper loses steps. A **jog dead-man** lives on the board: the
panel refreshes every 150 ms, the board decelerates after 500 ms of silence, so a dropped
link cannot leave the rover driving. The panel also stops on blur / pointerup /
visibilitychange, since the dead-man costs up to half a second of unwanted travel.

**Standstill whine is the drivers, not the firmware.** A4988-class drivers regulate coil
current by PWM chopping and keep doing so at rest to hold position, which is audible. The
ISR touches no pin while idle (`phase_inc == 0`, and `testSetPosition` asserts no pulses),
so it is not the step train. In order of effect: turn the driver Vref down (also fixes the
holding-current heat), enable idle-disable, or fit TMC2208/2209 for actual silence.

**Idle-disable exists but defaults OFF (`idle_disable_s = 0`).** With no endstop and no
encoder, an axis that creeps while de-energised is silently in the wrong place and nothing
can detect it -- that risk is worse than the noise, so it is opt-in from the panel.
`wakeDrivers()` re-energises and waits `DRIVER_WAKE_MS` before any move, so no steps are
lost coming out of sleep (verified: a 25 mm move after parking landed within 0.011 mm).
Setting the timeout back to 0 re-energises immediately rather than at the next move.

### Verified on the rig, 2026-08-29 (first bring-up of the rewrite)

- **Directions.** Y was correct as shipped; **X was reversed**, so `H_DIR_INVERT` is now
  `true` as well. Both flags were inferred from how the *old* firmware behaved and only one
  survived contact with the rig -- re-check them after any rewiring, by nudging 1 mm and
  watching, not by reasoning.
- **Magnitudes were right first time**, so no wheel calibration was needed: the loaded
  rolling diameter is close enough to the 66.0 mm measured with calipers to be within
  measurement error over the distances tried.
- **A move clamped by a soft limit used to leave the ideal position stranded.** Reported
  from the rig: sitting at 792 with the limit at 850, a +100 nudge correctly stopped at 850,
  but the following -100 went to 792 instead of 750. Cause: the *board* clamps the target
  inside `moveTo()`, so the move then completes normally and reports `completed`, never
  `limit` -- nothing told the Pi the target had been unreachable, and `ideal_mm` stayed at
  892. Fixed by clamping in `move_to_mm` on the Pi, before the ideal is updated, so the
  ideal can never point outside the envelope. The board still clamps as the backstop.
- **A power cycle could leave the board permanently off the network, and only a
  ROUTER restart fixed it.** Cause: `ensureNetwork()` tested `WiFi.status() ==
  WL_CONNECTED` and returned early on that alone. Associating and being on the network
  are different states -- an AP still holding a stale DHCP lease for the board's MAC
  (which is exactly what a power cut leaves behind) will associate it happily and then
  never answer its DHCP request, so `status()` reads `WL_CONNECTED` forever with
  `localIP() == 0.0.0.0`. Every retry path returned early and `ensureSocket()` kept
  restarting a client that could not possibly connect. Restarting the router cleared the
  lease, which is what made the symptom look like a router problem. There is now a
  `linkReady()` (associated **and** holding an address) that all the retry logic tests,
  and a failed DHCP attempt explicitly disconnects so the next one starts clean.
  `USE_STATIC_IP` in `config.h` sidesteps DHCP altogether if it ever recurs.
- **The board does not always reconnect by itself.** Restarting `rover_server.py` left it
  with healthy WiFi (still pingable, still associated) and a socket that never came back,
  needing a power cycle. `ensureSocket()` now tears down and restarts the client after
  `WS_RECONNECT_FORCE_MS` (10 s) of downtime while WiFi is up.
- **Status arrives at ~11 Hz, not the 20 Hz the firmware aims for.** Harmless -- it is
  display smoothness, not control -- but unexplained. Suspect WiFi latency in the R4's
  socket stack rather than the loop, since the loop does almost nothing.

### Calibration

X rolls on wheels, so its steps/mm is empirical -- a caliper gives the free diameter
(66.0 mm, confirmed) but a loaded wheel rolls on slightly less. `rover_calibrate` takes a
commanded and a measured distance and scales. Corrections beyond 0.5x-2x are refused as
a mis-entry rather than applied. Y is a leadscrew at exactly 200 steps/mm and should never
need it.

### The .ino preprocessor: define no types in `rover.ino`

The Arduino IDE auto-generates a prototype for every function in a `.ino` and inserts them
near the TOP of the file, above anything defined further down. A function taking a type
defined in the same `.ino` therefore fails to compile:

    error: 'PersistBlob' does not name a type
     static uint32_t blobCheck(const PersistBlob& b)

`PersistBlob` and `QueuedMove` live in `rover/types.h` for this reason. **Anything
`#include`d is visible before the generated prototypes; anything defined in the sketch is
not.** The same applies to default arguments on sketch-level functions, which the generated
prototype duplicates.

`build_check.sh` cannot catch this by compiling -- compiling the `.ino` as plain C++ skips
the `.ino` preprocessing entirely, which is exactly why this got through the type check and
only failed in the IDE. It greps for the pattern instead and fails the build.

Also note the sketch is `rover/rover.ino`, not `main.ino`: the IDE requires the `.ino`
filename to match its folder, or it offers to relocate the file and leave the headers
behind.

### Testing, given there is no Arduino toolchain on the Pi

The firmware cannot be compiled or run on the machine it is developed from, so:

- **`rover/motion_core.h` and `rover/protocol_core.h` contain no Arduino headers** and are
  compiled natively by `rover/test/test_core.cpp` (125 checks: ramp lands on the exact
  step, limits, watchdog, E-stop, JSON scan/emit). Keep new logic there, not in `rover.ino`.
- **`rover/test/build_check.sh`** runs those tests and then type-checks `rover.ino` as plain
  C++ against stubs in `rover/test/stubs/`. A pass means "will probably compile" -- it
  cannot verify the real libraries' signatures or anything about hardware.
- **`pi/rover/rover_sim.py`** speaks the board protocol over the real socket, so the server
  and panel are testable end to end with no rig. Every bug in the list above was found by
  it. It is a kinematic model of a *perfect* machine: no step timing, no missed steps, no
  slip, so it validates protocol and control flow and never mechanical accuracy.

### Protocol (line JSON, flat objects, sequence-numbered)

Pi -> board: `move` (absolute or relative, per axis), `jog`, `jog_hold`, `stop`, `estop`,
`clear_estop`, `set_pos`, `cfg`, `enable`, `ping`. Board -> Pi: `hello` (connect only),
`status` (20 Hz), `ack`, `done`, `err`. A repeated `seq` is re-acknowledged rather than
re-executed, so retransmission is safe; `jog_hold` is exempt and must not advance `lastSeq`,
which the Pi uses to discard status frames older than a position change.

WiFi credentials live in `rover/secrets.h`, **gitignored**, with `secrets.example.h`
committed. They were previously inline in `rover.ino`.

The automated grid raster is **done, but it lives in the C-Scan panel, not this one**
(2026-08-29) -- see "Rover-driven C-scan raster" below. The Rover panel stays jog, nudge,
calibration and tracking; it is the manual control surface, and driving a scan from it
would have meant a second copy of the grid definition.

## Rover-driven C-scan raster (2026-08-29)

The C-Scan panel has a **Scan Mode** toggle: `manual` (the original hand-held flow,
unchanged) or `rover`, which hands the same grid to the gantry. The rover option is
disabled unless the controller is actually linked (`roverConnected && board_connected`),
not merely when the Pi's rover server is up.

**Only the capture ORDER differs between the two modes.** The sweep, the standoff
provenance, and the background subtraction (captured reference or model, `applyBscanBg`)
are the manual path's code, untouched -- a grid captured either way is the same record and
feeds SAR / 2D Map / export identically.

### Two origins, one cell frame

Manual snakes **up from the bottom-left** (`cellForIndex`, unchanged). Rover snakes **down
from the top-left** (`roverCellForIndex`), because that is the natural way to drive a
gantry. Both write the same `grid_ix` / `grid_iy`, where `iy = 0` is still the bottom row,
so the plan view, the export and every downstream panel are order-agnostic.
`orderedCellForIndex(i, hCount, vCount, scanMode)` picks between them and is what
`CscanDisplay` uses for the dashed path, the START marker and the pulsing next cell.

The capture tag `bscanCaptureRef` is now an **object, not a boolean**: the rover raster has
to say which cell a sweep belongs to, because its capture index is not the manual snake's,
and `cellForIndex(prev.length, ...)` would mislabel every cell. `null` means nothing is
tagged.

### Locating the grid: the operator declares where they are, not where it is

There is no way to point at the wall, so the panel asks for the head's **current position
relative to the grid origin** (the top-left corner) in mm -- "right of origin" and "below
origin". At start the origin is `rover_position - right`, `rover_position + below`, and the
rover drives there in **one `rover_move_abs` on both axes**, so it travels left and up
together. Everything after that is absolute against that anchor, so quantisation cannot
accumulate (see `ideal_mm` above).

**A grid that does not fit inside the soft limits is refused, not clamped.** There are no
endstops, and `move_to_mm` clamps silently while still reporting `completed` -- so a grid
hanging over the end of a rail would raster a rectangle that is not the one on screen, with
duplicate cells at the limit. `gridRoverExtent()` is checked against `config.x/y_min/max_mm`
before a single move is issued, and the panel shows the travel range live so a mis-entered
offset is visible before it matters.

### The state machine (`hooks/useRoverScan.js`)

`homing -> moving -> settling -> capturing -> moving -> ...`, a ref plus a 40 ms interval
rather than a chain of effects -- every transition depends on the rover status stream, on
wall-clock timers and on a sweep landing, and as effect dependencies that re-entered itself
on unrelated re-renders.

**Arrival cannot be detected from "idle" alone.** The board acks a move -- advancing the
sequence its status stream reports -- *before* dispatching it from its queue, so there is a
window reporting "idle, old position" for a move that has not started (the same window that
makes `ideal_mm` unresyncable, above). Arrival therefore needs **all** of: `>= 500 ms` since
issue, `!moving`, `pending_moves == 0`, `queue_depth == 0`, **and** position within 1 mm of
the (Pi-identically-clamped) target. Idle-but-not-there for 3 s is a hard failure -- abort
and E-stop -- rather than a capture at the wrong place. Verified against `rover_sim.py` on a
3x2 grid: every cell reached, worst position error at capture **0.065 mm** (half an X step,
the documented quantisation bound), and the zero-length homing-to-cell-0 move -- the case
the 500 ms gate exists for -- was correctly held rather than passed through instantly.

**The settle delay discards the in-flight sweep.** Sweeps free-run at 3-6 Hz, so the sweep
arriving when the settle expires *started* while the rover was still moving and is smeared
across frequency. `skip: 1` on the capture tag drops exactly one; results are emitted
serially, so the one kept is guaranteed to have started after the settle window closed.
Costs ~250 ms a cell. Settle is a panel field, default **200 ms**.

**Stop Session is an E-stop in rover mode**, deliberately -- it is the only control on
screen while the gantry moves on its own. It latches, so the panel grows a Clear E-Stop
button and says the position is no longer trustworthy (cutting the step train at speed is
exactly where a stepper loses steps).

A raster **resumes**: it starts at `bscanData.length`, so stopping and restarting continues
where it left off rather than re-capturing. The origin is re-derived from the current
position each time, so the offset fields must be re-measured before a resume.

Each rover-captured cell records `rover_x_mm` / `rover_y_mm` (where the gantry actually
reported standing) beside `rover_target_x_mm` / `rover_target_y_mm` (where it was told to
go). Keeping both is the point: slip and missed steps are the only error sources nothing can
observe, so the two must not be assumed equal. Export is **v6**; import still reads v3-v5.
Import deliberately does NOT restore `scanMode` -- it is a live control, not data.

## Imaging Bench Panel — Offline Effect Comparison (2026-08-23)

Panel id `imaging` (`ImagingPanel.jsx` + `ImagingDisplay.jsx` + `lib/imagingEffects.js`),
sitting after `sfcw` in the `PANELS` array. It is **entirely offline**: it reads a
`waterfall_snapshot` JSON exported from the live SFCW waterfall and re-processes it through
a menu of 11 selectable imaging effects, so processing chains can be A/B'd against identical
recorded data without going back to the bench. It never touches the SDR socket.

**All effect math lives in `lib/imagingEffects.js` as pure `(snapshot, params)` functions**
returning plain arrays plus axis metadata. `ImagingDisplay` contains no signal processing —
it memoizes and draws. That split is what makes the effects testable head-first from node
with no React (see the round-trip check below).

### `rawHistory` — the raw complex ring buffer in SfcwDisplay

`waterfallHistory` stores only scalar magnitude rows (dB or linear per `scaleMode`) and is
wiped on every `scaleMode` change, so five of the effects — phase-as-hue, coherence, coherent
integration, dispersion, raw S21 — could not be built from it. `rawHistory` is a parallel
`useRef` buffer with the same `WATERFALL_MAX_ROWS = 100` cap, pushed in the same effect so
row *i* of one lines up with row *i* of the other. Each entry is the sweep untouched by
window / range-comp / averaging / dB conversion:

```
{ real: Float32Array, imag: Float32Array, num_steps, step_size, range_offset,
  start_freq, stop_freq, timestamp, phase_coherence }
```

Source is `sfcwResult.h_cal_real/h_cal_imag` via `hCalRef` (which now also caches
`start_freq` / `stop_freq` / `timestamp` / `phase_coherence`). It is cleared **only** on
unmount, never on a `scaleMode` flip — raw sweeps are unit-agnostic so there is nothing to
invalidate, and that is the one case where the two buffers can differ in length.

A `rawCount` state mirror exists purely so the EXPORT button can enable/disable itself; the
buffer is never read through React. The existing live render path is unchanged.

### `waterfall_<ts>.json` v1 format

Written by the neutral `EXPORT` button in the waterfall pane (`bottom-10 left-14`, inside the
waterfall's own relative container, so it sits alongside — not over — the range profile's
dB/LIN toggle). Gated on `!hideWaterfall`, so only the SFCW panel's instance has it; the
C-scan and BG-model instances do not. Disabled and dimmed when the buffer is empty.

```json
{
  "version": 1,
  "type": "waterfall_snapshot",
  "timestamp": "<ISO>",
  "common": { "num_steps": 51, "step_size": 60000000, "start_freq": 2000000000,
              "stop_freq": 5000000000, "range_offset": 0.5 },
  "displayState": { "scaleMode": "linear", "windowType": "rectangular",
                    "kaiserBeta": 3, "rangeComp": 0, "avgCount": 1 },
  "sweeps": [ { "t": 1755900000.12, "real": [], "imag": [],
                "phase_coherence": { "phase_std_rad": 0.11, "coherent": true } } ]
}
```

`sweeps` is oldest-first; `real`/`imag` are rounded to 8 decimals like the Pi does. ~124 KB
for 100 sweeps × 51 steps. `displayState` is **provenance only** — `App.jsx`
`handleLoadImagingSnapshot()` uses it to seed the bench's "None" mode and the shared
range-profile knobs so the bench opens on the image the operator was looking at, and it is
applied to nothing else.

**`sfcw_result` now carries `start_freq` / `stop_freq`** (`sfcw_engine.py` `_process_h_cal`).
This is the only Pi-side change the panel required. `stop_freq` is the *last frequency
actually visited* (`start + (num_steps-1)*step`), which equals `self.stop_freq` only when the
step divides the span evenly. Dispersion and raw-S21 need the real RF axis and deriving it
from `step_size` alone is guesswork. `snapshotFreqs()` falls back to step index for
pre-`start_freq` snapshots and `freqsKnown()` flags it; the panel says so in the readout.

### The 11 effects

| # | id | What it computes |
|---|---|---|
| 0 | `none` | Reference image — identical processing to the live waterfall |
| 1 | `compression` | `(\|H\|/peak)^p`, a continuous dial where dB and linear are two points |
| 2 | `percentile` | Colour limits from percentiles, whole-history or per-row |
| 3 | `binnorm` | Per-bin temporal normalisation — adaptive clutter map, no capture, no model |
| 4 | `cfar` | Signal / CFAR threshold in dB, so 0 dB is the detection threshold |
| 5 | `colormap` | Same image under all five maps side by side |
| 6 | `phasehue` | Hue = phase of the complex profile, value = magnitude |
| 7 | `coherence` | Normalised complex correlation at lag L over a sliding window |
| 8 | `integration` | Coherent vs non-coherent averaging, and their ratio |
| 9 | `dispersion` | Sub-band sweep — range across, sub-band centre frequency up |
| 10 | `s21` | Calibrated `h_cal` against frequency, before any IFFT |

Notes on the ones with non-obvious choices:

- **Effects 3, 7, 8 need multiple sweeps.** They return `{kind:'message'}` on a one-sweep
  snapshot and the dropdown disables them, rather than rendering garbage.
- **Effect 8 integrates in the range domain, not on `h_cal`.** Averaging complex `h_cal` over
  K sweeps and then transforming is *identical* to averaging the complex range profiles (the
  IFFT is linear), and the non-coherent partner — a mean of magnitudes — only means anything
  in the range domain. Averaging `|h_cal|` in frequency and then transforming would be
  nonsense. Side-by-side gives coherent and non-coherent one shared colour scale, which is
  the whole comparison; the ratio pane is a relative quantity in different units so it
  carries its own scale, marked `OWN SCALE` in amber.
- **Effect 9's sub-band count is capped by width and overlap.** `hop = subWidth*(1-overlap)`,
  so `maxCount = floor((numSteps-subWidth)/hop)+1`; the count slider is clamped to that and
  the canvas says so when it bites. The default `overlap` is **0.6**, which is where the
  default 8 sub-bands actually fit across a 51-step sweep — at 0.5 only 6 do. A sub-band
  starting at a non-zero step does not shift range (range is set by the *rate* of phase
  change with frequency, not the offset), so all sub-bands share one range axis.
- **Effect 10's residual mode is a direct corrupted-sweep detector** and is the reason it
  exists — see the `settle_count` regression history above. A sweep is flagged red when
  `max(computed_std, phase_coherence.phase_std_rad) > 0.3 rad`, matching the Pi's own cut.
  `real & imag` has no single scalar to colour a waterfall with, so that combination stays a
  line plot regardless of the display radio, and says so.
- **CFAR and the window functions were lifted out of `SfcwDisplay` into
  `imagingEffects.js`**, so both panels now call one implementation; CFAR gained GO/SO
  variants (GO holds the threshold up on the far side of the wall return, where CA lets a
  clutter edge drag it down). `computeCFAR` accumulates its CA sum in a side-then-k order
  that looks redundant next to the per-half accumulators the GO/SO variants need — **do not
  "simplify" it into `(loSum + hiSum) / (loCount + hiCount)`.** Float addition is not
  associative and that rewrite shifts the threshold by ~3e-14 dB, which is what the current
  form deliberately avoids: the live display's output is bit-identical to what it produced
  before the lift, verified across window lengths 51–256, Kaiser β 2–14 and five CFAR
  parameter sets.
- **CFAR runs on the full profile and clips afterwards**, so the range-zoom edges do not get
  a one-sided training window.
- **Range compensation is folded into the complex profile** as an amplitude gain of
  `r^(n/2)`, which is exactly the `+ n*10*log10(r)` dB the live display applies — doing it in
  `prepare()` keeps magnitude and phase consistent for the complex effects.

### Structure and cost

`prepare(snapshot, profile)` does the windowing and zero-padded IFFTs once and is memoized on
`[snapshot, params.profile]`; every range-domain effect reads its output, so switching effects
or dragging an effect slider never redoes them. Measured on 100 sweeps × 51 steps: `prepare`
7 ms, every effect ≤ 9 ms, worst case (sliding median, K=50, zero-pad ×8) 32 ms. No manual
Apply button is needed and none exists — every parameter updates the render immediately.

The View section's range zoom is applied **before** colour limits are computed, so percentiles
and dynamic scaling describe what is actually on screen. The colormap choice is global: it
persists as the active map across every effect, not just while entry 5 is selected.

`ImagingDisplay` draws via an offscreen `nx × ny` canvas + `putImageData` + one scaled
`drawImage`, not per-cell `fillRect` — at 100 × 1024 bins the latter is tens of thousands of
fills per frame. Non-finite cells (short coherence windows, masked bins) render as a dark grey
no colormap produces, so they are never mistaken for data.

### Verification

Effect math was checked head-first from node against a synthetic two-target scene: peak bins
land within one bin of the true range (0.22 / 0.60 / 1.00 m → 0.2196 / 0.6002 / 1.0003 m at
zero-pad ×8, 4.9 mm bins). The full export → import → validate → render chain was exercised
with the verbatim export payload, and all 11 effects were rendered in a real browser to
confirm the canvas output. There is no test runner in this repo, so those checks were
throwaway scripts rather than committed tests — worth rebuilding as real tests if
`imagingEffects.js` grows.

## Sweep Timing (measured 2026-08-20)

**Measured sweep times** at 151 steps (20 MHz spacing, 2–5 GHz):
- Mean: 548 ms, effective rate 1.82 Hz.
- Per-step time: 3.63 ms (10 settle buffers × 4096 samples / 2 Msps = 20.5 ms settle
  + 1 capture buffer, but the real wall time per step is 3.63 ms because RX callbacks
  overlap — the settle wait is for *new* callbacks arriving, not elapsed time).

The per-step wait is `settle_count` RX buffer callbacks in `_sweep_core`, now a
user-controlled `SFCWEngine` param (default 10, exposed in the panel as "Settle",
same param family as `num_buffers`/"Buffers") rather than hardcoded. Sweep RX buffers
are 4096 samples at the 10 Msps set in `_configure_hardware`, i.e. 0.41 ms per buffer —
`BUFFER_SAMPLES` / `SAMPLE_RATE` in `SfcwPanel.jsx` mirror those two numbers and must
track the engine. `num_buffers` genuinely averages that many post-settle captures per
step now (see Quick-tune master table below) — the panel's per-step estimate is
`(settle_count + num_buffers) * 0.41ms`.

**Regression, 2026-08-20 to 2026-08-23 (fixed): do not drop `settle_count` below 10
without a real per-step validation.** An optimization pass (`407e205`, `510a9fe`) cut
the quick-tune `settle_count` from 10 to 7, gated behind an experimental
`sweep_mode='fast'` flag with an explicit "reduced if Test C proves it safe" caveat —
then the very next commit merged it in as the unconditional default and rewrote the
caveat into an unsubstantiated "validated over 50 sweeps" claim, with no test artifact
in the repo. Symptom: intermittent fully-garbled sweeps (good scans mostly,
occasionally one random-looking sweep, rarely two in a row) — one step retuning late
means its capture still holds the previous frequency's IQ, and since the range profile
is one IFFT across all steps, a single bad bin corrupts the whole sweep rather than
just that bin. Default reverted to 10. If it ever needs to drop again, validate with a
per-step check (flag/log which step index was corrupted), not just an aggregate
correlation over whole sweeps — an aggregate metric is exactly what let this ship
unnoticed. `benchmark_sweep.py` is a leftover from that pass and is currently broken
(references `_sweep_core_fast`/`sweep_mode`/`_qt_profiles_rx`, all since removed, and
unpacks `_sweep_core` as a 2-tuple when it has returned `(h_cal, dropped_steps, adc_peak)`
since 2026-08-29) — needs a rewrite against the current `_sweep_core`/master-table API
before it's useful again.

**The same 2-tuple slip was live in the engine itself until 2026-08-31.**
`_sweep_core`'s early `if stop_event.is_set(): return None, 0` was missed when
`adc_peak` was added, so **every stop mid-sweep** raised
`ValueError: not enough values to unpack (expected 3, got 2)` in `_perform_sweep` /
`_perform_sweep_raw`. It presented as a harmless recurring
`[sfcw] Sweep error: not enough values to unpack (expected 3, got 2)` because
`_sweep_loop`'s broad `except` falls straight into a `finally` that stops TX/RX —
which is what stopping was about to do anyway — so the shutdown still happened, just
via the exception path. It was not harmless: it pushed a bogus `{'error': ...}` to
the groundstation on every stop, and it trained the operator to ignore the one line
a genuine sweep failure would print. Now returns `None, 0, None`; every consumer
already guards a null `adc_peak` (`_warn_if_adc_hot`'s `if not adc_peak`). **If a
fourth value is ever added to `_sweep_core`, that early return is the one to
remember** — it is the only path that does not fall through to the bottom.

**Regression, 2026-08-20 to 2026-08-23 (fixed): `num_buffers` default silently dropped
from 4 to 1, killing per-step noise averaging.** The `c33b0ce` "clean up" commit (same
day as the `settle_count` regression above) trimmed `sfcwParams`/`SFCWEngine` defaults
and dropped `numBuffers` from 4 to 1 — with no discussion, apparently just collateral
from tidying the defaults block. It went unnoticed at the time because the multi-buffer
averaging in `_sweep_core` was *itself* separately broken by the `407e205`/`510a9fe`
optimization pass: `num_buffers` only extended the settle wait but the code always
grabbed the single latest RX buffer regardless of its value, so for a few days the
setting had no effect at any value. `f98e208` (2026-08-23) fixed the averaging to
actually capture and mean `num_buffers` fresh buffers per step — but the default was
already 1, so the fix's benefit stayed invisible (1 buffer averaged with itself is a
no-op) until the default was corrected back. Symptom: sweep-to-sweep correlation stays
high (scene/multipath structure is unchanged) but per-sweep amplitude/phase noise is
visibly higher than before, burying fainter returns — because each step went from
averaging 4 captures (~6 dB of free SNR, `10*log10(4)`) down to 1. Confirmed live on
2026-08-23: 15-sweep static-scene comparison via the running `sdr_server`, mean
complex-domain deviation between sweeps was 0.0055 at `num_buffers=1` vs 0.0034 at
`num_buffers=4` (~39% reduction). Default restored to 4 in both `App.jsx` and
`SFCWEngine.__init__`. If `num_buffers` is ever dropped for speed again, check the
*current* live-sweep wobble against a static scene first, not just correlation —
correlation is insensitive to this because it doesn't wreck sweep structure, only
buries weak signal in noise.

## FPGA tuning mode kills the RX stream on the bladeRF 2.0 — do not re-enable (2026-08-28)

**Symptom:** RF Calib panel works fine, but starting an SFCW sweep gives
`[ERROR @ .../libusb.c:1089] Transfer timed out for RX buffer ...` +
`[bladerf] RX dual error: Operation timed out`, and no sweep is produced at all.

**Cause:** `SFCWEngine._configure_hardware()` called `driver.set_tuning_mode_fpga()`
(`bladerf_set_tuning_mode(BLADERF_TUNING_MODE_FPGA)`). On the bladeRF 2.0 micro that call
*succeeds* (`rc=0`, prints "Tuning mode set to FPGA") and then silently breaks the RX_X2
data path: `sync_rx()` starts throwing `TimeoutError` about 8 buffers later. Because
`_rx_loop_dual` catches the exception and exits, `_rx_seq` stops advancing, so every step of
`_sweep_core` falls through its `rx_cond.wait(timeout=1.0)` and the sweep returns nothing.

**Bisected on hardware** (device free, 10 Msps, RX_X2, same `sync_config` the driver uses):

| test | result |
|---|---|
| stream only | OK, 60/60 buffers |
| `set_tuning_mode(FPGA)` only | **FAILS after 8 buffers** |
| quick-tune master table only (151 profiles, no FPGA tuning) | OK, 60/60 buffers |
| both | **FAILS after 8 buffers** |

So the quick-tune table is innocent — it was *only* the tuning mode. Note the failure happens
before any `bladerf_schedule_retune()` call, so it is not a retune problem either.

**Not an FPGA-image problem.** Reproduced identically with the flashed image (0.16.0, "configured
from SPI flash") *and* with Nuand's official `v0.16.0/hostedxA9.rbf` downloaded and loaded into
RAM (`bladeRF-cli -l`, reports "configured by USB host"). Reflashing does not help — don't.
Test FPGA images with `-l` (RAM, reverts on power cycle), not `-L` (SPI flash), when
diagnosing; it is free to undo.

**Why it was never going to work:** libbladeRF's own `default_tuning_mode()`
(`host/libraries/libbladeRF/src/board/bladerf2/common.c`) opens with an unconditional
`mode = BLADERF_TUNING_MODE_HOST;`, and the `if (BLADERF_TUNING_MODE_FPGA == mode && ...)`
errata check immediately after it is dead code that can never run. FPGA tuning on bladerf2 is
reachable *only* via `BLADERF_DEFAULT_TUNING_MODE=fpga`, and the errata text it guards refers
to "errata related to FPGA-based tuning". Nuand does not default this board to FPGA tuning.

**Fix shipped:** the `set_tuning_mode_fpga()` call is removed from `_configure_hardware()`
(the comment there explains why — keep it). `driver.set_tuning_mode_fpga()` itself is left in
`bladerf_driver.py` but is now uncalled; `_fpga_tuning` was a write-only flag nothing ever
read, and is now just set False.

**It costs nothing.** Quick-tune still works in host tuning mode: `bladerf_schedule_retune()`
returns `rc=0` for every step, and a 51-step sweep measured **230 ms (4.35 Hz)**, inside the
3–6 Hz band this panel has always run at. Verified end-to-end through `SFCWEngine` itself:
3/3 sweeps, 51 steps each, no errors, sweep-to-sweep correlation **0.9984**.

**Caveat left open:** those verification sweeps ran with the antennas pointed at open room, and
reported `phase_std ≈ 1.44 rad` -> `coherent=False` (the Pi's cut is 0.3). Sweep-to-sweep
correlation of 0.9984 says this is repeatable structure, not the retune-timing corruption that
check exists to catch (corrupted sweeps do not repeat). Still worth re-confirming against a
known target that the range profile looks right.

## Quick-tune master table (2026-08-23)

Per-grid quick-tune profile caching is gone. `SFCWEngine._ensure_master_quick_tune_table()`
generates one fixed table spanning `QT_MASTER_START_FREQ`–`QT_MASTER_STOP_FREQ` (2–5 GHz)
at `QT_MASTER_STEP` (20 MHz) once per device connection — 151 profiles, paying the full
per-frequency VCO-cal cost (`bladerf_set_frequency` + `bladerf_get_quick_tune`) only that
once, ~6s total. `set_params()` snaps `start_freq`/`stop_freq` to the nearest 20 MHz and
clamps them into that range, and snaps `step_size` to a 20 MHz multiple (`_snap_freq`/
`_snap_step`), so every sweep's frequencies are guaranteed to land exactly on master grid
points. `_build_sweep_grid()` then just indexes into the cached table — no regeneration,
no device reset — so start/stop/step can change freely mid-session, live, with no
interruption.

This replaced the old scheme: profiles were cached per-`(start, stop, step)` combo, and
changing any of those three flipped `_freq_grid_dirty`, which forced a full
`driver.reset()` + reconfigure + restream on the next sweep (or mid-sweep, via
`_reconfigure_for_new_grid()`). That reset path was unreliable in practice — bladeRF
errors on the reopen — which is why it's gone rather than fixed. The master table only
needs invalidating (`SFCWEngine.invalidate_quick_tune_table()`) after an explicit
`device_reset` from the panel; `sdr_server.py`'s `device_reset` handler calls it.

**The table is the UNION of a 20 MHz and a 50 MHz grid (2026-08-31), not a single
uniform grid.** 20 alone could not represent a 50 MHz step: `set_params` snapped
50 -> 40 and the panel then described a sweep that was not the one running (61 steps
and 1.0 m of range against the 76 steps and 1.37 m actually swept, with nothing on
screen saying so). Cost against the 256 ceiling: 20 MHz -> 151 points, 50 MHz -> 61,
overlap (multiples of 100) -> 31, **union 181, leaving 75 spare**. Adding a third
family is not free -- check the union against the cap first.

Two consequences that are easy to get wrong:

- **A single sweep must stay inside ONE base family.** Mixing is not safe: starting
  at 2020 (on the 20 grid) and stepping 50 visits 2070, which is on *neither*
  family and is not in the table at all. `_snap_sweep()` therefore picks one base --
  whichever represents the requested STEP most closely, ties to the finest -- and
  snaps start, stop **and** step to multiples of it, which makes every visited
  frequency a multiple of that base and so present by construction. Because the
  base depends on the step, the three cannot be snapped independently;
  `_apply_freq_grid()` re-snaps all three from the values that were *requested*
  (`_req_start/_req_stop/_req_step`), so the result does not depend on the order
  the panel sets them in and does not drift when re-snapped.
- **`_build_sweep_grid` now looks up BY FREQUENCY, not by arithmetic.** It used to
  compute `start_idx + i * (step / QT_MASTER_STEP)`, which is only valid on a
  uniform table. Against the union grid that silently addresses the wrong
  profiles -- retuning each step to some other frequency while reporting the one
  asked for, the same class of failure the profile-cap check exists to prevent. It
  is `np.searchsorted` plus an **exact-match assertion that raises**: a frequency
  not in the table fails loudly rather than retuning to its neighbour.

**Rounding is half-up on both sides, deliberately.** Python's `round()` is
banker's and JavaScript's `Math.round` is half-up, so the two disagreed on exactly
the `.5` cases (50/20 -> 2 on the Pi, 3 in the browser). `_round_half_up()` in
`sfcw_engine.py` makes them identical; `master_grid_freqs()` and `_snap_sweep()`
are pure and can be checked without hardware.

**The groundstation mirrors all of this in `lib/sfcwGrid.js`.** The Pi never
reports its parameters back -- the panel is the source of truth and pushes -- but
`set_params` silently snaps, so `SfcwPanel` was computing R max, step count and
sweep time from what was typed rather than what would run. It now snaps the whole
triple on commit (`commitSweep`, mirroring `_apply_freq_grid`) so the field shows
what will actually run, and derives every readout from the snapped values as a
backstop. **Keep `sfcwGrid.js` in sync with `QT_MASTER_STEPS` / `_snap_sweep` /
`_round_half_up`** -- verified by generating 2500 cases from the Python and
asserting the JS reproduces all three outputs exactly, plus that every snapped
sweep lands entirely on the union grid.

**Hard ceiling: `MAX_QUICK_TUNE_PROFILES = 256`, do not exceed it.** The first version of
this table tried 1–6 GHz at 10 MHz spacing (501 profiles) and it was broken: verified
against libbladeRF's own source on the Pi
(`~/bladerf-src/host/libraries/libbladeRF/src/board/bladerf2/bladerf2.c:1419-1513`),
`bladerf_get_quick_tune()` is not a stateless read — every call *writes* a new fastlock
profile into a fixed-size on-device table (`board_data->quick_tune_tx/rx_profile`, capped
at `NUM_BBP_FASTLOCK_PROFILES = 256` in `fpga_common/include/bladerf2_common.h`, one shared
counter per direction across both TX/RX sub-channels). That counter only resets on a full
`bladerf_open()`. Past 256 calls it returns `BLADERF_ERR_UNEXPECTED` and leaves the profile
struct unpopulated — the original code didn't check the return code, so it silently stored
zeroed/garbage profiles for every frequency past the 256th, which `bladerf_schedule_retune()`
would then happily retune to the wrong RF state. Symptom on stdout: a wall of
`[ERROR @ .../bladerf2.c:1427/1456] Reached maximum number of TX/RX quick tune profiles.`
repeated once per frequency past the cap, on every `start.py` run. `_ensure_master_quick_tune_table()`
now raises immediately if `len(freqs) > MAX_QUICK_TUNE_PROFILES` (compile-time check) or if
`bladerf_get_quick_tune()` ever returns nonzero (runtime check) — fail loud, never store an
unchecked profile. 2–5 GHz at 20 MHz is 151 profiles, comfortably under 256.

Consequence: the sweep range is hard-bounded to 2–5 GHz (panel Start/Stop min/max 2000/5000
MHz) — anything requested outside that gets clamped, and step size floors at 20 MHz. Steps
that are a multiple of 20 or of 50 MHz are exact; anything else snaps to the nearest of
those (30→40, 45→40, 55→60, 90→100). Widening
either means trading against the 256-profile ceiling (span_MHz / step_MHz + 1 ≤ 256) — there's
no way to have both a wide range and fine resolution simultaneously on this hardware without a
different strategy (e.g. a lazy per-frequency cache with a reset-triggered eviction, discussed
and deferred 2026-08-23 in favor of just picking a range/step that fits).

**Default step size is 60 MHz (51 steps, 2–5 GHz)** — `sfcwParams.stepSize` in
`App.jsx` and `SFCWEngine.step_size` both carry it, and the groundstation pushes its
value to the Pi on connect (see the param-push note above).

## C-Scan Panel — 2D Raster (replaced the B-scan panel, 2026-08-20)

The B-scan panel is gone; `CscanPanel.jsx` + `CscanDisplay.jsx` + `lib/cscanGrid.js`
replace it. Panel id is `cscan` (was `bscan`). App-level state keeps its `bscan*` names
(`bscanData`, `bscanParams`, …) because the underlying record is still one B-scan trace
per position — only the panel and its geometry changed.

**Grid.** `bscanParams` is now `{ hCount, hStep, vCount, vStep, gateStart, gateEnd,
metric }` (it carried a `maxDepth` until 2026-08-31; see the C-scan imaging section). `stepSize` / `numPositions` are gone; SAR and the 2D Map are 1D and
read `stepSize: hStep` (injected in `sarParams` / `mapStepSize`) with the position count
taken from the data length as before. The Scan Grid section sits between Session and
Capture so the rectangle is described before any sweep is tagged.

**Snake raster order** (`lib/cscanGrid.js` `cellForIndex`). Capture starts at the
bottom-left cell, sweeps the bottom row left→right, steps up one row, sweeps right→left,
steps up, and repeats. Verified: a 3×2 grid captures (0,0) (4,0) (8,0) (8,6) (4,6) (0,6)
for hStep 4 / vStep 6. Every position stores `grid_ix`, `grid_iy`, `x_cm`, `y_cm`,
resolved from the capture index at capture time in the `sfcw_result` handler (via
`bscanParamsRef`), so editing the grid afterwards never relabels existing cells. Lidar
standoff is captured per cell exactly as before.

**Display.** The viewport is Live Sweep (top) over C-Scan Grid (left) + the selected
row's B-scan (right). The grid is a plan view holding the physical aspect ratio, colour =
`gatedIntensity()` over the depth gate (peak / energy / mean), drawn live as cells fill.
Uncaptured cells are outlined and empty; a cell captured with no range bin inside the gate
is mid-grey, distinct from uncaptured. The next target pulses cyan, the snake path is
dashed over the captured cells, and clicking a cell picks the row shown in the B-scan pane
(that pane sorts the row by `grid_ix`, so a right→left row still reads left→right).

**Colour scaling** is one `{ dynamic, min, max }` object shared by both panes. Dynamic
tracks the captured cells; switching to manual seeds the sliders from the current dynamic
limits so colours do not jump, then the two dB sliders drive both images live. The sliders
are disabled and dimmed while dynamic is on, and the colour bar turns amber and reads
MANUAL when it is off.

**Export is v6** (`cscan_<ts>.json`): grid params (now including `scanMode` and the rover
origin offsets as provenance) plus per-position `grid_ix` / `grid_iy` / `x_cm` / `y_cm`, and
for rover-driven cells the reported and commanded rover position. Import accepts v3–v6; a v4
(or earlier) linear scan maps onto a one-row grid (`hCount = numPositions`,
`hStep = stepSize`, `vCount = 1`). `scanMode` is deliberately not restored on import.

**Capture order is mode-dependent** (2026-08-29): by hand it snakes up from the bottom-left
as described above; under the rover it snakes down from the top-left. Cell indices are the
same either way — only the order the cells are visited differs. See "Rover-driven C-scan
raster".

**Known limitation:** SAR and the 2D Map still treat the capture sequence as a single line.
With `vCount = 1` that is exactly the old behaviour; with more rows their input is a
zig-zag path and the reconstruction is not meaningful until they are made grid-aware.

## BG Model — Capture Protocol and Findings

**Capture protocol (as of 2026-08-18).** One capture = N sweeps at a **static** standoff
(N configurable in the BG Model panel, default 40, persisted to `localStorage.bgmodel_sweeps`).
Positions are hand-placed and deliberately **irregular**; irregular beats uniform, because
uniform undersampling folds alias energy coherently onto a single wrong spatial frequency
while irregular spacing scatters it. Target: ~30 positions over the widest span the bench
allows (150 mm+).

`bgCaptureStats.computeCaptureStats()` runs at capture completion and stores, per position:
coherent complex mean (`h_mean_real/imag` — this is the training target), per-frequency noise
variance, per-sweep and post-averaging SNR, sweep-pair correlation, standoff mean/std, and
`radarRangeM` (range of the dominant return from the coherent mean, sub-bin interpolated).
`radarRangeM` is **diagnostic only** — nothing consumes it. It exists so the dataset carries an
independent standoff estimate to check the lidar against.

Training now uses **one sample per position** (the coherent mean), not every raw sweep. Replicas
measure the same standoff repeatedly, so feeding them individually adds no information — MSE
regresses to this mean anyway, at N× the epochs.

**Spacing limits** (`spacingLimits()`). An echo with path multiplier α oscillates in standoff
with period `c / (2·f·(α−1))`; worst case is α=3 (triple bounce) at the top of the band. At
5 GHz that period is 15 mm, so:
- ≤ 5 mm gaps — well sampled
- 5–7.5 mm — coarse but unaliased
- Above 7.5 mm — α=3 folds onto a wrong spatial frequency and *corrupts* a fit rather than missing detail

**Span sets echo resolution:** `Δα = c / (2·f_c·span)`. At 3.5 GHz, 85 mm span → Δα = 0.50;
150 mm → 0.29. Widest possible span is the single biggest accuracy lever.

**Export format v2** (`bgmodel_<N>pos_<ts>.json`): hoists `common` (num_steps, step_size,
range_offset) out of the per-sweep repetition and stores per-capture `stats` + column arrays
(`standoffs`, `real`, `imag`). ~7 MB for 30 positions × 40 sweeps. Import accepts v1 and v2 and
backfills stats when absent.

**Analysis of the existing MLP** (1 → 64 → 64 → 302 ReLU, 23,918 params, `bgmodel.worker.js`):
- Output is effectively **rank ~5** — SVD over the input domain puts 96% of energy in 5 PCs,
  in near-equal quadrature pairs (the signature of complex sinusoids in `d`). 19,328 of its
  parameters describe a rank-5 map.
- Only 13/64 first-layer knots land inside the input domain; 32/64 L1 and 17/64 L2 units are
  dead across the whole domain.
- Per-frequency residual magnitude spans **20 dB**, so pooled scalar target normalization makes
  MSE a silently power-weighted loss that starves the weak bins.
- `finalLoss` is training MSE with no held-out split anywhere. Param:data ratio was 0.32:1.
- Inference is 19.3 µs (~52k/s), ~1700× headroom at 30 fps. Sweeps run at 3–6 Hz, so the model
  is nowhere near the bottleneck — **data, not compute, is the constraint.**

**Lidar precision is the hard ceiling.** Two-way phase is `4πfd/c`, so at 5 GHz **1 mm of
standoff error = 12° of phase error**. 20 dB of coherent suppression needs the standoff to
~0.5 mm; the TF-LC02 is a ±few-mm sensor. Comparing `radarRangeM` against `standoffMm` across
the new dataset is the cheap test of whether a radar-derived standoff beats the lidar.

**Result on the 30-position bench set (2026-08-18) — the MLP was replaced.**
Leave-one-position-out suppression, `10*log10(signal/error)` on each held-out position's
measured spectrum, 30 positions over 155.8 mm, median gap 5.5 mm, 15 sweeps each:

| estimator | LOO suppression |
|---|---|
| **Akima interpolation, unwind α=0.80** (shipped) | **20.2 dB** (median 20.3, worst 4.0) |
| cubic spline, α=0.80 | 20.3 dB — best mean, but −12.3 dB on a bad knot |
| physics model, K=5 echoes, free A(f) | 18.7 dB |
| linear interpolation | 12.4 dB |
| Fourier-feature MLP (tanh, k=8..64) | 11.9 dB |
| nearest position | 7.4 dB |
| physics model, K=3, Chebyshev A(f) | 6.8 dB |
| **old 1-64-64-302 MLP** | **4.9 dB** |
| global mean | 0.9 dB |

The captures are dense relative to how fast the background varies, so interpolation wins
outright and needs no parameters. `bgModelInterp.js` ships Akima: it gives up 0.6 dB of mean
for a 16 dB better worst case, because it does not propagate a bad capture into neighbouring
intervals. Inference is 3.1 µs (320k/s, ~10,000× headroom at 30 fps); model file ~360 KB.
Models are `type: 'interp'`; `bgModelInfer.js` keeps the MLP path for previously saved files.

**Things that turned out differently than expected:**
- **The old MLP was underfitting, not overfitting.** 2000 full-batch epochs reached only
  7.97 dB in-sample; 10k → 12.0 dB, 40k → 13.3 dB. Its `finalLoss` looked small only because
  targets were normalized by a single pooled scalar. Its loss curve was still falling 7.4%
  per 100 epochs at epoch 1999. Verified by scoring the saved `models/model 3.json` weights
  directly: 8.6 dB in-sample, matching the numpy re-implementation used for the LOO sweep.
- **The unwind is better with α ≈ 0.80 than α = 1.0** (20.5 dB vs 19.1 dB), a broad plateau
  over 0.70–0.85. Unwinding removes fast phase but injects the lidar's own error into the
  target; 0.8 is the trade-off point. The α matched filter puts the wall echo at 0.93–0.95,
  consistent with the lidar over-reporting standoff *change* by 5–20%. Worth a calibration
  check, but not required — the interpolator absorbs it.
- **The `radarRangeM` diagnostic is unusable**: correlation 0.36 with lidar standoff, 39.9 mm
  scatter after a linear fit. Dominant-peak picking hops between echoes in the near field.
  Radar-derived standoff is not a usable input; the near-field skepticism was correct.
- **Echo structure is dominated by two components**: α≈0.0 (static cable/coupling reflection,
  strongest) and α≈0.93 (wall face, −3.4 dB). Everything else is ≥13 dB down. But a smooth
  (Chebyshev) `A_k(f)` caps the physics model at ~7 dB; with `A_k(f)` free per frequency it
  reaches 18.7 dB. The unmodelled echoes get absorbed into `A_k(f)` as fast frequency
  structure, so `A_k(f)` is *not* smooth.
- **A physics + spline hybrid gives exactly no gain** over the spline alone (20.60 vs 20.59 dB).
- **Measurement noise is not the limit.** The 15-sweep coherent mean sits 47.1 dB below signal.
  Position density is the limit.

**Position density is the dominant lever** (cleaned 22-position set, subsampled):

| median gap | LOO suppression |
|---|---|
| 5.9 mm | 19.3 dB |
| 11.8 mm | 6.9 dB |
| 17.7 mm | −3.3 dB |

Roughly 12 dB lost per doubling of gap. Capture as densely as patience allows; this matters
far more than any modelling choice.

**Range gating cannot separate in-wall targets from the wall face at this bandwidth.** The
entire background sits within 2–6 cm of range, and 3 GHz of bandwidth gives ~50 mm range
resolution. The "gated" metric therefore tracks the full-band metric closely. Separating a
target from the face needs more bandwidth or aperture, not better background subtraction.

## Background subtraction: standoff instrumentation and the false-target hunt (2026-08-28)

Investigating false targets (spurious returns where there is nothing) from the Akima
interpolating background model (`lib/bgModelInterp.js`), which scores 20.2 dB
leave-one-position-out on the bench but misbehaves live.

### `LIDAR_ANTENNA_OFFSET_MM` was 315 mm and that is wrong for this mounting — now 160 mm, and user-editable

**Measured on the bench 2026-08-28:** with the antenna aperture at the wall (true zero
standoff) the TF-LC02 reads **164.83 mm ± 0.68**. So the lidar→antenna offset is ~165 mm,
not 315 mm. The default is now **160 mm** (measured − 5 mm, so a real zero-standoff pose
reports slightly positive rather than negative), it lives in App.jsx state persisted to
`localStorage.lidar_antenna_offset_mm`, and it is editable in the SFCW panel's Standoff
section. **It is a per-mounting quantity — re-measure it after any re-mount** by putting
the aperture against the wall and reading the lidar.

Consequences worth understanding, because they are not all the same:
- A *constant* offset error **cancels exactly** for a model trained and used under that
  same offset: the unwind factor is common to every knot, factors through the linear
  interpolation, and is undone by the rewind. So the wrong 315 did not, by itself, break a
  freshly-trained model — which is consistent with the reported observation that fresh
  models fail the same way. Do not expect fixing the offset alone to fix false targets.
- What it *did* break is every standoff number being unphysical (150 mm behind the actual
  aperture), and any model built under a *different* offset being silently mis-indexed.
  `models/4th model.json` has knots spanning 11–167 mm; under the 315 offset the operating
  standoff is `lidar − 315`, and since the lidar reads ~165–315 mm in real use, that is
  −150…0 mm — **entirely below the model's span, so it clamped on every single sweep**,
  subtracting a background measured somewhere the rig never goes. That is a genuine
  false-target mechanism, and it was completely invisible: `inferInterpModel` clamps
  silently and the live SFCW path had no span check at all.

**The brief's premise that the operating range is 326–482 mm was wrong**, and so was the
claim that the earlier 164 mm noise characterization was "measured at the wrong distance".
164 mm *is* the zero-standoff position. Real operation is lidar ≈ **165–315 mm**.

### Phase 0 — instrumentation (no compensation math was changed)

`pi/sensors/stream.py`:
- `lidar_poll_loop` is capped at `LIDAR_POLL_HZ = 20` (was uncapped, measured **584 Hz** for
  a sensor whose internal measurement only updates at ~11–17 Hz). `--lidar-rate 0` restores
  the old uncapped behaviour for comparison. Measured effect: broadcast rate unchanged
  (48.79 → 48.35 Hz), IMU update rate unchanged/slightly better (**32.66 → 33.83 Hz**).
- Packet now carries `lidar_seq` (increments per *successful read*) and `lidar_ts`. At
  20 Hz the seq sequence seen by a client is contiguous — every reading reaches a packet.

`App.jsx`:
- `lidarAccumRef` dedupes by `lidar_seq` before averaging. Measured against live packets:
  **13.01 → 5.27** accumulated samples per 250 ms sweep. Without this, `lidar_std` would be
  fiction (repeats deflate the spread) and readings would be weighted by how long they
  happened to be held.
- Every sweep record, C-scan cell and BG-model sample now carries `lidar_n`, `lidar_std`,
  `lidar_offset_mm`, `roll_deg`, `pitch_deg` via one shared `provenance` object, so the
  three record types cannot drift apart. Pose is tilt-from-gravity (accel is body-frame
  [forward, left, up]); no yaw, which gravity cannot observe.
- `processedSfcwResult` became `sfcwProcessed = { result, diag }`. Every path that declines
  to subtract now reports a reason, the out-of-span/clamp case is detected and reported
  (matching `CscanPanel.jsx`), and a running clamp fraction is kept. The SFCW panel shows
  "BG applied: YES / YES (CLAMPED) / NO" plus the reason — **"a model is loaded" and "the
  model was applied" are different statements** and only the second is now visible.
- Models record a `geometry` stamp (`lidarAntennaOffsetMm`, full `sfcwParams`, `builtAt`)
  in `bgmodel.worker.js`; the panel warns visibly when a loaded model's stamp disagrees
  with current settings, and says so distinctly for pre-stamp models where it cannot tell.

### Phase 1.1 — LiDAR noise across the real operating range

`pi/sensors/lidar_noise_char.py` (keepable tool). 40 s per distance, 584 Hz polling,
**100% valid reads (`error_code=0`) at every distance**, 1 mm quantisation:

| lidar reads | σ(τ=250 ms, one sweep) | internal update | autocorr half-life | raw σ |
|---|---|---|---|---|
| 164.8 mm (standoff 0) | **0.396 mm** | 17.2 Hz | 37 ms | 0.68 mm |
| 261.7 mm (standoff ~100) | **0.433 mm** | 11.5 Hz | 45 ms | 0.66 mm |
| 339.7 mm (standoff ~180) | **0.560 mm** | 11.5 Hz | 58 ms | 0.78 mm |

σ degrades only mildly with distance. Averaging follows σ ∝ τ^−0.12…−0.30, far shallower
than white noise's τ^−0.5, and plateaus by ~2 s — so **longer averaging does not rescue
it**; there is a correlated/drift floor around 0.15–0.33 mm.

### Phase 1.2 — the oracle test says this is NOT standoff-limited

Reconstructed the 29 training spectra from `models/4th model.json` by rewinding its stored
unwound knots, then for each position built a leave-one-out model and searched standoff
over ±15 mm (0.05 mm grid; ±15 mm is the unambiguous window, two-way λ/2 at 5 GHz = 30 mm)
for the standoff maximising suppression. The numpy port reproduces the browser's own stored
LOO numbers to **1.1e-14 dB**, so the port is not the variable.

- Suppression at the recorded lidar standoff: **20.22 dB** mean.
- Suppression at the *oracle* standoff: **23.19 dB** mean. **The oracle buys only 3.0 dB.**
- `d_oracle − d_lidar` is **zero-mean** (−0.10 mm, σ 1.88 mm, t = −0.30 on 28 df) with no
  trend against position (r = −0.26). No constant bias → not a geometry/offset error at
  capture depth. No drift → not thermal/mechanical creep.

The ±1.88 mm scatter is **not** lidar error: these knots are 40-sweep coherent means whose
standoffs average to σ ≈ 0.05 mm, and a genuine 1.9 mm standoff error would cap suppression
at ~8 dB, not the 20 dB actually observed. The oracle offset is a free parameter absorbing
*model* error, not recovering a true standoff.

**Measured standoff sensitivity is far gentler than the analytic single-echo table**, which
is the key physical result. Deliberately offsetting the inference standoff by ε (LOO, real
data, mean over 29 positions):

| ε | 0.25 mm | 0.5 mm | 1 mm | 2 mm | 3 mm | 5 mm | 10 mm |
|---|---|---|---|---|---|---|---|
| analytic (single echo, α=0.93) | 26.2 | 20.2 | 14.2 | 8.2 | 4.8 | 0.6 | −4.4 |
| **measured** | **19.9** | **19.5** | **18.5** | **15.8** | **13.3** | **9.6** | **4.2** |

The reason is in CLAUDE.md's own echo decomposition: the **dominant** background component
sits at **α ≈ 0 — a static cable/coupling reflection that does not depend on standoff at
all** (confirmed here: the background range-profile peak sits at 0.53–0.54 m and moves by
σ 0.022 m across a 156 mm standoff span, i.e. it does not move). Only the weaker α ≈ 0.93
wall face is standoff-sensitive. So the analytic table, which assumes *all* energy is at
α = 0.93, is structurally pessimistic — **do not use it to predict suppression.** The
SFCW panel deliberately shows no suppression-ceiling tile for this reason: an early draft
had one and it pointed straight at the wrong suspect. Only σ and n are shown, as
measurements rather than predictions.

Monte-Carlo over the real data, adding Gaussian standoff noise at inference:

| σ | 0.05 | 0.25 | 0.43 | 1.0 | 2.0 | 5.0 mm |
|---|---|---|---|---|---|---|
| suppression | 20.19 | 19.99 | **19.74** | 18.80 | 16.78 | 12.58 dB |

At the measured per-sweep σ = 0.43 mm the penalty is **0.5 dB**, not the predicted ~15 dB.

**Where the residual actually goes (the false-target mechanism).** The LOO residual peaks
at −21 dB relative to the background peak, with peak/rms ≈ 5.0 (a flat noise-like residual
gives 3–4; a discrete false target gives ≫10). Its location clusters in two places: right
at the background peak (0.51–0.66 m) and a short-range group at 0.07–0.12 m. At the larger
standoffs the residual peak sits 8–12 cm *beyond* the background peak — and since
subtraction removes the true wall return, that residual becomes the largest feature left on
screen. So the false targets are **the model's own incompletely-cancelled wall/coupling
residual, re-ranked to the top by the subtraction**, not a standoff-noise artifact.

### Phase 1.3 — the regime gap is 5.3 dB, and standoff owns 0.4 dB of it

Fresh 24-position set captured 2026-08-28 under the corrected 160 mm offset
(`data/bgmodel_pass1.json`, gitignored): span 101.2 mm, **median gap 4.1 mm** (inside the
≤5 mm "well sampled" band), 40 sweeps each, SNR 21.3 dB/sweep, pose stable to ±0.04°.
Per-sweep standoff scatter within a static capture measured **0.388 mm**, independently
confirming Phase 1.1's σ(250 ms) of 0.40–0.43 mm.

Four scorings on the same data (`scratchpad/regime_gap.py`), each one step closer to what
live operation actually does:

| | spectrum | standoff | mean suppression |
|---|---|---|---|
| A | 40-sweep mean | capture mean | **24.27 dB** ← what `evaluateLoo` reports |
| B | 40-sweep mean | per-sweep | 23.90 dB |
| C | single sweep | capture mean | 19.11 dB |
| D | single sweep | per-sweep | **18.99 dB** ← what live operation gets |

**Regime gap D−A = 5.3 dB, not the predicted ~15 dB.** Decomposed: standoff noise costs
**0.37 dB** (A−B), single-sweep measurement noise costs **5.16 dB** (A−C). The 0.37 dB
matches the Phase 1.2 Monte-Carlo prediction of 0.48 dB at this σ. **The leading hypothesis
is falsified on both counts** — the gap is 3× smaller than predicted and the mechanism it
named contributes almost none of it. C is bounded by per-sweep SNR itself (21.3 dB): no
subtraction of a single sweep can beat its own noise floor, whatever the model does.

### The actual false-target mechanism: querying the model OUTSIDE its captured span

This is the finding that matters. On the fresh set, leave-one-out splits cleanly by whether
the held-out position is bracketed by other knots:

- **interior (interpolated): 25.98 dB** mean, worst 19.35
- **endpoints (clamped): 5.43 dB** mean
- **penalty for being outside the span: 20.55 dB**

How fast it falls off just past the edge (query below the model's lowest knot, scored
against the nearest real capture):

| outside by | 1 mm | 2 mm | 5 mm | 10 mm | 20 mm | 40 mm |
|---|---|---|---|---|---|---|
| suppression | 20.6 dB | 14.6 dB | 6.8 dB | 1.1 dB | **−3.5 dB** | **−5.2 dB** |

Negative means **the subtraction adds more energy than it removes** — it manufactures a
return where there is nothing. That is the false-target mechanism, and `inferInterpModel`
enters it silently: it clamps to the nearest knot and returns a confident-looking spectrum.

**Why this fired constantly with the old 315 mm offset.** Standoff = `lidar − offset`. The
lidar reads ~165–315 mm in real operation (measured), so under offset 315 every live query
was `−150…0 mm`, while `models/4th model.json` spans 11–167 mm. **Every single sweep was
clamped, by 11 to 161 mm** — i.e. essentially always in the energy-adding regime of the
table above. A model trained under a *different* mounting is the case where the otherwise
exact offset cancellation does not apply.

Note this also explains why *freshly* trained models failed the same way, which had been
taken as ruling the geometry out: a fresh model trained and used under the same offset does
cancel, but it only covers the span the operator actually swept. Pressing the aperture
closer than the nearest training knot puts the query outside the span at the near edge,
where 5 mm already costs 19 dB. Span coverage, not offset, is the thing to check.

**Residual shape confirms it.** On interior positions of the fresh set the residual sits
−28 dB below the background peak with peak/rms **2.58** — genuinely noise-like (a discrete
false target would be ≫10). So a correctly-queried model leaves no false target at all.

**Verdict: not standoff-limited, and not really model-limited either — span-limited.**
Ranked by cost: clamping outside the span **20.6 dB**, single-sweep SNR **5.2 dB**,
standoff noise **0.4 dB**. Chasing lidar precision would buy at most a few tenths of a dB.

### Phase 2 — the live traverse (2026-08-28): span-clamp CONFIRMED, but the model went stale

`span_confirm.py --seconds 120` -> `data/span_confirm_20260828-161212.json`, 377 sweeps,
267 in span / 110 outside, scored against `bgmodel_pass1.json` by `span_analyze.py`.

| bin | n | measured | Phase 1 offline LOO | resid pk/rms |
|---|---|---|---|---|
| in span | 267 | **8.14 dB** | 25.98 | 2.39 |
| 0-1 mm out | 7 | 9.40 | 20.6 | 1.77 |
| 1-2 mm out | 16 | 8.91 | 14.6 | 1.87 |
| 2-5 mm out | 7 | 7.22 | 6.8 | 2.50 |
| 5-10 mm out | 22 | 5.21 | 1.1 | 2.53 |
| 10-20 mm out | 2 | -0.55 | -3.5 | 2.50 |
| >20 mm out | 56 | **-3.32** | | 2.46 |

**Span-clamp mechanism confirmed live.** Suppression falls monotonically with distance
outside the span and goes *negative* past 10 mm — the subtraction adds more energy than it
removes, exactly as Phase 1 predicted offline (-3.32 measured vs -3.5 predicted at >20 mm).
Phase 1's central claim survives contact with live data.

**But no discrete false target was reproduced.** Residual peak/rms is 1.77-2.53 in *every*
bin, in span and far outside it alike — flat, noise-like, never the >>10 that marks a
phantom. So the out-of-span regime degrades suppression and injects energy broadband; this
traverse did not show it manufacturing a discrete peak. The original symptom is still not
reproduced live. Do not treat the false-target mechanism as fully closed.

**The 8.14 dB in-span is a STALE MODEL, not a new mechanism.** Ruled out in order:

- *Not geometry/offset.* Sweeping an inference-standoff offset over +/-20 mm peaks at
  **+3.3 mm for 8.76 dB** — 0.62 dB over doing nothing. Same shape as Phase 1.2's oracle
  test: the free standoff parameter absorbs model error, it does not recover a true standoff.
- *Not configuration.* Training and traverse `sfcwParams` are identical field-for-field
  (2000-5000 MHz, 60 MHz step, tx1/rx1 50/25, tx2/rx2 50/25, settle 10, buffers 4) and both
  ran at `lidarAntennaOffsetMm = 160`.
- *Not hand motion during the traverse.* A plausible confound, since the traverse moves
  while the training captures were static, and a sweep that moves mid-sweep smears its own
  frequency-vs-phase relationship. Stratifying the 267 in-span sweeps by `lidar_std` (a
  motion proxy) kills it: stillest quartile (0.35 mm, effectively static) gives **8.06 dB**,
  fastest quartile (2.12 mm) gives **8.01 dB** — indistinguishable. Whatever costs the
  17.8 dB is present even when the rig is holding still, so it is not a motion artifact.
- *Not the measurement.* Traverse-vs-traverse at <0.5 mm separation gives **17.96 dB,
  coherence 0.9914** (n=1031 pairs) — today's sweeps predict each other well, and 17.96 dB
  matches Phase 1.3's regime-D single-sweep figure of 18.99 dB. The radar is fine.
- *It is the training set.* Traverse-vs-training complex coherence is **0.9338**, and
  `-10*log10(1-rho^2)` for rho=0.9338 is **8.93 dB** — essentially the 8.14 dB observed. The
  measured spectrum has decorrelated from the trained background by ~6.6% of its energy.
  Per-step `|h|` ratio now/training runs **0.60 to 2.17 (mean 1.267)** — strongly
  frequency-dependent, so not a gain change.

**Most likely cause: the bench was physically disturbed between the two sessions.**
`bgmodel_pass1.json` was captured 00:38; the traverse ran at 16:12, after a day of bladeRF
USB troubleshooting (repeated replugging, power cycles, a move to a USB 2.0 port and back,
FPGA reloads). CLAUDE.md's own echo decomposition says the *dominant* background component
is the alpha ~ 0 static cable/coupling reflection — precisely the term that moving cables and
connectors changes. A frequency-dependent 0.6-2.2x amplitude change is what a re-seated
connector or shifted cable dress looks like.

**Operational consequence: a background model has a shelf life bounded by the bench staying
untouched.** Any RF cable, connector, or antenna disturbance invalidates it, and the failure
is silent — the model still interpolates confidently and still reports "BG applied: YES".
Retrain after any hardware work, and treat a sudden in-span suppression drop as a staleness
signal rather than a modelling problem. Cheap staleness check, no recapture needed: score
traverse-vs-traverse coherence against traverse-vs-model coherence; if the first is ~0.99
and the second is well below it, the model is stale, not wrong.

### Phase 3 — fresh model closes the staleness gap (2026-08-28)

Two 30-position sets captured 5 min apart (`bgmodel_pass2_*`, `bgmodel_pass3_*`, 15 sweeps
each, span 5-160 mm), then the traverse re-run against pass3.

**Shelf life measured for the first time.** Cross-session scoring (build from A, score on B's
measured spectra, interior only):

| model -> data | elapsed | suppression |
|---|---|---|
| pass2 -> pass3 | ~5 min | **21.21 dB** |
| pass3 -> pass2 | ~5 min | **21.35 dB** |
| pass1 -> pass2/3 | ~16 h | **7.90 / 8.76 dB** |

The ~8 dB at 16 h independently reproduces the traverse's 8.14 dB against pass1 by a
different route, confirming that shortfall was staleness. **Caveat: two time points are not
a curve** — this cannot yet distinguish gradual drift from a step change caused by the day's
bladeRF USB/FPGA work in between, and those imply very different retraining cadences. Leaving
a model overnight with the bench untouched and re-scoring would separate them.

**Live traverse against the fresh model: in span 8.14 -> 17.70 dB.** Falloff outside the span
confirmed again (-3.45 dB at >20 mm out vs -3.5 predicted). 17.70 dB combines single-sweep SNR (21.7 dB) with
cross-session model error (~21 dB) for a predicted 18.4 dB, which matches.

**Corrected 2026-08-28 — in-span is MODEL limited, not SNR limited.** Measured directly by
coherently averaging K consecutive sweeps of an 87-sweep static capture: K=1 gives 19.28 dB,
K=16 gives 21.95 dB — **+2.83 dB against an ideal +12.0**, plateauing at ~22 dB. Averaging
removes only the sweep-noise term; what remains is the model floor. So sweep noise is worth
~3 dB of the total and no more, and **raising `num_buffers` cannot buy more than that same
~3 dB** while costing sweep rate. The plateau (~22 dB) sits right at pass3's own LOO
(23.57 dB), so the binding constraint is **interpolation error at the achieved knot density**,
not background drift over minutes and not the estimator choice.

**Do not merge capture sessions.** Merging pass2+pass3 (51 knots, 3.3 mm median gap) scores
22.40 dB mean — *worse* than pass3 alone (23.57). The density gain is cancelled by the 21 dB
inter-session disagreement being injected into the interpolation. Even 5 minutes of elapsed
time is enough that combining sessions does not pay.

**Hand-placement scatter costs 6-7 dB and is now the largest capture-side lever.** The
scheduler asked for 3.8-6.8 mm gaps; pass2 achieved 0.6-13.7 mm. Held-out positions in the
tightest third of brackets scored 24.37 dB vs 17.14 dB in the widest third (pass2), 26.31 vs
20.46 (pass3). `capture_bgmodel.py --span-lo/--span-hi` now prints a per-position target, but
its move window still counts down blindly and captures wherever the operator happens to be —
gating capture on `|error| < 2 mm` would recover most of that.

**The false target has still never been reproduced** — but see the target A/B below: the
peak/rms metric used to reach that statement is now known to be incapable of detecting a
target at all, so this conclusion carries no weight and needs redoing with the magnitude-vs-
reference detector. **A separate remaining hypothesis
is the display, not the physics:** in the 0-30 cm window the residual's dynamic range is
**23.1 dB vs the raw profile's 20.6 dB**, so a dynamic colour scale stretches flat post-
subtraction noise across the full colormap exactly as it did real structure beforehand. Test
it by pinning `sfcwScaleRange` to manual at the pre-subtraction limits and seeing whether the
"targets" survive.

### Target A/B (2026-08-28): peak/rms is not a target detector, and what is

Static bed, standoff ~24 mm, target placed then removed with nothing else changed. 126 sweeps
with, 87 without, ~7 min apart, both scored against `bgmodel_pass3`.

**The target is unambiguous**: magnitude change **+4.4 dB peaked at 21.2 cm**, against a
**0.23 dB** noise floor measured in the 0-10 cm wall/coupling region, which the target leaves
completely undisturbed. Complex signature is 19.3 dB above the coherent-mean noise floor.

**Both detection statistics in use scored it BELOW target-free background:**

| | target present | target-free |
|---|---|---|
| residual peak/rms | **1.52** | 1.75-1.91 |
| peak excess over median | **3.80 dB** | 6.36-8.37 dB |

A target that is extended by range resolution (~50 mm at 3 GHz) plus sidelobes lifts the
residual *floor* rather than spiking one bin. peak/rms is self-normalised, so a raised floor
raises the RMS and the ratio falls. **Any self-referential peakiness measure is blind to a
real target, and blind in the wrong direction.** Do not use peak/rms, or excess-over-median,
to decide whether something is there.

**What works: magnitude range profile vs a target-free reference at matched standoff.**
Calibrated threshold from this data: a magnitude change **> ~0.7 dB** (3x the 0.23 dB control
region) indicates a target.

**The LiDAR has a slow ZERO-DRIFT of ~1 mm over minutes, and within-capture noise statistics
are blind to it.** The lidar reported the standoff moving 1.03 mm between the two target A/B
captures. It had not: range-gating the complex difference to the wall/coupling region (0-10 cm)
gives **-40.1 dB**, where an actual 1.03 mm move would give **-14.0 dB** -- 26 dB below, i.e. the
true geometry held to **~0.05 mm** while the lidar's reading wandered by a millimetre. Use the
wall-gate phase, not the lidar, to decide whether the rig moved; the radar is ~20x the better
position sensor at this scale.

Independently confirmed on the training sets: a constant-bias search across the pass2/pass3
cross-session pair wants **-1.35 mm** one way and **+1.45 mm** the other. Equal-and-opposite is
the signature of a real zero-drift between sessions, not a fitting artifact.

**Cost is real but modest: ~2 dB** (pass2->pass3 21.21 -> 23.15 dB when the bias is corrected).
Far less than the single-echo table predicts for 1.4 mm (14 dB) because the dominant background
term is the alpha~0 static coupling, which does not depend on standoff at all -- the same reason
Phase 1.2's measured falloff was much gentler than the analytic one. A 1-parameter bias search
at inference is therefore a cheap ~2 dB, and being a slowly-tracked global constant it cannot
absorb a target the way a per-sweep standoff search could.

**This does NOT explain the staleness.** The same search on the 16 h pair (pass1->pass3) recovers
only **+0.69 dB**, so the overnight decorrelation is genuine background change, not lidar drift.

**Magnitude and complex tolerate very different amounts of this.** The magnitude range profile
was unaffected -- the 0-10 cm control stayed at 0.23 dB whether or not the standoffs were
matched -- because a sub-millimetre shift is a small fraction of a range bin. The complex
difference is not: at the (spurious) 1.03 mm the shift term would have swamped the target. So
**target detection can use magnitude and tolerate ~1 mm of standoff error; background
subtraction needs coherent cancellation and is sensitive to it.**

**Without compensation the target sits 16.6 dB below the wall return** (-51.4 vs -34.8 dB) --
a small bump on the skirt of a much larger feature, which is precisely the case background
subtraction exists to fix.

### Tooling added

- `pi/sensors/lidar_noise_char.py` — noise vs averaging window at a given distance.
- `pi/radar/capture_bgmodel.py` — captures a `bgmodel_training_data` v2 set headlessly
  (same format the BG Model panel exports, plus per-sweep `lidar_n`/`lidar_std`/pose
  columns). **The SDR socket must be drained continuously**: `sfcw_start` free-runs, so a
  move window that does not read the socket lets sweeps pile up, and the capture then
  drains the backlog with several sweeps sharing one instant — every standoff after the
  first came back `None` in the first version. The reader task now pairs each sweep with
  the lidar samples that arrived since the previous one, at arrival time.
- `pi/radar/bgmodel_interp.py` — numpy port of `bgModelInterp.js` + `rangeProfile.js`
  (build / infer / range profile / LOO). Reproduces the browser's own numbers exactly on
  `data/bgmodel_pass1.json`: interior LOO 25.98 dB mean / 19.35 worst, clamped 5.43 dB.
  It deliberately preserves `inferInterpModel`'s asymmetry — interpolating at the
  *clamped* standoff while rewinding phase at the *unclamped* one — because that is the
  failure being measured; a "tidier" port would not reproduce it. Written because
  CLAUDE.md's reference to `scratchpad/regime_gap.py` is dead (that file was in a session
  scratchpad and no longer exists), so the port now lives in the repo instead.
- `pi/radar/span_confirm.py` + `pi/radar/span_analyze.py` — **live** confirmation of the
  span-clamp mechanism, which Phase 1 established only offline (its own caveat: "I never
  observed a false target live in the app"). `span_confirm.py` records a continuous
  standoff traverse through and past both span edges — **aim at a blank wall**, the method
  rests on every residual peak being false by construction. `span_analyze.py` applies the
  model at each sweep's own standoff and bins suppression + residual peak/rms by
  mm-outside-span, against Phase 1's offline falloff (1 mm → 20.6 dB, 5 → 6.8, 20 → −3.5).
  `span_analyze.py` reports residual **peak/rms**, which is NOT a target detector — see the
  target A/B below. It is only a coarse "is the residual spiky" indicator; do not read a low
  value as absence of a target.

## Sweep-to-sweep variability is set by the REFERENCE channel's level (2026-08-29)

Investigating "a static rig on a static scene gives range profiles whose shape correlates
highly but whose values move by multiple dB, worst at particular ranges". Measured on the
bench with `sdr_server` stopped, driving `SFCWEngine` directly so `h_signal` and
`h_reference` could be kept separately (the wire only carries their ratio).

### The symptom is a level effect, not a range effect — dB is the wrong lens

Across 100 static sweeps the **linear** std of the range profile is essentially flat with
range (-41.9 to -34.9 dBr, a 7 dB spread) while the **dB** std spans 0.11 to 6.08 dB, and
`corr(dB-std, -mean level) = 0.878`. There is one flat additive complex noise floor across
the whole profile; wherever the profile dips toward it, the dB reading swings wildly, and
where it is strong the dB reading is rock solid (0.12 dB at the peak). So "variability
depends on position" is entirely "how far is this bin above the floor" — **the floor is the
only number worth tracking, and it is one number for the whole profile.**

### It is not thermal noise, not drift, and not settling

| measurement | result |
|---|---|
| within a step (4 buffers, 0.4 ms apart) | 0.13-0.22% -> ~54 dB |
| sweep to sweep (same step, ~300 ms apart) | 5.3% -> 25.5 dB, **29x worse** |
| lag-1..50 correlation of the residual | 0.01-0.06 (white) |
| first-half vs second-half mean of 100 sweeps | 35.4 dB (no drift) |
| removing a per-sweep complex scalar | 21.41 -> 21.91 dB (nothing) |
| removing a per-sweep delay/range shift | -> 21.98 dB (nothing) |
| error vs step signal level | corr 0.011 (**multiplicative, not additive**) |
| error across adjacent frequency steps | corr 0.08 (white in f -> flat in range) |

Averaging more buffers cannot help (the fast noise is already 29x below), and a 6x longer
settle does not help either. Paired A/B at one frequency isolates the trigger:

| | scatter vs baseline |
|---|---|
| capture again with **no** retune in between | 2.4% |
| retune to the **same** frequency, then capture | 6.4% |
| retune 1 GHz away and back | 6.5% |
| retune, then settle 60 buffers instead of 10 | 6.7% |

So it is the retune that re-randomises it, and it is not a settling transient.

### Root cause: RX2 was ~76% of ADC full scale, i.e. in compression

Magnitudes are immune to the LO phase, so they separate the two paths cleanly:

| | sweep-to-sweep | within-step |
|---|---|---|
| `|h_signal|` | 1.19% | 0.020% |
| `|h_reference|` | **5.62%** | 0.118% |
| `|h_cal|` | 5.44% | 0.133% |

**The reference — the thing whose entire job is to be the stable standard — is 4.7x less
stable than the antenna channel, and h_cal inherits it essentially untouched.** Because it
divides every step, its noise is *multiplicative*, which is exactly why the scatter is
independent of each step's own signal level (a step at 26 ADC counts and one at 571 are
equally noisy in relative terms).

It is a level problem. Sweeping TX2/RX2 gain gives a textbook compression curve
(60-80 sweeps per point, `|S|` cv stays ~1.0-1.2% throughout, so this is purely the
reference path):

| rx2 ADC peak (of 2047) | 1567 | 896 | 530 | 425 | 307 | 169 | 105 | 51 | 27 |
|---|---|---|---|---|---|---|---|---|---|
| `|R|` cv | 4.7% | 2.2% | 1.3% | 0.82% | 0.81% | 0.92% | 1.06% | 1.38% | 6.17% |
| h_cal | 28.4 dB | 37.9 | 39.7 | 43.6 | 43.6 | 45.0 | 42.6 | 39.8 | 29.7 |

**Target the reference at roughly 150-400 counts peak (5-20% of full scale).** Above that
the RX2 front end compresses; below ~50 it runs out of SNR. (These counts, and the table
above, are a *mean* of the per-step maxima. `adc_peak` on the wire is a *max over the
sweep*, which for RX2 runs ~1.1x higher and for RX1 ~4x higher -- do not compare the two
statistics directly. The post-fix numbers further down are all max-over-sweep.) End to
end, 80 sweeps each:

| | h_cal | noise floor | dB std median | worst | @0.4 m |
|---|---|---|---|---|---|
| tx2=50 rx2=25 | 26.2 dB | -38.2 dBr | 0.55 dB | 6.64 | 0.98 |
| tx2=30 rx2=20 (engine default) | 42.2 dB | -44.4 dBr | 0.16 dB | 1.13 | 0.17 |
| tx2=45 rx2=10, rx1=20 | **44.0 dB** | **-50.4 dBr** | **0.06 dB** | 0.62 | 0.07 |

**With the reference correctly levelled the radar is extremely repeatable**: two 40-sweep
means taken minutes apart on a static scene give complex coherence **1.0000** and 41.5 dB
suppression. There is no mystery drift; nearly all the observed variability was this.

The remaining floor is `|h_signal|`'s own 1.1% per-retune magnitude wobble (~39 dB), which
is what caps h_cal at ~44 dB. TX runs at `amplitude=0.9` of DAC full scale, which is the
obvious next suspect, but it has not been tested.

### The reference gain is invisible, unreachable, and STICKY — this is the real trap

- `App.jsx` `sfcwParams` has **no** `tx2Gain`/`rx2Gain`, so `sendSfcwParams()` never sends
  them. The panel therefore is *not* the source of truth for these two, contrary to the
  invariant stated in the SFCW params section above.
- `SFCWEngine` keeps whatever was last set for the **life of the `sdr_server` process**, and
  `_configure_hardware()` re-pushes it before every sweep. `capture_bgmodel.py` and
  `span_confirm.py` (`SFCW_PARAMS`) send **tx2=50 rx2=25**. So running either tool once
  silently moves every subsequent browser sweep into the compressed 26 dB regime until
  `sdr_server` is restarted — with nothing on screen indicating it. That is the most likely
  explanation for the symptom appearing intermittently across sessions.

### Reference gain also RE-CALIBRATES h_cal, so it invalidates background models

Changing only TX2/RX2 gain is **not** a constant scaling of h_cal. Measured on a static
scene (40-sweep means):

| | coherence | suppression |
|---|---|---|
| 30/20 vs 30/20 (control) | 1.0000 | 41.5 dB |
| 30/20 vs 50/25 | 0.8807 | **6.49 dB** |

Per-step `|h_cal|` ratio runs **0.007 to 1.840** with 46 deg of phase scatter — a strongly
frequency-dependent recalibration, because different AD9361 gain settings distribute gain
differently across LNA/mixer/PGA and each has its own frequency response.

**Consequence: a background model is only valid at the reference gain it was captured at,
and nothing records or checks that.** Models built by `capture_bgmodel.py` (50/25) applied
to browser sweeps (30/20) would be mismatched by construction. This is worth re-examining
against the Phase 2 "stale model" finding above: its signature was a per-step `|h|` ratio of
0.60-2.17, "strongly frequency-dependent, so not a gain change" — which is exactly the
signature a reference-gain change produces. The `geometry` stamp added in Phase 0 records
`sfcwParams`, but since `sfcwParams` does not contain tx2/rx2 it cannot catch this.

### Separate bug: `sync_rx` in RX_X2 delivers HALF the samples the code assumes

`_rx_loop_dual` calls `sync_rx(buf, num_samples)` with `num_samples=4096` and a buffer sized
for 4096 samples *per channel*. In `RX_X2`, libbladeRF counts `num_samples` as the **total**
across both channels. Verified by poisoning the buffer with `0xAA` before the call: **the
upper 50% is never written**, and throughput is 4886 calls/s at 10 Msps = 2047 per-channel
samples per call (not 2441 calls/s / 4096 samples).

`buf` is allocated once outside the loop, so every capture is 2048 fresh samples followed by
2048 samples that are one buffer old. Steady-state harm is small — the stale half comes back
rotated by `exp(-j*2*pi*100e3*4096/10e6)`, costing 0.8% amplitude and a constant 7.2 deg that
cancels in the ratio — but:

- **A buffer carried 0.205 ms of signal, not 0.41 ms**, so `settle_count = 10` bought
  2.05 ms of *settled signal* rather than the 4.10 ms `SfcwPanel.jsx`'s
  `BUFFER_SAMPLES`/`SAMPLE_RATE` and CLAUDE.md's own sweep-timing section both claim. That
  bears on the `settle_count` 10 -> 7 regression documented above: it was really 1.43 ms.
- **The wall clock did NOT halve with it, and that is the confusing part.** Measured
  retune -> settle-satisfied is **4.40 ms** (p10 4.28, p90 4.50), not 2.05 ms, and it is
  not buffer loss -- arrivals run at 4882/s against an ideal 4883/s both idle and under
  sweep load, i.e. exactly real time with nothing dropped. The cause is that `_sweep_core`
  holds `_rx_cond`'s lock across the whole settle-plus-capture block, so `_rx_capture`
  blocks on it, the RX thread stalls, buffers back up in the 16-deep ring, and it catches
  up in the gap between steps (`seqd` counts ~14 arrivals per step inside the block while
  ~29 actually occur). So the *elapsed* time was roughly what the repo assumed while the
  *settled signal* was half, and the true settling sat somewhere between those two bounds
  depending on ring backlog -- unknowable, which is itself the problem. With the fix a
  buffer is genuinely 4096 samples / 0.41 ms and the ambiguity is gone.
- Consecutive buffers overlapped 50% in content, so `num_buffers` averaging was over fewer
  independent captures than it looked.

Fix is to request `2 * n` from `sync_rx` (or size the buffer for what actually arrives) and
to check the delivered count rather than assuming it.


### Fixes shipped and re-measured, same evening (2026-08-29)

Changed: `sync_rx` request corrected in `_rx_loop_dual`; `tx2_gain`/`rx2_gain` added to
`App.jsx` `sfcwParams` + `sendSfcwParams()` and to `SfcwPanel.jsx`'s Gains section;
`capture_bgmodel.py` `SFCW_PARAMS` brought into line; `adc_peak` + `gains` added to every
`sfcw_result` with a headroom bar in the panel and a hysteretic stdout warning;
`geometryMismatch()` now compares `tx2Gain`/`rx2Gain`.

100 sweeps per row through the real `_perform_sweep()`, static scene:

| | Hz | h_cal | floor | dB std med | worst | rx1 pk | rx2 pk |
|---|---|---|---|---|---|---|---|
| shipped, tx2/rx2 = 30/20 | 2.22 | **45.3 dB** | -44.2 dBr | **0.15 dB** | 1.29 | 873 | 373 |
| repeat of the same config | 2.22 | 45.1 dB | -44.5 dBr | 0.14 dB | 1.17 | 871 | 393 |
| what the capture tools forced, 50/25 | 2.22 | **34.0 dB** | -43.7 dBr | 0.25 dB | 3.69 | 857 | 1817 |

The two identical rows agreeing to 0.2 dB is the run-to-run error bar for every number
here. Against the 50/25 a session inherited after running `capture_bgmodel.py` this is
**+11.3 dB**; against the same 30/20 measured before the `sync_rx` fix (42.2 dB) the fix
alone is worth **+3.1 dB**; against the original measured baseline (50/25 with the
half-buffer bug, 25.5 dB) the total is **~20 dB**.

**The `sync_rx` fix costs sweep rate: 3.16 -> 2.22 Hz** (-30%), because a buffer now really
is 4096 samples / 0.41 ms instead of 2048 / 0.205 ms. What it buys is that `settle_count`
means what it says -- settling went from 2.05 ms to a genuine 4.10 ms. If the rate is
wanted back, `settle_count = 5` restores ~3.1 Hz at exactly the 2.05 ms of settling the
system has actually been running with all along; that is a knowing trade, not a free one,
and per the `settle_count` regression note above it needs a per-step validation first.

**`tx2/rx2 = 45/10` was shipped for part of this session and then reverted to 30/20.** It
came from a gain scan run before the `sync_rx` fix; the fix moved the optimum, and 45/10
measures 45.8 dB post-fix against 30/20's 46.2-47.1. The Pi-side default therefore ends up
unchanged -- **the engine default was never the bug.** The bugs were that the panel could
not set it and that the headless tools silently could.

**The ADC warning needs hysteresis, and this is not a detail.** RX1's per-sweep peak sits
right on any sensible threshold in normal operation -- measured flipping between 78% and
100% FS from one sweep to the next -- so a plain threshold test printed a warning and a
recovery every couple of sweeps. With `ADC_HOT_SWEEPS_TO_WARN = 8` /
`ADC_CLEAN_SWEEPS_TO_CLEAR = 30` the same 300 sweeps produce **exactly one line**, on the
50/25 run, naming RX2 at 90% FS. Also note RX1 and RX2 need *different* thresholds (0.75
vs 0.40) because `adc_peak` is a max: RX2's reference is flat across the band so its max
represents every step, while RX1's max is whichever single frequency the scene is
strongest at and says nothing about the other fifty.

**RX1 genuinely clips intermittently at `rx1_gain = 25`** -- peak hits the 2048 rail on
0-8% of sweeps in this scene (18% at one gain setting), which nothing could see before
`adc_peak` existed. It is not currently costing anything measurable (`|h_signal|` cv stays
~1.1% at rx1 20 / 25 / 30), so it has been left alone rather than changed blind, per the
"retest via RF Calib at the exact new numbers" rule above. Watch the panel's RX1 bar; if
it sits red on a strong target, drop `rx1Gain` and re-measure.

### The `coherent` flag does not measure what its name suggests

Over 100 sweeps per configuration it reads **`coherent = False` 100% of the time in every
configuration, including the good ones** -- and it did so before any of these changes, so
nothing here broke it. `_process_h_cal` computes `phase_std` as the residual of a *linear*
fit to unwrapped phase vs step index, which is only small when the scene is a single
dominant reflector. A real multi-target scene is not linear in frequency and never will
be, so against the fixed 0.3 rad cut the flag is pinned False regardless of hardware
health. Measured: 1.39 rad at 30/20, 3.90 rad at 50/25.

**What is diagnostic is the sweep-to-sweep spread of `phase_std`, not its value.** The
structure repeats almost exactly when the hardware is healthy and stops repeating when it
is not:

| | mean phase_std | sd across 100 sweeps |
|---|---|---|
| 30/20 | 1.3897 rad | **0.0047** |
| 30/20, repeat | 1.3915 rad | 0.0048 |
| 50/25 | 3.9029 rad | 0.0078 |

A retune-timing corruption -- the thing this check exists to catch -- would move `phase_std`
sweep to sweep, because corrupted sweeps do not repeat. So the useful test is
`std(phase_std)` over a window, or a comparison against the previous sweep's phase, not a
fixed absolute cut on one sweep. The flag as shipped is a scene detector wearing a
hardware-health label; treat it accordingly until it is reworked.

### What a bin AT the floor does, and the FLOOR overlay (2026-08-29)

A bin's dB wobble is `~8.686 * sigma / A` -- sigma fixed, `A` spanning 40+ dB across the
profile -- so the *same* error reads as 0.1 dB on the peak and tens of dB in a null. Bins
grouped by headroom above the floor, 200 static sweeps at the shipped config:

| headroom | dB std | worst dip | worst spike |
|---|---|---|---|
| >30 dB | 0.14 | 0.4 | 0.4 |
| 20-30 | 0.69 | 2.0 | 1.7 |
| 15-20 | 1.64 | 4.9 | 3.3 |
| 10-15 | 2.22 | 8.0 | 4.2 |

**The dips grow faster than the spikes, and that asymmetry is the fingerprint of a bin at
the floor**: a noise phasor can very nearly cancel the signal (-> -inf dB) but can at most
double it (-> +6 dB). A bin reported as "sits at -50 dB, drops to -65, jumps to -45" is
therefore not a fault and not a contradiction of the 0.72 dB worst-case quoted from a
different scene -- it is a bin with ~0 dB of headroom, i.e. no measurement at all. **Any
"worst-case wobble" figure is only meaningful together with the headroom range it was
measured over**; the number above was scoped to a window whose weakest bin was 22 dB up.

`SfcwDisplay` has a **FLOOR** toggle (next to CFAR) drawing the measured floor: per-bin sd
of LINEAR amplitude over the last `FLOOR_WINDOW = 16` sweeps, smoothed +/-10 bins, scaled by
`1/sqrt(avgCount)` (exact, since the error is white sweep-to-sweep). Linear, not dB, because
the error is additive in amplitude and flat in range -- and it follows the R^n gain
automatically since the stored rows are already range-compensated. Validated against
200 sweeps: predicted wobble `8.686 * floor/level` vs measured, at 0.10 / 0.30 / 0.39 /
0.50 / 0.70 m -> 0.09/0.10, 0.11/0.11, 0.19/0.19, 0.57/0.60, 0.29/0.30 dB. The line itself
is stable to 0.82 dB frame to frame. Suppressed in the zeroed trace modes, where an
absolute floor would be meaningless.

**Averaging works and follows 10*log10(N).** Measured floor: N=1 -44.5 dBr, N=4 -49.6,
N=8 -52.7, N=32 -60.9, with dB std median 0.171 -> 0.091 -> 0.058 -> 0.023. The display's
existing Avg control averages *magnitudes* (incoherent) rather than complex `h_cal`;
measured, the two are identical to three decimals on this scene because nothing gets
within 22 dB of the floor, and they only diverge inside ~6-10 dB of it. Worth switching for
deep-null work, not urgent.

### settle_count: 10 -> 3 (2026-08-29), validated per step

Once `sync_rx` was fixed a buffer is 0.41 ms again, and `settle_count` turned out not to
discriminate at all. 100 sweeps at each value, plus 400 each at the two finalists. "bad
steps" counts `(sweep, step)` cells more than 8 robust sigmas off that step's own median --
a step that retuned late holds the previous frequency's IQ and lands nowhere near it. This
is the **per-step** check the regression note above demands; an aggregate correlation is
what let the last regression ship.

| settle | Hz | h_cal | phase_std | ps sd | bad steps | worst z |
|---|---|---|---|---|---|---|
| 10 | 2.22 | 47.1 dB | 1.3615 | 0.0051 | 0/5100 | 2.5 |
| 5 | 2.88 | 47.3 | 1.3578 | 0.0048 | 0/5100 | 2.9 |
| 4 | 3.10 | 46.7 | 1.3575 | 0.0054 | 0/5100 | 2.2 |
| 3 | 3.27 | 47.3 | 1.3581 | 0.0040 | 0/5100 | 3.5 |
| 2 | 3.52 | 48.0 | 1.3566 | 0.0050 | 0/5100 | 2.9 |
| 1 | 3.80 | 48.6 | 1.3549 | 0.0039 | 0/5100 | 2.4 |

400-sweep confirmation: **settle=3 -> 3.35 Hz, 0/20400 bad, worst z 2.9**; settle=1 ->
3.90 Hz, 0/20400 bad, **worst z 7.6**. Gaussian expectation for 20,400 samples is ~4.1, so
settle=1 threw a genuine tail excursion and settle=3 did not. **3 is chosen for margin, not
for speed** -- it takes 51% of the rate back (2.22 -> 3.35 Hz) and the last 16% is not worth
the exact intermittent tail that caused the earlier regression.

Note the `coherent` flag was False 100% of the time at *every* settle value, so it could not
have picked a winner -- see the section on why it is a scene detector, not a health check.

### The per-retune magnitude wobble: what actually limits h_cal now

With the reference levelled, the binding term is that `|h_signal|` at a fixed frequency
changes between sweeps even though the scene does not. Measured over 120 sweeps at the
shipped config, keeping both channels separately:

| | sweep-to-sweep | within a step (4 buffers, 0.4 ms) |
|---|---|---|
| `|h_signal|` | 0.912% | 0.025% |
| `|h_reference|` | 0.731% | 0.011% |
| `|h_cal|` | 0.815% | **0.029% (70.8 dB)** |

**The instantaneous measurement is superb and the retune is what costs everything** -- 30x
worse between sweeps than within one, frozen for the duration of a step and re-drawn at the
next retune. So more `num_buffers` cannot help; the noise it averages is already 30x below
the limit.

Decomposing by how much the two channels move together (`rho = 0.562` between their
fractional magnitude fluctuations):

| component | size | cancelled by the ratio? |
|---|---|---|
| common to both channels | 0.593% | yes, entirely |
| signal-channel specific | 0.694% | **no** |
| reference-channel specific | 0.427% | **no** |

**Only about half of it is common-mode, and the signal channel carries the largest
uncancelled share.** That is the number to attack, and it points at what is different about
the TX1 -> antenna -> RX1 path: TX1 runs at 50 dB against TX2's 30 dB (20 dB more drive,
much closer to TX compression), the TX DAC sits at `amplitude = 0.9` of full scale, and RX1
sees a 43 dB spread of level across the band and hits the 2048 rail at the strong end.
Amplitude-dependent gain is exactly the mechanism that was costing 11 dB on the reference.

Untested experiments, in order: (1) sweep `tx_amplitude` 0.9 -> 0.3 and `tx1_gain` 50 -> 40;
(2) sweep `rx1_gain` 25 -> 10 watching clip% and the signal-specific term; (3) **equalise the
two chains' operating points** (same gains, a pad on the loopback to match levels) so more
of the wobble becomes common-mode and cancels -- structurally the right fix, since the
reference can only cancel what both paths share.

Caveat on the absolute numbers: this run reports `|h_cal|` at 41.8 dB where runs minutes
earlier gave 47-48 dB on the complex metric. Part is the metric (magnitude-only cv vs
complex residual, worth ~1 dB) and the rest is that the bench was being handled between
runs. **The decomposition is a ratio of terms measured within one run and is the robust
part; do not quote 41.8 dB as the system figure.**

### Three gain experiments against one metric (2026-08-29)

**The metric, and why the earlier ones were not good enough.** `S_repeat`, single-sweep
repeatability:

    S_repeat = 10*log10( sum_i <|H[k,i]|^2>_k / sum_i <|H[k,i] - H[k-1,i]|^2>_k / 2 )

signal energy over the energy of the ADJACENT-SWEEP difference. It is immune to slow drift
during a capture (deviation-from-the-mean is not -- that is what made one run read 41.8 dB
where runs minutes earlier gave 47-48), the /2 corrects for a difference of two independent
samples having 2x the variance, and being a ratio computed within each configuration it
stays comparable even though changing any gain re-calibrates h_cal's shape. Every run below
repeats the baseline config at the START and END; the three controls agreed to **0.2 dB**,
which is the error bar on everything here.

| experiment | result |
|---|---|
| **E1** TX DAC `amplitude` 0.9 -> 0.7 -> 0.5 -> 0.3 | 27.8 / 27.9 / 28.0 / 28.1 dB -- **no effect**, hypothesis falsified. Within-step SNR degrades 72.4 -> 66.2 dB, so it costs and buys nothing. Note `amplitude` is shared by both TX chains, so it is a common-mode change and cancels in the ratio. |
| **E2** `rx1_gain` 25 -> 20 -> 15 -> 10 | 28.1 / 28.5 / 28.6 dB -- **no effect**. rx1=20 removed RX1 clipping entirely (0% vs 12%) and gained nothing, confirming the clipping is not currently costing anything. |
| **E3a** `tx1_gain` 50 -> 45 -> 40 -> 35, RX1 compensating | 31.2 / 30.9 / 30.6 vs control 28.1 -- a real **+3 dB**, but see below: it does not combine. |
| **E3b** `tx2_gain` toward TX1 at ~constant reference level | 20/30 -> **19.7**, 30/20 -> **28.1**, 40/10 -> **36.7**, 45/5 -> **38.6 dB**. Monotonic across 19 dB. |

**E3b is the result.** Confirmed on the operational metric too, same bracketed run:

| | S_repeat | range-profile floor | dB std median | worst |
|---|---|---|---|---|
| tx2/rx2 = 30/20 (control x3) | 28.1 dB | -45.8 dBr | 0.196 | 2.2 |
| **tx2/rx2 = 45/5** | **38.6 dB** | **-53.2 dBr** | **0.065** | **0.84** |

**It is not a level effect.** 45/5 sits at 342 RX2 counts and 45/10 at 585 -- both 38.6 dB --
while the control at 391 counts, in between them, is 28.1 dB. Holding the level and moving
only the TX2/RX2 split is what changes it.

**But the mechanism is NOT confirmed, and the obvious story is wrong.** "Match the two
chains so more of the per-retune error is common-mode" predicts that tx1=45 with tx2=45
should be best; measured, it is **34.8 dB, worse** than tx1=50 with tx2=45 (38.7). The
magnitude-correlation statistic `rho` was too noisy to arbitrate (two identical controls
gave 0.570 and -0.043), and `|R|cv` is unchanged across all of these (0.33-0.38), so the
improvement is not in reference *magnitude* stability -- it is in phase. Most likely an
empirical property of where the AD9361 TX gain table lands at this frequency plan.
**Re-measure after any RF hardware change instead of assuming it transfers**, and note that
E3a's +3 dB does NOT add to E3b: 45/5 alone beats the combination by 3.9 dB. `tx1_gain`/
`rx1_gain` therefore stay at the bench-validated 50/25.

### num_buffers 4 -> 1, and the sweep-rate readout (2026-08-30)

`num_buffers` averages that many post-settle captures per step, so it can only attack noise
that changes WITHIN a step. Measured 2026-08-29 that term is **0.029% (70.8 dB)** while the
binding limit -- the per-retune wobble -- is **38.6 dB**. Averaging 4 buffers therefore
changes the total by `10*log10(1 + 10^-3.2)` = **0.003 dB** while costing 3 buffer-times per
step. Set to 1 in `SFCWEngine`, `App.jsx` and `capture_bgmodel.py`.

**This reverses the 2026-08-23 restoration of 4 documented above, and that restoration was
correct at the time** -- the reference was compressed then and the within-step term sat much
closer to the limit. The justification here is entirely the 32 dB gap between the two, so if
the RF chain ever regresses this needs re-deriving, not assuming. A bench A/B against
S_repeat with bracketed controls is still outstanding (the device was in use); the argument
above is arithmetic from a measurement, not a measured A/B.

`Viewport.jsx` gained `useSweepRate()` -- median of the adjacent differences of the Pi's own
`sfcw_result.timestamp` over a 12-sweep window, shown in the SFCW pane header as
`<ms> ms / sweep - <hz> Hz`. Median, not mean, so one stalled or dropped frame does not move
it; from the Pi's timestamps, not render timing, so it reports what the radar is doing
rather than how fast the browser redrew. The hook is called unconditionally at the top of
`Viewport` because the per-panel branches are early returns.

### Complex vs magnitude background subtraction (2026-08-30)

**The complex/magnitude toggle is back, and it must not be deleted again.** It was removed
once on the reasoning that complex was the only correct mode. The 2026-08-28 target A/B
showed the two do different jobs and both are needed:

| | what it is | what it is for | tolerance to standoff error |
|---|---|---|---|
| **complex** | vector difference of `h_cal` | removing the wall/coupling return, so a target 16.6 dB beneath it is not buried | poor -- 1 mm = 12 deg at 5 GHz |
| **magnitude** | `\|profile\|` minus `\|reference profile\|`, in dB | the detection DECISION -- this is the statistic that actually found the target (+4.4 dB at 21.2 cm against a 0.23 dB control) | good -- survives ~1 mm, which the complex difference does not |

Complex is for seeing; magnitude is for deciding. Neither is a better version of the other.

**Where each happens, and why they differ.** Complex subtraction stays in `App.jsx`
`sfcwProcessed` and writes back into `h_cal_real/imag` (it has to -- `SfcwDisplay`
recomputes its own profile from those, so anything replacing only `magnitudes` is
discarded). Magnitude subtraction *cannot* be expressed as a modified `h_cal`, so App
instead passes the background spectrum through as `bg_h_cal_real/imag` with
`bg_sub_mode: 'magnitude'`, and `SfcwDisplay` transforms both with whatever window is
currently selected and differences the results. Doing it there rather than in App keeps the
window / zero-pad controls live and guarantees both profiles are built identically, which
is the only way their difference means anything.

Consequences worth knowing:
- **dB and LINEAR are two DIFFERENT quantities here, not two views of one** (2026-08-30).
  dB gives `20log10|P| - 20log10|P_bg|`, a *ratio*: "by what fraction did this bin change",
  level-independent, and therefore amplifying noise wherever the profile nears the floor
  exactly as an ordinary dB trace does. LINEAR gives `|P| - |P_bg|`, an absolute
  *amplitude* difference: "how much energy was actually added here", so a bin sitting on
  the noise floor contributes almost nothing however wildly its ratio swings. Neither is
  the other rescaled, so the choice is made in the recompute (which now depends on
  `scaleMode`) and NOT by converting at draw time -- the usual `10^(x/20)` step is skipped
  in diff mode or it would be applied twice. The +/-0.7 dB detection band is drawn only on
  the dB flavour, because a relative cut has no single linear value.
- **R^n is disabled in magnitude mode.** Range compensation applies the same gain to both
  profiles, so it cancels exactly; leaving it live would be a control that does nothing.
- **The waterfall is cleared on a mode change**, because rows already in it are absolute dB
  in one mode and a ratio in the other. It refills at the sweep rate (~30 s for 100 rows).
  Making the toggle re-render the existing history instead would need the display to hold
  genuinely raw `h_cal` in both modes -- possible (App could pass the background in complex
  mode too, letting the display reconstruct `raw = result + bg`) but not done.
- **The FLOOR overlay is suppressed** in magnitude mode, and its estimator now skips the
  buffer entirely there: it converts stored rows with `10^(x/20)`, which is wrong for a dB
  ratio and wrong again for a linear difference.
- **The waterfall's wf transforms (CFAR-relative etc.) are bypassed** on a difference --
  they are defined against an absolute dB profile, and a difference is already referenced
  to something.
- A **+/- 0.7 dB detection band** is drawn instead, from `DETECT_THRESHOLD_DB`. That is 3x
  the 0.23 dB target-free control region from the A/B. **It is provisional** -- the control
  was measured before the reference-gain and sync_rx fixes dropped the floor ~15 dB, so the
  real threshold is now probably lower and re-running the A/B would buy sensitivity.

### Tooling

`scratchpad` probes only (not committed). If this needs redoing: drive `SFCWEngine`
directly with `sdr_server` stopped and keep `h_signal`/`h_reference` separately — the
websocket only carries `h_cal`, and the whole diagnosis turns on being able to tell the two
channels apart. Also record `max|I|,|Q|` per channel per step; ADC headroom is still
unguarded anywhere in `sfcw_engine.py`/`bladerf_driver.py`, and it was the entire answer here.

## RF Calib panel gains are NOT the SFCW sweep's gains (2026-08-25)

Easy to conflate since both transmit the same 100 kHz-offset CW tone (`set_waveform('cw',
offset=100_000, ...)`), but they carry **independent** gain state. RF Calib panel drives
`BladeRFDriver.tx_gain`/`rx_gain` directly (defaulting to 50 dB / 25 dB — see the RF Calib
defaults change above). `SFCWEngine._configure_hardware()` (`sfcw_engine.py:443-450`)
overwrites those same driver fields from its own `tx1_gain`/`rx1_gain`/`tx2_gain`/`rx2_gain`
and `amplitude=0.9` right before every sweep — these are set **independently** in both
`App.jsx` (`sfcwParams.tx1Gain`/`rx1Gain`) and `sfcw_engine.py`'s own `__init__` defaults,
and must be kept in sync manually; there's no shared source between the two.

**Verified 2026-08-25: user bench-tested the RF Calib panel at 60 dB TX / 90% amplitude /
40 dB RX (SFCW's gain point at the time) per the recommendation below, and confirmed the
result acceptable.** SFCW's own `tx1_gain`/`rx1_gain` defaults were then dropped from
60/40 to **50/25** (both `App.jsx` `sfcwParams` and `SFCWEngine.__init__`) to match the RF
Calib panel's defaults, since 50/25 was the tested-good operating point. `tx2_gain`/
`rx2_gain` (reference channel) were untouched. If SFCW's gains are ever changed again,
retest via the RF Calib panel at the *exact* new tx1/rx1 numbers first — testing at
whatever the RF Calib panel happens to default to does not characterize the sweep unless
the two are known to match, which is why they were brought into alignment here.

**Why harmonics of that 100 kHz tone (seen on the RF Calib panel's live FFT as spurs at
odd multiples — ~3×, ~5× — of the 100 kHz offset, tens of dB down) mostly don't reach the
SFCW range profile.** The RF Calib panel's FFT is a wideband capture — it shows everything
in the passband, harmonics included. `SFCWEngine._sweep_core` (`sfcw_engine.py:598-601`)
never does a wideband FFT at all: it demodulates by multiplying the raw RX IQ against
`exp(-j*2*pi*cw_offset*t)` and taking the mean over n=4096 samples — a single-frequency-bin
coherent extraction (matched filter) at exactly the 100 kHz reference frequency, not a
spectrum. Bin spacing at n=4096/10 Msps is ~2.44 kHz; a harmonic ~200-400 kHz away sits
roughly 80-165 bins off-target, which a rectangular-window single-bin DFT rejects by very
roughly another 45-55 dB beyond whatever level it already sits at in the wideband FFT. That
headroom is why the specific spurs found on the RF Calib panel are not expected to be a
first-order concern for h_cal quality — confirmed adequate at the 60/90/40 bench test above.

**What can actually corrupt h_cal, and isn't checked in software:** (1) TX compression at
the fundamental itself (100 kHz offset) — amplitude/phase nonlinearity right at the
frequency being measured isn't filtered out by the coherent extraction the way a harmonic
is, though it partially cancels through the `h_signal / h_reference` ratio if TX1/TX2
compress similarly; (2) RX ADC clipping from RX gain pushed too high — confirmed by grep,
there is no clipping/saturation check anywhere in `sfcw_engine.py` or `bladerf_driver.py`
(`_process_h_cal`'s phase-coherence check catches retune-timing corruption, not amplitude
clipping). Both are real, unguarded failure modes; the odd-harmonic spurs from the RF Calib
panel are, by contrast, structurally rejected by the demod and a lower-priority concern.

## Rover C-scans are background-gradient limited, not motion limited (2026-08-30)

Three 12x3 rover-driven C-scans (`3row`, `3row2`, `3row3`, 50 mm pitch, one pipe near
the scan centre; `3row` with no background, the other two with a corner `bgRef`) showed
nothing on the GUI, against a static rig that works consistently. Diagnosed offline from
the exported v6 JSON. **The target is not detectable in any of the three**, and the reason
is not the rover moving.

**Note `hStep`/`vStep` are in CENTIMETRES.** `hStep: 5` is a 50 mm cell pitch, and the
rover positions confirm it (x steps of 50.0 mm). Easy to misread as 5 mm and conclude the
gantry is mis-scaled.

### The dominant error is a smooth background gradient locked to ABSOLUTE rail position

`|gain|` of each cell against a common reference falls monotonically from ~1.12 at
x = 750 mm to ~0.80 at x = 1550 mm -- a ~30% amplitude ramp across the scan, plus a phase
ramp worth ~0.6-1 mm of apparent standoff. It is not drift and not noise:

- **It is spatial, not temporal.** The rover snake captures the middle row RIGHT-TO-LEFT
  and the outer two LEFT-TO-RIGHT. All nine rows across the three scans have the
  same-sign `ix` gradient (-0.006 to -0.020 per cell). A drift in time would flip the
  sign on the three reversed rows. It does not. **Regression cannot settle this** -- in a
  snake, capture index is a deterministic function of `(ix, iy)`, so "t adds nothing on
  top of x,y" is vacuous. The direction-reversal test is the only clean discriminator;
  use it.
- **It reproduces.** `3row2` and `3row3` overlap over 350 mm of rail and agree to ~0.03
  in `|gain|` at matched absolute x. Same position -> same background.
- **It lives at the WALL, not in the coupling.** Band-resolved swing across a row:
  0-6 cm (the alpha~0 coupling) **0.55 dB**, 6-10 cm 1.05 dB, **10-16 cm 6.3 dB**,
  16-30 cm 1.7 dB. So this is NOT the cable/coupling term CLAUDE.md's staleness section
  blames elsewhere -- that term is rock stable here. It is the wall return itself.
- **The standoff is fine.** The wall peak stays at 13.3-14.5 cm across the whole scan
  (<1 mm of movement), so the rail is parallel to the wall and this is not a range shift.

**The mechanism is a TILTED WALL changing the standoff, and it is anisotropic
(established 2026-08-30 from `row4`, which had LiDAR; supersedes an earlier
"coherent speckle" reading of the same data that was WRONG).** Two hypotheses were tested
and rejected first, do not re-run them: (1) beam/illumination change -- rejected because the
top-vs-bottom difference *reverses sign* across the band (+5.1 dB at 2-3 GHz, -2.7 at 3-4,
+1.5 at 4-5, same in all three scans); a real amplitude effect cannot do that. (2) a
two-echo relative path shift -- modelling `h(top) = a*NEAR(bot) + b*exp(-j2pi f tau)*FAR(bot)`
railed `tau` at the search boundary and bought 0.6-1.1 dB.

**The decorrelation is strongly anisotropic, and that is the whole tell:**

| separation | 50 mm | 100 mm | 150 mm | 200 mm |
|---|---|---|---|---|
| **horizontal** (same row) | 20.1-21.9 dB | 17.3-20.6 | 14.6-19.2 | 12.4-18.4 |
| **vertical** (same column) | 10.2-10.8 dB | 3.5-4.9 | 0.3 | -- |
| `row4` standoff change, horizontal | 1.2 mm | 1.8 | 2.2 | 2.4 |
| `row4` standoff change, vertical | 3.4 mm | 7.7 | 10.2 | -- |

Moving 200 mm sideways costs a few dB; moving 150 mm up destroys the background entirely.
The LiDAR says why: sideways changes the standoff by ~2 mm, upwards by ~10 mm. **Suppression
tracks standoff change, not distance travelled**, and the numbers sit close to CLAUDE.md's
own measured standoff-sensitivity table (5 mm -> 9.6 dB, 10 mm -> 4.2 dB). The same
anisotropy is present in the LiDAR-less `3row*` scans, so the tilt was there too.

**Methodological warning that produced the wrong first answer:** the original decorrelation
sweep matched cells on x, so *every* pair it measured was vertically separated. It probed
one axis and the conclusion "isotropic speckle, decorrelates in ~20 mm, cut the pitch"
was drawn from it. **Always measure both axes before calling a spatial effect isotropic.**
The pitch was never the problem -- horizontally, 50 mm sampling is fine.

**This is the explanation for "static works, moving does not".** Held still the background
is stable to ~20 dB and a +4.4 dB target is easy. Drive the rover vertically and the
standoff walks by ~7 mm per 100 mm of travel, which alone costs ~15 dB. The false target
seen while driving is the standoff-mismatched background failing to cancel.

**Consequence: a single captured `bgRef` cannot work on a rover raster.** Subtracting the
corner reference gives only 14-18 dB, and the best-matching cells are scattered randomly
over the grid with no spatial structure -- the residual left on screen is the gradient,
not the scene. A 6.3 dB position-dependent swing at exactly the wall range sits on top of
the **+4.4 dB** a real target produced in the 2026-08-28 static A/B.

### Per-cell repeatability drops ~18 dB under the rover

Revisiting the same physical (x,y) one minute apart (`3row2` vs `3row3`, 24 matched cells)
gives **20.3 dB** suppression / coherence 0.9943, against 38.6 dB single-sweep `S_repeat`
and 41.5 dB on the static bench. Magnitude-domain detector noise floor is **0.70-1.13 dB**
versus the static A/B's **0.23 dB** control region -- 3-5x worse.

**Settling is NOT the cause and is ruled out.** The first cell of each row follows a slow
50 mm Y move, the rest follow fast X moves; after removing a smooth 2D fit their residuals
are indistinguishable (-26.8 dB row-start vs -25.6 dB mean for the others, inside the
scatter). So the 100 ms `roverSettleMs` used here was not leaving the structure ringing.

### Adjacent cells are independent, so no image can form

Residual patches correlate **+0.79** between scans at the same absolute position, but only
**+0.11 to +0.23** between rows 50 mm apart within one scan. Each cell is individually
repeatable while its neighbour 50 mm away is an independent sample -- so the plan view is
a field of uncorrelated cells regardless of colour scale. No hyperbola appears in any
B-scan under any of four background strategies (raw, corner `bgRef`, per-row mean
subtraction, 2D polynomial detrend), and after detrending, residual bumps of 1-4 dB sit at
random x and repeat neither between rows nor between scans.

### Also: these scans have NO LiDAR at all

`lidar_standoff_mm` is `null` and `lidar_n` is `0` for all 36 cells in all three files.
So `bgForStandoff()`'s phase alignment was the identity and the BG-model path would have
been dead on arrival (`inferBgModel` needs a standoff). Harmless for a fixed-standoff rail
raster, but it means a model-based background was never an option for these captures --
check `lidar_n` before blaming the model.

### What to try next, in order

1. **Level the rig against the wall.** This is the root cause and by far the cheapest fix:
   `row4` measured a 17 mm standoff span (40.4 mm bottom-left to 23.2 mm top-right) over a
   700 x 150 mm grid. Get that under ~2 mm and the vertical axis behaves like the
   horizontal one already does. The 50 mm pitch is NOT the problem.
2. **Reference per row, not per scan.** Even untilted, a single `bgRef` only holds while
   the standoff does. Capturing one reference per row costs almost nothing and removes the
   dominant residual term.
3. **Or self-reference along the scan line**, the standard GPR move: subtract the per-row
   mean, or SVD out the leading components (`lib/svd.js` still exists and the Aligned /
   SAR / 2D Map panels already do this; it was removed from the C-scan panel). This needs
   no reference capture but will absorb any target broad enough to look like trend, which
   is the likely reason the polynomial detrends here found nothing.
3. Re-run the 2026-08-28 target A/B **on the rail** (target in / target out at one fixed
   cell) before scanning again. It is the only measurement that has ever detected this
   target, it costs two captures, and it separates "the target is invisible to the sensor"
   from "the raster is destroying it".

## `bscanBg.js` phase-alignment has an INVERTED SIGN and is net-harmful (2026-08-30)

Found on `row4` -- a 15x4 C-scan of an EMPTY wall (no target, so every detection is false by
construction) captured with the LiDAR working, background taken at the top-left corner.
Reported symptom: "the bottom rows came out near clean but the upper rows had detections."

### The sign is backwards

`bgForStandoff()` in `groundstation/frontend/src/lib/bscanBg.js` multiplies the reference by
`exp(+j*2*pi*2*deltaD*f/c)` where `deltaD = standoff_cell - standoff_ref`. An echo at
distance `d` is `exp(-j*2*pi*2*d*f/c)`, so that factor moves the background the **wrong way**.
Verified empirically: a synthetic echo placed at 100 mm, given `deltaD = +10 mm`, moves to
**90 mm** when it must move to 110 mm. **The correction therefore applies `-deltaD` instead
of `+deltaD`, doubling the standoff error to `2*deltaD` rather than removing it.**

### It costs 6.5 dB and drives suppression NEGATIVE

Mean suppression over `row4`'s 60 cells, sweeping an alignment strength `alpha` on `deltaD`:

| alpha | -1.0 (sign flipped) | -0.5 | **-0.25 (optimum)** | **0 (no alignment)** | +0.5 | **+1.0 (shipped)** |
|---|---|---|---|---|---|---|
| suppression | 6.41 dB | 10.44 | **11.88** | **10.90** | 6.96 | **4.38** |

Per row with the shipped code, top to bottom: **-0.2, 1.2, 6.6, 10.0 dB** -- the top two rows
go *negative*, i.e. the subtraction adds more energy than it removes. Without alignment the
same rows give 7.6, 9.7, 15.6, 10.6 dB. Even the reference cell itself drops 29.2 -> 20.3 dB,
because its own `deltaD` of 0.8 mm gets doubled to 1.6 mm.

**Fixing the sign is NOT enough and is not the recommended change.** A correctly-signed full
alignment (`alpha = -1`) still scores 6.41 dB, worse than doing nothing (10.90). The reason is
CLAUDE.md's own echo decomposition: the dominant background component sits at alpha ~ 0, a
static coupling reflection that does not move with standoff at all, so shifting the *whole*
spectrum corrupts the largest term in either direction. The optimum `alpha = -0.25` is the
fraction of background energy that actually tracks standoff, and it only buys 1.0 dB over
disabling alignment entirely. **Recommended: disable the alignment (or gate it behind a small
tuned alpha), do not merely flip the sign.**

### Why the top rows and not the bottom

The wall is tilted. LiDAR standoff runs **40.4 mm at bottom-left to 23.2 mm at top-right**;
row means bottom-to-top are 37.2 / 34.5 / 29.2 / 27.0 mm. The reference was captured at
**37.6 mm**, which matches the BOTTOM rows -- so the bottom rows are near-zero mismatch and
clean, while the top rows are 10-14 mm out. Regressing aligned suppression on standoff
mismatch and on lateral distance from the reference cell gives **-1.256 dB per mm of
mismatch, R^2 = 0.827** (correlation -0.910), with lateral distance contributing
**+0.0001 dB/mm, i.e. nothing**. Standoff mismatch is the entire story in this scan.

Note the top-left reference cell is an outlier for its own row (36.8 mm against 23-25 mm
across the rest of it), so "top-left corner" happened to pick a standoff representative of
the far side of the grid. Picking a corner is not a neutral choice on a tilted wall.

## C-scan pipeline changes (2026-08-30)

Three changes, all in the direction of "the pipeline does what the UI says it does".

**1. The captured-reference phase alignment is GONE** (`lib/bscanBg.js` `bgForStandoff`).
It is not a toggle and never was -- it applied itself silently whenever both the cell and
the `bgRef` carried a LiDAR standoff, which is why it only surfaced once the LiDAR was
fixed. The `3row*` scans predate that and were never aligned; `row4` was. See the sign-bug
section above for the measurements. `bgForStandoff` no longer takes `freqs`, and the
now-unused `SPEED_OF_LIGHT` came out of that file with it.

**2. Gate/depth defaults raised to 70 cm** (`App.jsx` `bscanParams`): `gateEnd` 15 -> 70 and
`maxDepth` 30 -> **70**. `maxDepth` was raised deliberately alongside it, not incidentally --
the Max Depth field's own `onChange` clamped `gateEnd` to `maxDepth`, so leaving it at 30
would have silently snapped the new 70 cm gate back to 30 the first time anyone touched that
field. **SUPERSEDED 2026-08-31: `maxDepth` no longer exists in `bscanParams`** -- see the
C-scan imaging section below. The coupling it describes is gone with it; SAR's depth is now
`sarMaxDepth` on the SAR panel.

**3. Rover session is now ARM then SCAN, two presses** (`hooks/useRoverScan.js`,
`CscanPanel.jsx`, `App.jsx` `handleBscanAction`). Start Session starts the sweep and drives
to the grid origin, then parks in a new `'ready'` phase and waits. A separate **Start Scan**
button begins the raster (`roverScan.beginRaster()`, action `'start_raster'`). The reason is
that a background reference is only valid near the position it was taken at, and the old
flow began rastering immediately, so there was no moment at a known position -- with the
sweep already running -- to capture one. `beginRaster()` re-reads `capturedCount` at the
moment it is pressed rather than trusting what arming saw, so capturing or undoing while
parked cannot desync the start index. It is a no-op unless the phase is exactly `'ready'`.

The `'ready'` phase keeps the machine and its 40 ms tick alive, so the link-lost and e-stop
checks at the top of `tick()` still run while parked -- that is the whole reason it does not
just halt. Verified headlessly (React shimmed to four functions, timers stubbed, fake clock):
arming parks at `ready` and stays there through 5 s of ticks without self-starting; Start
Scan then snakes a 3x2 grid top-left downwards in the correct order and finishes `done` with
6 cells; `beginRaster()` during `homing` is a no-op; e-stop and link-loss while parked both
abort and stop the sweep.

### Known hidden behaviour still in the C-scan path (audited 2026-08-30, NOT changed)

- ~~`alignShifts` dead code~~ **REMOVED 2026-08-30** (see below). Everything else in this
  list was audited and deliberately left alone.
- **The C-scan recomputes its own range profiles and throws the Pi's away.**
  `applyBscanBg` calls `computeRangeProfile` for every cell *even when background
  subtraction is off*, replacing `magnitudes`/`distances` from the wire. That profile is a
  **rectangular-windowed, 4x zero-padded IFFT with no range compensation and no averaging**
  -- so the C-scan and its B-scan pane do not match the SFCW panel's live display, whose
  window / Kaiser beta / R^n / Avg controls do not exist here.
- **The rover raster silently discards one sweep per cell** (`skip: 1`), because a sweep in
  flight when the settle expires started while the gantry was still moving. Costs ~250 ms a
  cell and is invisible on screen.
- **The BG *model* path still does a phase unwind** -- `lib/bgModelInterp.js`
  `UNWIND_ALPHA = 0.80`, applied on build and rewound on inference. This is a different
  animal from the deleted `bgRef` alignment (it is fitted across many knots rather than
  extrapolating one capture, and it measured 20.2 dB LOO) but it is equally invisible, and
  `inferBgModel` still **clamps silently** outside the model's standoff span.
- **`metric: 'energy'` is a mean, not a sum** (`cscanGrid.js` `gatedIntensity` returns
  `10*log10(sumLin/count)`), so it does not scale with gate width the way the name implies.

## C-scan imaging: one shared scale, per-cell BG status, magnitude mode, Super Fit (2026-08-31)

### The two panes were scaled to different things

Audit finding, now fixed. The C-scan grid took dynamic colour limits from the
**min/max of the gated cell values over the whole grid**; the B-scan pane took
them from **min/max of every bin in the SELECTED ROW ONLY** -- and included the
unsubtracted BG reference row in that. Consequences: the same colour meant a
different dB in two images shown side by side, clicking to another row silently
re-scaled the right-hand one, and with a background loaded the BG row (20-30 dB
above every residual) set `dbMax` on its own and crushed the actual data into the
bottom few percent of the colormap -- a **working subtraction looked empty**.

`computeSharedScale()` (`lib/cscanGrid.js`) now computes **one** pair of limits
from every bin of every valid cell -- exactly the pixels the B-scan pane draws --
and both displays use it. The BG row is still drawn but no longer votes.

**Limits are percentiles (p1 / p99.9), not min/max.** A range profile has deep
interference nulls and a subtracted one has more; a single bin at -140 dB would
otherwise set the bottom of the scale and flatten everything above it. A full
min/max stretch of a flat residual field is exactly the mechanism that
manufactures rainbow structure out of noise (see the display hypothesis in the
target A/B section above). There is a degenerate guard: subtracting a Super Fit
reference from the grid it was taken from gives exactly zero in every bin, which
is a useful "same data" signal, not an error.

### A cell whose background fails is INVALID, not zero

`applyBscanBg` used to fall through silently when `bgForStandoff` returned null
(no lidar standoff, `numSteps` mismatch), leaving that cell **un-subtracted in a
subtracted grid** -- 20-30 dB above its neighbours, so it both read as the
strongest target in the scan and single-handedly set the dynamic colour limits.
There was no diagnostic anywhere in the C-scan path; the SFCW panel got
`sfcwProcessed.diag` in Phase 0 and this never did. Recall the `3row*` scans had
`lidar_n: 0` on all 36 cells, so a model background would have failed on every
one of them.

Every position now carries a `bg_status` (`BG_STATUS` in `cscanGrid.js`):
`off` / `ok` / `clamped` / `no_standoff` / `no_ref` / `size_mismatch` /
`no_superfit_cell`. Invalid cells draw as a red cross on a dark red ground (a
colour no colormap produces), are excluded from every scale, and are counted in
an Applied / Clamped / Invalid readout in the panel.

**`clamped` is new information too.** `inferInterpModel` clamps to the nearest
knot and returns a confident-looking spectrum; the model span was previously
only checked against the *live* lidar, never against what a cell was actually
captured at. Clamped cells are drawn but carry an amber corner.

### Complex vs magnitude, in the C-scan

The toggle the SFCW panel already had. Complex is for **seeing** (removes the
wall so a target 16.6 dB beneath it is not buried); magnitude is for
**deciding** (+4.4 dB at 21.2 cm against a 0.23 dB control in the target A/B,
and it survives ~1 mm of standoff error, which the complex difference does not).
Neither is a better version of the other -- do not delete one again.

- Magnitude mode transforms BOTH spectra and differences the dB profiles.
  `h_cal_real/imag` are left **raw**, because a dB difference is not a spectrum
   -- so **SAR must stay on complex**, and it does: `processedBscanData` is
  complex-only and feeds SAR and the 2D Map, while `cscanProcessedData` carries
  the mode and feeds only the two C-scan panes (it aliases the complex result
  when the mode is complex, so nothing is computed twice).
- `gatedIntensity` needed **no change**: `lin*lin` with `lin = 10^(db/20)` is
  exactly `10^(db/10)`, so the same expression is a mean power in absolute mode
  and a mean **power ratio** in difference mode.
- The linear colour warp is bypassed in difference mode (it maps an amplitude;
  a dB ratio has none) and the colour bar gets a zero line and signed labels.
- Measured on a synthetic two-echo scene: **the largest |Δ| does not land
  bin-exactly on the target and is often NEGATIVE.** A target on the skirt of a
  much larger wall return interferes with it, so the peak lands where that
  interference is strongest -- 0.330 m for a target at 0.300 m, within the
  50 mm range resolution. Read |Δ| against a threshold, not the sign, and do not
  expect ranging from it.

### Super Fit: a whole reference GRID, matched cell for cell

A single captured reference is only right at one position. The 2026-08-30 rover
diagnosis measured a **17 mm standoff span over one 700x150 mm grid** (the rig is
not parallel to the wall), a 6.3 dB position-dependent swing at exactly the wall
range, and only 14-18 dB from a corner reference. Super Fit stores every cell of
a completed grid and subtracts each new capture from **the reference at its own
(grid_ix, grid_iy)**, so a standoff that varies across the grid is matched rather
than extrapolated.

Workflow: scan (or import) the bare wall, press **Super Fit This Grid**, clear
the grid, rescan the same wall from the same origin.

- **Keyed by cell index, never by capture order.** Manual snakes up from the
  bottom-left and the rover snakes down from the top-left, so the two orders
  visit the same cells in different sequences; matching on order would subtract
  every cell against the wrong patch of wall. Verified both orders produce an
  identical grid.
- **Grid geometry is locked while a Super Fit is loaded**, for the same reason --
  changing a count or a step silently re-keys every cell. The panel also warns if
  the geometry no longer matches what was captured.
- Mutually exclusive with the captured reference and the model, like they are
  with each other. `bscanBgDisplay` is deliberately **null** under Super Fit:
  its reference is a different spectrum per cell, so no single BG row represents
  it.
- Requires a **full** grid. A partial reference would leave cells with no
  background, and those are now refused rather than passed through raw.

Measured on a synthetic tilted wall (3x2 grid, wall face walking 4 mm per column
and 10 mm per row, one target 22 dB below the wall face in one cell):

| background | target cell | brightest other cell |
|---|---|---|
| corner captured reference | -23.2 dB | **-13.2 dB** (target is not the brightest) |
| **Super Fit** | -40.0 dB | **-101.7 dB** (61 dB of contrast) |

### The Live Sweep controls bar now drives the C-scan (2026-08-31)

The bar on top of the C-scan viewport used to affect only that pane -- its Window
and Avg settings never reached the grid, so the panel showed a range profile
processed differently from the two images under it. It is now the C-scan's
processing surface. `SfcwDisplay` gained optional `procParams` /
`onProcParamsChange` (controlled `{windowType, kaiserBeta, avgCount, avgMode}`),
`procLocked`, and `hideRangeComp` / `hideFloor` / `hideCfar` / `hideYMode`. Omit
them all and the component behaves exactly as before, which is what the SFCW
panel's own instance does.

**Four controls were removed from the C-scan's instance**, each because it could
only disagree with the image beside it:
- **R^n** applies the same gain to every cell and to the background alike, so it
  cancels out of every comparison a C-scan makes.
- **FLOOR** is estimated from this pane's own rolling sweep history, which
  describes the live sweep, not the recorded cell being drawn.
- **CFAR** is a per-trace detector nothing in the grid pipeline reads.
- **Y session/frame**: 'session' extremes accumulated over a whole raster would
  leave the axis set by whichever cell was loudest. Pinned to 'frame'.

Hidden controls are forced OFF rather than merely not rendered, so a stale
default cannot keep applying itself.

**The bar locks while a session runs** (`sfcwRunning || roverScan.active`).
`avgCount` genuinely cannot change part-way through a raster -- different cells
would hold different numbers of sweeps -- and the window is locked with it so
every cell in one grid is processed identically. Everything unlocks when the
session stops and then re-derives the whole grid live.

### Windowing is a display parameter; Avg is a capture parameter

- **Window / Kaiser beta** re-window every stored cell on change, so they can be
  moved freely over already-captured or imported data. `rangeProfile.js`
  `computeRangeProfile` takes an optional taper (from `imagingEffects.js`
  `windowFn`, the same one the SFCW display and Imaging Bench use). Measured on a
  single synthetic echo: sidelobes rel. peak **rect -18.0 dB, Kaiser beta 3 -29.1,
  Hanning -31.5**, with -3 dB mainlobes of **5 / 5 / 7 bins**. Rectangular stays
  the default -- Hanning's wider mainlobe would swallow a target 7 cm from the
  wall face, which is exactly the case here.
- **Avg** is how many sweeps are taken at each grid cell, so it only affects
  captures made after it is set.

### Every sweep of a cell is stored, so coh/inc stays live

A cell captured with Avg > 1 keeps **all** its sweeps in `pos.sweeps`, not just
the average, because coherent-vs-incoherent is a *display* choice and has to
remain flippable against recorded data. `pos.h_cal_real/imag` is the **coherent
mean** (not the last sweep -- that was a bug in the first draft), so everything
that reads `h_cal` without knowing about `sweeps` -- SAR, the BG-model trainer,
Super Fit, `svdFilter`, the export -- sees the averaged cell. `cellSweeps()`
falls back to the single spectrum, so a pre-v7 record is exactly an N=1 cell;
verified byte-identical output.

**Order of operations is deliberate: the background is subtracted from each
SWEEP, before averaging, not from the average.** For coherent averaging the two
are identical (both linear); for incoherent they are not -- averaging `|signal|`
first and subtracting a complex background afterwards is not a defined
operation, whereas subtracting per sweep and then averaging magnitudes is
exactly "N independent looks at the residual".

Measured (synthetic, 16 looks): coherent averaging lowers the residual peak by
**-13.7 dB against an ideal -12.0**. At a bin far above the noise floor coherent
and incoherent agree to **0.1 dB**, and at a null sitting at the floor incoherent
reads **1.6 dB high** -- the noise bias, and the entire difference between the
two modes. Note the SIZE of that gap is bounded by how far the null sits below
the single-look floor, and more looks do NOT widen it (checked at N=64): once
coherent has driven its own floor below the null, what remains is fixed by the
incoherent floor. The direction is the invariant, not the magnitude.

Cell-level provenance is pooled over the sweeps actually taken (mean standoff,
summed `lidar_n`), so a multi-sweep cell reports the standoff it was really
measured at rather than whichever sweep landed last -- the BG model is evaluated
at that number.

**SAR is forced to `avgMode: 'coherent'`** regardless of the toggle: it
back-projects complex data, so an incoherently averaged magnitude profile is not
an input it can use. It does follow the window.

**The rover capture watchdog scales with Avg.** `useRoverScan`'s flat
`CAPTURE_TIMEOUT_MS = 20000` was plenty at one sweep per cell and would have
tripped part-way through an Avg of 16; it is now
`20 s + 2 s * (sweepsPerCell - 1)`.

**Export is v7** (`sweeps` + `procParams`). Import restores `windowType`,
`kaiserBeta` and `avgMode`, but reads `avgCount` back from the data rather than
the header -- the sweeps are in the file and their number is whatever was
actually taken.

### Also fixed: Manual scaling seeded from the wrong data

`seedManualRange()` rebuilt the grid from `scanData`, which the Sidebar passes as
the **raw** capture list -- before background subtraction, and computed by the Pi
with a **Hanning window at nfft 204** where the display uses a **rectangular
window at nfft 256** (~4 dB apart for a single tone, on a different bin grid).
The seeded limits were wrong by the full suppression plus that offset, so the
colours jumped hard on every switch to manual -- the exact thing the comment
above it promised they would not. It now reads the shared scale the displays draw
with.

### Still true, and still worth knowing

- **The C-scan recomputes its own range profiles and throws the Pi's away**,
  rectangular / 4x zero-pad / no range compensation / no averaging, even when
  subtraction is off. The SFCW panel's Window / Kaiser / R^n / Avg / CFAR / FLOOR
  controls are `SfcwDisplay` local state and do **not** exist here. Note the
  window trade is not obvious: rectangular has -13 dB sidelobes but a ~9.8 cm
  null-to-null mainlobe, Hanning gets -31 dB sidelobes for a ~19.5 cm mainlobe --
  which would swallow a target 7 cm from the wall. Expose and A/B it against a
  target in/out capture; do not assume Hanning is the upgrade.
- **The Live Sweep pane at the top of the C-scan viewport uses the SFCW panel's
  background** (`sfcwBgModel`/`sfcwBgRef`), not the C-scan's. The two can
  disagree silently.
- **`maxDepth` is gone from the C-scan panel (2026-08-31).** It did two unrelated
  jobs: clipping the B-scan pane's display, and bounding SAR's reconstruction
  grid. The first was pointless -- the default clipped 70 cm off a ~74 cm record,
  so it hid the far end of the data and bought nothing, while looking like a
  depth control next to the Depth Slice gate, which is the actual one. The B-scan
  pane and the shared scale now cover the whole profile unconditionally. The
  second is a real parameter and moved to the **SAR panel** as `sarMaxDepth`
  (same 70 cm default), since it sets the extent and cost of the reconstruction
  rather than what a display shows. The Depth Slice gate's upper bound is now
  derived from the record itself (`c/(2*step)/2 - range_offset`, read off a
  captured profile) instead of tracking a second field. Import still reads a v3
  `wallThickness` / v4+ `maxDepth` and restores it into `sarMaxDepth`.
- Default gate is 2-70 cm with `metric: 'peak'`. With the wall at 13-14 cm,
  `max()` over that gate picks the wall in every cell -- the plan view is then a
  wall-strength map, which is precisely the gradient the rover scans showed.
  Narrow the gate around the expected target depth and prefer `energy`.
- `metric: 'energy'` is a mean, not a sum, so it does not scale with gate width.

### Verification

`lib/cscanGrid.js` and `lib/bscanBg.js` are pure and were exercised head-first
from node (21 checks: Super Fit self-subtraction exact-zero, target isolation
against a tilted wall, capture-order independence, magnitude-mode difference and
range, all six `bg_status` paths, invalid-cell exclusion from the shared scale,
percentile clipping, degenerate guard). There is no test runner in this repo, so
those were throwaway scripts -- worth rebuilding as real tests if this grows.

**Fixture trap worth remembering:** synthetic echoes must be placed at
`depth + range_offset` (0.5 m by default). Placing them at the intended display
depth puts them at negative distance, where `computeRangeProfile` drops them, and
every subsequent measurement is of sidelobes only -- which still looks plausible.

### The third standoff-alignment path, removed 2026-08-30

Separate from the `bgRef` phase ramp: `App.jsx` carried a ~45-line `alignShifts` `useMemo`
computing per-position range-bin shifts from LiDAR standoff, handed to `Viewport` as
`bscanAlignShifts`, declared as a prop there and **read by nothing**. Its consumer half
lived in `BscanDisplay` -- a per-frame lerp toward `targetShifts`, which `Viewport.jsx` fed
`rowData.map(() => 0)`, i.e. hard-wired to zero. So the display animated toward zero forever
and `getRowPixelOffset()` always returned 0.

The whole path is gone: the `useMemo`, both props, `getRowPixelOffset`, the `pxOffset` terms
in the two x-coordinate expressions, the two shift refs and the lerp. **Provably
behaviour-neutral** -- the offsets were always exactly 0.

Two things worth keeping in mind about what was left behind:
- **The rAF loop in `BscanDisplay` stays, deliberately.** With the lerp gone there is
  nothing animated, so a one-shot draw looks tempting -- but `drawBscan` sizes itself from
  `getBoundingClientRect()`, and redrawing every frame is what makes the canvas track a
  panel resize. Replacing it with a draw-on-change effect needs a ResizeObserver first.
- It also fixed a latent bug: `targetShifts={rowData.map(() => 0)}` built a fresh array
  every render, so that effect's dependency changed identity every render and tore down and
  restarted the rAF loop each time. It now re-runs only when something real changes.
