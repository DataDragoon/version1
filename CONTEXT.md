# version0 — SFCW Radar Wall Imaging System

## Mission

Within-wall imaging using a Stepped-Frequency Continuous Wave (SFCW) radar.
The goal is to image what is inside the wall (rebar, pipes, voids, studs),
not what is beyond it.

---

## Hardware

| Component | Model | Interface | Role |
|-----------|-------|-----------|------|
| Compute | Raspberry Pi (with AI HAT+) | — | On-board control, sensor fusion, data capture |
| LiDAR | TF-LC02 | UART (serial) | Range/distance reference |
| IMU | BNO085 (was MPU-6500 until 2026-08-24) | I2C | Orientation, acceleration, gyro |
| SDR | bladeRF | USB | SFCW radar TX/RX |
| Antennas | 2x Vivaldi | SMA to bladeRF | Wideband TX and RX |
| Rover | Stepper gantry + Arduino UNO | WebSocket over LAN | 2-axis positioning of the radar head |
| Network | Ethernet/WiFi | LAN | Pi <-> PC link |

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                        LAN                                   │
│                                                             │
│  ┌─────────────────────┐          ┌──────────────────────┐ │
│  │   Raspberry Pi       │          │   PC (Groundstation) │ │
│  │                     │          │                      │ │
│  │  - Sensor capture   │  ◄────►  │  - Control panel     │ │
│  │  - Radar TX/RX      │  socket  │  - Debug tools       │ │
│  │  - Data streaming   │          │  - Heavy processing  │ │
│  │                     │          │  - 3D visualization  │ │
│  └─────────────────────┘          └──────────────────────┘ │
└─────────────────────────────────────────────────────────────┘
```

## Operational Model

- **No direct input to the Pi.** All commands originate from the groundstation.
- **Pi streams data** (raw IQ, sensor logs) to groundstation.
- **Heavy processing** (image reconstruction, SAR focusing) runs on the PC.
- **Debug everything.** Every subsystem has a dedicated debug view on groundstation.

## Groundstation Debug Tools (planned)

- LiDAR distance log + live plot
- IMU orientation/accel live view
- 3D position visualizer (fused estimate)
- Radar TX/RX pattern viewer
- Raw IQ waterfall / spectrogram
- SFCW range profile display
- SAR image reconstruction view
- System health / link status

## Groundstation Control Panel (planned)

- Initiate scan
- Stop / pause / resume
- Configure radar parameters (freq range, step size, dwell time)
- Configure sensor sampling rates
- Trigger calibration routines
- Data recording start/stop

## Directory Structure

```
version0/
├── pi/                    # Code that runs on the Raspberry Pi
│   ├── sensors/           # LiDAR, IMU drivers/readers
│   ├── radar/             # bladeRF SFCW control
│   ├── rover/             # Stepper gantry control + position tracking
│   ├── comms/             # Network transport to groundstation
│   └── scripts/           # Startup, calibration, utilities
├── groundstation/         # Code that runs on the PC
│   ├── ui/                # Main GUI framework
│   ├── debug/             # All debug/visualization tools
│   ├── control/           # Command panel (start/stop/config)
│   ├── processing/        # Heavy compute (SAR, image recon)
│   └── comms/             # Network transport to Pi
├── shared/                # Code used by both Pi and PC
│   ├── protocols/         # Message formats, command definitions
│   └── config/            # Shared configuration constants
├── docs/                  # Additional documentation
├── CONTEXT.md             # THIS FILE — project global context
└── CLAUDE.md              # Claude Code project instructions
```

## Network Protocol (TBD)

Communication between Pi and groundstation. Likely ZeroMQ or raw TCP sockets
with a simple framed binary protocol. Requirements:
- Low-latency command delivery (groundstation -> Pi)
- High-throughput data streaming (Pi -> groundstation)
- Multiplexed channels (IQ data, sensor data, status)

## Radar Parameters

- Hardware: bladeRF xA9 (AD9361 RFIC)
- Frequency range: 1–3 GHz (configurable, max ~3.8 GHz)
- Step size: 10 MHz default
- Dwell time per step: 1 ms (PLL settle)
- TX power: 1.0 amplitude, gain 50 dB (RF Calib panel default; RX gain 25 dB, center freq
  2000 MHz, sample rate 10 Msps — see `BladeRFDriver.__init__` / `RfCalibPanel.jsx`)
- Antenna polarization: co-pol initially
- Phase coherence: Dual-channel reference method — TX2→RX2 short SMA cable
  provides phase reference. Signal (RX1) divided by reference (RX2) cancels
  random PLL phase offsets between TX and RX synthesizers at each step.
  AD9361 single-synth mode does NOT work (FDD requires both PLLs active).

## Wiring — BNO085 (I2C mode)

Replaced the MPU-6500 on 2026-08-24 (same I2C1 bus). Confirmed by probe: I2C address
`0x4A` (`i2cdetect -y 1`), SCL/SDA on the same Pin 5 / Pin 3 (GPIO 3 / GPIO 2) I2C1 bus the
MPU-6500 used. **Exact VCC/GND header pins and whether RST/PS0/PS1 are wired to anything on
the Pi are not confirmed** — no GPIO shows as driving a reset line for it (`gpioinfo`), so
either those pins are strapped on-board (normal for most BNO085 breakouts, which fix
PS0/PS1 for I2C app mode and leave RST pulled up) or genuinely floating. During bring-up the
chip briefly got stuck in a state where control-channel queries worked but sensor feature
reports wouldn't enable — a power cycle fixed it (see the CLAUDE.md IMU section) — so it
does need real power, not just I2C soft reset, to leave that state. Fill in the actual VCC
pin and RST/PS status here next time the wiring is physically checked instead of leaving
this as a known gap.

## Wiring — TF-LC02 LiDAR (UART)

**Moved to UART3 (2026-08-24) after UART0's receiver was found dead** — see CLAUDE.md's
LiDAR silent-serial investigation for the full diagnostic trail (loopback + `TIOCGICOUNT`
testing isolated it to UART0's RX peripheral specifically, not the module, not the wiring,
not the Pi's GPIO pins themselves). VCC is 3.3V and not shared with the IMU, confirmed
correct and unchanged throughout.

| TF-LC02 Pin | Raspberry Pi | Notes |
|---|---|---|
| VCC | 3.3V rail | Not shared with IMU |
| GND | Pin 6 (GND) | Common ground |
| TX | Pin 21 (GPIO 9 / RXD3) | LiDAR TX → Pi RX |
| RX | Pin 24 (GPIO 8 / TXD3) | Pi TX → LiDAR RX |

`uart3-pi5` overlay enabled in `config.txt` (`uart0-pi5` disabled, left commented rather than
removed). Device: `/dev/ttyAMA3` (was `/dev/serial0`/`ttyAMA10` — do not revert to that, its
receiver is dead). Serial console disabled.

## LiDAR → Antenna Offset (measured 2026-08-28)

**165 mm measured; 160 mm used** (5 mm buffer so a true zero-standoff pose reports
slightly positive). With the antenna aperture placed against the wall, the TF-LC02 reads
**164.83 mm ± 0.68**. Standoff = `lidar_reading − offset`, so real operation spans a lidar
reading of roughly **165–315 mm** for 0–150 mm of standoff.

The value was hardcoded at 315 mm in `App.jsx` until 2026-08-28 and did not match this
mounting. It is now App.jsx state, persisted to `localStorage.lidar_antenna_offset_mm` and
editable in the SFCW panel's Standoff section. **Re-measure it after any re-mount** — put
the aperture against the wall, read the lidar, subtract 5. Background models record the
offset they were built under (`geometry.lidarAntennaOffsetMm`) and the panel warns when a
loaded model disagrees with the current setting. See CLAUDE.md's background-subtraction
section for why a *constant* offset error cancels but a *changed* one does not.

## IMU Calibration & Orientation

**Stale as of 2026-08-24 — measured for the MPU-6500, not re-verified for the BNO085 that
replaced it.** Confirmed on the bench: the BNO085's raw gravity reading lands on a different
axis than the MPU-6500's did, so the mapping below is known wrong until re-run through the
calibration discovery tool. See `imu_calibration.py`'s module docstring and the CLAUDE.md
IMU section for specifics.

MPU-6500 mounting orientation (determined via calibration tool):
- IMU +X = physical UP (gravity reads +1g on X when level)
- IMU Y = pitch axis (pitch down = gyro -Y)
- IMU Z = roll/forward axis (roll right = gyro +Z)

Body frame convention (right-hand):
- Body X = FORWARD
- Body Y = LEFT
- Body Z = UP

Data sent over WebSocket (port 9001) is in body frame:
- `accel`: [forward, left, up] in g
- `gyro`: [roll_rate, pitch_rate, yaw_rate] in deg/s
- Positive: roll right, pitch up, yaw right

Startup calibration: 2s stationary capture → gyro/accel bias saved to `pi/sensors/imu_cal.json`.
Use `--skip-cal` flag on `stream.py` to reuse previous calibration.

## Rover — Stepper Gantry

Arduino UNO R4 WiFi + CNC Shield V3, four A4988-class drivers, firmware in `rover/`.
Two axes only — **X = left/right**, **Y = up/down**. There is no standoff (toward-wall)
axis, so standoff is still set by hand and measured by the LiDAR.

| axis | shield sockets | mechanism | resolution | travel | max / jog speed |
|---|---|---|---|---|---|
| **Y** vertical | X | leadscrew, 2 mm × 4-start = 8 mm/rev | **200.0 steps/mm** exact (5 µm) | 1 m, soft-limited 150–850 mm | 25 / 15 mm/s |
| **X** horizontal | Y + Z + A (three ganged) | 66 mm drive wheels, rolling | **7.7166 steps/mm** (130 µm) | 4 m, soft-limited 0–3900 mm | 150 / 60 mm/s |

1600 steps/rev (200-step motors at 1/8 microstepping). Note the socket labels do not match
the axes: the shield's *X* socket drives the *vertical* axis. Scans typically span ~100 mm.
Both axes need `*_DIR_INVERT` set in `rover/config.h` — verified on the rig 2026-08-29, and
magnitudes were correct without any wheel calibration.

**No endstops and no encoder.** Soft limits are the only travel protection, and the
operator declaring the position is the only ground truth. The horizontal axis rolls on
wheels, so slip is possible in principle (judged unlikely in practice); the odometer since
the last declared position is reported as the exposure.

The Arduino is a **WebSocket client**: it dials into the Pi on port 8765 and the Pi is the
server. The link speaks line JSON with sequence numbers — `move` / `jog` / `jog_hold` /
`stop` / `estop` / `clear_estop` / `set_pos` / `cfg` / `enable` outbound, and
`hello` / `status` (20 Hz) / `ack` / `done` / `err` inbound. **Everything on the wire is in
STEPS**; millimetres exist only on the Pi, which holds the calibration.

Step pulses come from a 20 kHz timer ISR so that WiFi servicing cannot disturb them, which
is what lets the board stay responsive during motion — continuous jog, E-stop mid-move and
live position all depend on it.

WiFi credentials are in `rover/secrets.h`, gitignored; `rover/secrets.example.h` is the
committed template.

The board's radio MAC is **04:CF:4B:B5:15:03** (Espressif — the R4's WiFi is an ESP32-S3),
reserved at **192.168.1.11** on the router alongside the Pi's own reservation. The firmware
prints its MAC at boot in both byte orders, since this library family fills the array
backwards. A reservation makes the address deterministic but does **not** guarantee the DHCP
exchange itself succeeds — `linkReady()` in the firmware is what makes a failed lease
recoverable, and `USE_STATIC_IP` in `config.h` skips DHCP entirely if it is ever needed.

Firmware layout: `config.h` (pins, mechanism, defaults), `motion_core.h` (ramp, limits,
watchdog — Arduino-free and unit-tested), `protocol_core.h` (JSON — likewise), `rover.ino`
(pins, timer, sockets, flash persistence). `rover/test/build_check.sh` runs the native
tests and type-checks the sketch; `pi/rover/rover_sim.py` stands in for the board so the
Pi and groundstation are testable without the rig.

## Network Ports

| Service | Port | Protocol | Direction |
|---------|------|----------|-----------|
| Sensor stream (IMU + LiDAR) | 9001 | WebSocket | Pi → Browser |
| Rover control + position | 9002 | WebSocket | Pi ↔ Browser |
| Rover ← Arduino UNO link | 8765 | WebSocket | UNO → Pi (the UNO dials in) |
| SDR control + IQ stream | 9003 | WebSocket | Pi ↔ Browser |
| Groundstation UI | 5000 | HTTP | PC local |

## Current Status

- [x] Project scaffolded
- [x] Context documented
- [x] Hardware connections (IMU + LiDAR wired and tested)
- [x] IMU driver (BNO085 over I2C, was MPU-6500)
- [x] LiDAR driver (TF-LC02 over UART)
- [x] Combined sensor WebSocket stream (port 9001)
- [x] Groundstation UI — IMU + LiDAR debug panel
- [x] IMU calibration (gyro bias + accel bias at startup, persisted to imu_cal.json)
- [ ] IMU axis remapping (IMU frame → body frame: forward/left/up) — done for MPU-6500,
      confirmed wrong for BNO085, needs re-discovery
- [x] Madgwick AHRS orientation filter (quaternion-based, groundstation 3D view)
- [x] IMU calibration discovery tool (groundstation panel)
- [x] BladeRF driver + AquaSense calibration panel (signal generator + oscilloscope)
- [ ] BladeRF SFCW implementation
- [x] Rover firmware rewrite (ISR stepping, JSON protocol, E-stop, soft limits, calibration)
- [x] Rover jog control + position tracking from the controller's own step counter
- [x] Rover automated grid raster (drives the C-Scan panel's grid; Scan Mode = Rover)
- [ ] Network protocol (formal)
- [ ] Integration testing
- [ ] SAR image reconstruction

---

## Maintenance Rules

**This file and CLAUDE.md must be kept up to date by Claude (or any AI assistant)
as the project evolves.** Whenever a session produces key information — design
decisions, hardware discoveries, protocol specs, calibration data, wiring
pinouts, architectural changes, or anything a future session would need — update
these files before the session ends. They are the persistent memory of this
project across sessions, collaborators, and machines.
