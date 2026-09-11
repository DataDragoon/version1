// Rover firmware configuration -- pins, mechanism, and defaults.
//
// Everything here is a compile-time DEFAULT. Speeds, accelerations and soft
// limits are all settable at runtime from the groundstation and persisted, so
// changing how the rig behaves does not mean reflashing. Only the pin map and
// the mechanism geometry are genuinely fixed by hardware.
#pragma once

// ── CNC Shield V3 pin map ───────────────────────────────────────────────────
// The shield's four driver sockets are labelled X/Y/Z/A. Which socket drives
// which AXIS is a wiring fact, recorded below under "axis assignment".
#define PIN_X_STEP   2
#define PIN_X_DIR    5
#define PIN_Y_STEP   3
#define PIN_Y_DIR    6
#define PIN_Z_STEP   4
#define PIN_Z_DIR    7
#define PIN_A_STEP  12
#define PIN_A_DIR   13
// Shared driver enable for all four sockets, ACTIVE LOW.
#define PIN_ENABLE   8

// ── Axis assignment ─────────────────────────────────────────────────────────
// Confirmed on the rig 2026-08-28 by driving each field and watching the head:
//
//   VERTICAL   (up/down)     <- shield socket X          -- leadscrew
//   HORIZONTAL (left/right)  <- shield sockets Y, Z, A   -- three ganged wheels
//
// Note this is the opposite of what the original stepper_testrig2.py header
// claimed. Trust this comment; it was measured.
#define AXIS_V 0
#define AXIS_H 1
#define NUM_AXES 2

// ── Mechanism ───────────────────────────────────────────────────────────────
// 200-step motors at 1/8 microstepping.
#define STEPS_PER_REV 1600.0f

// Vertical: 2 mm pitch x 4 start = 8 mm of travel per revolution.
#define V_MM_PER_REV 8.0f
#define V_STEPS_PER_MM (STEPS_PER_REV / V_MM_PER_REV)          // 200.0 exactly

// Horizontal: 66 mm drive wheels (measured; an older comment said 70 -- it was
// wrong). This one is not exact and is expected to be CALIBRATED at runtime:
// drive a known long distance, measure it, and scale. The groundstation has a
// workflow for that, and the corrected value lives on the Pi.
#define H_WHEEL_DIAMETER_MM 66.0f
#define H_MM_PER_REV (3.14159265f * H_WHEEL_DIAMETER_MM)       // 207.345
#define H_STEPS_PER_MM (STEPS_PER_REV / H_MM_PER_REV)          // 7.7166

// ── Direction sense ─────────────────────────────────────────────────────────
// The firmware defines the coordinate frame for the whole system: POSITIVE
// steps mean UP on the vertical axis and RIGHT on the horizontal one, matching
// the groundstation's +X-right / +Y-up convention (which in turn matches the
// C-scan grid, whose vertical index grows upward).
//
// These two flags are the only place that convention meets the wiring. The old
// firmware drove UP on a negative value and LEFT on a negative value, so the
// vertical sense is flipped here and the horizontal one is not. Previously this
// lived on the Pi as invert_x/invert_y; it belongs here, so that step counts on
// the wire mean the same thing to everyone.
// Both verified on the rig 2026-08-29 by nudging each axis and watching the
// head. Y was correct as shipped; X was reversed and is now inverted too.
#define V_DIR_INVERT true
#define H_DIR_INVERT true

// ── Motion defaults, in mm ──────────────────────────────────────────────────
// Chosen against the real travel (vertical 1 m, horizontal 4 m) so that the
// stop distance stays small next to a ~100 mm scan span. Stop distance is
// v^2/(2a): vertical 3.1 mm at full speed and 1.1 mm at jog speed, horizontal
// 22.5 mm and 3.6 mm.
//
// Jog speeds were raised from 5/20 on 2026-08-29 -- held keys felt sluggish next
// to a nudge, which runs at the much higher max speed. Use a nudge for fine
// placement; the jog is for getting somewhere.
#define V_MAX_SPEED_MM_S   25.0f
#define V_JOG_SPEED_MM_S   15.0f
#define V_ACCEL_MM_S2     100.0f

#define H_MAX_SPEED_MM_S  150.0f
#define H_JOG_SPEED_MM_S   60.0f
#define H_ACCEL_MM_S2     500.0f

// ── Soft limits, in mm ──────────────────────────────────────────────────────
// There are no endstops on this rig, so these are the ONLY thing standing
// between a jog and the end of the rail. They are enforced here as well as on
// the Pi deliberately: the Pi can crash or lose its link, the board cannot.
// Defaults keep a buffer inside the true mechanical travel.
#define V_MIN_MM 150.0f      // set from the rig's usable travel, 2026-08-29
#define V_MAX_MM 850.0f
#define H_MIN_MM   0.0f
#define H_MAX_MM 3900.0f     // 4 m of travel, 100 mm of headroom

// ── Timing ──────────────────────────────────────────────────────────────────
// Step pulses are generated in a fixed-frequency timer ISR, so WiFi servicing
// in the main loop can never disturb them -- the whole reason the previous
// firmware had to go deaf while moving.
//
// 20 kHz gives 50 us per tick. The budget is set by digitalWrite, which costs
// roughly a microsecond on this core: a worst-case tick writes 4 step pins low,
// 4 high (the horizontal axis has three ganged motors) and up to 2 direction
// pins, so ~10 us of a 50 us budget. Direct port writes would allow a faster
// ISR; that is the upgrade path if step timing ever proves rough.
//
// The fastest axis needs 5000 steps/s (vertical at 25 mm/s) = 4 ticks per step,
// so an individual interval can quantise by up to 25%. The phase accumulator
// keeps the AVERAGE rate exact regardless, and average rate is what position
// depends on -- a stepper's mechanical time constant absorbs the rest.
#define ISR_HZ 20000.0f

// Ramp updates (accelerate / decelerate / limit checks) run inside the same ISR
// but only every ISR_HZ/RAMP_HZ ticks, so the trapezoid costs little.
#define RAMP_HZ 1000.0f

// ── Safety ──────────────────────────────────────────────────────────────────
// A continuous jog runs until told to stop, so a lost link must not leave the
// rover driving. The groundstation refreshes the jog while a key is held; if
// refreshes stop arriving for this long the board decelerates on its own.
#define JOG_WATCHDOG_MS 500

// Below this speed a decelerating axis is simply stopped, and a move still
// short of its target creeps at this rate rather than approaching it
// asymptotically -- that is what guarantees it lands on the exact step.
#define MIN_SPEED_STEPS_S 20.0f

// A static IP sidesteps DHCP entirely. Uncomment and set these if the board ever
// associates but fails to get a lease -- the classic symptom is that it works
// until it is power-cycled and then only comes back after a ROUTER restart,
// because the AP is still holding a stale lease for its MAC. Pick an address
// outside the router's DHCP pool.
//
// #define USE_STATIC_IP
#define STATIC_IP      192, 168, 1,  77
#define STATIC_GATEWAY 192, 168, 1,   1
#define STATIC_SUBNET  255, 255, 255, 0
#define STATIC_DNS     192, 168, 1,   1

// How long to wait for a DHCP lease before treating the attempt as failed and
// starting over. Associating is not the same as being on the network.
#define DHCP_TIMEOUT_MS 12000

// ── Network recovery ladder ─────────────────────────────────────────────────
// See the long note above serviceNetwork() in rover.ino for why this is a
// ladder. In short: WiFi.status() is the modem's opinion of itself, it latches
// at WL_CONNECTED after the AP goes away without a clean deauth, and a firmware
// that trusts it can never re-associate. Each rung below is only reached when
// every rung above it has already failed.

// Rung 1. If the WebSocket stays down this long while WiFi claims to be up, the
// client is assumed wedged and is torn down and restarted. Without this, a
// rover_server restart on the Pi -- which happens routinely during development
// -- can leave the board sitting there with a live WiFi link and a socket that
// never comes back.
#define WS_RECONNECT_FORCE_MS 10000

// Rung 2. After this many fruitless socket restarts, stop believing the status
// register and ask the network itself (see NET_USE_PING). ~30 s at the interval
// above, which is comfortably longer than any legitimate reconnect.
#define NET_RECYCLE_AFTER_TRIES 3

// Ask the DEFAULT GATEWAY whether the network is real, rather than trusting
// WiFi.status(). This is what separates "the Pi is switched off" (gateway
// answers -- an everyday state, do nothing but keep knocking) from "the radio
// is wedged" (gateway silent while the modem insists it is connected).
//
// Set to 0 if WiFi.ping() is unavailable on the installed core. The ladder still
// works without it, but it can no longer tell those two apart and will recycle
// the radio -- and eventually reset the board -- whenever the Pi is off.
#ifndef NET_USE_PING
#define NET_USE_PING 1
#endif

// Rung 3. No network at all for this long, with every recycle having failed, and
// the board resets itself. This is the state the firmware could not previously
// clear by any means, and is why the operator ended up restarting the ROUTER.
// The reset is refused unless the rig is parked (not moving, nothing queued, no
// latched E-stop) and the position is written to flash first, so it costs
// nothing but the boot time. It can never be triggered merely by the Pi being
// off, because a gateway that answers counts as a live network.
#define NET_REBOOT_MS 300000

// Consecutive failed sendTXT calls before the link is treated as dead despite
// the library still reporting it up. 40 status frames at 20 Hz is 2 s, long
// enough that a transient full TX buffer cannot trip it.
#define NET_TX_FAIL_LIMIT 40

// Association retry backoff, and the association timeout used from the main
// loop. The timeout is bounded because connectWiFi() blocks: motion is
// unaffected (it is generated in the ISR) but the board is deaf to the
// groundstation for the duration, so it must not be tens of seconds.
#define NET_WIFI_RETRY_MS 3000
#define NET_WIFI_RETRY_MAX_MS 20000
#define NET_ASSOC_TIMEOUT_MS 10000

// Scan and report on every Nth failed association. A scan takes seconds and
// disturbs an attempt, so it is not run every time -- but it is the only thing
// that distinguishes "the AP is not there" from "the AP is refusing us", which
// is the distinction this fault was missing for months.
#define NET_SCAN_EVERY 4

// ── Protocol / networking ───────────────────────────────────────────────────
#define FIRMWARE_VERSION "2.0.0"
#define STATUS_INTERVAL_MS 50        // 20 Hz position feedback
#define CMD_QUEUE_DEPTH 4            // lets the Pi pipeline raster moves
#define RX_BUFFER_SIZE 256
#define TX_BUFFER_SIZE 384

// De-energise the drivers after this long with no motion. 0 disables the
// feature, which is the default and the safe choice: an axis that creeps while
// de-energised is silently in the wrong place, and with no endstop and no
// encoder there is no way to notice or recover it short of re-declaring the
// position by hand. Set it (from the panel) if the standstill whine or the
// holding-current heat matters more than that. Overridden at runtime by `cfg`.
#define IDLE_DISABLE_MS 0

// Time for a driver to come out of sleep and its coil current to settle before
// it is asked to step. The A4988 family wakes in about a millisecond.
#define DRIVER_WAKE_MS 5

// Position is saved to emulated EEPROM only after motion has been settled this
// long, so a jog does not write flash once per step.
#define PERSIST_SETTLE_MS 3000
#define PERSIST_MAGIC 0x52565231UL   // "RVR1"
