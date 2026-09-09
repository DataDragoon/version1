// Rover firmware -- 2-axis stepper gantry, Arduino UNO R4 WiFi + CNC Shield V3.
//
// ARCHITECTURE
// ------------
// Step pulses are generated in a fixed-frequency timer ISR. The main loop does
// nothing but talk: WiFi, WebSocket, JSON, status reporting, flash persistence.
// Motion is therefore completely decoupled from networking.
//
// That is the central change from the previous firmware, which ran the steppers
// from loop() and, to stop WiFi from disturbing them, refused to call
// webSocket.loop() while moving. The board was deaf for the whole duration of
// every move: no emergency stop, no position feedback, no way to even refuse a
// command -- it silently dropped anything that arrived mid-move. Everything the
// Pi side had to do to work around that (paced move trains, ack-grace timeouts,
// an "assume it worked" fallback) exists only because of it.
//
// With comms and motion independent, the board can be interrupted at any moment,
// so a held jog key is now a genuine continuous jog rather than a train of small
// discrete moves, and position is reported live at 20 Hz.
//
// WHAT LIVES WHERE
// ----------------
//   config.h          pins, mechanism, defaults          (compile-time)
//   secrets.h         WiFi credentials                   (gitignored)
//   motion_core.h     ramp, limits, watchdog, stepping   (Arduino-free, TESTED)
//   protocol_core.h   JSON scan and emit                 (Arduino-free, TESTED)
//   rover.ino          pins, timer, sockets, persistence  (this file, the glue)
//
// The two _core headers carry all the logic that can actually be wrong, and are
// compiled and tested natively with g++ by rover/test/test_core.cpp -- there is
// no Arduino toolchain on the Pi this is developed from, so that is the only
// verification the firmware gets before it reaches hardware. Keep new logic in
// those headers and keep this file thin.
//
// COORDINATES
// -----------
// One frame for the entire system: steps, positive = UP and RIGHT, origin at
// wherever the operator last declared it. The Pi sends and receives step counts
// and converts to mm with its own calibrated steps/mm. The board never sees mm.
//
// SAFETY
// ------
// There are no endstops on this rig, so soft limits are the only thing between
// a jog and the end of the rail. They are enforced here as well as on the Pi
// on purpose: the Pi can crash or lose its link, the board cannot. A jog also
// carries a dead-man -- it stops on its own if the groundstation stops
// refreshing it -- so a dropped WiFi link cannot leave the rover driving.

#include <WiFiS3.h>
#include <WebSocketsClient.h>
#include <EEPROM.h>
#include <FspTimer.h>

#include "config.h"
#include "types.h"
#include "secrets.h"
#include "motion_core.h"
#include "protocol_core.h"

// ── globals ─────────────────────────────────────────────────────────────────

WebSocketsClient webSocket;
FspTimer stepTimer;

Axis axes[NUM_AXES];

// Millisecond clock derived from the ISR tick count rather than millis(), so
// the timebase the ramp and the jog dead-man use is the same one that drives
// the steps, and neither depends on the main loop running.
volatile uint32_t isrMs = 0;
static uint16_t msDivider = 0;
static const uint16_t MS_DIV = (uint16_t)(ISR_HZ / 1000.0f);

// Which axes pulsed last tick, so only those pins are driven low this tick.
static volatile uint8_t pulseMask = 0;
static int8_t appliedDir[NUM_AXES] = { 0, 0 };

static bool driversEnabled = false;
static bool estopLatched = false;
static uint32_t idleDisableMs = IDLE_DISABLE_MS;

// Sequence of the most recently accepted command, echoed in status so the Pi
// can tell what the board has actually seen.
static uint32_t lastSeq = 0;
static uint32_t inFlightSeq = 0;
static bool movePending = false;
// Which axes the in-flight move actually commanded. `stop_reason` is per axis
// and is NOT cleared by moveTo, so an axis left out of a move still carries
// whatever ended its previous one -- and reporting that as this move's outcome
// makes a perfectly good move come back as 'limit' or 'requested'. The Pi
// aborts a raster on any reason but 'completed', so that is a scan lost to a
// stale byte.
static bool inFlightAxes[NUM_AXES] = { false, false };

static bool linkUp = false;
// Consecutive failed sendTXT calls. A half-open TCP connection -- the state a
// Pi that loses power abruptly leaves behind, with no FIN and no RST -- keeps
// the library reporting the client as connected until its own heartbeat gives
// up. A failed write is the earliest unambiguous evidence there is, it costs
// nothing (the board is already sending 20 status frames a second), and the old
// firmware threw the return value away.
static uint16_t txFails = 0;
static uint32_t lastStatusMs = 0;
static uint32_t lastMotionMs = 0;
static bool persistDirty = false;
static int32_t persistedPos[NUM_AXES] = { 0, 0 };
static bool positionValid = false;

// Queued moves, so the Pi can pipeline a raster instead of paying a network
// round trip per point. QueuedMove is defined in types.h -- see the note there
// about why no type may be defined in this file.
static QueuedMove moveQueue[CMD_QUEUE_DEPTH];
static uint8_t qHead = 0, qTail = 0, qCount = 0;

static char txBuf[TX_BUFFER_SIZE];

// ── pin helpers ─────────────────────────────────────────────────────────────

static inline void stepPinsLow(uint8_t mask) {
    if (mask & 0x01) digitalWrite(PIN_X_STEP, LOW);
    if (mask & 0x02) {
        digitalWrite(PIN_Y_STEP, LOW);
        digitalWrite(PIN_Z_STEP, LOW);
        digitalWrite(PIN_A_STEP, LOW);
    }
}

static inline void applyDirection(uint8_t axis) {
    const int8_t level = axes[axis].dir_level;
    if (level == appliedDir[axis]) return;
    appliedDir[axis] = level;
    const bool invert = (axis == AXIS_V) ? V_DIR_INVERT : H_DIR_INVERT;
    const bool high = (level > 0) != invert;
    if (axis == AXIS_V) {
        digitalWrite(PIN_X_DIR, high ? HIGH : LOW);
    } else {
        digitalWrite(PIN_Y_DIR, high ? HIGH : LOW);
        digitalWrite(PIN_Z_DIR, high ? HIGH : LOW);
        digitalWrite(PIN_A_DIR, high ? HIGH : LOW);
    }
}

static void setDriversEnabled(bool on);

// Re-energises the drivers if idle-disable has parked them, and waits for the
// coil current to come up. Every path that can start motion goes through this,
// so a move can never be issued into a sleeping driver -- which would silently
// drop the first few steps.
static void wakeDrivers() {
    if (driversEnabled || estopLatched) return;
    setDriversEnabled(true);
    delay(DRIVER_WAKE_MS);
}

static void setDriversEnabled(bool on) {
    driversEnabled = on;
    digitalWrite(PIN_ENABLE, on ? LOW : HIGH);   // shield EN is active LOW
}

// ── the ISR ─────────────────────────────────────────────────────────────────
//
// Budget at 20 kHz is 50 us. Worst case here is four step pins low, up to two
// direction writes and four step pins high -- about 10 us with digitalWrite on
// this core. If step timing ever needs to be tighter, replacing these with
// direct port writes is the upgrade path; nothing else has to change.

void onStepTimer(timer_callback_args_t* /*args*/) {
    // End the pulses started on the previous tick. One tick (50 us) is far more
    // than the ~1 us minimum the drivers need.
    if (pulseMask) {
        stepPinsLow(pulseMask);
        pulseMask = 0;
    }

    if (++msDivider >= MS_DIV) {
        msDivider = 0;
        ++isrMs;
    }

    if (!driversEnabled) return;

    if (axes[AXIS_V].tick(isrMs)) {
        applyDirection(AXIS_V);
        digitalWrite(PIN_X_STEP, HIGH);
        pulseMask |= 0x01;
    }
    if (axes[AXIS_H].tick(isrMs)) {
        applyDirection(AXIS_H);
        digitalWrite(PIN_Y_STEP, HIGH);
        digitalWrite(PIN_Z_STEP, HIGH);
        digitalWrite(PIN_A_STEP, HIGH);
        pulseMask |= 0x02;
    }
}

// A 32-bit aligned read is atomic on Cortex-M, so this needs no critical
// section -- and must not take one: it is called while building arguments for
// calls that are already inside one, where re-enabling interrupts early would
// tear the very update the section exists to protect.
static inline uint32_t nowMs() { return isrMs; }

// ── persistence ─────────────────────────────────────────────────────────────
//
// The Pi is the authoritative store (it persists rover_state.json and survives
// this board being swapped). What is kept here is a cross-check: on boot the
// board reports what it last saw, the Pi compares it against its own record,
// and a mismatch raises a re-home warning rather than one side silently
// winning. Written only after motion has been settled a while, so a jog does
// not write flash once per step.

static uint32_t blobCheck(const PersistBlob& b) {
    return b.magic ^ (uint32_t)b.pos[0] ^ ((uint32_t)b.pos[1] * 2654435761UL);
}

static void loadPosition() {
    PersistBlob b;
    EEPROM.get(0, b);
    if (b.magic == PERSIST_MAGIC && b.check == blobCheck(b)) {
        axes[AXIS_V].setPosition(b.pos[AXIS_V]);
        axes[AXIS_H].setPosition(b.pos[AXIS_H]);
        persistedPos[AXIS_V] = b.pos[AXIS_V];
        persistedPos[AXIS_H] = b.pos[AXIS_H];
        positionValid = true;
    }
}

static void savePosition() {
    PersistBlob b;
    b.magic = PERSIST_MAGIC;
    b.pos[AXIS_V] = axes[AXIS_V].position;
    b.pos[AXIS_H] = axes[AXIS_H].position;
    b.check = blobCheck(b);
    EEPROM.put(0, b);
    persistedPos[AXIS_V] = b.pos[AXIS_V];
    persistedPos[AXIS_H] = b.pos[AXIS_H];
    persistDirty = false;
}

// ── outgoing messages ───────────────────────────────────────────────────────

static void send(const proto::Writer& w) {
    if (w.overflow()) {
        // Never emit a truncated document -- the Pi would fail to parse it and
        // could not tell that from a link fault.
        Serial.println("[tx] DROPPED: message overflowed the buffer");
        return;
    }
    if (!linkUp) return;
    if (webSocket.sendTXT(w.c_str())) {
        txFails = 0;
    } else if (txFails < 0xFFFF) {
        ++txFails;
    }
}

static void sendHello() {
    proto::Writer w(txBuf, sizeof(txBuf));
    w.begin();
    w.str("t", "hello");
    w.str("fw", FIRMWARE_VERSION);
    // Advertised so the Pi has a sane starting point for its own calibration.
    // The Pi's configured value wins; this is informational.
    w.f("v_spmm", V_STEPS_PER_MM, 4);
    w.f("h_spmm", H_STEPS_PER_MM, 4);
    w.i32("v_pos", axes[AXIS_V].position);
    w.i32("h_pos", axes[AXIS_H].position);
    // Named to match the `cfg` command's keys exactly, so there is one name per
    // quantity across the whole protocol. An earlier draft called the limits
    // v_max/h_max, which collided with cfg's max-SPEED keys.
    w.i32("v_lo", axes[AXIS_V].p.min_limit);
    w.i32("v_hi", axes[AXIS_V].p.max_limit);
    w.i32("h_lo", axes[AXIS_H].p.min_limit);
    w.i32("h_hi", axes[AXIS_H].p.max_limit);
    w.f("v_speed", axes[AXIS_V].p.max_speed, 1);
    w.f("v_jog", axes[AXIS_V].p.jog_speed, 1);
    w.f("v_accel", axes[AXIS_V].p.accel, 1);
    w.f("h_speed", axes[AXIS_H].p.max_speed, 1);
    w.f("h_jog", axes[AXIS_H].p.jog_speed, 1);
    w.f("h_accel", axes[AXIS_H].p.accel, 1);
    w.boolean("limits", axes[AXIS_V].p.limits_enabled);
    w.boolean("pos_valid", positionValid);
    w.boolean("estop", estopLatched);
    w.u32("ms", nowMs());
    w.end();
    send(w);
}

static void sendStatus() {
    proto::Writer w(txBuf, sizeof(txBuf));
    w.begin();
    w.str("t", "status");
    w.u32("seq", lastSeq);
    w.i32("v_pos", axes[AXIS_V].position);
    w.i32("h_pos", axes[AXIS_H].position);
    w.f("v_spd", axes[AXIS_V].speed, 1);
    w.f("h_spd", axes[AXIS_H].speed, 1);
    w.i32("v_mode", (int32_t)axes[AXIS_V].mode);
    w.i32("h_mode", (int32_t)axes[AXIS_H].mode);
    // A jog that ends on a soft limit produces no `done` (it was never a move),
    // so the reason has to ride along here or the panel cannot explain why the
    // rover stopped on its own.
    w.i32("v_stop", (int32_t)axes[AXIS_V].stop_reason);
    w.i32("h_stop", (int32_t)axes[AXIS_H].stop_reason);
    w.boolean("moving", axes[AXIS_V].isMoving() || axes[AXIS_H].isMoving());
    w.boolean("estop", estopLatched);
    w.boolean("en", driversEnabled);
    w.boolean("pos_valid", positionValid);
    w.u32("idle_ms", idleDisableMs);
    w.i32("q", (int32_t)qCount);
    w.u32("ms", nowMs());
    w.end();
    send(w);
}

static void sendDone(uint32_t seq, StopReason reason) {
    proto::Writer w(txBuf, sizeof(txBuf));
    w.begin();
    w.str("t", "done");
    w.u32("seq", seq);
    w.i32("v_pos", axes[AXIS_V].position);
    w.i32("h_pos", axes[AXIS_H].position);
    w.i32("reason", (int32_t)reason);
    w.u32("ms", nowMs());
    w.end();
    send(w);
}

static void sendAck(uint32_t seq) {
    proto::Writer w(txBuf, sizeof(txBuf));
    w.begin();
    w.str("t", "ack");
    w.u32("seq", seq);
    w.end();
    send(w);
}

static void sendError(uint32_t seq, const char* code, const char* msg) {
    proto::Writer w(txBuf, sizeof(txBuf));
    w.begin();
    w.str("t", "err");
    w.u32("seq", seq);
    w.str("code", code);
    w.str("msg", msg);
    w.end();
    send(w);
    Serial.print("[err] "); Serial.print(code); Serial.print(": "); Serial.println(msg);
}

// ── move queue ──────────────────────────────────────────────────────────────

static bool queuePush(const QueuedMove& m) {
    if (qCount >= CMD_QUEUE_DEPTH) return false;
    moveQueue[qTail] = m;
    qTail = (uint8_t)((qTail + 1) % CMD_QUEUE_DEPTH);
    ++qCount;
    return true;
}

static void queueClear() {
    qHead = qTail = qCount = 0;
}

static void dispatchQueued() {
    if (qCount == 0 || estopLatched) return;
    if (axes[AXIS_V].isMoving() || axes[AXIS_H].isMoving()) return;

    const QueuedMove m = moveQueue[qHead];
    qHead = (uint8_t)((qHead + 1) % CMD_QUEUE_DEPTH);
    --qCount;

    inFlightSeq = m.seq;
    movePending = true;
    noInterrupts();
    for (uint8_t i = 0; i < NUM_AXES; ++i) {
        inFlightAxes[i] = m.has[i];
        if (!m.has[i]) continue;
        // Clear the previous move's outcome before starting this one, so the
        // reason reported below can only have come from this move.
        axes[i].stop_reason = STOP_NONE;
        axes[i].moveTo(m.rel ? axes[i].position + m.value[i] : m.value[i]);
    }
    interrupts();
}

// ── command handling ────────────────────────────────────────────────────────

// Parses one axis's settings into `out`. Deliberately does NOT touch the live
// axis: JSON scanning and snprintf take tens of microseconds, and doing that
// inside the critical section that guards the ISR would stall stepping long
// enough to be felt. The caller copies the result in under a short lock.
static void parseAxisConfig(const char* json, const AxisParams& current,
                            const char* prefix, AxisParams* out) {
    *out = current;
    char key[24];
    float f;
    int32_t i;

    snprintf(key, sizeof(key), "%s_speed", prefix);
    if (proto::getFloat(json, key, &f) && f > 0) out->max_speed = f;
    snprintf(key, sizeof(key), "%s_jog", prefix);
    if (proto::getFloat(json, key, &f) && f > 0) out->jog_speed = f;
    snprintf(key, sizeof(key), "%s_accel", prefix);
    if (proto::getFloat(json, key, &f) && f > 0) out->accel = f;
    snprintf(key, sizeof(key), "%s_lo", prefix);
    if (proto::getInt32(json, key, &i)) out->min_limit = i;
    snprintf(key, sizeof(key), "%s_hi", prefix);
    if (proto::getInt32(json, key, &i)) out->max_limit = i;

    // An inverted envelope would clamp every move to nothing and present as a
    // dead axis, so fix it here rather than leaving it to be discovered.
    if (out->min_limit > out->max_limit) {
        const int32_t t = out->min_limit;
        out->min_limit = out->max_limit;
        out->max_limit = t;
    }
}

static void handleCommand(const char* json) {
    char cmd[20];
    if (!proto::getStr(json, "c", cmd, sizeof(cmd))) {
        sendError(0, "bad_msg", "no command field");
        return;
    }

    int32_t seqRaw = 0;
    proto::getInt32(json, "seq", &seqRaw);
    const uint32_t seq = (uint32_t)seqRaw;

    // Idempotency: re-acknowledge a repeat rather than executing it twice, so
    // the Pi can retransmit safely when an ack goes missing.
    const bool isHold = (strcmp(cmd, "jog_hold") == 0);
    if (seq != 0 && seq == lastSeq && !isHold) {
        sendAck(seq);
        return;
    }
    // A hold arrives several times a second and changes nothing worth
    // sequencing. It must NOT advance lastSeq: the status stream reports
    // lastSeq as "the last command I acted on", and the Pi uses that to discard
    // status frames older than a position change it has just made. Holds
    // running through it would make that number meaningless.
    if (!isHold) lastSeq = seq;

    // Nothing but clearing the latch may run while E-stopped.
    // jog_hold is exempt: it arrives several times a second while a key is held,
    // and answering each one with an error would flood the link for no reason.
    // It is harmless when latched -- no axis is in MODE_JOG to refresh.
    if (estopLatched && strcmp(cmd, "clear_estop") != 0 &&
        strcmp(cmd, "ping") != 0 && strcmp(cmd, "status") != 0 &&
        strcmp(cmd, "jog_hold") != 0) {
        sendError(seq, "estop", "latched; clear the E-stop first");
        return;
    }

    if (strcmp(cmd, "move") == 0) {
        wakeDrivers();
        bool rel = true;
        proto::getBool(json, "rel", &rel);
        QueuedMove m;
        m.seq = seq;
        m.rel = rel;
        int32_t v = 0, h = 0;
        m.has[AXIS_V] = proto::getInt32(json, "v", &v);
        m.value[AXIS_V] = v;
        m.has[AXIS_H] = proto::getInt32(json, "h", &h);
        m.value[AXIS_H] = h;
        if (!m.has[AXIS_V] && !m.has[AXIS_H]) {
            sendError(seq, "bad_msg", "move needs v and/or h");
            return;
        }
        if (!queuePush(m)) {
            sendError(seq, "full", "move queue is full");
            return;
        }
        sendAck(seq);
        return;
    }

    if (strcmp(cmd, "jog") == 0) {
        wakeDrivers();
        char axName[4];
        int32_t dir = 0;
        int32_t holdMs = JOG_WATCHDOG_MS;
        if (!proto::getStr(json, "axis", axName, sizeof(axName)) ||
            !proto::getInt32(json, "dir", &dir) || dir == 0) {
            sendError(seq, "bad_msg", "jog needs axis and dir");
            return;
        }
        proto::getInt32(json, "hold_ms", &holdMs);
        const uint8_t ax = (axName[0] == 'v') ? AXIS_V : AXIS_H;
        // A jog while moves are queued would fight them; drop the queue.
        queueClear();
        movePending = false;
        const uint32_t deadline = nowMs() + (uint32_t)holdMs;
        noInterrupts();
        axes[ax].jog((int8_t)(dir > 0 ? 1 : -1), deadline);
        interrupts();
        sendAck(seq);
        return;
    }

    if (strcmp(cmd, "jog_hold") == 0) {
        int32_t holdMs = JOG_WATCHDOG_MS;
        proto::getInt32(json, "hold_ms", &holdMs);
        const uint32_t deadline = nowMs() + (uint32_t)holdMs;
        noInterrupts();
        axes[AXIS_V].refreshJog(deadline);
        axes[AXIS_H].refreshJog(deadline);
        interrupts();
        return;   // deliberately unacknowledged: this arrives many times a second
    }

    if (strcmp(cmd, "stop") == 0) {
        queueClear();
        movePending = false;
        noInterrupts();
        axes[AXIS_V].requestStop();
        axes[AXIS_H].requestStop();
        interrupts();
        sendAck(seq);
        return;
    }

    if (strcmp(cmd, "estop") == 0) {
        queueClear();
        movePending = false;
        noInterrupts();
        axes[AXIS_V].emergencyStop();
        axes[AXIS_H].emergencyStop();
        interrupts();
        estopLatched = true;
        setDriversEnabled(false);
        sendAck(seq);
        sendStatus();
        return;
    }

    if (strcmp(cmd, "clear_estop") == 0) {
        noInterrupts();
        axes[AXIS_V].clearEstop();
        axes[AXIS_H].clearEstop();
        interrupts();
        estopLatched = false;
        setDriversEnabled(true);
        // Position is suspect after an E-stop: cutting the step train at speed
        // is exactly the case where a stepper can lose steps. Say so rather
        // than letting the Pi carry on as if nothing happened.
        positionValid = false;
        sendAck(seq);
        sendStatus();     // status carries pos_valid; see the note under cfg
        return;
    }

    if (strcmp(cmd, "set_pos") == 0) {
        int32_t v, h;
        noInterrupts();
        if (proto::getInt32(json, "v", &v)) axes[AXIS_V].setPosition(v);
        if (proto::getInt32(json, "h", &h)) axes[AXIS_H].setPosition(h);
        interrupts();
        positionValid = true;
        persistDirty = true;
        sendAck(seq);
        sendStatus();
        return;
    }

    if (strcmp(cmd, "cfg") == 0) {
        AxisParams v, h;
        parseAxisConfig(json, axes[AXIS_V].p, "v", &v);
        parseAxisConfig(json, axes[AXIS_H].p, "h", &h);
        int32_t idle;
        if (proto::getInt32(json, "idle_ms", &idle) && idle >= 0) {
            idleDisableMs = (uint32_t)idle;
            // Turning the feature off must put the holding current back now, not
            // at the next move -- the whole point of leaving it off is that the
            // axes are held.
            if (idleDisableMs == 0) wakeDrivers();
        }
        bool limits;
        if (proto::getBool(json, "limits", &limits)) {
            v.limits_enabled = limits;
            h.limits_enabled = limits;
        }
        noInterrupts();
        axes[AXIS_V].p = v;
        axes[AXIS_H].p = h;
        interrupts();
        sendAck(seq);
        // Reply with status, NOT hello. The Pi pushes its configuration in
        // response to a hello, so answering cfg with one puts the two in a
        // tight cfg -> hello -> cfg loop that saturates the link. `hello` means
        // "this connection just came up" and nothing else.
        sendStatus();
        return;
    }

    if (strcmp(cmd, "enable") == 0) {
        bool on = true;
        proto::getBool(json, "on", &on);
        setDriversEnabled(on);
        sendAck(seq);
        sendStatus();
        return;
    }

    if (strcmp(cmd, "ping") == 0) { sendAck(seq); return; }
    if (strcmp(cmd, "status") == 0) { sendStatus(); return; }

    sendError(seq, "unknown", cmd);
}

// ── websocket ───────────────────────────────────────────────────────────────

void webSocketEvent(WStype_t type, uint8_t* payload, size_t length) {
    switch (type) {
    case WStype_CONNECTED:
        linkUp = true;
        txFails = 0;
        Serial.println("[ws] connected");
        sendHello();
        break;

    case WStype_DISCONNECTED:
        linkUp = false;
        txFails = 0;
        Serial.println("[ws] disconnected");
        movePending = false;
        // Losing the link must not leave the rover driving. The jog dead-man
        // would catch this within JOG_WATCHDOG_MS anyway; stopping here makes
        // it immediate.
        queueClear();
        noInterrupts();
        axes[AXIS_V].requestStop();
        axes[AXIS_H].requestStop();
        interrupts();
        break;

    case WStype_TEXT: {
        if (length >= RX_BUFFER_SIZE) {
            sendError(0, "too_long", "command exceeds the receive buffer");
            return;
        }
        char buf[RX_BUFFER_SIZE];
        memcpy(buf, payload, length);
        buf[length] = '\0';
        handleCommand(buf);
        break;
    }

    default:
        break;
    }
}

// ── network bring-up ────────────────────────────────────────────────────────
//
// WHY THIS IS A LADDER AND NOT TWO INDEPENDENT KEEP-ALIVES
// -------------------------------------------------------
// The previous version had two layers with a strictly one-way trust
// relationship: linkReady() was the sole authority on the WiFi layer, and the
// socket layer deferred to it unconditionally ("ensureNetwork owns that case").
// That is the bug behind "it never even tries to come back, and only a ROUTER
// restart fixes it".
//
// WiFi.status() and WiFi.localIP() are not measurements of a working network.
// They are the modem's opinion of itself, and the modem latches: after the AP
// goes away without a clean deauth -- which is exactly what a power cut at
// either end leaves behind, and what an AP still holding a stale station entry
// for this MAC produces -- status() can sit at WL_CONNECTED with the last-known
// address still in localIP() indefinitely. When that happened, ensureNetwork()
// returned at its first line every single iteration and never re-associated,
// while ensureSocket() churned disconnect()/begin() against a dead stack
// forever. Nothing could break the deadlock, because the one piece of hard
// evidence available -- a socket that will not come back -- was never allowed
// to act on the layer below it.
//
// So: a socket that stays dead is now treated as evidence about the RADIO, and
// the radio is checked against the NETWORK (a gateway ping) rather than against
// its own status register. The ladder, keyed on how long the link has been
// down:
//
//   0-10 s    nothing; the library's own reconnect is given its chance
//   10 s      restart the websocket client
//   ~30 s     after NET_RECYCLE_AFTER_TRIES restarts, ask the gateway whether
//             the network is actually there:
//               it answers  -> radio and LAN are fine, the Pi is simply not
//                              running. Keep retrying the socket. Do NOT
//                              recycle and do NOT reset -- a Pi that is off is
//                              an everyday state, not a fault.
//               it is silent -> the radio is lying about being connected. Tear
//                              it all the way down (WiFi.end()) and start over,
//                              which is the only thing that reliably clears a
//                              wedged modem and reclaims the sockets it holds.
//   5 min     the network has not answered once in all that time and every
//             recycle has failed. Reset the board. That is the state this
//             firmware could not otherwise clear, and it is what the operator
//             was reaching for by hand -- except they were power-cycling the
//             ROUTER, because power-cycling the BOARD did not help either.
//
// Nothing here can leave the rover driving: losing the link already stops the
// axes and clears the queue (see WStype_DISCONNECTED), the jog dead-man lives
// in the ISR, and the reset is refused unless the rig is parked.

// Prints the radio's MAC in both byte orders, because this family of libraries
// fills the array backwards (the Arduino examples print index 5 down to 0) and
// getting it wrong hands you a MAC that will never match a router reservation.
// Whichever line matches what the router shows is the real one.
static void printMacAddress() {
    uint8_t mac[6] = {0, 0, 0, 0, 0, 0};
    WiFi.macAddress(mac);
    char fwd[18], rev[18];
    snprintf(fwd, sizeof(fwd), "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    snprintf(rev, sizeof(rev), "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[5], mac[4], mac[3], mac[2], mac[1], mac[0]);
    Serial.print("[wifi] MAC (reversed, the usual order for this library): ");
    Serial.println(rev);
    Serial.print("[wifi] MAC (as stored):                                  ");
    Serial.println(fwd);
}

// Associated AND holding an address. Necessary, not sufficient -- see the note
// above about the modem latching this. Never treat it as proof of a network.
static bool linkReady() {
    return WiFi.status() == WL_CONNECTED &&
           WiFi.localIP() != IPAddress(0, 0, 0, 0);
}

// Is there actually a network out there? This is the only question in this file
// whose answer does not come from the modem's opinion of itself, which is why
// the whole ladder turns on it. A gateway that answers ICMP proves the radio is
// associated, that the link passes traffic, and that the LAN is up -- all of
// which WiFi.status() will happily claim while none of them are true.
//
// Deliberately NOT a ping of the Pi: the Pi being off is an ordinary state and
// must never be mistaken for a broken radio.
static bool networkResponds() {
#if NET_USE_PING
    const IPAddress gw = WiFi.gatewayIP();
    if (gw == IPAddress(0, 0, 0, 0)) return false;
    // Returns the round trip in ms, or negative on timeout/unreachable.
    return WiFi.ping(gw) >= 0;
#else
    // Fallback if WiFi.ping() is unavailable on this core. It answers NO, not
    // linkReady(): the whole question being asked is whether to believe the
    // modem, and answering it with the modem's own opinion reinstates the exact
    // deadlock this file exists to break -- a latched WL_CONNECTED would report
    // a healthy network for ever and nothing would ever escalate.
    //
    // Saying no instead means the ladder can no longer tell "the Pi is off"
    // from "the radio is wedged", so it recycles the radio (and eventually
    // resets) in both cases. That is wasteful when the Pi is merely off, and it
    // is the right way round: a needless recycle costs a few seconds, a missed
    // one costs the session.
    return false;
#endif
}

// Reports whether the configured SSID is even on the air, and at what level.
// Run only after repeated association failures, because a scan takes seconds
// and disturbs an association attempt. It is the measurement that separates
// "the AP is refusing this board" from "the AP is not there at all" -- and
// without it both look identical in the log, which is how this fault came to be
// diagnosed as "restart the router" rather than as anything specific.
static void reportScan() {
    Serial.println("[wifi] scanning to see whether the AP is on the air...");
    const int n = WiFi.scanNetworks();
    if (n <= 0) {
        Serial.println("[wifi] scan found NO networks at all -- that is this "
                       "radio or its antenna, not the AP");
        return;
    }
    bool found = false;
    for (int i = 0; i < n; ++i) {
        const char* ssid = WiFi.SSID(i);
        if (ssid && strcmp(ssid, WIFI_SSID) == 0) {
            found = true;
            Serial.print("[wifi] '");
            Serial.print(WIFI_SSID);
            Serial.print("' IS visible, rssi ");
            Serial.println((long)WiFi.RSSI(i));
        }
    }
    if (!found) {
        Serial.print("[wifi] '");
        Serial.print(WIFI_SSID);
        Serial.print("' NOT visible among ");
        Serial.print(n);
        Serial.println(" networks -- wrong SSID, out of range, or the AP is on "
                       "a band or channel this radio cannot see");
    } else {
        Serial.println("[wifi] the AP is there and we still cannot join it: "
                       "wrong password, MAC filtering, or the AP is holding a "
                       "stale station entry for this MAC. If only a ROUTER "
                       "restart ever clears it, enable USE_STATIC_IP.");
    }
}

static bool connectWiFi(uint32_t timeoutMs) {
    Serial.print("[wifi] connecting to ");
    Serial.println(WIFI_SSID);
    WiFi.disconnect();
    delay(200);

#ifdef USE_STATIC_IP
    // Skips DHCP entirely, which is the surest cure for a stale lease.
    WiFi.config(IPAddress(STATIC_IP), IPAddress(STATIC_DNS),
                IPAddress(STATIC_GATEWAY), IPAddress(STATIC_SUBNET));
    Serial.println("[wifi] using a static address");
#endif

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    const uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < timeoutMs) {
        delay(250);
    }
    if (WiFi.status() != WL_CONNECTED) {
        Serial.print("[wifi] association failed, status=");
        Serial.println(WiFi.status());
        // Printed on FAILURE as well as on success. The MAC is what a DHCP
        // reservation is keyed on, so it is wanted exactly when the board is
        // NOT getting on the network -- the old firmware printed it only after
        // a successful connect, i.e. never when it mattered.
        printMacAddress();
        return false;
    }

    const uint32_t ipStart = millis();
    while (WiFi.localIP() == IPAddress(0, 0, 0, 0) &&
           millis() - ipStart < DHCP_TIMEOUT_MS) {
        delay(100);
    }
    if (WiFi.localIP() == IPAddress(0, 0, 0, 0)) {
        // Associated but no address. Drop the association rather than sitting
        // in this half-connected state, so the next attempt starts clean.
        Serial.println("[wifi] associated but got NO DHCP LEASE -- disconnecting "
                       "so the next attempt starts fresh");
        printMacAddress();
        WiFi.disconnect();
        return false;
    }
    Serial.print("[wifi] ip ");
    Serial.print(WiFi.localIP());
    Serial.print("  gw ");
    Serial.print(WiFi.gatewayIP());
    Serial.print("  rssi ");
    Serial.println(WiFi.RSSI());
    printMacAddress();
    return true;
}

// Tear the radio all the way down and bring it back. WiFi.disconnect() only
// drops the association; WiFi.end() stops the WiFi stack in the modem, which is
// what releases the sockets it is still holding and re-arms its connection
// state machine. Reconnecting without it is what leaves a wedged modem wedged --
// and the socket layer asks for a fresh client on every reconnect attempt, so a
// link that has been down for an hour has asked the modem for hundreds of them.
static bool recycleRadio() {
    Serial.println("[wifi] RECYCLING the radio (end + re-associate)");
    webSocket.disconnect();
    WiFi.disconnect();
    WiFi.end();
    delay(500);
    return connectWiFi(NET_ASSOC_TIMEOUT_MS);
}

static void restartSocket() {
    webSocket.disconnect();
    webSocket.begin(PI_HOST, PI_PORT, "/");
}

// A reset is only ever taken from a standstill. It is not a safety event in
// itself -- motion is already stopped by the time the ladder gets this far, and
// the position survives in flash -- but it must not silently clear a latched
// E-stop or re-validate a position the operator was told not to trust.
static bool boardParked() {
    if (estopLatched) return false;
    if (movePending || qCount > 0) return false;
    return !axes[AXIS_V].isMoving() && !axes[AXIS_H].isMoving();
}

// Make the flash copy agree with the live one before a deliberate reset.
// loadPosition() marks the position VALID whenever it finds a well-formed blob,
// so coming back from a reset with a stale-but-checksummed blob would silently
// re-declare a position an E-stop had invalidated. Clearing the magic is how
// "no trustworthy position" is expressed in that format.
//
// Both branches are guarded against writing bytes that are already there. A
// board that can never reach the network resets every NET_REBOOT_MS, so an
// unconditional write here would be a flash erase every five minutes for as
// long as the fault lasts -- some 300 a day against the data flash's endurance,
// for no information gained.
static void persistBeforeReset() {
    if (positionValid) {
        if (axes[AXIS_V].position != persistedPos[AXIS_V] ||
            axes[AXIS_H].position != persistedPos[AXIS_H]) {
            savePosition();
        }
        return;
    }
    PersistBlob cur;
    EEPROM.get(0, cur);
    if (cur.magic == 0) return;          // already says "nothing trustworthy"
    PersistBlob b;
    b.magic = 0;
    b.pos[AXIS_V] = 0;
    b.pos[AXIS_H] = 0;
    b.check = 0;
    EEPROM.put(0, b);
}

// The whole ladder. Called once per loop(); everything is rate-limited inside.
static void serviceNetwork() {
    // Last moment the NETWORK was known good -- a live websocket, or a gateway
    // that answered. Only this drives the reset, so a Pi that is simply off can
    // never cause one however long it stays off.
    static uint32_t netOkMs = 0;
    static uint32_t nextTryMs = 0;
    static uint32_t wifiBackoff = NET_WIFI_RETRY_MS;
    static uint8_t socketTries = 0;
    static uint16_t assocFailures = 0;

    const uint32_t now = millis();

    // The one unambiguous good state. txFails is folded in because a websocket
    // whose writes all fail is not up, whatever the library thinks -- see
    // send(), where a half-open TCP connection first shows itself.
    if (linkUp && txFails < NET_TX_FAIL_LIMIT) {
        netOkMs = now;
        nextTryMs = now + WS_RECONNECT_FORCE_MS;
        wifiBackoff = NET_WIFI_RETRY_MS;
        socketTries = 0;
        assocFailures = 0;
        return;
    }

    // Wrap-safe: a plain `now < nextTryMs` stops working after 49.7 days.
    if ((int32_t)(now - nextTryMs) < 0) return;

    // ── last resort, checked BEFORE acting ──────────────────────────────────
    // It has to be here rather than after the branches below. Each of them
    // either returns on success or reschedules, so a modem that kept
    // associating happily onto a network that does not work would escalate for
    // ever and never reach a check placed after them -- which is the exact
    // shape of the wedge this whole ladder exists to break.
    //
    // The gateway gets one more chance first: netOkMs is only refreshed by a
    // live websocket or an answering gateway, so five minutes of silence is
    // also consistent with the network having come back quietly since the last
    // ping.
    if ((uint32_t)(now - netOkMs) >= NET_REBOOT_MS) {
        if (networkResponds()) {
            netOkMs = millis();
        } else if (!boardParked()) {
            // Refused while there is motion, queued work or a latched E-stop.
            // Rechecked next tick; the rig cannot be moving for long with no
            // link, because losing it stops the axes.
            Serial.println("[net] offline past the reset threshold but the rig "
                           "is not parked -- holding off");
            nextTryMs = millis() + WS_RECONNECT_FORCE_MS;
            return;
        } else {
            Serial.print("[net] no network for ");
            Serial.print((unsigned long)(NET_REBOOT_MS / 1000UL));
            Serial.println(" s and every recovery has failed -- RESETTING the "
                           "board");
            persistBeforeReset();
            delay(100);          // let the serial line drain
            NVIC_SystemReset();
        }
    }

    if (!linkReady()) {
        // ── the modem says it is not connected: ordinary re-association ─────
        Serial.print("[wifi] link not ready (status=");
        Serial.print(WiFi.status());
        Serial.print(" ip=");
        Serial.print(WiFi.localIP());
        Serial.print(") attempt ");
        Serial.println(assocFailures + 1);

        // Every attempt after the first goes through a full teardown. The first
        // does not, because the common case is a brief AP hiccup that a plain
        // re-associate clears in a second.
        const bool ok = (assocFailures == 0) ? connectWiFi(NET_ASSOC_TIMEOUT_MS)
                                             : recycleRadio();
        if (ok) {
            assocFailures = 0;
            wifiBackoff = NET_WIFI_RETRY_MS;
            socketTries = 0;
            restartSocket();
            // A brand-new association deserves the ladder from the top.
            nextTryMs = millis() + WS_RECONNECT_FORCE_MS;
            return;
        }

        ++assocFailures;
        if (assocFailures % NET_SCAN_EVERY == 0) reportScan();
        wifiBackoff = (wifiBackoff < NET_WIFI_RETRY_MAX_MS)
                          ? wifiBackoff * 2
                          : NET_WIFI_RETRY_MAX_MS;
        nextTryMs = millis() + wifiBackoff;
    } else {
        // ── the modem says it IS connected, but the socket is not ───────────
        ++socketTries;
        nextTryMs = now + WS_RECONNECT_FORCE_MS;

        if (socketTries < NET_RECYCLE_AFTER_TRIES) {
            Serial.print("[ws] socket down -- restarting the client (try ");
            Serial.print(socketTries);
            Serial.println(")");
            restartSocket();
            return;
        }

        // The socket has refused to come back several times running. Stop
        // believing the status register and ask the network itself.
        socketTries = 0;
        if (networkResponds()) {
            // Radio and LAN are fine. The Pi is not running, or not listening.
            // Nothing to escalate -- keep knocking.
            netOkMs = now;
            Serial.println("[net] gateway answers -- the network is fine and "
                           "the Pi is not. Retrying the socket.");
            restartSocket();
            return;
        }

        Serial.println("[net] socket dead AND the gateway silent, while "
                       "WiFi.status() claims we are connected -- the radio is "
                       "not to be trusted");
        if (recycleRadio()) {
            restartSocket();
            nextTryMs = millis() + WS_RECONNECT_FORCE_MS;
            return;
        }
        nextTryMs = millis() + NET_WIFI_RETRY_MS;
    }
}

// ── timer bring-up ──────────────────────────────────────────────────────────

static bool startStepTimer() {
    uint8_t type;
    int8_t ch = FspTimer::get_available_timer(type);
    if (ch < 0) {
        // Fall back to a timer the PWM code reserves; nothing here uses PWM.
        FspTimer::force_use_of_pwm_reserved_timer();
        ch = FspTimer::get_available_timer(type, true);
    }
    if (ch < 0) return false;
    if (!stepTimer.begin(TIMER_MODE_PERIODIC, type, (uint8_t)ch,
                         ISR_HZ, 0.0f, onStepTimer)) return false;
    if (!stepTimer.setup_overflow_irq()) return false;
    if (!stepTimer.open()) return false;
    return stepTimer.start();
}

// ── setup / loop ────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    delay(1500);
    Serial.println();
    Serial.print("rover firmware ");
    Serial.println(FIRMWARE_VERSION);

    const int stepPins[] = { PIN_X_STEP, PIN_Y_STEP, PIN_Z_STEP, PIN_A_STEP };
    const int dirPins[]  = { PIN_X_DIR,  PIN_Y_DIR,  PIN_Z_DIR,  PIN_A_DIR  };
    for (uint8_t i = 0; i < 4; ++i) {
        pinMode(stepPins[i], OUTPUT);
        digitalWrite(stepPins[i], LOW);
        pinMode(dirPins[i], OUTPUT);
        digitalWrite(dirPins[i], LOW);
    }
    pinMode(PIN_ENABLE, OUTPUT);
    setDriversEnabled(false);        // stay de-energised until the ISR is live

    axes[AXIS_V].p = {
        V_STEPS_PER_MM,
        V_MAX_SPEED_MM_S * V_STEPS_PER_MM,
        V_JOG_SPEED_MM_S * V_STEPS_PER_MM,
        V_ACCEL_MM_S2 * V_STEPS_PER_MM,
        (int32_t)lroundf(V_MIN_MM * V_STEPS_PER_MM),
        (int32_t)lroundf(V_MAX_MM * V_STEPS_PER_MM),
        true,
    };
    axes[AXIS_H].p = {
        H_STEPS_PER_MM,
        H_MAX_SPEED_MM_S * H_STEPS_PER_MM,
        H_JOG_SPEED_MM_S * H_STEPS_PER_MM,
        H_ACCEL_MM_S2 * H_STEPS_PER_MM,
        (int32_t)lroundf(H_MIN_MM * H_STEPS_PER_MM),
        (int32_t)lroundf(H_MAX_MM * H_STEPS_PER_MM),
        true,
    };
    for (uint8_t i = 0; i < NUM_AXES; ++i) {
        axes[i].begin(ISR_HZ, RAMP_HZ, MIN_SPEED_STEPS_S);
    }

    loadPosition();
    Serial.print("[pos] restored ");
    Serial.print(positionValid ? "yes" : "no");
    Serial.print("  v="); Serial.print(axes[AXIS_V].position);
    Serial.print("  h="); Serial.println(axes[AXIS_H].position);

    if (!startStepTimer()) {
        // Without the timer there is no motion at all. Say so loudly and keep
        // the drivers off rather than pretending to work.
        Serial.println("[timer] FAILED to start -- no motion is possible");
    } else {
        Serial.print("[timer] stepping at ");
        Serial.print(ISR_HZ);
        Serial.println(" Hz");
        setDriversEnabled(true);
    }

    // onEvent BEFORE begin: begin() is what arms the client, and a handler
    // registered after it is a race for no reason.
    webSocket.onEvent(webSocketEvent);
    webSocket.setReconnectInterval(3000);
    // Keeps the socket alive across long moves; the previous firmware could not
    // service pings while moving at all.
    webSocket.enableHeartbeat(15000, 3000, 2);

    // A failure here is not fatal and never was worth blocking on: serviceNetwork()
    // owns every retry from now on, including this one.
    connectWiFi(NET_ASSOC_TIMEOUT_MS);
    restartSocket();
    Serial.println("[ws] client started");
}

void loop() {
    // No motion code here on purpose -- see the header comment. Everything below
    // is allowed to block for milliseconds without affecting a single step.
    serviceNetwork();
    webSocket.loop();

    dispatchQueued();

    // One `done` per dispatched move, sent when EVERY axis it commanded has
    // stopped. Reporting per axis would send two for a diagonal move and none
    // at all for a move that ended on a limit (which raises no done_flag).
    if (movePending && !axes[AXIS_V].isMoving() && !axes[AXIS_H].isMoving()) {
        movePending = false;
        noInterrupts();
        // Only the axes this move commanded have anything to say about it.
        const StopReason vr = inFlightAxes[AXIS_V] ? axes[AXIS_V].stop_reason : STOP_NONE;
        const StopReason hr = inFlightAxes[AXIS_H] ? axes[AXIS_H].stop_reason : STOP_NONE;
        axes[AXIS_V].done_flag = false;
        axes[AXIS_H].done_flag = false;
        interrupts();
        // A limit stop is the more informative of the two to report.
        sendDone(inFlightSeq, (vr == STOP_LIMIT || hr == STOP_LIMIT)
                                  ? STOP_LIMIT
                                  : (vr != STOP_NONE ? vr : hr));
    }

    const uint32_t ms = nowMs();
    const bool moving = axes[AXIS_V].isMoving() || axes[AXIS_H].isMoving();
    if (moving) {
        lastMotionMs = ms;
        persistDirty = true;
    }

    if (ms - lastStatusMs >= STATUS_INTERVAL_MS) {
        lastStatusMs = ms;
        sendStatus();
    }

    // Park the drivers if they have been idle long enough and the feature is on.
    // Never while E-stopped (they are already off) and never with work pending.
    if (idleDisableMs > 0 && driversEnabled && !estopLatched && !moving &&
        qCount == 0 && (ms - lastMotionMs) > idleDisableMs) {
        Serial.println("[drv] idle -- de-energising");
        setDriversEnabled(false);
    }

    // Flash write, debounced well past the end of motion.
    if (persistDirty && !moving && (ms - lastMotionMs) > PERSIST_SETTLE_MS &&
        (axes[AXIS_V].position != persistedPos[AXIS_V] ||
         axes[AXIS_H].position != persistedPos[AXIS_H])) {
        savePosition();
    }
}
