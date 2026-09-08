"""Rover control server -- ws://0.0.0.0:9002.

    groundstation --ws:9002--> rover_server --ws:8765--> Arduino UNO R4

The Arduino is a WebSocket *client*: it dials into the Pi. This process is the
server for both links and is the only place rover calibration lives.

WHAT CHANGED, AND WHY MOST OF THIS FILE GOT SMALLER (2026-08-29)
---------------------------------------------------------------
The previous firmware ran its steppers from loop() and, so that WiFi would not
disturb them, refused to service the socket while moving. The board was deaf for
the whole duration of every move: it could not be stopped, could not be queried,
and silently discarded anything that arrived mid-move.

Everything this file used to do was a workaround for that. Click-and-hold was a
train of small discrete moves, because a continuous jog could not have been
stopped. Moves were paced by an *estimated* travel time so the board's buffer
could not overflow. An ack-grace timeout guessed at completion, and a heuristic
gave up on acks entirely for boards that never replied. Position was dead
reckoned from what we hoped had happened.

The firmware now steps from a timer ISR and stays responsive throughout, so all
of that is gone: `enforce_move_time`, `ack_grace_s`, `move_overhead_s`,
`ACK_GIVEUP`, the jog train, and the commanded-vs-confirmed split.

POSITION
--------
Position is no longer dead reckoned here at all. The board reports its own step
counter and this file converts it to mm. One coordinate frame end to end -- steps,
positive = up and right, origin wherever the operator last declared it -- so
there are no offsets to keep in sync. Axis direction sense moved into the
firmware (config.h V_DIR_INVERT / H_DIR_INVERT), which is why `invert_x` and
`invert_y` no longer exist here.

The remaining error sources are honest ones:

* **Quantisation.** Bounded, not accumulating: moves are commanded as ABSOLUTE
  step targets wherever possible, so the error is always <= half a step of the
  final position (65 um horizontally, 2.5 um vertically) no matter how many
  moves have been made. The old firmware rounded every RELATIVE move
  independently and discarded the remainder, which at a 1 mm jog quantum was a
  systematic +3.67% on the horizontal axis -- invisible at the coarse jog steps
  it was tested with, and 3.7 mm of error over a 100 mm raster.
* **Calibration.** The horizontal axis rolls on 66 mm wheels; its steps/mm is a
  measured quantity, and the effective rolling diameter under load is slightly
  under the caliper diameter. `rover_calibrate` corrects it from a commanded
  vs. measured distance. This is now the dominant horizontal error term.
* **Slip and missed steps.** Unobservable without an encoder. What we can do is
  report the odometer since the last declared position, which is the exposure.

Axis names: the groundstation says x (horizontal) and y (vertical); the board
says h and v. `BOARD_AXIS` is the only place that mapping lives.
"""

import argparse
import asyncio
import json
import math
import os
import signal
import time
from collections import deque

import websockets

PORT = 9002
ARDUINO_PORT = 8765

# Depth of this server's own move queue. Beyond this a caller is so far ahead of
# the machine that silently buffering more would hide a problem.
OUTBOX_MAX = 64
# Leave one slot free in the board's four-deep queue, so a jog or a stop issued
# in the same instant is never the command that gets rejected.
BOARD_QUEUE_ROOM = 3

# groundstation axis -> firmware axis
BOARD_AXIS = {'x': 'h', 'y': 'v'}

LOG_LINES = 80
STATE_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'rover_state.json')

# Mirrors StopReason in rover/motion_core.h. Kept in sync by hand; the firmware
# sends the integer.
STOP_REASON = {
    0: 'none', 1: 'completed', 2: 'requested',
    3: 'limit', 4: 'watchdog', 5: 'estop',
}
AXIS_MODE = {0: 'idle', 1: 'move', 2: 'jog', 3: 'stopping'}

DEFAULT_CONFIG = {
    # Vertical: leadscrew, 2 mm pitch x 4 start = 8 mm/rev at 1600 steps/rev.
    # Exact by construction, so this one should never need calibrating.
    'y_steps_per_mm': 200.0,
    # Horizontal: 66 mm wheels (caliper-measured). Expected to be calibrated --
    # a loaded rubber wheel rolls on slightly less than its free diameter.
    'x_steps_per_mm': 1600.0 / (math.pi * 66.0),   # 7.7166

    # mm/s and mm/s^2. Chosen against the real travel so stopping distance stays
    # small next to a ~100 mm scan span: horizontal 22.5 mm at full speed and
    # 0.4 mm at jog speed, vertical 3.1 mm and 0.13 mm.
    'x_max_speed': 150.0,
    'x_jog_speed': 60.0,
    'x_accel': 500.0,
    'y_max_speed': 25.0,
    'y_jog_speed': 15.0,
    'y_accel': 100.0,

    # There are no endstops, so these are the only thing between a jog and the
    # end of the rail. Buffered inside the true travel (vertical 1 m,
    # horizontal 4 m). Pushed to the board as well, so they survive this
    # process crashing or the link dropping.
    'limits_enabled': True,
    'x_min_mm': 0.0,
    'x_max_mm': 3900.0,
    'y_min_mm': 150.0,
    'y_max_mm': 850.0,

    # How long each jog heartbeat buys. The board stops on its own if refreshes
    # stop arriving, so this bounds how far a dropped link can carry the rover:
    # at the 20 mm/s jog speed, 500 ms is 10 mm plus 0.4 mm of stopping.
    'jog_hold_ms': 500,

    # De-energise the drivers after this many seconds of no motion. 0 = never,
    # and that is the deliberate default: with no endstop and no encoder, an axis
    # that creeps while de-energised is silently in the wrong place and there is
    # no way to notice. Turn it on if the standstill whine from the A4988-class
    # drivers, or their holding-current heat, matters more than that.
    'idle_disable_s': 0.0,
}

# Inclusive (min, max) per numeric setting. Typed into a panel field, an
# out-of-range number does not look wrong on screen -- an earlier session was
# found running a 500 mm jog quantum -- so every value is clamped here and again
# when loaded from disk.
CONFIG_BOUNDS = {
    'x_steps_per_mm': (0.01, 10000.0),
    'y_steps_per_mm': (0.01, 10000.0),
    'x_max_speed': (0.1, 1000.0),
    'x_jog_speed': (0.1, 1000.0),
    'x_accel': (1.0, 20000.0),
    'y_max_speed': (0.1, 1000.0),
    'y_jog_speed': (0.1, 1000.0),
    'y_accel': (1.0, 20000.0),
    'x_min_mm': (-10000.0, 10000.0),
    'x_max_mm': (-10000.0, 10000.0),
    'y_min_mm': (-10000.0, 10000.0),
    'y_max_mm': (-10000.0, 10000.0),
    'jog_hold_ms': (100.0, 5000.0),
    'idle_disable_s': (0.0, 3600.0),
}


class Rover:
    def __init__(self, persist=True):
        self.persist = persist
        self.clients = set()
        self.board = None
        self.board_since = None

        self.config = dict(DEFAULT_CONFIG)

        # Live mirror of the firmware's status stream. `steps` is authoritative
        # -- nothing here integrates commands to guess at position.
        self.steps = {'x': 0, 'y': 0}
        self.speed_steps = {'x': 0.0, 'y': 0.0}
        self.mode = {'x': 'idle', 'y': 'idle'}
        self.stop_reason = {'x': 'none', 'y': 'none'}
        self.moving = False
        self.estop = False
        self.enabled = False
        self.queue_depth = 0
        self.board_fw = None
        self.board_pos_valid = False
        self.last_status_at = None

        # Definitive move completion. The board sends exactly ONE `done` per
        # dispatched move, when every axis it commanded has stopped -- so this
        # counter advancing is the only unambiguous "the move is finished"
        # signal there is. Everything else a client can see is a heuristic:
        # `moving` is false in the window between acknowledging a move and
        # dispatching it from the queue (the same window that makes ideal_mm
        # unresyncable), and position alone cannot tell a move that has not
        # started from one that has finished.
        #
        # Monotonic for the life of this process. A client snapshots it when it
        # issues a move and waits for it to advance, which is immune to that
        # window and needs no timer.
        self.moves_done = 0
        self.last_done_seq = None
        self.last_done_reason = None

        # Exact, per-move completion. `moves_done` advancing is unambiguous
        # about SOMETHING having finished, but a client that snapshots the
        # counter when it issues a move can snapshot a status frame that
        # predates a `done` for somebody else's move -- an operator nudge, a
        # stop, a jog ending -- and then read that stale advance as its own move
        # completing instantly. That collapses arrival detection back onto
        # position alone, which is exactly the ack-before-dispatch window the
        # counter exists to close.
        #
        # So a client may attach an opaque `token` to a move; it is carried
        # through the outbox, mapped onto the board sequence the move is
        # actually sent with, and echoed here when THAT move's `done` arrives.
        # Waiting for `last_done_token` to equal your own is immune to every
        # other mover on the link, and needs no timer.
        self.last_done_token = None
        self._move_tokens = {}          # board seq -> caller's token

        # Odometer since the last declared position: total commanded travel, in
        # mm, which is the exposure to wheel slip and missed steps. This is the
        # honest replacement for the old "unconfirmed" budget -- the link is now
        # reliable, the mechanics are what remain unobservable.
        self.travel_mm = 0.0
        self._last_steps = None

        # Where the rover would be if steps were infinitely fine, in mm. Every
        # relative move accumulates here and the STEP TARGET is rounded from
        # this, never from the current position.
        #
        # This is the whole reason a 1 mm horizontal jog no longer drifts.
        # Rounding `current_mm + delta` reproduces the old error exactly: the
        # current position is itself a whole number of steps, so the rounding
        # lands on the same side every time and 40 x 1 mm came out as 41.47 mm
        # (+3.67%, measured). Rounding against an ideal that keeps its fraction
        # bounds the error at half a step of wherever we ended up, forever.
        self.ideal_mm = {'x': 0.0, 'y': 0.0}

        # Status frames sent before the board processed our last position-setting
        # command still carry the OLD position. Applying one would rewind the
        # readout and charge the odometer for a jump that never happened.
        # Every status echoes the last command the board acted on, so ignoring
        # anything older than the command we are waiting on closes that window.
        self._status_gate_seq = 0

        # Previous reported mode per axis, so a jog ending is detected as a state
        # transition rather than inferred from position.
        #
        # An earlier version resynced the ideal whenever an idle axis sat more
        # than a step away from it, reasoning only a cut-short move could cause
        # that. It could not work: the board acknowledges a move -- which
        # advances the sequence its status stream reports -- BEFORE dispatching
        # it from its queue, so every move passed through a window reporting
        # "new sequence, idle, old position", indistinguishable from a move that
        # was cut short. The ideal was destroyed on every move and the 3.67%
        # drift came back untouched. Resync on explicit events only.
        self._prev_mode = {'x': 'idle', 'y': 'idle'}

        # Guards against configuring the same link twice. A `hello` makes this
        # server push its configuration, so a controller that answers `cfg` with
        # a `hello` would put the two in an unbounded cfg -> hello -> cfg loop.
        # The firmware no longer does that, but the guard is what makes it
        # impossible rather than merely fixed -- when this was live it produced
        # 60,727 config pushes in one short test run, and because each hello
        # resyncs the ideal position it silently reintroduced the very
        # quantisation drift the ideal exists to prevent.
        self._link_configured = False

        # Outbound move queue with flow control. The board's own queue is only
        # four deep and it REJECTS a move that does not fit -- and a rejected
        # move is the worst possible failure here, because the ideal position
        # has already advanced past a move the rover never made. Measured before
        # this existed: 400 rapid 1 mm moves tracked 34 mm short.
        #
        # So moves are held here and released only while the board has room.
        # Callers that wait for `done` (the raster does) never touch this path;
        # it is what keeps a caller that does not from corrupting position.
        self._outbox = deque()
        self._board_queue = 0

        # Cross-check against the board's own flash copy. A disagreement on
        # connect means one of the two moved while the other was not looking.
        self.saved_steps = {'x': 0, 'y': 0}
        self.position_stale = False
        self.position_conflict = None

        self._seq = 0
        self._last_error = None
        self.log = deque(maxlen=LOG_LINES)

        self._load_state()

    # ── persistence ─────────────────────────────────────────────────────────

    def _load_state(self):
        if not self.persist:
            return
        try:
            with open(STATE_FILE) as f:
                saved = json.load(f)
        except (OSError, ValueError):
            return
        for k, v in (saved.get('config') or {}).items():
            if k not in self.config:
                continue
            if isinstance(self.config[k], bool):
                self.config[k] = bool(v)
                continue
            try:
                self.config[k] = float(v)
            except (TypeError, ValueError):
                continue
        # Same clamp as a live edit: a file written before these bounds existed,
        # or edited by hand, must not reinstate a bad value on every restart.
        for k, (lo, hi) in CONFIG_BOUNDS.items():
            if not (lo <= self.config[k] <= hi):
                self._note(f"saved {k}={self.config[k]} outside [{lo}, {hi}] "
                           f"-- reset to {DEFAULT_CONFIG[k]}")
                self.config[k] = DEFAULT_CONFIG[k]
        pos = saved.get('steps') or {}
        for ax in ('x', 'y'):
            if isinstance(pos.get(ax), int):
                self.saved_steps[ax] = pos[ax]
                self.steps[ax] = pos[ax]
        self.position_stale = True
        self._note(f"state restored: x={self.mm('x'):.3f} y={self.mm('y'):.3f} mm "
                   f"(stale until the board confirms or the operator re-declares)")

    def _save_state(self):
        if not self.persist:
            return
        try:
            tmp = STATE_FILE + '.tmp'
            with open(tmp, 'w') as f:
                json.dump({'config': self.config, 'steps': dict(self.steps),
                           'saved_at': time.time()}, f)
            os.replace(tmp, STATE_FILE)
        except OSError as e:
            self._note(f"WARNING: could not save state ({e})")

    # ── unit conversion ─────────────────────────────────────────────────────

    def spmm(self, axis):
        return self.config[f'{axis}_steps_per_mm']

    def mm(self, axis):
        return self.steps[axis] / self.spmm(axis)

    def to_steps(self, axis, mm):
        return int(round(mm * self.spmm(axis)))

    def limit_steps(self, axis):
        lo = self.to_steps(axis, self.config[f'{axis}_min_mm'])
        hi = self.to_steps(axis, self.config[f'{axis}_max_mm'])
        return (lo, hi) if lo <= hi else (hi, lo)

    # ── logging ─────────────────────────────────────────────────────────────

    def _note(self, line):
        entry = {'t': time.time(), 'line': line}
        self.log.append(entry)
        print(f"[rover] {line}", flush=True)
        return entry

    # ── status ──────────────────────────────────────────────────────────────

    def status(self):
        return {
            'board_connected': self.board is not None,
            'board_since': self.board_since,
            'board_fw': self.board_fw,
            'x_mm': round(self.mm('x'), 4),
            'y_mm': round(self.mm('y'), 4),
            'x_steps': self.steps['x'],
            'y_steps': self.steps['y'],
            'x_speed_mm_s': round(self.speed_steps['x'] / self.spmm('x'), 3),
            'y_speed_mm_s': round(self.speed_steps['y'] / self.spmm('y'), 3),
            'x_mode': self.mode['x'],
            'y_mode': self.mode['y'],
            'x_stop_reason': self.stop_reason['x'],
            'y_stop_reason': self.stop_reason['y'],
            'moving': self.moving,
            'estop': self.estop,
            'enabled': self.enabled,
            'queue_depth': self.queue_depth,
            'pending_moves': len(self._outbox),
            'travel_mm': round(self.travel_mm, 2),
            'position_stale': self.position_stale,
            'position_conflict': self.position_conflict,
            'board_pos_valid': self.board_pos_valid,
            'last_status_at': self.last_status_at,
            'moves_done': self.moves_done,
            'last_done_seq': self.last_done_seq,
            'last_done_reason': self.last_done_reason,
            'last_done_token': self.last_done_token,
            'last_error': self._last_error,
            'config': dict(self.config),
        }

    async def _fanout(self, msg):
        if not self.clients:
            return
        dead = set()
        for c in self.clients:
            try:
                await c.send(msg)
            except websockets.ConnectionClosed:
                dead.add(c)
        self.clients -= dead

    async def broadcast(self):
        await self._fanout(json.dumps({'type': 'rover_status', **self.status()}))

    async def broadcast_log(self, entry):
        await self._fanout(json.dumps({'type': 'rover_log', **entry}))

    async def broadcast_error(self, message):
        await self._fanout(json.dumps({'type': 'rover_error', 'message': message}))

    # ── board link ──────────────────────────────────────────────────────────

    def next_seq(self):
        self._seq += 1
        return self._seq

    async def send_board(self, **cmd):
        if self.board is None:
            raise RuntimeError("rover controller is not connected")
        # NOT setdefault: its default argument is evaluated eagerly, so it would
        # burn a sequence number on every jog heartbeat even though those carry
        # their own seq. Sequence numbers were jumping by ~380 per move.
        if 'seq' not in cmd:
            cmd['seq'] = self.next_seq()
        payload = json.dumps(cmd)
        try:
            await self.board.send(payload)
        except Exception as e:
            raise RuntimeError(f"send failed: {e}")
        # jog_hold arrives several times a second; logging each one would bury
        # everything else.
        if cmd.get('c') != 'jog_hold':
            entry = self._note(f"pi -> board: {payload}")
            await self.broadcast_log(entry)
        return cmd['seq']

    async def push_config(self):
        """Sends the full configuration to the board, in the board's units.

        Pushed on every connect and after every config change. The board holds
        limits and speeds itself so that they still apply when this process is
        not there to enforce them -- with no endstops, that matters.
        """
        x_lo, x_hi = self.limit_steps('x')
        y_lo, y_hi = self.limit_steps('y')
        await self.send_board(
            c='cfg',
            h_speed=self.config['x_max_speed'] * self.spmm('x'),
            h_jog=self.config['x_jog_speed'] * self.spmm('x'),
            h_accel=self.config['x_accel'] * self.spmm('x'),
            h_lo=x_lo, h_hi=x_hi,
            v_speed=self.config['y_max_speed'] * self.spmm('y'),
            v_jog=self.config['y_jog_speed'] * self.spmm('y'),
            v_accel=self.config['y_accel'] * self.spmm('y'),
            v_lo=y_lo, v_hi=y_hi,
            limits=bool(self.config['limits_enabled']),
            idle_ms=int(self.config['idle_disable_s'] * 1000),
        )

    def _ingest_status(self, msg):
        if int(msg.get('seq', 0)) < self._status_gate_seq:
            return False        # predates the command we are waiting on
        new = {'x': int(msg.get('h_pos', self.steps['x'])),
               'y': int(msg.get('v_pos', self.steps['y']))}
        if self._last_steps is not None:
            for ax in ('x', 'y'):
                self.travel_mm += abs(new[ax] - self._last_steps[ax]) / self.spmm(ax)
        self._last_steps = dict(new)
        self.steps = new
        self.speed_steps['x'] = float(msg.get('h_spd', 0.0))
        self.speed_steps['y'] = float(msg.get('v_spd', 0.0))
        self.mode['x'] = AXIS_MODE.get(int(msg.get('h_mode', 0)), '?')
        self.mode['y'] = AXIS_MODE.get(int(msg.get('v_mode', 0)), '?')
        self.stop_reason['x'] = STOP_REASON.get(int(msg.get('h_stop', 0)), '?')
        self.stop_reason['y'] = STOP_REASON.get(int(msg.get('v_stop', 0)), '?')
        self.moving = bool(msg.get('moving', False))
        self.estop = bool(msg.get('estop', False))
        self.enabled = bool(msg.get('en', False))
        if 'pos_valid' in msg:
            self.board_pos_valid = bool(msg['pos_valid'])
        self.queue_depth = int(msg.get('q', 0))
        # The board's count is authoritative; our optimistic one only bridges
        # the gap between sending and the next status.
        self._board_queue = self.queue_depth + (1 if self.moving else 0)
        self.last_status_at = time.time()

        # If anything cut a move short -- a soft limit, a stop, an E-stop -- the
        # ideal has run ahead of where the rover actually is. Snap it back, or
        # the next relative move would silently try to make up the difference.
        # Half a step of disagreement is normal and expected; more is not.
        # Resync the ideal when an axis comes to rest after a JOG. A jog moves
        # the rover without going through the ideal at all, so afterwards the
        # ideal is simply stale and the next relative move must start from
        # wherever the operator actually left it.
        for ax in ('x', 'y'):
            if self._prev_mode[ax] in ('jog', 'stopping') and self.mode[ax] == 'idle':
                self.ideal_mm[ax] = self.mm(ax)
            self._prev_mode[ax] = self.mode[ax]
        return True

    async def _ingest_hello(self, msg):
        self.board_fw = msg.get('fw')
        self.board_pos_valid = bool(msg.get('pos_valid', False))
        board_steps = {'x': int(msg.get('h_pos', 0)), 'y': int(msg.get('v_pos', 0))}

        # Cross-check the board's flash copy against ours. Neither side wins
        # automatically: a disagreement means something moved while the other
        # was not watching, and only the operator can say which is right.
        self.position_conflict = None
        if self.board_pos_valid and self.persist:
            dx = abs(board_steps['x'] - self.saved_steps['x']) / self.spmm('x')
            dy = abs(board_steps['y'] - self.saved_steps['y']) / self.spmm('y')
            if dx > 0.5 or dy > 0.5:
                self.position_conflict = {
                    'board_x_mm': round(board_steps['x'] / self.spmm('x'), 3),
                    'board_y_mm': round(board_steps['y'] / self.spmm('y'), 3),
                    'saved_x_mm': round(self.saved_steps['x'] / self.spmm('x'), 3),
                    'saved_y_mm': round(self.saved_steps['y'] / self.spmm('y'), 3),
                }
                self._note(f"position conflict on connect: board says "
                           f"({self.position_conflict['board_x_mm']}, "
                           f"{self.position_conflict['board_y_mm']}) mm, this Pi saved "
                           f"({self.position_conflict['saved_x_mm']}, "
                           f"{self.position_conflict['saved_y_mm']}) mm -- re-declare it")

        if self.board_pos_valid:
            self.steps = board_steps
            self._status_gate_seq = 0
        else:
            # Board booted without a usable position. Restore ours to it, so
            # both sides agree, and keep the stale flag up.
            self._note("board has no valid position -- restoring this Pi's last "
                       "known one; verify it against the rig")
            self._status_gate_seq = await self.send_board(
                c='set_pos', h=self.steps['x'], v=self.steps['y'])
        self._last_steps = dict(self.steps)
        self.ideal_mm = {'x': self.mm('x'), 'y': self.mm('y')}
        if not self._link_configured:
            self._link_configured = True
            await self.push_config()
        else:
            self._note("ignoring a repeat hello on an already-configured link "
                       "(the controller should send hello only on connect)")

    async def board_handler(self, ws):
        if self.board is not None:
            self._note("second controller connected -- replacing the previous link")
            try:
                await self.board.close()
            except Exception:
                pass
        self.board = ws
        self.board_since = time.time()
        self._link_configured = False

        # Outbound move queue with flow control. The board's own queue is only
        # four deep and it REJECTS a move that does not fit -- and a rejected
        # move is the worst possible failure here, because the ideal position
        # has already advanced past a move the rover never made. Measured before
        # this existed: 400 rapid 1 mm moves tracked 34 mm short.
        #
        # So moves are held here and released only while the board has room.
        # Callers that wait for `done` (the raster does) never touch this path;
        # it is what keeps a caller that does not from corrupting position.
        self._outbox = deque()
        self._board_queue = 0
        self._note("rover controller connected")
        await self.broadcast()
        try:
            async for raw in ws:
                text = str(raw).strip()
                try:
                    msg = json.loads(text)
                except ValueError:
                    entry = self._note(f"board (unparsed): {text}")
                    await self.broadcast_log(entry)
                    continue

                kind = msg.get('t')
                if kind == 'status':
                    if self._ingest_status(msg):
                        await self.pump()
                        await self.broadcast()
                    continue          # 20 Hz; far too chatty for the log

                if kind == 'hello':
                    entry = self._note(f"board hello: fw={msg.get('fw')} "
                                       f"pos_valid={msg.get('pos_valid')}")
                    await self.broadcast_log(entry)
                    await self._ingest_hello(msg)
                elif kind == 'done':
                    reason = STOP_REASON.get(int(msg.get('reason', 0)), '?')
                    # Recorded BEFORE the ideal-position resync below, so the
                    # broadcast that follows carries both together.
                    self.moves_done += 1
                    self.last_done_seq = msg.get('seq')
                    self.last_done_reason = reason
                    # None for a move nobody tagged, which is the honest answer:
                    # it says "that done was not yours" to every waiting client.
                    try:
                        done_seq = int(msg.get('seq', -1))
                    except (TypeError, ValueError):
                        done_seq = -1
                    self.last_done_token = self._move_tokens.pop(done_seq, None)
                    entry = self._note(
                        f"board done: seq={msg.get('seq')} reason={reason}")
                    await self.broadcast_log(entry)
                    # A move that ended anywhere but its target leaves the ideal
                    # ahead of reality. Take the board's word for it rather than
                    # inferring it from position.
                    if reason != 'completed':
                        self.ideal_mm = {'x': self.mm('x'), 'y': self.mm('y')}
                        self._note(f"move ended as '{reason}' -- ideal position "
                                   f"resynced to the rover's actual position")
                    self._save_state()
                elif kind == 'err':
                    self._last_error = f"{msg.get('code')}: {msg.get('msg')}"
                    entry = self._note(f"board error: {self._last_error}")
                    await self.broadcast_log(entry)
                    await self.broadcast_error(self._last_error)
                elif kind == 'ack':
                    pass
                else:
                    entry = self._note(f"board: {text}")
                    await self.broadcast_log(entry)
                await self.broadcast()
        except websockets.ConnectionClosed:
            pass
        finally:
            if self.board is ws:
                self.board = None
                self.board_since = None
                self.moving = False
                self._last_steps = None
                self._abandon_queued()
                self._note("rover controller disconnected")
                self._save_state()
                await self.broadcast()

    # ── operations ──────────────────────────────────────────────────────────

    async def jog(self, axis, direction):
        if axis not in BOARD_AXIS:
            raise ValueError("axis must be 'x' or 'y'")
        # The board drops its queue when a jog arrives, so drop ours to match --
        # otherwise queued moves would fire the moment the jog ended.
        self._abandon_queued()
        await self.send_board(c='jog', axis=BOARD_AXIS[axis],
                              dir=1 if direction >= 0 else -1,
                              hold_ms=int(self.config['jog_hold_ms']))

    async def jog_hold(self):
        if self.board is None:
            return
        await self.send_board(c='jog_hold', seq=0,
                              hold_ms=int(self.config['jog_hold_ms']))

    def _abandon_queued(self):
        """Drops queued moves and returns the ideal to the real position.

        Anything that abandons motion abandons these too, so the ideal must come
        back -- it is currently pointing at the end of moves that will never be
        made."""
        dropped = len(self._outbox)
        self._outbox.clear()
        self._board_queue = 0
        self.ideal_mm = {'x': self.mm('x'), 'y': self.mm('y')}
        if dropped:
            self._note(f"dropped {dropped} queued move(s)")

    async def stop(self):
        self._abandon_queued()
        await self.send_board(c='stop')

    async def estop_now(self):
        self._abandon_queued()
        await self.send_board(c='estop')
        # Cutting the step train at speed is exactly where a stepper can lose
        # steps, so nothing about the ideal survives an E-stop either.
        self.ideal_mm = {'x': self.mm('x'), 'y': self.mm('y')}

    async def clear_estop(self):
        await self.send_board(c='clear_estop')

    async def pump(self):
        """Releases queued moves to the board while it has room for them.

        `_board_queue` is incremented optimistically on send and corrected by the
        next status frame, which arrives at 20 Hz -- far faster than a move
        completes, so the estimate is never stale for long.
        """
        while (self._outbox and self.board is not None
               and self._board_queue < BOARD_QUEUE_ROOM):
            m = self._outbox.popleft()
            cmd = {'c': 'move', 'rel': False}
            if m['x'] is not None:
                cmd['h'] = self.to_steps('x', m['x'])
            if m['y'] is not None:
                cmd['v'] = self.to_steps('y', m['y'])
            self._board_queue += 1
            seq = await self.send_board(**cmd)
            if m.get('token') is not None:
                # Bounded: the board's queue is 4 deep and a token is only ever
                # resolved by its own `done`, but a caller that abandons moves
                # (a jog, an E-stop) leaves entries nothing will ever pop.
                if len(self._move_tokens) > 64:
                    self._move_tokens.clear()
                self._move_tokens[seq] = m['token']

    async def move_to_mm(self, x_mm=None, y_mm=None, token=None):
        """Absolute move, always sent as an absolute step target.

        Absolute targets are what keep quantisation from accumulating: the error
        is at most half a step of the requested position, however many moves have
        been made to get there.
        """
        if x_mm is None and y_mm is None:
            raise ValueError("move needs an x or a y target")

        # Clamp to the soft limits HERE, so the ideal can never point somewhere
        # the rover is not allowed to go.
        #
        # The board clamps too, but a clamped move completes normally from its
        # point of view -- it reports `completed`, not `limit` -- so nothing
        # tells the Pi the target was unreachable. Found on the rig 2026-08-29:
        # asking for 892 against an 850 limit left the ideal at 892 while the
        # rover sat at 850, so a following -100 nudge went to 792 instead of 750.
        if self.config['limits_enabled']:
            for ax, val in (('x', x_mm), ('y', y_mm)):
                if val is None:
                    continue
                lo = self.config[f'{ax}_min_mm']
                hi = self.config[f'{ax}_max_mm']
                if lo > hi:
                    lo, hi = hi, lo
                clamped = max(lo, min(hi, val))
                if clamped != val:
                    self._note(f"{ax} target {val:.3f} mm clamped to {clamped:.3f} mm "
                               f"by the soft limits")
                    if ax == 'x':
                        x_mm = clamped
                    else:
                        y_mm = clamped
        if len(self._outbox) >= OUTBOX_MAX:
            raise RuntimeError(
                f"move queue is full ({OUTBOX_MAX} waiting) -- the caller is "
                "issuing moves far faster than the rover can make them")
        # The ideal advances now because these moves WILL be made, just not yet.
        # It is the commanded trajectory, not the current position.
        if x_mm is not None:
            self.ideal_mm['x'] = x_mm
        if y_mm is not None:
            self.ideal_mm['y'] = y_mm
        self._outbox.append({'x': x_mm, 'y': y_mm, 'token': token})
        await self.pump()

    async def move_rel_mm(self, dx_mm=0.0, dy_mm=0.0):
        """Relative move, accumulated onto the ideal position (see ideal_mm).

        Note this deliberately does NOT read the current position. Doing so is
        what made repeated 1 mm steps drift: the current position is a whole
        number of steps, so rounding `current + 1 mm` rounds the same way every
        single time and the error compounds instead of cancelling.
        """
        await self.move_to_mm(
            x_mm=self.ideal_mm['x'] + dx_mm if dx_mm else None,
            y_mm=self.ideal_mm['y'] + dy_mm if dy_mm else None,
        )

    async def set_position(self, x_mm, y_mm):
        """Declare where the rover physically is. The only ground truth there is."""
        sx = self.to_steps('x', x_mm)
        sy = self.to_steps('y', y_mm)
        seq = self.next_seq()
        # All of this must land before the await below: send_board yields, and a
        # status frame processed in that window would be measured against the
        # pre-declaration position -- charging the odometer for a jump that
        # never physically happened.
        self._status_gate_seq = seq   # discard status older than this command
        self.steps = {'x': sx, 'y': sy}
        self._last_steps = dict(self.steps)
        self.ideal_mm = {'x': x_mm, 'y': y_mm}
        self.travel_mm = 0.0
        self.position_stale = False
        self.position_conflict = None
        self.saved_steps = dict(self.steps)
        await self.send_board(c='set_pos', h=sx, v=sy, seq=seq)
        self._note(f"position declared: x={x_mm} y={y_mm} mm (odometer reset)")
        self._save_state()

    async def calibrate(self, axis, commanded_mm, measured_mm):
        """Correct steps/mm from a commanded vs. measured distance.

        The horizontal axis rolls on wheels, so its steps/mm is an empirical
        number: a caliper gives the free diameter, but a loaded wheel rolls on
        slightly less. Driving a long known distance and measuring it is the
        only way to pin it down, and a long distance is what makes the
        measurement precise -- the panel says so.
        """
        if axis not in BOARD_AXIS:
            raise ValueError("axis must be 'x' or 'y'")
        if abs(measured_mm) < 1e-6 or abs(commanded_mm) < 1e-6:
            raise ValueError("commanded and measured distances must be non-zero")
        key = f'{axis}_steps_per_mm'
        old = self.config[key]
        # We asked for `commanded` and got `measured`, so each mm of real travel
        # needs the steps we actually spent divided by the distance they moved.
        new = old * (commanded_mm / measured_mm)
        lo, hi = CONFIG_BOUNDS[key]
        if not (lo <= new <= hi):
            raise ValueError(f"implied {key}={new:.4f} is outside [{lo}, {hi}]")
        ratio = new / old
        if not (0.5 <= ratio <= 2.0):
            raise ValueError(
                f"implied correction is {ratio:.3f}x, which is too large to be a "
                "calibration -- check the axis and the sign of the measurement")
        self.config[key] = new
        self._note(f"{key}: {old:.4f} -> {new:.4f} steps/mm "
                   f"({(ratio - 1) * 100:+.2f}%, commanded {commanded_mm} measured {measured_mm})")
        self._save_state()
        await self.push_config()

    def set_config(self, updates):
        changed = False
        for k, v in updates.items():
            if k not in self.config:
                continue
            if isinstance(self.config[k], bool):
                self.config[k] = bool(v)
                changed = True
                continue
            try:
                val = float(v)
            except (TypeError, ValueError):
                continue
            if val != val:
                continue
            lo, hi = CONFIG_BOUNDS.get(k, (float('-inf'), float('inf')))
            clamped = max(lo, min(hi, val))
            if clamped != val:
                self._note(f"{k}={val} outside [{lo}, {hi}] -- clamped to {clamped}")
            self.config[k] = clamped
            changed = True
        for lo_key, hi_key in (('x_min_mm', 'x_max_mm'), ('y_min_mm', 'y_max_mm')):
            if self.config[lo_key] > self.config[hi_key]:
                self.config[lo_key], self.config[hi_key] = (
                    self.config[hi_key], self.config[lo_key])
                self._note(f"{lo_key} exceeded {hi_key} -- swapped")
        if changed:
            self._save_state()
        return changed


# ── groundstation socket ────────────────────────────────────────────────────

async def client_handler(rover, ws):
    rover.clients.add(ws)
    try:
        await ws.send(json.dumps({'type': 'rover_status', **rover.status()}))
        await ws.send(json.dumps({'type': 'rover_log_history', 'lines': list(rover.log)}))
        async for raw in ws:
            try:
                cmd = json.loads(raw)
            except ValueError:
                continue
            await dispatch(rover, ws, cmd)
    except websockets.ConnectionClosed:
        pass
    finally:
        rover.clients.discard(ws)
        if not rover.clients and rover.board is not None:
            # Nobody is holding a key any more, by definition. The board's own
            # dead-man would catch this within jog_hold_ms; this makes it
            # immediate.
            try:
                await rover.stop()
            except Exception:
                pass


async def dispatch(rover, ws, cmd):
    action = cmd.get('cmd')
    try:
        if action == 'rover_get_status':
            await ws.send(json.dumps({'type': 'rover_status', **rover.status()}))
            return

        if action == 'rover_jog_hold':
            await rover.jog_hold()
            return

        if action == 'rover_set_config':
            if rover.set_config(cmd.get('config') or {}) and rover.board is not None:
                await rover.push_config()
            await rover.broadcast()
            return

        if action == 'rover_clear_error':
            rover._last_error = None
            await rover.broadcast()
            return

        # Everything past here needs the controller.
        if action == 'rover_jog_start':
            await rover.jog(cmd.get('axis'), int(cmd.get('dir', 1)))
        elif action == 'rover_jog_stop' or action == 'rover_stop':
            await rover.stop()
        elif action == 'rover_estop':
            await rover.estop_now()
        elif action == 'rover_clear_estop':
            await rover.clear_estop()
        elif action == 'rover_set_position':
            await rover.set_position(float(cmd.get('x_mm', 0)), float(cmd.get('y_mm', 0)))
        elif action == 'rover_move_rel':
            await rover.move_rel_mm(float(cmd.get('dx_mm', 0)), float(cmd.get('dy_mm', 0)))
        elif action == 'rover_move_abs':
            await rover.move_to_mm(
                x_mm=float(cmd['x_mm']) if 'x_mm' in cmd else None,
                y_mm=float(cmd['y_mm']) if 'y_mm' in cmd else None,
                token=cmd.get('token'))
        elif action == 'rover_calibrate':
            await rover.calibrate(cmd.get('axis'),
                                  float(cmd.get('commanded_mm', 0)),
                                  float(cmd.get('measured_mm', 0)))
        elif action == 'rover_enable':
            await rover.send_board(c='enable', on=bool(cmd.get('on', True)))
        else:
            return
        await rover.broadcast()

    except Exception as e:
        rover._last_error = str(e)
        await ws.send(json.dumps({'type': 'rover_error', 'message': str(e)}))
        await rover.broadcast()


async def main():
    parser = argparse.ArgumentParser(description='Rover / stepper gantry control server')
    parser.add_argument('--port', type=int, default=PORT,
                        help=f'groundstation WebSocket port (default: {PORT})')
    parser.add_argument('--arduino-port', type=int, default=ARDUINO_PORT,
                        help=f'port the controller dials in on (default: {ARDUINO_PORT})')
    parser.add_argument('--no-persist', action='store_true',
                        help='do not load or save rover_state.json')
    args = parser.parse_args()

    rover = Rover(persist=not args.no_persist)
    loop = asyncio.get_running_loop()
    stop = loop.create_future()

    def request_stop():
        if not stop.done():
            stop.set_result(None)

    loop.add_signal_handler(signal.SIGINT, request_stop)
    loop.add_signal_handler(signal.SIGTERM, request_stop)

    print(f"[rover] groundstation server on ws://0.0.0.0:{args.port}")
    print(f"[rover] waiting for controller on ws://0.0.0.0:{args.arduino_port}")

    async with websockets.serve(lambda ws: client_handler(rover, ws),
                                '0.0.0.0', args.port), \
               websockets.serve(rover.board_handler, '0.0.0.0', args.arduino_port,
                                ping_interval=20, ping_timeout=20):
        await stop

    rover._save_state()
    print("\n[rover] stopped.")


if __name__ == '__main__':
    asyncio.run(main())
