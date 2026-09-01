import { useCallback, useEffect, useRef, useState } from 'react';
import { gridStats, roverCellForIndex, cellRoverTarget, gridRoverExtent } from '../lib/cscanGrid';

// Automated C-scan raster driven by the rover gantry.
//
// The machine is deliberately a ref + interval rather than a chain of effects:
// every transition depends on the rover's status stream, on wall-clock timers
// and on a sweep landing, and expressing that as effect dependencies produced a
// machine that re-entered itself on unrelated re-renders. One tick reading the
// latest props through a ref is far easier to reason about, and the rig is not
// something to be casually wrong about.

const TICK_MS = 40;

// The board acknowledges a move -- advancing the sequence its status stream
// reports -- BEFORE dispatching it from its queue, so there is a window where
// status reads "idle, old position" for a move that has not started yet (see
// CLAUDE.md, the ideal_mm resync note). Status arrives at ~11 Hz, so half a
// second is roughly five frames of margin on top of that window.
const MIN_MOVE_MS = 500;

// Half a step is 65 um on X and 2.5 um on Y, so a millimetre is far looser than
// the mechanism -- it is here to catch a move that did not happen, not to judge
// precision.
const POS_TOL_MM = 1.0;

// How long the rover may sit idle at the wrong place before we call it a
// failure rather than a slow arrival.
const POS_GRACE_MS = 3000;

// Floor under the distance-derived move timeout, so short moves still get a
// sane allowance for acceleration and link latency.
const MOVE_TIMEOUT_FLOOR_MS = 6000;

// Sweeps free-run at 3-6 Hz, so anything approaching this means the sweep died.
// Budget for one cell's capture. A cell takes `sweepsPerCell` sweeps at roughly
// 3 Hz, plus the one discarded to the settle window, so the allowance has to
// scale -- a flat 20 s used to be plenty at one sweep per cell and would trip
// part-way through an Avg of 16.
const CAPTURE_TIMEOUT_MS = 20000;
const CAPTURE_MS_PER_SWEEP = 2000;

const IDLE = {
  active: false, phase: 'idle', index: 0, total: 0,
  cell: null, target: null, origin: null, message: null, error: null,
};

function clampAxis(value, lo, hi) {
  if (lo > hi) [lo, hi] = [hi, lo];
  return Math.max(lo, Math.min(hi, value));
}

// Mirror of the Pi's own clamp in `move_to_mm`. Applied here as well so the
// arrival check compares against the position the rover will actually reach --
// the board clamps silently and still reports the move `completed`.
function clampTarget(target, cfg) {
  if (!cfg || !cfg.limits_enabled) return { ...target };
  return {
    x_mm: clampAxis(target.x_mm, cfg.x_min_mm, cfg.x_max_mm),
    y_mm: clampAxis(target.y_mm, cfg.y_min_mm, cfg.y_max_mm),
  };
}

export function useRoverScan({
  params, roverStatus, roverConnected, sendRover,
  sfcwRunning, onStartSweep, onStopSweep,
  capturedCount, onRequestCapture, sweepsPerCell,
}) {
  // Everything the tick reads, refreshed every render. The interval closes over
  // this ref, never over the props themselves.
  const optsRef = useRef(null);
  optsRef.current = {
    params, roverStatus, roverConnected, sendRover,
    sfcwRunning, onStartSweep, onStopSweep, capturedCount, onRequestCapture, sweepsPerCell,
  };

  const [ui, setUi] = useState(IDLE);
  const machine = useRef(null);
  const timer = useRef(null);

  const publish = useCallback(() => {
    const st = machine.current;
    if (!st) return;
    setUi({
      active: true,
      phase: st.phase,
      index: st.index,
      total: st.total,
      cell: st.cell,
      target: st.target,
      origin: st.origin,
      message: st.message,
      error: null,
    });
  }, []);

  const halt = useCallback(() => {
    if (timer.current) { clearInterval(timer.current); timer.current = null; }
    machine.current = null;
  }, []);

  // Ends the run and leaves the rig safe. `estop` is used for the operator's
  // own stop and for a fault we caused; a lost link cannot be e-stopped so it
  // just stops sweeping.
  const finish = useCallback((phase, message, error, estop) => {
    const o = optsRef.current;
    halt();
    if (estop) {
      try { o.sendRover({ cmd: 'rover_estop' }); } catch { /* link already gone */ }
    }
    try { o.onStopSweep(); } catch { /* nothing to stop */ }
    setUi({ ...IDLE, phase, message, error });
  }, [halt]);

  const issueMove = useCallback((phase, target, label) => {
    const o = optsRef.current;
    const st = machine.current;
    const cfg = o.roverStatus?.config;
    const clamped = clampTarget(target, cfg);
    const status = o.roverStatus;

    // Distance / speed, doubled for the ramps, plus the floor.
    const dist = Math.hypot((status?.x_mm ?? 0) - clamped.x_mm, (status?.y_mm ?? 0) - clamped.y_mm);
    const speed = Math.max(1, Math.min(cfg?.x_max_speed || 150, cfg?.y_max_speed || 25));
    const timeoutMs = Math.max(MOVE_TIMEOUT_FLOOR_MS, (dist / speed) * 1000 * 3);

    st.phase = phase;
    st.target = clamped;
    st.message = label;
    st.issuedAt = performance.now();
    st.timeoutMs = timeoutMs;
    st.idleSince = null;
    o.sendRover({ cmd: 'rover_move_abs', x_mm: clamped.x_mm, y_mm: clamped.y_mm });
    publish();
  }, [publish]);

  const gotoCell = useCallback((index) => {
    const st = machine.current;
    const cell = roverCellForIndex(index, st.grid.hCount, st.grid.vCount);
    const target = cellRoverTarget(cell.ix, cell.iy, st.grid, st.origin);
    st.index = index;
    st.cell = cell;
    issueMove('moving', target, `Cell ${index + 1} of ${st.total}`);
  }, [issueMove]);

  const tick = useCallback(() => {
    const o = optsRef.current;
    const st = machine.current;
    if (!st) return;

    const status = o.roverStatus;
    const now = performance.now();

    if (!o.roverConnected || !status || !status.board_connected) {
      finish('error', null, 'Rover link lost mid-scan — position is no longer trustworthy.', false);
      return;
    }
    if (status.estop) {
      finish('error', null, 'E-stop latched — scan aborted.', false);
      return;
    }

    switch (st.phase) {
      case 'homing':
      case 'moving': {
        const since = now - st.issuedAt;
        const idle = !status.moving
          && (status.pending_moves | 0) === 0
          && (status.queue_depth | 0) === 0;

        if (since >= MIN_MOVE_MS && idle) {
          const off = Math.hypot(status.x_mm - st.target.x_mm, status.y_mm - st.target.y_mm);
          if (off <= POS_TOL_MM) {
            if (st.phase === 'homing') {
              // Homing parks at the origin and STOPS there. The raster is a
              // separate, explicit action so the operator gets a window at a
              // known position -- with the sweep already running -- to capture
              // a background reference before the gantry starts moving.
              st.phase = 'ready';
              st.message = 'At grid origin — capture a background reference now if you want one.';
              publish();
            } else {
              st.phase = 'settling';
              st.settleUntil = now + st.settleMs;
              publish();
            }
            return;
          }
          // Idle but not there. Give it a moment in case a queued move is still
          // in flight, then treat it as a real failure rather than capturing a
          // cell at the wrong place.
          if (st.idleSince == null) st.idleSince = now;
          else if (now - st.idleSince >= POS_GRACE_MS) {
            finish('error', null,
              `Rover stopped ${off.toFixed(1)} mm from its target ` +
              `(${st.target.x_mm.toFixed(1)}, ${st.target.y_mm.toFixed(1)}) mm — scan aborted.`, true);
          }
          return;
        }
        st.idleSince = null;
        if (since > st.timeoutMs) {
          finish('error', null, 'Move timed out — the rover never reached its target.', true);
        }
        return;
      }

      // Parked at the origin, sweeping, waiting for the operator to start the
      // raster. Nothing to time out -- the checks above still watch the link
      // and the e-stop, which is the whole reason the machine stays alive.
      case 'ready':
        return;

      case 'settling':
        if (now >= st.settleUntil) {
          st.phase = 'capturing';
          st.capturedBefore = o.capturedCount;
          st.captureIssuedAt = now;
          o.onRequestCapture(st.cell, { x: status.x_mm, y: status.y_mm }, st.target);
          publish();
        }
        return;

      case 'capturing':
        if (o.capturedCount > st.capturedBefore) {
          const next = st.index + 1;
          if (next >= st.total) {
            finish('done', `Grid complete — ${st.total} cells captured.`, null, false);
          } else {
            gotoCell(next);
          }
          return;
        }
        if (o.sfcwRunning) st.sawRunning = true;
        else if (st.sawRunning) {
          finish('error', null, 'Sweep stopped before the cell was captured.', false);
          return;
        }
        const captureBudget = CAPTURE_TIMEOUT_MS + CAPTURE_MS_PER_SWEEP * Math.max(0, (o.sweepsPerCell || 1) - 1);
        if (now - st.captureIssuedAt > captureBudget) {
          finish('error', null, 'No sweep arrived — is the SDR still sweeping?', false);
        }
        return;

      default:
        return;
    }
  }, [finish, gotoCell, publish]);

  const start = useCallback(() => {
    const o = optsRef.current;
    const status = o.roverStatus;
    const fail = (msg) => setUi({ ...IDLE, phase: 'error', error: msg });

    if (!o.roverConnected || !status || !status.board_connected) {
      return fail('Rover controller is not connected.');
    }
    if (status.estop) return fail('E-stop is latched — clear it before scanning.');

    const grid = { ...o.params };
    const stats = gridStats(grid);
    const startIndex = Math.min(o.capturedCount, stats.total);
    if (startIndex >= stats.total) {
      return fail('The grid is already full — start a new scan first.');
    }

    // Where the rover has to stand for the grid's top-left corner. The operator
    // declares where they currently are relative to that corner, so the origin
    // is behind them: left by however far right of it they are, up by however
    // far below it they are.
    const origin = {
      x: status.x_mm - (Number(grid.roverOriginRightMm) || 0),
      y: status.y_mm + (Number(grid.roverOriginBelowMm) || 0),
    };

    // No endstops: refuse a grid that does not fit rather than clamping into
    // it and rastering a rectangle that is not the one on screen.
    const cfg = status.config;
    if (cfg && cfg.limits_enabled) {
      const ext = gridRoverExtent(grid, origin);
      const bad = [];
      if (ext.xMin < cfg.x_min_mm || ext.xMax > cfg.x_max_mm) {
        bad.push(`X ${ext.xMin.toFixed(0)}–${ext.xMax.toFixed(0)} mm outside ${cfg.x_min_mm}–${cfg.x_max_mm}`);
      }
      if (ext.yMin < cfg.y_min_mm || ext.yMax > cfg.y_max_mm) {
        bad.push(`Y ${ext.yMin.toFixed(0)}–${ext.yMax.toFixed(0)} mm outside ${cfg.y_min_mm}–${cfg.y_max_mm}`);
      }
      if (bad.length) {
        return fail(`Grid does not fit inside the soft limits: ${bad.join('; ')}.`);
      }
    }

    machine.current = {
      phase: 'homing',
      grid,
      total: stats.total,
      index: startIndex,
      origin,
      cell: null,
      target: null,
      message: null,
      settleMs: Math.max(0, Number(grid.roverSettleMs) || 0),
      sawRunning: false,
      issuedAt: 0,
      timeoutMs: MOVE_TIMEOUT_FLOOR_MS,
      idleSince: null,
      settleUntil: 0,
      capturedBefore: 0,
      captureIssuedAt: 0,
    };

    o.onStartSweep();
    // One move on both axes, so the rover travels left and up together.
    issueMove('homing', { x_mm: origin.x, y_mm: origin.y }, 'Returning to grid origin');

    if (timer.current) clearInterval(timer.current);
    timer.current = setInterval(() => tick(), TICK_MS);
  }, [issueMove, tick]);

  // Second half of the start: begin the raster from the origin the arming run
  // parked on. Only valid from 'ready' -- pressing it at any other time would
  // race the state machine.
  const beginRaster = useCallback(() => {
    const st = machine.current;
    if (!st || st.phase !== 'ready') return;
    const o = optsRef.current;
    // The operator may have captured cells (or pressed Undo) while parked, so
    // take the count as it stands rather than what arming saw.
    const startIndex = Math.min(o.capturedCount, st.total);
    if (startIndex >= st.total) {
      finish('done', `Grid already full — ${st.total} cells captured.`, null, false);
      return;
    }
    st.message = null;
    gotoCell(startIndex);
  }, [finish, gotoCell]);

  // The operator's stop is an emergency stop: it latches, and it is meant to.
  const stop = useCallback(() => {
    if (machine.current) finish('stopped', 'Scan stopped — E-stop latched.', null, true);
    else {
      try { optsRef.current.sendRover({ cmd: 'rover_estop' }); } catch { /* no link */ }
      try { optsRef.current.onStopSweep(); } catch { /* nothing running */ }
      setUi({ ...IDLE, phase: 'stopped', message: 'E-stop latched.' });
    }
  }, [finish]);

  const clearStatus = useCallback(() => setUi(IDLE), []);

  useEffect(() => () => { if (timer.current) clearInterval(timer.current); }, []);

  return { ...ui, start, beginRaster, stop, clearStatus };
}
