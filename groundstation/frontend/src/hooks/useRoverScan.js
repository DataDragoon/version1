import { useCallback, useEffect, useRef, useState } from 'react';
import {
  gridStats, roverCellForIndex, cellRoverTarget, gridRoverExtent,
  gridRoverExtentContinuous, rowTraverse, traverseOverrun,
} from '../lib/cscanGrid';

// Unique per move for the life of this page, and unique ACROSS pages: the Pi
// echoes the last token it saw, so a reloaded tab must not be able to mistake
// the previous tab's completion for its own.
const TOKEN_PREFIX = `cs${Date.now().toString(36)}${Math.random().toString(36).slice(2, 7)}`;
let tokenCounter = 0;
const nextMoveToken = () => `${TOKEN_PREFIX}-${++tokenCounter}`;

// Automated C-scan raster driven by the rover gantry.
//
// The machine is deliberately a ref + interval rather than a chain of effects:
// every transition depends on the rover's status stream, on wall-clock timers
// and on a sweep landing, and expressing that as effect dependencies produced a
// machine that re-entered itself on unrelated re-renders. One tick reading the
// latest props through a ref is far easier to reason about, and the rig is not
// something to be casually wrong about.
//
// TWO TRAVERSE MODES, and they share everything except how a row is walked:
//
//  * 'stepped'    -- the original: drive to each cell, wait for arrival, settle,
//                    take avgCount sweeps, repeat. Proven, and kept as the
//                    fallback when the continuous path needs to be ruled out.
//  * 'continuous' -- drive a whole row in ONE move and bin the sweeps that land
//                    along the way by the position they were taken at. At a
//                    27.5 ms sweep this is 2-4x faster AND gives more averaging
//                    than the stepped path ever did, because the per-cell cost
//                    used to be ~93% overhead: a 500 ms arrival gate, a settle,
//                    and one deliberately discarded in-flight sweep.
//
// Both walk the grid in exactly the same order -- rowTraverse() reproduces
// roverCellForIndex() cell for cell -- so a grid captured either way is the
// same record and feeds SAR / 2D Map / export identically. See lib/roverTrack.js
// for the sampling and smear budget that makes continuous safe, and for why the
// binning is keyed on position rather than on arrival order.

const TICK_MS = 40;

// FALLBACK ONLY, for a Pi that predates `moves_done` in its status.
//
// The board acknowledges a move -- advancing the sequence its status stream
// reports -- BEFORE dispatching it from its queue, so there is a window where
// status reads "idle, old position" for a move that has not started yet (see
// CLAUDE.md, the ideal_mm resync note). With no completion signal the only way
// to sit that window out is to wait, and half a second is roughly five status
// frames of margin on top of it.
//
// It is a pure cost: every move pays max(move time, 500 ms) instead of its own
// duration. `moves_done` removes it -- the board sends exactly one `done` per
// dispatched move, when every axis it commanded has stopped, so waiting for
// that counter to advance is exact and needs no timer at all.
export const MIN_MOVE_MS = 500;

// Floor under "the move has finished" on the FALLBACK completion paths only.
//
// `moves_done` advancing says something finished, not that OUR move did: the
// counter is snapshotted from whatever status frame we happen to hold, and a
// `done` for somebody else's move (an operator nudge, a stop, a jog ending)
// that is still in flight when the raster issues a move makes that snapshot
// stale by one -- so the very next frame reads as an instant completion, and
// arrival collapses back onto position alone. Position alone cannot tell a
// move that has not started from one that has finished, which is the whole
// reason the counter is here.
//
// A move cannot possibly be reported finished before it has reached the board
// and a status frame has come back, so requiring a link round trip plus one
// status period closes that window. The token path below is exact and skips
// it entirely.
const MOVE_ACK_FLOOR_MS = 300;

// The link is up but the Pi has stopped telling us anything. Distinct from
// `board_connected` going false, which is the Pi reporting a known state; this
// is the Pi (or the browser tab) having gone quiet while the gantry may still
// be driving. There are no endstops, so a raster must not keep issuing moves
// against a position it can no longer see.
const STATUS_STALE_MS = 4000;

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

// Sweeps free-run, so anything approaching this means the sweep died. Budget
// for one cell's capture in STEPPED mode. A cell takes `sweepsPerCell` sweeps,
// plus the one discarded to the settle window, so the allowance has to scale --
// a flat 20 s used to be plenty at one sweep per cell and would trip part-way
// through an Avg of 16.
const CAPTURE_TIMEOUT_MS = 20000;
const CAPTURE_MS_PER_SWEEP = 2000;

const IDLE = {
  active: false, phase: 'idle', index: 0, total: 0,
  cell: null, target: null, origin: null, message: null, error: null,
  row: null, rowsTotal: 0, traverse: 'stepped', anchored: false, resumeRow: 0,
};

// A stored anchor only means anything for the grid it was taken on: changing a
// count or a step re-keys every cell, so an anchor from a different geometry
// would place the raster somewhere the operator never measured. Refuse it and
// fall back to deriving a fresh one.
function originAnchorFor(anchor, grid) {
  if (!anchor || !isFinite(anchor.x) || !isFinite(anchor.y)) return null;
  const same = (a, b) => Math.abs((Number(a) || 0) - (Number(b) || 0)) < 1e-9;
  if (!same(anchor.hCount, grid.hCount) || !same(anchor.vCount, grid.vCount)
      || !same(anchor.hStep, grid.hStep) || !same(anchor.vStep, grid.vStep)) return null;
  return { x: anchor.x, y: anchor.y };
}

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
  // Continuous mode only. `rowFill[r]` is how many DISTINCT columns grid row
  // `r` (counted from the top, the order the rover walks them) already holds,
  // from lib/cscanGrid `roverRowFill`. A row emits however many cells its bins
  // filled, so the flat capture count is not a row counter -- and a COUNT of
  // non-empty rows is not one either, because it reads a row that was stopped
  // part way through as finished and starts the resume below it.
  rowFill, onRowOpen, onRowClose,
  // Where the grid's top-left corner stands in the rover's frame, if this grid
  // has already been anchored. A raster derives the origin from the operator's
  // "I am this far right of / below it" offsets, which are only true where they
  // were measured; after a stop the head is somewhere in the middle of a row,
  // so re-deriving would silently anchor the resumed grid somewhere else. The
  // anchor is stored with the scan and reused for every later session on it.
  originAnchor, onOriginAnchor,
}) {
  // Everything the tick reads, refreshed every render. The interval closes over
  // this ref, never over the props themselves.
  const optsRef = useRef(null);
  optsRef.current = {
    params, roverStatus, roverConnected, sendRover,
    sfcwRunning, onStartSweep, onStopSweep, capturedCount, onRequestCapture, sweepsPerCell,
    rowFill, onRowOpen, onRowClose, originAnchor, onOriginAnchor,
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
      row: st.row,
      rowsTotal: st.rowsTotal,
      traverse: st.traverse,
      anchored: !!st.anchored,
      resumeRow: st.resumeRow || 0,
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
    const st = machine.current;

    // Harvest a row that is still open before anything else. A row in progress
    // is a minute of driving, and the operator stopping (or a link dropping) is
    // exactly when losing it would hurt -- same reasoning as the BG-model
    // continuous capture, which harvests on ANY end rather than only on the
    // toggle. onRowClose is idempotent.
    if (st && st.rowOpen) {
      st.rowOpen = false;
      try { o.onRowClose(); } catch { /* nothing accumulated */ }
    }
    // Put the rail's speed back. It was lowered for the scan, and set_config
    // PERSISTS on the Pi, so leaving it would quietly slow every later nudge
    // and jog too.
    if (st && st.speedApplied && st.prevMaxSpeed != null) {
      try {
        o.sendRover({ cmd: 'rover_set_config', config: { x_max_speed: st.prevMaxSpeed } });
      } catch { /* link already gone */ }
    }

    halt();
    if (estop) {
      try { o.sendRover({ cmd: 'rover_estop' }); } catch { /* link already gone */ }
    }
    try { o.onStopSweep(); } catch { /* nothing to stop */ }
    setUi({ ...IDLE, phase, message, error });
  }, [halt]);

  const issueMove = useCallback((phase, target, label, timeoutOverrideMs) => {
    const o = optsRef.current;
    const st = machine.current;
    const cfg = o.roverStatus?.config;
    const clamped = clampTarget(target, cfg);
    const status = o.roverStatus;

    // Distance / speed, doubled for the ramps, plus the floor. A continuous
    // traverse passes its own budget: it is a single move of up to a whole row
    // at a deliberately reduced speed, which the config-derived estimate below
    // would under-allow once the scan speed is lower than the axis maximum.
    const dist = Math.hypot((status?.x_mm ?? 0) - clamped.x_mm, (status?.y_mm ?? 0) - clamped.y_mm);
    const speed = Math.max(1, Math.min(cfg?.x_max_speed || 150, cfg?.y_max_speed || 25));
    const timeoutMs = timeoutOverrideMs != null
      ? Math.max(MOVE_TIMEOUT_FLOOR_MS, timeoutOverrideMs)
      : Math.max(MOVE_TIMEOUT_FLOOR_MS, (dist / speed) * 1000 * 3);

    st.phase = phase;
    st.target = clamped;
    st.message = label;
    st.issuedAt = performance.now();
    st.timeoutMs = timeoutMs;
    st.idleSince = null;
    // Snapshot the completion counter. Safe to read the status we happen to
    // hold: a move is only ever issued once the previous one is confirmed
    // finished, so nothing else is in flight to advance it behind our back.
    st.movesDoneAtIssue = typeof status?.moves_done === 'number' ? status.moves_done : null;
    // Exact completion, when the Pi supports it: it echoes this token back in
    // `last_done_token` when THIS move's `done` arrives, so no other mover on
    // the link can be mistaken for us and no timer is involved. A Pi that
    // predates it simply never echoes anything, and the counter path below
    // takes over.
    st.moveToken = nextMoveToken();
    st.tokenSupported = typeof status?.last_done_token !== 'undefined';
    o.sendRover({
      cmd: 'rover_move_abs', x_mm: clamped.x_mm, y_mm: clamped.y_mm, token: st.moveToken,
    });
    publish();
  }, [publish]);

  // ── stepped ───────────────────────────────────────────────────────────────

  const gotoCell = useCallback((index) => {
    const st = machine.current;
    const cell = roverCellForIndex(index, st.grid.hCount, st.grid.vCount);
    const target = cellRoverTarget(cell.ix, cell.iy, st.grid, st.origin);
    st.index = index;
    st.cell = cell;
    issueMove('moving', target, `Cell ${index + 1} of ${st.total}`);
  }, [issueMove]);

  // ── continuous ────────────────────────────────────────────────────────────

  // Drive to the start of a row, overrun included, and park. The traverse
  // itself only begins once the settle expires, so the ramp out of this stop is
  // spent outside the grid.
  const gotoRow = useCallback((rowFromTop) => {
    const st = machine.current;
    const tr = rowTraverse(rowFromTop, st.grid, st.origin, st.overrunMm);
    st.rowFromTop = rowFromTop;
    st.rowGeom = tr;
    st.row = { index: rowFromTop, iy: tr.iy, dir: tr.dir };
    st.cell = { ix: tr.dir > 0 ? 0 : st.grid.hCount - 1, iy: tr.iy };
    st.index = rowFromTop * st.grid.hCount;
    issueMove('row_start', { x_mm: tr.entryX, y_mm: tr.y_mm },
      `Row ${rowFromTop + 1} of ${st.rowsTotal} — driving to start`);
  }, [issueMove]);

  const closeRow = useCallback(() => {
    const st = machine.current;
    const o = optsRef.current;
    if (!st || !st.rowOpen) return;
    st.rowOpen = false;
    try { o.onRowClose(); } catch { /* nothing accumulated */ }
  }, []);

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
    // A link that is up but silent. `last_status_at` is the Pi's clock, so it is
    // only ever compared with ITSELF -- what is timed locally is how long we
    // have gone without seeing it change. Nothing may be commanded against a
    // position that is no longer being reported.
    if (status.last_status_at !== st.lastStatusAt) {
      st.lastStatusAt = status.last_status_at;
      st.lastStatusSeenAt = now;
    } else if (st.lastStatusSeenAt != null && now - st.lastStatusSeenAt > STATUS_STALE_MS) {
      finish('error', null,
        `No rover status for ${(STATUS_STALE_MS / 1000).toFixed(0)} s — the link is up but silent. `
        + 'Scan aborted; check the rover server and the controller.', false);
      return;
    }

    switch (st.phase) {
      case 'homing':
      case 'moving':
      case 'row_start':
      case 'traversing': {
        const since = now - st.issuedAt;
        const idle = !status.moving
          && (status.pending_moves | 0) === 0
          && (status.queue_depth | 0) === 0;

        // "The move has finished" -- exactly, when the Pi reports it. The board
        // sends one `done` per dispatched move once every axis it commanded has
        // stopped, so this cannot fire during the ack-before-dispatch window and
        // there is nothing to wait out. Falls back to the timer on a Pi that does
        // not report it.
        // Three completion signals, best first:
        //  1. our own token echoed back -- exact, and immune to anyone else's
        //     move finishing while ours is in flight;
        //  2. `moves_done` advancing, floored by a link round trip so a `done`
        //     already in flight when we issued cannot be read as ours;
        //  3. a plain timer, for a Pi that reports neither.
        const tokenPath = st.tokenSupported && st.moveToken != null
          && typeof status.last_done_token !== 'undefined';
        const counterPath = st.movesDoneAtIssue != null && typeof status.moves_done === 'number';
        const exact = tokenPath || counterPath;
        const completed = tokenPath
          ? status.last_done_token === st.moveToken
          : counterPath
            ? (status.moves_done > st.movesDoneAtIssue && since >= MOVE_ACK_FLOOR_MS)
            : since >= MIN_MOVE_MS;

        // A move that ended as anything but `completed` did not go where it was
        // told -- a soft limit, a stop, an e-stop. Targets are already clamped
        // on both sides, so this means the geometry is wrong, and capturing a
        // row against it would put every cell in the wrong place.
        if (exact && completed && status.last_done_reason
            && status.last_done_reason !== 'completed') {
          finish('error', null,
            `Move ended as '${status.last_done_reason}' rather than completing — `
            + 'the rover is not where the grid expects it. Scan aborted.', true);
          return;
        }

        // A traverse is where the sweep has to keep running -- losing it
        // half way along a row would silently produce a half-empty row rather
        // than a failure.
        if (st.phase === 'traversing') {
          if (o.sfcwRunning) st.sawRunning = true;
          else if (st.sawRunning) {
            finish('error', null, 'Sweep stopped mid-row — the partial row was kept.', false);
            return;
          }
        }

        if (completed && idle) {
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
            } else if (st.phase === 'traversing') {
              closeRow();
              const next = st.rowFromTop + 1;
              if (next >= st.rowsTotal) {
                finish('done', `Grid complete — ${st.rowsTotal} rows scanned.`, null, false);
              } else {
                gotoRow(next);
              }
            } else {
              // 'moving' (stepped) and 'row_start' (continuous) both settle
              // before doing anything; what happens after differs.
              st.phase = st.traverse === 'continuous' ? 'row_settle' : 'settling';
              st.settleUntil = now + st.settleMs;
              publish();
            }
            return;
          }
          // Idle but not there. Give it a moment in case a queued move is still
          // in flight, then treat it as a real failure rather than capturing at
          // the wrong place.
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

      // Continuous: the settle is spent parked at the row's ENTRY point, which
      // is already outside the grid, so the ramp that follows costs no cells.
      case 'row_settle':
        if (now >= st.settleUntil) {
          const tr = st.rowGeom;
          st.rowOpen = true;
          o.onRowOpen({
            iy: tr.iy,
            rowFromTop: tr.rowFromTop,
            dir: tr.dir,
            hCount: st.grid.hCount,
            hStepMm: st.grid.hStep * 10,
            originXMm: st.origin.x,
            y_mm: tr.y_mm,
          });
          st.sawRunning = false;
          // The whole row in one move. Absolute, so quantisation cannot
          // accumulate across rows (see the ideal_mm note in rover_server).
          const span = Math.abs(tr.exitX - tr.entryX);
          issueMove('traversing', { x_mm: tr.exitX, y_mm: tr.y_mm },
            `Row ${tr.rowFromTop + 1} of ${st.rowsTotal} — scanning`,
            (span / Math.max(1, st.speedMmS)) * 1000 * 3 + 10000);
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
  }, [finish, gotoCell, gotoRow, closeRow, publish]);

  const start = useCallback(() => {
    const o = optsRef.current;
    const status = o.roverStatus;
    const fail = (msg) => setUi({ ...IDLE, phase: 'error', error: msg });

    if (!o.roverConnected || !status || !status.board_connected) {
      return fail('Rover controller is not connected.');
    }
    if (status.estop) return fail('E-stop is latched — clear it before scanning.');

    // THE ROVER MUST BE AT REST. The origin is derived from where the head is
    // standing right now, so reading it off a rig that is still moving anchors
    // the whole grid wherever the last status frame happened to catch it --
    // and then every cell in the scan is somewhere other than the operator
    // measured. Reproduced on the simulator: pressing Start 200 ms into a
    // 600 mm nudge anchored the grid 1.5 mm off and the homing move timed out
    // chasing a target the rover was driving away from.
    if (status.moving || (status.pending_moves | 0) > 0 || (status.queue_depth | 0) > 0) {
      return fail('The rover is still moving — wait for it to stop before starting. '
                  + 'The grid origin is measured from where the head is standing.');
    }

    const grid = { ...o.params };
    const stats = gridStats(grid);
    const continuous = grid.roverTraverse !== 'stepped';
    const cfg = status.config;

    const speedMmS = Math.max(1, Number(grid.roverSpeedMmS) || 60);
    // The pitch is part of the overrun: cells are keyed by rounding position to
    // the nearest column, so a run-up shorter than half a pitch is inside the
    // first column rather than outside the grid.
    const overrunMm = continuous
      ? traverseOverrun(speedMmS, cfg?.x_accel || 500, (Number(grid.hStep) || 0) * 10)
      : 0;

    // Where a continuous raster picks up. The first row that is not FULL, not
    // the number of rows holding anything: a row stopped part way through would
    // otherwise count as done and be abandoned half empty.
    const fill = Array.isArray(o.rowFill) ? o.rowFill : [];
    const hCount = Math.max(1, grid.hCount);
    let resumeRow = 0;
    while (resumeRow < grid.vCount && (fill[resumeRow] || 0) >= hCount) resumeRow += 1;

    if (continuous) {
      if (resumeRow >= grid.vCount) {
        return fail('The grid is already full — start a new scan first.');
      }
    } else if (Math.min(o.capturedCount, stats.total) >= stats.total) {
      return fail('The grid is already full — start a new scan first.');
    }

    // Where the rover has to stand for the grid's top-left corner.
    //
    // ANCHORED ONCE PER SCAN. The operator declares where they currently are
    // relative to that corner, which is only true where they measured it -- so
    // re-deriving it on a resume, when the head is parked wherever the last row
    // was abandoned, would put the rest of the grid somewhere else entirely.
    // The first session on an empty grid derives and publishes the anchor; every
    // later one reuses it.
    const anchored = originAnchorFor(o.originAnchor, grid);
    const origin = anchored || {
      x: status.x_mm - (Number(grid.roverOriginRightMm) || 0),
      y: status.y_mm + (Number(grid.roverOriginBelowMm) || 0),
    };

    // No endstops: refuse a grid that does not fit rather than clamping into
    // it and rastering a rectangle that is not the one on screen. A continuous
    // raster reaches further than the grid on both sides, so the overrun is
    // part of what has to fit.
    if (cfg && cfg.limits_enabled) {
      const ext = continuous
        ? gridRoverExtentContinuous(grid, origin, overrunMm)
        : gridRoverExtent(grid, origin);
      const bad = [];
      if (ext.xMin < cfg.x_min_mm || ext.xMax > cfg.x_max_mm) {
        bad.push(`X ${ext.xMin.toFixed(0)}–${ext.xMax.toFixed(0)} mm outside ${cfg.x_min_mm}–${cfg.x_max_mm}`
          + (continuous ? ` (includes ${overrunMm.toFixed(0)} mm of run-up at each end)` : ''));
      }
      if (ext.yMin < cfg.y_min_mm || ext.yMax > cfg.y_max_mm) {
        bad.push(`Y ${ext.yMin.toFixed(0)}–${ext.yMax.toFixed(0)} mm outside ${cfg.y_min_mm}–${cfg.y_max_mm}`);
      }
      if (bad.length) {
        return fail(`Grid does not fit inside the soft limits: ${bad.join('; ')}.`);
      }
    }

    // Publish the anchor only once the grid has passed the soft-limit check, so
    // a refused geometry does not leave one behind.
    if (!anchored && typeof o.onOriginAnchor === 'function') {
      o.onOriginAnchor({
        x: origin.x, y: origin.y,
        hCount: grid.hCount, vCount: grid.vCount,
        hStep: grid.hStep, vStep: grid.vStep,
      });
    }

    machine.current = {
      phase: 'homing',
      traverse: continuous ? 'continuous' : 'stepped',
      grid,
      total: stats.total,
      rowsTotal: Math.max(1, grid.vCount),
      index: continuous ? resumeRow * grid.hCount : Math.min(o.capturedCount, stats.total),
      origin,
      anchored: !!anchored,
      resumeRow,
      cell: null,
      target: null,
      message: null,
      row: null,
      rowFromTop: 0,
      rowGeom: null,
      rowOpen: false,
      speedMmS,
      overrunMm,
      // Restored by finish(), whatever ends the run.
      prevMaxSpeed: cfg ? cfg.x_max_speed : null,
      speedApplied: false,
      // Stepped captures standing still and needs the rig to have stopped
      // ringing; continuous is already moving through its run-up by the time
      // the first cell arrives, so its extra settle defaults to none.
      settleMs: continuous
        ? Math.max(0, Number(grid.roverRunupExtraMs) || 0)
        : Math.max(0, Number(grid.roverSettleMs) || 0),
      sawRunning: false,
      issuedAt: 0,
      timeoutMs: MOVE_TIMEOUT_FLOOR_MS,
      idleSince: null,
      settleUntil: 0,
      capturedBefore: 0,
      captureIssuedAt: 0,
      moveToken: null,
      tokenSupported: false,
      lastStatusAt: status.last_status_at,
      lastStatusSeenAt: performance.now(),
    };

    o.onStartSweep();
    // One move on both axes, so the rover travels left and up together.
    issueMove('homing', { x_mm: origin.x, y_mm: origin.y },
      anchored
        ? `Returning to the grid origin this scan was anchored at — resuming on row ${resumeRow + 1} of ${grid.vCount}`
        : 'Returning to grid origin');

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

    if (st.traverse === 'continuous') {
      // The rail's maximum speed IS the traverse speed -- a `move` runs at
      // whatever the axis is configured for, so the scan speed is pushed here
      // and restored by finish(). Done at the raster rather than at arming so
      // the (possibly long) drive to the origin still runs at full speed.
      if (st.prevMaxSpeed != null && Math.abs(st.prevMaxSpeed - st.speedMmS) > 1e-6) {
        o.sendRover({ cmd: 'rover_set_config', config: { x_max_speed: st.speedMmS } });
        st.speedApplied = true;
      }
      // Rows already FULL are skipped; a row stopped part way through is
      // re-driven, and its columns replace what that row already holds. Read
      // afresh here rather than trusting what arming saw, since the operator
      // may have undone cells while parked.
      const fill = Array.isArray(o.rowFill) ? o.rowFill : [];
      const hCount = Math.max(1, st.grid.hCount);
      let startRow = 0;
      while (startRow < st.rowsTotal && (fill[startRow] || 0) >= hCount) startRow += 1;
      if (startRow >= st.rowsTotal) {
        finish('done', `Grid already full — ${st.rowsTotal} rows scanned.`, null, false);
        return;
      }
      st.message = null;
      gotoRow(startRow);
      return;
    }

    // The operator may have captured cells (or pressed Undo) while parked, so
    // take the count as it stands rather than what arming saw.
    const startIndex = Math.min(o.capturedCount, st.total);
    if (startIndex >= st.total) {
      finish('done', `Grid already full — ${st.total} cells captured.`, null, false);
      return;
    }
    st.message = null;
    gotoCell(startIndex);
  }, [finish, gotoCell, gotoRow]);

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

  // Unmounting mid-scan (the tab closing, a reload) is the one exit finish()
  // never sees. The rail's maximum speed PERSISTS on the Pi, so leaving it at
  // the scan speed would quietly slow every later nudge and jog with nothing on
  // screen explaining it. Best effort -- a hard tab close may outrun the send.
  useEffect(() => () => {
    if (timer.current) clearInterval(timer.current);
    const st = machine.current;
    if (st && st.speedApplied && st.prevMaxSpeed != null) {
      try {
        optsRef.current.sendRover({ cmd: 'rover_set_config', config: { x_max_speed: st.prevMaxSpeed } });
      } catch { /* link already gone */ }
    }
    machine.current = null;
  }, []);

  return { ...ui, start, beginRaster, stop, clearStatus };
}
