import { useCallback, useEffect, useRef, useState } from 'react';

const TICK_MS = 40;
const MIN_MOVE_MS = 500;
const POS_TOL_MM = 0.3;
const POS_GRACE_MS = 3000;
const MOVE_TIMEOUT_FLOOR_MS = 6000;
const CAPTURE_TIMEOUT_MS = 30000;
const SETTLE_MS = 200;

const IDLE = {
  active: false, phase: 'idle', index: 0, total: 0,
  message: null, error: null,
};

function clampAxis(value, lo, hi) {
  if (lo > hi) [lo, hi] = [hi, lo];
  return Math.max(lo, Math.min(hi, value));
}

export function useRoverBgScan({
  roverStatus, roverConnected, sendRover,
  sfcwRunning, onStartSweep, onStopSweep,
  captureCount, onRequestCapture, sweepsPerCapture,
}) {
  const optsRef = useRef(null);
  optsRef.current = {
    roverStatus, roverConnected, sendRover,
    sfcwRunning, onStartSweep, onStopSweep,
    captureCount, onRequestCapture, sweepsPerCapture,
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
      message: st.message,
      error: null,
    });
  }, []);

  const halt = useCallback(() => {
    if (timer.current) { clearInterval(timer.current); timer.current = null; }
    machine.current = null;
  }, []);

  const finish = useCallback((phase, message, error, estop) => {
    const o = optsRef.current;
    halt();
    if (estop) {
      try { o.sendRover({ cmd: 'rover_estop' }); } catch { /* */ }
    }
    if (phase !== 'done') {
      try { o.onStopSweep(); } catch { /* */ }
    }
    setUi({ ...IDLE, phase, message, error });
  }, [halt]);

  const issueMove = useCallback((targetX, yMm, label) => {
    const o = optsRef.current;
    const st = machine.current;
    const cfg = o.roverStatus?.config;
    const status = o.roverStatus;

    let clampedX = targetX;
    if (cfg && cfg.limits_enabled) {
      clampedX = clampAxis(targetX, cfg.x_min_mm, cfg.x_max_mm);
    }

    const dist = Math.abs((status?.x_mm ?? 0) - clampedX);
    const speed = Math.max(1, cfg?.x_max_speed || 150);
    const timeoutMs = Math.max(MOVE_TIMEOUT_FLOOR_MS, (dist / speed) * 1000 * 3);

    st.phase = 'moving';
    st.targetX = clampedX;
    st.message = label;
    st.issuedAt = performance.now();
    st.timeoutMs = timeoutMs;
    st.idleSince = null;
    o.sendRover({ cmd: 'rover_move_abs', x_mm: clampedX, y_mm: yMm });
    publish();
  }, [publish]);

  const startCapture = useCallback(() => {
    const o = optsRef.current;
    const st = machine.current;
    st.phase = 'capturing';
    st.capturedBefore = o.captureCount;
    st.captureIssuedAt = performance.now();
    st.sawRunning = false;
    st.message = `Capturing position ${st.index + 1} of ${st.total}`;
    o.onRequestCapture();
    publish();
  }, [publish]);

  const tick = useCallback(() => {
    const o = optsRef.current;
    const st = machine.current;
    if (!st) return;

    const status = o.roverStatus;
    const now = performance.now();

    if (!o.roverConnected || !status || !status.board_connected) {
      finish('error', null, 'Rover link lost — position is no longer trustworthy.', false);
      return;
    }
    if (status.estop) {
      finish('error', null, 'E-stop latched — capture aborted.', false);
      return;
    }

    switch (st.phase) {
      case 'moving': {
        const since = now - st.issuedAt;
        const idle = !status.moving
          && (status.pending_moves | 0) === 0
          && (status.queue_depth | 0) === 0;

        if (since >= MIN_MOVE_MS && idle) {
          const off = Math.abs(status.x_mm - st.targetX);
          if (off <= POS_TOL_MM) {
            st.phase = 'settling';
            st.settleUntil = now + SETTLE_MS;
            st.message = `Settling at position ${st.index + 1} of ${st.total}`;
            publish();
            return;
          }
          if (st.idleSince == null) st.idleSince = now;
          else if (now - st.idleSince >= POS_GRACE_MS) {
            finish('error', null,
              `Rover stopped ${off.toFixed(1)} mm from target — capture aborted.`, true);
          }
          return;
        }
        st.idleSince = null;
        if (since > st.timeoutMs) {
          finish('error', null, 'Move timed out — rover never reached target.', true);
        }
        return;
      }

      case 'settling':
        if (now >= st.settleUntil) {
          startCapture();
        }
        return;

      case 'capturing': {
        if (o.captureCount > st.capturedBefore) {
          const next = st.index + 1;
          if (next >= st.total) {
            finish('done', `Capture complete — ${st.total} positions.`, null, false);
          } else {
            st.index = next;
            issueMove(
              st.positions[next], st.yMm,
              `Moving to position ${next + 1} of ${st.total}`,
            );
          }
          return;
        }
        if (o.sfcwRunning) st.sawRunning = true;
        else if (st.sawRunning) {
          finish('error', null, 'Sweep stopped before capture completed.', false);
          return;
        }
        const captureBudget = CAPTURE_TIMEOUT_MS + 2000 * Math.max(0, (o.sweepsPerCapture || 40) - 1);
        if (now - st.captureIssuedAt > captureBudget) {
          finish('error', null, 'Capture timed out — is the SDR still sweeping?', false);
        }
        return;
      }

      default:
        return;
    }
  }, [finish, issueMove, startCapture, publish]);

  const start = useCallback(({ spanMm, stepMm, direction }) => {
    const o = optsRef.current;
    const status = o.roverStatus;
    const fail = (msg) => setUi({ ...IDLE, phase: 'error', error: msg });

    if (!o.roverConnected || !status || !status.board_connected) {
      return fail('Rover controller is not connected.');
    }
    if (status.estop) return fail('E-stop is latched — clear it before capturing.');
    if (spanMm <= 0) return fail('Span must be positive.');
    if (stepMm <= 0) return fail('Step must be positive.');

    const total = Math.floor(spanMm / stepMm) + 1;
    if (total < 2) return fail('Need at least 2 positions.');

    const sign = direction === 'backward' ? -1 : 1;
    const startX = status.x_mm;
    const yMm = status.y_mm;

    const positions = [];
    for (let i = 0; i < total; i++) {
      positions.push(startX + i * stepMm * sign);
    }

    const cfg = status.config;
    if (cfg && cfg.limits_enabled) {
      const xMin = Math.min(positions[0], positions[total - 1]);
      const xMax = Math.max(positions[0], positions[total - 1]);
      if (xMin < cfg.x_min_mm || xMax > cfg.x_max_mm) {
        return fail(`X range ${xMin.toFixed(0)}–${xMax.toFixed(0)} mm exceeds limits ${cfg.x_min_mm}–${cfg.x_max_mm}.`);
      }
    }

    machine.current = {
      phase: 'settling',
      positions,
      yMm,
      total,
      index: 0,
      targetX: positions[0],
      message: `Settling at position 1 of ${total}`,
      settleUntil: performance.now() + SETTLE_MS,
      sawRunning: false,
      issuedAt: 0,
      timeoutMs: MOVE_TIMEOUT_FLOOR_MS,
      idleSince: null,
      capturedBefore: 0,
      captureIssuedAt: 0,
    };

    o.onStartSweep();
    publish();

    if (timer.current) clearInterval(timer.current);
    timer.current = setInterval(() => tick(), TICK_MS);
  }, [publish, tick]);

  const stop = useCallback(() => {
    if (machine.current) finish('stopped', 'Capture stopped — E-stop latched.', null, true);
    else {
      try { optsRef.current.sendRover({ cmd: 'rover_estop' }); } catch { /* */ }
      try { optsRef.current.onStopSweep(); } catch { /* */ }
      setUi({ ...IDLE, phase: 'stopped', message: 'E-stop latched.' });
    }
  }, [finish]);

  const clearStatus = useCallback(() => setUi(IDLE), []);

  useEffect(() => () => { if (timer.current) clearInterval(timer.current); }, []);

  return { ...ui, start, stop, clearStatus };
}
