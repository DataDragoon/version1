// Mirror of the Pi's quick-tune master grid and its snapping rules.
//
// The Pi does NOT report its parameters back — CLAUDE.md's invariant is that the
// panel is the source of truth and pushes to the Pi. But `SFCWEngine.set_params`
// silently SNAPS start/stop/step onto the master quick-tune grid, so the panel
// was describing a sweep that was not the one running: at a 50 MHz step it
// reported 61 steps and 1.0 m of range while the Pi swept 40 MHz, 76 steps, and
// the plot ran to 1.37 m. Everything here exists so the readouts describe reality.
//
// KEEP IN SYNC WITH pi/radar/sfcw_engine.py — QT_MASTER_STEPS, _snap_sweep,
// _round_half_up. Frequencies here are in MHz; the Pi works in Hz.

export const QT_MASTER_START_MHZ = 2000;
export const QT_MASTER_STOP_MHZ = 5000;

// Base grids the master table covers. It is their UNION, so it is deliberately
// not uniformly spaced: 2000, 2020, 2040, 2050, 2060, 2080, 2100, ...
// 20 -> 151 points, 50 -> 61, overlap (multiples of 100) -> 31, union -> 181,
// against a 256-profile hardware ceiling.
export const QT_MASTER_STEPS_MHZ = [20, 50];
export const MAX_QUICK_TUNE_PROFILES = 256;

// Python's round() is banker's; JS Math.round is half-up. The Pi uses an explicit
// half-up helper so the two agree — they used to differ on exactly the .5 cases.
const roundHalfUp = (x) => Math.floor(x + 0.5);

const clampFreq = (v) => Math.min(Math.max(v, QT_MASTER_START_MHZ), QT_MASTER_STOP_MHZ);

/**
 * Snap a requested sweep onto one of the master table's base grids.
 *
 * A sweep must stay inside ONE base: starting at 2020 (on the 20 grid) and
 * stepping 50 visits 2070, which is on neither family and is not in the table.
 * The base chosen is whichever represents the requested STEP most closely, ties
 * to the finest. Returns { startFreq, stopFreq, stepSize, base } in MHz.
 */
export function snapSweep(startFreq, stopFreq, stepSize) {
  let best = null;
  for (const base of QT_MASTER_STEPS_MHZ) {
    const snapped = Math.max(base, roundHalfUp(stepSize / base) * base);
    const err = Math.abs(snapped - stepSize);
    if (best === null || err < best.err || (err === best.err && base < best.base)) {
      best = { err, base, snapped };
    }
  }
  const snapToBase = (v) => clampFreq(roundHalfUp(v / best.base) * best.base);
  return {
    startFreq: snapToBase(startFreq),
    stopFreq: snapToBase(stopFreq),
    stepSize: best.snapped,
    base: best.base,
  };
}

/** Did the request survive snapping unchanged? */
export function sweepIsExact(startFreq, stopFreq, stepSize) {
  const s = snapSweep(startFreq, stopFreq, stepSize);
  return s.startFreq === startFreq && s.stopFreq === stopFreq && s.stepSize === stepSize;
}

/** The union grid itself, sorted — used only to size-check the table. */
export function masterGridMhz() {
  const pts = new Set();
  for (const base of QT_MASTER_STEPS_MHZ) {
    for (let f = QT_MASTER_START_MHZ; f <= QT_MASTER_STOP_MHZ; f += base) pts.add(f);
  }
  return [...pts].sort((a, b) => a - b);
}
