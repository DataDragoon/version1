// Continuous rover raster: position track + along-row sweep binning.
//
// The stepped raster asks "drive to the cell, stop, settle, take N sweeps".
// That flow was built when a sweep took 550 ms, so any motion during one
// smeared it across frequency and stopping was the only option. At 27.5 ms a
// sweep the constraint has inverted, and the whole per-cell overhead -- the
// 500 ms arrival gate, the settle, the discarded in-flight sweep -- is now
// ~93% of the time spent. Continuous motion deletes all of it AND hands back
// free coherent averaging, because several sweeps land inside one cell.
//
// SMEAR IS NOT THE LIMIT ANY MORE. A sweep steps frequency sequentially, so
// motion during one is a phase error bilinear in (step index, velocity):
//   * the LINEAR term is range-Doppler coupling, an apparent range SHIFT of
//     (f_start/B) * D * sin(theta) ~= 0.65 * D, where D is the distance moved
//     during the 20.9 ms RF window (51 steps x NIOS_MIN_DWELL=4096 samples at
//     10 Msps -- note that is the per-STEP dwell, not RX_BUFFER_SAMPLES=2048,
//     which is the host-driven path's DMA granularity). At 150 mm/s, the X
//     axis maximum, D = 3.1 mm and the shift is 2.0 mm against a 50 mm range
//     cell. It is zero at broadside.
//   * the QUADRATIC term is the actual defocus, and it reaches 0.39 rad at
//     150 mm/s against the ~0.79 rad (pi/4) where defocus starts to matter.
// Both are comfortably inside budget at any speed this rail can reach. At the
// old 550 ms sweep the quadratic term was 3.4 rad at 50 mm/s, which is why the
// rig had to stop.
//
// WHAT IS THE LIMIT is spatial sampling, and it is one number:
//
//     sweep spacing = v * T_sweep        (27.5 ms at the shipped NIOS sweep)
//
// so 0.69 mm at 25 mm/s, 2.75 mm at 100, 4.12 mm at 150. A grid pitch finer
// than that leaves empty cells however long the scan runs -- the sweeps were
// never taken. Note also that pitch, speed and averaging depth are ONE
// resource, not three: sweeps/cell = pitch / (v * T_sweep). Pick two.
//
// And finer is not better past a point: spatial Nyquist for the imaging is
// dx <= lambda_min/(4 sin(theta_max)), i.e. 15-21 mm at 5 GHz, so 5 mm already
// carries 3x margin. Below that, halving the pitch buys no resolution and
// costs 3 dB of per-cell SNR, because it splits the same sweeps across twice
// as many cells.
//
// TIME BASE. Both streams are already stamped on the PI's clock -- sweeps by
// sfcw_result.timestamp (stamped in _process_h_cal) and rover positions by
// rover_status.last_status_at (the Pi's ingest of a board frame). So the
// association is done against one clock and never against performance.now(),
// which would fold two independent websocket latencies into the answer.
//
// What remains is a single constant: a sweep is stamped ~14 ms AFTER its own
// phase centre, and a status frame is stamped after its WiFi transit. Their
// difference is the caller's `latencyMs` -- one scalar per speed, default 0,
// and measurable from an out-and-back pass over one row, where the spatial lag
// between the two directions is exactly 2*v*tau. It is a BIAS, not noise: its
// sign follows the direction of travel, so in a snake it displaces alternate
// rows oppositely and a straight feature comes out as a zigzag of 2*v*tau.
// Nothing in the pipeline combines rows coherently today (C-scan focusing is
// per row, SAR treats the capture as one line), so its only effect is that
// zigzag in the plan view -- but it is why the field exists.

// Rover status arrives at ~11 Hz (the firmware aims for 20; WiFi latency in
// the R4's socket stack is the suspect). 600 samples is ~55 s of history --
// comfortably more than one row at any usable speed, which is all the binning
// ever looks back over.
const TRACK_MAX = 600;

// Trimming by a quarter rather than one sample per push: splice(0, 1) on every
// frame is O(n) at 11 Hz for no reason.
const TRACK_TRIM = TRACK_MAX >> 2;

/**
 * Rover position history, queried by Pi wall-clock time.
 *
 * at() INTERPOLATES ONLY and returns null outside the samples it holds. It
 * deliberately never extrapolates: a sweep whose timestamp is newer than the
 * newest status frame is held pending by the caller until a frame arrives that
 * brackets it (~91 ms at 11 Hz), which costs nothing visible and removes a
 * whole error class. Extrapolating on the board's reported velocity would be
 * right during a constant-velocity segment and wrong by half an acceleration
 * term -- 2.07 mm at 500 mm/s^2 over one status gap -- exactly at the ends of
 * a row, where the ramps are.
 */
export function createTrack(maxSamples = TRACK_MAX) {
  let ts = [], xs = [], ys = [];

  function push({ t, x, y }) {
    if (!isFinite(t) || !isFinite(x)) return false;
    const n = ts.length;
    // Monotonic only. A frame stamped no later than the newest we hold is a
    // duplicate or a clock step; either way it would make the bracket search
    // below ambiguous, and there is nothing to gain by sorting it in.
    if (n && t <= ts[n - 1]) return false;
    ts.push(t); xs.push(x); ys.push(isFinite(y) ? y : 0);
    if (ts.length > maxSamples) {
      ts = ts.slice(TRACK_TRIM); xs = xs.slice(TRACK_TRIM); ys = ys.slice(TRACK_TRIM);
    }
    return true;
  }

  function at(time) {
    const n = ts.length;
    if (n < 2 || !isFinite(time)) return null;
    if (time < ts[0] || time > ts[n - 1]) return null;
    let lo = 0, hi = n - 1;
    while (hi - lo > 1) {
      const mid = (lo + hi) >> 1;
      if (ts[mid] <= time) lo = mid; else hi = mid;
    }
    const span = ts[hi] - ts[lo];
    const f = span > 0 ? (time - ts[lo]) / span : 0;
    return {
      x: xs[lo] + f * (xs[hi] - xs[lo]),
      y: ys[lo] + f * (ys[hi] - ys[lo]),
      // How far apart the bracketing frames were. A gap far above the nominal
      // ~91 ms means the link stuttered and the interpolation spans a stretch
      // we have no evidence about.
      gap: span,
    };
  }

  return {
    push, at,
    newest: () => (ts.length ? ts[ts.length - 1] : null),
    oldest: () => (ts.length ? ts[0] : null),
    size: () => ts.length,
    clear: () => { ts = []; xs = []; ys = []; },
  };
}

// Bound on how many sweeps one cell keeps. Every look is stored (the
// coherent/incoherent choice is a DISPLAY control and has to stay flippable
// against recorded data), so an unbounded slow pass would grow the export
// without limit. 64 is far above what any usable speed produces -- 5 mm pitch
// at 15 mm/s is 12 -- so in practice this never bites; when it does, the count
// is reported rather than silently swallowed.
const MAX_PER_CELL = 64;

/**
 * Bins one row's sweeps into grid cells by the rover's x position.
 *
 * Cells are addressed by GRID COLUMN computed from position, never by arrival
 * order: the traverse overruns both ends of the row (so the ramps fall outside
 * the grid), a stuttered link can drop a bin entirely, and the two snake
 * directions visit the same columns in opposite orders. Position is the only
 * thing that means the same in all of those.
 */
export function createRowBin({ iy, hCount, hStepMm, originXMm, maxPerCell = MAX_PER_CELL }) {
  const bins = new Map();
  let kept = 0, outside = 0, dropped = 0;

  function add(x, sample, meta) {
    // Math.round is the half-pitch rule: a sweep is credited to the cell whose
    // centre it is nearest, so |x - centre| <= hStep/2 by construction.
    const ix = Math.round((x - originXMm) / hStepMm);
    if (!(ix >= 0 && ix < hCount)) { outside += 1; return -1; }
    let b = bins.get(ix);
    if (!b) {
      // The Pi's own profile and the sweep geometry are taken from the FIRST
      // sweep to land in the cell and never repeated -- they are identical
      // across a row and storing them per sweep would multiply the export.
      b = { ix, sweeps: [], xs: [], meta };
      bins.set(ix, b);
    }
    if (b.sweeps.length >= maxPerCell) { dropped += 1; return ix; }
    b.sweeps.push(sample);
    b.xs.push(x);
    kept += 1;
    return ix;
  }

  function cells() {
    return [...bins.values()]
      .sort((a, b) => a.ix - b.ix)
      .map(b => {
        const n = b.xs.length;
        const mean = b.xs.reduce((s, v) => s + v, 0) / n;
        // Spread of the sweeps inside the cell. Not an error -- it is the
        // aperture the coherent average was taken over, which is what bounds
        // the (small) angular loss that averaging across a bin costs: at a
        // 10 mm bin that is <= 1.65 dB even at grazing, against the 8-13 dB
        // the averaging itself buys.
        const std = n > 1
          ? Math.sqrt(b.xs.reduce((s, v) => s + (v - mean) ** 2, 0) / (n - 1))
          : 0;
        return { ix: b.ix, iy, sweeps: b.sweeps, meta: b.meta, xMean: mean, xStd: std };
      });
  }

  function summary() {
    const filled = bins.size;
    // Largest run of consecutive EMPTY columns. This, not the fill fraction,
    // is what decides whether a row is usable -- the same reason the BG-model
    // continuous capture watches Hole rather than Span.
    let run = 0, worst = 0;
    for (let i = 0; i < hCount; i++) {
      if (bins.has(i)) run = 0; else { run += 1; if (run > worst) worst = run; }
    }
    return {
      iy, filled, total: hCount, holes: hCount - filled, maxHoleRun: worst,
      kept, outside, dropped,
      perCell: filled ? kept / filled : 0,
    };
  }

  return { add, cells, summary };
}

// Fallback sweep period when none has been measured yet, ms. The shipped NIOS
// autonomous sweep runs 27.1-27.5 ms at 51 steps; the panel replaces this with
// the median of the Pi's own sfcw_result timestamps as soon as it has one.
export const NOMINAL_SWEEP_MS = 27.5;

// Fraction of the sweep period spent actually stepping frequency: 51 x 4096
// samples at 10 Msps is 20.9 ms of a 27.5 ms cadence, the rest being the
// harvest and demod. Motion smear is bounded by the RF window, not the cadence.
const RF_DUTY = 20.9 / 27.5;

// Where the sweep's effective phase centre sits inside the RF window. Range-
// Doppler coupling puts the apparent range at f_start/B = 2/3 through the
// sweep rather than at its midpoint, so this is also the fraction of the
// within-sweep displacement that shows up as an apparent range shift.
const PHASE_CENTRE = 2 / 3;

/**
 * What a given speed and pitch will actually produce. Pure arithmetic, shown
 * live in the panel so an unreachable pitch is visible before a row is driven
 * rather than discovered afterwards as a field of holes.
 */
export function samplingFor(speedMmS, hStepMm, sweepPeriodMs) {
  const period = (sweepPeriodMs > 0 ? sweepPeriodMs : NOMINAL_SWEEP_MS) / 1000;
  const v = Math.max(0, speedMmS);
  const spacing = Math.max(1e-6, v * period);
  const rfMove = v * period * RF_DUTY;
  return {
    spacingMm: spacing,
    perCell: hStepMm / spacing,
    // Distance moved during the RF window, and the apparent range shift it
    // causes for a scatterer in the direction of travel (zero at broadside).
    smearMm: rfMove,
    rangeShiftMm: rfMove * PHASE_CENTRE,
  };
}

// Sweeps that may wait for a bracketing rover position before being binned. One
// status period is ~91 ms, i.e. ~3 sweeps at 36 Hz; 400 is ~11 s of sweeping,
// so this only bites if the rover link has actually stopped.
const PENDING_MAX = 400;

/**
 * The whole continuous-capture path in one object: a position track, a queue of
 * sweeps waiting to be placed, and the row currently being binned.
 *
 * Kept here rather than inlined in App so it can be driven head-first against a
 * simulated rover -- the ordering between two websockets, the pending queue and
 * the row boundaries are exactly the parts that are hard to reason about and
 * impossible to check from the UI.
 *
 * Both streams are fed with PI wall-clock times: `sfcw_result.timestamp` for a
 * sweep and `rover_status.last_status_at` for a position.
 */
export function createRowCollector() {
  const track = createTrack();
  let pending = [];
  let bin = null;
  let geom = null;
  let latencyS = 0;

  // Resolve everything the track can now bracket. Called from BOTH sockets: a
  // sweep arriving may already be placeable, and a position arriving may place
  // sweeps that were not.
  function drain() {
    if (!bin || pending.length === 0) return 0;
    let i = 0;
    for (; i < pending.length; i++) {
      const item = pending[i];
      const at = track.at(item.t - latencyS);
      // Not yet bracketed. Entries are pushed in timestamp order, so nothing
      // after this one can be resolvable either -- stop rather than scan on.
      if (at === null) break;
      // A sweep resolving outside the grid is DROPPED, not clamped: the
      // traverse deliberately overruns both ends of the row so the ramps (where
      // interpolating between status frames is wrong by half an acceleration
      // term) and the final unbracketed stretch fall outside the cells.
      bin.add(at.x, item.sample, item.meta);
    }
    if (i > 0) pending = pending.slice(i);
    return i;
  }

  return {
    setLatencyMs(ms) { latencyS = (Number(ms) || 0) / 1000; },
    pushStatus(sample) {
      if (!track.push(sample)) return false;
      drain();
      return true;
    },
    pushSweep(item) {
      if (!bin) return false;
      if (pending.length >= PENDING_MAX) pending.shift();
      pending.push(item);
      drain();
      return true;
    },
    openRow(g) {
      geom = g;
      pending = [];
      bin = createRowBin({
        iy: g.iy, hCount: g.hCount, hStepMm: g.hStepMm, originXMm: g.originXMm,
      });
    },
    // Harvest. A partial row is still data -- a row is a minute of driving, and
    // the operator stopping or a link dropping is exactly when losing it would
    // hurt. Anything still pending resolves to a position never learned, which
    // by construction lies in the overrun past the last cell: dropped, not
    // guessed. Idempotent, because the state machine calls it both on arrival
    // and again from finish().
    closeRow() {
      if (!bin) return null;
      const out = {
        geom,
        cells: bin.cells(),
        summary: { ...bin.summary(), pending: 0, stranded: pending.length, done: true },
      };
      bin = null; geom = null; pending = [];
      return out;
    },
    isOpen: () => bin !== null,
    summary: () => (bin ? { ...bin.summary(), pending: pending.length } : null),
    trackSize: () => track.size(),
    reset() { bin = null; geom = null; pending = []; track.clear(); },
  };
}
