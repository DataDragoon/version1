import { saftFocusedProfile, metricOnProfile, gateDepths } from './saft';

// C-scan grid geometry.
//
// A C-scan is a rectangular raster of B-scan positions over the wall. Capture
// follows a boustrophedon ("snake") path so the operator never has to lift and
// return: start at the bottom-left cell, sweep right along the bottom row, step
// up one row, sweep back right-to-left, step up, sweep left-to-right, and so on
// until the grid is full.
//
//   iy=2  ┌───◄───◄───◄───┐
//   iy=1  └───►───►───►───┘
//   iy=0  ┌───◄───◄───◄───┘        (iy=0 captured first, left-to-right)
//         ix=0 ............ hCount-1
//
// Each captured position stores its own grid_ix / grid_iy, so the plan view
// stays correct even if the grid dimensions are edited part-way through a scan.

// Snake path: capture order index -> grid cell. Row 0 is the bottom row and
// runs left-to-right; every subsequent row reverses.
export function cellForIndex(index, hCount) {
  const h = Math.max(1, hCount);
  const iy = Math.floor(index / h);
  const along = index % h;
  return { ix: iy % 2 === 0 ? along : h - 1 - along, iy };
}

// Inverse of cellForIndex.
export function indexForCell(ix, iy, hCount) {
  const h = Math.max(1, hCount);
  const along = iy % 2 === 0 ? ix : h - 1 - ix;
  return iy * h + along;
}

// Physical offset of a cell from the bottom-left corner of the grid, in cm.
export function cellPosition(ix, iy, hStep, vStep) {
  return { xCm: ix * hStep, yCm: iy * vStep };
}

// Everything the panel shows about the swept rectangle.
export function gridStats({ hCount, hStep, vCount, vStep }) {
  const h = Math.max(1, hCount);
  const v = Math.max(1, vCount);
  const width = hStep * (h - 1);
  const height = vStep * (v - 1);
  return {
    total: h * v,
    width,
    height,
    area: width * height,
    // Sampling is only meaningful once there is more than one line in that axis.
    hSampled: h > 1,
    vSampled: v > 1,
  };
}

// Gated intensity of one position's range profile, in dB. This is the value a
// C-scan cell is coloured by: the depth axis collapsed to a single number over
// the slice the operator selected.
//
// Works unchanged on a magnitude-difference profile (bg_sub_mode 'magnitude'),
// where the per-bin values are dB ratios rather than absolute levels:
//   - peak: the largest change in the gate -- this is the detector statistic
//     the 2026-08-28 target A/B actually found the target with (+4.4 dB).
//   - mean: the average change.
//   - energy: 10*log10(mean(10^(db/10))), i.e. the mean POWER ratio, since
//     lin*lin with lin = 10^(db/20) is exactly 10^(db/10). The same expression
//     is a mean power in absolute mode and a mean power ratio in diff mode, so
//     no special case is needed.
export function gatedIntensity(magnitudes, distances, gateStartM, gateEndM, metric) {
  if (!magnitudes || !distances) return -Infinity;
  let peak = -Infinity;
  let sumLin = 0;
  let sumDb = 0;
  let count = 0;

  for (let j = 0; j < magnitudes.length && j < distances.length; j++) {
    if (distances[j] < gateStartM || distances[j] > gateEndM) continue;
    const db = magnitudes[j];
    if (db > peak) peak = db;
    const lin = Math.pow(10, db / 20);
    sumLin += lin * lin;
    sumDb += db;
    count++;
  }
  if (count === 0) return -Infinity;

  if (metric === 'energy') return 10 * Math.log10(sumLin / count + 1e-12);
  if (metric === 'mean') return sumDb / count;
  return peak;
}

// A cell whose background could not be resolved is INVALID, not zero. Colouring
// it like any other cell is the worst available failure: an un-subtracted
// spectrum sits 20-30 dB above its subtracted neighbours, so it both reads as a
// huge target and single-handedly sets the dynamic colour limits for the whole
// grid. These statuses are set by applyBscanBg and consumed here and by the
// displays, which draw invalid cells in their own colour and exclude them from
// every scale computation.
export const BG_STATUS = {
  OFF: 'off',                  // no background selected, or subtraction disabled
  OK: 'ok',
  CLAMPED: 'clamped',          // model applied, but the standoff is outside its span
  NO_STANDOFF: 'no_standoff',  // model needs a standoff and the cell has none
  NO_REF: 'no_ref',            // reference selected but unusable
  SIZE_MISMATCH: 'size_mismatch',
  NO_SUPERFIT_CELL: 'no_superfit_cell',
};

// Did the background actually get applied to this cell? 'off' is not an error --
// nothing was asked for -- but it must not be mixed into a scale with cells that
// WERE subtracted, so the caller checks the whole population, not each cell.
export function bgFailed(status) {
  return status != null && status !== BG_STATUS.OK
    && status !== BG_STATUS.OFF && status !== BG_STATUS.CLAMPED;
}

export const BG_STATUS_TEXT = {
  [BG_STATUS.OFF]: 'no background applied',
  [BG_STATUS.OK]: 'background applied',
  [BG_STATUS.CLAMPED]: 'MODEL CLAMPED — standoff outside the captured span',
  [BG_STATUS.NO_STANDOFF]: 'INVALID — no lidar standoff, model cannot be evaluated',
  [BG_STATUS.NO_REF]: 'INVALID — reference sweep unusable',
  [BG_STATUS.SIZE_MISMATCH]: 'INVALID — background numSteps does not match this sweep',
  [BG_STATUS.NO_SUPERFIT_CELL]: 'INVALID — no Super Fit reference for this cell',
};

// ── Per-cell values, focused or not ────────────────────────────────────────
//
// ONE function producing the number every plan-view cell is coloured by, used
// by buildCscanGrid (which draws them) and by computeGridScales (which sets the
// colour limits from them). They used to call gatedIntensity separately; with
// focusing in the picture that duplication would mean the grid could be drawn
// with values the scale was not computed from, which is silently a wrong image
// rather than a crash.
//
// FOCUSING IS PER ROW, and only along the row. A C-scan row is a line of
// positions at one height, which is exactly the geometry lib/saft.js's
// back-projection assumes; rows are focused independently of each other and
// never contribute across rows. That is deliberate rather than a simplification
// -- CLAUDE.md's 2026-08-30 rover diagnosis measured the vertical axis to be a
// completely different animal from the horizontal one (moving 200 mm sideways
// costs a few dB of background correlation, moving 150 mm up destroys it
// entirely, because the standoff walks ~10 mm), so summing across rows would be
// combining traces that do not describe the same wall.
//
// Neighbours are addressed by GRID COLUMN, not by position in the array, so a
// row with holes in it (an undone cell, a partial raster) still gets the right
// spacing instead of closing the gap up.
//
// A cell whose background failed contributes to nothing: it is un-subtracted
// and sits 20-30 dB above its neighbours, so letting it into an aperture would
// smear that error across every cell within half an aperture of it.
export function computeCellValues(scanData, params) {
  const gateStartM = params.gateStart / 100;
  const gateEndM = params.gateEnd / 100;
  const { metric } = params;
  const out = new Array(scanData.length).fill(-Infinity);
  const focus = !!params.focusEnabled && params.focusAperture >= 1;

  if (!focus) {
    for (let i = 0; i < scanData.length; i++) {
      const pos = scanData[i];
      if (!pos) continue;
      out[i] = gatedIntensity(pos.magnitudes, pos.distances, gateStartM, gateEndM, metric);
    }
    return out;
  }

  const stepM = (params.hStep > 0 ? params.hStep : 1) / 100;
  const halfAp = Math.floor(params.focusAperture / 2);

  const rows = new Map();
  for (let i = 0; i < scanData.length; i++) {
    const pos = scanData[i];
    if (!pos || !pos.magnitudes || !pos.distances) continue;
    if (bgFailed(pos.bg_status)) continue;
    const iy = pos.grid_iy != null ? pos.grid_iy : 0;
    let row = rows.get(iy);
    if (!row) { row = []; rows.set(iy, row); }
    row.push({
      i,
      n: pos.grid_ix != null ? pos.grid_ix : i,
      magnitudes: pos.magnitudes,
      distances: pos.distances,
    });
  }

  for (const traces of rows.values()) {
    traces.sort((a, b) => a.n - b.n);
    // The depth axis is the record's own, so a gate that falls outside it gives
    // no depths and every cell in the row reads "outside gate" -- the same
    // answer gatedIntensity gives, rather than a focused image of nothing.
    const depths = gateDepths(traces[0].distances, gateStartM, gateEndM);
    if (depths.length === 0) continue;
    for (let k = 0; k < traces.length; k++) {
      out[traces[k].i] = metricOnProfile(
        saftFocusedProfile(traces, k, depths, stepM, halfAp), metric);
    }
  }
  return out;
}

// Fill the grid with gated intensities. Returns a hCount x vCount array indexed
// [iy * hCount + ix], holding null where nothing has been captured yet. A cell
// that was captured but has no range bin inside the gate keeps its entry with a
// non-finite value, so the display can tell "empty" from "gated out"; a cell
// whose background failed is flagged invalid and contributes to no scale.
export function buildCscanGrid(scanData, params) {
  const { hCount, vCount } = params;
  const h = Math.max(1, hCount);
  const v = Math.max(1, vCount);
  const cells = new Array(h * v).fill(null);

  let min = Infinity;
  let max = -Infinity;

  const values = computeCellValues(scanData, params);

  for (let i = 0; i < scanData.length; i++) {
    const pos = scanData[i];
    // Positions imported from a v4 (linear) B-scan carry no grid indices, so
    // lay them out along the snake path the current grid describes.
    const cell = (pos.grid_ix != null && pos.grid_iy != null)
      ? { ix: pos.grid_ix, iy: pos.grid_iy }
      : cellForIndex(i, h);
    if (cell.ix < 0 || cell.ix >= h || cell.iy < 0 || cell.iy >= v) continue;

    const invalid = bgFailed(pos.bg_status);
    const value = invalid ? NaN : values[i];
    cells[cell.iy * h + cell.ix] = { value, pos, order: i, invalid, status: pos.bg_status };
    if (invalid || !isFinite(value)) continue;
    if (value < min) min = value;
    if (value > max) max = value;
  }

  return { cells, hCount: h, vCount: v, min, max, filled: cells.filter(Boolean).length };
}

// ── Shared colour scale ────────────────────────────────────────────────────
//
// ONE set of colour limits for the whole panel, computed from every captured
// cell rather than from one grid's gated scalars and one row's bins separately.
// Before this the two panes disagreed by construction: the C-scan grid stretched
// min..max of the gated values across all cells, while the B-scan pane stretched
// min..max of every bin in the SELECTED ROW ONLY (and included the unsubtracted
// background row in that, which crushed every residual into the bottom few
// percent of the colormap). The same colour meant two different dB in two
// images shown side by side, and clicking to another row silently re-scaled the
// one on the right. Now a colour means one dB everywhere.
//
// The population is every bin the B-scan pane actually draws -- the whole
// profile, over all valid cells -- so the two panes are literally scaled to the
// same pixels. Gated cell values are aggregates over a subset of those bins, so
// they land inside the same range by construction.
//
// Limits are PERCENTILES, not min/max. A range profile has deep interference
// nulls; after background subtraction it has more of them, and a single bin at
// -140 dB would otherwise set the bottom of the scale and flatten everything
// above it. p1..p99.9 keeps a genuine bright return (the top ~5 samples of a
// 60-cell grid) while ignoring the nulls.
const SCALE_P_LO = 0.01;
const SCALE_P_HI = 0.999;

// Every bin of one cell that is allowed to vote on a colour scale. A cell whose
// background failed is excluded -- it is un-subtracted and would set the top of
// the scale on its own.
function pushCellValues(pos, out) {
  if (!pos || !pos.magnitudes || !pos.distances) return;
  if (bgFailed(pos.bg_status)) return;
  const mags = pos.magnitudes;
  const dists = pos.distances;
  for (let i = 0; i < mags.length && i < dists.length; i++) {
    const v = mags[i];
    if (isFinite(v)) out.push(v);
  }
}

// Percentile limits from an already-collected population of dB values. Sorts in
// place, so hand it a scratch array.
function limitsFrom(vals) {
  if (vals.length === 0) return { min: -90, max: -20, n: 0, degenerate: true };

  vals.sort((a, b) => a - b);
  const at = (p) => vals[Math.min(vals.length - 1, Math.max(0, Math.round(p * (vals.length - 1))))];
  let min = at(SCALE_P_LO);
  let max = at(SCALE_P_HI);

  // Degenerate populations are real and must not produce a divide-by-zero
  // rainbow: subtracting a Super Fit reference from the very grid it was taken
  // from gives an exactly-zero residual in every bin (20*log10(1e-12) = -240 dB
  // everywhere), which is a useful "this is the same data" signal rather than an
  // error, but there is nothing to stretch.
  const degenerate = !(max - min > 0.5);
  if (degenerate) {
    const mid = (max + min) / 2;
    min = mid - 0.5;
    max = mid + 0.5;
  }
  return { min, max, n: vals.length, degenerate };
}

export function computeSharedScale(scanData) {
  const vals = [];
  for (const pos of scanData) pushCellValues(pos, vals);
  return limitsFrom(vals);
}

// PER-ROW colour limits: the same percentile treatment, but with the population
// restricted to one grid row. Keyed by grid_iy, so it survives either capture
// order (the hand snake climbs from the bottom-left, the rover snake descends
// from the top-left, and both write the same indices).
//
// Why it exists: the global scale is the honest one -- a colour means one dB
// everywhere -- but on a wall whose standoff varies row to row the wall return
// itself sets the limits, and every row but the loudest is crushed into the
// bottom of the colormap. Scoping to a row gives up cross-row comparability to
// get contrast back inside each one. Both are useful; neither is a better
// version of the other, which is why it is a toggle rather than a replacement.
//
// Data with no grid indices (an imported linear scan) all lands in row 0, so
// per-row is identical to global there.
export function computeRowScales(scanData) {
  const byRow = new Map();
  for (const pos of scanData) {
    if (!pos) continue;
    const iy = pos.grid_iy != null ? pos.grid_iy : 0;
    let vals = byRow.get(iy);
    if (!vals) { vals = []; byRow.set(iy, vals); }
    pushCellValues(pos, vals);
  }
  const out = new Map();
  for (const [iy, vals] of byRow) out.set(iy, limitsFrom(vals));
  return out;
}

// UNLINKED colour limits for the plan view: the population is the GATED CELL
// VALUES -- the scalars the grid actually draws -- rather than every bin of
// every profile.
//
// The difference is the whole point of the toggle. The linked scale is built
// from all bins, gate or no gate, so narrowing the gate onto a quiet depth does
// not move it: the cells all drop into the bottom of a range still set by the
// wall return, and the grid goes near-uniformly dark exactly when it is finally
// being asked the interesting question. Scaling the grid within its own gated
// values restores the contrast, at the cost of the guarantee the linked scale
// exists to provide -- a colour then means one dB in the plan view and a
// different dB in the B-scan beside it, so both panes flag it.
//
// Returns global and per-row limits together, because the two scope choices
// need the same population and it is one pass either way. Cells with no bin
// inside the gate are skipped, not floored: they are drawn as "gated out" in
// their own colour and have no value to contribute.
export function computeGridScales(scanData, params) {
  const all = [];
  const byRow = new Map();
  // The same values the grid draws, focusing included -- see computeCellValues.
  const values = computeCellValues(scanData, params);

  for (let i = 0; i < scanData.length; i++) {
    const pos = scanData[i];
    if (!pos || !pos.magnitudes || !pos.distances) continue;
    if (bgFailed(pos.bg_status)) continue;
    const v = values[i];
    if (!isFinite(v)) continue;
    all.push(v);
    const iy = pos.grid_iy != null ? pos.grid_iy : 0;
    let vals = byRow.get(iy);
    if (!vals) { vals = []; byRow.set(iy, vals); }
    vals.push(v);
  }

  const rows = new Map();
  for (const [iy, vals] of byRow) rows.set(iy, limitsFrom(vals));
  return { global: limitsFrom(all), rows };
}

// Roll the per-cell background statuses up into something the panel can show.
export function bgDiagnostics(scanData) {
  const counts = {};
  let invalid = 0;
  let clamped = 0;
  let applied = 0;
  for (const pos of scanData) {
    const st = pos && pos.bg_status != null ? pos.bg_status : BG_STATUS.OFF;
    counts[st] = (counts[st] || 0) + 1;
    if (bgFailed(st)) invalid++;
    else if (st === BG_STATUS.CLAMPED) { clamped++; applied++; }
    else if (st === BG_STATUS.OK) applied++;
  }
  return { counts, invalid, clamped, applied, total: scanData.length };
}

// ── Rover raster ───────────────────────────────────────────────────────────
//
// Driven by the gantry rather than by hand, the natural origin is the TOP-LEFT
// corner: the rover sweeps the top row left-to-right, drops one row, sweeps
// back right-to-left, and so on downwards. That is the mirror image of the
// hand-held order above, which starts bottom-left and climbs.
//
//   row 0 (iy = vCount-1)  ORIGIN ──►───►───►──┐   captured first
//   row 1 (iy = vCount-2)  ┌──◄───◄───◄────────┘
//   row 2 (iy = vCount-3)  └──►───►───►──┐
//
// Cell coordinates stay in the display's frame (iy = 0 is the bottom row) so
// the plan view, the export and everything downstream are identical whichever
// way the raster was driven. Only the capture ORDER differs.
export function roverCellForIndex(index, hCount, vCount) {
  const h = Math.max(1, hCount);
  const v = Math.max(1, vCount);
  const row = Math.floor(index / h);      // 0 = top row = the origin's row
  const along = index % h;
  return { ix: row % 2 === 0 ? along : h - 1 - along, iy: v - 1 - row };
}

// Capture order for whichever mode is driving the raster.
export function orderedCellForIndex(index, hCount, vCount, scanMode) {
  return scanMode === 'rover'
    ? roverCellForIndex(index, hCount, vCount)
    : cellForIndex(index, hCount);
}

// Rover-frame target of a grid cell, in mm.
//
// `origin` is where the rover has to stand for the grid's top-left corner, in
// the rover's own frame (x grows right, y grows up). The grid extends right and
// DOWNWARD from there, so a cell's y is below the origin by however many rows
// it sits above the bottom of the grid. Steps are cm in the params and mm on
// the rover, hence the tens.
export function cellRoverTarget(ix, iy, params, origin) {
  const v = Math.max(1, params.vCount);
  return {
    x_mm: origin.x + ix * params.hStep * 10,
    y_mm: origin.y - (v - 1 - iy) * params.vStep * 10,
  };
}

// The rectangle the rover has to reach, in its own frame. Used to check the
// grid fits inside the soft limits before a single move is issued -- there are
// no endstops, so finding out half way through is not an option.
export function gridRoverExtent(params, origin) {
  const stats = gridStats(params);
  return {
    xMin: origin.x,
    xMax: origin.x + stats.width * 10,
    yMin: origin.y - stats.height * 10,
    yMax: origin.y,
  };
}

// ── Continuous row traverse ─────────────────────────────────────────────────
//
// A continuous raster drives a whole row in ONE move and bins the sweeps that
// land along the way, instead of stopping at every cell. See lib/roverTrack.js
// for why that is now the right thing to do and what bounds it.

// How far past each end of a row the traverse runs, in mm.
//
// Two things have to fall OUTSIDE the grid, or they corrupt the cells at the
// ends of every row:
//
//  * THE RAMPS. Position between two status frames is interpolated linearly,
//    which is exact at constant velocity and wrong by half an acceleration
//    term while accelerating -- 2.07 mm at 500 mm/s^2 over one 91 ms status
//    gap, which at a 5 mm pitch is most of a cell. `v^2/2a` is the distance
//    the ramp itself occupies.
//  * THE LAST STATUS GAP. A sweep is only binned once a status frame arrives
//    that brackets it in time (the track never extrapolates), so the final
//    ~91 ms of a traverse resolves after the rover has already stopped. The
//    margin makes sure that stretch is overrun rather than grid.
//
// Cheap either way: 10.6 mm at 25 mm/s, 30 mm at 100 mm/s -- a few tenths of a
// second per row against a row that takes tens of seconds.
export function traverseOverrun(speedMmS, accelMmS2) {
  const v = Math.max(0, Number(speedMmS) || 0);
  const a = Math.max(1, Number(accelMmS2) || 500);
  const ramp = (v * v) / (2 * a);
  const margin = Math.max(10, v * 0.2);
  return ramp + margin;
}

// Geometry of one continuous row traverse, in the rover's frame.
//
// `rowFromTop` counts rows in CAPTURE order (0 = the origin's row, the top
// one), matching roverCellForIndex: even rows run left to right, odd rows run
// back right to left. Cell coordinates stay in the display frame, so `iy`
// still counts up from the bottom and everything downstream is unchanged.
export function rowTraverse(rowFromTop, params, origin, overrunMm = 0) {
  const h = Math.max(1, params.hCount);
  const v = Math.max(1, params.vCount);
  const r = Math.max(0, Math.min(v - 1, rowFromTop | 0));
  const dir = r % 2 === 0 ? 1 : -1;
  const iy = v - 1 - r;
  const stepMm = params.hStep * 10;
  const xAt = (ix) => origin.x + ix * stepMm;
  const firstX = xAt(dir > 0 ? 0 : h - 1);
  const lastX = xAt(dir > 0 ? h - 1 : 0);
  return {
    rowFromTop: r,
    iy,
    dir,
    y_mm: origin.y - r * params.vStep * 10,
    firstX,
    lastX,
    entryX: firstX - dir * overrunMm,
    exitX: lastX + dir * overrunMm,
  };
}

// The rectangle a CONTINUOUS raster actually reaches: the grid plus the
// overrun at both ends of every row. Checked against the soft limits before a
// move is issued, exactly like gridRoverExtent -- there are no endstops, and
// `move_to_mm` clamps silently while still reporting the move `completed`, so
// a traverse hanging over the end of a rail would raster a row that is not the
// one on screen and pile duplicate sweeps into the cell at the limit.
export function gridRoverExtentContinuous(params, origin, overrunMm = 0) {
  const ext = gridRoverExtent(params, origin);
  return { ...ext, xMin: ext.xMin - overrunMm, xMax: ext.xMax + overrunMm };
}

// ── Plan-view layout ────────────────────────────────────────────────────────
//
// Shared by the C-scan plan view and by the B-scan pane below it, so the two
// images line up column for column: the B-scan places each position at the x of
// the grid cell it was captured at, which is only possible if both derive that
// x from one function.
//
// `projection` selects the scale and, to scale, the placement:
//   { toScale: false }                              fit the grid inside the plot box
//   { toScale: true, pxPerCm, leftPx, topPx }       exact scale, exact placement
//
// To-scale exists for projecting the plan view back onto the wall it was swept
// over. Its whole point is that the mapping does NOT depend on the pane size,
// so the operator tunes one constant here plus the projector's own zoom and the
// image lands on the real geometry. A fitted scale would silently re-scale
// whenever the window or the pane split changed, which is exactly what makes an
// aligned projection drift.
//
// `leftPx` / `topPx` place the grid's top-left corner relative to the top-left
// of the VIEWPORT -- the whole area right of the sidebar -- not of this canvas.
// `canvasOffset` is where this canvas sits inside that viewport, and subtracting
// it is what makes the placement hold still when the Live Sweep pane appears or
// a row's B-scan opens underneath: the canvas moves, the grid does not. Measured
// from the canvas instead, every pane change would slide the projected image.
export const CSCAN_PAD = { top: 24, bottom: 38, left: 52, right: 64 };

export function cscanLayout(w, h, params, projection, canvasOffset) {
  const pad = CSCAN_PAD;
  const plotW = w - pad.left - pad.right;
  const plotH = h - pad.top - pad.bottom;

  const hCount = Math.max(1, params.hCount);
  const vCount = Math.max(1, params.vCount);
  // A single line in an axis still needs a finite cell size to draw.
  const cellSpanX = params.hStep > 0 ? params.hStep : 1;
  const cellSpanY = params.vStep > 0 ? params.vStep : 1;
  const spanX = hCount * cellSpanX;
  const spanY = vCount * cellSpanY;

  const fitScale = Math.min(plotW / spanX, plotH / spanY);
  const toScale = !!(projection && projection.toScale)
    && Number.isFinite(projection.pxPerCm) && projection.pxPerCm > 0;
  const scale = toScale ? projection.pxPerCm : fitScale;

  const gridW = spanX * scale;
  const gridH = spanY * scale;

  // Fitted, the grid is centred in the plot box and the axes get their margins.
  // To scale it is placed explicitly and may go anywhere on the canvas, so the
  // clip box is the whole canvas -- reserving margins there would silently
  // forbid placements the operator asked for.
  const off = canvasOffset || { x: 0, y: 0 };
  const leftPx = Number.isFinite(projection && projection.leftPx) ? projection.leftPx : 0;
  const topPx = Number.isFinite(projection && projection.topPx) ? projection.topPx : 0;

  const originX = toScale ? leftPx - off.x : pad.left + (plotW - gridW) / 2;
  // Vertical grows upward: iy = 0 sits at the BOTTOM of the grid box, so the
  // placed TOP edge is originY - gridH.
  const originY = toScale ? topPx - off.y + gridH : pad.top + (plotH + gridH) / 2;

  const clip = toScale
    ? { x: 0, y: 0, w, h }
    : { x: pad.left, y: pad.top, w: plotW, h: plotH };

  return {
    pad, plotW, plotH, scale, fitScale, toScale, gridW, gridH, originX, originY,
    clip, canvasOffset: off,
    cellW: cellSpanX * scale, cellH: cellSpanY * scale,
    // True when any part of the grid falls outside the box that can show it --
    // too big, or placed past an edge. The plan view clips and says so rather
    // than re-fitting, because re-fitting is the silent re-scaling this mode
    // exists to avoid.
    overflows: toScale && (
      originX < clip.x - 0.5 || originX + gridW > clip.x + clip.w + 0.5
      || originY - gridH < clip.y - 0.5 || originY > clip.y + clip.h + 0.5),
  };
}

// Which population the PLAN VIEW's dynamic colour limits come from.
//
// Linked, both panes read one set of limits over every bin of every cell, so a
// colour means the same dB in the grid and in the B-scan beside it. Unlinked,
// the grid scales within its own GATED cell values instead -- the only way to
// keep contrast when the gate is narrowed onto a quiet depth -- and the two
// colour bars stop agreeing, which both of them say on screen.
//
// Shared so the panel, the B-scan pane and the projector window cannot pick
// differently; three copies of this choice would drift, and the projected
// image disagreeing with the monitor is exactly the failure that would not be
// noticed until it was on the wall.
// `focused` forces the unlinked population whatever the toggle says. A focused
// cell value is a back-projected SUM over an aperture, not a bin of any
// profile, so it is systematically above the bin-domain limits the linked scale
// is built from -- linked, every cell would saturate at the top of the colormap
// the moment focus was switched on. The B-scan pane is NOT focused (focusing is
// a plan-view reduction, not a change to the records), so it keeps the bin
// scale and the two genuinely disagree; `effectiveLink` is what both displays
// are given so they say so.
export function planViewScales(scaleLink, gridScales, sharedScale, rowScales, focused) {
  const unlinked = !!gridScales && (focused || scaleLink === 'independent');
  return {
    unlinked,
    effectiveLink: unlinked ? 'independent' : 'linked',
    global: unlinked ? gridScales.global : sharedScale,
    rows: unlinked ? gridScales.rows : rowScales,
  };
}
