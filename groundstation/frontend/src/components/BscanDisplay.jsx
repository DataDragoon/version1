import { useRef, useEffect, useState } from 'react';
import { BG_STATUS, BG_STATUS_TEXT, bgFailed } from '@/lib/cscanGrid';
// Shared with the plan view beside it -- the two panes are scaled off one
// population of bins, so they must also be coloured by one map or a colour
// would stop meaning the same dB in both. Checked bit-identical to the local
// `jet` this replaces.
import { COLORMAPS } from '@/lib/imagingEffects';

const BG = '#000000';
const GRID_COLOR = '#1a1a1a';

// ── Orientation ─────────────────────────────────────────────────────────────
//
// 'horizontal' is the original image: depth runs left to right, scan position
// runs top to bottom.
//
// 'vertical' is that image rotated 90 degrees ANTICLOCKWISE, which is what the
// C-scan panel uses. Rotating anticlockwise sends the old left edge (bin 0) to
// the bottom and the old top edge (position 0) to the left, so depth now runs
// bottom-to-top and POSITION RUNS LEFT TO RIGHT -- the same axis, the same
// direction and (given `align`) the same pixels as the C-scan grid's horizontal
// axis. That is the point: the pane sits under the grid, one column per grid
// column, so a feature in the B-scan is directly below the cell it came from.
// It is also the cheap orientation for this rig, because a raster is usually
// far wider than it is tall, so the position axis wants the long screen axis
// and the pane only needs a short strip of height.
//
// Everything below is written against two slot functions -- one per axis --
// rather than duplicated per orientation, so the two images cannot drift apart.
function makeGeom(orientation, pad, plotW, plotH, numBins, totalRows, rowGridIx, hasBg, align) {
  const vertical = orientation === 'vertical';

  // Row slots. Aligned mode puts each position at the x of the C-scan cell it
  // was captured at, taken from the plan view's own layout; it needs every
  // drawn position to carry a grid index, so an imported linear scan (which has
  // none) falls back to spreading the rows evenly.
  const aligned = vertical && !!align && align.cellW > 0
    && rowGridIx.every((ix, i) => ix != null || (hasBg && i === 0));

  const rowSpanEven = (vertical ? plotW : plotH) / Math.max(1, totalRows);
  const rowAxisMin = vertical ? pad.left : pad.top;

  // The background reference is not a grid position, so in aligned mode it gets
  // its own slot just OUTSIDE the grid rather than being allowed to shift every
  // real column along by one. Left of the grid normally; right of it when the
  // grid is placed hard against the left edge, where clamping into the margin
  // would put it on top of the first data column instead of beside it.
  const bgSpan = aligned ? Math.min(align.cellW, 16) : rowSpanEven;
  let bgA = rowAxisMin;
  if (aligned) {
    const leftA = align.originX - bgSpan - 3;
    if (leftA >= 0) {
      bgA = leftA;
    } else {
      let maxIx = 0;
      for (const ix of rowGridIx) if (ix != null && ix > maxIx) maxIx = ix;
      bgA = align.originX + (maxIx + 1) * align.cellW + 3;
    }
  }

  const rowSlot = (rowIdx) => {
    if (!aligned) return { a: rowAxisMin + rowIdx * rowSpanEven, span: rowSpanEven };
    const gx = rowGridIx[rowIdx];
    if (gx == null) return { a: bgA, span: bgSpan };
    return { a: align.originX + gx * align.cellW, span: align.cellW };
  };

  const binSpan = (vertical ? plotH : plotW) / Math.max(1, numBins);
  // Bin 0 sits at the bottom when vertical (that is the anticlockwise rotation)
  // and at the left when horizontal.
  const binSlot = (binIdx) => ({
    a: vertical ? pad.top + plotH - (binIdx + 1) * binSpan : pad.left + binIdx * binSpan,
    span: binSpan,
  });

  const binAxisMin = vertical ? pad.top : pad.left;
  const binAxisMax = binAxisMin + (vertical ? plotH : plotW);

  return {
    vertical, aligned, binSpan, rowSpanEven, bgA, bgSpan,
    rowSlot, binSlot, binAxisMin, binAxisMax,
    // One (bin, row) cell.
    cell: (binIdx, rowIdx) => {
      const b = binSlot(binIdx);
      const r = rowSlot(rowIdx);
      return vertical
        ? { x: r.a, y: b.a, w: r.span, h: b.span }
        : { x: b.a, y: r.a, w: b.span, h: r.span };
    },
    // A whole row, across every bin.
    rowBand: (rowIdx) => {
      const r = rowSlot(rowIdx);
      return vertical
        ? { x: r.a, y: pad.top, w: r.span, h: plotH }
        : { x: pad.left, y: r.a, w: plotW, h: r.span };
    },
    // A span of the bin axis, across every row.
    binBand: (from, to) => (vertical
      ? { x: pad.left, y: from, w: plotW, h: to - from }
      : { x: from, y: pad.top, w: to - from, h: plotH }),
    // A line at one position on the bin axis, spanning every row.
    binLine: (a) => (vertical
      ? [pad.left, a, pad.left + plotW, a]
      : [a, pad.top, a, pad.top + plotH]),
    // A line at one position on the row axis, spanning every bin.
    rowLine: (a) => (vertical
      ? [a, pad.top, a, pad.top + plotH]
      : [pad.left, a, pad.left + plotW, a]),
    // Profile mode: a point at `norm` (0..1) of the way up this row's band. The
    // anticlockwise rotation sends "up" to "left", so the same expression gives
    // the value axis in both orientations.
    profilePoint: (binIdx, rowIdx, norm) => {
      const b = binSlot(binIdx);
      const r = rowSlot(rowIdx);
      const along = b.a + b.span / 2;
      const perp = r.a + r.span - norm * r.span;
      return vertical ? { x: perp, y: along } : { x: along, y: perp };
    },
  };
}

function drawBscan(canvas, scanData, params, crosshair, isLinear, displayMode, bgDisplay, scaleRange, sharedScale, subMode, showGate, scaleScope, scaleLink, orientation, align, colormap) {
  if (!canvas) return;
  const cmap = COLORMAPS[colormap] || COLORMAPS.jet;
  const ctx = canvas.getContext('2d');
  const rect = canvas.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  canvas.width = rect.width * dpr;
  canvas.height = rect.height * dpr;
  ctx.scale(dpr, dpr);
  const w = rect.width;
  const h = rect.height;

  ctx.fillStyle = BG;
  ctx.fillRect(0, 0, w, h);

  if (!scanData || scanData.length === 0) {
    ctx.fillStyle = '#333333';
    ctx.font = '11px monospace';
    ctx.textAlign = 'center';
    ctx.fillText('No scan data — capture positions to build B-scan', w / 2, h / 2);
    return;
  }

  const vertical = orientation === 'vertical';
  // Vertical shares the plan view's left/right padding, so the two plot boxes
  // (and their colour bars) line up on screen even when alignment is off.
  const pad = vertical
    ? { top: 22, bottom: 34, left: 52, right: 64 }
    : { top: 24, bottom: 36, left: 50, right: 20 };
  const plotW = w - pad.left - pad.right;
  const plotH = h - pad.top - pad.bottom;
  if (plotW < 20 || plotH < 20) return;

  const numPos = scanData.length;
  const allDistances = scanData[0].distances;
  const { hStep } = params;

  // Row label: a C-scan cell carries its own grid coordinates; fall back to the
  // capture index times the horizontal step for data imported from a linear scan.
  const rowLabelFor = (scanIdx) => {
    const pos = scanData[scanIdx];
    if (pos && pos.x_cm != null && pos.y_cm != null) return `${pos.x_cm.toFixed(0)},${pos.y_cm.toFixed(0)}`;
    return (scanIdx * (hStep || 1)).toFixed(0);
  };

  // The whole profile, always. There used to be a Max Depth field clipping this
  // to 70 cm against a displayed range of ~74 cm, which bought nothing and meant
  // the pane could silently hide the far end of the record. Depth SELECTION is
  // the Depth Slice gate's job, and that only governs the C-scan grid's colour;
  // this pane exists to show where things actually are before you gate. The
  // gate markers below shade the excluded bins but never drop them -- the data
  // outside the gate stays drawn, and stays readable, deliberately.
  const startBin = 0;
  const endBin = allDistances.length - 1;

  const numBins = endBin - startBin + 1;
  const distances = allDistances.slice(startBin, endBin + 1);
  const maxDist = distances[distances.length - 1];
  const minDist = distances[0];

  // Build combined row list: BG reference (if present) + scan positions
  const hasBg = bgDisplay && bgDisplay.magnitudes && bgDisplay.distances;
  const bgLabel = hasBg && bgDisplay.isModel ? 'BG MODEL' : 'BG REF';
  const totalRows = numPos + (hasBg ? 1 : 0);

  // Grid column of each drawn row; null for the background reference, which has
  // no position of its own.
  const rowGridIx = [];
  if (hasBg) rowGridIx.push(null);
  for (let i = 0; i < numPos; i++) rowGridIx.push(scanData[i].grid_ix != null ? scanData[i].grid_ix : null);
  const G = makeGeom(orientation, pad, plotW, plotH, numBins, totalRows, rowGridIx, hasBg, align);

  // Colour limits.
  //
  // Dynamic comes from the SHARED scale, computed once over EVERY cell of the
  // whole C-scan grid, not from this row. Two things were wrong before:
  //
  //   1. Limits were per-row, so the same colour meant a different dB in the
  //      grid and in this pane, and clicking to a different row silently
  //      re-scaled the image.
  //   2. The BG reference row was included. It is the UNSUBTRACTED background,
  //      20-30 dB above every subtracted residual row, so it set dbMax on its
  //      own and crushed the actual data into the bottom few percent of the
  //      colormap -- a working subtraction looked empty.
  //
  // The BG row is still DRAWN (it is the visual sanity check on the reference),
  // it just no longer votes on the limits. Invalid rows do not vote either.
  let dbMin = Infinity;
  let dbMax = -Infinity;
  if (sharedScale && isFinite(sharedScale.min) && isFinite(sharedScale.max)) {
    dbMin = sharedScale.min;
    dbMax = sharedScale.max;
  } else {
    for (let posIdx = 0; posIdx < numPos; posIdx++) {
      if (bgFailed(scanData[posIdx].bg_status)) continue;
      const mags = scanData[posIdx].magnitudes;
      for (let binIdx = 0; binIdx < numBins; binIdx++) {
        const db = mags[startBin + binIdx];
        if (db < dbMin) dbMin = db;
        if (db > dbMax) dbMax = db;
      }
    }
  }
  if (!isFinite(dbMin)) dbMin = -90;
  if (!isFinite(dbMax)) dbMax = -20;
  // Manual scaling pins both ends so cells stay comparable across captures.
  if (scaleRange && !scaleRange.dynamic) {
    dbMin = scaleRange.min;
    dbMax = scaleRange.max;
  }
  if (dbMax - dbMin < 1) { dbMin -= 0.5; dbMax += 0.5; }

  // A magnitude difference is a dB ratio centred on zero; the linear warp maps
  // an amplitude and is meaningless on it.
  const isDiff = subMode === 'magnitude';
  const useLinear = isLinear && !isDiff;

  const linMin = Math.pow(10, dbMin / 20);
  const linMax = Math.pow(10, dbMax / 20);

  const normOf = (db) => (useLinear
    ? (Math.pow(10, db / 20) - linMin) / (linMax - linMin)
    : (db - dbMin) / (dbMax - dbMin));

  // Helper: get magnitudes for a given row index (0 = BG if present, then scan data)
  const getMagsForRow = (rowIdx) => {
    if (hasBg && rowIdx === 0) return bgDisplay.magnitudes;
    const scanIdx = hasBg ? rowIdx - 1 : rowIdx;
    return scanData[scanIdx].magnitudes;
  };

  // Clip region for the plot area. In aligned mode the BG column deliberately
  // sits just outside the grid -- on either side, depending on where the grid
  // is placed -- so the clip is widened to whichever side it landed on.
  const bgSlot = hasBg ? G.rowSlot(0) : null;
  const clipX = bgSlot ? Math.min(pad.left, bgSlot.a) : pad.left;
  const clipR = bgSlot ? Math.max(pad.left + plotW, bgSlot.a + bgSlot.span) : pad.left + plotW;
  const clipW = clipR - clipX;
  ctx.save();
  ctx.beginPath();
  ctx.rect(clipX, pad.top, clipW, plotH);
  ctx.clip();

  if (displayMode === 'color') {
    for (let rowIdx = 0; rowIdx < totalRows; rowIdx++) {
      const mags = getMagsForRow(rowIdx);
      const rowPos = (hasBg && rowIdx === 0) ? null : scanData[hasBg ? rowIdx - 1 : rowIdx];
      // A row whose background could not be resolved is not drawn as data --
      // its profile is un-subtracted and would read as the strongest return in
      // the image.
      if (rowPos && bgFailed(rowPos.bg_status)) {
        const band = G.rowBand(rowIdx);
        ctx.fillStyle = '#2a0a10';
        ctx.fillRect(band.x, band.y, Math.ceil(band.w) + 1, Math.ceil(band.h) + 1);
        ctx.strokeStyle = '#ff4d6d';
        ctx.lineWidth = 1;
        ctx.beginPath();
        for (let d = -band.h; d < band.w; d += 10) {
          ctx.moveTo(band.x + d, band.y);
          ctx.lineTo(band.x + d + band.h, band.y + band.h);
        }
        ctx.stroke();
        continue;
      }
      for (let binIdx = 0; binIdx < numBins; binIdx++) {
        const db = (startBin + binIdx < mags.length) ? mags[startBin + binIdx] : dbMin;
        const [r, g, b] = cmap(normOf(db));
        ctx.fillStyle = `rgb(${r},${g},${b})`;
        const c = G.cell(binIdx, rowIdx);
        ctx.fillRect(c.x, c.y, Math.ceil(c.w) + 1, Math.ceil(c.h) + 1);
      }
    }

    // BG row indicator: an orange edge on the far side of the BG row.
    if (hasBg) {
      const s = G.rowSlot(0);
      const [x1, y1, x2, y2] = G.rowLine(s.a + s.span);
      ctx.strokeStyle = '#f59e0b';
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.moveTo(x1, y1);
      ctx.lineTo(x2, y2);
      ctx.stroke();
    }
  } else {
    // Range profile mode: draw each row as a line graph
    const lineColors = ['#6B9BD2', '#8BB8E8', '#D1855C', '#E8A87C', '#7EC8A0', '#C78BDB', '#E8D06B', '#E87B7B'];

    for (let rowIdx = 0; rowIdx < totalRows; rowIdx++) {
      const mags = getMagsForRow(rowIdx);
      const isBgRow = hasBg && rowIdx === 0;

      // BG row uses dashed orange; scan rows use solid colors
      if (isBgRow) {
        ctx.strokeStyle = '#f59e0b';
        ctx.lineWidth = 1.5;
        ctx.setLineDash([4, 3]);
      } else {
        const scanIdx = hasBg ? rowIdx - 1 : rowIdx;
        ctx.strokeStyle = lineColors[scanIdx % lineColors.length];
        ctx.lineWidth = 1;
        ctx.setLineDash([]);
      }
      ctx.globalAlpha = 0.9;
      ctx.beginPath();

      for (let binIdx = 0; binIdx < numBins; binIdx++) {
        const db = (startBin + binIdx < mags.length) ? mags[startBin + binIdx] : dbMin;
        const p = G.profilePoint(binIdx, rowIdx, normOf(db));
        if (binIdx === 0) ctx.moveTo(p.x, p.y);
        else ctx.lineTo(p.x, p.y);
      }
      ctx.stroke();
      ctx.setLineDash([]);
      ctx.globalAlpha = 1.0;

      // Subtle separator between rows
      if (rowIdx < totalRows - 1) {
        const s = G.rowSlot(rowIdx);
        const [x1, y1, x2, y2] = G.rowLine(s.a + s.span);
        ctx.strokeStyle = isBgRow ? '#f59e0b44' : '#ffffff08';
        ctx.lineWidth = 0.5;
        ctx.beginPath();
        ctx.moveTo(x1, y1);
        ctx.lineTo(x2, y2);
        ctx.stroke();
      }
    }
  }

  ctx.restore();

  // ── Depth gate markers ───────────────────────────────────────────────────
  //
  // The gate lives on the C-scan panel and ALWAYS governs the plan view: the
  // cell colour is gatedIntensity() over exactly these bins and nothing else.
  // These markers are a placement aid only -- turning them off stops drawing
  // them and changes nothing that is computed. That separation is deliberate;
  // a display toggle that silently altered a measurement is the class of bug
  // this panel has been bitten by before.
  //
  // The shaded region is the exact complement of the bins gatedIntensity()
  // keeps, found the same way it finds them, so what is left bright is what the
  // cell was actually built from -- not an approximation of it.
  if (showGate) {
    const gsM = (params.gateStart != null ? params.gateStart : 0) / 100;
    const geM = (params.gateEnd != null ? params.gateEnd : 0) / 100;
    let first = -1;
    let last = -1;
    for (let j = 0; j < numBins; j++) {
      const d = distances[j];
      if (d < gsM || d > geM) continue;
      if (first < 0) first = j;
      last = j;
    }
    const empty = first < 0;
    // The gate is a contiguous run of bins, but which END of the axis it starts
    // at flips with the orientation, so take the extremes rather than assuming.
    let gLo = G.binAxisMax;
    let gHi = G.binAxisMax;
    if (!empty) {
      const a = G.binSlot(first);
      const b = G.binSlot(last);
      gLo = Math.max(G.binAxisMin, Math.min(a.a, b.a));
      gHi = Math.min(G.binAxisMax, Math.max(a.a + a.span, b.a + b.span));
    }

    ctx.save();
    ctx.beginPath();
    ctx.rect(clipX, pad.top, clipW, plotH);
    ctx.clip();

    ctx.fillStyle = 'rgba(0,0,0,0.70)';
    if (gLo > G.binAxisMin) {
      const r = G.binBand(G.binAxisMin, gLo);
      ctx.fillRect(r.x, r.y, r.w, r.h);
    }
    if (gHi < G.binAxisMax) {
      const r = G.binBand(gHi, G.binAxisMax);
      ctx.fillRect(r.x, r.y, r.w, r.h);
    }

    if (!empty) {
      ctx.strokeStyle = '#22d3ee';
      ctx.lineWidth = 1.5;
      ctx.setLineDash([5, 3]);
      ctx.beginPath();
      for (const a of [gLo, gHi]) {
        const [x1, y1, x2, y2] = G.binLine(a);
        ctx.moveTo(x1, y1);
        ctx.lineTo(x2, y2);
      }
      ctx.stroke();
      ctx.setLineDash([]);
    }
    ctx.restore();

    ctx.fillStyle = '#22d3ee';
    ctx.font = 'bold 9px monospace';
    if (empty) {
      ctx.textAlign = 'center';
      ctx.fillText('GATE OUTSIDE THIS RECORD — no bins contribute',
        pad.left + plotW / 2, pad.top + plotH / 2);
    } else if (vertical) {
      // Bin 0 is at the bottom, so gHi is the NEAR end of the record.
      ctx.textAlign = 'left';
      ctx.fillText(`${params.gateEnd} cm`, pad.left + 3, Math.max(gLo + 9, pad.top + 9));
      ctx.fillText(`${params.gateStart} cm`, pad.left + 3, Math.min(gHi - 3, pad.top + plotH - 3));
      ctx.font = '8px monospace';
      ctx.fillStyle = '#22d3ee99';
      ctx.textAlign = 'right';
      ctx.fillText(`GATE · ${last - first + 1} bins`, pad.left + plotW - 4, (gLo + gHi) / 2);
    } else {
      ctx.textAlign = 'left';
      ctx.fillText(`${params.gateStart} cm`, Math.min(gLo + 3, pad.left + plotW - 44), pad.top + 10);
      ctx.textAlign = 'right';
      ctx.fillText(`${params.gateEnd} cm`, Math.max(gHi - 3, pad.left + 44), pad.top + 10);
      ctx.textAlign = 'center';
      ctx.font = '8px monospace';
      ctx.fillStyle = '#22d3ee99';
      ctx.fillText(`GATE · ${last - first + 1} bins`, (gLo + gHi) / 2, pad.top + plotH - 4);
    }
  }

  // Grid overlay
  ctx.strokeStyle = GRID_COLOR;
  ctx.lineWidth = 0.5;
  ctx.globalAlpha = 0.4;

  const depthTicks = 6;
  for (let i = 0; i <= depthTicks; i++) {
    const a = G.binAxisMin + (i / depthTicks) * (G.binAxisMax - G.binAxisMin);
    const [x1, y1, x2, y2] = G.binLine(a);
    ctx.beginPath();
    ctx.moveTo(x1, y1);
    ctx.lineTo(x2, y2);
    ctx.stroke();
  }

  const rowAxisMin = vertical ? pad.left : pad.top;
  const rowAxisMax = rowAxisMin + (vertical ? plotW : plotH);
  const posTicks = Math.min(numPos, 10);
  for (let i = 0; i <= posTicks; i++) {
    const a = rowAxisMin + (i / posTicks) * (rowAxisMax - rowAxisMin);
    const [x1, y1, x2, y2] = G.rowLine(a);
    ctx.beginPath();
    ctx.moveTo(x1, y1);
    ctx.lineTo(x2, y2);
    ctx.stroke();
  }
  ctx.globalAlpha = 1.0;

  // Depth labels along the bin axis. Vertical runs bottom-to-top, so the tick
  // at fraction i of the axis is at fraction (1 - i) of the record.
  ctx.fillStyle = '#555555';
  ctx.font = '9px monospace';
  // In aligned mode the BG column sits in the left margin; keep labels clear of it.
  const depthLabelRight = clipX - 6;
  for (let i = 0; i <= depthTicks; i++) {
    const f = i / depthTicks;
    const a = G.binAxisMin + f * (G.binAxisMax - G.binAxisMin);
    const dist = minDist + (vertical ? 1 - f : f) * (maxDist - minDist);
    if (vertical) {
      ctx.textAlign = 'right';
      ctx.fillText(dist.toFixed(2), depthLabelRight, a + 3);
    } else {
      ctx.textAlign = 'center';
      ctx.fillText(dist.toFixed(2), a, h - pad.bottom + 14);
    }
  }

  // Position labels, one per row, thinned so they never collide.
  ctx.fillStyle = '#555555';
  ctx.font = '9px monospace';
  const labelPitch = Math.max(0.01, G.rowSlot(Math.min(hasBg ? 1 : 0, totalRows - 1)).span);
  const rowEvery = Math.max(1, Math.ceil((vertical ? 42 : 12) / labelPitch));
  for (let rowIdx = 0; rowIdx < totalRows; rowIdx++) {
    if (hasBg && rowIdx === 0) continue;
    const scanIdx = hasBg ? rowIdx - 1 : rowIdx;
    if (scanIdx % rowEvery !== 0) continue;
    const s = G.rowSlot(rowIdx);
    if (vertical) {
      const cx = s.a + s.span / 2;
      if (cx < pad.left - 2 || cx > pad.left + plotW + 2) continue;
      ctx.textAlign = 'center';
      ctx.fillText(rowLabelFor(scanIdx), cx, h - pad.bottom + 13);
    } else {
      ctx.textAlign = 'right';
      ctx.fillText(rowLabelFor(scanIdx), pad.left - 6, s.a + s.span / 2 + 3);
    }
  }

  // BG label, placed on the BG row's own slot.
  if (hasBg) {
    const s = G.rowSlot(0);
    ctx.fillStyle = '#f59e0b';
    ctx.font = 'bold 8px monospace';
    if (vertical) {
      ctx.save();
      ctx.translate(s.a + s.span / 2 + 3, pad.top + plotH / 2);
      ctx.rotate(-Math.PI / 2);
      ctx.textAlign = 'center';
      ctx.fillText(bgLabel, 0, 0);
      ctx.restore();
    } else {
      ctx.textAlign = 'right';
      ctx.fillText(bgLabel, w - pad.right - 4, s.a + s.span / 2 + 3);
    }
  }

  // Axis titles
  ctx.fillStyle = '#444444';
  ctx.font = '9px monospace';
  if (vertical) {
    ctx.textAlign = 'center';
    ctx.fillText('Position x,y (cm)', pad.left + plotW / 2, h - pad.bottom + 27);
    ctx.save();
    ctx.translate(12, pad.top + plotH / 2);
    ctx.rotate(-Math.PI / 2);
    ctx.textAlign = 'center';
    ctx.fillText('Depth (m) →', 0, 0);
    ctx.restore();
  } else {
    ctx.textAlign = 'center';
    ctx.fillText('Depth (m)', pad.left + plotW / 2, h - pad.bottom + 28);
    ctx.save();
    ctx.translate(12, pad.top + plotH / 2);
    ctx.rotate(-Math.PI / 2);
    ctx.textAlign = 'center';
    ctx.fillText('Position x,y (cm)', 0, 0);
    ctx.restore();
  }

  // Title
  const scaleLabel = isDiff ? 'Δ MAG dB' : useLinear ? 'LINEAR' : 'dB';
  const modeLabel = displayMode === 'color' ? 'COLOR' : 'PROFILE';
  // The caller hands this pane whichever limits are in force; say which they
  // are, so a colour that does not match the plan view's is explained.
  const dyn = !(scaleRange && !scaleRange.dynamic);
  const perRow = scaleScope === 'row' && dyn;
  // Unlinked, the plan view is scaled to its own gated values and this pane is
  // not -- so the two colour bars mean different things and both must say so.
  const unlinked = scaleLink === 'independent' && dyn;
  const tag = [
    perRow ? 'PER ROW' : null,
    unlinked ? 'UNLINKED' : null,
    G.aligned ? 'ALIGNED' : null,
  ].filter(Boolean).join(' · ');
  ctx.fillStyle = '#6B9BD2';
  ctx.font = 'bold 10px monospace';
  ctx.textAlign = 'left';
  ctx.fillText(`B-SCAN (${scaleLabel} / ${modeLabel})${tag ? ` · ${tag}` : ''}`, pad.left, 14);

  ctx.fillStyle = '#444444';
  ctx.font = '9px monospace';
  ctx.textAlign = 'right';
  ctx.fillText(`${numPos} pos × ${numBins} bins`, w - pad.right, 14);

  // Color bar (only in color mode)
  if (displayMode === 'color') {
    const barW = 12;
    const barH = plotH;
    // Vertical uses the plan view's colour-bar offset, so with the panes stacked
    // the two bars sit at the same x on screen.
    const barX = vertical ? w - pad.right + 16 : w - pad.right + 6;
    const barY = pad.top;
    for (let i = 0; i < barH; i++) {
      const t = 1 - i / barH;
      const [r, g, b] = cmap(t);
      ctx.fillStyle = `rgb(${r},${g},${b})`;
      ctx.fillRect(barX, barY + i, barW, 1);
    }
    ctx.fillStyle = '#555555';
    ctx.font = '8px monospace';
    ctx.textAlign = 'left';
    if (useLinear) {
      ctx.fillText(linMax.toExponential(1), barX, barY - 4);
      ctx.fillText(linMin.toExponential(1), barX, barY + barH + 10);
    } else if (isDiff) {
      ctx.fillText(`${dbMax >= 0 ? '+' : ''}${dbMax.toFixed(1)} dB`, barX, barY - 4);
      ctx.fillText(`${dbMin >= 0 ? '+' : ''}${dbMin.toFixed(1)} dB`, barX, barY + barH + 10);
    } else {
      ctx.fillText(`${dbMax.toFixed(0)} dB`, barX, barY - 4);
      ctx.fillText(`${dbMin.toFixed(0)} dB`, barX, barY + barH + 10);
    }
  }

  // Crosshair
  if (crosshair) {
    // Invert the two slot mappings. Aligned columns are not evenly spaced (and
    // a partly-captured row has gaps), so the row is found by scanning the
    // slots rather than by dividing.
    const inBox = crosshair.x >= clipX && crosshair.x <= clipR
      && crosshair.y >= pad.top && crosshair.y <= pad.top + plotH;
    let binIdx = -1;
    let rowIdx = -1;
    if (inBox) {
      const binF = vertical
        ? 1 - (crosshair.y - pad.top) / plotH
        : (crosshair.x - pad.left) / plotW;
      binIdx = Math.max(0, Math.min(numBins - 1, Math.floor(binF * numBins)));
      const rowC = vertical ? crosshair.x : crosshair.y;
      for (let r = 0; r < totalRows; r++) {
        const s = G.rowSlot(r);
        if (rowC >= s.a && rowC < s.a + s.span) { rowIdx = r; break; }
      }
    }
    if (binIdx >= 0 && rowIdx >= 0) {
      const isBgRow = hasBg && rowIdx === 0;
      const mags = getMagsForRow(rowIdx);
      const dist = distances[binIdx];
      const db = (startBin + binIdx < mags.length) ? mags[startBin + binIdx] : dbMin;

      ctx.setLineDash([3, 3]);
      ctx.strokeStyle = '#ffffff44';
      ctx.lineWidth = 0.5;
      ctx.beginPath();
      ctx.moveTo(crosshair.x, pad.top);
      ctx.lineTo(crosshair.x, h - pad.bottom);
      ctx.stroke();
      ctx.beginPath();
      ctx.moveTo(clipX, crosshair.y);
      ctx.lineTo(Math.max(clipR, w - pad.right), crosshair.y);
      ctx.stroke();
      ctx.setLineDash([]);

      ctx.fillStyle = '#ffffff';
      ctx.font = '10px monospace';
      ctx.textAlign = 'left';
      const valLabel = useLinear
        ? Math.pow(10, db / 20).toExponential(2)
        : isDiff ? `${db >= 0 ? '+' : ''}${db.toFixed(2)}dB`
        : `${db.toFixed(1)}dB`;
      const posLabel = isBgRow ? bgLabel : `pos ${rowLabelFor(hasBg ? rowIdx - 1 : rowIdx)}cm`;
      const rowPos = isBgRow ? null : scanData[hasBg ? rowIdx - 1 : rowIdx];
      const rowFlag = rowPos && rowPos.bg_status && rowPos.bg_status !== BG_STATUS.OK
        && rowPos.bg_status !== BG_STATUS.OFF
        ? ` | ${BG_STATUS_TEXT[rowPos.bg_status] || rowPos.bg_status}` : '';
      const label = `${posLabel} | ${dist.toFixed(2)}m | ${valLabel}${rowFlag}`;
      const labelX = crosshair.x + 10 > w - 220 ? Math.max(4, crosshair.x - 220) : crosshair.x + 10;
      ctx.fillText(label, labelX, Math.max(crosshair.y - 8, pad.top + 10));
    }
  }
}

export default function BscanDisplay({ scanData, bgDisplay, params, capturing, sfcwProgress, scaleMode, displayMode, scaleRange, sharedScale, subMode, showGate, scaleScope, scaleLink, orientation, alignRef, colormap }) {
  const canvasRef = useRef(null);
  const animRef = useRef(null);
  const [crosshair, setCrosshair] = useState(null);

  const isLinear = scaleMode === 'linear';
  const mode = displayMode || 'color';
  const orient = orientation || 'horizontal';

  // Kept as a rAF loop rather than a one-shot draw: drawBscan sizes itself from
  // getBoundingClientRect(), so redrawing every frame is what makes the canvas
  // follow a panel resize. Nothing here is animated any more -- there used to be
  // a per-row lerp toward lidar-derived range-bin shifts, but the shifts were
  // hard-wired to zero at the call site and the producer was dead code, so the
  // whole path was removed (2026-08-30).
  //
  // The loop is also what makes `alignRef` work: the plan view writes its layout
  // into that ref every frame and this canvas reads the current value on its
  // own frame, so the two stay registered without a re-render at 60 Hz.
  useEffect(() => {
    const render = () => {
      drawBscan(canvasRef.current, scanData, params, crosshair, isLinear, mode, bgDisplay, scaleRange, sharedScale, subMode, showGate, scaleScope, scaleLink, orient, alignRef ? alignRef.current : null, colormap);
      animRef.current = requestAnimationFrame(render);
    };
    animRef.current = requestAnimationFrame(render);
    return () => { if (animRef.current) cancelAnimationFrame(animRef.current); };
  }, [scanData, params, crosshair, isLinear, mode, bgDisplay, scaleRange, sharedScale, subMode, showGate, scaleScope, scaleLink, orient, alignRef, colormap]);

  return (
    <div className="flex flex-col w-full h-full">
      {capturing && sfcwProgress && (
        <div className="absolute top-0 left-0 right-0 z-10 h-0.5">
          <div
            className="h-full bg-gradient-to-r from-[#6B9BD2] to-[#8BB8E8] transition-all duration-200"
            style={{ width: `${(sfcwProgress.step / sfcwProgress.total) * 100}%` }}
          />
        </div>
      )}

      <div className="relative flex-1 min-h-0">
        <canvas
          ref={canvasRef}
          className="absolute inset-0 w-full h-full"
          onMouseMove={(e) => {
            const rect = e.currentTarget.getBoundingClientRect();
            setCrosshair({ x: e.clientX - rect.left, y: e.clientY - rect.top });
          }}
          onMouseLeave={() => setCrosshair(null)}
        />
      </div>
    </div>
  );
}
