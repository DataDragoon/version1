import { useRef, useEffect, useState } from 'react';
import { orderedCellForIndex, buildCscanGrid, cscanLayout, BG_STATUS, BG_STATUS_TEXT } from '@/lib/cscanGrid';

const BG = '#000000';
const EMPTY_FILL = '#0d0d0d';
const EMPTY_STROKE = '#1f1f1f';
const GATED_OUT_FILL = '#3a3a3a';
// A cell whose background could not be resolved. Deliberately a colour no
// colormap produces, so it can never be read as a value: an un-subtracted cell
// sits 20-30 dB above its subtracted neighbours and would otherwise look like
// the strongest target in the scan.
const INVALID_FILL = '#2a0a10';
const INVALID_STROKE = '#ff4d6d';

function jet(t) {
  t = Math.max(0, Math.min(1, t));
  return [
    Math.round(255 * Math.min(1, Math.max(0, 1.5 - Math.abs(4 * t - 3)))),
    Math.round(255 * Math.min(1, Math.max(0, 1.5 - Math.abs(4 * t - 2)))),
    Math.round(255 * Math.min(1, Math.max(0, 1.5 - Math.abs(4 * t - 1)))),
  ];
}

function cellRect(ix, iy, L) {
  return {
    x: L.originX + ix * L.cellW,
    y: L.originY - (iy + 1) * L.cellH,
    w: L.cellW,
    h: L.cellH,
  };
}

function cellAt(px, py, L, hCount, vCount) {
  const ix = Math.floor((px - L.originX) / L.cellW);
  const iy = Math.floor((L.originY - py) / L.cellH);
  if (ix < 0 || ix >= hCount || iy < 0 || iy >= vCount) return null;
  return { ix, iy };
}

function snakeOrderOf(cell, hCount) {
  return cell.iy * hCount + (cell.iy % 2 === 0 ? cell.ix : hCount - 1 - cell.ix) + 1;
}

// Where this canvas sits inside the viewport (the whole area right of the
// sidebar). To-scale placement is measured from the viewport, so that the grid
// holds still on the wall when the Live Sweep pane appears or a row opens and
// the canvas itself moves.
function canvasOffsetIn(rootRef, rect) {
  const root = rootRef && rootRef.current;
  if (!root) return { x: 0, y: 0 };
  const r = root.getBoundingClientRect();
  return { x: rect.left - r.left, y: rect.top - r.top };
}

function drawCscan(canvas, scanData, params, crosshair, selected, nextIndex, isLinear, scaleRange, pulse, scanMode, sharedScale, subMode, rowScales, scaleScope, scaleLink, projection, onLayout, rootRef, chromeless) {
  // `isConnected` is false for a frame or two while the projector window is
  // being torn down, and drawing into a canvas whose document is going away
  // throws in some browsers.
  if (!canvas || !canvas.isConnected) return;
  const ctx = canvas.getContext('2d');
  const rect = canvas.getBoundingClientRect();
  // From the canvas's OWN window: the projector portal draws into a canvas in
  // a second window, which can be on a display with a different pixel ratio.
  const dpr = (canvas.ownerDocument.defaultView || window).devicePixelRatio || 1;
  canvas.width = rect.width * dpr;
  canvas.height = rect.height * dpr;
  ctx.scale(dpr, dpr);
  const w = rect.width;
  const h = rect.height;

  ctx.fillStyle = BG;
  ctx.fillRect(0, 0, w, h);
  if (w < 80 || h < 80) return;

  const L = cscanLayout(w, h, params, projection, canvasOffsetIn(rootRef, rect));
  // Published so the B-scan pane below can put each position under the grid
  // cell it was captured at. A ref, not state: both canvases redraw every
  // frame anyway, and re-rendering the tree at 60 Hz to share four numbers
  // would be the expensive way to do it.
  if (onLayout) onLayout(L);
  const { hStep, vStep, gateStart, gateEnd, metric } = params;
  const grid = buildCscanGrid(scanData, params);
  const total = grid.hCount * grid.vCount;

  // Colour limits. Dynamic comes from the SHARED scale computed over the whole
  // grid (every bin of every valid cell, percentile-clipped), not from this
  // grid's own gated min/max -- so the colour bar here and the one on the
  // B-scan pane mean the same dB, and a full min/max stretch of a flat residual
  // field no longer manufactures rainbow structure out of noise.
  //
  // The caller decides WHICH population these are drawn from -- every bin of
  // every cell (linked, so a colour means the same dB here and on the B-scan),
  // or this grid's own gated cell values (unlinked, which follows the gate and
  // is the only way to keep contrast when the gate is narrowed onto a quiet
  // depth). Either way the arithmetic below is the same; only the label changes.
  //
  // scaleScope === 'row' narrows that population to each cell's OWN grid row.
  // A colour then means a different dB in different rows -- which is the whole
  // point on a wall whose standoff varies row to row, where the loudest row
  // would otherwise set the limits and crush every other one -- so the colour
  // bar says so and shows the SELECTED row's limits rather than pretending to
  // describe the image. Manual scaling still overrides both.
  const manual = !!(scaleRange && !scaleRange.dynamic);
  const spread = (lo, hi) => (hi - lo < 1 ? { min: lo - 0.5, max: hi + 0.5 } : { min: lo, max: hi });

  let globalLim;
  if (manual) {
    globalLim = spread(scaleRange.min, scaleRange.max);
  } else if (sharedScale && isFinite(sharedScale.min) && isFinite(sharedScale.max)) {
    globalLim = spread(sharedScale.min, sharedScale.max);
  } else if (isFinite(grid.min) && isFinite(grid.max)) {
    globalLim = spread(grid.min, grid.max);
  } else {
    globalLim = { min: -90, max: -20 };
  }

  const perRow = !manual && scaleScope === 'row' && rowScales && rowScales.size > 0;
  const rowLimCache = new Map();
  const limitsFor = (iy) => {
    if (!perRow) return globalLim;
    if (rowLimCache.has(iy)) return rowLimCache.get(iy);
    const s = rowScales.get(iy);
    const lim = (s && isFinite(s.min) && isFinite(s.max)) ? spread(s.min, s.max) : globalLim;
    rowLimCache.set(iy, lim);
    return lim;
  };

  // A magnitude difference is a dB RATIO centred on zero, not an absolute
  // level, so the linear warp (which maps 10^(db/20), an amplitude) is
  // meaningless on it and is bypassed.
  const isDiff = subMode === 'magnitude';
  const useLinear = isLinear && !isDiff;

  const norm = (db, lim) => {
    if (useLinear) {
      const lo = Math.pow(10, lim.min / 20);
      const hi = Math.pow(10, lim.max / 20);
      return (Math.pow(10, db / 20) - lo) / (hi - lo);
    }
    return (db - lim.min) / (lim.max - lim.min);
  };

  // Everything positional is clipped to the plot box: at a to-scale setting
  // the grid can be far larger than the pane, and painting outside would run
  // over the axes and the colour bar.
  ctx.save();
  ctx.beginPath();
  ctx.rect(L.clip.x, L.clip.y, L.clip.w, L.clip.h);
  ctx.clip();

  // Cells
  for (let iy = 0; iy < grid.vCount; iy++) {
    for (let ix = 0; ix < grid.hCount; ix++) {
      const cell = grid.cells[iy * grid.hCount + ix];
      const r = cellRect(ix, iy, L);
      if (cell && cell.invalid) {
        // Captured, but no background could be produced for it. Drawn as an
        // explicit error rather than given a colour it has not earned.
        ctx.fillStyle = INVALID_FILL;
        ctx.fillRect(r.x, r.y, Math.ceil(r.w) + 0.5, Math.ceil(r.h) + 0.5);
        ctx.strokeStyle = INVALID_STROKE;
        ctx.lineWidth = 1;
        const inset = Math.min(r.w, r.h) * 0.28;
        ctx.beginPath();
        ctx.moveTo(r.x + inset, r.y + inset);
        ctx.lineTo(r.x + r.w - inset, r.y + r.h - inset);
        ctx.moveTo(r.x + r.w - inset, r.y + inset);
        ctx.lineTo(r.x + inset, r.y + r.h - inset);
        ctx.stroke();
      } else if (cell && isFinite(cell.value)) {
        const [cr, cg, cb] = jet(norm(cell.value, limitsFor(iy)));
        ctx.fillStyle = `rgb(${cr},${cg},${cb})`;
        ctx.fillRect(r.x, r.y, Math.ceil(r.w) + 0.5, Math.ceil(r.h) + 0.5);
        // The model was applied but clamped to the edge of its captured span.
        // Measured cost: 19 dB at 5 mm outside, NEGATIVE suppression past 10 mm.
        // The value is real enough to draw, but not to trust unmarked.
        if (cell.status === BG_STATUS.CLAMPED) {
          ctx.fillStyle = '#f59e0b';
          ctx.beginPath();
          ctx.moveTo(r.x + r.w, r.y);
          ctx.lineTo(r.x + r.w, r.y + Math.min(7, r.h));
          ctx.lineTo(r.x + r.w - Math.min(7, r.w), r.y);
          ctx.closePath();
          ctx.fill();
        }
      } else if (cell) {
        // Captured, but the depth gate falls outside its range profile.
        ctx.fillStyle = GATED_OUT_FILL;
        ctx.fillRect(r.x, r.y, Math.ceil(r.w) + 0.5, Math.ceil(r.h) + 0.5);
      } else {
        ctx.fillStyle = EMPTY_FILL;
        ctx.fillRect(r.x, r.y, r.w, r.h);
        ctx.strokeStyle = EMPTY_STROKE;
        ctx.lineWidth = 0.5;
        ctx.strokeRect(r.x + 0.25, r.y + 0.25, r.w - 0.5, r.h - 0.5);
      }
    }
  }

  // Chromeless is the projector's image: cells, the frame that bounds them and
  // the next-cell marker, and nothing else. Axes, titles, the colour bar, the
  // capture path and the hover readout are all instruments for reading the plan
  // view on a monitor -- projected onto the wall they would be light falling on
  // brick beside the measurement, and the labels would be in the wrong place
  // anyway once the grid is positioned by hand.
  //
  // The order the captured cells were actually visited in, taken from each
  // cell's own capture index rather than re-derived from `scanMode`.
  //
  // Re-deriving it was wrong for any record whose capture mode is not the one
  // currently selected -- which is the normal case for imported data, because
  // import deliberately does not restore `scanMode` (it is a live control, not
  // data). A rover grid reviewed in manual mode had its path drawn backwards
  // and its START marker on the bottom-left corner when the raster had really
  // begun at the top-left. `scanMode` still drives the NEXT-cell marker, which
  // is a statement about a capture that has not happened yet.
  const captureOrder = grid.cells
    .map((cell, idx) => (cell ? { idx, order: cell.order } : null))
    .filter(Boolean)
    .sort((a, b) => a.order - b.order);
  const cellOf = (idx) => ({ ix: idx % grid.hCount, iy: Math.floor(idx / grid.hCount) });

  // Path through the cells already captured — makes the raster order, and any
  // hole left by an undo, visible at a glance.
  if (!chromeless && grid.filled > 1 && L.cellW > 6 && L.cellH > 6) {
    ctx.strokeStyle = '#ffffff33';
    ctx.lineWidth = 1;
    ctx.setLineDash([3, 3]);
    ctx.beginPath();
    captureOrder.forEach((e, n) => {
      const { ix, iy } = cellOf(e.idx);
      const r = cellRect(ix, iy, L);
      const cx = r.x + r.w / 2;
      const cy = r.y + r.h / 2;
      if (n === 0) ctx.moveTo(cx, cy); else ctx.lineTo(cx, cy);
    });
    ctx.stroke();
    ctx.setLineDash([]);
  }

  // Start marker on the cell the raster actually began at. With nothing
  // captured yet there is no fact to read, so it falls back to where the
  // SELECTED mode would start: top-left under the rover (which snakes
  // downwards), bottom-left by hand.
  if (!chromeless) {
    const startCell = captureOrder.length > 0
      ? cellOf(captureOrder[0].idx)
      : { ix: 0, iy: scanMode === 'rover' ? grid.vCount - 1 : 0 };
    const r = cellRect(startCell.ix, startCell.iy, L);
    ctx.strokeStyle = '#4aff8a88';
    ctx.lineWidth = 1;
    ctx.strokeRect(r.x + 1, r.y + 1, r.w - 2, r.h - 2);
    if (L.cellW > 34 && L.cellH > 14) {
      ctx.fillStyle = '#4aff8a99';
      ctx.font = 'bold 8px monospace';
      ctx.textAlign = 'center';
      ctx.fillText('START', r.x + r.w / 2, r.y + r.h / 2 + 3);
    }
  }

  // Next cell to capture
  if (nextIndex != null && nextIndex < total) {
    const { ix, iy } = orderedCellForIndex(nextIndex, grid.hCount, grid.vCount, scanMode);
    if (iy < grid.vCount && iy >= 0) {
      const r = cellRect(ix, iy, L);
      ctx.strokeStyle = '#22d3ee';
      ctx.lineWidth = 2;
      ctx.globalAlpha = 0.55 + 0.45 * pulse;
      ctx.setLineDash([5, 3]);
      ctx.strokeRect(r.x + 1, r.y + 1, r.w - 2, r.h - 2);
      ctx.setLineDash([]);
      ctx.globalAlpha = 1;
    }
  }

  // Selected cell — this is the row/trace the B-scan pane is showing
  if (!chromeless && selected && selected.ix < grid.hCount && selected.iy < grid.vCount) {
    const r = cellRect(selected.ix, selected.iy, L);
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 1.5;
    ctx.strokeRect(r.x + 0.75, r.y + 0.75, r.w - 1.5, r.h - 1.5);
  }

  // Grid frame
  ctx.strokeStyle = '#2a2a2a';
  ctx.lineWidth = 1;
  ctx.strokeRect(L.originX, L.originY - L.gridH, L.gridW, L.gridH);

  ctx.restore();

  // Everything past here is chrome for reading the image on a monitor.
  if (chromeless) return;

  // Axis ticks at cell centres, thinned so labels never collide
  ctx.font = '9px monospace';
  ctx.fillStyle = '#555555';
  ctx.textAlign = 'center';
  const xEvery = Math.max(1, Math.ceil(grid.hCount / Math.max(1, Math.floor(L.gridW / 34))));
  for (let ix = 0; ix < grid.hCount; ix += xEvery) {
    const r = cellRect(ix, 0, L);
    const cx = r.x + r.w / 2;
    if (cx < L.clip.x || cx > L.clip.x + L.clip.w) continue;
    ctx.fillText((ix * hStep).toFixed(hStep % 1 === 0 ? 0 : 1), cx, L.originY + 14);
  }
  const yEvery = Math.max(1, Math.ceil(grid.vCount / Math.max(1, Math.floor(L.gridH / 16))));
  ctx.textAlign = 'right';
  for (let iy = 0; iy < grid.vCount; iy += yEvery) {
    const r = cellRect(0, iy, L);
    const cy = r.y + r.h / 2;
    if (cy < L.clip.y || cy > L.clip.y + L.clip.h) continue;
    ctx.fillText((iy * vStep).toFixed(vStep % 1 === 0 ? 0 : 1), Math.max(6, Math.min(L.originX, L.pad.left)) - 6, cy + 3);
  }

  // Axis titles
  ctx.fillStyle = '#444444';
  ctx.textAlign = 'center';
  ctx.fillText('Horizontal (cm)', L.pad.left + L.plotW / 2, h - L.pad.bottom + 28);
  ctx.save();
  ctx.translate(13, L.pad.top + L.plotH / 2);
  ctx.rotate(-Math.PI / 2);
  ctx.textAlign = 'center';
  ctx.fillText('Vertical (cm)', 0, 0);
  ctx.restore();

  // Title
  ctx.fillStyle = '#22d3ee';
  ctx.font = 'bold 10px monospace';
  ctx.textAlign = 'left';
  // Focusing is named in the title because it changes what the colours ARE --
  // a back-projected aperture sum rather than this cell's own gated profile --
  // and the B-scan pane beside it is deliberately NOT focused.
  const focusTag = params.focusEnabled ? ` · FOCUS ×${params.focusAperture}` : '';
  ctx.fillText(
    `C-SCAN (${String(metric).toUpperCase()} @ ${gateStart}-${gateEnd} cm${isDiff ? ' · Δ MAG' : ''}${focusTag})`,
    L.pad.left, 14);
  ctx.fillStyle = '#444444';
  ctx.font = '9px monospace';
  ctx.textAlign = 'right';
  ctx.fillText(`${grid.filled} / ${total} cells`, w - L.pad.right, 14);

  // A to-scale grid is a measurement claim about the projected image, so it is
  // labelled with the constant it is drawn at, and flagged when the grid is
  // bigger than the pane can show (the field above is clipped, not shrunk --
  // shrinking it would be exactly the silent re-scaling this mode avoids).
  if (L.toScale) {
    ctx.font = 'bold 9px monospace';
    ctx.textAlign = 'center';
    ctx.fillStyle = L.overflows ? '#f59e0b' : '#4aff8a';
    ctx.fillText(
      `TO SCALE · ${L.scale.toFixed(2)} px/cm @ ${(L.originX + L.canvasOffset.x).toFixed(0)},`
      + `${(L.originY - L.gridH + L.canvasOffset.y).toFixed(0)} px${L.overflows ? ' · CLIPPED' : ''}`,
      L.pad.left + L.plotW / 2, 14);
  }

  // Colour bar. Under per-row scaling there is no single range that describes
  // the image, so it shows the SELECTED row's -- the one the B-scan pane beside
  // it is drawn with -- and is labelled PER ROW so it is not read as global.
  const unlinked = !manual && scaleLink === 'independent';
  const barLim = perRow ? limitsFor(selected ? selected.iy : 0) : globalLim;
  const dbMin = barLim.min;
  const dbMax = barLim.max;
  const barW = 12;
  const barH = L.plotH;
  const barX = w - L.pad.right + 16;
  const barY = L.pad.top;
  for (let i = 0; i < barH; i++) {
    const [r, g, b] = jet(1 - i / barH);
    ctx.fillStyle = `rgb(${r},${g},${b})`;
    ctx.fillRect(barX, barY + i, barW, 1);
  }
  const flagged = manual || unlinked;
  ctx.strokeStyle = flagged ? '#f59e0b' : perRow ? '#22d3ee' : '#2a2a2a';
  ctx.lineWidth = 1;
  ctx.strokeRect(barX - 0.5, barY - 0.5, barW + 1, barH + 1);
  ctx.fillStyle = flagged ? '#f59e0b' : perRow ? '#22d3ee' : '#555555';
  ctx.font = '8px monospace';
  ctx.textAlign = 'left';
  const barFmt = (v) => (isDiff ? `${v >= 0 ? '+' : ''}${v.toFixed(1)} dB` : `${v.toFixed(0)} dB`);
  ctx.fillText(barFmt(dbMax), barX - 2, barY - 4);
  ctx.fillText(barFmt(dbMin), barX - 2, barY + barH + 10);

  // In difference mode zero is the decision line: below it the cell got quieter
  // than the reference, above it something was added.
  if (isDiff && dbMin < 0 && dbMax > 0) {
    const zy = barY + barH * (1 - (0 - dbMin) / (dbMax - dbMin));
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(barX, zy);
    ctx.lineTo(barX + barW, zy);
    ctx.stroke();
  }
  // Anything but the plain shared scale is named, because every one of these
  // means a colour here is not a colour on the B-scan beside it.
  const tag = manual ? 'MANUAL' : [
    unlinked ? (params.focusEnabled ? 'OWN SCALE · FOCUSED' : 'OWN SCALE · GATED') : null,
    perRow ? `PER ROW ${(selected ? selected.iy : 0) + 1}` : null,
  ].filter(Boolean).join(' · ');
  if (tag) {
    ctx.save();
    ctx.translate(barX + barW + 11, barY + barH / 2);
    ctx.rotate(Math.PI / 2);
    ctx.textAlign = 'center';
    ctx.fillText(tag, 0, 0);
    ctx.restore();
  }

  // Hover readout
  if (crosshair) {
    const hit = cellAt(crosshair.x, crosshair.y, L, grid.hCount, grid.vCount);
    if (hit) {
      const r = cellRect(hit.ix, hit.iy, L);
      ctx.strokeStyle = '#ffffff88';
      ctx.lineWidth = 1;
      ctx.strokeRect(r.x + 0.5, r.y + 0.5, r.w - 1, r.h - 1);

      const cell = grid.cells[hit.iy * grid.hCount + hit.ix];
      const coord = `(${(hit.ix * hStep).toFixed(1)}, ${(hit.iy * vStep).toFixed(1)}) cm`;
      const order = `#${snakeOrderOf(hit, grid.hCount)}`;
      const val = !cell ? 'not captured'
        : cell.invalid ? (BG_STATUS_TEXT[cell.status] || 'INVALID')
        : !isFinite(cell.value) ? 'outside gate'
        : isDiff ? `${cell.value >= 0 ? '+' : ''}${cell.value.toFixed(2)} dB`
        : (useLinear ? Math.pow(10, cell.value / 20).toExponential(2) : `${cell.value.toFixed(1)} dB`);
      const standoff = cell && cell.pos.lidar_standoff_mm != null
        ? ` | ${cell.pos.lidar_standoff_mm.toFixed(0)} mm` : '';
      const flag = cell && cell.status === BG_STATUS.CLAMPED ? ' | BG CLAMPED' : '';
      const label = `${order} ${coord} | ${val}${standoff}${flag}`;

      ctx.font = '10px monospace';
      const tw = ctx.measureText(label).width;
      const lx = Math.min(Math.max(crosshair.x + 12, 4), Math.max(4, w - tw - 8));
      const ly = Math.max(crosshair.y - 10, 24);
      ctx.fillStyle = '#000000cc';
      ctx.fillRect(lx - 4, ly - 11, tw + 8, 15);
      ctx.fillStyle = '#ffffff';
      ctx.textAlign = 'left';
      ctx.fillText(label, lx, ly);
    }
  }
}

export default function CscanDisplay({
  scanData, params, capturing, sfcwProgress, scaleMode, scaleRange,
  nextIndex, selectedCell, onSelectCell, scanMode, sharedScale, subMode,
  rowScales, scaleScope, scaleLink, projection, onLayout, rootRef, chromeless,
}) {
  const canvasRef = useRef(null);
  const animRef = useRef(null);
  const [crosshair, setCrosshair] = useState(null);
  const isLinear = scaleMode === 'linear';

  useEffect(() => {
    let start = null;
    // Frames are driven by the canvas's OWN window. For the panel that is this
    // window and nothing changes; for the projector portal it is the output
    // window, which matters because a browser throttles requestAnimationFrame
    // on a page it considers hidden -- so driving the projected image from the
    // control window would freeze the wall the moment the operator switched
    // tabs or minimised, which is a thing they will do mid-session.
    let win = window;
    const render = (t) => {
      const canvas = canvasRef.current;
      if (canvas && canvas.ownerDocument.defaultView) win = canvas.ownerDocument.defaultView;
      if (start === null) start = t;
      // Breathing highlight on the next target cell, only while a capture is pending.
      const pulse = capturing ? 0.5 + 0.5 * Math.sin((t - start) / 180) : 0;
      drawCscan(canvas, scanData, params, crosshair, selectedCell, nextIndex, isLinear, scaleRange, pulse, scanMode, sharedScale, subMode, rowScales, scaleScope, scaleLink, projection, onLayout, rootRef, chromeless);
      if (win.closed) return;
      animRef.current = win.requestAnimationFrame(render);
    };
    animRef.current = win.requestAnimationFrame(render);
    return () => {
      // Scheduled on whatever `win` was at the time, which is the same object
      // this reads; an id from the other window would simply not match and
      // cancelling an unknown id is a no-op either way.
      if (animRef.current && !win.closed) win.cancelAnimationFrame(animRef.current);
    };
  }, [scanData, params, crosshair, selectedCell, nextIndex, isLinear, scaleRange, capturing, scanMode, sharedScale, subMode, rowScales, scaleScope, scaleLink, projection, onLayout, rootRef, chromeless]);

  const pick = (e) => {
    const rect = e.currentTarget.getBoundingClientRect();
    const L = cscanLayout(rect.width, rect.height, params, projection, canvasOffsetIn(rootRef, rect));
    return cellAt(
      e.clientX - rect.left, e.clientY - rect.top, L,
      Math.max(1, params.hCount), Math.max(1, params.vCount),
    );
  };

  // The projector instance is output, not a control surface: no progress bar,
  // no pointer interaction, and inline styles rather than utility classes,
  // because it renders into a second document whose stylesheet is a clone and
  // should not be depended on for the geometry of the image itself.
  if (chromeless) {
    return (
      <div style={{ position: 'absolute', inset: 0, background: '#000' }}>
        <canvas
          ref={canvasRef}
          style={{ position: 'absolute', inset: 0, width: '100%', height: '100%' }}
        />
      </div>
    );
  }

  return (
    <div className="flex flex-col w-full h-full">
      {capturing && sfcwProgress && (
        <div className="absolute top-0 left-0 right-0 z-10 h-0.5">
          <div
            className="h-full bg-gradient-to-r from-[#22d3ee] to-[#67e8f9] transition-all duration-200"
            style={{ width: `${(sfcwProgress.step / sfcwProgress.total) * 100}%` }}
          />
        </div>
      )}
      <div className="relative flex-1 min-h-0">
        <canvas
          ref={canvasRef}
          className="absolute inset-0 w-full h-full cursor-crosshair"
          onMouseMove={(e) => {
            const rect = e.currentTarget.getBoundingClientRect();
            setCrosshair({ x: e.clientX - rect.left, y: e.clientY - rect.top });
          }}
          onMouseLeave={() => setCrosshair(null)}
          onClick={(e) => { const c = pick(e); if (c && onSelectCell) onSelectCell(c); }}
        />
      </div>
    </div>
  );
}
