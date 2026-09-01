import { useRef, useEffect, useState } from 'react';
import { orderedCellForIndex, buildCscanGrid, BG_STATUS, BG_STATUS_TEXT } from '@/lib/cscanGrid';

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

// Plan-view geometry: the grid keeps its physical aspect ratio inside the plot
// box, so a wide-and-short sweep looks wide and short.
function layout(w, h, params) {
  const pad = { top: 24, bottom: 38, left: 52, right: 64 };
  const plotW = w - pad.left - pad.right;
  const plotH = h - pad.top - pad.bottom;

  const hCount = Math.max(1, params.hCount);
  const vCount = Math.max(1, params.vCount);
  // A single line in an axis still needs a finite cell size to draw.
  const cellSpanX = params.hStep > 0 ? params.hStep : 1;
  const cellSpanY = params.vStep > 0 ? params.vStep : 1;
  const spanX = hCount * cellSpanX;
  const spanY = vCount * cellSpanY;
  const scale = Math.min(plotW / spanX, plotH / spanY);

  const gridW = spanX * scale;
  const gridH = spanY * scale;
  const originX = pad.left + (plotW - gridW) / 2;
  // Vertical grows upward: iy = 0 sits at the bottom of the grid box.
  const originY = pad.top + (plotH + gridH) / 2;

  return {
    pad, plotW, plotH, scale, gridW, gridH, originX, originY,
    cellW: cellSpanX * scale, cellH: cellSpanY * scale,
  };
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

function drawCscan(canvas, scanData, params, crosshair, selected, nextIndex, isLinear, scaleRange, pulse, scanMode, sharedScale, subMode) {
  if (!canvas) return;
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
  if (w < 80 || h < 80) return;

  const L = layout(w, h, params);
  const { hStep, vStep, gateStart, gateEnd, metric } = params;
  const grid = buildCscanGrid(scanData, params);
  const total = grid.hCount * grid.vCount;

  // Colour limits. Dynamic now comes from the SHARED scale computed over the
  // whole grid (every bin of every valid cell, percentile-clipped), not from
  // this grid's own gated min/max -- so the colour bar here and the one on the
  // B-scan pane mean the same dB, and a full min/max stretch of a flat residual
  // field no longer manufactures rainbow structure out of noise.
  let dbMin;
  let dbMax;
  if (scaleRange && !scaleRange.dynamic) {
    dbMin = scaleRange.min;
    dbMax = scaleRange.max;
  } else if (sharedScale && isFinite(sharedScale.min) && isFinite(sharedScale.max)) {
    dbMin = sharedScale.min;
    dbMax = sharedScale.max;
  } else if (isFinite(grid.min) && isFinite(grid.max)) {
    dbMin = grid.min;
    dbMax = grid.max;
  } else {
    dbMin = -90;
    dbMax = -20;
  }
  if (dbMax - dbMin < 1) { dbMin -= 0.5; dbMax += 0.5; }
  // A magnitude difference is a dB RATIO centred on zero, not an absolute
  // level, so the linear warp (which maps 10^(db/20), an amplitude) is
  // meaningless on it and is bypassed.
  const isDiff = subMode === 'magnitude';
  const useLinear = isLinear && !isDiff;
  const linMin = Math.pow(10, dbMin / 20);
  const linMax = Math.pow(10, dbMax / 20);

  const norm = (db) => (useLinear
    ? (Math.pow(10, db / 20) - linMin) / (linMax - linMin)
    : (db - dbMin) / (dbMax - dbMin));

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
        const [cr, cg, cb] = jet(norm(cell.value));
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

  // Snake path through the cells already captured — makes the raster order,
  // and any hole left by an undo, visible at a glance.
  if (grid.filled > 1 && L.cellW > 6 && L.cellH > 6) {
    ctx.strokeStyle = '#ffffff33';
    ctx.lineWidth = 1;
    ctx.setLineDash([3, 3]);
    ctx.beginPath();
    let started = false;
    for (let i = 0; i < total; i++) {
      const { ix, iy } = orderedCellForIndex(i, grid.hCount, grid.vCount, scanMode);
      if (iy >= grid.vCount || iy < 0) break;
      if (!grid.cells[iy * grid.hCount + ix]) continue;
      const r = cellRect(ix, iy, L);
      const cx = r.x + r.w / 2;
      const cy = r.y + r.h / 2;
      if (!started) { ctx.moveTo(cx, cy); started = true; } else ctx.lineTo(cx, cy);
    }
    ctx.stroke();
    ctx.setLineDash([]);
  }

  // Start marker on whichever corner the active raster begins from: bottom-left
  // by hand, top-left under the rover (which snakes downwards instead).
  {
    const r = cellRect(0, scanMode === 'rover' ? grid.vCount - 1 : 0, L);
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
  if (selected && selected.ix < grid.hCount && selected.iy < grid.vCount) {
    const r = cellRect(selected.ix, selected.iy, L);
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 1.5;
    ctx.strokeRect(r.x + 0.75, r.y + 0.75, r.w - 1.5, r.h - 1.5);
  }

  // Grid frame
  ctx.strokeStyle = '#2a2a2a';
  ctx.lineWidth = 1;
  ctx.strokeRect(L.originX, L.originY - L.gridH, L.gridW, L.gridH);

  // Axis ticks at cell centres, thinned so labels never collide
  ctx.font = '9px monospace';
  ctx.fillStyle = '#555555';
  ctx.textAlign = 'center';
  const xEvery = Math.max(1, Math.ceil(grid.hCount / Math.max(1, Math.floor(L.gridW / 34))));
  for (let ix = 0; ix < grid.hCount; ix += xEvery) {
    const r = cellRect(ix, 0, L);
    ctx.fillText((ix * hStep).toFixed(hStep % 1 === 0 ? 0 : 1), r.x + r.w / 2, L.originY + 14);
  }
  const yEvery = Math.max(1, Math.ceil(grid.vCount / Math.max(1, Math.floor(L.gridH / 16))));
  ctx.textAlign = 'right';
  for (let iy = 0; iy < grid.vCount; iy += yEvery) {
    const r = cellRect(0, iy, L);
    ctx.fillText((iy * vStep).toFixed(vStep % 1 === 0 ? 0 : 1), L.originX - 6, r.y + r.h / 2 + 3);
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
  ctx.fillText(
    `C-SCAN (${String(metric).toUpperCase()} @ ${gateStart}-${gateEnd} cm${isDiff ? ' · Δ MAG' : ''})`,
    L.pad.left, 14);
  ctx.fillStyle = '#444444';
  ctx.font = '9px monospace';
  ctx.textAlign = 'right';
  ctx.fillText(`${grid.filled} / ${total} cells`, w - L.pad.right, 14);

  // Colour bar
  const manual = !!(scaleRange && !scaleRange.dynamic);
  const barW = 12;
  const barH = L.plotH;
  const barX = w - L.pad.right + 16;
  const barY = L.pad.top;
  for (let i = 0; i < barH; i++) {
    const [r, g, b] = jet(1 - i / barH);
    ctx.fillStyle = `rgb(${r},${g},${b})`;
    ctx.fillRect(barX, barY + i, barW, 1);
  }
  ctx.strokeStyle = manual ? '#f59e0b' : '#2a2a2a';
  ctx.lineWidth = 1;
  ctx.strokeRect(barX - 0.5, barY - 0.5, barW + 1, barH + 1);
  ctx.fillStyle = manual ? '#f59e0b' : '#555555';
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
  if (manual) {
    ctx.save();
    ctx.translate(barX + barW + 11, barY + barH / 2);
    ctx.rotate(Math.PI / 2);
    ctx.textAlign = 'center';
    ctx.fillText('MANUAL', 0, 0);
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
}) {
  const canvasRef = useRef(null);
  const animRef = useRef(null);
  const [crosshair, setCrosshair] = useState(null);
  const isLinear = scaleMode === 'linear';

  useEffect(() => {
    let start = null;
    const render = (t) => {
      if (start === null) start = t;
      // Breathing highlight on the next target cell, only while a capture is pending.
      const pulse = capturing ? 0.5 + 0.5 * Math.sin((t - start) / 180) : 0;
      drawCscan(canvasRef.current, scanData, params, crosshair, selectedCell, nextIndex, isLinear, scaleRange, pulse, scanMode, sharedScale, subMode);
      animRef.current = requestAnimationFrame(render);
    };
    animRef.current = requestAnimationFrame(render);
    return () => { if (animRef.current) cancelAnimationFrame(animRef.current); };
  }, [scanData, params, crosshair, selectedCell, nextIndex, isLinear, scaleRange, capturing, scanMode, sharedScale, subMode]);

  const pick = (e) => {
    const rect = e.currentTarget.getBoundingClientRect();
    const L = layout(rect.width, rect.height, params);
    return cellAt(
      e.clientX - rect.left, e.clientY - rect.top, L,
      Math.max(1, params.hCount), Math.max(1, params.vCount),
    );
  };

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
