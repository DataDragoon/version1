import { useRef, useEffect, useState } from 'react';
import { BG_STATUS, BG_STATUS_TEXT, bgFailed } from '@/lib/cscanGrid';

const BG = '#000000';
const GRID_COLOR = '#1a1a1a';

function jet(t) {
  t = Math.max(0, Math.min(1, t));
  return [
    Math.round(255 * Math.min(1, Math.max(0, 1.5 - Math.abs(4 * t - 3)))),
    Math.round(255 * Math.min(1, Math.max(0, 1.5 - Math.abs(4 * t - 2)))),
    Math.round(255 * Math.min(1, Math.max(0, 1.5 - Math.abs(4 * t - 1)))),
  ];
}

function drawBscan(canvas, scanData, params, crosshair, isLinear, displayMode, bgDisplay, scaleRange, sharedScale, subMode, showGate, scaleScope, scaleLink) {
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

  if (!scanData || scanData.length === 0) {
    ctx.fillStyle = '#333333';
    ctx.font = '11px monospace';
    ctx.textAlign = 'center';
    ctx.fillText('No scan data — capture positions to build B-scan', w / 2, h / 2);
    return;
  }

  const pad = { top: 24, bottom: 36, left: 50, right: 20 };
  const plotW = w - pad.left - pad.right;
  const plotH = h - pad.top - pad.bottom;

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
  // outside the gate stays drawn, and stays clickable, deliberately.
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

  // Colour limits.
  //
  // Dynamic comes from the SHARED scale, computed once over EVERY cell of the
  // whole C-scan grid, not from this row. Two things were wrong before:
  //
  //   1. Limits were per-row, so the same colour meant a different dB in the
  //      grid on the left and in this pane on the right, and clicking to a
  //      different row silently re-scaled the image.
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

  // Axes: X = depth (left to right), Y = scan position (oldest at top, newest at bottom)
  const cellW = plotW / numBins;
  const cellH = plotH / totalRows;

  // Helper: get magnitudes for a given row index (0 = BG if present, then scan data)
  const getMagsForRow = (rowIdx) => {
    if (hasBg && rowIdx === 0) return bgDisplay.magnitudes;
    const scanIdx = hasBg ? rowIdx - 1 : rowIdx;
    return scanData[scanIdx].magnitudes;
  };

  // Clip region for plot area
  ctx.save();
  ctx.beginPath();
  ctx.rect(pad.left, pad.top, plotW, plotH);
  ctx.clip();

  if (displayMode === 'color') {
    for (let rowIdx = 0; rowIdx < totalRows; rowIdx++) {
      const mags = getMagsForRow(rowIdx);
      const rowPos = (hasBg && rowIdx === 0) ? null : scanData[hasBg ? rowIdx - 1 : rowIdx];
      // A row whose background could not be resolved is not drawn as data --
      // its profile is un-subtracted and would read as the strongest return in
      // the image.
      if (rowPos && bgFailed(rowPos.bg_status)) {
        ctx.fillStyle = '#2a0a10';
        ctx.fillRect(pad.left, pad.top + rowIdx * cellH, plotW, Math.ceil(cellH) + 1);
        ctx.strokeStyle = '#ff4d6d';
        ctx.lineWidth = 1;
        ctx.beginPath();
        for (let x = pad.left; x < pad.left + plotW; x += 10) {
          ctx.moveTo(x, pad.top + rowIdx * cellH);
          ctx.lineTo(x + cellH, pad.top + (rowIdx + 1) * cellH);
        }
        ctx.stroke();
        continue;
      }
      for (let binIdx = 0; binIdx < numBins; binIdx++) {
        const db = (startBin + binIdx < mags.length) ? mags[startBin + binIdx] : dbMin;
        let t;
        if (useLinear) {
          const lin = Math.pow(10, db / 20);
          t = (lin - linMin) / (linMax - linMin);
        } else {
          t = (db - dbMin) / (dbMax - dbMin);
        }
        const [r, g, b] = jet(t);
        ctx.fillStyle = `rgb(${r},${g},${b})`;
        const x = pad.left + binIdx * cellW;
        const y = pad.top + rowIdx * cellH;
        ctx.fillRect(x, y, Math.ceil(cellW) + 1, Math.ceil(cellH) + 1);
      }
    }

    // BG row indicator: orange border at the bottom of the BG row
    if (hasBg) {
      ctx.strokeStyle = '#f59e0b';
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      const bgBottom = pad.top + cellH;
      ctx.moveTo(pad.left, bgBottom);
      ctx.lineTo(w - pad.right, bgBottom);
      ctx.stroke();

      ctx.fillStyle = '#f59e0b';
      ctx.font = 'bold 8px monospace';
      ctx.textAlign = 'right';
      ctx.fillText(bgLabel, w - pad.right - 4, pad.top + cellH / 2 + 3);
    }
  } else {
    // Range profile mode: draw each row as a line graph
    const lineColors = ['#6B9BD2', '#8BB8E8', '#D1855C', '#E8A87C', '#7EC8A0', '#C78BDB', '#E8D06B', '#E87B7B'];

    for (let rowIdx = 0; rowIdx < totalRows; rowIdx++) {
      const mags = getMagsForRow(rowIdx);
      const bandTop = pad.top + rowIdx * cellH;
      const bandH = cellH;
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
        let normalized;
        if (useLinear) {
          const lin = Math.pow(10, db / 20);
          normalized = (lin - linMin) / (linMax - linMin);
        } else {
          normalized = (db - dbMin) / (dbMax - dbMin);
        }
        const x = pad.left + binIdx * cellW + cellW / 2;
        const y = bandTop + bandH - normalized * bandH;

        if (binIdx === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      }
      ctx.stroke();
      ctx.setLineDash([]);
      ctx.globalAlpha = 1.0;

      // BG label
      if (isBgRow) {
        ctx.fillStyle = '#f59e0b';
        ctx.font = 'bold 8px monospace';
        ctx.textAlign = 'right';
        ctx.fillText(bgLabel, w - pad.right - 4, bandTop + bandH / 2 + 3);
      }

      // Subtle separator between rows
      if (rowIdx < totalRows - 1) {
        ctx.strokeStyle = isBgRow ? '#f59e0b44' : '#ffffff08';
        ctx.lineWidth = 0.5;
        ctx.beginPath();
        const sepY = pad.top + (rowIdx + 1) * cellH;
        ctx.moveTo(pad.left, sepY);
        ctx.lineTo(w - pad.right, sepY);
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
    const plotR = pad.left + plotW;
    const xL = empty ? plotR : pad.left + first * cellW;
    const xR = empty ? plotR : Math.min(plotR, pad.left + (last + 1) * cellW);

    ctx.save();
    ctx.beginPath();
    ctx.rect(pad.left, pad.top, plotW, plotH);
    ctx.clip();

    ctx.fillStyle = 'rgba(0,0,0,0.70)';
    if (xL > pad.left) ctx.fillRect(pad.left, pad.top, xL - pad.left, plotH);
    if (xR < plotR) ctx.fillRect(xR, pad.top, plotR - xR, plotH);

    if (!empty) {
      ctx.strokeStyle = '#22d3ee';
      ctx.lineWidth = 1.5;
      ctx.setLineDash([5, 3]);
      ctx.beginPath();
      ctx.moveTo(xL, pad.top);
      ctx.lineTo(xL, pad.top + plotH);
      ctx.moveTo(xR, pad.top);
      ctx.lineTo(xR, pad.top + plotH);
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
    } else {
      ctx.textAlign = 'left';
      ctx.fillText(`${params.gateStart} cm`, Math.min(xL + 3, plotR - 44), pad.top + 10);
      ctx.textAlign = 'right';
      ctx.fillText(`${params.gateEnd} cm`, Math.max(xR - 3, pad.left + 44), pad.top + 10);
      ctx.textAlign = 'center';
      ctx.font = '8px monospace';
      ctx.fillStyle = '#22d3ee99';
      ctx.fillText(`GATE · ${last - first + 1} bins`, (xL + xR) / 2, pad.top + plotH - 4);
    }
  }

  // Grid overlay
  ctx.strokeStyle = GRID_COLOR;
  ctx.lineWidth = 0.5;
  ctx.globalAlpha = 0.4;

  const xTicks = 6;
  for (let i = 0; i <= xTicks; i++) {
    const x = pad.left + (i / xTicks) * plotW;
    ctx.beginPath();
    ctx.moveTo(x, pad.top);
    ctx.lineTo(x, h - pad.bottom);
    ctx.stroke();
  }

  const yTicks = Math.min(numPos, 10);
  for (let i = 0; i <= yTicks; i++) {
    const y = pad.top + (i / yTicks) * plotH;
    ctx.beginPath();
    ctx.moveTo(pad.left, y);
    ctx.lineTo(w - pad.right, y);
    ctx.stroke();
  }
  ctx.globalAlpha = 1.0;

  // X-axis labels (depth)
  for (let i = 0; i <= xTicks; i++) {
    const x = pad.left + (i / xTicks) * plotW;
    const dist = minDist + (i / xTicks) * (maxDist - minDist);
    ctx.fillStyle = '#555555';
    ctx.font = '9px monospace';
    ctx.textAlign = 'center';
    ctx.fillText(`${dist.toFixed(2)}`, x, h - pad.bottom + 14);
  }

  // Y-axis labels (one per row, thinned when there are many)
  ctx.fillStyle = '#555555';
  ctx.font = '9px monospace';
  ctx.textAlign = 'right';
  const rowEvery = Math.max(1, Math.ceil(totalRows / Math.max(1, Math.floor(plotH / 12))));
  for (let rowIdx = 0; rowIdx < totalRows; rowIdx += rowEvery) {
    if (hasBg && rowIdx === 0) continue;
    const y = pad.top + (rowIdx + 0.5) * cellH;
    ctx.fillText(rowLabelFor(hasBg ? rowIdx - 1 : rowIdx), pad.left - 6, y + 3);
  }

  // Axis titles
  ctx.fillStyle = '#444444';
  ctx.font = '9px monospace';
  ctx.textAlign = 'center';
  ctx.fillText('Depth (m)', pad.left + plotW / 2, h - pad.bottom + 28);

  ctx.save();
  ctx.translate(12, pad.top + plotH / 2);
  ctx.rotate(-Math.PI / 2);
  ctx.fillStyle = '#444444';
  ctx.font = '9px monospace';
  ctx.textAlign = 'center';
  ctx.fillText('Position x,y (cm)', 0, 0);
  ctx.restore();

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
  const tag = [perRow ? 'PER ROW' : null, unlinked ? 'UNLINKED' : null].filter(Boolean).join(' · ');
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
    const barX = w - pad.right + 6;
    const barY = pad.top;
    for (let i = 0; i < barH; i++) {
      const t = 1 - i / barH;
      const [r, g, b] = jet(t);
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
    const relX = (crosshair.x - pad.left) / plotW;
    const relY = (crosshair.y - pad.top) / plotH;
    if (relX >= 0 && relX <= 1 && relY >= 0 && relY <= 1) {
      const binIdx = Math.min(numBins - 1, Math.floor(relX * numBins));
      const rowIdx = Math.min(totalRows - 1, Math.floor(relY * totalRows));
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
      ctx.moveTo(pad.left, crosshair.y);
      ctx.lineTo(w - pad.right, crosshair.y);
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
      const labelX = crosshair.x + 10 > w - 220 ? crosshair.x - 220 : crosshair.x + 10;
      ctx.fillText(label, labelX, crosshair.y - 8);
    }
  }
}

export default function BscanDisplay({ scanData, bgDisplay, params, capturing, sfcwProgress, scaleMode, displayMode, scaleRange, sharedScale, subMode, showGate, scaleScope, scaleLink }) {
  const canvasRef = useRef(null);
  const animRef = useRef(null);
  const [crosshair, setCrosshair] = useState(null);

  const isLinear = scaleMode === 'linear';
  const mode = displayMode || 'color';

  // Kept as a rAF loop rather than a one-shot draw: drawBscan sizes itself from
  // getBoundingClientRect(), so redrawing every frame is what makes the canvas
  // follow a panel resize. Nothing here is animated any more -- there used to be
  // a per-row lerp toward lidar-derived range-bin shifts, but the shifts were
  // hard-wired to zero at the call site and the producer was dead code, so the
  // whole path was removed (2026-08-30).
  useEffect(() => {
    const render = () => {
      drawBscan(canvasRef.current, scanData, params, crosshair, isLinear, mode, bgDisplay, scaleRange, sharedScale, subMode, showGate, scaleScope, scaleLink);
      animRef.current = requestAnimationFrame(render);
    };
    animRef.current = requestAnimationFrame(render);
    return () => { if (animRef.current) cancelAnimationFrame(animRef.current); };
  }, [scanData, params, crosshair, isLinear, mode, bgDisplay, scaleRange, sharedScale, subMode, showGate, scaleScope, scaleLink]);

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
