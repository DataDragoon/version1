import { useRef, useEffect, useCallback, useState } from 'react';
import { saftFocusedProfile, metricOnProfile, gateDepths as gateDepthsIn } from '@/lib/saft';

const BG = '#000000';

function jet(t) {
  t = Math.max(0, Math.min(1, t));
  return [
    Math.round(255 * Math.min(1, Math.max(0, 1.5 - Math.abs(4 * t - 3)))),
    Math.round(255 * Math.min(1, Math.max(0, 1.5 - Math.abs(4 * t - 2)))),
    Math.round(255 * Math.min(1, Math.max(0, 1.5 - Math.abs(4 * t - 1)))),
  ];
}

function computeMapData(bscanData, gateStart, gateEnd, metric, focusEnabled, focusAperture, stepSize) {
  if (!bscanData || bscanData.length === 0) return null;

  const numPositions = bscanData.length;
  const gateStartM = gateStart / 100;
  const gateEndM = gateEnd / 100;
  const stepM = stepSize / 100;

  // Determine depth sample points within the gate
  const refDists = bscanData.find(p => p.distances)?.distances;
  if (!refDists) return null;

  const gateDepths = gateDepthsIn(refDists, gateStartM, gateEndM);
  if (gateDepths.length === 0) return null;

  // A linear scan's capture index IS its position along the line, so `n` is the
  // array index and the offsets below come out exactly as they did before this
  // was shared with the C-scan.
  const traces = bscanData.map((p, i) => ({ magnitudes: p.magnitudes, distances: p.distances, n: i }));

  const intensities = [];

  for (let i = 0; i < numPositions; i++) {
    if (!focusEnabled) {
      // Unfocused: just gate the local range profile
      const pos = bscanData[i];
      if (!pos.magnitudes || !pos.distances) {
        intensities.push(-Infinity);
        continue;
      }
      intensities.push(applyMetric(pos.magnitudes, pos.distances, gateStartM, gateEndM, metric));
    } else {
      // Focused: back-project the neighbours within the aperture, then apply the
      // same metric over the focused profile. See lib/saft.js.
      const halfAp = Math.floor(focusAperture / 2);
      intensities.push(metricOnProfile(
        saftFocusedProfile(traces, i, gateDepths, stepM, halfAp), metric));
    }
  }

  return intensities;
}

function applyMetric(mags, dists, gateStartM, gateEndM, metric) {
  let value = -Infinity;
  if (metric === 'peak') {
    for (let j = 0; j < mags.length; j++) {
      if (dists[j] >= gateStartM && dists[j] <= gateEndM) {
        if (mags[j] > value) value = mags[j];
      }
    }
  } else if (metric === 'energy') {
    let sum = 0, count = 0;
    for (let j = 0; j < mags.length; j++) {
      if (dists[j] >= gateStartM && dists[j] <= gateEndM) {
        const linear = Math.pow(10, mags[j] / 20);
        sum += linear * linear;
        count++;
      }
    }
    if (count > 0) value = 10 * Math.log10(sum / count + 1e-12);
  } else if (metric === 'mean') {
    let sum = 0, count = 0;
    for (let j = 0; j < mags.length; j++) {
      if (dists[j] >= gateStartM && dists[j] <= gateEndM) {
        sum += mags[j];
        count++;
      }
    }
    if (count > 0) value = sum / count;
  }
  return value;
}


function drawMap(canvas, bscanData, crosshair, params) {
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

  const { gateStart, gateEnd, dynRange, metric, stepSize, focusEnabled, focusAperture } = params;

  if (!bscanData || bscanData.length === 0) {
    ctx.fillStyle = '#333333';
    ctx.font = '11px monospace';
    ctx.textAlign = 'center';
    ctx.fillText('No B-scan data — capture or load a scan', w / 2, h / 2);
    return;
  }

  const intensities = computeMapData(bscanData, gateStart, gateEnd, metric, focusEnabled, focusAperture, stepSize);
  if (!intensities) return;

  const numPositions = intensities.length;

  let valMax = -Infinity;
  let valMin = Infinity;
  for (let i = 0; i < numPositions; i++) {
    if (intensities[i] > -Infinity && intensities[i] < Infinity) {
      if (intensities[i] > valMax) valMax = intensities[i];
      if (intensities[i] < valMin) valMin = intensities[i];
    }
  }
  if (!isFinite(valMin)) valMin = -90;
  if (!isFinite(valMax)) valMax = -20;
  if (valMax - valMin < 1) { valMin -= 0.5; valMax += 0.5; }
  if (dynRange && (valMax - valMin) > dynRange) {
    valMin = valMax - dynRange;
  }

  const pad = { top: 24, bottom: 48, left: 50, right: 50 };
  const plotW = w - pad.left - pad.right;
  const plotH = h - pad.top - pad.bottom;

  // Draw 1D intensity bar (horizontal strip showing position along x-axis)
  const barHeight = Math.min(80, plotH * 0.3);
  const barTop = pad.top + (plotH - barHeight) / 2;
  const cellW = plotW / numPositions;

  for (let i = 0; i < numPositions; i++) {
    const val = intensities[i];
    const t = val > -Infinity ? (val - valMin) / (valMax - valMin) : 0;
    const [r, g, b] = jet(Math.max(0, Math.min(1, t)));
    ctx.fillStyle = `rgb(${r},${g},${b})`;
    ctx.fillRect(pad.left + i * cellW, barTop, Math.ceil(cellW) + 1, barHeight);
  }

  // Outline
  ctx.strokeStyle = '#333';
  ctx.lineWidth = 1;
  ctx.strokeRect(pad.left, barTop, plotW, barHeight);

  // Line plot below the heatmap bar
  const lineTop = barTop + barHeight + 30;
  const lineH = pad.top + plotH - lineTop;

  if (lineH > 30) {
    ctx.strokeStyle = '#333';
    ctx.lineWidth = 0.5;
    ctx.beginPath();
    ctx.moveTo(pad.left, lineTop);
    ctx.lineTo(pad.left, lineTop + lineH);
    ctx.lineTo(pad.left + plotW, lineTop + lineH);
    ctx.stroke();

    ctx.beginPath();
    ctx.strokeStyle = '#4ade80';
    ctx.lineWidth = 1.5;
    for (let i = 0; i < numPositions; i++) {
      const x = pad.left + (i + 0.5) * cellW;
      const val = intensities[i];
      const t = val > -Infinity ? (val - valMin) / (valMax - valMin) : 0;
      const y = lineTop + lineH - t * lineH;
      if (i === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    }
    ctx.stroke();

    // Y-axis labels for line plot
    const yTicks = 4;
    for (let i = 0; i <= yTicks; i++) {
      const y = lineTop + (i / yTicks) * lineH;
      const val = valMax - (i / yTicks) * (valMax - valMin);
      ctx.fillStyle = '#555';
      ctx.font = '8px monospace';
      ctx.textAlign = 'right';
      ctx.fillText(`${val.toFixed(0)}`, pad.left - 6, y + 3);

      ctx.strokeStyle = '#1a1a1a';
      ctx.lineWidth = 0.5;
      ctx.globalAlpha = 0.4;
      ctx.beginPath();
      ctx.moveTo(pad.left, y);
      ctx.lineTo(pad.left + plotW, y);
      ctx.stroke();
      ctx.globalAlpha = 1;
    }
  }

  // X-axis labels (position in cm)
  const xTicks = Math.min(numPositions, 10);
  for (let i = 0; i <= xTicks; i++) {
    const x = pad.left + (i / xTicks) * plotW;
    const posCm = (i / xTicks) * (numPositions - 1) * stepSize;
    ctx.fillStyle = '#555';
    ctx.font = '9px monospace';
    ctx.textAlign = 'center';
    ctx.fillText(`${posCm.toFixed(0)}`, x, pad.top + plotH + 14);
  }

  // Axis title
  ctx.fillStyle = '#444';
  ctx.font = '9px monospace';
  ctx.textAlign = 'center';
  ctx.fillText('Position (cm)', pad.left + plotW / 2, pad.top + plotH + 30);

  // Title
  ctx.fillStyle = '#4ade80';
  ctx.font = 'bold 10px monospace';
  ctx.textAlign = 'left';
  const metricLabel = metric === 'peak' ? 'Peak' : metric === 'energy' ? 'Energy' : 'Mean';
  const focusLabel = focusEnabled ? ` | SAFT ×${focusAperture}` : '';
  ctx.fillText(`2D MAP — ${metricLabel} [gate ${gateStart}–${gateEnd} cm]${focusLabel}`, pad.left, 14);

  ctx.fillStyle = '#444';
  ctx.font = '9px monospace';
  ctx.textAlign = 'right';
  ctx.fillText(`${numPositions} positions`, w - pad.right, 14);

  // Color bar (vertical, right side)
  const cbarW = 12;
  const cbarX = w - pad.right + 16;
  const cbarH = barHeight;
  for (let i = 0; i < cbarH; i++) {
    const t = 1 - i / cbarH;
    const [r, g, b] = jet(t);
    ctx.fillStyle = `rgb(${r},${g},${b})`;
    ctx.fillRect(cbarX, barTop + i, cbarW, 1);
  }
  ctx.fillStyle = '#555';
  ctx.font = '8px monospace';
  ctx.textAlign = 'left';
  ctx.fillText(`${valMax.toFixed(0)} dB`, cbarX, barTop - 4);
  ctx.fillText(`${valMin.toFixed(0)} dB`, cbarX, barTop + cbarH + 10);

  // Crosshair
  if (crosshair) {
    const relX = (crosshair.x - pad.left) / plotW;
    if (relX >= 0 && relX <= 1) {
      const posIdx = Math.min(numPositions - 1, Math.floor(relX * numPositions));
      const posCm = posIdx * stepSize;
      const val = intensities[posIdx];

      ctx.setLineDash([3, 3]);
      ctx.strokeStyle = '#ffffff44';
      ctx.lineWidth = 0.5;
      ctx.beginPath();
      ctx.moveTo(crosshair.x, pad.top);
      ctx.lineTo(crosshair.x, pad.top + plotH);
      ctx.stroke();
      ctx.setLineDash([]);

      ctx.fillStyle = '#fff';
      ctx.font = '10px monospace';
      ctx.textAlign = 'left';
      const label = `pos ${posCm.toFixed(1)} cm, ${val > -Infinity ? val.toFixed(1) + ' dB' : '—'}`;
      const labelX = crosshair.x + 10 > w - 180 ? crosshair.x - 180 : crosshair.x + 10;
      ctx.fillText(label, labelX, crosshair.y - 8);
    }
  }
}

export default function MapDisplay({ bscanData, gateStart, gateEnd, dynRange, metric, stepSize, focusEnabled, focusAperture }) {
  const canvasRef = useRef(null);
  const animRef = useRef(null);
  const [crosshair, setCrosshair] = useState(null);

  const params = { gateStart, gateEnd, dynRange, metric, stepSize, focusEnabled, focusAperture };

  const draw = useCallback(() => {
    drawMap(canvasRef.current, bscanData, crosshair, params);
  }, [bscanData, crosshair, gateStart, gateEnd, dynRange, metric, stepSize, focusEnabled, focusAperture]);

  useEffect(() => {
    const render = () => {
      draw();
      animRef.current = requestAnimationFrame(render);
    };
    animRef.current = requestAnimationFrame(render);
    return () => { if (animRef.current) cancelAnimationFrame(animRef.current); };
  }, [draw]);

  return (
    <div className="flex flex-col w-full h-full">
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
