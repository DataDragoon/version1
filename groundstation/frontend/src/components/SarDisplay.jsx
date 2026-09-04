import { useRef, useEffect, useCallback, useState } from 'react';
import { COLORMAPS } from '@/lib/imagingEffects';

const BG = '#000000';

// Shared with the Imaging Bench rather than redefined here -- one implementation, as with
// windowFn and CFAR. Default inferno: it and viridis are perceptually uniform, where jet
// is not (its lightness is non-monotonic, so it invents banded structure that is not in
// the data -- on an image whose whole question is "is that feature real", exactly the
// wrong failure mode). jet stays available because it is what earlier images were read in.
//
// The dropdown drives the amplitude and combined panes ONLY -- see cohMap below.

// Coherence keeps its own fixed ramp, deliberately, and is NOT switchable. The two panes
// in split view are different quantities on different scales -- a relative dB amplitude
// and an absolute 0-1 ratio -- and the entire point of showing them at once is reading one
// against the other, which a shared palette makes harder rather than easier. Dark blue ->
// teal -> white, monotonic in lightness and nothing like inferno or viridis at a glance.
function cohMap(t) {
  t = Math.max(0, Math.min(1, t));
  return [
    Math.round(255 * Math.max(0, Math.min(1, 1.6 * t - 0.6))),
    Math.round(255 * Math.max(0, Math.min(1, 1.25 * t))),
    Math.round(255 * Math.max(0, Math.min(1, 0.35 + 0.65 * t))),
  ];
}

// One offscreen canvas per pane, sized in DATA pixels and scaled up on draw.
// 100x100 cells x two panes x 60 fps is 1.2M fillRect calls a second, which is
// exactly the cost ImagingDisplay was rewritten to avoid; putImageData plus one
// scaled drawImage is a single blit instead.
function blit(ctx, off, vals, pixelsX, pixelsZ, vMin, vMax, cmap, dest) {
  const oc = off.canvas;
  if (oc.width !== pixelsZ || oc.height !== pixelsX) {
    oc.width = pixelsZ;
    oc.height = pixelsX;
  }
  const id = off.createImageData(pixelsZ, pixelsX);
  const d = id.data;
  const span = vMax - vMin || 1;
  for (let xi = 0; xi < pixelsX; xi++) {
    for (let zi = 0; zi < pixelsZ; zi++) {
      const v = vals[zi * pixelsX + xi];
      const [r, g, b] = cmap((v - vMin) / span);
      const o = (xi * pixelsZ + zi) * 4;
      d[o] = r; d[o + 1] = g; d[o + 2] = b; d[o + 3] = 255;
    }
  }
  off.putImageData(id, 0, 0);
  ctx.imageSmoothingEnabled = false;
  ctx.drawImage(oc, dest.x, dest.y, dest.w, dest.h);
}

// Floor on the coherence weight, so a zero-coherence pixel lands at -40 dB instead of
// -infinity and cannot swallow the dynamic range on its own.
const COH_FLOOR = 0.01;

function drawPane(canvas, off, sarResult, crosshair, opts) {
  if (!canvas) return;
  const ctx = canvas.getContext('2d');
  const rect = canvas.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  // Assigning width/height reallocates the backing store AND clears it, so doing it
  // unconditionally inside a 60 fps rAF loop reallocated two canvases 120 times a
  // second. Only touch it when the box has actually changed; the redraw below still
  // happens every frame, which is what makes the pane track a panel resize.
  const wantW = Math.round(rect.width * dpr);
  const wantH = Math.round(rect.height * dpr);
  if (canvas.width !== wantW || canvas.height !== wantH) {
    canvas.width = wantW;
    canvas.height = wantH;
  }
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  const w = rect.width;
  const h = rect.height;

  ctx.fillStyle = BG;
  ctx.fillRect(0, 0, w, h);

  const isCoh = opts.mode === 'coherence';
  const isCombined = opts.mode === 'combined';
  const cmap = isCoh ? cohMap : (COLORMAPS[opts.colormap] || COLORMAPS.inferno);

  const msg = (text) => {
    ctx.fillStyle = '#333333';
    ctx.font = '11px monospace';
    ctx.textAlign = 'center';
    ctx.fillText(text, w / 2, h / 2);
  };

  // Deliberately silent: Viewport renders one centred "No SAR image" overlay over the
  // whole pane stack, and drawing it here too put the same sentence on screen three
  // times in split view.
  if (!sarResult || !sarResult.image) return;
  if (isCoh && !sarResult.coherence) {
    msg('Coherence needs Coherent mode — incoherent SAR has no phase');
    return;
  }

  const { image, coherence, pixelsX, pixelsZ, depthMax, apertureLength } = sarResult;
  const src = isCoh ? coherence : image;
  // Combined view degrades to plain amplitude rather than going blank when there is no
  // coherence to weight by (incoherent SAR), and says so in the title.
  const weighting = isCombined && !!coherence;

  const pad = { top: 24, bottom: 34, left: 50, right: 40 };
  const plotW = w - pad.left - pad.right;
  const plotH = h - pad.top - pad.bottom;
  if (plotW <= 4 || plotH <= 4) return;

  const isLinear = opts.scaleMode === 'linear';

  // Coherence is an ABSOLUTE ratio, so it is pinned to 0..1 and never dynamically
  // scaled. Stretching it would destroy the whole point: 0.9 means "the aperture
  // focused this" and 0.2 means "it did not" regardless of what else is in frame,
  // and that reading has to survive between scans.
  let vMin, vMax, displayVals;
  if (isCoh) {
    vMin = 0;
    vMax = 1;
    displayVals = src;
  } else {
    displayVals = new Float64Array(src.length);
    vMin = Infinity; vMax = -Infinity;
    for (let i = 0; i < src.length; i++) {
      // Weighting is a straight multiply of the LINEAR amplitude by the coherence, i.e.
      // +20log10(coh) in dB: coherence 1 leaves a pixel alone, 0.5 costs it 6 dB, and
      // clutter that was merely bright sinks. Note the amplitude already contains the
      // RAW coherence once by construction (|sum| = coh_raw * sum|.|), so this is a
      // second, deliberate weighting -- and by the DEBIASED figure, which is a different
      // quantity, so it is not simply squaring what is already there. On the test scan
      // it widened target-vs-clutter from 3.8 dB to 7.1 dB.
      const dbv = weighting
        ? src[i] + 20 * Math.log10(Math.max(coherence[i], COH_FLOOR))
        : src[i];
      const v = isLinear ? Math.pow(10, dbv / 20) : dbv;
      displayVals[i] = v;
      if (v < vMin) vMin = v;
      if (v > vMax) vMax = v;
    }
    if (!isFinite(vMin)) vMin = isLinear ? 0 : -90;
    if (!isFinite(vMax)) vMax = isLinear ? 1 : -20;
    if (vMax - vMin < (isLinear ? 0.001 : 1)) { vMin -= isLinear ? 0.0005 : 0.5; vMax += isLinear ? 0.0005 : 0.5; }
    // Dynamic range is a RATIO below the peak, so it means the same thing in both
    // scales -- it just has to be applied multiplicatively in linear. It used to be
    // skipped entirely when linear was selected, leaving a slider that read "dB",
    // stayed enabled, and did nothing.
    if (opts.dynRange) {
      if (isLinear) {
        const floor = vMax * Math.pow(10, -opts.dynRange / 20);
        if (vMin < floor) vMin = floor;
      } else if (vMax - vMin > opts.dynRange) {
        vMin = vMax - opts.dynRange;
      }
    }
  }

  blit(ctx, off, displayVals, pixelsX, pixelsZ, vMin, vMax, cmap,
       { x: pad.left, y: pad.top, w: plotW, h: plotH });

  // Grid
  ctx.strokeStyle = '#1a1a1a';
  ctx.lineWidth = 0.5;
  ctx.globalAlpha = 0.4;
  const xTicks = 5;
  const yTicks = 5;
  for (let i = 0; i <= xTicks; i++) {
    const x = pad.left + (i / xTicks) * plotW;
    ctx.beginPath(); ctx.moveTo(x, pad.top); ctx.lineTo(x, h - pad.bottom); ctx.stroke();
  }
  for (let i = 0; i <= yTicks; i++) {
    const y = pad.top + (i / yTicks) * plotH;
    ctx.beginPath(); ctx.moveTo(pad.left, y); ctx.lineTo(w - pad.right, y); ctx.stroke();
  }
  ctx.globalAlpha = 1.0;

  // X-axis labels (depth). True depth below the wall face now, not apparent range.
  for (let i = 0; i <= xTicks; i++) {
    const x = pad.left + (i / xTicks) * plotW;
    ctx.fillStyle = '#555555';
    ctx.font = '9px monospace';
    ctx.textAlign = 'center';
    ctx.fillText(`${((i / xTicks) * depthMax * 100).toFixed(1)}`, x, h - pad.bottom + 13);
  }
  for (let i = 0; i <= yTicks; i++) {
    const y = pad.top + (i / yTicks) * plotH;
    ctx.fillStyle = '#555555';
    ctx.font = '9px monospace';
    ctx.textAlign = 'right';
    ctx.fillText(`${((i / yTicks) * apertureLength * 100).toFixed(1)}`, pad.left - 6, y + 3);
  }

  ctx.fillStyle = '#444444';
  ctx.font = '9px monospace';
  ctx.textAlign = 'center';
  ctx.fillText('Depth into wall (cm)', pad.left + plotW / 2, h - pad.bottom + 26);
  ctx.save();
  ctx.translate(12, pad.top + plotH / 2);
  ctx.rotate(-Math.PI / 2);
  ctx.textAlign = 'center';
  ctx.fillStyle = '#444444';
  ctx.font = '9px monospace';
  ctx.fillText('Position (cm)', 0, 0);
  ctx.restore();

  // Title
  ctx.fillStyle = isCoh ? '#4ecdc4' : '#4ade80';
  ctx.font = 'bold 10px monospace';
  ctx.textAlign = 'left';
  const title = isCoh
    ? 'SAR COHERENCE (0–1, fixed)'
    : (isCombined
        ? (weighting
            ? `SAR AMPLITUDE × COHERENCE (${isLinear ? 'linear' : 'dB'})`
            : 'SAR AMPLITUDE — incoherent mode, nothing to weight by')
        : `SAR AMPLITUDE (${isLinear ? 'linear' : 'dB'})`);
  ctx.fillText(title, pad.left, 14);

  ctx.fillStyle = '#444444';
  ctx.font = '9px monospace';
  ctx.textAlign = 'right';
  const geom = sarResult.layered
    ? `layered ${(sarResult.wallThicknessCm ?? 0).toFixed(0)}cm`
    : 'straight ray';
  const meta = isCoh
    ? `${geom} | ${sarResult.numPositions} pos`
    : `εr ${(sarResult.epsilonR ?? 1).toFixed(2)} | ${geom} | ${sarResult.windowType || 'rect'} | ${sarResult.numPositions} pos`;
  ctx.fillText(meta, w - pad.right, 14);

  // Colour bar
  const barW = 12;
  const barX = w - pad.right + 8;
  for (let i = 0; i < plotH; i++) {
    const [r, g, b] = cmap(1 - i / plotH);
    ctx.fillStyle = `rgb(${r},${g},${b})`;
    ctx.fillRect(barX, pad.top + i, barW, 1);
  }
  ctx.fillStyle = '#555555';
  ctx.font = '8px monospace';
  ctx.textAlign = 'left';
  const fmt = (v) => (isCoh ? v.toFixed(2) : (isLinear ? v.toFixed(3) : v.toFixed(0)));
  ctx.fillText(fmt(vMax), barX, pad.top - 4);
  ctx.fillText(fmt(vMin), barX, pad.top + plotH + 10);

  // Crosshair — reports BOTH quantities wherever it is, so the panes can be read
  // against each other without moving the mouse between them.
  if (crosshair) {
    const relX = (crosshair.x - pad.left) / plotW;
    const relY = (crosshair.y - pad.top) / plotH;
    if (relX >= 0 && relX <= 1 && relY >= 0 && relY <= 1) {
      const zi = Math.min(pixelsZ - 1, Math.floor(relX * pixelsZ));
      const xi = Math.min(pixelsX - 1, Math.floor(relY * pixelsX));

      ctx.setLineDash([3, 3]);
      ctx.strokeStyle = '#ffffff44';
      ctx.lineWidth = 0.5;
      ctx.beginPath(); ctx.moveTo(crosshair.x, pad.top); ctx.lineTo(crosshair.x, h - pad.bottom); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(pad.left, crosshair.y); ctx.lineTo(w - pad.right, crosshair.y); ctx.stroke();
      ctx.setLineDash([]);

      const dbVal = image[zi * pixelsX + xi];
      const ampLabel = isLinear ? Math.pow(10, dbVal / 20).toFixed(4) : `${dbVal.toFixed(1)}dB`;
      const cohLabel = coherence ? `, coh ${coherence[zi * pixelsX + xi].toFixed(2)}` : '';
      const wLabel = weighting
        ? `, weighted ${(dbVal + 20 * Math.log10(Math.max(coherence[zi * pixelsX + xi], COH_FLOOR))).toFixed(1)}dB`
        : '';
      const label = `pos ${(relY * apertureLength * 100).toFixed(1)}cm, depth ${(relX * depthMax * 100).toFixed(1)}cm, ${ampLabel}${cohLabel}${wLabel}`;
      ctx.fillStyle = '#ffffff';
      ctx.font = '10px monospace';
      ctx.textAlign = 'left';
      const labelX = crosshair.x + 10 > w - 250 ? crosshair.x - 250 : crosshair.x + 10;
      ctx.fillText(label, labelX, crosshair.y - 8);
    }
  }
}

function Pane({ sarResult, mode, scaleMode, dynRange, colormap }) {
  const canvasRef = useRef(null);
  const offRef = useRef(null);
  const animRef = useRef(null);
  const [crosshair, setCrosshair] = useState(null);

  if (!offRef.current && typeof document !== 'undefined') {
    offRef.current = document.createElement('canvas').getContext('2d');
  }

  const draw = useCallback(() => {
    drawPane(canvasRef.current, offRef.current, sarResult, crosshair,
             { mode, scaleMode: scaleMode || 'db', dynRange, colormap });
  }, [sarResult, crosshair, mode, scaleMode, dynRange, colormap]);

  // Kept as a rAF loop rather than a draw-on-change effect for the same reason
  // BscanDisplay is: the canvas sizes itself from getBoundingClientRect(), so
  // redrawing every frame is what makes it track a panel resize. Replacing it
  // needs a ResizeObserver first.
  useEffect(() => {
    const render = () => {
      draw();
      animRef.current = requestAnimationFrame(render);
    };
    animRef.current = requestAnimationFrame(render);
    return () => { if (animRef.current) cancelAnimationFrame(animRef.current); };
  }, [draw]);

  return (
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
  );
}

// Both panes are shown at once rather than behind a toggle: amplitude answers "how
// bright is this" and coherence answers "did the aperture actually focus it", and the
// useful reading is the pair, not either alone. On a real 60-position scan the
// amplitude image put a known pipe within 3-4 dB of two other lobes at the same depth;
// coherence pushed the mid-scan one down to 0.56 against the pipe's 0.82 and zeroed two
// more. It did NOT uniquely pick the pipe -- a feature at the opposite aperture edge
// scored comparably (0.89) -- so read the pair, and treat an edge feature with
// suspicion until the scan is extended past it.
export default function SarDisplay({ sarResult, sarProgress, scaleMode, dynRange, viewMode, colormap }) {
  const combined = viewMode === 'combined';
  return (
    <div className="flex flex-col w-full h-full">
      {sarProgress !== null && (
        <div className="absolute top-0 left-0 right-0 z-10 h-0.5">
          <div
            className="h-full bg-gradient-to-r from-emerald-500 to-emerald-300 transition-all duration-200"
            style={{ width: `${sarProgress * 100}%` }}
          />
        </div>
      )}
      {combined ? (
        <Pane sarResult={sarResult} mode="combined" scaleMode={scaleMode} dynRange={dynRange} colormap={colormap} />
      ) : (
        <>
          <Pane sarResult={sarResult} mode="amplitude" scaleMode={scaleMode} dynRange={dynRange} colormap={colormap} />
          <div className="h-px bg-white/8 shrink-0" />
          <Pane sarResult={sarResult} mode="coherence" scaleMode={scaleMode} dynRange={dynRange} colormap={colormap} />
        </>
      )}
    </div>
  );
}
