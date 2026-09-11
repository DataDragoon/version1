import { useRef, useEffect, useState } from 'react';
import { analyzeCoverage } from '@/lib/bgCaptureStats';

// Coverage view. Positions are static now, so what matters while capturing is
// where the gaps are — the largest one is flagged so the next capture can fill
// it. Lower lane shows post-averaging SNR per position.
export default function BgModelDisplay({ captures, capturing, sfcwProgress, stopFreq }) {
  const canvasRef = useRef(null);
  const containerRef = useRef(null);
  const [size, setSize] = useState({ w: 0, h: 0 });

  useEffect(() => {
    const el = containerRef.current;
    if (!el) return;
    const ro = new ResizeObserver(([entry]) => {
      const { width, height } = entry.contentRect;
      setSize({ w: width, h: height });
    });
    ro.observe(el);
    return () => ro.disconnect();
  }, []);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas || size.w === 0) return;
    const ctx = canvas.getContext('2d');
    const dpr = window.devicePixelRatio || 1;
    canvas.width = size.w * dpr;
    canvas.height = size.h * dpr;
    ctx.scale(dpr, dpr);
    const w = size.w, h = size.h;

    ctx.fillStyle = '#000';
    ctx.fillRect(0, 0, w, h);
    if (!captures || captures.length === 0) return;

    const cov = analyzeCoverage(captures, stopFreq);
    if (cov.positions.length === 0) return;
    const { limits } = cov;

    const pad = { top: 44, bottom: 50, left: 60, right: 30 };
    const plotW = w - pad.left - pad.right;
    const plotH = h - pad.top - pad.bottom;
    const gapLaneY = pad.top + plotH * 0.18;
    const snrTop = pad.top + plotH * 0.38;
    const snrH = plotH * 0.62;

    const lo = cov.positions[0].mm;
    const hi = cov.positions[cov.positions.length - 1].mm;
    const margin = (hi - lo) * 0.05 || 5;
    const minD = lo - margin, maxD = hi + margin;
    const rangeD = maxD - minD || 1;
    const xOf = (mm) => pad.left + ((mm - minD) / rangeD) * plotW;

    // Vertical grid + axis
    ctx.strokeStyle = 'rgba(255,255,255,0.05)';
    ctx.lineWidth = 1;
    ctx.fillStyle = '#555';
    ctx.font = '10px monospace';
    ctx.textAlign = 'center';
    for (let i = 0; i <= 5; i++) {
      const x = pad.left + (i / 5) * plotW;
      ctx.beginPath();
      ctx.moveTo(x, pad.top);
      ctx.lineTo(x, pad.top + plotH);
      ctx.stroke();
      ctx.fillText((minD + (i / 5) * rangeD).toFixed(0), x, pad.top + plotH + 20);
    }
    ctx.fillText('Standoff (mm)', pad.left + plotW / 2, pad.top + plotH + 40);

    // Gap lane — one segment between consecutive positions, colored by severity
    for (const g of cov.gaps) {
      const x0 = xOf(g.lo), x1 = xOf(g.hi);
      const isWorst = g.mm === cov.maxGap;
      const color = g.mm <= limits.goodMm ? 'rgba(74,222,128,'
        : g.mm <= limits.aliasMm ? 'rgba(250,204,21,'
        : 'rgba(248,113,113,';
      ctx.fillStyle = color + (isWorst ? '0.42)' : '0.20)');
      ctx.fillRect(x0, gapLaneY - 7, Math.max(1, x1 - x0), 14);

      if (isWorst && x1 - x0 > 34) {
        ctx.fillStyle = '#fff';
        ctx.font = 'bold 9px monospace';
        ctx.fillText(`${g.mm.toFixed(1)}`, (x0 + x1) / 2, gapLaneY + 3);
      }
    }

    // SNR bars, one per position
    const snrs = cov.positions.map(p => (p.stats ? p.stats.snrDbAveraged : null));
    const valid = snrs.filter(v => v != null);
    const snrMax = valid.length ? Math.max(...valid) : 1;
    const snrMin = valid.length ? Math.min(...valid) : 0;
    const snrSpan = Math.max(6, snrMax - snrMin);

    cov.positions.forEach((p, i) => {
      const x = xOf(p.mm);
      // Position tick spanning both lanes
      ctx.strokeStyle = 'rgba(167,139,250,0.75)';
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.moveTo(x, gapLaneY - 12);
      ctx.lineTo(x, gapLaneY + 12);
      ctx.stroke();

      const snr = snrs[i];
      if (snr == null) return;
      const frac = (snr - (snrMin - snrSpan * 0.15)) / (snrSpan * 1.15);
      const barH = Math.max(2, frac * snrH);
      // null coherence means a single-sweep position, which cannot be scored at
      // all -- neutral grey, distinct from both a good bar and a bad one.
      const coh = p.stats.coherence;
      ctx.fillStyle = coh == null ? 'rgba(148,163,184,0.45)'
        : coh > 0.95 ? 'rgba(74,222,128,0.55)'
        : coh > 0.85 ? 'rgba(250,204,21,0.55)'
        : 'rgba(248,113,113,0.55)';
      ctx.fillRect(x - 2.5, snrTop + snrH - barH, 5, barH);
    });

    // SNR axis labels
    ctx.fillStyle = '#555';
    ctx.font = '9px monospace';
    ctx.textAlign = 'right';
    ctx.textBaseline = 'middle';
    ctx.fillText(`${snrMax.toFixed(0)} dB`, pad.left - 8, snrTop + snrH * 0.075);
    ctx.fillText(`${snrMin.toFixed(0)}`, pad.left - 8, snrTop + snrH);
    ctx.save();
    ctx.translate(16, snrTop + snrH / 2);
    ctx.rotate(-Math.PI / 2);
    ctx.textAlign = 'center';
    ctx.fillText('Averaged SNR', 0, 0);
    ctx.restore();

    // Header
    ctx.textAlign = 'left';
    ctx.textBaseline = 'top';
    ctx.fillStyle = '#888';
    ctx.font = 'bold 11px sans-serif';
    const totalSweeps = captures.reduce((n, c) => n + c.samples.length, 0);
    ctx.fillText(
      `${cov.positions.length} positions · ${totalSweeps} sweeps · ${cov.spanMm.toFixed(0)} mm span`,
      pad.left, 12,
    );
    if (cov.maxGap != null) {
      ctx.textAlign = 'right';
      ctx.fillStyle = cov.maxGap <= limits.goodMm ? '#4ade80'
        : cov.maxGap <= limits.aliasMm ? '#facc15' : '#f87171';
      ctx.font = '10px monospace';
      ctx.fillText(
        `max gap ${cov.maxGap.toFixed(1)} mm  (alias > ${limits.aliasMm.toFixed(1)})`,
        w - pad.right, 14,
      );
    }
  }, [captures, size, stopFreq]);

  return (
    <div ref={containerRef} className="absolute inset-0 flex flex-col">
      {capturing && sfcwProgress && (
        <div className="absolute top-2 right-4 z-10 flex items-center gap-2 px-3 py-1.5 rounded-lg bg-[#a78bfa]/10 border border-[#a78bfa]/30">
          <div className="w-2 h-2 rounded-full bg-[#a78bfa] animate-pulse" />
          <span className="text-[10px] font-medium text-[#a78bfa]">
            Capturing... {sfcwProgress.step}/{sfcwProgress.total}
          </span>
        </div>
      )}
      <canvas ref={canvasRef} className="w-full h-full" style={{ imageRendering: 'pixelated' }} />
    </div>
  );
}
