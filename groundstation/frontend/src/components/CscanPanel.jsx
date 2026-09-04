import { useState, useRef, useEffect, useCallback } from 'react';
import { cn } from '@/lib/utils';
import { Section, InfoTile } from './Sidebar';
import { orderedCellForIndex, gridStats, gridRoverExtent, BG_STATUS, BG_STATUS_TEXT } from '@/lib/cscanGrid';

const LIDAR_AVG_WINDOW = 20;

// What the automation is doing right now, in the operator's terms.
const PHASE_TEXT = {
  homing: 'Driving to the grid origin',
  ready: 'Parked at origin — ready to scan',
  moving: 'Moving to the next cell',
  settling: 'Settling',
  capturing: 'Sweeping',
};

export default function CscanPanel({
  isConnected, sdrConnected, sfcwRunning, scanData, scanCapturing, bgApplied, onBgAppliedChange,
  onScanAction, params, onParamsChange, scaleMode, onScaleModeChange, displayMode, onDisplayModeChange,
  scaleRange, onScaleRangeChange, lidarMm, lidarOffsetMm, bgRef, bgModel, bgCapturing,
  onCaptureBg, onLoadBgModel, onClearBg,
  bgSubMode, onBgSubModeChange,
  superFit, onCaptureSuperFit, onClearSuperFit,
  sharedScale, bgDiag, procParams, captureProgress,
  scaleScope, onScaleScopeChange, rowScales, showGate, onShowGateChange,
  scaleLink, onScaleLinkChange, gridScales,
  roverConnected, roverStatus, sendRover, roverScan,
}) {
  const {
    hStep, hCount, vStep, vCount, gateStart, gateEnd, metric,
    scanMode, roverOriginRightMm, roverOriginBelowMm, roverSettleMs,
  } = params;

  const update = (key, value) => {
    onParamsChange({ ...params, [key]: value });
  };

  const lidarBuf = useRef([]);
  const [lidarAvg, setLidarAvg] = useState(null);
  const [modelList, setModelList] = useState(null);
  const [modelListOpen, setModelListOpen] = useState(false);

  useEffect(() => {
    if (lidarMm == null) return;
    const buf = lidarBuf.current;
    buf.push(lidarMm);
    if (buf.length > LIDAR_AVG_WINDOW) buf.shift();
    const avg = buf.reduce((s, v) => s + v, 0) / buf.length;
    setLidarAvg(avg);
  }, [lidarMm]);

  const fetchModels = useCallback(() => {
    fetch('/api/models')
      .then(r => r.json())
      .then(data => { setModelList(Array.isArray(data) ? data : []); setModelListOpen(true); })
      .catch(() => setModelList([]));
  }, []);

  const loadModel = useCallback((filename) => {
    fetch(`/api/models/${filename}`)
      .then(r => r.json())
      .then(model => { onLoadBgModel(model); setModelListOpen(false); })
      .catch(err => console.error('Failed to load model:', err));
  }, [onLoadBgModel]);

  // Both the model and the captured reference live in standoff, so compare the
  // live lidar on the same basis the sweeps are tagged with.
  const standoffNow = lidarAvg != null ? lidarAvg - (lidarOffsetMm || 0) : null;

  // Reference: how far the current standoff has drifted from where it was taken.
  const deltaMm = (bgRef && bgRef.lidar_standoff_mm != null && standoffNow != null)
    ? standoffNow - bgRef.lidar_standoff_mm : null;
  const deltaOk = deltaMm != null && Math.abs(deltaMm) <= 5;

  // Model: Akima inference clamps outside the captured span, so flag it.
  const modelSpan = (bgModel && Array.isArray(bgModel.d) && bgModel.d.length > 1)
    ? { min: bgModel.d[0], max: bgModel.d[bgModel.d.length - 1] } : null;
  const outOfSpan = modelSpan != null && standoffNow != null
    && (standoffNow < modelSpan.min || standoffNow > modelSpan.max);

  const canActivate = isConnected && sdrConnected;
  const captured = scanData.length;
  const stats = gridStats(params);
  const gridFull = captured >= stats.total;

  // ── Rover mode ────────────────────────────────────────────────────────
  const roverMode = scanMode === 'rover';
  const roverLinked = roverConnected && !!roverStatus?.board_connected;
  const roverEstopped = !!roverStatus?.estop;
  const scanning = !!roverScan?.active;
  // Parked at the origin with the sweep running, waiting for the operator to
  // start the raster. This is the window in which a background reference can be
  // captured at a known position before anything moves.
  const armed = scanning && roverScan.phase === 'ready';
  // Actually rastering, as opposed to armed or homing.
  const rastering = scanning && !armed && roverScan.phase !== 'homing';
  // In rover mode the session is the raster, so the button tracks the
  // automation rather than the bare sweep.
  const sessionActive = roverMode ? (scanning || sfcwRunning) : sfcwRunning;

  // The rover position the grid's top-left corner sits at, given where the
  // operator says the head is standing relative to it. Shown before the scan
  // starts so a wrong entry is visible against the soft limits, not discovered
  // by driving into the end of a rail that has no endstop.
  const originPreview = (roverMode && roverStatus)
    ? {
        x: roverStatus.x_mm - (Number(roverOriginRightMm) || 0),
        y: roverStatus.y_mm + (Number(roverOriginBelowMm) || 0),
      }
    : null;
  const extent = originPreview ? gridRoverExtent(params, originPreview) : null;
  const cfg = roverStatus?.config;
  const fitsLimits = !(extent && cfg && cfg.limits_enabled) || (
    extent.xMin >= cfg.x_min_mm && extent.xMax <= cfg.x_max_mm
    && extent.yMin >= cfg.y_min_mm && extent.yMax <= cfg.y_max_mm
  );

  // Where the next capture lands. The two modes walk the same grid in opposite
  // orders — bottom-left upwards by hand, top-left downwards under the rover.
  const nextIndex = scanning ? roverScan.index : captured;
  const next = nextIndex >= stats.total ? null : orderedCellForIndex(nextIndex, hCount, vCount, scanMode);
  const nextLabel = next
    ? `col ${next.ix + 1}/${hCount}, row ${next.iy + 1}/${vCount}  ·  (${(next.ix * hStep).toFixed(1)}, ${(next.iy * vStep).toFixed(1)}) cm`
    : 'Grid complete';
  const rowDir = next
    ? ((roverMode ? (vCount - 1 - next.iy) : next.iy) % 2 === 0 ? 'left → right' : 'right → left')
    : null;

  // Handing over from dynamic to manual should not jump the colours, so the
  // sliders start wherever the dynamic limits currently sit.
  //
  // This used to rebuild the grid from `scanData` -- which is the RAW capture
  // list, before background subtraction and computed by the Pi with a Hanning
  // window at nfft 204, where the display uses a rectangular window at nfft 256.
  // The seeded limits were therefore wrong by the full suppression (tens of dB)
  // plus ~4 dB of window/nfft difference, and the colours jumped hard on every
  // switch to manual, which is the exact thing the comment above promises they
  // do not. It now reads the same shared scale the displays draw with.
  const seedManualRange = () => {
    if (!sharedScale || !isFinite(sharedScale.min) || !isFinite(sharedScale.max)) {
      return { min: scaleRange.min, max: scaleRange.max };
    }
    const lo = Math.floor(sharedScale.min);
    return { min: lo, max: Math.max(Math.ceil(sharedScale.max), lo + 1) };
  };

  // Grid geometry is locked while a Super Fit is loaded: the reference is keyed
  // by (ix, iy), so changing the counts or steps would silently re-key every
  // cell and subtract each new capture against the wrong patch of wall.
  const gridLocked = !!superFit;
  const superFitGridMatches = !superFit || (
    superFit.grid.hCount === hCount && superFit.grid.vCount === vCount
    && superFit.grid.hStep === hStep && superFit.grid.vStep === vStep
  );
  // How deep the record actually goes -- c/(2*step)/2 - range_offset, read off a
  // captured profile rather than configured. This is what bounds the gate; there
  // is no separate Max Depth to keep in sync with it any more.
  const depthLimitCm = (() => {
    const p0 = scanData.find(p => p && p.distances && p.distances.length);
    if (!p0) return 100;
    return Math.ceil(p0.distances[p0.distances.length - 1] * 100);
  })();

  // How far apart the per-row limits are: the number that says whether per-row
  // scaling is buying anything. A big spread means one row was setting the
  // global limits for all of them.
  const unlinked = scaleLink === 'independent';
  // What the PLAN VIEW is actually drawn from, so the tiles below describe the
  // image rather than the population it was linked to before the toggle.
  const gridGlobal = unlinked && gridScales ? gridScales.global : sharedScale;
  const gridRows = unlinked && gridScales ? gridScales.rows : rowScales;

  const rowSpreadLabel = (() => {
    const rowScales = gridRows;
    if (!rowScales || rowScales.size === 0) return '—';
    let lo = Infinity;
    let hi = -Infinity;
    for (const r of rowScales.values()) {
      if (!isFinite(r.max)) continue;
      if (r.max < lo) lo = r.max;
      if (r.max > hi) hi = r.max;
    }
    if (!isFinite(lo) || !isFinite(hi)) return '—';
    return `${(hi - lo).toFixed(1)} dB`;
  })();

  const isDiff = bgSubMode === 'magnitude';
  const scaleSliderMin = isDiff ? -40 : -140;
  const scaleSliderMax = isDiff ? 40 : 0;

  const startDisabled = !canActivate || (roverMode && (!roverLinked || roverEstopped || !fitsLimits));

  return (
    <>
      {/* Who drives the raster. Only the capture ORDER differs between the two —
          the sweep, the standoff provenance and the background subtraction are
          identical, so a grid captured either way is the same record. */}
      <Section label="Scan Mode">
        <div className="flex gap-2">
          {[
            { id: 'manual', label: 'Manual', hint: 'Place the head by hand' },
            { id: 'rover', label: 'Rover', hint: 'Gantry rasters the grid' },
          ].map((m) => {
            const disabled = (m.id === 'rover' && !roverLinked) || scanning;
            return (
              <button
                key={m.id}
                onClick={() => !disabled && update('scanMode', m.id)}
                disabled={disabled}
                className={cn(
                  'flex-1 flex flex-col gap-0.5 px-3 py-2.5 rounded-lg border text-left transition-all',
                  disabled ? 'bg-white/2 border-white/5 text-white/20 cursor-not-allowed'
                    : scanMode === m.id
                      ? 'bg-[#4aff8a]/10 border-[#4aff8a]/30 text-[#4aff8a] cursor-pointer'
                      : 'bg-white/5 border-white/10 text-white/50 hover:text-white/80 cursor-pointer',
                )}
              >
                <span className="text-xs font-semibold">{m.label}</span>
                <span className="text-[9px] leading-tight opacity-70">
                  {m.id === 'rover' && !roverLinked ? 'Controller not connected' : m.hint}
                </span>
              </button>
            );
          })}
        </div>
      </Section>

      {/* Scan grid — describes the rectangle to raster before any capture starts */}
      <Section label="Scan Grid">
        {gridLocked && (
          <div className="px-2 py-1.5 rounded-lg bg-[#a78bfa]/5 border border-[#a78bfa]/30 text-[9px] text-[#a78bfa] leading-relaxed">
            Locked by Super Fit — the reference is keyed by cell, so the geometry has
            to match the grid it was captured on. Clear Super Fit to edit.
          </div>
        )}
        <div className="px-1 text-[9px] font-medium uppercase tracking-wider text-[#555555]">Horizontal</div>
        <div className="grid grid-cols-2 gap-2">
          <EditableField
            label="H Positions"
            value={hCount}
            unit="ct"
            onChange={(v) => update('hCount', Math.round(v))}
            min={1}
            max={200}
            locked={gridLocked}
          />
          <EditableField
            label="H Step"
            value={hStep}
            unit="cm"
            onChange={(v) => update('hStep', v)}
            min={0.5}
            max={50}
            locked={gridLocked}
          />
        </div>
        <div className="px-1 pt-1 text-[9px] font-medium uppercase tracking-wider text-[#555555]">Vertical</div>
        <div className="grid grid-cols-2 gap-2">
          <EditableField
            label="V Positions"
            value={vCount}
            unit="ct"
            onChange={(v) => update('vCount', Math.round(v))}
            min={1}
            max={200}
            locked={gridLocked}
          />
          <EditableField
            label="V Step"
            value={vStep}
            unit="cm"
            onChange={(v) => update('vStep', v)}
            min={0.5}
            max={50}
            locked={gridLocked}
          />
        </div>

        {/* Sweep rectangle stats */}
        <div className="grid grid-cols-2 gap-2 pt-1">
          <InfoTile label="Sweep W × H" value={`${stats.width.toFixed(1)} × ${stats.height.toFixed(1)} cm`} />
          <InfoTile label="Area" value={`${stats.area.toFixed(0)} cm²`} />
        </div>
        <div className="grid grid-cols-2 gap-2">
          <InfoTile label="Total Cells" value={`${stats.total}`} />
          <InfoTile label="Captured" value={`${captured} / ${stats.total}`} />
        </div>

        <div className="px-2 py-1.5 rounded-lg bg-[#0a0a0a]/60 border border-white/5 text-[9px] text-white/40 leading-relaxed">
          {roverMode
            ? 'Origin is the top-left cell. The rover sweeps the top row left → right, drops one row, sweeps back, and snakes down.'
            : 'Raster starts bottom-left and snakes: row 1 left → right, row 2 right → left, and so on.'}
        </div>

        {/* Where the head is standing relative to the grid origin. The rover
            drives left and up by exactly this to reach the origin, in one move
            on both axes, before the raster starts. */}
        {roverMode && (
          <>
            <div className="px-1 pt-2 text-[9px] font-medium uppercase tracking-wider text-[#555555]">
              Current position from origin
            </div>
            <div className="grid grid-cols-2 gap-2">
              <EditableField
                label="Right of origin"
                value={roverOriginRightMm}
                unit="mm"
                onChange={(v) => update('roverOriginRightMm', v)}
                min={-100000}
                max={100000}
              />
              <EditableField
                label="Below origin"
                value={roverOriginBelowMm}
                unit="mm"
                onChange={(v) => update('roverOriginBelowMm', v)}
                min={-100000}
                max={100000}
              />
            </div>
            <EditableField
              label="Settle before sweep"
              value={roverSettleMs}
              unit="ms"
              onChange={(v) => update('roverSettleMs', Math.round(v))}
              min={0}
              max={10000}
            />

            {originPreview && (
              <div className="grid grid-cols-2 gap-2">
                <InfoTile label="Origin at" value={`${originPreview.x.toFixed(0)}, ${originPreview.y.toFixed(0)} mm`} />
                <InfoTile
                  label="Rover span"
                  value={extent ? `${(extent.xMax - extent.xMin).toFixed(0)} × ${(extent.yMax - extent.yMin).toFixed(0)} mm` : '—'}
                />
              </div>
            )}
            {extent && (
              <div className={cn(
                'px-2 py-1.5 rounded-lg border text-[9px] leading-relaxed',
                fitsLimits
                  ? 'bg-[#0a0a0a]/60 border-white/5 text-white/40'
                  : 'bg-red-500/5 border-red-500/30 text-red-400',
              )}>
                Rover travels X {extent.xMin.toFixed(0)} → {extent.xMax.toFixed(0)} mm,
                {' '}Y {extent.yMax.toFixed(0)} → {extent.yMin.toFixed(0)} mm.
                {!fitsLimits && ' That is outside the soft limits — there are no endstops, so the scan is refused rather than clamped.'}
              </div>
            )}
            <div className="px-2 text-[9px] text-white/40 leading-relaxed">
              Measure where the head is now relative to the grid's top-left corner and
              enter it here. Nothing else knows where the grid is — this is what the
              rover drives back to before the first cell. Negative values are fine if
              the head is left of, or above, the origin.
            </div>
          </>
        )}
      </Section>

      {/* Session control — a continuous sweep by hand, the whole raster by rover */}
      <Section label="Session">
        <button
          onClick={() => onScanAction(sessionActive ? 'stop_session' : 'start_session')}
          disabled={sessionActive ? false : startDisabled}
          className={cn(
            'group relative flex items-center gap-3 w-full p-4 rounded-2xl border',
            'transition-all duration-500 cursor-pointer',
            'disabled:cursor-not-allowed disabled:opacity-40',
            sessionActive
              ? roverMode
                ? 'bg-red-500/8 border-red-500/30 hover:border-red-500/50'
                : 'bg-orange-500/8 border-orange-500/30 hover:border-orange-500/50'
              : !startDisabled
                ? 'bg-[#6B9BD2]/8 border-[#6B9BD2]/30 hover:border-[#6B9BD2]/50'
                : 'bg-[#0a0a0a]/50 border-white/5',
          )}
        >
          <div className={cn(
            'flex items-center justify-center w-10 h-10 rounded-xl shrink-0 transition-all duration-500',
            sessionActive ? (roverMode ? 'bg-red-500/15' : 'bg-orange-500/15')
              : !startDisabled ? 'bg-[#6B9BD2]/15' : 'bg-white/5',
          )}>
            {sessionActive ? (
              <div className={cn('w-3 h-3 rounded-sm', roverMode ? 'bg-red-400' : 'bg-orange-400')} />
            ) : (
              <div className="w-3 h-3 rounded-full border-2 border-current text-[#6B9BD2]" />
            )}
          </div>
          <div className="flex flex-col gap-0.5 text-left min-w-0">
            <span className="text-sm font-semibold text-white">
              {sessionActive
                ? (roverMode ? 'Stop Session · E-Stop' : 'Stop Session')
                : 'Start Session'}
            </span>
            <span className="text-xs text-[#555555] leading-relaxed">
              {sessionActive
                ? (roverMode
                    ? (scanning
                        ? (armed
                            ? PHASE_TEXT.ready
                            : `${PHASE_TEXT[roverScan.phase] || 'Scanning'} — cell ${roverScan.index + 1}/${roverScan.total}`)
                        : 'Sweeping — stop latches the E-stop')
                    : 'Sweeping continuously...')
                : !sdrConnected ? 'SDR not connected'
                : roverMode
                  ? !roverLinked ? 'Rover controller not connected'
                    : roverEstopped ? 'E-stop latched — clear it first'
                    : !fitsLimits ? 'Grid does not fit inside the soft limits'
                    : gridFull ? 'Grid full — start a new scan'
                    : 'Sweep and drive to the grid origin — the raster starts separately'
                  : 'Start continuous sweep'}
            </span>
          </div>
        </button>

        {/* Second half of the start. Arming parks the head on the grid origin
            with the sweep already running, which is the only moment where a
            background reference can be taken at a known position before the
            gantry moves. Pressing this begins the raster. */}
        {roverMode && armed && (
          <>
            <button
              onClick={() => onScanAction('start_raster')}
              disabled={gridFull}
              className={cn(
                'group relative flex items-center gap-3 w-full p-4 rounded-2xl border',
                'transition-all duration-500 cursor-pointer',
                'disabled:cursor-not-allowed disabled:opacity-40',
                gridFull
                  ? 'bg-[#0a0a0a]/50 border-white/5'
                  : 'bg-[#4aff8a]/8 border-[#4aff8a]/30 hover:border-[#4aff8a]/50',
              )}
            >
              <div className={cn(
                'flex items-center justify-center w-10 h-10 rounded-xl shrink-0',
                gridFull ? 'bg-white/5' : 'bg-[#4aff8a]/15',
              )}>
                <svg className="w-4 h-4 text-[#4aff8a]" fill="none" viewBox="0 0 24 24" stroke="currentColor" strokeWidth={2}>
                  <path strokeLinecap="round" strokeLinejoin="round" d="M14 5l7 7m0 0l-7 7m7-7H3" />
                </svg>
              </div>
              <div className="flex flex-col gap-0.5 text-left min-w-0">
                <span className="text-sm font-semibold text-white">Start Scan</span>
                <span className="text-xs text-[#555555] leading-relaxed">
                  {gridFull
                    ? 'Grid full — start a new scan'
                    : `Raster ${stats.total - captured} cell${stats.total - captured === 1 ? '' : 's'} automatically`}
                </span>
              </div>
            </button>
            <div className="px-2 text-[9px] text-white/40 leading-relaxed">
              The head is parked on the grid origin and sweeping. Capture a
              background reference now if you want one — a reference is only
              valid near the position and standoff it was taken at, so taking it
              here, rather than mid-raster, is the point of this pause.
            </div>
          </>
        )}

        {/* Progress along the raster, and whatever ended it. */}
        {roverMode && rastering && (
          <>
            <div className="relative h-1.5 rounded-full bg-white/5 overflow-hidden">
              <div
                className="absolute inset-y-0 left-0 rounded-full bg-gradient-to-r from-[#4aff8a] to-[#22d3ee] transition-all duration-300"
                style={{ width: `${Math.min(100, (roverScan.index / Math.max(1, roverScan.total)) * 100)}%` }}
              />
            </div>
            {roverScan.target && (
              <div className="flex items-center justify-between px-3 py-1.5 rounded-lg bg-[#4aff8a]/5 border border-[#4aff8a]/20">
                <span className="text-[9px] font-medium uppercase tracking-wider text-[#555555]">Target</span>
                <span className="text-[10px] font-mono text-[#4aff8a]">
                  {roverScan.target.x_mm.toFixed(1)}, {roverScan.target.y_mm.toFixed(1)} mm
                </span>
              </div>
            )}
          </>
        )}
        {roverMode && !scanning && roverScan?.error && (
          <div className="flex items-start gap-2 p-3 rounded-xl border border-red-500/30 bg-red-500/5">
            <span className="text-[10px] leading-relaxed text-red-400">{roverScan.error}</span>
          </div>
        )}
        {roverMode && !scanning && !roverScan?.error && roverScan?.message && (
          <div className="px-2 text-[10px] leading-relaxed text-[#4aff8a]/70">{roverScan.message}</div>
        )}
        {roverMode && roverEstopped && (
          <button
            onClick={() => sendRover?.({ cmd: 'rover_clear_estop' })}
            className="w-full px-3 py-2 rounded-lg text-xs font-semibold border border-amber-500/40 bg-amber-500/10 text-amber-400 hover:bg-amber-500/15 transition-all"
          >
            Clear E-Stop
          </button>
        )}
        {roverMode && roverEstopped && (
          <div className="px-2 text-[9px] text-amber-400/60 leading-relaxed">
            Cutting the step train at speed is where a stepper loses steps, so the
            position is no longer trustworthy — re-declare it in the Rover panel
            before scanning again.
          </div>
        )}
      </Section>

      {/* Capture controls — by hand when the operator drives, automatic otherwise */}
      <Section label="Capture">
        {roverMode ? (
          <div className={cn(
            'flex items-center gap-3 w-full p-4 rounded-2xl border',
            scanning ? 'bg-[#4aff8a]/8 border-[#4aff8a]/30' : 'bg-[#0a0a0a]/50 border-white/5',
          )}>
            <div className={cn(
              'flex items-center justify-center w-10 h-10 rounded-xl shrink-0',
              scanning ? 'bg-[#4aff8a]/15' : 'bg-white/5',
            )}>
              {scanning ? (
                <div className="w-3 h-3 rounded-full border-2 border-[#4aff8a] border-t-transparent animate-spin" />
              ) : (
                <div className="w-3 h-3 rounded-full border-2 border-current text-[#555555]" />
              )}
            </div>
            <div className="flex flex-col gap-0.5 text-left min-w-0">
              <span className="text-sm font-semibold text-white">
                {scanning ? (PHASE_TEXT[roverScan.phase] || 'Scanning')
                  : gridFull ? 'Grid Complete' : 'Captured by the rover'}
              </span>
              <span className="text-xs text-[#555555] leading-relaxed">
                {scanning ? nextLabel
                  : gridFull ? `All ${stats.total} cells captured`
                  : `${captured} of ${stats.total} cells — start the scan to fill the rest`}
              </span>
            </div>
          </div>
        ) : (
        <button
          onClick={() => onScanAction('add_scan')}
          disabled={!sfcwRunning || scanCapturing || gridFull}
          className={cn(
            'group relative flex items-center gap-3 w-full p-4 rounded-2xl border',
            'transition-all duration-500 cursor-pointer',
            'disabled:cursor-not-allowed disabled:opacity-40',
            sfcwRunning && !scanCapturing && !gridFull
              ? 'bg-[#22d3ee]/8 border-[#22d3ee]/30 hover:border-[#22d3ee]/50'
              : 'bg-[#0a0a0a]/50 border-white/5',
          )}
        >
          <div className={cn(
            'flex items-center justify-center w-10 h-10 rounded-xl shrink-0 transition-all duration-500',
            sfcwRunning && !scanCapturing && !gridFull ? 'bg-[#22d3ee]/15' : 'bg-white/5',
          )}>
            {scanCapturing ? (
              <div className="w-3 h-3 rounded-full border-2 border-[#22d3ee] border-t-transparent animate-spin" />
            ) : (
              <svg className="w-4 h-4 text-[#22d3ee]" fill="none" viewBox="0 0 24 24" stroke="currentColor" strokeWidth={2}>
                <path strokeLinecap="round" strokeLinejoin="round" d="M12 4v16m8-8H4" />
              </svg>
            )}
          </div>
          <div className="flex flex-col gap-0.5 text-left min-w-0">
            <span className="text-sm font-semibold text-white">
              {scanCapturing ? 'Capturing...' : gridFull ? 'Grid Complete' : `Capture Cell ${captured + 1}`}
            </span>
            <span className="text-xs text-[#555555] leading-relaxed">
              {scanCapturing ? 'Waiting for next sweep' :
               gridFull ? `All ${stats.total} cells captured` :
               !sfcwRunning ? 'Start session first' : nextLabel}
            </span>
          </div>
        </button>
        )}

        {!gridFull && rowDir && (
          <div className="flex items-center justify-between px-3 py-1.5 rounded-lg bg-[#22d3ee]/5 border border-[#22d3ee]/20">
            <span className="text-[9px] font-medium uppercase tracking-wider text-[#555555]">Row {next.iy + 1} sweeps</span>
            <span className="text-[10px] font-mono text-[#22d3ee]">{rowDir}</span>
          </div>
        )}

        {captured > 0 && (
          <div className="relative h-1.5 rounded-full bg-white/5 overflow-hidden">
            <div
              className="absolute inset-y-0 left-0 rounded-full bg-gradient-to-r from-[#22d3ee] to-[#67e8f9] transition-all duration-300"
              style={{ width: `${Math.min(100, (captured / Math.max(1, stats.total)) * 100)}%` }}
            />
          </div>
        )}

        <div className="grid grid-cols-2 gap-2">
          <button
            onClick={() => onScanAction('new')}
            disabled={scanning}
            className={cn(
              'px-3 py-2 rounded-lg text-xs font-medium transition-all',
              scanning
                ? 'bg-white/2 border border-white/5 text-white/20 cursor-not-allowed'
                : 'bg-white/5 border border-white/10 text-white/70 hover:bg-white/10 hover:text-white',
            )}
          >
            New Scan
          </button>
          <button
            onClick={() => onScanAction('undo')}
            disabled={captured === 0 || scanning}
            className={cn(
              'px-3 py-2 rounded-lg text-xs font-medium transition-all',
              captured > 0 && !scanning
                ? 'bg-white/5 border border-white/10 text-white/70 hover:bg-white/10 hover:text-white'
                : 'bg-white/2 border border-white/5 text-white/20 cursor-not-allowed'
            )}
          >
            Undo Last
          </button>
        </div>

        {captured > 0 && scanData[captured - 1].lidar_standoff_mm != null && (
          <div className="px-2 text-[9px] text-white/40 leading-relaxed">
            Last cell standoff: {scanData[captured - 1].lidar_standoff_mm.toFixed(1)} mm
          </div>
        )}
      </Section>

      <Section label="Standoff">
        <div className="flex flex-col gap-2">
          <div className="flex items-baseline justify-between px-3 py-2 rounded-xl border border-white/8 bg-[#0a0a0a]/60">
            <span className="text-[10px] font-medium uppercase tracking-wider text-[#555555]">Standoff</span>
            <span className="text-base font-bold font-mono text-white">
              {standoffNow != null ? standoffNow.toFixed(1) : '—'} <span className="text-xs font-semibold text-[#888888]">mm</span>
            </span>
          </div>
          {deltaMm != null && (
            <div className={cn(
              'flex items-baseline justify-between px-3 py-2 rounded-xl border',
              deltaOk
                ? 'border-green-500/30 bg-green-500/5'
                : 'border-red-500/30 bg-red-500/5'
            )}>
              <span className="text-[10px] font-medium uppercase tracking-wider text-[#555555]">Delta vs BG</span>
              <span className={cn(
                'text-base font-bold font-mono',
                deltaOk ? 'text-green-400' : 'text-red-400'
              )}>
                {(deltaMm >= 0 ? '+' : '') + deltaMm.toFixed(1)} <span className="text-xs font-semibold text-[#888888]">mm</span>
              </span>
            </div>
          )}
          {modelSpan && (
            <div className={cn(
              'flex items-baseline justify-between px-3 py-2 rounded-xl border',
              outOfSpan ? 'border-red-500/30 bg-red-500/5' : 'border-white/8 bg-[#0a0a0a]/60'
            )}>
              <span className="text-[10px] font-medium uppercase tracking-wider text-[#555555]">Model span</span>
              <span className={cn('text-xs font-bold font-mono', outOfSpan ? 'text-red-400' : 'text-white')}>
                {modelSpan.min.toFixed(0)} – {modelSpan.max.toFixed(0)} mm
              </span>
            </div>
          )}
          {outOfSpan && (
            <div className="px-2 text-[9px] text-red-400/70 leading-relaxed">
              Standoff is outside the captured span — the model clamps to the nearest end.
            </div>
          )}
        </div>
      </Section>

      {/* Banking N sweeps into one cell is otherwise a silent pause. */}
      {captureProgress && captureProgress.need > 1 && (
        <Section label="Capturing Cell">
          <div className="flex items-center justify-between px-1">
            <span className="text-[10px] uppercase tracking-wider text-[#555555] font-medium">
              Sweep {captureProgress.got + 1} / {captureProgress.need}
            </span>
            <span className="text-[10px] font-mono text-[#22d3ee]">
              {Math.round((captureProgress.got / captureProgress.need) * 100)}%
            </span>
          </div>
          <div className="h-1 rounded-full bg-white/10 overflow-hidden">
            <div
              className="h-full bg-gradient-to-r from-[#22d3ee] to-[#67e8f9] transition-all duration-200"
              style={{ width: `${(captureProgress.got / captureProgress.need) * 100}%` }}
            />
          </div>
        </Section>
      )}

      {/* Depth slice — the C-scan collapses the depth axis over this gate */}
      <Section label="Depth Slice">
        <SliderRow
          label="Gate start"
          value={gateStart}
          unit="cm"
          min={0}
          max={Math.max(gateEnd - 0.5, 0.5)}
          step={0.5}
          accent="cyan"
          onChange={(v) => update('gateStart', v)}
        />
        <SliderRow
          label="Gate end"
          value={gateEnd}
          unit="cm"
          min={Math.min(gateStart + 0.5, gateEnd)}
          max={Math.max(depthLimitCm, gateEnd)}
          step={0.5}
          accent="cyan"
          onChange={(v) => update('gateEnd', v)}
        />
        <div className="flex gap-2">
          {['peak', 'energy', 'mean'].map((m) => (
            <button
              key={m}
              onClick={() => update('metric', m)}
              className={cn(
                'flex-1 px-2 py-2 rounded-lg text-xs font-medium capitalize transition-all border',
                metric === m
                  ? 'bg-[#22d3ee]/10 border-[#22d3ee]/30 text-[#22d3ee]'
                  : 'bg-white/5 border-white/10 text-white/50 hover:text-white/80'
              )}
            >
              {m}
            </button>
          ))}
        </div>

        {/* A placement aid, and nothing more. The gate always decides the plan
            view's cell values -- gatedIntensity() sums exactly the bins inside
            it -- whether or not the markers are drawn. */}
        <button
          onClick={() => onShowGateChange(!showGate)}
          className={cn(
            'w-full px-3 py-2 rounded-lg text-xs font-medium transition-all border',
            showGate
              ? 'bg-[#22d3ee]/10 border-[#22d3ee]/30 text-[#22d3ee]'
              : 'bg-white/5 border-white/10 text-white/50 hover:text-white/80',
          )}
        >
          {showGate ? '● Gate markers on B-scan' : 'Gate markers hidden'}
        </button>
        <div className="px-2 text-[9px] text-white/40 leading-relaxed">
          {showGate
            ? 'Two cyan lines on the B-scan mark the gate edges and everything outside them is dimmed — what stays bright is exactly the bins each plan-view cell is built from. Drag the sliders and watch it move.'
            : 'Markers hidden. The gate still decides every plan-view cell value — this toggle only stops drawing it.'}
        </div>
      </Section>

      <Section label="Display">
        <div className="flex gap-2">
          <button
            onClick={() => onScaleModeChange(scaleMode === 'linear' ? 'db' : 'linear')}
            className="flex-1 px-3 py-2 rounded-lg text-xs font-medium transition-all border bg-[#6B9BD2]/10 border-[#6B9BD2]/30 text-[#6B9BD2]"
          >
            {scaleMode === 'linear' ? 'Linear' : 'dB'}
          </button>
          <button
            onClick={() => onDisplayModeChange(displayMode === 'color' ? 'profile' : 'color')}
            className="flex-1 px-3 py-2 rounded-lg text-xs font-medium transition-all border bg-[#6B9BD2]/10 border-[#6B9BD2]/30 text-[#6B9BD2]"
          >
            {displayMode === 'color' ? 'Color' : 'Profile'}
          </button>
        </div>
        <div className="px-2 text-[9px] text-white/40 leading-relaxed">
          The B-scan pane draws the whole record — {depthLimitCm} cm at the current
          step size, gate or no gate. The Depth Slice gate chooses what the plan
          view colours by; on the right it only shades the excluded bins, and
          the data under the shading is still drawn and still readable.
        </div>

        {/* Mirrors the Live Sweep pane's controls bar, which is where they are
            set. Window and averaging MODE re-derive the whole grid on change;
            Avg is how many sweeps each cell takes and only applies to captures
            made after it is set. The bar locks while a session runs. */}
        {procParams && (
          <>
            <div className="grid grid-cols-2 gap-2">
              <InfoTile
                label="Window"
                value={procParams.windowType === 'kaiser'
                  ? `Kaiser β${procParams.kaiserBeta}`
                  : procParams.windowType === 'hanning' ? 'Hanning' : 'Rect'}
              />
              <InfoTile
                label="Avg / cell"
                value={procParams.avgCount > 1
                  ? `${procParams.avgCount}× ${procParams.avgMode === 'coherent' ? 'coh' : 'inc'}`
                  : 'Off'}
              />
            </div>
            <div className="px-2 text-[9px] text-white/40 leading-relaxed">
              Set on the Live Sweep pane's controls bar, and locked while a session
              runs. Window and coh/inc re-derive every stored cell immediately —
              each cell keeps all its sweeps, so the averaging mode stays a live
              choice. Avg only affects cells captured after it is changed.
            </div>
          </>
        )}
      </Section>

      {/* Colour scaling — dynamic tracks the data, manual pins both ends live */}
      <Section label="Scaling">
        <button
          onClick={() => onScaleRangeChange(scaleRange.dynamic
            ? { dynamic: false, ...seedManualRange() }
            : { ...scaleRange, dynamic: true })}
          className={cn(
            'w-full px-3 py-2.5 rounded-lg text-xs font-medium transition-all border',
            scaleRange.dynamic
              ? 'bg-[#6B9BD2]/10 border-[#6B9BD2]/30 text-[#6B9BD2]'
              : 'bg-[#f59e0b]/10 border-[#f59e0b]/30 text-[#f59e0b]'
          )}
        >
          {scaleRange.dynamic ? '● Dynamic Scaling' : 'Manual Scaling'}
        </button>
        <SliderRow
          label="Min"
          value={scaleRange.min}
          unit="dB"
          min={scaleSliderMin}
          max={scaleSliderMax}
          step={1}
          accent="amber"
          disabled={scaleRange.dynamic}
          onChange={(v) => onScaleRangeChange({ ...scaleRange, min: Math.min(v, scaleRange.max - 1) })}
        />
        <SliderRow
          label="Max"
          value={scaleRange.max}
          unit="dB"
          min={scaleSliderMin}
          max={scaleSliderMax}
          step={1}
          accent="amber"
          disabled={scaleRange.dynamic}
          onChange={(v) => onScaleRangeChange({ ...scaleRange, max: Math.max(v, scaleRange.min + 1) })}
        />
        {/* Do the two panes share one scale? Linked is the guarantee: a colour
            means one dB in the plan view and in the B-scan, because both are
            drawn from every bin of every valid cell. That population ignores
            the gate, so narrowing the gate onto a quiet depth leaves the cells
            in the bottom of a range still set by the wall and the grid goes
            dark. Unlinked scales the grid within its own GATED values, which
            is the only way to keep contrast there -- and gives up the
            guarantee, so both panes are labelled when it is on. */}
        <div className="flex gap-2">
          {[
            { id: 'linked', label: 'Linked', hint: 'One scale, both panes' },
            { id: 'independent', label: 'Grid: own scale', hint: 'Scales within the gate' },
          ].map((m) => (
            <button
              key={m.id}
              onClick={() => onScaleLinkChange(m.id)}
              disabled={!scaleRange.dynamic}
              className={cn(
                'flex-1 flex flex-col gap-0.5 px-3 py-2 rounded-lg border text-left transition-all',
                !scaleRange.dynamic
                  ? 'bg-white/5 border-white/10 text-white/25 cursor-not-allowed'
                  : scaleLink === m.id
                    ? (m.id === 'independent'
                      ? 'bg-[#f59e0b]/10 border-[#f59e0b]/30 text-[#f59e0b]'
                      : 'bg-[#6B9BD2]/10 border-[#6B9BD2]/30 text-[#6B9BD2]')
                    : 'bg-white/5 border-white/10 text-white/50 hover:text-white/80',
              )}
            >
              <span className="text-xs font-semibold">{m.label}</span>
              <span className="text-[9px] leading-tight opacity-70">{m.hint}</span>
            </button>
          ))}
        </div>
        {scaleRange.dynamic && unlinked && (
          <div className="px-2 py-1.5 rounded-lg bg-[#f59e0b]/5 border border-[#f59e0b]/30 text-[9px] text-[#f59e0b] leading-relaxed">
            The plan view is scaled to its own gated cell values — the same colour
            means a different dB in the two panes. Both are marked on screen
            (OWN SCALE · GATED on the grid, UNLINKED on the B-scan). Compare cells
            to cells here, not cells to bins.
          </div>
        )}

        {/* What population a DYNAMIC scale is drawn from. Global is the honest
            one -- a colour means one dB everywhere, so cells are comparable
            across the whole grid. Per-row gives that up to get contrast back:
            on a wall whose standoff varies row to row the loudest row otherwise
            sets the limits and crushes the rest. Neither is a better version of
            the other. Manual pinning overrides both, so it is disabled there. */}
        <div className="flex gap-2">
          {[
            { id: 'global', label: 'Global', hint: 'One scale, whole grid' },
            { id: 'row', label: 'Per row', hint: 'Each row scaled to itself' },
          ].map((m) => (
            <button
              key={m.id}
              onClick={() => onScaleScopeChange(m.id)}
              disabled={!scaleRange.dynamic}
              className={cn(
                'flex-1 flex flex-col gap-0.5 px-3 py-2 rounded-lg border text-left transition-all',
                !scaleRange.dynamic
                  ? 'bg-white/5 border-white/10 text-white/25 cursor-not-allowed'
                  : scaleScope === m.id
                    ? 'bg-[#6B9BD2]/10 border-[#6B9BD2]/30 text-[#6B9BD2]'
                    : 'bg-white/5 border-white/10 text-white/50 hover:text-white/80',
              )}
            >
              <span className="text-xs font-semibold">{m.label}</span>
              <span className="text-[9px] leading-tight opacity-70">{m.hint}</span>
            </button>
          ))}
        </div>
        {gridGlobal && scaleRange.dynamic && scaleScope !== 'row' && (
          <div className="grid grid-cols-2 gap-2">
            <InfoTile label={unlinked ? 'Grid low' : 'Scale low'} value={`${gridGlobal.min.toFixed(1)} dB`} />
            <InfoTile label={unlinked ? 'Grid high' : 'Scale high'} value={`${gridGlobal.max.toFixed(1)} dB`} />
          </div>
        )}
        {scaleRange.dynamic && scaleScope === 'row' && (
          <div className="grid grid-cols-2 gap-2">
            <InfoTile label="Rows scaled" value={gridRows ? `${gridRows.size}` : '—'} />
            <InfoTile
              label="Row spread"
              value={rowSpreadLabel}
            />
          </div>
        )}
        <div className="px-2 text-[9px] text-white/40 leading-relaxed">
          {!scaleRange.dynamic
            ? 'Colour limits pinned — both the C-scan and B-scan panes update live.'
            : unlinked
              ? (scaleScope === 'row'
                ? 'Each grid row is scaled to its own gated cell values, and the B-scan keeps the bin-domain scale. Contrast where you need it; no colour agrees with any other pane or row.'
                : 'The grid is scaled to its own gated cell values, so it follows the Depth Slice gate. The B-scan keeps the bin-domain scale, so the two colour bars no longer agree.')
              : scaleScope === 'row'
                ? 'Each grid row gets its own limits, from every bin of that row (1st–99.9th percentile). Contrast within a row, but a colour no longer means the same dB in different rows — the colour bar shows the selected row and is marked PER ROW.'
                : 'One scale for both panes, from every bin of every valid cell in the grid (1st–99.9th percentile, so a single interference null cannot flatten the image). A colour means the same dB in the plan view and in the B-scan.'}
        </div>
        {gridGlobal && gridGlobal.degenerate && scaleRange.dynamic && (
          <div className="px-2 py-1.5 rounded-lg bg-[#f59e0b]/5 border border-[#f59e0b]/30 text-[9px] text-[#f59e0b] leading-relaxed">
            Every bin has the same value — there is nothing to scale. Expected right
            after a Super Fit capture, where the grid is being subtracted from itself.
          </div>
        )}
      </Section>

      {/* Background — mutually exclusive sources, plus how the subtraction is done */}
      <Section label="Background">
        {/* Complex vs magnitude. Not two views of one thing: complex is for
            SEEING (it removes the wall so a target 16.6 dB beneath it is not
            buried) and magnitude is for DECIDING (the 2026-08-28 A/B found the
            target as +4.4 dB against a 0.23 dB control, and it survives ~1 mm of
            standoff error, which the complex difference does not). */}
        <div className="flex gap-2">
          {[
            { id: 'complex', label: 'Complex', hint: 'Vector — removes the wall' },
            { id: 'magnitude', label: 'Magnitude', hint: 'Δ dB — the detector' },
          ].map((m) => (
            <button
              key={m.id}
              onClick={() => onBgSubModeChange(m.id)}
              className={cn(
                'flex-1 flex flex-col gap-0.5 px-3 py-2 rounded-lg border text-left transition-all',
                bgSubMode === m.id
                  ? 'bg-[#22d3ee]/10 border-[#22d3ee]/30 text-[#22d3ee]'
                  : 'bg-white/5 border-white/10 text-white/50 hover:text-white/80',
              )}
            >
              <span className="text-xs font-semibold">{m.label}</span>
              <span className="text-[9px] leading-tight opacity-70">{m.hint}</span>
            </button>
          ))}
        </div>

        <div className="grid grid-cols-2 gap-2">
          <button
            onClick={onCaptureBg}
            disabled={!sfcwRunning || bgCapturing}
            className={cn(
              'px-3 py-2.5 rounded-lg text-xs font-medium transition-all border',
              bgRef
                ? 'bg-[#f59e0b]/10 border-[#f59e0b]/30 text-[#f59e0b]'
                : sfcwRunning && !bgCapturing
                  ? 'bg-white/5 border-white/10 text-white/70 hover:bg-white/10 hover:text-white'
                  : 'bg-white/2 border-white/5 text-white/20 cursor-not-allowed'
            )}
          >
            {bgCapturing ? 'Capturing...' : bgRef ? 'BG Ref Active' : 'Capture BG'}
          </button>
          <button
            onClick={onClearBg}
            disabled={!bgRef && !bgModel && !bgCapturing}
            className={cn(
              'px-3 py-2.5 rounded-lg text-xs font-medium transition-all border',
              bgRef || bgModel || bgCapturing
                ? 'bg-white/5 border-white/10 text-white/70 hover:bg-white/10 hover:text-white'
                : 'bg-white/2 border-white/5 text-white/20 cursor-not-allowed'
            )}
          >
            Clear BG
          </button>
        </div>

        <button
          onClick={fetchModels}
          className={cn(
            'w-full px-3 py-2 rounded-lg text-xs font-medium transition-all border',
            bgModel
              ? 'bg-[#a78bfa]/10 border-[#a78bfa]/30 text-[#a78bfa]'
              : 'bg-white/5 border-white/10 text-white/70 hover:bg-white/10 hover:text-white'
          )}
        >
          {bgModel ? `Model: ${bgModel.name || 'loaded'}` : 'Load Model'}
        </button>
        {modelListOpen && modelList && (
          <div className="flex flex-col gap-1 max-h-32 overflow-y-auto rounded-lg border border-white/10 bg-[#0a0a0a] p-2">
            {modelList.length === 0 && (
              <span className="text-[10px] text-white/30 px-1">No models saved</span>
            )}
            {modelList.map((m) => (
              <button
                key={m.filename}
                onClick={() => loadModel(m.filename)}
                className="flex items-baseline justify-between gap-2 text-left px-2 py-1.5 rounded text-[11px] text-white/70 hover:bg-white/10 hover:text-white transition-all"
              >
                <span className="truncate">{m.name || m.filename}</span>
                {m.suppressionDb != null ? (
                  <span className={cn('font-mono shrink-0 text-[10px]',
                    m.suppressionDb > 15 ? 'text-green-400/70'
                    : m.suppressionDb > 8 ? 'text-yellow-400/70' : 'text-red-400/70')}>
                    {m.suppressionDb.toFixed(1)} dB
                  </span>
                ) : (
                  <span className="font-mono shrink-0 text-[10px] text-white/25">legacy</span>
                )}
              </button>
            ))}
            <button
              onClick={() => setModelListOpen(false)}
              className="text-[10px] text-white/30 hover:text-white/60 mt-1 px-1"
            >
              Cancel
            </button>
          </div>
        )}

        {(bgRef || bgModel || superFit) && (
          <button
            onClick={() => onBgAppliedChange(!bgApplied)}
            className={cn(
              'w-full px-3 py-2 rounded-lg text-xs font-medium transition-all border',
              bgApplied
                ? 'bg-[#6B9BD2]/10 border-[#6B9BD2]/30 text-[#6B9BD2]'
                : 'bg-white/5 border-white/10 text-white/70 hover:bg-white/10 hover:text-white'
            )}
          >
            {bgApplied ? '● BG Applied' : 'BG Not Applied'}
          </button>
        )}
        {/* Per-cell diagnostics. "A background is loaded" and "the background was
            applied to this cell" are different statements, and only the second
            one matters — a cell the background could not be resolved for is
            drawn as an explicit error, never given a colour. */}
        {bgDiag && bgDiag.total > 0 && (bgRef || bgModel || superFit) && (
          <>
            <div className="grid grid-cols-3 gap-2">
              <InfoTile label="Applied" value={`${bgDiag.applied}/${bgDiag.total}`} />
              <InfoTile label="Clamped" value={`${bgDiag.clamped}`} />
              <InfoTile label="Invalid" value={`${bgDiag.invalid}`} />
            </div>
            {bgDiag.invalid > 0 && (
              <div className="px-2 py-1.5 rounded-lg bg-red-500/5 border border-red-500/30 text-[9px] text-red-400 leading-relaxed">
                {bgDiag.invalid} cell{bgDiag.invalid === 1 ? '' : 's'} could not be subtracted and
                are marked with a red cross in the grid. They are excluded from the colour
                scale — an un-subtracted cell sits 20–30 dB above its neighbours and would
                otherwise read as the strongest target in the scan.
                {Object.entries(bgDiag.counts)
                  .filter(([k]) => k !== BG_STATUS.OK && k !== BG_STATUS.OFF && k !== BG_STATUS.CLAMPED)
                  .map(([k, n]) => ` · ${n}× ${BG_STATUS_TEXT[k] || k}`)}
              </div>
            )}
            {bgDiag.clamped > 0 && (
              <div className="px-2 py-1.5 rounded-lg bg-[#f59e0b]/5 border border-[#f59e0b]/30 text-[9px] text-[#f59e0b] leading-relaxed">
                {bgDiag.clamped} cell{bgDiag.clamped === 1 ? '' : 's'} sit outside the model's
                captured standoff span and were clamped to its nearest end (amber corner in
                the grid). Measured cost: 19 dB at 5 mm outside, and past ~10 mm the
                subtraction adds more energy than it removes.
              </div>
            )}
          </>
        )}
        <div className="px-2 text-[9px] text-white/40 leading-relaxed">
          {superFit
            ? 'Super Fit: each cell is subtracted from the reference captured at that same cell.'
            : bgModel
              ? 'Model background, inferred per cell from that cell’s own lidar standoff.'
              : bgRef
                ? 'Reference sweep, subtracted exactly as captured. It only holds near the standoff and position it was taken at — recapture if either moves. On a grid whose standoff varies, use Super Fit instead.'
                : 'Capture a reference sweep, load a model, or Super Fit a reference grid.'}
        </div>
      </Section>

      {/* Super Fit — a whole reference GRID, matched cell for cell.
          A single captured reference is only right at one position: on the rover
          scans of 2026-08-30 the wall return swung 6.3 dB across the grid because
          the rig is not parallel to the wall (17 mm of standoff over one
          700x150 mm grid), and a corner reference scored only 14-18 dB against
          that. Super Fit subtracts each cell from the reference taken at that
          same cell, so a standoff that varies across the grid is matched rather
          than extrapolated. */}
      <Section label="Super Fit">
        {!superFit ? (
          <>
            <button
              onClick={onCaptureSuperFit}
              disabled={!gridFull}
              className={cn(
                'w-full px-3 py-2.5 rounded-lg text-xs font-medium transition-all border',
                gridFull
                  ? 'bg-[#a78bfa]/10 border-[#a78bfa]/30 text-[#a78bfa] hover:bg-[#a78bfa]/20'
                  : 'bg-white/2 border-white/5 text-white/20 cursor-not-allowed',
              )}
            >
              Super Fit This Grid
            </button>
            <div className="px-2 text-[9px] text-white/40 leading-relaxed">
              {gridFull
                ? 'Stores every cell of the current grid as a per-cell background. Then clear the grid and rescan the same wall from the same origin — each new cell is subtracted from the reference at its own cell.'
                : `Needs a full grid — ${captured} of ${stats.total} cells captured. Scan or import a complete sweep of the bare wall first.`}
            </div>
          </>
        ) : (
          <>
            <div className="grid grid-cols-2 gap-2">
              <InfoTile label="Reference cells" value={`${superFit.count}`} />
              <InfoTile label="Grid" value={`${superFit.grid.hCount} × ${superFit.grid.vCount}`} />
            </div>
            <button
              onClick={onClearSuperFit}
              className="w-full px-3 py-2.5 rounded-lg text-xs font-medium transition-all border bg-[#a78bfa]/10 border-[#a78bfa]/30 text-[#a78bfa] hover:bg-[#a78bfa]/20"
            >
              Clear Super Fit
            </button>
            {!superFitGridMatches && (
              <div className="px-2 py-1.5 rounded-lg bg-red-500/5 border border-red-500/30 text-[9px] text-red-400 leading-relaxed">
                The grid no longer matches the one this Super Fit was captured on
                ({superFit.grid.hCount} × {superFit.grid.vCount} at {superFit.grid.hStep} × {superFit.grid.vStep} cm).
                Cells are matched by index, so the subtraction is against the wrong
                patch of wall. Clear Super Fit or restore the grid.
              </div>
            )}
            {captured > 0 && (
              <div className="px-2 py-1.5 rounded-lg bg-[#0a0a0a]/60 border border-white/5 text-[9px] text-white/40 leading-relaxed">
                {captured} cell{captured === 1 ? '' : 's'} still on screen. If this is the grid
                the reference was taken from it now subtracts from itself and reads as
                exactly zero everywhere — clear it below and rescan the same wall from the
                same origin.
              </div>
            )}
          </>
        )}
      </Section>

      <Section label="Data">
        <div className="grid grid-cols-2 gap-2">
          <button
            onClick={() => onScanAction('export')}
            disabled={scanData.length === 0}
            className={cn(
              'px-3 py-2 rounded-lg text-xs font-medium transition-all',
              scanData.length > 0
                ? 'bg-white/5 border border-white/10 text-white/70 hover:bg-white/10 hover:text-white'
                : 'bg-white/2 border border-white/5 text-white/20 cursor-not-allowed'
            )}
          >
            Export
          </button>
          <button
            onClick={() => onScanAction('import')}
            className="px-3 py-2 rounded-lg text-xs font-medium bg-white/5 border border-white/10 text-white/70 hover:bg-white/10 hover:text-white transition-all"
          >
            Import
          </button>
        </div>
      </Section>

    </>
  );
}

function SliderRow({ label, value, unit, min, max, step, onChange, disabled, accent = 'cyan' }) {
  const accentClass = accent === 'amber' ? 'accent-amber-500' : 'accent-cyan-500';
  return (
    <div className={cn('flex flex-col gap-1', disabled && 'opacity-35 pointer-events-none')}>
      <div className="flex items-center justify-between">
        <span className="text-[10px] uppercase tracking-wider text-[#555555] font-medium">{label}</span>
        <span className="text-[10px] font-mono text-white/60">{value} {unit}</span>
      </div>
      <input
        type="range"
        min={min}
        max={max}
        step={step}
        value={value}
        disabled={disabled}
        onChange={(e) => onChange(parseFloat(e.target.value))}
        className={cn('w-full h-1 rounded-full appearance-none bg-white/10 cursor-pointer', accentClass,
          disabled && 'cursor-not-allowed')}
      />
      <div className="flex justify-between text-[9px] font-mono text-white/30">
        <span>{min}</span>
        <span>{max}</span>
      </div>
    </div>
  );
}

function EditableField({ label, value, unit, onChange, min, max, locked }) {
  const [editing, setEditing] = useState(false);
  const [draft, setDraft] = useState('');

  const startEdit = () => {
    if (locked) return;
    setDraft(String(value));
    setEditing(true);
  };

  const commit = () => {
    const num = parseFloat(draft);
    if (!isNaN(num) && num >= min && num <= max) {
      onChange(num);
    }
    setEditing(false);
  };

  return (
    <div
      onClick={!editing ? startEdit : undefined}
      className={cn(
        'relative flex flex-col gap-0.5 p-3 rounded-xl border',
        'transition-all duration-300',
        locked
          ? 'border-white/5 bg-[#0a0a0a]/40 opacity-40 cursor-not-allowed'
          : editing
            ? 'border-[#6B9BD2]/40 bg-[#6B9BD2]/5 cursor-text'
            : 'border-white/8 bg-[#0a0a0a]/60 cursor-pointer hover:border-white/20 hover:bg-white/[0.02]',
      )}
    >
      <span className="text-[10px] font-medium uppercase tracking-wider text-[#555555]">{label}</span>
      {editing ? (
        <div className="flex items-baseline gap-1">
          <input
            autoFocus
            type="text"
            value={draft}
            onChange={e => setDraft(e.target.value)}
            onBlur={commit}
            onKeyDown={e => { if (e.key === 'Enter') commit(); if (e.key === 'Escape') setEditing(false); }}
            className="bg-transparent text-base font-bold font-mono text-white outline-none w-14"
          />
          <span className="text-xs font-semibold text-[#888888]">{unit}</span>
        </div>
      ) : (
        <div className="flex items-baseline gap-1">
          <span className="text-base font-bold font-mono text-white">{value}</span>
          <span className="text-xs font-semibold text-[#888888]">{unit}</span>
        </div>
      )}
      {editing && (
        <div className="absolute bottom-0 left-3 right-3 h-px bg-gradient-to-r from-[#6B9BD2] to-[#8BB8E8] rounded-full" />
      )}
    </div>
  );
}
