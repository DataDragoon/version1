import { useState } from 'react';
import { cn } from '@/lib/utils';
import { Section, InfoTile } from './Sidebar';

export default function SarPanel({ bscanData, sarResult, sarProgress, bgEnabled, onBgEnabledChange, svdEnabled, svdK, svdStrength, onSvdEnabledChange, onSvdKChange, onSvdStrengthChange, scaleMode, onScaleModeChange, aperture, onApertureChange, coherent, onCoherentChange, dynRange, onDynRangeChange, maxDepth, onMaxDepthChange, epsilonR, onEpsilonRChange, windowType, onWindowTypeChange, wallThickness, onWallThicknessChange, refraction, onRefractionChange, viewMode, onViewModeChange, colormap, onColormapChange, onScanAction }) {
  const numPositions = bscanData ? bscanData.length : 0;

  // How many cells carry a lidar standoff. The back-projection uses each cell's own
  // value; cells without one fall back to zero, so a partially-instrumented scan is
  // silently mixing two conventions and the operator should be able to see that.
  const standoffN = sarResult ? sarResult.standoffN : null;
  const standoffPartial = standoffN !== null && standoffN > 0 && standoffN < numPositions;

  return (
    <>
      <Section label="Status">
        <div className="grid grid-cols-2 gap-2">
          <InfoTile label="Positions" value={numPositions < 2 ? `${numPositions} (need ≥2)` : numPositions} />
          {sarResult && <InfoTile label="Time" value={`${sarResult.computeTimeMs} ms`} />}
        </div>
        {sarResult && (
          <div className="grid grid-cols-2 gap-2">
            <InfoTile label="Grid" value={`${sarResult.pixelsX}×${sarResult.pixelsZ}`} />
            <InfoTile label="Aperture" value={`${(sarResult.apertureLength * 100).toFixed(1)} cm`} />
          </div>
        )}
        {sarProgress !== null && (
          <div className="flex flex-col gap-1">
            <div className="flex items-center justify-between">
              <span className="text-[10px] uppercase tracking-wider text-emerald-400 font-medium">Reconstructing...</span>
              <span className="text-[10px] font-mono text-white/60">{Math.round(sarProgress * 100)}%</span>
            </div>
            <div className="h-1 w-full rounded-full bg-white/5 overflow-hidden">
              <div
                className="h-full bg-emerald-500 rounded-full transition-[width] duration-100"
                style={{ width: `${sarProgress * 100}%` }}
              />
            </div>
          </div>
        )}
      </Section>

      <Section label="Mode">
        <div className="flex gap-2">
          <button
            onClick={() => onCoherentChange(true)}
            className={cn(
              'flex-1 px-3 py-2 rounded-lg text-xs font-medium transition-all border',
              coherent
                ? 'bg-purple-500/10 border-purple-500/30 text-purple-400'
                : 'bg-white/5 border-white/10 text-white/70 hover:bg-white/10 hover:text-white'
            )}
          >
            Coherent
          </button>
          <button
            onClick={() => onCoherentChange(false)}
            className={cn(
              'flex-1 px-3 py-2 rounded-lg text-xs font-medium transition-all border',
              !coherent
                ? 'bg-purple-500/10 border-purple-500/30 text-purple-400'
                : 'bg-white/5 border-white/10 text-white/70 hover:bg-white/10 hover:text-white'
            )}
          >
            Incoherent
          </button>
        </div>
      </Section>

      <Section label="Background">
        <button
          onClick={() => onBgEnabledChange(!bgEnabled)}
          className={cn(
            'w-full px-3 py-2 rounded-lg text-xs font-medium transition-all border',
            bgEnabled
              ? 'bg-emerald-500/10 border-emerald-500/30 text-emerald-400'
              : 'bg-white/5 border-white/10 text-white/70 hover:bg-white/10 hover:text-white'
          )}
        >
          {bgEnabled ? '● BG Subtract ON' : 'BG Subtract OFF'}
        </button>
      </Section>

      <Section label="SVD Filter">
        <button
          onClick={() => onSvdEnabledChange(!svdEnabled)}
          disabled={numPositions < 2}
          className={cn(
            'w-full px-3 py-2 rounded-lg text-xs font-medium transition-all border',
            numPositions < 2
              ? 'bg-white/2 border-white/5 text-white/20 cursor-not-allowed'
              : svdEnabled
                ? 'bg-[#6B9BD2]/10 border-[#6B9BD2]/30 text-[#6B9BD2]'
                : 'bg-white/5 border-white/10 text-white/70 hover:bg-white/10 hover:text-white'
          )}
        >
          {svdEnabled ? '● SVD ON' : 'SVD OFF'}
        </button>
        <div className="grid grid-cols-2 gap-2">
          <EditableField
            label="k (remove)"
            value={svdK}
            unit=""
            onChange={(v) => onSvdKChange(Math.round(v))}
            min={1}
            max={Math.max(1, numPositions - 1)}
          />
          <EditableField
            label="Strength"
            value={svdStrength}
            unit=""
            onChange={(v) => onSvdStrengthChange(v)}
            min={0.01}
            max={1}
          />
        </div>
      </Section>

      <Section label="Reconstruction">
        <div className="grid grid-cols-2 gap-2">
          {/* Relative permittivity of the medium. Sets the wave velocity the
              back-projection assumes, so it controls BOTH the shape of the hyperbola
              being matched (i.e. whether anything focuses at all) and the calibration
              of the depth axis. Until 2026-09-03 there was no such control and the
              reconstruction ran at the speed of light in air. 4.5 is dry brick. */}
          <EditableField
            label="εr (medium)"
            value={epsilonR}
            unit=""
            onChange={onEpsilonRChange}
            min={1}
            max={30}
          />
          <div className="flex flex-col gap-1 p-3 rounded-xl border border-white/8 bg-[#0a0a0a]/60">
            <span className="text-[10px] font-medium uppercase tracking-wider text-[#555555]">Window</span>
            <select
              value={windowType}
              onChange={(e) => onWindowTypeChange(e.target.value)}
              className="bg-transparent text-xs font-mono text-white outline-none cursor-pointer -ml-0.5"
            >
              <option value="rectangular" className="bg-[#0a0a0a]">Rectangular</option>
              <option value="hanning" className="bg-[#0a0a0a]">Hanning</option>
              <option value="kaiser" className="bg-[#0a0a0a]">Kaiser β3</option>
            </select>
          </div>
        </div>
        <div className="grid grid-cols-2 gap-2">
          {/* Operator-measured, because nothing on the rig can infer it. It is what tells
              the layered model where the dielectric STOPS -- beyond the back face it is
              air again, and a uniform-dielectric model puts anything back there at the
              wrong depth with the wrong hyperbola curvature. 29 cm is THIS bench's wall;
              re-measure it for any other. Cross-check: the back-face echo should land at
              standoff + √εr × thickness of apparent range. */}
          <EditableField
            label="Wall thick."
            value={wallThickness}
            unit="cm"
            onChange={onWallThicknessChange}
            min={0}
            max={200}
          />
          <button
            onClick={() => onRefractionChange(!refraction)}
            disabled={!(wallThickness > 0)}
            className={cn(
              'px-3 py-2 rounded-xl text-xs font-medium transition-all border',
              !(wallThickness > 0)
                ? 'bg-white/2 border-white/5 text-white/20 cursor-not-allowed'
                : refraction
                  ? 'bg-[#D1855C]/10 border-[#D1855C]/30 text-[#D1855C]'
                  : 'bg-white/5 border-white/10 text-white/70 hover:bg-white/10 hover:text-white'
            )}
          >
            {refraction ? '● Layered ray' : 'Straight ray'}
          </button>
        </div>
        <div className="px-1 text-[9px] text-[#555555] leading-relaxed">
          {/* Temporary A/B control. Once the layered model is confirmed better on real
              data the toggle goes and it becomes unconditional. */}
          <span className="text-[#888888]">Straight ray</span> adds the standoff as a pure
          delay and treats everything below the face as one infinite dielectric.{' '}
          <span className="text-[#888888]">Layered ray</span> traces air → wall → air with
          Snell at both faces — worth ~29° of two-way phase at wide angles, and it drops
          contributions no refracted ray can reach. Coherent mode only.
        </div>
        <div className="px-1 text-[9px] text-[#555555] leading-relaxed">
          {/* Deliberately not asserting a winner -- see the note in sar.worker.js: two
              measurements on the same scan disagreed about rectangular vs Hanning. */}
          Depth axis is true depth below the wall face; apparent range is √εr × depth.
          Window is worth A/B-ing on a target-in / target-out pair — rectangular keeps
          resolution, Hanning measured better target/clutter separation end-to-end.
        </div>
        {standoffPartial && (
          <div className="px-1 text-[9px] text-amber-400/80 leading-relaxed">
            Only {standoffN}/{numPositions} cells have a lidar standoff — the rest are
            treated as zero standoff.
          </div>
        )}
        {sarResult && sarResult.depthClipped && (
          <div className="px-1 text-[9px] text-amber-400/80 leading-relaxed">
            Max Depth clipped to {(sarResult.depthMax * 100).toFixed(1)} cm — the sweep
            does not reach further at εr {(sarResult.epsilonR ?? 1).toFixed(2)}.
          </div>
        )}
      </Section>

      <Section label="Display">
        {/* Split shows amplitude and coherence as two images; Combined multiplies the
            linear amplitude by the coherence into one, which is the "final image". */}
        <div className="flex gap-2">
          <button
            onClick={() => onViewModeChange('split')}
            className={cn(
              'flex-1 px-3 py-2 rounded-lg text-xs font-medium transition-all border',
              viewMode !== 'combined'
                ? 'bg-[#4ecdc4]/10 border-[#4ecdc4]/30 text-[#4ecdc4]'
                : 'bg-white/5 border-white/10 text-white/70 hover:bg-white/10 hover:text-white'
            )}
          >
            Split
          </button>
          <button
            onClick={() => onViewModeChange('combined')}
            className={cn(
              'flex-1 px-3 py-2 rounded-lg text-xs font-medium transition-all border',
              viewMode === 'combined'
                ? 'bg-[#4ecdc4]/10 border-[#4ecdc4]/30 text-[#4ecdc4]'
                : 'bg-white/5 border-white/10 text-white/70 hover:bg-white/10 hover:text-white'
            )}
          >
            Amp × Coh
          </button>
        </div>
        {/* Reconstruction depth. Sets the extent -- and the cost -- of the
            output grid. It used to live in the C-scan panel as
            bscanParams.maxDepth, where it also clipped the B-scan pane's
            display; that second job is gone and this is the only real one.
            Since 2026-09-03 it is TRUE depth into the wall, not apparent range,
            so at er 4.5 a 30 cm setting reaches ~63 cm of apparent range. */}
        <EditableField
          label="Max Depth"
          value={maxDepth}
          unit="cm"
          onChange={onMaxDepthChange}
          min={1}
          max={500}
        />
        <div className="flex flex-col gap-1 p-3 rounded-xl border border-white/8 bg-[#0a0a0a]/60">
          <span className="text-[10px] font-medium uppercase tracking-wider text-[#555555]">Colormap</span>
          <select
            value={colormap}
            onChange={(e) => onColormapChange(e.target.value)}
            className="bg-transparent text-xs font-mono text-white outline-none cursor-pointer -ml-0.5"
          >
            <option value="inferno" className="bg-[#0a0a0a]">inferno</option>
            <option value="viridis" className="bg-[#0a0a0a]">viridis</option>
            <option value="jet" className="bg-[#0a0a0a]">jet</option>
          </select>
          {/* Coherence is deliberately excluded -- it holds its own ramp so the two
              split-view panes cannot be mistaken for one another. */}
          <span className="text-[9px] text-[#555555]">amplitude &amp; combined only</span>
        </div>
        <div className="flex gap-2">
          <button
            onClick={() => onScaleModeChange('db')}
            className={cn(
              'flex-1 px-3 py-2 rounded-lg text-xs font-medium transition-all border',
              scaleMode === 'db'
                ? 'bg-emerald-500/10 border-emerald-500/30 text-emerald-400'
                : 'bg-white/5 border-white/10 text-white/70 hover:bg-white/10 hover:text-white'
            )}
          >
            dB
          </button>
          <button
            onClick={() => onScaleModeChange('linear')}
            className={cn(
              'flex-1 px-3 py-2 rounded-lg text-xs font-medium transition-all border',
              scaleMode === 'linear'
                ? 'bg-emerald-500/10 border-emerald-500/30 text-emerald-400'
                : 'bg-white/5 border-white/10 text-white/70 hover:bg-white/10 hover:text-white'
            )}
          >
            Linear
          </button>
        </div>
        <div className="flex flex-col gap-1">
          <div className="flex items-center justify-between">
            <span className="text-[10px] uppercase tracking-wider text-[#555555] font-medium">Nearby positions</span>
            <span className="text-[10px] font-mono text-white/60">{aperture}</span>
          </div>
          <input
            type="range"
            min={1}
            max={Math.max(1, numPositions - 1)}
            value={aperture}
            onChange={(e) => onApertureChange(parseInt(e.target.value))}
            className="w-full h-1 rounded-full appearance-none bg-white/10 accent-emerald-500 cursor-pointer"
          />
          <div className="flex justify-between text-[9px] font-mono text-white/30">
            <span>1</span>
            <span>{Math.max(1, numPositions - 1)}</span>
          </div>
        </div>
        <div className="flex flex-col gap-1">
          <div className="flex items-center justify-between">
            <span className="text-[10px] uppercase tracking-wider text-[#555555] font-medium">Dynamic range</span>
            <span className="text-[10px] font-mono text-white/60">{dynRange} dB</span>
          </div>
          <input
            type="range"
            min={5}
            max={60}
            value={dynRange}
            onChange={(e) => onDynRangeChange(parseInt(e.target.value))}
            className="w-full h-1 rounded-full appearance-none bg-white/10 accent-emerald-500 cursor-pointer"
          />
          <div className="flex justify-between text-[9px] font-mono text-white/30">
            <span>5</span>
            <span>60</span>
          </div>
        </div>
      </Section>

      <Section label="Data">
        {/* Same handler the C-scan panel's Import uses -- SAR reconstructs from the C-scan
            capture list, so loading one here is loading SAR's input. There is no Export:
            the C-scan panel owns the capture record and exporting from two places would
            invite two formats. */}
        <button
          onClick={() => onScanAction && onScanAction('import')}
          className="w-full px-3 py-2 rounded-lg text-xs font-medium bg-white/5 border border-white/10 text-white/70 hover:bg-white/10 hover:text-white transition-all"
        >
          Import C-Scan
        </button>
        <div className="px-1 text-[9px] text-[#555555] leading-relaxed">
          Loads a saved scan (v3–v7) into the shared capture list and reconstructs it
          immediately. Same file the C-scan panel exports.
        </div>
      </Section>
    </>
  );
}

function EditableField({ label, value, unit, onChange, min, max }) {
  const [editing, setEditing] = useState(false);
  const [draft, setDraft] = useState('');

  const startEdit = () => {
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
        editing
          ? 'border-emerald-500/40 bg-emerald-500/5 cursor-text'
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
          {unit && <span className="text-xs font-semibold text-[#888888]">{unit}</span>}
        </div>
      ) : (
        <div className="flex items-baseline gap-1">
          <span className="text-base font-bold font-mono text-white">{value}</span>
          {unit && <span className="text-xs font-semibold text-[#888888]">{unit}</span>}
        </div>
      )}
      {editing && (
        <div className="absolute bottom-0 left-3 right-3 h-px bg-gradient-to-r from-emerald-500 to-emerald-300 rounded-full" />
      )}
    </div>
  );
}
