import { useState } from 'react';
import { cn } from '@/lib/utils';
import { Section, InfoTile } from './Sidebar';

export default function SarPanel({ bscanData, sarResult, sarProgress, bgEnabled, onBgEnabledChange, svdEnabled, svdK, svdStrength, onSvdEnabledChange, onSvdKChange, onSvdStrengthChange, scaleMode, onScaleModeChange, aperture, onApertureChange, coherent, onCoherentChange, dynRange, onDynRangeChange, maxDepth, onMaxDepthChange }) {
  const numPositions = bscanData ? bscanData.length : 0;

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

      <Section label="Display">
        {/* Reconstruction depth. Sets the extent -- and the cost -- of the
            output grid. It used to live in the C-scan panel as
            bscanParams.maxDepth, where it also clipped the B-scan pane's
            display; that second job is gone and this is the only real one. */}
        <EditableField
          label="Max Depth"
          value={maxDepth}
          unit="cm"
          onChange={onMaxDepthChange}
          min={1}
          max={500}
        />
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
