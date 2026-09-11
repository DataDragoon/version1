import { useState, useRef, useEffect, useMemo } from 'react';
import { cn } from '@/lib/utils';
import { Section, InfoTile } from './Sidebar';
import { analyzeCoverage } from '@/lib/bgCaptureStats';

const LIDAR_AVG_WINDOW = 20;

function weakestKnotMm(q) {
  const w = q.per.filter(p => p.suppDb != null).reduce((a, b) => (b.suppDb < a.suppDb ? b : a));
  return w.d.toFixed(0);
}

export default function BgModelPanel({ isConnected, sdrConnected, sfcwRunning, modelCaptures, modelCapturing, accumCount, testing, testCount, testResult, trainingState, trainProgress, trainResult, trainError, sweepsPerCapture = 40, onSweepsChange, stopFreq, onModelAction, lidarMm, roverConnected, roverStatus, bgScanMode, onBgScanModeChange, bgRoverSpanMm, onBgRoverSpanChange, bgRoverStepMm, onBgRoverStepChange, bgRoverDirection, onBgRoverDirectionChange, roverBgScan, sendRover, continuousActive, continuousStats, contBinMm = 1, onContBinChange, contMaxSpeed = 40, onContMaxSpeedChange }) {
  const [modelName, setModelName] = useState('');
  const lidarBuf = useRef([]);
  const [lidarAvg, setLidarAvg] = useState(null);

  useEffect(() => {
    if (lidarMm == null) return;
    const buf = lidarBuf.current;
    buf.push(lidarMm);
    if (buf.length > LIDAR_AVG_WINDOW) buf.shift();
    const avg = buf.reduce((s, v) => s + v, 0) / buf.length;
    setLidarAvg(avg);
  }, [lidarMm]);

  const canActivate = isConnected && sdrConnected;
  const canRover = canActivate && roverConnected && roverStatus?.board_connected;
  const isRover = bgScanMode === 'rover';
  const roverPositions = bgRoverStepMm > 0 ? Math.floor(bgRoverSpanMm / bgRoverStepMm) + 1 : 0;
  const captureCount = modelCaptures.length;

  const totalSamples = modelCaptures.reduce((n, c) => n + c.samples.length, 0);
  // Memoized because a continuous run turns this from ~30 positions into a few
  // hundred, and the panel re-renders at the lidar rate (the live standoff
  // readout), not at the capture rate.
  const coverage = useMemo(() => analyzeCoverage(modelCaptures, stopFreq), [modelCaptures, stopFreq]);
  const { limits } = coverage;
  const POSITION_ROW_LIMIT = 80;

  const contSpeed = continuousStats?.speedMmS;
  const contSpeedTone = contSpeed == null ? 'text-white/30'
    : contMaxSpeed > 0 && Math.abs(contSpeed) > contMaxSpeed ? 'text-red-400'
    : contMaxSpeed > 0 && Math.abs(contSpeed) > 0.6 * contMaxSpeed ? 'text-yellow-400'
    : 'text-green-400';

  // Spacing verdict against the alpha=3 aliasing limit at the top of the band
  const gapVerdict = coverage.maxGap == null ? null
    : coverage.maxGap <= limits.goodMm ? { tone: 'good', label: 'well sampled' }
    : coverage.maxGap <= limits.aliasMm ? { tone: 'warn', label: 'coarse but unaliased' }
    : { tone: 'bad', label: 'aliases triple-bounce' };
  const toneClass = { good: 'text-green-400', warn: 'text-yellow-400', bad: 'text-red-400' };

  return (
    <>
      <Section label="Session">
        <div className="flex rounded-lg border border-white/10 overflow-hidden">
          <button
            onClick={() => onBgScanModeChange('manual')}
            className={cn(
              'flex-1 px-3 py-1.5 text-xs font-medium transition-all',
              !isRover ? 'bg-[#a78bfa]/15 text-[#a78bfa]' : 'bg-transparent text-white/40 hover:text-white/60',
            )}
          >Manual</button>
          <button
            onClick={() => canRover && onBgScanModeChange('rover')}
            disabled={!canRover}
            className={cn(
              'flex-1 px-3 py-1.5 text-xs font-medium transition-all',
              'disabled:opacity-30 disabled:cursor-not-allowed',
              isRover ? 'bg-[#a78bfa]/15 text-[#a78bfa]' : 'bg-transparent text-white/40 hover:text-white/60',
            )}
          >Rover</button>
        </div>

        <button
          onClick={() => onModelAction(sfcwRunning ? 'stop_session' : 'start_session')}
          disabled={!canActivate}
          className={cn(
            'group relative flex items-center gap-3 w-full p-4 rounded-2xl border',
            'transition-all duration-500 cursor-pointer',
            'disabled:cursor-not-allowed disabled:opacity-40',
            sfcwRunning
              ? 'bg-orange-500/8 border-orange-500/30 hover:border-orange-500/50'
              : canActivate
                ? 'bg-[#a78bfa]/8 border-[#a78bfa]/30 hover:border-[#a78bfa]/50'
                : 'bg-[#0a0a0a]/50 border-white/5',
          )}
        >
          <div className={cn(
            'flex items-center justify-center w-10 h-10 rounded-xl shrink-0 transition-all duration-500',
            sfcwRunning ? 'bg-orange-500/15' : canActivate ? 'bg-[#a78bfa]/15' : 'bg-white/5',
          )}>
            {sfcwRunning ? (
              <div className="w-3 h-3 rounded-sm bg-orange-400" />
            ) : (
              <div className="w-3 h-3 rounded-full border-2 border-current text-[#a78bfa]" />
            )}
          </div>
          <div className="flex flex-col gap-0.5 text-left min-w-0">
            <span className="text-sm font-semibold text-white">
              {sfcwRunning ? 'Stop Session' : 'Start Modeling'}
            </span>
            <span className="text-xs text-[#555555] leading-relaxed">
              {sfcwRunning ? 'Sweeping continuously...' :
               !sdrConnected ? 'SDR not connected' : 'Start continuous sweep'}
            </span>
          </div>
        </button>
      </Section>

      <Section label="Capture">
        {!isRover ? (
          <>
          <button
            onClick={() => onModelAction('capture')}
            disabled={!sfcwRunning || modelCapturing || continuousActive || roverBgScan?.active}
            className={cn(
              'group relative flex items-center gap-3 w-full p-4 rounded-2xl border',
              'transition-all duration-500 cursor-pointer',
              'disabled:cursor-not-allowed disabled:opacity-40',
              sfcwRunning && !modelCapturing && !continuousActive
                ? 'bg-[#a78bfa]/8 border-[#a78bfa]/30 hover:border-[#a78bfa]/50'
                : 'bg-[#0a0a0a]/50 border-white/5',
            )}
          >
            <div className={cn(
              'flex items-center justify-center w-10 h-10 rounded-xl shrink-0 transition-all duration-500',
              sfcwRunning && !modelCapturing && !continuousActive ? 'bg-[#a78bfa]/15' : 'bg-white/5',
            )}>
              {modelCapturing ? (
                <div className="w-3 h-3 rounded-full border-2 border-[#a78bfa] border-t-transparent animate-spin" />
              ) : (
                <svg className="w-4 h-4 text-[#a78bfa]" fill="none" viewBox="0 0 24 24" stroke="currentColor" strokeWidth={2}>
                  <path strokeLinecap="round" strokeLinejoin="round" d="M12 4v16m8-8H4" />
                </svg>
              )}
            </div>
            <div className="flex flex-col gap-0.5 text-left min-w-0">
              <span className="text-sm font-semibold text-white">
                {modelCapturing ? `Capturing ${accumCount}/${sweepsPerCapture}` : 'Capture Position'}
              </span>
              <span className="text-xs text-[#555555] leading-relaxed">
                {modelCapturing ? 'Hold still — averaging sweeps...' :
                 !sfcwRunning ? 'Start session first' :
                 `${captureCount} position${captureCount !== 1 ? 's' : ''} captured`}
              </span>
            </div>
          </button>

          <div className="flex flex-col gap-2 p-3 rounded-2xl border border-white/8 bg-[#0a0a0a]/60">
            <div className="flex items-center justify-between">
              <span className="text-[10px] font-medium uppercase tracking-wider text-[#555555]">Continuous capture</span>
              <span className="text-[9px] font-mono text-white/25">{sweepsPerCapture} sweeps / bin</span>
            </div>

            <button
              onClick={() => onModelAction(continuousActive ? 'continuous_stop' : 'continuous_start')}
              disabled={!sfcwRunning || modelCapturing}
              className={cn(
                'flex items-center gap-3 w-full px-3 py-2.5 rounded-xl border transition-all cursor-pointer',
                'disabled:cursor-not-allowed disabled:opacity-40',
                continuousActive
                  ? 'bg-orange-500/8 border-orange-500/30 hover:border-orange-500/50'
                  : sfcwRunning && !modelCapturing
                    ? 'bg-[#a78bfa]/8 border-[#a78bfa]/30 hover:border-[#a78bfa]/50'
                    : 'bg-[#0a0a0a]/50 border-white/5',
              )}
            >
              <div className={cn(
                'flex items-center justify-center w-8 h-8 rounded-lg shrink-0',
                continuousActive ? 'bg-orange-500/15' : sfcwRunning ? 'bg-[#a78bfa]/15' : 'bg-white/5',
              )}>
                {continuousActive ? (
                  <div className="w-2.5 h-2.5 rounded-sm bg-orange-400" />
                ) : (
                  <svg className="w-4 h-4 text-[#a78bfa]" fill="none" viewBox="0 0 24 24" stroke="currentColor" strokeWidth={2}>
                    <path strokeLinecap="round" d="M3 12c2-4 4-4 6 0s4 4 6 0 4-4 6 0" />
                  </svg>
                )}
              </div>
              <div className="flex flex-col gap-0.5 text-left min-w-0">
                <span className="text-xs font-semibold text-white">
                  {continuousActive ? 'Stop & Bin' : 'Start Continuous'}
                </span>
                <span className="text-[10px] text-[#555555] leading-relaxed">
                  {continuousActive ? 'Sweep the module slowly across the span'
                   : !sfcwRunning ? 'Start session first'
                   : 'Wave across the span, positions bin themselves'}
                </span>
              </div>
            </button>

            <div className="grid grid-cols-2 gap-2">
              <div className="flex items-center justify-between gap-1 px-2 py-1.5 rounded-lg border border-white/8">
                <span className="text-[9px] uppercase tracking-wider text-[#555555]">Bin mm</span>
                <input
                  type="number" min={0.5} max={20} step={0.5}
                  value={contBinMm}
                  disabled={continuousActive}
                  onChange={e => onContBinChange && onContBinChange(e.target.value)}
                  className="w-12 px-1 py-0.5 rounded text-[11px] font-mono text-right bg-[#0a0a0a] border border-white/10 text-white outline-none focus:border-[#a78bfa]/50 disabled:opacity-40"
                />
              </div>
              <div className="flex items-center justify-between gap-1 px-2 py-1.5 rounded-lg border border-white/8">
                <span className="text-[9px] uppercase tracking-wider text-[#555555]">Max mm/s</span>
                <input
                  type="number" min={0} max={1000} step={5}
                  value={contMaxSpeed}
                  disabled={continuousActive}
                  onChange={e => onContMaxSpeedChange && onContMaxSpeedChange(e.target.value)}
                  className="w-12 px-1 py-0.5 rounded text-[11px] font-mono text-right bg-[#0a0a0a] border border-white/10 text-white outline-none focus:border-[#a78bfa]/50 disabled:opacity-40"
                />
              </div>
            </div>

            {continuousActive && continuousStats && (
              <div className="flex flex-col gap-1.5 p-2.5 rounded-xl border border-[#a78bfa]/20 bg-[#a78bfa]/5">
                <div className="flex items-baseline justify-between">
                  <span className="text-[10px] text-[#555]">Speed</span>
                  <span className={cn('text-sm font-bold font-mono', contSpeedTone)}>
                    {contSpeed != null ? Math.abs(contSpeed).toFixed(0) : '—'}
                    <span className="text-[9px] font-semibold text-[#888] ml-1">mm/s</span>
                    {continuousStats.smearMm != null && (
                      <span className="text-[9px] font-normal text-white/30 ml-1.5">
                        {continuousStats.smearMm.toFixed(2)} mm/sweep
                      </span>
                    )}
                  </span>
                </div>
                <div className="grid grid-cols-2 gap-x-3 gap-y-0.5 text-[10px] pt-1 border-t border-white/5">
                  <div className="flex justify-between">
                    <span className="text-[#555]">Bins</span>
                    <span className="font-mono text-white">{continuousStats.bins}</span>
                  </div>
                  <div className="flex justify-between">
                    <span className="text-[#555]">Kept</span>
                    <span className="font-mono text-white">{continuousStats.accepted}</span>
                  </div>
                  <div className="flex justify-between">
                    <span className="text-[#555]">Span</span>
                    <span className="font-mono text-white">{continuousStats.spanMm.toFixed(0)} mm</span>
                  </div>
                  <div className="flex justify-between">
                    <span className="text-[#555]">Hole</span>
                    <span className={cn('font-mono',
                      continuousStats.maxGapMm == null ? 'text-white/30'
                      : continuousStats.maxGapMm > limits.aliasMm ? 'text-red-400'
                      : continuousStats.maxGapMm > limits.goodMm ? 'text-yellow-400' : 'text-green-400')}>
                      {continuousStats.maxGapMm != null ? continuousStats.maxGapMm.toFixed(1) + ' mm' : '—'}
                    </span>
                  </div>
                  <div className="flex justify-between">
                    <span className="text-[#555]">Med/bin</span>
                    <span className="font-mono text-white/70">{continuousStats.medianPerBin}</span>
                  </div>
                  <div className="flex justify-between">
                    <span className="text-[#555]">Rate</span>
                    <span className="font-mono text-white/70">
                      {continuousStats.sweepHz ? continuousStats.sweepHz.toFixed(0) + ' Hz' : '—'}
                    </span>
                  </div>
                  <div className="flex justify-between">
                    <span className="text-[#555]">Kept %</span>
                    <span className="font-mono text-white/70">
                      {continuousStats.total
                        ? ((100 * continuousStats.accepted) / continuousStats.total).toFixed(0) + '%'
                        : '—'}
                    </span>
                  </div>
                  <div className="flex justify-between">
                    <span className="text-[#555]">Lidar</span>
                    <span className="font-mono text-white/70">
                      {continuousStats.medianBracketMs != null
                        ? continuousStats.medianBracketMs.toFixed(0) + ' ms'
                        : '—'}
                    </span>
                  </div>
                </div>
                <div className="flex justify-between text-[9px] pt-1 border-t border-white/5 text-white/30">
                  <span>no bracket {continuousStats.no_lidar}</span>
                  <span>too fast {continuousStats.motion}</span>
                  <span>bin full {continuousStats.bin_full}</span>
                </div>
                {!continuousStats.interpolated && continuousStats.total > 20 && (
                  <div className="text-[9px] text-yellow-400/70 leading-relaxed pt-1 border-t border-white/5">
                    No lidar timestamps from the Pi — falling back to each sweep's own
                    averaged reading, which lags by up to one lidar period while moving.
                    Update stream.py.
                  </div>
                )}
              </div>
            )}

            {!continuousActive && continuousStats?.harvested != null && (
              <div className="p-2.5 rounded-xl border border-green-500/20 bg-green-500/5">
                <span className="text-[10px] text-green-400">
                  Added {continuousStats.harvested} position{continuousStats.harvested !== 1 ? 's' : ''} from {continuousStats.accepted} sweeps over {continuousStats.spanMm.toFixed(0)} mm
                  {continuousStats.screened > 0 && (
                    <span className="text-yellow-400/80">
                      {' '}· {continuousStats.screened} sweep{continuousStats.screened !== 1 ? 's' : ''} dropped
                      for disagreeing with their bin
                    </span>
                  )}
                </span>
              </div>
            )}

            <div className="text-[9px] text-white/30 leading-relaxed">
              Standoff is interpolated between the lidar measurements either side of each
              sweep, so every sweep is filed where it was actually taken rather than where
              the last reading said. Only sweeps the lidar never bracketed, or taken above
              the speed limit, are dropped. Pass back and forth to deepen the bins, and
              watch Hole rather than Span. The speed limit is a smear budget: a sweep steps
              through frequency in order, so moving during one shifts its apparent range by
              about 0.15 mm at 40 mm/s and 0.38 mm at 100 mm/s. A fast pass also spaces its
              lidar readings further apart than the bin width, which leaves holes — but
              further passes fill them, so a fast wave just needs more of them.
            </div>
          </div>
          </>
        ) : (
          <>
            <div className="flex flex-col gap-2">
              <div className="flex items-center justify-between gap-2 px-3 py-2 rounded-xl border border-white/8 bg-[#0a0a0a]/60">
                <span className="text-[10px] font-medium uppercase tracking-wider text-[#555555]">Span (mm)</span>
                <input
                  type="number"
                  min={1}
                  step={1}
                  value={bgRoverSpanMm}
                  disabled={roverBgScan?.active}
                  onChange={e => onBgRoverSpanChange(e.target.value)}
                  className="w-20 px-2 py-1 rounded-lg text-xs font-mono text-right bg-[#0a0a0a] border border-white/10 text-white outline-none focus:border-[#a78bfa]/50 disabled:opacity-40"
                />
              </div>
              <div className="flex items-center justify-between gap-2 px-3 py-2 rounded-xl border border-white/8 bg-[#0a0a0a]/60">
                <span className="text-[10px] font-medium uppercase tracking-wider text-[#555555]">Step (mm)</span>
                <input
                  type="number"
                  min={0.1}
                  step={0.1}
                  value={bgRoverStepMm}
                  disabled={roverBgScan?.active}
                  onChange={e => onBgRoverStepChange(e.target.value)}
                  className="w-20 px-2 py-1 rounded-lg text-xs font-mono text-right bg-[#0a0a0a] border border-white/10 text-white outline-none focus:border-[#a78bfa]/50 disabled:opacity-40"
                />
              </div>
              <div className="flex items-center justify-between gap-2 px-3 py-2 rounded-xl border border-white/8 bg-[#0a0a0a]/60">
                <span className="text-[10px] font-medium uppercase tracking-wider text-[#555555]">Direction</span>
                <div className="flex rounded-md border border-white/10 overflow-hidden">
                  <button
                    onClick={() => onBgRoverDirectionChange('forward')}
                    disabled={roverBgScan?.active}
                    className={cn(
                      'px-2.5 py-1 text-[10px] font-medium transition-all',
                      'disabled:opacity-40',
                      bgRoverDirection !== 'backward' ? 'bg-[#a78bfa]/15 text-[#a78bfa]' : 'text-white/40 hover:text-white/60',
                    )}
                  >+X</button>
                  <button
                    onClick={() => onBgRoverDirectionChange('backward')}
                    disabled={roverBgScan?.active}
                    className={cn(
                      'px-2.5 py-1 text-[10px] font-medium transition-all',
                      'disabled:opacity-40',
                      bgRoverDirection === 'backward' ? 'bg-[#a78bfa]/15 text-[#a78bfa]' : 'text-white/40 hover:text-white/60',
                    )}
                  >−X</button>
                </div>
              </div>
              <div className="px-3 py-1.5 text-[10px] text-white/40 font-mono">
                {roverPositions} positions over {bgRoverSpanMm.toFixed(1)} mm ({bgRoverDirection === 'backward' ? '−X' : '+X'})
              </div>
            </div>

            <button
              onClick={() => roverBgScan?.active
                ? roverBgScan.stop()
                : roverBgScan?.start({ spanMm: bgRoverSpanMm, stepMm: bgRoverStepMm, direction: bgRoverDirection })
              }
              disabled={!canRover || (!roverBgScan?.active && roverPositions < 2)}
              className={cn(
                'group relative flex items-center gap-3 w-full p-4 rounded-2xl border',
                'transition-all duration-500 cursor-pointer',
                'disabled:cursor-not-allowed disabled:opacity-40',
                roverBgScan?.active
                  ? 'bg-orange-500/8 border-orange-500/30 hover:border-orange-500/50'
                  : canRover
                    ? 'bg-[#a78bfa]/8 border-[#a78bfa]/30 hover:border-[#a78bfa]/50'
                    : 'bg-[#0a0a0a]/50 border-white/5',
              )}
            >
              <div className={cn(
                'flex items-center justify-center w-10 h-10 rounded-xl shrink-0 transition-all duration-500',
                roverBgScan?.active ? 'bg-orange-500/15' : canRover ? 'bg-[#a78bfa]/15' : 'bg-white/5',
              )}>
                {roverBgScan?.active ? (
                  <div className="w-3 h-3 rounded-sm bg-orange-400" />
                ) : (
                  <svg className="w-4 h-4 text-[#a78bfa]" fill="none" viewBox="0 0 24 24" stroke="currentColor" strokeWidth={2}>
                    <path strokeLinecap="round" strokeLinejoin="round" d="M5 3l14 9-14 9V3z" />
                  </svg>
                )}
              </div>
              <div className="flex flex-col gap-0.5 text-left min-w-0">
                <span className="text-sm font-semibold text-white">
                  {roverBgScan?.active ? 'Stop Scan' : 'Start Scan'}
                </span>
                <span className="text-xs text-[#555555] leading-relaxed">
                  {roverBgScan?.active
                    ? roverBgScan.message || `Position ${(roverBgScan.index || 0) + 1} of ${roverBgScan.total}`
                    : !canRover ? 'Rover not connected'
                    : `${roverPositions} positions, ${sweepsPerCapture} sweeps each`}
                </span>
              </div>
            </button>

            {roverBgScan?.active && (
              <div className="flex items-center gap-2 px-3 py-2 rounded-xl border border-[#a78bfa]/20 bg-[#a78bfa]/5">
                <div className="w-3 h-3 rounded-full border-2 border-[#a78bfa] border-t-transparent animate-spin" />
                <span className="text-[10px] text-[#a78bfa] capitalize">
                  {roverBgScan.phase} — position {(roverBgScan.index || 0) + 1} of {roverBgScan.total}
                </span>
              </div>
            )}

            {!roverBgScan?.active && roverBgScan?.error && (
              <div className="p-3 rounded-xl border border-red-500/20 bg-red-500/5">
                <span className="text-xs text-red-400">{roverBgScan.error}</span>
              </div>
            )}

            {!roverBgScan?.active && roverBgScan?.phase === 'done' && (
              <div className="p-3 rounded-xl border border-green-500/20 bg-green-500/5">
                <span className="text-xs text-green-400">{roverBgScan.message}</span>
              </div>
            )}

            {!roverBgScan?.active && roverBgScan?.phase === 'stopped' && (
              <div className="p-3 rounded-xl border border-yellow-500/20 bg-yellow-500/5">
                <span className="text-xs text-yellow-400">{roverBgScan.message}</span>
              </div>
            )}

            {isRover && roverStatus?.estop && (
              <div className="flex flex-col gap-2">
                <button
                  onClick={() => sendRover?.({ cmd: 'rover_clear_estop' })}
                  className="w-full px-3 py-2.5 rounded-lg text-xs font-medium transition-all border bg-amber-500/10 border-amber-500/30 text-amber-400 hover:bg-amber-500/20"
                >
                  Clear E-Stop
                </button>
                <div className="text-[9px] text-amber-400/60 leading-relaxed px-1">
                  Position is no longer trustworthy — re-measure offset before resuming.
                </div>
              </div>
            )}
          </>
        )}

        <div className="flex items-center justify-between gap-2 px-3 py-2 rounded-xl border border-white/8 bg-[#0a0a0a]/60">
          <div className="flex flex-col gap-0.5 min-w-0">
            <span className="text-[10px] font-medium uppercase tracking-wider text-[#555555]">Sweeps / position</span>
            <span className="text-[9px] text-white/30">
              +{(10 * Math.log10(sweepsPerCapture)).toFixed(1)} dB noise rejection
            </span>
          </div>
          <input
            type="number"
            min={1}
            max={500}
            value={sweepsPerCapture}
            disabled={modelCapturing || roverBgScan?.active}
            onChange={e => onSweepsChange && onSweepsChange(e.target.value)}
            className="w-16 px-2 py-1 rounded-lg text-xs font-mono text-right bg-[#0a0a0a] border border-white/10 text-white outline-none focus:border-[#a78bfa]/50 disabled:opacity-40"
          />
        </div>

        <div className="grid grid-cols-2 gap-2">
          <button
            onClick={() => onModelAction('undo')}
            disabled={captureCount === 0 || roverBgScan?.active}
            className={cn(
              'px-3 py-2 rounded-lg text-xs font-medium transition-all',
              captureCount > 0 && !roverBgScan?.active
                ? 'bg-white/5 border border-white/10 text-white/70 hover:bg-white/10 hover:text-white'
                : 'bg-white/2 border border-white/5 text-white/20 cursor-not-allowed'
            )}
          >
            Undo Last
          </button>
          <button
            onClick={() => onModelAction('clear')}
            disabled={captureCount === 0 || roverBgScan?.active}
            className={cn(
              'px-3 py-2 rounded-lg text-xs font-medium transition-all',
              captureCount > 0 && !roverBgScan?.active
                ? 'bg-white/5 border border-white/10 text-white/70 hover:bg-white/10 hover:text-white'
                : 'bg-white/2 border border-white/5 text-white/20 cursor-not-allowed'
            )}
          >
            Clear All
          </button>
        </div>
      </Section>

      <Section label="Standoff">
        <div className="flex flex-col gap-2">
          <div className="flex items-baseline justify-between px-3 py-2 rounded-xl border border-white/8 bg-[#0a0a0a]/60">
            <span className="text-[10px] font-medium uppercase tracking-wider text-[#555555]">Live</span>
            <span className="text-base font-bold font-mono text-white">
              {lidarAvg != null ? lidarAvg.toFixed(1) : '—'} <span className="text-xs font-semibold text-[#888888]">mm</span>
            </span>
          </div>
        </div>
      </Section>

      <Section label="Coverage">
        <div className="grid grid-cols-2 gap-2">
          <InfoTile label="Positions" value={`${captureCount} (${totalSamples})`} />
          <InfoTile label="Span" value={coverage.spanMm ? `${(coverage.spanMm / 10).toFixed(1)} cm` : '—'} />
        </div>

        {coverage.maxGap != null && (
          <div className="flex flex-col gap-1.5 p-3 rounded-xl border border-white/8 bg-[#0a0a0a]/60">
            <div className="flex justify-between text-[10px]">
              <span className="text-[#555]">Median gap</span>
              <span className="font-mono text-white">{coverage.medianGap.toFixed(1)} mm</span>
            </div>
            <div className="flex justify-between text-[10px]">
              <span className="text-[#555]">Largest gap</span>
              <span className={cn('font-mono font-bold', toneClass[gapVerdict.tone])}>
                {coverage.maxGap.toFixed(1)} mm
              </span>
            </div>
            <div className={cn('text-[9px] font-medium', toneClass[gapVerdict.tone])}>
              {gapVerdict.label}
            </div>
            {gapVerdict.tone !== 'good' && (
              <div className="text-[10px] text-[#a78bfa] font-mono pt-1 border-t border-white/5">
                next → {coverage.worstGapAt.toFixed(1)} mm
              </div>
            )}
            <div className="text-[9px] text-white/30 leading-relaxed pt-1 border-t border-white/5">
              Keep gaps under {limits.goodMm.toFixed(1)} mm. Above {limits.aliasMm.toFixed(1)} mm
              the triple-bounce echo aliases at {stopFreq / 1000} GHz. Irregular spacing is
              fine — better than uniform — but avoid leaving one big hole.
            </div>
          </div>
        )}

        {captureCount > 0 && (
          <div className="flex flex-col gap-1 max-h-40 overflow-y-auto px-1">
            {coverage.positions.slice(0, POSITION_ROW_LIMIT).map((p) => {
              const st = p.stats;
              return (
                <div key={p.index} className="flex justify-between items-baseline gap-2 text-[10px] px-1 py-0.5 rounded hover:bg-white/5">
                  <span className="text-white/40 shrink-0">#{p.index + 1}</span>
                  <span className="font-mono text-white/80">
                    {p.mm.toFixed(1)}
                    {st && st.standoffStdMm > 0 && (
                      <span className="text-white/25"> ±{st.standoffStdMm.toFixed(1)}</span>
                    )}
                  </span>
                  <span className="font-mono text-white/25 shrink-0 w-7 text-right">
                    {st ? `×${st.sweepCount}` : ''}
                  </span>
                  <span className={cn(
                    'font-mono shrink-0 w-12 text-right',
                    !st || st.coherence == null ? 'text-white/20'
                      : st.coherence > 0.95 ? 'text-green-400/70'
                      : st.coherence > 0.85 ? 'text-yellow-400/70'
                      : 'text-red-400/70'
                  )}>
                    {st && st.snrDbAveraged != null ? `${st.snrDbAveraged.toFixed(0)} dB` : '—'}
                  </span>
                </div>
              );
            })}
            {coverage.positions.length > POSITION_ROW_LIMIT && (
              <div className="text-[9px] text-white/25 text-center py-0.5">
                +{coverage.positions.length - POSITION_ROW_LIMIT} more
              </div>
            )}
          </div>
        )}
      </Section>

      <Section label="Phase Test">
        <button
          onClick={() => onModelAction('test_phase')}
          disabled={!sfcwRunning || testing || modelCapturing}
          className={cn(
            'w-full px-3 py-2.5 rounded-lg text-xs font-medium transition-all border',
            testing
              ? 'bg-yellow-500/10 border-yellow-500/30 text-yellow-400'
              : sfcwRunning && !modelCapturing
                ? 'bg-white/5 border-white/10 text-white/70 hover:bg-white/10 hover:text-white'
                : 'bg-white/2 border-white/5 text-white/20 cursor-not-allowed'
          )}
        >
          {testing ? `Testing ${testCount}/5...` : 'Test Phase Unwind'}
        </button>
        {testResult && (
          <div className="flex flex-col gap-2 p-3 rounded-xl border border-white/8 bg-[#0a0a0a]/60">
            <div className="flex justify-between text-[10px]">
              <span className="text-[#555]">Round-trip error</span>
              <span className={cn('font-mono font-bold', testResult.maxErrorOverall < 1e-10 ? 'text-green-400' : 'text-red-400')}>
                {testResult.maxErrorOverall.toExponential(2)}
              </span>
            </div>
            <div className="flex justify-between text-[10px]">
              <span className="text-[#555]">Residual SNR</span>
              <span className="font-mono font-bold text-white">{testResult.residualSnrDb.toFixed(1)} dB</span>
            </div>
            <div className="flex justify-between text-[10px]">
              <span className="text-[#555]">Sweep correlation</span>
              <span className={cn('font-mono font-bold', testResult.residualCorrelation > 0.95 ? 'text-green-400' : testResult.residualCorrelation > 0.8 ? 'text-yellow-400' : 'text-red-400')}>
                {testResult.residualCorrelation.toFixed(4)}
              </span>
            </div>
            <div className="flex justify-between text-[10px]">
              <span className="text-[#555]">Lidar spread</span>
              <span className="font-mono font-bold text-white">
                {(Math.max(...testResult.distances) - Math.min(...testResult.distances)).toFixed(1)} mm
              </span>
            </div>
            <div className="text-[9px] text-white/30 leading-relaxed pt-1 border-t border-white/5">
              Round-trip should be ~0 (lossless). High correlation = residuals are consistent across sweeps (good for model learning).
            </div>
          </div>
        )}
      </Section>

      <Section label="Data">
        <div className="grid grid-cols-2 gap-2">
          <button
            onClick={() => onModelAction('export')}
            disabled={captureCount === 0 || sfcwRunning}
            className={cn(
              'px-3 py-2 rounded-lg text-xs font-medium transition-all',
              captureCount > 0 && !sfcwRunning
                ? 'bg-white/5 border border-white/10 text-white/70 hover:bg-white/10 hover:text-white'
                : 'bg-white/2 border border-white/5 text-white/20 cursor-not-allowed'
            )}
          >
            Export
          </button>
          <button
            onClick={() => onModelAction('import')}
            disabled={sfcwRunning}
            className={cn(
              'px-3 py-2 rounded-lg text-xs font-medium transition-all',
              !sfcwRunning
                ? 'bg-white/5 border border-white/10 text-white/70 hover:bg-white/10 hover:text-white'
                : 'bg-white/2 border border-white/5 text-white/20 cursor-not-allowed'
            )}
          >
            Import
          </button>
        </div>
      </Section>

      <Section label="Model">
        <button
          onClick={() => onModelAction('build')}
          disabled={captureCount < 5 || trainingState === 'training'}
          className={cn(
            'w-full px-3 py-2.5 rounded-lg text-xs font-medium transition-all border',
            trainingState === 'training'
              ? 'bg-yellow-500/10 border-yellow-500/30 text-yellow-400'
              : captureCount >= 5
                ? 'bg-[#a78bfa]/10 border-[#a78bfa]/30 text-[#a78bfa] hover:bg-[#a78bfa]/20'
                : 'bg-white/2 border-white/5 text-white/20 cursor-not-allowed'
          )}
        >
          {trainingState === 'training' ? 'Building...' : 'Build Model'}
        </button>

        {trainingState === 'training' && trainProgress && (
          <div className="flex items-center gap-2 p-3 rounded-xl border border-yellow-500/20 bg-yellow-500/5">
            <div className="w-3 h-3 rounded-full border-2 border-yellow-400 border-t-transparent animate-spin" />
            <span className="text-[10px] text-yellow-400 capitalize">
              {trainProgress.stage === 'evaluating' ? 'Scoring leave-one-out...' : 'Building interpolant...'}
            </span>
          </div>
        )}

        {trainingState === 'error' && trainError && (
          <div className="p-3 rounded-xl border border-red-500/20 bg-red-500/5">
            <span className="text-xs text-red-400">{trainError}</span>
          </div>
        )}

        {trainingState === 'complete' && trainResult && (
          <div className="flex flex-col gap-2">
            <div className="flex flex-col gap-1 p-3 rounded-xl border border-green-500/20 bg-green-500/5">
              {trainResult.quality ? (
                <>
                  <div className="flex justify-between text-[10px]">
                    <span className="text-[#555]">Suppression (LOO)</span>
                    <span className={cn('font-mono font-bold',
                      trainResult.quality.meanSuppDb > 15 ? 'text-green-400'
                      : trainResult.quality.meanSuppDb > 8 ? 'text-yellow-400' : 'text-red-400')}>
                      {trainResult.quality.meanSuppDb.toFixed(1)} dB
                    </span>
                  </div>
                  <div className="flex justify-between text-[10px]">
                    <span className="text-[#555]">Median / worst</span>
                    <span className="font-mono text-white/70">
                      {trainResult.quality.medianSuppDb.toFixed(1)} / {trainResult.quality.worstSuppDb.toFixed(1)} dB
                    </span>
                  </div>
                  <div className="flex justify-between text-[10px]">
                    <span className="text-[#555]">Knots</span>
                    <span className="font-mono text-white">
                      {trainResult.numPositions}
                      {trainResult.mergedPositions > 0 && (
                        <span className="text-white/30"> ({trainResult.mergedPositions} merged)</span>
                      )}
                    </span>
                  </div>
                  {trainResult.quality.worstSuppDb < 8 && (
                    <div className="text-[9px] text-yellow-400/80 leading-relaxed pt-1 border-t border-white/5">
                      Weakest knot at {weakestKnotMm(trainResult.quality)} mm. A position that
                      predicts poorly from its neighbours is usually a bumped capture or an
                      oversized gap — recapture around there.
                    </div>
                  )}
                  <div className="text-[9px] text-white/30 leading-relaxed pt-1 border-t border-white/5">
                    Held-out suppression: each position predicted from the others only.
                  </div>
                </>
              ) : (
                <div className="flex justify-between text-[10px]">
                  <span className="text-[#555]">Positions</span>
                  <span className="font-mono text-white">{trainResult.numSamples}</span>
                </div>
              )}
            </div>
            <input
              type="text"
              value={modelName}
              onChange={e => setModelName(e.target.value)}
              placeholder="Model name..."
              className="w-full px-3 py-2 rounded-lg text-xs bg-[#0a0a0a] border border-white/10 text-white outline-none focus:border-[#a78bfa]/50 placeholder:text-[#333]"
              spellCheck={false}
            />
            <button
              onClick={() => {
                if (modelName.trim()) {
                  onModelAction('save_model', modelName.trim());
                  setModelName('');
                }
              }}
              disabled={!modelName.trim()}
              className={cn(
                'w-full px-3 py-2 rounded-lg text-xs font-medium transition-all border',
                modelName.trim()
                  ? 'bg-green-500/10 border-green-500/30 text-green-400 hover:bg-green-500/20'
                  : 'bg-white/2 border-white/5 text-white/20 cursor-not-allowed'
              )}
            >
              Save Model
            </button>
          </div>
        )}
      </Section>
    </>
  );
}
