import { Activity, Radio, Radar, ScanLine, Grid3x3, Map, ChevronLeft, Wifi, WifiOff, Brain, FlaskConical, Move } from 'lucide-react';
import { cn } from '@/lib/utils';
import ImuPanel from './ImuPanel';
import RfCalibPanel from './RfCalibPanel';
import BgModelPanel from './BgModelPanel';
import SfcwPanel from './SfcwPanel';
import ImagingPanel from './ImagingPanel';
import CscanPanel from './CscanPanel';
import SarPanel from './SarPanel';
import MapPanel from './MapPanel';
import RoverPanel from './RoverPanel';

const PANELS = [
  { id: 'imu',       label: 'IMU',       icon: Activity },
  { id: 'rfcalib',   label: 'RF Calibrate', icon: Radio },
  { id: 'sfcw',      label: 'SFCW',      icon: Radar },
  { id: 'imaging',   label: 'Imaging Bench', icon: FlaskConical },
  { id: 'bgmodel',   label: 'BG Model',  icon: Brain },
  { id: 'cscan',     label: 'C-Scan',    icon: ScanLine },
  { id: 'rover',     label: 'Rover Scan', icon: Move },
  { id: 'sar',       label: 'SAR',       icon: Grid3x3 },
  { id: 'map',       label: '2D Map',    icon: Map },
];

export default function Sidebar({
  isConnected,
  activePanel,
  onActivePanelChange,
  piIp,
  onPiIpChange,
  onConnect,
  onDisconnect,
  imuRate,
  imuData,
  lidarMm,
  sdrConnected,
  roverConnected,
  roverStatus,
  sendRover,
  onClearRoverTrail,
  txActive,
  rxActive,
  showFFT,
  onToggleFFT,
  graphPaused,
  onTogglePause,
  sendSdr,
  sfcwRunning,
  sfcwStatus,
  sfcwParams,
  onSfcwParamsChange,
  sfcwResult,
  sfcwBgSubMode,
  onSfcwBgSubModeChange,
  coherenceResult,
  sfcwRangeScale,
  onSfcwRangeScaleChange,
  sfcwScaleRange,
  onSfcwScaleRangeChange,
  getSfcwDynamicScale,
  sfcwBgModel,
  sfcwBgRef,
  sfcwBgCapturing,
  sfcwBgDiag,
  sfcwBgStats,
  onResetSfcwBgStats,
  sfcwLidarProvenance,
  onCaptureSfcwBg,
  onLoadSfcwBgModel,
  onClearSfcwBg,
  bscanData,
  bscanCapturing,
  bscanBgRef,
  bscanBgModel,
  bscanBgCapturing,
  onCaptureBscanBg,
  onLoadBscanBgModel,
  onClearBscanBg,
  lidarOffsetMm,
  onLidarOffsetChange,
  bgApplied,
  bscanBgSubMode,
  onBscanBgSubModeChange,
  bscanSuperFit,
  onCaptureSuperFit,
  onClearSuperFit,
  cscanSharedScale,
  cscanBgDiag,
  bscanProcParams,
  bscanCaptureProgress,
  onBgAppliedChange,
  bscanParams,
  onBscanParamsChange,
  onBscanAction,
  roverScan, roverRowStats, sweepPeriodMs,
  bscanScaleMode,
  onBscanScaleModeChange,
  bscanDisplayMode,
  onBscanDisplayModeChange,
  bscanScaleRange,
  bscanScaleScope,
  onBscanScaleScopeChange,
  bscanShowGate,
  onBscanShowGateChange,
  cscanProjection,
  onCscanProjectionChange,
  cscanProjector,
  onCscanProjectorChange,
  bscanScaleLink,
  onBscanScaleLinkChange,
  cscanRowScales,
  cscanGridScales,
  cscanLiveDiag,
  onBscanScaleRangeChange,
  sarBscanData,
  sarResult,
  sarProgress,
  sarBgEnabled,
  onSarBgEnabledChange,
  sarSvdEnabled,
  sarSvdK,
  sarSvdStrength,
  onSarSvdEnabledChange,
  onSarSvdKChange,
  onSarSvdStrengthChange,
  sarScaleMode,
  onSarScaleModeChange,
  sarAperture,
  onSarApertureChange,
  sarCoherent,
  onSarCoherentChange,
  sarDynRange,
  onSarDynRangeChange,
  sarMaxDepth,
  onSarMaxDepthChange,
  sarEpsilonR,
  onSarEpsilonRChange,
  sarWindowType,
  onSarWindowTypeChange,
  sarAutoStandoff,
  onSarAutoStandoffChange,
  sarManualStandoffMm,
  onSarManualStandoffChange,
  sarWallThickness,
  onSarWallThicknessChange,
  sarRefraction,
  onSarRefractionChange,
  sarViewMode,
  onSarViewModeChange,
  sarColormap,
  onSarColormapChange,
  mapBscanData,
  mapGateStart,
  mapGateEnd,
  onMapGateStartChange,
  onMapGateEndChange,
  mapDynRange,
  onMapDynRangeChange,
  mapMetric,
  onMapMetricChange,
  mapFocusEnabled,
  mapFocusAperture,
  onMapFocusEnabledChange,
  onMapFocusApertureChange,
  mapSvdEnabled,
  mapSvdK,
  mapSvdStrength,
  onMapSvdEnabledChange,
  onMapSvdKChange,
  onMapSvdStrengthChange,
  bgModelCaptures,
  bgModelCapturing,
  bgModelAccumCount,
  bgModelTesting,
  bgModelTestCount,
  bgModelTestResult,
  bgModelTraining,
  bgModelTrainProgress,
  bgModelTrainResult,
  bgModelTrainError,
  bgModelSweepsPerCapture,
  bgContinuousActive,
  bgContinuousStats,
  bgContBinMm,
  onBgContBinChange,
  bgContMaxSpeed,
  onBgContMaxSpeedChange,
  onBgModelSweepsChange,
  onBgModelAction,
  bgScanMode,
  onBgScanModeChange,
  bgRoverSpanMm,
  onBgRoverSpanChange,
  bgRoverStepMm,
  onBgRoverStepChange,
  bgRoverDirection,
  onBgRoverDirectionChange,
  roverBgScan,
  imagingSnapshot,
  imagingSnapshotName,
  onLoadImagingSnapshot,
  onClearImagingSnapshot,
  imagingEffect,
  onImagingEffectChange,
  imagingParams,
  onImagingParamsChange,
}) {
  return (
    <div className="flex h-screen shrink-0">

      {/* ── Icon rail ─────────────────────────────────────────────── */}
      <div className="relative flex flex-col items-center py-4 gap-1 shrink-0 w-[52px] bg-black border-r border-white/5">

        {/* Logo / connection indicator */}
        <button
          onClick={() => onActivePanelChange(null)}
          className={cn(
            'mb-2 flex items-center justify-center w-10 h-10 rounded-xl transition-all duration-500 cursor-pointer',
            isConnected && 'bg-emerald-500/15',
          )}
        >
          <span className={cn(
            'text-sm font-bold transition-opacity duration-500',
            isConnected ? 'text-emerald-400 opacity-90' : 'text-[#555] opacity-60',
          )}>v0</span>
        </button>

        <div className="w-6 h-px bg-white/5 my-1" />

        {/* Nav icons */}
        {PANELS.map(({ id, label, icon: Icon }) => {
          const isActive = activePanel === id;
          return (
            <button
              key={id}
              onClick={() => onActivePanelChange(activePanel === id ? null : id)}
              title={label}
              className={cn(
                'relative flex items-center justify-center w-10 h-10 rounded-xl',
                'transition-all duration-300 cursor-pointer',
                isActive ? 'bg-[#D1855C]/8' : 'hover:bg-white/4',
              )}
            >
              {isActive && (
                <div className="absolute left-0 w-[2px] h-5 rounded-r-full bg-gradient-to-b from-[#D1855C] to-[#E5A986]" />
              )}
              <Icon
                size={17}
                strokeWidth={isActive ? 2.2 : 1.8}
                className={isActive ? 'text-[#D1855C]' : 'text-[#555555]'}
              />
            </button>
          );
        })}

        <div className="flex-1" />

        {/* Connection toggle */}
        <button
          onClick={isConnected ? onDisconnect : onConnect}
          title={isConnected ? 'Disconnect' : 'Connect'}
          className={cn(
            'flex items-center justify-center w-8 h-8 rounded-lg transition-all cursor-pointer',
            isConnected ? 'text-emerald-400 opacity-60 hover:opacity-100' : 'text-[#555] opacity-40 hover:opacity-70',
          )}
        >
          {isConnected ? <Wifi size={14} /> : <WifiOff size={14} />}
        </button>

        {/* Collapse button */}
        {activePanel && (
          <button
            onClick={() => onActivePanelChange(null)}
            title="Collapse"
            className="flex items-center justify-center w-8 h-8 rounded-lg opacity-25 hover:opacity-60 transition-opacity cursor-pointer"
          >
            <ChevronLeft size={14} className="text-[#888888]" />
          </button>
        )}
      </div>

      {/* ── Detail panel ──────────────────────────────────────────── */}
      <div
        className="relative overflow-y-auto overflow-x-hidden bg-[#050505] border-r border-white/5 transition-all duration-300 ease-in-out"
        style={{ width: activePanel ? 276 : 0, opacity: activePanel ? 1 : 0 }}
      >
        {activePanel && (
          <div className="p-5 animate-fadeIn" style={{ minWidth: 276 }}>

            {/* Ambient glow */}
            <div className="absolute top-0 left-0 w-40 h-40 bg-[#D1855C]/5 blur-[70px] rounded-full pointer-events-none" />

            {/* Panel header */}
            <div className="relative flex items-center gap-3 mb-5">
              <div className="w-px h-3.5 bg-gradient-to-b from-[#D1855C] to-[#E5A986] rounded-full" />
              <span className="text-xs font-bold uppercase tracking-widest text-[#888888]">
                {PANELS.find(p => p.id === activePanel)?.label}
              </span>
            </div>

            {/* Connection block */}
            <div className="flex flex-col gap-6">
              <ConnectionBlock
                piIp={piIp}
                onPiIpChange={onPiIpChange}
                onConnect={onConnect}
                isConnected={isConnected}
                imuRate={imuRate}
                sdrConnected={sdrConnected}
              />

              {/* Panel-specific content */}
              {activePanel === 'imu' && (
                <ImuPanel isConnected={isConnected} imuData={imuData} />
              )}
              {activePanel === 'rfcalib' && (
                <RfCalibPanel
                  isConnected={isConnected}
                  sdrConnected={sdrConnected}
                  txActive={txActive}
                  rxActive={rxActive}
                  showFFT={showFFT}
                  onToggleFFT={onToggleFFT}
                  graphPaused={graphPaused}
                  onTogglePause={onTogglePause}
                  sendSdr={sendSdr}
                />
              )}
              {activePanel === 'bgmodel' && (
                <BgModelPanel
                  isConnected={isConnected}
                  sdrConnected={sdrConnected}
                  sfcwRunning={sfcwRunning}
                  modelCaptures={bgModelCaptures}
                  modelCapturing={bgModelCapturing}
                  accumCount={bgModelAccumCount}
                  testing={bgModelTesting}
                  testCount={bgModelTestCount}
                  testResult={bgModelTestResult}
                  trainingState={bgModelTraining}
                  trainProgress={bgModelTrainProgress}
                  trainResult={bgModelTrainResult}
                  trainError={bgModelTrainError}
                  sweepsPerCapture={bgModelSweepsPerCapture}
                  continuousActive={bgContinuousActive}
                  continuousStats={bgContinuousStats}
                  contBinMm={bgContBinMm}
                  onContBinChange={onBgContBinChange}
                  contMaxSpeed={bgContMaxSpeed}
                  onContMaxSpeedChange={onBgContMaxSpeedChange}
                  onSweepsChange={onBgModelSweepsChange}
                  stopFreq={sfcwParams.stopFreq}
                  onModelAction={onBgModelAction}
                  lidarMm={lidarMm}
                  roverConnected={roverConnected}
                  roverStatus={roverStatus}
                  bgScanMode={bgScanMode}
                  onBgScanModeChange={onBgScanModeChange}
                  bgRoverSpanMm={bgRoverSpanMm}
                  onBgRoverSpanChange={onBgRoverSpanChange}
                  bgRoverStepMm={bgRoverStepMm}
                  onBgRoverStepChange={onBgRoverStepChange}
                  bgRoverDirection={bgRoverDirection}
                  onBgRoverDirectionChange={onBgRoverDirectionChange}
                  roverBgScan={roverBgScan}
                  sendRover={sendRover}
                />
              )}
              {activePanel === 'sfcw' && (
                <SfcwPanel
                  isConnected={isConnected}
                  sdrConnected={sdrConnected}
                  sfcwRunning={sfcwRunning}
                  sfcwStatus={sfcwStatus}
                  sendSdr={sendSdr}
                  params={sfcwParams}
                  onParamsChange={onSfcwParamsChange}
                  coherenceResult={coherenceResult}
                  adcPeak={sfcwResult?.adc_peak}
                  rangeScale={sfcwRangeScale}
                  onRangeScaleChange={onSfcwRangeScaleChange}
                  scaleRange={sfcwScaleRange}
                  onScaleRangeChange={onSfcwScaleRangeChange}
                  getDynamicScale={getSfcwDynamicScale}
                  lidarMm={lidarMm}
                  bgModel={sfcwBgModel}
                  bgRef={sfcwBgRef}
                  bgCapturing={sfcwBgCapturing}
                  bgSubMode={sfcwBgSubMode}
                  onBgSubModeChange={onSfcwBgSubModeChange}
                  bgDiag={sfcwBgDiag}
                  bgStats={sfcwBgStats}
                  onResetBgStats={onResetSfcwBgStats}
                  lidarProvenance={sfcwLidarProvenance}
                  lidarOffsetMm={lidarOffsetMm}
                  onLidarOffsetChange={onLidarOffsetChange}
                  onCaptureBg={onCaptureSfcwBg}
                  onLoadBgModel={onLoadSfcwBgModel}
                  onClearBg={onClearSfcwBg}
                />
              )}
              {activePanel === 'imaging' && (
                <ImagingPanel
                  snapshot={imagingSnapshot}
                  snapshotName={imagingSnapshotName}
                  onLoadSnapshot={onLoadImagingSnapshot}
                  onClearSnapshot={onClearImagingSnapshot}
                  effect={imagingEffect}
                  onEffectChange={onImagingEffectChange}
                  params={imagingParams}
                  onParamsChange={onImagingParamsChange}
                />
              )}
              {activePanel === 'cscan' && (
                <CscanPanel
                  isConnected={isConnected}
                  sdrConnected={sdrConnected}
                  sfcwRunning={sfcwRunning}
                  scanData={bscanData}
                  scanCapturing={bscanCapturing}
                  bgApplied={bgApplied}
                  bgSubMode={bscanBgSubMode}
                  onBgSubModeChange={onBscanBgSubModeChange}
                  superFit={bscanSuperFit}
                  onCaptureSuperFit={onCaptureSuperFit}
                  onClearSuperFit={onClearSuperFit}
                  sharedScale={cscanSharedScale}
                  bgDiag={cscanBgDiag}
                  procParams={bscanProcParams}
                  captureProgress={bscanCaptureProgress}
                  onBgAppliedChange={onBgAppliedChange}
                  onScanAction={onBscanAction}
                  roverConnected={roverConnected}
                  roverStatus={roverStatus}
                  sendRover={sendRover}
                  roverScan={roverScan}
                  roverRowStats={roverRowStats}
                  sweepPeriodMs={sweepPeriodMs}
                  params={bscanParams}
                  onParamsChange={onBscanParamsChange}
                  scaleMode={bscanScaleMode}
                  onScaleModeChange={onBscanScaleModeChange}
                  displayMode={bscanDisplayMode}
                  onDisplayModeChange={onBscanDisplayModeChange}
                  scaleRange={bscanScaleRange}
                  onScaleRangeChange={onBscanScaleRangeChange}
                  scaleScope={bscanScaleScope}
                  onScaleScopeChange={onBscanScaleScopeChange}
                  rowScales={cscanRowScales}
                  gridScales={cscanGridScales}
                  liveDiag={cscanLiveDiag}
                  scaleLink={bscanScaleLink}
                  onScaleLinkChange={onBscanScaleLinkChange}
                  showGate={bscanShowGate}
                  onShowGateChange={onBscanShowGateChange}
                  projection={cscanProjection}
                  onProjectionChange={onCscanProjectionChange}
                  projector={cscanProjector}
                  onProjectorChange={onCscanProjectorChange}
                  lidarMm={lidarMm}
                  lidarOffsetMm={lidarOffsetMm}
                  bgRef={bscanBgRef}
                  bgModel={bscanBgModel}
                  bgCapturing={bscanBgCapturing}
                  onCaptureBg={onCaptureBscanBg}
                  onLoadBgModel={onLoadBscanBgModel}
                  onClearBg={onClearBscanBg}
                />
              )}
              {activePanel === 'rover' && (
                <RoverPanel
                  roverConnected={roverConnected}
                  roverStatus={roverStatus}
                  sendRover={sendRover}
                  onClearTrail={onClearRoverTrail}
                />
              )}
              {activePanel === 'sar' && (
                <SarPanel
                  bscanData={sarBscanData}
                  sarResult={sarResult}
                  sarProgress={sarProgress}
                  bgEnabled={sarBgEnabled}
                  onBgEnabledChange={onSarBgEnabledChange}
                  svdEnabled={sarSvdEnabled}
                  svdK={sarSvdK}
                  svdStrength={sarSvdStrength}
                  onSvdEnabledChange={onSarSvdEnabledChange}
                  onSvdKChange={onSarSvdKChange}
                  onSvdStrengthChange={onSarSvdStrengthChange}
                  scaleMode={sarScaleMode}
                  onScaleModeChange={onSarScaleModeChange}
                  aperture={sarAperture}
                  onApertureChange={onSarApertureChange}
                  coherent={sarCoherent}
                  onCoherentChange={onSarCoherentChange}
                  dynRange={sarDynRange}
                  onDynRangeChange={onSarDynRangeChange}
                  maxDepth={sarMaxDepth}
                  onMaxDepthChange={onSarMaxDepthChange}
                  epsilonR={sarEpsilonR}
                  onEpsilonRChange={onSarEpsilonRChange}
                  windowType={sarWindowType}
                  onWindowTypeChange={onSarWindowTypeChange}
                  autoStandoff={sarAutoStandoff}
                  onAutoStandoffChange={onSarAutoStandoffChange}
                  manualStandoffMm={sarManualStandoffMm}
                  onManualStandoffChange={onSarManualStandoffChange}
                  wallThickness={sarWallThickness}
                  onWallThicknessChange={onSarWallThicknessChange}
                  refraction={sarRefraction}
                  onRefractionChange={onSarRefractionChange}
                  viewMode={sarViewMode}
                  onViewModeChange={onSarViewModeChange}
                  colormap={sarColormap}
                  onColormapChange={onSarColormapChange}
                  onScanAction={onBscanAction}
                />
              )}
              {activePanel === 'map' && (
                <MapPanel
                  bscanData={mapBscanData}
                  gateStart={mapGateStart}
                  gateEnd={mapGateEnd}
                  onGateStartChange={onMapGateStartChange}
                  onGateEndChange={onMapGateEndChange}
                  dynRange={mapDynRange}
                  onDynRangeChange={onMapDynRangeChange}
                  metric={mapMetric}
                  onMetricChange={onMapMetricChange}
                  focusEnabled={mapFocusEnabled}
                  focusAperture={mapFocusAperture}
                  onFocusEnabledChange={onMapFocusEnabledChange}
                  onFocusApertureChange={onMapFocusApertureChange}
                  svdEnabled={mapSvdEnabled}
                  svdK={mapSvdK}
                  svdStrength={mapSvdStrength}
                  onSvdEnabledChange={onMapSvdEnabledChange}
                  onSvdKChange={onMapSvdKChange}
                  onSvdStrengthChange={onMapSvdStrengthChange}
                />
              )}
            </div>
          </div>
        )}
      </div>

    </div>
  );
}

function ConnectionBlock({ piIp, onPiIpChange, onConnect, isConnected, imuRate, sdrConnected }) {
  return (
    <Section label="Connection">
      <div className="flex flex-col gap-2">
        <div
          className={cn(
            'relative flex items-center gap-2 p-3 rounded-xl border transition-all duration-300',
            isConnected
              ? 'border-emerald-500/30 bg-emerald-500/5'
              : 'border-white/8 bg-[#0a0a0a]/60',
          )}
        >
          <input
            type="text"
            value={piIp}
            onChange={e => onPiIpChange(e.target.value)}
            onKeyDown={e => e.key === 'Enter' && onConnect()}
            placeholder="Pi IP address"
            className="flex-1 bg-transparent text-sm font-mono text-white outline-none placeholder:text-[#333]"
            spellCheck={false}
          />
          {isConnected && (
            <div className="w-1.5 h-1.5 rounded-full bg-emerald-400 animate-pulse" />
          )}
        </div>

        <div className="grid grid-cols-2 gap-2">
          <InfoTile label="IMU" value={isConnected ? `${imuRate} Hz` : '—'} />
          <InfoTile label="SDR" value={sdrConnected ? 'OK' : '—'} />
        </div>
      </div>
    </Section>
  );
}

export function Section({ label, children }) {
  return (
    <div className="flex flex-col gap-2">
      <p className="text-xs font-bold uppercase tracking-widest text-[#888888]">{label}</p>
      {children}
    </div>
  );
}

export function InfoTile({ label, value }) {
  return (
    <div className="flex flex-col gap-1 p-3 rounded-xl bg-[#0a0a0a]/50 border border-white/5">
      <span className="text-[10px] font-medium uppercase tracking-wider text-[#555555]">{label}</span>
      <span className="text-sm font-semibold text-white">{value}</span>
    </div>
  );
}

export function ToggleButton({
  active, canActivate, onToggle,
  activeLabel, idleLabel, activeSubLabel, idleSubLabel,
  color = 'orange',
}) {
  const isOrange = color === 'orange';
  const isCyan = color === 'cyan';
  const accent = isOrange ? '#D1855C' : isCyan ? '#22d3ee' : '#4aff8a';

  return (
    <button
      onClick={onToggle}
      disabled={!canActivate}
      className={cn(
        'group relative flex items-center gap-3 w-full p-4 rounded-2xl border',
        'transition-all duration-500 cursor-pointer',
        'disabled:cursor-not-allowed disabled:opacity-40',
        active
          ? isOrange
            ? 'bg-[#D1855C]/8 border-[#D1855C]/30 hover:border-[#D1855C]/50'
            : isCyan
              ? 'bg-[#22d3ee]/8 border-[#22d3ee]/30 hover:border-[#22d3ee]/50'
              : 'bg-[#4aff8a]/8 border-[#4aff8a]/30 hover:border-[#4aff8a]/50'
          : canActivate
            ? 'bg-[#0a0a0a]/50 border-white/5 hover:border-white/15 hover:bg-white/[0.03]'
            : 'bg-[#0a0a0a]/50 border-white/5',
      )}
    >
      <div
        className={cn(
          'flex items-center justify-center w-10 h-10 rounded-xl shrink-0 transition-all duration-500',
          active
            ? isOrange ? 'bg-[#D1855C]/15' : isCyan ? 'bg-[#22d3ee]/12' : 'bg-[#4aff8a]/12'
            : canActivate ? 'bg-white/5 group-hover:bg-white/8' : 'bg-white/5',
        )}
      >
        {active
          ? <div className="w-3 h-3 rounded-sm" style={{ backgroundColor: accent }} />
          : <div className="w-0 h-0 border-l-[7px] border-l-current border-y-[5px] border-y-transparent text-[#888] group-hover:text-white transition-colors" />
        }
      </div>

      <div className="flex flex-col gap-0.5 text-left min-w-0">
        <span className="text-sm font-semibold" style={{ color: active ? accent : 'white' }}>
          {active ? activeLabel : idleLabel}
        </span>
        <span className="text-xs text-[#555555] leading-relaxed">{active ? activeSubLabel : idleSubLabel}</span>
      </div>

      {active && (
        <div className="ml-auto shrink-0 flex items-center gap-1.5">
          <div className="w-1.5 h-1.5 rounded-full animate-pulse" style={{ backgroundColor: accent, boxShadow: `0 0 6px ${accent}cc` }} />
          <span className="text-[10px] font-bold uppercase tracking-wider" style={{ color: accent }}>Live</span>
        </div>
      )}
    </button>
  );
}

export function ErrorBadge({ message }) {
  return (
    <div className="flex items-start gap-2.5 p-3 rounded-xl border border-red-500/20 bg-red-500/5">
      <span className="text-xs text-red-400 leading-relaxed">{message}</span>
    </div>
  );
}
