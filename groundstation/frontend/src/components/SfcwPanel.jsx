import { useState, useEffect, useRef, useCallback } from 'react';
import { cn } from '@/lib/utils';
import { Section, InfoTile, ToggleButton } from './Sidebar';
import { snapSweep, masterGridMhz, QT_MASTER_STEPS_MHZ, MAX_QUICK_TUNE_PROFILES } from '@/lib/sfcwGrid';


const LIDAR_AVG_WINDOW = 20;

function fmtDeg(v) {
  return v == null || !isFinite(v) ? '—' : `${v.toFixed(1)}°`;
}

// Does a loaded model's build-time geometry still describe the current rig?
// A model is indexed by `lidar_reading - offset`, so a changed offset shifts
// every inference by that difference. The danger is not the shift itself but
// where it lands: shifted far enough, every query falls outside the model's
// captured span and silently clamps, which costs ~20 dB and past ~20 mm makes
// the subtraction add energy rather than remove it. Sweep params are checked
// for the same reason -- they change what h_cal is.
function geometryMismatch(bgModel, lidarOffsetMm, params) {
  if (!bgModel) return null;
  const g = bgModel.geometry;
  if (!g) return { unknown: true };
  const out = [];
  if (g.lidarAntennaOffsetMm != null && lidarOffsetMm != null
      && Math.abs(g.lidarAntennaOffsetMm - lidarOffsetMm) > 0.01) {
    out.push(`offset ${g.lidarAntennaOffsetMm} mm → ${lidarOffsetMm} mm`);
  }
  const gp = g.sfcwParams || {};
  // tx2Gain/rx2Gain belong here as much as any of the others: the reference gain
  // re-calibrates h_cal frequency-by-frequency (30/20 vs 50/25 measured at complex
  // coherence 0.88 / 6.5 dB suppression on an unchanged scene), so a model captured at
  // a different reference gain is invalid even though nothing about the scene moved.
  for (const k of ['startFreq', 'stopFreq', 'stepSize', 'tx1Gain', 'rx1Gain', 'tx2Gain', 'rx2Gain', 'numBuffers', 'settleCount']) {
    if (gp[k] != null && params?.[k] != null && gp[k] !== params[k]) {
      out.push(`${k} ${gp[k]} → ${params[k]}`);
    }
  }
  return out.length ? { fields: out } : null;
}

// Must match pi/radar/sfcw_engine.py: RX_BUFFER_SAMPLES (the per-channel RX
// buffer, which is the clock every settle/capture wait is expressed in) and
// DEMOD_SAMPLES (how many of those samples the demod actually correlates).
// They differ on purpose: 2048 keeps the sync_rx request an exact multiple of
// libbladeRF's DMA buffer -- measured 2048 samples/channel, and a request that
// does not divide it detaches buffer ARRIVAL time from buffer CONTENT, which
// blinds the settle gate and corrupts steps (S_repeat 34 -> 17 dB when 2000 was
// tried directly) -- while 2000 puts exactly 20.000 cycles of the 100 kHz tone
// in the demod window, so LO leakage at DC lands on an exact null of the
// rectangular window's sinc. See the long comment above RX_BUFFER_SAMPLES in
// sfcw_engine.py.
const BUFFER_SAMPLES = 2048;      // RX_BUFFER_SAMPLES -- timing
const DEMOD_SAMPLES = 2000;       // DEMOD_SAMPLES -- samples used per capture
const SAMPLE_RATE = 10_000_000;
const BUFFER_TIME_MS = (BUFFER_SAMPLES / SAMPLE_RATE) * 1000;
// Retune + demod + gate arrival phase, per step. Re-fitted 2026-09-06 against
// the Nios II/f firmware: 85.3 ms at 51 steps / settle 0 / 1 buffer with the
// 4096-sample buffer gives (85.3/51 - 2*0.4096) = 0.85 ms; the old 2.89 was
// fitted to the pre-Nios II/f retune and read 189 ms against a real 85.
const PER_STEP_OVERHEAD_MS = 0.85;

// SC16_Q11 full scale, and the fraction above which the AD9361 RX path compresses
// enough to matter. Mirrors ADC_FULL_SCALE / ADC_HOT_FRACTION_* in sfcw_engine.py --
// RX2 carries a flat CW reference so its per-sweep max represents every step, while
// RX1's max is whichever single frequency the scene happens to be strongest at, so the
// same threshold on RX1 fires on a perfectly healthy configuration. See the engine.
const ADC_FULL_SCALE = 2047;
const ADC_HOT = { rx1: 0.75, rx2: 0.40 };
// Below this the reference runs out of SNR instead -- measured 27 counts -> 29.7 dB,
// vs 45.0 dB at 169 counts. The good window is wide but it does have both edges.
const ADC_COLD_COUNTS = 60;

// Stale-standoff warning timing. LIDAR_CARRY_MS mirrors App.jsx (how long a
// last-fresh lidar reading is carried forward before the standoff goes null);
// STALE_DEBOUNCE_MS is how long the standoff must stay null before the warning
// is shown at all. Both exist because at 15 Hz sweeps a per-sweep warning
// strobes -- see the comment at the warning slot in the Standoff section.
const LIDAR_CARRY_MS = 1000;      // must match App.jsx
const STALE_DEBOUNCE_MS = 2000;

export default function SfcwPanel({ isConnected, sdrConnected, sfcwRunning, sfcwStatus, sendSdr, params, onParamsChange, coherenceResult, adcPeak, rangeScale, onRangeScaleChange, scaleRange, onScaleRangeChange, getDynamicScale, lidarMm, bgModel, bgRef, bgCapturing, onCaptureBg, onLoadBgModel, onClearBg, bgSubMode, onBgSubModeChange,
  bgDiag, bgStats, onResetBgStats, lidarProvenance, lidarOffsetMm, onLidarOffsetChange }) {
  const { startFreq, stopFreq, stepSize, numBuffers, settleCount, tx1Gain, rx1Gain, tx2Gain, rx2Gain, rangeOffset } = params;
  const [coherenceRunning, setCoherenceRunning] = useState(false);
  const lidarBuf = useRef([]);
  const [lidarAvg, setLidarAvg] = useState(null);
  const [modelList, setModelList] = useState(null);
  const [modelListOpen, setModelListOpen] = useState(false);

  // Debounce for the stale-standoff warning. Computed during render rather than
  // in an effect: lidarProvenance is state that updates on every sweep (~15 Hz),
  // so the render cadence is the clock, and a ref carries when the null run
  // began. No timer is needed -- and if sweeps stop arriving, the display
  // simply freezes in its current state, which is the right behaviour.
  const lidarStaleSinceRef = useRef(null);
  let lidarStaleVisible = false;
  if (lidarProvenance && lidarProvenance.lidar_standoff_mm === null) {
    if (lidarStaleSinceRef.current === null) lidarStaleSinceRef.current = performance.now();
    lidarStaleVisible = performance.now() - lidarStaleSinceRef.current > STALE_DEBOUNCE_MS;
  } else {
    lidarStaleSinceRef.current = null;
  }

  useEffect(() => {
    if (coherenceResult) setCoherenceRunning(false);
  }, [coherenceResult]);

  // Never stay on "Running..." forever: 100 sweeps take ~2-3 s, and the
  // server refuses the request outright while a sweep is running (its
  // 'error' reply is routed into coherenceResult by App.jsx). 30 s is far
  // beyond any legitimate run.
  useEffect(() => {
    if (!coherenceRunning) return undefined;
    const t = setTimeout(() => setCoherenceRunning(false), 30000);
    return () => clearTimeout(t);
  }, [coherenceRunning]);

  useEffect(() => {
    if (lidarMm == null) return;
    const buf = lidarBuf.current;
    buf.push(lidarMm);
    if (buf.length > LIDAR_AVG_WINDOW) buf.shift();
    const avg = buf.reduce((s, v) => s + v, 0) / buf.length;
    setLidarAvg(avg);
  }, [lidarMm]);

  const geoMismatch = geometryMismatch(bgModel, lidarOffsetMm, params);

  const update = (key, value) => {
    onParamsChange({ ...params, [key]: value });
  };

  const fetchModels = useCallback(() => {
    fetch('/api/models')
      .then(r => r.json())
      .then(data => { setModelList(Array.isArray(data) ? data : []); setModelListOpen(true); })
      .catch(() => setModelList([]));
  }, []);

  const loadModel = useCallback((filename) => {
    fetch(`/api/models/${filename}`)
      .then(r => r.json())
      .then(model => {
        onLoadBgModel(model);
        setModelListOpen(false);
      })
      .catch(err => console.error('Failed to load model:', err));
  }, [onLoadBgModel]);

  const canActivate = isConnected && sdrConnected;

  // Commit a change to start/stop/step by snapping the whole triple at once. The
  // base grid depends on the step, so changing the step can move which grid
  // start/stop belong to -- they cannot be snapped independently. This mirrors
  // SFCWEngine._apply_freq_grid exactly, so what the field shows is what runs.
  const commitSweep = (patch) => {
    const next = snapSweep(
      patch.startFreq ?? startFreq,
      patch.stopFreq ?? stopFreq,
      patch.stepSize ?? stepSize,
    );
    onParamsChange({
      ...params,
      startFreq: next.startFreq, stopFreq: next.stopFreq, stepSize: next.stepSize,
    });
    sendParams({ startFreq: next.startFreq, stopFreq: next.stopFreq, stepSize: next.stepSize });
  };

  const sendParams = (overrides = {}) => {
    sendSdr({
      cmd: 'sfcw_set_params',
      start_freq_mhz: overrides.startFreq ?? startFreq,
      stop_freq_mhz: overrides.stopFreq ?? stopFreq,
      step_size_mhz: overrides.stepSize ?? stepSize,
      num_buffers: overrides.numBuffers ?? numBuffers,
      settle_count: overrides.settleCount ?? settleCount,
      tx1_gain: overrides.tx1Gain ?? tx1Gain,
      rx1_gain: overrides.rx1Gain ?? rx1Gain,
      tx2_gain: overrides.tx2Gain ?? tx2Gain,
      rx2_gain: overrides.rx2Gain ?? rx2Gain,
      range_offset: overrides.rangeOffset ?? rangeOffset,
    });
  };

  // Amplitude scaling. The display owns the live dynamic limits, so handing
  // over to manual seeds the fields from whatever is on screen right now —
  // the colours and the Y axis must not jump on the toggle.
  const isDbScale = scaleRange.isDb !== false;
  const fmtScale = (v) => (isDbScale ? (Math.round(v * 10) / 10).toString() : Number(v).toExponential(2));
  const scaleUnit = isDbScale ? 'dB' : '';
  const scaleGap = isDbScale ? 1 : 1e-12;

  const toManualScale = () => {
    const live = getDynamicScale ? getDynamicScale() : null;
    if (!live || !isFinite(live.min) || !isFinite(live.max) || live.max <= live.min) {
      return { dynamic: false, min: scaleRange.min, max: scaleRange.max, isDb: scaleRange.isDb !== false };
    }
    return { dynamic: false, min: live.min, max: live.max, isDb: live.isDb !== false };
  };

  // Every derived number below describes the sweep the Pi will ACTUALLY run, not
  // the one that was typed. SFCWEngine.set_params snaps start/stop/step onto the
  // quick-tune master grid and never reports back, so before this the panel could
  // claim 61 steps and 1.0 m of range while the hardware swept 76 steps to 1.37 m.
  // The fields snap on commit too, so in practice these agree with what is shown —
  // this is the backstop for anything that sets params without going through them.
  const snapped = snapSweep(startFreq, stopFreq, stepSize);
  const gridAdjusted = snapped.startFreq !== startFreq
    || snapped.stopFreq !== stopFreq || snapped.stepSize !== stepSize;

  const numSteps = Math.floor((snapped.stopFreq - snapped.startFreq) / snapped.stepSize) + 1;
  const bandwidth = (snapped.stopFreq - snapped.startFreq) * 1e6;
  const rangeRes = bandwidth > 0 ? (299792458 / (2 * bandwidth)) : Infinity;
  const maxRange = snapped.stepSize > 0
    ? (299792458 / (4 * snapped.stepSize * 1e6) - rangeOffset) : Infinity;
  // "0-3m" display mode never shows past the sweep's actual unambiguous range
  // (matches sfcw_engine.py _process_h_cal's displayed_range_max) — no point
  // sizing the axis past where real data can ever land.
  const bigRangeMax = Math.max(0.5, Math.min(maxRange, 3));
  const captureTimeMs = numBuffers * BUFFER_TIME_MS;
  // Per-step time is a fixed overhead plus (1 + settleCount + numBuffers) buffer
  // periods: the leading 1 is the structural period the settle gate always waits
  // so a capture cannot straddle the retune (see sfcw_engine.py _sweep_core).
  // PER_STEP_OVERHEAD_MS is dominated by the two bladerf_schedule_retune calls
  // (~2.4 ms) plus the demod and the gate's arrival phase. Fitted to measurements
  // through the running server on 2026-09-05: settleCount 0 -> 168.3 ms/sweep and
  // settleCount 3 -> 231.1 ms at 51 steps, which this reproduces to 0.2 ms. The
  // old formula omitted the overhead entirely and so read 84 ms against a real
  // 210 ms.
  const sweepTime = numSteps * (PER_STEP_OVERHEAD_MS + (1 + settleCount + numBuffers) * BUFFER_TIME_MS) / 1000;

  return (
    <>
      {/* Sweep Range */}
      <Section label="Sweep Range">
        <div className="grid grid-cols-2 gap-2">
          <EditableField
            label="Start"
            value={startFreq}
            unit="MHz"
            onChange={(v) => commitSweep({ startFreq: v })}
            min={2000}
            max={5000}
          />
          <EditableField
            label="Stop"
            value={stopFreq}
            unit="MHz"
            onChange={(v) => commitSweep({ stopFreq: v })}
            min={2000}
            max={5000}
          />
        </div>
      </Section>

      {/* Step Configuration */}
      <Section label="Step Config">
        <div className="grid grid-cols-2 gap-2">
          <EditableField
            label="Step Size"
            value={stepSize}
            unit="MHz"
            onChange={(v) => commitSweep({ stepSize: v })}
            min={20}
            max={500}
          />
        </div>
        {/* The quick-tune master table is generated once per device connection and
            every sweep frequency has to be one of its points, so the step is not
            continuous. Snapping happens on commit, so the field always shows what
            will run -- this line explains why it may have moved. */}
        <div className="px-2 text-[9px] text-white/40 leading-relaxed">
          Steps snap to a multiple of {QT_MASTER_STEPS_MHZ.join(' or ')} MHz — the quick-tune
          master table only holds those frequencies ({masterGridMhz().length} of a
          {' '}{MAX_QUICK_TUNE_PROFILES}-profile hardware limit), and start/stop snap to the
          same family.
          {gridAdjusted && ' Your last entry was moved onto the grid.'}
        </div>
        <div className="flex flex-col gap-1">
          <div className="grid grid-cols-2 gap-2">
            <EditableField
              label="Buffers"
              value={numBuffers}
              unit="x2048 smp"
              onChange={(v) => { update('numBuffers', v); sendParams({ numBuffers: v }); }}
              min={1}
              max={64}
            />
            <EditableField
              label="Settle"
              value={settleCount}
              unit="buffers"
              onChange={(v) => { update('settleCount', v); sendParams({ settleCount: v }); }}
              min={0}
              max={30}
            />
          </div>
          <span className="text-[9px] text-[#333333] leading-tight px-1">
            {captureTimeMs.toFixed(2)} ms capture per step ({(numBuffers * DEMOD_SAMPLES).toLocaleString()} samples demodulated),
            averaged over {numBuffers} buffer{numBuffers === 1 ? '' : 's'} — after the retune the gate always
            waits one buffer so the capture cannot straddle it, plus {settleCount} buffer{settleCount === 1 ? '' : 's'} of extra settling
          </span>
        </div>
        <EditableField
          label="Range Offset"
          value={rangeOffset}
          unit="m"
          onChange={(v) => { update('rangeOffset', v); sendParams({ rangeOffset: v }); }}
          min={0}
          max={10}
        />
      </Section>

      {/* Distance */}
      <Section label="Standoff">
        <div className="flex flex-col gap-2">
          <div className="flex items-baseline justify-between px-3 py-2 rounded-xl border border-white/8 bg-[#0a0a0a]/60">
            <span className="text-[10px] font-medium uppercase tracking-wider text-[#555555]">Distance</span>
            <span className="text-base font-bold font-mono text-white">
              {lidarAvg != null ? lidarAvg.toFixed(1) : '—'} <span className="text-xs font-semibold text-[#888888]">mm</span>
            </span>
          </div>
          {/* Provenance of the standoff the last sweep actually used. `n` is
              DISTINCT lidar readings (deduped by lidar_seq), not packets --
              measured 5.3 per 250 ms sweep at the 20 Hz poll rate, vs 13.0
              packets before deduping. sigma is the spread of those samples
              (the sweep uses their mean, whose error is smaller).

              There is deliberately no "suppression ceiling" tile here. The
              obvious one -- treat sigma as a phase error on a single echo at
              12 deg/mm -- was measured on real data (2026-08-28) to be
              structurally pessimistic, because the dominant background
              component sits at path multiplier alpha ~ 0 and does not depend
              on standoff at all. At the measured sigma of 0.4 mm the true cost
              is 0.37 dB, not the ~20 dB that bound implies. Showing it would
              point every future investigation at the wrong suspect. What
              actually limits live suppression is the BG-applied block below
              (span clamping, 20.6 dB) and per-sweep SNR (5.2 dB). */}
          <div className="grid grid-cols-2 gap-2">
            <InfoTile
              label="σ"
              value={lidarProvenance?.lidar_std != null ? `${lidarProvenance.lidar_std.toFixed(2)} mm` : '—'}
            />
            <InfoTile
              label="n"
              value={lidarProvenance?.lidar_n != null ? String(lidarProvenance.lidar_n) : '—'}
            />
          </div>
          {/* Warn on a genuinely dead standoff, not on lidar_n === 0: at 15 Hz
              sweeps the sweep period is shorter than the TF-LC02's own ~60-90 ms
              update period, so individual sweeps routinely and healthily contain
              zero fresh readings (App.jsx carries the last fresh reading forward
              for up to 1 s). Two more rules, both from the warning flapping in
              practice (2026-09-06):
              - The slot below is ALWAYS rendered at a fixed height, so the
                warning appearing can never reflow the controls under it. A
                warning that shifts the layout at the sweep rate makes the whole
                panel flicker, which is worse than the condition it reports.
              - The warning is debounced: it shows only after the standoff has
                been continuously null for STALE_DEBOUNCE_MS. A brief null (a
                burst of invalid lidar returns at a bad target angle) fixes
                itself; only a sustained one is worth an operator's attention. */}
          <div className="px-2 h-4 text-[9px] leading-relaxed">
            {lidarStaleVisible && (
              <span className="text-red-400/80">
                No lidar reading for over {(LIDAR_CARRY_MS + STALE_DEBOUNCE_MS) / 1000} s — the standoff is stale.
              </span>
            )}
          </div>
          <EditableField
            label="Lidar→antenna offset"
            value={lidarOffsetMm}
            unit="mm"
            onChange={(v) => onLidarOffsetChange && onLidarOffsetChange(v)}
            min={-2000}
            max={2000}
            disabled={!onLidarOffsetChange}
          />
          {(lidarProvenance?.roll_deg != null || lidarProvenance?.pitch_deg != null) && (
            <div className="grid grid-cols-2 gap-2">
              <InfoTile label="Roll" value={fmtDeg(lidarProvenance.roll_deg)} />
              <InfoTile label="Pitch" value={fmtDeg(lidarProvenance.pitch_deg)} />
            </div>
          )}
        </div>
      </Section>

      {/* Gains */}
      <Section label="Gains">
        <div className="grid grid-cols-2 gap-2">
          <EditableField
            label="TX1"
            value={tx1Gain}
            unit="dB"
            onChange={(v) => { update('tx1Gain', v); sendParams({ tx1Gain: v }); }}
            min={0}
            max={66}
          />
          <EditableField
            label="RX1"
            value={rx1Gain}
            unit="dB"
            onChange={(v) => { update('rx1Gain', v); sendParams({ rx1Gain: v }); }}
            min={0}
            max={60}
          />
          <EditableField
            label="TX2 ref"
            value={tx2Gain}
            unit="dB"
            onChange={(v) => { update('tx2Gain', v); sendParams({ tx2Gain: v }); }}
            min={0}
            max={66}
          />
          <EditableField
            label="RX2 ref"
            value={rx2Gain}
            unit="dB"
            onChange={(v) => { update('rx2Gain', v); sendParams({ rx2Gain: v }); }}
            min={0}
            max={60}
          />
        </div>
        <span className="text-[9px] text-[#333333] leading-tight px-1">
          TX2/RX2 drive the reference loopback. h_cal divides by it, so its level sets the
          range-profile noise floor for every step at once — keep RX2 peak in the green band.
        </span>
        <AdcHeadroom adcPeak={adcPeak} />
      </Section>

      {/* Sweep Info */}
      <Section label="Sweep Info">
        <div className="grid grid-cols-2 gap-2">
          <InfoTile label="Steps" value={numSteps} />
          <InfoTile label="Sweep" value={sweepTime < 1 ? `${(sweepTime * 1000).toFixed(0)} ms` : `${sweepTime.toFixed(1)} s`} />
          <InfoTile label="Δr" value={rangeRes < 1 ? `${(rangeRes * 100).toFixed(1)} cm` : `${rangeRes.toFixed(2)} m`} />
          <InfoTile label="R max" value={maxRange < 1000 ? `${maxRange.toFixed(1)} m` : `${(maxRange / 1000).toFixed(1)} km`} />
        </div>
      </Section>

      {/* Sweep Control */}
      <Section label="Sweep">
        <ToggleButton
          active={sfcwRunning}
          canActivate={canActivate}
          onToggle={() => {
            if (sfcwRunning) { sendSdr({ cmd: 'sfcw_stop' }); return; }
            sendParams();
            sendSdr({ cmd: 'sfcw_start' });
          }}
          activeLabel="Stop Sweep"
          idleLabel="Start Sweep"
          activeSubLabel={`Sweeping ${startFreq}–${stopFreq} MHz`}
          idleSubLabel={!sdrConnected ? 'SDR not connected' : `${numSteps} steps ready`}
          color="orange"
        />
        <div className="grid grid-cols-2 gap-2 mt-2">
          <button
            onClick={onCaptureBg}
            disabled={!sfcwRunning || bgCapturing}
            className={cn(
              'px-3 py-2 rounded-lg text-xs font-medium transition-all border',
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
              'px-3 py-2 rounded-lg text-xs font-medium transition-all border',
              bgRef || bgModel || bgCapturing
                ? 'bg-white/5 border-white/10 text-white/70 hover:bg-white/10 hover:text-white'
                : 'bg-white/2 border-white/5 text-white/20 cursor-not-allowed'
            )}
          >
            Clear BG
          </button>
        </div>

        {/* Subtraction domain. Only meaningful once there is a background to subtract, so
            it appears with one. The two modes answer different questions -- see the note
            on sfcwBgSubMode in App.jsx -- and neither is a strictly better version of the
            other, which is why this is a toggle and not a setting with a right answer. */}
        {(bgRef || bgModel) && (
          <div className="mt-2 flex flex-col gap-1 px-3 py-2 rounded-xl border border-white/8 bg-[#0a0a0a]/60">
            <div className="flex items-center gap-2">
              <span className="text-[9px] uppercase tracking-wider text-white/40 shrink-0">Subtract</span>
              <div className="flex-1 grid grid-cols-2 gap-1">
                {[['complex', 'Complex'], ['magnitude', 'Magnitude']].map(([m, lbl]) => (
                  <button
                    key={m}
                    onClick={() => onBgSubModeChange(m)}
                    className={cn(
                      'px-2 py-1 rounded-md text-[10px] font-medium transition-all border',
                      bgSubMode === m
                        ? 'bg-[#4ecdc4]/15 border-[#4ecdc4]/40 text-[#4ecdc4]'
                        : 'bg-white/5 border-white/10 text-white/40 hover:text-white/70'
                    )}
                  >
                    {lbl}
                  </button>
                ))}
              </div>
            </div>
            <span className="text-[9px] text-[#333333] leading-tight">
              {bgSubMode === 'complex'
                ? 'Vector difference of h_cal — removes the wall return so a target beneath it is not buried. Needs sub-mm standoff: 1 mm = 12° at 5 GHz.'
                : 'Δ dB of the range profile vs the background — the statistic that detected the target (+4.4 dB), and it tolerates ~1 mm of standoff error. R^n and LIN are disabled: both cancel in a ratio.'}
            </span>
          </div>
        )}

        {/* Phase 0.4 -- what the subtraction ACTUALLY did on the last sweep.
            "A model is loaded" and "the model was applied" are different
            statements, and only the second one is visible here. Every path
            that declines to subtract reports its reason instead of warning to
            a console nobody has open. */}
        {bgDiag && !(bgDiag.reason === 'no background selected') && (
          <div className="mt-2 flex flex-col gap-1.5 px-3 py-2 rounded-xl border border-white/8 bg-[#0a0a0a]/60">
            <div className="flex items-baseline justify-between">
              <span className="text-[10px] font-medium uppercase tracking-wider text-[#555555]">BG applied</span>
              <span className={cn('text-xs font-bold font-mono',
                bgDiag.applied ? (bgDiag.clamped ? 'text-[#f59e0b]' : 'text-emerald-400') : 'text-red-400')}>
                {bgDiag.applied ? (bgDiag.clamped ? 'YES (CLAMPED)' : 'YES') : 'NO'}
                {bgDiag.applied && bgDiag.source ? ` · ${bgDiag.source}` : ''}
              </span>
            </div>
            {!bgDiag.applied && bgDiag.reason && (
              <div className="text-[9px] text-red-400/80 leading-relaxed">{bgDiag.reason}</div>
            )}
            {bgDiag.clamped && (
              <div className="text-[9px] text-[#f59e0b]/80 leading-relaxed">
                Standoff {bgDiag.standoffMm?.toFixed(1)} mm is {bgDiag.clampedBy?.toFixed(1)} mm outside the
                model span {bgDiag.modelSpan?.min?.toFixed(0)}–{bgDiag.modelSpan?.max?.toFixed(0)} mm.
                The model clamps, so it is subtracting a background measured at a different standoff.
              </div>
            )}
            {bgStats?.total > 0 && (
              <div className="flex items-baseline justify-between pt-1 border-t border-white/5">
                <span className="text-[9px] text-white/30">
                  clamped {bgStats.clamped}/{bgStats.total}
                  {' '}({(100 * bgStats.clamped / bgStats.total).toFixed(0)}%)
                  {bgStats.skipped > 0 && ` · skipped ${bgStats.skipped}`}
                </span>
                <button
                  onClick={onResetBgStats}
                  className="text-[9px] text-white/30 hover:text-white/60 underline underline-offset-2"
                >
                  reset
                </button>
              </div>
            )}
          </div>
        )}

        {/* Phase 0.3 -- the loaded model's build-time geometry vs the rig now. */}
        {geoMismatch && (
          <div className={cn('mt-2 px-3 py-2 rounded-xl border text-[9px] leading-relaxed',
            geoMismatch.unknown
              ? 'border-white/10 bg-[#0a0a0a]/60 text-white/35'
              : 'border-[#f59e0b]/30 bg-[#f59e0b]/5 text-[#f59e0b]/90')}>
            {geoMismatch.unknown
              ? 'This model predates geometry stamping — the lidar offset and sweep params it was built under are unknown, so a mismatch cannot be detected.'
              : <>Model was built under different settings: {geoMismatch.fields.join(', ')}. A changed lidar offset shifts every inference by that difference.</>}
          </div>
        )}

        <div className="mt-2 flex flex-col gap-2">
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
        </div>
      </Section>

      {/* Coherence Diagnostics */}
      <Section label="Coherence Test">
        <button
          onClick={() => {
            setCoherenceRunning(true);
            sendSdr({ cmd: 'sfcw_coherence_test' });
          }}
          disabled={sfcwRunning || coherenceRunning || !canActivate}
          className={cn(
            'w-full px-3 py-2 rounded-lg text-xs font-medium transition-all',
            !sfcwRunning && !coherenceRunning && canActivate
              ? 'bg-white/5 border border-white/10 text-white/70 hover:bg-white/10 hover:text-white'
              : 'bg-white/2 border border-white/5 text-white/20 cursor-not-allowed'
          )}
        >
          {coherenceRunning ? 'Running (100 sweeps)...' : 'Run Coherence Test'}
        </button>
        {coherenceResult?.error && (
          <div className="mt-2 text-[10px] text-[#f59e0b] px-1">
            {coherenceResult.error}
          </div>
        )}
        {coherenceResult && !coherenceResult.error && (
          <div className="mt-2 space-y-1">
            <div className="grid grid-cols-2 gap-2">
              <InfoTile
                label="Repeatability"
                value={coherenceResult.avg_repeatability?.toFixed(3)}
              />
              <InfoTile
                label="Correlation"
                value={coherenceResult.avg_correlation?.toFixed(3)}
              />
              <InfoTile
                label="Min corr"
                value={coherenceResult.min_correlation?.toFixed(3)}
              />
              <InfoTile
                label="S_repeat"
                value={Number.isFinite(coherenceResult.s_repeat_db)
                  ? `${coherenceResult.s_repeat_db.toFixed(1)} dB`
                  : '--'}
              />
            </div>
            <div className="text-[9px] text-[#555] px-1 space-y-0.5">
              <div>
                {coherenceResult.num_sweeps} of {coherenceResult.requested_sweeps ?? coherenceResult.num_sweeps} sweeps scored
                {coherenceResult.sweep_cores && (
                  <> ({Object.entries(coherenceResult.sweep_cores).map(([k, v]) => `${k}: ${v}`).join(', ')})</>
                )}
              </div>
              <div className="text-[#777] mt-1">
                correlation 1.0 = perfect, {'>'} 0.9 = good; S_repeat: signal over adjacent-sweep difference
              </div>
            </div>
          </div>
        )}
      </Section>

      {/* Amplitude scaling — dynamic tracks the sweep, manual pins the range
          profile's Y axis and the waterfall's colour range to fixed limits. */}
      <Section label="Amplitude Scaling">
        <button
          onClick={() => onScaleRangeChange(scaleRange.dynamic
            ? toManualScale()
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
        <div className="grid grid-cols-2 gap-2 mt-2">
          <EditableField
            label="Min"
            value={scaleRange.min}
            unit={scaleUnit}
            format={fmtScale}
            disabled={scaleRange.dynamic}
            onChange={(v) => onScaleRangeChange({ ...scaleRange, min: Math.min(v, scaleRange.max - scaleGap) })}
            min={-1e9}
            max={1e9}
          />
          <EditableField
            label="Max"
            value={scaleRange.max}
            unit={scaleUnit}
            format={fmtScale}
            disabled={scaleRange.dynamic}
            onChange={(v) => onScaleRangeChange({ ...scaleRange, max: Math.max(v, scaleRange.min + scaleGap) })}
            min={-1e9}
            max={1e9}
          />
        </div>
        <div className="px-1 mt-1 text-[9px] text-[#555555] leading-relaxed">
          {scaleRange.dynamic
            ? 'Limits track the sweep. Turn off to pin them at their current values.'
            : 'Limits pinned — range profile and waterfall both update live.'}
        </div>
      </Section>

      <Section label="Display Range">
        <button
          onClick={() => onRangeScaleChange(rangeScale && rangeScale.max === 0.5 ? { min: 0, max: bigRangeMax } : { min: 0, max: 0.5 })}
          className={cn(
            'w-full px-3 py-2 rounded-lg text-xs font-medium transition-all border',
            rangeScale && rangeScale.max === 0.5
              ? 'bg-[#D1855C]/10 border-[#D1855C]/30 text-[#D1855C]'
              : 'bg-white/5 border-white/10 text-white/70 hover:bg-white/10 hover:text-white'
          )}
        >
          {rangeScale && rangeScale.max === 0.5 ? '● 0 – 0.5 m' : `0 – ${bigRangeMax < 1 ? (bigRangeMax * 100).toFixed(0) + ' cm' : bigRangeMax.toFixed(2) + ' m'}`}
        </button>
      </Section>
    </>
  );
}

// Peak |I|,|Q| each RX reached over the last sweep, in ADC counts. This is the first
// thing to look at when sweeps get noisy: a reference above ~40% of full scale is
// compressing, which was worth 16 dB of range-profile noise floor when it was found.
function AdcHeadroom({ adcPeak }) {
  if (!adcPeak) return null;
  const full = adcPeak.full_scale || ADC_FULL_SCALE;
  const rows = [['RX1 sig', adcPeak.rx1, ADC_HOT.rx1], ['RX2 ref', adcPeak.rx2, ADC_HOT.rx2]];
  return (
    <div className="flex flex-col gap-0.5 px-1 pt-1">
      {rows.map(([label, v, hotFrac]) => {
        if (v == null) return null;
        const hot = hotFrac * full;
        const pct = (v / full) * 100;
        // Cold only matters on the reference — RX1 is however strong the scene is,
        // and a weak scene is not a misconfiguration.
        const cold = label === 'RX2 ref' && v < ADC_COLD_COUNTS;
        const isHot = v > hot;
        const tone = isHot ? 'text-[#cc4422]' : cold ? 'text-[#bb8800]' : 'text-[#227744]';
        return (
          <div key={label} className="flex items-center gap-1.5 text-[9px] leading-tight">
            <span className="text-[#333333] w-[42px] shrink-0">{label}</span>
            <div className="flex-1 h-[3px] bg-[#dddddd] relative">
              <div
                className={cn('h-full absolute left-0 top-0',
                  isHot ? 'bg-[#cc4422]' : cold ? 'bg-[#bb8800]' : 'bg-[#227744]')}
                style={{ width: `${Math.min(100, pct)}%` }}
              />
              <div className="absolute top-[-1px] bottom-[-1px] w-px bg-[#999999]"
                   style={{ left: `${hotFrac * 100}%` }} />
            </div>
            <span className={cn('tabular-nums w-[62px] text-right shrink-0', tone)}>
              {Math.round(v)} · {pct.toFixed(0)}%
            </span>
          </div>
        );
      })}
      {adcPeak.rx2 != null && adcPeak.rx2 > ADC_HOT.rx2 * full && (
        <span className="text-[9px] text-[#cc4422] leading-tight pt-0.5">
          Reference compressing — turn RX2/TX2 down. Costs up to 11 dB of noise floor.
        </span>
      )}
      {adcPeak.rx2 != null && adcPeak.rx2 < ADC_COLD_COUNTS && (
        <span className="text-[9px] text-[#bb8800] leading-tight pt-0.5">
          Reference too weak — turn RX2/TX2 up. Target 150–400 counts.
        </span>
      )}
      {adcPeak.rx1 != null && adcPeak.rx1 > ADC_HOT.rx1 * full && (
        <span className="text-[9px] text-[#cc4422] leading-tight pt-0.5">
          RX1 near full scale — signal path may be clipping. Turn RX1 down.
        </span>
      )}
    </div>
  );
}

function EditableField({ label, value, unit, onChange, min, max, disabled = false, format }) {
  const [editing, setEditing] = useState(false);
  const [draft, setDraft] = useState('');
  const committedRef = useRef(false);
  const shown = format ? format(value) : value;

  const startEdit = () => {
    setDraft(String(shown));
    setEditing(true);
    committedRef.current = false;
  };

  const commit = () => {
    if (committedRef.current) return;
    committedRef.current = true;
    const num = parseFloat(draft);
    if (!isNaN(num) && num >= min && num <= max) {
      onChange(num);
    }
    setEditing(false);
  };

  return (
    <div
      onClick={!editing && !disabled ? startEdit : undefined}
      className={cn(
        'relative flex flex-col gap-0.5 p-3 rounded-xl border',
        'transition-all duration-300',
        disabled
          ? 'border-white/5 bg-[#0a0a0a]/40 opacity-40 cursor-not-allowed'
          : editing
            ? 'border-[#D1855C]/40 bg-[#D1855C]/5 cursor-text'
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
          <span className="text-base font-bold font-mono text-white">{shown}</span>
          <span className="text-xs font-semibold text-[#888888]">{unit}</span>
        </div>
      )}
      {editing && (
        <div className="absolute bottom-0 left-3 right-3 h-px bg-gradient-to-r from-[#D1855C] to-[#E5A986] rounded-full" />
      )}
    </div>
  );
}
