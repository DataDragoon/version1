import { useState, useCallback, useRef, useEffect, useMemo, useReducer } from 'react';
import { useWebSocket } from './hooks/useWebSocket';
import Sidebar from './components/Sidebar';
import Viewport from './components/Viewport';
import { svdFilter } from './lib/svd';
import { useSarWorker } from './hooks/useSarWorker';
import { useBgModelWorker } from './hooks/useBgModelWorker';
import { inferBgModel } from './lib/bgModelInfer';
import { computeCaptureStats } from './lib/bgCaptureStats';
import { computeRangeProfile } from './lib/rangeProfile';
import { applyBscanBg, bgForStandoff, backgroundFor, coherentMean } from './lib/bscanBg';
import { computeSharedScale, computeRowScales, computeGridScales, bgDiagnostics } from './lib/cscanGrid';
import { cellForIndex, orderedCellForIndex, BG_STATUS, BG_STATUS_TEXT } from './lib/cscanGrid';
import { useRoverScan } from './hooks/useRoverScan';
import { DEFAULT_PARAMS as IMAGING_DEFAULT_PARAMS } from './lib/imagingEffects';

const SPEED_OF_LIGHT = 299792458;

const ROVER_TRAIL_MAX = 2000;
const ROVER_LOG_MAX = 120;

function runPhaseUnwindTest(samples, sfcwParams) {
  const startHz = sfcwParams.startFreq * 1e6;
  const stopHz = sfcwParams.stopFreq * 1e6;
  const numSteps = samples[0].num_steps;

  const freqs = [];
  for (let i = 0; i < numSteps; i++) {
    freqs.push(startHz + (i / (numSteps - 1)) * (stopHz - startHz));
  }

  const residuals = [];
  const reconstructionErrors = [];

  for (const sample of samples) {
    const d = sample.lidar_standoff_mm / 1000;
    const residualReal = new Array(numSteps);
    const residualImag = new Array(numSteps);
    let maxErr = 0;
    let sumErrSq = 0;

    for (let i = 0; i < numSteps; i++) {
      // Unwind: multiply by exp(+j * 4π * f * d / c)
      const phase = 4 * Math.PI * freqs[i] * d / SPEED_OF_LIGHT;
      const cosP = Math.cos(phase);
      const sinP = Math.sin(phase);
      const origR = sample.h_cal_real[i];
      const origI = sample.h_cal_imag[i];
      residualReal[i] = origR * cosP - origI * sinP;
      residualImag[i] = origR * sinP + origI * cosP;

      // Rewind: multiply by exp(-j * 4π * f * d / c)
      const reconR = residualReal[i] * cosP + residualImag[i] * sinP;
      const reconI = -residualReal[i] * sinP + residualImag[i] * cosP;

      const errR = reconR - origR;
      const errI = reconI - origI;
      const errMag = Math.sqrt(errR * errR + errI * errI);
      sumErrSq += errMag * errMag;
      if (errMag > maxErr) maxErr = errMag;
    }

    residuals.push({ real: residualReal, imag: residualImag, distance: d });
    reconstructionErrors.push({
      maxError: maxErr,
      rmsError: Math.sqrt(sumErrSq / numSteps),
    });
  }

  // Cross-sweep residual consistency: how similar are the 5 residuals to each other?
  const meanResidualReal = new Array(numSteps).fill(0);
  const meanResidualImag = new Array(numSteps).fill(0);
  for (const r of residuals) {
    for (let i = 0; i < numSteps; i++) {
      meanResidualReal[i] += r.real[i] / residuals.length;
      meanResidualImag[i] += r.imag[i] / residuals.length;
    }
  }

  let totalVariance = 0;
  let totalSignalPower = 0;
  for (const r of residuals) {
    for (let i = 0; i < numSteps; i++) {
      const diffR = r.real[i] - meanResidualReal[i];
      const diffI = r.imag[i] - meanResidualImag[i];
      totalVariance += diffR * diffR + diffI * diffI;
      totalSignalPower += meanResidualReal[i] ** 2 + meanResidualImag[i] ** 2;
    }
  }
  const snrLinear = totalSignalPower / (totalVariance || 1e-30);
  const snrDb = 10 * Math.log10(snrLinear);

  // Correlation between consecutive residuals
  let corrSum = 0;
  for (let k = 0; k < residuals.length - 1; k++) {
    let dotRe = 0, dotIm = 0, magA = 0, magB = 0;
    for (let i = 0; i < numSteps; i++) {
      const aR = residuals[k].real[i], aI = residuals[k].imag[i];
      const bR = residuals[k + 1].real[i], bI = residuals[k + 1].imag[i];
      dotRe += aR * bR + aI * bI;
      dotIm += aI * bR - aR * bI;
      magA += aR * aR + aI * aI;
      magB += bR * bR + bI * bI;
    }
    corrSum += Math.sqrt(dotRe * dotRe + dotIm * dotIm) / (Math.sqrt(magA) * Math.sqrt(magB) + 1e-30);
  }
  const avgCorrelation = corrSum / (residuals.length - 1);

  return {
    reconstructionErrors,
    maxErrorOverall: Math.max(...reconstructionErrors.map(e => e.maxError)),
    rmsErrorOverall: Math.sqrt(reconstructionErrors.reduce((s, e) => s + e.rmsError ** 2, 0) / reconstructionErrors.length),
    residualSnrDb: snrDb,
    residualCorrelation: avgCorrelation,
    residuals,
    freqs,
    distances: samples.map(s => s.lidar_standoff_mm),
  };
}

// Fold a background spectrum into one live sweep.
//
// Complex must write back into h_cal_real/imag: SfcwDisplay recomputes its own
// range profile from those fields, so a result that replaces only
// magnitudes/distances is silently discarded. Magnitude cannot be expressed as
// a modified h_cal at all -- a dB difference is not a spectrum -- so the
// background rides along and the display transforms both with whatever window
// it currently has and differences the results, which is the only way the two
// profiles are guaranteed to be built the same way.
function applyBgToSweep(sfcwResult, bgReal, bgImag, subMode) {
  const numSteps = sfcwResult.h_cal_real.length;
  if (subMode === 'magnitude') {
    return { ...sfcwResult, bg_h_cal_real: bgReal, bg_h_cal_imag: bgImag, bg_sub_mode: 'magnitude' };
  }
  const subReal = new Array(numSteps);
  const subImag = new Array(numSteps);
  for (let i = 0; i < numSteps; i++) {
    subReal[i] = sfcwResult.h_cal_real[i] - bgReal[i];
    subImag[i] = sfcwResult.h_cal_imag[i] - bgImag[i];
  }
  const rp = computeRangeProfile(subReal, subImag, numSteps, sfcwResult.step_size, sfcwResult.range_offset);
  return {
    ...sfcwResult,
    h_cal_real: subReal,
    h_cal_imag: subImag,
    magnitudes: rp.magnitudes,
    distances: rp.distances,
    bg_sub_mode: 'complex',
  };
}

export default function App() {
  const [activePanel, setActivePanel] = useState(null);
  const [piIp, setPiIp] = useState(() => localStorage.getItem('pi_ip') || '');

  // IMU state
  const [imuData, setImuData] = useState(null);
  const [imuRate, setImuRate] = useState(0);
  const imuCountRef = useRef(0);
  const [lidarMm, setLidarMm] = useState(null);
  // Provenance of the standoff used for the most recent sweep (Phase 0.1/0.2):
  // { lidar_standoff_mm, lidar_n, lidar_std, lidar_offset_mm, roll_deg, pitch_deg }
  const [sfcwLidarProvenance, setSfcwLidarProvenance] = useState(null);

  // Rover state. Mirrors the Pi's rover_status verbatim; the trail is the only
  // thing kept here, because the Pi has no reason to remember where it has been.
  const [roverStatus, setRoverStatus] = useState(null);
  const [roverTrail, setRoverTrail] = useState([]);
  const [roverLog, setRoverLog] = useState([]);

  // SDR / RF Calib state — antenna (TX1/RX1) and reference (TX2/RX2) both stream
  // simultaneously now, so RX preview/FFT are tracked per channel.
  const [sdrStatus, setSdrStatus] = useState(null);
  const [rxSamplesAnt, setRxSamplesAnt] = useState([]);
  const [rxSamplesRef, setRxSamplesRef] = useState([]);
  const [fftDataAnt, setFftDataAnt] = useState(null);
  const [fftDataRef, setFftDataRef] = useState(null);
  const [txActive, setTxActive] = useState(false);
  const [rxActive, setRxActive] = useState(false);
  const [showFFT, setShowFFT] = useState(true);
  const [graphPaused, setGraphPaused] = useState(false);

  // SFCW state
  const [sfcwRunning, setSfcwRunning] = useState(false);
  const [sfcwStatus, setSfcwStatus] = useState(null);
  const [sfcwResult, setSfcwResult] = useState(null);
  const [sfcwProgress, setSfcwProgress] = useState(null);
  const [coherenceResult, setCoherenceResult] = useState(null);

  // SFCW range scale ({min, max} in meters)
  const [sfcwRangeScale, setSfcwRangeScale] = useState({ min: 0, max: 3 });

  // SFCW amplitude scale. Dynamic by default; manual pins the range profile's
  // Y axis and the waterfall colour range together. `isDb` records which units
  // the pinned numbers are in — the display drops back to dynamic if its
  // dB/LIN mode changes underneath them. The live dynamic limits live in a ref
  // (the display rewrites them every frame) so the panel can seed on toggle
  // without re-rendering on every sweep.
  const [sfcwScaleRange, setSfcwScaleRange] = useState({ dynamic: true, min: -60, max: 0, isDb: true });
  const sfcwDynamicScale = useRef({ min: -60, max: 0, isDb: true });
  const handleSfcwDynamicScale = useCallback((r) => { sfcwDynamicScale.current = r; }, []);
  const getSfcwDynamicScale = useCallback(() => sfcwDynamicScale.current, []);

  // SFCW panel params (lifted so they survive panel switches)
  const [sfcwParams, setSfcwParams] = useState({
    startFreq: 2000,
    stopFreq: 5000,
    stepSize: 60,
    numBuffers: 1,
    // 0 is the minimum AND the default, and it is not "settling off": the Pi's
    // settle gate always waits one whole buffer period beyond this so a capture
    // cannot straddle the retune, and settleCount is settling on top of that.
    // Measured 2026-09-05 -- extra settling buys nothing (quick-tune fastlock is
    // long settled within a buffer period) and costs 0.41 ms per step, i.e.
    // 21 ms/sweep per unit. See pi/radar/sfcw_engine.py _sweep_core.
    settleCount: 0,
    tx1Gain: 50,
    rx1Gain: 25,
    // Reference channel (TX2 -> loopback cable -> RX2). These were previously absent
    // here, so sendSfcwParams never sent them and the Pi kept whatever SFCWEngine last
    // had -- which persists for the life of the sdr_server process, so running
    // capture_bgmodel.py or span_confirm.py once silently changed every subsequent
    // browser sweep with nothing on screen saying so. They matter twice over: the level
    // sets the range-profile noise floor (16 dB between a compressed and a well-levelled
    // reference), and the gain setting re-calibrates h_cal frequency-by-frequency, so a
    // background model is only valid at the reference gain it was captured at.
    // See CLAUDE.md "Sweep-to-sweep variability is set by the REFERENCE channel's level".
    tx2Gain: 45,
    rx2Gain: 5,
    rangeOffset: 0.5,
  });

  const sfcwParamsRef = useRef(sfcwParams);
  sfcwParamsRef.current = sfcwParams;

  // The ONLY two fields applyBscanBg reads out of sfcwParams (its freqGrid call).
  // Passing the whole object made every C-scan/SAR record re-derive -- and, through
  // sarBscanInput, restart the SAR worker -- whenever an unrelated field changed, so
  // nudging TX1 gain or Settle in the SFCW panel threw away and recomputed the entire
  // reconstruction. Provably behaviour-identical: nothing else in bscanBg.js touches
  // sfcwParams.
  const sfcwFreqParams = useMemo(
    () => ({ startFreq: sfcwParams.startFreq, stopFreq: sfcwParams.stopFreq }),
    [sfcwParams.startFreq, sfcwParams.stopFreq],
  );

  // How the background is removed. NOTE a complex/magnitude toggle was deliberately
  // deleted here once before, on the reasoning that complex was the only correct mode --
  // do not delete it again. The 2026-08-28 target A/B established that the two modes do
  // DIFFERENT jobs and both are needed:
  //   complex   -- subtracts the wall/coupling return so a target 16.6 dB below it is not
  //                buried. Coherent, so it needs sub-mm standoff accuracy: at 5 GHz 1 mm
  //                of standoff error is 12 deg of phase error.
  //   magnitude -- |current| - |reference| on the RANGE PROFILE, which is the statistic
  //                that actually detected the target (+4.4 dB at 21.2 cm against a
  //                0.23 dB control region) and which tolerated ~1 mm of standoff error
  //                where the complex difference would have been swamped by it.
  // Neither replaces the other; complex is for seeing, magnitude is for deciding.
  const [sfcwBgSubMode, setSfcwBgSubMode] = useState('complex');

  // Background Model state
  const [bgModelCaptures, setBgModelCaptures] = useState([]);
  // Positions are static, so sweeps within a capture are replicas whose only
  // job is coherent averaging. More of them buys 10*log10(N) dB of noise
  // rejection; sweeping is cheap, repositioning by hand is not.
  const [bgModelSweepsPerCapture, setBgModelSweepsPerCaptureState] = useState(
    () => Number(localStorage.getItem('bgmodel_sweeps')) || 40
  );
  const setBgModelSweepsPerCapture = useCallback((v) => {
    const n = Math.max(1, Math.min(500, Math.round(Number(v)) || 1));
    localStorage.setItem('bgmodel_sweeps', String(n));
    setBgModelSweepsPerCaptureState(n);
  }, []);
  const [bgModelCapturing, setBgModelCapturing] = useState(false);
  const [bgModelAccumCount, setBgModelAccumCount] = useState(0);
  const bgModelAccumRef = useRef(null);
  const [bgModelTesting, setBgModelTesting] = useState(false);
  const [bgModelTestCount, setBgModelTestCount] = useState(0);
  const [bgModelTestResult, setBgModelTestResult] = useState(null);
  const bgModelTestRef = useRef(null);
  const bgModelWorker = useBgModelWorker();

  // SFCW background subtraction. Two mutually exclusive sources, both applied
  // groundstation-side: a captured reference sweep, or a trained ML model.
  const [sfcwBgModel, setSfcwBgModel] = useState(null);
  const [sfcwBgRef, setSfcwBgRef] = useState(null);
  const [sfcwBgCapturing, setSfcwBgCapturing] = useState(false);
  const sfcwBgCaptureRef = useRef(false);
  const [sfcwStandoffMm, setSfcwStandoffMm] = useState(null);

  // Capturing a reference drops any loaded model, and vice versa.
  const handleSfcwCaptureBg = useCallback(() => {
    setSfcwBgModel(null);
    setSfcwBgCapturing(true);
    sfcwBgCaptureRef.current = true;
  }, []);

  const handleSfcwLoadBgModel = useCallback((model) => {
    setSfcwBgRef(null);
    sfcwBgCaptureRef.current = false;
    setSfcwBgCapturing(false);
    setSfcwBgModel(model);
  }, []);

  const handleSfcwClearBg = useCallback(() => {
    setSfcwBgModel(null);
    setSfcwBgRef(null);
    sfcwBgCaptureRef.current = false;
    setSfcwBgCapturing(false);
  }, []);

  // Imaging Bench state. Entirely offline — it reads a waterfall_snapshot
  // exported from the SFCW panel and never touches the Pi, so none of this is
  // wired to the SDR socket.
  const [imagingSnapshot, setImagingSnapshot] = useState(null);
  const [imagingSnapshotName, setImagingSnapshotName] = useState(null);
  const [imagingEffect, setImagingEffect] = useState('none');
  const [imagingParams, setImagingParams] = useState(IMAGING_DEFAULT_PARAMS);

  const handleLoadImagingSnapshot = useCallback((snap, name) => {
    setImagingSnapshot(snap);
    setImagingSnapshotName(name);
    // displayState is provenance, not processing: it seeds the "None" effect and
    // the shared range-profile knobs so the bench opens on the image the
    // operator was actually looking at, and is ignored everywhere else.
    const ds = snap.displayState || {};
    const maxRange = SPEED_OF_LIGHT / (2 * snap.common.step_size);
    setImagingParams(prev => ({
      ...prev,
      none: { ...prev.none, ...(ds.scaleMode ? { scaleMode: ds.scaleMode } : {}) },
      profile: {
        ...prev.profile,
        ...(ds.windowType ? { windowType: ds.windowType } : {}),
        ...(ds.kaiserBeta != null ? { kaiserBeta: ds.kaiserBeta } : {}),
        ...(ds.rangeComp != null ? { rangeComp: ds.rangeComp } : {}),
      },
      view: {
        ...prev.view,
        rangeMin: 0,
        rangeMax: maxRange / 2,
        sweepIndex: snap.sweeps.length - 1,
        followLatest: true,
      },
    }));
  }, []);

  const handleClearImagingSnapshot = useCallback(() => {
    setImagingSnapshot(null);
    setImagingSnapshotName(null);
  }, []);

  // B-Scan state. Background subtraction has the same two mutually exclusive
  // sources as the SFCW panel — a captured reference sweep or a trained model —
  // and both are applied groundstation-side. The Pi only ships raw h_cal.
  const [bscanData, setBscanData] = useState([]);
  const [bscanCapturing, setBscanCapturing] = useState(false);
  const [bscanBgRef, setBscanBgRef] = useState(null);
  const [bscanBgModel, setBscanBgModel] = useState(null);
  const [bscanBgCapturing, setBscanBgCapturing] = useState(false);
  const [bgApplied, setBgApplied] = useState(true);
  // The next sweep to arrive is tagged as a position / as the BG reference.
  //
  // The capture tag is an object, not a flag, because the rover raster needs to
  // say more than "capture": which cell this is (its order is not the manual
  // snake's), where the rover actually stood, and how many in-flight sweeps to
  // discard first. `null` when nothing is tagged.
  const bscanCaptureRef = useRef(null);
  const bscanBgCaptureRef = useRef(false);

  // Distance from the lidar's reference plane to the antenna aperture, so
  // standoff = lidar_reading - offset. It is a property of the mounting and
  // changes whenever the head is re-mounted, so it is user-editable and
  // persisted rather than hardcoded.
  //
  // Measured on the bench 2026-08-28: with the aperture against the wall (true
  // zero standoff) the lidar reads 164.83 mm +/- 0.68. 160 keeps a 5 mm buffer
  // so a real zero-standoff pose reports slightly positive rather than negative.
  // Re-measure after any re-mount.
  //
  // Getting this wrong is not symmetric. A constant offset error *cancels
  // exactly* for a model trained and used under that same offset: the unwind
  // factor is common to every knot, factors through the interpolation, and is
  // undone by the rewind. What it does break is the model's span. The previous
  // hardcoded 315 mm put zero standoff 150 mm behind the actual aperture, so
  // every live query against models/4th model.json (knots 11-167 mm) landed at
  // -150..0 mm -- outside the span on every single sweep, where inferInterpModel
  // silently clamps. Measured cost of that: ~20 dB, and past ~20 mm outside the
  // subtraction adds energy instead of removing it, which manufactures targets.
  // Hence the geometry stamp on new models (handleBgModelAction 'build') and the
  // out-of-span reporting in sfcwProcessed below.
  const [lidarOffsetMm, setLidarOffsetMmState] = useState(() => {
    const v = parseFloat(localStorage.getItem('lidar_antenna_offset_mm'));
    return Number.isFinite(v) ? v : 160;
  });
  const setLidarOffsetMm = useCallback((v) => {
    localStorage.setItem('lidar_antenna_offset_mm', String(v));
    setLidarOffsetMmState(v);
  }, []);
  const lidarOffsetRef = useRef(lidarOffsetMm);
  lidarOffsetRef.current = lidarOffsetMm;

  // Distinct lidar readings seen since the last sweep. Deduped by `lidar_seq`
  // (added to the Pi packet 2026-08-28): the stream broadcasts at ~50 Hz while
  // the LiDAR is polled at 20 Hz and only updates internally at ~17 Hz, so most
  // packets repeat the previous reading. Averaging the repeats would understate
  // the spread and silently weight each reading by how long it happened to be
  // held, so `lidar_n` and `lidar_std` recorded on each sweep would be fiction.
  const lidarAccumRef = useRef([]);
  const lidarLastSeqRef = useRef(null);
  // Most recent GENUINELY-FRESH reading and when it arrived. Needed because the
  // sweep period (65 ms at the 2048-sample RX buffer, 2026-09-06) is now SHORTER
  // than the TF-LC02's own update period (~60-90 ms internally, 11-17 Hz
  // measured), so whether any given sweep window contains a fresh reading is a
  // phase race -- lidar_n === 0 on a large fraction of sweeps is now the normal,
  // healthy state, not a fault. Verified live while diagnosing the flapping
  // "standoff is stale" warning: the sensor was delivering a perfectly clean
  // 16.2 fresh readings/s with zero seq gaps while the warning strobed at the
  // sweep rate. A sweep with no fresh reading falls back to this one if it is
  // younger than LIDAR_CARRY_MS; lidar_n stays 0 for that sweep (the count of
  // fresh readings is provenance and must stay honest), only the standoff is
  // carried. Older than LIDAR_CARRY_MS means the lidar has actually gone quiet
  // (several missed periods), and the standoff goes null exactly as before.
  const lidarLastFreshRef = useRef(null); // { mm, t }
  // Roll/pitch accumulated over the same window (Phase 0.2). Recorded so it is
  // possible to test later whether the background depends on pose as well as
  // standoff -- cheap to capture now, impossible to backfill.
  const poseAccumRef = useRef([]);
  // C-scan raster: a hCount x vCount grid captured along a snake path (see
  // lib/cscanGrid.js). vCount = 1 degenerates to the old single-line B-scan.
  const [bscanParams, setBscanParams] = useState({
    hCount: 20,
    hStep: 5,
    vCount: 1,
    vStep: 5,
    gateStart: 2,
    gateEnd: 70,
    metric: 'peak',
    // How the raster is driven. 'manual' is the hand-held original: the
    // operator places the head and presses Capture, snaking up from the
    // bottom-left. 'rover' hands the same grid to the gantry, which rasters it
    // from the TOP-left downwards (see lib/cscanGrid.js) with no button
    // presses. Only the capture ORDER differs -- cells, export and every
    // downstream panel are identical either way.
    scanMode: 'manual',
    // Where the head is standing right now relative to the grid origin (its
    // top-left corner), in mm. The rover drives left by the first and up by
    // the second to reach the origin before the raster starts.
    roverOriginRightMm: 0,
    roverOriginBelowMm: 0,
    // Mechanical settling allowed after a move before a sweep is taken.
    roverSettleMs: 200,
  });
  // The SDR message handler is mounted once, so it reads the grid through a ref.
  const bscanParamsRef = useRef(bscanParams);
  bscanParamsRef.current = bscanParams;

  // B-scan display toggles
  const [bscanScaleMode, setBscanScaleMode] = useState('linear');
  const [bscanDisplayMode, setBscanDisplayMode] = useState('color');
  // Colour limits: dynamic follows the data, manual pins both ends live.
  const [bscanScaleRange, setBscanScaleRange] = useState({ dynamic: true, min: -90, max: -20 });
  // Dynamic scaling scope: 'global' (one scale over the whole grid) or 'row'
  // (each grid row scaled to itself). Ignored while scaling is manual.
  const [bscanScaleScope, setBscanScaleScope] = useState('global');
  // Draw the depth gate on the B-scan pane. Purely a placement aid -- the gate
  // always drives the plan view's cell values whether or not it is drawn.
  const [bscanShowGate, setBscanShowGate] = useState(true);
  // 'linked' (both panes off one population of bins, so a colour means one dB
  // in both) or 'independent' (the plan view scales within its own gated cell
  // values). Ignored while scaling is manual, which pins both by definition.
  const [bscanScaleLink, setBscanScaleLink] = useState('linked');
  // Complex (vector) or magnitude (dB difference) subtraction. Complex is for
  // seeing -- it removes the wall so a target beneath it is not buried;
  // magnitude is for deciding -- it is the statistic the target A/B actually
  // detected with, and it tolerates ~1 mm of standoff error. See lib/bscanBg.js.
  const [bscanBgSubMode, setBscanBgSubMode] = useState('complex');

  // Super Fit: a whole previously-captured grid used as the background, matched
  // cell for cell rather than by standoff. Mutually exclusive with the other two
  // sources. Held as a lookup keyed "ix,iy" so a new scan in a different capture
  // ORDER (manual snakes up, the rover snakes down) still lines up.
  const [bscanSuperFit, setBscanSuperFit] = useState(null);
  // Sweeps banked toward the current cell, so a multi-sweep capture is not a
  // silent pause. Null when nothing is mid-capture.
  const [bscanCaptureProgress, setBscanCaptureProgress] = useState(null);

  // Processing params for the whole C-scan pipeline, driven by the controls bar
  // on the viewport's Live Sweep pane. `windowType`/`kaiserBeta` are pure
  // display -- they re-window every stored cell on change, so they can be moved
  // freely over already-captured data. `avgCount` is a CAPTURE parameter: it is
  // how many sweeps are taken at each grid cell, so it cannot change part-way
  // through a raster. `avgMode` is display again: every sweep is stored, so
  // coherent/incoherent can be flipped after the fact and the grid re-derives.
  // The bar is locked as a whole while a session runs, which keeps every cell in
  // one grid processed identically.
  const [bscanProcParams, setBscanProcParams] = useState({
    windowType: 'rectangular',
    kaiserBeta: 3,
    avgCount: 1,
    // Coherent by default here, unlike the SFCW panel's own live display:
    // incoherent averaging converges to |signal + noise| and so cannot say
    // whether anything is in a null, which is exactly what a C-scan cell asks.
    avgMode: 'coherent',
  });

  const bscanBgSource = useMemo(
    () => ({ bgRef: bscanBgRef, bgModel: bscanBgModel, superFit: bscanSuperFit }),
    [bscanBgRef, bscanBgModel, bscanSuperFit],
  );

  // Capturing a reference drops any loaded model, and vice versa.
  const handleBscanCaptureBg = useCallback(() => {
    setBscanBgModel(null);
    setBscanSuperFit(null);
    setBscanBgCapturing(true);
    bscanBgCaptureRef.current = true;
  }, []);

  const handleBscanLoadBgModel = useCallback((model) => {
    setBscanBgRef(null);
    setBscanSuperFit(null);
    bscanBgCaptureRef.current = false;
    setBscanBgCapturing(false);
    setBscanBgModel(model);
  }, []);

  const handleBscanClearBg = useCallback(() => {
    setBscanBgRef(null);
    setBscanBgModel(null);
    bscanBgCaptureRef.current = false;
    setBscanBgCapturing(false);
  }, []);

  // The background as a displayable range profile (no subtraction — raw BG),
  // drawn as one extra row on top of the B-scan pane.
  // With a model there is no single reference sweep, so it is evaluated at the
  // first captured position's standoff to give the same visual sanity check.
  //
  // Deliberately null for Super Fit: its reference is a different spectrum for
  // every cell, so there is no single row that represents it, and drawing any
  // one cell's would misrepresent the other 59. The per-cell reference is
  // visible instead by turning BG Applied off.
  const bscanBgDisplay = useMemo(() => {
    let real = null;
    let imag = null;
    let stepSize = null;
    let rangeOffset = null;
    let standoffMm = null;

    if (bscanBgModel) {
      const pos = bscanData.find(p => p.h_cal_real && p.lidar_standoff_mm != null);
      if (!pos) return null;
      const numSteps = pos.h_cal_real.length;
      const bg = bgForStandoff(bscanBgSource, pos.lidar_standoff_mm, numSteps);
      if (!bg) return null;
      real = bg.bgReal;
      imag = bg.bgImag;
      stepSize = pos.step_size;
      rangeOffset = pos.range_offset;
      standoffMm = pos.lidar_standoff_mm;
    } else if (bscanBgRef && bscanBgRef.h_cal_real && bscanBgRef.h_cal_imag) {
      real = bscanBgRef.h_cal_real;
      imag = bscanBgRef.h_cal_imag;
      stepSize = bscanBgRef.step_size;
      rangeOffset = bscanBgRef.range_offset;
      standoffMm = bscanBgRef.lidar_standoff_mm;
    } else {
      return null;
    }

    const rp = computeRangeProfile(real, imag, real.length, stepSize, rangeOffset);
    return { magnitudes: rp.magnitudes, distances: rp.distances, standoffMm, isModel: !!bscanBgModel };
  }, [bscanBgRef, bscanBgModel, bscanBgSource, bscanData, sfcwParams.startFreq, sfcwParams.stopFreq]);


  // Capture the current grid as a Super Fit reference. The grid must be full:
  // a partial reference would leave cells with no background at all, and those
  // are refused rather than silently passed through un-subtracted.
  const handleBscanCaptureSuperFit = useCallback(() => {
    const cells = {};
    let n = 0;
    for (const pos of bscanData) {
      if (pos.grid_ix == null || pos.grid_iy == null) continue;
      if (!pos.h_cal_real || !pos.h_cal_imag) continue;
      cells[`${pos.grid_ix},${pos.grid_iy}`] = {
        re: pos.h_cal_real,
        im: pos.h_cal_imag,
        standoffMm: pos.lidar_standoff_mm != null ? pos.lidar_standoff_mm : null,
      };
      n++;
    }
    if (n === 0) return;
    setBscanBgRef(null);
    setBscanBgModel(null);
    bscanBgCaptureRef.current = false;
    setBscanBgCapturing(false);
    setBscanSuperFit({
      cells,
      count: n,
      // The grid this was captured on. Editing the grid afterwards would
      // silently re-key every cell, so the panel locks the dimensions while a
      // Super Fit is loaded and these are what it locks them to.
      grid: {
        hCount: bscanParams.hCount, vCount: bscanParams.vCount,
        hStep: bscanParams.hStep, vStep: bscanParams.vStep,
      },
      capturedAt: new Date().toISOString(),
    });
  }, [bscanData, bscanParams]);

  const handleBscanClearSuperFit = useCallback(() => setBscanSuperFit(null), []);

  // B-scan processing: complex BG subtract (model, reference or Super Fit) → IFFT.
  // This is the COMPLEX-mode result, and it is what SAR and the 2D Map read --
  // SAR reconstructs from h_cal, which a magnitude difference cannot express.
  const processedBscanData = useMemo(
    () => applyBscanBg(bscanData, { enabled: bgApplied, ...bscanBgSource, mode: 'complex', ...bscanProcParams }, sfcwFreqParams),
    [bscanData, bscanBgSource, bgApplied, sfcwFreqParams, bscanProcParams],
  );

  // What the C-scan and B-scan panes draw. Identical to the above in complex
  // mode (reused rather than recomputed), and the dB-difference detector in
  // magnitude mode.
  const cscanProcessedData = useMemo(
    () => (bscanBgSubMode === 'magnitude'
      ? applyBscanBg(bscanData, { enabled: bgApplied, ...bscanBgSource, mode: 'magnitude', ...bscanProcParams }, sfcwFreqParams)
      : processedBscanData),
    [bscanBgSubMode, bscanData, bscanBgSource, bgApplied, sfcwFreqParams, processedBscanData, bscanProcParams],
  );

  // ONE colour scale for both panes, computed over the whole grid. See
  // computeSharedScale for why this is percentile-based and why it replaced the
  // per-grid / per-row limits the two displays used to compute independently.
  const cscanSharedScale = useMemo(
    () => computeSharedScale(cscanProcessedData),
    [cscanProcessedData],
  );

  // The same percentile treatment, one population per grid row. Computed
  // unconditionally rather than behind the scope toggle: it is O(bins) over the
  // grid, the same pass computeSharedScale already makes, and keeping it live
  // means flipping the toggle cannot make the colours lag a capture behind.
  const cscanRowScales = useMemo(
    () => computeRowScales(cscanProcessedData),
    [cscanProcessedData],
  );

  // The plan view's own population: one gated scalar per cell, global and
  // per-row. Depends on the gate and the metric, which the bin-domain scales do
  // not -- that asymmetry IS the unlinked mode.
  const cscanGridScales = useMemo(
    () => computeGridScales(cscanProcessedData, {
      gateStart: bscanParams.gateStart,
      gateEnd: bscanParams.gateEnd,
      metric: bscanParams.metric,
    }),
    [cscanProcessedData, bscanParams.gateStart, bscanParams.gateEnd, bscanParams.metric],
  );

  // The Live Sweep trace at the top of the C-SCAN viewport, subtracted against
  // THIS panel's background.
  //
  // It used to be handed processedSfcwResult -- the SFCW panel's background --
  // so capturing a reference or loading a model here changed the grid and the
  // B-scan under it and left the trace above them untouched, which is exactly
  // the "a background is loaded" / "the background was applied" confusion the
  // rest of this panel was instrumented to prevent.
  //
  // It resolves the source through the same backgroundFor() every grid cell
  // goes through, so the trace cannot disagree with the cell it is about to
  // become. Two things it must get right:
  //   - the STANDOFF is the live one, not a stored cell's, because that is what
  //     the next capture will be evaluated at;
  //   - Super Fit is keyed by cell, so the trace is subtracted against the
  //     reference for the cell ABOUT TO BE CAPTURED. bscanData.length is that
  //     index in both scan modes (a rover raster resumes at capturedCount, so
  //     its own index and this agree), and orderedCellForIndex maps it through
  //     whichever snake is running.
  const cscanLiveProcessed = useMemo(() => {
    const noop = (reason, status) => ({ result: sfcwResult, diag: { applied: false, reason, status } });
    if (!sfcwResult) return { result: sfcwResult, diag: null };
    if (!sfcwResult.h_cal_real || !sfcwResult.h_cal_imag) return noop('sweep carries no h_cal');
    if (!bgApplied) return noop('background subtraction is off');
    if (!bscanBgRef && !bscanBgModel && !bscanSuperFit) return noop('no background selected');

    const numSteps = sfcwResult.h_cal_real.length;
    const cell = orderedCellForIndex(
      bscanData.length, bscanParams.hCount, bscanParams.vCount, bscanParams.scanMode);
    const pos = {
      lidar_standoff_mm: sfcwStandoffMm,
      grid_ix: cell.ix,
      grid_iy: cell.iy,
    };

    const bg = backgroundFor(
      { bgRef: bscanBgRef, bgModel: bscanBgModel, superFit: bscanSuperFit }, pos, numSteps);
    if (!bg.bgReal) return noop(BG_STATUS_TEXT[bg.status] || bg.status, bg.status);

    return {
      result: applyBgToSweep(sfcwResult, bg.bgReal, bg.bgImag, bscanBgSubMode),
      diag: {
        applied: true,
        status: bg.status,
        clamped: bg.status === BG_STATUS.CLAMPED,
        source: bscanSuperFit ? 'superfit' : bscanBgModel ? 'model' : 'ref',
        mode: bscanBgSubMode,
        standoffMm: sfcwStandoffMm,
        cell,
      },
    };
  }, [sfcwResult, bgApplied, bscanBgRef, bscanBgModel, bscanSuperFit, bscanBgSubMode,
      sfcwStandoffMm, bscanData.length, bscanParams.hCount, bscanParams.vCount, bscanParams.scanMode]);

  const cscanBgDiag = useMemo(() => bgDiagnostics(cscanProcessedData), [cscanProcessedData]);

  // 2D Map state
  const [mapGateStart, setMapGateStart] = useState(2);
  const [mapGateEnd, setMapGateEnd] = useState(15);
  const [mapDynRange, setMapDynRange] = useState(30);
  const [mapMetric, setMapMetric] = useState('peak');
  const [mapFocusEnabled, setMapFocusEnabled] = useState(false);
  const [mapFocusAperture, setMapFocusAperture] = useState(7);
  const [mapSvdEnabled, setMapSvdEnabled] = useState(false);
  const [mapSvdK, setMapSvdK] = useState(1);
  const [mapSvdStrength, setMapSvdStrength] = useState(0.5);

  // SAR processing state (independent of B-scan panel)
  const [sarBgEnabled, setSarBgEnabled] = useState(true);
  const [sarSvdEnabled, setSarSvdEnabled] = useState(false);
  const [sarSvdK, setSarSvdK] = useState(1);
  const [sarSvdStrength, setSarSvdStrength] = useState(1.0);
  const [sarScaleMode, setSarScaleMode] = useState('db');
  const [sarAperture, setSarAperture] = useState(1);
  const [sarCoherent, setSarCoherent] = useState(true);
  const [sarDynRange, setSarDynRange] = useState(20);
  // How deep to reconstruct. This was `bscanParams.maxDepth`, edited from the
  // C-scan panel, where it did two unrelated jobs -- clipping the B-scan pane's
  // display (removed; that pane now draws the whole profile) and bounding SAR's
  // output grid. Only the second is a real parameter, and it belongs here: it
  // sets the extent and the cost of the reconstruction, not what a display shows.
  const [sarMaxDepth, setSarMaxDepth] = useState(70);
  // Where the per-position standoff comes from. Auto (the default) reads each cell's
  // own recorded lidar standoff, which is what makes an imprecise rig imageable --
  // corrected, focus survives 30 mm of standoff scatter; uncorrected it needs 3 mm.
  //
  // But the lidar can be WRONG rather than noisy, and no correction recovers a scan
  // from a wrong column. rebar1.json (2026-09-04) recorded ~670 mm on 32 of 43 cells
  // while its strongest scatterer sits at ~0.41 m of apparent range -- 26 cm in FRONT
  // of the claimed wall face. The lidar had been shooting past the target for most of
  // the traverse. The consequence is not subtle: reachable depth collapsed from 26 cm
  // to 2.8 cm and max coherence from 0.96 to 0.72, i.e. a depth-uniform smear.
  //
  // Turning this off substitutes one operator-entered standoff for the whole scan.
  const [sarAutoStandoff, setSarAutoStandoff] = useState(true);
  const [sarManualStandoffMm, setSarManualStandoffMm] = useState(0);
  // Relative permittivity of the wall. The SAR back-projection had NO velocity
  // parameter before 2026-09-03 and reconstructed at the speed of light in air, so
  // the hyperbola it matched had the wrong curvature and its depth axis read sqrt(er)
  // too deep. 4.5 = dry brick, cross-checked against a real scan (29 cm wall, back
  // face at 63.2 cm apparent -> er 4.68).
  const [sarEpsilonR, setSarEpsilonR] = useState(4.5);
  // Rectangular, not the Hanning that used to be hardcoded in the worker: measured
  // target coherence 0.654 rect / 0.627 kaiser b3 / 0.615 hanning on a real scan.
  const [sarWindowType, setSarWindowType] = useState('rectangular');
  // Operator-measured wall thickness, in cm. 29 is THIS bench's wall -- re-measure for
  // any other. It is what tells the layered model where the dielectric stops; 0 disables
  // the layered path entirely.
  const [sarWallThickness, setSarWallThickness] = useState(29);
  // Layered air/wall/air ray tracing with Snell at both faces, against the straight-ray
  // model that adds the standoff as a pure delay. Defaults OFF so the existing image
  // stays the A/B baseline; once the layered one is confirmed better on real data this
  // toggle should go and it becomes unconditional.
  const [sarRefraction, setSarRefraction] = useState(false);
  // 'split' = amplitude and coherence as two panes; 'combined' = one pane of amplitude
  // weighted by coherence.
  const [sarViewMode, setSarViewMode] = useState('split');
  // inferno: perceptually uniform, so a smooth gradient reads as smooth. jet's lightness
  // is not monotonic and manufactures banded structure that is not in the data -- a bad
  // property on an image whose whole question is "is that feature real". Kept selectable
  // because earlier images were read in jet. The coherence pane is NOT affected; it holds
  // its own ramp so the two split panes stay distinguishable.
  const [sarColormap, setSarColormap] = useState('inferno');

  const sarProcessedData = useMemo(
    () => applyBscanBg(bscanData, { enabled: sarBgEnabled, ...bscanBgSource }, sfcwFreqParams),
    [bscanData, bscanBgSource, sarBgEnabled, sfcwFreqParams],
  );

  // svdFilter works on `magnitudes`, which ONLY the worker's incoherent path reads --
  // the coherent path rebuilds its profiles from h_cal and runs its own complex SVD
  // there. Running it in coherent mode was a full main-thread power iteration over
  // (positions x bins) on every SVD slider move whose result was then discarded.
  const sarBscanInput = useMemo(() => {
    if (!sarSvdEnabled || sarCoherent || sarProcessedData.length < 2) return sarProcessedData;
    return svdFilter(sarProcessedData, sarSvdK, sarSvdStrength);
  }, [sarProcessedData, sarSvdEnabled, sarCoherent, sarSvdK, sarSvdStrength]);

  // SAR and the 2D Map are one-dimensional: they read the horizontal step as the
  // aperture spacing and treat the capture sequence as a line.
  //
  // Listed field by field rather than spread from bscanParams, deliberately. The spread
  // dragged in gateStart/gateEnd/metric/scanMode/roverOrigin*/roverSettleMs -- none of
  // which the worker reads -- and useSarWorker keys its debounce on this object, so
  // dragging the C-scan depth-slice gate, or the rover origin fields DURING a raster,
  // terminated and restarted the reconstruction.
  const sarParams = useMemo(() => ({
    stepSize: bscanParams.hStep,
    maxDepth: sarMaxDepth,
    aperture: sarAperture,
    coherent: sarCoherent,
    startFreq: sfcwParams.startFreq,
    svdEnabled: sarSvdEnabled,
    svdK: sarSvdK,
    svdStrength: sarSvdStrength,
    epsilonR: sarEpsilonR,
    windowType: sarWindowType,
    // The panel's third window option is labelled "Kaiser B3"; passed explicitly so it
    // is the label's value rather than the worker's fallback for a missing field.
    kaiserBeta: 3,
    wallThickness: sarWallThickness,
    refraction: sarRefraction,
    autoStandoff: sarAutoStandoff,
    manualStandoffMm: sarManualStandoffMm,
  }), [bscanParams.hStep, sarMaxDepth, sarAperture, sarCoherent, sfcwParams.startFreq, sarSvdEnabled, sarSvdK, sarSvdStrength, sarEpsilonR, sarWindowType, sarWallThickness, sarRefraction, sarAutoStandoff, sarManualStandoffMm]);
  const { sarResult, sarProgress } = useSarWorker(sarBscanInput, sarParams);

  // Max Depth auto-fit. The field is TRUE depth below the wall face, so the 70 cm
  // default asks for standoff + sqrt(4.5)*0.70 = ~157 cm of apparent range against a
  // sweep that reaches ~74 cm -- the "clipped" warning was therefore lit permanently at
  // defaults and carried no signal at all. Seed it once from the reconstruction's own
  // reachable depth, which adapts to the record AND to the current epsilon_r, then get
  // out of the way: any manual edit disarms it until the next import.
  const sarDepthAutoFitRef = useRef(true);
  useEffect(() => {
    if (!sarDepthAutoFitRef.current) return;
    if (!sarResult || !sarResult.depthClipped) return;
    // A clip caused by a standoff column that cannot be right is NOT something to fit
    // to. Fitting Max Depth down to the clipped value clears `depthClipped` on the next
    // pass, so the amber banner that announced the collapse fires once and vanishes --
    // leaving a 2 cm-deep image and nothing on screen saying why. Stay armed (do not
    // clear the ref) so the fit still happens once the standoff is corrected.
    if (sarResult.standoffSuspect) return;
    sarDepthAutoFitRef.current = false;
    // Floored, so the value it lands on cannot itself re-trip the clip.
    setSarMaxDepth(Math.max(1, Math.floor(sarResult.depthMax * 100)));
  }, [sarResult]);

  const handleSarMaxDepthChange = useCallback((v) => {
    sarDepthAutoFitRef.current = false;
    setSarMaxDepth(v);
  }, []);

  // Changing the standoff source or value moves the reachable depth, so it re-arms the
  // fit for the same reason an import does: the previous Max Depth was fitted to a
  // reachable depth that no longer applies.
  const handleSarAutoStandoffChange = useCallback((v) => {
    sarDepthAutoFitRef.current = true;
    setSarAutoStandoff(v);
  }, []);

  const handleSarManualStandoffChange = useCallback((v) => {
    sarDepthAutoFitRef.current = true;
    setSarManualStandoffMm(v);
  }, []);

  // 2D Map uses the same processed B-scan as the main B-scan panel, optionally with its own SVD
  const mapBscanData = useMemo(() => {
    if (!mapSvdEnabled || processedBscanData.length < 2) return processedBscanData;
    return svdFilter(processedBscanData, mapSvdK, mapSvdStrength);
  }, [processedBscanData, mapSvdEnabled, mapSvdK, mapSvdStrength]);

  // Apply BG subtraction to the live SFCW result: model or captured reference,
  // never both. Complex (vector) subtraction in all cases.
  //
  // Every way this can decline to subtract now reports itself (Phase 0.4).
  // Previously each one either warned to a console nobody has open or, in the
  // out-of-span case, did not check at all -- inferInterpModel silently clamps
  // to the nearest captured knot, so a standoff well outside the model's range
  // produced a confident-looking subtraction against a background measured
  // somewhere else entirely. That is indistinguishable on screen from a working
  // subtraction, and is a prime suspect for the false targets.
  const sfcwProcessed = useMemo(() => {
    const noop = (reason) => ({ result: sfcwResult, diag: { applied: false, reason } });
    if (!sfcwResult) return { result: sfcwResult, diag: null };
    if (!sfcwResult.h_cal_real || !sfcwResult.h_cal_imag) return noop('sweep carries no h_cal');

    const numSteps = sfcwResult.h_cal_real.length;
    let bgReal = null;
    let bgImag = null;
    const diag = { applied: true, reason: null, source: null, clamped: false, standoffMm: sfcwStandoffMm };

    if (sfcwBgModel) {
      diag.source = 'model';
      if (numSteps !== sfcwBgModel.sfcwParams.numSteps) {
        return noop(`numSteps mismatch: sweep ${numSteps} vs model ${sfcwBgModel.sfcwParams.numSteps}`);
      }
      if (sfcwStandoffMm == null) return noop('no lidar standoff available');
      // Out-of-span check, matching CscanPanel.jsx. The model is only valid
      // between its first and last captured knot; outside that the inference
      // clamps, so the "background" it returns is not for this standoff.
      const md = sfcwBgModel.d;
      if (Array.isArray(md) && md.length) {
        diag.modelSpan = { min: md[0], max: md[md.length - 1] };
        if (sfcwStandoffMm < md[0] || sfcwStandoffMm > md[md.length - 1]) {
          diag.clamped = true;
          diag.clampedBy = sfcwStandoffMm < md[0]
            ? md[0] - sfcwStandoffMm
            : sfcwStandoffMm - md[md.length - 1];
        }
      }
      ({ bgReal, bgImag } = inferBgModel(sfcwBgModel, sfcwStandoffMm, numSteps));
    } else if (sfcwBgRef) {
      diag.source = 'ref';
      if (sfcwBgRef.h_cal_real.length !== numSteps) {
        return noop(`numSteps mismatch: sweep ${numSteps} vs ref ${sfcwBgRef.h_cal_real.length}`);
      }
      bgReal = sfcwBgRef.h_cal_real;
      bgImag = sfcwBgRef.h_cal_imag;
    } else {
      return noop('no background selected');
    }

    diag.mode = sfcwBgSubMode;

    // Magnitude mode does its subtraction in the RANGE domain, not here: |profile| minus
    // |reference profile|. That cannot be expressed as a modified h_cal, so instead the
    // background spectrum rides along and SfcwDisplay transforms both with whatever
    // window it currently has and differences the results. Doing it that way keeps the
    // window / zero-pad / range-comp controls live and keeps both profiles built the
    // same way, which is the only way the difference means anything.
    return { result: applyBgToSweep(sfcwResult, bgReal, bgImag, sfcwBgSubMode), diag };
  }, [sfcwResult, sfcwBgModel, sfcwBgRef, sfcwStandoffMm, sfcwBgSubMode]);

  const processedSfcwResult = sfcwProcessed.result;

  // Running tally of how often the model was asked for a standoff it never saw.
  // A single clamped sweep is easy to miss; a clamp *fraction* is not, and it
  // is the difference between "the model is wrong" and "the model is being
  // asked the wrong question".
  //
  // Kept in a ref and mutated, not in state -- the same reason sfcwDynamicScale
  // is a ref: sweeps arrive at 3-6 Hz and a setState here would add a second
  // render pass to every one of them. App already re-renders per sweep (see
  // setSfcwResult below), so a snapshot taken during render is always current
  // to within one sweep, which is ample for a running tally.
  const sfcwBgStatsRef = useRef({ total: 0, clamped: 0, skipped: 0 });
  const [, bumpBgStats] = useReducer(x => x + 1, 0);
  const sfcwBgDiag = sfcwProcessed.diag;
  useEffect(() => {
    if (!sfcwBgDiag) return;
    // Only count sweeps where a background was actually selected; "no
    // background selected" is not a failure to report.
    if (!sfcwBgDiag.applied && sfcwBgDiag.reason === 'no background selected') return;
    const st = sfcwBgStatsRef.current;
    st.total += 1;
    if (sfcwBgDiag.clamped) st.clamped += 1;
    if (!sfcwBgDiag.applied) st.skipped += 1;
  }, [sfcwBgDiag]);
  const sfcwBgStats = { ...sfcwBgStatsRef.current };

  const resetSfcwBgStats = useCallback(() => {
    sfcwBgStatsRef.current = { total: 0, clamped: 0, skipped: 0 };
    bumpBgStats();  // nothing else would repaint if the sweep is stopped
  }, []);

  // IMU WebSocket
  const handleImuMessage = useCallback((msg) => {
    imuCountRef.current++;
    setImuData(msg);
    if (msg.lidar !== null && msg.lidar !== undefined) {
      setLidarMm(msg.lidar);
      // Only accumulate genuinely-new readings. A Pi without `lidar_seq`
      // (pre-2026-08-28 stream.py) reports undefined, in which case every
      // packet is accumulated -- the old behaviour, so the panel degrades
      // rather than silently recording zero samples.
      const seq = msg.lidar_seq;
      if (seq === undefined || seq === null || seq !== lidarLastSeqRef.current) {
        lidarLastSeqRef.current = seq;
        lidarAccumRef.current.push(msg.lidar);
        lidarLastFreshRef.current = { mm: msg.lidar, t: performance.now() };
      }
    }
    // Pose from the gravity vector. accel is body-frame [forward, left, up] in
    // g (see CONTEXT.md), so this is tilt only -- no yaw, which gravity cannot
    // observe. Good enough to answer "did the head tilt between captures".
    const a = msg.accel;
    if (Array.isArray(a) && a.length === 3 && a.every(v => typeof v === 'number')) {
      const [fwd, left, up] = a;
      poseAccumRef.current.push({
        roll: Math.atan2(left, up) * 180 / Math.PI,
        pitch: Math.atan2(-fwd, Math.hypot(left, up)) * 180 / Math.PI,
      });
    }
  }, []);

  const imuUrl = piIp ? `ws://${piIp}:9001` : null;
  const { status: imuStatus, connect: connectImu, disconnect: disconnectImu } = useWebSocket(imuUrl, handleImuMessage);

  // SDR WebSocket (RF Calib + SFCW share this connection)
  const handleSdrMessage = useCallback((msg) => {
    if (msg.type === 'status') {
      setSdrStatus(msg);
      setTxActive(msg.tx_active);
      setRxActive(msg.rx_active);
      if (!msg.rx_active) {
        setRxSamplesAnt([]);
        setRxSamplesRef([]);
        setFftDataAnt(null);
        setFftDataRef(null);
      }
    } else if (msg.type === 'rx_data') {
      setRxSamplesAnt(msg.antenna.i);
      setRxSamplesRef(msg.reference.i);
    } else if (msg.type === 'rx_fft') {
      const freqSpan = msg.freq_span || 2000000;
      setFftDataAnt({ magnitudes: msg.antenna.magnitudes, freq_span: freqSpan });
      setFftDataRef({ magnitudes: msg.reference.magnitudes, freq_span: freqSpan });
    } else if (msg.type === 'sfcw_status') {
      setSfcwRunning(msg.running);
      setSfcwStatus(msg);
    } else if (msg.type === 'sfcw_result') {
      // Averaged lidar standoff for this sweep, plus the provenance needed to
      // judge it: how many DISTINCT readings went into the mean and how far
      // they spread.
      //
      // Recorded for attribution, not because standoff noise is the limit --
      // it measurably is not. Scoring the same 24-position set four ways
      // (2026-08-28) put the benchmark-to-live gap at 5.3 dB, of which standoff
      // noise owns 0.37 dB and single-sweep SNR owns 5.16 dB. What these fields
      // are actually for is telling a bad standoff apart from a bad model when
      // a sweep does cancel poorly, which was previously impossible: lidar_n
      // near zero means the standoff is stale, not that the model is wrong.
      const accum = lidarAccumRef.current;
      const lidarN = accum.length;
      // No fresh reading this sweep is NORMAL at 15 Hz sweeps against an
      // 11-17 Hz lidar (see lidarLastFreshRef above) -- carry the last fresh
      // reading forward if it is recent, so the standoff (and everything that
      // consumes it, the BG-model inference especially) does not strobe
      // null/non-null at the sweep rate. lidar_n is NOT inflated by the carry.
      // ~11-16 lidar periods. Was 400 ms, raised 2026-09-06: bursts of invalid
      // reads (TF-LC02 error_code != 0 at a poor target angle -- documented at
      // 30-40% of reads on this bench) can outlast 400 ms, and the warning was
      // still flapping. Carrying a reading this old is safe for what consumes
      // it: the lidar's own zero-drift is ~1 mm over MINUTES, so a 1 s-old
      // reading on a static or slowly-moving rig is still sub-mm.
      const LIDAR_CARRY_MS = 1000;
      const fresh = lidarLastFreshRef.current;
      const carried = lidarN === 0 && fresh !== null
        && (performance.now() - fresh.t) < LIDAR_CARRY_MS;
      const avgLidarMm = lidarN > 0
        ? accum.reduce((s, v) => s + v, 0) / lidarN
        : (carried ? fresh.mm : null);
      const lidarStd = lidarN > 1
        ? Math.sqrt(accum.reduce((s, v) => s + (v - avgLidarMm) ** 2, 0) / (lidarN - 1))
        : null;
      const standoffMm = avgLidarMm !== null ? avgLidarMm - lidarOffsetRef.current : null;
      lidarAccumRef.current = [];

      const pose = poseAccumRef.current;
      const poseN = pose.length;
      const rollDeg = poseN > 0 ? pose.reduce((s, v) => s + v.roll, 0) / poseN : null;
      const pitchDeg = poseN > 0 ? pose.reduce((s, v) => s + v.pitch, 0) / poseN : null;
      poseAccumRef.current = [];

      // One object, spread into every record below, so the sweep record, the
      // C-scan cell and the BG-model sample can never drift apart.
      const provenance = {
        lidar_standoff_mm: standoffMm,
        lidar_n: lidarN,
        lidar_std: lidarStd,
        lidar_offset_mm: lidarOffsetRef.current,
        roll_deg: rollDeg,
        pitch_deg: pitchDeg,
      };
      setSfcwLidarProvenance(provenance);

      // Always update live display
      setSfcwResult(msg);
      setSfcwStandoffMm(standoffMm);

      // SFCW BG reference: first sweep after the button press becomes the reference
      if (sfcwBgCaptureRef.current && msg.h_cal_real && msg.h_cal_imag) {
        setSfcwBgRef({
          h_cal_real: [...msg.h_cal_real],
          h_cal_imag: [...msg.h_cal_imag],
          num_steps: msg.num_steps,
          step_size: msg.step_size,
          range_offset: msg.range_offset,
          ...provenance,
        });
        sfcwBgCaptureRef.current = false;
        setSfcwBgCapturing(false);
      }

      // C-scan capture: the first sweep after the button press is the cell.
      // The cell is resolved from the capture index along the snake path and
      // stored on the record, so editing the grid later never relabels it.
      if (bscanCaptureRef.current) {
        const tag = bscanCaptureRef.current;
        // A sweep already in flight when the tag was set began before the rover
        // finished settling, so it is smeared by the move itself. Results are
        // emitted serially, so discarding exactly one guarantees the sweep we
        // do keep STARTED after the settle window closed.
        if (tag.skip > 0) {
          tag.skip -= 1;
        } else if (tag.got.length + 1 < tag.need) {
          // Still filling this cell's Avg budget. Every sweep is kept, not just
          // the running mean: the coherent/incoherent choice is a DISPLAY
          // control, so it has to stay changeable against recorded data, and
          // that is only possible if the individual looks survive.
          tag.got.push({
            h_cal_real: msg.h_cal_real ? [...msg.h_cal_real] : null,
            h_cal_imag: msg.h_cal_imag ? [...msg.h_cal_imag] : null,
            timestamp: msg.timestamp,
            ...provenance,
          });
          setBscanCaptureProgress({ got: tag.got.length, need: tag.need });
        } else {
          const grid = bscanParamsRef.current;
          const sweeps = [...tag.got, {
            h_cal_real: msg.h_cal_real ? [...msg.h_cal_real] : null,
            h_cal_imag: msg.h_cal_imag ? [...msg.h_cal_imag] : null,
            timestamp: msg.timestamp,
            ...provenance,
          }];
          // Cell-level provenance is pooled over the sweeps actually taken, so a
          // multi-sweep cell reports the standoff it was really measured at
          // rather than whichever sweep happened to land last. The BG model is
          // evaluated at this number, so it matters.
          const meanSweep = coherentMean(sweeps, msg.num_steps);
          const stand = sweeps.map(w => w.lidar_standoff_mm).filter(v => v != null);
          const pooled = {
            ...provenance,
            lidar_standoff_mm: stand.length ? stand.reduce((a, b) => a + b, 0) / stand.length : null,
            lidar_n: sweeps.reduce((a, w) => a + (w.lidar_n || 0), 0),
            lidar_std: stand.length > 1
              ? Math.sqrt(stand.reduce((a, v) => a + (v - stand.reduce((x, y) => x + y, 0) / stand.length) ** 2, 0) / stand.length)
              : provenance.lidar_std,
          };
          setBscanData(prev => {
            const cell = tag.cell || cellForIndex(prev.length, grid.hCount);
            return [...prev, {
              // The Pi's own profile, kept for the export record. Nothing on
              // screen reads it -- every display recomputes from h_cal with the
              // panel's window -- but it is the only Hanning/nfft-204 version
              // that exists and it costs nothing to keep.
              magnitudes: [...msg.magnitudes],
              distances: [...msg.distances],
              sweeps,
              // The coherent mean, NOT the last sweep. Everything that reads
              // h_cal without knowing about `sweeps` -- SAR, the BG-model
              // trainer, Super Fit, svdFilter, the export -- then sees the
              // averaged cell, which is the whole point of having taken N.
              h_cal_real: meanSweep.re,
              h_cal_imag: meanSweep.im,
              num_steps: msg.num_steps,
              step_size: msg.step_size,
              range_offset: msg.range_offset,
              ...pooled,
              grid_ix: cell.ix,
              grid_iy: cell.iy,
              x_cm: cell.ix * grid.hStep,
              y_cm: cell.iy * grid.vStep,
              // Where the gantry actually stood, beside where it was asked to.
              // Slip and missed steps are the only error sources nothing can
              // observe, so the commanded target is kept next to the reported
              // position rather than assuming they agree.
              rover_x_mm: tag.rover ? tag.rover.x : null,
              rover_y_mm: tag.rover ? tag.rover.y : null,
              rover_target_x_mm: tag.target ? tag.target.x_mm : null,
              rover_target_y_mm: tag.target ? tag.target.y_mm : null,
            }];
          });
          bscanCaptureRef.current = null;
          setBscanCapturing(false);
          setBscanCaptureProgress(null);
        }
      }

      // B-scan BG reference: likewise tagged groundstation-side
      if (bscanBgCaptureRef.current && msg.h_cal_real && msg.h_cal_imag) {
        setBscanBgRef({
          h_cal_real: [...msg.h_cal_real],
          h_cal_imag: [...msg.h_cal_imag],
          num_steps: msg.num_steps,
          step_size: msg.step_size,
          range_offset: msg.range_offset,
          ...provenance,
        });
        bscanBgCaptureRef.current = false;
        setBscanBgCapturing(false);
      }
      if (bgModelAccumRef.current) {
        const accum = bgModelAccumRef.current;
        accum.samples.push({
          h_cal_real: msg.h_cal_real ? [...msg.h_cal_real] : null,
          h_cal_imag: msg.h_cal_imag ? [...msg.h_cal_imag] : null,
          ...provenance,
          num_steps: msg.num_steps,
          step_size: msg.step_size,
          range_offset: msg.range_offset,
          timestamp: msg.timestamp,
        });
        setBgModelAccumCount(accum.samples.length);

        if (accum.samples.length >= accum.target) {
          const capture = { samples: accum.samples, stats: computeCaptureStats(accum.samples) };
          setBgModelCaptures(prev => [...prev, capture]);
          bgModelAccumRef.current = null;
          setBgModelCapturing(false);
          setBgModelAccumCount(0);
        }
      }
      if (bgModelTestRef.current) {
        const test = bgModelTestRef.current;
        test.samples.push({
          h_cal_real: msg.h_cal_real ? [...msg.h_cal_real] : null,
          h_cal_imag: msg.h_cal_imag ? [...msg.h_cal_imag] : null,
          ...provenance,
          num_steps: msg.num_steps,
          step_size: msg.step_size,
        });
        setBgModelTestCount(test.samples.length);

        if (test.samples.length >= 5) {
          const result = runPhaseUnwindTest(test.samples, sfcwParamsRef.current);
          setBgModelTestResult(result);
          bgModelTestRef.current = null;
          setBgModelTesting(false);
          setBgModelTestCount(0);
        }
      }
      setSfcwProgress(null);
    } else if (msg.type === 'sfcw_progress') {
      setSfcwProgress(msg);
    } else if (msg.type === 'sfcw_error') {
      setSfcwRunning(false);
      setSfcwProgress(null);
      bscanCaptureRef.current = null;
      bscanBgCaptureRef.current = false;
      setBscanCapturing(false);
      setBscanBgCapturing(false);
    } else if (msg.type === 'coherence_result') {
      setCoherenceResult(msg);
      setSfcwRunning(false);
    }
  }, []);

  const sdrUrl = piIp ? `ws://${piIp}:9003` : null;
  const { status: sdrConnectionStatus, send: sendSdr, connect: connectSdr, disconnect: disconnectSdr } = useWebSocket(sdrUrl, handleSdrMessage);

  // Rover WebSocket (port 9002). The Pi is the only place rover position lives
  // -- it dead-reckons from what it commanded and we mirror it, so a browser
  // reload or a second tab cannot desynchronise the position from the rig.
  const handleRoverMessage = useCallback((msg) => {
    if (msg.type === 'rover_status') {
      setRoverStatus(msg);
      setRoverTrail(prev => {
        const last = prev[prev.length - 1];
        if (last && last.x === msg.x_mm && last.y === msg.y_mm) return prev;
        const next = prev.length >= ROVER_TRAIL_MAX
          ? prev.slice(prev.length - ROVER_TRAIL_MAX + 1)
          : prev.slice();
        next.push({ x: msg.x_mm, y: msg.y_mm });
        return next;
      });
    } else if (msg.type === 'rover_log') {
      setRoverLog(prev => [...prev.slice(-(ROVER_LOG_MAX - 1)), msg]);
    } else if (msg.type === 'rover_log_history') {
      setRoverLog((msg.lines || []).slice(-ROVER_LOG_MAX));
    } else if (msg.type === 'rover_error') {
      setRoverLog(prev => [...prev.slice(-(ROVER_LOG_MAX - 1)),
                           { t: Date.now() / 1000, line: `error: ${msg.message}` }]);
    }
  }, []);

  const roverUrl = piIp ? `ws://${piIp}:9002` : null;
  const { status: roverConnectionStatus, send: sendRover, connect: connectRover, disconnect: disconnectRover } = useWebSocket(roverUrl, handleRoverMessage);

  const clearRoverTrail = useCallback(() => setRoverTrail([]), []);

  // The Pi keeps its own SFCW defaults, so anything the panel shows is a guess
  // until we push. Sync on connect and again before every sweep, so the panel is
  // always the source of truth and the two sides cannot drift apart silently.
  const sendSfcwParams = useCallback(() => {
    const p = sfcwParamsRef.current;
    sendSdr({
      cmd: 'sfcw_set_params',
      start_freq_mhz: p.startFreq,
      stop_freq_mhz: p.stopFreq,
      step_size_mhz: p.stepSize,
      num_buffers: p.numBuffers,
      settle_count: p.settleCount,
      tx1_gain: p.tx1Gain,
      rx1_gain: p.rx1Gain,
      tx2_gain: p.tx2Gain,
      rx2_gain: p.rx2Gain,
      range_offset: p.rangeOffset,
    });
  }, [sendSdr]);

  useEffect(() => {
    if (sdrConnectionStatus === 'connected') sendSfcwParams();
  }, [sdrConnectionStatus, sendSfcwParams]);

  // ── Rover-driven C-scan raster ─────────────────────────────────────────
  //
  // The gantry replaces the operator's finger on Capture and nothing else: the
  // sweep, the standoff provenance and the background subtraction are all the
  // manual path's, unchanged. What the automation adds is where the head is
  // when each sweep is taken.
  const startSfcwSweep = useCallback(() => {
    if (sfcwRunning) return;
    sendSfcwParams();
    sendSdr({ cmd: 'sfcw_start' });
  }, [sfcwRunning, sendSfcwParams, sendSdr]);

  const stopSfcwSweep = useCallback(() => {
    sendSdr({ cmd: 'sfcw_stop' });
  }, [sendSdr]);

  // Tags the sweep after next as this cell -- `skip: 1` drops the one already
  // in flight, which began while the rover was still settling.
  const requestRoverCapture = useCallback((cell, rover, target) => {
    bscanCaptureRef.current = {
      cell, rover, target, skip: 1,
      need: Math.max(1, bscanProcParams.avgCount), got: [],
    };
    setBscanCapturing(true);
  }, [bscanProcParams.avgCount]);

  const roverScan = useRoverScan({
    params: bscanParams,
    roverStatus,
    roverConnected: roverConnectionStatus === 'connected',
    sendRover,
    sfcwRunning,
    onStartSweep: startSfcwSweep,
    onStopSweep: stopSfcwSweep,
    capturedCount: bscanData.length,
    onRequestCapture: requestRoverCapture,
    // A cell now takes avgCount sweeps, so the capture watchdog has to scale
    // with it or a high Avg would trip the timeout before the cell completes.
    sweepsPerCell: Math.max(1, bscanProcParams.avgCount),
  });

  // A raster that ends -- completed, stopped or failed -- must not leave a tag
  // armed, or the next sweep would be captured into a cell nobody asked for.
  const roverScanActive = roverScan.active;
  useEffect(() => {
    if (roverScanActive) return;
    if (bscanCaptureRef.current && bscanCaptureRef.current.cell) {
      bscanCaptureRef.current = null;
      setBscanCapturing(false);
    }
  }, [roverScanActive]);

  const handleBscanAction = useCallback((action) => {
    if (action === 'start_session') {
      // In rover mode the session ARMS the raster: it starts the sweep and
      // drives to the grid origin, then parks there. The raster itself is
      // 'start_raster' below, so the operator gets a window at a known position
      // -- sweeping -- to capture a background reference before anything moves.
      // Manual mode is untouched.
      if (bscanParams.scanMode === 'rover') {
        roverScan.start();
        return;
      }
      if (sfcwRunning) return;
      sendSfcwParams();
      sendSdr({ cmd: 'sfcw_start' });
    } else if (action === 'start_raster') {
      // Second half of the rover start; a no-op unless parked at the origin.
      if (bscanParams.scanMode === 'rover') roverScan.beginRaster();
    } else if (action === 'stop_session') {
      // Stopping a rover raster is an emergency stop, deliberately: it is the
      // only control on screen while the gantry is moving on its own.
      if (bscanParams.scanMode === 'rover') {
        roverScan.stop();
        return;
      }
      sendSdr({ cmd: 'sfcw_stop' });
    } else if (action === 'add_scan') {
      // The Pi has no notion of a B-scan; the next sweep it sends is the capture.
      // No cell: the manual raster resolves it from the capture index.
      setBscanCapturing(true);
      bscanCaptureRef.current = {
        cell: null, rover: null, target: null, skip: 0,
        need: Math.max(1, bscanProcParams.avgCount), got: [],
      };
    } else if (action === 'new') {
      setBscanData([]);
    } else if (action === 'undo') {
      setBscanData(prev => prev.slice(0, -1));
    } else if (action === 'export') {
      const exportData = {
        // v7 adds per-position `sweeps` (every look taken at that cell, so the
        // coherent/incoherent choice stays live after import) and `procParams`.
        version: 7,
        procParams: bscanProcParams,
        timestamp: new Date().toISOString(),
        params: bscanParams,
        sfcwParams: sfcwParams,
        lidarAntennaOffsetMm: lidarOffsetMm,
        data: bscanData,
        bgRef: bscanBgRef,
        bgModelName: bscanBgModel ? (bscanBgModel.name || null) : null,
      };
      const blob = new Blob([JSON.stringify(exportData)], { type: 'application/json' });
      const url = URL.createObjectURL(blob);
      const a = document.createElement('a');
      a.href = url;
      a.download = `cscan_${new Date().toISOString().replace(/[:.]/g, '-')}.json`;
      a.click();
      URL.revokeObjectURL(url);
    } else if (action === 'import') {
      const input = document.createElement('input');
      input.type = 'file';
      input.accept = '.json';
      input.onchange = (e) => {
        const file = e.target.files[0];
        if (!file) return;
        const reader = new FileReader();
        reader.onload = (ev) => {
          try {
            const imported = JSON.parse(ev.target.result);
            if (imported.data && Array.isArray(imported.data)) {
              setBscanData(imported.data);
              // A fresh scan gets one shot at fitting Max Depth to what it can reach.
              sarDepthAutoFitRef.current = true;
              // The FREQUENCY PLAN only -- deliberately not the whole sfcwParams. SAR
              // phase-compensates with startFreq and applyBscanBg builds its freq axis
              // from start/stop, and both were silently taking whatever the panel
              // happened to be set to: measured, importing a 2000 MHz scan while the
              // panel read 2500 moved the amplitude peak 7 pixels and changed its level
              // 1.9 dB. Gains, settle and buffers are NOT restored, because
              // sendSfcwParams() pushes the full set before every sfcw_start, so
              // restoring them would silently re-gain the radio off an old file --
              // exactly the trap CLAUDE.md documents for capture_bgmodel.py.
              if (imported.sfcwParams) {
                const { startFreq, stopFreq, stepSize: fStep, rangeOffset } = imported.sfcwParams;
                setSfcwParams(prev => ({
                  ...prev,
                  ...(startFreq != null && { startFreq }),
                  ...(stopFreq != null && { stopFreq }),
                  ...(fStep != null && { stepSize: fStep }),
                  ...(rangeOffset != null && { rangeOffset }),
                }));
              }
              // Window and averaging MODE are display choices and are restored so
              // the import opens on the image it was exported as. avgCount is a
              // CAPTURE parameter -- the sweeps are already in the file and the
              // number of them is whatever was taken, so it is read back from
              // the data rather than trusted from the header.
              if (imported.procParams) {
                const got = imported.data.find(d => Array.isArray(d.sweeps) && d.sweeps.length);
                setBscanProcParams(prev => ({
                  ...prev,
                  ...(imported.procParams.windowType && { windowType: imported.procParams.windowType }),
                  ...(imported.procParams.kaiserBeta != null && { kaiserBeta: imported.procParams.kaiserBeta }),
                  ...(imported.procParams.avgMode && { avgMode: imported.procParams.avgMode }),
                  avgCount: got ? got.sweeps.length : 1,
                }));
              }
              if (imported.bgRef) {
                setBscanBgModel(null);
                setBscanBgRef(imported.bgRef);
              }
              if (imported.params) {
                // v3 and earlier stored the depth extent as a wall thickness;
                // v4 and earlier were a single line (stepSize / numPositions),
                // which maps onto a one-row grid.
                const {
                  stepSize, numPositions, maxDepth, wallThickness,
                  hCount, hStep, vCount, vStep, gateStart, gateEnd, metric,
                } = imported.params;
                // v3 and earlier called it wallThickness. It is no longer a
                // C-scan parameter at all -- it only ever bounded SAR's
                // reconstruction -- so it is restored there instead.
                const depth = maxDepth != null ? maxDepth : wallThickness;
                if (depth != null) setSarMaxDepth(depth);
                setBscanParams(prev => ({
                  ...prev,
                  ...(stepSize != null && { hStep: stepSize }),
                  ...(numPositions != null && { hCount: numPositions, vCount: 1 }),
                  ...(hCount != null && { hCount }),
                  ...(hStep != null && { hStep }),
                  ...(vCount != null && { vCount }),
                  ...(vStep != null && { vStep }),
                  ...(gateStart != null && { gateStart }),
                  ...(gateEnd != null && { gateEnd }),
                  ...(metric != null && { metric }),
                }));
              }
            }
          } catch (err) {
            console.error('Failed to import B-scan:', err);
          }
        };
        reader.readAsText(file);
      };
      input.click();
    }
  }, [sendSdr, sendSfcwParams, sfcwRunning, bscanData, bscanParams, sfcwParams, bscanBgRef, bscanBgModel, roverScan, lidarOffsetMm]);

  const handleBgModelAction = useCallback((action, payload) => {
    if (action === 'start_session') {
      if (sfcwRunning) return;
      sendSfcwParams();
      sendSdr({ cmd: 'sfcw_start' });
    } else if (action === 'stop_session') {
      sendSdr({ cmd: 'sfcw_stop' });
    } else if (action === 'capture') {
      setBgModelCapturing(true);
      bgModelAccumRef.current = { samples: [], target: bgModelSweepsPerCapture };
    } else if (action === 'undo') {
      setBgModelCaptures(prev => prev.slice(0, -1));
    } else if (action === 'clear') {
      setBgModelCaptures([]);
    } else if (action === 'build') {
      // One training sample per position, using the coherent mean across that
      // position's sweeps. The replicas are the same standoff measured N times,
      // so feeding them individually adds no information — it just costs N x the
      // epochs and regresses to this mean anyway.
      const allSamples = bgModelCaptures.map(c => (
        c.stats
          ? {
              h_cal_real: c.stats.h_mean_real,
              h_cal_imag: c.stats.h_mean_imag,
              lidar_standoff_mm: c.stats.standoffMm,
              num_steps: c.stats.numSteps,
            }
          : c.samples[0]
      )).filter(s => s && s.h_cal_real && s.lidar_standoff_mm != null);
      if (allSamples.length < 5) return;
      // Geometry stamp. The model is a function of standoff, and standoff is
      // `lidar_reading - lidarOffsetMm`, so a model is only valid under the
      // offset it was built with -- a change between training and inference
      // shifts every query, and shifted past the captured span it clamps
      // silently (~20 dB, and worse than useless past ~20 mm out). The full
      // sfcwParams go along because gains, step size and settle count all
      // change what h_cal actually is.
      bgModelWorker.startTraining(allSamples, sfcwParams, {
        geometry: {
          lidarAntennaOffsetMm: lidarOffsetMm,
          sfcwParams: { ...sfcwParams },
          builtAt: new Date().toISOString(),
        },
      });
    } else if (action === 'save_model') {
      if (!bgModelWorker.resultRef.current || !payload) return;
      const modelData = {
        ...bgModelWorker.resultRef.current,
        name: payload,
        created: new Date().toISOString(),
      };
      fetch('/api/models', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(modelData),
      }).then(r => r.json()).then(res => {
        if (res.success) bgModelWorker.reset();
      }).catch(err => console.error('Failed to save model:', err));
    } else if (action === 'export') {
      if (bgModelCaptures.length === 0) return;
      // v2 hoists the fields that repeat identically on every sweep and stores
      // per-position stats alongside the raw sweeps, so the file is directly
      // usable for offline fitting without recomputation.
      const ref = bgModelCaptures[0].samples[0] || {};
      const exportData = {
        version: 2,
        type: 'bgmodel_training_data',
        timestamp: new Date().toISOString(),
        sfcwParams: sfcwParams,
        lidarAntennaOffsetMm: lidarOffsetMm,
        sweepsPerCapture: bgModelSweepsPerCapture,
        common: {
          num_steps: ref.num_steps,
          step_size: ref.step_size,
          range_offset: ref.range_offset,
        },
        captures: bgModelCaptures.map(c => ({
          stats: c.stats || computeCaptureStats(c.samples),
          standoffs: c.samples.map(s => s.lidar_standoff_mm),
          real: c.samples.map(s => s.h_cal_real),
          imag: c.samples.map(s => s.h_cal_imag),
        })),
      };
      const blob = new Blob([JSON.stringify(exportData)], { type: 'application/json' });
      const url = URL.createObjectURL(blob);
      const a = document.createElement('a');
      a.href = url;
      a.download = `bgmodel_${bgModelCaptures.length}pos_${new Date().toISOString().replace(/[:.]/g, '-')}.json`;
      a.click();
      URL.revokeObjectURL(url);
    } else if (action === 'import') {
      const input = document.createElement('input');
      input.type = 'file';
      input.accept = '.json';
      input.onchange = (e) => {
        const file = e.target.files[0];
        if (!file) return;
        const reader = new FileReader();
        reader.onload = (ev) => {
          try {
            const imported = JSON.parse(ev.target.result);
            if (imported.type !== 'bgmodel_training_data' || !Array.isArray(imported.captures)) return;
            const common = imported.common || {};
            const caps = imported.captures.map(c => {
              // v1 stored a plain samples array; v2 stores hoisted columns
              const samples = c.samples || c.real.map((r, i) => ({
                h_cal_real: r,
                h_cal_imag: c.imag[i],
                lidar_standoff_mm: c.standoffs[i],
                num_steps: common.num_steps,
                step_size: common.step_size,
                range_offset: common.range_offset,
              }));
              return { samples, stats: c.stats || computeCaptureStats(samples) };
            });
            setBgModelCaptures(caps);
          } catch (err) {
            console.error('Failed to import BG model data:', err);
          }
        };
        reader.readAsText(file);
      };
      input.click();
    } else if (action === 'test_phase') {
      setBgModelTesting(true);
      setBgModelTestResult(null);
      bgModelTestRef.current = { samples: [] };
    }
  }, [sendSdr, sendSfcwParams, sfcwRunning, bgModelCaptures, sfcwParams, bgModelSweepsPerCapture, bgModelWorker.startTraining, bgModelWorker.reset, bgModelWorker.resultRef]);

  // Rate counter interval
  const rateIntervalRef = useRef(null);

  const handleConnect = useCallback(() => {
    if (!piIp.trim()) return;
    localStorage.setItem('pi_ip', piIp);
    connectImu();
    connectSdr();
    connectRover();

    if (rateIntervalRef.current) clearInterval(rateIntervalRef.current);
    rateIntervalRef.current = setInterval(() => {
      setImuRate(imuCountRef.current);
      imuCountRef.current = 0;
    }, 1000);
  }, [piIp, connectImu, connectSdr, connectRover]);

  const handleDisconnect = useCallback(() => {
    disconnectImu();
    disconnectSdr();
    disconnectRover();
    if (rateIntervalRef.current) { clearInterval(rateIntervalRef.current); rateIntervalRef.current = null; }
    setImuRate(0);
  }, [disconnectImu, disconnectSdr, disconnectRover]);

  const isConnected = imuStatus === 'connected';

  // Auto-connect on mount if a saved IP exists
  const autoConnectedRef = useRef(false);
  useEffect(() => {
    if (!autoConnectedRef.current && piIp.trim()) {
      autoConnectedRef.current = true;
      handleConnect();
    }
  }, [handleConnect, piIp]);

  return (
    <div className="flex w-full min-h-screen bg-black">
      <Sidebar
        isConnected={isConnected}
        activePanel={activePanel}
        onActivePanelChange={setActivePanel}
        piIp={piIp}
        onPiIpChange={setPiIp}
        onConnect={handleConnect}
        onDisconnect={handleDisconnect}
        imuRate={imuRate}
        imuData={imuData}
        lidarMm={lidarMm}
        sdrConnected={sdrConnectionStatus === 'connected'}
        roverConnected={roverConnectionStatus === 'connected'}
        roverStatus={roverStatus}
        sendRover={sendRover}
        onClearRoverTrail={clearRoverTrail}
        txActive={txActive}
        rxActive={rxActive}
        showFFT={showFFT}
        onToggleFFT={setShowFFT}
        graphPaused={graphPaused}
        onTogglePause={setGraphPaused}
        sendSdr={sendSdr}
        sfcwRunning={sfcwRunning}
        sfcwStatus={sfcwStatus}
        sfcwParams={sfcwParams}
        onSfcwParamsChange={setSfcwParams}
        sfcwResult={processedSfcwResult}
        coherenceResult={coherenceResult}
        sfcwRangeScale={sfcwRangeScale}
        onSfcwRangeScaleChange={setSfcwRangeScale}
        sfcwScaleRange={sfcwScaleRange}
        onSfcwScaleRangeChange={setSfcwScaleRange}
        getSfcwDynamicScale={getSfcwDynamicScale}
        sfcwBgModel={sfcwBgModel}
        sfcwBgRef={sfcwBgRef}
        sfcwBgCapturing={sfcwBgCapturing}
        sfcwBgSubMode={sfcwBgSubMode}
        onSfcwBgSubModeChange={setSfcwBgSubMode}
        sfcwBgDiag={sfcwBgDiag}
        sfcwBgStats={sfcwBgStats}
        onResetSfcwBgStats={resetSfcwBgStats}
        sfcwLidarProvenance={sfcwLidarProvenance}
        onCaptureSfcwBg={handleSfcwCaptureBg}
        onLoadSfcwBgModel={handleSfcwLoadBgModel}
        onClearSfcwBg={handleSfcwClearBg}
        bscanData={bscanData}
        bscanCapturing={bscanCapturing}
        bscanBgRef={bscanBgRef}
        bscanBgModel={bscanBgModel}
        bscanBgCapturing={bscanBgCapturing}
        onCaptureBscanBg={handleBscanCaptureBg}
        onLoadBscanBgModel={handleBscanLoadBgModel}
        onClearBscanBg={handleBscanClearBg}
        lidarOffsetMm={lidarOffsetMm}
        onLidarOffsetChange={setLidarOffsetMm}
        bgApplied={bgApplied}
        onBgAppliedChange={setBgApplied}
        bscanParams={bscanParams}
        onBscanParamsChange={setBscanParams}
        onBscanAction={handleBscanAction}
        roverScan={roverScan}
        bscanScaleMode={bscanScaleMode}
        onBscanScaleModeChange={setBscanScaleMode}
        bscanDisplayMode={bscanDisplayMode}
        onBscanDisplayModeChange={setBscanDisplayMode}
        bscanScaleRange={bscanScaleRange}
        onBscanScaleRangeChange={setBscanScaleRange}
        bscanScaleScope={bscanScaleScope}
        onBscanScaleScopeChange={setBscanScaleScope}
        bscanShowGate={bscanShowGate}
        onBscanShowGateChange={setBscanShowGate}
        bscanScaleLink={bscanScaleLink}
        onBscanScaleLinkChange={setBscanScaleLink}
        cscanRowScales={cscanRowScales}
        cscanGridScales={cscanGridScales}
        cscanLiveDiag={cscanLiveProcessed.diag}
        bscanBgSubMode={bscanBgSubMode}
        onBscanBgSubModeChange={setBscanBgSubMode}
        bscanSuperFit={bscanSuperFit}
        onCaptureSuperFit={handleBscanCaptureSuperFit}
        onClearSuperFit={handleBscanClearSuperFit}
        cscanSharedScale={cscanSharedScale}
        cscanBgDiag={cscanBgDiag}
        bscanProcParams={bscanProcParams}
        bscanCaptureProgress={bscanCaptureProgress}
        sarBscanData={sarBscanInput}
        sarResult={sarResult}
        sarProgress={sarProgress}
        sarBgEnabled={sarBgEnabled}
        onSarBgEnabledChange={setSarBgEnabled}
        sarSvdEnabled={sarSvdEnabled}
        sarSvdK={sarSvdK}
        sarSvdStrength={sarSvdStrength}
        onSarSvdEnabledChange={setSarSvdEnabled}
        onSarSvdKChange={setSarSvdK}
        onSarSvdStrengthChange={setSarSvdStrength}
        sarScaleMode={sarScaleMode}
        onSarScaleModeChange={setSarScaleMode}
        sarAperture={sarAperture}
        onSarApertureChange={setSarAperture}
        sarCoherent={sarCoherent}
        onSarCoherentChange={setSarCoherent}
        sarDynRange={sarDynRange}
        onSarDynRangeChange={setSarDynRange}
        sarMaxDepth={sarMaxDepth}
        onSarMaxDepthChange={handleSarMaxDepthChange}
        sarEpsilonR={sarEpsilonR}
        onSarEpsilonRChange={setSarEpsilonR}
        sarWindowType={sarWindowType}
        onSarWindowTypeChange={setSarWindowType}
        sarAutoStandoff={sarAutoStandoff}
        onSarAutoStandoffChange={handleSarAutoStandoffChange}
        sarManualStandoffMm={sarManualStandoffMm}
        onSarManualStandoffChange={handleSarManualStandoffChange}
        sarWallThickness={sarWallThickness}
        onSarWallThicknessChange={setSarWallThickness}
        sarRefraction={sarRefraction}
        onSarRefractionChange={setSarRefraction}
        sarViewMode={sarViewMode}
        onSarViewModeChange={setSarViewMode}
        sarColormap={sarColormap}
        onSarColormapChange={setSarColormap}
        mapBscanData={mapBscanData}
        mapGateStart={mapGateStart}
        mapGateEnd={mapGateEnd}
        onMapGateStartChange={setMapGateStart}
        onMapGateEndChange={setMapGateEnd}
        mapDynRange={mapDynRange}
        onMapDynRangeChange={setMapDynRange}
        mapMetric={mapMetric}
        onMapMetricChange={setMapMetric}
        mapFocusEnabled={mapFocusEnabled}
        mapFocusAperture={mapFocusAperture}
        onMapFocusEnabledChange={setMapFocusEnabled}
        onMapFocusApertureChange={setMapFocusAperture}
        mapSvdEnabled={mapSvdEnabled}
        mapSvdK={mapSvdK}
        mapSvdStrength={mapSvdStrength}
        onMapSvdEnabledChange={setMapSvdEnabled}
        onMapSvdKChange={setMapSvdK}
        onMapSvdStrengthChange={setMapSvdStrength}
        bgModelCaptures={bgModelCaptures}
        bgModelCapturing={bgModelCapturing}
        bgModelAccumCount={bgModelAccumCount}
        bgModelTesting={bgModelTesting}
        bgModelTestCount={bgModelTestCount}
        bgModelTestResult={bgModelTestResult}
        bgModelTraining={bgModelWorker.trainingState}
        bgModelTrainProgress={bgModelWorker.progress}
        bgModelTrainResult={bgModelWorker.result}
        bgModelTrainError={bgModelWorker.error}
        bgModelSweepsPerCapture={bgModelSweepsPerCapture}
        onBgModelSweepsChange={setBgModelSweepsPerCapture}
        onBgModelAction={handleBgModelAction}
        imagingSnapshot={imagingSnapshot}
        imagingSnapshotName={imagingSnapshotName}
        onLoadImagingSnapshot={handleLoadImagingSnapshot}
        onClearImagingSnapshot={handleClearImagingSnapshot}
        imagingEffect={imagingEffect}
        onImagingEffectChange={setImagingEffect}
        imagingParams={imagingParams}
        onImagingParamsChange={setImagingParams}
      />
      <Viewport
        activePanel={activePanel}
        isConnected={isConnected}
        imuData={imuData}
        roverStatus={roverStatus}
        roverTrail={roverTrail}
        roverLog={roverLog}
        txActive={txActive}
        rxActive={rxActive}
        rxSamplesAnt={rxSamplesAnt}
        rxSamplesRef={rxSamplesRef}
        fftDataAnt={fftDataAnt}
        fftDataRef={fftDataRef}
        showFFT={showFFT}
        graphPaused={graphPaused}
        sfcwResult={processedSfcwResult}
        sfcwProgress={sfcwProgress}
        sfcwRunning={sfcwRunning}
        sfcwRangeScale={sfcwRangeScale}
        sfcwScaleRange={sfcwScaleRange}
        onSfcwScaleRangeChange={setSfcwScaleRange}
        onSfcwDynamicScale={handleSfcwDynamicScale}
        bscanData={cscanProcessedData}
        bscanBgDisplay={bscanBgDisplay}
        bscanBgSubMode={bscanBgSubMode}
        cscanSharedScale={cscanSharedScale}
        bscanProcParams={bscanProcParams}
        onBscanProcParamsChange={setBscanProcParams}
        bscanProcLocked={sfcwRunning || !!roverScan.active}
        bscanParams={bscanParams}
        bscanCapturing={bscanCapturing}
        roverScan={roverScan}
        bscanScaleMode={bscanScaleMode}
        bscanDisplayMode={bscanDisplayMode}
        bscanScaleRange={bscanScaleRange}
        bscanScaleScope={bscanScaleScope}
        bscanShowGate={bscanShowGate}
        bscanScaleLink={bscanScaleLink}
        cscanRowScales={cscanRowScales}
        cscanGridScales={cscanGridScales}
        cscanLiveResult={cscanLiveProcessed.result}
        sarResult={sarResult}
        sarProgress={sarProgress}
        sarScaleMode={sarScaleMode}
        sarDynRange={sarDynRange}
        sarViewMode={sarViewMode}
        sarColormap={sarColormap}
        mapBscanData={mapBscanData}
        mapGateStart={mapGateStart}
        mapGateEnd={mapGateEnd}
        mapDynRange={mapDynRange}
        mapMetric={mapMetric}
        mapStepSize={bscanParams.hStep}
        mapFocusEnabled={mapFocusEnabled}
        mapFocusAperture={mapFocusAperture}
        bgModelCaptures={bgModelCaptures}
        bgModelCapturing={bgModelCapturing}
        bgModelStopFreq={sfcwParams.stopFreq}
        imagingSnapshot={imagingSnapshot}
        imagingEffect={imagingEffect}
        imagingParams={imagingParams}
      />
    </div>
  );
}
