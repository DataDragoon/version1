import { useRef, useEffect, useCallback, useState } from 'react';
import { cn } from '@/lib/utils';
// Windows and CFAR are shared with the Imaging Bench so both panels run one
// implementation — see lib/imagingEffects.js.
import {
  computeCFAR, kaiserWindow, hanningWindow, rectangularWindow,
  CFAR_GUARD, CFAR_TRAIN, CFAR_ALPHA,
} from '@/lib/imagingEffects';

const BG = '#000000';
const GRID_COLOR = '#1a1a1a';
const TRACE_COLOR = '#D1855C';
const CFAR_COLOR = 'rgba(78, 205, 196, 0.6)';
const CFAR_FILL = 'rgba(78, 205, 196, 0.06)';
const NOISE_FILL = 'rgba(255, 255, 255, 0.02)';
const LINEAR_TRACE = '#6B9BD2';
const LOCK_HINT = 'Locked while a C-scan session is running — stop the session to change it';

function jet(t) {
  t = Math.max(0, Math.min(1, t));
  return [
    Math.round(255 * Math.min(1, Math.max(0, 1.5 - Math.abs(4 * t - 3)))),
    Math.round(255 * Math.min(1, Math.max(0, 1.5 - Math.abs(4 * t - 2)))),
    Math.round(255 * Math.min(1, Math.max(0, 1.5 - Math.abs(4 * t - 1)))),
  ];
}

// Perceptually-uniform maps. jet is kept because existing screenshots and
// habits are calibrated to it, but it bands sharply in smooth gradients, which
// makes post-subtraction noise read as structure -- the exact failure mode this
// panel is trying to avoid. viridis/inferno have a dark, low-contrast bottom so
// sub-threshold noise recedes instead of shimmering.
const VIRIDIS = [
  [68, 1, 84], [72, 40, 120], [62, 74, 137], [49, 104, 142], [38, 130, 142],
  [31, 158, 137], [53, 183, 121], [109, 205, 89], [253, 231, 37],
];
const INFERNO = [
  [0, 0, 4], [22, 11, 57], [66, 10, 104], [106, 23, 110], [147, 38, 103],
  [188, 55, 84], [221, 81, 58], [243, 120, 25], [252, 255, 164],
];

function rampLookup(ramp, t) {
  t = Math.max(0, Math.min(1, t));
  const x = t * (ramp.length - 1);
  const i = Math.min(ramp.length - 2, Math.floor(x));
  const f = x - i;
  const a = ramp[i], b = ramp[i + 1];
  return [
    Math.round(a[0] + (b[0] - a[0]) * f),
    Math.round(a[1] + (b[1] - a[1]) * f),
    Math.round(a[2] + (b[2] - a[2]) * f),
  ];
}

export const COLORMAPS = { jet, viridis: (t) => rampLookup(VIRIDIS, t), inferno: (t) => rampLookup(INFERNO, t) };

// Waterfall display transforms. RAW is the historical behaviour. DREF and CFAR
// both put a physically meaningful zero on the colour scale, which is what lets
// them be shown on FIXED limits -- with dynamic limits a new maximum rescales
// the whole image, so a target appearing DARKENS everything else instead of
// lighting one cell up.
// defaultSpan is the +/- dB the fixed colour scale uses. They differ a lot:
// measured on a static bench pair, dREF puts target-free background inside
// +/-0.23 dB (rms 0.07) and a real target at +4.5 dB, so +/-5 dB uses nearly the
// whole scale. CFAR values are offsets from a threshold already sitting ~alpha
// dB above the clutter mean, so they run wider.
export const WF_MODES = {
  raw:  { label: 'RAW',  needsDb: false, zeroed: false, defaultSpan: 12 },
  dref: { label: 'dREF', needsDb: true,  zeroed: true,  defaultSpan: 5  },
  cfar: { label: 'CFAR', needsDb: true,  zeroed: true,  defaultSpan: 12 },
};

// One waterfall row, transformed for display. Returns dB-relative values whose
// zero is meaningful (at the reference / at the detection threshold), or the row
// unchanged for RAW. Never mutates the input.
export function transformWfRow(rowDb, mode, ref, cfar) {
  if (mode === 'dref') {
    if (!ref || ref.length !== rowDb.length) return rowDb;
    const out = new Array(rowDb.length);
    for (let i = 0; i < rowDb.length; i++) out[i] = rowDb[i] - ref[i];
    return out;
  }
  if (mode === 'cfar') {
    const thr = computeCFAR(rowDb, cfar.guard, cfar.train, cfar.alpha, cfar.variant);
    const out = new Array(rowDb.length);
    for (let i = 0; i < rowDb.length; i++) out[i] = rowDb[i] - thr[i];
    return out;
  }
  return rowDb;
}

// Componentwise median across the last N rows. Median rather than mean so a
// single corrupted sweep (or one that caught a transient) cannot poison the
// reference the whole session is then measured against.
export function medianRows(rows) {
  if (!rows.length) return null;
  const n = rows[0].length;
  const out = new Array(n);
  const col = new Array(rows.length);
  for (let i = 0; i < n; i++) {
    for (let r = 0; r < rows.length; r++) col[r] = rows[r][i];
    col.sort((a, b) => a - b);
    const m = col.length >> 1;
    out[i] = col.length % 2 ? col[m] : 0.5 * (col[m - 1] + col[m]);
  }
  return out;
}

const SPEED_OF_LIGHT = 299_792_458;

// Radix-2 FFT (in-place, decimation-in-time)
function fft(re, im) {
  const n = re.length;
  // Bit-reverse permutation
  for (let i = 1, j = 0; i < n; i++) {
    let bit = n >> 1;
    for (; j & bit; bit >>= 1) j ^= bit;
    j ^= bit;
    if (i < j) {
      [re[i], re[j]] = [re[j], re[i]];
      [im[i], im[j]] = [im[j], im[i]];
    }
  }
  for (let len = 2; len <= n; len <<= 1) {
    const halfLen = len >> 1;
    const angle = -2 * Math.PI / len;
    const wRe = Math.cos(angle);
    const wIm = Math.sin(angle);
    for (let i = 0; i < n; i += len) {
      let curRe = 1, curIm = 0;
      for (let j = 0; j < halfLen; j++) {
        const uRe = re[i + j], uIm = im[i + j];
        const vRe = re[i + j + halfLen] * curRe - im[i + j + halfLen] * curIm;
        const vIm = re[i + j + halfLen] * curIm + im[i + j + halfLen] * curRe;
        re[i + j] = uRe + vRe;
        im[i + j] = uIm + vIm;
        re[i + j + halfLen] = uRe - vRe;
        im[i + j + halfLen] = uIm - vIm;
        const newCurRe = curRe * wRe - curIm * wIm;
        curIm = curRe * wIm + curIm * wRe;
        curRe = newCurRe;
      }
    }
  }
}

function ifft(re, im) {
  const n = re.length;
  // Conjugate
  for (let i = 0; i < n; i++) im[i] = -im[i];
  fft(re, im);
  // Conjugate and scale
  for (let i = 0; i < n; i++) {
    re[i] /= n;
    im[i] = -im[i] / n;
  }
}

function nextPow2(n) {
  let p = 1;
  while (p < n) p <<= 1;
  return p;
}

function computeRangeProfile(hCalReal, hCalImag, windowFn, zeroPadFactor) {
  const numSteps = hCalReal.length;
  const nfft = nextPow2(numSteps * zeroPadFactor);
  const re = new Float64Array(nfft);
  const im = new Float64Array(nfft);

  const win = windowFn(numSteps);
  for (let i = 0; i < numSteps; i++) {
    re[i] = hCalReal[i] * win[i];
    im[i] = hCalImag[i] * win[i];
  }

  ifft(re, im);

  const half = nfft >> 1;
  const magnitudeDb = new Float64Array(half);
  for (let i = 0; i < half; i++) {
    const mag = Math.sqrt(re[i] * re[i] + im[i] * im[i]);
    magnitudeDb[i] = 20 * Math.log10(mag + 1e-12);
  }
  return { magnitudeDb, nfft };
}

// procParams / onProcParamsChange make the window + averaging controls CONTROLLED,
// so the C-scan panel can drive its whole pipeline from this bar: the same window
// this pane draws with is the one every cell's profile is recomputed with, and the
// same Avg is the number of sweeps captured per grid cell. Omit them and the
// controls stay local, which is what the SFCW panel's own instance does.
//
// procLocked greys them out. avgCount cannot change part-way through a raster or
// different cells would hold different numbers of sweeps, and the window is locked
// with it so every cell in one grid is processed identically.
export default function SfcwDisplay({ sfcwResult, sfcwProgress, sfcwRunning, rangeScale, hideWaterfall, defaultScaleMode, onRangeScaleToggle, scaleRange, onScaleRangeChange, onDynamicScale,
  procParams, onProcParamsChange, procLocked, hideRangeComp, hideFloor, hideCfar, hideYMode }) {
  const rangeCanvasRef = useRef(null);
  const waterfallCanvasRef = useRef(null);
  const animRef = useRef(null);
  const latestResult = useRef(null);
  const [crosshairTrace, setCrosshairTrace] = useState(null);
  const [crosshairWaterfall, setCrosshairWaterfall] = useState(null);

  // Scale mode: 'db' or 'linear'
  const [scaleMode, setScaleMode] = useState(defaultScaleMode || 'db');

  // Windowing state
  const [windowTypeLocal, setWindowTypeLocal] = useState('rectangular');
  const [kaiserBetaLocal, setKaiserBetaLocal] = useState(3);
  const hCalRef = useRef(null);

  // Range compensation state
  const [rangeCompLocal, setRangeComp] = useState(0); // exponent: 0=off, 2=R², 4=R⁴

  // Averaging state
  const avgBuffer = useRef([]);
  const [avgCountLocal, setAvgCountLocal] = useState(1);
  const [averaged, setAveraged] = useState(null);
  // Measured noise floor: the sweep-to-sweep scatter of each range bin, which is what
  // actually decides whether a feature is readable. A bin's dB wobble is
  // ~8.7 * sigma/A, so a bin 30 dB above this line is steady to 0.3 dB while one AT it
  // swings by tens of dB and means nothing. Measured 2026-08-29 across 200 static
  // sweeps -- headroom above the floor vs what the bin does over the run:
  //   >30 dB -> 0.14 dB std, dips 0.4 dB      15-20 dB -> 1.64 dB std, dips 4.9 dB
  //   20-30  -> 0.69 dB std, dips 2.0 dB      10-15    -> 2.22 dB std, dips 8.0 dB
  // and the dips grow faster than the spikes, because a noise phasor can very nearly
  // cancel the signal (-> -inf dB) but can at most double it (-> +6 dB).
  const floorBuffer = useRef([]);
  const [noiseFloor, setNoiseFloor] = useState(null);
  const [showFloorLocal, setShowFloor] = useState(false);
  // Averaging mode. 'incoherent' averages the MAGNITUDE profiles (what this display has
  // always done); 'coherent' averages the complex h_cal and transforms once. Both reduce
  // the visible wobble by sqrt(N), but only coherent removes the noise's contribution to
  // the mean -- incoherent converges to |signal + noise|, which in a deep null is
  // dominated by the noise and so can never tell you whether anything is there. The two
  // are indistinguishable wherever a bin sits more than ~10 dB above the floor, which is
  // why this had gone unnoticed; the difference is entirely a null-depth question.
  const hCalBuffer = useRef([]);
  const lastPushedTs = useRef(null);
  const [avgModeLocal, setAvgModeLocal] = useState('incoherent');

  // Waterfall history buffer
  const waterfallHistory = useRef([]);
  // Display-transformed mirror of waterfallHistory, row-for-row.
  const wfDisplay = useRef([]);
  const WATERFALL_MAX_ROWS = 100;

  // Noise-floor estimator window. 16 sweeps is ~7 s at the 2.2 Hz sweep rate: long
  // enough that the sd estimate is stable (~18% at n=16, i.e. under 1.5 dB on the line
  // itself) and short enough to follow the floor if the bench actually changes.
  const FLOOR_WINDOW = 16;
  const FLOOR_MIN_SWEEPS = 6;
  const FLOOR_SMOOTH = 10;   // +/- bins for the along-range smoother
  const AVG_MAX = 32;       // largest selectable Avg, so the complex ring never grows past it
  // Detection threshold for the magnitude difference, in dB. Calibrated 2026-08-28 from
  // the target A/B: the target produced +4.4 dB at 21.2 cm while the target-free control
  // region (0-10 cm wall/coupling, which the target left undisturbed) held to 0.23 dB, so
  // 3x that control is the cut. PROVISIONAL -- that control was measured before the
  // reference-gain and sync_rx fixes dropped the noise floor ~15 dB, so the real
  // threshold is now likely lower and the A/B is worth re-running to recalibrate it.
  const DETECT_THRESHOLD_DB = 0.7;

  // Parallel ring buffer of the raw complex sweeps behind those rows, untouched
  // by window / range-comp / averaging / dB conversion. The Imaging Bench needs
  // complex data (phase-as-hue, coherence, coherent integration, dispersion,
  // raw S21), and the scalar waterfall rows have thrown all of that away.
  // Pushed in the same effect as waterfallHistory so row i lines up with row i.
  // It deliberately survives a scaleMode flip, which does wipe waterfallHistory
  // — the raw sweeps are unit-agnostic, so there is nothing to invalidate. That
  // one case is the only way the two buffers can differ in length.
  const rawHistory = useRef([]);
  // Mirrored into state purely so the EXPORT button can enable/disable itself;
  // the buffer itself is never read through React.
  const [rawCount, setRawCount] = useState(0);

  // Zoom/pan state for trace chart
  const [traceView, setTraceView] = useState({ xMin: 0, xMax: 1, yMin: -60, yMax: 40, autoY: true });
  const traceDrag = useRef(null);

  // CFAR params
  const [cfarGuard, setCfarGuard] = useState(CFAR_GUARD);
  const [cfarTrain, setCfarTrain] = useState(CFAR_TRAIN);
  const [cfarAlpha, setCfarAlpha] = useState(CFAR_ALPHA);
  const [cfarEnabledLocal, setCfarEnabled] = useState(true);
  // CA is the historical variant. GO holds the threshold up on the far side of
  // a strong return; SO is the one that survives an EXTENDED target, which is
  // what a real return looks like here (~50 mm range resolution plus sidelobes
  // smears a point scatterer across ~10 bins, and a CA/GO training window
  // sitting on that raises its own threshold and self-masks).
  const [cfarVariant, setCfarVariant] = useState('ca');

  // Display experiments — all four are independent so they can be A/B'd live
  // and the losers deleted. None of them touch the processing path: exports,
  // SAR input and BG-model training data are unaffected by every one.
  const [colormap, setColormap] = useState('jet');
  const [wfMode, setWfMode] = useState('raw');
  const [wfFixed, setWfFixed] = useState(true);     // fixed limits in zeroed modes
  const [wfSpan, setWfSpan] = useState(12);         // +/- dB when fixed
  const [spanTouched, setSpanTouched] = useState(false);
  const [refWindow, setRefWindow] = useState(16);   // frames medianed into a reference
  const [applyModeToTrace, setApplyModeToTrace] = useState(true);
  // 'session' never shrinks, so one early transient permanently squashes the
  // axis; 'frame' tracks the current sweep only.
  const [yModeLocal, setYMode] = useState('session');

  // ── Controlled processing params ────────────────────────────────────────
  // When procParams is supplied it replaces the four local states above; the
  // setters below write back through onProcParamsChange instead. Everything
  // downstream reads these names, so the two modes are indistinguishable to the
  // rest of the component.
  const controlled = !!procParams;
  const setProc = (patch) => onProcParamsChange && onProcParamsChange({ ...procParams, ...patch });
  const windowType = controlled ? procParams.windowType : windowTypeLocal;
  const setWindowType = controlled ? (v) => setProc({ windowType: v }) : setWindowTypeLocal;
  const kaiserBeta = controlled ? procParams.kaiserBeta : kaiserBetaLocal;
  const setKaiserBeta = controlled ? (v) => setProc({ kaiserBeta: v }) : setKaiserBetaLocal;
  const avgCount = controlled ? procParams.avgCount : avgCountLocal;
  const setAvgCount = controlled ? (v) => setProc({ avgCount: v }) : setAvgCountLocal;
  const avgMode = controlled ? procParams.avgMode : avgModeLocal;
  const setAvgMode = controlled ? (v) => setProc({ avgMode: v }) : setAvgModeLocal;

  // Hidden controls are forced off rather than merely not rendered, so a hidden
  // R^n / CFAR / FLOOR cannot keep applying itself from a stale default.
  const rangeComp = hideRangeComp ? 0 : rangeCompLocal;
  const showFloor = hideFloor ? false : showFloorLocal;
  const cfarEnabled = hideCfar ? false : cfarEnabledLocal;
  const yMode = hideYMode ? 'frame' : yModeLocal;

  // dREF reference row (dB). Held in a ref so capturing one does not re-render
  // at sweep rate; refVersion exists only to trigger the recompute effect.
  const refRow = useRef(null);
  const [refVersion, setRefVersion] = useState(0);
  const [hasRef, setHasRef] = useState(false);

  // Recomputed profile state
  const [recomputed, setRecomputed] = useState(null);
  // A magnitude difference is already a ratio in dB; 10^(x/20) of it is meaningless, so
  // the LIN path is forced off in that mode rather than left to draw a plausible-looking
  // wrong trace. Everything downstream reads effScaleMode, not scaleMode.
  const isDiff = !!(recomputed && recomputed.isDiff);
  const diffLinear = !!(recomputed && recomputed.diffLinear);
  // In diff mode the recompute already emitted the values in the selected units, so the
  // usual dB -> linear conversion must NOT be applied again on top.
  const effScaleMode = scaleMode;

  // Session-wide Y-axis tracking (only expands, never shrinks within a session)
  const sessionY = useRef({ min: Infinity, max: -Infinity });

  // Manual amplitude scaling pins both panes to the same limits. Undefined
  // scaleRange (the C-scan / BG-model live-sweep instances) means dynamic.
  const manualScale = !!(scaleRange && !scaleRange.dynamic);

  // The live dynamic limits are published upward so the panel can seed its
  // manual fields from whatever is on screen at the moment it is switched.
  const reportScale = useRef(null);
  useEffect(() => { reportScale.current = onDynamicScale || null; }, [onDynamicScale]);

  // Store raw h_cal when it arrives
  useEffect(() => {
    if (!sfcwResult) return;
    latestResult.current = sfcwResult;
    if (sfcwResult.h_cal_real && sfcwResult.h_cal_imag) {
      hCalRef.current = {
        real: sfcwResult.h_cal_real,
        imag: sfcwResult.h_cal_imag,
        step_size: sfcwResult.step_size,
        range_offset: sfcwResult.range_offset,
        num_steps: sfcwResult.num_steps,
        start_freq: sfcwResult.start_freq,
        stop_freq: sfcwResult.stop_freq,
        timestamp: sfcwResult.timestamp,
        phase_coherence: sfcwResult.phase_coherence,
      };
    }
  }, [sfcwResult]);

  // Clear averaging buffer when window params change so new window takes effect instantly
  useEffect(() => {
    avgBuffer.current = [];
    setAveraged(null);
    floorBuffer.current = [];
    setNoiseFloor(null);
  }, [windowType, kaiserBeta, rangeComp]);

  // Switching averaging mode must drop the magnitude buffer too, or the first frames
  // after a switch to coherent still show the stale incoherent average.
  useEffect(() => {
    avgBuffer.current = [];
    setAveraged(null);
  }, [avgMode]);

  // Recompute range profile client-side when window params change
  useEffect(() => {
    const hCal = hCalRef.current;
    if (!hCal) return;

    // Ring of raw complex sweeps, pushed once per ARRIVAL (not once per effect run --
    // this effect also fires on a window change, which must not duplicate a sweep).
    if (hCal.timestamp !== lastPushedTs.current) {
      lastPushedTs.current = hCal.timestamp;
      hCalBuffer.current.push({ real: Float64Array.from(hCal.real),
                                imag: Float64Array.from(hCal.imag) });
      if (hCalBuffer.current.length > AVG_MAX) hCalBuffer.current.shift();
    }

    // Coherent averaging: mean of the complex h_cal, transformed once. Equivalent to
    // averaging the complex range profiles (the IFFT is linear) but cheaper.
    let srcReal = hCal.real, srcImag = hCal.imag;
    if (avgMode === 'coherent' && avgCount > 1) {
      const rows = hCalBuffer.current.filter(r => r.real.length === hCal.real.length)
                                     .slice(-avgCount);
      if (rows.length > 1) {
        const len = hCal.real.length;
        const ar = new Float64Array(len), ai = new Float64Array(len);
        for (let j = 0; j < rows.length; j++) {
          for (let i = 0; i < len; i++) { ar[i] += rows[j].real[i]; ai[i] += rows[j].imag[i]; }
        }
        for (let i = 0; i < len; i++) { ar[i] /= rows.length; ai[i] /= rows.length; }
        srcReal = ar; srcImag = ai;
      }
    }

    const winFn = windowType === 'kaiser'
      ? (n) => kaiserWindow(n, kaiserBeta)
      : windowType === 'hanning'
        ? hanningWindow
        : rectangularWindow;

    const { magnitudeDb, nfft } = computeRangeProfile(
      srcReal, srcImag, winFn, 4
    );

    // Magnitude-difference mode: current profile minus reference profile, both
    // transformed with the SAME window and zero-pad so the difference means anything.
    // This is the statistic that actually detected the target in the 2026-08-28 A/B
    // (+4.4 dB at 21.2 cm against a 0.23 dB control region) -- and unlike the complex
    // difference it tolerates ~1 mm of standoff error, because a sub-mm shift is a small
    // fraction of a range bin in magnitude but a large phase rotation in complex.
    //
    // dB and LINEAR are genuinely DIFFERENT quantities here, not two views of one:
    //   dB     20log10|P| - 20log10|P_bg|  -- a RATIO. "by what fraction did this bin
    //          change", so it is level-independent and will show a weak bin changing a
    //          lot. That also means it amplifies noise wherever the profile approaches
    //          the floor, exactly as an ordinary dB profile does.
    //   linear |P| - |P_bg|                -- an absolute AMPLITUDE difference. "how much
    //          energy was actually added here", so a bin sitting at the noise floor
    //          contributes almost nothing however wildly its ratio swings.
    // Neither is derivable from the other by rescaling the axis, which is why the mode
    // has to be decided here rather than at draw time.
    const magDiff = sfcwResult && sfcwResult.bg_sub_mode === 'magnitude'
      && sfcwResult.bg_h_cal_real && sfcwResult.bg_h_cal_imag
      && sfcwResult.bg_h_cal_real.length === hCal.real.length;
    let diff = null;
    if (magDiff) {
      const bgDb = computeRangeProfile(sfcwResult.bg_h_cal_real, sfcwResult.bg_h_cal_imag,
                                       winFn, 4).magnitudeDb;
      diff = new Float64Array(magnitudeDb.length);
      if (scaleMode === 'linear') {
        for (let i = 0; i < magnitudeDb.length; i++) {
          diff[i] = Math.pow(10, magnitudeDb[i] / 20) - Math.pow(10, bgDb[i] / 20);
        }
      } else {
        for (let i = 0; i < magnitudeDb.length; i++) diff[i] = magnitudeDb[i] - bgDb[i];
      }
    }
    // The floor estimator must always see SINGLE-sweep profiles, in both modes, or it
    // measures the scatter of overlapping sliding averages and then gets divided by
    // sqrt(N) a second time. In incoherent mode magnitudeDb already is one sweep.
    const singleDb = srcReal === hCal.real
      ? magnitudeDb
      : computeRangeProfile(hCal.real, hCal.imag, winFn, 4).magnitudeDb;

    const maxRange = SPEED_OF_LIGHT / (2 * hCal.step_size);
    const half = nfft >> 1;
    const allDistances = new Float64Array(half);
    for (let i = 0; i < half; i++) {
      allDistances[i] = (i / nfft) * maxRange - hCal.range_offset;
    }

    let startIdx = 0;
    while (startIdx < half && allDistances[startIdx] < 0) startIdx++;
    const distances = allDistances.slice(startIdx);
    const clippedMag = (diff || magnitudeDb).slice(startIdx);
    const clippedSingle = diff ? clippedMag
      : (singleDb === magnitudeDb ? clippedMag : singleDb.slice(startIdx));

    // R^n range compensation (STC) — use true physical distance (add back offset).
    // Skipped on a magnitude difference: the same gain is applied to both profiles, so it
    // cancels exactly. Applying it would be a no-op that merely looked like a control.
    if (rangeComp > 0 && !diff) {
      for (let i = 0; i < clippedMag.length; i++) {
        const r = distances[i] + hCal.range_offset;
        if (r > 0.01) {
          const g = rangeComp * 10 * Math.log10(r);
          clippedMag[i] += g;
          if (clippedSingle !== clippedMag) clippedSingle[i] += g;
        }
      }
    }

    setRecomputed({ magnitudes: clippedMag, distances, single: clippedSingle,
                    isDiff: !!diff, diffLinear: !!diff && scaleMode === 'linear' });
  }, [windowType, kaiserBeta, rangeComp, sfcwResult, avgMode, avgCount, scaleMode]);

  // Averaging — uses recomputed data
  useEffect(() => {
    const mags = recomputed ? recomputed.magnitudes : (sfcwResult && sfcwResult.magnitudes);
    if (!mags) return;
    // In coherent mode the averaging already happened upstream on h_cal; averaging the
    // resulting magnitudes again would apply it twice and lag the display by 2N sweeps.
    if (avgMode === 'coherent') { setAveraged(null); return; }

    avgBuffer.current.push(Array.from(mags));
    if (avgBuffer.current.length > avgCount) {
      avgBuffer.current = avgBuffer.current.slice(-avgCount);
    }

    if (avgBuffer.current.length > 0) {
      const len = avgBuffer.current[0].length;
      const avg = new Float64Array(len);
      for (let i = 0; i < len; i++) {
        let sum = 0;
        for (let j = 0; j < avgBuffer.current.length; j++) {
          sum += avgBuffer.current[j][i];
        }
        avg[i] = sum / avgBuffer.current.length;
      }
      setAveraged(avg);
    }
  }, [recomputed, sfcwResult, avgCount, avgMode]);

  // Noise floor estimate: per-bin standard deviation of LINEAR amplitude across the last
  // FLOOR_WINDOW sweeps. Linear, not dB, because the error is additive in amplitude and
  // flat across range -- in dB the same error reads as a fraction of a dB on a strong bin
  // and tens of dB on a weak one, which is exactly the confusion this line exists to end.
  //
  // Estimated from the single-sweep rows even when averaging is on, then scaled by
  // 1/sqrt(avgCount) at draw time. That is exact here rather than optimistic: the error
  // is white sweep to sweep (lag-1 correlation 0.06) and the floor was measured to follow
  // 10*log10(N) out to N=32, so averaging really does buy the full reduction.
  useEffect(() => {
    // Skipped in diff mode: the estimator converts its rows with 10^(x/20), which is
    // wrong for a dB ratio and wrong again for a linear difference. FLOOR is suppressed
    // on those traces anyway, so there is nothing to gain by filling the buffer.
    if (recomputed && recomputed.isDiff) return;
    const mags = recomputed
      ? (recomputed.single || recomputed.magnitudes)
      : (sfcwResult && sfcwResult.magnitudes);
    if (!mags) return;
    const n = mags.length;
    const lin = new Float64Array(n);
    for (let i = 0; i < n; i++) lin[i] = Math.pow(10, mags[i] / 20);
    floorBuffer.current.push(lin);
    if (floorBuffer.current.length > FLOOR_WINDOW) floorBuffer.current.shift();

    const rows = floorBuffer.current;
    if (rows.length < FLOOR_MIN_SWEEPS || rows[0].length !== n) {
      if (rows.length && rows[0].length !== n) floorBuffer.current = [lin];
      return;
    }
    const sd = new Float64Array(n);
    for (let i = 0; i < n; i++) {
      let sum = 0;
      for (let j = 0; j < rows.length; j++) sum += rows[j][i];
      const mean = sum / rows.length;
      let acc = 0;
      for (let j = 0; j < rows.length; j++) {
        const d = rows[j][i] - mean;
        acc += d * d;
      }
      sd[i] = Math.sqrt(acc / (rows.length - 1));
    }
    // Smooth across range. The true floor varies only slowly with range (it is flat, or
    // follows the R^n gain if that is on), while a per-bin sd from a handful of sweeps is
    // itself noisy -- so smoothing costs no real detail and makes the line legible.
    const sm = new Float64Array(n);
    for (let i = 0; i < n; i++) {
      const lo = Math.max(0, i - FLOOR_SMOOTH), hi = Math.min(n - 1, i + FLOOR_SMOOTH);
      let sum = 0;
      for (let k = lo; k <= hi; k++) sum += sd[k];
      sm[i] = sum / (hi - lo + 1);
    }
    setNoiseFloor(sm);
  }, [recomputed, sfcwResult]);

  const drawChart = useCallback((canvas, opts) => {
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

    const result = latestResult.current;
    if (!result || !result.magnitudes || result.magnitudes.length === 0) {
      ctx.fillStyle = '#333333';
      ctx.font = '11px monospace';
      ctx.textAlign = 'center';
      ctx.fillText('No sweep data', w / 2, h / 2);
      return;
    }

    const {
      mags, dists, view, traceColor, title, crosshair,
      showCFAR, isDb, sessionY, manual, manualMin, manualMax, onScale,
      useSession = true, zeroLine = false, floor = null, floorDivisor = 1,
      detectThreshold = 0
    } = opts;
    const n = mags.length;
    const pad = { top: 24, bottom: 36, left: 52, right: 16 };
    const plotW = w - pad.left - pad.right;
    const plotH = h - pad.top - pad.bottom;

    // Determine Y range: pinned limits win, else session-wide extremes
    let yMin = view.yMin;
    let yMax = view.yMax;
    if (manual) {
      yMin = manualMin;
      yMax = manualMax;
    } else if (view.autoY && sessionY && useSession) {
      // Update session extremes with current frame data
      for (let i = 0; i < n; i++) {
        if (mags[i] < sessionY.current.min) sessionY.current.min = mags[i];
        if (mags[i] > sessionY.current.max) sessionY.current.max = mags[i];
      }
      // Use session extremes with margin
      const range = sessionY.current.max - sessionY.current.min;
      const margin = range * 0.05 || 1;
      yMin = sessionY.current.min - margin;
      yMax = sessionY.current.max + margin;
    } else if (view.autoY) {
      let dataMin = Infinity, dataMax = -Infinity;
      const startIdx = Math.max(0, Math.floor(view.xMin * (n - 1)));
      const endIdx = Math.min(n - 1, Math.ceil(view.xMax * (n - 1)));
      for (let i = startIdx; i <= endIdx; i++) {
        if (mags[i] < dataMin) dataMin = mags[i];
        if (mags[i] > dataMax) dataMax = mags[i];
      }
      const margin = (dataMax - dataMin) * 0.1 || 1;
      yMin = dataMin - margin;
      yMax = dataMax + margin;
    }

    // Publish the live limits so the panel can hand them to manual mode.
    if (!manual && onScale) onScale({ min: yMin, max: yMax, isDb });

    const xToPixel = (frac) => pad.left + ((frac - view.xMin) / (view.xMax - view.xMin)) * plotW;
    const yToPixel = (val) => pad.top + ((yMax - val) / (yMax - yMin)) * plotH;
    const pixelToX = (px) => view.xMin + ((px - pad.left) / plotW) * (view.xMax - view.xMin);

    // Grid
    ctx.strokeStyle = GRID_COLOR;
    ctx.lineWidth = 0.5;
    const yTicks = 5;
    for (let i = 0; i <= yTicks; i++) {
      const y = pad.top + (i / yTicks) * plotH;
      ctx.beginPath();
      ctx.moveTo(pad.left, y);
      ctx.lineTo(w - pad.right, y);
      ctx.stroke();
      const val = yMax - (i / yTicks) * (yMax - yMin);
      ctx.fillStyle = '#555555';
      ctx.font = '9px monospace';
      ctx.textAlign = 'right';
      ctx.fillText(isDb ? `${val.toFixed(0)} dB` : val.toExponential(1), pad.left - 6, y + 3);
    }

    const maxDist = dists[dists.length - 1];
    const minDist = dists[0];
    const distRange = maxDist - minDist;
    const xTicks = 6;
    for (let i = 0; i <= xTicks; i++) {
      const frac = view.xMin + (i / xTicks) * (view.xMax - view.xMin);
      const x = pad.left + (i / xTicks) * plotW;
      ctx.beginPath();
      ctx.moveTo(x, pad.top);
      ctx.lineTo(x, h - pad.bottom);
      ctx.stroke();
      const dist = minDist + frac * distRange;
      ctx.fillStyle = '#555555';
      ctx.font = '9px monospace';
      ctx.textAlign = 'center';
      ctx.fillText(`${dist.toFixed(2)} m`, x, h - pad.bottom + 14);
    }

    // In a zeroed mode 0 is the decision line (at the reference / at the CFAR
    // threshold), so draw it: "above this" is the call the operator is making.
    if (zeroLine && 0 > yMin && 0 < yMax) {
      const zy = yToPixel(0);
      ctx.save();
      ctx.strokeStyle = 'rgba(78, 205, 196, 0.55)';
      ctx.lineWidth = 1;
      ctx.setLineDash([6, 4]);
      ctx.beginPath();
      ctx.moveTo(pad.left, zy);
      ctx.lineTo(w - pad.right, zy);
      ctx.stroke();
      ctx.setLineDash([]);
      ctx.fillStyle = 'rgba(78, 205, 196, 0.8)';
      ctx.font = '9px monospace';
      ctx.textAlign = 'left';
      ctx.fillText('0 dB', pad.left + 4, zy - 3);
      ctx.restore();
    }

    // Detection threshold band for the magnitude difference. Everything inside +/- this
    // is "indistinguishable from the background"; a target is a rise that clears it and
    // stays there. Drawn as a shaded band rather than two lines so the eye reads the
    // inside as "nothing", which is the call being made.
    if (detectThreshold > 0) {
      const yHi = yToPixel(detectThreshold);
      const yLo = yToPixel(-detectThreshold);
      ctx.save();
      ctx.fillStyle = 'rgba(255, 255, 255, 0.05)';
      ctx.fillRect(pad.left, Math.min(yHi, yLo), plotW, Math.abs(yLo - yHi));
      ctx.strokeStyle = 'rgba(78, 205, 196, 0.35)';
      ctx.lineWidth = 1;
      ctx.setLineDash([3, 3]);
      for (const yy of [yHi, yLo]) {
        ctx.beginPath();
        ctx.moveTo(pad.left, yy);
        ctx.lineTo(w - pad.right, yy);
        ctx.stroke();
      }
      ctx.setLineDash([]);
      ctx.fillStyle = 'rgba(78, 205, 196, 0.75)';
      ctx.font = '9px monospace';
      ctx.textAlign = 'right';
      ctx.fillText(`\u00b1${detectThreshold.toFixed(2)} dB detect`, w - pad.right - 4, yHi - 4);
      ctx.restore();
    }

    // Measured noise floor. Drawn under everything else so the trace stays readable.
    // The point is not the line but the region below it: a bin there is not a weak
    // measurement, it is an absence of one, and in dB it will swing by tens of dB on a
    // completely static scene. See the noiseFloor estimator above for the numbers.
    if (floor && floor.length === n) {
      const toY = (linSigma) => {
        const v = linSigma / floorDivisor;
        return isDb ? 20 * Math.log10(v + 1e-15) : v;
      };
      ctx.save();
      ctx.beginPath();
      let started = false;
      for (let i = 0; i < n; i++) {
        const frac = i / (n - 1);
        if (frac < view.xMin || frac > view.xMax) continue;
        const x = xToPixel(frac);
        const y = yToPixel(toY(floor[i]));
        if (!started) { ctx.moveTo(x, y); started = true; } else ctx.lineTo(x, y);
      }
      if (started) {
        // Shade from the line down to the bottom of the plot.
        ctx.lineTo(xToPixel(Math.min(view.xMax, 1)), h - pad.bottom);
        ctx.lineTo(xToPixel(view.xMin), h - pad.bottom);
        ctx.closePath();
        ctx.fillStyle = 'rgba(232, 163, 61, 0.10)';
        ctx.fill();

        ctx.beginPath();
        started = false;
        for (let i = 0; i < n; i++) {
          const frac = i / (n - 1);
          if (frac < view.xMin || frac > view.xMax) continue;
          const x = xToPixel(frac);
          const y = yToPixel(toY(floor[i]));
          if (!started) { ctx.moveTo(x, y); started = true; } else ctx.lineTo(x, y);
        }
        ctx.strokeStyle = 'rgba(232, 163, 61, 0.75)';
        ctx.lineWidth = 1;
        ctx.setLineDash([4, 3]);
        ctx.stroke();
        ctx.setLineDash([]);

        const midIdx = Math.min(n - 1, Math.round(((view.xMin + view.xMax) / 2) * (n - 1)));
        const label = isDb
          ? `NOISE FLOOR ${toY(floor[midIdx]).toFixed(1)} dB`
          : 'NOISE FLOOR';
        ctx.fillStyle = 'rgba(232, 163, 61, 0.9)';
        ctx.font = '9px monospace';
        ctx.textAlign = 'left';
        ctx.fillText(label, pad.left + 4, yToPixel(toY(floor[midIdx])) + 11);
      }
      ctx.restore();
    }

    // CFAR threshold + noise shading
    let cfarThreshold = null;
    if (showCFAR && isDb) {
      cfarThreshold = computeCFAR(mags, cfarGuard, cfarTrain, cfarAlpha);

      // Shade below CFAR as noise floor
      ctx.beginPath();
      ctx.moveTo(pad.left, h - pad.bottom);
      for (let i = 0; i < n; i++) {
        const frac = i / (n - 1);
        if (frac < view.xMin || frac > view.xMax) continue;
        const x = xToPixel(frac);
        const y = yToPixel(cfarThreshold[i]);
        if (frac === view.xMin || i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      }
      ctx.lineTo(xToPixel(view.xMax), h - pad.bottom);
      ctx.lineTo(pad.left, h - pad.bottom);
      ctx.closePath();
      ctx.fillStyle = NOISE_FILL;
      ctx.fill();

      // CFAR threshold line
      ctx.beginPath();
      ctx.strokeStyle = CFAR_COLOR;
      ctx.lineWidth = 1;
      ctx.setLineDash([4, 4]);
      let started = false;
      for (let i = 0; i < n; i++) {
        const frac = i / (n - 1);
        if (frac < view.xMin || frac > view.xMax) continue;
        const x = xToPixel(frac);
        const y = yToPixel(cfarThreshold[i]);
        if (!started) { ctx.moveTo(x, y); started = true; }
        else ctx.lineTo(x, y);
      }
      ctx.stroke();
      ctx.setLineDash([]);
    }

    // Trace
    ctx.beginPath();
    ctx.strokeStyle = traceColor;
    ctx.lineWidth = 1.5;
    let first = true;
    for (let i = 0; i < n; i++) {
      const frac = i / (n - 1);
      if (frac < view.xMin || frac > view.xMax) continue;
      const x = xToPixel(frac);
      const y = yToPixel(mags[i]);
      const clampedY = Math.max(pad.top, Math.min(h - pad.bottom, y));
      if (first) { ctx.moveTo(x, clampedY); first = false; }
      else ctx.lineTo(x, clampedY);
    }
    ctx.stroke();

    // CFAR detections
    if (cfarThreshold && isDb) {
      for (let i = 1; i < n - 1; i++) {
        if (mags[i] > cfarThreshold[i] && mags[i] > mags[i - 1] && mags[i] > mags[i + 1]) {
          const frac = i / (n - 1);
          if (frac < view.xMin || frac > view.xMax) continue;
          const x = xToPixel(frac);
          const y = yToPixel(mags[i]);
          if (y < pad.top || y > h - pad.bottom) continue;

          ctx.beginPath();
          ctx.arc(x, y, 5, 0, Math.PI * 2);
          ctx.fillStyle = CFAR_FILL;
          ctx.fill();

          ctx.beginPath();
          ctx.arc(x, y, 2.5, 0, Math.PI * 2);
          ctx.fillStyle = CFAR_COLOR;
          ctx.fill();

          const dist = minDist + frac * distRange;
          ctx.fillStyle = CFAR_COLOR;
          ctx.font = '8px monospace';
          ctx.textAlign = 'center';
          ctx.fillText(`${dist.toFixed(2)}m`, x, y - 10);
        }
      }
    }

    // Crosshair
    if (crosshair) {
      const { x: mx } = crosshair;
      const relX = pixelToX(mx);
      if (relX >= view.xMin && relX <= view.xMax) {
        const idx = Math.round(relX * (n - 1));
        const clampedIdx = Math.max(0, Math.min(n - 1, idx));
        const cx = xToPixel(clampedIdx / (n - 1));
        const cy = yToPixel(mags[clampedIdx]);
        ctx.setLineDash([3, 3]);
        ctx.strokeStyle = '#ffffff44';
        ctx.lineWidth = 0.5;
        ctx.beginPath();
        ctx.moveTo(cx, pad.top);
        ctx.lineTo(cx, h - pad.bottom);
        ctx.stroke();
        ctx.setLineDash([]);

        const dist = minDist + (clampedIdx / (n - 1)) * distRange;
        ctx.fillStyle = '#ffffff';
        ctx.font = '10px monospace';
        ctx.textAlign = 'left';
        const label = isDb ? `${dist.toFixed(2)} m  ${mags[clampedIdx].toFixed(1)} dB` : `${dist.toFixed(2)} m  ${mags[clampedIdx].toExponential(2)}`;
        ctx.fillText(label, cx + 8, Math.max(pad.top + 12, cy - 4));
      }
    }

    // Title
    ctx.fillStyle = traceColor;
    ctx.font = 'bold 10px monospace';
    ctx.textAlign = 'left';
    ctx.fillText(title, pad.left, 14);
    if (manual) {
      const tw = ctx.measureText(title).width;
      ctx.fillStyle = '#f59e0b';
      ctx.font = '9px monospace';
      ctx.fillText('MANUAL', pad.left + tw + 8, 14);
    }
    if (result.range_resolution) {
      ctx.fillStyle = '#444444';
      ctx.font = '9px monospace';
      ctx.textAlign = 'right';
      ctx.fillText(`Δr=${(result.range_resolution * 100).toFixed(1)}cm`, w - pad.right, 14);
    }

    // Phase coherence (dB chart only)
    if (isDb && result.phase_coherence) {
      const pc = result.phase_coherence;
      const color = pc.coherent ? '#4ade80' : '#ef4444';
      ctx.fillStyle = color;
      ctx.font = '9px monospace';
      ctx.textAlign = 'right';
      ctx.fillText(
        `φ σ=${pc.phase_std_deg.toFixed(1)}° ${pc.coherent ? '● COHERENT' : '● INCOHERENT'}`,
        w - pad.right, 26
      );
    }

    return { pad, plotW, plotH, yMin, yMax };
  }, [cfarGuard, cfarTrain, cfarAlpha, cfarEnabled]);

  const drawWaterfall = useCallback((canvas, dists, view, crosshair, isDb, manual, manualMin, manualMax, opts = {}) => {
    const cmap = COLORMAPS[opts.colormap] || jet;
    const modeInfo = WF_MODES[opts.mode] || WF_MODES.raw;
    const zeroed = modeInfo.zeroed && isDb;
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

    const history = (zeroed || opts.mode === 'raw') && opts.rows ? opts.rows : waterfallHistory.current;
    if (history.length === 0) {
      ctx.fillStyle = '#333333';
      ctx.font = '11px monospace';
      ctx.textAlign = 'center';
      ctx.fillText('Waterfall — waiting for sweeps', w / 2, h / 2);
      return;
    }

    const pad = { top: 24, bottom: 36, left: 52, right: 20 };
    const plotW = w - pad.left - pad.right;
    const plotH = h - pad.top - pad.bottom;

    const n = history[0].length;
    const numRows = history.length;

    // Find visible bin range from view
    const startBin = Math.max(0, Math.floor(view.xMin * (n - 1)));
    const endBin = Math.min(n - 1, Math.ceil(view.xMax * (n - 1)));
    const visibleBins = endBin - startBin + 1;

    // Colour range: pinned limits win, else dynamic over all visible history
    let vMin = Infinity, vMax = -Infinity;
    if (manual) {
      vMin = manualMin;
      vMax = manualMax;
    } else if (zeroed && opts.fixed) {
      // The whole point of a zeroed mode: pinned symmetric limits, so a new
      // maximum lights up its own cell instead of rescaling every other one.
      vMin = -opts.span;
      vMax = opts.span;
    } else {
      for (let row = 0; row < numRows; row++) {
        for (let bin = startBin; bin <= endBin; bin++) {
          const v = history[row][bin];
          if (v < vMin) vMin = v;
          if (v > vMax) vMax = v;
        }
      }
    }
    if (!isFinite(vMin)) vMin = 0;
    if (!isFinite(vMax)) vMax = 1;
    if (vMax - vMin < 1e-6) { vMin -= 0.5; vMax += 0.5; }

    // Draw colormap
    const cellW = plotW / visibleBins;
    const cellH = plotH / WATERFALL_MAX_ROWS;

    for (let row = 0; row < numRows; row++) {
      for (let bin = startBin; bin <= endBin; bin++) {
        const t = (history[row][bin] - vMin) / (vMax - vMin);
        const [r, g, b] = cmap(t);
        ctx.fillStyle = `rgb(${r},${g},${b})`;
        const x = pad.left + (bin - startBin) * cellW;
        const y = pad.top + (numRows - 1 - row) * cellH;
        ctx.fillRect(x, y, Math.ceil(cellW) + 1, Math.ceil(cellH) + 1);
      }
    }

    // X-axis labels (distance)
    const maxDist = dists[dists.length - 1];
    const minDist = dists[0];
    const distRange = maxDist - minDist;
    const xTicks = 6;
    for (let i = 0; i <= xTicks; i++) {
      const frac = view.xMin + (i / xTicks) * (view.xMax - view.xMin);
      const x = pad.left + (i / xTicks) * plotW;
      ctx.fillStyle = '#555555';
      ctx.font = '9px monospace';
      ctx.textAlign = 'center';
      ctx.fillText(`${(minDist + frac * distRange).toFixed(2)} m`, x, h - pad.bottom + 14);
    }

    // Y-axis label
    ctx.save();
    ctx.translate(12, pad.top + plotH / 2);
    ctx.rotate(-Math.PI / 2);
    ctx.fillStyle = '#444444';
    ctx.font = '9px monospace';
    ctx.textAlign = 'center';
    ctx.fillText('Sweep #', 0, 0);
    ctx.restore();

    // Title
    ctx.fillStyle = '#D1855C';
    ctx.font = 'bold 10px monospace';
    ctx.textAlign = 'left';
    const wfTitle = zeroed
      ? `WATERFALL (${modeInfo.label} dB)`
      : (isDb ? 'WATERFALL (dB)' : 'WATERFALL (LINEAR)');
    ctx.fillText(wfTitle, pad.left, 14);
    if (manual) {
      const tw = ctx.measureText(wfTitle).width;
      ctx.fillStyle = '#f59e0b';
      ctx.font = '9px monospace';
      ctx.fillText('MANUAL', pad.left + tw + 8, 14);
    }

    ctx.fillStyle = '#444444';
    ctx.font = '9px monospace';
    ctx.textAlign = 'right';
    ctx.fillText(`${numRows} sweeps`, w - pad.right, 14);

    // Color bar
    const barW = 12;
    const barH = plotH;
    const barX = w - pad.right + 6;
    const barY = pad.top;
    for (let i = 0; i < barH; i++) {
      const t = 1 - i / barH;
      const [r, g, b] = cmap(t);
      ctx.fillStyle = `rgb(${r},${g},${b})`;
      ctx.fillRect(barX, barY + i, barW, 1);
    }
    // Where 0 falls on the bar -- the detection threshold in CFAR, "unchanged"
    // in dREF. Without it the operator has no anchor for what the colours mean.
    if (zeroed && vMin < 0 && vMax > 0) {
      const zy = barY + barH * (1 - (0 - vMin) / (vMax - vMin));
      ctx.strokeStyle = '#ffffff';
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.moveTo(barX - 3, zy);
      ctx.lineTo(barX + barW + 3, zy);
      ctx.stroke();
      ctx.fillStyle = '#ffffff';
      ctx.font = '8px monospace';
      ctx.textAlign = 'right';
      ctx.fillText('0', barX - 5, zy + 3);
    }
    ctx.fillStyle = manual ? '#f59e0b' : (zeroed && opts.fixed ? '#4ecdc4' : '#555555');
    ctx.font = '8px monospace';
    ctx.textAlign = 'left';
    if (isDb) {
      ctx.fillText(`${vMax.toFixed(0)} dB`, barX, barY - 4);
      ctx.fillText(`${vMin.toFixed(0)} dB`, barX, barY + barH + 10);
    } else {
      ctx.fillText(vMax.toExponential(1), barX, barY - 4);
      ctx.fillText(vMin.toExponential(1), barX, barY + barH + 10);
    }

    // Crosshair
    if (crosshair) {
      const relX = (crosshair.x - pad.left) / plotW;
      if (relX >= 0 && relX <= 1) {
        const frac = view.xMin + relX * (view.xMax - view.xMin);
        const dist = minDist + frac * distRange;
        ctx.setLineDash([3, 3]);
        ctx.strokeStyle = '#ffffff44';
        ctx.lineWidth = 0.5;
        ctx.beginPath();
        ctx.moveTo(crosshair.x, pad.top);
        ctx.lineTo(crosshair.x, h - pad.bottom);
        ctx.stroke();
        ctx.setLineDash([]);
        ctx.fillStyle = '#ffffff';
        ctx.font = '10px monospace';
        ctx.textAlign = 'left';
        ctx.fillText(`${dist.toFixed(2)} m`, crosshair.x + 8, crosshair.y - 4);
      }
    }
  }, []);

  // Capture the current background as the dREF reference. Median over the last
  // refWindow rows, so one bad sweep cannot poison it.
  const captureRef = useCallback(() => {
    const med = medianRows(waterfallHistory.current.slice(-refWindow));
    if (!med) return;
    refRow.current = med;
    setHasRef(true);
    setWfMode('dref');
    setRefVersion(v => v + 1);
  }, [refWindow]);

  useEffect(() => {
    if (spanTouched) return;
    const d = (WF_MODES[wfMode] || {}).defaultSpan;
    if (d) setWfSpan(d);
  }, [wfMode, spanTouched]);

  const clearRef = useCallback(() => {
    refRow.current = null;
    setHasRef(false);
    setRefVersion(v => v + 1);
  }, []);

  // Push new data into waterfall history
  useEffect(() => {
    const dbMags = averaged || (recomputed ? recomputed.magnitudes : (sfcwResult && sfcwResult.magnitudes));
    if (!dbMags) return;
    const row = (isDiff || effScaleMode === 'db')
      ? Array.from(dbMags)
      : Array.from(dbMags).map(db => Math.pow(10, db / 20));
    waterfallHistory.current.push(row);
    if (waterfallHistory.current.length > WATERFALL_MAX_ROWS) {
      waterfallHistory.current.shift();
    }

    // Same push, same cap — the raw complex sweep behind this row.
    const hCal = hCalRef.current;
    if (hCal && hCal.real && hCal.imag) {
      rawHistory.current.push({
        real: Float32Array.from(hCal.real),
        imag: Float32Array.from(hCal.imag),
        num_steps: hCal.num_steps,
        step_size: hCal.step_size,
        range_offset: hCal.range_offset,
        start_freq: hCal.start_freq,
        stop_freq: hCal.stop_freq,
        timestamp: hCal.timestamp,
        phase_coherence: hCal.phase_coherence,
      });
      if (rawHistory.current.length > WATERFALL_MAX_ROWS) {
        rawHistory.current.shift();
      }
    }
    // Transformed copy for display, computed once per row rather than per frame:
    // CFAR over 100 rows every animation frame would be tens of millions of ops
    // a second. Switching mode rebuilds this from waterfallHistory (below), so a
    // mode change re-renders the whole existing history instantly.
    // The wf transforms (CFAR-relative, reference-subtracted) are defined on an absolute
    // dB profile; a difference is already referenced to something, so they are bypassed.
    if (effScaleMode === 'db' && !isDiff) {
      wfDisplay.current.push(transformWfRow(row, wfMode, refRow.current, {
        guard: cfarGuard, train: cfarTrain, alpha: cfarAlpha, variant: cfarVariant,
      }));
    } else {
      wfDisplay.current.push(row);
    }
    if (wfDisplay.current.length > WATERFALL_MAX_ROWS) wfDisplay.current.shift();

    setRawCount(rawHistory.current.length);
  }, [averaged, recomputed, sfcwResult, effScaleMode, wfMode, cfarGuard, cfarTrain, cfarAlpha, cfarVariant]);

  // Rebuild the transformed history whenever the transform itself changes.
  useEffect(() => {
    const cfar = { guard: cfarGuard, train: cfarTrain, alpha: cfarAlpha, variant: cfarVariant };
    wfDisplay.current = (effScaleMode === 'db' && !isDiff)
      ? waterfallHistory.current.map(r => transformWfRow(r, wfMode, refRow.current, cfar))
      : waterfallHistory.current.slice();
    // The zeroed modes are in different units from RAW, so session extremes
    // carried across a mode change would be meaningless.
    sessionY.current = { min: Infinity, max: -Infinity };
  }, [wfMode, refVersion, cfarGuard, cfarTrain, cfarAlpha, cfarVariant, scaleMode]);

  // Clear waterfall when scale mode changes, and equally when the subtraction domain
  // does: rows already in the buffer are absolute dB in one mode and a dB RATIO in the
  // other, and stacking the two in one image would be quietly meaningless. The buffer
  // refills at the sweep rate (~30 s for a full 100 rows at 3.35 Hz). rawHistory is
  // intentionally left alone — see its declaration.
  useEffect(() => {
    waterfallHistory.current = [];
    wfDisplay.current = [];
    sessionY.current = { min: Infinity, max: -Infinity };
  }, [scaleMode, isDiff]);

  // Drop the raw buffer on unmount so a panel switch does not leak sweeps into
  // the next session.
  useEffect(() => () => { rawHistory.current = []; }, []);

  useEffect(() => {
    const render = () => {
      const result = latestResult.current;
      if (result && (recomputed || result.magnitudes)) {
        const dists = recomputed ? Array.from(recomputed.distances) : result.distances;
        const dbMags = averaged || (recomputed ? recomputed.magnitudes : result.magnitudes);
        const isDb = effScaleMode === 'db';
        const mags = (isDiff || isDb) ? dbMags : Array.from(dbMags).map(db => Math.pow(10, db / 20));
        const modeInfo = WF_MODES[wfMode] || WF_MODES.raw;
        const zeroed = modeInfo.zeroed && isDb;
        const traceOn = zeroed && applyModeToTrace;
        // Same transform the waterfall row got, so the two panes agree.
        const shownMags = traceOn
          ? transformWfRow(Array.from(mags), wfMode, refRow.current,
              { guard: cfarGuard, train: cfarTrain, alpha: cfarAlpha, variant: cfarVariant })
          : mags;
        const traceColor = isDb ? TRACE_COLOR : LINEAR_TRACE;
        const title = isDiff
          ? (diffLinear ? 'RANGE PROFILE (\u0394 LINEAR vs BG)' : 'RANGE PROFILE (\u0394 dB vs BG)')
          : traceOn
            ? `RANGE PROFILE (${modeInfo.label} dB)`
            : (isDb ? 'RANGE PROFILE (dB)' : 'RANGE PROFILE (LINEAR)');

        // Apply range scale to view
        let view = traceView;
        if (rangeScale && dists.length > 0) {
          const maxDist = dists[dists.length - 1];
          const minDist = dists[0];
          const distRange = maxDist - minDist;
          const xMin = Math.max(0, (rangeScale.min - minDist) / distRange);
          const xMax = Math.min(1, (rangeScale.max - minDist) / distRange);
          view = { ...traceView, xMin, xMax };
        }

        drawChart(rangeCanvasRef.current, {
          mags: shownMags, dists, view, traceColor,
          title, crosshair: crosshairTrace,
          // Already CFAR-normalised: overlaying the threshold again would draw
          // it at 0 and shade the entire plot.
          showCFAR: cfarEnabled && isDb && !(traceOn && wfMode === 'cfar'),
          isDb, sessionY,
          manual: manualScale,
          manualMin: scaleRange ? scaleRange.min : 0,
          manualMax: scaleRange ? scaleRange.max : 1,
          onScale: reportScale.current,
          useSession: yMode === 'session',
          // 0 dB is "identical to the background", so it is the decision line here too.
          zeroLine: traceOn || isDiff,
          // The 0.7 dB cut is a RELATIVE criterion, so it has no single linear value --
          // it corresponds to a different absolute amount at every bin. Shown only on the
          // dB flavour rather than drawn at some arbitrary linear level.
          detectThreshold: isDiff && !diffLinear ? DETECT_THRESHOLD_DB : 0,
          // Suppressed in the zeroed trace modes (CFAR-relative and friends): those
          // rescale the trace against their own reference, so an absolute floor drawn
          // over them would be meaningless rather than merely unhelpful.
          // Suppressed on a difference as well: the estimator's units are linear
          // amplitude, and on a ratio trace that line would be a number with no
          // meaning. The +/- threshold lines do this job in that mode instead.
          floor: showFloor && !traceOn && !isDiff ? noiseFloor : null,
          floorDivisor: Math.sqrt(Math.max(1, avgCount)),
        });
        drawWaterfall(
          waterfallCanvasRef.current, dists, view, crosshairWaterfall, isDb,
          manualScale,
          scaleRange ? scaleRange.min : 0,
          scaleRange ? scaleRange.max : 1,
          { colormap, mode: wfMode, fixed: wfFixed, span: wfSpan, rows: wfDisplay.current },
        );
      }
      animRef.current = requestAnimationFrame(render);
    };
    animRef.current = requestAnimationFrame(render);
    return () => { if (animRef.current) cancelAnimationFrame(animRef.current); };
  }, [drawChart, drawWaterfall, traceView, crosshairTrace, crosshairWaterfall, cfarEnabled, averaged, recomputed, effScaleMode, rangeScale, manualScale, scaleRange,
      colormap, wfMode, wfFixed, wfSpan, applyModeToTrace, yMode, cfarGuard, cfarTrain, cfarAlpha, cfarVariant,
      showFloor, noiseFloor, avgCount]);

  // Zoom handler
  const handleWheel = (e) => {
    if (rangeScale) return; // disable manual zoom when range scale is active
    e.preventDefault();
    const canvas = e.currentTarget;
    const rect = canvas.getBoundingClientRect();
    const pad = { left: 52, right: 16 };
    const plotW = rect.width - pad.left - pad.right;
    const mx = e.clientX - rect.left;
    const relX = (mx - pad.left) / plotW;
    const frac = traceView.xMin + relX * (traceView.xMax - traceView.xMin);

    const zoomFactor = e.deltaY > 0 ? 1.2 : 0.8;
    const newXMin = frac - (frac - traceView.xMin) * zoomFactor;
    const newXMax = frac + (traceView.xMax - frac) * zoomFactor;

    setTraceView(v => ({
      ...v,
      xMin: Math.max(0, newXMin),
      xMax: Math.min(1, newXMax),
    }));
  };

  // Pan handler
  const handleMouseDown = (e) => {
    if (rangeScale) return;
    if (e.button !== 0) return;
    const rect = e.currentTarget.getBoundingClientRect();
    const pad = { left: 52, right: 16 };
    const plotW = rect.width - pad.left - pad.right;
    const mx = e.clientX - rect.left;
    if (mx < pad.left || mx > rect.width - pad.right) return;
    traceDrag.current = { startX: e.clientX, startView: { ...traceView } };

    const onMove = (me) => {
      if (!traceDrag.current) return;
      const dx = me.clientX - traceDrag.current.startX;
      const xRange = traceDrag.current.startView.xMax - traceDrag.current.startView.xMin;
      const shift = -(dx / plotW) * xRange;
      const newMin = Math.max(0, traceDrag.current.startView.xMin + shift);
      const newMax = Math.min(1, traceDrag.current.startView.xMax + shift);
      if (newMax - newMin > 0.001) {
        setTraceView(v => ({ ...v, xMin: newMin, xMax: newMax }));
      }
    };
    const onUp = () => {
      traceDrag.current = null;
      window.removeEventListener('mousemove', onMove);
      window.removeEventListener('mouseup', onUp);
    };
    window.addEventListener('mousemove', onMove);
    window.addEventListener('mouseup', onUp);
  };

  // Snapshot the raw ring buffer to disk for the Imaging Bench. Same
  // Blob/anchor pattern as the C-scan export in App.jsx.
  const handleExportWaterfall = useCallback(() => {
    const buf = rawHistory.current;
    if (buf.length === 0) return;
    const ref = buf[buf.length - 1];
    const r8 = (arr) => Array.from(arr, v => Math.round(v * 1e8) / 1e8);
    const stamp = new Date().toISOString();
    const snapshot = {
      version: 1,
      type: 'waterfall_snapshot',
      timestamp: stamp,
      common: {
        num_steps: ref.num_steps,
        step_size: ref.step_size,
        start_freq: ref.start_freq,
        stop_freq: ref.stop_freq,
        range_offset: ref.range_offset,
      },
      displayState: { scaleMode, windowType, kaiserBeta, rangeComp, avgCount },
      sweeps: buf.map(sw => ({
        t: sw.timestamp,
        real: r8(sw.real),
        imag: r8(sw.imag),
        phase_coherence: sw.phase_coherence || null,
      })),
    };
    const blob = new Blob([JSON.stringify(snapshot)], { type: 'application/json' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `waterfall_${stamp.replace(/[:.]/g, '-')}.json`;
    a.click();
    URL.revokeObjectURL(url);
  }, [scaleMode, windowType, kaiserBeta, rangeComp, avgCount]);

  return (
    <div className="flex flex-col w-full h-full">
      {/* Progress bar */}
      {sfcwRunning && sfcwProgress && (
        <div className="absolute top-0 left-0 right-0 z-10 h-0.5">
          <div
            className="h-full bg-gradient-to-r from-[#D1855C] to-[#E5A986] transition-all duration-200"
            style={{ width: `${(sfcwProgress.step / sfcwProgress.total) * 100}%` }}
          />
        </div>
      )}

      {/* Controls bar */}
      <div className="flex flex-wrap items-center gap-3 px-3 py-1.5 border-b border-white/5 bg-black/40 shrink-0">
        {/* Window selector + Kaiser beta */}
        <div className="flex items-center gap-1.5">
          <span className="text-[9px] text-white/40 uppercase tracking-wider">Win</span>
          <select
            value={windowType}
            onChange={(e) => setWindowType(e.target.value)}
            disabled={procLocked}
            title={procLocked ? LOCK_HINT : undefined}
            className={cn('bg-white/5 border border-white/10 rounded px-1.5 py-0.5 text-[10px] outline-none',
              procLocked ? 'text-white/20 cursor-not-allowed' : 'text-white/70')}
          >
            <option value="rectangular">Rectangular</option>
            <option value="kaiser">Kaiser</option>
            <option value="hanning">Hanning</option>
          </select>
        </div>

        {windowType === 'kaiser' && (
          <div className="flex items-center gap-1.5">
            <span className="text-[9px] text-white/40">β</span>
            <input
              type="range"
              min={2}
              max={14}
              step={0.5}
              value={kaiserBeta}
              onChange={(e) => setKaiserBeta(Number(e.target.value))}
              disabled={procLocked}
              title={procLocked ? LOCK_HINT : undefined}
              className={cn('w-20 h-1 accent-primary', procLocked ? 'cursor-not-allowed opacity-30' : 'cursor-pointer')}
            />
            <span className="text-[10px] text-white/60 font-mono w-6">{kaiserBeta.toFixed(1)}</span>
          </div>
        )}

        {/* Range compensation. Hidden in the C-scan, where it would apply the same
            gain to every cell and to the background alike -- it cannot change any
            comparison between cells, which is the only thing that panel asks. */}
        {!hideRangeComp && (
          <>
            <div className="w-px h-3 bg-white/10" />
            <div className="flex items-center gap-1.5">
              <span className="text-[9px] text-white/40 uppercase tracking-wider">R^n</span>
              <select
                value={rangeComp}
                onChange={(e) => setRangeComp(Number(e.target.value))}
                className="bg-white/5 border border-white/10 rounded px-1.5 py-0.5 text-[10px] text-white/70 outline-none"
              >
                <option value={0}>Off</option>
                <option value={2}>R²</option>
                <option value={3}>R³</option>
                <option value={4}>R⁴</option>
              </select>
            </div>
          </>
        )}

        <div className="w-px h-3 bg-white/10" />

        {/* Averaging */}
        <div className="flex items-center gap-1.5">
          <span className="text-[9px] text-white/40 uppercase tracking-wider">Avg</span>
          <select
            value={avgCount}
            onChange={(e) => { setAvgCount(Number(e.target.value)); avgBuffer.current = []; }}
            disabled={procLocked}
            title={procLocked ? LOCK_HINT : undefined}
            className={cn('bg-white/5 border border-white/10 rounded px-1.5 py-0.5 text-[10px] outline-none',
              procLocked ? 'text-white/20 cursor-not-allowed' : 'text-white/70')}
          >
            {[1, 2, 4, 8, 16, 32].map(v => (
              <option key={v} value={v}>{v === 1 ? 'Off' : `${v}x`}</option>
            ))}
          </select>
          {/* Coherent vs incoherent. Disabled at Avg=Off, where the two are identical. */}
          <button
            onClick={() => setAvgMode(avgMode === 'coherent' ? 'incoherent' : 'coherent')}
            disabled={avgCount === 1 || procLocked}
            title={procLocked ? LOCK_HINT : avgCount === 1
              ? 'Set Avg above 1 to compare averaging modes'
              : 'COH averages complex h_cal (removes the noise bias in nulls); INC averages magnitudes'}
            className={cn(
              'px-1.5 py-0.5 rounded text-[9px] uppercase tracking-wider font-medium transition-all border',
              (avgCount === 1 || procLocked)
                ? 'bg-white/5 text-white/20 border-white/10 cursor-not-allowed'
                : avgMode === 'coherent'
                  ? 'bg-[#4ecdc4]/20 text-[#4ecdc4] border-[#4ecdc4]/30'
                  : 'bg-white/5 text-white/40 border-white/10'
            )}
          >
            {avgMode === 'coherent' ? 'Coh' : 'Inc'}
          </button>
        </div>

        {/* Measured noise floor. Off by default: it is a diagnostic overlay, and it only
            means anything once FLOOR_MIN_SWEEPS have accumulated. Hidden in the
            C-scan: it is estimated from this pane's own rolling sweep history,
            which describes the live sweep, not the recorded cell being drawn. */}
        {!hideFloor && (
          <>
            <div className="w-px h-3 bg-white/10" />
            <button
              onClick={() => setShowFloor(!showFloor)}
              title="Overlay the measured sweep-to-sweep noise floor. Bins near it are not measurements."
              className={cn(
                'px-2 py-0.5 rounded text-[9px] uppercase tracking-wider font-medium transition-all',
                showFloor ? 'bg-[#e8a33d]/20 text-[#e8a33d] border border-[#e8a33d]/30' : 'bg-white/5 text-white/30 border border-white/10'
              )}
            >
              Floor
            </button>
          </>
        )}

        {/* CFAR. Hidden in the C-scan: it is a per-trace detector on the live
            sweep and nothing in the grid pipeline reads it, so leaving it on
            would only disagree with the image beside it. */}
        {!hideCfar && (
          <>
            <div className="w-px h-3 bg-white/10" />
            <button
              onClick={() => setCfarEnabled(!cfarEnabled)}
              className={cn(
                'px-2 py-0.5 rounded text-[9px] uppercase tracking-wider font-medium transition-all',
                cfarEnabled ? 'bg-[#4ecdc4]/20 text-[#4ecdc4] border border-[#4ecdc4]/30' : 'bg-white/5 text-white/30 border border-white/10'
              )}
            >
              CFAR
            </button>
          </>
        )}

        {cfarEnabled && (
          <>
            <div className="flex items-center gap-1">
              <span className="text-[9px] text-white/30">G</span>
              <input type="number" min={1} max={20} value={cfarGuard}
                onChange={e => setCfarGuard(Number(e.target.value))}
                className="w-7 bg-white/5 border border-white/10 rounded px-1 py-0.5 text-[10px] text-white/70 outline-none"
              />
            </div>
            <div className="flex items-center gap-1">
              <span className="text-[9px] text-white/30">T</span>
              <input type="number" min={1} max={64} value={cfarTrain}
                onChange={e => setCfarTrain(Number(e.target.value))}
                className="w-7 bg-white/5 border border-white/10 rounded px-1 py-0.5 text-[10px] text-white/70 outline-none"
              />
            </div>
            <div className="flex items-center gap-1">
              <span className="text-[9px] text-white/30">α</span>
              <input type="number" min={1} max={20} value={cfarAlpha}
                onChange={e => setCfarAlpha(Number(e.target.value))}
                className="w-7 bg-white/5 border border-white/10 rounded px-1 py-0.5 text-[10px] text-white/70 outline-none"
              />
              <span className="text-[9px] text-white/20">dB</span>
            </div>
            {/* SO is the variant that survives an extended return; CA/GO can
                self-mask when the training window lands on the target. */}
            <select value={cfarVariant} onChange={e => setCfarVariant(e.target.value)}
              className="bg-white/5 border border-white/10 rounded px-1 py-0.5 text-[10px] text-white/70 outline-none">
              <option value="ca">CA</option>
              <option value="go">GO</option>
              <option value="so">SO</option>
            </select>
          </>
        )}

        {!hideWaterfall && (
          <>
            <div className="w-px h-3 bg-white/10" />

            {/* Waterfall display mode */}
            <div className="flex items-center gap-1">
              {Object.entries(WF_MODES).map(([k, v]) => (
                <button
                  key={k}
                  onClick={() => setWfMode(k)}
                  disabled={v.needsDb && scaleMode !== 'db'}
                  title={v.needsDb && scaleMode !== 'db' ? 'needs dB scale' : undefined}
                  className={cn(
                    'px-1.5 py-0.5 rounded text-[9px] uppercase tracking-wider font-medium transition-all border',
                    wfMode === k
                      ? 'bg-[#4ecdc4]/20 text-[#4ecdc4] border-[#4ecdc4]/30'
                      : 'bg-white/5 text-white/30 border-white/10',
                    v.needsDb && scaleMode !== 'db' && 'opacity-30 cursor-not-allowed',
                  )}
                >
                  {v.label}
                </button>
              ))}
            </div>

            {wfMode === 'dref' && (
              <>
                <button
                  onClick={captureRef}
                  className="px-2 py-0.5 rounded text-[9px] uppercase tracking-wider font-medium border bg-[#D1855C]/20 text-[#D1855C] border-[#D1855C]/30 hover:bg-[#D1855C]/30 transition-all"
                >
                  {hasRef ? 'Re-Ref' : 'Set Ref'}
                </button>
                <button
                  onClick={clearRef}
                  disabled={!hasRef}
                  className={cn('px-2 py-0.5 rounded text-[9px] border transition-all',
                    hasRef ? 'text-white/40 border-white/10 hover:text-white/70' : 'text-white/15 border-white/5 cursor-not-allowed')}
                >
                  Clr
                </button>
                <div className="flex items-center gap-1">
                  <span className="text-[9px] text-white/30">N</span>
                  <input type="number" min={1} max={100} value={refWindow}
                    onChange={e => setRefWindow(Math.max(1, Number(e.target.value) || 1))}
                    className="w-8 bg-white/5 border border-white/10 rounded px-1 py-0.5 text-[10px] text-white/70 outline-none"
                  />
                </div>
                {!hasRef && <span className="text-[9px] text-amber-400/70">no ref</span>}
              </>
            )}

            {(WF_MODES[wfMode] || {}).zeroed && (
              <>
                <button
                  onClick={() => setWfFixed(f => !f)}
                  className={cn('px-1.5 py-0.5 rounded text-[9px] uppercase tracking-wider font-medium border transition-all',
                    wfFixed ? 'bg-[#4ecdc4]/20 text-[#4ecdc4] border-[#4ecdc4]/30' : 'bg-white/5 text-white/30 border-white/10')}
                  title="Pin the colour scale so a new maximum does not rescale every other cell"
                >
                  Fix
                </button>
                {wfFixed && (
                  <div className="flex items-center gap-1">
                    <span className="text-[9px] text-white/30">±</span>
                    <input type="number" min={1} max={60} step="0.5" value={wfSpan}
                      onChange={e => { setSpanTouched(true); setWfSpan(Math.max(0.5, Number(e.target.value) || 1)); }}
                      className="w-8 bg-white/5 border border-white/10 rounded px-1 py-0.5 text-[10px] text-white/70 outline-none"
                    />
                    <span className="text-[9px] text-white/20">dB</span>
                  </div>
                )}
                <button
                  onClick={() => setApplyModeToTrace(t => !t)}
                  className={cn('px-1.5 py-0.5 rounded text-[9px] uppercase tracking-wider font-medium border transition-all',
                    applyModeToTrace ? 'bg-[#4ecdc4]/20 text-[#4ecdc4] border-[#4ecdc4]/30' : 'bg-white/5 text-white/30 border-white/10')}
                  title="Apply the same transform to the range profile"
                >
                  Trace
                </button>
              </>
            )}

            <div className="w-px h-3 bg-white/10" />

            {/* Colormap */}
            <select value={colormap} onChange={e => setColormap(e.target.value)}
              className="bg-white/5 border border-white/10 rounded px-1.5 py-0.5 text-[10px] text-white/70 outline-none"
              title="jet bands sharply and makes noise read as structure">
              <option value="jet">jet</option>
              <option value="viridis">viridis</option>
              <option value="inferno">inferno</option>
            </select>
          </>
        )}

        {/* Y-axis tracking. Hidden in the C-scan, where 'session' extremes
            accumulated across a whole raster would leave the live pane's axis
            set by whichever cell happened to be loudest. It is pinned to
            'frame' there. */}
        {!hideYMode && (
          <>
            <div className="w-px h-3 bg-white/10" />
            <div className="flex items-center gap-1">
              <span className="text-[9px] text-white/40 uppercase tracking-wider">Y</span>
              <select value={yMode} onChange={e => { setYMode(e.target.value); sessionY.current = { min: Infinity, max: -Infinity }; }}
                className="bg-white/5 border border-white/10 rounded px-1.5 py-0.5 text-[10px] text-white/70 outline-none"
                title="Session extremes never shrink, so one transient permanently squashes the axis">
                <option value="session">Session</option>
                <option value="frame">Frame</option>
              </select>
            </div>
          </>
        )}

        <div className="flex-1" />

        {/* Reset scale */}
        <button
          onClick={() => {
            sessionY.current = { min: Infinity, max: -Infinity };
            if (manualScale && onScaleRangeChange) onScaleRangeChange({ ...scaleRange, dynamic: true });
          }}
          className="px-2 py-0.5 rounded text-[9px] text-white/40 border border-white/10 hover:text-white/70 hover:border-white/20 transition-all"
        >
          Reset Scale
        </button>

        {/* Reset zoom */}
        <button
          onClick={() => setTraceView({ xMin: 0, xMax: 1, yMin: 0, yMax: 1, autoY: true })}
          className="px-2 py-0.5 rounded text-[9px] text-white/40 border border-white/10 hover:text-white/70 hover:border-white/20 transition-all"
        >
          Reset Zoom
        </button>
      </div>

      {/* Range Profile (trace) */}
      <div className="relative flex-1 min-h-0">
        <canvas
          ref={rangeCanvasRef}
          className="absolute inset-0 w-full h-full cursor-crosshair"
          onMouseMove={(e) => {
            const rect = e.currentTarget.getBoundingClientRect();
            setCrosshairTrace({ x: e.clientX - rect.left, y: e.clientY - rect.top });
          }}
          onMouseLeave={() => setCrosshairTrace(null)}
          onWheel={handleWheel}
          onMouseDown={handleMouseDown}
        />
        {/* Scale toggle button */}
        <button
          onClick={() => {
            setScaleMode(m => m === 'db' ? 'linear' : 'db');
            sessionY.current = { min: Infinity, max: -Infinity };
            // Pinned limits are in the units we are leaving — drop back to dynamic.
            if (manualScale && onScaleRangeChange) onScaleRangeChange({ ...scaleRange, dynamic: true });
          }}
          className={cn(
            'absolute bottom-10 left-14 px-2 py-1 rounded text-[9px] font-medium uppercase tracking-wider transition-all border z-10',
            scaleMode === 'db'
              ? 'bg-[#D1855C]/20 text-[#D1855C] border-[#D1855C]/30'
              : 'bg-[#6B9BD2]/20 text-[#6B9BD2] border-[#6B9BD2]/30'
          )}
        >
          {isDiff ? (scaleMode === 'db' ? '\u0394dB' : '\u0394LIN') : scaleMode === 'db' ? 'dB' : 'LIN'}
        </button>
        {/* Range scale toggle button (only in bscan mode) */}
        {onRangeScaleToggle && (
          <button
            onClick={onRangeScaleToggle}
            className="absolute bottom-10 left-28 px-2 py-1 rounded text-[9px] font-medium uppercase tracking-wider transition-all border z-10 bg-white/10 text-white/70 border-white/20 hover:text-white hover:border-white/40"
          >
            {rangeScale.max <= 0.5 ? '0.3m' : '3m'}
          </button>
        )}
      </div>

      {/* Waterfall */}
      {!hideWaterfall && (
        <div className="relative border-t border-white/5" style={{ flex: '0 0 45%' }}>
          <canvas
            ref={waterfallCanvasRef}
            className="absolute inset-0 w-full h-full cursor-crosshair"
            onMouseMove={(e) => {
              const rect = e.currentTarget.getBoundingClientRect();
              setCrosshairWaterfall({ x: e.clientX - rect.left, y: e.clientY - rect.top });
            }}
            onMouseLeave={() => setCrosshairWaterfall(null)}
          />
          {/* Snapshot the raw complex ring buffer for the Imaging Bench */}
          <button
            onClick={handleExportWaterfall}
            title={rawCount > 0 ? `Export ${rawCount} raw sweeps` : 'No sweeps buffered'}
            className={cn(
              'absolute bottom-10 left-14 px-2 py-1 rounded text-[9px] font-medium uppercase tracking-wider transition-all border z-10',
              rawCount > 0
                ? 'bg-white/10 text-white/70 border-white/20 hover:text-white hover:border-white/40'
                : 'bg-white/5 text-white/20 border-white/10 opacity-40 pointer-events-none'
            )}
          >
            Export
          </button>
        </div>
      )}
    </div>
  );
}
