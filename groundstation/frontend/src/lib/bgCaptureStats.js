// Per-position statistics for BG-model training captures.
//
// Positions are static (one capture = N sweeps at one standoff), so the sweeps
// within a capture are replicas. That lets us separate the repeatable part of
// h_cal from the sweep-to-sweep noise, which a training loss can use to
// downweight noisy positions.

const SPEED_OF_LIGHT = 299792458;

function ifft(re, im) {
  const n = re.length;
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
    const ang = 2 * Math.PI / len;
    const wr = Math.cos(ang), wi = Math.sin(ang);
    for (let i = 0; i < n; i += len) {
      let cr = 1, ci = 0;
      const half = len / 2;
      for (let k = 0; k < half; k++) {
        const ur = re[i + k], ui = im[i + k];
        const vr = re[i + k + half] * cr - im[i + k + half] * ci;
        const vi = re[i + k + half] * ci + im[i + k + half] * cr;
        re[i + k] = ur + vr; im[i + k] = ui + vi;
        re[i + k + half] = ur - vr; im[i + k + half] = ui - vi;
        const ncr = cr * wr - ci * wi;
        ci = cr * wi + ci * wr; cr = ncr;
      }
    }
  }
  for (let i = 0; i < n; i++) { re[i] /= n; im[i] /= n; }
}

// Radar-derived range of the dominant return, from the coherent mean spectrum.
// Diagnostic only: nothing consumes this. It exists so the exported dataset
// carries a second, independent standoff estimate to check the lidar against.
function dominantRange(mRe, mIm, stepSizeHz, rangeOffsetM) {
  if (!stepSizeHz) return null;
  const S = mRe.length;
  const NFFT = 2048;
  const re = new Float64Array(NFFT);
  const im = new Float64Array(NFFT);
  for (let i = 0; i < S; i++) {
    const w = 0.5 - 0.5 * Math.cos(2 * Math.PI * i / (S - 1)); // Hann
    re[i] = mRe[i] * w;
    im[i] = mIm[i] * w;
  }
  ifft(re, im);

  const half = NFFT / 2;
  let peak = 1, peakMag = -1;
  for (let i = 1; i < half - 1; i++) {
    const mag = Math.hypot(re[i], im[i]);
    if (mag > peakMag) { peakMag = mag; peak = i; }
  }
  const a = Math.hypot(re[peak - 1], im[peak - 1]);
  const b = peakMag;
  const c = Math.hypot(re[peak + 1], im[peak + 1]);
  const denom = a - 2 * b + c;
  const delta = denom !== 0 ? 0.5 * (a - c) / denom : 0;

  const maxRange = SPEED_OF_LIGHT / (2 * stepSizeHz);
  return ((peak + delta) / NFFT) * maxRange - (rangeOffsetM || 0);
}

export function computeCaptureStats(samples) {
  const valid = samples.filter(s => s.h_cal_real && s.h_cal_imag);
  if (valid.length === 0) return null;

  const n = valid.length;
  const S = valid[0].h_cal_real.length;

  // Coherent (complex) mean across sweeps: the training target for this position
  const mRe = new Float64Array(S);
  const mIm = new Float64Array(S);
  for (const s of valid) {
    for (let i = 0; i < S; i++) { mRe[i] += s.h_cal_real[i]; mIm[i] += s.h_cal_imag[i]; }
  }
  for (let i = 0; i < S; i++) { mRe[i] /= n; mIm[i] /= n; }

  // Per-frequency noise power about that mean
  const varF = new Float64Array(S);
  for (const s of valid) {
    for (let i = 0; i < S; i++) {
      const dr = s.h_cal_real[i] - mRe[i];
      const di = s.h_cal_imag[i] - mIm[i];
      varF[i] += dr * dr + di * di;
    }
  }
  const dof = Math.max(1, n - 1);
  for (let i = 0; i < S; i++) varF[i] /= dof;

  let sigPow = 0, noisePow = 0;
  for (let i = 0; i < S; i++) {
    sigPow += mRe[i] * mRe[i] + mIm[i] * mIm[i];
    noisePow += varF[i];
  }
  // With ONE sweep the variance about the mean is exactly zero -- the sample IS
  // the mean -- so these are 0/0, and the `|| 1e-30` guard below turns that into
  // ~318 dB and a coherence of exactly 1. That is not a very good position, it
  // is an unmeasurable one, and reporting it as the best score in the set is
  // actively misleading: the panel coloured such bins green and the coverage
  // chart scaled its whole SNR axis to 318 dB, flattening every real bar.
  // Continuous capture makes single-sweep bins common (the fast middle of a
  // pass), where the static protocol never produced one. Undefined, so null.
  const canScore = n >= 2;
  const coherence = canScore ? sigPow / (sigPow + noisePow || 1e-30) : null;
  const snrDb = canScore ? 10 * Math.log10(sigPow / (noisePow || 1e-30)) : null;

  // Consecutive sweep-pair complex correlation (same metric as the Phase Test)
  let corrSum = 0, corrN = 0;
  for (let k = 0; k < valid.length - 1; k++) {
    let dotRe = 0, dotIm = 0, magA = 0, magB = 0;
    const A = valid[k], B = valid[k + 1];
    for (let i = 0; i < S; i++) {
      const aR = A.h_cal_real[i], aI = A.h_cal_imag[i];
      const bR = B.h_cal_real[i], bI = B.h_cal_imag[i];
      dotRe += aR * bR + aI * bI;
      dotIm += aI * bR - aR * bI;
      magA += aR * aR + aI * aI;
      magB += bR * bR + bI * bI;
    }
    corrSum += Math.hypot(dotRe, dotIm) / (Math.sqrt(magA) * Math.sqrt(magB) + 1e-30);
    corrN++;
  }

  const dists = samples.map(s => s.lidar_standoff_mm).filter(v => v != null);
  const dMean = dists.length ? dists.reduce((a, b) => a + b, 0) / dists.length : null;
  const dStd = dists.length > 1
    ? Math.sqrt(dists.reduce((a, b) => a + (b - dMean) ** 2, 0) / (dists.length - 1))
    : 0;

  const ref = valid[0];
  return {
    sweepCount: n,
    numSteps: S,
    h_mean_real: Array.from(mRe),
    h_mean_imag: Array.from(mIm),
    noise_var: Array.from(varF),
    snrDbPerSweep: snrDb,
    // Averaging n sweeps cuts the incoherent power by n
    snrDbAveraged: canScore ? snrDb + 10 * Math.log10(n) : null,
    coherence,
    sweepCorrelation: corrN > 0 ? corrSum / corrN : null,
    standoffMm: dMean,
    standoffStdMm: dStd,
    standoffN: dists.length,
    radarRangeM: dominantRange(mRe, mIm, ref.step_size, ref.range_offset),
  };
}

// Spacing guidance. An echo with path multiplier alpha oscillates in standoff
// with period c / (2*f*(alpha-1)); worst case is the top of the band with
// alpha=3 (triple bounce). Sample above period/2 and it aliases onto a wrong
// spatial frequency, which corrupts a fit rather than merely missing detail.
export function spacingLimits(stopFreqMhz) {
  const fMax = (stopFreqMhz || 5000) * 1e6;
  const periodMm = (SPEED_OF_LIGHT / (4 * fMax)) * 1000;
  return {
    periodMm,
    goodMm: periodMm / 3,   // ~3 samples/period, comfortable
    aliasMm: periodMm / 2,  // Nyquist: above this, alpha=3 folds
  };
}

function avgStandoff(samples) {
  if (!samples || !samples.length) return null;
  const d = samples.map(s => s.lidar_standoff_mm).filter(v => v != null);
  return d.length ? d.reduce((a, b) => a + b, 0) / d.length : null;
}

export function analyzeCoverage(captures, stopFreqMhz) {
  const positions = captures
    .map((c, index) => {
      const d = (c.stats && c.stats.standoffMm != null) ? c.stats.standoffMm : avgStandoff(c.samples);
      return d == null ? null : { index, mm: d, stats: c.stats || null };
    })
    .filter(Boolean)
    .sort((a, b) => a.mm - b.mm);

  const limits = spacingLimits(stopFreqMhz);
  if (positions.length < 2) {
    return { positions, gaps: [], spanMm: 0, maxGap: null, medianGap: null, worstGapAt: null, limits };
  }

  const gaps = [];
  for (let i = 1; i < positions.length; i++) {
    gaps.push({ mm: positions[i].mm - positions[i - 1].mm, lo: positions[i - 1].mm, hi: positions[i].mm });
  }
  const sorted = gaps.map(g => g.mm).sort((a, b) => a - b);
  const worst = gaps.reduce((a, b) => (b.mm > a.mm ? b : a), gaps[0]);

  return {
    positions,
    gaps,
    spanMm: positions[positions.length - 1].mm - positions[0].mm,
    maxGap: worst.mm,
    medianGap: sorted[Math.floor(sorted.length / 2)],
    // Midpoint of the largest gap: where the next capture should go
    worstGapAt: (worst.lo + worst.hi) / 2,
    limits,
  };
}
