import { windowFn } from './imagingEffects';

const SPEED_OF_LIGHT = 299792458;

function fftInPlace(re, im) {
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

function ifftInPlace(re, im) {
  const n = re.length;
  for (let i = 0; i < n; i++) im[i] = -im[i];
  fftInPlace(re, im);
  for (let i = 0; i < n; i++) {
    re[i] /= n;
    im[i] = -im[i] / n;
  }
}

function complexSvdFilter(hCalReals, hCalImags, numPositions, numSteps, k, strength) {
  // SVD clutter filter on complex h_cal matrix (numPositions × numSteps)
  // Removes the first k spatial components (walls, static clutter)
  if (k < 1 || numPositions < 2) return { reals: hCalReals, imags: hCalImags };

  // Work with flat arrays: re[p*numSteps + s], im[p*numSteps + s]
  const re = new Float64Array(numPositions * numSteps);
  const im = new Float64Array(numPositions * numSteps);
  for (let p = 0; p < numPositions; p++) {
    for (let s = 0; s < numSteps; s++) {
      re[p * numSteps + s] = hCalReals[p][s];
      im[p * numSteps + s] = hCalImags[p][s];
    }
  }

  const s = Math.max(0, Math.min(1, strength));

  for (let comp = 0; comp < k; comp++) {
    // Power iteration to find dominant singular vector
    // Right singular vector v (complex, length numSteps)
    let vRe = new Float64Array(numSteps);
    let vIm = new Float64Array(numSteps);
    for (let i = 0; i < numSteps; i++) { vRe[i] = Math.cos(i); vIm[i] = Math.sin(i); }
    let norm = 0;
    for (let i = 0; i < numSteps; i++) norm += vRe[i] * vRe[i] + vIm[i] * vIm[i];
    norm = Math.sqrt(norm);
    for (let i = 0; i < numSteps; i++) { vRe[i] /= norm; vIm[i] /= norm; }

    let uRe = new Float64Array(numPositions);
    let uIm = new Float64Array(numPositions);
    let sigma = 0;

    for (let iter = 0; iter < 50; iter++) {
      // u = A * v (matrix × vector)
      for (let p = 0; p < numPositions; p++) {
        let sumR = 0, sumI = 0;
        for (let j = 0; j < numSteps; j++) {
          const ar = re[p * numSteps + j], ai = im[p * numSteps + j];
          sumR += ar * vRe[j] - ai * vIm[j];
          sumI += ar * vIm[j] + ai * vRe[j];
        }
        uRe[p] = sumR;
        uIm[p] = sumI;
      }

      // sigma = ||u||
      sigma = 0;
      for (let p = 0; p < numPositions; p++) sigma += uRe[p] * uRe[p] + uIm[p] * uIm[p];
      sigma = Math.sqrt(sigma);
      if (sigma < 1e-10) break;
      for (let p = 0; p < numPositions; p++) { uRe[p] /= sigma; uIm[p] /= sigma; }

      // v_new = A^H * u (adjoint × vector)
      let vNewRe = new Float64Array(numSteps);
      let vNewIm = new Float64Array(numSteps);
      for (let j = 0; j < numSteps; j++) {
        let sumR = 0, sumI = 0;
        for (let p = 0; p < numPositions; p++) {
          const ar = re[p * numSteps + j], ai = im[p * numSteps + j];
          // conj(A[p,j]) * u[p] = (ar - j*ai) * (uRe + j*uIm)
          sumR += ar * uRe[p] + ai * uIm[p];
          sumI += ar * uIm[p] - ai * uRe[p];
        }
        vNewRe[j] = sumR;
        vNewIm[j] = sumI;
      }

      // Normalize v
      norm = 0;
      for (let j = 0; j < numSteps; j++) norm += vNewRe[j] * vNewRe[j] + vNewIm[j] * vNewIm[j];
      norm = Math.sqrt(norm);
      if (norm < 1e-10) break;
      for (let j = 0; j < numSteps; j++) { vNewRe[j] /= norm; vNewIm[j] /= norm; }

      // Check convergence
      let diff = 0;
      for (let j = 0; j < numSteps; j++) diff += (vNewRe[j] - vRe[j]) ** 2 + (vNewIm[j] - vIm[j]) ** 2;
      vRe = vNewRe;
      vIm = vNewIm;
      if (diff < 1e-10) break;
    }

    // Subtract: A -= s * sigma * u * v^H
    for (let p = 0; p < numPositions; p++) {
      for (let j = 0; j < numSteps; j++) {
        // sigma * u[p] * conj(v[j]) = sigma * (uRe+j*uIm)*(vRe-j*vIm)
        const outerRe = uRe[p] * vRe[j] + uIm[p] * vIm[j];
        const outerIm = uIm[p] * vRe[j] - uRe[p] * vIm[j];
        re[p * numSteps + j] -= s * sigma * outerRe;
        im[p * numSteps + j] -= s * sigma * outerIm;
      }
    }
  }

  // Rebuild arrays
  const filteredReals = [];
  const filteredImags = [];
  for (let p = 0; p < numPositions; p++) {
    const r = new Array(numSteps);
    const i = new Array(numSteps);
    for (let j = 0; j < numSteps; j++) {
      r[j] = re[p * numSteps + j];
      i[j] = im[p * numSteps + j];
    }
    filteredReals.push(r);
    filteredImags.push(i);
  }
  return { reals: filteredReals, imags: filteredImags };
}

// The window used to be a hardcoded Hanning. It is a parameter now, defaulting to
// rectangular for range RESOLUTION.
//
// WHICH WINDOW IS ACTUALLY BEST HERE IS UNSETTLED -- two measurements on the same
// 60-position 10 mm-pitch scan disagreed. Evaluating coherence at one point with the
// full aperture and no debiasing put rectangular ahead (target 0.654 / kaiser b3 0.627
// / hanning 0.615, separation from clutter 0.273 / 0.248 / 0.234). Running the shipped
// pipeline end to end -- reconstruction grid, debiased coherence -- reversed it
// (separation rectangular 0.261 / kaiser 0.314 / hanning 0.362). The second is the more
// relevant measurement but it is still n=1 scan, n=1 target, so the control is exposed
// rather than the answer baked in: A/B it on a target-in / target-out pair.
//
// Note this is a different question from the range-profile DISPLAY's window, where the
// problem is a strong return's sidelobes leaking into a distant bin. Do not unify them.
function computeComplexRangeProfile(hCalReal, hCalImag, numSteps, freqStepHz, rangeOffset, win) {
  const nfftMin = numSteps * 4;
  const nfft = 1 << Math.ceil(Math.log2(nfftMin));
  const re = new Float64Array(nfft);
  const im = new Float64Array(nfft);
  for (let i = 0; i < numSteps; i++) {
    const w = win ? win[i] : 1;
    re[i] = hCalReal[i] * w;
    im[i] = hCalImag[i] * w;
  }
  ifftInPlace(re, im);

  const maxRange = SPEED_OF_LIGHT / (2 * freqStepHz);
  const half = nfft / 2;
  const distRe = [];
  const distIm = [];
  const distances = [];
  for (let i = 0; i < half; i++) {
    const d = (i / nfft) * maxRange - rangeOffset;
    if (d >= 0) {
      distRe.push(re[i]);
      distIm.push(im[i]);
      distances.push(d);
    }
  }
  return { re: distRe, im: distIm, distances };
}

// Last usable distance on the grid computeComplexRangeProfile() produces, without
// paying to build it. Deliberately sited next to that function: any change to nfft or
// to the `d >= 0` clip has to be made in both or the reachable-depth check below stops
// describing the array the reconstruction actually indexes.
function profileMaxDist(numSteps, freqStepHz, rangeOffset) {
  const nfft = 1 << Math.ceil(Math.log2(numSteps * 4));
  const maxRange = SPEED_OF_LIGHT / (2 * freqStepHz);
  return ((nfft / 2 - 1) / nfft) * maxRange - rangeOffset;
}

const RAY_NQ = 256;

// One-way OPTICAL (air-equivalent) path from an antenna standing off `s` in air, through
// `T` of wall at index `nw`, to a point at depth `z` below the wall face -- with air
// again beyond the back face, and Snell obeyed at both interfaces.
//
// Parameterised by the ray invariant q = n_i*sin(theta_i), which Snell makes constant
// across the whole stack. Sweeping q from 0 traces the ray fan outward from broadside,
// and both the lateral offset X(q) and the optical length L(q) increase monotonically
// with it -- so ONE table per (standoff, depth) inverts by interpolation for every
// lateral pixel in that row. Root-finding the crossing points per pixel gives the same
// answer at roughly 30x the inner-loop cost, which this cannot afford.
//
// Why it is worth doing at all: the straight-ray approximation (standoff added as a pure
// delay) is only good while the air gap is negligible in ANGLE, and it is not. Snell
// turns a 27 deg ray inside a 29 cm wall into a ~76 deg ray in the air gap, so the true
// crossing point moves ~1.6 cm sideways and the path shortens by ~2.4 mm -- 29 degrees of
// two-way phase at 5 GHz, applied exactly to the wide-angle contributions that
// cross-range resolution depends on.
//
// Note also what the fan cannot contain: sin(theta) inside the wall is capped at q/nw,
// so at er 4.5 no ray propagates further than ~28 deg off normal INSIDE the wall however
// wide the aperture gets. That is a physical aperture limit rather than a modelling
// choice, and it is consistent with the measured saturation of coherent gain at ~15 cm
// of one-sided aperture on this 29 cm wall.
function buildRayTable(s, z, T, nw, outX, outL) {
  const dA1 = s;                       // air, antenna -> wall face
  const dW = Math.min(z, T);           // inside the wall
  const dA2 = Math.max(0, z - T);      // air again, beyond the back face
  let qMax = (dA1 > 0 || dA2 > 0) ? 1 : nw;
  if (dW > 0 && nw < qMax) qMax = nw;
  for (let i = 0; i < RAY_NQ; i++) {
    // sin-spaced in angle, so samples crowd towards grazing where X(q) blows up.
    const q = qMax * Math.sin((i / (RAY_NQ - 1)) * (Math.PI / 2) * 0.9995);
    let X = 0;
    let L = 0;
    if (dA1 > 0) { const c = Math.sqrt(1 - q * q); X += dA1 * q / c; L += dA1 / c; }
    if (dW > 0) { const sw = q / nw; const c = Math.sqrt(1 - sw * sw); X += dW * sw / c; L += nw * dW / c; }
    if (dA2 > 0) { const c = Math.sqrt(1 - q * q); X += dA2 * q / c; L += dA2 / c; }
    outX[i] = X;
    outL[i] = L;
  }
}

// Invert the table: lateral offset -> optical path. Returns -1 when the offset is past
// the fan's reach, which the caller treats as "no ray gets there" and skips -- the
// honest answer, rather than extrapolating a path that does not exist.
function rayLookup(outX, outL, dx) {
  const last = RAY_NQ - 1;
  if (dx <= outX[0]) return outL[0];
  if (dx >= outX[last]) return -1;
  let lo = 0;
  let hi = last;
  while (hi - lo > 1) {
    const mid = (lo + hi) >> 1;
    if (outX[mid] <= dx) lo = mid; else hi = mid;
  }
  const span = outX[hi] - outX[lo];
  const t = span > 0 ? (dx - outX[lo]) / span : 0;
  return outL[lo] + t * (outL[hi] - outL[lo]);
}

self.onmessage = function (e) {
  const { bscanData, bscanParams } = e.data;
  const t0 = performance.now();

  const { stepSize, maxDepth, aperture, coherent } = bscanParams;
  const pixelsX = 100;
  const pixelsZ = 100;

  // NOTE: `bscanData` here is NOT a full C-scan record list. `useSarWorker`
  // projects it down to the handful of fields read below before posting, because
  // structured-cloning the whole thing (every raw sweep of every cell) reached
  // 282 MB and 3.2 s on a long scan and threw DataCloneError. If you need
  // another field, add it to SAR_INPUT_FIELDS in hooks/useSarWorker.js -- a
  // field that is not projected arrives as `undefined`, silently.
  const numPositions = bscanData.length;
  if (numPositions < 2 || !bscanData[0].magnitudes || !bscanData[0].distances) {
    self.postMessage({ type: 'result', result: null });
    return;
  }

  // Whether the coherent path can actually run. The `&&` chain used to be assigned
  // straight to hasHcal, which made it the h_cal_imag ARRAY rather than a flag --
  // harmless for branching, but it was posted back as `result.coherent`, so the field
  // that says which reconstruction ran was 51 floats and any consumer reading it as a
  // boolean would have been misled. Coerced here, and hoisted above `layered` and the
  // reachable-depth maths because both of those have to know which path will run.
  const hasHcal = !!(coherent && bscanData[0].h_cal_real && bscanData[0].h_cal_imag);

  // Relative permittivity of the medium. This WAS ABSENT ENTIRELY until 2026-09-03,
  // i.e. every image was back-projected at the speed of light in air: the hyperbola
  // being matched had the wrong curvature so nothing focused properly, and the depth
  // axis read sqrt(er) times too deep. Default 4.5 = dry brick, cross-checked against
  // a real scan -- a 29 cm wall whose back face landed at 63.2 cm of apparent range
  // gives sqrt(er) = 62.8/29 = 2.16, er = 4.68.
  const epsilonR = bscanParams.epsilonR > 0 ? bscanParams.epsilonR : 1;
  const n = Math.sqrt(epsilonR);

  // Per-position standoff in metres. Two sources, chosen by bscanParams.autoStandoff:
  //
  //   auto (default) -- each cell's own recorded lidar reading. Without it the
  //     back-projection assumes every antenna position sat exactly on the wall face,
  //     and a scan whose standoff wanders is defocused by the resulting phase error.
  //     Measured by injecting known scatter into a real scan and re-migrating both
  //     ways -- target coherence UNCORRECTED 0.591 flat -> 0.557 at 3 mm sigma ->
  //     0.358 at 5 mm -> 0.156 at 10 mm, while CORRECTED it holds 0.591 all the way
  //     to 30 mm. The lidar is far better than it needs to be here: adding 0.5 mm of
  //     noise to the correction changed nothing. This is what lets the rig be
  //     imprecise in standoff.
  //
  //   manual -- one operator-entered standoff applied to every position. This exists
  //     because the lidar can be WRONG rather than merely noisy, and when it is there
  //     is no way to recover the scan from the recorded column. Diagnosed on
  //     rebar1.json (2026-09-04): 32 of 43 cells recorded ~670 mm while the strongest
  //     scatterer in the reconstruction sits at ~0.41 m of apparent range -- i.e. 26 cm
  //     IN FRONT of the claimed wall face, which is impossible. The lidar had been
  //     shooting past the target for most of the traverse and only caught it on cells
  //     15-25 (170-222 mm). Forcing a consistent standoff took max coherence from 0.72
  //     to 0.96 and the reachable depth from 2.8 cm to 26 cm.
  const autoStandoff = bscanParams.autoStandoff !== false;
  const manualStandoffM = Math.max(0, (bscanParams.manualStandoffMm || 0) / 1000);

  // What the RECORD says, computed whichever source is active -- in manual mode the
  // operator is choosing a number to replace these with, so they are exactly what needs
  // to be on screen at that moment.
  const known = [];
  for (let p = 0; p < numPositions; p++) {
    const mm = bscanData[p].lidar_standoff_mm;
    if (mm !== null && mm !== undefined && isFinite(mm)) known.push(mm);
  }
  known.sort((a, b) => a - b);
  const standoffN = known.length;
  // Median, not mean, so one bad cell cannot drag the fill value.
  const medianMm = known.length
    ? (known.length % 2
      ? known[(known.length - 1) / 2]
      : 0.5 * (known[known.length / 2 - 1] + known[known.length / 2]))
    : null;
  const standoffMinMm = known.length ? known[0] : null;
  const standoffMaxMm = known.length ? known[known.length - 1] : null;
  const standoffSpreadMm = known.length ? standoffMaxMm - standoffMinMm : 0;

  const standoffs = [];
  if (autoStandoff) {
    // Cells with no reading are filled with the median of those that have one. Zero was
    // the old fallback and it is the wrong shape of guess: in a scan sitting at 670 mm
    // it injects a 0.67 m path error at exactly the position that knew least about
    // itself, which is far worse than assuming it sat where its neighbours did.
    const fillM = (medianMm === null ? 0 : medianMm) / 1000;
    for (let p = 0; p < numPositions; p++) {
      const mm = bscanData[p].lidar_standoff_mm;
      standoffs.push((mm === null || mm === undefined || !isFinite(mm)) ? fillM : mm / 1000);
    }
  } else {
    for (let p = 0; p < numPositions; p++) standoffs.push(manualStandoffM);
  }
  let maxStandoff = 0;
  for (let p = 0; p < numPositions; p++) if (standoffs[p] > maxStandoff) maxStandoff = standoffs[p];

  const distances = bscanData[0].distances;
  const numBins = distances.length;
  const numSteps = hasHcal ? bscanData[0].h_cal_real.length : 0;
  const freqStepHz = bscanData[0].step_size || 20000000;
  const rangeOffset = bscanData[0].range_offset || 0.378;

  // The two paths index DIFFERENT profiles -- the coherent one rebuilds its own
  // zero-padded IFFT from h_cal, the incoherent one uses the magnitudes/distances that
  // arrived -- so the reachable depth is derived from whichever grid is about to be
  // read, not from whatever happens to be sitting on bscanData[0]. They agree to ~2 mm
  // on the current sweep plan, but they are not the same axis and a different step size
  // would separate them.
  const apparentAvailable = hasHcal
    ? profileMaxDist(numSteps, freqStepHz, rangeOffset)
    : distances[numBins - 1];

  // Is the standoff column believable AGAINST THIS RECORD? Two tests, both cheap and
  // both grounded in the sweep's own reach rather than in a taste threshold:
  //
  //   over range -- the claimed wall face sits at or beyond the last range bin, so the
  //     sweep contains no wall and no target. Physically impossible for a scan that
  //     produced a range profile at all.
  //   spread -- the min-to-max standoff across the aperture eats more than a quarter
  //     of the record's total reach. A real traverse varies by millimetres (the
  //     2026-09-03 sartt.json scan held +/-1.5 mm; the worst tilted rover set managed
  //     30 mm over a metre) and the correction exists precisely to absorb that. Half a
  //     metre of spread is not a rig that wandered, it is a lidar that stopped seeing
  //     the wall -- which is exactly what rebar1.json recorded.
  //
  // Flagged, not silently repaired. The operator has to decide what the standoff
  // really was; the panel offers the manual override for saying so.
  const standoffOverRange = maxStandoff >= apparentAvailable;
  const standoffSuspect = autoStandoff
    && (standoffOverRange || standoffSpreadMm / 1000 > 0.25 * apparentAvailable);

  // maxDepth is TRUE depth below the wall face now, not apparent range. Reaching
  // depth z needs apparent range standoff + n*z, so the record itself bounds how deep
  // the grid can go; the request is clipped and flagged rather than silently imaging
  // past the end of the profile.
  // Operator-measured wall thickness. Only used by the layered model, which needs to
  // know where the dielectric STOPS -- beyond the back face it is air again, and a
  // uniform-dielectric model puts anything back there at the wrong depth and the wrong
  // hyperbola curvature. 0 means "not measured", and the layered path stays off.
  const wallT = Math.max(0, (bscanParams.wallThickness || 0) / 100);
  // Coherent-only, and enforced HERE rather than left to the reconstruction loop. It
  // used to be `refraction && wallT > 0` alone, which let the incoherent path take the
  // layered reachable depth -- deeper, because the leg beyond the back face travels at
  // c -- while still mapping range with the straight ray. Measured on a real
  // 101-position scan: the deepest row went from -39.1 dB to -85.4 dB, i.e. almost
  // every deep pixel fell to the "no contribution" sentinel, and the result still
  // reported layered:true so the display captioned a straight-ray image "layered".
  const layered = !!bscanParams.refraction && wallT > 0 && hasHcal;

  const requested = (maxDepth || 30) / 100;
  let reachable;
  if (layered) {
    // Broadside optical path to the back face, then air-for-air beyond it.
    const atBack = maxStandoff + n * wallT;
    reachable = apparentAvailable >= atBack
      ? wallT + (apparentAvailable - atBack)
      : (apparentAvailable - maxStandoff) / n;
  } else {
    reachable = (apparentAvailable - maxStandoff) / n;
  }
  reachable = Math.max(0.01, reachable);
  const depthMax = Math.min(requested, reachable);
  const depthClipped = requested > reachable + 1e-9;

  // One depth ramp for both paths. The 0.005 m floor is there because a zero-depth
  // pixel degenerates the layered ray table -- every leg has zero length, so the fan
  // has no lateral reach and the entire row is skipped -- but it used to be applied
  // only in the coherent branch, so the two modes reconstructed row 0 at 0.5 cm and
  // 0.0 cm while the axis labelled both 0.0.
  const depthAt = (zi) => Math.max(0.005, (zi / (pixelsZ - 1)) * depthMax);

  // Progress is a progress bar, not telemetry: one message per position meant 101
  // setState round-trips per reconstruction on a 101-cell scan, every one of them
  // queued behind a ~30 ms compute and re-rendering the whole app.
  const progressEvery = Math.max(1, Math.ceil(numPositions / 20));

  const apertureLength = (numPositions - 1) * stepSize / 100;

  const antennaX = [];
  for (let p = 0; p < numPositions; p++) {
    antennaX.push(p * stepSize / 100);
  }

  const image = new Float64Array(pixelsX * pixelsZ);
  const coherence = new Float64Array(pixelsX * pixelsZ);

  // Coherence is only a statement about the aperture if the aperture was actually
  // there. Pixels near the far corners are reached by only a handful of positions --
  // the rest need an apparent range the sweep does not contain -- and a sum of two or
  // three contributions is coherent by chance, not by focusing. Caught in testing: the
  // raw statistic peaked at 0.997 in the deepest corner, above the real target.
  const minContrib = Math.max(5, Math.ceil(0.25 * numPositions));


  if (hasHcal) {
    // Coherent SAR: complex range profiles + phase-compensated summation
    const startFreq = bscanParams.startFreq ? bscanParams.startFreq * 1e6 : 2e9;
    const k_start = 2 * Math.PI * startFreq / SPEED_OF_LIGHT;

    // Apply complex SVD clutter filter to h_cal before range profile computation
    let hCalReals = bscanData.map(p => p.h_cal_real);
    let hCalImags = bscanData.map(p => p.h_cal_imag);
    if (bscanParams.svdEnabled && bscanParams.svdK >= 1) {
      const filtered = complexSvdFilter(hCalReals, hCalImags, numPositions, numSteps, bscanParams.svdK, bscanParams.svdStrength);
      hCalReals = filtered.reals;
      hCalImags = filtered.imags;
    }

    const win = windowFn(bscanParams.windowType || 'rectangular', bscanParams.kaiserBeta || 3)(numSteps);

    // Compute complex range profiles for all positions
    const crps = [];
    for (let p = 0; p < numPositions; p++) {
      crps.push(computeComplexRangeProfile(
        hCalReals[p], hCalImags[p],
        numSteps, freqStepHz, rangeOffset, win
      ));
    }

    const crpDists = crps[0].distances;
    const crpNumBins = crpDists.length;
    const crpDistStart = crpDists[0];
    const crpDistStep = crpNumBins > 1 ? (crpDists[crpNumBins - 1] - crpDistStart) / (crpNumBins - 1) : 1;

    // The lookup limit is the WHOLE profile, not the display depth. With a dielectric
    // the apparent range to a pixel at depth z is standoff + n*z, which already runs
    // past z, and the hyperbola tails at large |dx| run further still -- clipping at
    // the display depth would truncate exactly the wide-angle contributions that
    // focusing depends on.
    const crpEndBin = crpNumBins - 1;

    // POSITION-OUTER, deliberately. The layered ray table depends only on (standoff,
    // depth), so building it once per (position, depth) row and reusing it across the
    // row's lateral pixels is what keeps refraction affordable; with the original
    // depth-outer order the table would be rebuilt for every pixel. The sum is
    // associative so the reordering is exact -- verified against the previous order.
    const accRe = new Float64Array(pixelsX * pixelsZ);
    const accIm = new Float64Array(pixelsX * pixelsZ);
    const accAbs = new Float64Array(pixelsX * pixelsZ);
    const accN = new Int32Array(pixelsX * pixelsZ);

    const tblX = new Float64Array(RAY_NQ);
    const tblL = new Float64Array(RAY_NQ);

    for (let p = 0; p < numPositions; p++) {
      const cre = crps[p].re;
      const cim = crps[p].im;
      const sp = standoffs[p];

      for (let zi = 0; zi < pixelsZ; zi++) {
        const depth = depthAt(zi);
        if (layered) buildRayTable(sp, depth, wallT, n, tblX, tblL);
        const row = zi * pixelsX;

        for (let xi = 0; xi < pixelsX; xi++) {
          const lateral = (xi / (pixelsX - 1)) * apertureLength;
          const dx = lateral - antennaX[p];

          let R;
          if (layered) {
            // air(standoff) / wall(wallT) / air(beyond), refracting at both faces.
            R = rayLookup(tblX, tblL, dx < 0 ? -dx : dx);
            if (R < 0) continue;
          } else {
            // Straight ray: the standoff is added as a pure delay and everything below
            // the face is treated as one infinite dielectric. Cheaper, and what this
            // worker did before the layered model existed -- kept as the A/B baseline.
            R = sp + n * Math.sqrt(dx * dx + depth * depth);
          }

          const binFloat = (R - crpDistStart) / crpDistStep;
          const binIdx = Math.floor(binFloat);
          if (binIdx < 0 || binIdx >= crpEndBin) continue;

          const frac = binFloat - binIdx;
          const valRe = cre[binIdx] * (1 - frac) + cre[binIdx + 1] * frac;
          const valIm = cim[binIdx] * (1 - frac) + cim[binIdx + 1] * frac;

          const phase = 2 * k_start * R;
          const cosP = Math.cos(phase);
          const sinP = Math.sin(phase);

          const o = row + xi;
          accRe[o] += valRe * cosP - valIm * sinP;
          accIm[o] += valRe * sinP + valIm * cosP;
          // Incoherent partner of the very same sum. |sum| / sum|.| asks a different
          // question from the amplitude: did the contributions AGREE IN PHASE (a
          // scatterer the aperture genuinely focused -> towards 1) or merely happen to
          // add up to something large (clutter -> ~0.2)?
          accAbs[o] += Math.sqrt(valRe * valRe + valIm * valIm);
          accN[o]++;
        }
      }

      if (p === numPositions - 1 || p % progressEvery === 0) {
        self.postMessage({ type: 'progress', progress: (p + 1) / numPositions });
      }
    }

    for (let i = 0; i < image.length; i++) {
      const mag = Math.sqrt(accRe[i] * accRe[i] + accIm[i] * accIm[i]);
      image[i] = 20 * Math.log10(mag + 1e-12);

      // Debias before storing. N contributions with random phases still sum to about
      // 1/sqrt(N) of their incoherent total, so the RAW ratio is not comparable between
      // pixels with different support -- and support varies a lot across the grid,
      // because the deep and off-centre pixels are the ones whose far positions fall off
      // the end of the range profile (or, under the layered model, are past the reach of
      // the refracted ray fan). Rescaling so chance maps to 0 and perfect to 1 makes the
      // number mean the same thing everywhere, which is what lets it be drawn on a fixed
      // 0-1 colour scale at all.
      const nUsed = accN[i];
      if (nUsed >= minContrib) {
        const chance = 1 / Math.sqrt(nUsed);
        const raw = accAbs[i] > 0 ? mag / accAbs[i] : 0;
        coherence[i] = Math.max(0, (raw - chance) / (1 - chance));
      } else {
        coherence[i] = 0;
      }
    }
  } else {
    // Incoherent SAR: magnitude-only with nearby-position averaging
    const distStart = distances[0];
    const endBin = numBins - 1;
    const distStep = endBin > 0 ? (distances[endBin] - distStart) / endBin : 1;
    const stepM = stepSize / 100;
    const maxDist = (aperture + 0.5) * stepM;

    for (let zi = 0; zi < pixelsZ; zi++) {
      const depth = depthAt(zi);

      for (let xi = 0; xi < pixelsX; xi++) {
        const lateral = (xi / (pixelsX - 1)) * apertureLength;
        let sum = 0;
        let count = 0;

        for (let p = 0; p < numPositions; p++) {
          const dx = lateral - antennaX[p];
          if (Math.abs(dx) > maxDist) continue;

          // Straight ray only. The layered model is coherent-only: this path sums
          // magnitudes, so it has no phase for a path-length correction to act on and
          // refraction would buy it nothing but cost.
          const range = standoffs[p] + n * Math.sqrt(dx * dx + depth * depth);
          const binFloat = (range - distStart) / distStep;
          const binIdx = Math.floor(binFloat);
          if (binIdx < 0 || binIdx >= endBin) continue;

          const frac = binFloat - binIdx;
          const mag = bscanData[p].magnitudes[binIdx] * (1 - frac) + bscanData[p].magnitudes[binIdx + 1] * frac;
          sum += mag;
          count++;
        }

        image[zi * pixelsX + xi] = count > 0 ? sum / count : -90;
      }

      if (zi % 10 === 0 || zi === pixelsZ - 1) {
        self.postMessage({ type: 'progress', progress: (zi + 1) / pixelsZ });
      }
    }
  }

  const computeTimeMs = Math.round(performance.now() - t0);

  self.postMessage({
    type: 'result',
    result: {
      image: Array.from(image),
      // Only the coherent path has phase to be coherent about; the incoherent one sums
      // magnitudes, so no such statistic exists and the display says so rather than
      // drawing a meaningless zero field.
      coherence: hasHcal ? Array.from(coherence) : null,
      pixelsX,
      pixelsZ,
      depthMax,
      apertureLength,
      numPositions,
      computeTimeMs,
      coherent: hasHcal,
      epsilonR,
      windowType: bscanParams.windowType || 'rectangular',
      standoffN,
      // Reported so the panel can say what the standoff column actually is, rather
      // than only how many cells carried one. `reachableDepth` is the un-clipped
      // bound: it stays visible even after Max Depth has been fitted down to it, so
      // the number that explains a collapsed image does not disappear along with the
      // clip warning that first announced it.
      standoffSource: autoStandoff ? 'lidar' : 'manual',
      standoffMinMm,
      standoffMaxMm,
      standoffMedianMm: medianMm,
      standoffSpreadMm,
      // What is actually being fed to the back-projection, which in manual mode is not
      // any of the three above.
      standoffAppliedMm: maxStandoff * 1000,
      standoffOverRange,
      standoffSuspect,
      reachableDepth: reachable,
      apparentAvailable,
      depthClipped,
      layered,
      wallThicknessCm: wallT * 100,
    },
  });
};
