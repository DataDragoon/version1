// Synthetic-aperture focusing along a line of positions.
//
// Lifted out of MapDisplay so the 2D Map and the C-scan plan view share ONE
// implementation, the way CFAR and the window functions were lifted into
// imagingEffects.js. Two copies of a focusing kernel would drift, and the
// failure would be invisible: both images would still look plausible while
// disagreeing about where a target is.
//
// The model is the cheap one -- incoherent (magnitude-domain) back-projection.
// For each depth `d` under position i, every neighbour within the aperture
// contributes its own trace read at the GEOMETRIC range to that point,
// R = sqrt(dx^2 + d^2), weighted by an obliquity factor (d/R)^2 that tapers
// steep angles where the antenna has no gain and the geometric lookup lands on
// unrelated clutter. It is not the coherent back-projection the SAR panel does
// (which needs phase, permittivity and refraction); it works on the magnitude
// profiles that are already on screen, and it is what the 2D Map has always
// done.

// Linear interpolation of a range profile at an arbitrary range. Off the ends
// of the record there is no measurement, so it contributes nothing rather than
// being clamped to the nearest bin -- clamping would smear the last bin across
// every depth the geometry cannot actually see.
export function interpMagnitudeAtRange(mags, dists, targetRange) {
  if (targetRange < dists[0] || targetRange > dists[dists.length - 1]) return -Infinity;
  let lo = 0, hi = dists.length - 1;
  while (lo < hi - 1) {
    const mid = (lo + hi) >> 1;
    if (dists[mid] <= targetRange) lo = mid;
    else hi = mid;
  }
  const t = (targetRange - dists[lo]) / (dists[hi] - dists[lo] + 1e-15);
  return mags[lo] + t * (mags[hi] - mags[lo]);
}

// Focused range profile at `traces[idx]`, sampled at `gateDepths` (metres).
//
// `traces` is `{ magnitudes, distances, n }` sorted by ASCENDING `n`, where `n`
// is the position's index along the line in whole steps -- capture index for a
// linear scan, grid column for a C-scan row. Keeping it as an integer times
// `stepM` rather than a precomputed metre coordinate is deliberate: the offset
// is then exactly `(n_b - n_a) * stepM`, which is what the 2D Map computed
// before this was extracted, so the extraction is bit-identical rather than
// merely equivalent. It also means a row with HOLES in it (an undone cell, a
// partial raster) still gets the right geometry, because the spacing comes from
// the column indices and not from how many entries happen to be in the array.
export function saftFocusedProfile(traces, idx, gateDepths, stepM, halfAperture) {
  const self = traces[idx];
  const out = new Float64Array(gateDepths.length);
  const lo = self.n - halfAperture;
  const hi = self.n + halfAperture;

  for (let di = 0; di < gateDepths.length; di++) {
    const d = gateDepths[di];
    if (d < 1e-6) { out[di] = -Infinity; continue; }
    let weightedSum = 0;

    for (let k = 0; k < traces.length; k++) {
      const t = traces[k];
      if (t.n < lo) continue;
      if (t.n > hi) break;              // sorted by n, so nothing further qualifies
      if (!t.magnitudes || !t.distances) continue;

      const dx = (t.n - self.n) * stepM;
      const R = Math.sqrt(dx * dx + d * d);
      const obliquity = (d * d) / (R * R);

      const mag = interpMagnitudeAtRange(t.magnitudes, t.distances, R);
      if (mag > -Infinity) {
        const linear = Math.pow(10, mag / 20);
        weightedSum += linear * obliquity;
      }
    }

    out[di] = weightedSum > 0 ? 20 * Math.log10(weightedSum + 1e-12) : -Infinity;
  }
  return out;
}

// Reduce a profile to the single number a plan-view cell is coloured by.
// Matches gatedIntensity's definitions exactly -- peak is the largest value,
// energy is 10*log10 of the mean POWER (lin*lin with lin = 10^(db/20) is
// 10^(db/10)), mean is the mean dB -- so switching focus on and off changes the
// image but never what the metric means.
export function metricOnProfile(profile, metric) {
  let value = -Infinity;
  if (metric === 'peak') {
    for (let i = 0; i < profile.length; i++) {
      if (profile[i] > value) value = profile[i];
    }
  } else if (metric === 'energy') {
    let sum = 0, count = 0;
    for (let i = 0; i < profile.length; i++) {
      if (profile[i] > -Infinity) {
        const linear = Math.pow(10, profile[i] / 20);
        sum += linear * linear;
        count++;
      }
    }
    if (count > 0) value = 10 * Math.log10(sum / count + 1e-12);
  } else if (metric === 'mean') {
    let sum = 0, count = 0;
    for (let i = 0; i < profile.length; i++) {
      if (profile[i] > -Infinity) {
        sum += profile[i];
        count++;
      }
    }
    if (count > 0) value = sum / count;
  }
  return value;
}

// The depths inside the gate, taken from a record's own range axis. Focusing
// resamples at geometric ranges, so it needs an explicit list of depths rather
// than the bin indices gatedIntensity walks.
export function gateDepths(distances, gateStartM, gateEndM) {
  const out = [];
  if (!distances) return out;
  for (let j = 0; j < distances.length; j++) {
    if (distances[j] >= gateStartM && distances[j] <= gateEndM) out.push(distances[j]);
  }
  return out;
}
