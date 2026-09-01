// Zero-padded IFFT range profile, shared by App and the B-scan background path.
//
// The window is optional and defaults to rectangular, which is what this file
// did unconditionally before the C-scan gained a window control. The trade is
// not obvious and rectangular is a defensible default rather than a placeholder:
// with 51 steps zero-padded to 256 it has -13 dB sidelobes but a ~9.8 cm
// null-to-null mainlobe, while Hanning buys -31 dB sidelobes for a ~19.5 cm
// mainlobe -- wide enough to swallow a target 7 cm from the wall face. Kaiser
// beta ~3 sits between them.

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

// Linear amplitude profile plus the distance axis. Everything else here is a
// view of this: dB is 20*log10 of it, and incoherent averaging has to happen on
// the linear amplitudes, not on the dB.
export function computeRangeAmplitude(hCalReal, hCalImag, numSteps, stepSize, rangeOffset, win) {
  const nfftMin = numSteps * 4;
  const nfft = 1 << Math.ceil(Math.log2(nfftMin));

  const re = new Float64Array(nfft);
  const im = new Float64Array(nfft);
  if (win) {
    for (let i = 0; i < numSteps; i++) {
      re[i] = hCalReal[i] * win[i];
      im[i] = hCalImag[i] * win[i];
    }
  } else {
    for (let i = 0; i < numSteps; i++) {
      re[i] = hCalReal[i];
      im[i] = hCalImag[i];
    }
  }

  ifftInPlace(re, im);

  const maxRange = SPEED_OF_LIGHT / (2 * stepSize);
  const half = nfft / 2;
  const amplitudes = [];
  const distances = [];
  for (let i = 0; i < half; i++) {
    const d = (i / nfft) * maxRange - rangeOffset;
    if (d >= 0) {
      amplitudes.push(Math.sqrt(re[i] * re[i] + im[i] * im[i]));
      distances.push(d);
    }
  }
  return { amplitudes, distances };
}

export function ampToDb(amplitudes) {
  const out = new Array(amplitudes.length);
  for (let i = 0; i < amplitudes.length; i++) out[i] = 20 * Math.log10(amplitudes[i] + 1e-12);
  return out;
}

export function computeRangeProfile(hCalReal, hCalImag, numSteps, stepSize, rangeOffset, win) {
  const { amplitudes, distances } = computeRangeAmplitude(
    hCalReal, hCalImag, numSteps, stepSize, rangeOffset, win);
  return { magnitudes: ampToDb(amplitudes), distances };
}

