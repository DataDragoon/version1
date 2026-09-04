// The starting vector is DETERMINISTIC, and that matters more than it looks. It was
// Math.random(), so every call returned a different filtered matrix -- measured on a
// real 101-position scan, three consecutive svdFilter(d, 2, 1.0) calls produced three
// different results, and since SAR re-reconstructs whenever any upstream parameter
// changes identity, the incoherent image visibly changed with nothing touched. Same
// cos() seed the worker's complexSvdFilter has always used, offset so that component 0
// does not start from the constant vector.
export function powerIteration(matrix, rows, cols, maxIter = 100, tol = 1e-10) {
  let v = new Float64Array(cols);
  for (let i = 0; i < cols; i++) v[i] = Math.cos(i + 0.5);

  let norm = Math.sqrt(v.reduce((s, x) => s + x * x, 0));
  for (let i = 0; i < cols; i++) v[i] /= norm;

  let u = new Float64Array(rows);
  let sigma = 0;

  for (let iter = 0; iter < maxIter; iter++) {
    for (let i = 0; i < rows; i++) {
      let sum = 0;
      for (let j = 0; j < cols; j++) {
        sum += matrix[i * cols + j] * v[j];
      }
      u[i] = sum;
    }

    sigma = Math.sqrt(u.reduce((s, x) => s + x * x, 0));
    if (sigma < tol) return { u, sigma: 0, v };

    for (let i = 0; i < rows; i++) u[i] /= sigma;

    let vNew = new Float64Array(cols);
    for (let j = 0; j < cols; j++) {
      let sum = 0;
      for (let i = 0; i < rows; i++) {
        sum += matrix[i * cols + j] * u[i];
      }
      vNew[j] = sum;
    }

    let newNorm = Math.sqrt(vNew.reduce((s, x) => s + x * x, 0));
    if (newNorm < tol) return { u, sigma, v };

    for (let j = 0; j < cols; j++) vNew[j] /= newNorm;

    let diff = 0;
    for (let j = 0; j < cols; j++) diff += (vNew[j] - v[j]) ** 2;

    v = vNew;
    if (diff < tol) break;
  }

  return { u, sigma, v };
}

export function svdFilter(bscanData, k, strength = 1.0) {
  if (!bscanData || bscanData.length < 2 || k < 1) return bscanData;

  const numPositions = bscanData.length;
  const numBins = bscanData[0].magnitudes.length;

  const flat = new Float64Array(numPositions * numBins);
  for (let p = 0; p < numPositions; p++) {
    for (let b = 0; b < numBins; b++) {
      flat[p * numBins + b] = bscanData[p].magnitudes[b];
    }
  }

  const s = Math.max(0, Math.min(1, strength));
  for (let i = 0; i < k; i++) {
    const { u, sigma, v } = powerIteration(flat, numPositions, numBins);
    if (sigma < 1e-10) break;

    for (let p = 0; p < numPositions; p++) {
      for (let b = 0; b < numBins; b++) {
        flat[p * numBins + b] -= s * sigma * u[p] * v[b];
      }
    }
  }

  const filtered = bscanData.map((pos, p) => {
    const newMags = new Array(numBins);
    for (let b = 0; b < numBins; b++) {
      newMags[b] = flat[p * numBins + b];
    }
    return { ...pos, magnitudes: newMags };
  });

  return filtered;
}
