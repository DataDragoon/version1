import { useEffect, useRef, useState } from 'react';
import SarWorker from '../lib/sar.worker.js?worker';

// The ONLY fields `sar.worker.js` reads off a position. Everything else on a
// C-scan record -- above all `sweeps`, which holds every raw look taken at that
// cell -- is dead weight to the reconstruction.
//
// This matters because postMessage STRUCTURED-CLONES: a deep copy of the whole
// array, synchronously, on the main thread, every time a job starts. Measured on
// a realistic long scan (101 x 15 cells):
//
//   whole record, 18 sweeps/cell    91.5 MB per clone,   916 ms
//   whole record, 64 sweeps/cell   282.2 MB per clone,  3176 ms   <- OOM
//   these fields only               16.6 MB per clone,    69 ms
//
// 282 MB of clone is what threw `DataCloneError: ... out of memory` on a long
// scan, and a multi-second synchronous block on every row change is what made
// React report "Maximum update depth exceeded" around it -- the live flush keeps
// setting state at 4 Hz while the main thread is stalled inside the clone.
//
// Projecting is done HERE rather than at the call site so it cannot be bypassed
// by a future caller. If the worker ever needs another field, add it to this
// list -- reading a field that is not here yields `undefined` in the worker, not
// an error, so keep the two in step.
const SAR_INPUT_FIELDS = [
  'h_cal_real', 'h_cal_imag', 'magnitudes', 'distances',
  'lidar_standoff_mm', 'step_size', 'range_offset',
];

function projectForSar(bscanData) {
  return bscanData.map((pos) => {
    const out = {};
    for (const k of SAR_INPUT_FIELDS) out[k] = pos[k];
    return out;
  });
}

export function useSarWorker(bscanData, bscanParams) {
  const [sarResult, setSarResult] = useState(null);
  const [sarProgress, setSarProgress] = useState(null);
  const workerRef = useRef(null);
  const debounceRef = useRef(null);
  const jobIdRef = useRef(0);

  useEffect(() => {
    return () => {
      if (workerRef.current) {
        workerRef.current.terminate();
        workerRef.current = null;
      }
      if (debounceRef.current) clearTimeout(debounceRef.current);
    };
  }, []);

  useEffect(() => {
    if (debounceRef.current) clearTimeout(debounceRef.current);

    if (!bscanData || bscanData.length < 2) {
      setSarResult(null);
      setSarProgress(null);
      return;
    }

    debounceRef.current = setTimeout(() => {
      if (workerRef.current) {
        workerRef.current.terminate();
        workerRef.current = null;
      }

      const jobId = ++jobIdRef.current;
      setSarProgress(0);

      const worker = new SarWorker();
      workerRef.current = worker;
      worker.onmessage = (e) => {
        if (jobIdRef.current !== jobId) return;
        if (e.data.type === 'progress') {
          setSarProgress(e.data.progress);
        } else if (e.data.type === 'result') {
          setSarResult(e.data.result);
          setSarProgress(null);
        }
      };
      // Projected, not the raw records -- see SAR_INPUT_FIELDS above.
      worker.postMessage({ bscanData: projectForSar(bscanData), bscanParams });
    }, 300);
  }, [bscanData, bscanParams]);

  return { sarResult, sarProgress };
}
