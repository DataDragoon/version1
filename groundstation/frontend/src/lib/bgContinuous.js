// Continuous ("wave the module around") BG-model capture.
//
// The static protocol -- park the module, press Capture, hold still for N
// sweeps, move, repeat -- exists because a sweep used to take ~550 ms, so any
// motion during one smeared it across frequency. At 36 Hz a sweep is 27.5 ms,
// which is short enough that a slow hand pass barely moves within it, so the
// positions can be swept continuously instead of visited one at a time.
//
// That inverts the constraint. Position density is the dominant accuracy lever
// (measured: ~12 dB of leave-one-out suppression lost per doubling of the
// median gap), and hand-placing 30 positions is what capped it at ~5 mm. Waving
// over the same span for a minute produces thousands of sweeps at a continuum
// of standoffs; the job of this module is to decide WHERE each of those sweeps
// was taken, and to turn the stream into the same per-position record the
// static path produces.
//
// ---------------------------------------------------------------------------
// Standoff comes from INTERPOLATING the lidar track, not from the sweep's own
// averaged reading. This is the whole accuracy argument, so it is worth stating
// in full.
//
// The TF-LC02 measures at 11-17 Hz (its own integration cadence -- polling
// faster cannot produce more measurements, and it gets SLOWER with distance
// because integration is adaptive). Sweeps arrive at 36 Hz. So most sweeps
// contain no new measurement at all, and `App.jsx` carries the last one forward
// so the live display does not strobe.
//
// Attaching that carried reading to a moving sweep is the error to avoid. It is
// a pure LAG: the standoff used is always the last one measured, so it trails
// the truth by up to a full lidar period. At 25 mm/s and an 87 ms period that is
// up to 2.2 mm -- and crucially it is a DIRECTION-DEPENDENT bias, not noise. The
// sign flips when the pass reverses, so an out-and-back wave lays the same
// physical standoff down in two places, which is exactly the error a coherent
// background model cannot absorb (measured sensitivity: 1 mm costs ~1.4 dB,
// 5 mm costs ~10 dB).
//
// There are two ways to avoid it, and the difference between them is what this
// module is. Requiring a fresh reading (`lidar_n > 0`) DOES remove the bias, and
// that is worth stating precisely because it is not obvious: a measurement that
// landed inside the sweep window is on average at the sweep's own midpoint, so
// the error is jitter rather than lag. What it costs is that only ~38% of sweeps
// have one, and the survivors still carry +/- half a sweep period of jitter.
// Interpolating between the measurement BEFORE the sweep and the one AFTER is
// unbiased as well, keeps ~99% of sweeps, and is quieter on top -- it averages
// two measurements where the filter takes one. Measured on a simulated 60 s
// out-and-back pass at 25 mm/s, 14 Hz lidar with 0.4 mm noise, 36 Hz sweeps,
// scored against known truth:
//
//                          rms error   out-and-back bias   sweeps kept
//   carried, unfiltered     0.851 mm        1.100 mm          100%
//   fresh-reading filter    0.445 mm        0.016 mm           38%
//   interpolated            0.322 mm        0.041 mm           99%
//
// So the naive carry is the thing to avoid; between the other two it is 2.6x
// the looks per bin and 28% less standoff error for the same run. On that same
// simulation the resulting model's leave-one-out suppression went 39.8 ->
// 46.2 dB.
//
// This needs the measurements to be accurately timestamped, which is why
// `stream.py` now polls at 200 Hz and advances `lidar_seq`/`lidar_ts` only when
// the value actually CHANGES: the poll rate buys timestamp precision (5 ms
// instead of 50 ms), not more measurements.
//
// What remains after interpolation is the sensor's own publication lag -- the
// fixed offset between the middle of its integration window and the moment the
// value appears on the wire. That is a constant time offset, so it is again a
// direction-dependent position bias, and it is NOT corrected here because it
// has never been measured. Instead every accepted sweep records the signed
// velocity it was taken at, so an out-and-back run contains both signs at the
// same standoff and the lag can be solved for offline from an export. Do not
// guess it.
// ---------------------------------------------------------------------------
//
// Two filters remain, each for a measured reason:
//
//  1. NOT BRACKETED. A sweep whose time is not spanned by two measurements
//     within MAX_BRACKET_GAP_S has no interpolant -- the lidar went quiet (a
//     burst of invalid returns at a poor target angle is documented at 30-40%
//     of reads on this bench) and interpolating across the hole would invent a
//     trajectory. Dropped rather than extrapolated.
//
//     Note this filter is about WHERE a sweep was, not how fast it was moving,
//     so it is unrelated to the speed gate below and neither substitutes for
//     the other.
//
//  2. TOO FAST. A sweep steps through frequency sequentially, so standoff
//     changing DURING it puts a phase ramp across the band, i.e. a range smear
//     that no amount of averaging removes and that interpolation cannot fix
//     either -- the sweep genuinely does not describe one position. The
//     displacement within one sweep is v * T_sweep, so a limit on v is a limit
//     on smear: 40 mm/s at 27.5 ms is 1.1 mm. Speed is a least-squares slope
//     over a window of the lidar track rather than a consecutive difference,
//     because the lidar's own 0.4 mm noise across a 70 ms gap is ~6 mm/s of
//     phantom speed on its own and a two-point estimate would reject much of a
//     perfectly slow pass.
//
// Sweeps then land in fixed standoff bins, each taking at most maxSweepsPerBin
// -- that cap is what bounds memory, since an unbounded 10-minute run at 36 Hz
// is ~22k sweeps. Each occupied bin becomes exactly one "capture" in the
// existing {samples, stats} shape, so coverage analysis, export, the trainer
// and the leave-one-out scoring are all untouched.
//
// One operational consequence of the lidar's cadence that the speed gate does
// NOT cover: within a single pass a measurement lands every `v * lidar_period`,
// so at 40 mm/s they are ~3 mm apart and one pass cannot fill 1 mm bins (a
// single pass measures a 13 mm hole against 1 mm for a 12 mm/s pass). Over many
// passes it fills anyway, because the lidar cadence and the pass timing are
// incommensurate so each pass samples different phases -- a 60 s run at 40 mm/s
// closes to a 1 mm hole. A fast wave is not broken, it just needs more passes,
// and the panel's Hole readout is what says whether it has had enough.

import { computeCaptureStats } from './bgCaptureStats';

// Half-width of the window used to fit velocity from the lidar track. Wide
// enough that the fit averages down the lidar's own noise, short enough to
// follow a hand reversing direction.
const SPEED_WINDOW_S = 0.35;
const SPEED_MIN_POINTS = 3;

// Two measurements further apart than this did not bracket the sweep in any
// useful sense -- the sensor was not reporting, and a straight line across the
// hole is an invention. Comfortably above the slowest observed internal period
// (~87 ms at 11.5 Hz) so ordinary cadence never trips it.
const MAX_BRACKET_GAP_S = 0.25;

// Lidar history retained. Only enough to bracket the pending sweeps and fit a
// velocity; at ~14 Hz this is ~70 points.
const TRACK_KEEP_S = 5;

// Adjacent sweep intervals used to estimate the sweep period, whose half is the
// offset from the Pi's end-of-sweep timestamp back to the sweep's midpoint.
// Median, not mean, so one stalled frame does not move it -- the same choice
// Viewport.jsx's useSweepRate makes for the same reason.
const PERIOD_WINDOW = 12;

// A sweep whose complex correlation with the rest of its own bin is below this
// did not measure the same thing they did. The radar throws the occasional
// corrupted sweep -- 0.46% idle and 2.05% under client load on the 2026-09-06
// measurements, plus the NIOS path's ~1.4% fallbacks -- and a 60 s run at 36 Hz
// is ~2200 sweeps, so tens of them land somewhere. The static protocol diluted
// one bad sweep across 40 good ones; a continuous bin holding 2 or 3 does not,
// and its coherent mean becomes a corrupted KNOT, which is the thing CLAUDE.md's
// leave-one-out scoring calls a weakest knot. Measured with one garbage sweep
// injected: n=2 scores -0.1 dB, n=3 3.7, n=5 10.6, n=18 21.9, n=40 29.1 -- so
// the damage is worst exactly where continuous capture is thinnest.
//
// 0.90 sits in a very wide empty gap: sweeps of the same scene within one 1 mm
// bin correlate >0.99 (the wall term rotates only ~12 deg per mm at 5 GHz, and
// single-sweep SNR is ~21 dB), while a garbled sweep has random phase per step
// and correlates ~1/sqrt(51) = 0.14.
const BIN_AGREE_MIN = 0.90;

// How long a run may go with no lidar measurement at all before concluding this
// Pi does not send timestamped ones and falling back to the legacy path. It
// cannot be decided on the first sweep -- at the start of EVERY run the track is
// empty until the first measurement lands, and treating those sweeps as legacy
// would silently give the first ~14 sweeps of each run the laggy standoff the
// whole module exists to avoid. Well above the 250 ms bracket limit, so an
// ordinary quiet spell cannot latch it.
const LEGACY_DECIDE_S = 1.5;

export const REJECT = {
  ACCEPTED: 'accepted',
  PENDING: 'pending',
  NO_LIDAR: 'no_lidar',
  MOTION: 'motion',
  BIN_FULL: 'bin_full',
};

export function createContinuousAccum({
  binWidthMm = 1.0,
  maxSweepsPerBin = 40,
  maxSpeedMmS = 40,
} = {}) {
  // Bin width below the interpolator's own MERGE_MM (0.5) would be pointless:
  // toKnots collapses knots closer than that anyway, so finer bins just split
  // the same looks across knots that are then re-merged with fewer sweeps each.
  const bin = Math.max(0.5, Number(binWidthMm) || 1.0);
  const cap = Math.max(1, Math.round(Number(maxSweepsPerBin) || 1));
  const speedLimit = Number(maxSpeedMmS) > 0 ? Number(maxSpeedMmS) : null;

  const track = [];                // {t, d} distinct measurements, Pi clock
  let trackFed = false;            // has this Pi ever sent a timestamped one
  let legacyMode = false;          // latched once it is clear none are coming
  const pending = [];              // sweeps waiting to be bracketed
  const bins = new Map();          // bin index -> sample array
  const counts = { total: 0, accepted: 0, no_lidar: 0, motion: 0, bin_full: 0, screened: 0 };
  const brackets = [];             // bracket widths of accepted sweeps, for the readout
  let speed = null;
  let minMm = null, maxMm = null;
  const periods = [];
  let lastSweepT = null;
  // Every sweep, accepted or not, so the reported rate is the radar's and the
  // smear estimate below is measured rather than assumed.
  let tFirst = null, tLast = null;

  function median(a) {
    if (!a.length) return null;
    const s = [...a].sort((x, y) => x - y);
    return s[Math.floor(s.length / 2)];
  }

  // Half the sweep period: the Pi stamps `timestamp` when the sweep is
  // processed, i.e. at its END, while the standoff wanted is the one at its
  // middle. 14 ms at 36 Hz -- 0.35 mm at 25 mm/s, which is the same order as
  // the lidar's own noise, so it is worth removing rather than ignoring.
  function halfSweep() {
    const p = median(periods);
    return p != null && p > 0 && p < 1 ? p / 2 : 0;
  }

  // Linear interpolation of the lidar track at time t. Returns null unless t is
  // genuinely bracketed -- never extrapolates past either end, because the
  // whole point is to stop using a reading from a place the module has left.
  function standoffAt(t) {
    const n = track.length;
    if (n < 2 || t < track[0].t || t > track[n - 1].t) return null;
    let lo = 0, hi = n - 1;
    while (hi - lo > 1) {
      const mid = (lo + hi) >> 1;
      if (track[mid].t <= t) lo = mid; else hi = mid;
    }
    const gap = track[hi].t - track[lo].t;
    if (!(gap > 0) || gap > MAX_BRACKET_GAP_S) return null;
    const f = (t - track[lo].t) / gap;
    return { d: track[lo].d + f * (track[hi].d - track[lo].d), gap };
  }

  // Least-squares slope of the track over +/- SPEED_WINDOW_S about t.
  function velocityAt(t) {
    let n = 0, st = 0, sd = 0;
    for (const p of track) {
      if (Math.abs(p.t - t) > SPEED_WINDOW_S) continue;
      n++; st += p.t; sd += p.d;
    }
    if (n < SPEED_MIN_POINTS) return null;
    const mt = st / n, md = sd / n;
    let num = 0, den = 0;
    for (const p of track) {
      if (Math.abs(p.t - t) > SPEED_WINDOW_S) continue;
      const dt = p.t - mt;
      num += dt * (p.d - md);
      den += dt * dt;
    }
    return den > 0 ? num / den : null;
  }

  function file(sample, d, v, gap) {
    if (speedLimit != null && v != null && Math.abs(v) > speedLimit) {
      counts.motion++;
      return REJECT.MOTION;
    }
    const idx = Math.floor(d / bin);
    let arr = bins.get(idx);
    if (!arr) { arr = []; bins.set(idx, arr); }
    if (arr.length >= cap) {
      counts.bin_full++;
      return REJECT.BIN_FULL;
    }
    arr.push({
      ...sample,
      // The interpolated standoff REPLACES the live one, because it is what
      // every downstream consumer reads (computeCaptureStats, toKnots, the
      // export). The live value is kept beside it so the two can be compared.
      lidar_standoff_mm: d,
      lidar_standoff_live_mm: sample.lidar_standoff_mm,
      lidar_interp: true,
      // Signed, so an out-and-back run carries both directions at the same
      // standoff and the sensor's fixed publication lag can be solved for
      // offline. Nothing here corrects for it -- see the header.
      lidar_v_mm_s: v,
      lidar_bracket_s: gap,
    });
    counts.accepted++;
    if (gap != null) brackets.push(gap);
    if (minMm == null || d < minMm) minMm = d;
    if (maxMm == null || d > maxMm) maxMm = d;
    return REJECT.ACCEPTED;
  }

  // Legacy path for a Pi that never sends `lidar_ts` (pre-2026-09-07
  // stream.py): fall back to the sweep's own averaged reading, requiring
  // lidar_n > 0 so at least the carried value is refused. Degrades rather than
  // capturing nothing, the same way App.jsx's `lidar_seq === undefined` check
  // does.
  function fileLegacy(sample) {
    const d = sample.lidar_standoff_mm;
    if (d == null || !(sample.lidar_n > 0)) {
      counts.no_lidar++;
      return REJECT.NO_LIDAR;
    }
    const t = Number.isFinite(sample.timestamp) ? sample.timestamp : counts.total / 30;
    track.push({ t, d });
    while (track.length > 2 && track[0].t < t - TRACK_KEEP_S) track.shift();
    speed = velocityAt(t);
    return file(sample, d, speed, null);
  }

  function resolve(final) {
    // No measurement has arrived yet. Sweeps wait rather than being filed by
    // the lagging live standoff -- but not forever; if none is coming this is a
    // Pi that does not publish `lidar_ts` and the legacy path takes over for
    // good.
    if (!trackFed && pending.length) {
      const span = pending[pending.length - 1].__t - pending[0].__t;
      if (final || span > LEGACY_DECIDE_S) {
        legacyMode = true;
        for (const q of pending.splice(0, pending.length)) fileLegacy(q.__s);
      }
      return;
    }
    const newest = track.length ? track[track.length - 1].t : null;
    let i = 0;
    while (i < pending.length) {
      const sample = pending[i];
      const t = sample.__t;
      const hit = standoffAt(t);
      if (hit) {
        const v = velocityAt(t);
        if (v != null) speed = v;
        file(sample.__s, hit.d, v, hit.gap);
        pending.splice(i, 1);
        continue;
      }
      // Not bracketed yet. It only becomes droppable once a later measurement
      // proves it never will be -- or at the end of the run, when no later
      // measurement is coming.
      const hopeless = final
        || (newest != null && newest > t)
        || (track.length && t < track[0].t);
      if (hopeless) {
        counts.no_lidar++;
        pending.splice(i, 1);
        continue;
      }
      i++;
    }
  }

  // A distinct lidar MEASUREMENT (deduped by lidar_seq upstream), with the Pi's
  // own clock -- the same clock the sweep timestamps come from, which is what
  // makes the two streams comparable across two websockets.
  function pushLidar({ t, d }) {
    if (!Number.isFinite(t) || !Number.isFinite(d)) return;
    trackFed = true;
    // Out of order or repeated: the track has to stay strictly increasing for
    // the bracket search, and a repeat carries no new information anyway.
    if (track.length && t <= track[track.length - 1].t) return;
    track.push({ t, d });
    while (track.length > 2 && track[0].t < t - TRACK_KEEP_S) track.shift();
    resolve(false);
  }

  function pushSweep(sample) {
    counts.total++;
    const ts = sample.timestamp;
    if (Number.isFinite(ts)) {
      if (tFirst == null) tFirst = ts;
      tLast = ts;
      if (lastSweepT != null && ts > lastSweepT) {
        periods.push(ts - lastSweepT);
        if (periods.length > PERIOD_WINDOW) periods.shift();
      }
      lastSweepT = ts;
    }
    if (legacyMode) return fileLegacy(sample);
    if (!Number.isFinite(ts)) { counts.no_lidar++; return REJECT.NO_LIDAR; }
    pending.push({ __s: sample, __t: ts - halfSweep() });
    resolve(false);
    return REJECT.PENDING;
  }

  // Cheap enough to call at a few Hz while capturing: it walks the occupied bin
  // indices, not the samples.
  function summary() {
    const idxs = [...bins.keys()].sort((a, b) => a - b);
    const perBin = idxs.map(i => bins.get(i).length).sort((a, b) => a - b);
    let maxGapMm = null;
    for (let i = 1; i < idxs.length; i++) {
      const g = (idxs[i] - idxs[i - 1]) * bin;
      if (maxGapMm == null || g > maxGapMm) maxGapMm = g;
    }
    const elapsed = tFirst != null && tLast > tFirst ? tLast - tFirst : null;
    const sweepHz = elapsed && counts.total > 1 ? (counts.total - 1) / elapsed : null;
    return {
      binWidthMm: bin,
      maxSweepsPerBin: cap,
      interpolated: trackFed,
      bins: idxs.length,
      fullBins: perBin.filter(n => n >= cap).length,
      thinBins: perBin.filter(n => n < Math.min(4, cap)).length,
      medianPerBin: perBin.length ? perBin[Math.floor(perBin.length / 2)] : 0,
      pending: pending.length,
      // Median spacing of the bracketing measurements: the lidar's own cadence
      // as actually observed, and the width the interpolation has to span.
      medianBracketMs: brackets.length ? median(brackets) * 1000 : null,
      minMm, maxMm,
      spanMm: minMm != null ? maxMm - minMm : 0,
      // Gap between OCCUPIED bins -- a hole the pass has not covered yet, which
      // is the one thing worth telling the operator mid-capture.
      maxGapMm,
      speedMmS: speed,
      sweepHz,
      elapsedS: elapsed,
      // How far the module moves DURING one sweep at the current speed: the
      // quantity the speed limit is really about, since that displacement is
      // the phase ramp smeared across the band.
      smearMm: speed != null && sweepHz ? Math.abs(speed) / sweepHz : null,
      ...counts,
    };
  }

  // Resolve whatever can still be resolved and give up on the rest. Called once
  // at harvest: the last sweep or two of a run have no measurement after them,
  // and extrapolating them is exactly the lag this module exists to remove.
  function flush() { resolve(true); }

  // Magnitude of the normalised complex correlation between two sweeps -- the
  // same statistic bgCaptureStats uses for sweepCorrelation and the Pi uses to
  // call a sweep visibly corrupted, so the three agree on what "agrees" means.
  function corr(a, b) {
    const S = Math.min(a.h_cal_real.length, b.h_cal_real.length);
    let dr = 0, di = 0, na = 0, nb = 0;
    for (let i = 0; i < S; i++) {
      const aR = a.h_cal_real[i], aI = a.h_cal_imag[i];
      const bR = b.h_cal_real[i], bI = b.h_cal_imag[i];
      dr += aR * bR + aI * bI;
      di += aI * bR - aR * bI;
      na += aR * aR + aI * aI;
      nb += bR * bR + bI * bI;
    }
    return Math.hypot(dr, di) / (Math.sqrt(na) * Math.sqrt(nb) + 1e-30);
  }

  // Drop sweeps that disagree with the rest of their own bin. Each sweep is
  // scored by its MEDIAN correlation against the others, not against their
  // mean: a mean is dragged by the very outlier being looked for, whereas a
  // median is unmoved by a minority of bad sweeps, which is the case here.
  //
  // Needs 3 sweeps to arbitrate -- with 2 that disagree there is no way to say
  // which is wrong, so both are kept and the bin's own SNR (negative, in that
  // case) is left to report it rather than guessing.
  function screenBin(samples) {
    const n = samples.length;
    if (n < 3) return samples;
    const usable = samples.filter(s => s.h_cal_real && s.h_cal_imag);
    if (usable.length < 3) return samples;
    const c = usable.map(() => []);
    for (let i = 0; i < usable.length; i++) {
      for (let j = i + 1; j < usable.length; j++) {
        const v = corr(usable[i], usable[j]);
        c[i].push(v); c[j].push(v);
      }
    }
    const keep = usable.filter((_, i) => {
      const m = c[i].sort((x, y) => x - y)[Math.floor(c[i].length / 2)];
      return m >= BIN_AGREE_MIN;
    });
    // Never empty a bin on this evidence: if nothing agrees with anything the
    // problem is not one outlier, and silently deleting the position would put
    // a hole in the model instead of a visibly bad knot.
    if (keep.length < 2) return samples;
    counts.screened += usable.length - keep.length;
    return keep;
  }

  // One capture per occupied bin, ordered by standoff. Same {samples, stats}
  // shape the static path produces, so nothing downstream can tell them apart.
  function toCaptures() {
    counts.screened = 0;
    return [...bins.keys()]
      .sort((a, b) => a - b)
      .map(i => {
        const samples = screenBin(bins.get(i));
        return { samples, stats: computeCaptureStats(samples), continuous: true };
      })
      .filter(c => c.stats != null);
  }

  return { pushLidar, pushSweep, flush, summary, toCaptures };
}
