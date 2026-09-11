"""Hosts sensor data (IMU + LiDAR) over WebSocket."""

import asyncio
import json
import time
import argparse
import signal

import websockets

from bno085 import BNO085
from tflc02 import TFLC02, is_link_reason, is_out_of_range_reason
from imu_calibration import CalibratedIMU

clients = set()

# The TF-LC02's internal measurement updates at ~11-17 Hz (measured 2026-08-27:
# a median run of 34 identical consecutive polls at ~584 Hz; and 17.2 Hz at
# 165 mm falling to 11.5 Hz at 340 mm, which is adaptive integration time, not a
# settable frame clock). Polling faster CANNOT produce more measurements.
#
# It is still polled fast, at 200 Hz, and that is deliberate: the poll rate does
# not set how many measurements exist, it sets how late we learn that one has
# appeared. At 20 Hz that was up to 50 ms, which on a hand-swept module at
# 25 mm/s is 1.25 mm of position error -- and a DIRECTION-DEPENDENT one, since
# the sign flips when the sweep reverses, so an out-and-back pass lays the same
# physical standoff down in two places. At 200 Hz it is 5 ms / 0.13 mm. The cost
# was measured at 584 Hz (2026-08-27) and is nil: broadcast 48.35 vs 48.79 Hz,
# IMU rate slightly BETTER at 33.83 vs 32.66 Hz.
#
# What makes that safe for everything downstream is that `seq` now counts
# MEASUREMENTS, not reads -- see lidar_poll_loop. Publishing 200 reads/s of a
# 14 Hz value would have inflated the groundstation's `lidar_n` and deflated
# `lidar_std` into fiction, which is exactly the trap the 20 Hz cap was avoiding.
LIDAR_POLL_HZ = 200

# A poll whose value differs from the last is unambiguously a new measurement.
# The converse is not true: on a still target two genuine measurements can land
# on the same integer millimetre (raw sigma is 0.66-0.78 mm against 1 mm
# quantisation, so identical pairs run ~40%). Rather than undercount, a reading
# that has been stable for this long is republished as a new measurement -- it
# IS a current reading of a static target, so stamping it "now" is correct, and
# the case only arises when the true spread over the window really is ~zero.
# Chosen well above the slowest observed internal period (~87 ms at 11.5 Hz) so
# it cannot fire between two genuinely-new measurements.
LIDAR_STABLE_REPUBLISH_S = 0.25

# How long the LiDAR must fail CONTINUOUSLY before it is worth a log line.
#
# Scattered single failures are normal and must stay silent: a non-zero error
# code at a poor target angle is documented at 30-40% of reads on this bench,
# and a line per occurrence would be ~60/s of noise that trains the operator to
# ignore the one line a real fault prints (the same trap the `_sweep_core`
# 2-tuple error fell into). 1.0 s is not an arbitrary threshold -- it is exactly
# LIDAR_CARRY_MS in App.jsx, i.e. the point at which the groundstation stops
# carrying the last good reading forward and `lidar_standoff_mm` actually goes
# null. Past here, C-scan cells start recording a null standoff and (under a BG
# model) rendering as INVALID red crosses, so anything this log reports is
# something the operator is about to see on screen.
LIDAR_DROPOUT_WARN_S = 1.0

# A dropout that persists gets one progress line at this interval rather than
# silence, so a long one is distinguishable from a hung process.
LIDAR_DROPOUT_REPEAT_S = 15.0


async def register(ws):
    clients.add(ws)
    try:
        await ws.wait_closed()
    finally:
        clients.discard(ws)
        _send_fails.pop(ws, None)


# How long one client gets to accept a frame before it is dropped as dead.
#
# `websockets.send()` awaits until the frame reaches the transport, and there
# used to be no bound on that -- so a single client that stopped draining (a
# browser whose main thread is wedged, or a tab killed without a FIN, which
# leaves a half-open TCP connection the 20 s keepalive has not reaped yet)
# blocked this loop and took EVERY other client down with it.
#
# Measured 2026-09-11, one client that never reads: a healthy client alongside
# it went from 48.3 Hz with a 25 ms worst gap to 34.0 Hz with a **10,000 ms**
# worst gap. That is exactly the reported symptom -- the standoff readout
# freezing for 5-10 s while the Pi-side LiDAR was provably healthy (zero dropout
# lines in the log across the whole period). It is a LiDAR blackout in the UI
# that has nothing to do with the LiDAR.
#
# 0.5 s matches `_send_to_all` in sdr_server.py and `_fanout` in
# rover_server.py, which were fixed for the identical bug. A client that cannot
# take a frame in half a second is not rendering it anyway, and the
# groundstation reconnects in RECONNECT_INTERVAL = 500 ms -- so dropping it
# costs that client ~1 s of data and costs every other client nothing, against
# the 10 s freeze it inflicts on all of them if it is kept.
BROADCAST_TIMEOUT_S = 0.5

# Consecutive timeouts before a client is dropped. One timeout is not evidence of
# a dead client -- the groundstation holds THREE sockets (9001 sensor, 9002 rover,
# 9003 SDR) drained by one main thread, so a GC pause or a heavy C-scan derive can
# blow a single 0.5 s deadline on a tab that is otherwise perfectly healthy.
# Evicting it there would force a reconnect, and a reconnect gap past
# LIDAR_CARRY_MS is itself a null standoff and therefore an INVALID C-scan cell --
# i.e. the cure would cause the disease.
#
# Three strikes still evicts a genuinely dead client in ~1.5 s, and costs the other
# clients nothing worse than 2 Hz for that period (gather sends concurrently, so
# healthy clients get each frame immediately; only the loop's next iteration is
# delayed). 2 Hz keeps readings flowing well inside the 1 s carry window, so even
# the eviction interval cannot produce a null standoff.
BROADCAST_FAIL_LIMIT = 3
_send_fails = {}


async def broadcast(msg):
    if not clients:
        return
    # Snapshot: `clients` is mutated by register()'s add/discard on this same
    # event loop, so every await below is a point at which the set can change
    # underneath an iterator. stream.py happened to be safe (the generator was
    # unpacked before the first await) where rover_server.py was not and raised
    # `Set changed size during iteration`; taking a copy makes it safe by
    # construction rather than by accident.
    targets = list(clients)
    results = await asyncio.gather(
        *(asyncio.wait_for(c.send(msg), BROADCAST_TIMEOUT_S) for c in targets),
        return_exceptions=True)
    for c, r in zip(targets, results):
        if isinstance(r, BaseException):
            n = _send_fails.get(c, 0) + 1
            _send_fails[c] = n
            if n >= BROADCAST_FAIL_LIMIT:
                print(f"[sensors] dropping client after {n} consecutive send "
                      f"failures ({type(r).__name__})", flush=True)
                clients.discard(c)
                _send_fails.pop(c, None)
                # Fire-and-forget: awaiting close() here would reintroduce
                # exactly the unbounded wait this function exists to remove.
                asyncio.create_task(_close_quietly(c))
        else:
            # CONSECUTIVE, so a tab that hiccups once an hour is never evicted.
            _send_fails.pop(c, None)


async def _close_quietly(ws):
    try:
        await ws.close()
    except Exception:
        pass


async def imu_poll_loop(imu, state):
    """Reads the IMU in its own uncapped loop, publishing the latest reading into
    `state`. Runs via run_in_executor so the blocking driver call never stalls the
    event loop -- see lidar_poll_loop for why this matters."""
    loop = asyncio.get_running_loop()
    fail_streak = 0
    IMU_FAIL_LIMIT = 20
    while True:
        try:
            body = await loop.run_in_executor(None, imu.read_body)
            state['accel'] = body['accel'].tolist()
            state['gyro'] = body['gyro'].tolist()
            state['temp'] = body['temp']
            fail_streak = 0
        except Exception as e:
            fail_streak += 1
            if fail_streak == 1:
                print(f"WARNING: IMU read failed ({e!r}), streaming IMU as null")
            state['accel'] = state['gyro'] = state['temp'] = None
            if fail_streak >= IMU_FAIL_LIMIT:
                print(f"IMU failed {IMU_FAIL_LIMIT} reads in a row, giving up on it")
                return


async def lidar_poll_loop(lidar, state, rate=LIDAR_POLL_HZ):
    """Reads the LiDAR in its own loop, publishing the latest reading into
    `state`.

    Polled at LIDAR_POLL_HZ (200), which is far above the sensor's own 11-17 Hz
    measurement rate ON PURPOSE -- see the constant for why: fast polling buys a
    tight TIMESTAMP for each measurement, not more of them.

    `seq` and `ts` therefore describe MEASUREMENTS, not reads. `seq` advances
    only when the value changes (or when a stable value ages past
    LIDAR_STABLE_REPUBLISH_S), and `ts` is stamped at that moment. This matters
    to every consumer: App.jsx dedupes accumulated readings by `lidar_seq`, so
    publishing every read at 200 Hz would have made `lidar_n` count duplicates
    and `lidar_std` measure the spread of a value repeated -- the "repeats
    deflate the spread" fiction the old 20 Hz cap existed to avoid. Counting
    measurements is both faster to timestamp AND more honest than the old
    per-read counter was at 20 Hz, where 20-40% of reads were already repeats.

    This is deliberately NOT awaited inline in sensor_loop's broadcast loop:
    TFLC02.read_distance() blocks on a 100ms UART timeout whenever the sensor
    doesn't answer, which happened to be true throughout the 2026-08-24
    debugging above. Awaiting it directly in the broadcast loop caps the whole
    stream (IMU included) at ~10Hz regardless of `rate` -- confirmed by timing
    read_distance() in isolation. Running it here, in its own task via
    run_in_executor, means a slow or dead LiDAR only slows *this* loop; the
    broadcast loop below keeps running at its full requested rate using
    whatever LiDAR reading was most recently published, stale or not."""
    loop = asyncio.get_running_loop()
    fail_streak = 0
    interval = 1.0 / rate if rate and rate > 0 else 0.0
    last_dist = None
    last_pub = 0.0
    # State for the dropout log. `reasons` tallies WHY the reads in the current
    # failure run failed, which is the whole point: a run of `sensor:4` is the
    # module saying it cannot range the target (aim the head), a run of
    # `link:no_bytes` is the module not answering (check power/wiring). Those
    # two were indistinguishable before 2026-09-11 and sent the last
    # investigation after the cables for days.
    fail_since = None
    fail_reasons = {}
    fail_reported = False
    fail_last_report = 0.0

    def summarise(reasons):
        parts = sorted(reasons.items(), key=lambda kv: -kv[1])
        return ', '.join(f'{k} x{v}' for k, v in parts[:4])

    while True:
        t0 = time.monotonic()
        try:
            dist, reason = await loop.run_in_executor(None, lidar.read_distance_detail)
        except Exception as e:
            fail_streak += 1
            if fail_streak == 1:
                print(f"WARNING: LiDAR read raised ({e!r})", flush=True)
            dist, reason = None, f'exception:{type(e).__name__}'
        else:
            fail_streak = 0

        state['dist'] = dist
        state['err'] = None if dist is not None else reason

        if dist is not None:
            now = time.time()
            state['good_mm'] = dist
            state['good_t'] = now
            # A changed value is unambiguously a new measurement. An unchanged
            # one is ambiguous, so it is republished only once it has outlived
            # any plausible internal period.
            if dist != last_dist or (now - last_pub) >= LIDAR_STABLE_REPUBLISH_S:
                state['seq'] += 1
                state['ts'] = now
                last_pub = now
            last_dist = dist
            if fail_reported:
                dur = t0 - fail_since
                print(f"LiDAR recovered after {dur:.1f}s ({summarise(fail_reasons)}); "
                      f"reading {dist} mm", flush=True)
            fail_since = None
            fail_reasons = {}
            fail_reported = False
        else:
            if fail_since is None:
                fail_since = t0
                fail_reasons = {}
                fail_reported = False
            fail_reasons[reason] = fail_reasons.get(reason, 0) + 1
            dur = t0 - fail_since
            # Silent below the threshold: scattered invalid returns are normal
            # and the groundstation carries the last good reading across them.
            due = (not fail_reported and dur >= LIDAR_DROPOUT_WARN_S) or \
                  (fail_reported and (t0 - fail_last_report) >= LIDAR_DROPOUT_REPEAT_S)
            if due:
                fail_reported = True
                fail_last_report = t0
                link = any(is_link_reason(r) for r in fail_reasons)
                oor = sum(v for r, v in fail_reasons.items() if is_out_of_range_reason(r))
                total = sum(fail_reasons.values())
                if link:
                    hint = "module NOT answering -- check power/wiring/baud"
                elif oor > 0.5 * total:
                    # Not a fault. The module is healthy and the target is simply
                    # beyond what it can measure; it says so with the 8888 sentinel.
                    hint = ("target OUT OF RANGE -- the module is working and "
                            "returns its 8888 sentinel. Aim it at something closer")
                else:
                    hint = ("module IS answering and reports no valid return -- "
                            "aim/reflectivity, not wiring")
                print(f"WARNING: no valid LiDAR reading for {dur:.1f}s "
                      f"[{summarise(fail_reasons)}] -- {hint}. "
                      f"Standoff is null downstream; C-scan cells captured now "
                      f"will be INVALID under a BG model.", flush=True)

        if interval:
            await asyncio.sleep(max(0, interval - (time.monotonic() - t0)))


async def sensor_loop(rate, skip_cal=False, lidar_rate=LIDAR_POLL_HZ):
    lidar = TFLC02()

    imu = None
    try:
        raw_imu = BNO085()
        print(f"BNO085 detected (part number {raw_imu.who_am_i()})")
        imu = CalibratedIMU(raw_imu, auto_calibrate=not skip_cal)
    except Exception as e:
        print(f"WARNING: IMU init failed ({e!r}), streaming without IMU")

    print(f"TF-LC02 on {lidar.ser.port}")

    interval = 1.0 / rate
    print(f"Streaming sensors at {rate}Hz on ws://0.0.0.0:9001")

    imu_state = {'accel': None, 'gyro': None, 'temp': None}
    # seq increments once per distinct MEASUREMENT (see lidar_poll_loop), so a
    # consumer can dedupe both the repeats that come from broadcasting faster
    # than the LiDAR updates and the repeats that come from polling faster.
    lidar_state = {'dist': None, 'seq': 0, 'ts': None,
                   'err': None, 'good_mm': None, 'good_t': None}
    print(f"LiDAR polled at {lidar_rate}Hz "
          f"(sensor measures internally at ~11-17Hz; seq counts measurements, not polls)")
    poll_tasks = [asyncio.create_task(lidar_poll_loop(lidar, lidar_state, lidar_rate))]
    if imu is not None:
        poll_tasks.append(asyncio.create_task(imu_poll_loop(imu, imu_state)))

    try:
        while True:
            t0 = time.monotonic()

            packet = {
                'accel': imu_state['accel'],
                'gyro': imu_state['gyro'],
                'temp': imu_state['temp'],
                'lidar': lidar_state['dist'],
                # Provenance for the LiDAR sample: seq identifies the
                # MEASUREMENT and ts is when that measurement was first seen
                # (not when this packet was sent, and not when it was re-read).
                'lidar_seq': lidar_state['seq'],
                'lidar_ts': lidar_state['ts'],
                # Why the most recent read failed, or None when it succeeded.
                # `lidar` going null says only THAT a read failed; this says
                # which of five distinct causes it was. Additive fields -- a
                # groundstation that predates them simply ignores them.
                'lidar_err': lidar_state['err'],
                # The last reading that was actually valid, and how old it is.
                # `lidar` is wiped to null by a SINGLE failed read, so on a
                # bench where 30-40% of reads return a non-zero error code it
                # flaps at high rate even when the sensor is perfectly healthy.
                # Publishing the last good value with its age lets a consumer
                # distinguish "one bad read" from "the sensor has been dark for
                # nine seconds" without having to reconstruct that itself.
                'lidar_last_good_mm': lidar_state['good_mm'],
                'lidar_last_good_age_s': (
                    None if lidar_state['good_t'] is None
                    else time.time() - lidar_state['good_t']),
                'timestamp': time.time(),
            }

            await broadcast(json.dumps(packet))
            elapsed = time.monotonic() - t0
            await asyncio.sleep(max(0, interval - elapsed))
    finally:
        for t in poll_tasks:
            t.cancel()
        for t in poll_tasks:
            try:
                await t
            except asyncio.CancelledError:
                pass
        if imu is not None:
            try:
                imu.close()
            except Exception:
                pass
        lidar.close()


async def main():
    parser = argparse.ArgumentParser(description='Host sensor data over WebSocket')
    parser.add_argument('--port', type=int, default=9001, help='WebSocket port (default: 9001)')
    parser.add_argument('--rate', type=int, default=50, help='Sample rate in Hz (default: 50)')
    parser.add_argument('--skip-cal', action='store_true', help='Skip gyro calibration (use saved)')
    parser.add_argument('--lidar-rate', type=float, default=LIDAR_POLL_HZ,
                        help=f'LiDAR poll rate in Hz (default: {LIDAR_POLL_HZ}; '
                             '0 = uncapped. Sets timestamp precision, not the '
                             'number of measurements -- the sensor measures at '
                             '11-17 Hz whatever this is)')
    args = parser.parse_args()

    stop = asyncio.get_event_loop().create_future()
    loop = asyncio.get_event_loop()

    def request_stop():
        # SIGINT (Ctrl-C, propagated to the whole process group) and SIGTERM
        # (start.py forwarding to this child) routinely both arrive — resolving
        # an already-done future raises InvalidStateError, so guard it.
        if not stop.done():
            stop.set_result(None)

    loop.add_signal_handler(signal.SIGINT, request_stop)
    loop.add_signal_handler(signal.SIGTERM, request_stop)

    def log_task_exception(t):
        if not t.cancelled() and t.exception() is not None:
            print(f"sensor_loop crashed: {t.exception()!r}")
            request_stop()

    async with websockets.serve(register, '0.0.0.0', args.port):
        task = asyncio.create_task(sensor_loop(args.rate, skip_cal=args.skip_cal,
                                              lidar_rate=args.lidar_rate))
        task.add_done_callback(log_task_exception)
        await stop
        task.cancel()

    print("\nStopped.")


if __name__ == '__main__':
    asyncio.run(main())
