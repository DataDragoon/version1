"""Hosts sensor data (IMU + LiDAR) over WebSocket."""

import asyncio
import json
import time
import argparse
import signal

import websockets

from bno085 import BNO085
from tflc02 import TFLC02
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


async def register(ws):
    clients.add(ws)
    try:
        await ws.wait_closed()
    finally:
        clients.discard(ws)


async def broadcast(msg):
    if clients:
        await asyncio.gather(*(c.send(msg) for c in clients), return_exceptions=True)


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
    while True:
        t0 = time.monotonic()
        try:
            dist = await loop.run_in_executor(None, lidar.read_distance)
            state['dist'] = dist
            if dist is not None:
                now = time.time()
                # A changed value is unambiguously a new measurement. An
                # unchanged one is ambiguous, so it is republished only once it
                # has outlived any plausible internal period.
                if dist != last_dist or (now - last_pub) >= LIDAR_STABLE_REPUBLISH_S:
                    state['seq'] += 1
                    state['ts'] = now
                    last_pub = now
                last_dist = dist
            fail_streak = 0
        except Exception as e:
            fail_streak += 1
            if fail_streak == 1:
                print(f"WARNING: LiDAR read failed ({e!r})")
            state['dist'] = None
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
    lidar_state = {'dist': None, 'seq': 0, 'ts': None}
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
