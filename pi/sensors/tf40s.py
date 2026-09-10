"""TF40-S LiDAR driver -- Benewake, Modbus RTU over UART.

Replaced the TF-LC02 on 2026-09-11. Shares the wiring but nothing else: 9600
baud (not 115200) and Modbus RTU (not the TF-LC02's `55 AA ...` framing). Kept
as a hand-rolled driver on pyserial with no Modbus library, matching this repo's
other sensor drivers (see bno085.py).

Interface is deliberately identical to the old TFLC02 class -- read_distance()
returns millimetres or None -- so stream.py's poll loop did not have to change
shape.

Two facts about this device that are easy to get wrong:

  * A register READ is a COMMAND. Every documented operation is a `func 0x03`
    read of a magic register: 0x000F takes a measurement, 0x0001 starts
    continuous mode, 0x000A stops it, 0x000D is the slave ID and 0x0000 the baud
    rate. So there is no such thing as a harmless register sweep here -- a blind
    scan issues real mode changes as it goes, and a stray WRITE can strand the
    module at an unknown baud or address. Only send documented frames.

  * `EN_PWR` (pin 5) must be high or the module answers nothing at all. It is a
    5-wire part; the TF-LC02 was 4-wire. On this bench EN is shorted to 3.3 V.

Distance replies carry a 32-bit value whose top byte is a sentinel on failure
(0xFF remeasure, 0xFE weak return, 0xFD too reflective, 0xFC out of range).
read_distance() returns None for all of them -- the same "no reading yet" signal
any other transient failure gives. That mirrors the TF-LC02 fix of 2026-08-24,
where an unchecked error code shipped a literal 8888 as a real 8.888 m standoff.
"""

import serial
import time

# Modbus RTU. Slave 1 is the factory default; func 0x03 (read holding
# registers) is the ONLY function the device accepts for measurement, and
# func 0x10 (write multiple) is used for the laser and for configuration.
DEFAULT_ADDR = 0x01
_READ_HOLDING = 0x03
_WRITE_MULTIPLE = 0x10

REG_DISTANCE = 0x000F      # reading it triggers a single measurement
REG_CONTINUOUS = 0x0001    # reading it starts continuous mode (5 Hz)
REG_STOP_CONT = 0x000A     # reading it stops continuous mode
REG_LASER = 0x0003         # write 1/0 to turn the alignment laser on/off
REG_SLAVE_ID = 0x000D      # write to change the Modbus slave address

# Failure is reported as one of these EXACT 32-bit values -- NOT as a top-byte
# pattern. That distinction is the whole ballgame and it cost real bench time:
# a valid reading is a NEGATIVE signed 32-bit number (see RANGE_OFFSET_MM), so
# it is sign-extended and ALSO begins with 0xFF. An earlier version of this
# driver tested `value >> 24` against these keys and therefore threw away every
# good measurement as an error, which presented as a sensor that talked
# perfectly and could not range at all.
ERROR_SENTINELS = {
    0xFF000000: 'miscalculation, remeasure',
    0xFE000000: 'weak return (low reflectivity or measurement too slow)',
    0xFD000000: 'target too reflective',
    0xFC000000: 'out of measuring range',
}

# The module reports a signed 32-bit distance carrying a large fixed negative
# offset, so a real distance arrives as e.g. -12501 for 1.000 m. Nothing in the
# manual documents this and there is no calibration command in it, so the
# constant is EMPIRICAL, measured on this unit 2026-09-11 against a tape at
# 1.000 m from a flat wall (raw -12501). Slope is taken as exactly 1 count =
# 1 mm, which is the datasheet resolution rather than an assumption.
#
# RE-DERIVE IT, do not trust it blindly, if this module is ever replaced or
# reset: park at a carefully measured distance D and read `raw` (the driver
# prints it in its __main__ self-test), then RANGE_OFFSET_MM = D_mm - raw.
RANGE_OFFSET_MM = 13501

# Modbus exception codes this device returns (in a `01 83 <code>` reply).
MODBUS_EXCEPTIONS = {
    0x01: 'illegal function',
    0x02: 'illegal start address',
    0x03: 'illegal register count',
    0x04: 'illegal register value',
}

# The device measures at 5 Hz, so a triggered read can legitimately take up to
# ~200 ms to answer. The old TF-LC02 timeout of 0.1 s would have timed out on
# perfectly good measurements.
DEFAULT_TIMEOUT_S = 0.3

# In continuous mode a frame is due every ~172 ms (measured 5.8 Hz). Two periods
# of slack absorbs one dropped frame without declaring the sensor silent.
STREAM_FRAME_TIMEOUT_S = 0.5

# If the stream goes quiet for this long the start command is re-sent. A
# `start continuous` issued while the module is still digesting the previous
# `stop` is silently DROPPED, and the module then never streams -- so a process
# restarting straight after another one closed could sit publishing nulls for
# ever with no error anywhere. Observed exactly that on the bench 2026-09-11:
# 0 readings in 9 s, while the same driver run standalone a moment later was
# perfect. Re-arming is cheap and idempotent, so it is done on a timer rather
# than trying to detect the race.
STREAM_RESTART_AFTER_S = 1.5


def crc16(data):
    """Modbus RTU CRC-16 (poly 0xA001, init 0xFFFF), returned host-order."""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def _frame(body):
    """Append the CRC (low byte first, as Modbus RTU sends it)."""
    c = crc16(body)
    return bytes(body) + bytes([c & 0xFF, c >> 8])


class TF40SError(Exception):
    pass


class TF40S:
    def __init__(self, port='/dev/ttyAMA3', baudrate=9600, address=DEFAULT_ADDR,
                 timeout=DEFAULT_TIMEOUT_S, continuous=True):
        self.address = address
        self.last_error = None
        self._buf = b''
        self._continuous = False
        self._last_start = 0.0
        self.ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)
        self.ser.reset_input_buffer()
        # The module emits `01 03 02 00 00 B8 44` when EN goes high and it
        # finishes initialising. If we opened the port long after boot that has
        # already gone; either way the buffer must start clean.
        time.sleep(0.05)
        # Always stop first: continuous mode PERSISTS in the module across host
        # restarts, so a process that died without stopping leaves the next one
        # opening onto a live stream it did not start.
        self._send(REG_STOP_CONT)
        time.sleep(0.2)
        self.ser.reset_input_buffer()
        if continuous:
            self.start_continuous()

    # -- why continuous is the default ----------------------------------
    #
    # MEASURED ON THE BENCH 2026-09-11, and the gap is large. Single-shot
    # polling (a `func 0x03` read of 0x000F per reading) answers only 16-52% of
    # requests for ~0.5-1.1 readings/s, whatever the pacing: the module measures
    # on its own 5 Hz cadence and simply drops a request that arrives mid
    # measurement. Continuous mode gave 58/58 well-formed CRC-valid frames at a
    # steady 5.8 Hz over the same link.
    #
    # It also fits stream.py's poll loop better than polling does: read_distance
    # blocks until the next streamed frame, so the loop SELF-PACES at the
    # sensor's real cadence and every successful read is a genuinely new
    # measurement rather than a re-read of a cached one.

    # -- transport -------------------------------------------------------

    def _send(self, reg, count=2):
        """Fire a register command without waiting for its reply."""
        self.ser.write(_frame([self.address, _READ_HOLDING,
                               reg >> 8, reg & 0xFF, count >> 8, count & 0xFF]))
        self.ser.flush()

    def _next_stream_frame(self, deadline):
        """Pull the next CRC-valid 9-byte reply out of the continuous stream.

        Resynchronises by scanning, because the buffer can start mid-frame after
        a dropped byte -- the CRC is what decides, never the position.
        """
        while True:
            i = 0
            while i + 9 <= len(self._buf):
                f = self._buf[i:i + 9]
                if (f[0] == self.address and f[1] == _READ_HOLDING and f[2] == 4
                        and crc16(f[:7]) == (f[7] | (f[8] << 8))):
                    self._buf = self._buf[i + 9:]
                    return f[3:7]
                i += 1
            # Keep only a partial frame's worth; anything older cannot start one.
            if len(self._buf) > 8:
                self._buf = self._buf[-8:]
            now = time.time()
            if now >= deadline:
                # Self-heal a dropped start command (see STREAM_RESTART_AFTER_S).
                if self._continuous and (now - self._last_start) >= STREAM_RESTART_AFTER_S:
                    self._send(REG_CONTINUOUS)
                    self._last_start = now
                    self.last_error = 'no stream frame (restarted)'
                else:
                    self.last_error = 'no stream frame'
                return None
            chunk = self.ser.read(64)
            if chunk:
                self._buf += chunk

    def _transact(self, body, expected_data_bytes):
        """Send one Modbus request, return its data payload, or None.

        None means no usable answer -- silence, a short frame, a bad CRC, or a
        Modbus exception. Callers treat all of those the same way, as "no
        reading yet"; the distinction is only useful when diagnosing, which is
        what last_error is for.
        """
        self.last_error = None
        self.ser.reset_input_buffer()
        self.ser.write(_frame(body))
        self.ser.flush()

        header = self.ser.read(3)
        if len(header) < 3:
            self.last_error = 'timeout'
            return None

        if header[0] != self.address:
            self.last_error = f'wrong slave 0x{header[0]:02x}'
            return None

        # Exception reply: addr, func|0x80, code, crc_lo, crc_hi
        if header[1] & 0x80:
            self.ser.read(2)
            code = header[2]
            self.last_error = f'modbus exception {code} ({MODBUS_EXCEPTIONS.get(code, "unknown")})'
            return None

        n = header[2]
        rest = self.ser.read(n + 2)
        if len(rest) < n + 2:
            self.last_error = 'short frame'
            return None

        payload = header + rest[:n]
        c = crc16(payload)
        if bytes([c & 0xFF, c >> 8]) != rest[n:]:
            self.last_error = 'bad CRC'
            return None

        if n != expected_data_bytes:
            self.last_error = f'unexpected length {n} (wanted {expected_data_bytes})'
            return None

        return rest[:n]

    def _read_registers(self, reg, count, expected_data_bytes):
        return self._transact(
            [self.address, _READ_HOLDING, reg >> 8, reg & 0xFF, count >> 8, count & 0xFF],
            expected_data_bytes)

    # -- measurement -----------------------------------------------------

    def read_distance(self):
        """Trigger one measurement. Returns distance in mm, or None on error.

        None covers both a failed transaction and a measurement the sensor
        itself reports as invalid -- see ERROR_SENTINELS. Use
        read_distance_with_error() when the distinction matters.
        """
        result = self.read_distance_with_error()
        if result is None:
            return None
        dist, err = result
        return None if err is not None else dist

    def read_distance_with_error(self):
        """Trigger one measurement.

        Returns (distance_mm, None) on success, (None, sentinel) when the
        sensor reports the measurement invalid, or None if the transaction
        itself failed (see last_error). The sentinel is the full 32-bit code.
        """
        self.last_error = None
        if self._continuous:
            data = self._next_stream_frame(time.time() + STREAM_FRAME_TIMEOUT_S)
        else:
            data = self._read_registers(REG_DISTANCE, 2, 4)
        if data is None:
            return None
        raw = int.from_bytes(data, 'big')
        if raw in ERROR_SENTINELS:
            return (None, raw)
        # Signed, then de-offset. Both steps are required -- see the comments on
        # ERROR_SENTINELS and RANGE_OFFSET_MM.
        signed = int.from_bytes(data, 'big', signed=True)
        return (signed + RANGE_OFFSET_MM, None)

    # -- optional modes / alignment aids ---------------------------------

    def start_continuous(self):  # noqa: D401
        """Put the module into its own 5 Hz streaming mode.

        Not used by stream.py, which polls -- polling keeps each measurement
        tied to a request we timestamped ourselves. Exposed because it is the
        cheaper way to run if the timing ever needs it.
        """
        self.ser.reset_input_buffer()
        self._buf = b''
        self._send(REG_CONTINUOUS)
        self._continuous = True
        self._last_start = time.time()
        return True

    def stop_continuous(self):
        self._continuous = False
        self._send(REG_STOP_CONT)
        time.sleep(0.2)
        self.ser.reset_input_buffer()
        self._buf = b''
        return True

    def set_laser(self, on):
        """Turn the visible alignment laser on or off (CLASS 2, 635 nm)."""
        body = [self.address, _WRITE_MULTIPLE, REG_LASER >> 8, REG_LASER & 0xFF,
                0x00, 0x01, 0x02, 0x00, 0x01 if on else 0x00]
        self.ser.reset_input_buffer()
        self.ser.write(_frame(body))
        self.ser.flush()
        return len(self.ser.read(8)) > 0

    @property
    def continuous(self):
        """True while the module is streaming.

        Consumers use this to decide whether a successful read is a distinct
        MEASUREMENT. In continuous mode it always is -- the module emits one
        frame per measurement on its own 5 Hz clock -- so nothing has to guess
        from whether the value changed.
        """
        return self._continuous

    def ping(self):
        """True if the module answers at this driver's address."""
        return self._read_registers(REG_DISTANCE, 2, 4) is not None

    def close(self):
        # Leaving it streaming would hand the next process a live stream it did
        # not start; __init__ also stops, but only one of the two can be relied
        # on if a process is killed.
        try:
            if self._continuous:
                self.stop_continuous()
        except Exception:
            pass
        self.ser.close()


def recover_address(port='/dev/ttyAMA3', baudrate=9600, new_address=DEFAULT_ADDR):
    """Force the module's slave ID back to `new_address` via Modbus BROADCAST.

    THIS IS A REAL FAILURE MODE, SEEN ON THE BENCH 2026-09-11. The module's
    slave ID had become **0**, which is the Modbus broadcast address, and that
    is close to undiagnosable from the outside:

      * a device on address 0 executes broadcast commands but, by Modbus spec,
        NEVER answers a unicast request -- so it looks completely dead;
      * scanning slave addresses 1..247 therefore finds nothing, at any baud;
      * passive listening finds nothing either, since it only speaks when asked.

    What gives it away is that it still ACTS: broadcasting the laser-on command
    lights the visible laser even while nothing answers. Once that is seen, this
    recovers it -- the write is broadcast, so it lands whatever the current ID is.

    Nothing here can be verified from the reply (a broadcast write is not
    answered by a conforming device -- ours did answer, which is itself a symptom
    of the address-0 state), so the check is a unicast read afterwards.
    """
    s = serial.Serial(port, baudrate=baudrate, timeout=DEFAULT_TIMEOUT_S)
    try:
        body = [0x00, _WRITE_MULTIPLE, REG_SLAVE_ID >> 8, REG_SLAVE_ID & 0xFF,
                0x00, 0x01, 0x02, new_address >> 8, new_address & 0xFF]
        s.reset_input_buffer()
        s.write(_frame(body))
        s.flush()
        time.sleep(0.5)
        s.read(32)
    finally:
        s.close()

    lidar = TF40S(port=port, baudrate=baudrate, address=new_address)
    try:
        # A measurement can legitimately fail; what is being checked here is
        # only whether the module ANSWERS at the new address.
        for _ in range(5):
            if lidar._read_registers(REG_DISTANCE, 2, 4) is not None:
                return True
            time.sleep(0.25)
        return False
    finally:
        lidar.close()


if __name__ == '__main__':
    import argparse
    ap = argparse.ArgumentParser(description='TF40-S quick check')
    ap.add_argument('--port', default='/dev/ttyAMA3')
    ap.add_argument('--baud', type=int, default=9600)
    ap.add_argument('-n', type=int, default=20, help='number of reads')
    a = ap.parse_args()

    lidar = TF40S(port=a.port, baudrate=a.baud)
    print(f"TF40-S on {lidar.ser.port} @ {lidar.ser.baudrate}")
    ok, vals = 0, []
    t0 = time.time()
    for i in range(a.n):
        r = lidar.read_distance_with_error()
        if r is None:
            print(f"  {i+1:3d}: no reply ({lidar.last_error})")
        elif r[1] is not None:
            print(f"  {i+1:3d}: sensor error 0x{r[1]:08X} -- {ERROR_SENTINELS[r[1]]}")
        else:
            ok += 1
            vals.append(r[0])
            print(f"  {i+1:3d}: {r[0]:6d} mm ({r[0]/1000:.3f} m)   "
                  f"[raw {r[0] - RANGE_OFFSET_MM}]")
    el = time.time() - t0
    print(f"\n{ok}/{a.n} valid in {el:.2f}s ({a.n/el:.1f} reads/s)")
    if len(vals) > 1:
        m = sum(vals) / len(vals)
        sd = (sum((v - m) ** 2 for v in vals) / len(vals)) ** 0.5
        print(f"mean {m:.1f} mm, sd {sd:.2f} mm, min {min(vals)}, max {max(vals)}")
    lidar.close()
