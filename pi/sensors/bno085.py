"""BNO085 IMU driver over I2C (SHTP / SH-2 protocol).

Raw SHTP implementation over smbus2 rather than adafruit-blinka: this repo's other
sensor drivers (mpu6500.py, tflc02.py) are direct register/byte-protocol drivers with
no framework dependency, and adafruit_bno08x has a real bug conflating host-TX and
device-RX sequence numbers per channel (see its __init__.py `_sequence_number` TODO) —
harmless here since we track our own independently, but not something to depend on
unreviewed.

Two things worth knowing if this needs debugging on the bench again:
  - The SH-2 app firmware must actually be running for feature reports to work.
    Product ID queries (the control-channel handshake) succeed even when it isn't —
    that got mistaken for "communication is fine" once already. If enable_feature()
    starts timing out again with product ID queries still working, don't re-chase
    protocol bugs — power-cycle the board first.
  - Every I2C transaction here is a single `i2c_rdwr` read of a fixed-size buffer
    (`_read`), never a header-peek followed by a separate body read. SHTP-over-I2C
    re-presents a pending packet from byte 0 on every new transaction, so splitting
    header and body reads into two transactions silently desyncs them.
"""

import struct
import time

import smbus2

ADDR = 0x4A

_CHAN_SHTP_COMMAND = 0
_CHAN_EXE = 1
_CHAN_CONTROL = 2
_CHAN_INPUT_REPORTS = 3

_PRODUCT_ID_REQUEST = 0xF9
_PRODUCT_ID_RESPONSE = 0xF8
_SET_FEATURE_COMMAND = 0xFD
_GET_FEATURE_RESPONSE = 0xFC
_BASE_TIMESTAMP = 0xFB
_TIMESTAMP_REBASE = 0xFA

REPORT_ACCELEROMETER = 0x01
REPORT_GYROSCOPE = 0x02

# report_id -> total record length in bytes, for splitting a batched packet.
_REPORT_LENGTHS = {
    _BASE_TIMESTAMP: 5,
    _TIMESTAMP_REBASE: 5,
    REPORT_ACCELEROMETER: 10,
    REPORT_GYROSCOPE: 10,
}

_ACCEL_SCALE = 2**-8  # Q8, m/s^2
_GYRO_SCALE = 2**-9  # Q9, rad/s
_MS2_TO_G = 1.0 / 9.80665
_RAD_TO_DEG = 180.0 / 3.141592653589793

# Deliberately small: a full-size I2C transaction costs real time (128 bytes at
# 100kHz is ~12ms; 48 bytes is ~4ms, measured on this Pi) and read_all() below
# drains in a loop, so oversizing this directly slows every poll. 48 bytes covers
# a base-timestamp record plus both sensor reports batched together (5+10+10=25)
# with headroom, as long as the reports don't pile up faster than we drain them —
# see the interval choice below.
_MAX_PACKET = 48


class BNO085:
    # 20ms (50Hz) matches stream.py's default loop rate. This isn't just a nicety:
    # the BNO085 pushes reports on its own schedule regardless of whether the host
    # is draining them, so an interval faster than the host can actually service
    # (one _read() transaction is a few ms, not free) makes the report queue back
    # up without bound — read_all()'s drain loop then hits its cap on every single
    # call, batches grow past _MAX_PACKET and get silently truncated, and
    # calibrate_gyro_bias's 200Hz sampling loop effectively never finishes. Confirmed
    # this exact hang at the old 10ms/100Hz default; don't drop below ~20ms without
    # re-checking read_all()'s per-call read count stays small in steady state.
    def __init__(self, bus_num=1, addr=ADDR, accel_interval_us=20000, gyro_interval_us=20000):
        self.bus = smbus2.SMBus(bus_num)
        self.addr = addr
        self._tx_seq = [0] * 6
        self._latest_accel = (0.0, 0.0, 0.0)
        self._latest_gyro = (0.0, 0.0, 0.0)
        self.part_number = None

        self._soft_reset()
        self._check_product_id()
        self._enable_feature(REPORT_ACCELEROMETER, accel_interval_us)
        self._enable_feature(REPORT_GYROSCOPE, gyro_interval_us)

    def who_am_i(self):
        return self.part_number

    def read_all(self):
        """Drain whatever sensor reports are currently pending and return the
        latest accel (g) / gyro (deg/s). Non-blocking: returns immediately once
        the device has nothing left queued, using the last known reading for
        anything that hasn't updated since the previous call."""
        for _ in range(8):
            packet = self._read()
            if packet is None:
                break
            if packet['channel'] == _CHAN_INPUT_REPORTS:
                self._handle_input_reports(packet['data'])

        return {
            'accel': self._latest_accel,
            'gyro': self._latest_gyro,
            'temp': None,
        }

    def close(self):
        self.bus.close()

    # ---------------- SHTP transport ----------------

    def _send(self, channel, data):
        header = struct.pack('<HBB', len(data) + 4, channel, self._tx_seq[channel])
        self._tx_seq[channel] = (self._tx_seq[channel] + 1) % 256
        self.bus.i2c_rdwr(smbus2.i2c_msg.write(self.addr, header + bytes(data)))

    def _read(self, maxlen=_MAX_PACKET):
        msg = smbus2.i2c_msg.read(self.addr, maxlen)
        self.bus.i2c_rdwr(msg)
        buf = bytes(msg)
        packet_len, channel, seq = struct.unpack_from('<HBB', buf)
        packet_len &= 0x7FFF
        if packet_len == 0:
            return None
        data_len = packet_len - 4
        return {'channel': channel, 'seq': seq, 'data': buf[4:4 + data_len]}

    def _drain(self, duration_s):
        t0 = time.monotonic()
        while time.monotonic() - t0 < duration_s:
            self._read()
            time.sleep(0.01)

    # ---------------- setup ----------------

    def _soft_reset(self):
        self._send(_CHAN_EXE, [0x01])
        time.sleep(0.3)
        self._drain(0.3)

    def _check_product_id(self):
        self._send(_CHAN_CONTROL, [_PRODUCT_ID_REQUEST, 0x00])
        t0 = time.monotonic()
        while time.monotonic() - t0 < 2.0:
            packet = self._read()
            if packet and packet['channel'] == _CHAN_CONTROL and packet['data'] and \
                    packet['data'][0] == _PRODUCT_ID_RESPONSE:
                self.part_number = struct.unpack_from('<I', packet['data'], offset=4)[0]
                return
            time.sleep(0.02)
        raise RuntimeError('BNO085: no product ID response (is it powered / reset?)')

    def _enable_feature(self, feature_id, report_interval_us):
        report = bytearray(17)
        report[0] = _SET_FEATURE_COMMAND
        report[1] = feature_id
        struct.pack_into('<I', report, 5, report_interval_us)
        self._send(_CHAN_CONTROL, report)

        t0 = time.monotonic()
        while time.monotonic() - t0 < 2.0:
            packet = self._read()
            if packet and packet['channel'] == _CHAN_CONTROL and packet['data'] and \
                    packet['data'][0] == _GET_FEATURE_RESPONSE and \
                    len(packet['data']) > 1 and packet['data'][1] == feature_id:
                return
            time.sleep(0.02)
        raise RuntimeError(f'BNO085: feature 0x{feature_id:02x} was not confirmed enabled')

    # ---------------- report parsing ----------------

    def _handle_input_reports(self, data):
        i = 0
        n = len(data)
        while i < n:
            report_id = data[i]
            length = _REPORT_LENGTHS.get(report_id)
            if length is None or i + length > n:
                break
            if report_id == REPORT_ACCELEROMETER:
                x, y, z = struct.unpack_from('<hhh', data, offset=i + 4)
                self._latest_accel = (x * _ACCEL_SCALE * _MS2_TO_G,
                                       y * _ACCEL_SCALE * _MS2_TO_G,
                                       z * _ACCEL_SCALE * _MS2_TO_G)
            elif report_id == REPORT_GYROSCOPE:
                x, y, z = struct.unpack_from('<hhh', data, offset=i + 4)
                self._latest_gyro = (x * _GYRO_SCALE * _RAD_TO_DEG,
                                      y * _GYRO_SCALE * _RAD_TO_DEG,
                                      z * _GYRO_SCALE * _RAD_TO_DEG)
            i += length
