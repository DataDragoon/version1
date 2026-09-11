"""TF-LC02 LiDAR driver over UART (command-response protocol)."""

import serial
import time

FRAME_HEADER = bytes([0x55, 0xAA])
FRAME_FOOTER = 0xFA
CMD_GET_DISTANCE = bytes([0x55, 0xAA, 0x81, 0x00, 0xFA])


class TFLC02:
    def __init__(self, port='/dev/ttyAMA3', baudrate=115200):
        self.ser = serial.Serial(port, baudrate=baudrate, timeout=0.1)
        self.ser.reset_input_buffer()
        time.sleep(0.2)

    def read_distance(self):
        """Request and read one distance measurement. Returns distance in mm, or None on error."""
        self.ser.reset_input_buffer()
        self.ser.write(CMD_GET_DISTANCE)
        return self._read_response()

    def _read_response(self):
        """Parse response: 55 AA 81 03 [dist_hi] [dist_lo] [error_code] FA"""
        # Read up to 8 bytes with sync
        data = self.ser.read(8)
        if len(data) < 8:
            return None

        # Find frame header
        idx = data.find(FRAME_HEADER)
        if idx < 0:
            return None
        if idx > 0:
            # Header not at start, read remaining bytes
            remaining = self.ser.read(idx)
            data = data[idx:] + remaining
            if len(data) < 8:
                return None

        if data[7] != FRAME_FOOTER:
            return None
        if data[2] != 0x81:
            return None

        dist = data[4] * 256 + data[5]
        error_code = data[6]
        if error_code != 0:
            return None
        return dist

    def read_distance_with_error(self):
        """Request distance. Returns (distance_mm, error_code) or None."""
        self.ser.reset_input_buffer()
        self.ser.write(CMD_GET_DISTANCE)

        data = self.ser.read(8)
        if len(data) < 8:
            return None

        idx = data.find(FRAME_HEADER)
        if idx < 0:
            return None
        if idx > 0:
            remaining = self.ser.read(idx)
            data = data[idx:] + remaining
            if len(data) < 8:
                return None

        if data[7] != FRAME_FOOTER:
            return None
        if data[2] != 0x81:
            return None

        dist = data[4] * 256 + data[5]
        error_code = data[6]
        return (dist, error_code)

    def close(self):
        self.ser.close()
