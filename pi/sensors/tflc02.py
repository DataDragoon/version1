"""TF-LC02 LiDAR driver over UART (command-response protocol)."""

import serial
import time

FRAME_HEADER = bytes([0x55, 0xAA])
FRAME_FOOTER = 0xFA
CMD_GET_DISTANCE = bytes([0x55, 0xAA, 0x81, 0x00, 0xFA])

# Why a failed read carries a REASON rather than a bare None.
#
# Every one of these used to collapse into `None`, and that single fact is what
# made a LiDAR dropout undiagnosable from anywhere in the system -- no Pi log,
# no field on the wire, no panel readout could tell "the sensor is replying
# normally and says it cannot range the target" from "the sensor is dead".
# Measured 2026-09-11 on the bench: total dropouts of 9-22 s during which the
# UART stayed perfectly healthy (177.6 commands/s out, 177.9 replies/s back,
# zero frame/parity/overrun errors per TIOCGICOUNT) -- so the failure was
# entirely in the payload, and nothing recorded which payload.
#
# The two classes need opposite responses, which is the whole point of
# separating them:
#   SENSOR_*  -- the link works and the module answered. A sustained run of
#                these is a SIGHTING problem (target out of range, too oblique,
#                too dark, ambient IR). Aim the head, do not re-check cables.
#   LINK_*    -- the module did not answer, or answered with something that is
#                not a frame. That is the wiring/power/baud class, and it is the
#                one the 2026-08-24 investigation chased at length.
REASON_OK = 'ok'

# The module reports "I cannot measure this" as a literal distance of 8888 with a
# non-zero error code. Measured 2026-09-11 over 33,825 failing reads pointed across
# a room: codes 4, 6, 20, 22 and 54 ALL returned exactly 8888 and nothing else, while
# 43,158 in-range reads returned real distances with code 0 and never 8888. So the
# sentinel -- not the code -- is the reliable discriminator, and no usable measurement
# is being thrown away when a non-zero code is rejected.
#
# This is worth naming separately rather than lumping in with the other failures,
# because it is NOT a fault: the sensor is working and the target is simply beyond
# what it can reach. Reported as `oor:<code>` so the code is still visible (the bits
# differ between an isolated miss and a sustained out-of-range run) while the common
# case reads plainly in a log.
OUT_OF_RANGE_DISTANCE = 8888
LINK_NO_BYTES = 'link:no_bytes'        # nothing arrived inside the read timeout
LINK_NO_HEADER = 'link:no_header'      # bytes arrived but no 55 AA in them
LINK_SHORT = 'link:short_frame'        # header found too late to hold a frame
LINK_BAD_FOOTER = 'link:bad_footer'    # framing lost (desync, wrong baud, noise)
LINK_BAD_OPCODE = 'link:bad_opcode'    # a reply, but not to the distance query


def is_link_reason(reason):
    """True if this failure means the module did not answer properly at all."""
    return isinstance(reason, str) and reason.startswith('link:')


def is_out_of_range_reason(reason):
    """True if the module answered and said the target is beyond its range."""
    return isinstance(reason, str) and reason.startswith('oor:')


class TFLC02:
    def __init__(self, port='/dev/ttyAMA3', baudrate=115200):
        self.ser = serial.Serial(port, baudrate=baudrate, timeout=0.1)
        self.ser.reset_input_buffer()
        time.sleep(0.2)

    def read_distance(self):
        """Request and read one distance measurement.

        Returns distance in mm, or None on any failure. Kept for callers that
        do not care why; prefer read_distance_detail() so a dropout can name
        its own cause."""
        dist, _reason = self.read_distance_detail()
        return dist

    def read_distance_detail(self):
        """Request one measurement. Returns (distance_mm or None, reason).

        `reason` is REASON_OK on success, one of the LINK_* constants when the
        module did not answer with a usable frame, or `sensor:<n>` when it
        answered normally with a non-zero error code. Note the module reports an
        invalid/no-return measurement as a literal distance of 8888 with
        error_code 4 -- returning that as if it were real is a spurious 8.888 m
        jump in the standoff display, so a non-zero error code is always a
        failure here however plausible the distance looks."""
        self.ser.reset_input_buffer()
        self.ser.write(CMD_GET_DISTANCE)
        dist, error_code, reason = self._read_response()
        if reason != REASON_OK:
            return None, reason
        if error_code != 0:
            # 8888 is the out-of-range sentinel, never a measurement -- see
            # OUT_OF_RANGE_DISTANCE. Either way the caller gets None; only the
            # reason differs, so nothing downstream can accidentally use 8888.
            if dist == OUT_OF_RANGE_DISTANCE:
                return None, f'oor:{error_code}'
            return None, f'sensor:{error_code}'
        return dist, REASON_OK

    def read_distance_with_error(self):
        """Request distance. Returns (distance_mm, error_code) or None.

        Unlike read_distance_detail() this hands back the raw reading even when
        error_code is non-zero, so a diagnostic tool can see the 8888 sentinel.
        Used by lidar_noise_char.py."""
        self.ser.reset_input_buffer()
        self.ser.write(CMD_GET_DISTANCE)
        dist, error_code, reason = self._read_response()
        if reason != REASON_OK:
            return None
        return (dist, error_code)

    def _read_response(self):
        """Parse response: 55 AA 81 03 [dist_hi] [dist_lo] [error_code] FA

        Returns (distance, error_code, reason). Distance and error_code are the
        raw frame contents and are None unless reason is REASON_OK; checking the
        error code is the caller's job, because read_distance_with_error() needs
        to see a frame this method would otherwise reject."""
        data = self.ser.read(8)
        if len(data) < 8:
            return None, None, LINK_NO_BYTES

        idx = data.find(FRAME_HEADER)
        if idx < 0:
            return None, None, LINK_NO_HEADER
        if idx > 0:
            # Header not at start, read the remainder of the frame.
            remaining = self.ser.read(idx)
            data = data[idx:] + remaining
            if len(data) < 8:
                return None, None, LINK_SHORT

        if data[7] != FRAME_FOOTER:
            return None, None, LINK_BAD_FOOTER
        if data[2] != 0x81:
            return None, None, LINK_BAD_OPCODE

        return data[4] * 256 + data[5], data[6], REASON_OK

    def close(self):
        self.ser.close()
