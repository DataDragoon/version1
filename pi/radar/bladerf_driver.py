"""bladeRF hardware abstraction — supports dual TX/RX for SFCW reference channel."""

import threading
import time
import numpy as np
import bladerf
from bladerf._bladerf import ChannelLayout, Format, ffi, libbladeRF

SCALE = 2047
MGC = libbladeRF.BLADERF_GAIN_MGC
TUNING_MODE_FPGA = libbladeRF.BLADERF_TUNING_MODE_FPGA


# ---------------------------------------------------------------------------
# bladerf_format values, resolved by NAME rather than through Format.<X>.
#
# The Python bindings installed on a Pi can be older than libbladeRF.so. The
# one that matters here predates BLADERF_FORMAT_SC16_Q11_PACKED, and because
# bladerf_format is a plain C enum, dropping a member shifts every later one
# down by one:
#
#     canonical (.so)   SC16_Q11 0  PACKED 1  META 2  PACKET_META 3  SC8 4 ...
#     stale binding     SC16_Q11 0            META 1  PACKET_META 2  SC8 3 ...
#
# sync_config passes fmt.value straight through, so Format.SC16_Q11_META sends
# 1 and the library reads SC16_Q11_PACKED. The visible symptoms are a buffer
# size computed at 3 bytes/sample ("4096 samples (12288 bytes)") and then
# BLADERF_ERR_INVAL from perform_format_config, because the shifted RX and TX
# formats disagree about timestamps.
#
# Detection rule: if the installed bindings expose SC16_Q11_PACKED they were
# generated against a header that has it, so they agree with the library and
# are used unchanged. If they do not, they are stale and the canonical values
# are used instead.
#
# This is a shim, not a fix. The fix is to install the bindings from
# bladerf-src/host/libraries/libbladeRF_bindings/python on the Pi, which also
# brings dsp_path_enabled and unpack_dsp_results.
# ---------------------------------------------------------------------------

_CANONICAL_FORMAT = {
    'SC16_Q11':        0,
    'SC16_Q11_PACKED': 1,
    'SC16_Q11_META':   2,
    'PACKET_META':     3,
    'SC8_Q7':          4,
    'SC8_Q7_META':     5,
}

def _detect_stale_bindings():
    """True when the binding's Format enum disagrees with libbladeRF.h.

    Checks EVERY member, not one of them. An earlier version tested only for
    SC16_Q11_PACKED and concluded the bindings were fine -- but the binding
    actually shipping on the Pi is missing SC16_Q11_META instead:

        installed   SC16_Q11 0  PACKED 1  PACKET_META 2  SC8_Q7 3  SC8_Q7_META 4
        canonical   SC16_Q11 0  PACKED 1  META 2  PACKET_META 3  SC8_Q7 4  ...

    so PACKET_META asks for 2 and the library delivers SC16_Q11_META, and
    SC16_Q11_META cannot be named at all. Any missing or shifted member means
    the whole enum is untrustworthy.
    """
    for name, want in _CANONICAL_FORMAT.items():
        member = getattr(Format, name, None)
        if member is None or member.value != want:
            return True
    return False


_BINDINGS_STALE = _detect_stale_bindings()
if _BINDINGS_STALE:
    _present = {m.name: m.value for m in Format}
    print("[bladerf] WARNING: installed Python bindings disagree with "
          "libbladeRF's sample-format enum; correcting in software.")
    print("[bladerf]   binding:   {}".format(_present))
    print("[bladerf]   canonical: {}".format(_CANONICAL_FORMAT))
    print("[bladerf]   install the bindings from bladerf-src to remove this.")


class _Fmt:
    """Duck-types Format for sync_config, which only ever reads .value."""
    __slots__ = ('name', 'value')

    def __init__(self, name, value):
        self.name = name
        self.value = value

    def __repr__(self):
        return "<Format.{}: {}>".format(self.name, self.value)


def fmt(name):
    """Resolve a bladerf_format by name to the value libbladeRF.so expects."""
    if not _BINDINGS_STALE:
        return getattr(Format, name)
    return _Fmt(name, _CANONICAL_FORMAT[name])


# RX sync ring depth (buffers) for dual-channel streaming -- see start_rx_dual.
RX_RING_DEPTH = 256


class BladeRFDriver:
    def __init__(self):
        self.device = None
        self.tx_running = False
        self.rx_running = False
        self.center_freq = 2_000_000_000
        self.sample_rate = 10_000_000
        self.bandwidth = 1_500_000
        self.tx_gain = 50
        self.rx_gain = 25
        self.tx2_gain = 10
        self.rx2_gain = 0
        self.waveform_type = 'cw'
        self.cw_offset = 100_000
        self.tx_amplitude = 1.0
        self.chirp_bw = 500_000
        self.chirp_duration = 0.001
        self.serial = None
        self._tx_thread = None
        self._rx_thread = None
        self._tx_stop = threading.Event()
        self._rx_stop = threading.Event()
        self._lock = threading.Lock()
        self._tx_buffer = None
        self._dual_channel = False

    def open(self):
        self.device = bladerf.BladeRF()
        self.serial = self.device.get_serial()
        self._configure_channels()

    def close(self):
        self.stop_tx()
        self.stop_rx()
        if self.device:
            self.device.close()
            self.device = None

    def reset(self):
        """Full device close + reopen. Clears all USB/RFIC state."""
        self.stop_tx()
        self.stop_rx()
        self.stop_tx_dual()
        self.stop_rx_dual()
        if self.device:
            self.device.close()
        self.device = bladerf.BladeRF()
        self.serial = self.device.get_serial()
        self._configure_channels()
        print("[bladerf] Device reset complete")

    def _configure_channels(self):
        ch_tx = self.device.Channel(bladerf.CHANNEL_TX(0))
        ch_rx = self.device.Channel(bladerf.CHANNEL_RX(0))
        ch_rx.gain_mode = MGC
        ch_tx.frequency = int(self.center_freq)
        ch_tx.sample_rate = int(self.sample_rate)
        self._warn_if_rate_snapped('TX0', ch_tx.sample_rate)
        ch_tx.bandwidth = int(self.bandwidth)
        ch_tx.gain = int(self.tx_gain)
        ch_rx.frequency = int(self.center_freq)
        ch_rx.sample_rate = int(self.sample_rate)
        self._warn_if_rate_snapped('RX0', ch_rx.sample_rate)
        ch_rx.bandwidth = int(self.bandwidth)
        ch_rx.gain = int(self.rx_gain)

    def _configure_channels_dual(self):
        """Configure all 4 channels (TX1+TX2, RX1+RX2) for SFCW reference mode."""
        dev_ptr = self.device.dev[0]
        gains_tx = [int(self.tx_gain), int(self.tx2_gain)]
        gains_rx = [int(self.rx_gain), int(self.rx2_gain)]
        actual_rate = ffi.new('unsigned int *')

        for ch_idx in range(2):
            tx_ch = bladerf.CHANNEL_TX(ch_idx)
            rx_ch = bladerf.CHANNEL_RX(ch_idx)
            libbladeRF.bladerf_set_frequency(dev_ptr, tx_ch, int(self.center_freq))
            libbladeRF.bladerf_set_sample_rate(dev_ptr, tx_ch, int(self.sample_rate), actual_rate)
            self._warn_if_rate_snapped(f'TX{ch_idx}', actual_rate[0])
            libbladeRF.bladerf_set_bandwidth(dev_ptr, tx_ch, int(self.bandwidth), ffi.NULL)
            libbladeRF.bladerf_set_frequency(dev_ptr, rx_ch, int(self.center_freq))
            libbladeRF.bladerf_set_sample_rate(dev_ptr, rx_ch, int(self.sample_rate), actual_rate)
            self._warn_if_rate_snapped(f'RX{ch_idx}', actual_rate[0])
            libbladeRF.bladerf_set_bandwidth(dev_ptr, rx_ch, int(self.bandwidth), ffi.NULL)
            libbladeRF.bladerf_set_gain_mode(dev_ptr, rx_ch, MGC)
            libbladeRF.bladerf_set_gain(dev_ptr, rx_ch, gains_rx[ch_idx])
            libbladeRF.bladerf_set_gain(dev_ptr, tx_ch, gains_tx[ch_idx])

        print(f"[bladerf] Dual-channel configured: TX1={gains_tx[0]}dB TX2={gains_tx[1]}dB RX1={gains_rx[0]}dB RX2={gains_rx[1]}dB")

    def _warn_if_rate_snapped(self, label, actual_hz):
        """bladerf_set_sample_rate can silently round the requested rate to the
        nearest one the RFIC's clock/decimation chain can actually produce —
        the call succeeds either way. Surface it instead of discarding `actual`,
        since a snapped rate is exactly what drives the total-throughput warning
        libbladeRF logs (it sums the *actual* per-channel rate, not the requested one).
        """
        if actual_hz != int(self.sample_rate):
            print(f"[bladerf] NOTE: {label} sample rate snapped to {actual_hz/1e6:g} Msps "
                  f"(requested {self.sample_rate/1e6:g} Msps)")

    def reapply_dual_gains(self):
        """Re-push TX1/TX2/RX1/RX2 gains after enabling TX/RX modules.

        enable_module() resets gain state, so any dual-channel start (start_tx_dual,
        start_rx_dual) needs this called afterward or the gains configured by
        _configure_channels_dual() are silently lost. Safe to call even if only one
        direction's modules are enabled — setting a gain register for a disabled
        module just takes effect whenever it's next enabled.
        """
        dev_ptr = self.device.dev[0]
        libbladeRF.bladerf_set_gain_mode(dev_ptr, bladerf.CHANNEL_RX(0), MGC)
        libbladeRF.bladerf_set_gain_mode(dev_ptr, bladerf.CHANNEL_RX(1), MGC)
        libbladeRF.bladerf_set_gain(dev_ptr, bladerf.CHANNEL_RX(0), int(self.rx_gain))
        libbladeRF.bladerf_set_gain(dev_ptr, bladerf.CHANNEL_RX(1), int(self.rx2_gain))
        libbladeRF.bladerf_set_gain(dev_ptr, bladerf.CHANNEL_TX(0), int(self.tx_gain))
        libbladeRF.bladerf_set_gain(dev_ptr, bladerf.CHANNEL_TX(1), int(self.tx2_gain))

    def set_tuning_mode_fpga(self):
        """Switch to FPGA tuning mode — retunes execute on-FPGA, no USB round-trip."""
        dev_ptr = self.device.dev[0]
        rc = libbladeRF.bladerf_set_tuning_mode(dev_ptr, TUNING_MODE_FPGA)
        if rc != 0:
            print(f"[bladerf] WARNING: set_tuning_mode FPGA returned {rc}")
        else:
            print("[bladerf] Tuning mode set to FPGA")

    def get_timestamp(self, direction):
        """Get current hardware timestamp (in sample counts) for TX or RX direction."""
        dev_ptr = self.device.dev[0]
        ts = ffi.new('uint64_t *')
        rc = libbladeRF.bladerf_get_timestamp(dev_ptr, direction, ts)
        if rc != 0:
            raise RuntimeError(f"bladerf_get_timestamp failed: rc={rc}")
        return ts[0]

    def set_frequency(self, freq_hz):
        with self._lock:
            self.center_freq = int(freq_hz)
            self.device.Channel(bladerf.CHANNEL_TX(0)).frequency = self.center_freq
            self.device.Channel(bladerf.CHANNEL_RX(0)).frequency = self.center_freq
            if self._dual_channel:
                self.device.Channel(bladerf.CHANNEL_TX(1)).frequency = self.center_freq
                self.device.Channel(bladerf.CHANNEL_RX(1)).frequency = self.center_freq

    def set_tx_gain(self, gain_db):
        with self._lock:
            self.tx_gain = int(gain_db)
            self.device.Channel(bladerf.CHANNEL_TX(0)).gain = self.tx_gain

    def set_rx_gain(self, gain_db):
        with self._lock:
            self.rx_gain = int(gain_db)
            ch = self.device.Channel(bladerf.CHANNEL_RX(0))
            ch.gain_mode = MGC
            ch.gain = self.rx_gain

    def set_sample_rate(self, rate):
        with self._lock:
            self.sample_rate = int(rate)
            self.bandwidth = int(rate * 0.75)
            ch_tx = self.device.Channel(bladerf.CHANNEL_TX(0))
            ch_rx = self.device.Channel(bladerf.CHANNEL_RX(0))
            ch_tx.sample_rate = self.sample_rate
            self._warn_if_rate_snapped('TX0', ch_tx.sample_rate)
            ch_tx.bandwidth = self.bandwidth
            ch_rx.sample_rate = self.sample_rate
            self._warn_if_rate_snapped('RX0', ch_rx.sample_rate)
            ch_rx.bandwidth = self.bandwidth
            if self._dual_channel:
                ch_tx2 = self.device.Channel(bladerf.CHANNEL_TX(1))
                ch_rx2 = self.device.Channel(bladerf.CHANNEL_RX(1))
                ch_tx2.sample_rate = self.sample_rate
                self._warn_if_rate_snapped('TX1', ch_tx2.sample_rate)
                ch_tx2.bandwidth = self.bandwidth
                ch_rx2.sample_rate = self.sample_rate
                self._warn_if_rate_snapped('RX1', ch_rx2.sample_rate)
                ch_rx2.bandwidth = self.bandwidth
            self._tx_buffer = self._generate(int(self.sample_rate * 0.01))
            if self._dual_channel:
                self._rebuild_tx_dual_buffer()

    def set_waveform(self, waveform_type, **params):
        with self._lock:
            self.waveform_type = waveform_type
            if 'offset' in params:
                self.cw_offset = int(params['offset'])
            if 'amplitude' in params:
                self.tx_amplitude = float(params['amplitude'])
            if 'chirp_bw' in params:
                self.chirp_bw = int(params['chirp_bw'])
            if 'chirp_duration' in params:
                self.chirp_duration = float(params['chirp_duration'])
            self._tx_buffer = self._generate(int(self.sample_rate * 0.01))
            if self._dual_channel:
                self._rebuild_tx_dual_buffer()

    def _rebuild_tx_dual_buffer(self):
        """Rebuild the interleaved TX1+TX2 buffer from self._tx_buffer.

        Call whenever self._tx_buffer changes while dual TX may be running —
        _tx_loop_dual re-reads _tx_dual_bytes every iteration (like _tx_loop does
        with _tx_buffer), so this is what makes live waveform/rate changes actually
        reach a running dual-channel TX instead of silently doing nothing.
        """
        buf = self._tx_buffer
        n_samples = len(buf) // 2
        tx_dual_buf = np.empty(len(buf) * 2, dtype=np.int16)
        tx_dual_buf[0::4] = buf[0::2]  # TX1 I
        tx_dual_buf[1::4] = buf[1::2]  # TX1 Q
        tx_dual_buf[2::4] = buf[0::2]  # TX2 I
        tx_dual_buf[3::4] = buf[1::2]  # TX2 Q
        self._tx_dual_buf = tx_dual_buf
        self._tx_dual_bytes = tx_dual_buf.tobytes()
        self._tx_dual_n_samples = n_samples

    def _generate(self, num_samples):
        if self.waveform_type == 'chirp':
            return self._gen_chirp(num_samples)
        elif self.waveform_type == 'noise':
            return self._gen_noise(num_samples)
        return self._gen_cw(num_samples)

    def _gen_cw(self, n):
        t = np.arange(n, dtype=np.float64) / self.sample_rate
        phase = 2 * np.pi * self.cw_offset * t
        iq = np.empty(n * 2, dtype=np.int16)
        iq[0::2] = np.clip(np.cos(phase) * self.tx_amplitude * SCALE, -2048, 2047).astype(np.int16)
        iq[1::2] = np.clip(np.sin(phase) * self.tx_amplitude * SCALE, -2048, 2047).astype(np.int16)
        return iq

    def _gen_chirp(self, n):
        t = np.arange(n, dtype=np.float64) / self.sample_rate
        f0 = -self.chirp_bw / 2
        f1 = self.chirp_bw / 2
        t_mod = t % self.chirp_duration
        phase = 2 * np.pi * (f0 * t_mod + (f1 - f0) / (2 * self.chirp_duration) * t_mod ** 2)
        iq = np.empty(n * 2, dtype=np.int16)
        iq[0::2] = np.clip(np.cos(phase) * self.tx_amplitude * SCALE, -2048, 2047).astype(np.int16)
        iq[1::2] = np.clip(np.sin(phase) * self.tx_amplitude * SCALE, -2048, 2047).astype(np.int16)
        return iq

    def _gen_noise(self, n):
        noise = np.random.randn(n * 2) * self.tx_amplitude * SCALE * 0.5
        return np.clip(noise, -2048, 2047).astype(np.int16)

    # -- Single-channel TX/RX (used by RF Calib panel) --

    def start_tx(self):
        if self.tx_running:
            return
        self._tx_buffer = self._generate(int(self.sample_rate * 0.01))
        self._tx_stop.clear()
        self.tx_running = True
        self.device.sync_config(
            layout=ChannelLayout.TX_X1,
            fmt=fmt('SC16_Q11'),
            num_buffers=16,
            buffer_size=4096,
            num_transfers=8,
            stream_timeout=3500
        )
        self.device.enable_module(bladerf.CHANNEL_TX(0), True)
        self._tx_thread = threading.Thread(target=self._tx_loop, daemon=True)
        self._tx_thread.start()

    def _tx_loop(self):
        try:
            while not self._tx_stop.is_set():
                with self._lock:
                    buf = self._tx_buffer
                self.device.sync_tx(buf.tobytes(), len(buf) // 2)
        except Exception as e:
            print(f"[bladerf] TX error: {e}")
        finally:
            try:
                self.device.enable_module(bladerf.CHANNEL_TX(0), False)
            except Exception:
                pass
            self.tx_running = False

    def stop_tx(self):
        if not self.tx_running:
            return
        self._tx_stop.set()
        if self._tx_thread:
            self._tx_thread.join(timeout=2)
            self._tx_thread = None
        self.tx_running = False

    def start_rx(self, callback, num_samples=16384):
        if self.rx_running:
            return
        self._rx_stop.clear()
        self.rx_running = True
        self.device.sync_config(
            layout=ChannelLayout.RX_X1,
            fmt=fmt('SC16_Q11'),
            num_buffers=16,
            buffer_size=4096,
            num_transfers=8,
            stream_timeout=3500
        )
        self.device.enable_module(bladerf.CHANNEL_RX(0), True)
        self._rx_thread = threading.Thread(target=self._rx_loop, args=(callback, num_samples), daemon=True)
        self._rx_thread.start()

    def _rx_loop(self, callback, num_samples):
        buf = bytearray(num_samples * 2 * 2)
        try:
            while not self._rx_stop.is_set():
                self.device.sync_rx(buf, num_samples)
                iq = np.frombuffer(buf, dtype=np.int16).copy()
                callback(iq)
        except Exception as e:
            print(f"[bladerf] RX error: {e}")
        finally:
            try:
                self.device.enable_module(bladerf.CHANNEL_RX(0), False)
            except Exception:
                pass
            self.rx_running = False

    def stop_rx(self):
        if not self.rx_running:
            return
        self._rx_stop.set()
        if self._rx_thread:
            self._rx_thread.join(timeout=2)
            self._rx_thread = None
        self.rx_running = False

    # -- Dual-channel TX/RX (used by SFCW engine for reference channel) --

    def start_tx_dual(self, timestamped=False):
        """Start TX on both channels (TX1=antenna, TX2=reference cable).

        `timestamped` selects SC16_Q11_META instead of SC16_Q11. It is not a
        preference -- it is forced by the RX side. libbladeRF refuses to run
        one direction timestamped and the other not:

            perform_format_config() (bladerf2/common.c)
              requires_timestamps(module_format[other]) != requires_timestamps(this)
                -> BLADERF_ERR_INVAL, "Invalid operation or parameter"

        because the timestamp enable is a single global GPIO bit, not per
        direction. The DSP path's RX is plain SC16_Q11 (start_rx_dsp), so TX
        must be plain too; pass timestamped=True only if RX is going to use a
        *_META format, or sync_config fails outright at stream start.
        """
        if self.tx_running:
            return
        self._tx_buffer = self._generate(int(self.sample_rate * 0.01))
        self._tx_stop.clear()
        self.tx_running = True
        self._dual_channel = True
        self._tx_timestamped = timestamped
        self._rebuild_tx_dual_buffer()
        self.device.sync_config(
            layout=ChannelLayout.TX_X2,
            fmt=fmt('SC16_Q11_META') if timestamped else fmt('SC16_Q11'),
            num_buffers=16,
            buffer_size=4096,
            num_transfers=8,
            stream_timeout=3500
        )
        self.device.enable_module(bladerf.CHANNEL_TX(0), True)
        self.device.enable_module(bladerf.CHANNEL_TX(1), True)
        self._tx_thread = threading.Thread(target=self._tx_loop_dual, daemon=True)
        self._tx_thread.start()

    def _tx_loop_dual(self):
        """TX loop for dual channel — replays the interleaved buffer, re-read each
        iteration (like _tx_loop) so live waveform/rate changes take effect."""
        meta = None
        timestamped = getattr(self, '_tx_timestamped', False)
        if timestamped:
            # SC16_Q11_META demands metadata on every sync_tx, and TX_NOW is
            # only legal ALONGSIDE BURST_START -- handle_tx_parameters() in
            # sync.c returns BLADERF_ERR_INVAL for "TX_NOW was specified
            # without BURST_START". Equally, BURST_START a second time while
            # already in a burst is also ERR_INVAL.
            #
            # So: open the burst once with BURST_START|TX_NOW, then keep
            # feeding it with no flags at all. BURST_END is never sent -- this
            # is a continuous carrier, and ending the burst would gate the
            # transmitter off between buffers.
            meta = ffi.new("struct bladerf_metadata *")
            meta.flags = (self._META_FLAG_TX_BURST_START
                          | self._META_FLAG_TX_NOW)
        try:
            while not self._tx_stop.is_set():
                with self._lock:
                    tx_bytes = self._tx_dual_bytes
                    n_samples = self._tx_dual_n_samples
                if meta is not None:
                    self.device.sync_tx(tx_bytes, n_samples, meta=meta)
                    # Burst is open from here on; further BURST_START would be
                    # rejected.
                    meta.flags = 0
                else:
                    self.device.sync_tx(tx_bytes, n_samples)
        except Exception as e:
            print(f"[bladerf] TX dual error: {e}")
        finally:
            try:
                self.device.enable_module(bladerf.CHANNEL_TX(0), False)
                self.device.enable_module(bladerf.CHANNEL_TX(1), False)
            except Exception:
                pass
            self.tx_running = False

    def stop_tx_dual(self):
        if not self.tx_running:
            return
        self._tx_stop.set()
        if self._tx_thread:
            self._tx_thread.join(timeout=2)
            self._tx_thread = None
        self.tx_running = False
        self._dual_channel = False

    def start_rx_dual(self, callback, num_samples=1024):
        """Start RX on both channels. Callback receives (rx1_iq, rx2_iq) tuple."""
        if self.rx_running:
            return
        self._rx_stop.clear()
        self.rx_running = True
        self._dual_channel = True
        self.device.sync_config(
            layout=ChannelLayout.RX_X2,
            fmt=fmt('SC16_Q11'),
            # 256, not 16 (changed 2026-09-07): the ring is the only thing
            # between an RX-thread stall and DROPPED samples, and stalls up to
            # 50.9 ms have been measured under full-stack load. 16 buffers is
            # 3.3 ms of tolerance; a drop is invisible in SC16_Q11 (no
            # metadata) and shifts every later sample position, which the NIOS
            # autonomous sweep's continuous-capture slicing cannot survive.
            # 256 buffers = 52 ms of stall tolerance at 4 MB of memory. For
            # the standard sweep this converts rare sample loss into delay,
            # which the lockstep settle gate already handles.
            num_buffers=RX_RING_DEPTH,
            buffer_size=4096,
            num_transfers=8,
            stream_timeout=3500
        )
        self.device.enable_module(bladerf.CHANNEL_RX(0), True)
        self.device.enable_module(bladerf.CHANNEL_RX(1), True)
        self._rx_thread = threading.Thread(target=self._rx_loop_dual, args=(callback, num_samples), daemon=True)
        self._rx_thread.start()

    def _rx_loop_dual(self, callback, num_samples):
        """RX loop for dual channel — deinterleaves RX1 and RX2."""
        # RX_X2: interleaved [RX1_I, RX1_Q, RX2_I, RX2_Q, ...]
        # num_samples is per-channel, so total buffer is num_samples * 2 channels * 2 (I+Q) * 2 bytes
        buf = bytearray(num_samples * 2 * 2 * 2)
        # libbladeRF counts sync_rx's num_samples as the TOTAL across both channels in
        # RX_X2, not per channel — so asking for num_samples here returned only
        # num_samples/2 per channel and left the upper half of buf untouched, i.e.
        # holding the PREVIOUS iteration's samples (buf is reused). Every capture was
        # half fresh, half one buffer stale, and every buffer-count-to-time conversion
        # in this repo (settle_count, SfcwPanel's BUFFER_TIME_MS) was 2x off as a result.
        # Verified 2026-08-29 by poisoning buf with 0xAA before the call: at num_samples
        # only the first half comes back written, at num_samples*2 all of it does, and
        # the arrival rate halves from 4886/s to 2442/s = exactly 4096 samples/channel
        # at 10 Msps. See CLAUDE.md "sync_rx in RX_X2 delivers HALF the samples".
        req = num_samples * 2
        try:
            while not self._rx_stop.is_set():
                self.device.sync_rx(buf, req)
                iq = np.frombuffer(buf, dtype=np.int16).copy()
                # Deinterleave: [I1, Q1, I2, Q2, I1, Q1, I2, Q2, ...]
                rx1 = np.empty(num_samples * 2, dtype=np.int16)
                rx2 = np.empty(num_samples * 2, dtype=np.int16)
                rx1[0::2] = iq[0::4]  # RX1 I
                rx1[1::2] = iq[1::4]  # RX1 Q
                rx2[0::2] = iq[2::4]  # RX2 I
                rx2[1::2] = iq[3::4]  # RX2 Q
                callback(rx1, rx2)
        except Exception as e:
            print(f"[bladerf] RX dual error: {e}")
        finally:
            try:
                self.device.enable_module(bladerf.CHANNEL_RX(0), False)
                self.device.enable_module(bladerf.CHANNEL_RX(1), False)
            except Exception:
                pass
            self.rx_running = False

    def stop_rx_dual(self):
        if not self.rx_running:
            return
        self._rx_stop.set()
        if self._rx_thread:
            self._rx_thread.join(timeout=2)
            self._rx_thread = None
        self.rx_running = False
        self._dual_channel = False

    def get_status(self):
        return {
            'connected': self.device is not None,
            'serial': self.serial,
            'freq': self.center_freq,
            'sample_rate': self.sample_rate,
            'bandwidth': self.bandwidth,
            'tx_gain': self.tx_gain,
            'rx_gain': self.rx_gain,
            'tx_active': self.tx_running,
            'rx_active': self.rx_running,
            'waveform': self.waveform_type,
            'cw_offset': self.cw_offset,
            'tx_amplitude': self.tx_amplitude,
            'chirp_bw': self.chirp_bw,
            'chirp_duration': self.chirp_duration,
        }

    # ------------------------------------------------------------------
    # On-FPGA DSP path
    #
    # With this selected the FPGA divides RX1/RX2 per sample, averages N of
    # them per step, and writes one 64-bit word per step into a small FIFO.
    # The host reads DSP_FIFO_WORDS words and has h_cal directly -- no demod,
    # no accumulate, no divide.
    #
# It computes the SAME quantity the standard sweep does. rx.vhd accumulates
    # each channel into its own seq_adder and divides the two sums once per step
    # (dsp_chain_tb prints [acc1] sum, [acc2] sum, then one [div]), so the result
    # is sum1/sum2 == mean1/mean2. There is no E[X/Y] vs E[X]/E[Y] divergence --
    # an earlier version of this comment claimed there was.
    #
    # Selected by control-register bit 6, which the fabric does not decode
    # (bladerf_p.vhd unpack() covers 31:30 and 21:7).
    # ------------------------------------------------------------------

    DSP_PATH_BIT     = 6
    DSP_FRAC_BITS    = 14        # Q14: 16384 == 1.0
    DSP_WORD_BYTES   = 8         # 32-bit I + 32-bit Q
    DSP_SWEEP_WORDS  = 51        # rx.vhd DSP_FIFO_WORDS -- must match the FPGA

    # v12: control-register bits 24:22 / 27:25 select the chain's per-step
    # counts from these tables (rx.vhd DSP_FLUSH_TABLE / DSP_ACCUM_TABLE).
    # Entry 0 is the image's compile-time value. On v11 and earlier the bits
    # do nothing and read back as 0.
    DSP_FLUSH_SHIFT  = 22
    DSP_ACCUM_SHIFT  = 25
    DSP_FLUSH_TABLE  = (1088, 768, 512, 384, 256, 192, 128, 64)
    DSP_ACCUM_TABLE  = (2400, 2000, 1600, 1200, 1000, 800, 600, 400)

    def dsp_set_chain(self, flush_sel=0, accum_sel=0):
        """Select FLUSH_N / ACCUM_N on a v12+ image.

        Returns (flush_n, accum_n, supported). Between sweeps only: the FPGA
        samples the selection continuously and a change mid-step corrupts
        that step. Read-modify-write, like dsp_path_enable.
        """
        fs = int(flush_sel) & 7
        ac = int(accum_sel) & 7
        mask = (7 << self.DSP_FLUSH_SHIFT) | (7 << self.DSP_ACCUM_SHIFT)
        val = (self._gpio_read() & ~mask) | (fs << self.DSP_FLUSH_SHIFT) | (ac << self.DSP_ACCUM_SHIFT)
        self._gpio_write(val)
        back = self._gpio_read()
        got_fs = (back >> self.DSP_FLUSH_SHIFT) & 7
        got_ac = (back >> self.DSP_ACCUM_SHIFT) & 7
        supported = (got_fs == fs and got_ac == ac)
        if not supported:
            if fs or ac:
                print("[bladerf] DSP chain select not supported by this image "
                      "(wrote flush {} accum {}, read back {} {}): running the "
                      "compile-time counts {} + {}".format(
                          fs, ac, got_fs, got_ac,
                          self.DSP_FLUSH_TABLE[0], self.DSP_ACCUM_TABLE[0]))
            got_fs = got_ac = 0
        return self.DSP_FLUSH_TABLE[got_fs], self.DSP_ACCUM_TABLE[got_ac], supported

    # bladerf_metadata.flags: take whatever the FIFO has, do not schedule.
    _META_FLAG_RX_NOW = 1 << 31
    # Send as soon as there is room; the timestamp field is then ignored.
    # Only legal together with BURST_START -- see _tx_loop_dual.
    _META_FLAG_TX_NOW = 1 << 2
    _META_FLAG_TX_BURST_START = 1 << 0
    _META_FLAG_TX_BURST_END = 1 << 1

    def _gpio_read(self):
        """Read config_gpio through libbladeRF directly.

        The binding installed on the Pi has NO config_gpio accessor at all --
        not get_config_gpio, not config_gpio_read, not the property. Only the
        newer bindings in bladerf-src do. But bladerf_config_gpio_read/write
        are plain exported C functions declared in the cdef, so calling them
        through cffi works on every binding version, and is how the rest of
        this file already reaches libbladeRF (see _configure_channels_dual).
        """
        val = ffi.new('uint32_t *')
        ret = libbladeRF.bladerf_config_gpio_read(self.device.dev[0], val)
        if ret != 0:
            raise RuntimeError(
                "bladerf_config_gpio_read failed: {}".format(ret))
        return int(val[0])

    def _gpio_write(self, val):
        ret = libbladeRF.bladerf_config_gpio_write(self.device.dev[0],
                                                   int(val) & 0xFFFFFFFF)
        if ret != 0:
            raise RuntimeError(
                "bladerf_config_gpio_write failed: {}".format(ret))

    def dsp_path_enable(self, on=True):
        """Route the sample FIFO ports to the DSP result FIFO, or back.

        Not safe to flip mid-transfer: the multiplexer is combinational, so a
        change while the FX3 is reading swaps the source underneath it. Call
        with RX stopped.

        Read-modify-write: this register also carries the RX mux selection,
        packet/8-bit mode, the LEDs and the clock selects, so a bare mask would
        clear all of them.
        """
        val = self._gpio_read()
        if on:
            val |= (1 << self.DSP_PATH_BIT)
        else:
            val &= ~(1 << self.DSP_PATH_BIT)
        self._gpio_write(val)

    def start_rx_dsp(self):
        """Configure RX to receive DSP results instead of raw samples.

        SC16_Q11 -- plain sample mode, no metadata, no packet mode -- and NOT
        PACKET_META. The previous version used PACKET_META so that fx3_gpif
        would take the transfer length from a metadata header. That path was
        never seen to deliver on hardware, and it carries two dependencies
        this one does not:

          * the sample-format enum. PACKET_META is 3 in the canonical
            libbladeRF header and 2 in the bindings shipped on the Pi (see the
            note at the top of this file). SC16_Q11 is 0 in every version.
          * the dsp_meta FIFO and its header. Sample mode never consults
            metadata at all.

        How sample mode reaches the FX3 with only 102 DWORDs in the FIFO:
        fx3_gpif's burst condition (fx3_gpif.vhd:273) is

            unsigned(rx_fifo_full & rx_fifo_usedw) >= gpif_buf_size

        with the FULL flag concatenated as the MSB. When the DSP FIFO fills --
        one whole sweep -- that flag lifts the value to 2**14 + 102 = 16486,
        past the 2048 threshold, and fx3_gpif moves one full-size DMA buffer:
        the first 2*DSP_SWEEP_WORDS DWORDs are the sweep and the rest is the
        FIFO's last word repeated, because reads past empty are ignored. The
        FX3 firmware sees exactly the fixed-size buffer it always sees.
        Verified with the real fx3_gpif in simulation (rx_fx3_tb, SAMPLE).

        That repeated tail is also a fingerprint: dsp_read_sweep uses it to
        tell a DSP burst from a buffer of raw samples, which is what arrives
        if bit 6 is not in effect.

        Layout stays RX_X2 -- the FPGA still needs both AD9361 channels running
        to have a signal and a reference to divide. Only the FIFO read port is
        muxed; the channels themselves are untouched.
        """
        if self.rx_running:
            raise RuntimeError("stop RX before switching to the DSP path")

        # One ring buffer == one GPIF DMA transfer, so every sync_rx returns
        # exactly one burst: 2048 samples at SuperSpeed, 1024 at High Speed.
        # Bit 7 of config_gpio is usb_speed (1 = HS) and, unlike bit 6, it
        # reads back. buffer_size is the TOTAL interleaved sample count
        # (sync.c: bytes = buffer_size * bytes_per_sample), not per channel.
        gpio = self._gpio_read()
        self._dsp_buf_samples = 1024 if (gpio >> 7) & 1 else 2048

        self.device.sync_config(
            layout=ChannelLayout.RX_X2,
            fmt=fmt('SC16_Q11'),
            num_buffers=RX_RING_DEPTH,
            buffer_size=self._dsp_buf_samples,
            num_transfers=8,
            stream_timeout=3500
        )
        self.device.enable_module(bladerf.CHANNEL_RX(0), True)
        self.device.enable_module(bladerf.CHANNEL_RX(1), True)

        # Bit 6 goes LAST. libbladeRF's format config does its own
        # read-modify-write of config_gpio, and bit 6 reads back as 0 (it has
        # no field in bladerf_p.vhd's unpack(), which is exactly why it was
        # free to use) -- so any libbladeRF write after this would clear it.
        # Do not try to read it back to confirm; dsp_read_sweep confirms the
        # DSP path functionally instead.
        self.dsp_path_enable(True)
        gpio = self._gpio_read()
        print("[bladerf] DSP result path selected, sample mode "
              "(config_gpio=0x{:08x}; bit {} is write-only and reads 0; "
              "{} samples per transfer)".format(
                  gpio, self.DSP_PATH_BIT, self._dsp_buf_samples))

        self.rx_running = True
        self._dual_channel = True

        # Between enable_module and dsp_path_enable the mux still pointed at
        # the stock FIFO, and in sample mode fx3_gpif streams raw samples from
        # it continuously -- so the ring now holds some buffers of raw IQ.
        # Drain them: with bit 6 set and no sweep running nothing else
        # arrives, so the drain ends on the first timeout.
        self._dsp_drain_ring()

    def _dsp_drain_ring(self, timeout_ms=150, limit=RX_RING_DEPTH + 8):
        """Discard whatever is queued in the RX ring, until a read times out."""
        nsamp = self._dsp_buf_samples
        buf = bytearray(nsamp * 4)
        drained = 0
        for i in range(limit):
            t0 = time.monotonic()
            try:
                self.device.sync_rx(buf, nsamp, timeout_ms=timeout_ms)
                drained += 1
            except Exception as exc:
                # Ring empty, which is the goal. libbladeRF reports the
                # timed-out wait as -1 (see dsp_read_sweep), so -1 after
                # ~timeout_ms is the normal end; -1 within a few ms on the
                # first call is the sync-worker STARTUP race, retried.
                code = exc.args[0] if getattr(exc, 'args', None) else None
                waited_ms = (time.monotonic() - t0) * 1000.0
                if code == -1 and waited_ms < 50.0 and drained == 0 and i < 3:
                    time.sleep(0.05)
                    continue
                break
        if drained:
            print("[bladerf] DSP: drained {} stale raw buffer(s) from the RX "
                  "ring".format(drained))

    def stop_rx_dsp(self):
        if not self.rx_running:
            return
        try:
            self.dsp_path_enable(False)
        except Exception:
            pass
        try:
            self.device.enable_module(bladerf.CHANNEL_RX(0), False)
            self.device.enable_module(bladerf.CHANNEL_RX(1), False)
        except Exception:
            pass
        self.rx_running = False
        self._dual_channel = False

    def dsp_read_sweep(self, num_steps=None, timeout_s=2.0):
        """Read one sweep of per-step ratios as complex64, or None.

        One sync_rx of exactly one GPIF transfer (see start_rx_dsp). The sweep
        is the first 2*num_steps DWORDs; everything after is the FIFO's last
        word repeated, because fx3_gpif keeps clocking reads past empty and
        the FIFO ignores them.

        THE TAIL IS THE CHECK. A buffer of raw IQ -- which is exactly what
        arrives when bit 6 is not in effect, since sample mode then streams the
        stock FIFO -- never has a constant tail. So a non-constant tail means
        the DSP path is not selected, and that is reported by name rather than
        returned as a plausible-looking sweep of garbage. On such a buffer
        bit 6 is re-asserted once and the read retried, in case a libbladeRF
        read-modify-write cleared it.
        """
        if num_steps is None:
            num_steps = self.DSP_SWEEP_WORDS
        want_dwords = 2 * num_steps
        nsamp = getattr(self, '_dsp_buf_samples', 2048)
        buf = bytearray(nsamp * 4)

        raw_seen = 0
        timeout_ms = int(timeout_s * 1000)
        t_start = time.monotonic()
        # Bounded by the ring depth: raw buffers are consumed one per
        # iteration, so this can never spin on a stale backlog, and a genuine
        # timeout ends it immediately.
        for attempt in range(RX_RING_DEPTH + 8):
            # sync_rx raises on error and returns None -- there is no count to
            # check in sample mode; a return means the whole buffer was filled.
            #
            # A TIMEOUT ARRIVES AS -1, NOT -6. libbladeRF's thread.h
            # posix_cond_timedwait() returns -1 on ETIMEDOUT, but
            # sync.c wait_for_buffer() compares against THREAD_TIMEOUT
            # (= ETIMEDOUT), never matches, and reports BLADERF_ERR_UNEXPECTED.
            # So "-1 after >= timeout_ms" means no buffer came, full stop. The
            # same -1 inside a few ms is the sync-worker STARTUP race straight
            # after sync_config (SYNC_STATE_CHECK_WORKER); only that is retried.
            t0 = time.monotonic()
            try:
                self.device.sync_rx(buf, nsamp, timeout_ms=timeout_ms)
            except Exception as exc:
                code = exc.args[0] if getattr(exc, 'args', None) else '?'
                waited_ms = (time.monotonic() - t0) * 1000.0
                if code == -1 and waited_ms < 50.0 and attempt < 3:
                    time.sleep(0.05)
                    continue
                if code == -1 and waited_ms >= 0.9 * timeout_ms:
                    print("[bladerf] DSP read: no burst within {:.0f} ms of "
                          "EXEC{} -- the DSP FIFO did not reach {} results "
                          "this sweep (a step's accumulation was cut short, "
                          "or a restart was missed)".format(
                              (time.monotonic() - t_start) * 1000.0,
                              " after {} raw buffer(s)".format(raw_seen)
                              if raw_seen else "", num_steps))
                    return None
                print("[bladerf] DSP sweep read failed: {} ({}, code {}) "
                      "after {:.0f} ms requesting {} samples{}".format(
                          exc, type(exc).__name__, code, waited_ms, nsamp,
                          " after {} raw buffer(s)".format(raw_seen)
                          if raw_seen else ""))
                return None

            raw = np.frombuffer(bytes(buf), dtype='<i4')
            tail = raw[want_dwords:]
            if tail.size and np.all(tail == tail[0]):
                print("[bladerf] DSP burst {:.0f} ms after EXEC{}".format(
                    (time.monotonic() - t_start) * 1000.0,
                    " ({} raw buffer(s) first)".format(raw_seen)
                    if raw_seen else ""))
                payload = raw[:want_dwords]
                scale = float(1 << self.DSP_FRAC_BITS)
                return ((payload[0::2].astype(np.float32) / scale)
                        + 1j * (payload[1::2].astype(np.float32) / scale)
                        ).astype(np.complex64)

            # Not a DSP burst: raw samples, captured while the FIFO mux was on
            # the stock path. A handful are EXPECTED after every
            # start_rx_dsp() / dsp_resync() -- the stock FIFO streams for the
            # milliseconds between enable_module and bit 6 taking effect --
            # so read through them silently; a genuine burst may be queued
            # right behind. Only a whole ring of them means bit 6 is not in
            # effect at all, and that is diagnosed below.
            raw_seen += 1

        print("[bladerf] DSP read: {} consecutive raw buffers and no DSP burst "
              "-- dsp_path_en (bit {}) is not taking effect on the FPGA "
              "(check_bit6.py tells whether the write lands)".format(
                  raw_seen, self.DSP_PATH_BIT))
        return None

    def dsp_resync(self):
        """Realign the FPGA's DSP FIFO with the next sweep, keeping the stream.

        v11 clears the DSP result FIFO, its sweep counter and the FIFO gate
        whenever dsp_path_en is LOW (rx.vhd), so dropping bit 6 and raising it
        again throws away a partial sweep -- the leftover of a step whose
        accumulation was cut short -- without touching the RX stream. Before
        this the only way to clear it was stop_rx_dsp()/start_rx_dsp(): a
        sync_config, two enable_module calls and a ring drain, about a second.

        On v9/v10 the toggle is harmless but clears nothing; the partial sweep
        then stays and the next burst spans two sweeps.

        Raw buffers land in the ring while bit 6 is low; dsp_read_sweep reads
        through them.
        """
        self.dsp_path_enable(False)
        self.dsp_path_enable(True)
