"""WebSocket server for bladeRF SDR control and IQ streaming (port 9003)."""

import asyncio
import json
import signal
import sys
import numpy as np
import websockets

from bladerf_driver import BladeRFDriver
from sfcw_engine import SFCWEngine

SCALE = 2047
PORT = 9003
VIS_FPS = 25
FFT_SIZE = 16384
VIS_SAMPLES = 512


class SDRServer:
    def __init__(self):
        self.driver = BladeRFDriver()
        self.sfcw = SFCWEngine(self.driver)
        self.clients = set()
        self.rx_queue = asyncio.Queue(maxsize=4)
        self.sfcw_queue = asyncio.Queue(maxsize=8)
        self._broadcast_task = None
        self._sfcw_broadcast_task = None
        # The event loop, captured in start(). Both queues are fed from FOREIGN
        # THREADS -- _sfcw_callback from the SFCW sweep thread and _rx_callback
        # from the driver's RX thread -- and asyncio.Queue is NOT thread-safe.
        # See _sfcw_callback for what that cost.
        self._loop = None

    async def start(self):
        try:
            self.driver.open()
        except Exception as e:
            print(f"[sdr] ERROR: Could not open bladeRF device: {e}")
            print("[sdr] Check that the bladeRF is connected and drivers are installed.")
            sys.exit(1)

        print(f"[sdr] Device: {self.driver.serial}")
        self.sfcw._ensure_master_quick_tune_table()
        print(f"[sdr] SFCW master quick_tune table ready")
        print(f"[sdr] Starting WebSocket server on port {PORT}")
        self._loop = asyncio.get_running_loop()
        self._broadcast_task = asyncio.create_task(self._broadcast_loop())
        self._sfcw_broadcast_task = asyncio.create_task(self._sfcw_broadcast_loop())

        # Shut the device down properly on SIGTERM/SIGINT. Without this the
        # process died with TX and RX still enabled and USB transfers in flight,
        # so bladerf_close() never ran and the FPGA kept DMA-ing into an endpoint
        # that had gone away. That is what left the board wedged -- it still
        # enumerates, but every open fails "No devices available" and even
        # libbladeRF's own USB reset on open does not clear it (recovery needed
        # `usbreset`, or a replug). start.py terminates this process with SIGTERM
        # on every restart, so the damage accumulated once per restart. See
        # CLAUDE.md "repeated sfcw_start/stop degrades the bladeRF".
        stop = asyncio.Event()
        loop = asyncio.get_running_loop()
        for sig in (signal.SIGINT, signal.SIGTERM):
            try:
                loop.add_signal_handler(sig, stop.set)
            except NotImplementedError:
                pass

        async with websockets.serve(self._handler, "0.0.0.0", PORT):
            await stop.wait()

        self._shutdown()

    def _shutdown(self):
        print("[sdr] shutting down: stopping streams and closing device")
        try:
            if self.sfcw.running or self.sfcw._warm:
                self.sfcw.stop()
        except Exception as e:
            print(f"[sdr] sweep stop during shutdown: {e}")
        for fn in (self.driver.stop_rx_dual, self.driver.stop_tx_dual,
                   self.driver.close):
            try:
                fn()
            except Exception as e:
                print(f"[sdr] {fn.__name__} during shutdown: {e}")
        print("[sdr] device closed")

    async def _handler(self, ws):
        self.clients.add(ws)
        try:
            await ws.send(json.dumps({'type': 'status', **self.driver.get_status()}))
            await ws.send(json.dumps({'type': 'sfcw_status', **self._get_sfcw_status()}))
            async for msg in ws:
                await self._dispatch(ws, json.loads(msg))
        except websockets.ConnectionClosed:
            pass
        finally:
            self.clients.discard(ws)

    async def _dispatch(self, ws, cmd):
        action = cmd.get('cmd')
        try:
            if action == 'start_tx':
                if self.sfcw._warm:
                    self.sfcw.cool_down()
                self.driver._configure_channels_dual()
                self.driver.start_tx_dual()
                self.driver.reapply_dual_gains()
                await self._broadcast_status()
            elif action == 'stop_tx':
                self.driver.stop_tx_dual()
                await self._broadcast_status()
            elif action == 'start_rx':
                if self.sfcw._warm:
                    self.sfcw.cool_down()
                self.driver._configure_channels_dual()
                self.driver.start_rx_dual(self._rx_callback, num_samples=16384)
                self.driver.reapply_dual_gains()
                await self._broadcast_status()
            elif action == 'stop_rx':
                self.driver.stop_rx_dual()
                await self._broadcast_status()
            elif action == 'set_freq':
                self.driver.set_frequency(float(cmd['value']) * 1e6)
                await self._broadcast_status()
            elif action == 'set_tx_gain':
                self.driver.set_tx_gain(int(cmd['value']))
                await self._broadcast_status()
            elif action == 'set_rx_gain':
                self.driver.set_rx_gain(int(cmd['value']))
                await self._broadcast_status()
            elif action == 'set_sample_rate':
                self.driver.set_sample_rate(float(cmd['value']) * 1e6)
                await self._broadcast_status()
            elif action == 'set_waveform':
                params = {}
                if 'offset_khz' in cmd:
                    params['offset'] = float(cmd['offset_khz']) * 1e3
                if 'amplitude' in cmd:
                    params['amplitude'] = float(cmd['amplitude'])
                if 'chirp_bw_khz' in cmd:
                    params['chirp_bw'] = float(cmd['chirp_bw_khz']) * 1e3
                if 'chirp_duration_ms' in cmd:
                    params['chirp_duration'] = float(cmd['chirp_duration_ms']) / 1000
                self.driver.set_waveform(cmd.get('type', 'cw'), **params)
                await self._broadcast_status()
            elif action == 'get_status':
                await ws.send(json.dumps({'type': 'status', **self.driver.get_status()}))

            # SFCW commands
            elif action == 'sfcw_set_params':
                params = {}
                if 'start_freq_mhz' in cmd:
                    params['start_freq'] = float(cmd['start_freq_mhz']) * 1e6
                if 'stop_freq_mhz' in cmd:
                    params['stop_freq'] = float(cmd['stop_freq_mhz']) * 1e6
                if 'step_size_mhz' in cmd:
                    params['step_size'] = float(cmd['step_size_mhz']) * 1e6
                if 'num_buffers' in cmd:
                    params['num_buffers'] = int(cmd['num_buffers'])
                if 'settle_count' in cmd:
                    params['settle_count'] = int(cmd['settle_count'])
                if 'tx1_gain' in cmd:
                    params['tx1_gain'] = int(cmd['tx1_gain'])
                if 'rx1_gain' in cmd:
                    params['rx1_gain'] = int(cmd['rx1_gain'])
                if 'tx2_gain' in cmd:
                    params['tx2_gain'] = int(cmd['tx2_gain'])
                if 'rx2_gain' in cmd:
                    params['rx2_gain'] = int(cmd['rx2_gain'])
                if 'range_offset' in cmd:
                    params['range_offset'] = float(cmd['range_offset'])
                if 'bscan_avg_count' in cmd:
                    params['bscan_avg_count'] = int(cmd['bscan_avg_count'])
                if 'bscan_primer' in cmd:
                    params['bscan_primer'] = bool(cmd['bscan_primer'])
                # NIOS autonomous sweep controls (fpga_branch port, 2026-09-07).
                # The GUI does not send these yet; benchmark/tool clients do.
                if 'sweep_mode' in cmd:
                    params['sweep_mode'] = str(cmd['sweep_mode'])
                if 'nios_dwell' in cmd:
                    params['nios_dwell'] = int(cmd['nios_dwell'])
                if 'nios_settle' in cmd:
                    params['nios_settle'] = int(cmd['nios_settle'])
                if 'nios_pipeline' in cmd:
                    params['nios_pipeline'] = bool(cmd['nios_pipeline'])
                self.sfcw.set_params(**params)
                await self._broadcast_sfcw_status()

            elif action == 'sfcw_start':
                if self.sfcw._warm:
                    self.sfcw.cool_down()
                if self.driver.tx_running:
                    self.driver.stop_tx()
                if self.driver.rx_running:
                    self.driver.stop_rx()
                await self._broadcast_status()
                self.sfcw.start(self._sfcw_callback)
                await self._broadcast_sfcw_status()

            elif action == 'sfcw_stop':
                self.sfcw.stop()
                await self._broadcast_sfcw_status()

            elif action == 'sfcw_coherence_test':
                if self.sfcw.running:
                    await ws.send(json.dumps({'type': 'error', 'message': 'Stop sweep before running coherence test'}))
                else:
                    # Optional 'num_sweeps' (default SfcwEngine.COHERENCE_SWEEPS = 100).
                    n = cmd.get('num_sweeps')
                    self.sfcw.run_coherence_test(self._sfcw_callback,
                                                 num_sweeps=int(n) if n else None)
                    await self._broadcast_sfcw_status()

            elif action == 'sfcw_get_status':
                await ws.send(json.dumps({'type': 'sfcw_status', **self._get_sfcw_status()}))

            elif action == 'sweep_capture':
                if self.sfcw.running and not self.sfcw._warm:
                    self.sfcw.stop()
                if not self.sfcw._warm:
                    self._stop_all_streams()
                    await self._broadcast_status()
                self.sfcw.run_single(self._sfcw_callback)
                await self._broadcast_sfcw_status()

            elif action == 'bscan_warm_up':
                if self.sfcw._warm:
                    pass
                else:
                    if self.sfcw.running:
                        self.sfcw.stop()
                    self._stop_all_streams()
                    await self._broadcast_status()
                    self.sfcw.warm_up()
                await self._broadcast_sfcw_status()

            elif action == 'bscan_cool_down':
                self.sfcw.cool_down()
                await self._broadcast_sfcw_status()

            elif action == 'sweep_capture_bg':
                if self.sfcw.running and not self.sfcw._warm:
                    self.sfcw.stop()
                if not self.sfcw._warm:
                    self._stop_all_streams()
                    await self._broadcast_status()
                self.sfcw.run_single(self._sfcw_callback)
                await self._broadcast_sfcw_status()

            elif action == 'device_reset':
                if self.sfcw.running:
                    self.sfcw.stop()
                self._stop_all_streams()
                self.driver.reset()
                self.sfcw.invalidate_quick_tune_table()
                await self._broadcast_status()

        except Exception as e:
            await ws.send(json.dumps({'type': 'error', 'message': str(e)}))

    def _stop_all_streams(self):
        if self.driver.tx_running:
            self.driver.stop_tx()
        if self.driver.rx_running:
            self.driver.stop_rx()

    def _get_sfcw_status(self):
        params = self.sfcw.get_params()
        params['running'] = self.sfcw.running
        return params

    @staticmethod
    def _offer(queue, item):
        """put_nowait with drop-oldest. MUST run on the event loop thread."""
        try:
            queue.put_nowait(item)
        except asyncio.QueueFull:
            try:
                queue.get_nowait()
            except asyncio.QueueEmpty:
                pass
            try:
                queue.put_nowait(item)
            except asyncio.QueueFull:
                pass

    def _post(self, queue, item):
        """Hand an item from a worker THREAD to the event loop.

        This is `call_soon_threadsafe`, not a bare `put_nowait`, and that is the
        whole point. `asyncio.Queue` is not thread-safe, and the part that bites
        is not a corrupted queue -- it is that a put from a foreign thread never
        WAKES the event loop. The waiting `get()` future is only completed via
        the loop's own `call_soon`, so a producer thread's put leaves the loop
        asleep in its selector until something else happens to wake it.
        `_sfcw_broadcast_loop` waits with `wait_for(..., timeout=0.1)`, so what
        actually woke it was that timeout: the broadcast ran at ~10 Hz no matter
        how fast the radar swept.

        Measured 2026-09-06, with the engine at 15.57 Hz: a client that did no
        JSON parsing at all still received only 10.90 sweeps/s -- 30% lost, and
        42.7% of its inter-sweep deltas were an exact 2x multiple (the signature
        of a dropped sweep), while the 8-deep queue silently discarded the rest.
        To an operator that is "mostly 15 Hz, randomly dropping to ~7 Hz for a
        bit" -- the rolling median halves whenever a run of sweeps is dropped.
        It only became obvious after the RX buffer change took the sweep from
        11.7 Hz to 15.3 Hz: at 11.7 Hz the engine was close enough to the 10 Hz
        poll for the loss to stay small.

        call_soon_threadsafe writes to the loop's self-pipe, so the loop wakes
        immediately and the queue is served at the rate the radar produces.
        """
        loop = self._loop
        if loop is None:            # pre-start callback; nothing to broadcast to yet
            return
        try:
            loop.call_soon_threadsafe(self._offer, queue, item)
        except RuntimeError:        # loop already closed during shutdown
            pass

    def _sfcw_callback(self, data):
        self._post(self.sfcw_queue, data)

    async def _sfcw_broadcast_loop(self):
        while True:
            try:
                data = await asyncio.wait_for(self.sfcw_queue.get(), timeout=0.1)
            except asyncio.TimeoutError:
                await asyncio.sleep(0.01)
                continue

            if not self.clients:
                continue

            if isinstance(data, dict) and 'error' in data:
                msg = json.dumps({'type': 'sfcw_error', 'message': data['error']})
            elif isinstance(data, dict) and data.get('type') == 'coherence_result':
                msg = json.dumps(data)
            elif isinstance(data, dict) and data.get('type') == 'progress':
                msg = json.dumps({'type': 'sfcw_progress', 'step': data['step'], 'total': data['total'], 'freq_mhz': round(data['freq_mhz'], 2)})
            elif isinstance(data, dict) and data.get('type') == 'range_profile':
                result_msg = {
                    'type': 'sfcw_result',
                    # np.round(...).tolist(), NOT [round(x, n) for x in ...].
                    # Byte-identical output, 2.778 -> 0.023 ms/sweep (122x). The
                    # comprehensions were pure Python over ~512 elements and so held
                    # the GIL for ~2.8 ms in one block -- about 7 RX buffer periods --
                    # stalling _rx_loop_dual exactly while the next sweep was stepping.
                    # That backlog is what corrupted a step: the sweep then drained
                    # pre-retune buffers holding the PREVIOUS frequency's IQ. Measured
                    # 2026-09-05: settle=1 is 0/299 sweeps contention-free but 18/399
                    # through the server, so this cost ~42 ms of sweep time in the
                    # settle margin needed to survive it. Keep this vectorised.
                    'distances': np.round(data['distances'], 4).tolist(),
                    'magnitudes': np.round(data['magnitudes'], 2).tolist(),
                    'h_cal_real': data.get('h_cal_real', []),
                    'h_cal_imag': data.get('h_cal_imag', []),
                    'range_resolution': round(data['range_resolution'], 4),
                    'unambiguous_range': round(data['unambiguous_range'], 4),
                    'displayed_range_max': round(data['displayed_range_max'], 4),
                    'num_steps': data['num_steps'],
                    'step_size': data.get('step_size', 0),
                    'range_offset': data.get('range_offset', 0),
                    'timestamp': data['timestamp'],
                }
                if 'phase_coherence' in data:
                    result_msg['phase_coherence'] = data['phase_coherence']
                if 'sweep_core' in data:
                    result_msg['sweep_core'] = data['sweep_core']
                if 'nios_diag' in data:
                    result_msg['nios_diag'] = data['nios_diag']
                msg = json.dumps(result_msg)
            else:
                continue

            dead = set()
            for client in self.clients:
                try:
                    await client.send(msg)
                except websockets.ConnectionClosed:
                    dead.add(client)
            self.clients -= dead

            if not self.sfcw.running:
                await self._broadcast_sfcw_status()

    async def _broadcast_sfcw_status(self):
        msg = json.dumps({'type': 'sfcw_status', **self._get_sfcw_status()})
        dead = set()
        for client in self.clients:
            try:
                await client.send(msg)
            except websockets.ConnectionClosed:
                dead.add(client)
        self.clients -= dead

    def _rx_callback(self, rx1_iq, rx2_iq):
        # Same foreign-thread handoff as _sfcw_callback -- this one runs on the
        # driver's RX thread. Same bug, same fix; see _post.
        self._post(self.rx_queue, (rx1_iq, rx2_iq))

    def _channel_vis(self, iq):
        """Time-domain preview + FFT magnitudes for one channel's raw interleaved IQ."""
        i_raw = iq[0::2].astype(np.float64)
        q_raw = iq[1::2].astype(np.float64)
        num = len(i_raw)

        vis_len = min(VIS_SAMPLES, num)
        i_vis = i_raw[:vis_len] / SCALE
        q_vis = q_raw[:vis_len] / SCALE

        fft_len = min(num, FFT_SIZE)
        complex_iq = (i_raw[:fft_len] + 1j * q_raw[:fft_len]) / SCALE
        window = np.hanning(fft_len)
        spectrum = np.fft.fftshift(np.fft.fft(complex_iq * window))
        magnitudes = 20 * np.log10(np.abs(spectrum) / fft_len + 1e-12)
        n_bins = 512
        if len(magnitudes) > n_bins:
            trim = len(magnitudes) - len(magnitudes) % n_bins
            magnitudes = magnitudes[:trim].reshape(n_bins, -1).max(axis=1)

        return (
            {'i': [round(v, 4) for v in i_vis.tolist()], 'q': [round(v, 4) for v in q_vis.tolist()]},
            {'magnitudes': [round(v, 1) for v in magnitudes.tolist()]},
        )

    async def _broadcast_loop(self):
        interval = 1.0 / VIS_FPS
        while True:
            try:
                rx1_iq, rx2_iq = await asyncio.wait_for(self.rx_queue.get(), timeout=0.1)
            except asyncio.TimeoutError:
                await asyncio.sleep(0.01)
                continue

            if not self.clients:
                await asyncio.sleep(interval)
                continue

            antenna_data, antenna_fft = self._channel_vis(rx1_iq)
            reference_data, reference_fft = self._channel_vis(rx2_iq)

            rx_msg = json.dumps({
                'type': 'rx_data',
                'antenna': antenna_data,
                'reference': reference_data,
            })

            fft_msg = json.dumps({
                'type': 'rx_fft',
                'antenna': antenna_fft,
                'reference': reference_fft,
                'freq_span': self.driver.sample_rate,
            })

            dead = set()
            for client in self.clients:
                try:
                    await client.send(rx_msg)
                    await client.send(fft_msg)
                except websockets.ConnectionClosed:
                    dead.add(client)
            self.clients -= dead

            await asyncio.sleep(interval)

    async def _broadcast_status(self):
        msg = json.dumps({'type': 'status', **self.driver.get_status()})
        dead = set()
        for client in self.clients:
            try:
                await client.send(msg)
            except websockets.ConnectionClosed:
                dead.add(client)
        self.clients -= dead


if __name__ == '__main__':
    server = SDRServer()
    asyncio.run(server.start())
