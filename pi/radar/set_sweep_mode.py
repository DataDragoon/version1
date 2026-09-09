#!/usr/bin/env python3
"""Switch the SFCW engine between sweep cores, with the required restart.

    python3 radar/set_sweep_mode.py dsp        # FPGA DSP result path (v7+)
    python3 radar/set_sweep_mode.py nios       # raw capture, NIOS-stepped
    python3 radar/set_sweep_mode.py standard   # raw capture, host-stepped
    python3 radar/set_sweep_mode.py            # just report the current mode

sweep_mode is only reachable over the sdr_server WebSocket -- there is no
config entry and the GUI does not send it -- which is why this exists.

The sweep is stopped and restarted around the change on purpose. The sample
format is fixed when sync_config runs, and 'dsp' uses PACKET_META where the
others use SC16_Q11, so crossing into or out of 'dsp' on a live stream cannot
work; the engine prints a notice and keeps the old format.

Watch the server console afterwards. On success 'dsp' prints

    [bladerf] DSP result path selected (config_gpio=0x...)

and each sweep result carries sweep_core='dsp'. 'fallback' means the DSP read
failed for that sweep and a standard sweep was substituted -- the reason is
printed once, then summarised.
"""

import asyncio
import json
import sys

try:
    import websockets
except ImportError:
    sys.exit("pip install websockets")

URI = "ws://127.0.0.1:9003"
MODES = ('dsp', 'nios', 'standard')


async def run(mode):
    async with websockets.connect(URI, max_size=None) as ws:

        async def send(payload):
            await ws.send(json.dumps(payload))

        if mode is None:
            await send({'cmd': 'sfcw_get_status'})
            # Status may arrive behind queued broadcasts; take the first frame
            # that actually carries the field.
            for _ in range(20):
                msg = json.loads(await asyncio.wait_for(ws.recv(), timeout=5))
                for blob in (msg, msg.get('data') or {}):
                    if isinstance(blob, dict) and 'sweep_mode' in blob:
                        print("sweep_mode =", blob['sweep_mode'])
                        return 0
            print("could not read sweep_mode from the status reply")
            return 1

        print("stopping sweep...")
        await send({'cmd': 'sfcw_stop'})
        await asyncio.sleep(1.0)

        print("setting sweep_mode = {}".format(mode))
        await send({'cmd': 'sfcw_set_params', 'sweep_mode': mode})
        await asyncio.sleep(0.5)

        print("starting sweep...")
        await send({'cmd': 'sfcw_start'})
        await asyncio.sleep(0.5)
        print("done -- watch the server console for the result")
        return 0


def main():
    mode = sys.argv[1].lower() if len(sys.argv) > 1 else None
    if mode is not None and mode not in MODES:
        sys.exit("mode must be one of: {}".format(', '.join(MODES)))
    try:
        return asyncio.run(run(mode))
    except Exception as e:
        sys.exit("failed to talk to {}: {}".format(URI, e))


if __name__ == '__main__':
    sys.exit(main())
