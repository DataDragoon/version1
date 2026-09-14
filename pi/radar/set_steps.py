#!/usr/bin/env python3
"""Set the number of steps in the SFCW sweep, keeping start and stop.

    python3 radar/set_steps.py 61
    python3 radar/set_steps.py 151

Sends {"cmd": "sfcw_set_params", "num_steps": N} to sdr_server. The
engine turns N into the nearest legal step size for the current start and
stop (every step must be a multiple of 20 or 50 MHz so its quick-tune
profile exists), so the count you get may differ: over 2-5 GHz the
reachable counts are 151, 76, 61, 51, 31, 26, 21, 16, 13, 11, 7, 6.
The reply prints the step size and count that were actually set. On the
v15 image (fifo-256) any of them goes out as one burst; on v13 only 51.

If a sweep is running it picks the new grid up on the next sweep, after a
one-time re-prime (~124 ms). The GUI pushes its own step size on Start,
so a count set here lasts until the next Start from the panel.
"""
import asyncio
import json
import sys

try:
    import websockets
except ImportError:
    sys.exit("needs the 'websockets' package (it is already used by sdr_server)")

URL = "ws://127.0.0.1:9003"


async def main(n):
    async with websockets.connect(URL, max_size=1 << 24) as ws:
        # The server greets every new client with an sfcw_status BEFORE it
        # has seen any request. Drain that (and anything else queued) first,
        # or the greeting is mistaken for the reply and shows the old grid.
        try:
            while True:
                await asyncio.wait_for(ws.recv(), timeout=0.3)
        except asyncio.TimeoutError:
            pass
        # The key is "cmd": sdr_server dispatches on cmd.get('cmd').
        await ws.send(json.dumps({"cmd": "sfcw_set_params", "num_steps": n}))
        # The reply is the next sfcw_status; sweep results may be
        # interleaved if a sweep is running, so read until a status shows.
        for _ in range(50):
            msg = json.loads(await asyncio.wait_for(ws.recv(), timeout=2.0))
            if msg.get("type") == "sfcw_status":
                start = float(msg["start_freq"])
                stop = float(msg["stop_freq"])
                step = float(msg["step_size"])
                got = int((stop - start) // step) + 1 if step else 0
                print(f"server now reports: {start / 1e6:g}-{stop / 1e6:g} MHz, "
                      f"step {step / 1e6:g} MHz = {got} steps"
                      + ("" if got == n else f"  (asked {n}; nearest on the grid)"))
                return
        print("no sfcw_status seen; check ~/radar.log for the '[sfcw] num_steps' line")


if __name__ == "__main__":
    if len(sys.argv) != 2 or not sys.argv[1].isdigit():
        sys.exit(__doc__)
    try:
        asyncio.run(main(int(sys.argv[1])))
    except (OSError, asyncio.TimeoutError) as exc:
        sys.exit(f"failed to talk to {URL}: {exc} -- is sdr_server running?")
