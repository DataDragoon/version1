#!/usr/bin/env python3
"""Count what the SDR server actually sends, with no GUI in the way.

Connects to sdr_server's WebSocket exactly like the groundstation does and
prints, once a second, how many sfcw_result messages arrived and the sweep
rate implied by the Pi's own timestamps inside them. Run it on the Pi
(ws://127.0.0.1:9003) to measure what leaves the server, or on the laptop
(ws://<pi-ip>:9003) to measure what crosses the Wi-Fi link. If both agree
with the engine's timing line, the radar and the link are fine and any
difference on screen is the browser's drawing rate.

    python3 radar/ws_rate.py                 # Pi, local server
    python  pi/radar/ws_rate.py 192.168.1.20 # laptop, Pi at that address
"""
import asyncio
import json
import sys
import time

try:
    import websockets
except ImportError:
    sys.exit("needs the 'websockets' package:  pip install websockets")


async def main(host, seconds):
    url = f"ws://{host}:9003"
    print(f"connecting to {url}")
    async with websockets.connect(url, max_size=1 << 24, ping_interval=None) as ws:
        print("connected; counting sfcw_result messages (Ctrl+C to stop)")
        t_wall = time.monotonic()
        t_end = t_wall + seconds if seconds else None
        n = 0
        other = 0
        ts = []
        while True:
            try:
                raw = await asyncio.wait_for(ws.recv(), timeout=1.0)
            except asyncio.TimeoutError:
                raw = None
            if raw is not None:
                try:
                    msg = json.loads(raw)
                except json.JSONDecodeError:
                    continue
                if msg.get('type') == 'sfcw_result':
                    n += 1
                    ts.append(float(msg.get('timestamp', 0.0)))
                else:
                    other += 1
            now = time.monotonic()
            if now - t_wall >= 1.0:
                dt = now - t_wall
                if len(ts) >= 2:
                    d = sorted(b - a for a, b in zip(ts, ts[1:]) if b > a)
                    med = d[len(d) // 2] if d else 0.0
                    pi_rate = f"{1.0 / med:6.1f} Hz by Pi timestamps (median {1e3 * med:.1f} ms)"
                else:
                    pi_rate = "   n/a by Pi timestamps"
                print(f"{n / dt:6.1f} sweeps/s received   {pi_rate}   other msgs {other}")
                t_wall, n, other, ts = now, 0, 0, ts[-1:]
            if t_end and now >= t_end:
                break


if __name__ == '__main__':
    host = sys.argv[1] if len(sys.argv) > 1 else '127.0.0.1'
    seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 0
    try:
        asyncio.run(main(host, seconds))
    except KeyboardInterrupt:
        pass
