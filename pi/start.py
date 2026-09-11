"""Single entry point for all Pi services."""

import subprocess
import sys
import os
import signal

base = os.path.dirname(os.path.abspath(__file__))
procs = []


def cleanup(sig=None, frame=None):
    print("\nShutting down...")
    for p in procs:
        p.terminate()
    for p in procs:
        try:
            p.wait(timeout=3)
        except subprocess.TimeoutExpired:
            p.kill()
    print("All services stopped.")
    sys.exit(0)


signal.signal(signal.SIGINT, cleanup)
signal.signal(signal.SIGTERM, cleanup)

# -u: the services' stdout is block-buffered whenever it is a pipe (tee, a
# log file, systemd), so their prints only appear 8 KB at a time -- the
# [bladerf]/[sfcw] diagnostics were invisible in /tmp/sdr.log on 2026-09-11.
services = [
    [sys.executable, '-u', os.path.join(base, 'sensors', 'stream.py')],
    [sys.executable, '-u', os.path.join(base, 'radar', 'sdr_server.py')],
    [sys.executable, '-u', os.path.join(base, 'rover', 'rover_server.py')],
]

print(f"Starting {len(services)} service(s)...")

for cmd in services:
    print(f"  → {os.path.basename(cmd[-1])}")
    procs.append(subprocess.Popen(cmd))

try:
    for p in procs:
        p.wait()
except KeyboardInterrupt:
    cleanup()
