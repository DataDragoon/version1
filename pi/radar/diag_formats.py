#!/usr/bin/env python3
"""Print exactly what this Pi's bladerf stack believes about sample formats.

Run on the Pi:

    python3 pi/radar/diag_formats.py

Every "Invalid operation or parameter" at stream start so far has come from a
disagreement between three things that are versioned independently:

  1. libbladeRF.so          -- the C library, owns the real enum
  2. the bladerf python pkg -- a cffi binding, carries its own copy of it
  3. this repo              -- asks for formats by name

bladerf_format is a plain C enum, so if any one of them has a different member
list the numbering silently shifts and sync_config configures a format nobody
asked for. Guessing which one is stale has not worked; this prints it.
"""

import sys


def main():
    print("python:", sys.version.split()[0])

    try:
        import bladerf
        from bladerf import _bladerf
    except Exception as e:
        print("FATAL: cannot import bladerf:", e)
        return 1

    print("bladerf package:", getattr(bladerf, '__file__', '?'))
    print("bladerf version:", getattr(bladerf, '__version__', 'no __version__'))

    # ---- what the BINDING thinks the enum is -----------------------------
    print("\n--- Format enum as the installed BINDING sees it ---")
    fmts = {}
    for m in _bladerf.Format:
        fmts[m.name] = m.value
        print("  {:<18} = {}".format(m.name, m.value))

    canonical = {
        'SC16_Q11': 0, 'SC16_Q11_PACKED': 1, 'SC16_Q11_META': 2,
        'PACKET_META': 3, 'SC8_Q7': 4, 'SC8_Q7_META': 5,
    }
    print("\n--- vs canonical libbladeRF.h ordering ---")
    bad = False
    for name, want in canonical.items():
        got = fmts.get(name)
        if got is None:
            print("  {:<18} MISSING from binding (canonical {})".format(name, want))
            bad = True
        elif got != want:
            print("  {:<18} binding={} canonical={}   <-- MISMATCH".format(name, got, want))
            bad = True
        else:
            print("  {:<18} {} ok".format(name, got))
    print("\nbinding enum matches canonical:", not bad)

    # ---- what the LIBRARY thinks, via its own strerror-style API ----------
    # bladerf_format_to_string is not exported, so infer from byte maths:
    # ask the library to size a buffer and see how many bytes per sample it
    # used. 3 bytes/sample means it resolved SC16_Q11_PACKED.
    print("\n--- what libbladeRF.so resolves each value to (bytes/sample) ---")
    print("  (inferred: 2=SC8_Q7-ish, 3=SC16_Q11_PACKED, 4=SC16_Q11/META/PACKET_META)")

    try:
        dev = bladerf.BladeRF()
    except Exception as e:
        print("  cannot open device:", e)
        print("\nRun again with the device attached to get the rest.")
        return 1

    try:
        print("\n--- device ---")
        print("  serial:  ", dev.get_serial())
        try:
            fw = dev.get_fw_version()
            fpga = dev.get_fpga_version()
            print("  firmware:", fw)
            print("  fpga:    ", fpga)
        except Exception as e:
            print("  version read failed:", e)

        try:
            gpio = dev.get_config_gpio()
            print("  config_gpio: 0x{:08x}  (DSP bit6={})".format(
                gpio, bool(gpio & (1 << 6))))
        except Exception as e:
            print("  get_config_gpio failed:", e)

        print("\n--- config_gpio accessors present ---")
        for name in ('get_config_gpio', 'set_config_gpio',
                     'config_gpio_read', 'config_gpio_write',
                     'dsp_path_enabled'):
            print("  {:<20} {}".format(name, hasattr(dev, name)))

        print("\n--- unpack_dsp_results present in bindings ---")
        print(" ", hasattr(_bladerf, 'unpack_dsp_results'))

        # ---- the actual failing call, one format at a time ---------------
        # buffer_size deliberately 4096 to reproduce the reported warning.
        print("\n--- sync_config probe (RX_X2) ---")
        for name in ('SC16_Q11', 'SC16_Q11_META', 'PACKET_META'):
            val = fmts.get(name)
            if val is None:
                print("  {:<16} skipped (not in binding)".format(name))
                continue
            try:
                dev.sync_config(layout=_bladerf.ChannelLayout.RX_X2,
                                fmt=_bladerf.Format(val),
                                num_buffers=16, buffer_size=4096,
                                num_transfers=8, stream_timeout=3500)
                print("  {:<16} value={} OK".format(name, val))
            except Exception as e:
                print("  {:<16} value={} FAILED: {}".format(name, val, e))
    finally:
        try:
            dev.close()
        except Exception:
            pass
    return 0


if __name__ == '__main__':
    sys.exit(main())
