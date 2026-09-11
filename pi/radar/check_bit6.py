#!/usr/bin/env python3
"""Does config_gpio bit 6 (dsp_path_en) reach the FPGA, and does it read back?

    python3 radar/check_bit6.py

Run on the Pi with the services STOPPED (it needs the device to itself).
Nothing is streamed; it only pokes the config-GPIO register:

    1. prints the FPGA version and, where libbladeRF supports it, whether the
       running image came from HOST (bladeRF-cli -l / libbladeRF autoload) or
       FLASH (the board's own autoload)
    2. reads config_gpio
    3. writes it back with bit 12 set (LED bit -- reads back on EVERY image)
       and checks bit 12 returns: proves the write path host -> FX3 -> Nios ->
       PIO and the read path back
    4. writes it back with bit 6 set and checks bit 6 returns: on the v10
       image (readback of nios_gpo_slv(6) added 2026-09-11) it does; on v9 and
       earlier the readback has no bit 6 and it reads 0 whatever was written
    5. restores the original value

Verdict table:

    bit 12 does not read back      -> the write (or the read) is not reaching
                                      the Nios at all; host/USB/Nios problem
    bit 12 ok, bit 6 reads 0       -> the loaded image is older than v10
                                      (typically the board's flash image
                                      after a power cycle): load the current
                                      one, e.g.
                                      bladeRF-cli -l .../fpga/images/hostedxA9_niosIIf_sweep_dsp_v11_signaltap.rbf
    bit 12 ok, bit 6 reads 1       -> the write lands and v10+ is running; if
                                      the FX3 still delivers raw samples the
                                      fault is inside the FPGA between
                                      nios_gpo_slv(6) and the rx.vhd mux
"""
import sys

try:
    import bladerf
    from bladerf._bladerf import ffi, libbladeRF
except ImportError as e:
    sys.exit("python bladerf bindings not importable: {}".format(e))

BIT6 = 1 << 6
BIT12 = 1 << 12


def gpio_read(dev):
    val = ffi.new('uint32_t *')
    ret = libbladeRF.bladerf_config_gpio_read(dev, val)
    if ret != 0:
        raise RuntimeError("bladerf_config_gpio_read failed: {}".format(ret))
    return int(val[0])


def gpio_write(dev, val):
    ret = libbladeRF.bladerf_config_gpio_write(dev, int(val) & 0xFFFFFFFF)
    if ret != 0:
        raise RuntimeError("bladerf_config_gpio_write failed: {}".format(ret))


def fpga_info(dev):
    try:
        v = ffi.new('struct bladerf_version *')
        if libbladeRF.bladerf_fpga_version(dev, v) == 0:
            desc = ffi.string(v.describe).decode() if v.describe != ffi.NULL else ''
            print("FPGA version : {}.{}.{} {}".format(v.major, v.minor, v.patch, desc))
    except Exception as e:
        print("FPGA version : (not readable: {})".format(e))
    try:
        src = ffi.new('bladerf_fpga_source *')
        if libbladeRF.bladerf_get_fpga_source(dev, src) == 0:
            names = {0: 'UNKNOWN', 1: 'FLASH (board autoload)', 2: 'HOST (bladeRF-cli -l / libbladeRF)'}
            print("FPGA source  : {}".format(names.get(int(src[0]), int(src[0]))))
    except Exception:
        print("FPGA source  : (this libbladeRF/binding has no bladerf_get_fpga_source)")


def main():
    d = bladerf.BladeRF()
    dev = d.dev[0]
    try:
        fpga_info(dev)
        g0 = gpio_read(dev)
        print("config_gpio  : 0x{:08x}  (as found; bit 6 = {})".format(g0, (g0 >> 6) & 1))

        # --- bit 12: proves the round trip on any image -------------------
        gpio_write(dev, g0 | BIT12)
        g12 = gpio_read(dev)
        ok12 = bool(g12 & BIT12)
        print("write bit 12 : wrote 0x{:08x}, read 0x{:08x}  -> bit 12 {}".format(
            g0 | BIT12, g12, "reads back" if ok12 else "LOST"))

        # --- bit 6: the one that matters -----------------------------------
        gpio_write(dev, g0 | BIT6)
        g6 = gpio_read(dev)
        ok6 = bool(g6 & BIT6)
        print("write bit 6  : wrote 0x{:08x}, read 0x{:08x}  -> bit 6 {}".format(
            g0 | BIT6, g6, "reads back" if ok6 else "reads 0"))

        gpio_write(dev, g0)
        print("restored     : 0x{:08x}".format(gpio_read(dev)))

        print()
        if not ok12:
            print("VERDICT: the write/read round trip to the Nios is broken -- "
                  "not an FPGA-image question yet (host / USB / Nios).")
            return 2
        if not ok6:
            print("VERDICT: writes land, but this image has no bit-6 readback: "
                  "the running FPGA is older than v10. Load the current image:")
            print("  bladeRF-cli -l /home/sfr/vikram_fpga_dump/version1/fpga/images/"
                  "hostedxA9_niosIIf_sweep_dsp_v11_signaltap.rbf")
            print("then run this again; bit 6 must read back before the DSP "
                  "path can be judged.")
            return 1
        print("VERDICT: a v10-or-later image is running and bit 6 lands in the "
              "control register. (v10 and v11 both report 0.16.0; the version "
              "number cannot tell them apart -- the md5 of the file you loaded "
              "can: v10 8079e007..., v11 4d0ff508....)")
        return 0
    finally:
        d.close()


if __name__ == '__main__':
    sys.exit(main())
