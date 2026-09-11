# FPGA images

## `hostedxA9_niosIIf_sweep_ts_v1.rbf`

The image the SFCW autonomous sweep (`sweep_mode='nios'`, the default) requires.
Validated on hardware 2026-09-07 — see CLAUDE.md, "Nios II/f FPGA image +
autonomous sweep".

    sha256  3449d1af4275fa704e108ff1853d543e7660a6eae138180dc81bdf38a2d24df1

Built from Nuand upstream `73ce750` plus three modified files (`rx.vhd`,
`devices.c`, `pkt_retune2.c`) — full build parameters, tool versions and
per-file hashes are in the `.PROVENANCE.txt` beside it. Two things it adds over
stock:

- **Nios II/f** (`NIOS_REV=Fast`, needs Quartus Prime **Standard** — Lite gives
  only Nios II/e, and Pro does not support Cyclone V at all). Cuts the per-step
  AD9361 SPI cost ~6x. This alone speeds up the ordinary host-driven sweep from
  65.5 ms to ~55.5 ms with no host-side change.
- **The autonomous sweep firmware + the `rx.vhd` timestamp fix.** Upstream
  releases the RX time tamer's reset on `meta_en` alone, so the sample counter
  is pegged at 0 in plain `SC16_Q11` and nothing can be scheduled against it.

Together these take the 51-step sweep to **27.5 ms (36.4 Hz)**.

## Loading it

**This image is FLASHED TO SPI as of 2026-09-11, so the board autoloads it and
nothing needs doing after a power cycle.** It was RAM-loaded (`-l`) until then,
which meant every power cycle silently halved the sweep rate — that is exactly
how it was lost on 2026-09-11, while the LiDAR was being rewired. The stock
0.16.0 image is no longer on the board; reverting means re-downloading it from
Nuand.

    bladeRF-cli -L fpga/images/hostedxA9_niosIIf_sweep_ts_v1.rbf   # flash, persists
    bladeRF-cli -l fpga/images/hostedxA9_niosIIf_sweep_ts_v1.rbf   # RAM, for testing

**Do not verify with the "configured by ..." string — its meaning flipped when
this was flashed.** *"configured from SPI flash"* used to mean the stock image
and was the signature of the fault; it now means this image loaded correctly.
*"configured by USB host"* now means something was `-l`-loaded over the top.

**Verify behaviourally instead:** run a sweep and read `sweep_core` on
`sfcw_result` — `nios` is working, `standard` means the capability latch tripped.
The rate alone also identifies it: ~37 Hz autonomous, **~18 Hz = this image but
no NIOS sweep**, ~15 Hz = the old II/e image.

If the sample counter is ever dead, nothing breaks: `SFCWEngine` detects it at
the first EXEC, prints

    [sfcw] NIOS autonomous sweep unavailable on this FPGA image ...

and runs the standard host-driven sweep for the rest of the session (~18 Hz).
