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

    bladeRF-cli -l fpga/images/hostedxA9_niosIIf_sweep_ts_v1.rbf

**`-l` loads into RAM and reverts on power cycle. That is deliberate — do not
use `-L` (SPI flash) without deciding to.** Confirm it took with
`bladeRF-cli -e info`, which should say *"configured by USB host"*.

If the board has been power-cycled and reverted to the stock flash image,
nothing breaks: `SFCWEngine` detects the dead sample counter at the first EXEC,
prints

    [sfcw] NIOS autonomous sweep unavailable on this FPGA image ...

and runs the standard host-driven sweep for the rest of the session (~18 Hz).
Reload the image and restart the sweep to get 36 Hz back.
