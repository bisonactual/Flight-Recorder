# Firmware

Our firmware is Expatria's, with the flight recorder added: a recorder variant
of each of their PrintNC builds, which extends it and changes only what the
recorder needs. Same board options, same UF2 bootloader, same way of flashing.

| Variant | Expatria build | For |
|---|---|---|
| `f446re_flexi_cnc_printnc_eth_sd_fr` | `printnc_eth_sd` | PrintNC with the uFlexiNET (network + SD) |
| `f446re_flexi_cnc_printnc_eth_sd_4x_fr` | `printnc_eth_sd_4x` | the same, 4 axes |
| `f446re_flexi_cnc_printnc_fr` | `printnc` | PrintNC over USB only |

Other Flexi-HAL builds work the same way (all 17 built on 31 Oct 2026); add
their names to `FAMILIES` in `flexihal-a7/variants.py` when needed.

## Flexi-HAL A7 (`flexihal-a7/`)

Based on [Expatria's STM32F4xx driver](https://github.com/Expatria-Technologies/STM32F4xx),
branch `F446_Flexi_HAL`, pinned in `flexihal-a7/driver.lock`.

Each variant adds to Expatria's build:

- the recorder plugin (`plugin/`) and `FLIGHT_RECORDER_ENABLE=1`;
- `EEPROM_IS_FRAM=1`: the FM24V10 FRAM fitted at U27 (see the top README);
- `core/executing-line.patch`, `core/plugin-init.patch` and
  `core/modbus-stats.patch` on the grblHAL core (the last only lets the
  event log read the Modbus driver's own error counters).

Builds that enable MPG are left out (none do today).

### Building

Needs git, python3 and [PlatformIO](https://platformio.org/) (`pip install platformio`).
Expatria's UF2 step runs `python`, so that name must be on PATH too (it is
inside a venv). Linux, macOS or WSL:

```bash
firmware/flexihal-a7/build.sh list                                   # the variants
firmware/flexihal-a7/build.sh f446re_flexi_cnc_printnc_eth_sd_fr     # one
firmware/flexihal-a7/build.sh                                        # all three
```

The driver is checked out under `firmware/build/` (or `$FR_BUILD_DIR`); UF2
files land in `firmware/flexihal-a7/out/`, named like Expatria's:
`FLEXI_HAL_<BUILD>_FR_<driver version>.uf2`, e.g.
`FLEXI_HAL_PRINTNC_ETH_SD_FR_20260225_220226.uf2`. Pick the one matching the
Expatria build the machine runs today.

### Flashing

Exactly as Expatria's own releases (their
[Flexi-HAL README](https://github.com/Expatria-Technologies/Flexi-HAL)):

1. Enter the bootloader: power up the MCU side (plug in USB) while holding
   **CYC/ST** and **FD/HLD**, or double-tap **RST** by the USB-C port.
2. The board appears as a USB drive called **FLEXI446**.
3. Copy the `.uf2` onto it; the board installs it and reboots. (Some systems
   complain the disk was removed without ejecting: that's normal.)

Back up the settings first (`$$`) as with any firmware change. The recorder's
own settings are `$790`–`$792` (see `docs/COMMANDS.md`); it starts after every
other plugin, so their saved settings stay where they were.

After flashing, `$I` should list `[PLUGIN:Flight Recorder v0.2.0]`, and
`$FRLOG` should show a `BOOT` entry. If it
says `Flight Recorder (no storage)`, the chip at U27 isn't the FM24V10 FRAM
and the recorder is off.

### Where Expatria's published tree needed help

- It records a status light (`rgb`) plugin commit that was never pushed
  (`f4b2457`), so a fresh checkout can't fetch it. We use the newest published
  commit (`672ebd8`) with `flexihal-a7/patches/rgb-bit-count.patch`: that
  version carries its own `bit_count()`, which now clashes with the core's
  (same meaning; the patch drops the copy).
- With that fix their stock `f446re_flexi_cnc_printnc_eth_sd` builds at
  flash 67.1 %, RAM 56.8 %; our variant adds about 4 KB of flash and
  300 bytes of RAM. (Not compared byte for byte with their published UF2,
  which was built with the unpublished commit.)

## Flexi-HAL 2350

Not yet: Expatria's RP2350 driver, the same way.
