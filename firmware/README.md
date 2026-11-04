# Firmware

Our firmware is Expatria's, with the flight recorder added: a recorder variant
of each of their PrintNC builds, which extends it and changes only what the
recorder needs. Same board options, same UF2 bootloader, same way of flashing.

| Variants | Expatria build | For |
|---|---|---|
| `f446re_flexi_cnc_printnc_eth_sd_fr`, `..._fr_wd` | `printnc_eth_sd` | PrintNC with the uFlexiNET (network + SD) |
| `f446re_flexi_cnc_printnc_eth_sd_4x_fr`, `..._fr_wd` | `printnc_eth_sd_4x` | the same, 4 axes |
| `f446re_flexi_cnc_printnc_fr`, `..._fr_wd` | `printnc` | PrintNC over USB only |

`_fr` is the recorder; `_fr_wd` adds the hardware watchdog
(`FR_WATCHDOG_ENABLE=1`, setting `$793`): a hung controller resets after
3–10 s and so stops the spindle (`docs/COMMANDS.md`, "Watchdog").

Other Flexi-HAL builds work the same way (all 17 built on 31 Oct 2026); add
a pair of sections for each to `flexihal-a7/platformio.local.ini` when needed.

## Flexi-HAL A7 (`flexihal-a7/`)

Based on [Expatria's STM32F4xx driver](https://github.com/Expatria-Technologies/STM32F4xx),
branch `F446_Flexi_HAL`, pinned in `flexihal-a7/driver.lock`.

Each variant adds to Expatria's build:

- the recorder plugin (`plugin/`) and `FLIGHT_RECORDER_ENABLE=1`;
- `EEPROM_IS_FRAM=1`: the FM24V10 FRAM fitted at U27 (see the top README);
- `MPG_ENABLE=0`: MPG stays off with the FRAM, whatever the parent build says;
- `core/executing-line.patch`, `core/plugin-init.patch` and
  `core/modbus-stats.patch` on the grblHAL core (the last only lets the
  event log read the Modbus driver's own error counters);
- `flexihal-a7/patches/i2c-wait-ready.patch` and `keypad-strobe-task.patch`:
  two fixes to Expatria's I2C, below.

The variants are written out in `flexihal-a7/platformio.local.ini`, which
`build.sh` copies into the driver.

### Building

Needs git, python3 and [PlatformIO](https://platformio.org/) (`pip install platformio`).
Expatria's UF2 step runs `python`, so that name must be on PATH too (it is
inside a venv). Linux, macOS or WSL:

```bash
firmware/flexihal-a7/build.sh list                                   # the variants
firmware/flexihal-a7/build.sh f446re_flexi_cnc_printnc_eth_sd_fr     # one
firmware/flexihal-a7/build.sh                                        # all six
```

The driver is checked out under `firmware/build/` (or `$FR_BUILD_DIR`); UF2
files land in `firmware/flexihal-a7/out/`, named like Expatria's:
`FLEXI_HAL_<BUILD>_FR[_WD]_<driver version>.uf2`, e.g.
`FLEXI_HAL_PRINTNC_ETH_SD_FR_WD_20260225_220226.uf2`. Pick the one matching the
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
own settings are `$790`–`$793` (see `docs/COMMANDS.md`); it starts after every
other plugin, so their saved settings stay where they were. Coming from
v0.2.0, its own settings go back to their defaults once (v0.3.0 added `$793`).
From v0.3.0 everything carries over: settings, the record and the event log.

After flashing, `$I` should list `[PLUGIN:Flight Recorder v0.4.0]`, and
`$EVENTS` should show a `BOOT` entry with a real cause: `reset=power-on` after
plugging in, `reset-pin` after pressing **RST**. `unknown` there means the
bootloader clears the reset flags: tell us. If `$I` says `Flight Recorder (no
storage)`, the chip at U27 isn't the FM24V10 FRAM and the recorder is off.

### First checks with a watchdog build

On the bench, spindle unloaded, Z high:

1. `$793` reads 5. Idle for a minute, jog, home, run a dwell (`G4 P15`):
   no reset (`$EVENTS` shows no new `BOOT`).
2. Press **RST** with the steppers enabled: Z mustn't drop while the board
   restarts. A watchdog reset is the same reset.
3. Start the spindle (`M3 S6000`), then `$WATCHDOGTEST`: the board hangs as if
   stuck. Within about `$793` s it resets, the spindle stops (Modbus VFDs too),
   and `$EVENTS` shows `BOOT|reset=watchdog` and `SPINDLE_OFF|after=watchdog`.
   If `BOOT` says `unknown`, the bootloader clears the reset flags: tell us.
   Over USB the sender sees the port drop and come back.

### Where Expatria's published tree needed help

- Its I2C code let a transfer start before the last one had finished with
  the port. The keypad's status packet to the pendant goes by DMA without
  waiting, and for a moment after it starts the port is busy but the bus
  isn't yet. A FRAM write in that moment was refused, which is where the
  recorder's `STORAGE failed_writes=1` lines came from.
  `patches/i2c-wait-ready.patch` waits until the port itself is ready (with a
  50 ms limit), and clears the wait for a key read that nobody answers.
- Its keypad plugin read the key from the strobe interrupt, which could land
  in the middle of another transfer. `patches/keypad-strobe-task.patch` moves
  the read into the main loop, as upstream grblHAL's keypad plugin now does.

- It records a status light (`rgb`) plugin commit that was never pushed
  (`f4b2457`), so a fresh checkout can't fetch it. We use the newest published
  commit (`672ebd8`) with `flexihal-a7/patches/rgb-bit-count.patch`: that
  version carries its own `bit_count()`, which now clashes with the core's
  (same meaning; the patch drops the copy).
- With that fix their stock `f446re_flexi_cnc_printnc_eth_sd` uses 352,056
  bytes of flash and 74,504 of RAM (measured in technical review A); our
  `_fr` variant 360,120 and 74,928: about 8 KB of flash and 0.4 KB of RAM
  more, and the watchdog another 0.4 KB of flash. That is 78.5 % of the
  448 KB the application has after the bootloader (PlatformIO's percentage
  counts the whole 512 KB chip). The USB-only `printnc_fr` uses 275,784
  (60 %). (Not compared byte for byte with their published UF2, which was
  built with the unpublished commit.)
