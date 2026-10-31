# Flight-Recorder

A grblHAL plugin that keeps a record of the running job in non-volatile memory
(the Flexi-HAL A7's I2C FRAM), so a sender can resume after the PC, the link or
the controller has gone away, and that gets the tool out of the cut when the
stream stops. Open source (GPLv3, like grblHAL).

- **What it records:** the N-number of the line the steppers are executing
  (never ahead of the machine), machine position, tool, work offset, and why
  the job stopped: program end, alarm, reset, reboot, or the stream going away.
- **Stream-loss retract:** with the spindle on, if the sender stops feeding the
  job (timeout) or its USB port / network client closes, the board blocks
  further input, retracts straight up to machine Z −20, stops the spindle,
  records it and raises alarm 80.
- **Resume:** the sender reads `$FR`, rebuilds the modal state up to the
  recorded line, re-enters straight down onto its start and streams on.
- **Event log:** the last 1000 events across reboots (`$FRLOG`): resets and
  their cause (brown-out, reset pin, watchdog), alarms, error replies, inputs
  and limit switches tripping, the sender's link coming and going, and Modbus
  timeouts and CRC errors. Aimed at the "silent" faults EMI causes.

The sender side is in [docs/COMMANDS.md](docs/COMMANDS.md).

## Supported hardware

- **Flexi-HAL Rev A7** with the EEPROM at **U27** replaced by an
  **FM24V10** (Infineon, 1 Mbit / 128 KB I2C FRAM, SOIC-8). This direct
  replacement is the only supported storage.
  - grblHAL's settings keep the first 16 KB, as before; the recorder uses the
    space after them.
  - At start-up the recorder checks the chip is that FRAM and leaves it alone
    otherwise (it reports `Flight Recorder (no storage)`): a 16 KB part would
    have its settings overwritten, and an EEPROM would wear out.
- **MPG stays off** in these builds.
- Connection over **USB** or the **uFlexiNET** network module.

## Layout

```text
plugin/                 the grblHAL plugin
  flight_recorder.c/h     record, stream-loss retract, $ commands, settings
  event_log.c/h           the event log
  storage_fram.c          the A7's FRAM (checked at start-up) and reset cause
  storage_file.c          a host file (simulator)
  library.json            PlatformIO library (FRAM storage only)
core/                   grblHAL core patches
  executing-line.patch    report the line the steppers are executing
  plugin-init.patch       start the recorder (firmware builds)
  modbus-stats.patch      read the Modbus driver's error counters (firmware builds)
firmware/               board firmware: Expatria's builds plus the recorder
  flexihal-a7/            Flexi-HAL A7 (STM32F446), UF2 for Expatria's bootloader
docs/COMMANDS.md        $ commands, record, settings, alarm, how to resume
tools/                  test sender, simulator harness, evidence scripts
evidence/               simulator runs: programs, results, plots
```

## Why a core patch

grblHAL's status report (`Ln:`) takes the line number from the planner block
the segment generator is preparing, which can be up to nine segments ahead of
the steppers. A record taken from it could name a line the machine hasn't
reached. Resuming from there would skip material and, worse, plunge where the
tool has never been. `core/executing-line.patch` (15 lines in `stepper.c/h`)
copies the line number into the stepper's block and records it when the
stepper interrupt starts executing that block. It is general enough to offer
upstream.

## Building

**Firmware (Flexi-HAL A7):** see [firmware/README.md](firmware/README.md).

```bash
firmware/flexihal-a7/build.sh f446re_flexi_cnc_printnc_eth_sd_fr
```

**Simulator** ([GRBLHAL-Sim](https://github.com/bisonactual/GRBLHAL-Sim),
branch `usb-stream` until merged):

```bash
cmake -S GRBLHAL-Sim -B build -DCMAKE_BUILD_TYPE=Release -DFLIGHT_RECORDER_DIR=$PWD/Flight-Recorder
cmake --build build -j3
./build/grblHAL_flexihal_sim -u /tmp/flexihal-usb -w 8080
```

## Evidence

See [evidence/](evidence/). Runs use the FreeCAD job in
`evidence/programs/freecad_fcL_spindle.nc` in the simulator at 4× speed.
