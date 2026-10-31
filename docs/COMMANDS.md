# Flight recorder: what a sender needs

The plugin keeps a record of the job in non-volatile memory (the A7's FRAM; a
file in the simulator). It survives the PC, the link and the controller going
away. This page is the whole interface.

`$I` reports `[PLUGIN:Flight Recorder v0.2.0]` when it's running, or
`Flight Recorder (no storage)` when the board has no supported FRAM; then
these commands are not available.

## What the sender must do

1. **Number every line** with its line number in the file: `N<line><text>`.
   Replace any N-words the file already has. The board records these numbers;
   without them the record has no line to point at. grblHAL rejects N above
   10,000,000.
2. **Stamp the job** before the first line: `$FRJ=<job>[,<flags>]`, where
   `<job>` is the CRC-32 of the file in hex (any 32-bit id works, as long as
   the resume computes the same one).
   - flag 1: no auto retract (the job uses undercut tools).
   - Must be sent while the machine is idle (`error:8` otherwise).
3. **Stream as usual.** Character counting is fine.
4. **Resume** (a button, after the operator has rehomed): see below.

## Commands

| Command | Reply | Notes |
|---|---|---|
| `$FR` | `[FR:...]` then `ok` | Report the record (any state, alarm included) |
| `$FRJ=<job>[,<flags>]` | `[FR:...]` then `ok` | Start recording a job. Warns `[MSG:Flight recorder: Z not homed, ...]` if the auto retract can't work |
| `$FRC` | `[FR:...]` then `ok` | Clear the record |
| `$FRLOG[=<n>]` | `[FRLOG:...]` lines, oldest first, then `ok` | The last n events (default 100, up to 1000) |
| `$FRLOGC` | `ok` | Clear the event log |

## The record

```
[FR:Retracted|why=timeout|job=CF3A1F7A|ln=1702|boot=1|alarm=0|tool=0|wcs=G54|flags=23|mpos=-296.744,-268.004,-66.000,0.000,0.000]
```

| Field | Meaning |
|---|---|
| state | `Idle` (none), `Running`, `Stopped`, `Retracted`, `Completed` |
| `why` | Why it stopped: `none`, `timeout` (no input with the spindle on), `usb` (USB port closed), `network` (network client gone), `alarm`, `reset`, `reboot` (controller restarted mid-job) |
| `job` | The id from `$FRJ` |
| `ln` | N-number of the line the steppers were executing. Never ahead of the machine: resuming from the start of this line re-cuts at most part of it |
| `boot` | Controller boot count when written |
| `alarm` | Alarm code when `why=alarm` |
| `tool`, `wcs` | Tool number and work offset in force |
| `flags` | Hex: 1 Z homed, 2 spindle on, 4 coolant on, 8 no auto retract, 10 streamed over USB, 20 over the network |
| `mpos` | Machine position when it stopped, mm |

A `Running` record found at boot becomes `Stopped`, `why=reboot`.

## Event log

The last 1000 events, kept in the FRAM across reboots: what to read when a
machine misbehaves "for no reason", which on these machines is usually
electrical noise.

```
[FRLOG:812|boot=7|t=1523.410|LIMIT|min=X|max=-|n=1]
[FRLOG:813|boot=7|t=1523.410|ALARM|alarm=1|ln=2341|n=1]
[FRLOG:814|boot=8|t=0.000|BOOT|reset=brown-out|flags=06|job=Running|n=1]
```

`FRLOG:<n>` counts events; `boot` is the controller boot they happened in;
`t` is seconds since that boot; `n` is how many times it happened: the same
event again within a second of the last is counted into one entry, so a
chattering switch or a burst of errors can't push the history out.

| Event | Fields | Logged when |
|---|---|---|
| `BOOT` | `reset=` power-on, brown-out, reset-pin, watchdog, software, low-power, unknown; `flags` (all reset flags, hex: 1 power-on, 2 brown-out, 4 pin, 8 software, 10 watchdog, 20 low-power); `job=` the record's state at boot | every start. `job=Running` means it restarted mid-job. A brown-out is a supply dip; reset-pin alone is the reset button or noise on the NRST line |
| `ALARM` | `alarm=` code, `ln=` line executing | every alarm |
| `ERROR` | `error=` status code, `ln=` last line number parsed | every error reply. Garbled lines from a noisy link show up here |
| `INPUT` | names: `reset`, `feed_hold`, `cycle_start`, `door`, `e_stop`, `probe_disconnected`, `motor_fault`, `motor_warning`, `toolsetter_overtravel`, `probe_overtravel` | an input going active |
| `LIMIT` | `min=` / `max=` axes | a limit switch tripping, outside homing |
| `LINK` | `usb` / `network`, `connected` / `disconnected` | the sender's USB port opening or closing, a network client arriving or leaving |
| `MODBUS` | `timeouts`, `crc` (errors), `exceptions`, `retries`, each up to 255 | once a second while the Modbus link (the VFD) has trouble |
| `JOB` | `start` with `job=`; or the state (`Stopped`, `Retracted`, `Completed`), `why=` and `ln=` | the job record changing |
| `STORAGE` | `failed_writes=` | FRAM writes that failed, logged once they work again |

Notes: the reset cause is read from the processor's reset flags; if the UF2
bootloader clears them first, `BOOT` says `unknown`. The simulator always
says `unknown`.

## Stream-loss retract

With a job running and the spindle on, the machine only goes idle with nothing
queued when the sender has stopped feeding it. When that lasts `$791` seconds,
or at once if the USB port or network client has closed, the board:

1. blocks further input, so late lines can't cut;
2. retracts straight up to **machine** Z = −`$792` (machine zero is the top of
   Z; never measured from work zero). The spindle keeps turning;
3. stops the spindle and coolant once clear;
4. records `Retracted` and raises **alarm 80**:
   "Stream lost mid-job: tool retracted, spindle stopped. Resume from the
   flight record."

Dwells (`G4`) and spindle spin-up waits don't count as lost input. There is no
retract if Z isn't homed, `$790` is 0, or the job has flag 1; the record still
says why it stopped.

## Settings

| Setting | Default | Meaning |
|---|---|---|
| `$790` | 1 | Auto retract on stream loss |
| `$791` | 3.0 s | Stream loss timeout |
| `$792` | 20.0 mm | Retract to this far below machine zero |

## Resume

1. `$FR`. Offer the resume if the state is `Stopped`, `Retracted` or `Running`
   and `job` matches the loaded file.
2. The operator clears the alarm and rehomes (`$X`, `$H`), and checks the tool
   in the spindle matches `tool`. Work offsets (G54...) persist. The tool
   length offset survives a power cycle only with the persistent tool plugin
   on (`$485=1`, built into the Flexi-HAL firmware); otherwise measure it again.
3. Scan the file up to line `ln` for the modal state: units, plane, distance
   mode, feed mode, work offset, feed, spindle speed and direction, coolant,
   motion mode, and the position at the end of the line before `ln`.
4. Send, then stream the file from line `ln` on:
   ```
   $FRJ=<job>
   G21 G90 G17 G94 G54             (the scanned modes)
   G53 G0 Z-20                     (machine Z, the retract height)
   G0 X<x> Y<y>                    (over the start of line ln)
   S<s> M3                         (and G4 for spin-up)
   M8                              (if coolant was on)
   G1 Z<z> F<plunge>               (straight down: the tool has been there)
   F<feed>
   ```
   If line `ln` has no motion word of its own, prefix it with the scanned
   motion mode (`G1`, `G2`, ...).

Straight down is safe for ordinary tools: the tool body has already swept the
column above every point its tip reached. Undercut tools are the exception.

`tools/stream.py` does all of this in Python, for the simulator runs.
