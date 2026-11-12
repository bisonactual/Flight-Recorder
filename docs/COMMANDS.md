# Flight recorder: what a sender needs

The plugin keeps a record of the job in non-volatile memory (the A7's FRAM; a
file in the simulator). It survives the PC, the link and the controller going
away. This page is the whole interface.

`$I` reports `[PLUGIN:Flight Recorder v0.3.0]` when it's running, or
`Flight Recorder (no storage)` when the board has no supported FRAM; then
these commands are not available.

## What the sender must do

1. **Number every line** with its line number in the file: `N<line><text>`.
   Replace any N-words the file already has. The board records these numbers;
   without them the record has no line to point at. grblHAL rejects N above
   10,000,000.
2. **Stamp the job** before the first line: `$FLIGHT=<job>[,<flags>]`, where
   `<job>` is the CRC-32 of the file in hex (any 32-bit id works, as long as
   the resume computes the same one).
   - flag 1: no auto retract (the job uses undercut tools).
   - Must be sent while the machine is idle (`error:8` otherwise).
3. **Stream as usual.** Character counting is fine.
4. **Resume** (a button, after the operator has rehomed): see below.

## Commands

| Command | Reply | Notes |
|---|---|---|
| `$FLIGHT` | `[FLIGHT:...]` then `ok` | Report the record (any state, alarm included) |
| `$FLIGHT=<job>[,<flags>]` | `[FLIGHT:...]` then `ok` | Start recording a job. Warns `[MSG:Flight recorder: Z not homed, ...]` if the auto retract can't work |
| `$FLIGHTCLEAR` | `[FLIGHT:...]` then `ok` | Clear the record. Not while a job runs or retracts (`error:8`) |
| `$EVENTS[=<n>]` | `[EVENT:...]` lines, oldest first, then `ok` | The last n events (default 100, up to 1000) |
| `$EVENTSCLEAR` | `ok` | Clear the event log |
| `$WATCHDOGTEST` | `[MSG:...]`, then the board resets | `_FR_WD` builds: hang, as a stuck controller would, to test the watchdog. Idle or alarm only (`error:8`); `error:53` with `$793=0` |

`$HELP` lists them. `$FLIGHT` and `$EVENTS` also answer during the alarms
where grblHAL refuses most commands until reset (e-stop, hard or soft limit,
motor fault). During a feed hold or program pause (`M0`) grblHAL reads no
commands at all, these included: they are answered on cycle start. Use the
status report (`?`) meanwhile.

## The record

```
[FLIGHT:Retracted|why=timeout|job=CF3A1F7A|ln=1702|boot=1|alarm=0|tool=0|wcs=G54|flags=23|mpos=-296.744,-268.004,-66.000,0.000,0.000]
```

| Field | Meaning |
|---|---|
| state | `Idle` (none), `Running`, `Stopped`, `Retracted`, `Completed` |
| `why` | Why it stopped: `none`, `timeout` (no input with the spindle on), `usb` (USB port closed, or the input moved to another stream), `network` (network client gone or replaced), `alarm`, `reset`, `reboot` (controller restarted mid-job) |
| `job` | The id from `$FLIGHT=<job>` |
| `ln` | N-number of the line the steppers were executing. Never ahead of the machine: resuming from the start of this line re-cuts at most part of it. Jogs don't count, even numbered ones |
| `boot` | Controller boot count when written |
| `alarm` | Alarm code when `why=alarm`: an alarm that stopped the job on its own (limit, e-stop...). A reset is `why=reset`, not the homing alarm that follows it |
| `tool`, `wcs` | Tool number and work offset in force |
| `flags` | Hex: 1 Z homed, 2 spindle on, 4 coolant on, 8 no auto retract, 10 streamed over USB, 20 over the network |
| `mpos` | Machine position when it stopped, mm |

A `Running` record found at boot becomes `Stopped`, `why=reboot`.

`ln` comes from the steppers; `tool`, `wcs`, `flags` and `mpos` from the
controller's state when it was written, which can already include a line
parsed but not yet executing (a `G55` queued behind a `G54` move, say). The
resume rebuilds them from the file instead.

Stamping the same job again after it stopped (a resume) keeps `ln` until the
first numbered line runs, so a second interruption during the re-entry moves
still points at the right line.

Only spindle 0 is watched (the one plain `M3`/`M5` address).

## Event log

The last 1000 events, kept in the FRAM across reboots: what to read when a
machine misbehaves "for no reason", which on these machines is usually
electrical noise.

```
[EVENT:812|boot=7|t=1523.410|LIMIT|min=X|max=-|n=1]
[EVENT:813|boot=7|t=1523.410|ALARM|alarm=1|ln=2341|n=1]
[EVENT:814|boot=8|t=0.000|BOOT|reset=brown-out|flags=06|job=Running|n=1]
```

`EVENT:<n>` counts events; `boot` is the controller boot they happened in;
`t` is seconds since that boot; `n` is how many times it happened: the same
event again within a second of the last is counted into one entry (up to
`n=256`), so one chattering switch or a burst of one error takes one entry.
Faults that alternate (two errors in turn, a switch and an error) take an
entry each and can push older history out. Inputs and limits are latched
between polls, so several trips in a few milliseconds count as one.

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
| `SPINDLE_OFF` | `after=` job, watchdog, brown-out | at boot after an unexpected reset: a job was running, the watchdog fired, or the supply dipped. grblHAL switches the spindle and coolant off at every start; this records that it mattered. The board sends the stop again 2 s later, for a Modbus VFD that missed the first |

Notes: the reset cause is read from the processor's reset flags as the
firmware starts, before Expatria's start-up code clears them. If `BOOT` still
says `unknown` on the A7, the UF2 bootloader clears them first, and a watchdog
reset logs no `SPINDLE_OFF` (the spindle is still stopped). The simulator
always says `unknown`. If the log's header is ever unreadable (a power cut
while it was written), it is rebuilt from the events at the next boot.

## Stream-loss retract

With a job running and the spindle on, the machine only goes idle with nothing
queued when the sender has stopped feeding it. When that lasts `$791` seconds,
or at once if the sender's link goes, the board:

1. blocks further input, so late lines can't cut, and drops any line it had
   half received;
2. lets the line already executing finish (a dwell, then its move), then
   retracts straight up to **machine** Z = −`$792` (machine zero is the top of
   Z; never measured from work zero). The spindle keeps turning;
3. stops the spindle and coolant once clear;
4. checks Z really is at the retract height, records `Retracted` and raises
   **alarm 80**: "Stream lost mid-job: tool retracted, spindle stopped. Resume
   from the flight record."

The link goes when the USB port closes, the network client leaves, or another
client takes the input over, however fast. From then until the job has
retracted or stopped, the board reads nothing from anyone: a sender that
reconnects meanwhile gets no reply (the status report, `?`, still works) and
should wait for the alarm, or for `Stopped` in the record, then ask again.

If the retract can't finish within two minutes (a feed hold or door during
it), the board stops everything instead, as a reset does, and raises **alarm
81**: "Stream lost mid-job, and the retract couldn't finish: machine stopped
where it was." The record stays `Stopped`. A reset or e-stop during the
retract stops it too; the record stays `Stopped`.

Dwells (`G4`) and spindle spin-up waits don't count as lost input. There is no
retract if Z isn't homed, `$790` is 0, or the job has flag 1; the record still
says why it stopped. Jobs run from the SD card (`$F=`) aren't watched: the
file can't hang, and there's no sender link to lose.

## Watchdog (`_FR_WD` builds)

The retract needs a working controller. If the controller itself hangs, its
outputs stay as they were, spindle included. In the `_FR_WD` builds the
processor's hardware watchdog resets it when the main loop hasn't run for
`$793` seconds. The board then starts as from any reset: spindle and coolant
off, homing alarm, nothing moves. The log shows `BOOT|reset=watchdog` and
`SPINDLE_OFF|after=watchdog`; a job that was running is recorded `Stopped`,
`why=reboot`, ready to resume once rehomed. The tool is not retracted: it stays
where the hang left it, with the spindle stopped.

So a frozen PC gets the retract after `$791` seconds; a frozen controller gets
the spindle stopped after `$793` seconds.

- The watchdog starts when the main loop first runs, so start-up can take as
  long as it needs. It is fed by the main loop and by blocking waits (dwells,
  spin-up delays), so neither trips it.
- Its clock is imprecise: 5 s fires between about 3.5 s and 9.5 s on the worst
  parts and temperatures, close to 5 s on most.
- Changing `$793` takes effect at the next boot. Once running, only a reset
  stops it.

## Settings

| Setting | Default | Meaning |
|---|---|---|
| `$790` | 1 | Auto retract on stream loss |
| `$791` | 3.0 s | Stream loss timeout |
| `$792` | 20.0 mm | Retract to this far below machine zero |
| `$793` | 5 s | Watchdog timeout, 3–10 s; 0 turns it off (1 and 2 act as 3). `_FR_WD` builds only; takes effect at the next boot |

## Resume

1. `$FLIGHT`. Offer the resume if the state is `Stopped`, `Retracted` or `Running`
   and `job` matches the loaded file.
2. The operator clears the alarm and rehomes (`$X`, `$H`), and checks the tool
   in the spindle matches `tool`. Work offsets (G54...) persist. The tool
   length offset survives a power cycle only with the persistent tool plugin
   on (`$485=1`, built into the Flexi-HAL firmware); otherwise measure it again.
3. Scan the file up to line `ln` for the modal state (units, plane, work
   offset, feed, spindle speed and direction, coolant, motion mode) and where
   each axis was at the end of the line before `ln`: its value in mm, and the
   frame it was programmed in, a work offset (`G54`...) or machine
   coordinates (`G53`). Units and work offset are the ones in force on that
   axis's own line, not at `ln`: a program can change either later without
   moving.
4. Refuse, before anything moves, what the scan can't rebuild: incremental
   moves (`G91`), inverse-time or per-revolution feed (`G93`, `G95`), cutter
   compensation, canned cycles or probing (also on line `ln` itself), `G10` or
   `G92` offsets, axes other than XYZ, a spindle turning with no `S`, an axis
   whose last move was `G28`/`G30`.
5. Send, then stream the file from line `ln` on:
   ```
   $FLIGHT=<job>
   G21 G90 G17 G94                 (re-entry in mm, absolute, per minute; the scanned plane)
   G53 G0 Z-20                     (machine Z, the retract height)
   G54 G0 X<x> Y<y>                (over the start of line ln, each axis in its own frame)
   S<s> M3                         (and G4 for spin-up)
   M8                              (if coolant was on)
   G54 G1 Z<z> F<plunge>           (straight down: the tool has been there; F in mm/min)
   G20 G55 F<feed>                 (the program's own units, work offset and feed back)
   ```
   If line `ln` has no motion word of its own, prefix it with the scanned
   motion mode (`G1`, `G2`, ...).

Straight down is safe for ordinary tools: the tool body has already swept the
column above every point its tip reached. Undercut tools are the exception.

`tools/stream.py` does all of this in Python, for the simulator runs.
