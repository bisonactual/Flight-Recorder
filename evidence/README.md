# Evidence: flight recorder in the simulator

The SD card storage (v0.4.0) has its own runs: `SD.md`. This page is the
FRAM path, which v0.4.0 leaves as it was.

31 Oct 2026, after the fix pass for technical review A. GRBLHAL-Sim branch
`usb-stream` with this repository built in (`-DFLIGHT_RECORDER_DIR`), WSL
(Linux), run at 4× simulated speed.

The whole-job runs use a simulator built with the processor clock at 1 MHz
instead of 16 (`-DCMAKE_C_FLAGS="-UF_CPU -DF_CPU=1000000"`). The simulator
steps the clock in software, so at 16 MHz a whole job only fits the
two-minute cap per run when the host is otherwise idle; at 1 MHz they take
about a minute. The timers are coarser, so it isn't timing-identical to the
board, but the records, lines and coverage came out the same as the 16 MHz
runs (and as technical review A's 1 MHz runs). The focused scenarios run on
the normal 16 MHz build.

## The job

`programs/freecad_fcL_spindle.nc`: the FreeCAD-made pocket from the Fusion
timing set (`freecad_fcL/program.nc`, 4,256 lines, F1000, arcs), with two
changes so it can run here: the tool change (`T1`/`M6`) removed, since the
simulator's ATCi plugin would take it, and `S12000 M3` + `G4 P1` added after
the header, `M5` before `M30`, so the spindle is on while cutting.

Machine: X/Y 6000 mm/min and 500 mm/s², Z 3000 mm/min and 300 mm/s²; G54 at
machine X−300 Y−300 Z−60; homed, so machine zero is the top of Z.

## Whole-job scenarios

| Run | What happens | Record afterwards | Resume | Coverage vs reference (p99 / max) |
|---|---|---|---|---|
| `reference` | uninterrupted, WebSocket | Completed, line 4255 | — | — |
| `hang-ws` | sender stops sending and reading at 40 %, WebSocket stays open | **Retracted**, timeout, line 1702 | over USB → Completed | 0.009 / 0.019 mm |
| `hang-usb` | same over USB, port stays open, at 50 % | **Retracted**, timeout, line 2128 | over WebSocket → Completed | 0.009 / 0.023 mm |
| `close-usb` | sender closes the USB port at 55 % | **Retracted**, usb, line 2318 | over USB → Completed | 0.009 / 0.024 mm |
| `close-ws` | WebSocket client closes at 70 % | **Retracted**, network, line 2960 | over WebSocket → Completed | 0.009 / 0.016 mm |
| `power-ws` | controller killed mid-cut at 30 % (power cut), restarted | **Stopped**, reboot, line 1172 | over WebSocket → Completed | 0.009 / 0.011 mm |
| `dwell-ws` | `G4 P6` (6 s, longer than the 3 s timeout) with the spindle on, line-at-a-time sender | **Running** through the dwell; then the sender hangs → **Retracted**, timeout | — | — |

- **Coverage** is the farthest any point of the reference toolpath inside the
  stock (below work Z 0) lies from where the interrupted run went inside the
  stock, both boots and the re-entry included. 0.02 mm is the resolution of
  the 10 ms position samples: no part of the cut was left out. It compares
  paths only: not their order, nor whether the spindle was turning (the
  focused scenarios below check those things where it matters).
- **No invented motion:** the path is never interpolated across samples
  farther apart than one sample can move. Each run that resumes without a
  reboot has one such jump, the homing cycle (`$H`), which the simulator's
  step log doesn't sample; both its ends are above the stock.
- **Power cut, never ahead:** following the step samples through the program
  in order, the last one before the kill lies on line 1172, 0.0015 mm from
  its path: the record named exactly the line being cut. (The pocket's later
  passes run back over the same points, so taking the nearest line to the
  sample instead picks a line 53 earlier and looks like the record ran ahead.
  That sample is up to 10 ms before the kill; the never-ahead argument is in
  the core patch, not in this sample.) At the next boot the log shows
  `SPINDLE_OFF|after=job`.
- **A lost link drops unread input:** the USB sender had sent up to line
  2341 and the WebSocket one up to 2979, but once the link goes the board
  reads nothing more, so the last lines executed, and recorded, were 2318
  and 2960. (Before the fix pass, `close-usb` carried on to 2341: lines the
  dead sender had sent were still read.)
- After every retract the machine was at machine Z −20 with the spindle off,
  in alarm 80, until the resume.

## Focused scenarios

`tools/focused.py`: technical review A's falsifiers, each a short numbered
job (G54, a move to work X10 Y10 Z−1, spindle on) with its own checks. All
pass after the fix pass; the review had found the first six failing.

| Scenario | What happens | Checked |
|---|---|---|
| `dwell-descend` | `G4P6 G1Z0` on one line; the client closes during the dwell | the plunge runs, then the retract: **Retracted at machine Z −20** (before: Retracted, but left at −60) |
| `numbered-jog` | `$J=... N900` during the job | record still at line 2 (before: 900) |
| `partial-line` | half a line sent, then the timeout | the first query after the retract is answered (before: swallowed by the half line) |
| `resume-power` | power cut during a resume's re-entry moves | record keeps line 2, Stopped/reboot (before: line 0) |
| `reconnect` | client closes, a new one connects 50 ms later and sends N5 | **N5 never runs**; Retracted, network (before: N5 ran) |
| `reset-control` | reset during a numbered move | `why=reset` (before: alarm 11) |
| `estop-retract`, `reset-retract` | e-stop / reset during the retract | Stopped, not Retracted; spindle off |
| `g20`, `g91` | the job in inches / incremental | Retracted at machine Z −20 |
| `check` | check mode | no retract |
| `m0` | program pause past the timeout | no retract; cycle start carries on (N5 recorded) |
| `hold-retract` | feed hold during the retract until it gives up (short-limit build) | **alarm 81**, Stopped, machine stopped |

### Event log (`events`, `dwell-ws`)

`events` provokes each kind of entry in the simulator and reads `$EVENTS` back
after a power cut:

```
ERROR|error=75|n=4                  four garbled lines, one entry
LIMIT|min=X  +  ALARM|alarm=1       limit tripped with hard limits on
INPUT|feed_hold|n=5                 five quick presses, one entry
INPUT|e_stop  +  ALARM|alarm=10     e-stop
LINK|network|connected/disconnected, LINK|usb|connected/disconnected
BOOT|reset=unknown|job=Idle         after the power cut (no reset cause in the simulator)
```

`dwell-ws` shows the job entries: `JOB start`, `JOB Stopped why=timeout ln=6`,
`JOB Retracted`, `ALARM 80`. A torn log header (bytes 5–15 zeroed in the
simulator's `FLIGHT.DAT`) is rebuilt from the events at the next boot, with
the whole history kept. Modbus and storage failures need the real board.

`results/<run>-z.png` plots machine Z over time; gaps are idle stretches
(no motion), including the timeout wait before a retract. `results/analysis.json`
has the numbers.

## Found on the way (simulator, not the plugin)

- **G54 didn't survive a restart** in the simulator: an upstream grblHAL bug
  visible only on x86 hosts (`nvs_buffer_sync_physical()` keeps NVS addresses
  in a `uint_fast8_t`, 8 bits there). Patched in the simulator; the board keeps
  its offsets (owner confirmed).
- **One unexplained `error:` reply** in one WebSocket run, before the
  simulator's thread-shared buffers were ordered at all; none since. The
  buffer indices now use acquire/release (`src/shared.h`), and the settings
  file is written in place, so a kill mid-write can't empty it.
- The step logger stopped as soon as the planner emptied, missing the last
  segments of each stretch of motion; it now runs until the steppers stop.

## Not covered here

- The A7 itself: FRAM writes over I2C and their timing, the USB stack's DTR
  behaviour, a real hung PC with the USB port still open, the reset cause
  read at start-up, the watchdog.
- Jobs from the SD card: the simulator's file runner is a stub.
- Windows: the simulator builds with MSVC, but the COM-port path (com0com) has
  not been run.
- The resume streamer re-enters at the job's feed (F1000); a real sender may
  want the program's plunge feed.

Reproduce: `tools/evidence.py <scenario> ...` on the 1 MHz simulator build and
`tools/focused.py <scenario> ...` on the normal one (see their docstrings),
then `tools/analyse.py`.
