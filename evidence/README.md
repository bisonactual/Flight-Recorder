# Evidence: flight recorder in the simulator

31 Oct 2026. GRBLHAL-Sim branch `usb-stream` with this repository built in
(`-DFLIGHT_RECORDER_DIR`), WSL (Linux), run at 4× simulated speed (power cut
at 6×, to stay under the two-minute cap per run).

## The job

`programs/freecad_fcL_spindle.nc`: the FreeCAD-made pocket from the Fusion
timing set (`freecad_fcL/program.nc`, 4,256 lines, F1000, arcs), with two
changes so it can run here: the tool change (`T1`/`M6`) removed, since the
simulator's ATCi plugin would take it, and `S12000 M3` + `G4 P1` added after
the header, `M5` before `M30`, so the spindle is on while cutting.

Machine: X/Y 6000 mm/min and 500 mm/s², Z 3000 mm/min and 300 mm/s²; G54 at
machine X−300 Y−300 Z−60; homed, so machine zero is the top of Z.

## Scenarios

| Run | What happens | Record afterwards | Resume | Coverage vs reference (p99 / max) |
|---|---|---|---|---|
| `reference` | uninterrupted, WebSocket | Completed, line 4255 | — | — |
| `hang-ws` | sender stops sending and reading at 40 %, WebSocket stays open | **Retracted**, timeout, line 1702 | over USB → Completed | 0.009 / 0.019 mm |
| `hang-usb` | same over USB, port stays open, at 50 % | **Retracted**, timeout, line 2128 | over WebSocket → Completed | 0.009 / 0.022 mm |
| `close-usb` | sender closes the USB port at 55 % | **Retracted**, usb, line 2341 | over USB → Completed | 0.009 / 0.017 mm |
| `close-ws` | WebSocket client closes at 70 % | **Retracted**, network, line 2960 | over WebSocket → Completed | 0.009 / 0.020 mm |
| `power-ws` | controller killed mid-cut at 30 % (power cut), restarted | **Stopped**, reboot, line 1165 | over WebSocket → Completed | 0.009 / 0.022 mm |
| `dwell-ws` | `G4 P6` (6 s, longer than the 3 s timeout) with the spindle on, line-at-a-time sender | **Running** through the dwell; then the sender hangs → **Retracted**, timeout | — | — |

- **Coverage** is the farthest any point of the reference toolpath lies from
  what the interrupted run cut, both boots and the retract and re-entry
  included. 0.02 mm is the resolution of the 10 ms position samples: nothing
  was skipped.
- **Power cut, never ahead:** the last stepper sample before the kill lies on
  line 1165, 0.0014 mm from it: the record named exactly the line being cut.
  (An earlier run of the same scenario: line 1163, 0.0002 mm.)
- **Network close drops unread input:** the sender had sent up to line 2979,
  but grblHAL drops a network client's unread input when it goes, as on the
  board, so the last line executed, and recorded, was 2960.
- After every retract the machine was at machine Z −20 with the spindle off,
  in alarm 80, until the resume.

### Event log (`events`, `dwell-ws`)

`events` provokes each kind of entry in the simulator and reads `$FRLOG` back
after a power cut:

```
ERROR|error=75|n=4                  four garbled lines, one entry
LIMIT|min=X  +  ALARM|alarm=1       limit tripped with hard limits on
INPUT|feed_hold|n=5                 five quick presses, one entry
INPUT|e_stop  +  ALARM|alarm=10     e-stop
LINK|network|connected, LINK|usb|connected, LINK|usb|disconnected
BOOT|reset=unknown|job=Idle         after the power cut (no reset cause in the simulator)
```

`dwell-ws` shows the job entries: `JOB start`, `JOB Stopped why=timeout ln=6`,
`JOB Retracted`, `ALARM 80`. Modbus and storage failures need the real board.

`results/<run>-z.png` plots machine Z over time; gaps are idle stretches
(no motion), including the timeout wait before a retract. `results/analysis.json`
has the numbers.

## Found on the way (simulator, not the plugin)

- **G54 didn't survive a restart** in the simulator: an upstream grblHAL bug
  visible only on x86 hosts (`nvs_buffer_sync_physical()` keeps NVS addresses
  in a `uint_fast8_t`, 8 bits there). Patched in the simulator; the board keeps
  its offsets (owner confirmed).
- **One unexplained `error:` reply** in one WebSocket run before the
  simulator's thread-shared buffers had memory fences; none in the 9 full-job
  runs (and the dwell run) since. The cause wasn't pinned down.
- The step logger stopped as soon as the planner emptied, missing the last
  segments of each stretch of motion; it now runs until the steppers stop.

## Not covered here

- The A7 itself: FRAM writes over I2C, the USB stack's DTR behaviour, a real
  hung PC with the USB port still open.
- Windows: the simulator builds with MSVC, but the COM-port path (com0com) has
  not been run.
- The resume streamer re-enters at the job's feed (F1000); a real sender may
  want the program's plunge feed.

Reproduce: `tools/evidence.py <scenario> ...` (see its docstring), then
`tools/analyse.py`.
