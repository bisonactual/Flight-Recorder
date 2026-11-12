# Evidence: SD card storage in the simulator

12 Nov 2026, branch `sd-backend`. GRBLHAL-Sim branch `sd-card` (from
`usb-stream`; a local branch, commits `6db015a` and `d96e11c`) with this
repository built in, WSL (Linux). Builds: `tools/build_sim.sh <sim> <out>`.
Runs: `tools/sd_evidence.py` (power cuts, long card pauses, an EMI storm), and
the existing `tools/focused.py` and `tools/evidence.py` with `--sd`. The
numbers are in `results/sd/`.

## The card the simulator emulates

The simulator had no SD card at the sector level (its job files are a host
directory, `fs_sim.c`), so it has one now: `src/sim_sdcard.c`, an image file
of 512-byte sectors (4 GiB sparse, FAT32 with 32 KiB clusters, formatted by
FatFs on first use; exFAT on request) with the board's FatFs (R0.14b) on it.
Its disk functions time things the way the Flexi-HAL's `Src/diskio.c` sees a
card on SPI:

| | Model | Why |
|---|---|---|
| Commands | each waits for the card to be ready first | `send_cmd()` → `wait_ready()`. On the board that wait's timeout counts down in a foreground task, so it never runs out while the foreground waits |
| SPI clock | 5.625 MHz: a 512-byte block is 0.75 ms | `FATFS_SPI_PRESCALER` 16 on APB2's 90 MHz |
| Read | 0.3–1.5 ms access time, then the transfer | |
| Write busy | 1–4 ms; with probability 1 % (or `SIM_SD_PAUSE`), 50–250 ms | 250 ms is the SD spec's write timeout |
| Power cut | while the card is busy, the sector is left torn: the old data, part of the new, garbage or erased (a quarter each) | the simulator killed with SIGKILL (`sim.kill()`), or by itself during its n-th write (`SIM_SD_CUT_AT`) |
| Unit tears | `SIM_SD_TEAR_UNIT=n`: the garbage or erasure takes the whole aligned unit of n sectors | cards whose controller rewrites more than the sector it was given |

Every write, finished write, cut and disk call that held the foreground 1 ms
or more (or waited for a busy card) goes into a trace next to the image. The
simulator's clock advances in 10 ms frames, so a time measured inside the
simulator can read up to 10 ms long; whether a call *waited for a busy card*
is exact, and is the measure used for "never blocks".

What it doesn't model: a real card's latency distribution, flash pages paired
with earlier writes (MLC/TLC), the card's own mapping tables getting damaged,
the W5500 actually sharing the bus. Those are for the hardware tests.

## What was decided, and the measurements behind each choice

| Choice | Measured |
|---|---|
| Raw sector writes into FLIGHT.DAT, not f_write + f_sync | f_sync rewrites the directory entry every time: 2.1 card writes per record or log write instead of 1; grblHAL waits for a busy card inside the realtime poll (up to 246 ms), which hung a job at line 2 every time; and a power cut in the directory sector lost FLIGHT.DAT (below) |
| Never wait for the card: RAM, then one sector per poll while it is idle | no disk call ever waited for a busy card; 16 pauses of up to 248 ms in 20 s left the realtime poll gap at 10 ms (one simulator frame) |
| Copies and log in three parts 4 MiB apart | with the parts packed together, a card that tears 32 KiB units lost both copies in 11 of 60 power cuts; 4 MiB apart, 0 of 100 (below) |
| Batched events | an EMI-like burst of 850 events took 102 card writes, against 1,078 written one by one |
| Poll in grblHAL's waits too | an alarm's log entry reached the card 100 ms late (after a 50 ms power cut) until the poll also ran from `on_execute_delay`; now 1 ms |

## Power cuts at random points (`sd_evidence.py power`)

Each cycle boots, stamps a job, streams a numbered zigzag of 2 mm cuts (a new
line every ~80 ms, so a record every 100 ms) and dies: during its n-th card
write (n random, 1–40, three cycles in four) or at a random moment in the
first 4 s. Before the next boot the two record copies are read straight from
the image. The next boot must then report the newest *valid* copy (an
independent Python check of CRC and sequence), never one older than the last
record the card *finished* (from the trace), the card must still mount with
FLIGHT.DAT, and at most one sector of the log (32 events) may be unreadable.

| Series | Layout | Tears | Cycles | Cuts during a write (record / header / log) | Failures |
|---|---|---|---|---|---|
| `power-unit1` | the build: three parts 4 MiB apart | the sector | 120 | 16 / 3 / 5 | **0** |
| `power-unit64` | the build | whole 32 KiB units | 100 | 18 / 6 / 2 | **0** |
| `power-compact-unit64` | the three parts packed together (`FR_SD_SPREAD=0`) | whole 32 KiB units | 60 | 9 / 3 / 2 | 11: both copies lost |
| old layout, copy 0 next to the log, copy 1 4 MiB away | (first draft) | whole 32 KiB units | 60 | | 2: came back with the older copy |
| `power-fwrite-unit1` | f_write + f_sync | the sector | 24 | | from cycle 10: FLIGHT.DAT lost |

The other cuts landed between writes (a random moment, or after the boot's
last write); 17 of the 280 cycles of the first two series died during boot,
before the sender had even connected, and the next boot still came up.

- **Sector tears, the build (the requirement):** every boot reported the
  newest valid copy, never one older than the card had finished, and the card
  mounted with FLIGHT.DAT every time. Three times the torn write had got the
  record's 56 bytes down before the cut (a "partial" tear past them), and the
  boot rightly took that newer copy. A torn log sector left up to 12 events
  unreadable (`[EVENT:?|...]`), never more than its sector.
- **Unit tears:** with the parts 4 MiB apart nothing changed. Packed
  together, a torn write of the header, the log or a record took the whole
  unit and with it both copies (11 of 60); the first draft, with copy 0
  sharing the log's unit, came back with the older copy twice in 60. That is
  why the build has three parts 4 MiB apart.
- **f_write + f_sync:** in cycle 10 the cut fell while f_sync wrote the root
  directory sector (sector 8192), which came back erased. The next boot found
  no FLIGHT.DAT, made a new one and started from nothing (`boot=1`, `Idle`):
  the record and the log were gone, and on a real card that sector also holds
  the entries of the G-code files beside it. (The later cycles of that series
  don't count: the file had moved.)

## Long card pauses (`sd_evidence.py pause`)

A dense job over WebSocket (0.3 mm segments at 2000 mm/min, a line every
9 ms) for 20 s, with 10 % of card writes pausing 200–250 ms. The sender asks
for a status report every 50 ms and times the reply; on the board the
WebSocket is the W5500, polled by grblHAL's foreground like here.

| | raw sectors (the build) | raw, no pauses | f_write + f_sync |
|---|---|---|---|
| Card writes in the job | 113 | 130 | 8, then the machine hung |
| Long pauses (≥ 100 ms) | 16, up to 248 ms | 0 | 1, 216 ms |
| Disk calls that waited for a busy card | **0** | 0 | 8, up to 222 ms |
| Realtime poll gap, longest (`$FLIGHTSD gap_max`) | 10 ms | 10 ms | — (hung) |
| Status reply, p50 / p99 / max | 23 / 47 / 47 ms | 23 / 37 / 52 ms | 23 / 30 / 325 ms |
| Machine idle mid-job | 0 | 0 | **359: hung at line 2** |
| Record on the card, longest between updates | 345 ms | 118 ms | — |
| Records replaced in RAM before they went out | 15 | 0 | — |

With raw writes a 250 ms pause only makes the record on the card older (by
up to one pause: 345 ms instead of the 100 ms interval); the newest record
replaces the one waiting in RAM and nothing else waits.

With f_write and f_sync the job hung at line 2, three runs out of three
(with the card's pauses turned off, the same build streams the whole job). A
216 ms wait for the card, inside the realtime poll, emptied the stepper's
segment buffer (10 segments of at most 10 ms with grblHAL's defaults, which
the PrintNC builds use). On an empty segment buffer grblHAL's stepper
interrupt takes the motion as finished and flags the cycle complete
(`stepper.c`), so the state went to Idle with a block still in the planner.
grblHAL was inside `protocol_buffer_synchronize()`, waiting for line 2's
rapid before line 3's `M3`; that loop runs while there is a block or a cycle,
and only tries to start the cycle on entry, so it waited for ever: the
machine stopped mid-move, status reports still answered, no more lines read.
On a real machine that is a stop mid-cut with the spindle on, until cycle
start or reset.

## An EMI storm (`sd_evidence.py storm`)

With the machine idle, feed hold and cycle start pressed in turn 10 ms apart
(1,000 presses, so each is an event of its own), with a garbled line every
40. Then alarm 9 is raised and the power cut 50 ms after grblHAL reports it;
then, after a reboot, an error reply and a cut 50 ms later.

| | batched (the build) | every event at once |
|---|---|---|
| Events in the log afterwards (burst, alarm, boots) | 857 | 883 |
| Card writes for the burst | **102** | 1,078 |
| Events dropped (`STORAGE`) | 0 | 0 |
| Alarm, cut 50 ms later: on the card | yes | yes |
| Error reply, cut 50 ms later: on the card | no (batched, as decided) | yes |
| Log after the cuts | readable, in order | readable, in order |

## Focused scenarios on the SD card

All 13 of `tools/focused.py`'s scenarios pass with `--sd` (the simulator
with no FRAM, recording on its card): dwell-descend, numbered-jog,
partial-line, resume-power, reconnect, reset-control, estop-retract,
reset-retract, g20, g91, check, m0 (at `--speed 1`, see below) and
hold-retract. `results/sd/focused-sd.json`.

## A whole job: power cut and resume from the SD record

`tools/evidence.py power --sd` on the 1 MHz build, the FreeCAD pocket from
`programs/` over WebSocket: the controller killed mid-cut at 30 %. The next
boot found `Stopped`, `why=reboot`, line 1172, logged
`SPINDLE_OFF|after=job`, and the resume from line 1172 ran to the end:
`Completed`, line 4255, no errors or alarms. (The October FRAM run of the same
scenario stopped on the same line.) `results/sd/resume-power-sd.json`.

## Found on the way

- **grblHAL's waits.** `hal.delay_ms()` (dwells, spin-up, and 100 ms of its own
  after an alarm) runs `on_execute_delay`, not the realtime poll. On FRAM it
  doesn't matter, the write has happened; an SD card's writes wait for the
  poll, so they stalled for the wait. The plugin now polls the storage from
  `on_execute_delay` too (every build with SD card support; watchdog builds
  already hooked it to feed the watchdog).
- **The simulator's delays** spun on the counter without calling
  `on_execute_delay`, unlike the board's driver, so plugins and foreground
  tasks stopped in every wait. Fixed in the simulator (`d96e11c`).
- **Sticky error replies in the simulator.** After one error reply, every
  later line gets the same error code, valid lines included (`G0X1X2` gives
  error 25, then `M100` and `G1X1` do too). The same with the v0.3.0 plugin and
  with no plugin at all, so it is the simulator or this core, not the
  recorder; the October `events` run shows it too ("four garbled lines, one
  entry"). Not chased here.
- **`m0` is timing-sensitive** at 4× speed: after the cycle start the 0.5 s
  stream timeout is 125 ms of wall time, and on a loaded host the test's next
  line comes later. It failed 2 of 4 times with the v0.3.0 plugin as well, and
  passes at `--speed 1` with both.
- **The FRAM path is unchanged:** all 13 focused scenarios pass on the file
  stand-in (`m0` at `--speed 1`), and v0.3.0's settings load unchanged in
  v0.4.0 (`$794` comes up 0).

## Not covered here

- Real cards: latency, pauses, what a power cut really damages (pages paired
  with earlier writes, the card's mapping tables), cards that lie about
  finishing a write.
- The W5500 really sharing the SPI bus with the card, and Ethernet under load.
- A job run from the same SD card (`$F=`, stamped with `$FLIGHT=`) while the
  recorder writes: in the simulator the job files are a separate host
  directory. On the board the job's FatFs reads wait in `wait_ready()` while
  the card finishes a recorder write: usually 1–5 ms, harmless. A read that
  lands on a long pause (100 ms or more) can empty the segment buffer, so the
  feed stops for that long mid-cut and goes on: a witness mark, not the hang
  above, since the reads happen in the main loop rather than inside a
  synchronize wait. How often depends on the card's pauses. If hardware test
  7 shows it, the choice of what to do is the owner's (no record writes to
  the card during a job from that card, or only on state changes, or a card
  rule).
- Anything else using the card during a job (an FTP upload, a WebUI file
  list) already holds grblHAL the same way without the recorder, and could
  hit the hang if it lands in a synchronize wait.
- FLIGHT.DAT being made (the first boot, through FatFs): a power cut then can
  damage the FAT or directory like any FatFs write.
- Swapping cards with the board on.

## Hardware tests owed (the owner's)

On a Flexi-HAL with the uFlexiNET and the stock EEPROM at U27 (so no FRAM),
flashing `FLEXI_HAL_PRINTNC_ETH_SD_FR_SD[_WD]` (not the plain `_FR` builds,
which take the chip at U27 for FRAM). Cards, at least one of each:

| Class | Cards | Why |
|---|---|---|
| Industrial, pSLC, power-fail protected | Swissbit S-56u (or S-50u), Kingston Industrial (SDCIT2), SanDisk Industrial (SDSDQAF3) | what we'd recommend |
| Consumer high endurance | Samsung PRO Endurance, SanDisk High/MAX Endurance, Kingston High Endurance | what people will actually buy |
| Ordinary consumer | SanDisk Ultra, Samsung EVO Select | the common case |
| Cheap | an unbranded / multipack card | the worst case |

Card sizes 8–32 GB (FAT32) and one 64 GB (exFAT).

1. **Comes up on SD.** `$I` shows `Flight Recorder (SD card) v0.4.0` and
   `[NVS STORAGE:*EEPROM]`; FLIGHT.DAT is on the card (8 MB, hidden);
   `$FLIGHTSD` shows `apart=8192` or more and `ok`. Settings stick: change a
   few (`$110`, a `G10 L2` offset, `$794`), power-cycle, read them back. With
   the FRAM board and a `_FR` build: `Flight Recorder v0.4.0`, `$794=0`,
   behaves as v0.3.0 (record every 20 ms, events at once, no `$FLIGHTSD`).
2. **Card latency, per card.** A 10-minute dense job (short segments) over
   the network with `$FLIGHTSD=R` first; afterwards note `busy_avg`,
   `busy_max`, `pauses`, `failed`, `blocked_max_us`, `gap_max`. Reject a card
   with `failed > 0` or pauses over 250 ms.
3. **Ethernet keeps working.** During (2): no stutter, the sender's status
   reports keep coming, `gap_max` in single-digit ms, no dropped connection.
4. **Power yanks, 20 per card** (pull the 24 V and USB together) at random
   points mid-job; at least 5 of them within a second of a status change
   (start, alarm, feed hold). After each: `$FLIGHT` is `Stopped`,
   `why=reboot`, `ln` within a few lines behind where it was cutting, never
   ahead; `$EVENTS=100` readable (at most one run of up to 32
   `unreadable`); `$I` still `(SD card)`. Then resume the job from the
   record once in five.
5. **After the yank series, the card in a PC:** a file system check
   (`chkdsk X:` read-only, or `fsck.vfat -n`), and a few G-code files put on
   the card beforehand still match their checksums.
6. **First boot yanks**, on a blank card, three times: power off within a
   second of the first boot (while FLIGHT.DAT is being made through FatFs).
   The next boot must still come up on SD (or remake the file); then check
   the card as in (5).
7. **Job from the same card.** `$F=` a G-code file from the card with the
   job stamped (`$FLIGHT=<id>`): the job runs without pauses or starving
   (the file reads wait while the card finishes a recorder write).
8. **FTP while running.** Upload a file over FTP during a job; FLIGHT.DAT
   can't be deleted over FTP (read only).
9. **Card pulled with the board idle:** `$FLIGHTSD` goes `failing` and
   nothing hangs; put it back and reboot.
10. **Watchdog build:** `$WATCHDOGTEST` with the spindle on: reset,
    `BOOT|reset=watchdog`, the record `Stopped`/`reboot`.

Reproduce (WSL): build the variants with `tools/build_sim.sh ~/fr-sd/sim
~/fr-sd/builds`, then e.g. `tools/sd_evidence.py power --sim
~/fr-sd/builds/sd/grblHAL_flexihal_sim --run /tmp/power --cycles 120`,
`... pause ...`, `... storm ...`, `tools/focused.py <scenario> --sd ...`,
`tools/evidence.py power --sd --sim ~/fr-sd/builds/sd-1mhz/... --program
evidence/programs/freecad_fcL_spindle.nc ...`.
