# Flight-Recorder

This addon is highly experimental, requires hardware changes, and only currently supports the Flexihal-A7 by Expatria. It very likely voids any warranty or customer support from Expatria. Use at your own risk.

This is part one of my changes, and does not include a spool. It adds a flight recorder that saves the position of your program every move, and in the event of communication loss with the sender, retracts and stops the spindle, and allows you to restore it from a supported sender (currently my fork of flexisender.... soon) - it also adds an event log to diagnose EMI etc.

Watchdog builds should additionally reboot the board should it be frozen. I suggest 5 seconds as the default setting. You need a watchdog enabled chip to use watchdog functions.

$EVENT brings up the event log (you can use $events=n for n records)

$FLIGHT shows the current flight record: state, why it stopped, line, tlo, wcs, etc 

$FLIGHTCLEAR clears it

$EVENTSCLEAR clears the event log

$WATCHDOGTEST makes the board freeze so you can test it with a watchdog build (only works in idle or alarm)

Alarm 80 means the connection was lost mid-job. The tool was retracted and the spindle stopped, so you can resume from the flight record.

Alarm 81 means the connection was lost and the retract couldn’t finish, so the machine stopped where it was.

All PNC's have a Z limit but Z must be homed for retract to work

Set $790=0 for tslots and lollipops etc to disable retracts

If you want to test this out before using on a real flexihal, I've updated my simulator.

The process is below:

<img width="1280" height="1046" alt="image" src="https://github.com/user-attachments/assets/dfe2bff7-8b76-45b1-97ad-a023a0b371a7" />

To begin, you need to locate U27 (to the left of the USB, between the MCU and port) and remove the EEPROM. You must replace this chip with ferroelectric RAM, and it must be I2C. 
I used https://www.infineon.com/part/FM24V10-G which is a 1mbit chip, but 512kbit may work. SPI will not work. It is mostly a drag and drop replacement, and you put your FRAM where the EEPROM used to be.

You then need to install the firmware provided here. Then use flexisender and off you go.

Bish Bash Bosh. 

(Strongly suggest you bench test this instead of sticking it straight on a pnc. USE AT YOUR OWN RISK)


In detail:

A grblHAL plugin that keeps a record of the running job in non-volatile memory
(the Flexi-HAL A7's I2C FRAM), so a sender can resume after the PC, the link or
the controller has gone away, and that gets the tool out of the cut when the
stream stops. Open source (GPLv3, like grblHAL).

  It records the N-number of the line the steppers are executing
  (never ahead of the machine), machine position, tool, work offset, and why
  the job stopped: program end, alarm, reset, reboot, or the stream going away.
  
  With the spindle on, if the sender stops feeding the
  job (timeout) or its link goes (USB port or network client closed, or the
  input taken over), the board blocks all input, lets the executing line
  finish, retracts straight up to machine Z −20, stops the spindle, records it
  and raises alarm 80. A retract that can't finish stops the machine (alarm 81).
  
  To resume the sender reads `$FLIGHT`, rebuilds the modal state up to the
  recorded line, re-enters straight down onto its start and streams on.
  
  The event log records the last 1000 events across reboots (`$EVENTS`): resets and
  their cause (brown-out, reset pin, watchdog), alarms, error replies, inputs
  and limit switches tripping, the sender's link coming and going, and Modbus
  timeouts and CRC errors. Aimed at the "silent" and annoying faults EMI causes.
  
  If the controller itself hangs, the
  hardware watchdog resets it after `$793` seconds (3–10, default 5), which
  stops the spindle; the board waits in its homing alarm and the job can be
  resumed. Without it a hung controller keeps the spindle turning until
  someone presses reset.


## Supported hardware

 - **Flexi-HAL Rev A7 ONLY** with the EEPROM at **U27** replaced by I2C FRAM (I used Infineon, 1 Mbit / 128 KB I2C FRAM, SOIC-8). This direct
  replacement is the only supported storage. You must use a watchdog enabled chip to use the watchdog functions.
  - grblHAL's settings keep the first 16 KB, as before; the recorder uses the
    space after them.
  - At start-up the recorder checks the chip is that FRAM and leaves it alone
    otherwise (it reports `Flight Recorder (no storage)`): a 16 KB part would
    have its settings overwritten, and an EEPROM would wear out.
- **Leave MPG off**
- Connection over **USB** or the **uFlexiNET** network module works

## Layout

 plugin/                 the grblHAL plugin
-  flight_recorder.c/h     record, stream-loss retract, $ commands, settings
-  event_log.c/h           the event log
-  storage_fram.c          the A7's FRAM (checked at start-up), reset cause, watchdog
-  storage_file.c          a host file (simulator)
-  library.json            PlatformIO library (FRAM storage only)
core/                   grblHAL core patches
-  executing-line.patch    report the line the steppers are executing
-  plugin-init.patch       start the recorder (firmware builds)
-  modbus-stats.patch      read the Modbus driver's error counters (firmware builds)
firmware/               board firmware: Expatria's builds plus the recorder
-  flexihal-a7/            Flexi-HAL A7 (STM32F446), UF2 for Expatria's bootloader
- docs/COMMANDS.md        $ commands, record, settings, alarm, how to resume
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
