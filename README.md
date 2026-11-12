# Flight-Recorder

This project adds a flight recorder to GRBLHAL to allow for safe project stops and resumes in the event of communication failure between sender and controller, or controller freeze. It is currently working on PNC's, but the below warnings apply. Please use the I2C fix branch firmware and report any issues. 

It is supported by [Bearsender](https://bisonactual.github.io/bearsender/bearsender.html) and will be supported by Bearcam on release, along with the full scurve and lookahead functions which are not yet available.

This addon is highly experimental, needs either a hardware change (the FRAM) or the uFlexiNET module's SD card, and only currently supports the Flexihal by Expatria (the A7 with the FRAM, or the A6/A7 with the uFlexiNET for the SD card). It very likely voids any warranty or customer support from Expatria. Use at your own risk.

This is part one of my changes, and does not include a spool. It adds a flight recorder that saves the position of your program as it runs (the line being executed, written at most every 20 ms on FRAM or every 100 ms on the SD card), and in the event of communication loss with the sender, retracts and stops the spindle, and allows you to restore it from a supported sender (currently my fork of flexisender.... soon) - it also adds an event log to diagnose EMI etc.

Watchdog builds should additionally reboot the board should it be frozen. I suggest 5 seconds as the default setting. You need a watchdog enabled chip to use watchdog functions.

$EVENT brings up the event log (you can use $events=n for n records)

$FLIGHT shows the current flight record: state, why it stopped, line, tlo, wcs, etc 

$FLIGHTCLEAR clears it

$EVENTSCLEAR clears the event log

$WATCHDOGTEST makes the board freeze so you can test it with a watchdog build (only works in idle or alarm)

$FLIGHTSD shows how the SD card is doing: writes, how long they take, long pauses, failures (SD card storage only; $FLIGHTSD=R starts the counts again)

$794 is how often the record is written while the line changes, in ms. 0 (the default) means the storage's own: 20 ms on FRAM, 100 ms on the SD card

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

### No FRAM? The uFlexiNET's SD card

If the board has the uFlexiNET module (network and micro-SD, on the 40-pin header), the recorder can keep its record on the SD card instead, no soldering. Flash one of the **`_FR_SD`** builds (`FLEXI_HAL_PRINTNC_ETH_SD_FR_SD_...uf2`, `..._4X_FR_SD_...`, `_WD` for the watchdog) and put a card in. `$I` shows `[PLUGIN:Flight Recorder (SD card) v0.4.0]`.

Use the `_FR_SD` builds on a board with the stock EEPROM at U27, never the plain `_FR` ones: those treat the chip at U27 as FRAM and skip the EEPROM's write delays, so grblHAL's own settings wouldn't save properly. (A `_FR_SD` build on a board with the FRAM works too: it uses the FRAM, and settings just save a little slower.)

- At the first boot the recorder makes `FLIGHT.DAT` on the card (8 MB, hidden, read only) and from then on writes its sectors directly, never through the file system. Your G-code files on the card are left alone.
- The record is written at most every 100 ms while the line changes (FRAM: 20 ms, `$794` to change it). Events are written about once a second; alarms, a lost link, resets and stream timeouts at once. So a power cut can lose the last tenth of a second of record and the last second of minor events (an error reply, say).
- It never waits for the card. Cards pause now and then (up to 250 ms is allowed), and that doesn't hold up the machine or the network, which shares the card's bus.
- Use a good card: an industrial or high-endurance one (pSLC, with power-loss protection if you can get it), not a cheap one. A cheap card losing power mid-write can damage more than the sector it was writing. FRAM is still the most robust option.
- Don't swap the card with the board on: the recorder writes where `FLIGHT.DAT` was on the old card. Change cards, then reboot.
- `$FLIGHTSD` shows how the card is doing; `apart=` should be 8192 or more (the two copies of the record and the log are 4 MB apart on the card, so a card that damages more than it was writing can't take them all).

So far this has been tested in the simulator only (`evidence/SD.md`); hardware tests are next.


In detail:

A grblHAL plugin that keeps a record of the running job in non-volatile memory
(the Flexi-HAL A7's I2C FRAM, or the SD card in the uFlexiNET module), so a sender can resume after the PC, the link or
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
  On FRAM each event is written as it happens; on the SD card they are written
  about once a second, and alarms, lost links, resets and stream timeouts at once.
  
  If the controller itself hangs, the
  hardware watchdog resets it after `$793` seconds (3–10, default 5), which
  stops the spindle; the board waits in its homing alarm and the job can be
  resumed. Without it a hung controller keeps the spindle turning until
  someone presses reset.


## Supported hardware

 - **Flexi-HAL Rev A7** with the EEPROM at **U27** replaced by I2C FRAM (I used Infineon, 1 Mbit / 128 KB I2C FRAM, SOIC-8). You must use a watchdog enabled chip to use the watchdog functions.
  - grblHAL's settings keep the first 16 KB, as before; the recorder uses the
    space after them.
  - At start-up the recorder checks the chip is that FRAM and leaves it alone
    otherwise: a 16 KB part would have its settings overwritten, and an EEPROM
    would wear out.
 - Or **Flexi-HAL A6/A7 with the uFlexiNET module** and a micro-SD card in it, with
   one of the `_FR_SD` builds: used when there is no FRAM (above).
 - Neither: the recorder reports `Flight Recorder (no storage)` and only does the
   stream loss retract.
- **Leave MPG off**
- Connection over **USB** or the **uFlexiNET** network module works

## Layout

- plugin/                 the grblHAL plugin which contains the plugin
- core/                   grblHAL core patches
- firmware/               board firmware: Expatria's builds plus the recorder
- tools/                  test sender, simulator harness, evidence scripts
- evidence/               simulator runs: programs, results, plots
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
