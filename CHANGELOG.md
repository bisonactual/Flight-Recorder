# Changelog

## 0.4.0-sd (release 0.3.0-sd-card, 13 Nov 2026)

Flight recorder builds for a Flexi-HAL without the FRAM: the record and the event log go on the micro-SD card in the uFlexiNET module, so no soldering is needed. These are Expatria's printnc_eth_sd builds with my changes, from the sd-backend branch. They've been tested in the simulator only, not on a machine yet.

Use these only on a board with the stock EEPROM at U27. If you've fitted the FRAM, use the 0.4.1 release: these builds don't have its I2C fixes or the new rapid override levels.

What's in it:
- **Record and event log on the SD card.** The recorder makes FLIGHT.DAT (8 MB, hidden, read only) at the first boot and writes its sectors directly from then on, so it never touches your other files or the card's FAT. `$I` shows `Flight Recorder (SD card) v0.4.0-sd`.
- **It never waits for the card.** Cards pause for up to 250 ms after a write now and then; the machine and the network carry on regardless.
- **Two copies of the record, 4 MB apart**, so a power cut mid-write always leaves the last good one.
- **The record is written at most every 100 ms** while the line changes. Events go to the card about once a second; alarms, a lost link, resets and stream timeouts at once.
- **`$794`** sets the record interval in ms. 0, the default, means 100 ms on the SD card.
- **`$FLIGHTSD`** shows how the card is doing: writes, how long they take, long pauses, failures.
- `$FLIGHTCLEAR` and `$EVENTS` are refused while a job runs, as in 0.4.1.
- If the card is unmounted (`$FU`) the recorder stops writing to it, and after `$FM` it carries on only if it's the same card. Don't swap cards with the board on; restart after changing cards.

Use a good card, industrial or high endurance, not a cheap one.

WD = watchdog on. 4X = 4 axes.
