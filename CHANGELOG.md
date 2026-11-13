# Changelog

## 0.4.1 (release 0.3.0, 13 Nov 2026)

These are the standard builds from Expatria with my changes, from main. They need the FRAM at U27. They haven't been tested on a machine yet.

What's new since 0.2.0:
- **The I2C fixes are now on main.** Everything from the 0.2.0-i2c-overrides release: FRAM writes no longer fail next to the keypad's I2C traffic, the keypad reads keys from the main loop, a failed write is retried once, and STORAGE events come at most once a minute.
- **Rapid override levels** 75% (`0xAE`) and 10% (`0xAF`), as in 0.2.0.
- **`$FLIGHTCLEAR` is refused while a job runs or retracts** (error:8). Before, clearing mid-job wiped the record and turned off the stream-loss retract for the rest of the job.
- **`$EVENTS` is refused while the machine moves** (error:8). Reading the log back could hold the controller up mid-cut. It still answers when idle and in an alarm.
- Settings, the job record and the event log carry over from 0.3.0 and 0.4.0.

WD = watchdog on.

NOKEYPAD = the same build without the I2C keypad plugin. It's only for diagnosis.
