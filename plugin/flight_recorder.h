/*

  flight_recorder.h - job record and stream loss retract

  Part of the Flight-Recorder plugin for grblHAL

  Copyright (c) 2026 bisonactual

  Flight-Recorder is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Flight-Recorder is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Flight-Recorder. If not, see <http://www.gnu.org/licenses/>.

*/

#ifndef _FLIGHT_RECORDER_H_
#define _FLIGHT_RECORDER_H_

#include <stdint.h>
#include <stdbool.h>

#define FR_VERSION     "0.4.0"
#define FR_MAGIC       0x31524646u  // "FFR1", record layout version 1
#define FR_AXES        5

// SD card storage, through grblHAL's SD card support. Only for cards on SPI.
#ifndef FR_SD_ENABLE
#if defined(GRBL_SIM)
#define FR_SD_ENABLE 0              // the simulator's build sets it when it has its card model
#elif SDCARD_ENABLE && !SDCARD_SDIO
#define FR_SD_ENABLE 1
#else
#define FR_SD_ENABLE 0
#endif
#endif

typedef enum {
    FR_Idle = 0,        // no job recorded, or cleared
    FR_Running,         // job in progress
    FR_Stopped,         // job interrupted, reason says why
    FR_Retracted,       // stream lost, tool retracted and spindle stopped
    FR_Completed        // program end reached
} fr_state_t;

typedef enum {
    FR_Reason_None = 0,
    FR_Reason_StreamTimeout,    // no input for the timeout with the spindle on
    FR_Reason_UsbClosed,        // the sender closed the USB port (DTR dropped), or the input moved to another stream
    FR_Reason_NetworkClosed,    // the network client went away or was replaced
    FR_Reason_Alarm,            // an alarm, see the alarm field
    FR_Reason_Reset,            // soft reset
    FR_Reason_Reboot            // the controller restarted with the job running
} fr_reason_t;

typedef union {
    uint8_t value;
    struct {
        uint8_t homed      :1,  // Z was homed when written
                spindle_on :1,
                coolant_on :1,
                no_retract :1,  // the job opted out of the auto retract
                usb        :1,  // the job was streamed over USB
                network    :1,  // the job was streamed over the network
                unused     :2;
    };
} fr_flags_t;

typedef struct {
    uint32_t magic;
    uint32_t seq;               // write counter, the newer valid copy wins
    uint32_t boot;              // controller boot count when written
    uint32_t job;               // job id from the sender, 0 if none
    uint32_t line;              // N-number of the line executing, 0 if none yet
    float    mpos[FR_AXES];     // machine position, mm
    int32_t  tool;
    uint8_t  state;             // fr_state_t
    uint8_t  reason;            // fr_reason_t
    uint8_t  alarm;             // alarm code when the reason is Alarm
    uint8_t  wcs;               // 0 = G54 ... 5 = G59
    fr_flags_t flags;
    uint8_t  reserved[3];
    uint32_t crc;               // CRC-32 of everything above
} fr_record_t;

// Storage, a byte addressed area that belongs to the recorder. The layout, and how
// often things are written, depend on the storage.
typedef struct {
    const char *label;          // added to the plugin's name in $I, NULL for none
    uint32_t record_stride;     // the second copy of the record is this far after the first
    uint32_t log_header;        // event log header
    uint32_t log_entries;       // first event slot
    uint16_t log_slots;         // slots in the event ring, FR_LOG_EVENTS or more
    uint16_t record_interval;   // default ms between record writes while the line changes
    bool batch_events;          // events wait in RAM and go out about once a second
} fr_storage_info_t;

// Picks the storage, FRAM if it is fitted, otherwise an SD card. FRAM is checked at
// start-up (late false), the SD card once grblHAL has mounted it (late true).
bool fr_storage_init (bool late);
const fr_storage_info_t *fr_storage_info (void);
bool fr_storage_read (uint32_t addr, void *data, uint32_t size);
// Doesn't wait for the storage. An SD card does the write later, in the background.
bool fr_storage_write (uint32_t addr, const void *data, uint32_t size);
// Sends whatever an SD card has waiting in RAM. FRAM writes have already gone.
void fr_storage_flush (void);
// Background work, from the realtime poll.
void fr_storage_poll (void);
// $FLIGHTSD, for an SD card: write counts and times. Returns false if the storage has
// nothing to report. reset starts the counts again.
bool fr_storage_report (bool reset);

const char *fr_state_name (uint8_t state);
const char *fr_reason_name (uint8_t reason);

void flight_recorder_init (void);

#endif
