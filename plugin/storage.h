/*

  storage.h - the flight recorder's storage backends

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

#ifndef _FR_STORAGE_H_
#define _FR_STORAGE_H_

#include "flight_recorder.h"
#include "event_log.h"

typedef struct {
    fr_storage_info_t info;
    bool (*init)(void);
    bool (*read)(uint32_t addr, void *data, uint32_t size);
    bool (*write)(uint32_t addr, const void *data, uint32_t size);
    void (*flush)(void);        // optional
    void (*poll)(void);         // optional
    void (*report)(bool reset); // optional, $FLIGHTSD
} fr_backend_t;

extern const fr_backend_t fr_storage_fram;     // storage_fram.c, the board
extern const fr_backend_t fr_storage_file;     // storage_file.c, the simulator's stand-in for the FRAM
extern const fr_backend_t fr_storage_sd;       // storage_sd.c

// The FRAM's layout, which the simulator's file shares: the record copies 64 bytes apart
// from 0, the log header at FR_LOG_BASE with the events right after it, and every write
// straight through.
#define FR_FRAM_INFO { \
    .label = NULL, \
    .record_stride = 64, \
    .log_header = FR_LOG_BASE, \
    .log_entries = FR_LOG_BASE + FR_LOG_HEADER_SIZE, \
    .log_slots = FR_LOG_EVENTS, \
    .record_interval = 20, \
    .batch_events = false \
}

#endif
