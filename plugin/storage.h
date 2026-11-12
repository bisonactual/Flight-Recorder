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

// From the board file: is the SD card still programming? Must not wait.
bool fr_sd_card_busy (void);

#endif
