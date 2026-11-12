/*

  storage_file.c - flight recorder storage in a file, for the simulator

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

/*
  FLIGHT.DAT in the simulator's working directory stands in for the FRAM, with
  the same layout. Every write is flushed straight away, so it survives the
  simulator being killed, which is how the evidence runs cut the power.

  With -F the simulator has no FRAM fitted, and the recorder uses the SD card
  (-d) instead.
*/

#include "driver.h"

#if FLIGHT_RECORDER_ENABLE

#include <stdio.h>
#include <string.h>

#include "storage.h"
#include "event_log.h"

#define STORAGE_FILE "FLIGHT.DAT"

static FILE *file = NULL;

static bool file_init (void)
{
    extern bool sim_fram_fitted (void);

    if(!sim_fram_fitted())
        return false;

    if(!(file = fopen(STORAGE_FILE, "r+b")))
        file = fopen(STORAGE_FILE, "w+b");

    return file != NULL;
}

// Anything never written reads back as 0xFF, like erased storage.
static bool file_read (uint32_t addr, void *data, uint32_t size)
{
    if(!file || fseek(file, (long)addr, SEEK_SET))
        return false;

    size_t count = fread(data, 1, size, file);
    if(count < size)
        memset((uint8_t *)data + count, 0xFF, size - count);

    return true;
}

static bool file_write (uint32_t addr, const void *data, uint32_t size)
{
    bool ok = file &&
               fseek(file, (long)addr, SEEK_SET) == 0 &&
                fwrite(data, 1, size, file) == size &&
                 fflush(file) == 0;

    fr_log_storage_result(ok);

    return ok;
}

const fr_backend_t fr_storage_file = {
    .info = {
        .label = NULL,
        .record_stride = 64,
        .log_header = FR_LOG_BASE,
        .log_entries = FR_LOG_BASE + 16,
        .log_slots = FR_LOG_EVENTS,
        .record_interval = 20,
        .batch_events = false
    },
    .init = file_init,
    .read = file_read,
    .write = file_write
};

#endif // FLIGHT_RECORDER_ENABLE
