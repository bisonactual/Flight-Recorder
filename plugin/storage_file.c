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
  FLIGHT.DAT in the simulator's working directory stands in for the FRAM.
  Every write is flushed straight away, so it survives the simulator being
  killed, which is how the evidence runs cut the power.
*/

#include <stdio.h>
#include <string.h>

#include "flight_recorder.h"
#include "event_log.h"

#define STORAGE_FILE "FLIGHT.DAT"

static FILE *file = NULL;

// The simulated processor has no reset flags and no watchdog.
uint8_t fr_reset_cause (void)
{
    return 0;
}

void fr_watchdog_start (uint32_t seconds)
{
    (void)seconds;
}

void fr_watchdog_feed (void)
{
}

bool fr_storage_init (void)
{
    if(!(file = fopen(STORAGE_FILE, "r+b")))
        file = fopen(STORAGE_FILE, "w+b");

    return file != NULL;
}

// Anything never written reads back as 0xFF, like erased storage.
bool fr_storage_read (uint32_t addr, void *data, uint32_t size)
{
    if(!file || fseek(file, (long)addr, SEEK_SET))
        return false;

    size_t count = fread(data, 1, size, file);
    if(count < size)
        memset((uint8_t *)data + count, 0xFF, size - count);

    return true;
}

bool fr_storage_write (uint32_t addr, const void *data, uint32_t size)
{
    return file &&
            fseek(file, (long)addr, SEEK_SET) == 0 &&
             fwrite(data, 1, size, file) == size &&
              fflush(file) == 0;
}
