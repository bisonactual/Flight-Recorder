/*

  storage.c - picks the flight recorder's storage

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
  FRAM if it is fitted, as before. Otherwise the SD card, if the build has SD
  card support and a card is in. Otherwise no storage, and the recorder only
  does the stream loss retract.
*/

#include "driver.h"

#if FLIGHT_RECORDER_ENABLE

#include <stddef.h>

#include "storage.h"

static const fr_backend_t *backend = NULL;

bool fr_storage_init (bool late)
{
#ifdef GRBL_SIM
    const fr_backend_t *candidate = late ? NULL : &fr_storage_file;
#else
    const fr_backend_t *candidate = late ? NULL : &fr_storage_fram;
#endif

#if FR_SD_ENABLE
    if(late)
        candidate = &fr_storage_sd;
#endif

    if(backend == NULL && candidate && candidate->init())
        backend = candidate;

    return backend != NULL;
}

const fr_storage_info_t *fr_storage_info (void)
{
    return &backend->info;
}

bool fr_storage_read (uint32_t addr, void *data, uint32_t size)
{
    return backend && backend->read(addr, data, size);
}

bool fr_storage_write (uint32_t addr, const void *data, uint32_t size)
{
    return backend && backend->write(addr, data, size);
}

void fr_storage_flush (void)
{
    if(backend && backend->flush)
        backend->flush();
}

void fr_storage_poll (void)
{
    if(backend && backend->poll)
        backend->poll();
}

bool fr_storage_report (bool reset)
{
    if(!(backend && backend->report))
        return false;

    backend->report(reset);

    return true;
}

#endif // FLIGHT_RECORDER_ENABLE
