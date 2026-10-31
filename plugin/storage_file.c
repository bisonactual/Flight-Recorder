/*
  storage_file.c — Flight recorder storage in a host file (simulator).

  FLIGHT.DAT in the simulator's working directory stands in for the FRAM.
  Every write is flushed to the operating system at once, so it survives the
  simulator being killed, which is how the evidence runs cut the power.
*/

#include <stdio.h>
#include <string.h>

#include "flight_recorder.h"
#include "event_log.h"

#define FR_FILE "FLIGHT.DAT"

// A simulated processor has no reset cause and no watchdog.
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

static FILE *file = NULL;

bool fr_storage_init (void)
{
    if(!(file = fopen(FR_FILE, "r+b")))
        file = fopen(FR_FILE, "w+b");

    return file != NULL;
}

// Bytes never written read as 0xFF, like erased storage.
bool fr_storage_read (uint32_t addr, void *data, uint32_t size)
{
    if(!file || fseek(file, (long)addr, SEEK_SET))
        return false;

    size_t n = fread(data, 1, size, file);
    if(n < size)
        memset((uint8_t *)data + n, 0xFF, size - n);

    return true;
}

bool fr_storage_write (uint32_t addr, const void *data, uint32_t size)
{
    return file &&
            fseek(file, (long)addr, SEEK_SET) == 0 &&
             fwrite(data, 1, size, file) == size &&
              fflush(file) == 0;
}
