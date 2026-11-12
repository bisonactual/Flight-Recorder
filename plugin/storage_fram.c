/*

  storage_fram.c - flight recorder storage in the Flexi-HAL A7's I2C FRAM

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
  The only supported chip is an FM24V10 (128 KB FRAM) fitted in place of the
  EEPROM at U27, address 0x50, with the firmware built with EEPROM_IS_FRAM=1.
  grblHAL's settings use the first 16 KB (the Flexi-HAL map forces
  EEPROM_ENABLE 128) and the recorder goes right after them, still inside the
  first 64 KB so ordinary 2 byte word addresses reach it. FRAM writes finish at
  bus speed and there are no pages, write delays or wear to worry about.

  Before using the chip we check that it really is that FRAM:
    - A 16 KB part, like the stock EEPROM, wraps 0x4000 back onto 0x0000, and
      the recorder would overwrite the settings. So the two must read
      differently. If they read the same and are blank, a one byte test write
      at 0x4000 tells us.
    - An EEPROM is busy for about 5 ms after a write and wears out. We rewrite
      one byte with the value it already has and probe straight away. FRAM
      answers, EEPROM doesn't.
  If either check fails the chip is left alone and the recorder runs without
  storage.
*/

#include "driver.h"

#if FLIGHT_RECORDER_ENABLE

#include <string.h>

#include "grbl/plugins.h"

#include "storage.h"
#include "event_log.h"

#define FRAM_ADDRESS    0x50
#define FRAM_BASE       0x4000
#define FRAM_PROBE      0x3FF       // scratch byte for the write check, after the record slots

static bool transfer (uint16_t word_addr, void *data, uint32_t size, bool read)
{
    nvs_transfer_t i2c = {
        .address = FRAM_ADDRESS,
        .word_addr_bytes = 2,
        .word_addr = word_addr,
        .count = size,
        .no_block = false,
        .data = (uint8_t *)data
    };

    return i2c_nvs_transfer(&i2c, read);
}

static bool all_same (const uint8_t *data, uint32_t size)
{
    while(--size)
        if(data[size] != data[0])
            return false;

    return true;
}

static bool is_fram (void)
{
    uint8_t low[16], high[16], probe;

    if(!(transfer(0, low, sizeof(low), true) && transfer(FRAM_BASE, high, sizeof(high), true)))
        return false;

    // The same data in both places means either a 16 KB part that wraps or a chip that
    // is still blank. Only write to a blank one: change 0x4000 and see if 0x0000 follows.
    if(memcmp(low, high, sizeof(low)) == 0) {

        if(!all_same(low, sizeof(low)))
            return false;

        uint8_t mark = ~low[0], seen = low[0];
        bool wraps = !(transfer(FRAM_BASE, &mark, 1, false) && transfer(0, &seen, 1, true)) || seen == mark;

        transfer(FRAM_BASE, &low[0], 1, false);
        if(wraps)
            return false;
    }

    return transfer(FRAM_BASE + FRAM_PROBE, &probe, 1, true) &&
            transfer(FRAM_BASE + FRAM_PROBE, &probe, 1, false) &&
             i2c_probe(FRAM_ADDRESS);   // an EEPROM is busy after the write and doesn't answer
}

static bool fram_init (void)
{
    return i2c_start().ok && i2c_probe(FRAM_ADDRESS) && is_fram();
}

static bool fram_read (uint32_t addr, void *data, uint32_t size)
{
    return transfer((uint16_t)(FRAM_BASE + addr), data, size, true);
}

static bool fram_write (uint32_t addr, const void *data, uint32_t size)
{
    bool ok = transfer((uint16_t)(FRAM_BASE + addr), (void *)data, size, false);

    fr_log_storage_result(ok);

    return ok;
}

const fr_backend_t fr_storage_fram = {
    .info = FR_FRAM_INFO,
    .init = fram_init,
    .read = fram_read,
    .write = fram_write
};

#endif
