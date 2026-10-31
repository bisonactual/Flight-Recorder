/*
  storage_fram.c — Flight recorder storage in the I2C FRAM (Flexi-HAL A7).

  Supported hardware: an FM24V10 (128 KB FRAM) fitted in place of the EEPROM
  at U27, at 0x50, with the firmware built with EEPROM_IS_FRAM=1. grblHAL's
  settings use the first 16 KB (the Flexi-HAL map forces EEPROM_ENABLE 128);
  the recorder sits just after them, inside the first 64 KB so plain 2-byte
  word addresses reach it. FRAM writes complete at bus speed, with no page
  limits, write delay or meaningful wear.

  Before using it, init checks that the chip really is that FRAM:
    - a 16 KB part (the stock EEPROM) wraps 0x4000 onto 0x0000: the recorder
      would overwrite grblHAL's settings. Read-only check: the two must differ;
    - an EEPROM is busy for ~5 ms after a write and wears out: rewrite one byte
      with its own value and probe at once. FRAM acknowledges, EEPROM doesn't.
  Either failing leaves the chip alone and the recorder without storage.
*/

#include "driver.h"

#if FLIGHT_RECORDER_ENABLE

#include <string.h>

#include "grbl/plugins.h"

#include "flight_recorder.h"
#include "event_log.h"

#define FR_FRAM_ADDRESS 0x50
#define FR_FRAM_BASE    0x4000
#define FR_FRAM_PROBE   0x3FF       // scratch byte for the write check, past the record slots

static bool transfer (uint16_t word_addr, void *data, uint32_t size, bool read)
{
    nvs_transfer_t i2c = {
        .address = FR_FRAM_ADDRESS,
        .word_addr_bytes = 2,
        .word_addr = word_addr,
        .count = size,
        .no_block = false,
        .data = (uint8_t *)data
    };

    return i2c_nvs_transfer(&i2c, read);
}

static bool is_fram (void)
{
    uint8_t low[16], high[16], probe;

    if(!(transfer(0, low, sizeof(low), true) && transfer(FR_FRAM_BASE, high, sizeof(high), true)))
        return false;

    if(memcmp(low, high, sizeof(low)) == 0)     // wraps: 16 KB part
        return false;

    return transfer(FR_FRAM_BASE + FR_FRAM_PROBE, &probe, 1, true) &&
            transfer(FR_FRAM_BASE + FR_FRAM_PROBE, &probe, 1, false) &&
             i2c_probe(FR_FRAM_ADDRESS);         // EEPROM: busy (no ACK) after a write
}

bool fr_storage_init (void)
{
    return i2c_start().ok && i2c_probe(FR_FRAM_ADDRESS) && is_fram();
}

// Why the STM32 last reset, from its reset flags, which are then cleared so the
// next boot reports its own. A power-on also sets brown-out and pin; a brown-out,
// watchdog or software reset also sets pin; the pin alone is the reset button,
// or noise on the NRST line (it is on the Pi header, pin 11).
uint8_t fr_reset_cause (void)
{
    uint32_t csr = RCC->CSR;
    uint8_t cause = 0;

    RCC->CSR |= RCC_CSR_RMVF;

    if(csr & RCC_CSR_PORRSTF)
        cause |= FR_Reset_PowerOn;
    if(csr & RCC_CSR_BORRSTF)
        cause |= FR_Reset_BrownOut;
    if(csr & RCC_CSR_PINRSTF)
        cause |= FR_Reset_Pin;
    if(csr & RCC_CSR_SFTRSTF)
        cause |= FR_Reset_Software;
    if(csr & (RCC_CSR_IWDGRSTF | RCC_CSR_WWDGRSTF))
        cause |= FR_Reset_Watchdog;
    if(csr & RCC_CSR_LPWRRSTF)
        cause |= FR_Reset_LowPower;

    return cause;
}

bool fr_storage_read (uint32_t addr, void *data, uint32_t size)
{
    return transfer((uint16_t)(FR_FRAM_BASE + addr), data, size, true);
}

bool fr_storage_write (uint32_t addr, const void *data, uint32_t size)
{
    return transfer((uint16_t)(FR_FRAM_BASE + addr), (void *)data, size, false);
}

#endif
