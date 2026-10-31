/*
  storage_fram.c — Flight recorder storage in the I2C FRAM (Flexi-HAL A7), and
  the other board pieces the recorder needs: the reset cause and the watchdog.

  Supported hardware: an FM24V10 (128 KB FRAM) fitted in place of the EEPROM
  at U27, at 0x50, with the firmware built with EEPROM_IS_FRAM=1. grblHAL's
  settings use the first 16 KB (the Flexi-HAL map forces EEPROM_ENABLE 128);
  the recorder sits just after them, inside the first 64 KB so plain 2-byte
  word addresses reach it. FRAM writes complete at bus speed, with no page
  limits, write delay or meaningful wear.

  Before using it, init checks that the chip really is that FRAM:
    - a 16 KB part (the stock EEPROM) wraps 0x4000 onto 0x0000: the recorder
      would overwrite grblHAL's settings. The two must differ; when both read
      the same and are blank, a one-byte write at 0x4000 tells;
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

static bool is_blank (const uint8_t *data, uint32_t size)
{
    while(--size)
        if(data[size] != data[0])
            return false;

    return true;
}

static bool is_fram (void)
{
    uint8_t low[16], high[16], probe;

    if(!(transfer(0, low, sizeof(low), true) && transfer(FR_FRAM_BASE, high, sizeof(high), true)))
        return false;

    // The same data in both: a 16 KB part wrapping, or a chip still blank. Only a
    // blank one is written to: change 0x4000 and see whether 0x0000 follows.
    if(memcmp(low, high, sizeof(low)) == 0) {
        if(!is_blank(low, sizeof(low)))
            return false;
        uint8_t mark = ~low[0], seen = low[0];
        bool wraps = !(transfer(FR_FRAM_BASE, &mark, 1, false) && transfer(0, &seen, 1, true)) || seen == mark;
        transfer(FR_FRAM_BASE, &low[0], 1, false);
        if(wraps)
            return false;
    }

    return transfer(FR_FRAM_BASE + FR_FRAM_PROBE, &probe, 1, true) &&
            transfer(FR_FRAM_BASE + FR_FRAM_PROBE, &probe, 1, false) &&
             i2c_probe(FR_FRAM_ADDRESS);         // EEPROM: busy (no ACK) after a write
}

bool fr_storage_init (void)
{
    return i2c_start().ok && i2c_probe(FR_FRAM_ADDRESS) && is_fram();
}

static uint32_t csr;

// The bootloader build's main() calls HAL_RCC_DeInit(), which clears the reset
// flags before any plugin starts: read them first, while the C runtime starts up.
__attribute__((constructor)) static void read_reset_flags (void)
{
    csr = RCC->CSR;
}

// Why the STM32 last reset, from its reset flags, which are then cleared so the
// next boot reports its own. A power-on also sets brown-out and pin; a brown-out,
// watchdog or software reset also sets pin; the pin alone is the reset button,
// or noise on the NRST line (it is on the Pi header, pin 11).
uint8_t fr_reset_cause (void)
{
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

// The independent watchdog runs from the ~32 kHz LSI clock (17-47 kHz across parts and
// temperature, so the timeout is approximate). Prescaler 256 gives 125 ticks a second;
// the 12-bit reload allows up to about 32 s. It keeps counting while a debugger halts
// the processor unless frozen, so freeze it.
void fr_watchdog_start (uint32_t seconds)
{
    uint32_t reload = seconds * 125u;

    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;

    IWDG->KR = 0xCCCC;                  // start
    IWDG->KR = 0x5555;                  // unlock the prescaler and reload registers
    IWDG->PR = 6;                       // divide by 256
    IWDG->RLR = reload > 0xFFF ? 0xFFF : reload;
    while(IWDG->SR);                    // wait for the new values to take
    IWDG->KR = 0xAAAA;                  // reload
}

void fr_watchdog_feed (void)
{
    IWDG->KR = 0xAAAA;
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
