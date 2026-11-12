/*

  board_stm32f4.c - the board specific bits of the flight recorder, for the STM32F4xx driver

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
  The processor side of the recorder: the reset cause, the independent
  watchdog, and a check on whether the SD card is still busy that doesn't wait
  for it.
*/

#include "driver.h"

#if FLIGHT_RECORDER_ENABLE

#include "flight_recorder.h"
#include "event_log.h"

#if FR_SD_ENABLE
#include "spi.h"
#endif

static uint32_t reset_flags;

// With the bootloader, main() calls HAL_RCC_DeInit() which clears the reset flags
// before any plugin starts. So grab them while the C runtime starts up instead.
__attribute__((constructor)) static void save_reset_flags (void)
{
    reset_flags = RCC->CSR;
}

// Why the STM32 last reset. The flags are cleared afterwards so the next boot reports
// its own. A power-on also sets brown-out and pin, and a brown-out, watchdog or software
// reset also sets pin. Pin on its own is the reset button, or noise on the NRST line
// (which is on the Pi header, pin 11).
uint8_t fr_reset_cause (void)
{
    uint8_t cause = 0;

    RCC->CSR |= RCC_CSR_RMVF;

    if(reset_flags & RCC_CSR_PORRSTF)
        cause |= FR_Reset_PowerOn;
    if(reset_flags & RCC_CSR_BORRSTF)
        cause |= FR_Reset_BrownOut;
    if(reset_flags & RCC_CSR_PINRSTF)
        cause |= FR_Reset_Pin;
    if(reset_flags & RCC_CSR_SFTRSTF)
        cause |= FR_Reset_Software;
    if(reset_flags & (RCC_CSR_IWDGRSTF | RCC_CSR_WWDGRSTF))
        cause |= FR_Reset_Watchdog;
    if(reset_flags & RCC_CSR_LPWRRSTF)
        cause |= FR_Reset_LowPower;

    return cause;
}

// The independent watchdog runs from the LSI clock, nominally 32 kHz but anywhere from
// 17 to 47 kHz depending on the part and temperature, so the timeout is only approximate.
// A prescaler of 256 gives 125 ticks a second, and the 12 bit reload goes up to about 32 s.
// It keeps counting while a debugger has the processor halted unless it is frozen, so we
// freeze it.
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

#if FR_SD_ENABLE

#if SDCARD_SDIO
#error "The flight recorder's SD card storage needs the card on SPI"
#endif

// The same bus speed Src/diskio.c uses for the card.
#ifndef FATFS_SPI_PRESCALER
#define FATFS_SPI_PRESCALER SPI_BAUDRATEPRESCALER_16
#endif

// An SD card holds its data out line low while it programs a block, but only while it is
// selected. So select it, clock a byte and deselect it, as diskio.c's wait_ready() does
// but without the wait. The W5500 and diskio.c each set their own bus speed when they
// select, and we put back the speed we found.
bool fr_sd_card_busy (void)
{
    uint32_t speed = spi_set_speed(FATFS_SPI_PRESCALER & SPI_BAUDRATEPRESCALER_256);

    DIGITAL_OUT(SD_CS_PORT, SD_CS_PIN, 0);
    spi_get_byte();                     // as wait_ready(), the first byte can still be the turnaround
    bool busy = spi_get_byte() != 0xFF;
    DIGITAL_OUT(SD_CS_PORT, SD_CS_PIN, 1);
    spi_get_byte();                     // and as diskio.c, one more clock so the card lets go of the line

    spi_set_speed(speed);

    return busy;
}

#endif // FR_SD_ENABLE

#endif // FLIGHT_RECORDER_ENABLE
