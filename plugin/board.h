/*

  board.h - the board specific parts of the flight recorder

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

#ifndef _FR_BOARD_H_
#define _FR_BOARD_H_

#include <stdint.h>
#include <stdbool.h>

// In board_stm32f4.c on the board, board_sim.c in the simulator.

// Why the processor last reset (fr_reset_t bits, event_log.h), 0 if not known.
uint8_t fr_reset_cause (void);

// The hardware watchdog. Once started only a reset stops it.
void fr_watchdog_start (uint32_t seconds);
void fr_watchdog_feed (void);

// Is the SD card still programming a write? Must not wait. SD card builds only.
bool fr_sd_card_busy (void);

#endif
