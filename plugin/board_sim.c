/*

  board_sim.c - the board specific bits of the flight recorder, for the simulator

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

#include "driver.h"

#if FLIGHT_RECORDER_ENABLE

#include <stdlib.h>

#include "flight_recorder.h"
#include "event_log.h"
#include "board.h"

#if FR_SD_ENABLE
#include "sim_sdcard.h"
#endif

// The simulated processor has no reset flags and no watchdog. For tests, the
// environment variable SIM_RESET_CAUSE (fr_reset_t bits) stands in for the flags.
uint8_t fr_reset_cause (void)
{
    const char *cause = getenv("SIM_RESET_CAUSE");

    return cause ? (uint8_t)atoi(cause) : 0;
}

void fr_watchdog_start (uint32_t seconds)
{
    (void)seconds;
}

void fr_watchdog_feed (void)
{
}

#if FR_SD_ENABLE

// The simulator's card model holds its data out line low while it programs, like a real card.
bool fr_sd_card_busy (void)
{
    return sim_sdcard_busy();
}

#endif

#endif // FLIGHT_RECORDER_ENABLE
