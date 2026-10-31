/*

  event_log.h - event log for the flight recorder

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

#ifndef _FR_EVENT_LOG_H_
#define _FR_EVENT_LOG_H_

#include <stdint.h>
#include <stdbool.h>

#define FR_LOG_EVENTS   1000
#define FR_LOG_BASE     0x400       // storage offset, after the record slots and the probe byte

typedef enum {
    FR_Event_Boot = 1,      // code: reset cause (fr_reset_t), data: job state found at boot
    FR_Event_Alarm,         // code: alarm, data: line executing
    FR_Event_Error,         // code: status code of an error reply, data: line number
    FR_Event_Input,         // data: control inputs that went active (control_signals_t)
    FR_Event_Limit,         // data: limit switches active (limit_signals_t), outside homing
    FR_Event_Link,          // code: link flags (fr_link_t), data: stream type
    FR_Event_Modbus,        // data, per second: timeouts | CRC errors << 8 | exceptions << 16 | retries << 24
    FR_Event_Job,           // code: record state (fr_state_t), data: line, or the job id at start
    FR_Event_Storage,       // data: storage writes that failed before this one worked
    FR_Event_SpindleOff     // code: why, at boot (fr_boot_stop_t). Keep this one last, event_log.c checks the range
} fr_event_type_t;

typedef enum {
    FR_BootStop_Job       = 1 << 0, // a job was running when the controller went down
    FR_BootStop_Watchdog  = 1 << 1,
    FR_BootStop_BrownOut  = 1 << 2
} fr_boot_stop_t;

typedef enum {
    FR_Reset_PowerOn  = 1 << 0,
    FR_Reset_BrownOut = 1 << 1,
    FR_Reset_Pin      = 1 << 2,     // the NRST pin, so the reset button or noise on the line
    FR_Reset_Software = 1 << 3,
    FR_Reset_Watchdog = 1 << 4,
    FR_Reset_LowPower = 1 << 5
} fr_reset_t;

typedef enum {
    FR_Link_Connected = 1 << 0,
    FR_Link_Usb       = 1 << 1,
    FR_Link_Network   = 1 << 2
} fr_link_t;

typedef struct {
    uint32_t time;          // ms since that boot
    uint32_t data;
    uint16_t boot;          // boot number, low 16 bits
    uint16_t index;         // running event number, gives the order
    uint8_t  type;          // fr_event_type_t
    uint8_t  code;
    uint8_t  repeats;       // further identical events within a second of each other, stops at 255
    uint8_t  check;         // CRC-8 of the bytes above
} fr_event_t;

void fr_log_init (uint32_t boot, uint8_t reset_cause, uint8_t job_state);
void fr_log_event (fr_event_type_t type, uint8_t code, uint32_t data);
void fr_log_poll (uint32_t now);
void fr_log_storage_result (bool ok);
void fr_log_report (uint32_t count);
void fr_log_clear (void);

// From the board's storage file: why the processor last reset (fr_reset_t bits), 0 if not known.
uint8_t fr_reset_cause (void);

// From the board's storage file: the hardware watchdog. Once started only a reset stops it.
void fr_watchdog_start (uint32_t seconds);
void fr_watchdog_feed (void);

#endif
