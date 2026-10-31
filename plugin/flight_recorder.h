/*
  flight_recorder.h — Flight recorder plugin for grblHAL.

  Keeps a small record of the job in non-volatile memory (FRAM on the
  Flexi-HAL A7, a host file in the simulator): the line the machine is
  executing, where it is, and why it stopped. A sender can resume a job from
  it after the PC or the controller has gone away.

  See docs/COMMANDS.md for the $ commands, settings and alarm.
*/

#ifndef _FLIGHT_RECORDER_H_
#define _FLIGHT_RECORDER_H_

#include <stdint.h>
#include <stdbool.h>

#define FR_VERSION     "0.2.0"
#define FR_MAGIC       0x31524646u  // "FFR1": layout version 1
#define FR_SLOT_SIZE   64u          // two copies, at 0 and FR_SLOT_SIZE
#define FR_AXES        5

typedef enum {
    FR_Idle = 0,        // no job recorded, or cleared
    FR_Running,         // job in progress
    FR_Stopped,         // job interrupted; reason says why
    FR_Retracted,       // stream lost: tool retracted, spindle stopped
    FR_Completed        // program end reached
} fr_state_t;

typedef enum {
    FR_Reason_None = 0,
    FR_Reason_StreamTimeout,    // no input for the timeout with the spindle on
    FR_Reason_UsbClosed,        // the sender closed the USB port (DTR dropped)
    FR_Reason_NetworkClosed,    // the network client went away
    FR_Reason_Alarm,            // an alarm (see alarm field)
    FR_Reason_Reset,            // soft reset or stop
    FR_Reason_Reboot            // controller restarted with the job running
} fr_reason_t;

typedef union {
    uint8_t value;
    struct {
        uint8_t homed      :1,  // Z was homed when written
                spindle_on :1,
                coolant_on :1,
                no_retract :1,  // job opted out of the auto retract
                usb        :1,  // job streamed over USB
                network    :1,  // job streamed over the network
                unused     :2;
    };
} fr_flags_t;

typedef struct {
    uint32_t magic;
    uint32_t seq;               // write counter: the newer valid copy wins
    uint32_t boot;              // controller boot count when written
    uint32_t job;               // job id from the sender, 0 = none
    uint32_t line;              // N-number of the line executing, 0 = none yet
    float    mpos[FR_AXES];     // machine position, mm
    int32_t  tool;
    uint8_t  state;             // fr_state_t
    uint8_t  reason;            // fr_reason_t
    uint8_t  alarm;             // alarm code when reason is Alarm
    uint8_t  wcs;               // 0 = G54 ... 5 = G59
    fr_flags_t flags;
    uint8_t  reserved[3];
    uint32_t crc;               // CRC-32 of everything above
} fr_record_t;

// Storage: a byte-addressed region owned by the recorder.
bool fr_storage_init (void);
bool fr_storage_read (uint32_t addr, void *data, uint32_t size);
bool fr_storage_write (uint32_t addr, const void *data, uint32_t size);

const char *fr_state_name (uint8_t state);
const char *fr_reason_name (uint8_t reason);

void flight_recorder_init (void);

#endif
