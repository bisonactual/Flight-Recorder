/*

  flight_recorder.c - job record and stream loss retract

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
  The record lives in non-volatile memory (FRAM on the Flexi-HAL A7, or the SD
  card in the uFlexiNET module) and holds the line number the steppers are
  executing, the machine position, tool and work offset, and why the job
  stopped. A sender can resume a job from it after the PC, the link or the
  controller has gone away. The line number comes from
  core/executing-line.patch, so it is never ahead of the machine. While the
  line changes it is written at most every $794 ms, by default 20 on FRAM and
  100 on an SD card. A job start, a stream loss, a reset and the retract are
  written at once. An alarm stop and the program end go with the next write.

  If the sender stops feeding a job while the spindle is on, the machine goes
  idle with the tool sitting in the cut. After $791 seconds of that, or straight
  away if the USB port or network client goes, we stop reading input, retract
  straight up to machine Z -$792, stop the spindle and coolant and raise alarm
  80. Straight up is safe because the tool body has already been everywhere
  above where the tip has been - except with undercut tools, so jobs using
  those can opt out when they are stamped. No retract unless Z is homed.

  After an unexpected reset the core stops the spindle and coolant at boot. We
  log that it happened and send the stop again a couple of seconds later, in
  case a Modbus VFD missed it. Watchdog builds (FR_WATCHDOG_ENABLE) also reset
  the board if the main loop stops for $793 seconds.

  The commands, settings and how to resume are in docs/COMMANDS.md.
*/

// These have to come before driver.h, otherwise the simulator's driver.h pulls in
// the unpatched core headers and st_get_executing_line_number() is missing.
#include "grbl/hal.h"
#include "grbl/stepper.h"

#include "driver.h"

#if FLIGHT_RECORDER_ENABLE

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "grbl/nvs_buffer.h"
#include "grbl/gcode.h"
#include "grbl/planner.h"
#include "grbl/motion_control.h"
#include "grbl/system.h"
#include "grbl/state_machine.h"
#include "grbl/settings.h"
#include "grbl/alarms.h"
#include "grbl/report.h"
#include "grbl/spindle_control.h"
#include "grbl/task.h"

#include "flight_recorder.h"
#include "event_log.h"

#define SETTING_AUTO_RETRACT    790
#define SETTING_TIMEOUT         791
#define SETTING_RETRACT_DEPTH   792
#define SETTING_WATCHDOG        793
#define SETTING_RECORD_INTERVAL 794

#define ALARM_RETRACTED         ((alarm_code_t)80)
#define ALARM_RETRACT_FAILED    ((alarm_code_t)81)

#define MIN_WRITE_INTERVAL      10u         // ms, the shortest $794 can ask for
#ifndef FR_RETRACT_LIMIT                    // can be set shorter for a test build
#define FR_RETRACT_LIMIT        120000u     // ms we give a retract before stopping the machine
#endif

// record_interval uses what was padding before v0.4.0, so older saved settings still load,
// with it at 0.
typedef struct {
    bool auto_retract;
    float timeout;          // s
    float retract_depth;    // mm below machine zero
    uint8_t watchdog;       // s, 0 = off. Only a setting in watchdog builds, but always here so the layout is the same
    uint16_t record_interval; // ms between record writes while the line changes, 0 = the storage's default
} recorder_settings_t;

typedef enum {
    Retract_Idle = 0,
    Retract_Wait,           // waiting for the line that was executing to finish
    Retract_Up,
    Retract_SpindleOff
} retract_phase_t;

static recorder_settings_t config;
static nvs_address_t nvs_address;
static bool have_storage = false;

static fr_record_t record;
static uint32_t newest_slot = 0;
static bool record_changed = false;
static uint32_t last_write = 0;
static uint32_t write_interval = 20;        // ms, $794 or the storage's default
static uint8_t boot_reset_cause = 0;

static uint32_t last_line = 0;
static uint32_t starved_since = 0;
static volatile uint32_t last_delay = 0;    // when a dwell or spindle spin-up delay last ran
static volatile bool reset_pending = false;
static fr_reason_t link_lost = FR_Reason_None;
static stream_type_t last_stream_type;
static bool drop_partial_line = false;

static struct {
    stream_type_t type;
    bool check_connected;
} job_stream;               // the stream the job was started on

static retract_phase_t retract_phase = Retract_Idle;
static bool command_queued = false;
static uint32_t retract_started = 0;
static float retract_z = 0.0f;

static on_execute_realtime_ptr on_execute_realtime;
static on_state_change_ptr on_state_change;
static on_program_completed_ptr on_program_completed;
static on_reset_ptr on_reset;
static on_stream_changed_ptr on_stream_changed;
static on_report_options_ptr on_report_options;
#if FR_WATCHDOG_ENABLE || FR_SD_ENABLE
static on_execute_realtime_ptr on_execute_delay;
#endif
static stream_read_ptr stream_read;
static void (*driver_delay_ms)(uint32_t ms, delay_callback_ptr callback);

// Record storage

static uint32_t crc32 (const uint8_t *data, uint32_t size)
{
    uint32_t crc = 0xFFFFFFFFu;

    while(size--) {
        crc ^= *data++;
        for(uint_fast8_t bit = 0; bit < 8; bit++)
            crc = (crc >> 1) ^ ((crc & 1u) ? 0xEDB88320u : 0u);
    }

    return ~crc;
}

static bool is_valid_record (const fr_record_t *rec)
{
    return rec->magic == FR_MAGIC && rec->crc == crc32((const uint8_t *)rec, offsetof(fr_record_t, crc));
}

// There are two copies and each write goes to the older one, so a power cut in the
// middle of a write still leaves the previous record. If a write fails we try the
// same slot again next time and never touch the newer copy. On an SD card storage_sd.c
// picks the copy itself, as the card finishes writes later.
static void write_record (void)
{
    if(!have_storage)
        return;

    record.magic = FR_MAGIC;
    record.seq++;
    record.crc = crc32((const uint8_t *)&record, offsetof(fr_record_t, crc));

    bool ok = fr_storage_write((newest_slot ^ 1) * fr_storage_info()->record_stride, &record, sizeof(fr_record_t));

    if(ok)
        newest_slot ^= 1;

    record_changed = !ok;
    last_write = hal.get_elapsed_ticks();
}

static bool load_record (void)
{
    fr_record_t copy[2];
    bool valid[2];
    uint32_t stride = fr_storage_info()->record_stride;

    for(uint_fast8_t i = 0; i < 2; i++)
        valid[i] = fr_storage_read(i * stride, &copy[i], sizeof(fr_record_t)) && is_valid_record(&copy[i]);

    if(!(valid[0] || valid[1]))
        return false;

    // Take the newer one. The subtraction keeps that right when seq wraps around.
    if(valid[0] && valid[1])
        newest_slot = (int32_t)(copy[1].seq - copy[0].seq) > 0;
    else
        newest_slot = valid[1];

    memcpy(&record, &copy[newest_slot], sizeof(fr_record_t));

    return true;
}

// Fill in where the machine is and what it is doing right now.
static void take_snapshot (void)
{
    float mpos[N_AXIS];

    system_convert_array_steps_to_mpos(mpos, sys.position);
    memset(record.mpos, 0, sizeof(record.mpos));
    memcpy(record.mpos, mpos, sizeof(float) * (N_AXIS < FR_AXES ? N_AXIS : FR_AXES));

    record.tool = gc_state.tool ? gc_state.tool->tool_id : 0;
    record.wcs = (uint8_t)gc_state.modal.g5x_offset.id;
    record.flags.homed = !!(sys.homed.mask & bit(Z_AXIS));
    record.flags.spindle_on = gc_state.modal.spindle[0].state.on;
    record.flags.coolant_on = !!gc_state.modal.coolant.value;
}

// Job state changes go in the event log as well, state in the low nibble and reason in the high one.
static void log_job_state (void)
{
    fr_log_event(FR_Event_Job, (uint8_t)(record.state | record.reason << 4),
                  record.state == FR_Running ? record.job : record.line);
}

static void stop_job (fr_reason_t reason)
{
    take_snapshot();
    record.state = FR_Stopped;
    record.reason = reason;
    record.alarm = 0;
    write_record();
    log_job_state();
}

// Reporting

const char *fr_state_name (uint8_t state)
{
    static const char *names[] = { "Idle", "Running", "Stopped", "Retracted", "Completed" };

    return state < sizeof(names) / sizeof(names[0]) ? names[state] : "?";
}

const char *fr_reason_name (uint8_t reason)
{
    static const char *names[] = { "none", "timeout", "usb", "network", "alarm", "reset", "reboot" };

    return reason < sizeof(names) / sizeof(names[0]) ? names[reason] : "?";
}

static void report_record (void)
{
    char buf[200];

    snprintf(buf, sizeof(buf),
             "[FLIGHT:%s|why=%s|job=%08lX|ln=%lu|boot=%lu|alarm=%u|tool=%ld|wcs=%s|flags=%02X|mpos=%.3f,%.3f,%.3f,%.3f,%.3f]",
             fr_state_name(record.state), fr_reason_name(record.reason), (unsigned long)record.job,
             (unsigned long)record.line, (unsigned long)record.boot, record.alarm, (long)record.tool,
             gc_coord_system_to_str((coord_system_id_t)record.wcs), record.flags.value,
             record.mpos[0], record.mpos[1], record.mpos[2], record.mpos[3], record.mpos[4]);

    hal.stream.write(buf);
    hal.stream.write(ASCII_EOL);
}

// Input blocking, used once the sender's link is lost

// Stands in for the stream's read function while input is blocked. The first call
// returns CAN, which makes the core throw away any line it has only half received,
// so the rest of it can't get joined onto whatever arrives next.
static int32_t blockedRead (void)
{
    if(drop_partial_line) {
        drop_partial_line = false;
        return ASCII_CAN;
    }

    return SERIAL_NO_DATA;
}

// Also empties the input buffer, every time it is called with on set.
static void block_input (bool on)
{
    if(on) {
        if(hal.stream.read != blockedRead) {
            stream_read = hal.stream.read;
            hal.stream.read = blockedRead;
            drop_partial_line = true;
        }
        if(hal.stream.reset_read_buffer)
            hal.stream.reset_read_buffer();
    } else if(hal.stream.read == blockedRead)
        hal.stream.read = stream_read;
}

// Stream loss retract

static void end_retract (const char *message)
{
    retract_phase = Retract_Idle;
    command_queued = false;
    report_message(message, Message_Warning);
}

// The retract couldn't finish, for example because of a feed hold. Stop the machine
// instead of letting input back in with the spindle possibly still running.
static void abort_retract (const char *message)
{
    end_retract(message);
    mc_reset();
    system_set_exec_alarm(ALARM_RETRACT_FAILED);
}

static void stream_lost (fr_reason_t reason)
{
    stop_job(reason);

    if(!config.auto_retract || record.flags.no_retract || !record.flags.homed) {
        report_message("Flight recorder: stream lost, no auto retract", Message_Warning);
        return;
    }

    block_input(true);
    retract_z = -config.retract_depth;
    retract_phase = Retract_Wait;
    command_queued = false;
    retract_started = hal.get_elapsed_ticks();
}

static void retract_poll (sys_state_t state, uint32_t now)
{
    float mpos[N_AXIS];

    system_convert_array_steps_to_mpos(mpos, sys.position);
    float z = mpos[Z_AXIS];

    if(state & (STATE_ALARM|STATE_ESTOP)) {
        end_retract("Flight recorder: retract abandoned");
        return;
    }

    if(now - retract_started > FR_RETRACT_LIMIT) {
        abort_retract("Flight recorder: retract did not finish, machine stopped");
        return;
    }

    block_input(true);

    switch(retract_phase) {

        // The link can go while a line is still executing, a dwell or a spin-up delay
        // say, and that line may still move afterwards. Queued commands only run
        // between lines and G4P0 waits for all motion before it, so once it has run
        // Z is where the job left it. Don't queue it until the CAN from blockedRead()
        // has been read, as the CAN would throw it away.
        case Retract_Wait:
            if(drop_partial_line)
                break;
            if(!command_queued)
                command_queued = grbl.enqueue_gcode("G4P0");
            else if(grbl.enqueue_gcode("")) {   // only accepted once G4P0 has run, and queues nothing
                retract_phase = z >= retract_z - 0.001f ? Retract_SpindleOff : Retract_Up;
                command_queued = false;
            }
            break;

        case Retract_Up:
            if(!command_queued) {
                char command[32];
                float value = gc_state.modal.units_imperial ? retract_z / MM_PER_INCH : retract_z;
                snprintf(command, sizeof(command), "G53G0Z%.4f", value);
                command_queued = grbl.enqueue_gcode(command);
            } else if(state == STATE_IDLE && plan_get_current_block() == NULL && fabsf(z - retract_z) < 0.01f) {
                retract_phase = Retract_SpindleOff;
                command_queued = false;
            }
            break;

        case Retract_SpindleOff:
            if(!command_queued)
                command_queued = grbl.enqueue_gcode("M5M9");
            else if(state == STATE_IDLE && !gc_state.modal.spindle[0].state.on) {
                // Should never happen since nothing else can move Z now, but check before we say Retracted.
                if(z < retract_z - 0.01f) {
                    abort_retract("Flight recorder: Z below the retract height, machine stopped");
                    break;
                }
                record.state = FR_Retracted;
                write_record();
                log_job_state();
                retract_phase = Retract_Idle;
                command_queued = false;
                system_raise_alarm(ALARM_RETRACTED);
            }
            break;

        default:
            break;
    }
}

// Watchdog and spindle stop at boot

static uint8_t spindle_stop_reason = 0;     // fr_boot_stop_t bits
static uint32_t spindle_stop_again = 0;     // when to send the stop again, 0 until logged
static bool storage_checked = false;        // the storage has started, or there is none

#if FR_WATCHDOG_ENABLE
static enum { Watchdog_NotChecked = 0, Watchdog_Off, Watchdog_On } watchdog = Watchdog_NotChecked;
#endif

#if FR_WATCHDOG_ENABLE || FR_SD_ENABLE

// grblHAL runs this instead of the realtime poll while it waits: dwells, spin-up, and
// its own delays such as the 100 ms after an alarm. It feeds the watchdog and keeps an
// SD card's writes going, as those wait for the poll.
static void onExecuteDelay (sys_state_t state)
{
#if FR_WATCHDOG_ENABLE
    if(watchdog == Watchdog_On)
        fr_watchdog_feed();
#endif
    if(have_storage)
        fr_storage_poll();

    on_execute_delay(state);
}

#endif

#if FR_WATCHDOG_ENABLE

// $793 is only read once, at the first poll, so a change takes effect at the next boot.
static void watchdog_poll (void)
{
    if(watchdog == Watchdog_On)
        fr_watchdog_feed();
    else if(watchdog == Watchdog_NotChecked) {
        if(config.watchdog) {
            fr_watchdog_start(config.watchdog < 3 ? 3 : (config.watchdog > 10 ? 10 : config.watchdog));
            watchdog = Watchdog_On;
        } else
            watchdog = Watchdog_Off;
    }
}

#endif

// The core has already stopped the spindle and coolant at boot. Log why it mattered,
// then send the stop again two seconds later in case a Modbus VFD wasn't listening yet.
// The SD card starts a little after boot, so wait for the storage, or the event would
// go to a log that isn't there yet.
static void spindle_stop_poll (uint32_t now)
{
    if(!spindle_stop_reason || !storage_checked)
        return;

    if(!spindle_stop_again) {
        fr_log_event(FR_Event_SpindleOff, spindle_stop_reason, 0);
        spindle_stop_again = now + 2000;
    } else if((int32_t)(now - spindle_stop_again) >= 0) {
        if(!gc_state.modal.spindle[0].state.on)
            spindle_all_off(true);
        if(!gc_state.modal.coolant.value)
            hal.coolant.set_state((coolant_state_t){0});
        spindle_stop_reason = 0;
    }
}

// Realtime polling

// Once the sender is gone nobody gets input until the job has retracted or stopped,
// so a client that takes over can't carry on the dead sender's job.
static void sender_lost (void)
{
    link_lost = record.flags.usb ? FR_Reason_UsbClosed : FR_Reason_NetworkClosed;
    block_input(true);
    report_message("Flight recorder: sender link lost", Message_Warning);
}

static void onExecuteRealtime (sys_state_t state)
{
    on_execute_realtime(state);

    uint32_t now = hal.get_elapsed_ticks();

#if FR_WATCHDOG_ENABLE
    watchdog_poll();
#endif
    spindle_stop_poll(now);

    if(!have_storage)
        return;

    fr_storage_poll();
    fr_log_poll(now);

    if(reset_pending) {
        reset_pending = false;
        if(retract_phase != Retract_Idle)
            end_retract("Flight recorder: retract stopped by reset");
        else if(record.state == FR_Running)
            stop_job(FR_Reason_Reset);
    }

    // Let input back in once the job has settled and the partial line has been dropped.
    if(retract_phase == Retract_Idle && record.state != FR_Running && hal.stream.read == blockedRead && !drop_partial_line)
        block_input(false);

    if(retract_phase != Retract_Idle) {
        retract_poll(state, now);
        return;
    }

    if(record.state == FR_Running) {

        // Follow the line the steppers are executing. Lines without an N-number (most
        // macros) come through as 0 and are skipped. So are jogs, as their N-number
        // isn't a line of the file.
        uint32_t line = st_get_executing_line_number();
        if(line != last_line) {
            last_line = line;
            if(line && state != STATE_JOG) {
                record.line = line;
                take_snapshot();
                record_changed = true;
            }
        }

        // Jobs run from the SD card aren't watched. The file can't hang and there is no
        // sender link to lose.
        if(hal.stream.type != StreamType_File) {

            // Has the sender gone? Either the input moved to another stream (onStreamChanged()
            // also catches that the moment it happens) or the connection closed.
            if(link_lost == FR_Reason_None &&
                (hal.stream.type != job_stream.type || hal.stream.state.is_usb != record.flags.usb ||
                  (job_stream.check_connected && hal.stream.is_connected && !hal.stream.is_connected())))
                sender_lost();
            else if(link_lost)
                block_input(true);

            bool nothing_queued = state == STATE_IDLE && plan_get_current_block() == NULL &&
                                   (hal.stream.get_rx_buffer_count == NULL || hal.stream.get_rx_buffer_count() == 0);
            bool starved = nothing_queued && gc_state.modal.spindle[0].state.on;

            if(!starved)
                starved_since = now;
            else if((int32_t)(last_delay - starved_since) > 0)   // a dwell or spin-up doesn't count
                starved_since = last_delay;

            if(starved && (link_lost || now - starved_since >= (uint32_t)(config.timeout * 1000.0f)))
                stream_lost(link_lost ? link_lost : FR_Reason_StreamTimeout);
            else if(link_lost && nothing_queued)    // sender gone with the spindle off, nothing to retract
                stop_job(link_lost);
        }
    }

    if(record_changed && now - last_write >= write_interval)
        write_record();
}

// Core hooks

// Note when something waits in a delay, so a dwell isn't taken for a hung sender.
static void delayMs (uint32_t ms, delay_callback_ptr callback)
{
    last_delay = hal.get_elapsed_ticks();
    driver_delay_ms(ms, callback);
}

static void onStateChanged (sys_state_t state)
{
    if(on_state_change)
        on_state_change(state);

    if(state & (STATE_ALARM|STATE_ESTOP))
        fr_log_event(FR_Event_Alarm, (uint8_t)sys.alarm, st_get_executing_line_number());

    // The alarm that follows a reset (homing required, cycle aborted) isn't why the job
    // stopped, the reset is, and the realtime poll records that. A critical alarm such
    // as an e-stop or a limit causes a reset too, and there the alarm is the reason.
    if(record.state == FR_Running && retract_phase == Retract_Idle && (state & (STATE_ALARM|STATE_ESTOP)) &&
        !(reset_pending && !alarm_is_critical(sys.alarm))) {
        take_snapshot();
        record.state = FR_Stopped;
        record.reason = FR_Reason_Alarm;
        record.alarm = (uint8_t)sys.alarm;
        record_changed = true;
        log_job_state();
    }
}

static void onProgramCompleted (program_flow_t program_flow, bool check_mode)
{
    if(on_program_completed)
        on_program_completed(program_flow, check_mode);

    if(!check_mode && record.state == FR_Running) {
        take_snapshot();
        record.state = FR_Completed;
        record.reason = FR_Reason_None;
        record_changed = true;
        log_job_state();
    }
}

// Called from an interrupt, so just make a note for the realtime poll.
static void onReset (void)
{
    reset_pending = true;

    if(on_reset)
        on_reset();
}

static void onStreamChanged (stream_type_t type)
{
    if(on_stream_changed)
        on_stream_changed(type);

    // Any change of input during a job means the sender has lost it, even if a new
    // client takes over before the next poll. Catching it here, before the new stream
    // is read, stops that client's first lines from running. Starting or ending a file
    // run from the SD card doesn't count.
    bool file = type == StreamType_File || last_stream_type == StreamType_File;

    last_stream_type = type;

    if(record.state == FR_Running && retract_phase == Retract_Idle && link_lost == FR_Reason_None && !file)
        sender_lost();
    else if(retract_phase != Retract_Idle || (record.state == FR_Running && link_lost))
        block_input(true);
}

// $ commands

static status_code_t start_job (sys_state_t state, char *args)
{
    char *end;
    uint32_t job = strtoul(args, &end, 16);
    uint32_t flags = *end == ',' ? strtoul(end + 1, NULL, 10) : 0;

    if(end == args || !(*end == '\0' || *end == ','))
        return Status_InvalidStatement;

    if(state != STATE_IDLE)
        return Status_IdleError;

    // The same job stamped again after it stopped is a resume. The moves back into the
    // cut have no N-numbers, so keep the old line until a numbered line runs, otherwise
    // a second interruption during them would lose it.
    bool resuming = job == record.job && (record.state == FR_Stopped || record.state == FR_Retracted);

    record.state = FR_Running;
    record.reason = FR_Reason_None;
    record.alarm = 0;
    record.job = job;
    if(!resuming)
        record.line = 0;
    record.flags.value = 0;
    record.flags.no_retract = !!(flags & 1);
    record.flags.usb = hal.stream.state.is_usb;
    record.flags.network = hal.stream.type == StreamType_WebSocket || hal.stream.type == StreamType_Telnet;
    take_snapshot();
    write_record();
    log_job_state();

    job_stream.type = hal.stream.type;
    job_stream.check_connected = hal.stream.is_connected && hal.stream.is_connected();
    link_lost = FR_Reason_None;
    last_line = st_get_executing_line_number();
    starved_since = hal.get_elapsed_ticks();

    if(config.auto_retract && !record.flags.no_retract && !record.flags.homed)
        report_message("Flight recorder: Z not homed, no auto retract for this job", Message_Warning);

    report_record();

    return Status_OK;
}

static status_code_t cmd_flight (sys_state_t state, char *args)
{
    if(args)
        return start_job(state, args);

    report_record();

    return Status_OK;
}

static status_code_t cmd_flight_clear (sys_state_t state, char *args)
{
    // Not during a job or a retract. Clearing the record would stop the recorder
    // watching the job, and the retract with it.
    if(record.state == FR_Running || retract_phase != Retract_Idle)
        return Status_IdleError;

    memset(&record.mpos, 0, sizeof(record.mpos));
    record.state = FR_Idle;
    record.reason = FR_Reason_None;
    record.alarm = 0;
    record.job = 0;
    record.line = 0;
    record.flags.value = 0;
    write_record();
    report_record();

    return Status_OK;
}

static status_code_t cmd_events (sys_state_t state, char *args)
{
    // Not while the machine moves: reading the log can take a while, on an SD card up to
    // 33 sector reads, each waiting first for a busy card.
    if(!(state == STATE_IDLE || (state & (STATE_ALARM|STATE_ESTOP|STATE_CHECK_MODE|STATE_SLEEP))))
        return Status_IdleError;

    uint32_t count = args ? strtoul(args, NULL, 10) : 100;

    fr_log_report(count ? count : 100);

    return Status_OK;
}

static status_code_t cmd_events_clear (sys_state_t state, char *args)
{
    fr_log_clear();

    return Status_OK;
}

#if FR_SD_ENABLE

// SD card storage only: how the card has been doing. $FLIGHTSD=R starts the counts again.
static status_code_t cmd_flight_sd (sys_state_t state, char *args)
{
    if(args && !(args[0] == 'R' && args[1] == '\0'))
        return Status_InvalidStatement;

    return fr_storage_report(args != NULL) ? Status_OK : Status_InvalidStatement;
}

#endif

#if FR_WATCHDOG_ENABLE

// Hang the way a stuck controller would, until the watchdog resets the board.
static status_code_t cmd_watchdog_test (sys_state_t state, char *args)
{
    if(watchdog != Watchdog_On)
        return Status_SettingDisabled;

    if(!(state == STATE_IDLE || state == STATE_ALARM))
        return Status_IdleError;

    report_message("Flight recorder: hanging until the watchdog resets the board", Message_Warning);
    for(;;);
}

#endif

// $FLIGHT and $EVENTS still answer during an e-stop or other blocking alarm.
static const sys_command_t flight_commands[] = {
    { "FLIGHT", cmd_flight, { .allow_blocking = On }, { .str = "report the flight record; =<job>[,<flags>] starts recording a job" } },
    { "FLIGHTCLEAR", cmd_flight_clear, { .noargs = On }, { .str = "clear the flight record" } },
    { "EVENTS", cmd_events, { .allow_blocking = On }, { .str = "list the last 100 events; =<n> the last n" } },
    { "EVENTSCLEAR", cmd_events_clear, { .noargs = On }, { .str = "clear the event log" } },
#if FR_SD_ENABLE
    { "FLIGHTSD", cmd_flight_sd, { .allow_blocking = On }, { .str = "SD card storage: writes and times; =R starts the counts again" } },
#endif
#if FR_WATCHDOG_ENABLE
    { "WATCHDOGTEST", cmd_watchdog_test, { .noargs = On }, { .str = "hang until the watchdog resets the board" } },
#endif
};

static void onReportOptions (bool newopt)
{
    if(on_report_options)
        on_report_options(newopt);

    if(!newopt) {
        char name[40];
        const char *label = have_storage ? fr_storage_info()->label : "no storage";
        if(label)
            snprintf(name, sizeof(name), "Flight Recorder (%s)", label);
        report_plugin(label ? name : "Flight Recorder", FR_VERSION);
    }
}

// Settings

static const setting_detail_t plugin_settings[] = {
    { SETTING_AUTO_RETRACT, Group_General, "Auto retract on stream loss", NULL, Format_Bool, NULL, NULL, NULL, Setting_NonCore, &config.auto_retract },
    { SETTING_TIMEOUT, Group_General, "Stream loss timeout", "s", Format_Decimal, "#0.0", "0.5", "60", Setting_NonCore, &config.timeout },
    { SETTING_RETRACT_DEPTH, Group_General, "Retract below machine zero", "mm", Format_Decimal, "##0.0", "1", "500", Setting_NonCore, &config.retract_depth },
#if FR_WATCHDOG_ENABLE
    // 3 to 10 s, or 0 for off. 1 and 2 are taken as 3. Takes effect at the next boot.
    { SETTING_WATCHDOG, Group_General, "Watchdog timeout", "s", Format_Int8, "#0", "0", "10", Setting_NonCore, &config.watchdog },
#endif
    // 0 for the storage's own, 20 ms on FRAM and 100 ms on an SD card. 1 to 9 are taken as 10.
    { SETTING_RECORD_INTERVAL, Group_General, "Flight record write interval", "ms", Format_Int16, "###0", "0", "1000", Setting_NonCore, &config.record_interval }
};

static void set_write_interval (void)
{
    uint32_t interval = config.record_interval ? config.record_interval : (have_storage ? fr_storage_info()->record_interval : 20);

    write_interval = interval < MIN_WRITE_INTERVAL ? MIN_WRITE_INTERVAL : interval;
}

static void plugin_settings_save (void)
{
    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&config, sizeof(config), true);
    set_write_interval();
}

static void plugin_settings_restore (void)
{
    config.auto_retract = true;
    config.timeout = 3.0f;
    config.retract_depth = 20.0f;
    config.watchdog = 5;
    config.record_interval = 0;
    plugin_settings_save();
}

static void plugin_settings_load (void)
{
    if(hal.nvs.memcpy_from_nvs((uint8_t *)&config, nvs_address, sizeof(config), true) != NVS_TransferResult_OK)
        plugin_settings_restore();
    set_write_interval();
}

// Once there is somewhere to keep it: load the record, note the boot, start the log.
static void storage_ready (void)
{
    static sys_commands_t commands = {
        .n_commands = sizeof(flight_commands) / sizeof(sys_command_t),
        .commands = flight_commands
    };

    have_storage = true;
    set_write_interval();

    if(!load_record())
        memset(&record, 0, sizeof(record));

    // A record still Running at boot means the controller restarted in the middle of a job.
    uint8_t state_at_boot = record.state;

    record.boot++;
    if(record.state == FR_Running) {
        record.state = FR_Stopped;
        record.reason = FR_Reason_Reboot;
        spindle_stop_reason |= FR_BootStop_Job;
    }
    write_record();

    fr_log_init(record.boot, boot_reset_cause, state_at_boot);
    system_register_commands(&commands);
}

#if FR_SD_ENABLE

// An SD card is only mounted once the drivers are up, after the plugins have started.
static void storage_start_late (void *data)
{
    if(fr_storage_init(true))
        storage_ready();
    storage_checked = true;
}

#endif

void flight_recorder_init (void)
{
    static setting_details_t settings = {
        .settings = plugin_settings,
        .n_settings = sizeof(plugin_settings) / sizeof(setting_detail_t),
        .load = plugin_settings_load,
        .save = plugin_settings_save,
        .restore = plugin_settings_restore
    };

    static const alarm_detail_t alarms[] = {
        { ALARM_RETRACTED, "Stream lost mid-job: tool retracted, spindle stopped. Resume from the flight record." },
        { ALARM_RETRACT_FAILED, "Stream lost mid-job, and the retract couldn't finish: machine stopped where it was. Check the tool before resuming." }
    };

    static alarm_details_t alarm_details = {
        .n_alarms = sizeof(alarms) / sizeof(alarm_detail_t),
        .alarms = alarms
    };

    if(!(nvs_address = nvs_alloc(sizeof(config))))
        return;

    settings_register(&settings);
    alarms_register(&alarm_details);

    boot_reset_cause = fr_reset_cause();

    // A power-on sets the brown-out flag as well, so only count a brown-out on its own.
    if(boot_reset_cause & FR_Reset_Watchdog)
        spindle_stop_reason |= FR_BootStop_Watchdog;
    if((boot_reset_cause & FR_Reset_BrownOut) && !(boot_reset_cause & FR_Reset_PowerOn))
        spindle_stop_reason |= FR_BootStop_BrownOut;

    // FRAM, if it is fitted, as before. Otherwise an SD card, once grblHAL has it mounted.
    if(fr_storage_init(false))
        storage_ready();
#if FR_SD_ENABLE
    else
        task_run_on_startup(storage_start_late, NULL);
#endif
    storage_checked = have_storage || !FR_SD_ENABLE;

    on_execute_realtime = grbl.on_execute_realtime;
    grbl.on_execute_realtime = onExecuteRealtime;

    on_state_change = grbl.on_state_change;
    grbl.on_state_change = onStateChanged;

    on_program_completed = grbl.on_program_completed;
    grbl.on_program_completed = onProgramCompleted;

    on_reset = grbl.on_reset;
    grbl.on_reset = onReset;

    last_stream_type = hal.stream.type;
    on_stream_changed = grbl.on_stream_changed;
    grbl.on_stream_changed = onStreamChanged;

    on_report_options = grbl.on_report_options;
    grbl.on_report_options = onReportOptions;

    driver_delay_ms = hal.delay_ms;
    hal.delay_ms = delayMs;

#if FR_WATCHDOG_ENABLE || FR_SD_ENABLE
    on_execute_delay = grbl.on_execute_delay;
    grbl.on_execute_delay = onExecuteDelay;
#endif
}

#endif // FLIGHT_RECORDER_ENABLE
