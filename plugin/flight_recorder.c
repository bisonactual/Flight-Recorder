/*
  flight_recorder.c — Flight recorder plugin for grblHAL.

  Records the job in non-volatile memory so a sender can resume it after a
  PC, link or controller failure:
    - the N-number of the line the steppers are executing (never ahead of
      the machine; needs core/executing-line.patch), the machine position,
      tool and work offset;
    - why the job stopped: program end, alarm, reset, reboot, or the stream
      going away.

  Stream-loss retract. With a job running and the spindle on, the machine
  only goes idle with nothing queued when the sender has stopped feeding it,
  and the tool is then dwelling in the cut. When that lasts the timeout
  ($791), or at once if the USB port or network client has closed, the
  plugin:
    1. blocks further input, so late lines can't cut;
    2. retracts straight up to machine Z = -$792 (machine zero is the top of
       Z: this is never measured from work zero), spindle still turning;
    3. stops the spindle and coolant once clear;
    4. records it and raises an alarm. The sender resumes from the record.
  Straight up is safe because the tool body has already swept the column
  above anywhere its tip has been. Undercut tools break that, so jobs using
  them opt out (flag 1 of $FLIGHT=<job>). Without Z homed there is no retract.

  Event log (event_log.c): resets and their cause, alarms, error replies,
  inputs, limits, the sender's link and Modbus errors, in order, across
  reboots: what to read when a machine misbehaves "for no reason".

  After an unexpected reset (a job was running, a watchdog reset or a
  brown-out), the stop of the spindle and coolant that the core sends at boot
  is logged, and sent again two seconds later for a Modbus VFD that missed
  it. Nothing moves: the board waits in its homing alarm.

  Watchdog builds (FR_WATCHDOG_ENABLE): if the main loop stops for $793
  seconds, the processor resets, and the above stops the spindle. It starts
  with the main loop, so slow start-up work can't trip it, and is fed from the
  main loop and from blocking delays.

  Commands, replies and settings ($790-$793): docs/COMMANDS.md.
*/

// grblHAL headers before driver.h: the simulator's driver.h would otherwise
// pull in the unpatched core headers, without st_get_executing_line_number().
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

#include "flight_recorder.h"
#include "event_log.h"

#define FR_SETTING_RETRACT  790
#define FR_SETTING_TIMEOUT  791
#define FR_SETTING_DEPTH    792
#define FR_SETTING_WATCHDOG 793

#define FR_ALARM            ((alarm_code_t)80)  // retracted
#define FR_ALARM_STOPPED    ((alarm_code_t)81)  // the retract couldn't finish: machine stopped

#define FR_WRITE_INTERVAL   20u     // ms between writes while the line changes
#ifndef FR_RETRACT_LIMIT            // shorter for a test build
#define FR_RETRACT_LIMIT    120000u // ms before a retract that hasn't finished is given up
#endif

typedef struct {
    bool  auto_retract;
    float timeout;          // s
    float retract_depth;    // mm below machine zero
    uint8_t watchdog;       // s, 0 = off; a setting in watchdog builds only, kept in all for one layout
} fr_settings_t;

/* ── Static state ─────────────────────────────────────────────────────────── */

static fr_settings_t config;
static nvs_address_t nvs_addr;
static bool storage_ok = false;

static fr_record_t rec;
static uint32_t slot = 0;           // slot written last
static bool dirty = false;
static uint32_t last_write = 0;

static uint32_t line_seen = 0;      // executing line last looked at
static uint32_t quiet_since = 0;    // start of the current starved stretch
static volatile uint32_t busy_at = 0; // last time something blocked in a delay (dwell, spin-up)
static volatile bool reset_seen = false;
static fr_reason_t link_lost = FR_Reason_None;
static stream_type_t stream_was;    // input before the last stream change
static bool discard = false;        // the next blocked read drops the half-received line

static struct {
    stream_type_t type;
    bool watch_connection;
} job_link;

static enum { Retract_Idle = 0, Retract_Sync, Retract_Move, Retract_SpindleOff } retract = Retract_Idle;
static bool retract_queued = false;
static uint32_t retract_started = 0;
static float retract_target = 0.0f;

static on_execute_realtime_ptr     on_execute_realtime = NULL;
static on_state_change_ptr         on_state_change = NULL;
static on_program_completed_ptr    on_program_completed = NULL;
static on_reset_ptr                on_reset = NULL;
static on_stream_changed_ptr       on_stream_changed = NULL;
static on_report_options_ptr       on_report_options = NULL;
#if FR_WATCHDOG_ENABLE
static on_execute_realtime_ptr     on_execute_delay = NULL;
#endif
static stream_read_ptr             stream_read = NULL;
static void (*delay_ms)(uint32_t ms, delay_callback_ptr callback) = NULL;

/* ── Record storage ───────────────────────────────────────────────────────── */

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

static bool record_valid (const fr_record_t *r)
{
    return r->magic == FR_MAGIC && r->crc == crc32((const uint8_t *)r, offsetof(fr_record_t, crc));
}

// Two copies: each write goes to the older slot, so a power cut mid-write
// leaves the previous record intact. A failed write leaves the newer copy
// alone too: the retry goes to the same older slot.
static void record_write (void)
{
    if(!storage_ok)
        return;

    rec.magic = FR_MAGIC;
    rec.seq++;
    rec.crc = crc32((const uint8_t *)&rec, offsetof(fr_record_t, crc));

    bool ok = fr_storage_write((slot ^ 1) * FR_SLOT_SIZE, &rec, sizeof(fr_record_t));
    fr_log_storage_result(ok);
    if(ok)
        slot ^= 1;

    dirty = !ok;
    last_write = hal.get_elapsed_ticks();
}

static bool record_load (void)
{
    fr_record_t r[2];
    bool ok[2];

    for(uint_fast8_t i = 0; i < 2; i++)
        ok[i] = fr_storage_read(i * FR_SLOT_SIZE, &r[i], sizeof(fr_record_t)) && record_valid(&r[i]);

    if(!(ok[0] || ok[1]))
        return false;

    slot = ok[0] && ok[1] ? (int32_t)(r[1].seq - r[0].seq) > 0 : ok[1];  // newer, across wrap-around
    memcpy(&rec, &r[slot], sizeof(fr_record_t));

    return true;
}

// Where the machine is and what it is doing, from the live state.
static void record_snapshot (void)
{
    float mpos[N_AXIS];

    system_convert_array_steps_to_mpos(mpos, sys.position);
    memset(rec.mpos, 0, sizeof(rec.mpos));
    memcpy(rec.mpos, mpos, sizeof(float) * (N_AXIS < FR_AXES ? N_AXIS : FR_AXES));

    rec.tool = gc_state.tool ? gc_state.tool->tool_id : 0;
    rec.wcs = (uint8_t)gc_state.modal.g5x_offset.id;
    rec.flags.homed = !!(sys.homed.mask & bit(Z_AXIS));
    rec.flags.spindle_on = gc_state.modal.spindle[0].state.on;
    rec.flags.coolant_on = !!gc_state.modal.coolant.value;
}

// The job's state changes go into the event log too: state in the low nibble, reason in the high.
static void log_job (void)
{
    fr_log_event(FR_Event_Job, (uint8_t)(rec.state | rec.reason << 4), rec.state == FR_Running ? rec.job : rec.line);
}

static void record_stop (fr_reason_t reason, alarm_code_t alarm)
{
    record_snapshot();
    rec.state = FR_Stopped;
    rec.reason = reason;
    rec.alarm = (uint8_t)alarm;
    record_write();
    log_job();
}

/* ── Reporting ────────────────────────────────────────────────────────────── */

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
             fr_state_name(rec.state), fr_reason_name(rec.reason), (unsigned long)rec.job,
             (unsigned long)rec.line, (unsigned long)rec.boot, rec.alarm, (long)rec.tool,
             gc_coord_system_to_str((coord_system_id_t)rec.wcs), rec.flags.value,
             rec.mpos[0], rec.mpos[1], rec.mpos[2], rec.mpos[3], rec.mpos[4]);
    hal.stream.write(buf);
    hal.stream.write(ASCII_EOL);
}

/* ── Input blocking (after the sender's link is lost) ─────────────────────── */

// The first read after blocking returns CAN, which makes the core drop the line it
// has half received: the rest of it must not be joined onto whatever comes next.
static int32_t blocked_read (void)
{
    if(discard) {
        discard = false;
        return ASCII_CAN;
    }

    return SERIAL_NO_DATA;
}

// Blocking also empties the input buffer, every time it's called.
static void input_block (bool on)
{
    if(on) {
        if(hal.stream.read != blocked_read) {
            stream_read = hal.stream.read;
            hal.stream.read = blocked_read;
            discard = true;
        }
        if(hal.stream.reset_read_buffer)
            hal.stream.reset_read_buffer();
    } else if(hal.stream.read == blocked_read)
        hal.stream.read = stream_read;
}

/* ── Stream-loss retract ──────────────────────────────────────────────────── */

static void retract_end (const char *message)
{
    retract = Retract_Idle;
    retract_queued = false;
    report_message(message, Message_Warning);
}

// The retract couldn't finish (a hold, or a command that never completed): stop the
// machine rather than reopen input with the spindle perhaps still turning.
static void retract_fail (const char *message)
{
    retract_end(message);
    mc_reset();
    system_set_exec_alarm(FR_ALARM_STOPPED);
}

static void stream_lost (fr_reason_t reason)
{
    record_stop(reason, Alarm_None);

    if(!config.auto_retract || rec.flags.no_retract || !rec.flags.homed) {
        report_message("Flight recorder: stream lost, no auto retract", Message_Warning);
        return;
    }

    input_block(true);
    retract_target = -config.retract_depth;
    retract = Retract_Sync;
    retract_queued = false;
    retract_started = hal.get_elapsed_ticks();
}

static void retract_step (sys_state_t state, uint32_t now)
{
    float mpos[N_AXIS];

    system_convert_array_steps_to_mpos(mpos, sys.position);
    float z = mpos[Z_AXIS];

    if(state & (STATE_ALARM|STATE_ESTOP)) {
        retract_end("Flight recorder: retract abandoned");
        return;
    }

    if(now - retract_started > FR_RETRACT_LIMIT) {
        retract_fail("Flight recorder: retract did not finish, machine stopped");
        return;
    }

    input_block(true);

    switch(retract) {

        // The stream can go while a line is still executing (a dwell, a spin-up), and
        // that line may move further. Queued commands run only between lines, and G4P0
        // waits for the motion before it: once it has run, Z is where the job left it.
        // Queued only after the blocked read's CAN, which would discard it.
        case Retract_Sync:
            if(discard)
                break;
            if(!retract_queued)
                retract_queued = grbl.enqueue_gcode("G4P0");
            else if(grbl.enqueue_gcode("")) {       // accepted only once G4P0 has run; queues nothing
                retract = z >= retract_target - 0.001f ? Retract_SpindleOff : Retract_Move;
                retract_queued = false;
            }
            break;

        case Retract_Move:
            if(!retract_queued) {
                char cmd[32];
                float value = gc_state.modal.units_imperial ? retract_target / MM_PER_INCH : retract_target;
                snprintf(cmd, sizeof(cmd), "G53G0Z%.4f", value);
                retract_queued = grbl.enqueue_gcode(cmd);
            } else if(state == STATE_IDLE && plan_get_current_block() == NULL &&
                       fabsf(z - retract_target) < 0.01f) {
                retract = Retract_SpindleOff;
                retract_queued = false;
            }
            break;

        case Retract_SpindleOff:
            if(!retract_queued)
                retract_queued = grbl.enqueue_gcode("M5M9");
            else if(state == STATE_IDLE && !gc_state.modal.spindle[0].state.on) {
                if(z < retract_target - 0.01f) {    // never expected: nothing else can move it
                    retract_fail("Flight recorder: Z below the retract height, machine stopped");
                    break;
                }
                rec.state = FR_Retracted;
                record_write();
                log_job();
                retract = Retract_Idle;
                retract_queued = false;
                system_raise_alarm(FR_ALARM);
            }
            break;

        default:
            break;
    }
}

/* ── Watchdog and boot safety ─────────────────────────────────────────────── */

static uint8_t boot_stop = 0;           // fr_boot_stop_t: why the spindle was stopped at boot
static uint32_t boot_stop_again = 0;    // when to stop it once more, 0 until logged

#if FR_WATCHDOG_ENABLE

static enum { Watchdog_Unread = 0, Watchdog_Off, Watchdog_On } watchdog = Watchdog_Unread;

static void fr_execute_delay (sys_state_t state)
{
    if(watchdog == Watchdog_On)
        fr_watchdog_feed();

    on_execute_delay(state);
}

#endif

static void boot_poll (uint32_t now)
{
#if FR_WATCHDOG_ENABLE
    if(watchdog == Watchdog_On)
        fr_watchdog_feed();
    else if(watchdog == Watchdog_Unread) {     // read once: $793 takes effect at the next boot
        if(config.watchdog) {
            fr_watchdog_start(config.watchdog < 3 ? 3 : (config.watchdog > 10 ? 10 : config.watchdog));
            watchdog = Watchdog_On;
        } else
            watchdog = Watchdog_Off;
    }
#endif

    // The core switches the spindle and coolant off at a cold start: log why, and
    // switch them off once more two seconds on, for a Modbus VFD that missed it.
    if(boot_stop) {
        if(!boot_stop_again) {
            fr_log_event(FR_Event_SpindleOff, boot_stop, 0);
            boot_stop_again = now + 2000;
        } else if((int32_t)(now - boot_stop_again) >= 0) {
            if(!gc_state.modal.spindle[0].state.on)
                spindle_all_off(true);
            if(!gc_state.modal.coolant.value)
                hal.coolant.set_state((coolant_state_t){0});
            boot_stop = 0;
        }
    }
}

/* ── Realtime polling ─────────────────────────────────────────────────────── */

// Nothing more from any client until the job is settled (retracted or stopped):
// a client taking over must not carry on the dead sender's job.
static void link_gone (void)
{
    link_lost = rec.flags.usb ? FR_Reason_UsbClosed : FR_Reason_NetworkClosed;
    input_block(true);
    report_message("Flight recorder: sender link lost", Message_Warning);
}

static void fr_execute_realtime (sys_state_t state)
{
    on_execute_realtime(state);

    uint32_t now = hal.get_elapsed_ticks();

    boot_poll(now);

    if(!storage_ok)
        return;

    fr_log_poll(now);

    if(reset_seen) {
        reset_seen = false;
        if(retract != Retract_Idle)
            retract_end("Flight recorder: retract stopped by reset");
        else if(rec.state == FR_Running)
            record_stop(FR_Reason_Reset, Alarm_None);
    }

    // Input comes back once the job is settled and the half-received line has gone.
    if(retract == Retract_Idle && rec.state != FR_Running && hal.stream.read == blocked_read && !discard)
        input_block(false);

    if(retract != Retract_Idle) {
        retract_step(state, now);
        return;
    }

    if(rec.state == FR_Running) {

        // Follow the line the steppers are executing. Lines without an
        // N-number (most macros) report 0 and are skipped; so are jogs,
        // whose N-number isn't the file's.
        uint32_t line = st_get_executing_line_number();
        if(line != line_seen) {
            line_seen = line;
            if(line && state != STATE_JOG) {
                rec.line = line;
                record_snapshot();
                dirty = true;
            }
        }

        // Not for a file run from the SD card: it can't hang, and its input isn't the sender's.
        if(hal.stream.type != StreamType_File) {

            // The sender's link: the stream changing away from it (also caught as it
            // happens, in fr_stream_changed), or closing.
            if(link_lost == FR_Reason_None &&
                (hal.stream.type != job_link.type || hal.stream.state.is_usb != rec.flags.usb ||
                  (job_link.watch_connection && hal.stream.is_connected && !hal.stream.is_connected())))
                link_gone();
            else if(link_lost)
                input_block(true);

            bool queue_empty = state == STATE_IDLE && plan_get_current_block() == NULL &&
                                (hal.stream.get_rx_buffer_count == NULL || hal.stream.get_rx_buffer_count() == 0);
            bool starved = queue_empty && gc_state.modal.spindle[0].state.on;

            if(!starved)
                quiet_since = now;
            else if((int32_t)(busy_at - quiet_since) > 0) // a dwell or spin-up is not starvation
                quiet_since = busy_at;

            if(starved && (link_lost || now - quiet_since >= (uint32_t)(config.timeout * 1000.0f)))
                stream_lost(link_lost ? link_lost : FR_Reason_StreamTimeout);
            else if(link_lost && queue_empty) // link gone, spindle off: nothing to retract
                record_stop(link_lost, Alarm_None);
        }
    }

    if(dirty && now - last_write >= FR_WRITE_INTERVAL)
        record_write();
}

/* ── Core hooks ───────────────────────────────────────────────────────────── */

static void fr_delay_ms (uint32_t ms, delay_callback_ptr callback)
{
    busy_at = hal.get_elapsed_ticks();
    delay_ms(ms, callback);
}

static void fr_state_change (sys_state_t state)
{
    if(on_state_change)
        on_state_change(state);

    if(state & (STATE_ALARM|STATE_ESTOP))
        fr_log_event(FR_Event_Alarm, (uint8_t)sys.alarm, st_get_executing_line_number());

    // After a reset, the alarm that follows (homing required, cycle aborted) isn't
    // why the job stopped: the poll records the reset. A critical alarm (e-stop,
    // limit) also resets, and is the reason.
    if(rec.state == FR_Running && retract == Retract_Idle && (state & (STATE_ALARM|STATE_ESTOP)) &&
        !(reset_seen && !alarm_is_critical(sys.alarm))) {
        record_snapshot();
        rec.state = FR_Stopped;
        rec.reason = FR_Reason_Alarm;
        rec.alarm = (uint8_t)sys.alarm;
        dirty = true;
        log_job();
    }
}

static void fr_program_completed (program_flow_t program_flow, bool check_mode)
{
    if(on_program_completed)
        on_program_completed(program_flow, check_mode);

    if(!check_mode && rec.state == FR_Running) {
        record_snapshot();
        rec.state = FR_Completed;
        rec.reason = FR_Reason_None;
        dirty = true;
        log_job();
    }
}

// Interrupt context: just note it, the realtime poll does the rest.
static void fr_reset (void)
{
    reset_seen = true;

    if(on_reset)
        on_reset();
}

static void fr_stream_changed (stream_type_t type)
{
    if(on_stream_changed)
        on_stream_changed(type);

    // Any change of input mid-job is the sender losing it, even when a new client
    // takes over before the next poll could see it. Caught here, before the new
    // stream is read. A file run from the SD card, and its end, are not.
    bool file = type == StreamType_File || stream_was == StreamType_File;
    stream_was = type;

    if(rec.state == FR_Running && retract == Retract_Idle && link_lost == FR_Reason_None && !file)
        link_gone();
    else if(retract != Retract_Idle || (rec.state == FR_Running && link_lost))
        input_block(true);
}

/* ── $ commands ───────────────────────────────────────────────────────────── */

static status_code_t job_start (sys_state_t state, char *args)
{
    char *end;
    uint32_t job = strtoul(args, &end, 16);
    uint32_t flags = *end == ',' ? strtoul(end + 1, NULL, 10) : 0;

    if(end == args || !(*end == '\0' || *end == ','))
        return Status_InvalidStatement;

    if(state != STATE_IDLE)
        return Status_IdleError;

    // Stamped again after it stopped: a resume. Its re-entry moves have no N-numbers,
    // so keep the line until a numbered line runs, or a second cut would lose it.
    bool resume = job == rec.job && (rec.state == FR_Stopped || rec.state == FR_Retracted);

    rec.state = FR_Running;
    rec.reason = FR_Reason_None;
    rec.alarm = 0;
    rec.job = job;
    if(!resume)
        rec.line = 0;
    rec.flags.value = 0;
    rec.flags.no_retract = !!(flags & 1);
    rec.flags.usb = hal.stream.state.is_usb;
    rec.flags.network = hal.stream.type == StreamType_WebSocket || hal.stream.type == StreamType_Telnet;
    record_snapshot();
    record_write();
    log_job();

    job_link.type = hal.stream.type;
    job_link.watch_connection = hal.stream.is_connected && hal.stream.is_connected();
    link_lost = FR_Reason_None;
    line_seen = st_get_executing_line_number();
    quiet_since = hal.get_elapsed_ticks();

    if(config.auto_retract && !rec.flags.no_retract && !rec.flags.homed)
        report_message("Flight recorder: Z not homed, no auto retract for this job", Message_Warning);

    report_record();

    return Status_OK;
}

static status_code_t cmd_flight (sys_state_t state, char *args)
{
    if(args)
        return job_start(state, args);

    report_record();

    return Status_OK;
}

static status_code_t cmd_flight_clear (sys_state_t state, char *args)
{
    memset(&rec.mpos, 0, sizeof(rec.mpos));
    rec.state = FR_Idle;
    rec.reason = FR_Reason_None;
    rec.alarm = 0;
    rec.job = 0;
    rec.line = 0;
    rec.flags.value = 0;
    record_write();
    report_record();

    return Status_OK;
}

static status_code_t cmd_events (sys_state_t state, char *args)
{
    uint32_t count = args ? strtoul(args, NULL, 10) : 100;

    fr_log_report(count ? count : 100);

    return Status_OK;
}

static status_code_t cmd_events_clear (sys_state_t state, char *args)
{
    fr_log_clear();

    return Status_OK;
}

#if FR_WATCHDOG_ENABLE

// Hang as a stuck controller would, until the watchdog resets the board.
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

// The queries also work during an e-stop or other blocking event.
static const sys_command_t fr_commands[] = {
    { "FLIGHT", cmd_flight, { .allow_blocking = On }, { .str = "report the flight record; =<job>[,<flags>] starts recording a job" } },
    { "FLIGHTCLEAR", cmd_flight_clear, { .noargs = On }, { .str = "clear the flight record" } },
    { "EVENTS", cmd_events, { .allow_blocking = On }, { .str = "list the last 100 events; =<n> the last n" } },
    { "EVENTSCLEAR", cmd_events_clear, { .noargs = On }, { .str = "clear the event log" } },
#if FR_WATCHDOG_ENABLE
    { "WATCHDOGTEST", cmd_watchdog_test, { .noargs = On }, { .str = "hang until the watchdog resets the board" } },
#endif
};

static void fr_report_options (bool newopt)
{
    if(on_report_options)
        on_report_options(newopt);

    if(!newopt)
        report_plugin(storage_ok ? "Flight Recorder" : "Flight Recorder (no storage)", FR_VERSION);
}

/* ── Settings ─────────────────────────────────────────────────────────────── */

static const setting_detail_t fr_settings[] = {
    { FR_SETTING_RETRACT, Group_General, "Auto retract on stream loss", NULL, Format_Bool, NULL, NULL, NULL, Setting_NonCore, &config.auto_retract },
    { FR_SETTING_TIMEOUT, Group_General, "Stream loss timeout", "s", Format_Decimal, "#0.0", "0.5", "60", Setting_NonCore, &config.timeout },
    { FR_SETTING_DEPTH,   Group_General, "Retract below machine zero", "mm", Format_Decimal, "##0.0", "1", "500", Setting_NonCore, &config.retract_depth },
#if FR_WATCHDOG_ENABLE
    // 3-10 s, or 0 for off (1 and 2 act as 3). Takes effect at the next boot.
    { FR_SETTING_WATCHDOG, Group_General, "Watchdog timeout", "s", Format_Int8, "#0", "0", "10", Setting_NonCore, &config.watchdog }
#endif
};

static void fr_settings_save (void)
{
    hal.nvs.memcpy_to_nvs(nvs_addr, (uint8_t *)&config, sizeof(config), true);
}

static void fr_settings_restore (void)
{
    config.auto_retract = true;
    config.timeout = 3.0f;
    config.retract_depth = 20.0f;
    config.watchdog = 5;
    fr_settings_save();
}

static void fr_settings_load (void)
{
    if(hal.nvs.memcpy_from_nvs((uint8_t *)&config, nvs_addr, sizeof(config), true) != NVS_TransferResult_OK)
        fr_settings_restore();
}

/* ── Public init ──────────────────────────────────────────────────────────── */

void flight_recorder_init (void)
{
    static setting_details_t settings = {
        .settings   = fr_settings,
        .n_settings = sizeof(fr_settings) / sizeof(setting_detail_t),
        .load       = fr_settings_load,
        .save       = fr_settings_save,
        .restore    = fr_settings_restore
    };

    static const alarm_detail_t alarms[] = {
        { FR_ALARM, "Stream lost mid-job: tool retracted, spindle stopped. Resume from the flight record." },
        { FR_ALARM_STOPPED, "Stream lost mid-job, and the retract couldn't finish: machine stopped where it was. Check the tool before resuming." }
    };

    static alarm_details_t alarm_details = {
        .n_alarms = sizeof(alarms) / sizeof(alarm_detail_t),
        .alarms   = alarms
    };

    static sys_commands_t commands = {
        .n_commands = sizeof(fr_commands) / sizeof(sys_command_t),
        .commands   = fr_commands
    };

    if(!(nvs_addr = nvs_alloc(sizeof(config))))
        return;

    settings_register(&settings);
    alarms_register(&alarm_details);

    uint8_t reset_cause = fr_reset_cause();

    // A brown-out flag on its own: a power-on sets it too.
    if(reset_cause & FR_Reset_Watchdog)
        boot_stop |= FR_BootStop_Watchdog;
    if((reset_cause & FR_Reset_BrownOut) && !(reset_cause & FR_Reset_PowerOn))
        boot_stop |= FR_BootStop_BrownOut;

    // A record found Running means the controller restarted mid-job.
    if((storage_ok = fr_storage_init())) {
        if(!record_load())
            memset(&rec, 0, sizeof(rec));
        uint8_t found = rec.state;
        rec.boot++;
        if(rec.state == FR_Running) {
            rec.state = FR_Stopped;
            rec.reason = FR_Reason_Reboot;
            boot_stop |= FR_BootStop_Job;
        }
        record_write();
        fr_log_init(rec.boot, reset_cause, found);
        system_register_commands(&commands);
    }

    on_execute_realtime = grbl.on_execute_realtime;
    grbl.on_execute_realtime = fr_execute_realtime;

    on_state_change = grbl.on_state_change;
    grbl.on_state_change = fr_state_change;

    on_program_completed = grbl.on_program_completed;
    grbl.on_program_completed = fr_program_completed;

    on_reset = grbl.on_reset;
    grbl.on_reset = fr_reset;

    stream_was = hal.stream.type;
    on_stream_changed = grbl.on_stream_changed;
    grbl.on_stream_changed = fr_stream_changed;

    on_report_options = grbl.on_report_options;
    grbl.on_report_options = fr_report_options;

    delay_ms = hal.delay_ms;
    hal.delay_ms = fr_delay_ms;

#if FR_WATCHDOG_ENABLE
    on_execute_delay = grbl.on_execute_delay;
    grbl.on_execute_delay = fr_execute_delay;
#endif
}

#endif /* FLIGHT_RECORDER_ENABLE */
