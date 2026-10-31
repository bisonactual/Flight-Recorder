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
  them opt out (flag 1 of $FRJ). Without Z homed there is no retract.

  Event log (event_log.c): resets and their cause, alarms, error replies,
  inputs, limits, the sender's link and Modbus errors, in order, across
  reboots: what to read when a machine misbehaves "for no reason".

  Commands:
    $FR                     — report the record
    $FRJ=<job>[,<flags>]    — start recording a job (job id in hex;
                              flags: 1 = no auto retract)
    $FRC                    — clear the record
    $FRLOG[=<n>]            — the last n events (default 100, up to 1000)
    $FRLOGC                 — clear the event log

  Settings:
    $790 — Auto retract on stream loss (0/1)
    $791 — Stream loss timeout, s
    $792 — Retract to this far below machine zero, mm
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
#include "grbl/system.h"
#include "grbl/state_machine.h"
#include "grbl/settings.h"
#include "grbl/alarms.h"
#include "grbl/report.h"

#include "flight_recorder.h"
#include "event_log.h"

#define FR_SETTING_RETRACT  790
#define FR_SETTING_TIMEOUT  791
#define FR_SETTING_DEPTH    792

#define FR_ALARM            ((alarm_code_t)80)

#define FR_WRITE_INTERVAL   20u     // ms between writes while the line changes
#define FR_RETRACT_LIMIT    120000u // ms before a retract that hasn't finished is given up

typedef struct {
    bool  auto_retract;
    float timeout;          // s
    float retract_depth;    // mm below machine zero
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

static struct {
    stream_type_t type;
    bool usb;
    bool watch_connection;
} job_link;

static enum { Retract_Idle = 0, Retract_Move, Retract_SpindleOff } retract = Retract_Idle;
static bool retract_queued = false;
static uint32_t retract_started = 0;
static float retract_target = 0.0f;

static on_execute_realtime_ptr     on_execute_realtime = NULL;
static on_state_change_ptr         on_state_change = NULL;
static on_program_completed_ptr    on_program_completed = NULL;
static on_reset_ptr                on_reset = NULL;
static on_stream_changed_ptr       on_stream_changed = NULL;
static on_report_options_ptr       on_report_options = NULL;
static on_unknown_sys_command_ptr  prev_on_unknown_sys_command = NULL;
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
// leaves the previous record intact.
static void record_write (void)
{
    if(!storage_ok)
        return;

    rec.magic = FR_MAGIC;
    rec.seq++;
    rec.crc = crc32((const uint8_t *)&rec, offsetof(fr_record_t, crc));
    slot ^= 1;
    fr_log_storage_result(fr_storage_write(slot * FR_SLOT_SIZE, &rec, sizeof(fr_record_t)));

    dirty = false;
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

    slot = ok[0] && ok[1] ? (r[1].seq > r[0].seq) : ok[1];
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
             "[FR:%s|why=%s|job=%08lX|ln=%lu|boot=%lu|alarm=%u|tool=%ld|wcs=%s|flags=%02X|mpos=%.3f,%.3f,%.3f,%.3f,%.3f]",
             fr_state_name(rec.state), fr_reason_name(rec.reason), (unsigned long)rec.job,
             (unsigned long)rec.line, (unsigned long)rec.boot, rec.alarm, (long)rec.tool,
             gc_coord_system_to_str((coord_system_id_t)rec.wcs), rec.flags.value,
             rec.mpos[0], rec.mpos[1], rec.mpos[2], rec.mpos[3], rec.mpos[4]);
    hal.stream.write(buf);
    hal.stream.write(ASCII_EOL);
}

/* ── Input blocking (during the retract) ──────────────────────────────────── */

static int32_t blocked_read (void)
{
    return SERIAL_NO_DATA;
}

static void input_block (bool on)
{
    if(on) {
        if(hal.stream.read != blocked_read) {
            stream_read = hal.stream.read;
            hal.stream.read = blocked_read;
        }
        if(hal.stream.reset_read_buffer)
            hal.stream.reset_read_buffer();
    } else if(hal.stream.read == blocked_read)
        hal.stream.read = stream_read;
}

/* ── Stream-loss retract ──────────────────────────────────────────────────── */

static void retract_end (const char *message)
{
    input_block(false);
    retract = Retract_Idle;
    retract_queued = false;
    report_message(message, Message_Warning);
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
    retract = Retract_Move;
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
        retract_end("Flight recorder: retract did not finish");
        return;
    }

    if(hal.stream.read != blocked_read) // stream changed under us
        input_block(true);
    else if(hal.stream.reset_read_buffer)
        hal.stream.reset_read_buffer();

    switch(retract) {

        case Retract_Move:
            if(!retract_queued) {
                if(z >= retract_target - 0.001f) {   // already above it
                    retract = Retract_SpindleOff;
                    break;
                }
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
                rec.state = FR_Retracted;
                record_write();
                log_job();
                input_block(false);
                retract = Retract_Idle;
                retract_queued = false;
                system_raise_alarm(FR_ALARM);
            }
            break;

        default:
            break;
    }
}

/* ── Realtime polling ─────────────────────────────────────────────────────── */

static fr_reason_t link_reason (void)
{
    return job_link.usb ? FR_Reason_UsbClosed : FR_Reason_NetworkClosed;
}

static void fr_execute_realtime (sys_state_t state)
{
    on_execute_realtime(state);

    if(!storage_ok)
        return;

    uint32_t now = hal.get_elapsed_ticks();

    fr_log_poll(now);

    if(reset_seen) {
        reset_seen = false;
        if(retract != Retract_Idle)
            retract_end("Flight recorder: retract stopped by reset");
        else if(rec.state == FR_Running)
            record_stop(FR_Reason_Reset, Alarm_None);
    }

    if(retract != Retract_Idle) {
        retract_step(state, now);
        return;
    }

    if(rec.state == FR_Running) {

        // Follow the line the steppers are executing. Lines without an
        // N-number (jogs, macros) report 0 and are skipped.
        uint32_t line = st_get_executing_line_number();
        if(line != line_seen) {
            line_seen = line;
            if(line) {
                rec.line = line;
                record_snapshot();
                dirty = true;
            }
        }

        // The sender's link: the stream changing away from it, or closing.
        if(link_lost == FR_Reason_None) {
            if(hal.stream.type != job_link.type || hal.stream.state.is_usb != job_link.usb ||
                (job_link.watch_connection && hal.stream.is_connected && !hal.stream.is_connected())) {
                link_lost = link_reason();
                report_message("Flight recorder: sender link lost", Message_Warning);
            }
        }

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

    if(rec.state == FR_Running && retract == Retract_Idle && (state & (STATE_ALARM|STATE_ESTOP))) {
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

    if(retract != Retract_Idle)
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

    memset(&rec.mpos, 0, sizeof(rec.mpos));
    rec.state = FR_Running;
    rec.reason = FR_Reason_None;
    rec.alarm = 0;
    rec.job = job;
    rec.line = 0;
    rec.flags.value = 0;
    rec.flags.no_retract = !!(flags & 1);
    rec.flags.usb = hal.stream.state.is_usb;
    rec.flags.network = hal.stream.type == StreamType_WebSocket || hal.stream.type == StreamType_Telnet;
    record_snapshot();
    record_write();
    log_job();

    job_link.type = hal.stream.type;
    job_link.usb = hal.stream.state.is_usb;
    job_link.watch_connection = hal.stream.is_connected && hal.stream.is_connected();
    link_lost = FR_Reason_None;
    line_seen = st_get_executing_line_number();
    quiet_since = hal.get_elapsed_ticks();

    if(config.auto_retract && !rec.flags.no_retract && !rec.flags.homed)
        report_message("Flight recorder: Z not homed, no auto retract for this job", Message_Warning);

    report_record();

    return Status_OK;
}

static status_code_t fr_sys_command (sys_state_t state, char *line)
{
    if(storage_ok) {

        if(strcmp(line, "FR") == 0) {
            report_record();
            return Status_OK;
        }

        if(strncmp(line, "FRJ=", 4) == 0)
            return job_start(state, line + 4);

        if(strcmp(line, "FRLOG") == 0 || strncmp(line, "FRLOG=", 6) == 0) {
            uint32_t count = line[5] == '=' ? strtoul(line + 6, NULL, 10) : 100;
            fr_log_report(count ? count : 100);
            return Status_OK;
        }

        if(strcmp(line, "FRLOGC") == 0) {
            fr_log_clear();
            return Status_OK;
        }

        if(strcmp(line, "FRC") == 0) {
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
    }

    return prev_on_unknown_sys_command
        ? prev_on_unknown_sys_command(state, line)
        : Status_Unhandled;
}

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
    { FR_SETTING_DEPTH,   Group_General, "Retract below machine zero", "mm", Format_Decimal, "##0.0", "1", "500", Setting_NonCore, &config.retract_depth }
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
        { FR_ALARM, "Stream lost mid-job: tool retracted, spindle stopped. Resume from the flight record." }
    };

    static alarm_details_t alarm_details = {
        .n_alarms = sizeof(alarms) / sizeof(alarm_detail_t),
        .alarms   = alarms
    };

    if(!(nvs_addr = nvs_alloc(sizeof(config))))
        return;

    settings_register(&settings);
    alarms_register(&alarm_details);

    uint8_t reset_cause = fr_reset_cause();

    // A record found Running means the controller restarted mid-job.
    if((storage_ok = fr_storage_init())) {
        if(!record_load())
            memset(&rec, 0, sizeof(rec));
        uint8_t found = rec.state;
        rec.boot++;
        if(rec.state == FR_Running) {
            rec.state = FR_Stopped;
            rec.reason = FR_Reason_Reboot;
        }
        record_write();
        fr_log_init(rec.boot, reset_cause, found);
    }

    on_execute_realtime = grbl.on_execute_realtime;
    grbl.on_execute_realtime = fr_execute_realtime;

    on_state_change = grbl.on_state_change;
    grbl.on_state_change = fr_state_change;

    on_program_completed = grbl.on_program_completed;
    grbl.on_program_completed = fr_program_completed;

    on_reset = grbl.on_reset;
    grbl.on_reset = fr_reset;

    on_stream_changed = grbl.on_stream_changed;
    grbl.on_stream_changed = fr_stream_changed;

    on_report_options = grbl.on_report_options;
    grbl.on_report_options = fr_report_options;

    prev_on_unknown_sys_command = grbl.on_unknown_sys_command;
    grbl.on_unknown_sys_command = fr_sys_command;

    delay_ms = hal.delay_ms;
    hal.delay_ms = fr_delay_ms;
}

#endif /* FLIGHT_RECORDER_ENABLE */
