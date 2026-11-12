/*

  event_log.c - event log for the flight recorder

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
  Keeps the last 1000 events (16 bytes each) in the recorder's storage, after
  the job record, so they survive reboots. It's meant for the faults that are
  hard to pin down on a CNC controller, which are mostly electrical noise:

    BOOT         every start, with the reset cause and the job state found
    ALARM        every alarm, with the line executing
    ERROR        every error reply to the sender (garbled lines end up here)
    INPUT        e-stop, feed hold, cycle start, door, reset, probe and motor
                 fault inputs going active
    LIMIT        limit switches tripping outside homing
    LINK         the sender's USB port or network client coming and going
    MODBUS       Modbus timeouts, CRC errors, exceptions and retries, per second
    JOB          job start, stop, retract and end
    STORAGE      storage writes that failed, logged once writes work again
    SPINDLE_OFF  the spindle stop at boot after an unexpected reset

  Inputs and limits come in on interrupts, which only latch them. The realtime
  poll writes them out. If the same event comes again within a second it is
  counted in the existing entry instead of taking a new one, so a chattering
  switch or a burst of the same error doesn't wipe out the history.

  On FRAM each event is written as it happens. On an SD card events wait in
  RAM and go out together about once a second, with one header write. Alarms,
  a lost link, resets, stream timeouts and start-ups go out at once. The
  storage says where the ring and its header are and how many slots there are
  (fr_storage_info()).
*/

// Before driver.h, see flight_recorder.c.
#include "grbl/hal.h"
#include "grbl/stepper.h"

#include "driver.h"

#if FLIGHT_RECORDER_ENABLE

#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "grbl/gcode.h"
#include "grbl/report.h"
#include "grbl/state_machine.h"
#include "grbl/system.h"
#if MODBUS_ENABLE
#include "grbl/modbus.h"
#endif

#include "flight_recorder.h"
#include "event_log.h"

#define LOG_MAGIC       0x31474C46u     // "FLG1"
#define REPEAT_WINDOW   1000u           // ms
#define MODBUS_PERIOD   1000u           // ms
#define BATCH_PERIOD    1000u           // ms events wait in RAM, when the storage batches them

// The inputs worth logging are the ones noise can trip. Probe contact, block delete,
// single block and so on are just normal operation.
#define LOGGED_INPUTS ((control_signals_t){ .reset = On, .feed_hold = On, .cycle_start = On, .safety_door_ajar = On, \
                                            .e_stop = On, .probe_disconnected = On, .motor_fault = On, .motor_warning = On, \
                                            .tls_overtravel = On, .probe_overtravel = On }).mask

typedef struct {
    uint32_t magic;
    uint16_t head;          // slot the next event goes in
    uint16_t count;         // events held, up to FR_LOG_EVENTS
    uint16_t next_index;
    uint8_t reserved[5];
    uint8_t check;
} log_header_t;

_Static_assert(sizeof(log_header_t) == FR_LOG_HEADER_SIZE, "FR_LOG_HEADER_SIZE must be the size of log_header_t");

static bool log_ready = false;
static log_header_t header;
static uint16_t boot;

// Where the log is, from the storage.
static uint32_t log_header, log_entries;
static uint16_t ring;                   // slots, FR_LOG_EVENTS or more. At most FR_LOG_EVENTS are reported
static bool batching = false;
static bool batch_pending = false;
static uint32_t batch_since;

// The last event written, kept so that repeats can be counted in it.
static fr_event_t last_event;
static uint16_t last_slot;
static bool have_last = false, last_changed = false;
static uint32_t last_seen, last_flushed;

// Set from interrupts, picked up by the poll.
static volatile uint_fast16_t inputs_latched = 0;
static volatile uint_fast16_t limits_latched = 0;

static uint8_t link_flags = 0xFF;
static stream_type_t link_type;
static uint32_t storage_failures = 0, storage_recovered = 0;

static control_signals_callback_ptr control_interrupt;
static limit_interrupt_callback_ptr limit_interrupt;
static status_message_ptr status_message;
static on_report_handlers_init_ptr on_report_handlers_init;

// Storage

static uint8_t crc8 (const uint8_t *data, uint32_t size)
{
    uint8_t crc = 0;

    while(size--) {
        crc ^= *data++;
        for(uint_fast8_t bit = 0; bit < 8; bit++)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    }

    return crc;
}

void fr_log_storage_result (bool ok)
{
    if(!ok)
        storage_failures++;
    else if(storage_failures) {
        storage_recovered += storage_failures;
        storage_failures = 0;
    }
}

// Without batching each write goes out at once. On FRAM it already has and the flush does nothing.
static void write_header (void)
{
    header.magic = LOG_MAGIC;
    header.check = crc8((const uint8_t *)&header, offsetof(log_header_t, check));
    fr_storage_write(log_header, &header, sizeof(log_header_t));
    if(!batching)
        fr_storage_flush();
}

static void write_slot (uint16_t slot, fr_event_t *event)
{
    event->check = crc8((const uint8_t *)event, offsetof(fr_event_t, check));
    fr_storage_write(log_entries + slot * sizeof(fr_event_t), event, sizeof(fr_event_t));
    if(!batching)
        fr_storage_flush();
}

static bool read_slot (uint16_t slot, fr_event_t *event)
{
    return fr_storage_read(log_entries + slot * sizeof(fr_event_t), event, sizeof(fr_event_t)) &&
            event->type >= FR_Event_Boot && event->type <= FR_Event_SpindleOff &&
             event->check == crc8((const uint8_t *)event, offsetof(fr_event_t, check));
}

// If the header is torn or unreadable, work it out again from the events themselves,
// which carry their own running number. This reads the whole ring once, which takes
// about half a second on the A7.
static void rebuild_header (void)
{
    fr_event_t event;
    bool found = false;

    memset(&header, 0, sizeof(log_header_t));

    for(uint16_t slot = 0; slot < ring; slot++) {
        if(read_slot(slot, &event)) {
            if(header.count < FR_LOG_EVENTS)
                header.count++;
            if(!found || (int16_t)(event.index - header.next_index) >= 0) {
                header.next_index = event.index + 1;
                header.head = (slot + 1) % ring;
                found = true;
            }
        }
    }
}

static void note_batch (uint32_t now)
{
    if(!batch_pending) {
        batch_pending = true;
        batch_since = now;
    }
}

static void flush_last_event (void)
{
    if(last_changed) {
        write_slot(last_slot, &last_event);
        last_changed = false;
        last_flushed = hal.get_elapsed_ticks();
        if(batching)
            note_batch(last_flushed);
    }
}

// Writes out the batch: the last event's repeat count, the header, and whatever the
// storage still has in RAM.
static void flush_log (void)
{
    flush_last_event();
    write_header();
    fr_storage_flush();
    batch_pending = false;
}

// Written at once rather than batched, as they matter most if the power goes next:
// alarms, a lost link, resets and start-ups, and a job stopping for any of those or
// for a stream timeout.
static bool is_urgent (fr_event_type_t type, uint8_t code, uint32_t data)
{
    switch(type) {

        case FR_Event_Boot:
        case FR_Event_Alarm:
            return true;

        case FR_Event_Link:
            return !(code & FR_Link_Connected);

        case FR_Event_Input:
            return !!(data & ((control_signals_t){ .reset = On }).mask);

        case FR_Event_Job:
            return (code >> 4) != FR_Reason_None;

        default:
            return false;
    }
}

static void append_event (fr_event_type_t type, uint8_t code, uint32_t data, uint32_t now)
{
    flush_last_event();

    last_event = (fr_event_t){ .time = now, .data = data, .boot = boot, .index = header.next_index,
                               .type = (uint8_t)type, .code = code };
    last_slot = header.head;
    write_slot(last_slot, &last_event);

    header.next_index++;
    header.head = (header.head + 1) % ring;
    if(header.count < FR_LOG_EVENTS)
        header.count++;
    if(batching)
        note_batch(now);
    else
        write_header();

    have_last = true;
    last_seen = last_flushed = now;
}

// Logging

// Inputs, limits and links only count as a repeat if the same signals are involved.
// Everything else repeats by type and code alone.
static bool repeat_needs_same_data (uint8_t type)
{
    return type == FR_Event_Input || type == FR_Event_Limit || type == FR_Event_Link;
}

// Adds four packed byte counters, each one stopping at 255.
static uint32_t add_saturating (uint32_t packed, uint32_t more)
{
    uint32_t sum = 0;

    for(uint_fast8_t shift = 0; shift < 32; shift += 8) {
        uint32_t byte = ((packed >> shift) & 0xFF) + ((more >> shift) & 0xFF);
        sum |= (byte > 0xFF ? 0xFF : byte) << shift;
    }

    return sum;
}

void fr_log_event (fr_event_type_t type, uint8_t code, uint32_t data)
{
    if(!log_ready)
        return;

    uint32_t now = hal.get_elapsed_ticks();
    uint32_t window = REPEAT_WINDOW + (type == FR_Event_Modbus ? MODBUS_PERIOD : 0);

    if(have_last && last_event.boot == boot && last_event.type == type && last_event.code == code &&
        (!repeat_needs_same_data(type) || last_event.data == data) && now - last_seen < window) {
        if(last_event.repeats < 0xFF)
            last_event.repeats++;
        if(type == FR_Event_Modbus)
            last_event.data = add_saturating(last_event.data, data);
        last_seen = now;
        last_changed = true;
    } else
        append_event(type, code, data, now);

    if(batching && is_urgent(type, code, data))
        flush_log();
}

// Where the events come from

// Called from an interrupt, so only latch the signals here.
static void onControlInterrupt (control_signals_t signals)
{
    if(!signals.deasserted && (signals.mask & LOGGED_INPUTS))
        hal.set_bits_atomic(&inputs_latched, signals.mask & LOGGED_INPUTS);

    control_interrupt(signals);
}

static void onLimitInterrupt (limit_signals_t state)
{
    if(state.bits && state_get() != STATE_HOMING)
        hal.set_bits_atomic(&limits_latched, state.bits);

    limit_interrupt(state);
}

static status_code_t onStatusMessage (status_code_t status_code)
{
    if(status_code != Status_OK)
        fr_log_event(FR_Event_Error, (uint8_t)status_code, gc_state.line_number);

    return status_message(status_code);
}

// The core sets its report handlers up again after start-up, so wrap ours back in each time.
static void onReportHandlersInit (void)
{
    if(on_report_handlers_init)
        on_report_handlers_init();

    if(grbl.report.status_message != onStatusMessage) {
        status_message = grbl.report.status_message;
        grbl.report.status_message = onStatusMessage;
    }
}

static void poll_link (void)
{
    uint8_t flags = (hal.stream.state.is_usb ? FR_Link_Usb : 0) |
                     (hal.stream.type == StreamType_WebSocket || hal.stream.type == StreamType_Telnet ? FR_Link_Network : 0) |
                      (hal.stream.is_connected && hal.stream.is_connected() ? FR_Link_Connected : 0);

    if(flags != link_flags || hal.stream.type != link_type) {
        link_flags = flags;
        link_type = hal.stream.type;
        fr_log_event(FR_Event_Link, flags, (uint32_t)link_type);
    }
}

#if MODBUS_ENABLE && defined(MODBUS_RTU_STATS)

static void poll_modbus (uint32_t now)
{
    static modbus_rtu_stats_t previous = {0};
    static uint32_t next_poll = 0;

    if((int32_t)(now - next_poll) < 0)
        return;

    next_poll = now + MODBUS_PERIOD;

    // The counters can be reset with $MODBUSSTATS=R, so a smaller value means it is all new.
    const modbus_rtu_stats_t *stats = modbus_rtu_get_stats();
    uint32_t timeouts = stats->timeouts >= previous.timeouts ? stats->timeouts - previous.timeouts : stats->timeouts;
    uint32_t crc_errors = stats->crc_errors >= previous.crc_errors ? stats->crc_errors - previous.crc_errors : stats->crc_errors;
    uint32_t exceptions = stats->rx_exceptions >= previous.rx_exceptions ? stats->rx_exceptions - previous.rx_exceptions : stats->rx_exceptions;
    uint32_t retries = stats->retries >= previous.retries ? stats->retries - previous.retries : stats->retries;

    previous = *stats;

    if(timeouts | crc_errors | exceptions | retries)
        fr_log_event(FR_Event_Modbus, 0, (timeouts > 0xFF ? 0xFF : timeouts) |
                                          (crc_errors > 0xFF ? 0xFF : crc_errors) << 8 |
                                           (exceptions > 0xFF ? 0xFF : exceptions) << 16 |
                                            (retries > 0xFF ? 0xFF : retries) << 24);
}

#endif

void fr_log_poll (uint32_t now)
{
    if(!log_ready)
        return;

    uint32_t inputs = (uint32_t)hal.set_value_atomic(&inputs_latched, 0);
    if(inputs)
        fr_log_event(FR_Event_Input, 0, inputs);

    uint32_t limits = (uint32_t)hal.set_value_atomic(&limits_latched, 0);
    if(limits)
        fr_log_event(FR_Event_Limit, 0, limits);

    poll_link();

#if MODBUS_ENABLE && defined(MODBUS_RTU_STATS)
    poll_modbus(now);
#endif

    if(storage_recovered) {
        uint32_t failed = storage_recovered;
        storage_recovered = 0;
        fr_log_event(FR_Event_Storage, 0, failed);
    }

    if(last_changed && now - last_flushed >= REPEAT_WINDOW)
        flush_last_event();

    if(batch_pending && now - batch_since >= BATCH_PERIOD)
        flush_log();
}

// Reporting

// Comma separated names of the bits set in mask.
static void list_names (char *buf, size_t size, uint32_t mask, const char *const *names, uint_fast8_t count)
{
    size_t len = 0;

    *buf = '\0';
    for(uint_fast8_t i = 0; i < count && len < size; i++) {
        if(mask & (1u << i) && names[i])
            len += snprintf(buf + len, size - len, "%s%s", len ? "," : "", names[i]);
    }
}

static void list_axes (char *buf, size_t size, uint8_t mask)
{
    static const char *const letters[] = { "X", "Y", "Z", "A", "B", "C", "U", "V" };

    list_names(buf, size, mask, letters, 8);
}

static void describe_event (const fr_event_t *event, char *buf, size_t size)
{
    static const char *const inputs[] = { "reset", "feed_hold", "cycle_start", "door", NULL, NULL, "e_stop",
                                          "probe_disconnected", "motor_fault", "motor_warning", NULL, NULL,
                                          "toolsetter_overtravel", "probe_overtravel" };
    static const char *const resets[] = { "power-on", "brown-out", "reset-pin", "software", "watchdog", "low-power" };
    char list[64], list2[64];

    switch(event->type) {

        case FR_Event_Boot: {
            // Several flags can be set at once (a power-on sets brown-out and pin as well),
            // so check them in order of importance and name the first one found.
            static const uint8_t order[] = { 0, 4, 1, 3, 5, 2 };
            const char *cause = "unknown";
            for(uint_fast8_t i = 0; i < sizeof(order); i++) {
                if(event->code & (1u << order[i])) {
                    cause = resets[order[i]];
                    break;
                }
            }
            snprintf(buf, size, "BOOT|reset=%s|flags=%02X|job=%s", cause, event->code, fr_state_name((uint8_t)event->data));
        }   break;

        case FR_Event_Alarm:
            snprintf(buf, size, "ALARM|alarm=%u|ln=%lu", event->code, (unsigned long)event->data);
            break;

        case FR_Event_Error:
            snprintf(buf, size, "ERROR|error=%u|ln=%lu", event->code, (unsigned long)event->data);
            break;

        case FR_Event_Input:
            list_names(list, sizeof(list), event->data, inputs, sizeof(inputs) / sizeof(inputs[0]));
            snprintf(buf, size, "INPUT|%s", list);
            break;

        case FR_Event_Limit: {
            limit_signals_t limits = { .bits = event->data };
            list_axes(list, sizeof(list), limits.min.mask | limits.min2.mask);
            list_axes(list2, sizeof(list2), limits.max.mask | limits.max2.mask);
            snprintf(buf, size, "LIMIT|min=%s|max=%s", *list ? list : "-", *list2 ? list2 : "-");
        }   break;

        case FR_Event_Link:
            snprintf(buf, size, "LINK|%s|%s", event->code & FR_Link_Usb ? "usb" : (event->code & FR_Link_Network ? "network" : "other"),
                                               event->code & FR_Link_Connected ? "connected" : "disconnected");
            break;

        case FR_Event_Modbus:
            snprintf(buf, size, "MODBUS|timeouts=%lu|crc=%lu|exceptions=%lu|retries=%lu",
                      (unsigned long)(event->data & 0xFF), (unsigned long)((event->data >> 8) & 0xFF),
                       (unsigned long)((event->data >> 16) & 0xFF), (unsigned long)(event->data >> 24));
            break;

        case FR_Event_Job:
            if((event->code & 0x0F) == FR_Running)
                snprintf(buf, size, "JOB|start|job=%08lX", (unsigned long)event->data);
            else
                snprintf(buf, size, "JOB|%s|why=%s|ln=%lu", fr_state_name(event->code & 0x0F), fr_reason_name(event->code >> 4),
                          (unsigned long)event->data);
            break;

        case FR_Event_Storage:
            snprintf(buf, size, "STORAGE|failed_writes=%lu", (unsigned long)event->data);
            break;

        case FR_Event_SpindleOff: {
            static const char *const reasons[] = { "job", "watchdog", "brown-out" };
            list_names(list, sizeof(list), event->code, reasons, sizeof(reasons) / sizeof(reasons[0]));
            snprintf(buf, size, "SPINDLE_OFF|after=%s", list);
        }   break;

        default:
            snprintf(buf, size, "TYPE%u|code=%u|data=%lu", event->type, event->code, (unsigned long)event->data);
            break;
    }
}

void fr_log_report (uint32_t count)
{
    char text[112], line[176];
    fr_event_t event;

    if(!log_ready)
        return;

    flush_last_event();

    if(count > header.count)
        count = header.count;

    for(uint32_t i = 0; i < count; i++) {
        uint16_t slot = (uint16_t)((header.head + ring - count + i) % ring);
        if(read_slot(slot, &event)) {
            describe_event(&event, text, sizeof(text));
            snprintf(line, sizeof(line), "[EVENT:%u|boot=%u|t=%lu.%03lu|%s|n=%u]", event.index, event.boot,
                      (unsigned long)(event.time / 1000), (unsigned long)(event.time % 1000), text, event.repeats + 1u);
        } else
            snprintf(line, sizeof(line), "[EVENT:?|unreadable slot %u]", slot);
        hal.stream.write(line);
        hal.stream.write(ASCII_EOL);
    }
}

void fr_log_clear (void)
{
    if(!log_ready)
        return;

    header.head = header.count = 0;
    have_last = last_changed = false;
    write_header();
    fr_storage_flush();
    batch_pending = false;
}

void fr_log_init (uint32_t boot_count, uint8_t reset_cause, uint8_t job_state)
{
    fr_event_t event;
    const fr_storage_info_t *storage = fr_storage_info();

    boot = (uint16_t)boot_count;
    log_header = storage->log_header;
    log_entries = storage->log_entries;
    ring = storage->log_slots;
    batching = storage->batch_events;

    if(!(fr_storage_read(log_header, &header, sizeof(log_header_t)) && header.magic == LOG_MAGIC &&
          header.check == crc8((const uint8_t *)&header, offsetof(log_header_t, check)) &&
           header.head < ring && header.count <= FR_LOG_EVENTS))
        rebuild_header();

    // If the power went after some events were written but before the header, the events
    // are still there, so take them. On FRAM that's at most one, on an SD card a batch.
    for(uint16_t n = 0; n < ring && read_slot(header.head, &event) && event.index == header.next_index; n++) {
        header.next_index++;
        header.head = (header.head + 1) % ring;
        if(header.count < FR_LOG_EVENTS)
            header.count++;
    }

    log_ready = true;
    fr_log_event(FR_Event_Boot, reset_cause, job_state);

    control_interrupt = hal.control.interrupt_callback;
    hal.control.interrupt_callback = onControlInterrupt;

    limit_interrupt = hal.limits.interrupt_callback;
    hal.limits.interrupt_callback = onLimitInterrupt;

    on_report_handlers_init = grbl.on_report_handlers_init;
    grbl.on_report_handlers_init = onReportHandlersInit;
    status_message = grbl.report.status_message;
    grbl.report.status_message = onStatusMessage;
}

#endif // FLIGHT_RECORDER_ENABLE
