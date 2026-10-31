/*
  event_log.c — Flight recorder event log: what happened, in order, across reboots.

  1000 events of 16 bytes in the recorder's storage, after the job record.
  Aimed at the silent faults of a CNC controller, mostly electrical noise:

    BOOT     every start, with why the processor reset (power-on, brown-out,
             reset pin, watchdog, software) and the job state it found
    ALARM    every alarm, with the line executing
    ERROR    every error reply to the sender (garbled lines show up here)
    INPUT    e-stop, feed hold, cycle start, door, reset, probe and motor
             fault inputs going active
    LIMIT    limit switches tripping outside homing
    LINK     the sender's USB port or network client coming and going
    MODBUS   Modbus timeouts, CRC errors, exceptions and retries, per second
    JOB      job start, stop, retract and end
    STORAGE  storage writes that failed (logged once they work again)

  Inputs and limits arrive in interrupts: the interrupt only latches them, and
  the realtime poll writes them. The same event again within a second of the
  last one is counted into its entry, not written anew, so a chattering
  switch or a burst of errors can't wipe the history.
*/

// grblHAL headers before driver.h (see flight_recorder.c).
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
#define LOG_HEADER      FR_LOG_BASE
#define LOG_ENTRIES     (FR_LOG_BASE + 16)
#define COALESCE_MS     1000u
#define MODBUS_PERIOD   1000u

// Inputs worth logging: the ones noise makes trip. Probe contact, block delete,
// single block and the like are normal operation.
#define INPUT_MASK ((control_signals_t){ .reset = On, .feed_hold = On, .cycle_start = On, .safety_door_ajar = On, \
                                         .e_stop = On, .probe_disconnected = On, .motor_fault = On, .motor_warning = On, \
                                         .tls_overtravel = On, .probe_overtravel = On }).mask

typedef struct {
    uint32_t magic;
    uint16_t head;          // slot the next event goes into
    uint16_t count;         // events held, up to FR_LOG_EVENTS
    uint16_t next_index;
    uint8_t  reserved[5];
    uint8_t  check;
} log_header_t;

/* ── Static state ─────────────────────────────────────────────────────────── */

static bool log_ok = false;
static log_header_t hdr;
static uint16_t boot;

// The last event written, kept so repeats can be counted into it.
static fr_event_t last;
static uint16_t last_slot;
static bool last_valid = false, last_dirty = false;
static uint32_t last_seen, last_flush;

// Latched by interrupts, collected by the poll.
static volatile uint_fast16_t inputs_seen = 0;
static volatile uint_fast16_t limits_seen = 0;

static uint8_t link_flags = 0xFF;
static stream_type_t link_type;
static uint32_t storage_failures = 0, storage_recovered = 0;

static control_signals_callback_ptr control_interrupt = NULL;
static limit_interrupt_callback_ptr limit_interrupt = NULL;
static status_message_ptr status_message = NULL;
static on_report_handlers_init_ptr on_report_handlers_init = NULL;

/* ── Storage ──────────────────────────────────────────────────────────────── */

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

static void header_write (void)
{
    hdr.magic = LOG_MAGIC;
    hdr.check = crc8((const uint8_t *)&hdr, offsetof(log_header_t, check));
    fr_log_storage_result(fr_storage_write(LOG_HEADER, &hdr, sizeof(log_header_t)));
}

static void slot_write (uint16_t slot, fr_event_t *e)
{
    e->check = crc8((const uint8_t *)e, offsetof(fr_event_t, check));
    fr_log_storage_result(fr_storage_write(LOG_ENTRIES + slot * sizeof(fr_event_t), e, sizeof(fr_event_t)));
}

static bool slot_read (uint16_t slot, fr_event_t *e)
{
    return fr_storage_read(LOG_ENTRIES + slot * sizeof(fr_event_t), e, sizeof(fr_event_t)) &&
            e->type != 0 && e->check == crc8((const uint8_t *)e, offsetof(fr_event_t, check));
}

static void flush_last (void)
{
    if(last_dirty) {
        slot_write(last_slot, &last);
        last_dirty = false;
        last_flush = hal.get_elapsed_ticks();
    }
}

static void append (fr_event_type_t type, uint8_t code, uint32_t data, uint32_t now)
{
    flush_last();

    last = (fr_event_t){ .time = now, .data = data, .boot = boot, .index = hdr.next_index,
                         .type = (uint8_t)type, .code = code };
    last_slot = hdr.head;
    slot_write(last_slot, &last);

    hdr.next_index++;
    hdr.head = (hdr.head + 1) % FR_LOG_EVENTS;
    if(hdr.count < FR_LOG_EVENTS)
        hdr.count++;
    header_write();

    last_valid = true;
    last_seen = last_flush = now;
}

/* ── Logging ──────────────────────────────────────────────────────────────── */

// Inputs, limits and links repeat only if the same signals do; the rest by kind.
static bool keyed_on_data (uint8_t type)
{
    return type == FR_Event_Input || type == FR_Event_Limit || type == FR_Event_Link;
}

static uint32_t add_saturating (uint32_t packed, uint32_t more)
{
    uint32_t out = 0;

    for(uint_fast8_t shift = 0; shift < 32; shift += 8) {
        uint32_t sum = ((packed >> shift) & 0xFF) + ((more >> shift) & 0xFF);
        out |= (sum > 0xFF ? 0xFF : sum) << shift;
    }

    return out;
}

void fr_log_event (fr_event_type_t type, uint8_t code, uint32_t data)
{
    if(!log_ok)
        return;

    uint32_t now = hal.get_elapsed_ticks();

    if(last_valid && last.boot == boot && last.type == type && last.code == code &&
        (!keyed_on_data(type) || last.data == data) && now - last_seen < COALESCE_MS + (type == FR_Event_Modbus ? MODBUS_PERIOD : 0)) {
        if(last.repeats < 0xFF)
            last.repeats++;
        if(type == FR_Event_Modbus)
            last.data = add_saturating(last.data, data);
        last_seen = now;
        last_dirty = true;
    } else
        append(type, code, data, now);
}

/* ── Sources ──────────────────────────────────────────────────────────────── */

// Interrupt context: latch only.
static void fr_control_interrupt (control_signals_t signals)
{
    if(!signals.deasserted && (signals.mask & INPUT_MASK))
        hal.set_bits_atomic(&inputs_seen, signals.mask & INPUT_MASK);

    control_interrupt(signals);
}

static void fr_limit_interrupt (limit_signals_t state)
{
    if(state.bits && state_get() != STATE_HOMING)
        hal.set_bits_atomic(&limits_seen, state.bits);

    limit_interrupt(state);
}

static status_code_t fr_status_message (status_code_t status_code)
{
    if(status_code != Status_OK)
        fr_log_event(FR_Event_Error, (uint8_t)status_code, gc_state.line_number);

    return status_message(status_code);
}

// The core resets its report handlers after start-up: wrap them again each time.
static void fr_report_handlers_init (void)
{
    if(on_report_handlers_init)
        on_report_handlers_init();

    if(grbl.report.status_message != fr_status_message) {
        status_message = grbl.report.status_message;
        grbl.report.status_message = fr_status_message;
    }
}

static void link_poll (void)
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

static void modbus_poll (uint32_t now)
{
    static modbus_rtu_stats_t prev = {0};
    static uint32_t next = 0;

    if((int32_t)(now - next) < 0)
        return;
    next = now + MODBUS_PERIOD;

    const modbus_rtu_stats_t *s = modbus_rtu_get_stats();
    // Counters can be reset ($MODBUSSTATS=R): a smaller value is all new.
    uint32_t timeouts = s->timeouts >= prev.timeouts ? s->timeouts - prev.timeouts : s->timeouts;
    uint32_t crc = s->crc_errors >= prev.crc_errors ? s->crc_errors - prev.crc_errors : s->crc_errors;
    uint32_t exceptions = s->rx_exceptions >= prev.rx_exceptions ? s->rx_exceptions - prev.rx_exceptions : s->rx_exceptions;
    uint32_t retries = s->retries >= prev.retries ? s->retries - prev.retries : s->retries;
    prev = *s;

    if(timeouts | crc | exceptions | retries)
        fr_log_event(FR_Event_Modbus, 0, add_saturating(0, (timeouts > 0xFF ? 0xFF : timeouts) |
                                                             (crc > 0xFF ? 0xFF : crc) << 8 |
                                                              (exceptions > 0xFF ? 0xFF : exceptions) << 16 |
                                                               (retries > 0xFF ? 0xFF : retries) << 24));
}

#endif

void fr_log_poll (uint32_t now)
{
    if(!log_ok)
        return;

    uint32_t inputs = (uint32_t)hal.set_value_atomic(&inputs_seen, 0);
    if(inputs)
        fr_log_event(FR_Event_Input, 0, inputs);

    uint32_t limits = (uint32_t)hal.set_value_atomic(&limits_seen, 0);
    if(limits)
        fr_log_event(FR_Event_Limit, 0, limits);

    link_poll();

#if MODBUS_ENABLE && defined(MODBUS_RTU_STATS)
    modbus_poll(now);
#endif

    if(storage_recovered) {
        uint32_t failed = storage_recovered;
        storage_recovered = 0;
        fr_log_event(FR_Event_Storage, 0, failed);
    }

    if(last_dirty && now - last_flush >= COALESCE_MS)
        flush_last();
}

/* ── Report ───────────────────────────────────────────────────────────────── */

static void names (char *buf, size_t size, uint32_t mask, const char *const *list, uint_fast8_t n)
{
    size_t len = 0;

    *buf = '\0';
    for(uint_fast8_t i = 0; i < n && len < size; i++) {
        if(mask & (1u << i) && list[i])
            len += snprintf(buf + len, size - len, "%s%s", len ? "," : "", list[i]);
    }
}

static void axes (char *buf, size_t size, uint8_t mask)
{
    static const char *const letters[] = { "X", "Y", "Z", "A", "B", "C", "U", "V" };
    names(buf, size, mask, letters, 8);
}

static void describe (const fr_event_t *e, char *buf, size_t size)
{
    static const char *const inputs[] = { "reset", "feed_hold", "cycle_start", "door", NULL, NULL, "e_stop",
                                          "probe_disconnected", "motor_fault", "motor_warning", NULL, NULL,
                                          "toolsetter_overtravel", "probe_overtravel" };
    static const char *const resets[] = { "power-on", "brown-out", "reset-pin", "software", "watchdog", "low-power" };
    char a[64], b[64];

    switch(e->type) {

        case FR_Event_Boot: {
            // Several flags come together (power-on sets brown-out and pin too): name the first cause.
            uint_fast8_t i = 0;
            uint8_t order[] = { 0, 4, 1, 3, 5, 2 };
            const char *cause = "unknown";
            for(i = 0; i < sizeof(order); i++) {
                if(e->code & (1u << order[i])) {
                    cause = resets[order[i]];
                    break;
                }
            }
            snprintf(buf, size, "BOOT|reset=%s|flags=%02X|job=%s", cause, e->code, fr_state_name((uint8_t)e->data));
        }   break;

        case FR_Event_Alarm:
            snprintf(buf, size, "ALARM|alarm=%u|ln=%lu", e->code, (unsigned long)e->data);
            break;

        case FR_Event_Error:
            snprintf(buf, size, "ERROR|error=%u|ln=%lu", e->code, (unsigned long)e->data);
            break;

        case FR_Event_Input:
            names(a, sizeof(a), e->data, inputs, sizeof(inputs) / sizeof(inputs[0]));
            snprintf(buf, size, "INPUT|%s", a);
            break;

        case FR_Event_Limit: {
            limit_signals_t l = { .bits = e->data };
            axes(a, sizeof(a), l.min.mask | l.min2.mask);
            axes(b, sizeof(b), l.max.mask | l.max2.mask);
            snprintf(buf, size, "LIMIT|min=%s|max=%s", *a ? a : "-", *b ? b : "-");
        }   break;

        case FR_Event_Link:
            snprintf(buf, size, "LINK|%s|%s", e->code & FR_Link_Usb ? "usb" : (e->code & FR_Link_Network ? "network" : "other"),
                                               e->code & FR_Link_Connected ? "connected" : "disconnected");
            break;

        case FR_Event_Modbus:
            snprintf(buf, size, "MODBUS|timeouts=%lu|crc=%lu|exceptions=%lu|retries=%lu",
                      (unsigned long)(e->data & 0xFF), (unsigned long)((e->data >> 8) & 0xFF),
                       (unsigned long)((e->data >> 16) & 0xFF), (unsigned long)(e->data >> 24));
            break;

        case FR_Event_Job:
            if((e->code & 0x0F) == FR_Running)
                snprintf(buf, size, "JOB|start|job=%08lX", (unsigned long)e->data);
            else
                snprintf(buf, size, "JOB|%s|why=%s|ln=%lu", fr_state_name(e->code & 0x0F), fr_reason_name(e->code >> 4), (unsigned long)e->data);
            break;

        case FR_Event_Storage:
            snprintf(buf, size, "STORAGE|failed_writes=%lu", (unsigned long)e->data);
            break;

        default:
            snprintf(buf, size, "TYPE%u|code=%u|data=%lu", e->type, e->code, (unsigned long)e->data);
            break;
    }
}

void fr_log_report (uint32_t count)
{
    char text[112], line[176];
    fr_event_t e;

    if(!log_ok)
        return;

    flush_last();

    if(count > hdr.count)
        count = hdr.count;

    for(uint32_t i = 0; i < count; i++) {
        uint16_t slot = (uint16_t)((hdr.head + FR_LOG_EVENTS - count + i) % FR_LOG_EVENTS);
        if(slot_read(slot, &e)) {
            describe(&e, text, sizeof(text));
            snprintf(line, sizeof(line), "[FRLOG:%u|boot=%u|t=%lu.%03lu|%s|n=%u]", e.index, e.boot,
                      (unsigned long)(e.time / 1000), (unsigned long)(e.time % 1000), text, e.repeats + 1u);
        } else
            snprintf(line, sizeof(line), "[FRLOG:?|unreadable slot %u]", slot);
        hal.stream.write(line);
        hal.stream.write(ASCII_EOL);
    }
}

void fr_log_clear (void)
{
    if(!log_ok)
        return;

    hdr.head = hdr.count = 0;
    header_write();
    last_valid = last_dirty = false;
}

/* ── Init ─────────────────────────────────────────────────────────────────── */

void fr_log_init (uint32_t boot_count, uint8_t reset_cause, uint8_t job_state)
{
    fr_event_t e;

    boot = (uint16_t)boot_count;

    if(!(fr_storage_read(LOG_HEADER, &hdr, sizeof(log_header_t)) && hdr.magic == LOG_MAGIC &&
          hdr.check == crc8((const uint8_t *)&hdr, offsetof(log_header_t, check)) &&
           hdr.head < FR_LOG_EVENTS && hdr.count <= FR_LOG_EVENTS))
        memset(&hdr, 0, sizeof(log_header_t));

    // Power cut between an event and its header: the event is there, take it.
    if(slot_read(hdr.head, &e) && e.index == hdr.next_index) {
        hdr.next_index++;
        hdr.head = (hdr.head + 1) % FR_LOG_EVENTS;
        if(hdr.count < FR_LOG_EVENTS)
            hdr.count++;
    }

    log_ok = true;
    fr_log_event(FR_Event_Boot, reset_cause, job_state);

    control_interrupt = hal.control.interrupt_callback;
    hal.control.interrupt_callback = fr_control_interrupt;

    limit_interrupt = hal.limits.interrupt_callback;
    hal.limits.interrupt_callback = fr_limit_interrupt;

    on_report_handlers_init = grbl.on_report_handlers_init;
    grbl.on_report_handlers_init = fr_report_handlers_init;
    status_message = grbl.report.status_message;
    grbl.report.status_message = fr_status_message;
}

#endif /* FLIGHT_RECORDER_ENABLE */
