/*

  storage_sd.c - flight recorder storage on an SD card, through grblHAL's SD card support

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
  SD card storage, for boards without the FRAM: the Flexi-HAL with the uFlexiNET
  module, whose micro-SD shares an SPI bus with the W5500.

  FLIGHT.DAT holds what the FRAM would, the two record copies and the event log.
  It is made once, at full size, through FatFs. After that we look up the card
  sectors behind it and write to those directly, so the file's size, the FAT and
  its directory entry stay as they are. f_write and f_sync would rewrite the
  directory entry with every record, and a power cut in that sector can take
  other files with it. The file is read only, hidden and system, so grblHAL's
  file commands and FTP leave it alone.

  The three parts are 4 MiB apart:

    0       record copy 0
    4 MiB   log header, then the ring: 33 sectors of 32 events (1056 slots)
    8 MiB   record copy 1

  That puts them in different allocation units on most cards. A card that
  damages more than the sector it was writing, up to a whole unit, can then only
  hurt the copy it was writing and not the one it finished last, and a torn log
  write hurts neither.

  The ring is a sector longer than the 1000 events kept. A sector the log moves
  into is started blank rather than read first, which drops the oldest events in
  it and leaves at least 1024.

  A card can stay busy for 250 ms after a write, sometimes longer, and FatFs
  would sit waiting for it with nothing else running. So writes only go into RAM
  here, and the realtime poll sends one sector at a time when the card is free.
  The sector goes with CMD24 and the card is let go at once. The next poll checks
  whether it has finished (fr_sd_card_busy() in the board file), and the W5500
  has the bus in between. A newer record replaces one still in RAM and goes out
  first, to the copy the card didn't finish last. Log sectors go out when the
  event log asks, ahead of the header, so the header doesn't point at events
  that aren't on the card yet. Only reads wait for the card, at start-up and for
  $EVENTS.
*/

#include "driver.h"
#include "flight_recorder.h"

#if FLIGHT_RECORDER_ENABLE && FR_SD_ENABLE

#include <stdio.h>
#include <string.h>

#ifdef GRBL_SIM
#include "sim_sdcard.h"
#else
#include "sdcard/sdcard.h"
#endif

#include "grbl/hal.h"
#include "grbl/report.h"

#include "storage.h"
#include "event_log.h"

#define SD_SECTOR       512u
#define SD_PATH         "/FLIGHT.DAT"
#define SD_ATTRIBUTES   (AM_RDO|AM_HID|AM_SYS)
#define SD_LOG_SECTORS  33u

#ifndef FR_SD_SPREAD
#define FR_SD_SPREAD    (4u * 1024u * 1024u)    // between the parts, 0 packs them together for comparison
#endif

#define SD_HEADER_AT    (FR_SD_SPREAD ? FR_SD_SPREAD : SD_SECTOR)
#define SD_LOG_AT       (SD_HEADER_AT + SD_SECTOR)
#define SD_COPY1_AT     (FR_SD_SPREAD ? 2u * FR_SD_SPREAD : SD_LOG_AT + SD_LOG_SECTORS * SD_SECTOR)
#define SD_FILE_SIZE    (SD_COPY1_AT + SD_SECTOR)

// The sectors we use, by index: copy 0, the header, the log's 33, copy 1.
#define SD_HEADER       1u
#define SD_LOG_FIRST    2u
#define SD_COPY1_INDEX  (SD_LOG_FIRST + SD_LOG_SECTORS)
#define SD_MAPPED       (SD_COPY1_INDEX + 1)

#define SD_CACHE        3           // event sectors kept in RAM
#define SD_RETRY        100u        // ms before trying again after a failed write
#define SD_STUCK        1000u       // ms busy after which the card counts as failing
#define SD_PAUSE        100u        // ms busy that counts as a long pause in the stats
#define SD_READ_WAIT    500u        // ms a read waits for the card to finish a write

#ifndef FR_SD_FATFS_WRITES
#define FR_SD_FATFS_WRITES 0        // 1 writes through f_write and f_sync instead, for comparison
#endif

#if FF_FS_TINY
#error "The flight recorder's SD card storage needs FatFs with FF_FS_TINY 0"
#endif

typedef struct {
    int16_t index;                  // mapped sector held, -1 for none
    bool dirty;
    uint32_t used, dirtied;         // for picking what to drop and what to write first
    uint8_t data[SD_SECTOR];
} cache_t;

typedef struct {
    uint32_t writes, failed, finished, pauses;
    uint32_t busy_total, busy_max;  // ms the card took per write
    uint32_t blocked_max;           // longest a call held the processor, us
    uint32_t gap_max;               // ms between realtime polls
} sd_stats_t;

static FATFS *fs = NULL;
static bool ready = false;
static LBA_t lba[SD_MAPPED];
static uint8_t tx[SD_SECTOR];       // the sector being sent, or read for the record and header

static struct {
    bool known;                     // once the recorder has written one
    bool pending;
    uint8_t finished;               // copy that holds the newest record the card has finished
    uint8_t size;
    uint8_t data[64];
} record;

static struct {
    bool dirty;
    uint8_t size;
    uint8_t data[16];
} header;

static cache_t cache[SD_CACHE];
static uint32_t use_count = 0, dirty_count = 0;
static int16_t last_written = -1;   // event sector written last, stays in the cache
static bool flush_requested = false;

static struct {
    bool busy;                      // a sector was sent and the card hasn't finished it
    bool check;                     // a send failed part way: look before the next command
    bool record;                    // the sector being written is a record copy
    uint8_t copy;
    uint32_t since;
} card;

static bool failing = false;        // the last write failed, or the card has been busy too long
static uint32_t retry_at = 0;
static uint32_t last_poll = 0;
static sd_stats_t stats;

static uint32_t micros (void)
{
    return hal.get_micros ? (uint32_t)hal.get_micros() : hal.get_elapsed_ticks() * 1000u;
}

static void blocked_since (uint32_t start)
{
    uint32_t us = micros() - start;

    if(us > stats.blocked_max)
        stats.blocked_max = us;
}

// Index to file sector and back. map_index() gives -1 for a sector we don't use.
static uint32_t file_sector (uint_fast8_t index)
{
    if(index == 0)
        return 0;

    return index == SD_COPY1_INDEX ? SD_COPY1_AT / SD_SECTOR : SD_HEADER_AT / SD_SECTOR + index - SD_HEADER;
}

static int16_t map_index (uint32_t sector)
{
    if(sector == 0)
        return 0;

    if(sector == SD_COPY1_AT / SD_SECTOR)
        return (int16_t)SD_COPY1_INDEX;

    if(sector >= SD_HEADER_AT / SD_SECTOR && sector < SD_HEADER_AT / SD_SECTOR + 1 + SD_LOG_SECTORS)
        return (int16_t)(SD_HEADER + sector - SD_HEADER_AT / SD_SECTOR);

    return -1;
}

static uint32_t sectors_apart (uint_fast8_t a, uint_fast8_t b)
{
    return lba[a] > lba[b] ? lba[a] - lba[b] : lba[b] - lba[a];
}

// The card

#if !FR_SD_FATFS_WRITES

// Notes when the card has finished the sector it was sent, and says if it is still busy.
static bool card_busy (void)
{
    if(!(card.busy || card.check))
        return false;

    uint32_t start = micros();
    bool busy = fr_sd_card_busy();
    blocked_since(start);

    uint32_t took = hal.get_elapsed_ticks() - card.since;

    if(busy) {
        if(took >= SD_STUCK)
            failing = true;
        return true;
    }

    if(card.busy) {
        if(card.record)
            record.finished = card.copy;
        stats.finished++;
        stats.busy_total += took;
        if(took > stats.busy_max)
            stats.busy_max = took;
        if(took >= SD_PAUSE)
            stats.pauses++;
    }

    card.busy = card.check = false;

    return false;
}

// Sends one sector and returns without waiting for the card to write it.
static bool send_sector (int16_t index, const uint8_t *data)
{
    uint32_t start = micros();
    bool ok = disk_write(fs->pdrv, data, lba[index], 1) == RES_OK;
    blocked_since(start);

    card.since = hal.get_elapsed_ticks();

    if(ok) {
        stats.writes++;
        card.busy = true;
        failing = false;
    } else {
        stats.failed++;
        failing = true;
        retry_at = card.since + SD_RETRY;
        card.check = true;              // it may have taken some of it, so look before the next command
    }

    return ok;
}

#endif // !FR_SD_FATFS_WRITES

// Reads wait for the card to finish any write. Only at start-up and for $EVENTS.
static bool wait_idle (void)
{
#if !FR_SD_FATFS_WRITES
    uint32_t start = hal.get_elapsed_ticks();

    while(card_busy()) {
        if(hal.get_elapsed_ticks() - start >= SD_READ_WAIT)
            return false;
    }
#endif

    return true;
}

static bool read_sector (int16_t index, uint8_t *data)
{
    if(!wait_idle())
        return false;

    uint32_t start = micros();
    bool ok = disk_read(fs->pdrv, data, lba[index], 1) == RES_OK;
    blocked_since(start);

    return ok;
}

#if !FR_SD_FATFS_WRITES

// Event sectors in RAM

// Gets an event sector into RAM. It is read from the card, unless the log has just
// moved into it (fresh, an event going into its first slot), and then it starts blank.
static cache_t *cache_get (int16_t index, bool fresh)
{
    cache_t *c, *drop = NULL;

    for(c = cache; c < cache + SD_CACHE; c++) {
        if(c->index == index)
            return c;
        if((c->index < 0 || (!c->dirty && c->index != last_written)) && (drop == NULL || c->used < drop->used))
            drop = c;
    }

    if((c = drop) == NULL)
        return NULL;

    c->index = index;
    c->used = ++use_count;

    if(fresh)
        memset(c->data, 0xFF, SD_SECTOR);
    else if(!read_sector(index, c->data)) {
        c->index = -1;
        return NULL;
    }

    return c;
}

static uint_fast8_t dirty_sectors (void)
{
    uint_fast8_t n = 0;

    for(cache_t *c = cache; c < cache + SD_CACHE; c++)
        n += c->dirty;

    return n;
}

#endif // !FR_SD_FATFS_WRITES

// Storage interface

static bool sd_read (uint32_t addr, void *data, uint32_t size)
{
    uint32_t offset = addr % SD_SECTOR;
    int16_t index = map_index(addr / SD_SECTOR);

    if(!ready || index < 0 || offset + size > SD_SECTOR)
        return false;

#if !FR_SD_FATFS_WRITES
    // Event sectors come from RAM if they are there, and stay there.
    if(index >= (int16_t)SD_LOG_FIRST && index < (int16_t)SD_COPY1_INDEX) {
        cache_t *c = cache_get(index, false);
        if(c == NULL)
            return false;
        memcpy(data, c->data + offset, size);
        c->used = ++use_count;
        return true;
    }

    // A header that hasn't gone out yet is newer than the card's.
    if(index == (int16_t)SD_HEADER && header.dirty) {
        memset(tx, 0xFF, SD_SECTOR);
        memcpy(tx, header.data, header.size);
    } else
#endif
    if(!read_sector(index, tx))
        return false;

    memcpy(data, tx + offset, size);

    return true;
}

#if FR_SD_FATFS_WRITES

// Comparison builds only: every write goes through FatFs and waits for the card.
static FIL file;

static bool sd_write (uint32_t addr, const void *data, uint32_t size)
{
    UINT written;
    uint32_t start = micros();
    bool ok = ready && f_lseek(&file, addr) == FR_OK && f_write(&file, data, size, &written) == FR_OK &&
               written == size && f_sync(&file) == FR_OK;
    blocked_since(start);

    stats.writes++;
    if(!ok)
        stats.failed++;

    return ok;
}

#else

static bool sd_write (uint32_t addr, const void *data, uint32_t size)
{
    uint32_t offset = addr % SD_SECTOR;
    int16_t index = map_index(addr / SD_SECTOR);

    if(!ready || index < 0 || offset + size > SD_SECTOR)
        return false;

    // A record goes to the copy the card didn't finish last, whichever one the recorder
    // asked for, so the newest finished record is kept.
    if(index == 0 || index == (int16_t)SD_COPY1_INDEX) {
        if(offset || size > sizeof(record.data))
            return false;
        if(!record.known) {
            record.known = true;
            record.finished = index == 0;   // the recorder writes over the older copy
        }
        memcpy(record.data, data, size);
        record.size = (uint8_t)size;
        record.pending = true;
        return !failing;
    }

    if(index == (int16_t)SD_HEADER) {
        if(offset || size > sizeof(header.data))
            return false;
        memcpy(header.data, data, size);
        header.size = (uint8_t)size;
        header.dirty = true;
        return !failing;
    }

    cache_t *c = cache_get(index, offset == 0);
    if(c == NULL)
        return false;

    memcpy(c->data + offset, data, size);
    c->used = ++use_count;
    if(!c->dirty) {
        c->dirty = true;
        c->dirtied = ++dirty_count;
    }
    last_written = index;

    // The log has moved on into another sector: the full one can go now.
    if(dirty_sectors() > 1)
        flush_requested = true;

    return !failing;
}

// The next thing for the card: the record, then the event sectors, oldest first, then the header.
static void send_next (void)
{
    if(record.pending) {
        uint8_t copy = record.finished ^ 1;
        memset(tx, 0xFF, SD_SECTOR);
        memcpy(tx, record.data, record.size);
        if(send_sector(copy ? (int16_t)SD_COPY1_INDEX : 0, tx)) {
            record.pending = false;
            card.record = true;
            card.copy = copy;
        }
        return;
    }

    if(!flush_requested)
        return;

    cache_t *next = NULL;

    for(cache_t *c = cache; c < cache + SD_CACHE; c++) {
        if(c->dirty && (next == NULL || c->dirtied < next->dirtied))
            next = c;
    }

    if(next) {
        if(send_sector(next->index, next->data)) {
            next->dirty = false;
            card.record = false;
        }
        return;
    }

    if(header.dirty) {
        memset(tx, 0xFF, SD_SECTOR);
        memcpy(tx, header.data, header.size);
        if(send_sector(SD_HEADER, tx)) {
            header.dirty = false;
            card.record = false;
        }
        return;
    }

    flush_requested = false;
}

#endif // FR_SD_FATFS_WRITES

static void sd_flush (void)
{
    flush_requested = true;
}

static void sd_poll (void)
{
    uint32_t now = hal.get_elapsed_ticks();

    if(!ready)
        return;

    if(last_poll && now - last_poll > stats.gap_max)
        stats.gap_max = now - last_poll;
    last_poll = now;

#if !FR_SD_FATFS_WRITES
    if(card_busy() || (failing && (int32_t)(now - retry_at) < 0))
        return;

    send_next();
#endif
}

static void sd_report (bool reset)
{
    char buf[200];
    uint32_t done = stats.finished;

    // How far apart the closest two of the three parts are on the card.
    uint32_t apart = sectors_apart(0, SD_COPY1_INDEX);
    if(sectors_apart(0, SD_HEADER) < apart)
        apart = sectors_apart(0, SD_HEADER);
    if(sectors_apart(SD_HEADER, SD_COPY1_INDEX) < apart)
        apart = sectors_apart(SD_HEADER, SD_COPY1_INDEX);

    snprintf(buf, sizeof(buf), "[FLIGHTSD:writes=%lu|failed=%lu|busy_avg=%lu.%lu|busy_max=%lu|pauses=%lu|blocked_max_us=%lu|gap_max=%lu|apart=%lu|%s]",
             (unsigned long)stats.writes, (unsigned long)stats.failed,
             (unsigned long)(done ? stats.busy_total / done : 0), (unsigned long)(done ? (stats.busy_total * 10 / done) % 10 : 0),
             (unsigned long)stats.busy_max, (unsigned long)stats.pauses, (unsigned long)stats.blocked_max,
             (unsigned long)stats.gap_max, (unsigned long)apart,
             failing ? "failing" : "ok");

    hal.stream.write(buf);
    hal.stream.write(ASCII_EOL);

    if(reset) {
        memset(&stats, 0, sizeof(stats));
        last_poll = 0;
    }
}

// Start-up

// Makes FLIGHT.DAT at its full size, everything we use erased (0xFF).
static bool make_file (void)
{
    FIL f;
    UINT written;
    bool ok;

    f_chmod(SD_PATH, 0, SD_ATTRIBUTES);     // so an old one of the wrong size can be replaced

    if(f_open(&f, SD_PATH, FA_CREATE_ALWAYS|FA_WRITE) != FR_OK)
        return false;

    memset(tx, 0xFF, SD_SECTOR);

    // Seeking past the end makes the file longer without writing what is skipped.
    ok = f_write(&f, tx, SD_SECTOR, &written) == FR_OK && written == SD_SECTOR && f_lseek(&f, SD_HEADER_AT) == FR_OK;
    for(uint_fast8_t i = 0; ok && i < 1 + SD_LOG_SECTORS; i++)
        ok = f_write(&f, tx, SD_SECTOR, &written) == FR_OK && written == SD_SECTOR;
    ok = ok && f_lseek(&f, SD_COPY1_AT) == FR_OK && f_write(&f, tx, SD_SECTOR, &written) == FR_OK &&
          written == SD_SECTOR && f_tell(&f) == SD_FILE_SIZE;

    ok = f_close(&f) == FR_OK && ok;

#if !FR_SD_FATFS_WRITES
    ok = ok && f_chmod(SD_PATH, SD_ATTRIBUTES, SD_ATTRIBUTES) == FR_OK;
#endif

    return ok;
}

// Finds the card sector behind each file sector we use. Reading a byte through FatFs
// leaves the sector in the file's buffer and its number in f.sect, and reading that
// sector straight from the card has to give the same data.
static bool map_file (void)
{
    FIL f;
    UINT got;
    uint8_t byte;
    bool ok = f_open(&f, SD_PATH, FA_READ) == FR_OK;

    for(uint_fast8_t i = 0; ok && i < SD_MAPPED; i++) {
        ok = f_lseek(&f, file_sector(i) * SD_SECTOR) == FR_OK && f_read(&f, &byte, 1, &got) == FR_OK && got == 1 && f.sect != 0;
        if(ok) {
            lba[i] = f.sect;
            ok = disk_read(fs->pdrv, tx, lba[i], 1) == RES_OK && memcmp(tx, f.buf, SD_SECTOR) == 0;
        }
    }

    f_close(&f);

    return ok;
}

static bool sd_init (void)
{
    FILINFO info;

    // Mounts the card if grblHAL hasn't already. No card, no SD storage.
    if((fs = sdcard_getfs()) == NULL)
        return false;

    if(!(f_stat(SD_PATH, &info) == FR_OK && info.fsize == SD_FILE_SIZE) && !make_file()) {
        report_message("Flight recorder: can't make FLIGHT.DAT on the SD card", Message_Warning);
        return false;
    }

    if(!map_file()) {
        report_message("Flight recorder: FLIGHT.DAT on the SD card doesn't read back", Message_Warning);
        return false;
    }

#if FR_SD_FATFS_WRITES
    f_chmod(SD_PATH, 0, SD_ATTRIBUTES);
    if(f_open(&file, SD_PATH, FA_READ|FA_WRITE) != FR_OK)
        return false;
#endif

    for(cache_t *c = cache; c < cache + SD_CACHE; c++)
        c->index = -1;

    ready = true;

    return true;
}

const fr_backend_t fr_storage_sd = {
    .info = {
        .label = "SD card",
        .record_stride = SD_COPY1_AT,
        .log_header = SD_HEADER_AT,
        .log_entries = SD_LOG_AT,
        .log_slots = SD_LOG_SECTORS * (SD_SECTOR / sizeof(fr_event_t)),
        .record_interval = 100,
#ifdef FR_SD_BATCH_EVENTS
        .batch_events = FR_SD_BATCH_EVENTS,     // 0 for the comparison build without batching
#else
        .batch_events = true,
#endif
    },
    .init = sd_init,
    .read = sd_read,
    .write = sd_write,
    .flush = sd_flush,
    .poll = sd_poll,
    .report = sd_report
};

#endif // FLIGHT_RECORDER_ENABLE && FR_SD_ENABLE
