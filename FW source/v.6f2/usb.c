// MSC callbacks — called on core 0 from within tud_task().
// NEVER call tud_task() from here.  IDE wait loops use busy_wait only.

#include "tusb.h"
#include "class/msc/msc_device.h"
#include "ide.h"
#include "config.h"
#include <string.h>

extern volatile bool is_mounted;
extern volatile bool media_changed_waiting;

// ---------------------------------------------------------------------------
//  Helpers
// ---------------------------------------------------------------------------

static uint64_t total_sectors(void) {
    if (!is_mounted) return 0;
    if (config.use_lba_mode) return config.lba_sectors;
    return (uint64_t)config.cyls * config.heads * config.spt;
}

// ---------------------------------------------------------------------------
//  MSC Required Callbacks
// ---------------------------------------------------------------------------

void tud_msc_inquiry_cb(uint8_t lun, uint8_t vendor_id[8],
                        uint8_t product_id[16], uint8_t product_rev[4]) {
    (void)lun;
    const char vid[] = "ATAboy";
    const char pid[] = "Hard Drive";
    const char rev[] = "V0.6";

    memset(vendor_id, ' ', 8);
    memcpy(vendor_id, vid, strlen(vid) < 8 ? strlen(vid) : 8);
    memset(product_id, ' ', 16);
    memcpy(product_id, pid, strlen(pid) < 16 ? strlen(pid) : 16);
    memset(product_rev, ' ', 4);
    memcpy(product_rev, rev, strlen(rev) < 4 ? strlen(rev) : 4);
}

bool tud_msc_is_writable_cb(uint8_t lun) {
    (void)lun;
    return !config.drive_write_protected;
}

bool tud_msc_test_unit_ready_cb(uint8_t lun) {
    (void)lun;
    return is_mounted;
}

void tud_msc_capacity_cb(uint8_t lun, uint32_t *block_count,
                         uint16_t *block_size) {
    (void)lun;
    *block_size = 512;
    uint64_t ts = total_sectors();
    *block_count = (ts > 0xFFFFFFFF) ? 0xFFFFFFFF : (uint32_t)ts;
}

bool tud_msc_start_stop_cb(uint8_t lun, uint8_t power_condition,
                           bool start, bool load_eject) {
    (void)lun; (void)power_condition; (void)start; (void)load_eject;
    return true;
}

// ---------------------------------------------------------------------------
//  READ10 — block transfer with partial first/last sector handling
// ---------------------------------------------------------------------------

int32_t tud_msc_read10_cb(uint8_t lun, uint32_t lba, uint32_t offset,
                          void *buffer, uint32_t bufsize) {
    (void)lun;
    if (!is_mounted) return -1;

    uint64_t max = total_sectors();
    if (max == 0) return -1;

    uint32_t remaining = bufsize;
    uint8_t *ptr = (uint8_t *)buffer;
    uint32_t cur_lba = lba;

    // Partial first sector (non-zero offset)
    if (offset && remaining > 0 && cur_lba < max) {
        uint8_t temp[512];
        if (ide_read_sectors(cur_lba, 1, temp) < 0) {
            tud_msc_set_sense(lun, SCSI_SENSE_MEDIUM_ERROR, 0x11, 0x00);
            return -1;
        }
        uint32_t n = 512 - offset;
        if (n > remaining) n = remaining;
        memcpy(ptr, temp + offset, n);
        ptr += n; remaining -= n; cur_lba++;
    }

    // Aligned middle sectors — single ATA command
    uint32_t aligned = remaining / 512;
    if (aligned > 0 && cur_lba < max) {
        if (cur_lba + aligned > max) aligned = (uint32_t)(max - cur_lba);
        if (ide_read_sectors(cur_lba, aligned, ptr) < 0) {
            tud_msc_set_sense(lun, SCSI_SENSE_MEDIUM_ERROR, 0x11, 0x00);
            return -1;
        }
        ptr += aligned * 512; remaining -= aligned * 512; cur_lba += aligned;
    }

    // Partial last sector
    if (remaining > 0 && cur_lba < max) {
        uint8_t temp[512];
        if (ide_read_sectors(cur_lba, 1, temp) < 0) {
            tud_msc_set_sense(lun, SCSI_SENSE_MEDIUM_ERROR, 0x11, 0x00);
            return -1;
        }
        memcpy(ptr, temp, remaining);
        ptr += remaining; remaining = 0;
    }

    if (remaining > 0) memset(ptr, 0, remaining);
    return (int32_t)bufsize;
}

// ---------------------------------------------------------------------------
//  WRITE10 — block transfer with partial first/last read-modify-write
// ---------------------------------------------------------------------------

int32_t tud_msc_write10_cb(uint8_t lun, uint32_t lba, uint32_t offset,
                           uint8_t *buffer, uint32_t bufsize) {
    (void)lun;
    if (!is_mounted || config.drive_write_protected) return -1;

    uint64_t max = total_sectors();
    if (max == 0) return -1;

    uint32_t remaining = bufsize;
    uint8_t *ptr = buffer;
    uint32_t cur_lba = lba;

    // Partial first sector — read-modify-write
    if (offset && remaining > 0 && cur_lba < max) {
        uint8_t temp[512];
        if (ide_read_sectors(cur_lba, 1, temp) < 0) {
            tud_msc_set_sense(lun, SCSI_SENSE_MEDIUM_ERROR, 0x11, 0x00);
            return -1;
        }
        uint32_t n = 512 - offset;
        if (n > remaining) n = remaining;
        memcpy(temp + offset, ptr, n);
        if (ide_write_sectors(cur_lba, 1, temp) < 0) {
            tud_msc_set_sense(lun, SCSI_SENSE_MEDIUM_ERROR, 0x03, 0x00);
            return -1;
        }
        ptr += n; remaining -= n; cur_lba++;
    }

    // Aligned middle sectors — single ATA command
    uint32_t aligned = remaining / 512;
    if (aligned > 0 && cur_lba < max) {
        if (cur_lba + aligned > max) aligned = (uint32_t)(max - cur_lba);
        if (ide_write_sectors(cur_lba, aligned, ptr) < 0) {
            tud_msc_set_sense(lun, SCSI_SENSE_MEDIUM_ERROR, 0x03, 0x00);
            return -1;
        }
        ptr += aligned * 512; remaining -= aligned * 512; cur_lba += aligned;
    }

    // Partial last sector — read-modify-write
    if (remaining > 0 && cur_lba < max) {
        uint8_t temp[512];
        if (ide_read_sectors(cur_lba, 1, temp) < 0) {
            tud_msc_set_sense(lun, SCSI_SENSE_MEDIUM_ERROR, 0x11, 0x00);
            return -1;
        }
        memcpy(temp, ptr, remaining);
        if (ide_write_sectors(cur_lba, 1, temp) < 0) {
            tud_msc_set_sense(lun, SCSI_SENSE_MEDIUM_ERROR, 0x03, 0x00);
            return -1;
        }
        remaining = 0;
    }

    return (int32_t)bufsize;
}

// ---------------------------------------------------------------------------
//  SCSI — Mode Sense + misc
// ---------------------------------------------------------------------------

int32_t tud_msc_scsi_cb(uint8_t lun, uint8_t const scsi_cmd[16],
                        void *buffer, uint16_t bufsize) {
    uint8_t opcode = scsi_cmd[0];
    uint8_t *buf = (uint8_t *)buffer;

    // Unit Attention on first access after mount/unmount
    if (is_mounted && media_changed_waiting) {
        tud_msc_set_sense(lun, SCSI_SENSE_UNIT_ATTENTION, 0x28, 0);
        media_changed_waiting = false;
        return -1;
    }

    switch (opcode) {
    case 0x1A:  // MODE SENSE (6)
    case 0x5A:  // MODE SENSE (10)
    {
        uint8_t page = scsi_cmd[2] & 0x3F;
        bool is10 = (opcode == 0x5A);
        uint8_t hdr = is10 ? 8 : 4;
        if (bufsize < hdr) return -1;
        memset(buffer, 0, bufsize);
        uint16_t pos = hdr;

        // Page 0x03: Format Device (SPT)
        if ((page == 0x03 || page == 0x3F) && pos + 24 <= bufsize) {
            buf[pos] = 0x03; buf[pos + 1] = 0x16;
            buf[pos + 10] = (uint8_t)(config.spt >> 8);
            buf[pos + 11] = (uint8_t)(config.spt & 0xFF);
            pos += 24;
        }
        // Page 0x04: Rigid Disk Geometry (Cyl/Heads)
        if ((page == 0x04 || page == 0x3F) && pos + 24 <= bufsize) {
            buf[pos] = 0x04; buf[pos + 1] = 0x16;
            buf[pos + 2] = 0;
            buf[pos + 3] = (uint8_t)(config.cyls >> 8);
            buf[pos + 4] = (uint8_t)(config.cyls & 0xFF);
            buf[pos + 5] = config.heads;
            pos += 24;
        }

        if (!is10) {
            buf[0] = (uint8_t)(pos - 1);
            buf[2] = config.drive_write_protected ? 0x80 : 0x00;
        } else {
            uint16_t full = pos - 2;
            buf[0] = (uint8_t)(full >> 8);
            buf[1] = (uint8_t)(full & 0xFF);
            buf[3] = config.drive_write_protected ? 0x80 : 0x00;
        }
        return (int32_t)pos;
    }

    case 0x00: return 0;  // TEST UNIT READY
    case 0x1B: return 0;  // START STOP UNIT
    case 0x35: return 0;  // SYNCHRONIZE CACHE
    case 0x1E: return 0;  // PREVENT ALLOW MEDIUM REMOVAL

    default:
        tud_msc_set_sense(lun, SCSI_SENSE_ILLEGAL_REQUEST, 0x20, 0);
        return -1;
    }
}
