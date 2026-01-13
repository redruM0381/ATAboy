#include "tusb.h"
#include "class/msc/msc_device.h"
#include "ide_logic.h"
#include <string.h>
#include <stdbool.h>

/* --- Externs from menus.c --- */
/* Media change notification flag (UI/menu) — keep this extern; other core1 globals were removed. */
extern bool media_changed_waiting;

// Removed other direct core1 global externs to prevent accidental Core0 access.
// Use ide_get_core1_state(), ide_is_mounted(), ide_set_* helpers instead.

/* --- Externs from ide_logic.cpp --- */
#ifdef __cplusplus
extern "C" {
#endif
    int32_t ide_read_sectors_lba(uint32_t lba, uint32_t count, uint8_t* buffer);
    int32_t ide_write_sectors_lba(uint32_t lba, uint32_t count, const uint8_t* buffer);
    int32_t ide_read_cached(uint32_t lba, uint32_t count, uint8_t* buffer);
    void ide_flush_cache(void); // <-- added
#ifdef __cplusplus
}
#endif

void tud_msc_inquiry_cb(uint8_t lun, uint8_t vendor_id[8], uint8_t product_id[16], uint8_t product_rev[4]) {
    (void) lun;
    const char vid[] = "IDEasy";
    const char pid[] = "Hard Drive";
    const char rev[] = "V0.4";

    // space-pad fields per SCSI
    memset(vendor_id, ' ', 8);
    size_t n = strlen(vid); if (n > 8) n = 8;
    memcpy(vendor_id, vid, n);

    memset(product_id, ' ', 16);
    n = strlen(pid); if (n > 16) n = 16;
    memcpy(product_id, pid, n);

    memset(product_rev, ' ', 4);
    n = strlen(rev); if (n > 4) n = 4;
    memcpy(product_rev, rev, n);
}

bool tud_msc_is_writable_cb(uint8_t lun) {
    (void) lun;
    ide_core1_state_t st;
    if (!ide_get_core1_state(&st)) return false;
    return !st.drive_write_protected;
}

bool tud_msc_test_unit_ready_cb(uint8_t lun) {
    (void) lun;

    /* Prefer a consistent snapshot of core1 state for decision logic.
       Fall back to the atomic mounted query if snapshot fails. */
    ide_core1_state_t st;
    if (ide_get_core1_state(&st)) {
        if (ide_is_mounted()) return true;
        if (st.use_lba_mode) return (st.total_lba > 0);
        return (st.cyls > 0 && st.heads > 0 && st.spt > 0);
    }
    return ide_is_mounted();
}

// Clamp LBA for LBA28 and CHS limits
static inline uint32_t clamp_lba(uint64_t lba)
{
    /* Use atomic mounted query instead of reading is_mounted directly */
    if (!ide_is_mounted()) return 0;

    ide_core1_state_t st;
    if (!ide_get_core1_state(&st)) return 0;

    uint64_t last64 = 0;

    if (st.use_lba_mode)
    {
        if (st.total_lba == 0) return 0;
        last64 = st.total_lba - 1; // last valid index
    }
    else
    {
        uint64_t total = (uint64_t)st.cyls * st.heads * st.spt;
        if (total == 0) return 0;
        last64 = total - 1;
    }

    if (last64 > UINT32_MAX) last64 = UINT32_MAX;
    return (lba > last64) ? (uint32_t)last64 : (uint32_t)lba;
}


// Invoked when received GET_CAPACITY10 command
void tud_msc_capacity_cb(uint8_t lun, uint32_t* block_count, uint16_t* block_size) {
    (void) lun;
    *block_size = 512;

    if (!ide_is_mounted()) {
        *block_count = 0;
        return;
   }

    ide_core1_state_t st;
    if (!ide_get_core1_state(&st)) {
        *block_count = 0;
        return;
    }

    uint64_t total_sectors = 0;
    if (st.use_lba_mode) {
        // In LBA mode, use the full count provided by IDENTIFY (snapshot)
        total_sectors = st.total_lba;
    } else {
        // In CHS mode, align to cylinder boundaries using snapshot geometry
        if (st.cyls == 0 || st.heads == 0 || st.spt == 0) {
            *block_count = 0;
            return;
        }
        uint32_t sectors_per_cylinder = (uint32_t)st.heads * st.spt;
        total_sectors = (uint64_t)st.cyls * sectors_per_cylinder;
    }

    if (total_sectors > 0xFFFFFFFF) {
        *block_count = 0xFFFFFFFF;
    } else {
        *block_count = (uint32_t)total_sectors;
    }
}


bool tud_msc_start_stop_cb(uint8_t lun, uint8_t power_condition, bool start, bool load_eject) {
    (void) lun; (void) power_condition; (void) load_eject;
    return true;
}

// READ10 callback with bounds checking + error checking for IDE reads
int32_t tud_msc_read10_cb(uint8_t lun, uint32_t lba, uint32_t offset, void* buffer, uint32_t bufsize)
{
    (void) lun;
    if (!ide_is_mounted() || buffer == NULL) return -1;
    if (bufsize == 0) return 0;
    if (offset >= 512) return -1;

    uint8_t* dst = (uint8_t*)buffer;

    /* Use core‑1 snapshot for bounds instead of touching core1 globals directly */
    ide_core1_state_t st;
    if (!ide_get_core1_state(&st)) return -1;

    uint64_t max_sectors = 0;
    if (st.use_lba_mode) {
        if (st.total_lba == 0) return -1;
        max_sectors = st.total_lba;
    } else {
        uint64_t total = (uint64_t)st.cyls * st.heads * st.spt;
        if (total == 0) return -1;
        max_sectors = total;
    }

    /* compute required sectors (cover offset + bufsize) */
    uint32_t sector_count = (offset + bufsize + 511) / 512;
    if ((uint64_t)lba + sector_count > max_sectors) return -1;

    uint32_t bytes_remaining = bufsize;

    for (uint32_t i = 0; i < sector_count && bytes_remaining > 0; ++i) {
        uint8_t tmp[512];
        uint32_t sector_off = (i == 0) ? offset : 0;
        uint32_t chunk = (bytes_remaining < (512 - sector_off)) ? bytes_remaining : (512 - sector_off);

        /* read sector (use cached reader which is IPC-safe) */
        if (ide_read_cached(lba + i, 1, tmp) < 0) return -1;

        memcpy(dst, tmp + sector_off, chunk);
        dst += chunk;
        bytes_remaining -= chunk;
    }

    return (int32_t)bufsize;
}

int32_t tud_msc_write10_cb(uint8_t lun, uint32_t lba, uint32_t offset, uint8_t* buffer, uint32_t bufsize) {
    (void) lun;

    /* Use atomic/multi-core safe checks */
    if (!ide_is_mounted()) return -1;

    uint32_t sector_count = (bufsize + 511) / 512;

    /* Use core‑1 snapshot for bounds and write-protect check */
    ide_core1_state_t st;
    if (!ide_get_core1_state(&st)) return -1;
    if (st.drive_write_protected) return -1;

    uint64_t max_sectors = 0;
    if (st.use_lba_mode) {
        if (st.total_lba == 0) return -1;
        max_sectors = st.total_lba;
    } else {
        uint64_t total = (uint64_t)st.cyls * st.heads * st.spt;
        if (total == 0) return -1;
        max_sectors = total;
    }

    if ((uint64_t)lba + sector_count > max_sectors) return -1;

    uint32_t bytes_remaining = bufsize;
    uint8_t* buf_ptr = buffer;

    for (uint32_t i = 0; i < sector_count && bytes_remaining > 0; ++i) {
        uint8_t temp[512];
        uint32_t copy_offset = (i == 0) ? offset : 0;
        uint32_t copy_bytes = (bytes_remaining < (512 - copy_offset)) ? bytes_remaining : (512 - copy_offset);

        if (copy_bytes == 512 && copy_offset == 0) {
            /* Full-sector write — write directly from buffer */
            if (ide_write_sectors_lba(lba + i, 1, buf_ptr) < 0) return -1;
        } else {
            /* Partial-sector: read-modify-write */
            if (ide_read_cached(lba + i, 1, temp) < 0) return -1;
            memcpy(temp + copy_offset, buf_ptr, copy_bytes);
            if (ide_write_sectors_lba(lba + i, 1, temp) < 0) return -1;
        }

        buf_ptr += copy_bytes;
        bytes_remaining -= copy_bytes;
    }

    ide_flush_cache();
    return (int32_t)bufsize;
}



// SCSI callback
int32_t tud_msc_scsi_cb(uint8_t lun, uint8_t const scsi_cmd[16], void* buffer, uint16_t bufsize) {
    uint8_t const opcode = scsi_cmd[0];
    uint8_t* buf = (uint8_t*)buffer;

    /* Unit Attention (Media Change) */
    if (ide_is_mounted() && ide_consume_media_changed()) {
        tud_msc_set_sense(lun, SCSI_SENSE_UNIT_ATTENTION, 0x28, 0);
        return -1;
    }

    switch (opcode) {
        case 0x1A: // MODE SENSE (6)
        case 0x5A: // MODE SENSE (10)
        {
            uint8_t page_code = scsi_cmd[2] & 0x3F;
            bool is_ms10 = (opcode == 0x5A);
            uint8_t header_len = is_ms10 ? 8 : 4;

            if (bufsize < header_len) return -1;

            memset(buffer, 0, bufsize);
            uint16_t current_pos = header_len;

            /* Use snapshot of core1 state for geometry/flags */
            ide_core1_state_t st;
            (void) ide_get_core1_state(&st);

            /* PAGE 0x03: Format Device Page (SPT) */
            if (page_code == 0x03 || page_code == 0x3F) {
                if ((uint32_t)current_pos + 24 <= bufsize) {
                    uint8_t *p = &buf[current_pos];
                    p[0] = 0x03;
                    p[1] = 0x16; // 22 bytes follow
                    uint16_t spt16 = (uint16_t)st.spt;
                    p[10] = (uint8_t)(spt16 >> 8);
                    p[11] = (uint8_t)(spt16 & 0xFF);
                    current_pos += 24;
                }
            }

            /* PAGE 0x04: Rigid Disk Geometry (Cyl/Heads) */
            if (page_code == 0x04 || page_code == 0x3F) {
                if ((uint32_t)current_pos + 24 <= bufsize) {
                    uint8_t *p = &buf[current_pos];
                    p[0] = 0x04;
                    p[1] = 0x16; // 22 bytes follow
                    p[2] = 0; // cur_cyls is 16-bit, field is 24-bit - high byte zero
                    p[3] = (uint8_t)(st.cyls >> 8);
                    p[4] = (uint8_t)(st.cyls & 0xFF);
                    p[5] = (uint8_t)st.heads;
                    current_pos += 24;
                }
            }

            /* Header fields and write-protect bit */
            if (!is_ms10) {
                buf[0] = (uint8_t)(current_pos - 1);
                buf[1] = 0x00; // Medium Type
                buf[2] = st.drive_write_protected ? 0x80 : 0x00;
                buf[3] = 0x00; // Block Descriptor Length
            } else {
                uint16_t full_len = current_pos - 2;
                buf[0] = (uint8_t)(full_len >> 8);
                buf[1] = (uint8_t)(full_len & 0xFF);
                buf[3] = st.drive_write_protected ? 0x80 : 0x00;
            }

            return (int32_t)current_pos;
        }

        case 0x00: return 0; // TEST UNIT READY
        case 0x1B: return 0; // START STOP UNIT
        case 0x35: // SYNCHRONIZE CACHE
            ide_flush_cache();
            return 0;
        case 0x1E: return 0; // PREVENT ALLOW MEDIUM REMOVAL

        default:
            tud_msc_set_sense(lun, SCSI_SENSE_ILLEGAL_REQUEST, 0x20, 0);
            return -1;
    }
}
