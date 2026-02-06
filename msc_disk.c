#include "tusb.h"
#include "class/msc/msc_device.h"
#include "ide_logic.h"
#include <string.h>

/* --- Externs from menus.c --- */
extern uint16_t cur_cyls;
extern uint8_t  cur_heads;
extern uint8_t  cur_spt;
extern bool     drive_write_protected;
extern bool     is_mounted;
extern bool     use_lba_mode;

extern uint64_t total_lba_sectors_from_identify;

bool media_changed_waiting = false; // Set this to true when you mount

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
    const char vid[] = "ATAboy";
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
    return !drive_write_protected;
}

bool tud_msc_test_unit_ready_cb(uint8_t lun) {
    (void) lun;
    return is_mounted;
}

// Clamp LBA for LBA28 and CHS limits
static inline uint32_t clamp_lba(uint64_t lba)
{
    if (!is_mounted) return 0;

    uint64_t last64 = 0;

    if (use_lba_mode)
    {
        if (total_lba_sectors_from_identify == 0) return 0;
        last64 = total_lba_sectors_from_identify - 1; // last valid index
    }
    else
    {
        uint64_t total = (uint64_t)cur_cyls * cur_heads * cur_spt;
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

    if (!is_mounted) {
        *block_count = 0;
        return;
    }

    uint64_t total_sectors = 0;
    if (use_lba_mode) {
        // In LBA mode, use the full count provided by IDENTIFY
        total_sectors = total_lba_sectors_from_identify;
    } 
    else {
        // In CHS mode, strictly align to cylinder boundaries
        // This prevents Windows VDS from crashing due to geometry/capacity mismatch
        uint32_t sectors_per_cylinder = (uint32_t)cur_heads * cur_spt;
        total_sectors = (uint64_t)cur_cyls * sectors_per_cylinder;
    }

    // TinyUSB expects a 32-bit value; clamp if > UINT32_MAX
    // Note: Most early drives will be well under this limit
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
    if (!is_mounted) return -1;

    uint32_t sector_count = (bufsize + 511) / 512;
    uint32_t last_lba = clamp_lba((uint64_t)lba + sector_count - 1);

    if (lba > last_lba) {
        memset(buffer, 0, bufsize);
        return bufsize;
    }

    uint32_t bytes_remaining = bufsize;
    uint8_t* buf_ptr = (uint8_t*)buffer;

    for (uint32_t i = 0; (lba + i) <= last_lba && bytes_remaining > 0; i++)
    {
        uint8_t temp[512];
        int32_t r = ide_read_cached(lba + i, 1, temp);   //int32_t r = ide_read_sectors_lba(lba + i, 1, temp);  Gemini changed this to enable cache.
        if (r < 0) {
            memset(buf_ptr, 0, bytes_remaining);
            return -1;
        }

        uint32_t copy_offset = (i == 0) ? offset : 0;
        uint32_t copy_bytes = (bytes_remaining < (512 - copy_offset)) ? bytes_remaining : (512 - copy_offset);

        memcpy(buf_ptr, temp + copy_offset, copy_bytes);
        buf_ptr += copy_bytes;
        bytes_remaining -= copy_bytes;
    }

    if (bytes_remaining > 0)
        memset(buf_ptr, 0, bytes_remaining);

    return bufsize;
}

int32_t tud_msc_write10_cb(uint8_t lun, uint32_t lba, uint32_t offset, uint8_t* buffer, uint32_t bufsize) {
    (void) lun;
    if (!is_mounted || drive_write_protected) return -1;

    uint32_t sector_count = (bufsize + 511) / 512;
    
    // Check if the entire write range is valid BEFORE starting
    uint64_t max_sectors = use_lba_mode ? total_lba_sectors_from_identify : ((uint64_t)cur_cyls * cur_heads * cur_spt);
    if ((uint64_t)lba + sector_count > max_sectors) {
        return -1; // Fail immediately if we would truncate
    }

    uint32_t bytes_remaining = bufsize;
    uint8_t* buf_ptr = buffer;

    for (uint32_t i = 0; i < sector_count && bytes_remaining > 0; i++) {
        uint8_t temp[512];
        uint32_t copy_offset = (i == 0) ? offset : 0;
        uint32_t copy_bytes = (bytes_remaining < (512 - copy_offset)) ? bytes_remaining : (512 - copy_offset);

        if (copy_bytes != 512 || copy_offset != 0) {
            if (ide_read_cached(lba + i, 1, temp) < 0) return -1;
        }

        memcpy(temp + copy_offset, buf_ptr, copy_bytes);
        if (ide_write_sectors_lba(lba + i, 1, temp) < 0) return -1;

        buf_ptr += copy_bytes;
        bytes_remaining -= copy_bytes;
    }

    // Invalidate read cache after successful write(s)
    ide_flush_cache();

    return bufsize;
}



// SCSI callback
int32_t tud_msc_scsi_cb(uint8_t lun, uint8_t const scsi_cmd[16], void* buffer, uint16_t bufsize) {
    uint8_t const opcode = scsi_cmd[0];
    uint8_t* buf = (uint8_t*)buffer;

    // Handle Unit Attention (Media Change)
    if (is_mounted && media_changed_waiting) {
        tud_msc_set_sense(lun, SCSI_SENSE_UNIT_ATTENTION, 0x28, 0); 
        media_changed_waiting = false;
        return -1; 
    }

    switch (opcode) {
        case 0x1A: // MODE SENSE (6)
        case 0x5A: // MODE SENSE (10)
        {
            uint8_t page_code = scsi_cmd[2] & 0x3F;
            bool is_ms10 = (opcode == 0x5A);
            uint8_t header_len = is_ms10 ? 8 : 4;

            // 1. Minimum buffer check
            if (bufsize < header_len) return -1;

            memset(buffer, 0, bufsize);
            uint16_t current_pos = header_len;

            // 2. PAGE 0x03: Format Device Page (SPT)
            if (page_code == 0x03 || page_code == 0x3F) {
                if (current_pos + 24 <= bufsize) {
                    uint8_t *p = &buf[current_pos];
                    p[0] = 0x03; 
                    p[1] = 0x16; // 22 bytes follow
                    p[10] = (uint8_t)(cur_spt >> 8); 
                    p[11] = (uint8_t)(cur_spt & 0xFF);
                    current_pos += 24;
                }
            }

            // 3. PAGE 0x04: Rigid Disk Geometry (Cyl/Heads)
            if (page_code == 0x04 || page_code == 0x3F) {
                if (current_pos + 24 <= bufsize) {
                    uint8_t *p = &buf[current_pos];
                    p[0] = 0x04; 
                    p[1] = 0x16; // 22 bytes follow
                    p[2] = 0;    // Explicitly 0: cur_cyls is 16-bit, field is 24-bit
                    p[3] = (uint8_t)(cur_cyls >> 8);
                    p[4] = (uint8_t)(cur_cyls & 0xFF);
                    p[5] = (uint8_t)cur_heads;
                    current_pos += 24;
                }
            }

            // 4. HEADER CALCULATIONS (Using current_pos ensures no length mismatch)
            if (!is_ms10) {
                // Mode Sense 6: Byte 0 is (Total Payload Length - 1)
                buf[0] = (uint8_t)(current_pos - 1);
                buf[1] = 0x00; // Medium Type
                buf[2] = drive_write_protected ? 0x80 : 0x00; 
                buf[3] = 0x00; // Block Descriptor Length
            } else {
                // Mode Sense 10: Bytes 0-1 is (Total Payload Length - 2)
                uint16_t full_len = current_pos - 2;
                buf[0] = (uint8_t)(full_len >> 8);
                buf[1] = (uint8_t)(full_len & 0xFF);
                // Byte 3: Device-specific parameter (Write Protect)
                buf[3] = drive_write_protected ? 0x80 : 0x00; 
            }

            return (int32_t)current_pos;
        }

        case 0x00: return 0; // TEST UNIT READY
        case 0x1B: return 0; // START STOP UNIT
        case 0x35: // SYNCHRONIZE CACHE
            // Ensure any read cache is invalidated to reflect recent writes
            ide_flush_cache();
            return 0;
        case 0x1E: return 0; // PREVENT ALLOW MEDIUM REMOVAL
        
        default:
            tud_msc_set_sense(lun, SCSI_SENSE_ILLEGAL_REQUEST, 0x20, 0);
            return -1;
    }
}