#include "ide_logic.h" 
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/structs/sio.h"
#include "hardware/sync.h"
#include "tusb.h"
#include "pico/time.h"
#include <string.h>

// Map board_millis to the Pico SDK equivalent
#define board_millis() to_ms_since_boot(get_absolute_time())
#define IDE_REG_STATUS 7

extern "C" {


// --- Simple read cache ---
// Adjust size as desired; 128 sectors = 64KB
#ifndef READ_CACHE_SECTORS
#define READ_CACHE_SECTORS 128
#endif

static uint8_t read_cache_buf[READ_CACHE_SECTORS * 512];
static uint64_t read_cache_start = 0xFFFFFFFFFFFFFFFF;
static uint32_t read_cache_count = 0;

/* ================= SHARED GLOBALS ================= */
uint8_t  drive_heads = 0;
uint8_t  drive_spt = 0;
uint16_t drive_cylinders = 0; 
bool     is_mounted = false; 
bool     drive_write_protected = true;
bool    use_lba_mode = false;
bool    drive_supports_lba48 = false;
extern uint64_t total_lba_sectors_from_identify; // <--- Now 64bit, was 32
extern uint16_t cur_cyls;
extern uint8_t  cur_heads;
extern uint8_t  cur_spt;

static inline void ide_setup_lba48(uint64_t lba, uint16_t count);

/* ================= PIN DEFINITIONS ================= */
#define DATA_MASK   0x0000FFFF 
#define IDE_CS0     24
#define IDE_CS1     25   
#define IDE_DIOR    26
#define IDE_DIOW    27
#define IDE_RESET   23
#define IDE_A0      20
#define IDE_A1      21
#define IDE_A2      22
#define ADDR_MASK   ((1 << IDE_A0) | (1 << IDE_A1) | (1 << IDE_A2))
#define IDE_DIR     16 
#define IDE_DIR1    17
#define IDE_OE      18
#define IDE_OE1     19
#define CTRL_MASK   ((1 << IDE_CS0) | (1 << IDE_CS1) | (1 << IDE_DIOR) | (1 << IDE_DIOW))






/* ================= BUS LOGIC ================= */
void bus_idle() {
    // Ensure both CS0 and CS1 are HIGH (inactive)
    sio_hw->gpio_set = (1 << IDE_CS0) | (1 << IDE_CS1) | (1 << IDE_DIOR) | (1 << IDE_DIOW);
    
    // Set transceivers to 'In' direction
    sio_hw->gpio_clr = (1 << IDE_DIR) | (1 << IDE_DIR1);
    
    // Disable transceivers (OE High)
    sio_hw->gpio_set = (1 << IDE_OE) | (1 << IDE_OE1);
    
    sio_hw->gpio_oe_clr = DATA_MASK; 
}

void set_address(uint8_t addr) {
    uint32_t addr_val = (uint32_t)(addr & 0x07) << IDE_A0;
    sio_hw->gpio_clr = ADDR_MASK;
    sio_hw->gpio_set = addr_val;
}

void data_mode_in() {
    // Match CHS_Bridge: Set OE High (Disable), Set Dir IN, Clear OE Low (Enable)
    sio_hw->gpio_set = (1 << IDE_OE) | (1 << IDE_OE1); 
    sio_hw->gpio_oe_clr = DATA_MASK;
    sio_hw->gpio_clr = (1 << IDE_DIR) | (1 << IDE_DIR1);
    busy_wait_at_least_cycles(20); 
    sio_hw->gpio_clr = (1 << IDE_OE) | (1 << IDE_OE1);
}

void data_mode_out(uint16_t val) {
    sio_hw->gpio_set = (1 << IDE_OE) | (1 << IDE_OE1);
    sio_hw->gpio_out = (sio_hw->gpio_out & ~DATA_MASK) | ((uint32_t)val & DATA_MASK);
    sio_hw->gpio_oe_set = DATA_MASK;
    sio_hw->gpio_set = (1 << IDE_DIR) | (1 << IDE_DIR1);
    busy_wait_at_least_cycles(100); 
    sio_hw->gpio_clr = (1 << IDE_OE) | (1 << IDE_OE1);
}

void lba_to_chs(uint32_t lba, uint16_t* c, uint8_t* h, uint8_t* s) {
    *s = (lba % drive_spt) + 1;
    uint32_t temp = lba / drive_spt;
    *h = temp % drive_heads;
    *c = temp / drive_heads;
}

/* ================= REGISTER I/O ================= */
uint8_t ide_read_8(uint8_t reg) {
    set_address(reg);
    sio_hw->gpio_set = (1 << IDE_DIOR) | (1 << IDE_DIOW); 
    data_mode_in();
    sio_hw->gpio_clr = (1 << IDE_CS0);
    busy_wait_at_least_cycles(20);
    sio_hw->gpio_clr = (1 << IDE_DIOR);
    
    // CHS_Bridge used 200, but 400 is safer for stability if you have it. 
    // We will stick to 400 as it's within spec for PIO Mode 0.
    busy_wait_at_least_cycles(400); 
    
    uint8_t val = (uint8_t)(sio_hw->gpio_in & 0xFF);
    sio_hw->gpio_set = (1 << IDE_DIOR) | (1 << IDE_CS0);
    bus_idle();
    return val;
}

uint8_t ide_read_register(uint8_t reg) {
    return ide_read_8(reg);
}

uint16_t ide_read_16(uint8_t reg) {
    set_address(reg);
    // Ensure DIOR starts HIGH
    sio_hw->gpio_set = (1 << IDE_DIOR) | (1 << IDE_DIOW);
    data_mode_in();
    
    sio_hw->gpio_clr = (1 << IDE_CS0);
    busy_wait_at_least_cycles(50); // Setup time (t1)
    
    sio_hw->gpio_clr = (1 << IDE_DIOR);
    busy_wait_at_least_cycles(500); // Pulse width (t2) - Increased for stability
    
    uint16_t val = (uint16_t)(sio_hw->gpio_in & DATA_MASK);
    
    sio_hw->gpio_set = (1 << IDE_DIOR);
    busy_wait_at_least_cycles(500); // Recovery time (t9) - CRITICAL for sector reads
    
    sio_hw->gpio_set = (1 << IDE_CS0);
    bus_idle();
    
    return val;
}

void ide_write_8(uint8_t reg, uint8_t val) {
    set_address(reg);
    data_mode_out(val);
    sio_hw->gpio_clr = (1 << IDE_CS0);
    busy_wait_at_least_cycles(10);
    sio_hw->gpio_clr = (1 << IDE_DIOW);    
    busy_wait_at_least_cycles(200); 
    sio_hw->gpio_set = (1 << IDE_DIOW) | (1 << IDE_CS0);
    bus_idle();
    busy_wait_at_least_cycles(200);
}

void ide_write_16(uint8_t reg, uint16_t val) {
    set_address(reg);
    data_mode_out(val);
    sio_hw->gpio_clr = (1 << IDE_CS0);
    busy_wait_at_least_cycles(10);
    sio_hw->gpio_clr = (1 << IDE_DIOW);    
    busy_wait_at_least_cycles(200); 
    sio_hw->gpio_set = (1 << IDE_DIOW) | (1 << IDE_CS0);
    bus_idle();
    busy_wait_at_least_cycles(200);
}

void ide_flush_cache_ext(void) {
    ide_write_8(6, 0xE0);
    ide_write_8(7, 0xEA); // FLUSH CACHE EXT
    ide_wait_until_ready(2000);
}

static inline void ide_setup_lba28(uint32_t lba, uint8_t count) {
    ide_write_8(2, count);           // Sector count
    ide_write_8(3, lba & 0xFF);      // LBA 0-7
    ide_write_8(4, (lba >> 8) & 0xFF);  // LBA 8-15
    ide_write_8(5, (lba >> 16) & 0xFF); // LBA 16-23
    ide_write_8(6, 0xE0 | ((lba >> 24) & 0x0F)); // Master + LBA + Top 4 bits
}

static inline void ide_setup_lba48(uint64_t lba, uint16_t count){
    // Caller must never pass count==0. Write the 16-bit count directly.

    // --- High bytes FIRST (HOB) ---
    ide_write_8(2, (count >> 8) & 0xFF);       // Sector count high
    ide_write_8(3, (lba >> 24) & 0xFF);        // LBA 24–31
    ide_write_8(4, (lba >> 32) & 0xFF);        // LBA 32–39
    ide_write_8(5, (lba >> 40) & 0xFF);        // LBA 40–47

    // --- Low bytes ---
    ide_write_8(2, count & 0xFF);              // Sector count low
    ide_write_8(3, lba & 0xFF);                // LBA 0–7
    ide_write_8(4, (lba >> 8) & 0xFF);         // LBA 8–15
    ide_write_8(5, (lba >> 16) & 0xFF);        // LBA 16–23

    // Select master + LBA mode
    ide_write_8(6, 0xE0);
}

static inline void invalidate_read_cache_range(uint64_t lba, uint32_t count)
{
    if (read_cache_count == 0) return;

    uint64_t cache_end = read_cache_start + read_cache_count;
    uint64_t write_end = lba + count;

    if (!(write_end <= read_cache_start || lba >= cache_end)) {
        read_cache_count = 0;
        read_cache_start = 0xFFFFFFFFFFFFFFFF;
    }
}



int32_t ide_write_sectors_lba(uint32_t lba32, uint32_t count, const uint8_t* buffer)
{
    uint64_t lba = (uint64_t)lba32;
    invalidate_read_cache_range(lba, count);

    if (drive_write_protected || count == 0)
        return -1;

    uint64_t max_sectors = use_lba_mode ? total_lba_sectors_from_identify 
                                        : (uint64_t)cur_cyls * (uint64_t)cur_heads * (uint64_t)cur_spt;

    if (max_sectors == 0) return -1;
    if (count > max_sectors) return -1;
    if (lba > max_sectors - count) return -1;

    uint32_t remaining = count;
    uint64_t current_lba = lba;
    size_t buf_off = 0;

    while (remaining > 0) {
        uint32_t max_xfer = (use_lba_mode && drive_supports_lba48) ? 0xFFFFu : 0xFFu;
        uint32_t xfer = remaining > max_xfer ? max_xfer : remaining;

        // Setup command for this chunk
        if (use_lba_mode) {
            if (drive_supports_lba48) {
                ide_setup_lba48(current_lba, (uint16_t)xfer);
                ide_write_8(7, 0x34); // WRITE SECTORS EXT
            } else {
                ide_setup_lba28((uint32_t)current_lba, (uint8_t)xfer);
                ide_write_8(7, 0x30); // WRITE SECTORS
            }
        } else {
            uint16_t c; uint8_t h, s;
            lba_to_chs((uint32_t)current_lba, &c, &h, &s);
            ide_write_8(2, (uint8_t)xfer);          // Sector Count
            ide_write_8(3, s);                     // Sector Number
            ide_write_8(4, (uint8_t)(c & 0xFF));   // Cylinder Low
            ide_write_8(5, (uint8_t)(c >> 8));     // Cylinder High
            uint8_t reg6_val = 0xA0 | (h & 0x0F);
            ide_write_8(6, reg6_val); 
            ide_write_8(7, 0x30);                  // WRITE SECTORS
        }

        // Transfer xfer sectors for this chunk
        for (uint32_t i = 0; i < xfer; i++) {
            uint32_t timeout = 1000000;
            while (timeout--) {
                uint8_t st = ide_read_8(7);
                if (st & 0x01) return -1; // ERR
                if (!(st & 0x80) && (st & 0x08)) break; // DRQ ready
                tud_task();
                busy_wait_us_32(10);
            }

            set_address(0);
            sio_hw->gpio_set = (1 << IDE_OE) | (1 << IDE_OE1);
            sio_hw->gpio_oe_set = DATA_MASK;
            sio_hw->gpio_set = (1 << IDE_DIR) | (1 << IDE_DIR1);
            busy_wait_at_least_cycles(50);
            sio_hw->gpio_clr = (1 << IDE_OE) | (1 << IDE_OE1);

            sio_hw->gpio_clr = (1 << IDE_CS0);
            busy_wait_at_least_cycles(50);

            const uint8_t* p = buffer + buf_off + (size_t)i * 512;
            for (int j = 0; j < 256; j++) {
                uint16_t w = p[j * 2] | (p[j * 2 + 1] << 8);
                sio_hw->gpio_out = (sio_hw->gpio_out & ~DATA_MASK) | w;
                busy_wait_at_least_cycles(20); 
                sio_hw->gpio_clr = (1 << IDE_DIOW);
                busy_wait_at_least_cycles(75);
                sio_hw->gpio_set = (1 << IDE_DIOW);
                busy_wait_at_least_cycles(75);
            }

            sio_hw->gpio_set = (1 << IDE_CS0);
            bus_idle();
        }

        remaining -= xfer;
        current_lba += xfer;
        buf_off += (size_t)xfer * 512;
    }

    if (!ide_wait_until_ready(500)) {
        ide_flush_cache_ext();
        return -1;
    }

    return (int32_t)(count * 512);
}




int32_t ide_read_sectors_lba(uint32_t lba32, uint32_t count, uint8_t* buffer)
{
    uint64_t lba = (uint64_t)lba32;

    uint64_t max_sectors = use_lba_mode ? total_lba_sectors_from_identify 
                                        : (uint64_t)cur_cyls * (uint64_t)cur_heads * (uint64_t)cur_spt;

    if (count == 0) return -1;
    if (max_sectors == 0) return -1;
    if (count > max_sectors) return -1;
    if (lba > max_sectors - count) return -1;

    uint32_t remaining = count;
    uint64_t current_lba = lba;
    size_t buf_off = 0;

    while (remaining > 0) {
        uint32_t max_xfer = (use_lba_mode && drive_supports_lba48) ? 0xFFFFu : 0xFFu;
        uint32_t xfer = remaining > max_xfer ? max_xfer : remaining;

        if (use_lba_mode) {
            if (drive_supports_lba48) {
                ide_setup_lba48(current_lba, (uint16_t)xfer);
                ide_write_8(7, 0x24); // READ SECTORS EXT
            } else {
                ide_setup_lba28((uint32_t)current_lba, (uint8_t)xfer);
                ide_write_8(7, 0x20); // READ SECTORS (Standard)
            }
        } else {
            uint16_t c; uint8_t h, s;
            lba_to_chs((uint32_t)current_lba, &c, &h, &s);
            ide_write_8(2, (uint8_t)xfer);
            ide_write_8(3, s);
            ide_write_8(4, (uint8_t)(c & 0xFF));
            ide_write_8(5, (uint8_t)(c >> 8));
            uint8_t reg6_val = 0xA0 | (h & 0x0F);
            ide_write_8(6, reg6_val);
            ide_write_8(7, 0x20);
        }

        for (uint32_t i = 0; i < xfer; i++) {
            uint32_t timeout = 1000000;
            while (timeout--) {
                uint8_t st = ide_read_8(7);
                if (st & 0x01) return -1; // ERR
                if (!(st & 0x80) && (st & 0x08)) break; // DRQ set and BSY clear
                tud_task();
                busy_wait_us_32(10);
            }

            set_address(0);
            data_mode_in();
            sio_hw->gpio_clr = (1 << IDE_CS0);
            busy_wait_at_least_cycles(50);

            uint8_t* p = buffer + buf_off + (size_t)i * 512;
            for (int j = 0; j < 256; j++) {
                sio_hw->gpio_clr = (1 << IDE_DIOR);
                busy_wait_at_least_cycles(75);
                uint16_t w = sio_hw->gpio_in & DATA_MASK;
                p[j * 2]     = w & 0xFF;
                p[j * 2 + 1] = w >> 8;
                sio_hw->gpio_set = (1 << IDE_DIOR);
                busy_wait_at_least_cycles(75);
            }

            sio_hw->gpio_set = (1 << IDE_CS0);
            bus_idle();
        }

        remaining -= xfer;
        current_lba += xfer;
        buf_off += (size_t)xfer * 512;
    }

    if (!ide_wait_until_ready(500))
        return -1;

    return (int32_t)(count * 512);
}




bool ide_wait_until_ready(uint32_t timeout_ms) {
    uint32_t start = board_millis();
    while (board_millis() - start < timeout_ms) {
        uint8_t status = ide_read_8(7);
        // Bit 7 (BSY) must be 0 for the drive to accept commands
        if (!(status & 0x80)) {
            return true;
        }
        
        // --- THE CRITICAL FIX ---
        // Without this, the Pico stops responding to Windows during seek/spin-up
        tud_task(); 
        busy_wait_us_32(10); 
    }
    return false;
}

void ide_hw_init(void) {
    uint32_t all_pins = DATA_MASK | ADDR_MASK | CTRL_MASK | 
                        (1 << IDE_RESET) | (1 << IDE_DIR) | 
                        (1 << IDE_DIR1) | (1 << IDE_OE) | (1 << IDE_OE1);
    
    // 1. Initialize all pins
    gpio_init_mask(all_pins);
    
    // 2. CRITICAL FIX: Explicitly disable pulls on Data Bus (pins 0-15)
    // CHS_Bridge.cpp did this, and it is required for vintage drives.
    for (int i = 0; i < 16; i++) {
        gpio_disable_pulls(i);
    }
    
    gpio_set_dir_out_masked(all_pins & ~DATA_MASK);
    gpio_put(IDE_RESET, 1);
    bus_idle();
}

void ide_reset_drive() {
    gpio_put(IDE_RESET, 0); 
    sleep_ms(50); 
    gpio_put(IDE_RESET, 1);
    sleep_ms(100); 
    is_mounted = false; 
}

bool wait_for_drive_ready(uint32_t timeout_ms) {
    uint32_t start = board_millis();
    while (board_millis() - start < timeout_ms) {
        uint8_t status = ide_read_register(IDE_REG_STATUS);
        if (!(status & 0x80) && (status & 0x40)) return true;
        sleep_ms(10);
    }
    return false; 
}

void ide_identify_drive() {
    // 1. Select Drive 0
    ide_write_8(6, 0xA0); 
    busy_wait_us_32(500); 

    // 2. Send IDENTIFY
    ide_write_8(7, 0xEC); 
    
    // NOTE: Removed the immediate bus_idle() here. 
    // We want to hold the bus state until the get_identify_data function takes over.
}

bool ide_get_identify_data(uint16_t* buffer) {
    uint32_t timeout = 100000; 
    while (timeout--) {
        uint8_t status = ide_read_8(7);
        if (status & 0x01) return false; // Aborted
        
        // RELAXED CHECK: Match CHS_Bridge.cpp
        // Break as soon as DRQ (0x08) is set. Ignore BSY for now.
        if (status & 0x08) break;
        
        busy_wait_us_32(50); 
        if (timeout % 100 == 0) tud_task(); 
    }

    if (timeout == 0) return false;

    // --- START OF OLD PROJECT LOGIC ---
    set_address(0); 
    data_mode_in();
    sio_hw->gpio_clr = (1 << IDE_CS0);
    busy_wait_at_least_cycles(100); // 100 cycles setup

    for (int i = 0; i < 256; i++) {
        sio_hw->gpio_clr = (1 << IDE_DIOR);
        busy_wait_at_least_cycles(400); // Pulse width
        buffer[i] = (uint16_t)(sio_hw->gpio_in & DATA_MASK);
        sio_hw->gpio_set = (1 << IDE_DIOR);
        busy_wait_at_least_cycles(400); // Recovery
    }
    
    drive_supports_lba48 = (buffer[83] & (1 << 10)) != 0;   // Check Word 83 Bit 10 for LBA48 support

    sio_hw->gpio_set = (1 << IDE_CS0);
    bus_idle();    
    return true;
}

void get_large_geometry(uint16_t native_cyl, uint8_t native_head, uint8_t native_spt, 
                        uint16_t* l_cyl, uint8_t* l_head) {
    *l_head = native_head << 1; 
    *l_cyl = native_cyl >> 1;
}

bool ide_set_geometry(uint8_t heads, uint8_t spt) {
    ide_write_8(6, 0xA0 | ((heads - 1) & 0x0F)); 
    ide_write_8(2, spt);
    ide_write_8(7, 0x91); 
    return ide_wait_until_ready(1000);
}



// Returns bytes read (count*512) or -1 on error
int32_t ide_read_cached(uint32_t lba, uint32_t count, uint8_t* buffer) {
    if (count == 0) return 0;

    // Compute device max sectors (same logic as MSC)
    uint64_t max_sectors = use_lba_mode ? total_lba_sectors_from_identify
                                       : (uint64_t)cur_cyls * (uint64_t)cur_heads * (uint64_t)cur_spt;

    if (lba >= max_sectors) return -1;
    if (lba + count > max_sectors) count = max_sectors - lba;

    // Large requests bypass cache (stream directly)
    if (count >= READ_CACHE_SECTORS) {
        return ide_read_sectors_lba(lba, count, buffer);
    }

    // Check cache hit
    if (read_cache_count > 0 &&
        lba >= read_cache_start &&
        (lba + count) <= (read_cache_start + read_cache_count)) {

        uint32_t offset_sectors = lba - read_cache_start;
        memcpy(buffer, &read_cache_buf[offset_sectors * 512], count * 512);
        return (int32_t)(count * 512);
    }

    // Cache miss: prefetch starting at requested LBA
    uint32_t fill_start = lba;
    uint32_t fill_count = READ_CACHE_SECTORS;
    if (fill_start + fill_count > max_sectors) fill_count = max_sectors - fill_start;

    int32_t ret = ide_read_sectors_lba(fill_start, fill_count, read_cache_buf);
    if (ret < 0) {
        // failed read
        read_cache_count = 0;
        read_cache_start = 0xFFFFFFFFFFFFFFFF;
        return -1;
    }

    read_cache_start = fill_start;
    read_cache_count = fill_count;

    // copy requested portion
    memcpy(buffer, &read_cache_buf[0], count * 512);
    return (int32_t)(count * 512);
}

} // End extern "C"