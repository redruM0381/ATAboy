#include "ide_logic.h" 
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/structs/sio.h"
#include "hardware/sync.h"
#include "tusb.h"
#include "pico/time.h"
#include <string.h>



// Immediate mitigation: prevent TinyUSB re-entry from IDE I/O paths.
// Replace calls to tud_task() in this file with TUSB_POLL_SAFE(), which
// yields without re-entering TinyUSB.  This will pause USB/CDC briefly
// during IDE ops but avoids reentrancy crashes.
#ifndef TUSB_POLL_SAFE
#define TUSB_POLL_SAFE() tight_loop_contents()
#endif

// Map board_millis to the Pico SDK equivalent
#define board_millis() to_ms_since_boot(get_absolute_time())
#define IDE_REG_STATUS 7

extern "C" {

// Access the global flag from menus.c
extern bool comp_timings;

// --- Simple read cache ---
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
extern uint64_t total_lba_sectors_from_identify; 
extern uint16_t cur_cyls;
extern uint8_t  cur_heads;
extern uint8_t  cur_spt;

// Forward decls for helpers
static inline void ide_setup_lba48(uint64_t lba, uint16_t count);
static inline void ide_setup_lba28(uint32_t lba, uint8_t count);

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
#define ERROR_LED   32  // GPIO for error LED (active high)



/* ================= BUS LOGIC HELPERS ================= */

void ide_flush_cache() {
    read_cache_start = 0xFFFFFFFFFFFFFFFF;
    read_cache_count = 0;
}

void bus_idle() {
    sio_hw->gpio_set = (1 << IDE_CS0) | (1 << IDE_CS1) | (1 << IDE_DIOR) | (1 << IDE_DIOW);
    sio_hw->gpio_clr = (1 << IDE_DIR) | (1 << IDE_DIR1); // In
    sio_hw->gpio_set = (1 << IDE_OE) | (1 << IDE_OE1);   // Disable
    sio_hw->gpio_oe_clr = DATA_MASK; 
}

void set_address(uint8_t addr) {
    uint32_t addr_val = (uint32_t)(addr & 0x07) << IDE_A0;
    sio_hw->gpio_clr = ADDR_MASK;
    sio_hw->gpio_set = addr_val;
}

void data_mode_in() {
    sio_hw->gpio_set = (1 << IDE_OE) | (1 << IDE_OE1); 
    sio_hw->gpio_oe_clr = DATA_MASK;
    sio_hw->gpio_clr = (1 << IDE_DIR) | (1 << IDE_DIR1);
    busy_wait_at_least_cycles(20); 
    sio_hw->gpio_clr = (1 << IDE_OE) | (1 << IDE_OE1);
}

// NOTE: We will use separate data_mode_out timings inline for compat vs fast
void data_mode_out_fast(uint16_t val) {
    sio_hw->gpio_set = (1 << IDE_OE) | (1 << IDE_OE1);
    sio_hw->gpio_out = (sio_hw->gpio_out & ~DATA_MASK) | ((uint32_t)val & DATA_MASK);
    sio_hw->gpio_oe_set = DATA_MASK;
    sio_hw->gpio_set = (1 << IDE_DIR) | (1 << IDE_DIR1);
    busy_wait_at_least_cycles(100); 
    sio_hw->gpio_clr = (1 << IDE_OE) | (1 << IDE_OE1);
}

void data_mode_out_compat(uint16_t val) {
    sio_hw->gpio_set = (1 << IDE_OE) | (1 << IDE_OE1);
    sio_hw->gpio_out = (sio_hw->gpio_out & ~DATA_MASK) | ((uint32_t)val & DATA_MASK);
    sio_hw->gpio_oe_set = DATA_MASK;
    sio_hw->gpio_set = (1 << IDE_DIR) | (1 << IDE_DIR1);
    busy_wait_at_least_cycles(400); // Increased buffer setup
    sio_hw->gpio_clr = (1 << IDE_OE) | (1 << IDE_OE1);
}

void lba_to_chs(uint32_t lba, uint16_t* c, uint8_t* h, uint8_t* s) {
    *s = (lba % drive_spt) + 1;
    uint32_t temp = lba / drive_spt;
    *h = temp % drive_heads;
    *c = temp / drive_heads;
}

/* ================= LOW LEVEL REGISTER I/O (DUPLICATED) ================= */




// --- FAST VERSIONS (Original Timings) ---
uint8_t ide_read_8_fast(uint8_t reg) {
    set_address(reg);
    sio_hw->gpio_set = (1 << IDE_DIOR) | (1 << IDE_DIOW); 
    data_mode_in();
    sio_hw->gpio_clr = (1 << IDE_CS0);
    busy_wait_at_least_cycles(20);
    sio_hw->gpio_clr = (1 << IDE_DIOR);
    busy_wait_at_least_cycles(400); 
    uint8_t val = (uint8_t)(sio_hw->gpio_in & 0xFF);
    sio_hw->gpio_set = (1 << IDE_DIOR) | (1 << IDE_CS0);
    bus_idle();
    return val;
}

void ide_write_8_fast(uint8_t reg, uint8_t val) {
    set_address(reg);
    data_mode_out_fast(val);
    sio_hw->gpio_clr = (1 << IDE_CS0);
    busy_wait_at_least_cycles(10);
    sio_hw->gpio_clr = (1 << IDE_DIOW);    
    busy_wait_at_least_cycles(200); 
    sio_hw->gpio_set = (1 << IDE_DIOW) | (1 << IDE_CS0);
    bus_idle();
    busy_wait_at_least_cycles(200);
}

uint16_t ide_read_16_fast(uint8_t reg) {
    set_address(reg);
    sio_hw->gpio_set = (1 << IDE_DIOR) | (1 << IDE_DIOW);
    data_mode_in();
    sio_hw->gpio_clr = (1 << IDE_CS0);
    busy_wait_at_least_cycles(50);
    sio_hw->gpio_clr = (1 << IDE_DIOR);
    busy_wait_at_least_cycles(500);
    uint16_t val = (uint16_t)(sio_hw->gpio_in & DATA_MASK);
    sio_hw->gpio_set = (1 << IDE_DIOR);
    busy_wait_at_least_cycles(500);
    sio_hw->gpio_set = (1 << IDE_CS0);
    bus_idle();
    return val;
}

// --- COMPAT VERSIONS (Relaxed Timings) ---
uint8_t ide_read_8_compat(uint8_t reg) {
    set_address(reg);
    sio_hw->gpio_set = (1 << IDE_DIOR) | (1 << IDE_DIOW); 
    data_mode_in();
    sio_hw->gpio_clr = (1 << IDE_CS0);
    busy_wait_at_least_cycles(150); // Setup increased
    sio_hw->gpio_clr = (1 << IDE_DIOR);
    busy_wait_at_least_cycles(1200); // Pulse increased significantly
    uint8_t val = (uint8_t)(sio_hw->gpio_in & 0xFF);
    sio_hw->gpio_set = (1 << IDE_DIOR) | (1 << IDE_CS0);
    bus_idle();
    return val;
}

void ide_write_8_compat(uint8_t reg, uint8_t val) {
    set_address(reg);
    data_mode_out_compat(val); // Uses relaxed data_mode_out
    sio_hw->gpio_clr = (1 << IDE_CS0);
    busy_wait_at_least_cycles(150);
    sio_hw->gpio_clr = (1 << IDE_DIOW);    
    busy_wait_at_least_cycles(1200); // Pulse increased
    sio_hw->gpio_set = (1 << IDE_DIOW) | (1 << IDE_CS0);
    bus_idle();
    busy_wait_at_least_cycles(1200); // Recovery increased
}

uint16_t ide_read_16_compat(uint8_t reg) {
    set_address(reg);
    sio_hw->gpio_set = (1 << IDE_DIOR) | (1 << IDE_DIOW);
    data_mode_in();
    sio_hw->gpio_clr = (1 << IDE_CS0);
    busy_wait_at_least_cycles(200);
    sio_hw->gpio_clr = (1 << IDE_DIOR);
    busy_wait_at_least_cycles(1500);
    uint16_t val = (uint16_t)(sio_hw->gpio_in & DATA_MASK);
    sio_hw->gpio_set = (1 << IDE_DIOR);
    busy_wait_at_least_cycles(1500);
    sio_hw->gpio_set = (1 << IDE_CS0);
    bus_idle();
    return val;
}

/* ================= DISPATCHERS ================= */
uint8_t ide_read_8(uint8_t reg) {
    uint8_t val = comp_timings ? ide_read_8_compat(reg) : ide_read_8_fast(reg);
    if (reg == 1) { // ATA Error register
        gpio_put(ERROR_LED, val ? 1 : 0);
    }
    return val;
}

void ide_write_8(uint8_t reg, uint8_t val) {
    if (comp_timings) ide_write_8_compat(reg, val);
    else ide_write_8_fast(reg, val);
}

uint16_t ide_read_16(uint8_t reg) {
    return comp_timings ? ide_read_16_compat(reg) : ide_read_16_fast(reg);
}

uint8_t ide_read_register(uint8_t reg) {
    return ide_read_8(reg);
}

// Helpers calling the dispatcher (which calls the correct version)
void ide_flush_cache_ext(void) {
    ide_write_8(6, 0xE0);
    ide_write_8(7, 0xEA); 
    ide_wait_until_ready(2000);
}

static inline void ide_setup_lba28(uint32_t lba, uint8_t count) {
    ide_write_8(2, count);           
    ide_write_8(3, lba & 0xFF);      
    ide_write_8(4, (lba >> 8) & 0xFF);  
    ide_write_8(5, (lba >> 16) & 0xFF); 
    ide_write_8(6, 0xE0 | ((lba >> 24) & 0x0F)); 
}

static inline void ide_setup_lba48(uint64_t lba, uint16_t count){
    ide_write_8(2, (count >> 8) & 0xFF);       
    ide_write_8(3, (lba >> 24) & 0xFF);        
    ide_write_8(4, (lba >> 32) & 0xFF);        
    ide_write_8(5, (lba >> 40) & 0xFF);        
    ide_write_8(2, count & 0xFF);              
    ide_write_8(3, lba & 0xFF);                
    ide_write_8(4, (lba >> 8) & 0xFF);         
    ide_write_8(5, (lba >> 16) & 0xFF);        
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

void ide_get_task_file(uint8_t* task_file) {
    // We read registers 1 through 7
    // 1: Error, 2: Sector Count, 3: Sector Number, 4: Cyl Low, 
    // 5: Cyl High, 6: Drive/Head, 7: Status
    for (int i = 1; i <= 7; i++) {
        task_file[i] = ide_read_8(i);
    }
}

/* ================= SECTOR I/O (DUPLICATED) ================= */

// --- WRITE SECTORS FAST ---
int32_t ide_write_sectors_lba_fast(uint32_t lba32, uint32_t count, const uint8_t* buffer)
{
    uint64_t lba = (uint64_t)lba32;
    invalidate_read_cache_range(lba, count);

    if (drive_write_protected || count == 0) return -1;

    uint64_t max_sectors = use_lba_mode ? total_lba_sectors_from_identify 
                                        : (uint64_t)cur_cyls * cur_heads * cur_spt;
    if (max_sectors == 0 || count > max_sectors || lba > max_sectors - count) return -1;

    uint32_t remaining = count;
    uint64_t current_lba = lba;
    size_t buf_off = 0;

    while (remaining > 0) {
        uint32_t max_xfer = (use_lba_mode && drive_supports_lba48) ? 0xFFFFu : 0xFFu;
        uint32_t xfer = remaining > max_xfer ? max_xfer : remaining;

        if (use_lba_mode) {
            if (drive_supports_lba48) {
                ide_setup_lba48(current_lba, (uint16_t)xfer);
                ide_write_8_fast(7, 0x34); 
            } else {
                ide_setup_lba28((uint32_t)current_lba, (uint8_t)xfer);
                ide_write_8_fast(7, 0x30); 
            }
        } else {
            uint16_t c; uint8_t h, s;
            lba_to_chs((uint32_t)current_lba, &c, &h, &s);
            ide_write_8_fast(2, (uint8_t)xfer);
            ide_write_8_fast(3, s);
            ide_write_8_fast(4, (uint8_t)(c & 0xFF));
            ide_write_8_fast(5, (uint8_t)(c >> 8));
            ide_write_8_fast(6, 0xA0 | (h & 0x0F)); 
            ide_write_8_fast(7, 0x30);
        }

        for (uint32_t i = 0; i < xfer; i++) {
            uint32_t timeout = 1000000;
            while (timeout--) {
                uint8_t st = ide_read_8_fast(7);
                if (st & 0x01) return -1; 
                if (!(st & 0x80) && (st & 0x08)) break; 
                TUSB_POLL_SAFE();
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
                busy_wait_at_least_cycles(75); // FAST PULSE
                sio_hw->gpio_set = (1 << IDE_DIOW);
                busy_wait_at_least_cycles(75); // FAST RECOVERY
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

// --- WRITE SECTORS COMPAT ---
int32_t ide_write_sectors_lba_compat(uint32_t lba32, uint32_t count, const uint8_t* buffer)
{
    uint64_t lba = (uint64_t)lba32;
    invalidate_read_cache_range(lba, count);

    if (drive_write_protected || count == 0) return -1;

    uint64_t max_sectors = use_lba_mode ? total_lba_sectors_from_identify 
                                        : (uint64_t)cur_cyls * cur_heads * cur_spt;
    if (max_sectors == 0 || count > max_sectors || lba > max_sectors - count) return -1;

    uint32_t remaining = count;
    uint64_t current_lba = lba;
    size_t buf_off = 0;

    while (remaining > 0) {
        uint32_t max_xfer = (use_lba_mode && drive_supports_lba48) ? 0xFFFFu : 0xFFu;
        uint32_t xfer = remaining > max_xfer ? max_xfer : remaining;

        // Use COMPAT register writes
        if (use_lba_mode) {
            if (drive_supports_lba48) {
                ide_setup_lba48(current_lba, (uint16_t)xfer); // uses wrapper -> compat
                ide_write_8_compat(7, 0x34); 
            } else {
                ide_setup_lba28((uint32_t)current_lba, (uint8_t)xfer);
                ide_write_8_compat(7, 0x30); 
            }
        } else {
            uint16_t c; uint8_t h, s;
            lba_to_chs((uint32_t)current_lba, &c, &h, &s);
            ide_write_8_compat(2, (uint8_t)xfer);
            ide_write_8_compat(3, s);
            ide_write_8_compat(4, (uint8_t)(c & 0xFF));
            ide_write_8_compat(5, (uint8_t)(c >> 8));
            ide_write_8_compat(6, 0xA0 | (h & 0x0F)); 
            ide_write_8_compat(7, 0x30);
        }

        for (uint32_t i = 0; i < xfer; i++) {
            uint32_t timeout = 1000000;
            while (timeout--) {
                uint8_t st = ide_read_8_compat(7);
                if (st & 0x01) return -1; 
                if (!(st & 0x80) && (st & 0x08)) break; 
                TUSB_POLL_SAFE();
                busy_wait_us_32(10);
            }

            set_address(0);
            sio_hw->gpio_set = (1 << IDE_OE) | (1 << IDE_OE1);
            sio_hw->gpio_oe_set = DATA_MASK;
            sio_hw->gpio_set = (1 << IDE_DIR) | (1 << IDE_DIR1);
            busy_wait_at_least_cycles(400); // Setup increased
            sio_hw->gpio_clr = (1 << IDE_OE) | (1 << IDE_OE1);
            sio_hw->gpio_clr = (1 << IDE_CS0);
            busy_wait_at_least_cycles(150);

            const uint8_t* p = buffer + buf_off + (size_t)i * 512;
            for (int j = 0; j < 256; j++) {
                uint16_t w = p[j * 2] | (p[j * 2 + 1] << 8);
                sio_hw->gpio_out = (sio_hw->gpio_out & ~DATA_MASK) | w;
                busy_wait_at_least_cycles(100); 
                sio_hw->gpio_clr = (1 << IDE_DIOW);
                busy_wait_at_least_cycles(300); // COMPAT PULSE
                sio_hw->gpio_set = (1 << IDE_DIOW);
                busy_wait_at_least_cycles(300); // COMPAT RECOVERY
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

int32_t ide_write_sectors_lba(uint32_t lba, uint32_t count, const uint8_t* buffer) {
    if (comp_timings) return ide_write_sectors_lba_compat(lba, count, buffer);
    else return ide_write_sectors_lba_fast(lba, count, buffer);
}

// --- READ SECTORS FAST ---
int32_t ide_read_sectors_lba_fast(uint32_t lba32, uint32_t count, uint8_t* buffer)
{
    uint64_t lba = (uint64_t)lba32;
    uint64_t max_sectors = use_lba_mode ? total_lba_sectors_from_identify 
                                        : (uint64_t)cur_cyls * cur_heads * cur_spt;

    if (count == 0 || max_sectors == 0 || count > max_sectors || lba > max_sectors - count) return -1;

    uint32_t remaining = count;
    uint64_t current_lba = lba;
    size_t buf_off = 0;

    while (remaining > 0) {
        uint32_t max_xfer = (use_lba_mode && drive_supports_lba48) ? 0xFFFFu : 0xFFu;
        uint32_t xfer = remaining > max_xfer ? max_xfer : remaining;

        if (use_lba_mode) {
            if (drive_supports_lba48) {
                ide_setup_lba48(current_lba, (uint16_t)xfer);
                ide_write_8_fast(7, 0x24); 
            } else {
                ide_setup_lba28((uint32_t)current_lba, (uint8_t)xfer);
                ide_write_8_fast(7, 0x20); 
            }
        } else {
            uint16_t c; uint8_t h, s;
            lba_to_chs((uint32_t)current_lba, &c, &h, &s);
            ide_write_8_fast(2, (uint8_t)xfer);
            ide_write_8_fast(3, s);
            ide_write_8_fast(4, (uint8_t)(c & 0xFF));
            ide_write_8_fast(5, (uint8_t)(c >> 8));
            ide_write_8_fast(6, 0xA0 | (h & 0x0F));
            ide_write_8_fast(7, 0x20);
        }

        for (uint32_t i = 0; i < xfer; i++) {
            uint32_t timeout = 1000000;
            while (timeout--) {
                uint8_t st = ide_read_8_fast(7);
                if (st & 0x01) return -1; 
                if (!(st & 0x80) && (st & 0x08)) break; 
                TUSB_POLL_SAFE();
                busy_wait_us_32(10);
            }

            set_address(0);
            data_mode_in();
            sio_hw->gpio_clr = (1 << IDE_CS0);
            busy_wait_at_least_cycles(50);

            uint8_t* p = buffer + buf_off + (size_t)i * 512;
            for (int j = 0; j < 256; j++) {
                sio_hw->gpio_clr = (1 << IDE_DIOR);
                busy_wait_at_least_cycles(75); // FAST PULSE
                uint16_t w = sio_hw->gpio_in & DATA_MASK;
                p[j * 2]     = w & 0xFF;
                p[j * 2 + 1] = w >> 8;
                sio_hw->gpio_set = (1 << IDE_DIOR);
                busy_wait_at_least_cycles(75); // FAST RECOVERY
            }

            sio_hw->gpio_set = (1 << IDE_CS0);
            bus_idle();
        }

        remaining -= xfer;
        current_lba += xfer;
        buf_off += (size_t)xfer * 512;
    }

    if (!ide_wait_until_ready(500)) return -1;
    return (int32_t)(count * 512);
}

// --- READ SECTORS COMPAT ---
int32_t ide_read_sectors_lba_compat(uint32_t lba32, uint32_t count, uint8_t* buffer)
{
    uint64_t lba = (uint64_t)lba32;
    uint64_t max_sectors = use_lba_mode ? total_lba_sectors_from_identify 
                                        : (uint64_t)cur_cyls * cur_heads * cur_spt;

    if (count == 0 || max_sectors == 0 || count > max_sectors || lba > max_sectors - count) return -1;

    uint32_t remaining = count;
    uint64_t current_lba = lba;
    size_t buf_off = 0;

    while (remaining > 0) {
        uint32_t max_xfer = (use_lba_mode && drive_supports_lba48) ? 0xFFFFu : 0xFFu;
        uint32_t xfer = remaining > max_xfer ? max_xfer : remaining;

        if (use_lba_mode) {
            if (drive_supports_lba48) {
                ide_setup_lba48(current_lba, (uint16_t)xfer);
                ide_write_8_compat(7, 0x24); 
            } else {
                ide_setup_lba28((uint32_t)current_lba, (uint8_t)xfer);
                ide_write_8_compat(7, 0x20); 
            }
        } else {
            uint16_t c; uint8_t h, s;
            lba_to_chs((uint32_t)current_lba, &c, &h, &s);
            ide_write_8_compat(2, (uint8_t)xfer);
            ide_write_8_compat(3, s);
            ide_write_8_compat(4, (uint8_t)(c & 0xFF));
            ide_write_8_compat(5, (uint8_t)(c >> 8));
            ide_write_8_compat(6, 0xA0 | (h & 0x0F));
            ide_write_8_compat(7, 0x20);
        }

        for (uint32_t i = 0; i < xfer; i++) {
            uint32_t timeout = 1000000;
            while (timeout--) {
                uint8_t st = ide_read_8_compat(7);
                if (st & 0x01) return -1; 
                if (!(st & 0x80) && (st & 0x08)) break; 
                TUSB_POLL_SAFE();
                busy_wait_us_32(10);
            }

            set_address(0);
            data_mode_in();
            sio_hw->gpio_clr = (1 << IDE_CS0);

            volatile uint32_t junk = sio_hw->gpio_in;  // Bus wash
            (void)junk;

            busy_wait_at_least_cycles(200);

            uint8_t* p = buffer + buf_off + (size_t)i * 512;
            for (int j = 0; j < 256; j++) {
                sio_hw->gpio_clr = (1 << IDE_DIOR);
                busy_wait_at_least_cycles(600); // COMPAT PULSE
                uint16_t w = sio_hw->gpio_in & DATA_MASK;
                p[j * 2]     = w & 0xFF;
                p[j * 2 + 1] = w >> 8;
                sio_hw->gpio_set = (1 << IDE_DIOR);
                busy_wait_at_least_cycles(4000); // COMPAT RECOVERY
            }

            sio_hw->gpio_set = (1 << IDE_CS0);
            bus_idle();
        }

        remaining -= xfer;
        current_lba += xfer;
        buf_off += (size_t)xfer * 512;
    }

    if (!ide_wait_until_ready(500)) return -1;
    return (int32_t)(count * 512);
}

int32_t ide_read_sectors_lba(uint32_t lba, uint32_t count, uint8_t* buffer) {
    if (comp_timings) return ide_read_sectors_lba_compat(lba, count, buffer);
    else return ide_read_sectors_lba_fast(lba, count, buffer);
}

// --- GENERAL FUNCS ---

// Poll timer to keep LED updated even when no host commands arrive
static repeating_timer_t error_poll_timer;
static bool error_poll_cb(repeating_timer_t *rt) {
    (void) rt;
    uint8_t err = comp_timings ? ide_read_8_compat(1) : ide_read_8_fast(1);
    gpio_put(ERROR_LED, err ? 1 : 0);
    return true; // keep repeating
}

bool ide_wait_until_ready(uint32_t timeout_ms) {
    uint32_t start = board_millis();
    while (board_millis() - start < timeout_ms) {
        uint8_t status = ide_read_8(7); // Wrapper handles timing
        if (!(status & 0x80)) {
            return true;
        }
        TUSB_POLL_SAFE();
        busy_wait_us_32(10); 
    }
    return false;
}

void ide_hw_init(void) {
    uint32_t all_pins = DATA_MASK | ADDR_MASK | CTRL_MASK | 
                        (1 << IDE_RESET) | (1 << IDE_DIR) | 
                        (1 << IDE_DIR1) | (1 << IDE_OE) | (1 << IDE_OE1);
    
    gpio_init_mask(all_pins);
    for (int i = 0; i < 16; i++) {
        gpio_disable_pulls(i);
    }
    gpio_set_dir_out_masked(all_pins & ~DATA_MASK);
    gpio_put(IDE_RESET, 1);
    bus_idle();

    // Initialize error LED pin
    gpio_init(ERROR_LED);
    gpio_disable_pulls(ERROR_LED);
    gpio_set_dir(ERROR_LED, GPIO_OUT);
    gpio_put(ERROR_LED, 0);

    // Start periodic poll (every 100 ms) to update error LED continuously.
    // Ignore failure silently (e.g., if timer already added).
    add_repeating_timer_ms(100, error_poll_cb, NULL, &error_poll_timer);
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
        uint8_t status = ide_read_8(IDE_REG_STATUS); // Wrapper
        if (!(status & 0x80) && (status & 0x40)) return true;
        sleep_ms(10);
    }
    return false; 
}

void ide_identify_drive() {
    ide_write_8(6, 0xA0); 
    busy_wait_us_32(500); 
    ide_write_8(7, 0xEC); 
}

// IDENTIFY uses unique timing loops, so we duplicate the loop logic here as well
bool ide_get_identify_data(uint16_t* buffer) {
    uint32_t timeout = 100000; 
    while (timeout--) {
        uint8_t status = ide_read_8(7); // Wrapper
        if (status & 0x01) return false; 
        if (status & 0x08) break;
        busy_wait_us_32(50); 
        if (timeout % 100 == 0) TUSB_POLL_SAFE();
    }

    if (timeout == 0) return false;

    set_address(0); 
    data_mode_in();
    sio_hw->gpio_clr = (1 << IDE_CS0);
    volatile uint32_t junk = sio_hw->gpio_in; // "Bus Wash": Read the status register once and throw it away to clear the Pico's internal GPIO state before the 16-bit loop.
    (void)junk;
    busy_wait_at_least_cycles(200); 

    if (comp_timings) {
        // COMPAT LOOP for IDENTIFY
        for (int i = 0; i < 256; i++) {
            sio_hw->gpio_clr = (1 << IDE_DIOR);
            busy_wait_at_least_cycles(600); // Very long pulse
            buffer[i] = (uint16_t)(sio_hw->gpio_in & DATA_MASK);
            sio_hw->gpio_set = (1 << IDE_DIOR);
            busy_wait_at_least_cycles(4000); // very long delay
        }
    } else {
        // FAST LOOP for IDENTIFY
        for (int i = 0; i < 256; i++) {
            sio_hw->gpio_clr = (1 << IDE_DIOR);
            busy_wait_at_least_cycles(400); 
            buffer[i] = (uint16_t)(sio_hw->gpio_in & DATA_MASK);
            sio_hw->gpio_set = (1 << IDE_DIOR);
            busy_wait_at_least_cycles(400); 
        }
    }
    
    drive_supports_lba48 = (buffer[83] & (1 << 10)) != 0; 
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

int32_t ide_read_cached(uint32_t lba, uint32_t count, uint8_t* buffer) {
    if (count == 0) return 0;
    uint64_t max_sectors = use_lba_mode ? total_lba_sectors_from_identify
                                       : (uint64_t)cur_cyls * (uint64_t)cur_heads * (uint64_t)cur_spt;

    if (lba >= max_sectors) return -1;
    if (lba + count > max_sectors) count = max_sectors - lba;

    if (count >= READ_CACHE_SECTORS) {
        return ide_read_sectors_lba(lba, count, buffer);
    }

    if (read_cache_count > 0 &&
        lba >= read_cache_start &&
        (lba + count) <= (read_cache_start + read_cache_count)) {

        uint32_t offset_sectors = lba - read_cache_start;
        memcpy(buffer, &read_cache_buf[offset_sectors * 512], count * 512);
        return (int32_t)(count * 512);
    }

    uint32_t fill_start = lba;
    uint32_t fill_count = READ_CACHE_SECTORS;
    if (fill_start + fill_count > max_sectors) fill_count = max_sectors - fill_start;

    // Use wrapper, so cache fill respects compat timing
    int32_t ret = ide_read_sectors_lba(fill_start, fill_count, read_cache_buf);
    if (ret < 0) {
        read_cache_count = 0;
        read_cache_start = 0xFFFFFFFFFFFFFFFF;
        return -1;
    }

    read_cache_start = fill_start;
    read_cache_count = fill_count;
    memcpy(buffer, &read_cache_buf[0], count * 512);
    return (int32_t)(count * 512);
}

} // End extern "C"