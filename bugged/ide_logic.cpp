#include "ide_logic.h"
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/structs/sio.h"
#include "pico/util/queue.h"
#include "pico/sem.h"
#include "hardware/gpio.h"
#include "pico/time.h"
#include "hardware/sync.h"   // added for interrupt save/restore
#include <string.h>

/* * ARCHITECTURE NOTE:
 * This file contains the low-level IDE (PATA) driver logic.
 * To separate USB (timing sensitive) from IDE (blocking I/O), this file implements 
 * a "Worker" model.
 * * - The actual GPIO manipulation functions are renamed to 'impl_...' and run exclusively on CORE 1.
 * - The public API functions (ide_read_sectors, etc.) act as proxies. They package arguments
 * into a struct, push it to a thread-safe Queue, and block on a Semaphore.
 * - Core 1 sits in a tight loop popping the queue and executing commands.
 */

// --- IPC Definitions ---
typedef enum {
    CMD_READ_SECTORS,
    CMD_WRITE_SECTORS,
    CMD_READ_CACHE,
    CMD_IDENTIFY_SEND,
    CMD_IDENTIFY_READ,
    CMD_RESET,
    CMD_FLUSH_CACHE,
    CMD_SET_GEO,
    CMD_REG_READ,
    CMD_REG_WRITE,
    CMD_GET_TASKFILE
} ide_cmd_type_t;

typedef struct {
    ide_cmd_type_t type;
    uint32_t lba;
    uint32_t count;
    uint8_t* buffer;        // Used for Read/Write buffers
    const uint8_t* c_buffer; // Used for Write buffers
    uint16_t* w_buffer;     // Used for Identify data
    uint8_t reg;
    uint8_t val;
    uint8_t heads;
    uint8_t spt;
    int32_t* result_ptr;    // Pointer to result variable on caller's stack
    semaphore_t* sem;       // Semaphore to signal completion
} ide_req_t;

static queue_t ide_queue;
/* Guard to ensure Core1 worker is only launched once */
static volatile bool core1_worker_started = false;

// Map board_millis to the Pico SDK equivalent
#define board_millis() to_ms_since_boot(get_absolute_time())
#define IDE_REG_STATUS 7

extern "C" {

// Access the global flag from menus.c
extern bool comp_timings;

/* ================= SHARED GLOBALS ================= */
uint8_t  drive_heads = 0;
uint8_t  drive_spt = 0;
uint16_t drive_cylinders = 0; 
bool is_mounted = false; 
bool drive_write_protected = true;
bool use_lba_mode = false;
bool drive_supports_lba48 = false;
bool comp_timings = false; // <-- define here so CORE1 owns this flag

// Internal Globals (Core 1 only)
uint64_t total_lba_sectors_from_identify = 0; 
uint16_t cur_cyls = 0;
uint8_t cur_heads = 0;
uint8_t cur_spt = 0;

// --- Simple read cache (Core 1 only) ---
#ifndef READ_CACHE_SECTORS
#define READ_CACHE_SECTORS 128
#endif
static uint8_t read_cache_buf[READ_CACHE_SECTORS * 512];
static uint64_t read_cache_start = 0xFFFFFFFFFFFFFFFF;
static uint32_t read_cache_count = 0;

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

// Forward declarations of IMPLEMENTATION functions (Core 1)
static int32_t impl_ide_read_sectors_lba(uint32_t lba, uint32_t count, uint8_t* buffer);
static int32_t impl_ide_write_sectors_lba(uint32_t lba, uint32_t count, const uint8_t* buffer);
static void impl_ide_identify_drive();
static bool impl_ide_get_identify_data(uint16_t* buffer);
static void impl_ide_reset_drive();
static void impl_ide_flush_cache();
static bool impl_ide_set_geometry(uint8_t heads, uint8_t spt);
static uint8_t impl_ide_read_8(uint8_t reg);
static void impl_ide_write_8(uint8_t reg, uint8_t val);
static int32_t impl_ide_read_cached(uint32_t lba, uint32_t count, uint8_t* buffer);
static bool impl_ide_wait_until_ready(uint32_t timeout_ms);

/* ================= BUS LOGIC HELPERS (CORE 1) ================= */

static inline void bus_idle() {
    sio_hw->gpio_set = (1 << IDE_CS0) | (1 << IDE_CS1) | (1 << IDE_DIOR) | (1 << IDE_DIOW);
    sio_hw->gpio_clr = (1 << IDE_DIR) | (1 << IDE_DIR1); // In
    sio_hw->gpio_set = (1 << IDE_OE) | (1 << IDE_OE1);   // Disable
    sio_hw->gpio_oe_clr = DATA_MASK; 
}

static inline void set_address(uint8_t addr) {
    uint32_t addr_val = (uint32_t)(addr & 0x07) << IDE_A0;
    sio_hw->gpio_clr = ADDR_MASK;
    sio_hw->gpio_set = addr_val;
}

static inline void data_mode_in() {
    sio_hw->gpio_set = (1 << IDE_OE) | (1 << IDE_OE1); 
    sio_hw->gpio_oe_clr = DATA_MASK;
    sio_hw->gpio_clr = (1 << IDE_DIR) | (1 << IDE_DIR1);
    busy_wait_at_least_cycles(20); 
    sio_hw->gpio_clr = (1 << IDE_OE) | (1 << IDE_OE1);
}

static inline void data_mode_out_fast(uint16_t val) {
    sio_hw->gpio_set = (1 << IDE_OE) | (1 << IDE_OE1);
    sio_hw->gpio_out = (sio_hw->gpio_out & ~DATA_MASK) | ((uint32_t)val & DATA_MASK);
    sio_hw->gpio_oe_set = DATA_MASK;
    sio_hw->gpio_set = (1 << IDE_DIR) | (1 << IDE_DIR1);
    busy_wait_at_least_cycles(100); 
    sio_hw->gpio_clr = (1 << IDE_OE) | (1 << IDE_OE1);
}

static inline void data_mode_out_compat(uint16_t val) {
    sio_hw->gpio_set = (1 << IDE_OE) | (1 << IDE_OE1);
    sio_hw->gpio_out = (sio_hw->gpio_out & ~DATA_MASK) | ((uint32_t)val & DATA_MASK);
    sio_hw->gpio_oe_set = DATA_MASK;
    sio_hw->gpio_set = (1 << IDE_DIR) | (1 << IDE_DIR1);
    busy_wait_at_least_cycles(400); 
    sio_hw->gpio_clr = (1 << IDE_OE) | (1 << IDE_OE1);
}

/* ================= LOW LEVEL REGISTER I/O (CORE 1) ================= */

// --- FAST VERSIONS ---
static inline uint8_t ide_read_8_fast(uint8_t reg) {
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

static inline void ide_write_8_fast(uint8_t reg, uint8_t val) {
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

// --- COMPAT VERSIONS ---
static inline uint8_t ide_read_8_compat(uint8_t reg) {
    set_address(reg);
    sio_hw->gpio_set = (1 << IDE_DIOR) | (1 << IDE_DIOW); 
    data_mode_in();
    sio_hw->gpio_clr = (1 << IDE_CS0);
    busy_wait_at_least_cycles(150); 
    sio_hw->gpio_clr = (1 << IDE_DIOR);
    busy_wait_at_least_cycles(1200); 
    uint8_t val = (uint8_t)(sio_hw->gpio_in & 0xFF);
    sio_hw->gpio_set = (1 << IDE_DIOR) | (1 << IDE_CS0);
    bus_idle();
    return val;
}

static inline void ide_write_8_compat(uint8_t reg, uint8_t val) {
    set_address(reg);
    data_mode_out_compat(val); 
    sio_hw->gpio_clr = (1 << IDE_CS0);
    busy_wait_at_least_cycles(150);
    sio_hw->gpio_clr = (1 << IDE_DIOW);    
    busy_wait_at_least_cycles(1200); 
    sio_hw->gpio_set = (1 << IDE_DIOW) | (1 << IDE_CS0);
    bus_idle();
    busy_wait_at_least_cycles(1200); 
}

// Dispatcher (Core 1)
static uint8_t impl_ide_read_8(uint8_t reg) {
    return comp_timings ? ide_read_8_compat(reg) : ide_read_8_fast(reg);
}

static void impl_ide_write_8(uint8_t reg, uint8_t val) {
    if (comp_timings) ide_write_8_compat(reg, val);
    else ide_write_8_fast(reg, val);
}

// Helper: Wait until ready (Core 1)
static bool impl_ide_wait_until_ready(uint32_t timeout_ms) {
    uint32_t start = board_millis();
    while (board_millis() - start < timeout_ms) {
        uint8_t status = impl_ide_read_8(7); 
        if (!(status & 0x80)) {
            return true;
        }
        busy_wait_us_32(10); 
    }
    return false;
}

static void ide_setup_lba48(uint64_t lba, uint16_t count){
    impl_ide_write_8(2, (count >> 8) & 0xFF);       
    impl_ide_write_8(3, (lba >> 24) & 0xFF);        
    impl_ide_write_8(4, (lba >> 32) & 0xFF);        
    impl_ide_write_8(5, (lba >> 40) & 0xFF);        
    impl_ide_write_8(2, count & 0xFF);              
    impl_ide_write_8(3, lba & 0xFF);                
    impl_ide_write_8(4, (lba >> 8) & 0xFF);         
    impl_ide_write_8(5, (lba >> 16) & 0xFF);        
    impl_ide_write_8(6, 0xE0);
}

static void ide_setup_lba28(uint32_t lba, uint8_t count) {
    impl_ide_write_8(2, count);           
    impl_ide_write_8(3, lba & 0xFF);      
    impl_ide_write_8(4, (lba >> 8) & 0xFF);  
    impl_ide_write_8(5, (lba >> 16) & 0xFF); 
    impl_ide_write_8(6, 0xE0 | ((lba >> 24) & 0x0F)); 
}

static void lba_to_chs(uint32_t lba, uint16_t* c, uint8_t* h, uint8_t* s) {
    *s = (lba % drive_spt) + 1;
    uint32_t temp = lba / drive_spt;
    *h = temp % drive_heads;
    *c = temp / drive_heads;
}

static inline void invalidate_read_cache_range(uint64_t lba, uint32_t count) {
    if (read_cache_count == 0) return;
    uint64_t cache_end = read_cache_start + read_cache_count;
    uint64_t write_end = lba + count;
    if (!(write_end <= read_cache_start || lba >= cache_end)) {
        read_cache_count = 0;
        read_cache_start = 0xFFFFFFFFFFFFFFFF;
    }
}

static void impl_ide_flush_cache() {
    read_cache_start = 0xFFFFFFFFFFFFFFFF;
    read_cache_count = 0;
}

/* ================= IMPLEMENTATION SECTOR I/O (CORE 1) ================= */

static int32_t impl_ide_write_sectors_lba(uint32_t lba32, uint32_t count, const uint8_t* buffer)
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
                impl_ide_write_8(7, 0x34); 
            } else {
                ide_setup_lba28((uint32_t)current_lba, (uint8_t)xfer);
                impl_ide_write_8(7, 0x30); 
            }
        } else {
            uint16_t c; uint8_t h, s;
            lba_to_chs((uint32_t)current_lba, &c, &h, &s);
            impl_ide_write_8(2, (uint8_t)xfer);
            impl_ide_write_8(3, s);
            impl_ide_write_8(4, (uint8_t)(c & 0xFF));
            impl_ide_write_8(5, (uint8_t)(c >> 8));
            impl_ide_write_8(6, 0xA0 | (h & 0x0F)); 
            impl_ide_write_8(7, 0x30);
        }

        for (uint32_t i = 0; i < xfer; i++) {
            uint32_t timeout = 1000000;
            while (timeout--) {
                uint8_t st = impl_ide_read_8(7);
                if (st & 0x01) return -1; 
                if (!(st & 0x80) && (st & 0x08)) break; 
                busy_wait_us_32(10);
            }

            set_address(0);
            sio_hw->gpio_set = (1 << IDE_OE) | (1 << IDE_OE1);
            sio_hw->gpio_oe_set = DATA_MASK;
            sio_hw->gpio_set = (1 << IDE_DIR) | (1 << IDE_DIR1);
            busy_wait_at_least_cycles(comp_timings ? 400 : 50);
            sio_hw->gpio_clr = (1 << IDE_OE) | (1 << IDE_OE1);
            sio_hw->gpio_clr = (1 << IDE_CS0);
            busy_wait_at_least_cycles(comp_timings ? 150 : 50);

            const uint8_t* p = buffer + buf_off + (size_t)i * 512;
            for (int j = 0; j < 256; j++) {
                uint16_t w = p[j * 2] | (p[j * 2 + 1] << 8);
                sio_hw->gpio_out = (sio_hw->gpio_out & ~DATA_MASK) | w;
                busy_wait_at_least_cycles(comp_timings ? 100 : 20); 
                sio_hw->gpio_clr = (1 << IDE_DIOW);
                busy_wait_at_least_cycles(comp_timings ? 300 : 75); 
                sio_hw->gpio_set = (1 << IDE_DIOW);
                busy_wait_at_least_cycles(comp_timings ? 300 : 75); 
            }

            sio_hw->gpio_set = (1 << IDE_CS0);
            bus_idle();
        }

        remaining -= xfer;
        current_lba += xfer;
        buf_off += (size_t)xfer * 512;
    }

    if (!impl_ide_wait_until_ready(500)) {
        impl_ide_write_8(6, 0xE0); // Flush logic inline
        impl_ide_write_8(7, 0xEA); 
        impl_ide_wait_until_ready(2000);
        return -1;
    }
    return (int32_t)(count * 512);
}

static int32_t impl_ide_read_sectors_lba(uint32_t lba32, uint32_t count, uint8_t* buffer)
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
                impl_ide_write_8(7, 0x24); 
            } else {
                ide_setup_lba28((uint32_t)current_lba, (uint8_t)xfer);
                impl_ide_write_8(7, 0x20); 
            }
        } else {
            uint16_t c; uint8_t h, s;
            lba_to_chs((uint32_t)current_lba, &c, &h, &s);
            impl_ide_write_8(2, (uint8_t)xfer);
            impl_ide_write_8(3, s);
            impl_ide_write_8(4, (uint8_t)(c & 0xFF));
            impl_ide_write_8(5, (uint8_t)(c >> 8));
            impl_ide_write_8(6, 0xA0 | (h & 0x0F));
            impl_ide_write_8(7, 0x20);
        }

        for (uint32_t i = 0; i < xfer; i++) {
            uint32_t timeout = 1000000;
            while (timeout--) {
                uint8_t st = impl_ide_read_8(7);
                if (st & 0x01) return -1; 
                if (!(st & 0x80) && (st & 0x08)) break; 
               
            }

            set_address(0);
            data_mode_in();
            sio_hw->gpio_clr = (1 << IDE_CS0);
            if (comp_timings) {
                volatile uint32_t junk = sio_hw->gpio_in; (void)junk;
                busy_wait_at_least_cycles(200);
            } else {
                busy_wait_at_least_cycles(50);
            }

            uint8_t* p = buffer + buf_off + (size_t)i * 512;
            for (int j = 0; j < 256; j++) {
                sio_hw->gpio_clr = (1 << IDE_DIOR);
                busy_wait_at_least_cycles(comp_timings ? 600 : 75); 
                uint16_t w = sio_hw->gpio_in & DATA_MASK;
                p[j * 2]     = w & 0xFF;
                p[j * 2 + 1] = w >> 8;
                sio_hw->gpio_set = (1 << IDE_DIOR);
                busy_wait_at_least_cycles(comp_timings ? 4000 : 75); 
            }

            sio_hw->gpio_set = (1 << IDE_CS0);
            bus_idle();
        }

        remaining -= xfer;
        current_lba += xfer;
        buf_off += (size_t)xfer * 512;
    }

    if (!impl_ide_wait_until_ready(500)) return -1;
    return (int32_t)(count * 512);
}

static int32_t impl_ide_read_cached(uint32_t lba, uint32_t count, uint8_t* buffer) {
    if (count == 0) return 0;
    uint64_t max_sectors = use_lba_mode ? total_lba_sectors_from_identify
                                       : (uint64_t)cur_cyls * (uint64_t)cur_heads * (uint64_t)cur_spt;

    if (lba >= max_sectors) return -1;
    if (lba + count > max_sectors) count = max_sectors - lba;

    // Direct read if large request
    if (count >= READ_CACHE_SECTORS) {
        return impl_ide_read_sectors_lba(lba, count, buffer);
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

    int32_t ret = impl_ide_read_sectors_lba(fill_start, fill_count, read_cache_buf);
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


static void impl_ide_reset_drive() {
    gpio_put(IDE_RESET, 0); 
    sleep_ms(50); 
    gpio_put(IDE_RESET, 1);
    sleep_ms(100); 
    is_mounted = false; 
}

static void impl_ide_identify_drive() {
    impl_ide_write_8(6, 0xA0); 
    busy_wait_us_32(500); 
    impl_ide_write_8(7, 0xEC); 
}

static bool impl_ide_get_identify_data(uint16_t* buffer) {
    uint32_t timeout = 100000; 
    while (timeout--) {
        uint8_t status = impl_ide_read_8(7); 
        if (status & 0x01) return false; 
        if (status & 0x08) break;
        busy_wait_us_32(50); 
    }

    if (timeout == 0) return false;

    set_address(0); 
    data_mode_in();
    sio_hw->gpio_clr = (1 << IDE_CS0);
    volatile uint32_t junk = sio_hw->gpio_in; 
    (void)junk;
    busy_wait_at_least_cycles(200); 

    if (comp_timings) {
        for (int i = 0; i < 256; i++) {
            sio_hw->gpio_clr = (1 << IDE_DIOR);
            busy_wait_at_least_cycles(600); 
            buffer[i] = (uint16_t)(sio_hw->gpio_in & DATA_MASK);
            sio_hw->gpio_set = (1 << IDE_DIOR);
            busy_wait_at_least_cycles(4000); 
        }
    } else {
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

static bool impl_ide_set_geometry(uint8_t heads, uint8_t spt) {
    impl_ide_write_8(6, 0xA0 | ((heads - 1) & 0x0F)); 
    impl_ide_write_8(2, spt);
    impl_ide_write_8(7, 0x91); 
    return impl_ide_wait_until_ready(1000);
}

/* ================= WORKER THREAD (CORE 1) ================= */

void ide_worker_task(void) {
    // Ensure Core1 configures IDE GPIOs before handling any requests.
    ide_hw_init();

     while (1) {
         ide_req_t req;
         queue_remove_blocking(&ide_queue, &req);
 
         switch (req.type) {
             case CMD_READ_CACHE:
                 if (req.result_ptr) *req.result_ptr = impl_ide_read_cached(req.lba, req.count, req.buffer);
                 break;
             case CMD_READ_SECTORS:
                 if (req.result_ptr) *req.result_ptr = impl_ide_read_sectors_lba(req.lba, req.count, req.buffer);
                 break;
             case CMD_WRITE_SECTORS:
                 if (req.result_ptr) *req.result_ptr = impl_ide_write_sectors_lba(req.lba, req.count, req.c_buffer);
                 break;
             case CMD_IDENTIFY_SEND:
                 impl_ide_identify_drive();
                 break;
             case CMD_IDENTIFY_READ:
                 if (req.result_ptr) *req.result_ptr = impl_ide_get_identify_data(req.w_buffer) ? 1 : 0;
                 break;
             case CMD_RESET:
                 impl_ide_reset_drive();
                 break;
             case CMD_FLUSH_CACHE:
                 impl_ide_flush_cache();
                 break;
             case CMD_SET_GEO:
                 if (req.result_ptr) *req.result_ptr = impl_ide_set_geometry(req.heads, req.spt) ? 1 : 0;
                 /* On success, update core1's exported 'drive_*' values here (safe: we are running on CORE1) */
                 if (req.result_ptr && *req.result_ptr) {
                     drive_heads = req.heads;
                     drive_spt = req.spt;
                     drive_cylinders = cur_cyls;
                 }
                 break;
             case CMD_REG_READ:
                 if (req.result_ptr) *req.result_ptr = impl_ide_read_8(req.reg);
                 break;
             case CMD_REG_WRITE:
                 impl_ide_write_8(req.reg, req.val);
                 break;
             case CMD_GET_TASKFILE:
                 if (req.buffer) {
                     // Ensure byte 0 is always defined for callers; fill 1..7 from regs
                     req.buffer[0] = 0;
                     for (int i = 1; i <= 7; i++) req.buffer[i] = impl_ide_read_8(i);
                 }
                 break;
         }
 
         if (req.sem) sem_release(req.sem);
     }
 }

void ide_start_core1_worker(void) {
    /* If called on CORE1, do nothing — prevent CORE1 from trying to (re)launch the worker
       which would cause re-entry / undefined behavior. */
    if ((sio_hw->cpuid & 1u) == 1u) return;

    /* Make this function safe to call multiple times from different places.
       Use interrupt disable/restore to avoid race at startup. */
    uint32_t ints = save_and_disable_interrupts();
    if (core1_worker_started) {
        restore_interrupts(ints);
        return;
    }
    core1_worker_started = true;
    queue_init(&ide_queue, sizeof(ide_req_t), 4);
    restore_interrupts(ints);

    multicore_launch_core1(ide_worker_task);
}

/* ================= PUBLIC PROXY FUNCTIONS (CORE 0) ================= */

// Helper to construct request and block
static void submit_cmd(ide_cmd_type_t type, ide_req_t* req_data) {
    // If invoked on CORE1, execute the request inline to avoid deadlock/re-entry.
    if ((sio_hw->cpuid & 1u) == 1u) {
        ide_req_t req = *req_data;
        switch (type) {
            case CMD_READ_CACHE:
                if (req.result_ptr) *req.result_ptr = impl_ide_read_cached(req.lba, req.count, req.buffer);
                break;
            case CMD_READ_SECTORS:
                if (req.result_ptr) *req.result_ptr = impl_ide_read_sectors_lba(req.lba, req.count, req.buffer);
                break;
            case CMD_WRITE_SECTORS:
                if (req.result_ptr) *req.result_ptr = impl_ide_write_sectors_lba(req.lba, req.count, req.c_buffer);
                break;
            case CMD_IDENTIFY_SEND:
                impl_ide_identify_drive();
                break;
            case CMD_IDENTIFY_READ:
                if (req.result_ptr) *req.result_ptr = impl_ide_get_identify_data(req.w_buffer) ? 1 : 0;
                break;
            case CMD_RESET:
                impl_ide_reset_drive();
                break;
            case CMD_FLUSH_CACHE:
                impl_ide_flush_cache();
                break;
            case CMD_SET_GEO:
                if (req.result_ptr) *req.result_ptr = impl_ide_set_geometry(req.heads, req.spt) ? 1 : 0;
                /* On success, update core1's exported 'drive_*' values here (safe: we are running on CORE1) */
                if (req.result_ptr && *req.result_ptr) {
                    drive_heads = req.heads;
                    drive_spt = req.spt;
                    drive_cylinders = cur_cyls;
                }
                break;
            case CMD_REG_READ:
                if (req.result_ptr) *req.result_ptr = impl_ide_read_8(req.reg);
                break;
            case CMD_REG_WRITE:
                impl_ide_write_8(req.reg, req.val);
                break;
            case CMD_GET_TASKFILE:
                if (req.buffer) {
                    // Ensure byte 0 is always defined for callers; fill 1..7 from regs
                    req.buffer[0] = 0;
                    for (int i = 1; i <= 7; i++) req.buffer[i] = impl_ide_read_8(i);
                }
                break;
        }
        return;
    }

    // Normal proxy path for Core0
    semaphore_t sem;
    sem_init(&sem, 0, 0);

    ide_req_t req = *req_data;
    req.type = type;
    req.sem = &sem;

    // Try to enqueue without blocking Core0 (keeps USB responsive).
    // If queue is full, fail fast and report error to caller via result_ptr.
    if (!queue_try_add(&ide_queue, &req)) {
        if (req.result_ptr) *req.result_ptr = -1;
        return;
    }

    // Wait for completion. This will block Core0, but queue_try_add above
    // prevents indefinite blocking in the enqueue path; callers in USB callbacks
    // should avoid calling blocking APIs and should use ide_get_core1_state / ide_is_mounted.
    sem_acquire_blocking(&sem);
}

int32_t ide_read_cached(uint32_t lba, uint32_t count, uint8_t* buffer) {
    // If running on CORE1, call implementation directly to avoid IPC re-entry.
    if ((sio_hw->cpuid & 1u) == 1u) {
        return impl_ide_read_cached(lba, count, buffer);
    }

    int32_t result = 0;
    ide_req_t req = {};
    req.lba = lba;
    req.count = count;
    req.buffer = buffer;
    req.result_ptr = &result;
    submit_cmd(CMD_READ_CACHE, &req);
    return result;
}

int32_t ide_read_sectors_lba(uint32_t lba, uint32_t count, uint8_t* buffer) {
    // If running on CORE1, call the implementation directly to avoid IPC re-entry.
    if ((sio_hw->cpuid & 1u) == 1u) {
        return impl_ide_read_sectors_lba(lba, count, buffer);
    }

    int32_t result = 0;
    ide_req_t req = {};
    req.lba = lba;
    req.count = count;
    req.buffer = buffer;
    req.result_ptr = &result;
    submit_cmd(CMD_READ_SECTORS, &req);
    return result;
}

int32_t ide_write_sectors_lba(uint32_t lba, uint32_t count, const uint8_t* buffer) {
    // If running on CORE1, call implementation directly to avoid IPC re-entry.
    if ((sio_hw->cpuid & 1u) == 1u) {
        return impl_ide_write_sectors_lba(lba, count, buffer);
    }

    int32_t result = 0;
    ide_req_t req = {};
    req.lba = lba;
    req.count = count;
    req.c_buffer = buffer;
    req.result_ptr = &result;
    submit_cmd(CMD_WRITE_SECTORS, &req);
    return result;
}

void ide_identify_drive(void) {
    // If running on CORE1, call implementation directly to avoid IPC re-entry.
    if ((sio_hw->cpuid & 1u) == 1u) {
        impl_ide_identify_drive();
        return;
    }
    ide_req_t req = {};
    submit_cmd(CMD_IDENTIFY_SEND, &req);
}

bool ide_get_identify_data(uint16_t* buffer) {
    if (!buffer) return false;

    // If running on CORE1, call implementation directly to avoid IPC re-entry.
    if ((sio_hw->cpuid & 1u) == 1u) {
        return impl_ide_get_identify_data(buffer);
    }

    int32_t result = 0;
    ide_req_t req = {};
    req.w_buffer = buffer;
    req.result_ptr = &result;
    submit_cmd(CMD_IDENTIFY_READ, &req);
    return (result != 0);
}

void ide_reset_drive(void) {
    // If called on CORE1, run implementation directly to avoid queueing/re-entry
    if ((sio_hw->cpuid & 1u) == 1u) {
        impl_ide_reset_drive();
        return;
    }
    ide_req_t req = {};
    submit_cmd(CMD_RESET, &req);
}

void ide_flush_cache(void) {
    // If running on CORE1, call implementation directly to avoid IPC re-entry.
    if ((sio_hw->cpuid & 1u) == 1u) {
        impl_ide_flush_cache();
        return;
    }
    ide_req_t req = {};
    submit_cmd(CMD_FLUSH_CACHE, &req);
}

void get_large_geometry(uint16_t native_cyl, uint8_t native_head, uint8_t native_spt, 
                        uint16_t* l_cyl, uint8_t* l_head) {
    // This is a standard translation for "Large" mode (bit-shifting)
    // to keep geometry within 1024 cylinders for older BIOSes
    *l_head = native_head << 1; 
    *l_cyl = native_cyl >> 1;
}

bool ide_set_geometry(uint8_t heads, uint8_t spt) {
    // If running on CORE1, call implementation directly to avoid IPC re-entry.
    if ((sio_hw->cpuid & 1u) == 1u) {
        bool ok = impl_ide_set_geometry(heads, spt);
        if (ok) {
            // update core1-visible exports directly (we are on CORE1)
            drive_heads = heads;
            drive_spt = spt;
            drive_cylinders = cur_cyls;
        }
        return ok;
    }

    int32_t result = 0;
    ide_req_t req = {};
    req.heads = heads;
    req.spt = spt;
    req.result_ptr = &result;
    submit_cmd(CMD_SET_GEO, &req);
    return (result != 0);
}

uint8_t ide_read_8(uint8_t reg) {
    // If called from Core 1 (the worker), call implementation directly
    // to avoid deadlock where Core1 would queue a request that it must process.
    // Use SIO CPUID to detect core instead of multicore_get_core_num().
    if ((sio_hw->cpuid & 1u) == 1u) {
        return impl_ide_read_8(reg);
    }

    int32_t result = 0;
    ide_req_t req = {};
    req.reg = reg;
    req.result_ptr = &result;
    submit_cmd(CMD_REG_READ, &req);
    return (uint8_t)result;
}

void ide_write_8(uint8_t reg, uint8_t val) {
    // If called from Core 1, call implementation directly to avoid queueing/re-entry.
    if ((sio_hw->cpuid & 1u) == 1u) {
        impl_ide_write_8(reg, val);
        return;
    }
    ide_req_t req = {};
    req.reg = reg;
    req.val = val;
    submit_cmd(CMD_REG_WRITE, &req);
}

void ide_get_task_file(uint8_t* task_file) {
    if (!task_file) return;

    // If running on CORE1, read registers directly to avoid queueing/re-entry
    if ((sio_hw->cpuid & 1u) == 1u) {
        task_file[0] = 0;
        for (int i = 1; i <= 7; ++i) {
            task_file[i] = impl_ide_read_8(i);
        }
        return;
    }

    ide_req_t req = {};
    req.buffer = task_file;
    submit_cmd(CMD_GET_TASKFILE, &req);

    // Ensure index 0 is defined for callers on Core0
    task_file[0] = task_file[0]; // no-op but keeps intent clear (worker sets indices 1..7)
}

// Proxied Wait: Uses status register polling on Core 1
// Note: Core 0 blocks while Core 1 polls
bool ide_wait_until_ready(uint32_t timeout_ms) {
    // If executed on CORE1, use the Core‑1 implementation directly (no IPC)
    if ((sio_hw->cpuid & 1u) == 1u) {
        return impl_ide_wait_until_ready(timeout_ms);
    }

    uint32_t start = board_millis();
    while (board_millis() - start < timeout_ms) {
        uint8_t status = ide_read_8(IDE_REG_STATUS);
        if (!(status & 0x80)) return true;
        busy_wait_us(100);
    }
    return false;
}

// For compatibility with msc_disk.c/menus.c
uint8_t ide_read_register(uint8_t reg) {
    // Avoid IPC re-entry: if running on CORE1 call impl directly
    if ((sio_hw->cpuid & 1u) == 1u) return impl_ide_read_8(reg);
    return ide_read_8(reg);
}

void ide_write_register(uint8_t reg, uint8_t val) {
    // Avoid IPC re-entry: if running on CORE1 call impl directly
    if ((sio_hw->cpuid & 1u) == 1u) { impl_ide_write_8(reg, val); return; }
    ide_write_8(reg, val);
}

void ide_hw_init(void) {
    // Only configure IDE GPIOs on CORE1 to avoid disturbing USB (Core0).
    if ((sio_hw->cpuid & 1u) != 1u) return;

    uint32_t all_pins = DATA_MASK | ADDR_MASK | CTRL_MASK | 
                        (1 << IDE_RESET) | (1 << IDE_DIR) | 
                        (1 << IDE_DIR1) | (1 << IDE_OE) | (1 << IDE_OE1);

    // Avoid touching high-numbered pins that may be used by USB or board muxing.
    // Initialize only pins < 24 here to prevent disturbing USB pinmux (Windows Code 43).
    const uint32_t SAFE_PIN_LIMIT = (1u << 24) - 1u; // pins 0..23
    uint32_t safe_pins = all_pins & SAFE_PIN_LIMIT;
    
    gpio_init_mask(safe_pins);
    // Disable pulls only for pins we're about to touch
    for (int i = 0; i < 24; ++i) {
        if (safe_pins & (1u << i)) gpio_disable_pulls(i);
    }
    gpio_set_dir_out_masked(safe_pins & ~DATA_MASK);
    gpio_put(IDE_RESET, 1);
    bus_idle();
}

/* Single new helper: set core1 total LBA atomically from Core0 */
void ide_set_total_lba(uint64_t total) {
    // If called on CORE1, write directly to avoid lockout deadlock
    if ((sio_hw->cpuid & 1u) == 1u) {
        total_lba_sectors_from_identify = total;
        return;
    }
    multicore_lockout_start_blocking();
    total_lba_sectors_from_identify = total;
    multicore_lockout_end_blocking();
}

/* Atomically update core1 geometry from Core0 */
void ide_set_core_geometry(uint16_t cyls, uint8_t heads, uint8_t spt) {
    // If called on CORE1, write directly to avoid lockout deadlock
    if ((sio_hw->cpuid & 1u) == 1u) {
        cur_cyls = cyls;
        cur_heads = heads;
        cur_spt  = spt;
        return;
    }
    multicore_lockout_start_blocking();
    cur_cyls = cyls;
    cur_heads = heads;
    cur_spt  = spt;
    multicore_lockout_end_blocking();
}

/* Single new helper: clear core1 geometry & total LBA atomically from Core0 */
void ide_clear_geometry(void) {
    // If called on CORE1, update directly to avoid lockout deadlock
    if ((sio_hw->cpuid & 1u) == 1u) {
        total_lba_sectors_from_identify = 0;
        cur_cyls = 0;
        cur_heads = 0;
        cur_spt = 0;
        drive_cylinders = 0;
        drive_heads = 0;
        drive_spt = 0;
        use_lba_mode = false;
        return;
    }

    multicore_lockout_start_blocking();
    total_lba_sectors_from_identify = 0;
    cur_cyls = 0;
    cur_heads = 0;
    cur_spt = 0;
    drive_cylinders = 0;
    drive_heads = 0;
    drive_spt = 0;
    use_lba_mode = false;
    multicore_lockout_end_blocking();
}

/* Atomically export drive geometry from Core0; if called on CORE1 write directly */
void ide_export_drive_geometry(uint16_t cyls, uint8_t heads, uint8_t spt) {
    if ((sio_hw->cpuid & 1u) == 1u) {
        drive_cylinders = cyls;
        drive_heads = heads;
        drive_spt = spt;
        return;
    }
    multicore_lockout_start_blocking();
    drive_cylinders = cyls;
    drive_heads = heads;
    drive_spt = spt;
    multicore_lockout_end_blocking();
}

/* Single new function: take a consistent snapshot of core1 state for core0 callers */
bool ide_get_core1_state(ide_core1_state_t *out) {
    if (!out) return false;

    // If called on CORE1, copy locals directly (no lockout).
    if ((sio_hw->cpuid & 1u) == 1u) {
        out->total_lba = total_lba_sectors_from_identify;
        out->cyls = cur_cyls;
        out->heads = cur_heads;
        out->spt = cur_spt;
        out->drive_write_protected = drive_write_protected;
        out->drive_supports_lba48 = drive_supports_lba48;
        out->use_lba_mode = use_lba_mode;
        return true;
    }

    /* Pause core1 to get a consistent snapshot */
    multicore_lockout_start_blocking();
    out->total_lba = total_lba_sectors_from_identify;
    out->cyls = cur_cyls;
    out->heads = cur_heads;
    out->spt = cur_spt;
    out->drive_write_protected = drive_write_protected;
    out->drive_supports_lba48 = drive_supports_lba48;
    out->use_lba_mode = use_lba_mode;
    multicore_lockout_end_blocking();
    return true;
}

/* Atomically set LBA mode from Core0; if called on Core1 write directly */
void ide_set_lba_mode(bool enable) {
    if ((sio_hw->cpuid & 1u) == 1u) {
        use_lba_mode = enable;
        return;
    }
    multicore_lockout_start_blocking();
    use_lba_mode = enable;
    multicore_lockout_end_blocking();
}

/* New: atomically set drive write-protect (safe for Core0 callers) */
void ide_set_drive_write_protected(bool wp) {
    // If called on CORE1, write directly to avoid lockout deadlock
    if ((sio_hw->cpuid & 1u) == 1u) {
        drive_write_protected = wp;
        return;
    }
    multicore_lockout_start_blocking();
    drive_write_protected = wp;
    multicore_lockout_end_blocking();
}

void ide_set_mounted(bool mounted) {
    // If called on CORE1, write directly to avoid lockout deadlock
    if ((sio_hw->cpuid & 1u) == 1u) {
        is_mounted = mounted;
        return;
    }
    multicore_lockout_start_blocking();
    is_mounted = mounted;
    multicore_lockout_end_blocking();
}

bool ide_is_mounted(void) {
    // Non-blocking atomic read to avoid stalling Core0 (USB/TinyUSB) when Core1 is busy.
    // Use an atomic load to get a consistent value without pausing core1.
    if ((sio_hw->cpuid & 1u) == 1u) return is_mounted;
    return (bool)__atomic_load_n(&is_mounted, __ATOMIC_ACQUIRE);
}

/* New: read comp_timings atomically (safe for Core0 callers) */
bool ide_get_core_timings(void) {
    // If running on CORE1, return directly
    if ((sio_hw->cpuid & 1u) == 1u) return comp_timings;

    bool v;
    multicore_lockout_start_blocking();
    v = comp_timings;
    multicore_lockout_end_blocking();
    return v;
}

void ide_set_core_timings(bool enable) {
    /* If running on CORE1, just set it directly to avoid deadlock */
    if ((sio_hw->cpuid & 1u) == 1u) {
        comp_timings = enable;
        return;
    }
    /* From Core0, use lockout to update core1-visible variable atomically */
    multicore_lockout_start_blocking();
    comp_timings = enable;
    multicore_lockout_end_blocking();
}

/* media-changed flag owned by this module */
static volatile bool media_changed_flag = false;

/* Called by UI (Core0) to notify a media change; sets flag atomically. */
void ide_notify_media_changed(void) {
    if ((sio_hw->cpuid & 1u) == 1u) {
        media_changed_flag = true;
        return;
    }
    multicore_lockout_start_blocking();
    media_changed_flag = true;
    multicore_lockout_end_blocking();
}

/* Consume the media-changed flag: return previous value and clear it atomically. */
bool ide_consume_media_changed(void) {
    if ((sio_hw->cpuid & 1u) == 1u) {
        bool v = media_changed_flag;
        media_changed_flag = false;
        return v;
    }
    bool v;
    multicore_lockout_start_blocking();
    v = media_changed_flag;
    media_changed_flag = false;
    multicore_lockout_end_blocking();
    return v;
}

} // End extern "C"