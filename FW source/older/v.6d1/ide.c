#include "ide.h"
#include "ide_pio.h"
#include "config.h"
#include "hardware/gpio.h"
#include "hardware/structs/sio.h"
#include "pico/stdlib.h"
#include "pico/time.h"

// ---------------------------------------------------------------------------
//  Bus helpers — address, transceiver, and chip-select (all SIO-managed)
// ---------------------------------------------------------------------------

static void bus_idle(void) {
    sio_hw->gpio_set = (1 << IDE_CS0) | (1 << IDE_CS1);
    sio_hw->gpio_clr = (1 << IDE_DIR) | (1 << IDE_DIR1);   // DIR = read
    sio_hw->gpio_set = (1 << IDE_OE) | (1 << IDE_OE1);     // OE disabled
}

static void set_address(uint8_t addr) {
    sio_hw->gpio_clr = ADDR_MASK;
    sio_hw->gpio_set = (uint32_t)(addr & 0x07) << IDE_A0;
}

static void xcvr_read(void) {
    sio_hw->gpio_set = (1 << IDE_OE) | (1 << IDE_OE1);     // disable first
    sio_hw->gpio_clr = (1 << IDE_DIR) | (1 << IDE_DIR1);   // DIR = IDE -> Pico
    busy_wait_at_least_cycles(20);                            // 74HCT245 switch time
    sio_hw->gpio_clr = (1 << IDE_OE) | (1 << IDE_OE1);     // enable
}

static void xcvr_write(void) {
    sio_hw->gpio_set = (1 << IDE_OE) | (1 << IDE_OE1);     // disable first
    sio_hw->gpio_set = (1 << IDE_DIR) | (1 << IDE_DIR1);   // DIR = Pico -> IDE
    busy_wait_at_least_cycles(20);
    sio_hw->gpio_clr = (1 << IDE_OE) | (1 << IDE_OE1);     // enable
}

// ---------------------------------------------------------------------------
//  Register I/O — single 8-bit reads/writes via PIO strobes
// ---------------------------------------------------------------------------

void ide_write_reg(uint8_t reg, uint8_t val) {
    set_address(reg);
    xcvr_write();
    sio_hw->gpio_clr = (1 << IDE_CS0);

    uint16_t word = (uint16_t)val;
    ide_pio_write(1, &word);

    sio_hw->gpio_set = (1 << IDE_CS0);
    bus_idle();
}

uint8_t ide_read_reg(uint8_t reg) {
    set_address(reg);
    xcvr_read();
    sio_hw->gpio_clr = (1 << IDE_CS0);

    uint16_t val;
    ide_pio_read(1, &val);

    sio_hw->gpio_set = (1 << IDE_CS0);
    bus_idle();
    return (uint8_t)(val & 0xFF);
}

uint8_t ide_read_alt_status(void) {
    set_address(6);
    xcvr_read();
    sio_hw->gpio_clr = (1 << IDE_CS1);

    uint16_t val;
    ide_pio_read(1, &val);

    sio_hw->gpio_set = (1 << IDE_CS1);
    bus_idle();
    return (uint8_t)(val & 0xFF);
}

void ide_write_control(uint8_t val) {
    set_address(6);
    xcvr_write();
    sio_hw->gpio_clr = (1 << IDE_CS1);

    uint16_t word = (uint16_t)val;
    ide_pio_write(1, &word);

    sio_hw->gpio_set = (1 << IDE_CS1);
    bus_idle();
}

void ide_set_iordy(bool enabled) {
    gpio_set_inover(IDE_IORDY, enabled ? GPIO_OVERRIDE_NORMAL : GPIO_OVERRIDE_HIGH);
}

// ---------------------------------------------------------------------------
//  INTRQ helper — wait for drive interrupt on GPIO pin
// ---------------------------------------------------------------------------

static bool ide_wait_intrq(uint32_t timeout_ms) {
    if (!config.intrq_enabled) return false;
    uint32_t start = to_ms_since_boot(get_absolute_time());
    while (to_ms_since_boot(get_absolute_time()) - start < timeout_ms) {
        if (gpio_get(IDE_INTRQ)) return true;
        busy_wait_us_32(1);
    }
    return false;
}

// ---------------------------------------------------------------------------
//  Init / Reset / Polling
// ---------------------------------------------------------------------------

void ide_hw_init(void) {
    // SIO-controlled output pins
    uint32_t sio_out = (1 << IDE_DIR) | (1 << IDE_DIR1) |
                       (1 << IDE_OE)  | (1 << IDE_OE1)  |
                       ADDR_MASK |
                       (1 << IDE_RESET) |
                       (1 << IDE_CS0) | (1 << IDE_CS1);

    gpio_init_mask(sio_out);
    gpio_set_dir_out_masked(sio_out);

    // INTRQ — active-high from drive, idle low
    gpio_init(IDE_INTRQ);
    gpio_set_dir(IDE_INTRQ, GPIO_IN);
    gpio_pull_down(IDE_INTRQ);

    // IORDY — active-high when ready, pulled up as fallback
    gpio_init(IDE_IORDY);
    gpio_set_dir(IDE_IORDY, GPIO_IN);
    gpio_pull_up(IDE_IORDY);
    // Force IORDY HIGH so PIO wait instructions never block during init
    gpio_set_inover(IDE_IORDY, GPIO_OVERRIDE_HIGH);

    // Data bus pads: no pulls (PIO owns these pins after ide_pio_init)
    for (int i = 0; i < 16; i++) gpio_disable_pulls(i);

    // Bring up PIO state machines
    ide_pio_init();

    gpio_put(IDE_RESET, 1);
    bus_idle();

    // nIEN=0: allow INTRQ from the drive
    ide_write_control(0x00);

    // Restore IORDY based on config (override was HIGH for safe init)
    ide_set_iordy(config.iordy_enabled);
}

void ide_reset_drive(void) {
    gpio_put(IDE_RESET, 0);
    sleep_ms(50);
    gpio_put(IDE_RESET, 1);
    sleep_ms(100);
    ide_write_control(0x00);
}

bool ide_wait_until_ready(uint32_t timeout_ms) {
    uint32_t start = to_ms_since_boot(get_absolute_time());
    while (to_ms_since_boot(get_absolute_time()) - start < timeout_ms) {
        uint8_t st = ide_read_reg(7);
        if (!(st & 0x80) && (st & 0x40)) return true;   // BSY=0, DRDY=1
        busy_wait_us_32(10);
    }
    return false;
}

bool ide_set_geometry(uint8_t heads, uint8_t spt) {
    ide_write_reg(6, 0xA0 | ((heads - 1) & 0x0F));
    ide_write_reg(2, spt);
    ide_write_reg(7, 0x91);
    return ide_wait_until_ready(1000);
}

// ---------------------------------------------------------------------------
//  IDENTIFY DEVICE (0xEC)
// ---------------------------------------------------------------------------

bool ide_identify(uint16_t *buf) {
    if (!ide_wait_until_ready(1000)) return false;
    ide_write_reg(6, 0xA0);
    ide_write_reg(7, 0xEC);

    // Wait for DRQ — try INTRQ first, then fall back to polling
    if (ide_wait_intrq(5000)) {
        uint8_t st = ide_read_reg(7);
        if (st & 0x01) return false;
        if (!(st & 0x80) && (st & 0x08)) goto ready;
    }
    for (uint32_t t = 0; t < 100000; t++) {
        uint8_t st = ide_read_reg(7);
        if (st & 0x01) return false;                      // ERR
        if (!(st & 0x80) && (st & 0x08)) goto ready;     // BSY=0, DRQ=1
        busy_wait_us_32(50);
    }
    return false;

ready:
    // Burst-read 256 words through PIO
    set_address(0);
    xcvr_read();
    sio_hw->gpio_clr = (1 << IDE_CS0);

    ide_pio_read(256, buf);

    sio_hw->gpio_set = (1 << IDE_CS0);
    bus_idle();
    return true;
}

// ---------------------------------------------------------------------------
//  Drain one sector of DRQ data (discard 256 words)
// ---------------------------------------------------------------------------

void ide_drain_sector(void) {
    uint16_t discard[256];
    set_address(0);
    xcvr_read();
    sio_hw->gpio_clr = (1 << IDE_CS0);
    ide_pio_read(256, discard);
    sio_hw->gpio_set = (1 << IDE_CS0);
    bus_idle();
}

// ---------------------------------------------------------------------------
//  Sector I/O — LBA28, single-sector loop
// ---------------------------------------------------------------------------

int32_t ide_read_sectors(uint32_t lba, uint32_t count, uint8_t *buf) {
    if (count == 0) return -1;
    if (!ide_wait_until_ready(5000)) return -1;

    ide_write_reg(2, (uint8_t)count);

    if (config.use_lba_mode) {
        ide_write_reg(3, lba & 0xFF);
        ide_write_reg(4, (lba >> 8) & 0xFF);
        ide_write_reg(5, (lba >> 16) & 0xFF);
        ide_write_reg(6, 0xE0 | ((lba >> 24) & 0x0F));
    } else {
        uint32_t tmp  = lba / config.spt;
        uint8_t  sec  = (lba % config.spt) + 1;           // 1-based
        uint8_t  head = tmp % config.heads;
        uint16_t cyl  = tmp / config.heads;
        ide_write_reg(3, sec);
        ide_write_reg(4, cyl & 0xFF);
        ide_write_reg(5, (cyl >> 8) & 0xFF);
        ide_write_reg(6, 0xA0 | (head & 0x0F));
    }

    ide_write_reg(7, 0x20);                                // READ SECTORS

    uint16_t *wbuf = (uint16_t *)buf;

    for (uint32_t s = 0; s < count; s++) {
        // Wait for DRQ — try INTRQ first, then fall back to polling
        if (ide_wait_intrq(10000)) {
            uint8_t st = ide_read_reg(7);
            if (st & 0x01) goto read_err;
            if (!(st & 0x80) && (st & 0x08)) goto drq_read;
        }
        for (uint32_t t = 0; t < 1000000; t++) {
            uint8_t st = ide_read_reg(7);
            if (st & 0x01) goto read_err;
            if (!(st & 0x80) && (st & 0x08)) goto drq_read;
            busy_wait_us_32(10);
        }
        goto read_err;

    drq_read:
        set_address(0);
        xcvr_read();
        sio_hw->gpio_clr = (1 << IDE_CS0);

        ide_pio_read(256, wbuf + s * 256);

        sio_hw->gpio_set = (1 << IDE_CS0);
        bus_idle();
    }

    return (int32_t)(count * 512);

read_err:
    // Drain any pending DRQ data so the drive doesn't block the next command
    if (ide_read_alt_status() & 0x08) ide_drain_sector();
    return -1;
}

int32_t ide_write_sectors(uint32_t lba, uint32_t count, const uint8_t *buf) {
    if (count == 0) return -1;
    if (!ide_wait_until_ready(5000)) return -1;

    ide_write_reg(2, (uint8_t)count);

    if (config.use_lba_mode) {
        ide_write_reg(3, lba & 0xFF);
        ide_write_reg(4, (lba >> 8) & 0xFF);
        ide_write_reg(5, (lba >> 16) & 0xFF);
        ide_write_reg(6, 0xE0 | ((lba >> 24) & 0x0F));
    } else {
        uint32_t tmp  = lba / config.spt;
        uint8_t  sec  = (lba % config.spt) + 1;
        uint8_t  head = tmp % config.heads;
        uint16_t cyl  = tmp / config.heads;
        ide_write_reg(3, sec);
        ide_write_reg(4, cyl & 0xFF);
        ide_write_reg(5, (cyl >> 8) & 0xFF);
        ide_write_reg(6, 0xA0 | (head & 0x0F));
    }

    ide_write_reg(7, 0x30);                                // WRITE SECTORS

    const uint16_t *wbuf = (const uint16_t *)buf;

    for (uint32_t s = 0; s < count; s++) {
        // INTRQ is NOT asserted for the first DRQ of a PIO write command;
        // only use it for subsequent sectors (s > 0)
        if (s > 0 && ide_wait_intrq(10000)) {
            uint8_t st = ide_read_reg(7);
            if (st & 0x01) return -1;
            if (!(st & 0x80) && (st & 0x08)) goto drq_write;
        }
        for (uint32_t t = 0; t < 1000000; t++) {
            uint8_t st = ide_read_reg(7);
            if (st & 0x01) return -1;
            if (!(st & 0x80) && (st & 0x08)) goto drq_write;
            busy_wait_us_32(10);
        }
        return -1;

    drq_write:
        set_address(0);
        xcvr_write();
        sio_hw->gpio_clr = (1 << IDE_CS0);

        ide_pio_write(256, wbuf + s * 256);

        sio_hw->gpio_set = (1 << IDE_CS0);
        bus_idle();
    }

    // Wait for drive to commit the last sector to media (BSY=0)
    if (ide_wait_intrq(10000)) {
        uint8_t st = ide_read_reg(7);
        if (st & 0x01) return -1;
        if (!(st & 0x80)) return (int32_t)(count * 512);
    }
    for (uint32_t t = 0; t < 1000000; t++) {
        uint8_t st = ide_read_reg(7);
        if (st & 0x01) return -1;
        if (!(st & 0x80)) return (int32_t)(count * 512);
        busy_wait_us_32(10);
    }
    return -1;
}

// ---------------------------------------------------------------------------
//  Diagnostics — task file snapshot and seek/read-one
// ---------------------------------------------------------------------------

void ide_read_taskfile(uint8_t tf[8]) {
    for (int i = 1; i <= 7; i++) tf[i] = ide_read_reg(i);
}

uint8_t ide_seek_read_one(uint32_t target, bool lba) {
    if (lba) {
        ide_write_reg(3, target & 0xFF);
        ide_write_reg(4, (target >> 8) & 0xFF);
        ide_write_reg(5, (target >> 16) & 0xFF);
        ide_write_reg(6, 0xE0 | ((target >> 24) & 0x0F));
    } else {
        uint16_t cyl = (uint16_t)target;
        ide_write_reg(3, 1);                                   // sector 1 (1-based)
        ide_write_reg(4, cyl & 0xFF);
        ide_write_reg(5, (cyl >> 8) & 0xFF);
        ide_write_reg(6, 0xA0);                                // head 0, CHS
    }
    ide_write_reg(2, 1);
    ide_write_reg(7, 0x20);                                    // READ SECTORS

    // Wait for BSY to clear
    if (!ide_wait_intrq(1000)) {
        for (uint32_t t = 0; t < 10000; t++) {
            if (!(ide_read_reg(7) & 0x80)) break;
            busy_wait_us_32(10);
        }
    } else {
        ide_read_reg(7);                                       // clear INTRQ
    }

    // Drain DRQ data if present
    if (ide_read_reg(7) & 0x08) ide_drain_sector();

    return ide_read_reg(7);
}
