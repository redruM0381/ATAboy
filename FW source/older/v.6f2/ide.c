#include "ide.h"
#include "ide_pio.h"
#include "config.h"
#include "hardware/gpio.h"
#include "hardware/structs/sio.h"
#include "pico/stdlib.h"
#include "pico/time.h"

static uint8_t dev_base = 0xA0;   // 0xA0 = master, 0xB0 = slave

void ide_select_device(uint8_t base) { dev_base = base; }

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
    // Force IORDY HIGH during reset — drive holds it LOW during POST
    ide_set_iordy(false);

    gpio_put(IDE_RESET, 0);
    sleep_ms(50);
    gpio_put(IDE_RESET, 1);
    sleep_ms(100);
    ide_write_control(0x00);
    ide_wait_until_ready(10000);

    // Recalibrate — seek heads to track 0 (required by some pre-ATA drives)
    ide_write_reg(6, dev_base);
    ide_write_reg(7, 0x10);
    ide_wait_until_ready(10000);

    // Restore IORDY to config setting — drive is ready for normal operation
    ide_set_iordy(config.iordy_enabled);
}

uint8_t ide_probe_devices(void) {
    ide_set_iordy(false);

    // Single hardware reset — both devices see it
    gpio_put(IDE_RESET, 0);
    sleep_ms(50);
    gpio_put(IDE_RESET, 1);
    sleep_ms(2000);           // generous POST delay (not status-based)
    ide_write_control(0x00);  // nIEN=0

    static const uint8_t addrs[] = {0xA0, 0xB0};
    for (int i = 0; i < 2; i++) {
        ide_write_reg(6, addrs[i]);       // select device
        busy_wait_us_32(50);              // let selection settle

        // Floating bus filter: 0xFF means no device (bus floats high)
        uint8_t st = ide_read_reg(7);
        if (st == 0xFF) continue;

        // Device present — wait for BSY to clear (not DRDY; some older
        // drives won't assert DRDY until after INITIALIZE DRIVE PARAMETERS)
        dev_base = addrs[i];
        uint32_t start = to_ms_since_boot(get_absolute_time());
        bool bsy_clear = false;
        while (to_ms_since_boot(get_absolute_time()) - start < 10000) {
            st = ide_read_reg(7);
            if (!(st & 0x80)) { bsy_clear = true; break; }
            busy_wait_us_32(10);
        }
        if (!bsy_clear) continue;

        // Recalibrate (required by some pre-ATA drives)
        ide_write_reg(6, addrs[i]);
        ide_write_reg(7, 0x10);
        ide_wait_until_ready(10000);

        ide_set_iordy(config.iordy_enabled);
        return addrs[i];
    }

    ide_set_iordy(config.iordy_enabled);
    return 0;  // no device found
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
    ide_write_reg(6, dev_base | ((heads - 1) & 0x0F));
    ide_write_reg(2, spt);
    ide_write_reg(7, 0x91);
    return ide_wait_until_ready(1000);
}

// ---------------------------------------------------------------------------
//  IDENTIFY DEVICE (0xEC)
// ---------------------------------------------------------------------------

bool ide_identify(uint16_t *buf) {
    if (!ide_wait_until_ready(1000)) return false;
    if (ide_read_reg(7) & 0x08) ide_drain_sector();   // drain stranded DRQ before command
    ide_write_reg(6, dev_base);
    ide_write_reg(7, 0xEC);
    busy_wait_us_32(1);             // give drive time to assert BSY

    // Poll for DRQ — INTRQ provides early-exit if enabled, otherwise pure polling
    for (uint32_t t = 0; t < 100000; t++) {
        if (config.intrq_enabled && gpio_get(IDE_INTRQ)) ide_read_reg(7);  // clear INTRQ
        uint8_t st = ide_read_reg(7);
        if (st & 0x01) { if (st & 0x08) ide_drain_sector(); return false; }  // ERR — drain stranded DRQ
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
        ide_write_reg(6, (dev_base | 0x40) | ((lba >> 24) & 0x0F));
    } else {
        uint32_t tmp  = lba / config.spt;
        uint8_t  sec  = (lba % config.spt) + 1;           // 1-based
        uint8_t  head = tmp % config.heads;
        uint16_t cyl  = tmp / config.heads;
        ide_write_reg(3, sec);
        ide_write_reg(4, cyl & 0xFF);
        ide_write_reg(5, (cyl >> 8) & 0xFF);
        ide_write_reg(6, dev_base | (head & 0x0F));
    }

    ide_write_reg(7, 0x20);                                // READ SECTORS

    uint16_t *wbuf = (uint16_t *)buf;

    for (uint32_t s = 0; s < count; s++) {
        // Poll for DRQ — INTRQ provides early-exit if enabled, otherwise pure polling
        for (uint32_t t = 0; t < 100000; t++) {
            if (config.intrq_enabled && gpio_get(IDE_INTRQ)) ide_read_reg(7);  // clear INTRQ
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
    // Soft-reset to abort any stuck command (drive may be retrying internally)
    ide_write_control(0x04);
    busy_wait_us_32(10);
    ide_write_control(0x00);
    ide_wait_until_ready(2000);
    // SRST clears INITIALIZE DRIVE PARAMETERS — restore CHS geometry
    if (!config.use_lba_mode)
        ide_set_geometry(config.heads, config.spt);
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
        ide_write_reg(6, (dev_base | 0x40) | ((lba >> 24) & 0x0F));
    } else {
        uint32_t tmp  = lba / config.spt;
        uint8_t  sec  = (lba % config.spt) + 1;
        uint8_t  head = tmp % config.heads;
        uint16_t cyl  = tmp / config.heads;
        ide_write_reg(3, sec);
        ide_write_reg(4, cyl & 0xFF);
        ide_write_reg(5, (cyl >> 8) & 0xFF);
        ide_write_reg(6, dev_base | (head & 0x0F));
    }

    ide_write_reg(7, 0x30);                                // WRITE SECTORS

    const uint16_t *wbuf = (const uint16_t *)buf;

    bool write_ok = true;

    for (uint32_t s = 0; s < count; s++) {
        // Poll for DRQ — INTRQ not asserted for first sector of PIO write per ATA spec;
        // for s > 0 it provides early-exit if enabled, otherwise pure polling
        bool got_drq = false;
        for (uint32_t t = 0; t < 100000; t++) {
            if (s > 0 && config.intrq_enabled && gpio_get(IDE_INTRQ)) ide_read_reg(7);
            uint8_t st = ide_read_reg(7);
            if (st & 0x01) { write_ok = false; break; }
            if (!(st & 0x80) && (st & 0x08)) { got_drq = true; break; }
            busy_wait_us_32(10);
        }
        if (!write_ok || !got_drq) { write_ok = false; break; }

        set_address(0);
        xcvr_write();
        sio_hw->gpio_clr = (1 << IDE_CS0);

        ide_pio_write(256, wbuf + s * 256);

        sio_hw->gpio_set = (1 << IDE_CS0);
        bus_idle();
    }

    // Wait for drive to commit the last sector to media (BSY=0)
    if (write_ok) {
        for (uint32_t t = 0; t < 100000; t++) {
            if (config.intrq_enabled && gpio_get(IDE_INTRQ)) ide_read_reg(7);
            uint8_t st = ide_read_reg(7);
            if (st & 0x01) { write_ok = false; break; }
            if (!(st & 0x80)) return (int32_t)(count * 512);
            busy_wait_us_32(10);
        }
    }

    // Soft-reset to abort any stuck command (drive may be retrying internally)
    ide_write_control(0x04);
    busy_wait_us_32(10);
    ide_write_control(0x00);
    ide_wait_until_ready(2000);
    // SRST clears INITIALIZE DRIVE PARAMETERS — restore CHS geometry
    if (!config.use_lba_mode)
        ide_set_geometry(config.heads, config.spt);
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
        ide_write_reg(6, (dev_base | 0x40) | ((target >> 24) & 0x0F));
    } else {
        uint16_t cyl = (uint16_t)target;
        ide_write_reg(3, 1);                                   // sector 1 (1-based)
        ide_write_reg(4, cyl & 0xFF);
        ide_write_reg(5, (cyl >> 8) & 0xFF);
        ide_write_reg(6, dev_base);                            // head 0, CHS
    }
    ide_write_reg(2, 1);
    ide_write_reg(7, 0x20);                                    // READ SECTORS

    // Wait for BSY to clear
    for (uint32_t t = 0; t < 10000; t++) {
        if (config.intrq_enabled && gpio_get(IDE_INTRQ)) ide_read_reg(7);  // clear INTRQ
        if (!(ide_read_reg(7) & 0x80)) break;
        busy_wait_us_32(10);
    }

    // Drain DRQ data if present
    if (ide_read_reg(7) & 0x08) ide_drain_sector();

    return ide_read_reg(7);
}
