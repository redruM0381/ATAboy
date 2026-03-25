// ATAboy Firmware — Main Entry Point
// Core 0: TinyUSB task loop + CDC queue bridge
// Core 1: Menu system (menus.c)
// NEVER call tud_task() from callbacks, IDE I/O, or core 1.

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "tusb.h"
#include "class/cdc/cdc_device.h"
#include "ide.h"
#include "config.h"
#include "pico/util/queue.h"
#include <string.h>

// ---------------------------------------------------------------------------
//  Shared state (accessed by usb.c callbacks and menus.c)
// ---------------------------------------------------------------------------

volatile bool is_mounted = false;
volatile bool media_changed_waiting = false;
volatile bool cdc_connected = false;

// ---------------------------------------------------------------------------
//  CDC queues — core 1 writes TX, reads RX; core 0 drains TX, fills RX
// ---------------------------------------------------------------------------

queue_t cdc_tx_queue;
queue_t cdc_rx_queue;

#define CDC_TX_QUEUE_SIZE 4096
#define CDC_RX_QUEUE_SIZE 256

// ---------------------------------------------------------------------------
//  cdc_task() — called from core 0 main loop only
// ---------------------------------------------------------------------------

static void cdc_task(void) {
    cdc_connected = tud_cdc_connected();

    // Drain TX queue -> USB CDC
    if (tud_cdc_connected()) {
        uint32_t avail = tud_cdc_write_available();
        while (avail > 0) {
            char c;
            if (!queue_try_remove(&cdc_tx_queue, &c)) break;
            tud_cdc_write_char(c);
            avail--;
        }
        tud_cdc_write_flush();
    }

    // Fill RX queue <- USB CDC
    while (tud_cdc_available()) {
        int c = tud_cdc_read_char();
        if (c >= 0) {
            char ch = (char)c;
            queue_try_add(&cdc_rx_queue, &ch);  // drop if full
        }
    }
}

// ---------------------------------------------------------------------------
//  Core 1 entry (defined in menus.c)
// ---------------------------------------------------------------------------

extern void core1_entry(void);

// ---------------------------------------------------------------------------
//  Auto-mount logic
// ---------------------------------------------------------------------------

static void try_auto_mount(void) {
    if (!config.auto_mount) return;

    bool has_geo = (config.use_lba_mode && config.lba_sectors > 0) ||
                   (!config.use_lba_mode && config.cyls > 0 && config.heads > 0 && config.spt > 0);
    if (!has_geo) return;

    // Wait for drive to spin up while servicing USB enumeration
    for (int i = 0; i < 50; i++) {
        tud_task();
        sleep_ms(100);
    }

    ide_reset_drive();

    // Poll drive ready while keeping USB alive
    uint32_t start = to_ms_since_boot(get_absolute_time());
    bool ready = false;
    while (to_ms_since_boot(get_absolute_time()) - start < 5000) {
        tud_task();
        uint8_t st = ide_read_reg(7);
        if (!(st & 0x80) && (st & 0x40)) { ready = true; break; }
        sleep_ms(10);
    }
    if (!ready) return;

    uint16_t id_buf[256];
    if (!ide_identify(id_buf)) return;

    // Set geometry on the drive if CHS mode
    if (!config.use_lba_mode)
        ide_set_geometry(config.heads, config.spt);

    is_mounted = true;
    media_changed_waiting = true;
}

// ---------------------------------------------------------------------------
//  main — core 0
// ---------------------------------------------------------------------------

int main(void) {
    // No pico_stdio — all console I/O goes through TinyUSB CDC via queues
    config_load();
    ide_hw_init();

    // Init CDC queues
    queue_init(&cdc_tx_queue, sizeof(char), CDC_TX_QUEUE_SIZE);
    queue_init(&cdc_rx_queue, sizeof(char), CDC_RX_QUEUE_SIZE);

    // Init TinyUSB
    tusb_init();

    // Allow core 1 to be paused for flash writes (config_save)
    multicore_lockout_victim_init();

    // Auto-mount if configured
    try_auto_mount();

    // Launch menu UI on core 1
    multicore_launch_core1(core1_entry);

    // Core 0 loop: service USB + CDC bridge
    while (true) {
        tud_task();
        cdc_task();
    }
}
