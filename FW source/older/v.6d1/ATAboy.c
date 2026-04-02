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
//  main — core 0 (USB only — no IDE operations here)
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

    // Launch menu UI on core 1 (auto-mount runs there too)
    multicore_launch_core1(core1_entry);

    // Core 0 loop: service USB + CDC bridge
    while (true) {
        tud_task();
        cdc_task();
    }
}
