#include "ide_pio.h"
#include "ide.h"
#include "ataboy.pio.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "pico/stdlib.h"

static PIO pio = pio0;
static uint sm_read;
static uint sm_write;
static uint offset_read;
static uint offset_write;

void ide_pio_init(void) {
    // Load both programs into PIO instruction memory (17 of 32 slots)
    offset_read  = pio_add_program(pio, &ide_read_program);
    offset_write = pio_add_program(pio, &ide_write_program);

    sm_read  = pio_claim_unused_sm(pio, true);
    sm_write = pio_claim_unused_sm(pio, true);

    // Hand GPIO 0-15 (data bus), 26 (DIOR), 27 (DIOW) to PIO
    for (int i = 0; i < 16; i++) pio_gpio_init(pio, i);
    pio_gpio_init(pio, 26);
    pio_gpio_init(pio, 27);

    // ---- Read SM (DIOR strobe) ----
    pio_sm_config c_rd = ide_read_program_get_default_config(offset_read);
    sm_config_set_in_pins(&c_rd, 0);                       // in base = GPIO 0
    sm_config_set_sideset_pins(&c_rd, 26);                  // side-set = DIOR
    sm_config_set_in_shift(&c_rd, false, true, 16);         // shift left, autopush @ 16
    sm_config_set_clkdiv(&c_rd, 15.0f);                    // 150 MHz / 15 = 10 MHz

    pio_sm_init(pio, sm_read, offset_read, &c_rd);

    // Set DIOR HIGH and output before enabling SM (avoids LOW glitch)
    pio_sm_set_pins_with_mask(pio, sm_read, 1u << 26, 1u << 26);
    pio_sm_set_consecutive_pindirs(pio, sm_read, 26, 1, true);

    // ---- Write SM (DIOW strobe) ----
    pio_sm_config c_wr = ide_write_program_get_default_config(offset_write);
    sm_config_set_out_pins(&c_wr, 0, 16);                  // out base = GPIO 0, 16 pins
    sm_config_set_sideset_pins(&c_wr, 27);                  // side-set = DIOW
    sm_config_set_out_shift(&c_wr, true, false, 32);        // shift right, manual pull
    sm_config_set_clkdiv(&c_wr, 15.0f);

    pio_sm_init(pio, sm_write, offset_write, &c_wr);

    // Set DIOW HIGH and output; data bus starts as input
    pio_sm_set_pins_with_mask(pio, sm_write, 1u << 27, 1u << 27);
    pio_sm_set_consecutive_pindirs(pio, sm_write, 27, 1, true);
    pio_sm_set_consecutive_pindirs(pio, sm_write, 0, 16, false);

    // Start both SMs (they immediately stall on pull block)
    pio_sm_set_enabled(pio, sm_read, true);
    pio_sm_set_enabled(pio, sm_write, true);
}

void ide_pio_read(uint32_t count, uint16_t *buf) {
    // Push count-1 to start the read burst
    pio_sm_put_blocking(pio, sm_read, count - 1);

    for (uint32_t i = 0; i < count; i++) {
        buf[i] = (uint16_t)pio_sm_get_blocking(pio, sm_read);
    }

    // Wait for PIO completion IRQ, then clear it to release the SM
    while (!(pio->irq & (1u << sm_read)))
        tight_loop_contents();
    pio->irq = 1u << sm_read;
}

void ide_pio_write(uint32_t count, const uint16_t *buf) {
    // Data bus to output for the duration of the write
    pio_sm_set_consecutive_pindirs(pio, sm_write, 0, 16, true);

    // Push count-1, then each data word
    pio_sm_put_blocking(pio, sm_write, count - 1);
    for (uint32_t i = 0; i < count; i++) {
        pio_sm_put_blocking(pio, sm_write, (uint32_t)buf[i]);
    }

    // Wait for completion
    while (!(pio->irq & (1u << sm_write)))
        tight_loop_contents();
    pio->irq = 1u << sm_write;

    // Data bus back to input
    pio_sm_set_consecutive_pindirs(pio, sm_write, 0, 16, false);
}

void ide_pio_set_data_dir(bool output) {
    pio_sm_set_consecutive_pindirs(pio, sm_write, 0, 16, output);
}

