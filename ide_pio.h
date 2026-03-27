#ifndef IDE_PIO_H
#define IDE_PIO_H

#include <stdint.h>
#include <stdbool.h>

// Initialize PIO state machines for IDE read/write strobes.
// Must be called after GPIO pad config but before any IDE bus operations.
void ide_pio_init(void);

// Execute 'count' DIOR strobes and store the 16-bit results in buf[].
// Caller must set up address, CS, and transceivers (read direction) first.
void ide_pio_read(uint32_t count, uint16_t *buf);

// Execute 'count' DIOW strobes, writing 16-bit words from buf[].
// Caller must set up address, CS, and transceivers (write direction) first.
// Automatically toggles data bus direction (output during write, input after).
void ide_pio_write(uint32_t count, const uint16_t *buf);

// Set data bus (GPIO 0-15) pin direction.  true = output, false = input.
void ide_pio_set_data_dir(bool output);

#endif
