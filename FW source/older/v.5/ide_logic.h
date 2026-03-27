//
// ATAboy Firmware
// Copyright (c) 2026 JJ Dasher
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License v3.0.
//
// See the LICENSE file in this directory for details.
//

#ifndef IDE_LOGIC_H
#define IDE_LOGIC_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ================= SHARED GLOBALS ================= */
extern uint8_t  drive_heads;
extern uint8_t  drive_spt;
extern uint16_t drive_cylinders;
extern bool     is_mounted;
extern bool     drive_write_protected;

/* ================= FUNCTION PROTOTYPES ================= */

uint8_t ide_read_register(uint8_t reg);
bool ide_wait_until_ready(uint32_t timeout_ms);

void ide_hw_init(void);
void ide_reset_drive(void);
void ide_identify_drive(void);
bool ide_get_identify_data(uint16_t* buffer);

bool wait_for_drive_ready(uint32_t timeout_ms);

int32_t ide_read_sectors_lba(uint32_t lba, uint32_t count, uint8_t* buffer);
int32_t ide_write_sectors_lba(uint32_t lba, uint32_t count, const uint8_t* buffer);

int32_t ide_read_cached(uint32_t lba, uint32_t count, uint8_t* buffer);

#ifdef __cplusplus
}
#endif

#endif

