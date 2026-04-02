#include "config.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "pico/multicore.h"
#include <string.h>

#define FLASH_TARGET_OFFSET (2048 * 1024 - FLASH_SECTOR_SIZE)
static const uint8_t *flash_target = (const uint8_t *)(XIP_BASE + FLASH_TARGET_OFFSET);

config_t config;

void config_defaults(void) {
    config.main_selected = 0;
    config.feat_selected = 0;
    config.drive_write_protected = true;
    config.auto_mount = false;
    config.iordy_enabled = true;
    config.intrq_enabled = true;
    config.use_lba_mode = false;
    config.cyls = 0;
    config.heads = 0;
    config.spt = 0;
    config.lba_sectors = 0;
}

void config_load(void) {
    config_t tmp;
    memcpy(&tmp, flash_target, sizeof(config_t));
    if (tmp.magic == CONFIG_MAGIC) {
        config = tmp;
    } else {
        config_defaults();
    }
}

void config_save(void) {
    config.magic = CONFIG_MAGIC;
    static uint8_t buf[FLASH_PAGE_SIZE];
    memset(buf, 0, sizeof(buf));
    memcpy(buf, &config, sizeof(config_t));
    multicore_lockout_start_blocking();
    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(FLASH_TARGET_OFFSET, buf, FLASH_PAGE_SIZE);
    restore_interrupts(ints);
    multicore_lockout_end_blocking();
}
