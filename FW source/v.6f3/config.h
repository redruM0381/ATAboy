#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>
#include <stdbool.h>

#define CONFIG_MAGIC 0x1DE45707

typedef struct {
    uint32_t magic;
    uint8_t  main_selected;
    uint8_t  feat_selected;
    bool     drive_write_protected;
    bool     auto_mount;
    bool     iordy_enabled;
    bool     intrq_enabled;
    bool     use_lba_mode;
    uint16_t cyls;
    uint8_t  heads;
    uint8_t  spt;
    uint64_t lba_sectors;
    uint8_t  dev_base;            // 0xA0 = master, 0xB0 = slave
} config_t;

extern config_t config;

void config_load(void);
void config_save(void);
void config_defaults(void);

#endif
