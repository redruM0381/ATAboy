#ifndef IDE_LOGIC_H
#define IDE_LOGIC_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ================= SHARED GLOBALS ================= */
/* Direct access to core1-owned globals removed — use ide_get_core1_state()
   or the ide_* helper APIs (ide_is_mounted, ide_get_core_timings, etc.) */
 
/* ================= FUNCTION PROTOTYPES ================= */

// Initialize the queue and launch the dedicated IDE handling thread on Core 1
void ide_start_core1_worker(void);

uint8_t ide_read_register(uint8_t reg);
void ide_write_register(uint8_t reg, uint8_t val); // Exposed for debug tools
bool ide_wait_until_ready(uint32_t timeout_ms);

void ide_hw_init(void);
void ide_reset_drive(void);
void ide_identify_drive(void);
bool ide_get_identify_data(uint16_t* buffer);
void get_large_geometry(uint16_t native_cyl, uint8_t native_head, uint8_t native_spt, uint16_t* l_cyl, uint8_t* l_head);
void ide_flush_cache(void);
bool ide_set_geometry(uint8_t heads, uint8_t spt);

// Register I/O helpers (wrappers around read/write register)
uint8_t ide_read_8(uint8_t reg);
void ide_write_8(uint8_t reg, uint8_t val);
void ide_get_task_file(uint8_t* task_file);

bool wait_for_drive_ready(uint32_t timeout_ms);

int32_t ide_read_sectors_lba(uint32_t lba, uint32_t count, uint8_t* buffer);
int32_t ide_write_sectors_lba(uint32_t lba, uint32_t count, const uint8_t* buffer);

int32_t ide_read_cached(uint32_t lba, uint32_t count, uint8_t* buffer);

typedef struct {
    uint64_t total_lba;
    uint16_t cyls;
    uint8_t  heads;
    uint8_t  spt;
    bool     drive_write_protected;
    bool     drive_supports_lba48;
    bool     use_lba_mode;
} ide_core1_state_t;

/* Snapshot getter: fills out-> fields from core1 internals (atomic via lockout) */
bool ide_get_core1_state(ide_core1_state_t *out);

/* New: atomically clear core1 geometry / total-LBA (safe from Core0) */
void ide_clear_geometry(void);

/* New: atomically set core1 total LBA (safe from Core0) */
void ide_set_total_lba(uint64_t total);

/* Atomically set core1 geometry (safe for Core0 callers) */
void ide_set_core_geometry(uint16_t cyls, uint8_t heads, uint8_t spt);

/* Atomically export drive geometry (drive_cylinders/drive_heads/drive_spt) */
void ide_export_drive_geometry(uint16_t cyls, uint8_t heads, uint8_t spt);

/* Atomically set core1 timing mode (safe for Core0 callers) */
void ide_set_core_timings(bool enable);

/* New: query core1 timing mode (safe for Core0 callers) */
bool ide_get_core_timings(void);

/* Atomically set LBA mode on core1 (safe for Core0 callers) */
void ide_set_lba_mode(bool enable);

/* Atomically set mounted state (safe for Core0 callers) */
void ide_set_mounted(bool mounted);

/* New: atomically set drive write-protect on core1 (safe for Core0 callers) */
void ide_set_drive_write_protected(bool wp);

/* New: query mounted state (safe for Core0 callers) */
bool ide_is_mounted(void);

/* Notify Core1 that media changed (called from UI/Core0) */
void ide_notify_media_changed(void);

/* Atomically consume & clear media-changed flag (called from Core0/msc) */
bool ide_consume_media_changed(void);

#ifdef __cplusplus
}
#endif

#endif // IDE_LOGIC_H