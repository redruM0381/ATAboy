#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>
#include "stdlib.h"
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "tusb.h"
#include "device/usbd.h"
#include "class/cdc/cdc_device.h"
#include "ide_logic.h"


/* --- Externs from ide_logic.cpp --- */
#ifdef __cplusplus
extern "C" {
#endif
    void ide_hw_init(void);
    void ide_reset_drive(void);
    void ide_identify_drive(void);
    void ide_flush_cache(void);
    bool ide_get_identify_data(uint16_t* buffer);
    bool ide_wait_until_ready(uint32_t timeout_ms);
    void get_large_geometry(uint16_t native_cyl, uint8_t native_head, uint8_t native_spt, uint16_t* l_cyl, uint8_t* l_head);
    bool ide_set_geometry(uint8_t heads, uint8_t spt);
    void ide_write_8(uint8_t reg, uint8_t val);
    uint8_t ide_read_8(uint8_t reg);
    void ide_get_task_file(uint8_t* task_file);
    
    
    // Globals for USB sync
    extern uint8_t  drive_heads;
    extern uint8_t  drive_spt;
    extern uint16_t drive_cylinders;
    extern bool     is_mounted;
    extern bool     drive_write_protected;
    extern bool     use_lba_mode;
    extern bool     media_changed_waiting;
#ifdef __cplusplus
}
#endif

// Include this for tight_loop_contents
#include "pico/util/queue.h"

// Define the offset: 2MB total flash minus one 4KB sector
#define FLASH_TARGET_OFFSET (2048 * 1024 - FLASH_SECTOR_SIZE)

typedef struct {
    uint32_t magic;
    uint8_t main_selected;
    uint8_t feat_selected;
    bool drive_write_protected;
    bool auto_mount;
    bool iordy_pin;
    bool comp_timings;   // <-- added
    uint16_t cyls;       
    uint8_t  heads;      
    uint8_t  spt;        
} config_t;

char hdd_model_raw[48] = "";
char hdd_status_text[80] = "";
uint16_t detect_cyls = 0;
uint8_t detect_heads = 0;
uint8_t detect_spt = 0;
uint16_t cur_cyls = 0;
uint8_t cur_heads = 0;
uint8_t cur_spt = 0;
uint64_t total_lba_sectors_from_identify = 0;

bool show_detect_result = false;
const uint8_t *flash_target_contents = (const uint8_t *) (XIP_BASE + FLASH_TARGET_OFFSET);
int confirm_type = 0; // 0 = Load Defaults, 1 = Save

// --- ANSI Escape Sequences ---
#define RESET       "\033[0m"
#define BG_BLUE     "\033[44m"
#define FG_WHITE    "\033[37;1m"
#define FG_RED      "\033[91m"
#define FG_GREEN    "\033[92m"
#define FG_YELLOW   "\033[33;1m"
#define HIDE_CUR    "\033[?25l"
#define CLR_SCR     "\033[2J"
#define SEL_RED     "\033[41;37;1m" 

// --- Key Mappings ---
#define KEY_UP    1001
#define KEY_DOWN  1002
#define KEY_LEFT  1003
#define KEY_RIGHT 1004
#define KEY_ENTER 13
#define KEY_ESC   27
#define KEY_PGUP  1005
#define KEY_PGDN  1006
#define KEY_F10   1010

typedef enum {
    SCREEN_MAIN,
    SCREEN_FEATURES,
    SCREEN_CONFIRM,
    SCREEN_MOUNTED,
    SCREEN_DEBUG
} screen_t;

bool unmount_confirm_active = false;
screen_t current_screen = SCREEN_MAIN;
bool needs_full_redraw = true;
bool last_cdc_connected = false;

// --- IDEasy Variables ---
int main_selected = 0;
int feat_selected = 0;
bool auto_mount = false;
bool iordy_pin = false;
bool comp_timings = false; // The Global Variable for Compat Timings

void draw_at(int x, int y, const char* text) {
    printf("\033[%d;%dH%s", y, x, text);
}

int visible_strlen(const char *s) {
    int len = 0;
    while (*s) {
        if (*s == '\033') {
            s++;
            if (*s == '[') {
                s++;
                while (*s && (*s < '@' || *s > '~')) s++;
                if (*s) s++;
            }
        } else {
            len++;
            s++;
        }
    }
    return len;
}

void sanitize_identify_model(char *s) {
    // Trim trailing spaces
    int len = strlen(s);
    while (len > 0 && s[len - 1] == ' ') {
        s[--len] = '\0';
    }
}

void print_help(const char* text) {
    int start_row = 6;
    int col = 57;
    int width = 21; 
    int max_rows = 11;
    
    for(int i = 0; i < max_rows; i++) {
        printf("\033[%d;%dH%*s", start_row + i, col, width, "");
    }

    printf("\033[%d;%dH", start_row, col);
    
    int current_row = start_row;
    int line_pos = 0;
    int i = 0;

    while (text[i] != '\0' && (current_row - start_row) < max_rows) {
        int word_len = 0;
        while (text[i + word_len] != '\0' && text[i + word_len] != ' ' && text[i + word_len] != '\n') {
            word_len++;
        }

        if (line_pos + word_len > width && line_pos > 0) {
            current_row++;
            if ((current_row - start_row) >= max_rows) break;
            line_pos = 0;
            printf("\033[%d;%dH", current_row, col);
        }

        for (int j = 0; j < word_len; j++) {
            putchar(text[i++]);
            line_pos++;
        }

        if (text[i] == '\n') {
            current_row++;
            if ((current_row - start_row) >= max_rows) break;
            line_pos = 0;
            printf("\033[%d;%dH", current_row, col);
            i++; 
        } else if (text[i] == ' ') {
            if (line_pos < width) {
                putchar(' ');
                line_pos++;
            }
            i++; 
        }
    }
}

void draw_bios_frame() {
    printf(BG_BLUE FG_WHITE CLR_SCR HIDE_CUR "\033[H");
    printf("\033[1;1H%80s", ""); 
    printf("\033[1;10H" FG_WHITE "IDEasy Setup Utility V 0.4 - Copyright (C) 2026 Obsolete Tech");
    
    bool is_features = (current_screen == SCREEN_FEATURES);

    if (!is_features) {
        printf("\033[2;1H╔══════════════════════════════════════╤═══════════════════════════════════════╗");
    } else {
        printf("\033[2;1H╔═════════════════════════════════════════════════════╤════════════════════════╗");
    }

    for (int i = 3; i < 24; i++) {
        printf("\033[%d;1H║", i); 
        printf("\033[%d;80H║", i);
        if (i < 18) {
            if (!is_features) printf("\033[%d;40H│", i); 
            else printf("\033[%d;55H│", i);             
        }
    }

    if (!is_features) {
        printf("\033[18;1H╟──────────────────────────────────────┴───────────────────────────────────────╢");
    } else {
        printf("\033[18;1H╟─────────────────────────────────────────────────────┴────────────────────────╢");
    }

    printf("\033[21;1H╟──────────────────────────────────────────────────────────────────────────────╢");
    printf("\033[24;1H╚══════════════════════════════════════════════════════════════════════════════╝");
}

void update_main_menu() {
    const char* items[] = {
        "► IDEasy Features Setup", "  Mount HDD to USB Mass Storage",
        "  Send RESET to HDD",         "  Auto Detect & Set Geometry",
        "  Load Setup Defaults",       "  Save Setup to EEPROM"
    };

    for (int i = 0; i < 6; i++) {
        int col = (i < 3) ? 4 : 43;
        int row = 6 + (i % 3) * 2;
        printf("\033[%d;%dH", row, col);
        if (i == main_selected) {
            printf(SEL_RED " %-28s " RESET BG_BLUE FG_WHITE, items[i]);
        } else {
            printf(FG_YELLOW " %-28s  " FG_WHITE, items[i]);
        }
    }

    printf("\033[19;3H ESC: Quit to Main Menu                         ↑ ↓ → ←: Select Item");
    printf("\033[20;3H F10: Save Current Setup to EEPROM               Enter: Select");

    const char *display;
    bool is_error = false;

    if (show_detect_result && hdd_status_text[0]) {
        display = hdd_status_text;   
        is_error = true;
    } else if (hdd_model_raw[0]) {
        display = hdd_model_raw;     
    } else {
        display = "No Drive Detected";
    }

    int label_len = 13; 
    int text_len = visible_strlen(display);
    int start_x = (80 - (label_len + text_len)) / 2;

    draw_at(start_x, 22, FG_WHITE "Current HDD: ");

    if (is_error) {
        printf("\033[91;1m%s" RESET BG_BLUE FG_WHITE "\033[K", display); 
    } else if (hdd_model_raw[0]) {
        printf("\033[32;1m%s" RESET BG_BLUE FG_WHITE "\033[K", display); 
    } else {
        printf(FG_YELLOW "%s" RESET BG_BLUE FG_WHITE "\033[K", display); 
    }

    printf(RESET BG_BLUE FG_WHITE "\033[K");
    printf("\033[22;78H  ║");

    char geo_vals[64];
    if (use_lba_mode) {
        uint32_t mb_size = (uint32_t)((uint64_t)total_lba_sectors_from_identify * 512 / 1048576);
        snprintf(geo_vals, sizeof(geo_vals), "LBA Mode Active (%lu MB)", mb_size);
    } else {
        snprintf(geo_vals, sizeof(geo_vals), "%d Cyl / %d Hd / %d SPT", cur_cyls, cur_heads, cur_spt);
    }

    int geo_total_len = 18 + strlen(geo_vals);
    int geo_x = (80 - geo_total_len) / 2;

    draw_at(geo_x, 23, FG_WHITE "Current Geometry: "); 

    bool is_valid = (use_lba_mode && total_lba_sectors_from_identify > 0) || 
                    (!use_lba_mode && cur_cyls > 0 && cur_heads > 0 && cur_spt > 0);

    if (is_valid) {
        printf("\033[92;1m%s", geo_vals); 
    } else {
        printf("\033[91;1m%s", geo_vals); 
    }

    printf(RESET BG_BLUE FG_WHITE "\033[K");
    printf("\033[23;78H  ║"); 

    printf("\033[24;79H");
    fflush(stdout);
}

void update_features_menu() {
    const char* labels[] = {"Write Protect", "Auto Mount at Start", "IORDY", "Timings", "Debug Mode"};
    const char* helps[] = {
        "Prevents any write commands from reaching the HDD hardware.",
        "Automatically mounts the drive to USB on power-up sequence.",
        "Selects hardware pin 27 for IORDY or software emulation.  NOT YET FUNCTIONAL!",
        "Allows relaxed timing for particularly picky/older drives.  May be slower.",
        "Open low-level drive diagnostics and register status screen."
    };

    printf("\033[3;63H" FG_WHITE "Item Help");
    printf("\033[4;55H\u251C────────────────────────\u2562"); 

    for (int i = 0; i < 5; i++) {
        int row = 4 + i; 
        printf("\033[%d;4H" FG_WHITE "%-25s", row, labels[i]);
        printf("\033[%d;35H" FG_YELLOW "[", row);
        if (i == feat_selected) printf(SEL_RED);
        else printf(FG_YELLOW);

        if (i == 0) printf("%-8s", drive_write_protected ? "Enabled" : "Disabled");
        else if (i == 1) printf("%-8s", auto_mount    ? "Enabled" : "Disabled");
        else if (i == 2) printf("%-8s", iordy_pin     ? "Pin 27" : "Software");
        else if (i == 3) printf("%-8s", comp_timings  ? "Compat" : "Normal");
        else if (i == 4) printf("%-8s", "Enter");
        
        printf(RESET BG_BLUE FG_WHITE "]");
        if (i == feat_selected) print_help(helps[i]);
    }
    printf("\033[19;3H ESC: Back  ↑↓: Select  +/-/PU/PD: Value  F10: Save");
}

int get_input() {
    int c = getchar_timeout_us(100000);
    if (c == PICO_ERROR_TIMEOUT) return -1;
    if (c == 27) { 
        int n1 = getchar_timeout_us(10000);
        if (n1 == '[') {
            int n2 = getchar_timeout_us(10000);
            if (n2 == 'A') return KEY_UP;
            if (n2 == 'B') return KEY_DOWN;
            if (n2 == 'C') return KEY_RIGHT;
            if (n2 == 'D') return KEY_LEFT;
            if (n2 == '2') {
                if (getchar_timeout_us(10000) == '1') {
                    if (getchar_timeout_us(10000) == '~') return KEY_F10;
                }
            }
            if (n2 == '5') {
                if (getchar_timeout_us(10000) == '~') return KEY_PGUP;
            }
            if (n2 == '6') {
                if (getchar_timeout_us(10000) == '~') return KEY_PGDN;
            }
        }
        return KEY_ESC;
    }
    return c;
}

void save_config() {
    config_t config = {
        .magic = 0x1DE45701,
        .main_selected = main_selected,
        .feat_selected = feat_selected,
        .drive_write_protected = drive_write_protected,
        .auto_mount = auto_mount,
        .iordy_pin = iordy_pin,
        .comp_timings = comp_timings,
        .cyls = cur_cyls,
        .heads = cur_heads,
        .spt = cur_spt
    };

    static uint8_t buffer[FLASH_PAGE_SIZE];
    memset(buffer, 0, FLASH_PAGE_SIZE);
    memcpy(buffer, &config, sizeof(config));
    
    multicore_lockout_start_blocking();
    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase((uint32_t)FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program((uint32_t)FLASH_TARGET_OFFSET, buffer, FLASH_PAGE_SIZE);
    restore_interrupts(ints);
    multicore_lockout_end_blocking();
}

void load_config() {
    config_t config;
    memcpy(&config, flash_target_contents, sizeof(config));
    
    if (config.magic == 0x1DE45701) {
        main_selected = config.main_selected;
        feat_selected = config.feat_selected;
        drive_write_protected = config.drive_write_protected;        
        auto_mount = config.auto_mount;
        iordy_pin = config.iordy_pin;
        comp_timings = config.comp_timings;
        cur_cyls = config.cyls;
        cur_heads = config.heads;
        cur_spt = config.spt;
    }
}

void load_defaults() {
    drive_write_protected = true;
    auto_mount = false;
    iordy_pin = false;
    comp_timings = false;
    main_selected = 0;
    feat_selected = 0;
    cur_cyls = 0;
    cur_heads = 0;
    cur_spt = 0;
}


// --- Debug UI Helpers ---

// Decodes ATA string (Big Endian byte pairs) into a char buffer
void decode_ata_string(uint16_t* buffer, int offset, int len_words, char* out_buf) {
    int char_idx = 0;
    for (int i = 0; i < len_words; i++) {
        uint16_t w = buffer[offset + i];
        out_buf[char_idx++] = (char)(w >> 8);     // High byte
        out_buf[char_idx++] = (char)(w & 0xFF);   // Low byte
    }
    out_buf[char_idx] = '\0';
    
    // Trim trailing spaces
    while (char_idx > 0 && out_buf[char_idx - 1] == ' ') {
        out_buf[--char_idx] = '\0';
    }
}

// Prints a line inside the black debug window
// line_idx: 0 to 16 (relative to the black box)
void debug_print(int line_idx, const char* color, const char* fmt, ...) {
    int start_col = 5;   
    int start_row = 3;
    int inner_x = start_col + 2;
    int inner_y = start_row + 1;
    int inner_w = 68; // box_w (72) - 4

    if (line_idx > 16) return; // Bounds check

    char buf[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    // Position cursor, set background black, clear line, print text
    printf("\033[%d;%dH\033[40m%s%-*s", 
        inner_y + line_idx, 
        inner_x, 
        color, // Formatting (color)
        inner_w, 
        buf);  
    
    printf(RESET); // Reset after printing
}

void debug_cls() {
    for(int i=0; i<17; i++) debug_print(i, FG_WHITE, "");
}

void run_debug_identify() {
    debug_cls();
    debug_print(0, FG_YELLOW, "Sending IDENTIFY DEVICE (0xEC)...");
    
    // Ensure drive is selected/ready
    ide_write_8(6, 0xA0); 
    if (!ide_wait_until_ready(1000)) {
        debug_print(1, "\033[91;1m", "TIMEOUT: Drive BSY or Not Present.");
        return;
    }

    ide_identify_drive();
    uint16_t id[256];
    
    if (ide_get_identify_data(id)) {
        char model[41];
        char serial[21];
        char fw[9];

        decode_ata_string(id, 27, 20, model);
        decode_ata_string(id, 10, 10, serial);
        decode_ata_string(id, 23, 4, fw);

        debug_print(0, "\033[92;1m", "IDENTIFY SUCCESSFUL");
        debug_print(2, FG_WHITE, "Model:  \033[96m%s", model);
        debug_print(3, FG_WHITE, "Serial: \033[96m%s", serial);
        debug_print(4, FG_WHITE, "FW Rev: \033[96m%s", fw);
        debug_print(5, FG_WHITE, "--------------------------------------------------");

        // CHS Info
        debug_print(6, FG_YELLOW, "[CHS Geometry]");
        debug_print(7, FG_WHITE, "Cyls: %-5d  Heads: %-3d  Sectors: %-3d", id[1], id[3], id[6]);
        
        // LBA Info
        bool lba_supp = (id[49] & 0x0200);
        bool lba48_supp = (id[83] & (1 << 10));
        uint32_t lba28_cap = id[60] | ((uint32_t)id[61] << 16);
        uint64_t lba48_cap = ((uint64_t)id[103] << 48) | ((uint64_t)id[102] << 32) |
                             ((uint64_t)id[101] << 16) | ((uint64_t)id[100]);

        debug_print(9, FG_YELLOW, "[Capabilities]");
        debug_print(10, FG_WHITE, "LBA Supported:    %s", lba_supp ? "\033[92mYes" : "\033[91mNo");
        debug_print(11, FG_WHITE, "LBA48 Supported:  %s", lba48_supp ? "\033[92mYes" : "\033[91mNo");
        
        if (lba48_supp) {
            uint32_t gb = (uint32_t)(((uint64_t)lba48_cap * 512) / 1000000000);
            debug_print(12, FG_WHITE, "Capacity:         %lu Sectors (~%lu GB)", (unsigned long)lba48_cap, gb);
        } else if (lba_supp) {
            uint32_t mb = (uint32_t)(((uint64_t)lba28_cap * 512) / 1000000);
            debug_print(12, FG_WHITE, "Capacity:         %lu Sectors (~%lu MB)", lba28_cap, mb);
        }

        // Features
        debug_print(14, FG_YELLOW, "[Advanced]");
        debug_print(15, FG_WHITE, "DMA Support: %04X  PIO Support: %04X", id[49], id[64]);
        debug_print(16, FG_WHITE, "ATA Major Ver: %04X", id[80]);

    } else {
        debug_print(1, "\033[91;1m", "ERROR: DRQ not asserted after command.");
        debug_print(2, FG_WHITE, "Check cabling, Master/Slave jumper, or power.");
    }
}

void draw_debug_overlay() {
    int box_w = 72;
    int box_h = 20;      
    int start_col = 5;   
    int start_row = 3;

    // Interior "Black Area" dimensions
    int inner_x = start_col + 2;
    int inner_y = start_row + 1;
    int inner_w = box_w - 4;
    int inner_h = box_h - 3;

    // 1. Force Reset and Draw Shadow
    printf(RESET);
    for (int i = 0; i < box_h; i++) {
        printf("\033[%d;%dH\033[40m%*s", start_row + 1 + i, start_col + 2, box_w, "");
    }

    // 2. Draw Main Blue Body
    for (int i = 0; i < box_h; i++) {
        printf("\033[%d;%dH\033[44m%*s", start_row + i, start_col, box_w, "");
    }

    // 3. Draw Black Debug Window
    for (int i = 0; i < inner_h; i++) {
        printf("\033[%d;%dH\033[40m%*s", inner_y + i, inner_x, inner_w, "");
    }

    // 4. Draw White Borders (Overlaying the backgrounds)
    printf(FG_WHITE BG_BLUE);
    draw_at(start_col, start_row + 0, "╔══════════════════════════════════════════════════════════════════════╗");
    for(int i = 1; i < box_h - 1; i++) {
        // We only draw the side pipes so we don't overwrite the black center
        draw_at(start_col, start_row + i, "║");
        draw_at(start_col + box_w - 1, start_row + i, "║");
    }
    draw_at(start_col, start_row + box_h - 1, "╚══════════════════════════════════════════════════════════════════════╝");

    // 5. Explicitly Position Text
    // Title (Top line of frame)
    draw_at(start_col + 29, start_row, "[ Debug Mode ]");
    
    // Footer (Bottom line of frame)
    // We use a specific row calculation to ensure it's on the blue border
    int footer_row = start_row + box_h - 2;
    printf("\033[%d;%dH" FG_WHITE BG_BLUE " ESC: Return  I: IDENT  T: Task File  E: Error Bits  S: Seek Test", footer_row, start_col + 3);
    
    printf(RESET);
    fflush(stdout);
}

void draw_confirm_box(const char* message) {
    int box_w = 52;
    int box_h = 5;
    int start_col = 14; 
    int start_row = 10;
    int interior_w = 50; 

    for (int i = 0; i < box_h; i++) {
        printf("\033[%d;%dH\033[40m%*s", start_row + 1 + i, start_col + 2, box_w, "");
    }
    for (int i = 0; i < box_h; i++) {
        printf("\033[%d;%dH\033[41m%*s", start_row + i, start_col, box_w, "");
    }

    printf(SEL_RED);
    printf("\033[%d;%dH╔══════════════════════════════════════════════════╗", start_row + 0, start_col);
    printf("\033[%d;%dH║                                                  ║", start_row + 1, start_col);
    
    int msg_len = visible_strlen(message);
    if (msg_len > interior_w) msg_len = interior_w; 
    int pad_left = (interior_w - msg_len) / 2;
    int pad_right = interior_w - msg_len - pad_left;

    printf("\033[%d;%dH║%*s%s%*s║", start_row + 2, start_col, pad_left, "", message, pad_right, "");
    printf("\033[%d;%dH║                                                  ║", start_row + 3, start_col);
    printf("\033[%d;%dH╚══════════════════════════════════════════════════╝", start_row + 4, start_col);
    printf(RESET);
}

void draw_error_box(const char* error_msg) {
    int box_w = 52;
    int box_h = 7; 
    int start_col = 14;
    int start_row = 9; 
    
    for (int i = 0; i < box_h; i++) {
        printf("\033[%d;%dH\033[40m%*s", start_row + 1 + i, start_col + 2, box_w, "");
    }
    for (int i = 0; i < box_h; i++) {
        printf("\033[%d;%dH\033[41m%*s", start_row + i, start_col, box_w, "");
    }

    printf(SEL_RED); 
    printf("\033[%d;%dH╔══════════════════════════════════════════════════╗", start_row + 0, start_col);
    printf("\033[%d;%dH║                      ERROR!                      ║", start_row + 1, start_col);
    printf("\033[%d;%dH╠══════════════════════════════════════════════════╣", start_row + 2, start_col);
    printf("\033[%d;%dH║                                                  ║", start_row + 3, start_col);
    
    int visible_len = visible_strlen(error_msg);
    int x_off = (50 - visible_len) / 2;

    printf("\033[%d;%dH\033[41;97m%s",
        start_row + 3,
        start_col + 1 + x_off,
        error_msg);


    printf(SEL_RED); 
    printf("\033[%d;%dH║                                                  ║", start_row + 4, start_col);
    printf("\033[%d;%dH║                       ", start_row + 5, start_col);
    printf("\033[103;30m[OK]\033[0m"); 
    printf(SEL_RED "                       ║");
    printf("\033[%d;%dH╚══════════════════════════════════════════════════╝", start_row + 6, start_col);

    printf(RESET);
    fflush(stdout);
}

void draw_selection_menu(uint16_t* identify_data, int selected_idx) {
    int box_w = 64;
    int box_h = 14;      
    int start_col = 9;   
    int start_row = 5;   

    uint16_t n_cyl = identify_data[1];
    uint8_t n_hd   = (uint8_t)identify_data[3];
    uint8_t n_spt  = (uint8_t)identify_data[6];
    
    bool drive_supports_lba = (identify_data[49] & 0x0200);
    uint32_t lba28 = identify_data[60] | ((uint32_t)identify_data[61] << 16);
    uint64_t lba48 = ((uint64_t)identify_data[103] << 48)
                   | ((uint64_t)identify_data[102] << 32)
                   | ((uint64_t)identify_data[101] << 16)
                   | ((uint64_t)identify_data[100] << 0);
    uint64_t lba_total64 = lba48 ? lba48 : (uint64_t)lba28;
    
    uint16_t l_cyl; 
    uint8_t l_hd;
    get_large_geometry(n_cyl, n_hd, n_spt, &l_cyl, &l_hd);

    uint32_t n_size = (uint32_t)(((uint64_t)n_cyl * n_hd * n_spt * 512) / 1048576);
    uint32_t l_size = (uint32_t)(((uint64_t)l_cyl * l_hd * n_spt * 512) / 1048576);
    uint32_t lba_size = (uint32_t)((lba_total64 * 512ULL) / 1048576ULL);
    bool drive_supports_lba48 = ((identify_data[83] & (1<<10)) != 0) || (lba_total64 > 0x0FFFFFFFULL);

    // Shadow
    for (int i = 0; i < box_h; i++) {
        printf("\033[%d;%dH\033[40m%*s", start_row + 1 + i, start_col + 2, box_w, "");
    }
    // Background
    for (int i = 0; i < box_h; i++) {
        printf("\033[%d;%dH\033[41m%*s", start_row + i, start_col, box_w, "");
    }

    printf(SEL_RED); 
    draw_at(start_col, start_row + 0, "╔══════════════════════════════════════════════════════════════╗");

    int interior_w = 62;
    int label_len = 14; 
    int visible_model_len = strlen(hdd_model_raw);
    int start_x = start_col + 1 + ((interior_w - (label_len + visible_model_len)) / 2);

    draw_at(start_col + 1, start_row + 1, "                                                              ");
    draw_at(start_col, start_row + 1, "║");
    printf("\033[%d;%dHDetected HDD: \033[32;1m%s\033[0m", start_row + 1, start_x, hdd_model_raw);
    printf(SEL_RED "\033[%d;%dH║", start_row + 1, start_col + 63);

    draw_at(start_col, start_row + 2, "║                Select Drive Geometry Option:                 ║");
    draw_at(start_col, start_row + 3, "╠══════════════════════════════════════════════════════════════╣");
    draw_at(start_col, start_row + 4, "║  MODE         SIZE       CYLS      HEADS      SPT            ║");
    draw_at(start_col, start_row + 5, "║ ──────────────────────────────────────────────────────────── ║");

    const char* modes[] = {"NORMAL  ", "LARGE   ", "LBA     ", "MANUAL  "};
    const char* lba_label = drive_supports_lba48 ? "LBA48  " : "LBA     ";
    
    for (int i = 0; i < 4; i++) {
        printf("\033[%d;%dH║ ", start_row + 6 + i, start_col);
        
        bool is_lba_row = (i == 2);
        bool lba_unsupported = (is_lba_row && !drive_supports_lba);

        if (i == selected_idx) {
            if (lba_unsupported) printf("\033[41;33m %-8s \033[0m" SEL_RED, (is_lba_row ? lba_label : modes[i])); 
            else printf("\033[103;30m %-8s \033[0m" SEL_RED, (is_lba_row ? lba_label : modes[i])); 
        } else {
            if (lba_unsupported) printf("\033[90m %-8s ", (is_lba_row ? lba_label : modes[i])); 
            else printf(FG_WHITE " %-8s ", (is_lba_row ? lba_label : modes[i]));
        }

        if (lba_unsupported) {
            printf("\033[90m      --- MB    (Not Supported by Drive)           \033[0m" SEL_RED);
        }
        else if (i == 0) { 
            printf("   %4lu MB    %-5u       %-3u       %-3u            ", n_size, n_cyl, n_hd, n_spt);
        } 
        else if (i == 1) { 
            printf("   %4lu MB    %-5u       %-3u       %-3u            ", l_size, l_cyl, l_hd, n_spt);
        } 
        else if (i == 2) { 
            if (drive_supports_lba48) {
                if (lba_size > 9999) {
                    uint32_t gb = lba_size / 1024;
                    printf("   %4lu GB           LBA (48-bit)                  ", (unsigned long)gb);
                } else {
                    printf("   %4lu MB           LBA (48-bit)                  ", (unsigned long)lba_size);
                }
            } else {
                if (lba_size > 9999) {
                    uint32_t gb = lba_size / 1024;
                    printf("   %4lu GB           LBA (28-bit)                  ", (unsigned long)gb);
                } else {
                    printf("   %4lu MB           LBA (28-bit)                  ", (unsigned long)lba_size);
                }
            }
        }
        else if (i == 3) { 
            if (detect_cyls > 0) {
                uint32_t m_size = (uint32_t)((uint64_t)detect_cyls * detect_heads * detect_spt * 512 / 1048576);
                printf("   %4lu MB    %-5u       %-3u       %-3u            ", m_size, detect_cyls, detect_heads, detect_spt);
            } else {
                printf("   ---- MB    -----       ---       ---            ");
            }
        }
        printf("║"); 
    }

    draw_at(start_col, start_row + 10, "╠══════════════════════════════════════════════════════════════╣");
    draw_at(start_col, start_row + 11, "║  ↑ ↓: Mode    TAB: Change CHS     Enter: Select    Esc: Quit ║");
    draw_at(start_col, start_row + 12, "║  \033[33m   LBA Recommended for modern drives; NORMAL for legacy.    \033[0m" SEL_RED "║");
    draw_at(start_col, start_row + 13, "╚══════════════════════════════════════════════════════════════╝");
    
    printf(RESET);
    fflush(stdout);
}

void core1_entry() {
    bool trigger_overlay = false;
    while (true) {
        bool connected = tud_cdc_connected();
        if (connected && !last_cdc_connected) {
            sleep_ms(200); 
            needs_full_redraw = true;
        }
        last_cdc_connected = connected;

        if (needs_full_redraw) {
            draw_bios_frame(); 
            if (current_screen == SCREEN_MAIN || current_screen == SCREEN_CONFIRM || current_screen == SCREEN_MOUNTED) {
                update_main_menu();
            } else if (current_screen == SCREEN_FEATURES) {
                update_features_menu();
            } else if (current_screen == SCREEN_DEBUG) {
                draw_debug_overlay();
            }

            if (show_detect_result || current_screen == SCREEN_CONFIRM || current_screen == SCREEN_MOUNTED || current_screen == SCREEN_DEBUG) {
                trigger_overlay = true;
            }
            needs_full_redraw = false;
        }

        if (!show_detect_result && current_screen != SCREEN_CONFIRM && current_screen != SCREEN_MOUNTED && current_screen != SCREEN_DEBUG) {
            if (current_screen == SCREEN_MAIN) update_main_menu();
            else if (current_screen == SCREEN_FEATURES) update_features_menu();
        }

        if (show_detect_result) {
            if (trigger_overlay) { 
                draw_error_box(hdd_status_text);
                trigger_overlay = false; 
            }
        } else if (current_screen == SCREEN_CONFIRM) {
            if (trigger_overlay) {
                if (confirm_type == 0)      draw_confirm_box("Load Defaults and Save to EEPROM (Y/N)?");
                else if (confirm_type == 1) draw_confirm_box("Save Current Setup to EEPROM (Y/N)?");
                else if (confirm_type == 2) draw_confirm_box("RESET the drive (Y/N)?");
                else if (confirm_type == 3) draw_confirm_box("Are you sure you want to mount the drive (Y/N)?");
                else if (confirm_type == 4) draw_confirm_box("Are you sure you want to unmount (Y/N)?");
                trigger_overlay = false;
            }
        } else if (current_screen == SCREEN_MOUNTED) {
            if (trigger_overlay) {
                draw_confirm_box("Drive has been mounted as USB Mass Storage!");
                draw_at(15, 12, SEL_RED "            Press 'U' to Unmount Drive            ║" RESET);
                trigger_overlay = false;
            }
        } else if (current_screen == SCREEN_DEBUG) {
            if (trigger_overlay) {
                draw_debug_overlay();
                trigger_overlay = false;
            }
        }

        printf("\033[24;79H"); fflush(stdout);

        int k = get_input();
        if (k == -1) { tight_loop_contents(); continue; }

        if (show_detect_result) {
            if (k == KEY_ENTER || k == KEY_ESC) {
                show_detect_result = false;
                hdd_status_text[0] = '\0';
                hdd_model_raw[0] = '\0';
                cur_cyls = 0; cur_heads = 0; cur_spt = 0;
                total_lba_sectors_from_identify = 0;
                use_lba_mode = false;
                drive_cylinders = 0; drive_heads = 0; drive_spt = 0;
                is_mounted = false;
                needs_full_redraw = true;
            }
            continue; 
        }

        if (current_screen == SCREEN_MOUNTED) {
            if (k == 'u' || k == 'U') {
                current_screen = SCREEN_CONFIRM; confirm_type = 4;
                trigger_overlay = true; needs_full_redraw = true;
            }
            continue;
        }

        if (current_screen == SCREEN_DEBUG) {
            if (k == KEY_ESC) {
                current_screen = SCREEN_FEATURES;
                needs_full_redraw = true;
            }
            else if (k == 'i' || k == 'I') {
                debug_cls();
                run_debug_identify();
            }
            else if (k == 't' || k == 'T') {
                debug_cls();
                uint8_t task_file[8];
                ide_get_task_file(task_file);
                
                // Color formatting for the status byte
                const char* status_color = (task_file[7] & 0x81) ? "\033[91;1m" : "\033[92m";

                // We use debug_print to handle the coordinate math and background clearing
                debug_print(0, FG_RED, "[Task File] " FG_WHITE "ERR:%02X SEC:%02X SN:%02X CL:%02X CH:%02X DH:%02X ST:%s%02X" RESET, 
                task_file[1], task_file[2], task_file[3], task_file[4], 
                task_file[5], task_file[6], status_color, task_file[7]);
            }
            else if (k == 'e' || k == 'E') {
                debug_cls();
                uint8_t err_reg = ide_read_8(1); // Read Error Register directly
                
                char err_buf[64] = {0};
                if (err_reg == 0) {
                    snprintf(err_buf, sizeof(err_buf), "\033[92mNo Errors Reported\033[0m");
                } else {
                    // Assemble the error string
                    strcat(err_buf, (err_reg & 0x80) ? "BBK " : "");
                    strcat(err_buf, (err_reg & 0x40) ? "UNC " : "");
                    strcat(err_buf, (err_reg & 0x20) ? "MC "  : "");
                    strcat(err_buf, (err_reg & 0x10) ? "IDNF " : "");
                    strcat(err_buf, (err_reg & 0x08) ? "MCR "  : "");
                    strcat(err_buf, (err_reg & 0x04) ? "ABRT " : "");
                    strcat(err_buf, (err_reg & 0x02) ? "TK0 "  : "");
                    strcat(err_buf, (err_reg & 0x01) ? "AMNF " : "");
                }

                debug_print(0, FG_RED, "[Error Bits] %s", err_buf);
            }
            else if (k == 's' || k == 'S') {
                debug_cls();
                debug_print(0, FG_YELLOW, "Seek Test Running...");

                uint16_t id[256];
                ide_write_8(6, 0xA0); 
                if (!ide_wait_until_ready(1000)) {
                    debug_print(1, FG_RED, "TIMEOUT: Drive not ready.");
                    sleep_ms(1500);
                    continue;
                }

                ide_identify_drive(); 
                if (!ide_get_identify_data(id)) {
                    debug_print(1, FG_RED, "ERROR: No data from IDENTIFY.");
                    sleep_ms(1500);
                    continue;
                }

                bool supports_lba = (id[49] & 0x0200);
                uint32_t max_range = supports_lba ? (id[60] | ((uint32_t)id[61] << 16)) : id[1];
                if (max_range == 0) max_range = 1024;

                double x = 0;
                while (true) {
                    int exit_check = getchar_timeout_us(0);
                    if (exit_check == 27) break; 

                    // Sine wave for positioning
                    double pos = (sin(x) * 0.45) + 0.5;
                    uint32_t target = (uint32_t)(pos * (max_range - 1));

                    if (supports_lba) {
                        ide_write_8(3, target & 0xFF);         
                        ide_write_8(4, (target >> 8) & 0xFF);  
                        ide_write_8(5, (target >> 16) & 0xFF); 
                        ide_write_8(6, 0xE0 | ((target >> 24) & 0x0F)); 
                    } else {
                        ide_write_8(4, target & 0xFF);         
                        ide_write_8(5, (target >> 8) & 0xFF);  
                        ide_write_8(6, 0xA0); 
                    }

                    // --- THE NOISE TRICK ---
                    ide_write_8(2, 1);    // Sector Count: 1
                    ide_write_8(7, 0x20); // READ SECTOR (Forced mechanical action)

                    // 1. Wait for BSY to clear and DRQ (Data Request) to be ready
                    // Using a tight loop to satisfy the drive quickly
                    while (ide_read_8(7) & 0x80); 
                    
                    // 2. DRAIN THE BUFFER: This is what creates the "Activity" noise
                    // We must read 256 words (512 bytes) to clear the drive's DRQ
                    if (ide_read_8(7) & 0x08) {
                        for (int i = 0; i < 256; i++) {
                            // We don't need to store it, just read the port
                            // Your ide_read_8(0) reads the 16-bit data port in your logic
                            ide_read_8(0); 
                        }
                    }

                    // Visual "Flow"
                    char bar[61];
                    memset(bar, '-', 60);
                    bar[(int)(pos * 59)] = '#';
                    bar[60] = '\0';

                    debug_print(14, FG_GREEN, "[%s]", bar);
                    debug_print(15, FG_WHITE, "Target %s: %lu (ST:%02X)", 
                                supports_lba ? "LBA" : "CYL", target, ide_read_8(7));

                    x += 0.12;    // Adjust for "musical" frequency
                    // No sleep_ms needed here because the I/O bottleneck 
                    // provides the timing naturally, just like a real OS.
                }
                debug_print(14, FG_WHITE, "                                                                ");
                debug_print(15, FG_WHITE, "                                                                ");

                // 2. Small delay to let the Serial/Terminal buffer settle
                sleep_ms(50); 

                // 3. Clear the whole terminal screen buffer
                printf("\033[2J\033[H"); 

                // 4. Force state reset
                debug_cls(); 
                current_screen = SCREEN_DEBUG; 
                needs_full_redraw = true;
            }
            continue;
        }

        if (current_screen == SCREEN_CONFIRM) {
            if (k == 'y' || k == 'Y') {
                if (confirm_type == 0) { load_defaults(); save_config(); current_screen = SCREEN_MAIN; }
                else if (confirm_type == 1) { save_config(); current_screen = SCREEN_MAIN; }
                else if (confirm_type == 2) { 
                    ide_reset_drive(); 
                    cur_cyls = 0; cur_heads = 0; cur_spt = 0;
                    drive_cylinders = 0; drive_heads = 0; drive_spt = 0;
                    is_mounted = false;
                    detect_cyls = 0; detect_heads = 0; detect_spt = 0;
                    hdd_model_raw[0] = '\0';
                    snprintf(hdd_status_text, sizeof(hdd_status_text), "\033[93;1mDrive Reset - Redetect Required");
                    show_detect_result = true; current_screen = SCREEN_MAIN;
                }
                else if (confirm_type == 3) { is_mounted = true; ide_flush_cache(); media_changed_waiting = true; current_screen = SCREEN_MOUNTED; } // Added a cache flush
                else if (confirm_type == 4) { is_mounted = false; media_changed_waiting = true; current_screen = SCREEN_MAIN; }
                needs_full_redraw = true;
            } 
            else if (k == 'n' || k == 'N' || k == KEY_ESC) {
                current_screen = (confirm_type == 4) ? SCREEN_MOUNTED : SCREEN_MAIN;
                needs_full_redraw = true;
            }
            continue;
        }

        if (k == KEY_F10) { 
            current_screen = SCREEN_CONFIRM; confirm_type = 1;
            trigger_overlay = true; needs_full_redraw = true;
            continue;
        }

        if (current_screen == SCREEN_MAIN) {
            if (k == KEY_UP && main_selected % 3 > 0) main_selected--;
            else if (k == KEY_DOWN && main_selected % 3 < 2) main_selected++;
            else if (k == KEY_RIGHT && main_selected < 3) main_selected += 3;
            else if (k == KEY_LEFT && main_selected >= 3) main_selected -= 3;
            else if (k == KEY_ENTER) {
                if (main_selected == 0) { current_screen = SCREEN_FEATURES; feat_selected = 0; needs_full_redraw = true; } 
                else if (main_selected == 1) { 
                    bool drive_valid = (hdd_model_raw[0] != '\0');
                    bool geometry_valid = (use_lba_mode && total_lba_sectors_from_identify > 0) ||
                                         (!use_lba_mode && cur_cyls > 0 && cur_heads > 0 && cur_spt > 0);

                    if (!drive_valid || !geometry_valid) {
                        snprintf(hdd_status_text, sizeof(hdd_status_text), "\033[91;1mDetect drive and set geometry first!");
                        hdd_model_raw[0] = '\0'; cur_cyls = 0; cur_heads = 0; cur_spt = 0;
                        total_lba_sectors_from_identify = 0; use_lba_mode = false;
                        drive_cylinders = 0; drive_heads = 0; drive_spt = 0; is_mounted = false;
                        show_detect_result = true; trigger_overlay = true; needs_full_redraw = true;
                    } else {
                        current_screen = SCREEN_CONFIRM; confirm_type = 3; trigger_overlay = true; needs_full_redraw = true;
                    }
                }
                else if (main_selected == 2) { current_screen = SCREEN_CONFIRM; confirm_type = 2; trigger_overlay = true; needs_full_redraw = true; }
                else if (main_selected == 3) { // AUTO DETECT DRIVE
                    ide_reset_drive();
                    if (ide_wait_until_ready(5000)) { 
                        sleep_ms(100); 
                        ide_identify_drive();
                        uint16_t id_buf[256];
                        if (ide_get_identify_data(id_buf)) {
                            for (int i = 0; i < 20; i++) {
                                uint16_t val = id_buf[27 + i];
                                hdd_model_raw[i * 2]     = (char)(val >> 8);
                                hdd_model_raw[i * 2 + 1] = (char)(val & 0xFF);
                            }
                            hdd_model_raw[40] = '\0'; sanitize_identify_model(hdd_model_raw);

                            bool drive_supports_lba = (id_buf[49] & 0x0200);
                            int geo_idx = drive_supports_lba ? 2 : 0;
                            bool waiting_for_choice = true;
                            bool selection_needs_draw = true; 
                            int start_col = 9; int start_row = 6;

                            while(waiting_for_choice) {
                                if (selection_needs_draw) {
                                    draw_selection_menu(id_buf, geo_idx); 
                                    selection_needs_draw = false; 
                                }

                                int choice = get_input();
                                if (choice == -1) { tight_loop_contents(); continue; }

                                if (choice == KEY_UP && geo_idx > 0) {
                                    geo_idx--;
                                    if (!drive_supports_lba && geo_idx == 2) geo_idx--;
                                    selection_needs_draw = true;
                                } 
                                else if (choice == KEY_DOWN && geo_idx < 3) {
                                    geo_idx++;
                                    if (!drive_supports_lba && geo_idx == 2) geo_idx++;
                                    selection_needs_draw = true;
                                } 
                                else if (choice == KEY_ESC) { waiting_for_choice = false; } 
                                
                                else if (choice == '\t' || (choice == KEY_ENTER && geo_idx == 3 && (detect_cyls == 0 || detect_heads == 0 || detect_spt == 0))) {
                                    geo_idx = 3; 
                                    draw_selection_menu(id_buf, geo_idx); 
                                    int m_fields[3] = {detect_cyls, detect_heads, detect_spt};
                                    int m_cols[3] = {start_col + 26, start_col + 38, start_col + 48};
                                    
                                    for (int f = 0; f < 3; f++) {
                                        char in_buf[7] = {0}; int p = 0;
                                        draw_at(m_cols[f], start_row + 8, "\033[103;30m     \033[0m");
                                        printf("\033[%d;%dH", start_row + 8, m_cols[f]);
                                        while(true) {
                                            int c = get_input();
                                            if (c >= '0' && c <= '9' && p < 5) { in_buf[p++] = (char)c; putchar(c); }
                                            else if ((c == 8 || c == 127) && p > 0) { p--; printf("\b \b\033[%d;%dH", start_row + 8, m_cols[f] + p); }
                                            else if (c == KEY_ENTER || c == '\t') { 
                                                in_buf[p] = '\0'; if (p > 0) m_fields[f] = atoi(in_buf); 
                                                draw_at(m_cols[f], start_row + 8, RESET "\033[41;37;1m"); 
                                                printf("%-5d", m_fields[f]); break; 
                                            }
                                            else if (c == KEY_ESC) { f = 3; break; }
                                            tight_loop_contents();
                                        }
                                    }
                                    detect_cyls = (uint16_t)m_fields[0];
                                    detect_heads = (uint8_t)m_fields[1];
                                    detect_spt = (uint8_t)m_fields[2];
                                    selection_needs_draw = true; 
                                }
                                else if (choice == KEY_ENTER) {
                                    bool selection_valid = false;
                                    if (geo_idx == 0) { 
                                        use_lba_mode = false;
                                        cur_cyls = id_buf[1]; cur_heads = (uint8_t)id_buf[3]; cur_spt = (uint8_t)id_buf[6];
                                        selection_valid = true;
                                    } 
                                    else if (geo_idx == 1) { 
                                        use_lba_mode = false;
                                        get_large_geometry(id_buf[1], (uint8_t)id_buf[3], (uint8_t)id_buf[6], &cur_cyls, &cur_heads);
                                        cur_spt = (uint8_t)id_buf[6]; selection_valid = true;
                                    } 
                                    else if (geo_idx == 2 && drive_supports_lba) { 
                                        use_lba_mode = true;
                                        cur_cyls = id_buf[1]; cur_heads = (uint8_t)id_buf[3]; cur_spt = (uint8_t)id_buf[6];
                                        uint64_t id_lba48 = ((uint64_t)id_buf[103] << 48) | ((uint64_t)id_buf[102] << 32) |
                                                           ((uint64_t)id_buf[101] << 16) | ((uint64_t)id_buf[100]);
                                        total_lba_sectors_from_identify = id_lba48;
                                        if (total_lba_sectors_from_identify == 0)
                                            total_lba_sectors_from_identify = id_buf[60] | ((uint32_t)id_buf[61] << 16);
                                        selection_valid = true;
                                    }
                                    else if (geo_idx == 3 && detect_cyls > 0) { 
                                        use_lba_mode = false;
                                        cur_cyls = detect_cyls; cur_heads = detect_heads; cur_spt = detect_spt;
                                        selection_valid = true;
                                    }

                                    if (selection_valid) {
                                        if (ide_set_geometry(cur_heads, cur_spt)) {
                                            drive_cylinders = cur_cyls; drive_heads = cur_heads; drive_spt = cur_spt;
                                            waiting_for_choice = false;
                                        }
                                    }
                                }
                            }
                        } else { 
                            strcpy(hdd_status_text, "\033[91;1mNo DRQ: Drive failed data"); 
                            show_detect_result = true; trigger_overlay = true; hdd_model_raw[0] = '\0';
                         }
                    } else { 
                        snprintf(hdd_status_text, sizeof(hdd_status_text), "\033[93;1mTimeout: Drive BSY");
                        show_detect_result = true; trigger_overlay = true;
                    }
                    needs_full_redraw = true; 
                }
                else if (main_selected == 4) { current_screen = SCREEN_CONFIRM; confirm_type = 0; trigger_overlay = true; needs_full_redraw = true; }
                else if (main_selected == 5) { current_screen = SCREEN_CONFIRM; confirm_type = 1; trigger_overlay = true; needs_full_redraw = true; }
            }
        } 
        else if (current_screen == SCREEN_FEATURES) {
            if (k == KEY_UP && feat_selected > 0) feat_selected--;
            else if (k == KEY_DOWN && feat_selected < 4) feat_selected++; // Max 4 for "Debug Mode"
            else if (k == KEY_ESC) current_screen = SCREEN_MAIN;
            else if (k == '+' || k == '-' || k == '=' || k == KEY_PGUP || k == KEY_PGDN || k == KEY_ENTER) {
                if (feat_selected == 0) drive_write_protected = !drive_write_protected;
                else if (feat_selected == 1) auto_mount = !auto_mount;
                else if (feat_selected == 2) iordy_pin = !iordy_pin;
                else if (feat_selected == 3) comp_timings = !comp_timings;
                else if (feat_selected == 4 && k == KEY_ENTER) {
                    current_screen = SCREEN_DEBUG;
                }
            }
            needs_full_redraw = true;
        }
    }
}

int main() {
    stdio_init_all();
    multicore_lockout_victim_init();
    tusb_init();
    ide_hw_init();
    load_defaults();
    load_config();

    if (auto_mount && cur_cyls > 0 && cur_heads > 0 && cur_spt > 0) {
        for(int i = 0; i < 50; i++) {
            tight_loop_contents();
            sleep_ms(100);
        }

        ide_reset_drive();
        if (ide_wait_until_ready(5000)) {
            ide_identify_drive();
            uint16_t id_buf[256];
            if (ide_get_identify_data(id_buf)) {
                for (int i = 0; i < 20; i++) {
                    uint16_t val = id_buf[27 + i];
                    hdd_model_raw[i * 2]     = (char)(val >> 8);
                    hdd_model_raw[i * 2 + 1] = (char)(val & 0xFF);
                }
                hdd_model_raw[40] = '\0';
                sanitize_identify_model(hdd_model_raw);

            }
        }

        if (use_lba_mode) {
            drive_cylinders = 0; 
            drive_heads = 0;
            drive_spt = 0;
        } else {
            if (ide_set_geometry(cur_heads, cur_spt)) {
                drive_cylinders = cur_cyls;
                drive_heads = cur_heads;
                drive_spt = cur_spt;
            }
        }
    }

    multicore_launch_core1(core1_entry);

    while (1) {
        tud_task(); 
    }
}