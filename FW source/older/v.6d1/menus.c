// ATAboy Menu System — runs on core 1
// All terminal I/O goes through cdc_printf/cdc_putchar/cdc_getchar_timeout_us
// which use pico SDK queues. Core 0 drains/fills these queues in cdc_task().
// NEVER call tud_task(), tud_cdc_*(), or any TinyUSB function from here.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>
#include "pico/stdlib.h"
#include "ide.h"
#include "config.h"
#include "pico/util/queue.h"

// ---------------------------------------------------------------------------
//  CDC queue I/O (queues owned by ATAboy.c)
// ---------------------------------------------------------------------------

extern queue_t cdc_tx_queue;
extern queue_t cdc_rx_queue;
extern volatile bool cdc_connected;
extern volatile bool is_mounted;
extern volatile bool media_changed_waiting;

static void cdc_putchar(char c) {
    queue_add_blocking(&cdc_tx_queue, &c);
}

static void cdc_puts(const char *s) {
    while (*s) cdc_putchar(*s++);
}

static void cdc_printf(const char *fmt, ...) {
    char buf[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    cdc_puts(buf);
}

static int cdc_getchar_timeout_us(uint64_t timeout_us) {
    char c;
    if (queue_try_remove(&cdc_rx_queue, &c)) return (uint8_t)c;
    absolute_time_t deadline = make_timeout_time_us(timeout_us);
    while (!time_reached(deadline)) {
        if (queue_try_remove(&cdc_rx_queue, &c)) return (uint8_t)c;
        busy_wait_us_32(100);
    }
    return PICO_ERROR_TIMEOUT;
}

static void cdc_flush(void) {
    sleep_ms(5);
}

// ---------------------------------------------------------------------------
//  Box-drawing helper — emit a UTF-8 box char repeated N times
// ---------------------------------------------------------------------------

static void emit_n(const char *utf8_char, int n) {
    for (int i = 0; i < n; i++) cdc_puts(utf8_char);
}

// Box-drawing character constants (UTF-8, 3 bytes each)
#define BOX_HH   "\xe2\x95\x90"  /* double horizontal */
#define BOX_HL   "\xe2\x94\x80"  /* single horizontal */
#define BOX_VH   "\xe2\x95\x91"  /* double vertical   */
#define BOX_VL   "\xe2\x94\x82"  /* single vertical   */
#define BOX_TL   "\xe2\x95\x94"  /* top-left double   */
#define BOX_TR   "\xe2\x95\x97"  /* top-right double  */
#define BOX_BL   "\xe2\x95\x9a"  /* bot-left double   */
#define BOX_BR   "\xe2\x95\x9d"  /* bot-right double  */
#define BOX_TD   "\xe2\x95\xa4"  /* T-down mixed      */
#define BOX_TU   "\xe2\x94\xb4"  /* T-up single       */
#define BOX_ML   "\xe2\x95\x9f"  /* left-T mixed      */
#define BOX_MR   "\xe2\x95\xa2"  /* right-T mixed     */
#define BOX_MLD  "\xe2\x95\xa0"  /* left-T double     */
#define BOX_MRD  "\xe2\x95\xa3"  /* right-T double    */
#define BOX_LTD  "\xe2\x94\x9c"  /* left-T single     */
#define BOX_ARROW "\xe2\x96\xba" /* right arrow       */
#define BOX_ARRU "\xe2\x86\x91"  /* up arrow          */
#define BOX_ARRD "\xe2\x86\x93"  /* down arrow        */
#define BOX_ARRR "\xe2\x86\x92"  /* right arrow thin  */
#define BOX_ARRL "\xe2\x86\x90"  /* left arrow thin   */

// ---------------------------------------------------------------------------
//  State
// ---------------------------------------------------------------------------

static char hdd_model_raw[48] = "";
static char hdd_status_text[80] = "";
static uint16_t detect_cyls = 0;
static uint8_t  detect_heads = 0;
static uint8_t  detect_spt = 0;
static uint16_t cur_cyls = 0;
static uint8_t  cur_heads = 0;
static uint8_t  cur_spt = 0;
static uint64_t total_lba_sectors = 0;
static bool     use_lba_mode = false;

static bool show_detect_result = false;
static int  confirm_type = 0;

#define RESET       "\033[0m"
#define BG_BLUE     "\033[44m"
#define FG_WHITE    "\033[37;1m"
#define FG_RED      "\033[91m"
#define FG_GREEN    "\033[92m"
#define FG_YELLOW   "\033[33;1m"
#define HIDE_CUR    "\033[?25l"
#define CLR_SCR     "\033[2J"
#define SEL_RED     "\033[41;37;1m"

#define KEY_UP    1001
#define KEY_DOWN  1002
#define KEY_LEFT  1003
#define KEY_RIGHT 1004
#define KEY_ENTER 13
#define KEY_ESC   27
#define KEY_F10   1010

typedef enum {
    SCREEN_MAIN,
    SCREEN_FEATURES,
    SCREEN_CONFIRM,
    SCREEN_MOUNTED,
    SCREEN_DEBUG
} screen_t;

static screen_t current_screen = SCREEN_MAIN;
static screen_t confirm_return_screen = SCREEN_MAIN;
static bool needs_full_redraw = true;
static bool last_cdc_connected = false;

// ---------------------------------------------------------------------------
//  Helpers
// ---------------------------------------------------------------------------

static void draw_at(int x, int y, const char *text) {
    cdc_printf("\033[%d;%dH%s", y, x, text);
}

static int visible_strlen(const char *s) {
    int len = 0;
    while (*s) {
        if (*s == '\033') {
            s++;
            if (*s == '[') { s++; while (*s && (*s < '@' || *s > '~')) s++; if (*s) s++; }
        } else { len++; s++; }
    }
    return len;
}

static void sanitize_identify_model(char *s) {
    int len = strlen(s);
    while (len > 0 && s[len - 1] == ' ') s[--len] = '\0';
}

static void decode_ata_string(uint16_t *buf, int offset, int len_words, char *out) {
    int idx = 0;
    for (int i = 0; i < len_words; i++) {
        uint16_t w = buf[offset + i];
        out[idx++] = (char)(w >> 8);
        out[idx++] = (char)(w & 0xFF);
    }
    out[idx] = '\0';
    while (idx > 0 && out[idx - 1] == ' ') out[--idx] = '\0';
}

static void get_large_geometry(uint16_t n_cyl, uint8_t n_hd, uint8_t n_spt,
                               uint16_t *l_cyl, uint8_t *l_hd) {
    uint32_t total = (uint32_t)n_cyl * n_hd;
    uint8_t heads = n_hd;
    while (heads < 255 && (total / heads) > 65535) heads++;
    *l_hd = heads;
    *l_cyl = (uint16_t)(total / heads);
}

// ---------------------------------------------------------------------------
//  Help text display (right panel)
// ---------------------------------------------------------------------------

static void print_help(const char *text) {
    int start_row = 6, col = 57, width = 21, max_rows = 11;
    for (int i = 0; i < max_rows; i++)
        cdc_printf("\033[%d;%dH%*s", start_row + i, col, width, "");
    cdc_printf("\033[%d;%dH", start_row, col);
    int current_row = start_row, line_pos = 0, i = 0;
    while (text[i] && (current_row - start_row) < max_rows) {
        int word_len = 0;
        while (text[i + word_len] && text[i + word_len] != ' ' && text[i + word_len] != '\n') word_len++;
        if (line_pos + word_len > width && line_pos > 0) {
            current_row++;
            if ((current_row - start_row) >= max_rows) break;
            line_pos = 0;
            cdc_printf("\033[%d;%dH", current_row, col);
        }
        for (int j = 0; j < word_len; j++) { cdc_putchar(text[i++]); line_pos++; }
        if (text[i] == '\n') {
            current_row++;
            if ((current_row - start_row) >= max_rows) break;
            line_pos = 0;
            cdc_printf("\033[%d;%dH", current_row, col);
            i++;
        } else if (text[i] == ' ') {
            if (line_pos < width) { cdc_putchar(' '); line_pos++; }
            i++;
        }
    }
}

// ---------------------------------------------------------------------------
//  BIOS Frame — 80 columns
//  Row 2 top, rows 3-17 content+divider, row 18 mid, 19-20 keys,
//  row 21 mid, 22-23 status, row 24 bottom
// ---------------------------------------------------------------------------

static void draw_bios_frame(void) {
    cdc_printf(BG_BLUE FG_WHITE CLR_SCR HIDE_CUR "\033[H");
    cdc_printf("\033[1;1H%80s", "");
    cdc_printf("\033[1;10H" FG_WHITE "ATAboy Setup Utility v0.6d - Copyright (C) 2026 obsoletetech.us");

    bool feat = (current_screen == SCREEN_FEATURES);

    // Row 2: top border (80 visible chars)
    // Main:     1 + 38 + 1 + 39 + 1 = 80
    // Features: 1 + 53 + 1 + 24 + 1 = 80
    cdc_puts("\033[2;1H");
    cdc_puts(BOX_TL);
    if (!feat) { emit_n(BOX_HH, 38); cdc_puts(BOX_TD); emit_n(BOX_HH, 39); }
    else       { emit_n(BOX_HH, 53); cdc_puts(BOX_TD); emit_n(BOX_HH, 24); }
    cdc_puts(BOX_TR);

    // Rows 3-23: side pipes + vertical divider
    for (int i = 3; i < 24; i++) {
        cdc_printf("\033[%d;1H" BOX_VH, i);
        cdc_printf("\033[%d;80H" BOX_VH, i);
        if (i < 18) {
            if (!feat) cdc_printf("\033[%d;40H" BOX_VL, i);
            else       cdc_printf("\033[%d;55H" BOX_VL, i);
        }
    }

    // Row 18: mid border (80 visible chars)
    // Main:     1 + 38 + 1 + 39 + 1 = 80
    // Features: 1 + 53 + 1 + 24 + 1 = 80
    cdc_puts("\033[18;1H");
    cdc_puts(BOX_ML);
    if (!feat) { emit_n(BOX_HL, 38); cdc_puts(BOX_TU); emit_n(BOX_HL, 39); }
    else       { emit_n(BOX_HL, 53); cdc_puts(BOX_TU); emit_n(BOX_HL, 24); }
    cdc_puts(BOX_MR);

    // Row 21: full-width mid border  1 + 78 + 1 = 80
    cdc_puts("\033[21;1H");
    cdc_puts(BOX_ML); emit_n(BOX_HL, 78); cdc_puts(BOX_MR);

    // Row 24: bottom border  1 + 78 + 1 = 80
    cdc_puts("\033[24;1H");
    cdc_puts(BOX_BL); emit_n(BOX_HH, 78); cdc_puts(BOX_BR);
}

// ---------------------------------------------------------------------------
//  HDD Status (rows 22-23) — shared by main + features screens
// ---------------------------------------------------------------------------

static void draw_hdd_status(void) {
    const char *display;
    bool is_error = false;
    if (show_detect_result && hdd_status_text[0]) { display = hdd_status_text; is_error = true; }
    else if (hdd_model_raw[0]) { display = hdd_model_raw; }
    else { display = "IDENT not run yet"; }

    int label_len = 13;
    int text_len = visible_strlen(display);
    int start_x = (80 - (label_len + text_len)) / 2;

    draw_at(start_x, 22, FG_WHITE "Current HDD: ");
    if (is_error)        cdc_printf("\033[91;1m%s" RESET BG_BLUE FG_WHITE "\033[K", display);
    else if (hdd_model_raw[0]) cdc_printf("\033[32;1m%s" RESET BG_BLUE FG_WHITE "\033[K", display);
    else                 cdc_printf(FG_YELLOW "%s" RESET BG_BLUE FG_WHITE "\033[K", display);

    cdc_puts(RESET BG_BLUE FG_WHITE "\033[K");
    cdc_puts("\033[22;78H  " BOX_VH);

    char geo_vals[64];
    if (use_lba_mode) {
        uint32_t mb = (uint32_t)((uint64_t)total_lba_sectors * 512 / 1048576);
        snprintf(geo_vals, sizeof(geo_vals), "LBA Mode Active (%lu MB)", (unsigned long)mb);
    } else {
        snprintf(geo_vals, sizeof(geo_vals), "%d Cyl / %d Hd / %d SPT", cur_cyls, cur_heads, cur_spt);
    }

    int geo_total_len = 18 + strlen(geo_vals);
    int geo_x = (80 - geo_total_len) / 2;
    draw_at(geo_x, 23, FG_WHITE "Current Geometry: ");

    bool is_valid = (use_lba_mode && total_lba_sectors > 0) ||
                    (!use_lba_mode && cur_cyls > 0 && cur_heads > 0 && cur_spt > 0);
    cdc_printf(is_valid ? "\033[92;1m%s" : "\033[91;1m%s", geo_vals);

    cdc_puts(RESET BG_BLUE FG_WHITE "\033[K");
    cdc_puts("\033[23;78H  " BOX_VH);
    cdc_puts("\033[24;79H");
    cdc_flush();
}

// ---------------------------------------------------------------------------
//  Main Menu
// ---------------------------------------------------------------------------

static void update_main_menu(void) {
    const char *items[] = {
        BOX_ARROW " ATAboy Features Setup",
        "  Mount HDD to USB Mass Storage",
        "  Auto Detect & Set Geometry",
        "  Load Setup Defaults",
        "  Save Setup to EEPROM"
    };

    for (int i = 0; i < 5; i++) {
        int col = (i < 3) ? 4 : 43;
        int row = 6 + (i % 3) * 2;
        cdc_printf("\033[%d;%dH", row, col);
        if (i == config.main_selected)
            cdc_printf(SEL_RED " %-28s " RESET BG_BLUE FG_WHITE, items[i]);
        else
            cdc_printf(FG_YELLOW " %-28s  " FG_WHITE, items[i]);
    }

    cdc_printf("\033[19;3H ESC: Quit to Main Menu                         "
               BOX_ARRU " " BOX_ARRD " " BOX_ARRR " " BOX_ARRL ": Select Item");
    cdc_puts("\033[20;3H F10: Save Current Setup to EEPROM               Enter: Select");

    draw_hdd_status();
}

// ---------------------------------------------------------------------------
//  Features Menu
// ---------------------------------------------------------------------------

static void update_features_menu(void) {
    const char *labels[] = {"Write Protect", "Auto Mount at Start", "IORDY", "INTRQ", "Debug Mode"};
    const char *helps[] = {
        "Prevents any write commands from reaching the HDD.",
        "Automatically mounts the drive to USB on power-up sequence.",
        "Enables hardware IORDY (pin 27) flow control on the IDE bus.  Toggling this may help with picky drives.",
        "Enables hardware INTRQ (pin 28) for faster IDE command completion.  Toggling this may help with picky drives.",
        "Open low-level drive diagnostics and register status screen."
    };

    cdc_puts("\033[3;63H" FG_WHITE "Item Help");
    // col 55: single-T left + 24 dashes + mixed-T right = 26 chars
    cdc_puts("\033[4;55H" BOX_LTD);
    emit_n(BOX_HL, 24);
    cdc_puts(BOX_MR);

    for (int i = 0; i < 5; i++) {
        int row = 4 + i;
        cdc_printf("\033[%d;4H" FG_WHITE "%-25s", row, labels[i]);
        cdc_printf("\033[%d;35H" FG_YELLOW "[", row);
        cdc_puts(i == config.feat_selected ? SEL_RED : FG_YELLOW);

        if (i == 0)      cdc_printf("%-8s", config.drive_write_protected ? "Enabled" : "Disabled");
        else if (i == 1) cdc_printf("%-8s", config.auto_mount ? "Enabled" : "Disabled");
        else if (i == 2) cdc_printf("%-8s", config.iordy_enabled ? "Enabled" : "Disabled");
        else if (i == 3) cdc_printf("%-8s", config.intrq_enabled ? "Enabled" : "Disabled");
        else if (i == 4) cdc_printf("%-8s", "Enter");

        cdc_puts(RESET BG_BLUE FG_WHITE "]");
        if (i == config.feat_selected) print_help(helps[i]);
    }
    cdc_printf("\033[19;3H ESC: Quit to Main Menu                         "
               BOX_ARRU " " BOX_ARRD ": Select Item");
    cdc_puts("\033[20;3H F10: Save Current Setup to EEPROM               Enter: Select");

    draw_hdd_status();
}

// ---------------------------------------------------------------------------
//  Input handler
// ---------------------------------------------------------------------------

static int get_input(void) {
    int c = cdc_getchar_timeout_us(100000);
    if (c == PICO_ERROR_TIMEOUT) return -1;
    if (c == 27) {
        int n1 = cdc_getchar_timeout_us(10000);
        if (n1 == '[') {
            int n2 = cdc_getchar_timeout_us(10000);
            if (n2 == 'A') return KEY_UP;
            if (n2 == 'B') return KEY_DOWN;
            if (n2 == 'C') return KEY_RIGHT;
            if (n2 == 'D') return KEY_LEFT;
            if (n2 == '2') { if (cdc_getchar_timeout_us(10000) == '1') if (cdc_getchar_timeout_us(10000) == '~') return KEY_F10; }
            // Consume unrecognized CSI sequences (e.g. PgUp/PgDn) so they don't trigger ESC
            if (n2 >= '0' && n2 <= '9') {
                int ch;
                do { ch = cdc_getchar_timeout_us(10000); } while (ch != '~' && ch != PICO_ERROR_TIMEOUT);
                return -1;
            }
        }
        return KEY_ESC;
    }
    return c;
}

// ---------------------------------------------------------------------------
//  Confirm box — 52 wide, red bg + shadow
// ---------------------------------------------------------------------------

static void draw_confirm_box(const char *message) {
    int box_w = 52, start_col = 14, start_row = 10;

    // Shadow
    for (int i = 0; i < 5; i++) cdc_printf("\033[%d;%dH\033[40m%*s", start_row+1+i, start_col+2, box_w, "");
    // Red bg
    for (int i = 0; i < 5; i++) cdc_printf("\033[%d;%dH\033[41m%*s", start_row+i, start_col, box_w, "");

    cdc_puts(SEL_RED);
    cdc_printf("\033[%d;%dH", start_row, start_col);
    cdc_puts(BOX_TL); emit_n(BOX_HH, 50); cdc_puts(BOX_TR);

    cdc_printf("\033[%d;%dH" BOX_VH "%50s" BOX_VH, start_row+1, start_col, "");

    int msg_len = visible_strlen(message);
    if (msg_len > 50) msg_len = 50;
    int pl = (50 - msg_len) / 2, pr = 50 - msg_len - pl;
    cdc_printf("\033[%d;%dH" BOX_VH "%*s%s%*s" BOX_VH, start_row+2, start_col, pl, "", message, pr, "");

    cdc_printf("\033[%d;%dH" BOX_VH "%50s" BOX_VH, start_row+3, start_col, "");

    cdc_printf("\033[%d;%dH", start_row+4, start_col);
    cdc_puts(BOX_BL); emit_n(BOX_HH, 50); cdc_puts(BOX_BR);
    cdc_puts(RESET);
}

// ---------------------------------------------------------------------------
//  Error box — 52 wide, 7 tall
// ---------------------------------------------------------------------------

static void draw_error_box(const char *error_msg) {
    int box_w = 52, start_col = 14, start_row = 9;

    for (int i = 0; i < 7; i++) cdc_printf("\033[%d;%dH\033[40m%*s", start_row+1+i, start_col+2, box_w, "");
    for (int i = 0; i < 7; i++) cdc_printf("\033[%d;%dH\033[41m%*s", start_row+i, start_col, box_w, "");

    cdc_puts(SEL_RED);
    cdc_printf("\033[%d;%dH", start_row, start_col);
    cdc_puts(BOX_TL); emit_n(BOX_HH, 50); cdc_puts(BOX_TR);

    cdc_printf("\033[%d;%dH" BOX_VH "                      ERROR!                      " BOX_VH, start_row+1, start_col);

    cdc_printf("\033[%d;%dH", start_row+2, start_col);
    cdc_puts(BOX_MLD); emit_n(BOX_HH, 50); cdc_puts(BOX_MRD);

    cdc_printf("\033[%d;%dH" BOX_VH "%50s" BOX_VH, start_row+3, start_col, "");
    int vl = visible_strlen(error_msg);
    int xo = (50 - vl) / 2;
    cdc_printf("\033[%d;%dH\033[41;97m%s", start_row+3, start_col+1+xo, error_msg);

    cdc_puts(SEL_RED);
    cdc_printf("\033[%d;%dH" BOX_VH "%50s" BOX_VH, start_row+4, start_col, "");

    cdc_printf("\033[%d;%dH" BOX_VH "                       ", start_row+5, start_col);
    cdc_puts("\033[103;30m[OK]\033[0m");
    cdc_puts(SEL_RED "                       " BOX_VH);

    cdc_printf("\033[%d;%dH", start_row+6, start_col);
    cdc_puts(BOX_BL); emit_n(BOX_HH, 50); cdc_puts(BOX_BR);
    cdc_puts(RESET);
    cdc_flush();
}

// ---------------------------------------------------------------------------
//  Geometry selection overlay — 64 wide, 14 tall
// ---------------------------------------------------------------------------

static void draw_selection_menu(uint16_t *id, int selected_idx) {
    int box_w = 64, start_col = 9, start_row = 5;

    uint16_t n_cyl = id[1];
    uint8_t  n_hd  = (uint8_t)id[3];
    uint8_t  n_spt = (uint8_t)id[6];

    bool lba_supp = (id[49] & 0x0200);
    uint32_t lba28 = id[60] | ((uint32_t)id[61] << 16);
    uint64_t lba48 = ((uint64_t)id[103] << 48) | ((uint64_t)id[102] << 32) |
                     ((uint64_t)id[101] << 16) | ((uint64_t)id[100]);
    uint64_t lba_total = lba48 ? lba48 : (uint64_t)lba28;

    uint16_t l_cyl; uint8_t l_hd;
    get_large_geometry(n_cyl, n_hd, n_spt, &l_cyl, &l_hd);

    uint32_t n_size = (uint32_t)(((uint64_t)n_cyl * n_hd * n_spt * 512) / 1048576);
    uint32_t l_size = (uint32_t)(((uint64_t)l_cyl * l_hd * n_spt * 512) / 1048576);
    uint32_t lba_size = (uint32_t)((lba_total * 512ULL) / 1048576ULL);
    bool lba48_supp = ((id[83] & (1 << 10)) != 0) || (lba_total > 0x0FFFFFFFULL);

    // Shadow + bg
    for (int i = 0; i < 14; i++) cdc_printf("\033[%d;%dH\033[40m%*s", start_row+1+i, start_col+2, box_w, "");
    for (int i = 0; i < 14; i++) cdc_printf("\033[%d;%dH\033[41m%*s", start_row+i, start_col, box_w, "");

    cdc_puts(SEL_RED);

    // Row 0: 1+62+1 = 64
    cdc_printf("\033[%d;%dH", start_row, start_col);
    cdc_puts(BOX_TL); emit_n(BOX_HH, 62); cdc_puts(BOX_TR);

    // Row 1: model name centered
    int interior_w = 62;
    int vis_model = strlen(hdd_model_raw);
    int title_x = start_col + 1 + ((interior_w - (14 + vis_model)) / 2);
    cdc_printf("\033[%d;%dH%*s", start_row+1, start_col+1, interior_w, "");
    cdc_printf("\033[%d;%dH" BOX_VH, start_row+1, start_col);
    cdc_printf("\033[%d;%dHDetected HDD: \033[32;1m%s\033[0m", start_row+1, title_x, hdd_model_raw);
    cdc_printf(SEL_RED "\033[%d;%dH" BOX_VH, start_row+1, start_col+63);

    cdc_printf("\033[%d;%dH" BOX_VH "                Select Drive Geometry Option:                 " BOX_VH, start_row+2, start_col);

    // Row 3: double separator
    cdc_printf("\033[%d;%dH", start_row+3, start_col);
    cdc_puts(BOX_MLD); emit_n(BOX_HH, 62); cdc_puts(BOX_MRD);

    cdc_printf("\033[%d;%dH" BOX_VH "  MODE         SIZE       CYLS      HEADS      SPT            " BOX_VH, start_row+4, start_col);

    // Row 5: single-line separator
    cdc_printf("\033[%d;%dH" BOX_VH " ", start_row+5, start_col);
    emit_n(BOX_HL, 60);
    cdc_puts(" " BOX_VH);

    const char *modes[] = {"NORMAL  ", "LARGE   ", "LBA     ", "MANUAL  "};
    const char *lba_label = lba48_supp ? "LBA48  " : "LBA     ";

    for (int i = 0; i < 4; i++) {
        cdc_printf("\033[%d;%dH" BOX_VH " ", start_row+6+i, start_col);
        bool is_lba = (i == 2), lba_na = (is_lba && !lba_supp);

        if (i == selected_idx) {
            if (lba_na) cdc_printf("\033[41;33m %-8s \033[0m" SEL_RED, is_lba ? lba_label : modes[i]);
            else        cdc_printf("\033[103;30m %-8s \033[0m" SEL_RED, is_lba ? lba_label : modes[i]);
        } else {
            if (lba_na) cdc_printf("\033[90m %-8s ", is_lba ? lba_label : modes[i]);
            else        cdc_printf(FG_WHITE " %-8s ", is_lba ? lba_label : modes[i]);
        }

        if (lba_na) {
            cdc_puts("\033[90m      --- MB    (Not Supported by Drive)           \033[0m" SEL_RED);
        } else if (i == 0) {
            cdc_printf("   %4lu MB    %-5u       %-3u       %-3u            ", (unsigned long)n_size, n_cyl, n_hd, n_spt);
        } else if (i == 1) {
            cdc_printf("   %4lu MB    %-5u       %-3u       %-3u            ", (unsigned long)l_size, l_cyl, l_hd, n_spt);
        } else if (i == 2) {
            const char *bits = lba48_supp ? "48-bit" : "28-bit";
            if (lba_size > 9999) cdc_printf("   %4lu GB           LBA (%s)                  ", (unsigned long)(lba_size/1024), bits);
            else                 cdc_printf("   %4lu MB           LBA (%s)                  ", (unsigned long)lba_size, bits);
        } else if (i == 3) {
            if (detect_cyls > 0) {
                uint32_t ms = (uint32_t)((uint64_t)detect_cyls * detect_heads * detect_spt * 512 / 1048576);
                cdc_printf("   %4lu MB    %-5u       %-3u       %-3u            ", (unsigned long)ms, detect_cyls, detect_heads, detect_spt);
            } else cdc_puts("   ---- MB    -----       ---       ---            ");
        }
        cdc_puts(BOX_VH);
    }

    // Row 10
    cdc_printf("\033[%d;%dH", start_row+10, start_col);
    cdc_puts(BOX_MLD); emit_n(BOX_HH, 62); cdc_puts(BOX_MRD);

    cdc_printf("\033[%d;%dH" BOX_VH "  " BOX_ARRU " " BOX_ARRD ": Mode    TAB: Change CHS     Enter: Select    Esc: Quit " BOX_VH, start_row+11, start_col);
    cdc_printf("\033[%d;%dH" BOX_VH "  \033[33m   LBA Recommended for modern drives; NORMAL for legacy.    \033[0m" SEL_RED BOX_VH, start_row+12, start_col);

    // Row 13: bottom
    cdc_printf("\033[%d;%dH", start_row+13, start_col);
    cdc_puts(BOX_BL); emit_n(BOX_HH, 62); cdc_puts(BOX_BR);

    cdc_puts(RESET);
    cdc_flush();
}

// ---------------------------------------------------------------------------
//  Debug mode — 72 wide, 20 tall overlay
// ---------------------------------------------------------------------------

static void debug_print(int line_idx, const char *color, const char *fmt, ...) {
    if (line_idx > 16) return;
    char buf[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    cdc_printf("\033[%d;%dH\033[40m%s%-*s", 4 + line_idx, 7, color, 68, buf);
    cdc_puts(RESET);
}

static void debug_cls(void) { for (int i = 0; i < 17; i++) debug_print(i, FG_WHITE, ""); }

static void draw_debug_overlay(void) {
    int box_w = 72, box_h = 20, sc = 5, sr = 3;
    int ix = sc+2, iy = sr+1, iw = box_w-4, ih = box_h-3;

    cdc_puts(RESET);
    for (int i = 0; i < box_h; i++) cdc_printf("\033[%d;%dH\033[40m%*s", sr+1+i, sc+2, box_w, "");
    for (int i = 0; i < box_h; i++) cdc_printf("\033[%d;%dH\033[44m%*s", sr+i, sc, box_w, "");
    for (int i = 0; i < ih; i++)    cdc_printf("\033[%d;%dH\033[40m%*s", iy+i, ix, iw, "");

    cdc_puts(FG_WHITE BG_BLUE);
    // Top: 1+70+1=72
    cdc_printf("\033[%d;%dH", sr, sc);
    cdc_puts(BOX_TL); emit_n(BOX_HH, 70); cdc_puts(BOX_TR);
    for (int i = 1; i < box_h-1; i++) {
        cdc_printf("\033[%d;%dH" BOX_VH, sr+i, sc);
        cdc_printf("\033[%d;%dH" BOX_VH, sr+i, sc+box_w-1);
    }
    // Bottom
    cdc_printf("\033[%d;%dH", sr+box_h-1, sc);
    cdc_puts(BOX_BL); emit_n(BOX_HH, 70); cdc_puts(BOX_BR);

    draw_at(sc+29, sr, "[ Debug Mode ]");
    cdc_printf("\033[%d;%dH" FG_WHITE BG_BLUE " ESC: Return  I: IDENT  T: Task File  E: Errors  S: Seek  R: Reset", sr+box_h-2, sc+3);
    cdc_puts(RESET);
    cdc_flush();
}

static void run_debug_identify(void) {
    debug_cls();
    debug_print(0, FG_YELLOW, "Sending IDENTIFY DEVICE (0xEC)...");

    uint16_t id[256];
    if (!ide_identify(id)) {
        debug_print(1, "\033[91;1m", "ERROR: Drive not ready or no data from IDENTIFY.");
        debug_print(2, FG_WHITE, "Check cabling, Master/Slave jumper, or power.");
        return;
    }

    char model[41], serial[21], fw[9];
    decode_ata_string(id, 27, 20, model);
    decode_ata_string(id, 10, 10, serial);
    decode_ata_string(id, 23, 4, fw);

    debug_print(0, "\033[92;1m", "IDENTIFY SUCCESSFUL");
    debug_print(2, FG_WHITE, "Model:  \033[96m%s", model);
    debug_print(3, FG_WHITE, "Serial: \033[96m%s", serial);
    debug_print(4, FG_WHITE, "FW Rev: \033[96m%s", fw);
    debug_print(5, FG_WHITE, "--------------------------------------------------");
    debug_print(6, FG_YELLOW, "[CHS Geometry]");
    debug_print(7, FG_WHITE, "Cyls: %-5d  Heads: %-3d  Sectors: %-3d", id[1], id[3], id[6]);

    bool lba_supp = (id[49] & 0x0200), lba48_supp = (id[83] & (1<<10));
    uint32_t lba28_cap = id[60] | ((uint32_t)id[61] << 16);
    uint64_t lba48_cap = ((uint64_t)id[103]<<48)|((uint64_t)id[102]<<32)|((uint64_t)id[101]<<16)|((uint64_t)id[100]);

    debug_print(9, FG_YELLOW, "[Capabilities]");
    debug_print(10, FG_WHITE, "LBA Supported:    %s", lba_supp ? "\033[92mYes" : "\033[91mNo");
    debug_print(11, FG_WHITE, "LBA48 Supported:  %s", lba48_supp ? "\033[92mYes" : "\033[91mNo");
    if (lba48_supp) {
        uint32_t gb = (uint32_t)(((uint64_t)lba48_cap*512)/1000000000);
        debug_print(12, FG_WHITE, "Capacity:         %lu Sectors (~%lu GB)", (unsigned long)lba48_cap, (unsigned long)gb);
    } else if (lba_supp) {
        uint32_t mb = (uint32_t)(((uint64_t)lba28_cap*512)/1000000);
        debug_print(12, FG_WHITE, "Capacity:         %lu Sectors (~%lu MB)", (unsigned long)lba28_cap, (unsigned long)mb);
    }
    debug_print(14, FG_YELLOW, "[Advanced]");
    debug_print(15, FG_WHITE, "DMA Support: %04X  PIO Support: %04X", id[49], id[64]);
    debug_print(16, FG_WHITE, "ATA Major Ver: %04X", id[80]);
}

static void run_debug_taskfile(void) {
    debug_cls();
    uint8_t tf[8];
    ide_read_taskfile(tf);
    const char *sc = (tf[7] & 0x81) ? "\033[91;1m" : "\033[92m";
    debug_print(0, FG_RED, "[Task File] " FG_WHITE "ERR:%02X SEC:%02X SN:%02X CL:%02X CH:%02X DH:%02X ST:%s%02X" RESET,
                tf[1], tf[2], tf[3], tf[4], tf[5], tf[6], sc, tf[7]);
}

static void run_debug_errors(void) {
    debug_cls();
    uint8_t tf[8];
    ide_read_taskfile(tf);
    uint8_t err = tf[1];
    char eb[64] = {0};
    if (!err) snprintf(eb, sizeof(eb), "\033[92mNo Errors Reported\033[0m");
    else {
        if (err&0x80) strcat(eb,"BBK "); if (err&0x40) strcat(eb,"UNC ");
        if (err&0x20) strcat(eb,"MC ");  if (err&0x10) strcat(eb,"IDNF ");
        if (err&0x08) strcat(eb,"MCR "); if (err&0x04) strcat(eb,"ABRT ");
        if (err&0x02) strcat(eb,"TK0 "); if (err&0x01) strcat(eb,"AMNF ");
    }
    debug_print(0, FG_RED, "[Error Bits] %s", eb);
}

static void run_seek_test(void) {
    debug_cls();
    debug_print(0, FG_YELLOW, "Seek Test Running...");

    uint16_t id[256];
    if (!ide_identify(id)) { debug_print(1, FG_RED, "ERROR: Drive not ready."); sleep_ms(1500); return; }

    bool lba = (id[49] & 0x0200);
    uint32_t max_range = lba ? (id[60]|((uint32_t)id[61]<<16)) : id[1];
    if (!max_range) max_range = 1024;

    double x = 0;
    while (true) {
        if (cdc_getchar_timeout_us(0) == 27) break;
        double pos = (sin(x)*0.45)+0.5;
        uint32_t target = (uint32_t)(pos*(max_range-1));

        uint8_t st = ide_seek_read_one(target, lba);

        char bar[61]; memset(bar, '-', 60); bar[(int)(pos*59)] = '#'; bar[60] = '\0';
        debug_print(14, FG_GREEN, "[%s]", bar);
        debug_print(15, FG_WHITE, "Target %s: %lu (ST:%02X)", lba?"LBA":"CYL", (unsigned long)target, st);
        x += 0.12;
    }
    debug_print(14, FG_WHITE, "%*s", 60, ""); debug_print(15, FG_WHITE, "%*s", 60, "");
    sleep_ms(50);
}

// ---------------------------------------------------------------------------
//  Sync local state to/from config_t
// ---------------------------------------------------------------------------

static void sync_from_config(void) {
    cur_cyls = config.cyls; cur_heads = config.heads; cur_spt = config.spt;
    use_lba_mode = config.use_lba_mode; total_lba_sectors = config.lba_sectors;
}

static void sync_to_config(void) {
    config.cyls = cur_cyls; config.heads = cur_heads; config.spt = cur_spt;
    config.use_lba_mode = use_lba_mode; config.lba_sectors = total_lba_sectors;
}

// ---------------------------------------------------------------------------
//  Auto-mount — runs on core 1 so IDE ops never block USB on core 0
// ---------------------------------------------------------------------------

static void try_auto_mount(void) {
    if (!config.auto_mount) return;

    bool has_geo = (config.use_lba_mode && config.lba_sectors > 0) ||
                   (!config.use_lba_mode && config.cyls > 0 && config.heads > 0 && config.spt > 0);
    if (!has_geo) return;

    // Wait for drive to spin up
    sleep_ms(5000);

    ide_reset_drive();

    // Poll drive ready
    uint32_t start = to_ms_since_boot(get_absolute_time());
    bool ready = false;
    while (to_ms_since_boot(get_absolute_time()) - start < 5000) {
        uint8_t st = ide_read_reg(7);
        if (!(st & 0x80) && (st & 0x40)) { ready = true; break; }
        sleep_ms(10);
    }
    if (!ready) return;

    uint16_t id_buf[256];
    if (!ide_identify(id_buf)) return;

    if (!config.use_lba_mode)
        ide_set_geometry(config.heads, config.spt);

    is_mounted = true;
    media_changed_waiting = true;
}

// ---------------------------------------------------------------------------
//  Core 1 entry point
// ---------------------------------------------------------------------------

void core1_entry(void) {
    sync_from_config();
    try_auto_mount();
    bool trigger_overlay = false;

    // If auto-mount succeeded, start on the mounted screen
    if (is_mounted) {
        current_screen = SCREEN_MOUNTED;
        trigger_overlay = true;
        needs_full_redraw = true;
    }

    while (true) {
        bool connected = cdc_connected;
        if (connected && !last_cdc_connected) { sleep_ms(200); needs_full_redraw = true; }
        last_cdc_connected = connected;
        if (!connected) { sleep_ms(100); continue; }

        if (needs_full_redraw) {
            draw_bios_frame();
            if (current_screen == SCREEN_MAIN || current_screen == SCREEN_CONFIRM || current_screen == SCREEN_MOUNTED)
                update_main_menu();
            else if (current_screen == SCREEN_FEATURES) update_features_menu();
            else if (current_screen == SCREEN_DEBUG) draw_debug_overlay();

            if (show_detect_result || current_screen == SCREEN_CONFIRM ||
                current_screen == SCREEN_MOUNTED || current_screen == SCREEN_DEBUG)
                trigger_overlay = true;
            needs_full_redraw = false;
        }

        if (!show_detect_result && current_screen != SCREEN_CONFIRM &&
            current_screen != SCREEN_MOUNTED && current_screen != SCREEN_DEBUG) {
            if (current_screen == SCREEN_MAIN) update_main_menu();
            else if (current_screen == SCREEN_FEATURES) update_features_menu();
        }

        if (show_detect_result) {
            if (trigger_overlay) { draw_error_box(hdd_status_text); trigger_overlay = false; }
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
                draw_confirm_box("Drive mounted!  Press 'U' to Unmount");
                trigger_overlay = false;
            }
        } else if (current_screen == SCREEN_DEBUG) {
            if (trigger_overlay) { draw_debug_overlay(); trigger_overlay = false; }
        }

        cdc_puts("\033[24;79H"); cdc_flush();
        int k = get_input();
        if (k == -1) { tight_loop_contents(); continue; }

        if (show_detect_result) {
            if (k == KEY_ENTER || k == KEY_ESC) {
                show_detect_result = false; hdd_status_text[0] = '\0'; hdd_model_raw[0] = '\0';
                cur_cyls = 0; cur_heads = 0; cur_spt = 0;
                total_lba_sectors = 0; use_lba_mode = false; is_mounted = false;
                sync_to_config(); needs_full_redraw = true;
            }
            continue;
        }

        if (current_screen == SCREEN_MOUNTED) {
            if (k == 'u' || k == 'U') { current_screen = SCREEN_CONFIRM; confirm_type = 4; trigger_overlay = true; needs_full_redraw = true; }
            continue;
        }

        if (current_screen == SCREEN_DEBUG) {
            if (k == KEY_ESC) { current_screen = SCREEN_FEATURES; needs_full_redraw = true; }
            else if (k == 'i' || k == 'I') run_debug_identify();
            else if (k == 't' || k == 'T') run_debug_taskfile();
            else if (k == 'e' || k == 'E') run_debug_errors();
            else if (k == 's' || k == 'S') { run_seek_test(); current_screen = SCREEN_DEBUG; needs_full_redraw = true; }
            else if (k == 'r' || k == 'R') {
                debug_cls();
                debug_print(0, FG_YELLOW, "Resetting drive...");
                ide_reset_drive();
                bool rdy = ide_wait_until_ready(5000);
                debug_print(1, rdy ? FG_GREEN : FG_RED, rdy ? "Drive ready." : "Drive not responding.");
            }
            continue;
        }

        if (current_screen == SCREEN_CONFIRM) {
            if (k == 'y' || k == 'Y') {
                if (confirm_type == 0) { config_defaults(); sync_from_config(); config_save(); current_screen = SCREEN_MAIN; }
                else if (confirm_type == 1) { sync_to_config(); config_save(); current_screen = confirm_return_screen; }
                else if (confirm_type == 3) { is_mounted = true; media_changed_waiting = true; current_screen = SCREEN_MOUNTED; }
                else if (confirm_type == 4) { is_mounted = false; media_changed_waiting = true; current_screen = SCREEN_MAIN; }
                needs_full_redraw = true;
            } else if (k == 'n' || k == 'N' || k == KEY_ESC) {
                current_screen = (confirm_type == 1) ? confirm_return_screen :
                                 (confirm_type == 4) ? SCREEN_MOUNTED : SCREEN_MAIN;
                needs_full_redraw = true;
            }
            continue;
        }

        if (k == KEY_F10) { confirm_return_screen = current_screen; current_screen = SCREEN_CONFIRM; confirm_type = 1; trigger_overlay = true; needs_full_redraw = true; continue; }

        if (current_screen == SCREEN_MAIN) {
            if (k == KEY_UP && config.main_selected % 3 > 0) config.main_selected--;
            else if (k == KEY_DOWN && config.main_selected % 3 < 2 && config.main_selected < 4) config.main_selected++;
            else if (k == KEY_RIGHT && config.main_selected < 3 && config.main_selected + 3 < 5) config.main_selected += 3;
            else if (k == KEY_LEFT && config.main_selected >= 3) config.main_selected -= 3;
            else if (k == KEY_ENTER) {
                if (config.main_selected == 0) {
                    current_screen = SCREEN_FEATURES; config.feat_selected = 0; needs_full_redraw = true;
                } else if (config.main_selected == 1) {
                    bool dv = (hdd_model_raw[0] != '\0');
                    bool gv = (use_lba_mode && total_lba_sectors > 0) || (!use_lba_mode && cur_cyls > 0 && cur_heads > 0 && cur_spt > 0);
                    if (!dv || !gv) {
                        snprintf(hdd_status_text, sizeof(hdd_status_text), "\033[91;1mDetect drive and set geometry first!");
                        hdd_model_raw[0] = '\0'; cur_cyls = 0; cur_heads = 0; cur_spt = 0;
                        total_lba_sectors = 0; use_lba_mode = false; is_mounted = false;
                        show_detect_result = true; trigger_overlay = true; needs_full_redraw = true;
                    } else { current_screen = SCREEN_CONFIRM; confirm_type = 3; trigger_overlay = true; needs_full_redraw = true; }
                } else if (config.main_selected == 2) {
                    // Auto Detect
                    ide_reset_drive();
                    if (ide_wait_until_ready(5000)) {
                        uint16_t id_buf[256];
                        if (ide_identify(id_buf)) {
                            for (int i = 0; i < 20; i++) {
                                uint16_t val = id_buf[27+i];
                                hdd_model_raw[i*2] = (char)(val>>8); hdd_model_raw[i*2+1] = (char)(val&0xFF);
                            }
                            hdd_model_raw[40] = '\0'; sanitize_identify_model(hdd_model_raw);

                            bool ls = (id_buf[49] & 0x0200);
                            int geo_idx = ls ? 2 : 0;
                            bool waiting = true, sd = true;

                            while (waiting) {
                                if (sd) { draw_selection_menu(id_buf, geo_idx); sd = false; }
                                int ch = get_input();
                                if (ch == -1) { tight_loop_contents(); continue; }

                                if (ch == KEY_UP && geo_idx > 0) { geo_idx--; if (!ls && geo_idx == 2) geo_idx--; sd = true; }
                                else if (ch == KEY_DOWN && geo_idx < 3) { geo_idx++; if (!ls && geo_idx == 2) geo_idx++; sd = true; }
                                else if (ch == KEY_ESC) { waiting = false; }
                                else if (ch == '\t' || (ch == KEY_ENTER && geo_idx == 3)) {
                                    geo_idx = 3; draw_selection_menu(id_buf, geo_idx);
                                    int mf[3] = {detect_cyls, detect_heads, detect_spt};
                                    int mc[3] = {9+26, 9+38, 9+48};
                                    for (int f = 0; f < 3; f++) {
                                        char ib[7] = {0}; int p = 0;
                                        draw_at(mc[f], 14, "\033[103;30m     \033[0m");
                                        cdc_printf("\033[%d;%dH", 14, mc[f]);
                                        while (true) {
                                            int c = get_input();
                                            if (c >= '0' && c <= '9' && p < 5) { ib[p++] = (char)c; cdc_putchar(c); }
                                            else if ((c == 8 || c == 127) && p > 0) { p--; cdc_printf("\b \b\033[%d;%dH", 14, mc[f]+p); }
                                            else if (c == KEY_ENTER || c == '\t') { ib[p] = '\0'; if (p > 0) mf[f] = atoi(ib); draw_at(mc[f], 14, RESET "\033[41;37;1m"); cdc_printf("%-5d", mf[f]); break; }
                                            else if (c == KEY_ESC) { f = 3; break; }
                                            tight_loop_contents();
                                        }
                                    }
                                    detect_cyls = (uint16_t)mf[0]; detect_heads = (uint8_t)mf[1]; detect_spt = (uint8_t)mf[2];
                                    if (detect_cyls > 0 && detect_heads > 0 && detect_spt > 0) {
                                        use_lba_mode = false;
                                        cur_cyls = detect_cyls; cur_heads = detect_heads; cur_spt = detect_spt;
                                        ide_set_geometry(cur_heads, cur_spt);
                                        sync_to_config(); waiting = false;
                                    } else { sd = true; }
                                } else if (ch == KEY_ENTER) {
                                    bool valid = false;
                                    if (geo_idx == 0) { use_lba_mode = false; cur_cyls = id_buf[1]; cur_heads = (uint8_t)id_buf[3]; cur_spt = (uint8_t)id_buf[6]; valid = true; }
                                    else if (geo_idx == 1) { use_lba_mode = false; get_large_geometry(id_buf[1],(uint8_t)id_buf[3],(uint8_t)id_buf[6],&cur_cyls,&cur_heads); cur_spt = (uint8_t)id_buf[6]; valid = true; }
                                    else if (geo_idx == 2 && ls) {
                                        use_lba_mode = true; cur_cyls = id_buf[1]; cur_heads = (uint8_t)id_buf[3]; cur_spt = (uint8_t)id_buf[6];
                                        uint64_t l48 = ((uint64_t)id_buf[103]<<48)|((uint64_t)id_buf[102]<<32)|((uint64_t)id_buf[101]<<16)|((uint64_t)id_buf[100]);
                                        total_lba_sectors = l48; if (!total_lba_sectors) total_lba_sectors = id_buf[60]|((uint32_t)id_buf[61]<<16);
                                        valid = true;
                                    }

                                    if (valid) { if (!use_lba_mode) ide_set_geometry(cur_heads, cur_spt); sync_to_config(); waiting = false; }
                                }
                            }
                        } else { strcpy(hdd_status_text, "\033[91;1mNo DRQ: Drive failed data"); show_detect_result = true; trigger_overlay = true; hdd_model_raw[0] = '\0'; }
                    } else { snprintf(hdd_status_text, sizeof(hdd_status_text), "\033[93;1mTimeout: Drive BSY"); show_detect_result = true; trigger_overlay = true; }
                    needs_full_redraw = true;
                } else if (config.main_selected == 3) { current_screen = SCREEN_CONFIRM; confirm_type = 0; trigger_overlay = true; needs_full_redraw = true; }
                else if (config.main_selected == 4) { current_screen = SCREEN_CONFIRM; confirm_type = 1; trigger_overlay = true; needs_full_redraw = true; }
            }
        } else if (current_screen == SCREEN_FEATURES) {
            if (k == KEY_UP && config.feat_selected > 0) config.feat_selected--;
            else if (k == KEY_DOWN && config.feat_selected < 4) config.feat_selected++;
            else if (k == KEY_ESC) current_screen = SCREEN_MAIN;
            else if (k == KEY_ENTER) {
                if (config.feat_selected == 0) config.drive_write_protected = !config.drive_write_protected;
                else if (config.feat_selected == 1) config.auto_mount = !config.auto_mount;
                else if (config.feat_selected == 2) { config.iordy_enabled = !config.iordy_enabled; ide_set_iordy(config.iordy_enabled); }
                else if (config.feat_selected == 3) config.intrq_enabled = !config.intrq_enabled;
                else if (config.feat_selected == 4) current_screen = SCREEN_DEBUG;
            }
            needs_full_redraw = true;
        }
    }
}
