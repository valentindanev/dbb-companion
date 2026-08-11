/*
 * db_fc_flash - reflash the ArduPilot flight controller over USB OTG.
 * See db_fc_flash.h for the design rationale.
 *
 * THREE TRAPS, EACH OF WHICH COST A FAILED HARDWARE RUN ON 10-08-2026.
 * They are guarded here; do not "simplify" them away.
 *
 *  1. ArduPilot's bootloader CRC is NOT standard CRC-32. Tools/scripts/uploader.py
 *     uses the same polynomial and table but init 0 and NO final XOR - neither
 *     inversion. zlib/CRC-32 does both. For the 4.7.0 DAKEFPVH743 image:
 *         zlib      0x953873E6        ArduPilot 0xEE520CE1  <- what the FC says
 *     Three "failures" had actually written the firmware perfectly.
 *
 *  2. The command/response handshake must be a stream, not a shared
 *     length/flag pair. A raw_len/raw_want/raw_mode triple touched by both the
 *     USB callback and the flashing task raced and died at 81% of the transfer.
 *     A FreeRTOS stream buffer gives the strictly-ordered "read exactly N bytes
 *     with a deadline" model that pyserial gives the reference uploader, which
 *     is why the reference has no such bug. Reset it before each command; that
 *     is uploader.py's port.flushInput().
 *
 *  3. Always close the CDC handle when no disconnect event arrives. Looping to
 *     cdc_acm_host_open() with a live handle leaks the endpoints
 *     ("EP with 1 address already allocated") and wedges the app permanently.
 */

#include "db_fc_flash.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/stream_buffer.h"
#include "usb/usb_host.h"
#include "usb/cdc_acm_host.h"
#include "cJSON.h"

#include "db_mavlink_msgs.h"
#include "db_sonar_log.h"   /* shares the /logs FAT mount; reserve is dynamic */

/*
 * *** HARD COUPLING - fails silently if not caught here ***
 *
 * This module drives the ESP32-S3's USB OTG peripheral on GPIO19/20.
 * CONFIG_DB_SERIAL_OPTION_JTAG routes the flight-controller serial link through
 * USB-Serial-JTAG, which is the SAME peripheral on the SAME pins. They cannot
 * coexist: whichever initialises second gets a non-obviously broken link.
 *
 * The boat uses the UART transport on TX12/RX14 (proven in the hull 09-08-2026),
 * so this is not a conflict today - but it would become one silently the moment
 * somebody flipped that Kconfig option. Same reasoning as the
 * CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE #error in db_ota_policy.c.
 */
#ifdef CONFIG_DB_SERIAL_OPTION_JTAG
#error "db_fc_flash needs the USB OTG peripheral (GPIO19/20), but \
CONFIG_DB_SERIAL_OPTION_JTAG puts the FC serial link on USB-Serial-JTAG, which \
is the same peripheral and the same pins. Choose one: keep the FC on the UART \
transport (the boat's TX12/RX14 path) and flash over USB, or use USB-JTAG for \
serial and remove db_fc_flash.c from main/CMakeLists.txt."
#endif

static const char *TAG = "DB_FC_FLASH";

/* ---- the flight controller as a USB device ---- */
#define FC_VID              (0x1209)
#define FC_PID              (0x5741)

/* ---- PX4 / ArduPilot bootloader protocol ---- */
#define PROTO_OK            (0x10)
#define PROTO_INSYNC        (0x12)
#define PROTO_EOC           (0x20)
#define PROTO_GET_SYNC      (0x21)
#define PROTO_GET_DEVICE    (0x22)
#define PROTO_CHIP_ERASE    (0x23)
#define PROTO_PROG_MULTI    (0x27)
#define PROTO_GET_CRC       (0x29)
#define PROTO_BOOT          (0x30)

#define INFO_BL_REV         (1)
#define INFO_BOARD_ID       (2)
#define INFO_FLASH_SIZE     (4)

#define PROG_CHUNK          (240)     /* reference allows 252; 240 is proven */
#define CMD_TIMEOUT_MS      (2000)
#define ERASE_TIMEOUT_MS    (45000)   /* H743 mass erase measured at 8.6-10.6 s */

#define USB_HOST_PRIORITY   (20)
#define FLASH_TASK_STACK    (6144)
#define RX_STREAM_BYTES     (1024)

/*
 * COMMAND_LONG (76) / MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, param1 = 3 ("reboot
 * and stay in the bootloader"), from 250/190 to 1/1. Precomputed with pymavlink
 * so this module needs no MAVLink CRC path of its own. Confirmed working.
 *
 * NOTE: the 1200-baud touch does NOT reliably do this on ArduPilot/ChibiOS.
 * Do not replace this with a line-coding trick.
 */
static const uint8_t MAV_REBOOT_TO_BOOTLOADER[] = {
    0xFD, 0x20, 0x00, 0x00, 0x00, 0xFA, 0xBE, 0x4C, 0x00, 0x00, 0x00, 0x00,
    0x40, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0xF6, 0x00, 0x01, 0x01, 0xAB, 0x33,
};

/* ------------------------------------------------------------------ state */

static db_fc_flash_status_t g_status;
static SemaphoreHandle_t    g_status_lock;
static StreamBufferHandle_t g_rx_stream;
static SemaphoreHandle_t    g_disconnected;
static volatile bool        g_raw_mode;
static uint8_t              g_reply[16];
static volatile bool        g_busy;
static cdc_acm_dev_hdl_t    g_dev;

/* ------------------------------------------------------------ console ring */

#define CONSOLE_LINES 128
#define CONSOLE_LINE_MAX 120

static struct {
    char     line[CONSOLE_LINES][CONSOLE_LINE_MAX];
    uint32_t seq[CONSOLE_LINES];
    uint32_t next_seq;
    SemaphoreHandle_t lock;
} g_console;

void db_fc_flash_log(const char *fmt, ...)
{
    char buf[CONSOLE_LINE_MAX];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    ESP_LOGI(TAG, "%s", buf);

    if (!g_console.lock) {
        return;
    }
    xSemaphoreTake(g_console.lock, portMAX_DELAY);
    uint32_t slot = g_console.next_seq % CONSOLE_LINES;
    strlcpy(g_console.line[slot], buf, CONSOLE_LINE_MAX);
    g_console.seq[slot] = g_console.next_seq;
    g_console.next_seq++;
    xSemaphoreGive(g_console.lock);
}

/*
 * Copy every line newer than `since_seq` as NUL-separated strings. The page
 * polls this; a ring plus a sequence number means a slow or briefly
 * disconnected client resumes without duplicating or losing lines, and no
 * httpd worker is held open for the duration of a flash.
 */
size_t db_fc_flash_console_read(uint32_t since_seq, char *out, size_t out_len,
                                uint32_t *next_seq)
{
    size_t used = 0;
    if (!g_console.lock || out_len == 0) {
        if (next_seq) *next_seq = 0;
        return 0;
    }
    xSemaphoreTake(g_console.lock, portMAX_DELAY);

    uint32_t first = g_console.next_seq > CONSOLE_LINES
                         ? g_console.next_seq - CONSOLE_LINES : 0;
    if (since_seq < first) {
        since_seq = first;               /* client fell behind the ring */
    }
    for (uint32_t s = since_seq; s < g_console.next_seq; s++) {
        uint32_t slot = s % CONSOLE_LINES;
        if (g_console.seq[slot] != s) {
            continue;
        }
        size_t n = strlen(g_console.line[slot]);
        if (used + n + 2 > out_len) {
            break;
        }
        memcpy(out + used, g_console.line[slot], n);
        used += n;
        out[used++] = '\n';
    }
    out[used] = '\0';
    if (next_seq) *next_seq = g_console.next_seq;

    xSemaphoreGive(g_console.lock);
    return used;
}

/* ------------------------------------------------------- ArduPilot CRC-32 */

static uint32_t g_crctab[256];
static bool     g_crctab_ready;

static void ap_crc_init(void)
{
    if (g_crctab_ready) return;
    for (uint32_t i = 0; i < 256; i++) {
        uint32_t c = i;
        for (int k = 0; k < 8; k++) {
            c = (c & 1) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
        }
        g_crctab[i] = c;
    }
    g_crctab_ready = true;
}

/* init 0, no final XOR - see the trap note at the top of this file */
static uint32_t ap_crc32(uint32_t state, const uint8_t *buf, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        state = g_crctab[(state ^ buf[i]) & 0xFF] ^ (state >> 8);
    }
    return state;
}

/* --------------------------------------------------------------- status */

static void status_set_state(db_fc_flash_state_t s)
{
    xSemaphoreTake(g_status_lock, portMAX_DELAY);
    g_status.state = s;
    xSemaphoreGive(g_status_lock);
}

static void status_fail(const char *fmt, ...)
{
    char buf[sizeof(g_status.last_error)];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    xSemaphoreTake(g_status_lock, portMAX_DELAY);
    strlcpy(g_status.last_error, buf, sizeof(g_status.last_error));
    g_status.state = DB_FC_FLASH_FAILED;
    xSemaphoreGive(g_status_lock);

    db_fc_flash_log("ERROR: %s", buf);
}

void db_fc_flash_get_status(db_fc_flash_status_t *out)
{
    if (!out) return;
    xSemaphoreTake(g_status_lock, portMAX_DELAY);
    *out = g_status;
    xSemaphoreGive(g_status_lock);
}

/*
 * Armed only counts when we have a FRESH heartbeat saying so. If the FC UART
 * link is down we cannot know - and treating "unknown" as armed would make the
 * feature unusable in exactly the situation where somebody most wants to
 * reflash. The real protection is physical: an armed boat is in the water, and
 * this page is reached over the boat's own Wi-Fi.
 */
bool db_fc_flash_vehicle_is_armed(void)
{
    db_mavlink_telemetry_t t;
    db_mavlink_get_telemetry(&t);
    return t.fc.seen && !t.fc.stale && t.fc.armed;
}

/* ------------------------------------------------------------- USB plumbing */

static bool handle_rx(const uint8_t *data, size_t len, void *arg)
{
    if (g_raw_mode) {
        xStreamBufferSend(g_rx_stream, data, len, 0);
    }
    /* When not talking to the bootloader we deliberately discard: the FC's
     * MAVLink stream belongs to the UART path, not to this module. */
    return true;
}

static void handle_event(const cdc_acm_host_dev_event_data_t *event, void *ctx)
{
    switch (event->type) {
    case CDC_ACM_HOST_DEVICE_DISCONNECTED:
        cdc_acm_host_close(event->data.cdc_hdl);
        g_dev = NULL;
        xSemaphoreGive(g_disconnected);
        break;
    case CDC_ACM_HOST_ERROR:
        db_fc_flash_log("USB CDC error %d", event->data.error);
        break;
    default:
        break;
    }
}

static void usb_lib_task(void *arg)
{
    while (1) {
        uint32_t flags;
        usb_host_lib_handle_events(portMAX_DELAY, &flags);
        if (flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            usb_host_device_free_all();
        }
    }
}

/* Trap 2: ordered, deadline-bounded reads. Never a shared length/flag pair. */
static bool bl_cmd(const uint8_t *cmd, size_t cmd_len, size_t want, uint32_t tmo_ms)
{
    if (!g_dev || want > sizeof(g_reply)) return false;

    g_raw_mode = true;
    xStreamBufferReset(g_rx_stream);              /* == port.flushInput() */

    if (cdc_acm_host_data_tx_blocking(g_dev, cmd, cmd_len, 2000) != ESP_OK) {
        return false;
    }
    size_t got = 0;
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(tmo_ms);
    while (got < want) {
        TickType_t now = xTaskGetTickCount();
        if (now >= deadline) return false;
        got += xStreamBufferReceive(g_rx_stream, g_reply + got, want - got,
                                    deadline - now);
    }
    return true;
}

static bool bl_ok(const uint8_t *cmd, size_t len, uint32_t tmo)
{
    if (!bl_cmd(cmd, len, 2, tmo)) return false;
    return g_reply[0] == PROTO_INSYNC && g_reply[1] == PROTO_OK;
}

static bool bl_get_u32(uint8_t what, uint32_t *out)
{
    const uint8_t cmd[] = { PROTO_GET_DEVICE, what, PROTO_EOC };
    if (!bl_cmd(cmd, sizeof(cmd), 6, CMD_TIMEOUT_MS)) return false;
    if (g_reply[4] != PROTO_INSYNC || g_reply[5] != PROTO_OK) return false;
    *out = g_reply[0] | (g_reply[1] << 8) | (g_reply[2] << 16) |
           ((uint32_t)g_reply[3] << 24);
    return true;
}

static bool bl_is_listening(void)
{
    const uint8_t sync[] = { PROTO_GET_SYNC, PROTO_EOC };
    /* The first GET_SYNC after enumeration is sometimes swallowed. */
    if (bl_ok(sync, sizeof(sync), 1000)) return true;
    return bl_ok(sync, sizeof(sync), 1000);
}

static esp_err_t open_fc(uint32_t timeout_ms)
{
    const cdc_acm_host_device_config_t cfg = {
        .connection_timeout_ms = 1000,
        .out_buffer_size = 512,
        .in_buffer_size  = 2048,
        .user_arg = NULL,
        .event_cb = handle_event,
        .data_cb  = handle_rx,
    };
    int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    while (esp_timer_get_time() < deadline) {
        if (cdc_acm_host_open(FC_VID, FC_PID, 0, &cfg, &g_dev) == ESP_OK) {
            cdc_acm_line_coding_t lc = {
                /* 115200 and never 1200: ArduPilot treats 1200 as a
                 * reboot-to-bootloader trigger. */
                .dwDTERate = 115200, .bCharFormat = 0,
                .bParityType = 0, .bDataBits = 8,
            };
            cdc_acm_host_line_coding_set(g_dev, &lc);
            cdc_acm_host_set_control_line_state(g_dev, true, true);
            vTaskDelay(pdMS_TO_TICKS(400));
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(300));
    }
    return ESP_ERR_NOT_FOUND;
}

/* Trap 3: always release the handle when no disconnect event arrived. */
static void close_fc(void)
{
    if (g_dev) {
        cdc_acm_host_close(g_dev);
        g_dev = NULL;
    }
    g_raw_mode = false;
}

/* -------------------------------------------------------- stored image I/O */

esp_err_t db_fc_flash_get_image_info(db_fc_flash_image_info_t *out)
{
    if (!out) return ESP_ERR_INVALID_ARG;
    memset(out, 0, sizeof(*out));

    FILE *f = fopen(DB_FC_FLASH_META_PATH, "r");
    if (!f) return ESP_ERR_NOT_FOUND;

    char buf[512];
    size_t n = fread(buf, 1, sizeof(buf) - 1, f);
    fclose(f);
    buf[n] = '\0';

    cJSON *root = cJSON_Parse(buf);
    if (!root) return ESP_ERR_INVALID_STATE;

    const cJSON *j;
    if ((j = cJSON_GetObjectItem(root, "board_id")))   out->board_id   = (uint32_t)j->valuedouble;
    if ((j = cJSON_GetObjectItem(root, "image_size"))) out->image_size = (uint32_t)j->valuedouble;
    if ((j = cJSON_GetObjectItem(root, "crc")))        out->crc        = (uint32_t)j->valuedouble;
    if ((j = cJSON_GetObjectItem(root, "source")) && cJSON_IsString(j))
        strlcpy(out->source_name, j->valuestring, sizeof(out->source_name));
    if ((j = cJSON_GetObjectItem(root, "description")) && cJSON_IsString(j))
        strlcpy(out->description, j->valuestring, sizeof(out->description));
    cJSON_Delete(root);

    struct stat st;
    if (stat(DB_FC_FLASH_IMAGE_PATH, &st) != 0 ||
        (uint32_t)st.st_size != out->image_size) {
        return ESP_ERR_INVALID_SIZE;      /* sidecar disagrees with the image */
    }
    out->present = true;
    return ESP_OK;
}

esp_err_t db_fc_flash_delete_image(void)
{
    remove(DB_FC_FLASH_IMAGE_PATH);
    remove(DB_FC_FLASH_META_PATH);
    db_sonar_log_refresh_limits();   /* hand the space back to the log */
    db_fc_flash_log("stored firmware deleted");
    return ESP_OK;
}

/* ----------------------------------------------------------------- upload */

static FILE     *g_upload_fp;
static uint32_t  g_upload_bytes;
static uint32_t  g_upload_board_id;
static char      g_upload_name[64];

esp_err_t db_fc_flash_upload_begin(const char *filename, uint32_t board_id)
{
    if (g_busy) return ESP_ERR_INVALID_STATE;
    db_fc_flash_upload_abort();

    g_upload_fp = fopen(DB_FC_FLASH_IMAGE_PATH, "wb");
    if (!g_upload_fp) {
        db_fc_flash_log("cannot open %s for writing", DB_FC_FLASH_IMAGE_PATH);
        return ESP_FAIL;
    }
    g_upload_bytes = 0;
    g_upload_board_id = board_id;
    strlcpy(g_upload_name, filename ? filename : "firmware.apj",
            sizeof(g_upload_name));
    remove(DB_FC_FLASH_META_PATH);          /* invalid until finish() succeeds */
    db_fc_flash_log("receiving %s", g_upload_name);
    return ESP_OK;
}

esp_err_t db_fc_flash_upload_write(const uint8_t *data, size_t len)
{
    if (!g_upload_fp) return ESP_ERR_INVALID_STATE;
    if (g_upload_bytes + len > DB_FC_FLASH_MAX_IMAGE_BYTES) {
        db_fc_flash_upload_abort();
        db_fc_flash_log("image larger than the FC's app region - rejected");
        return ESP_ERR_INVALID_SIZE;
    }
    if (fwrite(data, 1, len, g_upload_fp) != len) {
        db_fc_flash_upload_abort();
        db_fc_flash_log("write failed - is the logs partition full?");
        return ESP_FAIL;
    }
    g_upload_bytes += len;
    return ESP_OK;
}

void db_fc_flash_upload_abort(void)
{
    if (g_upload_fp) {
        fclose(g_upload_fp);
        g_upload_fp = NULL;
    }
    remove(DB_FC_FLASH_IMAGE_PATH);
    remove(DB_FC_FLASH_META_PATH);
    g_upload_bytes = 0;
}

/*
 * Finish an upload: compute the ArduPilot CRC over the stored bytes plus 0xFF
 * padding to the FC's app-region size, and write the sidecar.
 *
 * The CRC is recomputed HERE from what actually landed in flash rather than
 * trusting the value the browser sent. A truncated or corrupted upload is
 * caught now, on the boat, instead of after the FC has been erased.
 */
esp_err_t db_fc_flash_upload_finish(db_fc_flash_image_info_t *out)
{
    if (!g_upload_fp) return ESP_ERR_INVALID_STATE;
    fclose(g_upload_fp);
    g_upload_fp = NULL;

    if (g_upload_bytes == 0 || (g_upload_bytes % 4) != 0) {
        db_fc_flash_upload_abort();
        db_fc_flash_log("image is %u bytes - must be non-empty and word aligned",
                        (unsigned)g_upload_bytes);
        return ESP_ERR_INVALID_SIZE;
    }

    ap_crc_init();
    FILE *f = fopen(DB_FC_FLASH_IMAGE_PATH, "rb");
    if (!f) return ESP_FAIL;

    uint32_t crc = 0;
    uint8_t *buf = malloc(2048);
    if (!buf) { fclose(f); return ESP_ERR_NO_MEM; }
    size_t n;
    while ((n = fread(buf, 1, 2048, f)) > 0) {
        crc = ap_crc32(crc, buf, n);
    }
    fclose(f);

    memset(buf, 0xFF, 2048);
    uint32_t pad = DB_FC_FLASH_MAX_IMAGE_BYTES - g_upload_bytes;
    while (pad) {
        uint32_t step = pad > 2048 ? 2048 : pad;
        crc = ap_crc32(crc, buf, step);
        pad -= step;
    }
    free(buf);

    /* board_id comes from the .apj via the uploader; 0 means unknown and
     * db_fc_flash_start() will refuse rather than guess. */
    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "board_id", g_upload_board_id);
    cJSON_AddNumberToObject(root, "image_size", g_upload_bytes);
    cJSON_AddNumberToObject(root, "crc", crc);
    cJSON_AddStringToObject(root, "source", g_upload_name);
    cJSON_AddStringToObject(root, "description", "");
    char *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    FILE *m = fopen(DB_FC_FLASH_META_PATH, "w");
    if (!m || !json) {
        if (m) fclose(m);
        free(json);
        db_fc_flash_upload_abort();
        return ESP_FAIL;
    }
    fputs(json, m);
    fclose(m);
    free(json);

    db_sonar_log_refresh_limits();   /* the log must now stay clear of the slot */
    db_fc_flash_log("stored %u bytes, CRC 0x%08X (ArduPilot variant)",
                    (unsigned)g_upload_bytes, (unsigned)crc);

    if (out) {
        return db_fc_flash_get_image_info(out);
    }
    return ESP_OK;
}

/* ------------------------------------------------------------- flash task */

static void flash_task(void *arg)
{
    db_fc_flash_image_info_t img;
    FILE *f = NULL;

    db_fc_flash_log("=== FC flash starting ===");

    if (db_fc_flash_get_image_info(&img) != ESP_OK || !img.present) {
        status_fail("no valid firmware stored");
        goto done;
    }
    db_fc_flash_log("image %s: %u bytes, board_id %u",
                    img.source_name, (unsigned)img.image_size,
                    (unsigned)img.board_id);
    if (img.board_id == 0) {
        status_fail("stored image has no board_id - refusing to guess");
        goto done;
    }

    status_set_state(DB_FC_FLASH_WAITING_FOR_FC);
    db_fc_flash_log("waiting for the FC on USB ...");
    if (open_fc(15000) != ESP_OK) {
        status_fail("FC not found on USB - check the cable");
        goto done;
    }

    if (!bl_is_listening()) {
        /* Application is running: ask it to reboot into the bootloader. */
        status_set_state(DB_FC_FLASH_REBOOTING);
        db_fc_flash_log("application running; requesting reboot to bootloader");
        cdc_acm_host_data_tx_blocking(g_dev, MAV_REBOOT_TO_BOOTLOADER,
                                      sizeof(MAV_REBOOT_TO_BOOTLOADER), 1000);
        xSemaphoreTake(g_disconnected, pdMS_TO_TICKS(8000));
        close_fc();
        if (open_fc(15000) != ESP_OK || !bl_is_listening()) {
            status_fail("FC did not come back in its bootloader");
            goto done;
        }
    }
    db_fc_flash_log("bootloader is answering");

    uint32_t bl_rev = 0, board_id = 0, flash_size = 0;
    bl_get_u32(INFO_BL_REV, &bl_rev);
    if (!bl_get_u32(INFO_BOARD_ID, &board_id) ||
        !bl_get_u32(INFO_FLASH_SIZE, &flash_size)) {
        status_fail("could not read the bootloader's board identity");
        goto done;
    }
    xSemaphoreTake(g_status_lock, portMAX_DELAY);
    g_status.in_bootloader = true;
    g_status.bl_rev = bl_rev;
    g_status.board_id = board_id;
    g_status.fc_flash_size = flash_size;
    g_status.bytes_total = img.image_size;
    g_status.crc_expected = img.crc;
    xSemaphoreGive(g_status_lock);
    db_fc_flash_log("bl rev %u, board ID %u, flash %u",
                    (unsigned)bl_rev, (unsigned)board_id, (unsigned)flash_size);

    /* THE guard that catches a Pro/non-Pro mix-up. */
    if (board_id != img.board_id) {
        status_fail("board ID %u does not match the image's %u - WRONG FIRMWARE",
                    (unsigned)board_id, (unsigned)img.board_id);
        goto done;
    }
    if (flash_size < img.image_size) {
        status_fail("image does not fit this board");
        goto done;
    }

    status_set_state(DB_FC_FLASH_ERASING);
    db_fc_flash_log("erasing (this takes about 10 s) ...");
    int64_t t0 = esp_timer_get_time();
    const uint8_t erase[] = { PROTO_CHIP_ERASE, PROTO_EOC };
    if (!bl_ok(erase, sizeof(erase), ERASE_TIMEOUT_MS)) {
        status_fail("erase failed - the FC has no application; recover over USB");
        goto done;
    }
    db_fc_flash_log("erased in %.1f s", (esp_timer_get_time() - t0) / 1e6);

    status_set_state(DB_FC_FLASH_PROGRAMMING);
    f = fopen(DB_FC_FLASH_IMAGE_PATH, "rb");
    if (!f) {
        status_fail("cannot reopen the stored image");
        goto done;
    }
    t0 = esp_timer_get_time();
    uint8_t frame[3 + PROG_CHUNK];
    uint32_t off = 0, pct_last = 0;
    while (off < img.image_size) {
        uint32_t want = img.image_size - off;
        if (want > PROG_CHUNK) want = PROG_CHUNK;

        if (fread(frame + 2, 1, want, f) != want) {
            status_fail("read failed at %u - partial application on the FC",
                        (unsigned)off);
            goto done;
        }
        frame[0] = PROTO_PROG_MULTI;
        frame[1] = (uint8_t)want;
        frame[2 + want] = PROTO_EOC;

        if (!bl_ok(frame, 3 + want, CMD_TIMEOUT_MS)) {
            status_fail("programming failed at %u - partial application on the FC",
                        (unsigned)off);
            goto done;
        }
        off += want;

        xSemaphoreTake(g_status_lock, portMAX_DELAY);
        g_status.bytes_done = off;
        xSemaphoreGive(g_status_lock);

        uint32_t pct = (uint32_t)(100ULL * off / img.image_size);
        if (pct >= pct_last + 10) {
            pct_last = pct;
            db_fc_flash_log("  %u%% (%u / %u bytes)", (unsigned)pct,
                            (unsigned)off, (unsigned)img.image_size);
        }
    }
    fclose(f);
    f = NULL;
    double secs = (esp_timer_get_time() - t0) / 1e6;
    db_fc_flash_log("programmed %u bytes in %.1f s (%.1f KB/s)",
                    (unsigned)img.image_size, secs,
                    img.image_size / secs / 1024.0);

    status_set_state(DB_FC_FLASH_VERIFYING);
    const uint8_t getcrc[] = { PROTO_GET_CRC, PROTO_EOC };
    if (!bl_cmd(getcrc, sizeof(getcrc), 6, 10000)) {
        status_fail("no CRC reply - NOT rebooting the FC");
        goto done;
    }
    uint32_t crc = g_reply[0] | (g_reply[1] << 8) | (g_reply[2] << 16) |
                   ((uint32_t)g_reply[3] << 24);
    xSemaphoreTake(g_status_lock, portMAX_DELAY);
    g_status.crc_actual = crc;
    xSemaphoreGive(g_status_lock);

    db_fc_flash_log("CRC from FC 0x%08X, expected 0x%08X",
                    (unsigned)crc, (unsigned)img.crc);
    if (crc != img.crc) {
        status_fail("CRC MISMATCH - not rebooting. Reflash before use.");
        goto done;
    }

    db_fc_flash_log("*** CRC matches - firmware written correctly ***");
    const uint8_t boot[] = { PROTO_BOOT, PROTO_EOC };
    cdc_acm_host_data_tx_blocking(g_dev, boot, sizeof(boot), 2000);
    db_fc_flash_log("BOOT sent - the FC should return to normal firmware");
    status_set_state(DB_FC_FLASH_DONE);

done:
    if (f) fclose(f);
    close_fc();
    g_busy = false;
    db_fc_flash_log("=== finished ===");
    vTaskDelete(NULL);
}

esp_err_t db_fc_flash_start(void)
{
    if (g_busy) {
        return ESP_ERR_INVALID_STATE;
    }
    if (db_fc_flash_vehicle_is_armed()) {
        db_fc_flash_log("REFUSED: the vehicle is armed");
        return ESP_ERR_INVALID_STATE;
    }
    db_fc_flash_image_info_t img;
    if (db_fc_flash_get_image_info(&img) != ESP_OK || !img.present) {
        db_fc_flash_log("REFUSED: no valid firmware stored");
        return ESP_ERR_NOT_FOUND;
    }

    xSemaphoreTake(g_status_lock, portMAX_DELAY);
    memset(&g_status, 0, sizeof(g_status));
    g_status.usb_host_running = true;
    xSemaphoreGive(g_status_lock);

    g_busy = true;
    if (xTaskCreate(flash_task, "fc_flash", FLASH_TASK_STACK, NULL, 5, NULL)
        != pdPASS) {
        g_busy = false;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

/* ------------------------------------------------------------------- init */

esp_err_t db_fc_flash_init(void)
{
    ap_crc_init();
    g_status_lock  = xSemaphoreCreateMutex();
    g_console.lock = xSemaphoreCreateMutex();
    g_disconnected = xSemaphoreCreateBinary();
    g_rx_stream    = xStreamBufferCreate(RX_STREAM_BYTES, 1);
    if (!g_status_lock || !g_console.lock || !g_disconnected || !g_rx_stream) {
        return ESP_ERR_NO_MEM;
    }

    const usb_host_config_t host_cfg = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };
    esp_err_t err = usb_host_install(&host_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "usb_host_install failed: %s", esp_err_to_name(err));
        return err;
    }
    if (xTaskCreate(usb_lib_task, "usb_lib", 4096, NULL, USB_HOST_PRIORITY, NULL)
        != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    err = cdc_acm_host_install(NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "cdc_acm_host_install failed: %s", esp_err_to_name(err));
        return err;
    }

    g_status.usb_host_running = true;
    ESP_LOGI(TAG, "FC flash service ready (USB host on the OTG pins)");
    return ESP_OK;
}
