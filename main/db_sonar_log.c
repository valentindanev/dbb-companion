#include "db_sonar_log.h"
#include "db_fc_flash.h" /* DB_FC_FLASH_RESERVE_BYTES - shares this FAT mount */

#include <errno.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

#include "esp_log.h"
#include "esp_attr.h"
#ifdef CONFIG_DB_LOG_STORAGE_FATFS
#include "esp_vfs_fat.h"
#include "wear_levelling.h"
#else
#include "esp_spiffs.h"
#endif
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "db_mavlink_msgs.h"

#define DB_SONAR_LOG_LINE_MAX 384
#define DB_SONAR_LOG_IO_BUFFER 1024
#define DB_SONAR_LOG_RESERVED_BYTES 8192
#define DB_SONAR_LOG_HARDWIRED_PERIOD_MS 1000
#define DB_SONAR_LOG_DEEPER_PERIOD_MS 1500
#define DB_SONAR_LOG_FC_GPS_MAX_AGE_MS 3000
#define DB_SONAR_LOG_DIAG_MAGIC 0x44424C47U

enum {
  DB_SONAR_LOG_STAGE_UNKNOWN = 0,
  DB_SONAR_LOG_STAGE_IDLE = 1,
  DB_SONAR_LOG_STAGE_BOOT = 2,
  DB_SONAR_LOG_STAGE_SESSION_SAMPLE = 10,
  DB_SONAR_LOG_STAGE_GPS_ENTER = 20,
  DB_SONAR_LOG_STAGE_GPS_SNAPSHOT = 21,
  DB_SONAR_LOG_STAGE_GPS_FORMATTED = 22,
  DB_SONAR_LOG_STAGE_APPEND_ENTER = 30,
  DB_SONAR_LOG_STAGE_APPEND_SPACE_READY = 31,
  DB_SONAR_LOG_STAGE_APPEND_FILE_OPEN = 32,
  DB_SONAR_LOG_STAGE_APPEND_LINE_WRITTEN = 33,
  DB_SONAR_LOG_STAGE_APPEND_FILE_CLOSED = 34,
  DB_SONAR_LOG_STAGE_APPEND_USAGE_REFRESHED = 35,
};

static const char *TAG = "DB_SONAR_LOG";

static SemaphoreHandle_t g_sonar_log_mutex = NULL;
/* 0 unless a flight-controller image is stored; see db_sonar_log_refresh_limits */
static size_t g_sonar_log_fw_reserve_bytes = 0;
static bool g_sonar_log_available = false;
static size_t g_sonar_log_partition_total_bytes = 0;
static size_t g_sonar_log_partition_used_bytes = 0;
static size_t g_sonar_log_max_file_bytes = 0;
static size_t g_sonar_log_trim_to_bytes = 0;
static uint32_t g_sonar_log_compaction_count = 0;
#ifdef CONFIG_DB_LOG_STORAGE_FATFS
static wl_handle_t g_sonar_log_wl_handle = WL_INVALID_HANDLE;
#endif
/*
 * These buffers are used only while g_sonar_log_mutex is held. Keeping them
 * out of caller task stacks leaves the sonar/MAVLink tasks enough headroom for
 * libc and filesystem calls.
 */
static char g_sonar_log_line_buffer[DB_SONAR_LOG_LINE_MAX];
static char g_sonar_log_io_buffer[DB_SONAR_LOG_IO_BUFFER];

typedef struct {
  uint32_t magic;
  uint32_t stage;
  uint32_t sequence;
  uint32_t min_stack;
} db_sonar_log_rtc_diag_t;

RTC_NOINIT_ATTR static db_sonar_log_rtc_diag_t g_sonar_log_rtc_diag;
static db_sonar_log_crash_diag_t g_sonar_log_boot_diag;
static bool g_sonar_log_diag_initialized = false;

static TickType_t g_last_hardwired_publish_log_tick = 0;
static int g_last_hardwired_logged_depth_mm = -1;
static int g_last_hardwired_logged_raw_mm = -1;
static bool g_last_hardwired_logged_zero_run = false;
static bool g_last_hardwired_logged_hold = false;
static TickType_t g_last_deeper_track_log_tick = 0;
static int g_last_deeper_logged_depth_mm = -1;
static bool g_last_deeper_logged_fix = false;
static bool g_last_deeper_logged_coordinates = false;
static double g_last_deeper_logged_latitude = 0.0;
static double g_last_deeper_logged_longitude = 0.0;

typedef struct {
  bool active;
  bool armed_state_seen;
  uint32_t start_ms;
  TickType_t last_sample_tick;
  uint32_t polls;
  uint32_t valid_frames;
  uint32_t zero_frames;
  uint32_t nonzero_frames;
  uint32_t timeouts;
  uint32_t bad_frames;
  uint32_t esp_to_fc_publishes;
  uint32_t fc_to_esp_distance_sensors;
} db_sonar_log_session_t;

static db_sonar_log_session_t g_hardwired_session;

const char *db_sonar_log_diag_stage_name(uint32_t stage) {
  switch (stage) {
  case DB_SONAR_LOG_STAGE_IDLE: return "idle";
  case DB_SONAR_LOG_STAGE_BOOT: return "boot";
  case DB_SONAR_LOG_STAGE_SESSION_SAMPLE: return "session_sample";
  case DB_SONAR_LOG_STAGE_GPS_ENTER: return "gps_enter";
  case DB_SONAR_LOG_STAGE_GPS_SNAPSHOT: return "gps_snapshot";
  case DB_SONAR_LOG_STAGE_GPS_FORMATTED: return "gps_formatted";
  case DB_SONAR_LOG_STAGE_APPEND_ENTER: return "append_enter";
  case DB_SONAR_LOG_STAGE_APPEND_SPACE_READY: return "append_space_ready";
  case DB_SONAR_LOG_STAGE_APPEND_FILE_OPEN: return "append_file_open";
  case DB_SONAR_LOG_STAGE_APPEND_LINE_WRITTEN: return "append_line_written";
  case DB_SONAR_LOG_STAGE_APPEND_FILE_CLOSED: return "append_file_closed";
  case DB_SONAR_LOG_STAGE_APPEND_USAGE_REFRESHED:
    return "append_usage_refreshed";
  default: return "unknown";
  }
}

static void db_sonar_log_diag_init(void) {
  if (g_sonar_log_diag_initialized) return;

  const bool previous_valid =
      g_sonar_log_rtc_diag.magic == DB_SONAR_LOG_DIAG_MAGIC;
  memset(&g_sonar_log_boot_diag, 0, sizeof(g_sonar_log_boot_diag));
  g_sonar_log_boot_diag.reset_reason = (uint32_t)esp_reset_reason();
  if (previous_valid) {
    g_sonar_log_boot_diag.previous_stage = g_sonar_log_rtc_diag.stage;
    g_sonar_log_boot_diag.previous_sequence = g_sonar_log_rtc_diag.sequence;
    g_sonar_log_boot_diag.previous_min_stack =
        g_sonar_log_rtc_diag.min_stack == UINT32_MAX
            ? 0
            : g_sonar_log_rtc_diag.min_stack;
  } else {
    memset(&g_sonar_log_rtc_diag, 0, sizeof(g_sonar_log_rtc_diag));
    g_sonar_log_rtc_diag.magic = DB_SONAR_LOG_DIAG_MAGIC;
  }

  g_sonar_log_rtc_diag.stage = DB_SONAR_LOG_STAGE_BOOT;
  g_sonar_log_rtc_diag.sequence++;
  g_sonar_log_rtc_diag.min_stack = UINT32_MAX;
  g_sonar_log_diag_initialized = true;
}

static void db_sonar_log_diag_mark(uint32_t stage) {
  db_sonar_log_diag_init();
  g_sonar_log_rtc_diag.stage = stage;
  g_sonar_log_rtc_diag.sequence++;
  uint32_t stack = (uint32_t)uxTaskGetStackHighWaterMark(NULL);
  if (stack < g_sonar_log_rtc_diag.min_stack) {
    g_sonar_log_rtc_diag.min_stack = stack;
  }
}

void db_sonar_log_get_crash_diag(db_sonar_log_crash_diag_t *diag) {
  if (diag == NULL) return;
  db_sonar_log_diag_init();
  *diag = g_sonar_log_boot_diag;
  diag->current_stage = g_sonar_log_rtc_diag.stage;
  diag->current_sequence = g_sonar_log_rtc_diag.sequence;
  diag->current_min_stack =
      g_sonar_log_rtc_diag.min_stack == UINT32_MAX
          ? 0
          : g_sonar_log_rtc_diag.min_stack;
}

static uint32_t db_sonar_log_now_ms(void) {
  return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

static const char *db_sonar_log_source_name(db_sonar_source_t source) {
  switch (source) {
  case DB_SONAR_SOURCE_HARDWIRED:
    return "hardwired";
  case DB_SONAR_SOURCE_DEEPER:
    return "deeper";
  default:
    return "none";
  }
}

static void db_sonar_log_init_mutex(void) {
  if (g_sonar_log_mutex == NULL) {
    g_sonar_log_mutex = xSemaphoreCreateMutex();
  }
}

static int db_sonar_log_abs_i32(int value) {
  return value < 0 ? -value : value;
}

static double db_sonar_log_abs_double(double value) {
  return value < 0.0 ? -value : value;
}

/*
 * THE ARMED GATE - why it exists.
 *
 * The log is a finite ring: db_sonar_log_prepare_space_locked() trims by
 * keeping the TAIL, so whatever is written last survives and the oldest lines
 * are discarded. That makes idle logging actively destructive.
 *
 * The use case this protects: you take the boat to the lake to capture a
 * specific behaviour, then bring it home to read the log. Reading it takes
 * time - power up, connect Wi-Fi, open the UI. If the boat logged while merely
 * powered on, sitting on the bench or in your hands, that bench noise would
 * push out the very session you drove to the lake to record.
 *
 * So sonar data is written only while the FC is genuinely flying the boat:
 * heartbeat seen, not stale, and armed. A stale or absent FC blocks writes too,
 * which is deliberate - a bench-powered ESP with no FC must stay silent.
 *
 * Two paths intentionally bypass this gate. See db_sonar_log_note_fc_armed_state()
 * for the disarm-boundary summary, and db_sonar_log_log_boot(), which must record
 * a reboot even when disarmed - a brownout mid-session is exactly the event you
 * need evidence of, and after it the FC heartbeat has not been seen yet.
 */
static bool db_sonar_log_fc_allows_sonar_write(void) {
  db_mavlink_fc_state_t fc_state = {0};
  db_mavlink_get_fc_state(&fc_state);
  return fc_state.seen && !fc_state.stale && fc_state.armed;
}

static void db_sonar_log_reset_hardwired_session(void) {
  memset(&g_hardwired_session, 0, sizeof(g_hardwired_session));
  g_hardwired_session.armed_state_seen = true;
  g_hardwired_session.active = true;
  g_hardwired_session.start_ms = db_sonar_log_now_ms();
}

static void db_sonar_log_write_hardwired_session(const char *event) {
  if (!g_hardwired_session.active) return;
  uint32_t elapsed_ms = db_sonar_log_now_ms() - g_hardwired_session.start_ms;
  db_sonar_log_appendf(
      "boot_ms=%lu event=%s source=hardwired elapsed_ms=%lu polls=%lu "
      "valid_frames=%lu zero_frames=%lu nonzero_frames=%lu timeouts=%lu "
      "bad_frames=%lu esp_to_fc=%lu fc_to_esp=%lu",
      (unsigned long)db_sonar_log_now_ms(), event,
      (unsigned long)elapsed_ms, (unsigned long)g_hardwired_session.polls,
      (unsigned long)g_hardwired_session.valid_frames,
      (unsigned long)g_hardwired_session.zero_frames,
      (unsigned long)g_hardwired_session.nonzero_frames,
      (unsigned long)g_hardwired_session.timeouts,
      (unsigned long)g_hardwired_session.bad_frames,
      (unsigned long)g_hardwired_session.esp_to_fc_publishes,
      (unsigned long)g_hardwired_session.fc_to_esp_distance_sensors);
}

void db_sonar_log_note_fc_armed_state(bool armed) {
  if (!g_sonar_log_available) return;
  if (!g_hardwired_session.armed_state_seen) {
    g_hardwired_session.armed_state_seen = true;
    if (armed) db_sonar_log_reset_hardwired_session();
    return;
  }
  if (armed && !g_hardwired_session.active) {
    db_sonar_log_reset_hardwired_session();
  } else if (!armed && g_hardwired_session.active) {
    /* Deliberately bypass the armed gate: this is the disarm boundary. */
    db_sonar_log_write_hardwired_session("session_end");
    g_hardwired_session.active = false;
  }
}

void db_sonar_log_note_hardwired_poll(void) {
  if (g_hardwired_session.active) g_hardwired_session.polls++;
}

void db_sonar_log_note_hardwired_frame(int distance_mm) {
  if (!g_hardwired_session.active) return;
  g_hardwired_session.valid_frames++;
  if (distance_mm == 0) g_hardwired_session.zero_frames++;
  else if (distance_mm > 0) g_hardwired_session.nonzero_frames++;
}

void db_sonar_log_note_hardwired_timeout(void) {
  if (g_hardwired_session.active) g_hardwired_session.timeouts++;
}

void db_sonar_log_note_hardwired_bad_frame(void) {
  if (g_hardwired_session.active) g_hardwired_session.bad_frames++;
}

void db_sonar_log_note_fc_returned_distance_sensor(void) {
  if (g_hardwired_session.active) g_hardwired_session.fc_to_esp_distance_sensors++;
}

void db_sonar_log_maybe_log_hardwired_session_sample(void) {
  if (!g_hardwired_session.active) return;
  TickType_t now = xTaskGetTickCount();
  if (g_hardwired_session.last_sample_tick != 0 &&
      (now - g_hardwired_session.last_sample_tick) < pdMS_TO_TICKS(1000)) return;
  db_sonar_log_diag_mark(DB_SONAR_LOG_STAGE_SESSION_SAMPLE);
  db_sonar_log_write_hardwired_session("session_sample");
  g_hardwired_session.last_sample_tick = now;
}

/*
 * The FC is the boat-position authority. Capture its position at the instant
 * each depth line is written, rather than using the Deeper's separate GPS.
 */
static void db_sonar_log_format_fc_gps(char *dst, size_t dst_size) {
  if (dst == NULL || dst_size == 0) {
    return;
  }

  db_sonar_log_diag_mark(DB_SONAR_LOG_STAGE_GPS_ENTER);
  db_mavlink_gps_state_t gps = {0};
  db_mavlink_get_gps_state(&gps);
  db_sonar_log_diag_mark(DB_SONAR_LOG_STAGE_GPS_SNAPSHOT);
  if (!gps.valid || gps.fix_type < 3 || gps.age_ms < 0 ||
      gps.age_ms > DB_SONAR_LOG_FC_GPS_MAX_AGE_MS) {
    snprintf(dst, dst_size, "fc_gps=no_location");
    db_sonar_log_diag_mark(DB_SONAR_LOG_STAGE_GPS_FORMATTED);
    return;
  }

  snprintf(dst, dst_size, "fc_gps=%.7f,%.7f gps_age_ms=%lld gps_fix=%u",
           (double)gps.latitude_e7 / 10000000.0,
           (double)gps.longitude_e7 / 10000000.0,
           (long long)gps.age_ms, (unsigned int)gps.fix_type);
  db_sonar_log_diag_mark(DB_SONAR_LOG_STAGE_GPS_FORMATTED);
}

static size_t db_sonar_log_get_file_size_locked(void) {
  struct stat st = {0};
  if (stat(DB_SONAR_LOG_FILE_PATH, &st) != 0) {
    return 0;
  }
  return (size_t)st.st_size;
}

/*
 * Hold back filesystem headroom, plus a flight-controller firmware slot but
 * ONLY while an image is actually stored. db_fc_flash keeps its .bin in this
 * same FAT mount; with no reserve at all the log grows to fill the partition
 * (the original held back just 8 KiB) and the two starve each other - either
 * the upload fails, or the log's tail-copy trim has nowhere to write sonar.tmp.
 *
 * The reserve is dynamic because the workflow is upload -> flash -> delete: the
 * image is present for minutes, not for the boat's life, and a static reserve
 * cost ~1.4 MiB of log capacity permanently for a file usually not there.
 * See DB_FC_FLASH_RESERVE_BYTES in db_fc_flash.h.
 */
static void db_sonar_log_apply_limits_locked(size_t total) {
  size_t want_reserved =
      (size_t)DB_SONAR_LOG_RESERVED_BYTES + g_sonar_log_fw_reserve_bytes;
  size_t reserved = total > want_reserved ? want_reserved : total / 8;
  size_t limit = total > reserved ? total - reserved : total;
  g_sonar_log_max_file_bytes = limit;
  g_sonar_log_trim_to_bytes = (limit * 3) / 4;
}

static esp_err_t db_sonar_log_refresh_usage_locked(void) {
  if (!g_sonar_log_available) {
    return ESP_ERR_INVALID_STATE;
  }

  size_t total = 0;
  size_t used = 0;
#ifdef CONFIG_DB_LOG_STORAGE_FATFS
  uint64_t fat_total = 0;
  uint64_t fat_free = 0;
  esp_err_t err =
      esp_vfs_fat_info(DB_SONAR_LOG_MOUNT_POINT, &fat_total, &fat_free);
  total = (size_t)fat_total;
  used = (size_t)(fat_total - fat_free);
#else
  esp_err_t err =
      esp_spiffs_info(DB_SONAR_LOG_PARTITION_LABEL, &total, &used);
#endif
  if (err != ESP_OK) {
    return err;
  }

  g_sonar_log_partition_total_bytes = total;
  g_sonar_log_partition_used_bytes = used;
  db_sonar_log_apply_limits_locked(total);
  return ESP_OK;
}

/*
 * Re-read whether a flight-controller image is stored, then recompute the log
 * limits. Call this whenever db_fc_flash stores or deletes one.
 *
 * Deliberately NOT done inside db_sonar_log_refresh_usage_locked(): three of its
 * five callers are on the log write path, and a stat() per write on FAT is not
 * free. The reserve only changes on an upload or a delete, so it is cached.
 */
void db_sonar_log_refresh_limits(void) {
  if (g_sonar_log_mutex == NULL) {
    return;
  }
  struct stat st;
  size_t fw = (stat(DB_FC_FLASH_IMAGE_PATH, &st) == 0)
                  ? (size_t)DB_FC_FLASH_RESERVE_BYTES
                  : 0;
  xSemaphoreTake(g_sonar_log_mutex, portMAX_DELAY);
  if (fw != g_sonar_log_fw_reserve_bytes) {
    g_sonar_log_fw_reserve_bytes = fw;
    db_sonar_log_apply_limits_locked(g_sonar_log_partition_total_bytes);
    ESP_LOGI(TAG,
             "Log limit now %u bytes (firmware slot %s)",
             (unsigned int)g_sonar_log_max_file_bytes,
             fw ? "reserved" : "released");
  }
  xSemaphoreGive(g_sonar_log_mutex);
}

static esp_err_t db_sonar_log_copy_tail_locked(size_t keep_bytes) {
  FILE *source = fopen(DB_SONAR_LOG_FILE_PATH, "rb");
  if (source == NULL) {
    return ESP_OK;
  }

  FILE *tmp = fopen(DB_SONAR_LOG_MOUNT_POINT "/sonar.tmp", "wb");
  if (tmp == NULL) {
    fclose(source);
    return ESP_FAIL;
  }

  size_t file_size = db_sonar_log_get_file_size_locked();
  long start_offset = 0;
  if (keep_bytes < file_size) {
    start_offset = (long)(file_size - keep_bytes);
  }

  if (start_offset > 0 && fseek(source, start_offset, SEEK_SET) != 0) {
    fclose(source);
    fclose(tmp);
    remove(DB_SONAR_LOG_MOUNT_POINT "/sonar.tmp");
    return ESP_FAIL;
  }

  if (start_offset > 0) {
    int ch = 0;
    while ((ch = fgetc(source)) != EOF) {
      if (ch == '\n') {
        break;
      }
    }
  }

  size_t read_bytes = 0;
  while ((read_bytes = fread(g_sonar_log_io_buffer, 1,
                             sizeof(g_sonar_log_io_buffer), source)) > 0) {
    if (fwrite(g_sonar_log_io_buffer, 1, read_bytes, tmp) != read_bytes) {
      fclose(source);
      fclose(tmp);
      remove(DB_SONAR_LOG_MOUNT_POINT "/sonar.tmp");
      return ESP_FAIL;
    }
  }

  fclose(source);
  fclose(tmp);

  if (remove(DB_SONAR_LOG_FILE_PATH) != 0) {
    remove(DB_SONAR_LOG_MOUNT_POINT "/sonar.tmp");
    return ESP_FAIL;
  }

  if (rename(DB_SONAR_LOG_MOUNT_POINT "/sonar.tmp", DB_SONAR_LOG_FILE_PATH) !=
      0) {
    return ESP_FAIL;
  }

  g_sonar_log_compaction_count++;
  return ESP_OK;
}

static esp_err_t db_sonar_log_prepare_space_locked(size_t incoming_bytes) {
  if (!g_sonar_log_available) {
    return ESP_ERR_INVALID_STATE;
  }

  if (db_sonar_log_refresh_usage_locked() != ESP_OK) {
    return ESP_FAIL;
  }

  size_t file_size = db_sonar_log_get_file_size_locked();
  if (file_size + incoming_bytes <= g_sonar_log_max_file_bytes) {
    return ESP_OK;
  }

  ESP_LOGW(TAG,
           "Compacting sonar log file to keep the newest entries "
           "(current=%u bytes incoming=%u max=%u)",
           (unsigned int)file_size, (unsigned int)incoming_bytes,
           (unsigned int)g_sonar_log_max_file_bytes);

  return db_sonar_log_copy_tail_locked(g_sonar_log_trim_to_bytes);
}

bool db_sonar_log_is_available(void) { return g_sonar_log_available; }

esp_err_t db_sonar_log_init(void) {
  db_sonar_log_diag_init();
  db_sonar_log_init_mutex();
  if (g_sonar_log_mutex == NULL) {
    return ESP_ERR_NO_MEM;
  }

  xSemaphoreTake(g_sonar_log_mutex, portMAX_DELAY);
  if (g_sonar_log_available) {
    xSemaphoreGive(g_sonar_log_mutex);
    return ESP_OK;
  }

#ifdef CONFIG_DB_LOG_STORAGE_FATFS
  esp_vfs_fat_mount_config_t conf = {
      .format_if_mount_failed = true,
      .max_files = 3,
      .allocation_unit_size = 4096,
      .disk_status_check_enable = false,
      .use_one_fat = false,
  };
  esp_err_t err = esp_vfs_fat_spiflash_mount_rw_wl(
      DB_SONAR_LOG_MOUNT_POINT, DB_SONAR_LOG_PARTITION_LABEL, &conf,
      &g_sonar_log_wl_handle);
#else
  esp_vfs_spiffs_conf_t conf = {.base_path = DB_SONAR_LOG_MOUNT_POINT,
                                .partition_label = DB_SONAR_LOG_PARTITION_LABEL,
                                .max_files = 3,
                                .format_if_mount_failed = true};
  esp_err_t err = esp_vfs_spiffs_register(&conf);
#endif
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to mount sonar log partition (%s)",
             esp_err_to_name(err));
    g_sonar_log_available = false;
    xSemaphoreGive(g_sonar_log_mutex);
    return err;
  }

  g_sonar_log_available = true;
  err = db_sonar_log_refresh_usage_locked();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to query sonar log filesystem info (%s)",
             esp_err_to_name(err));
  } else {
    ESP_LOGI(TAG,
             "Sonar log filesystem ready. total=%u used=%u max_log=%u trim_to=%u",
             (unsigned int)g_sonar_log_partition_total_bytes,
             (unsigned int)g_sonar_log_partition_used_bytes,
             (unsigned int)g_sonar_log_max_file_bytes,
             (unsigned int)g_sonar_log_trim_to_bytes);
  }

  FILE *fp = fopen(DB_SONAR_LOG_FILE_PATH, "ab");
  if (fp != NULL) {
    fclose(fp);
  }

  xSemaphoreGive(g_sonar_log_mutex);
  return err;
}

esp_err_t db_sonar_log_get_status(db_sonar_log_status_t *status) {
  if (status == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  memset(status, 0, sizeof(*status));
  db_sonar_log_init_mutex();
  if (g_sonar_log_mutex == NULL) {
    return ESP_ERR_NO_MEM;
  }

  xSemaphoreTake(g_sonar_log_mutex, portMAX_DELAY);
  if (g_sonar_log_available) {
    db_sonar_log_refresh_usage_locked();
    status->mounted = true;
    status->partition_total_bytes = g_sonar_log_partition_total_bytes;
    status->partition_used_bytes = g_sonar_log_partition_used_bytes;
    status->log_file_bytes = db_sonar_log_get_file_size_locked();
    status->max_log_file_bytes = g_sonar_log_max_file_bytes;
    status->trim_to_bytes = g_sonar_log_trim_to_bytes;
    status->compaction_count = g_sonar_log_compaction_count;
  }
  xSemaphoreGive(g_sonar_log_mutex);
  return ESP_OK;
}

esp_err_t db_sonar_log_stream(db_sonar_log_chunk_writer_t chunk_writer,
                              void *user_ctx, size_t *out_bytes_streamed,
                              int *out_file_errno) {
  if (chunk_writer == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  if (out_bytes_streamed != NULL) {
    *out_bytes_streamed = 0;
  }
  if (out_file_errno != NULL) {
    *out_file_errno = 0;
  }

  db_sonar_log_init_mutex();
  if (g_sonar_log_mutex == NULL) {
    return ESP_ERR_NO_MEM;
  }

  /*
   * Keep the logger mutex for the complete stream. The file is intentionally
   * small (about 225 KB maximum), and a coherent diagnostic snapshot is more
   * important than allowing an append or compaction to interleave with an
   * active download. Writers resume as soon as the stream closes.
   */
  xSemaphoreTake(g_sonar_log_mutex, portMAX_DELAY);
  if (!g_sonar_log_available) {
    xSemaphoreGive(g_sonar_log_mutex);
    return ESP_ERR_INVALID_STATE;
  }

  errno = 0;
  FILE *fp = fopen(DB_SONAR_LOG_FILE_PATH, "rb");
  if (fp == NULL) {
    int file_errno = errno;
    if (out_file_errno != NULL) {
      *out_file_errno = file_errno;
    }
    ESP_LOGE(TAG, "Failed to open sonar log for streaming: path=%s errno=%d "
                  "(%s)",
             DB_SONAR_LOG_FILE_PATH, file_errno, strerror(file_errno));
    xSemaphoreGive(g_sonar_log_mutex);
    return ESP_ERR_NOT_FOUND;
  }

  char buffer[DB_SONAR_LOG_IO_BUFFER];
  size_t total_bytes = 0;
  esp_err_t result = ESP_OK;

  while (true) {
    errno = 0;
    size_t read_bytes = fread(buffer, 1, sizeof(buffer), fp);
    if (read_bytes > 0) {
      result = chunk_writer(buffer, read_bytes, user_ctx);
      if (result != ESP_OK) {
        /*
         * A transport failure (for example, a Wi-Fi client disconnect) is not
         * a filesystem error. Close promptly so logging cannot remain blocked.
         */
        ESP_LOGW(TAG,
                 "Sonar log stream consumer stopped after %u bytes (%s)",
                 (unsigned int)total_bytes, esp_err_to_name(result));
        break;
      }
      total_bytes += read_bytes;
    }

    if (read_bytes < sizeof(buffer)) {
      if (ferror(fp)) {
        int file_errno = errno;
        if (out_file_errno != NULL) {
          *out_file_errno = file_errno;
        }
        ESP_LOGE(TAG,
                 "Failed while reading sonar log: path=%s bytes=%u errno=%d "
                 "(%s)",
                 DB_SONAR_LOG_FILE_PATH, (unsigned int)total_bytes,
                 file_errno, strerror(file_errno));
        result = ESP_FAIL;
      }
      break;
    }
  }

  fclose(fp);
  xSemaphoreGive(g_sonar_log_mutex);

  if (out_bytes_streamed != NULL) {
    *out_bytes_streamed = total_bytes;
  }
  return result;
}

esp_err_t db_sonar_log_clear(void) {
  db_sonar_log_init_mutex();
  if (g_sonar_log_mutex == NULL) {
    return ESP_ERR_NO_MEM;
  }

  xSemaphoreTake(g_sonar_log_mutex, portMAX_DELAY);
  if (!g_sonar_log_available) {
    xSemaphoreGive(g_sonar_log_mutex);
    return ESP_ERR_INVALID_STATE;
  }

  remove(DB_SONAR_LOG_FILE_PATH);
  FILE *fp = fopen(DB_SONAR_LOG_FILE_PATH, "wb");
  if (fp == NULL) {
    xSemaphoreGive(g_sonar_log_mutex);
    return ESP_FAIL;
  }
  fclose(fp);

  g_sonar_log_compaction_count = 0;
  g_last_hardwired_publish_log_tick = 0;
  g_last_hardwired_logged_depth_mm = -1;
  g_last_hardwired_logged_raw_mm = -1;
  g_last_hardwired_logged_zero_run = false;
  g_last_hardwired_logged_hold = false;
  g_last_deeper_track_log_tick = 0;
  g_last_deeper_logged_depth_mm = -1;
  g_last_deeper_logged_fix = false;
  g_last_deeper_logged_coordinates = false;
  g_last_deeper_logged_latitude = 0.0;
  g_last_deeper_logged_longitude = 0.0;

  db_sonar_log_refresh_usage_locked();
  xSemaphoreGive(g_sonar_log_mutex);
  return ESP_OK;
}

esp_err_t db_sonar_log_appendf(const char *fmt, ...) {
  if (fmt == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  db_sonar_log_init_mutex();
  if (g_sonar_log_mutex == NULL) {
    return ESP_ERR_NO_MEM;
  }
  if (!g_sonar_log_available) {
    return ESP_ERR_INVALID_STATE;
  }

  xSemaphoreTake(g_sonar_log_mutex, portMAX_DELAY);
  va_list args;
  va_start(args, fmt);
  vsnprintf(g_sonar_log_line_buffer, sizeof(g_sonar_log_line_buffer), fmt,
            args);
  va_end(args);

  size_t line_len = strlen(g_sonar_log_line_buffer);
  bool needs_newline =
      line_len == 0 || g_sonar_log_line_buffer[line_len - 1] != '\n';

  db_sonar_log_diag_mark(DB_SONAR_LOG_STAGE_APPEND_ENTER);
  esp_err_t err =
      db_sonar_log_prepare_space_locked(line_len + (needs_newline ? 1 : 0));
  if (err != ESP_OK) {
    xSemaphoreGive(g_sonar_log_mutex);
    return err;
  }
  db_sonar_log_diag_mark(DB_SONAR_LOG_STAGE_APPEND_SPACE_READY);

  FILE *fp = fopen(DB_SONAR_LOG_FILE_PATH, "ab");
  if (fp == NULL) {
    xSemaphoreGive(g_sonar_log_mutex);
    return ESP_FAIL;
  }
  db_sonar_log_diag_mark(DB_SONAR_LOG_STAGE_APPEND_FILE_OPEN);

  if (fwrite(g_sonar_log_line_buffer, 1, line_len, fp) != line_len) {
    fclose(fp);
    xSemaphoreGive(g_sonar_log_mutex);
    return ESP_FAIL;
  }
  if (needs_newline) {
    fputc('\n', fp);
  }
  db_sonar_log_diag_mark(DB_SONAR_LOG_STAGE_APPEND_LINE_WRITTEN);

  fclose(fp);
  db_sonar_log_diag_mark(DB_SONAR_LOG_STAGE_APPEND_FILE_CLOSED);
  db_sonar_log_refresh_usage_locked();
  db_sonar_log_diag_mark(DB_SONAR_LOG_STAGE_APPEND_USAGE_REFRESHED);
  xSemaphoreGive(g_sonar_log_mutex);
  db_sonar_log_diag_mark(DB_SONAR_LOG_STAGE_IDLE);
  return ESP_OK;
}

void db_sonar_log_log_boot(db_sonar_source_t active_source, int boot_radio_mode,
                           bool deeper_connected, bool force_update_ap_mode,
                           bool web_fs_available) {
  if (!g_sonar_log_available) {
    return;
  }

  db_sonar_log_crash_diag_t diag = {0};
  db_sonar_log_get_crash_diag(&diag);
  db_sonar_log_appendf(
      "boot_ms=%lu event=boot source=%s radio_mode=%d deeper_connected=%d "
      "force_update_ap=%d web_fs=%d reset_reason=%lu prev_stage=%lu "
      "prev_stage_name=%s prev_seq=%lu prev_min_stack=%lu",
      (unsigned long)db_sonar_log_now_ms(),
      db_sonar_log_source_name(active_source), boot_radio_mode,
      deeper_connected ? 1 : 0, force_update_ap_mode ? 1 : 0,
      web_fs_available ? 1 : 0, (unsigned long)diag.reset_reason,
      (unsigned long)diag.previous_stage,
      db_sonar_log_diag_stage_name(diag.previous_stage),
      (unsigned long)diag.previous_sequence,
      (unsigned long)diag.previous_min_stack);
}

void db_sonar_log_log_hardwired_frame(int distance_mm, int frame_length,
                                      const char *frame_bytes) {
  if (!g_sonar_log_available || !db_sonar_log_fc_allows_sonar_write()) {
    return;
  }

  char fc_gps[80];
  db_sonar_log_format_fc_gps(fc_gps, sizeof(fc_gps));
  db_sonar_log_appendf(
      "boot_ms=%lu event=hardwired_frame raw_mm=%d len=%d bytes=%s %s",
      (unsigned long)db_sonar_log_now_ms(), distance_mm, frame_length,
      frame_bytes == NULL ? "<none>" : frame_bytes, fc_gps);
}

void db_sonar_log_log_hardwired_issue(const char *issue, const char *detail) {
  if (!g_sonar_log_available || !db_sonar_log_fc_allows_sonar_write()) {
    return;
  }

  db_sonar_log_appendf("boot_ms=%lu event=hardwired_issue issue=%s detail=%s",
                       (unsigned long)db_sonar_log_now_ms(),
                       issue == NULL ? "unknown" : issue,
                       detail == NULL ? "n/a" : detail);
}

void db_sonar_log_log_hardwired_zero_run_start(void) {
  if (!g_sonar_log_available || !db_sonar_log_fc_allows_sonar_write()) {
    return;
  }

  char fc_gps[80];
  db_sonar_log_format_fc_gps(fc_gps, sizeof(fc_gps));
  db_sonar_log_appendf(
      "boot_ms=%lu event=hardwired_zero_run_start raw_mm=0 %s",
      (unsigned long)db_sonar_log_now_ms(), fc_gps);
}

void db_sonar_log_log_hardwired_zero_run_clear(int distance_mm) {
  if (!g_sonar_log_available || !db_sonar_log_fc_allows_sonar_write()) {
    return;
  }

  char fc_gps[80];
  db_sonar_log_format_fc_gps(fc_gps, sizeof(fc_gps));
  db_sonar_log_appendf(
      "boot_ms=%lu event=hardwired_zero_run_clear restored_mm=%d %s",
      (unsigned long)db_sonar_log_now_ms(), distance_mm, fc_gps);
}

void db_sonar_log_maybe_log_hardwired_publish(
    int published_distance_mm, int published_distance_cm,
    const danevi_sonar_snapshot_t *snapshot) {
  if (!g_sonar_log_available || snapshot == NULL ||
      !db_sonar_log_fc_allows_sonar_write()) {
    return;
  }

  if (g_hardwired_session.active) g_hardwired_session.esp_to_fc_publishes++;

  TickType_t now_tick = xTaskGetTickCount();
  int raw_mm = snapshot->has_raw_distance ? snapshot->raw_depth_mm : -1;
  int delta = db_sonar_log_abs_i32(published_distance_mm -
                                   g_last_hardwired_logged_depth_mm);
  bool should_log =
      g_last_hardwired_publish_log_tick == 0 || delta >= 10 ||
      (raw_mm == 0 && g_last_hardwired_logged_raw_mm != 0) ||
      snapshot->zero_run_active != g_last_hardwired_logged_zero_run ||
      snapshot->zero_filter_holding_last_good != g_last_hardwired_logged_hold ||
      raw_mm != g_last_hardwired_logged_raw_mm ||
      (now_tick - g_last_hardwired_publish_log_tick) >=
          pdMS_TO_TICKS(DB_SONAR_LOG_HARDWIRED_PERIOD_MS);

  if (!should_log) {
    return;
  }

  char fc_gps[80];
  db_sonar_log_format_fc_gps(fc_gps, sizeof(fc_gps));
  db_sonar_log_appendf(
      "boot_ms=%lu event=publish source=hardwired depth_mm=%d fc_cm=%d "
      "raw_mm=%d raw_age_ms=%lu last_good_mm=%d last_good_age_ms=%lu "
      "zero_run=%d zero_age_ms=%lu zero_count=%lu holding=%d %s",
      (unsigned long)db_sonar_log_now_ms(), published_distance_mm,
      published_distance_cm, raw_mm,
      (unsigned long)snapshot->raw_sample_age_ms,
      snapshot->has_last_good_distance ? snapshot->last_good_depth_mm : -1,
      (unsigned long)snapshot->last_good_sample_age_ms,
      snapshot->zero_run_active ? 1 : 0,
      (unsigned long)snapshot->zero_run_age_ms,
      (unsigned long)snapshot->consecutive_zero_frames,
      snapshot->zero_filter_holding_last_good ? 1 : 0, fc_gps);

  g_last_hardwired_publish_log_tick = now_tick;
  g_last_hardwired_logged_depth_mm = published_distance_mm;
  g_last_hardwired_logged_raw_mm = raw_mm;
  g_last_hardwired_logged_zero_run = snapshot->zero_run_active;
  g_last_hardwired_logged_hold = snapshot->zero_filter_holding_last_good;
}

void db_sonar_log_maybe_log_deeper_track(const deeper_udp_snapshot_t *snapshot) {
  if (!g_sonar_log_available || snapshot == NULL ||
      !db_sonar_log_fc_allows_sonar_write()) {
    return;
  }

  TickType_t now_tick = xTaskGetTickCount();
  int depth_mm = snapshot->has_depth ? snapshot->depth_mm : -1;
  bool fix_valid = snapshot->has_satellites && snapshot->gps_fix_valid;
  bool coordinates_changed = false;
  if (snapshot->has_coordinates && g_last_deeper_logged_coordinates) {
    coordinates_changed =
        db_sonar_log_abs_double(snapshot->latitude_deg -
                                g_last_deeper_logged_latitude) >= 0.00001 ||
        db_sonar_log_abs_double(snapshot->longitude_deg -
                                g_last_deeper_logged_longitude) >= 0.00001;
  } else if (snapshot->has_coordinates != g_last_deeper_logged_coordinates) {
    coordinates_changed = true;
  }

  bool should_log =
      g_last_deeper_track_log_tick == 0 || coordinates_changed ||
      fix_valid != g_last_deeper_logged_fix ||
      snapshot->has_coordinates != g_last_deeper_logged_coordinates ||
      db_sonar_log_abs_i32(depth_mm - g_last_deeper_logged_depth_mm) >= 100 ||
      (now_tick - g_last_deeper_track_log_tick) >=
          pdMS_TO_TICKS(DB_SONAR_LOG_DEEPER_PERIOD_MS);

  if (!should_log) {
    return;
  }

  char fc_gps[80];
  db_sonar_log_format_fc_gps(fc_gps, sizeof(fc_gps));
  db_sonar_log_appendf(
      "boot_ms=%lu event=track source=deeper depth_mm=%d fc_cm=%d "
      "temp_c=%.1f deeper_gps_fix=%d deeper_sats=%d deeper_lat=%.6f "
      "deeper_lon=%.6f sample_age_ms=%lu %s",
      (unsigned long)db_sonar_log_now_ms(), depth_mm,
      depth_mm >= 0 ? depth_mm / 10 : -1,
      snapshot->has_temperature
          ? ((double)snapshot->temperature_c_tenths) / 10.0
          : -1000.0,
      fix_valid ? 1 : 0,
      snapshot->has_satellites ? snapshot->satellites : -1,
      snapshot->has_coordinates ? snapshot->latitude_deg : 0.0,
      snapshot->has_coordinates ? snapshot->longitude_deg : 0.0,
      (unsigned long)snapshot->newest_sample_age_ms, fc_gps);

  g_last_deeper_track_log_tick = now_tick;
  g_last_deeper_logged_depth_mm = depth_mm;
  g_last_deeper_logged_fix = fix_valid;
  g_last_deeper_logged_coordinates = snapshot->has_coordinates;
  if (snapshot->has_coordinates) {
    g_last_deeper_logged_latitude = snapshot->latitude_deg;
    g_last_deeper_logged_longitude = snapshot->longitude_deg;
  }
}
