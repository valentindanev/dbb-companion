#include "db_sonar_log.h"
#include "db_fc_flash.h"
#include "db_fc_params.h"
#include "db_mavlink_msgs.h"
#include "db_parameters.h"
#include "sonar_driver.h"

#include <dirent.h>
#include <errno.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

#include "esp_attr.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#ifdef CONFIG_DB_LOG_STORAGE_FATFS
#include "esp_vfs_fat.h"
#include "wear_levelling.h"
#else
#include "esp_spiffs.h"
#endif
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#define DB_LOG_LINE_MAX 512U
#define DB_LOG_IO_BUFFER 1024U
#define DB_LOG_SYSTEM_LIMIT_BYTES (256U * 1024U)
#define DB_LOG_SYSTEM_TRIM_BYTES (192U * 1024U)
#define DB_LOG_HEADROOM_BYTES (256U * 1024U)
/* FAT rounds every file up to a cluster, so bytes-used always reads a little
 * above the sum of file sizes. Only a GROSS divergence means directory
 * entries were lost while their cluster chains stayed allocated - the
 * 23-08-2026 fault, where 3.29 MB sat unreachable and unreclaimable and
 * nothing in the firmware noticed. */
#define DB_LOG_ORPHAN_ALARM_BYTES (256U * 1024U)
#define DB_LOG_HARDWIRED_PERIOD_MS 1000U
#define DB_LOG_DEEPER_PERIOD_MS 1500U
#define DB_LOG_TRIP_PERIOD_MS 1000U
#define DB_LOG_FC_GPS_MAX_AGE_MS 3000U
#define DB_LOG_MANUAL_MIN_SECONDS 30U
#define DB_LOG_MANUAL_MAX_SECONDS (12U * 60U * 60U)
#define DB_LOG_DIAG_MAGIC 0x44424C47U
#define DB_LOG_SYSTEM_PATH DB_SONAR_LOG_MOUNT_POINT "/SYSTEM.LOG"
#define DB_LOG_SYSTEM_TMP_PATH DB_SONAR_LOG_MOUNT_POINT "/SYS.TMP"

enum {
  DB_LOG_STAGE_UNKNOWN = 0,
  DB_LOG_STAGE_IDLE = 1,
  DB_LOG_STAGE_BOOT = 2,
  DB_LOG_STAGE_SESSION_SAMPLE = 10,
  DB_LOG_STAGE_GPS_ENTER = 20,
  DB_LOG_STAGE_GPS_SNAPSHOT = 21,
  DB_LOG_STAGE_GPS_FORMATTED = 22,
  DB_LOG_STAGE_APPEND_ENTER = 30,
  DB_LOG_STAGE_APPEND_SPACE_READY = 31,
  DB_LOG_STAGE_APPEND_FILE_OPEN = 32,
  DB_LOG_STAGE_APPEND_LINE_WRITTEN = 33,
  DB_LOG_STAGE_APPEND_FILE_CLOSED = 34,
  DB_LOG_STAGE_APPEND_USAGE_REFRESHED = 35,
};

typedef struct {
  uint32_t magic;
  uint32_t stage;
  uint32_t sequence;
  uint32_t min_stack;
} db_log_rtc_diag_t;

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
} db_log_hardwired_stats_t;

static const char *TAG = "DB_LOG";
static SemaphoreHandle_t g_log_mutex;
static bool g_log_available;
static size_t g_partition_total;
static size_t g_partition_used;
static size_t g_partition_free;
static uint32_t g_evicted_sessions;
#ifdef CONFIG_DB_LOG_STORAGE_FATFS
static wl_handle_t g_log_wl_handle = WL_INVALID_HANDLE;
#endif

static char g_line_buffer[DB_LOG_LINE_MAX];
static char g_io_buffer[DB_LOG_IO_BUFFER];
/* Field diagnosis: first/last write failure on the append path. */
static int g_last_write_errno;
static uint32_t g_last_write_fail_stage; /* 1=prepare 2=fopen 3=fwrite 4=init_create */
static db_mavlink_telemetry_t g_tick_telemetry;

RTC_NOINIT_ATTR static db_log_rtc_diag_t g_log_rtc_diag;
static db_sonar_log_crash_diag_t g_log_boot_diag;
static bool g_log_diag_initialized;

static bool g_session_active;
static bool g_armed_capture;
static uint32_t g_session_id;
static uint32_t g_next_session_id = 1;
static uint32_t g_session_start_ms;
static uint32_t g_manual_until_ms;
static uint32_t g_last_trip_tick_ms;
static uint32_t g_last_distance_tick_ms;
static double g_session_distance_m;
static db_log_hardwired_stats_t g_hardwired_stats;

static TickType_t g_last_hardwired_publish_tick;
static int g_last_hardwired_depth_mm = -1;
static int g_last_hardwired_raw_mm = -1;
static bool g_last_hardwired_zero_run;
static bool g_last_hardwired_hold;
static TickType_t g_last_deeper_track_tick;
static int g_last_deeper_depth_mm = -1;
static bool g_last_deeper_fix;
static bool g_last_deeper_coordinates;
static double g_last_deeper_latitude;
static double g_last_deeper_longitude;

static uint32_t db_log_now_ms(void) {
  return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

static int db_log_abs_i32(int value) { return value < 0 ? -value : value; }
static double db_log_abs_double(double value) {
  return value < 0.0 ? -value : value;
}

const char *db_sonar_log_diag_stage_name(uint32_t stage) {
  switch (stage) {
  case DB_LOG_STAGE_IDLE: return "idle";
  case DB_LOG_STAGE_BOOT: return "boot";
  case DB_LOG_STAGE_SESSION_SAMPLE: return "session_sample";
  case DB_LOG_STAGE_GPS_ENTER: return "gps_enter";
  case DB_LOG_STAGE_GPS_SNAPSHOT: return "gps_snapshot";
  case DB_LOG_STAGE_GPS_FORMATTED: return "gps_formatted";
  case DB_LOG_STAGE_APPEND_ENTER: return "append_enter";
  case DB_LOG_STAGE_APPEND_SPACE_READY: return "append_space_ready";
  case DB_LOG_STAGE_APPEND_FILE_OPEN: return "append_file_open";
  case DB_LOG_STAGE_APPEND_LINE_WRITTEN: return "append_line_written";
  case DB_LOG_STAGE_APPEND_FILE_CLOSED: return "append_file_closed";
  case DB_LOG_STAGE_APPEND_USAGE_REFRESHED: return "append_usage_refreshed";
  default: return "unknown";
  }
}

static void db_log_diag_init(void) {
  if (g_log_diag_initialized) return;
  const bool valid = g_log_rtc_diag.magic == DB_LOG_DIAG_MAGIC;
  memset(&g_log_boot_diag, 0, sizeof(g_log_boot_diag));
  g_log_boot_diag.reset_reason = (uint32_t)esp_reset_reason();
  if (valid) {
    g_log_boot_diag.previous_stage = g_log_rtc_diag.stage;
    g_log_boot_diag.previous_sequence = g_log_rtc_diag.sequence;
    g_log_boot_diag.previous_min_stack =
        g_log_rtc_diag.min_stack == UINT32_MAX ? 0 : g_log_rtc_diag.min_stack;
  } else {
    memset(&g_log_rtc_diag, 0, sizeof(g_log_rtc_diag));
    g_log_rtc_diag.magic = DB_LOG_DIAG_MAGIC;
  }
  g_log_rtc_diag.stage = DB_LOG_STAGE_BOOT;
  g_log_rtc_diag.sequence++;
  g_log_rtc_diag.min_stack = UINT32_MAX;
  g_log_diag_initialized = true;
}

static void db_log_diag_mark(uint32_t stage) {
  db_log_diag_init();
  g_log_rtc_diag.stage = stage;
  g_log_rtc_diag.sequence++;
  uint32_t stack = (uint32_t)uxTaskGetStackHighWaterMark(NULL);
  if (stack < g_log_rtc_diag.min_stack) g_log_rtc_diag.min_stack = stack;
}

void db_sonar_log_get_crash_diag(db_sonar_log_crash_diag_t *diag) {
  if (diag == NULL) return;
  db_log_diag_init();
  *diag = g_log_boot_diag;
  diag->current_stage = g_log_rtc_diag.stage;
  diag->current_sequence = g_log_rtc_diag.sequence;
  diag->current_min_stack =
      g_log_rtc_diag.min_stack == UINT32_MAX ? 0 : g_log_rtc_diag.min_stack;
}

static void db_log_init_mutex(void) {
  if (g_log_mutex == NULL) g_log_mutex = xSemaphoreCreateMutex();
}

static size_t db_log_file_size(const char *path) {
  struct stat st = {0};
  return path != NULL && stat(path, &st) == 0 ? (size_t)st.st_size : 0;
}

static char db_log_stream_prefix(db_log_stream_t stream) {
  switch (stream) {
  case DB_LOG_STREAM_TRIP: return 'T';
  case DB_LOG_STREAM_HARDWIRED: return 'H';
  case DB_LOG_STREAM_DEEPER: return 'D';
  default: return '\0';
  }
}

static bool db_log_make_session_path(char *path, size_t path_size,
                                     db_log_stream_t stream, uint32_t id,
                                     bool active) {
  char prefix = db_log_stream_prefix(stream);
  if (path == NULL || path_size == 0 || prefix == '\0' || id == 0 ||
      id > 9999999U) return false;
  int length = snprintf(path, path_size, DB_SONAR_LOG_MOUNT_POINT "/%c%07lu.%s",
                        prefix, (unsigned long)id, active ? "TMP" : "LOG");
  return length > 0 && (size_t)length < path_size;
}

static bool db_log_parse_session_name(const char *name, char required_prefix,
                                      uint32_t *id, bool *active) {
  if (name == NULL || strlen(name) != 12 || name[0] != required_prefix ||
      name[8] != '.') return false;
  for (size_t i = 1; i < 8; ++i) {
    if (name[i] < '0' || name[i] > '9') return false;
  }
  bool is_log = strcmp(&name[9], "LOG") == 0;
  bool is_tmp = strcmp(&name[9], "TMP") == 0;
  if (!is_log && !is_tmp) return false;
  if (id != NULL) *id = (uint32_t)strtoul(&name[1], NULL, 10);
  if (active != NULL) *active = is_tmp;
  return true;
}

static esp_err_t db_log_refresh_usage_locked(void) {
  if (!g_log_available) return ESP_ERR_INVALID_STATE;
#ifdef CONFIG_DB_LOG_STORAGE_FATFS
  uint64_t total = 0;
  uint64_t free_bytes = 0;
  esp_err_t err =
      esp_vfs_fat_info(DB_SONAR_LOG_MOUNT_POINT, &total, &free_bytes);
  if (err == ESP_OK) {
    g_partition_total = (size_t)total;
    g_partition_free = (size_t)free_bytes;
    g_partition_used = (size_t)(total - free_bytes);
  }
#else
  size_t total = 0;
  size_t used = 0;
  esp_err_t err =
      esp_spiffs_info(DB_SONAR_LOG_PARTITION_LABEL, &total, &used);
  if (err == ESP_OK) {
    g_partition_total = total;
    g_partition_used = used;
    g_partition_free = total - used;
  }
#endif
  return err;
}

/* Sum of every file the directory can still name. Compared against
 * g_partition_used, this is the orphaned-cluster detector. */
static size_t db_log_visible_bytes_locked(void) {
  DIR *dir = opendir(DB_SONAR_LOG_MOUNT_POINT);
  if (dir == NULL) return 0;
  size_t total = 0;
  struct dirent *entry;
  char path[300];
  while ((entry = readdir(dir)) != NULL) {
    if (entry->d_name[0] == 0x2E) continue;
    snprintf(path, sizeof(path), DB_SONAR_LOG_MOUNT_POINT "/%s",
             entry->d_name);
    total += db_log_file_size(path);
  }
  closedir(dir);
  return total;
}

static size_t db_log_required_free_locked(void) {
  size_t image_bytes = db_log_file_size(DB_FC_FLASH_IMAGE_PATH);
  size_t unfilled_reserve = image_bytes >= (size_t)DB_FC_FLASH_RESERVE_BYTES
                                ? 0
                                : (size_t)DB_FC_FLASH_RESERVE_BYTES - image_bytes;
  /* The FC parameter backup shares this mount and must not be eaten by the
   * session pool - it is the way back from a bad reflash. Same shape as the
   * firmware reserve: only the part not already on disk still needs holding. */
  size_t param_bytes = db_log_file_size(DB_FC_PARAM_FILE_PATH);
  size_t unfilled_param = param_bytes >= (size_t)DB_FC_PARAM_RESERVE_BYTES
                              ? 0
                              : (size_t)DB_FC_PARAM_RESERVE_BYTES - param_bytes;
  return (size_t)DB_LOG_HEADROOM_BYTES + unfilled_reserve + unfilled_param;
}

static void db_log_delete_session_files_locked(uint32_t id) {
  char path[40];
  for (db_log_stream_t stream = DB_LOG_STREAM_TRIP;
       stream <= DB_LOG_STREAM_DEEPER; ++stream) {
    if (db_log_make_session_path(path, sizeof(path), stream, id, false))
      remove(path);
    if (db_log_make_session_path(path, sizeof(path), stream, id, true))
      remove(path);
  }
}

static uint32_t db_log_oldest_completed_session_locked(void) {
  DIR *dir = opendir(DB_SONAR_LOG_MOUNT_POINT);
  if (dir == NULL) return 0;
  uint32_t oldest = 0;
  struct dirent *entry;
  while ((entry = readdir(dir)) != NULL) {
    uint32_t id = 0;
    bool active = false;
    if (db_log_parse_session_name(entry->d_name, 'T', &id, &active) &&
        !active && !(g_session_active && id == g_session_id) &&
        (oldest == 0 || id < oldest)) oldest = id;
  }
  closedir(dir);
  return oldest;
}

static esp_err_t db_log_prepare_space_locked(size_t incoming_bytes) {
  esp_err_t err = db_log_refresh_usage_locked();
  if (err != ESP_OK) return err;
  size_t required = db_log_required_free_locked();
  uint32_t last_evicted = 0;
  while (g_partition_free < required + incoming_bytes) {
    uint32_t oldest = db_log_oldest_completed_session_locked();
    /* An eviction that removes nothing (remove() failing on a corrupt FAT)
     * must not spin here forever under the mutex: the same id coming back,
     * or free space not growing, both mean give up rather than hang. */
    if (oldest == 0 || oldest == last_evicted) return ESP_ERR_NO_MEM;
    size_t free_before = g_partition_free;
    db_log_delete_session_files_locked(oldest);
    err = db_log_refresh_usage_locked();
    if (err != ESP_OK) return err;
    if (g_partition_free <= free_before) return ESP_ERR_NO_MEM;
    last_evicted = oldest;
    g_evicted_sessions++;
    ESP_LOGW(TAG, "Evicted complete logging session %lu to preserve reserves",
             (unsigned long)oldest);
  }
  return ESP_OK;
}

static esp_err_t db_log_compact_system_locked(void) {
  FILE *source = fopen(DB_LOG_SYSTEM_PATH, "rb");
  if (source == NULL) return ESP_OK;
  FILE *tmp = fopen(DB_LOG_SYSTEM_TMP_PATH, "wb");
  if (tmp == NULL) {
    fclose(source);
    return ESP_FAIL;
  }
  size_t file_size = db_log_file_size(DB_LOG_SYSTEM_PATH);
  long offset = file_size > DB_LOG_SYSTEM_TRIM_BYTES
                    ? (long)(file_size - DB_LOG_SYSTEM_TRIM_BYTES)
                    : 0;
  if (offset > 0 && fseek(source, offset, SEEK_SET) != 0) {
    fclose(source); fclose(tmp); remove(DB_LOG_SYSTEM_TMP_PATH); return ESP_FAIL;
  }
  if (offset > 0) {
    int ch;
    while ((ch = fgetc(source)) != EOF && ch != '\n') {}
  }
  size_t count;
  while ((count = fread(g_io_buffer, 1, sizeof(g_io_buffer), source)) > 0) {
    if (fwrite(g_io_buffer, 1, count, tmp) != count) {
      fclose(source); fclose(tmp); remove(DB_LOG_SYSTEM_TMP_PATH); return ESP_FAIL;
    }
  }
  fclose(source); fclose(tmp);
  if (remove(DB_LOG_SYSTEM_PATH) != 0 ||
      rename(DB_LOG_SYSTEM_TMP_PATH, DB_LOG_SYSTEM_PATH) != 0) {
    remove(DB_LOG_SYSTEM_TMP_PATH);
    return ESP_FAIL;
  }
  return ESP_OK;
}

static esp_err_t db_log_append_line_locked(db_log_stream_t stream,
                                           const char *line) {
  if (!g_log_available || line == NULL) return ESP_ERR_INVALID_STATE;
  char path[40];
  if (stream == DB_LOG_STREAM_SYSTEM) {
    snprintf(path, sizeof(path), "%s", DB_LOG_SYSTEM_PATH);
    if (db_log_file_size(path) + strlen(line) + 1 > DB_LOG_SYSTEM_LIMIT_BYTES) {
      esp_err_t compact_err = db_log_compact_system_locked();
      if (compact_err != ESP_OK) return compact_err;
    }
  } else {
    if (!g_session_active ||
        !db_log_make_session_path(path, sizeof(path), stream, g_session_id,
                                  true)) return ESP_ERR_INVALID_STATE;
  }
  db_log_diag_mark(DB_LOG_STAGE_APPEND_ENTER);
  size_t length = strlen(line);
  esp_err_t err = db_log_prepare_space_locked(length + 1);
  if (err != ESP_OK) {
    g_last_write_errno = err;
    g_last_write_fail_stage = 1;
    return err;
  }
  db_log_diag_mark(DB_LOG_STAGE_APPEND_SPACE_READY);
  errno = 0;
  FILE *fp = fopen(path, "ab");
  if (fp == NULL) {
    g_last_write_errno = errno;
    g_last_write_fail_stage = 2;
    return ESP_FAIL;
  }
  db_log_diag_mark(DB_LOG_STAGE_APPEND_FILE_OPEN);
  errno = 0;
  bool ok = fwrite(line, 1, length, fp) == length && fputc('\n', fp) != EOF;
  if (!ok) {
    g_last_write_errno = errno;
    g_last_write_fail_stage = 3;
  }
  db_log_diag_mark(DB_LOG_STAGE_APPEND_LINE_WRITTEN);
  fclose(fp);
  db_log_diag_mark(DB_LOG_STAGE_APPEND_FILE_CLOSED);
  db_log_refresh_usage_locked();
  db_log_diag_mark(DB_LOG_STAGE_APPEND_USAGE_REFRESHED);
  db_log_diag_mark(DB_LOG_STAGE_IDLE);
  return ok ? ESP_OK : ESP_FAIL;
}

/* Callers pass the line WITHOUT the boot_ms prefix - this function stamps it.
 *
 * The timestamp must be taken inside the same critical section that fixes
 * write order. When callers stamped it themselves, db_log_now_ms() was a
 * varargs argument evaluated BEFORE xSemaphoreTake(), so two producers could
 * both stamp, then contend, and the loser wrote its earlier timestamp after
 * the winner's later one. Measured 21-08-2026: boot_ms went backwards 6 times
 * (7-144 ms) in one Deeper session, which is the stream with genuinely
 * concurrent producers. Output format is unchanged. */
static esp_err_t db_log_vappendf(db_log_stream_t stream, const char *fmt,
                                 va_list args) {
  if (fmt == NULL) return ESP_ERR_INVALID_ARG;
  db_log_init_mutex();
  if (g_log_mutex == NULL) return ESP_ERR_NO_MEM;
  xSemaphoreTake(g_log_mutex, portMAX_DELAY);
  int prefix = snprintf(g_line_buffer, sizeof(g_line_buffer), "boot_ms=%lu ",
                        (unsigned long)db_log_now_ms());
  if (prefix < 0 || (size_t)prefix >= sizeof(g_line_buffer)) {
    xSemaphoreGive(g_log_mutex);
    return ESP_FAIL;
  }
  vsnprintf(g_line_buffer + prefix, sizeof(g_line_buffer) - (size_t)prefix, fmt,
            args);
  esp_err_t err = db_log_append_line_locked(stream, g_line_buffer);
  xSemaphoreGive(g_log_mutex);
  return err;
}

static esp_err_t db_log_appendf(db_log_stream_t stream, const char *fmt, ...) {
  va_list args;
  va_start(args, fmt);
  esp_err_t err = db_log_vappendf(stream, fmt, args);
  va_end(args);
  return err;
}

static void db_log_reset_source_state_locked(void) {
  memset(&g_hardwired_stats, 0, sizeof(g_hardwired_stats));
  g_hardwired_stats.active = true;
  g_hardwired_stats.armed_state_seen = true;
  g_hardwired_stats.start_ms = db_log_now_ms();
  g_last_hardwired_publish_tick = 0;
  g_last_hardwired_depth_mm = -1;
  g_last_hardwired_raw_mm = -1;
  g_last_hardwired_zero_run = false;
  g_last_hardwired_hold = false;
  g_last_deeper_track_tick = 0;
  g_last_deeper_depth_mm = -1;
  g_last_deeper_fix = false;
  g_last_deeper_coordinates = false;
}

static esp_err_t db_log_start_session_locked(const char *reason) {
  if (g_session_active) return ESP_OK;
  if (g_next_session_id == 0 || g_next_session_id > 9999999U)
    g_next_session_id = 1;
  g_session_id = g_next_session_id++;
  g_session_active = true;
  g_session_start_ms = db_log_now_ms();
  g_last_trip_tick_ms = 0;
  g_last_distance_tick_ms = g_session_start_ms;
  g_session_distance_m = 0.0;
  db_log_reset_source_state_locked();
  snprintf(g_line_buffer, sizeof(g_line_buffer),
           "boot_ms=%lu event=session_start session=%lu reason=%s",
           (unsigned long)g_session_start_ms, (unsigned long)g_session_id,
           reason == NULL ? "unknown" : reason);
  esp_err_t err = db_log_append_line_locked(DB_LOG_STREAM_TRIP, g_line_buffer);
  if (err != ESP_OK) {
    g_session_active = false;
    return err;
  }
  snprintf(g_line_buffer, sizeof(g_line_buffer),
           "boot_ms=%lu event=session_start session=%lu reason=%s",
           (unsigned long)g_session_start_ms, (unsigned long)g_session_id,
           reason == NULL ? "unknown" : reason);
  (void)db_log_append_line_locked(DB_LOG_STREAM_SYSTEM, g_line_buffer);
  return ESP_OK;
}

/* Session-end evidence must never be lost to a full filesystem: write it
 * WITHOUT the space check. ~200 B rides on the permanent 256 KiB headroom,
 * so a session that closes with reason=storage_full still records why. */
static void db_log_append_final_line_locked(db_log_stream_t stream,
                                            const char *line) {
  if (!g_log_available || line == NULL) return;
  char path[40];
  if (stream == DB_LOG_STREAM_SYSTEM) {
    snprintf(path, sizeof(path), "%s", DB_LOG_SYSTEM_PATH);
  } else if (!g_session_active ||
             !db_log_make_session_path(path, sizeof(path), stream, g_session_id,
                                       true)) {
    return;
  }
  FILE *fp = fopen(path, "ab");
  if (fp == NULL) return;
  size_t length = strlen(line);
  if (fwrite(line, 1, length, fp) == length) (void)fputc('\n', fp);
  fclose(fp);
}

static void db_log_finish_session_locked(const char *reason) {
  if (!g_session_active) return;
  uint32_t now = db_log_now_ms();
  snprintf(g_line_buffer, sizeof(g_line_buffer),
           "boot_ms=%lu event=session_end session=%lu reason=%s elapsed_ms=%lu "
           "distance_m=%.2f",
           (unsigned long)now, (unsigned long)g_session_id,
           reason == NULL ? "unknown" : reason,
           (unsigned long)(now - g_session_start_ms), g_session_distance_m);
  db_log_append_final_line_locked(DB_LOG_STREAM_TRIP, g_line_buffer);

  char from[40];
  char to[40];
  for (db_log_stream_t stream = DB_LOG_STREAM_TRIP;
       stream <= DB_LOG_STREAM_DEEPER; ++stream) {
    if (db_log_make_session_path(from, sizeof(from), stream, g_session_id, true) &&
        db_log_make_session_path(to, sizeof(to), stream, g_session_id, false) &&
        db_log_file_size(from) > 0) {
      remove(to);
      if (rename(from, to) != 0)
        ESP_LOGE(TAG, "Failed to close session file %s", from);
    }
  }
  /* The rename loop does not touch g_line_buffer: the formatted end line is
   * still valid for the SYSTEM copy. */
  db_log_append_final_line_locked(DB_LOG_STREAM_SYSTEM, g_line_buffer);
  g_session_active = false;
  g_session_id = 0;
  g_manual_until_ms = 0;
  g_hardwired_stats.active = false;
}

static uint32_t db_log_recover_and_find_next_locked(void) {
  DIR *dir = opendir(DB_SONAR_LOG_MOUNT_POINT);
  if (dir == NULL) return 0;
  uint32_t max_id = 0;
  uint32_t recovered = 0;
  struct dirent *entry;
  while ((entry = readdir(dir)) != NULL) {
    uint32_t id = 0;
    bool active = false;
    bool matched = false;
    for (const char *prefix = "THD"; *prefix != '\0'; ++prefix) {
      if (db_log_parse_session_name(entry->d_name, *prefix, &id, &active)) {
        matched = true;
        break;
      }
    }
    if (!matched) continue;
    if (id > max_id) max_id = id;
    if (active) {
      char from[40];
      char to[40];
      db_log_stream_t stream = entry->d_name[0] == 'T'
                                   ? DB_LOG_STREAM_TRIP
                                   : entry->d_name[0] == 'H'
                                         ? DB_LOG_STREAM_HARDWIRED
                                         : DB_LOG_STREAM_DEEPER;
      if (db_log_make_session_path(from, sizeof(from), stream, id, true) &&
          db_log_make_session_path(to, sizeof(to), stream, id, false)) {
        remove(to);
        if (rename(from, to) == 0 && stream == DB_LOG_STREAM_TRIP) recovered++;
      }
    }
  }
  closedir(dir);
  g_next_session_id = max_id >= 9999999U ? 1 : max_id + 1;
  return recovered;
}

bool db_sonar_log_is_available(void) { return g_log_available; }

esp_err_t db_sonar_log_init(void) {
  db_log_diag_init();
  db_log_init_mutex();
  if (g_log_mutex == NULL) return ESP_ERR_NO_MEM;
  xSemaphoreTake(g_log_mutex, portMAX_DELAY);
  if (g_log_available) {
    xSemaphoreGive(g_log_mutex);
    return ESP_OK;
  }
#ifdef CONFIG_DB_LOG_STORAGE_FATFS
  esp_vfs_fat_mount_config_t config = {
      .format_if_mount_failed = true,
      .max_files = 10,
      .allocation_unit_size = 4096,
      .disk_status_check_enable = false,
      .use_one_fat = false,
  };
  esp_err_t err = esp_vfs_fat_spiflash_mount_rw_wl(
      DB_SONAR_LOG_MOUNT_POINT, DB_SONAR_LOG_PARTITION_LABEL, &config,
      &g_log_wl_handle);
#else
  esp_vfs_spiffs_conf_t config = {.base_path = DB_SONAR_LOG_MOUNT_POINT,
                                  .partition_label = DB_SONAR_LOG_PARTITION_LABEL,
                                  .max_files = 10,
                                  .format_if_mount_failed = true};
  esp_err_t err = esp_vfs_spiffs_register(&config);
#endif
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to mount logging partition (%s)", esp_err_to_name(err));
    xSemaphoreGive(g_log_mutex);
    return err;
  }
  g_log_available = true;
  err = db_log_refresh_usage_locked();
  uint32_t recovered = db_log_recover_and_find_next_locked();
  errno = 0;
  FILE *system = fopen(DB_LOG_SYSTEM_PATH, "ab");
  bool healed_by_format = false;
#ifdef CONFIG_DB_LOG_STORAGE_FATFS
  if (system == NULL && errno == EACCES) {
    /* Field-diagnosed 20-08-2026 (boat VLLR): hard power cuts during
     * wear-level sector moves can leave FAT root-directory sectors erased
     * (0xFF). FatFS reads 0xFF entries as OCCUPIED, so every create fails
     * FR_DENIED/EACCES forever while reads still work. The volume cannot be
     * repaired through the FatFS API, so a sealed boat would silently never
     * log again. Self-heal: format, remount state, continue. Everything on
     * the volume is lost - by this point the volume was already unwritable,
     * and the boat's job is to record the NEXT session, not to preserve a
     * corrupt one. */
    ESP_LOGE(TAG, "Logs volume corrupt (create=EACCES): formatting to self-heal");
    esp_err_t fmt = esp_vfs_fat_spiflash_format_rw_wl(
        DB_SONAR_LOG_MOUNT_POINT, DB_SONAR_LOG_PARTITION_LABEL);
    if (fmt == ESP_OK) {
      healed_by_format = true;
      g_next_session_id = 1;
      g_last_write_errno = 0;
      g_last_write_fail_stage = 0;
      (void)db_log_refresh_usage_locked();
      errno = 0;
      system = fopen(DB_LOG_SYSTEM_PATH, "ab");
    } else {
      ESP_LOGE(TAG, "Self-heal format failed (%s)", esp_err_to_name(fmt));
    }
  }
#endif
  if (system != NULL) {
    fclose(system);
  } else {
    g_last_write_errno = errno;
    g_last_write_fail_stage = 4;
    ESP_LOGE(TAG, "SYSTEM.LOG create failed, errno=%d", errno);
  }
  if (healed_by_format) {
    snprintf(g_line_buffer, sizeof(g_line_buffer),
             "boot_ms=%lu event=fs_self_heal_format reason=create_eacces",
             (unsigned long)db_log_now_ms());
    (void)db_log_append_line_locked(DB_LOG_STREAM_SYSTEM, g_line_buffer);
  }
  if (recovered > 0) {
    snprintf(g_line_buffer, sizeof(g_line_buffer),
             "boot_ms=%lu event=session_recovery recovered=%lu",
             (unsigned long)db_log_now_ms(), (unsigned long)recovered);
    (void)db_log_append_line_locked(DB_LOG_STREAM_SYSTEM, g_line_buffer);
  }
  /* REPORT ONLY - never format on this heuristic. A wrong auto-format turns
   * a recoverable fault into a destroyed dataset; the 23-08 volume was still
   * fully carveable from a raw partition read. Formatting stays a deliberate
   * confirmed operator action (db_sonar_log_format_volume). */
  {
    size_t visible = db_log_visible_bytes_locked();
    if (g_partition_used > visible + DB_LOG_ORPHAN_ALARM_BYTES) {
      snprintf(g_line_buffer, sizeof(g_line_buffer),
               "boot_ms=%lu event=fs_orphaned used=%lu visible=%lu "
               "lost=%lu free=%lu",
               (unsigned long)db_log_now_ms(),
               (unsigned long)g_partition_used, (unsigned long)visible,
               (unsigned long)(g_partition_used - visible),
               (unsigned long)g_partition_free);
      (void)db_log_append_line_locked(DB_LOG_STREAM_SYSTEM, g_line_buffer);
      ESP_LOGE(TAG,
               "Logs volume has %lu bytes in unreachable clusters - the "
               "session pool cannot reclaim them. Pull "
               "/api/logs/raw/download, then POST /api/logs/format.",
               (unsigned long)(g_partition_used - visible));
    }
  }
  ESP_LOGI(TAG,
           "Logging filesystem ready: total=%u used=%u FC-reserve=%u headroom=%u",
           (unsigned)g_partition_total, (unsigned)g_partition_used,
           (unsigned)DB_FC_FLASH_RESERVE_BYTES, (unsigned)DB_LOG_HEADROOM_BYTES);
  xSemaphoreGive(g_log_mutex);
  return err;
}

void db_sonar_log_refresh_limits(void) {
  if (g_log_mutex == NULL) return;
  xSemaphoreTake(g_log_mutex, portMAX_DELAY);
  if (g_log_available) (void)db_log_refresh_usage_locked();
  xSemaphoreGive(g_log_mutex);
}

static void db_log_fill_session_info_locked(db_sonar_log_session_info_t *info,
                                            uint32_t id, bool active) {
  memset(info, 0, sizeof(*info));
  info->id = id;
  info->active = active;
  char path[40];
  for (db_log_stream_t stream = DB_LOG_STREAM_TRIP;
       stream <= DB_LOG_STREAM_DEEPER; ++stream) {
    db_log_make_session_path(path, sizeof(path), stream, id, active);
    size_t bytes = db_log_file_size(path);
    if (stream == DB_LOG_STREAM_TRIP) info->trip_bytes = bytes;
    else if (stream == DB_LOG_STREAM_HARDWIRED) info->hardwired_bytes = bytes;
    else info->deeper_bytes = bytes;
  }
}

static size_t db_log_list_sessions_locked(db_sonar_log_session_info_t *sessions,
                                          size_t capacity) {
  size_t count = 0;
  DIR *dir = opendir(DB_SONAR_LOG_MOUNT_POINT);
  if (dir == NULL) return 0;
  struct dirent *entry;
  while ((entry = readdir(dir)) != NULL) {
    uint32_t id = 0;
    bool active = false;
    if (!db_log_parse_session_name(entry->d_name, 'T', &id, &active)) continue;
    db_sonar_log_session_info_t item;
    db_log_fill_session_info_locked(&item, id, active);
    if (sessions != NULL &&
        (count < capacity || id > sessions[capacity - 1].id)) {
      size_t position = count < capacity ? count : capacity - 1;
      while (position > 0 && sessions[position - 1].id < id) {
        sessions[position] = sessions[position - 1];
        position--;
      }
      sessions[position] = item;
    }
    if (count < capacity) count++;
  }
  closedir(dir);
  return count;
}

esp_err_t db_sonar_log_list_sessions(db_sonar_log_session_info_t *sessions,
                                     size_t capacity, size_t *out_count) {
  if (sessions == NULL || capacity == 0 || out_count == NULL)
    return ESP_ERR_INVALID_ARG;
  db_log_init_mutex();
  if (g_log_mutex == NULL) return ESP_ERR_NO_MEM;
  xSemaphoreTake(g_log_mutex, portMAX_DELAY);
  if (!g_log_available) {
    xSemaphoreGive(g_log_mutex);
    return ESP_ERR_INVALID_STATE;
  }
  *out_count = db_log_list_sessions_locked(sessions, capacity);
  xSemaphoreGive(g_log_mutex);
  return ESP_OK;
}

esp_err_t db_sonar_log_get_status(db_sonar_log_status_t *status) {
  if (status == NULL) return ESP_ERR_INVALID_ARG;
  memset(status, 0, sizeof(*status));
  db_log_init_mutex();
  if (g_log_mutex == NULL) return ESP_ERR_NO_MEM;
  xSemaphoreTake(g_log_mutex, portMAX_DELAY);
  if (g_log_available) {
    db_log_refresh_usage_locked();
    status->mounted = true;
    status->partition_total_bytes = g_partition_total;
    status->partition_used_bytes = g_partition_used;
    status->partition_free_bytes = g_partition_free;
    status->fc_image_reserve_bytes = DB_FC_FLASH_RESERVE_BYTES;
    status->filesystem_headroom_bytes = DB_LOG_HEADROOM_BYTES;
    status->system_log_bytes = db_log_file_size(DB_LOG_SYSTEM_PATH);
    status->system_log_limit_bytes = DB_LOG_SYSTEM_LIMIT_BYTES;
    status->session_pool_limit_bytes =
        g_partition_total > DB_FC_FLASH_RESERVE_BYTES + DB_FC_PARAM_RESERVE_BYTES +
                            DB_LOG_HEADROOM_BYTES +
                                DB_LOG_SYSTEM_LIMIT_BYTES
            ? g_partition_total - DB_FC_FLASH_RESERVE_BYTES - DB_FC_PARAM_RESERVE_BYTES -
                  DB_LOG_HEADROOM_BYTES - DB_LOG_SYSTEM_LIMIT_BYTES
            : 0;
    status->legacy_log_bytes = db_log_file_size(DB_SONAR_LOG_LEGACY_PATH);
    /* Counted from a full directory scan - the 24-entry listing the UI shows
     * is a window, and the real on-disk count can exceed it. */
    DIR *dir = opendir(DB_SONAR_LOG_MOUNT_POINT);
    if (dir != NULL) {
      struct dirent *entry;
      while ((entry = readdir(dir)) != NULL) {
        uint32_t id = 0;
        bool active = false;
        if (db_log_parse_session_name(entry->d_name, 'T', &id, &active) &&
            !active)
          status->completed_session_count++;
      }
      closedir(dir);
    }
    status->evicted_session_count = g_evicted_sessions;
    status->session_active = g_session_active;
    status->active_session_id = g_session_id;
    uint32_t now = db_log_now_ms();
    status->active_session_age_ms =
        g_session_active ? now - g_session_start_ms : 0;
    status->manual_remaining_ms =
        g_manual_until_ms != 0 && (int32_t)(g_manual_until_ms - now) > 0
            ? g_manual_until_ms - now
            : 0;
    status->armed_capture_active = g_armed_capture;
    status->last_write_errno = g_last_write_errno;
    status->last_write_fail_stage = g_last_write_fail_stage;
  }
  xSemaphoreGive(g_log_mutex);
  return ESP_OK;
}

static bool db_log_resolve_stream_path_locked(char *path, size_t path_size,
                                              db_log_stream_t stream,
                                              uint32_t session_id) {
  if (stream == DB_LOG_STREAM_SYSTEM) {
    snprintf(path, path_size, "%s", DB_LOG_SYSTEM_PATH);
    return true;
  }
  if (stream == DB_LOG_STREAM_LEGACY) {
    snprintf(path, path_size, "%s", DB_SONAR_LOG_LEGACY_PATH);
    return true;
  }
  bool active = g_session_active && session_id == g_session_id;
  if (!db_log_make_session_path(path, path_size, stream, session_id, active))
    return false;
  if (db_log_file_size(path) == 0 && active) {
    db_log_make_session_path(path, path_size, stream, session_id, false);
  }
  return true;
}

esp_err_t db_sonar_log_stream(db_log_stream_t stream, uint32_t session_id,
                              db_sonar_log_chunk_writer_t writer, void *user_ctx,
                              size_t *out_bytes_streamed, int *out_file_errno) {
  if (writer == NULL) return ESP_ERR_INVALID_ARG;
  if (out_bytes_streamed != NULL) *out_bytes_streamed = 0;
  if (out_file_errno != NULL) *out_file_errno = 0;
  db_log_init_mutex();
  if (g_log_mutex == NULL) return ESP_ERR_NO_MEM;
  xSemaphoreTake(g_log_mutex, portMAX_DELAY);
  if (!g_log_available) {
    xSemaphoreGive(g_log_mutex);
    return ESP_ERR_INVALID_STATE;
  }
  char path[40];
  if (!db_log_resolve_stream_path_locked(path, sizeof(path), stream, session_id)) {
    xSemaphoreGive(g_log_mutex);
    return ESP_ERR_INVALID_ARG;
  }
  errno = 0;
  FILE *fp = fopen(path, "rb");
  if (fp == NULL) {
    if (out_file_errno != NULL) *out_file_errno = errno;
    xSemaphoreGive(g_log_mutex);
    return ESP_ERR_NOT_FOUND;
  }
  size_t total = 0;
  esp_err_t result = ESP_OK;
  while (true) {
    size_t count = fread(g_io_buffer, 1, sizeof(g_io_buffer), fp);
    if (count > 0) {
      result = writer(g_io_buffer, count, user_ctx);
      if (result != ESP_OK) break;
      total += count;
    }
    if (count < sizeof(g_io_buffer)) {
      if (ferror(fp)) {
        if (out_file_errno != NULL) *out_file_errno = errno;
        result = ESP_FAIL;
      }
      break;
    }
  }
  fclose(fp);
  xSemaphoreGive(g_log_mutex);
  if (out_bytes_streamed != NULL) *out_bytes_streamed = total;
  return result;
}

esp_err_t db_sonar_log_delete_session(uint32_t session_id) {
  if (session_id == 0) return ESP_ERR_INVALID_ARG;
  db_log_init_mutex();
  if (g_log_mutex == NULL) return ESP_ERR_NO_MEM;
  xSemaphoreTake(g_log_mutex, portMAX_DELAY);
  if (!g_log_available || (g_session_active && session_id == g_session_id)) {
    xSemaphoreGive(g_log_mutex);
    return ESP_ERR_INVALID_STATE;
  }
  db_log_delete_session_files_locked(session_id);
  db_log_refresh_usage_locked();
  xSemaphoreGive(g_log_mutex);
  return ESP_OK;
}

/* Reformat the whole logs volume. This is the ONLY path that reclaims
 * orphaned cluster chains: remove() needs a directory entry, so clear_all
 * cannot see them and will cheerfully report success having freed nothing.
 * Destroys every log on the volume - callers are expected to have pulled
 * /api/logs/raw/download first. */
esp_err_t db_sonar_log_format_volume(void) {
#ifndef CONFIG_DB_LOG_STORAGE_FATFS
  return ESP_ERR_NOT_SUPPORTED;
#else
  db_log_init_mutex();
  if (g_log_mutex == NULL) return ESP_ERR_NO_MEM;
  xSemaphoreTake(g_log_mutex, portMAX_DELAY);
  if (!g_log_available || g_session_active) {
    xSemaphoreGive(g_log_mutex);
    return ESP_ERR_INVALID_STATE;
  }
  esp_err_t err = esp_vfs_fat_spiflash_format_rw_wl(
      DB_SONAR_LOG_MOUNT_POINT, DB_SONAR_LOG_PARTITION_LABEL);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Logs volume format failed (%s)", esp_err_to_name(err));
    xSemaphoreGive(g_log_mutex);
    return err;
  }
  g_next_session_id = 1;
  g_evicted_sessions = 0;
  g_last_write_errno = 0;
  g_last_write_fail_stage = 0;
  (void)db_log_refresh_usage_locked();
  errno = 0;
  FILE *system = fopen(DB_LOG_SYSTEM_PATH, "ab");
  if (system != NULL) {
    fclose(system);
  } else {
    g_last_write_errno = errno;
    g_last_write_fail_stage = 4;
  }
  snprintf(g_line_buffer, sizeof(g_line_buffer),
           "boot_ms=%lu event=fs_manual_format used=%lu free=%lu",
           (unsigned long)db_log_now_ms(), (unsigned long)g_partition_used,
           (unsigned long)g_partition_free);
  (void)db_log_append_line_locked(DB_LOG_STREAM_SYSTEM, g_line_buffer);
  ESP_LOGW(TAG, "Logs volume reformatted: used=%u free=%u",
           (unsigned)g_partition_used, (unsigned)g_partition_free);
  xSemaphoreGive(g_log_mutex);
  return ESP_OK;
#endif
}

esp_err_t db_sonar_log_clear_all(void) {
  db_log_init_mutex();
  if (g_log_mutex == NULL) return ESP_ERR_NO_MEM;
  xSemaphoreTake(g_log_mutex, portMAX_DELAY);
  if (!g_log_available || g_session_active) {
    xSemaphoreGive(g_log_mutex);
    return ESP_ERR_INVALID_STATE;
  }
  DIR *dir = opendir(DB_SONAR_LOG_MOUNT_POINT);
  if (dir != NULL) {
    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL) {
      uint32_t id;
      bool active;
      bool session_file = false;
      for (const char *prefix = "THD"; *prefix != '\0'; ++prefix)
        session_file |= db_log_parse_session_name(entry->d_name, *prefix, &id,
                                                  &active);
      if (session_file) {
        char path[300];
        snprintf(path, sizeof(path), DB_SONAR_LOG_MOUNT_POINT "/%s",
                 entry->d_name);
        remove(path);
      }
    }
    closedir(dir);
  }
  remove(DB_LOG_SYSTEM_PATH);
  remove(DB_LOG_SYSTEM_TMP_PATH);
  remove(DB_SONAR_LOG_LEGACY_PATH);
  FILE *system = fopen(DB_LOG_SYSTEM_PATH, "wb");
  if (system != NULL) fclose(system);
  g_evicted_sessions = 0;
  db_log_refresh_usage_locked();
  xSemaphoreGive(g_log_mutex);
  return ESP_OK;
}

esp_err_t db_sonar_log_start_manual_capture(uint32_t duration_seconds,
                                            uint32_t *out_session_id) {
  if (duration_seconds < DB_LOG_MANUAL_MIN_SECONDS)
    duration_seconds = DB_LOG_MANUAL_MIN_SECONDS;
  if (duration_seconds > DB_LOG_MANUAL_MAX_SECONDS)
    duration_seconds = DB_LOG_MANUAL_MAX_SECONDS;
  db_log_init_mutex();
  if (g_log_mutex == NULL) return ESP_ERR_NO_MEM;
  xSemaphoreTake(g_log_mutex, portMAX_DELAY);
  if (!g_log_available) {
    xSemaphoreGive(g_log_mutex);
    return ESP_ERR_INVALID_STATE;
  }
  esp_err_t err = db_log_start_session_locked("manual");
  if (err == ESP_OK) {
    g_manual_until_ms = db_log_now_ms() + duration_seconds * 1000U;
    if (out_session_id != NULL) *out_session_id = g_session_id;
  }
  xSemaphoreGive(g_log_mutex);
  return err;
}

esp_err_t db_sonar_log_stop_manual_capture(void) {
  db_log_init_mutex();
  if (g_log_mutex == NULL) return ESP_ERR_NO_MEM;
  xSemaphoreTake(g_log_mutex, portMAX_DELAY);
  g_manual_until_ms = 0;
  if (g_session_active && !g_armed_capture)
    db_log_finish_session_locked("manual_stop");
  xSemaphoreGive(g_log_mutex);
  return ESP_OK;
}

void db_sonar_log_note_fc_armed_state(bool armed) {
  if (!g_log_available || g_log_mutex == NULL) return;
  xSemaphoreTake(g_log_mutex, portMAX_DELAY);
  bool changed = armed != g_armed_capture;
  g_armed_capture = armed;
  if (armed && !g_session_active)
    (void)db_log_start_session_locked("armed");
  else if (!armed && changed && g_session_active && g_manual_until_ms == 0)
    db_log_finish_session_locked("disarmed");
  xSemaphoreGive(g_log_mutex);
}

void db_sonar_log_tick(void) {
  if (!g_log_available || g_log_mutex == NULL) return;
  db_mavlink_get_telemetry(&g_tick_telemetry);
  uint32_t now = db_log_now_ms();
  xSemaphoreTake(g_log_mutex, portMAX_DELAY);
  if (g_manual_until_ms != 0 && (int32_t)(now - g_manual_until_ms) >= 0) {
    g_manual_until_ms = 0;
    if (g_session_active && !g_armed_capture) {
      db_log_finish_session_locked("manual_timeout");
      xSemaphoreGive(g_log_mutex);
      return;
    }
  }
  if (!g_session_active ||
      (g_last_trip_tick_ms != 0 && now - g_last_trip_tick_ms < DB_LOG_TRIP_PERIOD_MS)) {
    xSemaphoreGive(g_log_mutex);
    return;
  }
  uint32_t delta_ms = now - g_last_distance_tick_ms;
  if (delta_ms <= 5000U && g_tick_telemetry.vfr.valid &&
      g_tick_telemetry.vfr.age_ms >= 0 && g_tick_telemetry.vfr.age_ms <= 3000 &&
      g_tick_telemetry.vfr.groundspeed_mps >= 0.0f &&
      g_tick_telemetry.vfr.groundspeed_mps < 20.0f) {
    g_session_distance_m +=
        (double)g_tick_telemetry.vfr.groundspeed_mps * (double)delta_ms / 1000.0;
  }
  g_last_distance_tick_ms = now;
  int voltage_mv = g_tick_telemetry.battery.valid
                       ? g_tick_telemetry.battery.voltage_mv
                       : g_tick_telemetry.system.valid
                             ? g_tick_telemetry.system.voltage_mv
                             : -1;
  snprintf(g_line_buffer, sizeof(g_line_buffer),
           "boot_ms=%lu event=trip_sample session=%lu armed=%d voltage_mv=%d "
           "current_ca=%d consumed_mah=%ld remaining_pct=%d speed_mps=%.3f "
           "throttle_pct=%u distance_m=%.2f lat_e7=%ld lon_e7=%ld gps_fix=%u "
           "gps_sats=%u gps_age_ms=%lld unix_usec=%llu",
           (unsigned long)now, (unsigned long)g_session_id,
           g_armed_capture ? 1 : 0, voltage_mv,
           g_tick_telemetry.battery.valid ? g_tick_telemetry.battery.current_ca : -1,
           (long)(g_tick_telemetry.battery.valid
                      ? g_tick_telemetry.battery.consumed_mah
                      : -1),
           g_tick_telemetry.battery.valid
               ? g_tick_telemetry.battery.remaining_pct
               : -1,
           g_tick_telemetry.vfr.valid ? g_tick_telemetry.vfr.groundspeed_mps : -1.0,
           g_tick_telemetry.vfr.valid ? g_tick_telemetry.vfr.throttle_pct : 0,
           g_session_distance_m,
           (long)(g_tick_telemetry.gps.valid ? g_tick_telemetry.gps.latitude_e7 : 0),
           (long)(g_tick_telemetry.gps.valid ? g_tick_telemetry.gps.longitude_e7 : 0),
           g_tick_telemetry.gps.valid ? g_tick_telemetry.gps.fix_type : 0,
           g_tick_telemetry.gps.valid
               ? g_tick_telemetry.gps.satellites_visible
               : 0,
           (long long)g_tick_telemetry.gps.age_ms,
           (unsigned long long)(g_tick_telemetry.time.valid
                                    ? g_tick_telemetry.time.unix_usec
                                    : 0));
  esp_err_t err = db_log_append_line_locked(DB_LOG_STREAM_TRIP, g_line_buffer);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Trip logging stopped: %s", esp_err_to_name(err));
    db_log_finish_session_locked("storage_full");
  } else {
    g_last_trip_tick_ms = now;
  }
  xSemaphoreGive(g_log_mutex);
}

/* Free-form key=value line into SYSTEM.LOG. Unlike
 * db_sonar_log_log_system_event() the caller owns the whole field list, so
 * the result stays parseable as `key=value` pairs instead of collapsing
 * everything into one detail= blob. Blocks on the logging mutex - call it
 * from an ordinary task, never from an event handler (see db_netlog.h). */
void db_sonar_log_log_system_kv(const char *fmt, ...) {
  if (!g_log_available || fmt == NULL) return;
  va_list args;
  va_start(args, fmt);
  (void)db_log_vappendf(DB_LOG_STREAM_SYSTEM, fmt, args);
  va_end(args);
}

void db_sonar_log_log_system_event(const char *event, const char *detail) {
  if (!g_log_available) return;
  (void)db_log_appendf(DB_LOG_STREAM_SYSTEM,
                       "event=%s detail=%s",
                       event == NULL ? "system" : event,
                       detail == NULL ? "n/a" : detail);
}

static const char *db_log_source_name(db_sonar_source_t source) {
  if (source == DB_SONAR_SOURCE_HARDWIRED) return "hardwired";
  if (source == DB_SONAR_SOURCE_DEEPER) return "deeper";
  return "none";
}

void db_sonar_log_log_boot(db_sonar_source_t active_source, int boot_radio_mode,
                           bool deeper_connected, bool force_update_ap_mode,
                           bool web_fs_available) {
  if (!g_log_available) return;
  db_sonar_log_crash_diag_t diag = {0};
  db_sonar_log_get_crash_diag(&diag);
  (void)db_log_appendf(
      DB_LOG_STREAM_SYSTEM,
      "event=boot source=%s ss_type=%s radio_mode=%d deeper_connected=%d "
      "force_update_ap=%d web_fs=%d reset_reason=%lu prev_stage=%lu "
      "prev_stage_name=%s prev_seq=%lu prev_min_stack=%lu",
      db_log_source_name(active_source),
      sonar_driver_name((sonar_model_t)DB_PARAM_SONAR_TYPE),
      boot_radio_mode, deeper_connected ? 1 : 0, force_update_ap_mode ? 1 : 0,
      web_fs_available ? 1 : 0, (unsigned long)diag.reset_reason,
      (unsigned long)diag.previous_stage,
      db_sonar_log_diag_stage_name(diag.previous_stage),
      (unsigned long)diag.previous_sequence,
      (unsigned long)diag.previous_min_stack);
}

static void db_log_format_fc_gps(char *dst, size_t size) {
  if (dst == NULL || size == 0) return;
  db_log_diag_mark(DB_LOG_STAGE_GPS_ENTER);
  db_mavlink_gps_state_t gps = {0};
  db_mavlink_get_gps_state(&gps);
  db_log_diag_mark(DB_LOG_STAGE_GPS_SNAPSHOT);
  if (!gps.valid || gps.fix_type < 3 || gps.age_ms < 0 ||
      gps.age_ms > DB_LOG_FC_GPS_MAX_AGE_MS) {
    snprintf(dst, size, "fc_gps=no_location");
  } else {
    snprintf(dst, size, "fc_gps=%.7f,%.7f gps_age_ms=%lld gps_fix=%u",
             (double)gps.latitude_e7 / 10000000.0,
             (double)gps.longitude_e7 / 10000000.0,
             (long long)gps.age_ms, (unsigned)gps.fix_type);
  }
  db_log_diag_mark(DB_LOG_STAGE_GPS_FORMATTED);
}

static bool db_log_capture_active(void) { return g_log_available && g_session_active; }

void db_sonar_log_log_hardwired_frame(int distance_mm, int frame_length,
                                      const char *frame_bytes) {
  if (!db_log_capture_active()) return;
  char gps[80];
  db_log_format_fc_gps(gps, sizeof(gps));
  (void)db_log_appendf(DB_LOG_STREAM_HARDWIRED,
                       "event=raw_frame raw_mm=%d len=%d bytes=%s %s",
                       distance_mm, frame_length,
                       frame_bytes == NULL ? "<none>" : frame_bytes, gps);
}

void db_sonar_log_log_hardwired_issue(const char *issue, const char *detail) {
  if (!db_log_capture_active()) return;
  (void)db_log_appendf(DB_LOG_STREAM_HARDWIRED,
                       "event=issue issue=%s detail=%s",
                       issue == NULL ? "unknown" : issue,
                       detail == NULL ? "n/a" : detail);
}

void db_sonar_log_log_hardwired_zero_run_start(void) {
  if (!db_log_capture_active()) return;
  char gps[80]; db_log_format_fc_gps(gps, sizeof(gps));
  (void)db_log_appendf(DB_LOG_STREAM_HARDWIRED,
                       "event=zero_run_start raw_mm=0 %s", gps);
}

void db_sonar_log_log_hardwired_zero_run_clear(int distance_mm) {
  if (!db_log_capture_active()) return;
  char gps[80]; db_log_format_fc_gps(gps, sizeof(gps));
  (void)db_log_appendf(DB_LOG_STREAM_HARDWIRED,
                       "event=zero_run_clear restored_mm=%d %s",
                       distance_mm, gps);
}

void db_sonar_log_log_deeper_sentence(const char *sentence) {
  if (!db_log_capture_active() || sentence == NULL) return;
  (void)db_log_appendf(DB_LOG_STREAM_DEEPER,
                       "event=raw_nmea sentence=%s", sentence);
}

void db_sonar_log_note_hardwired_poll(void) {
  if (g_hardwired_stats.active) g_hardwired_stats.polls++;
}
void db_sonar_log_note_hardwired_frame(int distance_mm) {
  if (!g_hardwired_stats.active) return;
  g_hardwired_stats.valid_frames++;
  if (distance_mm == 0) g_hardwired_stats.zero_frames++;
  else if (distance_mm > 0) g_hardwired_stats.nonzero_frames++;
}
void db_sonar_log_note_hardwired_timeout(void) {
  if (g_hardwired_stats.active) g_hardwired_stats.timeouts++;
}
void db_sonar_log_note_hardwired_bad_frame(void) {
  if (g_hardwired_stats.active) g_hardwired_stats.bad_frames++;
}
void db_sonar_log_note_fc_returned_distance_sensor(void) {
  if (g_hardwired_stats.active) g_hardwired_stats.fc_to_esp_distance_sensors++;
}

static void db_log_write_hardwired_stats(const char *event) {
  if (!g_hardwired_stats.active || !db_log_capture_active()) return;
  (void)db_log_appendf(
      DB_LOG_STREAM_HARDWIRED,
      "event=%s elapsed_ms=%lu polls=%lu valid_frames=%lu "
      "zero_frames=%lu nonzero_frames=%lu timeouts=%lu bad_frames=%lu "
      "esp_to_fc=%lu fc_to_esp=%lu",
      event,
      (unsigned long)(db_log_now_ms() - g_hardwired_stats.start_ms),
      (unsigned long)g_hardwired_stats.polls,
      (unsigned long)g_hardwired_stats.valid_frames,
      (unsigned long)g_hardwired_stats.zero_frames,
      (unsigned long)g_hardwired_stats.nonzero_frames,
      (unsigned long)g_hardwired_stats.timeouts,
      (unsigned long)g_hardwired_stats.bad_frames,
      (unsigned long)g_hardwired_stats.esp_to_fc_publishes,
      (unsigned long)g_hardwired_stats.fc_to_esp_distance_sensors);
}

void db_sonar_log_maybe_log_hardwired_session_sample(void) {
  if (!g_hardwired_stats.active || !db_log_capture_active()) return;
  TickType_t now = xTaskGetTickCount();
  if (g_hardwired_stats.last_sample_tick != 0 &&
      now - g_hardwired_stats.last_sample_tick < pdMS_TO_TICKS(1000)) return;
  db_log_diag_mark(DB_LOG_STAGE_SESSION_SAMPLE);
  db_log_write_hardwired_stats("session_sample");
  g_hardwired_stats.last_sample_tick = now;
}

void db_sonar_log_maybe_log_hardwired_publish(
    int published_distance_mm, int published_distance_cm,
    const danevi_sonar_snapshot_t *snapshot) {
  if (!db_log_capture_active() || snapshot == NULL) return;
  g_hardwired_stats.esp_to_fc_publishes++;
  TickType_t now = xTaskGetTickCount();
  int raw_mm = snapshot->has_raw_distance ? snapshot->raw_depth_mm : -1;
  bool should_log =
      g_last_hardwired_publish_tick == 0 ||
      db_log_abs_i32(published_distance_mm - g_last_hardwired_depth_mm) >= 10 ||
      (raw_mm == 0 && g_last_hardwired_raw_mm != 0) ||
      snapshot->zero_run_active != g_last_hardwired_zero_run ||
      snapshot->zero_filter_holding_last_good != g_last_hardwired_hold ||
      raw_mm != g_last_hardwired_raw_mm ||
      now - g_last_hardwired_publish_tick >=
          pdMS_TO_TICKS(DB_LOG_HARDWIRED_PERIOD_MS);
  if (!should_log) return;
  char gps[80]; db_log_format_fc_gps(gps, sizeof(gps));
  (void)db_log_appendf(
      DB_LOG_STREAM_HARDWIRED,
      "event=publish depth_mm=%d fc_cm=%d raw_mm=%d "
      "raw_age_ms=%lu last_good_mm=%d last_good_age_ms=%lu zero_run=%d "
      "zero_age_ms=%lu zero_count=%lu holding=%d %s",
      published_distance_mm,
      published_distance_cm, raw_mm, (unsigned long)snapshot->raw_sample_age_ms,
      snapshot->has_last_good_distance ? snapshot->last_good_depth_mm : -1,
      (unsigned long)snapshot->last_good_sample_age_ms,
      snapshot->zero_run_active ? 1 : 0,
      (unsigned long)snapshot->zero_run_age_ms,
      (unsigned long)snapshot->consecutive_zero_frames,
      snapshot->zero_filter_holding_last_good ? 1 : 0, gps);
  g_last_hardwired_publish_tick = now;
  g_last_hardwired_depth_mm = published_distance_mm;
  g_last_hardwired_raw_mm = raw_mm;
  g_last_hardwired_zero_run = snapshot->zero_run_active;
  g_last_hardwired_hold = snapshot->zero_filter_holding_last_good;
}

void db_sonar_log_maybe_log_deeper_track(const deeper_udp_snapshot_t *snapshot) {
  if (!db_log_capture_active() || snapshot == NULL) return;
  TickType_t now = xTaskGetTickCount();
  int depth_mm = snapshot->has_depth ? snapshot->depth_mm : -1;
  bool fix = snapshot->has_satellites && snapshot->gps_fix_valid;
  bool coordinates_changed =
      snapshot->has_coordinates != g_last_deeper_coordinates ||
      (snapshot->has_coordinates && g_last_deeper_coordinates &&
       (db_log_abs_double(snapshot->latitude_deg - g_last_deeper_latitude) >=
            0.00001 ||
        db_log_abs_double(snapshot->longitude_deg - g_last_deeper_longitude) >=
            0.00001));
  bool should_log =
      g_last_deeper_track_tick == 0 || coordinates_changed ||
      fix != g_last_deeper_fix ||
      db_log_abs_i32(depth_mm - g_last_deeper_depth_mm) >= 100 ||
      now - g_last_deeper_track_tick >= pdMS_TO_TICKS(DB_LOG_DEEPER_PERIOD_MS);
  if (!should_log) return;
  char gps[80]; db_log_format_fc_gps(gps, sizeof(gps));
  (void)db_log_appendf(
      DB_LOG_STREAM_DEEPER,
      "event=track depth_mm=%d fc_cm=%d temp_c=%.1f "
      "deeper_gps_fix=%d deeper_sats=%d deeper_lat=%.6f deeper_lon=%.6f "
      "sample_age_ms=%lu %s",
      depth_mm,
      depth_mm >= 0 ? depth_mm / 10 : -1,
      snapshot->has_temperature
          ? (double)snapshot->temperature_c_tenths / 10.0
          : -1000.0,
      fix ? 1 : 0, snapshot->has_satellites ? snapshot->satellites : -1,
      snapshot->has_coordinates ? snapshot->latitude_deg : 0.0,
      snapshot->has_coordinates ? snapshot->longitude_deg : 0.0,
      (unsigned long)snapshot->newest_sample_age_ms, gps);
  g_last_deeper_track_tick = now;
  g_last_deeper_depth_mm = depth_mm;
  g_last_deeper_fix = fix;
  g_last_deeper_coordinates = snapshot->has_coordinates;
  if (snapshot->has_coordinates) {
    g_last_deeper_latitude = snapshot->latitude_deg;
    g_last_deeper_longitude = snapshot->longitude_deg;
  }
}
