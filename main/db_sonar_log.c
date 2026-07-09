#include "db_sonar_log.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

#include "esp_log.h"
#include "esp_spiffs.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#define DB_SONAR_LOG_LINE_MAX 384
#define DB_SONAR_LOG_IO_BUFFER 1024
#define DB_SONAR_LOG_RESERVED_BYTES 8192
#define DB_SONAR_LOG_HARDWIRED_PERIOD_MS 1000
#define DB_SONAR_LOG_DEEPER_PERIOD_MS 1500

static const char *TAG = "DB_SONAR_LOG";

static SemaphoreHandle_t g_sonar_log_mutex = NULL;
static bool g_sonar_log_available = false;
static size_t g_sonar_log_partition_total_bytes = 0;
static size_t g_sonar_log_partition_used_bytes = 0;
static size_t g_sonar_log_max_file_bytes = 0;
static size_t g_sonar_log_trim_to_bytes = 0;
static uint32_t g_sonar_log_compaction_count = 0;

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

static size_t db_sonar_log_get_file_size_locked(void) {
  struct stat st = {0};
  if (stat(DB_SONAR_LOG_FILE_PATH, &st) != 0) {
    return 0;
  }
  return (size_t)st.st_size;
}

static esp_err_t db_sonar_log_refresh_usage_locked(void) {
  if (!g_sonar_log_available) {
    return ESP_ERR_INVALID_STATE;
  }

  size_t total = 0;
  size_t used = 0;
  esp_err_t err =
      esp_spiffs_info(DB_SONAR_LOG_PARTITION_LABEL, &total, &used);
  if (err != ESP_OK) {
    return err;
  }

  g_sonar_log_partition_total_bytes = total;
  g_sonar_log_partition_used_bytes = used;
  if (g_sonar_log_max_file_bytes == 0 || g_sonar_log_max_file_bytes > total) {
    size_t reserved =
        total > DB_SONAR_LOG_RESERVED_BYTES ? DB_SONAR_LOG_RESERVED_BYTES
                                            : total / 8;
    g_sonar_log_max_file_bytes = total > reserved ? total - reserved : total;
    g_sonar_log_trim_to_bytes = (g_sonar_log_max_file_bytes * 3) / 4;
  }
  return ESP_OK;
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

  char buffer[DB_SONAR_LOG_IO_BUFFER];
  size_t read_bytes = 0;
  while ((read_bytes = fread(buffer, 1, sizeof(buffer), source)) > 0) {
    if (fwrite(buffer, 1, read_bytes, tmp) != read_bytes) {
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
  db_sonar_log_init_mutex();
  if (g_sonar_log_mutex == NULL) {
    return ESP_ERR_NO_MEM;
  }

  xSemaphoreTake(g_sonar_log_mutex, portMAX_DELAY);
  if (g_sonar_log_available) {
    xSemaphoreGive(g_sonar_log_mutex);
    return ESP_OK;
  }

  esp_vfs_spiffs_conf_t conf = {.base_path = DB_SONAR_LOG_MOUNT_POINT,
                                .partition_label = DB_SONAR_LOG_PARTITION_LABEL,
                                .max_files = 3,
                                .format_if_mount_failed = true};
  esp_err_t err = esp_vfs_spiffs_register(&conf);
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

  char line[DB_SONAR_LOG_LINE_MAX];
  va_list args;
  va_start(args, fmt);
  vsnprintf(line, sizeof(line), fmt, args);
  va_end(args);

  size_t line_len = strlen(line);
  bool needs_newline = line_len == 0 || line[line_len - 1] != '\n';

  xSemaphoreTake(g_sonar_log_mutex, portMAX_DELAY);
  esp_err_t err =
      db_sonar_log_prepare_space_locked(line_len + (needs_newline ? 1 : 0));
  if (err != ESP_OK) {
    xSemaphoreGive(g_sonar_log_mutex);
    return err;
  }

  FILE *fp = fopen(DB_SONAR_LOG_FILE_PATH, "ab");
  if (fp == NULL) {
    xSemaphoreGive(g_sonar_log_mutex);
    return ESP_FAIL;
  }

  if (fwrite(line, 1, line_len, fp) != line_len) {
    fclose(fp);
    xSemaphoreGive(g_sonar_log_mutex);
    return ESP_FAIL;
  }
  if (needs_newline) {
    fputc('\n', fp);
  }

  fclose(fp);
  db_sonar_log_refresh_usage_locked();
  xSemaphoreGive(g_sonar_log_mutex);
  return ESP_OK;
}

void db_sonar_log_log_boot(db_sonar_source_t active_source, int boot_radio_mode,
                           bool deeper_connected, bool force_update_ap_mode,
                           bool web_fs_available) {
  if (!g_sonar_log_available) {
    return;
  }

  db_sonar_log_appendf(
      "boot_ms=%lu event=boot source=%s radio_mode=%d deeper_connected=%d "
      "force_update_ap=%d web_fs=%d",
      (unsigned long)db_sonar_log_now_ms(),
      db_sonar_log_source_name(active_source), boot_radio_mode,
      deeper_connected ? 1 : 0, force_update_ap_mode ? 1 : 0,
      web_fs_available ? 1 : 0);
}

void db_sonar_log_log_hardwired_frame(int distance_mm, int frame_length,
                                      const char *frame_bytes) {
  if (!g_sonar_log_available) {
    return;
  }

  db_sonar_log_appendf(
      "boot_ms=%lu event=hardwired_frame raw_mm=%d len=%d bytes=%s",
      (unsigned long)db_sonar_log_now_ms(), distance_mm, frame_length,
      frame_bytes == NULL ? "<none>" : frame_bytes);
}

void db_sonar_log_log_hardwired_issue(const char *issue, const char *detail) {
  if (!g_sonar_log_available) {
    return;
  }

  db_sonar_log_appendf("boot_ms=%lu event=hardwired_issue issue=%s detail=%s",
                       (unsigned long)db_sonar_log_now_ms(),
                       issue == NULL ? "unknown" : issue,
                       detail == NULL ? "n/a" : detail);
}

void db_sonar_log_log_hardwired_zero_run_start(void) {
  if (!g_sonar_log_available) {
    return;
  }

  db_sonar_log_appendf("boot_ms=%lu event=hardwired_zero_run_start raw_mm=0",
                       (unsigned long)db_sonar_log_now_ms());
}

void db_sonar_log_log_hardwired_zero_run_clear(int distance_mm) {
  if (!g_sonar_log_available) {
    return;
  }

  db_sonar_log_appendf(
      "boot_ms=%lu event=hardwired_zero_run_clear restored_mm=%d",
      (unsigned long)db_sonar_log_now_ms(), distance_mm);
}

void db_sonar_log_maybe_log_hardwired_publish(
    int published_distance_mm, int published_distance_cm,
    const danevi_sonar_snapshot_t *snapshot) {
  if (!g_sonar_log_available || snapshot == NULL) {
    return;
  }

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

  db_sonar_log_appendf(
      "boot_ms=%lu event=publish source=hardwired depth_mm=%d fc_cm=%d "
      "raw_mm=%d raw_age_ms=%lu last_good_mm=%d last_good_age_ms=%lu "
      "zero_run=%d zero_age_ms=%lu zero_count=%lu holding=%d",
      (unsigned long)db_sonar_log_now_ms(), published_distance_mm,
      published_distance_cm, raw_mm,
      (unsigned long)snapshot->raw_sample_age_ms,
      snapshot->has_last_good_distance ? snapshot->last_good_depth_mm : -1,
      (unsigned long)snapshot->last_good_sample_age_ms,
      snapshot->zero_run_active ? 1 : 0,
      (unsigned long)snapshot->zero_run_age_ms,
      (unsigned long)snapshot->consecutive_zero_frames,
      snapshot->zero_filter_holding_last_good ? 1 : 0);

  g_last_hardwired_publish_log_tick = now_tick;
  g_last_hardwired_logged_depth_mm = published_distance_mm;
  g_last_hardwired_logged_raw_mm = raw_mm;
  g_last_hardwired_logged_zero_run = snapshot->zero_run_active;
  g_last_hardwired_logged_hold = snapshot->zero_filter_holding_last_good;
}

void db_sonar_log_maybe_log_deeper_track(const deeper_udp_snapshot_t *snapshot) {
  if (!g_sonar_log_available || snapshot == NULL) {
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

  db_sonar_log_appendf(
      "boot_ms=%lu event=track source=deeper depth_mm=%d fc_cm=%d "
      "temp_c=%.1f gps_fix=%d sats=%d lat=%.6f lon=%.6f sample_age_ms=%lu",
      (unsigned long)db_sonar_log_now_ms(), depth_mm,
      depth_mm >= 0 ? depth_mm / 10 : -1,
      snapshot->has_temperature
          ? ((double)snapshot->temperature_c_tenths) / 10.0
          : -1000.0,
      fix_valid ? 1 : 0,
      snapshot->has_satellites ? snapshot->satellites : -1,
      snapshot->has_coordinates ? snapshot->latitude_deg : 0.0,
      snapshot->has_coordinates ? snapshot->longitude_deg : 0.0,
      (unsigned long)snapshot->newest_sample_age_ms);

  g_last_deeper_track_log_tick = now_tick;
  g_last_deeper_logged_depth_mm = depth_mm;
  g_last_deeper_logged_fix = fix_valid;
  g_last_deeper_logged_coordinates = snapshot->has_coordinates;
  if (snapshot->has_coordinates) {
    g_last_deeper_logged_latitude = snapshot->latitude_deg;
    g_last_deeper_logged_longitude = snapshot->longitude_deg;
  }
}
