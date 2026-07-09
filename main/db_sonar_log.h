#ifndef DB_ESP32_SONAR_LOG_H
#define DB_ESP32_SONAR_LOG_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "globals.h"
#include "danevi_sonar.h"
#include "deeper_udp_sonar.h"

#define DB_SONAR_LOG_PARTITION_LABEL "logs"
#define DB_SONAR_LOG_MOUNT_POINT "/logs"
#define DB_SONAR_LOG_FILE_PATH DB_SONAR_LOG_MOUNT_POINT "/sonar.log"

typedef struct {
  bool mounted;
  size_t partition_total_bytes;
  size_t partition_used_bytes;
  size_t log_file_bytes;
  size_t max_log_file_bytes;
  size_t trim_to_bytes;
  uint32_t compaction_count;
} db_sonar_log_status_t;

esp_err_t db_sonar_log_init(void);
bool db_sonar_log_is_available(void);
esp_err_t db_sonar_log_get_status(db_sonar_log_status_t *status);
esp_err_t db_sonar_log_clear(void);
esp_err_t db_sonar_log_appendf(const char *fmt, ...);

void db_sonar_log_log_boot(db_sonar_source_t active_source, int boot_radio_mode,
                           bool deeper_connected, bool force_update_ap_mode,
                           bool web_fs_available);
void db_sonar_log_log_hardwired_frame(int distance_mm, int frame_length,
                                      const char *frame_bytes);
void db_sonar_log_log_hardwired_issue(const char *issue, const char *detail);
void db_sonar_log_log_hardwired_zero_run_start(void);
void db_sonar_log_log_hardwired_zero_run_clear(int distance_mm);
void db_sonar_log_maybe_log_hardwired_publish(
    int published_distance_mm, int published_distance_cm,
    const danevi_sonar_snapshot_t *snapshot);
void db_sonar_log_maybe_log_deeper_track(const deeper_udp_snapshot_t *snapshot);

#endif // DB_ESP32_SONAR_LOG_H
