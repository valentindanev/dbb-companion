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

/*
 * Reset-safe breadcrumbs for armed logger failures. The current stage lives in
 * RTC no-init memory, so the next boot can report where the previous boot
 * stopped without writing another byte to SPIFFS at the failure point.
 */
typedef struct {
  uint32_t reset_reason;
  uint32_t previous_stage;
  uint32_t previous_sequence;
  uint32_t previous_min_stack;
  uint32_t current_stage;
  uint32_t current_sequence;
  uint32_t current_min_stack;
} db_sonar_log_crash_diag_t;

/*
 * Consumer used by db_sonar_log_stream(). The logger deliberately owns file
 * access and synchronization; callers only receive immutable chunks. This
 * keeps HTTP transport details out of the storage module while preventing an
 * append or rolling compaction from changing sonar.log during a download.
 */
typedef esp_err_t (*db_sonar_log_chunk_writer_t)(const char *data,
                                                 size_t data_length,
                                                 void *user_ctx);

esp_err_t db_sonar_log_init(void);
bool db_sonar_log_is_available(void);
esp_err_t db_sonar_log_get_status(db_sonar_log_status_t *status);
void db_sonar_log_get_crash_diag(db_sonar_log_crash_diag_t *diag);
const char *db_sonar_log_diag_stage_name(uint32_t stage);
esp_err_t db_sonar_log_stream(db_sonar_log_chunk_writer_t chunk_writer,
                              void *user_ctx, size_t *out_bytes_streamed,
                              int *out_file_errno);
esp_err_t db_sonar_log_clear(void);

/*
 * Re-evaluate how much of the `logs` partition to hold back for a
 * flight-controller firmware image. Call after db_fc_flash stores or deletes
 * one: the reserve is only held while an image is actually present, so the log
 * gets the whole partition the rest of the time.
 */
void db_sonar_log_refresh_limits(void);
esp_err_t db_sonar_log_appendf(const char *fmt, ...);

void db_sonar_log_log_boot(db_sonar_source_t active_source, int boot_radio_mode,
                           bool deeper_connected, bool force_update_ap_mode,
                           bool web_fs_available);
void db_sonar_log_log_hardwired_frame(int distance_mm, int frame_length,
                                      const char *frame_bytes);
void db_sonar_log_log_hardwired_issue(const char *issue, const char *detail);
void db_sonar_log_log_hardwired_zero_run_start(void);
void db_sonar_log_log_hardwired_zero_run_clear(int distance_mm);
/* Armed-session diagnostics. Counters are RAM-only and end in one log summary. */
void db_sonar_log_note_fc_armed_state(bool armed);
void db_sonar_log_note_hardwired_poll(void);
void db_sonar_log_note_hardwired_frame(int distance_mm);
void db_sonar_log_note_hardwired_timeout(void);
void db_sonar_log_note_hardwired_bad_frame(void);
void db_sonar_log_note_fc_returned_distance_sensor(void);
void db_sonar_log_maybe_log_hardwired_session_sample(void);
void db_sonar_log_maybe_log_hardwired_publish(
    int published_distance_mm, int published_distance_cm,
    const danevi_sonar_snapshot_t *snapshot);
void db_sonar_log_maybe_log_deeper_track(const deeper_udp_snapshot_t *snapshot);

#endif // DB_ESP32_SONAR_LOG_H
