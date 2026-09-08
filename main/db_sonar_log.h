/*
 * db_sonar_log — separated persistent logging on the FAT `logs` partition.
 *
 * Contract (19/20-08-2026; see also HANDOVER_esp-logging-20260819.md):
 * - Four logical streams: bounded /logs/SYSTEM.LOG (256 KiB, compacts to its
 *   newest 192 KiB) plus per-session trip (T#######), hardwired sonar
 *   (H#######) and Deeper sonar (D#######) files. Active session files carry
 *   .TMP and are renamed .LOG together on clean close; interrupted .TMP files
 *   are recovered to .LOG at the next boot. 8.3 names only (FATFS_LFN_NONE).
 * - One session = one shared seven-digit id across the three streams. Armed
 *   capture starts/ends with the FC arm state; a manual timed capture
 *   (30 s..12 h) may start while disarmed or extend an armed session.
 * - Retention: before each write, 256 KiB filesystem headroom plus the
 *   unfilled part of the 1536 KiB FC-image reserve (db_fc_flash) are kept
 *   free by evicting the OLDEST COMPLETED session as a unit. The active
 *   session is never trimmed; if space runs out with only the active session
 *   left, capture closes instead.
 * - Locking: one mutex serializes every append, status, list, delete and
 *   stream read. db_sonar_log_stream() holds it for the WHOLE transfer, so
 *   a slow HTTP client stalls all logging and the sonar publish path for the
 *   duration - do not download during a capture (v2: copy-out or bounded
 *   take, planned after the 3 km run). The note_* counters are lock-free by
 *   design (single-writer sonar task; torn counts are acceptable).
 */
#ifndef DB_ESP32_SONAR_LOG_H
#define DB_ESP32_SONAR_LOG_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "danevi_sonar.h"
#include "deeper_udp_sonar.h"
#include "esp_err.h"
#include "globals.h"

#define DB_SONAR_LOG_PARTITION_LABEL "logs"
#define DB_SONAR_LOG_MOUNT_POINT "/logs"
#define DB_SONAR_LOG_MAX_SESSIONS 24U
/* Session files are named session-%07lu, so ids above this cannot name a valid
 * file. Requests beyond it are malformed (400), not server faults (500). */
#define DB_SONAR_LOG_MAX_SESSION_ID 9999999UL

typedef enum {
  DB_LOG_STREAM_SYSTEM = 0,
  DB_LOG_STREAM_TRIP = 1,
  DB_LOG_STREAM_HARDWIRED = 2,
  DB_LOG_STREAM_DEEPER = 3,
} db_log_stream_t;

typedef struct {
  bool mounted;
  size_t partition_total_bytes;
  size_t partition_used_bytes;
  size_t partition_free_bytes;
  size_t fc_image_reserve_bytes;
  size_t filesystem_headroom_bytes;
  size_t system_log_bytes;
  size_t system_log_limit_bytes;
  size_t session_pool_limit_bytes;
  uint32_t completed_session_count;
  uint32_t evicted_session_count;
  bool session_active;
  uint32_t active_session_id;
  uint32_t active_session_age_ms;
  uint32_t manual_remaining_ms;
  bool armed_capture_active;
  /* Last append-path failure, for field diagnosis: 0/0 = healthy. Stage:
   * 1=prepare_space 2=fopen 3=fwrite 4=init_create. */
  int last_write_errno;
  uint32_t last_write_fail_stage;
} db_sonar_log_status_t;

typedef struct {
  uint32_t id;
  bool active;
  size_t trip_bytes;
  size_t hardwired_bytes;
  size_t deeper_bytes;
} db_sonar_log_session_info_t;

/* Reset-safe breadcrumbs live only in RTC no-init memory. They are deliberately
 * separate from the raw `diag` recovery partition. */
typedef struct {
  uint32_t reset_reason;
  uint32_t previous_stage;
  uint32_t previous_sequence;
  uint32_t previous_min_stack;
  uint32_t current_stage;
  uint32_t current_sequence;
  uint32_t current_min_stack;
} db_sonar_log_crash_diag_t;

typedef esp_err_t (*db_sonar_log_chunk_writer_t)(const char *data,
                                                 size_t data_length,
                                                 void *user_ctx);

esp_err_t db_sonar_log_init(void);
bool db_sonar_log_is_available(void);
esp_err_t db_sonar_log_get_status(db_sonar_log_status_t *status);
esp_err_t db_sonar_log_list_sessions(db_sonar_log_session_info_t *sessions,
                                     size_t capacity, size_t *out_count);
esp_err_t db_sonar_log_stream(db_log_stream_t stream, uint32_t session_id,
                              db_sonar_log_chunk_writer_t chunk_writer,
                              void *user_ctx, size_t *out_bytes_streamed,
                              int *out_file_errno);
esp_err_t db_sonar_log_delete_session(uint32_t session_id);
esp_err_t db_sonar_log_clear_all(void);
/* Reformats the volume. Reclaims orphaned clusters that clear_all cannot
 * see; destroys everything on it. Refused while a capture is active. */
esp_err_t db_sonar_log_format_volume(void);

esp_err_t db_sonar_log_start_manual_capture(uint32_t duration_seconds,
                                            uint32_t *out_session_id);
esp_err_t db_sonar_log_stop_manual_capture(void);
void db_sonar_log_tick(void);

void db_sonar_log_get_crash_diag(db_sonar_log_crash_diag_t *diag);
const char *db_sonar_log_diag_stage_name(uint32_t stage);
void db_sonar_log_refresh_limits(void);

void db_sonar_log_log_system_event(const char *event, const char *detail);
void db_sonar_log_log_system_kv(const char *fmt, ...)
    __attribute__((format(printf, 1, 2)));
void db_sonar_log_log_boot(db_sonar_source_t active_source, int boot_radio_mode,
                           bool deeper_connected, bool force_update_ap_mode,
                           bool web_fs_available);
void db_sonar_log_log_hardwired_frame(int distance_mm, int frame_length,
                                      const char *frame_bytes);
void db_sonar_log_log_hardwired_issue(const char *issue, const char *detail);
void db_sonar_log_log_hardwired_zero_run_start(void);
void db_sonar_log_log_hardwired_zero_run_clear(int distance_mm);
void db_sonar_log_log_deeper_sentence(const char *sentence);

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
