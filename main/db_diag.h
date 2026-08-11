#ifndef DBB_DIAG_H
#define DBB_DIAG_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#define DB_DIAG_PARTITION_LABEL "diag"
#define DB_DIAG_RECORD_SIZE 256U

#define DB_DIAG_FLAG_COREDUMP_PRESENT (1U << 0)

typedef enum {
    DB_DIAG_EVENT_BOOT = 1,
    DB_DIAG_EVENT_OTA_CONFIRMED = 2,
    DB_DIAG_EVENT_SYSTEM = 3,
    DB_DIAG_EVENT_BENCH_CRASH_ARMED = 100,
} db_diag_event_t;

typedef enum {
    DB_DIAG_SEVERITY_INFO = 1,
    DB_DIAG_SEVERITY_WARNING = 2,
    DB_DIAG_SEVERITY_ERROR = 3,
} db_diag_severity_t;

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t format_version;
    uint16_t record_size;
    uint64_t sequence;
    uint64_t uptime_ms;
    int64_t unix_time;
    uint32_t reset_reason;
    uint16_t event_type;
    uint8_t severity;
    uint8_t flags;
    uint32_t ota_subtype;
    uint32_t secure_version;
    uint8_t app_elf_sha256[32];
    char app_version[32];
    char build_date[16];
    char build_time[16];
    char message[108];
    uint32_t crc32c;
} db_diag_record_t;

typedef struct {
    bool available;
    size_t partition_size;
    size_t sector_count;
    size_t valid_records;
    size_t programmed_invalid_records;
    uint64_t latest_sequence;
    size_t latest_sector;
} db_diag_status_t;

typedef esp_err_t (*db_diag_record_callback_t)(const db_diag_record_t *record,
                                               size_t sector_index,
                                               void *context);

esp_err_t db_diag_init(void);
esp_err_t db_diag_log_boot(void);
esp_err_t db_diag_log_ota_confirmed(void);
esp_err_t db_diag_log_event(db_diag_event_t event_type,
                            db_diag_severity_t severity,
                            uint32_t reset_reason, const char *message);
void db_diag_get_status(db_diag_status_t *status);
esp_err_t db_diag_foreach_chronological(db_diag_record_callback_t callback,
                                        void *context);

const char *db_diag_event_name(uint16_t event_type);
const char *db_diag_severity_name(uint8_t severity);
const char *db_diag_reset_reason_name(uint32_t reset_reason);

#endif
