#include "db_diag.h"

#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "esp_app_desc.h"
#include "esp_core_dump.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define DB_DIAG_MAGIC 0x44424244U /* "DBBD" */
#define DB_DIAG_FORMAT_VERSION 1U
#define DB_DIAG_SECTOR_SIZE 4096U
#define DB_DIAG_ERASED_WORD 0xFFFFFFFFU

_Static_assert(sizeof(db_diag_record_t) == DB_DIAG_RECORD_SIZE,
               "diagnostic record must remain exactly 256 bytes");

typedef struct {
    uint64_t sequence;
    size_t sector_index;
} db_diag_record_ref_t;

static const char *TAG = "DBB_DIAG";
static const esp_partition_t *s_partition;
static SemaphoreHandle_t s_mutex;
static bool s_init_attempted;
static esp_err_t s_init_result = ESP_ERR_INVALID_STATE;
static size_t s_sector_count;
static size_t s_valid_records;
static size_t s_programmed_invalid_records;
static uint64_t s_latest_sequence;
static size_t s_latest_sector;

static uint32_t db_diag_crc32c(const void *data, size_t length) {
    const uint8_t *bytes = (const uint8_t *)data;
    uint32_t crc = UINT32_MAX;
    for (size_t i = 0; i < length; ++i) {
        crc ^= bytes[i];
        for (unsigned bit = 0; bit < 8; ++bit) {
            crc = (crc >> 1) ^ (0x82F63B78U & (0U - (crc & 1U)));
        }
    }
    return ~crc;
}

static bool db_diag_sequence_is_newer(uint64_t candidate, uint64_t current) {
    return (int64_t)(candidate - current) > 0;
}

static bool db_diag_record_is_valid(const db_diag_record_t *record) {
    if (record->magic != DB_DIAG_MAGIC ||
        record->format_version != DB_DIAG_FORMAT_VERSION ||
        record->record_size != sizeof(*record) || record->sequence == 0) {
        return false;
    }
    return record->crc32c ==
           db_diag_crc32c(record, offsetof(db_diag_record_t, crc32c));
}

static esp_err_t db_diag_read_sector(size_t sector_index,
                                     db_diag_record_t *record) {
    if (s_partition == NULL || record == NULL ||
        sector_index >= s_sector_count) {
        return ESP_ERR_INVALID_ARG;
    }
    return esp_partition_read(s_partition, sector_index * DB_DIAG_SECTOR_SIZE,
                              record, sizeof(*record));
}

esp_err_t db_diag_init(void) {
    if (s_init_attempted) {
        return s_init_result;
    }
    s_init_attempted = true;

    s_mutex = xSemaphoreCreateMutex();
    if (s_mutex == NULL) {
        s_init_result = ESP_ERR_NO_MEM;
        return s_init_result;
    }

    s_partition = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY,
        DB_DIAG_PARTITION_LABEL);
    if (s_partition == NULL) {
        ESP_LOGW(TAG, "Partition '%s' is not present", DB_DIAG_PARTITION_LABEL);
        s_init_result = ESP_ERR_NOT_FOUND;
        return s_init_result;
    }
    if (s_partition->size < (2U * DB_DIAG_SECTOR_SIZE) ||
        (s_partition->size % DB_DIAG_SECTOR_SIZE) != 0) {
        ESP_LOGE(TAG, "Partition '%s' has invalid size %u",
                 DB_DIAG_PARTITION_LABEL, (unsigned)s_partition->size);
        s_init_result = ESP_ERR_INVALID_SIZE;
        return s_init_result;
    }

    s_sector_count = s_partition->size / DB_DIAG_SECTOR_SIZE;
    db_diag_record_t record;
    for (size_t sector = 0; sector < s_sector_count; ++sector) {
        esp_err_t err = db_diag_read_sector(sector, &record);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed reading diagnostic sector %u (%s)",
                     (unsigned)sector, esp_err_to_name(err));
            s_init_result = err;
            return s_init_result;
        }
        if (db_diag_record_is_valid(&record)) {
            ++s_valid_records;
            if (s_latest_sequence == 0 ||
                db_diag_sequence_is_newer(record.sequence,
                                          s_latest_sequence)) {
                s_latest_sequence = record.sequence;
                s_latest_sector = sector;
            }
        } else if (record.magic != DB_DIAG_ERASED_WORD) {
            ++s_programmed_invalid_records;
        }
    }

    s_init_result = ESP_OK;
    ESP_LOGI(TAG,
             "Journal ready: %u sectors, %u valid records, latest sequence %llu",
             (unsigned)s_sector_count, (unsigned)s_valid_records,
             (unsigned long long)s_latest_sequence);
    return ESP_OK;
}

static uint8_t db_diag_current_flags(void) {
    size_t address = 0;
    size_t size = 0;
    return esp_core_dump_image_get(&address, &size) == ESP_OK && size > 0
               ? DB_DIAG_FLAG_COREDUMP_PRESENT
               : 0;
}

esp_err_t db_diag_log_event(db_diag_event_t event_type,
                            db_diag_severity_t severity,
                            uint32_t reset_reason, const char *message) {
    esp_err_t err = db_diag_init();
    if (err != ESP_OK) {
        return err;
    }

    if (xSemaphoreTake(s_mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    db_diag_record_t record = {0};
    const esp_app_desc_t *app = esp_app_get_description();
    const esp_partition_t *running = esp_ota_get_running_partition();
    time_t now = time(NULL);

    record.magic = DB_DIAG_MAGIC;
    record.format_version = DB_DIAG_FORMAT_VERSION;
    record.record_size = sizeof(record);
    record.sequence = s_latest_sequence + 1U;
    if (record.sequence == 0) {
        record.sequence = 1;
    }
    record.uptime_ms = (uint64_t)(esp_timer_get_time() / 1000ULL);
    record.unix_time = now >= 1577836800 ? (int64_t)now : 0;
    record.reset_reason = reset_reason;
    record.event_type = (uint16_t)event_type;
    record.severity = (uint8_t)severity;
    record.flags = db_diag_current_flags();
    record.ota_subtype = running == NULL ? UINT32_MAX : running->subtype;
    if (app != NULL) {
        record.secure_version = app->secure_version;
        memcpy(record.app_elf_sha256, app->app_elf_sha256,
               sizeof(record.app_elf_sha256));
        strlcpy(record.app_version, app->version, sizeof(record.app_version));
        strlcpy(record.build_date, app->date, sizeof(record.build_date));
        strlcpy(record.build_time, app->time, sizeof(record.build_time));
    }
    if (message != NULL) {
        strlcpy(record.message, message, sizeof(record.message));
    }
    record.crc32c =
        db_diag_crc32c(&record, offsetof(db_diag_record_t, crc32c));

    size_t target_sector =
        s_latest_sequence == 0 ? 0 : (s_latest_sector + 1U) % s_sector_count;
    db_diag_record_t prior_record;
    bool replacing_valid =
        db_diag_read_sector(target_sector, &prior_record) == ESP_OK &&
        db_diag_record_is_valid(&prior_record);

    err = esp_partition_erase_range(s_partition,
                                    target_sector * DB_DIAG_SECTOR_SIZE,
                                    DB_DIAG_SECTOR_SIZE);
    if (err == ESP_OK) {
        err = esp_partition_write(s_partition,
                                  target_sector * DB_DIAG_SECTOR_SIZE,
                                  &record, sizeof(record));
    }
    if (err == ESP_OK) {
        db_diag_record_t verified;
        err = db_diag_read_sector(target_sector, &verified);
        if (err == ESP_OK &&
            (!db_diag_record_is_valid(&verified) ||
             verified.sequence != record.sequence)) {
            err = ESP_ERR_INVALID_CRC;
        }
    }

    if (err == ESP_OK) {
        s_latest_sequence = record.sequence;
        s_latest_sector = target_sector;
        if (!replacing_valid && s_valid_records < s_sector_count) {
            ++s_valid_records;
        }
        ESP_LOGI(TAG, "Stored %s record, sequence %llu, sector %u",
                 db_diag_event_name(record.event_type),
                 (unsigned long long)record.sequence,
                 (unsigned)target_sector);
    } else {
        ESP_LOGE(TAG, "Failed writing diagnostic record (%s)",
                 esp_err_to_name(err));
    }

    xSemaphoreGive(s_mutex);
    return err;
}

esp_err_t db_diag_log_boot(void) {
    return db_diag_log_event(DB_DIAG_EVENT_BOOT, DB_DIAG_SEVERITY_INFO,
                             (uint32_t)esp_reset_reason(), "firmware boot");
}

esp_err_t db_diag_log_ota_confirmed(void) {
    return db_diag_log_event(DB_DIAG_EVENT_OTA_CONFIRMED,
                             DB_DIAG_SEVERITY_INFO,
                             (uint32_t)esp_reset_reason(),
                             "pending OTA image confirmed");
}

void db_diag_get_status(db_diag_status_t *status) {
    if (status == NULL) {
        return;
    }
    memset(status, 0, sizeof(*status));
    if (db_diag_init() != ESP_OK) {
        return;
    }
    if (xSemaphoreTake(s_mutex, portMAX_DELAY) == pdTRUE) {
        status->available = true;
        status->partition_size = s_partition->size;
        status->sector_count = s_sector_count;
        status->valid_records = s_valid_records;
        status->programmed_invalid_records = s_programmed_invalid_records;
        status->latest_sequence = s_latest_sequence;
        status->latest_sector = s_latest_sector;
        xSemaphoreGive(s_mutex);
    }
}

static int db_diag_ref_compare(const void *left, const void *right) {
    const db_diag_record_ref_t *a = (const db_diag_record_ref_t *)left;
    const db_diag_record_ref_t *b = (const db_diag_record_ref_t *)right;
    if (a->sequence < b->sequence) return -1;
    if (a->sequence > b->sequence) return 1;
    return 0;
}

esp_err_t db_diag_foreach_chronological(db_diag_record_callback_t callback,
                                        void *context) {
    if (callback == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = db_diag_init();
    if (err != ESP_OK) {
        return err;
    }
    if (xSemaphoreTake(s_mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    db_diag_record_ref_t *refs =
        calloc(s_sector_count, sizeof(db_diag_record_ref_t));
    if (refs == NULL) {
        xSemaphoreGive(s_mutex);
        return ESP_ERR_NO_MEM;
    }

    size_t count = 0;
    db_diag_record_t record;
    for (size_t sector = 0; sector < s_sector_count; ++sector) {
        err = db_diag_read_sector(sector, &record);
        if (err != ESP_OK) {
            break;
        }
        if (db_diag_record_is_valid(&record)) {
            refs[count].sequence = record.sequence;
            refs[count].sector_index = sector;
            ++count;
        }
    }
    if (err == ESP_OK) {
        qsort(refs, count, sizeof(*refs), db_diag_ref_compare);
        for (size_t i = 0; i < count; ++i) {
            err = db_diag_read_sector(refs[i].sector_index, &record);
            if (err != ESP_OK) {
                break;
            }
            err = callback(&record, refs[i].sector_index, context);
            if (err != ESP_OK) {
                break;
            }
        }
    }

    free(refs);
    xSemaphoreGive(s_mutex);
    return err;
}

const char *db_diag_event_name(uint16_t event_type) {
    switch (event_type) {
        case DB_DIAG_EVENT_BOOT: return "boot";
        case DB_DIAG_EVENT_OTA_CONFIRMED: return "ota_confirmed";
        case DB_DIAG_EVENT_SYSTEM: return "system";
        case DB_DIAG_EVENT_BENCH_CRASH_ARMED: return "bench_crash_armed";
        default: return "unknown";
    }
}

const char *db_diag_severity_name(uint8_t severity) {
    switch (severity) {
        case DB_DIAG_SEVERITY_INFO: return "info";
        case DB_DIAG_SEVERITY_WARNING: return "warning";
        case DB_DIAG_SEVERITY_ERROR: return "error";
        default: return "unknown";
    }
}

const char *db_diag_reset_reason_name(uint32_t reset_reason) {
    switch ((esp_reset_reason_t)reset_reason) {
        case ESP_RST_UNKNOWN: return "unknown";
        case ESP_RST_POWERON: return "power_on";
        case ESP_RST_EXT: return "external_pin";
        case ESP_RST_SW: return "software";
        case ESP_RST_PANIC: return "panic";
        case ESP_RST_INT_WDT: return "interrupt_watchdog";
        case ESP_RST_TASK_WDT: return "task_watchdog";
        case ESP_RST_WDT: return "other_watchdog";
        case ESP_RST_DEEPSLEEP: return "deep_sleep";
        case ESP_RST_BROWNOUT: return "brownout";
        case ESP_RST_SDIO: return "sdio";
        case ESP_RST_USB: return "usb";
        case ESP_RST_JTAG: return "jtag";
        case ESP_RST_EFUSE: return "efuse";
        case ESP_RST_PWR_GLITCH: return "power_glitch";
        case ESP_RST_CPU_LOCKUP: return "cpu_lockup";
        default: return "unrecognized";
    }
}
