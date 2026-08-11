#include "db_location_store.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_partition.h"
#include "esp_random.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#if CONFIG_DB_LOCATION_STORE

#define DB_LOCATION_MAGIC 0x31424244U /* "DBB1" */
#define DB_LOCATION_HEADER_SIZE 256U
#define DB_LOCATION_SLOT_SIZE 0x20000U
/*
 * One snapshot slot per partition. db_a/db_b must each be exactly
 * DB_LOCATION_SLOT_SIZE * DB_LOCATION_SLOT_COUNT = 128 KiB, and init refuses -
 * silently, from the user's point of view - if they are not. Keep this constant
 * and partitions_s3_16mb.csv in step, in the same commit.
 *
 * Was 4 slots x 512 KiB per side. The database is bounded at
 * DB_LOCATION_MAX_RECORDS * DB_LOCATION_RECORD_SIZE = 72 KiB, so a single slot
 * holds every record that can ever exist; the surplus went to the log
 * partition. A/B alternation is unaffected - consecutive generations still land
 * in different partitions.
 */
#define DB_LOCATION_SLOT_COUNT 1U
#define DB_LOCATION_PAYLOAD_OFFSET 0x1000U
#define DB_LOCATION_ERASE_SECTOR 0x1000U
#define DB_LOCATION_ERASED_WORD 0xFFFFFFFFU

typedef struct __attribute__((packed, aligned(16))) {
    uint32_t magic;
    uint16_t format_version;
    uint16_t protocol_version;
    uint16_t header_size;
    uint16_t record_size;
    uint32_t flags;
    uint64_t database_uuid;
    uint32_t generation;
    uint32_t snapshot_sequence;
    uint32_t content_crc32c;
    uint16_t record_count;
    uint16_t reserved_u16;
    uint32_t payload_length;
    uint32_t next_internal_counter;
    uint32_t created_unix;
    uint32_t committed_unix;
    uint16_t slot_index;
    uint8_t partition_index;
    uint8_t reserved0;
    uint8_t reserved[192];
    uint32_t header_crc32c;
} db_location_snapshot_header_t;

typedef struct {
    bool valid;
    db_location_snapshot_header_t header;
} db_location_slot_meta_t;

_Static_assert(sizeof(db_location_snapshot_header_t) == DB_LOCATION_HEADER_SIZE,
               "location snapshot header must remain 256 bytes");
_Static_assert(sizeof(db_location_record_t) == DB_LOCATION_RECORD_SIZE,
               "location record must remain 96 bytes");
/* Originally required because flash-encrypted writes are XTS-AES block sized.
   Flash encryption was scrapped, so the alignment is no longer mandatory - but
   the record size is part of the on-flash format and must not drift. */
_Static_assert((DB_LOCATION_RECORD_SIZE % 16U) == 0,
               "location record size must stay 16-byte aligned");
_Static_assert(DB_LOCATION_MAX_RECORDS * DB_LOCATION_RECORD_SIZE <=
                   DB_LOCATION_SLOT_SIZE - DB_LOCATION_PAYLOAD_OFFSET,
               "record capacity must fit one snapshot slot");

static const char *TAG = "DBB_LOCATION_DB";
static const esp_partition_t *s_partitions[2];
static db_location_slot_meta_t s_slots[2][DB_LOCATION_SLOT_COUNT];
static db_location_record_t *s_records;
static SemaphoreHandle_t s_mutex;
static bool s_init_attempted;
static esp_err_t s_init_result = ESP_ERR_INVALID_STATE;
static bool s_recovery_fault;
static bool s_initialized_empty;
static db_location_snapshot_header_t s_active_header;
static uint8_t s_active_partition;
static uint8_t s_active_slot;
static uint8_t s_valid_slots[2];
static uint32_t s_newest_generation[2];
static uint8_t s_newest_slot[2];

static uint32_t db_location_crc32c(const void *data, size_t length) {
    const uint8_t *bytes = data;
    uint32_t crc = UINT32_MAX;
    for (size_t i = 0; i < length; ++i) {
        crc ^= bytes[i];
        for (unsigned bit = 0; bit < 8; ++bit) {
            crc = (crc >> 1) ^ (0x82F63B78U & (0U - (crc & 1U)));
        }
    }
    return ~crc;
}

static uint16_t db_location_crc16_ccitt(const void *data, size_t length) {
    const uint8_t *bytes = data;
    uint16_t crc = 0xFFFFU;
    for (size_t i = 0; i < length; ++i) {
        crc ^= (uint16_t)bytes[i] << 8;
        for (unsigned bit = 0; bit < 8; ++bit) {
            crc = (uint16_t)((crc << 1) ^
                             ((crc & 0x8000U) != 0 ? 0x1021U : 0U));
        }
    }
    return crc;
}

static bool db_location_generation_is_newer(uint32_t candidate,
                                             uint32_t current) {
    return (int32_t)(candidate - current) > 0;
}

static uint32_t db_location_now(void) {
    time_t now = time(NULL);
    return now >= 1577836800 ? (uint32_t)now : 0;
}

static bool db_location_record_valid(const db_location_record_t *record) {
    if (record == NULL || record->entity_id == 0 ||
        record->entity_type < DB_LOCATION_ENTITY_LAKE ||
        record->entity_type > DB_LOCATION_ENTITY_USAGE ||
        record->name_length > DB_LOCATION_NAME_BYTES) {
        return false;
    }
    if (record->entity_type == DB_LOCATION_ENTITY_POINT &&
        record->name_length > 24U) {
        return false;
    }
    return record->record_crc16 == db_location_crc16_ccitt(
                                       record,
                                       offsetof(db_location_record_t,
                                                record_crc16));
}

static void db_location_record_finalize(db_location_record_t *record) {
    record->record_crc16 = 0;
    memset(record->reserved1, 0, sizeof(record->reserved1));
    record->record_crc16 = db_location_crc16_ccitt(
        record, offsetof(db_location_record_t, record_crc16));
}

static bool db_location_header_shape_valid(
    const db_location_snapshot_header_t *header, uint8_t partition_index,
    uint8_t slot_index) {
    if (header->magic != DB_LOCATION_MAGIC ||
        header->format_version != DB_LOCATION_FORMAT_VERSION ||
        header->protocol_version != DB_LOCATION_PROTOCOL_VERSION ||
        header->header_size != sizeof(*header) ||
        header->record_size != sizeof(db_location_record_t) ||
        header->database_uuid == 0 ||
        header->record_count > DB_LOCATION_MAX_RECORDS ||
        header->payload_length !=
            (uint32_t)header->record_count * sizeof(db_location_record_t) ||
        header->partition_index != partition_index ||
        header->slot_index != slot_index) {
        return false;
    }
    return header->header_crc32c == db_location_crc32c(
                                         header,
                                         offsetof(db_location_snapshot_header_t,
                                                  header_crc32c));
}

static esp_err_t db_location_load_slot(uint8_t partition_index,
                                       uint8_t slot_index,
                                       db_location_snapshot_header_t *header,
                                       bool load_records) {
    if (partition_index > 1 || slot_index >= DB_LOCATION_SLOT_COUNT ||
        s_partitions[partition_index] == NULL || header == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    size_t slot_offset = (size_t)slot_index * DB_LOCATION_SLOT_SIZE;
    esp_err_t err = esp_partition_read(s_partitions[partition_index],
                                       slot_offset, header, sizeof(*header));
    if (err != ESP_OK) {
        return err;
    }
    if (header->magic == DB_LOCATION_ERASED_WORD) {
        return ESP_ERR_NOT_FOUND;
    }
    if (!db_location_header_shape_valid(header, partition_index, slot_index)) {
        return ESP_ERR_INVALID_CRC;
    }
    if (header->payload_length > 0) {
        err = esp_partition_read(s_partitions[partition_index],
                                 slot_offset + DB_LOCATION_PAYLOAD_OFFSET,
                                 s_records, header->payload_length);
        if (err != ESP_OK) {
            return err;
        }
    }
    if (header->content_crc32c !=
        db_location_crc32c(s_records, header->payload_length)) {
        return ESP_ERR_INVALID_CRC;
    }
    uint64_t prior_id = 0;
    for (uint16_t i = 0; i < header->record_count; ++i) {
        if (!db_location_record_valid(&s_records[i]) ||
            (i > 0 && s_records[i].entity_id <= prior_id)) {
            return ESP_ERR_INVALID_CRC;
        }
        prior_id = s_records[i].entity_id;
    }
    if (!load_records && header->payload_length > 0) {
        memset(s_records, 0, header->payload_length);
    }
    return ESP_OK;
}

static int db_location_record_compare(const void *left, const void *right) {
    const db_location_record_t *a = left;
    const db_location_record_t *b = right;
    return a->entity_id < b->entity_id ? -1 :
           a->entity_id > b->entity_id ? 1 : 0;
}

static void db_location_prepare_header(db_location_snapshot_header_t *header,
                                       uint8_t partition_index,
                                       uint8_t slot_index) {
    header->magic = DB_LOCATION_MAGIC;
    header->format_version = DB_LOCATION_FORMAT_VERSION;
    header->protocol_version = DB_LOCATION_PROTOCOL_VERSION;
    header->header_size = sizeof(*header);
    header->record_size = sizeof(db_location_record_t);
    header->payload_length =
        (uint32_t)header->record_count * sizeof(db_location_record_t);
    header->content_crc32c =
        db_location_crc32c(s_records, header->payload_length);
    header->partition_index = partition_index;
    header->slot_index = slot_index;
    header->committed_unix = db_location_now();
    header->header_crc32c = 0;
    header->header_crc32c = db_location_crc32c(
        header, offsetof(db_location_snapshot_header_t, header_crc32c));
}

static esp_err_t db_location_write_snapshot(
    uint8_t partition_index, uint8_t slot_index,
    db_location_snapshot_header_t *header) {
    size_t slot_offset = (size_t)slot_index * DB_LOCATION_SLOT_SIZE;
    db_location_prepare_header(header, partition_index, slot_index);
    esp_err_t err = esp_partition_erase_range(s_partitions[partition_index],
                                               slot_offset,
                                               DB_LOCATION_SLOT_SIZE);
    if (err == ESP_OK && header->payload_length > 0) {
        err = esp_partition_write(s_partitions[partition_index],
                                  slot_offset + DB_LOCATION_PAYLOAD_OFFSET,
                                  s_records, header->payload_length);
    }
    if (err == ESP_OK) {
        /* Header is the commit marker and is deliberately written last. */
        err = esp_partition_write(s_partitions[partition_index], slot_offset,
                                  header, sizeof(*header));
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Snapshot write failed on %c/%u (%s)",
                 partition_index == 0 ? 'A' : 'B', slot_index,
                 esp_err_to_name(err));
        return err;
    }
    db_location_snapshot_header_t verified;
    err = db_location_load_slot(partition_index, slot_index, &verified, true);
    if (err != ESP_OK || verified.generation != header->generation ||
        verified.content_crc32c != header->content_crc32c) {
        ESP_LOGE(TAG, "Snapshot read-back failed on %c/%u (%s)",
                 partition_index == 0 ? 'A' : 'B', slot_index,
                 esp_err_to_name(err));
        return err == ESP_OK ? ESP_ERR_INVALID_CRC : err;
    }
    *header = verified;
    return ESP_OK;
}

static void db_location_reset_scan_state(void) {
    memset(s_slots, 0, sizeof(s_slots));
    memset(s_valid_slots, 0, sizeof(s_valid_slots));
    memset(s_newest_generation, 0, sizeof(s_newest_generation));
    memset(s_newest_slot, 0, sizeof(s_newest_slot));
    s_recovery_fault = false;
}

static esp_err_t db_location_scan_and_select(void) {
    db_location_reset_scan_state();
    bool found = false;
    db_location_snapshot_header_t selected = {0};
    uint8_t selected_partition = 0;
    uint8_t selected_slot = 0;

    for (uint8_t partition = 0; partition < 2; ++partition) {
        for (uint8_t slot = 0; slot < DB_LOCATION_SLOT_COUNT; ++slot) {
            db_location_snapshot_header_t header;
            esp_err_t err = db_location_load_slot(partition, slot, &header,
                                                   false);
            if (err != ESP_OK) {
                continue;
            }
            s_slots[partition][slot].valid = true;
            s_slots[partition][slot].header = header;
            ++s_valid_slots[partition];
            if (s_valid_slots[partition] == 1 ||
                db_location_generation_is_newer(
                    header.generation, s_newest_generation[partition])) {
                s_newest_generation[partition] = header.generation;
                s_newest_slot[partition] = slot;
            }
            if (!found || db_location_generation_is_newer(
                              header.generation, selected.generation)) {
                found = true;
                selected = header;
                selected_partition = partition;
                selected_slot = slot;
                s_recovery_fault = false;
            } else if (header.generation == selected.generation &&
                       (header.database_uuid != selected.database_uuid ||
                        header.content_crc32c != selected.content_crc32c ||
                        header.record_count != selected.record_count)) {
                s_recovery_fault = true;
            }
        }
    }
    if (!found) {
        return ESP_ERR_NOT_FOUND;
    }
    if (s_recovery_fault) {
        ESP_LOGE(TAG, "Equal-generation snapshots disagree; recovery required");
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t err = db_location_load_slot(selected_partition, selected_slot,
                                           &selected, true);
    if (err != ESP_OK) {
        return err;
    }
    s_active_header = selected;
    s_active_partition = selected_partition;
    s_active_slot = selected_slot;
    return ESP_OK;
}

static uint64_t db_location_new_uuid(void) {
    uint64_t value = 0;
    while (value == 0) {
        esp_fill_random(&value, sizeof(value));
    }
    return value;
}

static esp_err_t db_location_initialize_empty(void) {
    memset(s_records, 0,
           DB_LOCATION_MAX_RECORDS * sizeof(db_location_record_t));
    db_location_snapshot_header_t header = {0};
    header.database_uuid = db_location_new_uuid();
    header.generation = 0;
    header.snapshot_sequence = 1;
    header.next_internal_counter = 1;
    header.record_count = 0;
    header.created_unix = db_location_now();
    esp_err_t err = db_location_write_snapshot(0, 0, &header);
    if (err != ESP_OK) {
        return err;
    }
    header.snapshot_sequence = 1;
    err = db_location_write_snapshot(1, 0, &header);
    if (err == ESP_OK) {
        s_initialized_empty = true;
    }
    return err;
}

esp_err_t db_location_store_init(void) {
#if !CONFIG_DB_LOCATION_STORE
    return ESP_ERR_NOT_SUPPORTED;
#else
    if (s_init_attempted) {
        return s_init_result;
    }
    s_init_attempted = true;
    s_mutex = xSemaphoreCreateMutex();
    if (s_mutex == NULL) {
        s_init_result = ESP_ERR_NO_MEM;
        return s_init_result;
    }
    s_partitions[0] = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY,
        DB_LOCATION_PARTITION_A);
    s_partitions[1] = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY,
        DB_LOCATION_PARTITION_B);
    if (s_partitions[0] == NULL || s_partitions[1] == NULL) {
        ESP_LOGW(TAG, "A/B location database partitions are unavailable");
        s_init_result = ESP_ERR_NOT_FOUND;
        return s_init_result;
    }
    for (unsigned i = 0; i < 2; ++i) {
        if (s_partitions[i]->size !=
                DB_LOCATION_SLOT_SIZE * DB_LOCATION_SLOT_COUNT ||
            (s_partitions[i]->size % DB_LOCATION_ERASE_SECTOR) != 0) {
            ESP_LOGE(TAG, "Partition %s has unexpected size %u",
                     s_partitions[i]->label,
                     (unsigned)s_partitions[i]->size);
            s_init_result = ESP_ERR_INVALID_SIZE;
            return s_init_result;
        }
    }
    s_records = heap_caps_aligned_alloc(
        16, DB_LOCATION_MAX_RECORDS * sizeof(db_location_record_t),
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (s_records == NULL) {
        ESP_LOGE(TAG, "Unable to allocate bounded location DB cache in PSRAM");
        s_init_result = ESP_ERR_NO_MEM;
        return s_init_result;
    }
    memset(s_records, 0,
           DB_LOCATION_MAX_RECORDS * sizeof(db_location_record_t));

    s_init_result = db_location_scan_and_select();
    if (s_init_result == ESP_ERR_NOT_FOUND) {
        s_init_result = db_location_initialize_empty();
        if (s_init_result == ESP_OK) {
            s_init_result = db_location_scan_and_select();
        }
    }
    if (s_init_result == ESP_OK) {
        ESP_LOGI(TAG,
                 "Ready: UUID %016llx generation %lu, %u records, active %c/%u",
                 (unsigned long long)s_active_header.database_uuid,
                 (unsigned long)s_active_header.generation,
                 s_active_header.record_count,
                 s_active_partition == 0 ? 'A' : 'B', s_active_slot);
    }
    return s_init_result;
#endif
}

void db_location_store_get_status(db_location_status_t *status) {
    if (status == NULL) {
        return;
    }
    memset(status, 0, sizeof(*status));
#if CONFIG_DB_LOCATION_STORE
    status->recovery_fault = s_recovery_fault;
    status->initialized_empty = s_initialized_empty;
    if (s_init_result != ESP_OK || s_mutex == NULL ||
        xSemaphoreTake(s_mutex, portMAX_DELAY) != pdTRUE) {
        return;
    }
    status->available = true;
    status->database_uuid = s_active_header.database_uuid;
    status->generation = s_active_header.generation;
    status->content_crc32c = s_active_header.content_crc32c;
    status->snapshot_sequence = s_active_header.snapshot_sequence;
    status->next_internal_counter = s_active_header.next_internal_counter;
    status->record_count = s_active_header.record_count;
    status->active_partition = s_active_partition == 0 ? 'A' : 'B';
    status->active_slot = s_active_slot;
    status->valid_slots_a = s_valid_slots[0];
    status->valid_slots_b = s_valid_slots[1];
    status->newest_generation_a = s_newest_generation[0];
    status->newest_generation_b = s_newest_generation[1];
    status->partition_size_a = s_partitions[0]->size;
    status->partition_size_b = s_partitions[1]->size;
    xSemaphoreGive(s_mutex);
#endif
}

esp_err_t db_location_store_foreach(db_location_record_callback_t callback,
                                    void *context) {
#if !CONFIG_DB_LOCATION_STORE
    return ESP_ERR_NOT_SUPPORTED;
#else
    if (callback == NULL || db_location_store_init() != ESP_OK) {
        return ESP_ERR_INVALID_ARG;
    }
    if (xSemaphoreTake(s_mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    esp_err_t err = ESP_OK;
    for (uint16_t i = 0; i < s_active_header.record_count; ++i) {
        err = callback(&s_records[i], context);
        if (err != ESP_OK) {
            break;
        }
    }
    xSemaphoreGive(s_mutex);
    return err;
#endif
}

static int db_location_find_index(uint64_t entity_id) {
    for (uint16_t i = 0; i < s_active_header.record_count; ++i) {
        if (s_records[i].entity_id == entity_id) {
            return i;
        }
    }
    return -1;
}

static bool db_location_parent_valid(db_location_entity_type_t type,
                                     uint64_t parent_id) {
    if (type == DB_LOCATION_ENTITY_LAKE) {
        return parent_id == 0;
    }
    int index = db_location_find_index(parent_id);
    if (index < 0 ||
        (s_records[index].flags & DB_LOCATION_FLAG_TOMBSTONE) != 0) {
        return false;
    }
    if (type == DB_LOCATION_ENTITY_SWIM) {
        return s_records[index].entity_type == DB_LOCATION_ENTITY_LAKE;
    }
    if (type == DB_LOCATION_ENTITY_POINT) {
        return s_records[index].entity_type == DB_LOCATION_ENTITY_SWIM;
    }
    return true;
}

static bool db_location_point_code_available(uint64_t swim_id, uint8_t code,
                                              uint64_t except_entity) {
    if (code == 0 || code > 200) {
        return false;
    }
    for (uint16_t i = 0; i < s_active_header.record_count; ++i) {
        const db_location_record_t *record = &s_records[i];
        if (record->entity_id != except_entity &&
            record->entity_type == DB_LOCATION_ENTITY_POINT &&
            record->parent_id == swim_id && record->active_code == code &&
            (record->flags & DB_LOCATION_FLAG_TOMBSTONE) == 0) {
            return false;
        }
    }
    return true;
}

static esp_err_t db_location_set_name(db_location_record_t *record,
                                      const char *name) {
    if (name == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    size_t length = strlen(name);
    size_t maximum = record->entity_type == DB_LOCATION_ENTITY_POINT ? 24U :
                                                                       32U;
    if (length == 0 || length > maximum) {
        return ESP_ERR_INVALID_SIZE;
    }
    memset(record->name, 0, sizeof(record->name));
    memcpy(record->name, name, length);
    record->name_length = (uint8_t)length;
    return ESP_OK;
}

static esp_err_t db_location_commit_candidate(void) {
    uint8_t old_partition = s_active_partition;
    uint8_t old_slot = s_active_slot;
    db_location_snapshot_header_t old_header = s_active_header;
    uint8_t target_partition = old_partition ^ 1U;
    uint8_t target_slot = s_valid_slots[target_partition] == 0
                              ? 0
                              : (uint8_t)((s_newest_slot[target_partition] + 1U) %
                                          DB_LOCATION_SLOT_COUNT);
    s_active_header.generation += 1U;
    if (s_active_header.generation == 0) {
        s_active_header.generation = 1;
    }
    s_active_header.snapshot_sequence += 1U;
    esp_err_t err = db_location_write_snapshot(target_partition, target_slot,
                                                &s_active_header);
    if (err != ESP_OK) {
        s_active_header = old_header;
        s_active_partition = old_partition;
        s_active_slot = old_slot;
        db_location_snapshot_header_t reloaded;
        esp_err_t reload_err = db_location_load_slot(
            old_partition, old_slot, &reloaded, true);
        if (reload_err == ESP_OK) {
            s_active_header = reloaded;
        }
        return err;
    }
    s_active_partition = target_partition;
    s_active_slot = target_slot;
    return db_location_scan_and_select();
}

esp_err_t db_location_store_apply(const db_location_mutation_t *mutation,
                                  db_location_record_t *committed_record) {
#if !CONFIG_DB_LOCATION_STORE
    return ESP_ERR_NOT_SUPPORTED;
#else
    esp_err_t init_err = db_location_store_init();
    if (init_err != ESP_OK || mutation == NULL) {
        return init_err != ESP_OK ? init_err : ESP_ERR_INVALID_ARG;
    }
    if (xSemaphoreTake(s_mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    esp_err_t err = ESP_OK;
    db_location_record_t *record = NULL;
    int index = mutation->entity_id == 0
                    ? -1
                    : db_location_find_index(mutation->entity_id);
    uint32_t now = mutation->unix_time != 0 ? mutation->unix_time :
                                              db_location_now();

    if (mutation->action == DB_LOCATION_MUTATION_CREATE) {
        if (index >= 0 || s_active_header.record_count >= DB_LOCATION_MAX_RECORDS ||
            !db_location_parent_valid(mutation->entity_type,
                                      mutation->parent_id)) {
            err = ESP_ERR_INVALID_STATE;
            goto done;
        }
        uint32_t client_id = mutation->client_id == 0 ? 1 : mutation->client_id;
        uint32_t counter = mutation->creator_counter;
        if (counter == 0) {
            counter = s_active_header.next_internal_counter++;
            if (s_active_header.next_internal_counter == 0) {
                s_active_header.next_internal_counter = 1;
            }
        }
        uint64_t entity_id = mutation->entity_id != 0
                                 ? mutation->entity_id
                                 : ((uint64_t)client_id << 32) | counter;
        if (entity_id == 0 || db_location_find_index(entity_id) >= 0) {
            err = ESP_ERR_INVALID_STATE;
            goto done;
        }
        if (mutation->entity_type == DB_LOCATION_ENTITY_POINT &&
            (!db_location_point_code_available(mutation->parent_id,
                                               mutation->active_code, 0) ||
             mutation->latitude_e7 < -900000000 ||
             mutation->latitude_e7 > 900000000 ||
             mutation->longitude_e7 < -1800000000 ||
             mutation->longitude_e7 > 1800000000 ||
             mutation->depth_mm < -1)) {
            err = ESP_ERR_INVALID_ARG;
            goto done;
        }
        record = &s_records[s_active_header.record_count++];
        memset(record, 0, sizeof(*record));
        record->entity_id = entity_id;
        record->parent_id = mutation->parent_id;
        record->revision = 1;
        record->created_unix = now;
        record->updated_unix = now;
        record->entity_type = mutation->entity_type;
        record->active_code = mutation->active_code;
        record->role = mutation->role;
        record->latitude_e7 = mutation->latitude_e7;
        record->longitude_e7 = mutation->longitude_e7;
        record->depth_mm = mutation->depth_mm;
        record->saved_unix = now;
        record->mapping_revision = mutation->mapping_revision;
        record->navigation_revision = mutation->navigation_revision;
        err = db_location_set_name(record, mutation->name);
        if (err != ESP_OK) {
            --s_active_header.record_count;
            memset(record, 0, sizeof(*record));
            goto done;
        }
        db_location_record_finalize(record);
        qsort(s_records, s_active_header.record_count, sizeof(*s_records),
              db_location_record_compare);
        index = db_location_find_index(entity_id);
        record = index >= 0 ? &s_records[index] : NULL;
    } else {
        if (index < 0) {
            err = ESP_ERR_NOT_FOUND;
            goto done;
        }
        record = &s_records[index];
        if (mutation->expected_revision != record->revision) {
            err = ESP_ERR_INVALID_STATE;
            goto done;
        }
        if (mutation->action == DB_LOCATION_MUTATION_UPDATE) {
            err = db_location_set_name(record, mutation->name);
            if (err != ESP_OK) {
                goto done;
            }
        } else if (mutation->action == DB_LOCATION_MUTATION_ARCHIVE) {
            record->flags |= DB_LOCATION_FLAG_ARCHIVED;
        } else if (mutation->action == DB_LOCATION_MUTATION_RESTORE) {
            record->flags &= (uint16_t)~DB_LOCATION_FLAG_ARCHIVED;
        } else if (mutation->action == DB_LOCATION_MUTATION_DELETE) {
            record->flags |= DB_LOCATION_FLAG_TOMBSTONE;
            record->active_code = 0;
        } else {
            err = ESP_ERR_INVALID_ARG;
            goto done;
        }
        record->revision += 1U;
        record->updated_unix = now;
        db_location_record_finalize(record);
    }

    err = db_location_commit_candidate();
    if (err == ESP_OK && committed_record != NULL && record != NULL) {
        int committed_index = db_location_find_index(record->entity_id);
        if (committed_index >= 0) {
            *committed_record = s_records[committed_index];
        }
    }
done:
    xSemaphoreGive(s_mutex);
    return err;
#endif
}

#if CONFIG_DB_LOCATION_BENCH_TEST
esp_err_t db_location_store_bench_seed(void) {
    db_location_status_t status;
    db_location_store_get_status(&status);
    if (!status.available || status.record_count != 0) {
        return ESP_ERR_INVALID_STATE;
    }
    db_location_record_t lake;
    db_location_mutation_t mutation = {
        .action = DB_LOCATION_MUTATION_CREATE,
        .entity_type = DB_LOCATION_ENTITY_LAKE,
        .name = "Bench Lake",
    };
    esp_err_t err = db_location_store_apply(&mutation, &lake);
    if (err != ESP_OK) {
        return err;
    }
    db_location_record_t swim;
    mutation = (db_location_mutation_t){
        .action = DB_LOCATION_MUTATION_CREATE,
        .entity_type = DB_LOCATION_ENTITY_SWIM,
        .parent_id = lake.entity_id,
        .name = "Bench Swim",
        .mapping_revision = 1,
    };
    err = db_location_store_apply(&mutation, &swim);
    if (err != ESP_OK) {
        return err;
    }
    mutation = (db_location_mutation_t){
        .action = DB_LOCATION_MUTATION_CREATE,
        .entity_type = DB_LOCATION_ENTITY_POINT,
        .parent_id = swim.entity_id,
        .name = "Bench Point",
        .active_code = 16,
        .role = 1,
        .latitude_e7 = 427000000,
        .longitude_e7 = 235000000,
        .depth_mm = 2180,
        .navigation_revision = 1,
    };
    return db_location_store_apply(&mutation, NULL);
}

esp_err_t db_location_store_bench_rename_point(void) {
    if (db_location_store_init() != ESP_OK ||
        xSemaphoreTake(s_mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_INVALID_STATE;
    }
    uint64_t point_id = 0;
    uint32_t revision = 0;
    for (uint16_t i = 0; i < s_active_header.record_count; ++i) {
        if (s_records[i].entity_type == DB_LOCATION_ENTITY_POINT &&
            (s_records[i].flags & DB_LOCATION_FLAG_TOMBSTONE) == 0) {
            point_id = s_records[i].entity_id;
            revision = s_records[i].revision;
            break;
        }
    }
    xSemaphoreGive(s_mutex);
    if (point_id == 0) {
        return ESP_ERR_NOT_FOUND;
    }
    db_location_mutation_t mutation = {
        .action = DB_LOCATION_MUTATION_UPDATE,
        .entity_id = point_id,
        .expected_revision = revision,
        .name = "Bench Point Updated",
    };
    return db_location_store_apply(&mutation, NULL);
}

esp_err_t db_location_store_bench_corrupt_active_header(void) {
    if (db_location_store_init() != ESP_OK ||
        xSemaphoreTake(s_mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_INVALID_STATE;
    }
    uint8_t other = s_active_partition ^ 1U;
    if (s_valid_slots[other] == 0) {
        xSemaphoreGive(s_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    size_t offset = (size_t)s_active_slot * DB_LOCATION_SLOT_SIZE;
    esp_err_t err = esp_partition_erase_range(
        s_partitions[s_active_partition], offset, DB_LOCATION_ERASE_SECTOR);
    xSemaphoreGive(s_mutex);
    return err;
}
#endif

#else

esp_err_t db_location_store_init(void) { return ESP_ERR_NOT_SUPPORTED; }

void db_location_store_get_status(db_location_status_t *status) {
    if (status != NULL) {
        memset(status, 0, sizeof(*status));
    }
}

esp_err_t db_location_store_foreach(db_location_record_callback_t callback,
                                    void *context) {
    (void)callback;
    (void)context;
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t db_location_store_apply(const db_location_mutation_t *mutation,
                                  db_location_record_t *committed_record) {
    (void)mutation;
    (void)committed_record;
    return ESP_ERR_NOT_SUPPORTED;
}

#endif
