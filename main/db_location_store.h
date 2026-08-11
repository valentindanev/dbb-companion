#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DB_LOCATION_PARTITION_A "db_a"
#define DB_LOCATION_PARTITION_B "db_b"
#define DB_LOCATION_FORMAT_VERSION 1U
#define DB_LOCATION_PROTOCOL_VERSION 1U
#define DB_LOCATION_RECORD_SIZE 96U
#define DB_LOCATION_MAX_RECORDS 768U
#define DB_LOCATION_NAME_BYTES 32U

typedef enum {
    DB_LOCATION_ENTITY_LAKE = 1,
    DB_LOCATION_ENTITY_SWIM = 2,
    DB_LOCATION_ENTITY_POINT = 3,
    DB_LOCATION_ENTITY_TRIP = 4,
    DB_LOCATION_ENTITY_USAGE = 5,
} db_location_entity_type_t;

typedef enum {
    DB_LOCATION_FLAG_ARCHIVED = 1U << 0,
    DB_LOCATION_FLAG_TOMBSTONE = 1U << 1,
    DB_LOCATION_FLAG_NEEDS_REVIEW = 1U << 2,
} db_location_record_flag_t;

typedef struct __attribute__((packed)) {
    uint64_t entity_id;
    uint64_t parent_id;
    uint32_t revision;
    uint32_t created_unix;
    uint32_t updated_unix;
    uint32_t mapping_revision;
    uint32_t navigation_revision;
    int32_t latitude_e7;
    int32_t longitude_e7;
    int32_t depth_mm;
    uint32_t saved_unix;
    uint16_t flags;
    uint8_t entity_type;
    uint8_t name_length;
    uint8_t active_code;
    uint8_t role;
    uint16_t reserved0;
    char name[DB_LOCATION_NAME_BYTES];
    uint16_t record_crc16;
    uint8_t reserved1[2];
} db_location_record_t;

typedef struct {
    bool available;
    bool recovery_fault;
    bool initialized_empty;
    uint64_t database_uuid;
    uint32_t generation;
    uint32_t content_crc32c;
    uint32_t snapshot_sequence;
    uint32_t next_internal_counter;
    uint16_t record_count;
    char active_partition;
    uint8_t active_slot;
    uint8_t valid_slots_a;
    uint8_t valid_slots_b;
    uint32_t newest_generation_a;
    uint32_t newest_generation_b;
    size_t partition_size_a;
    size_t partition_size_b;
} db_location_status_t;

typedef enum {
    DB_LOCATION_MUTATION_CREATE = 1,
    DB_LOCATION_MUTATION_UPDATE = 2,
    DB_LOCATION_MUTATION_ARCHIVE = 3,
    DB_LOCATION_MUTATION_RESTORE = 4,
    DB_LOCATION_MUTATION_DELETE = 5,
} db_location_mutation_action_t;

typedef struct {
    db_location_mutation_action_t action;
    db_location_entity_type_t entity_type;
    uint64_t entity_id;
    uint64_t parent_id;
    uint32_t expected_revision;
    uint32_t client_id;
    uint32_t creator_counter;
    const char *name;
    uint8_t active_code;
    uint8_t role;
    int32_t latitude_e7;
    int32_t longitude_e7;
    int32_t depth_mm;
    uint32_t mapping_revision;
    uint32_t navigation_revision;
    uint32_t unix_time;
} db_location_mutation_t;

typedef esp_err_t (*db_location_record_callback_t)(
    const db_location_record_t *record, void *context);

esp_err_t db_location_store_init(void);
void db_location_store_get_status(db_location_status_t *status);
esp_err_t db_location_store_foreach(db_location_record_callback_t callback,
                                    void *context);
esp_err_t db_location_store_apply(const db_location_mutation_t *mutation,
                                  db_location_record_t *committed_record);

#if CONFIG_DB_LOCATION_BENCH_TEST
esp_err_t db_location_store_bench_seed(void);
esp_err_t db_location_store_bench_rename_point(void);
esp_err_t db_location_store_bench_corrupt_active_header(void);
#endif

#ifdef __cplusplus
}
#endif
