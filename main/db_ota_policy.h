/*
 *  DBB Companion - OTA safety policy.
 *
 *  Recovery model (see partitions_s3_16mb.csv):
 *    factory   known-good golden image. ESP-IDF's OTA rotation never targets a
 *              factory partition, so it cannot be overwritten by an update.
 *    ota_0/1   the A/B update slots. app_count is 2, so the bootloader's
 *              (ota_seq - 1) % app_count matches this module's two-slot view.
 *
 *  Two independent bootloader paths fall back to factory: both otadata entries
 *  invalid, and the backwards partition scan when an image fails to verify.
 *  Neither needs help from firmware.
 *
 *  This module owns the one thing firmware MUST do: confirm a freshly booted
 *  image, or ask for it to be rolled back.
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  Copyright 2026 Valentin Danev
 */
#ifndef DBB_OTA_POLICY_H
#define DBB_OTA_POLICY_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_partition.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DB_OTA_FACTORY_PARTITION_LABEL "factory"

/*
 * Health items. Deliberately no "authenticated maintenance" or "staged signed
 * update" bits - those named Stage-B subsystems that no longer exist.
 */
#define DB_OTA_HEALTH_NVS (1U << 0)
#define DB_OTA_HEALTH_DIAGNOSTICS (1U << 1)
#define DB_OTA_HEALTH_LOCATION_DB (1U << 2)
#define DB_OTA_HEALTH_LOGS (1U << 3)
#define DB_OTA_HEALTH_WEB (1U << 4)
#define DB_OTA_HEALTH_RADIO (1U << 5)
#define DB_OTA_HEALTH_CONTROL (1U << 6)
#define DB_OTA_HEALTH_REST (1U << 7)
#define DB_OTA_HEALTH_FC (1U << 8)
#define DB_OTA_HEALTH_SONAR (1U << 9)
#define DB_OTA_HEALTH_ALL_MASK ((1U << 10) - 1U)

typedef enum {
    DB_OTA_HEALTH_NOT_REQUIRED = 0, /* running image is already confirmed */
    DB_OTA_HEALTH_WAITING,          /* gate armed, items still outstanding */
    DB_OTA_HEALTH_CONFIRMED,        /* image marked valid, rollback cancelled */
    DB_OTA_HEALTH_ROLLBACK_REQUESTED,
    DB_OTA_HEALTH_ERROR,
} db_ota_health_state_t;

typedef struct {
    db_ota_health_state_t state;
    bool running_pending_verify;
    bool gate_compiled_in;
    uint32_t required_mask;
    uint32_t passed_mask;
    uint32_t failed_mask;
    uint32_t elapsed_ms;
    uint32_t timeout_ms;
} db_ota_health_status_t;

typedef struct {
    bool factory_present;
    bool factory_bootable;
    uint32_t factory_size;
    bool running_is_factory;
    bool running_pending_verify;
    bool rollback_supported; /* CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE */
    char running_label[17];
    char boot_label[17];
    uint32_t crash_streak; /* consecutive crash reboots in the running slot */
} db_ota_slot_info_t;

/* --- slot queries ------------------------------------------------------- */

bool db_ota_partition_is_normal_slot(const esp_partition_t *partition);
const esp_partition_t *db_ota_select_normal_update_partition(void);
const esp_partition_t *db_ota_get_configured_boot_partition(void);
bool db_ota_running_is_pending_verify(void);
void db_ota_get_slot_info(db_ota_slot_info_t *info);

/* --- crash guard -------------------------------------------------------- */

/*
 * Call once, as early in app_main as practical. Counts consecutive crash-caused
 * reboots of the running slot in RTC no-init memory and, on the third, demotes
 * the slot and reboots so the bootloader picks another image.
 */
void db_ota_crash_guard_start(void);

/*
 * Call once the boot is judged good. Clears the crash streak, and - critically -
 * confirms the running image when no health gate is compiled in. With
 * CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE set, an image that is never confirmed is
 * rolled back on the next boot, so SOMETHING must confirm it.
 */
void db_ota_crash_guard_mark_healthy(void);

/* --- health gate -------------------------------------------------------- */

esp_err_t db_ota_health_gate_start(uint32_t required_mask,
                                   uint32_t initially_passed_mask);
void db_ota_health_mark_pass(uint32_t mask);
void db_ota_health_mark_fail(uint32_t mask);
void db_ota_health_get_status(db_ota_health_status_t *status);

#ifdef __cplusplus
}
#endif

#endif /* DBB_OTA_POLICY_H */
