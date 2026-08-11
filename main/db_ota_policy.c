/*
 *  DBB Companion - OTA safety policy.
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  Copyright 2026 Valentin Danev
 */
#include "db_ota_policy.h"

#include <string.h>

#include "esp_attr.h"
#include "esp_image_format.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#if CONFIG_DB_DIAG_JOURNAL
#include "db_diag.h"
#endif

/*
 * The gate depends on the bootloader promoting a new image to PENDING_VERIFY.
 * Without CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE that never happens, the gate's
 * arming condition can never be true, and it silently does nothing - which is
 * exactly how twelve health checks sat dead in the pre-rebuild firmware. The
 * Kconfig `depends on` should prevent this pairing; this makes it impossible.
 */
#if defined(CONFIG_DB_OTA_HEALTH_GATE) && \
    !defined(CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE)
#error "DB_OTA_HEALTH_GATE requires BOOTLOADER_APP_ROLLBACK_ENABLE; without it the gate can never arm."
#endif

#ifndef CONFIG_DB_OTA_HEALTH_TIMEOUT_SECONDS
#define CONFIG_DB_OTA_HEALTH_TIMEOUT_SECONDS 600
#endif

#define DB_OTA_CRASH_GUARD_MAGIC 0x44424247U /* "DBBG" */
#define DB_OTA_CRASH_LIMIT 3U

typedef struct {
    uint32_t magic;
    uint32_t partition_subtype;
    uint32_t consecutive_crashes;
} db_ota_crash_guard_t;

RTC_NOINIT_ATTR static db_ota_crash_guard_t s_crash_guard;

static const char *TAG = "db_ota_policy";

static volatile uint32_t s_required_mask;
static volatile uint32_t s_passed_mask;
static volatile uint32_t s_failed_mask;
static volatile int s_health_state = DB_OTA_HEALTH_NOT_REQUIRED;
static TickType_t s_started_at;
#ifdef CONFIG_DB_OTA_HEALTH_GATE
static TaskHandle_t s_health_task;
#endif

/* ------------------------------------------------------------------ slots */

bool db_ota_partition_is_normal_slot(const esp_partition_t *partition) {
    return partition != NULL && partition->type == ESP_PARTITION_TYPE_APP &&
           (partition->subtype == ESP_PARTITION_SUBTYPE_APP_OTA_0 ||
            partition->subtype == ESP_PARTITION_SUBTYPE_APP_OTA_1);
}

static const esp_partition_t *db_ota_find_factory(void) {
    return esp_partition_find_first(ESP_PARTITION_TYPE_APP,
                                    ESP_PARTITION_SUBTYPE_APP_FACTORY, NULL);
}

static bool db_ota_image_is_bootable(const esp_partition_t *partition) {
    if (partition == NULL || partition->type != ESP_PARTITION_TYPE_APP) {
        return false;
    }
    esp_partition_pos_t position = {
        .offset = partition->address,
        .size = partition->size,
    };
    esp_image_metadata_t metadata;
    return esp_image_verify(ESP_IMAGE_VERIFY_SILENT, &position, &metadata) ==
           ESP_OK;
}

/*
 * esp_ota_get_next_update_partition() already refuses to return a factory
 * partition, so the golden image is protected structurally rather than by a
 * label comparison. The extra check here only asserts that invariant.
 */
const esp_partition_t *db_ota_select_normal_update_partition(void) {
    const esp_partition_t *target = esp_ota_get_next_update_partition(NULL);
    if (!db_ota_partition_is_normal_slot(target)) {
        ESP_LOGE(TAG, "Refusing OTA: no A/B slot available as a target");
        return NULL;
    }
    if (target == esp_ota_get_running_partition()) {
        ESP_LOGE(TAG, "Refusing OTA: target is the running slot");
        return NULL;
    }
    return target;
}

const esp_partition_t *db_ota_get_configured_boot_partition(void) {
    return esp_ota_get_boot_partition();
}

bool db_ota_running_is_pending_verify(void) {
#ifdef CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t state;
    if (running == NULL ||
        esp_ota_get_state_partition(running, &state) != ESP_OK) {
        return false;
    }
    return state == ESP_OTA_IMG_PENDING_VERIFY;
#else
    return false;
#endif
}

static void db_ota_copy_label(char *dst, size_t dst_size,
                              const esp_partition_t *partition) {
    if (dst == NULL || dst_size == 0) return;
    if (partition == NULL) {
        dst[0] = '\0';
        return;
    }
    strncpy(dst, partition->label, dst_size - 1);
    dst[dst_size - 1] = '\0';
}

void db_ota_get_slot_info(db_ota_slot_info_t *info) {
    if (info == NULL) return;
    memset(info, 0, sizeof(*info));

    const esp_partition_t *factory = db_ota_find_factory();
    info->factory_present = factory != NULL;
    if (factory != NULL) {
        info->factory_size = (uint32_t)factory->size;
        info->factory_bootable = db_ota_image_is_bootable(factory);
    }

    const esp_partition_t *running = esp_ota_get_running_partition();
    info->running_is_factory =
        running != NULL &&
        running->subtype == ESP_PARTITION_SUBTYPE_APP_FACTORY;
    db_ota_copy_label(info->running_label, sizeof(info->running_label),
                      running);
    db_ota_copy_label(info->boot_label, sizeof(info->boot_label),
                      esp_ota_get_boot_partition());
    info->running_pending_verify = db_ota_running_is_pending_verify();
#ifdef CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE
    info->rollback_supported = true;
#endif
    if (s_crash_guard.magic == DB_OTA_CRASH_GUARD_MAGIC && running != NULL &&
        s_crash_guard.partition_subtype == (uint32_t)running->subtype) {
        info->crash_streak = s_crash_guard.consecutive_crashes;
    }
}

/* ------------------------------------------------------ confirm / rollback */

static esp_err_t db_ota_confirm_running(void) {
#ifdef CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE
    if (!db_ota_running_is_pending_verify()) {
        return ESP_OK; /* already valid, or factory - nothing to confirm */
    }
    esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Running image confirmed; rollback cancelled");
#if CONFIG_DB_DIAG_JOURNAL
        ESP_ERROR_CHECK_WITHOUT_ABORT(db_diag_log_ota_confirmed());
#endif
    } else {
        ESP_LOGE(TAG, "Failed to confirm running image (%s)",
                 esp_err_to_name(err));
    }
    return err;
#else
    return ESP_OK;
#endif
}

/*
 * Ask for the running image to be abandoned. Preferred route is IDF's own
 * rollback, which is only valid while the image is PENDING_VERIFY. An image
 * that already confirmed itself and only later began crash-looping is past that
 * point, so fall back to explicitly selecting another bootable image.
 */
static esp_err_t db_ota_abandon_running(void) {
#ifdef CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE
    if (db_ota_running_is_pending_verify()) {
        ESP_LOGE(TAG, "Marking running image invalid and rebooting");
        return esp_ota_mark_app_invalid_rollback_and_reboot(); /* no return */
    }
#endif
    const esp_partition_t *running = esp_ota_get_running_partition();
    const esp_partition_t *candidate = NULL;

    if (db_ota_partition_is_normal_slot(running)) {
        esp_partition_subtype_t other =
            running->subtype == ESP_PARTITION_SUBTYPE_APP_OTA_0
                ? ESP_PARTITION_SUBTYPE_APP_OTA_1
                : ESP_PARTITION_SUBTYPE_APP_OTA_0;
        candidate = esp_partition_find_first(ESP_PARTITION_TYPE_APP, other,
                                             NULL);
        if (!db_ota_image_is_bootable(candidate)) {
            candidate = NULL;
        }
    }
    if (candidate == NULL) {
        const esp_partition_t *factory = db_ota_find_factory();
        if (factory != running && db_ota_image_is_bootable(factory)) {
            candidate = factory;
            ESP_LOGE(TAG, "No healthy A/B slot; falling back to factory");
        }
    }
    if (candidate == NULL) {
        ESP_LOGE(TAG, "No bootable alternative image exists; staying put");
        return ESP_ERR_NOT_FOUND;
    }

    esp_err_t err = esp_ota_set_boot_partition(candidate);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not select '%s' (%s)", candidate->label,
                 esp_err_to_name(err));
        return err;
    }
    ESP_LOGE(TAG, "Rebooting into '%s'", candidate->label);
    esp_restart();
    return ESP_OK;
}

/* ----------------------------------------------------------- crash guard */

static bool db_ota_reset_is_crash(esp_reset_reason_t reason) {
    return reason == ESP_RST_PANIC || reason == ESP_RST_INT_WDT ||
           reason == ESP_RST_TASK_WDT || reason == ESP_RST_WDT;
}

void db_ota_crash_guard_start(void) {
    const esp_partition_t *running = esp_ota_get_running_partition();

    /*
     * The factory image is the last line of defence. Never demote it - there is
     * nothing to fall back to, and a reboot loop there is better diagnosed than
     * hidden by bouncing between broken slots.
     */
    if (!db_ota_partition_is_normal_slot(running)) {
        memset(&s_crash_guard, 0, sizeof(s_crash_guard));
        return;
    }

    if (!db_ota_reset_is_crash(esp_reset_reason())) {
        memset(&s_crash_guard, 0, sizeof(s_crash_guard));
        return;
    }

    if (s_crash_guard.magic == DB_OTA_CRASH_GUARD_MAGIC &&
        s_crash_guard.partition_subtype == (uint32_t)running->subtype) {
        s_crash_guard.consecutive_crashes++;
    } else {
        s_crash_guard.magic = DB_OTA_CRASH_GUARD_MAGIC;
        s_crash_guard.partition_subtype = (uint32_t)running->subtype;
        s_crash_guard.consecutive_crashes = 1U;
    }

    if (s_crash_guard.consecutive_crashes < DB_OTA_CRASH_LIMIT) {
        ESP_LOGW(TAG, "Boot crash guard %u/%u for '%s'",
                 (unsigned)s_crash_guard.consecutive_crashes,
                 (unsigned)DB_OTA_CRASH_LIMIT, running->label);
        return;
    }

    ESP_LOGE(TAG, "'%s' crashed %u times consecutively; abandoning it",
             running->label, (unsigned)s_crash_guard.consecutive_crashes);
    memset(&s_crash_guard, 0, sizeof(s_crash_guard));
    (void)db_ota_abandon_running();
}

void db_ota_crash_guard_mark_healthy(void) {
    memset(&s_crash_guard, 0, sizeof(s_crash_guard));
#ifndef CONFIG_DB_OTA_HEALTH_GATE
    /*
     * No gate is compiled in, so nothing else will ever confirm this image -
     * and with rollback enabled an unconfirmed image is discarded on the next
     * boot. Confirm here, otherwise every OTA silently reverts.
     */
    (void)db_ota_confirm_running();
#endif
}

/* ------------------------------------------------------------ health gate */

static db_ota_health_state_t db_ota_health_state_load(void) {
    return (db_ota_health_state_t)__atomic_load_n(&s_health_state,
                                                  __ATOMIC_ACQUIRE);
}

static void db_ota_health_state_store(db_ota_health_state_t state) {
    __atomic_store_n(&s_health_state, (int)state, __ATOMIC_RELEASE);
}

/* Only referenced from db_ota_health_gate_start()'s gate-enabled branch. */
#ifdef CONFIG_DB_OTA_HEALTH_GATE
static void db_ota_health_task(void *argument) {
    (void)argument;
    const TickType_t timeout_ticks =
        pdMS_TO_TICKS(CONFIG_DB_OTA_HEALTH_TIMEOUT_SECONDS * 1000U);

    while (true) {
        uint32_t required = __atomic_load_n(&s_required_mask, __ATOMIC_ACQUIRE);
        uint32_t passed = __atomic_load_n(&s_passed_mask, __ATOMIC_ACQUIRE);
        uint32_t failed = __atomic_load_n(&s_failed_mask, __ATOMIC_ACQUIRE);

        if ((failed & required) != 0U) {
            ESP_LOGE(TAG, "Health gate FAILED (required=%08x failed=%08x)",
                     (unsigned)required, (unsigned)failed);
            break;
        }

        if ((passed & required) == required) {
            esp_err_t err = db_ota_confirm_running();
            db_ota_health_state_store(err == ESP_OK ? DB_OTA_HEALTH_CONFIRMED
                                                    : DB_OTA_HEALTH_ERROR);
            s_health_task = NULL;
            vTaskDelete(NULL);
            return;
        }

        if ((xTaskGetTickCount() - s_started_at) >= timeout_ticks) {
            ESP_LOGE(TAG, "Health gate TIMED OUT (required=%08x passed=%08x)",
                     (unsigned)required, (unsigned)passed);
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }

    db_ota_health_state_store(DB_OTA_HEALTH_ROLLBACK_REQUESTED);
    esp_err_t err = db_ota_abandon_running(); /* normally does not return */
    db_ota_health_state_store(DB_OTA_HEALTH_ERROR);
    ESP_LOGE(TAG, "Rollback request failed (%s)", esp_err_to_name(err));
    s_health_task = NULL;
    vTaskDelete(NULL);
}
#endif /* CONFIG_DB_OTA_HEALTH_GATE */

esp_err_t db_ota_health_gate_start(uint32_t required_mask,
                                   uint32_t initially_passed_mask) {
    if (required_mask == 0U ||
        (required_mask & ~DB_OTA_HEALTH_ALL_MASK) != 0U ||
        (initially_passed_mask & ~required_mask) != 0U) {
        return ESP_ERR_INVALID_ARG;
    }

#ifndef CONFIG_DB_OTA_HEALTH_GATE
    db_ota_health_state_store(DB_OTA_HEALTH_NOT_REQUIRED);
    return ESP_OK;
#else
    if (!db_ota_running_is_pending_verify()) {
        db_ota_health_state_store(DB_OTA_HEALTH_NOT_REQUIRED);
        return ESP_OK;
    }
    if (s_health_task != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    __atomic_store_n(&s_required_mask, required_mask, __ATOMIC_RELEASE);
    __atomic_store_n(&s_passed_mask, initially_passed_mask, __ATOMIC_RELEASE);
    __atomic_store_n(&s_failed_mask, 0U, __ATOMIC_RELEASE);
    s_started_at = xTaskGetTickCount();
    db_ota_health_state_store(DB_OTA_HEALTH_WAITING);

    if (xTaskCreate(db_ota_health_task, "db_ota_health", 4096, NULL, 5,
                    &s_health_task) != pdPASS) {
        s_health_task = NULL;
        db_ota_health_state_store(DB_OTA_HEALTH_ERROR);
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGW(TAG, "Image awaits health gate %08x (already passed %08x)",
             (unsigned)required_mask, (unsigned)initially_passed_mask);
    return ESP_OK;
#endif
}

void db_ota_health_mark_pass(uint32_t mask) {
    if (db_ota_health_state_load() != DB_OTA_HEALTH_WAITING) return;
    uint32_t required = __atomic_load_n(&s_required_mask, __ATOMIC_ACQUIRE);
    __atomic_fetch_or(&s_passed_mask, mask & required, __ATOMIC_ACQ_REL);
}

void db_ota_health_mark_fail(uint32_t mask) {
    if (db_ota_health_state_load() != DB_OTA_HEALTH_WAITING) return;
    uint32_t required = __atomic_load_n(&s_required_mask, __ATOMIC_ACQUIRE);
    __atomic_fetch_or(&s_failed_mask, mask & required, __ATOMIC_ACQ_REL);
}

void db_ota_health_get_status(db_ota_health_status_t *status) {
    if (status == NULL) return;

    memset(status, 0, sizeof(*status));
    status->state = db_ota_health_state_load();
    status->running_pending_verify = db_ota_running_is_pending_verify();
    status->required_mask = __atomic_load_n(&s_required_mask, __ATOMIC_ACQUIRE);
    status->passed_mask = __atomic_load_n(&s_passed_mask, __ATOMIC_ACQUIRE);
    status->failed_mask = __atomic_load_n(&s_failed_mask, __ATOMIC_ACQUIRE);
#ifdef CONFIG_DB_OTA_HEALTH_GATE
    status->gate_compiled_in = true;
    status->timeout_ms = CONFIG_DB_OTA_HEALTH_TIMEOUT_SECONDS * 1000U;
#endif
    if (status->state == DB_OTA_HEALTH_WAITING) {
        status->elapsed_ms = pdTICKS_TO_MS(xTaskGetTickCount() - s_started_at);
    }
}
