#include "db_netlog.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "db_sonar_log.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#define TAG "DB_NETLOG"

#define DB_NETLOG_MSG_LEN 128
#define DB_NETLOG_DEPTH 12
/* At most this many are written per tick so a burst of Wi-Fi churn can never
 * monopolise the 2 Hz sonar timer task that drains us. */
#define DB_NETLOG_DRAIN_PER_TICK 4
#define DB_NETLOG_HEAP_PERIOD_MS 30000U

typedef struct {
    char text[DB_NETLOG_MSG_LEN];
} db_netlog_msg_t;

static QueueHandle_t s_queue = NULL;
static volatile uint32_t s_dropped = 0;
static uint32_t s_last_heap_ms = 0;

void db_netlog_init(void) {
    if (s_queue != NULL) return;
    s_queue = xQueueCreate(DB_NETLOG_DEPTH, sizeof(db_netlog_msg_t));
    if (s_queue == NULL) {
        ESP_LOGE(TAG, "queue alloc failed - Wi-Fi events will not be logged");
    }
}

void db_netlog_note(const char *fmt, ...) {
    if (s_queue == NULL || fmt == NULL) return;
    db_netlog_msg_t msg;
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(msg.text, sizeof(msg.text), fmt, ap);
    va_end(ap);
    /* Zero timeout: this runs on the Wi-Fi event task and must never wait. */
    if (xQueueSend(s_queue, &msg, 0) != pdTRUE) {
        s_dropped++;
    }
}

static uint32_t db_netlog_now_ms(void) {
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

static void db_netlog_emit_heap(void) {
    /* Internal DMA-capable RAM is the pool Wi-Fi and lwIP draw from. Total
     * free (esp_get_free_heap_size) includes PSRAM and can look healthy while
     * internal RAM is exhausted, which is exactly the failure that would kill
     * a socket without crashing the device - so log internal separately, and
     * the largest free block with it, because fragmentation fails allocations
     * long before the total reaches zero. */
    size_t free_int = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    size_t min_int = heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    size_t big_int = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    db_sonar_log_log_system_kv(
        "event=heap free=%lu min_free=%lu free_int=%lu min_int=%lu largest_int=%lu",
        (unsigned long)esp_get_free_heap_size(),
        (unsigned long)esp_get_minimum_free_heap_size(),
        (unsigned long)free_int, (unsigned long)min_int,
        (unsigned long)big_int);
}

void db_netlog_tick(void) {
    if (s_queue != NULL) {
        db_netlog_msg_t msg;
        for (int i = 0; i < DB_NETLOG_DRAIN_PER_TICK; i++) {
            if (xQueueReceive(s_queue, &msg, 0) != pdTRUE) break;
            uint32_t dropped = s_dropped;
            if (dropped != 0) {
                s_dropped = 0;
                db_sonar_log_log_system_kv("%s dropped_before=%lu", msg.text,
                                           (unsigned long)dropped);
            } else {
                db_sonar_log_log_system_kv("%s", msg.text);
            }
        }
    }

    uint32_t now = db_netlog_now_ms();
    if (s_last_heap_ms == 0 || (now - s_last_heap_ms) >= DB_NETLOG_HEAP_PERIOD_MS) {
        s_last_heap_ms = now;
        db_netlog_emit_heap();
    }
}
