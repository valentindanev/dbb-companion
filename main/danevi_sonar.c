#include "danevi_sonar.h"

#include "driver/gpio.h"
#include "driver/uart.h"
#include "db_sonar_log.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>


#define SONAR_UART_NUM UART_NUM_2
#define SONAR_BAUD_RATE 115200
#define BUF_SIZE 256
#define DANEVI_SONAR_TASK_STACK_SIZE 4096
#define DANEVI_DISTANCE_STALE_MS 3000
#define DANEVI_ZERO_HOLD_GRACE_MS 600
#define DANEVI_DEBUG_MAX_LINES 8
#define DANEVI_DEBUG_LINE_MAX 160
#define DANEVI_FRAME_SIZE 4
#define DANEVI_TRIGGER_BYTE 0xFF
#define DANEVI_RESPONSE_TIMEOUT_MS 30
#define DANEVI_TRIGGER_INTERVAL_MS 100

static const char *TAG = "DANEVI_SONAR";

// Last good non-zero hardwired depth that is allowed to reach MAVLink/UI.
static int g_current_distance_mm = -1;
static uint32_t g_distance_update_ms = 0;
// Last raw hardwired frame value, including zeroes.
static int g_raw_distance_mm = -1;
static uint32_t g_raw_distance_update_ms = 0;
// Zero-run state so brief splash/out-of-water events can hold the last good
// depth briefly instead of collapsing immediately to 0 mm.
static uint32_t g_consecutive_zero_frames = 0;
static uint32_t g_zero_run_start_ms = 0;
static SemaphoreHandle_t g_distance_mutex = NULL;
static SemaphoreHandle_t g_debug_mutex = NULL;
static char g_debug_lines[DANEVI_DEBUG_MAX_LINES][DANEVI_DEBUG_LINE_MAX];
static size_t g_debug_count = 0;
static size_t g_debug_next = 0;
static int g_last_logged_distance_mm = -1;
static TickType_t g_last_distance_log_tick = 0;
static TickType_t g_last_frame_issue_log_tick = 0;
static TickType_t g_last_no_response_log_tick = 0;

static uint32_t danevi_now_ms(void) {
  return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

static bool danevi_zero_run_is_active_locked(void) {
  return g_consecutive_zero_frames > 0 && g_zero_run_start_ms != 0;
}

static bool danevi_get_filtered_distance_locked(uint32_t now,
                                                int *out_distance_mm,
                                                uint32_t *out_sample_age_ms,
                                                bool *out_holding_last_good) {
  if (g_current_distance_mm < 0 || g_distance_update_ms == 0) {
    return false;
  }

  uint32_t last_good_age_ms = now - g_distance_update_ms;
  if (last_good_age_ms > DANEVI_DISTANCE_STALE_MS) {
    return false;
  }

  bool holding_last_good = false;
  if (danevi_zero_run_is_active_locked()) {
    uint32_t zero_run_age_ms = now - g_zero_run_start_ms;
    if (zero_run_age_ms > DANEVI_ZERO_HOLD_GRACE_MS) {
      return false;
    }
    holding_last_good = true;
  }

  if (out_distance_mm != NULL) {
    *out_distance_mm = g_current_distance_mm;
  }
  if (out_sample_age_ms != NULL) {
    *out_sample_age_ms = last_good_age_ms;
  }
  if (out_holding_last_good != NULL) {
    *out_holding_last_good = holding_last_good;
  }
  return true;
}

static void danevi_debug_init(void) {
  if (g_debug_mutex == NULL) {
    g_debug_mutex = xSemaphoreCreateMutex();
  }
}

static void danevi_store_debug_line(const char *line) {
  danevi_debug_init();
  if (g_debug_mutex == NULL || line == NULL) {
    return;
  }

  xSemaphoreTake(g_debug_mutex, portMAX_DELAY);
  strncpy(g_debug_lines[g_debug_next], line, DANEVI_DEBUG_LINE_MAX - 1);
  g_debug_lines[g_debug_next][DANEVI_DEBUG_LINE_MAX - 1] = '\0';
  g_debug_next = (g_debug_next + 1) % DANEVI_DEBUG_MAX_LINES;
  if (g_debug_count < DANEVI_DEBUG_MAX_LINES) {
    g_debug_count++;
  }
  xSemaphoreGive(g_debug_mutex);
}

static void danevi_format_frame_bytes(const uint8_t *frame, int length, char *dst,
                                      size_t dst_size) {
  if (dst == NULL || dst_size == 0) {
    return;
  }

  dst[0] = '\0';
  if (frame == NULL || length <= 0) {
    snprintf(dst, dst_size, "<none>");
    return;
  }

  for (int i = 0; i < length; i++) {
    char byte_str[8];
    snprintf(byte_str, sizeof(byte_str), i == 0 ? "%02X" : " %02X", frame[i]);
    strncat(dst, byte_str, dst_size - strlen(dst) - 1);
  }
}

static void danevi_log_valid_distance(const uint8_t *frame, int length,
                                      int distance_mm) {
  TickType_t now = xTaskGetTickCount();
  int distance_delta = distance_mm - g_last_logged_distance_mm;
  if (distance_delta < 0) {
    distance_delta = -distance_delta;
  }

  bool should_log = g_last_logged_distance_mm < 0 || distance_delta >= 10 ||
                    (now - g_last_distance_log_tick) >= pdMS_TO_TICKS(1000);
  if (!should_log) {
    return;
  }

  char frame_bytes[DANEVI_DEBUG_LINE_MAX / 2];
  danevi_format_frame_bytes(frame, length, frame_bytes, sizeof(frame_bytes));
  ESP_LOGI(TAG, "Hardwired sonar frame len=%d bytes=%s -> %d mm", length,
           frame_bytes, distance_mm);
  char debug_line[DANEVI_DEBUG_LINE_MAX];
  snprintf(debug_line, sizeof(debug_line), "[%lu ms] OK len=%d bytes=%s -> %d mm",
           (unsigned long)danevi_now_ms(), length, frame_bytes, distance_mm);
  danevi_store_debug_line(debug_line);
  db_sonar_log_log_hardwired_frame(distance_mm, length, frame_bytes);
  g_last_logged_distance_mm = distance_mm;
  g_last_distance_log_tick = now;
}

static void danevi_log_frame_issue(const char *reason, const uint8_t *frame,
                                   int length) {
  TickType_t now = xTaskGetTickCount();
  if ((now - g_last_frame_issue_log_tick) < pdMS_TO_TICKS(1000)) {
    return;
  }

  char frame_bytes[DANEVI_DEBUG_LINE_MAX / 2];
  danevi_format_frame_bytes(frame, length, frame_bytes, sizeof(frame_bytes));

  ESP_LOGW(TAG, "Hardwired sonar %s len=%d bytes=%s", reason, length,
           frame_bytes);
  char debug_line[DANEVI_DEBUG_LINE_MAX];
  snprintf(debug_line, sizeof(debug_line),
           "[%lu ms] ERR %s len=%d bytes=%s",
           (unsigned long)danevi_now_ms(), reason, length, frame_bytes);
  danevi_store_debug_line(debug_line);
  char persistent_detail[DANEVI_DEBUG_LINE_MAX];
  snprintf(persistent_detail, sizeof(persistent_detail), "len=%d bytes=%s",
           length, frame_bytes);
  db_sonar_log_log_hardwired_issue(reason, persistent_detail);
  g_last_frame_issue_log_tick = now;
}

static void danevi_log_no_response(void) {
  TickType_t now = xTaskGetTickCount();
  if ((now - g_last_no_response_log_tick) < pdMS_TO_TICKS(1000)) {
    return;
  }

  const char *line = "No hardwired sonar response within 100 ms";
  ESP_LOGW(TAG, "%s", line);
  char debug_line[DANEVI_DEBUG_LINE_MAX];
  snprintf(debug_line, sizeof(debug_line), "[%lu ms] WARN %s",
           (unsigned long)danevi_now_ms(), line);
  danevi_store_debug_line(debug_line);
  db_sonar_log_log_hardwired_issue("no_response",
                                   "timeout_ms=100 uart_read=0");
  g_last_no_response_log_tick = now;
}

void danevi_sonar_set_distance(int distance_mm) {
  if (g_distance_mutex != NULL) {
    uint32_t now = danevi_now_ms();
    xSemaphoreTake(g_distance_mutex, portMAX_DELAY);

    g_raw_distance_mm = distance_mm;
    g_raw_distance_update_ms = now;

    if (distance_mm > 0) {
      bool zero_run_was_active = danevi_zero_run_is_active_locked();
      g_current_distance_mm = distance_mm;
      g_distance_update_ms = now;
      g_consecutive_zero_frames = 0;
      g_zero_run_start_ms = 0;
      if (zero_run_was_active) {
        db_sonar_log_log_hardwired_zero_run_clear(distance_mm);
      }
    } else if (distance_mm == 0) {
      if (g_consecutive_zero_frames == 0) {
        g_zero_run_start_ms = now;
        db_sonar_log_log_hardwired_zero_run_start();
      }
      g_consecutive_zero_frames++;
    }

    xSemaphoreGive(g_distance_mutex);
  }
}

bool danevi_sonar_get_latest_distance(int *out_distance_mm) {
  if (out_distance_mm == NULL || g_distance_mutex == NULL) {
    return false;
  }

  bool has_distance = false;
  uint32_t now = danevi_now_ms();
  xSemaphoreTake(g_distance_mutex, portMAX_DELAY);
  has_distance = danevi_get_filtered_distance_locked(now, out_distance_mm, NULL,
                                                     NULL);
  xSemaphoreGive(g_distance_mutex);
  return has_distance;
}

bool danevi_sonar_get_snapshot(danevi_sonar_snapshot_t *snapshot) {
  if (snapshot == NULL) {
    return false;
  }

  memset(snapshot, 0, sizeof(*snapshot));
  snapshot->depth_mm = -1;
  snapshot->raw_depth_mm = -1;
  snapshot->last_good_depth_mm = -1;

  if (g_distance_mutex == NULL) {
    return false;
  }

  uint32_t now = danevi_now_ms();
  xSemaphoreTake(g_distance_mutex, portMAX_DELAY);

  if (danevi_get_filtered_distance_locked(now, &snapshot->depth_mm,
                                          &snapshot->sample_age_ms,
                                          &snapshot->zero_filter_holding_last_good)) {
    snapshot->has_distance = true;
  }

  if (g_raw_distance_mm >= 0 && g_raw_distance_update_ms != 0) {
    snapshot->has_raw_distance = true;
    snapshot->raw_depth_mm = g_raw_distance_mm;
    snapshot->raw_sample_age_ms = now - g_raw_distance_update_ms;
  }

  if (g_current_distance_mm >= 0 && g_distance_update_ms != 0) {
    snapshot->has_last_good_distance = true;
    snapshot->last_good_depth_mm = g_current_distance_mm;
    snapshot->last_good_sample_age_ms = now - g_distance_update_ms;
  }

  if (danevi_zero_run_is_active_locked()) {
    snapshot->zero_run_active = true;
    snapshot->zero_run_age_ms = now - g_zero_run_start_ms;
    snapshot->consecutive_zero_frames = g_consecutive_zero_frames;
  }
  xSemaphoreGive(g_distance_mutex);

  return snapshot->has_distance || snapshot->has_raw_distance ||
         snapshot->has_last_good_distance;
}

void danevi_sonar_get_debug_log(char *dst, size_t dst_size) {
  if (dst == NULL || dst_size == 0) {
    return;
  }

  danevi_debug_init();
  if (g_debug_mutex == NULL) {
    snprintf(dst, dst_size, "Hardwired debug buffer unavailable.");
    return;
  }

  uint32_t now = danevi_now_ms();
  xSemaphoreTake(g_debug_mutex, portMAX_DELAY);
  int filtered_distance_mm = -1;
  uint32_t filtered_age_ms = 0;
  bool holding_last_good = false;

  if (g_distance_mutex != NULL) {
    xSemaphoreTake(g_distance_mutex, portMAX_DELAY);
    danevi_get_filtered_distance_locked(now, &filtered_distance_mm,
                                        &filtered_age_ms,
                                        &holding_last_good);

    char raw_desc[48];
    char last_good_desc[48];
    const char *publish_state = "stale/no-data";
    if (filtered_distance_mm >= 0) {
      publish_state = holding_last_good ? "hold-last-good" : "live";
    } else if (danevi_zero_run_is_active_locked()) {
      publish_state = "suppressed-zero-run";
    }

    if (g_raw_distance_mm >= 0 && g_raw_distance_update_ms != 0) {
      snprintf(raw_desc, sizeof(raw_desc), "%d mm (%lu ms ago)",
               g_raw_distance_mm,
               (unsigned long)(now - g_raw_distance_update_ms));
    } else {
      snprintf(raw_desc, sizeof(raw_desc), "n/a");
    }

    if (g_current_distance_mm >= 0 && g_distance_update_ms != 0) {
      snprintf(last_good_desc, sizeof(last_good_desc), "%d mm (%lu ms ago)",
               g_current_distance_mm,
               (unsigned long)(now - g_distance_update_ms));
    } else {
      snprintf(last_good_desc, sizeof(last_good_desc), "n/a");
    }

    snprintf(dst, dst_size,
             "Filter: publish=%s raw=%s last_good=%s\n"
             "Zero run: active=%s count=%lu age=%lu ms grace=%d ms",
             publish_state, raw_desc, last_good_desc,
             danevi_zero_run_is_active_locked() ? "yes" : "no",
             (unsigned long)g_consecutive_zero_frames,
             (unsigned long)(danevi_zero_run_is_active_locked()
                                 ? (now - g_zero_run_start_ms)
                                 : 0),
             DANEVI_ZERO_HOLD_GRACE_MS);
    xSemaphoreGive(g_distance_mutex);
  } else {
    snprintf(dst, dst_size, "Hardwired debug buffer available, distance state unavailable.");
  }

  if (g_debug_count == 0) {
    strncat(dst, "\nNo hardwired sonar frames captured yet. This panel fills when "
                  "UART2 receives valid frames or frame errors.",
            dst_size - strlen(dst) - 1);
    xSemaphoreGive(g_debug_mutex);
    return;
  }

  size_t oldest_index =
      (g_debug_next + DANEVI_DEBUG_MAX_LINES - g_debug_count) %
      DANEVI_DEBUG_MAX_LINES;
  for (size_t i = 0; i < g_debug_count; i++) {
    size_t line_index = (oldest_index + i) % DANEVI_DEBUG_MAX_LINES;
    strncat(dst, "\n", dst_size - strlen(dst) - 1);
    strncat(dst, g_debug_lines[line_index], dst_size - strlen(dst) - 1);
  }
  xSemaphoreGive(g_debug_mutex);
}

static void danevi_sonar_task(void *arg) {
  uint8_t rx_buffer[BUF_SIZE];
  uint8_t trigger_cmd = DANEVI_TRIGGER_BYTE;

  ESP_LOGI(TAG, "Starting Hardwired Sonar Task on Core 1");

  while (1) {
    // Clear buffer before sending trigger
    uart_flush(SONAR_UART_NUM);

    // Trigger ping
    uart_write_bytes(SONAR_UART_NUM, (const char *)&trigger_cmd, 1);

    // Read the manufacturer-documented UART frame: FF Data_H Data_L SUM
    int length = uart_read_bytes(SONAR_UART_NUM, rx_buffer, DANEVI_FRAME_SIZE,
                                 pdMS_TO_TICKS(DANEVI_RESPONSE_TIMEOUT_MS));

    if (length == DANEVI_FRAME_SIZE && rx_buffer[0] == 0xFF) {
      uint8_t data_h = rx_buffer[1];
      uint8_t data_l = rx_buffer[2];
      uint8_t chk = rx_buffer[3];
      uint8_t sum = (uint8_t)((rx_buffer[0] + data_h + data_l) & 0xFF);

      if (sum == chk) {
        int distance = (data_h << 8) + data_l;
        danevi_sonar_set_distance(distance);
        danevi_log_valid_distance(rx_buffer, length, distance);
      } else {
        danevi_log_frame_issue("checksum mismatch", rx_buffer, length);
      }
    } else if (length > 0) {
      danevi_log_frame_issue("misaligned frame", rx_buffer, length);
      uart_flush(SONAR_UART_NUM); // Flush misaligned data
    } else {
      danevi_log_no_response();
    }

    // 100 ms is safely above the manufacturer minimum trigger interval.
    vTaskDelay(pdMS_TO_TICKS(DANEVI_TRIGGER_INTERVAL_MS));
  }
}

void danevi_sonar_init(int tx_pin, int rx_pin) {
  g_distance_mutex = xSemaphoreCreateMutex();
  danevi_debug_init();

  uart_config_t uart_config = {
      .baud_rate = SONAR_BAUD_RATE,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
  };

  ESP_ERROR_CHECK(
      uart_driver_install(SONAR_UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));
  ESP_ERROR_CHECK(uart_param_config(SONAR_UART_NUM, &uart_config));
  ESP_ERROR_CHECK(uart_set_pin(SONAR_UART_NUM, tx_pin, rx_pin,
                               UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

  xTaskCreatePinnedToCore(danevi_sonar_task, "danevi_sonar",
                          DANEVI_SONAR_TASK_STACK_SIZE, NULL, 5,
                          NULL, 1);

  ESP_LOGI(TAG, "Danevi Sonar initialized on UART2, TX:%d RX:%d", tx_pin,
           rx_pin);
  char debug_line[DANEVI_DEBUG_LINE_MAX];
  snprintf(debug_line, sizeof(debug_line),
           "[%lu ms] INFO Hardwired sonar UART2 init TX:%d RX:%d",
           (unsigned long)danevi_now_ms(), tx_pin, rx_pin);
  danevi_store_debug_line(debug_line);
}
