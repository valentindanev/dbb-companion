#include "danevi_sonar.h"

#include "driver/gpio.h"
#include "driver/rmt_rx.h"
#include "driver/uart.h"
#include "db_sonar_log.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sonar_driver.h"
#include <stdio.h>
#include <string.h>


#define SONAR_UART_NUM UART_NUM_2
#define BUF_SIZE 256

/* One microsecond per RMT tick. That is 0.74 mm of water per tick at 1480 m/s,
 * which is already finer than the 4.25 mm grid the UART path is stuck with, and
 * it keeps the 15-bit idle threshold worth 32.7 ms - far longer than any echo
 * this transducer can produce. Raising it would buy resolution we cannot use
 * and would shrink the idle window we rely on to end a no-echo cycle. */
#define DANEVI_PULSE_RESOLUTION_HZ 1000000U
/* Widths below this are ringing on the transducer, not an echo. */
#define DANEVI_PULSE_GLITCH_NS 1000U
/* An echo is a single high pulse, so two symbols is already generous. The RMT
 * hardware requires an even block size and 64 is its minimum on the S3. */
#define DANEVI_PULSE_SYMBOLS 64
#define DANEVI_SONAR_TASK_STACK_SIZE 4096
#define DANEVI_DEBUG_MAX_LINES 8
#define DANEVI_DEBUG_LINE_MAX 160

/* Which transducer this boot is talking to. Selected by `ss_type` and handed
 * in by main.c; everything below is model-agnostic. Only one model ever runs
 * per boot - the two sensors share UART2. */
static const sonar_driver_t *g_driver = NULL;

/* Never returns NULL: before init, and if a bad model id somehow reaches us,
 * fall back to the original transducer so behaviour matches the pre-driver
 * firmware rather than silently doing nothing. */
static const sonar_driver_t *danevi_driver(void) {
  return g_driver != NULL ? g_driver : &sonar_driver_dyp_l041mtw;
}

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
  if (last_good_age_ms > danevi_driver()->distance_stale_ms) {
    return false;
  }

  bool holding_last_good = false;
  if (danevi_zero_run_is_active_locked()) {
    uint32_t zero_run_age_ms = now - g_zero_run_start_ms;
    if (zero_run_age_ms > danevi_driver()->zero_hold_grace_ms) {
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

/* `frame`/`length` describe a UART reply; a PULSE model passes frame = NULL and
 * the captured echo width instead. Both end up as one `detail` string so the
 * debug ring, SYSTEM.LOG and the rate limiter stay common to every model. */
static void danevi_log_valid_distance(const uint8_t *frame, int length,
                                      uint32_t echo_us, int distance_mm) {
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

  char detail[DANEVI_DEBUG_LINE_MAX / 2];
  if (frame != NULL) {
    char frame_bytes[DANEVI_DEBUG_LINE_MAX / 4];
    danevi_format_frame_bytes(frame, length, frame_bytes, sizeof(frame_bytes));
    snprintf(detail, sizeof(detail), "len=%d bytes=%s", length, frame_bytes);
  } else {
    snprintf(detail, sizeof(detail), "echo=%lu us", (unsigned long)echo_us);
  }
  ESP_LOGI(TAG, "Hardwired sonar reading %s -> %d mm", detail, distance_mm);
  char debug_line[DANEVI_DEBUG_LINE_MAX];
  snprintf(debug_line, sizeof(debug_line), "[%lu ms] OK %s -> %d mm",
           (unsigned long)danevi_now_ms(), detail, distance_mm);
  danevi_store_debug_line(debug_line);
  db_sonar_log_log_hardwired_frame(distance_mm, length, detail);
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

  const sonar_driver_t *driver = danevi_driver();
  char line[80];
  snprintf(line, sizeof(line), "No hardwired sonar response within %lu ms",
           (unsigned long)driver->response_timeout_ms);
  ESP_LOGW(TAG, "%s", line);
  char debug_line[DANEVI_DEBUG_LINE_MAX];
  snprintf(debug_line, sizeof(debug_line), "[%lu ms] WARN %s",
           (unsigned long)danevi_now_ms(), line);
  danevi_store_debug_line(debug_line);
  char timeout_detail[48];
  snprintf(timeout_detail, sizeof(timeout_detail), "timeout_ms=%lu uart_read=0",
           (unsigned long)driver->response_timeout_ms);
  db_sonar_log_log_hardwired_issue("no_response", timeout_detail);
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
             (int)danevi_driver()->zero_hold_grace_ms);
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

/* ---------------------------------------------------------------------------
 * PULSE link. Drive a short high pulse on the trigger pin, then measure how
 * long the echo pin stays high. The measurement is done by RMT, not by us: the
 * peripheral timestamps both edges in hardware, so nothing in this file - task
 * priority, Wi-Fi interrupts, a busy core - can move the number. That is the
 * whole reason the 02-09 objection to pulse mode ("100 us of jitter is 7.4 cm")
 * does not apply. A software timer would deserve that objection; this does not.
 * ------------------------------------------------------------------------- */

typedef enum {
  SONAR_READ_OK,       /* a reading, including a legitimate 0 = no echo */
  SONAR_READ_REJECTED, /* something arrived but it was not usable */
  SONAR_READ_TIMEOUT,  /* nothing arrived at all */
} sonar_read_t;

static rmt_channel_handle_t g_echo_channel = NULL;
static QueueHandle_t g_echo_queue = NULL;
static rmt_symbol_word_t g_echo_symbols[DANEVI_PULSE_SYMBOLS];
static int g_trigger_pin = -1;

static bool danevi_echo_done(rmt_channel_handle_t channel,
                             const rmt_rx_done_event_data_t *edata,
                             void *user_ctx) {
  BaseType_t higher_woken = pdFALSE;
  xQueueSendFromISR((QueueHandle_t)user_ctx, edata, &higher_woken);
  return higher_woken == pdTRUE;
}

static bool danevi_pulse_init(int trigger_pin, int echo_pin) {
  gpio_config_t trigger_cfg = {
      .pin_bit_mask = 1ULL << (uint32_t)trigger_pin,
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  if (gpio_config(&trigger_cfg) != ESP_OK) {
    ESP_LOGE(TAG, "Sonar trigger GPIO %d could not be configured", trigger_pin);
    return false;
  }
  gpio_set_level((gpio_num_t)trigger_pin, 0);

  g_echo_queue = xQueueCreate(1, sizeof(rmt_rx_done_event_data_t));
  if (g_echo_queue == NULL) {
    return false;
  }

  rmt_rx_channel_config_t echo_cfg = {
      .gpio_num = (gpio_num_t)echo_pin,
      .clk_src = RMT_CLK_SRC_DEFAULT,
      .resolution_hz = DANEVI_PULSE_RESOLUTION_HZ,
      .mem_block_symbols = DANEVI_PULSE_SYMBOLS,
  };
  if (rmt_new_rx_channel(&echo_cfg, &g_echo_channel) != ESP_OK) {
    ESP_LOGE(TAG, "Sonar echo RMT channel on GPIO %d failed", echo_pin);
    return false;
  }
  rmt_rx_event_callbacks_t callbacks = {.on_recv_done = danevi_echo_done};
  if (rmt_rx_register_event_callbacks(g_echo_channel, &callbacks,
                                      g_echo_queue) != ESP_OK) {
    return false;
  }
  if (rmt_enable(g_echo_channel) != ESP_OK) {
    return false;
  }
  g_trigger_pin = trigger_pin;
  return true;
}

static sonar_read_t danevi_read_pulse(const sonar_driver_t *driver,
                                      uint32_t *out_echo_us) {
  *out_echo_us = 0;
  if (g_echo_channel == NULL || g_trigger_pin < 0) {
    return SONAR_READ_TIMEOUT;
  }

  rmt_receive_config_t receive_cfg = {
      .signal_range_min_ns = DANEVI_PULSE_GLITCH_NS,
      .signal_range_max_ns = driver->echo_max_us * 1000U,
  };
  /* Arm BEFORE the ping. In shallow water the echo can be back within about a
   * millisecond, and arming afterwards would race it. */
  xQueueReset(g_echo_queue);
  if (rmt_receive(g_echo_channel, g_echo_symbols, sizeof(g_echo_symbols),
                  &receive_cfg) != ESP_OK) {
    return SONAR_READ_TIMEOUT;
  }

  gpio_set_level((gpio_num_t)g_trigger_pin, 1);
  esp_rom_delay_us(driver->trigger_pulse_us);
  gpio_set_level((gpio_num_t)g_trigger_pin, 0);

  rmt_rx_done_event_data_t event;
  if (xQueueReceive(g_echo_queue, &event,
                    pdMS_TO_TICKS(driver->response_timeout_ms)) != pdTRUE) {
    /* The echo pin never moved at all. Tear the armed receive down so the next
     * cycle cannot complete on this ping's stale arming and report a width that
     * belongs to a trigger we already gave up on. */
    rmt_disable(g_echo_channel);
    rmt_enable(g_echo_channel);
    return SONAR_READ_TIMEOUT;
  }

  /* The echo is the first HIGH level in the capture; anything before it is the
   * line sitting idle low, which RMT records as a leading low symbol. */
  uint32_t width_us = 0;
  for (size_t i = 0; i < event.num_symbols && width_us == 0; i++) {
    if (event.received_symbols[i].level0 != 0 &&
        event.received_symbols[i].duration0 > 0) {
      width_us = event.received_symbols[i].duration0;
    } else if (event.received_symbols[i].level1 != 0 &&
               event.received_symbols[i].duration1 > 0) {
      width_us = event.received_symbols[i].duration1;
    }
  }

  /* A width at the ceiling is the RMT idle threshold cutting a pulse short, not
   * a measurement. HC-SR04-family modules answer "nothing out there" with a very
   * long pulse, so without this test a no-echo cycle would be published as the
   * deepest reading the window allows instead of as 0. */
  if (width_us + 4U >= driver->echo_max_us) {
    width_us = 0;
  }

  *out_echo_us = width_us;
  return SONAR_READ_OK;
}

static void danevi_sonar_task(void *arg) {
  uint8_t rx_buffer[BUF_SIZE];
  const sonar_driver_t *driver = danevi_driver();
  uint8_t trigger_cmd = driver->trigger_byte;
  int frame_size = (int)driver->frame_size;

  ESP_LOGI(TAG, "Starting Hardwired Sonar Task on Core 1 (model %s)",
           driver->name);

  const bool is_pulse = driver->link == SONAR_LINK_PULSE;

  while (1) {
    db_sonar_log_note_hardwired_poll();

    sonar_read_t outcome;
    int distance = 0;
    int length = 0;
    uint32_t echo_us = 0;

    if (is_pulse) {
      outcome = danevi_read_pulse(driver, &echo_us);
      if (outcome == SONAR_READ_OK) {
        /* An echo width of 0 is a successful measurement of "no echo" and has
         * to stay a real 0 all the way to the zero-run filter, exactly as a
         * 0 mm UART frame does, or the two links stop being comparable. */
        distance = driver->scale_us != NULL ? driver->scale_us(echo_us)
                                            : (int)echo_us;
      }
    } else {
      // Clear buffer before sending trigger
      uart_flush(SONAR_UART_NUM);

      // Trigger ping. trigger_byte == 0 means the model free-runs and wants no
      // prompt; we just listen for the next frame.
      if (trigger_cmd != 0) {
        uart_write_bytes(SONAR_UART_NUM, (const char *)&trigger_cmd, 1);
      }

      length = uart_read_bytes(SONAR_UART_NUM, rx_buffer, frame_size,
                               pdMS_TO_TICKS(driver->response_timeout_ms));

      int raw_mm = -1;
      if (length == frame_size && driver->parse(rx_buffer, length, &raw_mm)) {
        /* A valid frame reporting 0 mm is a successful decode of "no echo" and
         * must reach the zero-run filter as a real 0 - not as a bad frame. */
        distance = raw_mm;
        if (driver->scale_mm != NULL) {
          distance = driver->scale_mm(raw_mm);
        }
        outcome = SONAR_READ_OK;
      } else if (length > 0) {
        outcome = SONAR_READ_REJECTED;
      } else {
        outcome = SONAR_READ_TIMEOUT;
      }
    }

    if (outcome == SONAR_READ_OK) {
      db_sonar_log_note_hardwired_frame(distance);
      danevi_sonar_set_distance(distance);
      danevi_log_valid_distance(is_pulse ? NULL : rx_buffer, length, echo_us,
                                distance);
    } else if (outcome == SONAR_READ_REJECTED) {
      db_sonar_log_note_hardwired_bad_frame();
      /* The model's parse() is the authority on validity but cannot say WHY it
       * refused, so a full-length reject is reported as "frame rejected"
       * (bad header or bad checksum) rather than guessing "checksum mismatch"
       * and being wrong half the time. Short/long reads stay "misaligned". */
      danevi_log_frame_issue(length == frame_size ? "frame rejected"
                                                  : "misaligned frame",
                             rx_buffer, length);
      uart_flush(SONAR_UART_NUM); // Flush misaligned data
    } else {
      db_sonar_log_note_hardwired_timeout();
      danevi_log_no_response();
    }

    vTaskDelay(pdMS_TO_TICKS(driver->trigger_interval_ms));
  }
}

void danevi_sonar_init(const sonar_driver_t *driver, int tx_pin, int rx_pin) {
  /* A descriptor missing the function its own link depends on would null-deref
   * on the first poll, and this struct is the extension point for future
   * transducers - so refuse it here and fall back rather than crash-loop the
   * boat on someone's next model. The required function differs by link: a UART
   * model must be able to parse a frame, a PULSE model must be able to convert
   * an echo width, and neither check covers the other. */
  if (driver != NULL) {
    const bool incomplete = driver->link == SONAR_LINK_PULSE
                                ? driver->scale_us == NULL
                                : driver->parse == NULL;
    if (incomplete) {
      ESP_LOGE(TAG, "Sonar model '%s' is missing its %s handler; falling back to %s",
               driver->name != NULL ? driver->name : "?",
               driver->link == SONAR_LINK_PULSE ? "scale_us" : "parse",
               sonar_driver_dyp_l041mtw.name);
      driver = NULL;
    }
  }
  g_driver = driver != NULL ? driver : &sonar_driver_dyp_l041mtw;
  g_distance_mutex = xSemaphoreCreateMutex();
  danevi_debug_init();

  /* PULSE takes no UART at all. That is the second reason this link exists:
   * UART2 is the only thing standing between the receiver and a direct CRSF
   * tap, so a sonar that does not claim it hands that peripheral back. */
  if (g_driver->link == SONAR_LINK_PULSE) {
    if (!danevi_pulse_init(tx_pin, rx_pin)) {
      ESP_LOGE(TAG, "Pulse sonar init failed on TRIG:%d ECHO:%d", tx_pin,
               rx_pin);
      char failed_line[DANEVI_DEBUG_LINE_MAX];
      snprintf(failed_line, sizeof(failed_line),
               "[%lu ms] ERR %s pulse init failed TRIG:%d ECHO:%d",
               (unsigned long)danevi_now_ms(), g_driver->display_name, tx_pin,
               rx_pin);
      danevi_store_debug_line(failed_line);
      return;
    }
  } else {
    uart_config_t uart_config = {
        .baud_rate = (int)g_driver->baud_rate,
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
  }

  xTaskCreatePinnedToCore(danevi_sonar_task, "danevi_sonar",
                          DANEVI_SONAR_TASK_STACK_SIZE, NULL, 5,
                          NULL, 1);

  char debug_line[DANEVI_DEBUG_LINE_MAX];
  if (g_driver->link == SONAR_LINK_PULSE) {
    ESP_LOGI(TAG,
             "Danevi Sonar initialized on RMT, model %s, TRIG:%d ECHO:%d, "
             "%lu us trigger, %lu us echo window",
             g_driver->name, tx_pin, rx_pin,
             (unsigned long)g_driver->trigger_pulse_us,
             (unsigned long)g_driver->echo_max_us);
    snprintf(debug_line, sizeof(debug_line),
             "[%lu ms] INFO %s RMT init TRIG:%d ECHO:%d trig=%lu us win=%lu us",
             (unsigned long)danevi_now_ms(), g_driver->display_name, tx_pin,
             rx_pin, (unsigned long)g_driver->trigger_pulse_us,
             (unsigned long)g_driver->echo_max_us);
  } else {
    ESP_LOGI(TAG,
             "Danevi Sonar initialized on UART2, model %s @%lu baud, TX:%d RX:%d",
             g_driver->name, (unsigned long)g_driver->baud_rate, tx_pin, rx_pin);
    snprintf(debug_line, sizeof(debug_line),
             "[%lu ms] INFO %s UART2 init @%lu baud TX:%d RX:%d",
             (unsigned long)danevi_now_ms(), g_driver->display_name,
             (unsigned long)g_driver->baud_rate, tx_pin, rx_pin);
  }
  danevi_store_debug_line(debug_line);
}
