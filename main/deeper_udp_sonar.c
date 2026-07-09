#include "deeper_udp_sonar.h"

#include <errno.h>
#include <limits.h>
#include <lwip/inet.h>
#include <lwip/sockets.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "db_sonar_log.h"
#include "db_esp32_control.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#define TAG "DEEPER_UDP"
#define DEEPER_SONAR_IP "192.168.10.1"
#define DEEPER_SONAR_PORT 10110
#define DEEPER_REQUEST "$DEEP230,1*38\r\n"
#define DEEPER_TASK_STACK_SIZE 4096
#define DEEPER_TASK_PRIORITY 5
#define DEEPER_DISTANCE_STALE_MS 3000
#define DEEPER_REQUEST_PERIOD_MS 2000
#define DEEPER_RX_BUFFER_SIZE 512
#define DEEPER_LINE_BUFFER_SIZE 160

static SemaphoreHandle_t g_deeper_state_mutex = NULL;
static int g_deeper_distance_mm = -1;
static uint32_t g_deeper_distance_update_ms = 0;
static int g_deeper_temperature_c_tenths = INT_MIN;
static uint32_t g_deeper_temperature_update_ms = 0;
static int g_deeper_satellites = -1;
static bool g_deeper_gps_fix_valid = false;
static uint32_t g_deeper_satellites_update_ms = 0;
static double g_deeper_latitude_deg = 0.0;
static double g_deeper_longitude_deg = 0.0;
static uint32_t g_deeper_coordinates_update_ms = 0;
static bool g_deeper_task_started = false;
static uint32_t g_deeper_log_init_attempt_ms = 0;

static uint32_t deeper_now_ms(void) {
  return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

static void deeper_state_init(void) {
  if (g_deeper_state_mutex == NULL) {
    g_deeper_state_mutex = xSemaphoreCreateMutex();
  }
}

static void deeper_log_line(const char *prefix, const char *line) {
  if (line == NULL) {
    return;
  }

  char formatted_line[240];
  snprintf(formatted_line, sizeof(formatted_line), "[%lu ms] %s%s",
           (unsigned long)deeper_now_ms(), prefix == NULL ? "" : prefix, line);
  db_append_deeper_debug_line(formatted_line);
}

static void deeper_set_latest_distance(int distance_mm) {
  deeper_state_init();
  if (g_deeper_state_mutex == NULL) {
    return;
  }

  xSemaphoreTake(g_deeper_state_mutex, portMAX_DELAY);
  g_deeper_distance_mm = distance_mm;
  g_deeper_distance_update_ms = deeper_now_ms();
  xSemaphoreGive(g_deeper_state_mutex);
}

static void deeper_set_latest_temperature(int temperature_c_tenths) {
  deeper_state_init();
  if (g_deeper_state_mutex == NULL) {
    return;
  }

  xSemaphoreTake(g_deeper_state_mutex, portMAX_DELAY);
  g_deeper_temperature_c_tenths = temperature_c_tenths;
  g_deeper_temperature_update_ms = deeper_now_ms();
  xSemaphoreGive(g_deeper_state_mutex);
}

static void deeper_set_latest_satellites(int satellites, bool gps_fix_valid) {
  deeper_state_init();
  if (g_deeper_state_mutex == NULL) {
    return;
  }

  xSemaphoreTake(g_deeper_state_mutex, portMAX_DELAY);
  g_deeper_satellites = satellites;
  g_deeper_gps_fix_valid = gps_fix_valid;
  g_deeper_satellites_update_ms = deeper_now_ms();
  xSemaphoreGive(g_deeper_state_mutex);
}

static void deeper_set_latest_coordinates(double latitude_deg,
                                          double longitude_deg) {
  deeper_state_init();
  if (g_deeper_state_mutex == NULL) {
    return;
  }

  xSemaphoreTake(g_deeper_state_mutex, portMAX_DELAY);
  g_deeper_latitude_deg = latitude_deg;
  g_deeper_longitude_deg = longitude_deg;
  g_deeper_coordinates_update_ms = deeper_now_ms();
  xSemaphoreGive(g_deeper_state_mutex);
}

static void deeper_try_enable_persistent_log(
    const deeper_udp_snapshot_t *snapshot) {
  if (snapshot == NULL) {
    return;
  }
  (void)snapshot;

  if (db_sonar_log_is_available()) {
    return;
  }

  uint32_t now = deeper_now_ms();
  if (g_deeper_log_init_attempt_ms != 0 &&
      (now - g_deeper_log_init_attempt_ms) < 5000) {
    return;
  }

  g_deeper_log_init_attempt_ms = now;
  esp_err_t err = db_sonar_log_init();
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Deferred Deeper sonar log init failed (%s)",
             esp_err_to_name(err));
  }
}

static bool deeper_sentence_type_is(const char *sentence, const char *type) {
  return sentence != NULL && type != NULL && sentence[0] == '$' &&
         strlen(sentence) >= 6 && strncmp(&sentence[3], type, 3) == 0;
}

static bool deeper_parse_nmea_coordinate(const char *coordinate_field,
                                         char hemisphere, bool is_latitude,
                                         double *out_coordinate_deg) {
  if (coordinate_field == NULL || out_coordinate_deg == NULL ||
      coordinate_field[0] == '\0') {
    return false;
  }

  if ((is_latitude && hemisphere != 'N' && hemisphere != 'S') ||
      (!is_latitude && hemisphere != 'E' && hemisphere != 'W')) {
    return false;
  }

  char *parse_end = NULL;
  double raw_coordinate = strtod(coordinate_field, &parse_end);
  if (parse_end == coordinate_field) {
    return false;
  }

  int whole_degrees = (int)(raw_coordinate / 100.0);
  double minutes = raw_coordinate - (whole_degrees * 100.0);
  double coordinate_deg = whole_degrees + (minutes / 60.0);
  if (hemisphere == 'S' || hemisphere == 'W') {
    coordinate_deg = -coordinate_deg;
  }

  *out_coordinate_deg = coordinate_deg;
  return true;
}

static bool deeper_parse_dbt_sentence(const char *sentence,
                                      int *out_distance_mm) {
  if (sentence == NULL || out_distance_mm == NULL) {
    return false;
  }

  if (!deeper_sentence_type_is(sentence, "DBT")) {
    return false;
  }

  char copy[DEEPER_LINE_BUFFER_SIZE];
  strncpy(copy, sentence, sizeof(copy) - 1);
  copy[sizeof(copy) - 1] = '\0';

  char *checksum = strchr(copy, '*');
  if (checksum != NULL) {
    *checksum = '\0';
  }

  float feet = 0.0f;
  float meters = 0.0f;
  float fathoms = 0.0f;
  if (sscanf(copy + 6, ",%f,f,%f,M,%f,F", &feet, &meters, &fathoms) == 3) {
    *out_distance_mm = (int)(meters * 1000.0f + 0.5f);
    return true;
  }

  return false;
}

static bool deeper_parse_mtw_sentence(const char *sentence,
                                      int *out_temperature_c_tenths) {
  if (sentence == NULL || out_temperature_c_tenths == NULL ||
      !deeper_sentence_type_is(sentence, "MTW")) {
    return false;
  }

  char copy[DEEPER_LINE_BUFFER_SIZE];
  strncpy(copy, sentence, sizeof(copy) - 1);
  copy[sizeof(copy) - 1] = '\0';

  char *checksum = strchr(copy, '*');
  if (checksum != NULL) {
    *checksum = '\0';
  }

  float temperature_c = 0.0f;
  char unit = '\0';
  if (sscanf(copy + 6, ",%f,%c,", &temperature_c, &unit) >= 2 &&
      unit == 'C') {
    *out_temperature_c_tenths = (int)(temperature_c * 10.0f + 0.5f);
    return true;
  }

  return false;
}

static bool deeper_parse_gga_sentence(const char *sentence, int *out_satellites,
                                      bool *out_gps_fix_valid,
                                      double *out_latitude_deg,
                                      double *out_longitude_deg,
                                      bool *out_has_coordinates) {
  if (sentence == NULL || out_satellites == NULL || out_gps_fix_valid == NULL ||
      out_has_coordinates == NULL || !deeper_sentence_type_is(sentence, "GGA")) {
    return false;
  }

  char copy[DEEPER_LINE_BUFFER_SIZE];
  strncpy(copy, sentence, sizeof(copy) - 1);
  copy[sizeof(copy) - 1] = '\0';

  char *checksum = strchr(copy, '*');
  if (checksum != NULL) {
    *checksum = '\0';
  }

  int field_index = 0;
  int fix_quality = 0;
  int satellites = -1;
  char latitude_field[20] = {0};
  char longitude_field[20] = {0};
  char latitude_hemisphere = '\0';
  char longitude_hemisphere = '\0';
  char *cursor = copy;
  char *token = NULL;

  while ((token = strsep(&cursor, ",")) != NULL) {
    if (field_index == 2) {
      strncpy(latitude_field, token, sizeof(latitude_field) - 1);
    } else if (field_index == 3 && token[0] != '\0') {
      latitude_hemisphere = token[0];
    } else if (field_index == 4) {
      strncpy(longitude_field, token, sizeof(longitude_field) - 1);
    } else if (field_index == 5 && token[0] != '\0') {
      longitude_hemisphere = token[0];
    } else if (field_index == 6) {
      fix_quality = atoi(token);
    } else if (field_index == 7) {
      satellites = atoi(token);
      break;
    }
    field_index++;
  }

  if (satellites < 0) {
    return false;
  }

  *out_satellites = satellites;
  *out_gps_fix_valid = fix_quality > 0;
  *out_has_coordinates =
      fix_quality > 0 &&
      deeper_parse_nmea_coordinate(latitude_field, latitude_hemisphere, true,
                                   out_latitude_deg) &&
      deeper_parse_nmea_coordinate(longitude_field, longitude_hemisphere, false,
                                   out_longitude_deg);
  return true;
}

static bool deeper_parse_rmc_sentence(const char *sentence,
                                      double *out_latitude_deg,
                                      double *out_longitude_deg) {
  if (sentence == NULL || out_latitude_deg == NULL || out_longitude_deg == NULL ||
      !deeper_sentence_type_is(sentence, "RMC")) {
    return false;
  }

  char copy[DEEPER_LINE_BUFFER_SIZE];
  strncpy(copy, sentence, sizeof(copy) - 1);
  copy[sizeof(copy) - 1] = '\0';

  char *checksum = strchr(copy, '*');
  if (checksum != NULL) {
    *checksum = '\0';
  }

  int field_index = 0;
  char status = '\0';
  char latitude_field[20] = {0};
  char longitude_field[20] = {0};
  char latitude_hemisphere = '\0';
  char longitude_hemisphere = '\0';
  char *cursor = copy;
  char *token = NULL;

  while ((token = strsep(&cursor, ",")) != NULL) {
    if (field_index == 2 && token[0] != '\0') {
      status = token[0];
    } else if (field_index == 3) {
      strncpy(latitude_field, token, sizeof(latitude_field) - 1);
    } else if (field_index == 4 && token[0] != '\0') {
      latitude_hemisphere = token[0];
    } else if (field_index == 5) {
      strncpy(longitude_field, token, sizeof(longitude_field) - 1);
    } else if (field_index == 6 && token[0] != '\0') {
      longitude_hemisphere = token[0];
      break;
    }
    field_index++;
  }

  if (status != 'A') {
    return false;
  }

  return deeper_parse_nmea_coordinate(latitude_field, latitude_hemisphere, true,
                                      out_latitude_deg) &&
         deeper_parse_nmea_coordinate(longitude_field, longitude_hemisphere,
                                      false, out_longitude_deg);
}

static void deeper_process_sentence(const char *sentence) {
  if (sentence == NULL || sentence[0] == '\0') {
    return;
  }

  deeper_log_line("RX ", sentence);

  int distance_mm = -1;
  if (deeper_parse_dbt_sentence(sentence, &distance_mm)) {
    deeper_set_latest_distance(distance_mm);
    ESP_LOGI(TAG, "Parsed Deeper depth %d mm from %s", distance_mm, sentence);
  }

  int temperature_c_tenths = 0;
  if (deeper_parse_mtw_sentence(sentence, &temperature_c_tenths)) {
    deeper_set_latest_temperature(temperature_c_tenths);
  }

  int satellites = 0;
  bool gps_fix_valid = false;
  double latitude_deg = 0.0;
  double longitude_deg = 0.0;
  bool has_coordinates = false;
  if (deeper_parse_gga_sentence(sentence, &satellites, &gps_fix_valid,
                                &latitude_deg, &longitude_deg,
                                &has_coordinates)) {
    deeper_set_latest_satellites(satellites, gps_fix_valid);
    if (has_coordinates) {
      deeper_set_latest_coordinates(latitude_deg, longitude_deg);
    }
  }

  if (deeper_parse_rmc_sentence(sentence, &latitude_deg, &longitude_deg)) {
    deeper_set_latest_coordinates(latitude_deg, longitude_deg);
  }
}

static int deeper_open_socket(void) {
  int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
  if (sock < 0) {
    ESP_LOGE(TAG, "Failed to create Deeper UDP socket. errno=%d", errno);
    return -1;
  }

  int reuse_addr = 1;
  if (setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &reuse_addr,
                 sizeof(reuse_addr)) < 0) {
    ESP_LOGW(TAG, "SO_REUSEADDR failed on Deeper socket. errno=%d", errno);
  }

  struct sockaddr_in local_addr = {0};
  local_addr.sin_family = AF_INET;
  local_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  local_addr.sin_port = htons(DEEPER_SONAR_PORT);
  if (bind(sock, (struct sockaddr *)&local_addr, sizeof(local_addr)) < 0) {
    ESP_LOGE(TAG, "Failed to bind Deeper UDP socket to %d. errno=%d",
             DEEPER_SONAR_PORT, errno);
    close(sock);
    return -1;
  }

  struct timeval recv_timeout = {.tv_sec = 0, .tv_usec = 250000};
  if (setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &recv_timeout,
                 sizeof(recv_timeout)) < 0) {
    ESP_LOGW(TAG, "SO_RCVTIMEO failed on Deeper socket. errno=%d", errno);
  }

  return sock;
}

static void deeper_udp_sonar_task(void *arg) {
  db_clear_deeper_debug_log();
  deeper_log_line("INFO ", "Starting Deeper UDP/NMEA task");

  struct sockaddr_in deeper_addr = {0};
  deeper_addr.sin_family = AF_INET;
  deeper_addr.sin_port = htons(DEEPER_SONAR_PORT);
  deeper_addr.sin_addr.s_addr = inet_addr(DEEPER_SONAR_IP);

  int sock = -1;
  uint32_t last_request_ms = 0;
  uint32_t last_socket_error_ms = 0;
  uint8_t rx_buffer[DEEPER_RX_BUFFER_SIZE];
  char sentence_buffer[DEEPER_LINE_BUFFER_SIZE];
  size_t sentence_length = 0;

  while (1) {
    if (sock < 0) {
      sock = deeper_open_socket();
      if (sock < 0) {
        uint32_t now = deeper_now_ms();
        if ((now - last_socket_error_ms) >= 2000) {
          deeper_log_line("ERR ", "Failed to open UDP port 10110");
          last_socket_error_ms = now;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
        continue;
      }
      deeper_log_line("INFO ", "Bound local UDP port 10110");
      last_request_ms = 0;
      sentence_length = 0;
    }

    uint32_t now = deeper_now_ms();
    if ((now - last_request_ms) >= DEEPER_REQUEST_PERIOD_MS) {
      ssize_t sent = sendto(sock, DEEPER_REQUEST, strlen(DEEPER_REQUEST), 0,
                            (struct sockaddr *)&deeper_addr,
                            sizeof(deeper_addr));
      if (sent < 0) {
        ESP_LOGW(TAG, "Failed to send Deeper request. errno=%d", errno);
      } else if (last_request_ms == 0) {
        deeper_log_line("TX ", "$DEEP230,1*38");
      }
      last_request_ms = now;
    }

    struct sockaddr_in source_addr = {0};
    socklen_t source_addr_len = sizeof(source_addr);
    ssize_t recv_len =
        recvfrom(sock, rx_buffer, sizeof(rx_buffer), 0,
                 (struct sockaddr *)&source_addr, &source_addr_len);
    if (recv_len > 0) {
      for (ssize_t i = 0; i < recv_len; i++) {
        char ch = (char)rx_buffer[i];
        if (ch == '\r') {
          continue;
        }
        if (ch == '\n') {
          sentence_buffer[sentence_length] = '\0';
          deeper_process_sentence(sentence_buffer);
          sentence_length = 0;
          continue;
        }
        if (sentence_length < (sizeof(sentence_buffer) - 1)) {
          sentence_buffer[sentence_length++] = ch;
        } else {
          sentence_length = 0;
        }
      }
      continue;
    }

    if (recv_len < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
      ESP_LOGW(TAG, "Deeper UDP recvfrom failed. errno=%d", errno);
      deeper_log_line("ERR ", "UDP recvfrom failed, reopening socket");
      close(sock);
      sock = -1;
      sentence_length = 0;
      vTaskDelay(pdMS_TO_TICKS(500));
    }
  }
}

void deeper_udp_sonar_start(void) {
  if (g_deeper_task_started) {
    return;
  }

  deeper_state_init();
  g_deeper_task_started = true;
  xTaskCreatePinnedToCore(deeper_udp_sonar_task, "deeper_udp_sonar",
                          DEEPER_TASK_STACK_SIZE, NULL, DEEPER_TASK_PRIORITY,
                          NULL, 0);
}

bool deeper_udp_sonar_get_latest_distance(int *out_distance_mm) {
  if (out_distance_mm == NULL) {
    return false;
  }

  deeper_state_init();
  if (g_deeper_state_mutex == NULL) {
    return false;
  }

  bool has_fresh_distance = false;
  xSemaphoreTake(g_deeper_state_mutex, portMAX_DELAY);
  if (g_deeper_distance_update_ms != 0 &&
      (deeper_now_ms() - g_deeper_distance_update_ms) <=
          DEEPER_DISTANCE_STALE_MS) {
    *out_distance_mm = g_deeper_distance_mm;
    has_fresh_distance = true;
  }
  xSemaphoreGive(g_deeper_state_mutex);

  return has_fresh_distance;
}

bool deeper_udp_sonar_get_snapshot(deeper_udp_snapshot_t *snapshot) {
  if (snapshot == NULL) {
    return false;
  }

  memset(snapshot, 0, sizeof(*snapshot));
  snapshot->depth_mm = -1;
  snapshot->temperature_c_tenths = INT_MIN;
  snapshot->satellites = -1;

  deeper_state_init();
  if (g_deeper_state_mutex == NULL) {
    return false;
  }

  uint32_t now = deeper_now_ms();
  uint32_t newest_update_ms = 0;

  xSemaphoreTake(g_deeper_state_mutex, portMAX_DELAY);

  if (g_deeper_distance_update_ms != 0 &&
      (now - g_deeper_distance_update_ms) <= DEEPER_DISTANCE_STALE_MS) {
    snapshot->has_depth = true;
    snapshot->depth_mm = g_deeper_distance_mm;
    newest_update_ms = g_deeper_distance_update_ms;
  }

  if (g_deeper_temperature_update_ms != 0 &&
      (now - g_deeper_temperature_update_ms) <= DEEPER_DISTANCE_STALE_MS) {
    snapshot->has_temperature = true;
    snapshot->temperature_c_tenths = g_deeper_temperature_c_tenths;
    if (g_deeper_temperature_update_ms > newest_update_ms) {
      newest_update_ms = g_deeper_temperature_update_ms;
    }
  }

  if (g_deeper_satellites_update_ms != 0 &&
      (now - g_deeper_satellites_update_ms) <= DEEPER_DISTANCE_STALE_MS) {
    snapshot->has_satellites = true;
    snapshot->satellites = g_deeper_satellites;
    snapshot->gps_fix_valid = g_deeper_gps_fix_valid;
    if (g_deeper_satellites_update_ms > newest_update_ms) {
      newest_update_ms = g_deeper_satellites_update_ms;
    }
  }

  if (g_deeper_coordinates_update_ms != 0 &&
      (now - g_deeper_coordinates_update_ms) <= DEEPER_DISTANCE_STALE_MS) {
    snapshot->has_coordinates = true;
    snapshot->latitude_deg = g_deeper_latitude_deg;
    snapshot->longitude_deg = g_deeper_longitude_deg;
    if (g_deeper_coordinates_update_ms > newest_update_ms) {
      newest_update_ms = g_deeper_coordinates_update_ms;
    }
  }

  xSemaphoreGive(g_deeper_state_mutex);

  if (newest_update_ms != 0) {
    snapshot->newest_sample_age_ms = now - newest_update_ms;
  }

  bool has_snapshot = snapshot->has_depth || snapshot->has_temperature ||
                      snapshot->has_satellites || snapshot->has_coordinates;
  if (has_snapshot) {
    deeper_try_enable_persistent_log(snapshot);
    db_sonar_log_maybe_log_deeper_track(snapshot);
  }

  return has_snapshot;
}
