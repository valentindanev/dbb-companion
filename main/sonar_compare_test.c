#include "sonar_compare_test.h"

#include "sdkconfig.h"
#include "sonar_compare_core.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cJSON.h"
#include "db_mavlink_msgs.h"
#include "db_parameters.h"
#include "db_sonar_log.h"
#include "dbb_brain.h"
#include "deeper_udp_sonar.h"
#include "esp_app_desc.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "globals.h"

static const char *TAG = "SONAR_COMPARE";
static sonar_compare_core_t s_core;
static SemaphoreHandle_t s_mutex;
static uint32_t s_last_aj_echo_us;

static uint32_t now_ms(void) {
  return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

static bool compare_compiled(void) {
#if CONFIG_DB_SONAR_COMPARE_TEST
  return true;
#else
  return false;
#endif
}

static void ensure_mutex(void) {
  if (s_mutex == NULL) s_mutex = xSemaphoreCreateMutex();
}

void sonar_compare_test_init(bool deeper_connected, int output_source) {
  ensure_mutex();
  if (s_mutex == NULL) return;
  xSemaphoreTake(s_mutex, portMAX_DELAY);
  sonar_compare_core_init(&s_core, compare_compiled(), deeper_connected,
                          output_source);
  xSemaphoreGive(s_mutex);
  if (compare_compiled()) {
    ESP_LOGW(TAG, "SONAR COMPARE TEST - NOT FOR NORMAL USE; output source=%d",
             output_source);
    db_sonar_log_log_system_event("sonar_compare_test",
                                  "NOT_FOR_NORMAL_USE no_MAVLink_output");
  }
}

void sonar_compare_test_note_aj(bool valid, int raw_depth_mm, uint32_t echo_us,
                                bool timeout,
                                const danevi_sonar_snapshot_t *filtered) {
  if (!compare_compiled()) return;
  ensure_mutex();
  if (s_mutex == NULL) return;
  bool recording = false;
  xSemaphoreTake(s_mutex, portMAX_DELAY);
  sonar_compare_core_note_aj(&s_core, now_ms(), valid, raw_depth_mm, timeout);
  s_last_aj_echo_us = echo_us;
  recording = s_core.state == SONAR_COMPARE_RECORDING;
  xSemaphoreGive(s_mutex);
  if (!recording) return;

  db_mavlink_gps_state_t gps = {0};
  db_mavlink_get_gps_state(&gps);
  db_sonar_log_log_compare(
      "source=aj outcome=%s echo_us=%lu raw_mm=%d filtered_valid=%d "
      "filtered_mm=%d holding=%d water_mps=%d fc_lat_e7=%ld fc_lon_e7=%ld "
      "fc_gps_age_ms=%lld",
      timeout ? "timeout" : !valid ? "rejected" : raw_depth_mm == 0 ? "zero" : "valid",
      (unsigned long)echo_us, raw_depth_mm,
      filtered != NULL && filtered->has_distance ? 1 : 0,
      filtered != NULL && filtered->has_distance ? filtered->depth_mm : -1,
      filtered != NULL && filtered->zero_filter_holding_last_good ? 1 : 0,
      (int)DB_PARAM_WATER_MPS,
      (long)(gps.valid ? gps.latitude_e7 : 0),
      (long)(gps.valid ? gps.longitude_e7 : 0),
      (long long)gps.age_ms);
}

void sonar_compare_test_note_deeper(int depth_mm) {
  if (!compare_compiled()) return;
  ensure_mutex();
  if (s_mutex == NULL) return;
  const uint32_t at_ms = now_ms();
  bool recording = false;
  xSemaphoreTake(s_mutex, portMAX_DELAY);
  sonar_compare_core_note_deeper(&s_core, at_ms, depth_mm >= 0, depth_mm);
  sonar_compare_pair_t pair = sonar_compare_core_pair(&s_core, at_ms);
  if (pair.quality == SONAR_COMPARE_PAIR_PAIRED) s_core.valid_pairs++;
  recording = s_core.state == SONAR_COMPARE_RECORDING;
  xSemaphoreGive(s_mutex);
  if (!recording) return;

  deeper_udp_snapshot_t deeper = {0};
  deeper_udp_sonar_get_snapshot(&deeper);
  db_mavlink_gps_state_t gps = {0};
  db_mavlink_get_gps_state(&gps);
  db_sonar_log_log_compare(
      "source=deeper outcome=%s raw_mm=%d deeper_lat=%.7f deeper_lon=%.7f "
      "deeper_fix=%d deeper_sats=%d fc_lat_e7=%ld fc_lon_e7=%ld fc_gps_age_ms=%lld",
      depth_mm >= 0 ? "valid" : "invalid", depth_mm,
      deeper.has_coordinates ? deeper.latitude_deg : 0.0,
      deeper.has_coordinates ? deeper.longitude_deg : 0.0,
      deeper.gps_fix_valid ? 1 : 0,
      deeper.has_satellites ? deeper.satellites : -1,
      (long)(gps.valid ? gps.latitude_e7 : 0),
      (long)(gps.valid ? gps.longitude_e7 : 0),
      (long long)gps.age_ms);
}

static esp_err_t send_json(httpd_req_t *req, cJSON *root) {
  char *text = cJSON_PrintUnformatted(root);
  if (text == NULL) return ESP_ERR_NO_MEM;
  httpd_resp_set_type(req, "application/json");
  esp_err_t err = httpd_resp_sendstr(req, text);
  free(text);
  return err;
}

static esp_err_t compare_status_handler(httpd_req_t *req) {
  ensure_mutex();
  sonar_compare_core_t core = {0};
  sonar_compare_pair_t pair = {0};
  uint32_t aj_echo_us = 0;
  if (s_mutex != NULL) {
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    core = s_core;
    pair = sonar_compare_core_pair(&s_core, now_ms());
    aj_echo_us = s_last_aj_echo_us;
    xSemaphoreGive(s_mutex);
  }
  danevi_sonar_snapshot_t aj = {0};
  deeper_udp_snapshot_t deeper = {0};
  db_mavlink_gps_state_t gps = {0};
  (void)danevi_sonar_get_snapshot(&aj);
  deeper_udp_sonar_get_snapshot(&deeper);
  db_mavlink_get_gps_state(&gps);

  cJSON *root = cJSON_CreateObject();
  if (root == NULL) return ESP_ERR_NO_MEM;
  cJSON_AddBoolToObject(root, "compiled", core.compiled);
  cJSON_AddStringToObject(root, "banner",
                          core.compiled ? "SONAR COMPARE TEST - NOT FOR NORMAL USE" : "");
  cJSON_AddStringToObject(root, "state", sonar_compare_state_name(core.state));
  cJSON_AddStringToObject(root, "reason", core.reason);
  cJSON_AddNumberToObject(root, "session_id", core.session_id);
  cJSON_AddNumberToObject(root, "output_source", DB_ACTIVE_SONAR_SOURCE);
  cJSON_AddBoolToObject(root, "output_authority_unchanged",
                        sonar_compare_output_authority_ok(&core, DB_ACTIVE_SONAR_SOURCE));
  cJSON_AddNumberToObject(root, "aj_raw_mm", aj.has_raw_distance ? aj.raw_depth_mm : -1);
  cJSON_AddNumberToObject(root, "aj_raw_age_ms", aj.raw_sample_age_ms);
  cJSON_AddNumberToObject(root, "aj_filtered_mm", aj.has_distance ? aj.depth_mm : -1);
  cJSON_AddNumberToObject(root, "aj_filtered_age_ms", aj.sample_age_ms);
  cJSON_AddNumberToObject(root, "aj_echo_us", aj_echo_us);
  cJSON_AddNumberToObject(root, "deeper_mm", deeper.has_depth ? deeper.depth_mm : -1);
  cJSON_AddNumberToObject(root, "deeper_age_ms", deeper.newest_sample_age_ms);
  cJSON_AddStringToObject(root, "pair_quality",
                          sonar_compare_pair_quality_name(pair.quality));
  cJSON_AddNumberToObject(root, "pair_separation_ms", pair.separation_ms);
  cJSON_AddNumberToObject(root, "difference_mm", pair.difference_mm);
  cJSON_AddNumberToObject(root, "difference_percent", pair.difference_percent);
  cJSON_AddBoolToObject(root, "agreement",
                        pair.quality == SONAR_COMPARE_PAIR_PAIRED && pair.agreement);
  cJSON_AddNumberToObject(root, "aj_polls", core.aj_polls);
  cJSON_AddNumberToObject(root, "aj_zeros", core.aj_zeros);
  cJSON_AddNumberToObject(root, "aj_timeouts", core.aj_timeouts);
  cJSON_AddNumberToObject(root, "deeper_samples", core.deeper_samples);
  cJSON_AddNumberToObject(root, "valid_pairs", core.valid_pairs);
  cJSON_AddNumberToObject(root, "water_mps", DB_PARAM_WATER_MPS);
  cJSON_AddNumberToObject(root, "fc_latitude", gps.valid ? gps.latitude_e7 / 10000000.0 : 0.0);
  cJSON_AddNumberToObject(root, "fc_longitude", gps.valid ? gps.longitude_e7 / 10000000.0 : 0.0);
  cJSON_AddBoolToObject(root, "deeper_has_coordinates", deeper.has_coordinates);
  cJSON_AddNumberToObject(root, "deeper_latitude", deeper.latitude_deg);
  cJSON_AddNumberToObject(root, "deeper_longitude", deeper.longitude_deg);
  esp_err_t err = send_json(req, root);
  cJSON_Delete(root);
  return err;
}

static void safe_token(char *dst, size_t size, const cJSON *item,
                       const char *fallback) {
  snprintf(dst, size, "%s", cJSON_IsString(item) ? item->valuestring : fallback);
  for (size_t i = 0; dst[i] != '\0'; ++i) {
    if (dst[i] <= ' ' || dst[i] == '=') dst[i] = '_';
  }
}

static esp_err_t compare_start_handler(httpd_req_t *req) {
  char body[512] = {0};
  int received = 0;
  while (received < req->content_len && received < (int)sizeof(body) - 1) {
    size_t room = sizeof(body) - 1U - (size_t)received;
    size_t wanted = (size_t)(req->content_len - received);
    if (wanted > room) wanted = room;
    int n = httpd_req_recv(req, body + received,
                           wanted);
    if (n <= 0) break;
    received += n;
  }
  cJSON *json = received > 0 ? cJSON_Parse(body) : NULL;
  cJSON *preflight = json ? cJSON_GetObjectItem(json, "preflight_complete") : NULL;
  bool preflight_ok = cJSON_IsTrue(preflight);

  danevi_sonar_snapshot_t aj = {0};
  deeper_udp_snapshot_t deeper = {0};
  (void)danevi_sonar_get_snapshot(&aj);
  deeper_udp_sonar_get_snapshot(&deeper);
  bool aj_live = aj.has_raw_distance && aj.raw_sample_age_ms <= SONAR_COMPARE_SOURCE_LIVE_MS;
  bool deeper_live = deeper.has_depth &&
                     deeper.newest_sample_age_ms <= SONAR_COMPARE_SOURCE_LIVE_MS;
  bool uart_active = dbb_brain_synthetic_uart_nmea_active();

  ensure_mutex();
  sonar_compare_start_result_t result = SONAR_COMPARE_START_NOT_COMPILED;
  if (s_mutex != NULL) {
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    result = sonar_compare_core_start(&s_core, preflight_ok, uart_active,
                                      aj_live, deeper_live,
                                      DB_ACTIVE_SONAR_SOURCE);
    xSemaphoreGive(s_mutex);
  }
  if (result != SONAR_COMPARE_START_OK) {
    if (json) cJSON_Delete(json);
    httpd_resp_set_status(req, "409 Conflict");
    return compare_status_handler(req);
  }

  uint32_t session_id = 0;
  esp_err_t log_err = db_sonar_log_start_compare_capture(&session_id);
  if (log_err != ESP_OK) {
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_core.state = SONAR_COMPARE_INVALID;
    snprintf(s_core.reason, sizeof(s_core.reason),
             "refused: logging unavailable or another session is active");
    xSemaphoreGive(s_mutex);
    if (json) cJSON_Delete(json);
    httpd_resp_set_status(req, "409 Conflict");
    return compare_status_handler(req);
  }

  char site[48], bottom[32], notes[64];
  safe_token(site, sizeof(site), json ? cJSON_GetObjectItem(json, "site") : NULL, "unspecified");
  safe_token(bottom, sizeof(bottom), json ? cJSON_GetObjectItem(json, "bottom") : NULL, "unspecified");
  safe_token(notes, sizeof(notes), json ? cJSON_GetObjectItem(json, "notes") : NULL, "none");
  double spacing = json && cJSON_IsNumber(cJSON_GetObjectItem(json, "mount_spacing_mm"))
                       ? cJSON_GetObjectItem(json, "mount_spacing_mm")->valuedouble : 0.0;
  double aj_draft = json && cJSON_IsNumber(cJSON_GetObjectItem(json, "aj_draft_mm"))
                        ? cJSON_GetObjectItem(json, "aj_draft_mm")->valuedouble : 0.0;
  double deeper_draft = json && cJSON_IsNumber(cJSON_GetObjectItem(json, "deeper_draft_mm"))
                            ? cJSON_GetObjectItem(json, "deeper_draft_mm")->valuedouble : 0.0;
  double water_temp = json && cJSON_IsNumber(cJSON_GetObjectItem(json, "water_temp_c"))
                          ? cJSON_GetObjectItem(json, "water_temp_c")->valuedouble : -999.0;
  const esp_app_desc_t *app = esp_app_get_description();
  db_sonar_log_log_compare(
      "event=compare_start session=%lu firmware=%s model=aj-sr04m-pulse "
      "water_mps=%d tx_gpio=%d rx_gpio=%d pair_window_ms=%u "
      "mount_spacing_mm=%.1f aj_draft_mm=%.1f deeper_draft_mm=%.1f "
      "water_temp_c=%.1f site=%s bottom=%s notes=%s output_source=%d mavlink_publish=none",
      (unsigned long)session_id, app != NULL ? app->version : "unknown",
      (int)DB_PARAM_WATER_MPS, (int)DB_PARAM_SONAR_TX_GPIO,
      (int)DB_PARAM_SONAR_RX_GPIO, SONAR_COMPARE_PRIMARY_PAIR_WINDOW_MS,
      spacing, aj_draft, deeper_draft, water_temp, site, bottom, notes,
      DB_ACTIVE_SONAR_SOURCE);
  xSemaphoreTake(s_mutex, portMAX_DELAY);
  s_core.session_id = session_id;
  xSemaphoreGive(s_mutex);
  if (json) cJSON_Delete(json);
  return compare_status_handler(req);
}

static esp_err_t compare_stop_handler(httpd_req_t *req) {
  esp_err_t err = db_sonar_log_stop_compare_capture();
  if (err != ESP_OK) {
    httpd_resp_set_status(req, "409 Conflict");
    if (s_mutex != NULL) {
      xSemaphoreTake(s_mutex, portMAX_DELAY);
      snprintf(s_core.reason, sizeof(s_core.reason),
               "cannot seal while the FC is armed or no comparison is active");
      xSemaphoreGive(s_mutex);
    }
    return compare_status_handler(req);
  }
  xSemaphoreTake(s_mutex, portMAX_DELAY);
  sonar_compare_core_stop(&s_core);
  xSemaphoreGive(s_mutex);
  return compare_status_handler(req);
}

static esp_err_t compare_stream_chunk(const char *data, size_t length,
                                      void *ctx) {
  return httpd_resp_send_chunk((httpd_req_t *)ctx, data, length);
}

static esp_err_t compare_download_handler(httpd_req_t *req) {
  char query[64] = {0};
  bool preflight = false;
  if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
    char value[8] = {0};
    preflight = httpd_query_key_value(query, "preflight", value,
                                      sizeof(value)) == ESP_OK &&
                strcmp(value, "1") == 0;
  }
  httpd_resp_set_type(req, "text/plain");
  if (preflight) {
    httpd_resp_set_hdr(req, "Content-Disposition",
                       "attachment; filename=\"sonar-compare-empty.log\"");
    return httpd_resp_sendstr(req,
        "SONAR COMPARE TEST - EMPTY PREFLIGHT BUNDLE\nboot_ms=0 event=preflight_download_ok\n");
  }
  uint32_t session_id = 0;
  if (s_mutex != NULL) {
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    session_id = s_core.session_id;
    xSemaphoreGive(s_mutex);
  }
  if (session_id == 0) {
    httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "no comparison session");
    return ESP_FAIL;
  }
  char disposition[80];
  snprintf(disposition, sizeof(disposition),
           "attachment; filename=\"C%07lu.LOG\"", (unsigned long)session_id);
  httpd_resp_set_hdr(req, "Content-Disposition", disposition);
  esp_err_t err = db_sonar_log_stream(DB_LOG_STREAM_COMPARE, session_id,
                                      compare_stream_chunk, req, NULL, NULL);
  if (err != ESP_OK) return ESP_FAIL;
  return httpd_resp_send_chunk(req, NULL, 0);
}

void sonar_compare_test_register_http(void *server_v) {
  httpd_handle_t server = (httpd_handle_t)server_v;
  const httpd_uri_t routes[] = {
      {.uri = "/api/sonar-compare", .method = HTTP_GET,
       .handler = compare_status_handler},
      {.uri = "/api/sonar-compare/start", .method = HTTP_POST,
       .handler = compare_start_handler},
      {.uri = "/api/sonar-compare/stop", .method = HTTP_POST,
       .handler = compare_stop_handler},
      {.uri = "/api/sonar-compare/download", .method = HTTP_GET,
       .handler = compare_download_handler},
  };
  for (size_t i = 0; i < sizeof(routes) / sizeof(routes[0]); ++i) {
    esp_err_t err = httpd_register_uri_handler(server, &routes[i]);
    if (err != ESP_OK) ESP_LOGE(TAG, "route %s: %s", routes[i].uri,
                                esp_err_to_name(err));
  }
}
