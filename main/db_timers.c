/*
 *   This file is part of DroneBridge: https://github.com/DroneBridge/ESP32
 *
 *   Copyright 2026 Wolfgang Christl
 *
 *   Licensed under the Apache License, Version 2.0 (the "License");
 *   you may not use this file except in compliance with the License.
 *   You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 *   Unless required by applicable law or agreed to in writing, software
 *   distributed under the License is distributed on an "AS IS" BASIS,
 *   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *   See the License for the specific language governing permissions and
 *   limitations under the License.
 *
 */

#include "db_timers.h"
#include "danevi_sonar.h"
#include "db_sonar_log.h"
#include "db_fc_params.h"
#include "db_fc_tune.h"
#include "db_netlog.h"
#include "deeper_udp_sonar.h"
#include "db_mavlink_msgs.h"
#include "db_parameters.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "globals.h"
#include <esp_wifi.h>
#include <string.h>


#define TAG "DB_TIMERS"
#define DB_MAVLINK_SONAR_TASK_STACK_SIZE 6144
#define DB_MAVLINK_SONAR_TASK_PRIORITY 5
#define DB_MAVLINK_DEEPER_TEMP_PERIOD_MS 1000
#define DB_MAVLINK_DEEPER_TEMP_NAME "waterTemp"
#define DB_MAVLINK_FC_HEARTBEAT_PERIOD_MS 1000
// MAV_COMP_ID_ONBOARD_COMPUTER. Everything this firmware injects towards the FC carries this
// component id, and it is the identity the ELRS module's reply path already expects.
#define DB_MAVLINK_ESP_COMPONENT_ID 191

static TaskHandle_t s_sonar_publish_task_handle = NULL;
static TimerHandle_t s_sonar_timer_handle = NULL;
static TickType_t s_last_sonar_log_tick = 0;
static TickType_t s_last_deeper_temp_publish_tick = 0;
static TickType_t s_last_deeper_temp_log_tick = 0;
static TickType_t s_last_fc_heartbeat_tick = 0;
static bool s_sonar_task_missing_logged = false;
static uint8_t s_sonar_publish_buffer[296];
static fmav_status_t s_sonar_mav_status = {0};
static bool db_wifi_runtime_has_sta(void) {
  wifi_mode_t mode = WIFI_MODE_NULL;
  return esp_wifi_get_mode(&mode) == ESP_OK &&
         (mode == WIFI_MODE_STA || mode == WIFI_MODE_APSTA);
}

static bool db_wifi_runtime_has_ap(void) {
  wifi_mode_t mode = WIFI_MODE_NULL;
  return esp_wifi_get_mode(&mode) == ESP_OK &&
         (mode == WIFI_MODE_AP || mode == WIFI_MODE_APSTA);
}

/**
 * Callback that reads the RSSI that the ESP32 is having to the AP it is
 * connected to
 * @param pxTimer
 */
void db_timer_wifi_rssi_callback(TimerHandle_t pxTimer) {
  // This function is called periodically by the FreeRTOS timer
  if (db_wifi_runtime_has_sta()) {
    // update rssi variable - set to -127 when not available
    if (esp_wifi_sta_get_rssi((int *)&db_esp_signal_quality.air_rssi) !=
        ESP_OK) {
      db_esp_signal_quality.air_rssi = -127;
      ESP_LOGE(TAG, "Failed to get RSSI");
    } else { /* all good */
    }
  } else if (db_wifi_runtime_has_ap() &&
             DB_PARAM_RADIO_MODE == DB_WIFI_MODE_AP) {
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_ap_get_sta_list(
        &wifi_sta_list)); // update list of connected stations
  }
}

/**
 * Reads the RSSI that the ESP32 is having to the AP it is connected to every
 * DB_TIMER_RSSI_PERIOD_MS Or updated the client list in AP mode to get the
 * latest RSSI of every client
 */
void db_timer_start_wifi_rssi_timer() {
  static TimerHandle_t xRssiTimer;
  xRssiTimer =
      xTimerCreate("RSSI_Timer", pdMS_TO_TICKS(DB_TIMER_RSSI_PERIOD_MS), pdTRUE,
                   (void *)0, db_timer_wifi_rssi_callback);

  if (xRssiTimer == NULL) {
    ESP_LOGE(TAG, "Failed to create RSSI timer.");
  }
  if (xRssiTimer != NULL) {
    ESP_LOGI(TAG, "Starting RSSI timer.");
    xTimerStart(xRssiTimer, 0);
  }
}

/**
 * Injects a MAVLink heartbeat if MAVLink is selected as serial telemetry
 * protocol. If necessary, a fake heartbeat for the FC as well.
 * @param pxTimer
 */
void db_timer_mavlink_heartbeat_callback(TimerHandle_t pxTimer) {
  static uint8_t buff[296];
  if (DB_PARAM_SERIAL_PROTO != DB_SERIAL_PROTOCOL_MAVLINK) {
    return; // Do not send heartbeat in non mavlink modes
  }
  if (db_get_mav_sys_id() != 0) {
    // Send heartbeat via radio interface
    uint16_t length = db_mav_create_heartbeat(buff, &fmav_status_radio);
    db_send_to_all_radio_clients(buff, length);
  } else {
    // haven't seen a system ID from the FC yet so do not send any heartbeat or
    // heartbeats disabled
  }
}

/**
 * Starts a periodic MAVLink heartbeat sending timer.
 */
void db_timer_start_mavlink_heartbeat() {
  static TimerHandle_t xHeartBeatTimerHandle;
  xHeartBeatTimerHandle = xTimerCreate(
      "MAV_Heartbeat_Timer", pdMS_TO_TICKS(DB_TIMER_MAVLINK_HEARTBEAT_MS),
      pdTRUE, (void *)0, db_timer_mavlink_heartbeat_callback);

  if (xHeartBeatTimerHandle == NULL) {
    ESP_LOGE(TAG, "Failed to create heartbeat timer.");
  }
  if (xHeartBeatTimerHandle != NULL) {
    ESP_LOGI(TAG, "Starting to send heartbeats.");
    xTimerStart(xHeartBeatTimerHandle, 0);
  }
}

/**
 * Timer callback to send a Radio Status Message to the GND in case MAVLink is
 * set as serial protocol
 * @param pxTimer
 */
void db_timer_mavlink_radiostatus_callback(TimerHandle_t pxTimer) {
  if (DB_PARAM_SERIAL_PROTO != DB_SERIAL_PROTOCOL_MAVLINK) {
    return; // Do not send heartbeat in transparent mode
  }
  static uint8_t buff[296];
  bool runtime_sta = db_wifi_runtime_has_sta();
  bool runtime_ap = db_wifi_runtime_has_ap();
  // ESP32s that are connected to a flight controller via UART will send
  // RADIO_STATUS messages to the GND
  if (runtime_sta) {
    fmav_radio_status_t payload_r = {
        .fixed = 0,
        .txbuf = 0,
        .noise = db_esp_signal_quality.gnd_noise_floor,
        .remnoise = db_esp_signal_quality.air_noise_floor,
        .remrssi = db_format_rssi(db_esp_signal_quality.air_rssi,
                                  db_esp_signal_quality.air_noise_floor),
        .rssi = db_format_rssi(db_esp_signal_quality.gnd_rssi,
                               db_esp_signal_quality.gnd_noise_floor),
        .rxerrors = db_esp_signal_quality.gnd_rx_packets_lost};
    uint16_t len = fmav_msg_radio_status_encode_to_frame_buf(
        buff, db_get_mav_sys_id(), db_get_mav_comp_id(), &payload_r,
        &fmav_status_radio);
    db_send_to_all_radio_clients(buff, len);
  } else if (runtime_ap && DB_PARAM_RADIO_MODE == DB_WIFI_MODE_AP &&
             wifi_sta_list.num > 0) {
    // We assume ESP32 is not used in DB_WIFI_MODE_AP on the ground but only on
    // the drone side
    // -> We are in WiFi AP mode and connected to the drone
    // Send each connected client a radio status packet.
    // ToDo: Only the RSSI of the first (Wi-Fi) is considered.
    //  Easier for UDP since we have a nice list with mac addresses to use for
    //  mapping. Harder for TCP -> no MAC addresses available of connected
    //  clients
    fmav_radio_status_t payload_r = {
        .fixed = UINT8_MAX,
        .noise = UINT8_MAX,
        .remnoise = UINT8_MAX,
        .remrssi = UINT8_MAX,
        .rssi = db_format_rssi(wifi_sta_list.sta[0].rssi, -88),
        .rxerrors = 0,
        .txbuf = 0};
    uint16_t len = fmav_msg_radio_status_encode_to_frame_buf(
        buff, db_get_mav_sys_id(), db_get_mav_comp_id(), &payload_r,
        &fmav_status_radio);
    db_send_to_all_radio_clients(buff, len);
  } else {
    // In AP LR mode the clients will send the info to the GCS
    // directly, no need for the GND ESP32 to do anything
  }
}

/**
 * Starts a periodic MAVLink radio status sending timer.
 */
void db_timer_start_mavlink_radio_status() {
  static TimerHandle_t xRadioStatusTimerHandle;
  ;
  xRadioStatusTimerHandle = xTimerCreate(
      "MAV_Radiostatus_Timer", pdMS_TO_TICKS(DB_TIMER_MAVLINK_RADIOSTATUS_MS),
      pdTRUE, (void *)0, db_timer_mavlink_radiostatus_callback);

  if (xRadioStatusTimerHandle == NULL) {
    ESP_LOGE(TAG, "Failed to create Radio Status timer.");
  }
  if (xRadioStatusTimerHandle != NULL) {
    ESP_LOGI(TAG, "Starting to send radio status packets.");
    xTimerStart(xRadioStatusTimerHandle, 0);
  }
}

/**
 * Returns the latest distance from whichever sonar source is active for the
 * current boot policy.
 */
static bool db_get_active_sonar_distance(int *distance_mm,
                                         bool *use_deeper_sonar) {
  if (distance_mm == NULL || use_deeper_sonar == NULL) {
    return false;
  }

  *distance_mm = -1;
  *use_deeper_sonar = false;

  switch (DB_ACTIVE_SONAR_SOURCE) {
  case DB_SONAR_SOURCE_DEEPER:
    *use_deeper_sonar = true;
    return deeper_udp_sonar_get_latest_distance(distance_mm);
  case DB_SONAR_SOURCE_HARDWIRED:
    return danevi_sonar_get_latest_distance(distance_mm);
  default:
    return false;
  }
}

/**
 * Performs the actual DISTANCE_SENSOR encoding and transport work from a
 * dedicated task so the FreeRTOS timer service task stays lightweight.
 */
static void db_publish_active_sonar_distance(void) {
  /* One common cadence drives trip/energy collection and manual-capture
   * expiry, even when no sonar sample is currently available. */
  db_sonar_log_tick();
  db_fc_params_tick();
  db_fc_tune_tick();
  db_netlog_tick(); /* drains Wi-Fi breadcrumbs; must not run on the Wi-Fi task */
  if (DB_PARAM_SERIAL_PROTO != DB_SERIAL_PROTOCOL_MAVLINK) {
    return;
  }

  if (DB_ACTIVE_SONAR_SOURCE == DB_SONAR_SOURCE_HARDWIRED) {
    db_sonar_log_maybe_log_hardwired_session_sample();
  }

  int distance_mm = -1;
  bool use_deeper_sonar = false;
  if (!db_get_active_sonar_distance(&distance_mm, &use_deeper_sonar) ||
      distance_mm < 0) {
    return;
  }

  fmav_distance_sensor_t payload = {0};
  payload.time_boot_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
  payload.min_distance = use_deeper_sonar ? 0 : 5;
  payload.max_distance = use_deeper_sonar ? 10000 : 600;
  payload.current_distance = distance_mm / 10; // Convert mm to cm
  payload.type = 1; // MAV_DISTANCE_SENSOR_ULTRASOUND
  payload.id = use_deeper_sonar ? 1 : 0;
  payload.orientation = 25; // MAV_SENSOR_ROTATION_PITCH_270 (downward facing)
  payload.covariance = 0;

  uint16_t len = fmav_msg_distance_sensor_encode_to_frame_buf(
      s_sonar_publish_buffer, db_get_mav_sys_id() == 0 ? 1 : db_get_mav_sys_id(),
      DB_MAVLINK_ESP_COMPONENT_ID, &payload, &s_sonar_mav_status);

  write_to_serial(s_sonar_publish_buffer, len);
  db_send_to_all_radio_clients(s_sonar_publish_buffer, len);

  TickType_t now = xTaskGetTickCount();
  if ((now - s_last_sonar_log_tick) >= pdMS_TO_TICKS(1000)) {
    const char *source_name = use_deeper_sonar ? "Deeper" : "hardwired";
    ESP_LOGI(TAG, "Publishing %s sonar DISTANCE_SENSOR: %d mm (%d cm)",
             source_name, distance_mm, payload.current_distance);
    s_last_sonar_log_tick = now;
  }

  if (!use_deeper_sonar) {
    danevi_sonar_snapshot_t hardwired_snapshot = {0};
    if (danevi_sonar_get_snapshot(&hardwired_snapshot)) {
      db_sonar_log_maybe_log_hardwired_publish(
          distance_mm, payload.current_distance, &hardwired_snapshot);
    }
  }
}

/**
 * Emits a MAVLink HEARTBEAT towards the flight controller at ~1 Hz.
 *
 * ArduPilot forwards a message to a channel ONLY where it has learned the target lives, and it
 * learns purely from traffic it observes (MAVLink_routing::learn_route). This firmware otherwise
 * only listens and injects sensor data, so it used to appear in the FC's routing table solely as
 * a side effect of a fitted sonar - the hardwired transducer's DISTANCE_SENSOR traffic created
 * the entry within a second of every boot. With no sonar fitted the entry was never created at
 * all and the uplink TUNNEL path was silently dead, with no error anywhere.
 *
 * The FC's routing table never expires an entry, so one heartbeat per FC boot would technically
 * be enough. We send continuously anyway: this firmware cannot tell that the FC has rebooted, a
 * single lost frame would otherwise kill the link for the rest of the session, and the cost is
 * ~21 bytes per second.
 *
 * MAV_TYPE_GENERIC is deliberate. MAV_TYPE_ONBOARD_CONTROLLER would create the same routing
 * entry, but Rover looks that type up (find_by_mavtype, in ModeAuto::send_guided_position_target)
 * to choose the offboard navigation computer for NAV_GUIDED_ENABLE missions. Nothing here wants
 * that role, and GENERIC claims nothing while working identically.
 *
 * Serial only: the FC forwards broadcast heartbeats to its other channels itself, so also sending
 * this to the Wi-Fi clients would show the ground station a duplicate.
 */
static void db_publish_heartbeat_to_fc_if_due(void) {
  if (DB_PARAM_SERIAL_PROTO != DB_SERIAL_PROTOCOL_MAVLINK) {
    return;
  }

  TickType_t now_tick = xTaskGetTickCount();
  if (s_last_fc_heartbeat_tick != 0 &&
      (now_tick - s_last_fc_heartbeat_tick) <
          pdMS_TO_TICKS(DB_MAVLINK_FC_HEARTBEAT_PERIOD_MS)) {
    return;
  }

  // Not gated on having heard from the FC: the whole point is to be in its routing table as
  // early as possible after any FC boot, so fall back to system 1 until its id is known.
  uint16_t len = fmav_msg_heartbeat_pack_to_frame_buf(
      s_sonar_publish_buffer, db_get_mav_sys_id() == 0 ? 1 : db_get_mav_sys_id(),
      DB_MAVLINK_ESP_COMPONENT_ID, MAV_TYPE_GENERIC, MAV_AUTOPILOT_INVALID, 0, 0,
      MAV_STATE_ACTIVE, &s_sonar_mav_status);

  write_to_serial(s_sonar_publish_buffer, len);
  s_last_fc_heartbeat_tick = now_tick;
}

static void db_publish_deeper_temperature_if_due(void) {
  if (DB_PARAM_SERIAL_PROTO != DB_SERIAL_PROTOCOL_MAVLINK ||
      DB_ACTIVE_SONAR_SOURCE != DB_SONAR_SOURCE_DEEPER) {
    return;
  }

  TickType_t now_tick = xTaskGetTickCount();
  if (s_last_deeper_temp_publish_tick != 0 &&
      (now_tick - s_last_deeper_temp_publish_tick) <
          pdMS_TO_TICKS(DB_MAVLINK_DEEPER_TEMP_PERIOD_MS)) {
    return;
  }

  deeper_udp_snapshot_t snapshot = {0};
  if (!deeper_udp_sonar_get_snapshot(&snapshot) || !snapshot.has_temperature) {
    return;
  }

  fmav_named_value_float_t payload = {0};
  payload.time_boot_ms = now_tick * portTICK_PERIOD_MS;
  payload.value = ((float)snapshot.temperature_c_tenths) / 10.0f;
  memcpy(payload.name, DB_MAVLINK_DEEPER_TEMP_NAME,
         strlen(DB_MAVLINK_DEEPER_TEMP_NAME));

  uint16_t len = fmav_msg_named_value_float_encode_to_frame_buf(
      s_sonar_publish_buffer, db_get_mav_sys_id() == 0 ? 1 : db_get_mav_sys_id(),
      DB_MAVLINK_ESP_COMPONENT_ID, &payload, &s_sonar_mav_status);

  write_to_serial(s_sonar_publish_buffer, len);
  db_send_to_all_radio_clients(s_sonar_publish_buffer, len);
  s_last_deeper_temp_publish_tick = now_tick;

  if ((now_tick - s_last_deeper_temp_log_tick) >= pdMS_TO_TICKS(5000)) {
    ESP_LOGI(TAG, "Publishing Deeper water temperature NAMED_VALUE_FLOAT: %.1f C",
             payload.value);
    s_last_deeper_temp_log_tick = now_tick;
  }
}

/**
 * Dedicated sonar publisher task. The timer only wakes this task so the work
 * no longer runs inside the FreeRTOS timer service stack.
 */
static void db_mavlink_sonar_publish_task(void *arg) {
  (void)arg;
  ESP_LOGI(TAG, "Starting dedicated sonar MAVLink publish task.");
  while (1) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    // Heartbeat first: it is what keeps the FC's route to this component alive, and it must not
    // depend on a sonar being fitted or on any of the publishers below producing data.
    db_publish_heartbeat_to_fc_if_due();
    db_publish_active_sonar_distance();
    db_publish_deeper_temperature_if_due();
  }
}

/**
 * Timer callback for sonar publishing. Keep this tiny: only wake the dedicated
 * publish task instead of encoding and sending MAVLink inside Tmr Svc.
 * @param pxTimer
 */
void db_timer_mavlink_sonar_callback(TimerHandle_t pxTimer) {
  (void)pxTimer;
  if (DB_PARAM_SERIAL_PROTO != DB_SERIAL_PROTOCOL_MAVLINK) {
    return;
  }

  if (s_sonar_publish_task_handle == NULL) {
    if (!s_sonar_task_missing_logged) {
      ESP_LOGE(TAG,
               "Sonar publish timer fired before the dedicated sonar task was "
               "created.");
      s_sonar_task_missing_logged = true;
    }
    return;
  }

  xTaskNotifyGive(s_sonar_publish_task_handle);
}

/**
 * Starts a periodic MAVLink distance sensor sending timer at 10Hz.
 */
void db_timer_start_mavlink_sonar() {
  if (s_sonar_publish_task_handle == NULL) {
    BaseType_t task_created = xTaskCreate(
        db_mavlink_sonar_publish_task, "db_mav_sonar",
        DB_MAVLINK_SONAR_TASK_STACK_SIZE, NULL, DB_MAVLINK_SONAR_TASK_PRIORITY,
        &s_sonar_publish_task_handle);
    if (task_created != pdPASS) {
      s_sonar_publish_task_handle = NULL;
      ESP_LOGE(TAG, "Failed to create dedicated sonar publish task.");
      return;
    }
    s_sonar_task_missing_logged = false;
  }

  if (s_sonar_timer_handle == NULL) {
    s_sonar_timer_handle =
        xTimerCreate("MAV_Sonar_Timer",
                     pdMS_TO_TICKS(DB_TIMER_MAVLINK_SONAR_MS), // 100ms
                     pdTRUE, (void *)0, db_timer_mavlink_sonar_callback);
    if (s_sonar_timer_handle == NULL) {
      ESP_LOGE(TAG, "Failed to create Sonar Timer!");
      return;
    }
  }

  if (xTimerIsTimerActive(s_sonar_timer_handle) == pdFALSE) {
    ESP_LOGI(TAG, "Starting to send DISTANCE_SENSOR packets.");
    if (xTimerStart(s_sonar_timer_handle, 0) != pdPASS) {
      ESP_LOGE(TAG, "Failed to start Sonar Timer!");
    }
  }
}
