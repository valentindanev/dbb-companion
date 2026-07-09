/*
 *   This file is part of DroneBridge: https://github.com/DroneBridge/ESP32
 *
 *   Copyright 2018 Wolfgang Christl
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

#include <driver/gpio.h>
#include <esp_ota_ops.h>
#include <esp_wifi_types.h>
#include <lwip/apps/netbiosns.h>
#include <nvs_flash.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "db_esp32_control.h"
#include "db_protocol.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_spiffs.h"
#include "esp_vfs_semihost.h"
#include "esp_wifi.h"
#include "esp_wifi_default.h"
#include "esp_wifi_types.h"
#include "freertos/event_groups.h"
#include "http_server.h"
#include "dbb_brain.h"
#include "main.h"

#include <button_gpio.h>
#include <iot_button.h>

#include "deeper_udp_sonar.h"
#include "danevi_sonar.h"
#include "db_parameters.h"
#include "db_serial.h"
#include "db_sonar_log.h"
#include "globals.h"
#include "mdns.h"

#include <db_timers.h>

#define NVS_NAMESPACE "settings"

/* used to reset ESP to defaults and force restart or to reset the mode to
 * access point mode */
#ifdef CONFIG_IDF_TARGET_ESP32C3
#define DB_RESET_PIN GPIO_NUM_9
#elif CONFIG_IDF_TARGET_ESP32C6
#define DB_RESET_PIN GPIO_NUM_9
#elif CONFIG_IDF_TARGET_ESP32S2
#define DB_RESET_PIN GPIO_NUM_0
#elif CONFIG_IDF_TARGET_ESP32S3
#define DB_RESET_PIN GPIO_NUM_0
#elif CONFIG_IDF_TARGET_ESP32
#define DB_RESET_PIN GPIO_NUM_0
#else
#define DB_RESET_PIN GPIO_NUM_0
#endif

static const char *TAG = "DB_ESP32";
static const uint32_t DB_DEEPER_BOOT_CONNECT_WINDOW_MS = 60000;
static const size_t DB_PARAM_LOG_BUFFER_SIZE = 2048;
static const char *DB_UPDATE_AP_ONCE_KEY = "upd_ap_once";
static bool s_web_fs_available = false;

static void db_refresh_ap_sta_list_if_available(void) {
  esp_err_t err = esp_wifi_ap_get_sta_list(&wifi_sta_list);
  if (err == ESP_OK) {
    return;
  }

  if (err == ESP_ERR_WIFI_STOP_STATE || err == ESP_ERR_WIFI_MODE ||
      err == ESP_ERR_WIFI_NOT_INIT) {
    ESP_LOGD(TAG, "Skipping AP station list refresh during Wi-Fi shutdown (%s)",
             esp_err_to_name(err));
    return;
  }

  ESP_ERROR_CHECK_WITHOUT_ABORT(err);
}

static void db_log_param_values(const char *prefix) {
  uint8_t *param_str_buffer = calloc(DB_PARAM_LOG_BUFFER_SIZE, sizeof(uint8_t));
  if (param_str_buffer == NULL) {
    ESP_LOGE(TAG, "Failed to allocate parameter log buffer");
    return;
  }

  db_param_print_values_to_buffer(param_str_buffer, DB_PARAM_LOG_BUFFER_SIZE);
  if (prefix != NULL) {
    ESP_LOGI(TAG, "%s%s", prefix, param_str_buffer);
  } else {
    ESP_LOGI(TAG, "%s", param_str_buffer);
  }

  free(param_str_buffer);
}

char CURRENT_CLIENT_IP[IP4ADDR_STRLEN_MAX] = "192.168.42.1";
uint8_t DB_RADIO_IS_OFF = false; // keep track if we switched Wi-Fi/BLE off
                                 // already - by default a radio is started
db_esp_signal_quality_t db_esp_signal_quality = {.air_rssi = UINT8_MAX,
                                                 .air_noise_floor = UINT8_MAX,
                                                 .gnd_rssi = UINT8_MAX,
                                                 .gnd_noise_floor = UINT8_MAX};
wifi_sta_list_t wifi_sta_list = {.num = 0};
uint8_t LOCAL_MAC_ADDRESS[6];
db_sonar_source_t DB_ACTIVE_SONAR_SOURCE = DB_SONAR_SOURCE_NONE;

// Wi-Fi client mode vars
static int s_retry_num = 0;
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1
static TickType_t s_sta_retry_deadline_tick = 0;
static uint32_t s_sta_retry_window_ms = 0;
static bool s_sta_retry_uses_deadline = false;
static bool s_deeper_sta_session_active = false;

esp_netif_t *esp_default_netif;
static esp_event_handler_instance_t s_wifi_any_id_handler;
static esp_event_handler_instance_t s_sta_got_ip_handler;
static esp_event_handler_instance_t s_ap_staipassigned_handler;

static bool db_is_saved_wifi_target_deeper(void) {
  return DB_PARAM_DEEPER_EN &&
         strcmp(DB_PARAM_WIFI_SSID, DB_PARAM_DEEPER_SSID) == 0;
}

static bool db_is_deeper_sta_target(void) {
  return s_deeper_sta_session_active || db_is_saved_wifi_target_deeper();
}

static const char *db_wifi_ssid_for_log(void) {
  if (s_deeper_sta_session_active) {
    return DB_PARAM_DEEPER_SSID;
  }
  return DB_PARAM_WIFI_SSID;
}

static const char *db_wifi_password_for_log(void) {
  if (s_deeper_sta_session_active) {
    return strlen(DB_PARAM_DEEPER_PASS) > 0 ? "<configured>" : "<open>";
  }
  if (db_is_saved_wifi_target_deeper()) {
    return strlen(DB_PARAM_PASS) > 0 ? "<configured>" : "<open>";
  }
  return DB_PARAM_PASS;
}

static bool db_wifi_runtime_has_sta(void) {
  wifi_mode_t mode = WIFI_MODE_NULL;
  return esp_wifi_get_mode(&mode) == ESP_OK &&
         (mode == WIFI_MODE_STA || mode == WIFI_MODE_APSTA);
}

static void db_reset_sta_retry_window(void) {
  s_sta_retry_deadline_tick = 0;
  s_sta_retry_window_ms = 0;
  s_sta_retry_uses_deadline = false;
}

static void db_configure_sta_retry_window(uint32_t timeout_ms) {
  if (timeout_ms > 0) {
    s_sta_retry_deadline_tick = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
    s_sta_retry_window_ms = timeout_ms;
    s_sta_retry_uses_deadline = true;
  } else {
    db_reset_sta_retry_window();
  }
}

static bool db_sta_retry_window_expired(void) {
  if (!s_sta_retry_uses_deadline) {
    return false;
  }
  return (int32_t)(xTaskGetTickCount() - s_sta_retry_deadline_tick) >= 0;
}

static esp_err_t db_set_dns_server(esp_netif_t *netif, uint32_t addr,
                                   esp_netif_dns_type_t type) {
  if (addr && (addr != IPADDR_NONE)) {
    esp_netif_dns_info_t dns;
    dns.ip.u_addr.ip4.addr = addr;
    dns.ip.type = IPADDR_TYPE_V4;
    ESP_ERROR_CHECK(esp_netif_set_dns_info(netif, type, &dns));
  }
  return ESP_OK;
}

static void db_copy_cstr(char *dst, size_t dst_size, const char *src) {
  if (dst == NULL || dst_size == 0) {
    return;
  }
  if (src == NULL) {
    dst[0] = '\0';
    return;
  }
  strncpy(dst, src, dst_size - 1);
  dst[dst_size - 1] = '\0';
}

static bool db_consume_update_ap_mode_on_next_boot(void) {
  nvs_handle_t my_handle;
  uint8_t boot_ap_once = 0;
  bool should_force_ap = false;

  if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &my_handle) != ESP_OK) {
    ESP_LOGW(TAG, "Failed to open NVS while checking update-mode boot flag");
    return false;
  }

  esp_err_t err = nvs_get_u8(my_handle, DB_UPDATE_AP_ONCE_KEY, &boot_ap_once);
  if (err == ESP_OK && boot_ap_once == 1) {
    should_force_ap = true;
    ESP_ERROR_CHECK_WITHOUT_ABORT(
        nvs_erase_key(my_handle, DB_UPDATE_AP_ONCE_KEY));
    ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_commit(my_handle));
  } else if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
    ESP_LOGW(TAG, "Failed to read update-mode boot flag (%s)",
             esp_err_to_name(err));
  }

  nvs_close(my_handle);
  return should_force_ap;
}

esp_err_t db_request_update_ap_mode_on_next_boot(void) {
  nvs_handle_t my_handle;
  ESP_ERROR_CHECK(nvs_open(NVS_NAMESPACE, NVS_READWRITE, &my_handle));
  ESP_ERROR_CHECK(nvs_set_u8(my_handle, DB_UPDATE_AP_ONCE_KEY, 1));
  ESP_ERROR_CHECK(nvs_commit(my_handle));
  nvs_close(my_handle);
  return ESP_OK;
}

bool db_is_web_fs_available(void) { return s_web_fs_available; }

esp_err_t db_unmount_web_fs(void) {
  if (!s_web_fs_available) {
    return ESP_OK;
  }

#if CONFIG_WEB_DEPLOY_SF
  esp_err_t err = esp_vfs_spiffs_unregister(DB_WEB_PARTITION_LABEL);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to unmount SPIFFS before web OTA (%s)",
             esp_err_to_name(err));
    return err;
  }
#endif

  s_web_fs_available = false;
  return ESP_OK;
}

static void db_mark_running_ota_image_valid_if_needed(void) {
  const esp_partition_t *running = esp_ota_get_running_partition();
  esp_ota_img_states_t ota_state;
  esp_err_t err = esp_ota_get_state_partition(running, &ota_state);

  if (err == ESP_OK && ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
    err = esp_ota_mark_app_valid_cancel_rollback();
    if (err == ESP_OK) {
      ESP_LOGI(TAG, "Confirmed pending OTA image as valid.");
    } else {
      ESP_LOGE(TAG, "Failed to confirm pending OTA image (%s)",
               esp_err_to_name(err));
    }
  } else if (err != ESP_OK && err != ESP_ERR_NOT_SUPPORTED) {
    ESP_LOGW(TAG, "Unable to query OTA image state (%s)", esp_err_to_name(err));
  }
}

static void db_unregister_wifi_handlers(void) {
  if (s_wifi_any_id_handler != NULL) {
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_event_handler_instance_unregister(
        WIFI_EVENT, ESP_EVENT_ANY_ID, s_wifi_any_id_handler));
    s_wifi_any_id_handler = NULL;
  }
  if (s_sta_got_ip_handler != NULL) {
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_event_handler_instance_unregister(
        IP_EVENT, IP_EVENT_STA_GOT_IP, s_sta_got_ip_handler));
    s_sta_got_ip_handler = NULL;
  }
  if (s_ap_staipassigned_handler != NULL) {
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_event_handler_instance_unregister(
        IP_EVENT, IP_EVENT_AP_STAIPASSIGNED, s_ap_staipassigned_handler));
    s_ap_staipassigned_handler = NULL;
  }
}

static void db_cleanup_wifi_runtime(void) {
  db_unregister_wifi_handlers();
  s_deeper_sta_session_active = false;

  esp_err_t err = esp_wifi_stop();
  if (err != ESP_OK && err != ESP_ERR_WIFI_NOT_INIT &&
      err != ESP_ERR_WIFI_NOT_STARTED) {
    ESP_LOGW(TAG, "esp_wifi_stop during cleanup failed: %s",
             esp_err_to_name(err));
  }

  if (esp_default_netif != NULL) {
    esp_netif_destroy_default_wifi(esp_default_netif);
    esp_default_netif = NULL;
  }

  err = esp_wifi_deinit();
  if (err != ESP_OK && err != ESP_ERR_WIFI_NOT_INIT) {
    ESP_LOGW(TAG, "esp_wifi_deinit during cleanup failed: %s",
             esp_err_to_name(err));
  }

  err = esp_event_loop_delete_default();
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
    ESP_LOGW(TAG, "esp_event_loop_delete_default during cleanup failed: %s",
             esp_err_to_name(err));
  }

  if (s_wifi_event_group != NULL) {
    vEventGroupDelete(s_wifi_event_group);
    s_wifi_event_group = NULL;
  }

  s_retry_num = 0;
  db_reset_sta_retry_window();
}

/**
 * Assigns static IP to ESP32 when in client mode and static IP, GW and netmask
 * are set in config. Stops client DHCP server
 */
static void set_client_static_ip() {
  if (db_wifi_runtime_has_sta() &&
      strlen((char *)DB_PARAM_STA_IP) > 0 &&
      strlen((char *)DB_PARAM_STA_GW) > 0 &&
      strlen((char *)DB_PARAM_STA_IP_NETMASK) > 0) {
    ESP_LOGI(
        TAG,
        "Assigning static IP to ESP32: ESP32-IP: %s Gateway: %s Netmask: %s",
        (char *)DB_PARAM_STA_IP, (char *)DB_PARAM_STA_GW,
        (char *)DB_PARAM_STA_IP_NETMASK);

    if (esp_netif_dhcpc_stop(esp_default_netif) != ESP_OK) {
      ESP_LOGE(TAG, "Failed to stop dhcp client in order to set static IP");
      return;
    }
    esp_netif_ip_info_t ip;
    memset(&ip, 0, sizeof(esp_netif_ip_info_t));
    ip.ip.addr = ipaddr_addr((char *)DB_PARAM_STA_IP);
    ip.netmask.addr = ipaddr_addr((char *)DB_PARAM_STA_IP_NETMASK);
    ip.gw.addr = ipaddr_addr((char *)DB_PARAM_STA_GW);
    if (esp_netif_set_ip_info(esp_default_netif, &ip) != ESP_OK) {
      ESP_LOGE(TAG, "Failed to set static ip info");
    }
    ESP_LOGD(TAG, "Success to set static ip: %s, netmask: %s, gw: %s",
             (char *)DB_PARAM_STA_IP, (char *)DB_PARAM_STA_IP_NETMASK,
             (char *)DB_PARAM_STA_GW);
    ESP_ERROR_CHECK(db_set_dns_server(esp_default_netif,
                                      ipaddr_addr((char *)DB_PARAM_STA_GW),
                                      ESP_NETIF_DNS_MAIN));
    ESP_ERROR_CHECK(db_set_dns_server(esp_default_netif, ipaddr_addr("0.0.0.0"),
                                      ESP_NETIF_DNS_BACKUP));
  } else {
    // no static IP specified let the DHCP assign us one
  }
}

/**
 * Devices get added based on IP (check if IP & PORT are already listed) and
 * removed from the UDP broadcast connection based on MAC address
 *
 * @param arg
 * @param event_base
 * @param event_id
 * @param event_data
 */
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data) {
  // Wifi access point mode events
  if (event_id == WIFI_EVENT_AP_STACONNECTED) {
    wifi_event_ap_staconnected_t *event =
        (wifi_event_ap_staconnected_t *)event_data;
    ESP_LOGI(TAG, "WIFI_EVENT - Client connected - station:" MACSTR ", AID=%d",
             MAC2STR(event->mac), event->aid);
    db_refresh_ap_sta_list_if_available(); // update list of connected stations
  } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
    wifi_event_ap_stadisconnected_t *event =
        (wifi_event_ap_stadisconnected_t *)event_data;
    ESP_LOGI(TAG,
             "WIFI_EVENT - Client disconnected - station:" MACSTR ", AID=%d",
             MAC2STR(event->mac), event->aid);
    struct db_udp_client_t db_udp_client;
    memcpy(db_udp_client.mac, event->mac, sizeof(db_udp_client.mac));
    remove_from_known_udp_clients(udp_conn_list, db_udp_client);
    db_refresh_ap_sta_list_if_available(); // update list of connected stations
  } else if (event_id == WIFI_EVENT_AP_START) {
    ESP_LOGI(TAG, "WIFI_EVENT_AP_START (SSID: %s PASS: %s)", DB_PARAM_WIFI_SSID,
             DB_PARAM_PASS);
  } else if (event_id == WIFI_EVENT_AP_STOP) {
    ESP_LOGI(TAG, "WIFI_EVENT - AP stopped!");
  } else if (event_base == IP_EVENT && event_id == IP_EVENT_AP_STAIPASSIGNED) {
    ip_event_ap_staipassigned_t *event =
        (ip_event_ap_staipassigned_t *)event_data;
    ESP_LOGI(TAG, "IP_EVENT_AP_STAIPASSIGNED - New station IP:" IPSTR,
             IP2STR(&event->ip));
    ESP_LOGI(TAG, "IP_EVENT_AP_STAIPASSIGNED - MAC: " MACSTR,
             MAC2STR(event->mac));
    struct db_udp_client_t db_udp_client;
    db_udp_client.udp_client.sin_family = PF_INET;
    db_udp_client.udp_client.sin_port = htons(APP_PORT_PROXY_UDP);
    db_udp_client.udp_client.sin_len = 16;
    db_udp_client.udp_client.sin_addr.s_addr = event->ip.addr;
    memcpy(db_udp_client.mac, event->mac, sizeof(db_udp_client.mac));
    add_to_known_udp_clients(udp_conn_list, db_udp_client, false);
  }
  // Wifi client mode events
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
    ESP_LOGI(TAG, "WIFI_EVENT_STA_START - Wifi Started");
    if (!DB_RADIO_IS_OFF) { // maybe the other task did set it in the meantime
      ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_connect());
    } else {
      ESP_LOGW(TAG, "Did not start Wi-Fi since autopilot told us he is armed "
                    "(WIFI_EVENT_STA_START)");
    }
  } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED) {
    set_client_static_ip();
  } else if (event_base == WIFI_EVENT &&
             event_id == WIFI_EVENT_STA_DISCONNECTED) {
    wifi_event_sta_disconnected_t *event =
        (wifi_event_sta_disconnected_t *)event_data;
    ESP_LOGI(TAG,
             "WIFI_EVENT_STA_DISCONNECTED - Lost connection to access point "
             "(reason: %d)",
             event->reason);
    // Keep on trying
    if (!DB_RADIO_IS_OFF) {
      if (s_sta_retry_uses_deadline) {
        if (!db_sta_retry_window_expired()) {
          ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_connect());
          s_retry_num++;
          ESP_LOGI(TAG,
                   "Retry to connect to the AP (%i) within boot window (%lu ms)",
                   s_retry_num, (unsigned long)s_sta_retry_window_ms);
        } else {
          ESP_LOGW(TAG, "STA connect window expired after %lu ms. Triggering "
                        "fall-back!",
                   (unsigned long)s_sta_retry_window_ms);
          xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
      } else if (s_retry_num < 15) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_connect());
        s_retry_num++;
        ESP_LOGI(TAG, "Retry to connect to the AP (%i)", s_retry_num);
      } else {
        ESP_LOGW(TAG, "Max retries reached. Triggering fall-back!");
        xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
      }
    } else {
      ESP_LOGD(TAG, "WIFI_EVENT_STA_DISCONNECTED - did not try to re-connect "
                    "since WiFi was commanded off");
    }
  } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    ESP_LOGI(TAG, "IP_EVENT_STA_GOT_IP:" IPSTR, IP2STR(&event->ip_info.ip));
    sprintf(CURRENT_CLIENT_IP, IPSTR, IP2STR(&event->ip_info.ip));
    s_retry_num = 0;
    db_reset_sta_retry_window();
    xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
  }
}

void start_mdns_service() {
  esp_err_t err = mdns_init();
  if (err) {
    printf("MDNS Init failed: %d\n", err);
    return;
  }
  ESP_ERROR_CHECK(mdns_hostname_set("dronebridge"));
  ESP_ERROR_CHECK(mdns_instance_name_set("DroneBridge for ESP32"));

  ESP_ERROR_CHECK(mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0));
  ESP_ERROR_CHECK(
      mdns_service_add(NULL, "_db_proxy", "_tcp", APP_PORT_PROXY, NULL, 0));
  ESP_ERROR_CHECK(
      mdns_service_add(NULL, "_db_comm", "_tcp", APP_PORT_COMM, NULL, 0));
  ESP_ERROR_CHECK(
      mdns_service_instance_name_set("_http", "_tcp", "DroneBridge for ESP32"));
  ESP_LOGI(TAG, "MDNS Service started!");
}

#if CONFIG_WEB_DEPLOY_SEMIHOST
esp_err_t init_fs(void) {
  esp_err_t ret = esp_vfs_semihost_register(CONFIG_WEB_MOUNT_POINT,
                                            CONFIG_HOST_PATH_TO_MOUNT);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to register semihost driver (%s)!",
             esp_err_to_name(ret));
    return ESP_FAIL;
  }
  return ESP_OK;
}
#endif

#if CONFIG_WEB_DEPLOY_SF

esp_err_t init_fs(void) {
  esp_vfs_spiffs_conf_t conf = {.base_path = CONFIG_WEB_MOUNT_POINT,
                                .partition_label = DB_WEB_PARTITION_LABEL,
                                .max_files = 5,
                                .format_if_mount_failed = false};
  esp_err_t ret = esp_vfs_spiffs_register(&conf);

  if (ret != ESP_OK) {
    s_web_fs_available = false;
    if (ret == ESP_FAIL) {
      ESP_LOGE(TAG, "Failed to mount or format filesystem");
    } else if (ret == ESP_ERR_NOT_FOUND) {
      ESP_LOGE(TAG, "Failed to find SPIFFS partition");
    } else {
      ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
    }
    return ESP_FAIL;
  }

  size_t total = 0, used = 0;
  ret = esp_spiffs_info(DB_WEB_PARTITION_LABEL, &total, &used);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)",
             esp_err_to_name(ret));
  } else {
    s_web_fs_available = true;
    ESP_LOGI(TAG,
             "Filesystem init finished! Partition size: total: %d bytes, used: "
             "%d bytes (%i%%)",
             total, used, (used * 100) / total);
  }
  return ret;
}

#endif

/**
 * Launches an access point where ground stations can connect to
 *
 * @param wifi_mode Allows to overwrite an AP mode from traditional WiFi to LR
 * Mode
 */
void db_init_wifi_apmode(int wifi_mode) {
  db_cleanup_wifi_runtime();
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_default_netif = esp_netif_create_default_wifi_ap();
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL,
      &s_wifi_any_id_handler));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      IP_EVENT, IP_EVENT_AP_STAIPASSIGNED, &wifi_event_handler, NULL,
      &s_ap_staipassigned_handler));

  wifi_config_t wifi_config = {
      .ap = {.ssid = "DroneBridge ESP32 Init Error",
             .password = "dronebridge",
             .ssid_len = 0,
             .authmode = WIFI_AUTH_WPA2_PSK,
             .channel = db_param_channel.value.db_param_u8.value,
             .ssid_hidden = 0,
             .beacon_interval = 100,
             .max_connection = 10},
  };
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstringop-truncation"
  // Set Wi-Fi SSID and password from the stored parameters
  if (strlen(DB_PARAM_WIFI_SSID) >= db_param_ssid.value.db_param_str.min_len) {
    db_copy_cstr((char *)wifi_config.ap.ssid, sizeof(wifi_config.ap.ssid),
                 DB_PARAM_WIFI_SSID);
  } else {
    // something is wrong - switch to default value
    db_copy_cstr((char *)wifi_config.ap.ssid, sizeof(wifi_config.ap.ssid),
                 (char *)db_param_ssid.value.db_param_str.default_value);
  }
  // DBB: give each boat a unique AP SSID ("DaneviBaitBoats" -> "DaneviBaitBoats_XXXX")
  // so multiple boats on the water don't clash. Suffix = 4 letters derived from the
  // SoftAP MAC (unique per ESP32, stable per board). Skipped if a custom SSID is set.
  if (strcmp((char *)wifi_config.ap.ssid, "DaneviBaitBoats") == 0) {
    uint8_t dbb_mac[6];
    esp_read_mac(dbb_mac, ESP_MAC_WIFI_SOFTAP);
    const char *dbb_L = "ABCDEFGHIJKLMNOPQRSTUVWXYZ";
    size_t dbb_len = strlen((char *)wifi_config.ap.ssid);
    snprintf((char *)wifi_config.ap.ssid + dbb_len,
             sizeof(wifi_config.ap.ssid) - dbb_len, "_%c%c%c%c",
             dbb_L[dbb_mac[3] % 26], dbb_L[dbb_mac[4] % 26],
             dbb_L[dbb_mac[5] % 26],
             dbb_L[(dbb_mac[3] + dbb_mac[4] + dbb_mac[5]) % 26]);
    ESP_LOGI(TAG, "DBB unique AP SSID: %s", (char *)wifi_config.ap.ssid);
  }
  if (strlen(DB_PARAM_PASS) >= db_param_pass.value.db_param_str.min_len) {
    db_copy_cstr((char *)wifi_config.ap.password,
                 sizeof(wifi_config.ap.password), DB_PARAM_PASS);
  } else {
    // something is wrong - switch to default value
    db_copy_cstr((char *)wifi_config.ap.password,
                 sizeof(wifi_config.ap.password),
                 (char *)db_param_pass.value.db_param_str.default_value);
  }
#pragma GCC diagnostic pop

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
  ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_AP, WIFI_PROTOCOL_11B));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
  wifi_country_t wifi_country = {.cc = "US",
                                 .schan = 1,
                                 .nchan = 13,
                                 .policy = WIFI_COUNTRY_POLICY_MANUAL};
  ESP_ERROR_CHECK(esp_wifi_set_country(&wifi_country));
  ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
  ESP_ERROR_CHECK(esp_wifi_start());
  DB_RADIO_IS_OFF = false; // just to be sure, but should not be necessary

  /* Assign IP to ap/gateway */
  esp_netif_ip_info_t ip;
  memset(&ip, 0, sizeof(esp_netif_ip_info_t));
  ip.ip.addr = ipaddr_addr(DB_PARAM_AP_IP);
  ip.netmask.addr = ipaddr_addr("255.255.255.0");
  ip.gw.addr = ipaddr_addr(DB_PARAM_AP_IP);
  ESP_ERROR_CHECK(esp_netif_dhcps_stop(esp_default_netif));
  ESP_ERROR_CHECK(esp_netif_set_ip_info(esp_default_netif, &ip));
  ESP_ERROR_CHECK(esp_netif_dhcps_start(esp_default_netif));

  ESP_ERROR_CHECK(esp_netif_set_hostname(
      esp_default_netif,
      (char *)db_param_wifi_hostname.value.db_param_str.value));
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstringop-truncation"
  db_copy_cstr(CURRENT_CLIENT_IP, sizeof(CURRENT_CLIENT_IP), DB_PARAM_AP_IP);
#pragma GCC diagnostic pop
  ESP_ERROR_CHECK(esp_read_mac(LOCAL_MAC_ADDRESS, ESP_MAC_WIFI_SOFTAP));
}

/**
 * Initializes the ESP Wi-fi client/station mode where we connect to a known
 * access point.
 */
int db_init_wifi_clientmode(uint32_t connect_window_ms) {
  db_cleanup_wifi_runtime();
  s_wifi_event_group = xEventGroupCreate();
  db_configure_sta_retry_window(connect_window_ms);
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_default_netif = esp_netif_create_default_wifi_sta();
  assert(esp_default_netif);
  ESP_ERROR_CHECK(esp_netif_set_hostname(
      esp_default_netif,
      (char *)db_param_wifi_hostname.value.db_param_str.value));

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL,
      &s_wifi_any_id_handler));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL,
      &s_sta_got_ip_handler));

  bool is_deeper_sta = db_is_deeper_sta_target();
  bool is_deeper_open = is_deeper_sta && strlen(DB_PARAM_PASS) == 0;
  wifi_config_t wifi_config = {
      .sta = {.ssid = "DroneBridge_ESP32_Init",
              .password = "dronebridge",
              .threshold.authmode =
                  is_deeper_open ? WIFI_AUTH_OPEN
                                 : (is_deeper_sta ? WIFI_AUTH_WPA_PSK
                                                  : WIFI_AUTH_WEP)},
  };
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstringop-truncation"
  // Set Wi-Fi SSID and password from the stored parameters
  if (strlen(DB_PARAM_WIFI_SSID) >= db_param_ssid.value.db_param_str.min_len) {
    db_copy_cstr((char *)wifi_config.sta.ssid, sizeof(wifi_config.sta.ssid),
                 DB_PARAM_WIFI_SSID);
  } else {
    // something is wrong - switch to default value
    db_copy_cstr((char *)wifi_config.sta.ssid, sizeof(wifi_config.sta.ssid),
                 (char *)db_param_ssid.value.db_param_str.default_value);
  }
  if (is_deeper_open) {
    wifi_config.sta.password[0] = '\0';
    ESP_LOGI(TAG, "Using open security for Deeper STA connection");
  } else if (is_deeper_sta) {
    db_copy_cstr((char *)wifi_config.sta.password,
                 sizeof(wifi_config.sta.password), DB_PARAM_PASS);
    ESP_LOGI(TAG, "Using configured password for Deeper STA connection");
  } else if (strlen(DB_PARAM_PASS) >=
             db_param_pass.value.db_param_str.min_len) {
    db_copy_cstr((char *)wifi_config.sta.password,
                 sizeof(wifi_config.sta.password), DB_PARAM_PASS);
  } else {
    // something is wrong - switch to default value
    db_copy_cstr((char *)wifi_config.sta.password,
                 sizeof(wifi_config.sta.password),
                 (char *)db_param_pass.value.db_param_str.default_value);
  }
#pragma GCC diagnostic pop

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  bool force_gn_for_deeper = is_deeper_sta;
  if (force_gn_for_deeper && !DB_PARAM_WIFI_EN_GN) {
    ESP_LOGI(TAG, "Forcing 802.11b/g/n compatibility for Deeper STA "
                  "connection");
  }
  if (DB_PARAM_WIFI_EN_GN || force_gn_for_deeper) {
    // only makes sense if the AP can not do proper N or you do not need range
    // or want Wi-Fi 6 ax support
#ifdef CONFIG_IDF_TARGET_ESP32C6
    ESP_ERROR_CHECK(esp_wifi_set_protocol(
        WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N |
                         WIFI_PROTOCOL_11AX | WIFI_PROTOCOL_LR));
#else
    ESP_ERROR_CHECK(esp_wifi_set_protocol(
        WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N |
                         WIFI_PROTOCOL_LR));
#endif
  } else {
    ESP_ERROR_CHECK(esp_wifi_set_protocol(
        WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_LR)); // range for sure
  }
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE)); // disable power saving
  ESP_ERROR_CHECK(esp_wifi_start());
  DB_RADIO_IS_OFF = false; // just to be sure, but should not be necessary
  // Consider connection lost after 1s of no beacon - triggers reconnect via
  // WIFI_EVENT_STA_DISCONNECTED event
  ESP_ERROR_CHECK(esp_wifi_set_inactive_time(WIFI_IF_STA, 3));

  ESP_LOGI(TAG, "Init of WiFi Client-Mode finished. (SSID: %s PASS: %s)",
           db_wifi_ssid_for_log(), db_wifi_password_for_log());

  /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or
   * connection failed for the maximum number of re-tries (WIFI_FAIL_BIT). The
   * bits are set by event_handler() (see above) */
  EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                         WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                         pdFALSE, pdFALSE,
                                         connect_window_ms > 0
                                             ? pdMS_TO_TICKS(connect_window_ms)
                                             : portMAX_DELAY);

  /* xEventGroupWaitBits() returns the bits before the call returned, hence we
   * can test which event actually happened. */
  bool enable_temp_ap_mode = false;
  if (bits & WIFI_CONNECTED_BIT) {
    ESP_LOGI(TAG, "Connected to ap SSID:%s password:%s", db_wifi_ssid_for_log(),
             db_wifi_password_for_log());
  } else if (bits & WIFI_FAIL_BIT) {
    ESP_LOGW(TAG, "Failed to connect to SSID:%s, password:%s",
             db_wifi_ssid_for_log(), db_wifi_password_for_log());
    enable_temp_ap_mode = true;
  } else {
    if (connect_window_ms > 0) {
      ESP_LOGW(TAG, "Timed out after %lu ms while trying to connect to SSID:%s",
               (unsigned long)connect_window_ms, db_wifi_ssid_for_log());
      enable_temp_ap_mode = true;
    } else {
      ESP_LOGE(TAG, "UNEXPECTED WIFI EVENT");
    }
  }
  if (enable_temp_ap_mode) {
    ESP_LOGW(TAG, "WiFi client mode was not able to connect to the specified "
                  "access point");
    db_cleanup_wifi_runtime();
    // Return -1 so app_main knows we failed
    return -1;
  }

  ESP_LOGI(TAG, "WiFi client mode enabled and connected!");
  ESP_ERROR_CHECK(esp_read_mac(LOCAL_MAC_ADDRESS, ESP_MAC_WIFI_STA));
  return 0;
}

/**
 * Enables or disables (via reboot) the Wi-Fi/BLE if the
 * DB_PARAM_DIS_RADIO_ON_ARM parameter is set. Not used during boot. Usually
 * called when arm state change of the autopilot is detected. As internal check
 * if the Wi-Fi is already enabled/disabled Wi-Fi must be inited first (done
 * during boot).
 *
 * This is handy for modes like BLE & WiFi AP which are used to configure the
 * drone and usually do not act as a long range telemetry link. In these cases
 * interference on the drone can be reduced by disabling the radio.
 *
 * @param enable_wifi True to enable the Wi-Fi and FALSE to disable it
 */
void db_set_radio_status(uint8_t enable_wifi) {
  if (DB_PARAM_DIS_RADIO_ON_ARM) { // check if the user enables that feature
    if (enable_wifi && DB_RADIO_IS_OFF) {
      ESP_LOGI(TAG, "Rebooting ESP32 to re-enable Wi-Fi/BLE");
      esp_restart(); // enable Wi-Fi/BLE by restarting ESP32 - an easy way to
                     // make sure all things are set up right
    } else if (!enable_wifi && !DB_RADIO_IS_OFF) {
      ESP_LOGI(TAG, "Disabling Wi-Fi/BLE");
      {
        if (esp_wifi_stop() == ESP_OK) { // disable WiFi
          DB_RADIO_IS_OFF = true;
        } else {
          ESP_LOGW(TAG, "db_set_radio_status tried to disable Wi-Fi. FAILED");
        }
      }
    }
  } else {
    // nothing to do
  }
}

/**
 * Write settings to non-volatile memory so they can be loaded on next startup.
 * The UDP clients are saved using a separate function since the "save"
 * operation is triggered by a separate button on the UI.
 */
void db_write_settings_to_nvs() {
  // print parameters to console for logging
  ESP_LOGI(TAG, "Trying to save parameters:");
  db_log_param_values(NULL);
  ESP_LOGI(TAG, "Saving to NVS %s", NVS_NAMESPACE);
  nvs_handle_t my_handle;
  ESP_ERROR_CHECK(nvs_open(NVS_NAMESPACE, NVS_READWRITE, &my_handle));
  db_param_write_all_params_nvs(&my_handle);
  ESP_ERROR_CHECK(nvs_commit(my_handle));
  nvs_close(my_handle);
  ESP_LOGI(TAG, "Finished saving operation.");
}

/**
 * Saves an udp client to the NVM so it can be automatically added on the next
 * boot. No need for the user to manually add it again. Only one UDP client can
 * be saved to the NVM.
 * @param new_db_udp_client The client to add to NVM. Must have IP and port set.
 * @param clear_client Set to true to remove the current client from NVM. In
 * that case the new_db_udp_client param will be ignored.
 */
void save_udp_client_to_nvm(struct db_udp_client_t *new_db_udp_client,
                            bool clear_client) {
  char ip[INET_ADDRSTRLEN];
  uint16_t port;
  if (!clear_client) {
    // convert addr to string
    char client_str[INET_ADDRSTRLEN + 6];
    inet_ntop(AF_INET, &(new_db_udp_client->udp_client.sin_addr), ip,
              INET_ADDRSTRLEN);
    port = ntohs(new_db_udp_client->udp_client.sin_port);
    snprintf(client_str, sizeof(client_str), "%s:%d", ip, port);
    ESP_LOGI(TAG, "Saving UDP client %s to NVS %s", client_str, NVS_NAMESPACE);
  } else {
    // clear client from NVM by setting string to empty "" and port to 0
    ip[0] = '\0';
    port = 0;
    ESP_LOGI(TAG, "Clearing UDP client from NVM");
  }

  nvs_handle my_handle;
  ESP_ERROR_CHECK(nvs_open(NVS_NAMESPACE, NVS_READWRITE, &my_handle));
  ESP_ERROR_CHECK(
      nvs_set_str(my_handle, (char *)db_param_udp_client_ip.db_name, ip));
  ESP_ERROR_CHECK(
      nvs_set_u16(my_handle, (char *)db_param_udp_client_port.db_name, port));

  ESP_ERROR_CHECK(nvs_commit(my_handle));
  nvs_close(my_handle);
}

/**
 * Read stored settings from internal storage including the saved UDP client.
 */
void db_read_settings_nvs() {
  nvs_handle my_handle;
  if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &my_handle) != ESP_OK) {
    ESP_LOGI(
        TAG,
        "NVS namespace not found. Using default values, setting up NVS...");
    ESP_ERROR_CHECK(nvs_flash_erase());
    ESP_ERROR_CHECK(nvs_flash_init());

    // Set all parameters to their default values when flash is empty
    db_param_reset_all();

    // Now save these defaults to NVS
    db_write_settings_to_nvs();

    // Print parameters to console for logging
    db_log_param_values("Initialized with default values:\n");
  } else {
    ESP_LOGI(TAG, "Reading settings from NVS");
    db_param_read_all_params_nvs(&my_handle);
    nvs_close(my_handle);

    // print parameters to console for logging
    db_log_param_values(NULL);

    // Check if we have a saved UDP client from the last session. Add it to the
    // known udp clients if there is one.
    if (strlen((char *)db_param_udp_client_ip.value.db_param_str.value) > 0 &&
        db_param_udp_client_port.value.db_param_u16.value != 0) {
      // there was a saved UDP client in the NVM from last session - add it to
      // the udp clients list
      ESP_LOGI(TAG, "Adding %s:%i to known UDP clients.",
               (char *)db_param_udp_client_ip.value.db_param_str.value,
               db_param_udp_client_port.value.db_param_u8.value);
      struct sockaddr_in new_sockaddr;
      memset(&new_sockaddr, 0, sizeof(new_sockaddr));
      new_sockaddr.sin_family = AF_INET;
      inet_pton(AF_INET,
                (char *)db_param_udp_client_ip.value.db_param_str.value,
                &new_sockaddr.sin_addr);
      new_sockaddr.sin_port =
          htons(db_param_udp_client_port.value.db_param_u16.value);
      struct db_udp_client_t new_udp_client = {
          .udp_client = new_sockaddr, .mac = {0, 0, 0, 0, 0, 0} // dummy MAC
      };
      bool save_to_nvm = false; // no need to save it to NVM again
      add_to_known_udp_clients(udp_conn_list, new_udp_client, save_to_nvm);
    } else {
      // no saved UDP client - do nothing
      ESP_LOGI(TAG, "No saved UDP client - skipping");
    }
  }
}

/**
 * Callback for a short press (<CONFIG_BUTTON_SHORT_PRESS_TIME_MS) of the
 * reset/boot button. Sets mode to WiFi access point mode with default password
 * "dronebridge" so user can check/change the config
 * @param arg
 */
void short_press_callback(void *arg, void *usr_data) {
  ESP_LOGW(TAG, "Short press detected setting wifi mode to access point with "
                "password: dronebridge");
  DB_RADIO_MODE_DESIGNATED =
      DB_WIFI_MODE_AP; // Do not directly change DB_PARAM_RADIO_MODE since it is
                       // not safe and constantly processed by other tasks. Save
                       // settings and reboot will assign
                       // DB_RADIO_MODE_DESIGNATED to DB_PARAM_RADIO_MODE.
  db_param_set_to_default(&db_param_ssid);
  db_param_set_to_default(&db_param_pass);
  db_write_settings_to_nvs();
  esp_restart();
}

/**
 * Callback for a long press (>CONFIG_BUTTON_LONG_PRESS_TIME_MS) of the
 * reset/boot button. Resets all settings to defaults.
 * @param arg
 */
void long_press_callback(void *arg, void *usr_data) {
  ESP_LOGW(TAG, "Reset triggered via GPIO %i. Resetting settings and rebooting",
           DB_RESET_PIN);
  DB_RADIO_MODE_DESIGNATED =
      DB_WIFI_MODE_AP; // Do not directly change DB_PARAM_RADIO_MODE since it is
                       // not safe and constantly processed by other tasks. Save
                       // settings and reboot will assign
                       // DB_RADIO_MODE_DESIGNATED to DB_PARAM_RADIO_MODE.
  db_param_reset_all();
  db_write_settings_to_nvs();
  esp_restart();
}

/**
 * Set up the boot button GPIO to reset entire ESP32 settings and to force a
 * reboot of the system
 */
void set_reset_trigger() {
  button_config_t gpio_btn_cfg = {
      .long_press_time = CONFIG_BUTTON_LONG_PRESS_TIME_MS,
      .short_press_time = CONFIG_BUTTON_SHORT_PRESS_TIME_MS,
  };
  button_gpio_config_t btn_gpio_cfg = {
      .gpio_num = DB_RESET_PIN,
      .active_level = 0,
  };
  button_handle_t gpio_btn = NULL;
  ESP_ERROR_CHECK(
      iot_button_new_gpio_device(&gpio_btn_cfg, &btn_gpio_cfg, &gpio_btn));
  if (NULL == gpio_btn) {
    ESP_LOGE(TAG, "Button create failed");
  } else {
    iot_button_register_cb(gpio_btn, BUTTON_SINGLE_CLICK, NULL,
                           short_press_callback, NULL);
    iot_button_register_cb(gpio_btn, BUTTON_LONG_PRESS_UP, NULL,
                           long_press_callback, NULL);
  }
}

/**
 * For simple debugging when serial via JTAG is enabled. Printed once control
 * module configured USB serial socket. Write settings to JTAG/USB, so we can
 * debug issues better
 */
void db_jtag_serial_info_print() {
  uint8_t *buffer = calloc(DB_PARAM_LOG_BUFFER_SIZE, sizeof(uint8_t));
  if (buffer == NULL) {
    ESP_LOGE(TAG, "Failed to allocate JTAG parameter log buffer");
    return;
  }

  const int len = db_param_print_values_to_buffer(buffer, DB_PARAM_LOG_BUFFER_SIZE);
  write_to_serial(buffer, len);
  free(buffer);
}

/**
 * 1. Checks if RF Switch is configured.
 * 2. Enables RF switch (necessary on some boards).
 * 3. Sets RF switch according to user setting.
 * Must be called before radio/Wi-Fi gets initialized!
 */
void db_configure_antenna() {
#if defined(CONFIG_DB_HAS_RF_SWITCH) && defined(CONFIG_DB_RF_SWITCH_GPIO) &&   \
    (CONFIG_DB_RF_SWITCH_GPIO != 0)
#ifdef CONFIG_DB_OFFICIAL_BOARD_1_X_C6
  gpio_set_direction(GPIO_NUM_3, GPIO_MODE_OUTPUT);
  gpio_set_level(GPIO_NUM_3, 0); // set to low to enable RF switching
#endif
  gpio_set_direction(CONFIG_DB_RF_SWITCH_GPIO, GPIO_MODE_OUTPUT);
  gpio_set_level(
      CONFIG_DB_RF_SWITCH_GPIO,
      DB_PARAM_EN_EXT_ANT); // set level to enable external/internal antenna
  ESP_LOGI(TAG, "External antenna usage: %i", DB_PARAM_EN_EXT_ANT);
#endif
}

/**
 * Main entry point.
 */
void app_main() {
  bool deeper_sta_connected = false;
  bool hardwired_sonar_selected = false;
  bool force_update_ap_mode = false;
  int boot_radio_mode = DB_WIFI_MODE_AP;
  s_deeper_sta_session_active = false;

  db_param_init_parameters();
  udp_conn_list =
      udp_client_list_create(); // http server functions and
                                // db_read_settings_nvs expect the list to exist
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);
  db_read_settings_nvs();
  force_update_ap_mode = db_consume_update_ap_mode_on_next_boot();
  DB_RADIO_MODE_DESIGNATED =
      DB_PARAM_RADIO_MODE; // must always match, mismatch only allowed when
                           // changed by user action and not rebooted, yet.
  db_configure_antenna();
  DB_ACTIVE_SONAR_SOURCE = DB_SONAR_SOURCE_NONE;
  boot_radio_mode = DB_PARAM_RADIO_MODE;

  if (force_update_ap_mode) {
    ESP_LOGW(TAG, "One-time update AP mode requested. Skipping normal radio "
                  "selection and booting a visible AP for maintenance.");
    boot_radio_mode = DB_WIFI_MODE_AP;
    db_init_wifi_apmode(DB_WIFI_MODE_AP);
    hardwired_sonar_selected = DB_PARAM_HARDWIRED_EN;
    DB_ACTIVE_SONAR_SOURCE = hardwired_sonar_selected
                                 ? DB_SONAR_SOURCE_HARDWIRED
                                 : DB_SONAR_SOURCE_NONE;
  } else {
  switch (boot_radio_mode) {
  case DB_WIFI_MODE_AP:
    if (DB_PARAM_DEEPER_EN) {
      char original_wifi_ssid[MAX_SSID_LEN] = {0};
      char original_wifi_pass[DB_PARAM_VALUE_MAXLEN] = {0};
      db_copy_cstr(original_wifi_ssid, sizeof(original_wifi_ssid),
                   DB_PARAM_WIFI_SSID);
      db_copy_cstr(original_wifi_pass, sizeof(original_wifi_pass),
                   DB_PARAM_PASS);
      ESP_LOGI(TAG, "Deeper Sonar Fallback Enabled. Attempting a single %lu ms "
                    "boot-time STA connection window before AP fallback...",
               (unsigned long)DB_DEEPER_BOOT_CONNECT_WINDOW_MS);
      db_copy_cstr(DB_PARAM_WIFI_SSID, db_param_ssid.value.db_param_str.max_len,
                   DB_PARAM_DEEPER_SSID);
      db_copy_cstr(DB_PARAM_PASS, db_param_pass.value.db_param_str.max_len,
                   DB_PARAM_DEEPER_PASS);

      if (db_init_wifi_clientmode(DB_DEEPER_BOOT_CONNECT_WINDOW_MS) < 0) {
        ESP_LOGE(TAG, "STA Mode failed (Deeper not found). Falling back to "
                      "normal AP Mode.");

        // Restore AP config that was temporarily overwritten for the Deeper
        // probe without depending on a second NVS read here.
        db_copy_cstr(DB_PARAM_WIFI_SSID,
                     db_param_ssid.value.db_param_str.max_len,
                     original_wifi_ssid);
        db_copy_cstr(DB_PARAM_PASS, db_param_pass.value.db_param_str.max_len,
                     original_wifi_pass);

        db_init_wifi_apmode(boot_radio_mode);
        hardwired_sonar_selected = true;
        DB_ACTIVE_SONAR_SOURCE = DB_SONAR_SOURCE_HARDWIRED;
        ESP_LOGI(TAG, "Deeper boot probe failed. Staying in AP mode and using "
                      "hardwired sonar until the next reboot.");
      } else {
        s_deeper_sta_session_active = true;
        db_copy_cstr(DB_PARAM_WIFI_SSID, db_param_ssid.value.db_param_str.max_len,
                     original_wifi_ssid);
        db_copy_cstr(DB_PARAM_PASS, db_param_pass.value.db_param_str.max_len,
                     original_wifi_pass);
        ESP_LOGI(TAG, "Deeper Fallback SUCCESS! We are now connected to the "
                      "Deeper Sonar AP.");
        deeper_sta_connected = true;
        DB_ACTIVE_SONAR_SOURCE = DB_SONAR_SOURCE_DEEPER;
        // Keep DB_PARAM_RADIO_MODE technically as AP so the WebUI doesn't
        // permanently change to STA
      }
    } else {
      db_init_wifi_apmode(DB_PARAM_RADIO_MODE);
      hardwired_sonar_selected = DB_PARAM_HARDWIRED_EN;
      DB_ACTIVE_SONAR_SOURCE = hardwired_sonar_selected
                                   ? DB_SONAR_SOURCE_HARDWIRED
                                   : DB_SONAR_SOURCE_NONE;
    }
    break;
  default:
    // Wi-Fi client mode with LR mode enabled
    if (db_init_wifi_clientmode(0) < 0) {
      ESP_LOGE(TAG, "Failed to init Wifi Client Mode");
      ESP_LOGI(TAG, "Falling back to AP mode");
      boot_radio_mode = DB_WIFI_MODE_AP;
      db_init_wifi_apmode(DB_WIFI_MODE_AP);
      hardwired_sonar_selected = DB_PARAM_HARDWIRED_EN;
      DB_ACTIVE_SONAR_SOURCE = hardwired_sonar_selected
                                   ? DB_SONAR_SOURCE_HARDWIRED
                                   : DB_SONAR_SOURCE_NONE;
    } else if (db_is_deeper_sta_target()) {
      s_deeper_sta_session_active = true;
      deeper_sta_connected = true;
      DB_ACTIVE_SONAR_SOURCE = DB_SONAR_SOURCE_DEEPER;
    } else {
      hardwired_sonar_selected = DB_PARAM_HARDWIRED_EN;
      DB_ACTIVE_SONAR_SOURCE = hardwired_sonar_selected
                                   ? DB_SONAR_SOURCE_HARDWIRED
                                   : DB_SONAR_SOURCE_NONE;
    }
    break;
  }
  }

  if (hardwired_sonar_selected) {
    danevi_sonar_init(DB_PARAM_SONAR_TX_GPIO, DB_PARAM_SONAR_RX_GPIO);
  } else if (DB_ACTIVE_SONAR_SOURCE == DB_SONAR_SOURCE_DEEPER) {
    ESP_LOGI(TAG, "Deeper selected at boot. Hardwired sonar stays off.");
  } else {
    ESP_LOGI(TAG, "No hardwired sonar source selected for this boot.");
  }

  {
    // mDNS + NetBIOS are useful in every remaining Wi-Fi mode (AP/STA).
    start_mdns_service();
    netbiosns_init();
    netbiosns_set_name("dronebridge");
  }
  if (init_fs() != ESP_OK) {
    ESP_LOGW(TAG, "Web filesystem is unavailable. Continuing with the embedded "
                  "OTA recovery page only.");
  }
  // Temporary stability hotfix:
  // keep the persistent log path disabled while the ESP is booted into the
  // Deeper STA session, so we can separate Deeper-mode regressions from the
  // newer flash-backed logging feature.
  bool enable_persistent_sonar_log = !deeper_sta_connected;
  if (enable_persistent_sonar_log) {
    if (db_sonar_log_init() != ESP_OK) {
      ESP_LOGW(TAG, "Persistent sonar log filesystem is unavailable. "
                    "Continuing without downloadable rolling logs.");
    }
    db_sonar_log_log_boot(DB_ACTIVE_SONAR_SOURCE, boot_radio_mode,
                          deeper_sta_connected, force_update_ap_mode,
                          db_is_web_fs_available());
  } else {
    ESP_LOGW(TAG, "Persistent sonar log temporarily disabled for this Deeper "
                  "STA boot while the stability hotfix is active.");
  }
  db_start_control_module();
  if (deeper_sta_connected) {
    deeper_udp_sonar_start();
  }

  db_timer_start_mavlink_heartbeat();
  db_timer_start_mavlink_radio_status();
  db_timer_start_mavlink_sonar();

  // DBB Companion brain: no-op in the open base (weak stub). A private brain
  // component, if linked, overrides these hooks at link time (see README).
  dbb_brain_init();

  {
    // The web dashboard is served in every remaining Wi-Fi mode (AP/STA).
    ESP_ERROR_CHECK(start_rest_server(CONFIG_WEB_MOUNT_POINT));
    ESP_LOGI(TAG, "Rest Server started");
    // Disable legacy support for DroneBridge communication module - no use case
    // for DroneBridge for ESP32 communication_module();
  }
  db_mark_running_ota_image_valid_if_needed();
  ESP_LOGI(TAG, "app_main finished initial setup");
}
