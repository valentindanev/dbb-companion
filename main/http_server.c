/*
 *   This file is part of DroneBridge: https://github.com/DroneBridge/ESP32
 *
 *   Copyright 2021 Wolfgang Christl
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

#include "http_server.h"

#include <db_parameters.h>
#include <string.h>
#include <fcntl.h>
#include <lwip/sockets.h>
#include <esp_chip_info.h>
#include <esp_app_format.h>
#include <esp_ota_ops.h>
#include "dbb_brain.h"
#include <esp_partition.h>
#include "esp_http_server.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_vfs.h"
#include "cJSON.h"
#include "db_sonar_log.h"
#include "danevi_sonar.h"
#include "deeper_udp_sonar.h"
#include "globals.h"
#include "main.h"
#include "db_serial.h"

#define TAG "DB_HTTP_REST"
#define REST_CHECK(a, str, goto_tag, ...)                                              \
    do                                                                                 \
    {                                                                                  \
        if (!(a))                                                                      \
        {                                                                              \
            ESP_LOGE(TAG, "%s(%d): " str, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
            goto goto_tag;                                                             \
        }                                                                              \
    } while (0)

#define FILE_PATH_MAX (ESP_VFS_PATH_MAX + 128)
#define SCRATCH_BUFSIZE (10240)
#define DB_HTTP_DEBUG_LOG_BUFFER_SIZE (1600)
#define DB_HTTP_SERVER_STACK_SIZE (8192)
#define DB_HTTP_OTA_RESPONSE_DELAY_MS (1500)

static const char *DB_HTTP_OTA_PORTAL_HTML =
    "<!DOCTYPE html>"
    "<html lang=\"en\">"
    "<head>"
    "<meta charset=\"utf-8\">"
    "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">"
    "<title>ESP32 Wireless Update</title>"
    "<style>"
    "body{font-family:Segoe UI,Arial,sans-serif;background:#0b1220;color:#e5eefb;margin:0;padding:24px;}"
    ".wrap{max-width:880px;margin:0 auto;}"
    ".card{background:#121b2d;border:1px solid #31425f;border-radius:14px;padding:20px;margin-bottom:18px;box-shadow:0 10px 30px rgba(0,0,0,.22);}"
    "h1,h2{margin-top:0;color:#f5f9ff;}p,li{line-height:1.55;}"
    "code{background:#08101d;padding:2px 6px;border-radius:6px;color:#8fd3ff;}"
    "input[type=file]{display:block;margin:10px 0 14px 0;color:#dbe7ff;}"
    "button,a.btn{display:inline-block;background:#2a68ff;color:#fff;border:none;border-radius:10px;padding:12px 16px;text-decoration:none;font-weight:600;cursor:pointer;margin-right:10px;margin-top:8px;}"
    "button.alt,a.btn.alt{background:#25324a;}button.warn{background:#d76b00;}"
    "button:disabled{opacity:.55;cursor:not-allowed;}"
    ".status{padding:12px 14px;border-radius:10px;background:#0d1524;border:1px solid #31425f;white-space:pre-wrap;}"
    "</style>"
    "</head>"
    "<body>"
    "<div class=\"wrap\">"
    "<div class=\"card\">"
    "<h1>Wireless Update & Recovery</h1>"
    "<p>This page lives inside the firmware image, not inside SPIFFS. That means it stays available even if the web partition is damaged, which gives the bait boat a built-in recovery path.</p>"
    "<div id=\"ota-info\" class=\"status\">Loading update status...</div>"
    "<div style=\"margin-top:14px;\">"
    "<a class=\"btn alt\" href=\"/\">Open Main UI</a>"
    "<button class=\"warn\" onclick=\"rebootIntoUpdateMode()\">Reboot Into AP Update Mode</button>"
    "</div>"
    "</div>"
    "<div class=\"card\">"
    "<h2>Firmware OTA</h2>"
    "<p>Upload <code>db_esp32.bin</code>. The image is written to the inactive OTA slot, the boot partition is switched, and the ESP restarts.</p>"
    "<input id=\"appFile\" type=\"file\" accept=\".bin\">"
    "<button onclick=\"uploadFile('/api/update/app','appFile','Firmware upload started. The ESP will reboot when the upload completes successfully.')\">Upload Firmware & Reboot</button>"
    "</div>"
    "<div class=\"card\">"
    "<h2>Web UI OTA</h2>"
    "<p>Upload <code>www.bin</code> if the SPIFFS web assets changed. If this upload is interrupted, refresh <code>/ota</code> after the ESP reboots and try again.</p>"
    "<input id=\"webFile\" type=\"file\" accept=\".bin\">"
    "<button onclick=\"uploadFile('/api/update/web','webFile','Web UI upload started. The ESP will reboot when the upload completes successfully.')\">Upload Web UI & Reboot</button>"
    "</div>"
    "<div class=\"card\">"
    "<h2>Status</h2>"
    "<div id=\"status\" class=\"status\">Idle.</div>"
    "</div>"
    "</div>"
    "<script>"
    "function setStatus(msg){document.getElementById('status').textContent=msg;}"
    "function line(label,value){return label+': '+value;}"
    "async function refreshInfo(){"
    " const res=await fetch('/api/update/info');"
    " if(!res.ok){throw new Error('Failed to load OTA info: '+res.status);}"
    " const info=await res.json();"
    " const lines=["
    "  line('Running partition',info.running_partition_label+' ('+info.running_partition_subtype+')'),"
    "  line('Boot partition',info.boot_partition_label+' ('+info.boot_partition_subtype+')'),"
    "  line('Next OTA target',info.next_update_partition_label+' ('+info.next_update_partition_subtype+')'),"
    "  line('Firmware version',info.running_app_version),"
    "  line('Firmware build',info.running_app_date+' '+info.running_app_time),"
    "  line('Web filesystem mounted',info.web_fs_available ? 'yes' : 'no'),"
    "  line('Web partition size',info.web_partition_size_bytes+' bytes')"
    " ];"
    " document.getElementById('ota-info').textContent=lines.join('\\n');"
    "}"
    "async function uploadFile(endpoint,inputId,startMessage){"
    " const input=document.getElementById(inputId);"
    " if(!input.files.length){setStatus('Select a .bin file first.');return;}"
    " const file=input.files[0];"
    " setStatus(startMessage+'\\nUploading '+file.name+' ('+file.size+' bytes)... Do not power off the boat.');"
    " const res=await fetch(endpoint,{method:'POST',headers:{'Content-Type':'application/octet-stream'},body:file});"
    " const text=await res.text();"
    " let message=text;"
    " try{const json=JSON.parse(text);if(json.msg){message=json.msg;}}catch(e){}"
    " if(!res.ok){throw new Error(message || ('Upload failed with status '+res.status));}"
    " setStatus(message || 'Upload finished. Device rebooting...');"
    "}"
    "async function rebootIntoUpdateMode(){"
    " setStatus('Scheduling one-time AP update mode and rebooting...');"
    " const res=await fetch('/api/update/boot-ap-once',{method:'POST'});"
    " const text=await res.text();"
    " let message=text;"
    " try{const json=JSON.parse(text);if(json.msg){message=json.msg;}}catch(e){}"
    " if(!res.ok){throw new Error(message || ('Failed with status '+res.status));}"
    " setStatus(message || 'Rebooting into AP update mode...');"
    "}"
    "refreshInfo().catch(err=>setStatus(err.message));"
    "</script>"
    "</body>"
    "</html>";

typedef struct rest_server_context {
    char base_path[ESP_VFS_PATH_MAX + 1];
    char scratch[SCRATCH_BUFSIZE];
} rest_server_context_t;

#define CHECK_FILE_EXTENSION(filename, ext) (strcasecmp(&filename[strlen(filename) - strlen(ext)], ext) == 0)

static bool db_http_runtime_has_sta(void) {
    wifi_mode_t mode = WIFI_MODE_NULL;
    return esp_wifi_get_mode(&mode) == ESP_OK &&
           (mode == WIFI_MODE_STA || mode == WIFI_MODE_APSTA);
}

static bool db_http_runtime_has_ap(void) {
    wifi_mode_t mode = WIFI_MODE_NULL;
    return esp_wifi_get_mode(&mode) == ESP_OK &&
           (mode == WIFI_MODE_AP || mode == WIFI_MODE_APSTA);
}

static const char *db_http_partition_label_or_unknown(const esp_partition_t *part) {
    if (part == NULL || part->label[0] == '\0') {
        return "<none>";
    }
    return part->label;
}

static void db_http_resp_sendstr_with_retry(httpd_req_t *req, const char *resp_str);

static void db_http_send_json_and_restart(httpd_req_t *req, const char *resp_str) {
    httpd_resp_set_type(req, "application/json");
    db_http_resp_sendstr_with_retry(req, resp_str);
    vTaskDelay(pdMS_TO_TICKS(DB_HTTP_OTA_RESPONSE_DELAY_MS));
    esp_restart();
}

/**
 * Sends a http response with retries
 * @param req Request Object
 * @param resp_str Request String
 */
static void db_http_resp_sendstr_with_retry(httpd_req_t *req, const char *resp_str) {
    for (int i = 0; i < 3; i++) {
        if (httpd_resp_sendstr(req, resp_str) == ESP_OK) {
            return;
        }
        ESP_LOGW(TAG, "httpd_resp_sendstr failed, retrying %d/3", i + 1);
        vTaskDelay(pdMS_TO_TICKS(150));
    }
    ESP_LOGE(TAG, "httpd_resp_sendstr failed after 3 retries");
}

/**
 * Set HTTP response content type according to file extension
 */
static esp_err_t set_content_type_from_file(httpd_req_t *req, const char *filepath) {
    const char *type = "text/plain";
    if (CHECK_FILE_EXTENSION(filepath, ".html")) {
        type = "text/html";
    } else if (CHECK_FILE_EXTENSION(filepath, ".js")) {
        type = "application/javascript";
    } else if (CHECK_FILE_EXTENSION(filepath, ".css")) {
        type = "text/css";
    } else if (CHECK_FILE_EXTENSION(filepath, ".png")) {
        type = "image/png";
    } else if (CHECK_FILE_EXTENSION(filepath, ".ico")) {
        type = "image/x-icon";
    } else if (CHECK_FILE_EXTENSION(filepath, ".svg")) {
        type = "text/xml";
    }
    return httpd_resp_set_type(req, type);
}

static esp_err_t db_http_send_file_with_type(httpd_req_t *req,
                                             const char *filepath,
                                             const char *content_type) {
    rest_server_context_t *rest_context = (rest_server_context_t *)req->user_ctx;
    if (filepath == NULL || rest_context == NULL) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                            "invalid file request");
        return ESP_FAIL;
    }

    int fd = open(filepath, O_RDONLY, 0);
    if (fd == -1) {
        ESP_LOGE(TAG, "Failed to open file : %s", filepath);
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "file not found");
        return ESP_FAIL;
    }

    if (content_type != NULL) {
        httpd_resp_set_type(req, content_type);
    } else {
        set_content_type_from_file(req, filepath);
    }

    char *chunk = rest_context->scratch;
    ssize_t read_bytes = 0;
    do {
        read_bytes = read(fd, chunk, SCRATCH_BUFSIZE);
        if (read_bytes == -1) {
            ESP_LOGE(TAG, "Failed to read file : %s", filepath);
            close(fd);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                "failed to read file");
            return ESP_FAIL;
        } else if (read_bytes > 0) {
            if (httpd_resp_send_chunk(req, chunk, read_bytes) != ESP_OK) {
                close(fd);
                ESP_LOGW(TAG, "Client disconnected while sending file: %s",
                         filepath);
                return ESP_OK;
            }
        }
    } while (read_bytes > 0);

    close(fd);
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

/**
 * Send HTTP response with the contents of the requested file
 */
static esp_err_t rest_common_get_handler(httpd_req_t *req) {
    char filepath[FILE_PATH_MAX];

    rest_server_context_t *rest_context = (rest_server_context_t *) req->user_ctx;
    if (!db_is_web_fs_available()) {
        if (strcmp(req->uri, "/") == 0 || strcmp(req->uri, "/index.html") == 0) {
            httpd_resp_set_type(req, "text/html");
            return httpd_resp_send(req, DB_HTTP_OTA_PORTAL_HTML,
                                   HTTPD_RESP_USE_STRLEN);
        }
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND,
                            "web filesystem unavailable; open /ota for recovery");
        return ESP_FAIL;
    }
    strlcpy(filepath, rest_context->base_path, sizeof(filepath));
    if (req->uri[strlen(req->uri) - 1] == '/') {
        strlcat(filepath, "/index.html", sizeof(filepath));
    } else {
        strlcat(filepath, req->uri, sizeof(filepath));
    }
    return db_http_send_file_with_type(req, filepath, NULL);
}

/**
 * Process incoming settings change and reply with HTTP success message
 * @param req
 * @return ESP error code
 */
static esp_err_t settings_post_handler(httpd_req_t *req) {
    int total_len = req->content_len;
    int cur_len = 0;
    char *buf = ((rest_server_context_t *) (req->user_ctx))->scratch;
    int received = 0;
    if (total_len >= SCRATCH_BUFSIZE) {
        // This should be HTTPD_414_PAYLOAD_TOO_LARGE, but that's not
        // implemented yet, so use 400 Bad Request instead.
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "content too long");
        return ESP_FAIL;
    }
    while (cur_len < total_len) {
        received = httpd_req_recv(req, buf + cur_len, total_len - cur_len);
        if (received <= 0) {
            /* Respond with 500 Internal Server Error */
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to post control value");
            return ESP_FAIL;
        }
        cur_len += received;
    }
    buf[total_len] = '\0';

    cJSON *root = cJSON_Parse(buf);
    if (root == NULL) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid json");
        return ESP_FAIL;
    }
    db_param_read_all_params_json(root);
    db_write_settings_to_nvs();
    ESP_LOGI(TAG, "Settings changed!");

    cJSON_Delete(root);
    const char *resp_str = "{\n"
                           "    \"status\": \"success\",\n"
                           "    \"msg\": \"Settings changed! Rebooting ...\"\n"
                           "  }";
    db_http_resp_sendstr_with_retry(req, resp_str);

    vTaskDelay(pdMS_TO_TICKS(2000));  // wait to allow the website displaying the success message
    esp_restart();
    return ESP_OK;
}

/**
 * Process incoming UDP connection add request. Only one IPv4 connection can be added at a time
 * Expecting JSON in the form of:
 * {
 *   "ip": "XXX.XXX.XXX.XXX",
 *   "port": 452
 * }
 * @param req
 * @return
 */
static esp_err_t settings_clients_udp_post(httpd_req_t *req) {
    int total_len = req->content_len;
    int cur_len = 0;
    char *buf = ((rest_server_context_t *) (req->user_ctx))->scratch;
    int received = 0;
    if (total_len >= SCRATCH_BUFSIZE) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "content too long");
        return ESP_FAIL;
    }
    while (cur_len < total_len) {
        received = httpd_req_recv(req, buf + cur_len, total_len - cur_len);
        if (received <= 0) {
            /* Respond with 500 Internal Server Error */
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to post control value");
            return ESP_FAIL;
        }
        cur_len += received;
    }
    buf[total_len] = '\0';

    // Obtain & process JSON from request
    cJSON *root = cJSON_Parse(buf);
    if (root == NULL) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid json");
        return ESP_FAIL;
    }

    int new_udp_port = 0;
    char new_ip[IP4ADDR_STRLEN_MAX] = {0};
    uint8_t save_to_nvm = false;
    cJSON *json = cJSON_GetObjectItem(root, (char *) db_param_udp_client_ip.db_name);
    if (cJSON_IsString(json) && json->valuestring != NULL) {
        strncpy(new_ip, json->valuestring, sizeof(new_ip) - 1);
    }
    new_ip[IP4ADDR_STRLEN_MAX-1] = '\0';    // to remove warning and to be sure
    json = cJSON_GetObjectItem(root, (char *) db_param_udp_client_port.db_name);
    if (cJSON_IsNumber(json)) {
        new_udp_port = json->valueint;
    }
    json = cJSON_GetObjectItem(root, "save");
    if (json && cJSON_IsBool(json)) {
        if(cJSON_IsTrue(json)) {
            save_to_nvm = true;
        } else {
            save_to_nvm = false;
        }
    } else {}

    if (!is_valid_ip4(new_ip) || new_udp_port < 1 || new_udp_port > UINT16_MAX) {
        cJSON_Delete(root);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid udp client");
        return ESP_FAIL;
    }

    // populate the UDP connections list with a new connection
    struct sockaddr_in new_sockaddr;
    memset(&new_sockaddr, 0, sizeof(new_sockaddr));
    new_sockaddr.sin_family = AF_INET;
    inet_pton(AF_INET, new_ip, &new_sockaddr.sin_addr);
    new_sockaddr.sin_port = htons(new_udp_port);
    struct db_udp_client_t new_udp_client = {
            .udp_client = new_sockaddr,
            .mac = {0, 0, 0, 0, 0, 0}   // dummy MAC
    };
    // udp_conn_list is initialized as the very first thing during startup - we expect it to be there
    bool success = add_to_known_udp_clients(udp_conn_list, new_udp_client, save_to_nvm);

    // Clean up
    cJSON_Delete(root);
    if (success) {
        httpd_resp_sendstr(req, "{\n"
                                "    \"status\": \"success\",\n"
                                "    \"msg\": \"Added UDP connection!\"\n"
                                "  }");
    } else {
        httpd_resp_sendstr(req, "{\n"
                                "    \"status\": \"failed\",\n"
                                "    \"msg\": \"Failed to add UDP connection!\"\n"
                                "  }");
    }

    return ESP_OK;
}

/**
 * Process a request that shall clear all active UDP connections.
 * ESP32 will remove all maintained UDP connections from its internal list and UDP clients will have to register again.
 * This also clears the one UDP client connection that gets saved to NVM.
 *
 * @param req
 * @return ESP_OK if all was good. Else ESP_FAIL
 */
static esp_err_t settings_clients_clear_udp_get(httpd_req_t *req) {
    for (int i = 0; i < udp_conn_list->size; ++i) {
        memset(&udp_conn_list->db_udp_clients[i], 0, sizeof(struct db_udp_client_t));
    }
    udp_conn_list->size = 0;
    ESP_LOGI(TAG, "Removed all UDP clients from list!");
    // Clear saved client as well. Pass any client since it will be ignored as long as clear_client is set to true.
    save_udp_client_to_nvm(&udp_conn_list->db_udp_clients[0], true);
    db_http_resp_sendstr_with_retry(req, "{\n"
                                         "    \"status\": \"success\",\n"
                                         "    \"msg\": \"Cleared UDP clients!\"\n"
                                         "  }");
    return ESP_OK;
}

/**
 * Returns build information esp-idf version and build version
 * @param req
 * @return
 */
static esp_err_t system_info_get_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "application/json");
    cJSON *root = cJSON_CreateObject();
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    cJSON_AddStringToObject(root, "idf_version", IDF_VER);
    cJSON_AddNumberToObject(root, "db_build_version", DB_BUILD_VERSION);
    cJSON_AddNumberToObject(root, "major_version", DB_MAJOR_VERSION);
    cJSON_AddNumberToObject(root, "minor_version", DB_MINOR_VERSION);
    cJSON_AddNumberToObject(root, "patch_version", DB_PATCH_VERSION);
    cJSON_AddStringToObject(root, "maturity_version", DB_MATURITY_VERSION);
    cJSON_AddNumberToObject(root, "esp_chip_model", chip_info.model);
    cJSON_AddNumberToObject(root, "has_rf_switch", DB_HAS_RF_SWITCH);
    char mac_str[18];
    sprintf(mac_str, "%02X:%02X:%02X:%02X:%02X:%02X",
            LOCAL_MAC_ADDRESS[0], LOCAL_MAC_ADDRESS[1], LOCAL_MAC_ADDRESS[2], LOCAL_MAC_ADDRESS[3], LOCAL_MAC_ADDRESS[4], LOCAL_MAC_ADDRESS[5]);
    cJSON_AddStringToObject(root, "esp_mac", mac_str);
#ifdef CONFIG_DB_SERIAL_OPTION_JTAG
    cJSON_AddNumberToObject(root, "serial_via_JTAG", 1);
#else
    cJSON_AddNumberToObject(root, "serial_via_JTAG", 0);
#endif
    const char *sys_info = cJSON_Print(root);
    db_http_resp_sendstr_with_retry(req, sys_info);
    free((void *) sys_info);
    cJSON_Delete(root);
    return ESP_OK;
}

/**
 * Returns a JSON containing read bytes from UART, number of connected TCP connections and UDP broadcasts as well as the
 * current IP address of the ESP32
 * @param req
 * @return ESP_OK on successfully sending the http request
 */
static esp_err_t system_stats_get_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "application/json");
    cJSON *root = cJSON_CreateObject();
    bool runtime_sta = db_http_runtime_has_sta();
    bool runtime_ap = db_http_runtime_has_ap();
    danevi_sonar_snapshot_t hardwired_snapshot = {0};
    deeper_udp_snapshot_t deeper_snapshot = {0};
    char *hardwired_debug_log = calloc(1, DB_HTTP_DEBUG_LOG_BUFFER_SIZE);
    char *deeper_debug_log = calloc(1, DB_HTTP_DEBUG_LOG_BUFFER_SIZE);

    if (root == NULL || hardwired_debug_log == NULL || deeper_debug_log == NULL) {
        free(hardwired_debug_log);
        free(deeper_debug_log);
        cJSON_Delete(root);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                            "out of memory");
        return ESP_FAIL;
    }

    danevi_sonar_get_debug_log(hardwired_debug_log,
                               DB_HTTP_DEBUG_LOG_BUFFER_SIZE);
    danevi_sonar_get_snapshot(&hardwired_snapshot);
    db_get_deeper_debug_log(deeper_debug_log, DB_HTTP_DEBUG_LOG_BUFFER_SIZE);
    deeper_udp_sonar_get_snapshot(&deeper_snapshot);
    cJSON_AddNumberToObject(root, "read_bytes", serial_total_byte_count);
    cJSON_AddNumberToObject(root, "serial_dec_mav_msgs", serial_total_decoded_mav_msgs);
    cJSON_AddNumberToObject(root, "tcp_connected", num_connected_tcp_clients);
    cJSON_AddNumberToObject(root, "udp_connected", udp_conn_list->size);
    cJSON_AddNumberToObject(root, "active_sonar_source", DB_ACTIVE_SONAR_SOURCE);
    cJSON_AddStringToObject(root, "hardwired_debug", hardwired_debug_log);
    cJSON_AddNumberToObject(root, "hardwired_depth_mm",
                            hardwired_snapshot.has_distance
                                ? hardwired_snapshot.depth_mm
                                : -1);
    cJSON_AddNumberToObject(root, "hardwired_sample_age_ms",
                            hardwired_snapshot.sample_age_ms);
    cJSON_AddNumberToObject(root, "hardwired_raw_depth_mm",
                            hardwired_snapshot.has_raw_distance
                                ? hardwired_snapshot.raw_depth_mm
                                : -1);
    cJSON_AddNumberToObject(root, "hardwired_raw_sample_age_ms",
                            hardwired_snapshot.raw_sample_age_ms);
    cJSON_AddNumberToObject(root, "hardwired_last_good_depth_mm",
                            hardwired_snapshot.has_last_good_distance
                                ? hardwired_snapshot.last_good_depth_mm
                                : -1);
    cJSON_AddNumberToObject(root, "hardwired_last_good_sample_age_ms",
                            hardwired_snapshot.last_good_sample_age_ms);
    cJSON_AddNumberToObject(root, "hardwired_zero_run_active",
                            hardwired_snapshot.zero_run_active ? 1 : 0);
    cJSON_AddNumberToObject(root, "hardwired_zero_run_age_ms",
                            hardwired_snapshot.zero_run_age_ms);
    cJSON_AddNumberToObject(root, "hardwired_consecutive_zero_frames",
                            hardwired_snapshot.consecutive_zero_frames);
    cJSON_AddNumberToObject(root, "hardwired_zero_filter_holding",
                            hardwired_snapshot.zero_filter_holding_last_good
                                ? 1
                                : 0);
    cJSON_AddStringToObject(root, "deeper_debug", deeper_debug_log);
    cJSON_AddNumberToObject(root, "deeper_depth_mm",
                            deeper_snapshot.has_depth ? deeper_snapshot.depth_mm
                                                      : -1);
    cJSON_AddNumberToObject(root, "deeper_temp_c_tenths",
                            deeper_snapshot.has_temperature
                                ? deeper_snapshot.temperature_c_tenths
                                : -100000);
    cJSON_AddNumberToObject(root, "deeper_satellites",
                            deeper_snapshot.has_satellites
                                ? deeper_snapshot.satellites
                                : -1);
    cJSON_AddNumberToObject(root, "deeper_gps_fix",
                            deeper_snapshot.has_satellites &&
                                    deeper_snapshot.gps_fix_valid
                                ? 1
                                : 0);
    cJSON_AddNumberToObject(root, "deeper_has_coordinates",
                            deeper_snapshot.has_coordinates ? 1 : 0);
    cJSON_AddNumberToObject(root, "deeper_latitude_deg",
                            deeper_snapshot.has_coordinates
                                ? deeper_snapshot.latitude_deg
                                : 0.0);
    cJSON_AddNumberToObject(root, "deeper_longitude_deg",
                            deeper_snapshot.has_coordinates
                                ? deeper_snapshot.longitude_deg
                                : 0.0);
    cJSON_AddNumberToObject(root, "deeper_sample_age_ms",
                            deeper_snapshot.newest_sample_age_ms);
    // add IP:PORT info on connected UDP clients
    cJSON *udp_clients = cJSON_CreateArray();
    for (int i = 0; i < udp_conn_list->size; i++) {
        char ip_string[INET_ADDRSTRLEN];
        char ip_port_string[INET_ADDRSTRLEN+10];
        inet_ntop(AF_INET, &(udp_conn_list->db_udp_clients[i].udp_client.sin_addr), ip_string, INET_ADDRSTRLEN);
        sprintf(ip_port_string, "%s:%d", ip_string, htons (udp_conn_list->db_udp_clients[i].udp_client.sin_port));
        cJSON_AddItemToArray(udp_clients, cJSON_CreateString(ip_port_string));
    }
    cJSON_AddItemToObject(root, "udp_clients", udp_clients);
    // add RSSI and IP info
    if (runtime_sta) {
        cJSON_AddStringToObject(root, "current_client_ip", CURRENT_CLIENT_IP);
        cJSON_AddNumberToObject(root, "esp_rssi", db_esp_signal_quality.air_rssi);
    } else if (runtime_ap && DB_PARAM_RADIO_MODE == DB_WIFI_MODE_AP) {
        cJSON *sta_array = cJSON_AddArrayToObject(root, "connected_sta");
        for (int i = 0; i < wifi_sta_list.num; i++) {
            cJSON *connected_stations_status = cJSON_CreateObject();
            char mac_str[18];
            sprintf(mac_str, "%02X:%02X:%02X:%02X:%02X:%02X",
                    wifi_sta_list.sta[i].mac[0], wifi_sta_list.sta[i].mac[1], wifi_sta_list.sta[i].mac[2],
                    wifi_sta_list.sta[i].mac[3], wifi_sta_list.sta[i].mac[4], wifi_sta_list.sta[i].mac[5]);
            cJSON_AddStringToObject(connected_stations_status, "sta_mac", mac_str);
            cJSON_AddNumberToObject(connected_stations_status, "sta_rssi", wifi_sta_list.sta[i].rssi);
            cJSON_AddItemToArray(sta_array, connected_stations_status);
        }
    } else {
        // other modes like ESP-NOW do not activate HTTP server so do nothing
    }
    const char *sys_info = cJSON_Print(root);
    db_http_resp_sendstr_with_retry(req, sys_info);
    free((void *) sys_info);
    free(hardwired_debug_log);
    free(deeper_debug_log);
    cJSON_Delete(root);
    return ESP_OK;
}

/**
 * Returns a JSON containing all active UDP connections that the ESP32 sends to
 * @param req
 * @return ESP_OK on successfully sending the http request
 */
static esp_err_t system_clients_get_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "application/json");
    cJSON *root = cJSON_CreateObject();
//    cJSON *tcp_clients = cJSON_CreateArray();
//    for (int i = 0; i < udp_conn_list->size; i++) {
//        //TODO: Save the TCP IPs and port during accept and put them here into a JSON
//    }
//    cJSON_AddItemToObject(root, "tcp_clients", tcp_clients);

    cJSON *udp_clients = cJSON_CreateArray();
    for (int i = 0; i < udp_conn_list->size; i++) {
        char ip_string[INET_ADDRSTRLEN];
        char ip_port_string[INET_ADDRSTRLEN+10];
        inet_ntop(AF_INET, &(udp_conn_list->db_udp_clients[i].udp_client.sin_addr), ip_string, INET_ADDRSTRLEN);
        sprintf(ip_port_string, "%s:%d", ip_string, htons (udp_conn_list->db_udp_clients[i].udp_client.sin_port));
        cJSON_AddItemToArray(udp_clients, cJSON_CreateString(ip_port_string));
    }
    cJSON_AddItemToObject(root, "udp_clients", udp_clients);

    const char *sys_info = cJSON_Print(root);
    db_http_resp_sendstr_with_retry(req, sys_info);
    free((void *) sys_info);
    cJSON_Delete(root);
    return ESP_OK;
}

/**
 * Respond with all internally known settings
 * @param req
 * @return ESP_OK on successfully sending the http request
 */
static esp_err_t settings_get_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "application/json");
    cJSON *root = cJSON_CreateObject();
    db_param_write_all_params_json(root);
    const char *sys_info = cJSON_Print(root);
    db_http_resp_sendstr_with_retry(req, sys_info);
    free((void *) sys_info);
    cJSON_Delete(root);
    return ESP_OK;
}

static esp_err_t ota_portal_get_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, DB_HTTP_OTA_PORTAL_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t update_info_get_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "application/json");
    cJSON *root = cJSON_CreateObject();
    const esp_partition_t *running = esp_ota_get_running_partition();
    const esp_partition_t *boot = esp_ota_get_boot_partition();
    const esp_partition_t *next = esp_ota_get_next_update_partition(NULL);
    const esp_partition_t *web = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_SPIFFS,
        DB_WEB_PARTITION_LABEL);
    esp_app_desc_t running_desc;

    cJSON_AddStringToObject(root, "running_partition_label",
                            db_http_partition_label_or_unknown(running));
    cJSON_AddNumberToObject(root, "running_partition_subtype",
                            running ? running->subtype : -1);
    cJSON_AddStringToObject(root, "boot_partition_label",
                            db_http_partition_label_or_unknown(boot));
    cJSON_AddNumberToObject(root, "boot_partition_subtype",
                            boot ? boot->subtype : -1);
    cJSON_AddStringToObject(root, "next_update_partition_label",
                            db_http_partition_label_or_unknown(next));
    cJSON_AddNumberToObject(root, "next_update_partition_subtype",
                            next ? next->subtype : -1);
    cJSON_AddNumberToObject(root, "web_fs_available",
                            db_is_web_fs_available() ? 1 : 0);
    cJSON_AddNumberToObject(root, "web_partition_size_bytes",
                            web ? web->size : 0);

    if (running != NULL &&
        esp_ota_get_partition_description(running, &running_desc) == ESP_OK) {
        cJSON_AddStringToObject(root, "running_app_version",
                                running_desc.version);
        cJSON_AddStringToObject(root, "running_app_date", running_desc.date);
        cJSON_AddStringToObject(root, "running_app_time", running_desc.time);
    } else {
        cJSON_AddStringToObject(root, "running_app_version", "unknown");
        cJSON_AddStringToObject(root, "running_app_date", __DATE__);
        cJSON_AddStringToObject(root, "running_app_time", __TIME__);
    }

    const char *sys_info = cJSON_Print(root);
    db_http_resp_sendstr_with_retry(req, sys_info);
    free((void *)sys_info);
    cJSON_Delete(root);
    return ESP_OK;
}

static esp_err_t update_boot_ap_once_post_handler(httpd_req_t *req) {
    esp_err_t err = db_request_update_ap_mode_on_next_boot();
    if (err != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                            "failed to schedule update AP mode");
        return ESP_FAIL;
    }

    db_http_send_json_and_restart(
        req,
        "{\n"
        "  \"status\": \"success\",\n"
        "  \"msg\": \"One-time AP update mode scheduled. Rebooting now...\"\n"
        "}");
    return ESP_OK;
}

static esp_err_t update_app_post_handler(httpd_req_t *req) {
    rest_server_context_t *rest_context = (rest_server_context_t *)req->user_ctx;
    const esp_partition_t *update_partition =
        esp_ota_get_next_update_partition(NULL);
    esp_ota_handle_t update_handle = 0;
    bool checked_header = false;
    int remaining = req->content_len;

    if (update_partition == NULL) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                            "no OTA partition available");
        return ESP_FAIL;
    }

    if (req->content_len <= 0 || req->content_len > (int)update_partition->size) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                            "firmware size does not fit the OTA partition");
        return ESP_FAIL;
    }

    esp_err_t err =
        esp_ota_begin(update_partition, OTA_WITH_SEQUENTIAL_WRITES, &update_handle);
    if (err != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                            esp_err_to_name(err));
        return ESP_FAIL;
    }

    while (remaining > 0) {
        int to_read = remaining > SCRATCH_BUFSIZE ? SCRATCH_BUFSIZE : remaining;
        int received = httpd_req_recv(req, rest_context->scratch, to_read);
        if (received <= 0) {
            esp_ota_abort(update_handle);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                "firmware upload interrupted");
            return ESP_FAIL;
        }

        if (!checked_header) {
            if ((uint8_t)rest_context->scratch[0] != ESP_IMAGE_HEADER_MAGIC) {
                esp_ota_abort(update_handle);
                httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                    "uploaded file is not a valid ESP32 app image");
                return ESP_FAIL;
            }
            checked_header = true;
        }

        err = esp_ota_write(update_handle, rest_context->scratch, received);
        if (err != ESP_OK) {
            esp_ota_abort(update_handle);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                esp_err_to_name(err));
            return ESP_FAIL;
        }
        remaining -= received;
    }

    err = esp_ota_end(update_handle);
    if (err != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                            esp_err_to_name(err));
        return ESP_FAIL;
    }

    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                            esp_err_to_name(err));
        return ESP_FAIL;
    }

    db_http_send_json_and_restart(
        req,
        "{\n"
        "  \"status\": \"success\",\n"
        "  \"msg\": \"Firmware upload complete. Boot partition switched to the new OTA slot. Rebooting now...\"\n"
        "}");
    return ESP_OK;
}

static esp_err_t update_web_post_handler(httpd_req_t *req) {
    rest_server_context_t *rest_context = (rest_server_context_t *)req->user_ctx;
    const esp_partition_t *web_partition = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_SPIFFS,
        DB_WEB_PARTITION_LABEL);
    size_t write_offset = 0;
    int remaining = req->content_len;

    if (web_partition == NULL) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                            "web partition not found");
        return ESP_FAIL;
    }

    if (req->content_len != (int)web_partition->size) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                            "web image size must match the SPIFFS partition size");
        return ESP_FAIL;
    }

    esp_err_t err = db_unmount_web_fs();
    if (err != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                            "failed to unmount web filesystem");
        return ESP_FAIL;
    }

    err = esp_partition_erase_range(web_partition, 0, web_partition->size);
    if (err != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                            esp_err_to_name(err));
        return ESP_FAIL;
    }

    while (remaining > 0) {
        int to_read = remaining > SCRATCH_BUFSIZE ? SCRATCH_BUFSIZE : remaining;
        int received = httpd_req_recv(req, rest_context->scratch, to_read);
        if (received <= 0) {
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                "web UI upload interrupted; refresh /ota and retry");
            return ESP_FAIL;
        }

        err = esp_partition_write(web_partition, write_offset,
                                  rest_context->scratch, received);
        if (err != ESP_OK) {
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                esp_err_to_name(err));
            return ESP_FAIL;
        }

        write_offset += received;
        remaining -= received;
    }

    db_http_send_json_and_restart(
        req,
        "{\n"
        "  \"status\": \"success\",\n"
        "  \"msg\": \"Web UI partition updated successfully. Rebooting now...\"\n"
        "}");
    return ESP_OK;
}

static esp_err_t sonar_log_status_get_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "application/json");

    db_sonar_log_status_t status = {0};
    db_sonar_log_get_status(&status);

    cJSON *root = cJSON_CreateObject();
    if (root == NULL) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                            "out of memory");
        return ESP_FAIL;
    }

    cJSON_AddNumberToObject(root, "mounted", status.mounted ? 1 : 0);
    cJSON_AddNumberToObject(root, "partition_total_bytes",
                            status.partition_total_bytes);
    cJSON_AddNumberToObject(root, "partition_used_bytes",
                            status.partition_used_bytes);
    cJSON_AddNumberToObject(root, "log_file_bytes", status.log_file_bytes);
    cJSON_AddNumberToObject(root, "max_log_file_bytes",
                            status.max_log_file_bytes);
    cJSON_AddNumberToObject(root, "trim_to_bytes", status.trim_to_bytes);
    cJSON_AddNumberToObject(root, "compaction_count",
                            status.compaction_count);

    const char *resp = cJSON_Print(root);
    db_http_resp_sendstr_with_retry(req, resp);
    free((void *)resp);
    cJSON_Delete(root);
    return ESP_OK;
}

static esp_err_t sonar_log_get_handler(httpd_req_t *req) {
    db_sonar_log_status_t status = {0};
    db_sonar_log_get_status(&status);
    if (!status.mounted) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                            "persistent sonar log unavailable");
        return ESP_FAIL;
    }

    if (status.log_file_bytes == 0) {
        httpd_resp_set_type(req, "text/plain");
        return httpd_resp_send(req,
                               "No persistent sonar log entries yet.\n",
                               HTTPD_RESP_USE_STRLEN);
    }

    return db_http_send_file_with_type(req, DB_SONAR_LOG_FILE_PATH,
                                       "text/plain");
}

static esp_err_t sonar_log_download_get_handler(httpd_req_t *req) {
    db_sonar_log_status_t status = {0};
    db_sonar_log_get_status(&status);
    if (!status.mounted) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                            "persistent sonar log unavailable");
        return ESP_FAIL;
    }

    httpd_resp_set_hdr(req, "Content-Disposition",
                       "attachment; filename=\"sonar-log.txt\"");
    if (status.log_file_bytes == 0) {
        httpd_resp_set_type(req, "text/plain");
        return httpd_resp_send(req,
                               "No persistent sonar log entries yet.\n",
                               HTTPD_RESP_USE_STRLEN);
    }

    return db_http_send_file_with_type(req, DB_SONAR_LOG_FILE_PATH,
                                       "text/plain");
}

static esp_err_t sonar_log_delete_handler(httpd_req_t *req) {
    esp_err_t err = db_sonar_log_clear();
    if (err != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                            "failed to clear persistent sonar log");
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "application/json");
    db_http_resp_sendstr_with_retry(req,
                                    "{\n"
                                    "  \"status\": \"success\",\n"
                                    "  \"msg\": \"Persistent sonar log cleared.\"\n"
                                    "}");
    return ESP_OK;
}

esp_err_t start_rest_server(const char *base_path) {
    REST_CHECK(base_path, "wrong base path", err);
    rest_server_context_t *rest_context = calloc(1, sizeof(rest_server_context_t));
    REST_CHECK(rest_context, "No memory for rest context", err);
    strlcpy(rest_context->base_path, base_path, sizeof(rest_context->base_path));

    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.uri_match_fn = httpd_uri_match_wildcard;
    config.max_uri_handlers = 24;
    config.stack_size = DB_HTTP_SERVER_STACK_SIZE;
    config.max_open_sockets = 12;      // raised with LWIP_MAX_SOCKETS=24 (BLE off freed the RAM) — comfortable multi-browser headroom
    config.lru_purge_enable = true;    // pool full -> recycle the stalest idle connection instead of rejecting (fixes the blank page on a 2nd/3rd client)

    ESP_LOGI(TAG, "Starting HTTP Server");
    REST_CHECK(httpd_start(&server, &config) == ESP_OK, "Start server failed", err_start);

    /* URI handler for fetching system info */
    httpd_uri_t system_info_get_uri = {
            .uri = "/api/system/info",
            .method = HTTP_GET,
            .handler = system_info_get_handler,
            .user_ctx = rest_context
    };
    httpd_register_uri_handler(server, &system_info_get_uri);

    /* URI handler for fetching client connection info */
    httpd_uri_t system_clients_get_uri = {
            .uri = "/api/system/clients",
            .method = HTTP_GET,
            .handler = system_clients_get_handler,
            .user_ctx = rest_context
    };
    httpd_register_uri_handler(server, &system_clients_get_uri);

    /* URI handler for fetching system info */
    httpd_uri_t system_stats_get_uri = {
            .uri = "/api/system/stats",
            .method = HTTP_GET,
            .handler = system_stats_get_handler,
            .user_ctx = rest_context
    };
    httpd_register_uri_handler(server, &system_stats_get_uri);

    /* URI handler for fetching settings data */
    httpd_uri_t settings_get_uri = {
            .uri = "/api/settings",
            .method = HTTP_GET,
            .handler = settings_get_handler,
            .user_ctx = rest_context
    };
    httpd_register_uri_handler(server, &settings_get_uri);

    httpd_uri_t settings_post_uri = {
            .uri = "/api/settings",
            .method = HTTP_POST,
            .handler = settings_post_handler,
            .user_ctx = rest_context
    };
    httpd_register_uri_handler(server, &settings_post_uri);

    /* URI handler for adding a new udp client connection */
    httpd_uri_t settings_clients_udp_post_uri = {
            .uri = "/api/settings/clients/udp",
            .method = HTTP_POST,
            .handler = settings_clients_udp_post,
            .user_ctx = rest_context
    };
    httpd_register_uri_handler(server, &settings_clients_udp_post_uri);

    /* URI handler for removing all known UDP client connections */
    httpd_uri_t settings_clients_clear_udp_get_uri = {
            .uri = "/api/settings/clients/clear_udp",
            .method = HTTP_DELETE,
            .handler = settings_clients_clear_udp_get,
            .user_ctx = rest_context
    };
    httpd_register_uri_handler(server, &settings_clients_clear_udp_get_uri);

    httpd_uri_t ota_portal_get_uri = {
            .uri = "/ota",
            .method = HTTP_GET,
            .handler = ota_portal_get_handler,
            .user_ctx = rest_context
    };
    httpd_register_uri_handler(server, &ota_portal_get_uri);

    httpd_uri_t update_info_get_uri = {
            .uri = "/api/update/info",
            .method = HTTP_GET,
            .handler = update_info_get_handler,
            .user_ctx = rest_context
    };
    httpd_register_uri_handler(server, &update_info_get_uri);

    httpd_uri_t update_app_post_uri = {
            .uri = "/api/update/app",
            .method = HTTP_POST,
            .handler = update_app_post_handler,
            .user_ctx = rest_context
    };
    httpd_register_uri_handler(server, &update_app_post_uri);

    httpd_uri_t update_web_post_uri = {
            .uri = "/api/update/web",
            .method = HTTP_POST,
            .handler = update_web_post_handler,
            .user_ctx = rest_context
    };
    httpd_register_uri_handler(server, &update_web_post_uri);

    httpd_uri_t update_boot_ap_once_post_uri = {
            .uri = "/api/update/boot-ap-once",
            .method = HTTP_POST,
            .handler = update_boot_ap_once_post_handler,
            .user_ctx = rest_context
    };
    httpd_register_uri_handler(server, &update_boot_ap_once_post_uri);

    httpd_uri_t sonar_log_status_get_uri = {
            .uri = "/api/logs/status",
            .method = HTTP_GET,
            .handler = sonar_log_status_get_handler,
            .user_ctx = rest_context
    };
    httpd_register_uri_handler(server, &sonar_log_status_get_uri);

    httpd_uri_t sonar_log_get_uri = {
            .uri = "/api/logs/sonar",
            .method = HTTP_GET,
            .handler = sonar_log_get_handler,
            .user_ctx = rest_context
    };
    httpd_register_uri_handler(server, &sonar_log_get_uri);

    httpd_uri_t sonar_log_download_get_uri = {
            .uri = "/api/logs/sonar/download",
            .method = HTTP_GET,
            .handler = sonar_log_download_get_handler,
            .user_ctx = rest_context
    };
    httpd_register_uri_handler(server, &sonar_log_download_get_uri);

    httpd_uri_t sonar_log_delete_uri = {
            .uri = "/api/logs/sonar",
            .method = HTTP_DELETE,
            .handler = sonar_log_delete_handler,
            .user_ctx = rest_context
    };
    httpd_register_uri_handler(server, &sonar_log_delete_uri);

    /* DBB Companion brain: let a linked private brain register its own routes
     * (e.g. the monitor page) BEFORE the catch-all file handler below. No-op in
     * the open base (weak stub). */
    dbb_brain_register_http(server);

    /* URI handler for getting web server files */
    httpd_uri_t common_get_uri = {
            .uri = "/*",
            .method = HTTP_GET,
            .handler = rest_common_get_handler,
            .user_ctx = rest_context
    };
    httpd_register_uri_handler(server, &common_get_uri);

    return ESP_OK;
    err_start:
    free(rest_context);
    err:
    return ESP_FAIL;
}
