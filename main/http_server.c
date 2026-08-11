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

#include <errno.h>
#include <db_parameters.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <lwip/sockets.h>
#include <esp_chip_info.h>
#include <esp_app_format.h>
#include <esp_ota_ops.h>
#include "dbb_brain.h"
#include <esp_partition.h>
#include "esp_http_server.h"
#include "esp_core_dump.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_vfs.h"
#include "cJSON.h"
#include "db_diag.h"
#include "db_fc_flash.h"
#include "db_location_store.h"
#include "db_ota_policy.h"
#include "db_sonar_log.h"
#include "danevi_sonar.h"
#include "deeper_udp_sonar.h"
#include "db_mavlink_msgs.h"
#include "db_timers.h"
#include "globals.h"
#include "main.h"
#include "db_serial.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#if CONFIG_WEB_DEPLOY_EMBEDDED
#include "dbb_web_assets.h"
#endif

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
    "<p>This recovery page lives inside the firmware image and stays available with every application slot.</p>"
    "<button class=\"alt\" onclick=\"refreshInfo().catch(err=>setStatus(err.message))\">Refresh</button>"
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
#if CONFIG_WEB_DEPLOY_EMBEDDED
    "<div class=\"card\">"
    "<h2>Web UI</h2>"
    "<p>The web UI is embedded in the application. Updating the firmware atomically updates both, so a separate <code>www.bin</code> is not used.</p>"
    "</div>"
#else
    "<div class=\"card\">"
    "<h2>Web UI OTA</h2>"
    "<p>Upload the <code>www.bin</code> built alongside this firmware. If the upload is interrupted, refresh <code>/ota</code> after the ESP reboots and try again.</p>"
    "<input id=\"webFile\" type=\"file\" accept=\".bin\">"
    "<button onclick=\"uploadFile('/api/update/web','webFile','Web UI upload started. The ESP will reboot when the upload completes successfully.')\">Upload Web UI & Reboot</button>"
    "</div>"
#endif
    "<div class=\"card\">"
    "<h2>Status</h2>"
    "<div id=\"status\" class=\"status\">Idle.</div>"
    "</div>"
    "</div>"
    "<script>"
    "function setStatus(msg){document.getElementById('status').textContent=msg;}"
    "function line(label,value){return label+': '+value;}"
    "function requestHeaders(contentType){"
    " const headers={};"
    " if(contentType){headers['Content-Type']=contentType;}return headers;"
    "}"
    "async function refreshInfo(){"
    " const res=await fetch('/api/update/info',{headers:requestHeaders()});"
    " if(!res.ok){throw new Error('Failed to load OTA info: '+res.status);}"
    " const info=await res.json();"
    " const lines=["
    "  line('Running partition',info.running_partition_label+' ('+info.running_partition_subtype+')'),"
    "  line('Boot partition',info.boot_partition_label+' ('+info.boot_partition_subtype+')'),"
    "  line('Next OTA target',info.next_update_partition_label+' ('+info.next_update_partition_subtype+')'),"
    "  line('Firmware version',info.running_app_version),"
    "  line('Firmware build',info.running_app_date+' '+info.running_app_time),"
    "  line('Web assets',info.web_assets_embedded ? 'embedded in firmware' : (info.web_fs_available ? 'filesystem mounted' : 'unavailable'))"
    " ];"
    " document.getElementById('ota-info').textContent=lines.join('\\n');"
    "}"
    "async function uploadFile(endpoint,inputId,startMessage){"
    " const input=document.getElementById(inputId);"
    " if(!input.files.length){setStatus('Select a .bin file first.');return;}"
    " const file=input.files[0];"
    " setStatus(startMessage+'\\nUploading '+file.name+' ('+file.size+' bytes)... Do not power off the boat.');"
    " const res=await fetch(endpoint,{method:'POST',headers:requestHeaders('application/octet-stream'),body:file});"
    " const text=await res.text();"
    " let message=text;"
    " try{const json=JSON.parse(text);if(json.msg){message=json.msg;}}catch(e){}"
    " if(!res.ok){throw new Error(message || ('Upload failed with status '+res.status));}"
    " setStatus(message || 'Upload finished.');"
    "}"
    "async function rebootIntoUpdateMode(){"
    " setStatus('Scheduling one-time AP update mode and rebooting...');"
    " const res=await fetch('/api/update/boot-ap-once',{method:'POST',headers:requestHeaders()});"
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
#if !CONFIG_WEB_DEPLOY_EMBEDDED
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
#endif

/**
 * Send HTTP response with the contents of the requested file
 */
static esp_err_t rest_common_get_handler(httpd_req_t *req) {
#if CONFIG_WEB_DEPLOY_EMBEDDED
    const char *asset_path = req->uri;
    if (strcmp(asset_path, "/") == 0) {
        asset_path = "/index.html";
    }
    const dbb_web_asset_t *asset = dbb_web_asset_find(asset_path);
    if (asset == NULL) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "embedded asset not found");
        return ESP_FAIL;
    }
    httpd_resp_set_type(req, asset->content_type);
    return httpd_resp_send(req, (const char *)asset->data, asset->size);
#else
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
#endif
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
    /* Validate before applying. db_param_read_all_params_json() skips invalid
     * values without telling anyone, so applying first and reporting after
     * would leave some fields changed and others silently untouched. Refusing
     * the whole request keeps the saved settings consistent with what the user
     * was told, and names the offending fields instead of rebooting into a
     * config they did not ask for. */
    char reject_list[192];
    const int rejected =
        db_param_validate_json(root, reject_list, sizeof(reject_list));
    if (rejected > 0) {
        cJSON_Delete(root);
        ESP_LOGW(TAG, "Rejected settings request: %d invalid value(s): %s",
                 rejected, reject_list);
        /* Parameter names and generated reasons are alphanumeric plus
         * " ,-()" so they need no JSON escaping. */
        char err_resp[320];
        snprintf(err_resp, sizeof(err_resp),
                 "{\n"
                 "    \"status\": \"error\",\n"
                 "    \"msg\": \"Nothing was saved. %d invalid value(s): %s\"\n"
                 "  }",
                 rejected, reject_list);
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_set_type(req, "application/json");
        db_http_resp_sendstr_with_retry(req, err_resp);
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
    if (root == NULL) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                            "out of memory");
        return ESP_ERR_NO_MEM;
    }
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
    deeper_udp_diagnostics_t deeper_diagnostics = {0};
    db_sonar_publish_diagnostics_t sonar_diagnostics = {0};
    db_mavlink_fc_state_t fc_state = {0};
    db_mavlink_telemetry_t telemetry = {0};
    db_sonar_log_crash_diag_t logger_diag = {0};
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
    deeper_udp_sonar_get_diagnostics(&deeper_diagnostics);
    db_get_sonar_publish_diagnostics(&sonar_diagnostics);
    db_mavlink_get_fc_state(&fc_state);
    db_mavlink_get_telemetry(&telemetry);
    db_sonar_log_get_crash_diag(&logger_diag);
    cJSON_AddNumberToObject(root, "esp_uptime_ms",
                            (double)(esp_timer_get_time() / 1000ULL));
    cJSON_AddNumberToObject(root, "esp_free_heap_bytes",
                            esp_get_free_heap_size());
    cJSON_AddNumberToObject(root, "logger_reset_reason",
                            logger_diag.reset_reason);
    cJSON_AddNumberToObject(root, "logger_previous_stage",
                            logger_diag.previous_stage);
    cJSON_AddStringToObject(
        root, "logger_previous_stage_name",
        db_sonar_log_diag_stage_name(logger_diag.previous_stage));
    cJSON_AddNumberToObject(root, "logger_previous_sequence",
                            logger_diag.previous_sequence);
    cJSON_AddNumberToObject(root, "logger_previous_min_stack",
                            logger_diag.previous_min_stack);
    cJSON_AddNumberToObject(root, "logger_current_stage",
                            logger_diag.current_stage);
    cJSON_AddStringToObject(
        root, "logger_current_stage_name",
        db_sonar_log_diag_stage_name(logger_diag.current_stage));
    cJSON_AddNumberToObject(root, "logger_current_sequence",
                            logger_diag.current_sequence);
    cJSON_AddNumberToObject(root, "logger_current_min_stack",
                            logger_diag.current_min_stack);
    cJSON_AddNumberToObject(root, "read_bytes", serial_total_byte_count);
    cJSON_AddNumberToObject(root, "sent_bytes",
                            __atomic_load_n(&serial_total_sent_byte_count,
                                            __ATOMIC_RELAXED));
    cJSON_AddNumberToObject(root, "serial_dec_mav_msgs", serial_total_decoded_mav_msgs);
    cJSON_AddNumberToObject(root, "tcp_connected", num_connected_tcp_clients);
    cJSON_AddNumberToObject(root, "udp_connected", udp_conn_list->size);
    cJSON_AddNumberToObject(root, "active_sonar_source", DB_ACTIVE_SONAR_SOURCE);
    cJSON_AddNumberToObject(root, "fc_seen", fc_state.seen ? 1 : 0);
    cJSON_AddNumberToObject(root, "fc_stale", fc_state.stale ? 1 : 0);
    cJSON_AddNumberToObject(root, "fc_armed", fc_state.armed ? 1 : 0);
    cJSON_AddStringToObject(root, "fc_mode",
                            fc_state.mode_name == NULL ? "unknown"
                                                       : fc_state.mode_name);
    cJSON_AddNumberToObject(root, "fc_heartbeat_age_ms",
                            fc_state.heartbeat_age_ms);
    cJSON_AddNumberToObject(root, "fc_base_mode", fc_state.base_mode);
    cJSON_AddNumberToObject(root, "fc_custom_mode", fc_state.custom_mode);
    cJSON_AddNumberToObject(root, "fc_system_status",
                            fc_state.system_status);
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
    cJSON_AddNumberToObject(root, "deeper_request_count",
                            deeper_diagnostics.request_count);
    cJSON_AddNumberToObject(root, "deeper_depth_count",
                            deeper_diagnostics.depth_count);
    cJSON_AddNumberToObject(root, "deeper_last_depth_interval_ms",
                            deeper_diagnostics.last_depth_interval_ms);
    cJSON_AddNumberToObject(root, "deeper_max_depth_interval_ms",
                            deeper_diagnostics.max_depth_interval_ms);
    cJSON_AddNumberToObject(root, "deeper_last_depth_age_ms",
                            deeper_diagnostics.last_depth_age_ms);
    cJSON_AddNumberToObject(root, "deeper_fc_publish_count",
                            sonar_diagnostics.deeper_publish_count);
    cJSON_AddNumberToObject(root, "deeper_fresh_publish_count",
                            sonar_diagnostics.deeper_fresh_publish_count);
    cJSON_AddNumberToObject(root, "deeper_no_data_skip_count",
                            sonar_diagnostics.deeper_no_data_skip_count);
    cJSON_AddNumberToObject(root, "deeper_return_count",
                            telemetry.returned_distance_sensor.count);
    cJSON_AddNumberToObject(root, "deeper_return_last_interval_ms",
                            telemetry.returned_distance_sensor.last_interval_ms);
    cJSON_AddNumberToObject(root, "deeper_return_max_interval_ms",
                            telemetry.returned_distance_sensor.max_interval_ms);
    cJSON_AddNumberToObject(root, "deeper_return_age_ms",
                            telemetry.returned_distance_sensor.age_ms);
    cJSON_AddNumberToObject(root, "deeper_return_depth_mm",
                            telemetry.returned_distance_sensor.distance_mm);
    cJSON_AddNumberToObject(root, "deeper_return_sensor_id",
                            telemetry.returned_distance_sensor.sensor_id);
    cJSON_AddNumberToObject(root, "deeper_return_sysid",
                            telemetry.returned_distance_sensor.sysid);
    cJSON_AddNumberToObject(root, "deeper_return_compid",
                            telemetry.returned_distance_sensor.compid);
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

/* Public MAVLink telemetry: decoded in the open companion, not the private brain. */
static cJSON *db_telemetry_group(cJSON *root, const char *name, bool valid,
                                 int64_t age_ms) {
    cJSON *group = cJSON_AddObjectToObject(root, name);
    cJSON_AddBoolToObject(group, "valid", valid);
    cJSON_AddNumberToObject(group, "age_ms", (double)age_ms);
    return group;
}

static esp_err_t telemetry_get_handler(httpd_req_t *req) {
    db_mavlink_telemetry_t t = {0};
    db_mavlink_get_telemetry(&t);
    cJSON *root = cJSON_CreateObject();
    if (root == NULL) return ESP_ERR_NO_MEM;
#define GROUP(name, value) cJSON *name = db_telemetry_group(root, #name, (value).valid, (value).age_ms)
#define N(group, key, value) cJSON_AddNumberToObject((group), (key), (double)(value))
    cJSON *fc = cJSON_AddObjectToObject(root, "flight_controller");
    cJSON_AddBoolToObject(fc, "seen", t.fc.seen); cJSON_AddBoolToObject(fc, "stale", t.fc.stale);
    cJSON_AddBoolToObject(fc, "armed", t.fc.armed); N(fc,"heartbeat_age_ms",t.fc.heartbeat_age_ms);
    N(fc,"system_id",t.fc.sysid); N(fc,"component_id",t.fc.compid); cJSON_AddStringToObject(fc,"mode",t.fc.mode_name ? t.fc.mode_name : "unknown");
    GROUP(system, t.system); N(system,"load_pct",t.system.load/10.0); N(system,"voltage_v",t.system.voltage_mv/1000.0); N(system,"current_a",t.system.current_ca/100.0); N(system,"battery_remaining_pct",t.system.remaining_pct); N(system,"drop_rate_pct",t.system.drop_rate_comm/100.0); N(system,"errors_comm",t.system.errors_comm); N(system,"sensors_present",t.system.sensors_present); N(system,"sensors_enabled",t.system.sensors_enabled); N(system,"sensors_health",t.system.sensors_health);
    GROUP(time, t.time); N(time,"unix_usec",t.time.unix_usec); N(time,"boot_ms",t.time.boot_ms);
    GROUP(gps, t.gps); N(gps,"fix_type",t.gps.fix_type); N(gps,"satellites",t.gps.satellites_visible); N(gps,"hdop",t.gps.eph/100.0); N(gps,"latitude",t.gps.latitude_e7/10000000.0); N(gps,"longitude",t.gps.longitude_e7/10000000.0);
    GROUP(position, t.position); N(position,"latitude",t.position.latitude_e7/10000000.0); N(position,"longitude",t.position.longitude_e7/10000000.0); N(position,"altitude_m",t.position.altitude_mm/1000.0); N(position,"relative_altitude_m",t.position.relative_altitude_mm/1000.0); N(position,"vx_mps",t.position.vx_cms/100.0); N(position,"vy_mps",t.position.vy_cms/100.0); N(position,"vz_mps",t.position.vz_cms/100.0); N(position,"heading_deg",t.position.heading_cdeg/100.0);
    GROUP(attitude, t.attitude); N(attitude,"roll_rad",t.attitude.roll_rad); N(attitude,"pitch_rad",t.attitude.pitch_rad); N(attitude,"yaw_rad",t.attitude.yaw_rad); N(attitude,"rollspeed",t.attitude.rollspeed); N(attitude,"pitchspeed",t.attitude.pitchspeed); N(attitude,"yawspeed",t.attitude.yawspeed);
    GROUP(vfr, t.vfr); N(vfr,"airspeed_mps",t.vfr.airspeed_mps); N(vfr,"groundspeed_mps",t.vfr.groundspeed_mps); N(vfr,"heading_deg",t.vfr.heading_deg); N(vfr,"throttle_pct",t.vfr.throttle_pct); N(vfr,"altitude_m",t.vfr.altitude_m); N(vfr,"climb_mps",t.vfr.climb_mps);
    GROUP(pressure, t.pressure); N(pressure,"absolute_hpa",t.pressure.press_abs_hpa); N(pressure,"differential_hpa",t.pressure.press_diff_hpa); N(pressure,"temperature_c",t.pressure.temperature_cdeg/100.0);
    GROUP(raw_imu, t.raw_imu); N(raw_imu,"accel_x",t.raw_imu.xacc); N(raw_imu,"accel_y",t.raw_imu.yacc); N(raw_imu,"accel_z",t.raw_imu.zacc); N(raw_imu,"gyro_x",t.raw_imu.xgyro); N(raw_imu,"gyro_y",t.raw_imu.ygyro); N(raw_imu,"gyro_z",t.raw_imu.zgyro); N(raw_imu,"mag_x",t.raw_imu.xmag); N(raw_imu,"mag_y",t.raw_imu.ymag); N(raw_imu,"mag_z",t.raw_imu.zmag);
    GROUP(scaled_imu2, t.scaled_imu2); N(scaled_imu2,"accel_x",t.scaled_imu2.xacc); N(scaled_imu2,"accel_y",t.scaled_imu2.yacc); N(scaled_imu2,"accel_z",t.scaled_imu2.zacc); N(scaled_imu2,"gyro_x",t.scaled_imu2.xgyro); N(scaled_imu2,"gyro_y",t.scaled_imu2.ygyro); N(scaled_imu2,"gyro_z",t.scaled_imu2.zgyro); N(scaled_imu2,"mag_x",t.scaled_imu2.xmag); N(scaled_imu2,"mag_y",t.scaled_imu2.ymag); N(scaled_imu2,"mag_z",t.scaled_imu2.zmag);
    GROUP(power, t.power); N(power,"vcc_v",t.power.vcc_mv/1000.0); N(power,"servo_v",t.power.vservo_mv/1000.0); N(power,"flags",t.power.flags);
    GROUP(battery, t.battery); N(battery,"voltage_v",t.battery.voltage_mv/1000.0); N(battery,"current_a",t.battery.current_ca/100.0); N(battery,"remaining_pct",t.battery.remaining_pct); N(battery,"consumed_mah",t.battery.consumed_mah); N(battery,"temperature_c",t.battery.temperature_cdeg/100.0);
    GROUP(mission, t.mission); N(mission,"current_sequence",t.mission.seq);
    GROUP(servo, t.servo); N(servo,"port",t.servo.port); cJSON *servo_raw=cJSON_AddArrayToObject(servo,"raw_us"); for(int i=0;i<16;i++) cJSON_AddItemToArray(servo_raw,cJSON_CreateNumber(t.servo.raw[i]));
    GROUP(rc, t.rc); N(rc,"channel_count",t.rc.chancount); N(rc,"rssi",t.rc.rssi); N(rc,"updates",t.rc.updates); cJSON *rc_raw=cJSON_AddArrayToObject(rc,"raw_us"); for(int i=0;i<18;i++) cJSON_AddItemToArray(rc_raw,cJSON_CreateNumber(t.rc.chan[i]));
    GROUP(vibration, t.vibration); N(vibration,"x",t.vibration.vibration_x); N(vibration,"y",t.vibration.vibration_y); N(vibration,"z",t.vibration.vibration_z); N(vibration,"clipping_0",t.vibration.clipping_0); N(vibration,"clipping_1",t.vibration.clipping_1); N(vibration,"clipping_2",t.vibration.clipping_2);
    GROUP(timesync, t.timesync); N(timesync,"tc1",t.timesync.tc1); N(timesync,"ts1",t.timesync.ts1);
    GROUP(statustext, t.statustext); N(statustext,"severity",t.statustext.severity); cJSON_AddStringToObject(statustext,"text",t.statustext.text);
    cJSON *messages=cJSON_AddArrayToObject(root,"message_activity"); for(int i=0;i<t.message_stat_count;i++){cJSON *item=cJSON_CreateObject(); N(item,"id",t.message_stats[i].id); N(item,"count",t.message_stats[i].count); N(item,"age_ms",t.message_stats[i].age_ms); cJSON_AddItemToArray(messages,item);}
#undef N
#undef GROUP
    char *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (json == NULL) return ESP_ERR_NO_MEM;
    httpd_resp_set_type(req, "application/json"); httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    db_http_resp_sendstr_with_retry(req, json); free(json); return ESP_OK;
}

static const char DB_TELEMETRY_PAGE[] =
"<!doctype html><meta charset=utf-8><meta name=viewport content='width=device-width,initial-scale=1'><title>DBB Telemetry</title>"
"<style>:root{color-scheme:dark;--b:#0e1116;--c:#171c24;--m:#8b949e;--a:#3fb950}*{box-sizing:border-box}body{margin:0;background:var(--b);color:#e6edf3;font:14px system-ui,Segoe UI,sans-serif}header{padding:15px 18px;border-bottom:1px solid #252b34;display:flex;justify-content:space-between}h1{font-size:17px;margin:0}.mut{color:var(--m)}main{max-width:1280px;margin:auto;padding:16px;display:grid;gap:14px;grid-template-columns:repeat(auto-fit,minmax(280px,1fr))}.card{background:var(--c);border:1px solid #252b34;border-radius:10px;padding:13px}.card h2{font-size:12px;text-transform:uppercase;letter-spacing:.06em;margin:0 0 9px;color:var(--m)}table{width:100%;border-collapse:collapse}td{padding:4px;border-bottom:1px solid #252b34}td:last-child{text-align:right;font-family:ui-monospace,Consolas,monospace}.bad{color:#f85149}.good{color:var(--a)}#activity{grid-column:1/-1}</style>"
"<header><h1>DBB Companion <span class=mut>&middot; Public Telemetry</span></h1><span id=status class=mut>connecting...</span></header><main id=main></main>"
"<script>const names={0:'HEARTBEAT',1:'SYS_STATUS',2:'SYSTEM_TIME',24:'GPS_RAW_INT',27:'RAW_IMU',29:'SCALED_PRESSURE',30:'ATTITUDE',33:'GLOBAL_POSITION_INT',34:'RC_CHANNELS_RAW',36:'SERVO_OUTPUT_RAW',42:'MISSION_CURRENT',65:'RC_CHANNELS',74:'VFR_HUD',111:'TIMESYNC',116:'SCALED_IMU2',125:'POWER_STATUS',147:'BATTERY_STATUS',241:'VIBRATION',253:'STATUSTEXT'};const fmt=v=>typeof v==='number'?(Number.isInteger(v)?v:v.toFixed(4)):Array.isArray(v)?v.join(', '):v;function card(k,o){let rows=Object.entries(o).filter(([x])=>x!=='valid'&&x!=='age_ms').map(([x,v])=>`<tr><td>${x.replaceAll('_',' ')}</td><td>${fmt(v)}</td></tr>`).join('');let live=o.valid===undefined?o.seen:o.valid;let state=live?'<span class=good>live</span>':'<span class=bad>not received</span>';let age=o.age_ms===undefined?(o.heartbeat_age_ms===undefined?'':o.heartbeat_age_ms):o.age_ms;return `<section class=card><h2>${k.replaceAll('_',' ')} · ${state} <span class=mut>${age}ms</span></h2><table>${rows}</table></section>`}async function tick(){try{let d=await (await fetch('/api/telemetry',{cache:'no-store'})).json();let h='';for(const[k,v]of Object.entries(d)){if(k==='message_activity')continue;h+=card(k,v)}let a=d.message_activity.map(m=>`<tr><td>${names[m.id]||'MSG #'+m.id}</td><td>${m.count}</td><td>${m.age_ms}ms ago</td></tr>`).join('');h+=`<section id=activity class=card><h2>received MAVLink messages</h2><table>${a}</table></section>`;document.querySelector('main').innerHTML=h;document.querySelector('#status').innerHTML='<span class=good>live</span> · refreshes 1s'}catch(e){document.querySelector('#status').innerHTML='<span class=bad>no link</span>'}}tick();setInterval(tick,1000)</script>";

static esp_err_t telemetry_page_handler(httpd_req_t *req) { httpd_resp_set_type(req, "text/html"); return httpd_resp_send(req, DB_TELEMETRY_PAGE, HTTPD_RESP_USE_STRLEN); }

/* =====================================================================
 * Flight-controller flashing over USB OTG.
 *
 * Standalone page, like /telemetry and /ota - deliberately NOT part of
 * index.html, so it cannot disturb the frontend build (build_frontend.py
 * asserts exactly one inline CSS and two inline JS tags there).
 *
 * The console is POLLED, not streamed. Server-Sent Events would pin an httpd
 * worker for the whole 30 s flash on a server whose 8192-byte stack is already
 * load-bearing; polling a sequence-numbered ring also survives a Wi-Fi blip
 * mid-flash without losing or duplicating lines.
 * ===================================================================== */

static const char *db_fc_flash_state_name(db_fc_flash_state_t s)
{
    switch (s) {
    case DB_FC_FLASH_IDLE:            return "idle";
    case DB_FC_FLASH_WAITING_FOR_FC:  return "waiting_for_fc";
    case DB_FC_FLASH_REBOOTING:       return "rebooting";
    case DB_FC_FLASH_ERASING:         return "erasing";
    case DB_FC_FLASH_PROGRAMMING:     return "programming";
    case DB_FC_FLASH_VERIFYING:       return "verifying";
    case DB_FC_FLASH_DONE:            return "done";
    case DB_FC_FLASH_FAILED:          return "failed";
    }
    return "unknown";
}

static esp_err_t fcflash_status_get_handler(httpd_req_t *req)
{
    uint32_t since = 0;
    char q[64];
    if (httpd_req_get_url_query_str(req, q, sizeof(q)) == ESP_OK) {
        char v[16];
        if (httpd_query_key_value(q, "since", v, sizeof(v)) == ESP_OK) {
            since = (uint32_t)strtoul(v, NULL, 10);
        }
    }

    db_fc_flash_status_t st;
    db_fc_flash_get_status(&st);

    /* Console text is fetched into a heap buffer: it can be several KiB and the
     * handler stack is shared with db_mavlink_get_telemetry's large locals. */
    const size_t console_max = 4096;
    char *console = malloc(console_max);
    uint32_t next_seq = since;
    if (console) {
        db_fc_flash_console_read(since, console, console_max, &next_seq);
    }

    db_fc_flash_image_info_t img;
    bool have_img = (db_fc_flash_get_image_info(&img) == ESP_OK && img.present);

    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "state", db_fc_flash_state_name(st.state));
    cJSON_AddBoolToObject(root, "usb_host", st.usb_host_running);
    cJSON_AddBoolToObject(root, "in_bootloader", st.in_bootloader);
    cJSON_AddBoolToObject(root, "armed", db_fc_flash_vehicle_is_armed());
    cJSON_AddNumberToObject(root, "board_id", st.board_id);
    cJSON_AddNumberToObject(root, "bl_rev", st.bl_rev);
    cJSON_AddNumberToObject(root, "bytes_total", st.bytes_total);
    cJSON_AddNumberToObject(root, "bytes_done", st.bytes_done);
    cJSON_AddNumberToObject(root, "crc_expected", st.crc_expected);
    cJSON_AddNumberToObject(root, "crc_actual", st.crc_actual);
    cJSON_AddStringToObject(root, "error", st.last_error);
    cJSON_AddStringToObject(root, "console", console ? console : "");
    cJSON_AddNumberToObject(root, "next_seq", next_seq);
    cJSON_AddBoolToObject(root, "image_present", have_img);
    if (have_img) {
        cJSON_AddNumberToObject(root, "image_board_id", img.board_id);
        cJSON_AddNumberToObject(root, "image_size", img.image_size);
        cJSON_AddNumberToObject(root, "image_crc", img.crc);
        cJSON_AddStringToObject(root, "image_name", img.source_name);
    }
    free(console);

    const char *out = cJSON_PrintUnformatted(root);
    httpd_resp_set_type(req, "application/json");
    esp_err_t err = httpd_resp_sendstr(req, out ? out : "{}");
    free((void *)out);
    cJSON_Delete(root);
    return err;
}

static esp_err_t fcflash_upload_post_handler(httpd_req_t *req)
{
    char q[160] = {0};
    char name[64] = "firmware.apj";
    uint32_t board_id = 0;

    if (httpd_req_get_url_query_str(req, q, sizeof(q)) == ESP_OK) {
        char v[64];
        if (httpd_query_key_value(q, "board_id", v, sizeof(v)) == ESP_OK) {
            board_id = (uint32_t)strtoul(v, NULL, 10);
        }
        if (httpd_query_key_value(q, "name", v, sizeof(v)) == ESP_OK) {
            strlcpy(name, v, sizeof(name));
        }
    }
    if (board_id == 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                            "board_id missing - refusing an image we cannot verify");
        return ESP_FAIL;
    }
    if (db_fc_flash_upload_begin(name, board_id) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                            "cannot open the firmware slot for writing");
        return ESP_FAIL;
    }

    char *buf = malloc(2048);
    if (!buf) {
        db_fc_flash_upload_abort();
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "out of memory");
        return ESP_FAIL;
    }
    int remaining = req->content_len;
    while (remaining > 0) {
        int got = httpd_req_recv(req, buf, remaining < 2048 ? remaining : 2048);
        if (got <= 0) {
            if (got == HTTPD_SOCK_ERR_TIMEOUT) continue;
            free(buf);
            db_fc_flash_upload_abort();
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                "upload interrupted");
            return ESP_FAIL;
        }
        if (db_fc_flash_upload_write((const uint8_t *)buf, got) != ESP_OK) {
            free(buf);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                "write failed - is the logs partition full?");
            return ESP_FAIL;
        }
        remaining -= got;
    }
    free(buf);

    db_fc_flash_image_info_t info;
    if (db_fc_flash_upload_finish(&info) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                            "image rejected - wrong size or not word aligned");
        return ESP_FAIL;
    }

    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "ok", true);
    cJSON_AddNumberToObject(root, "image_size", info.image_size);
    cJSON_AddNumberToObject(root, "board_id", info.board_id);
    cJSON_AddNumberToObject(root, "crc", info.crc);
    const char *out = cJSON_PrintUnformatted(root);
    httpd_resp_set_type(req, "application/json");
    esp_err_t err = httpd_resp_sendstr(req, out ? out : "{}");
    free((void *)out);
    cJSON_Delete(root);
    return err;
}

static esp_err_t fcflash_start_post_handler(httpd_req_t *req)
{
    esp_err_t rc = db_fc_flash_start();
    if (rc == ESP_ERR_INVALID_STATE) {
        /* esp_http_server has no HTTPD_409_CONFLICT; 400 carries the message. */
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                            "refused - already running, or the vehicle is armed");
        return ESP_FAIL;
    }
    if (rc == ESP_ERR_NOT_FOUND) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                            "refused - no valid firmware stored");
        return ESP_FAIL;
    }
    if (rc != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "could not start");
        return ESP_FAIL;
    }
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"ok\":true}");
}

static esp_err_t fcflash_delete_handler(httpd_req_t *req)
{
    db_fc_flash_delete_image();
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"ok\":true}");
}

static const char DB_FCFLASH_PAGE[] =
"<!doctype html><html><head><meta charset='utf-8'>"
"<meta name='viewport' content='width=device-width,initial-scale=1'>"
"<title>DBB - Flight Controller Firmware</title><style>"
"body{background:#12151a;color:#d8dee9;font-family:system-ui,sans-serif;margin:0;padding:16px}"
"h1{font-size:20px;margin:0 0 4px}h2{font-size:15px;margin:18px 0 6px;color:#88c0d0}"
"a{color:#88c0d0}.sub{color:#7b8494;font-size:13px;margin-bottom:14px}"
".card{background:#1b1f27;border:1px solid #2a303b;border-radius:8px;padding:12px;margin-bottom:14px}"
"button{background:#3b4252;color:#eceff4;border:1px solid #4c566a;border-radius:6px;"
"padding:9px 16px;font-size:14px;cursor:pointer;margin-right:8px}"
"button:disabled{opacity:.4;cursor:not-allowed}"
"button.go{background:#5e8145;border-color:#6a9150}button.danger{background:#7b3b3b;border-color:#9a4a4a}"
/* Scales with the window and can be dragged taller. resize:vertical needs a
 * non-visible overflow to get a grab handle, which overflow-y:auto provides. */
"#console{background:#0b0d11;border:1px solid #2a303b;border-radius:6px;padding:10px;"
"height:45vh;min-height:160px;max-height:80vh;resize:vertical;overflow-y:auto;"
"white-space:pre-wrap;font-family:ui-monospace,monospace;"
"font-size:12px;line-height:1.45}"
"table{border-collapse:collapse;font-size:13px}td{padding:2px 14px 2px 0}"
".k{color:#7b8494}.warn{color:#ebcb8b}.err{color:#bf616a}.ok{color:#a3be8c}"
/* Browser-side console lines. They are drawn immediately while boat-side lines
 * only arrive on the next poll, so without a visible distinction the two
 * interleave and read as out of order - misleading exactly when something has
 * gone wrong. Dimmed and prefixed, so the reader can tell who is speaking. */
".pc{color:#6f7787}"
"progress{width:100%;height:16px}</style></head><body>"
"<h1>Flight Controller Firmware</h1>"
"<div class='sub'>Flashes the FC over USB OTG from the boat. "
"<a href='/'>&larr; dashboard</a></div>"

"<div class='card'><h2>1 &middot; Stored firmware</h2>"
"<table><tr><td class='k'>File</td><td id='iname'>&mdash;</td></tr>"
"<tr><td class='k'>Size</td><td id='isize'>&mdash;</td></tr>"
"<tr><td class='k'>Board ID</td><td id='ibid'>&mdash;</td></tr>"
"<tr><td class='k'>CRC</td><td id='icrc'>&mdash;</td></tr></table>"
"<p><input type='file' id='file' accept='.apj,.bin'> "
"<button id='up'>Upload</button><button id='del' class='danger'>Delete</button></p>"
"<p><progress id='uprog' value='0' max='100'></progress></p>"
"<div id='upmsg' class='sub'></div></div>"

"<div class='card'><h2>2 &middot; Flight controller</h2>"
"<table><tr><td class='k'>State</td><td id='state'>&mdash;</td></tr>"
"<tr><td class='k'>Board ID</td><td id='fbid'>&mdash;</td></tr>"
"<tr><td class='k'>Armed</td><td id='armed'>&mdash;</td></tr></table>"
"<p><progress id='prog' value='0' max='100'></progress></p>"
"<p><button id='flash' class='go' disabled>Flash the flight controller</button></p>"
"<div id='gate' class='sub'></div></div>"

"<div class='card'><h2>3 &middot; Console</h2><div id='console'></div></div>"

"<script>\n"
"var seq=0;\n"
"function $(i){return document.getElementById(i)}\n"
"function hex(n){return '0x'+(n>>>0).toString(16).toUpperCase().padStart(8,'0')}\n"
"function say(m,c){var d=$('console');var s=document.createElement('div');"
"if(c)s.className=c;s.textContent=m;d.appendChild(s);d.scrollTop=d.scrollHeight}\n"
/* pc() = this browser said it. say() with no class = the boat said it. */
"function pc(m,c){say('\\u00bb '+m,c||'pc')}\n"

/* Decode an ArduPilot .apj entirely in the browser: base64 then raw zlib via
 * the platform's DecompressionStream. Keeps zlib and a 1.3 MB decode buffer out
 * of the firmware. A .bin is passed straight through. */
"async function decode(f){\n"
" if(!f.name.toLowerCase().endsWith('.apj')){\n"
"  var b=new Uint8Array(await f.arrayBuffer());\n"
"  return {bin:b,board_id:0,name:f.name};}\n"
" var j=JSON.parse(await f.text());\n"
" var raw=Uint8Array.from(atob(j.image),function(c){return c.charCodeAt(0)});\n"
" if(typeof DecompressionStream==='undefined')\n"
"  throw new Error('This browser cannot decompress .apj. Use a current Chrome/Firefox/Safari.');\n"
" var st=new Blob([raw]).stream().pipeThrough(new DecompressionStream('deflate'));\n"
" var buf=await new Response(st).arrayBuffer();\n"
" return {bin:new Uint8Array(buf),board_id:j.board_id||0,name:f.name};}\n"

/* XMLHttpRequest, not fetch: fetch cannot report UPLOAD progress at all, and a
 * 1.2 MB post over the boat's Wi-Fi otherwise looks frozen for many seconds. */
"function send(bin,bid,name){return new Promise(function(res,rej){\n"
" var x=new XMLHttpRequest(),last=-1;\n"
" x.open('POST','/api/fcflash/upload?board_id='+bid+'&name='+encodeURIComponent(name));\n"
" x.upload.onprogress=function(e){\n"
"  if(!e.lengthComputable)return;\n"
"  var p=100*e.loaded/e.total;$('uprog').value=p;\n"
"  $('upmsg').textContent='Uploading '+(e.loaded>>10)+' / '+(e.total>>10)+' KiB ('+p.toFixed(0)+'%)';\n"
"  var q=Math.floor(p/10)*10;if(q>last){last=q;pc('upload '+q+'%');}};\n"
" x.onload=function(){if(x.status>=200&&x.status<300){try{res(JSON.parse(x.responseText))}\n"
"  catch(e){rej(new Error('bad reply from the boat'))}}else rej(new Error(x.responseText||('HTTP '+x.status)));};\n"
" x.onerror=function(){rej(new Error('network error - did the Wi-Fi drop?'))};\n"
" x.send(bin);});}\n"

"$('up').onclick=async function(){\n"
" var f=$('file').files[0];\n"
" if(!f){$('upmsg').textContent='Choose a .apj first.';return}\n"
" $('up').disabled=true;$('uprog').value=0;\n"
" $('upmsg').textContent='Decoding '+f.name+' ...';pc('decoding '+f.name);\n"
" try{\n"
"  var d=await decode(f);\n"
"  if(!d.board_id){throw new Error('No board_id in that file - refusing an image we cannot verify against the FC.');}\n"
"  pc('decoded '+d.bin.length+' bytes, board '+d.board_id);\n"
"  var j=await send(d.bin,d.board_id,d.name);\n"
"  $('uprog').value=100;\n"
"  $('upmsg').textContent='Stored. CRC '+hex(j.crc)+' (computed on the boat).';\n"
/* Deliberately does NOT restate size/board/CRC - the boat logs that itself a
 * moment later, and printing both made it look like it happened twice. */
"  pc('upload complete, waiting for the boat to verify');\n"
" }catch(e){$('upmsg').textContent='Failed: '+e.message;pc('upload failed: '+e.message,'err');}\n"
" $('up').disabled=false;};\n"

"$('del').onclick=async function(){\n"
" if(!confirm('Delete the stored firmware from the boat?'))return;\n"
" await fetch('/api/fcflash/image',{method:'DELETE'});};\n"

"$('flash').onclick=async function(){\n"
" if(!confirm('This ERASES the flight controller and writes new firmware.\\n\\n"
"Do not cut power until it finishes. Continue?'))return;\n"
" $('flash').disabled=true;\n"
" var r=await fetch('/api/fcflash/start',{method:'POST'});\n"
" if(!r.ok)pc('refused: '+await r.text(),'err');};\n"

"async function poll(){\n"
" try{\n"
"  var r=await fetch('/api/fcflash/status?since='+seq);var s=await r.json();\n"
"  if(s.console){s.console.split('\\n').forEach(function(l){if(l)say(l)});}\n"
"  seq=s.next_seq;\n"
"  $('state').textContent=s.state;\n"
"  $('fbid').textContent=s.board_id?s.board_id:'\\u2014';\n"
"  $('armed').innerHTML=s.armed?'<span class=err>ARMED</span>':'<span class=ok>disarmed</span>';\n"
"  $('iname').textContent=s.image_present?s.image_name:'\\u2014';\n"
"  $('isize').textContent=s.image_present?s.image_size+' bytes':'\\u2014';\n"
"  $('ibid').textContent=s.image_present?s.image_board_id:'\\u2014';\n"
"  $('icrc').textContent=s.image_present?hex(s.image_crc):'\\u2014';\n"
"  if(s.bytes_total)$('prog').value=100*s.bytes_done/s.bytes_total;\n"
"  var busy=['waiting_for_fc','rebooting','erasing','programming','verifying'].indexOf(s.state)>=0;\n"
"  var why='';\n"
"  if(s.armed)why='Blocked: the vehicle is armed.';\n"
"  else if(!s.image_present)why='Upload a firmware image first.';\n"
"  else if(busy)why='Busy \\u2014 flashing in progress.';\n"
"  else if(s.board_id&&s.image_present&&s.board_id!=s.image_board_id)\n"
"   why='Blocked: FC board '+s.board_id+' does not match the image ('+s.image_board_id+').';\n"
"  $('gate').textContent=why;\n"
"  $('flash').disabled=!!why;\n"
"  if(s.state==='failed'&&s.error)$('gate').innerHTML=\"<span class='err'>\"+s.error+\"</span>\";\n"
" }catch(e){}\n"
" setTimeout(poll,500);}\n"
"poll();\n"
"</script></body></html>";

static esp_err_t fcflash_page_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, DB_FCFLASH_PAGE, HTTPD_RESP_USE_STRLEN);
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
    const esp_partition_t *boot = db_ota_get_configured_boot_partition();
    const esp_partition_t *next = db_ota_select_normal_update_partition();
#if CONFIG_WEB_DEPLOY_SF
    const esp_partition_t *web = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_SPIFFS,
        DB_WEB_PARTITION_LABEL);
#endif
    esp_app_desc_t running_desc;
    db_ota_health_status_t health;
    db_ota_health_get_status(&health);

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
    cJSON_AddNumberToObject(root, "ota_health_state", health.state);
    cJSON_AddBoolToObject(root, "ota_pending_verify",
                          health.running_pending_verify);
    cJSON_AddNumberToObject(root, "ota_health_required_mask",
                            health.required_mask);
    cJSON_AddNumberToObject(root, "ota_health_passed_mask",
                            health.passed_mask);
    cJSON_AddNumberToObject(root, "ota_health_failed_mask",
                            health.failed_mask);
    cJSON_AddNumberToObject(root, "ota_health_elapsed_ms",
                            health.elapsed_ms);
    cJSON_AddNumberToObject(root, "ota_health_timeout_ms",
                            health.timeout_ms);
    cJSON_AddNumberToObject(root, "web_fs_available",
                            db_is_web_fs_available() ? 1 : 0);
#if CONFIG_WEB_DEPLOY_EMBEDDED
    cJSON_AddNumberToObject(root, "web_assets_embedded", 1);
    cJSON_AddNumberToObject(root, "web_partition_size_bytes", 0);
#else
    cJSON_AddNumberToObject(root, "web_assets_embedded", 0);
    cJSON_AddNumberToObject(root, "web_partition_size_bytes",
                            web ? web->size : 0);
#endif

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
    /* Direct A/B OTA: stream the image into the inactive slot, verify via
     * esp_ota_end, then switch the boot partition. */
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

#if CONFIG_WEB_DEPLOY_SF
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
#endif

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

typedef struct {
    httpd_req_t *req;
} sonar_log_http_stream_context_t;

static esp_err_t sonar_log_http_chunk_writer(const char *data,
                                             size_t data_length,
                                             void *user_ctx) {
    sonar_log_http_stream_context_t *stream_context =
        (sonar_log_http_stream_context_t *)user_ctx;
    if (data == NULL || stream_context == NULL ||
        stream_context->req == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    return httpd_resp_send_chunk(stream_context->req, data, data_length);
}

static esp_err_t sonar_log_stream_response(httpd_req_t *req,
                                           bool as_download) {
    db_sonar_log_status_t status = {0};
    esp_err_t status_err = db_sonar_log_get_status(&status);
    if (status_err != ESP_OK || !status.mounted) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                            "persistent sonar log unavailable");
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "text/plain");
    if (as_download) {
        httpd_resp_set_hdr(req, "Content-Disposition",
                           "attachment; filename=\"sonar-log.txt\"");
    }

    if (status.log_file_bytes == 0) {
        return httpd_resp_send(req,
                               "No persistent sonar log entries yet.\n",
                               HTTPD_RESP_USE_STRLEN);
    }

    sonar_log_http_stream_context_t stream_context = {.req = req};
    size_t bytes_streamed = 0;
    int file_errno = 0;
    esp_err_t stream_err = db_sonar_log_stream(
        sonar_log_http_chunk_writer, &stream_context, &bytes_streamed,
        &file_errno);

    if (stream_err != ESP_OK) {
        if (bytes_streamed == 0) {
            char error_message[160];
            snprintf(error_message, sizeof(error_message),
                     "failed to read persistent sonar log: %s (errno=%d: %s)",
                     esp_err_to_name(stream_err), file_errno,
                     file_errno != 0 ? strerror(file_errno) :
                                       "no filesystem errno");
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                error_message);
        }
        /*
         * If chunks were already sent, a new HTTP error response would corrupt
         * the partial download. Returning failure lets the server close that
         * response while the logger has already released its mutex and file.
         */
        return ESP_FAIL;
    }

    return httpd_resp_send_chunk(req, NULL, 0);
}

static esp_err_t sonar_log_get_handler(httpd_req_t *req) {
    return sonar_log_stream_response(req, false);
}

static esp_err_t sonar_log_download_get_handler(httpd_req_t *req) {
    return sonar_log_stream_response(req, true);
}

static esp_err_t sonar_log_raw_download_get_handler(httpd_req_t *req) {
    rest_server_context_t *rest_context = (rest_server_context_t *)req->user_ctx;
    if (rest_context == NULL) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                            "missing server context");
        return ESP_FAIL;
    }

    const esp_partition_t *logs_partition = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY,
        DB_SONAR_LOG_PARTITION_LABEL);
    if (logs_partition == NULL) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND,
                            "logs partition not found");
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "application/octet-stream");
    httpd_resp_set_hdr(req, "Content-Disposition",
                       "attachment; filename=\"logs-partition.bin\"");

    size_t offset = 0;
    while (offset < logs_partition->size) {
        size_t chunk_size = logs_partition->size - offset;
        if (chunk_size > SCRATCH_BUFSIZE) {
            chunk_size = SCRATCH_BUFSIZE;
        }

        esp_err_t err =
            esp_partition_read(logs_partition, offset, rest_context->scratch,
                               chunk_size);
        if (err != ESP_OK) {
            if (offset == 0) {
                httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                    esp_err_to_name(err));
            }
            return ESP_FAIL;
        }

        err = httpd_resp_send_chunk(req, rest_context->scratch, chunk_size);
        if (err != ESP_OK) {
            ESP_LOGW(TAG,
                     "Raw sonar log partition stream stopped after %u bytes "
                     "(%s)",
                     (unsigned int)offset, esp_err_to_name(err));
            return ESP_FAIL;
        }

        offset += chunk_size;
    }

    return httpd_resp_send_chunk(req, NULL, 0);
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

static const char *db_location_entity_name(uint8_t entity_type) {
    switch (entity_type) {
    case DB_LOCATION_ENTITY_LAKE:
        return "lake";
    case DB_LOCATION_ENTITY_SWIM:
        return "swim";
    case DB_LOCATION_ENTITY_POINT:
        return "point";
    case DB_LOCATION_ENTITY_TRIP:
        return "trip";
    case DB_LOCATION_ENTITY_USAGE:
        return "usage";
    default:
        return "unknown";
    }
}

static void db_location_u64_hex(uint64_t value, char output[17]) {
    snprintf(output, 17, "%016llx", (unsigned long long)value);
}

static esp_err_t location_db_status_get_handler(httpd_req_t *req) {
    db_location_status_t status = {0};
    db_location_store_get_status(&status);
    cJSON *root = cJSON_CreateObject();
    if (root == NULL) {
        return ESP_ERR_NO_MEM;
    }
    char uuid[17];
    char content_crc[9];
    db_location_u64_hex(status.database_uuid, uuid);
    snprintf(content_crc, sizeof(content_crc), "%08lx",
             (unsigned long)status.content_crc32c);
    cJSON_AddBoolToObject(root, "available", status.available);
    cJSON_AddBoolToObject(root, "recovery_fault", status.recovery_fault);
    cJSON_AddStringToObject(root, "database_uuid", uuid);
    cJSON_AddNumberToObject(root, "format_version",
                           DB_LOCATION_FORMAT_VERSION);
    cJSON_AddNumberToObject(root, "protocol_version",
                           DB_LOCATION_PROTOCOL_VERSION);
    cJSON_AddNumberToObject(root, "generation", status.generation);
    cJSON_AddStringToObject(root, "content_crc32c", content_crc);
    cJSON_AddNumberToObject(root, "snapshot_sequence",
                           status.snapshot_sequence);
    cJSON_AddNumberToObject(root, "record_count", status.record_count);
    char active[2] = {status.active_partition, '\0'};
    cJSON_AddStringToObject(root, "active_partition",
                            status.available ? active : "");
    cJSON_AddNumberToObject(root, "active_slot", status.active_slot);
    cJSON_AddNumberToObject(root, "valid_slots_a", status.valid_slots_a);
    cJSON_AddNumberToObject(root, "valid_slots_b", status.valid_slots_b);
    cJSON_AddNumberToObject(root, "newest_generation_a",
                           status.newest_generation_a);
    cJSON_AddNumberToObject(root, "newest_generation_b",
                           status.newest_generation_b);
    cJSON_AddNumberToObject(root, "partition_size_a",
                           (double)status.partition_size_a);
    cJSON_AddNumberToObject(root, "partition_size_b",
                           (double)status.partition_size_b);
    cJSON_AddNumberToObject(root, "maximum_records",
                           DB_LOCATION_MAX_RECORDS);
    cJSON_AddNumberToObject(root, "record_size_bytes",
                           DB_LOCATION_RECORD_SIZE);

    char *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (json == NULL) {
        return ESP_ERR_NO_MEM;
    }
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    esp_err_t err = httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
    free(json);
    return err;
}

typedef struct {
    httpd_req_t *request;
    bool first;
} db_location_export_context_t;

static esp_err_t db_location_export_record(
    const db_location_record_t *record, void *context) {
    db_location_export_context_t *export_context = context;
    cJSON *json_record = cJSON_CreateObject();
    if (json_record == NULL) {
        return ESP_ERR_NO_MEM;
    }
    char entity_id[17];
    char parent_id[17];
    char name[DB_LOCATION_NAME_BYTES + 1];
    db_location_u64_hex(record->entity_id, entity_id);
    db_location_u64_hex(record->parent_id, parent_id);
    size_t name_length = record->name_length;
    if (name_length > DB_LOCATION_NAME_BYTES) {
        name_length = DB_LOCATION_NAME_BYTES;
    }
    memcpy(name, record->name, name_length);
    name[name_length] = '\0';
    cJSON_AddStringToObject(json_record, "entity_id", entity_id);
    cJSON_AddStringToObject(json_record, "type",
                            db_location_entity_name(record->entity_type));
    cJSON_AddStringToObject(json_record, "parent_id", parent_id);
    cJSON_AddNumberToObject(json_record, "revision", record->revision);
    cJSON_AddNumberToObject(json_record, "flags", record->flags);
    cJSON_AddStringToObject(json_record, "name", name);
    cJSON_AddNumberToObject(json_record, "created_unix",
                           record->created_unix);
    cJSON_AddNumberToObject(json_record, "updated_unix",
                           record->updated_unix);
    if (record->entity_type == DB_LOCATION_ENTITY_SWIM) {
        cJSON_AddNumberToObject(json_record, "mapping_revision",
                               record->mapping_revision);
    } else if (record->entity_type == DB_LOCATION_ENTITY_POINT) {
        cJSON_AddNumberToObject(json_record, "active_code",
                               record->active_code);
        cJSON_AddNumberToObject(json_record, "role", record->role);
        cJSON_AddNumberToObject(json_record, "latitude_e7",
                               record->latitude_e7);
        cJSON_AddNumberToObject(json_record, "longitude_e7",
                               record->longitude_e7);
        cJSON_AddNumberToObject(json_record, "depth_mm", record->depth_mm);
        cJSON_AddNumberToObject(json_record, "saved_unix",
                               record->saved_unix);
        cJSON_AddNumberToObject(json_record, "navigation_revision",
                               record->navigation_revision);
    }
    char *json = cJSON_PrintUnformatted(json_record);
    cJSON_Delete(json_record);
    if (json == NULL) {
        return ESP_ERR_NO_MEM;
    }
    esp_err_t err = ESP_OK;
    if (!export_context->first) {
        err = httpd_resp_send_chunk(export_context->request, ",", 1);
    }
    if (err == ESP_OK) {
        err = httpd_resp_send_chunk(export_context->request, json,
                                    HTTPD_RESP_USE_STRLEN);
    }
    export_context->first = false;
    free(json);
    return err;
}

static esp_err_t location_db_export_get_handler(httpd_req_t *req) {
    db_location_status_t status = {0};
    db_location_store_get_status(&status);
    if (!status.available) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                            "location database unavailable");
        return ESP_FAIL;
    }
    char uuid[17];
    char crc[9];
    char prefix[256];
    db_location_u64_hex(status.database_uuid, uuid);
    snprintf(crc, sizeof(crc), "%08lx",
             (unsigned long)status.content_crc32c);
    snprintf(prefix, sizeof(prefix),
             "{\"format\":\"dbb-location-export-v1\","
             "\"database_uuid\":\"%s\",\"generation\":%lu,"
             "\"content_crc32c\":\"%s\",\"records\":[",
             uuid, (unsigned long)status.generation, crc);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_set_hdr(req, "Content-Disposition",
                       "attachment; filename=\"dbb-location-db.json\"");
    esp_err_t err = httpd_resp_send_chunk(req, prefix, HTTPD_RESP_USE_STRLEN);
    db_location_export_context_t context = {.request = req, .first = true};
    if (err == ESP_OK) {
        err = db_location_store_foreach(db_location_export_record, &context);
    }
    if (err == ESP_OK) {
        err = httpd_resp_send_chunk(req, "]}", 2);
    }
    if (err == ESP_OK) {
        err = httpd_resp_send_chunk(req, NULL, 0);
    }
    return err;
}

#if CONFIG_DB_LOCATION_BENCH_TEST
static esp_err_t location_db_bench_post_handler(httpd_req_t *req) {
    char body[128];
    if (req->content_len <= 0 || req->content_len >= (int)sizeof(body)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                            "small JSON action required");
        return ESP_FAIL;
    }
    int received_total = 0;
    while (received_total < req->content_len) {
        int received = httpd_req_recv(req, body + received_total,
                                      req->content_len - received_total);
        if (received <= 0) {
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                "request receive failed");
            return ESP_FAIL;
        }
        received_total += received;
    }
    body[received_total] = '\0';
    cJSON *root = cJSON_Parse(body);
    cJSON *action = root == NULL ? NULL : cJSON_GetObjectItem(root, "action");
    if (!cJSON_IsString(action) || action->valuestring == NULL) {
        cJSON_Delete(root);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid action");
        return ESP_FAIL;
    }
    bool reboot = false;
    esp_err_t err;
    if (strcmp(action->valuestring, "seed") == 0) {
        err = db_location_store_bench_seed();
    } else if (strcmp(action->valuestring, "rename-point") == 0) {
        err = db_location_store_bench_rename_point();
    } else if (strcmp(action->valuestring, "corrupt-active-header") == 0) {
        err = db_location_store_bench_corrupt_active_header();
        reboot = err == ESP_OK;
    } else {
        err = ESP_ERR_INVALID_ARG;
    }
    cJSON_Delete(root);
    if (err != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                            esp_err_to_name(err));
        return ESP_FAIL;
    }
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_sendstr(req, reboot
                               ? "{\"status\":\"active DB header erased; rebooting\"}"
                               : "{\"status\":\"committed\"}");
    if (reboot) {
        vTaskDelay(pdMS_TO_TICKS(250));
        esp_restart();
    }
    return ESP_OK;
}
#endif

typedef struct {
    httpd_req_t *request;
} db_diag_http_export_context_t;

static void db_diag_sha256_to_hex(const uint8_t sha[32], char output[65]) {
    static const char digits[] = "0123456789abcdef";
    for (size_t i = 0; i < 32; ++i) {
        output[i * 2] = digits[sha[i] >> 4];
        output[i * 2 + 1] = digits[sha[i] & 0x0f];
    }
    output[64] = '\0';
}

static esp_err_t db_diag_http_send_record(const db_diag_record_t *record,
                                          size_t sector_index,
                                          void *context) {
    db_diag_http_export_context_t *export_context = context;
    cJSON *root = cJSON_CreateObject();
    if (root == NULL) {
        return ESP_ERR_NO_MEM;
    }

    char app_sha256[65];
    db_diag_sha256_to_hex(record->app_elf_sha256, app_sha256);
    cJSON_AddNumberToObject(root, "sequence", (double)record->sequence);
    cJSON_AddNumberToObject(root, "sector", (double)sector_index);
    cJSON_AddNumberToObject(root, "uptime_ms", (double)record->uptime_ms);
    cJSON_AddNumberToObject(root, "unix_time", (double)record->unix_time);
    cJSON_AddNumberToObject(root, "reset_reason", record->reset_reason);
    cJSON_AddStringToObject(root, "reset_reason_name",
                            db_diag_reset_reason_name(record->reset_reason));
    cJSON_AddNumberToObject(root, "event_type", record->event_type);
    cJSON_AddStringToObject(root, "event",
                            db_diag_event_name(record->event_type));
    cJSON_AddStringToObject(root, "severity",
                            db_diag_severity_name(record->severity));
    cJSON_AddBoolToObject(root, "coredump_present",
                          (record->flags & DB_DIAG_FLAG_COREDUMP_PRESENT) != 0);
    cJSON_AddNumberToObject(root, "ota_subtype", record->ota_subtype);
    cJSON_AddNumberToObject(root, "secure_version", record->secure_version);
    cJSON_AddStringToObject(root, "app_elf_sha256", app_sha256);
    cJSON_AddStringToObject(root, "app_version", record->app_version);
    cJSON_AddStringToObject(root, "build_date", record->build_date);
    cJSON_AddStringToObject(root, "build_time", record->build_time);
    cJSON_AddStringToObject(root, "message", record->message);

    char *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (json == NULL) {
        return ESP_ERR_NO_MEM;
    }
    esp_err_t err = httpd_resp_send_chunk(export_context->request, json,
                                           HTTPD_RESP_USE_STRLEN);
    free(json);
    if (err == ESP_OK) {
        err = httpd_resp_send_chunk(export_context->request, "\n", 1);
    }
    return err;
}

static esp_err_t diagnostics_status_get_handler(httpd_req_t *req) {
    db_diag_status_t journal = {0};
    db_diag_get_status(&journal);

    cJSON *root = cJSON_CreateObject();
    if (root == NULL) {
        return ESP_ERR_NO_MEM;
    }
    cJSON *journal_json = cJSON_AddObjectToObject(root, "journal");
    cJSON_AddBoolToObject(journal_json, "available", journal.available);
    cJSON_AddNumberToObject(journal_json, "partition_size_bytes",
                            (double)journal.partition_size);
    cJSON_AddNumberToObject(journal_json, "sector_count",
                            (double)journal.sector_count);
    cJSON_AddNumberToObject(journal_json, "valid_records",
                            (double)journal.valid_records);
    cJSON_AddNumberToObject(journal_json, "programmed_invalid_records",
                            (double)journal.programmed_invalid_records);
    cJSON_AddNumberToObject(journal_json, "latest_sequence",
                            (double)journal.latest_sequence);

    cJSON *coredump = cJSON_AddObjectToObject(root, "coredump");
#if CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH
    cJSON_AddBoolToObject(coredump, "enabled", true);
#else
    cJSON_AddBoolToObject(coredump, "enabled", false);
#endif
    size_t image_address = 0;
    size_t image_size = 0;
    esp_err_t image_err =
        esp_core_dump_image_get(&image_address, &image_size);
    bool present = image_err == ESP_OK && image_size > 0;
    cJSON_AddBoolToObject(coredump, "present", present);
    cJSON_AddNumberToObject(coredump, "size_bytes",
                            present ? (double)image_size : 0);
    cJSON_AddStringToObject(coredump, "image_status",
                            esp_err_to_name(image_err));
    if (present) {
#if CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH
        esp_err_t check_err = esp_core_dump_image_check();
        cJSON_AddBoolToObject(coredump, "valid", check_err == ESP_OK);
        cJSON_AddStringToObject(coredump, "integrity_status",
                                esp_err_to_name(check_err));
#else
        cJSON_AddBoolToObject(coredump, "valid", false);
        cJSON_AddStringToObject(coredump, "integrity_status",
                                "integrity check unavailable in this build");
#endif
#if CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH && CONFIG_ESP_COREDUMP_DATA_FORMAT_ELF
        char panic_reason[200] = {0};
        if (esp_core_dump_get_panic_reason(panic_reason,
                                           sizeof(panic_reason)) == ESP_OK) {
            cJSON_AddStringToObject(coredump, "panic_reason", panic_reason);
        }
        esp_core_dump_summary_t summary = {0};
        if (esp_core_dump_get_summary(&summary) == ESP_OK) {
            cJSON_AddStringToObject(coredump, "crashed_task",
                                    summary.exc_task);
            cJSON_AddNumberToObject(coredump, "exception_pc",
                                    summary.exc_pc);
            cJSON_AddStringToObject(coredump, "app_elf_sha256",
                                    (const char *)summary.app_elf_sha256);
        }
#endif
    }

    char *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (json == NULL) {
        return ESP_ERR_NO_MEM;
    }
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    esp_err_t err = httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
    free(json);
    return err;
}

static esp_err_t diagnostics_journal_get_handler(httpd_req_t *req) {
    db_diag_status_t status = {0};
    db_diag_get_status(&status);
    if (!status.available) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND,
                            "diagnostic journal unavailable");
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "application/x-ndjson");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_set_hdr(req, "Content-Disposition",
                       "attachment; filename=\"dbb-diagnostics.ndjson\"");
    db_diag_http_export_context_t context = {.request = req};
    esp_err_t err =
        db_diag_foreach_chronological(db_diag_http_send_record, &context);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Diagnostic journal export stopped (%s)",
                 esp_err_to_name(err));
        return ESP_FAIL;
    }
    return httpd_resp_send_chunk(req, NULL, 0);
}

static esp_err_t diagnostics_coredump_get_handler(httpd_req_t *req) {
    size_t image_address = 0;
    size_t image_size = 0;
    esp_err_t err = esp_core_dump_image_get(&image_address, &image_size);
    if (err != ESP_OK || image_size == 0) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND,
                            "no coredump is stored");
        return ESP_FAIL;
    }

    const esp_partition_t *partition = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_COREDUMP,
        "coredump");
    if (partition == NULL || image_address < partition->address ||
        image_size > partition->size ||
        image_address - partition->address > partition->size - image_size) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                            "invalid coredump bounds");
        return ESP_FAIL;
    }

    rest_server_context_t *rest_context = req->user_ctx;
    httpd_resp_set_type(req, "application/octet-stream");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_set_hdr(req, "Content-Disposition",
                       "attachment; filename=\"dbb-coredump.elf\"");
    size_t offset = image_address - partition->address;
    size_t sent = 0;
    while (sent < image_size) {
        size_t chunk = image_size - sent;
        if (chunk > SCRATCH_BUFSIZE) {
            chunk = SCRATCH_BUFSIZE;
        }
        err = esp_partition_read(partition, offset + sent,
                                 rest_context->scratch, chunk);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Coredump read failed at %u (%s)",
                     (unsigned)sent, esp_err_to_name(err));
            return ESP_FAIL;
        }
        err = httpd_resp_send_chunk(req, rest_context->scratch, chunk);
        if (err != ESP_OK) {
            return ESP_FAIL;
        }
        sent += chunk;
    }
    return httpd_resp_send_chunk(req, NULL, 0);
}

#if CONFIG_DB_DIAG_BENCH_CRASH_TEST
static esp_err_t diagnostics_bench_crash_post_handler(httpd_req_t *req) {
    esp_err_t err = db_diag_log_event(DB_DIAG_EVENT_BENCH_CRASH_ARMED,
                                      DB_DIAG_SEVERITY_WARNING,
                                      (uint32_t)esp_reset_reason(),
                                      "owner-authorized bench crash test");
    if (err != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                            "failed to persist crash-test marker");
        return ESP_FAIL;
    }
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req,
                       "{\"status\":\"crash armed; device will reboot\"}");
    vTaskDelay(pdMS_TO_TICKS(250));
    abort();
    return ESP_FAIL;
}

static esp_err_t diagnostics_bench_coredump_delete_handler(httpd_req_t *req) {
    esp_err_t err = esp_core_dump_image_erase();
    if (err != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                            esp_err_to_name(err));
        return ESP_FAIL;
    }
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"status\":\"coredump erased\"}");
}
#endif

esp_err_t start_rest_server(const char *base_path) {
    REST_CHECK(base_path, "wrong base path", err);
    rest_server_context_t *rest_context = calloc(1, sizeof(rest_server_context_t));
    REST_CHECK(rest_context, "No memory for rest context", err);
    strlcpy(rest_context->base_path, base_path, sizeof(rest_context->base_path));

    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.uri_match_fn = httpd_uri_match_wildcard;
    config.max_uri_handlers = 36;
    config.stack_size = DB_HTTP_SERVER_STACK_SIZE;
    config.max_open_sockets = 12;      // raised with LWIP_MAX_SOCKETS=24 (BLE off freed the RAM) — comfortable multi-browser headroom
    config.lru_purge_enable = true;    // pool full -> recycle the stalest idle connection instead of rejecting (fixes the blank page on a 2nd/3rd client)

    ESP_LOGI(TAG, "Starting HTTP Server");
    REST_CHECK(httpd_start(&server, &config) == ESP_OK,
               "Start HTTP server failed", err_start);

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

    httpd_uri_t telemetry_get_uri = {
            .uri = "/api/telemetry",
            .method = HTTP_GET,
            .handler = telemetry_get_handler,
            .user_ctx = rest_context
    };
    httpd_register_uri_handler(server, &telemetry_get_uri);

    httpd_uri_t telemetry_page_uri = {
            .uri = "/telemetry",
            .method = HTTP_GET,
            .handler = telemetry_page_handler,
            .user_ctx = rest_context
    };
    httpd_register_uri_handler(server, &telemetry_page_uri);

    /* Flight-controller firmware page and its API. Standalone, like /telemetry. */
    httpd_uri_t fcflash_page_uri = {
            .uri = "/fcflash",
            .method = HTTP_GET,
            .handler = fcflash_page_handler,
            .user_ctx = rest_context
    };
    httpd_register_uri_handler(server, &fcflash_page_uri);

    httpd_uri_t fcflash_status_uri = {
            .uri = "/api/fcflash/status",
            .method = HTTP_GET,
            .handler = fcflash_status_get_handler,
            .user_ctx = rest_context
    };
    httpd_register_uri_handler(server, &fcflash_status_uri);

    httpd_uri_t fcflash_upload_uri = {
            .uri = "/api/fcflash/upload",
            .method = HTTP_POST,
            .handler = fcflash_upload_post_handler,
            .user_ctx = rest_context
    };
    httpd_register_uri_handler(server, &fcflash_upload_uri);

    httpd_uri_t fcflash_start_uri = {
            .uri = "/api/fcflash/start",
            .method = HTTP_POST,
            .handler = fcflash_start_post_handler,
            .user_ctx = rest_context
    };
    httpd_register_uri_handler(server, &fcflash_start_uri);

    httpd_uri_t fcflash_delete_uri = {
            .uri = "/api/fcflash/image",
            .method = HTTP_DELETE,
            .handler = fcflash_delete_handler,
            .user_ctx = rest_context
    };
    httpd_register_uri_handler(server, &fcflash_delete_uri);

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

#if CONFIG_WEB_DEPLOY_SF
    httpd_uri_t update_web_post_uri = {
            .uri = "/api/update/web",
            .method = HTTP_POST,
            .handler = update_web_post_handler,
            .user_ctx = rest_context
    };
    httpd_register_uri_handler(server, &update_web_post_uri);
#endif

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

    httpd_uri_t sonar_log_raw_download_get_uri = {
            .uri = "/api/logs/raw/download",
            .method = HTTP_GET,
            .handler = sonar_log_raw_download_get_handler,
            .user_ctx = rest_context
    };
    httpd_register_uri_handler(server, &sonar_log_raw_download_get_uri);

    httpd_uri_t sonar_log_delete_uri = {
            .uri = "/api/logs/sonar",
            .method = HTTP_DELETE,
            .handler = sonar_log_delete_handler,
            .user_ctx = rest_context
    };
    httpd_register_uri_handler(server, &sonar_log_delete_uri);

    httpd_uri_t location_db_status_get_uri = {
            .uri = "/api/location-db/status",
            .method = HTTP_GET,
            .handler = location_db_status_get_handler,
            .user_ctx = rest_context
    };
    httpd_register_uri_handler(server, &location_db_status_get_uri);

    httpd_uri_t location_db_export_get_uri = {
            .uri = "/api/location-db/export",
            .method = HTTP_GET,
            .handler = location_db_export_get_handler,
            .user_ctx = rest_context
    };
    httpd_register_uri_handler(server, &location_db_export_get_uri);

#if CONFIG_DB_LOCATION_BENCH_TEST
    httpd_uri_t location_db_bench_post_uri = {
            .uri = "/api/location-db/bench",
            .method = HTTP_POST,
            .handler = location_db_bench_post_handler,
            .user_ctx = rest_context
    };
    httpd_register_uri_handler(server, &location_db_bench_post_uri);
#endif

    httpd_uri_t diagnostics_status_get_uri = {
            .uri = "/api/diagnostics/status",
            .method = HTTP_GET,
            .handler = diagnostics_status_get_handler,
            .user_ctx = rest_context
    };
    httpd_register_uri_handler(server, &diagnostics_status_get_uri);

    httpd_uri_t diagnostics_journal_get_uri = {
            .uri = "/api/diagnostics/journal",
            .method = HTTP_GET,
            .handler = diagnostics_journal_get_handler,
            .user_ctx = rest_context
    };
    httpd_register_uri_handler(server, &diagnostics_journal_get_uri);

    httpd_uri_t diagnostics_coredump_get_uri = {
            .uri = "/api/diagnostics/coredump",
            .method = HTTP_GET,
            .handler = diagnostics_coredump_get_handler,
            .user_ctx = rest_context
    };
    httpd_register_uri_handler(server, &diagnostics_coredump_get_uri);

#if CONFIG_DB_DIAG_BENCH_CRASH_TEST
    httpd_uri_t diagnostics_bench_crash_post_uri = {
            .uri = "/api/diagnostics/bench-crash",
            .method = HTTP_POST,
            .handler = diagnostics_bench_crash_post_handler,
            .user_ctx = rest_context
    };
    httpd_register_uri_handler(server, &diagnostics_bench_crash_post_uri);

    httpd_uri_t diagnostics_bench_coredump_delete_uri = {
            .uri = "/api/diagnostics/bench-coredump",
            .method = HTTP_DELETE,
            .handler = diagnostics_bench_coredump_delete_handler,
            .user_ctx = rest_context
    };
    httpd_register_uri_handler(server,
                               &diagnostics_bench_coredump_delete_uri);
#endif

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
