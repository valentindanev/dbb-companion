#include <sys/cdefs.h>
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
#include <sys/cdefs.h>
#include <sys/fcntl.h>
#include <sys/param.h>
#include <ctype.h>
#include <stdio.h>
#include <string.h>
#include <esp_task_wdt.h>
#include <esp_vfs_dev.h>
#include <lwip/inet.h>
#include <esp_timer.h>
#include <esp_wifi.h>
#include <stdint-gcc.h>
#include <sys/types.h>
#include <lwip/netdb.h>
#include "esp_log.h"
#include "lwip/sockets.h"
#include "driver/uart.h"
#include "globals.h"
#include "db_protocol.h"
#include "tcp_server.h"
#include "db_esp32_control.h"
#ifdef CONFIG_BT_ENABLED
#include "db_ble.h"
#endif
#include <db_parameters.h>
#include "main.h"
#include "db_serial.h"
#include "freertos/semphr.h"

#define TAG "DB_CONTROL"
#define DB_PACKET_LOG_PREVIEW_BYTES 48
#define DB_DEEPER_DEBUG_MAX_LINES 8
#define DB_DEEPER_DEBUG_LINE_MAX 256

uint16_t app_port_proxy = APP_PORT_PROXY;

udp_conn_list_t *udp_conn_list;
int connected_tcp_clients[CONFIG_LWIP_MAX_ACTIVE_TCP];
int8_t num_connected_tcp_clients = 0;
static SemaphoreHandle_t g_deeper_debug_mutex = NULL;
static char g_deeper_debug_lines[DB_DEEPER_DEBUG_MAX_LINES][DB_DEEPER_DEBUG_LINE_MAX];
static size_t g_deeper_debug_count = 0;
static size_t g_deeper_debug_next = 0;

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

static void db_deeper_debug_init(void) {
    if (g_deeper_debug_mutex == NULL) {
        g_deeper_debug_mutex = xSemaphoreCreateMutex();
    }
}

static void db_store_deeper_debug_line(const char *line) {
    db_deeper_debug_init();
    if (g_deeper_debug_mutex == NULL || line == NULL) {
        return;
    }

    xSemaphoreTake(g_deeper_debug_mutex, portMAX_DELAY);
    strncpy(g_deeper_debug_lines[g_deeper_debug_next], line,
            DB_DEEPER_DEBUG_LINE_MAX - 1);
    g_deeper_debug_lines[g_deeper_debug_next][DB_DEEPER_DEBUG_LINE_MAX - 1] =
        '\0';
    g_deeper_debug_next =
        (g_deeper_debug_next + 1) % DB_DEEPER_DEBUG_MAX_LINES;
    if (g_deeper_debug_count < DB_DEEPER_DEBUG_MAX_LINES) {
        g_deeper_debug_count++;
    }
    xSemaphoreGive(g_deeper_debug_mutex);
}

void db_append_deeper_debug_line(const char *line) {
    db_store_deeper_debug_line(line);
}

void db_clear_deeper_debug_log(void) {
    db_deeper_debug_init();
    if (g_deeper_debug_mutex == NULL) {
        return;
    }

    xSemaphoreTake(g_deeper_debug_mutex, portMAX_DELAY);
    for (size_t i = 0; i < DB_DEEPER_DEBUG_MAX_LINES; i++) {
        g_deeper_debug_lines[i][0] = '\0';
    }
    g_deeper_debug_count = 0;
    g_deeper_debug_next = 0;
    xSemaphoreGive(g_deeper_debug_mutex);
}

void db_get_deeper_debug_log(char *dst, size_t dst_size) {
    if (dst == NULL || dst_size == 0) {
        return;
    }

    db_deeper_debug_init();
    if (g_deeper_debug_mutex == NULL) {
        snprintf(dst, dst_size, "Deeper debug buffer unavailable.");
        return;
    }

    xSemaphoreTake(g_deeper_debug_mutex, portMAX_DELAY);
    if (g_deeper_debug_count == 0) {
        snprintf(dst, dst_size,
                 "No Deeper packets captured yet. This panel fills only while "
                 "the ESP32 is running in STA mode and receives traffic.");
        xSemaphoreGive(g_deeper_debug_mutex);
        return;
    }

    dst[0] = '\0';
    size_t oldest_index =
        (g_deeper_debug_next + DB_DEEPER_DEBUG_MAX_LINES - g_deeper_debug_count) %
        DB_DEEPER_DEBUG_MAX_LINES;
    for (size_t i = 0; i < g_deeper_debug_count; i++) {
        size_t line_index = (oldest_index + i) % DB_DEEPER_DEBUG_MAX_LINES;
        if (dst[0] != '\0') {
            strncat(dst, "\n", dst_size - strlen(dst) - 1);
        }
        strncat(dst, g_deeper_debug_lines[line_index],
                dst_size - strlen(dst) - 1);
    }
    xSemaphoreGive(g_deeper_debug_mutex);
}

static void db_log_payload_preview(const char *label, const uint8_t *data,
                                   size_t data_length) {
    if (data == NULL || data_length == 0) {
        return;
    }

    size_t preview_length =
        data_length < DB_PACKET_LOG_PREVIEW_BYTES ? data_length
                                                  : DB_PACKET_LOG_PREVIEW_BYTES;
    char ascii[DB_PACKET_LOG_PREVIEW_BYTES + 1];
    char hex[(DB_PACKET_LOG_PREVIEW_BYTES * 3) + 1];
    size_t hex_pos = 0;

    for (size_t i = 0; i < preview_length; i++) {
        unsigned char byte = data[i];
        ascii[i] = isprint(byte) ? (char)byte : '.';
        if (hex_pos + 4 < sizeof(hex)) {
            int written =
                snprintf(&hex[hex_pos], sizeof(hex) - hex_pos, "%02X%s", byte,
                         (i + 1) < preview_length ? " " : "");
            if (written > 0) {
                hex_pos += (size_t)written;
            }
        }
    }
    ascii[preview_length] = '\0';
    hex[hex_pos] = '\0';

    ESP_LOGI(TAG,
             "%s len=%u preview=%u ascii=\"%s\" hex=%s%s",
             label, (unsigned int)data_length, (unsigned int)preview_length,
             ascii, hex, data_length > preview_length ? " ..." : "");

}

/**
 * Opens non-blocking UDP socket used for WiFi to UART communication. Does also accept broadcast packets just in case.
 * @return returns socket file descriptor
 */
int db_open_serial_udp_socket() {
    struct sockaddr_in server_addr;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(APP_PORT_PROXY_UDP);
    int addr_family = AF_INET;
    int ip_protocol = IPPROTO_IP;
    char addr_str[128];
    inet_ntoa_r(server_addr.sin_addr, addr_str, sizeof(addr_str) - 1);

    int udp_socket = socket(addr_family, SOCK_DGRAM, ip_protocol);
    if (udp_socket < 0) {
        ESP_LOGE(TAG, "Unable to create socket for serial communication: errno %d", errno);
        return -1;
    }
    int broadcastEnable = 1;
    int err = setsockopt(udp_socket, SOL_SOCKET, SO_BROADCAST, &broadcastEnable, sizeof(broadcastEnable));
    if (err < 0) {
        ESP_LOGE(TAG, "Socket unable to set udp socket to accept broadcast messages: errno %d", errno);
    }
    err = bind(udp_socket, (struct sockaddr *) &server_addr, sizeof(server_addr));
    if (err < 0) {
        ESP_LOGE(TAG, "Socket unable to bind to %i errno %d", APP_PORT_PROXY_UDP, errno);
    }
    fcntl(udp_socket, F_SETFL, O_NONBLOCK);
    ESP_LOGI(TAG, "Opened UDP socket on port %i", APP_PORT_PROXY_UDP);
    return udp_socket;
}

/**
 * Sends data to all clients that are part of the udp connection list. No resending of packets in case of failure.
 * Special behavior in DroneShow Edition:
 *  - If there is no UDP client detected or manually set, we send packets to broadcast address and pre-defined port so
 *    that the GCS can detect the ESP32.
 *  - This is a feature that can be enabled/disabled by the user together with the target port
 *
 * @param data Buffer with the data to send
 * @param data_length Length of the data in the buffer
 */
void db_send_to_all_udp_clients(const uint8_t *data, uint data_length) {
    for (int i = 0; i < udp_conn_list->size; i++) {  // send to all UDP clients
        int sent = sendto(udp_conn_list->udp_socket, data, data_length, 0,
                          (struct sockaddr *) &udp_conn_list->db_udp_clients[i].udp_client,
                          sizeof(udp_conn_list->db_udp_clients[i].udp_client));
        if (sent != data_length) {
            int err = errno;
            char *client_ip = inet_ntoa(((struct sockaddr_in *)&udp_conn_list->db_udp_clients[i].udp_client)->sin_addr);
            ESP_LOGE(TAG, "UDP - Error sending (%i/%i) to %s because of %s", sent, data_length, client_ip, strerror(err));
        }
    }
}

/**
 * Main call for sending anything over the air.
 * Send to all connected TCP & UDP clients depending on the mode (DB_WIFI_MODE) we are currently in.
 * Typically called by a function that read from UART.
 *
 * @param tcp_clients Array of socket IDs for the TCP clients
 * @param udp_conn Structure handling the UDP connection
 * @param data payload to send
 * @param data_length Length of payload to send
 */
void db_send_to_all_clients(int tcp_clients[], udp_conn_list_t *n_udp_conn_list, uint8_t data[], uint16_t data_length) {
#ifdef CONFIG_BT_ENABLED
    db_ble_queue_event_t bleData;
#endif
    switch (DB_PARAM_RADIO_MODE) {
        case DB_BLUETOOTH_MODE:
#ifdef CONFIG_BT_ENABLED
            bleData.data = malloc(data_length);
            bleData.data_len = data_length;
            memcpy(bleData.data, data, bleData.data_len);
            if (xQueueSend(db_uart_read_queue_ble, &bleData, portMAX_DELAY) != pdPASS) {
                ESP_LOGE(TAG, "Failed to send BLE data to queue");
                free(bleData.data);
            }
#endif
            break;

        default:
            // Other modes (WiFi Modes using TCP/UDP)
            db_send_to_all_tcp_clients(tcp_clients, data, data_length);
            db_send_to_all_udp_clients(data, data_length);
            break;
    }
}

/**
 * New function to replace the legacy db_send_to_all_clients(). No need to pass tcp & udp clients
 * Sends the message to all connected clients using the radio (WiFi, ESP-NOW or BLE) link.
 * @param data
 * @param data_length
 */
void db_send_to_all_radio_clients(uint8_t data[], uint16_t data_length) {
#ifdef CONFIG_BT_ENABLED
    db_ble_queue_event_t bleData;
#endif
    switch (DB_PARAM_RADIO_MODE) {
        case DB_BLUETOOTH_MODE:
#ifdef CONFIG_BT_ENABLED
            bleData.data = malloc(data_length);
            bleData.data_len = data_length;
            memcpy(bleData.data, data, bleData.data_len);
            if (xQueueSend(db_uart_read_queue_ble, &bleData, portMAX_DELAY) != pdPASS) {
                ESP_LOGE(TAG, "Failed to send BLE data to queue");
                free(bleData.data);
            }
#endif
            break;

        default:
            // Other modes (WiFi Modes using TCP/UDP)
            db_send_to_all_tcp_clients(connected_tcp_clients, data, data_length);
            db_send_to_all_udp_clients(data, data_length);
            break;
    }
}

/**
 * Check for incoming connections on TCP server
 *
 * @param tcp_master_socket Main open TCP socket to accept TCP connections/clients
 * @param tcp_clients List of active TCP client connections (socket IDs)
 */
void handle_tcp_master(const int tcp_master_socket, int tcp_clients[]) {
    struct sockaddr_in6 source_addr; // Large enough for both IPv4 or IPv6
    uint32_t addr_len = sizeof(source_addr);
    int new_tcp_client = accept(tcp_master_socket, (struct sockaddr *) &source_addr, &addr_len);
    if (new_tcp_client > 0) {
        for (int i = 0; i < CONFIG_LWIP_MAX_ACTIVE_TCP; i++) {
            if (tcp_clients[i] <= 0) {
                tcp_clients[i] = new_tcp_client;
                fcntl(tcp_clients[i], F_SETFL, O_NONBLOCK);
                char addr_str[128];
                inet_ntoa_r(((struct sockaddr_in *) &source_addr)->sin_addr.s_addr, addr_str, sizeof(addr_str) - 1);
                ESP_LOGI(TAG, "TCP: New client connected: %s", addr_str);
                num_connected_tcp_clients++;
                return;
            }
        }
        ESP_LOGI(TAG, "TCP: Could not accept connection. Too many clients connected.");
    }
}

/**
 *  Init/Create structure containing all UDP connection information
 * @return Structure containing all UDP connection information
 */
udp_conn_list_t *udp_client_list_create() {
    udp_conn_list_t *n_udp_conn_list = malloc(sizeof(udp_conn_list_t)); // Allocate memory for the list
    if (n_udp_conn_list == NULL) { // Check if the allocation failed
        return NULL; // Return NULL to indicate an error
    }
    n_udp_conn_list->size = 0; // Initialize the size to 0
    return n_udp_conn_list; // Return the pointer to the list
}

/**
 *  Destroy structure containing all UDP connection information
 * @param n_udp_conn_list Structure containing all UDP connection information
 */
void udp_client_list_destroy(udp_conn_list_t *n_udp_conn_list) {
    if (n_udp_conn_list == NULL) { // Check if the list is NULL
        return; // Do nothing
    }
    free(n_udp_conn_list); // Free the list
}

/**
 * Add a new UDP client to the list of known UDP clients. Checks if client is already known based on IP and port.
 * Added client will receive UDP packets with serial info and will be able to send UDP packets to the serial interface
 * of the ESP32.
 * PORT, MAC & IP should be set inside new_db_udp_client. If MAC is not set then the device cannot be removed later on.
 *
 * @param n_udp_conn_list Structure containing all UDP connection information
 * @param new_db_udp_client New client to add to the UDP list. PORT, MAC & IP must be set. If MAC is not set then the
 *                          device cannot be automatically removed later on. To remove it, the user must clear the entire list.
 * @param save_to_nvm Set to 1 (true) in case you want the UDP client to survive the reboot. Set to 0 (false) if client is temporary for this session.
 *                    It will then be saved to NVM and added to the udp_conn_list_t on startup. Only one client can be saved to NVM.
 * @return 1 if added - 0 if not
 */
bool
add_to_known_udp_clients(udp_conn_list_t *n_udp_conn_list, struct db_udp_client_t new_db_udp_client, bool save_to_nvm) {
    if (n_udp_conn_list == NULL) { // Check if the list is NULL
        return false; // Do nothing
    }
    if (n_udp_conn_list->size == MAX_UDP_CLIENTS) { // Check if the list is full
        return false; // Do nothing
    }
    for (int i = 0; i < n_udp_conn_list->size; i++) {
        if ((n_udp_conn_list->db_udp_clients[i].udp_client.sin_port == new_db_udp_client.udp_client.sin_port) &&
            (n_udp_conn_list->db_udp_clients[i].udp_client.sin_addr.s_addr ==
             new_db_udp_client.udp_client.sin_addr.s_addr)) {
            return false; // client existing - do not add
        }
    }
    n_udp_conn_list->db_udp_clients[n_udp_conn_list->size] = new_db_udp_client; // Copy the element data to the end of the array
    n_udp_conn_list->size++; // Increment the size of the list
    // some logging
    char ip_port_string[INET_ADDRSTRLEN + 10];
    char ip_string[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &(new_db_udp_client.udp_client.sin_addr), ip_string, INET_ADDRSTRLEN);
    sprintf(ip_port_string, "%s:%d", ip_string, htons (new_db_udp_client.udp_client.sin_port));
    ESP_LOGI(TAG, "Added %s to udp client distribution list - save to NVM: %i", ip_port_string, save_to_nvm);
    // save to memory
    if (save_to_nvm) {
        save_udp_client_to_nvm(&new_db_udp_client, false);
    } else {
        // do not save to NVM
    }
    return true;
}

/**
 * Remove a client from the sending list. Client will no longer receive UDP packets. MAC address must be given.
 * Usually called in AP-Mode when a station disconnects. In any other case we will not know since UDP is a connection-less
 * protocol
 *
 * @param n_udp_conn_list Structure containing all UDP connection information
 * @param new_db_udp_client The UDP client to remove based on its MAC address
 * @return true if removed - false if nothing was removed
 */
bool remove_from_known_udp_clients(udp_conn_list_t *n_udp_conn_list, struct db_udp_client_t new_db_udp_client) {
    if (n_udp_conn_list == NULL) { // Check if the list is NULL
        return false; // Do nothing
    }
    for (int i = 0; i < n_udp_conn_list->size; i++) { // Loop through the array
        if (memcmp(n_udp_conn_list->db_udp_clients[i].mac, new_db_udp_client.mac,
                   sizeof(n_udp_conn_list->db_udp_clients[i].mac)) ==
            0) { // Compare the current array element with the element
            // Found a match
            for (int j = i; j < n_udp_conn_list->size - 1; j++) { // Loop from the current index to the end of the array
                n_udp_conn_list->db_udp_clients[j] = n_udp_conn_list->db_udp_clients[j +
                                                                                     1]; // Shift the array elements to the left
            }
            n_udp_conn_list->size--; // Decrement the size of the list
            return true; // Exit the function
        }
    }
    // No match found
    return false;
}

/**
 * Reads serial (UART/USB/JTAG) transparently or parsing MAVLink protocol.
 * Then sends read data to all connected clients via TCP/UDP or ESP-NOW.
 * Non-Blocking function
 *
 * @param tcp_clients Array of connected TCP client sockets
 * @param transparent_buff_pos Counter variable for total read UART bytes
 * @param msp_message_buffer Destination-buffer for read serial/MAVLink messages
 * @param serial_buffer Destination-buffer for the serial data
 */
void read_process_serial_link(int *tcp_clients, uint *transparent_buff_pos,
                              uint8_t *msp_message_buffer,
                              uint8_t *serial_buffer) {
    switch (DB_PARAM_SERIAL_PROTO) {
        case DB_SERIAL_PROTOCOL_MAVLINK:
            db_read_serial_parse_mavlink(tcp_clients, udp_conn_list, msp_message_buffer, transparent_buff_pos);
            break;
        case DB_SERIAL_PROTOCOL_TRANSPARENT:
        default:
            db_read_serial_parse_transparent(tcp_clients, udp_conn_list, serial_buffer, transparent_buff_pos);
            break;
    }
}

/**
 * Thread that manages all incoming and outgoing TCP, UDP and serial (UART) connections.
 * Executed when Wi-Fi modes are set - ESP-NOW has its own thread
 */
_Noreturn void control_module_udp_tcp() {
    ESP_LOGI(TAG, "Starting control module (Wi-Fi)");
    esp_err_t serial_socket_status = open_serial_socket();
    if (serial_socket_status == ESP_FAIL) {
        ESP_LOGE(TAG, "JTAG serial socket not opened. Aborting start of control module.");
        vTaskDelete(NULL);
    } else {
#ifdef CONFIG_DB_SERIAL_OPTION_JTAG
        db_jtag_serial_info_print();
#endif
    }

    int tcp_master_socket = open_tcp_server(app_port_proxy);
    if (tcp_master_socket == ESP_FAIL) {
        ESP_LOGE(TAG, "TCP master socket failed to open. Aborting start of control module.");
        vTaskDelete(NULL);
    }
    fcntl(tcp_master_socket, F_SETFL, O_NONBLOCK);
    for (int i = 0; i < CONFIG_LWIP_MAX_ACTIVE_TCP; i++) {
        connected_tcp_clients[i] = -1;
    }

    udp_conn_list->udp_socket = db_open_serial_udp_socket();
    uint8_t udp_buffer[UDP_BUF_SIZE];
    struct db_udp_client_t new_db_udp_client = {0};
    socklen_t udp_socklen = sizeof(new_db_udp_client.udp_client);

    uint transparent_buff_pos = 0;
    uint8_t tcp_client_buffer[TCP_BUFF_SIZ];
    memset(tcp_client_buffer, 0, TCP_BUFF_SIZ);
    uint8_t msp_message_buffer[UART_BUF_SIZE];
    uint8_t serial_buffer[DB_PARAM_SERIAL_PACK_SIZE];
    int delay_timer_cnt = 0;

    ESP_LOGI(TAG, "Started control module (Wi-Fi)");
    while (1) {
        // Read incoming wireless data (Wi-Fi)
        // Wi-Fi based modes that use TCP and UDP communication
        bool data_processed = false;
        uint32_t prev_serial_count = serial_total_byte_count;
        int8_t prev_tcp_clients = num_connected_tcp_clients;

        handle_tcp_master(tcp_master_socket, connected_tcp_clients);
        if (num_connected_tcp_clients != prev_tcp_clients) data_processed = true;
        for (int i = 0; i < CONFIG_LWIP_MAX_ACTIVE_TCP; i++) {  // handle TCP clients
            if (connected_tcp_clients[i] > 0) {
                ssize_t recv_length = recv(connected_tcp_clients[i], tcp_client_buffer, TCP_BUFF_SIZ, 0);
                if (recv_length > 0) {
                    data_processed = true;
                    if (db_wifi_runtime_has_sta()) {
                        char label[32];
                        snprintf(label, sizeof(label), "TCP RX fd=%d", connected_tcp_clients[i]);
                        db_log_payload_preview(label, tcp_client_buffer, (size_t) recv_length);
                    }
                    if (DB_PARAM_SERIAL_PROTO == DB_SERIAL_PROTOCOL_MAVLINK) {
                        // Parse, so we can listen in and react to certain messages - function will send parsed messages to serial link.
                        // We can not write to serial first since we might inject packets and do not know when to do so to not "destroy" an existign packet
                        db_parse_mavlink_from_radio(connected_tcp_clients, udp_conn_list, tcp_client_buffer, recv_length);
                    } else {
                        // no parsing with any other protocol - transparent here
                        write_to_serial(tcp_client_buffer, recv_length);
                    }
                } else if (recv_length == 0) {
                    shutdown(connected_tcp_clients[i], 0);
                    close(connected_tcp_clients[i]);
                    connected_tcp_clients[i] = -1;
                    ESP_LOGI(TAG, "TCP client disconnected");
                    num_connected_tcp_clients--;
                } else if (errno != EAGAIN && errno != EWOULDBLOCK) {
                    ESP_LOGE(TAG, "Error receiving from TCP client %i (fd: %i): %d", i, connected_tcp_clients[i], errno);
                    shutdown(connected_tcp_clients[i], 0);
                    close(connected_tcp_clients[i]);
                    num_connected_tcp_clients--;
                    connected_tcp_clients[i] = -1;
                }
            }
        }
        // handle incoming UDP data on main port 14550 - Read UDP and forward to UART
        ssize_t recv_length = recvfrom(udp_conn_list->udp_socket, udp_buffer, UDP_BUF_SIZE, 0,
                                       (struct sockaddr *) &new_db_udp_client.udp_client, &udp_socklen);
        if (recv_length > 0) {
            data_processed = true;
            if (db_wifi_runtime_has_sta()) {
                char client_ip[IP4ADDR_STRLEN_MAX];
                char label[64];
                inet_ntoa_r(new_db_udp_client.udp_client.sin_addr, client_ip, sizeof(client_ip));
                snprintf(label, sizeof(label), "UDP RX %s:%u", client_ip,
                         ntohs(new_db_udp_client.udp_client.sin_port));
                db_log_payload_preview(label, udp_buffer, (size_t) recv_length);
            }
            if (DB_PARAM_SERIAL_PROTO == DB_SERIAL_PROTOCOL_MAVLINK) {
                // Parse, so we can listen in and react to certain messages - function will send parsed messages to serial link.
                // We can not write to serial first since we might inject packets and do not know when to do so to not "destroy" an existing packet
                db_parse_mavlink_from_radio(connected_tcp_clients, udp_conn_list, udp_buffer, recv_length);
            } else {
                // no parsing with any other protocol - transparent here
                write_to_serial(udp_buffer, recv_length);
            }
            // all devices that send us UDP data will be added to the list of UDP receivers
            // Allows to register new app on different port. Used e.g. for UDP conn setup in sta-mode.
            // Devices/Ports added this way cannot be removed in sta-mode since UDP is connectionless, and we cannot
            // determine if the client is still existing. This will blow up the list connected devices.
            // In AP-Mode the devices can be removed based on the IP/MAC address
            add_to_known_udp_clients(udp_conn_list, new_db_udp_client, false);
        } else {
            // received nothing, keep on going
        }
        // Second check for incoming UART data and send it to TCP/UDP
        read_process_serial_link(connected_tcp_clients, &transparent_buff_pos, msp_message_buffer, serial_buffer);
        if (serial_total_byte_count != prev_serial_count) data_processed = true;

        if (serial_total_byte_count != prev_serial_count) data_processed = true;

        if (delay_timer_cnt >= 6000) {
            // all actions are non-blocking so allow some delay so that the IDLE task of FreeRTOS and the watchdog can run
            // read: https://esp32developer.com/programming-in-c-c/tasks/tasks-vs-co-routines for reference
            vTaskDelay(10 / portTICK_PERIOD_MS);
            delay_timer_cnt = 0;
            // Use the opportunity to get some regular status information like rssi and send them via internal telemetry
            if (db_wifi_runtime_has_sta()) {
                // update rssi variable - set to -127 when not available
                if (esp_wifi_sta_get_rssi((int *) &db_esp_signal_quality.air_rssi) != ESP_OK) {
                    db_esp_signal_quality.air_rssi = -127;
                } else {/* all good */}
            } else if (!DB_RADIO_IS_OFF && db_wifi_runtime_has_ap()) {
                ESP_ERROR_CHECK_WITHOUT_ABORT(
                        esp_wifi_ap_get_sta_list(&wifi_sta_list)); // update list of connected stations
            } else {
                // no way of getting RSSI here. Do nothing
            }
//            size_t free_dram = heap_caps_get_free_size(MALLOC_CAP_8BIT);
//            size_t lowmark_dram = heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT);
//            ESP_LOGI(TAG, "Free heap: %i, low mark: %i", free_dram, lowmark_dram);
        } else {
            delay_timer_cnt++;
        }

        if (!data_processed) {
            vTaskDelay(pdMS_TO_TICKS(1));   // Add a delay of 1 ms since we did not read anything in the last cycle
        }
    }
    vTaskDelete(NULL);
}

#ifdef CONFIG_BT_ENABLED

_Noreturn void control_module_ble() {
    ESP_LOGI(TAG, "Starting control module (Bluetooth)");

    /* Initialize error code as failed, because UART is not initialized*/
    esp_err_t serial_socket = ESP_FAIL;

    /* open serial socket for comms with FC or GCS */
    serial_socket = open_serial_socket();
    if (serial_socket == ESP_FAIL) {
        ESP_LOGE(TAG, "UART socket not opened. Aborting start of control module.");
        vTaskDelete(NULL);
    }
#ifdef CONFIG_DB_SERIAL_OPTION_JTAG
    else {
      db_jtag_serial_info_print();
    }
#endif

    uint8_t msp_message_buffer[UART_BUF_SIZE];
    uint8_t serial_buffer[DB_PARAM_SERIAL_PACK_SIZE];
    db_ble_queue_event_t bleData;
    uint transparent_buff_pos = 0;
    uint delay_timer_cnt = 0;

    /* Event Loop */
    while (1) {
        /* Read UART and send data to BLE */
        read_process_serial_link(NULL,                  // NULL, not using TCP
                                 &transparent_buff_pos, // transparent buffer position
                                 msp_message_buffer,    // serial msg buffer
                                 serial_buffer          // serial buffer
        );

        if (db_uart_write_queue_ble != NULL && xQueueReceive(db_uart_write_queue_ble, &bleData, 1) == pdTRUE) {
            if (DB_PARAM_SERIAL_PROTO == DB_SERIAL_PROTOCOL_MAVLINK) {
                // Parse, so we can listen in and react to certain messages - function will send parsed messages to serial link.
                // We cannot write to serial first since we might inject packets and do not know when to do so to not "destroy" an
                // existing packet
                db_parse_mavlink_from_radio(NULL, NULL, bleData.data, bleData.data_len);
            } else {
                // no parsing with any other protocol - transparent here - just pass through
                write_to_serial(bleData.data, bleData.data_len);
            }
            free(bleData.data);
        } else {
            if (db_uart_write_queue_ble == NULL)
                ESP_LOGE(TAG, "db_uart_write_queue is NULL!");
            // no new data available to be sent via serial link do nothing
        }

        /**Yield to the scheduler if delay_timer_cnt reaches 5000,allowing other
         * tasks to execute and preventing starvation.
         */
        if (delay_timer_cnt == 5000) {
            vTaskDelay(10 / portTICK_PERIOD_MS);
            delay_timer_cnt = 0;
        } else {
            delay_timer_cnt++;
        }
    }
    vTaskDelete(NULL);
}

#endif

/**
 * @brief DroneBridge control module implementation for a ESP32 device. Bidirectional link between FC and ground. Can
 * handle MSPv1, MSPv2, LTM and MAVLink.
 * MSP & LTM is parsed and sent packet/frame by frame to ground
 * MAVLink is passed through (fully transparent). Can be used with any protocol.
 */
void db_start_control_module() {
    switch (DB_PARAM_RADIO_MODE) {
        case DB_BLUETOOTH_MODE:
#ifdef CONFIG_BT_ENABLED
            xTaskCreate(&control_module_ble,   /**< Task function for Bluetooth BLE communication */
                        "control_bluetooth",      /**< Task name (for debugging) */
                        40960,                /**< Stack size (in bytes) */
                        NULL,                 /**< Task parameters (unused) */
                        5,                       /**< Task priority */
                        NULL                 /**< Task handle (unused) */
            );
#else
            ESP_LOGE(TAG, "Bluetooth is not enabled. Aborting start of control module.");
#endif
            break;

        default:
            xTaskCreate(&control_module_udp_tcp,  /**< Task function for UDP/TCP communication */
                        "control_wifi",              /**< Task name (for debugging) */
                        46080,                   /**< Stack size (in bytes) */
                        NULL,                    /**< Task parameters (unused) */
                        5,                          /**< Task priority */
                        NULL                    /**< Task handle (unused) */
            );
            break;
    }
}
