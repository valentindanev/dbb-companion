/*
 *   This file is part of DroneBridge: https://github.com/DroneBridge/ESP32
 *
 *   Copyright 2024 Wolfgang Christl
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

#ifndef DB_ESP32_DB_MAVLINK_MSGS_H
#define DB_ESP32_DB_MAVLINK_MSGS_H

#include <stdint.h>
#include <stdbool.h>
#include "db_serial.h"
#include "common/common.h"

#define DB_VENDOR_ID 0xDB32  //
#define DB_PRODUCT_ID 1 // 1=DroneBridge for ESP32
#define DB_MAVLINK_FC_HEARTBEAT_STALE_MS 3000
#define DB_MAVLINK_TELEMETRY_MSG_TYPES_MAX 48

typedef struct {
    bool seen;
    bool stale;
    bool armed;
    uint8_t sysid;
    uint8_t compid;
    uint8_t type;
    uint8_t autopilot;
    uint8_t base_mode;
    uint32_t custom_mode;
    uint8_t system_status;
    uint32_t heartbeat_age_ms;
    const char *mode_name;
} db_mavlink_fc_state_t;

/* Generic FC telemetry cache. It is populated only from the local FC UART. */
typedef struct {
    bool valid;
    uint16_t chan[18];
    uint8_t chancount;
    uint8_t rssi;
    uint32_t updates;
    int64_t age_ms; /* -1 when no RC frame has been received */
} db_mavlink_rc_state_t;

typedef struct {
    bool valid;
    uint16_t vcc_mv;
    uint16_t vservo_mv;
    uint16_t flags;
    int64_t age_ms;
} db_mavlink_power_state_t;

typedef struct {
    bool valid;
    uint16_t voltage_mv;
    int16_t current_ca;
    int8_t remaining_pct;
    int32_t consumed_mah;
    int16_t temperature_cdeg;
    int64_t age_ms;
} db_mavlink_battery_state_t;

typedef struct {
    bool valid;
    uint8_t fix_type;
    uint8_t satellites_visible;
    uint16_t eph;
    /* GPS_RAW_INT coordinates in degrees × 1E7, kept losslessly for logs. */
    int32_t latitude_e7;
    int32_t longitude_e7;
    int64_t age_ms;
} db_mavlink_gps_state_t;

typedef struct { bool valid; uint16_t load; uint16_t voltage_mv; int16_t current_ca; int8_t remaining_pct; uint16_t drop_rate_comm; uint16_t errors_comm; uint32_t sensors_present; uint32_t sensors_enabled; uint32_t sensors_health; int64_t age_ms; } db_mavlink_system_state_t;
typedef struct { bool valid; uint64_t unix_usec; uint32_t boot_ms; int64_t age_ms; } db_mavlink_time_state_t;
typedef struct { bool valid; float roll_rad; float pitch_rad; float yaw_rad; float rollspeed; float pitchspeed; float yawspeed; int64_t age_ms; } db_mavlink_attitude_state_t;
typedef struct { bool valid; int32_t latitude_e7; int32_t longitude_e7; int32_t altitude_mm; int32_t relative_altitude_mm; int16_t vx_cms; int16_t vy_cms; int16_t vz_cms; uint16_t heading_cdeg; int64_t age_ms; } db_mavlink_position_state_t;
typedef struct { bool valid; float airspeed_mps; float groundspeed_mps; int16_t heading_deg; uint16_t throttle_pct; float altitude_m; float climb_mps; int64_t age_ms; } db_mavlink_vfr_state_t;
typedef struct { bool valid; float press_abs_hpa; float press_diff_hpa; int16_t temperature_cdeg; int64_t age_ms; } db_mavlink_pressure_state_t;
typedef struct { bool valid; int16_t xacc; int16_t yacc; int16_t zacc; int16_t xgyro; int16_t ygyro; int16_t zgyro; int16_t xmag; int16_t ymag; int16_t zmag; int64_t age_ms; } db_mavlink_imu_state_t;
typedef struct { bool valid; uint16_t seq; int64_t age_ms; } db_mavlink_mission_state_t;
typedef struct { bool valid; uint16_t raw[16]; uint8_t port; int64_t age_ms; } db_mavlink_servo_state_t;
typedef struct { bool valid; float vibration_x; float vibration_y; float vibration_z; uint32_t clipping_0; uint32_t clipping_1; uint32_t clipping_2; int64_t age_ms; } db_mavlink_vibration_state_t;
typedef struct { bool valid; int64_t tc1; int64_t ts1; int64_t age_ms; } db_mavlink_timesync_state_t;
typedef struct { bool valid; uint8_t severity; char text[51]; int64_t age_ms; } db_mavlink_statustext_state_t;
typedef struct { uint32_t id; uint32_t count; int64_t age_ms; } db_mavlink_message_stat_t;

/* DISTANCE_SENSOR messages arriving back from the FC on the ESP UART. */
typedef struct {
    uint32_t count;
    uint32_t last_interval_ms;
    uint32_t max_interval_ms;
    int32_t distance_mm;
    uint8_t sensor_id;
    uint8_t sysid;
    uint8_t compid;
    int64_t age_ms; /* -1 until the first returned message */
} db_mavlink_distance_sensor_state_t;

typedef struct {
    db_mavlink_fc_state_t fc;
    db_mavlink_rc_state_t rc;
    db_mavlink_power_state_t power;
    db_mavlink_battery_state_t battery;
    db_mavlink_gps_state_t gps;
    db_mavlink_system_state_t system;
    db_mavlink_time_state_t time;
    db_mavlink_attitude_state_t attitude;
    db_mavlink_position_state_t position;
    db_mavlink_vfr_state_t vfr;
    db_mavlink_pressure_state_t pressure;
    db_mavlink_imu_state_t raw_imu;
    db_mavlink_imu_state_t scaled_imu2;
    db_mavlink_mission_state_t mission;
    db_mavlink_servo_state_t servo;
    db_mavlink_vibration_state_t vibration;
    db_mavlink_timesync_state_t timesync;
    db_mavlink_statustext_state_t statustext;
    db_mavlink_distance_sensor_state_t returned_distance_sensor;
    uint8_t message_stat_count;
    db_mavlink_message_stat_t message_stats[DB_MAVLINK_TELEMETRY_MSG_TYPES_MAX];
} db_mavlink_telemetry_t;

uint8_t db_get_mav_comp_id();
uint8_t db_get_mav_sys_id();
void db_mavlink_get_fc_state(db_mavlink_fc_state_t *out_state);
void db_mavlink_get_gps_state(db_mavlink_gps_state_t *out_state);
void db_mavlink_get_telemetry(db_mavlink_telemetry_t *out_telemetry);
const char *db_mavlink_mode_name(uint8_t autopilot, uint8_t type,
                                 uint32_t custom_mode);
int8_t db_format_rssi(int8_t signal_strength, int8_t noise_floor);
uint16_t db_mav_create_heartbeat(uint8_t *buff, fmav_status_t *fmav_status);
uint16_t db_get_mavmsg_param_value(uint8_t *buff, fmav_status_t *fmav_status, uint16_t param_index, float_int_union *value, uint8_t type, char *param_id);
MAV_PARAM_TYPE db_mav_get_parameter_value(float_int_union *float_int, const char *param_id, const int16_t param_index);
bool db_write_mavlink_parameter(const fmav_param_set_t *param_set_payload);
void db_process_mavlink_command(fmav_command_long_t *the_command,
                                fmav_message_t *the_msg,
                                fmav_status_t *status,
                                uint8_t *buff,
                                enum DB_MAVLINK_DATA_ORIGIN origin, int *tcp_clients, udp_conn_list_t *udp_conns);
void handle_mavlink_message(fmav_message_t *new_msg, int *tcp_clients, udp_conn_list_t *udp_conns,
                            fmav_status_t *fmav_status,
                            enum DB_MAVLINK_DATA_ORIGIN origin);

#endif //DB_ESP32_DB_MAVLINK_MSGS_H
