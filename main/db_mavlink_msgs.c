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
#include <string.h>
#include <esp_log.h>
#include <esp_timer.h>
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

#include "db_mavlink_msgs.h"
#include "db_parameters.h"
#include "db_serial.h"
#include "db_sonar_log.h"
#include "db_fc_params.h"
#include "db_fc_tune.h"
#include "globals.h"
#include "main.h"
#include "dbb_brain.h"

#ifndef MIN
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a,b) ((a) > (b) ? (a) : (b))
#endif

#define FASTMAVLINK_ROUTER_LINKS_MAX  3
#define FASTMAVLINK_ROUTER_COMPONENTS_MAX  5

#define TAG "DB_MAV_MSGS"

typedef struct {
    bool seen;
    bool armed;
    uint8_t sysid;
    uint8_t compid;
    uint8_t type;
    uint8_t autopilot;
    uint8_t base_mode;
    uint32_t custom_mode;
    uint8_t system_status;
    int64_t last_heartbeat_us;
} db_mavlink_fc_state_internal_t;

static db_mavlink_fc_state_internal_t s_fc_state;
static portMUX_TYPE s_fc_state_mux = portMUX_INITIALIZER_UNLOCKED;

typedef struct {
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
    int64_t rc_last_us;
    int64_t power_last_us;
    int64_t battery_last_us;
    int64_t gps_last_us;
    int64_t system_last_us, time_last_us, attitude_last_us, position_last_us;
    int64_t vfr_last_us, pressure_last_us, raw_imu_last_us, scaled_imu2_last_us;
    int64_t mission_last_us, servo_last_us, vibration_last_us, timesync_last_us;
    int64_t statustext_last_us;
    uint8_t message_stat_count;
    struct { uint32_t id; uint32_t count; int64_t last_us; }
        message_stats[DB_MAVLINK_TELEMETRY_MSG_TYPES_MAX];
} db_mavlink_telemetry_cache_t;

static db_mavlink_telemetry_cache_t s_telemetry_cache;
static portMUX_TYPE s_telemetry_cache_mux = portMUX_INITIALIZER_UNLOCKED;

void db_mavlink_get_gps_state(db_mavlink_gps_state_t *out_state) {
    if (out_state == NULL) return;

    db_mavlink_gps_state_t gps = {0};
    int64_t gps_last_us = 0;
    taskENTER_CRITICAL(&s_telemetry_cache_mux);
    gps = s_telemetry_cache.gps;
    gps_last_us = s_telemetry_cache.gps_last_us;
    taskEXIT_CRITICAL(&s_telemetry_cache_mux);

    if (!gps.valid || gps_last_us <= 0) {
        gps.age_ms = -1;
    } else {
        int64_t age_us = esp_timer_get_time() - gps_last_us;
        gps.age_ms = age_us > 0 ? age_us / 1000 : 0;
    }
    *out_state = gps;
}

const char *db_mavlink_mode_name(uint8_t autopilot, uint8_t type,
                                 uint32_t custom_mode) {
    if (autopilot != MAV_AUTOPILOT_ARDUPILOTMEGA) {
        return "unknown";
    }

    if (type != MAV_TYPE_GROUND_ROVER && type != MAV_TYPE_SURFACE_BOAT) {
        return "unknown";
    }

    switch (custom_mode) {
    case 0:
        return "MANUAL";
    case 1:
        return "ACRO";
    case 3:
        return "STEERING";
    case 4:
        return "HOLD";
    case 5:
        return "LOITER";
    case 6:
        return "FOLLOW";
    case 7:
        return "SIMPLE";
    case 10:
        return "AUTO";
    case 11:
        return "RTL";
    case 12:
        return "SMART_RTL";
    case 15:
        return "GUIDED";
    case 16:
        return "INITIALISING";
    default:
        return "unknown";
    }
}

static void db_mavlink_update_fc_state(const fmav_message_t *msg,
                                       const fmav_heartbeat_t *heartbeat) {
    if (msg == NULL || heartbeat == NULL) {
        return;
    }

    taskENTER_CRITICAL(&s_fc_state_mux);
    s_fc_state.seen = true;
    s_fc_state.armed =
        (heartbeat->base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
    s_fc_state.sysid = msg->sysid;
    s_fc_state.compid = msg->compid;
    s_fc_state.type = heartbeat->type;
    s_fc_state.autopilot = heartbeat->autopilot;
    s_fc_state.base_mode = heartbeat->base_mode;
    s_fc_state.custom_mode = heartbeat->custom_mode;
    s_fc_state.system_status = heartbeat->system_status;
    s_fc_state.last_heartbeat_us = esp_timer_get_time();
    taskEXIT_CRITICAL(&s_fc_state_mux);
    db_sonar_log_note_fc_armed_state(
        (heartbeat->base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0);
}

void db_mavlink_get_fc_state(db_mavlink_fc_state_t *out_state) {
    if (out_state == NULL) {
        return;
    }

    memset(out_state, 0, sizeof(*out_state));
    db_mavlink_fc_state_internal_t local_state = {0};

    taskENTER_CRITICAL(&s_fc_state_mux);
    local_state = s_fc_state;
    taskEXIT_CRITICAL(&s_fc_state_mux);

    out_state->seen = local_state.seen;
    out_state->armed = local_state.armed;
    out_state->sysid = local_state.sysid;
    out_state->compid = local_state.compid;
    out_state->type = local_state.type;
    out_state->autopilot = local_state.autopilot;
    out_state->base_mode = local_state.base_mode;
    out_state->custom_mode = local_state.custom_mode;
    out_state->system_status = local_state.system_status;
    out_state->mode_name = db_mavlink_mode_name(
        local_state.autopilot, local_state.type, local_state.custom_mode);

    if (!local_state.seen || local_state.last_heartbeat_us <= 0) {
        out_state->stale = true;
        out_state->heartbeat_age_ms = UINT32_MAX;
        return;
    }

    int64_t age_us = esp_timer_get_time() - local_state.last_heartbeat_us;
    if (age_us < 0) {
        age_us = 0;
    }
    out_state->heartbeat_age_ms = (uint32_t)(age_us / 1000);
    out_state->stale =
        out_state->heartbeat_age_ms > DB_MAVLINK_FC_HEARTBEAT_STALE_MS;
}

/* Called only for messages received from the physical FC UART. */
static void db_mavlink_update_telemetry_cache(const fmav_message_t *msg) {
    if (msg == NULL) return;

    const int64_t now = esp_timer_get_time();
    taskENTER_CRITICAL(&s_telemetry_cache_mux);
    int message_index = -1;
    for (int i = 0; i < s_telemetry_cache.message_stat_count; i++) {
        if (s_telemetry_cache.message_stats[i].id == msg->msgid) {
            message_index = i;
            break;
        }
    }
    if (message_index < 0 && s_telemetry_cache.message_stat_count <
                                 DB_MAVLINK_TELEMETRY_MSG_TYPES_MAX) {
        message_index = s_telemetry_cache.message_stat_count++;
        s_telemetry_cache.message_stats[message_index].id = msg->msgid;
        s_telemetry_cache.message_stats[message_index].count = 0;
    }
    if (message_index >= 0) {
        s_telemetry_cache.message_stats[message_index].count++;
        s_telemetry_cache.message_stats[message_index].last_us = now;
    }
    taskEXIT_CRITICAL(&s_telemetry_cache_mux);
    switch (msg->msgid) {
    case FASTMAVLINK_MSG_ID_SYS_STATUS: {
        fmav_sys_status_t status; fmav_msg_sys_status_decode(&status, msg);
        taskENTER_CRITICAL(&s_telemetry_cache_mux);
        s_telemetry_cache.system = (db_mavlink_system_state_t){.valid=true, .load=status.load, .voltage_mv=status.voltage_battery, .current_ca=status.current_battery, .remaining_pct=status.battery_remaining, .drop_rate_comm=status.drop_rate_comm, .errors_comm=status.errors_comm, .sensors_present=status.onboard_control_sensors_present, .sensors_enabled=status.onboard_control_sensors_enabled, .sensors_health=status.onboard_control_sensors_health};
        s_telemetry_cache.system_last_us = now;
        taskEXIT_CRITICAL(&s_telemetry_cache_mux); break;
    }
    case FASTMAVLINK_MSG_ID_SYSTEM_TIME: {
        fmav_system_time_t time; fmav_msg_system_time_decode(&time, msg);
        taskENTER_CRITICAL(&s_telemetry_cache_mux);
        s_telemetry_cache.time = (db_mavlink_time_state_t){.valid=true, .unix_usec=time.time_unix_usec, .boot_ms=time.time_boot_ms}; s_telemetry_cache.time_last_us=now;
        taskEXIT_CRITICAL(&s_telemetry_cache_mux); break;
    }
    case FASTMAVLINK_MSG_ID_ATTITUDE: {
        fmav_attitude_t attitude; fmav_msg_attitude_decode(&attitude, msg);
        taskENTER_CRITICAL(&s_telemetry_cache_mux);
        s_telemetry_cache.attitude=(db_mavlink_attitude_state_t){.valid=true,.roll_rad=attitude.roll,.pitch_rad=attitude.pitch,.yaw_rad=attitude.yaw,.rollspeed=attitude.rollspeed,.pitchspeed=attitude.pitchspeed,.yawspeed=attitude.yawspeed}; s_telemetry_cache.attitude_last_us=now;
        taskEXIT_CRITICAL(&s_telemetry_cache_mux); break;
    }
    case FASTMAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
        fmav_global_position_int_t position; fmav_msg_global_position_int_decode(&position, msg);
        taskENTER_CRITICAL(&s_telemetry_cache_mux);
        s_telemetry_cache.position=(db_mavlink_position_state_t){.valid=true,.latitude_e7=position.lat,.longitude_e7=position.lon,.altitude_mm=position.alt,.relative_altitude_mm=position.relative_alt,.vx_cms=position.vx,.vy_cms=position.vy,.vz_cms=position.vz,.heading_cdeg=position.hdg}; s_telemetry_cache.position_last_us=now;
        taskEXIT_CRITICAL(&s_telemetry_cache_mux); break;
    }
    case FASTMAVLINK_MSG_ID_VFR_HUD: {
        fmav_vfr_hud_t vfr; fmav_msg_vfr_hud_decode(&vfr, msg);
        taskENTER_CRITICAL(&s_telemetry_cache_mux);
        s_telemetry_cache.vfr=(db_mavlink_vfr_state_t){.valid=true,.airspeed_mps=vfr.airspeed,.groundspeed_mps=vfr.groundspeed,.heading_deg=vfr.heading,.throttle_pct=vfr.throttle,.altitude_m=vfr.alt,.climb_mps=vfr.climb}; s_telemetry_cache.vfr_last_us=now;
        taskEXIT_CRITICAL(&s_telemetry_cache_mux); break;
    }
    case FASTMAVLINK_MSG_ID_SCALED_PRESSURE: {
        fmav_scaled_pressure_t pressure; fmav_msg_scaled_pressure_decode(&pressure, msg);
        taskENTER_CRITICAL(&s_telemetry_cache_mux);
        s_telemetry_cache.pressure=(db_mavlink_pressure_state_t){.valid=true,.press_abs_hpa=pressure.press_abs,.press_diff_hpa=pressure.press_diff,.temperature_cdeg=pressure.temperature}; s_telemetry_cache.pressure_last_us=now;
        taskEXIT_CRITICAL(&s_telemetry_cache_mux); break;
    }
    case FASTMAVLINK_MSG_ID_RAW_IMU: {
        fmav_raw_imu_t imu; fmav_msg_raw_imu_decode(&imu, msg);
        taskENTER_CRITICAL(&s_telemetry_cache_mux);
        s_telemetry_cache.raw_imu=(db_mavlink_imu_state_t){.valid=true,.xacc=imu.xacc,.yacc=imu.yacc,.zacc=imu.zacc,.xgyro=imu.xgyro,.ygyro=imu.ygyro,.zgyro=imu.zgyro,.xmag=imu.xmag,.ymag=imu.ymag,.zmag=imu.zmag}; s_telemetry_cache.raw_imu_last_us=now;
        taskEXIT_CRITICAL(&s_telemetry_cache_mux); break;
    }
    case FASTMAVLINK_MSG_ID_SCALED_IMU2: {
        fmav_scaled_imu2_t imu; fmav_msg_scaled_imu2_decode(&imu, msg);
        taskENTER_CRITICAL(&s_telemetry_cache_mux);
        s_telemetry_cache.scaled_imu2=(db_mavlink_imu_state_t){.valid=true,.xacc=imu.xacc,.yacc=imu.yacc,.zacc=imu.zacc,.xgyro=imu.xgyro,.ygyro=imu.ygyro,.zgyro=imu.zgyro,.xmag=imu.xmag,.ymag=imu.ymag,.zmag=imu.zmag}; s_telemetry_cache.scaled_imu2_last_us=now;
        taskEXIT_CRITICAL(&s_telemetry_cache_mux); break;
    }
    case FASTMAVLINK_MSG_ID_MISSION_CURRENT: {
        fmav_mission_current_t mission; fmav_msg_mission_current_decode(&mission, msg);
        taskENTER_CRITICAL(&s_telemetry_cache_mux); s_telemetry_cache.mission=(db_mavlink_mission_state_t){.valid=true,.seq=mission.seq}; s_telemetry_cache.mission_last_us=now; taskEXIT_CRITICAL(&s_telemetry_cache_mux); break;
    }
    case FASTMAVLINK_MSG_ID_SERVO_OUTPUT_RAW: {
        fmav_servo_output_raw_t servo; fmav_msg_servo_output_raw_decode(&servo, msg);
        uint16_t raw[16]={servo.servo1_raw,servo.servo2_raw,servo.servo3_raw,servo.servo4_raw,servo.servo5_raw,servo.servo6_raw,servo.servo7_raw,servo.servo8_raw,servo.servo9_raw,servo.servo10_raw,servo.servo11_raw,servo.servo12_raw,servo.servo13_raw,servo.servo14_raw,servo.servo15_raw,servo.servo16_raw};
        taskENTER_CRITICAL(&s_telemetry_cache_mux); s_telemetry_cache.servo.valid=true; s_telemetry_cache.servo.port=servo.port; memcpy(s_telemetry_cache.servo.raw,raw,sizeof(raw)); s_telemetry_cache.servo_last_us=now; taskEXIT_CRITICAL(&s_telemetry_cache_mux); break;
    }
    case FASTMAVLINK_MSG_ID_VIBRATION: {
        fmav_vibration_t vibration; fmav_msg_vibration_decode(&vibration, msg);
        taskENTER_CRITICAL(&s_telemetry_cache_mux); s_telemetry_cache.vibration=(db_mavlink_vibration_state_t){.valid=true,.vibration_x=vibration.vibration_x,.vibration_y=vibration.vibration_y,.vibration_z=vibration.vibration_z,.clipping_0=vibration.clipping_0,.clipping_1=vibration.clipping_1,.clipping_2=vibration.clipping_2}; s_telemetry_cache.vibration_last_us=now; taskEXIT_CRITICAL(&s_telemetry_cache_mux); break;
    }
    case FASTMAVLINK_MSG_ID_TIMESYNC: {
        fmav_timesync_t timesync; fmav_msg_timesync_decode(&timesync, msg);
        taskENTER_CRITICAL(&s_telemetry_cache_mux); s_telemetry_cache.timesync=(db_mavlink_timesync_state_t){.valid=true,.tc1=timesync.tc1,.ts1=timesync.ts1}; s_telemetry_cache.timesync_last_us=now; taskEXIT_CRITICAL(&s_telemetry_cache_mux); break;
    }
    case FASTMAVLINK_MSG_ID_STATUSTEXT: {
        fmav_statustext_t text; fmav_msg_statustext_decode(&text, msg);
        taskENTER_CRITICAL(&s_telemetry_cache_mux); s_telemetry_cache.statustext.valid=true; s_telemetry_cache.statustext.severity=text.severity; memcpy(s_telemetry_cache.statustext.text,text.text,50); s_telemetry_cache.statustext.text[50]='\0'; s_telemetry_cache.statustext_last_us=now; taskEXIT_CRITICAL(&s_telemetry_cache_mux); break;
    }
    case FASTMAVLINK_MSG_ID_RC_CHANNELS: {
        fmav_rc_channels_t rc;
        fmav_msg_rc_channels_decode(&rc, msg);
        uint16_t channels[18] = {rc.chan1_raw, rc.chan2_raw, rc.chan3_raw, rc.chan4_raw,
                                 rc.chan5_raw, rc.chan6_raw, rc.chan7_raw, rc.chan8_raw,
                                 rc.chan9_raw, rc.chan10_raw, rc.chan11_raw, rc.chan12_raw,
                                 rc.chan13_raw, rc.chan14_raw, rc.chan15_raw, rc.chan16_raw,
                                 rc.chan17_raw, rc.chan18_raw};
        taskENTER_CRITICAL(&s_telemetry_cache_mux);
        memcpy(s_telemetry_cache.rc.chan, channels, sizeof(channels));
        s_telemetry_cache.rc.chancount = rc.chancount > 18 ? 18 : rc.chancount;
        s_telemetry_cache.rc.rssi = rc.rssi;
        s_telemetry_cache.rc.updates++;
        s_telemetry_cache.rc.valid = true;
        s_telemetry_cache.rc_last_us = now;
        taskEXIT_CRITICAL(&s_telemetry_cache_mux);
        break;
    }
    case FASTMAVLINK_MSG_ID_RC_CHANNELS_RAW: {
        fmav_rc_channels_raw_t rc;
        fmav_msg_rc_channels_raw_decode(&rc, msg);
        taskENTER_CRITICAL(&s_telemetry_cache_mux);
        s_telemetry_cache.rc.chan[0]=rc.chan1_raw; s_telemetry_cache.rc.chan[1]=rc.chan2_raw;
        s_telemetry_cache.rc.chan[2]=rc.chan3_raw; s_telemetry_cache.rc.chan[3]=rc.chan4_raw;
        s_telemetry_cache.rc.chan[4]=rc.chan5_raw; s_telemetry_cache.rc.chan[5]=rc.chan6_raw;
        s_telemetry_cache.rc.chan[6]=rc.chan7_raw; s_telemetry_cache.rc.chan[7]=rc.chan8_raw;
        for (int i = 8; i < 18; i++) s_telemetry_cache.rc.chan[i] = 0xFFFF;
        s_telemetry_cache.rc.chancount = 8;
        s_telemetry_cache.rc.rssi = rc.rssi;
        s_telemetry_cache.rc.updates++;
        s_telemetry_cache.rc.valid = true;
        s_telemetry_cache.rc_last_us = now;
        taskEXIT_CRITICAL(&s_telemetry_cache_mux);
        break;
    }
    case FASTMAVLINK_MSG_ID_POWER_STATUS: {
        fmav_power_status_t power;
        fmav_msg_power_status_decode(&power, msg);
        taskENTER_CRITICAL(&s_telemetry_cache_mux);
        s_telemetry_cache.power.vcc_mv = power.Vcc;
        s_telemetry_cache.power.vservo_mv = power.Vservo;
        s_telemetry_cache.power.flags = power.flags;
        s_telemetry_cache.power.valid = true;
        s_telemetry_cache.power_last_us = now;
        taskEXIT_CRITICAL(&s_telemetry_cache_mux);
        break;
    }
    case FASTMAVLINK_MSG_ID_BATTERY_STATUS: {
        fmav_battery_status_t battery;
        fmav_msg_battery_status_decode(&battery, msg);
        taskENTER_CRITICAL(&s_telemetry_cache_mux);
        s_telemetry_cache.battery.voltage_mv = battery.voltages[0];
        s_telemetry_cache.battery.current_ca = battery.current_battery;
        s_telemetry_cache.battery.remaining_pct = battery.battery_remaining;
        s_telemetry_cache.battery.consumed_mah = battery.current_consumed;
        s_telemetry_cache.battery.temperature_cdeg = battery.temperature;
        s_telemetry_cache.battery.valid = true;
        s_telemetry_cache.battery_last_us = now;
        taskEXIT_CRITICAL(&s_telemetry_cache_mux);
        break;
    }
    case FASTMAVLINK_MSG_ID_GPS_RAW_INT: {
        fmav_gps_raw_int_t gps;
        fmav_msg_gps_raw_int_decode(&gps, msg);
        taskENTER_CRITICAL(&s_telemetry_cache_mux);
        s_telemetry_cache.gps.fix_type = gps.fix_type;
        s_telemetry_cache.gps.satellites_visible = gps.satellites_visible;
        s_telemetry_cache.gps.eph = gps.eph;
        s_telemetry_cache.gps.latitude_e7 = gps.lat;
        s_telemetry_cache.gps.longitude_e7 = gps.lon;
        s_telemetry_cache.gps.valid = true;
        s_telemetry_cache.gps_last_us = now;
        taskEXIT_CRITICAL(&s_telemetry_cache_mux);
        break;
    }
    case FASTMAVLINK_MSG_ID_DISTANCE_SENSOR: {
        /* Keep hardwired capture accounting; the Deeper UI tracer is removed. */
        db_sonar_log_note_fc_returned_distance_sensor();
        break;
    }
    default:
        break;
    }
}

void db_mavlink_get_telemetry(db_mavlink_telemetry_t *out_telemetry) {
    if (out_telemetry == NULL) return;

    memset(out_telemetry, 0, sizeof(*out_telemetry));
    db_mavlink_get_fc_state(&out_telemetry->fc);
    db_mavlink_telemetry_cache_t local_cache = {0};
    taskENTER_CRITICAL(&s_telemetry_cache_mux);
    local_cache = s_telemetry_cache;
    taskEXIT_CRITICAL(&s_telemetry_cache_mux);

    const int64_t now = esp_timer_get_time();
    out_telemetry->rc = local_cache.rc;
    out_telemetry->power = local_cache.power;
    out_telemetry->battery = local_cache.battery;
    out_telemetry->gps = local_cache.gps;
    out_telemetry->system = local_cache.system;
    out_telemetry->time = local_cache.time;
    out_telemetry->attitude = local_cache.attitude;
    out_telemetry->position = local_cache.position;
    out_telemetry->vfr = local_cache.vfr;
    out_telemetry->pressure = local_cache.pressure;
    out_telemetry->raw_imu = local_cache.raw_imu;
    out_telemetry->scaled_imu2 = local_cache.scaled_imu2;
    out_telemetry->mission = local_cache.mission;
    out_telemetry->servo = local_cache.servo;
    out_telemetry->vibration = local_cache.vibration;
    out_telemetry->timesync = local_cache.timesync;
    out_telemetry->statustext = local_cache.statustext;
    out_telemetry->rc.age_ms = local_cache.rc.valid ? (now - local_cache.rc_last_us) / 1000 : -1;
    out_telemetry->power.age_ms = local_cache.power.valid ? (now - local_cache.power_last_us) / 1000 : -1;
    out_telemetry->battery.age_ms = local_cache.battery.valid ? (now - local_cache.battery_last_us) / 1000 : -1;
    out_telemetry->gps.age_ms = local_cache.gps.valid ? (now - local_cache.gps_last_us) / 1000 : -1;
    out_telemetry->system.age_ms = local_cache.system.valid ? (now - local_cache.system_last_us) / 1000 : -1;
    out_telemetry->time.age_ms = local_cache.time.valid ? (now - local_cache.time_last_us) / 1000 : -1;
    out_telemetry->attitude.age_ms = local_cache.attitude.valid ? (now - local_cache.attitude_last_us) / 1000 : -1;
    out_telemetry->position.age_ms = local_cache.position.valid ? (now - local_cache.position_last_us) / 1000 : -1;
    out_telemetry->vfr.age_ms = local_cache.vfr.valid ? (now - local_cache.vfr_last_us) / 1000 : -1;
    out_telemetry->pressure.age_ms = local_cache.pressure.valid ? (now - local_cache.pressure_last_us) / 1000 : -1;
    out_telemetry->raw_imu.age_ms = local_cache.raw_imu.valid ? (now - local_cache.raw_imu_last_us) / 1000 : -1;
    out_telemetry->scaled_imu2.age_ms = local_cache.scaled_imu2.valid ? (now - local_cache.scaled_imu2_last_us) / 1000 : -1;
    out_telemetry->mission.age_ms = local_cache.mission.valid ? (now - local_cache.mission_last_us) / 1000 : -1;
    out_telemetry->servo.age_ms = local_cache.servo.valid ? (now - local_cache.servo_last_us) / 1000 : -1;
    out_telemetry->vibration.age_ms = local_cache.vibration.valid ? (now - local_cache.vibration_last_us) / 1000 : -1;
    out_telemetry->timesync.age_ms = local_cache.timesync.valid ? (now - local_cache.timesync_last_us) / 1000 : -1;
    out_telemetry->statustext.age_ms = local_cache.statustext.valid ? (now - local_cache.statustext_last_us) / 1000 : -1;
    out_telemetry->message_stat_count = local_cache.message_stat_count;
    for (int i = 0; i < local_cache.message_stat_count; i++) {
        out_telemetry->message_stats[i].id = local_cache.message_stats[i].id;
        out_telemetry->message_stats[i].count = local_cache.message_stats[i].count;
        out_telemetry->message_stats[i].age_ms =
            (now - local_cache.message_stats[i].last_us) / 1000;
    }
}

/**
 * Based on the system architecture and configured wifi mode the ESP32 may have a different role and system id.
 * Returns the best fitting component ID for the specific role.
 * @return component ID for ESP32
 */
uint8_t db_get_mav_comp_id() {
    return MAV_COMP_ID_TELEMETRY_RADIO;
}

/**
 * Return the Mavlink system ID. Set by handle_mavlink_message()
 * @return system ID for ESP32
 */
uint8_t db_get_mav_sys_id() {
    return DB_MAV_SYS_ID;
}

/**
 * Converts the measured (negative dBm) signal strength to a format the MAVLink RADIO STATUS packet accepts and the GCS likes.
 * If QGroundControl is desired output format it will not convert but send the value as int8. For Mission Planner it converts int8 to uint8. The value represents the absolute(dBm): -54 dBm -> 54
 * @param signal_strength Signal strength in dBm as reported by the ESP32
 * @param noise_floor   Spectrum noise floor - not used for now
 * @return Signal strength formatted for QGroundControl (0 to -127 [dBm]) or Mission Planner (0 to 100)
 */
int8_t db_format_rssi(int8_t signal_strength, int8_t noise_floor) {
    if (db_param_rssi_dbm.value.db_param_u8.value) {
        // report in [dBm] - no conversion since in Wi-Fi, BLE & ESP-NOW APIs natively report dBm
        return signal_strength;
    } else {
        // dBm from [-50 to -100] scaled to 100 to 0
        return MIN(100, 2 * (MAX(-100, MIN(-50, signal_strength)) + 100));
    }
}

/**
 * Creates and writes Mavlink heartbeat message to supplied buffer
 *
 * @param buff Buffer to write heartbeat to (>280 bytes)
 * @param fmav_status Status struct of the fmav parser. Will be used to set and update the squence number of the packet.
 * @return Length of the message in the buffer
 */
uint16_t db_mav_create_heartbeat(uint8_t *buff, fmav_status_t *fmav_status) {
    return fmav_msg_heartbeat_pack_to_frame_buf(
            buff, db_get_mav_sys_id(), db_get_mav_comp_id(),
            MAV_TYPE_ONBOARD_CONTROLLER, MAV_AUTOPILOT_INVALID, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 0, MAV_STATE_ACTIVE,
            fmav_status);
}

/**
 * Acknowledge mavlink command by sending ACK response
 *
 * @param the_msg The mavlink message that was parsed (containing the_command)
 * @param result
 * @param status fastmavlink parser status structure
 * @param buff Supply output buffer for sending data
 * @param origin Origin of the command as defined by DB_MAVLINK_DATA_ORIGIN
 * @param tcp_clients List of connected tcp clients as sockets
 * @param udp_conns List of connected UDP clients
 */
void db_ack_mavlink_command(uint16_t msg_id, MAV_RESULT result, const fmav_message_t *the_msg, fmav_status_t *status,
                            uint8_t *buff, enum DB_MAVLINK_DATA_ORIGIN *origin, int *tcp_clients,
                            udp_conn_list_t *udp_conns) {
    fmav_command_ack_t a = {.command = msg_id,
            .result = result,
            .target_system = the_msg->sysid,
            .target_component = the_msg->compid};
    uint16_t len = fmav_msg_command_ack_encode_to_frame_buf(buff, db_get_mav_sys_id(), db_get_mav_comp_id(), &a,
                                                            status);
    db_route_mavlink_response(buff, len, (*origin), tcp_clients, udp_conns);
}

/**
 * Creates a mavlink PARAM_VALUE message inside the provided buffer using the provided parameter
 *
 * @param buff Buffer to write the mavlink message to
 * @param fmav_status fastmavlink status stucture
 * @param param_index Index of the parameter to send
 * @param value The value of the parameter to be sent -> will be converted to IEEE 745
 * @param type The MAV_PARAM_TYPE
 * @param param_id The name of the parameter (ID)
 * @return Length of the mavlink message inside the buffer
 */
uint16_t db_get_mavmsg_param_value(uint8_t *buff, fmav_status_t *fmav_status, uint16_t param_index, float_int_union *value, uint8_t type, char *param_id) {
    fmav_param_value_t fmav_param_value = {
            .param_value = value->f,
            .param_type = type,
            .param_count = DB_PARAM_MAV_CNT,
            .param_index = param_index};
    if (strlen(param_id)>15) {  // max size of param_id is 16 bytes
        memcpy(fmav_param_value.param_id, param_id, 16);
    } else {
        strcpy(fmav_param_value.param_id, param_id);
    }
    return fmav_msg_param_value_encode_to_frame_buf(buff, db_get_mav_sys_id(), db_get_mav_comp_id(),
                                                    &fmav_param_value, fmav_status);
}

/**
 * Gets the mavlink parameter value as float_int_union based on the parameter ID from the internal variable.
 * Maps MAVLink parameter names to the internal variable names.
 *
 * @param float_int IEEE 754 storage for the retrieved value
 * @param param_id  Parameter name you want the value of
 * @param param_index Index of the parameter. May be -1 if requested parameter shall be found based on param_id
 * @return MAV_PARAM_TYPE of the parameter. Returns 0 if the parameter was not found
 */
MAV_PARAM_TYPE db_mav_get_parameter_value(float_int_union *float_int, const char *param_id, const int16_t param_index) {
    MAV_PARAM_TYPE type = 0;
    if (param_index >= DB_PARAM_MAV_CNT) {
        ESP_LOGE(TAG, "Requested mavlink parameter index %i is out of range (0-%i)", param_index, DB_PARAM_MAV_CNT-1);
        return 0;
    }
    for (int i = 0; i < sizeof(db_params) / sizeof(db_params[0]); i++) {
        if (strncmp(param_id, (char *) db_params[i]->mav_t.param_name, 16) == 0 || param_index == db_params[i]->mav_t.param_index) {
            // found the parameter to return its value
            type = db_params[i]->mav_t.param_type;
            switch (db_params[i]->type) {
                case STRING:
                    ESP_LOGE(TAG, "db_mav_get_parameter_value(): String parameter not supported.");
                break;
                case UINT8:
                    float_int->uint8 = db_params[i]->value.db_param_u8.value;
                break;
                case UINT16:
                    float_int->uint16 = db_params[i]->value.db_param_u16.value;
                break;
                case INT32:
                    float_int->int32 = db_params[i]->value.db_param_i32.value;
                break;
                default:
                    ESP_LOGE(TAG, "db_mav_get_parameter_value() -> db_parameter.type unknown!");
                break;
            }
        } else {
            // do nothing - no match
        }
    }
    return type;
}

/**
 * Writes the parameter received via mavlink PARAM_SET to the internal variable and triggers write to NVS.
 * For some parameters to become effective the ESP32 still needs to be rebooted!
 * No string/blob parameters supported for now.
 *
 * @param param_set_payload
 * @return 1 in case of success and 0 in case of failure
 */
bool db_write_mavlink_parameter(const fmav_param_set_t *param_set_payload) {
    // BEWARE: ONLY WORKS WITH NUMBERS FOR NOW! - NO SUPPORT FOR STRINGS
    float_int_union float_int;  // used to convert from IEEE 754
    float_int.f = param_set_payload->param_value;   // read parameter value into helper structure
    bool success = false;
    for (int i = 0; i < sizeof(db_params) / sizeof(db_params[0]); i++) {
        if (strncmp(param_set_payload->param_id, (char *) db_params[i]->mav_t.param_name, 16) == 0) {
            switch (db_params[i]->type) {
                case STRING:
                    ESP_LOGE(TAG, "db_write_mavlink_parameter(): String not supported");
                    success = false;
                    break;
                case UINT8:
                    success = db_param_is_valid_assign_u8(float_int.uint8, db_params[i]);
                    break;
                case UINT16:
                    success = db_param_is_valid_assign_u16(float_int.uint16, db_params[i]);
                    break;
                case INT32:
                    success = db_param_is_valid_assign_i32(float_int.int32, db_params[i]);
                    break;
                default:
                    success = false;
                    ESP_LOGE(TAG, "db_write_mavlink_parameter(): Unknown type");
                    break;
            }
        } else {
            // this is not the parameter we are looking for
        }
    }
    return success;
}

/**
 * Called by db_process_mavlink_command() to process MAV_CMD_REQUEST_MESSAGE
 * @param requested_msg_id MAVLINK MSG_ID that was requested
 * @param buff Supply output buffer for sending the response
 * @param origin Origin of the command as defined by DB_MAVLINK_DATA_ORIGIN
 * @param tcp_clients List of connected tcp clients as sockets
 * @param udp_conns List of connected UDP clients
 * @param the_msg The mavlink message that was parsed (containing the_command)
 * @param status fastmavlink parser status structure
 */
void db_answer_mavlink_cmd_request_message(uint16_t requested_msg_id,
                                           uint8_t *buff, enum DB_MAVLINK_DATA_ORIGIN origin,
                                           int *tcp_clients, udp_conn_list_t *udp_conns, fmav_message_t *the_msg,
                                           fmav_status_t *status) {
    switch (requested_msg_id) {
        case FASTMAVLINK_MSG_ID_AUTOPILOT_VERSION: {
            db_ack_mavlink_command(MAV_CMD_REQUEST_MESSAGE, MAV_RESULT_ACCEPTED, the_msg, status, buff, &origin, tcp_clients, udp_conns);
            fmav_autopilot_version_t autopilot_version = {
                .board_version = (0 & 0xFFFF0000) | (1205 & 0xFFFF), // AP_HW_ESP32_PERIPH for the first 16 bits
                .capabilities = MAV_PROTOCOL_CAPABILITY_MAVLINK2 | MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_BYTEWISE,
                .flight_sw_version = ((uint32_t)DB_MAJOR_VERSION << 24) |
                          ((uint32_t)DB_MINOR_VERSION << 16) |
                          ((uint32_t)DB_PATCH_VERSION << 8)  |
                          ((uint32_t)DB_TYPE_VERSION),
                .middleware_sw_version = DB_BUILD_VERSION,
                .os_sw_version = CONFIG_IDF_FIRMWARE_CHIP_ID,
                .vendor_id = DB_VENDOR_ID,
                .product_id = DB_PRODUCT_ID,
            };
            uint16_t len = fmav_msg_autopilot_version_encode_to_frame_buf(buff, db_get_mav_sys_id(),
                                                                 db_get_mav_comp_id(), &autopilot_version, status);
            db_route_mavlink_response(buff, len, origin, tcp_clients, udp_conns);
        }
            break;
        default: {
            ESP_LOGW(TAG, "Unsupported MavLink requested message: %i - ignoring", requested_msg_id);
        }
            break;
    }
}

/**
 * Called when a MSG_ID_COMMAND_LONG was received. Handles the command processing.
 * @param the_command The structure of the received command
 * @param the_msg The mavlink message that was parsed (containing the_command)
 * @param status fastmavlink parser status structure
 * @param buff Supply output buffer for sending data
 * @param origin Origin of the command as defined by DB_MAVLINK_DATA_ORIGIN
 * @param tcp_clients List of connected tcp clients as sockets
 * @param udp_conns List of connected UDP clients
 */
void db_process_mavlink_command(fmav_command_long_t *the_command,
                                fmav_message_t *the_msg,
                                fmav_status_t *status,
                                uint8_t *buff,
                                enum DB_MAVLINK_DATA_ORIGIN origin, int *tcp_clients, udp_conn_list_t *udp_conns) {
    switch (the_command->command) {
        case MAV_CMD_REQUEST_MESSAGE: {
            uint16_t req_msg_id = the_command->param1;
            ESP_LOGI(TAG, "\trequest for msg with ID: %i", req_msg_id);
            db_answer_mavlink_cmd_request_message(req_msg_id, buff, origin, tcp_clients, udp_conns,
                                                  the_msg, status);
        }
            break;
        default: {
            fmav_command_ack_t b = {.command = the_command->command,
                    .result = MAV_RESULT_UNSUPPORTED,
                    .target_system = the_msg->sysid,
                    .target_component = the_msg->compid};
            uint16_t len = fmav_msg_command_ack_encode_to_frame_buf(buff, db_get_mav_sys_id(),
                                                                    db_get_mav_comp_id(), &b, status);
            ESP_LOGW(TAG, "Unsupported MavLink command request: %i - ignoring", the_command->command);
            db_route_mavlink_response(buff, len, origin, tcp_clients, udp_conns);
        }
            break;
    }
}

/**
 * Expects GCS to have system ID 255.
 * Processes Mavlink messages and sends the radio status message and heartbeat to the GCS on every heartbeat received via UART
 *
 * We expect the FC to be connected to serial port when in WiFi-AP or in WiFi-Client Mode.
 * We expect the GCS to be connected to serial port when in AP-LR or ESP-NOW GND mode.
 *
 * @param new_msg Message to process
 * @param tcp_clients List of connected tcp clients
 * @param udp_conns List of connected UDP clients
 * @param fmav_status fastmavlink library parser status - setup once by the parser for a specific link/interface
 * @param origin Indicates from what kind of input/link we received the new message.
 */
void handle_mavlink_message(fmav_message_t *new_msg, int *tcp_clients, udp_conn_list_t *udp_conns,
                            fmav_status_t *fmav_status,
                            enum DB_MAVLINK_DATA_ORIGIN origin) {
    static uint8_t buff[296];   // buffer to handle the response messages - no need to init every time

    // Decode generic FC telemetry once, from the physical FC UART only.
    if (origin == DB_MAVLINK_DATA_ORIGIN_SERIAL) {
        db_mavlink_update_telemetry_cache(new_msg);
    }

    // DBB Companion brain hook - sees every parsed message (weak no-op in the open base).
    dbb_brain_handle_mavlink(new_msg, origin == DB_MAVLINK_DATA_ORIGIN_SERIAL);

    switch (new_msg->msgid) {
        case FASTMAVLINK_MSG_ID_HEARTBEAT:
            if (origin == DB_MAVLINK_DATA_ORIGIN_SERIAL) {
                // we only process heartbeats coming from the UART (local device) since we also use it as a trigger to send our heartbeat
                fmav_heartbeat_t payload;
                fmav_msg_heartbeat_decode(&payload, new_msg);
                if (payload.autopilot == MAV_AUTOPILOT_INVALID && payload.type == MAV_TYPE_GCS) {
                    ESP_LOGD(TAG, "Got heartbeat from GCS (sysID: %i)", new_msg->sysid);
                    DB_MAV_SYS_ID = new_msg->sysid;
                    // The FC-side ESP32 talks to a flight controller over UART, not a GCS.
                    // A GCS heartbeat on the UART means the wiring/configuration is wrong.
                    ESP_LOGW(TAG, "Received a heartbeat from a GCS on the UART. Check your "
                                  "configuration - this ESP32 should be wired to a flight "
                                  "controller, not a ground station.");
                } else if (payload.autopilot != MAV_AUTOPILOT_INVALID && new_msg->compid == MAV_COMP_ID_AUTOPILOT1) {
                    ESP_LOGD(TAG, "Got heartbeat from flight controller (sysID: %i)", new_msg->sysid);
                    // This means we are connected to the FC since we only parse mavlink on UART and thus only see the
                    // device we are connected to via UART
                    DB_MAV_SYS_ID = new_msg->sysid;
                    db_mavlink_update_fc_state(new_msg, &payload);
                    // Check if FC is armed and the Wi-Fi switch based on armed status is configured by the user
                    if (DB_PARAM_DIS_RADIO_ON_ARM &&
                    (payload.base_mode & MAV_MODE_FLAG_SAFETY_ARMED ||
                    (payload.system_status > MAV_STATE_STANDBY && payload.system_status != MAV_STATE_POWEROFF))) {
                        // autopilot indicates it is armed
                        db_set_radio_status(false);
                    } else {
                        // autopilot indicates it is <<not>> armed
                        db_set_radio_status(true);
                    }
                } else {
                    // We do not react to any other heartbeat!
                }
            } // do not react to heartbeats received via wireless interface - reaction to serial is sufficient
            break;
        case FASTMAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
            ESP_LOGI(TAG, "Received PARAM_REQUEST_LIST msg. Responding with parameters");
            float_int_union float_int;
            uint16_t len = 0;
            for (int i = 0; i < sizeof(db_params) / sizeof(db_params[0]); i++) {
                switch (db_params[i]->type) {
                    case STRING:
                        // ignoring strings. Not supported with this request
                        continue;
                    break;
                    case UINT8:
                        float_int.uint8 = db_params[i]->value.db_param_u8.value;
                    break;
                    case UINT16:
                        float_int.uint16 = db_params[i]->value.db_param_u16.value;
                    break;
                    case INT32:
                        float_int.int32 = db_params[i]->value.db_param_i32.value;
                    break;
                    default:
                        ESP_LOGE(TAG, "db_param_write_all_params_json() -> db_parameter.type unknown!");
                    break;
                }
                len = db_get_mavmsg_param_value(buff, fmav_status, db_params[i]->mav_t.param_index, &float_int,
                                                db_params[i]->mav_t.param_type, (char *) db_params[i]->mav_t.param_name);
                db_route_mavlink_response(buff, len, origin, tcp_clients, udp_conns);
            }
        }
            break;
        case FASTMAVLINK_MSG_ID_PARAM_REQUEST_READ: {
            fmav_param_request_read_t payload;
            fmav_msg_param_request_read_decode(&payload, new_msg);
            float_int_union float_int;
            ESP_LOGI(TAG, "GCS request reading parameter ID: %s with index %i", payload.param_id, payload.param_index);
            MAV_PARAM_TYPE type = db_mav_get_parameter_value(&float_int, payload.param_id, payload.param_index);
            if (type != 0) {
                uint16_t len = db_get_mavmsg_param_value(buff, fmav_status, payload.param_index, &float_int, type,
                                                         payload.param_id);
                db_route_mavlink_response(buff, len, origin, tcp_clients, udp_conns);
            } else {
                // send nothing, unknown parameter
                ESP_LOGW(TAG, "\tParameter is unknown. Not responding!");
            }
        }
            break;
        case FASTMAVLINK_MSG_ID_PARAM_VALUE: {
            /* A PARAM_VALUE from the FC is the answer to a dump or the echo of a
             * load write. Feed the collector; it ignores anything it did not ask
             * for. Falls through to normal routing so a GCS still sees it. */
            if (origin == DB_MAVLINK_DATA_ORIGIN_SERIAL) {
                fmav_param_value_t pv;
                fmav_msg_param_value_decode(&pv, new_msg);
                char id[DB_FC_PARAM_ID_LEN + 1];
                memset(id, 0, sizeof(id));
                memcpy(id, pv.param_id, DB_FC_PARAM_ID_LEN);
                db_fc_params_on_param_value(id, pv.param_value, pv.param_type,
                                            pv.param_index, pv.param_count);
                db_fc_tune_on_param_value(id, pv.param_value, pv.param_type);
            }
            break;
        }
        case FASTMAVLINK_MSG_ID_PARAM_SET: {
            fmav_param_set_t parame_set_payload;
            fmav_msg_param_set_decode(&parame_set_payload, new_msg);
            ESP_LOGI(TAG, "GCS requested setting parameter %s", parame_set_payload.param_id);
            if (db_write_mavlink_parameter(&parame_set_payload)) {
                // Respond with parameter
                float_int_union float_int;
                MAV_PARAM_TYPE type = db_mav_get_parameter_value(&float_int, parame_set_payload.param_id, -1);
                if (type != 0) {
                    uint16_t len = db_get_mavmsg_param_value(buff, fmav_status, 0, &float_int, type,
                                                             parame_set_payload.param_id);
                    db_route_mavlink_response(buff, len, origin, tcp_clients, udp_conns);
                    db_write_settings_to_nvs();
                } else {
                    ESP_LOGE(TAG, "Failed to get parameter %s - could not respond with new param", parame_set_payload.param_id);
                }
            } else {
                ESP_LOGE(TAG, "db_write_mavlink_parameter() failed to set new parameter %s ", parame_set_payload.param_id);
            }
        }
            break;
        case FASTMAVLINK_MSG_ID_PARAM_EXT_REQUEST_LIST: {
            ESP_LOGI(TAG, "GCS requested reading ext parameters list - not supported for now!");
            // ToDo Respond to PARAM_EXT_REQUEST_LIST
        }
            break;
        case FASTMAVLINK_MSG_ID_PARAM_EXT_REQUEST_READ: {
            ESP_LOGI(TAG, "GCS requested reading ext parameter - not supported for now!");
            // ToDo Respond to EXT_REQUEST_READ
        }
            break;
        case FASTMAVLINK_MSG_ID_COMMAND_LONG: {
            fmav_command_long_t payload;
            fmav_msg_command_long_decode(&payload, new_msg);
            ESP_LOGI(TAG, "Received command long with ID: %hu", payload.command);
            db_process_mavlink_command(&payload, new_msg, fmav_status, buff, origin, tcp_clients, udp_conns);
        }
            break;
        case FASTMAVLINK_MSG_ID_PING: {
            fmav_ping_t payload;
            fmav_msg_ping_decode(&payload, new_msg);
            payload.target_system = new_msg->sysid;
            payload.target_component = new_msg->compid;
            uint16_t len = fmav_msg_ping_encode_to_frame_buf(buff, db_get_mav_sys_id(), db_get_mav_comp_id(), &payload, fmav_status);
            ESP_LOGD(TAG, "Answering MAVLink ping from System %i, Component %i", new_msg->sysid, new_msg->compid);
            db_route_mavlink_response(buff, len, origin, tcp_clients, udp_conns);
        }
            break;
        case FASTMAVLINK_MSG_ID_REQUEST_DATA_STREAM: {
            fmav_request_data_stream_t data_stream_pay;
            fmav_msg_request_data_stream_decode(&data_stream_pay, new_msg);
            ESP_LOGW(TAG, "GCS requested data stream with ID: %i and rate: %i and start_stop: %i - ignoring!",
                     data_stream_pay.req_stream_id, data_stream_pay.req_message_rate, data_stream_pay.start_stop);
        }
            break;
        case FASTMAVLINK_MSG_ID_TUNNEL:
            /*
             * A linked private DBB brain may consume diagnostic/application
             * TUNNEL packets in the hook above. The open base intentionally
             * has no TUNNEL behavior, but this known message must not fall
             * into the per-packet "unknown targeted message" warning.
             */
            break;
        default: {
            if (new_msg->target_sysid == db_get_mav_sys_id() && new_msg->target_compid == db_get_mav_comp_id()) {
                ESP_LOGW(TAG, "Received unknown MAVLink message ID: %u - ignoring", new_msg->msgid);
            }
            break;
        }
    }
}
