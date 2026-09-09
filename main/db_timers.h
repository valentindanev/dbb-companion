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

#ifndef DB_ESP32_DB_TIMERS_H
#define DB_ESP32_DB_TIMERS_H

#define DB_TIMER_RSSI_PERIOD_MS 1000
#define DB_TIMER_MAVLINK_HEARTBEAT_MS 1000   // Heartbeat every second
#define DB_TIMER_MAVLINK_RADIOSTATUS_MS 1000 // Radio Status every second
// Sonar every 500ms (2Hz). Was 100ms/10Hz until 16-08-2026. The FC resamples the
// rangefinder onto its EXTRA3 stream, so publishing faster than the FC re-emits buys
// nothing downstream (10Hz and 5Hz emitters produced identical app logs), while every
// extra message is relayed onto the ~10.4kbps ELRS link. 2Hz drops that relayed load
// from ~20% to ~4%, which lets MAV4_OPTIONS stay 0 and keeps Mission Planner and
// TUNNEL(385) working over the ESP. Sensor polling is unaffected: danevi_sonar_task
// still triggers the GL041MT every 100ms, so the published value is always fresh.
#define DB_TIMER_MAVLINK_SONAR_MS 500        // Sonar every 500ms (2Hz)

#include <stdint.h>

void db_timer_start_wifi_rssi_timer();

void db_timer_start_mavlink_heartbeat();

void db_timer_start_mavlink_radio_status();

void db_timer_start_mavlink_sonar();

#endif // DB_ESP32_DB_TIMERS_H
