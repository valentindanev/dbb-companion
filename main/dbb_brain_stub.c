/*
 *  DBB Companion - weak no-op brain stubs (open base).
 *
 *  These weak symbols make the public firmware a complete sonar + MAVLink
 *  bridge with no autonomous "brain". If a private brain component is linked
 *  in, its strong symbols override these at link time (see README).
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  Copyright 2026 Valentin Danev
 */
#include "dbb_brain.h"
#include "esp_log.h"

__attribute__((weak)) void dbb_brain_init(void) {
    ESP_LOGI("DBB_BRAIN", "open base - no brain linked (stub)");
}

__attribute__((weak)) void dbb_brain_handle_mavlink(const fmav_message_t *msg, bool from_serial) {
    (void)msg;
    (void)from_serial;
}

__attribute__((weak)) void dbb_brain_register_http(void *server) {
    (void)server;
}
