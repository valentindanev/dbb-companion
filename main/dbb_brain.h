/*
 *  DBB Companion - boat "brain" hook interface.
 *
 *  The open base calls these hooks. In the public build they resolve to the
 *  weak no-op stubs in dbb_brain_stub.c, giving a plain sonar + MAVLink bridge.
 *  A private brain component may provide *strong* implementations that override
 *  the stubs at link time (see the project README - two-part architecture).
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  Copyright 2026 Valentin Danev
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "lib/fastmavlink_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Called once at boot, after the control module + MAVLink stack are up. */
void dbb_brain_init(void);

/**
 * Called for every parsed MAVLink message.
 * @param msg         the parsed message
 * @param from_serial true if it arrived on the UART/FC link (vs the radio side)
 */
void dbb_brain_handle_mavlink(const fmav_message_t *msg, bool from_serial);

/**
 * Called once from the web server (start_rest_server) so a linked brain can
 * register its own HTTP routes (e.g. a private monitor page) before the
 * catch-all file handler. `server` is an esp_http_server httpd_handle_t, kept
 * as void* here so the open-base header carries no HTTP dependency. No-op stub
 * in the public build.
 */
void dbb_brain_register_http(void *server);

/**
 * OTA health contribution from a linked brain.
 *
 * The open base cannot know whether a brain needs a health bit, so it asks.
 * Called from app_main immediately before db_start_ota_health_gate(), which is
 * the last point at which the required mask can still change - dbb_brain_init()
 * has run by then, so a brain may report the outcome of its own start-up.
 *
 * The stub sets all three to 0, so an open-base build requires nothing extra and
 * the gate behaves exactly as it did before the hook existed.
 *
 * @param required bits the brain needs gated this boot (e.g. DB_OTA_HEALTH_BRAIN)
 * @param passed   bits it has already satisfied
 * @param failed   bits it has already failed
 */
void dbb_brain_ota_health(uint32_t *required, uint32_t *passed, uint32_t *failed);

#ifdef __cplusplus
}
#endif
