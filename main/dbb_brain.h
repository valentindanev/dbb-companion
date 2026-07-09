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

#ifdef __cplusplus
}
#endif
