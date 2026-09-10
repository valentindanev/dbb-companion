#include "sonar_compare_core.h"

#include <stdio.h>
#include <string.h>

static uint32_t abs_diff_u32(uint32_t a, uint32_t b) {
  return a >= b ? a - b : b - a;
}

static void set_reason(sonar_compare_core_t *core, const char *reason) {
  if (core == NULL) return;
  snprintf(core->reason, sizeof(core->reason), "%s", reason == NULL ? "" : reason);
}

void sonar_compare_core_init(sonar_compare_core_t *core, bool compiled,
                             bool deeper_connected, int output_source) {
  if (core == NULL) return;
  memset(core, 0, sizeof(*core));
  core->compiled = compiled;
  core->output_source_at_boot = output_source;
  if (!compiled) {
    core->state = SONAR_COMPARE_IDLE;
    set_reason(core, "comparison test is not compiled into this firmware");
  } else if (!deeper_connected) {
    core->state = SONAR_COMPARE_INVALID;
    set_reason(core, "Deeper AP connection failed; comparison cannot start");
  } else {
    core->state = SONAR_COMPARE_READY;
    set_reason(core, "both sensors must be live before recording");
  }
}

bool sonar_compare_output_authority_ok(const sonar_compare_core_t *core,
                                       int current_output_source) {
  if (core == NULL) return false;
#ifdef DB_MUTATE_COMPARE_OUTPUT_AUTHORITY_GUARD
  (void)current_output_source;
  return true;
#else
  return current_output_source == core->output_source_at_boot;
#endif
}

sonar_compare_start_result_t sonar_compare_core_start(
    sonar_compare_core_t *core, bool preflight_complete,
    bool uart_nmea_emitter_active, bool aj_live, bool deeper_live,
    int current_output_source) {
  if (core == NULL || !core->compiled) {
    return SONAR_COMPARE_START_NOT_COMPILED;
  }
  if (core->state == SONAR_COMPARE_RECORDING) {
    return SONAR_COMPARE_START_ALREADY_RECORDING;
  }
#ifndef DB_MUTATE_COMPARE_PREFLIGHT_GUARD
  if (!preflight_complete) {
    core->state = SONAR_COMPARE_INVALID;
    set_reason(core, "preflight incomplete: join Deeper AP, load this page, verify banner and empty download");
    return SONAR_COMPARE_START_PREFLIGHT_REQUIRED;
  }
#else
  (void)preflight_complete;
#endif
#ifndef DB_MUTATE_COMPARE_UART_CONFLICT_GUARD
  if (uart_nmea_emitter_active) {
    core->state = SONAR_COMPARE_INVALID;
    set_reason(core, "refused: synthetic UART-NMEA emitter owns AJ trigger pin/UART2");
    return SONAR_COMPARE_START_UART_NMEA_ACTIVE;
  }
#else
  (void)uart_nmea_emitter_active;
#endif
#ifndef DB_MUTATE_COMPARE_BOTH_LIVE_GUARD
  if (!aj_live) {
    core->state = SONAR_COMPARE_INVALID;
    set_reason(core, "refused: AJ pulse source is not live");
    return SONAR_COMPARE_START_AJ_NOT_LIVE;
  }
  if (!deeper_live) {
    core->state = SONAR_COMPARE_INVALID;
    set_reason(core, "refused: Deeper source is not live");
    return SONAR_COMPARE_START_DEEPER_NOT_LIVE;
  }
#else
  (void)aj_live;
  (void)deeper_live;
#endif
  if (!sonar_compare_output_authority_ok(core, current_output_source)) {
    core->state = SONAR_COMPARE_INVALID;
    set_reason(core, "refused: production sonar output authority changed");
    return SONAR_COMPARE_START_OUTPUT_AUTHORITY_CHANGED;
  }
  core->state = SONAR_COMPARE_RECORDING;
  core->session_id = 0;
  core->valid_pairs = 0;
  set_reason(core, "recording raw observations; comparison publishes no MAVLink depth");
  return SONAR_COMPARE_START_OK;
}

void sonar_compare_core_stop(sonar_compare_core_t *core) {
  if (core == NULL || core->state != SONAR_COMPARE_RECORDING) return;
  core->state = SONAR_COMPARE_STOPPED;
  set_reason(core, "log sealed; download the comparison bundle");
}

void sonar_compare_core_note_aj(sonar_compare_core_t *core, uint32_t boot_ms,
                                bool valid, int depth_mm, bool timeout) {
  if (core == NULL || !core->compiled) return;
  core->aj_polls++;
  if (timeout) core->aj_timeouts++;
  if (valid && depth_mm == 0) core->aj_zeros++;
  core->aj.present = true;
  core->aj.valid = valid;
  core->aj.depth_mm = depth_mm;
  core->aj.boot_ms = boot_ms;
}

void sonar_compare_core_note_deeper(sonar_compare_core_t *core,
                                    uint32_t boot_ms, bool valid,
                                    int depth_mm) {
  if (core == NULL || !core->compiled) return;
  core->deeper_samples++;
  core->deeper.present = true;
  core->deeper.valid = valid;
  core->deeper.depth_mm = depth_mm;
  core->deeper.boot_ms = boot_ms;
}

sonar_compare_pair_t sonar_compare_core_pair(const sonar_compare_core_t *core,
                                             uint32_t now_ms) {
  sonar_compare_pair_t pair = {0};
  pair.quality = SONAR_COMPARE_PAIR_MISSING_BOTH;
  if (core == NULL) return pair;
#ifndef DB_MUTATE_COMPARE_MISSING_GUARD
  if (!core->aj.present || !core->aj.valid) {
    pair.quality = (!core->deeper.present || !core->deeper.valid)
                       ? SONAR_COMPARE_PAIR_MISSING_BOTH
                       : SONAR_COMPARE_PAIR_MISSING_AJ;
    return pair;
  }
  if (!core->deeper.present || !core->deeper.valid) {
    pair.quality = SONAR_COMPARE_PAIR_MISSING_DEEPER;
    return pair;
  }
#endif
  if (now_ms - core->aj.boot_ms > SONAR_COMPARE_SOURCE_LIVE_MS ||
      now_ms - core->deeper.boot_ms > SONAR_COMPARE_SOURCE_LIVE_MS) {
    pair.quality = SONAR_COMPARE_PAIR_STALE;
    return pair;
  }
  pair.separation_ms = abs_diff_u32(core->aj.boot_ms, core->deeper.boot_ms);
#ifndef DB_MUTATE_COMPARE_PAIR_AGE_GUARD
  if (pair.separation_ms > SONAR_COMPARE_PRIMARY_PAIR_WINDOW_MS) {
    pair.quality = SONAR_COMPARE_PAIR_REJECTED_AGE;
    return pair;
  }
#endif
  pair.quality = SONAR_COMPARE_PAIR_PAIRED;
  pair.difference_mm = core->aj.depth_mm - core->deeper.depth_mm;
  if (core->deeper.depth_mm != 0) {
    pair.difference_percent =
        100.0 * (double)pair.difference_mm / (double)core->deeper.depth_mm;
  }
  int absolute = pair.difference_mm < 0 ? -pair.difference_mm : pair.difference_mm;
  pair.agreement = absolute <= SONAR_COMPARE_AGREEMENT_MM;
  return pair;
}

int sonar_compare_required_model_id(int stored_model_id) {
#ifdef DB_MUTATE_COMPARE_REGISTRY_GUARD
  return stored_model_id;
#else
  (void)stored_model_id;
  return SONAR_COMPARE_AJ_PULSE_MODEL_ID;
#endif
}

const char *sonar_compare_state_name(sonar_compare_state_t state) {
  switch (state) {
    case SONAR_COMPARE_IDLE: return "idle";
    case SONAR_COMPARE_READY: return "ready";
    case SONAR_COMPARE_RECORDING: return "recording";
    case SONAR_COMPARE_STOPPED: return "stopped";
    case SONAR_COMPARE_INVALID: return "invalid";
  }
  return "invalid";
}

const char *sonar_compare_pair_quality_name(sonar_compare_pair_quality_t quality) {
  switch (quality) {
    case SONAR_COMPARE_PAIR_MISSING_BOTH: return "nothing to compare";
    case SONAR_COMPARE_PAIR_MISSING_AJ: return "missing AJ";
    case SONAR_COMPARE_PAIR_MISSING_DEEPER: return "missing Deeper";
    case SONAR_COMPARE_PAIR_STALE: return "stale";
    case SONAR_COMPARE_PAIR_REJECTED_AGE: return "rejected by pair age";
    case SONAR_COMPARE_PAIR_PAIRED: return "paired";
  }
  return "nothing to compare";
}

const char *sonar_compare_start_result_name(sonar_compare_start_result_t result) {
  switch (result) {
    case SONAR_COMPARE_START_OK: return "ok";
    case SONAR_COMPARE_START_NOT_COMPILED: return "not compiled";
    case SONAR_COMPARE_START_PREFLIGHT_REQUIRED: return "preflight required";
    case SONAR_COMPARE_START_UART_NMEA_ACTIVE: return "UART-NMEA conflict";
    case SONAR_COMPARE_START_AJ_NOT_LIVE: return "AJ not live";
    case SONAR_COMPARE_START_DEEPER_NOT_LIVE: return "Deeper not live";
    case SONAR_COMPARE_START_OUTPUT_AUTHORITY_CHANGED: return "output authority changed";
    case SONAR_COMPARE_START_ALREADY_RECORDING: return "already recording";
  }
  return "invalid";
}
