/*
 * On-boat steering and speed controller tuning.
 *
 * This is a C reimplementation of ArduPilot's rover-quicktune.lua applet, which
 * cannot run on this boat: the DAKEFPV H743 has no microSD card, so Lua scripts
 * cannot be loaded at all (the board doc says so outright, and MAVFTP confirms
 * it - every filesystem operation returns FileNotFound).
 *
 * The algorithm is deliberately identical to the applet's, because the applet
 * does something very simple: it measures the plant's steady-state gain by
 * averaging, and sets the gains from that.
 *
 *   steering:  FF = (sum |steering_out| / sum |turn_rate_rads|) * STR_FFRATIO
 *              P  = FF * STR_P_RATIO
 *              I  = FF * STR_I_RATIO
 *
 *   speed:     CRUISE_SPEED    = mean(forward speed)
 *              CRUISE_THROTTLE = mean(throttle_out) * 100 * SPD_FFRATIO
 *              ff_equivalent   = (sum throttle_out / sum speed) * SPD_FFRATIO
 *              ATC_SPEED_P/I   = ff_equivalent * SPD_{P,I}_RATIO
 *
 * TWO DELIBERATE DIFFERENCES FROM THE APPLET, both documented at the use site:
 *
 *  1. The applet drives the vehicle itself in Circle mode. This does NOT. The
 *     operator holds a turn manually and the boat only watches. That removes
 *     every autonomy risk from what is intended to become a customer-facing
 *     feature, and the arithmetic is unaffected - it does not care what
 *     produced the turn.
 *
 *  2. The applet gates on `steering_out >= 0.10`, i.e. positive only, because
 *     Circle mode always turns the same way. This gates on |steering_out| and
 *     accumulates magnitudes, so a circle in either direction works.
 *
 * Sampling runs at the 100 ms timer's 10 Hz rather than the applet's 40 Hz.
 * The algorithm only ever computes means over >= 10 s, so what matters is the
 * sample count, not the rate; 10 Hz over 10 s is 100 samples.
 *
 * NOTHING IS WRITTEN TO THE FC WITHOUT AN EXPLICIT SECOND STEP. The tune
 * computes, reports before/after, and stops. db_fc_tune_apply() performs the
 * write, through db_fc_params so every value is echo-verified.
 */
#ifndef DB_FC_TUNE_H
#define DB_FC_TUNE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#define DB_FC_TUNE_AXIS_STEERING 0x01
#define DB_FC_TUNE_AXIS_SPEED    0x02

#define DB_FC_TUNE_ID_LEN 16
#define DB_FC_TUNE_MAX_RESULTS 10

typedef enum {
  DB_FC_TUNE_IDLE = 0,
  DB_FC_TUNE_BASELINE,   /* reading the current values off the FC */
  DB_FC_TUNE_STEERING,   /* collecting turn-rate samples */
  DB_FC_TUNE_SPEED,      /* collecting speed samples */
  DB_FC_TUNE_READY,      /* computed, waiting for the operator to apply */
  DB_FC_TUNE_WRITING,    /* handed to db_fc_params */
  DB_FC_TUNE_COMPLETE,
  DB_FC_TUNE_FAILED,
} db_fc_tune_state_t;

typedef struct {
  char    id[DB_FC_TUNE_ID_LEN + 1];
  float   before;
  float   after;
  uint8_t type;     /* MAV_PARAM_TYPE_* the FC reported for this parameter.
                     * Integer types are rounded before the write so the
                     * read-back verify can actually match - see add_result_p(). */
  bool    applied;  /* true once the FC echoed THIS value back */
} db_fc_tune_result_t;

typedef struct {
  db_fc_tune_state_t state;
  uint8_t  axes;              /* which axes this run was asked to tune */
  uint8_t  steer_pct;         /* 0..100 collection progress */
  uint8_t  speed_pct;
  uint16_t steer_samples;
  uint16_t speed_samples;
  float    steer_rotation_deg; /* accumulated rotation, for the operator */
  /* live values, so the page can show WHY it is or is not collecting */
  float    live_steering;      /* -1..1 */
  float    live_turnrate_dps;
  float    live_throttle;      /* -1..1 */
  float    live_speed_mps;
  bool     live_valid;
  uint8_t  result_count;
  db_fc_tune_result_t results[DB_FC_TUNE_MAX_RESULTS];
  char     error[96];
} db_fc_tune_status_t;

/** Begin a tune. axes is a mask of DB_FC_TUNE_AXIS_*. */
esp_err_t db_fc_tune_start(uint8_t axes);

/** Write the computed gains to the FC. Only valid in DB_FC_TUNE_READY. */
esp_err_t db_fc_tune_apply(void);

/** Stop and discard. Nothing has been written unless apply() ran. */
void db_fc_tune_abort(const char *why);

void db_fc_tune_get_status(db_fc_tune_status_t *out);

/** Feed a PARAM_VALUE from the FC. Ignored unless a baseline read is running. */
/**
 * Feed one PARAM_VALUE to the baseline collector.
 *
 * param_type matters: Rover declares CRUISE_THROTTLE as AP_Int8, so asking
 * for 64.88 stores 64 and the read-back verify can never match. Capturing the
 * type here lets compute_*() round to what the FC can actually hold.
 */
void db_fc_tune_on_param_value(const char *param_id, float value,
                               uint8_t param_type);

/** Drives collection, timeouts and the apply hand-off. Call at 10 Hz. */
void db_fc_tune_tick(void);

/** Console ring, same contract as db_fc_flash_console_read(). */
size_t db_fc_tune_console_read(uint32_t since_seq, char *out, size_t out_len,
                               uint32_t *next_seq);

#endif /* DB_FC_TUNE_H */
