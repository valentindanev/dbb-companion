#include "db_fc_tune.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "common/mavlink.h"
#include "db_fc_params.h"
#include "db_mavlink_msgs.h"
#include "db_serial.h"

#define TAG "DB_FCTUNE"

#define FC_SYSID 1
#define FC_COMPID 1

/* ---- algorithm constants, matching rover-quicktune.lua ------------------- */
#define STR_FFRATIO   0.9f
#define STR_P_RATIO   0.5f
#define STR_I_RATIO   0.5f
#define SPD_FFRATIO   1.0f
#define SPD_P_RATIO   1.0f
#define SPD_I_RATIO   1.0f
#define FLT_MUL       0.5f     /* FLTT and FLTD = 0.5 * INS_GYRO_FILTER */

#define STR_STEERING_MIN   0.10f            /* |steering_out| */
#define STR_TURNRATE_MIN   0.1745329f       /* 10 deg/s in rad/s */
#define SPD_THROTTLE_MIN   0.20f
#define SPD_SPEED_MIN      0.50f            /* m/s */

/* The applet needs 10 s of qualifying data. Our tick is 10 Hz. */
#define TICK_HZ            10
#define REQUIRED_SAMPLES   (10 * TICK_HZ)
/* One full circle, so the FF average is not taken from a single brief swerve. */
#define REQUIRED_ROTATION_RAD (2.0f * (float)M_PI)

#define BASELINE_TIMEOUT_MS  10000
#define BASELINE_RETRY_MS    500
#define STALL_WARN_MS        15000
#define OVERALL_TIMEOUT_MS   420000   /* 7 minutes of trying is enough */

/* ---- baseline parameters ------------------------------------------------ */
enum {
  P_STR_FF = 0, P_STR_P, P_STR_I, P_STR_D, P_STR_FLTT, P_STR_FLTD,
  P_SPD_P, P_SPD_I,
  P_CRUISE_SPEED, P_CRUISE_THROTTLE,
  P_GYRO_FILTER,
  P_S1_MIN, P_S1_TRIM, P_S1_MAX,
  P_S2_MIN, P_S2_TRIM, P_S2_MAX,
  P_COUNT
};

static const char *const BASELINE_NAMES[P_COUNT] = {
    "ATC_STR_RAT_FF", "ATC_STR_RAT_P", "ATC_STR_RAT_I", "ATC_STR_RAT_D",
    "ATC_STR_RAT_FLTT", "ATC_STR_RAT_FLTD",
    "ATC_SPEED_P", "ATC_SPEED_I",
    "CRUISE_SPEED", "CRUISE_THROTTLE",
    "INS_GYRO_FILTER",
    "SERVO1_MIN", "SERVO1_TRIM", "SERVO1_MAX",
    "SERVO2_MIN", "SERVO2_TRIM", "SERVO2_MAX",
};

/* ---- console ring, same shape as db_fc_flash ---------------------------- */
#define CONSOLE_LINES 96
#define CONSOLE_LINE_MAX 120

static struct {
  char line[CONSOLE_LINES][CONSOLE_LINE_MAX];
  uint32_t seq[CONSOLE_LINES];
  uint32_t next_seq;
} s_console;

static SemaphoreHandle_t s_mutex;

static void tune_log(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void tune_log(const char *fmt, ...) {
  char buf[CONSOLE_LINE_MAX];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  ESP_LOGI(TAG, "%s", buf);
  uint32_t slot = s_console.next_seq % CONSOLE_LINES;
  snprintf(s_console.line[slot], CONSOLE_LINE_MAX, "%s", buf);
  s_console.seq[slot] = s_console.next_seq;
  s_console.next_seq++;
}

size_t db_fc_tune_console_read(uint32_t since_seq, char *out, size_t out_len,
                               uint32_t *next_seq) {
  size_t used = 0;
  if (out_len == 0) {
    if (next_seq) *next_seq = s_console.next_seq;
    return 0;
  }
  out[0] = '\0';
  uint32_t first = s_console.next_seq > CONSOLE_LINES
                       ? s_console.next_seq - CONSOLE_LINES : 0;
  if (since_seq < first) since_seq = first;
  for (uint32_t s = since_seq; s < s_console.next_seq; s++) {
    uint32_t slot = s % CONSOLE_LINES;
    if (s_console.seq[slot] != s) continue;
    size_t n = strlen(s_console.line[slot]);
    if (used + n + 2 > out_len) break;
    memcpy(out + used, s_console.line[slot], n);
    used += n;
    out[used++] = '\n';
  }
  /* MUST terminate: the caller hands this straight to cJSON as a C string, so
   * without it cJSON reads past `used` into whatever the heap block held
   * before - which leaked raw HTTP request bytes into the JSON response the
   * first time this ran. The `+ 2` above reserves room for the newline AND
   * this NUL, so there is always space. */
  out[used] = '\0';
  if (next_seq) *next_seq = s_console.next_seq;
  return used;
}

/* ---- state -------------------------------------------------------------- */
static db_fc_tune_state_t s_state = DB_FC_TUNE_IDLE;
static uint8_t s_axes;
static char s_error[96];

static float s_base[P_COUNT];
static bool s_base_got[P_COUNT];
static uint8_t s_base_type[P_COUNT];   /* MAV_PARAM_TYPE_* as the FC reports it */
static uint32_t s_base_last_req_ms;
static uint32_t s_phase_start_ms;
static uint32_t s_last_qualify_ms;
static uint32_t s_last_stall_warn_ms;

/* steering accumulators */
static double s_str_steer_sum;
static double s_str_rate_sum;      /* sum |rad/s| */
static double s_str_rotation_rad;  /* integral of |rad/s| dt */
static uint16_t s_str_count;

/* speed accumulators */
static double s_spd_thr_sum;
static double s_spd_spd_sum;
static uint16_t s_spd_count;

/* live values for the UI */
static float s_live_steer, s_live_rate_dps, s_live_thr, s_live_speed;
static bool s_live_valid;

static db_fc_tune_result_t s_results[DB_FC_TUNE_MAX_RESULTS];
static uint8_t s_result_count;

static uint32_t now_ms(void) { return (uint32_t)(esp_timer_get_time() / 1000ULL); }

static void ensure_mutex(void) {
  if (s_mutex == NULL) s_mutex = xSemaphoreCreateMutex();
}

static void fail_locked(const char *why) {
  s_state = DB_FC_TUNE_FAILED;
  snprintf(s_error, sizeof(s_error), "%s", why ? why : "failed");
  tune_log("FAILED: %s", s_error);
}

/* MAV_PARAM_TYPE_UINT8(1) .. INT64(8) are the integer types; REAL32 is 9. */
static bool param_type_is_integer(uint8_t t) { return t >= 1 && t <= 8; }

/**
 * Record one computed gain, keyed by its baseline index so the name, the
 * previous value and the FC's declared type can never be mismatched.
 *
 * Integer parameters are ROUNDED here rather than sent raw. ArduPilot casts
 * on write, so asking an AP_Int8 for 64.88 stores 64; the verifier then
 * compares 64 against 64.88 with a |want|*1e-4 tolerance and calls a perfectly
 * good write a failure. That is exactly what produced the misleading
 * "1 parameter(s) did not take" on 22-08-2026. Rounding first means we ask for
 * 65, the FC stores 65, and the read-back matches - and the operator is shown
 * the value that will really be in effect.
 */
static void add_result_p(int p, float after) {
  if (s_result_count >= DB_FC_TUNE_MAX_RESULTS) return;
  if (param_type_is_integer(s_base_type[p])) after = roundf(after);
  db_fc_tune_result_t *r = &s_results[s_result_count++];
  snprintf(r->id, sizeof(r->id), "%s", BASELINE_NAMES[p]);
  r->before = s_base[p];
  r->after = after;
  r->type = s_base_type[p];
  r->applied = false;
  tune_log("  %-17s %10.4f -> %10.4f", r->id, r->before, r->after);
}

/* Is `id` one of the newline-separated names in `list`? */
static bool name_in_nl_list(const char *list, const char *id) {
  if (list == NULL || id == NULL) return false;
  size_t n = strlen(id);
  for (const char *p = list; *p != '\0';) {
    const char *e = strchr(p, '\n');
    size_t len = e ? (size_t)(e - p) : strlen(p);
    if (len == n && strncmp(p, id, n) == 0) return true;
    if (e == NULL) break;
    p = e + 1;
  }
  return false;
}

/* ---- transmit ----------------------------------------------------------- */

static void send_request_read_name(const char *name) {
  uint8_t buf[296];
  fmav_status_t status = {0};
  fmav_param_request_read_t payload = {.param_index = -1,
                                       .target_system = FC_SYSID,
                                       .target_component = FC_COMPID};
  memset(payload.param_id, 0, sizeof(payload.param_id));
  strncpy(payload.param_id, name, sizeof(payload.param_id));
  uint16_t len = fmav_msg_param_request_read_encode_to_frame_buf(
      buf, db_get_mav_sys_id(), db_get_mav_comp_id(), &payload, &status);
  if (len) write_to_serial(buf, len);
}

/* ---- sampling ----------------------------------------------------------- */

/*
 * PWM -> -1..+1 against the channel's OWN endpoints. This must not assume
 * 1100/1500/1900: the boat's throttle range is deliberately narrowed to limit
 * top speed, and assuming the wide range would scale every sample wrong.
 */
static float norm_pwm(uint16_t pwm, float mn, float tr, float mx) {
  if (pwm == 0) return 0.0f;
  float p = (float)pwm;
  if (p >= tr) return (mx > tr) ? (p - tr) / (mx - tr) : 0.0f;
  return (tr > mn) ? (p - tr) / (tr - mn) : 0.0f;
}

/* Returns false when the telemetry needed is not currently available. */
static bool sample_now(float *steering, float *throttle, float *turnrate_rads,
                       float *fwd_speed) {
  db_mavlink_telemetry_t t;
  db_mavlink_get_telemetry(&t);
  if (!t.servo.valid || !t.attitude.valid) return false;

  float l = norm_pwm(t.servo.raw[0], s_base[P_S1_MIN], s_base[P_S1_TRIM],
                     s_base[P_S1_MAX]);
  float r = norm_pwm(t.servo.raw[1], s_base[P_S2_MIN], s_base[P_S2_TRIM],
                     s_base[P_S2_MAX]);
  /* ArduPilot skid mixing is left = throttle + steering, right = throttle -
   * steering, so both come straight back out of the two outputs. */
  *throttle = (l + r) * 0.5f;
  *steering = (l - r) * 0.5f;
  *turnrate_rads = t.attitude.yawspeed;

  *fwd_speed = 0.0f;
  if (t.position.valid) {
    float yaw = t.attitude.yaw_rad;
    float vn = (float)t.position.vx_cms * 0.01f;
    float ve = (float)t.position.vy_cms * 0.01f;
    *fwd_speed = vn * cosf(yaw) + ve * sinf(yaw);
  }
  return true;
}

/* ---- phase transitions -------------------------------------------------- */

static void begin_steering_locked(void) {
  s_state = DB_FC_TUNE_STEERING;
  s_phase_start_ms = s_last_qualify_ms = now_ms();
  s_str_steer_sum = s_str_rate_sum = s_str_rotation_rad = 0.0;
  s_str_count = 0;
  tune_log("STEERING: hold a steady turn - need |steering| >= %.2f and",
           STR_STEERING_MIN);
  tune_log("  turn rate >= 10 deg/s, for 10 s and one full circle");
}

static void begin_speed_locked(void) {
  s_state = DB_FC_TUNE_SPEED;
  s_phase_start_ms = s_last_qualify_ms = now_ms();
  s_spd_thr_sum = s_spd_spd_sum = 0.0;
  s_spd_count = 0;
  tune_log("SPEED: now drive STRAIGHT at a steady cruise - need throttle");
  tune_log("  >= %.0f%% and speed >= %.1f m/s, for 10 s", SPD_THROTTLE_MIN * 100,
           SPD_SPEED_MIN);
}

static void finish_collection_locked(void) {
  s_state = DB_FC_TUNE_READY;
  tune_log("--- computed, nothing written yet ---");
}

static void compute_steering_locked(void) {
  if (s_str_rate_sum <= 0.0) {
    fail_locked("no turn-rate data");
    return;
  }
  float ff = (float)(s_str_steer_sum / s_str_rate_sum) * STR_FFRATIO;
  tune_log("STEERING done: %u samples, %.0f deg of rotation", s_str_count,
           s_str_rotation_rad * 57.2958);
  tune_log("  mean steering %.4f / mean rate %.4f rad/s",
           s_str_steer_sum / s_str_count, s_str_rate_sum / s_str_count);
  add_result_p(P_STR_FF, ff);
  add_result_p(P_STR_P, ff * STR_P_RATIO);
  add_result_p(P_STR_I, ff * STR_I_RATIO);
  if (s_base[P_GYRO_FILTER] > 0.0f) {
    add_result_p(P_STR_FLTT, s_base[P_GYRO_FILTER] * FLT_MUL);
    add_result_p(P_STR_FLTD, s_base[P_GYRO_FILTER] * FLT_MUL);
  }
}

static void compute_speed_locked(void) {
  if (s_spd_count == 0 || s_spd_spd_sum <= 0.0) {
    fail_locked("no speed data");
    return;
  }
  float cruise_speed = (float)(s_spd_spd_sum / s_spd_count);
  float cruise_thr = (float)(s_spd_thr_sum / s_spd_count) * 100.0f * SPD_FFRATIO;
  float ff_equiv = (float)(s_spd_thr_sum / s_spd_spd_sum) * SPD_FFRATIO;
  tune_log("SPEED done: %u samples, mean %.2f m/s at %.0f%% throttle",
           s_spd_count, cruise_speed, cruise_thr);
  add_result_p(P_CRUISE_SPEED, cruise_speed);
  add_result_p(P_CRUISE_THROTTLE, cruise_thr);
  add_result_p(P_SPD_P, ff_equiv * SPD_P_RATIO);
  add_result_p(P_SPD_I, ff_equiv * SPD_I_RATIO);
}

/* ---- public ------------------------------------------------------------- */

esp_err_t db_fc_tune_start(uint8_t axes) {
  ensure_mutex();
  if (s_mutex == NULL) return ESP_ERR_NO_MEM;
  if (axes == 0) axes = DB_FC_TUNE_AXIS_STEERING | DB_FC_TUNE_AXIS_SPEED;

  db_fc_params_status_t ps;
  db_fc_params_get_status(&ps);
  if (ps.state == DB_FC_PARAM_DUMPING || ps.state == DB_FC_PARAM_LOADING) {
    return ESP_ERR_INVALID_STATE;   /* the parameter link is busy */
  }

  xSemaphoreTake(s_mutex, portMAX_DELAY);
  if (s_state != DB_FC_TUNE_IDLE && s_state != DB_FC_TUNE_COMPLETE &&
      s_state != DB_FC_TUNE_FAILED) {
    xSemaphoreGive(s_mutex);
    return ESP_ERR_INVALID_STATE;
  }
  s_axes = axes;
  s_error[0] = '\0';
  s_result_count = 0;
  s_live_valid = false;
  memset(s_base_got, 0, sizeof(s_base_got));
  memset(s_base, 0, sizeof(s_base));
  memset(s_base_type, 0, sizeof(s_base_type));
  s_state = DB_FC_TUNE_BASELINE;
  s_phase_start_ms = now_ms();
  s_last_stall_warn_ms = 0;   /* a fresh run should warn immediately if stuck */
  s_base_last_req_ms = 0;
  tune_log("=== tune started (axes: %s%s) ===",
           (axes & DB_FC_TUNE_AXIS_STEERING) ? "steering " : "",
           (axes & DB_FC_TUNE_AXIS_SPEED) ? "speed" : "");
  tune_log("reading %d current values from the FC", P_COUNT);
  xSemaphoreGive(s_mutex);
  return ESP_OK;
}

esp_err_t db_fc_tune_apply(void) {
  ensure_mutex();
  if (s_mutex == NULL) return ESP_ERR_NO_MEM;
  xSemaphoreTake(s_mutex, portMAX_DELAY);
  if (s_state != DB_FC_TUNE_READY || s_result_count == 0) {
    xSemaphoreGive(s_mutex);
    return ESP_ERR_INVALID_STATE;
  }
  db_fc_param_entry_t entries[DB_FC_TUNE_MAX_RESULTS];
  memset(entries, 0, sizeof(entries));
  /* Both ids are fixed 17-byte arrays and add_result() always NUL-terminates,
   * so a straight copy is exact - and unlike snprintf it gives the compiler a
   * bound it can prove, which -Werror=format-truncation requires here. */
  _Static_assert(sizeof(entries[0].id) == sizeof(s_results[0].id),
                 "parameter id buffers must be the same size");
  for (uint8_t i = 0; i < s_result_count; i++) {
    memcpy(entries[i].id, s_results[i].id, sizeof(entries[i].id));
    entries[i].value = s_results[i].after;
    entries[i].type = s_results[i].type;
    entries[i].type = MAV_PARAM_TYPE_REAL32;
  }
  uint8_t n = s_result_count;
  s_state = DB_FC_TUNE_WRITING;
  tune_log("applying %u parameters to the FC (each one echo-verified)", n);
  xSemaphoreGive(s_mutex);

  esp_err_t err = db_fc_params_start_load(entries, n);
  if (err != ESP_OK) {
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    fail_locked("could not start the parameter write");
    xSemaphoreGive(s_mutex);
  }
  return err;
}

void db_fc_tune_abort(const char *why) {
  ensure_mutex();
  if (s_mutex == NULL) return;
  xSemaphoreTake(s_mutex, portMAX_DELAY);
  if (s_state != DB_FC_TUNE_IDLE) {
    tune_log("aborted: %s (nothing written)", why ? why : "operator");
    s_state = DB_FC_TUNE_IDLE;
    s_result_count = 0;
    s_error[0] = '\0';
  }
  xSemaphoreGive(s_mutex);
}

void db_fc_tune_get_status(db_fc_tune_status_t *out) {
  if (out == NULL) return;
  memset(out, 0, sizeof(*out));
  ensure_mutex();
  if (s_mutex == NULL) return;
  xSemaphoreTake(s_mutex, portMAX_DELAY);
  out->state = s_state;
  out->axes = s_axes;
  out->steer_samples = s_str_count;
  out->speed_samples = s_spd_count;
  out->steer_rotation_deg = (float)(s_str_rotation_rad * 57.2957795);
  uint16_t sp = s_str_count * 100 / REQUIRED_SAMPLES;
  uint16_t rp = (uint16_t)(s_str_rotation_rad * 100.0 / REQUIRED_ROTATION_RAD);
  out->steer_pct = (uint8_t)((sp < rp ? sp : rp) > 100 ? 100 : (sp < rp ? sp : rp));
  uint16_t vp = s_spd_count * 100 / REQUIRED_SAMPLES;
  out->speed_pct = (uint8_t)(vp > 100 ? 100 : vp);
  out->live_steering = s_live_steer;
  out->live_turnrate_dps = s_live_rate_dps;
  out->live_throttle = s_live_thr;
  out->live_speed_mps = s_live_speed;
  out->live_valid = s_live_valid;
  out->result_count = s_result_count;
  memcpy(out->results, s_results, sizeof(s_results));
  snprintf(out->error, sizeof(out->error), "%s", s_error);
  xSemaphoreGive(s_mutex);
}

void db_fc_tune_on_param_value(const char *param_id, float value,
                               uint8_t param_type) {
  if (param_id == NULL || s_state != DB_FC_TUNE_BASELINE) return;
  ensure_mutex();
  if (s_mutex == NULL) return;
  xSemaphoreTake(s_mutex, portMAX_DELAY);
  if (s_state == DB_FC_TUNE_BASELINE) {
    for (int i = 0; i < P_COUNT; i++) {
      if (!s_base_got[i] && strcmp(param_id, BASELINE_NAMES[i]) == 0) {
        s_base[i] = value;
        s_base_type[i] = param_type;
        s_base_got[i] = true;
        break;
      }
    }
  }
  xSemaphoreGive(s_mutex);
}

void db_fc_tune_tick(void) {
  if (s_state == DB_FC_TUNE_IDLE || s_state == DB_FC_TUNE_COMPLETE ||
      s_state == DB_FC_TUNE_FAILED || s_state == DB_FC_TUNE_READY) {
    return;
  }
  ensure_mutex();
  if (s_mutex == NULL) return;
  xSemaphoreTake(s_mutex, portMAX_DELAY);
  uint32_t t = now_ms();

  if (s_state != DB_FC_TUNE_WRITING && t - s_phase_start_ms > OVERALL_TIMEOUT_MS) {
    fail_locked("timed out waiting for usable data");
    xSemaphoreGive(s_mutex);
    return;
  }

  switch (s_state) {
    case DB_FC_TUNE_BASELINE: {
      int missing = 0;
      for (int i = 0; i < P_COUNT; i++) if (!s_base_got[i]) missing++;
      if (missing == 0) {
        tune_log("baseline read OK (servo range L %.0f/%.0f/%.0f  R %.0f/%.0f/%.0f)",
                 s_base[P_S1_MIN], s_base[P_S1_TRIM], s_base[P_S1_MAX],
                 s_base[P_S2_MIN], s_base[P_S2_TRIM], s_base[P_S2_MAX]);
        if (s_axes & DB_FC_TUNE_AXIS_STEERING) begin_steering_locked();
        else begin_speed_locked();
        break;
      }
      if (t - s_phase_start_ms > BASELINE_TIMEOUT_MS) {
        char why[96];
        snprintf(why, sizeof(why), "FC did not return %d of %d parameters",
                 missing, P_COUNT);
        fail_locked(why);
        break;
      }
      if (t - s_base_last_req_ms > BASELINE_RETRY_MS) {
        s_base_last_req_ms = t;
        for (int i = 0; i < P_COUNT; i++) {
          if (!s_base_got[i]) send_request_read_name(BASELINE_NAMES[i]);
        }
      }
      break;
    }

    case DB_FC_TUNE_STEERING: {
      float st, th, rate, spd;
      if (!sample_now(&st, &th, &rate, &spd)) {
        s_live_valid = false;
        break;
      }
      s_live_valid = true;
      s_live_steer = st;
      s_live_rate_dps = rate * 57.2957795f;
      s_live_thr = th;
      s_live_speed = spd;
      if (fabsf(st) >= STR_STEERING_MIN && fabsf(rate) > STR_TURNRATE_MIN) {
        s_str_steer_sum += fabsf(st);
        s_str_rate_sum += fabsf(rate);
        s_str_rotation_rad += fabsf(rate) / (double)TICK_HZ;
        s_str_count++;
        s_last_qualify_ms = t;
        if (s_str_count % (2 * TICK_HZ) == 0) {
          tune_log("  steering %u/%u samples, %.0f/%.0f deg", s_str_count,
                   REQUIRED_SAMPLES, s_str_rotation_rad * 57.2958,
                   REQUIRED_ROTATION_RAD * 57.2958);
        }
      } else if (t - s_last_stall_warn_ms > STALL_WARN_MS) {
        s_last_stall_warn_ms = t;
        if (fabsf(st) < STR_STEERING_MIN) {
          tune_log("  waiting: more steering (%.2f < %.2f)", fabsf(st),
                   STR_STEERING_MIN);
        } else {
          tune_log("  waiting: faster turn (%.0f < 10 deg/s)",
                   fabsf(rate) * 57.2958);
        }
      }
      if (s_str_count >= REQUIRED_SAMPLES &&
          s_str_rotation_rad >= REQUIRED_ROTATION_RAD) {
        compute_steering_locked();
        if (s_state == DB_FC_TUNE_FAILED) break;
        if (s_axes & DB_FC_TUNE_AXIS_SPEED) begin_speed_locked();
        else finish_collection_locked();
      }
      break;
    }

    case DB_FC_TUNE_SPEED: {
      float st, th, rate, spd;
      if (!sample_now(&st, &th, &rate, &spd)) {
        s_live_valid = false;
        break;
      }
      s_live_valid = true;
      s_live_steer = st;
      s_live_rate_dps = rate * 57.2957795f;
      s_live_thr = th;
      s_live_speed = spd;
      if (th >= SPD_THROTTLE_MIN && spd > SPD_SPEED_MIN) {
        s_spd_thr_sum += th;
        s_spd_spd_sum += spd;
        s_spd_count++;
        s_last_qualify_ms = t;
        if (s_spd_count % (2 * TICK_HZ) == 0) {
          tune_log("  speed %u/%u samples, %.2f m/s at %.0f%%", s_spd_count,
                   REQUIRED_SAMPLES, spd, th * 100);
        }
      } else if (t - s_last_stall_warn_ms > STALL_WARN_MS) {
        s_last_stall_warn_ms = t;
        if (th < SPD_THROTTLE_MIN) {
          tune_log("  waiting: more throttle (%.0f%% < %.0f%%)", th * 100,
                   SPD_THROTTLE_MIN * 100);
        } else {
          tune_log("  waiting: more speed (%.2f < %.2f m/s)", spd, SPD_SPEED_MIN);
        }
      }
      if (s_spd_count >= REQUIRED_SAMPLES) {
        compute_speed_locked();
        if (s_state != DB_FC_TUNE_FAILED) finish_collection_locked();
      }
      break;
    }

    case DB_FC_TUNE_WRITING: {
      db_fc_params_status_t ps;
      db_fc_params_get_status(&ps);
      if (ps.state == DB_FC_PARAM_COMPLETE) {
        /* Mark each result from what actually happened. Setting them all true
         * put a green tick beside every row while the header said one had
         * failed - the operator could not tell which. */
        char failed_names[256];
        db_fc_params_failed_list(failed_names, sizeof(failed_names));
        for (uint8_t i = 0; i < s_result_count; i++) {
          s_results[i].applied =
              !name_in_nl_list(failed_names, s_results[i].id);
        }
        s_state = DB_FC_TUNE_COMPLETE;
        tune_log("=== applied: %u written, %u failed ===", ps.written, ps.failed);
        if (ps.failed) {
          /* %.60s, not %s: s_error is 96 bytes and failed_names is 256, which
           * -Werror=format-truncation rightly refuses. 10 + 16 + 60 always fits. */
          snprintf(s_error, sizeof(s_error), "%u did not take: %.60s", ps.failed,
                   failed_names[0] ? failed_names : "(names unavailable)");
          for (char *c = s_error; *c; c++) if (*c == '\n') *c = ' ';
        }
      } else if (ps.state == DB_FC_PARAM_FAILED) {
        fail_locked(ps.error[0] ? ps.error : "parameter write failed");
      }
      break;
    }

    default:
      break;
  }
  xSemaphoreGive(s_mutex);
}
