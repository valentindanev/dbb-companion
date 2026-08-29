#include "db_fc_params.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "db_serial.h"
#include "db_mavlink_msgs.h"
#include "common/mavlink.h"

#define TAG "DB_FCPARAM"

/* ArduPilot's default identity. The boat has exactly one autopilot on this
 * link, and every message we have ever seen from it uses 1/1. */
#define FC_SYSID 1
#define FC_COMPID 1

#define DUMP_REQUEST_TIMEOUT_MS 4000   /* silence before we re-ask */
#define DUMP_STALL_LIMIT 6             /* re-asks before giving up */
#define LOAD_ACK_TIMEOUT_MS 700
#define LOAD_RETRY_LIMIT 3

static SemaphoreHandle_t s_mutex;
static db_fc_param_entry_t *s_entries;
static uint16_t s_capacity;
static uint16_t s_count;          /* dump: distinct received. load: list length */
static uint16_t s_expected;
static db_fc_param_state_t s_state = DB_FC_PARAM_IDLE;
static char s_error[96];

/* load bookkeeping */
static uint16_t s_load_index;
static uint16_t s_written;
static uint8_t s_retries;
static uint32_t s_last_activity_ms;
static uint8_t s_stalls;
static char *s_failed;            /* newline separated names */
static bool s_store_pending;      /* dump finished; tick() writes the backup */
static size_t s_failed_len;

static uint32_t now_ms(void) { return (uint32_t)(esp_timer_get_time() / 1000ULL); }

static void ensure_mutex(void) {
  if (s_mutex == NULL) s_mutex = xSemaphoreCreateMutex();
}

static void free_all_locked(void) {
  free(s_entries);
  s_entries = NULL;
  free(s_failed);
  s_failed = NULL;
  s_failed_len = 0;
  s_capacity = s_count = s_expected = 0;
  s_load_index = s_written = 0;
  s_retries = s_stalls = 0;
  s_store_pending = false;
}

static void fail_locked(const char *why) {
  s_state = DB_FC_PARAM_FAILED;
  snprintf(s_error, sizeof(s_error), "%s", why == NULL ? "failed" : why);
  ESP_LOGW(TAG, "%s", s_error);
}

/* ---------------------------------------------------------------- transmit */

static void send_request_list(void) {
  uint8_t buf[296];
  fmav_status_t status = {0};
  fmav_param_request_list_t payload = {
      .target_system = FC_SYSID, .target_component = FC_COMPID};
  uint16_t len = fmav_msg_param_request_list_encode_to_frame_buf(
      buf, db_get_mav_sys_id(), db_get_mav_comp_id(), &payload, &status);
  if (len) write_to_serial(buf, len);
}

static void send_request_read(uint16_t index) {
  uint8_t buf[296];
  fmav_status_t status = {0};
  fmav_param_request_read_t payload = {
      .param_index = (int16_t)index,
      .target_system = FC_SYSID,
      .target_component = FC_COMPID};
  memset(payload.param_id, 0, sizeof(payload.param_id));
  uint16_t len = fmav_msg_param_request_read_encode_to_frame_buf(
      buf, db_get_mav_sys_id(), db_get_mav_comp_id(), &payload, &status);
  if (len) write_to_serial(buf, len);
}

static void send_param_set(const db_fc_param_entry_t *e) {
  uint8_t buf[296];
  fmav_status_t status = {0};
  fmav_param_set_t payload = {
      .param_value = e->value,
      .target_system = FC_SYSID,
      .target_component = FC_COMPID,
      .param_type = e->type ? e->type : MAV_PARAM_TYPE_REAL32};
  memset(payload.param_id, 0, sizeof(payload.param_id));
  strncpy(payload.param_id, e->id, sizeof(payload.param_id));
  uint16_t len = fmav_msg_param_set_encode_to_frame_buf(
      buf, db_get_mav_sys_id(), db_get_mav_comp_id(), &payload, &status);
  if (len) write_to_serial(buf, len);
}

/* ------------------------------------------------------------------- public */

esp_err_t db_fc_params_start_dump(void) {
  ensure_mutex();
  if (s_mutex == NULL) return ESP_ERR_NO_MEM;
  xSemaphoreTake(s_mutex, portMAX_DELAY);
  if (s_state == DB_FC_PARAM_DUMPING || s_state == DB_FC_PARAM_LOADING) {
    xSemaphoreGive(s_mutex);
    return ESP_ERR_INVALID_STATE;
  }
  free_all_locked();
  s_entries = calloc(DB_FC_PARAM_MAX, sizeof(*s_entries));
  if (s_entries == NULL) {
    fail_locked("out of memory for the parameter table");
    xSemaphoreGive(s_mutex);
    return ESP_ERR_NO_MEM;
  }
  s_capacity = DB_FC_PARAM_MAX;
  s_state = DB_FC_PARAM_DUMPING;
  s_error[0] = '\0';
  s_last_activity_ms = now_ms();
  xSemaphoreGive(s_mutex);
  send_request_list();
  ESP_LOGI(TAG, "parameter dump started");
  return ESP_OK;
}

esp_err_t db_fc_params_start_load(const db_fc_param_entry_t *entries, uint16_t count) {
  if (entries == NULL || count == 0) return ESP_ERR_INVALID_ARG;
  ensure_mutex();
  if (s_mutex == NULL) return ESP_ERR_NO_MEM;
  xSemaphoreTake(s_mutex, portMAX_DELAY);
  if (s_state == DB_FC_PARAM_DUMPING || s_state == DB_FC_PARAM_LOADING) {
    xSemaphoreGive(s_mutex);
    return ESP_ERR_INVALID_STATE;
  }
  free_all_locked();
  s_entries = calloc(count, sizeof(*s_entries));
  if (s_entries == NULL) {
    fail_locked("out of memory for the parameter list");
    xSemaphoreGive(s_mutex);
    return ESP_ERR_NO_MEM;
  }
  memcpy(s_entries, entries, (size_t)count * sizeof(*s_entries));
  for (uint16_t i = 0; i < count; ++i) s_entries[i].present = false;
  s_capacity = s_count = count;
  s_state = DB_FC_PARAM_LOADING;
  s_error[0] = '\0';
  s_load_index = s_written = 0;
  s_retries = 0;
  s_last_activity_ms = 0;   /* forces the first send on the next tick */
  xSemaphoreGive(s_mutex);
  ESP_LOGI(TAG, "parameter load started: %u entries", (unsigned)count);
  return ESP_OK;
}

void db_fc_params_abort(void) {
  ensure_mutex();
  if (s_mutex == NULL) return;
  xSemaphoreTake(s_mutex, portMAX_DELAY);
  free_all_locked();
  s_state = DB_FC_PARAM_IDLE;
  s_error[0] = '\0';
  xSemaphoreGive(s_mutex);
}

void db_fc_params_get_status(db_fc_params_status_t *out) {
  if (out == NULL) return;
  memset(out, 0, sizeof(*out));
  ensure_mutex();
  if (s_mutex == NULL) return;
  xSemaphoreTake(s_mutex, portMAX_DELAY);
  out->state = s_state;
  out->expected = s_expected;
  out->received = (s_state == DB_FC_PARAM_LOADING) ? 0 : s_count;
  out->total = (s_state == DB_FC_PARAM_LOADING ||
                (s_state != DB_FC_PARAM_DUMPING && s_written)) ? s_count : s_expected;
  out->written = s_written;
  out->failed = (uint16_t)(s_failed_len ? 0 : 0);
  /* count failures by counting newlines */
  if (s_failed != NULL) {
    uint16_t n = 0;
    for (size_t i = 0; i < s_failed_len; ++i)
      if (s_failed[i] == '\n') n++;
    out->failed = n;
  }
  snprintf(out->error, sizeof(out->error), "%s", s_error);
  xSemaphoreGive(s_mutex);
}

size_t db_fc_params_failed_list(char *buf, size_t buf_size) {
  if (buf == NULL || buf_size == 0) return 0;
  ensure_mutex();
  if (s_mutex == NULL) return 0;
  xSemaphoreTake(s_mutex, portMAX_DELAY);
  size_t n = 0;
  if (s_failed != NULL && s_failed_len) {
    n = s_failed_len < buf_size - 1 ? s_failed_len : buf_size - 1;
    memcpy(buf, s_failed, n);
  }
  buf[n] = '\0';
  xSemaphoreGive(s_mutex);
  return n;
}

static void record_failure_locked(const char *id) {
  size_t add = strlen(id) + 1;
  char *grown = realloc(s_failed, s_failed_len + add + 1);
  if (grown == NULL) return;   /* losing one name is better than crashing */
  s_failed = grown;
  memcpy(s_failed + s_failed_len, id, add - 1);
  s_failed[s_failed_len + add - 1] = '\n';
  s_failed_len += add;
  s_failed[s_failed_len] = '\0';
}

void db_fc_params_on_param_value(const char *param_id, float value, uint8_t type,
                                 uint16_t param_index, uint16_t param_count) {
  ensure_mutex();
  if (s_mutex == NULL || param_id == NULL) return;
  xSemaphoreTake(s_mutex, portMAX_DELAY);

  if (s_state == DB_FC_PARAM_DUMPING && s_entries != NULL) {
    if (param_count && param_count <= s_capacity) s_expected = param_count;
    if (param_index < s_capacity) {
      db_fc_param_entry_t *e = &s_entries[param_index];
      if (!e->present) s_count++;
      e->present = true;
      e->value = value;
      e->type = type;
      memset(e->id, 0, sizeof(e->id));
      strncpy(e->id, param_id, DB_FC_PARAM_ID_LEN);
      s_last_activity_ms = now_ms();
      s_stalls = 0;
      if (s_expected && s_count >= s_expected) {
        s_state = DB_FC_PARAM_COMPLETE;
        /* Writing the backup here would put FAT I/O on the MAVLink parse task,
         * which also feeds depth publication. Hand it to the timer instead. */
        s_store_pending = true;
        ESP_LOGI(TAG, "parameter dump complete: %u", (unsigned)s_count);
      }
    }
  } else if (s_state == DB_FC_PARAM_LOADING && s_entries != NULL &&
             s_load_index < s_count) {
    db_fc_param_entry_t *want = &s_entries[s_load_index];
    if (strncmp(param_id, want->id, DB_FC_PARAM_ID_LEN) == 0) {
      /* The FC echoes what it actually stored. Trust that, not our request. */
      float diff = fabsf(value - want->value);
      float tol = fabsf(want->value) * 1e-4f;
      if (tol < 1e-6f) tol = 1e-6f;
      if (diff <= tol) {
        want->present = true;
        s_written++;
      } else {
        record_failure_locked(want->id);
      }
      s_load_index++;
      s_retries = 0;
      s_last_activity_ms = 0;   /* send the next one immediately */
      if (s_load_index >= s_count) {
        s_state = DB_FC_PARAM_COMPLETE;
        ESP_LOGI(TAG, "parameter load complete: %u/%u written",
                 (unsigned)s_written, (unsigned)s_count);
      }
    }
  }
  xSemaphoreGive(s_mutex);
}

void db_fc_params_tick(void) {
  ensure_mutex();
  if (s_mutex == NULL) return;
  xSemaphoreTake(s_mutex, portMAX_DELAY);
  bool store_now = s_store_pending;
  s_store_pending = false;
  xSemaphoreGive(s_mutex);
  if (store_now) {
    /* db_fc_params_store() takes the mutex itself. */
    (void)db_fc_params_store();
  }
  xSemaphoreTake(s_mutex, portMAX_DELAY);
  uint32_t now = now_ms();

  if (s_state == DB_FC_PARAM_DUMPING) {
    if (now - s_last_activity_ms > DUMP_REQUEST_TIMEOUT_MS) {
      if (++s_stalls > DUMP_STALL_LIMIT) {
        fail_locked(s_count ? "flight controller stopped sending parameters"
                            : "no reply from the flight controller");
        xSemaphoreGive(s_mutex);
        return;
      }
      s_last_activity_ms = now;
      /* Re-ask for the first hole rather than the whole list again: on a slow
       * link a second full stream would take longer than the gap it fixes. */
      uint16_t hole = UINT16_MAX;
      if (s_expected) {
        for (uint16_t i = 0; i < s_expected && i < s_capacity; ++i) {
          if (!s_entries[i].present) { hole = i; break; }
        }
      }
      xSemaphoreGive(s_mutex);
      if (hole == UINT16_MAX) send_request_list();
      else send_request_read(hole);
      return;
    }
  } else if (s_state == DB_FC_PARAM_LOADING && s_entries != NULL) {
    if (s_load_index >= s_count) {
      s_state = DB_FC_PARAM_COMPLETE;
      xSemaphoreGive(s_mutex);
      return;
    }
    if (now - s_last_activity_ms > LOAD_ACK_TIMEOUT_MS) {
      if (s_retries >= LOAD_RETRY_LIMIT) {
        record_failure_locked(s_entries[s_load_index].id);
        s_load_index++;
        s_retries = 0;
        if (s_load_index >= s_count) {
          s_state = DB_FC_PARAM_COMPLETE;
          xSemaphoreGive(s_mutex);
          return;
        }
      } else {
        s_retries++;
      }
      s_last_activity_ms = now;
      db_fc_param_entry_t snapshot = s_entries[s_load_index];
      xSemaphoreGive(s_mutex);
      send_param_set(&snapshot);
      return;
    }
  }
  xSemaphoreGive(s_mutex);
}

size_t db_fc_params_render(char *buf, size_t buf_size, size_t offset, bool *done) {
  if (buf == NULL || buf_size == 0) return 0;
  ensure_mutex();
  if (s_mutex == NULL) return 0;
  xSemaphoreTake(s_mutex, portMAX_DELAY);
  size_t written = 0;
  size_t seen = 0;
  bool finished = true;
  if (s_entries != NULL) {
    for (uint16_t i = 0; i < s_capacity; ++i) {
      if (!s_entries[i].present) continue;
      char line[48];
      int n = snprintf(line, sizeof(line), "%s,%.6f\n", s_entries[i].id,
                       (double)s_entries[i].value);
      if (n <= 0) continue;
      if (seen + (size_t)n <= offset) { seen += (size_t)n; continue; }
      size_t start = offset > seen ? offset - seen : 0;
      size_t avail = (size_t)n - start;
      if (avail > buf_size - written) {
        memcpy(buf + written, line + start, buf_size - written);
        written = buf_size;
        finished = false;
        break;
      }
      memcpy(buf + written, line + start, avail);
      written += avail;
      seen += (size_t)n;
    }
  }
  if (done != NULL) *done = finished;
  xSemaphoreGive(s_mutex);
  return written;
}


/* ------------------------------------------------- backup stored on the boat */

void db_fc_params_stored_info(bool *present, size_t *bytes, uint16_t *count) {
  if (present) *present = false;
  if (bytes) *bytes = 0;
  if (count) *count = 0;
  FILE *f = fopen(DB_FC_PARAM_FILE_PATH, "r");
  if (f == NULL) return;
  uint16_t n = 0;
  size_t total = 0;
  char line[64];
  while (fgets(line, sizeof(line), f) != NULL) {
    total += strlen(line);
    if (line[0] != '#' && line[0] != '\n' && line[0] != '\r') n++;
  }
  fclose(f);
  if (present) *present = true;
  if (bytes) *bytes = total;
  if (count) *count = n;
}

esp_err_t db_fc_params_store(void) {
  ensure_mutex();
  if (s_mutex == NULL) return ESP_ERR_NO_MEM;
  xSemaphoreTake(s_mutex, portMAX_DELAY);
  if (s_entries == NULL || s_count == 0) {
    xSemaphoreGive(s_mutex);
    return ESP_ERR_INVALID_STATE;
  }
  /* Write to a temporary first: a power cut mid-write must not leave a
   * half-written backup where a complete one used to be. */
  FILE *f = fopen(DB_FC_PARAM_TMP_PATH, "w");
  if (f == NULL) {
    xSemaphoreGive(s_mutex);
    ESP_LOGE(TAG, "cannot open the parameter backup for writing");
    return ESP_FAIL;
  }
  fprintf(f, "# DBB FC parameter backup\n");
  uint16_t written = 0;
  for (uint16_t i = 0; i < s_capacity; ++i) {
    if (!s_entries[i].present) continue;
    if (fprintf(f, "%s,%.6f\n", s_entries[i].id, (double)s_entries[i].value) < 0) {
      fclose(f);
      remove(DB_FC_PARAM_TMP_PATH);
      xSemaphoreGive(s_mutex);
      ESP_LOGE(TAG, "write failed while storing the parameter backup");
      return ESP_FAIL;
    }
    written++;
  }
  fclose(f);
  remove(DB_FC_PARAM_FILE_PATH);
  if (rename(DB_FC_PARAM_TMP_PATH, DB_FC_PARAM_FILE_PATH) != 0) {
    xSemaphoreGive(s_mutex);
    ESP_LOGE(TAG, "could not put the parameter backup in place");
    return ESP_FAIL;
  }
  xSemaphoreGive(s_mutex);
  ESP_LOGI(TAG, "parameter backup stored: %u parameters", (unsigned)written);
  return ESP_OK;
}

esp_err_t db_fc_params_store_entries(const db_fc_param_entry_t *entries,
                                     uint16_t count) {
  if (entries == NULL || count == 0) return ESP_ERR_INVALID_ARG;
  FILE *f = fopen(DB_FC_PARAM_TMP_PATH, "w");
  if (f == NULL) return ESP_FAIL;
  fprintf(f, "# DBB FC parameter backup\n");
  for (uint16_t i = 0; i < count; ++i) {
    if (fprintf(f, "%s,%.6f\n", entries[i].id, (double)entries[i].value) < 0) {
      fclose(f);
      remove(DB_FC_PARAM_TMP_PATH);
      return ESP_FAIL;
    }
  }
  fclose(f);
  remove(DB_FC_PARAM_FILE_PATH);
  return rename(DB_FC_PARAM_TMP_PATH, DB_FC_PARAM_FILE_PATH) == 0
             ? ESP_OK : ESP_FAIL;
}

esp_err_t db_fc_params_delete_stored(void) {
  return remove(DB_FC_PARAM_FILE_PATH) == 0 ? ESP_OK : ESP_ERR_NOT_FOUND;
}

esp_err_t db_fc_params_start_load_from_stored(void) {
  FILE *f = fopen(DB_FC_PARAM_FILE_PATH, "r");
  if (f == NULL) return ESP_ERR_NOT_FOUND;
  db_fc_param_entry_t *list = calloc(DB_FC_PARAM_MAX, sizeof(*list));
  if (list == NULL) {
    fclose(f);
    return ESP_ERR_NO_MEM;
  }
  uint16_t n = 0;
  char line[64];
  while (fgets(line, sizeof(line), f) != NULL && n < DB_FC_PARAM_MAX) {
    if (line[0] == '#' || line[0] == '\n' || line[0] == '\r') continue;
    char *comma = strchr(line, ',');
    if (comma == NULL) continue;
    *comma = '\0';
    char *endp = NULL;
    float value = strtof(comma + 1, &endp);
    if (endp == comma + 1) continue;          /* no number after the comma */
    size_t len = strlen(line);
    if (len == 0 || len > DB_FC_PARAM_ID_LEN) continue;
    memset(list[n].id, 0, sizeof(list[n].id));
    memcpy(list[n].id, line, len);
    list[n].value = value;
    list[n].type = MAV_PARAM_TYPE_REAL32;
    n++;
  }
  fclose(f);
  if (n == 0) {
    free(list);
    return ESP_ERR_INVALID_SIZE;
  }
  esp_err_t err = db_fc_params_start_load(list, n);
  free(list);
  return err;
}
