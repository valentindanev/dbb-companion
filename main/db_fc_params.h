/*
 * FC parameter dump / load over the boat's existing MAVLink serial link.
 *
 * Dump is read-only: PARAM_REQUEST_LIST, collect every PARAM_VALUE, re-request
 * whatever is missing, then serve the result as a Mission-Planner-compatible
 * NAME,VALUE text file.
 *
 * Load WRITES TO THE FLIGHT CONTROLLER. Every write is verified against the
 * PARAM_VALUE the FC echoes back; a parameter that does not read back with the
 * value we asked for is reported as failed rather than silently assumed good.
 */
#ifndef DB_FC_PARAMS_H
#define DB_FC_PARAMS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

/*
 * Where the parameter backup lives, beside the FC firmware image in the same
 * FAT mount, so the recovery path is entirely boat-side: dump -> reflash the FC
 * -> restore, with no file needed on a phone at the lake.
 *
 * *** 8.3 NAME *** the build sets CONFIG_FATFS_LFN_NONE, so at most 8 characters
 * plus a 3-character extension. "fcparam.prm" fits; anything longer silently
 * fails to open. Same trap documented in db_fc_flash.h.
 */
#define DB_FC_PARAM_FILE_PATH "/logs/fcparam.prm"
/* Its own 8.3 name - "fcparam.prm.tmp" would be TWO extensions and fail to open. */
#define DB_FC_PARAM_TMP_PATH  "/logs/fcparam.tmp"

/* Counted against the logs partition alongside the firmware reserve so the
 * backup and the session pool cannot starve each other. 1150 params is ~27 KB. */
#define DB_FC_PARAM_RESERVE_BYTES (64u * 1024u)

#define DB_FC_PARAM_ID_LEN 16
/* ArduRover 4.7 exposes roughly 1400. The cap is a memory guard, not a target. */
#define DB_FC_PARAM_MAX 2200

typedef enum {
  DB_FC_PARAM_IDLE = 0,
  DB_FC_PARAM_DUMPING,
  DB_FC_PARAM_LOADING,
  DB_FC_PARAM_COMPLETE,
  DB_FC_PARAM_FAILED,
} db_fc_param_state_t;

typedef struct {
  char id[DB_FC_PARAM_ID_LEN + 1];
  float value;
  uint8_t type;
  bool present;
} db_fc_param_entry_t;

typedef struct {
  db_fc_param_state_t state;
  uint16_t expected;   /* param_count advertised by the FC */
  uint16_t received;   /* distinct parameters collected */
  uint16_t total;      /* load: how many were requested */
  uint16_t written;    /* load: how many verified */
  uint16_t failed;     /* load: how many did not take */
  char error[96];
} db_fc_params_status_t;

/** Begin a full parameter download. Fails if a job is already running. */
esp_err_t db_fc_params_start_dump(void);

/** Begin writing parameters. Takes ownership of nothing; copies the list. */
esp_err_t db_fc_params_start_load(const db_fc_param_entry_t *entries, uint16_t count);

/** Cancel whatever is running and free the buffers. */
void db_fc_params_abort(void);

void db_fc_params_get_status(db_fc_params_status_t *out);

/** Names that did not take, newline separated. Returns bytes written. */
size_t db_fc_params_failed_list(char *buf, size_t buf_size);

/**
 * Feed a PARAM_VALUE that arrived FROM the flight controller.
 * Safe to call for every PARAM_VALUE; ignored when no job is running.
 */
void db_fc_params_on_param_value(const char *param_id, float value, uint8_t type,
                                 uint16_t param_index, uint16_t param_count);

/** Drives retries, timeouts and the load queue. Call from the 100 ms timer. */
void db_fc_params_tick(void);

/**
 * Render the collected parameters as `NAME,VALUE` text.
 * Returns the number of bytes written, or 0 when there is nothing to serve.
 */
size_t db_fc_params_render(char *buf, size_t buf_size, size_t offset, bool *done);

/* ---- the backup stored on the boat ------------------------------------- */

/** Info about the stored backup. Any pointer may be NULL. */
void db_fc_params_stored_info(bool *present, size_t *bytes, uint16_t *count);

/** Write the parameters collected by the last dump to DB_FC_PARAM_FILE_PATH. */
esp_err_t db_fc_params_store(void);

/** Parse the stored backup and begin writing it to the flight controller. */
esp_err_t db_fc_params_start_load_from_stored(void);

/** Replace the stored backup with caller-supplied entries (an upload). */
esp_err_t db_fc_params_store_entries(const db_fc_param_entry_t *entries, uint16_t count);

/** Remove the stored backup. */
esp_err_t db_fc_params_delete_stored(void);

#endif /* DB_FC_PARAMS_H */
