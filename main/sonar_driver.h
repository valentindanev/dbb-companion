#ifndef SONAR_DRIVER_H
#define SONAR_DRIVER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * Hardwired-sonar model abstraction.
 *
 * danevi_sonar.c owns everything that is NOT sensor-specific: the UART task,
 * the depth store, the staleness and zero-run filter, the debug ring and every
 * db_sonar_log_* call. A sensor model contributes only the small set of facts
 * below, so adding a third transducer is one file plus one registry row - no
 * second copy of the filter to drift out of sync.
 *
 * Only ONE model runs per boot. The two sensors share UART2 and the same TX/RX
 * pins, so they are mutually exclusive by construction; `ss_type` selects which
 * descriptor main.c hands to danevi_sonar_init(). Owner decision 29-08-2026:
 * the boat never carries both at once.
 */

/* Persisted in NVS as `ss_type` - values are permanent, append only. */
typedef enum {
  SONAR_MODEL_DYP_L041MTW = 0, /* the original hardwired transducer */
  SONAR_MODEL_AJ_SR04M = 1,    /* AJ-SR04M / JSN-SR04T, UART-triggered mode */
  SONAR_MODEL_COUNT
} sonar_model_t;

typedef struct {
  const char *name;         /* short id, appears in SYSTEM.LOG as ss_type= */
  const char *display_name; /* human label for the web UI / debug panel */

  uint32_t baud_rate;
  uint8_t trigger_byte; /* 0 = listen-only: send nothing, just read */
  uint8_t frame_size;   /* bytes to read per reply */

  /* How long to wait for a reply before calling it a timeout. Must cover the
   * sensor's own no-echo timeout, not just the frame's transmission time. */
  uint32_t response_timeout_ms;
  /* Delay after each read attempt. Cycle = trigger + reply + this. */
  uint32_t trigger_interval_ms;
  /* How long a good reading may be held while zeros stream in. Scale this with
   * trigger_interval_ms or the hold silently changes meaning. */
  uint32_t zero_hold_grace_ms;
  /* Age at which a good reading stops being published at all. */
  uint32_t distance_stale_ms;

  /* Decode one frame. Return false if it is not a valid frame of this
   * protocol (bad header, bad checksum, wrong length). `out_mm` receives the
   * sensor's own reading, before any scaling. A valid frame reporting 0 mm is
   * a successful decode of "no echo" - return true with *out_mm = 0. */
  bool (*parse)(const uint8_t *frame, int length, int *out_mm);

  /* Convert the sensor's raw reading into millimetres of water. NULL when the
   * sensor already reports water millimetres. Must map 0 to 0 so the
   * no-echo signal survives. */
  int (*scale_mm)(int raw_mm);
} sonar_driver_t;

/* Returns NULL for an unknown model. */
const sonar_driver_t *sonar_driver_get(sonar_model_t model);

/* Short id for logging; "unknown" when the model is out of range. */
const char *sonar_driver_name(sonar_model_t model);

extern const sonar_driver_t sonar_driver_dyp_l041mtw;
extern const sonar_driver_t sonar_driver_aj_sr04m;

#endif // SONAR_DRIVER_H
