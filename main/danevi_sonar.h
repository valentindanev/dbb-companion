#ifndef DANEVI_SONAR_H
#define DANEVI_SONAR_H

#include "sonar_driver.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
  bool has_distance;
  int depth_mm;
  uint32_t sample_age_ms;
  bool has_raw_distance;
  int raw_depth_mm;
  uint32_t raw_sample_age_ms;
  bool has_last_good_distance;
  int last_good_depth_mm;
  uint32_t last_good_sample_age_ms;
  bool zero_run_active;
  uint32_t zero_run_age_ms;
  uint32_t consecutive_zero_frames;
  bool zero_filter_holding_last_good;
} danevi_sonar_snapshot_t;

// Initialize UART2 for the hardwired sonar and start the FreeRTOS task.
// `driver` selects the transducer model (see sonar_driver.h); NULL falls back
// to the original DYP-L041MTW so a bad `ss_type` cannot leave the boat without
// a sounder. Only one model runs per boot - the two share UART2.
void danevi_sonar_init(const sonar_driver_t *driver, int tx_pin, int rx_pin);

// Thread-safe getter for the latest distance
// Returns true if data is fresh/valid, false if no new data or timed out
bool danevi_sonar_get_latest_distance(int *out_distance_mm);

// Thread-safe setter for the deeper sonar distance (if active)
void danevi_sonar_set_distance(int distance_mm);

bool danevi_sonar_get_snapshot(danevi_sonar_snapshot_t *snapshot);
void danevi_sonar_get_debug_log(char *dst, size_t dst_size);

#endif
