#ifndef DBB_SONAR_COMPARE_CORE_H
#define DBB_SONAR_COMPARE_CORE_H

#include <stdbool.h>
#include <stdint.h>

#define SONAR_COMPARE_PRIMARY_PAIR_WINDOW_MS 250U
#define SONAR_COMPARE_SOURCE_LIVE_MS 3000U
#define SONAR_COMPARE_AGREEMENT_MM 100
#define SONAR_COMPARE_AJ_PULSE_MODEL_ID 2

typedef enum {
  SONAR_COMPARE_IDLE = 0,
  SONAR_COMPARE_READY,
  SONAR_COMPARE_RECORDING,
  SONAR_COMPARE_STOPPED,
  SONAR_COMPARE_INVALID,
} sonar_compare_state_t;

typedef enum {
  SONAR_COMPARE_START_OK = 0,
  SONAR_COMPARE_START_NOT_COMPILED,
  SONAR_COMPARE_START_PREFLIGHT_REQUIRED,
  SONAR_COMPARE_START_UART_NMEA_ACTIVE,
  SONAR_COMPARE_START_AJ_NOT_LIVE,
  SONAR_COMPARE_START_DEEPER_NOT_LIVE,
  SONAR_COMPARE_START_OUTPUT_AUTHORITY_CHANGED,
  SONAR_COMPARE_START_ALREADY_RECORDING,
} sonar_compare_start_result_t;

typedef enum {
  SONAR_COMPARE_PAIR_MISSING_BOTH = 0,
  SONAR_COMPARE_PAIR_MISSING_AJ,
  SONAR_COMPARE_PAIR_MISSING_DEEPER,
  SONAR_COMPARE_PAIR_STALE,
  SONAR_COMPARE_PAIR_REJECTED_AGE,
  SONAR_COMPARE_PAIR_PAIRED,
} sonar_compare_pair_quality_t;

typedef struct {
  bool present;
  bool valid;
  int depth_mm;
  uint32_t boot_ms;
} sonar_compare_observation_t;

typedef struct {
  sonar_compare_pair_quality_t quality;
  uint32_t separation_ms;
  int difference_mm;
  double difference_percent;
  bool agreement;
} sonar_compare_pair_t;

typedef struct {
  bool compiled;
  sonar_compare_state_t state;
  int output_source_at_boot;
  uint32_t session_id;
  sonar_compare_observation_t aj;
  sonar_compare_observation_t deeper;
  uint32_t aj_polls;
  uint32_t aj_zeros;
  uint32_t aj_timeouts;
  uint32_t deeper_samples;
  uint32_t valid_pairs;
  char reason[96];
} sonar_compare_core_t;

void sonar_compare_core_init(sonar_compare_core_t *core, bool compiled,
                             bool deeper_connected, int output_source);
sonar_compare_start_result_t sonar_compare_core_start(
    sonar_compare_core_t *core, bool preflight_complete,
    bool uart_nmea_emitter_active, bool aj_live, bool deeper_live,
    int current_output_source);
void sonar_compare_core_stop(sonar_compare_core_t *core);
void sonar_compare_core_note_aj(sonar_compare_core_t *core, uint32_t boot_ms,
                                bool valid, int depth_mm, bool timeout);
void sonar_compare_core_note_deeper(sonar_compare_core_t *core,
                                    uint32_t boot_ms, bool valid,
                                    int depth_mm);
sonar_compare_pair_t sonar_compare_core_pair(const sonar_compare_core_t *core,
                                             uint32_t now_ms);
bool sonar_compare_output_authority_ok(const sonar_compare_core_t *core,
                                       int current_output_source);
int sonar_compare_required_model_id(int stored_model_id);
const char *sonar_compare_state_name(sonar_compare_state_t state);
const char *sonar_compare_pair_quality_name(sonar_compare_pair_quality_t quality);
const char *sonar_compare_start_result_name(sonar_compare_start_result_t result);

#endif
