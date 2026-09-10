#ifndef DBB_SONAR_COMPARE_TEST_H
#define DBB_SONAR_COMPARE_TEST_H

#include <stdbool.h>
#include <stdint.h>

#include "danevi_sonar.h"

void sonar_compare_test_init(bool deeper_connected, int output_source);
void sonar_compare_test_register_http(void *server);
void sonar_compare_test_note_aj(bool valid, int raw_depth_mm, uint32_t echo_us,
                                bool timeout,
                                const danevi_sonar_snapshot_t *filtered);
void sonar_compare_test_note_deeper(int depth_mm);

#endif
