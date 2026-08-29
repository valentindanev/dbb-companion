#include "sonar_driver.h"

/*
 * DYP-L041MTW-V1.0 / GL041MT - the original hardwired transducer.
 *
 * Every constant here is exactly what danevi_sonar.c used before the driver
 * split, so this model's behaviour is unchanged by the refactor. Do not "tidy"
 * these numbers: they are the configuration this boat has run for months, and
 * the DYP path is the reference the AJ-SR04M is measured against.
 *
 * Protocol (manufacturer datasheet section 3.1, source T1/T4):
 *   host writes any serial byte to trigger; 0x55 is what the ArduPilot
 *   reference converter uses, so that is what we send.
 *   sensor replies FF Data_H Data_L SUM, SUM = (FF + Data_H + Data_L) & 0xFF.
 *   distance = (Data_H << 8) + Data_L, in MILLIMETRES OF WATER already.
 *
 * Timing: minimum trigger period T1 >= 19 ms, typical response T2 ~13 ms.
 * 100 ms of delay gives the ~113 ms cycle on record.
 *
 * A valid frame of FF 00 00 FF means "no echo" - the sensor is alive and heard
 * nothing. That is the dominant reading over silt (98.0 % on 28-08-2026), and
 * it must reach the zero-run filter as a real 0, not as a parse failure.
 */

static bool dyp_parse(const uint8_t *frame, int length, int *out_mm) {
  if (frame == NULL || out_mm == NULL || length != 4 || frame[0] != 0xFF) {
    return false;
  }
  uint8_t sum = (uint8_t)((frame[0] + frame[1] + frame[2]) & 0xFF);
  if (sum != frame[3]) {
    return false;
  }
  *out_mm = (frame[1] << 8) + frame[2];
  return true;
}

const sonar_driver_t sonar_driver_dyp_l041mtw = {
    .name = "dyp-l041mtw",
    .display_name = "DYP-L041MTW (original)",
    .baud_rate = 115200,
    .trigger_byte = 0x55,
    .frame_size = 4,
    .response_timeout_ms = 30,
    .trigger_interval_ms = 100,
    .zero_hold_grace_ms = 600,
    .distance_stale_ms = 3000,
    .parse = dyp_parse,
    .scale_mm = NULL, /* already reports water millimetres */
};
