#include "sonar_driver.h"

#include "db_parameters.h"

/*
 * AJ-SR04M / JSN-SR04T waterproof ultrasonic module, UART-triggered mode.
 *
 * HARDWARE PRECONDITIONS - the module will NOT work without these:
 *
 *  1. R19 must be fitted with 360 kOhm (UART-triggered binary mode). As shipped
 *     R19 is open and the module speaks pulse/echo on trigger+echo pins, not
 *     serial at all. 120 kOhm selects AUTONOMOUS mode - it then streams frames
 *     every ~100 ms unsolicited, which this driver would mostly miss, because
 *     it triggers and then waits a bounded time for the reply. Fitting 120k
 *     looks exactly like a dead sensor. Verify the value against the module's
 *     own silkscreen: revisions differ.
 *  2. Power 3.0-5.5 V. Use the existing 5 V rail - NOT the 12 V line the
 *     DYP now runs on. 12 V will destroy it.
 *  3. Level-shift its TX into the ESP's 3.3 V RX (20k/10k divider), or run the
 *     module at 3.3 V and wire direct.
 *
 * PROTOCOL - byte-identical to the DYP, which is why this is a near drop-in:
 *   trigger 0x55, reply FF Data_H Data_L SUM, SUM = (FF + Data_H + Data_L) & 0xFF.
 *   The only wire difference is the baud rate: 9600, not 115200.
 *
 * SCALING - this is the part that is genuinely different. The module is a
 * ranging amplifier plus a timer: it measures echo time and converts it with
 * the AIR speed of sound (348 m/s per the JSN-SR04M-2 datasheet). Sound travels
 * ~4.25x faster in water, so the raw reading UNDER-reports depth by that
 * factor and the host must rescale. ArduPilot never does this - its HC-SR04
 * driver hard-codes `(value_us * (1.0/58.0f)) * 0.01f` with no scaling
 * parameter, which is why this sensor belongs on the ESP and not on the FC.
 *
 * The water figure is a parameter (`ss_water_mps`, default 1480 m/s at ~20 C)
 * rather than a #define: the speed of sound in fresh water moves roughly
 * 1450-1500 m/s across a season, about 3 % - a few centimetres at these depths.
 *
 * RESPONSE TIMEOUT - 120 ms, deliberately far above the DYP's 30 ms. At 9600
 * baud the 4-byte frame alone takes ~4.2 ms, but the module's internal no-echo
 * timeout is sized for its 8 m AIR range (~46 ms round trip) and it answers
 * late rather than not at all. Leaving this at 30 ms produces timeouts that
 * look like a dead sensor.
 */

/* Speed of sound the module itself assumes, from its datasheet. */
#define SR04M_ASSUMED_AIR_MPS 348U

static bool sr04m_parse(const uint8_t *frame, int length, int *out_mm) {
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

static int sr04m_scale_mm(int raw_mm) {
  /* 0 means "no echo". It must survive scaling untouched, or the zero-run
   * filter and every zero_frames statistic stop meaning what they mean for the
   * DYP, and the two sensors become impossible to compare. */
  if (raw_mm <= 0) {
    return raw_mm;
  }

  uint32_t water_mps = (uint32_t)DB_PARAM_WATER_MPS;
  if (water_mps == 0U) {
    return raw_mm; /* parameter cleared: report the module's own number */
  }

  /* Undo the module's air assumption, apply water. 64-bit intermediate: at
   * 6000 mm x 1500 this is ~9e6, comfortably inside 32 bits, but the cast
   * costs nothing and removes the question. */
  int64_t scaled =
      ((int64_t)raw_mm * (int64_t)water_mps) / (int64_t)SR04M_ASSUMED_AIR_MPS;
  if (scaled > INT32_MAX) {
    return INT32_MAX;
  }
  return (int)scaled;
}

const sonar_driver_t sonar_driver_aj_sr04m = {
    .name = "aj-sr04m",
    .display_name = "AJ-SR04M / JSN-SR04T",
    .baud_rate = 9600,
    .trigger_byte = 0x55,
    .frame_size = 4,
    .response_timeout_ms = 120,
    .trigger_interval_ms = 100,
    .zero_hold_grace_ms = 600,
    .distance_stale_ms = 3000,
    .parse = sr04m_parse,
    .scale_mm = sr04m_scale_mm,
};
