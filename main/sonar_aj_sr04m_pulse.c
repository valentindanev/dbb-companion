#include "sonar_driver.h"

#include "db_parameters.h"

/*
 * AJ-SR04M / JSN-SR04T in its AS-SHIPPED pulse/echo mode (R19 open).
 *
 * This is the mode ArduPilot uses (`RNGFND1_TYPE = 30`, the HC-SR04 driver),
 * and the mode the module leaves the factory in. It is here because the
 * UART-triggered mode would not talk on the bench: the module measured and
 * blinked, but never put a byte on the wire.
 *
 * WHY THIS IS NOT A RETREAT - it removes a conversion instead of adding one.
 * In serial mode the module converts echo time to millimetres using the AIR
 * constant (348 m/s) and we then undo that and re-apply the water constant, so
 * every reading passes through two roundings and lands on a 4.25 mm grid: the
 * module reports whole air-millimetres, and one of those is 1480/348 = 4.25 mm
 * of real water depth. In pulse mode the module converts nothing. The echo pin
 * is simply high for the round-trip time and the host owns the speed of sound
 * outright, so one multiply gets us there and the grid is set by the capture
 * resolution instead - 0.74 mm per microsecond tick. Nearly 6x finer.
 *
 * WHY THE JITTER OBJECTION NO LONGER APPLIES. The 02-09 decision rejected
 * pulse/echo because the ESP would be timing a pulse in software under Wi-Fi
 * load, where 100 us of jitter is 7.4 cm of depth. That reasoning was correct
 * for a software timer. It does not apply here: danevi_sonar.c captures the
 * echo with the RMT peripheral, which timestamps edges in hardware with no CPU
 * involvement, so task scheduling, interrupts and Wi-Fi traffic cannot move the
 * number.
 *
 * HARDWARE - R19 must be OPEN (removed). That is the shipped state, mode 1.
 * Wiring is unchanged from the UART attempt: `ss_tx_pin` drives the module's
 * Trig, `ss_rx_pin` reads its Echo through the 10k/20k divider. No rewiring.
 *
 * TRIGGER WIDTH - 20 us, not ArduPilot's 10 us. Their trigger is driven by a
 * 5 V-tolerant autopilot pin; ours is a 3.3 V ESP pin into a module running at
 * 5 V, and users report the short pulse being missed in exactly that
 * arrangement. 20 us costs nothing and removes the question.
 *
 * BLIND ZONE - the module's dead band is a fixed RINGDOWN TIME, not a fixed
 * distance, so it scales with the medium just like everything else here. The
 * ~20 cm quoted in air is ~1.15 ms of round trip, and that same 1.15 ms is
 * ~85 cm of water. Expect a floor near 0.85 m. This is NOT introduced by pulse
 * mode - the physics is identical in serial mode, where it was simply never
 * written down.
 */

/* Round trip -> one way, and m/s -> mm/us, in a single divisor:
 *   depth_mm = echo_us * water_mps / 2000
 * Sanity: 4054 us at 1480 m/s = 3000 mm. */
#define SR04M_PULSE_US_PER_MM_DIVISOR 2000ULL

/* Fallback if `ss_water_mps` has been cleared to 0. The UART driver treats 0 as
 * "report the module's own number", but in pulse mode there IS no number of the
 * module's own - only microseconds - so 0 cannot mean pass-through here. Using
 * the parameter default keeps a cleared parameter from reporting zero depth. */
#define SR04M_PULSE_DEFAULT_WATER_MPS 1480U

static int sr04m_pulse_scale_us(uint32_t echo_us) {
  /* 0 means "no echo". It must survive untouched so the zero-run filter and
   * every zero_frames statistic keep the meaning they have for the DYP and for
   * the UART driver, and the three stay comparable. */
  if (echo_us == 0U) {
    return 0;
  }

  uint32_t water_mps = (uint32_t)DB_PARAM_WATER_MPS;
  if (water_mps == 0U) {
    water_mps = SR04M_PULSE_DEFAULT_WATER_MPS;
  }

  uint64_t depth_mm =
      ((uint64_t)echo_us * (uint64_t)water_mps) / SR04M_PULSE_US_PER_MM_DIVISOR;
  if (depth_mm > (uint64_t)INT32_MAX) {
    return INT32_MAX;
  }
  return (int)depth_mm;
}

const sonar_driver_t sonar_driver_aj_sr04m_pulse = {
    .name = "aj-sr04m-pulse",
    .display_name = "AJ-SR04M / JSN-SR04T (pulse)",
    .link = SONAR_LINK_PULSE,

    /* UART fields are unused on this link and are left zero deliberately:
     * a nonzero baud rate here would be a lie about how the reading arrives. */
    .baud_rate = 0,
    .trigger_byte = 0,
    .frame_size = 0,
    .parse = NULL,

    .trigger_pulse_us = 20,

    /* Longest echo we will wait out before calling the line idle. 12 ms is
     * 8.9 m of water, comfortably past anything this transducer can return, and
     * it bounds the cost of a no-echo cycle. It must stay under 32767 us: the
     * RMT idle threshold is 15 bits at the 1 MHz capture resolution. */
    .echo_max_us = 12000,

    /* Arming, the 12 ms idle detection above, and margin for the module's own
     * ringdown. Not the DYP's 30 ms - that number is about a UART frame. */
    .response_timeout_ms = 60,
    .trigger_interval_ms = 100,
    .zero_hold_grace_ms = 600,
    .distance_stale_ms = 3000,

    .scale_mm = NULL,
    .scale_us = sr04m_pulse_scale_us,
};
