#include "sonar_driver.h"

int sonar_pulse_preserve_no_echo(uint32_t echo_us, int scaled_mm) {
#ifdef DB_MUTATE_AJ_NO_ECHO_GUARD
  (void)echo_us;
  return scaled_mm;
#else
  return echo_us == 0U ? 0 : scaled_mm;
#endif
}

uint32_t sonar_pulse_clamp_idle_ceiling(uint32_t width_us,
                                       uint32_t echo_max_us) {
#ifdef DB_MUTATE_AJ_IDLE_CLAMP_GUARD
  (void)echo_max_us;
  return width_us;
#else
  return width_us + 4U >= echo_max_us ? 0U : width_us;
#endif
}

bool sonar_driver_descriptor_is_complete(const sonar_driver_t *driver) {
  if (driver == NULL) return false;
#ifdef DB_MUTATE_AJ_LINK_DESCRIPTOR_GUARD
  return driver->parse != NULL || driver->scale_us != NULL;
#else
  return driver->link == SONAR_LINK_PULSE ? driver->scale_us != NULL
                                          : driver->parse != NULL;
#endif
}
