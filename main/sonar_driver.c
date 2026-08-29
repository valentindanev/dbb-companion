#include "sonar_driver.h"

/* Model registry. One row per supported transducer. */
static const sonar_driver_t *const s_drivers[SONAR_MODEL_COUNT] = {
    [SONAR_MODEL_DYP_L041MTW] = &sonar_driver_dyp_l041mtw,
    [SONAR_MODEL_AJ_SR04M] = &sonar_driver_aj_sr04m,
};

const sonar_driver_t *sonar_driver_get(sonar_model_t model) {
  if (model < 0 || model >= SONAR_MODEL_COUNT) {
    return NULL;
  }
  return s_drivers[model];
}

const char *sonar_driver_name(sonar_model_t model) {
  const sonar_driver_t *driver = sonar_driver_get(model);
  return driver != NULL ? driver->name : "unknown";
}
