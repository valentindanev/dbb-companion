#ifndef DB_ESP32_DEEPER_UDP_SONAR_H
#define DB_ESP32_DEEPER_UDP_SONAR_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  bool has_depth;
  int depth_mm;
  bool has_temperature;
  int temperature_c_tenths;
  bool has_satellites;
  int satellites;
  bool gps_fix_valid;
  bool has_coordinates;
  double latitude_deg;
  double longitude_deg;
  uint32_t newest_sample_age_ms;
} deeper_udp_snapshot_t;

void deeper_udp_sonar_start(void);
bool deeper_udp_sonar_get_latest_distance(int *out_distance_mm);
bool deeper_udp_sonar_get_snapshot(deeper_udp_snapshot_t *snapshot);

#endif // DB_ESP32_DEEPER_UDP_SONAR_H
