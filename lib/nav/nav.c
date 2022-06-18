#include "pico/stdlib.h"

#include "nav.h"
#include "bno055.h"

const uint num_retry_max = 5;

bool nav_init() {
  bool init_ok = false;

  for (uint retry = 0; retry < num_retry_max; ++retry) {
    if (bno055_init()) {
      init_ok = true;
      break;
    }
    sleep_ms(1000);
  }

  return init_ok;
}

void nav_get_calib_state(uint8_t *gyro_ui8, uint8_t *accel_ui8,
                         uint8_t *mag_ui8) {
  uint8_t sys_calib_state_ui8;
  bno055_get_calib_state(&sys_calib_state_ui8, gyro_ui8, accel_ui8, mag_ui8);

  return;
}

void nav_get_rpy(float *rpy_est_f32) {
  float yrp_f32[3];
  bno055_get_rpy(yrp_f32);

  rpy_est_f32[0] = -yrp_f32[1];
  rpy_est_f32[1] =  yrp_f32[2];
  rpy_est_f32[2] =  yrp_f32[0];

  return;
}

void nav_get_ome(float *ome_est_f32) {
  bno055_get_ome(ome_est_f32);
  return;
}

float nav_get_temp() {
  return (float)bno055_read8(BNO055_TEMP_ADDR);
}
