/** STAN THE STANDING ROBOT
   Navigation function
   by David Beaudette   
**/

#ifndef NAV_H
#define NAV_H

#include <stdint.h>
#include "pico/stdlib.h"

void nav_get_rpy(float *rpy_est_f32);
void nav_get_ome(float *ome_est_f32);
float nav_get_temp();

void nav_get_calib_state(uint8_t *gyro_ui8, 
                         uint8_t *accel_ui8,
                         uint8_t *mag_ui8);

bool nav_init();

void nav();

#endif // NAV_H