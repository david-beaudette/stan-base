/** STAN THE STANDING ROBOT
   Program to control the camera pan tilt servos
   by David Beaudette   
**/

#ifndef CAM_H
#define CAM_H

#include <stdint.h>

// On the RP2040, any even pin and the next odd pin can be used in the same PWM slice
#define  CAM_PAN_PIN   7 
#define  CAM_TILT_PIN  6  

void cam_init();
void cam_center();
float cam_set_pan(float angle_deg_f32);
float cam_set_tilt(float angle_deg_f32);
float cam_get_pan();
float cam_get_tilt();
void cam_disable();


#endif // CAM_H