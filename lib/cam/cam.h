/** STAN THE STANDING ROBOT
   Camera control function
   by David Beaudette   
**/

#ifndef CAM_H
#define CAM_H

#define  CAM_PAN_PIN   11 // PB3
#define  CAM_TILT_PIN  3  // PD3

void cam_init();
void cam_center();
void cam_set_pan(float angle_deg_f32);
void cam_set_tilt(float angle_deg_f32);
void cam_pwm_cycle_end();

#endif // CAM_H