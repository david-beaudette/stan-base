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
float cam_set_pan(float angle_deg_f32);
float cam_set_tilt(float angle_deg_f32);
void cam_pwm_cycle_end();

float angle2ticks(volatile uint8_t *num_oci_tgt_ui8,
                  volatile uint8_t *oci_val_ui8,
                  const float angle_deg_f32,
                  const int32_t pw_min_ticks_i32,
                  const int32_t pw_max_ticks_i32);

float cam_get_pan();
float cam_get_tilt();

#endif // CAM_H