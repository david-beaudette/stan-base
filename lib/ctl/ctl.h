/** STAN THE STANDING ROBOT
   Control function
   by David Beaudette   
**/

#ifndef CTL_H
#define CTL_H

#include "Arduino.h"

#define  MOTL_STEP   9
#define  MOTL_DIR    5
#define  MOTR_STEP   10
#define  MOTR_DIR    7
#define  MOT_NSLEEP  8

void ctl_init();

void ctl_set_motor_speed(const float speed_left, 
                         const float speed_right);

void ctl_reset_motor_pos();

struct MotorPulse {
  uint32_t pol_ticks_ui32;
  uint32_t num_ovf_cur_ui32;
  uint32_t num_ovf_tgt_ui32;
  int32_t pulse_count_cur_i32;
  bool pulse_pol_cur_b;
};

#endif // CTL_H