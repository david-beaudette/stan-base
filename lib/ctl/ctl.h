/** STAN THE STANDING ROBOT
   Control function
   by David Beaudette   
**/

#ifndef CTL_H
#define CTL_H

#include "pico/stdlib.h"

void ctl_init(void);

void ctl_start(void);
void ctl_stop(void);

void ctl_set_pitch(const float pitch_cmd, 
                   const float pitch_est);

void ctl_get_wheel_pos(float *wheel_left_num_rev_f32, 
                       float *wheel_right_num_rev_f32);

#endif // CTL_H