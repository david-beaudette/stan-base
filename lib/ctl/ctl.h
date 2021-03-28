/** STAN THE STANDING ROBOT
   Control function
   by David Beaudette   
**/

#ifndef CTL_H
#define CTL_H

#include "Arduino.h"

#define  MOTL_STEP_PIN   9 // PB1
#define  MOTL_STEP_SET_ORMASK B00000010
#define  MOTL_STEP_CLR_ANDMASK B11111101
#define  MOTL_DIR    5
#define  MOTR_STEP_PIN   10 // PB2
#define  MOTR_STEP_SET_ORMASK B00000100
#define  MOTR_STEP_CLR_ANDMASK B11111011
#define  MOTR_DIR    7
#define  MOT_NSLEEP  8

#define  TIMER1_DOWN unsigned char sreg = SREG;  \
                     cli();                      \
                     TCCR1B = 0x00  

#define  TIMER1_UP   TCCR1B = 0x01;  \
                     sei();          \
                     SREG = sreg    


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