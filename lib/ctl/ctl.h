/** STAN THE STANDING ROBOT
   Control function
   by David Beaudette   
**/

#ifndef CTL_H
#define CTL_H

#include "Arduino.h"

#define  MOTL_STEP_PIN   9 // PB1
#define  MOTL_STEP_SET_OR_MASK B00000010
#define  MOTL_STEP_CLR_AND_MASK B11111101
#define  MOTL_DIR    5 // PD5
#define  MOTL_DIR_SET_OR_MASK B00100000
#define  MOTL_DIR_CLR_AND_MASK B11011111

#define  MOTR_STEP_PIN   10 // PB2
#define  MOTR_STEP_SET_OR_MASK B00000100
#define  MOTR_STEP_CLR_AND_MASK B11111011
#define  MOTR_DIR    7 // PD7
#define  MOTR_DIR_SET_OR_MASK B10000000
#define  MOTR_DIR_CLR_AND_MASK B01111111

#define  MOT_NSLEEP  8

#define  TIMER1_DOWN unsigned char sreg = SREG;  \
                     cli();                      \
                     TCCR1B = 0x00  

#define  TIMER1_UP   TCCR1B = 0x01;  \
                     sei();          \
                     SREG = sreg    

struct MotorPulse {
  uint32_t pol_ticks_cur_ui32;
  uint16_t timer_val_prev_ui16;
  uint16_t cmp_val_cur_ui16;
  uint32_t pol_ticks_tgt_ui32;
  int32_t pulse_count_cur_i32;
  int32_t dir_cur_i32;
  int32_t dir_sign_i32;
  byte step_clrandmask_cst_ui8;
  byte step_setormask_cst_ui8;
  byte dir_clrandmask_cst_ui8;
  byte dir_setormask_cst_ui8;
  byte compare_register_ui8;
  bool pulse_pol_cur_b;
};

void ctl_init();

void ctl_enable_motors(void);
  
void ctl_disable_motors(void);

void ctl_set_motor_speed(const float speed_left, 
                         const float speed_right);

bool ctl_set_single_motor_speed(volatile MotorPulse *mot, 
                                float speed_cmd_f32);

void ctl_reset_motor_pos();

void ctl_get_motor_num_rev(float &motl_num_rev_f32, 
                           float &motr_num_rev_f32);

void ctl_toggle_motor_pin(volatile MotorPulse *mot);


#endif // CTL_H