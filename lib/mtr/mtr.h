/** STAN THE STANDING ROBOT
   Program to control the stepper motors
   by David Beaudette   
**/

#ifndef MTR_H
#define MTR_H

#include <stdint.h>

// On the RP2040, any even pin and the next odd pin can be used in the same PWM slice
#define  MTR_STEP_L_PIN  8
#define  MTR_DIR_L_PIN   9
#define  MTR_STEP_R_PIN 10
#define  MTR_DIR_R_PIN  11
#define  MTR_MS1_PIN    14 
#define  MTR_MS2_PIN    15  
#define  MTR_NSLEEP_PIN 12  

#define MTR_LEFT  0U
#define MTR_RIGHT 1U

enum MotorMicroStep {
  MTR_USTEP_FULL    = 0,
  MTR_USTEP_HALF    = 1,
  MTR_USTEP_QUARTER = 2,
  MTR_USTEP_EIGHTH  = 3,
}; 

void mtr_init();
void mtr_enable();
void mtr_disable();

float mtr_set_speed(uint8_t motor_num_ui8, float speed_f32);

void mtr_set_microstep(uint8_t microstep_ui8);
float mtr_get_micro_step_frc(void);

uint16_t mtr_radps2wrap(float *radps_f32, 
                        const uint8_t micro_step_cur_ui8,
                        const float clk_div_inv_f32);

int64_t mtr_get_left_count(void);
int64_t mtr_get_right_count(void);

#endif // MTR_H