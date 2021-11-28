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

#define MTR_USTEP_FULL     1U
#define MTR_USTEP_HALF     2U
#define MTR_USTEP_QUARTER  4U
#define MTR_USTEP_EIGHTH   8U

void mtr_init();
void mtr_enable();
void mtr_disable();

void mtr_set_speed(uint8_t motor_num_ui8, float speed_f32);
void mtr_set_microstep(uint8_t microstep_ui8);

#endif // MTR_H