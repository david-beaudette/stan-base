/** STAN THE STANDING ROBOT
   Program to control the stepper motors
   by David Beaudette
**/

#include "mtr.h"

#include <math.h>
#include <stdio.h>

#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"

const float pulsesperstep_f32 = 2.0f;
const float stepperrev_f32 = 200.0f;
// const float clocktickspersec_f32 = (float)F_CPU;
// const float wheelradperstep_f32 = TWO_PI / stepperrev_f32;
// const float motorstepsperpulse_f32 = 1.0 / pulsesperstep_f32;
// const float pace2ticksperpol_f32 = 0.5f *
//                                    clocktickspersec_f32 *
//                                    wheelradperstep_f32 *
//                                    motorstepsperpulse_f32;

// const float speed_min_pos_f32 = 1.0f * DEG_TO_RAD;
// const float speed_min_neg_f32 = -1.0f * DEG_TO_RAD;

// const float speed_max_f32 = 5.23 * TWO_PI; // 5.23 rev/s

// const float rev_per_pulse_f32 = 1.0f / (stepperrev_f32 * pulsesperstep_f32);

uint slice_left;
uint channel_left; 
uint slice_right;
uint channel_right;

void mtr_init(void) {
  // Configure control pins
  gpio_init(MTR_NSLEEP_PIN);
  gpio_set_dir(MTR_NSLEEP_PIN, GPIO_OUT); 

  gpio_init(MTR_MS1_PIN);
  gpio_init(MTR_MS2_PIN);
  gpio_set_dir(MTR_MS1_PIN, GPIO_OUT);
  gpio_set_dir(MTR_MS2_PIN, GPIO_OUT);
  mtr_set_microstep(MTR_USTEP_FULL);

  // Configure direction outputs
  gpio_init(MTR_DIR_L_PIN);
  gpio_init(MTR_DIR_R_PIN);
  gpio_set_dir(MTR_DIR_L_PIN, GPIO_OUT); 
  gpio_set_dir(MTR_DIR_R_PIN, GPIO_OUT); 

  // Configure step outputs as PWM
  gpio_set_function(MTR_STEP_L_PIN, GPIO_FUNC_PWM);
  gpio_set_function(MTR_STEP_R_PIN, GPIO_FUNC_PWM);

  // Save slice and channel
  slice_left = pwm_gpio_to_slice_num(MTR_STEP_L_PIN);
  channel_left = pwm_gpio_to_channel(MTR_STEP_L_PIN);
  slice_right = pwm_gpio_to_slice_num(MTR_STEP_R_PIN);
  channel_right = pwm_gpio_to_channel(MTR_STEP_R_PIN);

  // Set the clock to uSecs
  pwm_set_clkdiv(slice_left, 125.0f);

  // Wrap every 20000 uSecs
  pwm_set_wrap(slice_left, 20000);
  pwm_set_wrap(slice_right, 20000);

  mtr_disable();

  mtr_set_speed(MTR_LEFT,  0.0f);
  mtr_set_speed(MTR_RIGHT, 0.0f);

}

void mtr_enable(void) {
  gpio_put(MTR_NSLEEP_PIN, 1);
  pwm_set_enabled(slice_left, true);
  pwm_set_enabled(slice_right, true);
}

void mtr_disable(void) {
  gpio_put(MTR_NSLEEP_PIN, 0);
  pwm_set_enabled(slice_left, false);
  pwm_set_enabled(slice_right, false);
}

void mtr_set_speed(uint8_t motor_num_ui8, float speed_f32) {
  if(motor_num_ui8 == MTR_LEFT) {
    if(speed_f32 > 0.0f) {
      // Forward left motor
      gpio_put(MTR_DIR_L_PIN, 0);
    }
    else {
      // Reverse left motor
      gpio_put(MTR_DIR_L_PIN, 1);
    }
  }
  else {
    if (speed_f32 > 0.0f) {
      // Forward right motor
      gpio_put(MTR_DIR_R_PIN, 1);
    } 
    else {
      // Reverse right motor
      gpio_put(MTR_DIR_R_PIN, 0);
    }
  }
}

void mtr_set_microstep(uint8_t micro_step_ui8) {
  switch(micro_step_ui8) {
    case MTR_USTEP_EIGHTH:
        gpio_put(MTR_MS1_PIN, 1);
        gpio_put(MTR_MS2_PIN, 1);
      break;
    case MTR_USTEP_QUARTER:
        gpio_put(MTR_MS1_PIN, 0);
        gpio_put(MTR_MS2_PIN, 1);
      break;
    case MTR_USTEP_HALF:
        gpio_put(MTR_MS1_PIN, 1);
        gpio_put(MTR_MS2_PIN, 0);
      break;
    default:
        gpio_put(MTR_MS1_PIN, 0);
        gpio_put(MTR_MS2_PIN, 0);
      break;
  }
}

