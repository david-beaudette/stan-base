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

const float stepperrev_f32 = 200.0f;
const float wheelradperstep_f32 = 2.0f * M_PI / stepperrev_f32;
// const float motorstepsperpulse_f32 = 1.0 / pulsesperstep_f32;
// const float pace2ticksperpol_f32 = 0.5f *
//                                    clocktickspersec_f32 *
//                                    wheelradperstep_f32 *
//                                    motorstepsperpulse_f32;

// const float speed_min_pos_f32 = 1.0f * DEG_TO_RAD;
// const float speed_min_neg_f32 = -1.0f * DEG_TO_RAD;

const float speed_max_f32 = 5.23f * 2.0f * M_PI; // 5.23 rev/s

// const float rev_per_pulse_f32 = 1.0f / (stepperrev_f32 * pulsesperstep_f32);

uint slice_left;
uint channel_left; 
uint slice_right;
uint channel_right;
uint8_t micro_step_cur_ui8;

float micro_step_frc_f32[4] = {1.0f, 0.5f, 0.25f, 0.125f};
float clocktickspersec_f32;

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

  pwm_set_chan_level(slice_left, channel_left, 1);
  pwm_set_chan_level(slice_right, channel_right, 1);

  mtr_disable();

  mtr_set_speed(MTR_LEFT,  0.0f);
  mtr_set_speed(MTR_RIGHT, 0.0f);

  clocktickspersec_f32 = (float)clock_get_hz(clk_sys);
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

float mtr_set_speed(uint8_t motor_num_ui8, float speed_f32) {
  float speed_cur_f32 = speed_f32;
  uint16_t wrap_val_ui16 = 0U;

  // Limit speed
  if(speed_cur_f32 < -speed_max_f32) {
    speed_cur_f32 = -speed_max_f32;
  }
  else if(speed_cur_f32 > speed_max_f32) {
    speed_cur_f32 = speed_max_f32;
  }

  if(motor_num_ui8 == MTR_LEFT) {
    if(speed_f32 > 0.0f) {
      // Forward left motor
      gpio_put(MTR_DIR_L_PIN, 0);
    }
    else {
      // Reverse left motor
      gpio_put(MTR_DIR_L_PIN, 1);
    }
    wrap_val_ui16 = mtr_radps2wrap(&speed_cur_f32, 
                                   micro_step_cur_ui8);
    pwm_set_wrap(slice_left, wrap_val_ui16);
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
    wrap_val_ui16 = mtr_radps2wrap(&speed_cur_f32, 
                                   micro_step_cur_ui8);
    pwm_set_wrap(slice_left, wrap_val_ui16);
  }
  return speed_cur_f32;
}

void mtr_set_microstep(uint8_t micro_step_ui8) {
  micro_step_cur_ui8 = micro_step_ui8;
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
        micro_step_cur_ui8 = MTR_USTEP_FULL;
        gpio_put(MTR_MS1_PIN, 0);
        gpio_put(MTR_MS2_PIN, 0);
      break;
  }
}

uint16_t mtr_radps2wrap(float *radps_f32, 
                        const uint8_t micro_step_cur_ui8) {
  float micro_step_frc_cur_f32 =
      micro_step_frc_f32[MIN(MTR_USTEP_EIGHTH, micro_step_cur_ui8)];

  // Compute the value to put in the PWM generator
  uint16_t wrap_ui16 =
      (uint16_t)(1.0f / (*radps_f32) * wheelradperstep_f32 *
                 micro_step_frc_cur_f32 * clocktickspersec_f32);

  // Compute back the resulting speed for reporting               
  (*radps_f32) =
      1.0f / (float)wrap_ui16 * micro_step_frc_cur_f32 * wheelradperstep_f32;

  return wrap_ui16;
}