/** STAN THE STANDING ROBOT
   Program to control the stepper motors
   by David Beaudette
**/

#include "mtr.h"

#include <math.h>
#include <stdio.h>

#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "pico/stdlib.h"

const float stepperrev_f32 = 200.0f;
const float wheelradperstep_f32 = 2.0f * M_PI / stepperrev_f32;

const float speed_max_fast_f32 = 5.23f * 2.0f * M_PI; // 5.23 rev/s
const float speed_max_slow_f32 = 0.02f; 
const float speed_min_f32 = 0.001f; // eighth microstepping with 61367 us

const float clk_div_fast_f32 = 125.0f; // Clock divider set for us
const float clk_div_fast_inv_f32 = 1.0f / clk_div_fast_f32;
const float clk_div_slow_f32 = 125000.0f; // Clock divider set for ms
const float clk_div_slow_inv_f32 = 1.0f / clk_div_fast_f32;

uint slice_left;
uint channel_left; 
uint slice_right;
uint channel_right;

uint8_t micro_step_cur_ui8;
float micro_step_frc_f32[4] = {1.0f, 0.5f, 0.25f, 0.125f};
int64_t step_count_left_i64 = 0;
int64_t step_count_right_i64 = 0;

float clktickspersec_f32;

void mtr_pulse_isr(void);

void mtr_init(void) {
  // Configure control pins
  gpio_init(MTR_NSLEEP_PIN);
  gpio_set_dir(MTR_NSLEEP_PIN, GPIO_OUT); 

  gpio_init(MTR_MS1_PIN);
  gpio_init(MTR_MS2_PIN);
  gpio_set_dir(MTR_MS1_PIN, GPIO_OUT);
  gpio_set_dir(MTR_MS2_PIN, GPIO_OUT);

  mtr_set_microstep(MTR_USTEP_EIGHTH);

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

  // Set the initial clock divider
  pwm_set_clkdiv(slice_left, clk_div_slow_f32);
  pwm_set_clkdiv(slice_right, clk_div_slow_f32);

  clktickspersec_f32 = (float)clock_get_hz(clk_sys);

  pwm_set_chan_level(slice_left, channel_left, 1);
  pwm_set_chan_level(slice_right, channel_right, 1);

  mtr_disable();

  // Attach interrupts to each PWM module
  pwm_set_irq_enabled(slice_left, true);
  pwm_set_irq_enabled(slice_right, true);
  irq_set_exclusive_handler(PWM_IRQ_WRAP, &mtr_pulse_isr);
  irq_set_enabled(PWM_IRQ_WRAP, true);

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

float mtr_set_speed(uint8_t motor_num_ui8, float speed_f32) {
  float speed_cur_f32 = speed_f32;
  float speed_mag_f32 = fabsf(speed_f32);
  uint16_t wrap_val_ui16 = 0U;
  uint slice_id;

  // Limit speed
  if(speed_mag_f32 > speed_max_fast_f32) {
    speed_cur_f32 = copysignf(speed_max_fast_f32, speed_f32);
  }

  // Compute direction and set slice number based on 
  // motor number
  if(motor_num_ui8 == MTR_LEFT) {
    slice_id = slice_left;
    if(speed_cur_f32 > 0.0f) {
      // Forward left motor
      gpio_put(MTR_DIR_L_PIN, 0);
    }
    else {
      // Reverse left motor
      gpio_put(MTR_DIR_L_PIN, 1);
    }
  }
  else {
    slice_id = slice_right;
    if (speed_cur_f32 > 0.0f) {
      // Forward right motor
      gpio_put(MTR_DIR_R_PIN, 1);
    } 
    else {
      // Reverse right motor
      gpio_put(MTR_DIR_R_PIN, 0);
    }
  }
  if (speed_mag_f32 < speed_min_f32) {
    pwm_set_enabled(slice_id, false);
    speed_cur_f32 = 0.0f;
  } 
  else {
    if (speed_mag_f32 < speed_max_slow_f32) {
      // Using slow clock divider
      pwm_set_clkdiv(slice_id, clk_div_slow_f32);
      wrap_val_ui16 = mtr_radps2wrap(&speed_cur_f32, micro_step_cur_ui8,
                                     clk_div_slow_inv_f32);
    } else {
      // Using fast clock divider
      pwm_set_clkdiv(slice_id, clk_div_fast_f32);
      wrap_val_ui16 = mtr_radps2wrap(&speed_cur_f32, micro_step_cur_ui8,
                                     clk_div_fast_inv_f32);
    }
    pwm_set_wrap(slice_id, wrap_val_ui16);
    pwm_set_enabled(slice_id, true);
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

float mtr_get_micro_step_frc(void) {
  return micro_step_frc_f32[MIN(MTR_USTEP_EIGHTH, micro_step_cur_ui8)];
}

uint16_t mtr_radps2wrap(float *radps_f32, 
                        const uint8_t micro_step_cur_ui8,
                        const float clk_div_inv_f32) {
  // Compute the value to put in the PWM generator
  float secprad2wrap_f32 = wheelradperstep_f32 * mtr_get_micro_step_frc() *
       clktickspersec_f32 * clk_div_inv_f32;
  float wrap_f32 = 1.0f / fabsf(*radps_f32) * secprad2wrap_f32;
  uint16_t wrap_ui16 = (uint16_t)wrap_f32;

  // Compute back the resulting speed for reporting               
  (*radps_f32) =
      copysignf(1.0f / (float)wrap_ui16 * secprad2wrap_f32, *radps_f32);

  return wrap_ui16;
}

int64_t mtr_get_left_count(void) {
  return step_count_left_i64;
}

int64_t mtr_get_right_count(void) {
  return step_count_right_i64;
}

// Pulse counter
void mtr_pulse_isr(void) {
  
  if(pwm_hw->ints & (1 << slice_left)) {
    if(gpio_get(MTR_DIR_L_PIN)) {
      // Reverse left 
      --step_count_left_i64;
    }
    else {
      // Forward left
      ++step_count_left_i64;
    }
    pwm_clear_irq(slice_left);
  }
  if(pwm_hw->ints & (1 << slice_right)) {
    if(gpio_get(MTR_DIR_R_PIN)) {
      // Forward right 
      ++step_count_right_i64;
    }
    else {
      // Reverse right
      --step_count_right_i64;
    }
    pwm_clear_irq(slice_right);
  }
}
