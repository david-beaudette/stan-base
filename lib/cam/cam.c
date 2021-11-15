/** STAN THE STANDING ROBOT
   Program to control the camera pan tilt servos
   by David Beaudette
**/

#include "cam.h"

#include <math.h>
#include <stdio.h>

#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"

// Private PWM data structure
typedef struct{
    int pin;
    int slice;
    int channel;
    int value;
    uint8_t enabled;
    int pulse_max;
    int pulse_min;
    int delay_us_per_deg;
    float angle_deg_prev_f32;
} PWM;

// Private functions from the servo library
void set_servo_angle(PWM *pwm,float degree);
PWM enable_servo(int pin);
void disable_servos();
void move_delay(int delay,
                float angle_start_deg_f32,
                float angle_end_deg_f32);

// Global variables
PWM cam_tilt_pwm;
PWM cam_pan_pwm;

void cam_init() {
  cam_tilt_pwm = enable_servo(CAM_TILT_PIN);
  cam_tilt_pwm.pulse_max = 2400;
  cam_tilt_pwm.pulse_min = 630;

  cam_pan_pwm = enable_servo(CAM_PAN_PIN);
  cam_pan_pwm.pulse_max = 2480;
  cam_pan_pwm.pulse_min = 700;
}

float cam_set_tilt(float angle_deg_f32) {   
  set_servo_angle(&cam_tilt_pwm, angle_deg_f32); 
  return angle_deg_f32;
}

float cam_set_pan(float angle_deg_f32) {   
  set_servo_angle(&cam_pan_pwm, angle_deg_f32); 
  return angle_deg_f32;
}

void cam_center() {   
  set_servo_angle(&cam_tilt_pwm, 0.0f); 
  set_servo_angle(&cam_pan_pwm, 0.0f); 
  return;
}

void cam_disable() {
  disable_servos();
}

// Private functions definitions
void set_servo_angle(PWM *pin, float angle_deg_f32) {
  if (angle_deg_f32 < -90.0f) {
    angle_deg_f32 = -90.0f;
  }
  if (angle_deg_f32 > 90.0f) {
    angle_deg_f32 = 90.0f;
  }
  // Convert the angle to uSec ticks
  int value =
      (int)((((float)(pin->pulse_max - pin->pulse_min)) * ((angle_deg_f32 + 90.0) / 180.0)) +
            pin->pulse_min);

  // Save the lastangle_deg_f32 so we can determine delay for movement of the servo
  int angle_deg_prev_f32 = pin->angle_deg_prev_f32;

  // Store the current angle_deg_f32 as lastangle_deg_f32
  pin->angle_deg_prev_f32 = angle_deg_f32;

  // Store the last value, probably don't need
  pin->value = value;

  // Set the pwm level, this is only for the first pin->pulse_max uSecs of the
  // frame.
  pwm_set_chan_level(pin->slice, pin->channel, pin->value);

  // Need time for the servo to move to its new position
  move_delay(pin->delay_us_per_deg, angle_deg_prev_f32, angle_deg_f32);

  printf("slice:%d channel:%d value:%d angle_deg_f32:%3.0f \r\n", pin->slice,
         pin->channel, pin->value, angle_deg_f32);
}

void move_delay(int delay_us_per_deg,
                float angle_start_deg_f32,
                float angle_end_deg_f32) {

  // Get the absolute value of the difference between the start and end point,
  // multiple by delay
  int delay = fabs(angle_start_deg_f32 - angle_end_deg_f32) * delay_us_per_deg;

  sleep_us(delay);
}

PWM enable_servo(int pin) {
  // Set defaults
  PWM pwm;
  pwm.pulse_max = 2500;
  pwm.pulse_min = 500;
  pwm.value = 0;
  pwm.delay_us_per_deg = 2000;
  pwm.angle_deg_prev_f32 = 0;
  pwm.pin = pin;

  // Turn the pin into a pwm pin
  gpio_set_function(pwm.pin, GPIO_FUNC_PWM);

  // Need to store the slice and channel
  pwm.slice = pwm_gpio_to_slice_num(pin);
  pwm.channel = pwm_gpio_to_channel(pin);

  // Set the clock to uSecs
  pwm_set_clkdiv(pwm.slice, 125.0f);

  // wrap every 20000 uSecs or 1 frame, first 2500 uSecs determine duty cycle or
  // how far the servo moves...
  pwm_set_wrap(pwm.slice, 20000);

  // Enable the servo
  pwm_set_enabled(pwm.slice, true);

  return pwm;
}

void disable_servos() {
  // Disable both servos (they're on the same slice)
  pwm_set_enabled(cam_tilt_pwm.slice, false);
}
