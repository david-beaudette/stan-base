/** STAN THE STANDING ROBOT
   Program to test the mtrera pan pan servo control function
   by David Beaudette
**/

// Libraries
#include <stdio.h>
#include "pico/stdlib.h"

#include "hardware/clocks.h"
#include "hardware/pwm.h"

#include "mtr.h"
#include "uip.h"
#include "uti.h"

#define RAMP_DELAY_MS 10
#define FIXPOS_DELAY_MS 250
#define NUM_SPEED_VALUES 5
// #define TEST_LIMITS

int main() {
  const uint LED_PIN = PICO_DEFAULT_LED_PIN;
  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);

  stdio_init_all();

  sleep_ms(500);

  mtr_init();
  uip_init();

  mtr_enable();
  printf("Motors are enabled.\n");

  printf("Slice   L %d  R %d\n", pwm_gpio_to_slice_num(MTR_STEP_L_PIN),
                                 pwm_gpio_to_slice_num(MTR_STEP_R_PIN));
  printf("Channel L %d  R %d\n", pwm_gpio_to_channel(MTR_STEP_L_PIN),
                                 pwm_gpio_to_channel(MTR_STEP_R_PIN));

  float speed_left_vec_f32[NUM_SPEED_VALUES] = {0.0f, -32.9f, 0.45f, 0.26f, -0.06f};
  float speed_right_vec_f32[NUM_SPEED_VALUES] = {0.0f, 1.0f, -1.0f, -2.0f, 2.0f};
  float speed_left_cur_f32 = 0.0f;
  float speed_right_cur_f32 = 0.0f;
  bool led_state_b = true;
  int speed_idx = 0;

  while (true) {    
    gpio_put(LED_PIN, led_state_b);
    led_state_b = !led_state_b;

    speed_left_cur_f32 = 
        mtr_set_speed(MTR_LEFT, speed_left_vec_f32[speed_idx]);
    speed_right_cur_f32 =
        mtr_set_speed(MTR_RIGHT, speed_right_vec_f32[speed_idx]);

    sleep_ms(1000);
    printf("Speed left: %f; right: %f\n",
           speed_left_cur_f32, 
           speed_right_cur_f32);

    ++speed_idx;
    if(speed_idx == NUM_SPEED_VALUES) {
      speed_idx = 0;
    }
  }
  printf("Stepper motor test complete.\n");
  mtr_disable();

  return 0;
}