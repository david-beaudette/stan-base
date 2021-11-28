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

  float speed_left_f32 = 1.0f;
  float speed_right_f32 = 1.0f;
  while (true) {    
    gpio_put(LED_PIN, 1);
    mtr_set_speed(MTR_LEFT, speed_left_f32);
    mtr_set_speed(MTR_RIGHT, speed_right_f32);
    sleep_ms(1000);

    gpio_put(LED_PIN, 0);
    mtr_set_speed(MTR_LEFT, speed_left_f32);
    mtr_set_speed(MTR_RIGHT, speed_right_f32);
    sleep_ms(1000);
  }
  printf("Stepper motor test complete.\n");
  mtr_disable();

  return 0;
}