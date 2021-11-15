/** STAN THE STANDING ROBOT
   Program to test the camera pan pan servo control function
   by David Beaudette
**/

// Libraries
#include <stdio.h>

#include "cam.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"

#define RAMP_DELAY_MS 50
#define FIXPOS_DELAY_MS 500
#define TEST_LIMITS

int main() {
  const uint LED_PIN = PICO_DEFAULT_LED_PIN;
  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);

  stdio_init_all();

  sleep_ms(2500);
  printf("starting...\r\n");

  cam_init();

  cam_center();
  sleep_ms(1000);

#ifdef TEST_LIMITS
    cam_set_tilt(-90.0f);
    cam_set_pan(-90.0f);
    sleep_ms(10000);

    cam_set_tilt(90.0f);
    cam_set_pan(90.0f);
    sleep_ms(10000);
#endif

  for (int i = -95; i < 95; i++) {
    cam_set_tilt((float)i);
    sleep_ms(RAMP_DELAY_MS);
  }
  for (int i = 95; i > -95; i--) {
    cam_set_tilt((float)i);
    sleep_ms(RAMP_DELAY_MS);
  }
  cam_set_tilt(-90.0f);
  sleep_ms(FIXPOS_DELAY_MS);
  cam_set_tilt(-45.0f);
  sleep_ms(FIXPOS_DELAY_MS);
  cam_set_tilt(0.0f);
  sleep_ms(FIXPOS_DELAY_MS);
  cam_set_tilt(45.0f);
  sleep_ms(FIXPOS_DELAY_MS);
  cam_set_tilt(90.0f);
  sleep_ms(FIXPOS_DELAY_MS);
  cam_set_tilt(-90.0f);
  sleep_ms(1000);

  cam_center();
  sleep_ms(1000);

  for (int i = -95; i < 95; i++) {
    cam_set_pan((float)i);
    sleep_ms(RAMP_DELAY_MS);
  }
  for (int i = 95; i > -95; i--) {
    cam_set_pan((float)i);
    sleep_ms(RAMP_DELAY_MS);
  }
  cam_set_pan(-90.0f);
  sleep_ms(FIXPOS_DELAY_MS);
  cam_set_pan(-45.0f);
  sleep_ms(FIXPOS_DELAY_MS);
  cam_set_pan(0.0f);
  sleep_ms(FIXPOS_DELAY_MS);
  cam_set_pan(45.0f);
  sleep_ms(FIXPOS_DELAY_MS);
  cam_set_pan(90.0f);
  sleep_ms(FIXPOS_DELAY_MS);
  cam_set_pan(-90.0f);
  sleep_ms(1000);

  cam_center();
  sleep_ms(1000);

  cam_disable();
  printf("done\r\n");

  while (true) {
    gpio_put(LED_PIN, 1);
    sleep_ms(2500);
    gpio_put(LED_PIN, 0);
    sleep_ms(2500);
    printf("Camera servo test complete.\n");
  }
}