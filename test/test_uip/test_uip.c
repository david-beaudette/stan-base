/** STAN THE STANDING ROBOT
   Program to test the camera pan pan servo control function
   by David Beaudette
**/

// Libraries
#include <stdio.h>
#include "pico/stdlib.h"
#include "uip.h"

int main() {
  const uint LED_PIN = PICO_DEFAULT_LED_PIN;
  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);
  stdio_init_all();

  uip_init();

  while (true) {
    gpio_put(LED_PIN, 1);
    sleep_ms(500);
    gpio_put(LED_PIN, 0);
    sleep_ms(500);
    printf("Button position %0.2f deg.\n", uip_get_pot_pct());
  }

  return 0;
}