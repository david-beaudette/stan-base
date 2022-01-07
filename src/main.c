/** STAN THE STANDING ROBOT
   Program for the base / low-level controller
   by David Beaudette
**/

#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "cam.h"
#include "uip.h"
#include "nav.h"

int main() {
  const uint LED_PIN = PICO_DEFAULT_LED_PIN;
  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);
  stdio_init_all();

  cam_init();
  uip_init();
  nav_init();

  cam_center();

  while (true) {
    gpio_put(LED_PIN, 1);
    sleep_ms(500);
    gpio_put(LED_PIN, 0);
    sleep_ms(500);
  } 

  cam_disable();

  return 0;
}
