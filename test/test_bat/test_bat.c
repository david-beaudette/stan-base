/** STAN THE STANDING ROBOT
   Program to test the camera pan pan servo control function
   by David Beaudette
**/

// Libraries
#include <stdio.h>
#include "pico/stdlib.h"
#include "bat.h"

int main() {
  const uint LED_PIN = PICO_DEFAULT_LED_PIN;
  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);
  stdio_init_all();

  bat_init();

  uint16_t bat_adc_val_ui16;

  while (true) {
    gpio_put(LED_PIN, 1);
    sleep_ms(500);
    gpio_put(LED_PIN, 0);
    sleep_ms(500);
    bat_adc_val_ui16 = bat_update();
    printf("Battery ADC reading %d.\n", bat_adc_val_ui16);
    bat_show_voltage();
    bat_show_state_of_charge();

    if (bat_check_level()) {
      printf("Battery level Ok.\n");
    } 
    else {
      printf("Battery level too low.\n");
    }
  }

  return 0;
}