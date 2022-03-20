/** STAN THE STANDING ROBOT
   Program to test the LCD2004 module interface
   by David Beaudette
**/

// Libraries
#include <stdio.h>

#include "lcd.h"
#include "pico/stdlib.h"
#include "uip.h"

int main() {
  const uint LED_PIN = PICO_DEFAULT_LED_PIN;
  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);
  stdio_init_all();

  uip_init();

  // Initialise sensor in accelerometer only mode
  while (!lcd_init()) {
    printf("LCD init failed, retrying in 2 s\n");
    sleep_ms(2000);
  }

  // Infinite Loop
  while (1) {
    //

    sleep_ms(500);
  }

  return 0;
}