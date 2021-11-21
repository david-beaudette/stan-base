/** STAN THE STANDING ROBOT
   Program to test the Bosch BNO055 9DoF sensor interface
   by David Beaudette
**/

// Libraries
#include <stdio.h>

#include "bno055.h"
#include "pico/stdlib.h"
#include "uip.h"

int main() {
  const uint LED_PIN = PICO_DEFAULT_LED_PIN;
  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);
  stdio_init_all();

  uip_init();

  // Initialise sensor in accelerometer only mode
  bno055_init();  
  accel_init();

  float acc[3];

  // Infinite Loop
  while (1) {
    bno055_get_accel(acc);

    // Print to serial monitor
    printf("X: %6.2f    Y: %6.2f    Z: %6.2f\n", acc[0], acc[1], acc[2]);
    sleep_ms(300);
  }

  return 0;
}