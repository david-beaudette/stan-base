/** STAN THE STANDING ROBOT
   Program to interface the Bosch BNO055 9DoF sensor
   by David Beaudette

   Adapted from Adafruit_BNO055 Arduino Library
   https://github.com/adafruit/Adafruit_BNO055
   See LICENSE file for permission details
   Copyright (c) 2018 Adafruit Industries
**/

#include <math.h>
#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "lcd.h"

static int LCD_I2C_ADDR = 0x27;

bool lcd_init(void) {

  // Setup I2C port
  i2c_init(LCD_I2C_PORT, 400 * 1000);
  gpio_set_function(4, GPIO_FUNC_I2C);
  gpio_set_function(5, GPIO_FUNC_I2C);
  gpio_pull_up(4);
  gpio_pull_up(5);

  // Add a short delay for the BNO005 to boot up
  // POR time is typ. 400 ms + 50% margin
  sleep_ms(600);  
  return true;
}

