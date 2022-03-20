/** STAN THE STANDING ROBOT
   Program to interface the Bosch LCD 9DoF sensor
   by David Beaudette

   Adapted from Adafruit_LCD Arduino Library
   https://github.com/adafruit/Adafruit_LCD
   See LICENSE file for permission details
   Copyright (c) 2018 Adafruit Industries
**/

#ifndef LCD_H
#define LCD_H

#include <stdint.h>
#include "pico/stdlib.h"

#define LCD_I2C_PORT i2c1

bool lcd_init(void);


#endif  // LCD_H