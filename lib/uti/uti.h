/** STAN THE STANDING ROBOT
   Program to control the user inputs
   by David Beaudette   
**/

#ifndef UTI_H
#define UTI_H

#include <stdint.h>

float map(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

#endif // UTI_H