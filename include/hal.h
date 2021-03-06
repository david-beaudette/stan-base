#ifndef HAL_H
#define  HAL_H
#include "Arduino.h"

const int  MOTL_STEP  = 4;
const int  MOTL_DIR   = 5;
const int  MOTR_STEP  = 6;
const int  MOTR_DIR   = 7;
const int  MOT_NSLEEP = 9;
const int  BAT_VOLT   = A6; // Battery voltage
const int  GYRO_VOLT  = A7; // Gyroscope

#endif // HAL_H