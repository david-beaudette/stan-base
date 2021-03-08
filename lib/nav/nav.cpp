#include "Arduino.h"
#include "nav.h"

// Complementary filter
float pitch_acc;
float acc_coeff = 0.02f;
float gyr_coeff = 0.98f;
float delta_t = 0.01f;

// One step of the pitch complementary filter
void complementary_filter_step(float &pitch, int ax, int ay, int az, int gy) {
  long squaresum = (long)ay * ay + (long)az * az;
  pitch += ((-gy / 32.8f) * (delta_t / 1000000.0f));
  pitch_acc = atan(ax / sqrt(squaresum)) * RAD_TO_DEG;
  pitch = gyr_coeff * pitch + acc_coeff * pitch_acc;
}