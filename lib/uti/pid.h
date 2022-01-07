
#ifndef PID_H
#define PID_H

#include "pico/stdlib.h"
#include <stdint.h>

typedef struct PidData
{
  float kp;              /**< Proportional gain constant */
  float ki;              /**< Integral gain constant */
  float kd;              /**< Derivative gain constant */
  float dt;              /**< Derivative gain constant */
  float out_max;         /**< Maximum value */
  float out_min;         /**< Minimum value */
  float err_int;         /**< Current integrator value */
} PidData;

enum PidTuningMethod{
  P,
  PI,
  PD,
  PID_CLASSIC,
  PID_PESSEN,
  PID_W_OVERSHOOT,
  PID_NO_OVERSHOOT
};

void pid_init(PidData *pid,
              const float out_min,
              const float out_max,
              const float dt);

void pid_set_gains(PidData *pid,
                   const float Ku,
                   const float Tu,
                   const enum PidTuningMethod method);

float pid_step(PidData *pid, const float setpoint, const float inp, const float dinp);

#endif // PID_Htypedef struct PidData
