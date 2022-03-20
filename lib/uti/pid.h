
#ifndef PID_H
#define PID_H

#include "pico/stdlib.h"
#include <stdint.h>

typedef struct PidData
{
  float kp;              /**< Proportional gain constant */
  float ki;              /**< Integral gain constant */
  float kd;              /**< Derivative gain constant */
  float dt;              /**< Controller sample time (s) */
  float out_max;         /**< Maximum value */
  float out_min;         /**< Minimum value */
  float err_int;         /**< Current integrator value */
} PidData;

enum PidTuningMethod{
  PID_TM_P,
  PID_TM_PI,
  PID_TM_PD,
  PID_TM_CLASSIC,
  PID_TM_PESSEN,
  PID_TM_W_OVERSHOOT,
  PID_TM_NO_OVERSHOOT
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
void pid_reset(PidData *pid);

#endif // PID_Htypedef struct PidData
