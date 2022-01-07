#include "pid.h"

void pid_init(PidData *pid,
              const float out_min,
              const float out_max,
              const float dt) {
  pid->err_int = 0.0f;
  pid->dt = dt;
  pid->out_min = out_min;
  pid->out_max = out_max;
}

void pid_set_gains(PidData *pid,
                   const float Ku,
                   const float Tu,
                   const enum PidTuningMethod method) {
  switch (method) {
    case P:
      pid->kp = 0.5f * Ku;
      pid->ki = 0.0f;
      pid->kd = 0.0f;
      break;
    case PI:
      pid->kp = 0.45f * Ku;
      pid->ki = 0.54f * Ku / Tu;
      pid->kd = 0.0f;
      break;
    case PD:
      pid->kp = 0.8f * Ku;
      pid->ki = 0.0f;
      pid->kd = 0.10f * Ku * Tu;
      break;
    case PID_CLASSIC:
      pid->kp = 0.6f * Ku;
      pid->ki = 1.2f * Ku / Tu;
      pid->kd = 0.075f * Ku * Tu;
      break;
    case PID_PESSEN:
      pid->kp = 0.7f * Ku;
      pid->ki = 1.75f * Ku / Tu;
      pid->kd = 0.105f * Ku * Tu;
      break;
    case PID_W_OVERSHOOT:
      pid->kp = 0.3333333433f * Ku;
      pid->ki = 0.6666666865f * Ku / Tu;
      pid->kd = 0.1111111119f * Ku * Tu;
      break;
    case PID_NO_OVERSHOOT:
    default:
      pid->kp = 0.2f * Ku;
      pid->ki = 0.40f * Ku / Tu;
      pid->kd = 0.06666667014f * Ku * Tu;
      break;
  }
}

float pid_step(PidData *pid, const float setpoint, const float inp, const float dinp) {
  bool int_ok = true;
  float err = setpoint - inp;
  float int_cur = pid->err_int + err;
  float out = pid->kp * err - pid->kd * dinp + pid->ki * int_cur;

  /* Check for saturation. In the event of saturation in any one direction,
     inhibit saving the integrator if doing so would deepen the saturation. */
  int_ok = true;
     
  /* Positive saturation? */
  if (out > pid->out_max)
  {
    /* Clamp the output */
    out = pid->out_max;

    /* Error is the same sign? Inhibit integration. */
    if (err > 0)
    {
      int_ok = false;
    }
  }
  /* Repeat for negative sign */
  else if (out < pid->out_min)
  {
    out = pid->out_min;
    
    if (err < 0)
    {
      int_ok = false;
    }
  }
  
  /* Update the integrator if allowed. */
  if (int_ok)
  {
    pid->err_int = int_cur;
  }

  return out;
}