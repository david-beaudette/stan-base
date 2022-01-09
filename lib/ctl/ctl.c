#include <math.h>

#include "ctl.h"
#include "pid.h"
#include "mtr.h"

PidData pid;
const float out_max = 5.23f * 2.0f * M_PI;
const float out_min = -out_max;
const float dt = 0.02;

const float Ku = 10.0;
const float Tu = 2.0;
enum PidTuningMethod method = PID_TM_P;

void ctl_init(void) {
  mtr_init();

  pid_init(&pid, out_min, out_max, dt);
  pid_set_gains(&pid, Ku, Tu, method);

  return;
}

void ctl_start(void) {
  mtr_enable();
  return;
}

void ctl_stop(void) {
  mtr_disable();
  pid_reset(&pid);
  return;
}

void ctl_set_pitch(const float pitch_cmd, 
                   const float pitch_est) {
  return;
}

void ctl_get_wheel_pos(float *wheel_left_num_rev_f32, 
                       float *wheel_right_num_rev_f32) {
  return;
}
