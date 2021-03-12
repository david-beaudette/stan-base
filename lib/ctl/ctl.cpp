#include "ctl.h"

#include <AccelStepper.h>

#include "Arduino.h"

// Define some steppers and the pins the will use
AccelStepper mot_l(AccelStepper::DRIVER, MOTL_STEP, MOTL_DIR);
AccelStepper mot_r(AccelStepper::DRIVER, MOTR_STEP, MOTR_DIR);

float motor_speed;

extern float pitch;

void ctl_init() {
  mot_l.setMaxSpeed(100000.0);
  mot_l.setAcceleration(100000.0);
  mot_l.moveTo(0);

  mot_r.setMaxSpeed(10000.0);
  mot_r.setAcceleration(100000.0);
  mot_r.moveTo(0);

  pinMode(MOT_NSLEEP, OUTPUT);
  digitalWrite(MOT_NSLEEP, HIGH);
}

void ctl() {
  // Change direction at the limits
  if (mot_l.distanceToGo() == 0) mot_l.moveTo((long)pitch * 4);
  if (mot_r.distanceToGo() == 0) mot_r.moveTo((long)-pitch * 4);
  mot_l.run();
  mot_r.run();
}