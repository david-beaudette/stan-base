#include "Arduino.h"
#include "ctl.h"

float motor_speed;

void ctl_init() {
  pinMode(MOTL_STEP, OUTPUT);
  pinMode(MOTL_DIR, OUTPUT);
  pinMode(MOTR_STEP, OUTPUT);
  pinMode(MOTR_DIR, OUTPUT);
  pinMode(MOT_NSLEEP, OUTPUT);

  digitalWrite(MOTL_STEP, LOW);
  digitalWrite(MOTL_DIR, LOW);
  digitalWrite(MOTR_STEP, LOW);
  digitalWrite(MOTR_DIR, HIGH);
  digitalWrite(MOT_NSLEEP, HIGH);}

void ctl() {
  digitalWrite(MOTL_STEP, HIGH);
  digitalWrite(MOTR_STEP, HIGH);
  delayMicroseconds(1);
  digitalWrite(MOTR_STEP, LOW);
  digitalWrite(MOTL_STEP, LOW);
}