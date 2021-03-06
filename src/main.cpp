#include <Arduino.h>
#include <hal.h>

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(MOTL_STEP, OUTPUT);
  pinMode(MOTL_DIR, OUTPUT);
  pinMode(MOTR_STEP, OUTPUT);
  pinMode(MOTR_DIR, OUTPUT);
  pinMode(MOT_NSLEEP, OUTPUT);

  digitalWrite(LED_BUILTIN, LOW);

  digitalWrite(MOTL_STEP, LOW);
  digitalWrite(MOTL_DIR, LOW);
  digitalWrite(MOTR_STEP, LOW);
  digitalWrite(MOTR_DIR, HIGH);
  digitalWrite(MOT_NSLEEP, HIGH);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(MOTL_STEP, HIGH);
  digitalWrite(MOTR_STEP, HIGH);
  delayMicroseconds(1);
  digitalWrite(MOTR_STEP, LOW);
  digitalWrite(MOTL_STEP, LOW);
  delayMicroseconds(100);
  
}