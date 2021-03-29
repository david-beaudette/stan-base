#include "Arduino.h"
#include "ctl.h"

void setup(void) {
  pinMode(MOTL_STEP_PIN, OUTPUT);
  cli();
}

void loop() {
  PORTB = 0;
  PORTB = 2;
  PORTB = 0;
  PORTB = 2;
}