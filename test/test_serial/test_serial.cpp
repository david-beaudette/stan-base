/** STAN THE STANDING ROBOT
   Program to test the serial interface
   by David Beaudette   
**/

// Libraries
#include "Arduino.h"

void setup() {
  Serial.begin(115200);

  Serial.println("Serial test.");
}

void loop() {
  // Test finished; blink
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
} 
