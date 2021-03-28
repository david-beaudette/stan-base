/** STAN THE STANDING ROBOT
   Battery management function
   by David Beaudette   
**/

#include "bat.h"

const float analog2volt = 12.56f / 968.0f;

bool check_bat_level(void) {
  return (analogRead(BAT_PIN) > BAT_LIM);
}

bool check_bat_level_verbose(void) {
  int bat_level = analogRead(BAT_PIN);
  Serial.print("Battery level: ");
  Serial.print((float)bat_level * analog2volt);
  Serial.println(" V.");
  return (bat_level > BAT_LIM);
}