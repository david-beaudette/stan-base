/** STAN THE STANDING ROBOT
   Battery management function
   by David Beaudette   
**/

#include "bat.h"
#include "horner.h"

const float analog2volt = 12.56f / 968.0f;
const float analog2pct_coeff[4] = {
  2.88555E-05,	
  -0.083329949,	
  80.6627318,	
  -26074.29762
};

int bat_level_last = 0;

bool bat_check_level(void) {
  return (analogRead(BAT_PIN) > BAT_LIM);
}

float bat_get_voltage(void) {
  int bat_level = analogRead(BAT_PIN);
  bat_level_last = bat_level;
  return ((float)bat_level * analog2volt);
}

float bat_get_last_voltage(void) {
  return ((float)bat_level_last * analog2volt);
}

void bat_show_voltage(void) {
  int bat_level = analogRead(BAT_PIN);
  bat_level_last = bat_level;
  Serial.print("Battery voltage: ");
  Serial.print((float)bat_level * analog2volt);
  Serial.println(" V.");
}

void bat_show_state_of_charge(void) {
  Serial.print("Battery state of charge: ");
  Serial.print(bat_get_state_of_charge(), 1);
  Serial.println("%.");
}

float bat_get_state_of_charge(void) {
  int bat_level = analogRead(BAT_PIN);
  float bat_level_f32 = (float)bat_level;
  bat_level_last = bat_level;
  float soc_f32 = horner(4, analog2pct_coeff, bat_level_f32);
  if(soc_f32 > 100.0f) {
    soc_f32 = 100.0f;
  }
  else if(soc_f32 < 0.0f) {
    soc_f32 = 0.0f;
  }
  return soc_f32;
}