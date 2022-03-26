/** STAN THE STANDING ROBOT
   Program to control the user inputs
   by David Beaudette   
**/

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "bat.h"
#include "uti.h"


const float analog2volt = 0.00343324f;
const float analog_offset = -0.29182561f;
const float analog2pct_coeff[4] = {
  5.34563E-07,	
  -0.005970516,	
  22.34677759,	
  -27930.96502
};

uint16_t bat_adc_val_prev_ui16 = 0;

void bat_init() {
  adc_init();

  adc_gpio_init(BAT_PIN);
}

uint16_t bat_update(void) {
  adc_select_input(BAT_ADC_CH);
  bat_adc_val_prev_ui16 = adc_read();

  return bat_adc_val_prev_ui16;
}

bool bat_check_level(void) {
  return (bat_adc_val_prev_ui16 > BAT_LIM);
}

float bat_get_voltage(void) {
  return ((float)bat_adc_val_prev_ui16 * analog2volt + analog_offset);
}

void bat_show_voltage(void) {
  printf("Battery voltage: %0.2f V.\n", bat_get_voltage());
}

float bat_get_state_of_charge(void) {
  float bat_level_f32 = (float)bat_adc_val_prev_ui16;
  float soc_f32 = horner(4, analog2pct_coeff, bat_level_f32);
  if(soc_f32 > 100.0f) {
    soc_f32 = 100.0f;
  }
  else if(soc_f32 < 0.0f) {
    soc_f32 = 0.0f;
  }
  return soc_f32;
}

void bat_show_state_of_charge(void) {
  printf("Battery state of charge: %0.1f %%.\n", bat_get_state_of_charge());
}

