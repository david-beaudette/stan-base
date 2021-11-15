/** STAN THE STANDING ROBOT
   Program to control the user inputs
   by David Beaudette   
**/

#include "hardware/adc.h"
#include "uip.h"
#include "horner.h"


void uip_init() {
  adc_init();

  adc_gpio_init(UIP_POT_PIN);
}

float uip_get_pot_angle(void) {
  adc_select_input(UIP_POT_ADC_CH);
  uint16_t adc_val_ui16 = adc_read();

  return uip_adc_val2angle(adc_val_ui16);
}

float uip_adc_val2angle(uint16_t adc_val_ui16) {

  const float analog2pct_coeff[4] = {
    2.614E-09f,	
    -2.12828E-05f,	
    0.061631117f,	
    9.1288014f
  };
  float adc_val_f32 = (float)adc_val_ui16;

  return horner(4, analog2pct_coeff, adc_val_f32);
}