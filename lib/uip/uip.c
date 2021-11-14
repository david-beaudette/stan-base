/** STAN THE STANDING ROBOT
   Program to control the user inputs
   by David Beaudette   
**/

#include "hardware/adc.h"
#include "uip.h"

void uip_init() {
  adc_init();

  adc_gpio_init(UIP_POT_PIN);
}

float uip_get_pot_angle(void) {
  adc_select_input(UIP_POT_ADC_CH);
  const float adc_count2deg = 3.3f / (1 << 12) * 180.0f;
  uint16_t adc_val_ui16 = adc_read();

  return ((float)adc_val_ui16 * adc_count2deg) - 90.0f;
}