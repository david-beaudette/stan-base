/** STAN THE STANDING ROBOT
   Program to control the user inputs
   by David Beaudette   
**/

#ifndef UIP_H
#define UIP_H

#include <stdint.h>

#define UIP_POT_PIN 27 
#define UIP_POT_ADC_CH 1 

void uip_init();
float uip_get_pot_angle(void);
float uip_adc_val2angle(uint16_t adc_val_ui16);

#endif // UIP_H