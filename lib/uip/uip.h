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
void uip_disable();


#endif // UIP_H