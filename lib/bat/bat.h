/** STAN THE STANDING ROBOT
   Program to control the user inputs
   by David Beaudette   
**/

#ifndef BAT_H
#define BAT_H

#include <stdint.h>
#include <stdbool.h>

#define BAT_PIN 26 
#define BAT_ADC_CH 0 

#define BAT_LIM 867 // 25% of 3S LiPo -> 11.25 VDC 

void bat_init(void);
uint16_t bat_update(void);

bool bat_check_level(void);

float bat_get_voltage(void);
void bat_show_voltage(void);

float bat_get_state_of_charge(void);
float bat_get_filt_state_of_charge(void);
void bat_show_state_of_charge(void);

#endif // BAT_H