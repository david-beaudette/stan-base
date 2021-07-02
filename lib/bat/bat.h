/** STAN THE STANDING ROBOT
   Battery management function
   by David Beaudette   
**/

#ifndef BAT_H
#define BAT_H

// Libraries
#include "Arduino.h"

#define BAT_PIN A6 
#define BAT_LIM 867 // 25% of 3S LiPo -> 11.25 VDC 

bool bat_check_level(void);
void bat_show_voltage(void);
void bat_show_state_of_charge(void);
float bat_get_state_of_charge(void);
float bat_get_voltage(void);
float bat_get_last_voltage(void);

#endif // BAT_H