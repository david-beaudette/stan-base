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

bool check_bat_level(void);
bool check_bat_level_verbose(void);

#endif // BAT_H