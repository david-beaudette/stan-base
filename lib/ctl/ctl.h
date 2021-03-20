/** STAN THE STANDING ROBOT
   Control function
   by David Beaudette   
**/

#ifndef CTL_H
#define CTL_H

#define  MOTL_STEP   4
#define  MOTL_DIR    5
#define  MOTR_STEP   6
#define  MOTR_DIR    7
#define  MOT_NSLEEP  9

extern float motor_speed;

void ctl_init();

void ctl();

void ctl_execute();

#endif // CTL_H