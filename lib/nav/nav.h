/** STAN THE STANDING ROBOT
   Navigation function
   by David Beaudette   
**/

#ifndef NAV_H
#define NAV_H

extern float accel_isr_count;
extern bool process_data_b;
extern bool filter_init_b;

extern int16_t ax, ay, az;
extern int16_t ay_offset, az_offset;
extern float ay_sum, az_sum;

// Gyroscope 
extern int16_t gy;  
extern int16_t gy_bias;  
extern float gy_sum;  

extern float pitch;

// One step of the pitch complementary filter
void complementary_filter_step(float &pitch, int ax, int ay, int az, int gy);

bool nav_init();

void nav();

#endif // NAV_H