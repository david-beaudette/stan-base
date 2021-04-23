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

// One step of the pitch complementary filter
void complementary_filter_step(float &pitch, int ax, int ay, int az, int gy);
void nav_set_filter_gain(float acc_coeff_upd);
void nav_set_avg_len(float avg_len_upd);

float nav_get_pitch();

bool nav_get_filter_init();
void nav_reset_filter();

bool nav_init(float dt);

void nav();

#endif // NAV_H