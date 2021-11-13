/** STAN THE STANDING ROBOT
   Program to control the camera pan tilt servos
   by David Beaudette   
**/

#ifndef CAM_H
#define CAM_H

// On the RP2040, any even pin and the next odd pin can be used in the same PWM slice
#define  CAM_PAN_PIN   7 
#define  CAM_TILT_PIN  6  

void cam_init();
void cam_center();
float cam_set_pan(float angle_deg_f32);
float cam_set_tilt(float angle_deg_f32);
void cam_pwm_cycle_end();

float angle2ticks(volatile uint8_t *num_oci_tgt_ui8,
                  volatile uint8_t *oci_val_ui8,
                  const float angle_deg_f32,
                  const int32_t pw_min_ticks_i32,
                  const int32_t pw_max_ticks_i32);

float cam_get_pan();
float cam_get_tilt();

#endif // CAM_H

typedef struct{
    int pin;
    int slice;
    int channel;
    int value;
    bool enabled;
    int pulseMax;
    int pulseMin;
    int delay;
    float lastDegree;
} PWM;


void setServo(PWM *pin,float degree);
PWM enableServo(int pin);
void disableServo(PWM myServo);
int calculateDelay(PWM myServo,int mSecPerDegree);
void moveDelay(int myDelay,float startDeg,float endDeg);