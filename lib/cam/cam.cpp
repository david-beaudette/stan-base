
#include "Arduino.h"
#include <cam.h>

// Current commanded angle in millidegrees
float pan_angle_cmd_deg_f32 = 0.0f;
float tilt_angle_cmd_deg_f32 = 0.0f;

// Angular displacement range of servos
const int32_t angle_min_mdeg_i32 = -90000;
const int32_t angle_max_mdeg_i32 =  90000;

// Pulse width range in nanoseconds
// Number of ns per timer count
#define tick2ns 2000

const int32_t pw_min_ticks_i32 =  100000 / tick2ns;
const int32_t pw_max_ticks_i32 = 3500000 / tick2ns;

uint8_t cam_pan_num_oci_cur_ui8 = 0U;
uint8_t cam_tilt_num_oci_cur_ui8 = 0U;

uint8_t cam_pan_oci_val_ui8 = 0U;
uint8_t cam_tilt_oci_val_ui8 = 0U;

uint8_t cam_pan_num_oci_tgt_ui8 = 0U;
uint8_t cam_tilt_num_oci_tgt_ui8 = 0U;

uint8_t cam_pwm_num_ovf_cur_ui8 = 0U;

// With the prescaler set at 32, 39 timer 
// overflows yield a 0.0199680 s period, 
// 0.2% off from nominal PWM 20 ms period
// Good enough since only the pulse width is
// usually relevant for servo control
const uint8_t cam_pwm_num_ovf_tgt_ui8 = 39U;

void cam_init() {

  // Digital pins setup
  pinMode(CAM_PAN_PIN, OUTPUT);
  pinMode(CAM_TILT_PIN, OUTPUT);
  digitalWrite(CAM_PAN_PIN, HIGH);
  digitalWrite(CAM_TILT_PIN, HIGH);

  // Disable interrupts globally
  cli();

  // Reset compare values
  OCR2A = 0U;
  OCR2B = 0U;

  // Enable output compare interrupts and overflow interrupt
  TIMSK2 = B00000111;

  // Set timer to operate in normal mode
  TCCR2A = 0x00;
  TCNT2 = 0;

  // Set clock source with 32 cycles prescaler
  TCCR2B = 0x03;

  // Set initial target pulse width 
  // to 1500 ms (servo centered)
  OCR2A = 127; 
  OCR2B = 127;

  // Enable interrupts globally
  sei();
  
}

void cam_center() {
  pan_angle_cmd_deg_f32 = 0;
  tilt_angle_cmd_deg_f32 = 0;
}

void cam_set_pan(float angle_deg_f32) {

  // Compute the corresponding pulse width
  pan_angle_cmd_deg_f32 = angle2ticks(&cam_pan_num_oci_tgt_ui8,
                                      &cam_pan_oci_val_ui8,
                                      angle_deg_f32);
}

void cam_set_tilt(float angle_deg_f32) {

  // Compute the corresponding pulse width
  tilt_angle_cmd_deg_f32 = angle2ticks(&cam_tilt_num_oci_tgt_ui8,
                                       &cam_tilt_oci_val_ui8,
                                       angle_deg_f32);
  
}
float angle2ticks(uint8_t *num_oci_tgt_ui8,
                  uint8_t *oci_val_ui8,
                  const float angle_deg_f32) {

  int32_t angle_mdeg_i32 = (int32_t)(angle_deg_f32 * 1000.0f);
  int32_t pw_len_ticks = map(angle_mdeg_i32, 
                             angle_min_mdeg_i32, 
                             angle_max_mdeg_i32, 
                             pw_min_ticks_i32, 
                             pw_max_ticks_i32);

  *num_oci_tgt_ui8 = (uint8_t)(pw_len_ticks >> 8);
  *oci_val_ui8 = (uint8_t)(pw_len_ticks & 255);
  
  // Reverse the computation to verify
  pw_len_ticks = ((int32_t)(*num_oci_tgt_ui8) << 8) + 
                  (int32_t)(*oci_val_ui8);

  angle_mdeg_i32 = map(pw_len_ticks, 
                      pw_min_ticks_i32, 
                      pw_max_ticks_i32, 
                      angle_min_mdeg_i32, 
                      angle_max_mdeg_i32); 

  float angle_clc_deg_f32 = (float)angle_mdeg_i32 * 0.001f;

  return angle_clc_deg_f32;
}

float cam_get_pan() {
  return pan_angle_cmd_deg_f32;
}

float cam_set_tilt() {
  return tilt_angle_cmd_deg_f32;
}

// Interrupt service run when Timer/Counter2 reaches OCR2A
ISR(TIMER2_COMPA_vect)
{
  ++cam_pan_num_oci_cur_ui8;
  // Check for end of pulse period
  if(cam_pan_num_oci_cur_ui8 == cam_pan_num_oci_tgt_ui8) {
    // End PWM pulse
    digitalWrite(CAM_PAN_PIN, LOW);
  }
}

// Interrupt service run when Timer/Counter2 reaches OCR2B
ISR(TIMER2_COMPB_vect)
{
  ++cam_tilt_num_oci_cur_ui8;
  // Check for end of pulse period
  if(cam_tilt_num_oci_cur_ui8 >= cam_tilt_num_oci_tgt_ui8) {
    // End PWM pulse
    digitalWrite(CAM_TILT_PIN, LOW);
  }
}

// Interrupt service run when Timer/Counter2 overflows
ISR(TIMER2_OVF_vect) {
  ++cam_pwm_num_ovf_cur_ui8;
  // Check for end of PWM period
  if(cam_pwm_num_ovf_cur_ui8 >= cam_pwm_num_ovf_tgt_ui8) {
    // Reset PWM cycle at rising edge
    digitalWrite(CAM_PAN_PIN, HIGH);
    digitalWrite(CAM_TILT_PIN, HIGH);

    // Reset all counters
    cam_pwm_num_ovf_cur_ui8 = 0U;
    cam_pan_num_oci_cur_ui8 = 0U;
    cam_tilt_num_oci_cur_ui8 = 0U;

    // Update counter compare values
    OCR2A = cam_pan_oci_val_ui8;
    OCR2B = cam_tilt_oci_val_ui8;
  }
}