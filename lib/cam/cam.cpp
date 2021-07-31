
#include "Arduino.h"
#include <cam.h>

float pan_angle = 0.0f;
float tilt_angle = 0.0f;

uint32_t cam_pan_ticks_cur_ui32 = 0U;
uint32_t cam_tilt_ticks_cur_ui32 = 0U;

uint32_t cam_pan_ticks_tgt_ui32 = 0U;
uint32_t cam_tilt_ticks_tgt_ui32 = 0U;

uint8_t pwm_clock_cycles_ui8 = (uint8_t)0.050 * ;

void cam_init() {
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

  // Set clock source without prescaler
  TCCR2B = 0x01;

  // Enable interrupts globally
  sei();
  
  //Initialize Timer2
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2 = 0;

  // Set up /1024 prescale
  bitSet(TCCR2B, CS20);
  bitSet(TCCR2B, CS21);
  bitSet(TCCR2B, CS22);

  //OCR2A = 32;      // Sets freq 50hz?    OC2A PIN 11
  OCR2A = 156; //Sets freq 50Hz
  //Timer 2 counts up and down for 312 total counts
  //312 x 1024 x.0625 = 19.968ms 50.08 Hz

  // enable timer compare interrupt
  //bitSet(TIMSK2, OCIE2A);

  OCR2B = 78;//50% duty cycle valid values 1-155
}

void cam_center() {
  
}

void cam_set_pan(float angle_deg_f32) {
  
}

void cam_set_tilt(float angle_deg_f32) {
  
}

void cam_pwm_cycle_end() {
  // Compute next PWM falling edge for both outputs


  // Reset PWM cycle at rising edge
  //digitalWrite(CAM_PAN_PIN, HIGH);
  //digitalWrite(CAM_TILT_PIN, HIGH);
}