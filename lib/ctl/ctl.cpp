#include "ctl.h"

#include "Arduino.h"

uint32_t overflow_count_ui32 = 0U;

int32_t motl_pulse_count_i32 = 0;
int32_t motr_pulse_count_i32 = 0;

int32_t motl_dir_cur_i32 = 0;
int32_t motr_dir_cur_i32 = 0;

const float clocktickspersec_f32 = (float)F_CPU;
const float wheelradperstep_f32 = TWO_PI / 200.0f;
const float motorstepsperpulse_f32 = 1.0 / 8.0f;
const float pace2ticksperpol_f32 = 0.5f *
                                  clocktickspersec_f32 * 
                                  wheelradperstep_f32 *
                                  motorstepsperpulse_f32;

const float speed_min_pos_f32 =  1.0f * DEG_TO_RAD;
const float speed_min_neg_f32 = -1.0f * DEG_TO_RAD;

const float speed_max_f32 = 5.23 * TWO_PI; // 5.23 rev/s

const float uint16_max_f32 = (float)UINT16_MAX;
const float uint16_max_inv_f32 = 1.0f / uint16_max_f32;

MotorPulse motl = {0};
MotorPulse motr = {0};

void ctl_init() {
  // Disable interrupts globally
  cli();

  // Reset compare values
  OCR1A = 0U;
  OCR2A = 0U;

  // Enable output compare and overflow interrupts
  TIMSK1 = B00000111;

  // Set timer to operate in normal mode
  TCCR1A = 0x00; 

  // Set clock source without prescaler
  TCCR1B = 0x01; 

  // Enable interrupts globally
  sei();

  // Setup motor control pins
  pinMode(MOTL_STEP_PIN,  OUTPUT);
  pinMode(MOTL_DIR,   OUTPUT);
  pinMode(MOTR_STEP_PIN,  OUTPUT);
  pinMode(MOTR_DIR,   OUTPUT);
  pinMode(MOT_NSLEEP, OUTPUT);

  digitalWrite(MOTL_STEP_PIN,  LOW);
  digitalWrite(MOTL_DIR,   HIGH);
  digitalWrite(MOTR_STEP_PIN,  LOW);
  digitalWrite(MOTR_DIR,   LOW);

  // Always start in sleep mode
  digitalWrite(MOT_NSLEEP, LOW);
}

void ctl_set_motor_speed(const float speed_left_f32, 
                         const float speed_right_f32) {
  float speed_f32 = 0.0f;
  float pace_f32 = INFINITY;
  float pol_ticks_f32 = INFINITY;
  uint32_t next_compare_val_ui32;
  bool motl_stopped_b = true;

  if(speed_left_f32 > speed_min_pos_f32) {
    digitalWrite(MOTL_DIR, HIGH);
    motl_dir_cur_i32 = 1;
    motl_stopped_b = false;
  }
  else if(speed_left_f32 < speed_min_neg_f32) {
    digitalWrite(MOTL_DIR, LOW);
    motl_dir_cur_i32 = -1;
    motl_stopped_b = false;
  }
  else {
    motl_dir_cur_i32 = 0;
  }
  if(!motl_stopped_b) {
    // Limit to maximum speed
    speed_f32 = fmin(fabs(speed_left_f32), speed_max_f32);

    // Compute pace (inverse of speed) and corresponding 
    // number of ticks per pulse polarity
    pace_f32 = 1.0f / speed_f32;
    pol_ticks_f32 = pace_f32 * pace2ticksperpol_f32;

#if 1
    Serial.print("Speed update: speed_f32 ");
    Serial.print(speed_f32);
    Serial.print(", pol ticks tgt ");
    Serial.print(pol_ticks_f32);
    Serial.println(".");
#endif
  }
  else {
    digitalWrite(MOTL_STEP_PIN, LOW);
    Serial.print("Left motor is stopped (current pos ");
    Serial.print((float)(motl_pulse_count_i32 >> 3) * 1.8f);
    Serial.println(" deg).");
    Serial.print("Right motor is stopped (current pos ");
    Serial.print((float)(motr_pulse_count_i32 >> 3) * 1.8f);
    Serial.println(" deg).");
  }    

  // If none of the motors run, controller should sleep
  if(motl_stopped_b && motl_stopped_b) {
    digitalWrite(MOT_NSLEEP, LOW);
  }
  else {
    digitalWrite(MOT_NSLEEP, HIGH);
  }
  // Pause timer to update values
  TIMER1_DOWN;

  motl.pol_ticks_ui32 = (uint32_t)pol_ticks_f32;
  next_compare_val_ui32 = (uint32_t)TCNT1;
  next_compare_val_ui32 += motl.pol_ticks_ui32;
  motl.num_ovf_tgt_ui32 = (next_compare_val_ui32 > 0? (next_compare_val_ui32 - 1): 0U) >> 16;
  OCR1A = next_compare_val_ui32 - (motl.num_ovf_tgt_ui32 << 16);

  TIMER1_UP;
}
void ctl_reset_motor_pos() {
  motl_pulse_count_i32 = 0;
  motr_pulse_count_i32 = 0;
}

// Interrupt service run when Timer/Counter1 OVERFLOW
ISR(TIMER1_OVF_vect) { 
  ++overflow_count_ui32; 

  // Count overflows for each motor
  ++motl.num_ovf_cur_ui32; 
  ++motr.num_ovf_cur_ui32; 
}

// Interrupt service run when Timer/Counter1 reaches OCR1A
ISR(TIMER1_COMPA_vect) 
{   
  // Check if desired period has elapsed and motor is running
  if((motl.num_ovf_cur_ui32 >= motl.num_ovf_tgt_ui32) &&
     (motl_dir_cur_i32 != 0)) {
    // Compute next transition
    uint32_t next_compare_val_ui32 = (uint32_t)OCR1A;
    next_compare_val_ui32 += motl.pol_ticks_ui32 - 5;
    motl.num_ovf_tgt_ui32 = next_compare_val_ui32 >> 16;
    OCR1A = next_compare_val_ui32 - (motl.num_ovf_tgt_ui32 << 16);

    // Toggle output pin and reset the overflow counter
    motl.pulse_pol_cur_b = !motl.pulse_pol_cur_b;
    if(motl.pulse_pol_cur_b) {
      PORTB |= MOTL_STEP_SET_ORMASK;
    }
    else {
      PORTB &= MOTL_STEP_CLR_ANDMASK;
    }
    // Update the pulse count on low to high transition
    if(motl.pulse_pol_cur_b) {
      motl_pulse_count_i32 += motl_dir_cur_i32;
    }
    motl.num_ovf_cur_ui32 = 0U;
  }       
}

// Interrupt service run when Timer/Counter1 reaches OCR1B
ISR(TIMER1_COMPB_vect) 
{   
  motr.pulse_pol_cur_b = !motr.pulse_pol_cur_b;
  if(motr.pulse_pol_cur_b) {
    PORTB |= MOTR_STEP_SET_ORMASK;
    ++motr_pulse_count_i32;
  }
  else {
    PORTB &= MOTR_STEP_CLR_ANDMASK;
  }
}