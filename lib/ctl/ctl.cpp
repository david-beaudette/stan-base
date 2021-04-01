
#include "Arduino.h"
#include "ctl.h"

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

volatile MotorPulse motl = {0};
volatile MotorPulse motr = {0};

void ctl_init() {
  // Disable interrupts globally
  cli();

  // Reset compare values
  OCR1A = 0U;
  OCR2A = 0U;

  // Enable output compare interrupts
  TIMSK1 = B00000110;

  // Set timer to operate in normal mode
  TCCR1A = 0x00; 

  // Set clock source without prescaler
  TCCR1B = 0x01; 

  // Enable interrupts globally
  sei();

  // Setup motor control pins
  pinMode(MOTL_STEP_PIN,  OUTPUT);
  pinMode(MOTL_DIR,       OUTPUT);
  pinMode(MOTR_STEP_PIN,  OUTPUT);
  pinMode(MOTR_DIR,       OUTPUT);
  pinMode(MOT_NSLEEP,     OUTPUT);

  digitalWrite(MOTL_STEP_PIN,  LOW);
  digitalWrite(MOTL_DIR,       HIGH);
  digitalWrite(MOTR_STEP_PIN,  LOW);
  digitalWrite(MOTR_DIR,       LOW);

  // Always start in sleep mode
  digitalWrite(MOT_NSLEEP, LOW);

  // Set step pin manipulation masks
  motl.clrandmask_cst_ui8 = MOTL_STEP_CLR_AND_MASK;
  motr.clrandmask_cst_ui8 = MOTR_STEP_CLR_AND_MASK;
  motl.setormask_cst_ui8  = MOTL_STEP_SET_OR_MASK;
  motr.setormask_cst_ui8  = MOTR_STEP_SET_OR_MASK;
}

void ctl_set_motor_speed(const float speed_left_f32, 
                         const float speed_right_f32) {
  float speed_f32 = 0.0f;
  float pace_f32 = 0.0f;
  float pol_ticks_f32 = 0.0f;
  volatile uint32_t next_compare_val_ui32;
  volatile uint32_t num_ovf_tgt_ui32;
  bool motl_stopped_b;
  bool motr_stopped_b = true;

  if(speed_left_f32 > speed_min_pos_f32) {
    digitalWrite(MOTL_DIR, HIGH);
    motl.dir_cur_i32 = 1;
    motl_stopped_b = false;
  }
  else if(speed_left_f32 < speed_min_neg_f32) {
    digitalWrite(MOTL_DIR, LOW);
    motl.dir_cur_i32 = -1;
    motl_stopped_b = false;
  }
  else {
    motl.dir_cur_i32 = 0;
    PORTB &= motl.clrandmask_cst_ui8;
    motl_stopped_b = true;
  }
  if(!motl_stopped_b) {
    // Limit to maximum speed
    speed_f32 = fmin(fabs(speed_left_f32), speed_max_f32);

    // Compute pace (inverse of speed) and corresponding 
    // number of ticks per pulse polarity
    pace_f32 = 1.0f / speed_f32;
    pol_ticks_f32 = pace_f32 * pace2ticksperpol_f32;


    // Pause timer to update values
    TIMER1_DOWN;

    // Clear pending interrupts
    TIFR1 = 0XFF;

    motl.pol_ticks_tgt_ui32 = (uint32_t)pol_ticks_f32;
    if(motl.pol_ticks_cur_ui32 >=  motl.pol_ticks_tgt_ui32) {
      // Pulse polarity change due now
      ctl_toggle_motor_pin(&motl);
      motl.pol_ticks_cur_ui32 = 0U;
    }

    motl.timer_val_prev_ui16 = TCNT1;

    next_compare_val_ui32 = (uint32_t)motl.timer_val_prev_ui16;
    next_compare_val_ui32 += (motl.pol_ticks_tgt_ui32 - 
                              motl.pol_ticks_cur_ui32);
    num_ovf_tgt_ui32 = next_compare_val_ui32 >> 16;
    motl.cmp_val_cur_ui16 = (uint16_t)(next_compare_val_ui32 - (num_ovf_tgt_ui32 << 16));
    OCR1A = motl.cmp_val_cur_ui16;

#if 0
    Serial.print("Speed update:\n");
    Serial.print("  speed_f32          ");
    Serial.println(speed_f32);
    Serial.print("  pol ticks tgt (f32) ");
    Serial.println(pol_ticks_f32);
    Serial.print("  pol ticks tgt       ");
    Serial.println(motl.pol_ticks_tgt_ui32);
    Serial.print("  pol ticks cur       ");
    Serial.println(motl.pol_ticks_cur_ui32);
    Serial.print("  current timer value ");
    Serial.println(motl.timer_val_prev_ui16);
    Serial.print("  next compare value  ");
    Serial.println(next_compare_val_ui32);
    Serial.print("  num overflows       ");
    Serial.println(num_ovf_tgt_ui32);
    Serial.print("  OCR1A value         ");
    Serial.println(OCR1A);
    Serial.print("  TIMSK value         ");
    Serial.println(TIMSK1);
    Serial.println(" ");
#endif
    TIMER1_UP;
  }
  else {
    digitalWrite(MOTL_STEP_PIN, LOW);
    Serial.print("Left motor is stopped (current pos ");
    Serial.print((float)(motl.pulse_count_cur_i32 >> 3) * 1.8f);
    Serial.println(" deg).");
    Serial.print("Right motor is stopped (current pos ");
    Serial.print((float)(motr.pulse_count_cur_i32 >> 3) * 1.8f);
    Serial.println(" deg).");
    motl.pol_ticks_cur_ui32 = UINT32_MAX;
  }    

  // If none of the motors run, controller should sleep
  if(motl_stopped_b && motr_stopped_b) {
    digitalWrite(MOT_NSLEEP, LOW);
  }
  else {
    digitalWrite(MOT_NSLEEP, HIGH);
  }
}

void ctl_reset_motor_pos() {
  motl.pulse_count_cur_i32 = 0;
  motr.pulse_count_cur_i32 = 0;
}

void ctl_toggle_motor_pin(volatile MotorPulse *mot) {
  mot->pol_ticks_cur_ui32 = 0U;
  mot->pulse_pol_cur_b = !mot->pulse_pol_cur_b;
  if(mot->pulse_pol_cur_b) {
    PORTB |= mot->setormask_cst_ui8;
  }
  else {
    PORTB &= mot->clrandmask_cst_ui8;
  }
  // Update the pulse count on low to high transition
  if(mot->pulse_pol_cur_b) {
    mot->pulse_count_cur_i32 += mot->dir_cur_i32;
  }
}

// Interrupt service run when Timer/Counter1 reaches OCR1A
ISR(TIMER1_COMPA_vect) 
{   
  uint32_t next_compare_val_ui32;
  uint32_t num_ovf_tgt_ui32;

  if(motl.dir_cur_i32 != 0) {
    // Compute time since last period check
    if(motl.timer_val_prev_ui16 >= motl.cmp_val_cur_ui16) {
      motl.pol_ticks_cur_ui32 += ((uint32_t)UINT16_MAX -
                                  (uint32_t)motl.timer_val_prev_ui16) + 
                                 ((uint32_t)motl.cmp_val_cur_ui16 + 1U);
    }
    else {
      motl.pol_ticks_cur_ui32 += (uint32_t)motl.cmp_val_cur_ui16 - 
                                 (uint32_t)motl.timer_val_prev_ui16;
    }
    motl.timer_val_prev_ui16 = motl.cmp_val_cur_ui16;

    // Check if desired period has elapsed and motor is running
    if(motl.pol_ticks_cur_ui32 >= motl.pol_ticks_tgt_ui32) {
      // Pulse polarity change due now
      ctl_toggle_motor_pin(&motl);
    }
    next_compare_val_ui32 = (uint32_t)motl.timer_val_prev_ui16;
    next_compare_val_ui32 += (motl.pol_ticks_tgt_ui32 - 
                              motl.pol_ticks_cur_ui32);
    num_ovf_tgt_ui32 = next_compare_val_ui32 >> 16;
    motl.cmp_val_cur_ui16 = (uint16_t)(next_compare_val_ui32 - (num_ovf_tgt_ui32 << 16));
    OCR1A = motl.cmp_val_cur_ui16;
  }
  else {
    PORTB &= motl.clrandmask_cst_ui8;
  }
}

// Interrupt service run when Timer/Counter1 reaches OCR1B
ISR(TIMER1_COMPB_vect) 
{   
  motr.pulse_pol_cur_b = !motr.pulse_pol_cur_b;
  if(motr.pulse_pol_cur_b) {
    PORTB |= motr.setormask_cst_ui8;
    ++motr.pulse_count_cur_i32;
  }
  else {
    PORTB &= motr.clrandmask_cst_ui8;
  }
}