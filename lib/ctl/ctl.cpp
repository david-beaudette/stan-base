
#include "Arduino.h"
#include "ctl.h"

const float pulsesperstep_f32 = 2.0f;
const float stepperrev_f32 = 200.0f;
const float clocktickspersec_f32 = (float)F_CPU;
const float wheelradperstep_f32 = TWO_PI / stepperrev_f32;
const float motorstepsperpulse_f32 = 1.0 / pulsesperstep_f32;
const float pace2ticksperpol_f32 = 0.5f *
                                   clocktickspersec_f32 *
                                   wheelradperstep_f32 *
                                   motorstepsperpulse_f32;

const float speed_min_pos_f32 = 1.0f * DEG_TO_RAD;
const float speed_min_neg_f32 = -1.0f * DEG_TO_RAD;

const float speed_max_f32 = 5.23 * TWO_PI; // 5.23 rev/s

const float rev_per_pulse_f32 = 1.0f / (stepperrev_f32 * pulsesperstep_f32);

volatile MotorPulse motl = {0};
volatile MotorPulse motr = {0};

void ctl_init()
{
  // Disable interrupts globally
  cli();

  // Reset compare values
  OCR1A = 0U;
  OCR1B = 0U;

  // Enable output compare interrupts
  TIMSK1 = B00000110;

  // Set timer to operate in normal mode
  TCCR1A = 0x00;

  // Set clock source without prescaler
  TCCR1B = 0x01;

  // Enable interrupts globally
  sei();

  // Setup motor control pins
  pinMode(MOTL_STEP_PIN, OUTPUT);
  pinMode(MOTL_DIR, OUTPUT);
  pinMode(MOTR_STEP_PIN, OUTPUT);
  pinMode(MOTR_DIR, OUTPUT);
  pinMode(MOT_NSLEEP, OUTPUT);

  digitalWrite(MOTL_STEP_PIN, LOW);
  digitalWrite(MOTL_DIR, HIGH);
  digitalWrite(MOTR_STEP_PIN, LOW);
  digitalWrite(MOTR_DIR, LOW);

  // Always start in sleep mode
  digitalWrite(MOT_NSLEEP, LOW);

  // Set step pin manipulation masks
  motl.step_clrandmask_cst_ui8 = MOTL_STEP_CLR_AND_MASK;
  motr.step_clrandmask_cst_ui8 = MOTR_STEP_CLR_AND_MASK;
  motl.step_setormask_cst_ui8 = MOTL_STEP_SET_OR_MASK;
  motr.step_setormask_cst_ui8 = MOTR_STEP_SET_OR_MASK;

  motl.dir_clrandmask_cst_ui8 = MOTL_DIR_CLR_AND_MASK;
  motr.dir_clrandmask_cst_ui8 = MOTR_DIR_CLR_AND_MASK;
  motl.dir_setormask_cst_ui8 = MOTL_DIR_SET_OR_MASK;
  motr.dir_setormask_cst_ui8 = MOTR_DIR_SET_OR_MASK;

  motl.dir_sign_i32 = 1;
  motr.dir_sign_i32 = -1;

  // Set compare register addresses
  motl.compare_register_ui8 = 0x88;
  motr.compare_register_ui8 = 0x8A;
}

void ctl_enable_motors(void)
{
  digitalWrite(MOT_NSLEEP, HIGH);
}

void ctl_disable_motors(void)
{
  digitalWrite(MOT_NSLEEP, LOW);
}

void ctl_set_motor_speed(const float speed_left_f32,
                         const float speed_right_f32)
{

  // Pause timer to update values
  TIMER1_DOWN;

  ctl_set_single_motor_speed(&motl,
                             speed_left_f32);
  ctl_set_single_motor_speed(&motr,
                             speed_right_f32);

  TIMER1_UP;
}

bool ctl_set_single_motor_speed(volatile MotorPulse *mot,
                                float speed_cmd_f32)
{
  float abs_speed_f32 = 0.0f;
  float pace_f32 = 0.0f;
  float pol_ticks_f32 = 0.0f;
  volatile uint32_t next_compare_val_ui32;
  volatile uint32_t num_ovf_tgt_ui32;
  bool mot_stopped_b;

  if (speed_cmd_f32 > speed_min_pos_f32)
  {
    mot->dir_cur_i32 = mot->dir_sign_i32;
    mot_stopped_b = false;
  }
  else if (speed_cmd_f32 < speed_min_neg_f32)
  {
    mot->dir_cur_i32 = -mot->dir_sign_i32;
    mot_stopped_b = false;
  }
  else
  {
    mot->dir_cur_i32 = 0;
    mot_stopped_b = true;
    mot->pol_ticks_cur_ui32 = UINT32_MAX;
    PORTB &= mot->step_clrandmask_cst_ui8;
  }
  if (!mot_stopped_b)
  {
    // Set direction pin value
    if (mot->dir_cur_i32 > 0)
    {
      PORTD |= mot->dir_setormask_cst_ui8;
    }
    else
    {
      PORTD &= mot->dir_clrandmask_cst_ui8;
    }
    // Limit to maximum speed
    abs_speed_f32 = fmin(fabs(speed_cmd_f32), speed_max_f32);

    // Compute pace (inverse of speed) and corresponding
    // number of ticks per pulse polarity
    pace_f32 = 1.0f / abs_speed_f32;
    pol_ticks_f32 = pace_f32 * pace2ticksperpol_f32;

    mot->pol_ticks_tgt_ui32 = (uint32_t)pol_ticks_f32;
    if (mot->pol_ticks_cur_ui32 >= mot->pol_ticks_tgt_ui32)
    {
      // Pulse polarity change due now
      ctl_toggle_motor_pin(mot);
      mot->pol_ticks_cur_ui32 = 0U;
    }

    mot->timer_val_prev_ui16 = TCNT1;

    next_compare_val_ui32 = (uint32_t)mot->timer_val_prev_ui16;
    next_compare_val_ui32 += (mot->pol_ticks_tgt_ui32 -
                              mot->pol_ticks_cur_ui32);
    num_ovf_tgt_ui32 = next_compare_val_ui32 >> 16;
    mot->cmp_val_cur_ui16 = (uint16_t)(next_compare_val_ui32 -
                                       (num_ovf_tgt_ui32 << 16));
    _SFR_MEM16(mot->compare_register_ui8) = mot->cmp_val_cur_ui16;

#if 0
    Serial.print("Speed update:\n");
    Serial.print("  abs_speed_f32       ");
    Serial.println(abs_speed_f32);
    Serial.print("  pol ticks tgt (f32) ");
    Serial.println(pol_ticks_f32);
    Serial.print("  pol ticks tgt       ");
    Serial.println(mot->pol_ticks_tgt_ui32);
    Serial.print("  pol ticks cur       ");
    Serial.println(mot->pol_ticks_cur_ui32);
    Serial.print("  current timer value ");
    Serial.println(mot->timer_val_prev_ui16);
    Serial.print("  next compare value  ");
    Serial.println(next_compare_val_ui32);
    Serial.print("  num overflows       ");
    Serial.println(num_ovf_tgt_ui32);
    Serial.print("  compare register    ");
    Serial.println(_SFR_MEM16(mot->compare_register_ui8));
    Serial.println(" ");
#endif
  }
  return mot_stopped_b;
}

void ctl_reset_motor_pos()
{
  motl.pulse_count_cur_i32 = 0;
  motr.pulse_count_cur_i32 = 0;
}

void ctl_get_motor_num_rev(float &motl_num_rev_f32,
                           float &motr_num_rev_f32)
{

  motl_num_rev_f32 = (float)(motl.pulse_count_cur_i32 * motl.dir_sign_i32) * rev_per_pulse_f32;
  motr_num_rev_f32 = (float)(motr.pulse_count_cur_i32 * motr.dir_sign_i32) * rev_per_pulse_f32;
}

void ctl_motor_interrupt(volatile MotorPulse *mot)
{
  uint32_t next_compare_val_ui32;
  uint32_t num_ovf_tgt_ui32;

  if (mot->dir_cur_i32 != 0)
  {
    // Compute time since last period check
    if (mot->timer_val_prev_ui16 >= mot->cmp_val_cur_ui16)
    {
      mot->pol_ticks_cur_ui32 += ((uint32_t)UINT16_MAX -
                                  (uint32_t)mot->timer_val_prev_ui16) +
                                 ((uint32_t)mot->cmp_val_cur_ui16 + 1U);
    }
    else
    {
      mot->pol_ticks_cur_ui32 += (uint32_t)mot->cmp_val_cur_ui16 -
                                 (uint32_t)mot->timer_val_prev_ui16;
    }
    mot->timer_val_prev_ui16 = mot->cmp_val_cur_ui16;

    // Check if desired period has elapsed and motor is running
    if (mot->pol_ticks_cur_ui32 >= mot->pol_ticks_tgt_ui32)
    {
      // Pulse polarity change due now
      ctl_toggle_motor_pin(mot);
    }
    next_compare_val_ui32 = (uint32_t)mot->timer_val_prev_ui16;
    next_compare_val_ui32 += (mot->pol_ticks_tgt_ui32 -
                              mot->pol_ticks_cur_ui32);
    num_ovf_tgt_ui32 = next_compare_val_ui32 >> 16;
    mot->cmp_val_cur_ui16 = (uint16_t)(next_compare_val_ui32 -
                                       (num_ovf_tgt_ui32 << 16));
    _SFR_MEM16(mot->compare_register_ui8) = mot->cmp_val_cur_ui16;
  }
  else
  {
    PORTB &= mot->step_clrandmask_cst_ui8;
  }
}

void ctl_toggle_motor_pin(volatile MotorPulse *mot)
{
  mot->pol_ticks_cur_ui32 = 0U;
  mot->pulse_pol_cur_b = !mot->pulse_pol_cur_b;
  if (mot->pulse_pol_cur_b)
  {
    PORTB |= mot->step_setormask_cst_ui8;
  }
  else
  {
    PORTB &= mot->step_clrandmask_cst_ui8;
  }
  // Update the pulse count on low to high transition
  if (mot->pulse_pol_cur_b)
  {
    mot->pulse_count_cur_i32 += mot->dir_cur_i32;
  }
}

// Interrupt service run when Timer/Counter1 reaches OCR1A
ISR(TIMER1_COMPA_vect)
{
  ctl_motor_interrupt(&motl);
}

// Interrupt service run when Timer/Counter1 reaches OCR1B
ISR(TIMER1_COMPB_vect)
{
  ctl_motor_interrupt(&motr);
}