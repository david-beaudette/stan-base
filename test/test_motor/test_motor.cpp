/** STAN THE STANDING ROBOT
   Program to test the highest speed achievable with servos
   by David Beaudette   
**/

// Libraries
#include "Arduino.h"
#include "unity.h"
#include "ctl.h"

#define NUM_DELAYS 3
#define BAT_PIN A6 
#define BAT_LIM 867 // 25% of 3S LiPo -> 11.25 VDC 

unsigned int delays[NUM_DELAYS] = {
  100U,
  75U,
  50U,
};

unsigned long spin_duration = 1000000U;
unsigned int wait_duration = 100000U;

int dly_idx;

void test_motor_speed(void) {

  unsigned long t_ini, t_cur;

  digitalWrite(MOT_NSLEEP, HIGH);

  t_ini = micros();
  t_cur = t_ini;
  while((t_cur - t_ini) <= spin_duration) {
    digitalWrite(MOTL_STEP, HIGH);
    digitalWrite(MOTR_STEP, HIGH);
    delayMicroseconds(delays[dly_idx]);
    digitalWrite(MOTL_STEP, LOW);
    digitalWrite(MOTR_STEP, LOW);
    delayMicroseconds(delays[dly_idx]);
    t_cur = micros();
  }
  // Dummy check for test to pass
  TEST_ASSERT_TRUE((t_cur - t_ini) > spin_duration);

  digitalWrite(MOT_NSLEEP, LOW);
}

void test_bat_level(void) {
  TEST_ASSERT_TRUE(analogRead(BAT_PIN) > BAT_LIM);
}

void setup() {

  pinMode(LED_BUILTIN, OUTPUT);

  // Sleep during setup
  pinMode(MOT_NSLEEP, OUTPUT);
  digitalWrite(MOT_NSLEEP, LOW);

  // Time is needed to establish a Serial connection
  // between a host machine and a target device
  delay(2000);

  // Initialize serial communication
  UNITY_BEGIN();

  pinMode(MOTL_DIR, OUTPUT);
  pinMode(MOTR_DIR, OUTPUT);
  pinMode(MOTL_STEP, OUTPUT);
  pinMode(MOTR_STEP, OUTPUT);

  digitalWrite(MOTL_DIR, LOW);
  digitalWrite(MOTR_DIR, HIGH);
  digitalWrite(MOTL_STEP, LOW);
  digitalWrite(MOTR_STEP, LOW);

  RUN_TEST(test_bat_level);
  for(dly_idx = 0; dly_idx < NUM_DELAYS; ++dly_idx) {
    RUN_TEST(test_motor_speed);
    delayMicroseconds(wait_duration);
  }

  UNITY_END();

}

void loop() {
  // Test finished; blink
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
}

