/** STAN THE STANDING ROBOT
   Program to test the control function
   by David Beaudette   
**/

// Libraries
#include "Arduino.h"
#include "unity.h"
#include "ctl.h"
#include "bat.h"


void test_motor_speed(void) {

  // Exceed the maximum speed
  ctl_reset_motor_pos();
  ctl_set_motor_speed(40.0f, 0.0f);
  delay(1000);

  ctl_set_motor_speed(0.0, 0.0f);
  delay(500);

  // Speed below 1 counter overflow
  ctl_reset_motor_pos();
  ctl_set_motor_speed(0.45f, 0.0f);
  delay(1000);

  ctl_set_motor_speed(0.0, 0.0f);
  delay(500);

  // Speed with exactly 1 counter overflow
  ctl_reset_motor_pos();
  ctl_set_motor_speed(0.256f, 0.0f);
  delay(1000);

  ctl_set_motor_speed(0.0, 0.0f);
  delay(500);

  // Speed with several counter overflows
  ctl_reset_motor_pos();
  ctl_set_motor_speed(0.064f, 0.0f);
  delay(1000);

  // Below minimum speed
  ctl_set_motor_speed(0.015f, 0.0f);
  delay(500);
  
  TEST_ASSERT_TRUE(true);
}

void test_bat_level(void) {
  TEST_ASSERT_TRUE(check_bat_level_verbose());
}

void setup() {

  ctl_init();

  pinMode(LED_BUILTIN, OUTPUT);

  // Time is needed to establish a Serial connection
  // between a host machine and a target device
  delay(2000);

  // Initialize serial communication
  UNITY_BEGIN();

  RUN_TEST(test_bat_level);
  RUN_TEST(test_motor_speed);

  UNITY_END();

}

void loop() {
  // Test finished; blink
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
}

