/** STAN THE STANDING ROBOT
   Program to test the control function
   by David Beaudette   
**/

// Libraries
#include "Arduino.h"
#include "unity.h"
#include "ctl.h"
#include "bat.h"

#define NUM_SPEED_VALUES 4

// Speed values to validate
float motl_speed_list[NUM_SPEED_VALUES] = { 40.0f,  0.45f,  -0.256f, -0.064f};
float motr_speed_list[NUM_SPEED_VALUES] = {-40.0f, -0.256f,  0.45f,   0.064f};

// Expected number of revolutions in 1 second
float motl_expected_rev_list[NUM_SPEED_VALUES] = {
  5.2294E+0f,
  71.6086E-3f,
 -0.04125f,
 -0.01f,
};
float motr_expected_rev_list[NUM_SPEED_VALUES] = {
 -5.2294E+0f,
 -0.040625f,
  71.6086E-3f,
  0.010625f,
};

void test_single_motor_speed(int speed_idx) {

  float num_rev_left, num_rev_right;

  // Exceed the maximum speed
  ctl_reset_motor_pos();
  ctl_set_motor_speed(motl_speed_list[speed_idx],
                      motr_speed_list[speed_idx]);
  delay(1000);

  ctl_get_motor_num_rev(num_rev_left, 
                        num_rev_right);
  ctl_set_motor_speed(0.0, 0.0f);
  delay(500);

  Serial.print("Left motor is stopped (currently = ");
  Serial.print(num_rev_left);
  Serial.println(" turns).");
  TEST_ASSERT_FLOAT_WITHIN(motl_expected_rev_list[speed_idx] * 0.01f, 
                           motl_expected_rev_list[speed_idx], num_rev_left);

  Serial.print("Right motor is stopped (currently = ");
  Serial.print(num_rev_right);
  Serial.println(" turns).");
  Serial.println(" ");
  TEST_ASSERT_FLOAT_WITHIN(motr_expected_rev_list[speed_idx] * 0.01f, 
                           motr_expected_rev_list[speed_idx], num_rev_right);
}

void test_all_motor_speed(void) {
  for(int i = 0; i < NUM_SPEED_VALUES; ++i) {
    test_single_motor_speed(i);
  }
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

  RUN_TEST(test_all_motor_speed);

  UNITY_END();

}

void loop() {
  // Test finished; blink
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
}

