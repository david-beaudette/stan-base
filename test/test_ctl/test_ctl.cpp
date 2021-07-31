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
  5.23f,
  0.071875f,
 -0.040625f,
 -0.010625f,
};
float motr_expected_rev_list[NUM_SPEED_VALUES] = {
 -5.23f,
 -0.040625f,
  0.071875f,
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
  ctl_set_motor_speed(0.0f, 0.0f);
  delay(500);

  Serial.print("Number of turns after test:\n  L = ");
  Serial.println(num_rev_left, 6);
  Serial.print("  R = ");
  Serial.println(num_rev_right, 6);
  Serial.println("  ");

  TEST_ASSERT_FLOAT_WITHIN(motl_expected_rev_list[speed_idx] * 0.05f, 
                           motl_expected_rev_list[speed_idx], num_rev_left);
  TEST_ASSERT_FLOAT_WITHIN(motr_expected_rev_list[speed_idx] * 0.05, 
                           motr_expected_rev_list[speed_idx], num_rev_right);
}

void test_all_motor_speed(void) {
  for(int i = 0; i < NUM_SPEED_VALUES; ++i) {
    test_single_motor_speed(i);
  }
}

void test_motor_ramp(void) {
  float speed_cur_f32 = 0.0f;
  float num_rev_left, num_rev_right;
  float expected_num_rev_f32 = 12.01f;

  ctl_reset_motor_pos();
  
  for(int i = 0; i < 100; ++i) {
    speed_cur_f32 += (float)i * 0.01f;
    ctl_set_motor_speed(speed_cur_f32, speed_cur_f32);
    delay(50);
  }
  ctl_set_motor_speed(0.0f, 0.0f);
  ctl_get_motor_num_rev(num_rev_left, 
                        num_rev_right);

  TEST_ASSERT_FLOAT_WITHIN(expected_num_rev_f32 * 0.01f, 
                           expected_num_rev_f32, num_rev_left);
  TEST_ASSERT_FLOAT_WITHIN(expected_num_rev_f32 * 0.01f, 
                           expected_num_rev_f32, num_rev_right);
}

void test_bat_level(void) {
  TEST_ASSERT_TRUE(bat_check_level());
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

  RUN_TEST(test_motor_ramp);

  UNITY_END();

}

void loop() {
  // Test finished; blink
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
}

