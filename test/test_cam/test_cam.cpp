/** STAN THE STANDING ROBOT
   Program to test the control function
   by David Beaudette   
**/

// Libraries
#include "Arduino.h"
#include "unity.h"
#include "cam.h"

void test_tilt_ramp(void) {
  float tilt_ang_cur_f32 = -90.0f;
  float num_rev_left, num_rev_right;

  for(int i = 0; i < 181; ++i) {
    tilt_ang_cur_f32 += 1.0f;
    cam_set_tilt(tilt_ang_cur_f32);

    Serial.print("Setting tilt angle to ");
    Serial.print(tilt_ang_cur_f32, 2);
    Serial.println(" degrees.");

    delay(500);
  }
  cam_set_tilt(0.0f);
  TEST_ASSERT_TRUE(true);
}

void setup() {

  cam_init();

  pinMode(LED_BUILTIN, OUTPUT);

  // Time is needed to establish a Serial connection
  // between a host machine and a target device
  delay(2000);

  // Initialize serial communication
  UNITY_BEGIN();

  RUN_TEST(test_tilt_ramp);

  UNITY_END();

}

void loop() {
  // Test finished; blink
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
}

