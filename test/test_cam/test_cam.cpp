/** STAN THE STANDING ROBOT
   Program to test the control function
   by David Beaudette   
**/

// Libraries
#include "Arduino.h"
#include "unity.h"
#include "cam.h"

void test_pan_ramp(void) {
  float pan_ang_cmd_f32 = -90.0f;
  float pan_ang_clc_f32;

  for(int i = 0; i < 90; ++i) {
    pan_ang_cmd_f32 += 2.0f;
    pan_ang_clc_f32 = cam_set_pan(pan_ang_cmd_f32);

    /*
    Serial.print("Setting pan angle to ");
    Serial.print(pan_ang_cmd_f32, 2);
    Serial.print(", clc = ");
    Serial.print(pan_ang_clc_f32, 2);
    Serial.println(" degrees.");
    */
    delay(200);
  }
  cam_set_pan(0.0f);
  TEST_ASSERT_TRUE(true);
}

void test_tilt_ramp(void) {
  float tilt_ang_cmd_f32 = -90.0f;
  float tilt_ang_clc_f32;

  for(int i = 0; i < 90; ++i) {
    tilt_ang_cmd_f32 += 2.0f;
    tilt_ang_clc_f32 = cam_set_tilt(tilt_ang_cmd_f32);
    /*
    Serial.print("Setting tilt angle to ");
    Serial.print(tilt_ang_cmd_f32, 2);
    Serial.print(", clc = ");
    Serial.print(tilt_ang_clc_f32, 2);
    Serial.println(" degrees.");
    */
    delay(200);
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

  RUN_TEST(test_pan_ramp);
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

