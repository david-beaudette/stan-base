/** STAN THE STANDING ROBOT
   Program to test the camera pan tilt servo control function
   by David Beaudette   
**/

// Libraries
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include <stdio.h>
#include "cam.h"

int main() {

  const uint LED_PIN = PICO_DEFAULT_LED_PIN;
  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);

  stdio_init_all();
     
  sleep_ms(2400);
  printf("starting...\r\n");

  PWM myServo = enableServo(CAM_TILT_PIN);
  myServo.pulseMax=2400;
  myServo.pulseMin=700;
  myServo.delay = 2000;
  for(int i = 0 ; i < 180;i++){
      setServo(&myServo,i);
      sleep_ms(10);
  }
  for(int i = 180 ; i > 0;i--){
      setServo(&myServo,i);
      sleep_ms(10);
  }
  setServo(&myServo,0.0);
  sleep_ms(500);
  setServo(&myServo,45.0);
  sleep_ms(500);
  setServo(&myServo,90.0);
  sleep_ms(500);
  setServo(&myServo,135.0);
  sleep_ms(500);
  setServo(&myServo,180.0);
  sleep_ms(500);
  setServo(&myServo,135.0);
  sleep_ms(500);
  setServo(&myServo,90.0);
  sleep_ms(500);
  setServo(&myServo,45.0);
  sleep_ms(500);
  setServo(&myServo,0.0);
  disableServo(myServo);
  printf("done\r\n");

  while (true) {
    gpio_put(LED_PIN, 1);
    sleep_ms(1500);
    gpio_put(LED_PIN, 0);
    sleep_ms(1500);
    printf("Hello, world!\n");
  } 

}
#if 0
void test_pan_limits(void) {
  float pan_ang_cmd_f32;
  float pan_ang_clc_f32;

  for(int i = -1; i <= 1; ++i) {
    pan_ang_cmd_f32 = (float)i * 90.0f;
    pan_ang_clc_f32 = cam_set_pan(pan_ang_cmd_f32);

    
    Serial.print("Setting pan angle to ");
    Serial.print(pan_ang_cmd_f32, 2);
    Serial.print(", clc = ");
    Serial.print(pan_ang_clc_f32, 2);
    Serial.println(" degrees.");
    
    delay(2000);
  }
  cam_set_pan(0.0f);
  TEST_ASSERT_TRUE(true);
}

void test_tilt_limits(void) {
  float tilt_ang_cmd_f32;
  float tilt_ang_clc_f32;

  for(int i = -1; i <= 1; ++i) {
    tilt_ang_cmd_f32 = (float)i * 90.0f;
    tilt_ang_clc_f32 = cam_set_tilt(tilt_ang_cmd_f32);

    /*
    Serial.print("Setting tilt angle to ");
    Serial.print(tilt_ang_cmd_f32, 2);
    Serial.print(", clc = ");
    Serial.print(tilt_ang_clc_f32, 2);
    Serial.println(" degrees.");
    */
    delay(2000);
  }
  cam_set_tilt(0.0f);
  TEST_ASSERT_TRUE(true);
}

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
    
    Serial.print("Setting tilt angle to ");
    Serial.print(tilt_ang_cmd_f32, 2);
    Serial.print(", clc = ");
    Serial.print(tilt_ang_clc_f32, 2);
    Serial.println(" degrees.");
    
    delay(200);
  }
  cam_set_tilt(0.0f);
  TEST_ASSERT_TRUE(true);
}

void setup() {

  cam_init();

  //pinMode(LED_BUILTIN, OUTPUT);

  // Time is needed to establish a Serial connection
  // between a host machine and a target device
  delay(2000);

  // Initialize serial communication
  UNITY_BEGIN();

  RUN_TEST(test_pan_limits);
  RUN_TEST(test_tilt_limits);
  RUN_TEST(test_pan_ramp);
  RUN_TEST(test_tilt_ramp);

  UNITY_END();

}
#endif